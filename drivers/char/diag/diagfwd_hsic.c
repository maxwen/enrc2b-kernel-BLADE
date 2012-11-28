/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/diagchar.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/ratelimit.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <asm/current.h>
#include <linux/delay.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#endif
#include "diagchar_hdlc.h"
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diagfwd_hsic.h"
#include <mach/board_htc.h>

static int is_hsic_poll_full;

static void diag_read_hsic_work_fn(struct work_struct *work)
{
	static int error_count = 0,timer_count = 0;
	unsigned long diff;

	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return;
	}

	/* If there is no hsic data being read from the hsic and there
	 * is no hsic data being written to the device
	 */
	if (!driver->in_busy_hsic_read &&
			 !driver->in_busy_hsic_write_on_device) {
		if (is_hsic_poll_full && driver->logging_mode == MEMORY_DEVICE_MODE) {
			do_gettimeofday(&driver->st1);
			diff = (driver->st1.tv_sec-driver->st0.tv_sec)*1000 + (driver->st1.tv_usec-driver->st0.tv_usec)/1000;
			if (diff) {
				pr_debug("[diag-dbg] Over time (%ld) %ld.%04ld -> %ld.%04ld\n", diff,
						(long)driver->st0.tv_sec, driver->st0.tv_usec/1000,
						(long)driver->st1.tv_sec, driver->st1.tv_usec/1000);
			}
			is_hsic_poll_full = 0;
		}
		/*
		 * Initiate the read from the hsic.  The hsic read is
		 * asynchronous.  Once the read is complete the read
		 * callback function will be called.
		 */
		int err;
		driver->in_busy_hsic_read = 1;
		APPEND_DEBUG('i');
		pr_debug("diag: read from HSIC\n");
		err = diag_bridge_read((char *)driver->buf_in_hsic,
					IN_BUF_SIZE);
		if (err) {
			pr_err_ratelimited("DIAG: Error initiating HSIC read, err: %d\n",
				err);

			/* some cases that hsic status is not recoverable but we can't identify by error code.
			     setup a condition to stop this error try. */

			if (timer_count > 50) {
				pr_err("DIAG: slow error try over 50 times. stop diag_read_hsic_work\n");
				err = -ESHUTDOWN;
				error_count = 0;
				timer_count = 0;
			} else if (error_count > 50) {
				pr_err("DIAG: error try over 50 times, slow down retry speed\n");
				msleep(1000);
				timer_count++;
			}  else {
				error_count++;
			}

			/*
			 * If the error is recoverable, then clear
			 * the read flag, so we will resubmit a
			 * read on the next frame.  Otherwise, don't
			 * resubmit a read on the next frame.
			 */
			if ((-ENODEV) != err)
				driver->in_busy_hsic_read = 0;
		} else {
			error_count = 0;
			timer_count = 0;
		}
	}

	/*
	 * If for some reason there was no hsic data, set up
	 * the next read
	 */
	if (!driver->in_busy_hsic_read)
		queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);
}

static void diag_hsic_read_complete_callback(void *ctxt, char *buf,
					int buf_size, int actual_size)
{
#if  DIAG_XPST
	int type;
	static int pkt_hdr, first_pkt = 1;
#endif
	int offset;
	/* The read of the data from the HSIC bridge is complete */
	driver->in_busy_hsic_read = 0;

	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return;
	}

	APPEND_DEBUG('j');
	if (actual_size > 0) {
		if (!buf) {
			pr_err("Out of diagmem for HSIC\n");
		} else {
			if (diag9k_debug_mask) {
				print_hex_dump(KERN_DEBUG, "Read Packet Data"
					" from modem(first 16 bytes)", 16, 1,
					DUMP_PREFIX_ADDRESS, buf, 16, 1);
			}
#if DIAG_XPST
			/* HTC: only route to user space if the packet smd received
			 * is the head of the full packet to avoid route wrong packet
			 * to userspace. BTW, to avoid lost 1st packet (do not know if
			 * the head of packet), we always check 1st packet. It should
			 * be the 0xc sync packet.
			 */
			if (pkt_hdr || (first_pkt == 1)) {
				if (unlikely(first_pkt == 1))
					first_pkt = 0;
				type = checkcmd_modem_epst(buf);
				if (type) {
					modem_to_userspace(buf, actual_size, type, 1);
					pkt_hdr = 1;
					return;
				}
				pkt_hdr = 0;
			}

			if ((actual_size == 1 && *buf == CONTROL_CHAR) ||
				(*(buf+actual_size-1) == CONTROL_CHAR &&
				*(buf+actual_size-2) != ESC_CHAR))
				pkt_hdr = 1;
#endif
			if (driver->logging_mode == USB_MODE) {
				driver->write_ptr_mdm->length = actual_size;
				/*
				 * Set flag to denote hsic data is currently
				 * being written to the usb mdm channel.
				 * driver->buf_in_hsic was given to
				 * diag_bridge_read(), so buf here should be
				 * driver->buf_in_hsic
				 */
				driver->in_busy_hsic_write_on_device = 1;
			} else {
				offset = driver->write_ptr_mdm->length;
				driver->write_ptr_mdm->length += actual_size;

				if (offset + actual_size > IN_POLL_BUF_SIZE) {
					pr_warning("[diag-dbg] Overflow"
						" offset:%d actual_size:%d"
						" in_busy:%d length:%d\n",
						offset, actual_size,
						driver->in_busy_hsic_write_on_device,
						driver->write_ptr_mdm->length);
					return;
				} else {
					memcpy(driver->buf_poll_in_hsic + offset, buf, actual_size);
					driver->in_busy_hsic_poll_write_on_device = 1;
				}

				if (driver->write_ptr_mdm->length >= (IN_POLL_BUF_SIZE - DIAG_PKT_SIZE)) {
					/*
					 * Set flag to denote hsic data is currently
					 * being written to the usb mdm channel.
					 * driver->buf_in_hsic was given to
					 * diag_bridge_read(), so buf here should be
					 * driver->buf_in_hsic
					 */
					driver->in_busy_hsic_write_on_device = 1;
					is_hsic_poll_full = 1;
					do_gettimeofday(&driver->st0);
				}
			}
			pr_debug("diag: write to device\n");
			diag_device_write((void *)buf, HSIC_DATA,
						driver->write_ptr_mdm);
		}
	} else {
		pr_debug("%s: actual_size: %d\n", __func__, actual_size);
	}

	/*
	 * If for some reason there was no hsic data to write to the
	 * mdm channel, set up another read
	 */
	if (!driver->in_busy_hsic_write_on_device && ((driver->logging_mode
			== MEMORY_DEVICE_MODE) || (driver->usb_mdm_connected &&
						    !driver->hsic_suspend)))
		queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);
}

static void diag_hsic_write_complete_callback(void *ctxt, char *buf,
					int buf_size, int actual_size)
{
	/* The write of the data to the HSIC bridge is complete */
	driver->in_busy_hsic_write = 0;

	if (driver->in_busy_hsic_write_wait) {
		driver->in_busy_hsic_write_wait = 0;
		wake_up_interruptible(&driver->wait_q);
	}

	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return;
	}

	if (actual_size < 0)
		pr_err("DIAG in %s: actual_size: %d\n", __func__, actual_size);

	if (driver->usb_mdm_connected)
		queue_work(driver->diag_hsic_wq, &driver->diag_read_mdm_work);
}

static int diag_hsic_suspend(void *ctxt)
{
	//htc_dbg
	if (get_radio_flag() & 0x0001) {
		pr_info("diag: hsic_suspend\n");
		pr_info("mode(%d):busy to mdm:%d, busy to device:%d\n",
			driver->logging_mode,
			driver->in_busy_hsic_write,
			driver->in_busy_hsic_write_on_device);
	}
	if (driver->in_busy_hsic_write)
		return -EBUSY;
#if 0
	/* Don't allow suspend if in MEMORY_DEVICE_MODE */
	if (driver->logging_mode == MEMORY_DEVICE_MODE)
		return -EBUSY;
#endif
	driver->hsic_suspend = 1;

	return 0;
}

static void diag_hsic_resume(void *ctxt)
{
	//htc_dbg
	if (get_radio_flag() & 0x0001)
		pr_info("diag: hsic_resume\n");
	driver->hsic_suspend = 0;

	if (!driver->in_busy_hsic_write_on_device && (driver->logging_mode
			== MEMORY_DEVICE_MODE || driver->usb_mdm_connected))
		queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);
}

static struct diag_bridge_ops hsic_diag_bridge_ops = {
	.ctxt = NULL,
	.read_complete_cb = diag_hsic_read_complete_callback,
	.write_complete_cb = diag_hsic_write_complete_callback,
	.suspend = diag_hsic_suspend,
	.resume = diag_hsic_resume,
};

static int diag_hsic_close(void)
{
	if (driver->hsic_device_enabled) {
		driver->hsic_ch = 0;
		if (driver->hsic_device_opened) {
			driver->hsic_device_opened = 0;
			diag_bridge_close();
		}
		pr_debug("diag: in %s: closed successfully\n", __func__);
	} else {
		pr_debug("diag: in %s: already closed\n", __func__);
	}

	return 0;
}

/* diagfwd_cancel_hsic is called to cancel outstanding read/writes */
int diagfwd_cancel_hsic(void)
{
	int err;

	if (driver->hsic_device_enabled) {
		if (driver->hsic_device_opened) {
			driver->hsic_ch = 0;
			driver->hsic_device_opened = 0;
			diag_bridge_close();
			err = diag_bridge_open(&hsic_diag_bridge_ops);
			if (err) {
				pr_err("DIAG: HSIC channel open error: %d\n",
						err);
			} else {
				pr_debug("DIAG: opened HSIC channel\n");
				driver->hsic_device_opened = 1;
				driver->hsic_ch = 1;
			}
		}
	}

	return 0;
}

/* diagfwd_connect_hsic is called when the USB mdm channel is connected */
int diagfwd_connect_hsic(int process_cable)
{
	int err;

	DIAGFWD_INFO("DIAG in %s\n", __func__);

	/* If the usb cable is being connected */
	if (process_cable) {
		err = usb_diag_alloc_req(driver->mdm_ch, N_MDM_WRITE,
			N_MDM_READ);
		if (err)
			pr_err("DIAG: unable to alloc USB req on mdm"
				" ch err:%d\n", err);

		driver->usb_mdm_connected = 1;
	}

	driver->in_busy_hsic_write_on_device = 0;
	driver->in_busy_hsic_read_on_device = 0;
	driver->in_busy_hsic_write = 0;
	driver->in_busy_hsic_read = 0;

	/* If the hsic (diag_bridge) platform device is not open */
	if (driver->hsic_device_enabled) {
		if (!driver->hsic_device_opened) {
			err = diag_bridge_open(&hsic_diag_bridge_ops);
			if (err) {
				pr_err("DIAG: HSIC channel open error: %d\n",
					err);
			} else {
				pr_debug("DIAG: opened HSIC channel\n");
				driver->hsic_device_opened = 1;
			}
		} else {
			pr_debug("DIAG: HSIC channel already open\n");
		}

		/*
		 * Turn on communication over usb mdm and hsic, if the hsic
		 * device driver is enabled and opened
		 */
		if (driver->hsic_device_opened)
			driver->hsic_ch = 1;

		/* Poll USB mdm channel to check for data */
		if (driver->logging_mode == USB_MODE)
			queue_work(driver->diag_hsic_wq,
					&driver->diag_read_mdm_work);

		/* Poll HSIC channel to check for data */
		queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);
	} else {
		/* The hsic device driver has not yet been enabled */
		pr_info("DIAG: HSIC channel not yet enabled\n");
	}

	return 0;
}

/*
 * diagfwd_disconnect_hsic is called when the USB mdm channel
 * is disconnected
 */
int diagfwd_disconnect_hsic(int process_cable)
{
	DIAGFWD_INFO("DIAG in %s\n", __func__);

	/* If the usb cable is being disconnected */
	if (process_cable) {
		driver->usb_mdm_connected = 0;
		usb_diag_free_req(driver->mdm_ch);
	}

	/* keep the connection even cable out
	 * to not disconnect epst communication
	 */
#if 0
	if (driver->logging_mode != MEMORY_DEVICE_MODE) {
		driver->in_busy_hsic_write_on_device = 1;
		driver->in_busy_hsic_read_on_device = 1;
		driver->in_busy_hsic_write = 1;
		driver->in_busy_hsic_read = 1;
		/* Turn off communication over usb mdm and hsic */
		return diag_hsic_close();
	}
#endif
	return 0;
}

/*
 * diagfwd_write_complete_hsic is called after the asynchronous
 * usb_diag_write() on mdm channel is complete
 */
int diagfwd_write_complete_hsic(void)
{
	/*
	 * Clear flag to denote that the write of the hsic data on the
	 * usb mdm channel is complete
	 */
	driver->in_busy_hsic_write_on_device = 0;
	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return 0;
	}

	APPEND_DEBUG('q');

	/* Read data from the hsic */
	queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);

	return 0;
}

/* Called after the asychronous usb_diag_read() on mdm channel is complete */
static int diagfwd_read_complete_hsic(struct diag_request *diag_read_ptr)
{
	static int error_count = 0,timer_count = 0;
	/* The read of the usb driver on the mdm (not hsic) has completed */
	driver->in_busy_hsic_read_on_device = 0;
	driver->read_len_mdm = diag_read_ptr->actual;

	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return 0;
	}

	/*
	 * The read of the usb driver on the mdm channel has completed.
	 * If there is no write on the hsic in progress, check if the
	 * read has data to pass on to the hsic. If so, pass the usb
	 * mdm data on to the hsic.
	 */
#if DIAG_XPST
	if (!driver->in_busy_hsic_write && driver->usb_buf_mdm_out &&
		(driver->read_len_mdm > 0) && !driver->nohdlc) {
#else
	if (!driver->in_busy_hsic_write && driver->usb_buf_mdm_out &&
		(driver->read_len_mdm > 0)) {
#endif

		/*
		 * Initiate the hsic write. The hsic write is
		 * asynchronous. When complete the write
		 * complete callback function will be called
		 */
		int err;
		driver->in_busy_hsic_write = 1;
		err = diag_bridge_write(driver->usb_buf_mdm_out,
					driver->read_len_mdm);
		if (err) {
			pr_err_ratelimited("DIAG: mdm data on hsic write err: %d\n",
					err);

			if (timer_count > 50) {
				pr_err("DIAG: slow error try over 50 times. stop diag_read_hsic_work\n");
				err = -ESHUTDOWN;
				error_count = 0;
				timer_count = 0;
			} else if (error_count > 50) {
				pr_err("DIAG: error try over 50 times, slow down retry speed\n");
				msleep(1000);
				timer_count++;
			}  else {
				error_count++;
			}

			/*
			 * If the error is recoverable, then clear
			 * the write flag, so we will resubmit a
			 * write on the next frame.  Otherwise, don't
			 * resubmit a write on the next frame.
			 */
			if ((-ENODEV) != err)
				driver->in_busy_hsic_write = 0;
		} else {
			error_count = 0;
			timer_count = 0;
		}
	}

	/*
	 * If there is no write of the usb mdm data on the
	 * hsic channel
	 */
	if (!driver->in_busy_hsic_write)
		queue_work(driver->diag_hsic_wq, &driver->diag_read_mdm_work);

	return 0;
}

static void diagfwd_hsic_notifier(void *priv, unsigned event,
					struct diag_request *d_req)
{
	switch (event) {
	case USB_DIAG_CONNECT:
		diagfwd_connect_hsic(1);
		break;
	case USB_DIAG_DISCONNECT:
		queue_work(driver->diag_hsic_wq, &driver->diag_disconnect_work);
		break;
	case USB_DIAG_READ_DONE:
		queue_work(driver->diag_hsic_wq,
				&driver->diag_usb_read_complete_work);
		break;
	case USB_DIAG_WRITE_DONE:
		diagfwd_write_complete_hsic();
		break;
	default:
		pr_err("DIAG in %s: Unknown event from USB diag:%u\n",
			__func__, event);
		break;
	}
}

static void diag_usb_read_complete_fn(struct work_struct *w)
{
	diagfwd_read_complete_hsic(driver->usb_read_mdm_ptr);
}

static void diag_disconnect_work_fn(struct work_struct *w)
{
	diagfwd_disconnect_hsic(1);
}

static void diag_read_mdm_work_fn(struct work_struct *work)
{
	if (!driver->hsic_ch) {
		pr_err("DIAG in %s: driver->hsic_ch == 0\n", __func__);
		return;
	}

	/*
	 * If there is no data being read from the usb mdm channel
	 * and there is no mdm channel data currently being written
	 * to the hsic
	 */
	if (!driver->in_busy_hsic_read_on_device &&
				 !driver->in_busy_hsic_write) {
		APPEND_DEBUG('x');

		/* Setup the next read from usb mdm channel */
		driver->in_busy_hsic_read_on_device = 1;
		driver->usb_read_mdm_ptr->buf = driver->usb_buf_mdm_out;
		driver->usb_read_mdm_ptr->length = USB_MAX_OUT_BUF;
		usb_diag_read(driver->mdm_ch, driver->usb_read_mdm_ptr);
		APPEND_DEBUG('y');
	}

	/*
	 * If for some reason there was no mdm channel read initiated,
	 * queue up the reading of data from the mdm channel
	 */
	if (!driver->in_busy_hsic_read_on_device)
		queue_work(driver->diag_hsic_wq, &driver->diag_read_mdm_work);
}

static int diag_hsic_probe(struct platform_device *pdev)
{
	int err = 0;
	pr_debug("diag: in %s\n", __func__);
	if (!driver->hsic_device_enabled) {
		driver->read_len_mdm = 0;
		if (driver->buf_in_hsic == NULL)
			driver->buf_in_hsic = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_hsic == NULL)
			goto err;
		if (driver->buf_poll_in_hsic == NULL)
			driver->buf_poll_in_hsic = kzalloc(IN_POLL_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_poll_in_hsic == NULL)
			goto err;
		if (driver->buf_poll_to_user == NULL)
			driver->buf_poll_to_user = kzalloc(IN_POLL_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_poll_to_user == NULL)
			goto err;
		if (driver->usb_buf_mdm_out  == NULL)
			driver->usb_buf_mdm_out = kzalloc(USB_MAX_OUT_BUF,
					GFP_KERNEL);
		if (driver->usb_buf_mdm_out == NULL)
			goto err;
		if (driver->write_ptr_mdm == NULL)
			driver->write_ptr_mdm = kzalloc(
					sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_mdm == NULL)
			goto err;
		if (driver->usb_read_mdm_ptr == NULL)
			driver->usb_read_mdm_ptr = kzalloc(
					sizeof(struct diag_request), GFP_KERNEL);
		if (driver->usb_read_mdm_ptr == NULL)
			goto err;
#ifdef CONFIG_DIAG_OVER_USB
		INIT_WORK(&(driver->diag_read_mdm_work), diag_read_mdm_work_fn);
#endif
		INIT_WORK(&(driver->diag_read_hsic_work),
				diag_read_hsic_work_fn);
		driver->hsic_device_enabled = 1;
	}

	/* The hsic (diag_bridge) platform device driver is enabled */
	err = diag_bridge_open(&hsic_diag_bridge_ops);
	if (err) {
		pr_err("diag: could not open HSIC, err: %d\n", err);
		driver->hsic_device_opened = 0;
		return err;
	}

	pr_info("diag: opened HSIC channel\n");
	driver->hsic_device_opened = 1;
	driver->hsic_ch = 1;
	driver->in_busy_hsic_write_on_device = 0;
	driver->in_busy_hsic_read_on_device = 0;
	driver->in_busy_hsic_write = 0;
	driver->in_busy_hsic_read = 0;

	/*
	 * The probe function was called after the usb was connected
	 * on the legacy channel OR ODL is turned on. Communication over usb
	 * mdm and hsic needs to be turned on.
	 */
	if (driver->usb_mdm_connected || (driver->logging_mode ==
				MEMORY_DEVICE_MODE)) {
		if (driver->usb_mdm_connected) {
			/* Poll USB mdm channel to check for data */
			queue_work(driver->diag_hsic_wq,
					&driver->diag_read_mdm_work);
		}

		/* Poll HSIC channel to check for data */
		queue_work(driver->diag_hsic_wq, &driver->diag_read_hsic_work);
	}

	return err;
err:
	pr_err("DIAG could not initialize buf for HSIC\n");
	kfree(driver->buf_in_hsic);
	kfree(driver->usb_buf_mdm_out);
	kfree(driver->write_ptr_mdm);
	kfree(driver->usb_read_mdm_ptr);
	if (driver->diag_hsic_wq)
		destroy_workqueue(driver->diag_hsic_wq);

	return -ENOMEM;
}

static int diag_hsic_remove(struct platform_device *pdev)
{
	pr_debug("DIAG: %s called\n", __func__);
	diag_hsic_close();
	return 0;
}

static int diagfwd_hsic_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diagfwd_hsic_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diagfwd_hsic_dev_pm_ops = {
	.runtime_suspend = diagfwd_hsic_runtime_suspend,
	.runtime_resume = diagfwd_hsic_runtime_resume,
};

static struct platform_driver msm_hsic_ch_driver = {
	.probe = diag_hsic_probe,
	.remove = diag_hsic_remove,
	.driver = {
		   .name = "diag_bridge",
		   .owner = THIS_MODULE,
		   .pm   = &diagfwd_hsic_dev_pm_ops,
		   },
};

void diagfwd_hsic_init(void)
{
	int ret;

	DIAGFWD_INFO("DIAG in %s\n", __func__);

	driver->diag_hsic_wq = create_singlethread_workqueue("diag_hsic_wq");
	INIT_WORK(&(driver->diag_disconnect_work), diag_disconnect_work_fn);
	INIT_WORK(&(driver->diag_usb_read_complete_work),
			diag_usb_read_complete_fn);

#ifdef CONFIG_DIAG_OVER_USB
	driver->mdm_ch = usb_diag_open(DIAG_MDM, driver, diagfwd_hsic_notifier);
	if (IS_ERR(driver->mdm_ch)) {
		pr_err("DIAG Unable to open USB diag MDM channel\n");
		goto err;
	}
#endif
	ret = platform_driver_register(&msm_hsic_ch_driver);
	if (ret)
		pr_err("DIAG could not register HSIC device, ret: %d\n", ret);
	else
		driver->hsic_initialized = 1;

	return;
err:
	pr_err("DIAG could not initialize for HSIC execution\n");
}

void diagfwd_hsic_exit(void)
{
	pr_debug("DIAG in %s\n", __func__);

	if (driver->hsic_initialized)
		diag_hsic_close();

#ifdef CONFIG_DIAG_OVER_USB
	if (driver->usb_mdm_connected)
		usb_diag_free_req(driver->mdm_ch);
#endif
	platform_driver_unregister(&msm_hsic_ch_driver);
#ifdef CONFIG_DIAG_OVER_USB
	usb_diag_close(driver->mdm_ch);
#endif
	kfree(driver->buf_in_hsic);
	kfree(driver->usb_buf_mdm_out);
	kfree(driver->write_ptr_mdm);
	kfree(driver->usb_read_mdm_ptr);
	destroy_workqueue(driver->diag_hsic_wq);

	driver->hsic_device_enabled = 0;
}
