/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
//#include <linux/mfd/pmic8058.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/mdm2.h>
#include <mach/restart.h>
//#include <mach/subsystem_notif.h>
#include <mach/subsystem_restart.h>
#include <linux/msm_charm.h>
//#include "msm_watchdog.h"
#include <linux/interrupt.h>
#include "mdm_private.h"
#include "gpio-names.h"

//++SSD_RIL:20120724:For function get_radio_flag()
#include <mach/board_htc.h>
//--SSD_RIL

//++SSD_RIL
extern bool device_ehci_shutdown;
//--SSD_RIL


extern int open_hsicctl_timeout_trigger_errfatal;


/* HTC added start */
#if defined(pr_debug)
#undef pr_debug
#endif
#define pr_debug(x...) do {				\
			printk(KERN_DEBUG "[MDM][COMM] "x);		\
	} while (0)

#if defined(pr_info)
#undef pr_info
#endif
#define pr_info(x...) do {				\
			printk(KERN_INFO "[MDM][COMM] "x);		\
	} while (0)

#if defined(pr_err)
#undef pr_err
#endif
#define pr_err(x...) do {				\
			printk(KERN_ERR "[MDM][COMM] "x);		\
	} while (0)

static atomic_t final_efs_wait = ATOMIC_INIT(0);
//++HTC
bool mdm_in_fatal_handler = false;
bool mdm_is_alive = false;
//--HTC

//++HTC
static int mdm2ap_gpio_status = 0;
static DEFINE_MUTEX(MDM_BOOT_STATUS_CHECK_LOCK);
//--HTC

int mdm_common_check_final_efs_wait()
{
	return atomic_read(&final_efs_wait);
}

//++SSD_RIL: Mars_Lin@20120608: Add mdm_common_enable_irq for enable/disable mdm irq
void mdm_common_enable_irq( struct mdm_modem_drv *mdm_drv, int enable )
{
	int irq_hsic_ready = 0, irq_mdm_errfatal = 0, irq_mdm_status = 0;

	if ( mdm_drv == NULL ) {
		pr_info("%s: mdm_drv == NULL\n", __func__);
		return;
	}

	if ( mdm_drv->mdm_irq_enabled == enable ) {
		pr_info("%s: mdm_irq_enabled=[%d][%d]\n", __func__, mdm_drv->mdm_irq_enabled, enable);
		return;
	}

	irq_hsic_ready = gpio_to_irq(mdm_drv->mdm2ap_hsic_ready_gpio);
	irq_mdm_errfatal = gpio_to_irq(mdm_drv->mdm2ap_errfatal_gpio);
	irq_mdm_status = gpio_to_irq(mdm_drv->mdm2ap_status_gpio);

	if ( irq_hsic_ready == 0 || irq_mdm_errfatal == 0 || irq_mdm_status == 0 ) {
		pr_info("%s: irq=[%d][%d][%d]\n", __func__, irq_hsic_ready, irq_mdm_errfatal, irq_mdm_status);
		return;
	}

	if ( enable == MDM_MODEM_IRQ_DISABLE ) {
		pr_info("%s: disable irq\n", __func__);
		disable_irq(irq_hsic_ready);
		disable_irq(irq_mdm_errfatal);
		disable_irq(irq_mdm_status);
	} else if ( enable == MDM_MODEM_IRQ_ENABLE ) {
		pr_info("%s: enable irq\n", __func__);
		enable_irq(irq_hsic_ready);
		enable_irq(irq_mdm_errfatal);
		enable_irq(irq_mdm_status);
//++SSD_RIL: 20120704: add suspend/resume function to enable/disable irq wake
	} else if ( enable == MDM_MODEM_IRQ_WAKE_DISABLE ) {
		pr_info("%s: disable irq wake\n", __func__);
		disable_irq_wake(irq_hsic_ready);
		disable_irq_wake(irq_mdm_errfatal);
		disable_irq_wake(irq_mdm_status);
	} else if ( enable == MDM_MODEM_IRQ_WAKE_ENABLE ) {
		pr_info("%s: enable irq wake\n", __func__);
		enable_irq_wake(irq_hsic_ready);
		enable_irq_wake(irq_mdm_errfatal);
		enable_irq_wake(irq_mdm_status);
//--SSD_RIL: 20120704
	} else {
		pr_info("%s: enable=[%d]\n", __func__, enable);
		return;
	}
	//keep latest state
	mdm_drv->mdm_irq_enabled = enable;

}
//--SSD_RIL
/* HTC added end */

#define MDM_MODEM_TIMEOUT	6000
#define MDM_MODEM_DELTA	100

static int mdm_debug_on;
static struct workqueue_struct *mdm_queue;

#define EXTERNAL_MODEM "external_modem"

static struct mdm_modem_drv *mdm_drv;

DECLARE_COMPLETION(mdm_needs_reload);
DECLARE_COMPLETION(mdm_boot);
DECLARE_COMPLETION(mdm_ram_dumps);
/* HTC added start: Workaournd for real-time MDM ramdump druing subsystem restart */
DECLARE_COMPLETION(port_released);
static int need_release_port;
/* HTC added end*/

static int first_boot = 1;

int err_radio;

//++SSD_RIL: Mars_Lin@20120606: Check 1st status.
static int get_hsic_ready_1st_hi = 0;;
//--SSD_RIL

//++SSD Kris Chen
extern spinlock_t enable_wake_irq_lock;
extern spinlock_t mdm_hsic_wakeup_lock;
static bool mdm2ap_wakeup_irq_enabled = false;
void htc_mdm2ap_wakeup_irq_enable_disable(bool enable)
{
	//return if no wakeup gpio
	if (!mdm_drv || mdm_drv->mdm2ap_wakeup_gpio == 0 || mdm_drv->mdm_wakeup_irq == 0)
		return;

	if (mdm2ap_wakeup_irq_enabled != enable)
	{
		if ( get_radio_flag() & 0x0001 )
			pr_info("%s(%d)\n", __func__, enable);

		if (enable)
		{
			enable_irq_wake(mdm_drv->mdm_wakeup_irq);
			enable_irq(mdm_drv->mdm_wakeup_irq);
		}
		else
		{
			disable_irq_wake(mdm_drv->mdm_wakeup_irq);
			disable_irq_nosync(mdm_drv->mdm_wakeup_irq);
		}
		mdm2ap_wakeup_irq_enabled = enable;
	}

	if ( get_radio_flag() & 0x0001 )
		pr_info("dump_gpio: %s\t= %d\n", "MDM2AP_WAKEUP", gpio_get_value(mdm_drv->mdm2ap_wakeup_gpio));
}
EXPORT_SYMBOL_GPL(htc_mdm2ap_wakeup_irq_enable_disable);

extern void mdm_hsic_wakeup(void);

static irqreturn_t mdm2ap_wakeup_isr(int irq, void *dev_id)
{
	if ( get_radio_flag() & 0x0008 )
		pr_info("%s\n", __func__);

	//disable irq
	spin_lock(&enable_wake_irq_lock);
	htc_mdm2ap_wakeup_irq_enable_disable(false);
	spin_unlock(&enable_wake_irq_lock);

	if (mdm_drv->mdm_ready && (gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 1)) {
		spin_lock(&mdm_hsic_wakeup_lock);
		mdm_hsic_wakeup();
		spin_unlock(&mdm_hsic_wakeup_lock);
	}
	else {
		pr_debug("%s: mdm_ready:%d mdm2ap_status_gpio:%d\n", __func__, mdm_drv->mdm_ready, gpio_get_value(mdm_drv->mdm2ap_status_gpio) );
	}

	if ( get_radio_flag() & 0x0001 )
		pr_info("%s end\n", __func__);

	return IRQ_HANDLED;
}

bool is_mdm_support_ap2mdm_sw_bc5 = false;
static irqreturn_t ap2mdm_sw_bc5_isr(int irq, void *dev_id)
{
	int gpio_status = gpio_get_value(mdm_drv->ap2mdm_sw_bc5_gpio);

	if (!is_mdm_support_ap2mdm_sw_bc5) {
		if (mdm_drv->ap2mdm_sw_bc5_status == 1 && gpio_status == 0)	{
			//falling happens
			is_mdm_support_ap2mdm_sw_bc5 = true;
			pr_info("is_mdm_support_ap2mdm_sw_bc5 = true\n");
		}
	}

	mdm_drv->ap2mdm_sw_bc5_status = gpio_status;

	if ( get_radio_flag() & 0x0008 )
		pr_info("%s dump_gpio: %s\t= %d\n", __func__, "AP2MDM_SW_BC5", mdm_drv->ap2mdm_sw_bc5_status);

	if ( get_radio_flag() & 0x0001 )
		trace_printk("AP2MDM_SW_BC5=%d\n", mdm_drv->ap2mdm_sw_bc5_status);
	return IRQ_HANDLED;
}

int get_ap2mdm_sw_bc5_status(void)
{
	mdm_drv->ap2mdm_sw_bc5_status = gpio_get_value(mdm_drv->ap2mdm_sw_bc5_gpio);
	if ( get_radio_flag() & 0x0008 )
		pr_info("%s dump_gpio: %s\t= %d\n", __func__, "AP2MDM_SW_BC5", mdm_drv->ap2mdm_sw_bc5_status);
	return mdm_drv->ap2mdm_sw_bc5_status;
}
//--SSD Kris Chen

long mdm_modem_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	int status, ret = 0;

	if (_IOC_TYPE(cmd) != CHARM_CODE) {
		pr_err("%s: invalid ioctl code\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: Entering ioctl cmd = %d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
	case WAKE_CHARM:
		pr_info("%s: Powering on mdm\n", __func__);
		//++SSD_RIL: Mars_Lin@20120606: reset HSIC READY STATUS
		get_hsic_ready_1st_hi = 0;
		//--SSD_RIL
		mdm_drv->mdm_ready = 0;	/* HTC added for the MDM reloading for the /dev/ttyUSB0 node checking failed situation */
		mdm_drv->ops->power_on_mdm_cb(mdm_drv);
		break;
	case CHECK_FOR_BOOT:
		if (gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 0)
			put_user(1, (unsigned long __user *) arg);
		else
			put_user(0, (unsigned long __user *) arg);
		break;
	case NORMAL_BOOT_DONE:
	{
		//++SSD_RIL: 20120624: check mdm2ap_status_gpio and trigger status_cb if value is 1
		int value = 0;
		//--SSD_RIL: 20120624

		pr_debug("%s: check if mdm is booted up\n", __func__);

		get_user(status, (unsigned long __user *) arg);
		if (status) {
			pr_debug("%s: normal boot failed\n", __func__);
			mdm_drv->mdm_boot_status = -EIO;
		} else {
			pr_info("%s: normal boot done\n", __func__);
			mdm_drv->mdm_boot_status = 0;
		}

		mdm_drv->mdm_ready = 1;

		//HTC_Kris+++
		mdm_is_alive = true;
		//HTC_Kris---

		//++SSD_RIL: 20120624: check mdm2ap_status_gpio and trigger status_cb if value is 1
		mutex_lock(&MDM_BOOT_STATUS_CHECK_LOCK);
		value = mdm2ap_gpio_status;
		pr_info("%s: mdm2ap_status_gpio=[%d]\n", __func__, value);
		if (value) {
		    pr_info("%s: Set mdm9k_status to 1\n", __func__);
		    mdm_drv->ops->status_cb(mdm_drv, value);
		}
		mutex_unlock(&MDM_BOOT_STATUS_CHECK_LOCK);
		//--SSD_RIL: 20120624

		if (mdm_drv->ops->normal_boot_done_cb != NULL)
		{
			pr_info("normal_boot_done_cb\n");
			mdm_drv->ops->normal_boot_done_cb(mdm_drv);
		}

		if (!first_boot)
			complete(&mdm_boot);
		else
			first_boot = 0;
	}
		break;
	case RAM_DUMP_DONE:
		pr_debug("%s: mdm done collecting RAM dumps\n", __func__);
		get_user(status, (unsigned long __user *) arg);
		if (status)
			mdm_drv->mdm_ram_dump_status = -EIO;
		else {
			pr_info("%s: ramdump collection completed\n", __func__);
			mdm_drv->mdm_ram_dump_status = 0;
		}
		complete(&mdm_ram_dumps);
		break;
	case WAIT_FOR_RESTART:
		pr_debug("%s: wait for mdm to need images reloaded\n",
				__func__);
		ret = wait_for_completion_interruptible(&mdm_needs_reload);
		if (!ret)
			put_user(mdm_drv->boot_type,
					 (unsigned long __user *) arg);
		INIT_COMPLETION(mdm_needs_reload);
		break;
/* HTC added start */
	case GET_MFG_MODE:
		pr_info("%s: board_mfg_mode()=%d\n", __func__, board_mfg_mode());
		put_user(board_mfg_mode(),
				 (unsigned long __user *) arg);
		break;
	case GET_RADIO_FLAG:
		pr_info("%s:get_radio_flag()=%x\n", __func__, get_radio_flag());
		put_user(get_radio_flag(),
				 (unsigned long __user *) arg);
		break;
	case EFS_SYNC_DONE:
		pr_info("%s:%s efs sync is done\n", __func__, (atomic_read(&final_efs_wait)? " FINAL": ""));
		atomic_set(&final_efs_wait, 0);
		break;
	case EFS_SYNC_TIMEOUT:
		break;
	/* HTC added start: Workaournd for real-time MDM ramdump druing subsystem restart */
	case WAIT_FOR_PORT_RELEASE:
		pr_info("%s: %s to wait for tty port released\n", __func__, need_release_port? "Need": "Don't need");
		if (need_release_port) {
			wait_for_completion(&port_released);
			INIT_COMPLETION(port_released);
		}
		break;
	case CHECK_PORT_UTIL:
		pr_info("%s: %s to release tty port\n", __func__, need_release_port? "Need": "Don't need");
		put_user(need_release_port, (unsigned long __user *) arg);
		if (need_release_port) {
			complete(&port_released);
			need_release_port = 0;
		}
		break;
	/* HTC workaound end */
/* HTC added end */
	default:
		pr_err("%s: invalid ioctl cmd = %d\n", __func__, _IOC_NR(cmd));
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* HTC added start */
static void dump_gpio(char *name, unsigned int gpio)
{
        if (gpio == 0) {
               pr_err("%s: Cannot dump %s, due to invalid gpio number %d\n", __func__, name, gpio);
                return;
       }
        pr_info("%s: %s\t= %d\n", __func__, name, gpio_get_value(gpio));

	return;
}

static void dump_mdm_related_gpio(void)
{
        dump_gpio("AP2MDM_STATUS", mdm_drv->ap2mdm_status_gpio);
        /* charm_dump_GPIO("AP2MDM_WAKEUP", mdm_drv->ap2mdm_wakeup_gpio); */
        dump_gpio("AP2MDM_ERRFATAL", mdm_drv->ap2mdm_errfatal_gpio);
        dump_gpio("AP2MDM_PMIC_RESET_N", mdm_drv->ap2mdm_pmic_reset_n_gpio);

        dump_gpio("MDM2AP_STATUS", mdm_drv->mdm2ap_status_gpio);
        /* charm_dump_GPIO("MDM2AP_WAKEUP", mdm_drv->mdm2ap_wakeup_gpio); */
        dump_gpio("MDM2AP_ERRFATAL", mdm_drv->mdm2ap_errfatal_gpio);
        dump_gpio("MDM2AP_WAKEUP", mdm_drv->mdm2ap_wakeup_gpio);
        dump_gpio("AP2MDM_SW_BC5", mdm_drv->ap2mdm_sw_bc5_gpio);

	return;
}
/* HTC added end */

static void mdm_fatal_fn(struct work_struct *work)
{
	//HTC_Kris+++
	unsigned long flags;
	extern bool is_mdm_hsic_phy_suspended;
	extern bool is_mdm_hsic_wakeup_in_progress;
	extern void mdm_hsic_disable_auto_suspend(void);
	int i;
	//HTC_Kris---

	pr_info("%s: Reseting the mdm due to an errfatal\n", __func__);

	//HTC_Kris+++
	pr_info("%s: mdm_hsic_disable_auto_suspend+\n", __func__);
	mdm_hsic_disable_auto_suspend();
	pr_info("%s: mdm_hsic_disable_auto_suspend-\n", __func__);
	//HTC_Kris---

	/* HTC added start */
	dump_mdm_related_gpio();
	//++SSD_RIL:20120724:delay 3 secs before call subsystem restart
	if ( get_radio_flag() & 0x0008 ) {
		mdelay(3000);
	}
	//--SSD_RIL
	/* HTC added end */

	//HTC_Kris+++
	//Before do radio restart, make sure mdm_hsic_phy is not suspended, otherwise, PORTSC will be kept at 1800
	if (is_mdm_hsic_phy_suspended) {
		pr_info("%s(%d): is_mdm_hsic_phy_suspended:%d\n", __func__, __LINE__, is_mdm_hsic_phy_suspended);
		pr_info("%s(%d): wakeup hsic\n", __func__, __LINE__);
		spin_lock_irqsave(&mdm_hsic_wakeup_lock, flags);
		mdm_hsic_wakeup();
		spin_unlock_irqrestore(&mdm_hsic_wakeup_lock, flags);

		//wait until mdm_hsic_phy is not suspended, at most 10 seconds
		for (i = 0; i < 100; i++) {
			msleep(100);
			if (!is_mdm_hsic_phy_suspended && !is_mdm_hsic_wakeup_in_progress)
				break;
		}
		pr_info("%s(%d): is_mdm_hsic_phy_suspended:%d\n", __func__, __LINE__, is_mdm_hsic_phy_suspended);
	}
	//HTC_Kris---

	subsystem_restart(EXTERNAL_MODEM);
}

static DECLARE_WORK(mdm_fatal_work, mdm_fatal_fn);

static void mdm_status_fn(struct work_struct *work)
{
	
	int value;

	//HTC+++
	mutex_lock(&MDM_BOOT_STATUS_CHECK_LOCK);
	mdm2ap_gpio_status = gpio_get_value(mdm_drv->mdm2ap_status_gpio);
	value = mdm2ap_gpio_status;
	//HTC---

	pr_info("mdm_status_fn\n");

	mdm_drv->ops->status_cb(mdm_drv, value);

	pr_debug("%s: status:%d\n", __func__, value);

	if ((value == 0) && mdm_drv->mdm_ready && !device_ehci_shutdown) {
		pr_info("%s: unexpected reset external modem\n", __func__);
		/* HTC added start */
		dump_mdm_related_gpio();
		/* HTC added end */
		subsystem_restart(EXTERNAL_MODEM);
	} else if (value == 1) {
	  //Sophia:0510-Change to NO_PULL
	  extern const int gpio_to_pingroup[TEGRA_MAX_GPIO];
	  tegra_pinmux_set_pullupdown(gpio_to_pingroup[mdm_drv->mdm2ap_status_gpio], TEGRA_PUPD_NORMAL);
		pr_info("%s: status = 1: mdm is now ready\n", __func__);
	}

	//HTC+++
	mutex_unlock(&MDM_BOOT_STATUS_CHECK_LOCK);
	//HTC---
}

extern void mdm_peripheral_connect(struct mdm_modem_drv *mdm_drv);
extern void mdm_peripheral_disconnect(struct mdm_modem_drv *mdm_drv);



static void  mdm_hsic_ready_fn(struct work_struct *work)
{
	int value = gpio_get_value(mdm_drv->mdm2ap_hsic_ready_gpio);

	pr_debug("%s: status:%d\n", __func__, value);

	//++SSD_RIL: Mars_Lin@20120606: Check 1st status.
	if ( value == 1 && get_hsic_ready_1st_hi == 0 ) {
		get_hsic_ready_1st_hi = 1;
		pr_info("%s: get_hsic_ready_1st_hi = 1\n", __func__);
	}

	if ( get_hsic_ready_1st_hi == 0 ) {
		pr_info("%s: get_hsic_ready_1st_hi == 0\n", __func__);
		return;
	}
	//--SSD_RIL

	if (value == 0) {
#if 0
		pr_info("%s: get HSIC low\n", __func__);
		pr_info("mdm_hsic_ready: unregister:tegra_ehci2_device\n");
		mdm_peripheral_disconnect(mdm_drv);
		pr_info("mdm_hsic_ready: register:tegra_ehci2_device\n");
		mdm_peripheral_connect(mdm_drv);
		//TEGRA_GPIO_PC6
		gpio_direction_output(mdm_drv->ap2mdm_wakeup_gpio, 1);
		tegra_gpio_enable(mdm_drv->ap2mdm_wakeup_gpio);
		gpio_set_value(mdm_drv->ap2mdm_wakeup_gpio, 1);
#endif
	}
}


static DECLARE_WORK(mdm_status_work, mdm_status_fn);

static DECLARE_WORK(mdm_hsic_ready_work, mdm_hsic_ready_fn);



static void mdm_disable_irqs(void)
{
	disable_irq_nosync(mdm_drv->mdm_errfatal_irq);
	disable_irq_nosync(mdm_drv->mdm_status_irq);
	disable_irq_nosync(mdm_drv->mdm_wakeup_irq);
}

static irqreturn_t mdm_errfatal(int irq, void *dev_id)
{
	pr_debug("%s: mdm got errfatal interrupt\n", __func__);

	if ( get_radio_flag() & 0x0001 ) {
		trace_printk("%s: mdm got errfatal interrupt\n", __func__);
		tracing_off();
	}

	if (mdm_drv->mdm_ready &&
		(gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 1) && !device_ehci_shutdown) {

		//HTC_Kris+++
		mdm_in_fatal_handler = true;
		mdm_is_alive = false;
		//HTC_Kris---

		pr_debug("%s: scheduling work now\n", __func__);
		queue_work(mdm_queue, &mdm_fatal_work);
	}
	return IRQ_HANDLED;
}

static int mdm_modem_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations mdm_modem_fops = {
	.owner		= THIS_MODULE,
	.open		= mdm_modem_open,
	.unlocked_ioctl	= mdm_modem_ioctl,
};


static struct miscdevice mdm_modem_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mdm",
	.fops	= &mdm_modem_fops
};

static int mdm_panic_prep(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	int i;

	pr_debug("%s: setting AP2MDM_ERRFATAL high for a non graceful reset\n",
			 __func__);
	mdm_disable_irqs();

	//HTC_Kris+++
	mdm_is_alive = false;
	//HTC_Kris---

	gpio_set_value(mdm_drv->ap2mdm_errfatal_gpio, 1);

	if (mdm_drv->ap2mdm_wakeup_gpio > 0)
		gpio_set_value(mdm_drv->ap2mdm_wakeup_gpio, 1);

	for (i = MDM_MODEM_TIMEOUT; i > 0; i -= MDM_MODEM_DELTA) {
		//pet_watchdog();
		mdelay(MDM_MODEM_DELTA);
		if (gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 0)
			break;
	}
	if (i <= 0)
		pr_err("%s: MDM2AP_STATUS never went low\n", __func__);
	return NOTIFY_DONE;
}

static struct notifier_block mdm_panic_blk = {
	.notifier_call  = mdm_panic_prep,
};

static irqreturn_t mdm_status_change(int irq, void *dev_id)
{
	pr_debug("%s: mdm sent status change interrupt\n", __func__);

	//HTC_Kris+++
	if (gpio_get_value(mdm_drv->mdm2ap_status_gpio) == 0) {
		pr_err("%s: mdm2ap_status_gpio is 0\n", __func__);
		mdm_is_alive = false;
	}
	//HTC_Kris---

	queue_work(mdm_queue, &mdm_status_work);

	return IRQ_HANDLED;
}



static irqreturn_t hsic_ready_status(int irq, void *dev_id)
{
	pr_debug("%s: mdm sent hsic_ready interrupt\n", __func__);

	queue_work(mdm_queue, &mdm_hsic_ready_work);

	return IRQ_HANDLED;
}

static int mdm_subsys_shutdown(const struct subsys_data *crashed_subsys)
{
	pr_debug("[%s]\n", __func__);
	mdm_drv->mdm_ready = 0;

	//HTC_Kris+++
	mdm_is_alive = false;
	//HTC_Kris---

	gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 1);
	if (mdm_drv->pdata->ramdump_delay_ms > 0) {
		/* Wait for the external modem to complete
		 * its preparation for ramdumps.
		 */
		mdelay(mdm_drv->pdata->ramdump_delay_ms);
	}
	mdm_drv->ops->power_down_mdm_cb(mdm_drv);
	/* HTC added start: Workaournd for real-time MDM ramdump druing subsystem restart */
	/* ap2mdm_errfatal_gpio should be pulled low otherwise MDM will assume 8K fatal after bootup*/
	need_release_port = 1;
	gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 0);
	dump_mdm_related_gpio();
	/* HTC added end */
	return 0;
}

static int mdm_subsys_powerup(const struct subsys_data *crashed_subsys)
{
	pr_debug("[%s]\n", __func__);
	gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 0);
	gpio_direction_output(mdm_drv->ap2mdm_status_gpio, 1);
	//mdm_drv->ops->power_on_mdm_cb(mdm_drv);
	mdm_drv->boot_type = CHARM_NORMAL_BOOT;
	complete(&mdm_needs_reload);
	wait_for_completion(&mdm_boot);
	pr_info("%s: mdm modem has been restarted\n", __func__);
	INIT_COMPLETION(mdm_boot);

	open_hsicctl_timeout_trigger_errfatal = 0;

	//++HTC
	mdm_in_fatal_handler = false;
	//--HTC

	return mdm_drv->mdm_boot_status;
}

static int mdm_subsys_ramdumps(int want_dumps,
				const struct subsys_data *crashed_subsys)
{
	mdm_drv->mdm_ram_dump_status = 0;
	pr_debug("%s: want_dumps is %d\n", __func__, want_dumps);
	if (want_dumps) {
		mdm_drv->boot_type = CHARM_RAM_DUMPS;
		complete(&mdm_needs_reload);
		wait_for_completion(&mdm_ram_dumps);
		INIT_COMPLETION(mdm_ram_dumps);
		gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 1);
		mdm_drv->ops->power_down_mdm_cb(mdm_drv);
		/* HTC added start: Workaournd for real-time MDM ramdump druing subsystem restart */
		/* ap2mdm_errfatal_gpio should be pulled low otherwise MDM will assume 8K fatal after bootup*/
		gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 0);
		dump_mdm_related_gpio();
		/* HTC added end */
	}
	return mdm_drv->mdm_ram_dump_status;
}

static struct subsys_data mdm_subsystem = {
	.shutdown = mdm_subsys_shutdown,
	.ramdump = mdm_subsys_ramdumps,
	.powerup = mdm_subsys_powerup,
	.name = EXTERNAL_MODEM,
};

static int mdm_debug_on_set(void *data, u64 val)
{
	mdm_debug_on = val;
	if (mdm_drv->ops->debug_state_changed_cb)
		mdm_drv->ops->debug_state_changed_cb(mdm_debug_on);
	return 0;
}

static int mdm_debug_on_get(void *data, u64 *val)
{
	*val = mdm_debug_on;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mdm_debug_on_fops,
			mdm_debug_on_get,
			mdm_debug_on_set, "%llu\n");

static int mdm_debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("mdm_dbg", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("debug_on", 0644, dent, NULL,
			&mdm_debug_on_fops);
	return 0;
}

static void mdm_modem_initialize_data(struct platform_device  *pdev,
				struct mdm_ops *mdm_ops)
{
	struct resource *pres;

	/* MDM2AP_ERRFATAL */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"MDM2AP_ERRFATAL");
	if (pres)
		mdm_drv->mdm2ap_errfatal_gpio = pres->start;

	/* AP2MDM_ERRFATAL */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_ERRFATAL");
	if (pres)
		mdm_drv->ap2mdm_errfatal_gpio = pres->start;

	/* MDM2AP_STATUS */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"MDM2AP_STATUS");
	if (pres)
		mdm_drv->mdm2ap_status_gpio = pres->start;

	/* AP2MDM_STATUS */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_STATUS");
	if (pres)
		mdm_drv->ap2mdm_status_gpio = pres->start;

	/* MDM2AP_WAKEUP */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"MDM2AP_WAKEUP");
	if (pres)
		mdm_drv->mdm2ap_wakeup_gpio = pres->start;

	/* AP2MDM_WAKEUP */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_WAKEUP");
	if (pres)
		mdm_drv->ap2mdm_wakeup_gpio = pres->start;

	/* AP2MDM_PMIC_RESET_N */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_PMIC_RESET_N");
	if (pres)
		mdm_drv->ap2mdm_pmic_reset_n_gpio = pres->start;

	/* AP2MDM_KPDPWR_N */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_KPDPWR_N");
	if (pres)
		mdm_drv->ap2mdm_kpdpwr_n_gpio = pres->start;

	/* MDM2AP_HSIC_READY */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"MDM2AP_HSIC_READY");
	if (pres)
		mdm_drv->mdm2ap_hsic_ready_gpio = pres->start;

	//HTC+++
	/* AP2MDM_SW_BC5 */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"AP2MDM_SW_BC5");
	if (pres)
		mdm_drv->ap2mdm_sw_bc5_gpio = pres->start;
	//HTC---

//++SSD_RIL:Mars_Lin@20120613: Add PM4 for operaul mdm power on
	/* V_DCIN_MODEM_EN */
	pres = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"V_DCIN_MODEM_EN");
	if (pres)
		mdm_drv->v_dcin_modem_en_gpio = pres->start;
//--SSD_RIL

	mdm_drv->boot_type                  = CHARM_NORMAL_BOOT;

	mdm_drv->ops      = mdm_ops;
	mdm_drv->pdata    = pdev->dev.platform_data;
}

int mdm_common_create(struct platform_device  *pdev,
					  struct mdm_ops *p_mdm_cb)
{
	int ret = -1, irq;
	
	pr_info("mdm_common_create\n");
	

	mdm_drv = kzalloc(sizeof(struct mdm_modem_drv), GFP_KERNEL);
	if (mdm_drv == NULL) {
		pr_err("%s: kzalloc fail.\n", __func__);
		goto alloc_err;
	}

	mdm_modem_initialize_data(pdev, p_mdm_cb);
	if (mdm_drv->ops->debug_state_changed_cb)
		mdm_drv->ops->debug_state_changed_cb(mdm_debug_on);

	gpio_request(mdm_drv->ap2mdm_status_gpio, "AP2MDM_STATUS");
	gpio_request(mdm_drv->ap2mdm_errfatal_gpio, "AP2MDM_ERRFATAL");
	gpio_request(mdm_drv->ap2mdm_kpdpwr_n_gpio, "AP2MDM_KPDPWR_N");
	gpio_request(mdm_drv->ap2mdm_pmic_reset_n_gpio, "AP2MDM_PMIC_RESET_N");
		
	gpio_request(mdm_drv->mdm2ap_status_gpio, "MDM2AP_STATUS");
	gpio_request(mdm_drv->mdm2ap_errfatal_gpio, "MDM2AP_ERRFATAL");
	gpio_request(mdm_drv->mdm2ap_wakeup_gpio, "MDM2AP_WAKEUP");

	gpio_request(mdm_drv->mdm2ap_hsic_ready_gpio, "MDM2AP_HSIC_READY");

	//HTC+++
	gpio_request(mdm_drv->ap2mdm_sw_bc5_gpio, "AP2MDM_SW_BC5");
	//HTC---

	//Set AP2MDM_WAKEUP -TEGRA_GPIO_PC6
	gpio_request(mdm_drv->ap2mdm_wakeup_gpio, "AP2MDM_WAKEUP");	
	gpio_direction_output(mdm_drv->ap2mdm_wakeup_gpio, 0);
	tegra_gpio_enable(mdm_drv->ap2mdm_wakeup_gpio);
	gpio_set_value(mdm_drv->ap2mdm_wakeup_gpio, 0);

   /*
	gpio_direction_output(mdm_drv->ap2mdm_status_gpio, 1);
	tegra_gpio_enable(mdm_drv->ap2mdm_status_gpio);
	gpio_set_value(mdm_drv->ap2mdm_status_gpio, 1);	
	*/

	gpio_direction_output(mdm_drv->ap2mdm_status_gpio,0 );
	tegra_gpio_enable(mdm_drv->ap2mdm_status_gpio);
	gpio_set_value(mdm_drv->ap2mdm_status_gpio, 0);

	gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 0);
	tegra_gpio_enable(mdm_drv->ap2mdm_errfatal_gpio);
	gpio_set_value(mdm_drv->ap2mdm_errfatal_gpio, 0);	

	if (mdm_drv->ap2mdm_wakeup_gpio > 0)
		gpio_direction_output(mdm_drv->ap2mdm_wakeup_gpio, 0);

	gpio_direction_input(mdm_drv->mdm2ap_status_gpio);
	tegra_gpio_enable(mdm_drv->mdm2ap_status_gpio);
	
	gpio_direction_input(mdm_drv->mdm2ap_errfatal_gpio);
	tegra_gpio_enable(mdm_drv->mdm2ap_errfatal_gpio);

	gpio_direction_input(mdm_drv->mdm2ap_wakeup_gpio);
	tegra_gpio_enable(mdm_drv->mdm2ap_wakeup_gpio);

	gpio_direction_input(mdm_drv->mdm2ap_hsic_ready_gpio);
	tegra_gpio_enable(mdm_drv->mdm2ap_hsic_ready_gpio);

	//HTC+++
	gpio_direction_input(mdm_drv->ap2mdm_sw_bc5_gpio);
	tegra_gpio_enable(mdm_drv->ap2mdm_sw_bc5_gpio);
	//HTC---

//++SSD_RIL:20120705: Export all related GPIO
	if ( mdm_drv->mdm_gpio_exported == 0 ) {

		if ( mdm_drv->mdm2ap_errfatal_gpio > 0 )
			gpio_export(mdm_drv->mdm2ap_errfatal_gpio, true);
		if ( mdm_drv->ap2mdm_errfatal_gpio > 0 )
			gpio_export(mdm_drv->ap2mdm_errfatal_gpio, true);
		if ( mdm_drv->mdm2ap_status_gpio > 0 )
			gpio_export(mdm_drv->mdm2ap_status_gpio, true);
		if ( mdm_drv->ap2mdm_status_gpio > 0 )
			gpio_export(mdm_drv->ap2mdm_status_gpio, true);
		if ( mdm_drv->mdm2ap_wakeup_gpio > 0 )
			gpio_export(mdm_drv->mdm2ap_wakeup_gpio, true);
		if ( mdm_drv->ap2mdm_wakeup_gpio > 0 )
			gpio_export(mdm_drv->ap2mdm_wakeup_gpio, true);
		if ( mdm_drv->ap2mdm_pmic_reset_n_gpio > 0 )
			gpio_export(mdm_drv->ap2mdm_pmic_reset_n_gpio, true);
		if ( mdm_drv->ap2mdm_kpdpwr_n_gpio > 0 )
			gpio_export(mdm_drv->ap2mdm_kpdpwr_n_gpio, true);
		if ( mdm_drv->mdm2ap_hsic_ready_gpio > 0 )
			gpio_export(mdm_drv->mdm2ap_hsic_ready_gpio, true);
		if ( mdm_drv->v_dcin_modem_en_gpio > 0 )
			gpio_export(mdm_drv->v_dcin_modem_en_gpio, true);
		if ( mdm_drv->ap2mdm_sw_bc5_gpio > 0 )
			gpio_export(mdm_drv->ap2mdm_sw_bc5_gpio, true);

		mdm_drv->mdm_gpio_exported = 1;
	}
//--SSD_RIL
//++SSD_RIL:Mars_Lin@20120613: Add PM4 for operaul mdm power on
	if ( mdm_drv->v_dcin_modem_en_gpio > 0 ) {
		gpio_request(mdm_drv->v_dcin_modem_en_gpio, "V_DCIN_MODEM_EN");
		pr_info("%s: Set MDM power\n", __func__);
		gpio_direction_output(mdm_drv->v_dcin_modem_en_gpio, 0);
		tegra_gpio_enable(mdm_drv->v_dcin_modem_en_gpio);
		gpio_set_value(mdm_drv->v_dcin_modem_en_gpio, 1);
	}
//--SSD_RIL

	mdm_queue = create_singlethread_workqueue("mdm_queue");
	if (!mdm_queue) {
		pr_err("%s: could not create workqueue. All mdm "
				"functionality will be disabled\n",
			__func__);
		ret = -ENOMEM;
		goto fatal_err;
	}

	atomic_notifier_chain_register(&panic_notifier_list, &mdm_panic_blk);
	mdm_debugfs_init();

	/* Register subsystem handlers */
	ssr_register_subsystem(&mdm_subsystem);
	
	/* Enable HSIC READY irq*/
	pr_info("Enable HSIC READY \n");
	irq = gpio_to_irq(mdm_drv->mdm2ap_hsic_ready_gpio);

	if (irq < 0) {
		pr_info("could not get MDM2AP_HSIC_READY IRQ resource\n");

		pr_err("%s: could not get MDM2AP_STATUS IRQ resource. "
			"error=%d No IRQ will be generated on status change.",
			__func__, irq);
		goto status_err;
	}
	err_radio = request_irq(irq,
			hsic_ready_status,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"MDM2AP_HSIC_READY",
			&mdm_drv);

		if (err_radio < 0) {
			pr_err("%s - request irq MDM2AP_STATUS failed\n",
				__func__);
			//return err;
		}



	/* ERR_FATAL irq. */
	//irq = MSM_GPIO_TO_INT(mdm_drv->mdm2ap_errfatal_gpio);
	irq = gpio_to_irq(mdm_drv->mdm2ap_errfatal_gpio);
	
	
	if (irq < 0) {
		pr_err("%s: could not get MDM2AP_ERRFATAL IRQ resource. "
			"error=%d No IRQ will be generated on errfatal.",
			__func__, irq);
		goto errfatal_err;
	}
	ret = request_irq(irq, mdm_errfatal,
		IRQF_TRIGGER_RISING , "mdm errfatal", NULL);

	if (ret < 0) {
		pr_err("%s: MDM2AP_ERRFATAL IRQ#%d request failed with error=%d"
			". No IRQ will be generated on errfatal.",
			__func__, irq, ret);
		goto errfatal_err;
	}
	mdm_drv->mdm_errfatal_irq = irq;

	/* WAKEUP irq. */
	if (mdm_drv->mdm2ap_wakeup_gpio > 0)
	{
		irq = gpio_to_irq(mdm_drv->mdm2ap_wakeup_gpio);
		pr_err("%s: get MDM2AP_WAKEUP IRQ resource: %d ",
			__func__, irq);

		if (irq < 0) {
			pr_err("%s: could not get MDM2AP_WAKEUP IRQ resource. "
				"error=%d No IRQ will be generated on errfatal.",
				__func__, irq);
			goto errfatal_err;
		}
		ret = request_irq(irq, mdm2ap_wakeup_isr,
			IRQF_TRIGGER_HIGH , "mdm_hsic_wakeup", NULL);

		if (ret < 0) {
			pr_err("%s: MDM2AP_WAKEUP IRQ#%d request failed with error=%d"
				". No IRQ will be generated on errfatal.",
				__func__, irq, ret);
			goto errfatal_err;
		}

		mdm_drv->mdm_wakeup_irq = irq;

		/* disable it by default */
		pr_info("%s: mdm_drv->mdm_wakeup_irq: %d ", __func__, mdm_drv->mdm_wakeup_irq);
		disable_irq(mdm_drv->mdm_wakeup_irq);
		disable_irq_wake(mdm_drv->mdm_wakeup_irq);
	}

	//HTC+++
	/* AP2MDM_SW_BC5 irq. */
	if (mdm_drv->ap2mdm_sw_bc5_gpio > 0)
	{
		irq = gpio_to_irq(mdm_drv->ap2mdm_sw_bc5_gpio);
		pr_err("%s: get AP2MDM_SW_BC5 IRQ resource: %d ",
			__func__, irq);

		if (irq < 0) {
			pr_err("%s: could not get AP2MDM_SW_BC5 IRQ resource. "
				"error=%d No IRQ will be generated on errfatal.",
				__func__, irq);
			goto errfatal_err;
		}
		ret = request_irq(irq, ap2mdm_sw_bc5_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING , "ap2mdm_sw_bc5_irq", NULL);

		if (ret < 0) {
			pr_err("%s: AP2MDM_SW_BC5 IRQ#%d request failed with error=%d"
				". No IRQ will be generated on errfatal.",
				__func__, irq, ret);
			goto errfatal_err;
		}

		mdm_drv->ap2mdm_sw_bc5_irq = irq;
		mdm_drv->ap2mdm_sw_bc5_status = gpio_get_value(mdm_drv->ap2mdm_sw_bc5_gpio);
		pr_info("%s: mdm_drv->ap2mdm_sw_bc5_status: %d ", __func__, mdm_drv->ap2mdm_sw_bc5_status);

		/* enable it by default */
		pr_info("%s: mdm_drv->ap2mdm_sw_bc5_irq: %d ", __func__, mdm_drv->ap2mdm_sw_bc5_irq);
		enable_irq(mdm_drv->ap2mdm_sw_bc5_irq);
	}
	//HTC---

errfatal_err:

	/* status irq */
	//irq = MSM_GPIO_TO_INT(mdm_drv->mdm2ap_status_gpio);
	irq = gpio_to_irq(mdm_drv->mdm2ap_status_gpio);
	
	//irq = gpio_to_irq(TEGRA_GPIO_PS4);
	
	if (irq < 0) {
		pr_err("%s: could not get MDM2AP_STATUS IRQ resource. "
			"error=%d No IRQ will be generated on status change.",
			__func__, irq);
		goto status_err;
	}
#if 0
	ret = request_threaded_irq(irq, NULL, mdm_status_change,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,
		"mdm status", mdm_drv);

	if (ret < 0) {
		pr_err("%s: MDM2AP_STATUS IRQ#%d request failed with error=%d"
			". No IRQ will be generated on status change.",
			__func__, irq, ret);
		goto status_err;
	}
#endif

		err_radio = request_irq(irq,
			mdm_status_change,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"MDM2AP_STATUS IRQ",
			&mdm_drv);

		if (err_radio < 0) {
			pr_err("%s - request irq MDM2AP_STATUS failed\n",
				__func__);
			//return err;
		}

	mdm_drv->mdm_status_irq = irq;

status_err:
	/* Perform early powerup of the external modem in order to
	 * allow tabla devices to be found.
	 */
	 
//mark it by sophia	
//mdm_drv->ops->power_on_mdm_cb(mdm_drv);

	//++SSD_RIL: Mars_Lin@20120608: disable irq because MDM not power on
	mdm_drv->mdm_irq_enabled = MDM_MODEM_IRQ_ENABLE;
	mdm_common_enable_irq(mdm_drv, MDM_MODEM_IRQ_DISABLE);
	//--SSD_RIL

	pr_info("%s: Registering mdm modem\n", __func__);
	return misc_register(&mdm_modem_misc);

fatal_err:
	gpio_free(mdm_drv->ap2mdm_status_gpio);
	gpio_free(mdm_drv->ap2mdm_errfatal_gpio);
	gpio_free(mdm_drv->ap2mdm_kpdpwr_n_gpio);
	gpio_free(mdm_drv->ap2mdm_pmic_reset_n_gpio);
	gpio_free(mdm_drv->mdm2ap_status_gpio);
	gpio_free(mdm_drv->mdm2ap_errfatal_gpio);

	if (mdm_drv->ap2mdm_wakeup_gpio > 0)
		gpio_free(mdm_drv->ap2mdm_wakeup_gpio);

	kfree(mdm_drv);
	ret = -ENODEV;

alloc_err:
	return ret;
}

int mdm_common_modem_remove(struct platform_device *pdev)
{
	int ret;

	gpio_free(mdm_drv->ap2mdm_status_gpio);
	gpio_free(mdm_drv->ap2mdm_errfatal_gpio);
	gpio_free(mdm_drv->ap2mdm_kpdpwr_n_gpio);
	gpio_free(mdm_drv->ap2mdm_pmic_reset_n_gpio);
	gpio_free(mdm_drv->mdm2ap_status_gpio);
	gpio_free(mdm_drv->mdm2ap_errfatal_gpio);

	if (mdm_drv->ap2mdm_wakeup_gpio > 0)
		gpio_free(mdm_drv->ap2mdm_wakeup_gpio);

	kfree(mdm_drv);

	ret = misc_deregister(&mdm_modem_misc);
	return ret;
}

void mdm_common_modem_shutdown(struct platform_device *pdev)
{
	pr_debug("%s: setting AP2MDM_STATUS low for a graceful restart\n",
		__func__);

	mdm_disable_irqs();

	atomic_set(&final_efs_wait, 1);	/* HTC added */

	//HTC_Kris+++
	mdm_is_alive = false;
	//HTC_Kris---

	gpio_set_value(mdm_drv->ap2mdm_status_gpio, 0);

	if (mdm_drv->ap2mdm_wakeup_gpio > 0)
		gpio_set_value(mdm_drv->ap2mdm_wakeup_gpio, 1);

	mdm_drv->ops->power_down_mdm_cb(mdm_drv);

	if (mdm_drv->ap2mdm_wakeup_gpio > 0)
		gpio_set_value(mdm_drv->ap2mdm_wakeup_gpio, 0);
}

//++SSD_RIL: 20120704: add suspend/resume function to enable/disable irq wake
void mdm_common_modem_suspend( )
{
	mdm_common_enable_irq(mdm_drv, MDM_MODEM_IRQ_WAKE_ENABLE);
}

void mdm_common_modem_resume( )
{
	mdm_common_enable_irq(mdm_drv, MDM_MODEM_IRQ_WAKE_DISABLE);
}
//--SSD_RIL: 20120704



void trigger_ap2mdm_errfatal(void)
{
	pr_info("%s+\n", __func__);

	//HTC_Kris+++
	mdm_is_alive = false;
	//HTC_Kris---

	gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 0);
	mdelay(1000);
	gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 1);
	mdelay(1000);
	gpio_direction_output(mdm_drv->ap2mdm_errfatal_gpio, 0);

	pr_info("%s-\n", __func__);
}
EXPORT_SYMBOL_GPL(trigger_ap2mdm_errfatal);
