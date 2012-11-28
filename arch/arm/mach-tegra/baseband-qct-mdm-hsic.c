/*
 * arch/arm/mach-tegra/baseband-qct-mdm-hsic.c
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <mach/usb_phy.h>
#include <mach/board_htc.h>
#include <linux/pm_qos_params.h>
#include <linux/usb.h>
#include <linux/usbdevice_fs.h>
#include <linux/usb/hcd.h>
#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "baseband-qct-mdm-hsic.h"


extern void htc_mdm2ap_wakeup_irq_enable_disable(bool enable);

//USB
struct usb_device *mdm_usb1_1_usbdev = NULL;
struct device *mdm_usb1_1_dev = NULL;
struct usb_hcd *mdm_hsic_usb_hcd = NULL;
struct ehci_hcd *mdm_hsic_ehci_hcd = NULL;

struct usb_device *mdm_usb1_usbdev = NULL;
struct device *mdm_usb1_dev = NULL;

//PM
static bool is_pm_suspended = false;

//MDM2AP WAKEUP
DEFINE_SPINLOCK(mdm_hsic_wakeup_lock);
DEFINE_SPINLOCK(enable_wake_irq_lock);
static struct workqueue_struct *mdm2ap_wakeup_queue;
struct usb_interface *mdm2ap_wakeup_intf;
static bool hsic_wakeup_pending = false;
bool is_mdm_hsic_phy_suspended = false;
bool is_mdm_hsic_wakeup_in_progress = false;
unsigned long  mdm_hsic_phy_resume_jiffies = 0;
unsigned long  mdm_hsic_phy_active_total_ms = 0;

//USB PM DBG
#define HSIC_PM_MON_DELAY 30000
static struct	delayed_work mdm_hsic_pm_monitor_delayed_work;
static void mdm_hsic_print_usb_dev_pm_info(struct usb_device *udev);
static void mdm_hsic_print_interface_pm_info(struct usb_device *udev);

//MISC.
static bool is_modem_qct_mdm9k = false;

/*****************************  Wake Lock  *****************************/

#define MDM_HSIC_WLOCK_INIT(wlock)					\
do {								\
	pr_info("%s(%d) mdm_hsic wake_lock_init(0x%p)\n", __func__, __LINE__, &(wlock));	\
	wake_lock_init(&(wlock), WAKE_LOCK_SUSPEND, "mdm_hsic_wake_lock");			\
} while (0)

#define MDM_HSIC_WLOCK_DESTROY(wlock)					\
do {								\
	pr_info("%s(%d) mdm_hsic wake_lock_destroy(0x%p)\n", __func__, __LINE__, &(wlock));	\
	wake_lock_destroy(&(wlock));			\
} while (0)

#define MDM_HSIC_WLOCK_LOCK(wlock)					\
do {	\
	if (!wake_lock_active(&(wlock)))	\
	{	\
		pr_info("%s(%d) mdm_hsic wake_lock(0x%p)  \n", __func__, __LINE__, &(wlock));	\
		wake_lock(&(wlock));	\
	}	\
} while (0)

#define MDM_HSIC_WLOCK_UNLOCK(wlock)					\
do {								\
	pr_info("%s(%d) mdm_hsic wake_unlock(0x%p) phy_active_total_ms:%lu\n", __func__, __LINE__, &(wlock), mdm_hsic_phy_active_total_ms);	\
	wake_unlock(&(wlock));	\
} while (0)

static struct wake_lock mdm_hsic_wlock;

/*****************************  USB notify *****************************/

#define USB1_VENDOR_ID				0x1d6b
#define USB1_HSIC_PRODUCT_ID		0x0002

#define MDM_HSIC_VENDOR_ID         0x05c6
#define MDM_HSIC_PRODUCT_ID        0x9048

static struct usb_device_id usb1_ids[] = {
	{ USB_DEVICE(USB1_VENDOR_ID, USB1_HSIC_PRODUCT_ID),
	.driver_info = 0 },
	{}
};

static struct usb_device_id mdm_hsic_ids[] = {
	{ USB_DEVICE(MDM_HSIC_VENDOR_ID, MDM_HSIC_PRODUCT_ID),
	.driver_info = 0 },
	{}
};

static void mdm_hsic_usb_device_add_handler(struct usb_device *udev)
{
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);
	const struct usb_device_id *usb1_id, *mdm_hsic_id;

	if (intf == NULL)
		return;

	pr_info("%s(%d) USB device added %d <%s %s>\n", __func__, __LINE__, udev->devnum, udev->manufacturer, udev->product);

	usb1_id = usb_match_id(intf, usb1_ids);
	if (usb1_id) {
		pr_info("%s(%d) VENDOR_ID:%x PRODUCT_ID:%x found \n", __func__, __LINE__, USB1_VENDOR_ID, USB1_HSIC_PRODUCT_ID);

		mdm_usb1_usbdev = udev;
		mdm_usb1_dev = &(udev->dev);
	}

	mdm_hsic_id = usb_match_id(intf, mdm_hsic_ids);
	if (mdm_hsic_id) {
		pr_info("%s(%d) VENDOR_ID:%x PRODUCT_ID:%x found \n", __func__, __LINE__, MDM_HSIC_VENDOR_ID, MDM_HSIC_PRODUCT_ID);

		mdm_usb1_1_usbdev = udev;
		mdm_usb1_1_dev = &(udev->dev);
	}

	if (mdm_hsic_id || usb1_id) {
		//enable auto suspend
		usb_enable_autosuspend(udev);
		dev_info(&(udev->dev), "%s usb_enable_autosuspend\n", __func__);

		//print pm info
		mdm_hsic_print_usb_dev_pm_info(udev);
	}
}

static void mdm_hsic_usb_device_remove_handler(struct usb_device *udev)
{
	if (mdm_usb1_usbdev == udev) {
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);
		mdm_usb1_usbdev = NULL;
		mdm_usb1_dev = NULL;
	}

	if (mdm_usb1_1_usbdev == udev) {
		pr_info("Remove device %d <%s %s>\n", udev->devnum,
			udev->manufacturer, udev->product);
		mdm_usb1_1_usbdev = NULL;
		mdm_usb1_1_dev = NULL;
	}
}

static int mdm_hsic_usb_notify(struct notifier_block *self, unsigned long action,
			void *blob)
{
	switch (action) {
	case USB_DEVICE_ADD:
		mdm_hsic_usb_device_add_handler(blob);
		break;
	case USB_DEVICE_REMOVE:
		mdm_hsic_usb_device_remove_handler(blob);
		break;
	}

	return NOTIFY_OK;
}

struct notifier_block mdm_hsic_usb_nb = {
	.notifier_call = mdm_hsic_usb_notify,
};

/***************************** PM notify *****************************/
static int mdm_hsic_power_pm_notifier_event(struct notifier_block *this,
					unsigned long event, void *ptr)
{
	unsigned long flags;

	pr_info("%s: event %ld\n", __func__, event);
	switch (event) {
	case PM_SUSPEND_PREPARE:
		pr_info("%s : PM_SUSPEND_PREPARE\n", __func__);
		spin_lock_irqsave(&mdm_hsic_wakeup_lock, flags);
		is_pm_suspended = true;
		spin_unlock_irqrestore(&mdm_hsic_wakeup_lock, flags);
		return NOTIFY_OK;

	case PM_POST_SUSPEND:
		pr_info("%s : PM_POST_SUSPEND\n", __func__);
		spin_lock_irqsave(&mdm_hsic_wakeup_lock, flags);
		is_pm_suspended = false;
		if (hsic_wakeup_pending)
		{
			mdm_hsic_wakeup();
		}
		spin_unlock_irqrestore(&mdm_hsic_wakeup_lock, flags);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

struct notifier_block mdm_hsic_power_pm_notifier = {
	.notifier_call = mdm_hsic_power_pm_notifier_event,
};

/************************ MDM2AP wakeup handle *************************/
static void mdm2ap_wakeup_thread(struct work_struct *work)
{
	int status;
	unsigned long start_time = jiffies;

	pr_info("%s: \n", __func__);

	usb_lock_device(mdm_usb1_1_usbdev);
	is_mdm_hsic_wakeup_in_progress = true;
	status = usb_autopm_get_interface(mdm2ap_wakeup_intf);
	if (status == 0) {
		pr_info("%s(%d) usb_autopm_get_interface OK %u ms\n", __func__, __LINE__, jiffies_to_msecs(jiffies-start_time));
		usb_autopm_put_interface(mdm2ap_wakeup_intf);
	}
	else {
		pr_info("%s(%d) error status:%d\n", __func__, __LINE__, status);
	}
	is_mdm_hsic_wakeup_in_progress = false;
	usb_unlock_device(mdm_usb1_1_usbdev);
}
static DECLARE_WORK(mdm2ap_wakeup_work, mdm2ap_wakeup_thread);


/*****************************  USB PM DBG *****************************/
static void mdm_hsic_pm_monitor_func(struct work_struct *work)
{
	extern int autosuspend_check(struct usb_device *udev);
	struct usb_device *udev = mdm_usb1_1_usbdev;
	struct device *dev = mdm_usb1_1_dev;
	struct usb_interface *intf;
	int status;

	if (udev == NULL || dev == NULL)
		return;

	pr_info("%s(%d)\n", __func__, __LINE__);

	mdm_hsic_print_pm_info();

	if (jiffies_to_msecs(jiffies - ACCESS_ONCE(udev->dev.power.last_busy)) > 3000 &&
		autosuspend_check(udev) == 0 &&
		dev->power.timer_expires == 0)
	{
		pr_info("%s(%d) activate timer by get put interface !!!\n", __func__, __LINE__);
		intf = usb_ifnum_to_if(udev, 0);

		if (intf) {
			usb_lock_device(udev);
			status = usb_autopm_get_interface(intf);
			if (status == 0) {
				pr_info("%s(%d) usb_autopm_get_interface OK\n", __func__, __LINE__);
				usb_autopm_put_interface(intf);
			}
			else {
				pr_info("%s(%d) error status:%d\n", __func__, __LINE__, status);
			}
			usb_unlock_device(udev);
		}
		else {
			pr_info("%s(%d) usb_ifnum_to_if error\n", __func__, __LINE__);
		}
	}
	schedule_delayed_work(&mdm_hsic_pm_monitor_delayed_work, msecs_to_jiffies(HSIC_PM_MON_DELAY));
}

static void mdm_hsic_print_interface_pm_info(struct usb_device *udev)
{
	struct usb_interface *intf;
	int			i = 0, n = 0;

	if (udev == NULL)
		return;

	dev_info(&udev->dev, "%s:\n", __func__);

	if (udev->actconfig) {
		n = udev->actconfig->desc.bNumInterfaces;
		for (i = 0; i <= n - 1; i++) {
			intf = udev->actconfig->interface[i];
			//--------------------------------------------------------
			#ifdef CONFIG_HTC_QCT_9K_MDM_HSIC_PM_DBG
			pr_info("  intf:%d last_busy_jiffies:%8lx busy_cnt:%10u data_busy_cnt:%10u pm_usage_cnt:%d usage_count:%d\n", i,
				ACCESS_ONCE(intf->last_busy_jiffies),
				intf->busy_cnt,
				intf->data_busy_cnt,
				atomic_read(&intf->pm_usage_cnt),
				atomic_read(&intf->dev.power.usage_count));
			#else
				#err_HTC_QCT_9K_MDM_HSIC_PM_DBG_not_defined
			#endif	//CONFIG_HTC_QCT_9K_MDM_HSIC_PM_DBG
			//--------------------------------------------------------
		}
	}
}

static void mdm_hsic_print_usb_dev_pm_info(struct usb_device *udev)
{
	if (udev != NULL)
	{
		struct device *dev = &(udev->dev);

		//--------------------------------------------------------
		#ifdef CONFIG_HTC_QCT_9K_MDM_HSIC_PM_DBG
		dev_info(&udev->dev, "[HSIC_PM_DBG] is_suspend:%d usage_count:%d last_busy:%8lx auto_suspend_timer_set:%d timer_expires:%8lx jiffies:%lx\n",
			udev->is_suspend,
			atomic_read(&(udev->dev.power.usage_count)),
			ACCESS_ONCE(udev->dev.power.last_busy),
			udev->auto_suspend_timer_set,
			dev->power.timer_expires,
			jiffies);
		mdm_hsic_print_interface_pm_info(udev);
		#endif	//CONFIG_HTC_QCT_9K_MDM_HSIC_PM_DBG
		//--------------------------------------------------------
	}
}

/***********************************************************************/

void mdm_hsic_phy_init(void)
{
	pr_info("%s\n", __func__);
}
EXPORT_SYMBOL_GPL(mdm_hsic_phy_init);

void mdm_hsic_phy_open(void)
{
	pr_info("%s\n", __func__);

	/* Init wake lock */
	MDM_HSIC_WLOCK_INIT(mdm_hsic_wlock);
	MDM_HSIC_WLOCK_LOCK(mdm_hsic_wlock);
}
EXPORT_SYMBOL_GPL(mdm_hsic_phy_open);

void mdm_hsic_phy_close(void)
{
	pr_info("%s\n", __func__);
	MDM_HSIC_WLOCK_DESTROY(mdm_hsic_wlock);
}
EXPORT_SYMBOL_GPL(mdm_hsic_phy_close);

void mdm_hsic_phy_pre_resume(void)
{
	unsigned long flags;

	if (get_radio_flag() & 0x0001)
		pr_info("%s\n", __func__);

	spin_lock_irqsave(&enable_wake_irq_lock, flags);
	htc_mdm2ap_wakeup_irq_enable_disable(false);
	spin_unlock_irqrestore(&enable_wake_irq_lock, flags);

	spin_lock_irqsave(&mdm_hsic_wakeup_lock, flags);
	hsic_wakeup_pending = false;
	spin_unlock_irqrestore(&mdm_hsic_wakeup_lock, flags);
}
EXPORT_SYMBOL_GPL(mdm_hsic_phy_pre_resume);

void mdm_hsic_phy_post_resume(void)
{
	if (get_radio_flag() & 0x0001)
		pr_info("%s\n", __func__);
}
EXPORT_SYMBOL_GPL(mdm_hsic_phy_post_resume);

void mdm_hsic_phy_pre_suspend(void)
{
	pr_info("%s\n", __func__);
}
EXPORT_SYMBOL_GPL(mdm_hsic_phy_pre_suspend);

void mdm_hsic_phy_post_suspend(void)
{
	unsigned long flags;

	if (get_radio_flag() & 0x0001)
		pr_info("%s\n", __func__);

	spin_lock_irqsave(&enable_wake_irq_lock, flags);
	htc_mdm2ap_wakeup_irq_enable_disable(true);
	spin_unlock_irqrestore(&enable_wake_irq_lock, flags);
}
EXPORT_SYMBOL_GPL(mdm_hsic_phy_post_suspend);

void mdm_hsic_phy_resume(void)
{
	if (get_radio_flag() & 0x0001)
		pr_info("%s\n", __func__);

	mdm_hsic_phy_resume_jiffies = jiffies;

	MDM_HSIC_WLOCK_LOCK(mdm_hsic_wlock);

	is_mdm_hsic_phy_suspended = false;

	schedule_delayed_work(&mdm_hsic_pm_monitor_delayed_work, msecs_to_jiffies(HSIC_PM_MON_DELAY));
}
EXPORT_SYMBOL_GPL(mdm_hsic_phy_resume);

void mdm_hsic_phy_suspend(void)
{
	unsigned long  elapsed_ms = 0;
	static unsigned int suspend_cnt = 0;

	if (get_radio_flag() & 0x0001)
		pr_info("%s\n", __func__);

	if (mdm_hsic_phy_resume_jiffies != 0) {
		elapsed_ms = jiffies_to_msecs(jiffies - mdm_hsic_phy_resume_jiffies);
		mdm_hsic_phy_active_total_ms += elapsed_ms ;
	}

	cancel_delayed_work(&mdm_hsic_pm_monitor_delayed_work);
	suspend_cnt++;
	if (elapsed_ms > 30000 || suspend_cnt >= 10) {
		suspend_cnt = 0;
		pr_info("%s elapsed_ms:%lu ms\n", __func__, elapsed_ms);
		mdm_hsic_print_pm_info();
	}

	MDM_HSIC_WLOCK_UNLOCK(mdm_hsic_wlock);

	is_mdm_hsic_phy_suspended = true;
}
EXPORT_SYMBOL_GPL(mdm_hsic_phy_suspend);

int mdm_hsic_driver_suspend(void)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&mdm_hsic_wakeup_lock, flags);
	if (hsic_wakeup_pending)
		ret = -EBUSY;
	spin_unlock_irqrestore(&mdm_hsic_wakeup_lock, flags);

	pr_info("%s ret:%d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(mdm_hsic_driver_suspend);

int mdm_hsic_driver_resume(void)
{
	pr_info("%s\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(mdm_hsic_driver_resume);

int mdm_hsic_driver_suspend_noirq(void)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&mdm_hsic_wakeup_lock, flags);
	if (hsic_wakeup_pending)
		ret = -EBUSY;
	spin_unlock_irqrestore(&mdm_hsic_wakeup_lock, flags);

	pr_info("%s ret:%d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(mdm_hsic_driver_suspend_noirq);

int mdm_hsic_driver_resume_noirq(void)
{
	pr_info("%s\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(mdm_hsic_driver_resume_noirq);

bool mdm_hsic_wakeup(void)
{
	bool ret = true;

	//lock wake_lock
	MDM_HSIC_WLOCK_LOCK(mdm_hsic_wlock);

	if (NULL == mdm_usb1_1_usbdev) {
		pr_info("%s !usbdev\n", __func__);
		ret = false;
		goto out;
	}

	mdm2ap_wakeup_intf = usb_ifnum_to_if(mdm_usb1_1_usbdev, 0);
	if(NULL == mdm2ap_wakeup_intf){
		pr_info("%s !intf\n", __func__);
		ret = false;
		goto out;
	}

	//if not in suspended, runtime resume usb interface 0
	if (!is_pm_suspended)
	{
		queue_work(mdm2ap_wakeup_queue, &mdm2ap_wakeup_work);
	}
	else
	{
		if (get_radio_flag() & 0x0008)
			pr_info("%s is_pm_suspended:%d\n", __func__, is_pm_suspended);
		hsic_wakeup_pending = true;
	}
out:
	if (get_radio_flag() & 0x0008)
		pr_info("%s ret:%d\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(mdm_hsic_wakeup);


int mdm_hsic_init(void)
{
	extern void ehci_qct_mdm_init(void);
	int ret = 0;

	//MISC.
	is_modem_qct_mdm9k = true;
	ehci_qct_mdm_init();

	//USB notify
	usb_register_notify(&mdm_hsic_usb_nb);
	register_pm_notifier(&mdm_hsic_power_pm_notifier);

	//USB PM DBG
	INIT_DELAYED_WORK(&mdm_hsic_pm_monitor_delayed_work, mdm_hsic_pm_monitor_func);

	//MDM2AP WAKEUP
	mdm2ap_wakeup_queue = create_singlethread_workqueue("mdm2ap_wakeup_queue");
	if (!mdm2ap_wakeup_queue) {
		pr_err("%s: could not create workqueue. All mdm "
				"functionality will be disabled\n",
			__func__);
		ret = -ENOMEM;
		goto out;
	}

out:
	return ret;
}
EXPORT_SYMBOL_GPL(mdm_hsic_init);

bool Modem_is_QCT_MDM9K(void)
{
	return is_modem_qct_mdm9k;
}
EXPORT_SYMBOL_GPL(Modem_is_QCT_MDM9K);

bool mdm_hsic_is_dev_allowed_skip_resume(void)
{
	bool ret = true;

	if (hsic_wakeup_pending)
	{
		pr_err("%s: hsic_wakeup_pending:%d", __func__, hsic_wakeup_pending);
		ret = false;
	}

	if ((mdm_usb1_1_dev->power.runtime_status != RPM_SUSPENDED) ||
		(mdm_usb1_dev->power.runtime_status != RPM_SUSPENDED) )
	{
		dev_err(mdm_usb1_1_dev, "%s runtime_status:%d disable_depth:%d\n", __func__, mdm_usb1_1_dev->power.runtime_status, mdm_usb1_1_dev->power.disable_depth);
		dev_err(mdm_usb1_dev, "%s runtime_status:%d disable_depth:%d\n", __func__, mdm_usb1_dev->power.runtime_status, mdm_usb1_dev->power.disable_depth);
		ret = false;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(mdm_hsic_is_dev_allowed_skip_resume);

void mdm_hsic_disable_auto_suspend(void)
{
	struct usb_device *udev = mdm_usb1_1_usbdev;

	if (udev)
		usb_disable_autosuspend(udev);
}
EXPORT_SYMBOL_GPL(mdm_hsic_disable_auto_suspend);

void mdm_hsic_print_pm_info(void)
{
	pr_info("\n");

	mdm_hsic_print_usb_dev_pm_info(mdm_usb1_1_usbdev);
	mdm_hsic_print_usb_dev_pm_info(mdm_usb1_usbdev);

	pr_info("\n");
}
EXPORT_SYMBOL_GPL(mdm_hsic_print_pm_info);

