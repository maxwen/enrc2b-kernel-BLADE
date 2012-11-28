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
#include <linux/regulator/consumer.h>
//#include <linux/mfd/pmic8058.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/mdm2.h>
#include <mach/restart.h>
//#include <mach/subsystem_notif.h>
//#include <mach/subsystem_restart.h>
#include <linux/msm_charm.h>
#include <linux/delay.h>
//#include "msm_watchdog.h"
#include "devices.h"
#include "clock.h"
#include "mdm_private.h"
#include "gpio-names.h"
#include <mach/usb_phy.h>

/* HTC added start */
#include <linux/proc_fs.h>
#include <mach/board_htc.h>

#if defined(pr_debug)
#undef pr_debug
#endif
#define pr_debug(x...) do {				\
		if (mdm_debug_on) \
			printk(KERN_INFO "[MDM] "x);		\
		else									\
			printk(KERN_DEBUG "[MDM] "x);		\
	} while (0)

#if defined(pr_info)
#undef pr_info
#endif
#define pr_info(x...) do {				\
			printk(KERN_INFO "[MDM] "x);		\
	} while (0)

#if defined(pr_err)
#undef pr_err
#endif
#define pr_err(x...) do {				\
			printk(KERN_ERR "[MDM] "x);		\
	} while (0)
/* HTC added end */

#define MDM_MODEM_TIMEOUT	10000	/* HTC changed from 6000 to 10000(ms) */
#define MDM_HOLD_TIME		4000
#define MDM_MODEM_DELTA		100

static struct regulator *enterprise_dsi_reg = NULL;//for avdd_csi_dsi


static int mdm_debug_on;
static int first_power_on = 1;
static int hsic_peripheral_status = 1;
static DEFINE_MUTEX(hsic_status_lock);

/* HTC added start */

//++SSD_RIL: Mars_Lin@20120606: For GPIO setting
extern const int gpio_to_pingroup[TEGRA_MAX_GPIO];
//--SSD_RIL: Mars_Lin@20120606

static int mdm9k_status;

static int mdm_loaded_status_proc(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	int ret;
	char *p = page;

	if (off > 0) {
		ret = 0;
	} else {
		p += sprintf(p, "%d\n", mdm9k_status);
		ret = p - page;
	}

	return ret;
}

static void mdm_loaded_info(void)
{
	struct proc_dir_entry *entry = NULL;

	mdm9k_status = 0;
	entry = create_proc_read_entry("mdm9k_status", 0, NULL, mdm_loaded_status_proc, NULL);
}

//++SSD_RIL: Mars_Lin@20120606: print gpio information
static void mdm2_dump_mdm_related_gpio(struct mdm_modem_drv *mdm_drv, char * comment)
{
	if ( mdm_drv == NULL ) {
		return;
	}

	pr_info("%s:[%s]\n	[%s(%d):%d]\n	[%s(%d):%d]\n	[%s(%d):%d]\n	[%s(%d):%d]\n	[%s(%d):%d]\n	[%s(%d):%d]\n	[%s(%d):%d]\n	[%s(%d):%d]\n	[%s(%d):%d] [%s(%d):%d]\n", __func__, comment,
		"MDM2AP_ERRFATAL", mdm_drv->mdm2ap_errfatal_gpio, gpio_get_value(mdm_drv->mdm2ap_errfatal_gpio),
		"AP2MDM_ERRFATAL", mdm_drv->ap2mdm_errfatal_gpio, gpio_get_value(mdm_drv->ap2mdm_errfatal_gpio),
		"MDM2AP_STATUS", mdm_drv->mdm2ap_status_gpio, gpio_get_value(mdm_drv->mdm2ap_status_gpio),
		"AP2MDM_STATUS", mdm_drv->ap2mdm_status_gpio, gpio_get_value(mdm_drv->ap2mdm_status_gpio),
		"AP2MDM_WAKEUP", mdm_drv->ap2mdm_wakeup_gpio, gpio_get_value(mdm_drv->ap2mdm_wakeup_gpio),
		"AP2MDM_PMIC_RESET_N", mdm_drv->ap2mdm_pmic_reset_n_gpio, gpio_get_value(mdm_drv->ap2mdm_pmic_reset_n_gpio),
		"AP2MDM_KPDPWR_N", mdm_drv->ap2mdm_kpdpwr_n_gpio, gpio_get_value(mdm_drv->ap2mdm_kpdpwr_n_gpio),
		"MDM2AP_HSIC_READY", mdm_drv->mdm2ap_hsic_ready_gpio, gpio_get_value(mdm_drv->mdm2ap_hsic_ready_gpio),
		"AP2MDM_SW_BC5", mdm_drv->ap2mdm_sw_bc5_gpio, gpio_get_value(mdm_drv->ap2mdm_sw_bc5_gpio),
		"MDM2AP_WAKEUP", mdm_drv->mdm2ap_wakeup_gpio, gpio_get_value(mdm_drv->mdm2ap_wakeup_gpio)
	);
	return;
}
//--SSD_RIL: Mars_Lin@20120606

/* HTC added end */


/*HTC++*/
int mdm_enable_avdd_dsi_csi_power(void)
{
	int ret=0;
	pr_info("+mdm_enable_avdd_dsi_csi_power\n");

	if (enterprise_dsi_reg == NULL) {
		 pr_info("mdm_enable_avdd_dsi_csi_power:avdd_dsi_csi\n");
		enterprise_dsi_reg = regulator_get(NULL, "avdd_dsi_csi");
		if (IS_ERR_OR_NULL(enterprise_dsi_reg)) {
			pr_err("dsi: Could not get regulator avdd_dsi_csi\n");
				enterprise_dsi_reg = NULL;
				return PTR_ERR(enterprise_dsi_reg);
		}
	}
	ret = regulator_enable(enterprise_dsi_reg);
	if (ret < 0) {
		printk(KERN_ERR
			"DSI regulator avdd_dsi_csi couldn't be enabled ret:%d\n",ret);
		return ret;
	}
	return ret;
}


int mdm_disable_avdd_dsi_csi_power(void)
{
	int ret=0;
	pr_info("mdm_disable_avdd_dsi_csi_power\n");
	if (enterprise_dsi_reg != NULL) {
		 pr_info("mdm_disable_avdd_dsi_csi_power:avdd_dsi_csi\n");
		enterprise_dsi_reg = regulator_get(NULL, "avdd_dsi_csi");
		if (IS_ERR_OR_NULL(enterprise_dsi_reg)) {
			pr_err("dsi: Could not get regulator avdd_dsi_csi\n");
				enterprise_dsi_reg = NULL;
				return PTR_ERR(enterprise_dsi_reg);
		}
		//++SSD_RIL@20120718: Klockwork#798: check enterprise_dsi_reg before call regulator_disable
		ret = regulator_disable(enterprise_dsi_reg);
		if (ret < 0) {
			printk(KERN_ERR
				"DSI regulator avdd_dsi_csi couldn't be disabled ret:%d\n",ret);
			return ret;
		}
		enterprise_dsi_reg=NULL;
	}

	return ret;
}
void mdm_peripheral_connect(struct mdm_modem_drv *mdm_drv)
{
		
	mutex_lock(&hsic_status_lock);
	if (hsic_peripheral_status)
		goto out;
	if (mdm_drv->pdata->peripheral_platform_device)
		platform_device_add(mdm_drv->pdata->peripheral_platform_device);
	hsic_peripheral_status = 1;
	
	//mdm_enable_avdd_dsi_csi_power();	//+Sophia
out:
	mutex_unlock(&hsic_status_lock);
}

void mdm_peripheral_disconnect(struct mdm_modem_drv *mdm_drv)
{
	mutex_lock(&hsic_status_lock);
	//mdm_disable_avdd_dsi_csi_power(); //+Sophia
	
	if (!hsic_peripheral_status)
		goto out;

//++SSD_RIL:Mars_Lin: Don't call platform_device_del if modem not power on
	if ( first_power_on ) {
		pr_info("%s: don't call platform_device_del if modem not power on\n", __func__);
		goto out;
	}
//--SSD_RIL

	if (mdm_drv->pdata->peripheral_platform_device)
		platform_device_del(mdm_drv->pdata->peripheral_platform_device);
	hsic_peripheral_status = 0;
out:
	mutex_unlock(&hsic_status_lock);
}

static void power_on_mdm(struct mdm_modem_drv *mdm_drv)
{
	int mfg_mode;
	pr_info("+power_on_mdm\n");

	mfg_mode = board_mfg_mode();

	if (mfg_mode == BOARD_MFG_MODE_OFFMODE_CHARGING) {
		pr_info("-power_on_mdm: offmode charging\n");
		return;
	}

	if (first_power_on) {
		pr_info("+power_on_mdm:add tegra_ehci2_device\n");
		platform_device_register(&tegra_ehci2_device);
	}
	else {
		pr_info("+power_on_mdm:mdm_peripheral_disconnect\n");
		mdm_peripheral_disconnect(mdm_drv);
		pr_info("+power_on_mdm:mdm_peripheral_connect\n");
		mdm_peripheral_connect(mdm_drv);
	}

	//++SSD_RIL: Mars_Lin@20120606: Print related GPIO setting
	mdm2_dump_mdm_related_gpio( mdm_drv, "Before power on" );
	//--SSD_RIL

	gpio_direction_output(mdm_drv->ap2mdm_wakeup_gpio, 0);
	tegra_gpio_enable(mdm_drv->ap2mdm_wakeup_gpio);
	gpio_set_value(mdm_drv->ap2mdm_wakeup_gpio, 0);

	//++SSD_RIL: Mars_Lin@20120606:Change J0 to TEGRA_PUPD_PULL_DOWN
	tegra_pinmux_set_pullupdown(gpio_to_pingroup[mdm_drv->mdm2ap_hsic_ready_gpio], TEGRA_PUPD_PULL_DOWN);
	//--SSD_RIL

	//++SSD_RIL: Mars_Lin@20120709: Set mdm_ready to 0 before power on mdm
	mdm_drv->mdm_ready = 0;
	//--SSD_RIL

	/* Pull RESET gpio low and wait for it to settle. */
	pr_info("%s: Pulling RESET gpio low\n", __func__);

	gpio_direction_output(mdm_drv->ap2mdm_pmic_reset_n_gpio, 0);	//TEGRA_GPIO_PP7
	tegra_gpio_enable(mdm_drv->ap2mdm_pmic_reset_n_gpio);
	gpio_set_value(mdm_drv->ap2mdm_pmic_reset_n_gpio, 0);

	usleep_range(5000, 10000);

	/* Deassert RESET first and wait for it to settle. */
	pr_info("%s: Pulling RESET gpio high\n", __func__);

	gpio_direction_output(mdm_drv->ap2mdm_pmic_reset_n_gpio, 1);
	tegra_gpio_enable(mdm_drv->ap2mdm_pmic_reset_n_gpio);
	gpio_set_value(mdm_drv->ap2mdm_pmic_reset_n_gpio, 1);

	msleep(40);

	//++SSD_RIL:20120705: pull ap2mdm_status GPIO to high
	gpio_direction_output(mdm_drv->ap2mdm_status_gpio, 1);
	//--SSD_RIL

	//++SSD_RIL: Mars_Lin@20120608: enable irq after power on mdm
	mdm_common_enable_irq(mdm_drv, MDM_MODEM_IRQ_ENABLE);
	//--SSD_RIL

	//++SSD_RIL: Mars_Lin@20120606:Change J0 to NO_PULL
	tegra_pinmux_set_pullupdown(gpio_to_pingroup[mdm_drv->mdm2ap_hsic_ready_gpio], TEGRA_PUPD_NORMAL);
	//--SSD_RIL

	msleep(40);

	/* Pull PWR gpio high and wait for it to settle, but only
	 * the first time the mdm is powered up.
	 * Some targets do not use ap2mdm_kpdpwr_n_gpio.
	 */
	if (first_power_on) {
		if (mdm_drv->ap2mdm_kpdpwr_n_gpio > 0) {
			int ret2;

			pr_info("%s: Powering on mdm modem:mdm_drv->ap2mdm_kpdpwr_n_gpio = (%d)\n", __func__,mdm_drv->ap2mdm_kpdpwr_n_gpio);
			ret2 = gpio_direction_output(mdm_drv->ap2mdm_kpdpwr_n_gpio, 1);
			if (ret2 < 0) {
				pr_err("%s: gpio_direction_output failed %d\n", __func__, ret2);
				gpio_free(mdm_drv->ap2mdm_kpdpwr_n_gpio);
				return;
			}
			tegra_gpio_enable(mdm_drv->ap2mdm_kpdpwr_n_gpio);
			gpio_set_value(mdm_drv->ap2mdm_kpdpwr_n_gpio, 1);

			udelay(1000);
		}
		first_power_on = 0;
	} else {
		//pr_info("+power_on_mdm:mdm_peripheral_connect\n");
		//mdm_peripheral_connect(mdm_drv);
	}
	msleep(200);
	pr_info("-power_on_mdm\n");
}

static void power_down_mdm(struct mdm_modem_drv *mdm_drv)
{
	//++HTC
	extern bool is_in_subsystem_restart;
	//--HTC
	int i;
	int mfg_mode;

	pr_info("+power_down_mdm\n");

	mfg_mode = board_mfg_mode();

	if (mfg_mode == BOARD_MFG_MODE_OFFMODE_CHARGING) {
		pr_info("-power_down_mdm: offmode charging\n");
		return;
	}

	//++SSD_RIL: Mars_Lin@20120608: disable irq before power off mdm
	mdm_common_enable_irq(mdm_drv, MDM_MODEM_IRQ_DISABLE);
	//--SSD_RIL

	/* APQ8064 only uses one pin to control pon and reset.
	 * If this pin is down over 300ms, memory data will loss.
	 * Currently sbl1 will help pull this pin up, and need 120~140ms.
	 * To decrease down time, we do not shut down MDM here and last until 8k PS_HOLD is pulled.
	 */
#if 1	//HTC
if (is_in_subsystem_restart && (get_radio_flag() & 0x8))
{
	pr_info("%s: Need to capture MDM RAM Dump, don't Pulling RESET gpio LOW here to prevent MDM memory data loss\n", __func__);
}
else
{
	pr_info("%s: Pulling RESET gpio LOW\n", __func__);
	gpio_direction_output(mdm_drv->ap2mdm_pmic_reset_n_gpio, 0);

	for (i = MDM_HOLD_TIME; i > 0; i -= MDM_MODEM_DELTA) {
		//pet_watchdog();
		msleep(MDM_MODEM_DELTA);
	}
}
#endif

	if (mdm_drv->ap2mdm_kpdpwr_n_gpio > 0)
		gpio_direction_output(mdm_drv->ap2mdm_kpdpwr_n_gpio, 0);
	//mdm_peripheral_disconnect(mdm_drv);
	
	pr_info("-power_down_mdm\n");
}

static void debug_state_changed(int value)
{
	mdm_debug_on = value;
}


static void mdm_status_changed(struct mdm_modem_drv *mdm_drv, int value)
{
	pr_debug("%s: value:%d\n", __func__, value);

	if (value && mdm_drv->mdm_ready) {//Sophia:0510-check if MD ready
		/*
		msleep(2000);
		pr_info("mdm_status_changed: unregister:tegra_ehci2_device\n");
		mdm_peripheral_disconnect(mdm_drv);		
		pr_info("mdm_status_changed: register:tegra_ehci2_device\n");
		mdm_peripheral_connect(mdm_drv);
		gpio_direction_output(TEGRA_GPIO_PC6, 1);   //TEGRA_GPIO_PC6
		tegra_gpio_enable(TEGRA_GPIO_PC6);
		gpio_set_value(TEGRA_GPIO_PC6, 1);
		*/

		pr_info("mdm_status_changed: unregister:tegra_ehci2_device\n");
		mdm_peripheral_disconnect(mdm_drv);
		pr_info("mdm_status_changed: register:tegra_ehci2_device\n");
		pr_info("mdm_status_changed: +mdm_peripheral_connect\n");
		mdm_peripheral_connect(mdm_drv);
		pr_info("mdm_status_changed: -mdm_peripheral_connect\n");
		//TEGRA_GPIO_PC6
		gpio_direction_output(mdm_drv->ap2mdm_wakeup_gpio, 1);
		tegra_gpio_enable(mdm_drv->ap2mdm_wakeup_gpio);
		gpio_set_value(mdm_drv->ap2mdm_wakeup_gpio, 1);

		/* HTC added start */
		mdm9k_status = 1;
		/* HTC added end */
	}
}

static struct mdm_ops mdm_cb = {
	.power_on_mdm_cb = power_on_mdm,
	.power_down_mdm_cb = power_down_mdm,
	.debug_state_changed_cb = debug_state_changed,
	.status_cb = mdm_status_changed,
};

static int __init mdm_modem_probe(struct platform_device *pdev)
{
		pr_info("mdm_modem_probe\n");
		
	return mdm_common_create(pdev, &mdm_cb);
}

static int __devexit mdm_modem_remove(struct platform_device *pdev)
{
	return mdm_common_modem_remove(pdev);
}

static void mdm_modem_shutdown(struct platform_device *pdev)
{
	mdm_common_modem_shutdown(pdev);
}

//++SSD_RIL: 20120704: add suspend/resume function to enable/disable irq wake
#ifdef CONFIG_PM

extern int mdm_hsic_driver_suspend(void);
extern int mdm_hsic_driver_resume(void);
extern int mdm_hsic_driver_suspend_noirq(void);
extern int mdm_hsic_driver_resume_noirq(void);

static int mdm_modem_driver_suspend(struct device *pdev)
{
	int ret = 0;

	pr_info("%s+\n", __func__);

	mdm_common_modem_suspend();
	ret = mdm_hsic_driver_suspend();

	pr_info("%s- ret:%d\n", __func__, ret);

	return 0;
}

static int mdm_modem_driver_resume(struct device *pdev)
{
	int ret = 0;

	pr_info("%s+\n", __func__);

	mdm_common_modem_resume();

	pr_info("%s-\n", __func__);
	return ret;
}

static int mdm_modem_driver_suspend_noirq(struct device *pdev)
{
	int ret = 0;

	pr_info("%s+\n", __func__);

	ret = mdm_hsic_driver_suspend_noirq();

	pr_info("%s- ret:%d\n", __func__, ret);
	return ret;
}

static int mdm_modem_driver_resume_noirq(struct device *pdev)
{
	int ret = 0;

	pr_info("%s+\n", __func__);

	mdm_hsic_driver_resume_noirq();

	pr_info("%s-\n", __func__);
	return ret;
}

#endif
//--SSD_RIL: 20120704

static const struct dev_pm_ops mdm_modem_driver_power_dev_pm_ops = {
	.suspend_noirq = mdm_modem_driver_suspend_noirq,
	.resume_noirq = mdm_modem_driver_resume_noirq,
	.suspend = mdm_modem_driver_suspend,
	.resume = mdm_modem_driver_resume,
};

static struct platform_driver mdm_modem_driver = {
	.remove         = mdm_modem_remove,
	.shutdown	= mdm_modem_shutdown,
	.driver         = {
		.name = "mdm2_modem",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &mdm_modem_driver_power_dev_pm_ops,
#endif
	},


};

static int __init mdm_modem_init(void)
{
	pr_info("mdm_modem_init\n");
	/* HTC added start */
	mdm_loaded_info();
	/* HTC added end */
	return platform_driver_probe(&mdm_modem_driver, mdm_modem_probe);
}

static void __exit mdm_modem_exit(void)
{
	platform_driver_unregister(&mdm_modem_driver);
}

module_init(mdm_modem_init);
module_exit(mdm_modem_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("mdm modem driver");
MODULE_VERSION("2.0");
MODULE_ALIAS("mdm_modem");
