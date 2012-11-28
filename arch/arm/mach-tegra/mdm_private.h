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
 */

#ifndef _ARCH_ARM_MACH_MSM_MDM_PRIVATE_H
#define _ARCH_ARM_MACH_MSM_MDM_PRIVATE_H

struct mdm_modem_drv;

//++SSD_RIL: Mars_Lin@20120608: Add MDM_MODEM_IRQ_STATE for enable/disable mdm irq
typedef enum {
	MDM_MODEM_IRQ_REQ = 0,
	MDM_MODEM_IRQ_ENABLE,
	MDM_MODEM_IRQ_DISABLE,
	MDM_MODEM_IRQ_WAKE_ENABLE,
	MDM_MODEM_IRQ_WAKE_DISABLE,
	MDM_MODEM_IRQ_FREE,
} MDM_MODEM_IRQ_STATE;
//--SSD_RIL

struct mdm_ops {
	void (*power_on_mdm_cb)(struct mdm_modem_drv *mdm_drv);
	void (*normal_boot_done_cb)(struct mdm_modem_drv *mdm_drv);
	void (*power_down_mdm_cb)(struct mdm_modem_drv *mdm_drv);
	void (*debug_state_changed_cb)(int value);
	void (*status_cb)(struct mdm_modem_drv *mdm_drv, int value);
};

/* Private mdm2 data structure */
struct mdm_modem_drv {
	unsigned mdm2ap_errfatal_gpio;
	unsigned ap2mdm_errfatal_gpio;
	unsigned mdm2ap_status_gpio;
	unsigned ap2mdm_status_gpio;
	unsigned mdm2ap_wakeup_gpio;
	unsigned ap2mdm_wakeup_gpio;
	unsigned ap2mdm_pmic_reset_n_gpio;
	unsigned ap2mdm_kpdpwr_n_gpio;
	unsigned mdm2ap_hsic_ready_gpio;
	unsigned ap2mdm_sw_bc5_gpio;	//htc
	int 	 ap2mdm_sw_bc5_status;	//htc
//++SSD_RIL:Mars_Lin@20120613: Add PM4 for operaul mdm power on
	unsigned v_dcin_modem_en_gpio;
//--SSD_RIL

	int mdm_errfatal_irq;
	int mdm_status_irq;
	int mdm_wakeup_irq;
	int ap2mdm_sw_bc5_irq;	//htc
	int mdm_ready;
	int mdm_boot_status;
	int mdm_ram_dump_status;
	enum charm_boot_type boot_type;
	int mdm_debug_on;

	struct mdm_ops *ops;
	struct mdm_platform_data *pdata;

	//++SSD_RIL:Mars_lin@20120608: add mdm_irq_enabled
	int mdm_irq_enabled;
	//--SSD_RIL
	//++SSD_RIL:20120705: Export all related GPIO
	int mdm_gpio_exported;
	//--SSD_RIL
};

int mdm_common_create(struct platform_device  *pdev,
					  struct mdm_ops *mdm_cb);
int mdm_common_modem_remove(struct platform_device *pdev);
void mdm_common_modem_shutdown(struct platform_device *pdev);
void mdm_common_set_debug_state(int value);

int mdm_common_check_final_efs_wait(void);		/* HTC added */

//++SSD_RIL: Mars_Lin@20120608: Add mdm_common_enable_irq for enable/disable mdm irq
void mdm_common_enable_irq( struct mdm_modem_drv *mdm_drv, int enable );
//--SSD_RIL

//++SSD_RIL: 20120704: add suspend/resume function to enable/disable irq wake
void mdm_common_modem_suspend(void);

void mdm_common_modem_resume(void);
//--SSD_RIL: 20120704

#endif

