/*
 * arch/arm/mach-tegra/baseband-qct-mdm-hsic.h
 *
 */

#ifndef BASEBAND_QCT_MDM_HSIC_H
#define BASEBAND_QCT_MDM_HSIC_H

#include <linux/pm.h>
#include <linux/suspend.h>

void mdm_hsic_phy_init(void);
void mdm_hsic_phy_open(void);
void mdm_hsic_phy_close(void);
void mdm_hsic_phy_pre_resume(void);
void mdm_hsic_phy_post_resume(void);
void mdm_hsic_phy_pre_suspend(void);
void mdm_hsic_phy_post_suspend(void);
void mdm_hsic_phy_suspend(void);
void mdm_hsic_phy_resume(void);

int mdm_hsic_driver_suspend(void);
int mdm_hsic_driver_resume(void);
int mdm_hsic_driver_suspend_noirq(void);
int mdm_hsic_driver_resume_noirq(void);

bool mdm_hsic_wakeup(void);
int mdm_hsic_init(void);
bool Modem_is_QCT_MDM9K(void);
bool mdm_hsic_is_dev_allowed_skip_resume(void);
void mdm_hsic_disable_auto_suspend(void);
void mdm_hsic_print_pm_info(void);

#endif  /* BASEBAND_QCT_MDM_HSIC_H */
