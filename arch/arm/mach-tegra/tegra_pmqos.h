/*
 * arch/arm/mach-tegra/tegra_pmqos.h
 *
 * Copyright (C) 2012 Paul Reioux (aka Faux123)
 *
 * Author:
 *	faux123 <reioux@gmail.com>
 *
 * History:
 *      -original version (Paul Reioux)
 *      -cleaned since oc was reworked (Dennis Rassmann)
 *      -added comment for T3_VARIANT_BOOST (Dennis Rassmann)
 *      -adapted for grouper (Dennis Rassmann)
 *      -removed distinction between 0boost and xboost
 *      -minimized version
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* in kHz */
#define T3_CPU_FREQ_MAX_0		1700000
#define T3_CPU_FREQ_MAX			1600000
#define T3_CPU_FREQ_MAX_OC		1700000
/* any caps will be respected */
#define T3_CPU_FREQ_BOOST		1600000
#define T3_CPU_MIN_FREQ     	51000
#define T3_SUSPEND_FREQ     	475000

// used for governors ideal or idle freq
#define GOV_IDLE_FREQ     		475000

// sysfs to change available
#define SUSPEND_CPU_NUM_MAX		2

// f_mtp.c
#define MTP_CPU_FREQ_MIN 1150000
#define MTP_ONLINE_CPUS_MIN 2

// tlv320aic3008.c - sysfs to change available
#define AUD_CPU_FREQ_MIN 102000

// android.c
#define USB_TP_CPU_FREQ_MIN 475000

// tegra_udc.h 
#define TEGRA_GADGET_CPU_FREQ_MIN 475000

// tegra_hsuart.c - not automatic must be enabled via sysfs
// we dont need that on enrc2b
#define TI_A2DP_CPU_FREQ_MIN 102000

// tegra_hsuart_brcm.c - not automatic must be enabled via sysfs
// sysfs to change available
#define A2DP_CPU_FREQ_MIN 204000
#define OPP_CPU_FREQ_MIN 475000

// wl_android.c
#define WIFI_CPU_FREQ_MIN 1150000
#define WIFI_ONLINE_CPUS_MIN 2

// usbnet.c
#define USBNET_CPU_FREQ_MIN 475000
#define USBNET_ONLINE_CPUS_MIN 2

extern unsigned int tegra_pmqos_cpu_freq_limits[];
extern unsigned int tegra_pmqos_cpu_freq_limits_min[];
extern unsigned int tegra_cpu_freq_max(unsigned int cpu);
extern unsigned int tegra_get_suspend_boost_freq(void);
extern unsigned int tegra_lpmode_freq_max(void);

