/*
 * Copyright (C) 2010 HTC, Inc.
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
#ifndef __ASM_ARCH_MSM_HTC_USB_H
#define __ASM_ARCH_MSM_HTC_USB_H

#ifdef CONFIG_ARCH_QSD8X50
void msm_hsusb_8x50_phy_reset(void);
#endif

#ifdef ERROR
#undef ERROR
#endif
#include <linux/usb/android_composite.h>
#include <linux/usb/f_accessory.h>

#ifdef CONFIG_USB_ANDROID_USBNET
static char *usb_functions_usbnet[] = {
	"usbnet",
};

static char *usb_functions_usbnet_adb[] = {
	"usbnet",
	"adb",
};
#endif

static char *usb_functions_ums[] = {
	"mass_storage",
};

static char *usb_functions_adb[] = {
	"mass_storage",
	"adb",
};

static char *usb_functions_mtp_ums_adb[] = {
	"mtp",
	"adb",
	"mass_storage",
};

static char *usb_functions_mtp_ums[] = {
	"mtp",
	"mass_storage",
};

#ifdef CONFIG_USB_ANDROID_ECM
static char *usb_functions_ecm[] = {
	"cdc_ethernet",
};
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
static char *usb_functions_rndis[] = {
	"rndis",
};
static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};
static char *usb_functions_rndis_mtp_ums[] = {
	"rndis",
	"mtp",
	"mass_storage",
};
#if 0
static char *usb_functions_rndis_mtp_ums_diag[] = {
	"rndis",
	"mtp",
	"mass_storage",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
#endif
static char *usb_functions_rndis_mtp_ums_adb[] = {
	"rndis",
	"mtp",
	"adb",
	"mass_storage",
};
#if 0
static char *usb_functions_rndis_mtp_ums_adb_diag[] = {
	"rndis",
	"mtp",
	"adb",
	"mass_storage",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
#endif
#if defined(CONFIG_USB_ANDROID_DIAG) || defined(CONFIG_USB_ANDROID_QCT_DIAG)
static char *usb_functions_rndis_diag[] = {
	"rndis",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_adb_diag[] = {
	"rndis",
	"adb",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
#endif
#endif
static char *usb_functions_accessory[] = { "accessory" };
static char *usb_functions_accessory_adb[] = { "accessory", "adb" };
static char *usb_functions_audio_source[] = { "audio_source" };
static char *usb_functions_audio_source_adb[] = { "audio_source", "adb" };
static char *usb_functions_accessory_audio_source[] = { "accessory", "audio_source" };
static char *usb_functions_accessory_audio_source_adb[] = { "accessory", "audio_source", "adb" };


static char *usb_functions_projector[] = {
	"mass_storage",
	"projector",
};
static char *usb_functions_adb_projector[] = {
	"mass_storage",
	"adb",
	"projector",
};
static char *usb_functions_mtp_ums_projector[] = {
	"mtp",
	"mass_storage",
	"projector",
};
static char *usb_functions_mtp_ums_adb_projector[] = {
	"mtp",
	"adb",
	"mass_storage",
	"projector",
};
#if defined(CONFIG_USB_ANDROID_DIAG) || defined(CONFIG_USB_ANDROID_QCT_DIAG)
static char *usb_function_adb_diag_projector[] = {
	"mass_storage",
	"adb",
	"diag",
	"projector",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_function_mtp_ums_adb_diag_projector[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"projector",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
#ifdef CONFIG_USB_ANDROID_SERIAL
static char *usb_function_adb_diag_modem_projector[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
	"projector",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_function_mtp_ums_adb_diag_modem_projector[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"projector",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
#endif
#endif

#if defined(CONFIG_USB_ANDROID_MTP36) || defined(CONFIG_USB_ANDROID_MTP)
static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};
#endif

#if defined(CONFIG_USB_ANDROID_MTP36) && defined(CONFIG_USB_ANDROID_MTP)
static char *usb_functions_mtp36[] = {
	"mtp36",
};

static char *usb_functions_mtp36_adb[] = {
	"mtp36",
	"adb",
};
#endif

#if defined(CONFIG_USB_ANDROID_DIAG) || defined(CONFIG_USB_ANDROID_QCT_DIAG)
static char *usb_functions_diag[] = {
	"mass_storage",
	"diag",
};
static char *usb_functions_diag2[] = {
	"mass_storage",
	"diag",
	"diag_mdm",
};
static char *usb_functions_adb_diag[] = {
	"mass_storage",
	"adb",
	"diag",
};
static char *usb_functions_adb_diag2[] = {
	"mass_storage",
	"adb",
	"diag",
	"diag_mdm",
};
static char *usb_functions_mtp_ums_adb_diag[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
};
static char *usb_functions_mtp_ums_diag[] = {
	"mtp",
	"mass_storage",
	"diag",
};
static char *usb_functions_mtp_ums_adb_diag2[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"diag_mdm",
};
static char *usb_functions_mtp_ums_diag2[] = {
	"mtp",
	"mass_storage",
	"diag",
	"diag_mdm",
};
#endif

#ifdef CONFIG_USB_ANDROID_SERIAL
static char *usb_functions_adb_serial[] = {
	"mass_storage",
	"adb",
	"serial",
};

static char *usb_functions_modem[] = {
	"mass_storage",
	"modem",
};
static char *usb_functions_adb_modem[] = {
	"mass_storage",
	"adb",
	"modem",
};
static char *usb_functions_adb_serial_modem[] = {
	"mass_storage",
	"adb",
	"modem",
	"serial",
};
static char *usb_functions_serial_modem[] = {
	"mass_storage",
	"modem",
	"serial",
};
static char *usb_functions_mtp_ums_adb_modem[] = {
	"mtp",
	"adb",
	"mass_storage",
	"modem",
};
static char *usb_functions_mtp_ums_modem[] = {
	"mtp",
	"mass_storage",
	"modem",
};
static char *usb_functions_mtp_ums_serial_modem[] = {
	"mtp",
	"mass_storage",
	"modem",
	"serial",
};
static char *usb_functions_mtp_ums_adb_serial_modem[] = {
	"mtp",
	"adb",
	"mass_storage",
	"modem",
	"serial",
};
static char *usb_functions_mtp_ums_adb_serial[] = {
	"mtp",
	"adb",
	"mass_storage",
	"serial",
};
#if defined(CONFIG_USB_ANDROID_DIAG) || defined(CONFIG_USB_ANDROID_QCT_DIAG)
static char *usb_functions_diag_modem[] = {
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_adb_diag_modem[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_adb_diag_serial[] = {
	"mass_storage",
	"adb",
	"diag",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_diag_serial[] = {
	"mass_storage",
	"diag",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_adb_diag_serial_modem[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_diag_serial_modem[] = {
	"mass_storage",
	"diag",
	"modem",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_adb_diag_modem[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_diag_serial[] = {
	"mtp",
	"mass_storage",
	"diag",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_adb_diag_serial[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_adb_diag_serial_modem[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_diag_serial_modem[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_diag_modem[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
#endif
#endif

#ifdef CONFIG_USB_ANDROID_ACM
static char *usb_functions_adb_acm[] = {
	"mass_storage",
	"adb",
	"acm",
};
static char *usb_functions_acm[] = {
	"acm",
};
#endif

#ifdef CONFIG_USB_ANDROID_NCM
static char *usb_functions_ncm_adb[] = {
	"ncm",
	"adb",
};
static char *usb_functions_ncm[] = {
	"ncm",
};
#endif

static char *usb_functions_adb_diag_modem_svlte2[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
};
static char *usb_functions_diag_modem_svlte2[] = {
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
};
static char *usb_functions_mtp_ums_adb_diag_modem_svlte2[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
};
static char *usb_functions_mtp_ums_diag_modem_svlte2[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
};
static char *usb_functions_adb_modem_svlte2[] = {
	"mass_storage",
	"adb",
	"modem",
	"modem_mdm",
};
static char *usb_functions_modem_svlte2[] = {
	"mass_storage",
	"modem",
	"modem_mdm",
};
static char *usb_functions_mtp_ums_adb_modem_svlte2[] = {
	"mtp",
	"adb",
	"mass_storage",
	"modem",
	"modem_mdm",
};
static char *usb_functions_mtp_ums_modem_svlte2[] = {
	"mtp",
	"mass_storage",
	"modem",
	"modem_mdm",
};
static char *usb_functions_adb_diag_modem_svlte2_rment[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
	"rmnet",
};
static char *usb_functions_diag_modem_svlte2_rment[] = {
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
	"rmnet",
};
static char *usb_functions_mtp_ums_adb_diag_modem_svlte2_rment[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
	"rmnet",
};
static char *usb_functions_mtp_ums_diag_modem_svlte2_rment[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
	"rmnet",
};

#if (defined(CONFIG_USB_ANDROID_RMNET_SDIO) || \
	defined(CONFIG_USB_ANDROID_RMNET_SMD_SDIO) || \
	defined(CONFIG_MSM_RMNET_BAM))
static char *usb_functions_adb_diag_modem_rment[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_diag_modem_rment[] = {
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_mtp_ums_adb_diag_modem_rment[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_mtp_ums_diag_modem_rment[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};

static char *usb_functions_adb_diag_rment[] = {
	"mass_storage",
	"adb",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_diag_rment[] = {
	"mass_storage",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_mtp_ums_adb_diag_rment[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_mtp_ums_diag_rment[] = {
	"mtp",
	"mass_storage",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_adb_rment[] = {
	"mass_storage",
	"adb",
	"rmnet",
};
static char *usb_functions_rment[] = {
	"mass_storage",
	"rmnet",
};
static char *usb_functions_mtp_ums_adb_rment[] = {
	"mtp",
	"adb",
	"mass_storage",
	"rmnet",
};
static char *usb_functions_mtp_ums_rment[] = {
	"mtp",
	"mass_storage",
	"rmnet",
};

#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"accessory",
	"audio_source",
#if defined(CONFIG_USB_ANDROID_MTP36) || defined(CONFIG_USB_ANDROID_MTP)
	"mtp",
#endif
#if defined(CONFIG_USB_ANDROID_MTP36) && defined(CONFIG_USB_ANDROID_MTP)
	"mtp36",
#endif
	"adb",
	"mass_storage",
#ifdef CONFIG_USB_ANDROID_ECM
	"cdc_ethernet",
#endif
#if defined(CONFIG_USB_ANDROID_DIAG) || defined(CONFIG_USB_ANDROID_QCT_DIAG)
	"diag",
#endif
#ifdef CONFIG_USB_ANDROID_SERIAL
	"serial",
#endif
	"projector",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_NCM
	"ncm",
#endif
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
#if (defined(CONFIG_USB_ANDROID_RMNET_SDIO) || \
	defined(CONFIG_USB_ANDROID_RMNET_SMD_SDIO) || \
	defined(CONFIG_MSM_RMNET_BAM))
	"rmnet",
#endif
#ifdef CONFIG_USB_ANDROID_USBNET
	"usbnet",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id = 0x0f90, /* vary by board */
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb),
		.functions	= usb_functions_mtp_ums_adb,
	},
	{
		.product_id = 0x0c02, /* vary by board */
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
	{
		.product_id	= 0x0f91,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums),
		.functions	= usb_functions_mtp_ums,
	},
	{
		.product_id	= 0x0ff9,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
#ifdef CONFIG_USB_ANDROID_USBNET
	{
		.product_id	= 0x0fcd,
		.num_functions	= ARRAY_SIZE(usb_functions_usbnet),
		.functions	= usb_functions_usbnet,
	},
	{
		.product_id	= 0x0fce,
		.num_functions	= ARRAY_SIZE(usb_functions_usbnet_adb),
		.functions	= usb_functions_usbnet_adb,
	},
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	{
		.product_id	= 0x0ff4,
		.num_functions	= ARRAY_SIZE(usb_functions_acm),
		.functions	= usb_functions_acm,
	},
	{
		.product_id	= 0x0ff5,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_acm),
		.functions	= usb_functions_adb_acm,
	},
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	{
		.product_id	= 0x0ff8,
		.num_functions	= ARRAY_SIZE(usb_functions_ecm),
		.functions	= usb_functions_ecm,
	},
#endif
#ifdef CONFIG_USB_ANDROID_NCM
	{
		.product_id	= 0x0f92,
		.num_functions	= ARRAY_SIZE(usb_functions_ncm_adb),
		.functions	= usb_functions_ncm_adb,
	},
	{
		.product_id	= 0x0f93,
		.num_functions	= ARRAY_SIZE(usb_functions_ncm),
		.functions	= usb_functions_ncm,
	},
#endif
#ifdef CONFIG_USB_ANDROID_SERIAL
	{
		.product_id	= 0x0fc5,
		.num_functions	= ARRAY_SIZE(usb_functions_serial_modem),
		.functions	= usb_functions_serial_modem,
	},
	{
		.product_id	= 0x0fc6,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_serial_modem),
		.functions	= usb_functions_adb_serial_modem,
	},
	{
		.product_id	= 0x0f9e,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_serial_modem),
		.functions	= usb_functions_mtp_ums_serial_modem,
	},
	{
		.product_id	= 0x0f9f,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_serial_modem),
		.functions	= usb_functions_mtp_ums_adb_serial_modem,
	},
	{
		.product_id	= 0x0fd1,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_serial),
		.functions	= usb_functions_adb_serial,
	},
	{
		.product_id	= 0x0fa0,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_serial),
		.functions	= usb_functions_mtp_ums_adb_serial,
	},

	{
		.product_id	= 0x0c03,
		.num_functions	= ARRAY_SIZE(usb_functions_modem),
		.functions	= usb_functions_modem,
	},
	{
		.product_id	= 0x0c04,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_modem),
		.functions	= usb_functions_adb_modem,
	},
	{
		.product_id	= 0x0f95,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_modem),
		.functions	= usb_functions_mtp_ums_adb_modem,
	},
	{
		.product_id	= 0x0f96,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_modem),
		.functions	= usb_functions_mtp_ums_modem,
	},
#if defined(CONFIG_USB_ANDROID_DIAG) || defined(CONFIG_USB_ANDROID_QCT_DIAG)
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	{
		.product_id	= 0x0fde,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem),
		.functions	= usb_functions_adb_diag_modem,
	},
	{
		.product_id	= 0x0fdf,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_serial),
		.functions	= usb_functions_diag_serial,
	},
	{
		.product_id	= 0x0fe0,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_serial),
		.functions	= usb_functions_adb_diag_serial,
	},
	{
		.product_id	= 0x0fe1,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_serial_modem),
		.functions	= usb_functions_adb_diag_serial_modem,
	},
	{
		.product_id	= 0x0fe2,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_serial_modem),
		.functions	= usb_functions_diag_serial_modem,
	},
	{
		.product_id	= 0x0fe7,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem),
		.functions	= usb_functions_diag_modem,
	},
	{
		.product_id	= 0x0fad,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem),
		.functions	= usb_functions_mtp_ums_adb_diag_modem,
	},
	{
		.product_id	= 0x0fae,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_serial),
		.functions	= usb_functions_mtp_ums_diag_serial,
	},
	{
		.product_id	= 0x0faf,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_serial),
		.functions	= usb_functions_mtp_ums_adb_diag_serial,
	},
	{
		.product_id	= 0x0fb0,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_serial_modem),
		.functions	= usb_functions_mtp_ums_adb_diag_serial_modem,
	},
	{
		.product_id	= 0x0fb1,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_serial_modem),
		.functions	= usb_functions_mtp_ums_diag_serial_modem,
	},
	{
		.product_id	= 0x0fb6,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem),
		.functions	= usb_functions_mtp_ums_diag_modem,
	},
#else
	{
		.product_id	= 0x0c88,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem),
		.functions	= usb_functions_adb_diag_modem,
	},
	{
		.product_id	= 0x0c89,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_serial),
		.functions	= usb_functions_diag_serial,
	},
	{
		.product_id	= 0x0c8a,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_serial),
		.functions	= usb_functions_adb_diag_serial,
	},
	{
		.product_id	= 0x0f9b,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem),
		.functions	= usb_functions_mtp_ums_adb_diag_modem,
	},
	{
		.product_id	= 0x0f9c,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_serial),
		.functions	= usb_functions_mtp_ums_diag_serial,
	},
	{
		.product_id	= 0x0f9d,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_serial),
		.functions	= usb_functions_mtp_ums_adb_diag_serial,
	},
	{
		.product_id	= 0x0fe8,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_serial_modem),
		.functions	= usb_functions_adb_diag_serial_modem,
	},
	{
		.product_id	= 0x0fe9,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_serial_modem),
		.functions	= usb_functions_diag_serial_modem,
	},
	{
		.product_id	= 0x0ffb,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem),
		.functions	= usb_functions_diag_modem,
	},
	{
		.product_id	= 0x0fa6,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_serial_modem),
		.functions	= usb_functions_mtp_ums_adb_diag_serial_modem,
	},
	{
		.product_id	= 0x0fa7,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_serial_modem),
		.functions	= usb_functions_mtp_ums_diag_serial_modem,
	},
	{
		.product_id	= 0x0faa,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem),
		.functions	= usb_functions_mtp_ums_diag_modem,
	},
#endif
#endif
#endif
	{
		.product_id	= 0x0c05,
		.num_functions	= ARRAY_SIZE(usb_functions_projector),
		.functions	= usb_functions_projector,
	},
	{
		.product_id	= 0x0c06,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_projector),
		.functions	= usb_functions_adb_projector,
	},
	{
		.product_id	= 0x0f97,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_projector),
		.functions	= usb_functions_mtp_ums_adb_projector,
	},
	{
		.product_id	= 0x0f98,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_projector),
		.functions	= usb_functions_mtp_ums_projector,
	},
#if defined(CONFIG_USB_ANDROID_DIAG) || defined(CONFIG_USB_ANDROID_QCT_DIAG)
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	{
		.product_id	= 0x0FE3,
		.num_functions	= ARRAY_SIZE(usb_function_adb_diag_projector),
		.functions	= usb_function_adb_diag_projector,
	},
	{
		.product_id	= 0x0FB2,
		.num_functions	= ARRAY_SIZE(usb_function_mtp_ums_adb_diag_projector),
		.functions	= usb_function_mtp_ums_adb_diag_projector,
	},
#ifdef CONFIG_USB_ANDROID_SERIAL
	{
		.product_id	= 0x0FE4,
		.num_functions	= ARRAY_SIZE(usb_function_adb_diag_modem_projector),
		.functions	= usb_function_adb_diag_modem_projector,
	},
	{
		.product_id	= 0x0FB3,
		.num_functions	= ARRAY_SIZE(usb_function_mtp_ums_adb_diag_modem_projector),
		.functions	= usb_function_mtp_ums_adb_diag_modem_projector,
	},
#endif
#else
	{
		.product_id	= 0x0FF1,
		.num_functions	= ARRAY_SIZE(usb_function_adb_diag_projector),
		.functions	= usb_function_adb_diag_projector,
	},
	{
		.product_id	= 0x0FA8,
		.num_functions	= ARRAY_SIZE(usb_function_mtp_ums_adb_diag_projector),
		.functions	= usb_function_mtp_ums_adb_diag_projector,
	},
#ifdef CONFIG_USB_ANDROID_SERIAL
	{
		.product_id	= 0x0FF2,
		.num_functions	= ARRAY_SIZE(usb_function_adb_diag_modem_projector),
		.functions	= usb_function_adb_diag_modem_projector,
	},
	{
		.product_id	= 0x0FA9,
		.num_functions	= ARRAY_SIZE(usb_function_mtp_ums_adb_diag_modem_projector),
		.functions	= usb_function_mtp_ums_adb_diag_modem_projector,
	},
#endif
#endif

#endif
#if defined(CONFIG_USB_ANDROID_DIAG) || defined(CONFIG_USB_ANDROID_QCT_DIAG)
	{
		.product_id	= 0x0FDC,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag2),
		.functions	= usb_functions_adb_diag2,
	},
	{
		.product_id	= 0x0FDD,
		.num_functions	= ARRAY_SIZE(usb_functions_diag2),
		.functions	= usb_functions_diag2,
	},
	{
		.product_id	= 0x0FAB,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag2),
		.functions	= usb_functions_mtp_ums_adb_diag2,
	},
	{
		.product_id	= 0x0FAC,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag2),
		.functions	= usb_functions_mtp_ums_diag2,
	},
	{
		.product_id	= 0x0c07,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
	},
	{
		.product_id	= 0x0c08,
		.num_functions	= ARRAY_SIZE(usb_functions_diag),
		.functions	= usb_functions_diag,
	},
	{
		.product_id	= 0x0f99,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag),
		.functions	= usb_functions_mtp_ums_adb_diag,
	},
	{
		.product_id	= 0x0f9a,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag),
		.functions	= usb_functions_mtp_ums_diag,
	},
#endif
#if defined(CONFIG_USB_ANDROID_MTP36) || defined(CONFIG_USB_ANDROID_MTP)
	{
		.product_id	= 0x0ca8,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb),
		.functions	= usb_functions_mtp_adb,
	},
	{
		.product_id	= 0x0c93,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp),
		.functions	= usb_functions_mtp,
	},
#endif
#if defined(CONFIG_USB_ANDROID_MTP36) && defined(CONFIG_USB_ANDROID_MTP)
	{
		.product_id	= 0x0ca8,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp36_adb),
		.functions	= usb_functions_mtp36_adb,
	},
	{
		.product_id	= 0x0c93,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp36),
		.functions	= usb_functions_mtp36,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		.product_id	= 0x0ffe,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x0ffc,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
	{
		.product_id	= 0x0fb4,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_adb),
		.functions	= usb_functions_rndis_mtp_ums_adb,
	},
	{
		.product_id	= 0x0fb5,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums),
		.functions	= usb_functions_rndis_mtp_ums,
	},
#if defined(CONFIG_USB_ANDROID_DIAG) || defined(CONFIG_USB_ANDROID_QCT_DIAG)
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	{
		.product_id	= 0x0fe5,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_diag),
		.functions	= usb_functions_rndis_adb_diag,
	},
	{
		.product_id	= 0x0fe6,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_diag),
		.functions	= usb_functions_rndis_diag,
	},
	{
		.product_id	= 0x0f8e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_adb_diag),
		.functions	= usb_functions_rndis_mtp_ums_adb_diag,
	},
	{
		.product_id	= 0x0f8f,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_diag),
		.functions	= usb_functions_rndis_mtp_ums_diag,
	},
#else
	{
		.product_id	= 0x0f8d,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_diag),
		.functions	= usb_functions_rndis_mtp_ums_diag,
	},
	{
		.product_id	= 0x0f8c,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_adb_diag),
		.functions	= usb_functions_rndis_mtp_ums_adb_diag,
	},
	{
		.product_id	= 0x0ff6,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_diag),
		.functions	= usb_functions_rndis_adb_diag,
	},
	{
		.product_id	= 0x0ff7,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_diag),
		.functions	= usb_functions_rndis_diag,
	},
#endif
#endif
#endif
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory),
		.functions	= usb_functions_accessory,
	},
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory_adb),
		.functions	= usb_functions_accessory_adb,
	},
	{
		.product_id	= 0x2d02,
		.num_functions	= ARRAY_SIZE(usb_functions_audio_source),
		.functions	= usb_functions_audio_source,
	},
	{
		.product_id	= 0x2d03,
		.num_functions	= ARRAY_SIZE(usb_functions_audio_source_adb),
		.functions	= usb_functions_audio_source_adb,
	},
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= 0x2d04,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory_audio_source),
		.functions	= usb_functions_accessory_audio_source,
	},
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= 0x2d05,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory_audio_source_adb),
		.functions	= usb_functions_accessory_audio_source_adb,
	},
	
#if (defined(CONFIG_USB_ANDROID_RMNET_SDIO) || \
	defined(CONFIG_USB_ANDROID_RMNET_SMD_SDIO) || \
	defined(CONFIG_MSM_RMNET_BAM))
#ifdef CONFIG_USB_ANDROID_MDM9K_DIAG
	{
		.product_id	= 0x0fd2,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem_rment),
		.functions	= usb_functions_adb_diag_modem_rment,
	},
	{
		.product_id	= 0x0fd3,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem_rment),
		.functions	= usb_functions_diag_modem_rment,
	},
	{
		.product_id	= 0x0fd4,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_rment),
		.functions	= usb_functions_adb_diag_rment,
	},
	{
		.product_id	= 0x0fd5,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_rment),
		.functions	= usb_functions_diag_rment,
	},
	{
		.product_id	= 0x0fbd,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem_rment),
		.functions	= usb_functions_mtp_ums_adb_diag_modem_rment,
	},
	{
		.product_id	= 0x0fbe,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_rment),
		.functions	= usb_functions_mtp_ums_diag_modem_rment,
	},
	{
		.product_id	= 0x0fbf,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_rment),
		.functions	= usb_functions_mtp_ums_adb_diag_rment,
	},
	{
		.product_id	= 0x0fc0,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_rment),
		.functions	= usb_functions_mtp_ums_diag_rment,
	},
#endif
	{
		.product_id	= 0x0fd6,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_rment),
		.functions	= usb_functions_adb_diag_rment,
	},
	{
		.product_id	= 0x0fd7,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_rment),
		.functions	= usb_functions_diag_rment,
	},

	{
		.product_id	= 0x0fd8,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem_rment),
		.functions	= usb_functions_adb_diag_modem_rment,
	},
	{
		.product_id	= 0x0fd9,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem_rment),
		.functions	= usb_functions_diag_modem_rment,
	},
	{
		.product_id	= 0x0fda,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_rment),
		.functions	= usb_functions_adb_rment,
	},
	{
		.product_id	= 0x0fdb,
		.num_functions	= ARRAY_SIZE(usb_functions_rment),
		.functions	= usb_functions_rment,
	},
	{
		.product_id	= 0x0fc1,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_rment),
		.functions	= usb_functions_mtp_ums_adb_diag_rment,
	},
	{
		.product_id	= 0x0fa1,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_rment),
		.functions	= usb_functions_mtp_ums_diag_rment,
	},

	{
		.product_id	= 0x0fa2,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem_rment),
		.functions	= usb_functions_mtp_ums_adb_diag_modem_rment,
	},
	{
		.product_id	= 0x0fa3,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_rment),
		.functions	= usb_functions_mtp_ums_diag_modem_rment,
	},
	{
		.product_id	= 0x0fa4,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_rment),
		.functions	= usb_functions_mtp_ums_adb_rment,
	},
	{
		.product_id	= 0x0fa5,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_rment),
		.functions	= usb_functions_mtp_ums_rment,
	},
#endif


	{
		.product_id	= 0x0fcf,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem_svlte2_rment),
		.functions	= usb_functions_adb_diag_modem_svlte2_rment,
	},
	{
		.product_id	= 0x0fd0,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem_svlte2_rment),
		.functions	= usb_functions_diag_modem_svlte2_rment,
	},
	{
		.product_id	= 0x0fbb,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem_svlte2_rment),
		.functions	= usb_functions_mtp_ums_adb_diag_modem_svlte2_rment,
	},
	{
		.product_id	= 0x0fbc,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_svlte2_rment),
		.functions	= usb_functions_mtp_ums_diag_modem_svlte2_rment,
	},
	{
		.product_id	= 0x0fc9,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem_svlte2),
		.functions	= usb_functions_adb_diag_modem_svlte2,
	},
	{
		.product_id	= 0x0fca,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem_svlte2),
		.functions	= usb_functions_diag_modem_svlte2,
	},
	{
		.product_id	= 0x0fcb,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_modem_svlte2),
		.functions	= usb_functions_adb_modem_svlte2,
	},
	{
		.product_id	= 0x0fcc,
		.num_functions	= ARRAY_SIZE(usb_functions_modem_svlte2),
		.functions	= usb_functions_modem_svlte2,
	},
	{
		.product_id	= 0x0fb7,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem_svlte2),
		.functions	= usb_functions_mtp_ums_adb_diag_modem_svlte2,
	},
	{
		.product_id	= 0x0fb8,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_svlte2),
		.functions	= usb_functions_mtp_ums_diag_modem_svlte2,
	},
	{
		.product_id	= 0x0fb9,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_modem_svlte2),
		.functions	= usb_functions_mtp_ums_adb_modem_svlte2,
	},
	{
		.product_id	= 0x0fba,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_modem_svlte2),
		.functions	= usb_functions_mtp_ums_modem_svlte2,
	},
};

#ifdef CONFIG_USB_GADGET_VERIZON_PRODUCT_ID
/*
 * Verizon: 0x0e05 ~ 0x0e72
 *	Internet sharing(0)  and Internet pass through(1) both use rndis
 */
#ifdef CONFIG_MACH_VIGOR
static int vigor_usb_product_id_match_array[] = {
	0x0c93, 0x0e05, /* mtp */
	0x0ca8, 0x0e06, /* mtp + adb */
	0x0fda, 0x0e07, /* ums + adb + 9k rmnet */
	0x0fdb, 0x0e08, /* ums + 9k rmnet */
	0x0c07, 0x0e0b, /* ums + adb + diag */
	0x0c08, 0x0e0c, /* ums + diag */
	0x0ff8, 0x0e0d, /* CDC-ECM */
	0x0fd5, 0x0e0e, /* ums + diag + 9k diag + rmnet */
	0x0fd4, 0x0e0f, /* ums + adb + diag + 9k diag + rmnet */
	0x0ff9, 0x0e73, /* ums */
	0x0fa4, 0x0e8f, /* mtp + adb + 9k rmnet */
	0x0fa5, 0x0e90, /* mtp + 9k rmnet */
	0x0f99, 0x0e91, /* mtp + adb + diag */
	0x0f9a, 0x0e92, /* mtp + diag */
	0x0fc0, 0x0e93, /* mtp + diag + 9k diag + rmnet */
	0x0fbf, 0x0e94, /* mtp + adb + diag + 9k diag + rmnet */
	-1,
};

static int vigor_usb_product_id_rndis[] = {
	0x0e09, 0x0e0a,
};
#endif /* CONFIG_MACH_VIGOR */

#ifdef CONFIG_MACH_VIVOW
static int vivow_usb_product_id_match_array[] = {
	0x0c93, 0x0e10, /* mtp */
	0x0ca8, 0x0e11, /* mtp + adb */
	0x0fda, 0x0e12, /* ums + adb + 9k rmnet */
	0x0fdb, 0x0e13, /* ums + 9k rmnet */
	0x0c07, 0x0e16, /* ums + adb + diag */
	0x0c08, 0x0e17, /* ums + diag */
	0x0ff8, 0x0e18, /* CDC-ECM */
//0e19 is recycle	0x0fd5, 0x0e19, /* ums + diag + 9k diag + rmnet */
//0e1a is recycle	0x0fd4, 0x0e1a, /* ums + adb + diag + 9k diag + rmnet */
	0x0ff9, 0x0e74, /* ums */
	0x0fa4, 0x0e87, /* mtp + adb + 9k rmnet */
	0x0fa5, 0x0e88, /* mtp + 9k rmnet */
	0x0f99, 0x0e89, /* mtp + adb + diag */
	0x0f9a, 0x0e8a, /* mtp + diag */
	-1,
};

static int vivow_usb_product_id_rndis[] = {
	0x0e14, 0x0e15,
};
#endif /* CONFIG_MACH_VIVOW */

#ifdef CONFIG_MACH_MECHA
static int mecha_usb_product_id_match_array[] = {
	0x0c93, 0x0e1b, /* mtp */
	0x0ca8, 0x0e1c, /* mtp + adb */
	0x0fda, 0x0e1d, /* ums + adb + 9k rmnet */
	0x0fdb, 0x0e1e, /* ums + 9k rmnet */
	0x0c07, 0x0e21, /* ums + adb + diag */
	0x0c08, 0x0e22, /* ums + diag */
	0x0ff8, 0x0e23, /* CDC-ECM */
	0x0fd5, 0x0e24, /* ums + diag + 9k diag + rmnet */
	0x0fd4, 0x0e25, /* ums + adb + diag + 9k diag + rmnet */
	0x0ff9, 0x0e75, /* ums */
	0x0fa4, 0x0e95, /* mtp + adb + 9k rmnet */
	0x0fa5, 0x0e96, /* mtp + 9k rmnet */
	0x0f99, 0x0e97, /* mtp + adb + diag */
	0x0f9a, 0x0e98, /* mtp + diag */
	0x0fc0, 0x0e99, /* mtp + diag + 9k diag + rmnet */
	0x0fbf, 0x0e9a, /* mtp + adb + diag + 9k diag + rmnet */
	-1,
};

static int mecha_usb_product_id_rndis[] = {
	0x0e1f, 0x0e20,
};
#endif /* CONFIG_MACH_MECHA */

#ifdef CONFIG_MACH_BLISSC
static int blissc_usb_product_id_match_array[] = {
	0x0c93, 0x0e26, /* mtp */
	0x0ca8, 0x0e27, /* mtp + adb */
	0x0fda, 0x0e28, /* ums + adb + 9k rmnet */
	0x0fdb, 0x0e29, /* ums + 9k rmnet */
	0x0c07, 0x0e2c, /* ums + adb + diag */
	0x0c08, 0x0e2d, /* ums + diag */
	0x0ff8, 0x0e2e, /* CDC-ECM */
//0e2f is recycle	0x0fd5, 0x0e2f, /* ums + diag + 9k diag + rmnet */
//0e30 is recycle	0x0fd4, 0x0e30, /* ums + adb + diag + 9k diag + rmnet */
	0x0ff9, 0x0ccb, /* ums */
	0x0ffc, 0x0e7d, /* adb + rndis, tethering with USB debugging */
	0x0fa4, 0x0e9b, /* mtp + adb + 9k rmnet */
	0x0fa5, 0x0e9c, /* mtp + 9k rmnet */
	0x0f99, 0x0e9d, /* mtp + adb + diag */
	0x0f9a, 0x0e9e, /* mtp + diag */
	-1,
};

static int blissc_usb_product_id_rndis[] = {
	0x0e2a, 0x0e2b,
};
#endif /* CONFIG_MACH_BLISSC */
#endif /* CONFIG_USB_GADGET_VERIZON_PRODUCT_ID */

#endif
