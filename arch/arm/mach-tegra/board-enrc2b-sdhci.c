/*
 * arch/arm/mach-tegra/board-enrc2b-sdhci.c
 *
 * Copyright (C) 2011 NVIDIA Corporation.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include "gpio-names.h"
#include "board.h"

/* HTC_WIFI_START */
#define WIFI_SDIO_CLK TEGRA_GPIO_PA6
#define WIFI_SDIO_CMD TEGRA_GPIO_PA7
#define WIFI_SDIO_D0 TEGRA_GPIO_PB7
#define WIFI_SDIO_D1 TEGRA_GPIO_PB6
#define WIFI_SDIO_D2 TEGRA_GPIO_PB5
#define WIFI_SDIO_D3 TEGRA_GPIO_PB4
#define WIFI_ENABLE	TEGRA_GPIO_PV2 	//WIFI_EN
#define WIFI_IRQ	TEGRA_GPIO_PO4	//WW_IRQ
/* HTC_WIFI_END */
#define CONFIG_DHD_USE_STATIC_BUF
#ifdef CONFIG_DHD_USE_STATIC_BUF
#include <linux/skbuff.h>

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *enrc2b_wifi_mem_prealloc(int section, unsigned long size)
{
	printk("wifi: prealloc buffer index: %d\n", section);
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}
#endif

int __init enrc2b_init_wifi_mem(void)
{
#ifdef CONFIG_DHD_USE_STATIC_BUF
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(PAGE_SIZE*2);
		else
			wlan_static_skb[i] = dev_alloc_skb(PAGE_SIZE*4);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
#endif
	return 0;
}
/* HTC_WIFI_END */

//#define EVITAUL_SD_CD TEGRA_GPIO_PI5

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int enrc2b_wifi_status_register(void (*callback)(int , void *), void *);

static int enrc2b_wifi_reset(int on);
/* HTC_WIFI_START */
//static int enrc2b_wifi_power(int on);
//static int enrc2b_wifi_set_carddetect(int val);
int enrc2b_wifi_power(int on);
int enrc2b_wifi_set_carddetect(int val);
int enrc2b_wifi_status(struct device *dev);
static int enrc2b_wifi_cd;		/* WIFI virtual 'card detect' status */
/* HTC_WIFI_END */

static struct wifi_platform_data enrc2b_wifi_control = {
	.set_power      = enrc2b_wifi_power,
	.set_reset      = enrc2b_wifi_reset,
	.set_carddetect = enrc2b_wifi_set_carddetect,
#ifdef CONFIG_DHD_USE_STATIC_BUF
	.mem_prealloc	= enrc2b_wifi_mem_prealloc,
#else
	.mem_prealloc	= NULL,
#endif
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		.start	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO4),
		.end	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PO4),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device enrc2b_wifi_device = {
	.name           = "bcmdhd_wlan",
	.id             = 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev            = {
		.platform_data = &enrc2b_wifi_control,
	},
};

static int emmc_suspend_gpiocfg(void)
{
	ENABLE_GPIO(SDMMC4_CLK, CC4, "SDMMC4_CLK", 0, 0, NORMAL);
	return 0;
}

static void emmc_resume_gpiocfg(void)
{
	DISABLE_GPIO(SDMMC4_CLK, CC4, NORMAL);
}

// No uSD
#if 0
static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif

static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct embedded_sdio_data embedded_sdio_data2 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4334,
	},
};

int enrc2b_wifi_suspend_gpio(void)
{
	printk("[WLAN] pull SDIO pins to low by suspend\n");
	
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_CMD, TEGRA_PUPD_NORMAL);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_DAT0, TEGRA_PUPD_NORMAL);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_DAT1, TEGRA_PUPD_NORMAL);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_DAT2, TEGRA_PUPD_NORMAL);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_DAT3, TEGRA_PUPD_NORMAL);
	
	tegra_gpio_enable(WIFI_SDIO_CLK);
	tegra_gpio_enable(WIFI_SDIO_CMD);
	tegra_gpio_enable(WIFI_SDIO_D0);
	tegra_gpio_enable(WIFI_SDIO_D1);
	tegra_gpio_enable(WIFI_SDIO_D2);
	tegra_gpio_enable(WIFI_SDIO_D3);
	
	gpio_direction_output(WIFI_SDIO_CLK, 0);
	gpio_direction_input(WIFI_SDIO_CMD);
	gpio_direction_input(WIFI_SDIO_D0);
	gpio_direction_input(WIFI_SDIO_D1);
	gpio_direction_input(WIFI_SDIO_D2);
	gpio_direction_input(WIFI_SDIO_D3);
	
	return 0;
}

int enrc2b_wifi_resume_gpio(void)
{
	printk("[WLAN] restore SDIO pins config by resume\n");
	
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_CMD, TEGRA_PUPD_PULL_UP);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_DAT0, TEGRA_PUPD_PULL_UP);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_DAT1, TEGRA_PUPD_PULL_UP);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_DAT2, TEGRA_PUPD_PULL_UP);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_SDMMC3_DAT3, TEGRA_PUPD_PULL_UP);
	
	tegra_gpio_disable(WIFI_SDIO_CLK);
	tegra_gpio_disable(WIFI_SDIO_CMD);
	tegra_gpio_disable(WIFI_SDIO_D0);
	tegra_gpio_disable(WIFI_SDIO_D1);
	tegra_gpio_disable(WIFI_SDIO_D2);
	tegra_gpio_disable(WIFI_SDIO_D3);
	
	return 0;
}

// No uSD
#if 0
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.mmc_data = {
		.register_status_notify	= enrc2b_wifi_status_register,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		.embedded_sdio = &embedded_sdio_data0,
#endif
		/* FIXME need to revert the built_in change
		once we use get the signal strength fix of
		bcmdhd driver from broadcom for bcm4329 chipset*/
		.built_in = 0,
	},
#ifndef CONFIG_MMC_EMBEDDED_SDIO
	.pm_flags = MMC_PM_KEEP_POWER,
#endif
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.max_clk_limit = 45000000,
};
#endif

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.mmc_data = {
		.status = enrc2b_wifi_status,
		.register_status_notify	= enrc2b_wifi_status_register,
		/* HTC_WIFI_START */
		.embedded_sdio = &embedded_sdio_data2,
		/* HTC_WIFI_END */
		.built_in = 1,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.suspend_gpiocfg = enrc2b_wifi_suspend_gpio,
	.resume_gpiocfg = enrc2b_wifi_resume_gpio,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x0F,
	.mmc_data = {
		.built_in = 1,
	},
	.suspend_gpiocfg = emmc_suspend_gpiocfg,
	.resume_gpiocfg = emmc_resume_gpiocfg,
};

// No uSD
#if 0
static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};
#endif

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int enrc2b_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

/* HTC_WIFI_START */
int enrc2b_wifi_status(struct device *dev)
{
	return enrc2b_wifi_cd;
}

//static int enrc2b_wifi_set_carddetect(int val)
int enrc2b_wifi_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);
	enrc2b_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(enrc2b_wifi_set_carddetect);

int enrc2b_wifi_sdclk (int enable){
    printk("set sdio clk:%d\n",enable);
    if(enable) {
        tegra_gpio_disable(WIFI_SDIO_CLK);
    }else{
        tegra_gpio_enable(WIFI_SDIO_CLK);
        gpio_direction_output(WIFI_SDIO_CLK, 0);
    }
}

//static int enrc2b_wifi_power(int on)
int enrc2b_wifi_power(int on)
{
	static int power_state;

	if (on == power_state)
		return 0;

	printk("%s: Powering %s wifi\n", __func__, (on ? "on" : "off"));

	power_state = on;
	enrc2b_wifi_sdclk(on);

	if (on) {
		gpio_set_value(WIFI_ENABLE, 1);
		mdelay(20);
	} else {
		gpio_set_value(WIFI_ENABLE, 0);
	}

	return 0;
/*	
	pr_debug("%s: %d\n", __func__, on);
	gpio_set_value(ENRC2B_WLAN_PWR, on);
	mdelay(100);
	gpio_set_value(ENRC2B_WLAN_RST, on);
	mdelay(200);

	return 0;
*/
}
EXPORT_SYMBOL(enrc2b_wifi_power);
/* HTC_WIFI_END */

static int enrc2b_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static int __init enrc2b_wifi_init(void)
{
	int rc;

	rc = gpio_request(WIFI_ENABLE, "wlan_power");
	if (rc)
		pr_err("WLAN_PWR gpio request failed:%d\n", rc);
	rc = gpio_request(WIFI_IRQ, "bcmsdh_sdmmc");
	if (rc)
		pr_err("WLAN_IRQ gpio request failed:%d\n", rc);
	/*configure wifi irq to input pull down default*/
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_ULPI_DATA3, TEGRA_PUPD_PULL_DOWN);

	tegra_gpio_enable(WIFI_ENABLE);
	tegra_gpio_enable(WIFI_IRQ);

	rc = gpio_direction_output(WIFI_ENABLE, 0);
	if (rc)
		pr_err("WLAN_PWR gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(WIFI_IRQ);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);
	enrc2b_init_wifi_mem();
	platform_device_register(&enrc2b_wifi_device);

	return 0;
}
extern bool wifi_isEvitarel;

int __init enrc2b_sdhci_init(void)
{
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);
	wifi_isEvitarel = false;
	enrc2b_wifi_init();
	return 0;
}
