/*
 * arch/arm/mach-tegra/board-evitareul.h
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MACH_TEGRA_BOARD_EVITAREUL_H
#define _MACH_TEGRA_BOARD_EVITAREUL_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <mach/htc_asoc_pdata.h>
#include <linux/mfd/tps80031.h>

/* Processor Board  ID */
#define BOARD_E1205		0x0C05
#define BOARD_E1197		0x0B61
#define SKU_BATTERY_SUPPORT	0x1

/* Board Fab version */
#define BOARD_FAB_A00		0x0
#define BOARD_FAB_A01		0x1
#define BOARD_FAB_A02		0x2
#define BOARD_FAB_A03		0x3
#define BOARD_FAB_A04		0x4

/* vdd_cpu voltage follower */
#define BOARD_SKU_VF_BIT	0x0400

int evitareul_charge_init(void);
int evitareul_sdhci_init(void);
int evitareul_pinmux_init(void);
int evitareul_panel_init(void);
int evitareul_sensors_init(void);
int evitareul_cam_init(void);
int touch_init(void);
int evitareul_kbc_init(void);
int evitareul_emc_init(void);
int evitareul_regulator_init(void);
int evitareul_modem_init(void);
int evitareul_suspend_init(void);
int evitareul_edp_init(void);
void evitareul_bpc_mgmt_init(void);
int evitareul_audio_codec_init(struct htc_asoc_platform_data *);

/* Invensense MPU Definitions */
#define MPU_TYPE_MPU3050	1
#define MPU_TYPE_MPU6050	2
#define MPU_GYRO_TYPE		MPU_TYPE_MPU3050
#define MPU_GYRO_IRQ_GPIO	TEGRA_GPIO_PH4
#define MPU_GYRO_ADDR		0x68
#define MPU_GYRO_BUS_NUM	0
#define MPU_GYRO_ORIENTATION	{ -1, 0, 0, 0, -1, 0, 0, 0, 1 }
#define MPU_ACCEL_NAME		"kxtf9"
#define MPU_ACCEL_IRQ_GPIO	0 /* DISABLE ACCELIRQ:  TEGRA_GPIO_PJ2 */
#define MPU_ACCEL_ADDR		0x0F
#define MPU_ACCEL_BUS_NUM	0
#define MPU_ACCEL_ORIENTATION	{ 0, 1, 0, -1, 0, 0, 0, 0, 1 }
#define MPU_COMPASS_NAME	"ak8975"
#define MPU_COMPASS_IRQ_GPIO	0
#define MPU_COMPASS_ADDR	0x0C
#define MPU_COMPASS_BUS_NUM	0
#define MPU_COMPASS_ORIENTATION	{ 0, 1, 0, -1, 0, 0, 0, 0, 1 }

/* PCA954x I2C bus expander bus addresses */
#define PCA954x_I2C_BUS_BASE	6
#define PCA954x_I2C_BUS0	(PCA954x_I2C_BUS_BASE + 0)
#define PCA954x_I2C_BUS1	(PCA954x_I2C_BUS_BASE + 1)
#define PCA954x_I2C_BUS2	(PCA954x_I2C_BUS_BASE + 2)
#define PCA954x_I2C_BUS3	(PCA954x_I2C_BUS_BASE + 3)

/*****************External GPIO tables ******************/
/* External peripheral gpio base. */
#define ENT_TPS80031_GPIO_BASE	   TEGRA_NR_GPIOS
#define ENT_TPS80031_GPIO_REGEN1 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_REGEN1)
#define ENT_TPS80031_GPIO_REGEN2 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_REGEN2)
#define ENT_TPS80031_GPIO_SYSEN	 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_SYSEN)
#define ENT_TPS80031_GPIO_END	(ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_NR)

/*****************External Interrupt tables ******************/
/* External peripheral irq base */
#define ENT_TPS80031_IRQ_BASE	TEGRA_NR_IRQS
#define ENT_TPS80031_IRQ_END  (ENT_TPS80031_IRQ_BASE + TPS80031_INT_NR)

/* Camera GPIOs */
#define MCAM_SPI_CLK    TEGRA_GPIO_PC2
#define MCAM_SPI_DO     TEGRA_GPIO_PC3
#define CAM1_PWDN       TEGRA_GPIO_PF4
#define CAM_D1V2_EN     TEGRA_GPIO_PF5
#define CAM2_D1V2_EN    TEGRA_GPIO_PF6
#define MCAM_SPI_CS0    TEGRA_GPIO_PJ5
#define MCAM_SPI_DI     TEGRA_GPIO_PJ6
#define FRONT_CAM_RST   TEGRA_GPIO_PM2
#define RAW_INTR0       TEGRA_GPIO_PR0
#define RAW_1V8_EN      TEGRA_GPIO_PR3
#define RAW_RSTN        TEGRA_GPIO_PR4
#define CAM1_ID         TEGRA_GPIO_PR6
#define CAM_I2C_SCL     TEGRA_GPIO_PBB1
#define CAM_I2C_SDA     TEGRA_GPIO_PBB2
#define CAMIO_1V8_EN    TEGRA_GPIO_PBB4
#define CAM1_VCM_PD     TEGRA_GPIO_PBB5
#define CAM_MCLK        TEGRA_GPIO_PCC0
#define CAM_SEL         TEGRA_GPIO_PCC1
#define RAW_INTR1       TEGRA_GPIO_PEE1

/* Audio-related GPIOs */
#define TEGRA_GPIO_HP_DET	TEGRA_GPIO_PW3
#define TEGRA_GPIO_AUD_2V85_EN	TEGRA_GPIO_PB2

/* Baseband GPIO addresses */

#define GPIO_BB_RESET		TEGRA_GPIO_PE1
#define GPIO_BB_PWRON		TEGRA_GPIO_PE0
#define GPIO_BB_APACK		TEGRA_GPIO_PE3
#define GPIO_BB_APACK2		TEGRA_GPIO_PE2
#define GPIO_BB_CPACK		TEGRA_GPIO_PU5
#define GPIO_BB_CPACK2		TEGRA_GPIO_PV0
#define GPIO_BB_RSVD1		TEGRA_GPIO_PV1
#define GPIO_BB_RSVD2		TEGRA_GPIO_PU4

#define BB_GPIO_MDM_PWRON_AP2BB		TEGRA_GPIO_PE0 /* LCD_D0 */
#define BB_GPIO_RESET_AP2BB		TEGRA_GPIO_PE1 /* LCD_D1 */
#define BB_GPIO_LCD_PWR1		TEGRA_GPIO_PC1
#define BB_GPIO_LCD_PWR2		TEGRA_GPIO_PC6
#define BB_GPIO_HS1_AP2BB		TEGRA_GPIO_PE3 /* LCD_D3 */
#define BB_GPIO_HS1_BB2AP		TEGRA_GPIO_PU5

#define XMM_GPIO_BB_ON			BB_GPIO_MDM_PWRON_AP2BB
#define XMM_GPIO_BB_RST			BB_GPIO_RESET_AP2BB
#define XMM_GPIO_IPC_HSIC_ACTIVE	BB_GPIO_LCD_PWR1
#define XMM_GPIO_IPC_HSIC_SUS_REQ	BB_GPIO_LCD_PWR2
#define XMM_GPIO_IPC_BB_WAKE		BB_GPIO_HS1_AP2BB
#define XMM_GPIO_IPC_AP_WAKE		BB_GPIO_HS1_BB2AP

/* BT */
#define EVITAREUL_GPIO_BT_UART3_CTS      TEGRA_GPIO_PA1
#define EVITAREUL_GPIO_BT_UART3_RTS      TEGRA_GPIO_PC0
#define EVITAREUL_GPIO_BT_UART3_TX       TEGRA_GPIO_PW6
#define EVITAREUL_GPIO_BT_UART3_RX       TEGRA_GPIO_PW7
#define EVITAREUL_GPIO_BT_WAKE           TEGRA_GPIO_PD4
#define EVITAREUL_GPIO_BT_SHUTDOWN_N     TEGRA_GPIO_PU0
#define EVITAREUL_GPIO_BT_HOST_WAKE      TEGRA_GPIO_PO5

#define TDIODE_OFFSET	(9000)	/* in millicelsius */

/* Battery Peak Current Management */
#define TEGRA_BPC_TRIGGER		TEGRA_GPIO_PS7
#define TEGRA_BPC_TIMEOUT		100 /* ms */
#define TEGRA_BPC_CPU_PWR_LIMIT	3500 /* in mW, (0 disables) */

#define TEGRA_CUR_MON_THRESHOLD		-3600
#define TEGRA_CUR_MON_RESISTOR		10
#define TEGRA_CUR_MON_MIN_CORES		2

/* PMU */
#define PMU_GPIO_PMU_MSECURE		TEGRA_GPIO_PF7

/* flashlight, FL_TORCH_FLASH, FL_FLASH_EN */
#define FL_TORCH_FLASH		TEGRA_GPIO_PR1
#define FL_FLASH_EN		TEGRA_GPIO_PBB3

/* Proximity sensor, PS_INT*/
#define PS_INT			TEGRA_GPIO_PK2

/* Baseband IDs */

enum tegra_bb_type {
	TEGRA_BB_PH450 = 1,
	TEGRA_BB_XMM6260,
	TEGRA_BB_M7400,
};

/* NFC GPIO */
#define RUBY_GPIO_NFC_INT       TEGRA_GPIO_PS4
#define RUBY_GPIO_NFC_VEN       TEGRA_GPIO_PM5
#define RUBY_GPIO_NFC_DL        TEGRA_GPIO_PM6

#endif /*_MACH_TEGRA_BOARD_EVITAREUL_H */
