/*
 * arch/arm/mach-tegra/board-evitareul-pinmux.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/pinmux.h>
#include "board.h"
#include "board-evitareul.h"
#include "gpio-names.h"

#define DEFAULT_DRIVE(_name)					\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_DISABLE,			\
		.schmitt = TEGRA_SCHMITT_ENABLE,		\
		.drive = TEGRA_DRIVE_DIV_1,			\
		.pull_down = TEGRA_PULL_31,			\
		.pull_up = TEGRA_PULL_31,			\
		.slew_rising = TEGRA_SLEW_SLOWEST,		\
		.slew_falling = TEGRA_SLEW_SLOWEST,		\
	}
/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 */
#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive, _pullup_drive, _pulldn_slew, _pullup_slew) \
	{                                               \
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,   \
		.hsm = TEGRA_HSM_##_hsm,                    \
		.schmitt = TEGRA_SCHMITT_##_schmitt,        \
		.drive = TEGRA_DRIVE_##_drive,              \
		.pull_down = TEGRA_PULL_##_pulldn_drive,    \
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,   \
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

/* !!!FIXME!!!! POPULATE THIS TABLE */
static __initdata struct tegra_drive_pingroup_config evitareul_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* SET_DRIVE(ATA, DISABLE, DISABLE, DIV_1, 31, 31, FAST, FAST) */

	/* All I2C pins are driven to maximum drive strength */
	/* GEN1 I2C */
	SET_DRIVE(DBG,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* GEN2 I2C */
	SET_DRIVE(AT5,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* CAM I2C */
	SET_DRIVE(GME,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* DDC I2C */
	SET_DRIVE(DDC,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* PWR_I2C */
	SET_DRIVE(AO1,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* UART3 */
	SET_DRIVE(UART3,	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),
};

#define I2C_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
        {                                                       \
                .pingroup       = TEGRA_PINGROUP_##_pingroup,   \
                .func           = TEGRA_MUX_##_mux,             \
                .pupd           = TEGRA_PUPD_##_pupd,           \
                .tristate       = TEGRA_TRI_##_tri,             \
                .io             = TEGRA_PIN_##_io,              \
                .lock           = TEGRA_PIN_LOCK_##_lock,       \
                .od             = TEGRA_PIN_OD_##_od,           \
                .ioreset        = TEGRA_PIN_IO_RESET_DEFAULT,   \
        }


#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)	\
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_DEFAULT,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

static __initdata struct tegra_pingroup_config evitareul_pinmux_common[] = {

        /* I2C1 pinmux */
        I2C_PINMUX(GEN1_I2C_SCL,        I2C1,           NORMAL, NORMAL, INPUT,  DISABLE,        DISABLE),
        I2C_PINMUX(GEN1_I2C_SDA,        I2C1,           NORMAL, NORMAL, INPUT,  DISABLE,        DISABLE),

        /* I2C2 pinmux */
        I2C_PINMUX(GEN2_I2C_SCL,        I2C2,           NORMAL, NORMAL, INPUT,  DISABLE,        DISABLE),
        I2C_PINMUX(GEN2_I2C_SDA,        I2C2,           NORMAL, NORMAL, INPUT,  DISABLE,        DISABLE),

        /* I2C3 pinmux */
        I2C_PINMUX(CAM_I2C_SCL,         I2C3,           NORMAL, NORMAL, INPUT,  DISABLE,        DISABLE),
        I2C_PINMUX(CAM_I2C_SDA,         I2C3,           NORMAL, NORMAL, INPUT,  DISABLE,        DISABLE),

        /* I2C4 pinmux */
        I2C_PINMUX(DDC_SCL,             I2C4,           NORMAL, NORMAL, INPUT,  DISABLE,        DISABLE),
        I2C_PINMUX(DDC_SDA,             I2C4,           NORMAL, NORMAL, INPUT,  DISABLE,        DISABLE),

        /* Power I2C pinmux */
        I2C_PINMUX(PWR_I2C_SCL,         I2CPWR,         NORMAL, NORMAL, INPUT,  DISABLE,        DISABLE),
        I2C_PINMUX(PWR_I2C_SDA,         I2CPWR,         NORMAL, NORMAL, INPUT,  DISABLE,        DISABLE),

	DEFAULT_PINMUX(SDMMC1_DAT3,     UARTE,           NORMAL,    NORMAL,     OUTPUT),            //AUD_REMO_TX
	DEFAULT_PINMUX(SDMMC1_DAT2,     UARTE,           NORMAL,    NORMAL,     INPUT),             //AUD_REMO_RX
	DEFAULT_PINMUX(SDMMC1_CLK,      SDMMC1,          NORMAL,    NORMAL,     OUTPUT),             //AUD_REMO_OE#

};

int __init evitareul_pinmux_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	tegra_pinmux_config_table(evitareul_pinmux_common, ARRAY_SIZE(evitareul_pinmux_common));
	/* all pins should correctly configured in bootloader,
	 * remove all pin config but drive config here
	 */
	tegra_drive_pinmux_config_table(evitareul_drive_pinmux,ARRAY_SIZE(evitareul_drive_pinmux));
	return 0;
}
