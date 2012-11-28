/*
 * arch/arm/mach-tegra/board-evitareul-sensors.c
 *
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/i2c/pca954x.h>
#include <linux/nct1008.h>
#include <linux/err.h>
#include <linux/mpu_htc.h>
#include <linux/platform_data/ina230.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <media/ar0832_main.h>
#include <media/tps61050.h>
#include <mach/edp.h>
#include <mach/thermal.h>
#include <linux/platform_device.h>
#include <mach/htc_battery_tps80032.h>
#include "cpu-tegra.h"
#include "gpio-names.h"
#include "board-evitareul.h"
#include "board.h"
#include <mach/board_htc.h>
#include <linux/cm3628.h>
#include <linux/cm3629.h>
#include <linux/pn544.h>

#define SENSOR_MPU_NAME "mpu3050"

static struct regulator *v_ps_2v85_en ;
static struct regulator *v_srio_1v8_en ;

static void cm3629_enable_power(int enable)
{
	if(enable == 1) {
		if (v_ps_2v85_en == NULL) {
			v_ps_2v85_en = regulator_get(NULL, "v_ps_2v85");
			if (WARN_ON(IS_ERR(v_ps_2v85_en))) {
				pr_err("[v_ps_2v85] %s: couldn't get regulator v_ps_2v85_en: %ld\n", __func__, PTR_ERR(v_ps_2v85_en));
			}
		}
		regulator_enable(v_ps_2v85_en);

		if (v_srio_1v8_en == NULL) {
	  		v_srio_1v8_en = regulator_get(NULL, "v_srio_1v8");
	  		if (WARN_ON(IS_ERR(v_srio_1v8_en))) {
				pr_err("[v_srio_1v8] %s: couldn't get regulator v_srio_1v8_en: %ld\n", __func__, PTR_ERR(v_srio_1v8_en));
			}
		}
		regulator_enable(v_srio_1v8_en);
	}else if(enable == 0) {
		if(regulator_is_enabled(v_srio_1v8_en)) {
			regulator_disable(v_srio_1v8_en);
		}
		if(regulator_is_enabled(v_ps_2v85_en)) {
			regulator_disable(v_ps_2v85_en);
		}
	}
}

static struct cm3628_platform_data cm3628_pdata = {
	/*.intr = PSNENOR_INTz,*/
	.pwr = NULL,
	.intr = TEGRA_GPIO_PK2,
	.levels = { 12, 14, 16, 41, 83, 3561, 6082, 6625, 7168, 65535},
	.golden_adc = 0x1145,
	.power = NULL,
	.ALS_slave_address = 0xC0>>1,
	.PS_slave_address = 0xC2>>1,
	.check_interrupt_add = 0x2C>>1	,
	.is_cmd = CM3628_ALS_IT_400ms | CM3628_ALS_PERS_2,
	.ps_thd_set = 0x4,
	.ps_conf2_val = 0,
	.ps_calibration_rule = 1,
	.ps_reset_thd = 1,
	.ps_conf1_val = CM3628_PS_DR_1_320 | CM3628_PS_IT_1_3T |
			CM3628_PS_PERS_4,
	.ps_thd_no_cal = 0x10,
	.ps_thd_with_cal = 0x4,
};

static struct cm3629_platform_data cm3629_pdata = {
	.model = CAPELLA_CM36282,
	.ps_select = CM3629_PS1_ONLY,
	.intr = TEGRA_GPIO_PK2,
	.levels = { 0x0, 0x30, 0xA7, 0x170, 0x5D5, 0xE71, 0x1836, 0x2027, 0x281A, 0xFFFF},
	.golden_adc = 0x13AA,
	.power = NULL,
	.cm3629_slave_address = 0xC0>>1,
	.ps_calibration_rule = 1,
	.ps1_thd_set = 0x13,
	.ps1_thd_no_cal = 0x13,
	.ps1_thd_with_cal = 0x13,
	.ps_conf1_val = CM3629_PS_DR_1_320 | CM3629_PS_IT_1_3T |
			CM3629_PS1_PERS_4,
	.ps_conf2_val = CM3629_PS_ITB_1 | CM3629_PS_ITR_1 |
			CM3629_PS2_INT_DIS | CM3629_PS1_INT_DIS,
	.ps_conf3_val = CM3629_PS2_PROL_32,
	.dark_level = 3,
};

static struct i2c_board_info i2c_CM3628_devices[] = {
	{
		I2C_BOARD_INFO("cm3628", 0xc0 >> 1),
		.platform_data = &cm3628_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
	}
};

static struct i2c_board_info i2c_CM3629_devices[] = {
	{
		I2C_BOARD_INFO(CM3629_I2C_NAME, 0xC0 >> 1),
		.platform_data = &cm3629_pdata,
			.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
	},
};

static void psensor_init(void)
{
	if(ps_type) {
		i2c_register_board_info(0,
				i2c_CM3629_devices, ARRAY_SIZE(i2c_CM3629_devices));
		pr_info("[PS][cm3629]%s ps_type = %d\n", __func__, ps_type);
	}
	else {
		i2c_register_board_info(0,
				i2c_CM3628_devices, ARRAY_SIZE(i2c_CM3628_devices));
		pr_info("[PS][cm3628]%s ps_type = %d\n", __func__, ps_type);
	}
}

static struct mpu3050_platform_data mpu3050_data = {
	.int_config  = 0x10,
	/* Orientation matrix for MPU on evitareul */
	.en_1v8 = 1,
	.orientation = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, // EVT
	.level_shifter = 0,

	.accel = {
		.get_slave_descr = get_accel_slave_descr,
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		//.address     = 0x0F,
		.address     = 0x19, //for A-project
		/* Orientation matrix for Kionix on evitareul */
		.orientation = { -1, 0, 0, 0, 1, 0, 0, 0,-1 }, // EVT .orientation = { -1, 0, 0, 0, 1, 0, 0, 0,-1 }, // EVT

	},

	.compass = {
		.get_slave_descr = get_compass_slave_descr,
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		//.address     = 0x0C,
		.address     = 0x0D,//for A-project
		/* Orientation matrix for AKM on evitareul */
		.orientation = { -1, 0, 0, 0, 1, 0, 0, 0, -1 }, // EVT
	},
};

static struct i2c_board_info __initdata mpu3050_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		//.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH4),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ2),
		.platform_data = &mpu3050_data,
	},
};

static void config_nfc_gpios(void)
{
    int ret = 0;

    ret = gpio_direction_output(RUBY_GPIO_NFC_VEN, 0);
    if (ret < 0) {
        pr_err("TEGRA_GPIO_PM5 output failed\n");
        pr_err("[NFC] %s: gpio_direction_output failed %d\n", __func__, ret);
        gpio_free(RUBY_GPIO_NFC_VEN);
        return;
    }
    tegra_gpio_enable(RUBY_GPIO_NFC_VEN);

    ret = gpio_direction_output(RUBY_GPIO_NFC_DL, 0);
    if (ret < 0) {
        pr_err("TEGRA_GPIO_PM6 output failed\n");
        pr_err("[NFC] %s: gpio_direction_output failed %d\n", __func__, ret);
        gpio_free(RUBY_GPIO_NFC_DL);
        return;
    }
    tegra_gpio_enable(RUBY_GPIO_NFC_DL);

    ret = gpio_direction_input(RUBY_GPIO_NFC_INT);
    if (ret < 0) {
        pr_err("TEGRA_GPIO_PY6 output failed\n");
        pr_err("[NFC] %s: gpio_direction_output failed %d\n", __func__, ret);
        gpio_free(RUBY_GPIO_NFC_INT);
        return;
    }
    tegra_gpio_enable(RUBY_GPIO_NFC_INT);

    gpio_set_value(RUBY_GPIO_NFC_VEN, 1);
    pr_info("%s\n", __func__);

}

static struct pn544_i2c_platform_data nfc_platform_data = {
    .gpio_init  = config_nfc_gpios,
    .irq_gpio = RUBY_GPIO_NFC_INT,
    .ven_gpio = RUBY_GPIO_NFC_VEN,
    .firm_gpio = RUBY_GPIO_NFC_DL,
    .ven_isinvert = 1,
};

static struct i2c_board_info pn544_i2c_boardinfo[] = {
    {
        I2C_BOARD_INFO(PN544_I2C_NAME, 0x50 >> 1),
        .platform_data = &nfc_platform_data,
        .irq = TEGRA_GPIO_TO_IRQ(RUBY_GPIO_NFC_INT),
    },
};

static void edge_nfc_init(void)
{
    i2c_register_board_info(0, pn544_i2c_boardinfo,
            ARRAY_SIZE(pn544_i2c_boardinfo));
}

static void evitareul_gsensor_irq_init(void)
{
	int ret = 0;

	//tegra_gpio_enable(TEGRA_GPIO_PH4);
	//ret = gpio_request(TEGRA_GPIO_PH4, SENSOR_MPU_NAME);
	pr_info("[GSNR] g-sensor irq_start...\n");
	ret = gpio_request(TEGRA_GPIO_PS0, "GSNR_INT");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PS0);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PS0);
		return;
	}
	tegra_gpio_enable(TEGRA_GPIO_PS0);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_KB_ROW8, TEGRA_PUPD_PULL_DOWN);

		
	pr_info("[GSNR] g-sensor irq end...\n");

}

static void evitareul_mpuirq_init(void)
{
	int ret = 0;

	tegra_gpio_enable(TEGRA_GPIO_PJ2);
	ret = gpio_request(TEGRA_GPIO_PJ2, SENSOR_MPU_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PJ2);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PJ2);
		return;
	}
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GMI_CS1_N, TEGRA_PUPD_PULL_DOWN);


	i2c_register_board_info(0, mpu3050_i2c0_boardinfo,
				ARRAY_SIZE(mpu3050_i2c0_boardinfo));
}

static void evitareul_comp_irq_init(void)
{
	int ret = 0;

	// comp int
	ret = gpio_request(TEGRA_GPIO_PN5, "COMP_INT");
	if (ret < 0) {
		pr_err("[COMP] %s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PN5);
	if (ret < 0) {
		pr_err("[COMP] %s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PN5);
		return;
	}

	tegra_gpio_enable(TEGRA_GPIO_PN5);
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_LCD_SDOUT, TEGRA_PUPD_NORMAL);
}

void evitareul_comp_reset_pin(int value)
{
	int ret = 0;
	int gpio = TEGRA_GPIO_PF2;
	ret = gpio_request(gpio, "COMP_RSPN");
	if (ret < 0) {
		pr_err("[COMP]Requesting GPIO %d failes\n", gpio);
		return;
	}
	ret = gpio_direction_output(gpio, value);
	if (ret < 0) {
		pr_err("[COMP]Requesting GPIO %d failes\n", gpio);
		gpio_free(gpio);
		return;
	}
	tegra_gpio_enable(gpio);
}

#include <media/rawchip/Yushan_Platform_Specific.h>
#include <media/rawchip/Yushan_HTC_Functions.h>
//#define CAMERA_REGULATOR

static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data,
						shutdown_temp);
}

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	thermal_device->name = "nct1008";
	thermal_device->data = data;
	thermal_device->id = THERMAL_DEVICE_ID_NCT_EXT;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = nct_get_temp;
	thermal_device->get_temp_low = nct_get_temp_low;
	thermal_device->set_limits = nct_set_limits;
	thermal_device->set_alert = nct_set_alert;
	thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_device_register(thermal_device);
}

static struct nct1008_platform_data evitareul_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
	.probe_callback = nct1008_probe_callback,
	.reg_name = "v_usb_3v3",
};

static struct i2c_board_info evitareul_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PCC2),
		.platform_data = &evitareul_nct1008_pdata,
	}
};

static void evitareul_nct1008_init(void)
{
	int ret;

	tegra_gpio_enable(TEGRA_GPIO_PCC2);
	ret = gpio_request(TEGRA_GPIO_PCC2, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(TEGRA_GPIO_PCC2);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(TEGRA_GPIO_PCC2);
		return;
	}

	i2c_register_board_info(4, evitareul_i2c4_nct1008_board_info,
				ARRAY_SIZE(evitareul_i2c4_nct1008_board_info));
}


static inline void evitareul_msleep(u32 t)
{
	/*
	If timer value is between ( 10us - 20ms),
	usleep_range() is recommended.
	Please read Documentation/timers/timers-howto.txt.
	*/
	usleep_range(t*1000, t*1000 + 500);
}

static struct i2c_board_info evitareul_i2c0_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29028", 0x44),
	}
};

static void evitareul_isl_init(void)
{
	i2c_register_board_info(0, evitareul_i2c0_isl_board_info,
				ARRAY_SIZE(evitareul_i2c0_isl_board_info));
}

static struct pca954x_platform_mode evitareul_pca954x_modes[] = {
	{ .adap_id = PCA954x_I2C_BUS0, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS1, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS2, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS3, .deselect_on_exit = true, },
};

static struct pca954x_platform_data evitareul_pca954x_data = {
	.modes    = evitareul_pca954x_modes,
	.num_modes      = ARRAY_SIZE(evitareul_pca954x_modes),
};

struct evitareul_battery_gpio {
	int gpio;
	const char *label;
};

#define TEGRA_BATTERY_GPIO(_gpio, _label)	\
	{					\
		.gpio = _gpio,			\
		.label = _label,		\
	}

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.gpio_mbat_in = 0,
	.gpio_mbat_in_trigger_level = MBAT_IN_LOW_TRIGGER,
	.guage_driver = GUAGE_TPS80032,
	.charger = SWITCH_CHARGER_TPS80032,
	.vzero_clb_channel = -1,
	.volt_adc_offset = 0,
	.power_off_by_id = 0,
	.sw_temp_25 = TEGRA_GPIO_PU1,
};

static struct platform_device htc_battery_pdev = {
	.name	= "htc_battery",
	.id	= -1,
	.dev	= {
	        .platform_data = &htc_battery_pdev_data,
	},
};

#if 1	/* fixme: for MFG build to disable mbat_in check */
static int __init check_mbat_in_tag(char *get_mbat_in)
{
	if (strlen(get_mbat_in) && !strcmp(get_mbat_in, "false")) {
		htc_battery_pdev_data.power_off_by_id = 0;
	}
	return 1;
}
__setup("mbat_in_check=", check_mbat_in_tag);
#endif

static void evitareul_battery_init(void)
{
	platform_device_register(&htc_battery_pdev);
}

#ifdef CONFIG_SENSORS_INA230
static struct ina230_platform_data ina230_platform = {
	.rail_name = "VDD_AC_BAT",
	.current_threshold = TEGRA_CUR_MON_THRESHOLD,
	.resistor = TEGRA_CUR_MON_RESISTOR,
	.min_cores_online = TEGRA_CUR_MON_MIN_CORES,
};

static struct i2c_board_info evitareul_i2c4_ina230_info[] = {
	{
		I2C_BOARD_INFO("ina230", 0x40),
		.platform_data = &ina230_platform,
		.irq = -1,
	},
};

static int __init evitareul_ina230_init(void)
{
	return i2c_register_board_info(4, evitareul_i2c4_ina230_info,
				       ARRAY_SIZE(evitareul_i2c4_ina230_info));
}
#endif

int __init evitareul_sensors_init(void)
{
	int ret = 0;
#if 0
	evitareul_isl_init();
#endif
	evitareul_nct1008_init();
	//Motion Sensors init
	evitareul_comp_irq_init();
	evitareul_gsensor_irq_init(); 
	evitareul_mpuirq_init();
	evitareul_comp_reset_pin(1);
	psensor_init();
	evitareul_battery_init();
#ifdef CONFIG_SENSORS_INA230
	evitareul_ina230_init();
#endif
	edge_nfc_init();
	return ret;
}

