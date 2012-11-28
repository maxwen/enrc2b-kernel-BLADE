#include "htc_audio_power.h"

static struct aic3008_power *aic3008_power_ctl;
static int pcbid;

static void aic3008_powerinit(void)
{
	power_config("AUD_MCLK_EN", TEGRA_GPIO_PX7, INIT_OUTPUT_HIGH);
	power_config("AUD_AIC3008_RST#", TEGRA_GPIO_PW5, INIT_OUTPUT_HIGH);

	power_config("AUD_MCLK", TEGRA_GPIO_PW4, INIT_OUTPUT_LOW);
	sfio_deconfig("AUD_MCLK", TEGRA_GPIO_PW4);

	if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
		power_config("AUD_HP_GAIN_CONTROL", TEGRA_GPIO_PD1, INIT_OUTPUT_LOW);
		power_config("AUD_SPK_RST#", TEGRA_GPIO_PP6, INIT_OUTPUT_HIGH);
		power_config("AUD_HEADPHONE_EN", TEGRA_GPIO_PP7, INIT_OUTPUT_LOW);
	} else {
		power_config("AUD_SPK_EN", TEGRA_GPIO_PP6, INIT_OUTPUT_LOW);
		power_config("AUD_LINEOUT_EN", TEGRA_GPIO_PP7, INIT_OUTPUT_LOW);
	}

	common_init();

	spin_lock_init(&aic3008_power_ctl->spin_lock);
	aic3008_power_ctl->isPowerOn = true;

	return;
}

static void aic3008_resume(void)
{
	spin_lock(&aic3008_power_ctl->spin_lock);
	power_config("AUD_MCLK_EN", TEGRA_GPIO_PX7, GPIO_OUTPUT);
	common_config();
	aic3008_power_ctl->isPowerOn = true;
	spin_unlock(&aic3008_power_ctl->spin_lock);
	return;
}

static void aic3008_suspend(void)
{
	spin_lock(&aic3008_power_ctl->spin_lock);
	power_deconfig("AUD_MCLK_EN", TEGRA_GPIO_PX7, GPIO_OUTPUT);
	common_deconfig();
	aic3008_power_ctl->isPowerOn = false;
	spin_unlock(&aic3008_power_ctl->spin_lock);
	return;
}

static void aic3008_mic_powerup(void)
{
	/* No Need to Mic PowerUp */
	return;
}

static void aic3008_mic_powerdown(void)
{
	/* No Need to Mic PowerDown */
	return;
}

static void aic3008_amp_powerup(int type)
{
	switch (type) {
	case HEADSET_AMP:
		if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
			mdelay(50);
			power_config("AUD_HEADPHONE_EN", TEGRA_GPIO_PP7, GPIO_OUTPUT);
		}
		break;
	case SPEAKER_AMP:
		mdelay(50);
		if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
#if (defined(CONFIG_SND_AMP_TFA9887))
			set_tfa9887_spkamp(1, 0);
#endif
		} else {
			power_config("AUD_SPK_EN", TEGRA_GPIO_PP6, GPIO_OUTPUT);
		}
		break;
	case DOCK_AMP:
		mdelay(50);
		if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
		} else {
			power_config("AUD_LINEOUT_EN", TEGRA_GPIO_PP7, GPIO_OUTPUT);
		}
		dock_config("TEGRA_GPIO_DESK_AUD", TEGRA_GPIO_PCC5, true, true);
		break;
	default:
		AUD_ERR("aic3008_amp_powerup unknown type %d\n", type);
		break;
	}
	return;
}

static void aic3008_amp_powerdown(int type)
{
	switch (type) {
	case HEADSET_AMP:
		if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
			power_deconfig("AUD_HEADPHONE_EN", TEGRA_GPIO_PP7, GPIO_OUTPUT);
		}
		break;
	case SPEAKER_AMP:
		if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
#if (defined(CONFIG_SND_AMP_TFA9887))
			set_tfa9887_spkamp(0, 0);
#endif
		} else {
			power_deconfig("AUD_SPK_EN", TEGRA_GPIO_PP6, GPIO_OUTPUT);
		}
		break;
	case DOCK_AMP:
		if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
		} else {
			power_deconfig("AUD_LINEOUT_EN", TEGRA_GPIO_PP7, GPIO_OUTPUT);
		}
		dock_config("TEGRA_GPIO_DESK_AUD", TEGRA_GPIO_PCC5, false, true);
		break;
	default:
		AUD_ERR("aic3008_amp_powerdown unknown type %d\n", type);
		break;
	}
	return;
}

static void aic3008_i2s_control(int dsp_enum)
{
	switch (dsp_enum) {
	case Phone_Default:
	case Phone_BT:
	case VOIP_BT:
	case VOIP_BT_HW_AEC:
		if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
			power_config("AUD_BT_SEL", TEGRA_GPIO_PK5, GPIO_OUTPUT);
		}
		break;
	case FM_Headset:
	case FM_Speaker:
		power_config("AUD_FM_SEL", TEGRA_GPIO_PK6, GPIO_OUTPUT);
		break;
	default:
		if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
			power_deconfig("AUD_BT_SEL", TEGRA_GPIO_PK5, GPIO_OUTPUT);
		}
		power_deconfig("AUD_FM_SEL", TEGRA_GPIO_PK6, GPIO_OUTPUT);
		break;
	}
	return;
}

static void aic3008_hs_vol_control(int db)
{
	switch (db) {
		case BEATS_GAIN_ON:
			if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
				power_config("AUD_HP_GAIN_CONTROL", TEGRA_GPIO_PD1, GPIO_OUTPUT);
			}
			break;
		case BEATS_GAIN_OFF:
			if (pcbid >= PROJECT_PHASE_XB || board_get_sku_tag() == 0x34600) {
				power_deconfig("AUD_HP_GAIN_CONTROL", TEGRA_GPIO_PD1, GPIO_OUTPUT);
			}
			break;
		default:
			break;
	}
	return;
}

int __init enrc2u_audio_codec_init(struct htc_asoc_platform_data *pdata)
{
	pcbid = htc_get_pcbid_info();

	aic3008_power_ctl = &pdata->aic3008_power;

	aic3008_power_ctl->mic_switch = false;
	aic3008_power_ctl->amp_switch = true;
	aic3008_power_ctl->i2s_switch = true;
	aic3008_power_ctl->hs_vol_control = true;
	aic3008_power_ctl->powerinit = aic3008_powerinit;
	aic3008_power_ctl->resume = aic3008_resume;
	aic3008_power_ctl->suspend = aic3008_suspend;
	aic3008_power_ctl->mic_powerup = aic3008_mic_powerup;
	aic3008_power_ctl->mic_powerdown = aic3008_mic_powerdown;
	aic3008_power_ctl->amp_powerup = aic3008_amp_powerup;
	aic3008_power_ctl->amp_powerdown = aic3008_amp_powerdown;
	aic3008_power_ctl->i2s_control = aic3008_i2s_control;
	aic3008_power_ctl->headset_vol_control = aic3008_hs_vol_control;
}

