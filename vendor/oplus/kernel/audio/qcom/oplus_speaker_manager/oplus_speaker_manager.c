/************************************************************************************
** File: - oplus_audio_speaker_manager.c
**
** Copyright (C), 2020-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      Add for oplus audio speaker manager
**
** Version: 1.0
**
** --------------------------- Revision History: --------------------------------
**    <author>       <date>          <desc>
************************************************************************************/

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "oplus_speaker_manager.h"
#include "oplus_speaker_manager_codec.h"

#define MAX_VOLTAGE_LEVEL_COUNT (2)

const char *const ext_amp_manufacturers[] = {
	"None",
	"Awinic",
	"Silicon",
	"Unknown"
};

const char *const ext_amp_vdd_need[] = { "None", "Need" };
const char *const ext_amp_boost_vol_text[] = {"Level_1", "Level_2", "Level_3", "Level_4"};
const char *const ext_amp_speaker_switch_function[] = { "Off", "On" };
const char *const ext_rcv_amp_function[] = { "Off", "On" };
const char *const ext_amp_speaker_mode_function[] = { "Off", "Music", "Voice", "Fm", "Rcv" };
const char *const ext_amp_voice_function[] = { "Off", "On" };
const char *const ext_amp_mute_function[] = { "Off", "On" };

static struct soc_enum oplus_amp_info_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_manufacturers), ext_amp_manufacturers),
};

static struct soc_enum oplus_amp_control_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_speaker_switch_function), ext_amp_speaker_switch_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_rcv_amp_function), ext_rcv_amp_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_speaker_mode_function), ext_amp_speaker_mode_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_vdd_need), ext_amp_vdd_need),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_boost_vol_text), ext_amp_boost_vol_text),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_mute_function), ext_amp_mute_function),
};

static const struct snd_kcontrol_new oplus_pa_manager_snd_controls[] = {
	SOC_SINGLE_EXT("Ext_Amp_Chipset", SND_SOC_NOPM, 0, 0xff, 0, ext_amp_chipset_get, NULL),
	SOC_ENUM_EXT("SpeakerL_Amp_MFR", (oplus_amp_info_enum)[0], speaker_l_mfr_get, NULL),
	SOC_ENUM_EXT("SpeakerR_Amp_MFR", (oplus_amp_info_enum)[0], speaker_r_mfr_get, NULL),
	SOC_ENUM_EXT("SpeakerL_Amp_Switch", (oplus_amp_control_enum)[0], speaker_l_amp_get, speaker_l_amp_set),
	SOC_ENUM_EXT("SpeakerR_Amp_Switch", (oplus_amp_control_enum)[0], speaker_r_amp_get, speaker_r_amp_set),
	SOC_ENUM_EXT("Rcv_Amp_Switch", (oplus_amp_control_enum)[1], rcv_amp_get, rcv_amp_set),
	SOC_ENUM_EXT("Ext_Amp_Mode", (oplus_amp_control_enum)[2], ext_amp_mode_get, ext_amp_mode_set),
	SOC_ENUM_EXT("Ext_Amp_Vdd_Need", (oplus_amp_control_enum)[3], ext_amp_vdd_get, ext_amp_vdd_set),
	SOC_ENUM_EXT("Ext_Amp_Boost_Volume", (oplus_amp_control_enum)[4], ext_amp_boost_volume_get, ext_amp_boost_volume_set),
	SOC_ENUM_EXT("Speaker_Mute_Switch", (oplus_amp_control_enum)[5], ext_amp_force_mute_get, ext_amp_force_mute_set),
};

static const struct snd_soc_dapm_widget oplus_analog_pa_manager_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("OPLUS_SPKR_DRV", oplus_spkr_pa_event),
};

static const struct snd_soc_dapm_route oplus_analog_pa_manager_dapm_map[] = {
	{"OPLUS_SPKR_DRV", NULL, "RX INT2 MIX2"},
	{"AUX_OUT", NULL, "OPLUS_SPKR_DRV"},
};

/*------------------------------------------------------------------------------*/
extern struct oplus_amp_status *contrl_status;
/*------------------------------------------------------------------------------*/
int ext_amp_chipset_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = contrl_status->chipset;

	pr_debug("%s, %d, ucontrol->value.integer.value[0] = %#x\n", __func__, __LINE__, ucontrol->value.integer.value[0]);

	return 0;
}

enum oplus_pa_manufacturer oplus_speaker_mfr_get(enum oplus_pa_type pa_type)
{
	enum oplus_pa_manufacturer mrf = MFR_NONE;

	struct oplus_speaker_device *speaker_device = get_speaker_dev(pa_type);
	if (speaker_device == NULL) {
		pr_err("%s, %d, pa_type = %d, speaker_device == NULL\n", __func__, __LINE__, pa_type);
	} else {
		mrf = speaker_device->speaker_manufacture;
	}

	return mrf;
}

int speaker_l_mfr_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	enum oplus_pa_manufacturer value = oplus_speaker_mfr_get(L_SPK);

	if (value >= MFR_NONE && value < MFR_COUNT) {
		ucontrol->value.integer.value[0] = value;
	} else if (value >= MFR_COUNT) {
		ucontrol->value.integer.value[0] = MFR_COUNT;
	} else {
		ucontrol->value.integer.value[0] = MFR_NONE;
	}

	pr_err("%s, %d, ucontrol->value.integer.value[0] = %d\n", __func__, __LINE__, ucontrol->value.integer.value[0]);

	return 0;
}

int speaker_r_mfr_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	enum oplus_pa_manufacturer value = oplus_speaker_mfr_get(R_SPK);

	if (value >= MFR_NONE && value < MFR_COUNT) {
		ucontrol->value.integer.value[0] = value;
	} else if (value >= MFR_COUNT) {
		ucontrol->value.integer.value[0] = MFR_COUNT;
	} else {
		ucontrol->value.integer.value[0] = MFR_NONE;
	}

	pr_err("%s, %d, ucontrol->value.integer.value[0] = %d\n", __func__, __LINE__, ucontrol->value.integer.value[0]);

	return 0;
}

int oplus_speaker_amp_get(enum oplus_pa_type pa_type)
{
	int value = 0;

	struct oplus_speaker_device *speaker_device = get_speaker_dev(pa_type);
	if (speaker_device == NULL) {
		pr_err("%s, %d, pa_type = %d, speaker_device == NULL\n", __func__, __LINE__, pa_type);

		return -ENODEV;
	} else {
		if (speaker_device->speaker_enable_get != NULL) {
			value = speaker_device->speaker_status;
		} else if (speaker_device->speaker_mode_get != NULL) {
			value = speaker_device->speaker_mode;
		}
	}

	return value;
}

int oplus_speaker_amp_set(enum oplus_pa_type pa_type, int value)
{
	int ret = 0;
	struct oplus_speaker_device *speaker_device = get_speaker_dev(pa_type);
	if (speaker_device == NULL) {
		pr_err("%s, %d, pa_type = %d, speaker_device == NULL\n", __func__, __LINE__, pa_type);

		return -ENODEV;
	} else if (contrl_status == NULL) {
		pr_err("%s, %d, contrl_status == NULL\n", __func__, __LINE__);

		return -ENOMEM;
	} else if (!contrl_status->amp_force_mute_status) {
		if (speaker_device->speaker_enable_set != NULL) {
			pr_debug("%s, %d, channel = %d, speaker_device->speaker_status = %d, value = %d\n",
				__func__, __LINE__, pa_type - L_SPK, speaker_device->speaker_status, value);

			if (speaker_device->speaker_status != value) {
				if (value != WORK_STATUS_OFF) {
					speaker_device->speaker_mode = contrl_status->amp_mode_setting;
				} else {
					speaker_device->speaker_mode = WORK_MODE_OFF;
				}

				pr_debug("%s, %d, channel = %d, value = %d, contrl_status->amp_mode_setting = %d\n",
					__func__, __LINE__, pa_type - L_SPK, value, contrl_status->amp_mode_setting);

				ret = speaker_device->speaker_enable_set(speaker_device, value);
				if (ret != 0) {
					pr_err("%s, %d, ret = %d\n", __func__, __LINE__, ret);
				} else {
					speaker_device->speaker_status = value;
				}

				if (speaker_device->speaker_protection_set != NULL) {
					pr_debug("%s, %d, channel = %d, value = %d\n", __func__, __LINE__, pa_type - L_SPK, value);

					speaker_device->speaker_protection_set(speaker_device, value);
				}
			}
		} else {
			pr_debug("%s, %d, channel = %d, speaker_mode_set == NULL\n", __func__, __LINE__, pa_type - L_SPK);
		}
	}

	return 0;
}

int speaker_l_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = 0;

	value = oplus_speaker_amp_get(L_SPK);

	ucontrol->value.integer.value[0] = (value >= 0) ? value : 0;

	pr_debug("%s, %d, status = %d\n", __func__, __LINE__, ucontrol->value.integer.value[0]);

	return 0;
}

int speaker_l_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];

	int ret = 0;

	ret = oplus_speaker_amp_set(L_SPK, value);

	pr_debug("%s, %d, value = %d, ret = %d\n", __func__, __LINE__, value, ret);

	return 0;
}

int speaker_r_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = 0;

	value = oplus_speaker_amp_get(R_SPK);

	ucontrol->value.integer.value[0] = (value >= 0) ? value : 0;

	pr_debug("%s, %d, status = %d\n", __func__, __LINE__, ucontrol->value.integer.value[0]);

	return 0;
}

int speaker_r_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];

	int ret = 0;

	ret = oplus_speaker_amp_set(R_SPK, value);

	pr_debug("%s, %d, value = %d, ret = %d\n", __func__, __LINE__, value, ret);

	return 0;
}

int rcv_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct oplus_speaker_device *speaker_device = get_speaker_dev(R_SPK);

	ucontrol->value.integer.value[0] = 0;

	if (speaker_device == NULL) {
		pr_err("%s, %d, speaker_device == NULL\n", __func__, __LINE__);

		return -ENODEV;
	} else if (contrl_status == NULL) {
		pr_err("%s, %d, contrl_status == NULL\n", __func__, __LINE__);

		return -ENOMEM;
	} else {
		ucontrol->value.integer.value[0] = contrl_status->rcv_enable;
	}

	pr_debug("%s, %d, status = %d\n", __func__, __LINE__, ucontrol->value.integer.value[0]);

	return 0;
}

int rcv_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];

	enum oplus_pa_work_mode work_mode = WORK_MODE_OFF;
	int protection_needed = 0;

	struct oplus_speaker_device *speaker_device = get_speaker_dev(R_SPK);
	int ret = 0;

	if (value == 0) {
		work_mode = WORK_MODE_OFF;
	} else {
		work_mode = WORK_MODE_RECEIVER;
		protection_needed = 1;
	}

	if (speaker_device == NULL) {
		pr_err("%s, %d, speaker_device == NULL\n", __func__, __LINE__);

		return -ENODEV;
	} else if (contrl_status == NULL) {
		pr_err("%s, %d, contrl_status == NULL\n", __func__, __LINE__);
	} else {
		if (speaker_device->speaker_enable_set != NULL) {
			pr_debug("%s, %d, speaker_device->speaker_status = %d, contrl_status->rcv_enable = %d\n",
				__func__, __LINE__, speaker_device->speaker_status, contrl_status->rcv_enable);

			if (speaker_device->speaker_mode != work_mode) {
				pr_debug("%s, %d, channel = %d, value = %d, contrl_status->amp_mode_setting = %d\n",
				__func__, __LINE__, R_SPK - L_SPK, value, contrl_status->amp_mode_setting);

				speaker_device->speaker_mode = work_mode;
				ret = speaker_device->speaker_enable_set(speaker_device, value);
				if (ret != 0) {
					pr_err("%s, %d, ret = %d\n", __func__, __LINE__, ret);
					speaker_device->speaker_mode = WORK_MODE_OFF;
				} else {
					contrl_status->amp_mode_setting = work_mode;
					contrl_status->rcv_enable = (speaker_device->speaker_mode == WORK_MODE_RECEIVER) ? 1:0;
				}

				if (speaker_device->speaker_protection_set != NULL) {
					pr_debug("%s, %d, channel = %d, value = %d\n", __func__, __LINE__, R_SPK - L_SPK, protection_needed);

					speaker_device->speaker_protection_set(speaker_device, protection_needed);
				}
			}
		} else {
			pr_debug("%s, %d, channel = %d, speaker_mode_set == NULL\n", __func__, __LINE__, R_SPK - L_SPK);
		}
	}

	return 0;
}

int ext_amp_mode_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;

	if (contrl_status == NULL) {
		pr_err("%s, %d, contrl_status == NULL\n", __func__, __LINE__);
	} else {
		ucontrol->value.integer.value[0] = contrl_status->amp_mode_setting;
	}

	pr_debug("%s, %d, ucontrol->value.integer.value[0] = %d\n", __func__, __LINE__, ucontrol->value.integer.value[0]);


	return 0;
}

int ext_amp_mode_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (contrl_status == NULL) {
		pr_err("%s, %d, contrl_status == NULL\n", __func__, __LINE__);

		return -ENODEV;
	} else {
		contrl_status->amp_mode_setting = ucontrol->value.integer.value[0];

		pr_debug("%s, %d, contrl_status->amp_mode_setting = %d\n", __func__, __LINE__, contrl_status->amp_mode_setting);
	}

	return 0;
}

int ext_amp_vdd_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

int ext_amp_vdd_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

int ext_amp_boost_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (contrl_status == NULL) {
		pr_err("%s, %d, contrl_status == NULL\n", __func__, __LINE__);
		ucontrol->value.integer.value[0] = MAX_VOLTAGE_LEVEL_COUNT - 1;
		return -ENODEV;
	} else {
		ucontrol->value.integer.value[0] = contrl_status->amp_boost_volume;
	}

	pr_debug("%s, %d, ucontrol->value.integer.value[0] = %d\n", __func__, __LINE__, ucontrol->value.integer.value[0]);

	return 0;
}

int ext_amp_boost_volume_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	enum oplus_pa_type pa_index = L_SPK;
	struct oplus_speaker_device *speaker_device = NULL;

	if (contrl_status == NULL) {
		pr_err("%s, %d, contrl_status == NULL\n", __func__, __LINE__);
		return -ENODEV;
	} else {
		contrl_status->amp_boost_volume = ucontrol->value.integer.value[0];

		for (pa_index = L_SPK; pa_index < ALL_SPK; pa_index++) {
			speaker_device = get_speaker_dev(pa_index);
			if (speaker_device == NULL) {
				pr_info("%s, %d, pa_index = %d, No more speaker_device\n", __func__, __LINE__, pa_index);

				break;
			} else {
				if (speaker_device->boost_voltage_set != NULL) {
					pr_debug("%s, %d, speaker_device->boost_voltage_set != NULL\n", __func__, __LINE__);
					speaker_device->boost_voltage_set(speaker_device, contrl_status->amp_boost_volume);
				} else {
					pr_debug("%s, %d, pa_index = %d, speaker_device->boost_voltage_set == NULL\n", __func__, __LINE__, pa_index);
				}
			}
		}
	}


	return 0;
}

int ext_amp_force_mute_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (contrl_status == NULL) {
		ucontrol->value.integer.value[0] = 0;
		pr_err("%s, %d, contrl_status == NULL!\n", __func__, __LINE__);
	} else {
		ucontrol->value.integer.value[0] = contrl_status->amp_force_mute_status;
	}

	pr_debug("%s(), ucontrol->value.integer.value[0] = %d\n", __func__, ucontrol->value.integer.value[0]);

	return 0;
}

int ext_amp_force_mute_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	enum oplus_pa_type pa_index = L_SPK;
	struct oplus_speaker_device *speaker_device = NULL;

	if (contrl_status == NULL) {
		ucontrol->value.integer.value[0] = 0;
		pr_err("%s, %d, contrl_status == NULL!\n", __func__, __LINE__);
		return -ENODEV;
	} else if (contrl_status->amp_force_mute_status == ucontrol->value.integer.value[0]) {
		pr_info("%s, %d, Speaker mute set is already %s\n", __func__, __LINE__, contrl_status->amp_force_mute_status == 1 ? "On": "Off");
	} else {
		contrl_status->amp_force_mute_status = ucontrol->value.integer.value[0];
		for (pa_index = L_SPK; pa_index < ALL_SPK; pa_index++) {
			speaker_device = get_speaker_dev(pa_index);
			if (speaker_device == NULL) {
				pr_err("%s, %d, pa_index = %d, No more speaker_device\n", __func__, __LINE__, pa_index);
				break;
			} else if (speaker_device->speaker_mute_set == NULL) {
				pr_debug("%s, %d, pa_index = %d, speaker_device->speaker_mute_set == NULL\n", __func__, __LINE__, pa_index);
			} else {
				speaker_device->speaker_mute_set(speaker_device, contrl_status->amp_force_mute_status);
				pr_info("%s, %d, force mute control is working in %s mode, pa_index = %d, contrl_status->amp_force_mute_status = %d\n",
					 __func__, __LINE__, contrl_status->amp_force_mute_status == 1 ? "On" : "Off", pa_index, contrl_status->amp_force_mute_status);
			}
		}
	}

	return 0;
}
/*------------------------------------------------------------------------------*/
int oplus_spkr_pa_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{
	pr_debug("%s, w->name = %s, event = %#x \n", __func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU :
		break;
	case SND_SOC_DAPM_PRE_PMD :
		oplus_speaker_amp_set(L_SPK, WORK_STATUS_OFF);
		if (contrl_status->chipset == ALL_SPK) {
			oplus_speaker_amp_set(R_SPK, WORK_STATUS_OFF);
		}
		break;
	default :
		pr_err("[  err][%s] event = %#x \n", __func__, event);
		break;
	}

	return 0;
}
/*------------------------------------------------------------------------------*/
int oplus_add_pa_manager_snd_controls(struct snd_soc_component *component)
{
	return snd_soc_add_component_controls(component, oplus_pa_manager_snd_controls,
		ARRAY_SIZE(oplus_pa_manager_snd_controls));
}
EXPORT_SYMBOL(oplus_add_pa_manager_snd_controls);
/*------------------------------------------------------------------------------*/
int oplus_add_analog_pa_manager_dapm(struct snd_soc_dapm_context *dapm)
{
	int ret = 0;

	ret = snd_soc_dapm_new_controls(dapm, oplus_analog_pa_manager_dapm_widgets,
			ARRAY_SIZE(oplus_analog_pa_manager_dapm_widgets));
	if (ret < 0) {
		pr_err("%s: failed to add controls\n", __func__);

		return ret;
	}

	snd_soc_dapm_ignore_suspend(dapm, "OPLUS_SPKR_DRV");

	ret = snd_soc_dapm_add_routes(dapm, oplus_analog_pa_manager_dapm_map,
			ARRAY_SIZE(oplus_analog_pa_manager_dapm_map));
	if (ret < 0) {
		pr_err("%s: failed to add routes\n", __func__);

		return ret;
	}

	pr_debug("%s, %d", __func__, __LINE__);

	return ret;
}
EXPORT_SYMBOL(oplus_add_analog_pa_manager_dapm);
/*------------------------------------------------------------------------------*/
static int __init oplus_pa_manager_init(void)
{
	if (contrl_status == NULL) {
		contrl_status = kzalloc(sizeof(struct oplus_amp_status), GFP_KERNEL);
		if (contrl_status == NULL) {
			pr_err("[%s],control status kzalloc fail!\n", __func__);
			return -ENOMEM;
		}
		contrl_status->chipset = 0;
		contrl_status->spkl_enable = 0;
		contrl_status->spkr_enable = 0;
		contrl_status->rcv_enable = 0;
		contrl_status->vdd_need = 0;
		contrl_status->amp_boost_volume = 0;
		contrl_status->amp_mode_setting = 0;
		contrl_status->amp_force_mute_status = 0;
		pr_info("%s(),control status init \n", __func__);
	}

	pr_err("%s, %d, sizeof(oplus_speaker_device) = %d\n", __func__, __LINE__, sizeof(struct oplus_speaker_device));

	return 0;
}

static void __exit oplus_pa_manager_exit(void)
{
	if (contrl_status != NULL) {
		kfree(contrl_status);
		contrl_status = NULL;
	}

	pr_err("%s, %d\n", __func__, __LINE__);
}

module_init(oplus_pa_manager_init);
module_exit(oplus_pa_manager_exit);

MODULE_DESCRIPTION("PA Manager ASoC driver");
MODULE_LICENSE("GPL");
