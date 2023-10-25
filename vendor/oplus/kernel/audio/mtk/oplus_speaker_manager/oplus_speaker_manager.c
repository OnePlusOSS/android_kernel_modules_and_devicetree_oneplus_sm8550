/************************************************************************************
** File: - oplus_audio_speaker_manager.c
** Copyright (C), 2020-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      Add for oplus audio speaker manager
**
** Version: 1.0
** Date created: 11/27/2020
**
** --------------------------- Revision History: --------------------------------
**    <author>       <date>          <desc>
**  Jianqing.Liao   2020.11.27     Add this file
** OPLUS Coding Static Checking Skip
************************************************************************************/

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "oplus_speaker_manager.h"
#include "oplus_speaker_manager_platform.h"
#include "oplus_speaker_manager_codec.h"


static struct list_head oplus_speaker_list = LIST_HEAD_INIT(oplus_speaker_list);
static DEFINE_MUTEX(list_mutex);

static struct oplus_amp_status *contrl_status = NULL;

const char *const ext_amp_vdd_need[] = { "None", "Need" };
const char *const ext_amp_boost_vol_function[] = {"Level_1", "Level_2", "Level_3", "Level_4"};
const char *const ext_amp_mode_function[] = { "Music", "Voice","Fm" };
const char *const ext_amp_mute_function[] = { "Off", "On" };
const char *const ext_amp_speaker_switch_function[] = { "Off", "On" };
const char *const ext_rcv_amp_function[] = { "Off", "On" };

static struct soc_enum oplus_amp_control_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_vdd_need), ext_amp_vdd_need),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_boost_vol_function), ext_amp_boost_vol_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_mode_function), ext_amp_mode_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_mute_function), ext_amp_mute_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_amp_speaker_switch_function), ext_amp_speaker_switch_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_rcv_amp_function), ext_rcv_amp_function),
};

static const struct snd_kcontrol_new oplus_pa_manager_snd_controls[] = {
	SOC_SINGLE_EXT("Ext_Amp_Chipset", SND_SOC_NOPM, 0, 0xff, 0,ext_amp_chipset_get,ext_amp_chipset_set),
	SOC_ENUM_EXT("Ext_Amp_Vdd_Need", (oplus_amp_control_enum)[0], ext_amp_vdd_get, ext_amp_vdd_set),
	SOC_ENUM_EXT("Ext_Amp_Boost_Voltage", (oplus_amp_control_enum)[1], ext_amp_boost_voltage_get, ext_amp_boost_voltage_set),
	SOC_ENUM_EXT("Ext_Amp_Mode", (oplus_amp_control_enum)[2], ext_amp_mode_get, ext_amp_mode_set),
	SOC_ENUM_EXT("Ext_Amp_Force_Mute", (oplus_amp_control_enum)[3], ext_amp_force_mute_get, ext_amp_force_mute_set),
	SOC_ENUM_EXT("SpeakerL_Amp_Switch", (oplus_amp_control_enum)[4], speaker_l_amp_get, speaker_l_amp_set),
	SOC_ENUM_EXT("SpeakerR_Amp_Switch", (oplus_amp_control_enum)[4], speaker_r_amp_get, speaker_r_amp_set),
	SOC_ENUM_EXT("Rcv_Amp_Switch", (oplus_amp_control_enum)[5], rcv_amp_get, rcv_amp_set),
	SOC_ENUM_EXT("RcvL_Amp_Switch", (oplus_amp_control_enum)[5], rcv_l_amp_get, rcv_l_amp_set),
};

/*******************************************************************************
 * chipset ID and VDD
 ******************************************************************************/
int ext_amp_chipset_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if (list_empty(&oplus_speaker_list) || (contrl_status == NULL)) {
		pr_err("%s(),no device regist",__func__);
		ucontrol->value.integer.value[0] = 0;
		return 1;
	}

	list_for_each(p,&oplus_speaker_list) {
		entry = list_entry(p,struct oplus_spk_dev_node,list);
		if (entry->device) {
			contrl_status->chipset |= entry->device->chipset;
		}
	}
	ucontrol->value.integer.value[0] = contrl_status->chipset;

	return 0;
}

int ext_amp_chipset_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}


int ext_amp_vdd_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if (list_empty(&oplus_speaker_list) || (contrl_status == NULL)) {
		pr_err("%s(),no device regist",__func__);
		ucontrol->value.integer.value[0] = 0;
		return 1;
	}

	list_for_each(p,&oplus_speaker_list) {
		entry = list_entry(p,struct oplus_spk_dev_node,list);

		if (entry->device && entry->device->vdd_need) {
			contrl_status->vdd_need = 1;
		}
	}
	ucontrol->value.integer.value[0] = contrl_status->vdd_need;

	return 0;
}

int ext_amp_vdd_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
EXPORT_SYMBOL(ext_amp_vdd_set);

/*******************************************************************************
 * boost voltage setting
 ******************************************************************************/
int ext_amp_boost_voltage_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (contrl_status) {
		ucontrol->value.integer.value[0] = contrl_status->amp_boost_voltage;
		pr_debug("%s(),Ext_Amp_Boost_Voltage get = %d",__func__,contrl_status->amp_boost_voltage);
	} else {
		ucontrol->value.integer.value[0] = 0;
	}

	return 0;
}

int ext_amp_boost_voltage_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if ((ucontrol->value.integer.value[0] >= ARRAY_SIZE(ext_amp_boost_vol_function)) ||
	   (ucontrol->value.integer.value[0] < 0)) {
		pr_err("%s(),return -EINVAL",__func__);
		return -EINVAL;
	}

	if (list_empty(&oplus_speaker_list) || (contrl_status == NULL)) {
		pr_err("%s(),no device regist",__func__);
		return 1;
	}

	contrl_status->amp_boost_voltage = ucontrol->value.integer.value[0];

	list_for_each(p,&oplus_speaker_list) {
		entry = list_entry(p,struct oplus_spk_dev_node,list);
		if (entry->device && entry->device->boost_voltage_set) {
			entry->device->boost_voltage_set(kcontrol, ucontrol);
		}
	}

	pr_debug("%s(),Ext_Amp_Boost_Voltage  set = %d",__func__,contrl_status->amp_boost_voltage);

	return 0;
}

/*******************************************************************************
 * voice setting
 ******************************************************************************/
int ext_amp_mode_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

	if (contrl_status) {
		ucontrol->value.integer.value[0] = contrl_status->amp_mode_setting;
		pr_debug("%s(),Ext_Amp_Voice_Setting = %d",__func__,contrl_status->amp_mode_setting);
	} else {
		ucontrol->value.integer.value[0] = 0;
	}

	return 0;
}

int ext_amp_mode_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if ((ucontrol->value.integer.value[0] >= ARRAY_SIZE(ext_amp_mode_function)) ||
	   (ucontrol->value.integer.value[0] < 0)) {
		pr_err("%s(),return -EINVAL",__func__);
		return -EINVAL;
	}

	if (list_empty(&oplus_speaker_list) || (contrl_status == NULL)) {
		pr_err("%s(),no device regist",__func__);
		return 1;
	}

	contrl_status->amp_mode_setting = ucontrol->value.integer.value[0];

	list_for_each(p,&oplus_speaker_list) {
		entry = list_entry(p,struct oplus_spk_dev_node,list);

		if (entry->device && entry->device->spk_mode_set) {
			entry->device->spk_mode_set(kcontrol, ucontrol);
		}
	}

	pr_debug("%s(),Ext_Amp_Voice_Setting = %d",__func__,contrl_status->amp_mode_setting);

	return 0;
}

/*******************************************************************************
 * force mute setting
 ******************************************************************************/
int ext_amp_force_mute_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (contrl_status) {
		ucontrol->value.integer.value[0] = contrl_status->amp_force_mute_status;
		pr_err("%s(), Ext_Amp_Force_Mute = %d\n", __func__, contrl_status->amp_force_mute_status);
	} else {
		ucontrol->value.integer.value[0] = 0;
	}

	return 0;
}

int ext_amp_force_mute_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if ((ucontrol->value.integer.value[0] >= ARRAY_SIZE(ext_amp_mute_function)) ||
	   (ucontrol->value.integer.value[0] < 0)) {
		pr_err("%s(),return -EINVAL",__func__);
		return -EINVAL;
	}

	if (contrl_status == NULL) {
		pr_err("%s,contrl not init\n",__func__);
		return 1;
	}

	if(ucontrol->value.integer.value[0] == contrl_status->amp_force_mute_status)
		return 1;

	contrl_status->amp_force_mute_status = ucontrol->value.integer.value[0];
	list_for_each(p,&oplus_speaker_list) {
		entry = list_entry(p,struct oplus_spk_dev_node,list);
		if (entry->device && entry->device->speaker_mute_set) {
			entry->device->speaker_mute_set(kcontrol, ucontrol);
		}
	}

	pr_err("%s(), Ext_Amp_Force_Mute = %d\n", __func__, contrl_status->amp_force_mute_status);

	return 0;
}

/*******************************************************************************
 * pa or rcv enable setting
 ******************************************************************************/
int oplus_ext_amp_l_enable(int enable)
{

	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if (list_empty(&oplus_speaker_list) || (contrl_status == NULL)) {
		pr_err("%s(),no device regist",__func__);
		return 1;
	}

	if (!contrl_status->amp_force_mute_status) {
		list_for_each(p,&oplus_speaker_list) {
			entry = list_entry(p,struct oplus_spk_dev_node,list);

			if (entry->device
				&& entry->device->speaker_enable_set
				&& (entry->device->type == L_SPK)) {
				entry->device->speaker_enable_set(enable,SPK_MODE);

				if (entry->device->speaker_protection_set != NULL) {
					entry->device->speaker_protection_set(enable, SPK_MODE);
				}
			}
		}
	}

	contrl_status->enable = enable;
	pr_debug("%s(),ext amp enable:%d",__func__,enable);

	return 0;
}
EXPORT_SYMBOL(oplus_ext_amp_l_enable);

int oplus_ext_amp_r_enable(int enable)
{

	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if (list_empty(&oplus_speaker_list) || (contrl_status == NULL)) {
		pr_err("%s(),no device regist",__func__);
		return 1;
	}

	if (!contrl_status->amp_force_mute_status) {
		list_for_each(p,&oplus_speaker_list) {
			entry = list_entry(p,struct oplus_spk_dev_node,list);

			if (entry->device
				&& entry->device->speaker_enable_set
				&& (entry->device->type == R_SPK)) {
				entry->device->speaker_enable_set(enable,SPK_MODE);

				if (entry->device->speaker_protection_set != NULL) {
					entry->device->speaker_protection_set(enable, SPK_MODE);
				}
			}
		}
	}

	contrl_status->enable = enable;
	pr_debug("%s(),ext amp enable:%d",__func__,enable);

	return 0;
}
EXPORT_SYMBOL(oplus_ext_amp_r_enable);

int oplus_ext_amp_recv_enable(int enable)
{

	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if (list_empty(&oplus_speaker_list) || (contrl_status == NULL)) {
		pr_err("%s(),no device regist",__func__);
		return 1;
	}

	if (!contrl_status->amp_force_mute_status) {
		list_for_each(p,&oplus_speaker_list) {
			entry = list_entry(p,struct oplus_spk_dev_node,list);

			if (entry->device
				&& entry->device->speaker_enable_set
				&& (entry->device->type == R_SPK)) {
				entry->device->speaker_enable_set(enable,RECV_MODE);
			}
		}
	}

	contrl_status->enable = enable;
	pr_debug("%s(),ext amp enable:%d",__func__,enable);

	return 0;
}
EXPORT_SYMBOL(oplus_ext_amp_recv_enable);

int oplus_ext_amp_recv_l_enable(int enable)
{

	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if (list_empty(&oplus_speaker_list) || (contrl_status == NULL)) {
		pr_err("%s(),no device regist", __func__);
		return 1;
	}

	if (!contrl_status->amp_force_mute_status) {
		list_for_each(p, &oplus_speaker_list) {
			entry = list_entry(p, struct oplus_spk_dev_node,list);

			if (entry->device
				&& entry->device->speaker_enable_set
				&& (entry->device->type == L_SPK)) {
				entry->device->speaker_enable_set(enable, RECV_MODE);
			}
		}
	}

	contrl_status->rcv_l_enable = enable;
	pr_debug("%s(),ext amp enable:%d", __func__, enable);

	return 0;
}
EXPORT_SYMBOL(oplus_ext_amp_recv_l_enable);

int speaker_l_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (contrl_status) {
		ucontrol->value.integer.value[0] = contrl_status->spkl_enable;
		pr_err("%s(), SpeakerL_Amp_Switch = %d\n", __func__, contrl_status->spkl_enable);
	} else {
		ucontrol->value.integer.value[0] = 0;
	}

	return 0;
}

int speaker_l_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	int ret = 0;

	if ((value >= ARRAY_SIZE(ext_amp_speaker_switch_function)) || (value < 0)) {
		pr_err("%s(),return -EINVAL",__func__);
		return -EINVAL;
	}

	ret = oplus_ext_amp_l_enable(value);
	pr_info("%s, %d, value = %d, ret = %d\n", __func__, __LINE__, value, ret);
	return 0;
}

int speaker_r_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (contrl_status) {
		ucontrol->value.integer.value[0] = contrl_status->spkr_enable;
		pr_err("%s(), SpeakerR_Amp_Switch = %d\n", __func__, contrl_status->spkr_enable);
	} else {
		ucontrol->value.integer.value[0] = 0;
	}

	return 0;
}

int speaker_r_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	int ret = 0;

	if ((value >= ARRAY_SIZE(ext_amp_speaker_switch_function)) || (value < 0)) {
		pr_err("%s(),return -EINVAL",__func__);
		return -EINVAL;
	}

	ret = oplus_ext_amp_r_enable(value);
	pr_info("%s, %d, value = %d, ret = %d\n", __func__, __LINE__, value, ret);
	return 0;
}

int rcv_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (contrl_status) {
		ucontrol->value.integer.value[0] = contrl_status->rcv_enable;
		pr_err("%s(), Rcv_Amp_Switch = %d\n", __func__, contrl_status->rcv_enable);
	} else {
		ucontrol->value.integer.value[0] = 0;
	}

	return 0;
}

int rcv_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	int ret = 0;

	if ((value >= ARRAY_SIZE(ext_rcv_amp_function)) || (value < 0)) {
		pr_err("%s(),return -EINVAL",__func__);
		return -EINVAL;
	}

	ret = oplus_ext_amp_recv_enable(value);
	pr_info("%s, %d, value = %d, ret = %d\n", __func__, __LINE__, value, ret);
	return 0;
}

int rcv_l_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	if (contrl_status) {
		ucontrol->value.integer.value[0] = contrl_status->rcv_l_enable;
		pr_err("%s(), Rcv_Amp_Switch = %d\n", __func__, contrl_status->rcv_l_enable);
	} else {
		ucontrol->value.integer.value[0] = 0;
	}

	return 0;
}

int rcv_l_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	int ret = 0;

	if ((value >= ARRAY_SIZE(ext_rcv_amp_function)) || (value < 0)) {
		pr_err("%s(),return -EINVAL",__func__);
		return -EINVAL;
	}

	ret = oplus_ext_amp_recv_l_enable(value);
	pr_info("%s, %d, value = %d, ret = %d\n", __func__, __LINE__, value, ret);
	return 0;
}


/*******************************************************************************
 * add PA manager 
 ******************************************************************************/
int oplus_add_pa_manager_snd_controls(struct snd_soc_component *cmpnt)
{
	return snd_soc_add_component_controls(cmpnt, oplus_pa_manager_snd_controls,
		ARRAY_SIZE(oplus_pa_manager_snd_controls));
}
EXPORT_SYMBOL(oplus_add_pa_manager_snd_controls);

/*******************************************************************************
 * get PA device
 ******************************************************************************/
struct oplus_speaker_device* get_speaker_dev(enum oplus_pa_type pa_type)
{
	struct list_head *p;
	struct oplus_spk_dev_node *entry;

	if (list_empty(&oplus_speaker_list)) {
		pr_err("%s, %d, oplus_speaker_list is empty!\n",__func__, __LINE__);

		return NULL;
	}

	list_for_each(p, &oplus_speaker_list) {
		entry = list_entry(p, struct oplus_spk_dev_node, list);
		if (entry->device && entry->device->type == pa_type) {
			return entry->device;
		}
	}

	return NULL;
}
EXPORT_SYMBOL(get_speaker_dev);

/*******************************************************************************
 * pa register and pa remove
 ******************************************************************************/
void *oplus_speaker_pa_register(struct oplus_speaker_device *device)
{
	struct oplus_spk_dev_node *new_node;

	if (device == NULL) {
		pr_err("[%s],bad param!\n",__func__);
		return NULL;
	}

	if (contrl_status == NULL) {
		contrl_status = kzalloc(sizeof(struct oplus_amp_status), GFP_KERNEL);
		if (contrl_status == NULL) {
			pr_err("[%s],control status kzalloc fail!\n",__func__);
			return NULL;
		}
		contrl_status->chipset = 0;
		contrl_status->enable = 0;
		contrl_status->vdd_need = 0;
		contrl_status->amp_boost_voltage = 0;
		contrl_status->amp_mode_setting = 0;
		contrl_status->amp_force_mute_status = 0;
		pr_info("%s(),contrl status init \n", __func__);
	}

	new_node = kzalloc(sizeof(struct oplus_spk_dev_node), GFP_KERNEL);
	if (new_node == NULL) {
		pr_err("[%s],node kzalloc fail!\n",__func__);
		return NULL;
	}

	new_node->device = device;

	mutex_lock(&list_mutex);
	list_add(&new_node->list,&oplus_speaker_list);
	mutex_unlock(&list_mutex);
	pr_info("%s(),register id:%d, type:%d \n", __func__, device->chipset, device->type);

	return new_node;
}
EXPORT_SYMBOL(oplus_speaker_pa_register);

int oplus_speaker_pa_remove(void *node)
{
	struct oplus_spk_dev_node *p = node;

	if (node == NULL) {
		pr_err("[%s],bab param!\n",__func__);
		return -1;
	}

	mutex_lock(&list_mutex);
	list_del(&p->list);
	mutex_unlock(&list_mutex);

	pr_info("%s(),register id:%d, type:%d \n", __func__, p->device->chipset, p->device->type);
	kfree(node);

	if (list_empty(&oplus_speaker_list) && contrl_status) {
		kfree(contrl_status);
		contrl_status = NULL;
	}

	return 0;
}
EXPORT_SYMBOL(oplus_speaker_pa_remove);

static int __init oplus_pa_manager_init(void)
{
	pr_info("%s() \n", __func__);
	return 0;
}
static void __exit oplus_pa_manager_exit(void)
{
	pr_info("%s() \n", __func__);
}
module_init(oplus_pa_manager_init);
module_exit(oplus_pa_manager_exit);

MODULE_AUTHOR("< oplus >");
MODULE_DESCRIPTION("PA Manager ASoC driver");
MODULE_LICENSE("GPL");


