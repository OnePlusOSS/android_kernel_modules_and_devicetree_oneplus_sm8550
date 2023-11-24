/************************************************************************************
** File: - oplus_audio_speaker_manager_codec.h
** Copyright (C), 2020-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      Add for oplus audio speaker manager.This header file id provided
**      to the third party driver.
**
** Version: 1.0
** Date created: 26/05/2021
**
** --------------------------- Revision History: --------------------------------
**    <author>       <date>          <desc>
**  Kun.Zhao   2021.05.26     Add this file
** OPLUS Coding Static Checking Skip
************************************************************************************/

#ifndef __OPLUS_AUDIO_SPEAKER_CODEC_H__
#define __OPLUS_AUDIO_SPEAKER_CODEC_H__

#include <sound/soc.h>

enum oplus_pa_chipset_def {
	MFR_NXP = 0x00,
	MFR_GOODIX = 0x00,
	MFR_AWINIC = 0x01,
	MFR_SI = 0x02,
	MFR_COUNT = 0x04,
};

enum oplus_pa_type {
	L_SPK = 0x1,
	R_SPK = 0x2,
	ALL_SPK = 0x3,
};

enum oplus_pa_mode {
	SPK_MODE = 0,
	RECV_MODE = 1,
};

struct oplus_amp_status {
	uint32_t chipset;
	int enable;
	int vdd_need;
	int spkl_enable;
	int spkr_enable;
	int rcv_enable;
	int rcv_l_enable;
	int amp_boost_voltage;
	int amp_mode_setting;
	int amp_force_mute_status;
};

struct oplus_speaker_device {
	uint32_t chipset;
	int type;
	int vdd_need;
	void (*speaker_enable_set)(int enable,int mode);
	void (*speaker_protection_set)(int enable, int mode);
	int (*boost_voltage_set)(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
	int (*boost_voltage_get)(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
	int (*spk_mode_set)(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
	int (*spk_mode_get)(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
	int (*speaker_mute_set)(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
	int (*speaker_mute_get)(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
};

struct oplus_spk_dev_node {
	struct oplus_speaker_device *device;
	struct list_head list;
};

/*******************************************************************************
 * oplus speaker manager functions to codec
 ******************************************************************************/
struct oplus_speaker_device* get_speaker_dev(enum oplus_pa_type pa_type);
void *oplus_speaker_pa_register(struct oplus_speaker_device *device);
int oplus_speaker_pa_remove(void *node);
#endif /* __OPLUS_AUDIO_SPEAKER_CODEC_H__ */

