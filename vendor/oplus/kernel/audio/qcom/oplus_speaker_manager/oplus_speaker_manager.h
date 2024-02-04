/************************************************************************************
** File: - oplus_audio_speaker_manager.h
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

#ifndef __OPLUS_SPEAKER_MANAGER_H__
#define __OPLUS_SPEAKER_MANAGER_H__

#include <sound/soc.h>

#include "oplus_speaker_manager_codec.h"
/*------------------------------------------------------------------------------*/
int ext_amp_chipset_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
enum oplus_pa_manufacturer oplus_speaker_mfr_get(enum oplus_pa_type pa_type);
int speaker_l_mfr_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int speaker_r_mfr_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int oplus_speaker_amp_get(enum oplus_pa_type pa_type);
int oplus_speaker_amp_set(enum oplus_pa_type pa_type, int value);
int speaker_l_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int speaker_l_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int speaker_r_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int speaker_r_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int rcv_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int rcv_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_mode_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_mode_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_vdd_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_vdd_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_boost_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_boost_volume_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_force_mute_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_force_mute_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int oplus_spkr_pa_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event);
int oplus_add_pa_manager_snd_controls(struct snd_soc_component *component);
int oplus_add_analog_pa_manager_dapm(struct snd_soc_dapm_context *dapm);
/*------------------------------------------------------------------------------*/
#endif /* __OPLUS_SPEAKER_MANAGER_H__ */
