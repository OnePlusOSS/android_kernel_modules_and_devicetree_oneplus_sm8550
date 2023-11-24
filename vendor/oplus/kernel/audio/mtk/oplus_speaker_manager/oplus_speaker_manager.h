/************************************************************************************
** File: - oplus_audio_speaker_manager_platform.h
** Copyright (C), 2020-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      Add for oplus audio speaker manager.This header file id provided
**      to oplus_speaker_manager.c .
**
** Version: 1.0
** Date created: 26/05/2021
**
** --------------------------- Revision History: --------------------------------
**    <author>       <date>          <desc>
**  Kun.Zhao   2021.05.26     Add this file
** OPLUS Coding Static Checking Skip
************************************************************************************/

#ifndef __OPLUS_AUDIO_SPEAKER_MANAGER_H__
#define __OPLUS_AUDIO_SPEAKER_MANAGER_H__

#include <sound/soc.h>

/*******************************************************************************
 * oplus speaker manager functions to oplus_speaker_manager.c
 ******************************************************************************/
int ext_amp_chipset_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_chipset_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_vdd_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_vdd_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_boost_voltage_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_boost_voltage_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_mode_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_mode_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_force_mute_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int ext_amp_force_mute_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int speaker_l_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int speaker_l_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int speaker_r_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int speaker_r_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int rcv_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int rcv_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int rcv_l_amp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int rcv_l_amp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);

#endif /* __OPLUS_AUDIO_SPEAKER_MANAGER_H__ */

