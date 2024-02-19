/************************************************************************************
** File: - oplus_audio_speaker_manager_platform.h
** Copyright (C), 2020-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      Add for oplus audio speaker manager.This header file id provided
**      to platform.
**
** Version: 1.0
** Date created: 26/05/2021
**
** --------------------------- Revision History: --------------------------------
**    <author>       <date>          <desc>
**  Kun.Zhao   2021.05.26     Add this file
** OPLUS Coding Static Checking Skip
************************************************************************************/

#ifndef __OPLUS_AUDIO_SPEAKER_MANAGER_PLATFORM_H__
#define __OPLUS_AUDIO_SPEAKER_MANAGER_PLATFORM_H__

#include <sound/soc.h>

/*******************************************************************************
 * oplus speaker manager functions to platform
 ******************************************************************************/
int oplus_ext_amp_l_enable(int enable);
int oplus_ext_amp_r_enable(int enable);
int oplus_ext_amp_recv_enable(int enable);
int oplus_ext_amp_recv_l_enable(int enable);
int oplus_add_pa_manager_snd_controls(struct snd_soc_component *cmpnt);

#endif /* __OPLUS_AUDIO_SPEAKER_MANAGER_PLATFORM_H__ */

