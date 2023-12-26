/************************************************************************************
** File: - oplus_audio_speaker_codec.h
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

#ifndef __OPLUS_SPEAKER_MANAGER_CODEC_H__
#define __OPLUS_SPEAKER_MANAGER_CODEC_H__

#include <sound/soc.h>
/*------------------------------------------------------------------------------*/
enum oplus_pa_channel {
	/* Single     -> Left */
	/* Dual       -> Left,Right */
	OPLUS_PA_CHANNEL_LEFT = 0x0,
	OPLUS_PA_CHANNEL_RIGHT = 0x1,
	OPLUS_PA_CHANNEL_COUNT = 0x2,
};

enum oplus_pa_manufacturer {
	MFR_NONE = 0x00,
	MFR_AWINIC = 0x01,
	MFR_SI = 0x02,
	MFR_COUNT
};

enum oplus_pa_work_status {
	WORK_STATUS_OFF = 0x00,
	WORK_STATUS_ON,
	WORK_STATUS_COUNT,
};

enum oplus_pa_work_mode {
	WORK_MODE_OFF = 0x00,
	WORK_MODE_MUSIC,
	WORK_MODE_VOICE,
	WORK_MODE_FM,
	WORK_MODE_RECEIVER,
	WORK_MODE_COUNT,
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
	int spkl_enable;
	int spkr_enable;
	int rcv_enable;
	int vdd_need;
	int amp_boost_volume;
	enum oplus_pa_work_mode amp_mode_setting;
	int amp_force_mute_status;
};

struct oplus_speaker_device {
	enum oplus_pa_manufacturer speaker_manufacture;
	uint32_t chipset;
	int type;
	int status;
	enum oplus_pa_work_status speaker_status;
	enum oplus_pa_work_mode speaker_mode;
	int speaker_voltage; /* Multiplied by 10 */
	int (*speaker_enable_set)(struct oplus_speaker_device *speaker_device, int enable);
	int (*speaker_enable_get)(struct oplus_speaker_device *speaker_device);
	int (*speaker_protection_set)(struct oplus_speaker_device *speaker_device, int enable);
	int (*speaker_protection_get)(struct oplus_speaker_device *speaker_device);
	/* Mode Switch,such as AW87359 */
	int (*speaker_mode_set)(struct oplus_speaker_device *speaker_device, int mode);
	int (*speaker_mode_get)(struct oplus_speaker_device *speaker_device);
	int (*boost_voltage_set)(struct oplus_speaker_device *speaker_device, int level);
	int (*boost_voltage_get)(struct oplus_speaker_device *speaker_device);
	void (*speaker_mute_set)(struct oplus_speaker_device *speaker_device, int enable);
	void (*speaker_mute_get)(struct oplus_speaker_device *speaker_device, int *enable);
};

struct oplus_spk_dev_node {
	struct oplus_speaker_device *device;
	struct list_head list;
};
/*------------------------------------------------------------------------------*/
struct oplus_speaker_device* get_speaker_dev(enum oplus_pa_type pa_type);
enum oplus_pa_manufacturer get_speaker_manufacturer(enum oplus_pa_type pa_type);
void *oplus_speaker_pa_register(struct oplus_speaker_device *speaker_device);
int oplus_speaker_pa_unregister(void *node);
/*------------------------------------------------------------------------------*/
#endif /* __OPLUS_SPEAKER_MANAGER_CODEC_H__ */
