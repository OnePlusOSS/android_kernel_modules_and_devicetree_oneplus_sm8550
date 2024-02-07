/*
 * Copyright (C) 2018, SI-IN, Yun Shi (yun.shi@si-in.com).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef _SIPA_COMMOMN_H
#define _SIPA_COMMOMN_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>


#define SIPA_DRIVER_VERSION					("3.0.8")
#define SIPA_MAX_CHANNEL_SUPPORT			(8)

struct sipa_err {
	unsigned long owi_set_mode_cnt;
	unsigned long owi_set_mode_err_cnt;
	unsigned long owi_write_err_cnt;
	unsigned long owi_polarity_err_cnt;
	unsigned long owi_max_retry_cnt;
	unsigned long owi_max_gap;
	unsigned long owi_max_deviation;
	unsigned long owi_write_data_err_cnt;
	unsigned long owi_write_data_cnt;
};

enum {
	AUDIO_SCENE_PLAYBACK = 0,
	AUDIO_SCENE_VOICE,
	AUDIO_SCENE_RECEIVER,
	AUDIO_SCENE_FACTORY,
	AUDIO_SCENE_NUM
};

typedef struct sipa_dev_s {
	char name[32];
	unsigned int chip_type;
	struct platform_device *pdev;
	struct i2c_client *client;

	struct workqueue_struct *sia91xx_wq;
	struct delayed_work interrupt_work;
	struct delayed_work monitor_work;
	struct delayed_work fw_load_work;
	int disable_pin;
	int rst_pin;
	int owi_pin;
	int irq_pin;
	int id_pin;

	unsigned int owi_delay_us;
	unsigned int owi_cur_mode[AUDIO_SCENE_NUM];
	unsigned int owi_polarity;

	spinlock_t rst_lock;
	spinlock_t owi_lock;

	struct regmap *regmap;
	unsigned int scene;

	uint32_t channel_num;
	uint32_t dyn_ud_vdd_port;
	uint32_t en_dyn_id;
	uint32_t en_dyn_ud_vdd;
	uint32_t en_dyn_ud_pvdd;
	uint32_t en_spk_cal_dl;
	uint32_t spk_model_flag;
	uint8_t  pa_status;

//#ifdef SIA91XX_TYPE
	struct snd_soc_codec *codec;
	int pstream;
	int cstream;
	int mute_mode;
//#endif

	struct list_head list;
	struct sipa_err err_info;
	bool sipa_on;
#if IS_ENABLED(CONFIG_SND_SOC_OPLUS_PA_MANAGER)
	unsigned int pre_scene;
#endif /*CONFIG_SND_SOC_OPLUS_PA_MANAGER*/
} sipa_dev_t;

struct sipa_chip_compat {
	const uint32_t sub_type;
	struct {
		const uint32_t *chips;
		const uint32_t num;
	};
};

enum {
	SIPA_CHANNEL_0 = 0,
	SIPA_CHANNEL_1,
	SIPA_CHANNEL_2,
	SIPA_CHANNEL_3,
#if 0
	SIPA_CHANNEL_4,
	SIPA_CHANNEL_5,
	SIPA_CHANNEL_6,
	SIPA_CHANNEL_7,
#endif
	SIPA_CHANNEL_NUM
};

enum {
	CHIP_TYPE_SIA8101 = 0,
	CHIP_TYPE_SIA8109,
	CHIP_TYPE_SIA8152,
	CHIP_TYPE_SIA8152S,
	CHIP_TYPE_SIA8159,
	CHIP_TYPE_SIA8159A,
	// add analog chip type here
	CHIP_TYPE_SIA9195,
	CHIP_TYPE_SIA9175,
	// add digital chip type here
	CHIP_TYPE_SIA8100X,
	CHIP_TYPE_SIA81X9,
	CHIP_TYPE_SIA8152X,
	CHIP_TYPE_SIA9196,
	// add compatible chip type here
	CHIP_TYPE_UNKNOWN,
	CHIP_TYPE_INVALID
};

#define SIPA_MAX_REG_ADDR					(0xFF)

#define SIA81XX_REG_R_O						(0x00000001)
#define SIA81XX_REG_W_O						(0x00000001 << 1)
#define SIA81XX_REG_RW						(SIA81XX_REG_R_O | SIA81XX_REG_W_O)

#define SIPA_CAL_SPK_DEFAULT_VAL			(0xFFFFFFFF)
#define SIPA_CAL_SPK_OK						(1)
#define SIPA_CAL_SPK_FAIL					(0)


/* error list */
/* pulse width time out */
#define EPTOUT								(100)
/* pulse electrical level opposite with the polarity */
#define EPOLAR								(101)
#define EEXEC								(102)
#define EOUTR								(103)

#define sipa_timer_usr_id_set_vdd(ch)			((ch))
#define sipa_timer_usr_id_spk_cal(ch)			((ch) + SIPA_CHANNEL_NUM)
#define sipa_timer_usr_id_cal_r0(ch)            ((ch) + 2*SIPA_CHANNEL_NUM)
#define sipa_timer_usr_id_set_spk_model(ch)     ((ch) + 3*SIPA_CHANNEL_NUM)
#define sipa_timer_usr_id_get_spk_model(ch)     ((ch) + 4*SIPA_CHANNEL_NUM)
#define sipa_timer_usr_id_close_f0_temp(ch)     ((ch) + 5*SIPA_CHANNEL_NUM)

sipa_dev_t *find_sipa_dev(const char *name,
	struct device_node *of_node);

#endif /* _SIPA_COMMOMN_H */
