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


#ifndef _SIA81XX_COMMOMN_H
#define _SIA81XX_COMMOMN_H

#define SIA81XX_DRIVER_VERSION			("1.2.1c")
#define SIA81XX_MAX_CHANNEL_SUPPORT		(8)

enum {
	SIA81XX_CHANNEL_L_ONE = 0,
	SIA81XX_CHANNEL_L_TWO,
	SIA81XX_CHANNEL_R_ONE,
	SIA81XX_CHANNEL_R_TWO,
	SIA81XX_CHANNEL_NUM
};


enum {
	AUDIO_SCENE_PLAYBACK = 0,
	AUDIO_SCENE_VOICE,
	AUDIO_SCENE_RECEIVER,
	AUDIO_SCENE_FACTORY,
	AUDIO_SCENE_NUM
};

enum {
	CHIP_TYPE_SIA8101 = 0,
	CHIP_TYPE_SIA8109,
	CHIP_TYPE_SIA8152,
	CHIP_TYPE_SIA8152S,
	CHIP_TYPE_SIA8159,
	CHIP_TYPE_SIA8159A,
	// add normal chip type here
	CHIP_TYPE_SIA8100X,
	CHIP_TYPE_SIA81X9,
	CHIP_TYPE_SIA8152X,
	CHIP_TYPE_UNKNOWN,
	CHIP_TYPE_INVALID
};

struct sia81xx_err {
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

typedef struct sia81xx_dev_s {
	char name[32];
	unsigned int chip_type;
	struct platform_device *pdev;
	struct i2c_client *client;

	int disable_pin;
	int rst_pin;
	int owi_pin;
	int id_pin;

	unsigned int owi_delay_us;
	unsigned int owi_cur_mode;
	unsigned int owi_polarity;

	spinlock_t rst_lock;
	spinlock_t owi_lock;

	struct regmap *regmap;
	unsigned int scene;

	uint32_t channel_num;
	unsigned int en_xfilter;
	unsigned int en_dyn_ud_vdd;
	unsigned int en_dyn_ud_pvdd;
	unsigned int dyn_ud_vdd_port;
	unsigned int en_dyn_id;
	uint32_t timer_task_hdl;
	uint32_t timer_task_hdl_backup;

	struct list_head list;

	struct sia81xx_err err_info;
} sia81xx_dev_t;

sia81xx_dev_t *find_sia81xx_dev(const char *name,struct device_node *of_node);

#endif /* _SIA81XX_COMMOMN_H */

