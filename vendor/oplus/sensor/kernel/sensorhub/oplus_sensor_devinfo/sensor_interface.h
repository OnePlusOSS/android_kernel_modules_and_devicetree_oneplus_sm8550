/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include "scp.h"

#define DEV_TAG					 "[sensor_devinfo] "
#define DEVINFO_LOG(fmt, args...)   pr_err(DEV_TAG"%s %d : "fmt, __func__, __LINE__, ##args)

#define BRL_MAX_LEN 7

struct als_info{
	uint16_t senstype;
	uint16_t brightness;
	uint16_t dc_mode;
	bool use_lb_algo;
	uint16_t pwm_turbo;
	uint16_t rt_bri;
	uint16_t fps;
} __packed __aligned(4);

struct cali_data {
	int acc_data[6];
	int gyro_data[6];
	union {
		struct {
			int ps0_offset;
			int ps0_value;
			int ps0_distance_delta;
			int ps1_offset;
			int ps1_value;
			int ps1_distance_delta;
		};
		struct {
			int ps_low_offset;
			int ps_normal_offset;
			int ps_low;
			int ps_normal;
			int nReserve[2];
		};
		int ps_cali_data[6];
	};
	int als_value;
	int als_factor;
	int cct_cali_data[6];
	int rear_als_value;
	int rear_als_factor;
};

struct sensorhub_interface {
	int (*send_factory_mode)(int sensor_type, int mode, void *result);
	int (*send_selft_test)(int sensor_type, void *result);
	int (*send_reg_config)(int sensor_type);
	int (*send_cfg)(struct cali_data* cali_data);
	int (*send_utc_time)(void);
	int (*send_lcdinfo)(struct als_info *lcd_info);
	int (*get_lcdinfo_brocast_type) (void);
	void (*init_sensorlist)(void);
	bool (*is_sensor_available)(char *name);
	bool (*is_sensor_type_available)(uint8_t sensor_type);
	bool (*get_sensor_name)(uint8_t sensor_type, char **name);
};

enum {
	NONE_TYPE = 0,
	LCM_DC_MODE_TYPE,
	LCM_BRIGHTNESS_TYPE,
	LCM_PWM_TURBO = 0x14,
	LCM_ADFR_MIN_FPS = 0x15,
	MAX_INFO_TYPE,
};

enum oplus_adfr_auto_min_fps_value {
	OPLUS_ADFR_AUTO_MIN_FPS_MAX = 0x00,
	OPLUS_ADFR_AUTO_MIN_FPS_60HZ = 0x01,
	OPLUS_ADFR_AUTO_MIN_FPS_30HZ = 0x03,
	OPLUS_ADFR_AUTO_MIN_FPS_20HZ = 0x05,
	OPLUS_ADFR_AUTO_MIN_FPS_10HZ = 0x0B,
	OPLUS_ADFR_AUTO_MIN_FPS_1HZ = 0x77,
};

enum {
	LCM_DC_OFF = 0,
	LCM_DC_ON  = 1
};

struct fifo_frame{
	uint8_t type;
	uint16_t data;
};

struct br_level_info {
	uint32_t pri_brl_num;
	uint32_t pri_brl_thrd[BRL_MAX_LEN];
};

struct ssc_interactive{
	DECLARE_KFIFO_PTR(fifo, struct fifo_frame);
	wait_queue_head_t wq;
	spinlock_t fifo_lock;
	struct als_info a_info;
	struct miscdevice mdev;
	struct notifier_block lcd_nb;
	struct notifier_block ready_nb;
	struct delayed_work lcdinfo_work;
	struct delayed_work ready_work;
	struct sensorhub_interface *si;
	struct br_level_info brl_info;
	bool support_pwm_turbo;
	bool support_bri_to_scp;
	bool support_bri_to_hal;
	bool need_to_sync_lcd_rate;
};

void init_sensorhub_interface(struct sensorhub_interface **si);
void ssc_interactive_exit(void);
int ssc_interactive_init(void);
#endif /*SENSOR_INTERFACE_H*/
