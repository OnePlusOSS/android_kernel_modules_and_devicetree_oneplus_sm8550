// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#ifndef __SENSOR_FEEDBACK_H__
#define __SENSOR_FEEDBACK_H__

#include <linux/miscdevice.h>
#ifdef CONFIG_ARM
#include <linux/sched.h>
#else
#include <linux/wait.h>
#endif
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/param.h>
#include <linux/proc_fs.h>
#include <linux/time.h>

#define THREAD_WAKEUP  0
#define THREAD_SLEEP   1

#undef	SUBSYS_COUNTS
#define	SUBSYS_COUNTS	(3)

struct sensor_fb_conf {
	uint16_t event_id;
	char *fb_field;
	char *fb_event_id;
};


enum sensor_fb_event_id {
	FD_HEAD_EVENT_ID = 0,
	/*1~100*/
	PS_INIT_FAIL_ID = 1,
	PS_I2C_ERR_ID = 2,
	PS_ALLOC_FAIL_ID = 3,
	PS_ESD_REST_ID = 4,
	PS_NO_INTERRUPT_ID = 5,
	PS_FIRST_REPORT_DELAY_COUNT_ID = 6,
	PS_ORIGIN_DATA_TO_ZERO_ID = 7,
	PS_CALI_DATA_ID = 8,
	PS_DYNAMIC_CALI_ID = 9,
	PS_ZERO_CALI_ID = 10,

	/*100~200*/
	ALS_INIT_FAIL_ID = 100,
	ALS_I2C_ERR_ID = 101,
	ALS_ALLOC_FAIL_ID = 102,
	ALS_ESD_REST_ID = 103,
	ALS_NO_INTERRUPT_ID = 104,
	ALS_FIRST_REPORT_DELAY_COUNT_ID = 105,
	ALS_ORIGIN_DATA_TO_ZERO_ID = 106,
	ALS_CALI_DATA_ID = 107,
	ALS_CG_RPT_INFO_ID = 108,

	/*200~250*/
	ACCEL_INIT_FAIL_ID = 200,
	ACCEL_I2C_ERR_ID = 201,
	ACCEL_ALLOC_FAIL_ID = 202,
	ACCEL_ESD_REST_ID = 203,
	ACCEL_NO_INTERRUPT_ID = 204,
	ACCEL_FIRST_REPORT_DELAY_COUNT_ID = 205,
	ACCEL_ORIGIN_DATA_TO_ZERO_ID = 206,
	ACCEL_CALI_DATA_ID = 207,
	ACCEL_DATA_BLOCK_ID = 208,
	ACCEL_SUB_DATA_BLOCK_ID = 209,
	ACCEL_DATA_FULL_RANGE_ID = 210,

	/*250~300*/
	ACCEL_SUB_INIT_FAIL_ID = 250,
	ACCEL_SUB_I2C_ERR_ID = 251,
	ACCEL_SUB_ALLOC_FAIL_ID = 252,
	ACCEL_SUB_ESD_REST_ID = 253,
	ACCEL_SUB_NO_INTERRUPT_ID = 254,
	ACCEL_SUB_FIRST_REPORT_DELAY_COUNT_ID = 255,
	ACCEL_SUB_ORIGIN_DATA_TO_ZERO_ID = 256,
	ACCEL_SUB_CALI_DATA_ID = 257,
	ACCEL_SUB_DATA_FULL_RANGE_ID = 258,

	/*300~350*/
	GYRO_INIT_FAIL_ID = 300,
	GYRO_I2C_ERR_ID = 301,
	GYRO_ALLOC_FAIL_ID = 302,
	GYRO_ESD_REST_ID = 303,
	GYRO_NO_INTERRUPT_ID = 304,
	GYRO_FIRST_REPORT_DELAY_COUNT_ID = 305,
	GYRO_ORIGIN_DATA_TO_ZERO_ID = 306,
	GYRO_CALI_DATA_ID = 307,
	GYRO_DATA_BLOCK_ID = 308,

	/*350~400*/
	GYRO_SUB_INIT_FAIL_ID = 350,
	GYRO_SUB_I2C_ERR_ID = 351,
	GYRO_SUB_ALLOC_FAIL_ID = 352,
	GYRO_SUB_ESD_REST_ID = 353,
	GYRO_SUB_NO_INTERRUPT_ID = 354,
	GYRO_SUB_FIRST_REPORT_DELAY_COUNT_ID = 355,
	GYRO_SUB_ORIGIN_DATA_TO_ZERO_ID = 356,
	GYRO_SUB_CALI_DATA_ID = 357,
	GYRO_SUB_DATA_BLOCK_ID = 358,

	/*400~500*/
	MAG_INIT_FAIL_ID = 400,
	MAG_I2C_ERR_ID = 401,
	MAG_ALLOC_FAIL_ID = 402,
	MAG_ESD_REST_ID = 403,
	MAG_NO_INTERRUPT_ID = 404,
	MAG_FIRST_REPORT_DELAY_COUNT_ID = 405,
	MAG_ORIGIN_DATA_TO_ZERO_ID = 406,
	MAG_CALI_DATA_ID = 407,
	MAG_DATA_BLOCK_ID = 408,
	MAG_DATA_FULL_RANGE_ID = 409,

	/*500~600*/
	SAR_INIT_FAIL_ID = 500,
	SAR_I2C_ERR_ID = 501,
	SAR_ALLOC_FAIL_ID = 502,
	SAR_ESD_REST_ID = 503,
	SAR_NO_INTERRUPT_ID = 504,
	SAR_FIRST_REPORT_DELAY_COUNT_ID = 505,
	SAR_ORIGIN_DATA_TO_ZERO_ID = 506,
	SAR_CALI_DATA_ID = 507,

	/*600~700*/
	BAROMETER_I2C_ERR_ID = 600,

	/*700~750*/
	HALL_I2C_ERR_ID = 700,

	/*750~789*/
	FOLD_DEVICE_FOLDE_COUNT_ID = 750,

	/*790~799*/
	FREE_FALL_TRIGGER_ID = 790,

	/*800~900*/
	POWER_SENSOR_INFO_ID = 800,
	POWER_ACCEL_INFO_ID = 801,
	POWER_GYRO_INFO_ID = 802,
	POWER_MAG_INFO_ID = 803,
	POWER_PROXIMITY_INFO_ID = 804,
	POWER_LIGHT_INFO_ID = 805,
	POWER_WISE_LIGHT_INFO_ID = 806,
	POWER_WAKE_UP_RATE_ID = 807,
	POWER_ADSP_SLEEP_RATIO_ID = 808,

	/*900~1000*/
	DOUBLE_TAP_REPORTED_ID = 901,
	DOUBLE_TAP_PREVENTED_BY_NEAR_ID = 902,
	DOUBLE_TAP_PREVENTED_BY_ATTITUDE_ID = 903,
	DOUBLE_TAP_PREVENTED_BY_FREEFALL_Z_ID = 904,
	DOUBLE_TAP_PREVENTED_BY_FREEFALL_SLOPE_ID = 905,

	/*1000*/
	ALAILABLE_SENSOR_LIST_ID = 1000,

	/*1100~1200*/
	CCT_I2C_ERR_ID = 1100,

	/* 10000 , sensor-hal*/
	HAL_SENSOR_NOT_FOUND = 10000,
	HAL_QMI_ERROR = 10001,
	HAL_SENSOR_TIMESTAMP_ERROR = 10002,
};

struct fd_data {
	int data_x;
	int data_y;
	int data_z;
};

#define EVNET_DATA_LEN 3
struct sns_fb_event {
	unsigned short event_id;
	unsigned int count;
	unsigned int name;
	union {
		int buff[EVNET_DATA_LEN];
		struct fd_data data;
	};
};

#define EVNET_NUM_MAX 109
struct fb_event_smem {
	struct sns_fb_event event[EVNET_NUM_MAX];
};

struct sensor_fb_cxt {
	/*struct miscdevice sensor_fb_dev;*/
	struct platform_device *sensor_fb_dev;
	spinlock_t   rw_lock;
	wait_queue_head_t wq;
	struct task_struct *report_task; /*kernel thread*/
        struct task_struct *poll_task;
	uint16_t adsp_event_counts;
	struct fb_event_smem fb_smem;
	uint16_t node_type;
	unsigned long wakeup_flag;
	uint32_t sensor_list[2];
	struct proc_dir_entry  *proc_sns;
        struct delayed_work enable_sensor_work;
        struct hf_client *client;
};
#endif /*__SENSOR_FEEDBACK_H__*/
