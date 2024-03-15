/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#ifndef __OPLUS_SSC_INTERACTIVE_H__
#define __OPLUS_SSC_INTERACTIVE_H__

#include <linux/miscdevice.h>
#include <linux/kfifo.h>
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
#include <linux/notifier.h>

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_FB_QC)
enum {
	SCREEN_INIT = 0,
	SCREEN_ON = 1,
	SCREEN_OFF = 2,
};
extern void ssc_fb_set_screen_status(int status);
#endif


enum {
	NONE_TYPE = 0x0,
	LCM_DC_MODE_TYPE,
	LCM_BRIGHTNESS_TYPE,
	LCM_BRIGHTNESS_TYPE_SEC,
	LCM_POWER_MODE,
	LCM_POWER_MODE_SEC,
	LCM_PWM_TURBO_TYPE,
	LCM_ADFR_MIN_FPS,
	LCM_HBM_LONG_INTE_TYPE,
	LCM_HBM_SHORT_INTE_TYPE,
	LCM_BLANK_MODE_TYPE,
	MAX_INFO_TYPE,
};

enum {
	HBM_MODE_OFF = 0,
	HBM_MODE_LONG_INTE = 1,
	HBM_MODE_SHORT_INTE = 2,
};

enum {
	LCM_DC_OFF = 0,
	LCM_DC_ON  = 1
};


struct als_info{
	uint16_t brightness;
	uint16_t dc_mode;
	uint16_t blank_mode;
};

struct fifo_frame{
	uint8_t type;
	uint16_t data;
};


#define BRL_MAX_LEN 7

struct br_level_info {
	uint32_t pri_brl_num;
	uint32_t pri_brl_thrd[BRL_MAX_LEN];
	uint32_t pri_brl_l2h_thrd;
	uint32_t secd_brl_num;
	uint32_t secd_brl_thrd[BRL_MAX_LEN];
	uint32_t secd_brl_l2h_thrd;
};

struct ssc_interactive{
	struct als_info   a_info;
	struct miscdevice mdev;
	DECLARE_KFIFO_PTR(fifo, struct fifo_frame);
	spinlock_t   fifo_lock;
	spinlock_t   rw_lock;
	wait_queue_head_t wq;
	struct notifier_block nb;
	struct br_level_info brl_info;

#if IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_NOTIFY)
	bool is_fold_dev;
	bool need_lb_algo;
	bool pwm_turbo_on;
	int hbm_on;
	uint16_t last_primary_bri;
#if IS_ENABLED(CONFIG_OPLUS_SENSOR_DRM_PANEL_ADFR_MIN_FPS)
	uint16_t last_freq;
#endif
	bool sup_power_fb;
	bool report_blank_mode;
	uint8_t sup_hbm_mode;
	bool notify_work_regiseted;
	bool notify_work_regiseted_second;
	uint8_t notify_work_retry;
	void *notifier_cookie;
	void *notifier_cookie_second;
	struct drm_panel *active_panel;
	struct drm_panel *active_panel_second;
	struct delayed_work regiseter_lcd_notify_work;
#endif
};

#endif
