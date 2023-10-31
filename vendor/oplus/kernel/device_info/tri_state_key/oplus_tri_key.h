/* SPDX-License-Identifier: GPL-2.0-only*/
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 * File: oplus_tri_key.c
 *
 * Description:
 *      Definitions for m1120 tri_state_key data process.
 *
 * Version: 1.0
 */
#ifndef __TRIKEY_H__
#define __TRIKEY_H__

#include <linux/alarmtimer.h>
#include <linux/version.h>
#include "ist_hall_ic/hall_ist8801.h"
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
#include <soc/oplus/system/kernel_fb.h>
#endif
#define THREEAXIS_DATA_NUM 9
#define HALL_CALIB_NUM 128
#define HALL_DATA_NUM 50
#define THREEAXIS_INTERF_LIMIT 700
#define THREEAXIS_INTERFJUDGE_SUMLIMIT 4000
#define THREEAXIS_INTERFJUDGE_ZLIMIT 2000
#define THREEAXIS_INTERF_MIDTOLEN 3700
#define THREEAXIS_INTERF_DUTOLEN 3000
#define THREEAXIS_POSITION_XTOLEN 2000
#define THREEAXIS_POSITION_YTOLEN 5000

#define TRIKEY_FB_INTERF_TYPE       "10001"
#define TRIKEY_FB_BUS_TRANS_TYPE    "10002"
#define TRIKEY_FB_CALIB_DATA_TYPE   "10003"
enum debug_level {
	LEVEL_BASIC,
	LEVEL_DEBUG,
};

enum dhall_id {
	DHALL_0 = 0,
	DHALL_1,
	DHALL_2,
};

enum enable_status {
	IRQ_DISABLE,
	IRQ__ENABLE,
};

enum dhall_detection_mode {
	DETECTION_MODE_POLLING = 0,
	DETECTION_MODE_INTERRUPT,
	DETECTION_MODE_INVALID,
};

enum motor_direction {
	MOTOR_DOWN = 0,
	MOTOR_UPWARD,
};

enum tri_key_position {
	UP_STATE,
	DOWN_STATE,
	MID_STATE,
};

#define MAX_LEN     16
#define READ_OFFECT 255
#define OFFECT_DOWN "down"
#define OFFECT_UP   "up"
#define OFFECT_CUT  ":"
#define OFFECT_MAX  100

#define FLOD_INTERF_OFFECT 5
#define FLOD_INTERF_MAX    3

enum interf_type {
	INTERF_TYPE_NONE,
	INTERF_TYPE_1,
	INTERF_TYPE_2,
	INTERF_TYPE_3,
	INTERF_TYPE_4,
	INTERF_TYPE_5,
	INTERF_TYPE_6,
	INTERF_TYPE_7,
	INTERF_TYPE_8,
};
extern unsigned int tristate_extcon_tab[];
extern unsigned int tri_key_debug;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
#define PDE_DATA pde_data
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
#define CREATE_PROC_OPS(name, open_func, read_func, write_func, release_func) \
                                        static const struct proc_ops name = { \
                                                .proc_open      = open_func,      \
                                                .proc_write = write_func,         \
                                                .proc_read      = read_func,      \
                                                .proc_release = release_func, \
                                                .proc_lseek     = default_llseek, \
                                        }
#else
#define CREATE_PROC_OPS(name, open_func, read_func, write_func, release_func) \
                                        static const struct file_operations name = { \
                                                .open  = open_func,      \
                                                .write = write_func,     \
                                                .read  = read_func,      \
                                                .release = release_func, \
                                                .owner = THIS_MODULE,    \
                                        }
#endif


struct dhall_data_t {
	short data0;
	short data1;
};

struct dhall_data_xyz {
	short hall_x;
	short hall_y;
	short hall_z;
	int hall_v;
	u8 st;
};

struct hall_srs {
	char name[12];
	uint8_t value;
	bool bias;
	uint32_t ratio;
};

struct dhall_operations {
	int (*get_data)(short *data);
	int (*set_detection_mode)(u8 mode);
	int (*enable_irq)(bool enable);
	int (*clear_irq)(void);
	int (*get_irq_state)(void);
	bool (*update_threshold)(int position, short lowthd, short highthd);
	void (*dump_regs)(u8 *buf);
	int (*set_reg)(int reg, int val);
	bool (*is_power_on)(void);
	void (*set_sensitivity)(char *data);
	int (*offect_data_handle)(int offect);
	int (*get_threeaxis_data)(struct dhall_data_xyz *data);
	bool (*update_threeaxis_threshold)(int position, short lowthd, short highthd, struct dhall_data_xyz *data, int interf);
};

struct extcon_dev_data {
	struct work_struct dwork;
	struct work_struct register_work;
	struct extcon_dev *edev;
	struct device *dev;
	struct i2c_client *client;
	struct input_dev     *input_dev;
	struct timer_list s_timer;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	struct delayed_work	up_work;
	struct delayed_work	down_work;
	struct dhall_operations *dhall_up_ops;
	struct dhall_operations *dhall_down_ops;
	struct dhall_operations *threeaxis_dhall_ops;
	struct mutex mtx;
	struct dhall_data_xyz	hall_value;
	struct dhall_data_xyz	pre_hall_value;
	const char *d_name;
	const char *m_name;
	int		position;
	int		last_position;
	int		project_info;
	int		interf;
	short		state;
	short		dhall_data0;
	short		dhall_data1;
	short		dnhall_upv;
	short		dnhall_mdv;
	short		dnhall_dnv;
	short		uphall_upv;
	short		uphall_mdv;
	short		uphall_dnv;
	int			manual2auto_up_switch;
	int			manual2auto_down_switch;
	int			irq;
	int         data_offect;
	char        data_offect_name[8];
	int         threeaxis_calib_data[9];
	bool        threeaxis_hall_support;
	bool        enable_esd_check;
	int         trigger_id;
};

extern int oplus_register_hall(const char *name,
		struct dhall_operations *ops, struct extcon_dev_data *hall_dev);
extern int oplus_hall_get_data(unsigned int id);
extern int oplus_hall_set_detection_mode(unsigned int id, u8 mode);
extern int oplus_hall_enable_irq(unsigned int id, bool enable);
extern int oplus_hall_clear_irq(unsigned int id);
extern int oplus_hall_irq_handler(unsigned int id);
extern int threeaxis_hall_irq_handler(unsigned int id);
extern int oplus_hall_get_irq_state(unsigned int id);
extern void oplus_hall_dump_regs(unsigned int id, u8 *buf);
extern int oplus_hall_set_reg(unsigned int id, int reg, int val);
extern int oplus_hall_update_threshold(unsigned int id,
				int position, short lowthd, short highthd);
extern bool oplus_hall_is_power_on(void);
extern int aw8697_op_haptic_stop(void);

#endif /* __TRIKEY_H__ */
