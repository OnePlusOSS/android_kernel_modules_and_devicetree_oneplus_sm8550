// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 * File: oplus_tri_key.c
 *
 * Description:
 *      Definitions for m1120 tri_state_key data process.
 *
 * Version: 1.0
 */

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/alarmtimer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/extcon.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/time.h>

#include <linux/string.h>
#include <linux/version.h>

#include "oplus_tri_key.h"

#define TRI_KEY_DEVICE "oplus,hall_tri_state_key"
#define TRI_KEY_TAG                  "[tri_state_key] "
#define TRI_KEY_ERR(fmt, args...)\
	pr_err(TRI_KEY_TAG" %s : "fmt, __func__, ##args)
#define TRI_KEY_LOG(fmt, args...)\
	pr_err(TRI_KEY_TAG" %s : "fmt, __func__, ##args)
#define TRI_KEY_DEBUG(fmt, args...)\
	do {\
		if (tri_key_debug == LEVEL_DEBUG)\
			pr_info(TRI_KEY_TAG " %s: " fmt, __func__, ##args);\
	} while (0)
enum {
	MODE_UNKNOWN,
	MODE_MUTE,
	MODE_DO_NOT_DISTURB,
	MODE_NORMAL,
	MODE_MAX_NUM
	} tri_mode;


unsigned int tristate_extcon_tab[] = {
		MODE_MUTE,
		MODE_DO_NOT_DISTURB,
		MODE_NORMAL,
		EXTCON_NONE,
	};

static struct hrtimer tri_key_timer;
struct work_struct tri_key_timeout_work;

static DEFINE_MUTEX(tri_key_mutex);


static struct extcon_dev_data *g_the_chip;
static struct extcon_dev_data *g_hall_dev;
static int last_d0;
static int last_d1;
static int last_position = -1;
static int last_interf = -1;
static int interf_count;
static int time = 1;
unsigned int tri_key_debug;

static short tol0 = 15;
static short tol2 = 40;
static short up_mid_tol = 15;
static short up_tolerance = 15;
static short down_tolerance = 15;
static short mid_down_tol = 15;
static short up_mid_distance;
static short mid_down_distance;
static short calib_upvaluesum, calib_mdvaluesum, calib_dnvaluesum;
static short calib_upvaluemin, calib_mdvaluemin, calib_dnvaluemin;
static short calib_dnhall_um_distance, calib_dnhall_md_distance;
static short calib_uphall_um_distance, calib_uphall_md_distance;
static short calib_uphall_ud_distance, calib_dnhall_ud_distance;

static bool is_has_data_offect = false;
static bool is_has_esd_check   = false;

void oplus_hall_disable_irq(bool enable)
{
	oplus_hall_clear_irq(DHALL_0);
	oplus_hall_clear_irq(DHALL_1);

	g_the_chip->dhall_down_ops->enable_irq(enable);
	g_the_chip->dhall_up_ops->enable_irq(enable);
}

int oplus_hall_enable_irq(unsigned int id, bool enable)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
			!g_the_chip->dhall_down_ops->enable_irq)
			TRI_KEY_ERR("enable hall0 irq error\n");
		else {
			oplus_hall_clear_irq(DHALL_0);
			oplus_hall_clear_irq(DHALL_1);
			return g_the_chip->dhall_down_ops->enable_irq(enable);
		}
		break;
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
			!g_the_chip->dhall_up_ops->enable_irq)
			TRI_KEY_ERR("enable hall1 irq error\n");
		else {
			oplus_hall_clear_irq(DHALL_0);
			oplus_hall_clear_irq(DHALL_1);
			return g_the_chip->dhall_up_ops->enable_irq(enable);
		}
		break;
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

	return -EINVAL;
}

int oplus_hall_clear_irq(unsigned int id)
{
	if (!g_the_chip)
		return -EINVAL;
	TRI_KEY_DEBUG("dhall_clear_irq\n");
	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
			!g_the_chip->dhall_down_ops->enable_irq)
			return -EINVAL;
		else
			return g_the_chip->dhall_down_ops->clear_irq();

	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
			!g_the_chip->dhall_up_ops->enable_irq)
			return -EINVAL;
		else
			return g_the_chip->dhall_up_ops->clear_irq();
	case DHALL_2:
		if (!g_the_chip->threeaxis_dhall_ops ||
			!g_the_chip->threeaxis_dhall_ops->enable_irq)
			return -EINVAL;
		else
			return g_the_chip->threeaxis_dhall_ops->clear_irq();
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

	return -EINVAL;
}

int threeaxis_hall_get_data(struct dhall_data_xyz *hall_val, bool raw)
{
	if (!g_the_chip || !g_the_chip->threeaxis_dhall_ops || !g_the_chip->threeaxis_dhall_ops->get_threeaxis_data) {
		TRI_KEY_LOG("%s:failed", __func__);
		return -EINVAL;
	}
	g_the_chip->threeaxis_dhall_ops->get_threeaxis_data(hall_val);
	TRI_KEY_LOG("%s:success", __func__);
	return 0;
}

int oplus_hall_get_data(unsigned int id)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->get_data)
			TRI_KEY_ERR("get hall0 data error\n");
		else {
			g_the_chip->dhall_down_ops->get_data(
				&g_the_chip->dhall_data0);
			return true;
		}
		break;
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->get_data)
			TRI_KEY_ERR("get hall1 data error\n");
		else {
			g_the_chip->dhall_up_ops->get_data(
				&g_the_chip->dhall_data1);
			return true;
		}
		break;
	case DHALL_2:
		threeaxis_hall_get_data(&g_the_chip->hall_value, false);
		TRI_KEY_LOG("%s:success", __func__);
		return true;
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
	}

	return -EINVAL;
}

int oplus_hall_update_threshold(unsigned int id, int position,
					short lowthd, short highthd)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->update_threshold)
			TRI_KEY_ERR("update hall0 threshold error\n");
		else {
			g_the_chip->dhall_down_ops->update_threshold(position,
					lowthd, highthd);
			return true;
		}
		break;
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
			!g_the_chip->dhall_up_ops->update_threshold)
			TRI_KEY_ERR("update hall1 threshold error\n");
		else {
			g_the_chip->dhall_up_ops->update_threshold(position,
					lowthd, highthd);
			return true;
		}
		break;
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
	}

	return -EINVAL;
}

static int threeaxis_hall_update_threshold(unsigned int id, int position,
					short lowthd, short highthd, struct dhall_data_xyz *data, int interf)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		fallthrough;
	case DHALL_1:
		fallthrough;
	case DHALL_2:
		if (!g_the_chip->threeaxis_dhall_ops||
			!g_the_chip->threeaxis_dhall_ops->update_threeaxis_threshold) {
			TRI_KEY_ERR("update threeaxis hall threshold error\n");
			return -EINVAL;
		} else {
			g_the_chip->threeaxis_dhall_ops->update_threeaxis_threshold(position,
					lowthd, highthd, data, interf);
			return true;
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}
}

int oplus_hall_set_detection_mode(unsigned int id, u8 mode)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->set_detection_mode)
			TRI_KEY_ERR("set hall 0 error\n");
		else {
			g_the_chip->dhall_down_ops->set_detection_mode(mode);
			return true;
		}
		break;
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->set_detection_mode)
			TRI_KEY_ERR("set error\n");
		else {
			g_the_chip->dhall_up_ops->set_detection_mode(mode);
			return true;
		}
		break;
	case DHALL_2:
		if (!g_the_chip->threeaxis_dhall_ops->set_detection_mode)
			TRI_KEY_ERR("set error\n");
		else {
			g_the_chip->threeaxis_dhall_ops->set_detection_mode(mode);
			return true;
		}
		break;
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
	}

	return -EINVAL;
}

int oplus_hall_get_irq_state(unsigned int id)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->get_irq_state)
			return -EINVAL;
		else
			return g_the_chip->dhall_down_ops->get_irq_state();
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->get_irq_state)
			return -EINVAL;
		else
			return g_the_chip->dhall_up_ops->get_irq_state();
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}
}



void oplus_hall_dump_regs(unsigned int id, struct seq_file *s)
{
	if (!g_the_chip)
		return;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->dump_regs)
			TRI_KEY_ERR("dump DHALL_0 error\n");
		else
			g_the_chip->dhall_down_ops->dump_regs(s);
		break;
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->dump_regs)
			TRI_KEY_ERR("dump DHALL_1 error\n");
		else
			g_the_chip->dhall_up_ops->dump_regs(s);
		break;
	case DHALL_2:
		if (!g_the_chip->threeaxis_dhall_ops ||
				!g_the_chip->threeaxis_dhall_ops->dump_regs) {
			TRI_KEY_ERR("dump DHALL_2 error\n");
		} else {
			g_the_chip->threeaxis_dhall_ops->dump_regs(s);
			TRI_KEY_LOG("%s  dump_regs\n" , __func__);
		}
		break;
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return;
	}
}

int oplus_hall_set_reg(unsigned int id, int reg, int val)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->set_reg)
			return -EINVAL;
		else
			return g_the_chip->dhall_down_ops->set_reg(reg, val);
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->set_reg)
			return -EINVAL;
		else
			return g_the_chip->dhall_up_ops->set_reg(reg, val);
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}
}

bool oplus_hall_is_power_on(void)
{
	if (!g_the_chip || !g_the_chip->dhall_down_ops ||
		!g_the_chip->dhall_down_ops->is_power_on ||
		!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->is_power_on) {
	} else {
		if (g_the_chip->dhall_down_ops->is_power_on() ||
			g_the_chip->dhall_up_ops->is_power_on())
			return true;
		else
			return false;
	}
	return false;
}

static void threeaxis_reboot_get_position(struct extcon_dev_data *chip)
{
	int xdata;
	if (chip->hall_value.hall_y < 0)
		xdata = -chip->hall_value.hall_x;
	else
		xdata = chip->hall_value.hall_x;

	if (xdata > chip->default_up_xdata)
		chip->position = UP_STATE;
	else if (-xdata > chip->default_down_xdata)
		chip->position = DOWN_STATE;
	else
		chip->position = MID_STATE;

	last_position = chip->position;
}

static void reboot_get_position(struct extcon_dev_data *chip)
{
	short delta;
	short up_data1;
	short down_data1;

	if (chip->dhall_data1 < 0 || chip->dhall_data0 < 0) {
		up_data1 = -chip->dhall_data1;
		down_data1 = -chip->dhall_data0;
		delta = up_data1 - down_data1;
	} else
		delta = chip->dhall_data1 - chip->dhall_data0;
	if (delta > 30)
		chip->position = UP_STATE;
	else if (-delta > 30)
		chip->position = DOWN_STATE;
	else
		chip->position = MID_STATE;
	last_position = chip->position;
}

static int interf_get_position(struct extcon_dev_data *chip)
{
	short delta0 = 0;
	short delta1 = 0;
	int   offect = 0;
	int   interf_cnt = 0;

	delta0 = chip->dhall_data0 - last_d0;
	delta1 = chip->dhall_data1 - last_d1;
	offect = tol0;

	do {
		if ((delta1 > calib_uphall_um_distance - offect &&
				delta1 < calib_uphall_um_distance + offect) &&
				((delta0 > calib_dnhall_um_distance - offect) &&
				(delta0 < calib_dnhall_um_distance + offect))) {
			if (last_position == MID_STATE)
				return UP_STATE;
		}

		if ((delta1 > calib_uphall_ud_distance - offect &&
			delta1 < calib_uphall_ud_distance + offect) &&
			((delta0 > calib_dnhall_ud_distance - offect) &&
			(delta0 < calib_dnhall_ud_distance + offect))) {
			if (is_has_data_offect != true)
				return UP_STATE;
		}

		if ((delta1 > -calib_uphall_md_distance - offect &&
			delta1 < -calib_uphall_md_distance + offect) &&
			((delta0 > -calib_dnhall_md_distance - offect) &&
			(delta0 < -calib_dnhall_md_distance + offect))) {
			if (last_position == MID_STATE)
				return DOWN_STATE;
		}

		if ((delta1 > -calib_uphall_ud_distance - offect &&
			delta1 < -calib_uphall_ud_distance + offect) &&
			((delta0 > -calib_dnhall_ud_distance - offect) &&
			(delta0 < -calib_dnhall_ud_distance + offect))) {
			if (is_has_data_offect != true)
				return DOWN_STATE;
		}

		if ((delta1 > -calib_uphall_um_distance - offect &&
			delta1 < -calib_uphall_um_distance + offect) &&
			((delta0 > -calib_dnhall_um_distance - offect) &&
			(delta0 < -calib_dnhall_um_distance + offect))) {
			if (last_position == UP_STATE)
				return MID_STATE;
		}

		if ((delta1 > calib_uphall_md_distance - offect &&
			delta1 < calib_uphall_md_distance + offect) &&
			((delta0 > calib_dnhall_md_distance - offect) &&
			(delta0 < calib_dnhall_md_distance + offect))) {
			if (last_position == DOWN_STATE)
				return MID_STATE;
		}

		interf_cnt++;
		offect += FLOD_INTERF_OFFECT;
		if (is_has_data_offect == true)
			TRI_KEY_LOG("foldPrj:not match every mode state, must retry %d, offect:%d\n", interf_cnt, offect);
	} while (is_has_data_offect == true && interf_cnt <= FLOD_INTERF_MAX);

	return -EINVAL;
}

static int get_position(struct extcon_dev_data *chip)
{
	short diff;

	diff = chip->dhall_data1 - chip->dhall_data0;
	if (chip->dhall_data0 > 0) {
		if (diff > calib_upvaluemin - up_mid_tol &&
			diff < calib_upvaluemin + up_tolerance)
			chip->position = UP_STATE;
		if (calib_mdvaluemin < 0) {
			if (diff > calib_mdvaluemin - mid_down_tol &&
				diff < calib_mdvaluemin + up_mid_tol)
				chip->position = MID_STATE;
			}
		if (calib_mdvaluemin > 0 || calib_mdvaluemin == 0) {
			if (diff > calib_mdvaluemin - mid_down_tol &&
				diff < calib_mdvaluemin + up_mid_tol)
				chip->position = MID_STATE;
			}
		if (diff > calib_dnvaluemin - down_tolerance &&
			diff < calib_dnvaluemin + mid_down_tol)
			chip->position = DOWN_STATE;
	} else {
		if (diff > calib_upvaluemin - up_tolerance &&
			diff < calib_upvaluemin + up_mid_tol)
			chip->position = UP_STATE;
		if (calib_mdvaluemin < 0) {
			if (diff > calib_mdvaluemin - mid_down_tol &&
				diff < calib_mdvaluemin + up_mid_tol)
				chip->position = MID_STATE;
			}
		if (calib_mdvaluemin > 0 || calib_mdvaluemin == 0) {
			if (diff > calib_mdvaluemin - mid_down_tol &&
				diff < calib_mdvaluemin + up_mid_tol)
				chip->position = MID_STATE;
			}
		if (diff > calib_dnvaluemin - mid_down_tol &&
			diff < calib_dnvaluemin + down_tolerance)
			chip->position = DOWN_STATE;
	}
	return 0;
}

static int judge_interference(struct extcon_dev_data *chip)
{
	short delta;
	short sum;

	delta = chip->dhall_data1 - chip->dhall_data0;
	TRI_KEY_LOG("tri_key:delta is %d\n", delta);
	sum = chip->dhall_data0 + chip->dhall_data1;
	TRI_KEY_LOG("tri_key:sum is %d\n", sum);
	if (chip->dhall_data1 > 0) {/*positive number*/
		if (delta > calib_upvaluemin - up_mid_tol &&
			delta < calib_upvaluemin + up_tolerance) {
			TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
				calib_upvaluemin, calib_upvaluesum);
			if (sum < calib_upvaluesum - tol2 ||
				sum > calib_upvaluesum + tol2) {
				chip->interf = 1;
				chip->state = 1;
			} else {
				chip->interf = 0;
				chip->state = 1;
			}
			return 0;
		}
		if (calib_mdvaluemin < 0) {
			if (delta > calib_mdvaluemin - mid_down_tol &&
				delta < calib_mdvaluemin + up_mid_tol) {
				TRI_KEY_LOG("calibMin:%d,calib_Sum:%d\n",
					calib_mdvaluemin, calib_mdvaluesum);

				if (sum > calib_mdvaluesum + tol2 ||
					sum < calib_mdvaluesum - tol2) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
				}
				return 0;
			}
		}
		if (calib_mdvaluemin > 0 || calib_mdvaluemin == 0) {
			if (delta > calib_mdvaluemin - mid_down_tol &&
				delta < calib_mdvaluemin + up_mid_tol) {
				TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
					calib_mdvaluemin, calib_mdvaluesum);

				if (sum > calib_mdvaluesum + tol2 ||
					sum < calib_mdvaluesum - tol2) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
					}
				return 0;
				}
			}
		if (delta > calib_dnvaluemin - down_tolerance &&
			delta < calib_dnvaluemin + up_mid_tol) {
			TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
				calib_dnvaluemin, calib_dnvaluesum);

			if (sum < calib_dnvaluesum - tol2 ||
				sum > calib_dnvaluesum + tol2) {
				chip->interf = 1;
				chip->state = 3;
			} else {
				chip->interf = 0;
				chip->state = 3;
			}
			return 0;
		}
		if (chip->updown_to_mid_support &&
			(last_position == UP_STATE) &&
			(delta < (calib_mdvaluemin - mid_down_tol))) {
			TRI_KEY_LOG("calib_Min:%d, mid_down_tol:%d\n", calib_mdvaluemin, mid_down_tol);
			if (sum > calib_mdvaluesum + tol2 || sum < calib_mdvaluesum - tol2) {
				chip->interf = 1;
				chip->state = 2;
			} else {
				chip->interf = 0;
				chip->state = 2;
				chip->position = MID_STATE;
			}
			return 0;
		}
		if (chip->updown_to_mid_support &&
			(last_position == DOWN_STATE) &&
			(delta > (calib_mdvaluemin + up_mid_tol))) {
			TRI_KEY_LOG("calib_Min:%d, up_mid_tol:%d\n", calib_mdvaluemin, up_mid_tol);
			if (sum > calib_mdvaluesum + tol2 || sum < calib_mdvaluesum - tol2) {
				chip->interf = 1;
				chip->state = 2;
			} else {
				chip->interf = 0;
				chip->state = 2;
				chip->position = MID_STATE;
			}
			return 0;
		}
		chip->interf = 1;
		chip->state = 0;
	} else {/*the hall data is negative number*/
		if (delta > calib_upvaluemin - up_tolerance &&
			delta < calib_upvaluemin + up_mid_tol) {
			TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
				calib_upvaluemin, calib_upvaluesum);

			if (sum < calib_upvaluesum - tol2 ||
				sum > calib_upvaluesum + tol2) {
				chip->interf = 1;
				chip->state = 1;
			} else {
				chip->interf = 0;
				chip->state = 1;
			}
			return 0;
		}
		if (calib_mdvaluemin < 0) {
			if (delta > calib_mdvaluemin - mid_down_tol &&
				delta < calib_mdvaluemin + up_mid_tol) {
				TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
					calib_mdvaluemin, calib_mdvaluesum);

				if (sum > calib_mdvaluesum + tol2 ||
					sum < calib_mdvaluesum - tol2) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
				}
				return 0;
			}
		}
		if (calib_mdvaluemin > 0 || calib_mdvaluemin == 0) {
			if (delta > calib_mdvaluemin - mid_down_tol &&
				delta < calib_mdvaluemin + up_mid_tol) {
				TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
					calib_mdvaluemin, calib_mdvaluesum);

				if (sum > calib_mdvaluesum + tol2 ||
						sum < calib_mdvaluesum - tol2) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
					}
				return 0;
				}
		}
		if (delta > calib_dnvaluemin - mid_down_tol &&
			delta < calib_dnvaluemin + down_tolerance) {
			TRI_KEY_LOG("tri_key:calib_Min:%d,calib_Sum:%d\n",
				calib_dnvaluemin, calib_dnvaluesum);

			if (sum < calib_dnvaluesum - tol2 ||
				sum > calib_dnvaluesum + tol2) {
				chip->interf = 1;
				chip->state = 3;
			} else {
				chip->interf = 0;
				chip->state = 3;
			}
			return 0;
		}
		chip->interf = 1;
		chip->state = 0;
	}
	return -EINVAL;
}

/* judge threeaxis hall interference*/
static void threeaxis_judge_interference(struct extcon_dev_data *chip)
{
	int interf = 0;
	int sum = 0;
	int sum_limit_min = 0;
	int sum_limit_max = 0;
	int z_limit_min = 0;
	int z_limit_max = 0;
	#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
	char payload[1024] = {0x00};
	#endif
	TRI_KEY_LOG("%s call\n", __func__);
	sum = abs(chip->hall_value.hall_x) + abs(chip->hall_value.hall_y);
	sum_limit_min = (abs(chip->threeaxis_calib_data[0]) + abs(chip->threeaxis_calib_data[1])
				+ abs(chip->threeaxis_calib_data[3]) + abs(chip->threeaxis_calib_data[4])
				+ abs(chip->threeaxis_calib_data[6]) + abs(chip->threeaxis_calib_data[7])) / 3 - chip->interf_exist_sumlimit;
	sum_limit_max = (abs(chip->threeaxis_calib_data[0]) + abs(chip->threeaxis_calib_data[1])
				+ abs(chip->threeaxis_calib_data[3]) + abs(chip->threeaxis_calib_data[4])
				+ abs(chip->threeaxis_calib_data[6]) + abs(chip->threeaxis_calib_data[7])) / 3 + chip->interf_exist_sumlimit;
	z_limit_min = (chip->threeaxis_calib_data[2] + chip->threeaxis_calib_data[5] + chip->threeaxis_calib_data[8])/3 - chip->interf_exist_zlimit;
	z_limit_max = (chip->threeaxis_calib_data[2] + chip->threeaxis_calib_data[5] + chip->threeaxis_calib_data[8])/3 + chip->interf_exist_zlimit;

	if ((sum <sum_limit_min) || (sum > sum_limit_max)  || (chip->hall_value.hall_z < z_limit_min) || (chip->hall_value.hall_z > z_limit_max)) {
		interf = 1;
		TRI_KEY_LOG("%s has interf,sum = %d [%d, %d], z =%d [%d, %d]\n",
				__func__, sum, sum_limit_min, sum_limit_max, chip->hall_value.hall_z, z_limit_min, z_limit_max);
		#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
			scnprintf(payload, sizeof(payload),
					"NULL$$EventField@@Interf%d$$FieldData@@cnt$$detailData@@pos%d %d %d %d",
					interf,
					chip->position, chip->hall_value.hall_x, chip->hall_value.hall_y, chip->hall_value.hall_z);
			oplus_kevent_fb(FB_TRI_STATE_KEY, TRIKEY_FB_INTERF_TYPE, payload);
		#endif
	}
	chip->interf = interf;
	return;
}

static int threeaxis_get_data(struct extcon_dev_data *chip)
{
	int res = 0;
	res = oplus_hall_get_data(DHALL_2);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get three_axis_hall DHALL_2 data failed,res =%d\n", res);
		return res;
	}

	return res;
}

static int oplus_get_data(struct extcon_dev_data *chip)
{
	int res = 0;

	res = oplus_hall_get_data(DHALL_0);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get DHALL_0 data failed,res =%d\n", res);
		return res;
	}
	res = oplus_hall_get_data(DHALL_1);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get DHALL_1 data failed,res =%d\n", res);
		return res;
	}

	return res;
}

static void threeaxis_update_position(struct extcon_dev_data *chip , int xtolen, int ytolen, int ztolen) {
	int xup_limit = 0;
	int xdown_limit = 0;
	int xmid_limit = 0;
	int ymid_limit = 0;
	TRI_KEY_LOG("%s: call interf = %d xtolen = %d, ytonlen=%d, ztolen =%d \n", __func__, chip->interf, xtolen, ytolen, ztolen);
	if (chip->threeaxis_calib_data[1] < 0) {
		xup_limit = chip->threeaxis_calib_data[0] + chip->position_tolen[0];
		xdown_limit = chip->threeaxis_calib_data[6] - chip->position_tolen[0];
		xmid_limit = chip->threeaxis_calib_data[3] - chip->position_tolen[0];
		ymid_limit = abs(chip->threeaxis_calib_data[4]) - chip->position_tolen[1];
		if (chip->hall_value.hall_x  < (xup_limit + xtolen)) {
				chip->state = 0;
				chip->position = UP_STATE;
				last_position = chip->position;
				chip->pre_hall_value.hall_x = chip->hall_value.hall_x;
				chip->pre_hall_value.hall_y = chip->hall_value.hall_y;
				chip->pre_hall_value.hall_z = chip->hall_value.hall_z;
				TRI_KEY_LOG(" UP_STATE hallx=%d xup_limit =%d the position is %d\n", chip->hall_value.hall_x, xup_limit, chip->state);
		}

		if (chip->hall_value.hall_x  > (xdown_limit - xtolen)) {
				chip->state = 1;
				chip->position = DOWN_STATE;
				last_position = chip->position;
				chip->pre_hall_value.hall_x = chip->hall_value.hall_x;
				chip->pre_hall_value.hall_y = chip->hall_value.hall_y;
				chip->pre_hall_value.hall_z = chip->hall_value.hall_z;
				TRI_KEY_LOG("DOWN_STATE hall x=%d  xdown_limit=%d the position is %d\n", chip->hall_value.hall_x, xdown_limit, chip->state);
		}
		if ((chip->hall_value.hall_x > (xmid_limit - xtolen)) && (abs(chip->hall_value.hall_y) > (ymid_limit + ytolen))) {
				chip->state = 2;
				chip->position = MID_STATE;
				last_position = chip->position;
				chip->pre_hall_value.hall_x = chip->hall_value.hall_x;
				chip->pre_hall_value.hall_y = chip->hall_value.hall_y;
				chip->pre_hall_value.hall_z = chip->hall_value.hall_z;
				TRI_KEY_LOG("MID_STATE hall x=%d , hall y = %d , xmid_limit=%d, ymid_limit=%d, the position is %d\n", \
				chip->hall_value.hall_x, chip->hall_value.hall_y, xmid_limit, ymid_limit, chip->state);
		}

	} else {
		xup_limit = chip->threeaxis_calib_data[0] - chip->position_tolen[0];
		xdown_limit = chip->threeaxis_calib_data[6] + chip->position_tolen[0];
		xmid_limit = chip->threeaxis_calib_data[3] - chip->position_tolen[0];
		ymid_limit = abs(chip->threeaxis_calib_data[4]) - chip->position_tolen[1];
		TRI_KEY_LOG("%s: xmid_limit = %d ymid_limit = %d,  \n", __func__, xmid_limit, ymid_limit);
		if (chip->hall_value.hall_x  > (xup_limit + xtolen)) {
				chip->state = 0;
				chip->position = UP_STATE;
				last_position = chip->position;
				chip->pre_hall_value.hall_x = chip->hall_value.hall_x;
				chip->pre_hall_value.hall_y = chip->hall_value.hall_y;
				chip->pre_hall_value.hall_z = chip->hall_value.hall_z;
				TRI_KEY_LOG(" UP_STATE hallx=%d xup_limit =%d the position is %d\n", \
				chip->hall_value.hall_x, xup_limit, chip->state);
		}

		if (chip->hall_value.hall_x  < (xdown_limit - xtolen)) {
				chip->state = 1;
				chip->position = DOWN_STATE;
				last_position = chip->position;
				chip->pre_hall_value.hall_x = chip->hall_value.hall_x;
				chip->pre_hall_value.hall_y = chip->hall_value.hall_y;
				chip->pre_hall_value.hall_z = chip->hall_value.hall_z;
				TRI_KEY_LOG("DOWN_STATE hall x=%d  xdown_limit=%d the position is %d\n", \
				chip->hall_value.hall_x, xdown_limit, chip->state);
		}
		if ((chip->hall_value.hall_x > (xmid_limit - xtolen)) && (abs(chip->hall_value.hall_y) > (ymid_limit + ytolen))) {
				chip->state = 2;
				chip->position = MID_STATE;
				last_position = chip->position;
				chip->pre_hall_value.hall_x = chip->hall_value.hall_x;
				chip->pre_hall_value.hall_y = chip->hall_value.hall_y;
				chip->pre_hall_value.hall_z = chip->hall_value.hall_z;
				TRI_KEY_LOG("MID_STATE hall x=%d , hall y = %d , xmid_limit=%d, ymid_limit=%d, the position is %d\n", \
				chip->hall_value.hall_x, chip->hall_value.hall_y, xmid_limit, ymid_limit, chip->state);
		}
	}
	last_interf = chip->interf;
}

static int threeaxis_reupdata_threshold(struct extcon_dev_data *chip)
{
	int res = 0;
	TRI_KEY_LOG("%s:call", __func__);
	switch (chip->position) {
	case UP_STATE:
			res = threeaxis_hall_update_threshold(DHALL_2, UP_STATE,
			-3500, -3000, &chip->hall_value, chip->interf);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("%s: UP_STATE low:-3500,high: -3000", __func__);
		break;
	case DOWN_STATE:
		res = threeaxis_hall_update_threshold(DHALL_2, DOWN_STATE,
			3500, 4000, &chip->hall_value, chip->interf);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("%s:DOWN_STATE low:3500,high: 4000", __func__);
		break;
	case MID_STATE:
		res = threeaxis_hall_update_threshold(DHALL_2, MID_STATE,
			14000, 14200, &chip->hall_value, chip->interf);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("%s: MID_STATE low:14000,high: 14200", __func__);
		break;
	default:
		break;
	}
fail:
	return res;
}

/* get threeaxis hall position*/
static int threeaxis_get_position(struct extcon_dev_data *chip)
{
	int xtolen = 0;
	int ytolen = 0;
	int ztolen = 0;
	int xinterf = 0;
	int yinterf = 0;
	int zinterf = 0;
	int res = 0;
	int um_xtol = chip->up_mid_tolen[0];
	int um_ytol = chip->up_mid_tolen[1];
	int ud_xtol = chip->up_down_tolen[0];
	int ud_ytol = chip->up_down_tolen[1];
	int mu_xtol = chip->mid_up_tolen[0];
	int mu_ytol = chip->mid_up_tolen[1];
	int md_xtol = chip->mid_down_tolen[0];
	int md_ytol = chip->mid_down_tolen[1];
	int du_xtol = chip->down_up_tolen[0];
	int du_ytol = chip->down_up_tolen[1];
	int dm_xtol = chip->down_mid_tolen[0];
	int dm_ytol = chip->down_mid_tolen[1];
	TRI_KEY_LOG("%s: call interf = %d,last_position =%d\n", __func__, chip->interf, last_position);
	if (chip->interf == 0) {
		threeaxis_update_position(chip, xtolen , ytolen, ztolen);
		res = threeaxis_reupdata_threshold(g_the_chip);
	} else {
		xinterf = chip->hall_value.hall_x;
		yinterf = chip->hall_value.hall_y;
		zinterf = chip->hall_value.hall_z;
		msleep(50);
		res = threeaxis_get_data(chip);
		if ((abs(chip->hall_value.hall_x - xinterf) > chip->interf_stable_xlimit) || (abs(chip->hall_value.hall_y - \
		yinterf) > chip->interf_stable_ylimit) || (abs(chip->hall_value.hall_z - zinterf) > chip->interf_stable_zlimit)) {
			TRI_KEY_LOG("%s: to next parse;   xtolen = %d,  ytolen = %d,chip->position =%d, last_interf=%d \n", __func__, xtolen, ytolen, chip->position, last_interf);
			xinterf = chip->hall_value.hall_x;
			yinterf = chip->hall_value.hall_y;
			zinterf = chip->hall_value.hall_z;
			msleep(50);
			res = threeaxis_get_data(chip);
			if ((abs(chip->hall_value.hall_x - xinterf) > chip->interf_stable_xlimit) || (abs(chip->hall_value.hall_y - \
			yinterf) > chip->interf_stable_ylimit) || (abs(chip->hall_value.hall_z - zinterf) > chip->interf_stable_zlimit)) {
				TRI_KEY_LOG("%s: it is not stable interfence\n", __func__);
				return res;
			}
		}
		xtolen = chip->hall_value.hall_x - chip->pre_hall_value.hall_x;
		ytolen = chip->hall_value.hall_y - chip->pre_hall_value.hall_y;
		switch (last_position) {
		case UP_STATE:
		if ((xtolen > (chip->threeaxis_calib_data[3] - chip->threeaxis_calib_data[0] - um_xtol)) && (xtolen < (chip->threeaxis_calib_data[3] - \
		chip->threeaxis_calib_data[0] + um_xtol)) && (ytolen > (chip->threeaxis_calib_data[4] - chip->threeaxis_calib_data[1] - \
		um_ytol)) && (ytolen < (chip->threeaxis_calib_data[4] - chip->threeaxis_calib_data[1] + um_ytol)))
			chip->position = last_position + 2;
		if ((xtolen > (chip->threeaxis_calib_data[6] - chip->threeaxis_calib_data[0] - ud_xtol)) && (xtolen < (chip->threeaxis_calib_data[6] - \
		chip->threeaxis_calib_data[0] + ud_xtol)) && (ytolen > (chip->threeaxis_calib_data[7] - chip->threeaxis_calib_data[1] - \
		ud_ytol)) && (ytolen < (chip->threeaxis_calib_data[7] - chip->threeaxis_calib_data[1] + ud_ytol)))
			chip->position = last_position + 1;
		break;
		case DOWN_STATE:
		if ((xtolen > (chip->threeaxis_calib_data[3] - chip->threeaxis_calib_data[6] - dm_xtol)) && (xtolen < (chip->threeaxis_calib_data[3] - \
		chip->threeaxis_calib_data[6] + dm_xtol)) && (ytolen > (chip->threeaxis_calib_data[4] - chip->threeaxis_calib_data[7] - \
		dm_ytol)) && (ytolen < (chip->threeaxis_calib_data[4] - chip->threeaxis_calib_data[7] + dm_ytol)))
			chip->position = last_position + 1;
		if ((xtolen > (chip->threeaxis_calib_data[0] - chip->threeaxis_calib_data[6] - du_xtol)) && (xtolen < (chip->threeaxis_calib_data[0] - \
		chip->threeaxis_calib_data[6] + du_xtol)) && (ytolen > (chip->threeaxis_calib_data[1] - chip->threeaxis_calib_data[7] - \
		du_ytol)) && (ytolen < (chip->threeaxis_calib_data[1] - chip->threeaxis_calib_data[7] + du_ytol)))
			chip->position = last_position -1;
		break;
		case MID_STATE:
		if ((xtolen > (chip->threeaxis_calib_data[0] - chip->threeaxis_calib_data[3] - mu_xtol)) && (xtolen < (chip->threeaxis_calib_data[0] - \
		chip->threeaxis_calib_data[3] + mu_xtol)) && (ytolen > (chip->threeaxis_calib_data[1] - chip->threeaxis_calib_data[4] - \
		mu_ytol)) && (ytolen < (chip->threeaxis_calib_data[1] - chip->threeaxis_calib_data[4] + mu_ytol)))
			chip->position = last_position - 2;
		if ((xtolen > (chip->threeaxis_calib_data[6] - chip->threeaxis_calib_data[3] - md_xtol)) && (xtolen < (chip->threeaxis_calib_data[6] - \
		chip->threeaxis_calib_data[3] + md_xtol)) && (ytolen > (chip->threeaxis_calib_data[7] - chip->threeaxis_calib_data[4] - \
		md_ytol)) && (ytolen < (chip->threeaxis_calib_data[7] - chip->threeaxis_calib_data[4] + md_ytol)))
			chip->position = last_position - 1;
		break;
		default:
		break;
		}
		last_position = chip->position;
		chip->pre_hall_value.hall_x = chip->hall_value.hall_x;
		chip->pre_hall_value.hall_y = chip->hall_value.hall_y;
		chip->pre_hall_value.hall_z = chip->hall_value.hall_z;
		last_interf = chip->interf;
		TRI_KEY_LOG("%s: call int stable interference   xtolen = %d,  ytolen = %d, chip->position =%d, last_interf=%d \n", \
		__func__, xtolen, ytolen, chip->position, last_interf);
		res = threeaxis_reupdata_threshold(g_the_chip);
	}
	return res;
}

static int reupdata_threshold(struct extcon_dev_data *chip)
{
	int res = 0;
	int tolen[3] = {22, 22, 22};

	if ((g_the_chip->tolen[0] > 0) && (g_the_chip->tolen[1] > 0) &&
		(g_the_chip->tolen[2] > 0)) {
		tolen[0] = g_the_chip->tolen[0];
		tolen[1] = g_the_chip->tolen[1];
		tolen[2] = g_the_chip->tolen[2];
	}

	switch (chip->position) {
	case UP_STATE:
			res = oplus_hall_update_threshold(DHALL_1, UP_STATE,
			chip->dhall_data1-tolen[0], chip->dhall_data1+tolen[0]);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
			if (g_the_chip->new_threshold_support) {
				res = oplus_hall_update_threshold(DHALL_0, UP_STATE,
					chip->dhall_data0 - tolen[0], chip->dhall_data0 + tolen[0]);
				if (res < 0) {
					TRI_KEY_LOG("updata_threshold fail:%d\n", res);
					goto fail;
				}
				TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high: %d\n",
				chip->dhall_data1 - tolen[0], chip->dhall_data1 + tolen[0]);
				TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high: %d\n",
				chip->dhall_data0 - tolen[0], chip->dhall_data0 + tolen[0]);
			} else {
				res = oplus_hall_update_threshold(DHALL_0, UP_STATE,
					-500, 500);
				if (res < 0) {
					TRI_KEY_LOG("updata_threshold fail:%d\n", res);
					goto fail;
				}
				TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high: %d\n",
				chip->dhall_data1 - tolen[0], chip->dhall_data1 + tolen[0]);
			}

		oplus_hall_clear_irq(DHALL_1);
		oplus_hall_clear_irq(DHALL_0);
		break;
	case MID_STATE:
		if (chip->dhall_data0 < 0 || chip->dhall_data1 < 0) {
			res = oplus_hall_update_threshold(DHALL_1, MID_STATE,
			chip->dhall_data1 - tolen[1], chip->dhall_data1 + tolen[1]);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high:%d\n",
			chip->dhall_data1 - tolen[1], chip->dhall_data1 + tolen[1]);
		} else {
			res = oplus_hall_update_threshold(DHALL_1, MID_STATE,
			chip->dhall_data1 - tolen[1], chip->dhall_data1 + tolen[1]);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high:%d\n",
			chip->dhall_data1 - tolen[1], chip->dhall_data1 + tolen[1]);
		}
		oplus_hall_clear_irq(DHALL_1);
		if (chip->dhall_data0 < 0 || chip->dhall_data1 < 0) {
			res = oplus_hall_update_threshold(DHALL_0, MID_STATE,
			chip->dhall_data0 - tolen[1], chip->dhall_data0 + tolen[1]);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high:%d\n",
		chip->dhall_data0 - tolen[1], chip->dhall_data0 + tolen[1]);
		} else {
			res = oplus_hall_update_threshold(DHALL_0, MID_STATE,
			chip->dhall_data0 - tolen[1], chip->dhall_data0 + tolen[1]);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high:%d\n",
			chip->dhall_data0 - tolen[1], chip->dhall_data0 + tolen[1]);
		}
		oplus_hall_clear_irq(DHALL_0);
		break;
	case DOWN_STATE:
		res = oplus_hall_update_threshold(DHALL_0, DOWN_STATE,
			chip->dhall_data0 - tolen[2], chip->dhall_data0 + tolen[2]);
		if (res < 0) {
			TRI_KEY_LOG("updata_threshold fail:%d\n", res);
			goto fail;
		}
		TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high:%d\n",
			chip->dhall_data0 - tolen[2], chip->dhall_data0 + tolen[2]);

		if (g_the_chip->new_threshold_support) {
			res = oplus_hall_update_threshold(DHALL_1, DOWN_STATE,
				chip->dhall_data1 - tolen[2], chip->dhall_data1 + tolen[2]);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
			TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high:%d\n",
			chip->dhall_data1 - tolen[2], chip->dhall_data1 + tolen[2]);
		} else {
			res = oplus_hall_update_threshold(DHALL_1, DOWN_STATE,
				-500, 500);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		}
		oplus_hall_clear_irq(DHALL_0);
		oplus_hall_clear_irq(DHALL_1);
		break;
	default:
		break;
		}
fail:
	last_d0 = chip->dhall_data0;
	last_d1 = chip->dhall_data1;
	last_interf = chip->interf;
	TRI_KEY_LOG("[last_d0:%d] [last_d1:%d] [last_interf:%d]\n",
		last_d0, last_d1, last_interf);
	oplus_hall_clear_irq(DHALL_0);
	oplus_hall_clear_irq(DHALL_1);
	return res;
}



static void report_key_value(struct extcon_dev_data *chip)
{
	if (chip->position == DOWN_STATE) {
		chip->state = 3;
		input_report_key(chip->input_dev, KEY_F3, 3);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, KEY_F3, 0);
		input_sync(chip->input_dev);
		TRI_KEY_LOG("tri_key: report down key successful!\n");
	} else if (chip->position == UP_STATE) {
		chip->state = 1;
		TRI_KEY_LOG("tri_key: report up key successful!\n");
		input_report_key(chip->input_dev, KEY_F3, 1);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, KEY_F3, 0);
		input_sync(chip->input_dev);
	} else if (chip->position == MID_STATE) {
		chip->state = 2;
		TRI_KEY_LOG("tri_key: report mid key successful!\n");
		input_report_key(chip->input_dev, KEY_F3, 2);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, KEY_F3, 0);
		input_sync(chip->input_dev);
	} else
		TRI_KEY_LOG("no report\n");
}

static int report_calibration_location(struct extcon_dev_data *chip)
{
	oplus_get_data(chip);
	judge_interference(chip);
	get_position(chip);
	reupdata_threshold(chip);
	if (chip->position == last_position) {
		TRI_KEY_LOG("no report\n");
		goto err;
	} else
		report_key_value(chip);
	last_position = chip->position;
	return 0;
err:
	return -EINVAL;
}

static int threeaxis_judge_calibration_data(struct extcon_dev_data *chip)
{
	int res = 0;
	if (chip->threeaxis_calib_data[1] == 0 || chip->threeaxis_calib_data[4] == 0 ||
	chip->threeaxis_calib_data[7] == 0) {
		res = threeaxis_get_data(chip);
		threeaxis_reboot_get_position(chip);
		if (chip->position == UP_STATE) {
			chip->threeaxis_calib_data[0] = chip->hall_value.hall_x;
			chip->threeaxis_calib_data[1] = chip->hall_value.hall_y;
			chip->threeaxis_calib_data[2] = chip->hall_value.hall_z;
			TRI_KEY_LOG("%s: UP_STATE calib_data =[%d %d %d]\n", __func__, chip->threeaxis_calib_data[0], \
			chip->threeaxis_calib_data[1], chip->threeaxis_calib_data[2]);
		}
		if (chip->position == MID_STATE) {
			chip->threeaxis_calib_data[3] = chip->hall_value.hall_x;
			chip->threeaxis_calib_data[4] = chip->hall_value.hall_y;
			chip->threeaxis_calib_data[5] = chip->hall_value.hall_z;
			TRI_KEY_LOG("%s: UP_STATE calib_data =[%d %d %d]\n", __func__, chip->threeaxis_calib_data[3], \
			chip->threeaxis_calib_data[4], chip->threeaxis_calib_data[5]);
		}
		if (chip->position == DOWN_STATE) {
			chip->threeaxis_calib_data[6] = chip->hall_value.hall_x;
			chip->threeaxis_calib_data[7] = chip->hall_value.hall_y;
			chip->threeaxis_calib_data[8] = chip->hall_value.hall_z;
			TRI_KEY_LOG("%s: UP_STATE calib_data =[%d %d %d]\n", __func__, chip->threeaxis_calib_data[6], \
			chip->threeaxis_calib_data[7], chip->threeaxis_calib_data[8]);
		}
		report_key_value(chip);
		res = threeaxis_reupdata_threshold(chip);
		return -EINVAL;
	}
	return 0;
}

static int judge_calibration_data(struct extcon_dev_data *chip)
{
	if (calib_upvaluemin == 0 || calib_upvaluesum == 0 ||
	calib_mdvaluesum == 0 || calib_dnvaluemin == 0
						|| calib_dnvaluesum == 0) {
		oplus_get_data(chip);
		reboot_get_position(chip);
		if (chip->position == UP_STATE) {
			calib_upvaluemin = chip->dhall_data1 -
						chip->dhall_data0;
			calib_upvaluesum = chip->dhall_data1 +
						chip->dhall_data0;
			TRI_KEY_LOG("UP_MIN is%d,UP_SUM is %d\n",
				calib_upvaluemin, calib_upvaluesum);
		}
		if (chip->position == MID_STATE) {
			calib_mdvaluemin = chip->dhall_data1 -
						chip->dhall_data0;
			calib_mdvaluesum = chip->dhall_data1 +
						chip->dhall_data0;
			TRI_KEY_LOG("MID_MIN is%d,MID_SUM is %d\n",
				calib_mdvaluemin, calib_mdvaluesum);
		}
		if (chip->position == DOWN_STATE) {
			calib_dnvaluemin = chip->dhall_data1 -
						chip->dhall_data0;
			calib_dnvaluesum = chip->dhall_data1 +
						chip->dhall_data0;
			TRI_KEY_LOG("UP_MIN is%d,UP_SUM is %d\n",
				calib_dnvaluemin, calib_dnvaluesum);
		}
		report_key_value(chip);
		reupdata_threshold(chip);
		return -EINVAL;
	}
	return 0;
}


int oplus_hall_irq_handler(unsigned int id)
{
	TRI_KEY_LOG("%d tri_key:call :%s\n", id, __func__);
	if (!g_the_chip)
		TRI_KEY_LOG("g_the_chip null\n ");
	else {
		if (is_has_esd_check == true)
			g_the_chip->trigger_id = id;
		schedule_work(&g_the_chip->dwork);
	}
	return IRQ_HANDLED;
}
EXPORT_SYMBOL(oplus_hall_irq_handler);

int threeaxis_hall_irq_handler(unsigned int id)
{
	int err = 0;
	if (!g_the_chip) {
		TRI_KEY_LOG("g_the_chip null\n :%s\n", __func__);
		return -EINVAL;
	}
	disable_irq(g_the_chip->irq);
	TRI_KEY_LOG("%d tri_key:call :%s\n", id, __func__);
	err = threeaxis_judge_calibration_data(g_the_chip);
	if (err < 0) {
		enable_irq(g_the_chip->irq);
		return 0;
	}
	err = threeaxis_get_data(g_the_chip);
	threeaxis_judge_interference(g_the_chip);
	err = threeaxis_get_position(g_the_chip);
	report_key_value(g_the_chip);
	msleep(10);
	enable_irq(g_the_chip->irq);
	return 0;
}
EXPORT_SYMBOL(threeaxis_hall_irq_handler);


void oplus_hall_control_irq(int hall_id, bool enable)
{
	TRI_KEY_ERR("[ID:%d][enable:%d]\n", hall_id, enable);
	g_the_chip->dhall_up_ops->enable_irq(enable);
	g_the_chip->dhall_down_ops->enable_irq(enable);

	if (hall_id == 0) {
		if (enable == IRQ_DISABLE)
			oplus_hall_clear_irq(DHALL_1);
	} else if (hall_id == 1) {
		if (enable == IRQ_DISABLE)
			oplus_hall_clear_irq(DHALL_0);
	}
}

static void tri_key_dev_work(struct work_struct *work)
{
	struct extcon_dev_data *chip = container_of(work,
			struct extcon_dev_data, dwork);
	int res = 0;
	int position = -1;
	int diff0 = 0;
	int diff1 = 0;
	int count = 0;
	int dhall0_sum = 0;
	int dhall1_sum = 0;
	int aver0 = 0;
	int aver1 = 0;
	ktime_t starttime, endtime;
	u64 usecs64;
	int usecs;
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
	char payload[1024] = {0x00};
#endif
	int interf_type = INTERF_TYPE_NONE;

	mutex_lock(&chip->mtx);

	starttime = ktime_get();
	msleep(50);
	if (!g_the_chip->threeaxis_hall_support) {
		res = judge_calibration_data(chip);
		if (res < 0)
			goto FINAL;

		if (is_has_esd_check == true) {
			oplus_hall_control_irq(g_the_chip->trigger_id, IRQ_DISABLE);
		}
/*get data*/
		res = oplus_get_data(chip);
		if (res < 0) {
			TRI_KEY_LOG("tri_key:get hall data failed!\n");
			goto fail;
		}
		TRI_KEY_LOG("tri_key:data1 is %d, data0 is %d\n",
					chip->dhall_data1, chip->dhall_data0);

/*judge interference*/
		res = judge_interference(chip);
		TRI_KEY_LOG("tri_key[interf:%d last_interf:%d] [chip->state:%d]\n",
						chip->interf, last_interf, chip->state);
		if (!last_interf && chip->interf) {
			interf_type = INTERF_TYPE_1;
			msleep(200);
			oplus_get_data(chip);
			TRI_KEY_LOG("tri_key:data1 is %d, data0 is %d\n",
						chip->dhall_data1, chip->dhall_data0);

			/*judge interference*/
			res = judge_interference(chip);
			TRI_KEY_LOG("tri_key[interf:%d last_interf:%d] [chip->state:%d]\n",
							chip->interf, last_interf, chip->state);
			if (!last_interf && chip->interf) {
				interf_type = INTERF_TYPE_1;
				msleep(200);
				oplus_get_data(chip);
				TRI_KEY_LOG("tri_key:data1 is %d, data0 is %d\n",
							chip->dhall_data1, chip->dhall_data0);

				judge_interference(chip);
			}
		}
	} else {
		res = threeaxis_judge_calibration_data(chip);
		if (res < 0)
			goto fail;
		res = threeaxis_get_data(chip);
		if (res < 0) {
			TRI_KEY_LOG("tri_key:get hall data failed!\n");
			goto fail;
		}
		threeaxis_judge_interference(chip);
		res = threeaxis_get_position(chip);
		mutex_unlock(&chip->mtx);
		return;
	}
/*get position*/
	if (!chip->interf) {
		hrtimer_cancel(&tri_key_timer);
		time = 1;
		if (!last_interf) {
			interf_count = 0;
			get_position(chip);
		TRI_KEY_LOG("tri_key:the position is %d\n", chip->position);
		} else {
			msleep(50);
			oplus_get_data(chip);
			judge_interference(chip);
			if (chip->interf) {
				interf_type = INTERF_TYPE_2;
				goto FINAL;
			} else {
				get_position(chip);
			}
		}
	} else {
		hrtimer_cancel(&tri_key_timer);
		TRI_KEY_LOG("tri_key:time0 is %d\n", time);
		hrtimer_start(&tri_key_timer, ktime_set(time, 0),
			HRTIMER_MODE_REL);
		while (count < 4) {
			msleep(35);
			oplus_hall_get_data(DHALL_0);
			oplus_hall_get_data(DHALL_1);
			dhall0_sum += chip->dhall_data0;
			dhall1_sum += chip->dhall_data1;
			count++;
		}
		aver0 = dhall0_sum / 4;
		aver1 = dhall1_sum / 4;
		if (!last_interf) {
			diff0 = aver0 - chip->dhall_data0;
			diff1 = aver1 - chip->dhall_data1;
			TRI_KEY_LOG("tri_key:diff0 is %d,diff1 is %d\n",
				diff0, diff1);
			if ((diff0 > -10 && diff0 < 10) &&
					(diff1 > -10 && diff1 < 10)) {
				interf_type = INTERF_TYPE_3;
				if (is_has_data_offect == true) {
					TRI_KEY_LOG("[The data is stable now] is_has_data_offect is true not interfence\n");
					get_position(chip);
					goto UPDATA_HTRES;
				} else
					chip->position = last_position;
					TRI_KEY_LOG("[getData stable now] interfType=TYPE3 [position:%d last_position:%d]\n",
						chip->position, last_position);
					goto UPDATA_HTRES;
			} else {/*inconstant interference*/
				interf_type = INTERF_TYPE_4;
				last_interf = chip->interf;
				goto FINAL;
			}
		}
		diff0 = aver0 - chip->dhall_data0;
		diff1 = aver1 - chip->dhall_data1;
		TRI_KEY_LOG("tri_key:diff0 is %d,diff1 is %d\n",
			diff0, diff1);

/*inconstantly interference*/
		if ((diff0 < -10 || diff0 > 10) &&
				(diff1 < -10 || diff1 > 10)) {
			interf_count++;
			if (interf_count == 15) {
				TRI_KEY_LOG("tri_key:count = 15,msleep 5s\n");
				msleep(5000);
				interf_type = INTERF_TYPE_5;
				interf_count = 0;
				goto FINAL;
			}
			TRI_KEY_LOG("tri_key:inconstantlt interference\n");
			interf_type = INTERF_TYPE_6;
			reupdata_threshold(chip);
			goto FINAL;
		}

		chip->dhall_data0 = aver0;
		chip->dhall_data1 = aver1;
		position = interf_get_position(chip);
		if (position == -22) {
			TRI_KEY_LOG("tri_key:get position failed\n");
			interf_type = INTERF_TYPE_7;
		} else {
			interf_type = INTERF_TYPE_8;
			chip->position = position;
		}
	}
	TRI_KEY_LOG("tri_key:t_diff0 is %d,t_diff1 is %d\n",
	chip->dhall_data0 - last_d0, chip->dhall_data1 - last_d1);
/*updata threshold*/
UPDATA_HTRES:
	res = reupdata_threshold(chip);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:updata_threshold failed!\n");
		goto fail;
		}

/*report key value*/
	if (chip->position == last_position)
		goto FINAL;
	else {
		report_key_value(chip);
		last_position = chip->position;
		endtime = ktime_get();
		usecs64 = ktime_to_ns(ktime_sub(endtime, starttime));
		do_div(usecs64, NSEC_PER_USEC);
		usecs = usecs64;
		if (usecs == 0)
			usecs = 1;
		TRI_KEY_LOG("report key after %ld.%03ld msecs\n",
			usecs / USEC_PER_MSEC, usecs % USEC_PER_MSEC);
	}
fail:
	if (res < 0)
		TRI_KEY_LOG("tri_key:dev_work failed,res =%d\n", res);
FINAL:
	if (is_has_esd_check == true)
		oplus_hall_control_irq(g_the_chip->trigger_id, IRQ__ENABLE);
	else
		oplus_hall_disable_irq(1);

#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
	if (interf_type > INTERF_TYPE_NONE) {
		scnprintf(payload, sizeof(payload),
				   "NULL$$EventField@@Interf%d$$FieldData@@cnt%d$$detailData@@pos%d %d %d",
				   interf_type, interf_count,
				   chip->position, chip->dhall_data0, chip->dhall_data1);
		oplus_kevent_fb(FB_TRI_STATE_KEY, TRIKEY_FB_INTERF_TYPE, payload);
	}
#endif
	TRI_KEY_LOG("%s achieve\n", __func__);
	mutex_unlock(&chip->mtx);
}

static enum hrtimer_restart tri_key_status_timeout(struct hrtimer *timer)
{
	schedule_work(&tri_key_timeout_work);
	return HRTIMER_NORESTART;
}

static void tri_key_timeout_work_func(struct work_struct *work)
{
	oplus_get_data(g_the_chip);
	judge_interference(g_the_chip);
	if (g_the_chip->interf) {
		time = time * 2;
		TRI_KEY_LOG("tri_key:time1 is %d\n", time);
		if (time > 2)
			time = 2;
		}
	else {
		get_position(g_the_chip);
		if (g_the_chip->position == last_position)
			return;
		reupdata_threshold(g_the_chip);
		report_key_value(g_the_chip);
		last_position = g_the_chip->position;
		time = 1;
		}
}

static void trikey_speedup_resume(struct work_struct *work)
{
	int err;

	TRI_KEY_LOG("%s call\n", __func__);
	if (g_the_chip->threeaxis_hall_support) {
		disable_irq(g_the_chip->irq);
		err = threeaxis_judge_calibration_data(g_the_chip);
		if (err < 0) {
			enable_irq(g_the_chip->irq);
			return;
		}
		err = threeaxis_get_data(g_the_chip);
		threeaxis_judge_interference(g_the_chip);
		err = threeaxis_get_position(g_the_chip);
		msleep(10);
		enable_irq(g_the_chip->irq);
	} else {
		reupdata_threshold(g_the_chip);
	}
	TRI_KEY_LOG("%s exit\n", __func__);
}

#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static void trikey_panel_notifier_callback(enum panel_event_notifier_tag tag,
		 struct panel_event_notification *notification, void *client_data)
{
	if (!notification) {
		TRI_KEY_LOG("Invalid notification\n");
		return;
	}
	if (notification->notif_type < DRM_PANEL_EVENT_FOR_TOUCH) {
		TRI_KEY_LOG("Notification type:%d, early_trigger:%d",
			notification->notif_type,
			notification->notif_data.early_trigger);
	}

	if (g_hall_dev->bus_ready == false) {
		TRI_KEY_LOG("bus_ready not ready, tp exit\n");
		return;
	}

	if (notification->notif_type == DRM_PANEL_EVENT_UNBLANK && notification->notif_data.early_trigger && g_the_chip->is_suspended) {
		g_the_chip->is_suspended = false;
		queue_work(g_the_chip->speedup_resume_wq, &g_the_chip->speed_up_work);
	} else if (notification->notif_type == DRM_PANEL_EVENT_BLANK) {
		g_the_chip->is_suspended = true;
	}
}

#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
static int trikey_mtk_drm_notifier_callback(struct notifier_block *nb,
	unsigned long event, void *data)
{
	int *blank = (int *)data;

	TRI_KEY_LOG("mtk gki notifier event:%lu, blank:%d",
			event, *blank);

	if (*blank == MTK_DISP_BLANK_UNBLANK && event == MTK_DISP_EARLY_EVENT_BLANK && g_the_chip->is_suspended) {
		g_the_chip->is_suspended = false;
		queue_work(g_the_chip->speedup_resume_wq, &g_the_chip->speed_up_work);
	} else if (*blank == MTK_DISP_BLANK_POWERDOWN) {
		g_the_chip->is_suspended = true;
	}

	return 0;
}

#else
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
	struct msm_drm_notifier *evdata = data;
#else
	struct fb_event *evdata = data;
#endif

	/*to aviod some kernel bug (at fbmem.c some local veriable are not initialized)*/
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
	if (event != MSM_DRM_EARLY_EVENT_BLANK && event != MSM_DRM_EVENT_BLANK)
#else
	if (event != FB_EARLY_EVENT_BLANK && event != FB_EVENT_BLANK)
#endif
		return 0;

	if (evdata && evdata->data) {
		blank = evdata->data;
		TRI_KEY_LOG("%s: event = %ld, blank = %d\n", __func__, event, *blank);
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
		if (*blank == MSM_DRM_BLANK_UNBLANK && event == MSM_DRM_EARLY_EVENT_BLANK && g_the_chip->is_suspended) { /*resume*/
#else
		if (*blank == FB_BLANK_UNBLANK && event == FB_EARLY_EVENT_BLANK && g_the_chip->is_suspended) {  /*resume*/
#endif
			g_the_chip->is_suspended = false;
			queue_work(g_the_chip->speedup_resume_wq, &g_the_chip->speed_up_work);
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
		} else if (*blank == MSM_DRM_BLANK_BLANK) { /*suspend*/
#else
		} else if (*blank == FB_BLANK_BLANK) {  /*suspend*/
#endif
			g_the_chip->is_suspended = true;
		}
	}

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY) || IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
struct drm_panel *trikey_dev_get_panel(struct device_node *of_node)
{
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;
	struct device_node *np;
	char disp_node[32] = {0};

	np = of_find_node_by_name(NULL, "oplus,dsi-display-dev");
	if (!np) {
		TRI_KEY_LOG("[oplus,dsi-display-dev] is missing, try to find [panel] node \n");
		np = of_node;
		TRI_KEY_LOG("np full name = %s \n", np->full_name);
		strncpy(disp_node, "panel", sizeof("panel"));
	} else {
		TRI_KEY_LOG("[oplus,dsi-display-dev] node found \n");
		/* for primary panel */
		strncpy(disp_node, "oplus,dsi-panel-primary", sizeof("oplus,dsi-panel-primary"));
	}
	TRI_KEY_LOG("disp_node = %s \n", disp_node);

	count = of_count_phandle_with_args(np, disp_node, NULL);
	if (count <= 0) {
		TRI_KEY_LOG("can not find [%s] node in dts \n", disp_node);
		return NULL;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, disp_node, i);
		panel = of_drm_find_panel(node);
		TRI_KEY_LOG("panel[%d] IS_ERR =%d \n", i, IS_ERR(panel));
		of_node_put(node);
		if (!IS_ERR(panel)) {
			TRI_KEY_LOG("Find available panel\n");
			return panel;
		}
	}

	return NULL;
}
EXPORT_SYMBOL(trikey_dev_get_panel);
#endif

int oplus_hall_register_notifier(void)
{
	int ret = 0;
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	void *cookie = NULL;
#endif
#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
	g_the_chip->active_panel = g_hall_dev->active_panel;
	g_the_chip->fb_notif.notifier_call = fb_notifier_callback;

	if (g_the_chip->active_panel)
		ret = drm_panel_notifier_register(g_the_chip->active_panel,
				&g_the_chip->fb_notif);
#elif IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	g_the_chip->active_panel = g_hall_dev->active_panel;
	if (g_the_chip->active_panel) {
		cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_PRIMARY,
				PANEL_EVENT_NOTIFIER_CLIENT_TRI_STATE_KEY, g_the_chip->active_panel,
				&trikey_panel_notifier_callback, g_the_chip);

		if (!cookie) {
			TRI_KEY_LOG("Unable to register fb_notifier: %d\n", ret);
		} else {
			g_the_chip->notifier_cookie = cookie;
		}
	}

#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	g_the_chip->disp_notifier.notifier_call = trikey_mtk_drm_notifier_callback;
	if (mtk_disp_notifier_register("Oplus_trikey", &g_the_chip->disp_notifier)) {
		TRI_KEY_LOG("Failed to register disp notifier client!!\n");
	}

#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
	g_the_chip->fb_notif.notifier_call = fb_notifier_callback;
	ret = msm_drm_register_client(&g_the_chip->fb_notif);

	if (ret) {
		TRI_KEY_LOG("Unable to register fb_notifier: %d\n", ret);
	}

#elif IS_ENABLED(CONFIG_FB)
	g_the_chip->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&g_the_chip->fb_notif);

	if (ret) {
		TRI_KEY_LOG("Unable to register fb_notifier: %d\n", ret);
	}
#endif/*CONFIG_FB*/

	INIT_WORK(&g_the_chip->speed_up_work, trikey_speedup_resume);
	g_the_chip->speedup_resume_wq = create_singlethread_workqueue("trikey_sp_resume");

	if (!g_the_chip->speedup_resume_wq) {
		ret = -ENOMEM;
	}

	return ret;
}
EXPORT_SYMBOL(oplus_hall_register_notifier);

int oplus_hall_unregister_notifier(void)
{
	int ret = 0;
#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
	if (g_the_chip->active_panel && g_the_chip->fb_notif.notifier_call) {
		ret = drm_panel_notifier_unregister(g_the_chip->active_panel,
			&g_the_chip->fb_notif);
		if (ret) {
			TRI_KEY_LOG("Unable to unregister fb_notifier: %d\n", ret);
		}
	}
#elif IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	if (g_the_chip->active_panel && g_the_chip->notifier_cookie) {
		panel_event_notifier_unregister(g_the_chip->notifier_cookie);
	}
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
	if (g_the_chip->disp_notifier.notifier_call) {
		mtk_disp_notifier_unregister(&g_the_chip->disp_notifier);
	}
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
	if (g_the_chip->fb_notif.notifier_call) {
		ret = msm_drm_unregister_client(&g_the_chip->fb_notif);

		if (ret) {
			TRI_KEY_LOG("Unable to register fb_notifier: %d\n", ret);
		}
	}
#elif IS_ENABLED(CONFIG_FB)
	if (g_the_chip->fb_notif.notifier_call) {
		ret = fb_unregister_client(&g_the_chip->fb_notif);

		if (ret) {
			TRI_KEY_LOG("Unable to unregister fb_notifier: %d\n", ret);
		}
	}
#endif/*CONFIG_FB*/
	if (g_the_chip->speedup_resume_wq) {
		cancel_work_sync(&g_the_chip->speed_up_work);
		flush_workqueue(g_the_chip->speedup_resume_wq);
		destroy_workqueue(g_the_chip->speedup_resume_wq);
	}
	return ret;
}
EXPORT_SYMBOL(oplus_hall_unregister_notifier);

static short Sum(short value0, short value1)
{
	short sum = 0;

	sum = value0 + value1;
	return sum;
}

static short Minus(short value0, short value1)
{
	short minus = 0;

	minus = value0 - value1;
	return minus;
}

void initialCalibValue(short calib_dnHall_UpV, short calib_dnHall_MdV,
			short calib_dnHall_DnV, short calib_upHall_UpV,
			short calib_upHall_MdV, short calib_upHall_DnV)
{
#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
	char payload[1024] = {0x00};
#endif
	calib_upvaluesum = Sum(calib_dnHall_UpV, calib_upHall_UpV);
	calib_mdvaluesum = Sum(calib_dnHall_MdV, calib_upHall_MdV);
	calib_dnvaluesum = Sum(calib_dnHall_DnV, calib_upHall_DnV);
	calib_upvaluemin = Minus(calib_upHall_UpV, calib_dnHall_UpV);
	calib_mdvaluemin = Minus(calib_upHall_MdV, calib_dnHall_MdV);
	calib_dnvaluemin = Minus(calib_upHall_DnV, calib_dnHall_DnV);
	calib_uphall_um_distance = Minus(calib_upHall_UpV, calib_upHall_MdV);
	calib_uphall_md_distance = Minus(calib_upHall_MdV, calib_upHall_DnV);
	calib_dnhall_um_distance = Minus(calib_dnHall_UpV, calib_dnHall_MdV);
	calib_dnhall_md_distance = Minus(calib_dnHall_MdV, calib_dnHall_DnV);
	calib_uphall_ud_distance = Minus(calib_upHall_UpV, calib_upHall_DnV);
	calib_dnhall_ud_distance = Minus(calib_dnHall_UpV, calib_dnHall_DnV);
	up_mid_tol = (short)(abs(calib_upvaluemin - calib_mdvaluemin)
			* 4 / 10);
	up_tolerance = (short)(abs(calib_upvaluemin - calib_mdvaluemin)
			* 11 / 10);
	mid_down_tol = (short)(abs(calib_mdvaluemin - calib_dnvaluemin)
			* 4 / 10);
	down_tolerance = (short)(abs(calib_mdvaluemin -
				calib_dnvaluemin) * 11 / 10);
	up_mid_distance = (short)(abs(calib_upvaluemin -
				calib_mdvaluemin) * 2 / 10);
	mid_down_distance = (short)(abs(calib_mdvaluemin -
				calib_dnvaluemin) * 2 / 10);

#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
	scnprintf(payload, sizeof(payload),
			   "NULL$$EventField@@CalibData$$FieldData@@%d,%d,%d,%d,%d,%d$$detailData@@%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
			   calib_dnHall_UpV, calib_dnHall_MdV, calib_dnHall_DnV, calib_upHall_UpV, calib_upHall_MdV, calib_upHall_DnV,
			   calib_upvaluesum, calib_mdvaluesum, calib_dnvaluesum, calib_upvaluemin, calib_mdvaluemin, calib_dnvaluemin,
			   calib_uphall_um_distance, calib_uphall_md_distance, calib_dnhall_um_distance,
			   calib_dnhall_md_distance, calib_uphall_ud_distance, calib_dnhall_ud_distance,
			   up_mid_tol, up_tolerance, mid_down_tol, down_tolerance, up_mid_distance, mid_down_distance);
	oplus_kevent_fb(FB_TRI_STATE_KEY, TRIKEY_FB_CALIB_DATA_TYPE, payload);
#endif
	TRI_KEY_LOG("Upmin:%d, Mdmin:%d, Dnmin:%d\n",
		calib_upvaluemin, calib_mdvaluemin, calib_dnvaluemin);
	TRI_KEY_LOG("up_mid_tol:%d, mid_down_tol:%d\n",
		up_mid_tol, mid_down_tol);
}

static ssize_t proc_hall_data_read(struct file *file, char __user *user_buf,
			size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[HALL_DATA_NUM] = {0};
	TRI_KEY_LOG("%s:call", __func__);
	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		snprintf(page, HALL_DATA_NUM, "%d\n", -1);
	} else {
		if (g_the_chip->threeaxis_hall_support) {
			oplus_hall_get_data(DHALL_2);
			snprintf(page, HALL_DATA_NUM, "%d, %d, %d, %d \n",
			g_the_chip->hall_value.hall_x, g_the_chip->hall_value.hall_y, g_the_chip->hall_value.hall_z, g_the_chip->hall_value.hall_v);
		} else {
			oplus_hall_get_data(DHALL_0);
			oplus_hall_get_data(DHALL_1);
			snprintf(page, HALL_DATA_NUM, "%d, %d\n",
			g_the_chip->dhall_data0, g_the_chip->dhall_data1);
		}
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static const struct proc_ops proc_hall_data_ops = {
	.proc_read  = proc_hall_data_read,
	.proc_open  = simple_open,
	.proc_lseek	= default_llseek,
};

static ssize_t proc_tri_hall_read(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[6] = {0};
	TRI_KEY_LOG("%s:call", __func__);
	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		snprintf(page, 6, "%d\n", -1);
	} else {
		if (!g_the_chip->threeaxis_hall_support) {
			oplus_hall_get_data(DHALL_0);
			oplus_hall_get_data(DHALL_1);
		} else {
			ret = 1;
		}
		snprintf(page, 6, "%d\n", ret);
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static const struct proc_ops proc_tri_hall_ops = {
	.proc_read  = proc_tri_hall_read,
	.proc_open  = simple_open,
	.proc_lseek	= default_llseek,
};

static ssize_t proc_tri_state_read(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[6] = {0};
	TRI_KEY_LOG("%s:call", __func__);
	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		snprintf(page, 6, "%d\n", -1);
	} else {
		if (!g_the_chip->threeaxis_hall_support) {
			oplus_hall_get_data(DHALL_0);
			oplus_hall_get_data(DHALL_1);
		}
		snprintf(page, 6, "%d\n", g_the_chip->state);
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static const struct proc_ops proc_tri_state_ops = {
	.proc_read  = proc_tri_state_read,
	.proc_open  = simple_open,
	.proc_lseek	= default_llseek,
};


static ssize_t proc_hall_data_calib_read(struct file *file, char __user *user_buf,
			size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[HALL_CALIB_NUM] = {0};
	 TRI_KEY_LOG("proc_hall_data_calib_read\n");
	if (!g_the_chip) {
		 TRI_KEY_LOG("g_the_chip null\n");
		snprintf(page, HALL_CALIB_NUM, "%d\n", -1);
	} else {
		if (g_the_chip->threeaxis_hall_support) {
			TRI_KEY_LOG("g_the_chip->threeaxis_hall_support\n");
			snprintf(page, HALL_CALIB_NUM, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			g_the_chip->threeaxis_calib_data[0], g_the_chip->threeaxis_calib_data[1],
			g_the_chip->threeaxis_calib_data[2], g_the_chip->threeaxis_calib_data[3],
			g_the_chip->threeaxis_calib_data[4], g_the_chip->threeaxis_calib_data[5],
			g_the_chip->threeaxis_calib_data[6], g_the_chip->threeaxis_calib_data[7],
			g_the_chip->threeaxis_calib_data[8]);
		} else {
			snprintf(page, HALL_CALIB_NUM, "%d,%d,%d,%d,%d,%d,%d,%d\n",
			g_the_chip->dnhall_upv, g_the_chip->uphall_upv,
			g_the_chip->dnhall_mdv, g_the_chip->uphall_mdv,
			g_the_chip->dnhall_dnv, g_the_chip->uphall_dnv,
			up_mid_tol, mid_down_tol);
		}
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t proc_hall_data_calib_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *ppos)
{
	int data[9] = {0};
	char temp[HALL_CALIB_NUM] = {0};
	int ret = -1;
	int err = 0;
	#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
	char payload[1024] = {0x00};
	#endif
	TRI_KEY_LOG("%s: call.\n", __func__);
	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		return count;
	}
	if (count >= HALL_CALIB_NUM) {
		return count;
	}
	ret = copy_from_user(temp, buffer, count);
	if (ret) {
		 TRI_KEY_ERR("%s: read proc input error.\n", __func__);
		return count;
	}
	TRI_KEY_LOG("temp is %s:\n", temp);
	if (g_the_chip->threeaxis_hall_support) {
		 TRI_KEY_LOG("threeaxis_hall_support\n");
		if (sscanf(temp, "%d,%d,%d,%d,%d,%d,%d,%d,%d", &data[0], &data[1], &data[2],
			&data[3], &data[4], &data[5], &data[6], &data[7], &data[8]) == 9) {
			g_the_chip->threeaxis_calib_data[0] = data[0];
			g_the_chip->threeaxis_calib_data[1] = data[1];
			g_the_chip->threeaxis_calib_data[2] = data[2];
			g_the_chip->threeaxis_calib_data[3] = data[3];
			g_the_chip->threeaxis_calib_data[4] = data[4];
			g_the_chip->threeaxis_calib_data[5] = data[5];
			g_the_chip->threeaxis_calib_data[6] = data[6];
			g_the_chip->threeaxis_calib_data[7] = data[7];
			g_the_chip->threeaxis_calib_data[8] = data[8];
			TRI_KEY_LOG("data[%d %d %d %d %d %d %d %d %d]\n", data[0], data[1],
					data[2], data[3], data[4], data[5], data[6], data[7], data[8]);

			#if defined(CONFIG_OPLUS_FEATURE_FEEDBACK) || defined(CONFIG_OPLUS_FEATURE_FEEDBACK_MODULE)
				scnprintf(payload, sizeof(payload),
						"NULL$$EventField@@CalibData$$FieldData@@%d,%d,%d,%d,%d,%d,%d,%d,%d",
						g_the_chip->threeaxis_calib_data[0], g_the_chip->threeaxis_calib_data[1], g_the_chip->threeaxis_calib_data[2],
						g_the_chip->threeaxis_calib_data[3], g_the_chip->threeaxis_calib_data[4], g_the_chip->threeaxis_calib_data[5],
						g_the_chip->threeaxis_calib_data[6], g_the_chip->threeaxis_calib_data[7], g_the_chip->threeaxis_calib_data[8]);
				oplus_kevent_fb(FB_TRI_STATE_KEY, TRIKEY_FB_CALIB_DATA_TYPE, payload);
			#endif
			err = threeaxis_get_data(g_the_chip);
			threeaxis_judge_interference(g_the_chip);
			threeaxis_update_position(g_the_chip, 0 , 0, 0);
			report_key_value(g_the_chip);
		} else {
			TRI_KEY_LOG("sscanf fail\n");
		}
	} else {
		if (sscanf(temp, "%d,%d,%d,%d,%d,%d", &data[0], &data[1], &data[2],
			&data[3], &data[4], &data[5]) == 6) {
			g_the_chip->dnhall_upv = data[0]; /* UP   H1 */
			g_the_chip->uphall_upv = data[1]; /* UP   H2 */
			g_the_chip->dnhall_mdv = data[2]; /* MID  H1 */
			g_the_chip->uphall_mdv = data[3]; /* MID  H2 */
			g_the_chip->dnhall_dnv = data[4]; /* DOWN H1 */
			g_the_chip->uphall_dnv = data[5]; /* DOWN H2 */
			TRI_KEY_ERR("data[%d %d %d %d %d %d]\n", data[0], data[1],
					data[2], data[3], data[4], data[5]);
		} else
			TRI_KEY_ERR("fail\n");

			initialCalibValue(g_the_chip->dnhall_upv, g_the_chip->dnhall_mdv,
				g_the_chip->dnhall_dnv, g_the_chip->uphall_upv,
				g_the_chip->uphall_mdv, g_the_chip->uphall_dnv);
			report_calibration_location(g_the_chip);
	}
	return count;
}

static const struct proc_ops proc_hall_data_calib_ops = {
	.proc_write = proc_hall_data_calib_write,
	.proc_read  = proc_hall_data_calib_read,
	.proc_open  = simple_open,
	.proc_lseek	= default_llseek,
};

static ssize_t proc_hall_debug_info_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *ppos)
{
	int tmp = 0;
	char buf[4] = {0};
	int ret = -1;

	if (count > 2)
		return count;
	ret = copy_from_user(buf, buffer, count);
	if (ret) {
		TRI_KEY_ERR("%s: read proc input error.\n", __func__);
		return -EFAULT;
	}
	if (!kstrtoint(buf, 0, &tmp))
		tri_key_debug = tmp;
	else
		TRI_KEY_DEBUG("invalid content: '%s', length = %zd\n",
		buf, count);

	return count;
}

static ssize_t proc_hall_debug_info_read(struct file *file, char __user *user_buf,
			 size_t count, loff_t *ppos)
{
	int ret = -1;
	char page[6] = {0};

	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		snprintf(page, 6, "%d\n", -1);
	} else
		snprintf(page, 6, "%d\n", tri_key_debug);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static const struct proc_ops proc_hall_debug_info_ops = {
	.proc_write = proc_hall_debug_info_write,
	.proc_read  = proc_hall_debug_info_read,
	.proc_open  = simple_open,
	.proc_lseek	= default_llseek,
};

static ssize_t proc_hall_enable_irq_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *ppos)
{
	int tmp = 0;
	char buf[4] = {0};
	int ret = -1;

	if (count > 2)
		return count;
	ret = copy_from_user(buf, buffer, count);
	if (ret) {
		TRI_KEY_ERR("%s: read proc input error.\n", __func__);
		return -EFAULT;
	}

	if (!kstrtoint(buf, 0, &tmp)) {
		oplus_hall_enable_irq(0, tmp);
		oplus_hall_enable_irq(1, tmp);
	} else
		TRI_KEY_DEBUG("invalid content: '%s', length = %zd\n",
		buf, count);

	return count;
}

static const struct proc_ops proc_hall_enable_irq_ops = {
	.proc_write = proc_hall_enable_irq_write,
	.proc_open  = simple_open,
	.proc_lseek	= default_llseek,
};

static ssize_t proc_hall_data_offect_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *ppos)
{
	int tmp = 0;
	char buf[MAX_LEN] = {0};
	int ret = -1;
	char *token  = NULL;
	char *tmp_str = NULL;

	if (count >= MAX_LEN)
		return count;

	mutex_lock(&g_hall_dev->mtx);

	ret = copy_from_user(buf, buffer, count);
	if (ret) {
		TRI_KEY_ERR("%s: read proc input error.\n", __func__);
		return -EFAULT;
	}

	if (strstr(buf, OFFECT_UP) != NULL) {
		TRI_KEY_ERR("%s:detect %s", __func__, OFFECT_UP);
		tmp_str = buf;
		token = strsep(&tmp_str, OFFECT_CUT);
		token = strsep(&tmp_str, OFFECT_CUT);
		strcpy(buf, token);
		if (!kstrtoint(buf, 0, &tmp)) {
			if (tmp > OFFECT_MAX) {
				TRI_KEY_ERR("%s:illegal data!! not to write[%d]", __func__, tmp);
				goto OUT;
			}
			g_the_chip->data_offect = tmp;
			TRI_KEY_ERR("%s: write->data_offect:%d\n", __func__, g_the_chip->data_offect);
			ret = g_the_chip->dhall_up_ops->offect_data_handle(g_the_chip->data_offect);
		}
	}

OUT:
	mutex_unlock(&g_hall_dev->mtx);
	return count;
}

static ssize_t proc_hall_data_offect_read(struct file *file, char __user *user_buf,
			 size_t count, loff_t *ppos)
{
	int ret = -1;
	char page[MAX_LEN] = {0};

	mutex_lock(&g_hall_dev->mtx);

	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		snprintf(page, MAX_LEN, "%d\n", -1);
	} else {
		if (!strcmp(g_hall_dev->data_offect_name, OFFECT_UP)) {
			TRI_KEY_ERR("now get up offect data\n");
			ret = g_the_chip->dhall_up_ops->offect_data_handle(READ_OFFECT);
			snprintf(page, MAX_LEN, "up: offect:%d\n", ret);
		}
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	mutex_unlock(&g_hall_dev->mtx);
	return ret;
}

static const struct proc_ops proc_hall_data_offect_ops = {
	.proc_write = proc_hall_data_offect_write,
	.proc_read  = proc_hall_data_offect_read,
	.proc_open  = simple_open,
	.proc_lseek = default_llseek,
};

static int proc_hall_dump_register_read_func(struct seq_file *s, void *v)
{

	if (g_the_chip->threeaxis_hall_support) {
		seq_printf(s, "-----------DUMP HALL2 START-----------\n");
		seq_printf(s, "HALL2[\n");
		oplus_hall_dump_regs(DHALL_2, s);
		seq_printf(s, "]\n");
		msleep(100);
		seq_printf(s, "-----------DUMP HALL2 END-------------\n");
	} else {
		seq_printf(s, "-----------DUMP HALL0 START-----------\n");
		seq_printf(s, "HALL0[\n");
		oplus_hall_dump_regs(DHALL_0, s);
		seq_printf(s, "]\n");
		msleep(100);
		seq_printf(s, "-----------DUMP HALL0 END-------------\n");
		seq_printf(s, "-----------DUMP HALL1 START-----------\n");
		seq_printf(s, "HALL1[\n");
		oplus_hall_dump_regs(DHALL_1, s);
		seq_printf(s, "]\n");
		msleep(100);
		seq_printf(s, "-----------DUMP HALL1 END-------------\n");
	}
	return 0;
}

static int dump_register_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_hall_dump_register_read_func, PDE_DATA(inode));
}

CREATE_PROC_OPS(proc_hall_dump_register_read_ops, dump_register_open, seq_read, NULL, single_release);

static int init_trikey_proc(struct extcon_dev_data *hall_dev)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_trikey = NULL;
	struct proc_dir_entry *prEntry_tmp = NULL;

	TRI_KEY_LOG("%s start\n", __func__);
	prEntry_trikey = proc_mkdir("tristatekey", NULL);
	if (prEntry_trikey == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create trikey proc entry\n", __func__);
	}

	prEntry_tmp = proc_create("hall_data", 0644, prEntry_trikey, &proc_hall_data_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	prEntry_tmp = proc_create("tri_state", 0644, prEntry_trikey, &proc_tri_state_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	prEntry_tmp = proc_create("hall_data_calib", 0666, prEntry_trikey,
			&proc_hall_data_calib_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	prEntry_tmp = proc_create("hall_debug_info", 0666, prEntry_trikey,
			&proc_hall_debug_info_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	prEntry_tmp = proc_create("hall_enable_irq", 0666, prEntry_trikey,
			&proc_hall_enable_irq_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}
	if (g_the_chip->threeaxis_hall_support) {
		TRI_KEY_ERR("%s: detect threeaxis_hall_support create proc\n", __func__);
		prEntry_tmp = proc_create("tri_hall", 0666, prEntry_trikey,
			&proc_tri_hall_ops);
		if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
		}
	}

	TRI_KEY_ERR("%s: offect:%d.\n", __func__, is_has_data_offect);

	if (is_has_data_offect) {
		TRI_KEY_ERR("%s: detect %s offect and create proc\n", __func__, g_hall_dev->data_offect_name);
		prEntry_tmp = proc_create("data_offect", 0666, prEntry_trikey,
				&proc_hall_data_offect_ops);
		if (prEntry_tmp == NULL) {
			ret = -ENOMEM;
			TRI_KEY_ERR("%s: Couldn't create data_offect proc, %d\n", __func__, __LINE__);
		}
	}

	TRI_KEY_ERR("%s: open dump register\n", __func__);
	prEntry_tmp = proc_create("dump_register", 0666, prEntry_trikey,
		&proc_hall_dump_register_read_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create dump_register proc, %d\n", __func__, __LINE__);
	}

	return ret;
}

static void register_tri_key_dev_work(struct work_struct *work)
{
	struct extcon_dev_data *chip = container_of(work, struct extcon_dev_data,
			register_work);
	struct extcon_dev_data *pdev;
	int err = 0;
	int res = 0;

	TRI_KEY_LOG("call %s\n", __func__);

	if (!g_the_chip) {
		chip = kzalloc(sizeof(struct extcon_dev_data), GFP_KERNEL);
		if (!chip) {
			TRI_KEY_ERR("kzalloc err\n");
			return;
		}
		g_the_chip = chip;
	} else {
		chip = g_the_chip;
	}
	pdev = g_hall_dev;
	chip->input_dev = pdev->input_dev;
	chip->input_dev = input_allocate_device();
	if (chip->input_dev == NULL) {
		res = -ENOMEM;
		TRI_KEY_ERR("Failed to allocate input device\n");
		goto fail;
	}
	chip->input_dev->name = TRI_KEY_DEVICE;

	set_bit(EV_SYN, chip->input_dev->evbit);
	set_bit(EV_KEY, chip->input_dev->evbit);
	set_bit(KEY_F3, chip->input_dev->keybit);
	res = input_register_device(chip->input_dev);
	if (res) {
		TRI_KEY_ERR("%s: Failed to register input device\n", __func__);
		input_free_device(chip->input_dev);
		goto fail;
	}
	res = init_trikey_proc(pdev);
	if (res < 0) {
		TRI_KEY_ERR("create trikey proc fail\n");
		goto fail;
	}

	if (0 && (!chip->dhall_up_ops || !chip->dhall_down_ops)) {
		TRI_KEY_ERR("no dhall available\n");
		goto fail;
	}



	INIT_WORK(&chip->dwork, tri_key_dev_work);
	hrtimer_init(&tri_key_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tri_key_timer.function = tri_key_status_timeout;
	INIT_WORK(&tri_key_timeout_work, tri_key_timeout_work_func);
/*get data when reboot*/
	if (g_the_chip->threeaxis_hall_support) {
		res = threeaxis_get_data(chip);
		if (res < 0) {
		TRI_KEY_LOG("tri_key:get hall data failed!\n");
		goto fail;
		}
		threeaxis_judge_interference(g_the_chip);
		err = threeaxis_get_position(g_the_chip);
		err = oplus_hall_set_detection_mode(DHALL_2,
			DETECTION_MODE_INTERRUPT);
		TRI_KEY_LOG("%s three_axis_hall probe success.\n", __func__);
		return;
	}
	res = oplus_get_data(chip);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get hall data failed!\n");
		goto fail;
	}
	TRI_KEY_LOG("tri_key:data1 is %d, data0 is %d\n",
				chip->dhall_data1, chip->dhall_data0);

/*get position when reboot*/
	reboot_get_position(chip);
/*set threshold when reboot;*/
	err = reupdata_threshold(chip);
	if (err < 1) {
		TRI_KEY_ERR("%s reupdata_threshold failed\n", __func__);
		goto fail;
	}
/*report key value*/
	report_key_value(chip);
	last_position = chip->position;
	err = oplus_hall_set_detection_mode(DHALL_0,
			DETECTION_MODE_INTERRUPT);
	TRI_KEY_LOG("tri_key:set 0 detection mode\n");
	if (err < 0) {
		TRI_KEY_ERR("%s set HALL0 detection mode failed %d\n",
			__func__, err);
		goto fail;
	}
	err = oplus_hall_set_detection_mode(DHALL_1,
			DETECTION_MODE_INTERRUPT);

	TRI_KEY_LOG("tri_key:set 1 detection mode\n");
	if (err < 0) {
		TRI_KEY_ERR("%s set HALL1 detection mode failed %d\n",
			__func__, err);
		goto fail;
	}
	TRI_KEY_LOG("%s probe success.\n", __func__);
	return;

fail:
	kfree(chip);
	g_the_chip = NULL;
	TRI_KEY_LOG("fail\n");
}

static int init_parse_dts(struct device *dev, struct extcon_dev_data *g_the_chip) {
	struct device_node *np = NULL;
	int ret = 0;
	int temp_array[8];
	int tolen[3] = {0};
	TRI_KEY_LOG(" %s call\n", __func__);
	np = dev->of_node;
	if (!np) {
		TRI_KEY_LOG(" %s dts node is NULL\n", __func__);
		g_the_chip->threeaxis_hall_support = false;
		g_the_chip->new_threshold_support = false;
		return -1;
	}

	g_the_chip->new_threshold_support = of_property_read_bool(np, "new_threshold_support");
	TRI_KEY_LOG("%s:new_up_threshold_support:%d\n", __func__, g_the_chip->new_threshold_support);

	g_the_chip->updown_to_mid_support = of_property_read_bool(np, "updown-to-mid-support");
	TRI_KEY_LOG("%s:updown_to_mid_support:%d\n", __func__, g_the_chip->updown_to_mid_support);

	ret = of_property_read_u32_array(np, "tolen", tolen, 3);
	if (ret) {
		TRI_KEY_LOG("tolen use default\n");
		tolen[0] = 0;
		tolen[1] = 0;
		tolen[2] = 0;
		ret = 0;
	} else {
		TRI_KEY_LOG("get tolen:<0-2>{%d,%d,%d.}\n", tolen[0], tolen[1], tolen[2]);
	}
	g_the_chip->tolen[0] = tolen[0];
	g_the_chip->tolen[1] = tolen[1];
	g_the_chip->tolen[2] = tolen[2];

	g_the_chip->threeaxis_hall_support = of_property_read_bool(np, "threeaxis_hall_support");

	if (g_the_chip->threeaxis_hall_support) {
		/* [x y z]  use to judege whether the interfence is stable */
		ret = of_property_read_u32_array(np, "interf_stable_limit", temp_array, 3);
		if (ret) {
			TRI_KEY_LOG(" %s interf_stable_limit is not sepecifit! using default\n", __func__);
			g_the_chip->interf_stable_xlimit = 700;
			g_the_chip->interf_stable_ylimit = 700;
			g_the_chip->interf_stable_zlimit = 700;
		} else {
			g_the_chip->interf_stable_xlimit = temp_array[0];
			g_the_chip->interf_stable_ylimit = temp_array[1];
			g_the_chip->interf_stable_zlimit = temp_array[2];
		}

		/* [|x|+|y|(sum)  z]  use to judege whether the interfence is exist */
		ret = of_property_read_u32_array(np, "interf_judege_limit", temp_array, 2);
		if (ret) {
			TRI_KEY_LOG(" %s interf_judege_limit is not sepecifit! using default\n", __func__);
			g_the_chip->interf_exist_sumlimit = 4000;
			g_the_chip->interf_exist_zlimit = 2000;
		} else {
			g_the_chip->interf_exist_sumlimit = temp_array[0];
			g_the_chip->interf_exist_zlimit = temp_array[1];
		}

		/* [x y x y]  use to judege the up position change in the interfence */
		ret = of_property_read_u32_array(np, "interf_up_tolen", temp_array, 4);
		if (ret) {
			TRI_KEY_LOG(" %s interf_up_tolen is not sepecifit! using default\n", __func__);
			g_the_chip->up_mid_tolen[0] = 3700;
			g_the_chip->up_mid_tolen[1] = 3700;
			g_the_chip->up_down_tolen[0] = 3000;
			g_the_chip->up_down_tolen[1] = 3000;
		} else {
			g_the_chip->up_mid_tolen[0] = temp_array[0];
			g_the_chip->up_mid_tolen[1] = temp_array[1];
			g_the_chip->up_down_tolen[0] = temp_array[2];
			g_the_chip->up_down_tolen[1] = temp_array[3];
			TRI_KEY_LOG(" %s interf_up_tolen using [%d %d %d %d]\n", __func__, temp_array[0], temp_array[1], temp_array[2], temp_array[3]);
		}

		/* [x y x y]  use to judege the mid position change in the interfence */
		ret = of_property_read_u32_array(np, "interf_mid_tolen", temp_array, 4);
		if (ret) {
			TRI_KEY_LOG(" %s interf_mid_tolen is not sepecifit! using default\n", __func__);
			g_the_chip->mid_up_tolen[0] = 3000;
			g_the_chip->mid_up_tolen[1] = 3000;
			g_the_chip->mid_down_tolen[0] = 3000;
			g_the_chip->mid_down_tolen[1] = 3000;
		} else {
			g_the_chip->mid_up_tolen[0] = temp_array[0];
			g_the_chip->mid_up_tolen[1] = temp_array[1];
			g_the_chip->mid_down_tolen[0] = temp_array[2];
			g_the_chip->mid_down_tolen[1] = temp_array[3];
			TRI_KEY_LOG(" %s interf_mid_tolen using [%d %d %d %d]\n", __func__, temp_array[0], temp_array[1], temp_array[2], temp_array[3]);
		}

		/* [x y x y]  use to judege the down position change in the interfence */
		ret = of_property_read_u32_array(np, "interf_down_tolen", temp_array, 4);
		if (ret) {
			TRI_KEY_LOG(" %s interf_down_tolen is not sepecifit! using default\n", __func__);
			g_the_chip->down_up_tolen[0] = 3000;
			g_the_chip->down_up_tolen[1] = 3000;
			g_the_chip->down_mid_tolen[0] = 3700;
			g_the_chip->down_mid_tolen[1] = 3700;
		} else {
			g_the_chip->down_up_tolen[0] = temp_array[0];
			g_the_chip->down_up_tolen[1] = temp_array[1];
			g_the_chip->down_mid_tolen[0] = temp_array[2];
			g_the_chip->down_mid_tolen[1] = temp_array[3];
			TRI_KEY_LOG(" %s interf_down_tolen using [%d %d %d %d]\n", __func__, temp_array[0], temp_array[1], temp_array[2], temp_array[3]);
		}

		/* [x y]  use to judege the nomal position */
		ret = of_property_read_u32_array(np, "position_judge_tolen", temp_array, 2);
		if (ret) {
			TRI_KEY_LOG(" %s position_judge_tolen is not sepecifit! using default\n", __func__);
			g_the_chip->position_tolen[0] = 4000;
			g_the_chip->position_tolen[1] = 5000;
		} else {
			g_the_chip->position_tolen[0] = temp_array[0];
			g_the_chip->position_tolen[1] = temp_array[1];
			TRI_KEY_LOG(" %s position_judge_tolen using [%d %d]\n", __func__, temp_array[0], temp_array[1]);
		}

		/* [x x]  use to judege the nomal position without calib */
		ret = of_property_read_u32_array(np, "default_position_xtolen", temp_array, 2);
		if (ret) {
			TRI_KEY_LOG(" %s default_position_xtolen is not sepecifit! using default\n", __func__);
			g_the_chip->default_up_xdata = 3000;
			g_the_chip->default_down_xdata = 3000;
		} else {
			g_the_chip->default_up_xdata = temp_array[0];
			g_the_chip->default_down_xdata = temp_array[1];
			TRI_KEY_LOG(" %s default_position_xtolen using [%d %d]\n", __func__, temp_array[0], temp_array[1]);
		}
	}

	return ret;
}


int oplus_register_hall(const char *name, struct dhall_operations *ops,
		struct extcon_dev_data *hall_dev_t)
{
	static int hall_count;
	struct extcon_dev_data *hall_dev = hall_dev_t;
	int ret = 0;
	if (!name || !ops) {
		TRI_KEY_ERR("name is NULL or ops is NULL, would not register digital hall\n");
		return -EINVAL;
	}

	mutex_lock(&tri_key_mutex);
	if (!g_the_chip) {
		struct extcon_dev_data *chip = kzalloc(sizeof(struct
				extcon_dev_data), GFP_KERNEL);
		if (!chip) {
			TRI_KEY_ERR("kzalloc err\n");
			mutex_unlock(&tri_key_mutex);
			return -ENOMEM;
		}
		g_the_chip = chip;
		g_hall_dev = hall_dev;
	}

	TRI_KEY_LOG("name : %s\n", name);
	if (strcmp(name, "three_axis_hall") == 0) {
		TRI_KEY_LOG("name == three_axis_hall");
		if (!g_the_chip->threeaxis_dhall_ops) {
			if (ops) {
				g_the_chip->threeaxis_dhall_ops = ops;
				g_the_chip->d_name = name;
				hall_count++;
			} else {
				TRI_KEY_ERR("three_axis_hall_ops NULL\n");
				mutex_unlock(&tri_key_mutex);
				return -EINVAL;
			}
		} else {
			TRI_KEY_ERR("three_axis_hall_ops has been register\n");
			mutex_unlock(&tri_key_mutex);
			return -EINVAL;
		}
	}
	if (strcmp(name, "hall_down") == 0) {
		TRI_KEY_LOG("name == hall_down");
		if (!g_the_chip->dhall_down_ops) {
			if (ops) {
				g_the_chip->dhall_down_ops = ops;
				g_the_chip->d_name = name;
				hall_count++;
			} else {
				TRI_KEY_ERR("dhall_down_ops NULL\n");
				mutex_unlock(&tri_key_mutex);
				return -EINVAL;
			}
		} else {
			TRI_KEY_ERR("dhall_down_ops has been register\n");
			mutex_unlock(&tri_key_mutex);
			return -EINVAL;
		}
	}
	if (strcmp(name, "hall_up") == 0) {
		TRI_KEY_LOG("name == hall_up");
		if (!g_the_chip->dhall_up_ops) {
			if (ops) {
				g_the_chip->dhall_up_ops = ops;
				g_the_chip->d_name = name;
				hall_count++;
			}  else {
				TRI_KEY_ERR("dhall_up_ops NULL\n");
				mutex_unlock(&tri_key_mutex);
				return -EINVAL;
			}
		} else {
			TRI_KEY_ERR("dhall_up_ops has been register\n");
			mutex_unlock(&tri_key_mutex);
			return -EINVAL;
		}
	}

	TRI_KEY_LOG("%s: offect:%s.\n", __func__, g_hall_dev->data_offect_name);
	if (!strncmp(g_hall_dev->data_offect_name, OFFECT_UP, strlen(OFFECT_UP)) ||
		!strncmp(g_hall_dev->data_offect_name, OFFECT_DOWN, strlen(OFFECT_DOWN))) {
		is_has_data_offect = true;
		if (g_hall_dev->enable_esd_check == true) {
			TRI_KEY_LOG("%s: start esd check\n", __func__);
			is_has_esd_check = true;
		}
	}
	ret = init_parse_dts(g_hall_dev->dev, g_the_chip);
	if (ret < 0) {
		TRI_KEY_ERR("%s : dts init failed!\n", __func__);
		mutex_unlock(&tri_key_mutex);
		return -1;
	}
	if (hall_count > 1 || g_the_chip->threeaxis_hall_support) {
		INIT_WORK(&g_the_chip->register_work, register_tri_key_dev_work);
		schedule_work(&g_the_chip->register_work);
	}
	TRI_KEY_LOG("name : %s success\n", name);
	mutex_unlock(&tri_key_mutex);
	return 0;
}
EXPORT_SYMBOL(oplus_register_hall);
MODULE_LICENSE("GPL");


