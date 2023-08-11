// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>

#include "frame_boost.h"
#include "frame_group.h"
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#endif

struct fbg_vendor_hook fbg_hook;
int stune_boost[BOOST_MAX_TYPE];
EXPORT_SYMBOL_GPL(stune_boost);

static inline bool vaild_stune_boost_type(unsigned int type)
{
	return (type >= 0) && (type < BOOST_MAX_TYPE);
}

void fbg_set_stune_boost(int value, unsigned int type)
{
	if (!vaild_stune_boost_type(type))
		return;

	stune_boost[type] = value;
}
EXPORT_SYMBOL_GPL(fbg_set_stune_boost);

int fbg_get_stune_boost(unsigned int type)
{
	if (!vaild_stune_boost_type(type))
		return 0;

	return stune_boost[type];
}
EXPORT_SYMBOL_GPL(fbg_get_stune_boost);

void fbg_update_ed_task_duration(unsigned int type)
{
	switch (type) {
	case BOOST_ED_TASK_MID_DURATION:
		update_ed_task_boost_mid_duration(stune_boost[BOOST_ED_TASK_MID_DURATION]);
		break;
	case BOOST_ED_TASK_MAX_DURATION:
		update_ed_task_boost_max_duration(stune_boost[BOOST_ED_TASK_MAX_DURATION]);
		break;
	case BOOST_ED_TASK_TIME_OUT_DURATION:
		update_ed_task_boost_timeout_duration(stune_boost[BOOST_ED_TASK_TIME_OUT_DURATION]);
		break;
	}
}
EXPORT_SYMBOL_GPL(fbg_update_ed_task_duration);

/******************
 * moduler function
 *******************/
static int __init oplus_frame_boost_init(void)
{
	int ret = 0;

	fbg_sysctl_init();

	ret = frame_info_init();
	if (ret != 0)
		goto out;

	ret = frame_group_init();
	if (ret != 0)
		goto out;

	/* please register your hooks at the end of init. */
	register_frame_group_vendor_hooks();

	fbg_migrate_task_callback = fbg_skip_migration;
	fbg_android_rvh_schedule_callback = fbg_android_rvh_schedule_handler;
	ofb_debug("oplus_bsp_frame_boost.ko init succeed!!\n");

	stune_boost[BOOST_ED_TASK_MID_DURATION] = 60; /* default 0.6*window */
	stune_boost[BOOST_ED_TASK_MAX_DURATION] = 80; /* default 0.8*window */
	stune_boost[BOOST_ED_TASK_TIME_OUT_DURATION] = 200; /* default 2*window */
	stune_boost[BOOST_ED_TASK_MID_UTIL] = 600; /* default mid util */
	stune_boost[BOOST_ED_TASK_MAX_UTIL] = 900; /* default max util */

out:
	return ret;
}

module_init(oplus_frame_boost_init);
MODULE_DESCRIPTION("Oplus Frame Boost Moduler");
MODULE_LICENSE("GPL v2");
