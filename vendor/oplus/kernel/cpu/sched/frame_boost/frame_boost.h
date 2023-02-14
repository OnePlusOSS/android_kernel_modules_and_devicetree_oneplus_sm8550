/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#ifndef _FRAME_BOOST_H_
#define _FRAME_BOOST_H_

#include "frame_info.h"
#include "frame_group.h"
#include "cluster_boost.h"

#define ofb_debug(fmt, ...) \
	pr_info("[frame boost][%s]"fmt, __func__, ##__VA_ARGS__)

#define ofb_err(fmt, ...) \
	pr_err("[frame boost][%s]"fmt, __func__, ##__VA_ARGS__)

#define SLIDE_SCENE    (1 << 0)
#define INPUT_SCENE    (1 << 1)

enum STUNE_BOOST_TYPE {
	BOOST_DEF_MIGR = 0,
	BOOST_DEF_FREQ,
	BOOST_SF_MIGR,
	BOOST_SF_FREQ,
	BOOST_MAX_TYPE,
};

struct fbg_vendor_hook {
	void (*update_freq)(struct rq *rq, unsigned int flags);
};

extern int sysctl_frame_boost_enable;
extern int sysctl_frame_boost_debug;
extern int sysctl_slide_boost_enabled;
extern int sysctl_input_boost_enabled;
extern int stune_boost[BOOST_MAX_TYPE];
extern struct fbg_vendor_hook fbg_hook;
extern unsigned int ed_task_boost_mid_duration_scale;
extern unsigned int ed_task_boost_max_duration_scale;
extern unsigned int ed_task_boost_timeout_duration_scale;

void fbg_set_stune_boost(int value, unsigned int type);
int fbg_get_stune_boost(unsigned int type);
void fbg_sysctl_init(void);
#endif /* _FRAME_BOOST_H_ */
