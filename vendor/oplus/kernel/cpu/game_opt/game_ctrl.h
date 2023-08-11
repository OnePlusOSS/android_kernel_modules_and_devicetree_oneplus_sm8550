// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#ifndef __GAME_CTRL_H__
#define __GAME_CTRL_H__

#include <linux/sched.h>
#include "../../../kernel/sched/sched.h"

#define MAX_TID_COUNT 256
#define MAX_TASK_NR 15
#define RESULT_PAGE_SIZE 1024

extern struct proc_dir_entry *game_opt_dir;

int cpu_load_init(void);
int cpufreq_limits_init(void);
int task_util_init(void);
int rt_info_init(void);
int dstate_dump_init(void);

#endif /*__GAME_CTRL_H__*/
