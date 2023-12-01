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

extern atomic_t need_stat_util;
extern atomic_t need_stat_wake;

int cpu_load_init(void);
int cpufreq_limits_init(void);
int task_util_init(void);
int rt_info_init(void);
int fake_cpufreq_init(void);
void try_to_wake_up_success_hook2(struct task_struct *task);

static inline u32 task_util(struct task_struct *p)
{
	return READ_ONCE(p->se.avg.util_avg);
}

static inline bool need_stat_cpu_load(void)
{
	return (atomic_read(&need_stat_util) > 0) ||
		(atomic_read(&need_stat_wake) > 0);
}

#endif /*__GAME_CTRL_H__*/
