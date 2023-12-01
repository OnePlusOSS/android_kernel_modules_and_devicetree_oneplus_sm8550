/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#ifndef _OPLUS_SA_PRIORITY_H_
#define _OPLUS_SA_PRIORITY_H_

#include "linux/sched.h"

#define ENABLE_PRESET_VRUNTIME 1

#define NICE_0_LOAD            (1L << NICE_0_LOAD_SHIFT)
#define PRIORITY_LEVEL_NUM     11
#define PRIO_EXEC_GAP          4000000ULL  /* 4ms */
#define NICE_EXEC_GAP           500000ULL  /* 0.5ms */
#define CFS_SCHED_MIN_GRAN      750000ULL
#define CFS_SCHED_NR_LATENCY            8
#define CFS_SCHED_LATENCY      6000000ULL
#define CFS_WAKEUP_GRAN        2000000ULL

extern const int ux_prio_to_weight[PRIORITY_LEVEL_NUM];

int ux_state_to_priority(int ux_state);
int ux_state_to_nice(int ux_state);

void initial_prio_nice_and_vruntime(struct oplus_rq *orq, struct oplus_task_struct *ots, int ux_prio, int ux_nice);
void update_vruntime_task_detach(struct oplus_rq *orq, struct oplus_task_struct *ots);
void update_vruntime_task_attach(struct oplus_rq *orq, struct oplus_task_struct *ots);
void insert_task_to_ux_timeline(struct oplus_task_struct *ots, struct oplus_rq *orq);
void update_ux_timeline_task_change(struct oplus_rq *orq, struct oplus_task_struct *ots, int new_prio, int new_nice);
void update_ux_timeline_task_tick(struct oplus_rq *orq, struct oplus_task_struct *ots);
void update_ux_timeline_task_removal(struct oplus_rq *orq, struct oplus_task_struct *ots);
bool need_resched_ux(struct oplus_rq *orq, struct oplus_task_struct *curr, unsigned long delta_exec);
bool need_wakeup_preempt(struct oplus_rq *orq, struct oplus_task_struct *curr);

void android_vh_sched_stat_runtime_handler(void *unused, struct task_struct *tsk, u64 runtime, u64 vruntime);
#endif /* _OPLUS_SA_PRIORITY_H_ */
