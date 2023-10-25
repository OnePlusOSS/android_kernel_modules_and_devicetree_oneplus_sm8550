// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#ifndef _FAKE_CAP_H
#define _FAKE_CAP_H

#ifdef CONFIG_FAIR_GROUP_SCHED
/* An entity is a task if it doesn't "own" a runqueue */
#define oplus_entity_is_task(se)	(!se->my_q)
#else
#define oplus_entity_is_task(se)	(1)
#endif

#define TOPAPP 4
#define BGAPP 3
#define DEFAULTAPP 1
#define FGAPP 2
#define NULLAPP 0

#define AD_TOP 0
#define AD_FG 1
#define AD_BG 2
#define AD_DF 3
#define AD_MAX_GROUP 4
#define AD_TASK_UTIL_INDEX (2*AD_MAX_GROUP)
#define AD_VTIME_SLICE_INDEX (2*AD_MAX_GROUP+1)

#define FAKE_CLUSTERS 5
#define FAKE_CPUS 8
#define EAS_GOLD_PLUS_CAPACITY 1024

struct group_adjust_param {
	int util_compensate;
	unsigned long vtime_compensate;
};

struct groups_adjust {
	int task_compensate;
	u64 adjust_std_vtime_slice;
	struct group_adjust_param group_param[AD_MAX_GROUP];
};

extern int fake_cap_multiple[FAKE_CLUSTERS];
extern int nr_fake_cap_multiple[FAKE_CLUSTERS];
extern int util_thresh_percent[FAKE_CLUSTERS];
extern int util_thresh_cvt[FAKE_CLUSTERS];
extern int back_cap[FAKE_CLUSTERS];
extern unsigned long real_cpu_cap[FAKE_CPUS];
extern struct groups_adjust group_adjust;

extern int eas_opt_enable;
extern int eas_opt_debug_enable;
extern int sa_adjust_group_enable;
extern struct groups_adjust group_adjust;
extern void fake_cap_systrace_c(unsigned int cpu, unsigned long cap_orig, unsigned long cap);
extern int get_grp_adinfo(struct task_struct *p);
extern bool adjust_group_task(struct task_struct *p, int cpu);

struct proc_dir_entry;
extern int fake_cap_init(struct proc_dir_entry *dir);
extern void fake_cap_proc_remove(struct proc_dir_entry *dir);

#endif /* _FAKE_CAP_H */
