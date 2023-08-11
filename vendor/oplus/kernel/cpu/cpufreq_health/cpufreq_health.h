/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2021 Oplus. All rights reserved.
 */


#ifndef _CPUFREQ_HEALTH_H
#define _CPUFREQ_HEALTH_H
#include <linux/kernel_stat.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/cpu.h>
#include <linux/types.h>
#include <linux/slab.h>

struct oplus_cpufreq_frequency_table {
	unsigned int frequency;
	u64 floor_time;
	u64 ceiling_time;
	s64 floor_count;
	s64 ceiling_count;
	unsigned int voltage;
};

#if IS_ENABLED(CONFIG_SCHED_WALT)
struct oplus_cpu_edtask_table {
	int edtask_state;
};
#endif

#if IS_ENABLED(CONFIG_MTK_GEARLESS_SUPPORT)
struct oplus_leakage_info {
	unsigned int a;
	unsigned int b;
	unsigned int c;
	unsigned int power;
};
#endif

struct oplus_cpufreq_health {
	struct oplus_cpufreq_frequency_table *table;
	unsigned int prev_floor_freq;
	unsigned int curr_floor_freq;
	unsigned int prev_ceiling_freq;
	unsigned int curr_ceiling_freq;
	u64 last_update_time;
	u64 bootup_time;
	unsigned int len;
	unsigned int cpu_count;
	spinlock_t lock;
	struct kobject kobj;
	int is_init;
#if IS_ENABLED(CONFIG_SCHED_WALT)
	struct oplus_cpu_edtask_table *edtask_table;
	u64 newtask_boost_time;
	u64 edtask_boost_time;
	u64 last_update_util_time;
	u64 last_update_edtask_time;
	int pre_newtask_state;
	int pre_edtask_state;
	s64 newtask_count;
	s64 edtask_count;
#endif
#if IS_ENABLED(CONFIG_MTK_GEARLESS_SUPPORT)
	struct oplus_leakage_info leakage_info;
#endif
};

/* add for midas data collection */
struct cpufreq_health_info {
	u64 ceiling_time;
	u64 ceiling_count;
	u64 floor_time;
	u64 floor_count;
};

extern void cpufreq_health_get_state(struct cpufreq_policy *policy);
extern int freq_to_voltage(int cluster_id, unsigned int target_freq);

#if IS_ENABLED(CONFIG_SCHED_WALT)
extern void cpufreq_health_get_newtask_state(struct cpufreq_policy *policy, int newtask_flag);
extern void cpufreq_health_get_edtask_state(int cpu, int edtask_flag);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OCH)
void get_cpufreq_health_info(int *cnt, struct cpufreq_health_info *val);
extern int cpufreq_health_register(struct cpufreq_policy *policy);
#else
static inline void get_cpufreq_health_info(int *cnt, struct cpufreq_health_info *val) {}
extern int cpufreq_health_register(struct cpufreq_policy *policy) { return 0; }
#endif

#endif /* _CPUFREQ_HEALTH_H */
