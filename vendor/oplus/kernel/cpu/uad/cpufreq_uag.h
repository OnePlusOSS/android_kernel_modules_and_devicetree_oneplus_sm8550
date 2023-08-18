/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#ifndef _CPUFREQ_UAG_H_
#define _CPUFREQ_UAG_H_
#include <linux/cpufreq_health.h>

#ifdef CONFIG_OPLUS_MULTI_LV_TL
enum ua_util_type {
	UA_UTIL_SYS,
	UA_UTIL_FBG,
	UA_UTIL_SIZE,
};

struct multi_target_loads {
	unsigned int	*target_loads;
	unsigned int	*util_loads;
	int		ntarget_loads;
};
#endif

struct uag_gov_tunables {
	struct gov_attr_set		attr_set;
	unsigned int			hispeed_load;
	unsigned int			hispeed_freq;
	bool				cobuck_enable;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
#ifdef CONFIG_OPLUS_UAG_USE_TL
	spinlock_t			target_loads_lock;
	unsigned int			*target_loads;
	unsigned int			*util_loads;
	int				ntarget_loads;
#endif

#ifdef CONFIG_OPLUS_UAG_AMU_AWARE
	bool				stall_aware;
	int				reduce_pct_of_stall;
	int				max_stall_reduce_of_util;
	int				report_policy;
#endif

#ifdef CONFIG_OPLUS_MULTI_LV_TL
	bool				multi_tl_enable;
	struct multi_target_loads	multi_tl[UA_UTIL_SIZE];
#endif

#ifdef CONFIG_OPLUS_UAG_SOFT_LIMIT
	spinlock_t			soft_limit_freq_lock;
	unsigned long			soft_limit_freq;
	unsigned long			soft_limit_util;
	bool				soft_limit_enable;
	unsigned int			break_freq_margin;
#endif
};

struct uag_gov_policy {
	struct cpufreq_policy	*policy;
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	u64			last_ws;
	u64			curr_cycles;
	u64			last_cyc_update_time;
	unsigned long		avg_cap;
#endif /* CONFIG_OPLUS_SYSTEM_KERNEL_QCOM */

	struct uag_gov_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;	/* For shared policies */
	u64			last_freq_update_time;
	s64			min_rate_limit_ns;
	s64			up_rate_delay_ns;
	s64			down_rate_delay_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;
	unsigned long		hispeed_util;
#ifdef CONFIG_OPLUS_UAG_USE_TL
	unsigned int		len;
#endif

	/* The next fields are only needed if fast switch cannot be used: */
	struct irq_work		irq_work;
	struct kthread_work	work;
	struct mutex		work_lock;
	struct kthread_worker	worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;
	bool			cobuck_boosted;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	unsigned int		flags;
#endif
#ifdef CONFIG_OPLUS_MULTI_LV_TL
	unsigned long		multi_util[UA_UTIL_SIZE];
	unsigned int		multi_util_type;
	unsigned int		fbg_gt_sys_cnt;
	unsigned int		total_cnt;
#endif
};

#define MAX_CLUSTERS 3
static int init_flag[MAX_CLUSTERS];

#ifdef CONFIG_ARCH_MEDIATEK
extern u64 curr_frame_start;
extern u64 prev_frame_start;
#endif /* CONFIG_ARCH_MEDIATEK */

#ifdef CONFIG_UAG_NONLINEAR_FREQ_CTL
int init_opp_cap_info(struct proc_dir_entry *dir);
void clear_opp_cap_info(void);

unsigned int pd_get_nr_caps(int cluster_id);
unsigned long pd_get_opp_capacity(int cpu, int opp);
unsigned int pd_get_cpu_freq(int cpu, int idx);
void nlopp_map_util_freq(void *data, unsigned long util, unsigned long freq,
				  struct cpumask *cpumask, unsigned long *next_freq);
#ifdef CONFIG_ARCH_MEDIATEK
void nonlinear_map_util_freq(void *data, unsigned long util, unsigned long freq,
		unsigned long cap, unsigned long *next_freq, struct cpufreq_policy *policy,
		bool *need_freq_update);
#endif /* CONFIG_ARCH_MEDIATEK */
#endif /* CONFIG_UAG_NONLINEAR_FREQ_CTL */

#ifdef CONFIG_OPLUS_FEATURE_TOUCH_BOOST
int touch_boost_init(void);
#endif

#ifdef CONFIG_OPLUS_UAG_USE_TL
unsigned int choose_util(struct uag_gov_policy *sg_policy, unsigned int util);
bool get_capacity_margin_dvfs_changed_uag(void);
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
void uag_update_util(void *data, unsigned long util, unsigned long freq,
		unsigned long cap, unsigned long *max_util, struct cpufreq_policy *policy,
		bool *need_freq_update);
#endif /* CONFIG_OPLUS_SYSTEM_KERNEL_QCOM */
#endif /* CONFIG_OPLUS_UAG_USE_TL */

#include "stall_util_cal.h"

#endif /* _CPUFREQ_UAG_H_ */
