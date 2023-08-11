/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM uad

#if !defined(_TRACE_UAG_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_UAG_H
#include <linux/string.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#ifdef CONFIG_OPLUS_UAG_USE_TL
TRACE_EVENT(uag_next_util_tl,
	TP_PROTO(unsigned int cpu, unsigned long util, unsigned long max,
		unsigned int target_util),
	TP_ARGS(cpu, util, max, target_util),
	TP_STRUCT__entry(
		__field(unsigned int, cpu)
		__field(unsigned long, util)
		__field(unsigned long, max)
		__field(unsigned int, target_util)),
	TP_fast_assign(
		__entry->cpu = cpu;
		__entry->util = util;
		__entry->max = max;
		__entry->target_util = target_util;),
	TP_printk("cpu=%u util=%lu max=%lu target_util=%u",
		__entry->cpu,
		__entry->util,
		__entry->max,
		__entry->target_util)
);

TRACE_EVENT(uag_next_freq_info,
	    TP_PROTO(int cluster_id, unsigned long util, int opp,
		     unsigned int next_freq),
	    TP_ARGS(cluster_id, util, opp, next_freq),
	    TP_STRUCT__entry(
		    __field(int, cluster_id)
		    __field(unsigned long, util)
		    __field(int, opp)
		    __field(unsigned int, next_freq)),
	    TP_fast_assign(
		    __entry->cluster_id = cluster_id;
		    __entry->util = util;
		    __entry->opp = opp;
		    __entry->next_freq = next_freq;),
	    TP_printk("cluster_id=%d util=%lu opp=%d next_freq=%u",
		      __entry->cluster_id,
		      __entry->util,
		      __entry->opp,
		      __entry->next_freq)
);

TRACE_EVENT(choose_util,
	    TP_PROTO(unsigned int cpu, unsigned int util, unsigned int prevutil, unsigned int utilmax,
		     unsigned int utilmin, unsigned int tl),
	    TP_ARGS(cpu, util, prevutil, utilmax, utilmin, tl),
	    TP_STRUCT__entry(
			__field(unsigned int, cpu)
			__field(unsigned int, util)
			__field(unsigned int, prevutil)
			__field(unsigned int, utilmax)
			__field(unsigned int, utilmin)
			__field(unsigned int, tl)),
	    TP_fast_assign(
			__entry->cpu = cpu;
			__entry->util = util;
			__entry->prevutil = prevutil;
			__entry->utilmax = utilmax;
			__entry->utilmin = utilmin;
			__entry->tl = tl;),
	    TP_printk("cpu=%u util=%u prevutil=%u utilmax=%u utilmin=%u tl=%u",
			__entry->cpu,
			__entry->util,
			__entry->prevutil,
			__entry->utilmax,
			__entry->utilmin,
			__entry->tl)
);

TRACE_EVENT(uag_next_freq_tl,
	    TP_PROTO(unsigned int cpu, unsigned long raw_util, unsigned long raw_freq,
		     unsigned long util, unsigned long next_freq),
	    TP_ARGS(cpu, raw_util, raw_freq, util, next_freq),
	    TP_STRUCT__entry(
		    __field(unsigned int, cpu)
		    __field(unsigned long, raw_util)
		    __field(unsigned long, raw_freq)
		    __field(unsigned long, util)
		    __field(unsigned long, next_freq)),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->raw_util = raw_util;
		    __entry->raw_freq = raw_freq;
		    __entry->util = util;
		    __entry->next_freq = next_freq;),
	    TP_printk("cpu=%u raw_util=%lu raw_freq=%lu util=%lu next_freq=%lu",
		      __entry->cpu,
		      __entry->raw_util,
		      __entry->raw_freq,
		      __entry->util,
		      __entry->next_freq)
);
#endif

#ifdef CONFIG_OPLUS_UAG_AMU_AWARE
#include "stall_util_cal.h"

DECLARE_PER_CPU(struct amu_data, amu_delta);
DECLARE_PER_CPU(u64, amu_update_delta_time);
DECLARE_PER_CPU(u64, amu_normal_util);
DECLARE_PER_CPU(u64, amu_stall_util);

TRACE_EVENT(uag_update_amu_counter,
	    TP_PROTO(int cpu, u64 time),
	    TP_ARGS(cpu, time),
	    TP_STRUCT__entry(
		    __field(int, cpu)
		    __field(u64, time)
		    __field(u64, delta_0)
		    __field(u64, delta_1)
		    __field(u64, delta_2)
		    __field(u64, delta_3)
		    __field(u64, delta_time)
		    __field(u64, normal_util)
		    __field(u64, stall_util)),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->time = time;
		    __entry->delta_0 = per_cpu(amu_delta, cpu).val[0];
		    __entry->delta_1 = per_cpu(amu_delta, cpu).val[1];
		    __entry->delta_2 = per_cpu(amu_delta, cpu).val[2];
		    __entry->delta_3 = per_cpu(amu_delta, cpu).val[3];
		    __entry->delta_time = per_cpu(amu_update_delta_time, cpu);
		    __entry->normal_util = per_cpu(amu_normal_util, cpu);
		    __entry->stall_util = per_cpu(amu_stall_util, cpu);),
	    TP_printk("cpu=%d delta_cntr=%llu,%llu,%llu,%llu delta_time=%llu util=%llu,%llu time=%llu",
		    __entry->cpu,
		    __entry->delta_0,
		    __entry->delta_1,
		    __entry->delta_2,
		    __entry->delta_3,
		    __entry->delta_time,
		    __entry->normal_util,
		    __entry->stall_util,
		    __entry->time)
);

TRACE_EVENT(uag_amu_cnt_calc,
	    TP_PROTO(int cpu,
		    unsigned long avg_freq, unsigned long stall_avg,
		    unsigned long max_freq, unsigned long capacity),
	    TP_ARGS(cpu, avg_freq, stall_avg,
		    max_freq, capacity),
	    TP_STRUCT__entry(
		    __field(int, cpu)
		    __field(unsigned long, avg_freq)
		    __field(unsigned long, stall_avg)
		    __field(unsigned long, max_freq)
		    __field(unsigned long, capacity)),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->avg_freq = avg_freq;
		    __entry->stall_avg = stall_avg;
		    __entry->max_freq = max_freq;
		    __entry->capacity = capacity;),
	    TP_printk("cpu=%d avg_freq=%lu stall_avg=%lu max_freq=%lu capacity=%lu",
		    __entry->cpu,
		    __entry->avg_freq,
		    __entry->stall_avg,
		    __entry->max_freq,
		    __entry->capacity)
);

TRACE_EVENT(uag_amu_adjust_util,
	    TP_PROTO(int cpu,
		    u64 orig, u64 normal, u64 stall, u64 reduce_pct,
		    u64 amu_result, u64 final_util, int policy),
	    TP_ARGS(cpu, orig, normal, stall, reduce_pct,
		    amu_result, final_util, policy),
	    TP_STRUCT__entry(
		    __field(int, cpu)
		    __field(u64, orig)
		    __field(u64, normal)
		    __field(u64, stall)
		    __field(u64, reduce_pct)
		    __field(u64, amu_result)
		    __field(u64, final_util)
		    __field(int, policy)),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->orig = orig;
		    __entry->normal = normal;
		    __entry->stall = stall;
		    __entry->reduce_pct = reduce_pct;
		    __entry->amu_result = amu_result;
		    __entry->final_util = final_util;
		    __entry->policy = policy;),
	    TP_printk("cpu=%d orig=%llu normal=%llu stall=%llu reduce_pct=%llu amu_result=%llu final_util=%llu report_policy=%d",
		    __entry->cpu,
		    __entry->orig,
		    __entry->normal,
		    __entry->stall,
		    __entry->reduce_pct,
		    __entry->amu_result,
		    __entry->final_util,
		    __entry->policy)
);
#endif /* CONFIG_OPLUS_UAG_AMU_AWARE */

#ifdef CONFIG_OPLUS_MULTI_LV_TL
#include "cpufreq_uag.h"
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
TRACE_EVENT(uag_update_multi_util,
	    TP_PROTO(struct uag_gov_policy *sg_policy),
	    TP_ARGS(sg_policy),
	    TP_STRUCT__entry(
		    __field(unsigned int, cpu)
		    __field(unsigned long, sys_util)
		    __field(unsigned long, fbg_util)
		    __field(unsigned int, flags)),
	    TP_fast_assign(
		    __entry->cpu = sg_policy->policy->cpu;
		    __entry->sys_util = sg_policy->multi_util[UA_UTIL_SYS];
		    __entry->fbg_util = sg_policy->multi_util[UA_UTIL_FBG];
		    __entry->flags = sg_policy->flags;),
	    TP_printk("cpu=%u sys_util=%lu fbg_util=%lu, flags=%x",
		      __entry->cpu,
		      __entry->sys_util,
		      __entry->fbg_util,
		      __entry->flags)
);
#endif
TRACE_EVENT(uag_choose_multi_util,
	    TP_PROTO(struct uag_gov_policy *sg_policy, unsigned long target_util),
	    TP_ARGS(sg_policy, target_util),
	    TP_STRUCT__entry(
		    __field(unsigned int, cpu)
		    __field(unsigned long, sys_util)
		    __field(unsigned long, fbg_util)
		    __field(unsigned long, target_util)),
	    TP_fast_assign(
		    __entry->cpu = sg_policy->policy->cpu;
		    __entry->target_util = target_util;
		    __entry->sys_util = sg_policy->multi_util[UA_UTIL_SYS];
		    __entry->fbg_util = sg_policy->multi_util[UA_UTIL_FBG];),
	    TP_printk("cpu=%u target_util=%lu sys_util=%lu fbg_util=%lu",
		      __entry->cpu,
		      __entry->target_util,
		      __entry->sys_util,
		      __entry->fbg_util)
);

TRACE_EVENT(uag_select_multi_tl,
	    TP_PROTO(struct uag_gov_policy *sg_policy, unsigned long util, unsigned int tl),
	    TP_ARGS(sg_policy, util, tl),
	    TP_STRUCT__entry(
		    __field(unsigned int, cpu)
		    __field(unsigned int, multi_util_type)
		    __field(unsigned int, tl)
		    __field(unsigned long, util)),
	    TP_fast_assign(
		    __entry->cpu = sg_policy->policy->cpu;
		    __entry->multi_util_type = sg_policy->multi_util_type;
		    __entry->util = util;
		    __entry->tl = tl;),
	    TP_printk("cpu=%u multi_util_type=%u util=%lu tl=%u",
		      __entry->cpu,
		      __entry->multi_util_type,
		      __entry->util,
		      __entry->tl)
);
#endif

#ifdef CONFIG_OPLUS_UAG_SOFT_LIMIT
TRACE_EVENT(soft_limit,
	    TP_PROTO(unsigned long soft_freq, unsigned long raw_soft_util, unsigned long soft_util,
			unsigned long ua_util, unsigned long raw_util, unsigned int break_freq_margin,
			unsigned long util),
	    TP_ARGS(soft_freq, raw_soft_util, soft_util, ua_util, raw_util, break_freq_margin, util),
	    TP_STRUCT__entry(
			__field(unsigned long, soft_freq)
			__field(unsigned long, raw_soft_util)
			__field(unsigned long, soft_util)
			__field(unsigned long, ua_util)
			__field(unsigned long, raw_util)
			__field(unsigned int, break_freq_margin)
			__field(unsigned long, util)),
	    TP_fast_assign(
			__entry->soft_freq = soft_freq;
			__entry->raw_soft_util = raw_soft_util;
			__entry->soft_util = soft_util;
			__entry->ua_util = ua_util;
			__entry->raw_util = raw_util;
			__entry->break_freq_margin = break_freq_margin;
			__entry->util = util;),
	    TP_printk("soft_freq = %u raw_soft_util=%u soft_util=%u ua_util=%u raw_util=%u break_freq_margin=%u util=%u",
			__entry->soft_freq,
			__entry->raw_soft_util,
			__entry->soft_util,
			__entry->ua_util,
			__entry->raw_util,
			__entry->break_freq_margin,
			__entry->util)
);

TRACE_EVENT(set_soft_limit_freq,
	    TP_PROTO(unsigned int first_cpu, unsigned int soft_freq, unsigned long soft_util),
	    TP_ARGS(first_cpu, soft_freq, soft_util),
	    TP_STRUCT__entry(
			__field(unsigned int, first_cpu)
			__field(unsigned int, soft_freq)
			__field(unsigned long, soft_util)),
	    TP_fast_assign(
			__entry->first_cpu = first_cpu;
			__entry->soft_freq = soft_freq;
			__entry->soft_util = soft_util;),
	    TP_printk("first_cpu = %u soft_freq=%u soft_util=%u",
			__entry->first_cpu,
			__entry->soft_freq,
			__entry->soft_util)
);
#endif

#ifdef CONFIG_ARCH_MEDIATEK
TRACE_EVENT(hispeed_freq,
	    TP_PROTO(unsigned long hispeed_util, unsigned int prev_frame_loading, unsigned long util),
	    TP_ARGS(hispeed_util, prev_frame_loading, util),
	    TP_STRUCT__entry(
			__field(unsigned long, hispeed_util)
			__field(unsigned int, prev_frame_loading)
			__field(unsigned long, util)),
	    TP_fast_assign(
			__entry->hispeed_util = hispeed_util;
			__entry->prev_frame_loading = prev_frame_loading;
			__entry->util = util;),
	    TP_printk("hispeed util = %lu, prev_frame_loading = %u, util = %lu\n",
			__entry->hispeed_util,
			__entry->prev_frame_loading,
			__entry->util)
);
#endif /* CONFIG_ARCH_MEDIATEK */

TRACE_EVENT(ed_task_boost,

	    TP_PROTO(unsigned long cpu_util, unsigned long util, unsigned int ed_task_boost_type,
			unsigned int ed_task_boost_mid_util, unsigned int ed_task_boost_max_util),

	    TP_ARGS(cpu_util, util, ed_task_boost_type, ed_task_boost_mid_util, ed_task_boost_max_util),

	    TP_STRUCT__entry(
			__field(unsigned long, cpu_util)
			__field(unsigned long, util)
			__field(unsigned int, ed_task_boost_type)
			__field(unsigned int, ed_task_boost_mid_util)
			__field(unsigned int, ed_task_boost_max_util)),

	    TP_fast_assign(
			__entry->cpu_util = cpu_util;
			__entry->util = util;
			__entry->ed_task_boost_type = ed_task_boost_type;
			__entry->ed_task_boost_mid_util = ed_task_boost_mid_util;
			__entry->ed_task_boost_max_util = ed_task_boost_max_util;),

	    TP_printk("cpu_util = %lu, util = %lu, ed_task_boost_type = %d, mid_util = %d, max_util = %d",
			__entry->cpu_util, __entry->util, __entry->ed_task_boost_type,
			__entry->ed_task_boost_mid_util, __entry->ed_task_boost_max_util)
);
#endif /* _TRACE_UAG_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE uag_trace
/* This part must be outside protection */
#include <trace/define_trace.h>
