// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

/*
 * we need this file to avoid circle detected.
*/
#include "walt.h"

#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#include <../kernel/oplus_cpu/sched/sched_assist/sa_fair.h>
#if IS_ENABLED(CONFIG_SCHED_WALT)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_boost.h>
#endif


#define CREATE_TRACE_POINTS
#include "trace_oem_sched.h"

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
#if IS_ENABLED(CONFIG_SCHED_WALT)
bool task_high_load(struct task_struct *tsk)
{
	unsigned int cpu = 0;
	unsigned long capacity = capacity_orig_of(cpu);
	unsigned long max_capacity = cpu_rq(cpu)->rd->max_cpu_capacity;
	unsigned int margin;
	int sched_type = task_lb_sched_type(tsk);
	unsigned long load = task_util(tsk);

	if (sched_type == SA_BG)
		load = min(load, tsk->se.avg.util_avg);

	load = clamp(load,
		     uclamp_eff_value(tsk, UCLAMP_MIN),
		     uclamp_eff_value(tsk, UCLAMP_MAX));

	if (capacity == max_capacity)
		return true;

	if (capacity_orig_of(task_cpu(tsk)) > capacity_orig_of(cpu))
		margin = sched_capacity_margin_down[cpu];
	else
		margin = sched_capacity_margin_up[task_cpu(tsk)];

	return capacity * 1024 < load * margin;
}
#else
bool task_high_load(struct task_struct *tsk)
{
	return false;
}
#endif

void update_load_flag(struct task_struct *tsk, struct rq *rq)
{
	int curr_high_load = 0;
	int curr_task_type = 0;
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);

	if (IS_ERR_OR_NULL(ots) || !ots->ld_flag) {
		return;
	}

	curr_high_load = task_high_load(tsk);
	curr_task_type = task_lb_sched_type(tsk);
	if (ots->lb_state != 0) {
		int prev_high_load = ots->lb_state & 0x1;
		int prev_task_type = (ots->lb_state >> 1) & 0x7;

		if (prev_high_load == curr_high_load && prev_task_type == curr_task_type)
			return;
		else
			dec_task_lb(tsk, rq, prev_high_load, prev_task_type);
	}

	inc_task_lb(tsk, rq, curr_high_load, curr_task_type);
	ots->lb_state = (curr_task_type << 1) | curr_high_load;
}

static void find_spread_lowest_nr_cpu(struct task_struct *p, cpumask_t *visit_cpus_t, int sched_type,
	int prev_cpu, int skip_cpu, int *lowest_nr, int *lowest_nr_load, int *lowest_nr_cpu)
{
	int i = 0;

	for_each_cpu(i, visit_cpus_t) {
		int ux_nr = 0;
		int top_nr = 0;
		int fg_nr = 0;
		int bg_nr = 0;
		int rq_nr = 0;
		int rq_nr_load = 0;

		if (!cpu_active(i))
			continue;

#if IS_ENABLED(CONFIG_SCHED_WALT)
		if (is_reserved(i))
			continue;

		if (sched_cpu_high_irqload(i))
			continue;
#endif

		if (skip_cpu == i)
			continue;

		ux_nr = per_cpu(task_lb_count, i).ux_high + per_cpu(task_lb_count, i).ux_low;
		top_nr = per_cpu(task_lb_count, i).top_high + per_cpu(task_lb_count, i).top_low;
		fg_nr = per_cpu(task_lb_count, i).foreground_high + per_cpu(task_lb_count, i).foreground_low;
		bg_nr = per_cpu(task_lb_count, i).background_high + per_cpu(task_lb_count, i).background_low;

		if (sched_type == SA_UX) {
			rq_nr = ux_nr;
		} else if (sched_type == SA_TOP) {
			rq_nr = ux_nr + top_nr;
		} else if (sched_type == SA_FG) {
			rq_nr = ux_nr + top_nr + fg_nr;
		} else if (sched_type == SA_BG) {
			rq_nr = ux_nr + top_nr + fg_nr + bg_nr;
		}
		rq_nr_load = 1000 * ux_nr + 100 * top_nr + 10 * fg_nr + bg_nr;

		if (rq_nr > *lowest_nr) {
			continue;
		}

		if (rq_nr == *lowest_nr) {
			if (rq_nr_load < *lowest_nr_load)
				goto find;
			if (rq_nr_load == *lowest_nr_load && i == prev_cpu)
				goto find;

			continue;
		}

find:
		*lowest_nr = rq_nr;
		*lowest_nr_load = rq_nr_load;
		*lowest_nr_cpu = i;
	}
}

#if IS_ENABLED(CONFIG_SCHED_WALT)
inline bool should_adjust_task_placement(struct task_struct *p)
{
	/* only valid for Heavy-UX threads */
	if (!p || !is_heavy_ux_task(p))
		return false;

	if (sched_assist_scene(SA_LAUNCH | SA_ANIM))
		return true;

	/* input and slide scenes within the app interface also need boost */
	if (sysctl_slide_boost_enabled || sysctl_input_boost_enabled)
		return true;

	return false;
}

void sched_assist_spread_tasks(struct task_struct *p, cpumask_t new_allowed_cpus,
		int order_index, int end_index, int skip_cpu, cpumask_t *cpus, bool strict)
{
	int sched_type = task_lb_sched_type(p);
	int cluster = 0;
	cpumask_t visit_cpus;
	int lowest_nr = INT_MAX;
	int lowest_nr_load = INT_MAX;
	int lowest_nr_cpu = -1;
	int prev_cpu = task_cpu(p);
	bool force_spread = false;

	if (unlikely(!is_spread_task_enabled()))
		return;

	if (num_sched_clusters <= 1 || sched_type == -1)
		return;

	/*
	 * NOTE:
	 *     The end_index parameter indicates the number of clusters to be checked!
	 *
	 * Function:
	 *     Different threads traverse the cluster in the following order when selecting the CPU:
	 *
	 *     SA_LAUNCH scene
	 *     a) ui: cls2
	 *     b) rd: cls2 -> cls1
	 *
	 *     other scene
	 *     a) all thread: cls1 -> cls2
	 */
	if (should_adjust_task_placement(p)) {
		strict = true;
		if (sched_assist_scene(SA_LAUNCH) || num_sched_clusters <= 2) {
			order_index = num_sched_clusters - 1;
			end_index = (p->pid == p->tgid) ? 0 : 1;
		} else {
			order_index = num_sched_clusters - 2;
			end_index = 1;
		}
	}

	/* force scheduler to spread tasks */
	if (should_force_spread_tasks())
		force_spread = true;
	else
		return;

	for (cluster = 0; cluster < num_sched_clusters; cluster++) {
		cpumask_and(&visit_cpus, &new_allowed_cpus,
				&cpu_array[order_index][cluster]);

		find_spread_lowest_nr_cpu(p, &visit_cpus, sched_type, prev_cpu, skip_cpu,
			&lowest_nr, &lowest_nr_load, &lowest_nr_cpu);

		/* should we visit next cluster? */
		if (strict && cluster >= end_index) {
			break;
		}

		if (force_spread)
			continue;

		break;
	}

	if (lowest_nr_cpu != -1) {
		cpumask_set_cpu(lowest_nr_cpu, cpus);
		trace_sched_assist_spread_tasks(p, sched_type, lowest_nr_cpu);
	}
}
#else
void sched_assist_spread_tasks(struct task_struct *p, cpumask_t new_allowed_cpus,
		int order_index, int end_index, int skip_cpu, cpumask_t *cpus, bool strict)
{
}
#endif

void printf_cpu_spread_nr_info(unsigned int cpu, char *nr_info, int info_size)
{
	int rq_ux_nr = 0, rq_top_nr = 0, rq_fg_nr = 0, rq_bg_nr = 0;

	if (cpu >= 0 && cpu < OPLUS_NR_CPUS) {
		rq_ux_nr = per_cpu(task_lb_count, cpu).ux_low +
			per_cpu(task_lb_count, cpu).ux_high;
		rq_top_nr = per_cpu(task_lb_count, cpu).top_low +
			per_cpu(task_lb_count, cpu).top_high;
		rq_fg_nr = per_cpu(task_lb_count, cpu).foreground_low +
			per_cpu(task_lb_count, cpu).foreground_high;
		rq_bg_nr = per_cpu(task_lb_count, cpu).background_low +
			per_cpu(task_lb_count, cpu).background_high;

		scnprintf(nr_info, info_size, "(%d:%d:%d:%d:%d)",
			cpu_rq(cpu)->nr_running, rq_ux_nr, rq_top_nr, rq_fg_nr, rq_bg_nr);
	}
}

#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
