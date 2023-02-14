// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#include "sched_assist.h"
#include "sa_common.h"
#include "sa_fair.h"
#include <kernel/sched/sched.h>
#include <linux/list.h>
#include <include/linux/sched.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_group.h>
#endif

#include <trace_sched_assist.h>

extern unsigned int sysctl_sched_latency;
extern struct ux_sched_cputopo ux_sched_cputopo;

#define MS_TO_NS (1000000)

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
#define NR_IMBALANCE_THRESHOLD (24)
struct cpumask nr_mask;
DEFINE_PER_CPU(struct task_count_rq, task_lb_count);
EXPORT_PER_CPU_SYMBOL(task_lb_count);
#endif

int oplus_idle_cpu(int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	if (rq->curr != rq->idle)
		return 0;

	if (rq->nr_running)
		return 0;

#if IS_ENABLED(CONFIG_SMP)
	if (rq->ttwu_pending)
		return 0;
#endif

	return 1;
}

static inline int get_task_cls_for_scene(struct task_struct *task)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_max = ux_cputopo.cls_nr - 1;
	int cls_mid = cls_max - 1;

	/* only one cluster or init failed */
	if (unlikely(cls_max <= 0))
		return 0;

	/* for 2 clusters cpu, mid = max */
	if (cls_mid == 0)
		cls_mid = cls_max;

	/* for launch scene, heavy ux task should not move to min capacity cluster */
	if (sched_assist_scene(SA_LAUNCH) && test_sched_assist_ux_type(task, SA_TYPE_HEAVY | SA_TYPE_ANIMATOR))
		return test_sched_assist_ux_type(task, SA_TYPE_ANIMATOR) ? cls_mid : cls_max;

	if (sched_assist_scene(SA_ANIM) && test_sched_assist_ux_type(task, SA_TYPE_ANIMATOR))
		return is_task_util_over(task, BOOST_THRESHOLD_UNIT) ? cls_mid : 0;

	if (sched_assist_scene(SA_LAUNCHER_SI))
		return is_task_util_over(task, BOOST_THRESHOLD_UNIT) ? cls_mid : 0;

	if (oplus_get_im_flag(task) == IM_FLAG_CAMERA_HAL)
		return cls_mid;

	return 0;
}

static inline bool is_ux_task_prefer_cpu_for_scene(struct task_struct *task, unsigned int cpu)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_id = ux_cputopo.cls_nr - 1;

	/* only one cluster or init failed */
	if (unlikely(cls_id <= 0))
		return true;

	cls_id = get_task_cls_for_scene(task);
	return arch_scale_cpu_capacity(cpu) >= ux_cputopo.sched_cls[cls_id].capacity;
}

static inline bool skip_rt_and_ux(struct task_struct *p)
{
	return !(sched_assist_scene(SA_LAUNCH) && p->pid == p->tgid
		&& !test_sched_assist_ux_type(p, SA_TYPE_ANIMATOR | SA_TYPE_LIGHT));
}

bool should_ux_task_skip_cpu(struct task_struct *task, unsigned int dst_cpu)
{
	struct oplus_rq *orq = NULL;
	int reason = -1;

	if (unlikely(!global_sched_assist_enabled))
		return false;

	if (!test_task_ux(task))
		return false;

	if (!is_ux_task_prefer_cpu_for_scene(task, dst_cpu)) {
		reason = 0;
		goto skip;
	}

	if (skip_rt_and_ux(task)) {
		if (cpu_rq(dst_cpu)->rt.rt_nr_running) {
			reason = 1;
			goto skip;
		}

		/* camera hal thread only skip rt, because they are too much,
		 * if they skip each other, maybe easily jump to super big core. :(
		 */
		if (oplus_get_im_flag(task) == IM_FLAG_CAMERA_HAL)
			return false;

		orq = (struct oplus_rq *) cpu_rq(dst_cpu)->android_oem_data1;
		if (orq_has_ux_tasks(orq)) {
			reason = 2;
			goto skip;
		}
	}

	return false;

skip:
	if (unlikely(global_debug_enabled & DEBUG_FTRACE))
		trace_printk("ux task=%-12s pid=%d skip_cpu=%d reason=%d\n", task->comm, task->pid, dst_cpu, reason);

	return true;
}
EXPORT_SYMBOL(should_ux_task_skip_cpu);

static inline bool strict_ux_task(struct task_struct *task)
{
	return sched_assist_scene(SA_LAUNCH) && (task->pid == task->tgid)
		&& (task->tgid == save_top_app_tgid);
}

bool set_ux_task_to_prefer_cpu(struct task_struct *task, int *orig_target_cpu)
{
	struct rq *rq = NULL;
	struct oplus_rq *orq = NULL;
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_nr = ux_cputopo.cls_nr - 1;
	int start_cls = -1;
	int cpu = 0;
	int direction = -1;
	int strict_cpu = -1, subopt_cpu = -1;
	bool walk_next_cls = false;
	bool invalid_target = false;

	if (unlikely(!global_sched_assist_enabled))
		return false;

	if (unlikely(cls_nr <= 0))
		return false;

	if (!test_task_ux(task))
		return false;

	if (*orig_target_cpu < 0 || *orig_target_cpu >= OPLUS_NR_CPUS)
		invalid_target = true;

	if (!invalid_target && !sched_assist_scene(SA_LAUNCH) && is_ux_task_prefer_cpu_for_scene(task, *orig_target_cpu))
		return false;

	start_cls = cls_nr = get_task_cls_for_scene(task);
	if (cls_nr != ux_cputopo.cls_nr - 1)
		direction = 1;
retry:
	for_each_cpu(cpu, &ux_cputopo.sched_cls[cls_nr].cpus) {
		rq = cpu_rq(cpu);
		orq = (struct oplus_rq *) rq->android_oem_data1;

		if (strict_ux_task(task) && cpu_online(cpu) && cpu_active(cpu) && cpumask_test_cpu(cpu, task->cpus_ptr)) {
			/*
			 * If the thread running on the CPU being traversed is neither UX nor RT,
			 * then it is the best one, otherwise it is an alternative CPU.
			 */
			if (oplus_list_empty(&orq->ux_list) && !rt_rq_is_runnable(&rq->rt)) {
				strict_cpu = cpu;
				walk_next_cls = false;
			} else {
				subopt_cpu = cpu;
				walk_next_cls = (direction == 1) && (cls_nr != ux_cputopo.cls_nr - 1);
			}
		}

		if (orq_has_ux_tasks(orq))
			continue;

		if (rq->curr->prio < MAX_RT_PRIO)
			continue;

		/* If there are rt threads in runnable state on this CPU, drop it! */
		if (rt_rq_is_runnable(&rq->rt))
			continue;

		if (cpu_online(cpu) && cpu_active(cpu) && cpumask_test_cpu(cpu, task->cpus_ptr)) {
			*orig_target_cpu = cpu;
			trace_set_ux_task_to_prefer_cpu(task, *orig_target_cpu, strict_cpu, cls_nr, start_cls);
			return true;
		}
	}

	if (strict_cpu != -1) {
		*orig_target_cpu = strict_cpu;
		trace_set_ux_task_to_prefer_cpu(task, *orig_target_cpu, strict_cpu, cls_nr, start_cls);
		return true;
	}

	if (!walk_next_cls && subopt_cpu != -1) {
		*orig_target_cpu = subopt_cpu;
		trace_set_ux_task_to_prefer_cpu(task, *orig_target_cpu, strict_cpu, cls_nr, start_cls);
		return true;
	}

	cls_nr = cls_nr + direction;
	if (cls_nr > 0 && cls_nr < ux_cputopo.cls_nr)
		goto retry;

	return false;
}
EXPORT_SYMBOL(set_ux_task_to_prefer_cpu);

bool should_ux_task_skip_eas(struct task_struct *p)
{
	return test_task_ux(p) && global_sched_assist_scene && !sched_assist_scene(SA_CAMERA);
}
EXPORT_SYMBOL(should_ux_task_skip_eas);

#ifdef CONFIG_FAIR_GROUP_SCHED
/* Walk up scheduling entities hierarchy */
#define for_each_sched_entity(se) \
		for (; se; se = se->parent)
#else
#define for_each_sched_entity(se) \
		for (; se; se = NULL)
#endif

extern void set_next_entity(struct cfs_rq *cfs_rq, struct sched_entity *se);
void oplus_replace_next_task_fair(struct rq *rq, struct task_struct **p, struct sched_entity **se, bool *repick, bool simple)
{
	struct oplus_rq *orq = (struct oplus_rq *) rq->android_oem_data1;
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct oplus_task_struct *ots = NULL;
	struct task_struct *temp = NULL;
	struct cfs_rq *cfs_rq = NULL;

	if (unlikely(!global_sched_assist_enabled))
		return;

	if (!orq_has_ux_tasks(orq))
		return;

	list_for_each_safe(pos, n, &orq->ux_list) {
		ots = list_entry(pos, struct oplus_task_struct, ux_entry);
		if (IS_ERR_OR_NULL(ots))
			continue;

		temp = ots_to_ts(ots);
		if (IS_ERR_OR_NULL(temp))
			continue;

		if (unlikely(task_cpu(temp) != rq->cpu))
			continue;

		if (unlikely(!test_task_ux(temp)))
			continue;

		*p = temp;
		*se = &temp->se;
		*repick = true;

		/*
		 * NOTE:
		 * Because the following code is not merged in kernel-5.15,
		 * set_next_entity() will no longer be called to remove the
		 * task from the red-black tree when pick_next_task_fair(),
		 * so we remove the picked task here.
		 *
		 * https://android-review.googlesource.com/c/kernel/common/+/1667002
		 */
		if (simple) {
			for_each_sched_entity((*se)) {
				cfs_rq = cfs_rq_of(*se);
				set_next_entity(cfs_rq, *se);
			}
		}

		break;
	}
}
EXPORT_SYMBOL(oplus_replace_next_task_fair);

inline void oplus_check_preempt_wakeup(struct rq *rq, struct task_struct *p, bool *preempt, bool *nopreempt)
{
	struct task_struct *curr = rq->curr;
	bool wake_ux = false;
	bool curr_ux = false;
	struct list_head *list = NULL;

	if (likely(!global_sched_assist_enabled))
		return;

	wake_ux = test_task_ux(p);
	curr_ux = test_task_ux(curr);

	if (!wake_ux && !curr_ux)
		return;

	/* ux can preempt un-ux */
	if (wake_ux && !curr_ux) {
		*preempt = true;
		return;
	}

	if (!wake_ux && curr_ux) {
		*nopreempt = true;
		goto update;
	}

	/* both of wake_task and curr_task are ux */
	if (prio_higher(oplus_get_ux_state(p), oplus_get_ux_state(curr)))
		*preempt = true;

update:
	/* if curr is ux task, update it's runtime here */
	list = oplus_get_ux_entry(curr);
	if (!IS_ERR_OR_NULL(list) && !oplus_list_empty(list))
		account_ux_runtime(rq, curr);
}
EXPORT_SYMBOL(oplus_check_preempt_wakeup);

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
bool is_spread_task_enabled(void)
{
	return (global_sched_assist_enabled & FEATURE_SPREAD) && !sched_assist_scene(SA_CAMERA);
}
EXPORT_SYMBOL(is_spread_task_enabled);

void update_rq_nr_imbalance(int cpu)
{
	int total_nr = 0;
	int i = -1;
	int threshold = NR_IMBALANCE_THRESHOLD;

	/* Note: check without holding rq lock */
	for_each_cpu(i, cpu_active_mask) {
		total_nr += cpu_rq(i)->nr_running;
		if (oplus_idle_cpu(i))
			cpumask_clear_cpu(i, &nr_mask);
	}

	if (!oplus_idle_cpu(cpu) && (total_nr >= threshold))
		cpumask_set_cpu(cpu, &nr_mask);
	else
		cpumask_clear_cpu(cpu, &nr_mask);
}

bool should_force_spread_tasks(void)
{
	return !cpumask_empty(&nr_mask);
}
EXPORT_SYMBOL(should_force_spread_tasks);

static inline int task_cgroup_id(struct task_struct *task)
{
	struct cgroup_subsys_state *css = task_css(task, cpu_cgrp_id);

	return css ? css->id : -1;
}

int task_lb_sched_type(struct task_struct *tsk)
{
	int cgroup_type = task_cgroup_id(tsk);

	if (test_task_ux(tsk))
		return SA_UX;
	else if (cgroup_type == SA_CGROUP_TOP_APP)
		return SA_TOP;
	else if (cgroup_type == SA_CGROUP_FOREGROUND || cgroup_type == SA_CGROUP_DEFAULT)
		return SA_FG;
	else if (cgroup_type == SA_CGROUP_BACKGROUND)
		return SA_BG;

	return SA_INVALID;
}
EXPORT_SYMBOL(task_lb_sched_type);

void dec_task_lb(struct task_struct *tsk, struct rq *rq,
	int high_load, int task_type)
{
	int cpu = cpu_of(rq);

	if (high_load == SA_HIGH_LOAD) {
		switch (task_type) {
		case SA_UX:
			per_cpu(task_lb_count, cpu).ux_high--;
			break;
		case SA_TOP:
			per_cpu(task_lb_count, cpu).top_high--;
			break;
		case SA_FG:
			per_cpu(task_lb_count, cpu).foreground_high--;
			break;
		case SA_BG:
			per_cpu(task_lb_count, cpu).background_high--;
			break;
		}
	} else if (high_load == SA_LOW_LOAD) {
		switch (task_type) {
		case SA_UX:
			per_cpu(task_lb_count, cpu).ux_low--;
			break;
		case SA_TOP:
			per_cpu(task_lb_count, cpu).top_low--;
			break;
		case SA_FG:
			per_cpu(task_lb_count, cpu).foreground_low--;
			break;
		case SA_BG:
			per_cpu(task_lb_count, cpu).background_low--;
			break;
		}
	}
}
EXPORT_SYMBOL(dec_task_lb);

void inc_task_lb(struct task_struct *tsk, struct rq *rq,
	int high_load, int task_type)
{
	int cpu = cpu_of(rq);

	if (high_load == SA_HIGH_LOAD) {
		switch (task_type) {
		case SA_UX:
			per_cpu(task_lb_count, cpu).ux_high++;
			break;
		case SA_TOP:
			per_cpu(task_lb_count, cpu).top_high++;
			break;
		case SA_FG:
			per_cpu(task_lb_count, cpu).foreground_high++;
			break;
		case SA_BG:
			per_cpu(task_lb_count, cpu).background_high++;
			break;
		}
	} else if (high_load == SA_LOW_LOAD) {
		switch (task_type) {
		case SA_UX:
			per_cpu(task_lb_count, cpu).ux_low++;
			break;
		case SA_TOP:
			per_cpu(task_lb_count, cpu).top_low++;
			break;
		case SA_FG:
			per_cpu(task_lb_count, cpu).foreground_low++;
			break;
		case SA_BG:
			per_cpu(task_lb_count, cpu).background_low++;
			break;
		}
	}
}
EXPORT_SYMBOL(inc_task_lb);

void inc_ld_stats(struct task_struct *tsk, struct rq *rq)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);
	int curr_high_load;
	int curr_task_type;

	if (IS_ERR_OR_NULL(ots))
		return;

	curr_high_load = ots->lb_state & 0x1;
	curr_task_type = (ots->lb_state >> 1) & 0x7;

	inc_task_lb(tsk, rq, curr_high_load, curr_task_type);
	ots->ld_flag = 1;
}
EXPORT_SYMBOL(inc_ld_stats);

void dec_ld_stats(struct task_struct *tsk, struct rq *rq)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);
	int curr_high_load;
	int curr_task_type;

	if (IS_ERR_OR_NULL(ots))
		return;

	curr_high_load = ots->lb_state & 0x1;
	curr_task_type = (ots->lb_state >> 1) & 0x7;

	ots->ld_flag = 0;
	dec_task_lb(tsk, rq, curr_high_load, curr_task_type);
}
EXPORT_SYMBOL(dec_ld_stats);

#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */


/* implement vender hook in driver/android/fair.c */
void android_rvh_place_entity_handler(void *unused, struct cfs_rq *cfs_rq, struct sched_entity *se, int initial, u64 *vruntime)
{
}

void android_rvh_check_preempt_tick_handler(void *unused, struct task_struct *p, unsigned long *ideal_runtime)
{
}

void android_rvh_pick_next_entity_handler(void *unused, struct cfs_rq *cfs_rq, struct sched_entity *curr,
			struct sched_entity **se)
{
}

void android_rvh_check_preempt_wakeup_ignore_handler(void *unused, struct task_struct *p, bool *ignore)
{
}

void android_rvh_check_preempt_wakeup_handler(void *unused, struct rq *rq, struct task_struct *p, bool *preempt, bool *nopreempt,
	int wake_flags, struct sched_entity *se, struct sched_entity *pse, int next_buddy_marked, unsigned int granularity)
{
	oplus_check_preempt_wakeup(rq, p, preempt, nopreempt);
}

void android_rvh_replace_next_task_fair_handler(void *unused,
		struct rq *rq, struct task_struct **p, struct sched_entity **se, bool *repick, bool simple, struct task_struct *prev)
{
	oplus_replace_next_task_fair(rq, p, se, repick, simple);
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
migrate_task_callback_t fbg_migrate_task_callback;
EXPORT_SYMBOL(fbg_migrate_task_callback);
#endif

void android_rvh_can_migrate_task_handler(void *unused, struct task_struct *p, int dst_cpu, int *can_migrate)
{
	if (should_ux_task_skip_cpu(p, dst_cpu))
		*can_migrate = 0;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (fbg_migrate_task_callback &&
		fbg_migrate_task_callback(p, task_cpu(p), dst_cpu))
		*can_migrate = 0;
#endif
}

void task_tpd_mask(struct task_struct *tsk, cpumask_t *request)
{
	int i = 0, j;
	int tpd, tmp_tpd;
	char buf[16];
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int max_tpd_val = (1 << ux_cputopo.cls_nr);

	cpumask_t mask = CPU_MASK_NONE;

	if (test_task_ux(tsk))
		return;

	tpd = tmp_tpd = oplus_get_tpd(tsk);
	if (!tpd)
		return;

	if (unlikely(tpd >= max_tpd_val))
		return;

	while (tmp_tpd > 0) {
		if (tmp_tpd & 1) {
			for_each_cpu(j, &ux_cputopo.sched_cls[i].cpus)
				cpumask_set_cpu(j, &mask);
			i++;
		}
		tmp_tpd >>= 1;
	}

	cpumask_copy(request, &mask);
	oplus_show_cpus(request, buf);
	ux_debug("%s: pid = %d: comm = %s, tpd = %d, related_cpus = %s\n", __func__, tsk->pid, tsk->comm, tpd, buf);
}
EXPORT_SYMBOL(task_tpd_mask);

bool task_tpd_check(struct task_struct *tsk, int dst_cpu)
{
	int i = 0;
	int tpd;
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int max_tpd_val = (1 << ux_cputopo.cls_nr);

	tpd = oplus_get_tpd(tsk);
	if (!tpd)
		return true;

	if (unlikely(tpd >= max_tpd_val))
		return true;

	while (tpd > 0) {
		if (tpd & 1) {
			if (cpumask_test_cpu(dst_cpu, &ux_cputopo.sched_cls[i].cpus))
				return true;
			i++;
		}
		tpd >>= 1;
	}

	return false;
}
EXPORT_SYMBOL(task_tpd_check);
