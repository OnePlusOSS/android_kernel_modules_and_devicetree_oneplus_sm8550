// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#include "sched_assist.h"
#include "sa_common.h"
#include "sa_fair.h"
#include "sa_priority.h"
#include <kernel/sched/sched.h>
#include <linux/list.h>
#include <include/linux/sched.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_group.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAKE_CAP)
#include "../eas_opt/fake_cap.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
#include "sa_balance.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
#include "sa_pipeline.h"
#endif

#include <trace_sched_assist.h>
#ifdef CONFIG_LOCKING_PROTECT
#include "sched_assist_locking.h"
#endif

extern unsigned int sysctl_sched_latency;

#define MS_TO_NS (1000000)

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
#define NR_IMBALANCE_THRESHOLD (24)
struct cpumask nr_mask;
DEFINE_PER_CPU(struct task_count_rq, task_lb_count);
EXPORT_PER_CPU_SYMBOL(task_lb_count);
#endif

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
struct cpumask *ux_cpu_halt_mask = NULL;
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
		&& !test_sched_assist_ux_type(p, SA_TYPE_URGENT_MASK));
}

bool should_ux_task_skip_cpu(struct task_struct *task, unsigned int dst_cpu)
{
	struct oplus_rq *orq = NULL;
	int reason = -1;

	if (unlikely(!global_sched_assist_enabled))
		return false;

	if (!test_task_ux(task))
		return false;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	if (pipeline_task_skip_cpu(task, dst_cpu)) {
		reason = -1;
		goto skip;
	}
#endif

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

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
		if (ux_cpu_halt_mask && cpumask_test_cpu(cpu, ux_cpu_halt_mask))
			continue;
#endif

		if (strict_ux_task(task) && cpu_online(cpu) && cpu_active(cpu) && cpumask_test_cpu(cpu, task->cpus_ptr)) {
			/*
			 * If the thread running on the CPU being traversed is neither UX nor RT,
			 * then it is the best one, otherwise it is an alternative CPU.
			 */
			if (oplus_rbtree_empty(&orq->ux_list) && !rt_rq_is_runnable(&rq->rt)) {
				strict_cpu = cpu;
				walk_next_cls = false;
			} else {
				subopt_cpu = cpu;
				walk_next_cls = (direction == 1) && (cls_nr != ux_cputopo.cls_nr - 1);
			}
		}

		/* If an ux thread running on this CPU, drop it! */
		if (oplus_get_ux_state(rq->curr) & SCHED_ASSIST_UX_MASK)
			continue;

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
	struct rb_node *node;
	unsigned long irqflag;

	if (unlikely(!global_sched_assist_enabled))
		return;

	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	if (!orq_has_ux_tasks(orq)) {
		spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
		return;
	}

	while ((node = rb_first_cached(&orq->ux_list)) != NULL) {
		struct oplus_task_struct *ots = rb_entry(node, struct oplus_task_struct, ux_entry);
		struct task_struct *temp = ots_to_ts(ots);
		if (IS_ERR_OR_NULL(temp))
			continue;

		if (unlikely(task_cpu(temp) != rq->cpu)) {
			update_ux_timeline_task_removal(orq, ots);
			put_task_struct(temp);
			DEBUG_BUG_ON(1);
			continue;
		}

		if (unlikely(!test_task_ux(temp))) {
			update_ux_timeline_task_removal(orq, ots);
			put_task_struct(temp);

			/*
			 * WARNING:
			 * Too many print logs may cause the following problems
			 * so WARN_ON here is not smart:
			 * a) this may affect standby power consumption;
			 * b) Too many logs may cause the device to crash because
			 *	  it currently holds rq->lock;
			 */
			/* WARN_ON(1); */
			continue;
		}

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
				struct cfs_rq *cfs_rq = cfs_rq_of(*se);
				set_next_entity(cfs_rq, *se);
			}
		}

		break;
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
}
EXPORT_SYMBOL(oplus_replace_next_task_fair);

inline void oplus_check_preempt_wakeup(struct rq *rq, struct task_struct *p, bool *preempt, bool *nopreempt)
{
	struct task_struct *curr;
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	unsigned long irqflag;
	bool wake_ux;
	bool curr_ux;

	/* this cpu is running in this function, no rcu primitives needed*/
	curr = rq->curr;
	ots = get_oplus_task_struct(curr);
#ifdef CONFIG_LOCKING_PROTECT
	if (!IS_ERR_OR_NULL(ots)) {
		if (task_inlock(ots) && !task_skip_protect(p)) {
			if (locking_protect_disable == false)
				locking_wakeup_preepmt_enable = 1;
			*nopreempt = true;
			*preempt = false;
			return;
		}
	}
#endif

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
		return;
	}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	/*
	 * ui task can preempt pipeline task,
	 * other ux task can not preempt pipeline task
	 */
	if ((p->pid != curr->tgid) && oplus_is_pipeline_task(curr)) {
		*nopreempt = true;
		return;
	}
#endif

	/* both of wake_task and curr_task are ux */
	orq = (struct oplus_rq *) rq->android_oem_data1;
	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	if (!IS_ERR_OR_NULL(ots) && !oplus_rbnode_empty(&ots->ux_entry)) {
		/* account_ux_runtime(rq, curr); */
		if (need_wakeup_preempt(orq, ots)) {
			*preempt = true;
		} else {
			*nopreempt = true;
		}
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
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
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAKE_CAP)
	struct task_struct *se_task = NULL;
	int cpu = cpu_of(rq_of(cfs_rq));
	unsigned int cluster_id = topology_physical_package_id(cpu);
	u64 adjust_time = 0;

	if (!sa_adjust_group_enable || fake_cap_multiple[cluster_id] <= 100)
		return;

	if (!oplus_entity_is_task(se) || initial)
		return;

	se_task = task_of(se);
	if (test_task_ux(se_task))
		return;

	switch (get_grp_adinfo(se_task)) {
	case AD_TOP:
		adjust_time = (group_adjust.adjust_std_vtime_slice * group_adjust.group_param[AD_TOP].vtime_compensate * fake_cap_multiple[cluster_id]);
		break;
	case AD_FG:
		adjust_time = (group_adjust.adjust_std_vtime_slice * group_adjust.group_param[AD_FG].vtime_compensate * fake_cap_multiple[cluster_id]);
		break;
	case AD_BG:
		adjust_time = (group_adjust.adjust_std_vtime_slice * group_adjust.group_param[AD_BG].vtime_compensate * fake_cap_multiple[cluster_id]);
		break;
	case AD_DF:
		adjust_time = (group_adjust.adjust_std_vtime_slice * group_adjust.group_param[AD_DF].vtime_compensate * fake_cap_multiple[cluster_id]);
		break;
	default:
		break;
	}
	adjust_time = clamp_val(adjust_time, 0, se->vruntime);
	se->vruntime -= adjust_time;
	if (unlikely(eas_opt_debug_enable))
		trace_printk("[eas_opt]: common:%s, pid: %d, cpu: %d, group_id: %d, adjust_time: %u, adjust_after_vtime: %llu\n",
				se_task->comm, se_task->pid, cpu, get_grp_adinfo(se_task), adjust_time, se->vruntime);
#endif
}

void android_rvh_check_preempt_tick_handler(void *unused, struct task_struct *task,
			unsigned long *ideal_runtime, bool *skip_preempt,
			unsigned long delta_exec, struct cfs_rq *cfs_rq,
			struct sched_entity *se, unsigned int granularity)
{
	struct rq *rq;
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	unsigned long irqflag;

#ifdef CONFIG_LOCKING_PROTECT
	check_preempt_tick_handler_locking(task, ideal_runtime, skip_preempt, delta_exec,
		cfs_rq,	se, granularity);
#endif

	if (likely(!global_sched_assist_enabled))
		return;

	/* check_preempt_tick hook is called within a loop, only handle when se is task's.
	 * when entity_is_task is true, task_of(se), parameter task and current are the same.
	 */
	if (!entity_is_task(se)) {
		return;
	}
	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots)) {
		return;
	}

	rq = task_rq(task);
	orq = (struct oplus_rq *) rq->android_oem_data1;

	if (oplus_rbnode_empty(&ots->ux_entry) && (!oplus_rbtree_empty(&orq->ux_list))) {
		resched_curr(rq);
		return;
	}

	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	if (!oplus_rbnode_empty(&ots->ux_entry)) {
		if (need_resched_ux(orq, ots, delta_exec)) {
			resched_curr(rq);
		} else {
			*skip_preempt = true;
		}
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
}

void android_rvh_enqueue_entity_handler(void *unused, struct cfs_rq *cfs, struct sched_entity *se)
{
#ifdef CONFIG_LOCKING_PROTECT
	struct task_struct *p = entity_is_task(se) ? task_of(se) : NULL;
	struct rq *rq = rq_of(cfs);

	enqueue_locking_thread(rq, p);
#endif
}

void android_rvh_dequeue_entity_handler(void *unused, struct cfs_rq *cfs, struct sched_entity *se)
{
#ifdef CONFIG_LOCKING_PROTECT
	struct task_struct *p = entity_is_task(se) ? task_of(se) : NULL;
	struct rq *rq = rq_of(cfs);

	dequeue_locking_thread(rq, p);
#endif
}

void android_rvh_check_preempt_wakeup_handler(void *unused, struct rq *rq, struct task_struct *p, bool *preempt, bool *nopreempt,
	int wake_flags, struct sched_entity *se, struct sched_entity *pse, int next_buddy_marked, unsigned int granularity)
{
	oplus_check_preempt_wakeup(rq, p, preempt, nopreempt);
}

#ifndef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
/*add hook for new task util init*/
void android_rvh_post_init_entity_util_avg_handler(void *unused, struct sched_entity *se)
{
	struct task_struct *task = task_of(se);
	struct sched_avg *sa = &se->avg;

	/*in douyin scene,decease new task util for low power issue*/
	if ((!strcmp(task->group_leader->comm, "droid.ugc.aweme")) && (sa->util_avg >= 50))
		sa->util_avg = 50;
}
#endif

void android_rvh_replace_next_task_fair_handler(void *unused,
		struct rq *rq, struct task_struct **p, struct sched_entity **se, bool *repick, bool simple, struct task_struct *prev)
{
	oplus_replace_next_task_fair(rq, p, se, repick, simple);
#ifdef CONFIG_LOCKING_PROTECT
	if (*repick != true)
		oplus_replace_locking_task_fair(rq, p, se, repick, simple);
#endif

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
			struct cfs_rq *cfs_rq = cfs_rq_of(*se);
			set_next_entity(cfs_rq, *se);
		}
	}
}
EXPORT_SYMBOL(android_rvh_replace_next_task_fair_handler);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
migrate_task_callback_t fbg_migrate_task_callback;
EXPORT_SYMBOL(fbg_migrate_task_callback);
#endif

void android_rvh_can_migrate_task_handler(void *unused, struct task_struct *p, int dst_cpu, int *can_migrate)
{
	if (should_ux_task_skip_cpu(p, dst_cpu))
		*can_migrate = 0;

	/* have indicated that migration is rejected, no need more judgement */
	if (*can_migrate == 0)
		return;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (fbg_migrate_task_callback &&
		fbg_migrate_task_callback(p, task_cpu(p), dst_cpu))
		*can_migrate = 0;
#endif
}

void task_tpd_mask(struct task_struct *tsk, cpumask_t *request)
{
}
EXPORT_SYMBOL(task_tpd_mask);

bool task_tpd_check(struct task_struct *tsk, int dst_cpu)
{
	return true;
}
EXPORT_SYMBOL(task_tpd_check);

#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
bool oplus_cpu_halted(unsigned int cpu)
{
	return ux_cpu_halt_mask && cpumask_test_cpu(cpu, ux_cpu_halt_mask);
}

void init_ux_halt_mask(struct cpumask *halt_mask)
{
	ux_cpu_halt_mask = halt_mask;
}
EXPORT_SYMBOL_GPL(init_ux_halt_mask);
#endif
