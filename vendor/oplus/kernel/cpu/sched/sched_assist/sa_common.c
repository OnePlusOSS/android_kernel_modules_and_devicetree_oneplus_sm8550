// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/jiffies.h>

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <linux/cpumask.h>
#include <linux/sched/topology.h>
#include <linux/sched/task.h>

#include <kernel/sched/sched.h>
#include <fs/proc/internal.h>
#include <linux/signal.h>
#include <linux/cpufeature.h>
#include <linux/thread_info.h>
#include <linux/profile.h>
#include <linux/kprobes.h>

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
#include <linux/cpuhotplug.h>
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
#include "sa_jankinfo.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
#include "sa_balance.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_group.h>
#endif

#include "sched_assist.h"
#include "sa_common.h"
#include "sa_priority.h"
#ifdef CONFIG_LOCKING_PROTECT
#include "sched_assist_locking.h"
#endif

#ifdef CONFIG_OPLUS_CPU_AUDIO_PERF
#include "sa_audio.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
#include "sa_pipeline.h"
#endif

#include "sa_exec.h"

#define CREATE_TRACE_POINTS
#include <trace_sched_assist.h>

#define MS_TO_NS (1000000)
#define MAX_INHERIT_GRAN ((u64)(64 * MS_TO_NS))

#define INHERIT_UX_SEC_WIDTH				8
#define INHERIT_UX_MASK_BASE				0x00000000ff

#define inherit_ux_offset_of(type)			(type * INHERIT_UX_SEC_WIDTH)
#define inherit_ux_mask_of(type)			((u64)(INHERIT_UX_MASK_BASE) << (inherit_ux_offset_of(type)))
#define inherit_ux_get_bits(value, type)	((value & inherit_ux_mask_of(type)) >> inherit_ux_offset_of(type))
#define inherit_ux_value(type, value)		((u64)value << inherit_ux_offset_of(type))

/* debug print frequency limit */
static DEFINE_PER_CPU(int, prev_ux_state);
static DEFINE_PER_CPU(int, prev_ux_priority);
static DEFINE_PER_CPU(u64, prev_vruntime);
static DEFINE_PER_CPU(u64, prev_min_vruntime);
static DEFINE_PER_CPU(u64, prev_preset_vruntime);
static DEFINE_PER_CPU(int, prev_hwbinder_flag);

#if IS_ENABLED(CONFIG_SCHED_WALT)
#define WINDOW_SIZE (16000000)
#define scale_demand(d) ((d)/(WINDOW_SIZE >> SCHED_CAPACITY_SHIFT))
#endif


#define TOPAPP 4
#define BGAPP  3

bool is_top(struct task_struct *p)
{
	struct cgroup_subsys_state *css;

	if (p == NULL)
		return false;

	rcu_read_lock();
	css = task_css(p, cpu_cgrp_id);
	if (!css) {
		rcu_read_unlock();
		return false;
	}
	rcu_read_unlock();

	return css->id == TOPAPP;
}

#ifdef CONFIG_OPLUS_FEATURE_INPUT_BOOST
bool is_webview(struct task_struct *p)
{
	if (!is_top(p))
		return false;

	if (oplus_get_im_flag(p) == IM_FLAG_WEBVIEW)
		return true;

	return false;
}
#endif

struct ux_sched_cputopo ux_sched_cputopo;

static inline void sched_init_ux_cputopo(void)
{
	int i = 0;

	ux_sched_cputopo.cls_nr = 0;
	for (; i < OPLUS_NR_CPUS; ++i) {
		cpumask_clear(&ux_sched_cputopo.sched_cls[i].cpus);
		ux_sched_cputopo.sched_cls[i].capacity = ULONG_MAX;
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
void dump_oplus_cpu_array(void)
{
	struct ux_sched_cputopo *ux_cputopo = &ux_sched_cputopo;
	int cls_nr = ux_cputopo->cls_nr;
	int i, j;
	char buf[256];
	int count = 0;

	for (i = 0; i < 2*cls_nr; i++) {
		count += snprintf(buf + count, PAGE_SIZE - count, "OPLUS_CPU_ARRAY: order_idx=%d, [", i);
		for (j = 0; j < cls_nr; j++) {
			count += snprintf(buf + count, PAGE_SIZE - count, "%d%s",
				topology_physical_package_id(cpumask_first(&ux_cputopo->oplus_cpu_array[i][j])),
				j == cls_nr-1 ? "":" -> ");
		}
		count += snprintf(buf + count, PAGE_SIZE - count, "]\n");
		pr_info("%s", buf);
		memset(buf, 0, sizeof(buf));
		count = 0;
	}
}
EXPORT_SYMBOL_GPL(dump_oplus_cpu_array);

static void build_oplus_cpu_array(void)
{
	struct ux_sched_cputopo *ux_cputopo = &ux_sched_cputopo;
	int cls_nr = ux_cputopo->cls_nr;
	int i;

	/* Construct cpu_array row by row */
	for (i = 0; i < cls_nr; i++) {
		int j, k = 1;

		/* Fill out first column with appropriate cpu arrays */
		cpumask_copy(&ux_cputopo->oplus_cpu_array[i][0],
				&ux_cputopo->sched_cls[i].cpus);

		/*
		 * k starts from column 1 because 0 is filled
		 * Fill clusters for the rest of the row,
		 * above i in ascending order
		 */
		for (j = i + 1; j < cls_nr; j++) {
			cpumask_copy(&ux_cputopo->oplus_cpu_array[i][k],
					&ux_cputopo->sched_cls[j].cpus);
			k++;
		}

		/*
		 * k starts from where we left off above.
		 * Fill clusters below i in descending order.
		 */
		for (j = i - 1; j >= 0; j--) {
			cpumask_copy(&ux_cputopo->oplus_cpu_array[i][k],
					&ux_cputopo->sched_cls[j].cpus);
			k++;
		}
	}

	for (i = cls_nr; i < 2*cls_nr; i++) {
		int j, k = 1;

		/* Fill out first column with appropriate cpu arrays */
		cpumask_copy(&ux_cputopo->oplus_cpu_array[i][0],
				&ux_cputopo->sched_cls[i-cls_nr].cpus);

		/*
		 * k starts from column 1 because 0 is filled
		 * Fill clusters for the rest of the row,
		 * above i in ascending order
		 */
		for (j = i - cls_nr - 1; j >= 0; j--) {
			cpumask_copy(&ux_cputopo->oplus_cpu_array[i][k],
					&ux_cputopo->sched_cls[j].cpus);
			k++;
		}

		/*
		 * k starts from where we left off above.
		 * Fill clusters below i in descending order.
		 */
		for (j = i + 1; j < 2*cls_nr; j++) {
			cpumask_copy(&ux_cputopo->oplus_cpu_array[i][k],
					&ux_cputopo->sched_cls[j-cls_nr].cpus);
			k++;
		}
	}
}
#endif

void update_ux_sched_cputopo(void)
{
	unsigned long prev_cap = 0;
	unsigned long cpu_cap = 0;
	unsigned int cpu = 0;
	int i = 0, insert_idx = 0, cls_nr = 0;
	struct ux_sched_cluster sched_cls;

	/* reset prev cpu topo info */
	sched_init_ux_cputopo();

	/* update new cpu topo info */
	for_each_possible_cpu(cpu) {
		cpu_cap = arch_scale_cpu_capacity(cpu);
		/* add cpu with same capacity into target sched_cls */
		if (cpu_cap == prev_cap) {
			for (i = 0; i < ux_sched_cputopo.cls_nr; ++i) {
				if (cpu_cap == ux_sched_cputopo.sched_cls[i].capacity) {
					cpumask_set_cpu(cpu, &ux_sched_cputopo.sched_cls[i].cpus);
					break;
				}
			}

			continue;
		}

		cpumask_clear(&sched_cls.cpus);
		cpumask_set_cpu(cpu, &sched_cls.cpus);
		sched_cls.capacity = cpu_cap;
		cls_nr = ux_sched_cputopo.cls_nr;

		if (!cls_nr) {
			ux_sched_cputopo.sched_cls[cls_nr] = sched_cls;
		} else {
			for (i = 0; i <= ux_sched_cputopo.cls_nr; ++i) {
				if (sched_cls.capacity < ux_sched_cputopo.sched_cls[i].capacity) {
					insert_idx = i;
					break;
				}
			}
			if (insert_idx == ux_sched_cputopo.cls_nr) {
				ux_sched_cputopo.sched_cls[insert_idx] = sched_cls;
			} else {
				for (; cls_nr > insert_idx; cls_nr--)
					ux_sched_cputopo.sched_cls[cls_nr] = ux_sched_cputopo.sched_cls[cls_nr-1];

				ux_sched_cputopo.sched_cls[insert_idx] = sched_cls;
			}
		}
		ux_sched_cputopo.cls_nr++;

		prev_cap = cpu_cap;
	}
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
	build_oplus_cpu_array();
#endif
}

bool task_is_runnable(struct task_struct *task)
{
	if (!task)
		return false;

	if (READ_ONCE((task)->__state) != TASK_RUNNING)
		return false;

	return (task->on_rq && !task->on_cpu);
}

int get_ux_state(struct task_struct *task)
{
	struct oplus_task_struct *ots;

	if (!task)
		return 0;

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		return 0;

	return ots->ux_state;
}

bool is_min_cluster(int cpu)
{
	/*
	 * The package id value of the cpu in min_cluster is 0.
	 */
	return !topology_physical_package_id(cpu);
}

bool is_max_cluster(int cpu)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_nr = ux_cputopo.cls_nr;

	/*
	 * The package_id value of the cpu in mxn_cluster is the largest.
	 */
	return topology_physical_package_id(cpu) == cls_nr-1;
}

bool is_mid_cluster(int cpu)
{
	return !(is_min_cluster(cpu) || is_max_cluster(cpu));
}
EXPORT_SYMBOL(update_ux_sched_cputopo);

/* UX synchronization rules
 * 1. when task set ux first time, or alter ux state,
 *    ACQUIRE (rq->lock)         prevent task migrate between rq
 *    ACQUIRE (ux_list->lock)
 *    add task to list or change position in list
 *    RELEASE (ux_list->lock)
 *    RELEASE (rq->lock)

 * 2. when task ux -> 0, or dequeue from list
 *    ACQUIRE (rq->lock)         prevent task migrate between rq
 *    ACQUIRE (ux_list->lock)
 *    list_del_init(ux_entry)
 *    RELEASE (ux_list->lock)
 *    RELEASE (rq->lock)

 * 3. oplus_rbtree_empty(ux_list) is atomic, unnecessary to lock
 *    ux_list_first_entry(ux_list) isn't atomic, but is safe to get without lock

 * 4. oplus_rbtree_empty(ux_list) and then list_first_entry(ux_list)
 *    ACQUIRE (ux_list->lock)
 *    oplus_rbtree_empty(ux_list)
 *    list_first_entry(ux_list)
 *    RELEASE(ux_list->lock)

*/
void oplus_set_ux_state_lock(struct task_struct *t, int ux_state, bool need_lock_rq)
{
	struct rq *rq;
	struct rq_flags flags;
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	unsigned long irqflag;

	if (need_lock_rq)
		rq = task_rq_lock(t, &flags);
	else
		rq = task_rq(t);

	ots = get_oplus_task_struct(t);
	if (IS_ERR_OR_NULL(ots) || !test_task_is_fair(t) || (ux_state == ots->ux_state)) {
		goto out;
	}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	if (pipeline_task_skip_ux_change(ots, &ux_state))
		goto out;
#endif

	orq = (struct oplus_rq *) rq->android_oem_data1;
	spin_lock_irqsave(orq->ux_list_lock, irqflag);

	ots->ux_state = ux_state;

	if (!(ux_state & SCHED_ASSIST_UX_MASK)) {
		if (!oplus_rbnode_empty(&ots->ux_entry)) {
			lockdep_assert_rq_held(rq);
			update_ux_timeline_task_removal(orq, ots);
			if (task_current(rq, t) && (!oplus_rbtree_empty(&orq->ux_list))) {
				resched_curr(rq);
			}
			put_task_struct(t);
			/* make sure task is removed from the list before ux_priority set to invalid */
			smp_wmb();
		}
		ots->ux_priority = -1;
		ots->ux_nice = -1;
	} else if (task_on_rq_queued(t)) {
		bool unlinked, is_fair;
		struct task_struct *curr;
		lockdep_assert_rq_held(rq);
		unlinked = oplus_rbnode_empty(&ots->ux_entry);
		if (unlinked) {
			get_task_struct(t);
			/* when obtain ux state first time after enqueued,
			  sum_exec_baseline reset to task's curr exec runtime
			  make sure this task gain ux bonus exec time. */
			/*if (!ots->total_exec) {
				ots->sum_exec_baseline = t->se.sum_exec_runtime;
			}*/

			initial_prio_nice_and_vruntime(orq, ots, ux_state_to_priority(ux_state), ux_state_to_nice(ux_state));
			insert_task_to_ux_timeline(ots, orq);
		} else {
			update_ux_timeline_task_change(orq, ots, ux_state_to_priority(ux_state), ux_state_to_nice(ux_state));
		}
		rcu_read_lock();
		curr = rcu_dereference(rq->curr);
		is_fair = (curr != NULL) && test_task_is_fair(curr);
		rcu_read_unlock();
		if (is_fair && !task_current(rq, t) &&
			(ots == ux_list_first_entry(&orq->ux_list))) {
			resched_curr(rq);
		}
	} else {
		ots->ux_priority = ux_state_to_priority(ux_state);
		ots->ux_nice = ux_state_to_nice(ux_state);
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);

	out:
	if (need_lock_rq)
		task_rq_unlock(rq, t, &flags);
}

noinline int tracing_mark_write(const char *buf)
{
	trace_printk(buf);
	return 0;
}

void ux_state_systrace_c(unsigned int cpu, struct task_struct *p)
{
	int ux_state = (oplus_get_ux_state(p) & (SCHED_ASSIST_UX_MASK | SA_TYPE_INHERIT));

	if (per_cpu(prev_ux_state, cpu) != ux_state) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_ux_state|%d\n", cpu, ux_state);
		tracing_mark_write(buf);
		per_cpu(prev_ux_state, cpu) = ux_state;
	}
}

void ux_priority_systrace_c(unsigned int cpu, struct task_struct *t)
{
	struct rq *rq;
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	int ux_priority;
	u64 value;

	if (NULL == t) {
		return;
	}
	ots = get_oplus_task_struct(t);
	if (IS_ERR_OR_NULL(ots)) {
		return;
	}

	ux_priority = ots->ux_priority * 10 + ots->ux_nice;
	if (per_cpu(prev_ux_priority, cpu) != ux_priority) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9998|Cpu%d_ux_priority|%d\n", cpu, ux_priority);
		tracing_mark_write(buf);
		per_cpu(prev_ux_priority, cpu) = ux_priority;
	}

	value = ots->vruntime;
	if (per_cpu(prev_vruntime, cpu) != value) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9998|Cpu%d_vruntime|%llu\n", cpu, value);
		tracing_mark_write(buf);
		per_cpu(prev_vruntime, cpu) = value;
	}

	rq = cpu_rq(cpu);
	orq = (struct oplus_rq *) rq->android_oem_data1;
	value = orq->min_vruntime;
	if (per_cpu(prev_min_vruntime, cpu) != value) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9998|Cpu%d_min_vruntime|%llu\n", cpu, value);
		tracing_mark_write(buf);
		per_cpu(prev_min_vruntime, cpu) = value;
	}

	value = ots->preset_vruntime;
	if (per_cpu(prev_preset_vruntime, cpu) != value) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9998|Cpu%d_preset_vtime|%llu\n", cpu, value);
		tracing_mark_write(buf);
		per_cpu(prev_preset_vruntime, cpu) = value;
	}
}

void sa_scene_systrace_c(void)
{
	static int prev_ux_scene = 0;
	int assist_scene = global_sched_assist_scene;
	if (prev_ux_scene != assist_scene) {
		char buf[64];

		snprintf(buf, sizeof(buf), "C|9999|Ux_Scene|%d\n", global_sched_assist_scene);
		tracing_mark_write(buf);
		prev_ux_scene = assist_scene;
	}
}


void hwbinder_systrace_c(unsigned int cpu, int flag)
{
	if (per_cpu(prev_hwbinder_flag, cpu) != flag) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_hwbinder|%d\n", cpu, flag);
		tracing_mark_write(buf);
		per_cpu(prev_hwbinder_flag, cpu) = flag;
	}
}

void sched_assist_init_oplus_rq(void)
{
	int cpu;
	struct oplus_rq *orq;

	for_each_possible_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);

		if (!rq) {
			ux_err("failed to init oplus rq(%d)", cpu);
			continue;
		}
		orq = (struct oplus_rq *) rq->android_oem_data1;
		orq->ux_list = RB_ROOT_CACHED;
		orq->exec_timeline = RB_ROOT_CACHED;
		orq->ux_list_lock = kmalloc(sizeof(spinlock_t), GFP_KERNEL);
		spin_lock_init(orq->ux_list_lock);
		orq->nr_running = 0;
		orq->min_vruntime = 0;
		orq->load_weight = 0;
#ifdef CONFIG_LOCKING_PROTECT
		INIT_LIST_HEAD(&orq->locking_thread_list);
		orq->locking_list_lock = kmalloc(sizeof(spinlock_t), GFP_KERNEL);
		spin_lock_init(orq->locking_list_lock);
		orq->rq_locking_task = 0;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_LOADBALANCE)
		orq->lb.pid = INVALID_PID;
#endif

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
		per_cpu(task_lb_count, cpu_of(rq)).ux_low = 0;
		per_cpu(task_lb_count, cpu_of(rq)).ux_high = 0;
		per_cpu(task_lb_count, cpu_of(rq)).top_low = 0;
		per_cpu(task_lb_count, cpu_of(rq)).top_high = 0;
		per_cpu(task_lb_count, cpu_of(rq)).foreground_low = 0;
		per_cpu(task_lb_count, cpu_of(rq)).foreground_high = 0;
		per_cpu(task_lb_count, cpu_of(rq)).background_low = 0;
		per_cpu(task_lb_count, cpu_of(rq)).background_high = 0;
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
	}

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
	cpumask_clear(&nr_mask);
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
}

bool is_task_util_over(struct task_struct *tsk, int threshold)
{
	bool sum_over = false;
	bool demand_over = false;

#if IS_ENABLED(CONFIG_SCHED_WALT)
	sum_over = scale_demand(task_wts_sum(tsk)) >= threshold;
	demand_over = oplus_task_util(tsk) >= threshold;
#else
	sum_over = tsk->se.avg.util_avg >= threshold;
#endif

	return sum_over || demand_over;
}

static inline bool oplus_is_min_capacity_cpu(int cpu)
{
	struct ux_sched_cputopo ux_cputopo = ux_sched_cputopo;
	int cls_nr = ux_cputopo.cls_nr - 1;

	if (unlikely(cls_nr <= 0))
		return false;

	return capacity_orig_of(cpu) <= ux_cputopo.sched_cls[0].capacity;
}

bool oplus_task_misfit(struct task_struct *tsk, int cpu)
{
	if (is_task_util_over(tsk, BOOST_THRESHOLD_UNIT) && oplus_is_min_capacity_cpu(cpu))
		return true;

	return false;
}

inline bool test_task_is_fair(struct task_struct *task)
{
	DEBUG_BUG_ON(!task);

	/* valid CFS priority is MAX_RT_PRIO..MAX_PRIO-1 */
	if ((task->prio >= MAX_RT_PRIO) && (task->prio <= MAX_PRIO-1))
		return true;
	return false;
}

inline bool test_task_is_rt(struct task_struct *task)
{
	DEBUG_BUG_ON(!task);

	/* valid RT priority is 0..MAX_RT_PRIO-1 */
	if ((task->prio >= 0) && (task->prio <= MAX_RT_PRIO-1))
		return true;

	return false;
}
EXPORT_SYMBOL_GPL(test_task_is_rt);

unsigned int ux_task_exec_limit(struct task_struct *p)
{
	int ux_state = oplus_get_ux_state(p);
	unsigned int exec_limit = UX_EXEC_SLICE;

	if (sched_assist_scene(SA_LAUNCH) && !(ux_state & SA_TYPE_INHERIT)) {
		exec_limit *= 30;
		return exec_limit;
	}

	if (ux_state & SA_TYPE_SWIFT)
		exec_limit *= 2;
	else if (ux_state & SA_TYPE_ANIMATOR)
		exec_limit *= 12;
	else if (ux_state & SA_TYPE_LIGHT)
		exec_limit *= 3;
	else if (ux_state & SA_TYPE_HEAVY)
		exec_limit *= 25;
	else if (ux_state & SA_TYPE_LISTPICK)
		exec_limit *= 30;

	return exec_limit;
}
EXPORT_SYMBOL_GPL(ux_task_exec_limit);

/* identify ux only opt in some case, but always keep it's id_type, and wont do inherit through test_task_ux() */
bool test_task_identify_ux(struct task_struct *task, int id_type_ux)
{
	return false;
}

inline bool test_list_pick_ux(struct task_struct *task)
{
	return false;
}

bool test_task_ux(struct task_struct *task)
{
	if (unlikely(!global_sched_assist_enabled))
		return false;

	if (!task)
		return false;

	if (!test_task_is_fair(task))
		return false;

	if (oplus_get_ux_state(task) & SCHED_ASSIST_UX_MASK) {
		struct oplus_task_struct *ots;
		unsigned int limit;

		ots = get_oplus_task_struct(task);
		if (IS_ERR_OR_NULL(ots))
			return false;

		limit = ux_task_exec_limit(task);
		if (ots->total_exec && (ots->total_exec > limit)) {
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("task is not ux by limit, comm=%-12s pid=%d ux_state=%d total_exec=%llu limit=%llu\n",
					task->comm, task->pid, ots->ux_state, ots->total_exec, limit);

			return false;
		}

		return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(test_task_ux);

int get_ux_state_type(struct task_struct *task)
{
	if (!task)
		return UX_STATE_INVALID;

	if (!test_task_is_fair(task))
		return UX_STATE_INVALID;

	if (oplus_get_ux_state(task) & SA_TYPE_INHERIT)
		return UX_STATE_INHERIT;

	if (oplus_get_ux_state(task) & SCHED_ASSIST_UX_MASK)
		return UX_STATE_SCHED_ASSIST;

	return UX_STATE_NONE;
}
EXPORT_SYMBOL_GPL(get_ux_state_type);

/* check if a's ux prio higher than b's prio */
bool prio_higher(int a, int b)
{
	int a_priority = a & SCHED_ASSIST_UX_PRIORITY_MASK;
	int b_priority = b & SCHED_ASSIST_UX_PRIORITY_MASK;

	if (a_priority != b_priority)
		return (a_priority > b_priority);

	if (a & SA_TYPE_SWIFT)
		return !(b & SA_TYPE_SWIFT);

	if (a & SA_TYPE_ANIMATOR)
		return !(b & SA_TYPE_ANIMATOR);

	if (a & SA_TYPE_LIGHT)
		return !(b & (SA_TYPE_ANIMATOR | SA_TYPE_LIGHT | SA_TYPE_SWIFT));

	if (a & SA_TYPE_HEAVY)
		return !(b & (SA_TYPE_ANIMATOR | SA_TYPE_LIGHT | SA_TYPE_HEAVY | SA_TYPE_SWIFT));

	/* SA_TYPE_LISTPICK */
	return false;
}

/*s64 __maybe_unused account_ux_runtime(struct rq *rq, struct task_struct *curr)
{
	struct oplus_rq *orq = (struct oplus_rq *) rq->android_oem_data1;
	struct oplus_task_struct *ots = get_oplus_task_struct(curr);
	s64 delta;
	unsigned int limit;

	if (IS_ERR_OR_NULL(ots))
		return;

	lockdep_assert_rq_held(rq);

	if (!(rq->clock_update_flags & RQCF_UPDATED))
		update_rq_clock(rq);

	delta = curr->se.sum_exec_runtime - ots->sum_exec_baseline;

	if (delta < 0)
		delta = 0;
	else
		delta += rq_clock_task(rq) - curr->se.exec_start;

	if (delta < CFS_SCHED_MIN_GRAN)
		return delta;

	ots->sum_exec_baseline += delta;
	ots->total_exec += delta;

	ots->vruntime += calc_delta_fair(delta, ots->ux_priority);

	limit = ux_task_exec_limit(curr);
	if (ots->total_exec > limit) {
		remove_ux_task(orq, ots);
		put_task_struct(curr);
	} else {
		// if ux slice has expired but total exectime not, just requeue without put/get task_struct
		remove_ux_task(orq, ots);
		insert_task_to_ux_timeline(ots, orq);
	}

	return delta;
}*/

static void enqueue_ux_thread(struct rq *rq, struct task_struct *p)
{
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	unsigned long irqflag;

	if (unlikely(!global_sched_assist_enabled))
		return;

	oplus_set_enqueue_time(p, rq->clock);
#ifdef CONFIG_OPLUS_CPU_AUDIO_PERF
	oplus_sched_assist_audio_enqueue_hook(p);
#endif

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	pipeline_task_enqueue(ots);
#endif

	if (!test_task_is_fair(p) || !oplus_rbnode_empty(&ots->ux_entry))
		return;

	orq = (struct oplus_rq *) rq->android_oem_data1;
	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	if (!oplus_rbnode_empty(&ots->ux_entry)) {
		DEBUG_BUG_ON(1);
		spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
		return;
	}

	/* task's ux entry should be initialized with INIT_LIST_HEAD() */
	/*if (ots->ux_entry.__rb_parent_color == 0)
		RB_CLEAR_NODE(&ots->ux_entry);*/

	if (test_task_ux(p)) {
		get_task_struct(p);
		if (!ots->total_exec) {
			int ux_priority, ux_nice;
			/* ots->sum_exec_baseline = p->se.sum_exec_runtime; */
			ux_priority = ux_state_to_priority(ots->ux_state);
			ux_nice = ux_state_to_nice(ots->ux_state);
			initial_prio_nice_and_vruntime(orq, ots, ux_priority, ux_nice);
		} else {
			update_vruntime_task_attach(orq, ots);
		}
		insert_task_to_ux_timeline(ots, orq);
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
}

static void dequeue_ux_thread(struct rq *rq, struct task_struct *p)
{
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	unsigned long irqflag;

	if (!rq || !p)
		return;

	oplus_set_enqueue_time(p, 0);
	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	orq = (struct oplus_rq *) rq->android_oem_data1;
	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	if (!oplus_rbnode_empty(&ots->ux_entry)) {
		u64 now = jiffies_to_nsecs(jiffies);

		update_ux_timeline_task_removal(orq, ots);

		/* inherit ux can only keep it's ux state in MAX_INHERIT_GRAN(64 ms) */
		if (get_ux_state_type(p) == UX_STATE_INHERIT && (now - ots->inherit_ux_start > MAX_INHERIT_GRAN)) {
			atomic64_set(&ots->inherit_ux, 0);
			ots->ux_depth = 0;
			ots->ux_state = 0;
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("dequeue and unset inherit ux task=%-12s pid=%d tgid=%d now=%llu inherit_start=%llu\n",
					p->comm, p->pid, p->tgid, now, ots->inherit_ux_start);
		}

		if (ots->ux_state & SA_TYPE_ONCE) {
			atomic64_set(&ots->inherit_ux, 0);
			ots->ux_depth = 0;
			ots->ux_state = 0;
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("dequeue and unset once ux task=%-12s pid=%d tgid=%d now=%llu inherit_start=%llu\n",
					p->comm, p->pid, p->tgid, now, ots->inherit_ux_start);
		}
		put_task_struct(p);
	}

	if (p->__state != TASK_RUNNING) {
		ots->total_exec = 0;
		ots->vruntime = 0;
		ots->preset_vruntime = 0;
	} else {
		update_vruntime_task_detach(orq, ots);
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
}

void queue_ux_thread(struct rq *rq, struct task_struct *p, int enqueue)
{
	if (enqueue)
		enqueue_ux_thread(rq, p);
	else
		dequeue_ux_thread(rq, p);
}
EXPORT_SYMBOL(queue_ux_thread);

bool test_inherit_ux(struct task_struct *task, int type)
{
	u64 inherit_ux;

	if (!task)
		return false;

	inherit_ux = oplus_get_inherit_ux(task);
	return inherit_ux_get_bits(inherit_ux, type) > 0;
}
EXPORT_SYMBOL_GPL(test_inherit_ux);

inline void inherit_ux_inc(struct task_struct *task, int type)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return;

	atomic64_add(inherit_ux_value(type, 1), &ots->inherit_ux);
}

inline void inherit_ux_sub(struct task_struct *task, int type, int value)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return;

	atomic64_sub(inherit_ux_value(type, value), &ots->inherit_ux);
}

void inc_inherit_ux_refs(struct task_struct *task, int type)
{
	inherit_ux_inc(task, type);
}
EXPORT_SYMBOL_GPL(inc_inherit_ux_refs);

inline bool test_task_ux_depth(int ux_depth)
{
	return ux_depth < UX_DEPTH_MAX;
}

bool test_set_inherit_ux(struct task_struct *tsk)
{
	int ux_depth = oplus_get_ux_depth(tsk);

	return tsk && test_task_ux(tsk) && test_task_ux_depth(ux_depth);
}
EXPORT_SYMBOL_GPL(test_set_inherit_ux);

void set_inherit_ux(struct task_struct *task, int type, int depth, int inherit_val)
{
	if (!task || type >= INHERIT_UX_MAX)
		return;

	if (!test_task_is_fair(task)) {
		return;
	}

	inherit_ux_inc(task, type);
	oplus_set_ux_depth(task, depth + 1);

	if (inherit_val & SA_TYPE_LISTPICK) {
		inherit_val &= (~SA_TYPE_LISTPICK);
		inherit_val |= SA_TYPE_HEAVY;
	}
	oplus_set_ux_state_lock(task, (inherit_val & SCHED_ASSIST_UX_MASK) | SA_TYPE_INHERIT, true);
	oplus_set_inherit_ux_start(task, jiffies_to_nsecs(jiffies));
	trace_inherit_ux_set(task, type, oplus_get_ux_state(task), oplus_get_inherit_ux(task), oplus_get_ux_depth(task));
}
EXPORT_SYMBOL_GPL(set_inherit_ux);

void reset_inherit_ux(struct task_struct *inherit_task, struct task_struct *ux_task, int reset_type)
{
	int reset_depth = 0;
	int reset_inherit = 0;
	int ux_state;

	if (!inherit_task || !ux_task || reset_type >= INHERIT_UX_MAX)
		return;

	reset_inherit = oplus_get_ux_state(ux_task);
	reset_depth = oplus_get_ux_depth(ux_task);

	if (!test_inherit_ux(inherit_task, reset_type) || !(reset_inherit & SA_TYPE_ANIMATOR))
		return;

	ux_state = (oplus_get_ux_state(inherit_task) & ~SCHED_ASSIST_UX_MASK) | reset_inherit;
	oplus_set_ux_depth(inherit_task, reset_depth + 1);
	oplus_set_ux_state_lock(inherit_task, ux_state, true);
	trace_inherit_ux_reset(inherit_task, reset_type, oplus_get_ux_state(inherit_task),
		oplus_get_inherit_ux(inherit_task), oplus_get_ux_depth(inherit_task));
}
EXPORT_SYMBOL_GPL(reset_inherit_ux);

void unset_inherit_ux_value(struct task_struct *task, int type, int value)
{
	s64 inherit_ux;
	struct oplus_task_struct *ots;

	if (!task || type >= INHERIT_UX_MAX)
		return;

	inherit_ux_sub(task, type, value);
	inherit_ux = oplus_get_inherit_ux(task);

	if (inherit_ux > 0) {
		return;
	}

	if (inherit_ux < 0)
		oplus_set_inherit_ux(task, 0);

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		return;

	ots->ux_depth = 0;
	oplus_set_ux_state_lock(task, 0, true);

	trace_inherit_ux_unset(task, type, oplus_get_ux_state(task), oplus_get_inherit_ux(task), oplus_get_ux_depth(task));
}
EXPORT_SYMBOL_GPL(unset_inherit_ux_value);

void unset_inherit_ux(struct task_struct *task, int type)
{
	unset_inherit_ux_value(task, type, 1);
}
EXPORT_SYMBOL_GPL(unset_inherit_ux);

bool endwith(char *str, char *s)
{
	int len1 = strlen(str), len2 = strlen(s);
	if (len1 < len2)
		return false;
	return !strcmp(str + len1 - len2, s);
}

bool is_ui_thread(char *comm, struct task_struct *p)
{
	if (strcmp(p->group_leader->comm, "m.taobao.taobao"))
		return false;

	return isdigit(*comm) && endwith(comm, ".ui");
}

bool im_mali(char *comm)
{
	return !strcmp(comm, "mali-event-hand") ||
		!strcmp(comm, "mali-mem-purge") || !strcmp(comm, "mali-cpu-comman") ||
		!strcmp(comm, "mali-compiler");
}

bool cgroup_check_set_sched_assist_boost(char *comm, struct task_struct *p)
{
	return im_mali(comm) || is_ui_thread(comm, p);
}

void cgroup_set_sched_assist_boost_task(struct task_struct *p, char *comm)
{
	int ux_state;

	if (cgroup_check_set_sched_assist_boost(comm, p)) {
		ux_state = oplus_get_ux_state(p);
		/* maybe the ux state is set by fwk hook, not oomadj */
		if (is_top(p) || test_task_ux(p->group_leader)) {
			if (oplus_get_im_flag(p->group_leader) == IM_FLAG_LAUNCHER)
				oplus_set_ux_state_lock(p, (ux_state | SA_TYPE_LIGHT), true);
			else
				oplus_set_ux_state_lock(p, (ux_state | SA_TYPE_HEAVY), true);
		} else
			oplus_set_ux_state_lock(p, (ux_state & ~SA_TYPE_HEAVY), true);
	}
}

void clear_all_inherit_type(struct task_struct *p)
{
	struct oplus_task_struct *ots;

	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	atomic64_set(&ots->inherit_ux, 0);
	ots->ux_depth = 0;
	oplus_set_ux_state_lock(p, 0, true);
}

void sched_assist_target_comm(struct task_struct *task, const char *buf)
{
	char comm[128];
	int ux_state;

	strlcpy(comm, buf, sizeof(comm));

#ifdef CONFIG_LOCKING_PROTECT
	if (locking_protect_disable == false) {
		if (strstr(buf, "kernel_net_tes")) {
			locking_protect_disable = true;
			locking_wakeup_preepmt_enable = 0;
		}
	}
#endif

	/*set mali-event-handle ux when task fork*/
	cgroup_set_sched_assist_boost_task(task, comm);

	/*set audio task ux in bilibili*/
	if (task->group_leader &&
		unlikely(strcmp(task->group_leader->comm, "tv.danmaku.bili"))) {
		if (!strncmp(comm, "ff_audio_dec", 12)) {
			ux_state = oplus_get_ux_state(task);
			oplus_set_ux_state_lock(task, (ux_state | SA_TYPE_LIGHT), true);
		}
	}

	if (!strncmp(buf, "HwBinder", 8))
		oplus_set_im_flag(task, IM_FLAG_HWBINDER);

	/* set audio task ux in com.autonavi.minimap */
	if (!strncmp(comm, "AudioOutputDevi", 15) && task->group_leader &&
		strstr(task->group_leader->comm, "vilege_process0")) {
		ux_state = oplus_get_ux_state(task);
		oplus_set_ux_state_lock(task, (ux_state | SA_TYPE_LIGHT), true);
	}
}

void adjust_rt_lowest_mask(struct task_struct *p, struct cpumask *local_cpu_mask, int ret, bool force_adjust)
{
	struct cpumask mask_backup;
	int next_cpu = -1;
	int keep_target_cpu = -1;
	int keep_backup_cpu = -1;
	int lowest_prio = INT_MIN;
	unsigned int drop_cpu;
	struct rq *rq;
	struct task_struct *task;
	struct oplus_rq *orq;
	struct oplus_task_struct *ots = NULL;
	unsigned long irqflag;

	if (!ret || !local_cpu_mask || cpumask_empty(local_cpu_mask))
		return;

	cpumask_copy(&mask_backup, local_cpu_mask);

	drop_cpu = cpumask_first(local_cpu_mask);
	while (drop_cpu < nr_cpu_ids) {
		int rt_task_im_flag;
		int ux_task_state;

		/* unlocked access */
		rq = cpu_rq(drop_cpu);
		orq = (struct oplus_rq *) rq->android_oem_data1;
		task = rcu_dereference(rq->curr);

		if (!task || (task->flags & PF_EXITING)) {
			drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
			continue;
		}

		spin_lock_irqsave(orq->ux_list_lock, irqflag);
		if (!test_task_ux(task)) {
			if (oplus_rbtree_empty(&orq->ux_list)) {
				spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
				drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
				continue;
			} else {
				ots =  ux_list_first_entry(&orq->ux_list);
				if (!IS_ERR_OR_NULL(ots)) {
					task = ots->task;
					/*
					 * if PF_EXITING, the task will be free later so that it doesn't need to be preempt-protect,
					 * In the meantime, it will result to panic when calling ots or task_struct after free them.
					 * */
					if (!task || (task->flags & PF_EXITING)) {
						spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
						drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
						continue;
					}
				} else {
					spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
					drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
					continue;
				}
			}
		}
		ux_task_state = oplus_get_ux_state(task);
		spin_unlock_irqrestore(orq->ux_list_lock, irqflag);

		rt_task_im_flag = oplus_get_im_flag(p);
		/* avoid sf premmpt heavy ux tasks,such as ui, render... */
		if ((rt_task_im_flag == IM_FLAG_SURFACEFLINGER || rt_task_im_flag == IM_FLAG_RENDERENGINE) &&
			((ux_task_state & SA_TYPE_HEAVY) || (ux_task_state & SA_TYPE_LISTPICK))) {
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("clear cpu from lowestmask, curr_heavy task=%-12s pid=%d drop_cpu=%d\n", task->comm, task->pid, drop_cpu);
		}

#ifdef CONFIG_OPLUS_SCHED_MT6895
		if (ux_task_state & SA_TYPE_HEAVY) {
#else
		if (sched_assist_scene(SA_LAUNCH) && (ux_task_state & SA_TYPE_HEAVY)) {
#endif
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("clear cpu from lowestmask, curr_heavy task=%-12s pid=%d drop_cpu=%d\n", task->comm, task->pid, drop_cpu);
		}

		if (ux_task_state & SA_TYPE_ANIMATOR) {
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("clear cpu from lowestmask, curr_anima task=%-12s pid=%d drop_cpu=%d\n", task->comm, task->pid, drop_cpu);
		}

#ifdef CONFIG_OPLUS_CPU_AUDIO_PERF
		if (oplus_sched_assist_audio_perf_check_exit_latency(p, drop_cpu))
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
#endif

		drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
	}

	if (likely(!cpumask_empty(local_cpu_mask)))
		return;

	if (unlikely(global_debug_enabled & DEBUG_FTRACE))
		trace_printk("lowest mask is empty, force is %d\n", force_adjust);

	/* We may get empty local_cpu_mask if we do unsuitable drop work */
	if (!force_adjust) {
		cpumask_copy(local_cpu_mask, &mask_backup);
		return;
	}

	next_cpu = cpumask_first(&mask_backup);
	while (next_cpu < nr_cpu_ids) {
		/* unlocked access */
		struct task_struct *task;
		bool is_target;
		int prio;

		rq = cpu_rq(next_cpu);
		task = rcu_dereference(rq->curr);

		/*
		* if PF_EXITING, the task will be free later so that it doesn't need to be preempt-protect,
		* In the meantime, it will result to panic when calling ots or task_struct after free them.
		* */
		if (!task || (task->flags & PF_EXITING)) {
			next_cpu = cpumask_next(next_cpu, &mask_backup);
			continue;
		}

		is_target = !(oplus_get_ux_state(task) & SA_TYPE_ANIMATOR);
		prio = task->prio;

		if (lowest_prio == INT_MIN) {
			if (is_target)
				keep_target_cpu = next_cpu;
			else
				keep_backup_cpu = next_cpu;

			lowest_prio = prio;
		} else if (is_target && prio > lowest_prio) {
			keep_target_cpu = next_cpu;
			lowest_prio = prio;
		} else if (!is_target && prio > lowest_prio) {
			keep_backup_cpu = next_cpu;
			lowest_prio = task->prio;
		}

		next_cpu = cpumask_next(next_cpu, &mask_backup);
	}

	if (keep_target_cpu != -1)
		cpumask_set_cpu(keep_target_cpu, local_cpu_mask);
	else if (keep_backup_cpu != -1)
		cpumask_set_cpu(keep_backup_cpu, local_cpu_mask);
}
EXPORT_SYMBOL(adjust_rt_lowest_mask);

bool sa_skip_rt_sync(struct rq *rq, struct task_struct *p, bool *sync)
{
	int cpu = cpu_of(rq);
	struct oplus_rq *orq = (struct oplus_rq *) rq->android_oem_data1;
	struct oplus_task_struct *ots;
	unsigned long irqflag;

	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	ots = ux_list_first_entry(&orq->ux_list);
	if (IS_ERR_OR_NULL(ots) || (ots->im_flag == IM_FLAG_CAMERA_HAL)) {
		spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
		return false;
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);

	if (*sync) {
		*sync = false;
		if (unlikely(global_debug_enabled & DEBUG_FTRACE))
			trace_printk("comm=%-12s pid=%d cpu=%d\n", p->comm, p->pid, cpu);

		return true;
	}

	return false;
}
EXPORT_SYMBOL(sa_skip_rt_sync);

bool sa_rt_skip_ux_cpu(int cpu)
{
	struct rq *rq;
	struct oplus_rq *orq;
	struct task_struct *curr;

	rq = cpu_rq(cpu);
	orq = (struct oplus_rq *) rq->android_oem_data1;
	curr = rq->curr;

	/* skip running ux */
	if (curr && test_task_ux(curr))
		return true;

	/* skip runnable ux */
	if (!oplus_rbtree_empty(&orq->ux_list))
		return true;

	return false;
}
EXPORT_SYMBOL(sa_rt_skip_ux_cpu);

ssize_t oplus_show_cpus(const struct cpumask *mask, char *buf)
{
	ssize_t i = 0;
	unsigned int cpu;

	for_each_cpu(cpu, mask) {
		if (i)
			i += scnprintf(&buf[i], (PAGE_SIZE - i - 2), " ");
		i += scnprintf(&buf[i], (PAGE_SIZE - i - 2), "%u", cpu);
		if (i >= (PAGE_SIZE - 5))
			break;
	}
	i += sprintf(&buf[i], "\n");
	return i;
}

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
void sa_spread_systrace_c(void)
{
	static int prev_ux_spread = 0;
	int ux_spread = should_force_spread_tasks();

	if (prev_ux_spread != ux_spread) {
		char buf[32];
		snprintf(buf, sizeof(buf), "C|9999|Ux_Spread|%d\n", ux_spread);
		tracing_mark_write(buf);
		prev_ux_spread = ux_spread;
	}
}
#endif

/* register vender hook in kernel/sched/topology.c */
void android_vh_build_sched_domains_handler(void *unused, bool has_asym)
{
	update_ux_sched_cputopo();
}

/* register vender hook in  kernel/sched/rt.c */
void android_rvh_select_task_rq_rt_handler(void *unused,
			struct task_struct *p, int prev_cpu, int sd_flag, int wake_flags, int *new_cpu)
{
}

void android_rvh_find_lowest_rq_handler(void *unused,
			struct task_struct *p, struct cpumask *local_cpu_mask, int ret, int *best_cpu)
{
	adjust_rt_lowest_mask(p, local_cpu_mask, ret, true);

	if (!ret || !local_cpu_mask)
		return;

	if (cpumask_empty(local_cpu_mask))
		return;

	if (cpumask_test_cpu(task_cpu(p), local_cpu_mask))
		*best_cpu = task_cpu(p);
	else
		*best_cpu = cpumask_first(local_cpu_mask);
}

/* register vender hook in kernel/sched/core.c */
void android_rvh_sched_fork_handler(void *unused, struct task_struct *p)
{
	init_task_ux_info(p);
}

void android_rvh_enqueue_task_handler(void *unused, struct rq *rq, struct task_struct *p, int flags)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	jankinfo_android_rvh_enqueue_task_handler(unused, rq, p, flags);
#endif
	queue_ux_thread(rq, p, 1);
}
EXPORT_SYMBOL(android_rvh_enqueue_task_handler);

void android_rvh_dequeue_task_handler(void *unused, struct rq *rq, struct task_struct *p, int flags)
{
	queue_ux_thread(rq, p, 0);
}
EXPORT_SYMBOL(android_rvh_dequeue_task_handler);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
android_rvh_schedule_handler_t fbg_android_rvh_schedule_callback;
EXPORT_SYMBOL(fbg_android_rvh_schedule_callback);
#endif

void android_rvh_schedule_handler(void *unused, struct task_struct *prev, struct task_struct *next, struct rq *rq)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
	jankinfo_android_rvh_schedule_handler(unused, prev, next, rq);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (fbg_android_rvh_schedule_callback)
		fbg_android_rvh_schedule_callback(prev, next, rq);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_PIPELINE)
	if (likely(prev != next))
		pipeline_task_switch_out(prev);
#endif

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && likely(prev != next)) {
		ux_state_systrace_c(cpu_of(rq), next);
	}

#ifdef CONFIG_LOCKING_PROTECT
	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && likely(prev != next))
		locking_state_systrace_c(cpu_of(rq), next);
#endif
}

void android_vh_scheduler_tick_handler(void *unused, struct rq *rq)
{
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
	update_rq_nr_imbalance(smp_processor_id());
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE)) {
		if (cpu_of(rq) == 0) {
			sa_scene_systrace_c();
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
			sa_spread_systrace_c();
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
		}
#if (SA_DEBUG_ON >= 1)
		if (cpu_of(rq) == 1) {
			ux_priority_systrace_c(cpu_of(rq), rq->curr);
		}
#endif
	}
}

static int boost_kill = 1;
module_param_named(boost_kill, boost_kill, uint, 0644);
#ifdef CONFIG_LOCKING_PROTECT
module_param_named(locking_wakeup_preepmt_enable, locking_wakeup_preepmt_enable, uint, 0644);
#endif
int get_grp(struct task_struct *p)
{
	struct cgroup_subsys_state *css;

	if (p == NULL)
		return false;

	rcu_read_lock();
	css = task_css(p, cpu_cgrp_id);
	if (!css) {
		rcu_read_unlock();
		return false;
	}
	rcu_read_unlock();

	return css->id;
}

static inline void do_boost_kill_task(struct task_struct *p)
{
	cpumask_var_t boost_mask;
	int is_32bit = test_ti_thread_flag(&p->thread_info, TIF_32BIT);

	set_user_nice(p, -20);
	if (is_32bit)
		cpumask_and(boost_mask, cpu_active_mask, system_32bit_el0_cpumask());
	else
		cpumask_copy(boost_mask, cpu_active_mask);
	if (!cpumask_empty(boost_mask)) {
		cpumask_copy(&p->cpus_mask, boost_mask);
		p->nr_cpus_allowed = cpumask_weight(boost_mask);
	}

}

void android_vh_exit_signal_handler(void *unused, struct task_struct *p)
{
	if (p == NULL)
		return;

	if (boost_kill && get_grp(p) == BGAPP) {
		do_boost_kill_task(p);
	}
}

static int process_exit_notifier(struct notifier_block *self,
			unsigned long cmd, void *v)
{
	struct task_struct *p = v;

	/* only boost background tasks */
	if (boost_kill && get_grp(p) == BGAPP) {
		rcu_read_lock();
		do_boost_kill_task(p);
		rcu_read_unlock();
	}

	return NOTIFY_OK;
}


struct notifier_block process_exit_notifier_block = {
	.notifier_call	= process_exit_notifier,
};

void android_vh_cgroup_set_task_handler(void *unused, int ret, struct task_struct *task)
{
	struct task_struct *p;

	if (!ret) {
		rcu_read_lock();
		if (task == task->group_leader) {
			p = task;
			do {
				if (unlikely(im_mali(task->comm))) {
					cgroup_set_sched_assist_boost_task(task, task->comm);
					break;
				}
			} while_each_thread(p, task);
		}
		rcu_read_unlock();
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_BAN_APP_SET_AFFINITY)
void android_vh_sched_setaffinity_early_handler(void *unused, struct task_struct *task, const struct cpumask *new_mask, int *skip)
{
	int im_flag = IM_FLAG_NONE;
	int curr_uid = current_uid().val;

	if ((curr_uid < FIRST_APPLICATION_UID) || (curr_uid > LAST_APPLICATION_UID))
		return;

	if (task->pid == task->tgid) {
		im_flag = oplus_get_im_flag(task);
	} else {
		struct task_struct *main_task;

		rcu_read_lock();
		main_task = find_task_by_vpid(task->tgid);
		if (main_task)
			im_flag = oplus_get_im_flag(main_task);
		rcu_read_unlock();
	}

	if (im_flag == IM_FLAG_FORBID_SET_CPU_AFFINITY)
		*skip = 1;
}
#endif

