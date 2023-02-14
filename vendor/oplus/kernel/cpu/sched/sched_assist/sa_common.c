// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
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
#include <asm/cpufeature.h>

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
#include <linux/cpuhotplug.h>
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_CPU_JANKINFO)
#include "sa_jankinfo.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_group.h>
#endif

#include "sched_assist.h"
#include "sa_common.h"

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

DEFINE_PER_CPU(struct list_head, ux_thread_list);

/* debug print frequency limit */
static DEFINE_PER_CPU(int, prev_ux_state);
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
}

static noinline int tracing_mark_write(const char *buf)
{
	trace_printk(buf);
	return 0;
}

void ux_state_systrace_c(unsigned int cpu, struct task_struct *p)
{
	int ux_state = oplus_get_ux_state(p);
	if (per_cpu(prev_ux_state, cpu) != ux_state) {
		char buf[256];
		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_ux_state|%d\n", cpu, ux_state & SCHED_ASSIST_UX_MASK);
		tracing_mark_write(buf);
		per_cpu(prev_ux_state, cpu) = ux_state;
	}
}

void fbg_state_systrace_c(unsigned int cpu, struct task_struct *p)
{
	char buf[256];
	struct oplus_task_struct *ots = get_oplus_task_struct(p);

	snprintf(buf, sizeof(buf), "C|10000|Cpu%d_fbg_state|%d\n", cpu, ots->fbg_state);
	tracing_mark_write(buf);
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
		INIT_LIST_HEAD(&orq->ux_list);
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
	BUG_ON(!task);

	/* valid CFS priority is MAX_RT_PRIO..MAX_PRIO-1 */
	if ((task->prio >= MAX_RT_PRIO) && (task->prio <= MAX_PRIO-1))
		return true;
	return false;
}

inline bool test_task_is_rt(struct task_struct *task)
{
	BUG_ON(!task);

	/* valid RT priority is 0..MAX_RT_PRIO-1 */
	if ((task->prio >= 0) && (task->prio <= MAX_RT_PRIO-1))
		return true;

	return false;
}
EXPORT_SYMBOL_GPL(test_task_is_rt);

static unsigned int ux_task_exec_limit(struct task_struct *p)
{
	int ux_state = oplus_get_ux_state(p);
	unsigned int exec_limit = UX_EXEC_SLICE;

	if (sched_assist_scene(SA_LAUNCH) && !(ux_state & SA_TYPE_INHERIT)) {
		exec_limit *= 25;
		return exec_limit;
	}

	if (ux_state & SA_TYPE_ANIMATOR)
		exec_limit *= 8;
	else if (ux_state & SA_TYPE_LIGHT)
		exec_limit *= 2;
	else if (ux_state & SA_TYPE_HEAVY)
		exec_limit *= 8;
	else if (ux_state & SA_TYPE_LISTPICK)
		exec_limit *= 25;

	return exec_limit;
}

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

	if (oplus_get_ux_state(task) & (SA_TYPE_HEAVY | SA_TYPE_LIGHT | SA_TYPE_ANIMATOR | SA_TYPE_LISTPICK)) {
		struct oplus_task_struct *ots = get_oplus_task_struct(task);
		unsigned int limit = ux_task_exec_limit(task);

		if (IS_ERR_OR_NULL(ots))
			return false;

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

	if (oplus_get_ux_state(task) & (SA_TYPE_HEAVY | SA_TYPE_LIGHT | SA_TYPE_ANIMATOR | SA_TYPE_LISTPICK))
		return UX_STATE_SCHED_ASSIST;

	return UX_STATE_NONE;
}
EXPORT_SYMBOL_GPL(get_ux_state_type);

/* check if a's ux prio higher than b's prio */
bool prio_higher(int a, int b)
{
	if (a & SA_TYPE_ANIMATOR)
		return !(b & SA_TYPE_ANIMATOR);

	if (a & SA_TYPE_LIGHT)
		return !(b & (SA_TYPE_ANIMATOR | SA_TYPE_LIGHT));

	if (a & SA_TYPE_HEAVY)
		return !(b & (SA_TYPE_ANIMATOR | SA_TYPE_LIGHT | SA_TYPE_HEAVY));

	/* SA_TYPE_LISTPICK */
	return false;
}

static void insert_ux_task_into_list(struct oplus_task_struct *ots, struct oplus_rq *orq)
{
	struct list_head *pos;
	struct task_struct *tsk = NULL;

	list_for_each(pos, &orq->ux_list) {
		struct oplus_task_struct *tmp_ots = container_of(pos, struct oplus_task_struct,
			ux_entry);

		if (IS_ERR_OR_NULL(tmp_ots))
			continue;

		if (prio_higher(ots->ux_state, tmp_ots->ux_state))
			break;
	}
	list_add(&ots->ux_entry, pos->prev);

	tsk = ots_to_ts(ots);
	if (unlikely(global_debug_enabled & DEBUG_FTRACE))
		trace_printk("insert ux=%-12s pid=%d ux_state=%d\n", tsk->comm, tsk->pid, ots->ux_state);
}

void account_ux_runtime(struct rq *rq, struct task_struct *curr)
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

	if (delta < UX_EXEC_SLICE)
		return;

	ots->sum_exec_baseline += delta;
	ots->total_exec += delta;

	limit = ux_task_exec_limit(curr);
	if (ots->total_exec > limit) {
		list_del_init(&ots->ux_entry);
		put_task_struct(curr);
		return;
	}

	/* if ux slice has expired but total exectime not, just requeue without put/get task_struct */
	list_del_init(&ots->ux_entry);
	insert_ux_task_into_list(ots, orq);
}

static void enqueue_ux_thread(struct rq *rq, struct task_struct *p)
{
	struct list_head *pos, *n;
	bool exist = false;
	struct oplus_rq *orq = NULL;
	struct oplus_task_struct *ots = NULL;

	if (unlikely(!global_sched_assist_enabled))
		return;

	ots = get_oplus_task_struct(p);

	if (IS_ERR_OR_NULL(ots) || !test_task_is_fair(p) || !oplus_list_empty(&ots->ux_entry))
		return;

	/* task's ux entry should be initialized with INIT_LIST_HEAD() */
	if (ots->ux_entry.prev == 0 && ots->ux_entry.next == 0)
		INIT_LIST_HEAD(&ots->ux_entry);

	oplus_set_enqueue_time(p, rq->clock);

	if (test_task_ux(p)) {
		orq = (struct oplus_rq *) rq->android_oem_data1;

		list_for_each_safe(pos, n, &orq->ux_list) {
			if (pos == oplus_get_ux_entry(p)) {
				exist = true;
				BUG_ON(1);
				break;
			}
		}

		if (!exist) {
			insert_ux_task_into_list(ots, orq);
			get_task_struct(p);
		}

		if (!ots->total_exec)
			ots->sum_exec_baseline = p->se.sum_exec_runtime;
	}
}

static void dequeue_ux_thread(struct rq *rq, struct task_struct *p)
{
	struct oplus_task_struct *ots = NULL;

	if (!rq || !p)
		return;

	oplus_set_enqueue_time(p, 0);
	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		return;

	if (!oplus_list_empty(&ots->ux_entry)) {
		u64 now = jiffies_to_nsecs(jiffies);

		list_del_init(&ots->ux_entry);
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

	if (p->__state != TASK_RUNNING)
		ots->total_exec = 0;
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
	struct rq_flags flags;
	struct rq *rq;

	rq = task_rq_lock(task, &flags);
	inherit_ux_inc(task, type);
	task_rq_unlock(rq, task, &flags);
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
	struct rq_flags flags;
	struct rq *rq = NULL;
	struct list_head *list = NULL;

	if (!task || type >= INHERIT_UX_MAX)
		return;

	rq = task_rq_lock(task, &flags);

	if (!test_task_is_fair(task)) {
		task_rq_unlock(rq, task, &flags);
		return;
	}

	inherit_ux_inc(task, type);
	oplus_set_ux_depth(task, depth + 1);

	if (inherit_val & SA_TYPE_LISTPICK) {
		inherit_val &= (~SA_TYPE_LISTPICK);
		inherit_val |= SA_TYPE_HEAVY;
	}

	if (oplus_set_ux_state(task, (inherit_val & SCHED_ASSIST_UX_MASK) | SA_TYPE_INHERIT))
		goto out;

	oplus_set_inherit_ux_start(task, jiffies_to_nsecs(jiffies));
	trace_inherit_ux_set(task, type, oplus_get_ux_state(task), oplus_get_inherit_ux(task), oplus_get_ux_depth(task));

	list = oplus_get_ux_entry(task);
	if (task->on_rq && list && oplus_list_empty(list)) {
		struct oplus_task_struct *ots = get_oplus_task_struct(task);
		struct oplus_rq *orq = (struct oplus_rq *) rq->android_oem_data1;

		if (IS_ERR_OR_NULL(ots))
			goto out;

		insert_ux_task_into_list(ots, orq);
		get_task_struct(task);
	}
out:
	task_rq_unlock(rq, task, &flags);
}
EXPORT_SYMBOL_GPL(set_inherit_ux);

void reset_inherit_ux(struct task_struct *inherit_task, struct task_struct *ux_task, int reset_type)
{
	struct rq_flags flags;
	struct rq *rq;
	int reset_depth = 0;
	int reset_inherit = 0;
	int ux_state;

	if (!inherit_task || !ux_task || reset_type >= INHERIT_UX_MAX)
		return;

	reset_inherit = oplus_get_ux_state(ux_task);
	reset_depth = oplus_get_ux_depth(ux_task);

	if (!test_inherit_ux(inherit_task, reset_type) || !(reset_inherit & SA_TYPE_ANIMATOR))
		return;

	rq = task_rq_lock(inherit_task, &flags);

	ux_state = (oplus_get_ux_state(inherit_task) & ~SCHED_ASSIST_UX_MASK) | reset_inherit;
	oplus_set_ux_depth(inherit_task, reset_depth + 1);

	if (oplus_set_ux_state(inherit_task, ux_state))
		goto out;

	trace_inherit_ux_reset(inherit_task, reset_type, oplus_get_ux_state(inherit_task),
		oplus_get_inherit_ux(inherit_task), oplus_get_ux_depth(inherit_task));

out:
	task_rq_unlock(rq, inherit_task, &flags);
}
EXPORT_SYMBOL_GPL(reset_inherit_ux);

void unset_inherit_ux_value(struct task_struct *task, int type, int value)
{
	struct rq_flags flags;
	struct rq *rq = NULL;
	s64 inherit_ux = 0;
	struct oplus_task_struct *ots = NULL;

	if (!task || type >= INHERIT_UX_MAX)
		return;

	rq = task_rq_lock(task, &flags);

	inherit_ux_sub(task, type, value);
	inherit_ux = oplus_get_inherit_ux(task);

	if (inherit_ux > 0) {
		task_rq_unlock(rq, task, &flags);
		return;
	}

	if (inherit_ux < 0)
		oplus_set_inherit_ux(task, 0);

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		goto out;

	ots->ux_depth = 0;
	ots->ux_state = 0;

	trace_inherit_ux_unset(task, type, oplus_get_ux_state(task), oplus_get_inherit_ux(task), oplus_get_ux_depth(task));

out:
	task_rq_unlock(rq, task, &flags);
}
EXPORT_SYMBOL_GPL(unset_inherit_ux_value);

void unset_inherit_ux(struct task_struct *task, int type)
{
	unset_inherit_ux_value(task, type, 1);
}
EXPORT_SYMBOL_GPL(unset_inherit_ux);

bool im_mali(struct task_struct *p)
{
	return strstr(p->comm, "mali-event-hand");
}

bool cgroup_check_set_sched_assist_boost(struct task_struct *p)
{
	return im_mali(p);
}

void cgroup_set_sched_assist_boost_task(struct task_struct *p)
{
	struct rq_flags flags;
	struct rq *rq;
	int ux_state;

	if(cgroup_check_set_sched_assist_boost(p)) {
		rq = task_rq_lock(p, &flags);

		ux_state = oplus_get_ux_state(p);
		if (is_top(p))
			oplus_set_ux_state(p, (ux_state | SA_TYPE_HEAVY));
		else
			oplus_set_ux_state(p, (ux_state & ~SA_TYPE_HEAVY));

		task_rq_unlock(rq, p, &flags);
	} else
		return;
}

void clear_all_inherit_type(struct task_struct *p)
{
	struct rq_flags flags;
	struct rq *rq = NULL;
	struct oplus_task_struct *ots = NULL;

	rq = task_rq_lock(p, &flags);
	ots = get_oplus_task_struct(p);
	if (IS_ERR_OR_NULL(ots))
		goto out;

	atomic64_set(&ots->inherit_ux, 0);
	ots->ux_depth = 0;
	ots->ux_state = 0;

out:
	task_rq_unlock(rq, p, &flags);
}

void sched_assist_target_comm(struct task_struct *task, const char *buf)
{
	/*set mali-event-handle ux when task fork*/
	cgroup_set_sched_assist_boost_task(task);

	/* TODO: remove it in next version */
	if (task->group_leader && strstr(task->group_leader->comm, "lizhifm"))
		if (strstr(buf, "Aplayer"))
			oplus_set_im_flag(task, IM_FLAG_3RD_AUDIO);

	if (!strncmp(buf, "HwBinder", 8))
		oplus_set_im_flag(task, IM_FLAG_HWBINDER);
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

	if (!ret || !local_cpu_mask || cpumask_empty(local_cpu_mask))
		return;

	cpumask_copy(&mask_backup, local_cpu_mask);

	drop_cpu = cpumask_first(local_cpu_mask);
	while (drop_cpu < nr_cpu_ids) {
		/* unlocked access */
		rq = cpu_rq(drop_cpu);
		orq = (struct oplus_rq *) rq->android_oem_data1;
		task = rq->curr;

		if (!test_task_ux(task)) {
			if (oplus_list_empty(&orq->ux_list)) {
				drop_cpu = cpumask_next(drop_cpu, local_cpu_mask);
				continue;
			} else {
				ots = list_first_entry(&orq->ux_list, struct oplus_task_struct, ux_entry);
				task  = ots_to_ts(ots);
			}
		}

		/* avoid sf premmpt heavy ux tasks,such as ui, render... */
		if ((oplus_get_im_flag(p) == IM_FLAG_SURFACEFLINGER || oplus_get_im_flag(p) == IM_FLAG_RENDERENGINE) &&
			((oplus_get_ux_state(task) & SA_TYPE_HEAVY) || (oplus_get_ux_state(task) & SA_TYPE_LISTPICK))) {
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("clear cpu from lowestmask, curr_heavy task=%-12s pid=%d drop_cpu=%d\n", task->comm, task->pid, drop_cpu);
		}

#ifdef CONFIG_OPLUS_SCHED_MT6895
		if (oplus_get_ux_state(task) & SA_TYPE_HEAVY) {
#else
		if (sched_assist_scene(SA_LAUNCH) && (oplus_get_ux_state(task) & SA_TYPE_HEAVY)) {
#endif
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("clear cpu from lowestmask, curr_heavy task=%-12s pid=%d drop_cpu=%d\n", task->comm, task->pid, drop_cpu);
		}

		if (oplus_get_ux_state(task) & SA_TYPE_ANIMATOR) {
			cpumask_clear_cpu(drop_cpu, local_cpu_mask);
			if (unlikely(global_debug_enabled & DEBUG_FTRACE))
				trace_printk("clear cpu from lowestmask, curr_anima task=%-12s pid=%d drop_cpu=%d\n", task->comm, task->pid, drop_cpu);
		}

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
		struct task_struct *task = READ_ONCE(cpu_rq(next_cpu)->curr);
		bool is_target = !(oplus_get_ux_state(task) & SA_TYPE_ANIMATOR);
		int prio = task->prio;

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
	struct oplus_task_struct *ots = NULL;
	bool ret = false;

	if (oplus_list_empty(&orq->ux_list)) {
		ret = false;
		goto out;
	}

	ots = list_first_entry(&orq->ux_list, struct oplus_task_struct, ux_entry);
	if (IS_ERR_OR_NULL(ots) || ots->im_flag == IM_FLAG_CAMERA_HAL) {
		ret = false;
		goto out;
	}

	if (*sync) {
		*sync = false;
		if (unlikely(global_debug_enabled & DEBUG_FTRACE))
			trace_printk("comm=%-12s pid=%d cpu=%d\n", p->comm, p->pid, cpu);

		ret = true;
	}

out:
	return ret;
}
EXPORT_SYMBOL(sa_skip_rt_sync);

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

void android_rvh_dequeue_task_handler(void *unused, struct rq *rq, struct task_struct *p, int flags)
{
	queue_ux_thread(rq, p, 0);
}

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

	oplus_set_enqueue_time(prev, rq->clock);

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && likely(prev != next))
		ux_state_systrace_c(cpu_of(rq), next);

	if (unlikely(global_debug_enabled & DEBUG_FBG) && likely(prev != next))
		fbg_state_systrace_c(cpu_of(rq), next);
}

void android_vh_scheduler_tick_handler(void *unused, struct rq *rq)
{
	struct list_head *list = NULL;

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
	update_rq_nr_imbalance(smp_processor_id());
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

	raw_spin_rq_lock(rq);
	list = oplus_get_ux_entry(rq->curr);
	if (!IS_ERR_OR_NULL(list) && !oplus_list_empty(list))
		account_ux_runtime(rq, rq->curr);
	raw_spin_rq_unlock(rq);

	if (unlikely(global_debug_enabled & DEBUG_SYSTRACE) && (cpu_of(rq) == 0)) {
		sa_scene_systrace_c();
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
		sa_spread_systrace_c();
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
	}
}
static int boost_kill = 1;
module_param_named(boost_kill, boost_kill, uint, 0644);
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

void android_vh_exit_signal_handler(void *unused, struct task_struct *p)
{
	int is_32bit;
	cpumask_var_t boost_mask;
	if (p == NULL)
		return;
	if (boost_kill && get_grp(p) == BGAPP) {
		is_32bit = test_ti_thread_flag(&p->thread_info, TIF_32BIT);
		set_user_nice(p, -20);
		if (is_32bit) {
			cpumask_and(boost_mask, cpu_active_mask, system_32bit_el0_cpumask());
		}
		else {
			cpumask_copy(boost_mask, cpu_active_mask);
		}
		if (!cpumask_empty(boost_mask)) {
			cpumask_copy(&p->cpus_mask, boost_mask);
			p->nr_cpus_allowed = cpumask_weight(boost_mask);
		}
	}
}

void android_vh_cgroup_set_task_handler(void *unused, int ret, struct task_struct *task)
{
	if (!ret) {
		cgroup_set_sched_assist_boost_task(task);
	}
}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_BAN_APP_SET_AFFINITY)
void android_vh_sched_setaffinity_early_handler(void *unused, struct task_struct *task, const struct cpumask *new_mask, int *skip)
{
	int im_flag = IM_FLAG_NONE;

	int curr_uid = current_uid().val;
	if ((curr_uid < FIRST_APPLICATION_UID) || (curr_uid > LAST_APPLICATION_UID)) {
		return;
	}

	if (task->pid == task->tgid) {
		im_flag = oplus_get_im_flag(task);
	} else {
		struct task_struct *main_task;

		rcu_read_lock();
		main_task = find_task_by_vpid(task->tgid);
		if (main_task) {
			im_flag = oplus_get_im_flag(main_task);
		}
		rcu_read_unlock();
	}

	if (im_flag == IM_FLAG_FORBID_SET_CPU_AFFINITY) {
		*skip = 1;
	}
}
#endif
