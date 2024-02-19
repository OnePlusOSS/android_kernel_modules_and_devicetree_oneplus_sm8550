// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <kernel/sched/sched.h>
#include <linux/math64.h>
#include <linux/sched.h>

#include "sa_common.h"
#include "sa_priority.h"

/*
 * Nice levels are multiplicative, with a gentle 10% change for every
 * nice level changed. I.e. when a CPU-bound task goes from nice 0 to
 * nice 1, it will get ~10% less CPU time than another CPU-bound task
 * that remained on nice 0.
 *
 * The "10% effect" is relative and cumulative: from _any_ nice level,
 * if you go up 1 level, it's -10% CPU usage, if you go down 1 level
 * it's +10% CPU usage. (to achieve that we use a multiplier of 1.25.
 * If a task goes up by ~10% and another task goes down by ~10% then
 * the relative distance between them is ~25%.)
 */
const int ux_prio_to_weight[PRIORITY_LEVEL_NUM] = {
	1024, 1277, 1586, 1991, 2501, 3121, 3906, 4904, 6100, 7620, 9548,
};

/*
 * Inverse (2^32/x) values of the ux_prio_to_weight[] array, precalculated.
 *
 * In cases where the weight does not change often, we can use the
 * precalculated inverse to speed up arithmetics by turning divisions
 * into multiplications:
 */
const u32 ux_prio_to_wmult[PRIORITY_LEVEL_NUM] = {
	4194304, 3363326, 2708050, 2157191, 1717300, 1376151, 1099582, 875809, 704093, 563644, 449829,
};

/*
 * Accumulate calc_delta_fair(PRIO_EXEC_GAP, every higher priority).
 *
 * ux_prio_to_preset_vruntime[PRIORITY_LEVEL_NUM - 1] = 0;
 * for (i = PRIORITY_LEVEL_NUM - 2; i >= 0; i--) {
 * 	u64 delta = calc_delta_fair(PRIO_EXEC_GAP, i + 1);
 * 	DEBUG_BUG_ON(delta > 4294967295);
 * 	ux_prio_to_preset_vruntime[i] = delta;
 * }
 *
 * for (i = PRIORITY_LEVEL_NUM - 2; i >= 0; i--) {
 * 	u64 delta = (u64)ux_prio_to_preset_vruntime[i] + (u64)ux_prio_to_preset_vruntime[i + 1];
 * 	DEBUG_BUG_ON(delta > 4294967295);
 * 	ux_prio_to_preset_vruntime[i] = delta;
 * }
 *
 */
const u32 ux_prio_to_preset_vruntime[PRIORITY_LEVEL_NUM] = {
	14319390, 11111873, 8529276, 6472019, 4834275, 3521876, 2473233, 1637997, 966522, 428990, 0,
};

static inline u64 max_vruntime(u64 max_vruntime, u64 vruntime)
{
	s64 delta = (s64)(vruntime - max_vruntime);
	if (delta > 0)
		max_vruntime = vruntime;

	return max_vruntime;
}

static inline u64 min_vruntime(u64 min_vrt, u64 vruntime)
{
	s64 delta = (s64)(vruntime - min_vrt);
	if (delta < 0)
		min_vrt = vruntime;

	return min_vrt;
}

static inline int vruntime_before(u64 a_vruntime, u64 b_vruntime)
{
	return (s64)(a_vruntime - b_vruntime) < 0;
}

#ifdef CONFIG_64BIT
#define NICE_0_LOAD_SHIFT (SCHED_FIXEDPOINT_SHIFT + SCHED_FIXEDPOINT_SHIFT)
#define scale_load(w) ((w) << SCHED_FIXEDPOINT_SHIFT)
#define scale_load_down(w)                                 \
	({                                                     \
		unsigned long __w = (w);                           \
		if (__w)                                           \
			__w = max(2UL, __w >> SCHED_FIXEDPOINT_SHIFT); \
		__w;                                               \
	})
#else
#define NICE_0_LOAD_SHIFT (SCHED_FIXEDPOINT_SHIFT)
#define scale_load(w) (w)
#define scale_load_down(w) (w)
#endif

#define WMULT_CONST (~0U)
#define WMULT_SHIFT 32

static void __update_inv_weight(struct load_weight *lw)
{
	unsigned long w;

	if (likely(lw->inv_weight))
		return;

	w = scale_load_down(lw->weight);

	if (BITS_PER_LONG > 32 && unlikely(w >= WMULT_CONST))
		lw->inv_weight = 1;
	else if (unlikely(!w))
		lw->inv_weight = WMULT_CONST;
	else
		lw->inv_weight = WMULT_CONST / w;
}

/*
 * delta_exec * weight / lw.weight
 *   OR
 * (delta_exec * (weight * lw->inv_weight)) >> WMULT_SHIFT
 *
 * Either weight := NICE_0_LOAD and lw \e ux_prio_to_wmult[], in which case
 * we're guaranteed shift stays positive because inv_weight is guaranteed to
 * fit 32 bits, and NICE_0_LOAD gives another 10 bits; therefore shift >= 22.
 *
 * Or, weight =< lw.weight (because lw.weight is the runqueue weight), thus
 * weight/lw.weight <= 1, and therefore our shift will also be positive.
 */
static u64 __calc_delta(u64 delta_exec, unsigned long weight, struct load_weight *lw)
{
	u64 fact = scale_load_down(weight);
	int shift = WMULT_SHIFT;

	__update_inv_weight(lw);

	if (unlikely(fact >> 32)) {
		while (fact >> 32) {
			fact >>= 1;
			shift--;
		}
	}

	fact = mul_u32_u32(fact, lw->inv_weight);

	while (fact >> 32) {
		fact >>= 1;
		shift--;
	}

	return mul_u64_u32_shr(delta_exec, fact, shift);
}

static u64 __sched_period(unsigned long nr_running)
{
	if (unlikely(nr_running > CFS_SCHED_NR_LATENCY))
		return nr_running * CFS_SCHED_MIN_GRAN;
	else
		return CFS_SCHED_LATENCY;
}

static inline u64 calc_delta_fair(u64 delta, int prio)
{
	struct load_weight load;
	DEBUG_BUG_ON(prio < 0 || prio >= PRIORITY_LEVEL_NUM);

	load.weight = ux_prio_to_weight[prio];
	load.inv_weight = ux_prio_to_wmult[prio];

	if (unlikely(load.weight != NICE_0_LOAD))
		delta = __calc_delta(delta, NICE_0_LOAD, &load);

	return delta;
}

static u64 sched_slice(struct oplus_rq *orq, struct oplus_task_struct *ots, bool on_rq)
{
	unsigned int nr_running = orq->nr_running;
	u64 slice;
	struct load_weight load;

	slice = __sched_period(nr_running + !on_rq);

	DEBUG_BUG_ON((s64)orq->load_weight <= 0);
	load.weight = orq->load_weight;
	load.inv_weight = 0;
	slice = __calc_delta(slice, ux_prio_to_weight[ots->ux_priority], &load);

	slice = max(slice, (u64)CFS_SCHED_MIN_GRAN);

	return slice;
}

static u64 sched_vslice(struct oplus_rq *orq, struct oplus_task_struct *ots)
{
	return calc_delta_fair(sched_slice(orq, ots, false), ots->ux_priority);
}

inline int ux_state_to_priority(int ux_state)
{
	int prio = (uint)(ux_state & SCHED_ASSIST_UX_PRIORITY_MASK) >> SCHED_ASSIST_UX_PRIORITY_SHIFT;

	DEBUG_BUG_ON(prio < 0 || prio >= PRIORITY_LEVEL_NUM);

	if (prio >= PRIORITY_LEVEL_NUM) {
		prio = PRIORITY_LEVEL_NUM - 1;
	}

	return prio;
}

inline int ux_state_to_nice(int ux_state)
{
	int nice = 0;

	/* NOTE: could nice exec time surpass priority exec gap? */
	if (ux_state & SA_TYPE_SWIFT) {
		nice = 0;
	} else if (ux_state & SA_TYPE_ANIMATOR) {
		nice = 1;
	} else if (ux_state & SA_TYPE_LIGHT) {
		nice = 4;
	} else if (ux_state & SA_TYPE_HEAVY) {
		nice = 7;
	} else if (ux_state & SA_TYPE_LISTPICK) {
		/* surpass priority exec gap */
		nice = 30;
	} else {
		DEBUG_BUG_ON(1);
	}
	return nice;
}

u64 prio_nice_to_vruntime(int ux_priority, int ux_nice)
{
	u64 delta = NICE_EXEC_GAP * ux_nice;
	delta = calc_delta_fair(delta, ux_priority);
	delta += ux_prio_to_preset_vruntime[ux_priority];
	return delta;
}

void initial_prio_nice_and_vruntime(struct oplus_rq *orq, struct oplus_task_struct *ots, int ux_prio, int ux_nice)
{
	u64 preset_vruntime;

	lockdep_assert_held(orq->ux_list_lock);
	DEBUG_BUG_ON(ux_prio < 0 || ux_prio >= PRIORITY_LEVEL_NUM);

	ots->ux_priority = ux_prio;
	ots->ux_nice = ux_nice;
#ifdef ENABLE_PRESET_VRUNTIME
	preset_vruntime = prio_nice_to_vruntime(ux_prio, ux_nice);
	if (orq->nr_running > 0) {
		struct oplus_task_struct *first_ots = ux_list_first_entry(&orq->ux_list);
		DEBUG_BUG_ON(first_ots == NULL);
		/* if new ux task's prio is lower than current task's, add period slice */
		if (ux_prio < first_ots->ux_priority) {
			preset_vruntime += sched_vslice(orq, ots);
		} else if (ux_prio == first_ots->ux_priority) {
			/* add half fo period slice, smaller than one priority exec gap. */
			preset_vruntime += (sched_vslice(orq, ots) >> 1);
		}
	}
#else
	preset_vruntime = 0;
	if (orq->nr_running > 0) {
		preset_vruntime = sched_vslice(orq, ots);
	}
#endif

	ots->preset_vruntime = preset_vruntime;
	ots->vruntime = orq->min_vruntime + preset_vruntime;
}

void update_vruntime_task_detach(struct oplus_rq *orq, struct oplus_task_struct *ots)
{
	u64 exec_vruntime;
	lockdep_assert_held(orq->ux_list_lock);

	exec_vruntime = ots->vruntime - ots->preset_vruntime;
	DEBUG_BUG_ON((s64)(exec_vruntime) < 0);

	ots->vruntime = max_vruntime(exec_vruntime - orq->min_vruntime, 0);
	ots->preset_vruntime = 0;
}

void update_vruntime_task_attach(struct oplus_rq *orq, struct oplus_task_struct *ots)
{
	u64 exec_vruntime;

	lockdep_assert_held(orq->ux_list_lock);
	DEBUG_BUG_ON(ots->preset_vruntime != 0);

	exec_vruntime = ots->vruntime;
	initial_prio_nice_and_vruntime(orq, ots, ots->ux_priority, ots->ux_nice);
	ots->vruntime = ots->vruntime + exec_vruntime;
}

static inline bool __entity_less(struct rb_node *a, const struct rb_node *b)
{
	struct oplus_task_struct *ots_a = rb_entry(a, struct oplus_task_struct,
			ux_entry);
	struct oplus_task_struct *ots_b = rb_entry(b, struct oplus_task_struct,
			ux_entry);
	return vruntime_before(ots_a->vruntime, ots_b->vruntime);
}

static inline bool __entity_less_exec(struct rb_node *a, const struct rb_node *b)
{
	struct oplus_task_struct *ots_a = rb_entry(a, struct oplus_task_struct,
			exec_time_node);
	struct oplus_task_struct *ots_b = rb_entry(b, struct oplus_task_struct,
			exec_time_node);
	return vruntime_before(ots_a->vruntime - ots_a->preset_vruntime, ots_b->vruntime - ots_b->preset_vruntime);
}

void insert_task_to_ux_timeline(struct oplus_task_struct *ots, struct oplus_rq *orq)
{
	rb_add_cached(&ots->ux_entry, &orq->ux_list, __entity_less);
	rb_add_cached(&ots->exec_time_node, &orq->exec_timeline, __entity_less_exec);

	orq->nr_running++;
	DEBUG_BUG_ON(ots->ux_priority < 0 || ots->ux_priority >= PRIORITY_LEVEL_NUM);
	orq->load_weight += ux_prio_to_weight[ots->ux_priority];
}

void update_ux_timeline_task_change(struct oplus_rq *orq, struct oplus_task_struct *ots, int new_prio, int new_nice)
{
	int old_prio = ots->ux_priority;
	int old_nice = ots->ux_nice;
#ifdef ENABLE_PRESET_VRUNTIME
	u64 old_preset_vrt;
#endif
	lockdep_assert_held(orq->ux_list_lock);

	orq->load_weight = orq->load_weight - ux_prio_to_weight[old_prio] + ux_prio_to_weight[new_prio];
	DEBUG_BUG_ON((orq->nr_running < 0) || ((s64)orq->load_weight < 0));
	DEBUG_BUG_ON((orq->nr_running == 0) && (orq->load_weight != 0));
	DEBUG_BUG_ON((orq->nr_running != 0) && (orq->load_weight == 0));
	DEBUG_BUG_ON(new_prio < 0 || new_prio >= PRIORITY_LEVEL_NUM);

	ots->ux_priority = new_prio;
	ots->ux_nice = new_nice;


#ifdef ENABLE_PRESET_VRUNTIME
	old_preset_vrt = ots->preset_vruntime;
	ots->preset_vruntime = old_preset_vrt - prio_nice_to_vruntime(old_prio, old_nice) + prio_nice_to_vruntime(new_prio, new_nice);
	ots->vruntime = ots->vruntime - old_preset_vrt + ots->preset_vruntime;
#endif
	DEBUG_BUG_ON((s64)(ots->preset_vruntime) < 0);
	DEBUG_BUG_ON((s64)(ots->vruntime) < 0);

	/* rebalance vruntime timeline */
	rb_erase_cached(&ots->ux_entry, &orq->ux_list);
	rb_add_cached(&ots->ux_entry, &orq->ux_list, __entity_less);
}

void update_ux_timeline_task_tick(struct oplus_rq *orq, struct oplus_task_struct *ots) {
	struct rb_node *next;
	struct oplus_task_struct *ots_next;
	bool need_update_min_vrt;

	lockdep_assert_held(orq->ux_list_lock);
	DEBUG_BUG_ON(orq->nr_running == 0);

	next = rb_next(&ots->ux_entry);
	if (next != NULL) {
		ots_next = rb_entry(next, struct oplus_task_struct, ux_entry);
		/* after adding vruntime, only if bigger than next entry, change position */
		if (!vruntime_before(ots->vruntime, ots_next->vruntime)) {
			rb_erase_cached(&ots->ux_entry, &orq->ux_list);
			rb_add_cached(&ots->ux_entry, &orq->ux_list, __entity_less);
		}
	}

	/* only if the leftmost exec node has changed, update min exec vruntime */
	need_update_min_vrt = (&ots->exec_time_node == rb_first_cached(&orq->exec_timeline));

	next = rb_next(&ots->exec_time_node);
	if (next != NULL) {
		ots_next = rb_entry(next, struct oplus_task_struct, exec_time_node);
		/* after adding vruntime, only if bigger than next entry, change position */
		if (!vruntime_before(ots->vruntime - ots->preset_vruntime, ots_next->vruntime - ots_next->preset_vruntime)) {
			rb_erase_cached(&ots->exec_time_node, &orq->exec_timeline);
			rb_add_cached(&ots->exec_time_node, &orq->exec_timeline, __entity_less_exec);
		}
	}

	if (need_update_min_vrt) {
		/* NOTE: ots->vruntime may move backwards if the priority of task becomes higher.
		* And its exec_vruntime (ots->vruntime - ots->preset_vruntime) keeps forewards while keeps running.
		* When switch to next task, next task's exec_vruntime may begin at zero again.
		* follow up exec_vruntime let orq->min_vruntime move on.
		*/
		struct oplus_task_struct * first_ots;
		u64 exec_vruntime;

		first_ots = exec_timeline_first_entry(&orq->exec_timeline);
		exec_vruntime = first_ots->vruntime - first_ots->preset_vruntime;
		DEBUG_BUG_ON((s64)(exec_vruntime) < 0);
		orq->min_vruntime = max_vruntime(orq->min_vruntime, exec_vruntime);
	}
}

void update_ux_timeline_task_removal(struct oplus_rq *orq, struct oplus_task_struct *ots) {
	bool need_update_min_vrt;

	lockdep_assert_held(orq->ux_list_lock);
	DEBUG_BUG_ON(orq->nr_running == 0);

	/* only if the leftmost exec node has removed, update min exec vruntime */
	need_update_min_vrt = (&ots->exec_time_node == rb_first_cached(&orq->exec_timeline));

	rb_erase_cached(&ots->ux_entry, &orq->ux_list);
	/*  after rb_erase_cached(), RB_EMPTY_NODE(node) may return false. confirm by RB_CLEAR_NODE() */
	RB_CLEAR_NODE(&ots->ux_entry);

	rb_erase_cached(&ots->exec_time_node, &orq->exec_timeline);

	orq->nr_running--;
	orq->load_weight -= ux_prio_to_weight[ots->ux_priority];

	DEBUG_BUG_ON((orq->nr_running < 0) || ((s64)orq->load_weight < 0));
	DEBUG_BUG_ON((orq->nr_running == 0) && (orq->load_weight != 0));
	DEBUG_BUG_ON((orq->nr_running != 0) && (orq->load_weight == 0));

	if (orq->nr_running == 0) {
		orq->min_vruntime = 0;
		DEBUG_BUG_ON(NULL != ux_list_first_entry(&orq->ux_list));
		DEBUG_BUG_ON(NULL != exec_timeline_first_entry(&orq->exec_timeline));
		return;
	}

	if (need_update_min_vrt) {
		/* NOTE: ots->vruntime may move backwards if the priority of task becomes higher.
		* And its exec_vruntime (ots->vruntime - ots->preset_vruntime) keeps forewards while keeps running.
		* When switch to next task, next task's exec_vruntime may begin at zero again.
		* follow up exec_vruntime let orq->min_vruntime move on.
		*/
		struct oplus_task_struct * first_ots;
		u64 exec_vruntime;

		first_ots = exec_timeline_first_entry(&orq->exec_timeline);
		exec_vruntime = first_ots->vruntime - first_ots->preset_vruntime;
		DEBUG_BUG_ON((s64)(exec_vruntime) < 0);
		orq->min_vruntime = max_vruntime(orq->min_vruntime, exec_vruntime);
	}
}

bool need_resched_ux(struct oplus_rq *orq, struct oplus_task_struct *curr, unsigned long delta_exec)
{
	struct oplus_task_struct *ots;
	s64 vdiff;
	unsigned long ideal_runtime;

	lockdep_assert_held(orq->ux_list_lock);

	if (delta_exec < CFS_SCHED_MIN_GRAN)
		return false;

	ots = ux_list_first_entry(&orq->ux_list);

	DEBUG_BUG_ON(ots == NULL);
	if ((ots == NULL) || (curr == ots)) {
		return false;
	}

	ideal_runtime = sched_slice(orq, curr, true);
	if (delta_exec >= ideal_runtime) {
		return true;
	}

	vdiff = curr->vruntime - ots->vruntime;

	if (vdiff <= 0) {
		return false;
	}

	return (vdiff > ideal_runtime);
}

bool need_wakeup_preempt(struct oplus_rq *orq, struct oplus_task_struct *curr)
{
	s64 ux_wakeup_gran_vtime, vdiff;

	struct oplus_task_struct *ots = ux_list_first_entry(&orq->ux_list);
	DEBUG_BUG_ON(ots == NULL);
	if (curr == ots) {
		return false;
	}

	vdiff = curr->vruntime - ots->vruntime;
	if (vdiff <= 0) {
		return false;
	}

	ux_wakeup_gran_vtime = calc_delta_fair(CFS_WAKEUP_GRAN, ots->ux_priority);

	return (vdiff > ux_wakeup_gran_vtime);
}

void android_vh_sched_stat_runtime_handler(void *unused, struct task_struct *task, u64 delta, u64 vruntime)
{
	struct rq *rq;
	struct oplus_rq *orq;
	struct oplus_task_struct *ots;
	unsigned long irqflag;

	rq = task_rq(task);
	orq = (struct oplus_rq *)rq->android_oem_data1;
	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots)) {
		return;
	}

	spin_lock_irqsave(orq->ux_list_lock, irqflag);
	if (!oplus_rbnode_empty(&ots->ux_entry)) {
		unsigned int limit;

		ots->total_exec += delta;
		ots->vruntime += calc_delta_fair(delta, ots->ux_priority);
		limit = ux_task_exec_limit(task);
		if (ots->total_exec >= limit) {
			/*TODO: The cfs vruntime(sched_entity->vruntime) of task accumulates when task is running as ux.
			 *And when removed from the ux list, task is still runnable.
			 *Its vruntime is much bigger than other tasks in cfs rbtree,
			 *So it waits a lot and makes up for the executed vruntime.
			 *No vendor hook can fix this now.
			 */
			update_ux_timeline_task_removal(orq, ots);
			put_task_struct(task);
		} else {
			/* rebalance ux timeline after task's vruntime changed */
			update_ux_timeline_task_tick(orq, ots);
		}
	}
	spin_unlock_irqrestore(orq->ux_list_lock, irqflag);
}
