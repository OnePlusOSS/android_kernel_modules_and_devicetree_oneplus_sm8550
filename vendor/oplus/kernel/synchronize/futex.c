// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#include <include/linux/sched.h>
#include <kernel/sched/sched.h>
#include <include/linux/hrtimer.h>
#include <include/linux/futex.h>
#include <include/linux/sched/task.h>
#include <linux/pid.h>
#include <trace/hooks/futex.h>

#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#include "locking_main.h"

#ifdef CONFIG_MMU
# define FLAGS_SHARED		0x01
#else
# define FLAGS_SHARED		0x00
#endif

#define FLAGS_OWNER_SHIFT		(8)

/*
 * Note:
 * Add new futex ops, which should be different from other ops defined
 * in include/uapi/linux/futex.h.
 */
#define FUTEX_WAIT_NOTIFY_WAITER 15

#define FUTEX_WAITER_TOLERATE_THRESHOLD (200000000) /* 200ms */

atomic64_t futex_inherit_set_times;
atomic64_t futex_inherit_unset_times;
atomic64_t futex_inherit_useless_times;

atomic64_t futex_low_count;
atomic64_t futex_high_count;

/*
 * Note:
 * structure futex_q should keep same as it's definition
 * in kernel/futex.c.
 */
struct futex_q {
	struct plist_node list;

	struct task_struct *task;
	spinlock_t *lock_ptr;
	union futex_key key;
	struct futex_pi_state *pi_state;
	struct rt_mutex_waiter *rt_waiter;
	union futex_key *requeue_pi_key;
	u32 bitset;
} __randomize_layout;

#define INHERIT_SET (1)
#define INHERIT_INC (2)
static int futex_set_inherit_ux_refs(struct task_struct *holder, struct task_struct *p)
{
	bool set_ux;

	if (unlikely(!holder || !p))
		return 0;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
	set_ux = test_set_inherit_ux(p);
#else
	set_ux = false;
#endif

	if (set_ux) {
		int type = get_ux_state_type(holder);
		struct oplus_task_struct *ots = get_oplus_task_struct(p);

		if (unlikely(IS_ERR_OR_NULL(ots)))
			return 0;

		if (type == UX_STATE_NONE) {
			if (holder->__state & TASK_NORMAL)
				goto skip;

			set_inherit_ux(holder, INHERIT_UX_FUTEX, ots->ux_depth, ots->ux_state);
			atomic64_inc(&futex_inherit_set_times);
			cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
				"INHERIT_SET-holder(%-12s pid=%d tgid=%d inherit_ux=%llx) p=(%-12s pid=%d tgid=%d)\n",
				holder->comm, holder->pid, holder->tgid, oplus_get_inherit_ux(holder),
				p->comm, p->pid, p->tgid);

			return INHERIT_SET;
		} else if (type == UX_STATE_INHERIT) {
			if (holder->__state & TASK_NORMAL)
				goto skip;

			inc_inherit_ux_refs(holder, INHERIT_UX_FUTEX);
			atomic64_inc(&futex_inherit_set_times);
			cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
				"INHERIT_INC-holder(%-12s pid=%d tgid=%d inherit_ux=%llx) p=(%-12s pid=%d tgid=%d)\n",
				holder->comm, holder->pid, holder->tgid, oplus_get_inherit_ux(holder),
				p->comm, p->pid, p->tgid);

			return INHERIT_INC;
		}
skip:
		atomic64_inc(&futex_inherit_useless_times);
	}

	return 0;
}

static void futex_unset_inherit_ux_refs(struct task_struct *p, int value, int path)
{
	bool clear = false;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
	if (test_inherit_ux(p, INHERIT_UX_FUTEX)) {
		clear = true;
		unset_inherit_ux_value(p, INHERIT_UX_FUTEX, value);
	}

	/*
	 * Owner may clear it's inherit ux before waking q,
	 * so we do sub out of condition above.
	 */
	atomic64_sub(value, &futex_inherit_unset_times);
#endif
	cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
		"INHERIT_CLEAR-holder(%-12s pid=%d tgid=%d inherit_ux=%llx) clear=%d path=%d\n",
		p->comm, p->pid, p->tgid, oplus_get_inherit_ux(p), clear, path);
}

static inline bool curr_is_ux_thread(void)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
	return test_task_ux(current);
#else
	return false;
#endif
}

static inline bool curr_is_inherit_futex(void)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
	return test_inherit_ux(current, INHERIT_UX_FUTEX);
#else
	return false;
#endif
}

static bool boost_holder(struct task_struct *holder, struct task_struct *waiter)
{
	struct oplus_task_struct *ots;

	ots = get_oplus_task_struct(waiter);
	if (unlikely(IS_ERR_OR_NULL(ots)))
		return false;

	/*
	 * When futex-opt disabled, still keep futex-holder tracing, it won't affect
	 * performance/power. But stop these two functions:
	 * 1. modify ux thread's pnode-prio in android_vh_alter_futex_plist_add() hook.
	 * 2. set inherit ux thread with futex type in android_vh_futex_sleep_start() hook.
	 */
	if (unlikely(!locking_opt_enable(LK_FUTEX_ENABLE)))
		return false;

	/*
	 * If current(ux thread) upgrade it's holder to inherit ux successfully,
	 * mark ux_contrib as true(Ya, we have contributed an inherit ux thread).
	 */
	if (futex_set_inherit_ux_refs(holder, waiter)) {
		ots->lkinfo.ux_contrib = true;

		cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
			"futex holder comm=%-12s pid=%d tgid=%d ux_state=%d\n",
			holder->comm, holder->pid, holder->tgid,
			oplus_get_ux_state(holder));

		return true;
	}

	return false;
}

/**
 * match_futex - Check whether two futex keys are equal
 * @key1:	Pointer to key1
 * @key2:	Pointer to key2
 *
 * Return 1 if two futex_keys are equal, 0 otherwise.
 */
static inline int match_futex(union futex_key *key1, union futex_key *key2)
{
	return (key1 && key2
		&& key1->both.word == key2->both.word
		&& key1->both.ptr == key2->both.ptr
		&& key1->both.offset == key2->both.offset);
}

static struct task_struct *futex_find_task_by_pid(unsigned int pid)
{
	struct task_struct *p;

	if (pid <= 0 || pid > PID_MAX_DEFAULT)
		return NULL;

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (p)
		get_task_struct(p);
	rcu_read_unlock();

	return p;
}

static bool set_holder(struct task_struct *waiter, struct task_struct *holder)
{
	struct oplus_task_struct *ots;

	if (waiter->tgid == holder->tgid) {
		ots = get_oplus_task_struct(waiter);
		/* If waiter's holder is set, do put_task_struct(holder) in futex_wait_end() */
		if (likely(!IS_ERR_OR_NULL(ots))) {
			ots->lkinfo.holder = holder;
			return true;
		}
	}

	put_task_struct(holder);
	return false;
}

static void futex_notify_waiter(unsigned long pid)
{
	struct task_struct *waiter, *holder = current;
	bool is_target = false;
	bool try_to_boost;

	waiter = futex_find_task_by_pid(pid);
	if (!waiter)
		return;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
	is_target = test_task_ux(waiter);
#endif

	if (!is_target)
		goto out_put_waiter;

	get_task_struct(holder);
	try_to_boost = set_holder(waiter, holder);

	/* TODO: should check waiter's state? it should be sleeping */
	if (try_to_boost)
		boost_holder(holder, waiter);

out_put_waiter:
	put_task_struct(waiter);
}

#define WAKE_MSG (64)
#define NOTIFY_WAITER_SHIFT (16)
#define NOTIFY_OWNER_MASK   ((1 << NOTIFY_WAITER_SHIFT)-1)
static void android_vh_do_futex_handler(void *unused, int cmd,
		  unsigned int *flags, u32 __user *uaddr2)
{
	unsigned long waiter_pid, holder_pid;
	char wake_msg[WAKE_MSG];

	switch (cmd) {
	case FUTEX_WAIT:
		fallthrough;
	case FUTEX_WAIT_BITSET:
		holder_pid = (unsigned long)uaddr2;
		if (holder_pid > 0 && holder_pid <= PID_MAX_DEFAULT)
			*flags += holder_pid << FLAGS_OWNER_SHIFT;
		break;
	case FUTEX_WAIT_NOTIFY_WAITER:
		waiter_pid = (unsigned long)uaddr2 >> NOTIFY_WAITER_SHIFT;
		holder_pid = (unsigned long)uaddr2 & NOTIFY_OWNER_MASK;
		futex_notify_waiter(waiter_pid);
		break;
	case FUTEX_WAKE:
		if ((uaddr2 != NULL) && !copy_from_user(wake_msg, uaddr2, WAKE_MSG - 1))
			wake_msg[WAKE_MSG-1] = '\0';
		break;
	}
}

static void android_vh_futex_wait_start_handler(void *unused, unsigned int flags,
		  u32 bitset)
{
	struct task_struct *holder;

	if (!curr_is_ux_thread())
		return;

	holder = futex_find_task_by_pid(flags >> FLAGS_OWNER_SHIFT);
	if (!holder)
		return;

	set_holder(current, holder);
}

static void android_vh_futex_wait_end_handler(void *unused, unsigned int flags,
		  u32 bitset)
{
	struct oplus_task_struct *ots;

	ots = get_oplus_task_struct(current);
	if (IS_ERR_OR_NULL(ots))
		return;

	if (ots->lkinfo.holder) {
		struct task_struct *holder = ots->lkinfo.holder;
		if (unlikely(ots->lkinfo.ux_contrib)) {
			cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
				"curr(%-12s pid=%d tgid=%d) has woken, but ux_contrib is true! holder(%-12s pid=%d tgid=%d inherit_ux=%llx)\n",
				current->comm, current->pid, current->tgid,
				holder->comm, holder->pid,
				holder->tgid, oplus_get_inherit_ux(holder));
			futex_unset_inherit_ux_refs(ots->lkinfo.holder, 1, 3);
		}

		put_task_struct(ots->lkinfo.holder);
		ots->lkinfo.holder = NULL;
	}
}

void android_vh_alter_futex_plist_add_handler(void *unused, struct plist_node *node,
		    struct plist_head *head, bool *already_on_hb)
{
	struct sched_entity *se;
	struct rq *rq;
	struct rq_flags rf;
	struct futex_q *this, *next, *cur;
	struct task_struct *first_normal_waiter = NULL;
	u64 sleep_start;
	int prio;
	struct oplus_task_struct *ots;

	if (unlikely(!locking_opt_enable(LK_FUTEX_ENABLE) || *already_on_hb))
		return;

	if (!curr_is_ux_thread())
		return;

	ots = get_oplus_task_struct(current);
	if (ots->lkinfo.holder)
		boost_holder(ots->lkinfo.holder, current);

	cur = (struct futex_q *) node;
	/*
	 * Find out the first normal thread in &hb->chain(FIFO if it's a normal node),
	 * make sure &hb->lock is held.
	 */
	plist_for_each_entry_safe(this, next, head, list) {
		struct plist_node *tmp = &this->list;

		if (match_futex(&this->key, &cur->key)) {
			if (tmp->prio == MAX_RT_PRIO) {
				first_normal_waiter = this->task;
				break;
			}
		}
	}

	if (first_normal_waiter == NULL)
		goto re_init;

	rq = task_rq_lock(first_normal_waiter, &rf);
	se = &first_normal_waiter->se;
	sleep_start = schedstat_val(se->statistics.sleep_start);

	if (sleep_start) {
		u64 delta = rq_clock(rq) - sleep_start;

		if ((s64)delta < 0)
			delta = 0;

		if (delta > FUTEX_WAITER_TOLERATE_THRESHOLD) {
			cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
				"futex waiter(comm=%-12s pid=%d tgid=%d sleep_start=%llu) has blocked too long(%llu ns), do not rob it, please\n",
				first_normal_waiter->comm, first_normal_waiter->pid,
				first_normal_waiter->tgid, sleep_start, delta);
			task_rq_unlock(rq, first_normal_waiter, &rf);
			return;
		}
	}
	task_rq_unlock(rq, first_normal_waiter, &rf);

re_init:
	/*
	 * &hb->chain's pnode should be one of below:
	 * RT thread: pnode's prio is p->normal_prio.
	 * UX thread: pnode's prio is MAX_RT_PRIO - 1.
	 * Normal thread: pnode's prio is MAX_RT_PRIO.
	 */
	prio = min(current->normal_prio, MAX_RT_PRIO - 1);
	plist_node_init(node, prio);

	cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
		"insert current(comm=%-12s pid=%d tgid=%d ux_state=%d) before normal waiter(comm=%-12s pid=%d tgid=%d)\n",
		current->comm, current->pid, current->tgid,
		oplus_get_ux_state(current),
		first_normal_waiter ? first_normal_waiter->comm : "null",
		first_normal_waiter ? first_normal_waiter->pid : -1,
		first_normal_waiter ? first_normal_waiter->tgid : -1);
}

static void android_vh_futex_sleep_start_handler(void *unused,
		  struct task_struct *p)
{
	struct oplus_task_struct *ots;

	ots = get_oplus_task_struct(p);
	if (unlikely(IS_ERR_OR_NULL(ots)))
		return;
}

static void android_vh_futex_wake_traverse_plist_handler(void *unused, struct plist_head *chain,
		  int *target_nr, union futex_key key, u32 bitset)
{
	struct futex_q *this, *next;
	struct task_struct *p;
	struct oplus_task_struct *ots;
	int idx = 0;

	*target_nr = 0;
	if (likely(!curr_is_inherit_futex()))
		return;

	/*
	 * If waker is an inherit-futex ux thread, see how many ux waiters in this chain,
	 * make sure &hb->lock is held.
	 */
	plist_for_each_entry_safe(this, next, chain, list) {
		if (match_futex(&this->key, &key)) {
			p = this->task;
			ots = get_oplus_task_struct(p);
			if (unlikely(IS_ERR_OR_NULL(ots)))
				continue;

			if ((ots->lkinfo.holder == current) && ots->lkinfo.ux_contrib) {
				*target_nr += 1;
				ots->lkinfo.ux_contrib = false;
			}
			cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
				"idx=%d this=%-12s pid=%d tgid=%d nr=%d\n",
				++idx, p->comm, p->pid, p->tgid, *target_nr);
		}
	}
}

static void android_vh_futex_wake_this_handler(void *unused, int ret, int nr_wake, int target_nr,
		  struct task_struct *p)
{
	struct oplus_task_struct *ots;

	ots = get_oplus_task_struct(p);
	if (unlikely(IS_ERR_OR_NULL(ots)))
		return;

	if (ots->lkinfo.ux_contrib && ots->lkinfo.holder) {
		futex_unset_inherit_ux_refs(ots->lkinfo.holder, 1, 2);
		ots->lkinfo.ux_contrib = false;
	}

	if (likely(!curr_is_inherit_futex()))
		return;

	cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
		"this(%-12s pid=%d) ret=%d nr_wake=%d target_nr=%d curr(%-12s pid=%d inherit_ux=%llx)\n",
		p->comm, p->pid,
		ret, nr_wake, target_nr,
		current->comm, current->pid,
		oplus_get_inherit_ux(current));
}

static void android_vh_futex_wake_up_q_finish_handler(void *unused, int nr_wake,
		  int target_nr)
{
	if (likely(!curr_is_inherit_futex()))
		return;

	/* Owner will be changed, remove current's inherit count. */
	if (target_nr) {
		futex_unset_inherit_ux_refs(current, target_nr, 1);
		cond_trace_printk(locking_opt_debug(LK_DEBUG_FTRACE),
			"comm=%-12s pid=%d tgid=%d nr=%d inherit_ux=%llx\n",
			current->comm, current->pid, current->tgid,
			target_nr, oplus_get_inherit_ux(current));
	}
}

void register_futex_vendor_hooks(void)
{
	register_trace_android_vh_do_futex(
		android_vh_do_futex_handler,
		NULL);
	register_trace_android_vh_futex_wait_start(
		android_vh_futex_wait_start_handler,
		NULL);
	register_trace_android_vh_futex_wait_end(
		android_vh_futex_wait_end_handler,
		NULL);
	register_trace_android_vh_alter_futex_plist_add(
		android_vh_alter_futex_plist_add_handler,
		NULL);
	register_trace_android_vh_futex_sleep_start(
		android_vh_futex_sleep_start_handler,
		NULL);
	register_trace_android_vh_futex_wake_traverse_plist(
		android_vh_futex_wake_traverse_plist_handler,
		NULL);
	register_trace_android_vh_futex_wake_this(
		android_vh_futex_wake_this_handler,
		NULL);
	register_trace_android_vh_futex_wake_up_q_finish(
		android_vh_futex_wake_up_q_finish_handler,
		NULL);
}

void unregister_futex_vendor_hooks(void)
{
	unregister_trace_android_vh_do_futex(
		android_vh_do_futex_handler,
		NULL);
	unregister_trace_android_vh_futex_wait_start(
		android_vh_futex_wait_start_handler,
		NULL);
	unregister_trace_android_vh_futex_wait_end(
		android_vh_futex_wait_end_handler,
		NULL);
	unregister_trace_android_vh_alter_futex_plist_add(
		android_vh_alter_futex_plist_add_handler,
		NULL);
	unregister_trace_android_vh_futex_sleep_start(
		android_vh_futex_sleep_start_handler,
		NULL);
	unregister_trace_android_vh_futex_wake_traverse_plist(
		android_vh_futex_wake_traverse_plist_handler,
		NULL);
	unregister_trace_android_vh_futex_wake_this(
		android_vh_futex_wake_this_handler,
		NULL);
	unregister_trace_android_vh_futex_wake_up_q_finish(
		android_vh_futex_wake_up_q_finish_handler,
		NULL);
}
