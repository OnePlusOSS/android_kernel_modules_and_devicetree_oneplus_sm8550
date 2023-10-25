// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/rwsem.h>
#include <../kernel/sched/sched.h>
#include <trace/hooks/rwsem.h>
#include <trace/hooks/dtask.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#endif

#include "locking_main.h"
#ifdef CONFIG_OPLUS_LOCKING_OSQ
#include "rwsem.h"
#endif
#ifdef CONFIG_OPLUS_LOCKING_MONITOR
#include "kern_lock_stat.h"
#endif

#define RWSEM_READER_OWNED	(1UL << 0)
#define RWSEM_RD_NONSPINNABLE	(1UL << 1)
#define RWSEM_WR_NONSPINNABLE	(1UL << 2)
#define RWSEM_NONSPINNABLE	(RWSEM_RD_NONSPINNABLE | RWSEM_WR_NONSPINNABLE)
#define RWSEM_OWNER_FLAGS_MASK	(RWSEM_READER_OWNED | RWSEM_NONSPINNABLE)

#define RWSEM_WRITER_LOCKED	(1UL << 0)
#define RWSEM_WRITER_MASK	RWSEM_WRITER_LOCKED

/*
 * Note:
 * The following macros must be the same as in kernel/locking/rwsem.c
 */
#define RWSEM_FLAG_WAITERS	(1UL << 1)
#define RWSEM_FLAG_HANDOFF	(1UL << 2)

#define rwsem_first_waiter(sem) \
	list_first_entry(&sem->wait_list, struct rwsem_waiter, list)

static inline struct task_struct *rwsem_owner(struct rw_semaphore *sem)
{
	return (struct task_struct *)
		(atomic_long_read(&sem->owner) & ~RWSEM_OWNER_FLAGS_MASK);
}

static inline bool rwsem_test_oflags(struct rw_semaphore *sem, long flags)
{
	return atomic_long_read(&sem->owner) & flags;
}

static inline bool is_rwsem_reader_owned(struct rw_semaphore *sem)
{
#if IS_ENABLED(CONFIG_DEBUG_RWSEMS)
	/*
	 * Check the count to see if it is write-locked.
	 */
	long count = atomic_long_read(&sem->count);

	if (count & RWSEM_WRITER_MASK)
		return false;
#endif
	return rwsem_test_oflags(sem, RWSEM_READER_OWNED);
}

static bool rwsem_list_add_ux(struct list_head *entry, struct list_head *head, bool handoff_set)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct rwsem_waiter *waiter = NULL;
	struct rwsem_waiter *ux_waiter = NULL;
	struct task_struct *task;

	if (!entry || !head)
		return false;

	ux_waiter = list_entry(entry, struct rwsem_waiter, list);

	list_for_each_safe(pos, n, head) {
		/* Skip the first handoff-set waiter. */
		if (unlikely(handoff_set)) {
			handoff_set = false;
			continue;
		}

		waiter = list_entry(pos, struct rwsem_waiter, list);
		task = waiter->task;
		if (!task)
			continue;

		if (waiter->task->prio > MAX_RT_PRIO && !test_task_ux(waiter->task)) {
			list_add(entry, waiter->list.prev);
			ux_waiter->timeout = waiter->timeout;
			return true;
		}
	}

	list_add_tail(entry, head);
	return true;
}

static void rwsem_set_inherit_ux(struct rw_semaphore *sem)
{
	bool is_ux = test_set_inherit_ux(current);
	struct task_struct *owner = rwsem_owner(sem);

	/* set writer as ux task */
	if (is_ux && !is_rwsem_reader_owned(sem) && !test_inherit_ux(owner, INHERIT_UX_RWSEM)) {
		int type = get_ux_state_type(owner);

		if ((type == UX_STATE_NONE) || (type == UX_STATE_INHERIT))
			set_inherit_ux(owner, INHERIT_UX_RWSEM, oplus_get_ux_depth(current),
				oplus_get_ux_state(current));
	}
}

static void rwsem_unset_inherit_ux(struct rw_semaphore *sem)
{
	if (test_inherit_ux(current, INHERIT_UX_RWSEM))
		unset_inherit_ux(current, INHERIT_UX_RWSEM);
}

#ifdef RWSEM_DEBUG
static unsigned long list_count(struct list_head *list)
{
	struct list_head *pos;
	unsigned long count = 0;

	list_for_each(pos, list)
		count++;

	return count;
}

void pre_dump_waitlist(struct rwsem_waiter *curr, struct rw_semaphore *sem)
{
	struct rwsem_waiter *waiter, *tmp;
	struct task_struct *task;
	int id = 0;

	if (!curr || !sem)
		return;

	if (!test_task_ux(curr->task))
		return;

	task = curr->task;
	trace_printk("DBG_RWSEM [%s$%d]: [X] [%s] [%s]: task=%s, pid=%d, tgid=%d, prio=%d, timeout=%lu, handoff=%d, empty=%d, count=%lu\n",
		__func__, __LINE__,
		(curr->type == RWSEM_WAITING_FOR_READ) ? "R" : "W",
		test_task_ux(task) ? "Y" : "N",
		task->comm, task->pid, task->tgid, task->prio, curr->timeout,
		atomic_long_read(&sem->count) & RWSEM_FLAG_HANDOFF,
		list_empty(&sem->wait_list), list_count(&sem->wait_list));

	list_for_each_entry_safe(waiter, tmp, &sem->wait_list, list) {
		task = waiter->task;
		if (!task)
			continue;

		trace_printk("DBG_RWSEM [%s$%d]: [%d] [%s] [%s]: task=%s, pid=%d, tgid=%d, prio=%d, timeout=%lu\n",
			__func__, __LINE__, id++,
			(waiter->type == RWSEM_WAITING_FOR_READ) ? "R" : "W",
			test_task_ux(task) ? "Y" : "N",
			task->comm, task->pid, task->tgid, task->prio, waiter->timeout);
	}
}

void post_dump_waitlist(struct rwsem_waiter *curr, struct rw_semaphore *sem)
{
	struct rwsem_waiter *waiter, *tmp;
	struct task_struct *task;
	int id = 0;

	if (!curr || !sem)
		return;

	if (!test_task_ux(curr->task))
		return;

	list_for_each_entry_safe(waiter, tmp, &sem->wait_list, list) {
		task = waiter->task;
		if (!task)
			continue;

		trace_printk("DBG_RWSEM [%s$%d]: [%d] [%s] [%s]: task=%s, pid=%d, tgid=%d, prio=%d, timeout=%lu\n",
			__func__, __LINE__, id++,
			(waiter->type == RWSEM_WAITING_FOR_READ) ? "R" : "W",
			test_task_ux(task) ? "Y" : "N",
			task->comm, task->pid, task->tgid, task->prio, waiter->timeout);
	}
}
#endif

/* timeout is 5s */
#define WAIT_TIMEOUT		5000

inline bool test_wait_timeout(struct rw_semaphore *sem)
{
	struct rwsem_waiter *waiter;
	unsigned long timeout;
	struct task_struct *task;
	long count;
	bool ret = false;

	if (!sem)
		return false;

	count = atomic_long_read(&sem->count);
	if (!(count & RWSEM_FLAG_WAITERS))
		return false;

	waiter = rwsem_first_waiter(sem);
	if (!waiter)
		return false;

	timeout = waiter->timeout;
	task = waiter->task;
	if (!task)
		return false;

	ret = time_is_before_jiffies(timeout + msecs_to_jiffies(WAIT_TIMEOUT));
#ifdef RWSEM_DEBUG
	if (ret) {
		trace_printk("DBG_RWSEM [%s$%d]: task=%s, pid=%d, tgid=%d, prio=%d, ux=%d, timeout=%lu(0x%x), t_m=%lu(0x%x), jiffies=%lu(0x%x)\n",
			__func__, __LINE__,
			task->comm, task->pid, task->tgid, task->prio, test_task_ux(task),
			timeout, timeout,
			timeout + msecs_to_jiffies(WAIT_TIMEOUT), timeout + msecs_to_jiffies(WAIT_TIMEOUT),
			jiffies, jiffies);
	}
#endif

	return ret;
}

/* implement vender hook in kernel/locking/rwsem.c */
static void __android_vh_alter_rwsem_list_add_handler(struct rwsem_waiter *waiter,
			struct rw_semaphore *sem, bool *already_on_list)
{
	bool ret = false;
	bool handoff_set = false;

	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE)))
		return;

	if (!waiter || !sem)
		return;

	/*
	 * Avoid putting a read-waiter into the head of wait list,
	 * this will waking lots of read-waiters through rwsem_mark_wake().
	 */
	if (waiter->type == RWSEM_WAITING_FOR_READ)
		return;

	if (!test_task_ux(waiter->task))
		return;

	if (test_wait_timeout(sem))
		return;

#ifdef RWSEM_DEBUG
	pre_dump_waitlist(waiter, sem);
#endif

	/* See if the semaphore set a handoff bit. */
	if (atomic_long_read(&sem->count) & RWSEM_FLAG_HANDOFF)
		handoff_set = true;

	/* See if the first_waiter set a handoff bit. */
	if (!handoff_set && !list_empty(&sem->wait_list)) {
		struct rwsem_waiter *first_waiter;

		first_waiter = rwsem_first_waiter(sem);
		handoff_set = first_waiter->handoff_set;
	}

	ret = rwsem_list_add_ux(&waiter->list, &sem->wait_list, handoff_set);
#ifdef RWSEM_DEBUG
	post_dump_waitlist(waiter, sem);
#endif

	if (ret)
		*already_on_list = true;
}

#ifndef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
void oplus_android_vh_alter_rwsem_list_add(void *unused, struct rwsem_waiter *waiter,
		struct rw_semaphore *sem, bool *already_on_list)
{
	__android_vh_alter_rwsem_list_add_handler(waiter, sem, already_on_list);
}
EXPORT_SYMBOL(oplus_android_vh_alter_rwsem_list_add);
#endif

#ifdef CONFIG_OPLUS_LOCKING_OSQ
static void rwsem_update_ux_cnt_when_add(struct rw_semaphore *sem)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(current);
	struct oplus_rw_semaphore *osem = get_oplus_rw_semaphore(sem);

	if (unlikely(!ots))
		return;

	/* Record the ux flag when task is added to wait list */
	ots->lkinfo.is_block_ux = (ots->ux_state & 0xf) || (current->prio < MAX_RT_PRIO);
	if (ots->lkinfo.is_block_ux)
		atomic_long_inc(&osem->count);
}

static void rwsem_update_ux_cnt_when_remove(struct rw_semaphore *sem)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(current);
	struct oplus_rw_semaphore *osem = get_oplus_rw_semaphore(sem);

	if (unlikely(!ots))
		return;

	/*
	 * Update the ux tasks when tasks exit the waiter list.
	 * We only care about the recorded ux flag, but don't
	 * care about the current ux flag. This avoids the corruption
	 * of the number caused by ux change when in waiter list.
	 */
	if (ots->lkinfo.is_block_ux) {
		/* Count == 0! Something is wrong! */
		WARN_ON_ONCE(!atomic_long_read(&osem->count));
		atomic_long_add(-1, &osem->count);
		ots->lkinfo.is_block_ux = false;
	}
}
#endif

static void android_vh_alter_rwsem_list_add_handler(void *unused, struct rwsem_waiter *waiter,
			struct rw_semaphore *sem, bool *already_on_list)
{
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	if (likely(locking_opt_enable(LK_RWSEM_ENABLE)))
		__android_vh_alter_rwsem_list_add_handler(waiter, sem, already_on_list);
#endif

#ifdef CONFIG_OPLUS_LOCKING_OSQ
	if (likely(locking_opt_enable(LK_OSQ_ENABLE)))
		rwsem_update_ux_cnt_when_add(sem);
#endif
}

static void android_vh_rwsem_wake_handler(void *unused, struct rw_semaphore *sem)
{
	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE)))
		return;

	rwsem_set_inherit_ux(sem);
}

static void android_vh_rwsem_wake_finish_handler(void *unused, struct rw_semaphore *sem)
{
	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE)))
		return;

	rwsem_unset_inherit_ux(sem);
}

#ifdef CONFIG_OPLUS_LOCKING_OSQ
static int rwsem_opt_spin_time_threshold = NSEC_PER_USEC * 15;
static int rwsem_ux_opt_spin_time_threshold = NSEC_PER_USEC * 40;
static int rwsem_opt_spin_total_cnt = 0;
static int rwsem_opt_spin_timeout_exit_cnt = 0;
module_param(rwsem_opt_spin_time_threshold, int, 0644);
module_param(rwsem_ux_opt_spin_time_threshold, int, 0644);
module_param(rwsem_opt_spin_total_cnt, int, 0444);
module_param(rwsem_opt_spin_timeout_exit_cnt, int, 0444);

static void android_vh_rwsem_opt_spin_start_handler(void *unused,
		struct rw_semaphore *sem, bool *time_out, int *cnt, bool chk_only)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(current);
	u64 delta;

	if (unlikely(IS_ERR_OR_NULL(ots)))
		return;

	if (!ots->lkinfo.opt_spin_start_time) {
		if (chk_only)
			return;

		rwsem_opt_spin_total_cnt++;
		ots->lkinfo.opt_spin_start_time = sched_clock();
		return;
	}

	if (unlikely(!locking_opt_enable(LK_OSQ_ENABLE)))
		return;

	++(*cnt);
	/* Check whether time is out every 16 times in order to reduce the cost of calling sched_clock() */
	if ((*cnt) & 0xf)
		return;

	delta = sched_clock() - ots->lkinfo.opt_spin_start_time;
	if (((ots->ux_state & 0xf) && delta > rwsem_ux_opt_spin_time_threshold) ||
	    (!(ots->ux_state & 0xf) && delta > rwsem_opt_spin_time_threshold)) {
		rwsem_opt_spin_timeout_exit_cnt++;
		*time_out = true;
	}
}

static void android_vh_rwsem_opt_spin_finish_handler(void *unused,
		struct rw_semaphore *sem, bool taken)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(current);
#ifdef CONFIG_OPLUS_INTERNAL_VERSION
	u64 delta;
#endif

	if (unlikely(IS_ERR_OR_NULL(ots)))
		return;

	if (likely(ots->lkinfo.opt_spin_start_time)) {
#ifdef CONFIG_OPLUS_INTERNAL_VERSION
		delta = sched_clock() - ots->lkinfo.opt_spin_start_time;
		handle_wait_stats(OSQ_RWSEM_WRITE, delta);
#endif
		ots->lkinfo.opt_spin_start_time = 0;
	}
}

#define MAX_VALID_COUNT (1000)
static void android_vh_rwsem_can_spin_on_owner_handler(void *unused,
		struct rw_semaphore *sem, bool *ret)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(current);
	struct oplus_rw_semaphore *osem = get_oplus_rw_semaphore(sem);
	long block_ux_cnt;

	if (unlikely(!locking_opt_enable(LK_OSQ_ENABLE) || !ots))
		return;

	/* NOTE:kernel-5.15, this will always be writer, should just go? */
	/* ux or rt task just go */
	if ((ots->ux_state & 0xf) || (current->prio < MAX_RT_PRIO))
		return;

	block_ux_cnt = atomic_long_read(&osem->count);
	/* If some ux task is in the waiter list, non-ux can't optimistic spin. */
	if (block_ux_cnt > 0 && block_ux_cnt < MAX_VALID_COUNT)
		*ret = false;
}
#endif

static void android_vh_rwsem_read_wait_finish_handler(void *unused, struct rw_semaphore *sem)
{
#ifdef CONFIG_OPLUS_LOCKING_OSQ
	rwsem_update_ux_cnt_when_remove(sem);
#endif
}

static void android_vh_rwsem_write_wait_finish_handler(void *unused, struct rw_semaphore *sem)
{
#ifdef CONFIG_OPLUS_LOCKING_OSQ
	rwsem_update_ux_cnt_when_remove(sem);
#endif
}

void register_rwsem_vendor_hooks(void)
{
#ifdef CONFIG_OPLUS_LOCKING_OSQ
	BUILD_BUG_ON(sizeof(struct oplus_rw_semaphore) > sizeof(((struct rw_semaphore *)0)->android_oem_data1));
#endif
	register_trace_android_vh_alter_rwsem_list_add(android_vh_alter_rwsem_list_add_handler, NULL);
	register_trace_android_vh_rwsem_wake(android_vh_rwsem_wake_handler, NULL);
	register_trace_android_vh_rwsem_wake_finish(android_vh_rwsem_wake_finish_handler, NULL);
#ifdef CONFIG_OPLUS_LOCKING_OSQ
	register_trace_android_vh_rwsem_opt_spin_start(android_vh_rwsem_opt_spin_start_handler, NULL);
	register_trace_android_vh_rwsem_opt_spin_finish(android_vh_rwsem_opt_spin_finish_handler, NULL);
	register_trace_android_vh_rwsem_can_spin_on_owner(android_vh_rwsem_can_spin_on_owner_handler, NULL);
#endif
	register_trace_android_vh_rwsem_read_wait_finish(android_vh_rwsem_read_wait_finish_handler, NULL);
	register_trace_android_vh_rwsem_write_wait_finish(android_vh_rwsem_write_wait_finish_handler, NULL);
}

void unregister_rwsem_vendor_hooks(void)
{
	unregister_trace_android_vh_alter_rwsem_list_add(android_vh_alter_rwsem_list_add_handler, NULL);
	unregister_trace_android_vh_rwsem_wake(android_vh_rwsem_wake_handler, NULL);
	unregister_trace_android_vh_rwsem_wake_finish(android_vh_rwsem_wake_finish_handler, NULL);
#ifdef CONFIG_OPLUS_LOCKING_OSQ
	unregister_trace_android_vh_rwsem_opt_spin_start(android_vh_rwsem_opt_spin_start_handler, NULL);
	unregister_trace_android_vh_rwsem_opt_spin_finish(android_vh_rwsem_opt_spin_finish_handler, NULL);
	unregister_trace_android_vh_rwsem_can_spin_on_owner(android_vh_rwsem_can_spin_on_owner_handler, NULL);
#endif
	unregister_trace_android_vh_rwsem_read_wait_finish(android_vh_rwsem_read_wait_finish_handler, NULL);
	unregister_trace_android_vh_rwsem_write_wait_finish(android_vh_rwsem_write_wait_finish_handler, NULL);
}
