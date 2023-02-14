// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/rwsem.h>
#include <../kernel/sched/sched.h>
#include <trace/hooks/rwsem.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#endif

#include "locking_main.h"

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

static void rwsem_list_add_ux(struct list_head *entry, struct list_head *head)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct rwsem_waiter *waiter = NULL;
	struct rwsem_waiter *ux_waiter = NULL;
	struct task_struct *task;

	ux_waiter = list_entry(entry, struct rwsem_waiter, list);

	list_for_each_safe(pos, n, head) {
		waiter = list_entry(pos, struct rwsem_waiter, list);

		task = waiter->task;
		if (!task)
			continue;

#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
		if (waiter->task->prio > MAX_RT_PRIO && !test_task_ux(waiter->task)) {
#else
		if (!test_task_ux(waiter->task)) {
#endif
			list_add(entry, waiter->list.prev);
			ux_waiter->timeout = waiter->timeout;
			return;
		}
	}

	if (pos == head)
		list_add_tail(entry, head);
}

bool oplus_rwsem_list_add(struct task_struct *tsk, struct list_head *entry, struct list_head *head)
{
	bool is_ux = false;

	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE)))
		return false;

	if (!entry || !head)
		return false;

	is_ux = test_task_ux(tsk);
	if (is_ux) {
		rwsem_list_add_ux(entry, head);
		return true;
	}

	return false;
}
EXPORT_SYMBOL(oplus_rwsem_list_add);

bool rwsem_list_add_skip_ux(struct task_struct *in_tsk, struct task_struct *tsk)
{
	if (unlikely(!locking_opt_enable(LK_RWSEM_ENABLE)))
		return false;

	if (in_tsk->prio > MAX_RT_PRIO && test_task_ux(tsk))
		return true;

	return false;
}
EXPORT_SYMBOL(rwsem_list_add_skip_ux);

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
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
static void android_vh_alter_rwsem_list_add_handler(void *unused, struct rwsem_waiter *waiter,
			struct rw_semaphore *sem, bool *already_on_list)
{
	bool ret = false;

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

	if (atomic_long_read(&sem->count) & RWSEM_FLAG_HANDOFF) {
		ret = oplus_rwsem_list_add(waiter->task, &waiter->list, sem->wait_list.next);
	} else {
		ret = oplus_rwsem_list_add(waiter->task, &waiter->list, &sem->wait_list);
	}

#ifdef RWSEM_DEBUG
	post_dump_waitlist(waiter, sem);
#endif

	if (ret)
		*already_on_list = true;
}
#endif

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

void register_rwsem_vendor_hooks(void)
{
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	register_trace_android_vh_alter_rwsem_list_add(android_vh_alter_rwsem_list_add_handler, NULL);
#endif
	register_trace_android_vh_rwsem_wake(android_vh_rwsem_wake_handler, NULL);
	register_trace_android_vh_rwsem_wake_finish(android_vh_rwsem_wake_finish_handler, NULL);
}

void unregister_rwsem_vendor_hooks(void)
{
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	unregister_trace_android_vh_alter_rwsem_list_add(android_vh_alter_rwsem_list_add_handler, NULL);
#endif
	unregister_trace_android_vh_rwsem_wake(android_vh_rwsem_wake_handler, NULL);
	unregister_trace_android_vh_rwsem_wake_finish(android_vh_rwsem_wake_finish_handler, NULL);
}

