/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 Google, Inc.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM binder_sched

#if !defined(_BINDER_SCHED_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _BINDER_SCHED_TRACE_H

#include <linux/types.h>
#include <linux/sched.h>
#include <drivers/android/binder_internal.h>
#include <uapi/linux/android/binder.h>
#include "binder_sched.h"
#include <linux/tracepoint.h>

TRACE_EVENT(binder_t_obs,
	TP_PROTO(struct binder_transaction *t, struct oplus_binder_struct *obs, char *info),
	TP_ARGS(t, obs, info),

	TP_STRUCT__entry(
		__field(struct binder_transaction *, t)
		__field(unsigned int, flags)
		__field(struct oplus_binder_struct *, obs)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->t = t;
		__entry->flags = (t ? t->flags : 0);
		__entry->obs = obs;
		__entry->info = info;
	),
	TP_printk("t=0x%llx flags=0x%x obs=0x%llx info=%s", (unsigned long long)__entry->t,
		__entry->flags, (unsigned long long)__entry->obs, __entry->info)
);

TRACE_EVENT(binder_get_obs,
	TP_PROTO(struct binder_transaction *t, struct oplus_binder_struct *obs, int alloc, char *info),
	TP_ARGS(t, obs, alloc, info),

	TP_STRUCT__entry(
		__field(struct binder_transaction *, t)
		__field(struct oplus_binder_struct *, obs)
		__field(int, alloc)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->t = t;
		__entry->obs = obs;
		__entry->alloc = alloc;
		__entry->info = info;
	),
	TP_printk("t=0x%llx obs=0x%llx alloc=%d info=%s", (unsigned long long)__entry->t,
		(unsigned long long)__entry->obs, __entry->alloc, __entry->info)
);

TRACE_EVENT(binder_ux_task,
	TP_PROTO(int sync, int pending_async, int set_ux, struct task_struct *ux_task, int ux_enable, struct binder_transaction *t,
		struct oplus_binder_struct *obs, char *info),
	TP_ARGS(sync, pending_async, set_ux, ux_task, ux_enable, t, obs, info),

	TP_STRUCT__entry(
		__field(int, sync)
		__field(int, pending_async)
		__field(int, set_ux)
		__field(int, pid)
		__field(int, tgid)
		__field(char *, comm)
		__field(int, ux_enable)
		__field(struct binder_transaction *, t)
		__field(struct oplus_binder_struct *, obs)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->sync = sync;
		__entry->pending_async = pending_async;
		__entry->set_ux = set_ux;
		__entry->pid = (ux_task ? ux_task->pid : 0);
		__entry->tgid = (ux_task ? ux_task->tgid : 0);
		__entry->comm = (ux_task ? ux_task->comm : "null");
		__entry->ux_enable = ux_enable;
		__entry->t = t;
		__entry->obs = obs;
		__entry->info = info;
	),
	TP_printk("sync=%d pending_async=%d set_ux=%d pid=%d tgid=%d comm=%s ux_enable=%d t=0x%llx obs=0x%llx info=%s",
		__entry->sync, __entry->pending_async, __entry->set_ux, __entry->pid, __entry->tgid, __entry->comm,
		__entry->ux_enable, (unsigned long long)__entry->t, (unsigned long long)__entry->obs, __entry->info)
);

TRACE_EVENT(binder_tr_buffer,
	TP_PROTO(struct binder_transaction_data *tr, int ret, unsigned int value,
		struct binder_transaction *t, struct oplus_binder_struct *obs, char *info),
	TP_ARGS(tr, ret, value, t, obs, info),

	TP_STRUCT__entry(
		__field(struct binder_transaction_data *, tr)
		__field(unsigned long, data_size)
		__field(int, ret)
		__field(unsigned int, value)
		__field(struct binder_transaction *, t)
		__field(struct oplus_binder_struct *, obs)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->tr = tr;
		__entry->data_size = (tr ? tr->data_size : 0);
		__entry->ret = ret;
		__entry->value = value;
		__entry->t = t;
		__entry->obs = obs;
		__entry->info = info;
	),
	TP_printk("tr=0x%llx data_size=%ld ret=%d value=0x%x t=0x%llx obs=0x%llx info=%s",
		(unsigned long long)__entry->tr, __entry->data_size, __entry->ret, __entry->value,
		(unsigned long long)__entry->t, (unsigned long long)__entry->obs, __entry->info)
);

TRACE_EVENT(binder_ux_enable,
	TP_PROTO(struct task_struct *task, int enable, struct binder_transaction *t,
		struct oplus_binder_struct *obs, char *info),
	TP_ARGS(task, enable, t, obs, info),

	TP_STRUCT__entry(
		__field(int, pid)
		__field(int, tgid)
		__field(char *, comm)
		__field(int, enable)
		__field(struct binder_transaction *, t)
		__field(struct oplus_binder_struct *, obs)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->pid = (task ? task->pid : 0);
		__entry->tgid = (task ? task->tgid : 0);
		__entry->comm = (task ? task->comm : "null");
		__entry->enable = enable;
		__entry->t = t;
		__entry->obs = obs;
		__entry->info = info;
	),
	TP_printk("task(pid=%d tgid=%d comm=%s) ux_enable=%d t=0x%llx obs=0x%llx info=%s",
		__entry->pid, __entry->tgid, __entry->comm, __entry->enable, (unsigned long long)__entry->t,
		(unsigned long long)__entry->obs, __entry->info)
);

TRACE_EVENT(binder_set_get_ux,
	TP_PROTO(struct task_struct *task, int pid, int enable, char *info),
	TP_ARGS(task, pid, enable, info),

	TP_STRUCT__entry(
		__field(int, set_pid)
		__field(int, pid)
		__field(int, tgid)
		__field(char *, comm)
		__field(int, enable)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->set_pid = pid;
		__entry->pid = (task ? task->pid : 0);;
		__entry->tgid = (task ? task->tgid : 0);
		__entry->comm = (task ? task->comm : "null");
		__entry->enable = enable;
		__entry->info = info;
	),
	TP_printk("task(set_pid=%d pid=%d tgid=%d comm=%s) enable=%d info=%s",
		__entry->set_pid, __entry->pid, __entry->tgid, __entry->comm, __entry->enable, __entry->info)
);


TRACE_EVENT(binder_ux_work,
	TP_PROTO(struct binder_work *work, struct list_head *target_list, struct list_head *w_entry,
		bool insert, int ux_count, char *info),
	TP_ARGS(work, target_list, w_entry, insert, ux_count, info),

	TP_STRUCT__entry(
		__field(struct binder_work *, work)
		__field(struct list_head *, target_list)
		__field(struct list_head *, w_entry)
		__field(bool, insert)
		__field(int, ux_count)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->work = work;
		__entry->target_list = target_list;
		__entry->w_entry = w_entry;
		__entry->insert = insert;
		__entry->ux_count = ux_count;
		__entry->info = info;
	),
	TP_printk("work=0x%llx target_list=0x%llx w_entry=0x%llx insert=%d sync_or_ux_count=%d info=%s",
		(unsigned long long)__entry->work, (unsigned long long)__entry->target_list,
		(unsigned long long)__entry->w_entry, __entry->insert, __entry->ux_count, __entry->info)
);

TRACE_EVENT(binder_proc_thread,
	TP_PROTO(struct task_struct *proc_task, struct task_struct *thread_task,
		int sync, int pending_async, struct binder_transaction *t, struct binder_proc *proc, char *info),
	TP_ARGS(proc_task, thread_task, sync, pending_async, t, proc, info),

	TP_STRUCT__entry(
		__field(int, proc_pid)
		__field(int, proc_tgid)
		__field(char *, proc_comm)
		__field(int, thread_pid)
		__field(int, thread_tgid)
		__field(char *, thread_comm)
		__field(unsigned int, code)
		__field(int, sync)
		__field(int, pending_async)
		__field(struct binder_transaction *, t)
		__field(struct binder_proc *, proc)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->proc_pid = (proc_task ? proc_task->pid : 0);
		__entry->proc_tgid = (proc_task ? proc_task->tgid : 0);
		__entry->proc_comm = (proc_task ? proc_task->comm : "null");
		__entry->thread_pid = (thread_task ? thread_task->pid : 0);
		__entry->thread_tgid = (thread_task ? thread_task->tgid : 0);
		__entry->thread_comm = (thread_task ? thread_task->comm : "null");
		__entry->code = (t ? t->code : 0);
		__entry->sync = sync;
		__entry->pending_async = pending_async;
		__entry->t = t;
		__entry->proc = proc;
		__entry->info = info;
	),
	TP_printk("proc_tsk(pid=%d tgid=%d comm=%s) thread_tsk(pid=%d tgid=%d comm=%s) code=%d sync=%d pending_async=%d t=0x%llx proc=0x%llx info=%s",
		__entry->proc_pid, __entry->proc_tgid, __entry->proc_comm,
		__entry->thread_pid, __entry->thread_tgid, __entry->thread_comm,
		__entry->code, __entry->sync, __entry->pending_async, (unsigned long long)__entry->t,
		(unsigned long long)__entry->proc, __entry->info)
);

TRACE_EVENT(get_async_thread,
	TP_PROTO(struct binder_proc *proc, struct binder_thread *thread, int count,
		struct binder_node *ots_node, struct binder_node *node, int has_async, ktime_t time, char *info),
	TP_ARGS(proc, thread, count, ots_node, node, has_async, time, info),

	TP_STRUCT__entry(
		__field(int, proc_pid)
		__field(int, proc_tgid)
		__field(char *, proc_comm)
		__field(int, thread_pid)
		__field(int, thread_tgid)
		__field(char *, thread_comm)
		__field(int, count)
		__field(int, max_threads)
		__field(int, requested_threads)
		__field(int, requested_threads_started)
		__field(struct binder_node *, ots_node)
		__field(struct binder_node *, node)
		__field(int, has_async)
		__field(ktime_t, time)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->proc_pid = (proc ? proc->tsk->pid : 0);
		__entry->proc_tgid = (proc ? proc->tsk->tgid : 0);
		__entry->proc_comm = (proc ? proc->tsk->comm : "null");
		__entry->thread_pid = (thread ? thread->task->pid : 0);
		__entry->thread_tgid = (thread ? thread->task->tgid : 0);
		__entry->thread_comm = (thread ? thread->task->comm : 0);
		__entry->count = count;
		__entry->max_threads = (proc ? proc->max_threads : 0);
		__entry->requested_threads = (proc ? proc->requested_threads : 0);
		__entry->requested_threads_started = (proc ? proc->requested_threads_started : 0);
		__entry->ots_node = ots_node;
		__entry->node = node;
		__entry->has_async = has_async;
		__entry->time = time,
		__entry->info = info;
	),
	TP_printk("proc_tsk(pid=%d tgid=%d comm=%s) thread_tsk(pid=%d tgid=%d comm=%s) count=%d max_threads=%d request=%d started=%d ots_node=0x%llx node=0x%llx has_async=%d time=%lldns info=%s",
		__entry->proc_pid, __entry->proc_tgid, __entry->proc_comm, __entry->thread_pid, __entry->thread_tgid,
		__entry->thread_comm, __entry->count, __entry->max_threads, __entry->requested_threads,
		__entry->requested_threads_started, (unsigned long long)__entry->ots_node,
		(unsigned long long)__entry->node, __entry->has_async, __entry->time, __entry->info)
);

TRACE_EVENT(set_thread_mode,
	TP_PROTO(struct task_struct *task, struct binder_node *node, bool sync, char *info),
	TP_ARGS(task, node, sync, info),

	TP_STRUCT__entry(
		__field(struct task_struct *, task)
		__field(int, pid)
		__field(int, tgid)
		__field(char *, comm)
		__field(struct binder_node *, node)
		__field(bool, sync)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->task = task;
		__entry->pid = (task ? task->pid : 0);
		__entry->tgid = (task ? task->tgid : 0);
		__entry->comm = (task ? task->comm : "null");
		__entry->node = node;
		__entry->sync = sync;
		__entry->info = info;
	),
	TP_printk("task(pid=%d tgid=%d comm=%s) node=0x%llx sync=%d info=%s",
		__entry->pid, __entry->tgid, __entry->comm,
		(unsigned long long)__entry->node, __entry->sync, __entry->info)
);

TRACE_EVENT(binder_free_buf,
	TP_PROTO(struct binder_proc *proc, struct binder_thread *thread, struct binder_buffer *buffer, char *info),
	TP_ARGS(proc, thread, buffer, info),

	TP_STRUCT__entry(
		__field(int, proc_pid)
		__field(int, proc_tgid)
		__field(char *, proc_comm)
		__field(int, thread_pid)
		__field(int, thread_tgid)
		__field(char *, thread_comm)
		__field(struct binder_transaction *, t)
		__field(struct binder_buffer *, buffer)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->proc_pid = (proc ? proc->tsk->pid : 0);
		__entry->proc_tgid = (proc ? proc->tsk->tgid : 0);
		__entry->proc_comm = (proc ? proc->tsk->comm : "null");
		__entry->thread_pid = (thread ? thread->task->pid : 0);
		__entry->thread_tgid = (thread ? thread->task->tgid : 0);
		__entry->thread_comm = (thread ? thread->task->comm : "null");
		__entry->t = (buffer ? buffer->transaction : 0);
		__entry->buffer = buffer;
		__entry->info = info;
	),
	TP_printk("proc(pid=%d tgid=%d comm=%s) thread(pid=%d tgid=%d comm=%s) t=0x%llx buffer=0x%llx info=%s",
		__entry->proc_pid, __entry->proc_tgid, __entry->proc_comm, __entry->thread_pid, __entry->thread_tgid,
		__entry->thread_comm, (unsigned long long)__entry->t, (unsigned long long)__entry->buffer, __entry->info)
);

TRACE_EVENT(binder_inherit_ux,
	TP_PROTO(struct task_struct *from, struct task_struct *target,
		int ux_depth, int ux_state, int type, int async_ux_sts, bool sync, char *info),
	TP_ARGS(from, target, ux_depth, ux_state, type, async_ux_sts, sync, info),

	TP_STRUCT__entry(
		__field(int, from_pid)
		__field(int, from_tgid)
		__field(char *, from_comm)
		__field(int, target_pid)
		__field(int, target_tgid)
		__field(char *, target_comm)
		__field(int, ux_depth)
		__field(int, ux_state)
		__field(int, type)
		__field(int, async_ux_sts)
		__field(bool, sync)
		__field(char *, info)
	),
	TP_fast_assign(
		__entry->from_pid = (from ? from->pid : 0);
		__entry->from_tgid = (from ? from->tgid : 0);
		__entry->from_comm = (from ? from->comm : "null");
		__entry->target_pid = (target ? target->pid : 0);
		__entry->target_tgid = (target ? target->tgid : 0);
		__entry->target_comm = (target ? target->comm : "null");
		__entry->ux_depth = ux_depth;
		__entry->ux_state = ux_state;
		__entry->type = type;
		__entry->async_ux_sts = async_ux_sts;
		__entry->sync = sync;
		__entry->info = info;
	),
	TP_printk("from(pid=%d tgid=%d comm=%s) target(pid=%d tgid=%d comm=%s) depth=%d state=%d type=%d async_ux_sts=%d sync=%d info=%s",
		__entry->from_pid, __entry->from_tgid, __entry->from_comm,
		__entry->target_pid, __entry->target_tgid, __entry->target_comm,
		__entry->ux_depth, __entry->ux_state, __entry->type, __entry->async_ux_sts,
		__entry->sync, __entry->info)
);

#endif /* _BINDER_SCHED_TRACE_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE binder_sched_trace

/* This part must be outside protection */
#include <trace/define_trace.h>
