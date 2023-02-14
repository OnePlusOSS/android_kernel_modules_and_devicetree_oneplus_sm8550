/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 MediaTek Inc.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM sched_assist

#if !defined(_TRACE_SCHED_ASSIST_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SCHED_ASSIST_H

#include <linux/sched.h>
#include <linux/types.h>
#include <linux/tracepoint.h>
#include "sa_common.h"
#include "sa_fair.h"


TRACE_EVENT(set_ux_task_to_prefer_cpu,

	TP_PROTO(struct task_struct *p, int target_cpu, int strict_cpu, int cls_nr, int start_cls),

	TP_ARGS(p, target_cpu, strict_cpu, cls_nr, start_cls),

	TP_STRUCT__entry(
		__field(int,		pid)
		__array(char,		comm, TASK_COMM_LEN)
		__array(char,		cpus, 32)
		__field(unsigned long,	util)
		__field(int,		target_cpu)
		__field(int,		strict_cpu)
		__field(int,		cls_nr)
		__field(int,		start_cls)),

	TP_fast_assign(
		__entry->pid			= p->pid;
		memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
		__entry->util			= oplus_task_util(p);
		scnprintf(__entry->cpus, sizeof(__entry->cpus), "%*pbl", cpumask_pr_args(&p->cpus_mask));
		__entry->target_cpu		= target_cpu;
		__entry->strict_cpu		= strict_cpu;
		__entry->cls_nr			= cls_nr;
		__entry->start_cls		= start_cls;),

	TP_printk("pid=%d comm=%s util=%lu cpus_allowed=%s target_cpu=%d strict_cpu=%d cls_nr=%d start_cls=%d",
		__entry->pid, __entry->comm, __entry->util, __entry->cpus,
		__entry->target_cpu, __entry->strict_cpu, __entry->cls_nr, __entry->start_cls)
);

DECLARE_EVENT_CLASS(inherit_ux_template,

	TP_PROTO(struct task_struct *p, int type, int ux_state, s64 inherit_ux, int depth),

	TP_ARGS(p, type, ux_state, inherit_ux, depth),

	TP_STRUCT__entry(
		__field(int,	pid)
		__array(char,	comm, TASK_COMM_LEN)
		__field(int,	type)
		__field(int,	ux_state)
		__field(s64,	inherit_ux)
		__field(int,	depth)),

	TP_fast_assign(
		__entry->pid			= p->pid;
		memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
		__entry->type			= type;
		__entry->ux_state		= ux_state;
		__entry->inherit_ux		= inherit_ux;
		__entry->depth			= depth;),

	TP_printk("pid=%d comm=%s inherit_type=%d ux_state=%d inherit_ux=%llx ux_depth=%d",
		__entry->pid, __entry->comm, __entry->type, __entry->ux_state,
		__entry->inherit_ux, __entry->depth)
);

DEFINE_EVENT(inherit_ux_template, inherit_ux_set,
	TP_PROTO(struct task_struct *p, int type, int ux_state, s64 inherit_ux, int depth),
	TP_ARGS(p, type, ux_state, inherit_ux, depth));

DEFINE_EVENT(inherit_ux_template, inherit_ux_reset,
	TP_PROTO(struct task_struct *p, int type, int ux_state, s64 inherit_ux, int depth),
	TP_ARGS(p, type, ux_state, inherit_ux, depth));

DEFINE_EVENT(inherit_ux_template, inherit_ux_unset,
	TP_PROTO(struct task_struct *p, int type, int ux_state, s64 inherit_ux, int depth),
	TP_ARGS(p, type, ux_state, inherit_ux, depth));

#endif /*_TRACE_SCHED_ASSIST_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace_sched_assist
/* This part must be outside protection */
#include <trace/define_trace.h>

