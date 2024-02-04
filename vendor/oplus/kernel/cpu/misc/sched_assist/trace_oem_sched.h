/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 MediaTek Inc.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM sched_assist

#if !defined(_TRACE_OEM_SCHED_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_OEM_SCHED_H

#include <linux/tracepoint.h>


#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
extern void printf_cpu_spread_nr_info(unsigned int cpu, char *nr_info, int info_size);

TRACE_EVENT(sched_assist_spread_tasks,

	TP_PROTO(struct task_struct *p, int sched_type, int lowest_nr_cpu),

	TP_ARGS(p, sched_type, lowest_nr_cpu),

	TP_STRUCT__entry(
		__field(int,	pid)
		__array(char,	comm, TASK_COMM_LEN)
		__field(int,	sched_type)
		__field(int,	lowest_nr_cpu)
		__array(char,	nr_info_0,	32)
		__array(char,	nr_info_1,	32)
		__array(char,	nr_info_2,	32)
		__array(char,	nr_info_3,	32)
		__array(char,	nr_info_4,	32)
		__array(char,	nr_info_5,	32)
		__array(char,	nr_info_6,	32)
		__array(char,	nr_info_7,	32)
	),

	TP_fast_assign(
		__entry->pid			= p->pid;
		memcpy(__entry->comm, p->comm, TASK_COMM_LEN);
		__entry->sched_type		= sched_type;
		__entry->lowest_nr_cpu	= lowest_nr_cpu;
		printf_cpu_spread_nr_info(0, __entry->nr_info_0, sizeof(__entry->nr_info_0));
		printf_cpu_spread_nr_info(1, __entry->nr_info_1, sizeof(__entry->nr_info_1));
		printf_cpu_spread_nr_info(2, __entry->nr_info_2, sizeof(__entry->nr_info_2));
		printf_cpu_spread_nr_info(3, __entry->nr_info_3, sizeof(__entry->nr_info_3));
		printf_cpu_spread_nr_info(4, __entry->nr_info_4, sizeof(__entry->nr_info_4));
		printf_cpu_spread_nr_info(5, __entry->nr_info_5, sizeof(__entry->nr_info_5));
		printf_cpu_spread_nr_info(6, __entry->nr_info_6, sizeof(__entry->nr_info_6));
		printf_cpu_spread_nr_info(7, __entry->nr_info_7, sizeof(__entry->nr_info_7));
	),

	TP_printk("comm=%-12s pid=%d sched_type=%d lowest_nr_cpu=%d nr_info=%s%s%s%s%s%s%s%s",
		__entry->comm, __entry->pid, __entry->sched_type, __entry->lowest_nr_cpu,
		__entry->nr_info_0, __entry->nr_info_1, __entry->nr_info_2, __entry->nr_info_3,
		__entry->nr_info_4, __entry->nr_info_5, __entry->nr_info_6, __entry->nr_info_7)
);
#endif

#endif /*_TRACE_OEM_SCHED_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../kernel/sched/walt/oem_sched

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace_oem_sched
/* This part must be outside protection */
#include <trace/define_trace.h>
