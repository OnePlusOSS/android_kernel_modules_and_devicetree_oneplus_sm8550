/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */

#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/ratelimit.h>
#include <linux/ktime.h>
#include <linux/seq_file.h>
#include <linux/version.h>
#include <trace/events/sched.h>
#include <trace/hooks/mm.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/preemptirq.h>

#include "../sched_assist/sa_common.h"
#include "osi_healthinfo.h"
#include "osi_base.h"
#include "osi_preemptirq.h"

#define  PREEMPT_OFF_NS              (2000000)
#define  IRQ_OFF_NS                  (10000000)

static DEFINE_PER_CPU(u64, irq_disabled_ts);
static DEFINE_PER_CPU(u64, preempt_disabled_ts);

struct preemptirq_store {
	u64		ts;
	unsigned int	cnt;
	unsigned long callstack[4];
};

static DEFINE_PER_CPU(struct preemptirq_store, preempt_ps);
static DEFINE_PER_CPU(struct preemptirq_store, irq_ps);

static void irq_disable_begin(void *u1, unsigned long u2, unsigned long u3)
{
	if (is_idle_task(current))
		return;
	this_cpu_write(irq_disabled_ts, sched_clock());
}

static void irq_disable_end(void *u1, unsigned long ip, unsigned long parent_ip)
{
	struct preemptirq_store *cur_irq;
	u64 times, begin_times;

	begin_times = this_cpu_read(irq_disabled_ts);
	if (!begin_times)
		return;
	times = sched_clock() - begin_times;
	this_cpu_write(irq_disabled_ts, 0);
	if (is_idle_task(current) && times < IRQ_OFF_NS)
		return;
	cur_irq = &per_cpu(irq_ps, raw_smp_processor_id());
	if (cur_irq->ts < times) {
		cur_irq->ts = times;
		cur_irq->callstack[0] = CALLER_ADDR2;
		cur_irq->callstack[1] = CALLER_ADDR3;
		cur_irq->callstack[2] = CALLER_ADDR3;
		cur_irq->callstack[3] = CALLER_ADDR4;
	}
	cur_irq->cnt++;
}

static void preempt_disable_begin(void *u1, unsigned long u2, unsigned long u3)
{
	this_cpu_write(preempt_disabled_ts, sched_clock());
}

static void preempt_disable_end(void *u1, unsigned long ip, unsigned long parent_ip)
{
	struct preemptirq_store *cur_preempt;
	u64 times, begin_times;

	begin_times = this_cpu_read(preempt_disabled_ts);
	times = sched_clock() - begin_times;
	if (!begin_times || (times < PREEMPT_OFF_NS))
		return;
	this_cpu_write(preempt_disabled_ts, 0);
	cur_preempt = &per_cpu(preempt_ps, raw_smp_processor_id());
	if (cur_preempt->ts < times) {
		cur_preempt->ts = times;
		cur_preempt->callstack[0] = CALLER_ADDR2;
		cur_preempt->callstack[1] = CALLER_ADDR3;
		cur_preempt->callstack[2] = CALLER_ADDR4;
		cur_preempt->callstack[3] = CALLER_ADDR5;
	}
	cur_preempt->cnt++;
}

static int proc_osi_preemptirq_show(struct seq_file *m, void *v)
{
	int cpu;
	struct preemptirq_store *cur_preempt, *cur_irq;

	for_each_possible_cpu(cpu) {
		cur_preempt = &per_cpu(preempt_ps, cpu);
		seq_printf(m, "cpu:%d, %lu, %d, (%ps<-%ps<-%ps<-%ps)\n", cpu, cur_preempt->ts,
			cur_preempt->cnt, (void *)cur_preempt->callstack[0], (void *)cur_preempt->callstack[1],
			(void *)cur_preempt->callstack[2], (void *)cur_preempt->callstack[3]);
	}

	for_each_possible_cpu(cpu) {
		cur_irq = &per_cpu(irq_ps, cpu);
		seq_printf(m, "cpu:%d, %lu, %d, (%pS<-%pS<-%pS<-%pS)\n", cpu, cur_irq->ts,
			cur_irq->cnt, (void *)cur_irq->callstack[0], (void *)cur_irq->callstack[1],
			(void *)cur_irq->callstack[2], (void *)cur_irq->callstack[3]);
	}
	return 0;
}
static int proc_osi_preemptirq_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, proc_osi_preemptirq_show, inode);
}

static const struct proc_ops proc_osi_preemptirq_operations = {
	.proc_open	=	proc_osi_preemptirq_open,
	.proc_read	=	seq_read,
	.proc_lseek	=	seq_lseek,
	.proc_release   =	single_release,
};

static void osi_preemptirq_proc_init(struct proc_dir_entry *pde)
{
	struct proc_dir_entry *entry = NULL;
	entry = proc_create("osi_preemptirq", S_IRUGO | S_IWUGO,
				pde, &proc_osi_preemptirq_operations);
	if (!entry) {
		osi_err("create osi_preemptirq fail\n");
		return;
	}
}

static int register_long_preempt_vendor_hooks(void)
{
	int ret = 0;

	REGISTER_TRACE_VH(android_rvh_irqs_disable, irq_disable_begin);
	REGISTER_TRACE_VH(android_rvh_irqs_enable, irq_disable_end);
	REGISTER_TRACE_VH(android_rvh_preempt_disable, preempt_disable_begin);
	REGISTER_TRACE_VH(android_rvh_preempt_disable, preempt_disable_end);
	return ret;
}


int oplus_long_preempt_init(struct proc_dir_entry *pde)
{
	int ret = 0;

	register_long_preempt_vendor_hooks();
	osi_preemptirq_proc_init(pde);
	return ret;
}

int oplus_long_preempt_exit(void)
{
	int ret = 0;

	REGISTER_TRACE_VH(android_rvh_irqs_disable, irq_disable_begin);
	REGISTER_TRACE_VH(android_rvh_irqs_enable, irq_disable_end);
	REGISTER_TRACE_VH(android_rvh_preempt_disable, preempt_disable_begin);
	REGISTER_TRACE_VH(android_rvh_preempt_disable, preempt_disable_end);
	return ret;
}
