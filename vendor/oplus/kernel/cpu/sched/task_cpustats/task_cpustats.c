// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/kernel_stat.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/energy_model.h>
#include "task_cpustats.h"
#include <../fs/proc/internal.h>
#include <../kernel/sched/sched.h>
#include <trace/hooks/sched.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/version.h>

/* FIXME get max_pid on the runtime.*/
#define MAX_PID (32768)
#define CTP_WINDOW_SZ (5)
#define CPU_NUM (8)
static struct kernel_task_cpustat ktask_cpustat[CPU_NUM];
static int cputime_one_jiffy;
static int task_cpustats_enable;

struct acct_cpustat {
	pid_t tgid;
	unsigned int pwr;
	char comm[TASK_COMM_LEN];
};

static struct acct_cpustat cpustats[MAX_PID];

static int get_power(int cpu, int freq)
{
	int i;
	struct em_perf_domain *domain = em_cpu_get(cpu);

	if (!domain)
		goto err_found;

	for (i = domain->nr_perf_states - 1; i > -1; i--) {
		struct em_perf_state *cs = domain->table + i;

		if (cs->frequency == freq)
			return cs->power;
	}
err_found:
	pr_err("not found %d %d in sge.\n", cpu, freq);
	return 0;
}

static int task_cpustats_show(struct seq_file *m, void *v)
{
	int *idx = (int *) v;

	seq_printf(m, "%d\t%d\t%d\t%s\n", *idx, cpustats[*idx].tgid, cpustats[*idx].pwr, cpustats[*idx].comm);
	return 0;
}

static void *task_cpustats_next(struct seq_file *m, void *v, loff_t *ppos)
{
	int *idx = (int *)v;

	(*idx)++;
	(*ppos)++;
	for (; *idx < MAX_PID; (*idx)++, (*ppos)++) {
		if (cpustats[*idx].pwr)
			return idx;
	}
	return NULL;
}

static void *task_cpustats_start(struct seq_file *m, loff_t *ppos)
{
	int *idx = m->private;

	if (!task_cpustats_enable)
		return NULL;

	*idx = *ppos;

	if (*idx >= MAX_PID)
		goto start_error;

	for (; *idx < MAX_PID; (*idx)++, (*ppos)++) {
		if (cpustats[*idx].pwr)
			return idx;
	}
start_error:
	return NULL;
}

static void task_cpustats_stop(struct seq_file *m, void *v)
{
}

static const struct seq_operations seq_ops = {
	.start	= task_cpustats_start,
	.next	= task_cpustats_next,
	.stop	= task_cpustats_stop,
	.show	= task_cpustats_show
};

static int sge_show(struct seq_file *m, void *v)
{
	int i, cpu;


	for_each_possible_cpu(cpu) {
		struct em_perf_domain *domain = em_cpu_get(cpu);
		struct cpufreq_policy *p = cpufreq_cpu_get_raw(cpu);
		int max_freq;
		int min_freq;

		if (!domain || !p)
			continue;

		max_freq = p->cpuinfo.max_freq;
		min_freq = p->cpuinfo.min_freq;
		seq_printf(m, "cpu %d\n", cpu);
		for (i = domain->nr_perf_states - 1; i > -1; i--) {
			struct em_perf_state *cs = domain->table + i;

			if (cs->frequency >= min_freq && cs->frequency <= max_freq)
				seq_printf(m, "freq %lu pwr %lu\n", cs->frequency, cs->power);
		}
	}

	return 0;
}

static int sge_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sge_show, NULL);
}

static int task_cpustats_open(struct inode *inode, struct file *file)
{
	int *offs;
	int  j, i = 0;
	unsigned long r_time;
	unsigned long begin = jiffies - CTP_WINDOW_SZ * HZ, end = jiffies;

	if (!task_cpustats_enable)
		return -ENOMEM;

	memset(cpustats, 0, MAX_PID * sizeof(struct acct_cpustat));

	while (i < CPU_NUM) {
		for (j = 0; j < MAX_CTP_WINDOW; j++) {
			r_time =  ktask_cpustat[i].cpustat[j].end -  ktask_cpustat[i].cpustat[j].begin;
			if (ktask_cpustat[i].cpustat[j].pid >= MAX_PID)
				continue;
			if (ktask_cpustat[i].cpustat[j].begin >= begin && ktask_cpustat[i].cpustat[j].end <= end) {
				if (cpustats[ktask_cpustat[i].cpustat[j].pid].pwr == 0) {
					memcpy(cpustats[ktask_cpustat[i].cpustat[j].pid].comm, ktask_cpustat[i].cpustat[j].comm, TASK_COMM_LEN);
					cpustats[ktask_cpustat[i].cpustat[j].pid].tgid = ktask_cpustat[i].cpustat[j].tgid;
				}
				cpustats[ktask_cpustat[i].cpustat[j].pid].pwr += get_power(i, ktask_cpustat[i].cpustat[j].freq) * jiffies_to_msecs(r_time);
			}
		}
		i++;
	}

	offs = __seq_open_private(file, &seq_ops, sizeof(int));

	if (!offs)
		return -ENOMEM;
	return 0;
}

static int task_cpustats_release(struct inode *inode, struct file *file)
{
	return seq_release_private(inode, file);
}

static ssize_t proc_task_cpustats_enable_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[PROC_NUMBUF];
	int err;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtouint(strstrip(buffer), 0, &task_cpustats_enable);
	if (err)
		return err;

	if (task_cpustats_enable)
		memset(ktask_cpustat, 0, 8 * sizeof(struct kernel_task_cpustat));

	return count;
}

static const struct proc_ops sge_proc_fops = {
	.proc_open	= sge_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static const struct proc_ops task_cpustats_proc_fops = {
	.proc_open	= task_cpustats_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= task_cpustats_release,
};

static const struct proc_ops task_cpustats_enable_proc_fops = {
	.proc_write	= proc_task_cpustats_enable_write,
	.proc_lseek	= default_llseek,
};

static int sgefreq_show(struct seq_file *m, void *v)
{
	int cpu;
	bool show_boost = false;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *p = cpufreq_cpu_get_raw(cpu);
		struct cpufreq_frequency_table *pos, *pt;

		if (!p)
			continue;

		pt = p->freq_table;
		if (!pt)
			continue;

		seq_printf(m, "cpu %d\n", cpu);

		cpufreq_for_each_valid_entry(pos, pt) {
			if (show_boost ^ (pos->flags & CPUFREQ_BOOST_FREQ))
				continue;
			seq_printf(m, "%d\n", pos->frequency);
		}
	}
	return 0;
}

static int sgefreq_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sgefreq_show, NULL);
}

static const struct proc_ops sgefreq_proc_fops = {
	.proc_open	= sgefreq_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

#define TASK_CPUSTATS_FILE "task_info"
#define TASK_CPUSTATS_PROC_NODE "task_cpustats"
#define TASK_CPUSTATS_PROC_EXIST_NODE "task_info/task_cpustats"
static struct proc_dir_entry *task_info;
static struct proc_dir_entry *task_cpustats;

static int proc_task_cpustats_init(void)
{
	struct proc_dir_entry *proc_entry;

	task_info = NULL;
	task_cpustats = NULL;

	task_info = proc_mkdir(TASK_CPUSTATS_FILE, NULL);

	if (!task_info)
		task_cpustats = proc_mkdir(TASK_CPUSTATS_PROC_EXIST_NODE, NULL);
	else
		task_cpustats = proc_mkdir(TASK_CPUSTATS_PROC_NODE, task_info);

	if (!task_cpustats) {
		task_err("create proc/task_info/task_cpustats failed\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("task_cpustats_enable", 0666, task_cpustats, &task_cpustats_enable_proc_fops);
	if (!proc_entry) {
		task_err("create task_cpustats_enable failed\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("task_cpustats", 0666, NULL, &task_cpustats_proc_fops);
	if (!proc_entry) {
		task_err("create task_cpustats failed\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("sgeinfo", 0666, task_cpustats, &sge_proc_fops);
	if (!proc_entry) {
		task_err("create sgeinfo failed\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("sgefreqinfo", 0666, task_cpustats, &sgefreq_proc_fops);
	if (!proc_entry) {
		task_err("create sgefreqinfo failed\n");
		goto ERROR_INIT_VERSION;
	}

	return 0;

ERROR_INIT_VERSION:
	remove_proc_entry(TASK_CPUSTATS_PROC_NODE, NULL);
	return -ENOENT;
}

static void account_task_time_handler(void *data, struct task_struct *p, struct rq *rq, int user_tick)
{
	int idx;
	int cpu = task_cpu(p);

	if (!task_cpustats_enable)
		return;

	if (p == rq->idle)
		return;

	if (!cputime_one_jiffy)
		cputime_one_jiffy = nsecs_to_jiffies(TICK_NSEC);

	idx = ktask_cpustat[cpu].idx % MAX_CTP_WINDOW;
	ktask_cpustat[cpu].cpustat[idx].pid = p->pid;
	ktask_cpustat[cpu].cpustat[idx].tgid = p->tgid;
	ktask_cpustat[cpu].cpustat[idx].freq = cpufreq_quick_get(cpu);
	ktask_cpustat[cpu].cpustat[idx].begin = jiffies - cputime_one_jiffy;
	ktask_cpustat[cpu].cpustat[idx].end = jiffies;
	memcpy(ktask_cpustat[cpu].cpustat[idx].comm, p->comm, TASK_COMM_LEN);
	ktask_cpustat[cpu].idx = idx + 1;
}

static int register_vendor_hooks(void)
{
	int rc = 0;

	rc = register_trace_android_vh_account_task_time(account_task_time_handler, NULL);
	if (rc != 0) {
		pr_err("CTP：register_trace_android_vh_account_task_time failed! rc=%d\n", rc);
		return rc;
	}
	return 0;
}

static int __init oplus_task_cpustats_init(void)
{
	int rc;

	task_cpustats_enable = 0;

	rc = proc_task_cpustats_init();
	if (rc != 0)
		return rc;

	rc = register_vendor_hooks();
	if (rc != 0)
		return rc;

	return 0;
}

static int unregister_vendor_hooks(void)
{
	int rc = 0;

	rc = unregister_trace_android_vh_account_task_time(account_task_time_handler, NULL);
	if (rc != 0) {
		pr_err("CTP：unregister_trace_android_vh_account_task_time failed! rc=%d\n", rc);
		return rc;
	}
	return 0;
}

static void __exit oplus_task_cpustats_exit(void)
{
	unregister_vendor_hooks();

	if (task_cpustats) {
		remove_proc_entry(TASK_CPUSTATS_PROC_NODE, NULL);
		task_cpustats = NULL;
	}

	if (task_info)
		remove_proc_entry(TASK_CPUSTATS_FILE, NULL);
}

module_init(oplus_task_cpustats_init);
module_exit(oplus_task_cpustats_exit);
MODULE_DESCRIPTION("Oplus Task Cpustats Vender Hooks Driver");
MODULE_LICENSE("GPL v2");
