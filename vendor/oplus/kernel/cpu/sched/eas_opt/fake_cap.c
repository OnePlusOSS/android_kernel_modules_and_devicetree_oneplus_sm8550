// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/uaccess.h>
#include <linux/reciprocal_div.h>
#include <linux/cpumask.h>
#include <linux/arch_topology.h>
#include <../kernel/sched/sched.h>
#include "fake_cap.h"

#define LARGE_BUFFER_SIZE 250
#define MIDDLE_BUFFER_SIZE 50
#define L_BUFFER_SIZE 10

int eas_opt_enable = 0;
EXPORT_SYMBOL(eas_opt_enable);

int eas_opt_debug_enable = 0;
EXPORT_SYMBOL(eas_opt_debug_enable);

int sa_adjust_group_enable = 0;
EXPORT_SYMBOL(sa_adjust_group_enable);

int fake_cap_multiple[FAKE_CLUSTERS] = {100, 100, 100, 100, 100};
int nr_fake_cap_multiple[FAKE_CLUSTERS] = {100, 100, 100, 100, 100};
int back_cap[FAKE_CLUSTERS] = {1024, 1024, 1024, 1024, 1024};
unsigned long real_cpu_cap[FAKE_CPUS] = {1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024};
EXPORT_SYMBOL(real_cpu_cap);
EXPORT_SYMBOL(fake_cap_multiple);
EXPORT_SYMBOL(nr_fake_cap_multiple);
EXPORT_SYMBOL(back_cap);

struct groups_adjust group_adjust = {
	.task_compensate = 0,
	.adjust_std_vtime_slice = 75,
	.group_param = {
		{
			.util_compensate = 0,
			.vtime_compensate = 200,
		},
		{
			.util_compensate = 0,
			.vtime_compensate = 150,
		},
		{
			.util_compensate = 0,
			.vtime_compensate = 0,
		},
		{
			.util_compensate = 0,
			.vtime_compensate = 50,
		},
	},
};
EXPORT_SYMBOL(group_adjust);

int util_thresh_percent[FAKE_CLUSTERS] = {100, 100, 100, 100, 100};
EXPORT_SYMBOL(util_thresh_percent);
int util_thresh_cvt[FAKE_CLUSTERS] = {1000, 1000, 1000, 1000, 1000};
EXPORT_SYMBOL(util_thresh_cvt);

int get_grp_adinfo(struct task_struct *p)
{
	struct cgroup_subsys_state *css;

	if (p == NULL)
		return AD_DF;

	rcu_read_lock();
	css = task_css(p, cpu_cgrp_id);
	if (!css) {
		rcu_read_unlock();
		return AD_DF;
	}
	rcu_read_unlock();

	switch (css->id) {
	case TOPAPP:
		return AD_TOP;
	case FGAPP:
		return AD_FG;
	case BGAPP:
		return AD_BG;
	case DEFAULTAPP:
	case NULLAPP:
		return AD_DF;
	default:
		return AD_DF;
	}
	return AD_DF;
}
EXPORT_SYMBOL(get_grp_adinfo);

static inline unsigned long cpu_util(int cpu)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	if (sched_feat(UTIL_EST))
		util = max(util, READ_ONCE(cfs_rq->avg.util_est.enqueued));

	return util;
}

static inline unsigned long task_util(struct task_struct *p)
{
	return READ_ONCE(p->se.avg.util_avg);
}

static inline unsigned long _task_util_est(struct task_struct *p)
{
	struct util_est ue = READ_ONCE(p->se.avg.util_est);

	return max(ue.ewma, (ue.enqueued & ~UTIL_AVG_UNCHANGED));
}

#define lsub_positive(_ptr, _val) do {				\
		typeof(_ptr) ptr = (_ptr);				\
		*ptr -= min_t(typeof(*ptr), *ptr, _val);		\
} while (0)

unsigned long cpu_util_without(int cpu, struct task_struct *p)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	/* Task has no contribution or is new */
	if (cpu != task_cpu(p) || !READ_ONCE(p->se.avg.last_update_time))
		return cpu_util(cpu);

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	/* Discount task's util from CPU's util */
	lsub_positive(&util, task_util(p));

	/*
	 * Covered cases:
	 *
	 * a) if *p is the only task sleeping on this CPU, then:
	 *      cpu_util (== task_util) > util_est (== 0)
	 *    and thus we return:
	 *      cpu_util_without = (cpu_util - task_util) = 0
	 *
	 * b) if other tasks are SLEEPING on this CPU, which is now exiting
	 *    IDLE, then:
	 *      cpu_util >= task_util
	 *      cpu_util > util_est (== 0)
	 *    and thus we discount *p's blocked utilization to return:
	 *      cpu_util_without = (cpu_util - task_util) >= 0
	 *
	 * c) if other tasks are RUNNABLE on that CPU and
	 *      util_est > cpu_util
	 *    then we use util_est since it returns a more restrictive
	 *    estimation of the spare capacity on that CPU, by just
	 *    considering the expected utilization of tasks already
	 *    runnable on that CPU.
	 *
	 * Cases a) and b) are covered by the above code, while case c) is
	 * covered by the following code when estimated utilization is
	 * enabled.
	 */
	if (sched_feat(UTIL_EST)) {
		unsigned int estimated =
			READ_ONCE(cfs_rq->avg.util_est.enqueued);

		/*
		 * Despite the following checks we still have a small window
		 * for a possible race, when an execl's select_task_rq_fair()
		 * races with LB's detach_task():
		 *
		 *   detach_task()
		 *     p->on_rq = TASK_ON_RQ_MIGRATING;
		 *     ---------------------------------- A
		 *     deactivate_task()                   \
		 *       dequeue_task()                     + RaceTime
		 *         util_est_dequeue()              /
		 *     ---------------------------------- B
		 *
		 * The additional check on "current == p" it's required to
		 * properly fix the execl regression and it helps in further
		 * reducing the chances for the above race.
		 */
		if (unlikely(task_on_rq_queued(p) || current == p))
			lsub_positive(&estimated, _task_util_est(p));

		util = max(util, estimated);
	}

	/*
	 * Utilization (estimated) can exceed the CPU capacity, thus let's
	 * clamp to the maximum CPU capacity to ensure consistency with
	 * the cpu_util call.
	 */
	return util;
}

static inline bool should_eas_task_skip_cpu(int cpu)
{
	return arch_scale_cpu_capacity(cpu) < EAS_GOLD_PLUS_CAPACITY;
}

bool adjust_group_task(struct task_struct *p, int cpu)
{
	int id = 0;
	unsigned long cpu_util_no_p;
	int margin;
	unsigned long util_p;

	if (!eas_opt_enable || !sa_adjust_group_enable || !should_eas_task_skip_cpu(cpu))
		return false;

	cpu_util_no_p = cpu_util_without(cpu, p);

	id = get_grp_adinfo(p);
	switch (id) {
	case AD_TOP:
		margin = group_adjust.group_param[AD_TOP].util_compensate;
		break;
	case AD_FG:
		margin = group_adjust.group_param[AD_FG].util_compensate;
		break;
	case AD_BG:
		margin = group_adjust.group_param[AD_BG].util_compensate;
		break;
	case AD_DF:
	default:
		margin = group_adjust.group_param[AD_DF].util_compensate;
	}

	util_p = max(task_util(p), _task_util_est(p));

	if ((group_adjust.task_compensate + util_p > real_cpu_cap[cpu]) || (margin + (int)util_p + (int)cpu_util_no_p > (int)real_cpu_cap[cpu])) {
		if (unlikely(eas_opt_debug_enable))
			trace_printk("[eas_opt]:id:%d, comm:%s, util_p:%d, margin:%d, cpu_util_no_p:%d, origin_cpu:%d, origin_cpu_cap:%d\n",
					id, p->comm, util_p, margin, cpu_util_no_p, cpu, real_cpu_cap[cpu]);
		return true;
	}

	return false;
}
EXPORT_SYMBOL(adjust_group_task);

#define PARAM_NUM 2
static ssize_t proc_eas_opt_enabled_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[50];
	char *str, *token;
	char opt_str[PARAM_NUM][L_BUFFER_SIZE] = {"0", "0"};
	int cnt = 0;
	int i = 0;
	int val = 0;
	int err;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);
	while ((token = strsep(&str, " ")) && *token && (cnt < 2)) {
		strlcpy(opt_str[cnt], token, sizeof(opt_str[cnt]));
		cnt += 1;
	}

	for (i = 0; i < PARAM_NUM && i < cnt; i++) {
		err = kstrtoint(strstrip(opt_str[i]), 10, &val);
		if (err)
			return err;

		if (i == 0)
			eas_opt_enable = val;

		if (i == 1)
			eas_opt_debug_enable = val;
	}

	return count;
}

static ssize_t proc_eas_opt_enabled_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MIDDLE_BUFFER_SIZE];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "eas_opt_enable=%d eas_opt_debug_enable=%d\n", eas_opt_enable, eas_opt_debug_enable);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_eas_opt_enabled_fops = {
	.proc_write		= proc_eas_opt_enabled_write,
	.proc_read		= proc_eas_opt_enabled_read,
};

static ssize_t proc_group_adjust_enabled_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[L_BUFFER_SIZE];
	int err, val;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	err = kstrtoint(strstrip(buffer), 10, &val);
	if (err)
		return err;

	sa_adjust_group_enable = val;

	return count;
}

static ssize_t proc_group_adjust_enabled_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MIDDLE_BUFFER_SIZE];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "sa_adjust_group_enable=%d\n", sa_adjust_group_enable);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_group_adjust_enabled_fops = {
	.proc_write		= proc_group_adjust_enabled_write,
	.proc_read		= proc_group_adjust_enabled_read,
};

static ssize_t proc_util_thresh_percent_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MIDDLE_BUFFER_SIZE];
	char *str, *token;
	char opt_str[FAKE_CLUSTERS][L_BUFFER_SIZE] = {"100", "100", "100", "100", "100"};
	int cnt = 0;
	int i = 0;
	int tmp = 100;
	int err = 0;
	int reset_util_thresh_percent[FAKE_CLUSTERS] = {100, 100, 100, 100, 100};
	int reset_util_thresh_cvt[FAKE_CLUSTERS] = {1000, 1000, 1000, 1000, 1000};

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);
	while ((token = strsep(&str, " ")) && *token && (cnt < FAKE_CLUSTERS)) {
		strlcpy(opt_str[cnt], token, sizeof(opt_str[cnt]));
		cnt += 1;
	}

	for (i = 0; i < FAKE_CLUSTERS && i < cnt; i++) {
		err = kstrtoint(strstrip(opt_str[i]), 10, &tmp);
		if (err)
			return err;
		if (tmp == -1) {
			memcpy(util_thresh_percent, reset_util_thresh_percent, sizeof(reset_util_thresh_percent));
			memcpy(util_thresh_cvt, reset_util_thresh_cvt, sizeof(reset_util_thresh_cvt));
			i = i + 1;
			goto util_thresh_finished;
		}
		util_thresh_percent[i] = tmp;
		util_thresh_cvt[i] = tmp*10;
	}

util_thresh_finished:
	if ((i == FAKE_CLUSTERS) || (i == cnt))
		return count;

	eas_opt_enable = 0;
	eas_opt_debug_enable = 0;
	sa_adjust_group_enable = 0;
	pr_err("[eas_opt]: set util_thresh_percent failed, close eas_opt feature\n");
	return -EFAULT;
}

static ssize_t proc_util_thresh_percent_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[LARGE_BUFFER_SIZE];
	size_t len = 0;
	int i;

	buffer[0] = '\0';
	for (i = 0; i < FAKE_CLUSTERS; i++) {
		len = snprintf(buffer, sizeof(buffer), "%s%d:util_thresh_percent=%d\n",
					buffer, i, util_thresh_percent[i]);
	}

	if (len > LARGE_BUFFER_SIZE) {
		len = LARGE_BUFFER_SIZE;
		buffer[LARGE_BUFFER_SIZE-1] = '\0';
	}

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_util_thresh_percent_fops = {
	.proc_write		= proc_util_thresh_percent_write,
	.proc_read		= proc_util_thresh_percent_read,
};

#ifdef CONFIG_HAVE_SCHED_AVG_IRQ
static inline unsigned long walt_cpu_util_irq(struct rq *rq)
{
	return rq->avg_irq.util_avg;
}

static inline
unsigned long walt_scale_irq_capacity(unsigned long util, unsigned long irq, unsigned long max)
{
	util *= (max - irq);
	util /= max;

	return util;
}
#else
static inline unsigned long walt_cpu_util_irq(struct rq *rq)
{
	return 0;
}
static inline
unsigned long walt_scale_irq_capacity(unsigned long util, unsigned long irq, unsigned long max)
{
	return util;
}
#endif

#ifdef CONFIG_SCHED_THERMAL_PRESSURE
static inline u64 walt_thermal_load_avg(struct rq *rq)
{
		return READ_ONCE(rq->avg_thermal.load_avg);
}
#else
static inline u64 walt_thermal_load_avg(struct rq *rq)
{
		return 0;
}
#endif

static unsigned long walt_scale_rt_capacity(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long max = arch_scale_cpu_capacity(cpu);
	unsigned long used, free;
	unsigned long irq;

	irq = walt_cpu_util_irq(rq);

	if (unlikely(irq >= max))
		return 1;

	/*
	 * avg_rt.util_avg and avg_dl.util_avg track binary signals
	 * (running and not running) with weights 0 and 1024 respectively.
	 * avg_thermal.load_avg tracks thermal pressure and the weighted
	 * average uses the actual delta max capacity(load).
	 */
	used = READ_ONCE(rq->avg_rt.util_avg);
	used += READ_ONCE(rq->avg_dl.util_avg);
	used += walt_thermal_load_avg(rq);

	if (unlikely(used >= max))
		return 1;

	free = max - used;

	return walt_scale_irq_capacity(free, irq, max);
}

noinline int fake_tracing_mark_write(const char *buf)
{
	trace_printk(buf);
	return 0;
}

void fake_cap_systrace_c(unsigned int cpu, unsigned long cap_orig, unsigned long cap)
{
	char buf_orig[MIDDLE_BUFFER_SIZE];
	char buf[MIDDLE_BUFFER_SIZE];

	snprintf(buf_orig, sizeof(buf_orig), "C|10001|Cpu%d_fake_cap_orig|%d\n", cpu, cap_orig);
	fake_tracing_mark_write(buf_orig);
	snprintf(buf, sizeof(buf), "C|10001|Cpu%d_real_cap_orig|%d\n", cpu, cap);
	fake_tracing_mark_write(buf);
}
EXPORT_SYMBOL(fake_cap_systrace_c);

static int set_cluster_fake_cap(int cluster_idx, int fake_cap_multiple)
{
	unsigned long thermal_pressure;
	unsigned long fmax_capacity;
	unsigned long thermal_cap;
	struct rq *rq = NULL;
	unsigned long diff_capacity;
	unsigned int cpu;
	struct cpu_topology *cpu_topo;
	struct cpumask cpus;

	for_each_possible_cpu(cpu) {
		cpu_topo = &cpu_topology[cpu];
		if (cpu_topo->package_id == cluster_idx)
			cpumask_set_cpu(cpu, &cpus);
	}

	if (cpumask_weight(&cpus) < 1)
		return 1;

	thermal_pressure = arch_scale_thermal_pressure(cpumask_first(&cpus));
	fmax_capacity = arch_scale_cpu_capacity(cpumask_first(&cpus));
	thermal_cap = fmax_capacity - thermal_pressure;

	for_each_cpu(cpu, &cpus) {
		diff_capacity = fmax_capacity-walt_scale_rt_capacity(cpu);
		rq = cpu_rq(cpu);
		rq->cpu_capacity_orig = mult_frac(thermal_cap, fake_cap_multiple, 100);
		rq->cpu_capacity = max(rq->cpu_capacity_orig-diff_capacity, 1UL);
		if (unlikely(eas_opt_debug_enable))
			fake_cap_systrace_c(cpu, rq->cpu_capacity_orig, real_cpu_cap[cpu]);
	}
	return 0;
}

static ssize_t proc_fake_cap_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[MIDDLE_BUFFER_SIZE];
	char *str, *token;
	char opt_str[FAKE_CLUSTERS][L_BUFFER_SIZE] = {"100", "100", "100", "100", "100"};
	int cnt = 0;
	int i = 0, j = 0;
	int tmp = 100;
	int err = 0;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);
	while ((token = strsep(&str, " ")) && *token && (cnt < FAKE_CLUSTERS)) {
		strlcpy(opt_str[cnt], token, sizeof(opt_str[cnt]));
		cnt += 1;
	}

	for (i = 0; i < FAKE_CLUSTERS && i < cnt; i++) {
		err = kstrtoint(strstrip(opt_str[i]), 10, &tmp);
		if (err)
			return err;

		if (tmp == -1) {
			for (j = 0; j < FAKE_CLUSTERS; j++) {
				if (fake_cap_multiple[j] == 100)
					continue;
				if (j == FAKE_CLUSTERS-1) {
					fake_cap_multiple[j] = 100;
					continue;
				}
				if (!set_cluster_fake_cap(j, 100)) {
					fake_cap_multiple[j] = 100;
				} else {
					eas_opt_enable = 0;
					eas_opt_debug_enable = 0;
					sa_adjust_group_enable = 0;
					pr_err("[eas_opt]: reset cpu_%d failed, close eas_opt feature\n", j);
				}
			}
			i = i + 1;
			goto fake_cap_multiple_finished;
		}

		if (!set_cluster_fake_cap(i, tmp))
			fake_cap_multiple[i] = tmp;

		if (i == FAKE_CLUSTERS-1)
			fake_cap_multiple[i] = tmp;
	}

fake_cap_multiple_finished:
	for (j = 0; j < FAKE_CLUSTERS; j++) {
		nr_fake_cap_multiple[j] = mult_frac(fake_cap_multiple[FAKE_CLUSTERS-1], 1024, fake_cap_multiple[j]);
		back_cap[j] = mult_frac(1024, 100, fake_cap_multiple[j]);
	}

	if ((i == 5) || (i == cnt))
		return count;

	eas_opt_enable = 0;
	eas_opt_debug_enable = 0;
	sa_adjust_group_enable = 0;
	pr_err("[eas_opt]: set fake_cpu_mulitple failed, close eas_opt feature\n");

	return -EFAULT;
}

static ssize_t proc_fake_cap_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[LARGE_BUFFER_SIZE];
	size_t len = 0;
	int i;

	buffer[0] = '\0';
	for (i = 0; i < FAKE_CLUSTERS; i++) {
		len = snprintf(buffer, sizeof(buffer), "%scluster%d:%d\n",
					buffer, i, fake_cap_multiple[i]);
	}
	for (i = 0; i < FAKE_CPUS; i++) {
		len = snprintf(buffer, sizeof(buffer), "%scpu%d:%d\n",
					buffer, i, real_cpu_cap[i]);
	}

	if (len > LARGE_BUFFER_SIZE) {
		len = LARGE_BUFFER_SIZE;
		buffer[LARGE_BUFFER_SIZE-1] = '\0';
	}

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_fake_cap_fops = {
	.proc_write		= proc_fake_cap_write,
	.proc_read		= proc_fake_cap_read,
};

static void set_group_adjust_param(struct groups_adjust *groups, int index, int param)
{
	switch (index) {
	case AD_TASK_UTIL_INDEX:
		groups->task_compensate = param;
		break;
	case AD_VTIME_SLICE_INDEX:
		groups->adjust_std_vtime_slice = (u64)param;
		break;
	default:
		if ((index)&1) {
			groups->group_param[(index)>>1].util_compensate = param;
		} else {
			groups->group_param[(index)>>1].vtime_compensate = (unsigned long)param;
		}
		break;
	}
}

static ssize_t proc_group_adjust_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[2*MIDDLE_BUFFER_SIZE];
	char *str, *token;
	char opt_str[AD_MAX_GROUP*2+2][L_BUFFER_SIZE] = {"100", "0", "100", "0", "100", "0",  "100", "0", "0", "750000"};
	int cnt = 0;
	int i = 0;
	unsigned int tmp = 100;
	int err = 0;
	struct groups_adjust reset_group_adjust = {
		.task_compensate = 0,
		.adjust_std_vtime_slice = 75,
		.group_param = {
			{
				.util_compensate = 0,
				.vtime_compensate = 200,
			},
			{
				.util_compensate = 0,
				.vtime_compensate = 150,
			},
			{
				.util_compensate = 0,
				.vtime_compensate = 0,
			},
			{
				.util_compensate = 0,
				.vtime_compensate = 50,
			},
		},
	};

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';
	str = strstrip(buffer);
	while ((token = strsep(&str, " ")) && *token && (cnt < (AD_MAX_GROUP*2+2))) {
		strlcpy(opt_str[cnt], token, sizeof(opt_str[cnt]));
		cnt += 1;
	}

	for (i = 0; i < (AD_MAX_GROUP*2+2) && i < cnt; i++) {
		err = kstrtoint(strstrip(opt_str[i]), 10, &tmp);
		if (err)
			return err;

		if (tmp == -1) {
			memcpy(&group_adjust, &reset_group_adjust, sizeof(reset_group_adjust));
			i = i + 1;
			goto group_adjust_finished;
		}
		set_group_adjust_param(&group_adjust, i, tmp);
	}

group_adjust_finished:
	if (i == cnt)
		return count;

	return -EFAULT;
}

static ssize_t proc_group_adjust_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[LARGE_BUFFER_SIZE];
	size_t len = 0;
	char groups[AD_MAX_GROUP][L_BUFFER_SIZE] = {"top", "fg", "bg", "def"};
	int i;


	buffer[0] = '\0';
	for (i = 0; i < AD_MAX_GROUP; i++) {
		len = snprintf(buffer, sizeof(buffer), "%sgroup_adjust_%s:%d %d\n",
					buffer, groups[i], group_adjust.group_param[i].vtime_compensate, group_adjust.group_param[i].util_compensate);
	}
	len = snprintf(buffer, sizeof(buffer), "%stask_util_thresh:%d\n", buffer, group_adjust.task_compensate);
	len = snprintf(buffer, sizeof(buffer), "%sadjust_std_vtime_slice:%d\n", buffer, group_adjust.adjust_std_vtime_slice);

	if (len > LARGE_BUFFER_SIZE) {
		len = LARGE_BUFFER_SIZE;
		buffer[LARGE_BUFFER_SIZE-1] = '\0';
	}

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_group_adjust_fops = {
	.proc_write		= proc_group_adjust_write,
	.proc_read		= proc_group_adjust_read,
};

struct proc_dir_entry *fake_cap_dir_parent = NULL;
int fake_cap_init(struct proc_dir_entry *dir)
{
	struct proc_dir_entry *proc_node;
	int cpu;

	fake_cap_dir_parent = dir;

	proc_node = proc_create("group_adjust_enable", 0666, fake_cap_dir_parent, &proc_group_adjust_enabled_fops);
	proc_node = proc_create("eas_opt_enable", 0666, fake_cap_dir_parent, &proc_eas_opt_enabled_fops);
	proc_node = proc_create("fake_cap_multiple", 0666, fake_cap_dir_parent, &proc_fake_cap_fops);
	proc_node = proc_create("group_adjust", 0666, fake_cap_dir_parent, &proc_group_adjust_fops);
	proc_node = proc_create("util_thresh_percent", 0666, fake_cap_dir_parent, &proc_util_thresh_percent_fops);

	for_each_possible_cpu(cpu)
		real_cpu_cap[cpu] = cpu_rq(cpu)->cpu_capacity_orig;

	return 0;
}

void fake_cap_proc_remove(struct proc_dir_entry *dir)
{
	remove_proc_entry("group_adjust_enable", fake_cap_dir_parent);
	remove_proc_entry("eas_opt_enable", fake_cap_dir_parent);
	remove_proc_entry("fake_cap_multiple", fake_cap_dir_parent);
	remove_proc_entry("group_adjust", fake_cap_dir_parent);
	remove_proc_entry("util_thresh_percent", fake_cap_dir_parent);
}
