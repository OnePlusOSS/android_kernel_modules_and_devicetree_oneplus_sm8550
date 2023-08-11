// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#include <linux/kernel_stat.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/energy_model.h>
#include <linux/sched/task.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/time64.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/cpumask.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/reciprocal_div.h>
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
#include <linux/sched/walt.h>
#endif
#include <../../fs/proc/internal.h>
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#include <../kernel/oplus_cpu/thermal/horae_shell_temp.h>
#include <../kernel/sched/sched.h>
#include <linux/cred.h>
#include "task_overload.h"

#define MAX_PID	(32768)
#define CLUSTER_NUM (3)

/* the max size of abnormal task in every collection period for midas*/
#define MAX_SIZE (1024)


/* UP_THRESH: the upper time percent to setting affinity
 *            for power abnormal task
 * DOWN_THRESH : the floor time percent to releasing affinity
 *               for power abnormal task
 */
#define TRIGGER_FREQ	(1300000)
#define TRIGGER_UTIL	(80)
#define CPU_THRESHOLD	(85)
#define CPU_CAP	(100)
#define SA_CGROUP_BACKGROUND (3)

#define ABNORMAL_MIN_CHECK (100)

#define SYSTEM_APP_UID (1000)
#define ROOT_APP_UID (0)
#define CPU_MAX 8
#define MAX_CLUSTERS 3

#define SET_UCLAMP 500
#define RESUME_UCLAMP 1024

#define UCLAMP_BUCKET_DELTA DIV_ROUND_CLOSEST(SCHED_CAPACITY_SCALE, UCLAMP_BUCKETS)
static DEFINE_SPINLOCK(tol_lock);
/* add for midas collection */
static int atd_count;

static int golden_cpu;
static int golden_cpu_first;
static int goplus_cpu;

static int max_cluster_id;
static int min_cluster_id;
static struct abnormal_tsk_info task_info[MAX_SIZE];

static struct proc_dir_entry *parent;

unsigned int walt_scale_demand_divisor;

int sysctl_abnormal_enable;
EXPORT_SYMBOL(sysctl_abnormal_enable);

#define scale_demand(d) ((d)/walt_scale_demand_divisor)

#define IS_ROOT_UID(uid) (uid == 0)
#define IS_SYSTEM_UID(uid) (uid == 1000)

static inline bool is_max_cluster_cpu(int cpu)
{
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	int cluster_id;

	cluster_id = topology_physical_package_id(cpu);
	if (cluster_id == max_cluster_id)
#else
	if (arch_scale_cpu_capacity(cpu) == max_cluster_id)
#endif
		return true;
	return false;
}

static inline bool is_min_cluster_cpu(int cpu)
{
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	int cluster_id;

	cluster_id = topology_physical_package_id(cpu);
	if (cluster_id == min_cluster_id)
#else
	if (arch_scale_cpu_capacity(cpu) == min_cluster_id)
#endif
		return true;
	return false;
}

static inline unsigned long task_util(struct task_struct *p)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	struct walt_task_struct *wts = (struct walt_task_struct *) p->android_vendor_data1;

	return wts->demand_scaled;
#else
	return READ_ONCE(p->se.avg.util_avg);
#endif
}

static inline unsigned long cpu_util(int cpu)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	if (sched_feat(UTIL_EST))
		util = max(util, READ_ONCE(cfs_rq->avg.util_est.enqueued));

	return min_t(unsigned long, util, capacity_orig_of(cpu));
}

unsigned int uclamp_bucket_id(unsigned int clamp_value)
{
	return min_t(unsigned int, clamp_value / UCLAMP_BUCKET_DELTA, UCLAMP_BUCKETS - 1);
}

void set_uclamp_max(struct task_struct *task)
{
	task->uclamp_req[UCLAMP_MAX].value = SET_UCLAMP;
	task->uclamp_req[UCLAMP_MAX].bucket_id = uclamp_bucket_id(SET_UCLAMP);
}

void resume_uclamp_max(struct task_struct *task)
{
	task->uclamp_req[UCLAMP_MAX].value = RESUME_UCLAMP;
	task->uclamp_req[UCLAMP_MAX].bucket_id = uclamp_bucket_id(RESUME_UCLAMP);
}

int get_task_cgroup_id(struct task_struct *task)
{
	struct cgroup_subsys_state *css = task_css(task, cpu_cgrp_id);
	return css ? css->id : -1;
}

bool test_task_bg(struct task_struct *task)
{
	return (get_task_cgroup_id(task) == SA_CGROUP_BACKGROUND) ? 1 : 0;
}

void tol_init_cpus(void)
{
	int cpu;

	for (cpu = 0; cpu < CPU_MAX; cpu++) {
		if (!is_max_cluster_cpu(cpu) && !is_min_cluster_cpu(cpu)) {
			golden_cpu++;
			if (golden_cpu_first > cpu || golden_cpu_first == 0)
				golden_cpu_first = cpu;
		}
		if (is_max_cluster_cpu(cpu))
			goplus_cpu = cpu;
	}
}

void walt_update_cluster_id(int min_cluster, int max_cluster)
{
	min_cluster_id = min_cluster;
	max_cluster_id = max_cluster;
	tol_init_cpus();
}
EXPORT_SYMBOL(walt_update_cluster_id);

bool is_task_overload(struct task_struct *task)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return false;
	if (ots->abnormal_flag <= ABNORMAL_THRESHOLD)
		return false;
	return true;
}
EXPORT_SYMBOL(is_task_overload);

bool test_task_overload(struct task_struct *task)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return false;

	if (ots->abnormal_flag % ABNORMAL_TIME == 0) {
		if (check_abnormal_task_util(task) && check_abnormal_freq(task) && check_abnormal_cpu_util()
			&& test_task_uid(task) && ots->abnormal_flag <= ABNORMAL_THRESHOLD) {
			ots->abnormal_flag++;
		}
	} else if (ots->abnormal_flag <= ABNORMAL_THRESHOLD)
		ots->abnormal_flag++;
	if (ots->abnormal_flag == ABNORMAL_THRESHOLD) {
		set_task_state(task);
		if (sysctl_abnormal_enable)
			set_uclamp_max(task);
		ots->abnormal_flag++;
	}

	if (ots->abnormal_flag > ABNORMAL_THRESHOLD && test_task_bg(task)) {
		ots->abnormal_flag -= ABNORMAL_MIN_CHECK;
		return true;
	}
	return false;
}
EXPORT_SYMBOL(test_task_overload);

bool check_skip_task_goplus(struct task_struct *task, unsigned int prev_cpu, unsigned int new_cpu)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots))
		return false;

	if ((capacity_orig_of(new_cpu) < capacity_orig_of(prev_cpu)) && (ots->abnormal_flag > ABNORMAL_THRESHOLD) && is_max_cluster_cpu(prev_cpu))
		return true;
	return false;
}
EXPORT_SYMBOL(check_skip_task_goplus);

bool test_task_uid(struct task_struct *task)
{
	int cur_uid = 0;
	const struct cred *tcred;

	rcu_read_lock();
	tcred = __task_cred(task);
	cur_uid = __kuid_val(tcred->uid);
	rcu_read_unlock();
	if (IS_ROOT_UID(cur_uid) || IS_SYSTEM_UID(cur_uid))
		return false;
	return true;
}
EXPORT_SYMBOL(test_task_uid);

bool check_abnormal_freq(struct task_struct *p)
{
	unsigned int freq = 0;
	unsigned int freq_max = 0;
	struct oplus_task_struct *ots = get_oplus_task_struct(p);

	if (IS_ERR_OR_NULL(ots))
		return false;
	freq = cpufreq_quick_get(goplus_cpu);
	freq_max = cpufreq_quick_get_max(goplus_cpu);

	if (freq > TRIGGER_FREQ && freq == freq_max) {
		return true;
	} else {
		if (ots->abnormal_flag <= ABNORMAL_THRESHOLD && ots->abnormal_flag > ABNORMAL_MIN_CHECK)
			ots->abnormal_flag -= ABNORMAL_TIME;
	}

	return false;
}
EXPORT_SYMBOL(check_abnormal_freq);

bool check_abnormal_task_util(struct task_struct *p)
{
	int cpu;
	unsigned long thresh_load;
	struct reciprocal_value spc_rdiv = reciprocal_value(100);
	struct oplus_task_struct *ots = get_oplus_task_struct(p);

	if (IS_ERR_OR_NULL(ots))
		return false;
	if (!p)
		return false;

	if (test_task_bg(p) && ots->abnormal_flag >= ABNORMAL_MIN_CHECK) {
		ots->abnormal_flag -= ABNORMAL_TIME;
		if (ots->abnormal_flag <= ABNORMAL_MIN_CHECK) {
			if (p->uclamp_req[UCLAMP_MAX].value == SET_UCLAMP)
				resume_uclamp_max(p);
		}
		return false;
	}
	cpu = task_cpu(p);

	if (is_max_cluster_cpu(cpu)) {
		thresh_load = capacity_orig_of(cpu) * TRIGGER_UTIL;
		if (task_util(p) >  reciprocal_divide(thresh_load, spc_rdiv))
			return true;
	}
	if (ots->abnormal_flag <= ABNORMAL_THRESHOLD && ots->abnormal_flag > ABNORMAL_MIN_CHECK)
		ots->abnormal_flag -= ABNORMAL_TIME;
	return false;
}
EXPORT_SYMBOL(check_abnormal_task_util);

bool check_abnormal_cpu_util(void)
{
	int i;
	unsigned long goden_sum = 0;
	unsigned long goden_cap = 0;

	for (i = 0; i < golden_cpu; i++) {
		goden_sum += cpu_util(golden_cpu_first + i);
		goden_cap += capacity_orig_of(golden_cpu_first + i) / 2;
	}

	if ((capacity_orig_of(goplus_cpu) * CPU_THRESHOLD) < (cpu_util(goplus_cpu) * CPU_CAP) && goden_cap > goden_sum)
		return true;
	else
		return false;
}
EXPORT_SYMBOL(check_abnormal_cpu_util);

void set_task_state(struct task_struct *p)
{
	struct abnormal_tsk_info *tsk;
	struct timespec64 boot_time;
	unsigned long flags;
	const struct cred *tcred;

	if (p->pid >= MAX_PID || atd_count >= MAX_SIZE)
		return;
	spin_lock_irqsave(&tol_lock, flags);
	tsk = task_info + atd_count;
	tsk->pid = p->pid;
	rcu_read_lock();
	tcred = __task_cred(p);
	tsk->uid = __kuid_val(tcred->uid);
	rcu_read_unlock();
	memcpy(tsk->comm, p->comm, TASK_COMM_LEN);
	tsk->limit_flag = sysctl_abnormal_enable;
	ktime_get_real_ts64(&boot_time);
	tsk->date = boot_time.tv_sec * MSEC_PER_SEC + boot_time.tv_nsec/NSEC_PER_MSEC;
	tsk->temp = get_current_temp();
	tsk->freq = cpufreq_quick_get(goplus_cpu);
	atd_count = atd_count + 1;
	spin_unlock_irqrestore(&tol_lock, flags);
}

static int tsk_show(struct seq_file *m, void *v)
{
	int idx;
	unsigned long flags;

	seq_printf(m, "pid\tuid\tlimit_flag\tcomm\tdate\ttemp\tfreq\n");

	spin_lock_irqsave(&tol_lock, flags);
	for (idx = 0; idx < min_t(int, atd_count, MAX_SIZE); idx++) {
		struct abnormal_tsk_info *tsk = task_info + idx;
		seq_printf(m, "%-5d\t%-5d\t%-5d\t%-16s\t%ld\t%-5d\t%-16d\n",
					tsk->pid, tsk->uid, tsk->limit_flag, tsk->comm, tsk->date, tsk->temp, tsk->freq);
	}
	spin_unlock_irqrestore(&tol_lock, flags);
	return 0;
}

static int tsk_open(struct inode *inode, struct file *file)
{
	return single_open(file, tsk_show, NULL);
}

static void tsk_clear(void)
{
	atd_count = 0;
}

static ssize_t tsk_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[PROC_NUMBUF];

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	if (!strcmp(buffer, "FINISH_READ"))
		tsk_clear();
	return count;
}
static ssize_t proc_skip_goplus_enabled_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[8];
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

	sysctl_abnormal_enable = val;

	return count;
}

static ssize_t proc_skip_goplus_enabled_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char buffer[20];
	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "debug_enabled=%d\n", sysctl_abnormal_enable);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops tsk_proc_fops = {
	.proc_open		= tsk_open,
	.proc_read		= seq_read,
	.proc_write     = tsk_write,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static const struct proc_ops proc_skip_goplus_enabled_fops = {
	.proc_write		= proc_skip_goplus_enabled_write,
	.proc_read		= proc_skip_goplus_enabled_read,
	.proc_lseek		= default_llseek,
};

static int __init proc_task_overload_init(void)
{
	struct proc_dir_entry *pentry;

	parent = proc_mkdir("task_overload", NULL);
	if (!parent)
		goto ERROR_INIT_DIR;

	pentry = proc_create("abnormal_task", 0, parent, &tsk_proc_fops);
	if (!pentry) {
		pr_err("create abnormal_task proc failed\n");
		goto ERROR_INIT_PROC;
	}

	pentry = proc_create("skip_goplus_enabled", 0666, parent, &proc_skip_goplus_enabled_fops);
	if (!pentry)
		pr_err("create skip_goplus_enabled proc failed\n");

	return 0;

ERROR_INIT_PROC:
	remove_proc_entry("task_overload", NULL);
ERROR_INIT_DIR:
	pr_err("can't create task_overload proc\n");
	return -ENOENT;
}

module_init(proc_task_overload_init);
MODULE_LICENSE("GPL v2");
