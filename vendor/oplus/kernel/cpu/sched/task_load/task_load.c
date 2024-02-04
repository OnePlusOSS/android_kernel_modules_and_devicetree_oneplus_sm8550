// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/hashtable.h>
#include <linux/sched/signal.h>
#include <linux/sched/cpufreq.h>
#include <linux/timekeeping.h>
#include <linux/arch_topology.h>
#include <trace/hooks/sched.h>
#include <trace/events/task.h>
#include <../fs/proc/internal.h>
#include <../kernel/sched/sched.h>
#include "task_load.h"
#include "../sched_assist/sa_common.h"


#define REGISTER_TRACE_VH(vender_hook, handler) \
({ \
		rc = register_##vender_hook(handler, NULL); \
		if (rc) { \
			task_load_err("TaskLoad:register_"#vender_hook", ret=%d\n", rc); \
			return rc; \
		} \
})

#define TARGET_PROCESS_NUM 3
static atomic64_t target_process_runtime[TARGET_PROCESS_NUM] = {ATOMIC64_INIT(0)};
static u64 target_process_pid[TARGET_PROCESS_NUM] = {0};
static int target_pid_num = -1;

static unsigned int monitor_status;
static atomic64_t all_running_time = ATOMIC64_INIT(0);
static atomic64_t realtime_base = ATOMIC64_INIT(0);
static atomic64_t normalize_all_running_time = ATOMIC64_INIT(0);

struct rt_task_info {
	pid_t pid;
	char comm[TASK_COMM_LEN];
	u64 duration;
};

struct rt_task_info rt_task_info;


#if defined(CONFIG_SCHED_WALT) && defined(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM)
static inline u64 scale_exec_time(u64 delta, struct rq *rq)
{
	struct walt_rq *wrq = (struct walt_rq *) rq->android_vendor_data1;

	return (delta * wrq->task_exec_scale) >> 10;
}
#else
static inline u64 scale_exec_time(u64 delta, struct rq *rq)
{
	return delta;
}
#endif /* defined(CONFIG_SCHED_WALT) && defined(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM) */

static void is_target_process(int tgid, struct oplus_task_struct *ots_p)
{
	int num = 0;

	for (num = 0; num < TARGET_PROCESS_NUM; num++) {
		if (tgid == target_process_pid[num]) {
			ots_p->target_process = num;
			return;
		}
	}
	ots_p->target_process = TARGET_PROCESS_NUM;
}

static int proc_target_process_show(struct seq_file *m, void *v)
{
	u64 show_target_process_runtime;
	if (!monitor_status)
		return -EFAULT;
	if (target_pid_num == -1 || target_pid_num >= TARGET_PROCESS_NUM) {
		task_load_err("get target pid fail\n");
		goto ERROR_INFO;
	}
	show_target_process_runtime = atomic64_read(&target_process_runtime[target_pid_num]);
	seq_printf(m, "%llu\n", show_target_process_runtime >> 20);

	return 0;

ERROR_INFO:
	seq_puts(m, "0\n");
	return 0;
}

static int proc_target_process_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_target_process_show, inode);
}

static ssize_t proc_target_process_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[PROC_NUMBUF];
	int err, target_process_num;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtouint(strstrip(buffer), 0, &target_process_num);
	if (err)
		return err;

	target_pid_num = target_process_num;

	return count;
}


static const struct proc_ops proc_target_process_operations = {
	.proc_open	= proc_target_process_open,
	.proc_write	= proc_target_process_write,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int proc_running_time_show(struct seq_file *m, void *v)
{
	u64 show_all_running_time;
	if (!monitor_status)
		return -EFAULT;

	show_all_running_time = atomic64_read(&all_running_time);
	seq_printf(m, "%llu\n", show_all_running_time>>20);

	return 0;
}

static int proc_running_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_running_time_show, inode);
}

static const struct proc_ops proc_running_time_operations = {
	.proc_open	= proc_running_time_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int proc_normalize_running_time_show(struct seq_file *m, void *v)
{
	u64 show_normalize_all_running_time;
	if (!monitor_status)
		return -EFAULT;

	show_normalize_all_running_time = atomic64_read(&normalize_all_running_time);
	seq_printf(m, "%llu\n", show_normalize_all_running_time>>20);

	return 0;
}

static int proc_normalize_running_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_normalize_running_time_show, inode);
}

static const struct proc_ops proc_normalize_running_time_operations = {
	.proc_open	= proc_normalize_running_time_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int proc_real_time_show(struct seq_file *m, void *v)
{
	struct timespec64 ts;
	u64 wall_time;
	u64 show_realtime_base;

	if (!monitor_status)
		return -EFAULT;

	show_realtime_base = atomic64_read(&realtime_base);
	ktime_get_real_ts64(&ts);
	wall_time = (u64)ts.tv_sec * (u64)NSEC_PER_SEC + (u64)ts.tv_nsec - show_realtime_base;
	seq_printf(m, "%llu\n", wall_time>>20);

	return 0;
}

static int proc_real_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_real_time_show, inode);
}

static const struct proc_ops proc_real_time_operations = {
	.proc_open	= proc_real_time_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int proc_normalize_real_time_show(struct seq_file *m, void *v)
{
	int i;
	struct timespec64 ts;
	u64 wall_time = 0;
	u64 normalize_wall_time = 0;
	u64 show_realtime_base;

	if (!monitor_status)
		return -EFAULT;

	show_realtime_base = atomic64_read(&realtime_base);
	ktime_get_real_ts64(&ts);
	wall_time = (u64)ts.tv_sec * (u64)NSEC_PER_SEC + (u64)ts.tv_nsec - show_realtime_base;

	for_each_possible_cpu(i) {
		normalize_wall_time += wall_time * (u64)per_cpu(cpu_scale, i);
	}

	seq_printf(m, "%llu\n", normalize_wall_time>>30);

	return 0;
}

static int proc_normalize_real_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_normalize_real_time_show, inode);
}


static const struct proc_ops proc_normalize_real_time_operations = {
	.proc_open	= proc_normalize_real_time_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};


static int proc_max_rt_runtime_show(struct seq_file *m, void *v)
{
	int im_flag = IM_FLAG_NONE;

	if (!rt_task_info.duration) {
		seq_printf(m, "rt task duration is 0\n");
		return 0;
	}
	seq_printf(m, "%s, %d, %lu\n", rt_task_info.comm, rt_task_info.pid, rt_task_info.duration);
	if (current->group_leader)
		im_flag = oplus_get_im_flag(current->group_leader);
	if (im_flag == IM_FLAG_MIDASD) {
		rt_task_info.duration = 0;
		rt_task_info.pid = 0;
		memset(rt_task_info.comm, 0, TASK_COMM_LEN);
	}
	return 0;
}


static int proc_max_rt_runtime_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_max_rt_runtime_show, inode);
}

static const struct proc_ops proc_max_rt_runtime_operations = {
	.proc_open      = proc_max_rt_runtime_open,
	.proc_read      = seq_read,
	.proc_lseek     = seq_lseek,
	.proc_release   = single_release,
};

static ssize_t proc_monitor_status_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[PROC_NUMBUF];
	int err, enable;
	struct timespec64 ts;
	int num = 0;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtouint(strstrip(buffer), 0, &enable);
	if (err)
		return err;

	ktime_get_real_ts64(&ts);
	monitor_status = enable;
	atomic64_set(&realtime_base, (u64)ts.tv_sec * (u64)NSEC_PER_SEC + (u64)ts.tv_nsec);

	if (!monitor_status) {
		for (num = 0; num < TARGET_PROCESS_NUM; num++)
			atomic64_set(&target_process_runtime[num], 0);

		atomic64_set(&all_running_time, 0);
		atomic64_set(&normalize_all_running_time, 0);
	}

	return count;
}

static ssize_t proc_monitor_status_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[PROC_NUMBUF];

	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", monitor_status);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}


static const struct proc_ops proc_monitor_status_operations = {
	.proc_write	= proc_monitor_status_write,
	.proc_read	= proc_monitor_status_read,
	.proc_lseek	= default_llseek,
};

static void task_load_info_init(void)
{
	monitor_status = 0;
	atomic64_set(&all_running_time, 0);
	atomic64_set(&realtime_base, 0);
	atomic64_set(&normalize_all_running_time, 0);
}

#define RUNTIME_DIR "task_info"
#define TASK_LOAD_NODE "task_load_info"
#define TASK_LOAD_EXIST_NODE "task_info/task_load_info"
static struct proc_dir_entry *task_load_info;

static int __init proc_task_load_init(void)
{
	struct proc_dir_entry *proc_entry, *task_runtime;

	task_load_info = NULL;

	task_load_info = proc_mkdir(RUNTIME_DIR, NULL);
	if (!task_load_info)
		task_runtime = proc_mkdir(TASK_LOAD_EXIST_NODE, NULL);
	else
		task_runtime = proc_mkdir(TASK_LOAD_NODE, task_load_info);

	if (!task_runtime) {
		task_load_err("create task_load_info fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("monitor_status", 0666, task_runtime, &proc_monitor_status_operations);
	if (!proc_entry) {
		task_load_err("create monitor_status fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("running_time", 0666, task_runtime, &proc_running_time_operations);
	if (!proc_entry) {
		task_load_err("create running_time fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("normalize_running_time", 0666, task_runtime, &proc_normalize_running_time_operations);
	if (!proc_entry) {
		task_load_err("create normalize_running_time fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("real_time", 0666, task_runtime, &proc_real_time_operations);
	if (!proc_entry) {
		task_load_err("create real_time fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("max_rt_runtime", 0666, task_runtime, &proc_max_rt_runtime_operations);
	if (!proc_entry) {
		task_load_err("create max_rt_runtime fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("normalize_real_time", 0666, task_runtime, &proc_normalize_real_time_operations);
	if (!proc_entry) {
		task_load_err("create normalize_real_time fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("target_process", 0666, task_runtime, &proc_target_process_operations);
	if (!proc_entry) {
		task_load_err("create target_process fail\n");
		goto ERROR_INIT_VERSION;
	}

	return 0;

ERROR_INIT_VERSION:
	remove_proc_entry(TASK_LOAD_NODE, NULL);
	return -ENOENT;
}

static void account_normalize_runtime_handler(void *data, struct rq *rq)
{
	u64 normalize_delta = 0;
	struct task_struct *curr = rq->curr;
	struct oplus_task_struct *ots_cur = get_oplus_task_struct(curr);
	u64 delta = 0;

	if (IS_ERR_OR_NULL(ots_cur) || !monitor_status)
		return;

	if (!ots_cur->is_update_runtime) {
		ots_cur->exec_calc_runtime = curr->se.sum_exec_runtime;
		ots_cur->is_update_runtime = 1;
	}

	if (curr->se.sum_exec_runtime > ots_cur->exec_calc_runtime) {
		delta = curr->se.sum_exec_runtime - ots_cur->exec_calc_runtime;
		normalize_delta = scale_exec_time(delta, rq);
		atomic64_add(delta, &all_running_time);
		atomic64_add(normalize_delta, &normalize_all_running_time);

		if (ots_cur->target_process == -1)
			is_target_process(curr->tgid, ots_cur);

		if (ots_cur->target_process < TARGET_PROCESS_NUM)
			atomic64_add(delta, &target_process_runtime[ots_cur->target_process]);

		ots_cur->exec_calc_runtime = curr->se.sum_exec_runtime;
	}
}

static void update_rt_runtime(struct task_struct *tsk,  u64 duration)
{
	if (!rt_task(tsk))
		return;
	if (duration > rt_task_info.duration) {
		rt_task_info.duration = duration;
		rt_task_info.pid = tsk->pid;
		memcpy(rt_task_info.comm, tsk->comm, TASK_COMM_LEN);
	}
}

static void update_runtime_handler(void *data, bool preempt, struct task_struct *prev, struct task_struct *next)
{
	u64 normalize_delta = 0;
	u64 delta = 0;
	struct oplus_task_struct *ots_prev = get_oplus_task_struct(prev);
	struct oplus_task_struct *ots_next = get_oplus_task_struct(next);

	if (IS_ERR_OR_NULL(ots_prev) || IS_ERR_OR_NULL(ots_next) || !monitor_status)
		return;

	if (!ots_prev->is_update_runtime) {
		ots_next->exec_calc_runtime = next->se.sum_exec_runtime;
		ots_next->is_update_runtime = 1;
	} else {
		if (prev->se.sum_exec_runtime > ots_prev->exec_calc_runtime) {
			delta = prev->se.sum_exec_runtime - ots_prev->exec_calc_runtime;
			update_rt_runtime(prev, delta);
			normalize_delta = scale_exec_time(delta, cpu_rq(prev->cpu));
			atomic64_add(delta, &all_running_time);
			atomic64_add(normalize_delta, &normalize_all_running_time);

			if (ots_prev->target_process == -1)
				is_target_process(prev->tgid, ots_prev);

			if (ots_prev->target_process < TARGET_PROCESS_NUM)
				atomic64_add(delta, &target_process_runtime[ots_prev->target_process]);
		}
		ots_prev->exec_calc_runtime = 0;
		ots_prev->is_update_runtime = 0;
		ots_next->exec_calc_runtime = next->se.sum_exec_runtime;
		ots_next->is_update_runtime = 1;
	}
}

static void get_target_process_handler(void *data, struct task_struct *task, const char *comm)
{
	struct task_struct *grp;
	struct oplus_task_struct *ots_task = get_oplus_task_struct(task);

	if (IS_ERR_OR_NULL(ots_task))
		return;

	if (strstr(task->comm, "om.oplus.camera")) {
		target_process_pid[CAMERA] = task->pid;
		ots_task->target_process = CAMERA;
		return;
	}

	grp = task->group_leader;

	if (!grp)
		return;

	if (strstr(grp->comm, "cameraserver")) {
		target_process_pid[CAMERASERVER] = grp->pid;
		ots_task->target_process = CAMERASERVER;
		return;
	}

#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	if (strstr(grp->comm, "vendor.qti.came")) {
		target_process_pid[CAMERAPROVIDER] = grp->pid;
		ots_task->target_process = CAMERAPROVIDER;
		return;
	}
#else
	if (strstr(grp->comm, "camerahalserver")) {
		target_process_pid[CAMERAPROVIDER] = grp->pid;
		ots_task->target_process = CAMERAPROVIDER;
		return;
	}
#endif
}


int register_task_load_vendor_hooks(void)
{
	int rc = 0;

	REGISTER_TRACE_VH(trace_sched_switch, update_runtime_handler);
	REGISTER_TRACE_VH(trace_android_vh_scheduler_tick, account_normalize_runtime_handler);
	REGISTER_TRACE_VH(trace_task_rename, get_target_process_handler);

	return rc;
}

static int __init oplus_task_load_init(void)
{
	int rc;

	task_load_info_init();

	rc = register_task_load_vendor_hooks();
	if (rc != 0)
		return rc;

	rc = proc_task_load_init();
	if (rc != 0)
		return rc;

	return rc;
}

module_init(oplus_task_load_init);
MODULE_DESCRIPTION("Oplus Task Load Vender Hooks Driver");
MODULE_LICENSE("GPL v2");
