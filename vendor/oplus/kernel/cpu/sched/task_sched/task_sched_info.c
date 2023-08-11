// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Oplus. All rights reserved.
 */
#include <linux/string.h>
#include <linux/kernel_stat.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <../fs/proc/internal.h>
#include <linux/uaccess.h>
#include <linux/timekeeping.h>
#include <linux/delay.h>
#include <linux/sched/clock.h>
#include <linux/cpufreq.h>
#include <linux/processor.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#include <trace/events/sched.h>
#include <linux/workqueue.h>
#include <../kernel/sched/sched.h>
#include <linux/sched.h>
#include <trace/hooks/cpufreq.h>
#include <trace/hooks/sched.h>
#include <trace/events/power.h>
#include <trace/events/task.h>
#include "task_sched_info.h"
#include "../sched_assist/sa_common.h"
#include <asm/stacktrace.h>
#include <linux/kallsyms.h>
#include <linux/sched_clock.h>
#include <linux/topology.h>

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_group.h>
#endif

#define MAX_CPU_CLUSTER 4
#define CTP_VERSION 2
#define MAX_TID_NUM 10
#define MAX_PID_NUM 9
#define BUFFER_SIZE 5760
#define NOTIFY_SIZE 2880
#define DURATION_THRESHOLD 16777215
#define CPU_NUM 8
#define NUM_LEN 21
#define SYSTEM_SERVER_NAME "SS"
#define SURFACE_FLINGER_NAME "SF"
#define MAX_THRESHOLD_NUM 5
#define DEFAULT_THRESHOLD 0x7FFFFFFFFFFFFFFF
#define MAX_BACKTRACE_LAYER 16


#define REGISTER_TRACE_VH(vender_hook, handler) \
({ \
	rc = register_##vender_hook(handler, NULL); \
	if (rc) { \
		sched_err("CTP-3：register_"#vender_hook", ret=%d\n", rc); \
		return rc; \
		} \
})

#define REGISTER_TRACE_RVH		REGISTER_TRACE_VH

int systemserver_pid = -1;
int surfaceflinger_pid = -1;
unsigned int task_sched_info_enable;
static unsigned int arr_read;
static unsigned int arr_write;
u64 time_threshold[MAX_THRESHOLD_NUM] = {DEFAULT_THRESHOLD, DEFAULT_THRESHOLD, DEFAULT_THRESHOLD, DEFAULT_THRESHOLD, DEFAULT_THRESHOLD};
static int target_pids[MAX_PID_NUM] = {-1};
static int target_tids[MAX_TID_NUM] = {-1};
unsigned int target_pids_num;
unsigned int target_tids_num;
static u64 data_arr[BUFFER_SIZE];
static u64 datainfo[BUFFER_SIZE];
static int data_size = BUFFER_SIZE;
static unsigned int dt;
static int prev_freq[CPU_NUM] = {-1};
static int prev_freq_min[CPU_NUM] = {-1};
static int prev_freq_max[CPU_NUM] = {-1};

#define MAX_UEVENT_PARAM 4
static struct kobject *sched_kobj;
static struct kset *sched_kset;
static struct work_struct sched_detect_ws;
char *sched_detect_env[MAX_UEVENT_PARAM] = {"SCHEDACTION=uevent", NULL};
static int uevent_id;

static spinlock_t read_write_lock;
static DEFINE_SPINLOCK(limit_worker);
static DEFINE_SPINLOCK(limit_thread);
static DEFINE_SPINLOCK(read_write_lock);

bool ctp_send_message;

static u64 write_pid_time;
static u64 write_tid_time;

static int last_read_idx;

static unsigned long d_convert_info;

static int d_backtrace_enable;
static int io_backtrace_enable;
static u64 d_backtrace_threshold = DEFAULT_THRESHOLD;
static u64 io_backtrace_threshold = DEFAULT_THRESHOLD;
static int io_backtrace_layer = 1;
static int d_backtrace_layer = 1;
static int cpu_cluster_num;
struct freq_limit_notifier cpu_freq_limit[MAX_CPU_CLUSTER];

LIST_HEAD(limit_call_worker);
LIST_HEAD(limit_call_thread);


static void update_task_sched_info(struct task_struct *p, u64 delay, int type, int cpu);
void cpufreq_limit_notifier_init(void);

static void sched_action_trig(int cpu)
{
	if (sched_kobj == NULL) {
		sched_err("kobj NULL\n");
		return;
	}

	if (cpu > 0)
		return;

	sprintf(sched_detect_env[1], "SCHEDNUM=%d", uevent_id);
	sched_detect_env[MAX_UEVENT_PARAM - 2] = NULL;
	sched_detect_env[MAX_UEVENT_PARAM - 1] = NULL;
	schedule_work(&sched_detect_ws);
	uevent_id++;
}

static void sched_detect_work(struct work_struct *work)
{
	kobject_uevent_env(sched_kobj, KOBJ_CHANGE, sched_detect_env);
}

static void sched_action_init(void)
{
	int i = 0;

	sched_kobj = NULL;
	uevent_id = 0;

	sched_kset = kset_create_and_add("task_sched", NULL, kernel_kobj);
	if (!sched_kset) {
		sched_err("error creating sched_kset\n");
		return;
	}

	for (i = 1; i < MAX_UEVENT_PARAM - 2; i++) {
		sched_detect_env[i] = kzalloc(50, GFP_KERNEL);
		if (!sched_detect_env[i]) {
			sched_err("kzalloc sched uevent param failed\n");
			goto sched_action_init_free_memory;
		}
	}

	sched_kobj = kobject_create_and_add("task_sched_info", &sched_kset->kobj);
	if (sched_kobj == NULL) {
		sched_err("sched_kobj init err\n");
		goto sched_action_init_free_memory;
	}

	sched_kobj->kset = sched_kset;

	INIT_WORK(&sched_detect_ws, sched_detect_work);
	return;

sched_action_init_free_memory:
	kset_unregister(sched_kset);
	for (i--; i > 0; i--)
		kfree(sched_detect_env[i]);
	sched_err("Failed!\n");
}

static void get_target_thread_pid(struct task_struct *task)
{
	struct task_struct *grp;

	if (task_uid(task).val == 1000) {
		if (strstr(task->comm, "android.anim")) {
			systemserver_pid = task->tgid;
			return;
		}

		grp = task->group_leader;
		if (strstr(grp->comm, "surfaceflinger")) {
			surfaceflinger_pid = grp->pid;
			return;
		}
	}
}

static void update_wake_tid(struct task_struct *p, struct task_struct *cur, unsigned int type)
{
	u64 cur_tid = cur->pid;

	struct oplus_task_struct *ots = get_oplus_task_struct(p);

	if (IS_ERR_OR_NULL(ots))
		return;

	ots->wake_tid = cur_tid | (type << 16);
}

static void update_running_start_time(struct task_struct *prev, struct task_struct *next)
{
	u64 clock, delay;
	struct oplus_task_struct *ots_prev = get_oplus_task_struct(prev);
	struct oplus_task_struct *ots_next = get_oplus_task_struct(next);

	clock = cpu_clock(prev->cpu);
	if (!IS_ERR_OR_NULL(ots_prev)) {
		delay = clock - ots_prev->running_start_time;
		if (ots_prev->update_running_start_time)
			update_task_sched_info(prev, delay, 0, prev->cpu);
		ots_prev->running_start_time = 0;
		ots_prev->update_running_start_time = false;
	}

	if (!IS_ERR_OR_NULL(ots_next)) {
		ots_next->running_start_time = clock;
		ots_next->update_running_start_time = true;
	}

	return;
}

static void put_in_arr(struct task_sched_info *info, u64 *data_arr)
{
	unsigned long flags;

	spin_lock_irqsave(&read_write_lock, flags);

	data_arr[arr_write] = info->sched_info_one;
	data_arr[arr_write + 1] = info->sched_info_two;

	arr_write = (arr_write + 2) < BUFFER_SIZE ? arr_write + 2 : 0;
	dt += 2;

	if (arr_read == arr_write)
		arr_read = (arr_read + 2) < BUFFER_SIZE ? arr_read + 2 : 0;

	spin_unlock_irqrestore(&read_write_lock, flags);

	if (dt >= NOTIFY_SIZE) {
		ctp_send_message = true;
		dt -= NOTIFY_SIZE;
	}
}

static u64 is_target_pid(int tgid)
{
	unsigned int num = 0;

	while (num < target_pids_num) {
		if (target_pids[num] == tgid)
			return num;
		num++;
	}

	return MAX_PID_NUM;
}

bool is_target_tid(int tid)
{
	unsigned int num = 0;

	while (num < target_tids_num) {
		if (target_tids[num] == tid) {
			return true;
		}
		num++;
	}

	return false;
}

static void update_backtrace_info(struct task_struct *p, u64 start_time, int type, u64 delay)
{
	struct task_sched_info task_backtrace;
	int layer = 0;

	if (type == task_sched_info_D && !d_backtrace_enable)
		return;

	if (type == task_sched_info_IO && !io_backtrace_enable)
		return;

	if (type == task_sched_info_D && d_backtrace_enable && delay >= d_backtrace_threshold) {
		layer = d_backtrace_layer;
	}

	if (type == task_sched_info_IO && io_backtrace_enable && delay >= io_backtrace_threshold) {
		layer = io_backtrace_layer;
	}

	if (!layer)
		return;

	task_backtrace.sched_info_one = task_sched_info_backtrace | (p->pid << 8);
	task_backtrace.sched_info_two = (u64)get_wchan(p);

	if (!d_convert_info)
		d_convert_info = (unsigned long)task_backtrace.sched_info_two;

	put_in_arr(&task_backtrace, data_arr);
}

static void update_task_sched_info(struct task_struct *p, u64 delay, int type, int cpu)
{
	struct task_sched_info sched_info;
	u64 p_tid, wake_tid;
	u64 cur_sched_clock;
	u64 pid_num = MAX_PID_NUM;
	bool target_tid = false;
	struct oplus_task_struct *ots = get_oplus_task_struct(p);
	u64 delay_ori = delay;

	if (IS_ERR_OR_NULL(ots))
		return;

	if (!task_sched_info_enable)
		return;

	if (!p)
		return;

	if (type > task_sched_info_backtrace || type < task_sched_info_running)
		return;

	if (delay < time_threshold[type])
		return;

	sched_info.sched_info_one = 0;
	sched_info.sched_info_two = 0;
	pid_num = is_target_pid(p->tgid);
	target_tid = is_target_tid(p->pid);
	if (pid_num != MAX_PID_NUM || target_tid) {
		p_tid = p->pid;
		wake_tid = ots->wake_tid;
		cur_sched_clock = cpu_clock(cpu);
		delay = delay >> 20;
		if (delay > DURATION_THRESHOLD)
			delay = DURATION_THRESHOLD;

		switch (type) {
		case task_sched_info_running:
			sched_info.sched_info_one = type | (cpu << 5);
			sched_info.sched_info_one = sched_info.sched_info_one | (cur_sched_clock << 8);
			sched_info.sched_info_two = delay | (p_tid << 24);
			sched_info.sched_info_two =  sched_info.sched_info_two | (pid_num << 58);
			break;

		case task_sched_info_runnable:
			sched_info.sched_info_one = type | (cpu << 5);
			sched_info.sched_info_one = sched_info.sched_info_one | (cur_sched_clock << 8);
			sched_info.sched_info_two = delay | (p_tid << 24);
			sched_info.sched_info_two = sched_info.sched_info_two | (wake_tid << 40);
			sched_info.sched_info_two =  sched_info.sched_info_two | (pid_num << 58);
			break;

		case task_sched_info_D:
		case task_sched_info_IO:
			update_backtrace_info(p, cur_sched_clock, type, delay_ori);

		case task_sched_info_S:
			wake_tid = wake_tid & (~(1 << 16));
			sched_info.sched_info_one = type | (cpu << 5);
			sched_info.sched_info_one = sched_info.sched_info_one | (cur_sched_clock << 8);
			sched_info.sched_info_two = delay | (p_tid << 24);
			sched_info.sched_info_two = sched_info.sched_info_two | (wake_tid << 40);
			sched_info.sched_info_two =  sched_info.sched_info_two | (pid_num << 58);
		}

		put_in_arr(&sched_info, data_arr);
	}
}

static void update_freq_info(struct cpufreq_policy *policy)
{
	u64 cur, clock, cpu;
	struct task_sched_info freq_info;

	if (!task_sched_info_enable)
		return;

	clock = cpu_clock(policy->cpu);
	cur = (u64)policy->cur;
	cpu = policy->cpu;

	if (prev_freq[policy->cpu] == cur)
		return;

	prev_freq[policy->cpu] = cur;

	freq_info.sched_info_one = task_sched_info_freq | (cpu << 5);
	freq_info.sched_info_one = freq_info.sched_info_one | (clock << 8);
	freq_info.sched_info_two = cur;
	put_in_arr(&freq_info, data_arr);
}

u64 update_limit_call_info(bool flag_worker, int target_tid, char *comm)
{
	int cur_num = 0;
	struct freq_limit_worker *freq_worker;
	struct freq_limit_thread *freq_thread;
	unsigned long flags;

	if (flag_worker) {
		spin_lock_irqsave(&limit_worker, flags);
		list_for_each_entry(freq_worker, &limit_call_worker, worker_list_entry) {
			if (!IS_ERR_OR_NULL(freq_worker) && (freq_worker->pid == target_tid)) {
				for (cur_num = 0; cur_num < freq_worker->num; cur_num++) {
					if (!strcmp(freq_worker->worker[cur_num] , comm)) {
						spin_unlock_irqrestore(&limit_worker, flags);
						return cur_num;
					}
				}
				if (cur_num < WORKER_NAME_NUM) {
					strncpy(freq_worker->worker[cur_num], comm, COMM_LEN);
					freq_worker->num++;
					spin_unlock_irqrestore(&limit_worker, flags);
					return cur_num;
				}
				sched_err("update_limit_call_info:worker name num more than max worker name num\n");
				spin_unlock_irqrestore(&limit_worker, flags);
				return -1;
			}
		}
		spin_unlock_irqrestore(&limit_worker, flags);

		freq_worker = kvzalloc(sizeof(*freq_worker), GFP_KERNEL);
		if (IS_ERR_OR_NULL(freq_worker)) {
			sched_err("kvzalloc freq_worker failed\n");
			return 0;
		}
		freq_worker->pid = target_tid;
		freq_worker->num = 1;
		strncpy(freq_worker->worker[cur_num], comm, COMM_LEN);

		spin_lock_irqsave(&limit_worker, flags);
		list_add_tail(&freq_worker->worker_list_entry, &limit_call_worker);
		spin_unlock_irqrestore(&limit_worker, flags);

		return cur_num;
	} else {
		spin_lock_irqsave(&limit_thread, flags);
		list_for_each_entry(freq_thread, &limit_call_thread, thread_list_entry) {
			if (!IS_ERR_OR_NULL(freq_thread) && (freq_thread->pid == target_tid)) {
				spin_unlock_irqrestore(&limit_thread, flags);
				return 0;
			}
		}
		spin_unlock_irqrestore(&limit_thread, flags);

		freq_thread = kvzalloc(sizeof(*freq_thread), GFP_KERNEL);
		if (IS_ERR_OR_NULL(freq_thread)) {
			sched_err("kvzalloc freq_thread failed\n");
			return 0;
		}
		freq_thread->pid = target_tid;
		strncpy(freq_thread->thread, comm, COMM_LEN);

		spin_lock_irqsave(&limit_thread, flags);
		list_add_tail(&freq_thread->thread_list_entry, &limit_call_thread);
		spin_unlock_irqrestore(&limit_thread, flags);

		return 0;
	}
}


static void update_freq_limit_info(struct cpufreq_policy *policy)
{
	u64 min, max, clock, cur_tid, cpu;
	int cluster_id;
	unsigned long flags;
	struct task_sched_info freq_limit_info;
	u64 num = 0;
	bool is_worker;
	char target_comm[COMM_LEN];

	if (!task_sched_info_enable)
		return;

	cpu = policy->cpu;
	min = (u64)policy->min;
	max = (u64)policy->max;

	if (prev_freq_min[policy->cpu] == min && prev_freq_max[policy->cpu] == max)
		return;

	prev_freq_min[cpu] = min;
	prev_freq_max[cpu] = max;

	clock = cpu_clock(policy->cpu);

	cur_tid = 0;
	cluster_id = topology_physical_package_id(cpu);
	if (cpu_freq_limit[cluster_id].cur_type) {
		spin_lock_irqsave(&cpu_freq_limit[cluster_id].notifier_lock, flags);
		if (cpu_freq_limit[cluster_id].cur_type == req_freq_limit_min) {
			cur_tid = cpu_freq_limit[cluster_id].req_min_tid;
			is_worker = cpu_freq_limit[cluster_id].min_flag_worker;
			strncpy(target_comm,  cpu_freq_limit[cluster_id].min_comm, COMM_LEN);
		}
		else {
			cur_tid = cpu_freq_limit[cluster_id].req_max_tid;
			is_worker = cpu_freq_limit[cluster_id].max_flag_worker;
			strncpy(target_comm,  cpu_freq_limit[cluster_id].max_comm, COMM_LEN);
		}
		spin_unlock_irqrestore(&cpu_freq_limit[cluster_id].notifier_lock, flags);
		num = update_limit_call_info(is_worker, (int)cur_tid, target_comm);
	}

	if (num == -1)
		return;

	freq_limit_info.sched_info_one = task_sched_info_freq_limit | (cpu << 5);
	freq_limit_info.sched_info_one = freq_limit_info.sched_info_one | (clock << 8);
	freq_limit_info.sched_info_two = min | (max << 23);
	freq_limit_info.sched_info_two = freq_limit_info.sched_info_two | (cur_tid << 46);
	freq_limit_info.sched_info_two = freq_limit_info.sched_info_two | (num << 62);
	put_in_arr(&freq_limit_info, data_arr);
}

void update_cpu_isolate_info(int cpu, u64 type)
{
	u64 clock, cur_tid;
	struct task_sched_info isolate_info;

	if (!task_sched_info_enable)
		return;

	clock = cpu_clock(cpu);
	cur_tid = current->pid;

	isolate_info.sched_info_one = task_sched_info_isolate | (cpu << 5);
	isolate_info.sched_info_one = isolate_info.sched_info_one | (clock << 8);
	isolate_info.sched_info_two = type | (cur_tid << 8);
	put_in_arr(&isolate_info, data_arr);
}
EXPORT_SYMBOL(update_cpu_isolate_info);

void update_cpus_isolate_info(struct cpumask *cpus, u64 type)
{
	int cpu;

	if (!task_sched_info_enable)
		return;

	for_each_cpu(cpu, cpus)
		update_cpu_isolate_info(cpu, type);
}
EXPORT_SYMBOL(update_cpus_isolate_info);

static int proc_pids_set_show(struct seq_file *m, void *v)
{
	int i = 0;

	if (!task_sched_info_enable)
		return -EFAULT;

	seq_printf(m, "%llu ", write_pid_time);

	while (i < target_pids_num) {
		seq_printf(m, "%d ", target_pids[i]);
		i++;
	}
	seq_puts(m, "\n");

	return 0;
}

static int proc_pids_set_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_pids_set_show, inode);
}

static ssize_t proc_pids_set_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[NUM_LEN * MAX_PID_NUM];
	char *all, *single;
	const char *target = NULL;
	int err;
	struct timespec64 ts;

	if (!task_sched_info_enable)
		return -EFAULT;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	all = buffer;

	target_pids_num = 2;
	while ((single = strsep(&all, " ")) != NULL && target_pids_num < MAX_PID_NUM) {
		target = single;

		err = kstrtouint(target, 0, &target_pids[target_pids_num]);
		if (err) {
			target = NULL;
			continue;
		}
		target = NULL;
		target_pids_num++;
	}

	ktime_get_real_ts64(&ts);
	write_pid_time = (u64)ts.tv_sec * 1000000 + (u64)(ts.tv_nsec/1000);

	return count;
}

static const struct proc_ops proc_pids_set_operations = {
	.proc_open	=	proc_pids_set_open,
	.proc_read	=	seq_read,
	.proc_write	=	proc_pids_set_write,
	.proc_lseek	=	seq_lseek,
	.proc_release	=	single_release,
};

static int proc_tids_set_show(struct seq_file *m, void *v)
{
	int i = 0;

	if (!task_sched_info_enable)
		return -EFAULT;

	seq_printf(m, "%llu ", write_tid_time);

	while (i < target_tids_num) {
		seq_printf(m, "%d ", target_tids[i]);
		i++;
	}
	seq_printf(m, "\n");

	return 0;
}

static int proc_tids_set_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_tids_set_show, inode);
}

static ssize_t proc_tids_set_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[NUM_LEN * MAX_TID_NUM];
	char *all, *single;
	const char *target = NULL;
	int err;
	struct timespec64 ts;

	if(!task_sched_info_enable)
		return -EFAULT;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1) {
		count = sizeof(buffer) - 1;
	}

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	all = buffer;

	target_tids_num = 0;
	while ((single = strsep(&all, " ")) != NULL && target_tids_num < MAX_TID_NUM) {
		target = single;
		err = kstrtouint(target, 0, &target_tids[target_tids_num]);
		if(err) {
			target = NULL;
			continue;
		}
		target = NULL;
		target_tids_num++;
	}

	ktime_get_real_ts64(&ts);
	write_tid_time = (u64)ts.tv_sec * 1000000 + (u64)(ts.tv_nsec/1000);


	return count;
}

static const struct proc_ops proc_tids_set_operations = {
	.proc_open	=	proc_tids_set_open,
	.proc_read	=	seq_read,
	.proc_write	=	proc_tids_set_write,
	.proc_lseek	=	seq_lseek,
	.proc_release	=	single_release,
};

static void *sched_start(struct seq_file *m, loff_t *ppos)
{
	int *idx = m->private;

	*ppos = *ppos < last_read_idx ? last_read_idx : *ppos;

	*idx = *ppos;

	if (*idx >= BUFFER_SIZE)
		return NULL;

	return idx;
}

static void *sched_next(struct seq_file *m, void *v, loff_t *ppos)
{
	int *idx = (int *)v;
	int new = (*idx + 256) < data_size ? 256 : (data_size-*idx);

	*idx += new;
	*ppos += new;
	if (*idx == data_size)
		return NULL;

	return idx;
}

static int sched_show(struct seq_file *m, void *v)
{
	int *idx = (int *)v;
	int num = 0;
	int cur = *idx;

	while (num < 256 && cur < data_size) {
		seq_printf(m, "%llu %llu\n", datainfo[cur], datainfo[cur + 1]);
		num += 2;
		cur = num + *idx;
	}

	if (m->count == m->size)
		return 0;

	last_read_idx = cur;

	return 0;
}

static void sched_stop(struct seq_file *m, void *v)
{
}

static const struct seq_operations seq_ops = {
	.start	= sched_start,
	.next	= sched_next,
	.stop	= sched_stop,
	.show	= sched_show,
};

static int sched_buffer_open(struct inode *inode, struct file *file)
{
	unsigned int cur_read, cur_write;
	unsigned long flags;

	unsigned int num1 = 0;

	int *offs;

	if (!task_sched_info_enable)
		return -ENOMEM;

	memset(datainfo, 0, BUFFER_SIZE * sizeof(u64));
	spin_lock_irqsave(&read_write_lock, flags);

	data_size = (arr_write < arr_read) ? (BUFFER_SIZE + arr_write - arr_read) : arr_write - arr_read;
	cur_read = arr_read;
	cur_write = arr_write;

	arr_read = arr_write;

	dt = 0;

	if (cur_write < cur_read) {
		num1 = BUFFER_SIZE - cur_read;
		memcpy(datainfo, data_arr + cur_read, sizeof(u64) * num1);
		memcpy(datainfo + num1, data_arr, sizeof(u64) * cur_write);
	} else
		memcpy(datainfo, data_arr + cur_read, sizeof(u64) * data_size);

	spin_unlock_irqrestore(&read_write_lock, flags);

	last_read_idx = 0;

	offs = __seq_open_private(file, &seq_ops, sizeof(int));

	if (!offs)
		return -ENOMEM;

	return 0;
}

static int sched_buffer_release(struct inode *inode, struct file *file)
{
	last_read_idx = 0;

	return seq_release_private(inode, file);
}

static const struct proc_ops proc_sched_buffer_operations = {
	.proc_open	= sched_buffer_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= sched_buffer_release,
};

static int proc_task_sched_info_enable_show(struct seq_file *m, void *v)
{
	u64 cur_cpu_clock[CPU_NUM];
	struct timespec64 ts;
	unsigned int i = 0;
	u64 wall_time;

	if (!task_sched_info_enable)
		return -EFAULT;

	ktime_get_real_ts64(&ts);
	wall_time = (u64)ts.tv_sec * 1000000 + (u64)(ts.tv_nsec/1000);

	seq_printf(m, "%llu ", wall_time);

	while (i < CPU_NUM) {
		cur_cpu_clock[i] = cpu_clock(i);
		seq_printf(m, "%llu ", cur_cpu_clock[i]);
		i++;
	}
	seq_puts(m, "\n");
	return 0;
}

static int proc_task_sched_info_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_task_sched_info_enable_show, inode);
}

static ssize_t proc_task_sched_info_enable_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[PROC_NUMBUF];
	int err, enable;
	struct freq_limit_worker *worker_info;
	struct freq_limit_thread *thread_info;
	struct freq_limit_worker *tmp_worker;
	struct freq_limit_thread *tmp_thread;
	unsigned long flags;

	memset(buffer, 0, sizeof(buffer));

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtouint(strstrip(buffer), 0, &enable);
	if (err) {
		sched_err("enable/disable task_sched_info_enable fail. buffer_info: %s\n", buffer);
		return err;
	}

	if (enable) {
		target_pids[0] = systemserver_pid;
		target_pids[1] = surfaceflinger_pid;
		target_pids_num = 2;
		if (!cpu_cluster_num)
			cpufreq_limit_notifier_init();
	}

	task_sched_info_enable = enable;
	if (!task_sched_info_enable) {
		arr_read = 0;
		arr_write = 0;
		dt = 0;
		memset(data_arr, 0, BUFFER_SIZE * sizeof(u64));
		memset(target_pids, -1, MAX_PID_NUM * sizeof(int));
		memset(target_tids, -1, MAX_TID_NUM * sizeof(int));
		target_pids_num = 0;
		target_tids_num = 0;
		write_pid_time = 0;
		write_tid_time = 0;
		d_convert_info = 0;

		spin_lock_irqsave(&limit_thread, flags);
		list_for_each_entry_safe(thread_info, tmp_thread, &limit_call_thread, thread_list_entry) {
			list_del(&thread_info->thread_list_entry);
			kfree(thread_info);
		}
		spin_unlock_irqrestore(&limit_thread, flags);

		spin_lock_irqsave(&limit_worker, flags);
		list_for_each_entry_safe(worker_info, tmp_worker, &limit_call_worker, worker_list_entry) {
			list_del(&worker_info->worker_list_entry);
			kfree(worker_info);
		}
		spin_unlock_irqrestore(&limit_worker, flags);
	}

	return count;
}

static const struct proc_ops proc_task_sched_info_enable_operations = {
	.proc_open	=	proc_task_sched_info_enable_open,
	.proc_write	=	proc_task_sched_info_enable_write,
	.proc_read	=	seq_read,
	.proc_lseek	=	seq_lseek,
	.proc_release	=	single_release,
};

static int proc_sched_info_threshold_show(struct seq_file *m, void *v)
{
	if (!task_sched_info_enable)
		return -EFAULT;

	seq_printf(m, "running:%llu\trunnable:%llu\tblock/IO:%llu\tD:%llu\tS:%llu\n", time_threshold[0],
			time_threshold[1], time_threshold[2], time_threshold[3], time_threshold[4]);
	return 0;
}

static int proc_sched_info_threshold_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_sched_info_threshold_show, inode);
}

static ssize_t proc_sched_info_threshold_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[NUM_LEN * 5];
	char *all, *single;
	const char *target = NULL;
	int err, threshold_num = 0;

	memset(buffer, 0, sizeof(buffer));

	if (!task_sched_info_enable)
		return -EFAULT;

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	all = buffer;

	while ((single = strsep(&all, " ")) != NULL && threshold_num < MAX_THRESHOLD_NUM) {
		target = single;
		err = kstrtoull(target, 0, &time_threshold[threshold_num]);
		if (err) {
			sched_err("sched_info_threshold: %s\n", target);
			target = NULL;
			continue;
		}
		target = NULL;
		threshold_num++;
	}

	return count;
}

static const struct proc_ops proc_sched_info_threshold_operations = {
	.proc_open	=	proc_sched_info_threshold_open,
	.proc_write	=	proc_sched_info_threshold_write,
	.proc_read	=	seq_read,
	.proc_lseek	=	seq_lseek,
	.proc_release	=	single_release,
};

static int proc_d_convert_show(struct seq_file *m, void *v)
{
	if (!task_sched_info_enable)
		return -EFAULT;

	if (d_convert_info)
		seq_printf(m, "%pS %px\n", (void *)d_convert_info, (void *)d_convert_info);
	else
		seq_puts(m, "\n");

	return 0;
}

static int proc_d_convert_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_d_convert_show, inode);
}

static const struct proc_ops proc_d_convert_operations = {
	.proc_open	=	proc_d_convert_open,
	.proc_read	=	seq_read,
	.proc_lseek	=	seq_lseek,
	.proc_release	=	single_release,
};

static ssize_t proc_version_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[NUM_LEN];

	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d\n", CTP_VERSION);

	return  simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_version_operations = {
	.proc_read	=	proc_version_read,
	.proc_lseek     =       default_llseek,
};

static ssize_t proc_d_backtrace_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[NUM_LEN * 3];
	char *all, *single;
	const char *target = NULL;
	int err, num = 0;
	int tmp_enable = 0;
	u64 tmp_threshold = DEFAULT_THRESHOLD;
	int tmp_d_backtrace_layer = 0;

	memset(buffer, 0, sizeof(buffer));

	if (!task_sched_info_enable)
		return -EFAULT;

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	all = buffer;

	while ((single = strsep(&all, " ")) != NULL && num < 3) {
		target = single;
		if (num == 0)
			err = kstrtoint(target, 0, &tmp_enable);
		else if (num == 1)
			err = kstrtou64(target, 0, &tmp_threshold);
		else
			err = kstrtoint(target, 0, &tmp_d_backtrace_layer);

		if(err) {
			sched_err("d_backtrace: %s\n", target);
			target = NULL;
			goto out;
		}

		d_backtrace_layer = tmp_d_backtrace_layer < MAX_BACKTRACE_LAYER ? tmp_d_backtrace_layer :  MAX_BACKTRACE_LAYER;
		target = NULL;
		num++;
	}
	d_backtrace_enable = tmp_enable;
	d_backtrace_threshold = tmp_threshold;
	d_backtrace_layer = tmp_d_backtrace_layer < MAX_BACKTRACE_LAYER ? tmp_d_backtrace_layer : MAX_BACKTRACE_LAYER;

out:
	return count;
}

static ssize_t proc_d_backtrace_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[NUM_LEN * 3];

	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d %llu %d\n", d_backtrace_enable, d_backtrace_threshold, d_backtrace_layer);

	return  simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_d_backtrace_operations = {
	.proc_write	=	proc_d_backtrace_write,
	.proc_read	=	proc_d_backtrace_read,
	.proc_lseek	=	default_llseek,
};

static ssize_t proc_io_backtrace_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[NUM_LEN * 3];
	char *all, *single;
	const char *target = NULL;
	int err, num = 0;
	int tmp_enable = 0;
	u64 tmp_threshold = DEFAULT_THRESHOLD;
	int tmp_io_backtrace_layer = 0;

	memset(buffer, 0, sizeof(buffer));

	if (!task_sched_info_enable)
		return -EFAULT;

	if (count > sizeof(buffer) - 1) {
		count = sizeof(buffer) - 1;
	}

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	all = buffer;

	while ((single = strsep(&all, " ")) != NULL && num < 3) {
		target = single;
		if (num == 0)
			err = kstrtoint(target, 0, &tmp_enable);
		else if (num == 1)
			err = kstrtou64(target, 0, &tmp_threshold);
		else
			err = kstrtoint(target, 0, &tmp_io_backtrace_layer);
		if(err) {
			sched_err("io_backtrace:num:%d err:%s\n", num, target);
			target = NULL;
			goto out;
		}
		target = NULL;
		num++;
	}

	io_backtrace_enable = tmp_enable;
	io_backtrace_threshold = tmp_threshold;
	io_backtrace_layer = tmp_io_backtrace_layer < MAX_BACKTRACE_LAYER ? tmp_io_backtrace_layer : MAX_BACKTRACE_LAYER;

out:
	return count;
}

static ssize_t proc_io_backtrace_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[NUM_LEN * 3];

	size_t len = 0;

	len = snprintf(buffer, sizeof(buffer), "%d %llu %d\n", io_backtrace_enable, io_backtrace_threshold, io_backtrace_layer);

	return  simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops proc_io_backtrace_operations = {
	.proc_write	=	proc_io_backtrace_write,
	.proc_read	=	proc_io_backtrace_read,
	.proc_lseek	=	default_llseek,
};

static int proc_limit_table_show(struct seq_file *m, void *v)
{
	int i = 0;
	struct freq_limit_worker *worker_info;
	struct freq_limit_thread *thread_info;
	struct timespec64 ts;
	u64 wall_time;
	unsigned long flags;

	if (!task_sched_info_enable)
		return -EFAULT;

	ktime_get_real_ts64(&ts);
	wall_time = (u64)ts.tv_sec * 1000000 + (u64)(ts.tv_nsec/1000);

	seq_printf(m, "%llu\n", wall_time);

	spin_lock_irqsave(&limit_thread, flags);
	if (!list_empty(&limit_call_thread)) {
		list_for_each_entry(thread_info, &limit_call_thread, thread_list_entry) {
			if (!IS_ERR_OR_NULL(thread_info))
				seq_printf(m, "%d %s\n", thread_info->pid, thread_info->thread);
		}
	}
	spin_unlock_irqrestore(&limit_thread, flags);

	spin_lock_irqsave(&limit_worker, flags);
	if (!list_empty(&limit_call_worker)) {
		list_for_each_entry(worker_info, &limit_call_worker, worker_list_entry) {
			if (!IS_ERR_OR_NULL(worker_info)) {
				seq_printf(m, "%d ", worker_info->pid);
				for(i = 0; i < worker_info->num; i++)
					seq_printf(m, "%s ", worker_info->worker[i]);

				seq_puts(m, "\n");
			}
		}
	}
	spin_unlock_irqrestore(&limit_worker, flags);

	return 0;
}

static int proc_limit_table_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_limit_table_show, inode);
}

static const struct proc_ops proc_limit_table_operations = {
	.proc_open	=	proc_limit_table_open,
	.proc_read	=	seq_read,
	.proc_lseek	=	seq_lseek,
	.proc_release	=	single_release,
};


static int record_limit_tid_min(struct notifier_block *notifier, unsigned long freq, void *data) {
	struct freq_limit_notifier *target_cpu_freq_limit;

	if (!task_sched_info_enable)
		return 0;

	target_cpu_freq_limit = container_of(notifier, struct freq_limit_notifier, notifier_min);
	if (target_cpu_freq_limit) {
		target_cpu_freq_limit->req_min_tid = current->pid;
		target_cpu_freq_limit->cur_type = req_freq_limit_min;
		memset(target_cpu_freq_limit->min_comm, 0, sizeof(target_cpu_freq_limit->min_comm));
		if (current->flags & PF_WQ_WORKER) {
			wq_worker_comm(target_cpu_freq_limit->min_comm, sizeof(target_cpu_freq_limit->min_comm), current);
			target_cpu_freq_limit->min_flag_worker = true;
		}
		else {
			__get_task_comm(target_cpu_freq_limit->min_comm, sizeof(target_cpu_freq_limit->min_comm), current);
			target_cpu_freq_limit->min_flag_worker = false;
		}
	}

	return 0;
}

static int record_limit_tid_max(struct notifier_block *notifier, unsigned long freq, void *data) {
	struct freq_limit_notifier *target_cpu_freq_limit;
	if (!task_sched_info_enable)
		return 0;

	target_cpu_freq_limit = container_of(notifier, struct freq_limit_notifier, notifier_max);
	if (target_cpu_freq_limit) {
		target_cpu_freq_limit->req_max_tid = current->pid;
		target_cpu_freq_limit->cur_type = req_freq_limit_max;
		memset(target_cpu_freq_limit->max_comm, 0, sizeof(target_cpu_freq_limit->max_comm));
		if (current->flags & PF_WQ_WORKER) {
			wq_worker_comm(target_cpu_freq_limit->max_comm, sizeof(target_cpu_freq_limit->max_comm), current);
			target_cpu_freq_limit->max_flag_worker = true;
		}
		else {
			__get_task_comm(target_cpu_freq_limit->max_comm, sizeof(target_cpu_freq_limit->max_comm), current);
			target_cpu_freq_limit->max_flag_worker = false;
		}
	}

	return 0;
}


void cpufreq_limit_notifier_init(void)
{
	int ret, cpu;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy = cpufreq_cpu_get_raw(cpu);

		if (!policy) {
			sched_err("cpufreq_limit_notifier_init:get cpu policy null, cpu:%d\n", cpu);
			continue;
		}

		if (policy->cpu != cpu) {
			continue;
		}

		if (cpu_cluster_num >= MAX_CPU_CLUSTER)
			break;

		spin_lock_init(&cpu_freq_limit[cpu_cluster_num].notifier_lock);
		cpu_freq_limit[cpu_cluster_num].cpu = cpu;
		cpu_freq_limit[cpu_cluster_num].req_min_tid = -1;
		cpu_freq_limit[cpu_cluster_num].req_max_tid = -1;
		cpu_freq_limit[cpu_cluster_num].min_flag_worker = false;
		cpu_freq_limit[cpu_cluster_num].max_flag_worker = false;
		cpu_freq_limit[cpu_cluster_num].cur_type = 0;
		cpu_freq_limit[cpu_cluster_num].notifier_min.notifier_call = record_limit_tid_min;
		cpu_freq_limit[cpu_cluster_num].notifier_max.notifier_call = record_limit_tid_max;

		ret = freq_qos_add_notifier(&policy->constraints, FREQ_QOS_MIN, &cpu_freq_limit[cpu_cluster_num].notifier_min);
		if (ret) {
			sched_err("create freqlimit min notifier fail\n");
		}

		ret = freq_qos_add_notifier(&policy->constraints, FREQ_QOS_MAX, &cpu_freq_limit[cpu_cluster_num].notifier_max);
		if (ret) {
			sched_err("create freqlimit max notifier fail\n");
		}
		cpu_cluster_num++;
	}
}


#define SCHED_INFO_DIR "task_info"
#define SCHED_INFO_PROC_NODE "task_sched_info"
#define SCHED_INFO_PROC_EXIST_NODE "task_info/task_sched_info"
static struct proc_dir_entry *task_info;
static struct proc_dir_entry *sched_info;

int sched_info_init(void)
{
	struct proc_dir_entry *proc_entry;

	task_info = NULL;
	sched_info = NULL;

	task_sched_info_enable = 0;
	arr_read = 0;
	arr_write = 0;
	target_pids_num = 0;
	dt = 0;
	ctp_send_message = false;
	write_pid_time = 0;
	last_read_idx = 0;
	d_convert_info = 0;
	d_backtrace_enable = 0;
	io_backtrace_enable = 0;
	cpu_cluster_num = 0;

	sched_action_init();

	task_info = proc_mkdir(SCHED_INFO_DIR, NULL);

	if (!task_info)
		sched_info = proc_mkdir(SCHED_INFO_PROC_EXIST_NODE, NULL);
	else
		sched_info = proc_mkdir(SCHED_INFO_PROC_NODE, task_info);

	if (!sched_info) {
		sched_err("create task_sched_info fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("pids_set", 0666, sched_info, &proc_pids_set_operations);
	if (!proc_entry) {
		sched_err("create pids_set fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("tids_set", 0666, sched_info, &proc_tids_set_operations);
	if(!proc_entry) {
		sched_err("create tids_set fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("sched_buffer", 0666, sched_info, &proc_sched_buffer_operations);
	if (!proc_entry) {
		sched_err("create sched_buffer fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("task_sched_info_enable", 0666, sched_info, &proc_task_sched_info_enable_operations);
	if (!proc_entry) {
		sched_err("create task_sched_info_enable fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("sched_info_threshold", 0666, sched_info, &proc_sched_info_threshold_operations);
	if (!proc_entry) {
		sched_err("create sched_info_threshold fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("d_convert", 0666, sched_info, &proc_d_convert_operations);
	if (!proc_entry) {
		sched_err("create d_convert fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("version", 0666, sched_info, &proc_version_operations);
	if(!proc_entry) {
		sched_err("create version fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("d_backtrace", 0666, sched_info, &proc_d_backtrace_operations);
	if(!proc_entry) {
		sched_err("create d_backtrace fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("io_backtrace", 0666, sched_info, &proc_io_backtrace_operations);
	if(!proc_entry) {
		sched_err("create io_backtrace fail\n");
		goto ERROR_INIT_VERSION;
	}

	proc_entry = proc_create("limit_table", 0666, sched_info, &proc_limit_table_operations);
	if(!proc_entry) {
		sched_err("create limit_table fail\n");
		goto ERROR_INIT_VERSION;
	}

	cpufreq_limit_notifier_init();

	return 0;

ERROR_INIT_VERSION:
	remove_proc_entry(SCHED_INFO_PROC_NODE, NULL);
	return -ENOENT;
}


static void ctp_send_message_handler(void *data, struct task_struct *p, struct rq *rq, int user_tick)
{
	if (!task_sched_info_enable)
		return;

	if (ctp_send_message) {
		sched_action_trig(rq->cpu);
		ctp_send_message = false;
	}
}

static void set_task_comm_handler(void *data, struct task_struct *tsk, const char *comm)
{
	get_target_thread_pid(tsk);
}

static void sched_stat_wait_handler(void *data, struct task_struct *tsk, u64 delta)
{
	if (!task_sched_info_enable)
		return;

	update_task_sched_info(tsk, delta, task_sched_info_runnable, task_cpu(tsk));
}

static void sched_stat_sleep_handler(void *data, struct task_struct *tsk, u64 delta)
{
	if (!task_sched_info_enable)
		return;

	update_task_sched_info(tsk, delta, task_sched_info_S, task_cpu(tsk));
}

static void sched_stat_blocked_handler(void *data, struct task_struct *tsk, u64 delta)
{
	if (!task_sched_info_enable)
		return;

	if (tsk->in_iowait)
		update_task_sched_info(tsk, delta, task_sched_info_IO, task_cpu(tsk));
	else
		update_task_sched_info(tsk, delta, task_sched_info_D, task_cpu(tsk));
}

static void sched_switch_handler(void *data, bool preempt, struct task_struct *prev, struct task_struct *next)
{
	if (!task_sched_info_enable)
		return;

	update_wake_tid(prev, next, running_runnable);

	update_running_start_time(prev, next);
}

static void sched_waking_handler(void *data, struct task_struct *p)
{
	if (!task_sched_info_enable)
		return;

	update_wake_tid(p, current, block_runnable);
}

static void cpu_frequency_limits_handler(void *data, struct cpufreq_policy *policy)
{
	if (!task_sched_info_enable)
		return;

	update_freq_limit_info(policy);
}

static void cpufreq_transition_handler(void *data, struct cpufreq_policy *policy)
{
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	fbg_android_rvh_cpufreq_transition(policy);
#endif

	if (!task_sched_info_enable)
		return;

	update_freq_info(policy);
}


int register_sched_info_vendor_hooks(void)
{
	int rc = 0;

	REGISTER_TRACE_VH(trace_android_vh_account_task_time, ctp_send_message_handler);
	REGISTER_TRACE_VH(trace_task_rename, set_task_comm_handler);
	REGISTER_TRACE_VH(trace_sched_stat_wait, sched_stat_wait_handler);
	REGISTER_TRACE_VH(trace_sched_stat_sleep, sched_stat_sleep_handler);
	REGISTER_TRACE_VH(trace_sched_stat_blocked, sched_stat_blocked_handler);
	REGISTER_TRACE_VH(trace_sched_switch, sched_switch_handler);
	REGISTER_TRACE_VH(trace_sched_waking, sched_waking_handler);
	REGISTER_TRACE_VH(trace_cpu_frequency_limits, cpu_frequency_limits_handler);
	REGISTER_TRACE_RVH(trace_android_rvh_cpufreq_transition, cpufreq_transition_handler);

	return rc;
}

static int __init oplus_task_sched_init(void)
{
	int rc;

	rc = sched_info_init();
	if (rc != 0)
		return rc;

	rc = register_sched_info_vendor_hooks();
	if (rc != 0)
		return rc;

	return 0;
}

module_init(oplus_task_sched_init);
MODULE_DESCRIPTION("Oplus Task Sched Vender Hooks Driver");
MODULE_LICENSE("GPL v2");
