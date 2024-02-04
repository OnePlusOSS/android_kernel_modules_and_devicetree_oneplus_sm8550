// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/sort.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/syscore_ops.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/cpufreq.h>
#include <linux/sched/cpufreq.h>
#include <trace/hooks/sched.h>

#include "game_ctrl.h"

struct task_wakeup_info {
	pid_t pid;
	struct task_struct *task;
	u64 wake_up_time;
} child_threads[MAX_TID_COUNT];

struct task_util_info {
	pid_t pid;
	char comm[TASK_COMM_LEN];
	u32 util;
};

static struct task_struct *game_leader = NULL;
static int child_num;

static DEFINE_RAW_SPINLOCK(g_lock);
atomic_t need_stat_util = ATOMIC_INIT(0);

static ssize_t game_pid_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char page[32] = {0};
	int ret, pid;
	struct task_struct *leader = NULL;

	ret = simple_write_to_buffer(page, sizeof(page), ppos, buf, count);
	if (ret <= 0)
		return ret;

	ret = sscanf(page, "%d", &pid);
	if (ret != 1)
		return -EINVAL;

	atomic_set(&need_stat_util, 0);

	raw_spin_lock(&g_lock);

	if (game_leader) {
		put_task_struct(game_leader);
		game_leader = NULL;
	}

	/* release */
	if (pid <= 0) {
		ret = count;
		goto unlock;
	}

	/* acquire */
	rcu_read_lock();
	leader = find_task_by_vpid(pid);
	if (!leader || leader->pid != leader->tgid) { /* must be process id */
		rcu_read_unlock();
		ret = -EINVAL;
		goto unlock;
	} else {
		game_leader = leader;
		get_task_struct(game_leader);
		rcu_read_unlock();
	}

	atomic_set(&need_stat_util, 1);

	ret = count;

unlock:
	child_num = 0;
	raw_spin_unlock(&g_lock);
	return ret;
}

static ssize_t game_pid_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char page[64] = {0};
	int pid, len;

	raw_spin_lock(&g_lock);
	if (game_leader)
		pid = game_leader->pid;
	else
		pid = -1;
	len = sprintf(page, "game_pid=%d child_num=%d\n", pid, child_num);
	raw_spin_unlock(&g_lock);

	return simple_read_from_buffer(buf, count, ppos, page, len);
}

static const struct proc_ops game_pid_proc_ops = {
	.proc_write		= game_pid_proc_write,
	.proc_read		= game_pid_proc_read,
	.proc_lseek		= default_llseek,
};

/*
 * Ascending order by util
 */
static int cmp_task_util(const void *a, const void *b)
{
	struct task_util_info *prev, *next;

	prev = (struct task_util_info *)a;
	next = (struct task_util_info *)b;
	if (unlikely(!prev || !next))
		return 0;

	if (prev->util > next->util)
		return -1;
	else if (prev->util < next->util)
		return 1;
	else
		return 0;
}

#define VALID_ACTIVE_WINDOW (100 * 1000 * 1000) /* 100ms */
static bool fill_task_util_info(struct task_wakeup_info *winfo, struct task_util_info *uinfo)
{
	struct task_struct * task = NULL;

	if (ktime_get_ns() - winfo->wake_up_time > VALID_ACTIVE_WINDOW)
		return false;

	rcu_read_lock();
	task = find_task_by_vpid(winfo->pid);
	if (task && (task == winfo->task)) {
		uinfo->pid = task->pid;
		strncpy(uinfo->comm, task->comm, TASK_COMM_LEN);
		uinfo->util = task_util(task);

		rcu_read_unlock();
		return true;
	}

	rcu_read_unlock();
	return false;
}

static int heavy_task_info_show(struct seq_file *m, void *v)
{
	char *page;
	struct task_util_info *results;
	int i, j, result_num;
	ssize_t len = 0;

	if (atomic_read(&need_stat_util) == 0)
		return -ESRCH;

	page = kzalloc(RESULT_PAGE_SIZE, GFP_KERNEL);
	if (!page)
		return -ENOMEM;
	results = kmalloc(sizeof(struct task_util_info) * MAX_TID_COUNT, GFP_KERNEL);
	if (!results) {
		kfree(page);
		return -ENOMEM;
	}

	raw_spin_lock(&g_lock);
	for (i = 0, j = 0; i < child_num; i++) {
		if (fill_task_util_info(&child_threads[i], &results[j]))
			j++;
	}

	result_num = j;
	child_num = 0;
	raw_spin_unlock(&g_lock);

	/* ascending order by util */
	sort(results, result_num, sizeof(struct task_util_info),
		&cmp_task_util, NULL);

	for (i = 0; i < result_num && i < MAX_TASK_NR; i++) {
		len += snprintf(page + len, RESULT_PAGE_SIZE - len, "%d;%s;%u\n",
			results[i].pid, results[i].comm, results[i].util);
	}

	if (len > 0)
		seq_puts(m, page);

	kfree(results);
	kfree(page);

	return 0;
}

static int heavy_task_info_proc_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, heavy_task_info_show, inode);
}

static const struct proc_ops heavy_task_info_proc_ops = {
	.proc_open		= heavy_task_info_proc_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static struct task_wakeup_info *find_child_thread(struct task_struct *task)
{
	int i;

	for (i = 0; i < child_num; i++) {
		if ((child_threads[i].task == task) && (child_threads[i].pid == task->pid))
			return &child_threads[i];
	}

	return NULL;
}

void try_to_wake_up_success_hook2(struct task_struct *task)
{
	struct task_wakeup_info *child_thread;

	if (atomic_read(&need_stat_util) == 0)
		return;

	/*
	 * only update task wake_up_time when lock is available,
	 * if not available, skip.
	 */
	if (raw_spin_trylock(&g_lock)) {
		if (!game_leader || task->tgid != game_leader->tgid)
			goto unlock;

		child_thread = find_child_thread(task);
		if (!child_thread) {
			if (child_num >= MAX_TID_COUNT)
				goto unlock;
			child_thread = &child_threads[child_num];
			child_thread->pid = task->pid;
			child_thread->task = task;
			child_thread->wake_up_time = ktime_get_ns();
			child_num++;
		} else {
			child_thread->wake_up_time = ktime_get_ns();
		}

unlock:
		raw_spin_unlock(&g_lock);
	}
}

int task_util_init(void)
{
	if (unlikely(!game_opt_dir))
		return -ENOTDIR;

	proc_create_data("game_pid", 0664, game_opt_dir, &game_pid_proc_ops, NULL);
	proc_create_data("heavy_task_info", 0444, game_opt_dir, &heavy_task_info_proc_ops, NULL);

	return 0;
}
