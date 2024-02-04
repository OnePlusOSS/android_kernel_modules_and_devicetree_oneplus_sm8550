/* drivers/misc/procs_cpu_usage.c
 * Copyright (c)  2023  Guangdong OPLUS Mobile Comm Corp., Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/timekeeping.h>
#include <linux/kernel_stat.h>
#include <linux/sort.h>
#include <linux/cpumask.h>
#include <linux/sched/cputime.h>
#include <linux/tick.h>
#include <asm/uaccess.h>


#define MIN_WINDOW_TIME		1000000000L
#define MAX_WINDOW_TIME		(10 * 60 * MIN_WINDOW_TIME)
#define MAX_THREAD_INFO		4096
#define TASK_INFO_MEM_SIZE	(MAX_THREAD_INFO * sizeof(struct task_info))

struct task_info {
	u64 sum_exec;
	pid_t pid;
	char comm[TASK_COMM_LEN];
};

struct procs_cpu_usage {
	struct task_info *curr_tasks;
	int curr_num;
	struct task_info *last_tasks;
	int last_num;
	struct task_info *rank_tasks;
	int rank_num;
	ktime_t last_win_start;
	struct kernel_cpustat last_cpustat;
	spinlock_t lock;
};

struct procs_cpu_usage g_pcu;


#ifdef arch_idle_time
static u64 cpu_idle_time(int cpu)
{
	u64 idle;

	idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
		idle += arch_idle_time(cpu);
	return idle;
}

static u64 cpu_iowait_time(int cpu)
{
	u64 iowait;

	iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	if (cpu_online(cpu) && nr_iowait_cpu(cpu))
		iowait += arch_idle_time(cpu);
	return iowait;
}
#else
static u64 cpu_idle_time(int cpu)
{
	u64 idle, idle_usecs = -1ULL;

	if (cpu_online(cpu))
		idle_usecs = get_cpu_idle_time_us(cpu, NULL);

	if (idle_usecs == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.idle */
		idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	else
		idle = idle_usecs * NSEC_PER_USEC;

	return idle;
}

static u64 cpu_iowait_time(int cpu)
{
	u64 iowait, iowait_usecs = -1ULL;

	if (cpu_online(cpu))
		iowait_usecs = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_usecs == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.iowait */
		iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	else
		iowait = iowait_usecs * NSEC_PER_USEC;

	return iowait;
}
#endif

static u64 cpustat_time(struct kernel_cpustat *now, struct kernel_cpustat *last)
{
	int i;
	u64 total_time = 0;

	for (i = CPUTIME_USER; i < NR_STATS; i++) {
		total_time += now->cpustat[i] - last->cpustat[i];
	}
	return total_time / num_possible_cpus();
}

static void get_cpustat(struct kernel_cpustat *stat)
{
	int i, cpu;

	memset(stat, 0, sizeof(struct kernel_cpustat));
	for_each_possible_cpu(cpu) {
		for (i = 0; i < NR_STATS; i++) {
			if (i == CPUTIME_IDLE)
				stat->cpustat[CPUTIME_IDLE] += cpu_idle_time(cpu);
			else if (i == CPUTIME_IOWAIT)
				stat->cpustat[CPUTIME_IOWAIT] += cpu_iowait_time(cpu);
			else
				stat->cpustat[i] += kcpustat_cpu(cpu).cpustat[i];
		}
	}
}

static int cmp_tasks(const void *a, const void *b)
{
	const struct task_info *s1 = a;
	const struct task_info *s2 = b;

	if (s1->sum_exec > s2->sum_exec)
		return -1;

	if (s1->sum_exec < s2->sum_exec)
		return 1;

	return 0;
}

static void calc_tasks_time(struct task_info *curr, int total)
{
	int i, j;
	int cnt = 0;
	u64 diff_exec;

	for (i = 0; i < g_pcu.last_num; i++) {
		for (j = 0; j < total; j++) {
			if (g_pcu.last_tasks[i].pid == curr[j].pid) {
				diff_exec = curr[j].sum_exec - g_pcu.last_tasks[i].sum_exec;
				if (diff_exec > 0 && diff_exec < MAX_WINDOW_TIME) {
					g_pcu.rank_tasks[cnt] = g_pcu.last_tasks[i];
					g_pcu.rank_tasks[cnt].sum_exec = diff_exec;
					cnt++;
				}
				break;
			}
		}
	}
	g_pcu.rank_num = cnt;
}

static int procs_cpu_usage_show(struct seq_file *m, void *v)
{
	struct task_struct *p, *t;
	int total = 0;
	int i;
	ktime_t start_time, end_time;
	u64 elapsed_time, window_time;
	struct task_info *swap_task_info;
	struct kernel_cpustat cpustat;

	spin_lock(&g_pcu.lock);
	start_time = ktime_get();
	window_time = ktime_sub(start_time, g_pcu.last_win_start);
	if (window_time < MIN_WINDOW_TIME) {
		printk(KERN_WARNING "window time too short: %lld ms.", ktime_to_ms(window_time));
		spin_unlock(&g_pcu.lock);
		return -EAGAIN;
	}

	g_pcu.last_win_start = start_time;
	get_cpustat(&cpustat);
	for_each_process(p) {
		u64 sum_thread = 0;
		for_each_thread(p, t) {
			sum_thread += t->se.sum_exec_runtime;
		}
		g_pcu.curr_tasks[total].sum_exec = sum_thread;
		g_pcu.curr_tasks[total].pid = p->pid;
		strcpy(g_pcu.curr_tasks[total].comm, p->comm);
		total++;
		if (total >= MAX_THREAD_INFO)
			break;
	}
	g_pcu.curr_num = total;

	if (g_pcu.last_num > 0) {
		int n_rank = 0;
		u64 total_cputime;

		calc_tasks_time(g_pcu.curr_tasks, total);
		sort(g_pcu.rank_tasks, g_pcu.rank_num, sizeof(struct task_info), cmp_tasks, NULL);
		n_rank = g_pcu.rank_num < 5 ? g_pcu.rank_num : 5;
		total_cputime = cpustat_time(&cpustat, &g_pcu.last_cpustat);

		for (i = 0; i < n_rank; i++) {
			seq_printf(m, "%d %d %s\n", g_pcu.rank_tasks[i].pid,
				(g_pcu.rank_tasks[i].sum_exec * 100) / total_cputime,
				g_pcu.rank_tasks[i].comm);
		}
	}

	/* swap curr task info to prev task info */
	swap_task_info = g_pcu.last_tasks;
	g_pcu.last_tasks = g_pcu.curr_tasks;
	g_pcu.curr_tasks = swap_task_info;
	g_pcu.last_num = g_pcu.curr_num;
	g_pcu.curr_num = 0;
	g_pcu.rank_num = 0;
	g_pcu.last_cpustat = cpustat;

	/* update stats cycle time */
	end_time = ktime_get();
	elapsed_time = ktime_to_us(ktime_sub(end_time, start_time));
	spin_unlock(&g_pcu.lock);

	printk(KERN_INFO "spent time: %lld us, window time: %lld ms\n",
		elapsed_time, window_time / 1000 / 1000);

	return 0;
}

static int procs_cpu_usage_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, procs_cpu_usage_show, PDE_DATA(inode));
}

static struct proc_ops procs_cpu_usage_fops = {
	.proc_open		= procs_cpu_usage_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release   = single_release,
};

static int __init procs_cpu_usage_init(void)
{
	struct proc_dir_entry *p_parent;

	g_pcu.curr_tasks = (struct task_info *)vmalloc(TASK_INFO_MEM_SIZE);
	if (!g_pcu.curr_tasks) {
		pr_err("%s: failed to malloc curr_tasks buffer.\n", __func__);
		goto err1;
	}

	g_pcu.last_tasks = (struct task_info *)vmalloc(TASK_INFO_MEM_SIZE);
	if (!g_pcu.last_tasks) {
		pr_err("%s: failed to malloc last_tasks buffer.\n", __func__);
		goto err2;
	}

	g_pcu.rank_tasks = (struct task_info *)vmalloc(TASK_INFO_MEM_SIZE);
	if (!g_pcu.rank_tasks) {
		pr_err("%s: failed to malloc rank_tasks buffer.\n", __func__);
		goto err3;
	}
	spin_lock_init(&g_pcu.lock);
	g_pcu.curr_num = 0;
	g_pcu.last_num = 0;
	g_pcu.rank_num = 0;

	p_parent = proc_mkdir("oplus_power", NULL);
	if (!p_parent) {
		pr_err("%s: failed to create oplus_power directory.\n", __func__);
		goto err4;
	}
	proc_create("top_process", 0444, p_parent, &procs_cpu_usage_fops);

	return 0;

err4:
	vfree(g_pcu.rank_tasks);
err3:
	vfree(g_pcu.last_tasks);
err2:
	vfree(g_pcu.curr_tasks);
err1:
	return -ENOMEM;
}

early_initcall(procs_cpu_usage_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xieshaohua");
MODULE_DESCRIPTION("process cpu usage stats for power");
