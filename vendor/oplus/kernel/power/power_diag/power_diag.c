/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/threads.h>
#include <linux/cgroup-defs.h>
#include <linux/sched/stat.h>
#include <linux/sched/nohz.h>
#include <linux/sched/topology.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/suspend.h>
#include <linux/sort.h>
#include <linux/cpufreq.h>
#include <linux/kernel_stat.h>
#include <linux/energy_model.h>
#include <trace/hooks/sched.h>
#include "sched.h"
#include "power_diag.h"
#include "../../../../kernel/workqueue_internal.h"
#if 0
#define CREATE_TRACE_POINTS
#include "trace_powerd.h"
#endif
#define power_diag_tag "[POWER_DIAG]"
#define power_diag_info(fmt, args...) pr_info(power_diag_tag fmt, ##args)
#define power_diag_err(fmt, args...) pr_err(power_diag_tag fmt, ##args)
#define CPU_NUM (8)
#define MAX_PID (32768)
#define CTP_WINDOW_SZ (5)
#define NUM_PRINT_HEAVY_LOAD_TASK (15)
#define SYSTEM_POWER_ON_TIME (300)
#define MAX_ACTIVE_WAKEUP_SOUCE_LEN (256)
#define POWER_DEBUG_TIME_INTERVAL_MIN (5)
#define POWER_DEBUG_TIME_INTERVAL_MAX (600)

static int cputime_one_jiffy;
struct power_debug power_debug_info;
static bool is_enter_suspend = false;
unsigned int debug_power_timer_interval_s = 5;
static unsigned int pm_qos_debug_flag = 0;
struct task_stat cpustats_saved[MAX_PID];
static struct kernel_task_cpustat_cp k_cpustat[CPU_NUM];
static char ws_msg[MAX_ACTIVE_WAKEUP_SOUCE_LEN];

static int get_power(int cpu, int freq) {
	int i;
	struct em_perf_domain *domain = em_cpu_get(cpu);
	if (!domain)
		goto err_found;
	for (i = domain->nr_perf_states - 1; i > -1; i--) {
		struct em_perf_state* cs = domain->table + i;
		if (cs->frequency == freq) {
			return cs->power;
		}
	}
err_found:
	pr_err("not found %d %d in sge.\n", cpu, freq);
	return 0;
}

static inline struct kthread_cp *to_kthread_cp(struct task_struct *k)
{
	WARN_ON(!(k->flags & PF_KTHREAD));
	return (__force void *)k->set_child_tid;
}
void *kthread_data_cp(struct task_struct *task)
{
	return to_kthread_cp(task)->data;
}
static inline struct worker* get_worker(struct task_struct *p)
{
	struct worker *worker = NULL;
	if (p && (p->flags & PF_WQ_WORKER))
		worker = kthread_data_cp(p);
	return worker;
}

static void print_heavy_loads_tasks()
{
	int i = 0;
	for (; i < NUM_PRINT_HEAVY_LOAD_TASK; i++) {
		if (!cpustats_saved[i].pwr)
			break;
		if (cpustats_saved[i].worker.kworker)
			power_diag_info("%s\t%u\t%u\t%u\t%u\t%u\t%ps(cf)\t%ps(lf)\n", cpustats_saved[i].comm, cpustats_saved[i].pid,
				cpustats_saved[i].tgid, cpustats_saved[i].pwr, cpustats_saved[i].lcore_pwr, cpustats_saved[i].r_time,
				cpustats_saved[i].worker.current_func, cpustats_saved[i].worker.last_func);
		else
			power_diag_info("%s\t%u\t%u\t%u\t%u\t%u\n", cpustats_saved[i].comm, cpustats_saved[i].pid,
				cpustats_saved[i].tgid, cpustats_saved[i].pwr, cpustats_saved[i].lcore_pwr, cpustats_saved[i].r_time);
	}
}

static int cpustats_saved_comp(const void *a, const void *b)
{
	const struct task_stat sa = *(const struct task_stat *)a;
	const struct task_stat sb = *(const struct task_stat *)b;
	return sb.pwr - sa.pwr;
}
static void parse_task_stats()
{
	int i = 0, num = 0;

	while (i < MAX_PID && num <MAX_PID) {
		while (num < MAX_PID && cpustats_saved[num].pwr)
			num++;
		while (i < MAX_PID && !cpustats_saved[i].pwr)
			i++;
		if (i < num) {
			i = num + 1;
			continue;
		}
		if (num < MAX_PID && i < MAX_PID) {
			cpustats_saved[num] = cpustats_saved[i];
			memset(&cpustats_saved[i], 0, sizeof(struct task_stat));
		}
		i++;
		num++;
	}
	if (num >= MAX_PID)
		num = MAX_PID - 1;
	sort(cpustats_saved, num, sizeof(struct task_stat), cpustats_saved_comp, NULL);
}

static void task_power_stats()
{
	int i, j;
	unsigned long begin = jiffies - CTP_WINDOW_SZ * HZ, end = jiffies;
	unsigned int tmp_pwr;
	memset(cpustats_saved, 0, sizeof(cpustats_saved));
	for_each_possible_cpu(i) {
		struct kernel_task_cpustat_cp* kstat = &k_cpustat[i];
		for (j = 0; j < MAX_CTP_WINDOW_TICK; j++) {
			struct task_cpustat_cp *ts = kstat->cpustat + j;
			unsigned long r_time = ts->end - ts->begin;
			if (ts->pid >= MAX_PID)
				continue;
			if (ts->begin >= begin && ts->end <= end) {
				struct task_stat *as = cpustats_saved + ts->pid;
				if (as->pwr == 0) {
					memcpy(as->comm, ts->comm, TASK_COMM_LEN);
					as->pid = ts->pid;
					as->tgid = ts->tgid;
					memcpy(&as->worker, &ts->worker, sizeof(struct kworker_stat));
				}
				/* 4 ms each tick */
				tmp_pwr = get_power(i, ts->freq) * jiffies_to_msecs(r_time);

				if (ts->l_core)
					as->lcore_pwr += tmp_pwr;
				as->pwr += tmp_pwr;
				as->r_time += jiffies_to_msecs(r_time);
			}
		}
	}
	parse_task_stats();
	print_heavy_loads_tasks();
}
static inline unsigned long cfs_util(int cpu)
{
	struct cfs_rq *cfs_rq;
	unsigned int util;

	cfs_rq = &cpu_rq(cpu)->cfs;
	util = READ_ONCE(cfs_rq->avg.util_avg);

	if (sched_feat(UTIL_EST))
		util = max(util, READ_ONCE(cfs_rq->avg.util_est.enqueued));

	return min_t(unsigned long, util, capacity_orig_of(cpu));
}

static void print_active_wakeup_sources()
{
	memset(ws_msg, 0, sizeof(ws_msg));
	pm_get_active_wakeup_sources(ws_msg, MAX_ACTIVE_WAKEUP_SOUCE_LEN);
	power_diag_info("%s\n", ws_msg);
	if (strstr(ws_msg, "cmdq") && !strstr(ws_msg, "crtc0"))
		cmdq_dump_usage();
}
static void debug_power_timer_func(struct timer_list *unused)
{
	unsigned int cpu_freq = 0;
	int cpu, c_cap = 0, c_util = 0;

	if (is_enter_suspend)
		return;

	for_each_possible_cpu(cpu) {
		c_cap = arch_scale_cpu_capacity(cpu);
		c_util = cfs_util(cpu);
		cpu_freq = cpufreq_quick_get(cpu) / 1000;
		power_diag_info("cpu_info(%d): %dMhz\t%d(c_util)\t%d(cap)\n",
			cpu, cpu_freq, c_util, c_cap);
	}

	task_power_stats();
	power_diag_info("print active wakeup sources\n");
	print_active_wakeup_sources();
	mod_timer(&power_debug_info.debug_power_timer, jiffies + debug_power_timer_interval_s * HZ);

}

static int debug_power_pm_event_func(struct notifier_block *notifier, unsigned long pm_event, void *unused)
{
	switch (pm_event) {
		case PM_HIBERNATION_PREPARE:
		case PM_RESTORE_PREPARE:
		case PM_POST_HIBERNATION:
			return NOTIFY_DONE;
		case PM_SUSPEND_PREPARE:
			is_enter_suspend = true;
			power_diag_info("SUSPEND DETECTED\n");
			del_timer_sync(&power_debug_info.debug_power_timer);
			return NOTIFY_DONE;
		case PM_POST_SUSPEND:
			is_enter_suspend = false;
			power_diag_info("RESUME DETECTED\n");
			if (timer_pending(&power_debug_info.debug_power_timer))
				power_diag_err("timer error after suspend\n");
			else
				mod_timer(&power_debug_info.debug_power_timer, jiffies + debug_power_timer_interval_s * HZ);
			return NOTIFY_DONE;
	}
	return NOTIFY_DONE;
}

static ssize_t power_debug_interval_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", debug_power_timer_interval_s);
}
static ssize_t power_debug_interval_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
	unsigned int val;
	int err;
	err = -EINVAL;
	if (kstrtoint(buf, 0, &val)) {
		power_diag_err("Failed to save power_debug_interval.\n");
		return err;
	}
	if (val > POWER_DEBUG_TIME_INTERVAL_MAX || val < POWER_DEBUG_TIME_INTERVAL_MIN) {
		power_diag_err("power_debug_interval out of limit(%d %d)\n",
			POWER_DEBUG_TIME_INTERVAL_MIN, POWER_DEBUG_TIME_INTERVAL_MAX);
		return err;
	}
	debug_power_timer_interval_s = val;
	power_diag_info("power_debug_interval saved to %u\n", debug_power_timer_interval_s);
	err = n;
	return err;
}
power_debug_attr(power_debug_interval);

static ssize_t pm_qos_flag_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pm_qos_debug_flag);
}
static ssize_t pm_qos_flag_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
	unsigned int val;
	int err;
	err = -EINVAL;
	if (kstrtoint(buf, 0, &val)) {
		power_diag_err("Failed to save pm_qos_debug_flag.\n");
		return err;
	}
	pm_qos_debug_flag = val;
	power_diag_info("pm_qos_debug_flag saved to %u\n", pm_qos_debug_flag);
	err = n;
	return err;
}
power_debug_attr(pm_qos_flag);

static struct attribute * power_debug[] = {
	&power_debug_interval_attr.attr,
	&pm_qos_flag_attr.attr,
	NULL,
};

static const struct attribute_group power_debug_attr_group = {
	.attrs = power_debug,
};
#if 0
static void parse_sched_setaffinity(void *ignore, struct task_struct *p,
	const struct cpumask *in_mask, int *retval) {
	int cpu = cpumask_first(in_mask);
	if (cpu == 4 || cpu == 7)
		power_diag_info("task %s bind to big cpus %lx\n", p->comm, cpumask_bits(in_mask)[0]);
}

static void parse_sched_setuclamp(void *ignore, struct task_struct *p,
	int clamp_id, unsigned int uclamp_val) {
	if (clamp_id == UCLAMP_MIN)
		trace_sched_setuclamp(p, uclamp_val);
}
#endif
static void account_task_time_cp(void *data, struct task_struct *p, struct rq *rq, int user_tick) {
	int idx;
	struct kernel_task_cpustat_cp *kstat;
	struct task_cpustat_cp *s;
	int cpu = task_cpu(p);
	struct worker *t_kworker;
	if (p == rq->idle)
		return;

	if (!cputime_one_jiffy)
		cputime_one_jiffy = nsecs_to_jiffies(TICK_NSEC);
	kstat = &k_cpustat[cpu];
	idx = kstat->idx % MAX_CTP_WINDOW_TICK;
	s = &kstat->cpustat[idx];
	s->pid = p->pid;
	s->tgid = p->tgid;
	s->l_core = arch_scale_cpu_capacity(cpu) == SCHED_CAPACITY_SCALE ? 0 : 1;
	s->freq = cpufreq_quick_get(cpu);
	s->begin = jiffies - cputime_one_jiffy;
	s->end = jiffies;
	s->worker.kworker = p->flags & PF_WQ_WORKER;
	if (s->worker.kworker) {
		t_kworker = get_worker(p);
		if (t_kworker) {
			s->worker.current_func = t_kworker->current_func;
			s->worker.last_func = t_kworker->last_func;
		}
	}
	memcpy(s->comm, p->comm, TASK_COMM_LEN);
	kstat->idx = idx + 1;
}

static int register_vendor_hooks()
{
	int rc = 0;
#if 0
	rc = register_trace_android_rvh_sched_setaffinity(parse_sched_setaffinity, NULL);
	if (rc != 0) {
		power_diag_err("register_trace_android_rvh_sched_setaffinity failed! rc=%d\n",rc);
		return rc;
	}

	rc = register_trace_android_vh_setscheduler_uclamp(parse_sched_setuclamp, NULL);
	if (rc != 0) {
		power_diag_err("register_trace_android_vh_setscheduler_uclamp failed! rc=%d\n",rc);
		return rc;
	}
#endif
	rc = register_trace_android_vh_account_task_time(account_task_time_cp, NULL);
	if (rc != 0) {
		power_diag_err("register_trace_android_rvh_sched_setaffinity failed! rc=%d\n",rc);
		return rc;
	}
	return 0;
}
static int __init init_debug_power(void)
{
	int res = 0;
	register_vendor_hooks();
	power_debug_info.debug_load_pm_event.notifier_call = debug_power_pm_event_func;
	timer_setup(&power_debug_info.debug_power_timer, debug_power_timer_func, 0);
#ifdef CONFIG_PM
	res = register_pm_notifier(&power_debug_info.debug_load_pm_event);
	if (res) {
		power_diag_err("Failed to register PM notifier.\n");
		return res;
	}
#endif

	mod_timer(&power_debug_info.debug_power_timer, jiffies + SYSTEM_POWER_ON_TIME * HZ);//fixme

	res = sysfs_create_group(kernel_kobj, &power_debug_attr_group);
	if (res)
		return res;
	return 0;
}

late_initcall(init_debug_power);
MODULE_LICENSE("GPL v2");