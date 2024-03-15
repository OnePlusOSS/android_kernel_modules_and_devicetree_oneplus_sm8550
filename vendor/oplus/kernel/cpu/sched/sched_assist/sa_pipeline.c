#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/syscore_ops.h>
#include <linux/sched/cputime.h>
#include <kernel/sched/sched.h>

#include "sa_pipeline.h"

#define MAX_PIPELINE_TASK_NUM 5
static int pipeline_pids[MAX_PIPELINE_TASK_NUM] = {-1, -1, -1, -1, -1};
static int pipeline_cpus[MAX_PIPELINE_TASK_NUM] = {-1, -1, -1, -1, -1};

#define PIPELINE_TASK_UX_STATE (UX_PRIORITY_PIPELINE | SA_TYPE_HEAVY)

#define ENABLE_REJECT_MIGRATE_PIPELINE_TASK 0
#define MAX_REJECT_MIGRATE_TIME 200000 /* 200us */

static DEFINE_MUTEX(p_mutex);

#if ENABLE_REJECT_MIGRATE_PIPELINE_TASK
static ktime_t ktime_last;
static bool ux_ktime_suspended;

static u64 ux_ktime_get_ns(void)
{
	if (unlikely(ux_ktime_suspended))
		return ktime_to_ns(ktime_last);

	return ktime_get_ns();
}

static void ux_resume(void)
{
	ux_ktime_suspended = false;
}

static int ux_suspend(void)
{
	ktime_last = ktime_get();
	ux_ktime_suspended = true;
	return 0;
}

static struct syscore_ops ux_syscore_ops = {
	.resume		= ux_resume,
	.suspend	= ux_suspend
};
#endif

static inline bool is_valid_pipeline_task(int pipeline_cpu, int ux_state)
{
	return (pipeline_cpu > 0) && (pipeline_cpu < nr_cpu_ids) &&
		((ux_state & PIPELINE_TASK_UX_STATE) == PIPELINE_TASK_UX_STATE);
}

inline int oplus_get_task_pipeline_cpu(struct task_struct *task)
{
	struct oplus_task_struct *ots;
	int pipeline_cpu;

	if (unlikely(!global_sched_assist_enabled))
		return -1;

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		return -1;

	pipeline_cpu = ots->pipeline_cpu;
	if (is_valid_pipeline_task(pipeline_cpu, ots->ux_state))
		return pipeline_cpu;

	return -1;
}
EXPORT_SYMBOL_GPL(oplus_get_task_pipeline_cpu);

inline bool oplus_is_pipeline_task(struct task_struct *task)
{
	struct oplus_task_struct *ots;

	ots = get_oplus_task_struct(task);
	if (IS_ERR_OR_NULL(ots))
		return false;

	return is_valid_pipeline_task(ots->pipeline_cpu, ots->ux_state);
}

inline bool pipeline_task_skip_ux_change(struct oplus_task_struct *ots, int *ux_state)
{
	/* no pipeline task, simply return */
	if (!is_valid_pipeline_task(ots->pipeline_cpu, ots->ux_state))
		return false;

	/* pipeline task, not allowed to set ux_state 0 */
	if (*ux_state == 0)
		return true;

	/* pipeline task, not allowed to change ux_state, unless add SA_TYPE_INHERIT */
	*ux_state &= ~SCHED_ASSIST_UX_MASK;
	*ux_state &= ~SCHED_ASSIST_UX_PRIORITY_MASK;
	*ux_state |= PIPELINE_TASK_UX_STATE;

	return false;
}

inline bool pipeline_task_skip_cpu(struct task_struct *task, unsigned int dst_cpu)
{
#if ENABLE_REJECT_MIGRATE_PIPELINE_TASK
	struct oplus_task_struct *ots = get_oplus_task_struct(task);
	struct rq *rq;
	u64 now;
	s64 wait_time;
	bool skip = false;

	if (IS_ERR_OR_NULL(ots) || !is_valid_pipeline_task(ots->pipeline_cpu, ots->ux_state))
		return false;

	rq = task_rq(task);
	if (unlikely(rq->cpu == dst_cpu))
		return false;

	now = ux_ktime_get_ns();
	wait_time = now - max(ots->pipeline_enqueue_ts, ots->pipeline_switch_out_ts);

	/* befor allowing pipeline task migrated to another cpu, waiting for MAX_REJECT_MIGRATE_TIME ns */
	if ((wait_time >= 0) && (wait_time < MAX_REJECT_MIGRATE_TIME))
		skip = true;

	if (unlikely(global_debug_enabled & DEBUG_PIPELINE))
		trace_printk("comm=%s pid=%d tgid=%d, %s migrating to dst_cpu=%d from src_cpu=%d, wait_time=%lld ns\n",
			task->comm, task->pid, task->tgid, skip? "rejected" : "allowed", dst_cpu, rq->cpu, wait_time);

	return skip;
#else
	return false;
#endif
}

inline void pipeline_task_enqueue(struct oplus_task_struct *ots)
{
#if ENABLE_REJECT_MIGRATE_PIPELINE_TASK
	if (!is_valid_pipeline_task(ots->pipeline_cpu, ots->ux_state))
		return;

	ots->pipeline_enqueue_ts = ux_ktime_get_ns();
#endif
}

inline void pipeline_task_switch_out(struct task_struct *prev)
{
#if ENABLE_REJECT_MIGRATE_PIPELINE_TASK
	struct oplus_task_struct *ots = get_oplus_task_struct(prev);

	if (IS_ERR_OR_NULL(ots) || !is_valid_pipeline_task(ots->pipeline_cpu, ots->ux_state))
		return;

	/* recrod pipeline task be preempted timestatmp, probably rt task preempts pipeline task */
	ots->pipeline_switch_out_ts = ux_ktime_get_ns();
#endif
}

core_ctl_set_boost_t oplus_core_ctl_set_boost = NULL;
EXPORT_SYMBOL_GPL(oplus_core_ctl_set_boost);
core_ctl_set_cluster_boost_t oplus_core_ctl_set_cluster_boost = NULL;
EXPORT_SYMBOL_GPL(oplus_core_ctl_set_cluster_boost);

static cpumask_t cpus_for_pipeline = { CPU_BITS_NONE };
static inline void pipeline_set_boost(bool boost)
{
	int i, j, cpu;
	struct cpumask pipeline_cpu_mask;
	struct cpufreq_policy policy;
	struct cpu_topology *cpu_topo;

	if (oplus_core_ctl_set_boost != NULL) {
		oplus_core_ctl_set_boost(boost);
		printk("oplus_core_ctl_set_boost, boost=%d\n", boost);
		return;
	}

	cpumask_copy(&pipeline_cpu_mask, &cpus_for_pipeline);

	if (oplus_core_ctl_set_cluster_boost != NULL) {
		for_each_cpu(i, &pipeline_cpu_mask) {
			if (cpufreq_get_policy(&policy, i))
				continue;

			for_each_cpu(j, policy.related_cpus)
				cpumask_clear_cpu(j, &pipeline_cpu_mask);

			cpu = policy.cpu;

			cpu_topo = &cpu_topology[cpu];
			if (cpu_topo->package_id == -1)
				continue;

			oplus_core_ctl_set_cluster_boost(cpu_topo->package_id, boost);
			printk("oplus_core_ctl_set_cluster_boost, cluster_id=%d, boost=%d\n",
				cpu_topo->package_id, boost);
		}
	}
}

static ssize_t pipeline_pids_proc_write(struct file *file,
			const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[256] = {0};
	int pids[MAX_PIPELINE_TASK_NUM] = {-1, -1, -1, -1, -1};
	int cpus[MAX_PIPELINE_TASK_NUM] = {-1, -1, -1, -1, -1};
	int ret;
	int i;
	struct rq *rq;
	struct rq_flags rf;
	struct task_struct *task;
	struct oplus_task_struct *ots;

	ret = simple_write_to_buffer(buffer, sizeof(buffer) - 1, ppos, buf, count);
	if (ret <= 0)
		return ret;

	ret = sscanf(buffer, "%d %d %d %d %d %d %d %d %d %d", &pids[0], &cpus[0],
			&pids[1], &cpus[1], &pids[2], &cpus[2],
			&pids[3], &cpus[3], &pids[4], &cpus[4]);

	if (ret < 0)
		return -EINVAL;

	for (i = 0; i < MAX_PIPELINE_TASK_NUM; i++) {
		if (!((cpus[i] == -1) || ((cpus[i] > 0) && (cpus[i] < nr_cpu_ids))))
			return -EINVAL;
	}

	mutex_lock(&p_mutex);

	if (pipeline_pids[0] != -1) {
		for (i = 0; i < MAX_PIPELINE_TASK_NUM; i++) {
			if (pipeline_pids[i] == -1)
				continue;

			/* get_pid_task have called get_task_struct */
			task = get_pid_task(find_vpid(pipeline_pids[i]), PIDTYPE_PID);
			if (task) {
				rq = task_rq_lock(task, &rf);
				ots = get_oplus_task_struct(task);
				if (!IS_ERR_OR_NULL(ots)) {
					ots->pipeline_cpu = -1;
					oplus_set_ux_state_lock(task, 0, false);
				}
				task_rq_unlock(rq, task, &rf);
				put_task_struct(task);
			}

			pipeline_pids[i] = -1;
			pipeline_cpus[i] = -1;
		}

		if (!cpumask_empty(&cpus_for_pipeline)) {
			pipeline_set_boost(false);
			cpumask_clear(&cpus_for_pipeline);
		}
	}

	if (pids[0] != -1) {
		for (i = 0; i < MAX_PIPELINE_TASK_NUM; i++) {
			if (pids[i] == -1)
				continue;

			/* get_pid_task have called get_task_struct */
			task = get_pid_task(find_vpid(pids[i]), PIDTYPE_PID);
			if (task) {
				rq = task_rq_lock(task, &rf);
				ots = get_oplus_task_struct(task);
				if (!IS_ERR_OR_NULL(ots)) {
					oplus_set_ux_state_lock(task, PIPELINE_TASK_UX_STATE, false);
					ots->pipeline_cpu = cpus[i];
				}
				task_rq_unlock(rq, task, &rf);
				put_task_struct(task);

				pipeline_pids[i] = pids[i];
				pipeline_cpus[i] = cpus[i];

				cpumask_set_cpu(cpus[i], &cpus_for_pipeline);
			}
		}

		if (!cpumask_empty(&cpus_for_pipeline))
			pipeline_set_boost(true);
	}

	mutex_unlock(&p_mutex);

	return count;
}

static ssize_t pipeline_pids_proc_read(struct file *file,
			char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[256] = {0};
	int len;

	mutex_lock(&p_mutex);
	len = snprintf(buffer, sizeof(buffer), "%d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\n",
		pipeline_pids[0], pipeline_cpus[0], pipeline_pids[1], pipeline_cpus[1],
		pipeline_pids[2], pipeline_cpus[2], pipeline_pids[3], pipeline_cpus[3],
		pipeline_pids[4], pipeline_cpus[4]);
	mutex_unlock(&p_mutex);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static const struct proc_ops pipeline_pids_proc_ops = {
	.proc_write		= pipeline_pids_proc_write,
	.proc_read		= pipeline_pids_proc_read,
	.proc_lseek		= default_llseek,
};

void oplus_pipeline_init(struct proc_dir_entry *pde)
{
	proc_create("pipeline_pids_cpus", (S_IRUGO|S_IWUSR|S_IWGRP), pde, &pipeline_pids_proc_ops);

#if ENABLE_REJECT_MIGRATE_PIPELINE_TASK
	register_syscore_ops(&ux_syscore_ops);
#endif
}
