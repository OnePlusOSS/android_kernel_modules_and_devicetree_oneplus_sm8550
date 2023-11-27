#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/uaccess.h>
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#include "frame_group.h"
#include "cluster_boost.h"

static DEFINE_RAW_SPINLOCK(preferred_cluster_id_lock);
/*
 * Only interested in these threads their tgid euqal to user_interested_tgid.
 * the purpose of adding this condition is to exclude these threads created
 * befor FBG module init, the preferred_cluster_id of their oplus_task_struct is 0,
 * they will be preferred to run cluster 0 incorrectly.
 */
static atomic_t user_interested_tgid = ATOMIC_INIT(-1);

int __fbg_set_task_preferred_cluster(pid_t tid, int cluster_id)
{
	struct task_struct *task = NULL;
	struct oplus_task_struct *ots = NULL;
	unsigned long flags;

	rcu_read_lock();
	task = find_task_by_vpid(tid);
	if (task)
		get_task_struct(task);
	rcu_read_unlock();

	if (task) {
		ots = get_oplus_task_struct(task);
		if (IS_ERR_OR_NULL(ots)) {
			put_task_struct(task);
			return 0;
		}
		atomic_set(&user_interested_tgid, task->tgid);
		raw_spin_lock_irqsave(&preferred_cluster_id_lock, flags);
		if ((cluster_id >= 0) && (cluster_id < num_sched_clusters))
			ots->preferred_cluster_id = cluster_id;
		else
			ots->preferred_cluster_id = -1;
		raw_spin_unlock_irqrestore(&preferred_cluster_id_lock, flags);
		put_task_struct(task);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(__fbg_set_task_preferred_cluster);

bool fbg_cluster_boost(struct task_struct *p, int *target_cpu)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(p);
	unsigned long flags;
	int preferred_cluster_id;
	struct oplus_sched_cluster *preferred_cluster;
	struct cpumask *preferred_cpus;
	cpumask_t search_cpus = CPU_MASK_NONE;
	int iter_cpu;
	int active_cpu = -1;
	int max_spare_cap_cpu = -1;
	unsigned long spare_cap = 0, max_spare_cap = 0;

	if (IS_ERR_OR_NULL(ots))
		return false;
	if (likely(atomic_read(&user_interested_tgid) != p->tgid))
		return false;

	raw_spin_lock_irqsave(&preferred_cluster_id_lock, flags);
	preferred_cluster_id = ots->preferred_cluster_id;
	raw_spin_unlock_irqrestore(&preferred_cluster_id_lock, flags);

	if (preferred_cluster_id < 0 || preferred_cluster_id >= num_sched_clusters)
		return false;

	preferred_cluster = fb_cluster[preferred_cluster_id];
	preferred_cpus = &preferred_cluster->cpus;
	cpumask_and(&search_cpus, &p->cpus_mask, cpu_active_mask);
#ifdef CONFIG_OPLUS_ADD_CORE_CTRL_MASK
	if (fbg_cpu_halt_mask)
		cpumask_andnot(&search_cpus, &search_cpus, fbg_cpu_halt_mask);
#endif
	cpumask_and(&search_cpus, &search_cpus, preferred_cpus);

	for_each_cpu(iter_cpu, &search_cpus) {
		if (active_cpu == -1)
			active_cpu = iter_cpu;

		if (available_idle_cpu(iter_cpu) || (iter_cpu == task_cpu(p) && p->__state == TASK_RUNNING)) {
			max_spare_cap_cpu = iter_cpu;
			break;
		}

		spare_cap = max_t(long, capacity_of(iter_cpu) - cpu_util_without(iter_cpu, p), 0);
		if (spare_cap > max_spare_cap) {
			max_spare_cap = spare_cap;
			max_spare_cap_cpu = iter_cpu;
		}
	}

	if (max_spare_cap_cpu == -1)
		max_spare_cap_cpu = active_cpu;

	if ((max_spare_cap_cpu == -1)
		|| ((cpumask_weight(&search_cpus) == 1) && (!available_idle_cpu(max_spare_cap_cpu)))) {
		return false;
	}

	*target_cpu = max_spare_cap_cpu;

	return true;
}
