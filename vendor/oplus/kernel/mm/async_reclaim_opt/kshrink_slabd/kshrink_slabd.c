// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "shrink_async: " fmt

#include <linux/module.h>
#include <trace/hooks/vmscan.h>
#include <linux/swap.h>
#include <linux/proc_fs.h>
#include <linux/gfp.h>
#include <linux/types.h>
#include <linux/cpufreq.h>
#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/memcontrol.h>
#include <linux/mutex.h>

#define SHRINK_SLABD_NAME "kshrink_slabd"
extern unsigned long shrink_slab(gfp_t gfp_mask, int nid,
				 struct mem_cgroup *memcg,
				 int priority);

static int kshrink_slabd_pid;
static struct task_struct *shrink_slabd_tsk = NULL;
static bool async_shrink_slabd_setup = false;

wait_queue_head_t shrink_slabd_wait;

struct async_slabd_parameter {
	struct mem_cgroup *shrink_slabd_memcg;
	gfp_t shrink_slabd_gfp_mask;
	int shrink_slabd_runnable;
	int shrink_slabd_nid;
	int priority;
	struct reclaim_state *reclaim_state;
} asp;

static struct reclaim_state async_reclaim_state = {
		.reclaimed_slab = 0,
};

/* copy from mm/vmscan.c */
static void set_task_reclaim_state(struct task_struct *task,
				   struct reclaim_state *rs)
{
	/* Check for an overwrite */
	WARN_ON_ONCE(rs && task->reclaim_state);

	/* Check for the nulling of an already-nulled member */
	WARN_ON_ONCE(!rs && !task->reclaim_state);

	task->reclaim_state = rs;
}

static DEFINE_MUTEX(async_shrink_slab_mutex);

static bool is_shrink_slabd_task(struct task_struct *tsk)
{
	return tsk->pid == kshrink_slabd_pid;
}

bool wakeup_shrink_slabd(gfp_t gfp_mask, int nid,
				 struct mem_cgroup *memcg,
				 int priority)
{
	if (unlikely(!async_shrink_slabd_setup))
		return false;

	if (!mutex_trylock(&async_shrink_slab_mutex))
		return false;

	if (asp.shrink_slabd_runnable == 1) {
		mutex_unlock(&async_shrink_slab_mutex);
		return false;
	}
	async_reclaim_state.reclaimed_slab = 0;
	asp.reclaim_state = &async_reclaim_state;
	asp.shrink_slabd_gfp_mask = gfp_mask;
	asp.shrink_slabd_nid = nid;
	asp.shrink_slabd_memcg = memcg;
	asp.priority = priority;
	asp.shrink_slabd_runnable = 1;
	css_get(&memcg->css);
	mutex_unlock(&async_shrink_slab_mutex);

	wake_up_interruptible(&shrink_slabd_wait);
	return true;
}

void set_async_slabd_cpus(void)
{
	struct cpumask mask;
	struct cpumask *cpumask = &mask;
	pg_data_t *pgdat = NODE_DATA(0);
	unsigned int cpu = 0, cpufreq_max_tmp = 0;
	struct cpufreq_policy *policy_max;
	static bool set_slabd_cpus_success = false;

	if (unlikely(!async_shrink_slabd_setup))
		return;

	if (likely(set_slabd_cpus_success))
		return;
	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (policy == NULL)
			continue;

		if (policy->cpuinfo.max_freq >= cpufreq_max_tmp) {
			cpufreq_max_tmp = policy->cpuinfo.max_freq;
			policy_max = policy;
		}
	}

	cpumask_copy(cpumask, cpumask_of_node(pgdat->node_id));
	cpumask_andnot(cpumask, cpumask, policy_max->related_cpus);

	if (!cpumask_empty(cpumask)) {
		set_cpus_allowed_ptr(shrink_slabd_tsk, cpumask);
		set_slabd_cpus_success = true;
	}
}

static int kshrink_slabd_func(void *p)
{
	struct mem_cgroup *memcg;
	gfp_t gfp_mask;
	int nid, priority;
	/*
	 * Tell the memory management that we're a "memory allocator",
	 * and that if we need more memory we should get access to it
	 * regardless (see "__alloc_pages()"). "kswapd" should
	 * never get caught in the normal page freeing logic.
	 *
	 * (Kswapd normally doesn't need memory anyway, but sometimes
	 * you need a small amount of memory in order to be able to
	 * page out something else, and this flag essentially protects
	 * us from recursively trying to free more memory as we're
	 * trying to free the first piece of memory in the first place).
	 */
	current->flags |= PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD;
	set_freezable();

	asp.reclaim_state = NULL;
	asp.shrink_slabd_gfp_mask = 0;
	asp.shrink_slabd_nid = 0;
	asp.shrink_slabd_memcg = NULL;
	asp.shrink_slabd_runnable = 0;
	asp.priority = 0;

	while (!kthread_should_stop()) {
		wait_event_freezable(shrink_slabd_wait, asp.shrink_slabd_runnable == 1);

		set_async_slabd_cpus();

		mutex_lock(&async_shrink_slab_mutex);
		set_task_reclaim_state(current, asp.reclaim_state);
		nid = asp.shrink_slabd_nid;
		gfp_mask = asp.shrink_slabd_gfp_mask;
		priority = asp.priority;
		memcg = asp.shrink_slabd_memcg;
		asp.shrink_slabd_runnable = 0;
		mutex_unlock(&async_shrink_slab_mutex);

		shrink_slab(gfp_mask, nid, memcg, priority);
		css_put(&memcg->css);
		set_task_reclaim_state(current, NULL);
	}
	current->flags &= ~(PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD);
	current->reclaim_state = NULL;

	return 0;
}


static void should_shrink_async(void *data, gfp_t gfp_mask, int nid,
			struct mem_cgroup *memcg, int priority, bool *bypass)
{
	if (unlikely(!async_shrink_slabd_setup)) {
		*bypass = false;
		return;
	}

	if (is_shrink_slabd_task(current)) {
		*bypass = false;
	} else {
		*bypass = wakeup_shrink_slabd(gfp_mask, nid, memcg, priority);
	}
}

static int register_shrink_async_vendor_hooks(void)
{
	int ret = 0;

	ret = register_trace_android_vh_shrink_slab_bypass(should_shrink_async, NULL);
	if (ret != 0) {
		pr_err("register_trace_android_vh_shrink_slab_bypass failed! ret=%d\n", ret);
		goto out;
	}
out:
	return ret;
}

static void unregister_shrink_async_vendor_hooks(void)
{
	unregister_trace_android_vh_shrink_slab_bypass(should_shrink_async, NULL);

	return;
}

static int __init shrink_async_init(void)
{
	int ret = 0;

	ret = register_shrink_async_vendor_hooks();
	if (ret != 0)
		return ret;

	init_waitqueue_head(&shrink_slabd_wait);

	shrink_slabd_tsk = kthread_run(kshrink_slabd_func, NULL, SHRINK_SLABD_NAME);
	if (IS_ERR_OR_NULL(shrink_slabd_tsk)) {
		pr_err("Failed to start shrink_slabd on node 0\n");
		ret = PTR_ERR(shrink_slabd_tsk);
		shrink_slabd_tsk = NULL;
		unregister_shrink_async_vendor_hooks();
		return ret;
	}

	kshrink_slabd_pid = shrink_slabd_tsk->pid;
	async_shrink_slabd_setup = true;

	pr_info("kshrink_async succeed!\n");
	return 0;
}

static void __exit shrink_async_exit(void)
{
	unregister_shrink_async_vendor_hooks();

	pr_info("shrink_async exit succeed!\n");

	return;
}

module_init(shrink_async_init);
module_exit(shrink_async_exit);

MODULE_LICENSE("GPL v2");
