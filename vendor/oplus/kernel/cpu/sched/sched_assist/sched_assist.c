// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <kernel/sched/sched.h>
#include <trace/hooks/cgroup.h>
#include <trace/hooks/signal.h>

#include "sched_assist.h"
#include "sa_common.h"
#include "sa_sysfs.h"
#include "sa_exec.h"
#include "sa_fair.h"
#include "sa_oemdata.h"


static int register_scheduler_vendor_hooks(void)
{
	int ret = 0;

	/* register vender hook in kernel/sched/fair.c */
	REGISTER_TRACE_RVH(android_rvh_place_entity, android_rvh_place_entity_handler);
	REGISTER_TRACE_RVH(android_rvh_pick_next_entity, android_rvh_pick_next_entity_handler);
	REGISTER_TRACE_RVH(android_rvh_can_migrate_task, android_rvh_can_migrate_task_handler);
	/* REGISTER_TRACE_RVH(android_rvh_select_task_rq_fair, android_rvh_select_task_rq_fair_handler); */
	/* REGISTER_TRACE_RVH(android_rvh_find_energy_efficient_cpu, android_rvh_find_energy_efficient_cpu_handler); */

#ifndef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	REGISTER_TRACE_RVH(android_rvh_check_preempt_wakeup, android_rvh_check_preempt_wakeup_handler);
	REGISTER_TRACE_RVH(android_rvh_replace_next_task_fair, android_rvh_replace_next_task_fair_handler);
#endif

	/* register vender hook in kernel/sched/topology.c */
	REGISTER_TRACE_VH(android_vh_build_sched_domains, android_vh_build_sched_domains_handler);

	/* register vender hook in  kernel/sched/rt.c */
#ifndef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	/* REGISTER_TRACE_RVH(android_rvh_select_task_rq_rt, android_rvh_select_task_rq_rt_handler); */
	REGISTER_TRACE_RVH(android_rvh_find_lowest_rq, android_rvh_find_lowest_rq_handler);
#endif

	/* register vender hook in kernel/sched/core.c */
	REGISTER_TRACE_RVH(android_rvh_sched_fork, android_rvh_sched_fork_handler);
	REGISTER_TRACE_RVH(android_rvh_schedule, android_rvh_schedule_handler);
	REGISTER_TRACE_RVH(android_vh_scheduler_tick, android_vh_scheduler_tick_handler);
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	REGISTER_TRACE_RVH(android_rvh_enqueue_task, android_rvh_enqueue_task_handler);
	REGISTER_TRACE_RVH(android_rvh_dequeue_task, android_rvh_dequeue_task_handler);
#endif

	/* register vender hook in fs/exec.c */
	REGISTER_TRACE_VH(task_rename, task_rename_handler);

	/* register vendor hook in kernel/cgroup/cgroup-v1.c */
	REGISTER_TRACE_VH(android_vh_cgroup_set_task, android_vh_cgroup_set_task_handler);
	/* register vendor hook in kernel/signal.c  */
	REGISTER_TRACE_VH(android_vh_exit_signal, android_vh_exit_signal_handler);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_BAN_APP_SET_AFFINITY)
	/* register vendor hook in kernel/core.c */
	REGISTER_TRACE_VH(android_vh_sched_setaffinity_early, android_vh_sched_setaffinity_early_handler);
#endif
	return 0;
}

static int __init oplus_sched_assist_init(void)
{
	int ret;

	ret = sa_oemdata_init();
	if (ret != 0)
		return ret;

	global_sched_assist_enabled |= FEATURE_COMMON;
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
	global_sched_assist_enabled |= FEATURE_SPREAD;
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

	sched_assist_init_oplus_rq();
	update_ux_sched_cputopo();

	ret = oplus_sched_assist_proc_init();
	if (ret != 0)
		return ret;

	ret = register_scheduler_vendor_hooks();
	if (ret != 0)
		return ret;

	ux_debug("sched assist init succeed!\n");
	return 0;
}

module_init(oplus_sched_assist_init);

MODULE_DESCRIPTION("Oplus Sched Assist Vender Hooks Driver");
MODULE_LICENSE("GPL v2");
