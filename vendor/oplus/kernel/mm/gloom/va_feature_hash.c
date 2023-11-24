// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/mm.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/rculist.h>
#include <linux/hash.h>
#include <linux/types.h>
#include <linux/cache.h>
#include <linux/spinlock_types.h>
#include <linux/pid_namespace.h>
#include <linux/init_task.h>
#include <linux/syscalls.h>
#include <linux/proc_ns.h>
#include <linux/proc_fs.h>
#include <linux/anon_inodes.h>
#include <linux/sched/signal.h>
#include <linux/sched/task.h>

#include "va_feature_hash.h"
#include "reserve_area.h"

#define HASH_TABLE_SHIFT 5
#define pid_va_feature_hashfn(nr, ns)	\
	hash_long((unsigned long)nr + (unsigned long)ns, HASH_TABLE_SHIFT)

static struct hlist_head *pid_va_feature_hash;
static __cacheline_aligned_in_smp DEFINE_SPINLOCK(pidhash_lock);

int pid_va_feature_hash_init(void)
{
	int size = sizeof(*pid_va_feature_hash) * (1 << HASH_TABLE_SHIFT);

	pid_va_feature_hash = (struct hlist_head *)kzalloc(size, GFP_KERNEL);
	if (!pid_va_feature_hash) {
		pr_err("gloom [kzalloc_debug] %s alloc failed!\n", __func__);
		return -ENOMEM;
	}

	pr_info("gloom pid_va_feature_hash_init succeed!\n");

	return 0;
}

void pid_va_feature_hash_deinit(void)
{
	kfree(pid_va_feature_hash);
	pr_info("gloom pid_va_feature_hash_deinit succeed!\n");
}

void oplus_gloom_va_feature_add_hash(pid_t pid, struct va_metadata *data)
{
	spin_lock_irq(&pidhash_lock);
	hlist_add_head_rcu(&data->pid_chain,
		&pid_va_feature_hash[pid_va_feature_hashfn(pid, task_active_pid_ns(current))]);
	spin_unlock_irq(&pidhash_lock);
}

struct va_metadata *oplus_gloom_va_feature_search_hash(pid_t pid)
{
	struct va_metadata *data;

	hlist_for_each_entry_rcu(data,
			&pid_va_feature_hash[pid_va_feature_hashfn(pid, task_active_pid_ns(current))],
			pid_chain)
		if (data->nr == pid)
			return data;

	return NULL;
}

void oplus_gloom_va_feature_delete_hash_and_trigger_event(pid_t pid, struct mm_struct *mm)
{
	struct va_metadata *data;
	int flag = 0;

	spin_lock_irq(&pidhash_lock);
	hlist_for_each_entry_rcu(data,
			&pid_va_feature_hash[pid_va_feature_hashfn(pid, task_active_pid_ns(current))],
			pid_chain) {
		if (data->nr != pid)
			continue;

		flag = 1;
		hlist_del_rcu(&data->pid_chain);
		break;
	}
	spin_unlock_irq(&pidhash_lock);

	if (1 == flag)
		trigger_svm_oom_event(data, mm, false, false);

	kfree(data);
	return;
}
