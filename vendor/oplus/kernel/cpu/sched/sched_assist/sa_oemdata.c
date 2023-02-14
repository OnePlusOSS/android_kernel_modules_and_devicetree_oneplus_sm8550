// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */
#include <linux/slab.h>
#include <linux/sched/task.h>
#include <linux/minmax.h>
#include <linux/align.h>
#include <asm/cache.h>
#include <linux/topology.h>
#include <linux/vmalloc.h>
#include <asm/barrier.h>

#include "sa_oemdata.h"
#include "sched_assist.h"
#include "sa_common.h"

struct kmem_cache *oplus_task_struct_cachep;
EXPORT_SYMBOL(oplus_task_struct_cachep);

static inline struct oplus_task_struct *alloc_oplus_task_struct_node(int node)
{
	return kmem_cache_alloc(oplus_task_struct_cachep, GFP_ATOMIC);
}

static inline void free_oplus_task_struct(struct oplus_task_struct *ots)
{
	if (!ots)
		return;

	kmem_cache_free(oplus_task_struct_cachep, ots);
}

/* called from kernel_clone() to get node information for about to be created task */
static int oplus_tsk_fork_get_node(struct task_struct *tsk)
{
	return NUMA_NO_NODE;
}

void android_vh_dup_task_struct_handler(void *unused,
		struct task_struct *tsk, struct task_struct *orig)
{
	int node;
	struct oplus_task_struct *ots = NULL;

	if (!tsk || !orig)
		return;

	/* The required space has been allocated */
	if (!IS_ERR_OR_NULL((void *)tsk->android_oem_data1[OTS_IDX]))
		return;

	node = oplus_tsk_fork_get_node(orig);
	ots = alloc_oplus_task_struct_node(node);
	if (IS_ERR_OR_NULL(ots))
		return;

	ots->task = tsk;
	smp_mb();

	WRITE_ONCE(tsk->android_oem_data1[OTS_IDX], (u64) ots);
}

void android_vh_free_task_handler(void *unused, struct task_struct *tsk)
{
	struct oplus_task_struct *ots = NULL;

	if (!tsk)
		return;

	ots = (struct oplus_task_struct *) READ_ONCE(tsk->android_oem_data1[OTS_IDX]);
	if (IS_ERR_OR_NULL(ots))
		return;

	WRITE_ONCE(tsk->android_oem_data1[OTS_IDX], 0);
	barrier();

	list_del_init(&ots->ux_entry);
	list_del_init(&ots->fbg_list);
	ots->task = NULL;

	free_oplus_task_struct(ots);
}

static int register_oemdata_hooks(void)
{
	int ret = 0;

	REGISTER_TRACE_VH(android_vh_dup_task_struct, android_vh_dup_task_struct_handler);
	REGISTER_TRACE_VH(android_vh_free_task, android_vh_free_task_handler);

	return ret;
}

static void unregister_oemdata_hooks(void)
{
	UNREGISTER_TRACE_VH(android_vh_dup_task_struct, android_vh_dup_task_struct_handler);
	UNREGISTER_TRACE_VH(android_vh_free_task, android_vh_free_task_handler);
}

/*
 * NOTE:
 * Initialize the oplus_task_struct here.
 */
static void init_oplus_task_struct(void *ptr)
{
	struct oplus_task_struct *ots = ptr;

	memset(ots, 0, sizeof(struct oplus_task_struct));

	INIT_LIST_HEAD(&ots->ux_entry);
	INIT_LIST_HEAD(&ots->fbg_list);
}

int sa_oemdata_init(void)
{
	oplus_task_struct_cachep = kmem_cache_create("oplus_task_struct",
			sizeof(struct oplus_task_struct), 0,
			SLAB_PANIC|SLAB_ACCOUNT, init_oplus_task_struct);

	if (!oplus_task_struct_cachep)
		return -ENOMEM;

	register_oemdata_hooks();
	return 0;
}

void __maybe_unused sa_oemdata_deinit(void)
{
	unregister_oemdata_hooks();
	kmem_cache_destroy(oplus_task_struct_cachep);
}

