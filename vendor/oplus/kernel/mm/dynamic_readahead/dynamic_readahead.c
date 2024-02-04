// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "dynamic_readahead: " fmt

#include <linux/module.h>
#include <trace/hooks/vmscan.h>
#include <trace/hooks/mm.h>
#include <linux/swap.h>
#include <linux/proc_fs.h>
#include <linux/gfp.h>
#include <linux/types.h>
#include <linux/printk.h>
#include <linux/cgroup.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/vmstat.h>
#include <linux/oom.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <uapi/linux/sched/types.h>
#include <linux/cpufreq.h>
#include <linux/mm.h>
#include <linux/mm_inline.h>
#include <linux/pagemap.h>
#include <linux/page-flags.h>
#include <linux/pageblock-flags.h>
#include <linux/debugfs.h>
#include <linux/memcontrol.h>
#include <linux/mm_types.h>
#include <linux/kasan.h>
#include <linux/page_owner.h>
#include <linux/page_ref.h>
#include <linux/mmzone.h>
#include <linux/sched/rt.h>
#include "../../mm/internal.h"

#include <../../cpu/sched/sched_assist/sa_common.h>
#include <../../cpu/sched/sched_info/osi_healthinfo.h>


static unsigned long long high_wm = 0;
/* true by default, false when oplus_bsp_dynamic_readahead.enable=N in cmdline */
bool enable = true;
module_param(enable, bool, S_IRUGO | S_IWUSR);

struct pglist_data *first_online_pgdat(void)
{
	return NODE_DATA(first_online_node);
}

struct pglist_data *next_online_pgdat(struct pglist_data *pgdat)
{
	int nid = next_online_node(pgdat->node_id);

	if (nid == MAX_NUMNODES)
		return NULL;
	return NODE_DATA(nid);
}

struct zone *next_zone(struct zone *zone)
{
	pg_data_t *pgdat = zone->zone_pgdat;

	if (zone < pgdat->node_zones + MAX_NR_ZONES - 1)
		zone++;
	else {
		pgdat = next_online_pgdat(pgdat);
		if (pgdat)
			zone = pgdat->node_zones;
		else
			zone = NULL;
	}
	return zone;
}

inline int task_is_fg(struct task_struct *tsk)
{
	int cur_uid;

	cur_uid = task_uid(tsk).val;
	if (is_fg(cur_uid))
		return 1;
	return 0;
}

static bool is_key_task(struct task_struct *tsk)
{
	return (
		test_task_ux(tsk) ||
		task_is_fg(tsk) ||
		rt_task(tsk));
}

static inline bool is_lowmem(void)
{
	return global_zone_page_state(NR_FREE_PAGES) < high_wm;
}

static void adjust_readaround(void *data, unsigned int ra_pages, pgoff_t offset,
		pgoff_t *start, unsigned int *size, unsigned int *async_size)
{
	if (is_key_task(current))
		return;

	if (is_lowmem()) {
		ra_pages /= 2;
		*start = max_t(long, 0, offset - ra_pages / 2);
		*size = ra_pages;
		*async_size = ra_pages / 4;
	}
}

static void adjust_readahead(void *data, struct readahead_control *ractl, unsigned long *max_pages)
{
	struct file_ra_state *ra = &ractl->file->f_ra;

	if (is_key_task(current))
		return;

	if (is_lowmem())
		*max_pages = min_t(long, *max_pages, ra->ra_pages / 2);
}

static int __init dynamic_readahead_init(void)
{
	int ret = 0;
	struct zone *zone = NULL;

	if (!enable) {
		pr_err("oplus_bsp_dynamic_readahead is disabled in cmdline\n");
		return -EINVAL;
	}

	for_each_zone(zone) {
		high_wm += high_wmark_pages(zone);
	}

	ret = register_trace_android_vh_tune_mmap_readaround(adjust_readaround, NULL);
	if (ret != 0) {
		pr_err("register_trace_android_vh_tune_mmap_readaround failed! ret=%d\n", ret);
		goto out;
	}

	ret = register_trace_android_vh_ra_tuning_max_page(adjust_readahead, NULL);
	if (ret != 0) {
		pr_err("register_trace_android_vh_ra_tuning_max_page failed! ret=%d\n", ret);
		goto out;
	}

	pr_info("dynamic_readahead_init succeed!\n");
out:
	return ret;
}

static void __exit dynamic_readahead_exit(void)
{
	unregister_trace_android_vh_ra_tuning_max_page(adjust_readahead, NULL);
	unregister_trace_android_vh_tune_mmap_readaround(adjust_readaround, NULL);
	pr_info("dynamic_readahead_exit succeed!\n");
}

module_init(dynamic_readahead_init);
module_exit(dynamic_readahead_exit);

MODULE_LICENSE("GPL v2");

