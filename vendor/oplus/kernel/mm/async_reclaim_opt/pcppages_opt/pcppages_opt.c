// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "dstate_opt: " fmt

#include <linux/module.h>
#include <trace/hooks/vmscan.h>
#include <trace/hooks/mm.h>
#include <linux/swap.h>
#include <linux/proc_fs.h>
#include <linux/gfp.h>
#include <linux/types.h>
#include <linux/printk.h>

static void should_drain_all_pages(void *data, gfp_t gfp_mask, unsigned int order, unsigned long alloc_flags,
					int migratetype, unsigned long did_some_progress, bool *bypass)
{
	*bypass = true;
}


static int register_dstate_opt_vendor_hooks(void)
{
	int ret = 0;

	ret = register_trace_android_vh_drain_all_pages_bypass(should_drain_all_pages, NULL);
	if (ret != 0) {
		pr_err("register_dstate_opt_vendor_hooks failed! ret=%d\n", ret);
		goto out;
	}

out:
	return ret;
}

static void unregister_dstate_opt_vendor_hooks(void)
{
	unregister_trace_android_vh_drain_all_pages_bypass(should_drain_all_pages, NULL);

	return;
}

static int __init pcppages_opt_init(void)
{
	int ret = 0;

	ret = register_dstate_opt_vendor_hooks();
	if (ret != 0)
		return ret;

	pr_info("dstate_opt_init succeed!\n");
	return 0;
}

static void __exit pcppages_opt_exit(void)
{
	unregister_dstate_opt_vendor_hooks();

	pr_info("dstate_opt_exit exit succeed!\n");

	return;
}

module_init(pcppages_opt_init);
module_exit(pcppages_opt_exit);

MODULE_LICENSE("GPL v2");
