// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <trace/hooks/mm.h>

#include "gloom_common.h"
#include "reserve_area.h"
#include "va_feature_hash.h"
#include "va_feature_node.h"

static int register_gloom_vendor_hooks(void)
{
	int ret = 0;

	/* register vender hook in kernel/mm/mmap.c */
	REGISTER_TRACE_VH(android_vh_get_from_fragment_pool, get_unmapped_area_from_fragment_pool_handler);
	REGISTER_TRACE_VH(android_vh_exclude_reserved_zone, get_unmapped_area_exclude_reserved_zone_handler);
	REGISTER_TRACE_VH(android_vh_include_reserved_zone, get_unmapped_area_include_reserved_zone_handler);
	REGISTER_TRACE_VH(android_vh_exit_mm, exit_mm_handler);

	pr_info("gloom register_gloom_vendor_hooks succeed!\n"); /* for debug */

	return 0;
}

static void unregister_gloom_vendor_hooks(void)
{
	UNREGISTER_TRACE_VH(android_vh_get_from_fragment_pool, get_unmapped_area_from_fragment_pool_handler);
	UNREGISTER_TRACE_VH(android_vh_exclude_reserved_zone, get_unmapped_area_exclude_reserved_zone_handler);
	UNREGISTER_TRACE_VH(android_vh_include_reserved_zone, get_unmapped_area_include_reserved_zone_handler);
	UNREGISTER_TRACE_VH(android_vh_exit_mm, exit_mm_handler);

	pr_info("gloom unregister_gloom_vendor_hooks succeed!\n"); /* for debug */
}

static int __init oplus_gloom_init(void)
{
	int ret = 0;

	ret = oplus_gloom_proc_init();
	if (ret != 0) {
		return ret;
	}

	ret = pid_va_feature_hash_init();
	if (ret != 0) {
		return ret;
	}

	ret = register_gloom_vendor_hooks();
	if (ret != 0) {
		return ret;
	}

	pr_info("gloom oplus_gloom_init succeed!\n");
	return 0;
}

static void __exit oplus_gloom_exit(void)
{
	unregister_gloom_vendor_hooks();
	oplus_gloom_proc_deinit();
	pid_va_feature_hash_deinit();
	pr_info("gloom oplus_gloom_exit succeed!\n");
}

module_init(oplus_gloom_init);
module_exit(oplus_gloom_exit);
MODULE_DESCRIPTION("Oplus Gloom VA Feature Vendor Hooks Driver");
MODULE_LICENSE("GPL v2");
