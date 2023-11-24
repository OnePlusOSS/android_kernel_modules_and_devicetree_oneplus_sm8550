// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "abort_mm_opt: " fmt

#include <linux/module.h>
#include <trace/hooks/vmscan.h>
#include <trace/hooks/mm.h>
#include <linux/swap.h>
#include <linux/proc_fs.h>
#include <linux/gfp.h>
#include <linux/types.h>
#include <linux/printk.h>

static void should_abort_madvise(void *data, struct vm_area_struct *vma, bool *abort_madvise)
{
	if (unlikely(task_sigpending(current) &&
		sigismember(&current->pending.signal, SIGUSR2)))
		*abort_madvise = true;
}

static void should_abort_compact(void *data, bool *abort_compact)
{
	if (unlikely(task_sigpending(current) &&
		sigismember(&current->pending.signal, SIGUSR2)))
		*abort_compact = true;
}

static int register_abort_mm_opt_vendor_hooks(void)
{
	int ret = 0;

	ret = register_trace_android_vh_madvise_cold_or_pageout_abort(should_abort_madvise, NULL);
	if (ret != 0) {
		pr_err("register_trace_android_vh_madvise_cold_or_pageout_abort failed! ret=%d\n", ret);
		goto out;
	}

	ret = register_trace_android_vh_compact_finished(should_abort_compact, NULL);
	if (ret != 0) {
		unregister_trace_android_vh_madvise_cold_or_pageout_abort(should_abort_madvise, NULL);
		pr_err("register_trace_android_vh_compact_finished failed! ret=%d\n", ret);
		goto out;
	}

out:
	return ret;
}

static void unregister_abort_mm_opt_vendor_hooks(void)
{
	unregister_trace_android_vh_compact_finished(should_abort_compact, NULL);
	unregister_trace_android_vh_madvise_cold_or_pageout_abort(should_abort_madvise, NULL);

	return;
}

static int __init abort_mm_opt_init(void)
{
	int ret = 0;

	ret = register_abort_mm_opt_vendor_hooks();
	if (ret != 0)
		return ret;

	pr_info("abort_mm_opt_init succeed!\n");
	return 0;
}

static void __exit abort_mm_opt_exit(void)
{
	unregister_abort_mm_opt_vendor_hooks();

	pr_info("abort_mm_opt_exit exit succeed!\n");

	return;
}

module_init(abort_mm_opt_init);
module_exit(abort_mm_opt_exit);

MODULE_LICENSE("GPL v2");
