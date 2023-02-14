// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "vmalloc_debug: " fmt

#ifndef _VMALLOC_DEBUG_
#define _VMALLOC_DEBUG_
#include <linux/sort.h>
#include <linux/stacktrace.h>
#include <linux/sched/clock.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <trace/hooks/mm.h>
#include "memleak_debug_stackdepot.h"

/* save vmalloc stack. */
#define VMALLOC_STACK_DEPTH 12
#define VD_VALUE_LEN 32
#define KBUF_LEN 64

static struct proc_dir_entry *spentry;

static ssize_t vmalloc_used_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char kbuf[KBUF_LEN] = {'\0'};
	unsigned int len;

	len = scnprintf(kbuf, KBUF_LEN, "%ld\n", vmalloc_nr_pages() << 2);
	if ((len == KBUF_LEN) && (kbuf[len - 1] != '\n'))
		return -EFAULT;

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buff, kbuf, (len < count ? len : count))) {
		pr_err("vmalloc_debug : copy to user failed.\n");
		return -EFAULT;
	}

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static const struct proc_ops vmalloc_used_fops = {
	.proc_read = vmalloc_used_read,
	.proc_lseek = default_llseek,
};

int __init create_vmalloc_debug(struct proc_dir_entry *parent)
{
	spentry = proc_create("vmalloc_used", S_IRUGO, parent,
			&vmalloc_used_fops);
	if (!spentry) {
		pr_err("create vmalloc_used proc failed.\n");
		return -ENOMEM;
	}

	return 0;
}

void vmalloc_debug_exit(void)
{
	proc_remove(spentry);
}
#endif /* _VMALLOC_DEBUG_ */
