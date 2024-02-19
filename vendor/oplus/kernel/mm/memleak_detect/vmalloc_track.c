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

static int vmalloc_debug_enable = 0;
static atomic64_t hash_cal_sum_us = ATOMIC64_INIT(0);
static atomic64_t hash_cal_times = ATOMIC64_INIT(0);
static unsigned long hash_cal_max_us;
static struct proc_dir_entry *spentry;
static struct proc_dir_entry *epentry;
static struct proc_dir_entry *tpentry;

extern struct proc_dir_entry *memleak_detect_dir;

#define SET_VM_STACK_HASH(vm, val) (vm->android_oem_data1 = val)
#define GET_VM_STACK_HASH(vm) ((u32)(vm->android_oem_data1))

static void show_stack_hash(void *data, struct seq_file *m,
		struct vm_struct *v)
{
	if (vmalloc_debug_enable)
		seq_printf(m, " hash=%u", GET_VM_STACK_HASH(v));
}

static void save_vmalloc_stack(void *data, unsigned long flags,
		struct vm_struct *vm)
{
	unsigned long entries[VMALLOC_STACK_DEPTH];
#ifndef CONFIG_ARCH_STACKWALK
	struct stack_trace trace = {
		.nr_entries = 0,
		.entries = entries,
		.max_entries = VMALLOC_STACK_DEPTH,
#ifdef CONFIG_64BIT
		.skip = 5
#else
		.skip = 4
#endif
	};
#else
	uint n;
#endif
	ml_depot_stack_handle_t handle;
	unsigned long delay;
	unsigned long start;

	if (!vmalloc_debug_enable)
		return;

	start = sched_clock();
#ifndef CONFIG_ARCH_STACKWALK
	save_stack_trace(&trace);
	if (trace.nr_entries != 0 &&
			trace.entries[trace.nr_entries-1] == ULONG_MAX)
		trace.nr_entries--;
	handle = ml_depot_save_stack(trace.entries, trace.nr_entries, flags);
#else
	n = stack_trace_save(entries, ARRAY_SIZE(entries), 1);
	handle = ml_depot_save_stack(entries, n, flags);
#endif
	if (handle) {
		delay = (sched_clock() - start) / 1000;
		if (delay > hash_cal_max_us)
			hash_cal_max_us = delay;
		atomic64_add(delay, &hash_cal_sum_us);
		atomic64_inc(&hash_cal_times);
		SET_VM_STACK_HASH(vm, handle);
	}
}

static ssize_t vmalloc_debug_enable_write(struct file *file,
		const char __user *buff, size_t len, loff_t *ppos)
{
	char kbuf[VD_VALUE_LEN] = {'0'};
	long val;
	int ret;

	if (!buff || len <= 0)
		return -EINVAL;

	if (len > (VD_VALUE_LEN - 1))
		len = VD_VALUE_LEN - 1;

	if (copy_from_user(&kbuf, buff, len))
		return -EFAULT;
	kbuf[len] = '\0';

	ret = kstrtol(kbuf, 10, &val);
	if (ret)
		return -EINVAL;

	if (val) {
		ret = ml_depot_init(memleak_detect_dir);
		if (ret)
			return ret;
	}
	vmalloc_debug_enable = val;

	return len;
}

#define HASH_VAL_BUFF_LEN (4096 - sizeof(int))
#define HASH_VAL_HASH_LEN ((4096 - sizeof(int))/sizeof(unsigned int))

struct hash_val_info {
	union {
		char buff[HASH_VAL_BUFF_LEN];
		unsigned int hash[HASH_VAL_HASH_LEN];
	};

	int count;
};

static ssize_t vmalloc_debug_enable_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[VD_VALUE_LEN] = {'0'};
	int len;

	len = scnprintf(kbuf, VD_VALUE_LEN, "%d\n", vmalloc_debug_enable);
	if ((len == VD_VALUE_LEN) && (kbuf[len - 1] != '\n'))
		kbuf[len - 1] = '\n';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static ssize_t hash_cal_time_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[128] = {'0'};
	int len;
	unsigned long sum_us = atomic64_read(&hash_cal_sum_us);
	unsigned long times = atomic64_read(&hash_cal_times);

	len = scnprintf(kbuf, 127, "%lu %lu %lu %lu\n",
			sum_us, times, sum_us / times, hash_cal_max_us);
	if (kbuf[len - 1] != '\n')
		kbuf[len++] = '\n';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf + *off, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

int init_vmalloc_debug(void)
{
	int ret;

	ret = ml_depot_init(memleak_detect_dir);
	if (ret) {
		pr_err("init depot failed, oom.\n");
		return ret;
	}

	ret = register_trace_android_vh_save_vmalloc_stack(save_vmalloc_stack,
			NULL);
	if (ret) {
		ml_depot_destory();
		pr_err("register_trace_android_vh_save_vmalloc_stack failed\n");
		return ret;
	}

	ret = register_trace_android_vh_show_stack_hash(show_stack_hash, NULL);
	if (ret) {
		unregister_trace_android_vh_save_vmalloc_stack(save_vmalloc_stack,
					NULL);
		ml_depot_destory();
		pr_err("register_trace_android_vh_show_stack_hash failed\n");
		return ret;
	}

	return 0;
}

void disable_vmalloc_debug(void)
{
	vmalloc_debug_enable = 0;
}

static const struct proc_ops hash_cal_time_ops = {
	.proc_read		= hash_cal_time_read,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops proc_vmalloc_debug_enable_ops = {
	.proc_write		= vmalloc_debug_enable_write,
	.proc_read		= vmalloc_debug_enable_read,
	.proc_lseek		= default_llseek,
};

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

	epentry = proc_create("vmalloc_debug_enable", S_IRUGO|S_IWUGO,
			parent, &proc_vmalloc_debug_enable_ops);
	if (!epentry) {
		pr_err("create vmalloc_debug_enable proc failed.\n");
		proc_remove(spentry);
		return -ENOMEM;
	}

	tpentry = proc_create("vmalloc_hash_cal", S_IRUGO, parent,
			&hash_cal_time_ops);
	if (!tpentry) {
		pr_err("create vmalloc_hash_cal proc failed.\n");
		proc_remove(epentry);
		proc_remove(spentry);
		return -ENOMEM;
	}

	return 0;
}

void vmalloc_debug_exit(void)
{
	proc_remove(tpentry);
	proc_remove(epentry);
	proc_remove(spentry);
	unregister_trace_android_vh_show_stack_hash(show_stack_hash, NULL);
	unregister_trace_android_vh_save_vmalloc_stack(save_vmalloc_stack, NULL);
}
#endif /* _VMALLOC_DEBUG_ */
