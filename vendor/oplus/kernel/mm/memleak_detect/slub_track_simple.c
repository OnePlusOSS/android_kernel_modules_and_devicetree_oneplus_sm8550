// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "kmalloc_debug: " fmt

#ifndef _SLUB_TRACK_
#define _SLUB_TRACK_
#include <linux/sort.h>
#include <linux/jhash.h>
#include <linux/version.h>
#include <linux/swap.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <trace/hooks/mm.h>

#include "slab.h"
#include "memleak_debug_stackdepot.h"

int kmalloc_debug = 1;
int vmalloc_debug = 1;
module_param_named(kmalloc_debug, kmalloc_debug, int, 0444);
module_param_named(vmalloc_debug, vmalloc_debug, int, 0444);

extern int __init create_vmalloc_debug(struct proc_dir_entry *parent);
extern void vmalloc_debug_exit(void);

#define KMALLOC_USED_LEN 4096

/*
 * kmalloc_debug_caches store the kmalloc caches with debug flag.
 */
atomic64_t kmalloc_debug_caches[NR_KMALLOC_TYPES][KMALLOC_SHIFT_HIGH + 1] = {{ATOMIC64_INIT(0)}};

extern unsigned long calculate_kmalloc_slab_size(struct kmem_cache *s);

static struct proc_dir_entry *upentry;
struct proc_dir_entry *memleak_detect_dir;
struct proc_dir_entry *oplus_mem_dir;

static __always_inline unsigned int kmalloc_size(unsigned int n)
{
	if (n > 2)
		return 1U << n;

	if (n == 1 && KMALLOC_MIN_SIZE <= 32)
		return 96;

	if (n == 2 && KMALLOC_MIN_SIZE <= 64)
		return 192;

	return 0;
}

static inline int type_to_num(int type)
{
	switch (type) {
	case KMALLOC_NORMAL:
		return 10;
	case KMALLOC_RECLAIM:
		return 20;
#ifdef CONFIG_ZONE_DMA
	case KMALLOC_DMA:
		return 30;
#endif
	default:
		return -1;
	}

	return -1;
}

unsigned long calculate_kmalloc_slab_size(struct kmem_cache *s)
{
	struct slabinfo sinfo;

	memset(&sinfo, 0, sizeof(sinfo));
	get_slabinfo(s, &sinfo);
	return sinfo.num_objs * s->object_size;
}

static ssize_t kmalloc_used_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char *kbuf = NULL;
	int len = 0;
	int i, type, ret;

	kbuf = vzalloc(KMALLOC_USED_LEN);
	if (!kbuf)
		return -ENOMEM;

	for (i = 0; i <= KMALLOC_SHIFT_HIGH; i++) {
		for(type = KMALLOC_NORMAL; type < NR_KMALLOC_TYPES; type++) {
			struct kmem_cache *s;
			unsigned long slab_size = 0;

#ifdef CONFIG_MEMCG_KMEM
			if (type == KMALLOC_CGROUP)
				continue;
#endif

			s = kmalloc_caches[type][i];
			if (s)
				slab_size = calculate_kmalloc_slab_size(s);

			s = (struct kmem_cache *)atomic64_read(&kmalloc_debug_caches[type][i]);
			if (s)
				slab_size += calculate_kmalloc_slab_size(s);

			if (slab_size != 0)
				len += scnprintf(&kbuf[len],
						KMALLOC_USED_LEN - 1 - len, "%d%-19d %lu\n",
						type_to_num(type),
						kmalloc_size(i),
						slab_size >> 10);

			if (len >= KMALLOC_USED_LEN - 1)
				break;
		}
	}

	if ((len > 0) && (kbuf[len - 1] != '\n'))
		kbuf[len++] = '\n';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	ret = copy_to_user(buffer, kbuf + *off, (len < count ? len : count));
	vfree(kbuf);
	if (ret)
		return -EFAULT;
	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static const struct proc_ops kmalloc_used_ops = {
	.proc_read	= kmalloc_used_read,
	.proc_lseek	= default_llseek,
};

int __init create_kmalloc_debug(struct proc_dir_entry *parent)
{
	upentry = proc_create("kmalloc_used", S_IRUGO, parent,
			&kmalloc_used_ops);
	if (!upentry) {
		pr_err("create kmalloc_used proc failed.\n");
		goto remove_epentry;
	}
	return 0;

remove_epentry:
	return -ENOMEM;
}

void destroy_kmalloc_debug(void)
{
	proc_remove(upentry);
}

static int __init memleak_detect_init(void)
{
	int ret;

	oplus_mem_dir = proc_mkdir("oplus_mem", NULL);
	if (!oplus_mem_dir)
		memleak_detect_dir = proc_mkdir("oplus_mem/memleak_detect", NULL);
	else
		memleak_detect_dir = proc_mkdir("memleak_detect", oplus_mem_dir);
	if (!memleak_detect_dir) {
		pr_err("create memleak_detect proc failed.\n");
		return -ENOMEM;
	}

	if (kmalloc_debug) {
		ret = create_kmalloc_debug(memleak_detect_dir);
		if (ret) {
			pr_err("create_kmalloc_debug failed\n");
			goto fail_out;
		}
	}

	if (vmalloc_debug) {
		ret = create_vmalloc_debug(memleak_detect_dir);
		if (ret) {
			pr_err("create_vmalloc_debug failed\n");
			goto destroy_kmalloc_debug;
		}
	}

	return 0;

destroy_kmalloc_debug:
	if (kmalloc_debug)
		destroy_kmalloc_debug();
fail_out:
	proc_remove(memleak_detect_dir);
	return ret;
}

static void memleak_detect_exit(void)
{
	if (kmalloc_debug)
		destroy_kmalloc_debug();

	if (vmalloc_debug) {
		vmalloc_debug_exit();
	}

	proc_remove(memleak_detect_dir);
}

module_init(memleak_detect_init);
module_exit(memleak_detect_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("kmalloc vmalloc memleak debugging feature");
MODULE_IMPORT_NS(MINIDUMP);
#endif /* _SLUB_TRACK_ */
