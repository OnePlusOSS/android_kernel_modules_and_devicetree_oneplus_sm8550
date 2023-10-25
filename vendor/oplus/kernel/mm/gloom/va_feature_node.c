// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/seq_file.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/random.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include "va_feature_hash.h"
#include "reserve_area.h"

static int pid_s;

/* cat /proc/oplus_mem/gloom_va_feature_debug */
static ssize_t proc_va_feature_debug_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int pid;
	int ret = -EINVAL;
	char buffer[32] = {'\0'};
	struct task_struct* tsk = NULL;
	struct va_metadata *va_metadata_p = NULL;

	pid = pid_s;
	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (!tsk) {
		rcu_read_unlock();
		pr_err("gloom debug : can not find task of pid %d\n", pid);
		goto fail;
	}
	rcu_read_unlock();

	pr_info("gloom proc_va_feature_debug_read tsk->tgid:%d, tsk->pid:%d\n", tsk->tgid, tsk->pid);

	va_metadata_p = oplus_gloom_va_feature_search_hash(pid);
	if (va_metadata_p) {
		ret = snprintf(buffer, sizeof(buffer), "%d:%d:%#lx\n", va_metadata_p->nr,
				va_metadata_p->va_feature, va_metadata_p->va_feature_rnd);
		pr_info("gloom proc_va_feature_debug_read %d:%d:%#lx, ret = %d\n", va_metadata_p->nr, va_metadata_p->va_feature, va_metadata_p->va_feature_rnd, ret);
		if (ret > 0) {
			ret = simple_read_from_buffer(buf, count, ppos, buffer, ret);
			pr_info("gloom proc_va_feature_debug_read [%s], ret = %d\n", buffer, ret);
			return ret;
		} else {
			pr_err("gloom proc_va_feature_debug_read, ret = %d\n", ret);
			goto fail;
		}
	} else {
		pr_err("gloom proc_va_feature_debug_read, search hash pid:%d failed!\n", pid);
		goto fail;
	}

fail:
	pr_err("gloom debug : cat /proc/oplus_mem/gloom_va_feature_debug failed!\n");
	return ret;
}

/* echo pid > /proc/oplus_mem/gloom_va_feature_debug */
static ssize_t proc_va_feature_debug_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	int pid;
	int ret = -EINVAL;
	struct task_struct* tsk = NULL;

	ret = kstrtoint_from_user(buf, count, 0, &pid);
	if (ret)
		goto fail;

	pr_info("gloom proc_va_feature_debug_write pid = %d\n", pid);

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (!tsk) {
		rcu_read_unlock();
		pr_err("gloom debug : can not find task of pid %d\n", pid);
		ret = -ESRCH;
		goto fail;
	}
	rcu_read_unlock();
	pid_s = pid;
	pr_info("gloom debug : echo pid:%d > /proc/oplus_mem/gloom_va_feature_debug succeed!\n", pid);
	return count;
fail:
	pr_err("gloom debug : echo pid:%d > /proc/oplus_mem/gloom_va_feature_debug failed!\n", pid);
	return ret;
}

static ssize_t proc_va_feature_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	pid_t pid;
	int ret = -EINVAL;
	char buffer[32] = {'\0'};
	unsigned int heapsize = 0;
	struct va_metadata *va_metadata_tmp = NULL;

	pid = current->group_leader->pid;

	/* search in the pid hash table to find metadata */
	va_metadata_tmp = oplus_gloom_va_feature_search_hash(pid);
	if (va_metadata_tmp) {
		if (va_metadata_tmp->va_feature & ZYGOTE_HEAP)
			heapsize = (va_metadata_tmp->zygoteheap_in_mb > ZYGOTE_HEAP_DEFAULT_SIZE) ? \
						va_metadata_tmp->zygoteheap_in_mb : ZYGOTE_HEAP_DEFAULT_SIZE;

		ret = snprintf(buffer, sizeof(buffer), "%#x",
						(heapsize & HEAP_SIZE_MASK) + va_metadata_tmp->va_feature);
		if (ret > 0)
			ret = simple_read_from_buffer(buf, count, ppos, buffer, ret);
	}

	return ret;
}

static ssize_t proc_va_feature_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	pid_t pid;
	int ret = -EINVAL;
	unsigned int value;
	unsigned int heapsize;
	struct mm_struct *mm = NULL;
	struct va_metadata *va_metadata_p = NULL;

	ret = kstrtouint_from_user(buf, count, 16, &value);
	if (ret)
		return ret;
	heapsize = value & HEAP_SIZE_MASK;

	if (!test_thread_flag(TIF_32BIT))
		return -ENOTTY;

	pid = current->group_leader->pid;
	va_metadata_p = (struct va_metadata *)kzalloc(sizeof(struct va_metadata), GFP_KERNEL);
	if (!va_metadata_p) {
		pr_err("gloom [kzalloc_debug] %s alloc failed!\n", __func__);
		return -ENOMEM;
	}
	va_metadata_p->nr = pid;
	va_metadata_p->va_feature = get_va_feature_value(value & ~HEAP_SIZE_MASK);

	if ((va_metadata_p->va_feature & ZYGOTE_HEAP) &&
			(va_metadata_p->zygoteheap_in_mb == 0))
			va_metadata_p->zygoteheap_in_mb = heapsize;

	mm = get_task_mm(current);
	if (mm) {
		unsigned long old_mmap_base = mm->mmap_base;

		if (va_metadata_p->va_feature & ANTI_FRAGMENT_AREA) {
			va_metadata_p->va_feature_rnd = (ANTI_FRAGMENT_AREA_BASE_SIZE +
							(get_random_long() % ANTI_FRAGMENT_AREA_MASK));
			va_metadata_p->va_feature_rnd &= ANTI_FRAGMENT_AREA_ALIGN;

			special_arch_pick_mmap_layout(mm);

			pr_info("%s (%d): rnd val is 0x%llx, mmap_base 0x%llx -> 0x%llx\n",
					current->group_leader->comm, current->pid,
					va_metadata_p->va_feature_rnd, old_mmap_base, mm->mmap_base);
		}
		mmput(mm);
	}

	/* add in the pid hash table to record metadata */
	oplus_gloom_va_feature_add_hash(pid, va_metadata_p);

	return count;
}

static const struct proc_ops proc_va_feature_operations = {
	.proc_read		= proc_va_feature_read,
	.proc_write		= proc_va_feature_write,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops proc_va_feature_debug_operations = {
	.proc_read		= proc_va_feature_debug_read,
	.proc_write		= proc_va_feature_debug_write,
	.proc_lseek		= default_llseek,
};

static struct proc_dir_entry *root_dir_entry;
static struct proc_dir_entry *gloom_va_feature;
static struct proc_dir_entry *gloom_va_feature_debug;

int __init oplus_gloom_proc_init(void)
{
	root_dir_entry = proc_mkdir("oplus_mem", NULL);
	if (!root_dir_entry) {
		/* dir /proc/oplus_mem already exist Or mkdir fail */
		gloom_va_feature = proc_create("oplus_mem/gloom_va_feature",
						0666, NULL, &proc_va_feature_operations);
		if (!gloom_va_feature) {
			pr_err("gloom failed to create proc node gloom_va_feature\n");
			return -ENOMEM;
		}

		gloom_va_feature_debug = proc_create("oplus_mem/gloom_va_feature_debug",
						0666, NULL, &proc_va_feature_debug_operations);
		if (!gloom_va_feature_debug) {
			pr_err("gloom failed to create proc node gloom_va_feature_debug\n");
			return -ENOMEM;
		}
	} else {
		/* mkdir /proc/oplus_mem success */
		gloom_va_feature = proc_create("gloom_va_feature", 0666,
						root_dir_entry, &proc_va_feature_operations);
		if (!gloom_va_feature) {
			pr_err("gloom failed to create proc node gloom_va_feature\n");
			return -ENOMEM;
		}

		gloom_va_feature_debug = proc_create("gloom_va_feature_debug", 0666,
						root_dir_entry, &proc_va_feature_debug_operations);
		if (!gloom_va_feature_debug) {
			pr_err("gloom failed to create proc node gloom_va_feature_debug\n");
			return -ENOMEM;
		}
	}

	pr_info("gloom oplus_gloom_proc_init succeed!\n");

	return 0;
}

void oplus_gloom_proc_deinit(void)
{
	proc_remove(gloom_va_feature);
	proc_remove(gloom_va_feature_debug);
	pr_info("gloom oplus_gloom_proc_deinit succeed!\n");
}
