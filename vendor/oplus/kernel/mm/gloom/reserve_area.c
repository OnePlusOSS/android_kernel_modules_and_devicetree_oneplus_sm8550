// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/sched/signal.h>
#include <linux/security.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/mman.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/seq_file.h>
#include <linux/highmem.h>
#include <linux/android_debug_symbols.h>

#include "va_feature_hash.h"
#include "reserve_area.h"
#include "kernel/oplus_cpu/sched/healthinfo/healthinfo.h"

#define STACK_RLIMIT_OVERFFLOW		(32<<20)
#define THRIDPART_APP_UID_LOW_LIMIT	10000UL
#define NS_PER_SEC					(1000000000LLU)
#define TRIGGER_TIME_NS				(300*NS_PER_SEC)

int svm_oom_pid = -1;
unsigned long svm_oom_jiffies = 0;
static int va_feature = RESERVE_FEATURES;

int get_va_feature_value(unsigned int feature)
{
	return va_feature & (feature & ~RESERVE_LOGGING);
}

void get_unmapped_area_from_fragment_pool_handler(void *data, struct mm_struct *mm,
				struct vm_unmapped_area_info *info, unsigned long *addr)
{
	struct va_metadata *va_metadata_tmp = NULL;
	pid_t pid = current->group_leader->pid;

	/* search in the pid hash table to find metadata */
	va_metadata_tmp = oplus_gloom_va_feature_search_hash(pid);

	if (va_metadata_tmp) {
		if ((va_metadata_tmp->va_feature & ANTI_FRAGMENT_AREA) &&
			(info->high_limit == mm->mmap_base)) {
			struct vm_unmapped_area_info info_b;
			unsigned long address = 0;

			switch (info->length) {
			case 4096: case 8192: case 16384: case 32768:
			case 65536: case 131072: case 262144:
				info_b = *info;
				info_b.high_limit = va_metadata_tmp->va_feature_rnd - (SIZE_10M * (ilog2(info->length) - 12));
				info_b.low_limit = va_metadata_tmp->va_feature_rnd - (SIZE_10M * 7);
				info_b.flags = VM_UNMAPPED_AREA_TOPDOWN;
				/* address = unmapped_area_topdown(&info_b); in kernel-5.10, this function is static */
				address = vm_unmapped_area(&info_b);
				if (!offset_in_page(address)) {
					*addr = address;
					return;
				}
			default:
				break;
			}
			*addr = 0;
		}
	} else
		*addr = 0;
}

void get_unmapped_area_exclude_reserved_zone_handler(void *data, struct mm_struct *mm,
				struct vm_unmapped_area_info *info)
{
	struct va_metadata *va_metadata_tmp = NULL;
	pid_t pid = current->group_leader->pid;

	/* search in the pid hash table to find metadata */
	va_metadata_tmp = oplus_gloom_va_feature_search_hash(pid);

	if (va_metadata_tmp) {
		if (va_metadata_tmp->va_feature & RESERVE_AREA)
			info->low_limit = max_t(unsigned long, GET_UNMMAPED_AREA_FIRST_LOW_LIMIT, info->low_limit);
	}
}

void get_unmapped_area_include_reserved_zone_handler(void *data, struct mm_struct *mm,
				struct vm_unmapped_area_info *info, unsigned long *addr)
{
	struct va_metadata *va_metadata_tmp = NULL;
	pid_t pid = current->group_leader->pid;

	/* search in the pid hash table to find metadata */
	va_metadata_tmp = oplus_gloom_va_feature_search_hash(pid);

	if (va_metadata_tmp) {
		if ((va_metadata_tmp->va_feature & RESERVE_AREA) && offset_in_page(*addr)) {
			info->flags = VM_UNMAPPED_AREA_TOPDOWN;
			info->low_limit = max(PAGE_SIZE, *(unsigned long *)android_debug_symbol(ADS_MMAP_MIN_ADDR));
			info->high_limit = mm->mmap_base;
			*addr = vm_unmapped_area(info);
		}
	}
}

/* delete the hash item when process exit, add vendor hook in exit_mm() */
void exit_mm_handler(void *data, struct mm_struct *mm)
{
	if (current->pid == current->tgid)
		oplus_gloom_va_feature_delete_hash_and_trigger_event(current->tgid, mm);

	return;
}

static inline bool check_parent_is_zygote(struct task_struct *tsk)
{
	struct task_struct *t;
	bool ret = false;

	rcu_read_lock();
	t = rcu_dereference(tsk->real_parent);
	if (t) {
		const struct cred *tcred = __task_cred(t);

		if (!strcmp(t->comm, "main") && (tcred->uid.val == 0) &&
				(t->parent != NULL) && !strcmp(t->parent->comm, "init"))
			ret = true;
	}
	rcu_read_unlock();
	return ret;
}

static ssize_t va_feature_write(struct file *file,
		const char __user *buff, size_t len, loff_t *ppos)
{
	char kbuf[12] = {'0'};
	long val;
	int ret;

	if (len > 11)
		len = 11;

	if (copy_from_user(&kbuf, buff, len))
		return -EFAULT;
	kbuf[len] = '\0';

	ret = kstrtol(kbuf, 16, &val);
	if (ret)
		return -EINVAL;

	va_feature = val & RESERVE_FEATURES;
	return len;
}

static ssize_t va_feature_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[12] = {'0'};
	int len;

	len = snprintf(kbuf, 12, "%#x\n", va_feature);
	if (kbuf[len] != '\n')
		kbuf[len] = '\n';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf + *off, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static const struct proc_ops proc_va_feature_ops = {
	.proc_read		= va_feature_read,
	.proc_write		= va_feature_write,
	.proc_lseek		= default_llseek,
};

int create_reserved_area_enable_proc(struct proc_dir_entry *parent)
{
	if (parent && proc_create("va_feature", S_IRUSR|S_IWUSR,
				parent, &proc_va_feature_ops)) {
		pr_info("gloom register va_feature interface succeed!\n");
		return 0;
	}
	pr_err("gloom register va_feature interface failed!\n");
	return -ENOMEM;
}
EXPORT_SYMBOL(create_reserved_area_enable_proc);

#ifdef CONFIG_OPLUS_FEATURE_HEALTHINFO
void trigger_svm_oom_event(struct va_metadata *data, struct mm_struct *mm,
				bool brk_risk, bool is_locked)
{
	int len = 0;
	int oom = 0;
	int res = 0;
	int over_time = 0;
	int change_stack = 0;
	struct rlimit *rlim;
	unsigned long long current_time_ns;
	char *svm_oom_msg = NULL;
	unsigned int uid = (unsigned int)(current_uid().val);

	if (!((va_feature & RESERVE_LOGGING) &&
				(current->pid == current->tgid) &&
				is_compat_task() &&
				check_parent_is_zygote(current) &&
				(uid >= THRIDPART_APP_UID_LOW_LIMIT)))
		return;

	svm_oom_msg = (char*)kmalloc(128, GFP_KERNEL);
	if (!svm_oom_msg)
		return;

	if (is_locked) {
		if (data->va_feature & RESERVE_AREA)
			res = 1;
	} else {
		down_read(&mm->mmap_lock);
		if (data->va_feature & RESERVE_AREA)
			res = 1;
		up_read(&mm->mmap_lock);
	}

	if ((svm_oom_pid == current->pid) &&
			time_after_eq((svm_oom_jiffies + 15*HZ), jiffies)) {
		svm_oom_pid = -1;
		oom = 1;
	}
	rlim = current->signal->rlim + RLIMIT_STACK;
	if (rlim->rlim_cur > STACK_RLIMIT_OVERFFLOW || brk_risk)
		change_stack = 1;

	if (change_stack) {
		len = snprintf(svm_oom_msg, 127,
				"{\"version\":1, \"size\":%ld, \"uid\":%u, \"type\":\"%s,%s,%s\"}",
				(long)rlim->rlim_cur, uid,
				(oom ? "oom" : "no_oom"),
				(res ? "res" : "no_res"),
				(brk_risk ? "brk" : "no_brk"));
		svm_oom_msg[len] = '\0';
		ohm_action_trig_with_msg(OHM_RLIMIT_MON, svm_oom_msg);
		kfree(svm_oom_msg);
		return;
	}

	current_time_ns = ktime_to_ns(ktime_get_boottime());

	if ((current_time_ns > current->start_boottime) ||
			(current_time_ns - current->start_boottime >= TRIGGER_TIME_NS))
		over_time = 1;

	if (oom || (!change_stack && !res && over_time)) {
		len = snprintf(svm_oom_msg, 127,
				"{\"version\":1, \"size\":%lu, \"uid\":%u, \"type\":\"%s,%s,%s\"}",
				0, uid,
				(oom ? "oom" : "no_oom"),
				(res ? "res" : "no_res"),
				(change_stack ? "stack" : "no_stack"));
		svm_oom_msg[len] = '\0';
		ohm_action_trig_with_msg(OHM_SVM_MON, svm_oom_msg);
	}
	kfree(svm_oom_msg);
}
#else
#error [gloom] CONFIG_OPLUS_HEALTHINFO is not enabled.
#endif

static int fetch_vma_name(struct vm_area_struct *vma, char *kbuf, int klen)
{
	const char __user *name = vma_get_anon_name(vma);
	struct mm_struct *mm = vma->vm_mm;

	unsigned long page_start_vaddr;
	unsigned long page_offset;
	unsigned long num_pages;
	unsigned long max_len = klen - 16;
	int i;
	int cnt = 0;

	page_start_vaddr = (unsigned long)name & PAGE_MASK;
	page_offset = (unsigned long)name - page_start_vaddr;
	num_pages = DIV_ROUND_UP(page_offset + max_len, PAGE_SIZE);

	cnt += snprintf(kbuf, klen, "[anon:");

	for (i = 0; i < num_pages; i++) {
		int len;
		int write_len;
		const char *kaddr;
		long pages_pinned;
		struct page *page;

		pages_pinned = get_user_pages_remote(mm,
				page_start_vaddr, 1, 0, &page, NULL, NULL);
		if (pages_pinned < 1) {
			cnt += snprintf(kbuf + cnt, klen - cnt, "<fault>]");
			return cnt;
		}

		kaddr = (const char *)kmap(page);
		len = min(max_len, PAGE_SIZE - page_offset);
		write_len = strnlen(kaddr + page_offset, len);
		memcpy(kbuf + cnt, kaddr + page_offset, write_len);
		kunmap(page);
		put_page(page);
		cnt += write_len;

		/* if strnlen hit a null terminator then we're done */
		if (write_len != len)
			break;

		max_len -= len;
		if ((int)max_len <= 0)
			break;
		page_offset = 0;
		page_start_vaddr += PAGE_SIZE;
	}

	cnt += snprintf(kbuf + cnt, klen - cnt, "]");
	return cnt;
}

static void dump_one_vma(struct vm_area_struct *vma)
{
	struct mm_struct *mm = vma->vm_mm;
	struct file *file = vma->vm_file;
	vm_flags_t flags = vma->vm_flags;
	unsigned long ino = 0;
	unsigned long long pgoff = 0;
	unsigned long start, end;
	dev_t dev = 0;
	char vma_name[256] = {'\0'};
	const char *name = NULL;
	int len;

	start = vma->vm_start;
	end = vma->vm_end;

	if (file) {
		struct inode *inode = file_inode(vma->vm_file);

		dev = inode->i_sb->s_dev;
		ino = inode->i_ino;
		pgoff = ((loff_t)vma->vm_pgoff) << PAGE_SHIFT;
		name = d_path(&file->f_path, vma_name, 255);

		if (IS_ERR(name))
			name = NULL;
		goto done;
	}

	if (vma->vm_ops && vma->vm_ops->name) {
		name = vma->vm_ops->name(vma);
		if (name)
			goto done;
	}

	if (!mm) {
		name = "[vdso]";
		goto done;
	}

	if (start <= mm->brk && end >= mm->start_brk) {
		name = "[heap]";
		goto done;
	}

	/* whether the vma is stack */
	if (start <= mm->start_stack && end >= mm->start_stack) {
		name = "[stack]";
		goto done;
	}

	if (vma_get_anon_name(vma)) {
		len = fetch_vma_name(vma, vma_name, 256);
		if (len) {
			name = vma_name;
			if ((len == 256) && (vma_name[len-1] != '\0'))
				vma_name[len-1] = '\0';
		} else if (len == 0)
			name = "";
	}
done:
	pr_err("dbgmm %08lx-%08lx %c%c%c%c %08llx %02u:%02u %ld %s",
			start, end,
			flags & VM_READ ? 'r' : '-',
			flags & VM_WRITE ? 'w' : '-',
			flags & VM_EXEC ? 'x' : '-',
			flags & VM_MAYSHARE ? 's' : 'p',
			pgoff, MAJOR(dev), MINOR(dev), ino,
			name ?: "");
}

static void dump_mm_info(unsigned long len, unsigned long flags, int dump_vma)
{
	struct mm_struct *mm = current->mm;
	unsigned long swap, anon, file, shmem;
	unsigned long total_vm, total_rss;
	struct vm_area_struct *vma;
	unsigned long chunk, chunk_start;
	unsigned long used_size, lastend;
	int dump_num = 0;

	anon = get_mm_counter(mm, MM_ANONPAGES);
	file = get_mm_counter(mm, MM_FILEPAGES);
	shmem = get_mm_counter(mm, MM_SHMEMPAGES);
	swap = get_mm_counter(mm, MM_SWAPENTS);
	total_vm = mm->total_vm;
	total_rss = anon + file + shmem;

	lastend = 0;
	chunk = 0;
	chunk_start = 0;
	used_size = 0;

	for (vma = mm->mmap; vma; vma = vma->vm_next) {
		if ((vma->vm_start - lastend) >	chunk) {
			chunk = vma->vm_start - lastend;
			chunk_start = lastend;
		}
		lastend = vma->vm_end;
		used_size += vma->vm_end - vma->vm_start;
		if (dump_vma) {
			dump_one_vma(vma);
			dump_num++;
		}
	}

	if ((TASK_SIZE - lastend) > chunk) {
		chunk = TASK_SIZE - lastend;
		chunk_start = lastend;
	}

	pr_err("dbgmm task %s %d parent %s %d VmSize %luKB VmRSS %luKB "\
			"RssAnon %luKB RssFile %luKB RssShmem %luKB chunk %lu "\
			"chunk_start 0x%lx used_size %lu map_count %d len %lu "\
			"dump_num %d va_feature 0x%x flags %#lx\n",
			current->comm, current->pid,
			current->group_leader->comm,
			current->group_leader->pid,
			total_vm << (PAGE_SHIFT-10),
			total_rss << (PAGE_SHIFT-10),
			anon << (PAGE_SHIFT-10),
			file << (PAGE_SHIFT-10),
			shmem << (PAGE_SHIFT-10),
			chunk, chunk_start,
			used_size >> 10, mm->map_count, len, dump_num,
			va_feature, flags);
}

void update_oom_pid_and_time(unsigned long len, unsigned long val,
		unsigned long flags)
{
	static DEFINE_RATELIMIT_STATE(svm_log_limit, 1*HZ, 1);
	static unsigned long last_dump_jiffies = 0;
	static int dump_times;
	int dump = 0;

	if (!IS_ERR_VALUE(val))
		return;

	if (svm_oom_pid == current->tgid) {
		if (dump_times >= 5)
			dump = 0;
		else if (time_before(jiffies, last_dump_jiffies + (12 * HZ)))
			dump = 0;
		else
			dump = 1;
	} else {
		dump_times = 0;
		dump = 1;
	}

	if (__ratelimit(&svm_log_limit)) {
		svm_oom_pid = current->tgid;
		svm_oom_jiffies = jiffies;
	}

	if (dump) {
		dump_times++;
		last_dump_jiffies = jiffies;
		dump_mm_info(len, flags, 1);
	} else
		dump_mm_info(len, flags, 0);
}
EXPORT_SYMBOL_GPL(update_oom_pid_and_time);
