// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2023-2030 Oplus. All rights reserved.
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/sched/mm.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/version.h>
#include <linux/sched/clock.h>
#include <linux/bits.h>
#include <linux/highmem.h>
#include <linux/compiler.h>
#include <asm/stacktrace.h>
#include <asm/memory.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
#include <trace/hooks/sysrqcrash.h>
#endif

#define OPLUS_PATH_NAME_LEN       256
#define OPLUS_HEAD_INFO_LEN       128
#define OPLUS_BT_MAX_DEPTH        16
#define PRTINT_LOG     printk_deferred

/* #define DEBUG_ESCAPE_TIME         1 */

char *anon_name_array[] = {
	"[null]",
	"[anon]",
	"[heap]",
	"[stack]",
	"[vdso]"
};

enum anon_name_type {
	ANON_NAME_NULL = 0,
	ANON_NAME_ANON,
	ANON_NAME_HEAP,
	ANON_NAME_STACK,
	ANON_NAME_VDSO
};

static int __access_remote_vm_no_lock(struct mm_struct *mm, unsigned long addr,
				void *buf, int len, unsigned int gup_flags)
{
	struct vm_area_struct *vma;
	void *old_buf = buf;
	int write = gup_flags & FOLL_WRITE;
/*
	if (mmap_read_lock_killable(mm))
		return 0;
*/

	/* ignore errors, just check how much was successfully transferred */
	while (len) {
		int bytes, ret, offset;
		void *maddr;
		struct page *page = NULL;

		ret = get_user_pages_remote(mm, addr, 1,
				gup_flags, &page, &vma, NULL);
		if (ret <= 0) {
#ifndef CONFIG_HAVE_IOREMAP_PROT
			break;
#else
			/*
			 * Check if this is a VM_IO | VM_PFNMAP VMA, which
			 * we can access using slightly different code.
			 */
			vma = vma_lookup(mm, addr);
			if (!vma)
				break;
			if (vma->vm_ops && vma->vm_ops->access)
				ret = vma->vm_ops->access(vma, addr, buf,
							  len, write);
			if (ret <= 0)
				break;
			bytes = ret;
#endif
		} else {
			bytes = len;
			offset = addr & (PAGE_SIZE-1);
			if (bytes > PAGE_SIZE-offset)
				bytes = PAGE_SIZE-offset;

			maddr = kmap(page);
			if (write) {
				copy_to_user_page(vma, page, addr,
						  maddr + offset, buf, bytes);
				set_page_dirty_lock(page);
			} else {
				copy_from_user_page(vma, page, addr,
						    buf, maddr + offset, bytes);
			}
			kunmap(page);
			put_page(page);
		}
		len -= bytes;
		buf += bytes;
		addr += bytes;
	}
/*
	mmap_read_unlock(mm);
*/
	return buf - old_buf;
}

static int access_process_vm_ubt(struct task_struct *tsk, unsigned long addr,
				void *buf, int len, unsigned int gup_flags)
{
	struct mm_struct *mm;
	int ret;

	mm = get_task_mm(tsk);
	if (!mm)
		return 0;

	ret = __access_remote_vm_no_lock(mm, addr, buf, len, gup_flags);
	mmput(mm);
	return ret;
}

static void dump_frames(unsigned long *stack_frames, int frames,
			struct task_struct *task, char *tpath, int size)
{
	struct vm_area_struct *vma;
	struct file *file;
	struct mm_struct *mm;
	struct path base_path;
	unsigned long long pgoff = 0;
	unsigned long current_pc = 0;
	int current_frame = 0;
	const char *path_p = NULL;

	while (current_frame < frames) {
		current_pc = stack_frames[current_frame];

		if (!current_pc) {
			current_frame++;
			continue;
		}

		vma = find_vma(task->mm, current_pc);

		if (!vma) {
			/* PRTINT_LOG(KERN_ERR "#%d %08lx\n", current_frame, current_pc); */
			current_frame++;
			continue;
		}

		file = vma->vm_file;
		pgoff = ((loff_t)vma->vm_pgoff) << PAGE_SHIFT;

		if (file) {
			base_path = file->f_path;
			path_p = d_path(&base_path, tpath, size);

		} else {
			if (vma->vm_ops && vma->vm_ops->name) {
				path_p = vma->vm_ops->name(vma);

				if (!path_p) {
					path_p = anon_name_array[ANON_NAME_NULL];
				}

			} else {
				mm = vma->vm_mm;

				if (mm) {
					if (vma->vm_start <= mm->brk && vma->vm_end >= mm->start_brk) {
						path_p = anon_name_array[ANON_NAME_HEAP];

					} else if (vma->vm_start <= mm->start_stack && vma->vm_end >= mm->start_stack) {
						path_p = anon_name_array[ANON_NAME_STACK];

					} else if (vma->anon_name) {
						/* if need more details info, look at seq_print_vma_name */
						path_p = anon_name_array[ANON_NAME_ANON];

					} else {
						path_p = anon_name_array[ANON_NAME_NULL];
					}

				} else {
					path_p = anon_name_array[ANON_NAME_VDSO];
				}
			}
		}

		PRTINT_LOG(KERN_ERR "#%d 0x%08llx %s\n", current_frame,
			   current_pc - vma->vm_start + pgoff, path_p);
		current_frame++;
	}
}

static int get_stack_frames(struct task_struct *task,
			    unsigned long *stack_frames, int max_frames)
{
	struct pt_regs *user_regs;
	struct vm_area_struct *vma;
	unsigned long stack_start = 0;
	unsigned long stack_end = 0;
	unsigned long tmp_fp, tmp_fp_next, tmp_lr;
	unsigned long pc_mask = BIT_MASK(VA_BITS) - 1;
	int copied = 0;
	int index = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	MA_STATE(mas, 0, 0, 0);
	mas.tree = &task->mm->mm_mt;
#endif

	if (max_frames < 2) {
		return -EINVAL;
	}

	user_regs = task_pt_regs(task);
	if (!user_mode(user_regs)) {
		return -EFAULT;
	}

	stack_start = (unsigned long)user_regs->user_regs.sp;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	mas_for_each(&mas, vma, ULONG_MAX) {
		if (vma->vm_start <= stack_start && vma->vm_end >= stack_start) {
			stack_end = vma->vm_end;
			break;
		}
	}
#else /* kernel-5.15 or below */
	vma = task->mm->mmap;

	while (vma) {
		if (vma->vm_start <= stack_start && vma->vm_end >= stack_start) {
			stack_end = vma->vm_end;
			break;
		}

		vma = vma->vm_next;

		if (vma == task->mm->mmap) {
			break;
		}
	}
#endif
	if (!stack_end) {
		return -EFAULT;
	}

	stack_frames[index] = user_regs->user_regs.pc;
	index++;
	/* ARM 3-stage pipeline need -4 for LR */
	stack_frames[index] = user_regs->user_regs.regs[30] - 4;
	tmp_fp = user_regs->user_regs.regs[29];

	/* get frames */
	while (tmp_fp < stack_end && tmp_fp > stack_start && index < max_frames - 1) {
		copied = access_process_vm_ubt(task, tmp_fp, &tmp_fp_next,
					   sizeof(tmp_fp_next), 0);

		if (copied != sizeof(tmp_fp_next)) {
			PRTINT_LOG(KERN_ERR "get fp error\n");
			return -EFAULT;
		}

		copied = access_process_vm_ubt(task, tmp_fp + 0x08, &tmp_lr,
						sizeof(tmp_lr), 0);

		if (copied != sizeof(tmp_lr)) {
			PRTINT_LOG(KERN_ERR "get lr error\n");
			return -EFAULT;
		}

		tmp_fp = tmp_fp_next;
		index++;
		stack_frames[index] = tmp_lr & pc_mask - 4;
	}

	/* final frame counts need index add one more, because of begain with index 0 */
	return index + 1;
}

static void show_user_backtrace(struct task_struct *task)
{
	unsigned long stack_frames[OPLUS_BT_MAX_DEPTH];
	char path_name[OPLUS_PATH_NAME_LEN];
	int frames = 0;

	if (!get_task_mm(task)) {
		return;
	}

	if (unlikely(!mmap_read_trylock(task->mm))) {
		mmput(task->mm);
		return;
	}
	frames = get_stack_frames(task, stack_frames, OPLUS_BT_MAX_DEPTH);

	if (frames <= 0) {
		goto exit;
	}

	dump_frames(stack_frames, frames, task, path_name, sizeof(path_name));
exit:
	mmap_read_unlock(task->mm);
	mmput(task->mm);
}


void dump_userspace_backtrace(struct task_struct *task)
{
#ifdef __aarch64__
	char process_info[OPLUS_HEAD_INFO_LEN];
	int info_size = sizeof(process_info);
	int str_size = 0;
#ifdef DEBUG_ESCAPE_TIME
	unsigned long long begin_time = 0;
	unsigned long long end_time = 0;
#endif /* DEBUG_ESCAPE_TIME */

	/* caller should be task context */
	if (!in_task()) {
		PRTINT_LOG(KERN_ERR "ubt: atomic context, just return\n");
		return;
	}

	if (!task) {
		return;
	}

	if (task != task->group_leader) {
		return;
	}

	get_task_struct(task);
	str_size = snprintf(process_info, info_size,
				"name:%s status:%c pid:%d",
				task->comm,
				task_state_to_char(task),
				task_pid_nr(task));

#if IS_ENABLED(CONFIG_SCHED_INFO)
	if (str_size < info_size) {
		str_size += snprintf(process_info + str_size,
					info_size - str_size,
					" last_sched:%llu",
					task->sched_info.last_arrival);
	}
#endif
	PRTINT_LOG(KERN_ERR "%s\n", process_info);

#ifdef DEBUG_ESCAPE_TIME
	begin_time = local_clock();
#endif /* DEBUG_ESCAPE_TIME */

	show_user_backtrace(task);

#ifdef DEBUG_ESCAPE_TIME
	end_time = local_clock();
	PRTINT_LOG(KERN_ERR "dump trace escape time:%llu\n", end_time - begin_time);
#endif /* DEBUG_ESCAPE_TIME */

	put_task_struct(task);
#endif /*__aarch64__*/
}
EXPORT_SYMBOL(dump_userspace_backtrace);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
static void userspace_backtrace_systrigger(void *unused, void *data)
{
	dump_userspace_backtrace((struct task_struct *)data);
}
#endif

static int userspace_backtrace_probe(struct platform_device *pdev)
{
	dev_set_drvdata(&pdev->dev, dump_userspace_backtrace);
	return 0;
}

static int userspace_backtrace_remove(struct platform_device *pdev)
{
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

/* match device using id table */
static const struct platform_device_id userspace_backtrace_id_table[] = {
	/* name within 20 bytes */
	{"ubt,shutdown", 0},
	{"ubt,hung_task", 0},
	{ },
};

static struct platform_driver userspace_backtrace_driver = {
	.probe = userspace_backtrace_probe,
	.remove = userspace_backtrace_remove,
	.driver = {
		.name = "userspace_backtrace",
		.owner  = THIS_MODULE,
	},
	.id_table = userspace_backtrace_id_table,
};

static int __init usersapce_backtrace_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&userspace_backtrace_driver);
	if (ret) {
		PRTINT_LOG(KERN_ERR "%s register platform driver failed\n", __func__);
		return ret;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	ret = register_trace_android_vh_sysrq_crash(userspace_backtrace_systrigger, NULL);
	if (ret) {
		PRTINT_LOG(KERN_ERR "%s register_trace_android_vh_sysrq_crash failed\n", __func__);
		return ret;
	}
#endif
	PRTINT_LOG(KERN_INFO "%s installed\n", __func__);
	return 0;
}
device_initcall(usersapce_backtrace_init);

static void __exit usersapce_backtrace_exit(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	unregister_trace_android_vh_sysrq_crash(userspace_backtrace_systrigger, NULL);
#endif
	platform_driver_unregister(&userspace_backtrace_driver);

	PRTINT_LOG(KERN_INFO "%s has removed\n", __func__);
}
module_exit(usersapce_backtrace_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OPLUS userspace backtrace");

