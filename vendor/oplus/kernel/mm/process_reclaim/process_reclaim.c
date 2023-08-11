// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "process_reclaim: " fmt

#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched/task.h>
#include <linux/seq_file.h>
#include <linux/mm.h>
#include <linux/sched/mm.h>
#include <linux/rmap.h>
#include <linux/cred.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/mm_inline.h>
#include <linux/swap.h>
#include <linux/hugetlb.h>
#include <linux/huge_mm.h>
#include <asm/tlb.h>
#include <asm/tlbflush.h>
#include <linux/proc_fs.h>
#include <uapi/linux/uio.h>
#include <linux/version.h>
#ifdef  CONFIG_FG_TASK_UID
#include <linux/healthinfo/fg.h>
#endif
#include "process_reclaim.h"

/*
 * check current need cancel reclaim or not, please check task not NULL first.
 * If the reclaimed task has goto foreground, cancel reclaim immediately
 */
enum reclaim_type {
	RECLAIM_ANON,
	RECLAIM_FILE,
	RECLAIM_ALL,
};

struct process_reclaim_cmd {
	union {
		struct iovec *vec;
		char *cmd;
	};
	unsigned int cnt;
	unsigned int max_cnt;
	int readahead;
	unsigned long last_addr;
};

#define PR_TASK_FG	1
#define PR_TASK_RUN	2
#define PR_TASK_DIE	3

#define VECT_LEN_LIMIT (512 << 20)

#define PROCESS_RECLAIM_CMD_LEN 64
static int process_reclaim_enable = 1;
module_param_named(process_reclaim_enable, process_reclaim_enable, int, 0644);

static DEFINE_MUTEX(reclaim_mutex);
static struct proc_dir_entry *enable = NULL;
static struct proc_dir_entry *process_reclaim = NULL;

static inline int is_reclaim_should_cancel(struct task_struct *task,
		struct mm_struct *mm)
{
	if (mm != task->mm)
		return -PR_TASK_DIE;

#ifdef CONFIG_FG_TASK_UID
	if (task_is_fg(task))
		return -PR_TASK_FG;
#endif

	if (task->state == TASK_RUNNING)
		return -PR_TASK_RUN;

	return 0;
}

static inline bool can_madv_lru_vma(struct vm_area_struct *vma)
{
	return !(vma->vm_flags & (VM_LOCKED|VM_HUGETLB|VM_PFNMAP));
}

static inline bool can_do_pageout(struct vm_area_struct *vma, enum reclaim_type type)
{
	switch (type) {
	case RECLAIM_ANON:
		return vma_is_anonymous(vma);
	case RECLAIM_FILE:
		if (vma_is_anonymous(vma))
			return false;
		if (!vma->vm_file)
			return false;
		return true;
	case RECLAIM_ALL:
		if (vma_is_anonymous(vma))
			return true;
		if (!vma->vm_file)
			return false;
	default:
		return false;
	}

	return false;
}

static noinline ssize_t reclaim_task(struct task_struct *task, char *buffer,
		struct iovec *uvector, size_t max_cnt, unsigned long *last_addr,
		int readahead)
{
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	enum reclaim_type type;
	enum reclaim_type vma_type;
	char *type_buf;
	unsigned long start, end;
	int ret = 0, i;

	if (task == current->group_leader)
		goto out_err;

	type_buf = strstrip(buffer);
	if (!strcmp(type_buf, "file"))
		type = RECLAIM_FILE;
	else if (!strcmp(type_buf, "anon"))
		type = RECLAIM_ANON;
	else if (!strcmp(type_buf, "all"))
		type = RECLAIM_ALL;
	else {
		ret = -EINVAL;
		goto out_err;
	}

	mm = get_task_mm(task);
	if (!mm)
		return -EINVAL;

	start = 0;
	end = 0;
	i = 0;
	down_read(&mm->mmap_lock);
	for (vma = mm->mmap; vma; vma = vma->vm_next) {
		if (is_vm_hugetlb_page(vma))
			continue;

		if (!readahead) {
			if (vma->vm_end < *last_addr)
				continue;
			if (!can_madv_lru_vma(vma))
				continue;
		}

		if (type == RECLAIM_ALL)
			vma_type = RECLAIM_ALL;
		else
			vma_type = !vma_is_anonymous(vma);

		if (type == vma_type) {
			if (start == 0) {
				start = vma->vm_start;
				end = vma->vm_end;
			} else if (vma->vm_start == end) {
				end = vma->vm_end;
				if (end - start >= VECT_LEN_LIMIT) {
					uvector[i].iov_base = (void*)start;
					uvector[i].iov_len = (size_t)(end - start);
					*last_addr = end;
					i++;
					start = 0;
				}
			} else {
				uvector[i].iov_base = (void*)start;
				uvector[i].iov_len = (size_t)(end - start);
				*last_addr = end;
				i++;
				start = vma->vm_start;
				end = vma->vm_end;
			}
		} else {
			if (start == 0)
				continue;
			uvector[i].iov_base = (void*)start;
			uvector[i].iov_len = (size_t)(end - start);
			*last_addr = end;
			i++;
			start = 0;
		}

		if (!readahead) {
			ret = is_reclaim_should_cancel(task, mm);
			if (ret)
				break;
		}

		if (i >= max_cnt)
			break;
	}

	if (ret < 0) {
		pr_err("task %s cancel reclaim, readahead %d ret %d\n",
				task->comm, readahead, ret);
		up_read(&mm->mmap_lock);
		mmput(mm);
		ret = -EPERM;
		goto out_err;
	}

	if (type == vma_type && start != 0) {
		uvector[i].iov_base = (void*)start;
		uvector[i].iov_len = (size_t)(end - start);
		*last_addr = end;
		i++;
	}

	/* the last one is not save anything*/
	if (i > 0)
		i--;

	ret = i;
	up_read(&mm->mmap_lock);
	mmput(mm);

out_err:
	return ret;
}

static unsigned int scan_process_iovec(char *kbuf, size_t count,
		struct iovec *uvector, size_t max_cnt, unsigned long *last_addr,
		int readahead)
{
	const unsigned int params_num = 2;
	pid_t tsk_pid;
	struct task_struct* tsk;
	ssize_t ret = 0;
	char type[12] = {'\0'};

	ret = mutex_trylock(&reclaim_mutex);
	if (!ret) {
		pr_err("There is another thread doing process reclaim\n");
		return -EAGAIN;
	}

	kbuf = strstrip(kbuf);
	if (sscanf(kbuf, "%d %s", &tsk_pid, &type) != params_num) {
		pr_err("kbuf value is invalid: %s\n", kbuf);
		ret = -EINVAL;
		goto done;
	}

	rcu_read_lock();
	tsk = find_task_by_vpid(tsk_pid);
	if (!tsk) {
		rcu_read_unlock();
		pr_err("process_reclaim can not find task of pid:%d\n", tsk_pid);
		ret = -ESRCH;
		goto done;
	}
	if (tsk != tsk->group_leader)
		tsk = tsk->group_leader;
	get_task_struct(tsk);
	rcu_read_unlock();
	ret = reclaim_task(tsk, type, uvector, max_cnt, last_addr, readahead);
	put_task_struct(tsk);

	if (ret < 0)
		pr_err("process_reclaim failed, command [%s]\n", kbuf);

done:
	mutex_unlock(&reclaim_mutex);
	return ret;
}

static ssize_t process_reclaim_enable_write(struct file *file,
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

	ret = kstrtol(kbuf, 10, &val);
	if (ret)
		return -EINVAL;

	process_reclaim_enable = val ? 1 : 0;
	return len;
}

static ssize_t process_reclaim_enable_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[12] = {'0'};
	int len;

	len = snprintf(kbuf, 12, "%d\n", process_reclaim_enable);
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

long process_reclaim_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct process_reclaim_cmd cmd_entry;
	unsigned int iovec_size;
	char cmdline[PROCESS_RECLAIM_CMD_LEN];
	struct iovec *uvectors;
	int ret;
	unsigned long last_addr;

	if (!process_reclaim_enable) {
		pr_warn("Process memory reclaim is disabled!\n");
		return -EPERM;
	}

	if (!access_ok((void *)arg, sizeof(struct process_reclaim_cmd))) {
		pr_err("arg is invalid\n");
		return -EINVAL;
	}

	if (copy_from_user(&cmd_entry, (void *)arg,
				sizeof(struct process_reclaim_cmd))) {
		pr_err("copy arg failed");
		return -ENOMEM;
	}

	if (cmd_entry.cnt == 0 || cmd_entry.max_cnt == 0 ||
			cmd_entry.cmd == NULL ||
			cmd_entry.cnt >= PROCESS_RECLAIM_CMD_LEN ||
			cmd_entry.max_cnt > UIO_MAXIOV * 2) {
		pr_err("cmd data is invalid, cnt %u max_cnt %u cmd %p\n",
				cmd_entry.cnt, cmd_entry.max_cnt,
				cmd_entry.cmd);
		return -EINVAL;
	}

	iovec_size = cmd_entry.max_cnt * sizeof(struct iovec);
	if (!access_ok((void *)cmd_entry.cmd, iovec_size)) {
		pr_err("cmd is invalid\n");
		return -EINVAL;
	}

	memset(cmdline, 0, PROCESS_RECLAIM_CMD_LEN);
	if (copy_from_user(cmdline, (void *)cmd_entry.cmd, cmd_entry.cnt)) {
		pr_err("copy arg failed");
		return -ENOMEM;
	}

	uvectors = vmalloc(iovec_size);
	if (!uvectors) {
		pr_err("vmalloc uvectors failed.\n");
		return -ENOMEM;
	}

	last_addr = cmd_entry.last_addr;
	ret = scan_process_iovec(cmdline, cmd_entry.cnt, uvectors,
			cmd_entry.max_cnt, &last_addr, cmd_entry.readahead);
	if (ret > 0) {
		cmd_entry.last_addr = last_addr;
		(void)copy_to_user(cmd_entry.vec, uvectors,
				ret * sizeof(struct iovec));
		cmd_entry.cnt = ret;

		(void) copy_to_user((void *)arg, (void*)&cmd_entry,
				sizeof(struct process_reclaim_cmd));
	}
	vfree(uvectors);

	return ret;
}

static const struct proc_ops proc_process_reclaim_enable_ops = {
	.proc_write          = process_reclaim_enable_write,
	.proc_read		= process_reclaim_enable_read,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops process_reclaim_fops = {
	.proc_ioctl		= process_reclaim_ioctl,
	.proc_compat_ioctl	= process_reclaim_ioctl,
	.proc_lseek		= default_llseek,
};

static int __init process_reclaim_init_procfs(void)
{
	struct proc_dir_entry *root_dir_entry = proc_mkdir("oplus_mem", NULL);

	process_reclaim = proc_create((root_dir_entry ?
				"process_reclaim" : "oplus_mem/process_reclaim"),
			S_IRUGO|S_IWUGO, root_dir_entry, &process_reclaim_fops);
	if (!process_reclaim) {
		pr_err("Failed to create process_reclaim\n");
		return -ENOMEM;
	}

	enable = proc_create((root_dir_entry ? "process_reclaim_enable" :
				"oplus_mem/process_reclaim_enable"),
			S_IRUGO|S_IWUGO, root_dir_entry,
			&proc_process_reclaim_enable_ops);
	if (!enable) {
		pr_err("Register process_reclaim_enable failed.\n");
		proc_remove(process_reclaim);
		process_reclaim = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void process_reclaim_destory_procfs(void)
{
	proc_remove(enable);
	proc_remove(process_reclaim);
}

static int __init process_reclaim_proc_init(void)
{
	return process_reclaim_init_procfs();
}

static void process_reclaim_proc_exit(void)
{
	process_reclaim_destory_procfs();
}

module_init(process_reclaim_proc_init);
module_exit(process_reclaim_proc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("process reclaim feature");
