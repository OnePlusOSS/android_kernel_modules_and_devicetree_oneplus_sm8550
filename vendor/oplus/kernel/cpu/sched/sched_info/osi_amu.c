// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/jiffies.h>

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cpumask.h>
#include <linux/sched/topology.h>
#include <linux/sched/task.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/creds.h>

#include "../sched_assist/sa_exec.h"
#include "../sched_assist/sa_common.h"
#include "osi_debug.h"
#include "osi_amu.h"

pid_t global_pid;
DEFINE_PER_CPU(u32, old_pid);
DEFINE_PER_CPU(u64, prev_instr);
DEFINE_PER_CPU(u64, prev_cycle);



struct cpumask amu_cpus __read_mostly;
EXPORT_SYMBOL(amu_cpus);
bool is_support_amu;



static atomic64_t nums_uid = ATOMIC64_INIT(0);
DECLARE_HASHTABLE(amu_uid_table, 6);
EXPORT_SYMBOL(amu_uid_table);
DEFINE_SPINLOCK(amu_uid_lock);

#define ULONG_MAX_DEC_BIT                       (20)
#define UID_PRINT_PER_LINE                      ((28) + 2*ULONG_MAX_DEC_BIT +(2))
#define UID_PRINT_HEADER                        (60)

#define ID_AA64PFR0_AMU_MASK	            ULL(0xf)
#define ID_AA64PFR0_AMU_SUPPORTED                (1)
static inline bool cpu_has_amu(void)
{
	unsigned long amu_bit;

	asm volatile("mrs %0, id_aa64pfr0_el1" : "=r"(amu_bit));
	/* Check AMU bit in ID_AA64PFR0_EL1 */
	amu_bit = (amu_bit >> ID_AA64PFR0_AMU_SHIFT) & ID_AA64PFR0_AMU_MASK;

	return (amu_bit == ID_AA64PFR0_AMU_SUPPORTED);
}

static inline bool cpu_has_amu_feature(int cpu)
{
	return cpumask_test_cpu(cpu, &amu_cpus);
}

static void detect_amu_feat(void *cpu_idx)
{
	int *cpu = cpu_idx;

	if (cpu_has_amu()) {
		cpumask_set_cpu(*cpu, &amu_cpus);
	}
}

/* get  inst, only get from current cpu */
static inline  unsigned long __sched_get_cur_instr(void)
{
	return read_sysreg_s(SYS_AMEVCNTR0_INST_RET_EL0);
}

static inline  unsigned long __sched_get_cur_cycle(void)
{
	return read_sysreg_s(SYS_AMEVCNTR0_CORE_EL0);
}

static void read_amu(void *cpu_idx)
{
	int *cpu = cpu_idx;

	if (cpu_has_amu_feature(*cpu)) {
		per_cpu(prev_instr, *cpu) = __sched_get_cur_instr();
		per_cpu(prev_cycle, *cpu) = __sched_get_cur_cycle();
	}
}

static struct amu_uid_entry *find_uid_entry(uid_t uid)
{
	struct amu_uid_entry *amu_uid_entry;
	hash_for_each_possible(amu_uid_table, amu_uid_entry, node, uid) {
		if (amu_uid_entry->uid == uid)
			return amu_uid_entry;
	}
	return NULL;
}

static __maybe_unused struct amu_uid_entry *find_or_add_uid(uid_t uid)
{
	struct amu_uid_entry *amu_uid_entry;

	amu_uid_entry = find_uid_entry(uid);
	if (amu_uid_entry)
		return amu_uid_entry;

	amu_uid_entry = kzalloc(sizeof(struct amu_uid_entry), GFP_ATOMIC);
	if (!amu_uid_entry)
		return NULL;
	atomic64_inc(&nums_uid);

	amu_uid_entry->uid = uid;
	hash_add(amu_uid_table, &amu_uid_entry->node, uid);

	return amu_uid_entry;
}

static void amu_sched_switch_handler(void *unused, bool preempt, struct task_struct *prev, struct task_struct *next)
{
	u32 cpu = task_cpu(prev);
	u64 instr, cycle;
	pid_t cur_pid = prev->pid;
	u64 delta_amu_instrs, delta_amu_cycle;
	struct oplus_task_struct *ots;
	struct uid_struct *uid_struct;
	unsigned long flags;

	if (!cpu_has_amu_feature(cpu))
		return;
	ots = get_oplus_task_struct(prev);
	if (ots && (per_cpu(old_pid, cpu) != -1) && (cpu == smp_processor_id())) {
		instr = __sched_get_cur_instr();
		cycle = __sched_get_cur_cycle();
		delta_amu_instrs = instr - per_cpu(prev_instr, cpu);
		delta_amu_cycle = cycle - per_cpu(prev_cycle, cpu);
		ots->amu_instruct += delta_amu_instrs;
		ots->amu_cycle += delta_amu_cycle;
		uid_struct = ots->uid_struct;
		if (uid_struct) {
			spin_lock_irqsave(&uid_struct->lock, flags);
			uid_struct->uid_total_inst += delta_amu_instrs;
			uid_struct->uid_total_cycle += delta_amu_cycle;
			spin_unlock_irqrestore(&uid_struct->lock, flags);
		}
		per_cpu(prev_instr, cpu) = instr;
		per_cpu(prev_cycle, cpu) = cycle;
	}
	per_cpu(old_pid, cpu) = cur_pid;
}


static ssize_t proc_osi_amu_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct task_struct *task = NULL, *t = NULL;
	struct oplus_task_struct *ots;
	pid_t tgid;
	char buffer[256];
	size_t len = 0;
	u64 tgid_inst = 0, tgid_cycle = 0;

	if (global_pid > 0 && global_pid <= PID_MAX_DEFAULT) {
		rcu_read_lock();
		task = find_task_by_vpid(global_pid);
		if (task)
			get_task_struct(task);
		rcu_read_unlock();
	}
	if (task) {
		ots = get_oplus_task_struct(task);
		if (ots)
			len = snprintf(buffer, sizeof(buffer), "pid:%d, comm:%s, instr:%llu, cycle:%llu\n",
				task->pid, task->comm, ots->amu_instruct, ots->amu_cycle);

		tgid = task->tgid;

		t = task;
		read_lock(&tasklist_lock);
		do {
			ots = get_oplus_task_struct(t);
			if (ots) {
				tgid_inst += ots->amu_instruct;
				tgid_cycle += ots->amu_cycle;
			}
		} while_each_thread(task, t);
		read_unlock(&tasklist_lock);

		put_task_struct(task);
		len = snprintf(buffer, sizeof(buffer), "pid:%d, tgid:%d, instr:%llu, cycle:%llu\n",
			global_pid, tgid, tgid_inst, tgid_cycle);
	} else
		len = snprintf(buffer, sizeof(buffer), "Can not find task\n");
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_osi_amu_write(struct file *file,
		const char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[PROC_NUMBUF];
	int  pid;
	int  err;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;
	err = kstrtouint(strstrip(buffer), 0, &pid);
	if (err)
		return err;
	global_pid = pid;

	return count;
}

static __maybe_unused void clear_amu_data(struct task_struct *p)
{
	int im_flag = IM_FLAG_NONE;
	struct amu_uid_entry *amu_uid_entry = NULL;
        struct uid_struct *uid_struct;
	struct hlist_node *tmp;
	unsigned long flags;
        int bkt;

	if (p && p->group_leader)
		im_flag = oplus_get_im_flag(p->group_leader);
	if (im_flag == IM_FLAG_MIDASD) {
		spin_lock(&amu_uid_lock);
		hash_for_each_safe(amu_uid_table, bkt, tmp, amu_uid_entry, node) {
			if (amu_uid_entry && amu_uid_entry->uid_struct) {
				uid_struct = amu_uid_entry->uid_struct;
				spin_lock_irqsave(&uid_struct->lock, flags);
				uid_struct->uid_total_inst = 0;
				uid_struct->uid_total_cycle = 0;
				spin_unlock_irqrestore(&uid_struct->lock, flags);
			}
		}
		spin_unlock(&amu_uid_lock);
	}
}


static ssize_t proc_osi_amu_uid_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	int len = 0;
	struct amu_uid_entry *amu_uid_entry = NULL;
	struct uid_struct *uid_struct;
	int bkt, i;
	uid_t uid;
	ssize_t ret;
	char *buffer;
	u64 uid_cnt =  atomic64_read(&nums_uid);

	buffer = kzalloc(uid_cnt * UID_PRINT_PER_LINE + UID_PRINT_HEADER, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	pr_info("proc_osi_amu_uid_read, count:%d", uid_cnt);
	len += sprintf(&buffer[len], "%10s,%15s,%15s,%15s\n",
		"uid", "comm", "inst", "cycle");

	spin_lock(&amu_uid_lock);
	hash_for_each(amu_uid_table, bkt, amu_uid_entry, node) {
		i = 0;
		if (amu_uid_entry && amu_uid_entry->uid_struct) {
			uid_struct = amu_uid_entry->uid_struct;
			uid = uid_struct->uid;
			if(++i > uid_cnt)
				return -EFAULT;
			if (uid >= FIRST_APPLICATION_UID && uid <= LAST_APPLICATION_UID) {
				len += sprintf(&buffer[len], "%10d,%15s,%llu,%llu\n",
					uid_struct->uid, uid_struct->leader_comm,
					uid_struct->uid_total_inst, uid_struct->uid_total_cycle);
			} else {
				len += sprintf(&buffer[len], "%10d,%15s,%llu,%llu\n",
					uid_struct->uid, "RESERVED_UID",
					uid_struct->uid_total_inst, uid_struct->uid_total_cycle);
			}
		}
	}
	spin_unlock(&amu_uid_lock);

	clear_amu_data(current);
	ret = simple_read_from_buffer(buf, count, ppos, buffer, len);
	kfree(buffer);
	return ret;
}

static ssize_t proc_osi_amu_cpu(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	int len;
	char buffer[256];

	len = snprintf(buffer, sizeof(buffer), "amu_cpu:%*pbl\n", cpumask_pr_args(&amu_cpus));
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}


static const struct proc_ops proc_osi_amu_uid_ops = {
	.proc_read      =       proc_osi_amu_uid_read,
	.proc_lseek     =       default_llseek,
};

static const struct proc_ops proc_osi_amu_data_ops = {
	.proc_read	=	proc_osi_amu_read,
	.proc_write     =       proc_osi_amu_write,
	.proc_lseek     =       default_llseek,
};

static const struct proc_ops proc_osi_amu_cpu_ops = {
	.proc_read      =       proc_osi_amu_cpu,
	.proc_lseek             = default_llseek,
};

static struct proc_dir_entry *osi_amu_proc_init(
			struct proc_dir_entry *pde)
{
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("amu_data_tgid", S_IWUGO | S_IRUGO, pde, &proc_osi_amu_data_ops);
	entry = proc_create("amu_data_uid", S_IWUGO | S_IRUGO, pde, &proc_osi_amu_uid_ops);
	entry = proc_create("amu_cpus", S_IWUGO | S_IRUGO, pde, &proc_osi_amu_cpu_ops);

	return entry;
}

static void osi_amu_proc_deinit(struct proc_dir_entry *pde)
{
	remove_proc_entry("amu_data_tgid", pde);
	remove_proc_entry("amu_data_uid", pde);
	remove_proc_entry("amu_cpus", pde);
}


static void dup_or_update_uid_struct(const struct task_struct *task, bool is_commit_cred)
{
	uid_t uid;
	struct uid_struct *new_uid_struct;
	struct oplus_task_struct *ots;
	struct amu_uid_entry *amu_uid_entry;
	struct user_namespace *user_ns = current_user_ns();

	ots = get_oplus_task_struct((struct task_struct *)task);
	if (!ots)
		return;

	rcu_read_lock();
	uid = from_kuid_munged(user_ns, task_uid(task));
	rcu_read_unlock();

	spin_lock(&amu_uid_lock);
	amu_uid_entry = find_or_add_uid(uid);
	if (amu_uid_entry) {
		if (!amu_uid_entry->uid_struct) {
			new_uid_struct =  kzalloc(sizeof(struct uid_struct), GFP_ATOMIC);
			new_uid_struct->uid = uid;
			spin_lock_init(&new_uid_struct->lock);
			amu_uid_entry->uid_struct = new_uid_struct;
		}
		ots->uid_struct = amu_uid_entry->uid_struct;
		if (is_commit_cred)
			memcpy(ots->uid_struct->leader_comm, task->comm, TASK_COMM_LEN);
	}
	spin_unlock(&amu_uid_lock);
}

void android_rvh_sched_fork_init_handler(void *unused, struct task_struct *task)
{
	dup_or_update_uid_struct(task, false);
}

void android_rvh_commit_creds_handler(void *unused, const struct task_struct *task, const struct cred *cred)
{
	static char stack1[50], stack2[50];
	sprintf(stack1, "%ps", CALLER_ADDR3);
	sprintf(stack2, "%ps", CALLER_ADDR2);
	/*
	 *Note: main thread's cred derived from zygote64(uid is 0), so we should update
	 *uid_struct when main thread setuid from userspace.
	 */
	if (!strcmp(stack1, "__sys_setresuid") || !strcmp(stack2, "__sys_setresuid")) {
		dup_or_update_uid_struct(task, true);
	}
}

void osi_task_rename_handler(void *unused, struct task_struct *tsk, const char *buf)
{
	struct oplus_task_struct *ots;
	char comm[128];

	strlcpy(comm, buf, sizeof(comm));
	/*main thread set task_comm in exec after fork*/
	if (tsk->tgid != tsk->pid)
		return;
	ots = get_oplus_task_struct(tsk);
	if (!ots)
		return;
	if (ots->uid_struct)
		memcpy(ots->uid_struct->leader_comm, comm, TASK_COMM_LEN);
}

int osi_amu_init(struct proc_dir_entry *pde)
{
	int ret = 0;
	int cpu, cur_cpu;

	osi_amu_proc_init(pde);
	preempt_disable();
	cur_cpu = smp_processor_id();
	preempt_enable();
	for_each_possible_cpu(cpu) {
		if (cur_cpu == cpu) {
			if (cpu_has_amu()) {
				is_support_amu = true;
				cpumask_set_cpu(cpu, &amu_cpus);
			}
		} else
			smp_call_function_single(cpu, detect_amu_feat, &cpu, 1);
	}

	if (is_support_amu) {
		preempt_disable();
		cur_cpu = smp_processor_id();
		preempt_enable();
		for_each_possible_cpu(cpu) {
			per_cpu(old_pid, cpu) = -1;
			if (cur_cpu == cpu) {
				if (cpu_has_amu_feature(cpu)) {
					per_cpu(prev_instr, cpu) = __sched_get_cur_instr();
					per_cpu(prev_cycle, cpu) = __sched_get_cur_cycle();
				}
			} else
				smp_call_function_single(cpu, read_amu, &cpu, 1);
		}
		REGISTER_TRACE_VH(sched_switch, amu_sched_switch_handler);
		/* register vendor hook in kernel/cred.c  */
		REGISTER_TRACE_RVH(android_rvh_commit_creds, android_rvh_commit_creds_handler);
		/* register vendor hook in kernel/sched/core.c  */
		REGISTER_TRACE_RVH(android_rvh_sched_fork_init, android_rvh_sched_fork_init_handler);
		/* register vender hook in fs/exec.c */
		REGISTER_TRACE_VH(task_rename, osi_task_rename_handler);
	}
	return ret;
}
int osi_amu_exit(struct proc_dir_entry *pde)
{
	int ret = 0;

	if (is_support_amu) {
		UNREGISTER_TRACE_VH(sched_switch, amu_sched_switch_handler);
		UNREGISTER_TRACE_RVH(android_rvh_commit_creds, android_rvh_commit_creds_handler);
		UNREGISTER_TRACE_RVH(android_rvh_sched_fork_init, android_rvh_sched_fork_init_handler);
		UNREGISTER_TRACE_VH(task_rename, osi_task_rename_handler);
	}
	osi_amu_proc_deinit(pde);
	return ret;
}
