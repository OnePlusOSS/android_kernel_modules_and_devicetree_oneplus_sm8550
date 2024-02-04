/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2021 Oplus. All rights reserved.
 */
#ifndef _OPLUS_TASK_SCHED_INFO_H
#define _OPLUS_TASK_SCHED_INFO_H

#define sched_err(fmt, ...) \
		printk(KERN_ERR "[SCHED_INFO_ERR][%s]"fmt, __func__, ##__VA_ARGS__)

#define COMM_LEN 64
#define WORKER_NAME_NUM 4

enum {
	task_sched_info_running = 0,
	task_sched_info_runnable,
	task_sched_info_IO,
	task_sched_info_D,
	task_sched_info_S,
	task_sched_info_freq,
	task_sched_info_freq_limit,
	task_sched_info_isolate,
	task_sched_info_backtrace,
};

enum {
	block_runnable = 1,
	running_runnable,
};

enum {
	cpu_unisolate = 0,
	cpu_isolate,
};

enum {
	req_freq_limit_min = 1,
	req_freq_limit_max,
};

struct task_sched_info {
	u64 sched_info_one;
	u64 sched_info_two;
};

struct freq_limit_notifier {
	struct notifier_block notifier_min;
	struct notifier_block notifier_max;
	int cpu;
	int req_min_tid;
	int req_max_tid;
	u8 cur_type;
	bool min_flag_worker;
	bool max_flag_worker;
	char min_comm[COMM_LEN];
	char max_comm[COMM_LEN];
	spinlock_t notifier_lock;
};

struct freq_limit_worker {
	unsigned int pid;
	char worker[WORKER_NAME_NUM][COMM_LEN];
	int num;
	struct list_head worker_list_entry;
};

struct freq_limit_thread {
	unsigned int pid;
	char thread[COMM_LEN];
	struct list_head thread_list_entry;
};

extern int sched_info_init(void);
extern int register_sched_info_vendor_hooks(void);
extern void update_cpu_isolate_info(int cpu, u64 type);
extern void update_cpus_isolate_info(struct cpumask *cpus, u64 type);

#endif /* _OPLUS_TASK_SCHED_INFO_H */

