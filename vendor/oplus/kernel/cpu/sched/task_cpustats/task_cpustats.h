/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */
#ifndef _OPLUS_TASK_CPUSTATS_H
#define _OPLUS_TASK_CPUSTATS_H
#include <linux/kernel_stat.h>
#include <linux/cpufreq.h>

#define MAX_CTP_WINDOW (10 * NSEC_PER_SEC / TICK_NSEC)

#define task_err(fmt, ...) \
		printk(KERN_ERR "[TASK_CPUSTATS_ERR][%s]"fmt, __func__, ##__VA_ARGS__)

struct task_cpustat {
	pid_t pid;
	pid_t tgid;
	enum cpu_usage_stat type;
	int freq;
	unsigned long begin;
	unsigned long end;
	char comm[TASK_COMM_LEN];
};

struct kernel_task_cpustat {
	unsigned int idx;
	struct task_cpustat cpustat[MAX_CTP_WINDOW];
};

#endif /* _OPLUS_TASK_CPUSTATS_H */
