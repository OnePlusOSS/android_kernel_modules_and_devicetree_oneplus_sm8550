/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SA_SYSFS_H_
#define _OPLUS_SA_SYSFS_H_

extern int global_sched_assist_enabled;
extern int global_sched_assist_scene;
extern char global_ux_task[];

int oplus_sched_assist_proc_init(void);
void oplus_sched_assist_proc_deinit(void);

extern struct task_struct *find_task_by_vpid(pid_t vnr);

#endif /* _OPLUS_SA_SYSFS_H_ */
