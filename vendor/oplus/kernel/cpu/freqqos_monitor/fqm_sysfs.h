/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */

#ifndef _OPLUS_FQM_SYSFS_H_
#define _OPLUS_FQM_SYSFS_H_

int freqqos_monitor_proc_init(void);
void freqqos_monitor_proc_exit(void);

extern int g_fqm_monitor_enable;
extern int g_fqm_debug_enable;

#endif /* _OPLUS_FQM_SYSFS_H*/
