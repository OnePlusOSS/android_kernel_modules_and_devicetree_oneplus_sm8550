/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _LINUX_MEMLEAK_STACKDEPOT_H
#define _LINUX_MEMLEAK_STACKDEPOT_H

typedef u32 ml_depot_stack_handle_t;

ml_depot_stack_handle_t ml_depot_save_stack(unsigned long *entries, unsigned int nr_entries, gfp_t flags);
unsigned int ml_depot_fetch_stack(ml_depot_stack_handle_t handle, unsigned long **entries);
int ml_depot_init(struct proc_dir_entry *parent);
void ml_depot_destory(void);
#endif /* _LINUX_MEMLEAK_STACKDEPOT_H */
