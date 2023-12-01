/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */

#ifndef _OPLUS_SA_PIPELINE_H_
#define _OPLUS_SA_PIPELINE_H_

#include "sa_common.h"

void oplus_pipeline_init(struct proc_dir_entry *pde);
inline int oplus_get_task_pipeline_cpu(struct task_struct *task);
inline bool oplus_is_pipeline_task(struct task_struct *task);
inline bool pipeline_task_skip_ux_change(struct oplus_task_struct *ots, int *ux_state);
inline bool pipeline_task_skip_cpu(struct task_struct *task, unsigned int dst_cpu);
inline void pipeline_task_enqueue(struct oplus_task_struct *ots);
inline void pipeline_task_switch_out(struct task_struct *prev);

typedef int (*core_ctl_set_boost_t)(bool boost);
typedef int (*core_ctl_set_cluster_boost_t)(int idx, bool boost);

extern core_ctl_set_boost_t oplus_core_ctl_set_boost;
extern core_ctl_set_cluster_boost_t oplus_core_ctl_set_cluster_boost;

#endif /* _OPLUS_SA_PIPELINE_H_ */
