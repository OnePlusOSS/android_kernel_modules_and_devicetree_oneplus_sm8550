/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SA_BALANCE_H_
#define _OPLUS_SA_BALANCE_H_


void oplus_loadbalance_init(void);
void oplus_loadbalance_deinit(void);
bool __oplus_tick_balance(void *data, struct rq *rq);
bool __oplus_newidle_balance(void *data, struct rq *this_rq,
					struct rq_flags *rf, int *pulled_task, int *done);

int im_flag_to_prio(int im_flag);
void add_rt_boost_task(struct task_struct *p);
void remove_rt_boost_task(struct task_struct *p);
bool task_is_rt_boost(struct task_struct *task);

#endif /* _OPLUS_SA_BALANCE_H_ */

