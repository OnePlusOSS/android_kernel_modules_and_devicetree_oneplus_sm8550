/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#ifndef _OPLUS_SA_AUDIO_H_
#define _OPLUS_SA_AUDIO_H_

int oplus_sched_assist_audio_proc_init(struct proc_dir_entry *dir);
void oplus_sched_assist_audio_proc_remove(struct proc_dir_entry *dir);
void oplus_sched_assist_audio_perf_set_status(int status);
void oplus_sched_assist_audio_latency_sensitive(struct task_struct *t, bool *latency_sensitive);
void oplus_sched_assist_audio_time_slack(struct task_struct *task);
bool oplus_sched_assist_audio_idle_balance(struct rq *this_rq);
void oplus_sched_assist_audio_perf_addIm(struct task_struct *task, int im_flag);
bool oplus_sched_assist_audio_perf_check_exit_latency(struct task_struct *task, int cpu);
void oplus_sched_assist_audio_enqueue_hook(struct task_struct *task);
extern void oplus_sched_assist_audio_set_wake_up_idle(struct task_struct *p);
#endif /* _OPLUS_SA_AUDIO_H_ */
