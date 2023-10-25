// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include "sched_assist.h"
#include "sa_common.h"
#include "sa_exec.h"

/* register vender hook in driver/android/exec.c */
void task_rename_handler(void *unused, struct task_struct *tsk, const char *buf)
{
	sched_assist_target_comm(tsk, buf);
}
