/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 Google, Inc.
 */

#ifndef _OPLUS_BINDER_SCHED_H_
#define _OPLUS_BINDER_SCHED_H_

#include <linux/sched.h>
#include <uapi/linux/android/binder.h>

#define SET_ASYNC_UX_ENABLE				0x45555801
#define ASYNC_UX_ENABLE_DATA_SIZE		4
#define OBS_NOT_ASYNC_UX_VALUE			0xfffffffffffffff1	//(unsigned long - MAX_ERRNO - ...)

#define CURRENT_TASK_PID				-1

enum OBS_STATUS {
	 OBS_INVALID,
	 OBS_VALID,
	 OBS_NOT_ASYNC_UX,
};

#define INVALID_VALUE           -1
#define MAX_UX_IN_LIST			20

enum ASYNC_UX_TEST_ITEM {
	 ASYNC_UX_TEST_DISABLE,
	 ASYNC_UX_RANDOM_LOW_INSERT_TEST,
	 ASYNC_UX_RANDOM_HIGH_INSERT_TEST,
	 ASYNC_UX_RANDOM_LOW_ENQUEUE_TEST,
	 ASYNC_UX_RANDOM_HIGH_ENQUEUE_TEST,
	 ASYNC_UX_INORDER_TEST,
};

enum ASYNC_UX_ENABLE_ITEM {
	ASYNC_UX_DISABLE,
	ASYNC_UX_ENABLE_ENQUEUE,
	ASYNC_UX_ENABLE_INSERT_QUEUE,
	ASYNC_UX_ENABLE_MAX,
};

enum BINDER_THREAD_MODE {
	THREAD_MODE_UNKNOWN,
	THREAD_MODE_SYNC,
	THREAD_MODE_ASYNC,
};

enum {
	BINDER_LOG_CRITICAL		= 1U << 0,
	BINDER_LOG_INFO			= 1U << 1,
	BINDER_LOG_DEBUG		= 1U << 2,
};

struct oplus_binder_struct {
	int async_ux_enable;
};

#endif /* _OPLUS_BINDER_SCHED_H_ */
