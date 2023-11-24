// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#ifndef __OPLUS_RWSEM_H__
#define __OPLUS_RWSEM_H__

#include <linux/rwsem.h>

struct oplus_rw_semaphore {
	/* number of ux or rt tasks in wait list */
	atomic_long_t count;
};

static inline struct oplus_rw_semaphore *get_oplus_rw_semaphore(struct rw_semaphore *sem)
{
	return (struct oplus_rw_semaphore *)(sem->android_oem_data1);
}

#endif /* __OPLUS_RWSEM_H__ */
