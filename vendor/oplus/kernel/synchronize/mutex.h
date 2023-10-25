// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2023 Oplus. All rights reserved.
 */

#ifndef __OPLUS_MUTEX_H__
#define __OPLUS_MUTEX_H__

#include <linux/mutex.h>

struct oplus_mutex {
	/* number of ux or rt tasks in waiter list */
	atomic_long_t count;
};

static inline struct oplus_mutex * get_oplus_mutex(struct mutex *lock)
{
	return (struct oplus_mutex *)lock->android_oem_data1;
}

#endif /* __OPLUS_MUTEX_H__ */
