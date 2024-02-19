/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef _ION_BOOST_POOL_H
#define _ION_BOOST_POOL_H

#include <linux/kthread.h>
#include <linux/types.h>
#ifdef CONFIG_QCOM_DMABUF_HEAPS_SYSTEM
#include "qcom_dynamic_page_pool.h"
#else
#include "page_pool.h"
#endif

#define LOWORDER_WATER_MASK (64*4)

struct dynamic_boost_pool {
	char *name;
	struct list_head list;
	int sf_pages, camera_pages;
	int low, high, origin;
	pid_t camera_pid;
	struct task_struct *tsk, *prefill_tsk;
	unsigned int wait_flag, prefill_wait_flag;
	wait_queue_head_t waitq, prefill_waitq;
	bool force_stop, prefill;
	struct mutex prefill_mutex;
	struct dynamic_page_pool **pools;
};

int dynamic_boost_pool_free(struct dynamic_boost_pool *pool, struct page *page,
			    int index);
void dynamic_boost_pool_alloc_pack(struct dynamic_boost_pool *boost_pool, unsigned long *size_remaining_p,
				   unsigned int *max_order_p, struct list_head *pages_p, int *i_p);
struct dynamic_boost_pool *dynamic_boost_pool_create_pack(void);

#endif /* _ION_BOOST_POOL_H */
