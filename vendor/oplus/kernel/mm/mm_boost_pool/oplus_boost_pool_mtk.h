// SPDX-License-Identifier: GPL-2.0
/*
 * dmabuf boost pool changed by oplus
 *
 * DMA BUF page pool system
 *
 * Copyright (C) 2020 Linaro Ltd.
 *
 * Based on the ION page pool code
 * Copyright (C) 2011 Google, Inc.
 */

#ifndef _BOOST_POOL_H
#define _BOOST_POOL_H

#include <linux/device.h>
#include <linux/kref.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/shrinker.h>
#include <linux/types.h>

#include <page_pool.h>

/**
 * struct boost_page_pool - pagepool struct
 * @count[]:		array of number of pages of that type in the pool
 * @items[]:		array of list of pages of the specific type
 * @mutex:		lock protecting this struct and especially the count
 *			item list
 * @gfp_mask:		gfp_mask to use from alloc
 * @order:		order of pages in the pool
 * @list:		list node for list of pools
 *
 * Allows you to keep a pool of pre allocated pages to use
 */
struct boost_page_pool {
	int count[POOL_TYPE_SIZE];
	struct list_head items[POOL_TYPE_SIZE];
	struct mutex mutex;
	gfp_t gfp_mask;
	unsigned int order;
	struct list_head list;
};

struct boost_pool {
	bool stop, prefill;
	unsigned int wait_flag;
	pid_t custom_pid;
	wait_queue_head_t waitq;
	struct task_struct *prefill_task;
	struct mutex prefill_lock;
	int min, low, high;
	struct list_head list;
	struct boost_page_pool *pools[0];
};

#define MAX_BOOST_POOL_HIGH		(SZ_1G >> PAGE_SHIFT)
#define MAX_BOOST_POOL_CACHE		(SZ_128M >> PAGE_SHIFT)
#define MAX_BOOST_POOL_TIMEOUT		(2 * HZ)

#define CMD_BOOST_POOL_STOP		(-1)
#define CMD_BOOST_POOL_RESET		(0)

#define VERSION_BOOST_POOL_MAJOR	(1)
#define VERSION_BOOST_POOL_MINOR	(0)
#define VERSION_BOOST_POOL_REVISION	(1)

struct boost_pool *boost_pool_create(const char *name);
struct page *boost_pool_fetch(struct boost_pool *boost_pool,
			      int order_index);
int boost_pool_free(struct boost_pool *boost_pool, struct page *page,
		    int order_index);
#endif /* _BOOST_POOL_H */
