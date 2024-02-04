// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#ifndef _UXMEM_OPT_H
#define _UXMEM_OPT_H

#include <linux/device.h>
#include <linux/kref.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/shrinker.h>
#include <linux/types.h>

/* #define UXPAGEPOOL_DEBUG 1 */

enum POOL_MIGRATETYPE {
	POOL_MIGRATETYPE_UNMOVABLE,
	POOL_MIGRATETYPE_MOVABLE,
	POOL_MIGRATETYPE_TYPES_SIZE
};
struct page_pool {
	int low[POOL_MIGRATETYPE_TYPES_SIZE];
	int high[POOL_MIGRATETYPE_TYPES_SIZE];
	int count[POOL_MIGRATETYPE_TYPES_SIZE];
	struct list_head items[POOL_MIGRATETYPE_TYPES_SIZE];
	spinlock_t lock;
	unsigned int order;
	gfp_t gfp_mask;
};

struct page *ux_page_pool_alloc_pages(unsigned int order, int migratetype);
int ux_page_pool_refill(struct page *page, unsigned int order, int migratetype);

#endif /* _UXMEM_OPT_H */
