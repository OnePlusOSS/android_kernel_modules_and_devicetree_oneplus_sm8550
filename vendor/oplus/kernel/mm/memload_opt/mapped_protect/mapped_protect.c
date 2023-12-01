// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "mapped_protect: " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <trace/hooks/vmscan.h>
#include <trace/hooks/mm.h>
#include <linux/swap.h>
#include <linux/proc_fs.h>
#include <linux/gfp.h>
#include <linux/printk.h>
#include <linux/mmzone.h>
#include <linux/mm.h>
#include <linux/mm_inline.h>
#include <linux/pagemap.h>
#include <linux/page-flags.h>
#include <linux/debugfs.h>
#include <linux/memcontrol.h>

#ifdef CONFIG_MAPPED_PROTECT_ALL
#define MAPCOUNT_PROTECT_THRESHOLD (20)
#define RETRY_GET_MAPCOUNT (3)
/*
 * [0]: nr_anon_mapped_multiple
 * [1]: nr_file_mapped_multiple
 *
 * */
static atomic_long_t nr_mapped_multiple[2] = {
	ATOMIC_INIT(0)
};
static atomic_long_t nr_mapped_multiple_debug[2] = {
	ATOMIC_INIT(0)
};

long  max_nr_mapped_multiple[2] = { 0 };
unsigned long  mapcount_protected_high[2] = { 0 };

unsigned long  memavail_noprotected = 0;
static bool mapcount_protect_setup = false;
extern struct lruvec* page_to_lruvec(struct page *page, pg_data_t *pgdat);
extern void do_traversal_all_lruvec(void);
extern int __page_mapcount(struct page *page);

static inline bool page_evictable(struct page *page)
{
	bool ret;

	/* Prevent address_space of inode and swap cache from being freed */
	rcu_read_lock();
	ret = !mapping_unevictable(page_mapping(page)) && !PageMlocked(page);
	rcu_read_unlock();
	return ret;
}

static bool mapped_protected_is_full(int file)
{
	if (atomic_long_read(&nr_mapped_multiple[file])
			> mapcount_protected_high[file])
		return true;
	else
		return false;
}

static bool mem_available_is_low(void)
{
	long available = si_mem_available();

	if (available < memavail_noprotected)
		return true;

	return false;
}

static void mark_page_accessed_handler(void *data, struct page* page)
{
	struct pglist_data *pgdat = page_pgdat(page);
	struct lruvec *lruvec;
	int file;

	if (likely(page_mapcount(page) < MAPCOUNT_PROTECT_THRESHOLD))
		return;

	if (!PageLRU(page) || PageUnevictable(page))
		return;

	if (!PageActive(page) && !PageUnevictable(page) &&
			(PageReferenced(page) || page_mapcount(page) > 10))
		return;

	file = page_is_file_lru(page);
	if (unlikely(mapped_protected_is_full(file)))
		return;

	lruvec = page_to_lruvec(page, pgdat);
	if (spin_trylock_irq(&lruvec->lru_lock)) {
		int lru;

		lru = page_lru(page);
		if (PageLRU(page) && !PageUnevictable(page))
			list_move(&page->lru, &lruvec->lists[lru]);
		spin_unlock_irq(&lruvec->lru_lock);
	}
}

static void page_should_be_protect(void *data, struct page* page,
				bool *should_protect)
{
	int file;

	if (unlikely(!mapcount_protect_setup)) {
		*should_protect = false;
		return;
	}

	if (likely(page_mapcount(page) < MAPCOUNT_PROTECT_THRESHOLD)) {
		*should_protect = false;
		return;
	}

	if (unlikely(!page_evictable(page) || PageUnevictable(page))) {
		*should_protect = false;
		return;
	}

	file = page_is_file_lru(page);
	if (unlikely(mapped_protected_is_full(file))) {
		*should_protect = false;
		return;
	}

	if (unlikely(mem_available_is_low())) {
		*should_protect = false;
		return;
	}

	*should_protect = true;
}

static void update_page_mapcount(void *data, struct page *page, bool inc_size, bool compound, bool *ret, bool *success)
{
	unsigned long nr_mapped_multi_pages;
	int file, mapcount;

	*success = true;
	if (inc_size) {
		mapcount = atomic_inc_return(&page->_mapcount);
		if (ret)
			*ret = ((mapcount == 0) ? true : false);
	} else {
		mapcount = atomic_add_return(-1, &page->_mapcount);
		if (ret)
			*ret = ((mapcount < 0) ? true : false);
	}
	/* we update multi-mapped counts when page_mapcount(page) changing:
	 * - 19->20
	 * - 20->19
	 * Because we judge mapcount by page_mapcount(page) which return
	 * page->_mapcount + 1, variable "mapcount" + 1 = page_mapcount(page).
	 * If inc_size is equal to true, "mapcount" + 1 = MAPCOUNT_PROTECT_THRESHOLD
	 * which means 19->20.
	 * If inc_size is equal to false, "mapcount" + 2 = MAPCOUNT_PROTECT_THRESHOLD
	 * which means 20->19.
	 * */
	if (likely((mapcount + (inc_size ? 1 : 2)) !=
					MAPCOUNT_PROTECT_THRESHOLD))
		return;

	if (unlikely(!mapcount_protect_setup))
		return;

	if (!PageLRU(page) || PageUnevictable(page))
		return;

	file = page_is_file_lru(page);

	if (inc_size) {
		atomic_long_add(1, &nr_mapped_multiple[file]);
		nr_mapped_multi_pages = atomic_long_read(&nr_mapped_multiple[file]);
		if (max_nr_mapped_multiple[file] < nr_mapped_multi_pages)
			max_nr_mapped_multiple[file] = nr_mapped_multi_pages;
	} else {
		atomic_long_sub(1, &nr_mapped_multiple[file]);
	}

	return;
}

static void add_page_to_lrulist(void *data, struct page *page, bool compound, enum lru_list lru)
{
	unsigned long nr_mapped_multi_pages;
	int file;

	if (unlikely(!mapcount_protect_setup))
		return;

	if (likely(page_mapcount(page) < MAPCOUNT_PROTECT_THRESHOLD))
		return;

	if (lru == LRU_UNEVICTABLE)
		return;

	file = page_is_file_lru(page);

	atomic_long_add(1, &nr_mapped_multiple[file]);
	nr_mapped_multi_pages = atomic_long_read(&nr_mapped_multiple[file]);
	if (max_nr_mapped_multiple[file] < nr_mapped_multi_pages)
		max_nr_mapped_multiple[file] = nr_mapped_multi_pages;
}

static void del_page_from_lrulist(void *data, struct page *page, bool compound, enum lru_list lru)
{
	int file;

	if (unlikely(!mapcount_protect_setup))
		return;

	if (likely(page_mapcount(page) < MAPCOUNT_PROTECT_THRESHOLD))
		return;

	if (lru == LRU_UNEVICTABLE)
		return;

	file = page_is_file_lru(page);

	atomic_long_sub(1, &nr_mapped_multiple[file]);
}

static void do_traversal_lruvec(void *data, struct lruvec *lruvec)
{
	struct page *page;
	int lru, i;

	spin_lock_irq(&lruvec->lru_lock);
	for_each_evictable_lru(lru) {
		int file = is_file_lru(lru);
                list_for_each_entry(page, &lruvec->lists[lru], lru) {
			if (!page)
				continue;
			if (PageHead(page)) {
				for (i = 0; i < compound_nr(page); i++)
					if (page_mapcount(&page[i]) >= MAPCOUNT_PROTECT_THRESHOLD)
						atomic_long_add(1, &nr_mapped_multiple_debug[file]);
				continue;
			}
			if (page_mapcount(page) >= MAPCOUNT_PROTECT_THRESHOLD)
				atomic_long_add(thp_nr_pages(page), &nr_mapped_multiple_debug[file]);
		}
	}
	spin_unlock_irq(&lruvec->lru_lock);
}

static void show_mapcount_pages(void *data, void *unused)
{
	printk(" multi_mapped0:%ld multi_mapped1:%ld max0:%ld max1:%ld\n",
		atomic_long_read(&nr_mapped_multiple[0]),
		atomic_long_read(&nr_mapped_multiple[1]),
		max_nr_mapped_multiple[0],
		max_nr_mapped_multiple[1]);
}
#endif

static void should_skip_page_referenced(void *data, struct page *page, unsigned long nr_to_scan, int lru, bool *bypass)
{
#ifdef CONFIG_MAPPED_PROTECT_ALL
	if (page_mapcount(page) >= 20)
#else
	if ((atomic_read(&page->_mapcount) + 1) >= 20)
#endif
		*bypass = true;
}

static int register_mapped_protect_vendor_hooks(void)
{
	int ret = 0;

	ret = register_trace_android_vh_page_referenced_check_bypass(should_skip_page_referenced, NULL);
	if (ret != 0) {
		pr_err("register_trace_android_vh_skip_page_referenced failed! ret=%d\n", ret);
		goto out;
	}

#ifdef CONFIG_MAPPED_PROTECT_ALL
	ret = register_trace_android_vh_update_page_mapcount(update_page_mapcount, NULL);
	if (ret != 0) {
		pr_err("register update_mapped_mul vendor_hooks failed! ret=%d\n", ret);
		goto out;
	}
	ret = register_trace_android_vh_add_page_to_lrulist(add_page_to_lrulist, NULL);
	if (ret != 0) {
		pr_err("register add_mapped_mul_op_lrulist vendor_hooks failed! ret=%d\n", ret);
		goto out1;
	}
	ret = register_trace_android_vh_del_page_from_lrulist(del_page_from_lrulist, NULL);
	if (ret != 0) {
		pr_err("register dec_mapped_mul_op_lrulist vendor_hooks failed! ret=%d\n", ret);
		goto out2;
	}
	ret = register_trace_android_vh_page_should_be_protected(page_should_be_protect, NULL);
	if (ret != 0) {
		pr_err("register page_should_be_protect vendor_hooks failed! ret=%d\n", ret);
		goto out3;
	}
	ret = register_trace_android_vh_mark_page_accessed(mark_page_accessed_handler, NULL);
	if (ret != 0) {
		pr_err("register mark_page_accessed vendor_hooks failed! ret=%d\n", ret);
		goto out4;
	}
	ret = register_trace_android_vh_do_traversal_lruvec(do_traversal_lruvec, NULL);
	if (ret != 0) {
		pr_err("register do_traversal_lruvec vendor_hooks failed! ret=%d\n", ret);
		goto out5;
	}
	ret = register_trace_android_vh_show_mapcount_pages(show_mapcount_pages, NULL);
	if (ret != 0) {
		pr_err("register do_traversal_lruvec vendor_hooks failed! ret=%d\n", ret);
		goto out6;
	}
	return ret;

out6:
	unregister_trace_android_vh_do_traversal_lruvec(do_traversal_lruvec, NULL);
out5:
	unregister_trace_android_vh_mark_page_accessed(mark_page_accessed_handler, NULL);
out4:
	unregister_trace_android_vh_page_should_be_protected(page_should_be_protect, NULL);
out3:
	unregister_trace_android_vh_del_page_from_lrulist(del_page_from_lrulist, NULL);
out2:
	unregister_trace_android_vh_add_page_to_lrulist(add_page_to_lrulist, NULL);
out1:
	unregister_trace_android_vh_update_page_mapcount(update_page_mapcount, NULL);
#endif
out:
	return ret;
}

static void unregister_mapped_protect_vendor_hooks(void)
{
#ifdef CONFIG_MAPPED_PROTECT_ALL
	unregister_trace_android_vh_update_page_mapcount(update_page_mapcount, NULL);
	unregister_trace_android_vh_add_page_to_lrulist(add_page_to_lrulist, NULL);
	unregister_trace_android_vh_del_page_from_lrulist(del_page_from_lrulist, NULL);
	unregister_trace_android_vh_page_should_be_protected(page_should_be_protect, NULL);
	unregister_trace_android_vh_mark_page_accessed(mark_page_accessed_handler, NULL);
	unregister_trace_android_vh_do_traversal_lruvec(do_traversal_lruvec, NULL);
	unregister_trace_android_vh_show_mapcount_pages(show_mapcount_pages, NULL);
#endif
	unregister_trace_android_vh_page_referenced_check_bypass(should_skip_page_referenced, NULL);
	return;
}

#ifdef CONFIG_MAPPED_PROTECT_ALL
static int mapcount_protect_show(struct seq_file *m, void *arg)
{
	atomic_long_set(&nr_mapped_multiple_debug[0], 0);
	atomic_long_set(&nr_mapped_multiple_debug[1], 0);
	do_traversal_all_lruvec();

	seq_printf(m,
		   "now_anon_nr_mapped:     %ld\n"
		   "now_file_nr_mapped:     %ld\n"
		   "nr_anon_mapped_multiple:     %ld\n"
		   "nr_file_mapped_multiple:     %ld\n"
		   "max_nr_anon_mapped_multiple:     %ld\n"
		   "max_nr_file_mapped_multiple:     %ld\n"
		   "nr_anon_mapped_high:     %lu\n"
		   "nr_file_mapped_high:     %lu\n"
		   "memavail_noprotected:     %lu\n",
		   nr_mapped_multiple_debug[0],
		   nr_mapped_multiple_debug[1],
		   nr_mapped_multiple[0],
		   nr_mapped_multiple[1],
		   max_nr_mapped_multiple[0],
		   max_nr_mapped_multiple[1],
		   mapcount_protected_high[0],
		   mapcount_protected_high[1],
		   memavail_noprotected);
	seq_putc(m, '\n');

	return 0;
}
#endif

static int __init mapped_protect_init(void)
{
	int ret = 0;
#ifdef CONFIG_MAPPED_PROTECT_ALL
	int retry = 0;
#endif

	ret = register_mapped_protect_vendor_hooks();
	if (ret != 0)
		return ret;
#ifdef CONFIG_MAPPED_PROTECT_ALL
	memavail_noprotected = totalram_pages() / 10;
	mapcount_protected_high[0] = memavail_noprotected >> 1;
	mapcount_protected_high[1] = memavail_noprotected >> 1;

	do_traversal_all_lruvec();
	atomic_long_set(&nr_mapped_multiple[0], atomic_long_read(&nr_mapped_multiple_debug[0]));
	atomic_long_set(&nr_mapped_multiple[1], atomic_long_read(&nr_mapped_multiple_debug[1]));

retry_get_num_mapcpount:
	if (retry++ < RETRY_GET_MAPCOUNT) {
		atomic_long_set(&nr_mapped_multiple_debug[0], 0);
		atomic_long_set(&nr_mapped_multiple_debug[1], 0);

		do_traversal_all_lruvec();

		if (atomic_long_read(&nr_mapped_multiple[0]) !=
				atomic_long_read(&nr_mapped_multiple_debug[0]) ||
				atomic_long_read(&nr_mapped_multiple[1]) !=
				atomic_long_read(&nr_mapped_multiple_debug[1])) {
			atomic_long_set(&nr_mapped_multiple[0], atomic_long_read(&nr_mapped_multiple_debug[0]));
			atomic_long_set(&nr_mapped_multiple[1], atomic_long_read(&nr_mapped_multiple_debug[1]));
			goto retry_get_num_mapcpount;
		}
	}
	mapcount_protect_setup = true;
	proc_create_single("mapped_protect_show", 0, NULL, mapcount_protect_show);
#endif
	pr_info("mapped_protect_init succeed!\n");
	return 0;
}

static void __exit mapped_protect_exit(void)
{
	unregister_mapped_protect_vendor_hooks();
	remove_proc_entry("mapped_protect_show", NULL);
	pr_info("mapped_protect_exit exit succeed!\n");

	return;
}

module_init(mapped_protect_init);
module_exit(mapped_protect_exit);

MODULE_LICENSE("GPL v2");
