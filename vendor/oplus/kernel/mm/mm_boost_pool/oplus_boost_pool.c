// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "boostpool: " fmt

#include <asm/page.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/vmstat.h>
#include <linux/oom.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <uapi/linux/sched/types.h>

#include "oplus_boost_pool.h"

#define MAX_BOOST_POOL_HIGH (1024 * 256)

#define K(x) ((x) << (PAGE_SHIFT-10))
#define M(x) (K(x) >> 10)
#define PAGES(x) (x >> PAGE_SHIFT)

atomic64_t boost_pool_pages = ATOMIC64_INIT(0);

static LIST_HEAD(boost_pool_list);
static DEFINE_MUTEX(boost_pool_list_lock);

#define DEFINE_BOOST_POOL_PROC_RW_ATTRIBUTE(__name)			\
static int __name ## _open(struct inode *inode, struct file *file)	\
{									\
	struct dynamic_boost_pool *data = PDE_DATA(inode);		\
	return single_open(file, __name ## _show, data);		\
}									\
									\
static const struct proc_ops __name ## _proc_ops = {			\
	.proc_open	= __name ## _open,				\
	.proc_read	= seq_read,					\
	.proc_write	= __name ## _write,				\
	.proc_lseek	= seq_lseek,					\
	.proc_release	= single_release,				\
}

/*
 * The selection of the orders used for allocation (1MB, 64K, 4K) is designed
 * to match with the sizes often found in IOMMUs. Using order 4 pages instead
 * of order 0 pages can significantly improve the performance of many IOMMUs
 * by reducing TLB pressure and time spent updating page tables.
 */
static bool boost_pool_enable = true;

void boost_page_pool_add(struct dynamic_page_pool *pool, struct page *page)
{
	mutex_lock(&pool->mutex);
	if (PageHighMem(page)) {
		list_add_tail(&page->lru, &pool->high_items);
		pool->high_count++;
	} else {
		list_add_tail(&page->lru, &pool->low_items);
		pool->low_count++;
	}

	atomic_inc(&pool->count);
	atomic64_add(1 << pool->order, &boost_pool_pages);
	mutex_unlock(&pool->mutex);
}

struct page *boost_page_pool_remove(struct dynamic_page_pool *pool, bool high)
{
	struct page *page;

	if (high) {
		BUG_ON(!pool->high_count);
		page = list_first_entry(&pool->high_items, struct page, lru);
		pool->high_count--;
	} else {
		BUG_ON(!pool->low_count);
		page = list_first_entry(&pool->low_items, struct page, lru);
		pool->low_count--;
	}

	atomic_dec(&pool->count);
	list_del(&page->lru);
	atomic64_sub(1 << pool->order, &boost_pool_pages);
	return page;
}

static void boost_page_pool_free(struct dynamic_page_pool *pool, struct page *page)
{
	BUG_ON(pool->order != compound_order(page));

	boost_page_pool_add(pool, page);
}

static struct dynamic_page_pool *dynamic_page_pool_create_new(gfp_t gfp_mask, unsigned int order)
{
	struct dynamic_page_pool *pool = kmalloc(sizeof(*pool), GFP_KERNEL);

	if (!pool)
		return NULL;
	pool->high_count = 0;
	pool->low_count = 0;
	INIT_LIST_HEAD(&pool->low_items);
	INIT_LIST_HEAD(&pool->high_items);
	pool->gfp_mask = gfp_mask | __GFP_COMP;
	pool->order = order;
	mutex_init(&pool->mutex);

	return pool;
}

static void dynamic_page_pool_destroy_new(struct dynamic_page_pool *pool)
{
	struct page *page, *tmp;
	LIST_HEAD(pages);
	int num_pages = 0;
	int ret = DYNAMIC_POOL_SUCCESS;

	mutex_lock(&pool->mutex);
	while (true) {
		if (pool->low_count)
			page = boost_page_pool_remove(pool, false);
		else if (pool->high_count)
			page = boost_page_pool_remove(pool, true);
		else
			break;

		list_add(&page->lru, &pages);
		num_pages++;
	}
	mutex_unlock(&pool->mutex);

	if (num_pages && pool->prerelease_callback)
		ret = pool->prerelease_callback(pool, &pages, num_pages);

	if (ret != DYNAMIC_POOL_SUCCESS) {
		pr_err("Failed to reclaim pages when destroying the pool!\n");
		return;
	}

	list_for_each_entry_safe(page, tmp, &pages, lru) {
		list_del(&page->lru);
		__free_pages(page, pool->order);
	}

	kfree(pool);
}

static struct dynamic_page_pool **dynamic_page_pool_create_pools_new(int vmid,
							  prerelease_callback callback)
{
	struct dynamic_page_pool **pool_list;
	int i;
	int ret;

	pool_list = kmalloc_array(NUM_ORDERS, sizeof(*pool_list), GFP_KERNEL);
	if (!pool_list)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < NUM_ORDERS; i++) {
		pool_list[i] = dynamic_page_pool_create_new(order_flags[i],
							orders[i]);
		pool_list[i]->vmid = vmid;
		pool_list[i]->prerelease_callback = callback;
		atomic_set(&pool_list[i]->count, 0);
		pool_list[i]->last_low_watermark_ktime = 0;

		if (IS_ERR_OR_NULL(pool_list[i])) {
			int j;

			pr_err("%s: page pool creation failed for the order %u pool!\n",
			       __func__, orders[i]);
			for (j = 0; j < i; j++)
				dynamic_page_pool_destroy_new(pool_list[j]);

			ret = -ENOMEM;
			goto free_pool_arr;
		}
	}

	return pool_list;

free_pool_arr:
	kfree(pool_list);

	return ERR_PTR(ret);
}

static void dynamic_page_pool_release_pools_new(struct dynamic_page_pool **pool_list)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++)
		dynamic_page_pool_destroy_new(pool_list[i]);

	kfree(pool_list);
}

static inline unsigned int order_to_size(int order)
{
	return PAGE_SIZE << order;
}

static int dynamic_boost_pool_nr_pages(struct dynamic_boost_pool *pool)
{
	int i;
	int count = 0;

	if (unlikely(NULL == pool)) {
		pr_err("%s: pool is NULL!\n", __func__);
		return 0;
	}

	for (i = 0; i < NUM_ORDERS; i++)
		count += dynamic_page_pool_total(pool->pools[i], 1);

	return count;
}

static int dynamic_boost_page_pool_refill(struct dynamic_page_pool *pool)
{
	struct page *page;
	gfp_t gfp_refill = pool->gfp_mask;

	if (NULL == pool) {
		pr_err("%s: pool is NULL!\n", __func__);
		return -ENOENT;
	}

	page = alloc_pages(gfp_refill, pool->order);
	if (NULL == page)
		return -ENOMEM;

	boost_page_pool_free(pool, page);
	return 0;
}

static int dynamic_boost_pool_kworkthread(void *p)
{
	int i;
	struct dynamic_boost_pool *boost_pool;
	int ret;

	if (NULL == p) {
		pr_err("%s: p is NULL!\n", __func__);
		return 0;
	}

	boost_pool = (struct dynamic_boost_pool *)p;

	while (true) {
		ret = wait_event_interruptible(boost_pool->waitq,
					       (boost_pool->wait_flag == 1));
		if (ret < 0)
			continue;

		boost_pool->wait_flag = 0;

		for (i = 0; i < NUM_ORDERS; i++) {
			while (!boost_pool->force_stop && dynamic_boost_pool_nr_pages(boost_pool) < boost_pool->low) {
				if (dynamic_boost_page_pool_refill(boost_pool->pools[i]) < 0)
					break;
			}
		}
	}

	return 0;
}

/* extern int direct_vm_swappiness; */
static int dynamic_boost_pool_prefill_kworkthread(void *p)
{
	int i;
	struct dynamic_boost_pool *pool;
	u64 timeout_jiffies;
	int ret;
	unsigned long begin;

	if (NULL == p) {
		pr_err("%s: p is NULL!\n", __func__);
		return 0;
	}

	pool = (struct dynamic_boost_pool *)p;
	while (true) {
		ret = wait_event_interruptible(pool->prefill_waitq,
					       (pool->prefill_wait_flag == 1));
		if (ret < 0)
			continue;

		pool->prefill_wait_flag = 0;

		mutex_lock(&pool->prefill_mutex);
		timeout_jiffies = get_jiffies_64() + 2 * HZ;
		/* direct_vm_swappiness = 20; */
		begin = jiffies;

		pr_info("prefill start >>>>> nr_page: %dMib high: %dMib.\n",
			M(dynamic_boost_pool_nr_pages(pool)), M(pool->high));

		for (i = 0; i < NUM_ORDERS; i++) {
			while (!pool->force_stop && dynamic_boost_pool_nr_pages(pool) < pool->high) {
				/* support timeout to limit alloc pages. */
				if (time_after64(get_jiffies_64(), timeout_jiffies)) {
					pr_warn("prefill timeout.\n");
					break;
				}

				if (dynamic_boost_page_pool_refill(pool->pools[i]) < 0)
					break;
			}
		}

		pr_info("prefill end <<<<< nr_page: %dMib high:%dMib use %dms\n",
			M(dynamic_boost_pool_nr_pages(pool)), M(pool->high),
			jiffies_to_msecs(jiffies - begin));

		pool->high = max(dynamic_boost_pool_nr_pages(pool), pool->low);
		pool->prefill = false;
		/* direct_vm_swappiness = 60; */
		mutex_unlock(&pool->prefill_mutex);
	}

	return 0;
}

static struct page *dynamic_boost_pool_alloc(struct dynamic_boost_pool *pool,
				      unsigned long size,
				      unsigned int max_order)
{
	int i;
	struct page *page = NULL;

	if (NULL == pool) {
		pr_err("%s: pool is NULL!\n", __func__);
		return NULL;
	}

	for (i = 0; i < NUM_ORDERS; i++) {
		if (size < order_to_size(orders[i]))
			continue;
		if (max_order < orders[i])
			continue;

		mutex_lock(&pool->pools[i]->mutex);
		if (pool->pools[i]->high_count)
			page = boost_page_pool_remove(pool->pools[i], true);
		else if (pool->pools[i]->low_count)
			page = boost_page_pool_remove(pool->pools[i], false);
		mutex_unlock(&pool->pools[i]->mutex);

		if (!page)
			continue;
		return page;
	}
	return NULL;
}

static void dynamic_boost_pool_dec_high(struct dynamic_boost_pool *pool, int nr_pages)
{
	if (pool->prefill)
		return;

	if (unlikely(nr_pages < 0))
		return;

	pool->high = max(pool->low, pool->high - nr_pages);

	return;
}

void dynamic_boost_pool_alloc_pack(struct dynamic_boost_pool *boost_pool, unsigned long *size_remaining_p,
					unsigned int *max_order_p, struct list_head *pages_p, int *i_p)
{
	struct page *page;
	unsigned long alloc_sz = 0;

	if (boost_pool == NULL)
		return;

	if (boost_pool->camera_pid && current->tgid != boost_pool->camera_pid &&
	    dynamic_boost_pool_nr_pages(boost_pool) < boost_pool->camera_pages)
		return;

	while (*size_remaining_p > 0) {
		/*
		 * Avoid trying to allocate memory if the process
		* has been killed by SIGKILL
		*/
		if (fatal_signal_pending(current))
			return;

		page = dynamic_boost_pool_alloc(boost_pool,
						*size_remaining_p, *max_order_p);
		if (!page)
			break;

		list_add_tail(&page->lru, pages_p);
		*size_remaining_p -= page_size(page);
		alloc_sz += page_size(page);
		*max_order_p = compound_order(page);
		(*i_p)++;
	}

	dynamic_boost_pool_dec_high(boost_pool, alloc_sz >> PAGE_SHIFT);
	*max_order_p = orders[0];
}
EXPORT_SYMBOL_GPL(dynamic_boost_pool_alloc_pack);

static void dynamic_boost_pool_wakeup_process(struct dynamic_boost_pool *pool)
{
	if (!boost_pool_enable)
		return;

	if (NULL == pool) {
		pr_err("%s: boost_pool is NULL!\n", __func__);
		return;
	}

	/* if set force_stop, we can get page from ion_free instead of alloc pages */
	/* from system. it can reduce the system loading */
	if (!pool->force_stop && !pool->prefill) {
		pool->wait_flag = 1;
		wake_up_interruptible(&pool->waitq);
	}
}

static int dynamic_page_pool_do_shrink(struct dynamic_page_pool *pool, gfp_t gfp_mask,
				       int nr_to_scan)
{
	int freed = 0;
	bool high;
	struct page *page, *tmp;
	LIST_HEAD(pages);
	int ret = DYNAMIC_POOL_SUCCESS;

	if (current_is_kswapd())
		high = true;
	else
		high = !!(gfp_mask & __GFP_HIGHMEM);

	if (nr_to_scan == 0)
		return dynamic_page_pool_total(pool, high);

	while (freed < nr_to_scan) {
		mutex_lock(&pool->mutex);
		if (pool->low_count) {
			page = boost_page_pool_remove(pool, false);
		} else if (high && pool->high_count) {
			page = boost_page_pool_remove(pool, true);
		} else {
			mutex_unlock(&pool->mutex);
			break;
		}
		mutex_unlock(&pool->mutex);
		list_add(&page->lru, &pages);
		freed += (1 << pool->order);
	}

	if (freed && pool->prerelease_callback)
		ret = pool->prerelease_callback(pool, &pages, freed >> pool->order);

	if (ret != DYNAMIC_POOL_SUCCESS) {
		pr_err("Failed to reclaim secure page pool pages!\n");
		return 0;
	}

	list_for_each_entry_safe(page, tmp, &pages, lru) {
		list_del(&page->lru);
		__free_pages(page, pool->order);
	}

	return freed;
}

static void dynamic_boost_pool_all_free(struct dynamic_boost_pool *pool, gfp_t gfp_mask,
				int nr_to_scan)
{
	int i;

	if (NULL == pool) {
		pr_err("%s: boost_pool is NULL!\n", __func__);
		return;
	}

	for (i = 0; i < NUM_ORDERS; i++)
		dynamic_page_pool_do_shrink(pool->pools[i], gfp_mask, nr_to_scan);
}

int dynamic_boost_pool_free(struct dynamic_boost_pool *pool, struct page *page,
		    int index)
{
	if (!boost_pool_enable) {
		dynamic_boost_pool_all_free(pool, __GFP_HIGHMEM, MAX_BOOST_POOL_HIGH);
		return -1;
	}

	if ((NULL == pool) || (NULL == page)) {
		pr_err("%s: pool/page is NULL!\n", __func__);
		return -1;
	}

	if (dynamic_boost_pool_nr_pages(pool) > pool->low)
		return -1;

	boost_page_pool_free(pool->pools[index], page);
	return 0;
}

static int dynamic_boost_pool_do_shrink(struct dynamic_boost_pool *boost_pool,
				gfp_t gfp_mask, int nr_to_scan)
{
	int nr_max_free;
	int nr_to_free;
	int nr_freed;
	int nr_total = 0;
	int only_scan = 0;
	int i;

	if (NULL == boost_pool) {
		pr_err("%s: boostpool is NULL!\n", __func__);
		return 0;
	}

	if (boost_pool->tsk->pid == current->pid ||
	    boost_pool->prefill_tsk->pid == current->pid)
		return 0;

	if (!nr_to_scan) {
		only_scan = 1;
	} else {
		nr_max_free = dynamic_boost_pool_nr_pages(boost_pool) -
			(boost_pool->high + LOWORDER_WATER_MASK);
		nr_to_free = min(nr_max_free, nr_to_scan);
		if (nr_to_free <= 0)
			return 0;
	}

	for (i = 0; i < NUM_ORDERS; i++) {
		if (only_scan) {
			nr_total += dynamic_page_pool_do_shrink(boost_pool->pools[i],
								gfp_mask, nr_to_scan);
		} else {
			nr_freed = dynamic_page_pool_do_shrink(boost_pool->pools[i],
							       gfp_mask, nr_to_free);
			nr_to_free -= nr_freed;
			nr_total += nr_freed;
			if (nr_to_free <= 0)
				break;
		}
	}

	return nr_total;
}

static int dynamic_boost_pool_shrink(gfp_t gfp_mask, int nr_to_scan)
{
	struct dynamic_boost_pool *boost_pool;
	int nr_total = 0;
	int nr_freed;
	int only_scan = 0;

	if (!mutex_trylock(&boost_pool_list_lock))
		return 0;

	if (!nr_to_scan)
		only_scan = 1;

	list_for_each_entry(boost_pool, &boost_pool_list, list) {
		if (only_scan) {
			nr_total += dynamic_boost_pool_do_shrink(boost_pool,
								 gfp_mask,
								 nr_to_scan);
		} else {
			nr_freed = dynamic_boost_pool_do_shrink(boost_pool,
								gfp_mask,
								nr_to_scan);
			nr_to_scan -= nr_freed;
			nr_total += nr_freed;
			if (nr_to_scan <= 0)
				break;
		}
	}
	mutex_unlock(&boost_pool_list_lock);

	return nr_total;
}

static unsigned long dynamic_boost_pool_shrink_count(struct shrinker *shrinker,
						    struct shrink_control *sc)
{
	return dynamic_boost_pool_shrink(sc->gfp_mask, 0);
}

static unsigned long dynamic_boost_pool_shrink_scan(struct shrinker *shrinker,
						   struct shrink_control *sc)
{
	int to_scan = sc->nr_to_scan;

	if (to_scan == 0)
		return 0;

	return dynamic_boost_pool_shrink(sc->gfp_mask, to_scan);
}

static void dynamic_boost_pool_destroy(struct dynamic_boost_pool *pool)
{
	mutex_lock(&boost_pool_list_lock);
	list_del(&pool->list);
	mutex_unlock(&boost_pool_list_lock);

	dynamic_page_pool_release_pools_new(pool->pools);
	return;
}

static int dynamic_boost_pool_proc_show(struct seq_file *s, void *v)
{
	struct dynamic_boost_pool *boost_pool = s->private;
	int i;

	seq_printf(s, "Name:%s: %dMib, prefill: %d origin: %dMib low: %dMib high: %dMib\n",
		   boost_pool->name,
		   M(dynamic_boost_pool_nr_pages(boost_pool)),
		   boost_pool->prefill,
		   M(boost_pool->origin),
		   M(boost_pool->low),
		   M(boost_pool->high));

	for (i = 0; i < NUM_ORDERS; i++) {
		struct dynamic_page_pool *pool = boost_pool->pools[i];

		seq_printf(s, "%d order %u highmem pages in boost pool = %lu total\n",
			   pool->high_count, pool->order,
			   (PAGE_SIZE << pool->order) * pool->high_count);
		seq_printf(s, "%d order %u lowmem pages in boost pool = %lu total\n",
			   pool->low_count, pool->order,
			   (PAGE_SIZE << pool->order) * pool->low_count);
	}
	return 0;
}

static int dynamic_boost_pool_proc_open(struct inode *inode, struct file *file)
{
	struct dynamic_boost_pool *data = PDE_DATA(inode);
	return single_open(file, dynamic_boost_pool_proc_show, data);
}

static ssize_t dynamic_boost_pool_proc_write(struct file *file,
				     const char __user *buf,
				     size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, nr_pages;
	struct dynamic_boost_pool *boost_pool = PDE_DATA(file_inode(file));

	if (IS_ERR_OR_NULL(boost_pool)) {
		pr_err("%s: boost pool is NULL.\n");
		return -EFAULT;
	}

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtoint(strstrip(buffer), 0, &nr_pages);
	if(err)
		return err;

	if (nr_pages == 0) {
		pr_info("%s: reset flag.\n", current->comm);
		boost_pool->high = boost_pool->low = boost_pool->origin;
		boost_pool->force_stop = false;
		return count;
	}

	if (nr_pages == -1) {
		pr_info("%s: force stop.\n", current->comm);
		boost_pool->force_stop = true;
		return count;
	}

	if (nr_pages < 0 || nr_pages >= MAX_BOOST_POOL_HIGH ||
	    nr_pages <= boost_pool->low)
		return -EINVAL;

	if (mutex_trylock(&boost_pool->prefill_mutex)) {
		long mem_avail = si_mem_available();

		pr_info("%s: set high wm => %dMib. current avail => %dMib\n",
			current->comm, M(nr_pages), M(mem_avail));

		boost_pool->prefill = true;
		boost_pool->force_stop = false;
		boost_pool->high = nr_pages;

		boost_pool->prefill_wait_flag = 1;
		wake_up_interruptible(&boost_pool->prefill_waitq);
		mutex_unlock(&boost_pool->prefill_mutex);
	} else {
		pr_err("%s: prefill already running. \n");
		return -EBUSY;
	}

	return count;
}

static const struct proc_ops dynamic_boost_pool_proc_ops = {
	.proc_open	= dynamic_boost_pool_proc_open,
	.proc_read	= seq_read,
	.proc_write	= dynamic_boost_pool_proc_write,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int dynamic_boost_pool_low_proc_show(struct seq_file *s, void *v)
{
	struct dynamic_boost_pool *boost_pool = s->private;

	seq_printf(s, "low %dMib.\n", M(boost_pool->camera_pages));

	return 0;
}

static int dynamic_boost_pool_pages_proc_open(struct inode *inode, struct file *file)
{
	struct dynamic_boost_pool *data = PDE_DATA(inode);
	return single_open(file, dynamic_boost_pool_low_proc_show, data);
}

static ssize_t dynamic_boost_pool_pages_proc_write(struct file *file,
						 const char __user *buf,
						 size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, nr_pages;
	struct dynamic_boost_pool *boost_pool = PDE_DATA(file_inode(file));

	if (IS_ERR_OR_NULL(boost_pool)) {
		pr_err("%s: boost pool is NULL.\n");
		return -EFAULT;
	}

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtoint(strstrip(buffer), 0, &nr_pages);
	if(err)
		return err;

	if (nr_pages <= 0 || nr_pages >= MAX_BOOST_POOL_HIGH)
		return -EINVAL;

	boost_pool->camera_pages = nr_pages;
	boost_pool->origin = boost_pool->high = boost_pool->low =
		boost_pool->sf_pages + boost_pool->camera_pages;
	dynamic_boost_pool_wakeup_process(boost_pool);
	return count;
}

static const struct proc_ops dynamic_boost_pool_pages_proc_ops = {
	.proc_open	= dynamic_boost_pool_pages_proc_open,
	.proc_read	= seq_read,
	.proc_write	= dynamic_boost_pool_pages_proc_write,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int dynamic_boost_pool_stat_proc_show(struct seq_file *s, void *v)
{
	struct dynamic_boost_pool *boost_pool = s->private;

	seq_printf(s, "%d,%d,%ld\n",
		   M(dynamic_boost_pool_nr_pages(boost_pool)),
		   boost_pool->prefill, M(si_mem_available()));

	return 0;
}

static int dynamic_boost_pool_stat_proc_open(struct inode *inode, struct file *file)
{
	struct dynamic_boost_pool *data = PDE_DATA(inode);
	return single_open(file, dynamic_boost_pool_stat_proc_show, data);
}

static const struct proc_ops dynamic_boost_pool_stat_proc_ops = {
	.proc_open	= dynamic_boost_pool_stat_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static struct shrinker boost_pool_shrinker = {
	.count_objects = dynamic_boost_pool_shrink_count,
	.scan_objects = dynamic_boost_pool_shrink_scan,
	.seeks = DEFAULT_SEEKS,
	.batch = 0,
};

static int dynamic_boost_pool_init_shrinker(void)
{
	int ret;
	static bool registered;

	if (registered)
		return 0;

	ret = register_shrinker(&boost_pool_shrinker);
	if (ret)
		return ret;

	registered = true;
	return 0;
}

static inline void set_cpumask(int end_cpu, struct cpumask *mask)
{
	int i;

	cpumask_clear(mask);
	for (i = 0; i <= end_cpu; i++)
		cpumask_set_cpu(i, mask);
}

/* limit max cpu here. in gki kernel CONFIG_NR_CPUS=32. */
#define MAX_SUPPORT_CPUS (8)
static ssize_t cpu_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, cpu, i;
	struct cpumask cpu_mask = { CPU_BITS_NONE };
	struct dynamic_boost_pool *boost_pool = PDE_DATA(file_inode(file));

	if (boost_pool == NULL)
		return -EFAULT;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;
	err = kstrtoint(strstrip(buffer), 0, &cpu);
	if (err)
		return err;

	if (cpu < 0 || cpu >= MAX_SUPPORT_CPUS)
		return -EINVAL;

	for (i = 0; i <= cpu; i++)
		cpumask_set_cpu(i, &cpu_mask);

	set_cpus_allowed_ptr(boost_pool->prefill_tsk, &cpu_mask);

	pr_info("%s:%d set %s cpu [0-%d]\n",
		current->comm, current->tgid,
		boost_pool->prefill_tsk->comm, cpu);
	return count;
}

static int cpu_show(struct seq_file *s, void *unused)
{
	struct dynamic_boost_pool *boost_pool = s->private;

	seq_printf(s, "%*pbl\n",
		   cpumask_pr_args(boost_pool->prefill_tsk->cpus_ptr));
	return 0;
}
DEFINE_BOOST_POOL_PROC_RW_ATTRIBUTE(cpu);

static ssize_t camera_pid_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	char buffer[13];
	struct task_struct *task;
	int err, pid;
	struct dynamic_boost_pool *boost_pool = PDE_DATA(file_inode(file));

	if (boost_pool == NULL)
		return -EFAULT;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;
	err = kstrtoint(strstrip(buffer), 0, &pid);
	if (err)
		return err;

	if (pid == 0) {
		boost_pool->camera_pid = 0;
		pr_info("reset camera_pid\n");
		return count;
	}

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (task != NULL) {
		pr_info("%s:%d set camera_pid %s:%d\n",
			current->comm, current->tgid,
			task->comm, task->tgid);
		boost_pool->camera_pid = task->tgid;
	}
	rcu_read_unlock();

	if (!task)
		return -EINVAL;

	return count;
}

static int camera_pid_show(struct seq_file *s, void *unused)
{
	struct dynamic_boost_pool *boost_pool = s->private;

	seq_printf(s, "%d\n", boost_pool->camera_pid);
	return 0;
}
DEFINE_BOOST_POOL_PROC_RW_ATTRIBUTE(camera_pid);

static struct dynamic_boost_pool *dynamic_boost_pool_create(int sf_pages,
							    int camera_pages,
							    struct proc_dir_entry *root_dir,
							    char *name)
{
	struct task_struct *tsk;
	struct dynamic_boost_pool *boost_pool;
	char buf[128];
	struct cpumask mask;
	int end_cpu = 3;
	int ret = 0;
	struct proc_dir_entry *proc_info, *proc_camera_pages, *proc_stat, *proc_pid, *proc_cpu;

	if (NULL == root_dir) {
		pr_err("%s: boost_pool dir not exits.\n", __func__);
		return NULL;
	}

	ret = dynamic_boost_pool_init_shrinker();
	if (ret) {
		pr_err("%s: boost_pool init shrinker failed.\n", __func__);
		return NULL;
	}

	boost_pool = kzalloc(sizeof(struct dynamic_boost_pool) +
			     sizeof(struct dynamic_page_pool *) * NUM_ORDERS,
			     GFP_KERNEL);

	if (NULL == boost_pool) {
		pr_err("%s: boost_pool is NULL!\n", __func__);
		return NULL;
	}

	boost_pool->pools = dynamic_page_pool_create_pools_new(0, NULL);

	boost_pool->sf_pages = sf_pages;
	boost_pool->camera_pages = camera_pages;
	boost_pool->origin = boost_pool->high = boost_pool->low =
		boost_pool->sf_pages + boost_pool->camera_pages;

	boost_pool->name = name;

	proc_info = proc_create_data(name, 0666,
				     root_dir,
				     &dynamic_boost_pool_proc_ops,
				     boost_pool);
	if (IS_ERR_OR_NULL(proc_info)) {
		pr_err("Unable to initialise /proc/boost_pool/%s\n",
			name);
		goto destroy_pools;
	}

	snprintf(buf, 128, "%s_pages", name);
	proc_camera_pages = proc_create_data(buf, 0666, root_dir,
					     &dynamic_boost_pool_pages_proc_ops,
					     boost_pool);
	if (IS_ERR_OR_NULL(proc_camera_pages)) {
		pr_err("Unable to initialise /proc/boost_pool/%s_low\n",
			name);
		goto destroy_proc_info;
	}

	snprintf(buf, 128, "%s_stat", name);
	proc_stat = proc_create_data(buf, 0444,
						 root_dir,
						 &dynamic_boost_pool_stat_proc_ops,
						 boost_pool);
	if (IS_ERR_OR_NULL(proc_stat)) {
		pr_info("Unable to initialise /proc/boost_pool/%s_stat\n",
			name);
		goto destroy_proc_camera_pages;
	}

	snprintf(buf, 128, "%s_cpu", name);
	proc_cpu = proc_create_data(buf, 0666, root_dir, &cpu_proc_ops,
				    boost_pool);
	if (!proc_cpu) {
		pr_err("create proc_fs cpu failed\n");
		goto destroy_proc_stat;
	}

	snprintf(buf, 128, "%s_pid", name);
	proc_pid = proc_create_data(buf, 0666, root_dir, &camera_pid_proc_ops,
				    boost_pool);
	if (!proc_pid) {
		pr_err("create proc_fs cpu failed\n");
		goto destroy_proc_cpu;
	}

	init_waitqueue_head(&boost_pool->waitq);
	tsk = kthread_run(dynamic_boost_pool_kworkthread, boost_pool,
			  "bp_%s", name);

	if (IS_ERR_OR_NULL(tsk)) {
		pr_err("%s: kthread_create failed!\n", __func__);
		goto destroy_proc_pid;
	}
	boost_pool->tsk = tsk;
	/* FIXME, we should not use magic number.. */
	set_cpumask(end_cpu, &mask);
	set_cpus_allowed_ptr(tsk, &mask);

	mutex_init(&boost_pool->prefill_mutex);
	init_waitqueue_head(&boost_pool->prefill_waitq);
	tsk = kthread_run(dynamic_boost_pool_prefill_kworkthread, boost_pool,
			  "bp_prefill_%s", name);
	if (IS_ERR_OR_NULL(tsk)) {
		pr_err("%s: kthread_create failed!\n", __func__);
		goto destroy_proc_stat;
	}
	boost_pool->prefill_tsk = tsk;
	set_cpus_allowed_ptr(tsk, &mask);

	dynamic_boost_pool_wakeup_process(boost_pool);

	mutex_lock(&boost_pool_list_lock);
	list_add(&boost_pool->list, &boost_pool_list);
	mutex_unlock(&boost_pool_list_lock);
	return boost_pool;

destroy_proc_pid:
	proc_remove(proc_pid);
destroy_proc_cpu:
	proc_remove(proc_cpu);
destroy_proc_stat:
	proc_remove(proc_stat);
destroy_proc_camera_pages:
	proc_remove(proc_camera_pages);
destroy_proc_info:
	proc_remove(proc_info);
destroy_pools:
	dynamic_boost_pool_destroy(boost_pool);

	kfree(boost_pool);
	boost_pool = NULL;
	return NULL;
}

struct dynamic_boost_pool *dynamic_boost_pool_create_pack(void)
{
	struct dynamic_boost_pool *boost_pool = NULL;
	struct proc_dir_entry *boost_root_dir;

	boost_root_dir = proc_mkdir("boost_pool", NULL);
	if (!IS_ERR_OR_NULL(boost_root_dir)) {
		int sf_pages = PAGES(SZ_32M), camera_pages = 0;

		/* SZ_4G is ULL */
		if (totalram_pages() > PAGES(SZ_4G)) {
			sf_pages = PAGES(SZ_64M);
			camera_pages = PAGES(SZ_128M);
		}

		boost_pool = dynamic_boost_pool_create(sf_pages, camera_pages,
						       boost_root_dir, "camera");
		if (NULL == boost_pool)
			pr_err("%s: create boost_pool camera failed!\n", __func__);
	} else {
		pr_err("%s: create boost_root_dir boost_pool failed.\n", __func__);
	}

	return boost_pool;
}
EXPORT_SYMBOL_GPL(dynamic_boost_pool_create_pack);
