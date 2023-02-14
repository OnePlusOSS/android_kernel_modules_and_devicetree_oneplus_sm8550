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

#define pr_fmt(fmt) "[boost_pool] "fmt

#include <linux/freezer.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched/signal.h>

#include "oplus_boost_pool_mtk.h"

#define CREATE_TRACE_POINTS
#include "trace_dma_buf.h"
EXPORT_TRACEPOINT_SYMBOL(dma_buf_alloc_start);
EXPORT_TRACEPOINT_SYMBOL(dma_buf_alloc_end);

/* this region must same as the system_heap */
#define LOW_ORDER_GFP (GFP_HIGHUSER | __GFP_ZERO | __GFP_COMP)
#define MID_ORDER_GFP (LOW_ORDER_GFP | __GFP_NOWARN)
#define HIGH_ORDER_GFP  (((GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN \
				| __GFP_NORETRY) & ~__GFP_RECLAIM) \
				| __GFP_COMP)
static gfp_t order_flags[] = {HIGH_ORDER_GFP, HIGH_ORDER_GFP, LOW_ORDER_GFP};
/*
 * The selection of the orders used for allocation (1MB, 64K, 4K) is designed
 * to match with the sizes often found in IOMMUs. Using order 4 pages instead
 * of order 0 pages can significantly improve the performance of many IOMMUs
 * by reducing TLB pressure and time spent updating page tables.
 */
static const unsigned int orders[] = {8, 4, 0};
#define NUM_ORDERS ARRAY_SIZE(orders)

static LIST_HEAD(pool_list);
static DEFINE_MUTEX(pool_list_lock);
static struct proc_dir_entry *procdir;

static bool boost_pool_enable = true;

#if PAGE_SHIFT < 20
#define P2M(pages)	((pages) >> (20 - PAGE_SHIFT))
#define M2P(mb)	((mb) << (20 - PAGE_SHIFT))
#else				/* PAGE_SHIFT > 20 */
#define P2M(pages)	((pages) << (PAGE_SHIFT - 20))
#define M2P(mb)	((mb) >> (PAGE_SHIFT - 20))
#endif

#define DEFINE_BOOST_POOL_PROC_SHOW_ATTRIBUTE(__name)			\
static int __name ## _open(struct inode *inode, struct file *file)	\
{									\
	struct boost_pool *data = PDE_DATA(inode);			\
	return single_open(file, __name ## _show, data);		\
}									\
									\
static const struct proc_ops __name ## _proc_ops = {			\
	.proc_open	= __name ## _open,				\
	.proc_read	= seq_read,					\
	.proc_lseek	= seq_lseek,					\
	.proc_release	= single_release,				\
}

#define DEFINE_BOOST_POOL_PROC_RW_ATTRIBUTE(__name)			\
static int __name ## _open(struct inode *inode, struct file *file)	\
{									\
	struct boost_pool *data = PDE_DATA(inode);			\
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

#define DEFINE_BOOST_POOL_MGR_PROC_RW_ATTRIBUTE(__name)			\
static int __name ## _open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, __name ## _show, inode->i_private);	\
}									\
									\
static const struct proc_ops __name ## _proc_ops = {			\
	.proc_open	= __name ## _open,				\
	.proc_read	= seq_read,					\
	.proc_write	= __name ## _write,				\
	.proc_lseek	= seq_lseek,					\
	.proc_release	= single_release,				\
}

int boost_pool_mgr_init(void);

static inline
struct page *boost_page_pool_alloc_pages(struct boost_page_pool *pool)
{
	if (fatal_signal_pending(current))
		return NULL;
	return alloc_pages(pool->gfp_mask, pool->order);
}

static inline void boost_page_pool_free_pages(struct boost_page_pool *pool,
					      struct page *page)
{
	__free_pages(page, pool->order);
}

static void boost_page_pool_add(struct boost_page_pool *pool, struct page *page)
{
	int index;

	if (PageHighMem(page))
		index = POOL_HIGHPAGE;
	else
		index = POOL_LOWPAGE;

	mutex_lock(&pool->mutex);
	list_add_tail(&page->lru, &pool->items[index]);
	pool->count[index]++;
	mod_node_page_state(page_pgdat(page), NR_KERNEL_MISC_RECLAIMABLE,
			    1 << pool->order);
	mutex_unlock(&pool->mutex);
}

static struct page *boost_page_pool_remove(struct boost_page_pool *pool, int index)
{
	struct page *page;

	mutex_lock(&pool->mutex);
	page = list_first_entry_or_null(&pool->items[index], struct page, lru);
	if (page) {
		pool->count[index]--;
		list_del(&page->lru);
		mod_node_page_state(page_pgdat(page), NR_KERNEL_MISC_RECLAIMABLE,
				    -(1 << pool->order));
	}
	mutex_unlock(&pool->mutex);

	return page;
}

struct page *boost_pool_fetch(struct boost_pool *boost_pool, int order_index)
{
	struct page *page;
	struct boost_page_pool *pool = boost_pool->pools[order_index];

	if (boost_pool->custom_pid && current->tgid != boost_pool->custom_pid)
		return NULL;

	page = boost_page_pool_remove(pool, POOL_HIGHPAGE);
	if (!page)
		page = boost_page_pool_remove(pool, POOL_LOWPAGE);

	if (page && likely(!boost_pool->prefill))
		boost_pool->high = max(boost_pool->low,
				       boost_pool->high - (1 << pool->order));

	return page;
}
EXPORT_SYMBOL_GPL(boost_pool_fetch);

static void boost_page_pool_free(struct boost_page_pool *pool, struct page *page)
{
	if (WARN_ON(pool->order != compound_order(page)))
		return;

	boost_page_pool_add(pool, page);
}

static int boost_page_pool_total(struct boost_page_pool *pool, bool high)
{
	int count = pool->count[POOL_LOWPAGE];

	if (high)
		count += pool->count[POOL_HIGHPAGE];

	return count << pool->order;
}

static int boost_page_pool_prefill(struct boost_page_pool *pool)
{
	struct page *page;

	page = alloc_pages(pool->gfp_mask, pool->order);
	if (page == NULL)
		return -ENOMEM;

	boost_page_pool_free(pool, page);

	return 0;
}

static struct boost_page_pool *boost_page_pool_create(gfp_t gfp_mask, unsigned int order)
{
	struct boost_page_pool *pool = kmalloc(sizeof(*pool), GFP_KERNEL);
	int i;

	if (!pool)
		return NULL;

	for (i = 0; i < POOL_TYPE_SIZE; i++) {
		pool->count[i] = 0;
		INIT_LIST_HEAD(&pool->items[i]);
	}
	pool->gfp_mask = gfp_mask | __GFP_COMP;
	pool->order = order;
	mutex_init(&pool->mutex);

	return pool;
}

void boost_page_pool_destroy(struct boost_page_pool *pool)
{
	struct page *page;
	int i;

	/* Free any remaining pages in the pool */
	for (i = 0; i < POOL_TYPE_SIZE; i++) {
		while ((page = boost_page_pool_remove(pool, i)))
			boost_page_pool_free_pages(pool, page);
	}

	kfree(pool);
}

static int boost_page_pool_do_shrink(struct boost_page_pool *pool,
				     gfp_t gfp_mask, int nr_to_scan)
{
	int freed = 0;
	bool high;

	if (current_is_kswapd())
		high = true;
	else
		high = !!(gfp_mask & __GFP_HIGHMEM);

	if (nr_to_scan == 0)
		return boost_page_pool_total(pool, high);

	while (freed < nr_to_scan) {
		struct page *page;

		/* Try to free low pages first */
		page = boost_page_pool_remove(pool, POOL_LOWPAGE);
		if (!page)
			page = boost_page_pool_remove(pool, POOL_HIGHPAGE);

		if (!page)
			break;

		boost_page_pool_free_pages(pool, page);
		freed += (1 << pool->order);
	}

	return freed;
}

static void boost_pool_flush_all(struct boost_pool *boost_pool, gfp_t gfp_mask,
				 int nr_to_scan)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++)
		boost_page_pool_do_shrink(boost_pool->pools[i],
					  gfp_mask,
					  nr_to_scan);
}

static int boost_pool_nr_pages(struct boost_pool *boost_pool)
{
	int i, count = 0;

	for (i = 0; i < NUM_ORDERS; i++)
		count += boost_page_pool_total(boost_pool->pools[i], true);
	return count;
}

static inline void boost_pool_reset_wmark(struct boost_pool *boost_pool)
{
	boost_pool->high = boost_pool->low = boost_pool->min;
}

int boost_pool_free(struct boost_pool *boost_pool, struct page *page,
		    int order_index)
{
	if (!boost_pool_enable) {
		boost_pool_flush_all(boost_pool, __GFP_HIGHMEM,
				     MAX_BOOST_POOL_HIGH);
		return -1;
	}

	if (boost_pool_nr_pages(boost_pool) < boost_pool->low + MAX_BOOST_POOL_CACHE) {
		boost_page_pool_free(boost_pool->pools[order_index], page);
		return 0;
	}
	return -1;
}
EXPORT_SYMBOL_GPL(boost_pool_free);

static int boost_pool_prefill_kthread(void *p)
{
	int i, ret;
	u64 timeout;
	struct boost_pool *boost_pool = (struct boost_pool *)p;

	while (true) {
		ret = wait_event_interruptible(boost_pool->waitq,
					       (boost_pool->wait_flag == 1));
		if (ret < 0)
			continue;

		boost_pool->wait_flag = 0;

		mutex_lock(&boost_pool->prefill_lock);
		boost_pool->prefill = true;
		boost_pool->stop = false;
		timeout = get_jiffies_64() + MAX_BOOST_POOL_TIMEOUT;

		pr_info("%s prefill start >>>>> nr_page: %dMib, high: %dMib\n",
			current->comm,
			P2M(boost_pool_nr_pages(boost_pool)),
			P2M(boost_pool->high));

		for (i = 0; i < NUM_ORDERS; i++) {
			while (!boost_pool->stop &&
			       boost_pool_nr_pages(boost_pool) <
			       boost_pool->high) {
				if (time_after64(get_jiffies_64(), timeout)) {
					pr_warn("prefill timeout\n");
					break;
				}

				if (boost_page_pool_prefill(boost_pool->pools[i]) < 0)
					break;
			}
		}

		pr_info("%s prefill  end <<<<< nr_page: %dMib high: %dMib\n",
			current->comm,
			P2M(boost_pool_nr_pages(boost_pool)),
			P2M(boost_pool->high));

		boost_pool->prefill = false;
		boost_pool->stop = true;
		boost_pool->high = max(boost_pool_nr_pages(boost_pool),
				       boost_pool->low);

		mutex_unlock(&boost_pool->prefill_lock);

	}
	return 0;
}

static void boost_pool_wakeup_prefill(struct boost_pool *boost_pool)
{
	boost_pool->wait_flag = 1;
	wake_up_interruptible(&boost_pool->waitq);
}

static ssize_t low_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, nr_pages, mib;
	struct boost_pool *boost_pool = PDE_DATA(file_inode(file));

	if (boost_pool == NULL)
		return -EFAULT;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;
	err = kstrtoint(strstrip(buffer), 0, &mib);
	if (err)
		return err;

	nr_pages = M2P(mib);
	if (nr_pages < 0 || nr_pages > MAX_BOOST_POOL_HIGH)
		return -EINVAL;

	pr_info("%s:%d set %s low %dMib\n",
		current->comm, current->tgid, boost_pool->prefill_task->comm,
		P2M(nr_pages));

	boost_pool->low = boost_pool->high = nr_pages;
	return count;
}

static int low_show(struct seq_file *s, void *unused)
{
	struct boost_pool *boost_pool = s->private;

	seq_printf(s, "%d\n", P2M(boost_pool->low));
	return 0;
}
DEFINE_BOOST_POOL_PROC_RW_ATTRIBUTE(low);

static ssize_t min_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, nr_pages, mib;
	struct boost_pool *boost_pool = PDE_DATA(file_inode(file));

	if (boost_pool == NULL)
		return -EFAULT;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;
	err = kstrtoint(strstrip(buffer), 0, &mib);
	if (err)
		return err;

	nr_pages = M2P(mib);
	if (nr_pages < 0 || nr_pages > MAX_BOOST_POOL_HIGH)
		return -EINVAL;

	pr_info("%s:%d set %s min %dMib\n",
		current->comm, current->tgid, boost_pool->prefill_task->comm,
		P2M(nr_pages));

	boost_pool->min = boost_pool->low = boost_pool->high = nr_pages;
	boost_pool_wakeup_prefill(boost_pool);
	return count;
}

static int min_show(struct seq_file *s, void *unused)
{
	struct boost_pool *boost_pool = s->private;

	seq_printf(s, "%d\n", P2M(boost_pool->min));
	return 0;
}
DEFINE_BOOST_POOL_PROC_RW_ATTRIBUTE(min);

/* limit max cpu here. in gki kernel CONFIG_NR_CPUS=32. */
#define MAX_SUPPORT_CPUS (8)
static ssize_t cpu_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, cpu, i;
	struct cpumask cpu_mask = { CPU_BITS_NONE };
	struct boost_pool *boost_pool = PDE_DATA(file_inode(file));

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

	set_cpus_allowed_ptr(boost_pool->prefill_task, &cpu_mask);

	pr_info("%s:%d set %s cpu [0-%d]\n",
		current->comm, current->tgid,
		boost_pool->prefill_task->comm, cpu);
	return count;
}

static int cpu_show(struct seq_file *s, void *unused)
{
	struct boost_pool *boost_pool = s->private;

	seq_printf(s, "%*pbl\n",
		   cpumask_pr_args(boost_pool->prefill_task->cpus_ptr));
	return 0;
}
DEFINE_BOOST_POOL_PROC_RW_ATTRIBUTE(cpu);

static ssize_t custom_pid_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	char buffer[13];
	struct task_struct *task;
	int err, pid;
	struct boost_pool *boost_pool = PDE_DATA(file_inode(file));

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

	rcu_read_lock();
	task = find_task_by_vpid(pid);
	if (task != NULL) {
		pr_info("%s:%d set custom_pid %s:%d\n",
			current->comm, current->tgid,
			task->comm, task->tgid);
		boost_pool->custom_pid = task->tgid;
	}
	rcu_read_unlock();

	if (!boost_pool->custom_pid)
		return -EINVAL;

	return count;
}

static int custom_pid_show(struct seq_file *s, void *unused)
{
	struct boost_pool *boost_pool = s->private;

	seq_printf(s, "%d\n", boost_pool->custom_pid);
	return 0;
}
DEFINE_BOOST_POOL_PROC_RW_ATTRIBUTE(custom_pid);

static ssize_t alloc_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, nr_pages, mib;
	struct boost_pool *boost_pool = PDE_DATA(file_inode(file));

	if (boost_pool == NULL)
		return -EFAULT;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;
	err = kstrtoint(strstrip(buffer), 0, &mib);
	if (err)
		return err;

	if (mib == CMD_BOOST_POOL_STOP) {
		pr_info("%s:%d stop %s\n", current->comm, current->tgid,
			boost_pool->prefill_task->comm);
		boost_pool->stop = true;
		return count;
	}

	if (mib == CMD_BOOST_POOL_RESET) {
		pr_info("%s:%d reset %s\n", current->comm, current->tgid,
			boost_pool->prefill_task->comm);
		boost_pool->stop = true;
		boost_pool_reset_wmark(boost_pool);
		return count;
	}

	nr_pages = M2P(mib);
	if (nr_pages < 0 || nr_pages > MAX_BOOST_POOL_HIGH ||
	    nr_pages <= boost_pool->low)
		return -EINVAL;

	if (mutex_trylock(&boost_pool->prefill_lock)) {
		pr_info("%s:%d alloc %s %dMib current:%d Mib mem_available: %luMib\n",
		current->comm, current->tgid, boost_pool->prefill_task->comm,
		P2M(nr_pages), P2M(boost_pool_nr_pages(boost_pool)),
		P2M(si_mem_available()));

		boost_pool->prefill = false;
		boost_pool->high = nr_pages;
		mutex_unlock(&boost_pool->prefill_lock);
		boost_pool_wakeup_prefill(boost_pool);
	} else {
		pr_err("prefill already working\n");
		return -EBUSY;
	}
	return count;
}

static int alloc_show(struct seq_file *s, void *unused)
{
	struct boost_pool *boost_pool = s->private;

	seq_printf(s, "%d,%d,%d\n",
		   P2M(boost_pool_nr_pages(boost_pool)),
		   boost_pool->prefill,
		   P2M(si_mem_available()));
	return 0;
}
DEFINE_BOOST_POOL_PROC_RW_ATTRIBUTE(alloc);

static void boost_pool_destroy_pools(struct boost_page_pool **pools)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++)
		if (pools[i])
			boost_page_pool_destroy(pools[i]);
}

static int boost_pool_create_pools(struct boost_page_pool **pools)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++) {
		struct boost_page_pool *pool;

		pool = boost_page_pool_create(order_flags[i], orders[i]);
		if (!pool)
			goto err_create_pool;
		pools[i] = pool;
	}
	return 0;

err_create_pool:
	boost_pool_destroy_pools(pools);
	return -ENOMEM;
}

static int boost_pool_shrink(struct boost_pool *boost_pool,
			     struct boost_page_pool *pool,
			     gfp_t gfp_mask, int nr_to_scan)
{
	int nr_max_free;
	int nr_to_free;

	if (nr_to_scan == 0)
		return boost_page_pool_do_shrink(pool, gfp_mask, 0);

	nr_max_free = boost_pool_nr_pages(boost_pool) - boost_pool->high;
	nr_to_free = min(nr_max_free, nr_to_scan);
	if (nr_to_free <= 0)
		return 0;

	return boost_page_pool_do_shrink(pool, gfp_mask, nr_to_free);
}

static int boost_pool_mgr_shrink(gfp_t gfp_mask, int nr_to_scan)
{
	struct boost_pool *boost_pool;
	struct boost_page_pool *pool;
	int i;
	int nr_total = 0;
	int nr_freed;
	int only_scan = 0;

	if (!mutex_trylock(&pool_list_lock))
		return 0;

	if (!nr_to_scan)
		only_scan = 1;

	list_for_each_entry(boost_pool, &pool_list, list) {
		/* do not shrink my self */
		if (current->pid == boost_pool->prefill_task->pid)
			continue;

		for (i = 0; i < NUM_ORDERS; i++) {
			pool = boost_pool->pools[i];

			if (only_scan) {
				nr_total += boost_pool_shrink(boost_pool, pool,
							      gfp_mask,
							      nr_to_scan);
			} else {
				nr_freed = boost_pool_shrink(boost_pool, pool,
							     gfp_mask,
							     nr_to_scan);
				nr_to_scan -= nr_freed;
				nr_total += nr_freed;
				if (nr_to_scan <= 0)
					goto unlock;
			}
		}
	}
unlock:
	mutex_unlock(&pool_list_lock);
	return nr_total;
}

static unsigned long boost_pool_mgr_shrink_count(struct shrinker *shrinker,
					     struct shrink_control *sc)
{
	return boost_pool_mgr_shrink(sc->gfp_mask, 0);
}

static unsigned long boost_pool_mgr_shrink_scan(struct shrinker *shrinker,
						  struct shrink_control *sc)
{
	if (sc->nr_to_scan == 0)
		return 0;
	return boost_pool_mgr_shrink(sc->gfp_mask, sc->nr_to_scan);
}

struct shrinker pool_shrinker = {
	.count_objects = boost_pool_mgr_shrink_count,
	.scan_objects = boost_pool_mgr_shrink_scan,
	.seeks = DEFAULT_SEEKS,
	.batch = 0,
};

struct boost_pool *boost_pool_create(const char *name)
{
	int ret, nr_pages;
	struct boost_pool *boost_pool;
	struct proc_dir_entry *proc_root;
	struct proc_dir_entry *proc_low, *proc_min, *proc_alloc,
			      *proc_cpu, *proc_custom_pid;

	ret = boost_pool_mgr_init();
	if (ret)
		return NULL;

	boost_pool = kzalloc(sizeof(struct boost_pool) +
			     sizeof(struct ion_page_pool *) * NUM_ORDERS,
			     GFP_KERNEL);
	if (!boost_pool)
		return NULL;

	if (boost_pool_create_pools(boost_pool->pools))
		goto free_pool;

	proc_root = proc_mkdir(name, procdir);
	if (!proc_root) {
		pr_err("create proc_fs dir failed\n");
		goto destroy_pools;
	}

	proc_min = proc_create_data("min", 0666, proc_root, &min_proc_ops,
				    boost_pool);
	if (!proc_min) {
		pr_err("create proc_fs min failed\n");
		goto destroy_proc_root;
	}

	proc_low = proc_create_data("low", 0666, proc_root, &low_proc_ops,
				    boost_pool);
	if (!proc_low) {
		pr_err("create proc_fs low failed\n");
		goto destroy_proc_min;
	}

	proc_alloc = proc_create_data("alloc", 0666, proc_root,
				      &alloc_proc_ops, boost_pool);
	if (!proc_alloc) {
		pr_err("create proc_fs alloc failed\n");
		goto destroy_proc_low;
	}

	proc_cpu = proc_create_data("cpu", 0666, proc_root, &cpu_proc_ops,
				    boost_pool);
	if (!proc_cpu) {
		pr_err("create proc_fs cpu failed\n");
		goto destroy_proc_alloc;
	}

	proc_custom_pid = proc_create_data("custom_pid", 0666, proc_root,
					   &custom_pid_proc_ops, boost_pool);
	if (!proc_custom_pid) {
		pr_err("create proc_fs custom_pid failed\n");
		goto destroy_proc_cpu;
	}

	nr_pages = SZ_32M >> PAGE_SHIFT;
	boost_pool->min = nr_pages;
	boost_pool->low = nr_pages;
	boost_pool->high = nr_pages;

	mutex_init(&boost_pool->prefill_lock);
	init_waitqueue_head(&boost_pool->waitq);
	boost_pool->prefill_task = kthread_run(boost_pool_prefill_kthread,
					       boost_pool, "bp_%s", name);
	if (IS_ERR(boost_pool->prefill_task)) {
		pr_err("kthread run failed\n");
		goto destroy_proc_cpu;
	}

	mutex_lock(&pool_list_lock);
	list_add(&boost_pool->list, &pool_list);
	mutex_unlock(&pool_list_lock);
	boost_pool_wakeup_prefill(boost_pool);
	return boost_pool;

destroy_proc_cpu:
	proc_remove(proc_cpu);
destroy_proc_alloc:
	proc_remove(proc_alloc);
destroy_proc_low:
	proc_remove(proc_low);
destroy_proc_min:
	proc_remove(proc_min);
destroy_proc_root:
	proc_remove(proc_root);
destroy_pools:
	boost_pool_destroy_pools(boost_pool->pools);
free_pool:
	kfree(boost_pool);
	return NULL;
}
EXPORT_SYMBOL_GPL(boost_pool_create);


static int dump_show(struct seq_file *s, void *unused)
{
	struct boost_pool *boost_pool;
	int i;

	seq_printf(s, "oplus_boost_pool v%d.%d.%d\n",
		   VERSION_BOOST_POOL_MAJOR,
		   VERSION_BOOST_POOL_MINOR,
		   VERSION_BOOST_POOL_REVISION);

	mutex_lock(&pool_list_lock);
	list_for_each_entry(boost_pool, &pool_list, list) {
		seq_printf(s, "\n%s:%d free:%dMib prefill:%d\n",
			   boost_pool->prefill_task->comm,
			   boost_pool->prefill_task->pid,
			   P2M(boost_pool_nr_pages(boost_pool)),
			   boost_pool->prefill);
		seq_printf(s, "    min          %dMib\n",
			   P2M(boost_pool->min));
		seq_printf(s, "    low          %dMib\n",
			   P2M(boost_pool->low));
		seq_printf(s, "    high         %dMib\n",
			   P2M(boost_pool->high));
		for (i = 0; i < NUM_ORDERS; i++) {
			struct boost_page_pool *pool = boost_pool->pools[i];

			seq_printf(s, "    order-%d  %d\n",
				   pool->order,
				   pool->count[POOL_LOWPAGE] +
				   pool->count[POOL_HIGHPAGE]);
		}
		seq_printf(s, "    cpu          %*pbl\n",
			   cpumask_pr_args(boost_pool->prefill_task->cpus_ptr));
		seq_printf(s, "    custom_pid   %d\n",
			   boost_pool->custom_pid);

	}
	mutex_unlock(&pool_list_lock);
	return 0;
}
DEFINE_PROC_SHOW_ATTRIBUTE(dump);

static int enable_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%d\n", boost_pool_enable);
	return 0;
}

static ssize_t enable_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	char buffer[13];
	int err, enable;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtoint(strstrip(buffer), 0, &enable);
	if (err)
		return err;

	boost_pool_enable = !!enable;
	pr_info("%s:%d enable:%d\n", current->comm, current->tgid,
		boost_pool_enable);
	return count;
}
DEFINE_BOOST_POOL_MGR_PROC_RW_ATTRIBUTE(enable);

int boost_pool_mgr_init(void)
{
	int ret = -ENOMEM;
	struct proc_dir_entry *dump, *enable;
	static bool init;

	if (init)
		return 0;

	procdir = proc_mkdir("boost_pool", NULL);
	if (!procdir) {
		pr_err("mkdir failed\n");
		return ret;
	}

	dump = proc_create_data("dump", 0444, procdir, &dump_proc_ops, NULL);
	if (!dump) {
		pr_err("create proc_fs dump failed\n");
		goto destroy_proc_root;
	}

	enable = proc_create_data("enable", 0444, procdir, &enable_proc_ops, NULL);
	if (!enable) {
		pr_err("create proc_fs enable failed\n");
		goto destroy_proc_dump;
	}

	ret = register_shrinker(&pool_shrinker);
	if (ret) {
		pr_err("register shrinker failed\n");
		goto destroy_proc_enable;
	}
	init = true;
	return 0;

destroy_proc_enable:
	proc_remove(enable);
destroy_proc_dump:
	proc_remove(dump);
destroy_proc_root:
	proc_remove(procdir);
	return ret;
}
MODULE_LICENSE("GPL v2");
