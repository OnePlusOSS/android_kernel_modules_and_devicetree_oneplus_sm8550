// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "kmalloc_debug: " fmt

#ifndef _SLUB_TRACK_
#define _SLUB_TRACK_
#include <linux/sort.h>
#include <linux/jhash.h>
#include <linux/version.h>
#include <linux/swap.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <trace/hooks/mm.h>

#include "slab.h"
#include "memleak_debug_stackdepot.h"

int kmalloc_debug = 1;
int vmalloc_debug = 1;
int daemon_thread = 0;
module_param_named(kmalloc_debug, kmalloc_debug, int, 0444);
module_param_named(vmalloc_debug, vmalloc_debug, int, 0444);
module_param_named(daemon_thread, daemon_thread, int, 0444);

extern int __init create_vmalloc_debug(struct proc_dir_entry *parent);
extern void vmalloc_debug_exit(void);

#define DUMP_BUFF_LEN (PAGE_SIZE << 1)

#define OO_SHIFT	16
#define OO_MASK		((1 << OO_SHIFT) - 1)
#define MAX_OBJS_PER_PAGE	32767 /* since page.objects is u15 */

#define KMALLOC_DEBUG_R_LEN 1024
#define KMALLOC_USED_LEN 4096

/* Internal SLUB flags */
/* Poison object */
#define __OBJECT_POISON		((slab_flags_t __force)0x80000000U)
/* Use cmpxchg_double */
#define __CMPXCHG_DOUBLE	((slab_flags_t __force)0x40000000U)
/*
 * sort the locations with count from more to less.
 */
#define LOCATIONS_TRACK_BUF_SIZE(s) ((s->object_size == 128) ? (PAGE_SIZE << 10) : (PAGE_SIZE * 128))
#define KD_SLABTRACE_STACK_CNT TRACK_ADDRS_COUNT
#define KD_BUFF_LEN(total, len) (total - len - 101)
#define KD_BUFF_LEN_MAX(total) (total - 101 - 100)
#define KD_BUFF_LEN_EXT(total, len) (total - len - 55)

#define MEMLEAK_DETECT_SLEEP_SEC (600 * HZ)
#define KD_VALUE_LEN (32)
#define DATA_LEN (PAGE_SIZE)
#define ALL_KMALLOC_HIGH (KMALLOC_SHIFT_HIGH * 2 + 1)
#define CACHE_INDEX(index) ((index) % (KMALLOC_SHIFT_HIGH + 1))
#define CACHE_TYPE(index) ((index) / (KMALLOC_SHIFT_HIGH + 1))

extern int init_vmalloc_debug(void);
extern void disable_vmalloc_debug(void);
extern void dump_kmalloc_debug_info(struct kmem_cache *s, int index,
		char *dump_buff, int len);
extern int kbuf_dump_kmalloc_debug(struct kmem_cache *s, char *kbuf, int buff_len);

static const struct proc_ops memleak_detect_thread_operations;

enum kmalloc_str_format {
	KMALLOC_STR,
	KMALLOC_NUM,
};

struct kd_location {
	unsigned long count;
	unsigned long addr;
	long long sum_time;
	long min_time;
	long max_time;
	long min_pid;
	long max_pid;
	DECLARE_BITMAP(cpus, NR_CPUS);
	nodemask_t nodes;
	unsigned long addrs[KD_SLABTRACE_STACK_CNT]; /* caller address */
	u32 depth;
	u32 hash;
};

struct kd_loc_track {
	unsigned long max;
	unsigned long count;
	struct kd_location *loc;
};

/*
 * kmalloc_debug_info add debug to slab name.
 */
#ifdef CONFIG_ZONE_DMA
#define INIT_KMALLOC_DEBUG_INFO(__size, __short_size)			\
{								\
	.name[KMALLOC_NORMAL]  = "kmalloc-debug-" #__short_size,	\
	.name[KMALLOC_RECLAIM] = "kmalloc-rcl-debug-" #__short_size,	\
	.name[KMALLOC_DMA]     = "dma-kmalloc-debug-" #__short_size,	\
	.size = __size,						\
}
#else
#define INIT_KMALLOC_DEBUG_INFO(__size, __short_size)			\
{								\
	.name[KMALLOC_NORMAL]  = "kmalloc-debug-" #__short_size,	\
	.name[KMALLOC_RECLAIM] = "kmalloc-rcl-debug-" #__short_size,	\
	.size = __size,						\
}
#endif

#define get_track_hash(track) ((u32)((track)->android_oem_data1))
#define set_track_hash(track, hash) ((track)->android_oem_data1 = hash)

const struct kmalloc_info_struct kmalloc_debug_info[] = {
	INIT_KMALLOC_DEBUG_INFO(0, 0),
	INIT_KMALLOC_DEBUG_INFO(96, 96),
	INIT_KMALLOC_DEBUG_INFO(192, 192),
	INIT_KMALLOC_DEBUG_INFO(8, 8),
	INIT_KMALLOC_DEBUG_INFO(16, 16),
	INIT_KMALLOC_DEBUG_INFO(32, 32),
	INIT_KMALLOC_DEBUG_INFO(64, 64),
	INIT_KMALLOC_DEBUG_INFO(128, 128),
	INIT_KMALLOC_DEBUG_INFO(256, 256),
	INIT_KMALLOC_DEBUG_INFO(512, 512),
	INIT_KMALLOC_DEBUG_INFO(1024, 1k),
	INIT_KMALLOC_DEBUG_INFO(2048, 2k),
	INIT_KMALLOC_DEBUG_INFO(4096, 4k),
	INIT_KMALLOC_DEBUG_INFO(8192, 8k),
	INIT_KMALLOC_DEBUG_INFO(16384, 16k),
	INIT_KMALLOC_DEBUG_INFO(32768, 32k),
	INIT_KMALLOC_DEBUG_INFO(65536, 64k),
	INIT_KMALLOC_DEBUG_INFO(131072, 128k),
	INIT_KMALLOC_DEBUG_INFO(262144, 256k),
	INIT_KMALLOC_DEBUG_INFO(524288, 512k),
	INIT_KMALLOC_DEBUG_INFO(1048576, 1M),
	INIT_KMALLOC_DEBUG_INFO(2097152, 2M),
	INIT_KMALLOC_DEBUG_INFO(4194304, 4M),
	INIT_KMALLOC_DEBUG_INFO(8388608, 8M),
	INIT_KMALLOC_DEBUG_INFO(16777216, 16M),
	INIT_KMALLOC_DEBUG_INFO(33554432, 32M),
	INIT_KMALLOC_DEBUG_INFO(67108864, 64M)
};

#ifdef CONFIG_PREEMPTION
/*
 * Calculate the next globally unique transaction for disambiguation
 * during cmpxchg. The transactions start with the cpu number and are then
 * incremented by CONFIG_NR_CPUS.
 */
#define TID_STEP  roundup_pow_of_two(CONFIG_NR_CPUS)
#else
/*
 * No preemption supported therefore also no need to check for
 * different cpus.
 */
#define TID_STEP 1
#endif

/*
 * kmalloc_debug_caches store the kmalloc caches with debug flag.
 */
atomic64_t kmalloc_debug_caches[NR_KMALLOC_TYPES][KMALLOC_SHIFT_HIGH + 1] = {{ATOMIC64_INIT(0)}};
int kmalloc_debug_enable = 0;

struct task_struct *memleak_detect_task = NULL;
static DEFINE_MUTEX(debug_mutex);

extern unsigned long calculate_kmalloc_slab_size(struct kmem_cache *s);
static unsigned long kd_object_map[BITS_TO_LONGS(MAX_OBJS_PER_PAGE)];
static DEFINE_SPINLOCK(kd_object_map_lock);

static bool freelist_corrupted(struct kmem_cache *s, struct page *page,
		void **freelist, void *nextfree);
static inline bool __cmpxchg_double_slab(struct kmem_cache *s, struct page *page,
		void *freelist_old, unsigned long counters_old,
		void *freelist_new, unsigned long counters_new,
		const char *n);
static void remove_full(struct kmem_cache *s, struct kmem_cache_node *n, struct page *page);
static void add_full(struct kmem_cache *s, struct kmem_cache_node *n, struct page *page);
static int slab_pad_check(struct kmem_cache *s, struct page *page);
static int check_object(struct kmem_cache *s, struct page *page, void *object, u8 val);
static inline void dec_slabs_node(struct kmem_cache *s, int node, int objects);
static void kd_deactivate_slab(struct kmem_cache *s, struct page *page,
		void *freelist, struct kmem_cache_cpu *c);
static struct proc_dir_entry *dpentry;
static struct proc_dir_entry *opentry;
static struct proc_dir_entry *cpentry;
static struct proc_dir_entry *epentry;
static struct proc_dir_entry *upentry;
static struct proc_dir_entry *mpentry;
struct proc_dir_entry *memleak_detect_dir;
struct proc_dir_entry *oplus_mem_dir;

static void save_track_hash_hook(void *data, bool alloc, unsigned long addr)
{
	struct track *p = (struct track *)addr;
	unsigned int hash, nr_entries;

	if (!p || (alloc == false))
		return;

	if (!kmalloc_debug_enable)
		return;

	for (nr_entries = 0; nr_entries < TRACK_ADDRS_COUNT; nr_entries++) {
		if (p->addrs[nr_entries] == 0)
			break;
	}

	/* save the stack hash */
	hash = jhash2((u32 *)p->addrs,
			nr_entries * sizeof(unsigned long) / sizeof(u32),
			0xface);
	set_track_hash(p, hash);
}

static void kmalloc_slab_hook(void *data, unsigned int index, gfp_t flags,
		struct kmem_cache **s)
{
	struct kmem_cache *cache_s;

	cache_s = (struct kmem_cache *)atomic64_read(
			&kmalloc_debug_caches[kmalloc_type(flags)][index]);
	if (cache_s)
		*s = cache_s;
}

static inline unsigned long next_tid(unsigned long tid)
{
	return tid + TID_STEP;
}

static inline void stat(const struct kmem_cache *s, enum stat_item si)
{
#ifdef CONFIG_SLUB_STATS
	/*
	 * The rmw is racy on a preemptible kernel but this is acceptable, so
	 * avoid this_cpu_add()'s irq-disable overhead.
	 */
	raw_cpu_inc(s->cpu_slab->stat[si]);
#endif
}

/********************************************************************
 * 			Core slab cache functions
 *******************************************************************/

/*
 * Returns freelist pointer (ptr). With hardening, this is obfuscated
 * with an XOR of the address where the pointer is held and a per-cache
 * random number.
 */
static inline void *freelist_ptr(const struct kmem_cache *s, void *ptr,
		unsigned long ptr_addr)
{
#ifdef CONFIG_SLAB_FREELIST_HARDENED
	/*
	 * When CONFIG_KASAN_SW/HW_TAGS is enabled, ptr_addr might be tagged.
	 * Normally, this doesn't cause any issues, as both set_freepointer()
	 * and get_freepointer() are called with a pointer with the same tag.
	 * However, there are some issues with CONFIG_SLUB_DEBUG code. For
	 * example, when __free_slub() iterates over objects in a cache, it
	 * passes untagged pointers to check_object(). check_object() in turns
	 * calls get_freepointer() with an untagged pointer, which causes the
	 * freepointer to be restored incorrectly.
	 */
	return (void *)((unsigned long)ptr ^ s->random ^
			swab((unsigned long)kasan_reset_tag((void *)ptr_addr)));
#else
	return ptr;
#endif
}

/* Returns the freelist pointer recorded at location ptr_addr. */
static inline void *freelist_dereference(const struct kmem_cache *s,
		void *ptr_addr)
{
	return freelist_ptr(s, (void *)*(unsigned long *)(ptr_addr),
			(unsigned long)ptr_addr);
}

static inline void *get_freepointer(struct kmem_cache *s, void *object)
{
	object = kasan_reset_tag(object);
	return freelist_dereference(s, object + s->offset);
}

static inline void *get_freepointer_safe(struct kmem_cache *s, void *object)
{
	unsigned long freepointer_addr;
	void *p;

	if (!debug_pagealloc_enabled_static())
		return get_freepointer(s, object);

	freepointer_addr = (unsigned long)object + s->offset;
	copy_from_kernel_nofault(&p, (void **)freepointer_addr, sizeof(p));
	return freelist_ptr(s, p, freepointer_addr);
}

static inline void set_freepointer(struct kmem_cache *s, void *object, void *fp)
{
	unsigned long freeptr_addr = (unsigned long)object + s->offset;

#ifdef CONFIG_SLAB_FREELIST_HARDENED
	BUG_ON(object == fp); /* naive detection of double free or corruption */
#endif

	freeptr_addr = (unsigned long)kasan_reset_tag((void *)freeptr_addr);
	*(void **)freeptr_addr = freelist_ptr(s, fp, freeptr_addr);
}

void *fixup_red_left(struct kmem_cache *s, void *p)
{
	if (kmem_cache_debug_flags(s, SLAB_RED_ZONE))
		p += s->red_left_pad;

	return p;
}

/* Loop over all objects in a slab */
#define for_each_object(__p, __s, __addr, __objects) \
	for (__p = fixup_red_left(__s, __addr); \
			__p < (__addr) + (__objects) * (__s)->size; \
			__p += (__s)->size)

static inline unsigned int order_objects(unsigned int order, unsigned int size)
{
	return ((unsigned int)PAGE_SIZE << order) / size;
}

static inline struct kmem_cache_order_objects oo_make(unsigned int order,
		unsigned int size)
{
	struct kmem_cache_order_objects x = {
		(order << OO_SHIFT) + order_objects(order, size)
	};

	return x;
}

/*
 * Management of partially allocated slabs.
 */
	static inline void
__add_partial(struct kmem_cache_node *n, struct page *page, int tail)
{
	n->nr_partial++;
	if (tail == DEACTIVATE_TO_TAIL)
		list_add_tail(&page->slab_list, &n->partial);
	else
		list_add(&page->slab_list, &n->partial);
}

static inline void add_partial(struct kmem_cache_node *n,
		struct page *page, int tail)
{
	lockdep_assert_held(&n->list_lock);
	__add_partial(n, page, tail);
}

static inline void remove_partial(struct kmem_cache_node *n,
		struct page *page)
{
	lockdep_assert_held(&n->list_lock);
	list_del(&page->slab_list);
	n->nr_partial--;
}

static void __free_slab(struct kmem_cache *s, struct page *page)
{
	int order = compound_order(page);
	int pages = 1 << order;

	if (kmem_cache_debug_flags(s, SLAB_CONSISTENCY_CHECKS)) {
		void *p;

		slab_pad_check(s, page);
		for_each_object(p, s, page_address(page),
				page->objects)
			check_object(s, page, p, SLUB_RED_INACTIVE);
	}

	__ClearPageSlabPfmemalloc(page);
	__ClearPageSlab(page);

	page->mapping = NULL;
	if (current->reclaim_state)
		current->reclaim_state->reclaimed_slab += pages;
	unaccount_slab_page(page, order, s);
	__free_pages(page, order);
}

static void rcu_free_slab(struct rcu_head *h)
{
	struct page *page = container_of(h, struct page, rcu_head);

	__free_slab(page->slab_cache, page);
}

static void free_slab(struct kmem_cache *s, struct page *page)
{
	if (unlikely(s->flags & SLAB_TYPESAFE_BY_RCU)) {
		call_rcu(&page->rcu_head, rcu_free_slab);
	} else
		__free_slab(s, page);
}

static void discard_slab(struct kmem_cache *s, struct page *page)
{
	dec_slabs_node(s, page_to_nid(page), page->objects);
	free_slab(s, page);
}

/*
 * Remove the cpu slab
 */
static void kd_deactivate_slab(struct kmem_cache *s, struct page *page,
		void *freelist, struct kmem_cache_cpu *c)
{
	enum slab_modes { M_NONE, M_PARTIAL, M_FULL, M_FREE };
	struct kmem_cache_node *n = get_node(s, page_to_nid(page));
	int lock = 0;
	enum slab_modes l = M_NONE, m = M_NONE;
	void *nextfree;
	int tail = DEACTIVATE_TO_HEAD;
	struct page new;
	struct page old;

	if (page->freelist) {
		stat(s, DEACTIVATE_REMOTE_FREES);
		tail = DEACTIVATE_TO_TAIL;
	}

	/*
	 * Stage one: Free all available per cpu objects back
	 * to the page freelist while it is still frozen. Leave the
	 * last one.
	 *
	 * There is no need to take the list->lock because the page
	 * is still frozen.
	 */
	while (freelist && (nextfree = get_freepointer(s, freelist))) {
		void *prior;
		unsigned long counters;

		/*
		 * If 'nextfree' is invalid, it is possible that the object at
		 * 'freelist' is already corrupted.  So isolate all objects
		 * starting at 'freelist'.
		 */
		if (freelist_corrupted(s, page, &freelist, nextfree))
			break;

		do {
			prior = page->freelist;
			counters = page->counters;
			set_freepointer(s, freelist, prior);
			new.counters = counters;
			new.inuse--;
			VM_BUG_ON(!new.frozen);
		} while (!__cmpxchg_double_slab(s, page,
					prior, counters,
					freelist, new.counters,
					"drain percpu freelist"));

		freelist = nextfree;
	}

	/*
	 * Stage two: Ensure that the page is unfrozen while the
	 * list presence reflects the actual number of objects
	 * during unfreeze.
	 *
	 * We setup the list membership and then perform a cmpxchg
	 * with the count. If there is a mismatch then the page
	 * is not unfrozen but the page is on the wrong list.
	 *
	 * Then we restart the process which may have to remove
	 * the page from the list that we just put it on again
	 * because the number of objects in the slab may have
	 * changed.
	 */
redo:

	old.freelist = page->freelist;
	old.counters = page->counters;
	VM_BUG_ON(!old.frozen);

	/* Determine target state of the slab */
	new.counters = old.counters;
	if (freelist) {
		new.inuse--;
		set_freepointer(s, freelist, old.freelist);
		new.freelist = freelist;
	} else
		new.freelist = old.freelist;

	new.frozen = 0;

	if (!new.inuse && n->nr_partial >= s->min_partial)
		m = M_FREE;
	else if (new.freelist) {
		m = M_PARTIAL;
		if (!lock) {
			lock = 1;
			/*
			 * Taking the spinlock removes the possibility
			 * that acquire_slab() will see a slab page that
			 * is frozen
			 */
			spin_lock(&n->list_lock);
		}
	} else {
		m = M_FULL;
		if ((s->flags & SLAB_STORE_USER) && !lock) {
			lock = 1;
			/*
			 * This also ensures that the scanning of full
			 * slabs from diagnostic functions will not see
			 * any frozen slabs.
			 */
			spin_lock(&n->list_lock);
		}
	}

	if (l != m) {
		if (l == M_PARTIAL)
			remove_partial(n, page);
		else if (l == M_FULL)
			remove_full(s, n, page);

		if (m == M_PARTIAL)
			add_partial(n, page, tail);
		else if (m == M_FULL)
			add_full(s, n, page);
	}

	l = m;
	if (!__cmpxchg_double_slab(s, page,
				old.freelist, old.counters,
				new.freelist, new.counters,
				"unfreezing slab"))
		goto redo;

	if (lock)
		spin_unlock(&n->list_lock);

	if (m == M_PARTIAL)
		stat(s, tail);
	else if (m == M_FULL)
		stat(s, DEACTIVATE_FULL);
	else if (m == M_FREE) {
		stat(s, DEACTIVATE_EMPTY);
		discard_slab(s, page);
		stat(s, FREE_SLAB);
	}

	c->page = NULL;
	c->freelist = NULL;
}

/*
 * Unfreeze all the cpu partial slabs.
 *
 * This function must be called with interrupts disabled
 * for the cpu using c (or some other guarantee must be there
 * to guarantee no concurrent accesses).
 */
static void kd_unfreeze_partials(struct kmem_cache *s,
		struct kmem_cache_cpu *c)
{
#ifdef CONFIG_SLUB_CPU_PARTIAL
	struct kmem_cache_node *n = NULL, *n2 = NULL;
	struct page *page, *discard_page = NULL;

	while ((page = slub_percpu_partial(c))) {
		struct page new;
		struct page old;

		slub_set_percpu_partial(c, page);

		n2 = get_node(s, page_to_nid(page));
		if (n != n2) {
			if (n)
				spin_unlock(&n->list_lock);

			n = n2;
			spin_lock(&n->list_lock);
		}

		do {
			old.freelist = page->freelist;
			old.counters = page->counters;
			VM_BUG_ON(!old.frozen);

			new.counters = old.counters;
			new.freelist = old.freelist;

			new.frozen = 0;
		} while (!__cmpxchg_double_slab(s, page,
					old.freelist, old.counters,
					new.freelist, new.counters,
					"unfreezing slab"));

		if (unlikely(!new.inuse && n->nr_partial >= s->min_partial)) {
			page->next = discard_page;
			discard_page = page;
		} else {
			add_partial(n, page, DEACTIVATE_TO_TAIL);
			stat(s, FREE_ADD_PARTIAL);
		}
	}

	if (n)
		spin_unlock(&n->list_lock);

	while (discard_page) {
		page = discard_page;
		discard_page = discard_page->next;

		stat(s, DEACTIVATE_EMPTY);
		discard_slab(s, page);
		stat(s, FREE_SLAB);
	}
#endif	/* CONFIG_SLUB_CPU_PARTIAL */
}

static inline void kd_flush_slab(struct kmem_cache *s, struct kmem_cache_cpu *c)
{
	stat(s, CPUSLAB_FLUSH);
	kd_deactivate_slab(s, c->page, c->freelist, c);

	c->tid = next_tid(c->tid);
}

/*
 * Flush cpu slab.
 *
 * Called from IPI handler with interrupts disabled.
 */
static inline void __flush_cpu_slab(struct kmem_cache *s, int cpu)
{
	struct kmem_cache_cpu *c = per_cpu_ptr(s->cpu_slab, cpu);

	if (c->page)
		kd_flush_slab(s, c);

	kd_unfreeze_partials(s, c);
}

static void kd_flush_cpu_slab(void *d)
{
	struct kmem_cache *s = d;

	__flush_cpu_slab(s, smp_processor_id());
}

static bool has_cpu_slab(int cpu, void *info)
{
	struct kmem_cache *s = info;
	struct kmem_cache_cpu *c = per_cpu_ptr(s->cpu_slab, cpu);

	return c->page || slub_percpu_partial(c);
}

static void kd_flush_all(struct kmem_cache *s)
{
	on_each_cpu_cond(has_cpu_slab, kd_flush_cpu_slab, s, 1);
}
/*
 * Per slab locking using the pagelock
 */
static __always_inline void slab_lock(struct page *page)
{
	VM_BUG_ON_PAGE(PageTail(page), page);
	bit_spin_lock(PG_locked, &page->flags);
}

static __always_inline void slab_unlock(struct page *page)
{
	VM_BUG_ON_PAGE(PageTail(page), page);
	__bit_spin_unlock(PG_locked, &page->flags);
}

/* Interrupts must be disabled (for the fallback code to work right) */
static inline bool __cmpxchg_double_slab(struct kmem_cache *s, struct page *page,
		void *freelist_old, unsigned long counters_old,
		void *freelist_new, unsigned long counters_new,
		const char *n)
{
	VM_BUG_ON(!irqs_disabled());
#if defined(CONFIG_HAVE_CMPXCHG_DOUBLE) && \
	defined(CONFIG_HAVE_ALIGNED_STRUCT_PAGE)
	if (s->flags & __CMPXCHG_DOUBLE) {
		if (cmpxchg_double(&page->freelist, &page->counters,
					freelist_old, counters_old,
					freelist_new, counters_new))
			return true;
	} else
#endif
	 {
		slab_lock(page);
		if (page->freelist == freelist_old &&
				page->counters == counters_old) {
			page->freelist = freelist_new;
			page->counters = counters_new;
			slab_unlock(page);
			return true;
		}
		slab_unlock(page);
	}

	cpu_relax();
	stat(s, CMPXCHG_DOUBLE_FAIL);

#ifdef SLUB_DEBUG_CMPXCHG
	pr_info("%s %s: cmpxchg double redo ", n, s->name);
#endif

	return false;
}

static inline bool cmpxchg_double_slab(struct kmem_cache *s, struct page *page,
		void *freelist_old, unsigned long counters_old,
		void *freelist_new, unsigned long counters_new,
		const char *n)
{
#if defined(CONFIG_HAVE_CMPXCHG_DOUBLE) && \
	defined(CONFIG_HAVE_ALIGNED_STRUCT_PAGE)
	if (s->flags & __CMPXCHG_DOUBLE) {
		if (cmpxchg_double(&page->freelist, &page->counters,
					freelist_old, counters_old,
					freelist_new, counters_new))
			return true;
	} else
#endif
	 {
		unsigned long flags;

		local_irq_save(flags);
		slab_lock(page);
		if (page->freelist == freelist_old &&
				page->counters == counters_old) {
			page->freelist = freelist_new;
			page->counters = counters_new;
			slab_unlock(page);
			local_irq_restore(flags);
			return true;
		}
		slab_unlock(page);
		local_irq_restore(flags);
	}

	cpu_relax();
	stat(s, CMPXCHG_DOUBLE_FAIL);

#ifdef SLUB_DEBUG_CMPXCHG
	pr_info("%s %s: cmpxchg double redo ", n, s->name);
#endif

	return false;
}

/*
 * Determine a map of object in use on a page.
 *
 * Node listlock must be held to guarantee that the page does
 * not vanish from under us.
 */
static unsigned long *kd_get_map(struct kmem_cache *s, struct page *page)
{
	void *p;
	void *addr = page_address(page);

	VM_BUG_ON(!irqs_disabled());

	spin_lock(&kd_object_map_lock);

	bitmap_zero(kd_object_map, page->objects);

	for (p = page->freelist; p; p = get_freepointer(s, p))
		set_bit(__obj_to_index(s, addr, p), kd_object_map);

	return kd_object_map;
}

static void kd_put_map(unsigned long *map) __releases(&kd_object_map_lock)
{
	VM_BUG_ON(map != kd_object_map);
	spin_unlock(&kd_object_map_lock);
}

static inline unsigned int size_from_object(struct kmem_cache *s)
{
	if (s->flags & SLAB_RED_ZONE)
		return s->size - s->red_left_pad;

	return s->size;
}

static inline void *restore_red_left(struct kmem_cache *s, void *p)
{
	if (s->flags & SLAB_RED_ZONE)
		p -= s->red_left_pad;

	return p;
}

#if defined(CONFIG_KASAN_GENERIC) || defined(CONFIG_KASAN_SW_TAGS)
static void kasan_enable_current_dup(void)
{
	current->kasan_depth++;
}

static void kasan_disable_current_dup(void)
{
	current->kasan_depth--;
}
#else
static void kasan_enable_current_dup(void) {}
static void kasan_disable_current_dup(void) {}
#endif /* CONFIG_KASAN_GENERIC || CONFIG_KASAN_SW_TAGS */

/*
 * slub is about to manipulate internal object metadata.  This memory lies
 * outside the range of the allocated object, so accessing it would normally
 * be reported by kasan as a bounds error.  metadata_access_enable() is used
 * to tell kasan that these accesses are OK.
 */
static inline void metadata_access_enable(void)
{
	kasan_disable_current_dup();
}

static inline void metadata_access_disable(void)
{
	kasan_enable_current_dup();
}

/*
 * Object debugging
 */

/* Verify that a pointer has an address that is valid within a slab page */
static inline int check_valid_pointer(struct kmem_cache *s,
		struct page *page, void *object)
{
	void *base;

	if (!object)
		return 1;

	base = page_address(page);
	object = kasan_reset_tag(object);
	object = restore_red_left(s, object);
	if (object < base || (object >= (base + page->objects * s->size))
			|| (object - base) % s->size) {
		return 0;
	}

	return 1;
}

static void print_section(char *level, char *text, u8 *addr,
		unsigned int length)
{
	metadata_access_enable();
	print_hex_dump(level, kasan_reset_tag(text), DUMP_PREFIX_ADDRESS,
			16, 1, addr, length, 1);
	metadata_access_disable();
}

/*
 * See comment in calculate_sizes().
 */
static inline bool freeptr_outside_object(struct kmem_cache *s)
{
	return s->offset >= s->inuse;
}

/*
 * Return offset of the end of info block which is inuse + free pointer if
 * not overlapping with object.
 */
static inline unsigned int get_info_end(struct kmem_cache *s)
{
	if (freeptr_outside_object(s))
		return s->inuse + sizeof(void *);
	else
		return s->inuse;
}

static noinline struct track *kd_get_track(struct kmem_cache *s, void *object,
		enum track_item alloc)
{
	struct track *p;

	p = object + get_info_end(s);

	return kasan_reset_tag(p + alloc);
}

static noinline void print_track(const char *s, struct track *t,
		unsigned long pr_time)
{
	if (!t->addr)
		return;

	pr_err("INFO: %s in %pS age=%lu cpu=%u pid=%d\n",
			s, (void *)t->addr, pr_time - t->when, t->cpu, t->pid);
#ifdef CONFIG_STACKTRACE
	 {
		int i;
		for (i = 0; i < TRACK_ADDRS_COUNT; i++)
			if (t->addrs[i])
				pr_err("\t%pS\n", (void *)t->addrs[i]);
			else
				break;
	}
#endif
}

void print_tracking(struct kmem_cache *s, void *object)
{
	unsigned long pr_time = jiffies;
	if (!(s->flags & SLAB_STORE_USER))
		return;

	print_track("Allocated", kd_get_track(s, object, TRACK_ALLOC), pr_time);
	print_track("Freed", kd_get_track(s, object, TRACK_FREE), pr_time);
}

static void print_page_info(struct page *page)
{
	pr_err("INFO: Slab 0x%p objects=%u used=%u fp=0x%p flags=0x%04lx\n",
			page, page->objects, page->inuse, page->freelist, page->flags);
}

static void slab_bug(struct kmem_cache *s, char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	pr_err("=============================================================================\n");
	pr_err("BUG %s : %pV\n", s->name, &vaf);
	pr_err("-----------------------------------------------------------------------------\n\n");

	add_taint(TAINT_BAD_PAGE, LOCKDEP_NOW_UNRELIABLE);
	va_end(args);
}

static void slab_fix(struct kmem_cache *s, char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	pr_err("FIX %s: %pV\n", s->name, &vaf);
	va_end(args);
}

static bool freelist_corrupted(struct kmem_cache *s, struct page *page,
		void **freelist, void *nextfree)
{
	if ((s->flags & SLAB_CONSISTENCY_CHECKS) &&
			!check_valid_pointer(s, page, nextfree) && freelist) {
		object_err(s, page, *freelist, "Freechain corrupt");
		*freelist = NULL;
		slab_fix(s, "Isolate corrupted freechain");
		return true;
	}

	return false;
}

static void print_trailer(struct kmem_cache *s, struct page *page, u8 *p)
{
	unsigned int off;	/* Offset of last byte */
	u8 *addr = page_address(page);

	print_tracking(s, p);

	print_page_info(page);

	pr_err("INFO: Object 0x%p @offset=%tu fp=0x%p\n\n",
			p, p - addr, get_freepointer(s, p));

	if (s->flags & SLAB_RED_ZONE)
		print_section(KERN_ERR, "Redzone ", p - s->red_left_pad,
				s->red_left_pad);
	else if (p > addr + 16)
		print_section(KERN_ERR, "Bytes b4 ", p - 16, 16);

	print_section(KERN_ERR, "Object ", p,
			min_t(unsigned int, s->object_size, PAGE_SIZE));
	if (s->flags & SLAB_RED_ZONE)
		print_section(KERN_ERR, "Redzone ", p + s->object_size,
				s->inuse - s->object_size);

	off = get_info_end(s);

	if (s->flags & SLAB_STORE_USER)
		off += 2 * sizeof(struct track);

	if (off != size_from_object(s))
		/* Beginning of the filler is the free pointer */
		print_section(KERN_ERR, "Padding ", p + off,
				size_from_object(s) - off);

	dump_stack();
}

void object_err(struct kmem_cache *s, struct page *page,
		u8 *object, char *reason)
{
	slab_bug(s, "%s", reason);
	print_trailer(s, page, object);
}

static __printf(3, 4) void slab_err(struct kmem_cache *s, struct page *page,
		const char *fmt, ...)
{
	va_list args;
	char buf[100];

	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	slab_bug(s, "%s", buf);
	print_page_info(page);
	dump_stack();
}

static void restore_bytes(struct kmem_cache *s, char *message, u8 data,
		void *from, void *to)
{
	slab_fix(s, "Restoring 0x%p-0x%p=0x%x\n", from, to - 1, data);
	memset(from, data, to - from);
}

static int check_bytes_and_report(struct kmem_cache *s, struct page *page,
		u8 *object, char *what,
		u8 *start, unsigned int value, unsigned int bytes)
{
	u8 *fault;
	u8 *end;
	u8 *addr = page_address(page);

	metadata_access_enable();
	fault = memchr_inv(kasan_reset_tag(start), value, bytes);
	metadata_access_disable();
	if (!fault)
		return 1;

	end = start + bytes;
	while (end > fault && end[-1] == value)
		end--;

	slab_bug(s, "%s overwritten", what);
	pr_err("INFO: 0x%p-0x%p @offset=%tu. First byte 0x%x instead of 0x%x\n",
			fault, end - 1, fault - addr,
			fault[0], value);
	print_trailer(s, page, object);

	restore_bytes(s, what, value, fault, end);
	return 0;
}

/*
 * Object layout:
 *
 * object address
 * 	Bytes of the object to be managed.
 * 	If the freepointer may overlay the object then the free
 *	pointer is at the middle of the object.
 *
 * 	Poisoning uses 0x6b (POISON_FREE) and the last byte is
 * 	0xa5 (POISON_END)
 *
 * object + s->object_size
 * 	Padding to reach word boundary. This is also used for Redzoning.
 * 	Padding is extended by another word if Redzoning is enabled and
 * 	object_size == inuse.
 *
 * 	We fill with 0xbb (RED_INACTIVE) for inactive objects and with
 * 	0xcc (RED_ACTIVE) for objects in use.
 *
 * object + s->inuse
 * 	Meta data starts here.
 *
 * 	A. Free pointer (if we cannot overwrite object on free)
 * 	B. Tracking data for SLAB_STORE_USER
 * 	C. Padding to reach required alignment boundary or at mininum
 * 		one word if debugging is on to be able to detect writes
 * 		before the word boundary.
 *
 *	Padding is done using 0x5a (POISON_INUSE)
 *
 * object + s->size
 * 	Nothing is used beyond s->size.
 *
 * If slabcaches are merged then the object_size and inuse boundaries are mostly
 * ignored. And therefore no slab options that rely on these boundaries
 * may be used with merged slabcaches.
 */
static int check_pad_bytes(struct kmem_cache *s, struct page *page, u8 *p)
{
	unsigned long off = get_info_end(s);	/* The end of info */

	if (s->flags & SLAB_STORE_USER)
		/* We also have user information there */
		off += 2 * sizeof(struct track);

	if (size_from_object(s) == off)
		return 1;

	return check_bytes_and_report(s, page, p, "Object padding",
			p + off, POISON_INUSE, size_from_object(s) - off);
}

/* Check the pad bytes at the end of a slab page */
static int slab_pad_check(struct kmem_cache *s, struct page *page)
{
	u8 *start;
	u8 *fault;
	u8 *end;
	u8 *pad;
	int length;
	int remainder;

	if (!(s->flags & SLAB_POISON))
		return 1;

	start = page_address(page);
	length = page_size(page);
	end = start + length;
	remainder = length % s->size;
	if (!remainder)
		return 1;

	pad = end - remainder;
	metadata_access_enable();
	fault = memchr_inv(kasan_reset_tag(pad), POISON_INUSE, remainder);
	metadata_access_disable();
	if (!fault)
		return 1;
	while (end > fault && end[-1] == POISON_INUSE)
		end--;

	slab_err(s, page, "Padding overwritten. 0x%p-0x%p @offset=%tu",
			fault, end - 1, fault - start);
	print_section(KERN_ERR, "Padding ", pad, remainder);

	restore_bytes(s, "slab padding", POISON_INUSE, fault, end);
	return 0;
}

static int check_object(struct kmem_cache *s, struct page *page,
		void *object, u8 val)
{
	u8 *p = object;
	u8 *endobject = object + s->object_size;

	if (s->flags & SLAB_RED_ZONE) {
		if (!check_bytes_and_report(s, page, object, "Redzone",
					object - s->red_left_pad, val, s->red_left_pad))
			return 0;

		if (!check_bytes_and_report(s, page, object, "Redzone",
					endobject, val, s->inuse - s->object_size))
			return 0;
	} else {
		if ((s->flags & SLAB_POISON) && s->object_size < s->inuse) {
			check_bytes_and_report(s, page, p, "Alignment padding",
					endobject, POISON_INUSE,
					s->inuse - s->object_size);
		}
	}

	if (s->flags & SLAB_POISON) {
		if (val != SLUB_RED_ACTIVE && (s->flags & __OBJECT_POISON) &&
				(!check_bytes_and_report(s, page, p, "Poison", p,
							 POISON_FREE, s->object_size - 1) ||
				 !check_bytes_and_report(s, page, p, "Poison",
					 p + s->object_size - 1, POISON_END, 1)))
			return 0;
		/*
		 * check_pad_bytes cleans up on its own.
		 */
		check_pad_bytes(s, page, p);
	}

	if (!freeptr_outside_object(s) && val == SLUB_RED_ACTIVE)
		/*
		 * Object and freepointer overlap. Cannot check
		 * freepointer while object is allocated.
		 */
		return 1;

	/* Check free pointer validity */
	if (!check_valid_pointer(s, page, get_freepointer(s, p))) {
		object_err(s, page, p, "Freepointer corrupt");
		/*
		 * No choice but to zap it and thus lose the remainder
		 * of the free objects in this slab. May cause
		 * another error because the object count is now wrong.
		 */
		set_freepointer(s, p, NULL);
		return 0;
	}
	return 1;
}

/*
 * Tracking of fully allocated slabs for debugging purposes.
 */
static void add_full(struct kmem_cache *s,
		struct kmem_cache_node *n, struct page *page)
{
	if (!(s->flags & SLAB_STORE_USER))
		return;

	lockdep_assert_held(&n->list_lock);
	list_add(&page->slab_list, &n->full);
}

static void remove_full(struct kmem_cache *s, struct kmem_cache_node *n, struct page *page)
{
	if (!(s->flags & SLAB_STORE_USER))
		return;

	lockdep_assert_held(&n->list_lock);
	list_del(&page->slab_list);
}

/* Tracking of the number of slabs for debugging purposes */
static inline unsigned long slabs_node(struct kmem_cache *s, int node)
{
	struct kmem_cache_node *n = get_node(s, node);

	return atomic_long_read(&n->nr_slabs);
}

static inline unsigned long node_nr_slabs(struct kmem_cache_node *n)
{
	return atomic_long_read(&n->nr_slabs);
}

static inline void inc_slabs_node(struct kmem_cache *s, int node, int objects)
{
	struct kmem_cache_node *n = get_node(s, node);

	/*
	 * May be called early in order to allocate a slab for the
	 * kmem_cache_node structure. Solve the chicken-egg
	 * dilemma by deferring the increment of the count during
	 * bootstrap (see early_kmem_cache_node_alloc).
	 */
	if (likely(n)) {
		atomic_long_inc(&n->nr_slabs);
		atomic_long_add(objects, &n->total_objects);
	}
}

static inline void dec_slabs_node(struct kmem_cache *s, int node, int objects)
{
	struct kmem_cache_node *n = get_node(s, node);

	atomic_long_dec(&n->nr_slabs);
	atomic_long_sub(objects, &n->total_objects);
}

static int kd_location_cmp(const void *la, const void *lb)
{
	return ((struct kd_location *)lb)->count - ((struct kd_location *)la)->count;
}

static void kd_location_swap(void *la, void *lb, int size)
{
	struct kd_location l_tmp;

	memcpy(&l_tmp, la, size);
	memcpy(la, lb, size);
	memcpy(lb, &l_tmp, size);
}

static void kd_free_loc_track(struct kd_loc_track *t)
{
	if (t->max)
		vfree(t->loc);
}

static int kd_alloc_loc_track(struct kd_loc_track *t, int buff_size)
{
	struct kd_location *l;

	l = (void *)vzalloc(buff_size);
	if (!l) {
		buff_size >>= 1;
		l = (void *)vzalloc(buff_size);
		if (!l)
			return -ENOMEM;
	}

	t->count = 0;
	t->max = buff_size / sizeof(struct kd_location);
	t->loc = l;
	return 0;
}

static int kd_add_location(struct kd_loc_track *t, struct kmem_cache *s,
		const struct track *track)
{
	long start, end, pos;
	struct kd_location *l;
	/*
	 * save the stack depth and hash.
	 */
	u32 hash;
	unsigned long age;

	if (get_track_hash(track) == 0)
		return -EINVAL;

	age = jiffies - track->when;
	start = -1;
	end = t->count;

	for (;;) {
		pos = start + (end - start + 1) / 2;

		/*
		 * There is nothing at "end". If we end up there
		 * we need to add something to before end.
		 */
		if (pos == end)
			break;

		hash = t->loc[pos].hash;
		if (get_track_hash(track) == hash) {
			l = &t->loc[pos];
			l->count++;
			if (track->when) {
				l->sum_time += age;
				if (age < l->min_time)
					l->min_time = age;
				if (age > l->max_time)
					l->max_time = age;

				if (track->pid < l->min_pid)
					l->min_pid = track->pid;
				if (track->pid > l->max_pid)
					l->max_pid = track->pid;
			}
			return 0;
		}
		/*
		 * use hash value to record the stack.
		 */
		if (get_track_hash(track) < hash)
			end = pos;
		else
			start = pos;
	}

	/*
	 * Not found. Insert new tracking element.
	 */
	if (t->count >= t->max)
		return -ENOMEM;

	l = t->loc + pos;
	if (pos < t->count)
		memmove(l + 1, l,
				(t->count - pos) * sizeof(struct kd_location));
	t->count++;
	l->count = 1;
	l->addr = track->addr;
	l->sum_time = age;
	l->min_time = age;
	l->max_time = age;
	l->min_pid = track->pid;
	l->max_pid = track->pid;
	l->depth = (u32)(sizeof(l->addrs)/sizeof(l->addrs[0]));
	l->hash = get_track_hash(track);
#ifdef COMPACT_OPLUS_SLUB_TRACK
	 {
		int i;
		for (i = 0; i < l->depth; i++)
			l->addrs[i] = track->addrs[i] + MODULES_VADDR;
	}
#else
	memcpy(l->addrs, track->addrs, sizeof(l->addrs[0])*l->depth);
#endif
	return 0;
}

static int kd_process_slab(struct kd_loc_track *t, struct kmem_cache *s,
		struct page *page, enum track_item alloc)
{
	void *addr = page_address(page);
	void *p;
	unsigned long *map;
	int dropped = 0;

	map = kd_get_map(s, page);
	for_each_object(p, s, addr, page->objects)
		if (!test_bit(__obj_to_index(s, addr, p), map))
			if (kd_add_location(t, s, kd_get_track(s, p, alloc)))
				dropped++;
	kd_put_map(map);
	return dropped;
}

static noinline int kd_list_locations(struct kmem_cache *s, char *buf,
		int buff_len, enum track_item alloc)
{
	unsigned long i, j;
	int len = 0;
	int node, ret;
	int dropped = 0;
	struct kmem_cache_node *n;
	struct kd_loc_track t = { 0, 0, NULL };

	if (kd_alloc_loc_track(&t, LOCATIONS_TRACK_BUF_SIZE(s))) {
		return sprintf(buf, "Out of memory\n");
	}

	/* Push back cpu slabs */
	kd_flush_all(s);

	for_each_kmem_cache_node(s, node, n) {
		unsigned long flags;
		struct page *page;

		if (!atomic_long_read(&n->nr_slabs))
			continue;

		spin_lock_irqsave(&n->list_lock, flags);
		list_for_each_entry(page, &n->partial, slab_list) {
			ret = kd_process_slab(&t, s, page, alloc);
			if (ret)
				dropped += ret;
		}

		list_for_each_entry(page, &n->full, slab_list) {
			ret = kd_process_slab(&t, s, page, alloc);
			if (ret)
				dropped += ret;
		}
		spin_unlock_irqrestore(&n->list_lock, flags);
	}

	/*
	 * sort the locations with count from more to less.
	 */
	sort(&t.loc[0], t.count, sizeof(struct kd_location), kd_location_cmp,
			kd_location_swap);

	for (i = 0; i < t.count; i++) {
		struct kd_location *l = &t.loc[i];

		if (len >= KD_BUFF_LEN_MAX(buff_len))
			break;

		len += scnprintf(buf + len, KD_BUFF_LEN(buff_len, len), "%7ld ",
				l->count);

		if (l->addr)
			len += scnprintf(buf + len, KD_BUFF_LEN(buff_len, len), "%pS",
					(void *)l->addr);
		else
			len += scnprintf(buf + len, KD_BUFF_LEN(buff_len, len),
					"<not-available>");

		if (l->sum_time != l->min_time)
			len += scnprintf(buf + len, KD_BUFF_LEN(buff_len, len),
					" age=%ld/%ld/%ld",
					l->min_time,
					(long)div_u64(l->sum_time, l->count),
					l->max_time);
		else
			len += scnprintf(buf + len, KD_BUFF_LEN(buff_len, len),
					" age=%ld", l->min_time);

		if (l->min_pid != l->max_pid)
			len += scnprintf(buf + len, KD_BUFF_LEN(buff_len, len),
					" pid=%ld-%ld", l->min_pid, l->max_pid);
		else
			len += scnprintf(buf + len, KD_BUFF_LEN(buff_len, len),
					" pid=%ld", l->min_pid);
		len += scnprintf(buf + len, KD_BUFF_LEN(buff_len, len), "\n");

		for (j = 0; j < l->depth; j++) {
			if (!l->addrs[j])
				break;

			len += scnprintf(buf + len, KD_BUFF_LEN(buff_len, len),
					"%pS\n", (void *)l->addrs[j]);
		}
		len += scnprintf(buf + len, KD_BUFF_LEN(buff_len, len), "\n");
	}
	if (t.count && (buf[len -1] != '\n'))
		buf[len++] = '\n';
	kd_free_loc_track(&t);

	if (!t.count)
		len += scnprintf(buf + len, KD_BUFF_LEN_EXT(buff_len, len),
				"%s no data\n", s->name);
	if (dropped)
		len += scnprintf(buf + len, KD_BUFF_LEN_EXT(buff_len, len),
				"%s dropped %d %lu %lu\n",
				s->name, dropped, t.count, t.max);
	if (buf[len -1] != '\n')
		buf[len++] = '\n';
	return len;
}

int kbuf_dump_kmalloc_debug(struct kmem_cache *s, char *kbuf, int buff_len)
{
	memset(kbuf, 0, buff_len);
	return kd_list_locations(s, kbuf, buff_len, TRACK_ALLOC);
}

#define KMALLOC_DEBUG_MIN_WATERMARK 100u
#define KMALLOC_DEBUG_DUMP_STEP 20u
#define BUFLEN(total, len) (total - len - 81)
#define BUFLEN_EXT(total, len) (total - len - 1)
#define KMALLOC_LOG_TAG "kmalloc_debug"

static unsigned int kmalloc_debug_watermark[KMALLOC_SHIFT_HIGH + 1];

void kmalloc_debug_watermark_init(void)
{
	int i;
	unsigned int water;

	for (i = 0; i <= KMALLOC_SHIFT_HIGH; i++) {
		if (kmalloc_debug_info[i].size <= 128)
			water = KMALLOC_DEBUG_MIN_WATERMARK;
		else
			water = KMALLOC_DEBUG_MIN_WATERMARK >> 1;
		kmalloc_debug_watermark[i] = water;
	}
}

static void dump_locations(struct kmem_cache *s, int slab_size, int index,
		char *dump_buff, int len, enum track_item alloc)
{
	unsigned long i, j;
	struct kd_loc_track t = { 0, 0, NULL };
	int node;
	struct kmem_cache_node *n;
	int dump_buff_len = 0;

	if (kd_alloc_loc_track(&t, PAGE_SIZE)) {
		sprintf(dump_buff, "Out of memory\n");
		goto out;
	}

	/* Push back cpu slabs */
	kd_flush_all(s);

	for_each_kmem_cache_node(s, node, n) {
		unsigned long flags;
		struct page *page;

		if (!atomic_long_read(&n->nr_slabs))
			continue;

		spin_lock_irqsave(&n->list_lock, flags);
		list_for_each_entry(page, &n->partial, slab_list)
			kd_process_slab(&t, s, page, alloc);
		list_for_each_entry(page, &n->full, slab_list)
			kd_process_slab(&t, s, page, alloc);
		spin_unlock_irqrestore(&n->list_lock, flags);
	}

	sort(&t.loc[0], t.count, sizeof(struct kd_location), kd_location_cmp,
			kd_location_swap);

	dump_buff_len = scnprintf(dump_buff + dump_buff_len,
			len - dump_buff_len - 2,
			"%s used %u MB Water %u MB:\n", s->name, slab_size,
			kmalloc_debug_watermark[index]);

	for (i = 0; i < t.count; i++) {
		struct kd_location *l = &t.loc[i];

		dump_buff_len += scnprintf(dump_buff + dump_buff_len,
				BUFLEN(len, dump_buff_len),
				"%ld KB %pS age=%ld/%ld/%ld pid=%ld-%ld\n",
				(l->count * s->object_size) >> 10,
				(void *)l->addr,
				l->min_time,
				(long)div_u64(l->sum_time, l->count),
				l->max_time,
				l->min_pid, l->max_pid);

		for (j = 0; j < l->depth; j++) {
			if (!l->addrs[j])
				break;

			dump_buff_len += scnprintf(dump_buff + dump_buff_len,
					BUFLEN(len, dump_buff_len),
					"%pS\n", (void *)l->addrs[j]);
		}

		dump_buff_len += scnprintf(dump_buff + dump_buff_len,
				BUFLEN(len, dump_buff_len), "-\n");
	}

	kd_free_loc_track(&t);
	if (!t.count)
		dump_buff_len += scnprintf(dump_buff + dump_buff_len,
				BUFLEN_EXT(len, dump_buff_len),
				"[kmalloc_debug]%s no data\n", s->name);

	dump_buff_len += scnprintf(dump_buff + dump_buff_len,
			BUFLEN_EXT(len, dump_buff_len),
			"%s %lu %lu\n", s->name, t.count, t.max);
	dump_buff[dump_buff_len++] = '\n';
out:
	printk("%s", dump_buff);
}

void dump_kmalloc_debug_info(struct kmem_cache *s, int index, char *dump_buff,
		int len)
{
	unsigned int slab_size;

	slab_size = calculate_kmalloc_slab_size(s) >> 20;
	if (slab_size < kmalloc_debug_watermark[index]) {
		pr_warn("[kmalloc_debug]slab %s size %uMB is not over %uMB, ignore it.\n",
				s->name, slab_size, kmalloc_debug_watermark[index]);
		return;
	}

	if (!dump_buff) {
		pr_err("[kmalloc_debug] dump_buff is NULL.\n");
		return;
	}

	kmalloc_debug_watermark[index] += KMALLOC_DEBUG_DUMP_STEP;
	dump_locations(s, slab_size, index, dump_buff, len, TRACK_ALLOC);
}

static inline const char *
kmalloc_debug_cache_name(const char *prefix, unsigned int size)
{
	static const char units[3] = "\0kM";
	int idx = 0;

	while (size >= 1024 && (size % 1024 == 0)) {
		size /= 1024;
		idx++;
	}

	return kasprintf(GFP_NOWAIT, "%s-%u%c", prefix, size, units[idx]);
}

static inline bool check_valid_size(int size, int type)
{
	int i;
	struct kmem_cache *s;

	for (i = 0; i <= KMALLOC_SHIFT_HIGH; i++) {
		s = kmalloc_caches[type][i];
		if (!s)
			continue;

		if (size == s->object_size)
			return true;
	}

	return false;
}

static struct kmem_cache *create_kmalloc_debug_caches(size_t size,
		unsigned long flags, enum kmalloc_cache_type kmalloc_type)
{
	unsigned int index = kmalloc_index(size);
	struct kmem_cache *s;
	const char *name;

	if ((!index) || (index < KMALLOC_SHIFT_LOW) ||
			(index > KMALLOC_SHIFT_HIGH)) {
		pr_warn("kmalloc debug cache create failed size %lu index %d\n",
				size, index);
		return NULL;
	}

	s = kmalloc_caches[kmalloc_type][index];
	if (!s) {
		pr_warn("kmalloc-%lu slab is NULL, do not create debug one\n",
				size);
		return NULL;
	}

	if (s->flags & SLAB_STORE_USER) {
		pr_warn("%s slab is enable SLAB_STORE_USER, do not "\
				"create a new debug slab and size %lu.\n",
				s->name, size);
		return NULL;
	}

	if (kmalloc_type == KMALLOC_RECLAIM)
		flags |= SLAB_RECLAIM_ACCOUNT;

#ifdef CONFIG_ZONE_DMA
	if (kmalloc_type == KMALLOC_DMA) {
		s = (struct kmem_cache *)atomic64_read(&kmalloc_debug_caches[KMALLOC_NORMAL][index]);
		if (!s) {
			pr_warn("Can not create dma kmalloc slab of size %d\n", size);
			return NULL;
		}
		flags |= SLAB_CACHE_DMA;
	}
#endif

	name = kmalloc_debug_info[index].name[kmalloc_type];
	if (!name)
		return NULL;

	s = kmem_cache_create_usercopy(name, kmalloc_debug_info[index].size,
			ARCH_KMALLOC_MINALIGN, flags, 0,
			kmalloc_debug_info[index].size, NULL);

	return s;
}


static ssize_t kmalloc_debug_create_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char *kbuf;
	struct kmem_cache *s;
	int i, type;
	int len = 0;

	kbuf = kzalloc(KMALLOC_DEBUG_R_LEN, GFP_KERNEL);
	if (!kbuf) {
		pr_warn("[kmalloc_debug] %s allo kbuf failed.\n", __func__);
		return -ENOMEM;
	}

	for (type = KMALLOC_NORMAL; type < NR_KMALLOC_TYPES; type++) {
		for (i = 0; i <= KMALLOC_SHIFT_HIGH; i++) {
			s = (struct kmem_cache *)atomic64_read(
					&kmalloc_debug_caches[type][i]);
			if (s)
				len += scnprintf(kbuf+len, KMALLOC_DEBUG_R_LEN - len - 1,
						"%s\n", s->name);
		}
	}

	for (type = KMALLOC_NORMAL; type < NR_KMALLOC_TYPES; type++) {
		for (i = 0; i <= KMALLOC_SHIFT_HIGH; i++) {
			s = kmalloc_caches[type][i];
			if (s && (s->flags & SLAB_STORE_USER))
				len += scnprintf(kbuf+len, KMALLOC_DEBUG_R_LEN - len - 1,
						"%s\n", s->name);
		}
	}

	if ((len > 0) && (kbuf[len - 1] != '\n'))
		kbuf[len++] = '\n';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf, (len < count ? len : count))) {
		kfree(kbuf);
		return -EFAULT;
	}
	kfree(kbuf);

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static inline int parser_kmalloc_cache_type(char *str)
{
	if (!str)
		return -EINVAL;
	if (strncmp(str, "10", 2) == 0)
		return KMALLOC_NORMAL;
	if (strncmp(str, "20", 2) == 0)
		return KMALLOC_RECLAIM;
	if (strncmp(str, "30", 2) == 0)
#ifdef CONFIG_ZONE_DMA
		return KMALLOC_DMA;
#else
		return -EINVAL;
#endif

	return -EINVAL;
}

static inline int type_to_num(int type)
{
	switch (type) {
	case KMALLOC_NORMAL:
		return 10;
	case KMALLOC_RECLAIM:
		return 20;
#ifdef CONFIG_ZONE_DMA
	case KMALLOC_DMA:
		return 30;
#endif
	default:
		return -1;
	}

	return -1;
}

static inline int get_kmalloc_cache_type(char *str, enum kmalloc_str_format *format)
{
	int type;

	if (!str || !format)
		return -EINVAL;

	type = parser_kmalloc_cache_type(str);
	if (type >= KMALLOC_NORMAL) {
		*format = KMALLOC_NUM;
		return type;
	}

	*format = KMALLOC_STR;
	if (!strstr(str, "kmalloc-"))
		return -EINVAL;

	if (strstr(str, "kmalloc-debug-"))
		return -EINVAL;

	if (strstr(str, "kmalloc-rcl-"))
		return KMALLOC_RECLAIM;

	if (strstr(str, "dma-kmalloc-")) {
#ifdef CONFIG_ZONE_DMA
		return KMALLOC_DMA;
#else
		return -EINVAL;
#endif
	}

	return KMALLOC_NORMAL;
}

static ssize_t kmalloc_debug_create_write(struct file *file, const char __user *buff,
		size_t len, loff_t *ppos)
{
	char kbuf_val[64] = {'0'};
	long size;
	int ret, type, offset;
	unsigned int index;
	struct kmem_cache *s;
	char *kbuf;
	enum kmalloc_str_format format;

	if (!kmalloc_debug_enable) {
		pr_err("%s kmalloc_debug_enable is 0\n", __func__);
		return -EPERM;
	}

	if (!buff || len <= 0)
		return -EINVAL;

	if (len > 63)
		len = 63;

	if (copy_from_user(&kbuf_val, buff, len))
		return -EFAULT;
	kbuf_val[len] = '\0';

	kbuf = strstrip(kbuf_val);
	type = get_kmalloc_cache_type(kbuf, &format);
	if (type < 0) {
		pr_err("%s is not a kmalloc slab name\n", kbuf);
		return -EINVAL;
	}

	if (format == KMALLOC_NUM)
		offset = 2;
	else {
		if (type == KMALLOC_NORMAL)
			offset = 8; /* kmalloc-128 */
		else
			offset = 12; /* kmalloc-rcl- or dma-kmalloc- */
	}

	ret = kstrtol(kbuf+offset, 10, &size);
	if (ret)
		return -EINVAL;

	if (!check_valid_size(size, type)) {
		pr_err("kbuf %s slab size %ld is error\n", kbuf, size);
		return -EINVAL;
	}

	index = kmalloc_index(size);
	mutex_lock(&debug_mutex);
	s = (struct kmem_cache *)atomic64_read(&kmalloc_debug_caches[type][index]);
	if (s) {
		pr_warn("slab %s index %d type %d size %lu has been created\n",
				s->name, index, type, size);
		mutex_unlock(&debug_mutex);
		return -EEXIST;
	}

	s = create_kmalloc_debug_caches((size_t)size, SLAB_STORE_USER, type);
	if (!s) {
		mutex_unlock(&debug_mutex);
		return -ENOMEM;
	}
	atomic64_set(&kmalloc_debug_caches[type][index], (unsigned long)s);
	mutex_unlock(&debug_mutex);

	return len;
}

static const struct proc_ops kmalloc_debug_create_operations = {
	.proc_read	= kmalloc_debug_create_read,
	.proc_write     = kmalloc_debug_create_write,
	.proc_lseek		= default_llseek,
};

unsigned long calculate_kmalloc_slab_size(struct kmem_cache *s)
{
	struct slabinfo sinfo;

	memset(&sinfo, 0, sizeof(sinfo));
	get_slabinfo(s, &sinfo);
	return sinfo.num_objs * s->object_size;
}

static void *kmo_start(struct seq_file *m, loff_t *pos)
{
	struct kmem_cache *s = NULL;

	while (*pos <= ALL_KMALLOC_HIGH) {
		s = kmalloc_caches[CACHE_TYPE(*pos)][CACHE_INDEX(*pos)];
		if (s && (s->flags & SLAB_STORE_USER))
			return (void *)s;
		++*pos;
	}

	return NULL;
}

static void *kmo_next(struct seq_file *m, void *p, loff_t *pos)
{
	struct kmem_cache *s = NULL;

	++*pos;
	while (*pos <= ALL_KMALLOC_HIGH) {
		s = kmalloc_caches[CACHE_TYPE(*pos)][CACHE_INDEX(*pos)];
		if (s && (s->flags & SLAB_STORE_USER))
			return (void *)s;
		++*pos;
	}

	return NULL;
}

static void *kmd_start(struct seq_file *m, loff_t *pos)
{
	struct kmem_cache *s = NULL;

	while (*pos <= ALL_KMALLOC_HIGH) {
		s = (struct kmem_cache *)atomic64_read(&kmalloc_debug_caches[CACHE_TYPE(*pos)][CACHE_INDEX(*pos)]);
		if (s && (s->flags & SLAB_STORE_USER))
			return (void *)s;
		++*pos;
	}

	return NULL;
}

static void *kmd_next(struct seq_file *m, void *p, loff_t *pos)
{
	struct kmem_cache *s = NULL;

	++*pos;
	while (*pos <= ALL_KMALLOC_HIGH) {
		s = (struct kmem_cache *)atomic64_read(&kmalloc_debug_caches[CACHE_TYPE(*pos)][CACHE_INDEX(*pos)]);
		if (s && (s->flags & SLAB_STORE_USER))
			return (void *)s;
		++*pos;
	}

	return NULL;
}

static void kmd_stop(struct seq_file *m, void *p)
{
}

static int kmd_show(struct seq_file *m, void *p)
{
	struct kmem_cache *s = (struct kmem_cache *)p;

	(void)kbuf_dump_kmalloc_debug(s, (char *)m->private, DATA_LEN);
	seq_printf(m, "=== slab %s debug info:\n", s->name);
	seq_printf(m, "%s\n", (char *)m->private);

	return 0;
}

static const struct seq_operations kmalloc_debug_op = {
	.start	= kmd_start,
	.next	= kmd_next,
	.show	= kmd_show,
	.stop	= kmd_stop
};

static const struct seq_operations kmalloc_origin_op = {
	.start	= kmo_start,
	.next	= kmo_next,
	.show	= kmd_show,
	.stop	= kmd_stop
};

static int kmalloc_debug_open(struct inode *inode, struct file *file)
{
	void *priv = __seq_open_private(file, &kmalloc_debug_op, DATA_LEN);

	if (!priv)
		return -ENOMEM;

	return 0;
}

static int kmalloc_origin_open(struct inode *inode, struct file *file)
{
	void *priv = __seq_open_private(file, &kmalloc_origin_op, DATA_LEN);

	if (!priv)
		return -ENOMEM;

	return 0;
}

static ssize_t kmalloc_debug_enable_write(struct file *file,
		const char __user *buff, size_t len, loff_t *ppos)
{
	char kbuf[KD_VALUE_LEN] = {'0'};
	long val;
	int ret;

	if (!buff || len <= 0)
		return -EINVAL;

	if (len > (KD_VALUE_LEN - 1))
		len = KD_VALUE_LEN - 1;

	if (copy_from_user(&kbuf, buff, len))
		return -EFAULT;
	kbuf[len] = '\0';

	ret = kstrtol(kbuf, 10, &val);
	if (ret)
		return -EINVAL;

	kmalloc_debug_enable = val ? 1 : 0;
	return len;
}

static ssize_t kmalloc_debug_enable_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[KD_VALUE_LEN] = {'0'};
	int len;

	len = scnprintf(kbuf, KD_VALUE_LEN - 1, "%d\n", kmalloc_debug_enable);
	if (kbuf[len - 1] != '\n')
		kbuf[len++] = '\n';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf + *off, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static __always_inline unsigned int kmalloc_size(unsigned int n)
{
	if (n > 2)
		return 1U << n;

	if (n == 1 && KMALLOC_MIN_SIZE <= 32)
		return 96;

	if (n == 2 && KMALLOC_MIN_SIZE <= 64)
		return 192;

	return 0;
}

static ssize_t kmalloc_used_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char *kbuf = NULL;
	int len = 0;
	int i, type, ret;

	kbuf = vzalloc(KMALLOC_USED_LEN);
	if (!kbuf)
		return -ENOMEM;

	for (i = 0; i <= KMALLOC_SHIFT_HIGH; i++) {
		for(type = KMALLOC_NORMAL; type < NR_KMALLOC_TYPES; type++) {
			struct kmem_cache *s;
			unsigned long slab_size = 0;

			s = kmalloc_caches[type][i];
			if (s)
				slab_size = calculate_kmalloc_slab_size(s);

			s = (struct kmem_cache *)atomic64_read(&kmalloc_debug_caches[type][i]);
			if (s)
				slab_size += calculate_kmalloc_slab_size(s);

			if (slab_size != 0)
				len += scnprintf(&kbuf[len],
						KMALLOC_USED_LEN - 1 - len, "%d%-19d %lu\n",
						type_to_num(type),
						kmalloc_size(i),
						slab_size >> 10);

			if (len >= KMALLOC_USED_LEN - 1)
				break;
		}
	}

	if ((len > 0) && (kbuf[len - 1] != '\n'))
		kbuf[len++] = '\n';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	ret = copy_to_user(buffer, kbuf + *off, (len < count ? len : count));
	vfree(kbuf);
	if (ret)
		return -EFAULT;
	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static const struct proc_ops kmalloc_used_ops = {
	.proc_read	= kmalloc_used_read,
	.proc_lseek		= default_llseek,
};

static const struct proc_ops kmalloc_debug_enable_operations = {
	.proc_write     = kmalloc_debug_enable_write,
	.proc_read	= kmalloc_debug_enable_read,
	.proc_lseek	= default_llseek,
};

static const struct proc_ops kmalloc_debug_operations = {
	.proc_open	= kmalloc_debug_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= seq_release_private,
};

static const struct proc_ops kmalloc_origin_operations = {
	.proc_open	= kmalloc_origin_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= seq_release_private,
};

int __init create_kmalloc_debug(struct proc_dir_entry *parent)
{
	dpentry = proc_create("kmalloc_debug", S_IRUGO, parent,
			&kmalloc_debug_operations);
	if (!dpentry) {
		pr_err("create kmalloc_debug proc failed.\n");
		return -ENOMEM;
	}

	opentry = proc_create("kmalloc_origin", S_IRUGO, parent,
			&kmalloc_origin_operations);
	if (!opentry) {
		pr_err("create kmalloc_origin proc failed.\n");
		goto remove_dpentry;
	}

	epentry = proc_create("kmalloc_debug_enable",
			S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH,
			parent,	&kmalloc_debug_enable_operations);
	if (!epentry) {
		pr_err("create kmalloc_debug_enable proc failed.\n");
		goto remove_opentry;
	}

	upentry = proc_create("kmalloc_used", S_IRUGO, parent,
			&kmalloc_used_ops);
	if (!upentry) {
		pr_err("create kmalloc_used proc failed.\n");
		goto remove_epentry;
	}

	/* add new proc interface for create kmalloc debug caches.  */
	cpentry = proc_create("kmalloc_debug_create", S_IRUGO|S_IWUGO, parent,
			&kmalloc_debug_create_operations);
	if (!cpentry) {
		pr_err("create kmalloc_debug_create proc failed.\n");
		goto remove_upentry;
	}

	mpentry = proc_create("memleak_detect_thread", S_IRUGO|S_IWUGO, parent,
			&memleak_detect_thread_operations);
	if (!cpentry) {
		pr_err("create memleak_detect_thread_operations proc failed.\n");
		goto remove_cpentry;
	}

	return 0;

remove_cpentry:
	proc_remove(cpentry);
	cpentry = NULL;
remove_upentry:
	proc_remove(upentry);
	upentry = NULL;
remove_epentry:
	proc_remove(epentry);
	epentry = NULL;
remove_opentry:
	proc_remove(opentry);
	opentry = NULL;
remove_dpentry:
	proc_remove(dpentry);
	dpentry = NULL;
	return -ENOMEM;
}

void destroy_kmalloc_debug(void)
{
	proc_remove(mpentry);
	proc_remove(cpentry);
	proc_remove(upentry);
	proc_remove(epentry);
	proc_remove(opentry);
	proc_remove(dpentry);
}

static inline int enable_kmalloc_debug(void)
{
	int ret;

	ret = register_trace_android_vh_save_track_hash(save_track_hash_hook,
			NULL);
	if (ret)
		pr_err("rgister save_track_hash_hook failed, rc:%d\n", ret);

	ret = register_trace_android_vh_kmalloc_slab(kmalloc_slab_hook, NULL);
	if (ret) {
		unregister_trace_android_vh_save_track_hash(save_track_hash_hook,
				NULL);
		pr_err("regsiter kmalloc_slab_hook failed, rc:%d\n", ret);
	}

	pr_err("enable_kmalloc_debug return %d\n", ret);

	return ret;
}

static inline void disable_kmalloc_debug(void)
{
	unregister_trace_android_vh_save_track_hash(save_track_hash_hook, NULL);
	unregister_trace_android_vh_kmalloc_slab(kmalloc_slab_hook, NULL);
}

static int memleak_detect_thread(void *arg)
{
	long ret = 0;
	long sleep_jiffies = MEMLEAK_DETECT_SLEEP_SEC;
	int i, type;

	do {
		char *dump_buff = NULL;

		current->__state = TASK_INTERRUPTIBLE;
		ret = schedule_timeout(sleep_jiffies);
		if (ret) {
			sleep_jiffies = ret;
			continue;
		}
		sleep_jiffies = MEMLEAK_DETECT_SLEEP_SEC;

		dump_buff = (char *)vmalloc(DUMP_BUFF_LEN);
		if (!dump_buff) {
			pr_err("[%s] vmalloc dump_buff failed.\n", __func__);
			continue;
		}

		for (type = KMALLOC_NORMAL; type < NR_KMALLOC_TYPES; type++) {
			for (i = 0; i <= KMALLOC_SHIFT_HIGH; i++) {
				struct kmem_cache *s = kmalloc_caches[type][i];

				if (!s)
					continue;
				if (s->flags & SLAB_STORE_USER)
					dump_kmalloc_debug_info(s, i, dump_buff,
							DUMP_BUFF_LEN);
			}
		}

		for (type = KMALLOC_NORMAL; type < NR_KMALLOC_TYPES; type++) {
			for (i = 0; i <= KMALLOC_SHIFT_HIGH; i++) {
				struct kmem_cache *s = (struct kmem_cache *)atomic64_read(&kmalloc_debug_caches[type][i]);

				if (!s)
					continue;
				if (s->flags & SLAB_STORE_USER)
					dump_kmalloc_debug_info(s, i, dump_buff,
							DUMP_BUFF_LEN);
			}
		}

		vfree(dump_buff);
	} while (!kthread_should_stop());

	return 0;
}

static inline void init_kmalloc_debug_caches(unsigned long flags)
{
	static int inited = 0;
	int i, type;
	struct kmem_cache *s;

	if (inited)
		return;

	kmalloc_debug_watermark_init();
	for (type = KMALLOC_NORMAL; type <= KMALLOC_RECLAIM; type++) {
		for (i = KMALLOC_SHIFT_LOW; i <= KMALLOC_SHIFT_HIGH; i++) {
			if (atomic64_read(&kmalloc_debug_caches[type][i]))
				continue;

			s = create_kmalloc_debug_caches(kmalloc_debug_info[i].size,
					flags, type);

			/*
			 * Caches that are not of the two-to-the-power-of size.
			 * These have to be created immediately after the
			 * earlier power of two caches
			 */
			if (KMALLOC_MIN_SIZE <= 32 && !kmalloc_caches[type][1] && i == 6)
				s = create_kmalloc_debug_caches(kmalloc_debug_info[1].size,
						flags, type);
			if (KMALLOC_MIN_SIZE <= 64 && !kmalloc_caches[type][2] && i == 7)
				s = create_kmalloc_debug_caches(kmalloc_debug_info[2].size,
						flags, type);

			if (!s)
				break;

			atomic64_set(&kmalloc_debug_caches[type][i], (unsigned long)s);
		}
	}

#ifdef CONFIG_ZONE_DMA
	type = KMALLOC_DMA;
	for (i = 0; i <= KMALLOC_SHIFT_HIGH; i++) {
		s = (struct kmem_cache *)atomic64_read(&kmalloc_debug_caches[KMALLOC_NORMAL][i]);

		if (s) {
			s = create_kmalloc_debug_caches(
					kmalloc_debug_info[i].size,
					SLAB_CACHE_DMA | flags, type);
			if (s) {
				atomic64_set(&kmalloc_debug_caches[type][i], (unsigned long)s);
			}
		}
	}
#endif

	inited = 1;
}

static ssize_t memleak_detect_thread_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[16] = {'\0'};
	int len = 0;

	if (!buffer || count <= 0)
		return -EINVAL;

	if (count > 15)
		count = 15;

	len = scnprintf(kbuf, count, "%d\n", memleak_detect_task ? 1 : 0);
	kbuf[len++] = '\0';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf, len))
		return -EFAULT;

	*off += len;
	return len;
}

static ssize_t memleak_detect_thread_write(struct file *file, const char __user *buff,
		size_t len, loff_t *ppos)
{
	char kbuf[16] = {'0'};
	long val;
	int ret;

	if (!buff || len <= 0)
		return -EINVAL;

	if (len > 15)
		len = 15;

	if (copy_from_user(&kbuf, buff, len))
		return -EFAULT;
	kbuf[len] = '\0';

	ret = kstrtol(kbuf, 10, &val);
	if (ret)
		return -EINVAL;

	mutex_lock(&debug_mutex);
	if (val > 0) {
		if (memleak_detect_task)
			goto out;
		init_kmalloc_debug_caches(SLAB_STORE_USER);
		kmalloc_debug_enable = 1;

		memleak_detect_task = kthread_create(memleak_detect_thread,
				NULL, "memleak_detect");
		if (IS_ERR(memleak_detect_task)) {
			pr_warn("[kmalloc_debug][vmalloc_debug] memleak_detect_init failed.\n");
			memleak_detect_task = NULL;
		} else
			wake_up_process(memleak_detect_task);
	} else if (memleak_detect_task) {
		kmalloc_debug_enable = 0;
		kthread_stop(memleak_detect_task);
		memleak_detect_task = NULL;
	}

out:
	mutex_unlock(&debug_mutex);
	return len;
}

static const struct proc_ops memleak_detect_thread_operations = {
	.proc_read	= memleak_detect_thread_read,
	.proc_write	= memleak_detect_thread_write,
	.proc_lseek	= default_llseek,
};

static int __init memleak_detect_init(void)
{
	int ret;

	oplus_mem_dir = proc_mkdir("oplus_mem", NULL);
	if (!oplus_mem_dir)
		memleak_detect_dir = proc_mkdir("oplus_mem/memleak_detect", NULL);
	else
		memleak_detect_dir = proc_mkdir("memleak_detect", oplus_mem_dir);
	if (!memleak_detect_dir) {
		pr_err("create memleak_detect proc failed.\n");
		return -ENOMEM;
	}

	if (kmalloc_debug) {
		ret = create_kmalloc_debug(memleak_detect_dir);
		if (ret) {
			pr_err("create_kmalloc_debug failed\n");
			goto fail_out;
		}

		ret = enable_kmalloc_debug();
		if (ret)
			goto fail_out;
	}

	if (vmalloc_debug) {
		ret = create_vmalloc_debug(memleak_detect_dir);
		if (ret) {
			pr_err("create_vmalloc_debug failed\n");
			goto destroy_kmalloc_debug;
		}

		ret = init_vmalloc_debug();
		if (ret)
			goto destroy_kmalloc_debug;
	}

	if (kmalloc_debug && daemon_thread) {
		init_kmalloc_debug_caches(SLAB_STORE_USER);
		kmalloc_debug_enable = 1;

		memleak_detect_task = kthread_create(memleak_detect_thread,
				NULL, "memleak_detect");
		if (IS_ERR(memleak_detect_task)) {
			pr_warn("[kmalloc_debug][vmalloc_debug] memleak_detect_init failed.\n");
			memleak_detect_task = NULL;
			ret = -ENOMEM;
			goto destroy_vmalloc_debug;
		} else
			wake_up_process(memleak_detect_task);
	}

	return 0;

destroy_vmalloc_debug:
	if (vmalloc_debug) {
		disable_vmalloc_debug();
		vmalloc_debug_exit();
		ml_depot_destory();
	}
destroy_kmalloc_debug:
	if (kmalloc_debug)
		destroy_kmalloc_debug();
fail_out:
	proc_remove(memleak_detect_dir);
	return ret;
}

static void memleak_detect_exit(void)
{
	if (daemon_thread && memleak_detect_task) {
		kthread_stop(memleak_detect_task);
		memleak_detect_task = NULL;
	}

	if (kmalloc_debug)
		destroy_kmalloc_debug();

	if (vmalloc_debug) {
		disable_vmalloc_debug();
		vmalloc_debug_exit();
		ml_depot_destory();
	}

	proc_remove(memleak_detect_dir);
}

module_init(memleak_detect_init);
module_exit(memleak_detect_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("kmalloc vmalloc memleak debugging feature");
MODULE_IMPORT_NS(MINIDUMP);
#endif /* _SLUB_TRACK_ */
