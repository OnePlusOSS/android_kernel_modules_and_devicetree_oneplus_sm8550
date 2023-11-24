// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "vmalloc_debug: " fmt

#include <linux/gfp.h>
#include <linux/jhash.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/percpu.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/stacktrace.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "memleak_debug_stackdepot.h"

#define VD_VALUE_LEN 32
#define DEPOT_STACK_BITS (sizeof(ml_depot_stack_handle_t) * 8)

#define STACK_ALLOC_NULL_PROTECTION_BITS 1
#define STACK_ALLOC_ORDER 0 /* 'Slab' size order for stack depot, 1 pages */
#define STACK_ALLOC_SIZE (1LL << (PAGE_SHIFT + STACK_ALLOC_ORDER))
#define STACK_ALLOC_ALIGN 4
#define STACK_ALLOC_OFFSET_BITS (STACK_ALLOC_ORDER + PAGE_SHIFT - \
		STACK_ALLOC_ALIGN)
#define STACK_ALLOC_INDEX_BITS (DEPOT_STACK_BITS - \
		STACK_ALLOC_NULL_PROTECTION_BITS - STACK_ALLOC_OFFSET_BITS)
#define STACK_ALLOC_SLABS_CAP 10240
#define STACK_ALLOC_MAX_SLABS \
	(((1LL << (STACK_ALLOC_INDEX_BITS)) < STACK_ALLOC_SLABS_CAP) ? \
	 (1LL << (STACK_ALLOC_INDEX_BITS)) : STACK_ALLOC_SLABS_CAP)

/* The compact structure to store the reference to stacks. */
union ml_handle_parts {
	ml_depot_stack_handle_t handle;
	struct {
		u32 slabindex : STACK_ALLOC_INDEX_BITS;
		u32 offset : STACK_ALLOC_OFFSET_BITS;
		u32 valid : STACK_ALLOC_NULL_PROTECTION_BITS;
	};
};

struct ml_stack_record {
	struct ml_stack_record *next;	/* Link in the hashtable */
	u32 hash;			/* Hash in the hastable */
	u32 size;			/* Number of frames in the stack */
	union ml_handle_parts handle;
	unsigned long entries[1];	/* Variable-sized array of entries. */
};

static void **ml_stack_slabs = NULL;

static int ml_depot_index;
static int ml_next_slab_inited;
static size_t ml_depot_offset;
static DEFINE_SPINLOCK(ml_depot_lock);
static DEFINE_MUTEX(ml_depot_init_mutex);
static atomic_t ml_stack_depot_inited = ATOMIC_INIT(0);
static long want_search_hash = -1;
static DEFINE_MUTEX(search_hash_lock);
static struct proc_dir_entry *hpentry;

static bool ml_init_stack_slab(void **prealloc)
{
	if (!*prealloc)
		return false;
	/*
	 * This smp_load_acquire() pairs with smp_store_release() to
	 * |ml_next_slab_inited| below and in ml_depot_alloc_stack().
	 */
	if (smp_load_acquire(&ml_next_slab_inited))
		return true;
	if (ml_stack_slabs[ml_depot_index] == NULL) {
		ml_stack_slabs[ml_depot_index] = *prealloc;
	} else {
		ml_stack_slabs[ml_depot_index + 1] = *prealloc;
		/*
		 * This smp_store_release pairs with smp_load_acquire() from
		 * |ml_next_slab_inited| above and in ml_depot_save_stack().
		 */
		smp_store_release(&ml_next_slab_inited, 1);
	}
	*prealloc = NULL;
	return true;
}

/* Allocation of a new stack in raw storage */
static struct ml_stack_record *ml_depot_alloc_stack(unsigned long *entries, int size,
		u32 hash, void **prealloc, gfp_t alloc_flags)
{
	int required_size = offsetof(struct ml_stack_record, entries) +
		sizeof(unsigned long) * size;
	struct ml_stack_record *stack;

	required_size = ALIGN(required_size, 1 << STACK_ALLOC_ALIGN);

	if (unlikely(ml_depot_offset + required_size > STACK_ALLOC_SIZE)) {
		if (unlikely(ml_depot_index + 1 >= STACK_ALLOC_MAX_SLABS)) {
			WARN_ONCE(1, "Stack depot reached limit capacity");
			return NULL;
		}
		ml_depot_index++;
		ml_depot_offset = 0;
		/*
		 * smp_store_release() here pairs with smp_load_acquire() from
		 * |ml_next_slab_inited| in ml_depot_save_stack() and
		 * ml_init_stack_slab().
		 */
		if (ml_depot_index + 1 < STACK_ALLOC_MAX_SLABS)
			smp_store_release(&ml_next_slab_inited, 0);
	}
	ml_init_stack_slab(prealloc);
	if (ml_stack_slabs[ml_depot_index] == NULL)
		return NULL;

	stack = ml_stack_slabs[ml_depot_index] + ml_depot_offset;

	stack->hash = hash;
	stack->size = size;
	stack->handle.slabindex = ml_depot_index;
	stack->handle.offset = ml_depot_offset >> STACK_ALLOC_ALIGN;
	stack->handle.valid = 1;
	memcpy(stack->entries, entries, size * sizeof(unsigned long));
	ml_depot_offset += required_size;

	return stack;
}

#define STACK_HASH_SIZE (1L << 19)
#define STACK_HASH_MASK (STACK_HASH_SIZE - 1)
#define STACK_HASH_SEED 0x9747b28c

static struct ml_stack_record **ml_stack_table = NULL;

/* Calculate hash for a stack */
static inline u32 ml_hash_stack(unsigned long *entries, unsigned int size)
{
	return jhash2((u32 *)entries,
			array_size(size,  sizeof(*entries)) / sizeof(u32),
			STACK_HASH_SEED);
}

/* Use our own, non-instrumented version of memcmp().
 *
 * We actually don't care about the order, just the equality.
 */
	static inline
int ml_stackdepot_memcmp(const unsigned long *u1, const unsigned long *u2,
		unsigned int n)
{
	for (; n-- ; u1++, u2++) {
		if (*u1 != *u2)
			return 1;
	}
	return 0;
}

/* Find a stack that is equal to the one stored in entries in the hash */
static inline struct ml_stack_record *ml_find_stack(struct ml_stack_record *bucket,
		unsigned long *entries, int size,
		u32 hash)
{
	struct ml_stack_record *found;

	for (found = bucket; found; found = found->next) {
		if (found->hash == hash &&
				found->size == size &&
				!ml_stackdepot_memcmp(entries, found->entries, size))
			return found;
	}
	return NULL;
}

/**
 * stack_depot_fetch - Fetch stack entries from a depot
 *
 * @handle:		Stack depot handle which was returned from
 *			stack_depot_save().
 * @entries:		Pointer to store the entries address
 *
 * Return: The number of trace entries for this depot.
 */
unsigned int ml_depot_fetch_stack(ml_depot_stack_handle_t handle,
		unsigned long **entries)
{
	union ml_handle_parts parts = { .handle = handle };
	void *slab;
	size_t offset = parts.offset << STACK_ALLOC_ALIGN;
	struct ml_stack_record *stack;

	*entries = NULL;
	if (atomic_read(&ml_stack_depot_inited) == 0) {
		pr_err("ml_stack_depot_inited is not inited\n");
		return 0;
	}

	if (parts.slabindex > ml_depot_index) {
		WARN(1, "slab index %d out of bounds (%d) for stack id %08x\n",
				parts.slabindex, ml_depot_index, handle);
		return 0;
	}
	slab = ml_stack_slabs[parts.slabindex];
	if (!slab)
		return 0;
	stack = slab + offset;

	*entries = stack->entries;
	return stack->size;
}

static struct ml_stack_record *ml_depot_find_stack(ml_depot_stack_handle_t handle)
{
	union ml_handle_parts parts = { .handle = handle };
	void *slab;
	size_t offset = parts.offset << STACK_ALLOC_ALIGN;
	struct ml_stack_record *stack;

	if (atomic_read(&ml_stack_depot_inited) == 0) {
		pr_err("ml_stack_depot_inited is not inited\n");
		return NULL;
	}

	if (parts.slabindex > ml_depot_index) {
		WARN(1, "slab index %d out of bounds (%d) for stack id %08x\n",
				parts.slabindex, ml_depot_index, handle);
		return NULL;
	}
	slab = ml_stack_slabs[parts.slabindex];
	if (!slab)
		return NULL;
	stack = slab + offset;
	return (struct ml_stack_record *)stack;
}
/**
 * stack_depot_save - Save a stack trace from an array
 *
 * @entries:		Pointer to storage array
 * @nr_entries:		Size of the storage array
 * @alloc_flags:	Allocation gfp flags
 *
 * Return: The handle of the stack struct stored in depot
 */
ml_depot_stack_handle_t ml_depot_save_stack(unsigned long *entries,
		unsigned int nr_entries, gfp_t alloc_flags)
{
	u32 hash;
	ml_depot_stack_handle_t retval = 0;
	struct ml_stack_record *found = NULL, **bucket;
	unsigned long flags;
	struct page *page = NULL;
	void *prealloc = NULL;

	if (atomic_read(&ml_stack_depot_inited) == 0) {
		pr_err("ml_stack_depot_inited is not inited\n");
		goto fast_exit;
	}

	if (unlikely(nr_entries == 0))
		goto fast_exit;

	hash = ml_hash_stack(entries, nr_entries);
	bucket = &ml_stack_table[hash & STACK_HASH_MASK];

	/*
	 * Fast path: look the stack trace up without locking.
	 * The smp_load_acquire() here pairs with smp_store_release() to
	 * |bucket| below.
	 */
	found = ml_find_stack(smp_load_acquire(bucket), entries,
			nr_entries, hash);
	if (found)
		goto exit;

	/*
	 * Check if the current or the next stack slab need to be initialized.
	 * If so, allocate the memory - we won't be able to do that under the
	 * lock.
	 *
	 * The smp_load_acquire() here pairs with smp_store_release() to
	 * |ml_next_slab_inited| in ml_depot_alloc_stack() and ml_init_stack_slab().
	 */
	if (unlikely(!smp_load_acquire(&ml_next_slab_inited))) {
		/*
		 * Zero out zone modifiers, as we don't have specific zone
		 * requirements. Keep the flags related to allocation in atomic
		 * contexts and I/O.
		 */
		alloc_flags &= ~GFP_ZONEMASK;
		alloc_flags &= (GFP_ATOMIC | GFP_KERNEL);
		alloc_flags |= __GFP_NOWARN;
		page = alloc_pages(alloc_flags, STACK_ALLOC_ORDER);
		if (page)
			prealloc = page_address(page);
	}

	spin_lock_irqsave(&ml_depot_lock, flags);
	found = ml_find_stack(*bucket, entries, nr_entries, hash);
	if (!found) {
		struct ml_stack_record *new =
			ml_depot_alloc_stack(entries, nr_entries,
					hash, &prealloc, alloc_flags);
		if (new) {
			new->next = *bucket;
			/*
			 * This smp_store_release() pairs with
			 * smp_load_acquire() from |bucket| above.
			 */
			smp_store_release(bucket, new);
			found = new;
		}
	} else if (prealloc) {
		/*
		 * We didn't need to store this stack trace, but let's keep
		 * the preallocated memory for the future.
		 */
		WARN_ON(!ml_init_stack_slab(&prealloc));
	}
	spin_unlock_irqrestore(&ml_depot_lock, flags);

	if (prealloc) {
		/* Nobody used this memory, ok to free it. */
		free_pages((unsigned long)prealloc, STACK_ALLOC_ORDER);
	}

exit:
	if (found)
		retval = found->handle.handle;
fast_exit:
	return retval;
}

static ssize_t stack_hash_write(struct file *file,
		const char __user *buff, size_t len, loff_t *ppos)
{
	char kbuf[VD_VALUE_LEN] = {'0'};
	long val;
	int ret;

	if (!buff || (len <= 0))
		return -EINVAL;

	if (len > (VD_VALUE_LEN - 1))
		len = VD_VALUE_LEN - 1;

	if (copy_from_user(&kbuf, buff, len))
		return -EFAULT;
	kbuf[len] = '\0';

	ret = kstrtol(kbuf, 10, &val);
	if (ret)
		return -EINVAL;

	if (val < -1)
		return -EINVAL;

	want_search_hash = val;

	return len;
}

static void *stack_hash_start(struct seq_file *m, loff_t *pos)
{
	struct ml_stack_record *s = NULL;
	struct ml_stack_record **bucket;

	if (want_search_hash >= 0) {
		s = ml_depot_find_stack(want_search_hash);
		*pos = STACK_HASH_SIZE;
		return s;
	}

	if (atomic_read(&ml_stack_depot_inited) == 0) {
		pr_err("ml_stack_depot_inited is not inited\n");
		return (void *)-EINVAL;
	}

	while (*pos < STACK_HASH_SIZE) {
		bucket = &ml_stack_table[*pos];
		if (*bucket) {
			s = *bucket;
			return s;
		}
		++*pos;
	}

	return NULL;
}

static void *stack_hash_next(struct seq_file *m, void *p, loff_t *pos)
{
	struct ml_stack_record *s = NULL;
	struct ml_stack_record **bucket;

	if (want_search_hash >= 0)
		return NULL;

	s = ((struct ml_stack_record *)p)->next;
	if (s)
		return s;

	++*pos;
	while (*pos < STACK_HASH_SIZE) {
		bucket = &ml_stack_table[*pos];
		if (*bucket) {
			s = *bucket;
			return s;
		}
		++*pos;
	}

	return NULL;
}

static void stack_hash_stop(struct seq_file *m, void *p)
{
	if (want_search_hash >= 0)
		want_search_hash = -1;
}

static int stack_hash_show(struct seq_file *m, void *p)
{
	int j;
	struct ml_stack_record *s = (struct ml_stack_record *)p;

	seq_printf(m, "hash %u depth %d:\n", s->handle.handle, s->size);
	for (j = 0; j < s->size; j++)
		seq_printf(m, " %pS\n", (void *)s->entries[j]);

	return 0;
}

static const struct seq_operations stack_hash_op = {
	.start	= stack_hash_start,
	.next	= stack_hash_next,
	.show	= stack_hash_show,
	.stop	= stack_hash_stop
};

static int stack_hash_open(struct inode *inode, struct file *file)
{
	int ret = seq_open(file, &stack_hash_op);

	if (ret)
		return ret;

	mutex_lock(&search_hash_lock);
	return 0;
}

static  int stack_hash_release(struct inode *inode, struct file *file)
{
	seq_release(inode, file);
	mutex_unlock(&search_hash_lock);
	return 0;
}

static const struct proc_ops hash_val_ops = {
	.proc_open	= stack_hash_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= stack_hash_release,
	.proc_write	= stack_hash_write,
};

int ml_depot_init(struct proc_dir_entry *parent)
{
	int inited;
	unsigned int size;

	mutex_lock(&ml_depot_init_mutex);
	inited = atomic_read(&ml_stack_depot_inited);
	if (inited != 0) {
		pr_err("ml_stack_depot_inited is inited, val %d\n", inited);
		mutex_unlock(&ml_depot_init_mutex);
		return 0;
	}

	size = sizeof(void *) * STACK_ALLOC_MAX_SLABS;
	ml_stack_slabs = (void **)vmalloc(size);
	if (!ml_stack_slabs) {
		pr_err("vmalloc ml_stack_slabs %d failed.\n", size);
		goto err_out;
	}
	memset(ml_stack_slabs, 0, size);

	size = sizeof(struct ml_stack_record *) * STACK_HASH_SIZE;
	ml_stack_table = (struct ml_stack_record **)vmalloc(size);
	if (!ml_stack_table) {
		pr_err("vmalloc ml_stack_table %d failed.\n", size);
		goto err_out;
	}
	memset(ml_stack_table, 0, size);

	hpentry = proc_create("stack_hash", S_IRUGO|S_IWUGO, parent, &hash_val_ops);
	if (!hpentry) {
		pr_err("create stack_hash proc failed.\n");
		goto err_out;
	}

	atomic_set(&ml_stack_depot_inited, 1);
	mutex_unlock(&ml_depot_init_mutex);

	printk("ml_stack_depot_inited init success.\n");
	return 0;

err_out:
	if (ml_stack_slabs) {
		vfree(ml_stack_slabs);
		ml_stack_slabs = NULL;
	}

	if (ml_stack_table) {
		vfree(ml_stack_table);
		ml_stack_table = NULL;
	}
	mutex_unlock(&ml_depot_init_mutex);

	return -ENOMEM;
}

void ml_depot_destory(void)
{
	unsigned long flags;
	int i;

	mutex_lock(&ml_depot_init_mutex);
	if (atomic_read(&ml_stack_depot_inited) == 0) {
		mutex_unlock(&ml_depot_init_mutex);
		pr_err("[%s] ml_stack_depot_inited is not inited\n", __func__);
		return;
	}

	spin_lock_irqsave(&ml_depot_lock, flags);
	atomic_set(&ml_stack_depot_inited, 0);
	for (i = 0; i < STACK_ALLOC_MAX_SLABS; i++) {
		if (ml_stack_slabs[i]) {
			free_pages((unsigned long)ml_stack_slabs[i], STACK_ALLOC_ORDER);
			ml_stack_slabs[i] = NULL;
		}
	}

	vfree(ml_stack_slabs);
	ml_stack_slabs = NULL;

	vfree(ml_stack_table);
	ml_stack_table = NULL;
	spin_unlock_irqrestore(&ml_depot_lock, flags);

	proc_remove(hpentry);
	mutex_unlock(&ml_depot_init_mutex);
}
