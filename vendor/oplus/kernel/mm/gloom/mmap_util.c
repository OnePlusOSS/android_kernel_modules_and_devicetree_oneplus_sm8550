// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/compiler.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/signal.h>
#include <linux/sched/task_stack.h>
#include <linux/security.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/mman.h>
#include <linux/hugetlb.h>
#include <linux/vmalloc.h>
#include <linux/userfaultfd_k.h>
#include <linux/elf.h>
#include <linux/elf-randomize.h>
#include <linux/personality.h>
#include <linux/random.h>
#include <linux/processor.h>
#include <linux/sizes.h>
#include <linux/compat.h>
#include <linux/uaccess.h>
#include <linux/android_debug_symbols.h>

#ifndef STACK_RND_MASK
#define STACK_RND_MASK (0x7ff >> (PAGE_SHIFT - 12))     /* 8MB of VA */
#endif

#define MIN_GAP		(SZ_128M)
#define MAX_GAP		(STACK_TOP / 6 * 5)

static int mmap_is_legacy(struct rlimit *rlim_stack)
{
	if (current->personality & ADDR_COMPAT_LAYOUT)
		return 1;

	if (rlim_stack->rlim_cur == RLIM_INFINITY)
		return 1;

	return *(int *)android_debug_symbol(ADS_SYSCTL_LEGACY_VA_LAYOUT);
}

static unsigned long mmap_base(unsigned long rnd, struct rlimit *rlim_stack)
{
	unsigned long gap = rlim_stack->rlim_cur;
	unsigned long pad = *(unsigned long *)android_debug_symbol(ADS_STACK_GUARD_GAP);

	/* Account for stack randomization if necessary */
	if (current->flags & PF_RANDOMIZE)
		pad += (STACK_RND_MASK << PAGE_SHIFT);

	/* Values close to RLIM_INFINITY can overflow. */
	if (gap + pad > gap)
		gap += pad;

	if (gap < MIN_GAP)
		gap = MIN_GAP;
	else if (gap > MAX_GAP)
		gap = MAX_GAP;

	return PAGE_ALIGN(STACK_TOP - gap - rnd);
}

void special_arch_pick_mmap_layout(struct mm_struct *mm)
{
	unsigned long random_factor = 0UL;
	unsigned long old_mmap_base = mm->mmap_base;
	unsigned long new_mmap_base;
	struct rlimit *rlim_stack = &current->signal->rlim[RLIMIT_STACK];

	if ((current->flags & PF_RANDOMIZE)
			&& !mmap_is_legacy(rlim_stack)) {
		random_factor = arch_mmap_rnd() % (1 << 25);
		new_mmap_base = mmap_base(random_factor, rlim_stack);
		/* pr_info("gloom special_arch_pick_mmap_layout pid = %d, mmap_base 0x%lx -> 0x%lx\n", current->pid, old_mmap_base, new_mmap_base); */
		mm->mmap_base = max_t(unsigned long, new_mmap_base, old_mmap_base);
	}
}
