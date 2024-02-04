// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2014 Sergey Senozhatsky.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/cpu.h>
#include <linux/crypto.h>
#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
#include <linux/mm.h>
#endif

#include "zcomp.h"

static const char * const backends[] = {
	"lzo",
	"lzo-rle",
#if IS_ENABLED(CONFIG_CRYPTO_LZ4)
	"lz4",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_LZ4HC)
	"lz4hc",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_842)
	"842",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_ZSTD)
	"zstd",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_ZSTDN)
	"zstdn",
#endif
};

#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
static DEFINE_PER_CPU(void *, zstrm_buffer);

/* create zstrm buffer(order=5) when zcomp_create instead of zcomp_cpu_up_prepare*/
int zcomp_create_thp_zstrm_buffer(void)
{
	int i;
	for_each_possible_cpu(i) {
		per_cpu(zstrm_buffer, i) = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO|__GFP_COMP, (HPAGE_CONT_PTE_ORDER+1));
		if (!per_cpu(zstrm_buffer, i)) {
			if (i) {
				for (i--; i >= 0; i--)
					free_pages((unsigned long)per_cpu(zstrm_buffer, i), (HPAGE_CONT_PTE_ORDER+1));
			}
			return -ENOMEM;
		}
	}
	return 0;
}

void zcomp_destroy_thp_zstrm_buffer(void)
{
	int i;
	for_each_possible_cpu(i) {
		free_pages((unsigned long)per_cpu(zstrm_buffer, i), (HPAGE_CONT_PTE_ORDER+1));
	}
}
#endif
static void zcomp_strm_free(struct zcomp_strm *zstrm
#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
, struct zcomp *comp
#endif
)
{
	if (!IS_ERR_OR_NULL(zstrm->tfm))
		crypto_free_comp(zstrm->tfm);
#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
	if(comp->is_thp_comp == false)
		free_pages((unsigned long)zstrm->buffer, 1);
#else
	free_pages((unsigned long)zstrm->buffer, 1);
#endif

	zstrm->tfm = NULL;
	zstrm->buffer = NULL;
}

/*
 * Initialize zcomp_strm structure with ->tfm initialized by backend, and
 * ->buffer. Return a negative value on error.
 */
static int zcomp_strm_init(struct zcomp_strm *zstrm, struct zcomp *comp
#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
, unsigned int cpu
#endif
)
{
	zstrm->tfm = crypto_alloc_comp(comp->name, 0, 0);
	/*
	 * allocate 2 pages. 1 for compressed data, plus 1 extra for the
	 * case when compressed size is larger than the original one
	 */
#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
	if(comp->is_thp_comp == false)
		zstrm->buffer = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, 1);
	else
		zstrm->buffer = per_cpu(zstrm_buffer, cpu);
#else
	zstrm->buffer = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, 1);
#endif

	if (IS_ERR_OR_NULL(zstrm->tfm) || !zstrm->buffer) {
#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
		zcomp_strm_free(zstrm,comp);
#else
		zcomp_strm_free(zstrm);
#endif
		return -ENOMEM;
	}
	return 0;
}

bool zcomp_available_algorithm(const char *comp)
{
	int i;

	i = sysfs_match_string(backends, comp);
	if (i >= 0)
		return true;

	/*
	 * Crypto does not ignore a trailing new line symbol,
	 * so make sure you don't supply a string containing
	 * one.
	 * This also means that we permit zcomp initialisation
	 * with any compressing algorithm known to crypto api.
	 */
	return crypto_has_comp(comp, 0, 0) == 1;
}

/* show available compressors */
ssize_t zcomp_available_show(const char *comp, char *buf)
{
	bool known_algorithm = false;
	ssize_t sz = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(backends); i++) {
		if (!strcmp(comp, backends[i])) {
			known_algorithm = true;
			sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
					"[%s] ", backends[i]);
		} else {
			sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
					"%s ", backends[i]);
		}
	}

	/*
	 * Out-of-tree module known to crypto api or a missing
	 * entry in `backends'.
	 */
	if (!known_algorithm && crypto_has_comp(comp, 0, 0) == 1)
		sz += scnprintf(buf + sz, PAGE_SIZE - sz - 2,
				"[%s] ", comp);

	sz += scnprintf(buf + sz, PAGE_SIZE - sz, "\n");
	return sz;
}

struct zcomp_strm *zcomp_stream_get(struct zcomp *comp)
{
	local_lock(&comp->stream->lock);
	return this_cpu_ptr(comp->stream);
}

void zcomp_stream_put(struct zcomp *comp)
{
	local_unlock(&comp->stream->lock);
}

int zcomp_compress(struct zcomp_strm *zstrm,
		const void *src, unsigned int *dst_len)
{
	/*
	 * Our dst memory (zstrm->buffer) is always `2 * PAGE_SIZE' sized
	 * because sometimes we can endup having a bigger compressed data
	 * due to various reasons: for example compression algorithms tend
	 * to add some padding to the compressed buffer. Speaking of padding,
	 * comp algorithm `842' pads the compressed length to multiple of 8
	 * and returns -ENOSP when the dst memory is not big enough, which
	 * is not something that ZRAM wants to see. We can handle the
	 * `compressed_size > PAGE_SIZE' case easily in ZRAM, but when we
	 * receive -ERRNO from the compressing backend we can't help it
	 * anymore. To make `842' happy we need to tell the exact size of
	 * the dst buffer, zram_drv will take care of the fact that
	 * compressed buffer is too big.
	 */
	*dst_len = PAGE_SIZE * 2;

	return crypto_comp_compress(zstrm->tfm,
			src, PAGE_SIZE,
			zstrm->buffer, dst_len);
}

int zcomp_decompress(struct zcomp_strm *zstrm,
		const void *src, unsigned int src_len, void *dst)
{
	unsigned int dst_len = PAGE_SIZE;

	return crypto_comp_decompress(zstrm->tfm,
			src, src_len,
			dst, &dst_len);
}

#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
int zcomp_compress_thp(struct zcomp_strm *zstrm,
		const void *src, unsigned int *dst_len)
{
	/*
	 * Our dst memory (zstrm->buffer) is always `2 * CONT_PTE_SIZE' sized
	 * because sometimes we can endup having a bigger compressed data
	 * due to various reasons: for example compression algorithms tend
	 * to add some padding to the compressed buffer. Speaking of padding,
	 * comp algorithm `842' pads the compressed length to multiple of 8
	 * and returns -ENOSP when the dst memory is not big enough, which
	 * is not something that ZRAM wants to see. We can handle the
	 * `compressed_size > CONT_PTE_SIZE' case easily in ZRAM, but when we
	 * receive -ERRNO from the compressing backend we can't help it
	 * anymore. To make `842' happy we need to tell the exact size of
	 * the dst buffer, zram_drv will take care of the fact that
	 * compressed buffer is too big.
	 */
	*dst_len = CONT_PTE_SIZE * 2;

	return crypto_comp_compress(zstrm->tfm,
			src, CONT_PTE_SIZE,
			zstrm->buffer, dst_len);
}

int zcomp_decompress_thp(struct zcomp_strm *zstrm,
		const void *src, unsigned int src_len, void *dst)
{
	unsigned int dst_len = CONT_PTE_SIZE;

	return crypto_comp_decompress(zstrm->tfm,
			src, src_len,
			dst, &dst_len);
}
#endif

int zcomp_cpu_up_prepare(unsigned int cpu, struct hlist_node *node)
{
	struct zcomp *comp = hlist_entry(node, struct zcomp, node);
	struct zcomp_strm *zstrm;
	int ret;

	zstrm = per_cpu_ptr(comp->stream, cpu);
	local_lock_init(&zstrm->lock);

#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
	ret = zcomp_strm_init(zstrm, comp, cpu);
#else
	ret = zcomp_strm_init(zstrm, comp);
#endif
	if (ret)
		pr_err("Can't allocate a compression stream\n");
	return ret;
}

int zcomp_cpu_dead(unsigned int cpu, struct hlist_node *node)
{
	struct zcomp *comp = hlist_entry(node, struct zcomp, node);
	struct zcomp_strm *zstrm;

	zstrm = per_cpu_ptr(comp->stream, cpu);
#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
	zcomp_strm_free(zstrm, comp);
#else
	zcomp_strm_free(zstrm);
#endif
	return 0;
}

static int zcomp_init(struct zcomp *comp)
{
	int ret;

	comp->stream = alloc_percpu(struct zcomp_strm);
	if (!comp->stream)
		return -ENOMEM;

	ret = cpuhp_state_add_instance(CPUHP_ZCOMP_PREPARE, &comp->node);
	if (ret < 0)
		goto cleanup;
	return 0;

cleanup:
	free_percpu(comp->stream);
	return ret;
}

void zcomp_destroy(struct zcomp *comp)
{
	cpuhp_state_remove_instance(CPUHP_ZCOMP_PREPARE, &comp->node);
	free_percpu(comp->stream);
	kfree(comp);
}

/*
 * search available compressors for requested algorithm.
 * allocate new zcomp and initialize it. return compressing
 * backend pointer or ERR_PTR if things went bad. ERR_PTR(-EINVAL)
 * if requested algorithm is not supported, ERR_PTR(-ENOMEM) in
 * case of allocation error, or any other error potentially
 * returned by zcomp_init().
 */
struct zcomp *zcomp_create(const char *compress
#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
,bool is_thp_comp
#endif
)
{
	struct zcomp *comp;
	int error;

	if (!zcomp_available_algorithm(compress))
		return ERR_PTR(-EINVAL);

	comp = kzalloc(sizeof(struct zcomp), GFP_KERNEL);
	if (!comp)
		return ERR_PTR(-ENOMEM);

#ifdef CONFIG_CONT_PTE_HUGEPAGE_64K_ZRAM
	if(is_thp_comp == true) {
		comp->is_thp_comp = true;
	}
	else {
		comp->is_thp_comp = false;
	}
#endif

	comp->name = compress;
	error = zcomp_init(comp);
	if (error) {
		kfree(comp);
		return ERR_PTR(error);
	}
	return comp;
}
