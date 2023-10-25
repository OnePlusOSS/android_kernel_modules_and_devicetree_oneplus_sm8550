// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2022, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2022.
 */
#include <linux/string.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>
#include <linux/printk.h>

#include "dsi_iris_api.h"
#include "dsi_iris_log.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lut.h"
#include "dsi_iris_dts_fw.h"


#define FDT_BEGIN_NODE	0x1		/* Start node: full name */
#define FDT_END_NODE	0x2		/* End node */
#define FDT_PROP	0x3		/* Property: name off, size, content */
#define FDT_NOP		0x4		/* nop */
#define FDT_END		0x9

#ifdef __CHECKER__
#define FDT_FORCE __attribute__((force))
#define FDT_BITWISE __attribute__((bitwise))
#else
#define FDT_FORCE
#define FDT_BITWISE
#endif

typedef uint16_t FDT_BITWISE fdt16_t;
typedef uint32_t FDT_BITWISE fdt32_t;
typedef uint64_t FDT_BITWISE fdt64_t;


struct fdt_header {
	fdt32_t magic;			 /* magic word FDT_MAGIC */
	fdt32_t totalsize;		 /* total size of DT block */
	fdt32_t off_dt_struct;		 /* offset to structure */
	fdt32_t off_dt_strings;		 /* offset to strings */
	fdt32_t off_mem_rsvmap;		 /* offset to memory reserve map */
	fdt32_t version;		 /* format version */
	fdt32_t last_comp_version;	 /* last compatible version */

	/* version 2 fields below */
	fdt32_t boot_cpuid_phys;	 /* Which physical CPU id we're booting on */
	/* version 3 fields below */
	fdt32_t size_dt_strings;	 /* size of the strings block */

	/* version 17 fields below */
	fdt32_t size_dt_struct;		 /* size of the structure block */
};

#define EXTRACT_BYTE(x, n)	((unsigned long long)((uint8_t *)&x)[n])
#define CPU_TO_FDT16(x) ((EXTRACT_BYTE(x, 0) << 8) | EXTRACT_BYTE(x, 1))
#define CPU_TO_FDT32(x) ((EXTRACT_BYTE(x, 0) << 24) | (EXTRACT_BYTE(x, 1) << 16) | \
			 (EXTRACT_BYTE(x, 2) << 8) | EXTRACT_BYTE(x, 3))

static inline uint32_t fdt32_to_cpu(fdt32_t x)
{
	return (FDT_FORCE uint32_t)CPU_TO_FDT32(x);
}

#define MAX_NAME_LEN  200
enum {
	NONE_DATA,
	PRINTABLE_DATA,
	CHAR_DATA,
	MAX_DATA,
};

LIST_HEAD(g_dts_list);

struct iris_dts_node {
	struct list_head head;
	char name[MAX_NAME_LEN];
	char *data;
	int  len;
	char type;
};


void iris_init_dts_node_head(void)
{
	INIT_LIST_HEAD(&g_dts_list);
}


struct list_head *iris_get_dts_node_head(void)
{
	return &g_dts_list;
}

static void iris_save_node(char *name, char *data, int len)
{
	struct iris_dts_node *p_node = NULL;

	p_node = vmalloc(sizeof(*p_node));
	if (!p_node) {
		IRIS_LOGE("could not malloc space for dts node");
		return;
	}
	memset(p_node, 0x00, sizeof(*p_node));
	strcpy(p_node->name, name);

	//IRIS_LOGI("%s", p_node->name);
	p_node->data = data;
	p_node->len = len;

	list_add_tail(&p_node->head, iris_get_dts_node_head());
}

static void iris_del_node_list(void)
{
	struct iris_dts_node *p_node = NULL;
	struct iris_dts_node *p_next_node = NULL;
	struct list_head *head_node = iris_get_dts_node_head();

	list_for_each_entry_safe(p_node, p_next_node, head_node, head) {
		list_del(&p_node->head);
		vfree(p_node);
		p_node = NULL;
	}
}

#define PALIGN(p, a)	((void *)(ALIGN((unsigned long)(p), (a))))
#define GET_CELL(p)	(p += 4, *((const fdt32_t *)(p-4)))

bool iris_is_printable_string(const void *data, int len)
{
	const char *s = data;
	const char *ss, *se;

	/* zero length is not */
	if (len == 0)
		return 0;

	/* must terminate with zero */
	if (s[len - 1] != '\0')
		return 0;

	se = s + len;

	while (s < se) {
		ss = s;
		while (s < se && *s && isprint((unsigned char)*s))
			s++;

		/* not zero, or not done yet */
		if (*s != '\0' || s == ss)
			return 0;

		s++;
	}

	return 1;
}


struct iris_dts_node *iris_find_dts_node(const char *name)
{
	char *data = NULL;
	int len;
	struct iris_dts_node *p_node = NULL;
	struct list_head *head_node = iris_get_dts_node_head();

	if (!name) {
		IRIS_LOGE("name is null");
		return p_node;
	}

	list_for_each_entry(p_node, head_node, head) {
		if (!strcmp(p_node->name, name))
			break;
	}

	if (head_node == &(p_node->head)) {
		IRIS_LOGE("could not find name:%s dts node", name);
		return NULL;
	}

	data = p_node->data;
	len = p_node->len;

	/* no data, don't print */
	if (len == 0)
		p_node->type = NONE_DATA;
	else if (iris_is_printable_string(data, len))
		p_node->type = PRINTABLE_DATA;
	else
		p_node->type = CHAR_DATA;
	return p_node;
}


static const void *iris_fw_get_property(const struct device_node *np, const char *name,
			    int *lenp)
{
	char *data = NULL;
	struct iris_dts_node *p_node = NULL;

	p_node = iris_find_dts_node(name);
	if (!p_node)
		return data;

	data = p_node->data;
	if (lenp)
		*lenp = (int)p_node->len;

	return data;
}

static int iris_fw_value_of_size(struct iris_dts_node **pp_node,
		const char *propname, size_t sz)
{
	struct iris_dts_node *p_node = NULL;

	p_node = iris_find_dts_node(propname);
	if (!p_node)
		return -EINVAL;

	if (!p_node->data)
		return -ENODATA;

	if (sz > p_node->len)
		return -EOVERFLOW;

	*pp_node = p_node;
	return 0;
}


static int iris_fw_read_u8_array(const struct device_node *np,
						const char *propname,
						u8 *out_values, size_t sz)
{
	int ret = -EINVAL;
	struct iris_dts_node *p_node = NULL;

	if (!propname || !out_values)
		return ret;

	ret = iris_fw_value_of_size(&p_node, propname, sz);
	if (!ret)
		memcpy(out_values, p_node->data, sz);

	return ret;
}

static int iris_fw_read_u8(const struct device_node *np,
	   const char *propname, u8 *out_value)
{
	return iris_fw_read_u8_array(np, propname, out_value, 1);
}

static int iris_fw_read_u32_array(const struct device_node *np,
						const char *propname,
						u32 *out_values, size_t sz)
{
	int ret = -EINVAL;
	char *ptr = NULL;
	int count = sz;
	struct iris_dts_node *p_node = NULL;

	if (!propname || !out_values)
		return ret;

	ret = iris_fw_value_of_size(&p_node, propname, sz*sizeof(*out_values));
	if (!ret) {
		ptr = p_node->data;
		while (count--)
			*out_values++ = fdt32_to_cpu(GET_CELL(ptr));
	}

	return ret;
}

static int32_t iris_fw_read_u32(const struct device_node *np,
	   const char *propname, u32 *out_value)
{
	return iris_fw_read_u32_array(np, propname, out_value, 1);
}

static int iris_fw_count_u8_elems(const struct device_node *np,
				const char *propname)
{
	struct iris_dts_node *p_node = NULL;

	p_node = iris_find_dts_node(propname);
	if (!p_node)
		return -EINVAL;

	if (!p_node->data)
		return -ENODATA;

	if (p_node->len % sizeof(u8) == 0)
		return p_node->len / sizeof(u8);

	return -EINVAL;
}

static bool iris_fw_read_bool(const struct device_node *np,
		 const char *propname)
{
	return iris_find_dts_node(propname) ? true : false;
}


static struct iris_dts_ops *gp_dts_ops;
static struct iris_dts_ops g_of_dts_ops = {
	.id = DTS_CTX_FROM_IMG,
	.get_property = of_get_property,
	.read_bool = of_property_read_bool,
	.read_u8 = of_property_read_u8,
	.count_u8_elems = of_property_count_u8_elems,
	.read_u8_array = of_property_read_u8_array,
	.read_u32 = of_property_read_u32,
	.read_u32_array = of_property_read_u32_array,
};
static struct iris_dts_ops g_fw_dts_ops = {
	.id = DTS_CTX_FROM_FW,
	.get_property = iris_fw_get_property,
	.read_bool = iris_fw_read_bool,
	.read_u8 = iris_fw_read_u8,
	.count_u8_elems = iris_fw_count_u8_elems,
	.read_u8_array = iris_fw_read_u8_array,
	.read_u32 = iris_fw_read_u32,
	.read_u32_array = iris_fw_read_u32_array,
};

struct iris_dts_ops *iris_get_dts_ops(void)
{
	if (!gp_dts_ops)
		IRIS_LOGE("%s dts_ops do not init", __func__);
	return gp_dts_ops;
}

void iris_set_dts_ops(int id)
{
	switch (id) {
	case DTS_CTX_FROM_IMG:
		gp_dts_ops = &g_of_dts_ops;
		break;
	case DTS_CTX_FROM_FW:
		gp_dts_ops = &g_fw_dts_ops;
		break;
	default:
		IRIS_LOGE("could not get dts ops");
		break;
	}
}

void iris_blob(const uint8_t *blob, uint32_t len)
{
	struct fdt_header *bph = (struct fdt_header *)blob;
	uint32_t off_dt = fdt32_to_cpu(bph->off_dt_struct);
	uint32_t off_str = fdt32_to_cpu(bph->off_dt_strings);
	uint8_t *p_struct = (uint8_t *)blob + off_dt;
	uint8_t *p_strings = (uint8_t *)blob + off_str;
	uint32_t version = fdt32_to_cpu(bph->version);
	uint32_t tag;
	uint8_t *p, *s, *t;
	int depth, sz, shift;

	depth = 0;
	shift = 4;

	p = p_struct;
	while ((tag = fdt32_to_cpu(GET_CELL(p))) != FDT_END) {

		if (tag == FDT_BEGIN_NODE) {
			s = p;
			p = PALIGN(p + strlen(s) + 1, 4);
			continue;
		}

		if (tag == FDT_END_NODE)
			continue;

		if (tag == FDT_NOP) {
			IRIS_LOGI("%*s// [NOP]\n", depth * shift, "");
			continue;
		}

		if (tag != FDT_PROP) {
			IRIS_LOGI("%*s ** Unknown tag\n", depth * shift, "");
			break;
		}
		sz = fdt32_to_cpu(GET_CELL(p));
		s = p_strings + fdt32_to_cpu(GET_CELL(p));
		if (version < 16 && sz >= 8)
			p = PALIGN(p, 8);

		t = p;
		p = PALIGN(p + sz, 4);
		iris_save_node(s, t, sz);
	}
}


int iris_parse_dts_ctx(const uint8_t *fw_name)
{
	int ret = 0;
	const struct firmware *fw = NULL;

	if (!fw_name) {
		IRIS_LOGE("fw name is null");
		return -EINVAL;
	}
	// Load "pxlw.fw".
	ret = iris_request_firmware(&fw, fw_name);
	if (!ret)
		IRIS_LOGI("%s(%d), request name: %s, size: %u.",
				__func__, __LINE__, fw_name, fw->size);
	else {
		IRIS_LOGE("%s(), failed to request: %s", __func__,
				fw_name);
		return -EINVAL;
	}

	iris_init_dts_node_head();

	iris_blob(fw->data, fw->size);

	iris_parse_cmd_param(NULL);

	iris_del_node_list();
	iris_release_firmware(&fw);

	return 0;
}
