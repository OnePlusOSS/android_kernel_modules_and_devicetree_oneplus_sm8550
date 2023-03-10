/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 */

#ifndef _DSI_PARSER_H_
#define _DSI_PARSER_H_

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/version.h>

#if IS_ENABLED(CONFIG_DSI_PARSER)
void *dsi_parser_get(struct device *dev);
void dsi_parser_put(void *data);
int dsi_parser_dbg_init(void *parser, struct dentry *dir);
void *dsi_parser_get_head_node(void *parser,
		const u8 *data, u32 size);

const void *dsi_parser_get_property(const struct device_node *np,
			const char *name, int *lenp);
bool dsi_parser_read_bool(const struct device_node *np,
			const char *propname);
int dsi_parser_read_u64(const struct device_node *np, const char *propname,
			 u64 *out_value);
int dsi_parser_read_u32(const struct device_node *np,
			const char *propname, u32 *out_value);
int dsi_parser_read_u32_index(const struct device_node *np,
			const char *propname, u32 index, u32 *out_value);
int dsi_parser_read_u32_array(const struct device_node *np,
			const char *propname,
			u32 *out_values, size_t sz);
int dsi_parser_read_string(const struct device_node *np,
			const char *propname, const char **out_string);
struct device_node *dsi_parser_get_child_by_name(const struct device_node *node,
				const char *name);
int dsi_parser_get_child_count(const struct device_node *np);
struct property *dsi_parser_find_property(const struct device_node *np,
			const char *name, int *lenp);
struct device_node *dsi_parser_get_next_child(const struct device_node *np,
	struct device_node *prev);
int dsi_parser_count_u32_elems(const struct device_node *np,
				const char *propname);
int dsi_parser_count_strings(const struct device_node *np,
			    const char *propname);
int dsi_parser_read_string_index(const struct device_node *np,
				const char *propname,
				int index, const char **output);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
int dsi_parser_get_named_gpio(const struct device_node *np,
				const char *propname, int index);
#else
int dsi_parser_get_named_gpio(struct device_node *np,
				const char *propname, int index);
#endif
#else /* CONFIG_DSI_PARSER */
static inline void *dsi_parser_get(struct device *dev)
{
	return NULL;
}

static inline void dsi_parser_put(void *data)
{
}

static inline int dsi_parser_dbg_init(void *parser, struct dentry *dir)
{
	return -ENODEV;
}

static inline void *dsi_parser_get_head_node(void *parser,
		const u8 *data, u32 size)
{
	return NULL;
}

static inline const void *dsi_parser_get_property(const struct device_node *np,
			const char *name, int *lenp)
{
	return NULL;
}

static inline bool dsi_parser_read_bool(const struct device_node *np,
			const char *propname)
{
	return false;
}

static inline int dsi_parser_read_u64(const struct device_node *np,
			const char *propname, u64 *out_value)
{
	return -ENODEV;
}

static inline int dsi_parser_read_u32(const struct device_node *np,
			const char *propname, u32 *out_value)
{
	return -ENODEV;
}

static inline int dsi_parser_read_u32_index(const struct device_node *np,
			const char *propname, u32 index, u32 *out_value)
{
	return -ENODEV;
}

static inline int dsi_parser_read_u32_array(const struct device_node *np,
			const char *propname, u32 *out_values, size_t sz)
{
	return -ENODEV;
}

static inline int dsi_parser_read_string(const struct device_node *np,
			const char *propname, const char **out_string)
{
	return -ENODEV;
}

static inline struct device_node *dsi_parser_get_child_by_name(
				const struct device_node *node,
				const char *name)
{
	return NULL;
}

static inline int dsi_parser_get_child_count(const struct device_node *np)
{
	return -ENODEV;
}

static inline struct property *dsi_parser_find_property(
			const struct device_node *np,
			const char *name, int *lenp)
{
	return NULL;
}

static inline struct device_node *dsi_parser_get_next_child(
				const struct device_node *np,
				struct device_node *prev)
{
	return NULL;
}

static inline int dsi_parser_count_u32_elems(const struct device_node *np,
				const char *propname)
{
	return -ENODEV;
}

static inline int dsi_parser_count_strings(const struct device_node *np,
			    const char *propname)
{
	return -ENODEV;
}

static inline int dsi_parser_read_string_index(const struct device_node *np,
				const char *propname,
				int index, const char **output)
{
	return -ENODEV;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
static inline int dsi_parser_get_named_gpio(const struct device_node *np,
				const char *propname, int index)
{
	return -ENODEV;
}
#else
static inline int dsi_parser_get_named_gpio(struct device_node *np,
				char *propname, int index)
{
	return -ENODEV;
}
#endif

#endif /* CONFIG_DSI_PARSER */

#define dsi_for_each_child_node(parent, child) \
	for (child = utils->get_next_child(parent, NULL); \
	     child != NULL; \
	     child = utils->get_next_child(parent, child))

struct dsi_parser_utils {
	void *data;
	struct device_node *node;

	const void *(*get_property)(const struct device_node *np,
			const char *name, int *lenp);
	int (*read_u64)(const struct device_node *np,
			const char *propname, u64 *out_value);
	int (*read_u32)(const struct device_node *np,
			const char *propname, u32 *out_value);
	int (*read_u32_index)(const struct device_node *np,
			const char *propname, u32 index, u32 *out_value);
	bool (*read_bool)(const struct device_node *np,
			 const char *propname);
	int (*read_u32_array)(const struct device_node *np,
			const char *propname, u32 *out_values, size_t sz);
	int (*read_string)(const struct device_node *np, const char *propname,
				const char **out_string);
	int (*read_string_index)(const struct device_node *np,
				const char *propname,
				int index, const char **output);
	struct device_node *(*get_child_by_name)(
				const struct device_node *node,
				const char *name);
	int (*get_child_count)(const struct device_node *np);
	struct property *(*find_property)(const struct device_node *np,
			const char *name, int *lenp);
	struct device_node *(*get_next_child)(const struct device_node *np,
		struct device_node *prev);
	int (*count_u32_elems)(const struct device_node *np,
		const char *propname);
	int (*count_strings)(const struct device_node *np,
					const char *propname);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	int (*get_named_gpio)(const struct device_node *np,
				const char *propname, int index);
#else
	int (*get_named_gpio)(struct device_node *np,
				const char *propname, int index);
#endif
	int (*get_available_child_count)(const struct device_node *np);
};

static inline struct dsi_parser_utils *dsi_parser_get_of_utils(void)
{
	static struct dsi_parser_utils of_utils = {
		.get_property = of_get_property,
		.read_bool = of_property_read_bool,
		.read_u64 = of_property_read_u64,
		.read_u32 = of_property_read_u32,
		.read_u32_index = of_property_read_u32_index,
		.read_u32_array = of_property_read_u32_array,
		.read_string = of_property_read_string,
		.read_string_index = of_property_read_string_index,
		.get_child_by_name = of_get_child_by_name,
		.get_child_count = of_get_child_count,
		.get_available_child_count = of_get_available_child_count,
		.find_property = of_find_property,
		.get_next_child = of_get_next_child,
		.count_u32_elems = of_property_count_u32_elems,
		.count_strings = of_property_count_strings,
		.get_named_gpio = of_get_named_gpio,
	};

	return &of_utils;
}

static inline struct dsi_parser_utils *dsi_parser_get_parser_utils(void)
{
	static struct dsi_parser_utils parser_utils = {
		.get_property = dsi_parser_get_property,
		.read_bool = dsi_parser_read_bool,
		.read_u64 = dsi_parser_read_u64,
		.read_u32 = dsi_parser_read_u32,
		.read_u32_index = dsi_parser_read_u32_index,
		.read_u32_array = dsi_parser_read_u32_array,
		.read_string = dsi_parser_read_string,
		.read_string_index = dsi_parser_read_string_index,
		.get_child_by_name = dsi_parser_get_child_by_name,
		.get_child_count = dsi_parser_get_child_count,
		.get_available_child_count = dsi_parser_get_child_count,
		.find_property = dsi_parser_find_property,
		.get_next_child = dsi_parser_get_next_child,
		.count_u32_elems = dsi_parser_count_u32_elems,
		.count_strings = dsi_parser_count_strings,
		.get_named_gpio = dsi_parser_get_named_gpio,
	};

	return &parser_utils;
}
#endif
