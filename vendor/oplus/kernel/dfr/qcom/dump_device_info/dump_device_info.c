// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/version.h>
#include <soc/oplus/system/oplus_project.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
#include <../drivers/soc/qcom/debug_symbol.h>
#else
#include <linux/android_debug_symbols.h>
#endif
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/memblock.h>
#include <linux/rslib.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <asm/page.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include "device_info.h"

#define MEM_TYPE_WCOMBINE 0
#define MEM_TYPE_NONCACHED 1
#define MEM_TYPE_NORMAL 2

#define DEVICE_INFO_BUFFER_SIZE 32768

static char project_version[8];
static char pcb_version[8];
static char rf_version[8];

const char *platform_name;
const char *kernel_version;

extern char serial_no[];

static char zero_buf[DEVICE_INFO_BUFFER_SIZE] = {'\0'};

struct persistent_ram_buffer {
	uint32_t start;
	uint8_t data[0];
};

struct device_info_platform_data {
	unsigned long mem_size;
	unsigned long mem_address;
	unsigned int mem_type;
};

struct device_info_ramzone_type {
	phys_addr_t paddr;
	void *vaddr;
	size_t device_info_size;
	struct persistent_ram_buffer *buffer;
};

static struct device_info_ramzone_type device_info_ramzone;

static void *persistent_ram_vmap(phys_addr_t start, size_t size,
		unsigned int memtype)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	switch (memtype) {
	case MEM_TYPE_NORMAL:
		prot = PAGE_KERNEL;
		break;
	case MEM_TYPE_NONCACHED:
		prot = pgprot_noncached(PAGE_KERNEL);
		break;
	case MEM_TYPE_WCOMBINE:
		prot = pgprot_writecombine(PAGE_KERNEL);
		break;
	default:
		pr_err("invalid mem_type=%d\n", memtype);
		return NULL;
	}

	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n",
			__func__, page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);

	return vaddr + offset_in_page(start);
}

static int persistent_ram_buffer_map(phys_addr_t start, phys_addr_t size,
		struct device_info_ramzone_type *drz, int memtype)
{
	drz->paddr = start;
	drz->device_info_size = size;

	if (pfn_valid(start >> PAGE_SHIFT))
		drz->vaddr = persistent_ram_vmap(start, size, memtype);
	else {
		pr_err("%s: Physical address 0x%llx is not in RAM\n", __func__, (unsigned long long)start);
		return -ENOMEM;
	}

	if (!drz->vaddr) {
		pr_err("%s: Failed to map 0x%llx pages at 0x%llx\n", __func__,
			(unsigned long long)size, (unsigned long long)start);
		return -ENOMEM;
	}

	drz->buffer = drz->vaddr;
	drz->device_info_size = size - sizeof(struct persistent_ram_buffer);

	return 0;
}

static void persistent_ram_init(void)
{
	struct persistent_ram_buffer *buffer = device_info_ramzone.buffer;
	size_t size = device_info_ramzone.device_info_size;
	buffer->start = 0;
	memset(zero_buf, ' ', sizeof(zero_buf));
	memcpy_toio(buffer->data, zero_buf, size);
}

static void persistent_ram_update(const void *s, unsigned int count)
{
	struct persistent_ram_buffer *buffer = device_info_ramzone.buffer;
	memcpy_toio(buffer->data + buffer->start, s, count);
	buffer->start += count;
}

static void pstore_write_device_info(const char *s, unsigned int c)
{
	struct persistent_ram_buffer *buffer = device_info_ramzone.buffer;
	if (c > device_info_ramzone.device_info_size)
		c = device_info_ramzone.device_info_size;
	if (buffer->start + c > device_info_ramzone.device_info_size)
		c = device_info_ramzone.device_info_size - buffer->start;
	persistent_ram_update(s, c);
}

static const char *get_machine_name(void)
{
	int ret;
	const char *str;
	struct device_node *np;

	np = of_find_node_by_path("/");
	ret = of_property_read_string(np, "model", &str);
	if (!ret) {
		return str;
	}

	ret = of_property_read_string(np, "compatible", &str);
	if (!ret) {
		return str;
	}

	return "unknown";
}

static void board_hw_info_init(void)
{
	scnprintf(pcb_version, sizeof(pcb_version), "%d", get_PCB_Version());
	scnprintf(project_version, sizeof(project_version), "%d", get_project());
	scnprintf(rf_version, sizeof(rf_version), "%d", get_Modem_Version());

	platform_name = get_machine_name();
        #if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
        kernel_version = DEBUG_SYMBOL_LOOKUP(linux_banner);
        #else
        kernel_version = android_debug_symbol(ADS_LINUX_BANNER);
        #endif
}

static void write_device_info(const char *key, const char *value)
{
	pstore_write_device_info(key, strlen(key));
	pstore_write_device_info(": ", 2);
	pstore_write_device_info(value, strlen(value));
	pstore_write_device_info("\r\n", 2);
}

static int of_device_info_platform_data(struct device_node *node,
	struct device_info_platform_data *pdata)
{
	const u32 *addr;
	u64 size;
	struct device_node *pnode;

	memset(pdata, 0, sizeof(*pdata));

	pnode = of_parse_phandle(node, "linux,contiguous-region", 0);
	if (pnode) {
		addr = of_get_address(pnode, 0, &size, NULL);
		if (!addr) {
			pr_err("failed to parse the device info memory address\n");
			of_node_put(pnode);
			return -EINVAL;
		}
		pdata->mem_address = of_read_ulong(addr, 2);
		pdata->mem_size = (unsigned long) size;
		of_node_put(pnode);
	} else {
		pr_err("mem reservation for device info not present\n");
		return -EINVAL;
	}

	return 0;
}

static int dump_device_info_probe(struct platform_device *pdev)
{
	struct device_info_platform_data *pdata = pdev->dev.platform_data;
	struct device_info_platform_data of_pdata;

	phys_addr_t paddr;
	int err = -EINVAL;

	pr_info("dump_device_info_probe!\n");

	if (pdev->dev.of_node) {
		if (of_device_info_platform_data(pdev->dev.of_node,
			&of_pdata)) {
			pr_err("Invalid device info device tree data\n");
			goto fail_out;
		}
		pdata = &of_pdata;
	}

	if (!pdata->mem_size) {
		pr_err("memory size and record size must be non-zero\n");
		goto fail_out;
	}

	paddr = pdata->mem_address;


	err = persistent_ram_buffer_map(paddr, pdata->mem_size,
		&device_info_ramzone, MEM_TYPE_WCOMBINE);
	if (err)
		goto fail_out;

	persistent_ram_init();
	board_hw_info_init();

	write_device_info("project version", project_version);
	write_device_info("pcb version", pcb_version);
	write_device_info("rf version", rf_version);
	write_device_info("serial no", serial_no);

        if (platform_name != NULL) {
	   write_device_info("soc version", platform_name);
        }

        if (kernel_version != NULL) {
	   write_device_info("kernel version", kernel_version);
        }

	return 0;

fail_out:
	pr_err("dump_device_info, fail_out!\n");
	return err;
}


static int __exit dump_device_info_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

static const struct of_device_id dump_device_info_of_match[] = {
	{ .compatible = "dump_device_info", },
	{ },
};
MODULE_DEVICE_TABLE(of, dump_device_info_of_match);

static struct platform_driver dump_device_info_driver = {
	.probe = dump_device_info_probe,
	.remove = __exit_p(dump_device_info_remove),
	.driver = {
		.name = "dump_device_info",
		.owner = THIS_MODULE,
		.of_match_table = dump_device_info_of_match,
	},
};

static int __init dump_device_info_init(void)
{
	return platform_driver_register(&dump_device_info_driver);
}
late_initcall(dump_device_info_init);

static void __exit dump_device_info_exit(void)
{
	platform_driver_unregister(&dump_device_info_driver);
}
module_exit(dump_device_info_exit);

void save_dump_reason_to_device_info(char *reason) {
		write_device_info("dump reason is ", reason);
}
EXPORT_SYMBOL(save_dump_reason_to_device_info);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leo.liuch");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
MODULE_IMPORT_NS(MINIDUMP);
#endif
