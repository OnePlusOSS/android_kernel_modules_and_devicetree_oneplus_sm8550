// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/soc/qcom/smem.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/syscalls.h>
#include <linux/arm-smccc.h>
#include <uapi/linux/psci.h>
#include <linux/types.h>
#include <linux/version.h>

#define REG_ADDR 0xA08
#define DISABLE_PMIC_CTRL_FUNCTION 0
#define ENABLE_PMIC_CTRL_FUNCTION 1
#define TRIGGER_PM_POWEROFF 2
#define TRIGGER_HARD_RESET 3

/*#if IS_ENABLED(CONFIG_64BIT)
#define PSCI_FN_NATIVE(version, name)   PSCI_##version##_FN64_##name
#else
#define PSCI_FN_NATIVE(version, name)   PSCI_##version##_FN_##name
#endif*/

extern void (*pm_power_off)(void);

static int mtk_pmic_custommade_enable = 1;

static void psci_sys_reset(void)
{
	struct arm_smccc_res res;
	arm_smccc_smc(PSCI_0_2_FN_SYSTEM_RESET, 0, 0, 0, 0, 0, 0, 0, &res);
}

static ssize_t mtk_pmic_custommade_write(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
	int ret = 0;
	char buffer[4] = {0};
	unsigned int value = 0;

	if (size > 2) {
		pr_err("size error\n");
		return size;
	}

	if (copy_from_user(buffer, buf, size)) {
		pr_err("%s: read proc input error.\n", __func__);
		return size;
	}

	if (size != 0) {
		pr_info("[%s] buffer is %s, size is %d\n", __func__, buffer, (int)size);
		ret = kstrtou32(buffer, 16, (unsigned int *)&value);

		if (mtk_pmic_custommade_enable == 0 && value != 1) {
			pr_err("mtk_pmic_custommade closed\n");
			return size;
		}

		switch(value) {
		case DISABLE_PMIC_CTRL_FUNCTION:
			pr_info("mtk_pmic_custommade_enable set 0\n");
			mtk_pmic_custommade_enable = 0;
			break;
		case ENABLE_PMIC_CTRL_FUNCTION:
			pr_info("mtk_pmic_custommade_enable set 1\n");
			mtk_pmic_custommade_enable = 1;
			break;
		case TRIGGER_PM_POWEROFF:
			pr_info("trigger machine_power_off");
			if (pm_power_off) {
				pm_power_off();
			}
			break;
		case TRIGGER_HARD_RESET:
			psci_sys_reset();
			break;
		default:
			pr_info("not support\n");
			break;
		}
	}

	return size;
}

static int mtk_pmic_custommade_read_func(struct seq_file *s, void *v)
{
	seq_printf(s, "-------------notice------------\n");
	seq_printf(s, "%d: DISABLE_PMIC_CTRL_FUNCTION\n", DISABLE_PMIC_CTRL_FUNCTION);
	seq_printf(s, "%d: ENABLE_PMIC_CTRL_FUNCTION\n", ENABLE_PMIC_CTRL_FUNCTION);
	seq_printf(s, "%d: TRIGGER_PM_POWEROFF\n", TRIGGER_PM_POWEROFF);
	seq_printf(s, "%d: TRIGGER_HARD_RESET\n", TRIGGER_HARD_RESET);
	seq_printf(s, "\nmtk_pmic_custommade_enable : %d\n", mtk_pmic_custommade_enable);

	return 0;
}

static int mtk_pmic_custommade_open(struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	return single_open(file, mtk_pmic_custommade_read_func, PDE_DATA(inode));
#else
	return single_open(file, mtk_pmic_custommade_read_func, pde_data(inode));
#endif
}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static const struct file_operations mtk_pmic_custommade_fops = {
	.open  = mtk_pmic_custommade_open,
	.read  = seq_read,
	.write = mtk_pmic_custommade_write,
	.release = single_release,
};
#else
static const struct proc_ops mtk_pmic_custommade_fops = {
	.proc_open  = mtk_pmic_custommade_open,
	.proc_read  = seq_read,
	.proc_write = mtk_pmic_custommade_write,
	.proc_release = single_release,
};
#endif

static int __init mtk_pmic_custommade_init(void)
{
	struct proc_dir_entry *p_entry;

	p_entry = proc_create_data("mtk_pmic_shutdown", S_IRUGO, NULL, &mtk_pmic_custommade_fops, NULL);
	if (!p_entry)
		goto error_init;

	return 0;

error_init:
	return -ENOENT;
}

late_initcall(mtk_pmic_custommade_init);

MODULE_DESCRIPTION("mtk pmic custommade version");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("DJ");
