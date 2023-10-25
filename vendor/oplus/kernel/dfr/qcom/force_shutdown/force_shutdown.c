// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/qcom_scm.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input/qpnp-power-on.h>
#include <linux/printk.h>
#include <linux/pm.h>


static struct proc_dir_entry *oplus_ke_proc_dir;

static ssize_t proc_force_shutdown_read(struct file *file,
				char __user *buf, size_t size, loff_t *ppos)
{
	return 0;
}

static ssize_t proc_force_shutdown_write(struct file *file,
			const char __user *buf, size_t size, loff_t *ppos)
{
	#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0))
	qpnp_pon_system_pwr_off(PON_POWER_OFF_SHUTDOWN);

	qcom_scm_deassert_ps_hold();
	#endif
	if(pm_power_off)
		pm_power_off();

	return 0;
}

static struct proc_ops proc_force_shutdown_fops = {
	.proc_read = proc_force_shutdown_read,
	.proc_write = proc_force_shutdown_write,
};

static int __init force_shutdown_init(void)
{
	oplus_ke_proc_dir = proc_mkdir("oplus_ke", NULL);
	if (oplus_ke_proc_dir == NULL) {
		pr_err("oplus_ke proc_mkdir failed\n");
		return -EFAULT;
	}

	if (!proc_create("force_shutdown", S_IFREG | 0600, oplus_ke_proc_dir, &proc_force_shutdown_fops)) {
		pr_info("proc_create force_shutdown failed\n");
		return -EFAULT;
	}
	return 0;
}

module_init(force_shutdown_init);

MODULE_LICENSE("GPL v2");
