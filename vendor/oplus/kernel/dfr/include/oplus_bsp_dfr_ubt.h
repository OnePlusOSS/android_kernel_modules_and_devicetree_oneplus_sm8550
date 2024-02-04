// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2023-2030 Oplus. All rights reserved.
*/
#ifndef _OPLUS_BSP_DFR_USERSPACE_BACKTRACE_H_
#define _OPLUS_BSP_DFR_USERSPACE_BACKTRACE_H_
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/compiler.h>
#include <linux/platform_device.h>

/* How to use userspace backtrace module:
 * The first method    If module userspace_backtrace.ko loading before other dependent ko,
 * you can call function dump_userspace_backtrace directly.
 * The second method   If you are not sure, please using the second method that platform
 * drvier & device matching. Even if the ko is not loaded, your drvier never panic.
*/

/* =========================The first method =========================== */
void dump_userspace_backtrace(struct task_struct *task);

/* =========================The second method ========================== */
static struct platform_device userspace_backtrace_device = {
	.name = NULL,
	.id = -1,
};

/* dump backtrace for native process */
static inline void dump_userspace_bt(struct task_struct *task)
{
	void (*dump_bt_fn)(struct task_struct *task) = NULL;
	struct platform_device *pdev = &userspace_backtrace_device;

	dump_bt_fn = dev_get_drvdata(&pdev->dev);
	if (dump_bt_fn) {
		dump_bt_fn(task);
	}
}

/* using dump_userspace_init to init platform device at init function
 * name should be in userspace_backtrace_id_table list of userspace_backtrace.c
 * for match driver.
*/
static inline int dump_userspace_init(const char *name)
{
	struct platform_device *pdev = &userspace_backtrace_device;
	int ret = 0;

	if (!name) {
		pr_err("%s: must set device name!\n", __func__);
		return -EINVAL;
	}

	pdev->name = name;/* config device name */
	ret = platform_device_register(pdev);
	if (ret) {
		pdev->name = NULL;
		pr_err("%s: register platform device failed\n", __func__);
		return ret;
	}
	return 0;
}

static inline void dump_userspace_exit(void)
{
	struct platform_device *pdev = &userspace_backtrace_device;

	if (pdev->name)
		platform_device_unregister(pdev);
}

#endif /* _OPLUS_BSP_DFR_USERSPACE_BACKTRACE_H_ */
