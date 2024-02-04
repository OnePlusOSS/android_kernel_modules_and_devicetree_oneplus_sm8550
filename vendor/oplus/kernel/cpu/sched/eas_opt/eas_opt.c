// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/proc_fs.h>

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAKE_CAP)
#include "fake_cap.h"
#endif

#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
#include "oplus_iowait.h"
#endif

struct proc_dir_entry *eas_opt_dir = NULL;
static int __init oplus_eas_opt_init(void)
{
	int ret = 0;

	eas_opt_dir = proc_mkdir("eas_opt", NULL);
	if (!eas_opt_dir) {
		pr_err("fail to mkdir /proc/eas_opt\n");
		return -ENOMEM;
	}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAKE_CAP)
	ret = fake_cap_init(eas_opt_dir);
	if (ret != 0)
		return ret;

	pr_info("[eas_opt]: fake_cap init success\n");
#endif

#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
	ret = oplus_sched_assist_iowait_proc_init(eas_opt_dir);
	if (ret != 0)
		return ret;

	pr_info("[eas_opt]: iowait_config init success\n");
#endif

	return ret;
}

module_init(oplus_eas_opt_init);
MODULE_DESCRIPTION("Oplus Eas Opt Moduler");
MODULE_LICENSE("GPL v2");
