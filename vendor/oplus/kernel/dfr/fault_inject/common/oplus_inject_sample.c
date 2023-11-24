// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2024 Oplus. All rights reserved.
 */
/***************************************************************
** OPLUS_FEATURE_DEVICE_FAULT_INJECT
** File : oplus_device_fault_inject.c
** Description : framework for device fault inject test
** Version : 1.0
******************************************************************/

#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/version.h>

#include "oplus_inject_hook.h"
#include "oplus_inject_proc.h"

/* record the hookhandler whether register */
static int bhook = 0;
static int capcitystate = 0;
static int voltagestate = 0;

#define SEQ_printf(m, x...)     \
        do {                        \
                if (m)                  \
                seq_printf(m, x);   \
                else                    \
                pr_debug(x);        \
        } while (0)

static int register_charger_hook(void);
static void unregister_charger_hook(void);

static int oplus_sample_help_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "=== oplus_sample_fault inject test ===\n");
	SEQ_printf(m, "input param:\n");
	SEQ_printf(m, "hook battery capacity and voltage\n");
	SEQ_printf(m, "@param1 capacity or valtage \n");
	SEQ_printf(m, "@param0 should 1 or 2 \n");

	/* print current status value */

	SEQ_printf(m, "capcitystate %d \n", capcitystate);
	SEQ_printf(m, "voltagestate %d \n", voltagestate);
	return 0;
}

static int oplus_sample_fault_inject_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, oplus_sample_help_show, inode->i_private);
}

static ssize_t oplus_sample_fault_inject_write(struct file *file,
		const char __user *buf,
		size_t count, loff_t *off)
{
	char buffer[64] = {0};
	int ret = 0;
	char testcase[64] = {0};
	int nval = 0;

	if (count > 64) {
		count = 64;
	}

	if (copy_from_user(buffer, buf, count)) {
		pr_err("%s: read proc input error.\n", __func__);
		return count;
	}

	ret = sscanf(buffer, "%s %d", testcase,  &nval);

	if (ret <= 0) {
		pr_err("%s input param error\n", __func__);
		return count;
	}

	if (!bhook) {
		/*
                 * no need register hook when init, may caused performance issue.
                 * suggest when inject test trigger do the hook
                 */
		ret = register_charger_hook();

		if (!ret) {
			bhook = 1;
		}
	}

	if (strcmp(testcase, "capacity") == 0) {
		capcitystate = nval;

	} else if (strcmp(testcase, "voltage") == 0) {
		voltagestate = nval;

        } else if (strcmp(testcase, "clear") == 0) {
                unregister_charger_hook();
                pr_info("%s do unregister\n", __func__);
        } else {
		pr_err("%s testcase param invalid\n", __func__);
	}

	return count;
}

static struct proc_ops oplus_charger_inject_ops = {
	.proc_open = oplus_sample_fault_inject_open,
	.proc_read = seq_read,
	.proc_write = oplus_sample_fault_inject_write,
	.proc_release = single_release,
};

/*
 * when userspace write value, will callback the handler function
 */

int oplus_hook_handler(oplus_gauge_get_remaining_capacity) (struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	/* set capcity to low or max */
	if (capcitystate == 1) {
		oplus_hook_return(regs, 5);

	} else if (capcitystate == 2) {
		oplus_hook_return(regs, 99);
	}

	pr_err("%s capcitystate %d\n", __func__, capcitystate);
	return 0;
}

int oplus_hook_handler(haptics_wait_hboost_ready)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	/* set battery voltage to low or max */
	if (voltagestate == 1) {
		oplus_hook_return(regs, 1215);

	} else if (voltagestate == 2) {
		oplus_hook_return(regs, 809);
	}

	pr_err("%s voltagestate %d\n", __func__, voltagestate);
	return 0;
}

/* init the hook struct */
oplus_hook_define(oplus_gauge_get_remaining_capacity);
oplus_hook_define(haptics_wait_hboost_ready);

static struct oplus_hook *charger_interfaces[] = {
	&oplus_hook_name(oplus_gauge_get_remaining_capacity),
	&oplus_hook_name(haptics_wait_hboost_ready),
};

static int register_charger_hook(void)
{
	int ret = 0;

	ret = register_oplus_hooks(charger_interfaces, ARRAY_SIZE(charger_interfaces));

	if (ret) {
		pr_err("%s failed %d\n", __func__, ret);
	}

	return ret;
}

static void unregister_charger_hook(void)
{
        if (bhook) {
                unregister_oplus_hooks(charger_interfaces, ARRAY_SIZE(charger_interfaces));
        }

        bhook = 0;
}

static struct proc_tree_node *gNode = NULL;
static int oplus_inject_sample_init(void)
{
	gNode = oplus_proc_inject_init("charger/capacity", &oplus_charger_inject_ops, NULL);

	return 0;
}

static void oplus_inject_sample_exit(void)
{
	if (bhook) {
		unregister_oplus_hooks(charger_interfaces, ARRAY_SIZE(charger_interfaces));
	}

        if (gNode) {
                oplus_proc_inject_exit(gNode);
        }
}

module_init(oplus_inject_sample_init);
module_exit(oplus_inject_sample_exit);

MODULE_DESCRIPTION("OPLUS device fault inject driver");
MODULE_LICENSE("GPL v2");
