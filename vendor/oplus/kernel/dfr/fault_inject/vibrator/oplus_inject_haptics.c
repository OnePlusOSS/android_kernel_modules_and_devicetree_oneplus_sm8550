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

#include "../common/oplus_inject_hook.h"
#include "../common/oplus_inject_proc.h"

/* record the hookhandler whether register */
static int bhook = 0;
#define ARG_0    0
#define ARG_1    1
#define ARG_2    2
#define ARG_3    3
#define ARG_4    4

#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
int oplus_haptic_track_dev_err(uint32_t track_type, uint32_t reg_addr, uint32_t err_code);
#endif
#define HAPTIC_SPMI_READ_TRACK_ERR_FB    5
#define HAPTIC_SPMI_WRITE_TRACK_ERR_FB   6

struct haptics_testcase {
	int haptics_inject_type;
	char *test_case_name;
	int valid_flag;
};

enum haptics_testcase_list {
	HAPTICS_SPMI_READ_INJECT = 0,
	HAPTICS_SPMI_WRITE_INJECT,
	HAPTICS_LRA_FRE_INJECT,
	HAPTICS_OPENLOOP_LRA_PERIOD_INJECT,
	HAPTICS_CONSTANT_EFFECT_INJECT,
	HAPTICS_CUSTOM_EFFECT_INJECT,
	HAPTICS_VMAX_SET_INJECT,
	HAPTICS_FIFO_MEM_FULL_INJECT,
	HAPTICS_INJECT_MAX,
};

static struct haptics_testcase g_haptic_inject_cases[HAPTICS_INJECT_MAX] = {
	{HAPTICS_SPMI_READ_INJECT, "haptic_read", 0},
	{HAPTICS_SPMI_WRITE_INJECT, "haptic_write", 0},
	{HAPTICS_LRA_FRE_INJECT, "lra_period", 0},
	{HAPTICS_OPENLOOP_LRA_PERIOD_INJECT, "closelooplra", 0},
	{HAPTICS_CONSTANT_EFFECT_INJECT, "constant_effect", 0},
	{HAPTICS_CUSTOM_EFFECT_INJECT, "custom_effect", 0},
	{HAPTICS_VMAX_SET_INJECT, "vmax_set", 0},
	{HAPTICS_FIFO_MEM_FULL_INJECT, "fifo_mem", 0},
};

#define SEQ_printf(m, x...)     \
        do {                        \
                if (m)                  \
                seq_printf(m, x);   \
                else                    \
                pr_debug(x);        \
        } while (0)

static int register_haptics_hook(void);

static int oplus_sample_help_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "=== oplus_sample_fault inject test ===\n");

	/* print current status value */
	SEQ_printf(m, "haptic_read %d \n", g_haptic_inject_cases[HAPTICS_SPMI_READ_INJECT].valid_flag);
	SEQ_printf(m, "haptic_write %d \n", g_haptic_inject_cases[HAPTICS_SPMI_WRITE_INJECT].valid_flag);
	SEQ_printf(m, "lra_period %d \n", g_haptic_inject_cases[HAPTICS_LRA_FRE_INJECT].valid_flag);
	SEQ_printf(m, "closelooplra %d \n", g_haptic_inject_cases[HAPTICS_OPENLOOP_LRA_PERIOD_INJECT].valid_flag);
	SEQ_printf(m, "constant_effect %d \n", g_haptic_inject_cases[HAPTICS_CONSTANT_EFFECT_INJECT].valid_flag);
	SEQ_printf(m, "custom_effect %d \n", g_haptic_inject_cases[HAPTICS_CUSTOM_EFFECT_INJECT].valid_flag);
	SEQ_printf(m, "vmax_set %d \n", g_haptic_inject_cases[HAPTICS_VMAX_SET_INJECT].valid_flag);
	SEQ_printf(m, "fifo_mem %d \n", g_haptic_inject_cases[HAPTICS_FIFO_MEM_FULL_INJECT].valid_flag);

	return 0;
}

static int oplus_sample_fault_inject_open(struct inode *inode, struct file *file)
{
	return single_open(file, oplus_sample_help_show, inode->i_private);
}

static ssize_t oplus_sample_fault_inject_write(struct file *file,
		const char __user *buf, size_t count, loff_t *off)
{
	char buffer[64] = {0};
	int ret = 0;
	char testcase[64] = {0};
	int nval = 0;
	int i = 0;

	if (count > 64) {
		count = 64;
	}

	if (copy_from_user(buffer, buf, count)) {
		pr_err("%s: read proc input error.\n", __func__);
		return count;
	}

	ret = sscanf(buffer, "%s %d", testcase, &nval);

	if (ret <= 0) {
		pr_err("%s input param error\n", __func__);
		return count;
	}

	if (!bhook) {
		/*
		 * no need register hook when init, may caused performance issue.
		 * suggest when inject test trigger do the hook
		 */
		ret = register_haptics_hook();
		if (!ret) {
			bhook = 1;
		}
	}

	for(i = 0; i < HAPTICS_INJECT_MAX; i++) {
		if (strcmp(testcase, g_haptic_inject_cases[i].test_case_name) == 0) {
			g_haptic_inject_cases[i].valid_flag = nval;
			break;
		}
	}

	return count;
}

static struct proc_ops oplus_haptics_inject_ops = {
	.proc_open = oplus_sample_fault_inject_open,
	.proc_read = seq_read,
	.proc_write = oplus_sample_fault_inject_write,
	.proc_release = single_release,
};

/*
 * when userspace write value, will callback the handler function
 */

int oplus_hook_handler(haptics_read)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	u16 addr = 0;
	struct OplusHookRegs *rd = (struct OplusHookRegs *)ri->data;

	/* set capcity to low or max */
	if (g_haptic_inject_cases[HAPTICS_SPMI_READ_INJECT].valid_flag == 1) {
		addr = *(u16 *)(&(rd->args[1])) + *(u8 *)(&(rd->args[2]));
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		oplus_haptic_track_dev_err(HAPTIC_SPMI_READ_TRACK_ERR_FB, addr, -1);
#endif
		oplus_hook_return(regs, -1);
	}
	return 0;
}


int oplus_hook_handler(haptics_write)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	u16 addr = 0;
	struct OplusHookRegs *rd = (struct OplusHookRegs *)ri->data;

	/* set capcity to low or max */
	if (g_haptic_inject_cases[HAPTICS_SPMI_WRITE_INJECT].valid_flag == 1) {
		addr = *(u16 *)(&(rd->args[1])) + *(u8 *)(&(rd->args[2]));
#ifdef CONFIG_HAPTIC_FEEDBACK_MODULE
		oplus_haptic_track_dev_err(HAPTIC_SPMI_WRITE_TRACK_ERR_FB, addr, -1);
#endif
		oplus_hook_return(regs, -1);
	}

	return 0;
}


/* inject detect_lra_frequency err */
int oplus_hook_handler(haptics_get_closeloop_lra_period)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	/* set oplus_haptics_get_closeloop_lra_period return false */
	if (g_haptic_inject_cases[HAPTICS_LRA_FRE_INJECT].valid_flag == 1) {
		oplus_hook_return(regs, -EINVAL);
	}

	return 0;
}

/* inject config_openloop_lra_period err */
int oplus_hook_handler_entry(haptics_config_openloop_lra_period)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	int val = 0;

	if (g_haptic_inject_cases[HAPTICS_OPENLOOP_LRA_PERIOD_INJECT].valid_flag != 0) {
		val = g_haptic_inject_cases[HAPTICS_OPENLOOP_LRA_PERIOD_INJECT].valid_flag;
		oplus_hook_setarg(regs, ARG_1, g_haptic_inject_cases[HAPTICS_OPENLOOP_LRA_PERIOD_INJECT].valid_flag);
	}

	return 0;
}

/* inject haptics_load_constant_effect err */
int oplus_hook_handler_entry(haptics_load_constant_effect)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	int amplitude = 0;

	/* set oplus_haptics_get_closeloop_lra_period return false */
	if (g_haptic_inject_cases[HAPTICS_CONSTANT_EFFECT_INJECT].valid_flag == 1) {
		oplus_hook_setarg(regs, ARG_1, amplitude);
	}

	return 0;
}



/* inject haptics_load_custom_effect err */
int oplus_hook_handler_entry(haptics_load_custom_effect)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	s16 magnitude = 0;

	/* set oplus_haptics_get_closeloop_lra_period return false */
	if (g_haptic_inject_cases[HAPTICS_CUSTOM_EFFECT_INJECT].valid_flag == 1) {
		oplus_hook_setarg(regs, ARG_3, magnitude);
	}

	if (g_haptic_inject_cases[HAPTICS_CUSTOM_EFFECT_INJECT].valid_flag == 1) {
		oplus_hook_setarg(regs, ARG_2, 0);
	}

	return 0;
}


/* inject oplus_haptics_set_vmax_mv */
int oplus_hook_handler_entry(haptics_set_vmax_mv)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	/* set haptics_fifo_empty_irq_config false */
	if (g_haptic_inject_cases[HAPTICS_VMAX_SET_INJECT].valid_flag != 0) {
		oplus_hook_setarg(regs, ARG_1, g_haptic_inject_cases[HAPTICS_VMAX_SET_INJECT].valid_flag);
	}

	return 0;
}

/* inject oplus_haptics_get_available_fifo_memory */
int oplus_hook_handler(haptics_get_available_fifo_memory)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	/* set haptics_boost_vreg_enable fail */
	if (g_haptic_inject_cases[HAPTICS_FIFO_MEM_FULL_INJECT].valid_flag == 1) {
		oplus_hook_return(regs, -1);
	}

	return 0;
}


/* init the hook struct */
oplus_hook_define(haptics_read);
oplus_hook_define(haptics_write);
oplus_hook_entry_define(haptics_config_openloop_lra_period);
oplus_hook_define(haptics_get_closeloop_lra_period);
oplus_hook_entry_define(haptics_load_constant_effect);
oplus_hook_entry_define(haptics_load_custom_effect);
oplus_hook_entry_define(haptics_set_vmax_mv);
oplus_hook_define(haptics_get_available_fifo_memory);




static struct oplus_hook *haptics_interfaces[] = {
	&oplus_hook_name(haptics_read),
	&oplus_hook_name(haptics_write),
	&oplus_hook_name(haptics_config_openloop_lra_period),
	&oplus_hook_name(haptics_get_closeloop_lra_period),
	&oplus_hook_name(haptics_load_constant_effect),
	&oplus_hook_name(haptics_load_custom_effect),
	&oplus_hook_name(haptics_set_vmax_mv),
	&oplus_hook_name(haptics_get_available_fifo_memory),
};

static int register_haptics_hook(void)
{
	int ret = 0;

	ret = register_oplus_hooks(haptics_interfaces, ARRAY_SIZE(haptics_interfaces));

	if (ret) {
		pr_err("%s failed %d\n", __func__, ret);
	}

	return ret;
}

static int oplus_inject_sample_init(void)
{
	oplus_proc_inject_init("haptics/capacity", &oplus_haptics_inject_ops, NULL);

	return 0;
}

static void oplus_inject_sample_exit(void)
{
	if (bhook) {
		unregister_oplus_hooks(haptics_interfaces, ARRAY_SIZE(haptics_interfaces));
	}
}

module_init(oplus_inject_sample_init);
module_exit(oplus_inject_sample_exit);

MODULE_DESCRIPTION("OPLUS device fault inject driver");
MODULE_LICENSE("GPL v2");
