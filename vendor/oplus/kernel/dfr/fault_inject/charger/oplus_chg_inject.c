// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[INJECT]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/version.h>

#include "oplus_inject_hook.h"
#include "oplus_inject_proc.h"

#define SEQ_printf(m, x...)         \
        do {                        \
                if (m)              \
                seq_printf(m, x);   \
                else                \
                pr_debug(x);        \
        } while (0)

enum inject_cmd {
	CMD_REG = 0,
	CMD_UNREG,
	CMD_DISABLE,
};

int oplus_hook_handler(oplus_gauge_get_prev_batt_current) (struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_gauge_get_sub_batt_current)(struct oplus_hook_instance *, struct pt_regs *);

int oplus_hook_handler(oplus_chg_match_temp_for_chging)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_gauge_get_sub_batt_temperature)(struct oplus_hook_instance *, struct pt_regs *);

int oplus_hook_handler(oplus_gauge_get_prev_batt_soc)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_gauge_get_sub_batt_soc)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_gauge_get_batt_soc_v1)(struct oplus_hook_instance *, struct pt_regs *);

int oplus_hook_handler(oplus_chg_wls_get_skewing_current)(struct oplus_hook_instance *, struct pt_regs *);

int oplus_hook_handler(oplus_i2c_smbus_write_byte_data)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_i2c_smbus_write_word_data)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_i2c_smbus_read_byte_data)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_i2c_smbus_read_word_data)(struct oplus_hook_instance *, struct pt_regs *);

int oplus_hook_handler(oplus_gauge_get_prev_batt_mvolts_2cell_max)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_gauge_get_prev_batt_mvolts_2cell_min)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_gauge_get_sub_batt_mvolts)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_gauge_get_batt_mvolts_v1)(struct oplus_hook_instance *, struct pt_regs *);

int oplus_hook_handler(oplus_gauge_get_batt_mvolts_2cell_max_v1)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_gauge_get_batt_mvolts_2cell_min_v1)(struct oplus_hook_instance *, struct pt_regs *);
int oplus_hook_handler(oplus_gauge_get_batt_current_v1)(struct oplus_hook_instance *, struct pt_regs *);

/* init the hook struct */
oplus_hook_define(oplus_gauge_get_batt_current_v1);
oplus_hook_define(oplus_gauge_get_prev_batt_current);
oplus_hook_define(oplus_gauge_get_sub_batt_current);

oplus_hook_define(oplus_chg_match_temp_for_chging);
oplus_hook_define(oplus_gauge_get_sub_batt_temperature);

oplus_hook_define(oplus_gauge_get_prev_batt_soc);
oplus_hook_define(oplus_gauge_get_sub_batt_soc);
oplus_hook_define(oplus_gauge_get_batt_soc_v1);

oplus_hook_define(oplus_chg_wls_get_skewing_current);

oplus_hook_define(oplus_i2c_smbus_write_byte_data);
oplus_hook_define(oplus_i2c_smbus_write_word_data);
oplus_hook_define(oplus_i2c_smbus_read_byte_data);
oplus_hook_define(oplus_i2c_smbus_read_word_data);

oplus_hook_define(oplus_gauge_get_prev_batt_mvolts_2cell_max);
oplus_hook_define(oplus_gauge_get_prev_batt_mvolts_2cell_min);
oplus_hook_define(oplus_gauge_get_sub_batt_mvolts);
oplus_hook_define(oplus_gauge_get_batt_mvolts_v1);
oplus_hook_define(oplus_gauge_get_batt_mvolts_2cell_max_v1);
oplus_hook_define(oplus_gauge_get_batt_mvolts_2cell_min_v1);

struct chg_hook_info {
	const char *name;
	struct oplus_hook *hook;
	long val;
	bool bhook;
	bool enabled;
};

#define ADD_HOOK_INFO(name) { #name, &oplus_hook_name(name), 0, false }

static struct chg_hook_info g_hook_info[] = {
	ADD_HOOK_INFO(oplus_gauge_get_prev_batt_current),
	ADD_HOOK_INFO(oplus_gauge_get_sub_batt_current),
	ADD_HOOK_INFO(oplus_chg_match_temp_for_chging),
	ADD_HOOK_INFO(oplus_gauge_get_sub_batt_temperature),
	ADD_HOOK_INFO(oplus_gauge_get_prev_batt_soc),
	ADD_HOOK_INFO(oplus_gauge_get_sub_batt_soc),
	ADD_HOOK_INFO(oplus_chg_wls_get_skewing_current),
	ADD_HOOK_INFO(oplus_i2c_smbus_write_byte_data),
	ADD_HOOK_INFO(oplus_i2c_smbus_write_word_data),
	ADD_HOOK_INFO(oplus_gauge_get_prev_batt_mvolts_2cell_max),
	ADD_HOOK_INFO(oplus_gauge_get_prev_batt_mvolts_2cell_min),
	ADD_HOOK_INFO(oplus_gauge_get_sub_batt_mvolts),
	ADD_HOOK_INFO(oplus_gauge_get_batt_current_v1),
	ADD_HOOK_INFO(oplus_gauge_get_batt_mvolts_v1),
	ADD_HOOK_INFO(oplus_gauge_get_batt_mvolts_2cell_max_v1),
	ADD_HOOK_INFO(oplus_gauge_get_batt_mvolts_2cell_min_v1),
	ADD_HOOK_INFO(oplus_i2c_smbus_read_byte_data),
	ADD_HOOK_INFO(oplus_i2c_smbus_read_word_data),
	ADD_HOOK_INFO(oplus_gauge_get_batt_soc_v1),
};

int oplus_hook_handler(oplus_gauge_get_prev_batt_current) (struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[0].enabled) {
		oplus_hook_return(regs, g_hook_info[0].val);
		pr_err("%s: %d\n", __func__, g_hook_info[0].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_sub_batt_current)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[1].enabled) {
		oplus_hook_return(regs, g_hook_info[1].val);
		pr_err("%s: %d\n", __func__, g_hook_info[1].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_chg_match_temp_for_chging)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[2].enabled) {
		oplus_hook_return(regs, g_hook_info[2].val);
		pr_err("%s: %d\n", __func__, g_hook_info[2].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_sub_batt_temperature)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[3].enabled) {
		oplus_hook_return(regs, g_hook_info[3].val);
		pr_err("%s: %d\n", __func__, g_hook_info[3].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_prev_batt_soc)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[4].enabled) {
		oplus_hook_return(regs, g_hook_info[4].val);
		pr_err("%s: %d\n", __func__, g_hook_info[4].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_sub_batt_soc)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[5].enabled) {
		oplus_hook_return(regs, g_hook_info[5].val);
		pr_err("%s: %d\n", __func__, g_hook_info[5].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_chg_wls_get_skewing_current)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[6].enabled) {
		oplus_hook_return(regs, g_hook_info[6].val);
		pr_err("%s: %d\n", __func__, g_hook_info[6].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_i2c_smbus_write_byte_data)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[7].enabled) {
		oplus_hook_return(regs, g_hook_info[7].val);
		pr_err("%s: %d\n", __func__, g_hook_info[7].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_i2c_smbus_write_word_data)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[8].enabled) {
		oplus_hook_return(regs, g_hook_info[8].val);
		pr_err("%s: %d\n", __func__, g_hook_info[8].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_prev_batt_mvolts_2cell_max)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[9].enabled) {
		oplus_hook_return(regs, g_hook_info[9].val);
		pr_err("%s: %d\n", __func__, g_hook_info[9].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_prev_batt_mvolts_2cell_min)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[10].enabled) {
		oplus_hook_return(regs, g_hook_info[10].val);
		pr_err("%s: %d\n", __func__, g_hook_info[10].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_sub_batt_mvolts)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[11].enabled) {
		oplus_hook_return(regs, g_hook_info[11].val);
		pr_err("%s: %d\n", __func__, g_hook_info[11].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_batt_current_v1)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[12].enabled) {
		oplus_hook_return(regs, g_hook_info[12].val);
		pr_err("%s: %d\n", __func__, g_hook_info[12].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_batt_mvolts_v1)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[13].enabled) {
		oplus_hook_return(regs, g_hook_info[13].val);
		pr_err("%s: %d\n", __func__, g_hook_info[13].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_batt_mvolts_2cell_max_v1)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[14].enabled) {
		oplus_hook_return(regs, g_hook_info[14].val);
		pr_err("%s: %d\n", __func__, g_hook_info[14].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_batt_mvolts_2cell_min_v1)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[15].enabled) {
		oplus_hook_return(regs, g_hook_info[15].val);
		pr_err("%s: %d\n", __func__, g_hook_info[15].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_i2c_smbus_read_byte_data)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[16].enabled) {
		oplus_hook_return(regs, g_hook_info[16].val);
		pr_err("%s: %d\n", __func__, g_hook_info[16].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_i2c_smbus_read_word_data)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[17].enabled) {
		oplus_hook_return(regs, g_hook_info[17].val);
		pr_err("%s: %d\n", __func__, g_hook_info[17].val);
	}
	return 0;
}

int oplus_hook_handler(oplus_gauge_get_batt_soc_v1)(struct oplus_hook_instance *ri, struct pt_regs *regs)
{
	if (g_hook_info[18].enabled) {
		oplus_hook_return(regs, g_hook_info[18].val);
		pr_err("%s: %d\n", __func__, g_hook_info[18].val);
	}
	return 0;
}

static int register_charger_hook(const char *name, long val)
{
	int rc = -EINVAL;
	int i;

	for (i = 0; i < ARRAY_SIZE(g_hook_info); i++) {
		if (strcmp(name, g_hook_info[i].name) != 0)
			continue;

		g_hook_info[i].val = val;
		if (!g_hook_info[i].bhook) {
			g_hook_info[i].hook->kp.addr = 0;
			rc = register_oplus_hooks(&g_hook_info[i].hook, 1);
			if (rc) {
				pr_err("%s failed, rc=%d\n", __func__, rc);
			} else {
				g_hook_info[i].bhook = true;
				g_hook_info[i].enabled = true;
			}
		} else {
			g_hook_info[i].enabled = true;
		}
	}

	return rc;
}

static void unregister_charger_hook(const char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_hook_info); i++) {
		if (strcmp(name, g_hook_info[i].name) != 0)
			continue;

		if (g_hook_info[i].bhook) {
			unregister_oplus_hooks(&g_hook_info[i].hook, 1);
			g_hook_info[i].bhook = false;
		}
		g_hook_info[i].val = 0;
	}
}

static void disable_charger_hook(const char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_hook_info); i++) {
		if (strcmp(name, g_hook_info[i].name) != 0)
			continue;

		g_hook_info[i].enabled = false;
		g_hook_info[i].val = 0;
	}
}

static int oplus_sample_help_show(struct seq_file *m, void *v)
{
	SEQ_printf(m, "=== charger fault inject ===\n");
	SEQ_printf(m, "register: reg func_name=val\n");
	SEQ_printf(m, "unregister: unreg func_name\n");

	return 0;
}

static int oplus_fault_inject_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, oplus_sample_help_show, inode->i_private);
}

static int oplus_fault_parse_cmd(char *buf, enum inject_cmd *cmd, const char **name, long *val)
{
	int rc;
	int i = 0;
	char *cmd_addr = buf;
	const char *name_addr = NULL;
	const char *val_addr = NULL;
	int buf_len = strlen(buf);

	cmd_addr = buf;
	while(buf[i] != 0) {
		if ((name_addr == NULL) && (buf[i] == ' ')) {
			buf[i] = 0;
			i++;
			if (i < buf_len)
				name_addr = buf + i;
			continue;
		} else if ((val_addr == NULL) && (buf[i] == '=')) {
			buf[i] = 0;
			i++;
			if (i < buf_len)
				val_addr = buf + i;
			continue;
		}
		i++;
	}

	if (name_addr == NULL) {
		pr_err("cmd format error, func name not found\n");
		return -EINVAL;
	}
	*name = name_addr;

	if (strcmp(cmd_addr, "reg") == 0) {
		*cmd = CMD_REG;
	} else if (strcmp(cmd_addr, "unreg") == 0) {
		*cmd = CMD_UNREG;
		return 0;
	} else if (strcmp(cmd_addr, "disable") == 0) {
		*cmd = CMD_DISABLE;
		return 0;
	} else {
		pr_err("Unknown cmd: %s\n", cmd_addr);
		return -EINVAL;
	}

	rc = sscanf(val_addr, "%ld", val);
	if (rc < 0) {
		pr_err("can't get func val, src=%s\n", val_addr);
		return -EINVAL;
	}

	return 0;
}

#define CMD_BUF_SIZE 512
static ssize_t oplus_fault_inject_write(struct file *file,
		const char __user *buf,
		size_t count, loff_t *off)
{
	char cmd_buf[CMD_BUF_SIZE];
	enum inject_cmd cmd;
	long val;
	const char *name;
	int rc;

	if (count > CMD_BUF_SIZE)
		count = CMD_BUF_SIZE;

	if (copy_from_user(cmd_buf, buf, count)) {
		pr_err("%s: read proc input error.\n", __func__);
		return -EINVAL;
	}
	cmd_buf[count - 1] = 0;

	rc = oplus_fault_parse_cmd(cmd_buf, &cmd, &name, &val);
	if (rc < 0)
		return rc;

	switch (cmd) {
	case CMD_REG:
		rc = register_charger_hook(name, val);
		if (rc < 0) {
			pr_err("register %s hook error, rc=%d\n", name, rc);
			return rc;
		}
		pr_info("register %s hook, val=%ld\n", name, val);
		break;
	case CMD_UNREG:
		unregister_charger_hook(name);
		pr_info("unregister %s hook\n", name);
		break;
	case CMD_DISABLE:
		disable_charger_hook(name);
		pr_info("disable %s hook\n", name);
		break;
	}

	return count;
}

static struct proc_ops oplus_charger_inject_ops = {
	.proc_open = oplus_fault_inject_open,
	.proc_read = seq_read,
	.proc_write = oplus_fault_inject_write,
	.proc_release = single_release,
};

static struct proc_tree_node *gNode = NULL;
static int oplus_chg_inject_init(void)
{
	gNode = oplus_proc_inject_init("charger/inject", &oplus_charger_inject_ops, NULL);
	if (!gNode) {
		pr_err("\"charger/inject\" register error\n");
		return -EFAULT;
	}

	return 0;
}

static void oplus_chg_inject_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(g_hook_info); i++) {
		if (g_hook_info[i].bhook)
			unregister_oplus_hooks(&g_hook_info[i].hook, 1);
	}
	if (gNode)
		oplus_proc_inject_exit(gNode);
}

module_init(oplus_chg_inject_init);
module_exit(oplus_chg_inject_exit);

MODULE_LICENSE("GPL v2");
