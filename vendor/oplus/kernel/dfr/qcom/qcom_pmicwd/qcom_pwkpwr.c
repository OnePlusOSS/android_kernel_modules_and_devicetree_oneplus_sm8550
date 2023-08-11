// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/kthread.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <uapi/linux/sched/types.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include "soc/oplus/system/oplus_project.h"
#include <linux/debugfs.h>
#include <linux/version.h>

#include "qcom_pmicwd.h"

#define QPNP_PON_KPDPWR_S1_TIMER(pon)		((pon)->base + 0x40)
#define QPNP_PON_KPDPWR_S2_TIMER(pon)		((pon)->base + 0x41)
#define QPNP_PON_KPDPWR_S2_CNTL(pon)		((pon)->base + 0x42)
#define QPNP_PON_KPDPWR_S2_CNTL2(pon)		((pon)->base + 0x43)

#define QPNP_PON_RESET_S1_TIMER(pon)		((pon)->base + 0x44)
#define QPNP_PON_RESET_S2_TIMER(pon)		((pon)->base + 0x45)
#define QPNP_PON_RESET_S2_CNTL(pon) 		((pon)->base + 0x46)
#define QPNP_PON_RESET_S2_CNTL2(pon)		((pon)->base + 0x47)

#define QPNP_PON_S2_CNTL2_EN			BIT(7)
#define QPNP_PON_S2_CNTL2_DIS			0x0

#define QPNP_PON_S2_CNTL_TYPE_MASK		(0xF)
#define QPNP_PON_S2_CNTL_EN			0x1
#define QPNP_PON_S2_CNTL_DIS			0x7

#define QPNP_PON_S1_TIMER_MASK			(0xF)
#define QPNP_PON_KPDPWR_S1_TIMER_TIME    0xF

#define QPNP_PON_S2_TIMER_MASK			(0x7)
#define QPNP_PON_KPDPWR_S2_TIMER_TIME    0x7

/**
 * add qpnp-power-on  kpdpwr off/on
 *
**/
static bool pwkpwr_enable;
static bool volup_enable;
extern struct pmicwd_desc *pmicwd;

#define PWKPWR_DISABLE	(0)
#define PWKPWR_ENABLE	(1)
#define VOLUP_DISABLE	(100)
#define VOLUP_ENABLE	(101)

static void powerkey_dump_disable(struct qpnp_pon *reset_pon)
{
	int rc = 0;

	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_KPDPWR_S2_CNTL2(reset_pon),
			QPNP_PON_S2_CNTL2_EN, QPNP_PON_S2_CNTL2_DIS);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_KPDPWR_S2_CNTL2(reset_pon), rc);
	}

	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_KPDPWR_S2_CNTL(reset_pon),
			QPNP_PON_S2_CNTL_TYPE_MASK, QPNP_PON_S2_CNTL_DIS);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_KPDPWR_S2_CNTL(reset_pon), rc);
	}

	pwkpwr_enable = false;
}

static void powerkey_dump_enable(struct qpnp_pon *reset_pon)
{
	int rc = 0;

	PWD_INFO("QPNP_PON_KPDPWR_S2_CNTL2(pon) addr:%x", QPNP_PON_KPDPWR_S2_CNTL2(reset_pon));
	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_KPDPWR_S2_CNTL2(reset_pon),
			QPNP_PON_S2_CNTL2_EN, QPNP_PON_S2_CNTL2_EN);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_KPDPWR_S2_CNTL2(reset_pon), rc);
	}

	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_KPDPWR_S2_CNTL(reset_pon),
			QPNP_PON_S2_CNTL_TYPE_MASK, QPNP_PON_S2_CNTL_EN);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_KPDPWR_S2_CNTL(reset_pon), rc);
	}

	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_KPDPWR_S1_TIMER(reset_pon),
			QPNP_PON_S1_TIMER_MASK, QPNP_PON_KPDPWR_S1_TIMER_TIME);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_KPDPWR_S1_TIMER(reset_pon), rc);
	}

	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_KPDPWR_S2_TIMER(reset_pon),
			QPNP_PON_S2_TIMER_MASK, QPNP_PON_KPDPWR_S2_TIMER_TIME);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_KPDPWR_S2_TIMER(reset_pon), rc);
	}

	pwkpwr_enable = true;
}

static void volup_dump_disable(struct qpnp_pon *reset_pon)
{
	int rc = 0;

	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_RESET_S2_CNTL2(reset_pon),
			QPNP_PON_S2_CNTL2_EN, QPNP_PON_S2_CNTL2_DIS);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_RESET_S2_CNTL2(reset_pon), rc);
	}

	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_RESET_S2_CNTL(reset_pon),
			QPNP_PON_S2_CNTL_TYPE_MASK, QPNP_PON_S2_CNTL_DIS);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_RESET_S2_CNTL(reset_pon), rc);
	}

	volup_enable = false;
}

static void volup_dump_enable(struct qpnp_pon *reset_pon)
{
	int rc = 0;

	PWD_INFO("QPNP_PON_RESET_S2_CNTL2(pon) addr:%x", QPNP_PON_RESET_S2_CNTL2(reset_pon));
	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_RESET_S2_CNTL2(reset_pon),
			QPNP_PON_S2_CNTL2_EN, QPNP_PON_S2_CNTL2_EN);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_RESET_S2_CNTL2(reset_pon), rc);
	}

	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_RESET_S2_CNTL(reset_pon),
			QPNP_PON_S2_CNTL_TYPE_MASK, QPNP_PON_S2_CNTL_EN);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_RESET_S2_CNTL(reset_pon), rc);
	}

	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_RESET_S1_TIMER(reset_pon),
			QPNP_PON_S1_TIMER_MASK, QPNP_PON_KPDPWR_S1_TIMER_TIME);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_RESET_S1_TIMER(reset_pon), rc);
	}

	rc = dup_qpnp_pon_masked_write(reset_pon, QPNP_PON_RESET_S2_TIMER(reset_pon),
			QPNP_PON_S2_TIMER_MASK, QPNP_PON_KPDPWR_S2_TIMER_TIME);
	if (rc) {
		PWD_ERR("Unable to write to addr=%x, rc(%d)\n", QPNP_PON_RESET_S2_TIMER(reset_pon), rc);
	}

	volup_enable = true;
}

static ssize_t kpdpwr_proc_write(struct file *filep, const char __user *ubuf,
		size_t cnt,loff_t *data)
{
	int rc = 0;
	long val = 0;
	char buf[64] = {0};

	if (!pmicwd) {
		PWD_ERR("kpdpwr_proc_write: pmicwd is null !\n");
		return -EFAULT;
	}

	if (cnt >= sizeof(buf)) {
		PWD_ERR("error param <cnt> !\n");
		return -EINVAL;
	}

	if (copy_from_user(&buf, ubuf, cnt)) {
		PWD_ERR("copy_from_user failed  !\n");
		return -EFAULT;
	}
	buf[cnt] = 0;
	rc = kstrtoul(buf, 0, (unsigned long *)&val);
	if (rc < 0) {
		return rc;
	}

	switch (val) {
	case PWKPWR_DISABLE:
		powerkey_dump_disable(pmicwd->pon);
		break;
	case PWKPWR_ENABLE:
		powerkey_dump_enable(pmicwd->pon);
		break;
	case VOLUP_DISABLE:
		volup_dump_disable(pmicwd->pon);
		break;
	case VOLUP_ENABLE:
		volup_dump_enable(pmicwd->pon);
		break;
	default:
		PWD_ERR("error param <val> !\n");
		break;
	}

	return cnt;
}

static int kpdpwr_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Long pwk dump %s\nLong volup dump %s\n",
			pwkpwr_enable ? "enable\n" : "disable\n",
			volup_enable ? "enable\n" : "disable\n");
	return 0;
}

static int kpdpwr_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, kpdpwr_proc_show, inode->i_private);
}

static struct proc_ops kpdpwr_proc_pops = {
	.proc_open 			= kpdpwr_proc_open,
	.proc_read 			= seq_read,
	.proc_write 			= kpdpwr_proc_write,
	.proc_release		= single_release,
};

void kpdpwr_init(void)
{
	struct proc_dir_entry *pe;

	pe = proc_create("kpdpwr", 0664, NULL, &kpdpwr_proc_pops);
	if (!pe) {
		PWD_ERR("kpdpwr:Failed to register kpdpwr interface\n");
	}

	return;
}
