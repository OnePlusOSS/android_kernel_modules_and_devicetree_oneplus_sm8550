/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#ifndef __QCOM_PMICWD_H__
#define __QCOM_PMICWD_H__

#include <linux/regmap.h>
#include <linux/input/qpnp-power-on.h>

struct pmicwd_desc {
        struct qpnp_pon    *pon;
        struct task_struct *wd_task;
        struct mutex       wd_task_mutex;
        unsigned int       pmicwd_state;/* |reserver|rst type|timeout|enable| */
        u8                 suspend_state;/* record the suspend state */
};

#define PWD_TAG "[PMICWD]"
#define PWD_INFO(fmt, ...) printk(KERN_INFO PWD_TAG pr_fmt(fmt), ##__VA_ARGS__)
#define PWD_WARN(fmt, ...) printk(KERN_WARNING PWD_TAG pr_fmt(fmt), ##__VA_ARGS__)
#define PWD_ERR(fmt, ...) printk(KERN_ERR PWD_TAG pr_fmt(fmt), ##__VA_ARGS__)

#undef ASSERT
#define ASSERT(x) BUG_ON(!(x))

static inline int dup_qpnp_pon_masked_write(struct qpnp_pon *pon, u16 addr, u8 mask, u8 val)
{
        int rc;

        rc = regmap_update_bits(pon->regmap, addr, mask, val);
        if (rc) {
                PWD_ERR("Register write failed, addr=0x%04X, rc=%d\n", addr, rc);
        }

        return rc;
}

void kpdpwr_init(void);
void pmicwd_init(struct platform_device *pdev);
int qpnp_pon_wd_pet(struct qpnp_pon *pon);

#endif
