/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __LINUX_EAR_POWER_H
#define __LINUX_EAR_POWER_H

#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/pinctrl/consumer.h>

enum PINCTRL_PIN_STATE {
    PIN_STATE_EAR_PORST_LOW = 0,
    PIN_STATE_EAR_PORST_HIGH,
    PIN_STATE_EAR_DVDD_DISABLE,
    PIN_STATE_EAR_DVDD_ENABLE,
    PIN_STATE_EAR_AVDD_DISABLE,
    PIN_STATE_EAR_AVDD_ENABLE,
    PIN_STATE_MAX
};

/*
 * Platform data for the bluetooth power driver.
 */
struct earpower_platform_data {
    struct platform_device*   pdev;
    struct pinctrl*           pinctrl;
    struct pinctrl_state*     pin_states[PIN_STATE_MAX];
    struct regulator*         dvdd_0p6;
    struct regulator*         dvdd_0p75;
    struct regulator*         avdd_0p9;
    struct regulator*         dvdd_1p8_parent;
    struct regulator*         avdd_1p8_parent;
    struct regulator*         spdt;
    char compatible[32]; /*Bluetooth SoC name */
};

#define EAR_CMD_PWR_CTRL       0xbfad

#endif /* __LINUX_EAR_POWER_H */
