/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2020-2021 The Linux Foundation. All rights reserved.
 */

#ifndef __SDE_THERMAL_CORE_H__
#define __SDE_THERMAL_CORE_H__

#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/thermal.h>
#include <linux/notifier.h>

struct sde_cdev {
	struct blocking_notifier_head notifier_head;
	struct thermal_cooling_device *cdev;
	struct backlight_device *bd;
	unsigned long thermal_state;
	unsigned int cdev_sf;
};

#if IS_ENABLED(CONFIG_THERMAL_OF)
struct sde_cdev *backlight_cdev_register(struct device *dev,
					struct backlight_device *bd,
					struct notifier_block *n);
void backlight_cdev_unregister(struct sde_cdev *cdev);
#else
static inline struct sde_cdev *
backlight_cdev_register(struct device *dev,
			struct backlight_device *bd, struct notifier_block *n)
{
	return NULL;
}
static inline void backlight_cdev_unregister(struct sde_cdev *cdev)
{ }
#endif /* CONFIG_THERMAL_OF */

#endif
