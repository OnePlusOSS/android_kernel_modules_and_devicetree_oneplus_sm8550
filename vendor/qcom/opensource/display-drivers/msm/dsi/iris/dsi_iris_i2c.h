/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef DSI_IRIS_I2C_H
#define DSI_IRIS_I2C_H

#include <linux/i2c.h>
#include <linux/of.h>

int iris_pure_i2c_burst_write(uint32_t addr, uint32_t *val, uint32_t reg_num);
int iris_pure_i2c_single_write(uint32_t addr, uint32_t val);
int iris_pure_i2c_single_read(uint32_t addr, uint32_t *val);
#endif
