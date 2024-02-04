/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_BACK_H_
#define _DSI_IRIS_BACK_H_

void iris_loop_back_reset(void);

void iris_set_esd_status(bool enable);

int iris_loop_back_validate(void);

int iris_mipi_rx0_validate(void);

int iris_dbgfs_loop_back_init(struct dsi_display *display);


/* Internal API in kernel for I7 */
int iris_loop_back_validate_i7(void);

int iris_mipi_rx0_validate_i7(void);

void iris_set_loopback_flag_i7(uint32_t val);

uint32_t iris_get_loopback_flag_i7(void);


/* Internal API in kernel for I7p */
int iris_loop_back_validate_i7p(void);

int iris_mipi_rx0_validate_i7p(void);

void iris_set_loopback_flag_i7p(uint32_t val);

uint32_t iris_get_loopback_flag_i7p(void);

#endif // _DSI_IRIS_BACK_H_
