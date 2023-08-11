/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_BACK_H_
#define _DSI_IRIS_BACK_H_


/* API in kernel for recovery mode */
int iris_loop_back_validate(void);

int iris_dbgfs_loop_back_init(struct dsi_display *display);


enum LOOP_BACK_ERR_TYPE {
	ERR_NO_ERR			= 0,
	ERR_PURE_LOOP_BACK	= 1,
	ERR_DUAL_PT			= 2,
	ERR_ATSPEED_EFIFO	= 3,
	ERR_SMT_PMU         = 4,
};

#define BIT_PURE_LOOPBACK		(1 << 0)
#define BIT_DUAL_PT				(1 << 1)
#define BIT_ATSPEED_EFIFO		(1 << 2)
#define BIT_SMT_PMU				(1 << 3)

int iris_mipi_rx0_validate(void);
#endif // _DSI_IRIS_BACK_H_
