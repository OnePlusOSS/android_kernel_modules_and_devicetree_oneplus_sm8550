/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_IOCTL_H_
#define _DSI_IRIS_IOCTL_H_

int iris_configure(u32 display, u32 type, u32 value);

int iris_configure_ex(u32 display, u32 type, u32 count, u32 *values);

int iris_configure_get(u32 display, u32 type, u32 count, u32 *values);

int iris_dbgfs_adb_type_init(struct dsi_display *display);

void iris_ioctl_lock(void);

void iris_ioctl_unlock(void);

/* Internal API in kernel for I7p */

int iris_configure_i7p(u32 display, u32 type, u32 value);

int iris_configure_ex_i7p(u32 display, u32 type, u32 count, u32 *values);

int iris_configure_get_i7p(u32 display, u32 type, u32 count, u32 *values);

int iris_dbgfs_adb_type_init_i7p(struct dsi_display *display);


/* Internal API in kernel for I7 */

int iris_configure_i7(u32 display, u32 type, u32 value);

int iris_configure_ex_i7(u32 display, u32 type, u32 count, u32 *values);

int iris_configure_get_i7(u32 display, u32 type, u32 count, u32 *values);

int iris_dbgfs_adb_type_init_i7(struct dsi_display *display);



#endif // _DSI_IRIS_IOCTL_H_
