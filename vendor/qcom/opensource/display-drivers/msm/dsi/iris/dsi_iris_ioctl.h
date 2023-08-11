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
int iris_debug_display_info_get(char *kbuf, int size);
int iris_debug_display_mode_get(char *kbuf, int size, bool debug);
void iris_debug_info_get(u32 *value, u32 count);
void iris_debug_info_set(u32 type, u32 value);
#endif // _DSI_IRIS_IOCTL_H_
