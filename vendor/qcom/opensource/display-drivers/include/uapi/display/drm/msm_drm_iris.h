/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note
 *
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#ifndef __MSM_DRM_IRIS_H__
#define __MSM_DRM_IRIS_H__

#include <linux/types.h>

#define DRM_MSM_IRIS_OPERATE_CONF      0x50
#define DRM_MSM_IRIS_OPERATE_TOOL      0x51

enum iris_oprt_type {
	IRIS_OPRT_TOOL_DSI,
	IRIS_OPRT_CONFIGURE,
	IRIS_OPRT_CONFIGURE_NEW,
	IRIS_OPRT_CONFIGURE_NEW_GET,
	IRIS_OPRT_MAX_TYPE,
};

struct msmfb_mipi_dsi_cmd {
	__u8 dtype;
	__u8 vc;
#define MSMFB_MIPI_DSI_COMMAND_LAST 1
#define MSMFB_MIPI_DSI_COMMAND_ACK  2
#define MSMFB_MIPI_DSI_COMMAND_HS   4
#define MSMFB_MIPI_DSI_COMMAND_BLLP 8
#define MSMFB_MIPI_DSI_COMMAND_DEBUG 16
#define MSMFB_MIPI_DSI_COMMAND_TO_PANEL 32
#define MSMFB_MIPI_DSI_COMMAND_T 64

	__u32 iris_ocp_type;
	__u32 iris_ocp_addr;
	__u32 iris_ocp_value;
	__u32 iris_ocp_size;

	__u16 flags;
	__u16 length;
	__u8 *payload;
	__u16 rx_length;
	__u8 *rx_buf;
};

struct msm_iris_operate_value {
	unsigned int type;
	unsigned int count;
	void *values;
};

struct msmfb_iris_ambient_info {
	__u32 ambient_lux;
	__u32 ambient_bl_ratio;
	void *lut_lut2_payload;
};

struct msmfb_iris_maxcll_info {
	__u32 mMAXCLL;
	void *lut_luty_payload;
	void *lut_lutuv_payload;
};

#define DRM_IOCTL_MSM_IRIS_OPERATE_CONF \
	DRM_IOW(DRM_COMMAND_BASE + DRM_MSM_IRIS_OPERATE_CONF, struct msm_iris_operate_value)
#define DRM_IOCTL_MSM_IRIS_OPERATE_TOOL \
	DRM_IOW(DRM_COMMAND_BASE + DRM_MSM_IRIS_OPERATE_TOOL, struct msm_iris_operate_value)

#endif /* __MSM_DRM_IRIS_H__ */
