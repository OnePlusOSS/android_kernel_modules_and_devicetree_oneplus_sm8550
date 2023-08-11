/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_DUAL_
#define _DSI_IRIS_DUAL_

#include "dsi_iris_reg.h"

#define FHD_H_ACTIVE 1080
#define FHD_V_ACTIVE 2412
#define QHD_H_ACTIVE 1440
#define QHD_V_ACTIVE 3216
#define REFRESH_RATE 120

enum IRIS_DUAL_CH_CTRL_OP {
	DUAL_CTRL_POWER_ON_MIPI_1_DATA_PATH = 0x01,
	DUAL_CTRL_ENABLE_BLENDING = 0x02,
	DUAL_CTRL_DISABLE_BLENDNG = 0x04,
	DUAL_CTRL_POWER_OFF_MIPI_1_DATA_PATH = 0x08,
	DUAL_CTRL_SWITCH_TIMEOUT = 0x10,
	DUAL_CTRL_DUAL_FRC_PREPRE = 0x20,
	DUAL_CTRL_SINGLE_PT_POST = 0x40,
};

enum GET_IRIS_KERNEL_STATUS_OP {
	GET_IRIS_MIPI_CH_1_STATE = 1,
	GET_AP_MIPI_CH_1_STATE = 2,
	GET_IRIS_BLEND_STATE = 4,
	GET_IRIS_PWIL_MODE_STATE = 8,
	GET_AP_MIPI1_AUTO_REFRESH_STATE = 0x10,
	GET_IRIS_FI_REPEAT_STATE = 0x20,
	GET_AP_DISP_MODE_STATE = 0x40,
	GET_MV_RESOLUTION = 0x80,
	GET_FRC_VFR_STATUS = 0x100,
	GET_AP_PANEL_TIMING = 0x200,
	GET_AP_ADFR_MODE    = 0x400,
	GET_AP_TE_TYPE = 0x800,
};

enum IRIS_H_SKEW_TYPE {
	AP_SDC_ADFR = 0,				/* SA */
	AP_SDC_MFR  = 1,				/* SM */
	AP_CT_ADFR  = 2,				/* CA */
	AP_CT_MFR   = 3,				/* CM */
};

enum IRIS_PANEL_TIMING_TYPE {
    PANEL_OTHER_TIMING = 0,
    PANEL_FHD_120HZ_ADFR_TIMING = 1,
    PANEL_QHD_120HZ_ADFR_TIMING = 2,
    PANEL_FHD_120HZ_NORMAL_TIMING = 3,
    PANEL_QHD_120HZ_NORMAL_TIMING = 4,
};

void iris_mipi1_enable(void);
int iris_dual_ch_ctrl_cmd_proc(u32 value);
u32 iris_kernel_status_get_cmd_proc(u32 get_op);
void iris_kernel_multistatus_get(u32 op, u32 count, u32 *values);
int iris_osd_auto_refresh_enable(u32 val);
int iris_osd_overflow_status_get(void);
void iris_osd_irq_cnt_init(void);
void iris_pwil0_efifo_enable(bool enable);
void iris_pwil0_efifo_setting_reset(void);
void iris_switch_timeout_dump(void);
void iris_input_frame_cnt_record(void);
void iris_dual_status_clear(bool init);
void iris_osd_comp_ready_pad_select(bool enable, bool commit);
u32  iris_get_ap_timing(void);
#endif
