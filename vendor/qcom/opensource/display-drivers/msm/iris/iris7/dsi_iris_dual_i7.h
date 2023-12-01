/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_DUAL_I7_
#define _DSI_IRIS_DUAL_I7_

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
	GET_IRIS_ESD_CNT = 0x400,
};

void iris_mipi1_enable_i7(void);
int iris_dual_ch_ctrl_cmd_proc_i7(u32 value);
u32 iris_kernel_status_get_cmd_proc_i7(u32 get_op);
void iris_kernel_multistatus_get_i7(u32 op, u32 count, u32 *values);
int iris_osd_auto_refresh_enable_i7(u32 val);
int iris_osd_overflow_status_get_i7(void);
u32 iris_pwil_mode_state_get_i7(void);
void iris_osd_irq_cnt_init_i7(void);
void iris_pwil0_efifo_enable_i7(bool enable);
void iris_pwil0_efifo_setting_reset_i7(void);
void iris_switch_timeout_dump_i7(void);
void iris_input_frame_cnt_record_i7(void);
void iris_dual_status_clear_i7(bool init);
void iris_osd_comp_ready_pad_select_i7(bool enable, bool commit);
u32 iris_ap_is_support_memcsdr2hdr_timing_i7(void);
bool iris_is_display1_autorefresh_enabled_i7(void *phys_enc);
void iris_register_osd_irq_i7(void *disp);
void iris_inc_osd_irq_cnt_i7(void);
#endif
