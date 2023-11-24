/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_MEMC_I7P_
#define _DSI_IRIS_MEMC_I7P_

enum IRIS_MEMC_CTRL_OP {
	MEMC_CTRL_FRC_PREPARE = 1,
	MEMC_CTRL_PT2FRC,
	MEMC_CTRL_PT_PREPARE,
	MEMC_CTRL_FRC2PT,
	MEMC_CTRL_PT2RFB,
	MEMC_CTRL_RFB2PT,
	MEMC_CTRL_PT_POST,
	MEMC_CTRL_DPORT_DISABLE,
	MEMC_CTRL_DPORT_ENABLE,
	MEMC_CTRL_FRC_POST,
	MEMC_CTRL_DUAL_FRC2PT,
	MEMC_CTRL_VFR_DISABLE,
	MEMC_CTRL_SWITCH_TIMEOUT = 0x10,
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
	GET_AP_DSI_CLOCK_STATUS = 0x200, //Do not ref HDK
};

enum DEBUG_MEMC_OP {
	DEBUG_FRC_VFR_OW = 0,
	DEBUG_DIS_FRC_DYNEN,
	DEBUG_MCU_ENABLE,
	DEBUG_DIS_MCU_CHECK,
	DEBUG_SWITCH_DUMP_CLEAR,
	DEBUG_FI_DROP_FRM_THR,
	DEBUG_LOW_LATENCY_OW,
	DEBUG_3_BUF_LOW_LATENCY,
	DEBUG_MV_RES_OW,
	DEBUG_N2M_MODE_OW,
	DEBUG_FRC_FORCE_REPEAT,
	DEBUG_OCP_READ_BY_I2C,
	DEBUG_DSPP_PQ_DISABLE,
	DEBUG_FRC_DSC_OW,
	DEBUG_EMV_DEC_ADJ,
	DEBUG_EMV_LOCAL_FALLBACK,
};


void iris_pwil0_efifo_setting_reset_i7p(void);
void iris_memc_info_set_i7p(u32 *values);
void iris_memc_ctrl_cmd_proc_i7p(u32 cmd);
void iris_dspp_pq_mode_set_i7p(u32 value);
void iris_fi_demo_window_set_i7p(u32 mode);
void iris_set_n2m_enable_i7p(bool enable, u8 n2m_ratio);
void iris_dtg_te_n2m_ctrl_setting_send_i7p(bool enable);
void iris_memc_status_clear_i7p(bool init);
void iris_frc_timing_setting_update_i7p(void);
void iris_frc_setting_init_i7p(void);
void iris_parse_memc_param0_i7p(struct device_node *np);
void iris_parse_memc_param1_i7p(void);
int iris_dbgfs_memc_init_i7p(struct dsi_display *display);
void iris_memc_ctrl_frc_prepare_i7p(void);
void iris_memc_ctrl_pt_post_i7p(void);
int iris_low_latency_mode_get_i7p(void);
int iris_debug_memc_option_get_i7p(char *kbuf, int size);
void iris_debug_memc_option_set_i7p(u32 type, u32 value);
void iris_dport_output_mode_reset_i7p(void);
void iris_kernel_status_get_i7p(u32 get_op, u32 count, u32 *values);

#endif
