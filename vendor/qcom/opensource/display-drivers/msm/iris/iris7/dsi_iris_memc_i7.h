/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_MEMC_I7_
#define _DSI_IRIS_MEMC_I7_

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

enum dsc_ratio_type {
	IRIS_DSC_8BPC_65BPP,
	IRIS_DSC_10BPC_65BPP,
	IRIS_DSC_RATIO_TYPE_MAX
};

static u32 dsc_rc_buf_thresh[] = {0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54,
	0x62, 0x69, 0x70, 0x77, 0x79, 0x7b, 0x7c, 0x7d};

static char dsc_rc_range_min_qp_1_1[][15] = {
	{0, 1, 2, 3, 3, 4, 4, 4, 5, 5, 6, 7, 8, 12, 12},
	{0, 4, 6, 7, 7, 8, 8, 8, 9, 9, 10, 11, 12, 16, 16},
	};

static char dsc_rc_range_max_qp_1_1[][15] = {
	{2, 4, 6, 6, 7, 7, 7, 8, 9, 10, 11, 12, 12, 12, 12},
	{6, 8, 10, 10, 11, 11, 11, 12, 13, 14, 15, 16, 16, 16, 16},
	};

static char dsc_rc_range_bpg_offset[] = {0, -2, -2, -4, -6, -6, -6, -8,
	-8, -10, -10, -12, -12, -12, -12};

struct dsc_info {
	u8 version;
	u8 scr_rev;

	int pic_height;
	int pic_width;
	int slice_height;
	int slice_width;

	int initial_lines;
	int pkt_per_line;
	int bytes_in_slice;
	int bytes_per_pkt;
	int eol_byte_num;
	int pclk_per_line;
	int full_frame_slices;
	int slice_last_group_size;
	int bpp;
	int bpc;
	int line_buf_depth;

	int slice_per_pkt;
	int chunk_size;
	bool block_pred_enable;
	int vbr_enable;
	int enable_422;
	int convert_rgb;
	int input_10_bits;

	int initial_dec_delay;
	int initial_xmit_delay;
	int initial_scale_value;
	int scale_decrement_interval;
	int scale_increment_interval;
	int first_line_bpg_offset;
	int nfl_bpg_offset;
	int slice_bpg_offset;
	int initial_offset;
	int final_offset;

	int rc_model_size;
	int det_thresh_flatness;
	int max_qp_flatness;
	int min_qp_flatness;
	int edge_factor;
	int quant_incr_limit0;
	int quant_incr_limit1;
	int tgt_offset_hi;
	int tgt_offset_lo;

	u32 *buf_thresh;
	char *range_min_qp;
	char *range_max_qp;
	char *range_bpg_offset;
};

struct dsc_encoder_parameters {
	int groupnum_perline;
	int groupnum_persliceline;
	int pnum_persliceline;
	int slice_num;
	int stream_hsize;
	int groupnum_perslice;
	int len_codedslice;
	int len8_codedslice;
	int dwpercodedslice;
	int last_hslice_size;
	int last_hslice_groupnum;
	int remchunks;
	int last_slice_vsize;
};

struct dsc_encoder_ctrl_reg {
	u32 slice_size2;
	u32 slice_size3;
	u32 stream_size;
	u32 len_codedslice;
	u32 len8_codedslice;
	u32 dwpercodedslice;
	u32 ctrl_setting;
	u32 last_hslice;
};

u32 iris_disable_mipi1_autorefresh_get_i7(void);
void iris_memc_info_set_i7(u32 *values);
void iris_memc_ctrl_cmd_proc_i7(u32 cmd);
void iris_blending_cusor_timeout_set_i7(bool dual_frc);
void iris_memc_cmd_payload_init_i7(void);
void iris_memc_cmd_payload_send_i7(void);
void iris_frc_reg_add_i7(u32 addr, u32 val, bool last);
void iris_fi_demo_window_set_i7(u32 mode);
int iris_fi_osd_protect_window_calc_i7(u32 top_left_pos, u32 bottom_right_pos,
				u32 osd_window_ctrl, u32 enable, u32 dynCompensate);
void iris_osd_protect_window_set_i7(void);
void iris_set_panel_te_i7(u8 panel_te);
void iris_update_panel_ap_te_i7(u32 new_te);
void iris_set_n2m_enable_i7(bool enable, u8 n2m_ratio);
void iris_dtg_ovs_dly_setting_send_i7(bool enable);
void iris_dtg_te_n2m_ctrl_setting_send_i7(bool enable);
void iris_hangup_timeout_cnt_update_i7(bool enable);
void iris_memc_vfr_video_update_monitor_i7(struct iris_cfg *pcfg, struct dsi_display *display);
void iris_memc_vfr_update_work_init_i7(void);
void iris_memc_status_clear_i7(bool init);
void iris_memc_mspwil_setting_init_i7(void);
u32 iris_frc_video_hstride_calc_i7(u16 hres, u16 bpp, u16 slice_num);
void iris_frc_dsc_setting_update_i7(u8 mode);
void iris_frc_timing_setting_update_i7(void);
void iris_set_out_frame_rate_i7(u32 fps);
void iris_frc_setting_init_i7(void);
void iris_parse_memc_param0_i7(struct device_node *np);
void iris_parse_memc_param1_i7(void);
int iris_dbgfs_memc_init_i7(struct dsi_display *display);
void iris_frc_ioinc_change_i7(void);
void iris_frc_dsc_change_i7(void);
void iris_sr_change_i7(int inHeight, int inWidth, int outHeight, int outWidth, bool sr_sel, bool update);
void iris_memc_frc_phase_update_i7(void);
void iris_memc_ctrl_frc_prepare_i7(void);
void iris_memc_ctrl_pt_post_i7(void);
void iris_pt_sr_set_i7(int enable, int processWidth, int processHeight);
void iris_pt_sr_reset_i7(void);
void iris_sr_update_i7(void);
int iris_low_latency_mode_get_i7(void);
int iris_tnr_mode_get_i7(void);
int iris_debug_memc_option_get_i7(char *kbuf, int size);
void iris_debug_memc_option_set_i7(u32 type, u32 value);
void iris_dport_output_mode_reset_i7(void);
void iris_mcu_state_set_i7(u32 mode);
void iris_mcu_mode_reset_i7(void);
void iris_pt_sr_restore_i7(void);
void iris_mcu_ctrl_set_i7(u32 sdr_ctrl);
void iris_set_pwil_mode_i7(u8 mode, bool osd_enable, int state, bool commit);
void iris_pwil_idle_mask_update(bool enable);
#endif
