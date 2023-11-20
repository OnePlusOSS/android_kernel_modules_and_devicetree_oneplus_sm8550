/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_PQ_H_
#define _DSI_IRIS_PQ_H_


enum {
	IRIS_COLOR_TEMP_OFF = 0x00,
	IRIS_COLOR_TEMP_MANUL,
	IRIS_COLOR_TEMP_AUTO,
};


#define IP_OPT_MAX				25

void iris_set_skip_dma(bool skip);

void iris_pq_parameter_init(void);

void iris_dpp_demo_window_set(u8 enable, u8 owAndGamma, u32 xWindow, u32 yWindow);

void iris_dpp_3dlut_gain(u32 count, u32 *values, bool bcommit);

void iris_dpp_3dlut_send(u8 lut_optid, uint8_t chain);

void iris_dpp_fingerDisplay_set(u8 enable, u8 shapeInFill, u32 radius_a, u32 radius_b, u32 radius_c, u32 radius_d, u32 position, u32 fillcolor);

void iris_dpp_fadeinout_enable(u8 enable);

void iris_dpp_fadeinout_step(u8 enable, u32 fadestep);

void iris_dpp_gammamode_set(u32 gammmode, u32 gammaIndex);

void iris_cm_colortemp_mode_set(u32 mode, bool bcommit);

void iris_cm_color_temp_set(void);

u32 iris_cm_ratio_set_for_iic(void);

void iris_cm_color_gamut_pre_set(u32 source_switch);

void iris_cm_color_gamut_pre_clear(void);

void iris_cm_color_gamut_set(u32 level, bool bcommit);

void iris_dpp_gamma_set(void);

void iris_dpp_prelut_rgb_set(u32 lut_optid, bool bcommit);

void iris_dpp_precsc_enable(u32 enable, bool bcommit);

void iris_dpp_precsc_set(bool enable);

void iris_reading_mode_set(u32 level);
void iris_ambient_light_lut_set(u32 lut_offset);
void iris_maxcll_lut_set(u32 lutpos);
void iris_sdr2hdr_level_set(u32 level);
void iris_panel_nits_set(u32 bl_ratio, bool bSystemRestore, int level);
void iris_sdr2hdr_set_demo_win(u8 demo_en, u8 win_out, u32 win_x, u32 win_y);
void iris_sdr2hdr_update_size2(uint32_t img_width, uint32_t img_height);
void iris_sdr2hdr_set_img_size(uint32_t img_width, uint32_t img_height);
void iris_sdr2hdr_set_lce(u32 value);
void iris_sdr2hdr_set_de(u32 value);
void iris_sdr2hdr_set_tf_coef(u32 value);
void iris_sdr2hdr_set_ftc(u32 value);
void iris_sdr2hdr_set_de_ftc(u32 value);
void iris_sdr2hdr_csc_switch(u32 value);
void iris_sdr2hdr_graphic_set(u32 value);
void iris_sdr2hdr_scurve_set(u32 value);
void iris_al_enable(bool enable);
void iris_lux_set(u32 level, bool update);
void iris_aux_channel_lux_set(u32 level);
void iris_scaler_filter_update(u8 scaler_type, u32 level);
void iris_sdr2hdr_ai_tm(u32 value);
void iris_sdr2hdr_ai_lce(u32 value);
void iris_sdr2hdr_ai_de(u32 value);
void iris_sdr2hdr_ai_graphic(u32 value);

void iris_init_ipopt_t(void);
void iris_hdr_csc_prepare(void);
void iris_hdr_csc_complete(int step);
void iris_hdr_csc_frame_ready(void);
void iris_ioinc_filter_ratio_send(void);
void iris_psf_mif_efifo_set(u8 mode, bool osd_enable);
void iris_rx_meta_dma_list_send(u32 meta, bool commit);
void iris_dtg_frame_rate_set(void);

int32_t iris_update_ip_opt(uint8_t ip, uint8_t opt_id, uint8_t chain);

int iris_dbgfs_pq_init(struct dsi_display *display);

u8 iris_get_dbc_lut_index(void);

struct iris_setting_info *iris_get_setting(void);

void iris_set_HDR10_YCoCg(bool val);

bool iris_get_debug_cap(void);

void iris_set_debug_cap(bool val);

void iris_set_sdr2hdr_mode(u8 val);

void iris_quality_setting_off(void);

struct msmfb_iris_ambient_info *iris_get_ambient_lut(void);

struct msmfb_iris_maxcll_info *iris_get_maxcll_info(void);

void iris_scaler_gamma_enable(u32 level);

void iris_dom_set(int mode);

void iris_brightness_level_set(u32 *value);

int32_t iris_parse_color_temp_range(struct device_node *np, struct iris_cfg *pcfg);

int32_t iris_parse_default_pq_param(struct device_node *np, struct iris_cfg *pcfg);

struct msmfb_iris_tm_points_info *iris_get_tm_points_info(void);

void iris_update_tm_lut(void);
void iris_hdr_ai_input_bl(u32 bl_value, bool update);
void iris_ai_bl_al_enable(u32 value, bool update);
void iris_sdr2hdr_allow(bool enable);
void iris_pwil_dport_disable(bool enable, u32 value);
void iris_sdr2hdr_hdr_en(void);
void iris_csc_para_set(uint32_t *values);
void iris_csc_para_reset(void);
void iris_csc2_para_set(uint32_t *values);
void iris_csc2_para_reset(void);
void iris_hdr_power_set(void);
struct msmfb_iris_demura_info *iris_get_demura_info(void);
void iris_update_demura_lut(void);
void iris_demura_enable(bool enable);
void iris_pwil_dpp_en(bool dpp_en);
void iris_update_demura_xy_lut(void);
struct msmfb_iris_demura_xy *iris_get_demura_xy(void);
void iris_sr_level_set(u32 mode, u32 guided_level, u32 dejaggy_level, u32 peaking_level, u32 DLTI_level);
void iris_set_ai_lce_disable(bool val);
bool iris_get_ai_lce_disable(void);
void iris_sdr2hdr_hdr_update(void);
void iris_init_tm_points_lut(void);
void iris_brightness_para_reset(void);
void iris_sdr2hdr_set_degain(u32 mode);
#endif // _DSI_IRIS_PQ_H_
