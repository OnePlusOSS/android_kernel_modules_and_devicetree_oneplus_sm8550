/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2021, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_EMV_I7_
#define _DSI_IRIS_EMV_I7_

int iris_dbgfs_emv_init_i7(struct dsi_display *display);
void irisSetDisableDsppPq_i7(bool enable);
void irisSetExtMvFrc_i7(struct extmv_frc_meta *meta);
bool iris_emv_game_size_changed_i7(void);
int iris_emv_mode_get_i7(void);
void iris_emv_clockinout_i7(int index);
uint32_t iris_emv_perf_dump_i7(int startIndex, int endIndex);
u32 iris_blending_intstat_get_i7(void);
bool iris_health_care_i7(void);
bool iris_health_care2_i7(void);
u32 iris_fi_repeat_state_get_i7(void);
u32 iris_efifo_state_get_i7(void);
void iris_rx0_meta_get_i7(void);
void iris_blending_intstat_clear_i7(bool direct);
void iris_pwil0_intr_raw_clear_i7(bool direct);
bool iris_emv_shock_check_i7(void);
int iris_emv_get_path_bitwidth_i7(void);
void iris_emv_on_lightoff_i7(void);
void iris_emv_on_mipi1_up_i7(void);
void iris_emv_on_mipi1_down_i7(void);
void iris_osd_pipeline_switch_i7(bool on);
void iris_emv_on_dual_open_i7(void);
bool iris_emv_on_dual_on_i7(bool success);
void iris_emv_on_dual_closing_i7(void);
void iris_emv_on_dual_close_i7(void);
bool iris_emv_set_frc_prepare_i7(void);
bool iris_emv_set_frc_on_i7(void);
bool iris_emv_set_frc_off_i7(void);
bool iris_emv_on_display_mode_i7(u32 state);
void iris_emv_mode_switch_i7(int mode);
bool iris_emv_new_dual_enabled_i7(void);
bool iris_emv_game_mode_enabled_i7(void);
void iris_emv_resize_osd_buffer_i7(bool extend);
void iris_emv_efifo_pre_config_i7(void);
void iris_emv_efifo_restore_config_i7(void);
void iris_emv_efifo_enable_i7(bool enable);
void onConfigureMvdMeta_i7(u32 count, u32 *values);

#endif
