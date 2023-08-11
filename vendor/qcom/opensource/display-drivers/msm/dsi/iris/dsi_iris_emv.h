/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2021, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef MDSS_DSI_IRIS_EMV__H
#define MDSS_DSI_IRIS_EMV__H

#include "dsi_iris_reg.h"

#define IRIS_PATH_BW10	10

int iris_dbgfs_emv_init(struct dsi_display *display);
void irisSetDisableDsppPq(bool enable);
void irisSetExtMvFrc(struct extmv_frc_meta meta);
bool iris_emv_game_size_changed(void);
int iris_emv_mode_get(void);
void iris_emv_clockinout(int index);
uint32_t iris_emv_perf_dump(int startIndex, int endIndex);
u32 iris_blending_intstat_get(void);
bool iris_health_care(void);
u32 iris_fi_repeat_state_get(void);
u32 iris_efifo_state_get(void);
void iris_rx0_meta_get(void);
void iris_blending_intstat_clear(bool direct);
bool iris_emv_shock_check(void);
int iris_emv_get_path_bitwidth(void);
void iris_emv_on_lightoff(void);
void iris_emv_on_mipi1_up(void);
void iris_emv_on_mipi1_down(void);
void iris_osd_pipeline_switch(bool on);
void iris_emv_on_dual_open(void);
bool iris_emv_on_dual_on(bool success);
void iris_emv_on_dual_closing(void);
void iris_emv_on_dual_close(void);
bool iris_emv_set_frc_prepare(void);
bool iris_emv_set_frc_on(void);
bool iris_emv_set_frc_off(void);
bool iris_emv_on_display_mode(u32 state);
void iris_emv_mode_switch(int mode);
bool iris_emv_new_dual_enabled(void);
bool iris_emv_game_mode_enabled(void);
void iris_emv_efifo_pre_config(void);
void iris_emv_efifo_restore_config(void);
void iris_emv_efifo_enable(bool enable);

#endif
