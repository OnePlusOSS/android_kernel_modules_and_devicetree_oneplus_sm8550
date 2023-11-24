/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef __DSI_IRIS_TIMING_SWITCH__
#define __DSI_IRIS_TIMING_SWITCH__


void iris_init_timing_switch(void);
void iris_deinit_timing_switch(void);
int32_t iris_parse_timing_switch_info(struct device_node *np);
uint32_t iris_get_cont_type_with_timing_switch(struct dsi_panel *panel);
uint32_t iris_get_cmd_list_cnt(void);
bool iris_is_abyp_timing(const struct dsi_mode_info *new_timing);
void iris_update_last_pt_timing(const struct dsi_mode_info *pt_timing);
void iris_update_panel_timing(const struct dsi_mode_info *panel_timing);
uint32_t iris_get_cmd_list_index(void);
bool iris_is_master_timing_supported(void);
uint8_t iris_get_master_timing_type(void);
void iris_sync_bitmask(struct iris_update_regval *pregval);
void iris_sync_payload(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value);
void iris_sync_current_ipopt(uint8_t ip, uint8_t opt_id);
void iris_switch_from_abyp_to_pt(void);

void iris_set_tm_sw_loglevel(uint32_t level);
uint32_t iris_get_tm_sw_loglevel(void);
void iris_dump_cmdlist(uint32_t val);
void iris_send_tsp_vsync_scanline_cmd(bool enable);

#endif //__DSI_IRIS_TIMING_SWITCH__
