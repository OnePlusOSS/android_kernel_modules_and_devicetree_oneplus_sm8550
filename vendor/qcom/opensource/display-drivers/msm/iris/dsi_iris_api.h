/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_API_H_
#define _DSI_IRIS_API_H_

#define IRIS_CMD_SIZE SZ_512K


enum IRIS_PLATFORM {
	IRIS_FPGA = 0,
	IRIS_ASIC,
};


#include "dsi_display.h"

void iris_deinit(struct dsi_display *display);
void iris_power_on(struct dsi_panel *panel);
void iris_reset(void);
void iris_power_off(struct dsi_panel *panel);
int iris_set_pinctrl_state(bool enable);
int iris_pt_send_panel_cmd(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset);
int iris_enable(struct dsi_panel *panel, struct dsi_panel_cmd_set *on_cmds);
int iris_disable(struct dsi_panel *panel, bool dead, struct dsi_panel_cmd_set *off_cmds);
void iris_set_panel_timing(struct dsi_display *display, uint32_t index,
		const struct dsi_mode_info *timing);
void iris_pre_switch(struct dsi_panel *panel,
		      struct dsi_mode_info *new_timing);
int iris_switch(struct dsi_panel *panel,
		      struct dsi_panel_cmd_set *switch_cmds,
		      struct dsi_mode_info *new_timing);
void iris_update_2nd_active_timing(struct dsi_panel *panel);
int iris_status_get(struct dsi_display_ctrl *ctrl, struct dsi_panel *panel);
int iris_set_aod(struct dsi_panel *panel, bool aod);
bool iris_get_aod(struct dsi_panel *panel);
int iris_set_fod(struct dsi_panel *panel, bool fod);
int iris_post_fod(struct dsi_panel *panel);

void iris_send_cont_splash(struct dsi_display *display);
bool iris_is_pt_mode(struct dsi_panel *panel);
void iris_prepare(struct dsi_display *display);

int iris_update_backlight(u32 bl_lvl);
int iris_update_backlight_value(u32 bl_lvl);
void iris_control_pwr_regulator(bool on);
int iris_panel_ctrl_read_reg(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel, u8 *rx_buf, int rlen,
		struct dsi_cmd_desc *cmd);

bool iris_is_display1_autorefresh_enabled(void *phys_enc);
bool iris_is_virtual_encoder_phys(void *phys_enc);
void iris_register_osd_irq(void *disp);
void iris_inc_osd_irq_cnt(void);

void iris_query_capability(struct dsi_panel *panel);
void iris_memc_func_init(void);

bool iris_is_chip_supported(void);
bool iris_is_softiris_supported(void);
bool iris_is_dual_supported(void);
u32 iris_get_chip_caps(void);

void iris_sde_plane_setup_csc(void *csc_ptr);
int iris_sde_kms_iris_operate(struct msm_kms *kms,
		u32 operate_type, struct msm_iris_operate_value *operate_value);
void iris_sde_update_dither_depth_map(uint32_t *map, uint32_t depth);
void iris_sde_prepare_commit(uint32_t num_phys_encs, void *phys_enc);
void iris_sde_prepare_for_kickoff(uint32_t num_phys_encs, void *phys_enc);
void iris_sde_encoder_sync_panel_brightness(uint32_t num_phys_encs,
		void *phys_enc);
void iris_sde_encoder_kickoff(uint32_t num_phys_encs, void *phys_enc);
void iris_sde_encoder_wait_for_event(uint32_t num_phys_encs,
		void *phys_enc, uint32_t event);
int iris_sde_connector_set_metadata(uint32_t val);

int msm_ioctl_iris_operate_conf(struct drm_device *dev, void *data,
		struct drm_file *file);
int msm_ioctl_iris_operate_tool(struct drm_device *dev, void *data,
		struct drm_file *file);

void iris_dsi_display_res_init(struct dsi_display *display);
void iris_dsi_display_debugfs_init(struct dsi_display *display,
		struct dentry *dir, struct dentry *dump_file);
void iris_dsi_panel_dump_pps(struct dsi_panel_cmd_set *set);
void iris_dsi_ctrl_dump_desc_cmd(struct dsi_ctrl *dsi_ctrl,
		const struct mipi_dsi_msg *msg);

void iris_sde_hw_sspp_setup_csc_v2(void *pctx, const void *pfmt, void *pdata);
int iris_platform_get(void);
void iris_qsync_set(bool enable);
int iris_esd_ctrl_get(void);
bool iris_check_reg_read(struct dsi_panel *panel);
void iris_dsi_rx_mode_switch(uint8_t rx_mode);
void iris_sw_te_enable(void);
uint32_t iris_schedule_line_no_get(void);
void iris_ioctl_lock(void);
void iris_ioctl_unlock(void);

int iris_pure_i2c_bus_init(void);
void iris_pure_i2c_bus_exit(void);
int iris_i2c_bus_init(void);
void iris_i2c_bus_exit(void);
bool iris_need_short_read_workaround(void);

void iris_fpga_split_set_max_return_size(struct dsi_ctrl *dsi_ctrl, u16 *pdflags);
void iris_fpga_adjust_read_cnt(u32 read_offset,
							   u32 rx_byte,
							   u32 read_cnt,
							   int *pcnt);
void iris_fpga_adjust_read_buf(u32 repeated_bytes,
							   u32 read_offset,
							   u32 rx_byte,
							   u32 read_cnt,
							   u8 *rd_buf);

void iris_update_panel_ap_te(uint32_t new_te);
int iris_EDR_backlight_ctrl(u32 hdr_nit, u32 ratio_panel);
int iris_need_update_pps_one_time(void);
bool iris_qsync_update_need(void);

void iris_sde_crtc_atomic_begin(struct drm_crtc *crtc,
				struct drm_crtc_state *old_state);

#endif // _DSI_IRIS_API_H_
