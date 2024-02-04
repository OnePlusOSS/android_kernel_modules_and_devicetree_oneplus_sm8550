// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#ifndef _DSI_IRIS_MEMC_COMMON_
#define _DSI_IRIS_MEMC_COMMON_

enum mcu_state_op {
	MCU_START,
	MCU_INT_DISABLE,
	MCU_STOP,
};

struct iris_memc_func {
	void (*register_osd_irq)(void *disp);
	void (*update_panel_ap_te)(u32 new_te);
	void (*inc_osd_irq_cnt)(void);
	bool (*is_display1_autorefresh_enabled)(void *phys_enc);
	void (*pt_sr_set)(int enable, int processWidth, int processHeight);
	int (*configure_memc)(u32 type, u32 value);
	int (*configure_ex_memc)(u32 type, u32 count, u32 *values);
	int (*configure_get_memc)(u32 type, u32 count, u32 *values);
	void (*init_memc)(void);
	void (*lightoff_memc)(void);
	void (*enable_memc)(struct dsi_panel *panel);
	void (*sr_update)(void);
	void (*frc_setting_init)(void);
	int (*dbgfs_memc_init)(struct dsi_display *display);
	void (*parse_memc_param0)(struct device_node *np);
	void (*parse_memc_param1)(void);
	void (*frc_timing_setting_update)(void);
	void (*pt_sr_reset)(void);
	void (*mcu_state_set)(u32 mode);
	void (*mcu_ctrl_set)(u32 ctrl);
	void (*memc_vfr_video_update_monitor)(struct iris_cfg *pcfg, struct dsi_display *display);
	int (*low_latency_mode_get)(void);
	bool (*health_care)(void);
	void (*dsi_rx_mode_switch)(uint8_t rx_mode);
};

void iris_memc_func_init_i7(struct iris_memc_func *memc_func);
void iris_memc_func_init_i7p(struct iris_memc_func *memc_func);
void iris_pt_sr_set(int enable, int processWidth, int processHeight);
int iris_configure_memc(u32 type, u32 value);
int iris_configure_ex_memc(u32 type, u32 count, u32 *values);
int iris_configure_get_memc(u32 type, u32 count, u32 *values);
void iris_init_memc(void);
void iris_lightoff_memc(void);
void iris_enable_memc(struct dsi_panel *panel);
void iris_sr_update(void);
void iris_frc_setting_init(void);
int iris_dbgfs_memc_init(struct dsi_display *display);
void iris_parse_memc_param0(struct device_node *np);
void iris_parse_memc_param1(void);
void iris_frc_timing_setting_update(void);
void iris_pt_sr_reset(void);
void iris_mcu_state_set(u32 mode);
void iris_mcu_ctrl_set(u32 ctrl);
void iris_memc_vfr_video_update_monitor(struct iris_cfg *pcfg, struct dsi_display *display);
int iris_low_latency_mode_get(void);
bool iris_health_care(void);






#endif
