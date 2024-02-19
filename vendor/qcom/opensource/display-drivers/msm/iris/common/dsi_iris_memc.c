// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/types.h>
#include <dsi_drm.h>
#include <sde_encoder_phys.h>
#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_iris.h"
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_log.h"
#include "dsi_iris_memc.h"


static struct iris_memc_func memc_func;


void iris_memc_func_init(void)
{
	u32 chip = iris_get_chip_type();

	if (chip == CHIP_IRIS7)
		iris_memc_func_init_i7(&memc_func);
	else
		iris_memc_func_init_i7p(&memc_func);
}

void iris_register_osd_irq(void *disp)
{
	if (!memc_func.register_osd_irq)
		return;

	memc_func.register_osd_irq(disp);
}

void iris_update_panel_ap_te(u32 new_te)
{
	if (!memc_func.update_panel_ap_te)
		return;

	memc_func.update_panel_ap_te(new_te);
}

void iris_inc_osd_irq_cnt(void)
{
	if (!memc_func.inc_osd_irq_cnt)
		return;

	memc_func.inc_osd_irq_cnt();
}

bool iris_is_display1_autorefresh_enabled(void *phys_enc)
{
	bool rc = false;

	if (!memc_func.is_display1_autorefresh_enabled)
		return rc;

	rc = memc_func.is_display1_autorefresh_enabled(phys_enc);

	return rc;
}

void iris_pt_sr_set(int enable, int processWidth, int processHeight)
{
	if (!memc_func.pt_sr_set)
		return;

	memc_func.pt_sr_set(enable, processWidth, processHeight);
}

int iris_configure_memc(u32 type, u32 value)
{
	int rc = 0;

	if (!memc_func.configure_memc)
		return rc;

	rc = memc_func.configure_memc(type, value);

	return rc;
}

int iris_configure_ex_memc(u32 type, u32 count, u32 *values)
{
	int rc = 0;

	if (!memc_func.configure_ex_memc)
		return rc;

	rc = memc_func.configure_ex_memc(type, count, values);

	return rc;
}

int iris_configure_get_memc(u32 type, u32 count, u32 *values)
{
	int rc = 0;

	if (!memc_func.configure_get_memc)
		return rc;

	rc = memc_func.configure_get_memc(type, count, values);

	return rc;
}

void iris_init_memc(void)
{
	if (!memc_func.init_memc)
		return;

	memc_func.init_memc();
}

void iris_lightoff_memc(void)
{
	if (!memc_func.lightoff_memc)
		return;

	memc_func.lightoff_memc();
}

void iris_enable_memc(struct dsi_panel *panel)
{
	if (!memc_func.enable_memc)
		return;

	memc_func.enable_memc(panel);
}

void iris_sr_update(void)
{
	if (!memc_func.sr_update)
		return;

	memc_func.sr_update();
}

void iris_frc_setting_init(void)
{
	if (!memc_func.frc_setting_init)
		return;

	memc_func.frc_setting_init();
}

int iris_dbgfs_memc_init(struct dsi_display *display)
{
	int rc = 0;

	if (!memc_func.dbgfs_memc_init)
		return rc;

	rc = memc_func.dbgfs_memc_init(display);

	return rc;
}

void iris_parse_memc_param0(struct device_node *np)
{
	if (!memc_func.parse_memc_param0)
		return;

	memc_func.parse_memc_param0(np);
}

void iris_parse_memc_param1(void)
{
	if (!memc_func.parse_memc_param1)
		return;

	memc_func.parse_memc_param1();
}

void iris_frc_timing_setting_update(void)
{
	if (!memc_func.frc_timing_setting_update)
		return;

	memc_func.frc_timing_setting_update();
}

void iris_pt_sr_reset(void)
{
	if (!memc_func.pt_sr_reset)
		return;

	memc_func.pt_sr_reset();
}

void iris_mcu_state_set(u32 mode)
{
	if (!memc_func.mcu_state_set)
		return;

	memc_func.mcu_state_set(mode);
}

void iris_mcu_ctrl_set(u32 ctrl)
{
	if (!memc_func.mcu_ctrl_set)
		return;

	memc_func.mcu_ctrl_set(ctrl);
}

void iris_memc_vfr_video_update_monitor(struct iris_cfg *pcfg, struct dsi_display *display)
{
	if (!memc_func.memc_vfr_video_update_monitor)
		return;

	memc_func.memc_vfr_video_update_monitor(pcfg, display);
}

int iris_low_latency_mode_get(void)
{
	int rc = 0;

	if (!memc_func.low_latency_mode_get)
		return rc;

	rc = memc_func.low_latency_mode_get();

	return rc;
}

bool iris_health_care(void)
{
	bool rc = 0;

	if (!memc_func.health_care)
		return rc;

	rc = memc_func.health_care();

	return rc;
}

void iris_dsi_rx_mode_switch(u8 rx_mode)
{
	if (!memc_func.dsi_rx_mode_switch)
		return;

	memc_func.dsi_rx_mode_switch(rx_mode);
}
