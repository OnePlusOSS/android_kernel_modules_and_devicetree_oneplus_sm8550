// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <dsi_display.h>
#include "dsi_iris_timing_switch_def.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_memc.h"


#define TM_SW_MAX_SEQ_COUNT 32
static struct iris_ctrl_opt dynamic_switch_opt[TM_SW_MAX_SEQ_COUNT] = {};
static uint32_t dynamic_opt_index;


static void _iris_reset_dynamic_seq(void)
{
	dynamic_opt_index = 0;
	memset(dynamic_switch_opt, 0, sizeof(dynamic_switch_opt));
}

static void _iris_sync_blending(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg->dual_enabled || !pcfg->frc_enabled)
		return;

	dynamic_switch_opt[dynamic_opt_index].ip = IRIS_IP_BLEND;
	dynamic_switch_opt[dynamic_opt_index].opt_id = 0x20;
	dynamic_switch_opt[dynamic_opt_index].chain = 0x01;
	dynamic_opt_index++;
}

static void _iris_sync_frc_phase(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint8_t opt = 0;

	if (!pcfg->frc_enabled)
		return;

	if (pcfg->memc_info.panel_fps == 120)
		opt = 2;
	else if (pcfg->memc_info.panel_fps == 90)
		opt = 1;

	dynamic_switch_opt[dynamic_opt_index].ip = FRC_PHASE_LUT;
	dynamic_switch_opt[dynamic_opt_index].opt_id = opt;
	dynamic_switch_opt[dynamic_opt_index].chain = 0x01;
	dynamic_opt_index++;
}

static void _iris_sync_dtg(void)
{
	dynamic_switch_opt[dynamic_opt_index].ip = IRIS_IP_DTG;
	dynamic_switch_opt[dynamic_opt_index].opt_id = 0x00;
	dynamic_switch_opt[dynamic_opt_index].chain = 0x01;
	dynamic_opt_index++;

	if (iris_low_latency_mode_get() == ULTRA_LT_MODE) {
		dynamic_switch_opt[dynamic_opt_index].ip = IRIS_IP_DTG;
		dynamic_switch_opt[dynamic_opt_index].opt_id = 0xF4;
		dynamic_switch_opt[dynamic_opt_index].chain = 0x01;
		dynamic_opt_index++;
	}

	dynamic_switch_opt[dynamic_opt_index].ip = IRIS_IP_DTG;
	dynamic_switch_opt[dynamic_opt_index].opt_id = 0xF9;
	dynamic_switch_opt[dynamic_opt_index].chain = 0x01;
	dynamic_opt_index++;
}

static void _iris_trigger_dma(void)
{
	// for blending, channel 13
	dynamic_switch_opt[dynamic_opt_index].ip = IRIS_IP_DMA;
	dynamic_switch_opt[dynamic_opt_index].opt_id = 0xE5;
	dynamic_switch_opt[dynamic_opt_index].chain = 0x00;
	dynamic_opt_index++;
}


void iris_init_timing_switch_i7(void)
{
	IRIS_LOGI("%s()", __func__);

	_iris_reset_dynamic_seq();
}

void iris_send_dynamic_seq_i7(void)
{
	ktime_t ktime = 0;

	IRIS_LOGI("%s()", __func__);

	if (LOG_DEBUG_INFO)
		ktime = ktime_get();

	SDE_ATRACE_BEGIN(__func__);
	_iris_reset_dynamic_seq();
	_iris_sync_blending();
	_iris_sync_frc_phase();
	_iris_sync_dtg();
	_iris_trigger_dma();
	iris_send_assembled_pkt(dynamic_switch_opt, dynamic_opt_index);
	usleep_range(100, 101);
	SDE_ATRACE_END(__func__);

	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), cost '%d us'", __func__,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));

	return;
}
