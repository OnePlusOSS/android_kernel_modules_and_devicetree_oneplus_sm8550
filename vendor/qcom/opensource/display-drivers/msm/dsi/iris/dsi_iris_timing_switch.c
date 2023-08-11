// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <dsi_drm.h>
#include <sde_trace.h>
#include "dsi_panel.h"
#include "dsi_iris.h"
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_log.h"
#include "dsi_iris_memc.h"
#include "dsi_iris_emv.h"
#include "dsi_iris_dts_fw.h"
#include "dsi_iris_timing_switch.h"

#define to_dsi_display(x) container_of(x, struct dsi_display, host)

enum {
	SWITCH_ABYP_TO_ABYP = 0,
	SWITCH_ABYP_TO_PT,
	SWITCH_PT_TO_ABYP,
	SWITCH_PT_TO_PT,
	SWITCH_NONE,
};

#define SWITCH_CASE(case)[SWITCH_##case] = #case
static const char * const switch_case_name[] = {
	SWITCH_CASE(ABYP_TO_ABYP),
	SWITCH_CASE(ABYP_TO_PT),
	SWITCH_CASE(PT_TO_ABYP),
	SWITCH_CASE(PT_TO_PT),
	SWITCH_CASE(NONE),
};
#undef SWITCH_CASE

enum {
	TIMING_SWITCH_RES_SEQ,
	TIMING_SWITCH_FPS_SEQ,
	TIMING_SWITCH_FPS_CLK_SEQ,
	TIMING_SWITCH_SEQ_CNT,
};


#define TM_SW_MAX_SEQ_COUNT 32
static struct iris_ctrl_opt dynamic_switch_opt[TM_SW_MAX_SEQ_COUNT] = {};
static uint32_t dynamic_opt_index;
static uint32_t switch_case;
static struct iris_ctrl_seq tm_switch_seq[TIMING_SWITCH_SEQ_CNT];
static uint32_t cmd_list_index;
static uint32_t panel_tm_num;
static uint32_t iris_cmd_list_cnt;
static uint8_t *tm_cmd_map_arry;
static uint8_t *master_tm_cmd_map_arry;
static struct dsi_mode_info *panel_tm_arry;
static uint32_t cur_tm_index;
static uint32_t new_tm_index;
static uint32_t last_pt_tm_index;
static uint32_t log_level;

enum {
	LOG_NORMAL_LEVEL,
	LOG_DEBUG_LEVEL,
	LOG_VERBOSE_LEVEL,
	LOG_VERY_VERBOSE_LEVEL,
	LOG_DUMP_CMDLIST_LEVEL = 10,
};
#define LOG_NORMAL_INFO		(IRIS_IF_LOGI())
#define LOG_DEBUG_INFO		((log_level >= LOG_DEBUG_LEVEL) || IRIS_IF_LOGD())
#define LOG_VERBOSE_INFO	((log_level >= LOG_VERBOSE_LEVEL) || IRIS_IF_LOGV())
#define LOG_VERY_VERBOSE_INFO	((log_level >= LOG_VERY_VERBOSE_LEVEL) || IRIS_IF_LOGVV())


static void _iris_reset_dynamic_seq(void)
{
	dynamic_opt_index = 0;
	memset(dynamic_switch_opt, 0, sizeof(dynamic_switch_opt));
}

void iris_init_timing_switch(void)
{
	IRIS_LOGI("%s()", __func__);
	switch_case = SWITCH_ABYP_TO_ABYP;
	cmd_list_index = IRIS_DTSI_PIP_IDX_START;
	_iris_reset_dynamic_seq();
}

void iris_deinit_timing_switch(void)
{
	if (panel_tm_arry) {
		vfree(panel_tm_arry);
		panel_tm_arry = NULL;
	}

	if (tm_cmd_map_arry) {
		vfree(tm_cmd_map_arry);
		tm_cmd_map_arry = NULL;
	}

	if (master_tm_cmd_map_arry) {
		vfree(master_tm_cmd_map_arry);
		master_tm_cmd_map_arry = NULL;
	}
}

static bool _iris_support_timing_switch(void)
{
	if (panel_tm_arry == NULL || panel_tm_num == 0)
		return false;

	return true;
}

void iris_set_panel_timing(struct dsi_display *display, uint32_t index,
		const struct dsi_mode_info *timing)
{
	if (!iris_is_chip_supported())
		return;

	if (display == NULL || timing == NULL || index >= panel_tm_num)
		return;

	// for primary display only, skip secondary
	if (strcmp(display->display_type, "primary"))
		return;

	if (!_iris_support_timing_switch())
		return;

	IRIS_LOGI("%s(), timing@%u: %ux%u@%uHz",
			__func__, index,
			timing->h_active, timing->v_active, timing->refresh_rate);
	memcpy(&panel_tm_arry[index], timing, sizeof(struct dsi_mode_info));
}

static void _iris_init_param(struct device_node *np)
{
	int32_t pnl_tm_num = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return;

	pnl_tm_num = p_dts_ops->count_u8_elems(np, "pxlw,timing-cmd-map");
	if (pnl_tm_num < 1)
		pnl_tm_num = 0;

	panel_tm_num = pnl_tm_num;
	panel_tm_arry = NULL;
	tm_cmd_map_arry = NULL;
	master_tm_cmd_map_arry = NULL;
	cur_tm_index = 0;
	new_tm_index = 0;
	last_pt_tm_index = 0;

	IRIS_LOGI("%s(), panel timing num: %d", __func__, pnl_tm_num);
	if (panel_tm_num > 1) {
		int32_t master_tm_num = 0;
		u32 buf_size = panel_tm_num * sizeof(struct dsi_mode_info);

		panel_tm_arry = vzalloc(buf_size);
		tm_cmd_map_arry = vzalloc(panel_tm_num);

		master_tm_num = p_dts_ops->count_u8_elems(np, "pxlw,master-timing-cmd-map");
		if (master_tm_num > 0) {
			IRIS_LOGI("%s(), master timing map number: %d",
					__func__, master_tm_num);
			if (master_tm_num == panel_tm_num) {
				master_tm_cmd_map_arry = vzalloc(panel_tm_num);
				IRIS_LOGI("%s(), support master timing", __func__);
			}
		} else {
			IRIS_LOGI("%s(), don't support master timing", __func__);
		}
	}
}

static int32_t _iris_parse_timing_cmd_map(struct device_node *np)
{
	int32_t rc = 0;
	uint32_t i = 0;
	uint32_t j = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	rc = p_dts_ops->read_u8_array(np, "pxlw,timing-cmd-map",
			tm_cmd_map_arry, panel_tm_num);

	if (rc != 0) {
		IRIS_LOGE("%s(), failed to parse timing cmd map", __func__);
		return rc;
	}

	for (i = 0; i < panel_tm_num; i++) {
		IRIS_LOGI("%s(), cmd list %u for timing@%u",
				__func__, tm_cmd_map_arry[i], i);

		if (tm_cmd_map_arry[i] != IRIS_DTSI_NONE) {
			bool redup = false;

			for (j = 0; j < i; j++) {
				if (tm_cmd_map_arry[j] == tm_cmd_map_arry[i]) {
					redup = true;
					break;
				}
			}

			if (!redup)
				iris_cmd_list_cnt++;
		}
	}

	IRIS_LOGI("%s(), valid cmd list count is %u", __func__, iris_cmd_list_cnt);

	return rc;
}

static int32_t _iris_parse_master_timing_cmd_map(struct device_node *np)
{
	int32_t rc = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;
	if (master_tm_cmd_map_arry == NULL)
		return rc;

	rc = p_dts_ops->read_u8_array(np, "pxlw,master-timing-cmd-map",
			master_tm_cmd_map_arry, panel_tm_num);
	if (rc != 0) {
		IRIS_LOGE("%s(), failed to parse master timing cmd map", __func__);
		return rc;
	}

	if (LOG_NORMAL_INFO) {
		uint32_t i = 0;

		for (i = 0; i < panel_tm_num; i++) {
			IRIS_LOGI("%s(), master timing map[%u] = %u", __func__,
					i, master_tm_cmd_map_arry[i]);
		}
	}

	return rc;
}

static int32_t _iris_parse_res_switch_seq(struct device_node *np)
{
	int32_t rc = 0;
	const uint8_t *key = "pxlw,iris-res-switch-sequence";


	rc = iris_parse_optional_seq(np, key, &tm_switch_seq[TIMING_SWITCH_RES_SEQ]);
	IRIS_LOGE_IF(rc != 0, "%s(), failed to parse %s seq", __func__, key);

	return rc;
}

static int32_t _iris_parse_fps_switch_seq(struct device_node *np)
{
	int32_t rc = 0;
	const uint8_t *key = "pxlw,iris-fps-switch-sequence";

	rc = iris_parse_optional_seq(np, key, &tm_switch_seq[TIMING_SWITCH_FPS_SEQ]);
	IRIS_LOGE_IF(rc != 0, "%s(), failed to parse %s seq", __func__, key);

	return rc;
}

static int32_t _iris_parse_fps_clk_switch_seq(struct device_node *np)
{
	int32_t rc = 0;
	const uint8_t *key = "pxlw,iris-fps-clk-switch-sequence";

	rc = iris_parse_optional_seq(np, key, &tm_switch_seq[TIMING_SWITCH_FPS_CLK_SEQ]);
	IRIS_LOGE_IF(rc != 0, "%s(), failed to parse %s seq", __func__, key);

	return rc;
}

uint32_t iris_get_cmd_list_cnt(void)
{
	if (iris_cmd_list_cnt == 0)
		return IRIS_DTSI_PIP_IDX_CNT;

	return iris_cmd_list_cnt;
}

int32_t iris_parse_timing_switch_info(struct device_node *np)
{
	int32_t rc = 0;

	_iris_init_param(np);

	if (panel_tm_num == 0 || panel_tm_num == 1)
		return 0;

	rc = _iris_parse_timing_cmd_map(np);
	IRIS_LOGI_IF(rc != 0,
			"%s(), [optional] have not timing cmd map", __func__);

	rc = _iris_parse_master_timing_cmd_map(np);
	IRIS_LOGI_IF(rc != 0,
			"%s(), [optional] have not master timing cmd map", __func__);

	rc = _iris_parse_res_switch_seq(np);
	IRIS_LOGI_IF(rc != 0,
			"%s(), [optional] have not resolution switch sequence", __func__);

	rc = _iris_parse_fps_switch_seq(np);
	IRIS_LOGI_IF(rc != 0,
			"%s(), [optional] have not fps switch sequence", __func__);

	rc = _iris_parse_fps_clk_switch_seq(np);
	IRIS_LOGI_IF(rc != 0,
			"%s(), [optional] have not fps clk switch sequence", __func__);

	return 0;
}

static void _iris_send_res_switch_pkt(void)
{
	struct iris_ctrl_seq *pseq = &tm_switch_seq[TIMING_SWITCH_RES_SEQ];
	struct iris_ctrl_opt *arr = NULL;
	ktime_t ktime = 0;

	SDE_ATRACE_BEGIN(__func__);
	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), cmd list index: %02x", __func__, cmd_list_index);

	if (pseq == NULL) {
		IRIS_LOGE("%s(), seq is NULL", __func__);
		SDE_ATRACE_END(__func__);
		return;
	}

	arr = pseq->ctrl_opt;

	if (LOG_VERBOSE_INFO) {
		int32_t i = 0;

		for (i = 0; i < pseq->cnt; i++) {
			IRIS_LOGI("%s(), i_p: %02x, opt: %02x, chain: %02x", __func__,
					arr[i].ip, arr[i].opt_id, arr[i].chain);
		}
	}

	if (LOG_DEBUG_INFO)
		ktime = ktime_get();
	iris_send_assembled_pkt(arr, pseq->cnt);
	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), send sequence cost '%d us'", __func__,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));

	usleep_range(100, 101);
	SDE_ATRACE_END(__func__);
}

static bool _iris_need_fps_clk_seq(void)
{
	if (tm_switch_seq[TIMING_SWITCH_FPS_CLK_SEQ].ctrl_opt == NULL)
		return false;

	if (panel_tm_arry[new_tm_index].clk_rate_hz
			!= panel_tm_arry[last_pt_tm_index].clk_rate_hz) {
		IRIS_LOGI_IF(LOG_DEBUG_INFO,
				"%s(), switch with different clk, from %llu to %llu",
				__func__,
				panel_tm_arry[last_pt_tm_index].clk_rate_hz,
				panel_tm_arry[new_tm_index].clk_rate_hz);
		return true;
	}

	return false;
}

static void _iris_send_fps_switch_pkt(void)
{
	struct iris_ctrl_seq *pseq = &tm_switch_seq[TIMING_SWITCH_FPS_SEQ];
	struct iris_ctrl_opt *arr = NULL;
	ktime_t ktime = 0;

	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), cmd list index: %02x", __func__, cmd_list_index);

	if (pseq == NULL) {
		IRIS_LOGE("%s(), seq is NULL", __func__);
		return;
	}

	if (_iris_need_fps_clk_seq()) {
		pseq = &tm_switch_seq[TIMING_SWITCH_FPS_CLK_SEQ];
		IRIS_LOGI("with different clk");
	}

	arr = pseq->ctrl_opt;

	if (LOG_VERBOSE_INFO) {
		int32_t i = 0;

		for (i = 0; i < pseq->cnt; i++) {
			IRIS_LOGI("%s(), i_p: %02x, opt: %02x, chain: %02x", __func__,
					arr[i].ip, arr[i].opt_id, arr[i].chain);
		}
	}

	SDE_ATRACE_BEGIN(__func__);
	if (LOG_DEBUG_INFO)
		ktime = ktime_get();

	iris_send_assembled_pkt(arr, pseq->cnt);
	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), send dtsi seq cost '%d us'", __func__,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));

	//usleep_range(100, 101);
	SDE_ATRACE_END(__func__);
}

static uint32_t _iris_get_timing_index(const struct dsi_mode_info *timing)
{
	uint32_t i = 0;

	if (!_iris_support_timing_switch())
		return 0;

	for (i = 0; i < panel_tm_num; i++) {
		struct dsi_mode_info *t = &panel_tm_arry[i];

		if (timing->v_active == t->v_active &&
				timing->h_active == t->h_active &&
				timing->refresh_rate == t->refresh_rate)
			return i;
	}

	return 0;
}

static uint32_t _iris_generate_switch_case(struct dsi_panel *panel,
		const struct dsi_mode_info *new_timing)
{
	bool cur_pt_mode = false;
	u32 new_cmd_list_idx = 0;

	if (!_iris_support_timing_switch())
		return SWITCH_ABYP_TO_ABYP;

	cur_pt_mode = iris_is_pt_mode(panel);
	new_tm_index = _iris_get_timing_index(new_timing);
	new_cmd_list_idx = tm_cmd_map_arry[new_tm_index];

	if (new_cmd_list_idx != IRIS_DTSI_NONE)
		cmd_list_index = new_cmd_list_idx;

	if (cur_pt_mode) {
		if (new_cmd_list_idx == IRIS_DTSI_NONE)
			return SWITCH_PT_TO_ABYP;

		return SWITCH_PT_TO_PT;
	}

	return SWITCH_ABYP_TO_ABYP;
}

static bool _iris_is_same_res(const struct dsi_mode_info *new_timing,
		const struct dsi_mode_info *old_timing)
{
	IRIS_LOGI_IF(LOG_VERBOSE_INFO,
			"%s(), switch from %ux%u to %ux%u",
			__func__,
			old_timing->h_active, old_timing->v_active,
			new_timing->h_active, new_timing->v_active);

	if (old_timing->h_active == new_timing->h_active
			&& old_timing->v_active == new_timing->v_active)
		return true;

	return false;
}

static bool _iris_is_same_fps(const struct dsi_mode_info *new_timing,
		const struct dsi_mode_info *old_timing)
{
	IRIS_LOGI_IF(LOG_VERBOSE_INFO,
			"%s(), switch from %u to %u",
			__func__,
			old_timing->refresh_rate, new_timing->refresh_rate);

	if (new_timing->refresh_rate == old_timing->refresh_rate)
		return true;

	return false;
}

void iris_update_last_pt_timing(const struct dsi_mode_info *pt_timing)
{
	if (!_iris_support_timing_switch())
		return;

	last_pt_tm_index = _iris_get_timing_index(pt_timing);
}

void iris_update_panel_timing(const struct dsi_mode_info *panel_timing)
{
	u32 new_cmd_list_idx = 0;

	if (!_iris_support_timing_switch())
		return;

	new_tm_index = _iris_get_timing_index(panel_timing);
	new_cmd_list_idx = tm_cmd_map_arry[new_tm_index];

	if (new_cmd_list_idx != IRIS_DTSI_NONE)
		cmd_list_index = new_cmd_list_idx;

	cur_tm_index = new_tm_index;
}

bool iris_is_abyp_timing(const struct dsi_mode_info *new_timing)
{
	uint32_t tm_index = 0;

	if (!new_timing)
		return false;

	if (tm_cmd_map_arry == NULL)
		return false;

	tm_index = _iris_get_timing_index(new_timing);
	if (tm_cmd_map_arry[tm_index] == IRIS_DTSI_NONE)
		return true;

	return false;
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

static void _iris_switch_fps_impl(void)
{
	ktime_t ktime = 0;

	_iris_send_fps_switch_pkt();

	if (LOG_DEBUG_INFO)
		ktime = ktime_get();

	_iris_reset_dynamic_seq();
	_iris_sync_blending();
	_iris_sync_frc_phase();
	_iris_sync_dtg();
	_iris_trigger_dma();
	iris_send_assembled_pkt(dynamic_switch_opt, dynamic_opt_index);
	usleep_range(100, 101);

	if (LOG_VERBOSE_INFO) {
		int32_t i = 0;

		for (i = 0; i < dynamic_opt_index; i++) {
			IRIS_LOGI("%s(), i_p: %02x, opt: %02x, chain: %02x",
					__func__,
					dynamic_switch_opt[i].ip,
					dynamic_switch_opt[i].opt_id,
					dynamic_switch_opt[i].chain);
		}
	}

	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), send dynamic seq cost '%d us'", __func__,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));
}

uint32_t iris_get_cmd_list_index(void)
{
	return cmd_list_index;
}

bool iris_is_master_timing_supported(void)
{
	if (master_tm_cmd_map_arry != NULL)
		return true;

	return false;
}

uint8_t iris_get_master_timing_type(void)
{
	if (iris_is_master_timing_supported())
		return master_tm_cmd_map_arry[cur_tm_index];

	return IRIS_DTSI_NONE;
}

static uint32_t _iris_cmd_to_timing(uint32_t cmd_index)
{
	uint32_t i = 0;

	for (i = 0; i < panel_tm_num; i++) {
		if (tm_cmd_map_arry[i] == cmd_index)
			return i;
	}

	return cur_tm_index;
}

static bool _iris_skip_sync(uint32_t cmd_index)
{
	uint32_t cur_cmd_index = iris_get_cmd_list_index();
	struct dsi_mode_info *cur_tm = NULL;
	struct dsi_mode_info *tm = NULL;

	IRIS_LOGI_IF(LOG_VERY_VERBOSE_INFO,
			"%s(), cmd index: %u, current cmd index: %u", __func__,
			cmd_index, cur_cmd_index);

	if (cmd_index == cur_cmd_index)
		return true;

	tm = &panel_tm_arry[_iris_cmd_to_timing(cmd_index)];
	cur_tm = &panel_tm_arry[_iris_cmd_to_timing(cur_cmd_index)];

	IRIS_LOGI_IF(LOG_VERY_VERBOSE_INFO,
			"%s(), timing: %ux%u@%u, current timing: %ux%u@%u",
			__func__,
			tm->h_active, tm->v_active, tm->refresh_rate,
			cur_tm->h_active, cur_tm->v_active, cur_tm->refresh_rate);

	if (tm->v_active == cur_tm->v_active &&
			tm->h_active == cur_tm->h_active &&
			tm->refresh_rate != cur_tm->refresh_rate)
		return false;

	return true;
}

void iris_sync_payload(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value)
{
	struct dsi_cmd_desc *pdesc = NULL;
	struct iris_cfg *pcfg = NULL;
	uint32_t *pvalue = NULL;
	uint32_t type = 0;

	if (!_iris_support_timing_switch())
		return;

	if (iris_is_master_timing_supported())
		return;

	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return;

	for (type = IRIS_DTSI_PIP_IDX_START; type < iris_get_cmd_list_cnt(); type++) {
		if (_iris_skip_sync(type))
			continue;

		pdesc = iris_get_specific_desc_from_ipopt(ip, opt_id, pos, type);
		if (!pdesc) {
			IRIS_LOGE("%s(), failed to find right desc.", __func__);
			return;
		}

		pcfg = iris_get_cfg();
		pvalue = (uint32_t *)((uint8_t *)pdesc->msg.tx_buf + (pos * 4) % pcfg->split_pkt_size);
		pvalue[0] = value;
	}
}

void iris_sync_bitmask(struct iris_update_regval *pregval)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t orig_val = 0;
	uint32_t *data = NULL;
	uint32_t val = 0;
	struct iris_ip_opt *popt = NULL;
	uint32_t type = 0;

	if (!_iris_support_timing_switch())
		return;

	if (iris_is_master_timing_supported())
		return;

	if (!pregval) {
		IRIS_LOGE("%s(), invalid input", __func__);
		return;
	}

	ip = pregval->ip;
	opt_id = pregval->opt_id;
	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return;

	for (type = IRIS_DTSI_PIP_IDX_START; type < iris_get_cmd_list_cnt(); type++) {
		if (_iris_skip_sync(type))
			continue;

		popt = iris_find_specific_ip_opt(ip, opt_id, type);
		if (popt == NULL) {
			IRIS_LOGW("%s(), can't find i_p: 0x%02x opt: 0x%02x, from type: %u",
					__func__, ip, opt_id, type);
			continue;
		} else if (popt->cmd_cnt != 1) {
			IRIS_LOGW("%s(), invalid bitmask for i_p: 0x%02x, opt: 0x%02x, type: %u, popt len: %d",
					__func__, ip, opt_id, type, popt->cmd_cnt);
			continue;
		}

		data = (uint32_t *)popt->cmd[0].msg.tx_buf;
		orig_val = cpu_to_le32(data[2]);
		val = orig_val & (~pregval->mask);
		val |= (pregval->value & pregval->mask);
		data[2] = val;
	}
}

void iris_sync_current_ipopt(uint8_t ip, uint8_t opt_id)
{
	struct iris_ip_opt *popt = NULL;
	struct iris_ip_opt *spec_popt = NULL;
	uint32_t type = 0;
	int i = 0;
	ktime_t ktime = 0;

	if (LOG_DEBUG_INFO)
		ktime = ktime_get();

	if (!_iris_support_timing_switch())
		return;

	if (iris_is_master_timing_supported())
		return;

	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return;

	popt = iris_find_ip_opt(ip, opt_id);
	if (popt == NULL)
		return;

	for (type = IRIS_DTSI_PIP_IDX_START; type < iris_get_cmd_list_cnt(); type++) {
		if (_iris_skip_sync(type))
			continue;

		spec_popt = iris_find_specific_ip_opt(ip, opt_id, type);
		if (spec_popt == NULL)
			continue;

		for (i = 0; i < popt->cmd_cnt; i++) {
			memcpy((void *)spec_popt->cmd[i].msg.tx_buf,
					popt->cmd[i].msg.tx_buf,
					popt->cmd[i].msg.tx_len);
			if (LOG_VERBOSE_INFO)
				print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 32, 4,
						popt->cmd[i].msg.tx_buf, popt->cmd[i].msg.tx_len, false);
		}
	}

	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), for i_p: %02x opt: 0x%02x, cost '%d us'",
			__func__, ip, opt_id,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));
}

void iris_pre_switch(struct dsi_panel *panel,
		struct dsi_mode_info *new_timing)
{
	if (panel == NULL || new_timing == NULL)
		return;

	SDE_ATRACE_BEGIN(__func__);
	iris_update_panel_ap_te(new_timing->refresh_rate);
	switch_case = _iris_generate_switch_case(panel, new_timing);

	IRIS_LOGI("%s(), timing@%u, h_skew = %u, %ux%u@%uHz, cmd list: %u, case: %s",
			__func__,
			new_tm_index,
			new_timing->h_skew,
			new_timing->h_active,
			new_timing->v_active,
			new_timing->refresh_rate,
			cmd_list_index,
			switch_case_name[switch_case]);
	IRIS_LOGI_IF(iris_is_pt_mode(panel),
			"%s(), FRC: %s, DUAL: %s",
			__func__,
			iris_get_cfg()->frc_enabled ? "true" : "false",
			iris_get_cfg()->dual_enabled ? "true" : "false");

	cur_tm_index = new_tm_index;
	SDE_ATRACE_END(__func__);

	IRIS_LOGI_IF(LOG_VERBOSE_INFO,
			"%s(), exit.", __func__);
}

static void _iris_switch_proc(struct dsi_mode_info *new_timing)
{
	struct dsi_mode_info *last_pt_timing = &panel_tm_arry[last_pt_tm_index];

	if (new_timing == NULL || last_pt_timing == NULL)
		return;

	if (!_iris_is_same_res(new_timing, last_pt_timing)) {
		IRIS_LOGI("%s(), RES switch.", __func__);
		_iris_send_res_switch_pkt();
		return;
	}

	if (!_iris_is_same_fps(new_timing, last_pt_timing)) {
		IRIS_LOGI("%s(), FPS switch.", __func__);
		_iris_switch_fps_impl();
	}
}

int iris_switch(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *switch_cmds,
		struct dsi_mode_info *new_timing)
{
	int rc = 0;
	int lightup_opt = iris_lightup_opt_get();
	u32 refresh_rate = new_timing->refresh_rate;
	ktime_t ktime = 0;

	IRIS_LOGI_IF(LOG_DEBUG_INFO,
			"%s(), new timing index: timing@%u", __func__, new_tm_index);

	if (LOG_NORMAL_INFO)
		ktime = ktime_get();

	SDE_ATRACE_BEGIN(__func__);
	iris_update_panel_ap_te(refresh_rate);

	if (lightup_opt & 0x8) {
		rc = iris_abyp_send_panel_cmd(panel, switch_cmds);
		IRIS_LOGI("%s(), force switch from ABYP to ABYP, total cost '%d us'",
				__func__,
				(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));
		SDE_ATRACE_END(__func__);
		return rc;
	}

	switch (switch_case) {
	case SWITCH_ABYP_TO_ABYP:
		SDE_ATRACE_BEGIN("iris_abyp_send_panel_cmd");
		rc = iris_abyp_send_panel_cmd(panel, switch_cmds);
		SDE_ATRACE_END("iris_abyp_send_panel_cmd");
		break;
	case SWITCH_PT_TO_PT:
		{
		ktime_t ktime_0 = ktime_get();

		SDE_ATRACE_BEGIN("iris_pt_send_panel_cmd");
		rc = iris_pt_send_panel_cmd(panel, switch_cmds);
		SDE_ATRACE_END("iris_pt_send_panel_cmd");
		IRIS_LOGI("%s(), send panel cmd cost '%d us'", __func__,
				(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime_0)));


		_iris_switch_proc(new_timing);
		if (panel->qsync_mode > 0)
			iris_qsync_set(true);
		last_pt_tm_index = new_tm_index;
		//iris_health_care();
		}
		break;
	case SWITCH_PT_TO_ABYP:
		iris_abyp_switch_proc(to_dsi_display(panel->host), ANALOG_BYPASS_MODE);
		rc = iris_abyp_send_panel_cmd(panel, switch_cmds);
		break;
	default:
		IRIS_LOGE("%s(), invalid case: %u", __func__, switch_case);
		break;
	}

	SDE_ATRACE_END(__func__);
	IRIS_LOGI("%s(), return %d, total cost '%d us'",
			__func__,
			rc, (u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));

	return rc;
}

uint32_t iris_get_cont_type_with_timing_switch(struct dsi_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t type = IRIS_CONT_SPLASH_NONE;
	uint32_t sw_case = SWITCH_NONE;

	if (pcfg->valid >= PARAM_PARSED)
		sw_case = _iris_generate_switch_case(panel, &panel->cur_mode->timing);

	IRIS_LOGI("%s(), switch case: %s, %ux%u@%u",
			__func__,
			switch_case_name[sw_case],
			panel->cur_mode->timing.h_active,
			panel->cur_mode->timing.v_active,
			panel->cur_mode->timing.refresh_rate);

	switch (sw_case) {
	case SWITCH_PT_TO_PT:
		type = IRIS_CONT_SPLASH_LK;
		break;
	case SWITCH_ABYP_TO_ABYP:
	case SWITCH_ABYP_TO_PT:
		type = IRIS_CONT_SPLASH_BYPASS_PRELOAD;
		break;
	case SWITCH_PT_TO_ABYP:
		// This case does not happen
	default:
		type = IRIS_CONT_SPLASH_NONE;
		break;
	}

	return type;
}

void iris_switch_from_abyp_to_pt(void)
{
	ktime_t ktime = 0;
	struct dsi_mode_info *cur_timing = NULL;

	if (!_iris_support_timing_switch())
		return;

	IRIS_LOGI("%s(), current timing@%u, last pt timing@%u",
			__func__, cur_tm_index, last_pt_tm_index);

	if (cur_tm_index == last_pt_tm_index)
		return;

	if (tm_cmd_map_arry[cur_tm_index] == IRIS_DTSI_NONE)
		return;

	if (LOG_NORMAL_INFO)
		ktime = ktime_get();

	cur_timing = &panel_tm_arry[cur_tm_index];

	SDE_ATRACE_BEGIN(__func__);
	iris_update_panel_ap_te(cur_timing->refresh_rate);
	_iris_switch_proc(cur_timing);
	last_pt_tm_index = cur_tm_index;
	SDE_ATRACE_END(__func__);

	IRIS_LOGI("%s(), total cost '%d us'",
			__func__,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));
}


void iris_dump_cmdlist(uint32_t val)
{
	int ip_type = 0;
	int ip_index = 0;
	int opt_index = 0;
	int desc_index = 0;
	struct iris_ip_index *pip_index = NULL;

	if (val == 0)
		return;

	IRIS_LOGW("%s() enter.", __func__);

	for (ip_type = 0; ip_type < iris_get_cmd_list_cnt(); ip_type++) {
		pip_index = iris_get_ip_idx(ip_type);
		pr_err("\n");
		if (ip_type == cmd_list_index)
			pr_err("*iris-cmd-list-%d*\n", ip_type);
		else
			pr_err("iris-cmd-list-%d\n", ip_type);

		for (ip_index = 0; ip_index < IRIS_IP_CNT; ip_index++) {
			if (pip_index[ip_index].opt_cnt == 0 || pip_index[ip_index].opt == NULL)
				continue;

			for (opt_index = 0; opt_index < pip_index[ip_index].opt_cnt; opt_index++) {
				if (pip_index[ip_index].opt[opt_index].cmd_cnt == 0 ||
						pip_index[ip_index].opt[opt_index].cmd == NULL)
					continue;

				pr_err("\n");
				pr_err("%02x %02x\n", ip_index, pip_index[ip_index].opt[opt_index].opt_id);
				for (desc_index = 0; desc_index < pip_index[ip_index].opt[opt_index].cmd_cnt; desc_index++) {
					if (pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_buf == NULL ||
							pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_len == 0)
						continue;

					print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 32, 4,
							pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_buf,
							pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_len, false);
				}
			}
		}
	}

	IRIS_LOGW("%s() exit.", __func__);
}

void iris_send_adfr_te_scanline_cmd(bool enable)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc desc[1] = {{{0},0,0,0,0}};
	struct dsi_panel_cmd_set cmdset = {
		.count = 1,
		.cmds = &desc[0]
	};
	u8 dataOn[10]  = {0x39, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x44, 0x09, 0x3A};
	u8 dataOff[10] = {0x39, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x44, 0x00, 0x00};

	desc[0].msg.type = dataOn[0];
	desc[0].last_command = true;
	desc[0].msg.channel = dataOn[2];
	desc[0].post_wait_ms = dataOn[4];
	if (enable)
		desc[0].msg.tx_buf = &dataOn[7];
	else
		desc[0].msg.tx_buf = &dataOff[7];
	desc[0].msg.tx_len = dataOn[6];

	if (iris_get_abyp_mode(pcfg->panel) == PASS_THROUGH_MODE)
		rc = iris_pt_send_panel_cmd(pcfg->panel, &cmdset);
	else
		rc = iris_abyp_send_panel_cmd(pcfg->panel, &cmdset);

	if (rc)
		IRIS_LOGE("[%s] enable: %d failed\n", __func__, enable);
	else
		IRIS_LOGI("[%s] enable: %d successed\n", __func__, enable);
}

#define TSP_VSYNC_CMD_COUNT 9
#define TSP_VSYNC_CMD_DATA_COUNT 12

void iris_send_tsp_vsync_scanline_cmd(bool enable)
{
	int rc = 0;
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc desc[TSP_VSYNC_CMD_COUNT] = {{{0},0,0,0,0}};
	struct dsi_panel_cmd_set cmdset = {
		.count = TSP_VSYNC_CMD_COUNT,
		.cmds = &desc[0]
	};
	u8 cmd_data[TSP_VSYNC_CMD_COUNT][TSP_VSYNC_CMD_DATA_COUNT] = {
		{0x39, 0x00, 0x00, 0x40, 0x00, 0x00, 0x03, 0xF0, 0x5A, 0x5A},
		{0x39, 0x00, 0x00, 0x40, 0x00, 0x00, 0x04, 0xB0, 0x00, 0x24, 0xB9},
		{0x15, 0x00, 0x00, 0x40, 0x00, 0x00, 0x02, 0xB9, 0x21},
		{0x39, 0x00, 0x00, 0x40, 0x00, 0x00, 0x04, 0xB0, 0x00, 0x38, 0xB9},
		{0x15, 0x00, 0x00, 0x40, 0x00, 0x00, 0x02, 0xB9, 0x02},  /* TSP_VSYNC Fixed TE 02:120 03:90 05:60 */
		{0x39, 0x00, 0x00, 0x40, 0x00, 0x00, 0x04, 0xB0, 0x00, 0x2A, 0xB9},
		{0x39, 0x00, 0x00, 0x40, 0x00, 0x00, 0x03, 0xB9, 0x03, 0xE8},  /* TSP_VSYNC Fixed TE + 1000 H */
		{0x15, 0x00, 0x00, 0x40, 0x00, 0x00, 0x02, 0xF7, 0x0F},
		{0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0xA5, 0xA5}
	};

	switch (pcfg->panel_te) {
		case 60:
			cmd_data[4][8] = 0x05;
			break;
		case 90:
			cmd_data[4][8] = 0x03;
			break;
		default:
			break;
	}

	if (!enable) {
		cmd_data[6][8] = 0x00;
		cmd_data[6][9] = 0x00;
	}

	for (i = 0; i < TSP_VSYNC_CMD_COUNT; i++) {
		desc[i].msg.type = cmd_data[i][0];
		desc[i].last_command = ((cmd_data[i][3] & MIPI_DSI_MSG_BATCH_COMMAND)? false:true);
		desc[i].msg.channel = cmd_data[i][2];
		desc[i].post_wait_ms = cmd_data[i][4];
		desc[i].msg.tx_buf = &cmd_data[i][7];
		desc[i].msg.tx_len = cmd_data[i][6];
		IRIS_LOGI("type %x, last %x, vc %x, wait %x, len %x\n", desc[i].msg.type, desc[i].last_command, desc[i].msg.channel, desc[i].post_wait_ms, desc[i].msg.tx_len);
		IRIS_LOGI("data %x %x %x %x %x\n", cmd_data[i][7], cmd_data[i][8], cmd_data[i][9], cmd_data[i][10], cmd_data[i][11]);
	}

	if (iris_get_abyp_mode(pcfg->panel) == PASS_THROUGH_MODE)
		rc = iris_pt_send_panel_cmd(pcfg->panel, &cmdset);
	else
		rc = iris_abyp_send_panel_cmd(pcfg->panel, &cmdset);

	if (rc)
		IRIS_LOGE("[%s] enable: %d failed\n", __func__, enable);
	else
		IRIS_LOGI("[%s] enable: %d successed\n", __func__, enable);
}


void iris_set_tm_sw_loglevel(uint32_t level)
{
	if (level == LOG_DUMP_CMDLIST_LEVEL) {
		iris_dump_cmdlist(1);
		return;
	}

	log_level = level;
}

uint32_t iris_get_tm_sw_loglevel(void)
{
	return log_level;
}
