// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/types.h>
#include <dsi_drm.h>
#include "dsi_iris_def.h"
#include "dsi_iris_log.h"
#include "dsi_iris_memc_helper.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_i3c.h"


#define IRIS_PPS_REG_COUNT 24
#define PPS_SIZE 128
#define IRIS_BPP(val) (((val) * 10) >> 4)
#define IRIS_UP(val) ((val) * 10)
#define IRIS_DOWN(val) ((val) / 10)
#define PROC_INFO(_h, _v) BITS_SET(_v, 16, 16, _h)


enum iris_dsc_ratio_type {
	IRIS_DSC_V12_444_10BPC_10BPP,
	IRIS_DSC_V12_444_10BPC_8BPP,
	IRIS_DSC_V12_444_10BPC_6_5BPP,
	IRIS_DSC_V12_444_8BPC_8BPP,
	IRIS_DSC_V12_444_8BPC_6_5BPP,

	IRIS_DSC_V12_422_10BPC_10BPP,
	IRIS_DSC_V12_422_10BPC_8BPP,
	IRIS_DSC_V12_422_10BPC_6_5BPP,
	IRIS_DSC_V12_422_8BPC_8BPP,
	IRIS_DSC_V12_422_8BPC_6_5BPP,

	IRIS_DSC_V11_444_10BPC_10BPP,
	IRIS_DSC_V11_444_10BPC_8BPP,
	IRIS_DSC_V11_444_10BPC_6_5BPP,
	IRIS_DSC_V11_444_8BPC_8BPP,
	IRIS_DSC_V11_444_8BPC_6_5BPP,
};

enum {
	MEMC_DSC_VER = 0,
	MEMC_DSC_BPC,
	MEMC_DSC_BPP = 2,
	MEMC_DSC_SLICE_PER_PKT,
	MEMC_DSC_SLICE_HEIGHT = 4,
	MEMC_DSC_FMT,
	MEMC_DSC_HW_SLICE_PREFER = 6,
};


static u16 iris_dsc_rc_buf_thresh[DSC_NUM_BUF_RANGES - 1] = {
	0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b, 0x7d, 0x7e
};

static char iris_dsc_rc_range_min_qp[][DSC_NUM_BUF_RANGES] = {
	{0, 4, 5, 5, 7, 7, 7, 7, 7, 7, 9, 9, 9, 12, 15},
	{0, 4, 5, 5, 7, 7, 7, 7, 7, 8, 9, 9, 9, 12, 16},
	{0, 4, 6, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 12, 18},
	{0, 0, 1, 1, 3, 3, 3, 3, 3, 4, 5, 5, 5, 8, 12},
	{0, 1, 2, 3, 4, 4, 5, 5, 5, 5, 6, 6, 6, 9, 14},

	{0, 2, 2, 3, 4, 5, 5, 6, 6, 7, 8, 8, 9, 11, 12},
	{0, 2, 3, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 11, 14},
	{0, 4, 5, 6, 6, 6, 6, 7, 7, 8, 9, 9, 9, 12, 16},
	{0, 0, 1, 2, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 10},
	{0, 0, 1, 2, 3, 3, 3, 3, 3, 3, 5, 5, 5, 8, 12},

	{0, 4, 5, 5, 7, 7, 7, 7, 7, 7, 9, 9, 9, 12, 16},
	{0, 4, 6, 6, 7, 8, 8, 8, 8, 8, 9, 9, 10, 12, 17},
	{0, 4, 7, 7, 9, 9, 9, 9, 9, 10, 10, 10, 10, 12, 18},
	{0, 0, 2, 2, 3, 4, 4, 4, 4, 4, 5, 5, 6, 9, 13},
	{0, 1, 3, 3, 5, 5, 5, 5, 5, 6, 6, 6, 6, 9, 14},
};

static char iris_dsc_rc_range_max_qp[][DSC_NUM_BUF_RANGES] = {
	{7, 8, 9, 10, 11, 11, 11, 12, 13, 13, 14, 14, 15, 15, 16},
	{8, 8, 9, 10, 11, 11, 11, 12, 13, 14, 14, 15, 15, 16, 17},
	{8, 10, 11, 12, 12, 12, 13, 14, 15, 15, 16, 16, 16, 17, 19},
	{4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 10, 11, 11, 12, 13},
	{4, 6, 7, 8, 8, 8, 9, 10, 11, 11, 12, 12, 12, 13, 15},

	{2, 3, 3, 5, 5, 7, 7, 8, 9, 9, 10, 11, 11, 12, 13},
	{4, 6, 7, 9, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 15},
	{8, 8, 9, 10, 11, 11, 11, 12, 13, 14, 14, 15, 15, 16, 17},
	{2, 4, 5, 6, 7, 7, 7, 8, 8, 9, 9, 9, 9, 10, 11},
	{4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 10, 11, 11, 12, 13},

	{7, 8, 9, 10, 11, 11, 11, 12, 13, 13, 14, 14, 15, 15, 17},
	{8, 9, 10, 11, 11, 12, 12, 13, 14, 14, 15, 15, 16, 17, 18},
	{8, 10, 12, 12, 13, 13, 13, 14, 15, 16, 16, 16, 16, 17, 19},
	{4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 13, 14},
	{4, 6, 8, 8, 9, 9, 9, 10, 11, 12, 12, 12, 12, 13, 15},
};

static char iris_dsc_rc_range_bpg[][DSC_NUM_BUF_RANGES] = {
	{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
	{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
	{1, -1, -1, -3, -5, -6, -8, -8, -8, -10, -10, -11, -12, -12, -12},
	{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
	{1, -1, -1, -3, -5, -6, -8, -8, -8, -10, -10, -11, -12, -12, -12},

	{10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -10, -10, -12, -12, -12},
	{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
	{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -11, -12, -12, -12},
	{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
	{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -11, -12, -12, -12},

	{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
	{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
	{1, -1, -1, -3, -5, -6, -8, -8, -8, -10, -10, -11, -12, -12, -12},
	{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12},
	{1, -1, -1, -3, -5, -6, -8, -8, -8, -10, -10, -11, -12, -12, -12},
};

static struct iris_memc_dsc_info iris_dsc_info[DSC_PPS_SET_CNT];
static struct iris_memc_dsc_info iris_dsc_last_info[DSC_PPS_SET_CNT];
static uint32_t iris_dsc_active_path = DSC_PPS_SET_CNT;


static void _iris_dsc_parse_payload(uint32_t *payload, uint32_t pps_sel)
{
	uint32_t version_minor = 0x02;
	uint32_t chroma_format = MSM_CHROMA_422;

	if (payload == NULL) {
		IRIS_LOGE("%s(), invalid payload.", __func__);
		return;
	}

	iris_dsc_info[pps_sel].pic_width =
		(BITS_GET(payload[2], 8, 0) << 8) | BITS_GET(payload[2], 8, 8);
	iris_dsc_info[pps_sel].pic_height =
		(BITS_GET(payload[1], 8, 16) << 8) | BITS_GET(payload[1], 8, 24);
	iris_dsc_info[pps_sel].slice_width =
		(BITS_GET(payload[3], 8, 0) << 8) | BITS_GET(payload[3], 8, 8);
	iris_dsc_info[pps_sel].slice_height =
		(BITS_GET(payload[2], 8, 16) << 8) | BITS_GET(payload[2], 8, 24);
	iris_dsc_info[pps_sel].slice_per_pkt =
		iris_dsc_info[pps_sel].slice_width == 0 ? 1 :
		iris_dsc_info[pps_sel].pic_width /
		iris_dsc_info[pps_sel].slice_width;
	iris_dsc_info[pps_sel].bits_per_component = BITS_GET(payload[0], 4, 28);
	iris_dsc_info[pps_sel].bits_per_pixel = BITS_GET(payload[1], 8, 8);

	version_minor = BITS_GET(payload[0], 4, 0);
	if (version_minor != 0x01 && version_minor != 0x02)
		version_minor = 0x02;
	iris_dsc_info[pps_sel].version_minor = version_minor;

	chroma_format = BITS_GET(payload[22], 1, 0);
	if (chroma_format != MSM_CHROMA_444 && chroma_format != MSM_CHROMA_422)
		chroma_format = MSM_CHROMA_444;
	iris_dsc_info[pps_sel].chroma_format = chroma_format;

	/* DSC1.1 doesn't support YUV422 */
	if (iris_dsc_info[pps_sel].version_minor == 0x01 &&
			iris_dsc_info[pps_sel].chroma_format != MSM_CHROMA_444)
		iris_dsc_info[pps_sel].chroma_format = MSM_CHROMA_444;
}

static void _iris_dsc_parse_info(void)
{
	uint32_t *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_ENC_2, 0x00, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_DSC_ENC_2, 0x00);
		return;
	}

	_iris_dsc_parse_payload(payload, DSC_PPS_SET0);
	iris_dsc_info[DSC_PPS_SET0].hw_slice_prefer = false;

	IRIS_LOGI("%s(), MEMC DSC1.%u, fmt: %s, pc: %#08x, sl: %u, %#08x, bpc: %u, bpp: %u",
			__func__,
			iris_dsc_info[DSC_PPS_SET0].version_minor,
			iris_dsc_info[DSC_PPS_SET0].chroma_format == MSM_CHROMA_444 ? "444" : "422",
			PROC_INFO(iris_dsc_info[DSC_PPS_SET0].pic_width,
				iris_dsc_info[DSC_PPS_SET0].pic_height),
			iris_dsc_info[DSC_PPS_SET0].slice_per_pkt,
			PROC_INFO(iris_dsc_info[DSC_PPS_SET0].slice_width,
				iris_dsc_info[DSC_PPS_SET0].slice_height),
			iris_dsc_info[DSC_PPS_SET0].bits_per_component,
			iris_dsc_info[DSC_PPS_SET0].bits_per_pixel);

	_iris_dsc_parse_payload(payload + IRIS_PPS_REG_COUNT, DSC_PPS_SET1);
	iris_dsc_info[DSC_PPS_SET1].hw_slice_prefer = true;

	IRIS_LOGI("%s(), RFB DSC1.%u, fmt: %s, pc: %#08x, sl: %u, %#08x, bpc: %u, bpp: %u",
			__func__,
			iris_dsc_info[DSC_PPS_SET1].version_minor,
			iris_dsc_info[DSC_PPS_SET1].chroma_format == MSM_CHROMA_444 ? "444" : "422",
			PROC_INFO(iris_dsc_info[DSC_PPS_SET1].pic_width,
				iris_dsc_info[DSC_PPS_SET1].pic_height),
			iris_dsc_info[DSC_PPS_SET1].slice_per_pkt,
			PROC_INFO(iris_dsc_info[DSC_PPS_SET1].slice_width,
				iris_dsc_info[DSC_PPS_SET1].slice_height),
			iris_dsc_info[DSC_PPS_SET1].bits_per_component,
			iris_dsc_info[DSC_PPS_SET1].bits_per_pixel);
}

static void _iris_dsc_init_param(void)
{
	IRIS_LOGI("%s()", __func__);

	iris_dsc_active_path = DSC_PPS_SET_CNT;
}

static void _iris_dsc_setting_off(void)
{
	IRIS_LOGI("%s()", __func__);

	memset(iris_dsc_last_info, 0,
			sizeof(struct iris_memc_dsc_info) * DSC_PPS_SET_CNT);
}

static uint8_t _iris_dsc_dec_initial_tc(uint32_t pps_sel)
{
	uint32_t *payload = NULL;
	uint8_t reg_offset = 2;
	uint32_t bpp = 0;
	uint32_t slice_width = 0;
	uint32_t initial_dec_delay = 0;
	uint8_t dec_initial_tc = 0xf;
	uint8_t fac = 1;

	if (pps_sel == DSC_PPS_SET1)
		reg_offset += IRIS_PPS_REG_COUNT;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_DEN_2, 0x01, reg_offset);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_DSC_DEN_2, 0x01);
		return dec_initial_tc;
	}

	bpp = BITS_GET(payload[1], 2, 0);
	bpp <<= 8;
	bpp |= BITS_GET(payload[1], 8, 8);
	slice_width = BITS_GET(payload[3], 8, 0);
	slice_width <<= 8;
	slice_width |= BITS_GET(payload[3], 8, 8);
	initial_dec_delay = BITS_GET(payload[4], 8, 16);
	initial_dec_delay <<= 8;
	initial_dec_delay |= BITS_GET(payload[4], 8, 24);

	IRIS_LOGI("%s(), bpp = %u, sw = %#x, initial dec delay = %u",
			__func__, bpp, slice_width, initial_dec_delay);

	if (bpp == 0 || slice_width == 0) {
		IRIS_LOGE("%s(), use default value", __func__);
		return dec_initial_tc;
	}

	if (iris_dsc_info[pps_sel].chroma_format == MSM_CHROMA_422)
		fac = 2;

	dec_initial_tc = DIV_ROUND_UP((initial_dec_delay + 3 + 8 +
				DIV_ROUND_UP(6 * 192 * 16, bpp)) * fac, slice_width) + 1;

	IRIS_LOGI("%s(), for pps %u, return: %u",
			__func__, pps_sel, dec_initial_tc);

	return dec_initial_tc;
}

static uint8_t _iris_dsc_enc_initial_lines(uint8_t sw_ss_num, uint8_t hw_ss_num,
		uint32_t pps_sel)
{
	uint32_t *payload = NULL;
	uint8_t reg_offset = 2;
	uint32_t bpp = 0;
	uint32_t slice_width = 0;
	uint32_t chunk_size = 0;
	uint32_t initial_xmit_delay = 0;
	uint32_t base_hs_latency = 0;
	uint32_t output_rate_ratio = 0;
	uint32_t output_ratio_offset = 0;
	uint32_t output_rate_extra_buget_bits = 0;
	uint32_t multi_hs_extr_latency = 0;
	uint32_t enc_initial_lines = 1;

	if (pps_sel == DSC_PPS_SET1)
		reg_offset += IRIS_PPS_REG_COUNT;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_ENC_2, 0x00, reg_offset);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_DSC_ENC_2, 0x00);
		return enc_initial_lines;
	}

	bpp = BITS_GET(payload[1], 2, 0);
	bpp <<= 8;
	bpp |= BITS_GET(payload[1], 8, 8);
	slice_width = BITS_GET(payload[3], 8, 0);
	slice_width <<= 8;
	slice_width |= BITS_GET(payload[3], 8, 8);
	chunk_size = BITS_GET(payload[3], 8, 16);
	chunk_size <<= 8;
	chunk_size |= BITS_GET(payload[3], 8, 24);
	initial_xmit_delay = BITS_GET(payload[4], 2, 0);
	initial_xmit_delay <<= 8;
	initial_xmit_delay |= BITS_GET(payload[4], 8, 8);

	if (iris_dsc_info[pps_sel].chroma_format == MSM_CHROMA_422)
		slice_width <<= 1;

	IRIS_LOGI("%s(), bpp = %u, sw = %#x, chunk size = %u, initial xmit delay = %u",
			__func__, bpp, slice_width, chunk_size, initial_xmit_delay);

	if (bpp == 0 || slice_width == 0) {
		IRIS_LOGE("%s(), use default value", __func__);
		return enc_initial_lines;
	}

	base_hs_latency = initial_xmit_delay + (28 + (3 * (91 + 2) * sw_ss_num)) +
		DIV_ROUND_UP((9 * 128 + 48) * 16, bpp) + 1;
	output_rate_ratio = DIV_ROUND_UP(bpp * 1000, 16 * 64);

	if (hw_ss_num > 1)
		output_ratio_offset = DIV_ROUND_UP(output_rate_ratio * sw_ss_num *
				(1000 - output_rate_ratio) *
				(1000 - (1000 / hw_ss_num) - output_rate_ratio),
				1000000);

	output_rate_extra_buget_bits = DIV_ROUND_UP(
			(1000 - output_rate_ratio - output_ratio_offset) * chunk_size * 8,
			1000);
	multi_hs_extr_latency = DIV_ROUND_UP(
			(DIV_ROUND_UP((hw_ss_num - 1) * chunk_size * 8, hw_ss_num) +
			 output_rate_extra_buget_bits) * 16, bpp) + 5;
	enc_initial_lines = DIV_ROUND_UP(base_hs_latency + multi_hs_extr_latency,
			slice_width);

	IRIS_LOGI("%s(), enc initial lines: %u", __func__, enc_initial_lines);

	return enc_initial_lines;
}

static void _iris_dsc_update_pwil(uint32_t pps_sel, bool send)
{
	uint32_t *payload = NULL;
	uint8_t yuv_422_mode = 0;
	uint8_t frc_bitwidth = 1;
	uint8_t commit = send ? 0 : 1;

	IRIS_LOGI("%s(), path%u send: %s",
			__func__, pps_sel, send ? "true" : "false");

	if (iris_dsc_info[pps_sel].chroma_format == MSM_CHROMA_422)
		yuv_422_mode = 1;

	if (iris_dsc_info[pps_sel].bits_per_component == 8)
		frc_bitwidth = 0;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 4);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find pwil control.", __func__);
		return;
	}
	payload[0] = BITS_SET(payload[0], 2, 14, yuv_422_mode); // FRC_DATA_FORMAT
	payload[1] = BITS_SET(payload[1], 1, 1, frc_bitwidth);  // FRC_BITWIDTH

	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x01, 0x01, commit);
	if (send)
		iris_update_pq_opt(PATH_DSI, true);
}

static void _iris_dsc_update_ctrl(uint32_t pps_sel)
{
	uint32_t *payload = NULL;
	uint8_t hw_slice_num = 1;
	uint8_t sw_slice_num = iris_dsc_info[pps_sel].slice_per_pkt;
	uint8_t flatness_det_thresh = 0;
	uint8_t dec_initial_tc = 0;
	uint8_t enc_initial_lines = 0;
	uint32_t enc_ob_max_addr = 831;
	uint32_t dec_ib_max_addr = 599;

	_iris_dsc_update_pwil(pps_sel, false);

	flatness_det_thresh = 2 << (iris_dsc_info[pps_sel].bits_per_component - 8);

	if (iris_dsc_info[pps_sel].slice_per_pkt == 2) {
		enc_ob_max_addr = iris_dsc_info[pps_sel].hw_slice_prefer ? 735 : 415;
		dec_ib_max_addr = 299;
	}

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_DEN_2, 0x00, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_DSC_DEN_2, 0x00);
		return;
	}

	dec_initial_tc = _iris_dsc_dec_initial_tc(pps_sel);

	if (pps_sel == DSC_PPS_SET0) {
		payload[1] = BITS_SET(payload[1], 4, 0, dec_initial_tc);
		payload[1] = BITS_SET(payload[1], 14, 18, dec_ib_max_addr);
		payload[3] = BITS_SET(payload[3], 8, 0, flatness_det_thresh);
	}

	if (pps_sel == DSC_PPS_SET1) {
		payload[2] = BITS_SET(payload[2], 4, 0, dec_initial_tc);
		payload[2] = BITS_SET(payload[2], 14, 18, dec_ib_max_addr);
		payload[3] = BITS_SET(payload[3], 8, 16, flatness_det_thresh);
	}
	iris_init_update_ipopt_t(IRIS_IP_DSC_DEN_2, 0x00, 0x00, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_DEN_3, 0x00, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_DSC_DEN_3, 0x00);
		return;
	}

	if (pps_sel == DSC_PPS_SET0) {
		payload[1] = BITS_SET(payload[1], 4, 0, dec_initial_tc);
		payload[1] = BITS_SET(payload[1], 14, 18, dec_ib_max_addr);
		payload[3] = BITS_SET(payload[3], 8, 0, flatness_det_thresh);
	}

	if (pps_sel == DSC_PPS_SET1) {
		payload[2] = BITS_SET(payload[2], 4, 0, dec_initial_tc);
		payload[2] = BITS_SET(payload[2], 14, 18, dec_ib_max_addr);
		payload[3] = BITS_SET(payload[3], 8, 16, flatness_det_thresh);
	}
	iris_init_update_ipopt_t(IRIS_IP_DSC_DEN_3, 0x00, 0x00, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_ENC_2, 0x01, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_DSC_ENC_2, 0x01);
		return;
	}

	if (iris_dsc_info[pps_sel].hw_slice_prefer) {
		hw_slice_num = iris_dsc_info[pps_sel].slice_per_pkt;
		sw_slice_num = 1;
	}

	enc_initial_lines = _iris_dsc_enc_initial_lines(sw_slice_num, hw_slice_num,
			pps_sel);

	if (pps_sel == DSC_PPS_SET0) {
		payload[8] = BITS_SET(payload[8], 8, 5, enc_initial_lines);
		payload[8] = BITS_SET(payload[8], 14, 13, enc_ob_max_addr);
		payload[7] = BITS_SET(payload[7], 2, 19, sw_slice_num);
		payload[14] = BITS_SET(payload[14], 2, 3, hw_slice_num);
		payload[15] = BITS_SET(payload[15], 8, 0, flatness_det_thresh);
	}

	if (pps_sel == DSC_PPS_SET1) {
		payload[9] = BITS_SET(payload[9], 8, 0, enc_initial_lines);
		payload[9] = BITS_SET(payload[9], 14, 8, enc_ob_max_addr);
		payload[9] = BITS_SET(payload[9], 2, 22, hw_slice_num);
		payload[9] = BITS_SET(payload[9], 2, 24, sw_slice_num);
		payload[15] = BITS_SET(payload[15], 8, 8, flatness_det_thresh);
	}
	iris_init_update_ipopt_t(IRIS_IP_DSC_ENC_2, 0x01, 0x01, 1);
}

static void _iris_dsc_update_pps(void)
{
	iris_init_update_ipopt_t(IRIS_IP_DSC_DEN_2, 0x01, 0x01, 1);
	iris_init_update_ipopt_t(IRIS_IP_DSC_DEN_3, 0x01, 0x01, 1);
	iris_init_update_ipopt_t(IRIS_IP_DSC_ENC_2, 0x00, 0x00, 1);
}

static void _iris_dsc_update_param(uint32_t pps_sel)
{
	SDE_ATRACE_BEGIN(__func__);
	_iris_dsc_update_ctrl(pps_sel);
	_iris_dsc_update_pps();
	SDE_ATRACE_END(__func__);
}

static void _iris_dsc_replace_pps(uint32_t *pps_reg, uint32_t pps_sel)
{
	uint32_t *payload = NULL;
	uint32_t reg_offset = 2;

	if (pps_reg == NULL) {
		IRIS_LOGE("%s(), invalid input parameter!", __func__);
		return;
	}

	SDE_ATRACE_BEGIN(__func__);
	if (pps_sel == DSC_PPS_SET1)
		reg_offset += IRIS_PPS_REG_COUNT;

	// enc
	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_ENC_2, 0x00, reg_offset);
	if (payload == NULL)
		return;
	memcpy(payload, pps_reg, sizeof(uint32_t) * IRIS_PPS_REG_COUNT);

	// dec0
	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_DEN_2, 0x01, reg_offset);
	if (payload == NULL)
		return;
	memcpy(payload, pps_reg, sizeof(uint32_t) * IRIS_PPS_REG_COUNT);

	// dec1
	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_DEN_3, 0x01, reg_offset);
	if (payload == NULL)
		return;
	memcpy(payload, pps_reg, sizeof(uint32_t) * IRIS_PPS_REG_COUNT);
	SDE_ATRACE_END(__func__);
}

static uint8_t _iris_dsc_get_v1_2_bpg_offset(struct drm_dsc_config *dsc)
{
	uint8_t bpg_offset = 0;
	uint8_t uncompressed_bpg_rate;
	uint16_t bpp = IRIS_BPP(dsc->bits_per_pixel);

	if (dsc->slice_height < 8)
		bpg_offset = 2 * (dsc->slice_height - 1);
	else if (dsc->slice_height < 20)
		bpg_offset = 12;
	else if (dsc->slice_height <= 30)
		bpg_offset = 13;
	else if (dsc->slice_height < 42)
		bpg_offset = 14;
	else
		bpg_offset = 15;

	if (dsc->native_422)
		uncompressed_bpg_rate = IRIS_DOWN(3 * 4 * bpp);
	else if (dsc->native_420)
		uncompressed_bpg_rate = IRIS_DOWN(3 * bpp);
	else
		uncompressed_bpg_rate = (IRIS_DOWN(3 * bpp) + 2) * 3;

	if (bpg_offset < (uncompressed_bpg_rate - IRIS_DOWN(3 * bpp)))
		return bpg_offset;
	else
		return (uncompressed_bpg_rate - IRIS_DOWN(3 * bpp));
}

static int32_t _iris_dsc_get_rc_table_index(struct drm_dsc_config *dsc)
{
	int32_t dsc_type = IRIS_DSC_V12_444_10BPC_6_5BPP;

	if (dsc->dsc_version_minor == 0x2) {
		if (dsc->native_422) {
			if (dsc->bits_per_component == 10 && dsc->bits_per_pixel == 160) {
				dsc_type = IRIS_DSC_V12_422_10BPC_10BPP;
			} else if (dsc->bits_per_component == 10 && dsc->bits_per_pixel == 128) {
				dsc_type = IRIS_DSC_V12_422_10BPC_8BPP;
			} else if (dsc->bits_per_component == 10 && dsc->bits_per_pixel == 104) {
				dsc_type = IRIS_DSC_V12_422_10BPC_6_5BPP;
			} else if (dsc->bits_per_component == 8 && dsc->bits_per_pixel == 128) {
				dsc_type = IRIS_DSC_V12_422_8BPC_8BPP;
			} else if (dsc->bits_per_component == 8 && dsc->bits_per_pixel == 104) {
				dsc_type = IRIS_DSC_V12_422_8BPC_6_5BPP;
			} else {
				IRIS_LOGE("%s(), don't support bpc = %d bpp = %d for DSC1.2 native_422",
						__func__, dsc->bits_per_component, dsc->bits_per_pixel);
			}

			return dsc_type;
		}

		if (dsc->bits_per_component == 10 && dsc->bits_per_pixel == 160) {
			dsc_type = IRIS_DSC_V12_444_10BPC_10BPP;
		} else if (dsc->bits_per_component == 10 && dsc->bits_per_pixel == 128) {
			dsc_type = IRIS_DSC_V12_444_10BPC_8BPP;
		} else if (dsc->bits_per_component == 10 && dsc->bits_per_pixel == 104) {
			dsc_type = IRIS_DSC_V12_444_10BPC_6_5BPP;
		} else if (dsc->bits_per_component == 8 && dsc->bits_per_pixel == 128) {
			dsc_type = IRIS_DSC_V12_444_8BPC_8BPP;
		} else if (dsc->bits_per_component == 8 && dsc->bits_per_pixel == 104) {
			dsc_type = IRIS_DSC_V12_444_8BPC_6_5BPP;
		} else {
			IRIS_LOGE("%s(), don't support bpc = %d bpp = %d for DSC1.2",
					__func__, dsc->bits_per_component, dsc->bits_per_pixel);
		}

		return dsc_type;
	}

	if (dsc->dsc_version_minor == 0x1) {
		if (dsc->bits_per_component == 10 && dsc->bits_per_pixel == 160) {
			dsc_type = IRIS_DSC_V11_444_10BPC_10BPP;
		} else if (dsc->bits_per_component == 10 && dsc->bits_per_pixel == 128) {
			dsc_type = IRIS_DSC_V11_444_10BPC_8BPP;
		} else if (dsc->bits_per_component == 10 && dsc->bits_per_pixel == 104) {
			dsc_type = IRIS_DSC_V11_444_10BPC_6_5BPP;
		} else if (dsc->bits_per_component == 8 && dsc->bits_per_pixel == 128) {
			dsc_type = IRIS_DSC_V11_444_8BPC_8BPP;
		} else if (dsc->bits_per_component == 8 && dsc->bits_per_pixel == 104) {
			dsc_type = IRIS_DSC_V11_444_8BPC_6_5BPP;
		} else {
			IRIS_LOGE("%s(), don't support bpc = %d bpp = %d for DSC1.1",
					__func__, dsc->bits_per_component, dsc->bits_per_pixel);
		}

		return dsc_type;
	}

	return dsc_type;
}

static void _iris_dsc_populate_dsc_config(struct drm_dsc_config *dsc)
{
	int bpp, bpc;
	int groups_per_line, groups_total;
	int min_rate_buffer_size;
	int hrd_delay;
	int pre_num_extra_mux_bits, num_extra_mux_bits;
	int slice_bits;
	int data;
	int final_value, final_scale;
	int i, ratio_idx;
	int scr_ver = 0;
	int slice_width_mod;

	if (dsc == NULL)
		return;

	SDE_ATRACE_BEGIN(__func__);
	dsc->rc_model_size = 8192;

	if ((dsc->dsc_version_major == 0x1) &&
			(dsc->dsc_version_minor == 0x1)) {
		if (scr_ver == 0x1)
			dsc->first_line_bpg_offset = 15;
		else
			dsc->first_line_bpg_offset = 12;
	} else if (dsc->dsc_version_minor == 0x2) {
		dsc->first_line_bpg_offset = _iris_dsc_get_v1_2_bpg_offset(dsc);
	}

	dsc->rc_edge_factor = 6;
	dsc->rc_tgt_offset_high = 3;
	dsc->rc_tgt_offset_low = 3;
	dsc->simple_422 = 0;
	dsc->convert_rgb = 0;
	dsc->vbr_enable = 0;

	bpp = IRIS_BPP(dsc->bits_per_pixel);
	bpc = dsc->bits_per_component;

	ratio_idx = _iris_dsc_get_rc_table_index(dsc);

	for (i = 0; i < DSC_NUM_BUF_RANGES - 1; i++)
		dsc->rc_buf_thresh[i] = iris_dsc_rc_buf_thresh[i];

	for (i = 0; i < DSC_NUM_BUF_RANGES; i++) {
		dsc->rc_range_params[i].range_min_qp =
			iris_dsc_rc_range_min_qp[ratio_idx][i];
		dsc->rc_range_params[i].range_max_qp =
			iris_dsc_rc_range_max_qp[ratio_idx][i];
		dsc->rc_range_params[i].range_bpg_offset =
			iris_dsc_rc_range_bpg[ratio_idx][i];
	}

	if (bpp == 80) {
		dsc->initial_offset = 6144;
		dsc->initial_xmit_delay = 512;
	} else if (bpp == 100) {
		dsc->initial_offset = 5632;
		dsc->initial_xmit_delay = 410;
	} else if (bpp == 65) {
		dsc->initial_offset = 6144;
		dsc->initial_xmit_delay = 630;
	} else if (bpp == 60) {
		dsc->initial_offset = 6144;
		dsc->initial_xmit_delay = 683;
	} else {
		dsc->initial_offset = 2048;
		dsc->initial_xmit_delay = 341;
	}

	if (dsc->native_422) {
		if (bpp == 65)
			dsc->initial_offset = 5632;
		else
			dsc->initial_offset = 2048;
		dsc->initial_xmit_delay /= 2;
	}

	slice_width_mod = dsc->slice_width;
	if (dsc->native_422 || dsc->native_420) {
		slice_width_mod = dsc->slice_width / 2;
		bpp = bpp * 2;
	}

	dsc->line_buf_depth = bpc + 1;

	if (bpc == 8) {
		dsc->flatness_min_qp = 3;
		dsc->flatness_max_qp = 12;
		dsc->rc_quant_incr_limit0 = 11;
		dsc->rc_quant_incr_limit1 = 11;
		dsc->mux_word_size = DSC_MUX_WORD_SIZE_8_10_BPC;
	} else if (bpc == 10) { /* 10bpc */
		dsc->flatness_min_qp = 7;
		dsc->flatness_max_qp = 16;
		dsc->rc_quant_incr_limit0 = 15;
		dsc->rc_quant_incr_limit1 = 15;
		dsc->mux_word_size = DSC_MUX_WORD_SIZE_8_10_BPC;
	} else { /* 12 bpc */
		dsc->flatness_min_qp = 11;
		dsc->flatness_max_qp = 20;
		dsc->rc_quant_incr_limit0 = 19;
		dsc->rc_quant_incr_limit1 = 19;
		dsc->mux_word_size = DSC_MUX_WORD_SIZE_12_BPC;
	}
	if ((dsc->dsc_version_minor == 0x2) && (dsc->native_420)) {
		dsc->second_line_bpg_offset = 12;
		dsc->second_line_offset_adj = 512;
		dsc->nsl_bpg_offset = 2048 *
			(DIV_ROUND_UP(dsc->second_line_bpg_offset,
				(dsc->slice_height - 1)));
	}

	groups_per_line = DIV_ROUND_UP(slice_width_mod, 3);

	dsc->slice_chunk_size = IRIS_DOWN(slice_width_mod * bpp) / 8;
	if (IRIS_DOWN(slice_width_mod * bpp) % 8)
		dsc->slice_chunk_size++;

	/* rbs-min */
	min_rate_buffer_size =  dsc->rc_model_size - dsc->initial_offset +
			IRIS_DOWN(dsc->initial_xmit_delay * bpp) +
			groups_per_line * dsc->first_line_bpg_offset;

	hrd_delay = DIV_ROUND_UP(IRIS_UP(min_rate_buffer_size), bpp);

	dsc->initial_dec_delay = hrd_delay - dsc->initial_xmit_delay;

	dsc->initial_scale_value = 8 * dsc->rc_model_size /
			(dsc->rc_model_size - dsc->initial_offset);

	slice_bits = 8 * dsc->slice_chunk_size * dsc->slice_height;

	groups_total = groups_per_line * dsc->slice_height;

	data = dsc->first_line_bpg_offset * 2048;

	dsc->nfl_bpg_offset = DIV_ROUND_UP(data, (dsc->slice_height - 1));

	if (dsc->native_422)
		pre_num_extra_mux_bits = 4 * dsc->mux_word_size + (4 * bpc + 4) + (3 * 4 * bpc) - 2;
	else if (dsc->native_420)
		pre_num_extra_mux_bits = 3 * dsc->mux_word_size + (4 * bpc + 4) + (2 * 4 * bpc) - 2;
	else
		pre_num_extra_mux_bits = 3 * (dsc->mux_word_size + (4 * bpc + 4) - 2);

	num_extra_mux_bits = pre_num_extra_mux_bits - (dsc->mux_word_size -
		((slice_bits - pre_num_extra_mux_bits) % dsc->mux_word_size));

	data = 2048 * (dsc->rc_model_size - dsc->initial_offset
		+ num_extra_mux_bits);
	dsc->slice_bpg_offset = DIV_ROUND_UP(data, groups_total);

	data = IRIS_DOWN(dsc->initial_xmit_delay * bpp);
	final_value =  dsc->rc_model_size - data + num_extra_mux_bits;

	final_scale = 8 * dsc->rc_model_size /
		(dsc->rc_model_size - final_value);

	dsc->final_offset = final_value;

	data = (final_scale - 9) * (dsc->nfl_bpg_offset +
		dsc->slice_bpg_offset);
	dsc->scale_increment_interval = (2048 * dsc->final_offset) / data;

	dsc->scale_decrement_interval = groups_per_line /
		(dsc->initial_scale_value - 8);
	SDE_ATRACE_END(__func__);
}

static void _iris_dsc_populate_private_params(
		struct msm_display_dsc_info *dsc_info, int intf_width)
{
	int mod_offset;
	int slice_per_pkt, slice_per_intf;
	int bytes_in_slice, total_bytes_per_intf;
	uint16_t bpp;
	uint32_t bytes_in_dsc_pair;
	uint32_t total_bytes_in_dsc_pair;

	if (!dsc_info || !dsc_info->config.slice_width ||
			!dsc_info->config.slice_height ||
			intf_width < dsc_info->config.slice_width) {
		IRIS_LOGE("%s(), invalid input, intf_width = %d slice_width = %d",
				__func__,
				intf_width, dsc_info ? dsc_info->config.slice_width : -1);
		return;
	}

	SDE_ATRACE_BEGIN(__func__);
	mod_offset = dsc_info->config.slice_width % 3;

	switch (mod_offset) {
	case 0:
		dsc_info->slice_last_group_size = 2;
		break;
	case 1:
		dsc_info->slice_last_group_size = 0;
		break;
	case 2:
		dsc_info->slice_last_group_size = 1;
		break;
	default:
		break;
	}

	dsc_info->det_thresh_flatness =
		2 << (dsc_info->config.bits_per_component - 8);

	slice_per_pkt = dsc_info->slice_per_pkt;
	slice_per_intf = DIV_ROUND_UP(intf_width,
			dsc_info->config.slice_width);

	/*
	 * If slice_per_pkt is greater than slice_per_intf then default to 1.
	 * This can happen during partial update.
	 */
	if (slice_per_pkt > slice_per_intf)
		slice_per_pkt = 1;

	bpp = IRIS_BPP(dsc_info->config.bits_per_pixel);
	bytes_in_slice = DIV_ROUND_UP(
			IRIS_DOWN(dsc_info->config.slice_width * bpp), 8);
	total_bytes_per_intf = bytes_in_slice * slice_per_intf;

	dsc_info->eol_byte_num = total_bytes_per_intf % 3;
	dsc_info->pclk_per_line =  DIV_ROUND_UP(total_bytes_per_intf, 3);
	dsc_info->bytes_in_slice = bytes_in_slice;
	dsc_info->bytes_per_pkt = bytes_in_slice * slice_per_pkt;
	dsc_info->pkt_per_line = slice_per_intf / slice_per_pkt;

	bytes_in_dsc_pair = DIV_ROUND_UP(bytes_in_slice * 2, 3);
	if (bytes_in_dsc_pair % 8) {
		dsc_info->dsc_4hsmerge_padding = 8 - (bytes_in_dsc_pair % 8);
		total_bytes_in_dsc_pair = bytes_in_dsc_pair +
				dsc_info->dsc_4hsmerge_padding;
		if (total_bytes_in_dsc_pair % 16)
			dsc_info->dsc_4hsmerge_alignment = 16 -
					(total_bytes_in_dsc_pair % 16);
	}
	SDE_ATRACE_END(__func__);
}

static void _iris_dsc_create_pps_buf_cmd(struct msm_display_dsc_info *dsc_info,
		char *buf, int pps_id, u32 len)
{
	struct drm_dsc_config *dsc = &dsc_info->config;
	char *bp = buf;
	char data;
	u32 i, bpp;

	SDE_ATRACE_BEGIN(__func__);
	memset(buf, 0, len);
	/* pps0 */
	*bp++ = (dsc->dsc_version_minor |
			dsc->dsc_version_major << 4);
	*bp++ = (pps_id & 0xff);		/* pps1 */
	bp++;					/* pps2, reserved */

	data = dsc->line_buf_depth & 0x0f;
	data |= ((dsc->bits_per_component & 0xf) << DSC_PPS_BPC_SHIFT);
	*bp++ = data;				/* pps3 */

	bpp = dsc->bits_per_pixel;
	if (dsc->native_422 || dsc->native_420)
		bpp = 2 * bpp;
	data = (bpp >> DSC_PPS_MSB_SHIFT);
	data &= 0x03;				/* upper two bits */
	data |= ((dsc->block_pred_enable & 0x1) << 5);
	data |= ((dsc->convert_rgb & 0x1) << 4);
	data |= ((dsc->simple_422 & 0x1) << 3);
	data |= ((dsc->vbr_enable & 0x1) << 2);
	*bp++ = data;				/* pps4 */
	*bp++ = (bpp & DSC_PPS_LSB_MASK);	/* pps5 */

	*bp++ = ((dsc->pic_height >> 8) & 0xff); /* pps6 */
	*bp++ = (dsc->pic_height & 0x0ff);	/* pps7 */
	*bp++ = ((dsc->pic_width >> 8) & 0xff);	/* pps8 */
	*bp++ = (dsc->pic_width & 0x0ff);	/* pps9 */

	*bp++ = ((dsc->slice_height >> 8) & 0xff);/* pps10 */
	*bp++ = (dsc->slice_height & 0x0ff);	/* pps11 */
	*bp++ = ((dsc->slice_width >> 8) & 0xff); /* pps12 */
	*bp++ = (dsc->slice_width & 0x0ff);	/* pps13 */

	*bp++ = ((dsc->slice_chunk_size >> 8) & 0xff);/* pps14 */
	*bp++ = (dsc->slice_chunk_size & 0x0ff);	/* pps15 */

	*bp++ = (dsc->initial_xmit_delay >> 8) & 0x3; /* pps16 */
	*bp++ = (dsc->initial_xmit_delay & 0xff);/* pps17 */

	*bp++ = ((dsc->initial_dec_delay >> 8) & 0xff); /* pps18 */
	*bp++ = (dsc->initial_dec_delay & 0xff);/* pps19 */

	bp++;				/* pps20, reserved */

	*bp++ = (dsc->initial_scale_value & 0x3f); /* pps21 */

	*bp++ = ((dsc->scale_increment_interval >> 8) & 0xff); /* pps22 */
	*bp++ = (dsc->scale_increment_interval & 0xff); /* pps23 */

	*bp++ = ((dsc->scale_decrement_interval >> 8) & 0xf); /* pps24 */
	*bp++ = (dsc->scale_decrement_interval & 0x0ff);/* pps25 */

	bp++;					/* pps26, reserved */

	*bp++ = (dsc->first_line_bpg_offset & 0x1f);/* pps27 */

	*bp++ = ((dsc->nfl_bpg_offset >> 8) & 0xff);/* pps28 */
	*bp++ = (dsc->nfl_bpg_offset & 0x0ff);	/* pps29 */
	*bp++ = ((dsc->slice_bpg_offset >> 8) & 0xff);/* pps30 */
	*bp++ = (dsc->slice_bpg_offset & 0x0ff);/* pps31 */

	*bp++ = ((dsc->initial_offset >> 8) & 0xff);/* pps32 */
	*bp++ = (dsc->initial_offset & 0x0ff);	/* pps33 */

	*bp++ = ((dsc->final_offset >> 8) & 0xff);/* pps34 */
	*bp++ = (dsc->final_offset & 0x0ff);	/* pps35 */

	*bp++ = (dsc->flatness_min_qp & 0x1f);	/* pps36 */
	*bp++ = (dsc->flatness_max_qp & 0x1f);	/* pps37 */

	*bp++ = ((dsc->rc_model_size >> 8) & 0xff);/* pps38 */
	*bp++ = (dsc->rc_model_size & 0x0ff);	/* pps39 */

	*bp++ = (dsc->rc_edge_factor & 0x0f);	/* pps40 */

	*bp++ = (dsc->rc_quant_incr_limit0 & 0x1f);	/* pps41 */
	*bp++ = (dsc->rc_quant_incr_limit1 & 0x1f);	/* pps42 */

	data = ((dsc->rc_tgt_offset_high & 0xf) << 4);
	data |= (dsc->rc_tgt_offset_low & 0x0f);
	*bp++ = data;				/* pps43 */

	for (i = 0; i < DSC_NUM_BUF_RANGES - 1; i++)
		*bp++ = (dsc->rc_buf_thresh[i] & 0xff); /* pps44 - pps57 */

	for (i = 0; i < DSC_NUM_BUF_RANGES; i++) {
		/* pps58 - pps87 */
		data = (dsc->rc_range_params[i].range_min_qp & 0x1f);
		data <<= 3;
		data |= ((dsc->rc_range_params[i].range_max_qp >> 2) & 0x07);
		*bp++ = data;
		data = (dsc->rc_range_params[i].range_max_qp & 0x03);
		data <<= 6;
		data |= (dsc->rc_range_params[i].range_bpg_offset & 0x3f);
		*bp++ = data;
	}

	if (dsc->dsc_version_minor == 0x2) {
		if (dsc->native_422)
			data = BIT(0);
		else if (dsc->native_420)
			data = BIT(1);
		else
			data = 0;
		*bp++ = data;				/* pps88 */
		*bp++ = dsc->second_line_bpg_offset;	/* pps89 */

		*bp++ = ((dsc->nsl_bpg_offset >> 8) & 0xff);/* pps90 */
		*bp++ = (dsc->nsl_bpg_offset & 0x0ff);	/* pps91 */

		*bp++ = ((dsc->second_line_offset_adj >> 8) & 0xff); /* pps92*/
		*bp++ = (dsc->second_line_offset_adj & 0x0ff);	/* pps93 */

		/* rest bytes are reserved and set to 0 */
	}
	SDE_ATRACE_END(__func__);
}

static void _iris_dsc_generate_pps(struct iris_memc_dsc_info *iris_dsc,
		uint32_t pps_sel)
{
	uint8_t pps_buf[PPS_SIZE];
	uint32_t pps_reg[IRIS_PPS_REG_COUNT];
	struct msm_display_dsc_info dsc;
	int32_t intf_width = 0;
	int32_t i = 0;
	int32_t j = 0;

	if (iris_dsc == NULL)
		return;

	SDE_ATRACE_BEGIN(__func__);
	dsc.config.dsc_version_major = 0x1;
	dsc.scr_rev = 0x0;

	dsc.config.pic_width = iris_dsc->pic_width;
	dsc.config.pic_height = iris_dsc->pic_height;
	dsc.slice_per_pkt = iris_dsc->slice_per_pkt;
	dsc.config.slice_width = iris_dsc->slice_width;
	dsc.config.slice_height = iris_dsc->slice_height;
	dsc.config.bits_per_component = iris_dsc->bits_per_component;
	dsc.config.bits_per_pixel = iris_dsc->bits_per_pixel;
	dsc.config.dsc_version_minor = iris_dsc->version_minor;
	dsc.chroma_format = iris_dsc->chroma_format;

	if (dsc.chroma_format == MSM_CHROMA_422) {
		dsc.config.native_422 = true;
		dsc.config.simple_422 = true;
	} else if (dsc.chroma_format == MSM_CHROMA_420) {
		dsc.config.native_420 = true;
	}

	dsc.source_color_space = MSM_YUV; //MSM_RGB;
	dsc.config.block_pred_enable = true; //false;

	intf_width = dsc.config.pic_width;
	dsc.config.slice_count = DIV_ROUND_UP(intf_width, dsc.config.slice_width);

	_iris_dsc_populate_dsc_config(&dsc.config);
	_iris_dsc_populate_private_params(&dsc, intf_width);
	_iris_dsc_create_pps_buf_cmd(&dsc, pps_buf, 0, PPS_SIZE);

	if (IRIS_IF_LOGD()) {
		// dump pps
		for (i = 0; i < IRIS_PPS_REG_COUNT * 4; i++)
			IRIS_LOGD("%s(), pps%d = %02x", __func__, i, pps_buf[i]);
	}

	for (i = 0; i < IRIS_PPS_REG_COUNT * 4; i += 4) {
		pps_reg[j++] = pps_buf[i] |
			(pps_buf[i + 1] << 8) |
			(pps_buf[i + 2] << 16) |
			(pps_buf[i + 3] << 24);
		IRIS_LOGD("%s(), pps reg: %08x", __func__, pps_reg[j-1]);
	}

	_iris_dsc_replace_pps(pps_reg, pps_sel);
	SDE_ATRACE_END(__func__);
}

static bool _iris_dsc_need_change(uint32_t pps_sel)
{
	struct iris_memc_dsc_info *cur_info = &iris_dsc_info[pps_sel];
	struct iris_memc_dsc_info *last_info = &iris_dsc_last_info[pps_sel];

	if (pps_sel >= DSC_PPS_SET_CNT)
		return false;

	if (cur_info->pic_width == last_info->pic_width &&
			cur_info->pic_height == last_info->pic_height &&
			cur_info->slice_per_pkt == last_info->slice_per_pkt &&
			cur_info->slice_width == last_info->slice_width &&
			cur_info->slice_height == last_info->slice_height &&
			cur_info->bits_per_component == last_info->bits_per_component &&
			cur_info->bits_per_pixel == last_info->bits_per_pixel &&
			cur_info->version_minor == last_info->version_minor &&
			cur_info->chroma_format == last_info->chroma_format &&
			cur_info->hw_slice_prefer == last_info->hw_slice_prefer)
		return false;

	memcpy(&iris_dsc_last_info[pps_sel], &iris_dsc_info[pps_sel],
			sizeof(struct iris_memc_dsc_info));

	return true;
}

static void _iris_dsc_change(uint32_t pps_sel)
{
	ktime_t ktime = ktime_get();

	if (!_iris_dsc_need_change(pps_sel)) {
		/* if DSC path switched, reconfig data format and bitwidth in pwil */
		if (iris_dsc_active_path != pps_sel) {
			_iris_dsc_update_pwil(pps_sel, true);
			iris_dsc_active_path = pps_sel;

			IRIS_LOGI("%s(), path%u, reconfig data format, time cost %llu us.",
					__func__, pps_sel, ktime_to_us(ktime_get() - ktime));
		}

		return;
	}

	IRIS_LOGI("%s(), pc: %#08x, sl: %u, %#08x, bpc: %d, bpp: %d, ver: %d, fmt: %d, hw slice: %s",
			__func__,
			PROC_INFO(iris_dsc_info[pps_sel].pic_width,
				iris_dsc_info[pps_sel].pic_height),
			iris_dsc_info[pps_sel].slice_per_pkt,
			PROC_INFO(iris_dsc_info[pps_sel].slice_width,
				iris_dsc_info[pps_sel].slice_height),
			iris_dsc_info[pps_sel].bits_per_component,
			iris_dsc_info[pps_sel].bits_per_pixel,
			iris_dsc_info[pps_sel].version_minor,
			iris_dsc_info[pps_sel].chroma_format,
			iris_dsc_info[pps_sel].hw_slice_prefer ? "true" : "false");

	SDE_ATRACE_BEGIN(__func__);
	_iris_dsc_generate_pps(&iris_dsc_info[pps_sel], pps_sel);
	_iris_dsc_update_param(pps_sel);
	iris_dma_trig(DMA_CH15, 0);
	iris_update_pq_opt(PATH_DSI, true);
	iris_dsc_active_path = pps_sel;
	SDE_ATRACE_END(__func__);

	IRIS_LOGI("%s(), path%u, time cost %llu us.", __func__,
			pps_sel, ktime_to_us(ktime_get() - ktime));
}

static void _iris_memc_dsc_change(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t pps_sel = DSC_PPS_SET0;

	iris_dsc_info[pps_sel].pic_width = pcfg->frc_setting.mv_hres;
	iris_dsc_info[pps_sel].pic_height = pcfg->frc_setting.mv_vres;
	iris_dsc_info[pps_sel].slice_width =
		iris_dsc_info[pps_sel].slice_per_pkt == 0 ?
		iris_dsc_info[pps_sel].pic_width :
		iris_dsc_info[pps_sel].pic_width / iris_dsc_info[pps_sel].slice_per_pkt;

	_iris_dsc_change(pps_sel);
}

void iris_rfb_helper_change(void)
{
	_iris_dsc_change(DSC_PPS_SET1);
}

static void _iris_memc_dsc_mv_config_specific(uint32_t count, uint32_t *values)
{
	uint32_t type = 0;
	uint32_t val = 0;
	uint32_t pps_sel = DSC_PPS_SET0;

	if (count != 2 && count != 3)
		return;

	type = values[0];
	val = values[1];
	if (count == 3)
		pps_sel = values[2];

	IRIS_LOGI("%s(), type: %u, value: %u, path: %u",
			__func__, type, val, pps_sel);

	switch (type) {
	case MEMC_DSC_VER:
		iris_dsc_info[pps_sel].version_minor = val;
		switch (val) {
		case 0x01: /* DSC1.1 can support YUV444 only */
			iris_dsc_info[pps_sel].chroma_format = MSM_CHROMA_444;
			break;
		case 0x02: /* DSC1.2 uses YUV422 by default */
			iris_dsc_info[pps_sel].chroma_format = MSM_CHROMA_422;
			break;
		default:
			IRIS_LOGE("%s(), invalid DSC version: %d", __func__, val);
			break;
		}
		break;
	case MEMC_DSC_BPC:
		iris_dsc_info[pps_sel].bits_per_component = val;
		break;
	case MEMC_DSC_BPP:
		iris_dsc_info[pps_sel].bits_per_pixel = val;
		break;
	case MEMC_DSC_SLICE_PER_PKT:
		iris_dsc_info[pps_sel].slice_per_pkt = val;
		break;
	case MEMC_DSC_SLICE_HEIGHT:
		iris_dsc_info[pps_sel].slice_height = val;
		break;
	case MEMC_DSC_FMT:
		iris_dsc_info[pps_sel].chroma_format = val;
		break;
	case MEMC_DSC_HW_SLICE_PREFER:
		iris_dsc_info[pps_sel].hw_slice_prefer = (val == 1);
		break;
	default:
		break;
	}
}

static void _iris_memc_dsc_mv_config_all(uint32_t count, uint32_t *values)
{
	uint32_t pps_sel = DSC_PPS_SET0;

	if (count != 6 && count != 7)
		return;

	if (count == 7)
		pps_sel = values[6];

	IRIS_LOGI("%s(), ver: 1.%u, bpc: %u, bpp: %u, slice per pkt: %u, slice height: %u, fmt: %u, pps: %u",
			__func__,
			values[0], values[1], values[2], values[3],
			values[4], values[5], pps_sel);

	iris_dsc_info[pps_sel].version_minor = values[0];
	iris_dsc_info[pps_sel].bits_per_component = values[1];
	iris_dsc_info[pps_sel].bits_per_pixel = values[2];
	iris_dsc_info[pps_sel].slice_per_pkt = values[3];
	iris_dsc_info[pps_sel].slice_height = values[4];
	iris_dsc_info[pps_sel].chroma_format = values[5];
}

static void _iris_memc_dsc_dynamic_switch_pps(uint32_t count, uint32_t *values)
{
	uint32_t pps_sel = DSC_PPS_SET0;

	if (count < 8)
		return;

	IRIS_LOGI("%s()", __func__);

	iris_dsc_info[pps_sel].pic_width = values[0];
	iris_dsc_info[pps_sel].pic_height = values[1];
	iris_dsc_info[pps_sel].slice_per_pkt = values[2];
	iris_dsc_info[pps_sel].slice_width = values[3];
	iris_dsc_info[pps_sel].slice_height = values[4];
	iris_dsc_info[pps_sel].bits_per_component = values[5];
	iris_dsc_info[pps_sel].bits_per_pixel = values[6];
	iris_dsc_info[pps_sel].version_minor = 0x02;
	iris_dsc_info[pps_sel].chroma_format = MSM_CHROMA_444;

	if (count > 7 && (values[7] == 0x01 || values[7] == 0x02))
		iris_dsc_info[pps_sel].version_minor = values[7];

	if (count > 8 &&
			(values[8] >= MSM_CHROMA_444 && values[8] <= MSM_CHROMA_420))
		iris_dsc_info[pps_sel].chroma_format = values[8];

	if (count > 9)
		pps_sel = values[9];

	_iris_dsc_generate_pps(&iris_dsc_info[pps_sel], pps_sel);
	_iris_dsc_update_param(pps_sel);

	iris_dma_trig(DMA_CH15, 0);
	iris_update_pq_opt(PATH_DSI, true);
}

void iris_memc_dsc_config(uint32_t count, uint32_t *values)
{
	IRIS_LOGI("%s(), count: %u", __func__, count);

	if (count < 2 || values == NULL) {
		IRIS_LOGE("%s(), invalid parameter!", __func__);
		return;
	}

	if (IRIS_IF_LOGD()) {
		int32_t i = 0;

		for (i = 0; i < count; i++)
			IRIS_LOGI("%s(), value[%d] = %u", __func__, i, values[i]);
	}

	_iris_memc_dsc_mv_config_specific(count, values);
	_iris_memc_dsc_mv_config_all(count, values);
	_iris_memc_dsc_dynamic_switch_pps(count, values);
}

struct iris_memc_dsc_info *iris_memc_dsc_info(uint32_t pps_sel)
{
	if (pps_sel >= DSC_PPS_SET_CNT)
		return NULL;

	return &iris_dsc_info[pps_sel];
}


enum {
	SCL_1D_ONLY,
	SCL_2D_ONLY,
	SCL_CNN_ONLY,
	SCL_CNN_1D,
	SCL_CNN_2D,
	SCL_STRATEGY_CNT
};

#define SRCNN_CASE(case)[case] = #case
static const char * const srcnn_case_name[] = {
	SRCNN_CASE(SCL_1D_ONLY),
	SRCNN_CASE(SCL_2D_ONLY),
	SRCNN_CASE(SCL_CNN_ONLY),
	SRCNN_CASE(SCL_CNN_1D),
	SRCNN_CASE(SCL_CNN_2D),
};
#undef SRCNN_CASE

enum {
	SCL_DATA_PATH0,
	SCL_DATA_PATH1,
	SCL_DATA_PATH_CNT
};

enum {
	SRCNN_MODE_AUTO,
	SRCNN_MODE_DISABLE,
	SRCNN_MODE_ENABLE,
	SRCNN_MODE_CNT,
};

enum {
	IRIS_SCL_IN,
	IRIS_SCL_OUT,
	IRIS_SCL_INOUT,
	IRIS_SCL_TYPE_CNT
};

enum {
	CNN_LOWPOWER_MODEL = 0,
	CNN_NORMAL_MODEL1 = 1, /* Default for Right-Buffer */
	CNN_NORMAL_MODEL2 = 2, /* Default for Left-Buffer */
	CNN_NORMAL_MODEL3,
	CNN_NORMAL_MODEL4,
	CNN_NORMAL_MODEL5,
	CNN_NORMAL_MODEL6,
	CNN_NORMAL_MODEL7,
	/* ... */
	CNN_MODEL_CNT = 16
};

enum { /* scl specific type */
	MV_STRATEGY = 0,
	IOINC_TAP,
};

enum { /* ioinc tap */
	IOINC_TAG5 = 5,
	IOINC_TAG9 = 9,
};

enum { /* ioinc filter group */
	FILTER_SOFT = 0,
	FILTER_SHARP,
	FILTER_GROUP_CNT
};

enum { /* ioinc filter type */
	FILTER_HS_Y_LEVEL = 0,
	FILTER_HS_UV_LEVEL,
	FILTER_VS_Y_LEVEL,
	FILTER_VS_UV_LEVEL,
	FILTER_TYPE_CNT
};

enum {
	SCL_2D_GF = 0,
	SCL_2D_DETECT,
	SCL_2D_PEAKING,
	SCL_2D_DTI,
	SCL_2D_PQ_CNT
};

enum { /* SCL change type */
	SCL_NO_CHANGE = 0,
	SCL_SWITCH_ONLY,
	SCL_FULL_CHANGE,
	SCL_CHANGE_TYPE_CNT
};

static struct iris_scl_conf {
	bool ioinc_enable;
	int32_t ioinc_in_h;
	int32_t ioinc_in_v;
	int32_t ioinc_out_h;
	int32_t ioinc_out_v;
	uint32_t ioinc_tap;
	bool sr_enable;
	int32_t sr_in_h;
	int32_t sr_in_v;
	uint32_t sr_strategy;
} _iris_scl_conf[SCL_DATA_PATH_CNT];

static uint32_t iris_expected_ioinc_tap[SCL_DATA_PATH_CNT] = {IOINC_TAG5, IOINC_TAG5};
static uint32_t iris_expected_strategy[SCL_DATA_PATH_CNT] = {SCL_2D_ONLY, SCL_2D_ONLY};
static uint32_t iris_cnn_using_model = CNN_NORMAL_MODEL1; /* Default model */
static uint32_t iris_cnn_loaded_models[2] = {CNN_NORMAL_MODEL2, CNN_NORMAL_MODEL1};
static uint32_t iris_cnn_models[SCL_DATA_PATH_CNT] = {CNN_NORMAL_MODEL1, CNN_NORMAL_MODEL1};
static uint32_t iris_sr2d_using_level[SCL_2D_PQ_CNT] = {0, 0, 0, 0};
static uint32_t iris_sr2d_level[SCL_DATA_PATH_CNT][SCL_2D_PQ_CNT] = {{0, 0, 0, 0}, {0, 0, 0, 0}};
static bool iris_ptsr_1to1;


static void _iris_scl_parse_info(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_scl_conf *scl_conf = &_iris_scl_conf[SCL_DATA_PATH0];

	IRIS_LOGI("%s()", __func__);

	scl_conf->ioinc_enable = false;
	scl_conf->ioinc_in_h = pcfg->frc_setting.disp_hres;
	scl_conf->ioinc_in_v = pcfg->frc_setting.disp_vres;
	scl_conf->ioinc_out_h = 0;
	scl_conf->ioinc_out_v = 0;
	scl_conf->ioinc_tap = IOINC_TAG5;

	scl_conf->sr_enable = false;
	scl_conf->sr_in_h = 0;
	scl_conf->sr_in_v = 0;
	scl_conf->sr_strategy = SCL_2D_ONLY;

	memcpy(&_iris_scl_conf[SCL_DATA_PATH1], scl_conf,
			sizeof(struct iris_scl_conf));
}

static void _iris_scl_reset_datapath(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t *payload = NULL;

	IRIS_LOGI("%s()", __func__);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 4);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find pwil control.", __func__);
		return;
	}
	payload[0] = BITS_SET(payload[0], 2, 12, 0); /* CSC auto */
	payload[1] = BITS_SET(payload[1], 2, 7, 0);  /* SCL auto */
	payload[1] = BITS_SET(payload[1], 2, 9, 0);  /* SRCNN auto */

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x02, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find pwil graphic control.", __func__);
		return;
	}
	payload[5] = ((pcfg->frc_setting.disp_vres & BITS_MSK(16)) << 16) |
		(pcfg->frc_setting.disp_hres & BITS_MSK(16));
}

static void _iris_scl_reset_param(void)
{
	_iris_scl_conf[SCL_DATA_PATH0].ioinc_enable = false;
	_iris_scl_conf[SCL_DATA_PATH0].ioinc_out_h = 0;
	_iris_scl_conf[SCL_DATA_PATH0].ioinc_out_v = 0;
	_iris_scl_conf[SCL_DATA_PATH0].sr_enable = false;
	_iris_scl_conf[SCL_DATA_PATH0].sr_in_h = 0;
	_iris_scl_conf[SCL_DATA_PATH0].sr_in_v = 0;

	_iris_scl_conf[SCL_DATA_PATH1].ioinc_enable = false;
	_iris_scl_conf[SCL_DATA_PATH1].ioinc_out_h = 0;
	_iris_scl_conf[SCL_DATA_PATH1].ioinc_out_v = 0;
	_iris_scl_conf[SCL_DATA_PATH1].sr_enable = false;
	_iris_scl_conf[SCL_DATA_PATH1].sr_in_h = 0;
	_iris_scl_conf[SCL_DATA_PATH1].sr_in_v = 0;

	memset(iris_sr2d_using_level, 0, sizeof(uint32_t) * SCL_2D_PQ_CNT);
	memset(iris_sr2d_level, 0, sizeof(uint32_t) * SCL_DATA_PATH_CNT * SCL_2D_PQ_CNT);
	iris_expected_strategy[SCL_DATA_PATH0] = SCL_2D_ONLY;
	iris_cnn_using_model = CNN_NORMAL_MODEL1;
	iris_cnn_loaded_models[0] = CNN_NORMAL_MODEL2;
	iris_cnn_loaded_models[1] = CNN_NORMAL_MODEL1;
	iris_cnn_models[SCL_DATA_PATH0] = CNN_NORMAL_MODEL1;
	iris_cnn_models[SCL_DATA_PATH1] = CNN_NORMAL_MODEL1;
}

static void _iris_scl_init_param(void)
{
	IRIS_LOGI("%s()", __func__);

	_iris_scl_reset_datapath();
	_iris_scl_reset_param();
}

static void _iris_scl_setting_off(void)
{
	IRIS_LOGI("%s()", __func__);
}

static uint32_t _iris_ioinc_change_type(bool enable,
		int32_t in_h, int32_t in_v, int32_t out_h, int32_t out_v,
		uint32_t tap, uint32_t path_sel)
{
	struct iris_scl_conf *cur_conf = &_iris_scl_conf[path_sel];

	if (path_sel >= SCL_DATA_PATH_CNT)
		return SCL_NO_CHANGE;

	IRIS_LOGI("%s(), path%u, current: [%#08x->%#08x], %u, %s, set: [%#08x->%#08x], %u, %s",
			__func__, path_sel,
			PROC_INFO(cur_conf->ioinc_in_h, cur_conf->ioinc_in_v),
			PROC_INFO(cur_conf->ioinc_out_h, cur_conf->ioinc_out_v),
			cur_conf->ioinc_tap, cur_conf->ioinc_enable ? "true" : "false",
			PROC_INFO(in_h, in_v), PROC_INFO(out_h, out_v),
			tap, enable ? "true" : "false");

	if (cur_conf->ioinc_in_h == in_h && cur_conf->ioinc_in_v == in_v &&
			cur_conf->ioinc_out_h == out_h && cur_conf->ioinc_out_v == out_v &&
			cur_conf->ioinc_tap == tap) {
		if (cur_conf->ioinc_enable == enable)
			return SCL_NO_CHANGE;

		cur_conf->ioinc_enable = enable;

		return SCL_SWITCH_ONLY;
	}

	cur_conf->ioinc_enable = enable;
	cur_conf->ioinc_in_h = in_h;
	cur_conf->ioinc_in_v = in_v;
	cur_conf->ioinc_out_h = out_h;
	cur_conf->ioinc_out_v = out_v;
	cur_conf->ioinc_tap = tap;

	return SCL_FULL_CHANGE;
}

static uint32_t _iris_sr_change_type(bool enable,
		int32_t in_h, int32_t in_v, uint32_t strategy, uint32_t path_sel)
{
	struct iris_scl_conf *cur_conf = &_iris_scl_conf[path_sel];

	if (path_sel >= SCL_DATA_PATH_CNT)
		return SCL_NO_CHANGE;

	IRIS_LOGI("%s(), path%u, current: %#08x, %u, %s, set: %#08x, %u, %s",
			__func__, path_sel,
			PROC_INFO(cur_conf->sr_in_h, cur_conf->sr_in_v),
			cur_conf->sr_strategy, cur_conf->sr_enable ? "true" : "false",
			PROC_INFO(in_h, in_v), strategy, enable ? "true" : "false");

	if (cur_conf->sr_in_h == in_h && cur_conf->sr_in_v == in_v &&
			cur_conf->sr_strategy == strategy) {
		if (cur_conf->sr_enable == enable)
			return SCL_NO_CHANGE;

		cur_conf->sr_enable = enable;

		return SCL_SWITCH_ONLY;
	}

	cur_conf->sr_enable = enable;
	cur_conf->sr_in_h = in_h;
	cur_conf->sr_in_v = in_v;
	cur_conf->sr_strategy = strategy;

	return SCL_FULL_CHANGE;
}

static void _iris_ioinc_config_pwil(bool enable, int32_t proc_h, int32_t proc_v,
		uint32_t path_sel)
{
	uint32_t *payload = NULL;

	if (path_sel == SCL_DATA_PATH0) { // path0
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x03, 2);
		if (payload == NULL) {
			IRIS_LOGE("%s(), failed to find pwil video control.", __func__);
			return;
		}

		/* restore video path to MV res */
		if (!enable) {
			struct iris_cfg *pcfg = iris_get_cfg();

			proc_h = pcfg->frc_setting.mv_hres;
			proc_v = pcfg->frc_setting.mv_vres;
		}

		payload[5] = ((proc_v & BITS_MSK(16)) << 16) | (proc_h & BITS_MSK(16));
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x03, 0x03, 1);
	}

	if (path_sel == SCL_DATA_PATH1) { // path1
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x02, 2);
		if (payload == NULL) {
			IRIS_LOGE("%s(), failed to find pwil graphic control.", __func__);
			return;
		}

		/* restore graphic path to display res */
		if (!enable) {
			struct iris_cfg *pcfg = iris_get_cfg();

			proc_h = pcfg->frc_setting.disp_hres;
			proc_v = pcfg->frc_setting.disp_vres;
		}

		payload[5] = ((proc_v & BITS_MSK(16)) << 16) | (proc_h & BITS_MSK(16));
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x02, 0x02, 1);
	}
}

static int32_t _iris_calc_offset(int32_t in_hv, int32_t out_hv,
		int32_t offset, int32_t inc, int32_t frac,
		int32_t init_phas, int32_t total_phas)
{
	long long delt_acc = ((long long)((in_hv - offset) * total_phas -
				init_phas) << frac) - BIT_MSK(frac - 1);

	return out_hv - 1 - (int)((delt_acc + inc - 1) / inc);
}

static void _iris_ioinc_config(int32_t in_h, int32_t in_v,
		int32_t out_h, int32_t out_v, uint32_t tap_num, uint32_t path_sel)
{
	const int32_t frac = 13; // fraction
	const int32_t prec = tap_num == IOINC_TAG5 ? 19 : 18;
	const int32_t total_phas = tap_num == IOINC_TAG5 ? 64 : 32;

	uint32_t *payload = NULL;
	int32_t v_inc = (in_v << prec) / out_v;
	int32_t v_top_offs = (v_inc >> (prec + 1)) + (tap_num + 1) / 2;
	int32_t v_init_phas = (v_inc >> (frac + 1)) &
		(tap_num == IOINC_TAG5 ? BITS_MSK(6) : BITS_MSK(5));
	int32_t v_bot_offs = _iris_calc_offset(in_v, out_v, v_top_offs, v_inc,
			frac, v_init_phas, total_phas);
	int32_t v_max_lines = out_v;

	int32_t h_inc = (in_h << prec) / out_h;
	int32_t h_left_offs = (h_inc >> (prec + 1)) + (tap_num + 1) / 2;
	int32_t h_init_phas = (h_inc >> (frac + 1)) &
		(tap_num == IOINC_TAG5 ? BITS_MSK(6) : BITS_MSK(5));
	int32_t h_right_offs = _iris_calc_offset(in_h, out_h, h_left_offs, h_inc,
			frac, h_init_phas, total_phas);
	int32_t h_max_lines = out_h;
	uint8_t tap_sel = tap_num == IOINC_TAG5 ? 0 : 1;

	IRIS_LOGI("%s(), %#08x->%#08x, tap: %d, path: %u", __func__,
			PROC_INFO(in_h, in_v), PROC_INFO(out_h, out_v), tap_num, path_sel);

	payload = iris_get_ipopt_payload_data(IRIS_IP_IOINC1D, 0x00, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x.",
				__func__, IRIS_IP_IOINC1D, 0x00);
		return;
	}

	// enable
	payload[0] = BITS_SET(payload[0], 1, 0, 0);

	// tap sel
	payload[0] = BITS_SET(payload[0], 1, 18, tap_sel);
	payload[0] = BITS_SET(payload[0], 1, 19, tap_sel);

	payload[4] = BITS_SET(payload[4], 4, 12, 7);
	payload[13] = BITS_SET(payload[13], 4, 12, 7);

	if (path_sel == SCL_DATA_PATH0) { // path0
		payload[2] = v_inc;
		payload[4] = BITS_SET(payload[4], 6, 0, v_init_phas);
		payload[5] = BITS_SET(payload[5], 12, 0, v_max_lines);
		payload[6] = v_top_offs | (v_bot_offs << 16);

		payload[11] = h_inc;
		payload[13] = BITS_SET(payload[13], 6, 0, h_init_phas);
		payload[14] = BITS_SET(payload[14], 12, 0, h_max_lines);
		payload[15] = h_left_offs | (h_right_offs << 16);

		IRIS_LOGI("%s(), path0[inc, alg, max, offs], v[%#08x, %#08x, %#08x, %#08x], h[%#08x, %#08x, %#08x, %#08x]",
				__func__,
				payload[2], payload[4], payload[5], payload[6],
				payload[11], payload[13], payload[14], payload[15]);
	}

	if (path_sel == SCL_DATA_PATH1) { // path1
		payload[3] = v_inc;
		payload[4] = BITS_SET(payload[4], 6, 6, v_init_phas);
		payload[5] = BITS_SET(payload[5], 12, 16, v_max_lines);
		payload[7] = v_top_offs | (v_bot_offs << 16);

		payload[12] = h_inc;
		payload[13] = BITS_SET(payload[13], 6, 6, h_init_phas);
		payload[14] = BITS_SET(payload[14], 12, 16, h_max_lines);
		payload[16] = h_left_offs | (h_right_offs << 16);

		IRIS_LOGI("%s(), path1[inc, alg, max, offs], v[%#08x, %#08x, %#08x, %#08x], h[%#08x, %#08x, %#08x, %#08x]",
				__func__,
				payload[3], payload[4], payload[5], payload[7],
				payload[12], payload[13], payload[14], payload[16]);
	}

	iris_init_update_ipopt_t(IRIS_IP_IOINC1D, 0x00, 0x00, 1);
}

static bool _iris_ioinc_perform_config(bool enable, int32_t in_h, int32_t in_v,
		int32_t out_h, int32_t out_v, uint32_t path_sel)
{
	uint32_t change_type = SCL_NO_CHANGE;
	uint32_t tap_num = IOINC_TAG5;

	if (path_sel >= SCL_DATA_PATH_CNT)
		return false;

	tap_num = iris_expected_ioinc_tap[path_sel];
	change_type = _iris_ioinc_change_type(enable, in_h, in_v, out_h, out_v,
				tap_num, path_sel);

	IRIS_LOGI("%s(), path%u, change type: %u", __func__,
			path_sel, change_type);

	if (change_type == SCL_NO_CHANGE)
		return false;

	if (change_type == SCL_SWITCH_ONLY) {
		_iris_ioinc_config_pwil(enable, out_h, out_v, path_sel);
		return true;
	}

	_iris_ioinc_config(in_h, in_v, out_h, out_v, tap_num, path_sel);
	_iris_ioinc_config_pwil(enable, out_h, out_v, path_sel);

	return true;
}

static bool _iris_memc_ioinc_change(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	bool ioinc_changed = false;

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE) {
		ioinc_changed = _iris_ioinc_perform_config(true,
				pcfg->frc_setting.disp_hres, pcfg->frc_setting.disp_vres,
				pcfg->frc_setting.disp_hres, pcfg->frc_setting.disp_vres,
				SCL_DATA_PATH1);

		ioinc_changed |= _iris_ioinc_perform_config(true,
				pcfg->frc_setting.mv_hres, pcfg->frc_setting.mv_vres,
				pcfg->frc_setting.mv_hres, pcfg->frc_setting.mv_vres,
				SCL_DATA_PATH0);
	} else {
		ioinc_changed = _iris_ioinc_perform_config(true,
				pcfg->frc_setting.disp_hres, pcfg->frc_setting.disp_vres,
				pcfg->frc_setting.mv_hres, pcfg->frc_setting.mv_vres,
				SCL_DATA_PATH0);
	}

	IRIS_LOGI_IF(ioinc_changed, "%s(), MEMC mode: %d, [%#08x]->[%#08x]", __func__,
			pcfg->memc_info.memc_mode,
			PROC_INFO(pcfg->frc_setting.disp_hres, pcfg->frc_setting.disp_vres),
			PROC_INFO(pcfg->frc_setting.mv_hres, pcfg->frc_setting.mv_vres));

	return ioinc_changed;
}

static void _iris_ioinc_force_disable(void)
{
	uint32_t *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_IOINC1D, 0x00, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_IOINC1D, 0x00);
		return;
	}

	payload[0] = BITS_SET(payload[0], 1, 0, 1);

	iris_init_update_ipopt_t(IRIS_IP_IOINC1D, 0x00, 0x00, 1);
}

static void _iris_ioinc_disable(uint32_t path_sel)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	_iris_ioinc_force_disable();
	_iris_ioinc_config_pwil(false,
			pcfg->frc_setting.disp_hres, pcfg->frc_setting.disp_vres, path_sel);
	_iris_scl_conf[path_sel].ioinc_enable = false;
}

static void _iris_srcnn_ctrl(bool enable, uint32_t strategy, uint32_t path_sel)
{
	uint32_t *payload = NULL;
	uint32_t srcnn_ctrl = 0;
	uint8_t path_ctrl = 0;
	uint8_t start_pos = 0;

	if (strategy >= SCL_STRATEGY_CNT) {
		IRIS_LOGE("%s(), invalid strategy: %u", __func__, strategy);
		return;
	}

	payload = iris_get_ipopt_payload_data(IRIS_IP_SR, 0x01, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_SR, 0x01);
		return;
	}
	srcnn_ctrl = payload[0];

	if (path_sel == SCL_DATA_PATH1)
		start_pos += 8;

	srcnn_ctrl = BITS_SET(srcnn_ctrl, 8, start_pos, 0); // clear ctrl bits
	if (enable)
		path_ctrl = BIT_SET(path_ctrl, 0); // SRCNN_EN

	switch (strategy) {
	case SCL_1D_ONLY:
		path_ctrl = BIT_SET(path_ctrl, 1); // CNN_BYPASS
		path_ctrl = BIT_SET(path_ctrl, 3); // GF_BYPASS
		path_ctrl = BIT_SET(path_ctrl, 4); // BS_BYPASS
		path_ctrl = BIT_SET(path_ctrl, 6); // ENH_BYPASS
		path_ctrl = BIT_CLR(path_ctrl, 7); // DETAIL_FORCE
		break;
	case SCL_2D_ONLY:
		path_ctrl = BIT_SET(path_ctrl, 1); // CNN_BYPASS
		path_ctrl = BIT_SET(path_ctrl, 7); // DETAIL_FORCE
		break;
	case SCL_CNN_ONLY:
		path_ctrl = BIT_SET(path_ctrl, 2); // SCL_BYPASS
		path_ctrl = BIT_SET(path_ctrl, 6); // ENH_BYPASS
		path_ctrl = BIT_SET(path_ctrl, 7); // DETAIL_FORCE
		break;
	case SCL_CNN_1D:
		path_ctrl = BIT_SET(path_ctrl, 3); // GF_BYPASS
		path_ctrl = BIT_SET(path_ctrl, 4); // BS_BYPASS
		path_ctrl = BIT_SET(path_ctrl, 6); // ENH_BYPASS
		path_ctrl = BIT_CLR(path_ctrl, 7); // DETAIL_FORCE
		break;
	case SCL_CNN_2D:
		path_ctrl = BIT_SET(path_ctrl, 7); // DETAIL_FORCE
		break;
	default:
		break;
	}

	srcnn_ctrl = BITS_SET(srcnn_ctrl, 8, start_pos, path_ctrl);
	payload[0] = srcnn_ctrl;

	IRIS_LOGI("%s(), strategy: %s(%u), path: %u, control: %#08x", __func__,
			srcnn_case_name[strategy], strategy, path_sel, srcnn_ctrl);
}

static void _iris_srcnn_calc(int32_t in_h, int32_t in_v,
		int32_t out_h, int32_t out_v,
		bool enable_cnn, uint32_t path_sel)
{
	const int32_t frac = 13; // fraction
	const int32_t prec = 19;
	int32_t uv_inc_h = (in_h << prec) / out_h;
	int32_t uv_inc_v = (in_v << prec) / out_v;
	int32_t y_inc_h = uv_inc_h;
	int32_t y_inc_v = uv_inc_v;
	int32_t y_init_phas_h = 0;
	int32_t y_init_phas_v = 0;
	int32_t uv_init_phas_h = 0;
	int32_t uv_init_phas_v = 0;
	uint32_t *payload = NULL;

	if (enable_cnn) {
		y_inc_h <<= 1;
		y_inc_v <<= 1;
		if (in_h << 1 == out_h)
			y_inc_h = BIT_MSK(prec);
		if (in_v << 1 == out_v)
			y_inc_v = BIT_MSK(prec);
	}

	y_init_phas_h = (y_inc_h >> (frac + 1)) & BITS_MSK(6);
	y_init_phas_v = (y_inc_v >> (frac + 1)) & BITS_MSK(6);
	uv_init_phas_h = (uv_inc_h >> (frac + 1)) & BITS_MSK(6);
	uv_init_phas_v = (uv_inc_v >> (frac + 1)) & BITS_MSK(6);

	IRIS_LOGI("%s(), %#08x->%#08x, path: %u, y_h[%#08x, %#08x], y_v[%#08x, %#08x], uv_h[%#08x, %#08x], uv_v[%#08x, %#08x]",
			__func__,
			PROC_INFO(in_h, in_v), PROC_INFO(out_h, out_v), path_sel,
			y_inc_h, y_init_phas_h, y_inc_v, y_init_phas_v,
			uv_inc_h, uv_init_phas_h, uv_inc_v, uv_init_phas_v);

	payload = iris_get_ipopt_payload_data(IRIS_IP_SR, 0x01, 4);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_SR, 0x01);
		return;
	}

	if (path_sel == SCL_DATA_PATH0) { // path0
		payload[0] = BITS_SET(payload[0], 14, 0, out_v);
		payload[0] = BITS_SET(payload[0], 14, 16, out_h);

		payload[2] = BITS_SET(payload[2], 24, 0, y_inc_h);
		payload[2] = BITS_SET(payload[2], 6, 24, y_init_phas_h);
		payload[3] = BITS_SET(payload[3], 24, 0, y_inc_v);
		payload[3] = BITS_SET(payload[3], 6, 24, y_init_phas_v);

		payload[4] = BITS_SET(payload[4], 24, 0, uv_inc_h);
		payload[4] = BITS_SET(payload[4], 6, 24, uv_init_phas_h);
		payload[5] = BITS_SET(payload[5], 24, 0, uv_inc_v);
		payload[5] = BITS_SET(payload[5], 6, 24, uv_init_phas_v);
	}

	if (path_sel == SCL_DATA_PATH1) { // path1
		payload[1] = BITS_SET(payload[1], 14, 0, out_v);
		payload[1] = BITS_SET(payload[1], 14, 16, out_h);

		payload[6] = BITS_SET(payload[6], 24, 0, y_inc_h);
		payload[6] = BITS_SET(payload[6], 6, 24, y_init_phas_h);
		payload[7] = BITS_SET(payload[7], 24, 0, y_inc_v);
		payload[7] = BITS_SET(payload[7], 6, 24, y_init_phas_v);

		payload[8] = BITS_SET(payload[8], 24, 0, uv_inc_h);
		payload[8] = BITS_SET(payload[8], 6, 24, uv_init_phas_h);
		payload[9] = BITS_SET(payload[9], 24, 0, uv_inc_v);
		payload[9] = BITS_SET(payload[9], 6, 24, uv_init_phas_v);
	}
}

static bool _iris_srcnn_check_valid(int32_t proc_h, int32_t proc_v,
		uint32_t strategy)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int32_t ratio_h = 1;
	int32_t ratio_v = 1;

	if (proc_h == 0 || proc_v == 0 ||
			pcfg->frc_setting.disp_hres == 0 ||
			pcfg->frc_setting.disp_vres == 0) {
		IRIS_LOGE("%s(), invalid param: %#08x, %#08x", __func__,
				PROC_INFO(proc_h, proc_v),
				PROC_INFO(pcfg->frc_setting.disp_hres,
					pcfg->frc_setting.disp_vres));
		return false;
	}

	ratio_h = pcfg->frc_setting.disp_hres * 100 / proc_h;
	ratio_v = pcfg->frc_setting.disp_vres * 100 / proc_v;

	IRIS_LOGI("%s(), ratio: %dx%d for %s(%u)", __func__,
			ratio_h, ratio_v, srcnn_case_name[strategy], strategy);

	switch (strategy) {
	case SCL_1D_ONLY:
		if (ratio_h < 50 || ratio_h > 800 || ratio_v < 50 || ratio_v > 800) {
			IRIS_LOGW("%s(), out of range for %s(%u).",
					__func__, srcnn_case_name[strategy], strategy);
			return false;
		}
		break;
	case SCL_2D_ONLY:
		if (ratio_h < 100 || ratio_h > 400 || ratio_v < 100 || ratio_v > 400) {
			IRIS_LOGW("%s(), out of range for %s(%u).",
					__func__, srcnn_case_name[strategy], strategy);
			return false;
		}
		break;
	case SCL_CNN_ONLY:
		if (ratio_h != 200 || ratio_v != 200) {
			IRIS_LOGW("%s(), out of range for %s(%u), 2x only.",
					__func__, srcnn_case_name[strategy], strategy);
			return false;
		}
		break;
	case SCL_CNN_1D:
		if (ratio_h < 100 || ratio_h > 800 || ratio_v < 100 || ratio_v > 800) {
			IRIS_LOGW("%s(), out of range for %s(%u).",
					__func__, srcnn_case_name[strategy], strategy);
			return false;
		}
		break;
	case SCL_CNN_2D:
		if (ratio_h < 200 || ratio_h > 800 || ratio_v < 200 || ratio_v > 800) {
			IRIS_LOGW("%s(), out of range for %s(%u).",
					__func__, srcnn_case_name[strategy], strategy);
			return false;
		}
		break;
	default:
		break;
	}

	return true;
}

static void _iris_srcnn_config_pwil_csc(bool enable)
{
	uint32_t *payload = NULL;
	int32_t pwil_csc_mode = enable ? 2 : 0;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 4);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find pwil control.", __func__);
		return;
	}

	payload[0] = BITS_SET(payload[0], 2, 12, pwil_csc_mode);

	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x01, 0x01, 1);
}

static void _iris_srcnn_config_pwil(uint32_t sr_mode)
{
	uint32_t *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 5);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find pwil data path control.", __func__);
		return;
	}

	payload[0] = BITS_SET(payload[0], 2, 9, sr_mode);
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x01, 0x01, 1);
}

static void _iris_srcnn_proc(bool enable, int32_t proc_h, int32_t proc_v,
		uint32_t strategy, uint32_t path_sel)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	bool enable_cnn = false;
	uint32_t sr_mode = SRCNN_MODE_AUTO;

	if (path_sel >= SCL_DATA_PATH_CNT) {
		IRIS_LOGE("%s(), invalid data path: %u.", __func__, path_sel);
		return;
	}

	switch (strategy) {
	case SCL_CNN_ONLY:
	case SCL_CNN_1D:
	case SCL_CNN_2D:
		enable_cnn = true;
		break;
	default:
		break;
	}

	_iris_srcnn_ctrl(enable, strategy, path_sel);
	_iris_srcnn_calc(proc_h, proc_v,
			pcfg->frc_setting.disp_hres, pcfg->frc_setting.disp_vres,
			enable_cnn, path_sel);

	iris_init_update_ipopt_t(IRIS_IP_SR, 0x01, 0x01, 1);

	if (path_sel == SCL_DATA_PATH1) {
		if (strategy == SCL_2D_ONLY &&
				proc_h == pcfg->frc_setting.disp_hres &&
				proc_v == pcfg->frc_setting.disp_vres)
			iris_ptsr_1to1 = true;
		else
			iris_ptsr_1to1 = false;
	}

	if (enable && iris_ptsr_1to1)
		sr_mode = SRCNN_MODE_ENABLE;

	_iris_srcnn_config_pwil(sr_mode);
}

bool iris_scl_ptsr_1to1(void)
{
	return iris_ptsr_1to1;
}

static void _iris_srcnn_enable(bool enable, uint32_t path_sel)
{
	uint32_t *payload = NULL;
	uint8_t enable_pos = 0;
	uint32_t sr_mode = SRCNN_MODE_AUTO;
	uint8_t path_ctrl = enable ? 1 : 0;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SR, 0x01, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_SR, 0x01);
		return;
	}

	IRIS_LOGI("%s(), %s, path: %u", __func__,
			enable ? "on" : "off", path_sel);

	if (path_sel == SCL_DATA_PATH1)
		enable_pos += 8;

	/* SRCNN_EN */
	payload[0] = BITS_SET(payload[0], 1, enable_pos, path_ctrl);
	iris_init_update_ipopt_t(IRIS_IP_SR, 0x01, 0x01, 1);

	if (enable && iris_ptsr_1to1)
		sr_mode = SRCNN_MODE_ENABLE;

	_iris_srcnn_config_pwil(sr_mode);
}

static bool _iris_srcnn_perform_config(bool enable,
		int32_t proc_h, int32_t proc_v, uint32_t strategy, uint32_t path_sel)
{
	uint32_t change_type = SCL_NO_CHANGE;

	if (path_sel >= SCL_DATA_PATH_CNT)
		return false;

	change_type = _iris_sr_change_type(enable, proc_h, proc_v,
			strategy, path_sel);

	IRIS_LOGI("%s(), path%u, change type: %u", __func__,
			path_sel, change_type);

	if (change_type == SCL_NO_CHANGE)
		return false;

	if (change_type == SCL_SWITCH_ONLY) {
		_iris_srcnn_enable(enable, path_sel);
		return true;
	}

	_iris_srcnn_proc(enable, proc_h, proc_v, strategy, path_sel);

	return true;
}

static void _iris_srcnn_disable(void)
{
	uint32_t *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SR, 0x01, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find: %02x, %02x",
				__func__, IRIS_IP_SR, 0x01);
		return;
	}

	if (BITS_GET(payload[0], 1, 0) == 0 && BITS_GET(payload[0], 1, 8) == 0)
		return;

	_iris_scl_conf[SCL_DATA_PATH0].sr_enable = false;
	_iris_scl_conf[SCL_DATA_PATH1].sr_enable = false;
	payload[0] = BITS_SET(payload[0], 1, 0, 0);
	payload[0] = BITS_SET(payload[0], 1, 8, 0);
	iris_init_update_ipopt_t(IRIS_IP_SR, 0x01, 0x01, 1);

	_iris_srcnn_config_pwil(SRCNN_MODE_DISABLE);
}

static bool _iris_memc_srcnn_change(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	bool changed = false;

	changed = _iris_srcnn_perform_config(true,
			pcfg->frc_setting.mv_hres, pcfg->frc_setting.mv_vres,
			iris_expected_strategy[SCL_DATA_PATH0], SCL_DATA_PATH0);

	if (changed)
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x01, 0x01, 1);

	IRIS_LOGI_IF(changed, "%s(), sr: %#08x, strategy: %u", __func__,
			PROC_INFO(pcfg->frc_setting.mv_hres, pcfg->frc_setting.mv_vres),
			iris_expected_strategy[SCL_DATA_PATH0]);

	return changed;
}

static bool _iris_memc_mode(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
		return true;

	return false;
}

static void _iris_scl_triger_dma(int32_t event_channels)
{
	uint32_t event_source = DMA_EVENT_DTG_TE;

	IRIS_LOGD("%s(), event trigger: %#x", __func__, event_channels);

	SDE_ATRACE_BEGIN(__func__);
	if (event_channels == 0)
		goto commit;

	if (_iris_memc_mode())
		event_source = DMA_EVENT_DTG_EVS_PRE;

	iris_dma_gen_ctrl(event_channels, event_source, 0);

commit:
	iris_update_pq_opt(PATH_DSI, true);
	SDE_ATRACE_END(__func__);
}

static bool _iris_scl_update_pq(uint32_t type);
static void _iris_memc_scl_change(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	bool changed = false;
	ktime_t ktime = ktime_get();

	SDE_ATRACE_BEGIN(__func__);

retry:
	if (!_iris_srcnn_check_valid(
				pcfg->frc_setting.mv_hres, pcfg->frc_setting.mv_vres,
				iris_expected_strategy[SCL_DATA_PATH0])) {
		IRIS_LOGE("%s(), invalid input: %#08x, strategy: %u, path%u", __func__,
				PROC_INFO(pcfg->frc_setting.mv_hres, pcfg->frc_setting.mv_vres),
				iris_expected_strategy[SCL_DATA_PATH0], SCL_DATA_PATH0);

		if (iris_expected_strategy[SCL_DATA_PATH0] != SCL_2D_ONLY) {
			IRIS_LOGW("%s(), force MEMC uses 2d and retry.", __func__);

			iris_expected_strategy[SCL_DATA_PATH0] = SCL_2D_ONLY;
			goto retry;
		}
		SDE_ATRACE_END(__func__);

		return;
	}

	changed = _iris_memc_ioinc_change();
	changed |= _iris_memc_srcnn_change();
	changed |= _iris_scl_update_pq(SCL_DATA_PATH0);

	if (changed)
		_iris_scl_triger_dma(DMA_EVENT_CH8 | DMA_EVENT_CH9);

	SDE_ATRACE_END(__func__);

	IRIS_LOGI_IF(changed, "%s(), time cost %llu us.",
			__func__, ktime_to_us(ktime_get() - ktime));
}

static void _iris_scl_mv_strategy(uint32_t strategy)
{
	IRIS_LOGI("%s(), change strategy: %u->%u", __func__,
			_iris_scl_conf[SCL_DATA_PATH0].sr_strategy, strategy);

	iris_expected_strategy[SCL_DATA_PATH0] = strategy;
}

static void _iris_scl_ioinc_tap(uint32_t tap_num)
{
	IRIS_LOGI("%s(), change ioinc tap: %u->%u", __func__,
			_iris_scl_conf[SCL_DATA_PATH0].ioinc_tap, tap_num);

	iris_expected_ioinc_tap[SCL_DATA_PATH0] = tap_num;
}

static void _iris_scl_set_specific(uint32_t count, uint32_t *values)
{
	uint32_t type = MV_STRATEGY;

	if (count != 1)
		return;

	type = BITS_GET(values[0], 16, 16);

	switch (type) {
	case MV_STRATEGY:
		_iris_scl_mv_strategy(BITS_GET(values[0], 16, 0));
		break;
	case IOINC_TAP:
		_iris_scl_ioinc_tap(BITS_GET(values[0], 16, 0));
		break;
	default:
		break;
	}
}

static void _iris_scl_disable(uint32_t count, uint32_t *values,
		uint32_t scl_type)
{
	uint32_t path_sel = SCL_DATA_PATH1;

	if (count != 2 && count != 3)
		return;

	IRIS_LOGI("%s()", __func__);

	if (count == 3)
		path_sel = values[2];

	switch (scl_type) {
	case IRIS_SCL_IN:
		_iris_ioinc_disable(path_sel);
		IRIS_LOGI("%s(), disable IOINC1D.", __func__);
		break;
	case IRIS_SCL_OUT:
		_iris_srcnn_disable();
		_iris_srcnn_config_pwil_csc(false);
		IRIS_LOGI("%s(), disable SRCNN.", __func__);
		break;
	case IRIS_SCL_INOUT:
		_iris_srcnn_disable();
		_iris_ioinc_disable(path_sel);
		_iris_srcnn_config_pwil_csc(false);
		_iris_scl_reset_param();
		IRIS_LOGI("%s(), disable IOINC1D and SRCNN.", __func__);
		break;
	default:
		break;
	}

	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x01, 0x01, 1);
	_iris_scl_triger_dma(DMA_EVENT_CH8 | DMA_EVENT_CH9);
}

static bool _iris_scl_perform(bool enable, uint32_t scl_type,
		int32_t proc_h, int32_t proc_v, uint32_t strategy, uint32_t path_sel)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	bool changed = false;

	IRIS_LOGI("%s(), scl type: %u, proc: %#08x, strategy: %u, path: %u",
			__func__, scl_type, PROC_INFO(proc_h, proc_v), strategy, path_sel);

	switch (scl_type) {
	case IRIS_SCL_IN:
		changed = _iris_ioinc_perform_config(enable,
				pcfg->frc_setting.disp_hres, pcfg->frc_setting.disp_vres,
				proc_h, proc_v, path_sel);
		if (changed)
			_iris_srcnn_disable();
		break;
	case IRIS_SCL_OUT:
		if (!_iris_srcnn_check_valid(proc_h, proc_v, strategy)) {
			IRIS_LOGE("%s(), invalid input: %#08x, %u, for path%u", __func__,
					PROC_INFO(proc_h, proc_v), strategy, path_sel);
			return false;
		}
		changed = _iris_srcnn_perform_config(enable,
				proc_h, proc_v, strategy, path_sel);
		break;
	case IRIS_SCL_INOUT:
		if (!_iris_srcnn_check_valid(proc_h, proc_v, strategy)) {
			IRIS_LOGE("%s(), invalid input: %#08x, %u, for path%u", __func__,
					PROC_INFO(proc_h, proc_v), strategy, path_sel);
			return false;
		}
		changed = _iris_ioinc_perform_config(enable,
				pcfg->frc_setting.disp_hres, pcfg->frc_setting.disp_vres,
				proc_h, proc_v, path_sel);
		changed |= _iris_srcnn_perform_config(enable,
				proc_h, proc_v, strategy, path_sel);
		if (changed)
			_iris_srcnn_config_pwil_csc(enable);
		break;
	default:
		break;
	}

	return changed;
}

static void _iris_scl_proc(uint32_t count, uint32_t *values,
		uint32_t scl_type)
{
	uint32_t strategy = SCL_2D_ONLY;
	uint32_t path_sel = SCL_DATA_PATH1;
	bool changed = false;

	if (count != 4 && count != 5)
		return;

	IRIS_LOGI("%s()", __func__);

	strategy = values[3];
	if (count == 5)
		path_sel = values[4];

	changed = _iris_scl_perform(true, scl_type, values[1], values[2],
			strategy, path_sel);
	if (changed)
		_iris_scl_triger_dma(DMA_EVENT_CH8 | DMA_EVENT_CH9);
}

void iris_scl_config(uint32_t count, uint32_t *values)
{
	uint32_t scl_type = IRIS_SCL_INOUT;

	IRIS_LOGI("%s(), count: %u", __func__, count);

	if (count > 5 || values == NULL) {
		IRIS_LOGE("%s(), invalid parameter!", __func__);
		return;
	}

	if (IRIS_IF_LOGD()) {
		int32_t i = 0;

		for (i = 0; i < count; i++)
			IRIS_LOGD("%s(), value[%d] = %u", __func__, i, values[i]);
	}

	if (count != 1) {
		scl_type = values[0];
		if (scl_type >= IRIS_SCL_TYPE_CNT) {
			IRIS_LOGE("%s(), invalid scl type: %u", __func__, scl_type);
			return;
		}

		IRIS_LOGI("%s(), scl type: %u", __func__, scl_type);
	}

	SDE_ATRACE_BEGIN(__func__);
	if (IRIS_IF_LOGI()) {
		uint32_t path_sel = SCL_DATA_PATH1;

		switch (count) {
		case 1:
			IRIS_LOGI("%s(), %u, %u", __func__,
					BITS_GET(values[0], 16, 16), BITS_GET(values[0], 0, 16));
			break;
		case 2:
		case 3:
			if (count == 3)
				path_sel = values[2];

			IRIS_LOGI("%s(), %s, path: %u", __func__,
					values[1] == 0 ? "off" : "on", path_sel);
			break;
		case 4:
		case 5:
			if (count == 5)
				path_sel = values[4];

			IRIS_LOGI("%s(), proc: %#08x, strategy: %u, path: %u", __func__,
					PROC_INFO(values[1], values[2]), values[3], path_sel);
			break;
		default:
			IRIS_LOGE("%s(), invalid count: %u", __func__, count);
			break;
		}
	}

	_iris_scl_set_specific(count, values);
	_iris_scl_disable(count, values, scl_type);
	_iris_scl_proc(count, values, scl_type);
	SDE_ATRACE_END(__func__);
}

static uint32_t _iris_scl_cnn_model_count(void)
{
	uint32_t model_cnt = 0;

	model_cnt = iris_get_cnn_model_count();
	if (model_cnt == 0)
		model_cnt = CNN_MODEL_CNT;

	IRIS_LOGD("%s(), cnn model count %u.", __func__, model_cnt);

	return model_cnt;
}

static void _iris_scl_change_model_proc(uint32_t cnn_model)
{
	int32_t dma_event_channels = DMA_EVENT_CH12;
	uint32_t *payload = NULL;
	uint8_t opt = 0;
	uint8_t lbuf_valid = 0;
	uint8_t rbuf_valid = 0;
	uint8_t body_ctrl = 0;
	bool dpg_on = false;
	bool reconfig_ctrl = false;
	ktime_t ktime;

	if (cnn_model >= _iris_scl_cnn_model_count()) {
		IRIS_LOGE("%s_count(), invalid model: %u", __func__, cnn_model);
		return;
	}

	if (cnn_model == iris_cnn_using_model) {
		IRIS_LOGI("%s(), same model %u, do nothing.", __func__, cnn_model);
		return;
	}

	ktime = ktime_get();

	SDE_ATRACE_BEGIN(__func__);
	if (cnn_model % 2 == 0) {
		dma_event_channels = DMA_EVENT_CH11;
	}

	/* switch ping-pong buffer */
	if (dma_event_channels == DMA_EVENT_CH11)
		lbuf_valid = 1;
	else
		rbuf_valid = 1;

	if (cnn_model != CNN_LOWPOWER_MODEL &&
			iris_cnn_using_model == CNN_LOWPOWER_MODEL) {
		body_ctrl = 0xF;
		reconfig_ctrl = true;
	}

	if (cnn_model == CNN_LOWPOWER_MODEL &&
			iris_cnn_using_model != CNN_LOWPOWER_MODEL) {
		body_ctrl = 0x3;
		reconfig_ctrl = true;
	}

	if (reconfig_ctrl) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_SR, 0x01, 3);
		if (payload == NULL) {
			IRIS_LOGE("%s(), failed to find: %02x, %02x",
					__func__, IRIS_IP_SR, 0x01);
			SDE_ATRACE_END(__func__);
			return;
		}

		IRIS_LOGI("%s(), body ctrl changed: %#x", __func__, body_ctrl);

		dma_event_channels |= DMA_EVENT_CH8;

		payload[0] = BITS_SET(payload[0], 4, 16, body_ctrl);
		iris_init_update_ipopt_t(IRIS_IP_SR, 0x01, 0x01, 1);
	}

	payload = iris_get_ipopt_payload_data(IRIS_IP_DMA, 0xF1, 2);
	if (payload == NULL) {
		IRIS_LOGE("%s(), failed to find GDMA CTRL.", __func__);
		SDE_ATRACE_END(__func__);
		return;
	}

	payload[0] = BITS_SET(payload[0], 1, 31, lbuf_valid);
	payload[3] = BITS_SET(payload[3], 1, 31, rbuf_valid);
	iris_init_update_ipopt_t(IRIS_IP_DMA, 0xF1, 0xF1, 1);

	iris_cnn_using_model = cnn_model;

	if (cnn_model == iris_cnn_loaded_models[0] ||
			cnn_model == iris_cnn_loaded_models[1]) {
		_iris_scl_triger_dma(dma_event_channels);
		SDE_ATRACE_END(__func__);

		IRIS_LOGI("%s(), model %u, trigger %s buffer only, time cost %llu us.",
				__func__, cnn_model, lbuf_valid == 1 ? "Left" : "Right",
				ktime_to_us(ktime_get() - ktime));
		return;
	}

	if (iris_dynamic_power_get()) {
		iris_dynamic_power_set(false, false);
		dpg_on = true;
	}

	opt = (cnn_model * 0x10 + 0x00) & 0xFF;
	iris_init_update_ipopt_t(SR_LUT, opt, opt, 1);
	opt = (cnn_model * 0x10 + 0x01) & 0xFF;
	iris_init_update_ipopt_t(SR_LUT, opt, opt, 1);
	opt = (cnn_model * 0x10 + 0x02) & 0xFF;
	iris_init_update_ipopt_t(SR_LUT, opt, opt, 1);
	opt = (cnn_model * 0x10 + 0x03) & 0xFF;
	iris_init_update_ipopt_t(SR_LUT, opt, opt, 1);
	opt = (cnn_model * 0x10 + 0x04) & 0xFF;
	iris_init_update_ipopt_t(SR_LUT, opt, opt, 1);

	iris_cnn_loaded_models[cnn_model % 2] = cnn_model;
	_iris_scl_triger_dma(dma_event_channels);

	if (dpg_on)
		iris_dynamic_power_set(true, false);

	SDE_ATRACE_END(__func__);

	IRIS_LOGI("%s(), load model %u and trigger %s buffer, time cost %llu us.",
			__func__, cnn_model, lbuf_valid == 1 ? "Left" : "Right",
			ktime_to_us(ktime_get() - ktime));
}

static void _iris_scl_cnn_set(uint32_t type, uint32_t model);
void iris_scl_change_model(uint32_t count, uint32_t *values)
{
	uint32_t cnn_model = 0;

	IRIS_LOGI("%s(), count: %u", __func__, count);

	if (count != 1 || values == NULL) {
		IRIS_LOGE("%s(), invalid parameter!", __func__);
		return;
	}

	cnn_model = values[0];
	_iris_scl_cnn_set(SCL_DATA_PATH1, cnn_model);
	_iris_scl_change_model_proc(cnn_model);
}

static void _iris_scl_ptsr_switch_proc(bool enable,
		uint32_t proc_h, uint32_t proc_v)
{
	IRIS_LOGI("%s(), %s, proc: %#08x", __func__,
			enable ? "on" : "off", PROC_INFO(proc_h, proc_v));

	_iris_scl_perform(enable, IRIS_SCL_INOUT, proc_h, proc_v,
			iris_expected_strategy[SCL_DATA_PATH1], SCL_DATA_PATH1);
}

static void _iris_scl_ptsr_switch(uint32_t count, uint32_t *values)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	bool enable = false;
	uint32_t proc_h = 0;
	uint32_t proc_v = 0;

	if (count != 1 && count != 2)
		return;

	SDE_ATRACE_BEGIN(__func__);
	enable = values[0] == 1;
	if (enable) {
		proc_h = _iris_scl_conf[SCL_DATA_PATH1].sr_in_h;
		proc_v = _iris_scl_conf[SCL_DATA_PATH1].sr_in_v;
	} else {
		proc_h = pcfg->frc_setting.disp_hres;
		proc_v = pcfg->frc_setting.disp_vres;
	}

	if (count == 2)
		iris_expected_strategy[SCL_DATA_PATH1] = values[1];

	_iris_ioinc_config_pwil(enable, proc_h, proc_v, SCL_DATA_PATH1);
	_iris_srcnn_config_pwil_csc(enable);
	_iris_srcnn_enable(enable, SCL_DATA_PATH1);

	_iris_scl_conf[SCL_DATA_PATH1].ioinc_enable = enable;
	_iris_scl_conf[SCL_DATA_PATH1].sr_enable = enable;
	SDE_ATRACE_END(__func__);
}

static bool _iris_sr2d_valid_level(uint32_t level)
{
	if (level >= 0 && level <= 15)
		return true;

	return false;
}

static void _iris_scl_sr2d_set(uint32_t type, uint32_t gf_level,
		uint32_t detect_level, uint32_t peaking_level, uint32_t dti_level)
{
	if (type >= SCL_DATA_PATH_CNT) {
		IRIS_LOGE("%s(), invalid type: %u", __func__, type);
		return;
	}

	IRIS_LOGI("%s(), path%u, [%u %u %u %u]", __func__, type,
			gf_level, detect_level, peaking_level, dti_level);

	if (_iris_sr2d_valid_level(gf_level))
		iris_sr2d_level[type][SCL_2D_GF] = gf_level;
	if (_iris_sr2d_valid_level(detect_level))
		iris_sr2d_level[type][SCL_2D_DETECT] = detect_level;
	if (_iris_sr2d_valid_level(peaking_level))
		iris_sr2d_level[type][SCL_2D_PEAKING] = peaking_level;
	if (_iris_sr2d_valid_level(dti_level))
		iris_sr2d_level[type][SCL_2D_DTI] = dti_level;
}

static bool _iris_scl_sr2d_update(uint32_t type)
{
	bool ret = false;
	uint32_t gf_level = 0;
	uint32_t detect_level = 0;
	uint32_t peaking_level = 0;
	uint32_t dti_level = 0;

	if (type >= SCL_DATA_PATH_CNT) {
		IRIS_LOGE("%s(), invalid type: %u", __func__, type);
		return ret;
	}

	gf_level = iris_sr2d_level[type][SCL_2D_GF];
	detect_level = iris_sr2d_level[type][SCL_2D_DETECT];
	peaking_level = iris_sr2d_level[type][SCL_2D_PEAKING];
	dti_level = iris_sr2d_level[type][SCL_2D_DTI];

	/* GF */
	if (iris_sr2d_using_level[SCL_2D_GF] != gf_level) {
		IRIS_LOGI("%s(), GF level: %u", __func__, gf_level);

		iris_sr2d_using_level[SCL_2D_GF] = gf_level;
		gf_level += 0x10;
		iris_update_ip_opt(IRIS_IP_SR, gf_level, 1);
		ret = true;
	}

	/* DETECT */
	if (iris_sr2d_using_level[SCL_2D_DETECT] != detect_level) {
		IRIS_LOGI("%s(), DETECT level: %u", __func__, detect_level);

		iris_sr2d_using_level[SCL_2D_DETECT] = detect_level;
		detect_level += 0x20;
		iris_update_ip_opt(IRIS_IP_SR, detect_level, 1);
		ret = true;
	}

	/* PEAKING */
	if (iris_sr2d_using_level[SCL_2D_PEAKING] != peaking_level) {
		IRIS_LOGI("%s(), PEAKING level: %u", __func__, peaking_level);

		iris_sr2d_using_level[SCL_2D_PEAKING] = peaking_level;
		peaking_level += 0x40;
		iris_update_ip_opt(IRIS_IP_SR, peaking_level, 1);
		ret = true;
	}

	/* DTI */
	if (iris_sr2d_using_level[SCL_2D_DTI] != dti_level) {
		IRIS_LOGI("%s(), DTI level: %u", __func__, dti_level);

		iris_sr2d_using_level[SCL_2D_DTI] = dti_level;
		dti_level += 0x30;
		iris_update_ip_opt(IRIS_IP_SR, dti_level, 1);
		ret = true;
	}

	return ret;
}

static void _iris_scl_cnn_set(uint32_t type, uint32_t model)
{
	if (type >= SCL_DATA_PATH_CNT) {
		IRIS_LOGE("%s(), invalid type: %u", __func__, type);
		return;
	}

	if (model >= _iris_scl_cnn_model_count()) {
		IRIS_LOGE("%s(), invalid model: %u", __func__, model);
		return;
	}

	iris_cnn_models[type] = model;
}

static void _iris_scl_cnn_update(uint32_t type)
{
	if (type >= SCL_DATA_PATH_CNT) {
		IRIS_LOGE("%s(), invalid type: %u", __func__, type);
		return;
	}

	_iris_scl_change_model_proc(iris_cnn_models[type]);
}

static bool _iris_scl_update_pq(uint32_t type)
{
	bool ret = false;

	if (type >= SCL_DATA_PATH_CNT) {
		IRIS_LOGE("%s(), invalid type: %u", __func__, type);
		return ret;
	}

	IRIS_LOGI("%s(), type: %u, strategy: %u", __func__,
			type, iris_expected_strategy[type]);

	switch (iris_expected_strategy[type]) {
	case SCL_2D_ONLY:
		ret = _iris_scl_sr2d_update(type);
		break;
	case SCL_CNN_ONLY:
	case SCL_CNN_1D:
		ret = (iris_cnn_models[type] != iris_cnn_using_model);
		_iris_scl_cnn_update(type);
		break;
	case SCL_CNN_2D:
		ret = _iris_scl_sr2d_update(type);
		ret |= (iris_cnn_models[type] != iris_cnn_using_model);
		_iris_scl_cnn_update(type);
		break;
	default:
		break;
	}

	return ret;
}


static void _iris_scl_ptsr_sr2d(uint32_t count, uint32_t *values)
{
	bool enable = false;
	uint32_t proc_h = 0;
	uint32_t proc_v = 0;

	if (count != 7)
		return;

	SDE_ATRACE_BEGIN(__func__);
	enable = values[0] == 1;
	proc_h = values[1];
	proc_v = values[2];
	iris_expected_strategy[SCL_DATA_PATH1] = SCL_2D_ONLY;

	_iris_scl_sr2d_set(SCL_DATA_PATH1,
			values[3], values[4], values[5], values[6]);
	if (!_iris_memc_mode())
		_iris_scl_sr2d_update(SCL_DATA_PATH1);
	_iris_scl_ptsr_switch_proc(enable, proc_h, proc_v);
	SDE_ATRACE_END(__func__);
}

static void _iris_scl_ptsr_cnn(uint32_t count, uint32_t *values)
{
	bool enable = false;
	uint32_t proc_h = 0;
	uint32_t proc_v = 0;
	uint32_t cnn_model = 0;

	if (count != 4 && count != 5)
		return;

	SDE_ATRACE_BEGIN(__func__);
	enable = values[0] == 1;
	proc_h = values[1];
	proc_v = values[2];
	cnn_model = values[3];
	if (count == 5)
		iris_expected_strategy[SCL_DATA_PATH1] = values[4];
	else
		iris_expected_strategy[SCL_DATA_PATH1] = SCL_CNN_ONLY;

	_iris_scl_cnn_set(SCL_DATA_PATH1, cnn_model);
	if (!_iris_memc_mode())
		_iris_scl_change_model_proc(cnn_model);
	_iris_scl_ptsr_switch_proc(enable, proc_h, proc_v);
	SDE_ATRACE_END(__func__);
}

static void _iris_scl_ptsr_cnn_2d(uint32_t count, uint32_t *values)
{
	bool enable = false;
	uint32_t proc_h = 0;
	uint32_t proc_v = 0;
	uint32_t cnn_model = 0;

	if (count != 8)
		return;

	SDE_ATRACE_BEGIN(__func__);
	enable = values[0] == 1;
	proc_h = values[1];
	proc_v = values[2];
	cnn_model = values[7];
	iris_expected_strategy[SCL_DATA_PATH1] = SCL_CNN_2D;

	_iris_scl_cnn_set(SCL_DATA_PATH1, cnn_model);
	_iris_scl_sr2d_set(SCL_DATA_PATH1,
			values[3], values[4], values[5], values[6]);
	if (!_iris_memc_mode()) {
		_iris_scl_sr2d_update(SCL_DATA_PATH1);
		_iris_scl_change_model_proc(cnn_model);
	}
	_iris_scl_ptsr_switch_proc(enable, proc_h, proc_v);
	SDE_ATRACE_END(__func__);
}

void iris_scl_ptsr_config(uint32_t count, uint32_t *values)
{
	ktime_t ktime;

	if (count > 8 || values == NULL) {
		IRIS_LOGE("%s(), invalid parameter, count: %u", __func__, count);
		return;
	}

	ktime = ktime_get();

	SDE_ATRACE_BEGIN(__func__);
	if (IRIS_IF_LOGI()) {
		switch (count) {
		case 1:
			IRIS_LOGI("%s(), %s", __func__,
					values[0] == 1 ? "on" : "off");
			break;
		case 2:
			IRIS_LOGI("%s(), %s, strategy: %s(%u)", __func__,
					values[0] == 1 ? "on" : "off",
					values[1], srcnn_case_name[values[1]]);
			break;
		case 4:
			IRIS_LOGI("%s(), CNN: %s, %u x %u, model: %u", __func__,
					values[0] == 1 ? "on" : "off",
					values[1], values[2], values[3]);
			break;
		case 5:
			IRIS_LOGI("%s(), CNN+1D: %s, %u x %u, model: %u", __func__,
					values[0] == 1 ? "on" : "off",
					values[1], values[2], values[3]);
			break;
		case 7:
			IRIS_LOGI("%s(), 2D: %s, %u x %u, [%u %u %u %u]", __func__,
					values[0] == 1 ? "on" : "off",
					values[1], values[2],
					values[3], values[4], values[5], values[6]);
			break;
		case 8:
			IRIS_LOGI("%s(), CNN+2D: %s, %u x %u, [%u %u %u %u], model: %u",
					__func__,
					values[0] == 1 ? "on" : "off",
					values[1], values[2],
					values[3], values[4], values[5], values[6],
					values[7]);
			break;
		default:
			IRIS_LOGE("%s(), invalid count: %u", __func__, count);
			break;
		}
	}

	_iris_scl_ptsr_switch(count, values);
	_iris_scl_ptsr_sr2d(count, values);
	_iris_scl_ptsr_cnn(count, values);
	_iris_scl_ptsr_cnn_2d(count, values);

	_iris_scl_triger_dma(DMA_EVENT_CH8 | DMA_EVENT_CH9);
	SDE_ATRACE_END(__func__);

	IRIS_LOGI("%s(), time cost %llu us.", __func__,
			ktime_to_us(ktime_get() - ktime));
}

void iris_memc_set_pq_level(uint32_t count, uint32_t *values)
{
	switch (count) {
	case 2:
		_iris_scl_cnn_set(SCL_DATA_PATH0, values[1]);
		break;
	case 3:
		_iris_scl_cnn_set(SCL_DATA_PATH0, values[1]);
		iris_expected_strategy[SCL_DATA_PATH0] = values[2];
		break;
	case 5:
		_iris_scl_sr2d_set(SCL_DATA_PATH0,
				values[1], values[2], values[3], values[4]);
		break;
	case 6:
		_iris_scl_sr2d_set(SCL_DATA_PATH0,
				values[1], values[2], values[3], values[4]);
		iris_expected_strategy[SCL_DATA_PATH0] = values[5];
		break;
	case 7:
		_iris_scl_cnn_set(SCL_DATA_PATH0, values[5]);
		_iris_scl_sr2d_set(SCL_DATA_PATH0,
				values[1], values[2], values[3], values[4]);
		break;
	case 8:
		_iris_scl_cnn_set(SCL_DATA_PATH0, values[5]);
		_iris_scl_sr2d_set(SCL_DATA_PATH0,
				values[1], values[2], values[3], values[4]);
		iris_expected_strategy[SCL_DATA_PATH0] = values[6];
		break;
	default:
		IRIS_LOGE("%s(), invalid parameter, count: %u", __func__, count);
		return;
	}

	if (!_iris_memc_mode())
		return;

	if (_iris_scl_update_pq(SCL_DATA_PATH0))
		_iris_scl_triger_dma(DMA_EVENT_CH8);
}

void iris_memc_get_pq_level(uint32_t count, uint32_t *values)
{
	if (values == NULL) {
		IRIS_LOGE("%s(), invalid parameter, count: %u", __func__, count);
		return;
	}

	values[0] = iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_GF];
	values[1] = iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_DETECT];
	values[2] = iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_PEAKING];
	values[3] = iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_DTI];
	values[4] = iris_cnn_models[SCL_DATA_PATH0];
}

void iris_scl_ptsr_get(uint32_t count, uint32_t *values)
{
	IRIS_LOGI("%s(), count: %u", __func__, count);

	if (count != 9) {
		IRIS_LOGE("%s(), invalid count.", __func__);
		return;
	}

	values[0] = _iris_scl_conf[SCL_DATA_PATH1].sr_enable;
	values[1] = _iris_scl_conf[SCL_DATA_PATH1].sr_in_h;
	values[2] = _iris_scl_conf[SCL_DATA_PATH1].sr_in_v;
	values[3] = _iris_scl_conf[SCL_DATA_PATH1].sr_strategy;
	values[4] = iris_sr2d_level[SCL_DATA_PATH1][SCL_2D_GF];
	values[5] = iris_sr2d_level[SCL_DATA_PATH1][SCL_2D_DETECT];
	values[6] = iris_sr2d_level[SCL_DATA_PATH1][SCL_2D_PEAKING];
	values[7] = iris_sr2d_level[SCL_DATA_PATH1][SCL_2D_DTI];
	values[8] = iris_cnn_models[SCL_DATA_PATH1];
}

static void _iris_ioinc_group_filter(uint32_t count, uint32_t *values)
{
	uint32_t tap = IOINC_TAG5;
	uint8_t ip_soft = IOINC1D_LUT;
	uint8_t ip_sharp = IOINC1D_LUT_SHARP;
	uint8_t level = 0;

	if (count != 3)
		return;

	tap = values[0];
	if (tap == IOINC_TAG9) {
		ip_soft = IOINC1D_LUT_9TAP;
		ip_sharp = IOINC1D_LUT_SHARP_9TAP;
	}

	/* soft filter */
	level = 0x00 + values[1]; /* hs_y */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);
	level = 0x40 + values[1]; /* hs_uv */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);
	level = 0x80 + values[1]; /* vs_y */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);
	level = 0xC0 + values[1]; /* vs_uv */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);

	/* sharp filter */
	level = 0x00 + values[2]; /* hs_y */
	iris_init_update_ipopt_t(ip_sharp, level, level, 1);
	level = 0x40 + values[2]; /* hs_uv */
	iris_init_update_ipopt_t(ip_sharp, level, level, 1);
	level = 0x80 + values[2]; /* vs_y */
	iris_init_update_ipopt_t(ip_sharp, level, level, 1);
	level = 0xC0 + values[2]; /* vs_uv */
	iris_init_update_ipopt_t(ip_sharp, level, level, 0);

	iris_update_pq_opt(PATH_DSI, true);
}

static void _iris_ioinc_specific_filter(uint32_t count, uint32_t *values)
{
	uint32_t tap = IOINC_TAG5;
	uint32_t group = FILTER_SOFT;
	uint32_t type = FILTER_HS_Y_LEVEL;
	uint8_t ip = IOINC1D_LUT;
	uint8_t level = 0;

	if (count != 4)
		return;

	tap = values[0];
	group = values[1];
	type = values[2];

	if (tap == IOINC_TAG5 && group == FILTER_SOFT)
		ip = IOINC1D_LUT;
	else if (tap == IOINC_TAG5 && group == FILTER_SHARP)
		ip = IOINC1D_LUT_SHARP;
	else if (tap == IOINC_TAG9 && group == FILTER_SOFT)
		ip = IOINC1D_LUT_9TAP;
	else if (tap == IOINC_TAG9 && group == FILTER_SHARP)
		ip = IOINC1D_LUT_SHARP_9TAP;

	switch (type) {
	case FILTER_HS_Y_LEVEL:
		level = 0x00;
		break;
	case FILTER_HS_UV_LEVEL:
		level = 0x40;
		break;
	case FILTER_VS_Y_LEVEL:
		level = 0x80;
		break;
	case FILTER_VS_UV_LEVEL:
		level = 0xC0;
		break;
	default:
		break;
	}

	level += values[3];
	iris_init_update_ipopt_t(ip, level, level, 0);

	iris_update_pq_opt(PATH_DSI, true);
}

static void _iris_ioinc_group_hv_filter(uint32_t count, uint32_t *values)
{
	uint32_t tap = IOINC_TAG5;
	uint8_t ip_soft = IOINC1D_LUT;
	uint8_t ip_sharp = IOINC1D_LUT_SHARP;
	uint8_t level = 0;

	if (count != 5)
		return;

	tap = values[0];
	if (tap == IOINC_TAG9) {
		ip_soft = IOINC1D_LUT_9TAP;
		ip_sharp = IOINC1D_LUT_SHARP_9TAP;
	}

	/* soft filter */
	level = 0x00 + values[1]; /* hs_y */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);
	level = 0x40 + values[1]; /* hs_uv */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);
	level = 0x80 + values[2]; /* vs_y */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);
	level = 0xC0 + values[2]; /* vs_uv */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);

	/* sharp filter */
	level = 0x00 + values[3]; /* hs_y */
	iris_init_update_ipopt_t(ip_sharp, level, level, 1);
	level = 0x40 + values[3]; /* hs_uv */
	iris_init_update_ipopt_t(ip_sharp, level, level, 1);
	level = 0x80 + values[4]; /* vs_y */
	iris_init_update_ipopt_t(ip_sharp, level, level, 1);
	level = 0xC0 + values[4]; /* vs_uv */
	iris_init_update_ipopt_t(ip_sharp, level, level, 0);

	iris_update_pq_opt(PATH_DSI, true);
}

static void _iris_ioinc_group_all_filter(uint32_t count,
		uint32_t *values)
{
	uint32_t tap = IOINC_TAG5;
	uint32_t group = FILTER_SOFT;
	uint8_t ip = IOINC1D_LUT;
	uint8_t level = 0;

	if (count != 6)
		return;

	tap = values[0];
	group = values[1];

	if (tap == IOINC_TAG5 && group == FILTER_SOFT)
		ip = IOINC1D_LUT;
	else if (tap == IOINC_TAG5 && group == FILTER_SHARP)
		ip = IOINC1D_LUT_SHARP;
	else if (tap == IOINC_TAG9 && group == FILTER_SOFT)
		ip = IOINC1D_LUT_9TAP;
	else if (tap == IOINC_TAG9 && group == FILTER_SHARP)
		ip = IOINC1D_LUT_SHARP_9TAP;

	level = 0x00 + values[2]; /* hs_y */
	iris_init_update_ipopt_t(ip, level, level, 1);
	level = 0x40 + values[3]; /* hs_uv */
	iris_init_update_ipopt_t(ip, level, level, 1);
	level = 0x80 + values[4]; /* vs_y */
	iris_init_update_ipopt_t(ip, level, level, 1);
	level = 0xC0 + values[5]; /* vs_uv */
	iris_init_update_ipopt_t(ip, level, level, 0);

	iris_update_pq_opt(PATH_DSI, true);
}

static void _iris_ioinc_all_filter(uint32_t count, uint32_t *values)
{
	uint32_t tap = IOINC_TAG5;
	uint8_t ip_soft = IOINC1D_LUT;
	uint8_t ip_sharp = IOINC1D_LUT_SHARP;
	uint8_t level = 0;

	if (count != 9)
		return;

	tap = values[0];
	if (tap == IOINC_TAG9) {
		ip_soft = IOINC1D_LUT_9TAP;
		ip_sharp = IOINC1D_LUT_SHARP_9TAP;
	}

	/* soft filter */
	level = 0x00 + values[1]; /* hs_y */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);
	level = 0x40 + values[2]; /* hs_uv */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);
	level = 0x80 + values[3]; /* vs_y */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);
	level = 0xC0 + values[4]; /* vs_uv */
	iris_init_update_ipopt_t(ip_soft, level, level, 1);

	/* sharp filter */
	level = 0x00 + values[5]; /* hs_y */
	iris_init_update_ipopt_t(ip_sharp, level, level, 1);
	level = 0x40 + values[6]; /* hs_uv */
	iris_init_update_ipopt_t(ip_sharp, level, level, 1);
	level = 0x80 + values[7]; /* vs_y */
	iris_init_update_ipopt_t(ip_sharp, level, level, 1);
	level = 0xC0 + values[8]; /* vs_uv */
	iris_init_update_ipopt_t(ip_sharp, level, level, 0);

	iris_update_pq_opt(PATH_DSI, true);
}

void iris_scl_ioinc_filter(uint32_t count, uint32_t *values)
{
	IRIS_LOGI("%s(), count: %u", __func__, count);

	if (count < 3 || values == NULL) {
		IRIS_LOGE("%s(), invalid parameter!", __func__);
		return;
	}

	_iris_ioinc_group_filter(count, values);
	_iris_ioinc_specific_filter(count, values);
	_iris_ioinc_group_hv_filter(count, values);
	_iris_ioinc_group_all_filter(count, values);
	_iris_ioinc_all_filter(count, values);
}

static void _iris_sr1d_force_filter(uint32_t count, uint32_t *values)
{
	uint8_t level = 0;

	if (count != 1)
		return;

	level = 0x40 + values[0]; /* hs_y */
	iris_init_update_ipopt_t(IOINC1D_PP_LUT, level, level, 1);
	level = 0xC0 + values[0]; /* hs_uv */
	iris_init_update_ipopt_t(IOINC1D_PP_LUT, level, level, 1);
	level = 0x00 + values[0]; /* vs_y */
	iris_init_update_ipopt_t(IOINC1D_PP_LUT, level, level, 1);
	level = 0x80 + values[0]; /* vs_uv */
	iris_init_update_ipopt_t(IOINC1D_PP_LUT, level, level, 0);

	iris_update_pq_opt(PATH_DSI, true);
}

static void _iris_sr1d_specific_filter(uint32_t count, uint32_t *values)
{
	uint32_t type = FILTER_HS_Y_LEVEL;
	uint8_t level = 0;

	if (count != 2)
		return;

	type = values[0];

	switch (type) {
	case FILTER_HS_Y_LEVEL:
		level = 0x40;
		break;
	case FILTER_HS_UV_LEVEL:
		level = 0xC0;
		break;
	case FILTER_VS_Y_LEVEL:
		level = 0x00;
		break;
	case FILTER_VS_UV_LEVEL:
		level = 0x80;
		break;
	default:
		break;
	}

	level += values[1];
	iris_init_update_ipopt_t(IOINC1D_PP_LUT, level, level, 0);

	iris_update_pq_opt(PATH_DSI, true);
}

static void _iris_sr1d_all_filter(uint32_t count, uint32_t *values)
{
	uint8_t level = 0;

	if (count != 4)
		return;

	level = 0x40 + values[1]; /* hs_y */
	iris_init_update_ipopt_t(IOINC1D_PP_LUT, level, level, 1);
	level = 0xC0 + values[3]; /* hs_uv */
	iris_init_update_ipopt_t(IOINC1D_PP_LUT, level, level, 1);
	level = 0x00 + values[0]; /* vs_y */
	iris_init_update_ipopt_t(IOINC1D_PP_LUT, level, level, 1);
	level = 0x80 + values[2]; /* vs_uv */
	iris_init_update_ipopt_t(IOINC1D_PP_LUT, level, level, 0);

	iris_update_pq_opt(PATH_DSI, true);
}

void iris_scl_sr1d_filter(uint32_t count, uint32_t *values)
{
	IRIS_LOGI("%s(), count: %u", __func__, count);

	if (count > 4 || values == NULL) {
		IRIS_LOGE("%s(), invalid parameter!", __func__);
		return;
	}

	_iris_sr1d_force_filter(count, values);
	_iris_sr1d_specific_filter(count, values);
	_iris_sr1d_all_filter(count, values);
}


void iris_memc_init_param(void)
{
	SDE_ATRACE_BEGIN(__func__);
	_iris_dsc_init_param();
	_iris_scl_init_param();
	SDE_ATRACE_END(__func__);
}

void iris_memc_parse_info(void)
{
	SDE_ATRACE_BEGIN(__func__);
	_iris_dsc_parse_info();
	_iris_scl_parse_info();
	SDE_ATRACE_END(__func__);
}

void iris_memc_setting_off(void)
{
	SDE_ATRACE_BEGIN(__func__);
	_iris_dsc_setting_off();
	_iris_scl_setting_off();
	SDE_ATRACE_END(__func__);
}

void iris_memc_helper_change(void)
{
	SDE_ATRACE_BEGIN(__func__);
	_iris_memc_scl_change();
	_iris_memc_dsc_change();
	SDE_ATRACE_END(__func__);
}

void iris_memc_helper_post(void)
{
	SDE_ATRACE_BEGIN(__func__);
	if (_iris_scl_update_pq(SCL_DATA_PATH1))
		_iris_scl_triger_dma(DMA_EVENT_CH8);
	SDE_ATRACE_END(__func__);
}


const char *iris_ptsr_status(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	const char *status = NULL;

	if (pcfg->abyp_ctrl.abypass_mode != PASS_THROUGH_MODE)
		return status;

	if (pcfg->pwil_mode == FRC_MODE) {
		if (!_iris_scl_conf[SCL_DATA_PATH0].sr_enable)
			return status;

		/* 'MEMC + SR' for Game only */
		if (pcfg->memc_info.memc_mode != MEMC_SINGLE_GAME_ENABLE)
			return status;

		switch (_iris_scl_conf[SCL_DATA_PATH0].sr_strategy) {
		case SCL_CNN_ONLY:
		case SCL_CNN_1D:
		case SCL_CNN_2D:
			status = "SINGLE-MEMC-HESR";
			break;
		case SCL_1D_ONLY:
			status = "SINGLE-MEMC-SR";
			break;
		case SCL_2D_ONLY:
			if (iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_GF] != 0 ||
				iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_DETECT] != 0 ||
				iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_PEAKING] != 0 ||
				iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_DTI] != 0)
				status = "SINGLE-MEMC-SR";
			break;
		default:
			break;
		}

		return status;
	}

	if (!_iris_scl_conf[SCL_DATA_PATH1].sr_enable)
		return status;

	switch (_iris_scl_conf[SCL_DATA_PATH1].sr_strategy) {
	case SCL_CNN_ONLY:
	case SCL_CNN_1D:
	case SCL_CNN_2D:
		status = "SINGLE-PT-HESR";
		break;
	case SCL_2D_ONLY:
	case SCL_1D_ONLY:
		status = "SINGLE-PT-SR";
		break;
	default:
		break;
	}

	IRIS_LOGI_IF(status != NULL, "%s(), status: %s", __func__, status);

	return status;
}

static const char *_iris_sr_status(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	const char *status = NULL;

	if (pcfg->abyp_ctrl.abypass_mode != PASS_THROUGH_MODE)
		return status;

	if (pcfg->pwil_mode == FRC_MODE) {
		if (!_iris_scl_conf[SCL_DATA_PATH0].sr_enable)
			return status;

		switch (_iris_scl_conf[SCL_DATA_PATH0].sr_strategy) {
		case SCL_CNN_ONLY:
			status = "SINGLE-MEMC-HESR(CNN)";
			break;
		case SCL_CNN_1D:
			status = "SINGLE-MEMC-HESR(CNN-1D)";
			break;
		case SCL_CNN_2D:
			status = "SINGLE-MEMC-HESR(CNN-2D)";
			break;
		case SCL_2D_ONLY:
			if (iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_GF] != 0 ||
				iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_DETECT] != 0 ||
				iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_PEAKING] != 0 ||
				iris_sr2d_level[SCL_DATA_PATH0][SCL_2D_DTI] != 0)
				status = "SINGLE-MEMC-LPSR(2D)";
			break;
		case SCL_1D_ONLY:
			status = "SINGLE-MEMC-LPSR(1D)";
			break;
		default:
			break;
		}

		return status;
	}

	if (!_iris_scl_conf[SCL_DATA_PATH1].sr_enable)
		return status;

	switch (_iris_scl_conf[SCL_DATA_PATH1].sr_strategy) {
	case SCL_CNN_ONLY:
		status = "SINGLE-PT-HESR(CNN)";
		break;
	case SCL_CNN_1D:
		status = "SINGLE-PT-HESR(CNN-1D)";
		break;
	case SCL_CNN_2D:
		status = "SINGLE-PT-HESR(CNN-2D)";
		break;
	case SCL_2D_ONLY:
		status = "SINGLE-PT-LPSR(2D)";
		break;
	case SCL_1D_ONLY:
		status = "SINGLE-PT-LPSR(1D)";
		break;
	default:
		break;
	}

	IRIS_LOGI_IF(status != NULL, "%s(), status: %s", __func__, status);

	return status;
}

static int _iris_get_sr_info(char *kbuf, int size)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t path_sel = SCL_DATA_PATH1;
	const char *status = _iris_sr_status();
	int len = 0;

	if (pcfg->pwil_mode == FRC_MODE)
		path_sel = SCL_DATA_PATH0;

	len += snprintf(kbuf, size,
			"%-20s:\t%s\n", "SR mode", status == NULL ? "Not set" : status);
	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%u x %u\n", "proc size",
			_iris_scl_conf[path_sel].sr_in_h, _iris_scl_conf[path_sel].sr_in_v);
	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%u %u %u %u\n", "2D parameter",
			iris_sr2d_using_level[SCL_2D_GF],
			iris_sr2d_using_level[SCL_2D_DETECT],
			iris_sr2d_using_level[SCL_2D_PEAKING],
			iris_sr2d_using_level[SCL_2D_DTI]);
	len += snprintf(kbuf + len, size - len,
			"%-20s:\t%u\n", "CNN model", iris_cnn_using_model);

	return len;
}

static ssize_t _iris_dbg_show_sr_info(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	const BUF_SIZE = 256;
	char *kbuf = NULL;
	int size = count < BUF_SIZE ? BUF_SIZE : count;

	if (*ppos)
		return 0;

	kbuf = vzalloc(size);
	if (kbuf == NULL) {
		IRIS_LOGE("Fatal erorr: No mem!\n");
		return -ENOMEM;
	}

	size = _iris_get_sr_info(kbuf, size);
	if (size >= count)
		size = count - 1;

	if (copy_to_user(ubuf, kbuf, size)) {
		vfree(kbuf);
		return -EFAULT;
	}

	vfree(kbuf);

	*ppos += size;

	return size;
}

static const struct file_operations iris_dbg_sr_info_fops = {
	.open = simple_open,
	.read = _iris_dbg_show_sr_info,
};

int iris_dbgfs_scl_init(struct dsi_display *display)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("%s(), create dir for iris failed, error %ld",
					__func__, PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	if (debugfs_create_file("sr_info", 0644, pcfg->dbg_root, display,
				&iris_dbg_sr_info_fops) == NULL)
		IRIS_LOGE("%s(), create file 'sr_info' failed!", __func__);

	return 0;
}
