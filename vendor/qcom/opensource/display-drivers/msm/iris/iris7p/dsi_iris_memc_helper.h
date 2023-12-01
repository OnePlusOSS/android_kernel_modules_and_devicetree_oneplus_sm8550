/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_MEMC_HELPER_
#define _DSI_IRIS_MEMC_HELPER_


enum {
	DSC_PPS_SET0, /* for MEMC */
	DSC_PPS_SET1, /* for RFB */
	DSC_PPS_SET_CNT
};

struct iris_memc_dsc_info {
	uint32_t pic_width;
	uint32_t pic_height;
	uint32_t slice_per_pkt;
	uint32_t slice_width;
	uint32_t slice_height;
	uint32_t bits_per_component;
	uint32_t bits_per_pixel;
	uint32_t version_minor;
	uint32_t chroma_format;
	bool hw_slice_prefer;
};

void iris_memc_parse_info(void);
void iris_memc_init_param(void);
void iris_memc_setting_off(void);
void iris_memc_helper_change(void);
void iris_rfb_helper_change(void);
void iris_memc_helper_post(void);

void iris_memc_dsc_config(uint32_t count, uint32_t *values);
struct iris_memc_dsc_info *iris_memc_dsc_info(uint32_t pps_sel);

void iris_memc_set_pq_level(uint32_t count, uint32_t *values);
void iris_memc_get_pq_level(uint32_t count, uint32_t *values);

bool iris_scl_ptsr_1to1(void);
void iris_scl_config(uint32_t count, uint32_t *values);
void iris_scl_change_model(uint32_t count, uint32_t *values);
void iris_scl_ioinc_filter(uint32_t count, uint32_t *values);
void iris_scl_sr1d_filter(uint32_t count, uint32_t *values);
void iris_scl_ptsr_config(uint32_t count, uint32_t *values);
void iris_scl_ptsr_get(uint32_t count, uint32_t *values);
const char *iris_ptsr_status(void);
int iris_dbgfs_scl_init(struct dsi_display *display);

#endif
