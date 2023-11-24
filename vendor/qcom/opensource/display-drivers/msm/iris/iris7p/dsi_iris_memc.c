// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/types.h>
#include <dsi_drm.h>
#include <video/mipi_display.h>
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
#include "dsi_iris_memc_i7p.h"
#include "dsi_iris_memc_helper.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_i3c.h"
#include "dsi_iris_dts_fw.h"
#include "dsi_iris_reg_i7p.h"
#include "dsi_iris_cmpt.h"

extern u32 iris_pq_disable;
static u32 debug_disable_frc_dynen;
static u32 debug_frc_force_repeat;
static u32 iris_frc_layer_b_enable;
static u32 iris_frc_layer_c_enable = 1;
static u32 iris_mcu_enable = 1;
static u32 debug_disable_mcu_check;
static u32 debug_switch_dump_clear;
static u32 iris_fi_drop_frm_thr = 3;  //0: means disable vfr, others: means enable vfr.
static u32 val_frcc_reg3 = 0x00060003;
static u32 debug_low_latency_overwrite;
static u32 debug_frc_vfr_overwrite;
static u32 debug_n2m_mode_overwrite;
static u32 debug_mv_res_overwrite;
static u32 debug_eco_overwrite;
static u32 debug_raw_cmd_overwrite;
static u32 debug_emv_dec_adj = 16;
static u32 debug_emv_local_fallback = 3;
static u32 iris_three_buffer_low_latency;
static int iris_memc_cmd_payload_count;
static u32 *iris_memc_cmd_payload;
#define IRIS_MEMC_CMD_PAYLOAD_LENGTH	300	// 128 registers



void iris_pwil0_efifo_setting_reset_i7p(void)
{
	u32 value;
	u32 *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 4);
	if (!payload)
		return;
	value = payload[0];
	value &= ~(0x6);
	/* update cmdlist */
	iris_set_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 4, value);
}

void iris_pwil0_efifo_enable_i7p(bool enable)
{
	u32 reg_val;
	u32 *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 4);
	if (!payload)
		return;
	reg_val = payload[0];
	if (enable)
		/* set DATA_PATH_CTRL0->EFIFO_EN,EFIFO_POSITION to 0x3 */
		payload[0] = BITS_SET(reg_val, 2, 1, 0x3);
	else
		/* set DATA_PATH_CTRL0->EFIFO_EN,EFIFO_POSITION to 0 */
		payload[0] = BITS_SET(reg_val, 2, 1, 0);
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x01, 0x01, 1);

	/* update */
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x80, 0x80, 1);
	iris_dma_trig(DMA_CH9, 0);
	iris_update_pq_opt(PATH_DSI, true);
}

void iris_set_pwil_mode_i7p(u8 mode, int state, bool commit)
{
	char pwil_mode[2] = {0x00, 0x00};
	struct dsi_cmd_desc_pxlw iris_pwil_mode_cmd_pxlw = {
		{0, MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM, 0, 0, 0,
			sizeof(pwil_mode), pwil_mode, 0, NULL}, 1, 0};
	struct dsi_cmd_desc iris_pwil_mode_cmd;
	struct dsi_panel_cmd_set panel_cmds = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &iris_pwil_mode_cmd,
	};
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 meta;

	remap_to_qcom_style(&iris_pwil_mode_cmd, &iris_pwil_mode_cmd_pxlw, 1);

	if (mode == PT_MODE) {
		pwil_mode[0] = 0x0;
		pwil_mode[1] = 0x81;
	} else if (mode == RFB_MODE) {
		pwil_mode[0] = 0xc;
		pwil_mode[1] = 0x81;
	} else if (mode == FRC_MODE) {
		pwil_mode[0] = 0x4;
		pwil_mode[1] = 0x82;
	}

	if (pcfg->panel->cur_mode && pcfg->panel->cur_mode->priv_info &&
			pcfg->panel->cur_mode->priv_info->dsc_enabled)
		pwil_mode[0] |= 0x10;

	if (mode == FRC_MODE && pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE)
		pwil_mode[0] |= 0x40;

	IRIS_LOGI("%s(), set pwil mode: %x, %x", __func__, pwil_mode[0], pwil_mode[1]);

	if (commit)
		iris_dsi_send_cmds(pcfg->panel, panel_cmds.cmds,
				panel_cmds.count, panel_cmds.state, pcfg->vc_ctrl.to_iris_vc_id);

	meta = pwil_mode[0] | (pwil_mode[1] << 8);
	iris_rx_meta_dma_list_send(meta, commit);
}

void iris_dtg_eco_i7p(bool enable, bool chain)
{
	u32 *payload = NULL;

	IRIS_LOGI("%s: %d", __func__, enable);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x10, 2);
	if (enable) {
		payload[0] &= ~0x100;
		payload[0] |= 0x800;
	} else {
		payload[0] |= 0x100;
		payload[0] &= ~0x800;
	}
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x10, 0x10, 1);
	if (chain) {
		iris_dma_trig(DMA_CH9, 0);
		iris_update_pq_opt(PATH_DSI, true);
	}
}

void iris_input_frame_cnt_record_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_switch_dump *dump = &pcfg->switch_dump;

	/* recode input frame count when send video meta */
	if (!dump->trigger)
		dump->rx_frame_cnt0 = iris_ocp_read(0xf1a00354, DSI_CMD_SET_STATE_HS);
}

void iris_memc_info_set_i7p(u32 *values)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_memc_info *info = NULL;

	info = (struct iris_memc_info *)values;
	if (info->bit_mask == 0xff) {
		pcfg->memc_info = *info;
	} else if (info->bit_mask == 0x04) {
		pcfg->memc_info.video_fps = info->video_fps;
	}
}

int iris_low_latency_mode_get_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return pcfg->memc_info.low_latency_mode;
}

void iris_memc_cmd_payload_init_i7p(void)
{
	iris_memc_cmd_payload_count = 0;
	if (iris_memc_cmd_payload == NULL)
		iris_memc_cmd_payload = vmalloc(IRIS_MEMC_CMD_PAYLOAD_LENGTH * sizeof(u32));
	if (iris_memc_cmd_payload == NULL)
		IRIS_LOGE("can not vmalloc size = %d in %s", IRIS_MEMC_CMD_PAYLOAD_LENGTH, __func__);
}

void iris_memc_cmd_payload_send_i7p(void)
{
	if (iris_memc_cmd_payload != NULL && iris_memc_cmd_payload_count != 0) {
		IRIS_LOGI("iris_memc_cmd_payload_count: %d", iris_memc_cmd_payload_count);
		iris_ocp_write_mult_vals(iris_memc_cmd_payload_count, iris_memc_cmd_payload);
		vfree(iris_memc_cmd_payload);
		iris_memc_cmd_payload = NULL;
	}
}

u32 iris_disp_vtotal_get_i7p(void)
{
	uint32_t *payload;
	uint32_t vtotal;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0x00, 2);
	if (!payload)
		return 0;
	vtotal = payload[4] + payload[5] + payload[6]  + payload[7];

	return vtotal;
}

u32 iris_frc_video_hstride_calc_i7p(u16 hres, u16 bpp, u16 slice_num)
{
	u32 hstride;

	if (debug_raw_cmd_overwrite) {
		hstride = (hres * 30 + 63) / 64;
	} else {
		hstride = bpp * (hres + slice_num - 1);
		hstride = (hstride + 15) >> 4;
		hstride = ((hstride  + 7) / 8) * 8;
		hstride = (hstride + 63) / 64;
	}

	IRIS_LOGI("%s:%d hstride:%d", __func__, __LINE__, hstride);
	return hstride;
}

void iris_frc_reg_add_i7p(u32 addr, u32 val, bool last)
{
	u32 *payload = &iris_memc_cmd_payload[iris_memc_cmd_payload_count];

	if (iris_memc_cmd_payload_count < IRIS_MEMC_CMD_PAYLOAD_LENGTH) {
		*payload = addr;
		payload++;
		*payload = val;
		iris_memc_cmd_payload_count += 2;
	} else {
		IRIS_LOGE("payload buffer length is not enough! %s", __func__);
	}
}

void iris_frc_mif_emv_reg_set_i7p(void)
{
	u32 *payload = NULL;
	u32 mv_addr, mv_hstride, mv_offset, ratio = 1;
	u32 ctrl12 = 0x04008a48;
	struct iris_cfg *pcfg = iris_get_cfg();

	/* mv addr ctrl */
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x0a, 2);
	if (!payload)
		return;

	mv_addr = payload[7];
	mv_hstride = payload[9] & 0xffff;
	mv_offset = payload[2] - payload[7];

	if (debug_emv_local_fallback & 0x02) {
		ratio = pcfg->memc_info.panel_fps / pcfg->memc_info.video_fps;
		ctrl12 |= (1 << 27);
		if (ratio > 1)
			ctrl12 |= ((ratio - 2) << 28);
	}

	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG12, ctrl12, 0);

	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_EXT_MV_BSADR, mv_addr, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_EXT_MV_HSTRIDE, mv_hstride, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_EXT_MV_OFF, mv_offset, 0);
}

void iris_frc_mif_reg_set_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;

	u32 val_frcc_reg0 = 0x001b0101;
	u32 val_frcc_reg1 = 0;
	u32 val_frcc_reg2 = 0x47d5e6f2;
	u32 val_frcc_reg4 = 0x09168000;
	u32 val_frcc_reg5 = 0x06020414;
	u32 val_frcc_reg6 = 0x00008000;
	u32 val_frcc_phase_ctrl_0 = 0x02adfc04;
	u32 val_frcc_phase_ctrl_1 = 0x00551919;
	u32 val_frcc_fk_ctrl_3 = 0x3c3e0218;
	u32 val_frcc_fk_ctrl_4 = 0x8000332d;
	u32 val_frcc_fk_ctrl_5 = 0x0051a7a8;
	u32 val_frcc_enable_0 = 0x0025d09f;
	u32 val_frcc_enable_1 = 0x904b1000;
	u32 val_frcc_reg10 = 0x29101d;
	u32 val_frcc_reg11 = 0x1002f03d;
	u32 val_frcc_reg13 = 0;
	u32 val_eco_reserved_ctrl = 0x0000080c;
	u32 val_frc_ds_ctrl = 0x000a0186;
	u32 val_frcc_mvc = 0x00000018;

	u16 fmif_vd_hstride,  fmif_mv_hstride;
	u32 fmif_vd_offset, fmif_mv_frm_offset;
	u16 mv_hres, mv_vres;
	u32 disp_vtotal = iris_disp_vtotal_get_i7p();

	u32 fmif_a_mv_hstride = 0;
	u32 fmif_a_mv_frm_offset = 0;
	u32 fmif_c_mv_hstride = 0;
	u32 fmif_c_mv_frm_offset = 0;
	u32 mvc_gen_cnt_thr, fi_meta_fast_entry_thr;
	u32 memc_level = 3;

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE) {
		val_frcc_enable_0 = 0x0025d090;
		val_frc_ds_ctrl = 0x000a0180;
	}

	fmif_vd_hstride = iris_frc_video_hstride_calc_i7p(frc_setting->mv_hres,
			iris_memc_dsc_info(DSC_PPS_SET0)->bits_per_pixel,
			iris_memc_dsc_info(DSC_PPS_SET0)->slice_per_pkt);
	fmif_vd_offset = fmif_vd_hstride * frc_setting->mv_vres * 8;
	fmif_mv_hstride = ((frc_setting->mv_hres + 15) / 16 * 48 + 63) / 64;
	fmif_mv_frm_offset = fmif_mv_hstride * CEILING(frc_setting->mv_vres, 16) * 8;
	mv_hres = CEILING(frc_setting->mv_hres, 16);
	mv_vres = CEILING(frc_setting->mv_vres, 16);

	fmif_a_mv_hstride = ((frc_setting->mv_hres + 15) / 16 * 48 + 63) / 64;
	fmif_a_mv_frm_offset = fmif_a_mv_hstride*((frc_setting->mv_vres + 15)/16)*8;

	fmif_c_mv_hstride = ((frc_setting->mv_hres + 7) / 8 * 48 + 63) / 64;
	fmif_c_mv_frm_offset = fmif_c_mv_hstride*((frc_setting->mv_vres + 7)/8)*8;

	mvc_gen_cnt_thr = frc_setting->mv_vres >> 2;
	fi_meta_fast_entry_thr = mvc_gen_cnt_thr + 2;

	val_frcc_reg1 = (mvc_gen_cnt_thr << 16);
	val_frcc_reg6 |= fi_meta_fast_entry_thr;
	//if (pcfg->rx_mode == DSI_OP_VIDEO_MODE)
	//	val_frcc_reg2 &= ~0x1000000;

	/*frc_ds*/
	iris_frc_reg_add_i7p(IRIS_FRC_DS_ADDR + FRC_DS_CTRL, val_frc_ds_ctrl, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_DS_ADDR + FRC_DS_RESO, frc_setting->mv_hres | (frc_setting->mv_vres << 16), 0);
	iris_frc_reg_add_i7p(IRIS_FRC_DS_ADDR + FRC_DS_SHDW, 0x00000003, 0);

	if (pcfg->rx_mode == DSI_OP_CMD_MODE && pcfg->tx_mode == DSI_OP_VIDEO_MODE)
		val_frcc_enable_0 &= ~0x2;	// disable FMD

	if (pcfg->memc_info.panel_fps == 120) {
		val_frcc_fk_ctrl_3 = 0x1e1f0218;
		val_frcc_fk_ctrl_4 = 0x800033d6;
		val_frcc_fk_ctrl_5 = 0x0028d3d4;
	} else if (pcfg->memc_info.panel_fps == 90) {
		val_frcc_fk_ctrl_3 = 0x282a0218;
		val_frcc_fk_ctrl_4 = 0x800033de;
		val_frcc_fk_ctrl_5 = 0x0035151a;
	}

	if (iris_low_latency_mode_get_i7p() == ULTRA_LT_MODE) {
		/* two buffer mode, disable REP_FRM_DET_EN */
		if (!iris_three_buffer_low_latency)
			val_frcc_enable_0 &= (~0x00000080);
		val_frcc_reg1 = 0x00280000;
		val_frcc_phase_ctrl_0 = 0x042dfc04;
		val_frcc_phase_ctrl_1 = 0x00511919;
		/* phase_mapping_en=1 & DEFAULT_MTN_LVL = 4 on UL mode*/
		val_frcc_fk_ctrl_4 = (val_frcc_fk_ctrl_4 & (~0x0003c000)) | (4 << 14);
	} else if ((iris_low_latency_mode_get_i7p() == LT_MODE) ||
			(iris_low_latency_mode_get_i7p() == NORMAL_LT)) {
		val_frcc_enable_0 |= ((pcfg->frc_setting.layer_c_en << 9) |
							(iris_frc_layer_b_enable << 8));
	} else {
		val_frcc_enable_0 |= ((pcfg->frc_setting.layer_c_en << 9) |
							(iris_frc_layer_b_enable << 8) |
							(pcfg->frc_setting.frc_dynen << 13));
	}
	if (debug_raw_cmd_overwrite)
		val_frcc_enable_0 &= (~0x00000010);//disable video compression for raw

	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE) &&
		(pcfg->memc_info.n2m_mode == 1 || pcfg->memc_info.n2m_mode == 2)) {
		val_frcc_enable_0 &= (~0x00000080);  //enable video N2M need disable REP_FRM_DET
		val_frcc_reg5 = 0x06000400; //disable REP_FRM_DET need set REP_FRM_GMD_LOWTHR 0
		val_frcc_phase_ctrl_1 = 0x00551996;
	}
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_ENABLE_0, val_frcc_enable_0, 0);

	if ((pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE) ||
		(iris_low_latency_mode_get_i7p() == ULTRA_LT_MODE &&
		 !iris_three_buffer_low_latency))
		val_frcc_reg0 = (val_frcc_reg0 & (~0x00070000)) | (2 << 16);
	if (pcfg->memc_info.n2m_mode == 1 || pcfg->memc_info.n2m_mode == 2)
		val_frcc_reg0 = (val_frcc_reg0 & (~0x0000ffff)) | (2 << 8) | 1;

	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG0, val_frcc_reg0, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG1, val_frcc_reg1, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG5, val_frcc_reg5, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_PHASE_CTRL_0, val_frcc_phase_ctrl_0, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_PHASE_CTRL_1, val_frcc_phase_ctrl_1, 0);

	if (pcfg->rx_mode == DSI_OP_VIDEO_MODE)
		val_frcc_enable_1 |= 0x200;	// enable FI_MIF_LOCK_EN
	else if (pcfg->rx_mode == DSI_OP_CMD_MODE && pcfg->tx_mode == DSI_OP_VIDEO_MODE)
		val_frcc_enable_1 |= 0x200;	// enable FI_MIF_LOCK_EN
	if (iris_low_latency_mode_get_i7p() == ULTRA_LT_MODE) {
		val_frcc_enable_1 |= 0x804;	// enable FI_FAST_ENTRY & PHASE_FWD_EN
	} else if (iris_low_latency_mode_get_i7p() == LT_MODE)
		val_frcc_enable_1 |= 0x800; // only enable PHASE_FWD_EN

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE)
		val_frcc_enable_1 |= 0x48004;
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_ENABLE_1, val_frcc_enable_1, 0);
	/* in vin-vout mode, need disable COMMAND_MODE */
	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
		val_frcc_reg2 &= ~0x01000000;
		val_frcc_reg4 |= 0x20000000;
		val_frcc_reg11 |= 0x40000000;
	}
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG2, val_frcc_reg2, 0);

	if (pcfg->frc_setting.frc_vfr_disp)
		val_frcc_reg3 = (val_frcc_reg3 & 0x0fffffff) | ((iris_fi_drop_frm_thr & 0xf) << 28);
	else
		val_frcc_reg3 = val_frcc_reg3 & 0x0fffffff;
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG3, val_frcc_reg3, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG4, val_frcc_reg4, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG6, val_frcc_reg6, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_FK_CTRL_3, val_frcc_fk_ctrl_3, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_FK_CTRL_4,
					val_frcc_fk_ctrl_4 | (memc_level << 6), 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_FK_CTRL_5, val_frcc_fk_ctrl_5, 0);
	if ((frc_setting->mv_hres > 96) && (frc_setting->mv_vres > 384))
		val_frcc_reg10 &= (~0x00000010);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG10, val_frcc_reg10, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG11, val_frcc_reg11, 0);

	//ceil(hsize * bpp / 32)
	val_frcc_reg13 = DIV_ROUND_UP(frc_setting->mv_hres *
			iris_memc_dsc_info(DSC_PPS_SET0)->bits_per_pixel * 1000, 16000);
	val_frcc_reg13 = DIV_ROUND_UP(val_frcc_reg13 * 1000, 32000);
	val_frcc_reg13 = BITS_SET(val_frcc_reg13, 1, 13, 1);
	val_frcc_reg13 |= 0x00108000; //need set OUT_RATIO_2 1 as default
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG13, val_frcc_reg13, 0);

	if (iris_low_latency_mode_get_i7p() == NORMAL_LT || iris_low_latency_mode_get_i7p() == LT_MODE)
		val_frcc_mvc = 0x8;
	else if (iris_low_latency_mode_get_i7p() == ULTRA_LT_MODE)
		val_frcc_mvc = 0xd8;
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MVC_CTRL_0, val_frcc_mvc, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MVF_CTRL_0, (frc_setting->force_repeat << 1), 0);

	if (pcfg->rx_mode == DSI_OP_VIDEO_MODE && pcfg->tx_mode == DSI_OP_VIDEO_MODE)
		val_eco_reserved_ctrl = val_eco_reserved_ctrl | 0x000000d0;
	if (iris_low_latency_mode_get_i7p() == ULTRA_LT_MODE) {
		if (debug_eco_overwrite == 1 && pcfg->memc_info.memc_mode == MEMC_SINGLE_GAME_ENABLE)
			val_eco_reserved_ctrl = val_eco_reserved_ctrl | 0x00000610;
		else
			val_eco_reserved_ctrl = val_eco_reserved_ctrl | 0x00000600;
	}
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_CTRL_ECO_REV_CTRL, val_eco_reserved_ctrl, 0);

	//film border init value
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_FBD_CTRL_0,
		(frc_setting->mv_vres << 16) | frc_setting->mv_hres, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_FBD_CTRL_1, frc_setting->mv_hres << 16, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_FBD_CTRL_2, frc_setting->mv_vres << 16, 0);

	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + VD_MEMORY_BSADR, frc_setting->video_baseaddr, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + VD_MEMORY_OFFSET, fmif_vd_offset, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + VD_MEMORY_HSTRIDE, fmif_vd_hstride, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A1_BSADR,
		frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A1_OFF, fmif_a_mv_frm_offset, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A1_HSTRIDE, fmif_a_mv_hstride, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A0_BSADR,
		frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
		+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A0_OFF, fmif_a_mv_frm_offset, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A0_HSTRIDE, fmif_a_mv_hstride, 0);

	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_OFF, fmif_c_mv_frm_offset, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_HSTRIDE, fmif_c_mv_hstride, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_OFF, fmif_c_mv_frm_offset, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_HSTRIDE, fmif_c_mv_hstride, 0);

	if (iris_low_latency_mode_get_i7p() == ULTRA_LT_MODE) {
		if (!iris_three_buffer_low_latency) {
			iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_BSADR,
				frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
				+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
			iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_BSADR,
				frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
				+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
		} else {
			iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_BSADR,
				frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
				+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
			iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_BSADR,
				frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
				+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2
				+ fmif_c_mv_frm_offset*frc_setting->mv_buf_num, 0);
		}
	} else if (!pcfg->frc_setting.layer_c_en) {
		iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_BSADR,
			frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
			+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
		iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_BSADR,
			frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
			+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
	} else {
		iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_BSADR,
			frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
			+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
		iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_BSADR,
			frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
			+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2
			+ fmif_c_mv_frm_offset*frc_setting->mv_buf_num, 0);
	}

	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + IMIF_MODE_CTRL,
		0x4 + ((frc_setting->mv_vres * 3 / 4) << 16), 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + IMIF_DW_PER_LINE, fmif_vd_hstride, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + IMIF_VSIZE, (frc_setting->mv_vres), 0);

	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + MMIF_CTRL1, 0x00402007, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + MMIF_CTRL2, 0x8 + (disp_vtotal << 16), 0);

	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FMIF_CTRL, 0xFF004034, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FMIF_VD_FRM_ATTRIBUTE0, (0x10 << 16) + fmif_vd_hstride, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FMIF_VD_FRM_ATTRIBUTE1, (0x40 << 16) + frc_setting->mv_vres, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FMIF_MV_FRM_ATTRIBUTE4, (mv_hres << 16) + mv_vres, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FMIF_MV_FRM_ATTRIBUTE5, (0x4 << 16) + fmif_mv_hstride, 0);

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE)
		iris_frc_mif_emv_reg_set_i7p();
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FRCC_REG_SHDW, 0x00000003, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + IMIF_SW_UPDATE_EN, 0x00000003, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + MMIF_UPDATE, 0x00000003, 0);
	iris_frc_reg_add_i7p(IRIS_FRC_MIF_ADDR + FMIF_REG_SHDW, 0x00000003, 0);
}

void iris_gmd_reg_set_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;

	iris_frc_reg_add_i7p(IRIS_GMD_ADDR + GMD_GAIN, 0x0000C488, 0);
	iris_frc_reg_add_i7p(IRIS_GMD_ADDR + GMD_FILT, 0x00000000, 0);
	iris_frc_reg_add_i7p(IRIS_GMD_ADDR + GMD_ACCUM, 0x00000659, 0);
	iris_frc_reg_add_i7p(IRIS_GMD_ADDR + GMD_SHIFT, 0x00000070, 0);
	iris_frc_reg_add_i7p(IRIS_GMD_ADDR + GMD_START, 0x00000000, 0);
	iris_frc_reg_add_i7p(IRIS_GMD_ADDR + GMD_STOP,
		frc_setting->mv_hres + (frc_setting->mv_vres << 16), 0);
	/* it is aimed at Genshin Impact and Game for Peace */
	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_GAME_ENABLE &&
		(pcfg->memc_info.memc_app == 3 || pcfg->memc_info.memc_app == 0)) {
		iris_frc_reg_add_i7p(IRIS_GMD_ADDR + GMD_START_WIN, 0x00320000, 0);
		iris_frc_reg_add_i7p(IRIS_GMD_ADDR + GMD_STOP_WIN, ((frc_setting->mv_vres - 50) << 16) +
			(frc_setting->mv_hres), 0);
		iris_frc_reg_add_i7p(IRIS_GMD_ADDR + GMD_SUM_SEL, 0x00000001, 0);
	}
	iris_frc_reg_add_i7p(IRIS_GMD_ADDR + GMD_CTRL, 0x00000001, 0);
}

void iris_fbd_reg_set_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;

	iris_frc_reg_add_i7p(IRIS_FBD_ADDR + FILMBD_RESOLUTION,
				(frc_setting->mv_hres) + ((frc_setting->mv_vres) << 16), 0);
	iris_frc_reg_add_i7p(IRIS_FBD_ADDR + FILMBD_WIN_STOP_SET,
				frc_setting->mv_hres + (frc_setting->mv_vres << 16), 0);
	iris_frc_reg_add_i7p(IRIS_FBD_ADDR + FILMBD_TOP_CTRL, 0x00010025, 1);

}

void iris_cad_reg_set_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_frc_reg_add_i7p(IRIS_CAD_ADDR + NEW_FRM_FLG_DET_1, 0x06920311, 0);
	iris_frc_reg_add_i7p(IRIS_CAD_ADDR + CAD_DET_BASIC_CAD, 0x0000010c, 0);
	iris_frc_reg_add_i7p(IRIS_CAD_ADDR + CAD_DET_STVT, 0x00004083, 0);
	iris_frc_reg_add_i7p(IRIS_CAD_ADDR + CAD_DET_BAD_EDIT, 0x00011255, 0);
	iris_frc_reg_add_i7p(IRIS_CAD_ADDR + CAD_DET_VOF_0, 0x096823C1, 0);
	iris_frc_reg_add_i7p(IRIS_CAD_ADDR + DUMMY, 0x000000F1, 0);
	if (pcfg->rx_mode == DSI_OP_VIDEO_MODE) {
		switch (pcfg->memc_info.video_fps) {
		case 24:
		case 30:
			iris_frc_reg_add_i7p(IRIS_CAD_ADDR + COMMON, 0x00000079, 0);
			break;
		default:
			iris_frc_reg_add_i7p(IRIS_CAD_ADDR + COMMON, 0x00000078, 0);
			break;
		}
	}

	iris_frc_reg_add_i7p(IRIS_CAD_ADDR + SW_DONE, 0x00000001, 1);
}

void iris_game_memc_registers_set_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 i;

	i = 0;
	while (pcfg->memc_info.latencyValue[i]) {
		iris_frc_reg_add_i7p(pcfg->memc_info.latencyValue[i], pcfg->memc_info.latencyValue[i+1], 0);
		i += 2;
	}
}

void iris_game_memc_OSD_registers_set_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 i;

	i = 0;
	while (pcfg->memc_info.OSDProtection[i]) {
		iris_frc_reg_add_i7p(pcfg->memc_info.OSDProtection[i], pcfg->memc_info.OSDProtection[i+1], 0);
		i += 2;
	}
}
void iris_mvc_reg_set_i7p(void)
{
	int mode = 0;

	mode = iris_low_latency_mode_get_i7p();
	if (mode == NORMAL_LT || mode == LT_MODE) {
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_POSTFILT_0, 0x67b444c1, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_KFBOSD_0, 0x12429f43, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_OSDWIN_0, 0x21000e18, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_OSDWIN_1, 0x81024e18, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_OSDWIN_2, 0x80300018, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_TEXTFLAG_0, 0x44440440, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_REGIONMV_2, 0x4440800, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_SAD_2, 0x18ffffff, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_SW_UPDATE, 0x00000001, 0);
	} else if (mode == ULTRA_LT_MODE) {
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_POSTFILT_0, 0x67b444c1, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_KFBOSD_0, 0x12429f43, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_OSDWIN_0, 0x21000e1b, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_OSDWIN_1, 0x81024e1b, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_OSDWIN_2, 0x80300019, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_TEXTFLAG_0, 0x44440448, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_REGIONMV_2, 0x4440880, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_SAD_2, 0x18a8ffff, 0);
		iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_SW_UPDATE, 0x00000001, 0);
	}

	iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_INTEN, 0x00000020, 0);
	iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_INTPRI, 0x00000020, 0);
	iris_frc_reg_add_i7p(IRIS_MVC_ADDR + MVC_INTCLR, 0x00000020, 0);
}

void iris_mvf_reg_set_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int mode = 0;

	mode = iris_low_latency_mode_get_i7p();
	if (mode == NORMAL_LT || mode == LT_MODE) {
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_SMI_CTRL, 0x64, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_VERTIRANGE_CFG, 0x18c, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_CAND_CFG, 0x1f10882c, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + HR_MVC_CFG1, 0x8124, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + HR_MVC_CFG2, 0x90f27, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_MISC_CFG3, 0x2449, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_OSDMV, 0x428c8d2, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_OSDMV2, 0x700013ff, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_OSDMV3, 0x44444093, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_HMTNFB, 0x8818, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_SHDW_CTRL, 0x00000013, 0);
	} else if (mode == ULTRA_LT_MODE) {
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_SMI_CTRL, 0x64, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_VERTIRANGE_CFG, 0x18c, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_CAND_CFG, 0x1f10882c, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + HR_MVC_CFG1, 0x8224, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + HR_MVC_CFG2, 0x90524, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_MISC_CFG3, 0x2449, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_OSDMV, 0x488c8d2, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_OSDMV2, 0x70001102, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_OSDMV3, 0x44444093, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_HMTNFB, 0x148818, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_SHDW_CTRL, 0x00000013, 0);
	}

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE) {
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + EMVSEL_CFG2, 0x42200480, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + EMVSEL_CFG10, 0x10112134, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + EMVSEL_CFG13, 0x0f002780, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + EMVSEL_CFG15, 0x000280a0, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + EMVSEL_CFG19, 0x00411100, 0);
		iris_frc_reg_add_i7p(IRIS_MVF_ADDR + MVF_SHDW_CTRL, 0x13, 0);
	}
}

/* control mvc dsy buffer mode and buffer size */
void iris_ramctrl_mvc_dsy_ctrl_i7p(void)
{
	u8 dsy_bufmode_sel = 0;
	u8 dsy_buf_8m = 0;
	u32 value;

	/* in low latency mode, the mvc_dsy ram ctrl setting is 2buffers, 8M.*/
	if (iris_low_latency_mode_get_i7p() == ULTRA_LT_MODE && !iris_three_buffer_low_latency) {
		dsy_bufmode_sel = 1;
		dsy_buf_8m = 1;
	}

	value = dsy_bufmode_sel | (dsy_buf_8m << 1);
	iris_frc_reg_add_i7p(IRIS_SRAM_CTRL_ADDR + RAMCTRL_MVC_DSY_CTRL, value, 0);
}

u32 iris_fi_demo_win_color_get_i7p(void)
{
	u32 val = 0x51ef5a;
	struct iris_cfg *pcfg = iris_get_cfg();

	switch (pcfg->memc_info.memc_mode) {
	case MEMC_SINGLE_VIDEO_ENABLE:
		val = 0x51ef5a;  /* red */
		break;
	case MEMC_SINGLE_GAME_ENABLE:
		val = 0xd29210;  /* yellow */
		break;
	case MEMC_SINGLE_EXTMV_ENABLE:
		val = 0x286def;  /* blue */
		break;
	default:
		break;
	}

	return val;
}

void iris_fi_reg_set_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;
	u32 val_range_ctrl;
	u32 lines, search_lines;
	u32 vrange_top, vrange_bot;
	u32 demo_win_yuv = 0x51ef5a;
	u32 packmode = 20; /* YUV422 */
	u32 misc_ctrl = 0x0000002A; /* YUV422 */

	if (iris_memc_dsc_info(DSC_PPS_SET0)->chroma_format == MSM_CHROMA_444 || debug_raw_cmd_overwrite) {
		misc_ctrl = 0x00000030; /* YUV444 */
		packmode = 30;
	}

	lines = CEILING(packmode * frc_setting->mv_hres * 2, 512) * 512;
	search_lines = 2 * 1024 * 1024 / lines * 2 - 4;

	//in FPGA, the frc solution is very small, so the range will use a small value.
	//TODO
	//vrange_top = 70;
	//vrange_bot = 71;

	vrange_top = search_lines / 2 - 1;
	vrange_bot = search_lines / 2;
	val_range_ctrl = vrange_top + (vrange_bot << 8);
	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_RANGE_CTRL, val_range_ctrl, 0);
	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_MODE_CTRL, 0x00004020 | pcfg->frc_label << 7
			| (pcfg->frc_demo_window ? 3 : 0), 0);

	demo_win_yuv = iris_fi_demo_win_color_get_i7p();
	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_YUV_VALUE, demo_win_yuv, 0);

	if (pcfg->frc_demo_window == 1) {
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_0, 0, 0);
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_1,
				frc_setting->mv_vres << 15 | frc_setting->mv_hres, 0);
	} else if (pcfg->frc_demo_window == 2) {
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_0,
				frc_setting->mv_vres << 15, 0);
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_1,
				frc_setting->mv_vres << 16 | frc_setting->mv_hres, 0);
	} else if (pcfg->frc_demo_window == 3) {
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_0, 0, 0);
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_1,
			frc_setting->mv_vres << 16 | frc_setting->mv_hres >> 1, 0);
	} else if (pcfg->frc_demo_window == 4) {
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_0,
			frc_setting->mv_hres >> 1, 0);
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_1,
			frc_setting->mv_vres << 16 | frc_setting->mv_hres, 0);
	} else if (pcfg->frc_demo_window == 5) {
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_0,  0, 0);
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_1,
			frc_setting->mv_vres << 16 | frc_setting->mv_hres, 0);
	}

	if (iris_low_latency_mode_get_i7p() == NORMAL_LT || iris_low_latency_mode_get_i7p() == LT_MODE) {
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_CLOCK_GATING_CTRL_1, 0x68e00, 0);
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_BLENDING_CTRL1, 0x4445555f, 0);
	} else if (iris_low_latency_mode_get_i7p() == ULTRA_LT_MODE)
		iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_BLENDING_CTRL1, 0x4455555f, 0);

	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_VIDEO_BUF_CTRL, 0x00002000, 0);
	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_V9_GENERIC_CTRL, 0xffffffff, 0);

	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_MISC_CTRL, misc_ctrl, 0);
	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_ECO_CTRL, 0x00000000, 0);
	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_BLENDING_CTRL3, 0x40000121, 0);
	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_BLENDING_CTRL4, 0x15008fff, 0);
	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_DEBUGBUS_MUX, 0x40000, 0);

	iris_frc_reg_add_i7p(IRIS_FI_ADDR + FI_SHDW_CTRL, 0x00000100, 1); /*set last flag*/
}

void iris_pwil_intstat_clear_i7p(void)
{
	iris_frc_reg_add_i7p(IRIS_PWIL_ADDR + PWIL_INTCLR, 0x40, 0);
}

void iris_fi_demo_window_set_i7p(u32 mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->frc_label = (mode >> 4) & 0x01;
	pcfg->frc_demo_window = mode & 0xf;
}

static void iris_download_mcu_code_i7p(void)
{
	iris_send_ipopt_cmds(APP_CODE_LUT, 0);
	IRIS_LOGI("%s", __func__);
}

static void iris_mcu_sw_reset_i7p(u32 reset)
{
	iris_frc_reg_add_i7p(IRIS_SYS_ADDR + MCU_SW_RESET, reset, 1);
}

static void iris_mcu_int_disable_i7p(void)
{
	iris_frc_reg_add_i7p(IRIS_UNIT_CTRL_INTEN, 0x0, 0x0);
}

static bool iris_mcu_is_idle_i7p(void)
{
	u32 mcu_info_2, mcu_sw_reset;

	if (!debug_disable_mcu_check) {
		mcu_sw_reset = iris_ocp_read(IRIS_SYS_ADDR + MCU_SW_RESET, DSI_CMD_SET_STATE_HS);
		IRIS_LOGI("mcu_sw_reset: %x", mcu_sw_reset);
		if ((mcu_sw_reset & 0x1) == 0x0) {
			mcu_info_2 = iris_ocp_read(IRIS_MCU_INFO_2, DSI_CMD_SET_STATE_HS);
			IRIS_LOGI("mcu_info_2: %x", mcu_info_2);
			if (((mcu_info_2 >> 8) & 0x3) == 3)
				return true;
			else
				return false;
		} else
			return true;
	} else {
		return true;
	}

}

void iris_update_panel_ap_te_i7p(u32 new_te)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->panel_te = new_te;
	pcfg->ap_te = new_te;
}

void iris_set_n2m_enable_i7p(bool enable, u8 n2m_ratio)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->n2m_enable = enable;
	pcfg->n2m_ratio = n2m_ratio;
	IRIS_LOGI("%s: n2m_enable:%d n2m_ratio:%d", __func__, pcfg->n2m_enable, pcfg->n2m_ratio);
}

void iris_frc_vfr_threshold_set_i7p(int count, bool force_update)
{
	u32 write_data[4];

	count &= 0x0f;

	write_data[0] = IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG3;
	write_data[1] = (val_frcc_reg3 & 0x0fffffff) | (count << 28);
	write_data[2] = IRIS_FRC_MIF_ADDR + FRCC_REG_SHDW;
	if (force_update)
		write_data[3] = 0x2;
	else
		write_data[3] = 0x1;
	iris_ocp_write_mult_vals(4, write_data);
	IRIS_LOGD("%s: %d", __func__, count);
}

void iris_dtg_ovs_dly_setting_send_i7p(bool enable)
{
	int len = 0;
	static int iris_low_latency_backup;

	struct iris_ctrl_opt dtg_pt[] = {
		{IRIS_IP_DTG, 0xf8, 0x01},
		{IRIS_IP_DTG, 0xf0, 0x00},
	};

	struct iris_ctrl_opt dtg_low_latency[] = {
		{IRIS_IP_DTG, 0xF4, 0x01},
		{IRIS_IP_DTG, 0xf0, 0x00},
	};

	len = sizeof(dtg_pt)/sizeof(struct iris_ctrl_opt);

	if (iris_low_latency_mode_get_i7p() == ULTRA_LT_MODE) {
		if (enable) {
			iris_send_assembled_pkt(dtg_low_latency, len);
			iris_low_latency_backup = 1;
		} else {
			iris_send_assembled_pkt(dtg_pt, len);
			iris_low_latency_backup = 0;
		}
	} else {
		if ((!enable) && (iris_low_latency_backup)) {
			iris_send_assembled_pkt(dtg_pt, len);
			iris_low_latency_backup = 0;
		}
	}

	IRIS_LOGI("%s, enable: %d, len: %d", __func__, enable, len);
}

void iris_dtg_te_n2m_ctrl_setting_send_i7p(bool enable)
{
	u32 cmd[6];
	u32 *payload = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
		cmd[0] = IRIS_DTG_ADDR + FRC_SW_CTRL2;
		if (enable)
			/*enable n2m frc on video mode*/
			cmd[1] = 0x80020001;
		else
			cmd[1] = 0x00010001;
		cmd[2] = IRIS_DTG_ADDR + DTG_UPDATE;
		cmd[3] = 0xe;
		iris_ocp_write_mult_vals(4, cmd);
	}

	if (enable && pcfg->memc_info.memc_mode == MEMC_DISABLE) {
		if (pcfg->dtg_ctrl_pt == 0) {
			/*enable n2m on PT mode*/
			cmd[0] = IRIS_SYS_ADDR + ALT_CTRL0;
			cmd[1] = 0;

			payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, ID_DTG_TE_SEL, 2);
			if (!payload)
				return;
			pcfg->dtg_ctrl_pt = payload[0];

			cmd[2] = IRIS_DTG_ADDR + DTG_CTRL;
			cmd[3] = (pcfg->dtg_ctrl_pt & (~0x000001c0)) | (1 << 6);
			cmd[4] = IRIS_DTG_ADDR + DTG_UPDATE;
			cmd[5] = 0xe;
			iris_ocp_write_mult_vals(6, cmd);
		}

		cmd[1] = 0x80000100 | pcfg->n2m_ratio;
	} else if (enable && pcfg->memc_info.memc_mode > MEMC_DISABLE) {
		/*enable n2m on frc mode*/
		cmd[1] = 0x80000102;
	} else
		cmd[1] = 0x00000101;
	cmd[0] = IRIS_DTG_ADDR + TE_N2M_CTRL;
	cmd[2] = IRIS_DTG_ADDR + DTG_UPDATE;
	cmd[3] = 0xe;
	iris_ocp_write_mult_vals(4, cmd);

	IRIS_LOGI("%s, enable: %d, dtg_ctrl_pt: 0x%x", __func__, enable, pcfg->dtg_ctrl_pt);
	if (enable == false && pcfg->dtg_ctrl_pt != 0) {
		/*reset sys alt_ctrl0 after disable n2m on PT mode*/
		cmd[0] = IRIS_SYS_ADDR + ALT_CTRL0;
		cmd[1] = 0x02000000;

		/*reset dtg_ctrl after disable n2m on PT mode*/
		cmd[2] = IRIS_DTG_ADDR + DTG_CTRL;
		cmd[3] = pcfg->dtg_ctrl_pt;
		cmd[4] = IRIS_DTG_ADDR + DTG_UPDATE;
		cmd[5] = 0xe;
		iris_ocp_write_mult_vals(6, cmd);

		pcfg->dtg_ctrl_pt = 0;
	}
}

void iris_hangup_timeout_cnt_update_i7p(bool enable)
{
	u32 cmd[4];
	u32 *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe2, 2);
	if (!payload)
		return;
	cmd[0] = IRIS_PWIL_ADDR + PWIL_HANGUP_TIMEOUT_CNT;
	if (enable)
		cmd[1] = payload[0];
	else
		cmd[1] = 0x01ffffff;
	cmd[2] = IRIS_PWIL_ADDR + PWIL_REG_UPDATE;
	cmd[3] = 0x00000100;

	iris_ocp_write_mult_vals(4, cmd);
}

void iris_parse_ap_panel_te_i7p(struct device_node *np)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return;

	pcfg->panel_te = 60;
	p_dts_ops->read_u32(np, "pxlw,panel-te", &(pcfg->panel_te));
	pcfg->ap_te = 60;
	p_dts_ops->read_u32(np, "pxlw,ap-te", &(pcfg->ap_te));
}

void iris_frc_timing_setting_update_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_mode_info *timing;

	if (!pcfg->panel->cur_mode)
		return;

	timing = &pcfg->panel->cur_mode->timing;

	pcfg->panel_te = timing->refresh_rate;
	pcfg->ap_te = timing->refresh_rate;
	// temp treat display timing same as input timing
	pcfg->frc_setting.disp_hres = timing->h_active;
	pcfg->frc_setting.disp_vres = timing->v_active;
}

static void iris_parse_mv_resolution_i7p(void)
{
	u32 *payload = NULL;
	u8 single_opt_id = 0x03;
	struct iris_cfg *pcfg = iris_get_cfg();

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, single_opt_id, 2);
	if (!payload)
		return;
	pcfg->frc_setting.init_single_mv_hres = payload[5] & 0xffff;
	pcfg->frc_setting.init_single_mv_vres = (payload[5] >> 16) & 0xffff;
	pcfg->frc_setting.init_video_baseaddr = payload[7];

	IRIS_LOGI("%s single algo: 0x%04x%04x VideoAddr: %x",
				__func__, pcfg->frc_setting.init_single_mv_hres,
				pcfg->frc_setting.init_single_mv_vres,
				pcfg->frc_setting.init_video_baseaddr);

}

void iris_parse_memc_param0_i7p(struct device_node *np)
{
	iris_parse_ap_panel_te_i7p(np);
}

void iris_parse_memc_param1_i7p(void)
{
	iris_parse_mv_resolution_i7p();
	iris_memc_parse_info();
}

void iris_frc_setting_init_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->frc_setting.mv_buf_num = 3;
	pcfg->frc_setting.vid_buf_num = 3;

	pcfg->frc_setting.mv_hres = pcfg->frc_setting.init_single_mv_hres;
	pcfg->frc_setting.mv_vres = pcfg->frc_setting.init_single_mv_vres;
	pcfg->frc_setting.video_baseaddr = pcfg->frc_setting.init_video_baseaddr;

	iris_frc_timing_setting_update_i7p();
}

void iris_memc_status_clear_i7p(bool init)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	memset(&pcfg->memc_info, 0, sizeof(struct iris_memc_info));
	pcfg->iris_frc_vfr_st = false;
	pcfg->dtg_eco_enabled = false;

	if (debug_switch_dump_clear) {
		pcfg->switch_dump.trigger = false;
		debug_switch_dump_clear = false;
	}
	if (init) {
		pcfg->frc_enabled = false;
		if (pcfg->rx_mode == pcfg->tx_mode)
			pcfg->pwil_mode = PT_MODE;
		else
			pcfg->pwil_mode = RFB_MODE;
		pcfg->iris_pwil_mode_state = 2;

		pcfg->mcu_code_downloaded = false;
	}
}

void iris_memc_frc_phase_update_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	/* FRC LUT */
	if (pcfg->memc_info.panel_fps == 120)
		iris_send_ipopt_cmds(FRC_PHASE_LUT, 2);
	else if (pcfg->memc_info.panel_fps == 90)
		iris_send_ipopt_cmds(FRC_PHASE_LUT, 1);
	else
		iris_send_ipopt_cmds(FRC_PHASE_LUT, 0);
}

void iris_memc_info_update_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	/* if debug via 173 cmd, use default info */
	if (pcfg->memc_info.memc_mode == MEMC_DISABLE) {
		pcfg->memc_info.memc_mode = MEMC_SINGLE_VIDEO_ENABLE;
		pcfg->memc_info.mv_hres = pcfg->frc_setting.init_single_mv_hres;
		pcfg->memc_info.mv_vres = pcfg->frc_setting.init_single_mv_vres;

		pcfg->memc_info.memc_level = 3;
		pcfg->memc_info.video_fps = 30;
		pcfg->memc_info.panel_fps = pcfg->panel_te;
	}

	if (debug_low_latency_overwrite & 0x80)
		pcfg->memc_info.low_latency_mode = debug_low_latency_overwrite & 0x0f;
	if (debug_frc_vfr_overwrite & 0x80)
		pcfg->memc_info.vfr_en = debug_frc_vfr_overwrite & 0x0f;
	if (debug_n2m_mode_overwrite & 0x80)
		pcfg->memc_info.n2m_mode = debug_n2m_mode_overwrite & 0x0f;

	if (debug_mv_res_overwrite & 0x80000000) {
		pcfg->memc_info.mv_hres = debug_mv_res_overwrite & 0xffff;
		pcfg->memc_info.mv_vres = (debug_mv_res_overwrite >> 16) & 0x7fff;
	}

	if (pcfg->memc_info.mv_vres %
			iris_memc_dsc_info(DSC_PPS_SET0)->slice_height)
		iris_memc_dsc_info(DSC_PPS_SET0)->slice_height =
			pcfg->memc_info.mv_vres;

	//when tx output mode is video mode, disable vfr function.
	if (pcfg->tx_mode == DSI_OP_VIDEO_MODE ||
		pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE ||
		pcfg->memc_info.memc_mode == MEMC_SINGLE_GAME_ENABLE)
		pcfg->memc_info.vfr_en = 0;

	if (pcfg->tx_mode == DSI_OP_VIDEO_MODE ||
		pcfg->memc_info.low_latency_mode >= LT_INVALID ||
		(pcfg->memc_info.memc_mode != MEMC_SINGLE_VIDEO_ENABLE &&
		pcfg->memc_info.memc_mode != MEMC_SINGLE_GAME_ENABLE))
		pcfg->memc_info.low_latency_mode = 0;

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE)
		pcfg->memc_info.n2m_mode = 0;

	IRIS_LOGI("memc info: mode-%d, fw-%d, level-%d, ratio-%d-%d, vfr-%d, lt-%d, n2m-%d, osdwin-%d",
		pcfg->memc_info.memc_mode, (pcfg->memc_info.mv_hres << 16) | pcfg->memc_info.mv_vres,
		pcfg->memc_info.memc_level, pcfg->memc_info.video_fps, pcfg->memc_info.panel_fps,
		pcfg->memc_info.vfr_en, pcfg->memc_info.low_latency_mode, pcfg->memc_info.n2m_mode,
		pcfg->memc_info.osd_window_en);
}

void iris_frc_setting_update_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;

	frc_setting->mv_hres = pcfg->memc_info.mv_hres;
	frc_setting->mv_vres = pcfg->memc_info.mv_vres;
	if ((pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE) ||
		(iris_low_latency_mode_get_i7p() == ULTRA_LT_MODE &&
		 !iris_three_buffer_low_latency)) {
		frc_setting->vid_buf_num = 2;
		frc_setting->mv_buf_num = 3;
	} else {
		frc_setting->vid_buf_num = 3;
		frc_setting->mv_buf_num = 3;
	}

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_VIDEO_ENABLE) {
		frc_setting->frc_vfr_disp = pcfg->memc_info.vfr_en;
	} else {
		frc_setting->frc_vfr_disp = false;
	}
	pcfg->iris_frc_vfr_st = frc_setting->frc_vfr_disp;

	if (debug_disable_frc_dynen ||
		pcfg->memc_info.memc_mode == MEMC_SINGLE_GAME_ENABLE ||
		pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE)
		frc_setting->frc_dynen = false;
	else
		frc_setting->frc_dynen = true;

	if (debug_frc_force_repeat)
		frc_setting->force_repeat = true;
	else
		frc_setting->force_repeat = false;

	frc_setting->layer_c_en = iris_frc_layer_c_enable;
	if (frc_setting->mv_hres > 640)
		frc_setting->layer_c_en = false;

	iris_frc_timing_setting_update_i7p();

	IRIS_LOGI("frc setting: disp-%dx%d@%d, fw-%d, buf-%d@%x frc_vfr:%d",
		frc_setting->disp_hres, frc_setting->disp_vres, pcfg->panel_te,
		(frc_setting->mv_hres << 16) | frc_setting->mv_vres,
		frc_setting->mv_buf_num, frc_setting->video_baseaddr,
		frc_setting->frc_vfr_disp);
}

void iris_mcu_memc_info_send_i7p(void)
{
	u32 info[4];
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_memc_info *memc_info;

	memc_info = &pcfg->memc_info;
	info[0] = (memc_info->memc_mode << 28) |
		  (memc_info->low_latency_mode << 24) |
		  (memc_info->vfr_en << 22) |
		  memc_info->memc_app;
	info[1] = (pcfg->frc_setting.mv_hres << 16) |
		  (pcfg->frc_setting.mv_vres);
	info[2] = (pcfg->frc_setting.disp_hres << 16) |
		  (pcfg->frc_setting.disp_vres);
	info[3] = (memc_info->video_fps << 24) |
		  (memc_info->panel_fps << 16) |
		  (pcfg->panel_te << 8);

	iris_frc_reg_add_i7p(IRIS_PROXY_ADDR + 0x08, info[0], 0x0);
	iris_frc_reg_add_i7p(IRIS_PROXY_ADDR + 0x10, info[1], 0x0);
	iris_frc_reg_add_i7p(IRIS_PROXY_ADDR + 0x18, info[2], 0x0);
	iris_frc_reg_add_i7p(IRIS_PROXY_ADDR + 0x20, info[3], 0x0);
}

void iris_mcu_version_read_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	static bool ret;

	if (!ret) {
		pcfg->app_version1 = iris_ocp_read(IRIS_PROXY_ADDR, DSI_CMD_SET_STATE_HS);
		ret = true;
	}
}

void iris_mcu_state_set_i7p(u32 mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_mcu_enable)
		return;

	if (mode == MCU_START) {
		if (!pcfg->mcu_code_downloaded) {
			iris_download_mcu_code_i7p();
			pcfg->mcu_code_downloaded = true;
		}
		iris_mcu_memc_info_send_i7p();
		iris_mcu_sw_reset_i7p(0);
	} else if (mode == MCU_INT_DISABLE) {
		iris_mcu_int_disable_i7p();
	} else if (mode == MCU_STOP) {
		if (iris_mcu_is_idle_i7p())
			iris_mcu_sw_reset_i7p(1);
		else
			IRIS_LOGI("iris mcu not in stop, can't reset mcu");
	}
}

void iris_pwil_reg_set_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;
	u32 *payload = NULL;
	u32 hstride = 0;

	/* frc ctrl */
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x04, 2);
	if (!payload)
		return;
	payload[0] &= (~0x00000001);
	payload[0] |= pcfg->memc_info.vfr_en;

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE) {
		payload[1] &= (~0x00077f1d);
	} else {
		payload[1] &= (~0x00073f10);
		payload[1] |= 0x400d;
	}
	payload[1] |= (frc_setting->vid_buf_num << 8) |
				(frc_setting->vid_buf_num << 11) |
				(frc_setting->mv_buf_num << 16);
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x04, 0x04, 1);

	/* video ctrl */
	hstride = iris_frc_video_hstride_calc_i7p(frc_setting->mv_hres,
			iris_memc_dsc_info(DSC_PPS_SET0)->bits_per_pixel,
			iris_memc_dsc_info(DSC_PPS_SET0)->slice_per_pkt);
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x03, 2);
	if (!payload)
		return;

	payload[0] &= (~0xf);
	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE)
		payload[0] |= 2;
	else
		payload[0] |= 3;

	if (debug_raw_cmd_overwrite)
		payload[1] &= (~0x00010000);//disable dsc compression for raw
	else
		payload[1] |= 0x00010000;//enable dsc compression
	payload[5] = (frc_setting->mv_vres << 16) | frc_setting->mv_hres;
	payload[6] = hstride;
	payload[7] = frc_setting->video_baseaddr;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x03, 0x03, 1);

	/* pwil ctrl */
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 2);
	if (!payload)
		return;
	/* RAW FRC use YUV444 */
	if (debug_raw_cmd_overwrite)
		payload[2] &= (~0x0000c000);

	if (!iris_scl_ptsr_1to1())
		payload[3] = BITS_SET(payload[3], 2, 9, 0); // SR auto mode
	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE) {
		payload[3] &= (~0x00000180);
		payload[3] |= (2 << 7);
	}
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x01, 0x01, 1);

	/* emv ctrl */
	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_EXTMV_ENABLE) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x0a, 2);
		if (!payload)
			return;
		if (debug_emv_dec_adj) {
			payload[10] &= ~(0x3ff);
			payload[10] |= (debug_emv_dec_adj << 2);
		}
		payload[13] &= (~0x1e);
		if (pcfg->memc_info.emv_game_mode == 0)
			payload[13] |= 0x8;
		else
			payload[13] |= 0x14;
		if (debug_emv_local_fallback & 0x1)
			payload[13] |= 0x7c0000;
		else
			payload[13] &= (~0x7c0000);
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x0a, 0x0a, 1);
	}
	/* update */
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x80, 0x80, 1);
	iris_dma_trig(DMA_CH9, 0);
	iris_update_pq_opt(PATH_DSI, true);

}

void iris_pwil_datapath_reset_i7p(bool init)
{
	u32 *payload = NULL;

	/* pwil ctrl */
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 2);
	if (!payload)
		return;
	if (((payload[3] >> 7) & 0x3) == 2) {
		payload[3] &= (~0x00000180);
		if (init) {
			/* update cmdlist */
			iris_set_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 5, payload[3]);
		} else {
			iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x01, 0x01, 1);

			iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x80, 0x80, 1);
			iris_dma_trig(DMA_CH9, 0);
			iris_update_pq_opt(PATH_DSI, true);
		}
	}
}

void iris_memc_ctrl_frc_prepare_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_memc_info_update_i7p();
	iris_frc_setting_update_i7p();

	/* disable FLFP before memc */
	iris_frc_lp_switch(true, false);

	/* disable Line lock before memc in vin-vout*/
	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE))
		iris_linelock_set(false, true);

	iris_dtg_ovs_dly_setting_send_i7p(true);
	iris_hangup_timeout_cnt_update_i7p(true);
	if (pcfg->memc_info.n2m_mode == 1 || pcfg->memc_info.n2m_mode == 2)
		iris_dtg_te_n2m_ctrl_setting_send_i7p(true);

	iris_memc_cmd_payload_init_i7p();
	/* send filter ratio cmd to iris */
	iris_ioinc_filter_ratio_send();

	/* update ioinc/dsc/sr cmdlist and send to iris */
	iris_memc_helper_change();
	/*Fix emv changes WA dont refer HDK codes*/
	usleep_range(17000, 17010);

	iris_pwil_reg_set_i7p();

	/* power domain on */
	iris_pmu_frc_set(true);
	iris_pmu_bsram_set(true);
	iris_pmu_dscu_set(true);

	/* in vin-vout mode, enter memc to need to enable efifo and dtg eco */
	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
		iris_pwil0_efifo_enable_i7p(true);
		iris_dtg_eco_i7p(true, true);
		pcfg->dtg_eco_enabled = true;
	}
	/* clear pwil intstat */
	iris_pwil_intstat_clear_i7p();
	/* mcu */
	iris_mcu_state_set_i7p(MCU_START);
	/* ramctrl */
	iris_ramctrl_mvc_dsy_ctrl_i7p();
	/* config FRC_MIF */
	iris_frc_mif_reg_set_i7p();
	/* GMD */
	iris_gmd_reg_set_i7p();
	/* FBD */
	iris_fbd_reg_set_i7p();
	/* CAD */
	iris_cad_reg_set_i7p();
	/* MVC */
	iris_mvc_reg_set_i7p();
	/* MVF */
	iris_mvf_reg_set_i7p();
	/* FI */
	iris_fi_reg_set_i7p();
	/* DEBUG REGISTERS SETTING */
	iris_game_memc_registers_set_i7p();
	iris_game_memc_OSD_registers_set_i7p();

	/* FRC LUT */
	iris_memc_frc_phase_update_i7p();

	iris_memc_cmd_payload_send_i7p();
}

void iris_memc_ctrl_pt_frc_meta_set_i7p(bool frc_enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (frc_enable) {
		/* send pwil meta cmd */
		iris_set_pwil_mode_i7p(FRC_MODE, DSI_CMD_SET_STATE_HS, true);
		pcfg->frc_enabled = true;
	} else {
		u8 mode = pcfg->rx_mode == pcfg->tx_mode ? PT_MODE : RFB_MODE;
		if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)
			&& (pcfg->memc_info.n2m_mode > 0))
			mode = RFB_MODE;
		/* send pwil meta cmd */
		iris_set_pwil_mode_i7p(mode, DSI_CMD_SET_STATE_HS, true);
		pcfg->frc_enabled = false;
	}

	iris_input_frame_cnt_record_i7p();
}

void iris_memc_ctrl_pt_post_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->rx_mode != pcfg->tx_mode)
		return;

	iris_memc_cmd_payload_init_i7p();

	if (pcfg->dtg_eco_enabled) {
		iris_pwil0_efifo_enable_i7p(false);
		iris_dtg_eco_i7p(false, true);
		pcfg->dtg_eco_enabled = false;
	}
	iris_pwil_datapath_reset_i7p(false);

	/* power domain off */
	iris_pmu_frc_set(false);
	iris_pmu_dscu_set(false);
	iris_pmu_bsram_set(false);

	/* enable flfp after exit frc */
	iris_frc_lp_switch(false, false);

	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)
		&& (pcfg->memc_info.n2m_mode > 0)) {
		return;
	}

	/* enable Line lock before memc in vin-vout*/
	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
		iris_ovs_dly_change(false);
		iris_linelock_set(true, true);
		iris_pwil0_efifo_enable_i7p(false);
	}

	iris_dtg_ovs_dly_setting_send_i7p(false);
	iris_hangup_timeout_cnt_update_i7p(false);
	iris_dtg_te_n2m_ctrl_setting_send_i7p(false);
	/* mcu */
	iris_mcu_state_set_i7p(MCU_STOP);
	iris_memc_cmd_payload_send_i7p();

	iris_memc_status_clear_i7p(false);

	iris_memc_helper_post();
}

void iris_memc_ctrl_pt_to_frc_i7p(void)
{
	iris_tx_pb_req_set(false, false);

	iris_memc_ctrl_pt_frc_meta_set_i7p(true);
}

void iris_memc_ctrl_frc_to_pt_i7p(void)
{
	iris_memc_cmd_payload_init_i7p();

	iris_memc_ctrl_pt_frc_meta_set_i7p(false);

	iris_mcu_state_set_i7p(MCU_INT_DISABLE);

	iris_memc_cmd_payload_send_i7p();
}

void iris_memc_ctrl_vfr_disable_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_frc_vfr_st) {
		iris_frc_vfr_threshold_set_i7p(0, true);
		pcfg->iris_frc_vfr_st = false;
	}
}

void iris_memc_ctrl_pt_rfb_meta_set_i7p(bool rfb_enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (rfb_enable) {
		/* send pwil meta cmd */
		iris_set_pwil_mode_i7p(RFB_MODE, DSI_CMD_SET_STATE_HS, true);
	} else {
		/* send pwil meta cmd */
		iris_set_pwil_mode_i7p(PT_MODE, DSI_CMD_SET_STATE_HS, true);
		pcfg->memc_info.n2m_mode = 0;
	}
}

void iris_memc_ctrl_rfb_prepare_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	/* power domain on */
	iris_pmu_frc_set(true);
	iris_pmu_bsram_set(true);
	iris_pmu_dscu_set(true);

	iris_rfb_helper_change();
	/* disable Line lock before RFB mode in vin-vout*/
	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
		iris_linelock_set(false, true);
		iris_pwil0_efifo_enable_i7p(true);
		iris_ovs_dly_change(true);
	}
}

void iris_memc_ctrl_frc_post_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	/* in vin-vout mode, enter memc to need to enable efifo and dtg eco */
	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
		iris_pwil0_efifo_enable_i7p(false);
		iris_dtg_eco_i7p(false, true);
		pcfg->dtg_eco_enabled = false;
	}
}

void iris_dspp_pq_mode_set_i7p(u32 value)
{
	iris_pq_disable = value;
}

void iris_dport_output_mode_reset_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 *payload = NULL;
	u32 dport_ctrl0, mode;

	if (pcfg->tx_mode == DSI_OP_CMD_MODE)
		mode = 2;
	else
		mode = 1;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPORT, 0x01, 2);
	if (!payload)
		return;
	dport_ctrl0 = payload[0];
	dport_ctrl0 &= ~0xc000;
	dport_ctrl0 |= (mode & 0x3) << 14;

	payload[0] = dport_ctrl0;
	/* update cmdlist */
	iris_set_ipopt_payload_data(IRIS_IP_DPORT, 0x01, 2, dport_ctrl0);
}

void iris_dport_output_mode_select_i7p(bool enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 *payload = NULL;
	u32 dport_ctrl0, mode;
	u32 cmd[4];

	if (!enable)
		mode = 0;
	else if (pcfg->tx_mode == DSI_OP_CMD_MODE)
		mode = 2;
	else
		mode = 1;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPORT, 0x01, 2);
	if (!payload)
		return;
	dport_ctrl0 = payload[0];
	dport_ctrl0 &= ~0xc000;
	dport_ctrl0 |= (mode & 0x3) << 14;

	payload[0] = dport_ctrl0;
	iris_init_update_ipopt_t(IRIS_IP_DPORT, 0x01, 0x01, 0);
	iris_update_pq_opt(PATH_DSI, true);

	cmd[0] = IRIS_DPORT_CTRL0;
	cmd[1] = dport_ctrl0;
	cmd[2] = IRIS_DPORT_REGSEL;
	cmd[3] = 0x1;
	iris_ocp_write_mult_vals(4, cmd);
}

void iris_switch_timeout_dump_i7p(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_switch_dump *dump = &pcfg->switch_dump;

	dump->trigger = true;
	dump->sw_pwr_ctrl = iris_ocp_read(0xf0000068, DSI_CMD_SET_STATE_HS);
	dump->rx_frame_cnt1 = iris_ocp_read(0xf1a00354, DSI_CMD_SET_STATE_HS);
	dump->rx_video_meta = iris_ocp_read(0xf1a00080, DSI_CMD_SET_STATE_HS);
	dump->pwil_cur_meta0 = iris_ocp_read(0xf1940228, DSI_CMD_SET_STATE_HS);
	dump->pwil_status = iris_ocp_read(0xf194015c, DSI_CMD_SET_STATE_HS);
	dump->pwil_disp_ctrl0 = iris_ocp_read(0xf1941000, DSI_CMD_SET_STATE_HS);
	dump->pwil_int = iris_ocp_read(0xf195ffe4, DSI_CMD_SET_STATE_HS);
	dump->dport_int = iris_ocp_read(0xf12dffe4, DSI_CMD_SET_STATE_HS);
	if (dump->sw_pwr_ctrl & 0x10) {
		dump->fi_debugbus[0] = iris_ocp_read(0xf219ffd8, DSI_CMD_SET_STATE_HS);
		dump->fi_debugbus[1] = iris_ocp_read(0xf219ffd8, DSI_CMD_SET_STATE_HS);
		dump->fi_debugbus[2] = iris_ocp_read(0xf219ffd8, DSI_CMD_SET_STATE_HS);
	}

	IRIS_LOGI("sw_pwr_ctrl: %08x, rx_video_meta: %08x", dump->sw_pwr_ctrl, dump->rx_video_meta);
	IRIS_LOGI("rx_frame_cnt0: %08x, rx_frame_cnt1: %08x", dump->rx_frame_cnt0, dump->rx_frame_cnt1);
	IRIS_LOGI("pwil_cur_meta0: %08x, pwil_status: %08x", dump->pwil_cur_meta0, dump->pwil_status);
	IRIS_LOGI("pwil_disp_ctrl0: %08x, pwil_int: %08x", dump->pwil_disp_ctrl0, dump->pwil_int);
	IRIS_LOGI("dport_int: %08x, fi_debugbus[0]: %08x", dump->dport_int, dump->fi_debugbus[0]);
	IRIS_LOGI("fi_debugbus[1]: %08x, fi_debugbus[2]: %08x", dump->fi_debugbus[1], dump->fi_debugbus[2]);
}

void iris_memc_ctrl_cmd_proc_i7p(u32 cmd)
{
	switch (cmd) {
	case MEMC_CTRL_FRC_PREPARE:
		iris_memc_ctrl_frc_prepare_i7p();
		IRIS_LOGI("MEMC_CTRL_FRC_PREPARE");
		break;
	case MEMC_CTRL_PT2FRC:
		iris_mcu_version_read_i7p();
		iris_memc_ctrl_pt_to_frc_i7p();
		IRIS_LOGI("MEMC_CTRL_PT2FRC");
		break;
	case MEMC_CTRL_FRC_POST:
		iris_memc_ctrl_frc_post_i7p();
		IRIS_LOGI("MEMC_CTRL_FRC_POST");
		break;
	case MEMC_CTRL_PT_PREPARE:
		iris_memc_ctrl_frc_to_pt_i7p();
		IRIS_LOGI("MEMC_CTRL_PT_PREPARE");
		break;
	case MEMC_CTRL_FRC2PT:
		iris_memc_ctrl_pt_post_i7p();
		IRIS_LOGI("MEMC_CTRL_FRC2PT");
		break;
	case MEMC_CTRL_PT2RFB:
		iris_memc_ctrl_rfb_prepare_i7p();
		iris_memc_ctrl_pt_rfb_meta_set_i7p(true);
		IRIS_LOGI("MEMC_CTRL_PT2RFB");
		break;
	case MEMC_CTRL_RFB2PT:
		iris_memc_ctrl_pt_rfb_meta_set_i7p(false);
		iris_memc_ctrl_pt_post_i7p();
		IRIS_LOGI("MEMC_CTRL_RFB2PT");
		break;
	case MEMC_CTRL_PT_POST:
		iris_memc_ctrl_pt_post_i7p();
		IRIS_LOGI("MEMC_CTRL_PT_POST");
		break;
	case MEMC_CTRL_DPORT_DISABLE:
		iris_dport_output_mode_select_i7p(false);
		IRIS_LOGI("MEMC_CTRL_DPORT_DISABLE");
		break;
	case MEMC_CTRL_DPORT_ENABLE:
		iris_dport_output_mode_select_i7p(true);
		IRIS_LOGI("MEMC_CTRL_DPORT_ENABLE");
		break;
	case MEMC_CTRL_VFR_DISABLE:
		iris_memc_ctrl_vfr_disable_i7p();
		IRIS_LOGI("MEMC_CTRL_VFR_DISABLE");
		break;
	case MEMC_CTRL_SWITCH_TIMEOUT:
		iris_switch_timeout_dump_i7p();
		IRIS_LOGI("MEMC_CTRL_SWITCH_TIMEOUT");
	default:
		break;
	}
}

int iris_debug_memc_option_get_i7p(char *kbuf, int size)
{
	int len = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_FRC_VFR_OW, "memc_vfr_enable_ow", debug_frc_vfr_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_DIS_FRC_DYNEN, "memc_disable_frc_dynen", debug_disable_frc_dynen);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_MCU_ENABLE, "memc_mcu_enable", iris_mcu_enable);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_DIS_MCU_CHECK, "memc_disable_mcu_check", debug_disable_mcu_check);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_SWITCH_DUMP_CLEAR, "memc_switch_dump_clear", debug_switch_dump_clear);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_FI_DROP_FRM_THR, "memc_fi_drop_frm_thr", iris_fi_drop_frm_thr);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_LOW_LATENCY_OW, "memc_low_latency_ow", debug_low_latency_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_3_BUF_LOW_LATENCY, "memc_3buffer_low_latency",
			iris_three_buffer_low_latency);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_MV_RES_OW, "memc_mv_res_ow", debug_mv_res_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_N2M_MODE_OW, "memc_n2m_mode_ow", debug_n2m_mode_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_FRC_FORCE_REPEAT, "memc_force_repeat", debug_frc_force_repeat);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_OCP_READ_BY_I2C, "ocp_read_by_i2c", pcfg->ocp_read_by_i2c);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_DSPP_PQ_DISABLE, "dspp_pq_disable", iris_pq_disable);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_FRC_DSC_OW, "raw_cmd_enable", debug_raw_cmd_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_EMV_DEC_ADJ, "memc_emv_dec_adj", debug_emv_dec_adj);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", DEBUG_EMV_LOCAL_FALLBACK, "memc_emv_local_fallback",
			debug_emv_local_fallback);

	return len;
}

void iris_debug_memc_option_set_i7p(u32 type, u32 value)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	switch (type) {
	case DEBUG_FRC_VFR_OW:
		debug_frc_vfr_overwrite = value;
		break;
	case DEBUG_DIS_FRC_DYNEN:
		debug_disable_frc_dynen = value;
		break;
	case DEBUG_MCU_ENABLE:
		iris_mcu_enable = value;
		break;
	case DEBUG_DIS_MCU_CHECK:
		debug_disable_mcu_check = value;
		break;
	case DEBUG_SWITCH_DUMP_CLEAR:
		pcfg->switch_dump.trigger = false;
		debug_switch_dump_clear = value;
		break;
	case DEBUG_FI_DROP_FRM_THR:
		iris_fi_drop_frm_thr = value;
		break;
	case DEBUG_LOW_LATENCY_OW:
		debug_low_latency_overwrite = value;
		break;
	case DEBUG_3_BUF_LOW_LATENCY:
		iris_three_buffer_low_latency = value;
		break;
	case DEBUG_MV_RES_OW:
		debug_mv_res_overwrite = value;
		break;
	case DEBUG_N2M_MODE_OW:
		debug_n2m_mode_overwrite = value;
		break;
	case DEBUG_FRC_FORCE_REPEAT:
		debug_frc_force_repeat = value;
		break;
	case DEBUG_OCP_READ_BY_I2C:
		pcfg->ocp_read_by_i2c = value;
		break;
	case DEBUG_DSPP_PQ_DISABLE:
		iris_pq_disable = value;
		break;
	case DEBUG_FRC_DSC_OW:
		debug_raw_cmd_overwrite = value;
		break;
	case DEBUG_EMV_DEC_ADJ:
		debug_emv_dec_adj = value;
		break;
	case DEBUG_EMV_LOCAL_FALLBACK:
		debug_emv_local_fallback = value;
		break;
	default:
		break;
	}
}

int iris_dbgfs_memc_init_i7p(struct dsi_display *display)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("memc_vfr_enable_ow", 0644, pcfg->dbg_root,
		(u32 *)&debug_frc_vfr_overwrite);
	debugfs_create_u32("memc_disable_frc_dynen", 0644, pcfg->dbg_root,
		(u32 *)&debug_disable_frc_dynen);
	debugfs_create_u32("memc_mcu_enable", 0644, pcfg->dbg_root,
			(u32 *)&iris_mcu_enable);
	debugfs_create_u32("memc_disable_mcu_check", 0644, pcfg->dbg_root,
			(u32 *)&debug_disable_mcu_check);
	debugfs_create_u32("memc_switch_dump_clear", 0644, pcfg->dbg_root,
			(u32 *)&debug_switch_dump_clear);
	debugfs_create_u32("memc_layer_b_enable", 0644, pcfg->dbg_root,
			(u32 *)&iris_frc_layer_b_enable);
	debugfs_create_u32("memc_layer_c_enable", 0644, pcfg->dbg_root,
			(u32 *)&iris_frc_layer_c_enable);
	debugfs_create_u32("memc_fi_drop_frm_thr", 0644, pcfg->dbg_root,
			(u32 *)&iris_fi_drop_frm_thr);
	debugfs_create_u32("memc_low_latency_ow", 0644, pcfg->dbg_root,
			(u32 *)&debug_low_latency_overwrite);
	debugfs_create_u32("memc_3buffer_low_latency", 0644, pcfg->dbg_root,
			(u32 *)&iris_three_buffer_low_latency);
	debugfs_create_u32("memc_mv_res_ow", 0644, pcfg->dbg_root,
			(u32 *)&debug_mv_res_overwrite);
	debugfs_create_u32("memc_n2m_mode_ow", 0644, pcfg->dbg_root,
		(u32 *)&debug_n2m_mode_overwrite);
	debugfs_create_u32("memc_force_repeat", 0644, pcfg->dbg_root,
		(u32 *)&debug_frc_force_repeat);
	debugfs_create_u32("memc_eco_overwrite", 0644, pcfg->dbg_root,
		(u32 *)&debug_eco_overwrite);
	debugfs_create_u32("dspp_pq_disable", 0644, pcfg->dbg_root,
		(u32 *)&iris_pq_disable);
	debugfs_create_u32("raw_cmd_enable", 0644, pcfg->dbg_root,
		(u32 *)&debug_raw_cmd_overwrite);
	debugfs_create_u32("memc_emv_dec_adj", 0644, pcfg->dbg_root,
		(u32 *)&debug_emv_dec_adj);
	debugfs_create_u32("memc_emv_local_fallback", 0644, pcfg->dbg_root,
		(u32 *)&debug_emv_local_fallback);

	return 0;
}

u32 iris_pwil_mode_state_get_i7p(void)
{
	u32 rc = 0;
	bool frc_idle = false;
	struct iris_cfg *pcfg = iris_get_cfg();

	rc = iris_ocp_read(IRIS_PWIL_ADDR + PWIL_STATUS, DSI_CMD_SET_STATE_HS);
	IRIS_LOGI("IRIS_PWIL_STATUS= %x", rc);
	frc_idle = (rc >> 28) & 0x01;
	rc = (rc >> 5) & 0x3f;
	if (rc == 8)
		pcfg->pwil_mode = FRC_MODE;
	else if (rc == 2) {
		if (frc_idle)
			pcfg->pwil_mode = PT_MODE;
		else
			rc = 8; /* if frc is not idle, iris is still in FRC */
	} else if (rc == 4) {
		pcfg->pwil_mode = RFB_MODE;
	}

	return rc;
}

void iris_kernel_status_get_i7p(u32 get_op, u32 count, u32 *values)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	switch (get_op) {
	case GET_IRIS_PWIL_MODE_STATE:
		mutex_lock(&pcfg->panel->panel_lock);
		pcfg->iris_pwil_mode_state = iris_pwil_mode_state_get_i7p();
		mutex_unlock(&pcfg->panel->panel_lock);
		*values = pcfg->iris_pwil_mode_state;

		IRIS_LOGI("GET_IRIS_PWIL_MODE_STATE: %d", *values);
		break;
	case GET_AP_DISP_MODE_STATE:
		if (iris_is_curmode_cmd_mode() &&
			pcfg->rx_mode == DSI_OP_CMD_MODE)
			*values = 1;
		else if (iris_is_curmode_vid_mode() &&
			pcfg->rx_mode == DSI_OP_VIDEO_MODE)
			*values = 2;
		else
			*values = 0;
		IRIS_LOGI("GET_AP_DISP_MODE_STATE: %d", *values);
		break;
	case GET_FRC_VFR_STATUS:
		*values = pcfg->memc_info.vfr_en;
		/* disable VFR when frc2pt */
		pcfg->memc_info.vfr_en = false;
		IRIS_LOGI("GET_FRC_VFR_STATUS: %d", *values);
		break;
	case GET_MV_RESOLUTION:
		values[0] = pcfg->frc_setting.init_single_mv_hres;
		values[1] = pcfg->frc_setting.init_single_mv_vres;
		values[2] = pcfg->frc_setting.init_single_mv_hres;
		values[3] = pcfg->frc_setting.init_single_mv_vres;
		IRIS_LOGI("GET_MV_RESOLUTION: %d", count);
		break;
	case GET_AP_DSI_CLOCK_STATUS:
		IRIS_LOGI("%s: cached_clk_rate:%u, clk_rate_hz:%u, expect clk_rate_hz: %u.", __func__,
					pcfg->display->cached_clk_rate,
					pcfg->panel->cur_mode->timing.clk_rate_hz, pcfg->display->dyn_bit_clk);
		if (pcfg->display->cached_clk_rate == pcfg->display->dyn_bit_clk)
			*values = 1;
		else
			*values = 0;
		break;
	default:
		break;
	}
}

void iris_debug_info_get_i7p(u32 *value, u32 count)
{
	int len = 0;
	char *buf = (char *)value;

	count *= 4;
	switch (value[0]) {
	case 0:
		len = iris_debug_display_info_get(buf, count);
		break;
	case 1:
		len = iris_debug_display_mode_get(buf, count, true);
		break;
	case 2:
		len = iris_debug_memc_option_get_i7p(buf, count);
		break;
	default:
		break;
	}
	if (len < count)
		buf[len] = '\0';
}

int iris_configure_memc_i7p(u32 type, u32 value)
{
	int rc = 0;

	switch (type) {
	case IRIS_MEMC_CTRL:
		iris_memc_ctrl_cmd_proc_i7p(value);
		break;
	case IRIS_MEMC_OSD:
		iris_dspp_pq_mode_set_i7p(value);
		break;
	case USER_DEMO_WND:
		iris_fi_demo_window_set_i7p(value);
		break;
	case IRIS_N2M_ENABLE:
		if (value > 1) {
			IRIS_LOGI("%s(), enable N2M function in PT mode", __func__);
			iris_set_n2m_enable_i7p(true, value);
			iris_dtg_te_n2m_ctrl_setting_send_i7p(true);
		}
		if (value == 0) {
			IRIS_LOGI("%s(), disable N2M function in PT mode", __func__);
			iris_set_n2m_enable_i7p(false, 1);
			iris_dtg_te_n2m_ctrl_setting_send_i7p(false);
		}
		break;
	default:
		IRIS_LOGI("%s: type = %d value = %d", __func__, type, value);
		break;
	}

	return rc;
}

int iris_configure_ex_memc_i7p(u32 type, u32 count, u32 *values)
{
	int rc = 0;

	switch (type) {
	case IRIS_MEMC_INFO_SET:
		iris_memc_info_set_i7p(values);
		break;
	case IRIS_DEBUG_SET:
		iris_debug_memc_option_set_i7p(values[0], values[1]);
		break;
	default:
		IRIS_LOGI("%s: type = %d count = %d", __func__, type, count);
		break;
	}

	return rc;
}

int iris_configure_get_memc_i7p(u32 type, u32 count, u32 *values)
{
	int rc = 0;
	u32 get_op = 0;

	switch (type) {
	case IRIS_KERNEL_STATUS_GET:
		get_op = *values;
		iris_kernel_status_get_i7p(get_op, count, values);
		break;
	case IRIS_DEBUG_GET:
		iris_debug_info_get_i7p(values, count);
		break;
	default:
		IRIS_LOGI("%s: type = %d count = %d", __func__, type, count);
		break;
	}

	return rc;
}

void iris_init_memc_i7p(void)
{
	//iris_frc_setting_init_i7p();
	iris_memc_status_clear_i7p(true);
}

void iris_lightoff_memc_i7p(void)
{
	iris_memc_status_clear_i7p(true);

	iris_pwil0_efifo_setting_reset_i7p();
	iris_pwil_datapath_reset_i7p(true);
	iris_dport_output_mode_reset_i7p();
	iris_set_pwil_mode_i7p(PT_MODE, DSI_CMD_SET_STATE_HS, false);
}

void iris_dsi_rx_mode_switch_i7p(u8 rx_mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;
	u32 ovs_dly_rfb;
	u32 *payload = NULL;

	IRIS_LOGI("%s: %d", __func__, rx_mode);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 3);
	if (rx_mode == DSI_OP_CMD_MODE)
		payload[0] |= 0x20001;
	else
		payload[0] &= ~0x20001;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x01, 0x01, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x04, 3);
	if (rx_mode == DSI_OP_CMD_MODE)
		payload[0] &= ~0x4000;
	else
		payload[0] |= 0x4000;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x04, 0x04, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x10, 2);
	if (rx_mode == DSI_OP_CMD_MODE)
		payload[0] &= ~0x800;
	else
		payload[0] |= 0x800;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x10, 0x10, 1);
	iris_dma_trig(DMA_CH9, 1);

	regval.ip = IRIS_IP_DTG;
	regval.opt_id = ID_DTG_TE_SEL;
	regval.mask = 0x0000001C;
	regval.value = ((rx_mode == DSI_OP_CMD_MODE) ? 0x00000014 : 0x00000000);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);

	if (rx_mode == DSI_OP_CMD_MODE)
		iris_init_update_ipopt_t(IRIS_IP_DTG, 0xf3, 0xf3, 0x01);
	else {
		payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0xf5, 2);
		ovs_dly_rfb = payload[0];
		payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0xf8, 2);
		payload[3] = ovs_dly_rfb;
		iris_init_update_ipopt_t(IRIS_IP_DTG, 0xf8, 0xf8, 0x01);
	}

	regval.ip = IRIS_IP_DTG;
	regval.opt_id = 0xF0;
	regval.mask = 0x0000000F;
	regval.value = 0x2;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DTG, 0xF0, 0xF0, 0);
	iris_update_pq_opt(PATH_DSI, true);

	pcfg->rx_mode = rx_mode;
}

void iris_memc_func_init_i7p(struct iris_memc_func *memc_func)
{
	memc_func->register_osd_irq = NULL;
	memc_func->update_panel_ap_te = iris_update_panel_ap_te_i7p;
	memc_func->inc_osd_irq_cnt = NULL;
	memc_func->is_display1_autorefresh_enabled = NULL;
	memc_func->pt_sr_set = NULL;
	memc_func->configure_memc = iris_configure_memc_i7p;
	memc_func->configure_ex_memc = iris_configure_ex_memc_i7p;
	memc_func->configure_get_memc = iris_configure_get_memc_i7p;
	memc_func->init_memc = iris_init_memc_i7p;
	memc_func->lightoff_memc = iris_lightoff_memc_i7p;
	memc_func->enable_memc = NULL;
	memc_func->sr_update = NULL;
	memc_func->frc_setting_init = iris_frc_setting_init_i7p;
	memc_func->dbgfs_memc_init = iris_dbgfs_memc_init_i7p;
	memc_func->parse_memc_param0 = iris_parse_memc_param0_i7p;
	memc_func->parse_memc_param1 = iris_parse_memc_param1_i7p;
	memc_func->frc_timing_setting_update = iris_frc_timing_setting_update_i7p;
	memc_func->pt_sr_reset = NULL;
	memc_func->mcu_state_set = iris_mcu_state_set_i7p;
	memc_func->mcu_ctrl_set = NULL;
	memc_func->memc_vfr_video_update_monitor = NULL;
	memc_func->low_latency_mode_get = iris_low_latency_mode_get_i7p;
	memc_func->health_care = NULL;
	memc_func->dsi_rx_mode_switch = iris_dsi_rx_mode_switch_i7p;
}
