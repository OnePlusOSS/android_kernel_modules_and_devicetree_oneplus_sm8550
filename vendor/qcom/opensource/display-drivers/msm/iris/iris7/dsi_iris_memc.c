
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
#include "dsi_iris_reg_i7.h"
#include "dsi_iris_memc.h"
#include "dsi_iris_dual_i7.h"
#include "dsi_iris_memc_i7.h"
#include "dsi_iris_emv_i7.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_i3c.h"
#include "dsi_iris_dts_fw.h"

static u32 debug_memc_level_overwrite;
static u32 judder_angle = 150;
static u32 iris_sr_enable = 1;
static u32 debug_disable_vfr_dual = false;
static u32 debug_disable_frc_dynen = false;
static u32 debug_frc_force_repeat = false;
static u32 iris_frc_layer_b_enable;
static u32 iris_frc_layer_c_enable = 1;
static u32 iris_mcu_enable = 1;
static u32 iris_mcu_ctrl;
static u32 iris_mcu_ctrl_org;
static u32 debug_disable_mcu_check;
static u32 debug_switch_dump_clear = 0;
static u32 iris_fi_drop_frm_thr = 3;  //0: means disable vfr, others: means enable vfr.
static int iris_frc_dma_disable = 1;
static u32 val_frcc_reg3 = 0x00060003;
static struct dsc_encoder_ctrl_reg enc_ctrl_reg;
static struct dsc_info frc_dsc_info;
static u32 dsc_pps_table[22];
extern u8 iris_pq_update_path;
static u32 debug_low_latency_overwrite = 0;
static u32 debug_tnr_en_overwrite = 0;
static u32 debug_frc_vfr_overwrite = 0;
static u32 debug_n2m_mode_overwrite = 0;
static u32 debug_mv_res_overwrite = 0;
static u32 debug_eco_overwrite;
static u32 debug_disable_mipi1_autorefresh;

static u32 iris_three_buffer_low_latency;
static u32 iris_frc_dec_initial_delay;
static int iris_memc_cmd_payload_count;
static u32 *iris_memc_cmd_payload;
#define IRIS_MEMC_CMD_PAYLOAD_LENGTH	300	// 128 registers
static u32 iris_mcu_mode = MCU_INT_DISABLE;
static bool iris_mcu_stop;

void iris_memc_info_set_i7(u32 *values)
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

int iris_low_latency_mode_get_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return pcfg->memc_info.low_latency_mode;
}

int iris_tnr_mode_get_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return pcfg->memc_info.tnr_en;
}

void iris_dtg_eco_i7(bool enable, bool chain)
{
	u32 *payload = NULL;

	IRIS_LOGI("%s: %d", __func__, enable);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x90, 2);
	if (enable)
		payload[0] |= 0x800;
	else
		payload[0] &= ~0x800;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x90, 0x90, 1);
	if (chain) {
		iris_dma_trig(DMA_CH12, 0);
		iris_update_pq_opt(PATH_DSI, true);
	}
}

u32 iris_disable_mipi1_autorefresh_get_i7(void)
{
	return debug_disable_mipi1_autorefresh;
}

void iris_memc_cmd_payload_init_i7(void)
{
	iris_memc_cmd_payload_count = 0;
	if (iris_memc_cmd_payload == NULL)
		iris_memc_cmd_payload = vmalloc(IRIS_MEMC_CMD_PAYLOAD_LENGTH * sizeof(u32));
	if (iris_memc_cmd_payload == NULL)
		IRIS_LOGE("can not vmalloc size = %d in %s", IRIS_MEMC_CMD_PAYLOAD_LENGTH, __func__);
}

void iris_memc_cmd_payload_send_i7(void)
{
	if (iris_memc_cmd_payload != NULL && iris_memc_cmd_payload_count != 0) {
		IRIS_LOGI("iris_memc_cmd_payload_count: %d", iris_memc_cmd_payload_count);
		iris_ocp_write_mult_vals(iris_memc_cmd_payload_count, iris_memc_cmd_payload);
	}

	if (iris_memc_cmd_payload) {
		vfree(iris_memc_cmd_payload);
		iris_memc_cmd_payload = NULL;
	}
}

u32 iris_disp_vtotal_get_i7(void)
{
	uint32_t *payload;
	uint32_t vtotal;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0x00, 2);
	if (!payload)
		return 0;
	vtotal = payload[4] + payload[5] + payload[6]  + payload[7];

	return vtotal;
}

u32 iris_frc_video_hstride_calc_i7(u16 hres, u16 bpp, u16 slice_num)
{
	u32 hstride;

	hstride = bpp * ((hres + slice_num - 1) / slice_num);
	hstride = (hstride + 15) >> 4;
	hstride = ((hstride  + 7) / 8) * 8;
	hstride = (hstride + 63) / 64;

	return hstride;
}

bool iris_frc_reg_is_external_i7(u32 addr)
{
	if (addr >= 0xf1941000 && addr <= 0xf1941098)
		return true;
	if (addr >= 0xf1981000 && addr <= 0xf1981010)
		return true;
	if (addr >= 0xf12d0000 && addr <= 0xf12d001c)
		return true;

	return false;
}

void iris_frc_reg_add_i7(u32 addr, u32 val, bool last)
{
	u32 *payload = &iris_memc_cmd_payload[iris_memc_cmd_payload_count];

	if (iris_frc_reg_is_external_i7(addr))
		IRIS_LOGE("%s: Warning! external_reg: 0x%x", __func__, addr);

	if (iris_memc_cmd_payload_count < IRIS_MEMC_CMD_PAYLOAD_LENGTH) {
		*payload = addr;
		payload++;
		*payload = val;
		iris_memc_cmd_payload_count += 2;
	} else {
		IRIS_LOGE("payload buffer length is not enough! %s", __func__);
	}
}

void iris_frc_mif_reg_set_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;
	bool enableGameMode = false;

	u32 val_frcc_reg0 = 0x001b0101;
	u32 val_frcc_reg1 = 0;
	u32 val_frcc_reg2 = 0x47d5e6f2;
	u32 val_frcc_reg4 = 0x09168000;
	u32 val_frcc_reg6 = 0x00018000;
	u32 val_frcc_phase_ctrl_0 = 0x02adfc04;
	u32 val_frcc_phase_ctrl_1 = 0x00551919;
	u32 val_frcc_fk_ctrl_3 = 0x3c3e0218;
	u32 val_frcc_fk_ctrl_4 = 0x8000332d;
	u32 val_frcc_fk_ctrl_5 = 0x0051a7a8;
	u32 val_frcc_enable_0 = 0x0025d49f;
	u32 val_frcc_enable_1 = 0x90db1000;
	u32 val_frcc_reg10 = 0x29101f;
	u32 val_eco_reserved_ctrl = 0x0000080c;
	u32 val_frc_ds_ctrl = 0x000a019e;
	u32 val_frcc_mvc = 0x00000058;

	u16 fmif_vd_hstride,  fmif_mv_hstride;
	u32 fmif_vd_offset, fmif_mv_frm_offset;
	u16 mv_hres, mv_vres;
	u32 disp_vtotal = iris_disp_vtotal_get_i7();

	u32 fmif_a_mv_hstride, fmif_c_mv_hstride, fmif_a_mv_frm_offset, fmif_c_mv_frm_offset;
	u32 mvc_gen_cnt_thr, rgme_calc_cnt_thr, fi_meta_fast_entry_thr;
	u32 fmif_rgme_hstride, fmif_rgme_frm_offset;
	u32 memc_level = 3;

	enableGameMode = (pcfg->memc_info.memc_mode == MEMC_DUAL_VIDEO_ENABLE);
	enableGameMode &= (pcfg->memc_info.low_latency_mode == ULTRA_LT_MODE);
	enableGameMode |= (pcfg->memc_info.memc_mode == MEMC_SINGLE_GAME_ENABLE);
	enableGameMode |= (pcfg->memc_info.memc_mode == MEMC_DUAL_GAME_ENABLE);

	fmif_vd_hstride = iris_frc_video_hstride_calc_i7(frc_setting->mv_hres, frc_setting->mv_coef, 1);
	fmif_vd_offset = fmif_vd_hstride * frc_setting->mv_vres * 8;
	fmif_mv_hstride = ((frc_setting->mv_hres + 15) / 16 * 32 + 63) / 64;
	fmif_mv_frm_offset = fmif_mv_hstride * CEILING(frc_setting->mv_vres, 16) * 8;
	mv_hres = CEILING(frc_setting->mv_hres, 16);
	mv_vres = CEILING(frc_setting->mv_vres, 16);

	fmif_rgme_hstride =   CEILING(CEILING(frc_setting->mv_hres, 4)*8, 64);
	fmif_rgme_frm_offset = fmif_rgme_hstride * CEILING(frc_setting->mv_vres, 8)*8;
	fmif_a_mv_hstride = ((frc_setting->mv_hres + 15) / 16 * 48 + 63) / 64;
	fmif_c_mv_hstride = ((frc_setting->mv_hres + 7) / 8 * 48 + 63) / 64;
	fmif_a_mv_frm_offset = fmif_a_mv_hstride*((frc_setting->mv_vres + 15)/16)*8;
	fmif_c_mv_frm_offset = fmif_c_mv_hstride*((frc_setting->mv_vres + 7)/8)*8;

	mvc_gen_cnt_thr = frc_setting->mv_vres >> 2;
	rgme_calc_cnt_thr = mvc_gen_cnt_thr + 5;
	fi_meta_fast_entry_thr = mvc_gen_cnt_thr + 2;

	val_frcc_reg1 = (mvc_gen_cnt_thr << 16);
	val_frcc_reg4 |= rgme_calc_cnt_thr;
	val_frcc_reg6 |= fi_meta_fast_entry_thr;
	//if (pcfg->rx_mode == DSI_OP_VIDEO_MODE)
	//	val_frcc_reg2 &= ~0x1000000;

	/*frc_ds*/
	/* low latency mode is ultra_lt_mode or tnr mode, using two video buffer, disable PPC_EN and PPC_OUT_EN */
	if (iris_tnr_mode_get_i7())
		val_frc_ds_ctrl = val_frc_ds_ctrl & (~0x00000018);
	else if ((iris_low_latency_mode_get_i7() == ULTRA_LT_MODE) && (!iris_three_buffer_low_latency))
		val_frc_ds_ctrl = val_frc_ds_ctrl & (~0x00000018);
	iris_frc_reg_add_i7(IRIS_FRC_DS_ADDR + FRC_DS_CTRL, val_frc_ds_ctrl, 0);
	iris_frc_reg_add_i7(IRIS_FRC_DS_ADDR + FRC_DS_RESO, frc_setting->mv_hres | (frc_setting->mv_vres << 16), 0);
	iris_frc_reg_add_i7(IRIS_FRC_DS_ADDR + FRC_DS_SHDW, 0x00000003, 0);

	/*rgme*/
	iris_frc_reg_add_i7(IRIS_RGME_ADDR + RGME_MIF_CTRL, 0x04301832, 0);
	iris_frc_reg_add_i7(IRIS_RGME_ADDR + FRAME_RES,
		((frc_setting->mv_hres + 3)/4) | (((frc_setting->mv_vres + 7)/8) << 16), 0);
	iris_frc_reg_add_i7(IRIS_RGME_ADDR + DSY_HSTRIDE, fmif_rgme_hstride, 0);
	iris_frc_reg_add_i7(IRIS_RGME_ADDR + VD_FM_OFFSET, fmif_rgme_frm_offset, 0);
	iris_frc_reg_add_i7(IRIS_RGME_ADDR + FRAME_BUFFER_0,
		frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num, 0);

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

	if (iris_low_latency_mode_get_i7() == ULTRA_LT_MODE) {
		/* two buffer mode, disable REP_FRM_DET_EN and MVC_PPC_EN */
		if (!iris_three_buffer_low_latency)
			val_frcc_enable_0 &= (~0x00000480);
		val_frcc_reg1 = 0x00280000;
		val_frcc_phase_ctrl_0 = 0x042dfc04;
		val_frcc_phase_ctrl_1 = 0x00511919;
		val_frcc_fk_ctrl_4 = (val_frcc_fk_ctrl_4 & (~0x0003c000)) | (4 << 14);
	} else if ((iris_low_latency_mode_get_i7() == LT_MODE) ||
			(iris_low_latency_mode_get_i7() == NORMAL_LT)) {
		val_frcc_enable_0 |= ((pcfg->frc_setting.layer_c_en << 9) |
							(iris_frc_layer_b_enable << 8));
	} else if (iris_tnr_mode_get_i7()) {
		val_frcc_enable_0 |= ((iris_frc_layer_b_enable << 8) |
							(pcfg->frc_setting.frc_dynen << 13) |
							(1 << 5));
		val_frcc_enable_0 &= (~0x00000480);
	} else {
		val_frcc_enable_0 |= ((pcfg->frc_setting.layer_c_en << 9) |
							(iris_frc_layer_b_enable << 8) |
							(pcfg->frc_setting.frc_dynen << 13));
	}

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_ENABLE_0, val_frcc_enable_0, 0);
	if (iris_tnr_mode_get_i7())
		val_frcc_reg0 = (val_frcc_reg0 & (~0x00070000)) | (2 << 16);
	else if ((iris_low_latency_mode_get_i7() == ULTRA_LT_MODE)
		&& (!iris_three_buffer_low_latency))
		val_frcc_reg0 = (val_frcc_reg0 & (~0x00070000)) | (2 << 16);
	if (pcfg->memc_info.n2m_mode == 1 || pcfg->memc_info.n2m_mode == 2)
		val_frcc_reg0 = (val_frcc_reg0 & (~0x0000ffff)) | (2 << 8) | 1;

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG0, val_frcc_reg0, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG1, val_frcc_reg1, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_PHASE_CTRL_0, val_frcc_phase_ctrl_0, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_PHASE_CTRL_1, val_frcc_phase_ctrl_1, 0);

	if (pcfg->rx_mode == DSI_OP_VIDEO_MODE)
		val_frcc_enable_1 |= 0x200;	// enable FI_MIF_LOCK_EN
	else if (pcfg->rx_mode == DSI_OP_CMD_MODE && pcfg->tx_mode == DSI_OP_VIDEO_MODE)
		val_frcc_enable_1 |= 0x200;	// enable FI_MIF_LOCK_EN
	if (iris_low_latency_mode_get_i7() == ULTRA_LT_MODE)
		val_frcc_enable_1 |= 0x804;	// enable FI_FAST_ENTRY & PHASE_FWD_EN
	else if (iris_low_latency_mode_get_i7() == LT_MODE)
		val_frcc_enable_1 |= 0x800; // only enable PHASE_FWD_EN
	if (iris_low_latency_mode_get_i7() == ULTRA_LT_MODE &&
		(pcfg->memc_info.n2m_mode == 1 || pcfg->memc_info.n2m_mode == 2)) {
		val_frcc_enable_1 = (val_frcc_enable_1 & (~0x00002000)) | (1 << 13);
	}

	if (pcfg->memc_info.memc_level == 2)
		val_frcc_enable_1 |= (1 << 14);

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_ENABLE_1, val_frcc_enable_1 | (iris_tnr_mode_get_i7() << 3), 0);

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG2, val_frcc_reg2, 0);

	if (pcfg->frc_setting.frc_vfr_disp)
		val_frcc_reg3 = (val_frcc_reg3 & 0x0fffffff) | ((iris_fi_drop_frm_thr & 0xf) << 28);
	else
		val_frcc_reg3 = val_frcc_reg3 & 0x0fffffff;

	if (pcfg->memc_info.memc_level == 2) {
		val_frcc_reg4 &= ~0x1ff000;
		val_frcc_reg4 |= (judder_angle << 12);
	}
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG3, val_frcc_reg3, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG4, val_frcc_reg4, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG6, val_frcc_reg6, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_FK_CTRL_3, val_frcc_fk_ctrl_3, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_FK_CTRL_4,
					val_frcc_fk_ctrl_4 | (memc_level << 6), 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_FK_CTRL_5, val_frcc_fk_ctrl_5, 0);
	if ((frc_setting->mv_hres > 96) && (frc_setting->mv_vres > 384))
		val_frcc_reg10 &= (~0x00000010);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG10, val_frcc_reg10, 0);

	if (iris_low_latency_mode_get_i7() == NORMAL_LT || iris_low_latency_mode_get_i7() == LT_MODE)
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MVC_CTRL_0, 0x8, 0);
	else if (iris_low_latency_mode_get_i7() == ULTRA_LT_MODE)
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MVC_CTRL_0, 0xd8, 0);
	else
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MVC_CTRL_0, val_frcc_mvc, 0);

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MVF_CTRL_0, (frc_setting->force_repeat << 1), 0);

	if (pcfg->rx_mode == DSI_OP_VIDEO_MODE && pcfg->tx_mode == DSI_OP_VIDEO_MODE)
		val_eco_reserved_ctrl = val_eco_reserved_ctrl | 0x000000d0;
	if (iris_low_latency_mode_get_i7() == ULTRA_LT_MODE) {
		if (debug_eco_overwrite == 1 && enableGameMode)
			val_eco_reserved_ctrl = val_eco_reserved_ctrl | 0x00000610;
		else
			val_eco_reserved_ctrl = val_eco_reserved_ctrl | 0x00000600;
	} else if (iris_tnr_mode_get_i7() && (pcfg->memc_info.panel_fps == 120))
		val_eco_reserved_ctrl = val_eco_reserved_ctrl | 0x00000010;
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_ECO_REV_CTRL, val_eco_reserved_ctrl, 0);

	//film border init value
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_FBD_CTRL_0,
		(frc_setting->mv_vres << 16) | frc_setting->mv_hres, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_FBD_CTRL_1, frc_setting->mv_hres << 16, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_FBD_CTRL_2, frc_setting->mv_vres << 16, 0);

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_VD_MEM_ADDR, frc_setting->video_baseaddr, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_TNR_VD_BUF_SIZE, frc_setting->video_baseaddr, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_VD_MEM_OFF, fmif_vd_offset, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_VD_MEM_HSTRIDE, fmif_vd_hstride, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A1_BSADR,
		frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
		+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A1_OFF, fmif_a_mv_frm_offset, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A1_HSTRIDE, fmif_a_mv_hstride, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A0_BSADR,
		frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
		+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
		+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A0_OFF, fmif_a_mv_frm_offset, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_A0_HSTRIDE, fmif_a_mv_hstride, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_OFF, fmif_c_mv_frm_offset, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_HSTRIDE, fmif_c_mv_hstride, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_OFF, fmif_c_mv_frm_offset, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_HSTRIDE, fmif_c_mv_hstride, 0);
	if (iris_tnr_mode_get_i7()) {
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_BSADR,
			frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
			+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
			+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_BSADR,
			frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
			+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
			+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
	} else if (iris_low_latency_mode_get_i7() == ULTRA_LT_MODE) {
		if (!iris_three_buffer_low_latency) {
			iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_BSADR,
				frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
				+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
				+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
			iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_BSADR,
				frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
				+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
				+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
		} else {
			iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_BSADR,
				frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
				+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
				+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
			iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_BSADR,
				frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
				+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
				+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2
				+ fmif_c_mv_frm_offset*frc_setting->mv_buf_num, 0);
		}
	} else if (!pcfg->frc_setting.layer_c_en) {
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_BSADR,
			frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
			+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
			+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_BSADR,
			frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
			+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
			+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
	} else {
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C1_BSADR,
			frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
			+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
			+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2, 0);
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MV_MEM_C0_BSADR,
			frc_setting->video_baseaddr + fmif_vd_offset*frc_setting->vid_buf_num
			+ fmif_rgme_frm_offset*frc_setting->rgme_buf_num
			+ fmif_a_mv_frm_offset*frc_setting->mv_buf_num*2
			+ fmif_c_mv_frm_offset*frc_setting->mv_buf_num, 0);
	}

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + IMIF_MODE_CTRL,
		0x4 + ((frc_setting->mv_vres * 3 / 4) << 16), 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + IMIF_DW_PER_LINE, fmif_vd_hstride, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + IMIF_VSIZE, (frc_setting->mv_vres), 0);

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + MMIF_CTRL1, 0x00402007, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + MMIF_CTRL2, 0x8 + (disp_vtotal << 16), 0);

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FMIF_CTRL, 0xFF004034, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FMIF_VD_FRM_ATTRIBUTE0, (0x10 << 16) + fmif_vd_hstride, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FMIF_VD_FRM_ATTRIBUTE1, (0x40 << 16) + frc_setting->mv_vres, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FMIF_MV_FRM_ATTRIBUTE0, (mv_hres << 16) + mv_vres, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FMIF_MV_FRM_ATTRIBUTE1, (0x4 << 16) + fmif_mv_hstride, 0);

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_REG_SHDW, 0x00000003, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + IMIF_SW_UPDATE_EN, 0x00000003, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + MMIF_UPDATE, 0x00000003, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FMIF_REG_SHDW, 0x00000003, 0);
}

void iris_gmd_reg_set_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;

	iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_GAIN, 0x0000C488, 0);
	iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_FILT, 0x00000000, 0);
	iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_ACCUM, 0x00000659, 0);
	iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_SHIFT, 0x00000070, 0);
	iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_START, 0x00000000, 0);
	iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_STOP,
		frc_setting->mv_hres + (frc_setting->mv_vres << 16), 0);
	/* it is only aimed at Genshin Impact */
	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_GAME_ENABLE && pcfg->memc_info.memc_app == 3) {
		iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_START_WIN, 0x00320000, 0);
		iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_STOP_WIN, ((frc_setting->mv_vres - 50) << 16) +
			(frc_setting->mv_hres), 0);
		iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_SUM_SEL, 0x00000001, 0);
	}
	iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_CTRL, 0x00000001, 0);
}

void iris_fbd_reg_set_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;

	iris_frc_reg_add_i7(IRIS_FBD_ADDR + FILMBD_RESOLUTION,
				(frc_setting->mv_hres) + ((frc_setting->mv_vres) << 16), 0);
	iris_frc_reg_add_i7(IRIS_FBD_ADDR + FILMBD_WIN_STOP_SET,
				frc_setting->mv_hres + (frc_setting->mv_vres << 16), 0);
	iris_frc_reg_add_i7(IRIS_FBD_ADDR + FILMBD_TOP_CTRL, 0x00010025, 1);

}

void iris_cad_reg_set_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_frc_reg_add_i7(IRIS_CAD_ADDR + NEW_FRM_FLG_DET_1, 0x06920311, 0);
	iris_frc_reg_add_i7(IRIS_CAD_ADDR + CAD_DET_BASIC_CAD, 0x0000010c, 0);
	iris_frc_reg_add_i7(IRIS_CAD_ADDR + CAD_DET_STVT, 0x00004083, 0);
	iris_frc_reg_add_i7(IRIS_CAD_ADDR + CAD_DET_BAD_EDIT, 0x00011255, 0);
	iris_frc_reg_add_i7(IRIS_CAD_ADDR + CAD_DET_VOF_0, 0x096823C1, 0);
	iris_frc_reg_add_i7(IRIS_CAD_ADDR + DUMMY, 0x000000F1, 0);
	if (pcfg->rx_mode == DSI_OP_VIDEO_MODE) {
		switch (pcfg->memc_info.video_fps) {
		case 24:
		case 30:
			iris_frc_reg_add_i7(IRIS_CAD_ADDR + COMMON, 0x00000079, 0);
			break;
		default:
			iris_frc_reg_add_i7(IRIS_CAD_ADDR + COMMON, 0x00000078, 0);
			break;
		}
	}

	iris_frc_reg_add_i7(IRIS_CAD_ADDR + SW_DONE, 0x00000001, 1);

}

void iris_game_memc_registers_set_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 i;

	i = 0;
	while (pcfg->memc_info.latencyValue[i]) {
		iris_frc_reg_add_i7(pcfg->memc_info.latencyValue[i], pcfg->memc_info.latencyValue[i+1], 0);
		i += 2;
	}
}

void iris_game_memc_OSD_registers_set_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 i;

	i = 0;
	while (pcfg->memc_info.OSDProtection[i]) {
		iris_frc_reg_add_i7(pcfg->memc_info.OSDProtection[i], pcfg->memc_info.OSDProtection[i+1], 0);
		i += 2;
	}
}

void iris_mvc_reg_set_i7(void)
{
	int mode = 0;

	mode = iris_low_latency_mode_get_i7();
	if (mode == NORMAL_LT || mode == LT_MODE) {
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_POSTFILT_0, 0x67b444c1, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_KFBOSD_0, 0x12429f43, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_KFBOSD_1, 0x10001000, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_OSDWIN_0, 0x21000e18, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_OSDWIN_1, 0x81024e18, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_OSDWIN_2, 0x80300018, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_TEXTFLAG_0, 0x44440440, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_REGIONMV_2, 0x4440800, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_SAD_2, 0x18ffffff, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_SW_UPDATE, 0x00000001, 0);
	} else if (mode == ULTRA_LT_MODE) {
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_POSTFILT_0, 0x67b444c1, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_KFBOSD_0, 0x12429f43, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_KFBOSD_1, 0x10001000, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_OSDWIN_0, 0x21000e1b, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_OSDWIN_1, 0x81024e1b, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_OSDWIN_2, 0x80300019, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_TEXTFLAG_0, 0x44440448, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_REGIONMV_2, 0x4440880, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_SAD_2, 0x18a8ffff, 0);
		iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_SW_UPDATE, 0x00000001, 0);
	}

	iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_INTEN, 0x00000020, 0);
	iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_INTPRI, 0x00000020, 0);
	iris_frc_reg_add_i7(IRIS_MVC_ADDR + MVC_INTCLR, 0x00000020, 0);
}

void iris_mvf_reg_set_i7(void)
{
	int mode = 0;

	mode = iris_low_latency_mode_get_i7();
	if (mode == NORMAL_LT || mode == LT_MODE) {
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_SMI_CTRL, 0x64, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_VERTIRANGE_CFG, 0x18c, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_CAND_CFG, 0x1f10882c, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + HR_MVC_CFG1, 0x8124, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + HR_MVC_CFG2, 0x90f27, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_MISC_CFG3, 0x2449, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_OSDMV, 0x428c8d2, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_OSDMV2, 0x700013f2, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_OSDMV3, 0x44444093, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_HMTNFB, 0x8818, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_SHDW_CTRL, 0x00000013, 0);
	} else if (mode == ULTRA_LT_MODE) {
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_SMI_CTRL, 0x64, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_VERTIRANGE_CFG, 0x18c, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_CAND_CFG, 0x1f10882c, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + HR_MVC_CFG1, 0x8224, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + HR_MVC_CFG2, 0x90524, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_MISC_CFG3, 0x2449, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_OSDMV, 0x488c8d2, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_OSDMV2, 0x70001102, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_OSDMV3, 0x44444093, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_HMTNFB, 0x148818, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_SHDW_CTRL, 0x00000013, 0);
	}
}

/* control mvc dsy buffer mode and buffer size */
void iris_ramctrl_mvc_dsy_ctrl_i7(void)
{
	u8 dsy_bufmode_sel = 0;
	u8 dsy_buf_8m = 0;
	u32 value;

	/* in low latency mode, the mvc_dsy ram ctrl setting is 2buffers, 8M.*/
	if (iris_tnr_mode_get_i7()) {
		dsy_bufmode_sel = 1;
		dsy_buf_8m = 1;
	} else if (iris_low_latency_mode_get_i7() == ULTRA_LT_MODE && !iris_three_buffer_low_latency) {
		dsy_bufmode_sel = 1;
		dsy_buf_8m = 1;
	}

	value = dsy_bufmode_sel | (dsy_buf_8m << 1);
	iris_frc_reg_add_i7(IRIS_SRAM_CTRL_ADDR + RAMCTRL_MVC_DSY_CTRL, value, 0);
}

void iris_blending_cusor_timeout_set_i7(bool dual_frc)
{
	struct iris_update_regval regval;

	regval.ip = IRIS_IP_BLEND;
	regval.opt_id = dual_frc ? 0x20 : 0x10;
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);

	iris_dma_trig(DMA_CH13, 0);
	iris_update_pq_opt(PATH_DSI, true);
}

u32 iris_fi_demo_win_color_get_i7(void)
{
	u32 val = 0x51ef5a;
	struct iris_cfg *pcfg = iris_get_cfg();

	switch (pcfg->memc_info.memc_mode) {
	case MEMC_SINGLE_VIDEO_ENABLE:
		if (pcfg->memc_info.tnr_en)
			val = 0xa934a5;
		else
			val = 0x51ef5a;  /* red */
		break;
	case MEMC_DUAL_VIDEO_ENABLE:
		val = 0x902235;  /* green */
		break;
	case MEMC_DUAL_GAME_ENABLE:
		val = 0xceddca;  /* magenta */
		break;
	case MEMC_SINGLE_GAME_ENABLE:
		val = 0xd29210;  /* yellow */
		break;
	case MEMC_DUAL_EXTMV_ENABLE:
		val = 0x286def;  /* blue */
		break;
	default:
		break;
	}

	return val;
}

void iris_fi_reg_set_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;
	u32 max_search_range, val_range_ctrl;
	u32 vrange_top, vrange_bot;
	u32 hres = (frc_setting->mv_hres / 4) * 4;
	u32 demo_win_yuv = 0x51ef5a;

	if (frc_setting->mv_hres % 4)
		hres += 4;
	max_search_range = 0x20000 / hres - 4;
	if (max_search_range > 510)
		max_search_range = 510;
	vrange_top = max_search_range / 2 - 1;
	vrange_bot = max_search_range / 2;
	// vrange_bot should be odd number
	if (vrange_bot%2 == 0) {
		vrange_bot -= 1;
		vrange_top -= 1;
	}
	//in FPGA, the frc solution is very small, so the range will use a small value.
	//TODO
	vrange_top = 70;
	vrange_bot = 71;
	val_range_ctrl = vrange_top + (vrange_bot << 8);
	iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_RANGE_CTRL, val_range_ctrl, 0);
	iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_MODE_CTRL, 0x00004020 | pcfg->frc_label << 7
			| (pcfg->frc_demo_window ? 3 : 0), 0);

	demo_win_yuv = iris_fi_demo_win_color_get_i7();
	iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_YUV_VALUE, demo_win_yuv, 0);

	if (pcfg->frc_demo_window == 1) {
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_0,  0, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_1,
				frc_setting->mv_vres << 15 | frc_setting->mv_hres, 0);
	} else if (pcfg->frc_demo_window == 2) {
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_0,
				frc_setting->mv_vres << 15, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_1,
				frc_setting->mv_vres << 16 | frc_setting->mv_hres, 0);
	} else if (pcfg->frc_demo_window == 3) {
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_0, 0, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_1,
			frc_setting->mv_vres << 16 | frc_setting->mv_hres >> 1, 0);
	} else if (pcfg->frc_demo_window == 4) {
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_0,
			frc_setting->mv_hres >> 1, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_1,
			frc_setting->mv_vres << 16 | frc_setting->mv_hres, 0);
	} else if (pcfg->frc_demo_window == 5) {
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_0,  0, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEMO_WIN_SIZE_1,
			frc_setting->mv_vres << 16 | frc_setting->mv_hres, 0);
	}
	iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_VIDEO_BUF_CTRL, 0x00002000, 0);
	iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_V9_GENERIC_CTRL, 0xffffffff, 0);

	if (iris_tnr_mode_get_i7()) {
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_MISC_CTRL, 0x00000032, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_ECO_CTRL, 0x00000010, 0);
	} else {
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_MISC_CTRL, 0x0000002a, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_ECO_CTRL, 0x00000000, 0);
	}

	if (iris_low_latency_mode_get_i7()) {
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD_WINDOW_CTRL, 0x0, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD0_BR, 0x2d50145, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD0_TL, 0x2cb013b, 0);
	}

	if (iris_low_latency_mode_get_i7() == NORMAL_LT || iris_low_latency_mode_get_i7() == LT_MODE)
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_BLENDING_CTRL1, 0x4445555f, 0);
	else if (iris_low_latency_mode_get_i7() == ULTRA_LT_MODE)
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_BLENDING_CTRL1, 0x4455555f, 0);

	if (pcfg->memc_info.memc_mode != MEMC_DUAL_EXTMV_ENABLE) {
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_BLENDING_CTRL3, 0x40000121, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_BLENDING_CTRL4, 0x15008fff, 0);
	}
	iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_DEBUGBUS_MUX, 0x40000, 0);
	iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_SHDW_CTRL, 0x00000100, 1);	// set last flag
}

void iris_pwil_intstat_clear_i7(void)
{
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_INTCLR, 0x40, 0);
}

void iris_fi_demo_window_set_i7(u32 mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->osd_label = (mode >> 5) & 0x01;
	pcfg->frc_label = (mode >> 4) & 0x01;
	pcfg->frc_demo_window = mode & 0xf;
}

int iris_fi_osd_protect_window_calc_i7(u32 top_left_pos, u32 bottom_right_pos,
				u32 osd_window_ctrl, u32 enable, u32 dynCompensate)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 temp0, temp1, temp2, temp3;

	if (osd_window_ctrl > 4) {
		IRIS_LOGE("OSD protect window number only have 5.");
		return -EINVAL;
	}

	temp0 = top_left_pos & 0xffff;
	temp1 = (top_left_pos >> 16) & 0xffff;
	temp2 = bottom_right_pos & 0xffff;
	temp3 = (bottom_right_pos >> 16) & 0xffff;

	if ((temp0 > (pcfg->frc_setting.disp_hres - 1))
			|| (temp1 > (pcfg->frc_setting.disp_vres - 1))
			|| (temp2 > (pcfg->frc_setting.disp_hres - 1))
			|| (temp3 > (pcfg->frc_setting.disp_vres - 1))) {
		IRIS_LOGE("OSD protect window size error.");
		return -EINVAL;
	}

	if ((temp0 > temp2) || (temp1 > temp3)) {
		IRIS_LOGE("OSD protect window begin point position is bigger than end point position.");
		return -EINVAL;
	}

	if (!dynCompensate)
		pcfg->frc_setting.iris_osd_win_dynCompensate &= (~(1 << osd_window_ctrl));
	else
		pcfg->frc_setting.iris_osd_win_dynCompensate |= (1 << osd_window_ctrl);

	if (!enable) {
		pcfg->frc_setting.iris_osd_window_ctrl &= (~(7 << (osd_window_ctrl * 3)));
	} else {
		pcfg->frc_setting.iris_osd_window_ctrl |= (1 << (osd_window_ctrl * 3));
		switch (osd_window_ctrl) {
		case 0:
			pcfg->frc_setting.iris_osd0_tl = top_left_pos;
			pcfg->frc_setting.iris_osd0_br = bottom_right_pos;
			break;
		case 1:
			pcfg->frc_setting.iris_osd1_tl = top_left_pos;
			pcfg->frc_setting.iris_osd1_br = bottom_right_pos;
			break;
		case 2:
			pcfg->frc_setting.iris_osd2_tl = top_left_pos;
			pcfg->frc_setting.iris_osd2_br = bottom_right_pos;
			break;
		case 3:
			pcfg->frc_setting.iris_osd3_tl = top_left_pos;
			pcfg->frc_setting.iris_osd3_br = bottom_right_pos;
			break;
		case 4:
			pcfg->frc_setting.iris_osd4_tl = top_left_pos;
			pcfg->frc_setting.iris_osd4_br = bottom_right_pos;
			break;
		default:
			break;
		}
	}
	return rc;
}

void iris_osd_protect_window_set_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;
	u32 osd0_tl, osd0_br, osd1_tl, osd1_br, osd2_tl, osd2_br;
	u32 osd3_tl, osd3_br, osd4_tl, osd4_br;
	u32 temp0, temp1, temp2, temp3;

	//osd window 0
	temp0 = pcfg->frc_setting.iris_osd0_tl & 0xfff;
	temp0 = (temp0 * frc_setting->mv_hres) / frc_setting->disp_hres;
	temp1 = (pcfg->frc_setting.iris_osd0_tl >> 16) & 0xfff;
	temp1 = (temp1 * frc_setting->mv_vres) / frc_setting->disp_vres;

	temp2 = pcfg->frc_setting.iris_osd0_br & 0xfff;
	temp2 = (temp2 * frc_setting->mv_hres) / frc_setting->disp_hres;
	temp3 = (pcfg->frc_setting.iris_osd0_br >> 16) & 0xfff;
	temp3 = (temp3 * frc_setting->mv_vres) / frc_setting->disp_vres;
	osd0_tl = (temp1 << 16) | temp0;
	osd0_br = (temp3 << 16) | temp2;
	//osd window 1
	temp0 = pcfg->frc_setting.iris_osd1_tl & 0xfff;
	temp0 = (temp0 * frc_setting->mv_hres)/frc_setting->disp_hres;
	temp1 = (pcfg->frc_setting.iris_osd1_tl >> 16) & 0xfff;
	temp1 = (temp1 * frc_setting->mv_vres) / frc_setting->disp_vres;

	temp2 = pcfg->frc_setting.iris_osd1_br & 0xfff;
	temp2 = (temp2 * frc_setting->mv_hres) / frc_setting->disp_hres;
	temp3 = (pcfg->frc_setting.iris_osd1_br >> 16) & 0xfff;
	temp3 = (temp3 * frc_setting->mv_vres) / frc_setting->disp_vres;
	osd1_tl = (temp1 << 16) | temp0;
	osd1_br = (temp3 << 16) | temp2;
	//osd window 2
	temp0 = pcfg->frc_setting.iris_osd2_tl & 0xfff;
	temp0 = (temp0 * frc_setting->mv_hres) / frc_setting->disp_hres;
	temp1 = (pcfg->frc_setting.iris_osd2_tl >> 16) & 0xfff;
	temp1 = (temp1 * frc_setting->mv_vres) / frc_setting->disp_vres;

	temp2 = pcfg->frc_setting.iris_osd2_br & 0xfff;
	temp2 = (temp2 * frc_setting->mv_hres) / frc_setting->disp_hres;
	temp3 = (pcfg->frc_setting.iris_osd2_br >> 16) & 0xfff;
	temp3 = (temp3 * frc_setting->mv_vres) / frc_setting->disp_vres;
	osd2_tl = (temp1 << 16) | temp0;
	osd2_br = (temp3 << 16) | temp2;
	//osd window 3
	temp0 = pcfg->frc_setting.iris_osd3_tl & 0xfff;
	temp0 = (temp0 * frc_setting->mv_hres) / frc_setting->disp_hres;
	temp1 = (pcfg->frc_setting.iris_osd3_tl >> 16) & 0xfff;
	temp1 = (temp1 * frc_setting->mv_vres) / frc_setting->disp_vres;

	temp2 = pcfg->frc_setting.iris_osd3_br & 0xfff;
	temp2 = (temp2 * frc_setting->mv_hres) / frc_setting->disp_hres;
	temp3 = (pcfg->frc_setting.iris_osd3_br >> 16) & 0xfff;
	temp3 = (temp3 * frc_setting->mv_vres) / frc_setting->disp_vres;
	osd3_tl = (temp1 << 16) | temp0;
	osd3_br = (temp3 << 16) | temp2;
	//osd window 4
	temp0 = pcfg->frc_setting.iris_osd4_tl & 0xfff;
	temp0 = (temp0 * frc_setting->mv_hres) / frc_setting->disp_hres;
	temp1 = (pcfg->frc_setting.iris_osd4_tl >> 16) & 0xfff;
	temp1 = (temp1 * frc_setting->mv_vres) / frc_setting->disp_vres;

	temp2 = pcfg->frc_setting.iris_osd4_br & 0xfff;
	temp2 = (temp2 * frc_setting->mv_hres) / frc_setting->disp_hres;
	temp3 = (pcfg->frc_setting.iris_osd4_br >> 16) & 0xfff;
	temp3 = (temp3 * frc_setting->mv_vres) / frc_setting->disp_vres;
	osd4_tl = (temp1 << 16) | temp0;
	osd4_br = (temp3 << 16) | temp2;

	iris_memc_cmd_payload_init_i7();
	if (pcfg->frc_setting.iris_osd_window_ctrl & 0x1249) { //osd protection
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD0_TL, osd0_tl, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD0_BR, osd0_br, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD1_TL, osd1_tl, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD1_BR, osd1_br, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD2_TL, osd2_tl, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD2_BR, osd2_br, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD3_TL, osd3_tl, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD3_BR, osd3_br, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD4_TL, osd4_tl, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD4_BR, osd4_br, 0);
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD_WINDOW_CTRL, pcfg->frc_setting.iris_osd_window_ctrl, 0);
	} else {
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_OSD_WINDOW_CTRL, 0x00000000, 0);
	}
	iris_memc_cmd_payload_send_i7();
}

static void iris_download_mcu_code_i7(void)
{
	iris_send_ipopt_cmds(APP_CODE_LUT, 0);
	IRIS_LOGI("%s", __func__);
}

static void iris_mcu_sw_reset_i7(u32 reset)
{
	u32 write_data[2];

	write_data[0] = IRIS_SYS_ADDR + MCU_SW_RESET;
	write_data[1] = reset;
	iris_ocp_write_mult_vals(2, write_data);
}

static void iris_mcu_int_disable_i7(void)
{
	u32 write_data[2];

	write_data[0] = IRIS_UNIT_CTRL_INTEN;
	write_data[1] = 0;
	iris_ocp_write_mult_vals(2, write_data);
}

static bool iris_mcu_is_idle_i7(void)
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

void iris_set_panel_te_i7(u8 panel_te)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->panel_te = panel_te;
}

void iris_update_panel_ap_te_i7(u32 new_te)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->panel_te = new_te;
	pcfg->ap_te = new_te;
}

void iris_set_n2m_enable_i7(bool enable, u8 n2m_ratio)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->n2m_enable = enable;
	pcfg->n2m_ratio = n2m_ratio;
	IRIS_LOGI("%s: n2m_enable:%d n2m_ratio:%d", __func__, pcfg->n2m_enable, pcfg->n2m_ratio);
}

void iris_frc_vfr_threshold_set_i7(int count, bool force_update)
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

void iris_dtg_ovs_dly_setting_send_i7(bool enable)
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

	if (iris_low_latency_mode_get_i7() == ULTRA_LT_MODE) {
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

void iris_dtg_te_n2m_ctrl_setting_send_i7(bool enable)
{
	u32 cmd[6];
	u32 *payload = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

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

void iris_hangup_timeout_cnt_update_i7(bool enable)
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

void iris_parse_ap_panel_te_i7(struct device_node *np)
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

void iris_parse_frc_dec_initial_delay_i7(struct device_node *np)
{
	int ret = 0;
	uint32_t val = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return;

	ret = p_dts_ops->read_u32(np, "pxlw,frc-dsc-init-delay", &val);
	if (ret != 0) {
		IRIS_LOGI("%s(), doesn't need manual frc dec initial delay, return: %d",
				__func__, ret);
		return;
	}

	if (val != 0) {
		iris_frc_dec_initial_delay = BITS_GET(val, 16, 0);

		IRIS_LOGI("%s(), manual frc initial delay: %u, 0x%04x",
				__func__, val, iris_frc_dec_initial_delay);
	}
}

void iris_frc_timing_setting_update_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_mode_info *timing;

	if (!pcfg->panel->cur_mode)
		return;

	timing = &pcfg->panel->cur_mode->timing;

	pcfg->panel_te = timing->refresh_rate;
	pcfg->ap_te = timing->refresh_rate;
	pcfg->frc_setting.input_vtotal = DSI_V_TOTAL(timing);
	// temp treat display timing same as input timing
	pcfg->frc_setting.disp_hres = timing->h_active;
	pcfg->frc_setting.disp_vres = timing->v_active;
	pcfg->frc_setting.disp_dsc = timing->dsc_enabled;
	pcfg->frc_setting.disp_htotal = DSI_H_TOTAL(timing);
	pcfg->frc_setting.disp_vtotal = DSI_V_TOTAL(timing);
}

void iris_set_out_frame_rate_i7(u32 fps)
{
	IRIS_LOGI("%s, set fps: %u", __func__, fps);

	iris_dtg_frame_rate_set();
}

void iris_parse_mv_resolution_i7(void)
{
	u32 *payload = NULL;
	u8 single_opt_id = 0xb0;
	u8 dual_opt_id = 0xb1;
	struct iris_cfg *pcfg = iris_get_cfg();

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, single_opt_id, 2);
	if (!payload)
		return;
	pcfg->frc_setting.init_single_mv_hres = payload[5] & 0xffff;
	pcfg->frc_setting.init_single_mv_vres = (payload[5] >> 16) & 0xffff;
	pcfg->frc_setting.init_video_baseaddr = payload[7];

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, dual_opt_id, 2);
	if (!payload)
		return;
	pcfg->frc_setting.init_dual_mv_hres = payload[5] & 0xffff;
	pcfg->frc_setting.init_dual_mv_vres = (payload[5] >> 16) & 0xffff;
	pcfg->frc_setting.init_dual_video_baseaddr = payload[7];

	IRIS_LOGI("%s single algo: (0x%04x%04x) VideoAddr: %x, dual algo: (0x%04x%04x) VideoAddr: %x",
				__func__, pcfg->frc_setting.init_single_mv_hres,
				pcfg->frc_setting.init_single_mv_vres,
				pcfg->frc_setting.init_video_baseaddr,
				pcfg->frc_setting.init_dual_mv_hres,
				pcfg->frc_setting.init_dual_mv_vres,
				pcfg->frc_setting.init_dual_video_baseaddr);

}

void iris_parse_memc_param0_i7(struct device_node *np)
{
	iris_parse_ap_panel_te_i7(np);
	iris_parse_frc_dec_initial_delay_i7(np);
}

void iris_parse_memc_param1_i7(void)
{
	iris_parse_mv_resolution_i7();
}

void iris_frc_setting_init_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->frc_setting.mv_buf_num = 3;
	pcfg->frc_setting.vid_buf_num = 3;
	pcfg->frc_setting.rgme_buf_num = 3;
	pcfg->frc_setting.video_baseaddr = 0x20000;
	pcfg->frc_setting.mv_coef = 0x68;

	pcfg->frc_setting.mv_hres = pcfg->frc_setting.init_single_mv_hres;
	pcfg->frc_setting.mv_vres = pcfg->frc_setting.init_single_mv_vres;

	iris_frc_timing_setting_update_i7();
	iris_frc_dsc_setting_update_i7(MEMC_SINGLE_VIDEO_ENABLE);

	iris_memc_mspwil_setting_init_i7();
}

void iris_frc_dsc_setting_update_i7(u8 mode)
{
	u32 *payload = NULL;
	u32 pps_select, sr_select;
	u8 opt_id_0 = 0xb0;
	struct iris_cfg *pcfg = iris_get_cfg();
	bool dual = false;

	if (mode == MEMC_DUAL_VIDEO_ENABLE || mode == MEMC_DUAL_EXTMV_ENABLE || mode == MEMC_DUAL_GAME_ENABLE)
		dual = true;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, opt_id_0, 2);
	if (!payload) {
		IRIS_LOGE("%s(), can not get pwil_frc_dsc property in pwil setting", __func__);
		return;
	}
	pps_select = (payload[1] >> 18) & 0x1;
	sr_select = (payload[1] >> 21) & 0x1;

	if (dual)
		pcfg->frc_setting.video_baseaddr = pcfg->frc_setting.init_dual_video_baseaddr;
	else
		pcfg->frc_setting.video_baseaddr = pcfg->frc_setting.init_video_baseaddr;

	pcfg->frc_setting.pps_table_sel = pps_select;
	pcfg->frc_setting.sr_sel = sr_select;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_ENC_2, 0xf1, 2);
	if (!payload) {
		IRIS_LOGE("%s(), can not get dsc 0xf1 property in DSC setting", __func__);
		return;
	}
	pcfg->frc_setting.mv_coef = (payload[42] >> 8) & 0xff;
	pcfg->frc_setting.dsc_enc_ctrl0 = payload[8];
	pcfg->frc_setting.dsc_enc_ctrl1 = payload[16];
}

void iris_vinvout_frc_exit_i7(void)
{
#define VINVOUT_CMD_SIZE 14
	u32 *payload = NULL;
	u32 vinvout_frc_cmd[VINVOUT_CMD_SIZE];

	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0xf3, 1);
	if (!payload)
		return;
	memcpy(vinvout_frc_cmd, payload, (VINVOUT_CMD_SIZE - 2) * sizeof(u32));
	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0x00, 2); //get dtg init packet
	if (!payload)
		return;
	vinvout_frc_cmd[1] = payload[11];	//EVS_DLY
	vinvout_frc_cmd[3] = payload[12];	//FRC_EVS_DLY
	vinvout_frc_cmd[5] = payload[13];	//OVS_DLY_SWITCH
	vinvout_frc_cmd[7] = payload[15];	//OVS_DLY
	vinvout_frc_cmd[9] = payload[29];	//OVS_DLY_FRC
	vinvout_frc_cmd[11] = payload[8];	//DTG_CTRL
	vinvout_frc_cmd[12] = 0xF1350000;	//update
	vinvout_frc_cmd[13] = 0x0000000F;
	iris_ocp_write_mult_vals(VINVOUT_CMD_SIZE, vinvout_frc_cmd);
}

void iris_memc_mspwil_setting_init_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->chip_mspwil_setting.memc_mode = MEMC_SINGLE_VIDEO_ENABLE;
	pcfg->chip_mspwil_setting.memc_lt_mode = 0;
	pcfg->chip_mspwil_setting.memc_n2m_mode = 0;
	pcfg->chip_mspwil_setting.tnr_mode = 0;
	pcfg->chip_mspwil_setting.mv_hres = pcfg->frc_setting.mv_hres;
	pcfg->chip_mspwil_setting.mv_vres = pcfg->frc_setting.mv_vres;
	pcfg->chip_mspwil_setting.input_scale_level = 0x10;
	pcfg->chip_mspwil_setting.pp_scale_level = 0xb;
	pcfg->chip_mspwil_setting.disp_hres = pcfg->frc_setting.disp_hres;
	pcfg->chip_mspwil_setting.disp_vres = pcfg->frc_setting.disp_vres;
}

int iris_frc_in2_width_get_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int width = pcfg->frc_setting.mv_hres;

	if (pcfg->memc_info.memc_mode == MEMC_DUAL_EXTMV_ENABLE) {
		if ((pcfg->emv_info.gameWidth != pcfg->emv_info.gameWidthSrc)
			&& (pcfg->emv_info.overflow == 0))
			width = pcfg->emv_info.gameWidthSrc&(~(u32)0x1);
	}
	return width;
}

int iris_frc_in2_height_get_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int height = pcfg->frc_setting.mv_vres;

	if (pcfg->memc_info.memc_mode == MEMC_DUAL_EXTMV_ENABLE) {
		if ((pcfg->emv_info.gameHeight != pcfg->emv_info.gameHeightSrc)
			&& (pcfg->emv_info.overflow == 0))
			height = pcfg->emv_info.gameHeightSrc&(~(u32)0x1);
	}
	return height;
}

int iris_frc_in1_width_get_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int width = pcfg->frc_setting.disp_hres;

	if (pcfg->memc_info.memc_mode == MEMC_DUAL_EXTMV_ENABLE) {
		if (pcfg->emv_info.overflow > 0)
			width = pcfg->emv_info.gameWidthSrc&(~(u32)0x1);
	}
	return width;
}

int iris_frc_in1_height_get_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int height = pcfg->frc_setting.disp_vres;

	if (pcfg->memc_info.memc_mode == MEMC_DUAL_EXTMV_ENABLE) {
		if (pcfg->emv_info.overflow > 0)
			height = pcfg->emv_info.gameHeightSrc&(~(u32)0x1);
	}
	return height;
}

void iris_memc_mspwil_setting_update_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u8 memc_mode = pcfg->memc_info.memc_mode;
	bool frc_setting_changed = false;
	bool sr_changed = true;
	struct iris_mspwil_setting *chip_setting = &pcfg->chip_mspwil_setting;

	frc_setting_changed =
		((pcfg->memc_info.memc_mode != chip_setting->memc_mode) ||
		 (pcfg->memc_info.tnr_en != chip_setting->tnr_mode) ||
		 (pcfg->memc_info.low_latency_mode != chip_setting->memc_lt_mode) ||
		 (pcfg->memc_info.n2m_mode != chip_setting->memc_n2m_mode) ||
		 (pcfg->frc_setting.mv_hres != chip_setting->mv_hres) ||
		 (pcfg->frc_setting.mv_vres != chip_setting->mv_vres) ||
		 (pcfg->panel_te != chip_setting->panel_te) ||
		 (pcfg->frc_setting.disp_hres != chip_setting->disp_hres) ||
		 (pcfg->frc_setting.disp_vres != chip_setting->disp_vres));


	if (memc_mode == MEMC_DUAL_EXTMV_ENABLE)
		frc_setting_changed |= iris_emv_game_size_changed_i7();

	if (frc_setting_changed || sr_changed) {
		iris_sr_change_i7(iris_frc_in2_height_get_i7(),
				iris_frc_in2_width_get_i7(),
				pcfg->frc_setting.disp_vres,
				pcfg->frc_setting.disp_hres,
				pcfg->frc_setting.sr_sel,
				true);

		/* send sdr2hdr size2 to iris */
		iris_sdr2hdr_update_size2(pcfg->frc_setting.mv_hres, pcfg->frc_setting.mv_vres);
	}

	if (frc_setting_changed) {
		/* update ioinc/dsc/sr cmdlist and send to iris */
		iris_frc_ioinc_change_i7();
		iris_frc_dsc_change_i7();

		/* record memc setting of dma cmdlist */
		pcfg->chip_mspwil_setting.memc_mode = pcfg->memc_info.memc_mode;
		pcfg->chip_mspwil_setting.memc_lt_mode = pcfg->memc_info.low_latency_mode;
		pcfg->chip_mspwil_setting.memc_n2m_mode = pcfg->memc_info.n2m_mode;
		pcfg->chip_mspwil_setting.tnr_mode = pcfg->memc_info.tnr_en;
		pcfg->chip_mspwil_setting.mv_hres = pcfg->frc_setting.mv_hres;
		pcfg->chip_mspwil_setting.mv_vres = pcfg->frc_setting.mv_vres;
		pcfg->chip_mspwil_setting.input_scale_level = 0x10;
		pcfg->chip_mspwil_setting.pp_scale_level = 0xb;
		pcfg->chip_mspwil_setting.panel_te = pcfg->panel_te;
		pcfg->chip_mspwil_setting.disp_hres = pcfg->frc_setting.disp_hres;
		pcfg->chip_mspwil_setting.disp_vres = pcfg->frc_setting.disp_vres;
	}
}

void iris_memc_status_clear_i7(bool init)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	memset(&pcfg->memc_info, 0, sizeof(struct iris_memc_info));
	memset(&pcfg->emv_info, 0, sizeof(struct extmv_frc_meta));
	pcfg->iris_frc_vfr_st = false;
	atomic_set(&pcfg->video_update_wo_osd, 0);
	//cancel_work_sync(&pcfg->vfr_update_work);
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

void iris_memc_frc_phase_update_i7(void)
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

void iris_memc_info_update_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	bool enableGameMode = false;

	/* if debug via 173 cmd, use default info */
	if (pcfg->memc_info.memc_mode == MEMC_DISABLE) {
		if (pcfg->dual_enabled) {
			pcfg->memc_info.memc_mode = MEMC_DUAL_VIDEO_ENABLE;
			pcfg->memc_info.mv_hres = pcfg->frc_setting.init_dual_mv_hres;
			pcfg->memc_info.mv_vres = pcfg->frc_setting.init_dual_mv_vres;
		} else {
			pcfg->memc_info.memc_mode = MEMC_SINGLE_VIDEO_ENABLE;
			pcfg->memc_info.mv_hres = pcfg->frc_setting.init_single_mv_hres;
			pcfg->memc_info.mv_vres = pcfg->frc_setting.init_single_mv_vres;
		}
		pcfg->memc_info.memc_level = 0;
		pcfg->memc_info.video_fps = 30;
		pcfg->memc_info.panel_fps = pcfg->panel_te;
	}

	if (debug_low_latency_overwrite & 0x80)
		pcfg->memc_info.low_latency_mode = debug_low_latency_overwrite & 0x0f;
	if (debug_tnr_en_overwrite & 0x80)
		pcfg->memc_info.tnr_en = debug_tnr_en_overwrite & 0x0f;
	if (debug_frc_vfr_overwrite & 0x80)
		pcfg->memc_info.vfr_en = debug_frc_vfr_overwrite & 0x0f;
	if (debug_n2m_mode_overwrite & 0x80)
		pcfg->memc_info.n2m_mode = debug_n2m_mode_overwrite & 0x0f;
	if (debug_memc_level_overwrite & 0x80)
		pcfg->memc_info.memc_level = debug_memc_level_overwrite & 0x0f;

	if (debug_mv_res_overwrite & 0x80000000) {
		pcfg->memc_info.mv_hres = debug_mv_res_overwrite & 0xffff;
		pcfg->memc_info.mv_vres = (debug_mv_res_overwrite >> 16) & 0x7fff;
	}

	enableGameMode = (pcfg->memc_info.memc_mode == MEMC_DUAL_VIDEO_ENABLE);
	enableGameMode &= (pcfg->memc_info.low_latency_mode == ULTRA_LT_MODE);
	enableGameMode |= (pcfg->memc_info.memc_mode == MEMC_SINGLE_GAME_ENABLE);
	enableGameMode |= (pcfg->memc_info.memc_mode == MEMC_DUAL_GAME_ENABLE);

	//when tx output mode is video mode, disable vfr function.
	if (pcfg->tx_mode == DSI_OP_VIDEO_MODE ||
		pcfg->memc_info.memc_mode == MEMC_DUAL_EXTMV_ENABLE || enableGameMode)
		pcfg->memc_info.vfr_en = 0;

	if (pcfg->tx_mode == DSI_OP_VIDEO_MODE ||
		pcfg->memc_info.memc_mode != MEMC_SINGLE_VIDEO_ENABLE)
		pcfg->memc_info.tnr_en = 0;

	if (pcfg->tx_mode == DSI_OP_VIDEO_MODE ||
		pcfg->memc_info.low_latency_mode >= LT_INVALID ||
		(pcfg->memc_info.memc_mode != MEMC_SINGLE_VIDEO_ENABLE &&
		(!enableGameMode)))
		pcfg->memc_info.low_latency_mode = 0;

	if (pcfg->memc_info.memc_mode == MEMC_DUAL_EXTMV_ENABLE ||
		pcfg->memc_info.memc_mode == MEMC_DUAL_VIDEO_ENABLE ||
		pcfg->memc_info.memc_mode == MEMC_DUAL_GAME_ENABLE)
		pcfg->memc_info.n2m_mode = 0;

	IRIS_LOGI("memc info: mode-%d, fw-%d, level-%d, ratio-%d-%d, vfr-%d, tnr-%d, lt-%d, n2m-%d, osdwin-%d",
		pcfg->memc_info.memc_mode, (pcfg->memc_info.mv_hres << 16) | pcfg->memc_info.mv_vres,
		pcfg->memc_info.memc_level, pcfg->memc_info.video_fps, pcfg->memc_info.panel_fps,
		pcfg->memc_info.vfr_en, pcfg->memc_info.tnr_en,
		pcfg->memc_info.low_latency_mode, pcfg->memc_info.n2m_mode,
		pcfg->memc_info.osd_window_en);
}

void iris_frc_setting_update_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;
	bool enableGameMode = false;

	enableGameMode = (pcfg->memc_info.memc_mode == MEMC_DUAL_VIDEO_ENABLE);
	enableGameMode &= (pcfg->memc_info.low_latency_mode == ULTRA_LT_MODE);
	enableGameMode |= (pcfg->memc_info.memc_mode == MEMC_SINGLE_GAME_ENABLE);
	enableGameMode |= (pcfg->memc_info.memc_mode == MEMC_DUAL_GAME_ENABLE);

	if (pcfg->memc_info.memc_mode == MEMC_DUAL_EXTMV_ENABLE) {
		frc_setting->mv_hres = pcfg->emv_info.gameWidth;
		frc_setting->mv_vres = pcfg->emv_info.gameHeight;
	} else {
		frc_setting->mv_hres = pcfg->memc_info.mv_hres;
		frc_setting->mv_vres = pcfg->memc_info.mv_vres;
	}

	if ((iris_low_latency_mode_get_i7() == ULTRA_LT_MODE) &&
		(!iris_three_buffer_low_latency)) {
		frc_setting->vid_buf_num = 2;
		frc_setting->rgme_buf_num = 0;
		frc_setting->mv_buf_num = 3;
	} else if (iris_tnr_mode_get_i7()) {
		frc_setting->vid_buf_num = 2;
		frc_setting->rgme_buf_num = 0;
		frc_setting->mv_buf_num = 3;
	} else {
		frc_setting->vid_buf_num = 3;
		frc_setting->rgme_buf_num = 3;
		frc_setting->mv_buf_num = 3;
	}

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_VIDEO_ENABLE) {
		frc_setting->frc_vfr_disp = pcfg->memc_info.vfr_en;
	} else {
		frc_setting->frc_vfr_disp = false;
	}
	pcfg->iris_frc_vfr_st = frc_setting->frc_vfr_disp;

	if (debug_disable_frc_dynen || enableGameMode)
		frc_setting->frc_dynen = false;
	else
		frc_setting->frc_dynen = true;

	if (debug_frc_force_repeat)
		frc_setting->force_repeat = true;
	else
		frc_setting->force_repeat = false;

	frc_setting->sr_en = iris_sr_enable;

	frc_setting->layer_c_en = iris_frc_layer_c_enable;
	if (frc_setting->mv_hres > 640)
		frc_setting->layer_c_en = false;

	iris_frc_timing_setting_update_i7();
	iris_frc_dsc_setting_update_i7(pcfg->memc_info.memc_mode);

	IRIS_LOGI("frc setting: disp-%dx%d@%d, fw-%d coef-%d, buf-%d@%x pps-%d sr_sel-%d frc_vfr:%d dsc-%x/%x",
		frc_setting->disp_hres, frc_setting->disp_vres, pcfg->panel_te,
		(frc_setting->mv_hres << 16) | frc_setting->mv_vres, frc_setting->mv_coef,
		frc_setting->mv_buf_num, frc_setting->video_baseaddr,
		frc_setting->pps_table_sel, frc_setting->sr_sel, frc_setting->frc_vfr_disp,
		frc_setting->dsc_enc_ctrl0, frc_setting->dsc_enc_ctrl1);
}

void iris_mcu_memc_info_send_i7(void)
{
	u32 info[5];
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_memc_info *memc_info;

	memc_info = &pcfg->memc_info;
	info[0] = (memc_info->memc_mode << 28) |
		  (memc_info->low_latency_mode << 24) |
		  (memc_info->tnr_en << 23) |
		  (memc_info->vfr_en << 22) |
		  memc_info->memc_app;
	info[1] = (pcfg->frc_setting.mv_hres << 16) |
		  (pcfg->frc_setting.mv_vres);
	info[2] = (pcfg->frc_setting.disp_hres << 16) |
		  (pcfg->frc_setting.disp_vres);
	info[3] = (1 << 31) |
		  (memc_info->video_fps << 24) |
		  (memc_info->panel_fps << 16) |
		  (pcfg->panel_te << 8);
	info[4] = ((u32)(memc_info->memc_level) << 28) | judder_angle;

	iris_frc_reg_add_i7(IRIS_PROXY_ADDR + 0x08, info[0], 0x0);
	iris_frc_reg_add_i7(IRIS_PROXY_ADDR + 0x10, info[1], 0x0);
	iris_frc_reg_add_i7(IRIS_PROXY_ADDR + 0x18, info[2], 0x0);
	iris_frc_reg_add_i7(IRIS_PROXY_ADDR + 0x20, info[3], 0x0);
	iris_frc_reg_add_i7(IRIS_PROXY_ADDR + 0x40, info[4], 0x0);
}

void iris_mcu_version_read_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	static bool ret = false;

	if (!ret) {
		pcfg->app_version1 = iris_ocp_read(IRIS_PROXY_ADDR, DSI_CMD_SET_STATE_HS);
		ret = true;
	}
}

void iris_mcu_state_set_i7(u32 mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_mcu_enable
		|| (iris_mcu_ctrl && mode >= MCU_INT_DISABLE)
		|| (!iris_mcu_ctrl && !iris_mcu_ctrl_org &&  mode >= MCU_INT_DISABLE)
		|| (iris_mcu_ctrl_org && mode == MCU_START)
		|| (iris_mcu_mode == mode)
		|| (mode == MCU_INT_DISABLE && iris_mcu_mode != MCU_START)) {
		if (mode != MCU_INT_DISABLE)
			iris_mcu_ctrl_org = iris_mcu_ctrl;
		return;
	}

	if (mode == MCU_START) {
		if (!pcfg->mcu_code_downloaded) {
			iris_download_mcu_code_i7();
			pcfg->mcu_code_downloaded = true;
		}
		iris_mcu_sw_reset_i7(0);
		iris_mcu_stop = false;
	} else if (mode == MCU_INT_DISABLE) {
		iris_mcu_int_disable_i7();
	} else if (mode == MCU_STOP && iris_mcu_stop == false) {
		iris_mcu_stop = true;
		if (iris_mcu_is_idle_i7())
			iris_mcu_sw_reset_i7(1);
		else
			IRIS_LOGI("iris mcu not in stop, can't reset mcu");
	}
	iris_mcu_mode = mode;
	if (mode != MCU_INT_DISABLE)
		iris_mcu_ctrl_org = iris_mcu_ctrl;
}

void iris_mcu_ctrl_set_i7(u32 sdr_ctrl)
{
	iris_mcu_ctrl &= ~0x02;
	iris_mcu_ctrl |= (sdr_ctrl << 1);
}

void iris_mcu_mode_reset_i7(void)
{
	iris_mcu_mode = MCU_INT_DISABLE;
	iris_mcu_ctrl = 0;
	iris_mcu_ctrl_org = 0;
	iris_mcu_stop = false;
}

void iris_pwil_reg_set_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_frc_setting *frc_setting = &pcfg->frc_setting;
	u32 *payload = NULL;
	u32 hstride = 0;

	/* frc ctrl */
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x70, 2);
	if (!payload)
		return;
	payload[0] &= (~0x00000001);
	payload[0] |= pcfg->memc_info.vfr_en;
	payload[1] &= (~0x00073f10);
	payload[1] |= (iris_tnr_mode_get_i7() << 4) |
				(frc_setting->vid_buf_num << 8) |
				(frc_setting->vid_buf_num << 11) |
				(frc_setting->mv_buf_num << 16);
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x70, 0x70, 1);

	/* video ctrl */
	hstride = iris_frc_video_hstride_calc_i7(frc_setting->mv_hres, frc_setting->mv_coef, 1);
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xB0, 2);
	if (!payload)
		return;
	payload[5] = (frc_setting->mv_vres << 16) | frc_setting->mv_hres;
	payload[6] = hstride;
	payload[7] = frc_setting->video_baseaddr;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xB0, 0xB0, 1);

	/* pwil ctrl */
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xf0, 2);
	if (!payload)
		return;
	payload[2] &= (~0x0000c000);
	if (!iris_tnr_mode_get_i7())
		payload[2] |= (1 << 14);
	payload[3] &= (~0x00000600);
	if (!frc_setting->sr_en)
		payload[3] |= (1 << 9);
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xf0, 0xf0, 1);

	/* update */
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x80, 0x80, 1);
	iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe4, 0xe4, 0);
	iris_update_pq_opt(PATH_DSI, true);

	IRIS_LOGI("%s: dsc_bpp 0x%x, hstride %d", __func__, frc_setting->mv_coef, hstride);
}

void iris_set_pwil_mode_i7(u8 mode, bool osd_enable, int state, bool commit)
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
	if (osd_enable)
		pwil_mode[0] |= 0x80;

	if (pcfg->panel->cur_mode && pcfg->panel->cur_mode->priv_info &&
			pcfg->panel->cur_mode->priv_info->dsc_enabled)
		pwil_mode[0] |= 0x10;

	IRIS_LOGI("%s(), set pwil mode: %x, %x", __func__, pwil_mode[0], pwil_mode[1]);

	if (commit)
		iris_dsi_send_cmds(pcfg->panel, panel_cmds.cmds,
				panel_cmds.count, panel_cmds.state, pcfg->vc_ctrl.to_iris_vc_id);

	meta = pwil_mode[0] | (pwil_mode[1] << 8);
	iris_rx_meta_dma_list_send(meta, commit);
}

void iris_memc_ctrl_frc_prepare_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	bool enableGameMode = false;

	enableGameMode = (pcfg->memc_info.memc_mode == MEMC_DUAL_VIDEO_ENABLE);
	enableGameMode &= (pcfg->memc_info.low_latency_mode == ULTRA_LT_MODE);
	enableGameMode |= pcfg->memc_info.memc_mode == MEMC_SINGLE_GAME_ENABLE;
	enableGameMode |= pcfg->memc_info.memc_mode == MEMC_DUAL_GAME_ENABLE;

	pcfg->pt_sr_enable_restore = pcfg->pt_sr_enable;
	iris_pt_sr_reset_i7();
	iris_memc_info_update_i7();
	iris_frc_setting_update_i7();

	if (pcfg->dual_enabled)
		iris_blending_al_coef_enable(false);

	/* disable FLFP before memc */
	if (!pcfg->dual_enabled)
		iris_frc_lp_switch(true, false);

	/* disable Line lock before memc in vin-vout*/
	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE))
		iris_linelock_set(false, true);

	iris_dtg_ovs_dly_setting_send_i7(true);
	iris_pwil_idle_mask_update(true);
	iris_hangup_timeout_cnt_update_i7(true);
	if (pcfg->memc_info.n2m_mode == 1 || pcfg->memc_info.n2m_mode == 2)
		iris_dtg_te_n2m_ctrl_setting_send_i7(true);

	iris_memc_cmd_payload_init_i7();
	/* send filter ratio cmd to iris */
	iris_ioinc_filter_ratio_send();

	/* update dma link list if need */
	iris_memc_mspwil_setting_update_i7();
	iris_pwil_reg_set_i7();

	/* power domain on */
	//iris_pmu_frc_set(true);
	if (enableGameMode || pcfg->memc_info.memc_mode == MEMC_DUAL_EXTMV_ENABLE)
		iris_sr_level_set(FRC_MODE, pcfg->frcgame_pq_guided_level,
			pcfg->frcgame_pq_dejaggy_level,
			pcfg->frcgame_pq_peaking_level,
			pcfg->frcgame_pq_DLTI_level);
	else
		iris_sr_level_set(FRC_MODE, pcfg->frc_pq_guided_level,
			pcfg->frc_pq_dejaggy_level,
			pcfg->frc_pq_peaking_level,
			pcfg->frc_pq_DLTI_level);
	iris_sr_update_i7();
	iris_pmu_bsram_set(true);
	iris_pmu_dscu_set(true);

	/* in vin-vout mode, enter memc to need to enable efifo and dtg eco */
	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
		iris_pwil0_efifo_enable_i7(true);
		iris_dtg_eco_i7(true, true);
		pcfg->dtg_eco_enabled = true;
	}
	/* clear pwil intstat */
	iris_pwil_intstat_clear_i7();
	/* mcu */
	iris_mcu_memc_info_send_i7();
	iris_mcu_ctrl |= 0x01;
	iris_mcu_state_set_i7(MCU_START);
	/* ramctrl */
	iris_ramctrl_mvc_dsy_ctrl_i7();
	/* config FRC_MIF */
	iris_frc_mif_reg_set_i7();
	/* GMD */
	iris_gmd_reg_set_i7();
	/* FBD */
	iris_fbd_reg_set_i7();
	/* CAD */
	iris_cad_reg_set_i7();
	/* MVC */
	iris_mvc_reg_set_i7();
	/* MVF */
	iris_mvf_reg_set_i7();
	/* FI */
	iris_fi_reg_set_i7();
	/* DEBUG REGISTERS SETTING */
	iris_game_memc_registers_set_i7();
	iris_game_memc_OSD_registers_set_i7();

	/* FRC LUT */
	iris_memc_frc_phase_update_i7();

	iris_memc_cmd_payload_send_i7();
}

void iris_memc_ctrl_pt_frc_meta_set_i7(bool frc_enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (frc_enable) {
		/* send pwil meta cmd */
		iris_set_pwil_mode_i7(FRC_MODE, pcfg->dual_enabled, DSI_CMD_SET_STATE_HS, true);
		pcfg->frc_enabled = true;
	} else {
		u8 mode = pcfg->rx_mode == pcfg->tx_mode ? PT_MODE : RFB_MODE;
		/* send pwil meta cmd */
		iris_set_pwil_mode_i7(mode, pcfg->dual_enabled, DSI_CMD_SET_STATE_HS, true);
		pcfg->frc_enabled = false;
	}

	iris_input_frame_cnt_record_i7();
}

void iris_memc_ctrl_pt_frc_grcp_set_i7(bool frc_enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 meta = 0x8204;
	u32 cmd[4];
	u32 *payload = NULL;

	if (pcfg->dual_enabled)
		meta |= 0x80;
	if (pcfg->panel->cur_mode && pcfg->panel->cur_mode->priv_info &&
	    pcfg->panel->cur_mode->priv_info->dsc_enabled)
		meta |= 0x10;

	payload = iris_get_ipopt_payload_data(IRIS_IP_BLEND, 0x20, 2);
	if (!payload)
		return;

	cmd[0] = IRIS_RX0_ADDR + RX0_VDO_META;
	cmd[1] = meta;
	cmd[2] = IRIS_BLENDING + CSR_TO;
	cmd[3] = payload[0];

	if (frc_enable) {
		iris_ocp_write_mult_vals(4, cmd);
		iris_rx_meta_dma_list_send(meta, true);
		pcfg->frc_enabled = true;
	}

	iris_input_frame_cnt_record_i7();
}

void iris_memc_ctrl_pt_post_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->rx_mode != pcfg->tx_mode)
		return;

	iris_memc_cmd_payload_init_i7();
	/* blending */
	if (pcfg->dual_enabled)
		iris_blending_cusor_timeout_set_i7(false);

	if (pcfg->dtg_eco_enabled) {
		iris_pwil0_efifo_enable_i7(false);
		iris_dtg_eco_i7(false, true);
		pcfg->dtg_eco_enabled = false;
	}

	/* power domain off */
	if (!pcfg->pt_sr_enable_restore)
		iris_pmu_frc_set(false);
	iris_pmu_dscu_set(false);
	if (!pcfg->dual_enabled)
		iris_pmu_bsram_set(false);

	iris_pwil_idle_mask_update(false);
	/* enable flfp after exit frc */
	if (!pcfg->dual_enabled)
		iris_frc_lp_switch(false, false);

	/* enable Line lock before memc in vin-vout*/
	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
		iris_ovs_dly_change(false);
		iris_linelock_set(true, true);
		iris_pwil0_efifo_enable_i7(false);
	}

	iris_dtg_ovs_dly_setting_send_i7(false);
	iris_hangup_timeout_cnt_update_i7(false);
	iris_dtg_te_n2m_ctrl_setting_send_i7(false);
	/* mcu */
	iris_mcu_ctrl &= ~0x01;
	iris_mcu_state_set_i7(MCU_STOP);
	iris_memc_cmd_payload_send_i7();

	iris_memc_status_clear_i7(false);
	if (!pcfg->dual_enabled)
		iris_pt_sr_restore_i7();
	if (pcfg->pwil_mode == FRC_MODE) {
		iris_pwil_mode_state_get_i7();
		if (pcfg->pwil_mode == FRC_MODE) {
			pcfg->pwil_mode = PT_MODE;
			IRIS_LOGW("Warning: unexpected work mode. Reset to PT.");
		}
	}
}

void iris_memc_ctrl_pt_to_frc_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_tx_pb_req_set(false, false);
	if (pcfg->dual_enabled)
		iris_memc_ctrl_pt_frc_grcp_set_i7(true);
	else
		iris_memc_ctrl_pt_frc_meta_set_i7(true);

	if (pcfg->dual_enabled) {
		iris_blending_cusor_timeout_set_i7(true);
		iris_blending_al_coef_enable(true);
	}
}

void iris_memc_ctrl_frc_to_pt_i7(void)
{
	iris_memc_cmd_payload_init_i7();

	iris_memc_ctrl_pt_frc_meta_set_i7(false);

	iris_mcu_ctrl &= ~0x01;
	iris_mcu_state_set_i7(MCU_INT_DISABLE);

	iris_memc_cmd_payload_send_i7();
}

void iris_memc_ctrl_dual_frc_to_pt_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_memc_cmd_payload_init_i7();

	iris_set_pwil_mode_i7(PT_MODE, false, DSI_CMD_SET_STATE_HS, true);
	pcfg->frc_enabled = false;
	pcfg->dual_enabled = false;
	iris_input_frame_cnt_record_i7();

	iris_mcu_ctrl &= ~0x01;
	iris_mcu_state_set_i7(MCU_INT_DISABLE);

	iris_memc_cmd_payload_send_i7();
}

void iris_memc_ctrl_vfr_disable_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_frc_vfr_st) {
		iris_frc_vfr_threshold_set_i7(0, true);
		pcfg->iris_frc_vfr_st = false;
	}
}

void iris_memc_ctrl_pt_rfb_meta_set_i7(bool rfb_enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (rfb_enable) {
		/* send pwil meta cmd */
		iris_set_pwil_mode_i7(RFB_MODE, pcfg->dual_enabled, DSI_CMD_SET_STATE_HS, true);
	} else {
		/* send pwil meta cmd */
		iris_set_pwil_mode_i7(PT_MODE, pcfg->dual_enabled, DSI_CMD_SET_STATE_HS, true);
	}
}

void iris_memc_ctrl_rfb_prepare_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	/* power domain on */
	//iris_pmu_frc_set(true);
	iris_pmu_bsram_set(true);
	iris_pmu_dscu_set(true);
	/* disable Line lock before RFB mode in vin-vout*/
	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
		iris_linelock_set(false, true);
		iris_pwil0_efifo_enable_i7(true);
		iris_ovs_dly_change(true);
	}
}

void iris_dport_output_mode_reset_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 *payload = NULL;
	u32 dport_ctrl0, mode;

	if (pcfg->tx_mode == DSI_OP_CMD_MODE)
		mode = 2;
	else
		mode = 1;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPORT, 0xF0, 2);
	if (!payload)
		return;
	dport_ctrl0 = payload[0];
	dport_ctrl0 &= ~0xc000;
	dport_ctrl0 |= (mode & 0x3) << 14;

	payload[0] = dport_ctrl0;
	/* update cmdlist */
	iris_set_ipopt_payload_data(IRIS_IP_DPORT, 0xF0, 2, dport_ctrl0);
}

void iris_dport_output_mode_select_i7(bool enable)
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

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPORT, 0xF0, 2);
	if (!payload)
		return;
	dport_ctrl0 = payload[0];
	dport_ctrl0 &= ~0xc000;
	dport_ctrl0 |= (mode & 0x3) << 14;

	payload[0] = dport_ctrl0;
	iris_init_update_ipopt_t(IRIS_IP_DPORT, 0xF0, 0xF0, 0);
	iris_update_pq_opt(iris_pq_update_path, true);

	cmd[0] = IRIS_DPORT_CTRL0;
	cmd[1] = dport_ctrl0;
	cmd[2] = IRIS_DPORT_REGSEL;
	cmd[3] = 0x1;
	iris_ocp_write_mult_vals(4, cmd);
}

void iris_memc_ctrl_cmd_proc_i7(u32 cmd)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	switch (cmd) {
	case MEMC_CTRL_FRC_PREPARE:
		iris_memc_ctrl_frc_prepare_i7();
		IRIS_LOGI("MEMC_CTRL_FRC_PREPARE");
		iris_emv_set_frc_prepare_i7();
		break;
	case MEMC_CTRL_PT2FRC:
		if (iris_emv_set_frc_on_i7()) {
			if (pcfg->dual_enabled)
				iris_blending_al_coef_enable(true);
			break;
		}
		iris_mcu_version_read_i7();
		iris_memc_ctrl_pt_to_frc_i7();
		IRIS_LOGI("MEMC_CTRL_PT2FRC");
		break;
	case MEMC_CTRL_FRC_POST:
		/* in vin-vout mode, enter memc to need to enable efifo and dtg eco */
		if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
			iris_pwil0_efifo_enable_i7(false);
			iris_dtg_eco_i7(false, true);
			pcfg->dtg_eco_enabled = false;
		}
		IRIS_LOGI("MEMC_CTRL_FRC_POST");
		break;
	case MEMC_CTRL_PT_PREPARE:
		if (iris_emv_set_frc_off_i7())
			break;
		iris_memc_ctrl_frc_to_pt_i7();
		IRIS_LOGI("MEMC_CTRL_PT_PREPARE");
		break;
	case MEMC_CTRL_FRC2PT:
		iris_memc_ctrl_pt_post_i7();
		IRIS_LOGI("MEMC_CTRL_FRC2PT");
		break;
	case MEMC_CTRL_PT2RFB:
		iris_memc_ctrl_rfb_prepare_i7();
		iris_memc_ctrl_pt_rfb_meta_set_i7(true);
		IRIS_LOGI("MEMC_CTRL_PT2RFB");
		break;
	case MEMC_CTRL_RFB2PT:
		iris_memc_ctrl_pt_rfb_meta_set_i7(false);
		IRIS_LOGI("MEMC_CTRL_RFB2PT");
		break;
	case MEMC_CTRL_PT_POST:
		iris_memc_ctrl_pt_post_i7();
		IRIS_LOGI("MEMC_CTRL_PT_POST");
		break;
	case MEMC_CTRL_DPORT_DISABLE:
		iris_dport_output_mode_select_i7(false);
		IRIS_LOGI("MEMC_CTRL_DPORT_DISABLE");
		break;
	case MEMC_CTRL_DPORT_ENABLE:
		iris_dport_output_mode_select_i7(true);
		IRIS_LOGI("MEMC_CTRL_DPORT_ENABLE");
		break;
	case MEMC_CTRL_DUAL_FRC2PT:
		iris_memc_ctrl_dual_frc_to_pt_i7();
		IRIS_LOGI("MEMC_CTRL_DUAL_FRC2PT");
		break;
	case MEMC_CTRL_VFR_DISABLE:
		iris_memc_ctrl_vfr_disable_i7();
		IRIS_LOGI("MEMC_CTRL_VFR_DISABLE");
		break;
	case MEMC_CTRL_SWITCH_TIMEOUT:
		iris_switch_timeout_dump_i7();
		IRIS_LOGI("MEMC_CTRL_SWITCH_TIMEOUT");
	default:
		break;
	}
}

bool iris_memc_vfr_setting_update_i7(struct iris_cfg *pcfg, bool enable)
{
	if (!mutex_trylock(&pcfg->panel->panel_lock)) {
		IRIS_LOGI("%s:%d panel_lock is locked!", __func__, __LINE__);
		mutex_lock(&pcfg->panel->panel_lock);
	}
	if (!pcfg->memc_info.vfr_en || debug_disable_vfr_dual) {
		mutex_unlock(&pcfg->panel->panel_lock);
		IRIS_LOGI("vfr_en is disable, return");
		return false;
	}

	if (enable) {
		iris_frc_vfr_threshold_set_i7(iris_fi_drop_frm_thr, false);
		pcfg->iris_frc_vfr_st = true;
		IRIS_LOGI("enable vfr");
	} else {
		iris_frc_vfr_threshold_set_i7(0, false);
		pcfg->iris_frc_vfr_st = false;
		IRIS_LOGI("disable vfr");
	}
	mutex_unlock(&pcfg->panel->panel_lock);
	return true;
}

void iris_memc_vfr_video_update_monitor_i7(struct iris_cfg *pcfg, struct dsi_display *display)
{
	/* only update VFR in dual FRC mode*/
	if ((pcfg->iris_pwil_blend_st != 1) || (pcfg->pwil_mode != FRC_MODE))
		return;

	if (debug_disable_vfr_dual)
		return;

	if (iris_virtual_display(display)) {
		int video_update_wo_osd = atomic_read(&pcfg->video_update_wo_osd);

		IRIS_LOGV("clean video_update_wo_osd");
		atomic_set(&pcfg->video_update_wo_osd, 0);
		if (video_update_wo_osd >= 4) {
			cancel_work_sync(&pcfg->vfr_update_work);
			schedule_work(&pcfg->vfr_update_work);
		}
	} else {
		IRIS_LOGV("video_update_wo_osd: %d", atomic_read(&pcfg->video_update_wo_osd));
		atomic_inc(&pcfg->video_update_wo_osd);
		if (atomic_read(&pcfg->video_update_wo_osd) == 4) {
			cancel_work_sync(&pcfg->vfr_update_work);
			schedule_work(&pcfg->vfr_update_work);
		}
	}
}

void iris_memc_vfr_update_work_i7(struct work_struct *work)
{
	struct iris_cfg *pcfg = container_of(work, struct iris_cfg, vfr_update_work);

	if (atomic_read(&pcfg->video_update_wo_osd) >= 4) {
		iris_memc_vfr_setting_update_i7(pcfg, true);
	} else {
		iris_memc_vfr_setting_update_i7(pcfg, false);
	}
}

void iris_memc_vfr_update_work_init_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	INIT_WORK(&pcfg->vfr_update_work, iris_memc_vfr_update_work_i7);
}

int iris_debug_memc_option_get_i7(char *kbuf, int size)
{
	int len = 0;
	int index = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	len += snprintf(kbuf, size,
			"%d-%s: %x\n", index++, "memc_disable_vfr_dual", debug_disable_vfr_dual);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_vfr_enable_ow", debug_frc_vfr_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_disable_frc_dynen", debug_disable_frc_dynen);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_mcu_enable", iris_mcu_enable);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_disable_mcu_check", debug_disable_mcu_check);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_switch_dump_clear", debug_switch_dump_clear);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_dma_disable", iris_frc_dma_disable);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_tnr_enable_ow", debug_tnr_en_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_layer_c_enable", iris_frc_layer_c_enable);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_fi_drop_frm_thr", iris_fi_drop_frm_thr);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_sr_enable", iris_sr_enable);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_low_latency_ow", debug_low_latency_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_3buffer_low_latency", iris_three_buffer_low_latency);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_mv_res_ow", debug_mv_res_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_disable_mipi1_autorefresh", debug_disable_mipi1_autorefresh);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_n2m_mode_ow", debug_n2m_mode_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_force_repeat", debug_frc_force_repeat);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "ocp_read_by_i2c", pcfg->ocp_read_by_i2c);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_level_ow", debug_memc_level_overwrite);
	len += snprintf(kbuf + len, size - len,
			"%d-%s: %x\n", index++, "memc_judder_angle", judder_angle);
	return len;
}

void iris_debug_memc_option_set_i7(u32 type, u32 value)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	switch (type) {
	case 0:
		debug_disable_vfr_dual = value;
		break;
	case 1:
		debug_frc_vfr_overwrite = value;
		break;
	case 2:
		debug_disable_frc_dynen = value;
		break;
	case 3:
		iris_mcu_enable = value;
		break;
	case 4:
		debug_disable_mcu_check = value;
		break;
	case 5:
		pcfg->switch_dump.trigger = false;
		debug_switch_dump_clear = value;
		break;
	case 6:
		iris_frc_dma_disable = value;
		break;
	case 7:
		debug_tnr_en_overwrite = value;
		break;
	case 8:
		iris_frc_layer_c_enable = value;
		break;
	case 9:
		iris_fi_drop_frm_thr = value;
		break;
	case 10:
		iris_sr_enable = value;
		break;
	case 11:
		debug_low_latency_overwrite = value;
		break;
	case 12:
		iris_three_buffer_low_latency = value;
		break;
	case 13:
		debug_mv_res_overwrite = value;
		break;
	case 14:
		debug_disable_mipi1_autorefresh = value;
		break;
	case 15:
		debug_n2m_mode_overwrite = value;
		break;
	case 16:
		debug_frc_force_repeat = value;
		break;
	case 17:
		pcfg->ocp_read_by_i2c = value;
		break;
	case 18:
		debug_memc_level_overwrite = value;
		break;
	case 19:
		judder_angle = value;
		break;
	default:
		break;
	}
}

int iris_dbgfs_memc_init_i7(struct dsi_display *display)
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
	debugfs_create_u32("memc_level_ow", 0644, pcfg->dbg_root,
		(u32 *)&debug_memc_level_overwrite);
	debugfs_create_u32("memc_judder_angle", 0644, pcfg->dbg_root,
		(u32 *)&judder_angle);
	debugfs_create_u32("memc_disable_vfr_dual", 0644, pcfg->dbg_root,
		(u32 *)&debug_disable_vfr_dual);
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
	debugfs_create_u32("memc_dma_disable", 0644, pcfg->dbg_root,
			(u32 *)&iris_frc_dma_disable);
	debugfs_create_u32("memc_tnr_enable_ow", 0644, pcfg->dbg_root,
			(u32 *)&debug_tnr_en_overwrite);
	debugfs_create_u32("memc_layer_c_enable", 0644, pcfg->dbg_root,
			(u32 *)&iris_frc_layer_c_enable);
	debugfs_create_u32("memc_fi_drop_frm_thr", 0644, pcfg->dbg_root,
			(u32 *)&iris_fi_drop_frm_thr);
	debugfs_create_u32("memc_sr_enable", 0644, pcfg->dbg_root,
			(u32 *)&iris_sr_enable);
	debugfs_create_u32("memc_low_latency_ow", 0644, pcfg->dbg_root,
			(u32 *)&debug_low_latency_overwrite);
	debugfs_create_u32("memc_3buffer_low_latency", 0644, pcfg->dbg_root,
			(u32 *)&iris_three_buffer_low_latency);
	debugfs_create_u32("memc_mv_res_ow", 0644, pcfg->dbg_root,
			(u32 *)&debug_mv_res_overwrite);
	debugfs_create_u32("memc_disable_mipi1_autorefresh", 0644, pcfg->dbg_root,
			(u32 *)&debug_disable_mipi1_autorefresh);
	debugfs_create_u32("memc_n2m_mode_ow", 0644, pcfg->dbg_root,
		(u32 *)&debug_n2m_mode_overwrite);
	debugfs_create_u32("memc_force_repeat", 0644, pcfg->dbg_root,
		(u32 *)&debug_frc_force_repeat);
	debugfs_create_u32("memc_eco_overwrite", 0644, pcfg->dbg_root,
		(u32 *)&debug_eco_overwrite);

	iris_dbgfs_emv_init_i7(display);

	return 0;
}

//=====================================================================
#define FRACTION   13     // Fractional part of nHinc, nVinc, nHacc and nVacc;
int CalcRightOffset_i7(int nLeftOffset, int nInWidth, int nOutWidth,
			int nInc, int nInitPhase, int nTotalPhases)
{
	long long nDeltaAcc = ((long long)((nInWidth - nLeftOffset) * nTotalPhases -
			nInitPhase) << FRACTION) - (1 << (FRACTION-1));
	return (nOutWidth - 1 - (int)((nDeltaAcc + nInc - 1) / nInc));
}

static void frc_ioinc1d_config_i7(int nInHeight, int nInWidth, int nOutWidth,
				int nVSOutHeight, int m_VS_TAP_NUM,
				uint8_t ip, bool ioinc_sel, uint8_t chain)
{
	int PREC = m_VS_TAP_NUM == 5 ? 19 : 18;
	int nTotalPhases = m_VS_TAP_NUM == 5 ? 64 : 32;
	int m_nVinc0, m_nHinc0;
	int m_nVSTopOffset0, m_nVSInitPhase0, m_nVSBotOffset0, m_nVSMaxLines0;
	int m_nHSLeftOffset0, m_nHSInitPhase0, m_nHSRightOffset0, m_nHSMaxLines0;
	uint32_t temp;
	uint32_t *payload = NULL;
	int h_offset = 9;
	//ioinc1d downscaler switch
	payload = iris_get_ipopt_payload_data(ip, 0xf0, 4);
	if (!payload)
		return;
	IRIS_LOGI("%s, algo information: [0x%04x%04x], [0x%04x%04x]", __func__,
			nInWidth, nInHeight,
			nOutWidth, nVSOutHeight);

	m_nVinc0 = (int)(((long long)nInHeight << PREC) / (long long)nVSOutHeight);
	m_nVSTopOffset0 = (m_nVinc0 / 2 >> PREC) + (m_VS_TAP_NUM + 1) / 2;
	m_nVSInitPhase0 = (m_nVinc0 / 2 >> FRACTION) & (m_VS_TAP_NUM == 5 ? 0x3f : 0x1f);
	m_nVSBotOffset0 = CalcRightOffset_i7(m_nVSTopOffset0, nInHeight,
				nVSOutHeight, m_nVinc0, m_nVSInitPhase0, nTotalPhases);
	m_nVSMaxLines0 = nVSOutHeight;

	m_nHinc0 = (int)(((long long)nInWidth << PREC) / (long long)nOutWidth);
	m_nHSLeftOffset0 = (m_nHinc0 / 2 >> PREC) + (m_VS_TAP_NUM + 1) / 2;
	m_nHSInitPhase0 = (m_nHinc0 / 2 >> FRACTION) & (m_VS_TAP_NUM == 5 ? 0x3f : 0x1f);
	m_nHSRightOffset0 = CalcRightOffset_i7(m_nHSLeftOffset0, nInWidth,
				nOutWidth, m_nHinc0, m_nHSInitPhase0, nTotalPhases);
	m_nHSMaxLines0 = nOutWidth;

	if (ioinc_sel) {
		payload[1] = m_nVinc0;
		temp = payload[2] & 0x3f;
		payload[2] = temp | (m_nVSInitPhase0 << 6) | (7 << 12);
		temp = payload[3] & 0xfff;
		payload[3] = temp | (m_nVSMaxLines0 << 16);
		payload[5] = m_nVSTopOffset0 | (m_nVSBotOffset0 << 16);
	} else {
		payload[0] = m_nVinc0;
		temp = (payload[2] >> 6) & 0x3f;
		payload[2] = m_nVSInitPhase0 | (temp << 6) | (7 << 12);
		temp = (payload[3] >> 16) & 0xfff;
		payload[3] = m_nVSMaxLines0 | (temp << 16);
		payload[4] = m_nVSTopOffset0 | (m_nVSBotOffset0 << 16);
	}
	IRIS_LOGD("%s, ioinc V information: [%x], [%x], [%x], [%x], [%x], [%x]", __func__,
				payload[0], payload[1],
				payload[2], payload[3],
				payload[4], payload[5]);

	if (ioinc_sel) {
		payload[1+h_offset] = m_nHinc0;
		temp = payload[2+h_offset] & 0x3f;
		payload[2+h_offset] = temp | (m_nHSInitPhase0 << 6) | (7 << 12);
		temp = payload[3+h_offset] & 0xfff;
		payload[3+h_offset] = temp | (m_nHSMaxLines0 << 16);
		payload[5+h_offset] = m_nHSLeftOffset0 | (m_nHSRightOffset0 << 16);
	} else {
		payload[0+h_offset] = m_nHinc0;
		temp = (payload[2+h_offset] >> 6) & 0x3f;
		payload[2+h_offset] = m_nHSInitPhase0 | (temp << 6) | (7 << 12);
		temp = (payload[3+h_offset] >> 16) & 0xfff;
		payload[3+h_offset] = m_nHSMaxLines0 | (temp << 16);
		payload[4+h_offset] = m_nHSLeftOffset0 | (m_nHSRightOffset0 << 16);
	}
	IRIS_LOGD("%s, ioinc H information: [%x], [%x], [%x], [%x], [%x], [%x]", __func__,
				payload[0+h_offset], payload[1+h_offset],
				payload[2+h_offset], payload[3+h_offset],
				payload[4+h_offset], payload[5+h_offset]);

	iris_sync_current_ipopt(ip, 0xf0);
	iris_init_update_ipopt_t(ip, 0xf0, 0xf0, chain);
}

static void dsc_create_pps_buf_cmd_i7(struct dsc_info *dsc)
{
	char buf[88], *bp, data;
	int i, bpp, pps_id = 0;

	bp = buf;

	*bp++ = (dsc->version & 0xff);		/* pps0 */
	*bp++ = (pps_id & 0xff);		/* pps1 */
	*bp++ = 0;					/* pps2, reserved */

	data = dsc->line_buf_depth & 0x0f;
	data |= ((dsc->bpc & 0xf) << 4);
	*bp++ = data;				/* pps3 */

	bpp = dsc->bpp;
	//bpp <<= 4;				/* 4 fraction bits */
	data = (bpp >> 8);
	data &= 0x03;				/* upper two bits */
	data |= ((dsc->block_pred_enable & 0x1) << 5);
	data |= ((dsc->convert_rgb & 0x1) << 4);
	data |= ((dsc->enable_422 & 0x1) << 3);
	data |= ((dsc->vbr_enable & 0x1) << 2);
	*bp++ = data;				/* pps4 */
	*bp++ = (bpp & 0xff);			/* pps5 */

	*bp++ = ((dsc->pic_height >> 8) & 0xff); /* pps6 */
	*bp++ = (dsc->pic_height & 0x0ff);	/* pps7 */
	*bp++ = ((dsc->pic_width >> 8) & 0xff);	/* pps8 */
	*bp++ = (dsc->pic_width & 0x0ff);	/* pps9 */

	*bp++ = ((dsc->slice_height >> 8) & 0xff);/* pps10 */
	*bp++ = (dsc->slice_height & 0x0ff);	/* pps11 */
	*bp++ = ((dsc->slice_width >> 8) & 0xff); /* pps12 */
	*bp++ = (dsc->slice_width & 0x0ff);	/* pps13 */

	*bp++ = ((dsc->chunk_size >> 8) & 0xff);/* pps14 */
	*bp++ = (dsc->chunk_size & 0x0ff);	/* pps15 */

	*bp++ = (dsc->initial_xmit_delay >> 8) & 0x3; /* pps16, bit 0, 1 */
	*bp++ = (dsc->initial_xmit_delay & 0xff);/* pps17 */

	*bp++ = ((dsc->initial_dec_delay >> 8) & 0xff); /* pps18 */
	*bp++ = (dsc->initial_dec_delay & 0xff);/* pps19 */

	*bp++ = 0;				/* pps20, reserved */

	*bp++ = (dsc->initial_scale_value & 0x3f); /* pps21 */

	*bp++ = ((dsc->scale_increment_interval >> 8) & 0xff); /* pps22 */
	*bp++ = (dsc->scale_increment_interval & 0xff); /* pps23 */

	*bp++ = ((dsc->scale_decrement_interval >> 8) & 0xf); /* pps24 */
	*bp++ = (dsc->scale_decrement_interval & 0x0ff);/* pps25 */

	*bp++ = 0;				/* pps26, reserved */

	*bp++ = (dsc->first_line_bpg_offset & 0x1f);/* pps27 */

	*bp++ = ((dsc->nfl_bpg_offset >> 8) & 0xff);/* pps28 */
	*bp++ = (dsc->nfl_bpg_offset & 0x0ff);	/* pps29 */
	*bp++ = ((dsc->slice_bpg_offset >> 8) & 0xff);/* pps30 */
	*bp++ = (dsc->slice_bpg_offset & 0x0ff);/* pps31 */

	*bp++ = ((dsc->initial_offset >> 8) & 0xff);/* pps32 */
	*bp++ = (dsc->initial_offset & 0x0ff);	/* pps33 */

	*bp++ = ((dsc->final_offset >> 8) & 0xff);/* pps34 */
	*bp++ = (dsc->final_offset & 0x0ff);	/* pps35 */

	*bp++ = (dsc->min_qp_flatness & 0x1f);	/* pps36 */
	*bp++ = (dsc->max_qp_flatness & 0x1f);	/* pps37 */

	*bp++ = ((dsc->rc_model_size >> 8) & 0xff);/* pps38 */
	*bp++ = (dsc->rc_model_size & 0x0ff);	/* pps39 */

	*bp++ = (dsc->edge_factor & 0x0f);	/* pps40 */

	*bp++ = (dsc->quant_incr_limit0 & 0x1f);	/* pps41 */
	*bp++ = (dsc->quant_incr_limit1 & 0x1f);	/* pps42 */

	data = ((dsc->tgt_offset_hi & 0xf) << 4);
	data |= (dsc->tgt_offset_lo & 0x0f);
	*bp++ = data;				/* pps43 */

	for (i = 0; i < 14; i++)
		*bp++ = (dsc->buf_thresh[i] & 0xff); /* pps44 - pps57 */

	for (i = 0; i < 15; i++) {		/* pps58 - pps87 */
		data = (dsc->range_min_qp[i] & 0x1f);
		data <<= 3;
		data |= ((dsc->range_max_qp[i] >> 2) & 0x07);
		*bp++ = data;
		data = (dsc->range_max_qp[i] & 0x03);
		data <<= 6;
		data |= (dsc->range_bpg_offset[i] & 0x3f);
		*bp++ = data;
	}

	for (i = 0; i < 88; ) {
		dsc_pps_table[i/4] = buf[i] | (buf[i+1] << 8) |
				(buf[i+2] << 16) | (buf[i+3] << 24);
		IRIS_LOGD("pps table: %x, %x, %x, %x", buf[i], buf[i+1], buf[i+2], buf[i+3]);
		i += 4;
	}
}

static void dsc_populate_static_param_i7(struct dsc_info *dsc)
{
	int bpp, bpc;
	int mux_words_size;
	int groups_per_line, groups_total;
	int min_rate_buffer_size;
	int hrd_delay;
	int pre_num_extra_mux_bits, num_extra_mux_bits;
	int slice_bits;
	int data;
	int final_value, final_scale;
	int ratio_index = IRIS_DSC_8BPC_65BPP, mod_offset;

	dsc->version = 0x11;
	dsc->scr_rev = 0;
	dsc->rc_model_size = 8192;
	dsc->first_line_bpg_offset = 12;

	dsc->edge_factor = 6;
	dsc->tgt_offset_hi = 3;
	dsc->tgt_offset_lo = 3;
	dsc->enable_422 = 1;
	dsc->convert_rgb = 0;
	dsc->vbr_enable = 1;
	dsc->block_pred_enable = 1;

	dsc->buf_thresh = dsc_rc_buf_thresh;

	bpp = dsc->bpp;
	bpc = dsc->bpc;

	if ((bpc == 10) && (bpp == 104))  //10bpc-6.5bpp
		ratio_index = IRIS_DSC_10BPC_65BPP;
	else if ((bpc == 8) && (bpp == 104))  //8bpc-6.5bpp
		ratio_index = IRIS_DSC_8BPC_65BPP;
	else
		IRIS_LOGE("(%s, %d) Don't support this type!!!!!!!", __func__, __LINE__);


	dsc->range_min_qp = dsc_rc_range_min_qp_1_1[ratio_index];
	dsc->range_max_qp = dsc_rc_range_max_qp_1_1[ratio_index];
	dsc->range_bpg_offset = dsc_rc_range_bpg_offset;

	dsc->initial_offset = 6144;
	dsc->line_buf_depth = bpc + 1;

	if (bpc == 8) {
		dsc->input_10_bits = 0;
		dsc->min_qp_flatness = 3;
		dsc->max_qp_flatness = 12;
		dsc->quant_incr_limit0 = 11;
		dsc->quant_incr_limit1 = 11;
		mux_words_size = 48;
	} else if (bpc == 10) { /* 10bpc */
		dsc->input_10_bits = 1;
		dsc->min_qp_flatness = 7;
		dsc->max_qp_flatness = 16;
		dsc->quant_incr_limit0 = 15;
		dsc->quant_incr_limit1 = 15;
		mux_words_size = 48;
	} else { /* 12 bpc */
		dsc->input_10_bits = 0;
		dsc->min_qp_flatness = 11;
		dsc->max_qp_flatness = 20;
		dsc->quant_incr_limit0 = 19;
		dsc->quant_incr_limit1 = 19;
		mux_words_size = 64;
	}

	mod_offset = dsc->slice_width % 3;
	switch (mod_offset) {
	case 0:
		dsc->slice_last_group_size = 2;
		break;
	case 1:
		dsc->slice_last_group_size = 0;
		break;
	case 2:
		dsc->slice_last_group_size = 1;
		break;
	default:
		break;
	}

	dsc->det_thresh_flatness = 2 << (bpc - 8);
	dsc->initial_xmit_delay = (dsc->rc_model_size * 16)/(bpp * 2);
	groups_per_line = CEILING(dsc->slice_width, 3);

	dsc->chunk_size = dsc->slice_width * bpp/16 / 8;
	if ((dsc->slice_width * bpp/16) % 8)
		dsc->chunk_size++;

	/* rbs-min */
	min_rate_buffer_size =  dsc->rc_model_size - dsc->initial_offset +
			dsc->initial_xmit_delay * bpp / 16 +
			groups_per_line * dsc->first_line_bpg_offset;

	hrd_delay = CEILING(min_rate_buffer_size*16, bpp);

	dsc->initial_dec_delay = hrd_delay - dsc->initial_xmit_delay;

	dsc->initial_scale_value = 8 * dsc->rc_model_size /
			(dsc->rc_model_size - dsc->initial_offset);

	slice_bits = 8 * dsc->chunk_size * dsc->slice_height;

	groups_total = groups_per_line * dsc->slice_height;

	data = dsc->first_line_bpg_offset * 2048;

	dsc->nfl_bpg_offset = CEILING(data, (dsc->slice_height - 1));

	pre_num_extra_mux_bits = 3 * (mux_words_size + (4 * bpc + 4) - 2);

	num_extra_mux_bits = pre_num_extra_mux_bits - (mux_words_size -
		((slice_bits - pre_num_extra_mux_bits) % mux_words_size));

	data = 2048 * (dsc->rc_model_size - dsc->initial_offset
		+ num_extra_mux_bits);
	dsc->slice_bpg_offset = CEILING(data, groups_total);

	data = dsc->initial_xmit_delay * bpp / 16;
	final_value =  dsc->rc_model_size - data + num_extra_mux_bits;

	final_scale = 8 * dsc->rc_model_size /
		(dsc->rc_model_size - final_value);

	dsc->final_offset = final_value;

	data = (final_scale - 9) * (dsc->nfl_bpg_offset +
		dsc->slice_bpg_offset);
	dsc->scale_increment_interval = (2048 * dsc->final_offset) / data;

	dsc->scale_decrement_interval = groups_per_line /
		(dsc->initial_scale_value - 8);

}

static void dsc_encoder_ctrl_parameters_i7(struct dsc_info *dsc)
{
	struct dsc_encoder_parameters dsc_enc_ctrl;
	u32 nRemainRows, nRemChunks;
	u32 nslice_height, nslice_width, npic_height, npic_width;
	u32 nchunksize, nExtraBytes;
	u32 nRcmodelsize, initoffset, nflbpgoffset,  nGrpsPerLine;
	struct iris_cfg *pcfg = iris_get_cfg();

	nslice_height = dsc->slice_height;
	nslice_width = dsc->slice_width;
	npic_height = dsc->pic_height;
	npic_width = dsc->pic_width;
	nchunksize = dsc->chunk_size;
	nRcmodelsize = dsc->rc_model_size;
	initoffset = dsc->initial_offset;
	nflbpgoffset = dsc->nfl_bpg_offset;
	nGrpsPerLine = (dsc->slice_width + 2)/3;
	nRemainRows = dsc->pic_height % dsc->slice_height;

	dsc_enc_ctrl.slice_num  = dsc->pic_width / dsc->slice_width;
	dsc_enc_ctrl.last_hslice_size = dsc->pic_width % dsc->slice_width;
	if (dsc_enc_ctrl.last_hslice_size == 0)
		dsc_enc_ctrl.last_hslice_size = dsc->slice_width;

	dsc_enc_ctrl.last_hslice_groupnum = (dsc_enc_ctrl.last_hslice_size + 2)/3;
	dsc_enc_ctrl.groupnum_persliceline = (dsc->slice_width + 2)/3;
	dsc_enc_ctrl.groupnum_perline = dsc_enc_ctrl.groupnum_persliceline *
					dsc_enc_ctrl.slice_num;
	dsc_enc_ctrl.pnum_persliceline = dsc_enc_ctrl.groupnum_persliceline * 3;
	dsc_enc_ctrl.groupnum_perslice = dsc_enc_ctrl.groupnum_persliceline;
	dsc_enc_ctrl.len_codedslice = dsc->chunk_size * dsc->slice_height;
	dsc_enc_ctrl.len8_codedslice = CEILING(dsc->chunk_size, 8) * 8 * dsc->slice_height;
	dsc_enc_ctrl.stream_hsize = (dsc->chunk_size *
			((dsc->pic_width + dsc->slice_width - 1)/dsc->slice_width) + 3) >> 2;
	dsc_enc_ctrl.dwpercodedslice = dsc_enc_ctrl.stream_hsize * dsc->slice_height;
	dsc_enc_ctrl.last_slice_vsize = dsc->pic_height % dsc->slice_height;


	if (nRemainRows == 0)
		nRemChunks = 0;
	else {
		nExtraBytes = ((nRcmodelsize - initoffset) * (nslice_height - nRemainRows) /
				nslice_height + nflbpgoffset * nGrpsPerLine *
				(nslice_height - nRemainRows) / (nslice_height - 1) + 7) >> 3;
		nRemChunks = ((nExtraBytes + nchunksize - 1) / nchunksize);
		if (nRemChunks > (nslice_height - nRemainRows))
			nRemChunks = nslice_height - nRemainRows;
	}

	enc_ctrl_reg.slice_size2 =
		(dsc_enc_ctrl.groupnum_perline & 0x3fff) |
			((dsc_enc_ctrl.groupnum_persliceline & 0x3fff) << 16);
	enc_ctrl_reg.slice_size3 =
		(dsc_enc_ctrl.pnum_persliceline & 0x3fff) |
			((dsc_enc_ctrl.slice_num & 0xf) << 16);
	enc_ctrl_reg.stream_size =
		(dsc_enc_ctrl.stream_hsize & 0x3fff) |
			((dsc_enc_ctrl.groupnum_perslice & 0x3fff) << 16);
	enc_ctrl_reg.len_codedslice = dsc_enc_ctrl.len_codedslice;
	enc_ctrl_reg.len8_codedslice = dsc_enc_ctrl.len8_codedslice;
	enc_ctrl_reg.dwpercodedslice = dsc_enc_ctrl.dwpercodedslice;
	if (pcfg->frc_setting.pps_table_sel == 0)
		enc_ctrl_reg.ctrl_setting = (pcfg->frc_setting.dsc_enc_ctrl0 & 0xe000700f) |
						((nRemChunks & 0xff) << 4) |
						((dsc_enc_ctrl.last_slice_vsize & 0x3fff) << 15);
	else
		enc_ctrl_reg.ctrl_setting = (pcfg->frc_setting.dsc_enc_ctrl1 & 0xe000700f) |
						((nRemChunks & 0xff) << 4) |
						((dsc_enc_ctrl.last_slice_vsize & 0x3fff) << 15);
	enc_ctrl_reg.last_hslice = (dsc_enc_ctrl.last_hslice_size & 0xfff) |
						((dsc_enc_ctrl.last_hslice_groupnum & 0x3ff) << 16);
}

void iris_frc_dsc_change_i7(void)
{
	uint8_t  chain = 0;
	int len, i;
	uint32_t *payload = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	frc_dsc_info.pic_width = pcfg->frc_setting.mv_hres;
	frc_dsc_info.pic_height = pcfg->frc_setting.mv_vres;
	frc_dsc_info.slice_height = frc_dsc_info.pic_height;
	frc_dsc_info.slice_width = frc_dsc_info.pic_width;
	frc_dsc_info.bpp = pcfg->frc_setting.mv_coef;
	frc_dsc_info.bpc = 10;
	if (pcfg->memc_info.memc_mode == MEMC_DUAL_EXTMV_ENABLE)
		frc_dsc_info.bpc = iris_emv_get_path_bitwidth_i7();

	IRIS_LOGD("%s, dsc information: [%d], [%d], [%d], [%d], [%d], [%d]", __func__,
				frc_dsc_info.pic_width, frc_dsc_info.pic_height,
				frc_dsc_info.slice_height, frc_dsc_info.slice_width,
				frc_dsc_info.bpp, frc_dsc_info.bpc);

	dsc_populate_static_param_i7(&frc_dsc_info);
	dsc_create_pps_buf_cmd_i7(&frc_dsc_info);
	dsc_encoder_ctrl_parameters_i7(&frc_dsc_info);

	if (!iris_frc_dma_disable)
		chain = 1;
	//pps table switch.
	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_DEN_2, 0xf1, 5);
	if (!payload)
		return;
	for (i = 0; i < 11; i++) {
		if ((iris_frc_dec_initial_delay != 0) && (i == 4))
			continue;
		payload[i + 22*pcfg->frc_setting.pps_table_sel] = dsc_pps_table[i];
	}

	iris_sync_current_ipopt(IRIS_IP_DSC_DEN_2, 0xf1);
	len  = iris_init_update_ipopt_t(IRIS_IP_DSC_DEN_2,
				0xf1, 0xf1, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_DEN_3, 0xf1, 5);
	if (!payload)
		return;
	for (i = 0; i < 11; i++) {
		if ((iris_frc_dec_initial_delay != 0) && (i == 4))
			continue;
		payload[i + 22*pcfg->frc_setting.pps_table_sel] = dsc_pps_table[i];
	}

	iris_sync_current_ipopt(IRIS_IP_DSC_DEN_3, 0xf1);
	len  = iris_init_update_ipopt_t(IRIS_IP_DSC_DEN_3,
				0xf1, 0xf1, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_ENC_2, 0xf1, 21);
	if (!payload)
		return;
	for (i = 0; i < 11; i++)
		payload[i + 22*pcfg->frc_setting.pps_table_sel] = dsc_pps_table[i];
	len  = iris_init_update_ipopt_t(IRIS_IP_DSC_ENC_2,
				0xf1, 0xf1, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_ENC_TNR, 0xf1, 21);
	if (!payload)
		return;
	for (i = 0; i < 11; i++)
		payload[i + 22*pcfg->frc_setting.pps_table_sel] = dsc_pps_table[i];
	len  = iris_init_update_ipopt_t(IRIS_IP_DSC_ENC_TNR, 0xf1, 0xf1, 1);

	//encoder ctrl register.
	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_ENC_2, 0xf1, 4);
	if (!payload)
		return;
	payload[0 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.slice_size2;
	payload[1 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.slice_size3;
	payload[2 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.stream_size;
	payload[3 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.len_codedslice;
	payload[4 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.len8_codedslice;
	payload[5 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.dwpercodedslice;
	payload[6 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.ctrl_setting;
	payload[7 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.last_hslice;

	iris_sync_current_ipopt(IRIS_IP_DSC_ENC_2, 0xf1);
	len  = iris_init_update_ipopt_t(IRIS_IP_DSC_ENC_2,
				0xf1, 0xf1, 1);

	//encoder ctrl register.
	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_ENC_TNR, 0xf1, 4);
	if (!payload)
		return;
	payload[0 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.slice_size2;
	payload[1 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.slice_size3;
	payload[2 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.stream_size;
	payload[3 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.len_codedslice;
	payload[4 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.len8_codedslice;
	payload[5 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.dwpercodedslice;
	payload[6 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.ctrl_setting;
	payload[7 + 8*pcfg->frc_setting.pps_table_sel] = enc_ctrl_reg.last_hslice;

	iris_sync_current_ipopt(IRIS_IP_DSC_ENC_TNR, 0xf1);
	len  = iris_init_update_ipopt_t(IRIS_IP_DSC_ENC_TNR,
				0xf1, 0xf1, chain);

	/* trigger DMA for PWIL */
	//if (!iris_frc_dma_disable)
	//	len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe8, 0xe8, 0);

	iris_update_pq_opt(iris_pq_update_path, true);
}

void iris_frc_ioinc_change_i7(void)
{
	uint8_t  chain = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	int tap_sel = 5;
	int frc_ioinc_sel = 0;

	if (!iris_frc_dma_disable)
		chain = 1;

	frc_ioinc1d_config_i7(iris_frc_in1_height_get_i7(), iris_frc_in1_width_get_i7(),
			pcfg->frc_setting.mv_hres, pcfg->frc_setting.mv_vres,
			tap_sel, IRIS_IP_IOINC1D, frc_ioinc_sel, chain);

	IRIS_LOGI("ioinc: in %d,[0x%04x%04x]->[0x%04x%04x],[0x%04x%04x],"
	    "[0x%04x%04x],[0x%04x%04x][0x%04x%04x]->[0x%04x%04x]",
		pcfg->memc_info.memc_mode,
		iris_frc_in1_width_get_i7(),
		iris_frc_in1_height_get_i7(),
		pcfg->frc_setting.mv_hres,
		pcfg->frc_setting.mv_vres,
		pcfg->emv_info.gameWidth,
		pcfg->emv_info.gameHeight,
		pcfg->emv_info.gameWidthSrc,
		pcfg->emv_info.gameHeightSrc,
		pcfg->emv_info.gameWidthSrc&(~(u32)0x1),
		pcfg->emv_info.gameHeightSrc&(~(u32)0x1),
		iris_frc_in2_width_get_i7(),
		iris_frc_in2_height_get_i7(),
		pcfg->frc_setting.disp_hres,
		pcfg->frc_setting.disp_vres);

	/* trigger DMA for PWIL */
	//if (!iris_frc_dma_disable)
	//	len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe0, 0xe0, 0);

	iris_update_pq_opt(iris_pq_update_path, true);
}

void iris_sr_change_i7(int inHeight, int inWidth, int outHeight, int outWidth, bool sr_sel, bool update)
{
	uint8_t  chain = 0;
	uint32_t *payload = NULL;
	uint32_t nhinit_phase = 0x10, nvinit_phase = 0x10, nhinc, nvinc;
	int PREC = 19;

	if (!iris_frc_dma_disable)
		chain = 1;

	nvinc = (int)(((long long)inHeight << PREC) / (long long)outHeight);
	nhinc = (int)(((long long)inWidth << PREC) / (long long)outWidth);

	//sr
	payload = iris_get_ipopt_payload_data(IRIS_IP_SR, 0xe0, 2);
	if (!payload)
		return;
	if (sr_sel) {
		payload[3] = (nhinc << 8) | nhinit_phase;
		payload[4] = (nvinc << 8) | nvinit_phase;
		payload[6] = (outWidth << 16) | outHeight;
	} else {
		payload[1] = (nhinc << 8) | nhinit_phase;
		payload[2] = (nvinc << 8) | nvinit_phase;
		payload[5] = (outWidth << 16) | outHeight;
	}

	iris_sync_current_ipopt(IRIS_IP_SR, 0xe0);

	if (update) {
		iris_init_update_ipopt_t(IRIS_IP_SR, 0xe0, 0xe0, chain);

		iris_update_pq_opt(iris_pq_update_path, true);
	} else
		iris_init_update_ipopt_t(IRIS_IP_SR, 0xe0, 0xe0, 1);
}

static void iris_sr_pwil_update_i7(bool enable, int processWidth, int processHeight, uint8_t chain)
{
	uint32_t *payload = NULL;
	u32 cmd[6];

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xf0, 5);
	if (!payload)
		return;
	if (enable)
		payload[0] = BITS_SET(payload[0], 2, 9, 0);
	else
		payload[0] = BITS_SET(payload[0], 2, 9, 1);
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xf0, 0xf0, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xA0, 2);
	if (!payload)
		return;
	if (enable)
		payload[1] = BITS_SET(payload[1], 4, 19, 15);
	else
		payload[1] = BITS_SET(payload[1], 4, 19, 0);
	payload[5] = processWidth | processHeight << 16;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xA0, 0xA0, chain);

	if (enable == 0 && iris_dynamic_power_get()) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL1;
		cmd[1] = payload[1];
		cmd[2] = IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL5;
		cmd[3] = payload[5];
		cmd[4] = IRIS_PWIL_ADDR + PWIL_REG_UPDATE;
		cmd[5] = 0x00000100;

		iris_ocp_write_mult_vals(6, cmd);
	}

	if (!chain)
		iris_update_pq_opt(iris_pq_update_path, true);
}

static bool iris_pmu_frc_get_i7(void)
{
	uint32_t *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SYS, ID_SYS_MPG_I7, 2);
	if (!payload)
		return 0;
	return (payload[0] & FRC_PWR_I7) != 0;
}

static void iris_sr_reg_sel_clean_i7(void)
{
	u32 cmd[2];

	cmd[0] = 0xf1800054;
	cmd[1] = 0x0;
	iris_ocp_write_mult_vals(2, cmd);
}

void iris_sr_update_i7(void)
{
	if (iris_pmu_frc_get_i7() == false) {
		IRIS_LOGI("FRC Power up");
		iris_pmu_frc_set(true);
	} else {
		IRIS_LOGI("FRC DMA Trigger");
		iris_sr_reg_sel_clean_i7();
		iris_dma_trig(DMA_CH2, 0);
		iris_update_pq_opt(iris_pq_update_path, true);
	}
}

void iris_pt_sr_set_i7(int enable, int processWidth, int processHeight)
{
	int tap_sel = 5;
	int pt_scale_sel = 1;	// TODO
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->pt_sr_enable = enable & 0x1;
	if (pcfg->pt_sr_enable) {
		pcfg->pt_sr_hsize = processWidth;
		pcfg->pt_sr_vsize = processHeight;
		iris_sdr2hdr_update_size2(pcfg->pt_sr_hsize, pcfg->pt_sr_vsize);
	} else {
		pcfg->pt_sr_hsize = pcfg->frc_setting.disp_hres;
		pcfg->pt_sr_vsize = pcfg->frc_setting.disp_vres;
	}

	iris_sr_pwil_update_i7(pcfg->pt_sr_enable, pcfg->pt_sr_hsize, pcfg->pt_sr_vsize, 1);

	// iris_update_pq_opt update in below iris_sr_change
	frc_ioinc1d_config_i7(pcfg->frc_setting.disp_vres,
			pcfg->frc_setting.disp_hres,
			pcfg->pt_sr_hsize, pcfg->pt_sr_vsize,
			tap_sel, IRIS_IP_IOINC1D, pt_scale_sel, 1);

	iris_sr_change_i7(pcfg->pt_sr_vsize,
		pcfg->pt_sr_hsize,
		pcfg->frc_setting.disp_vres,
		pcfg->frc_setting.disp_hres,
		pt_scale_sel,
		false);// graphic_sr_in_sel

	if (enable == 1) {
		iris_ioinc_filter_ratio_send();
		iris_sr_update_i7();
		iris_dma_trig(DMA_CH0 | DMA_CH12, 0);
		iris_update_pq_opt(iris_pq_update_path, true);
	} else if (enable == 0) {
		iris_dma_trig(DMA_CH0 | DMA_CH12, 0);
		iris_update_pq_opt(iris_pq_update_path, true);
		iris_pmu_frc_set(false);
	}

	/* debug code */
	if (enable >= 2) {
		if (enable & 0x2)
			iris_sr_update_i7();
		else
			iris_pmu_frc_set(false);
		if (enable & 0x4) {
			iris_dma_trig(DMA_CH0, 0);
			iris_update_pq_opt(iris_pq_update_path, true);
		}
		if (enable & 0x8) {
			iris_dma_trig(DMA_CH12, 0);
			iris_update_pq_opt(iris_pq_update_path, true);
		}
	}
}

void iris_pt_sr_reset_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->pt_sr_enable) {
		iris_sr_pwil_update_i7(false, pcfg->frc_setting.disp_hres,
				pcfg->frc_setting.disp_vres, 1);
		pcfg->pt_sr_enable = false;
		iris_dma_trig(DMA_CH12, 0);
		iris_update_pq_opt(iris_pq_update_path, true);
		iris_dma_trig(DMA_CH0, 0);
		iris_update_pq_opt(iris_pq_update_path, true);
		iris_pmu_frc_set(false);
		iris_sdr2hdr_set_degain();
	}
}

void iris_pt_sr_restore_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->pt_sr_enable_restore == false)
		return;

	iris_sr_level_set(PT_MODE, pcfg->pt_sr_guided_level,
		pcfg->pt_sr_dejaggy_level,
		pcfg->pt_sr_peaking_level,
		pcfg->pt_sr_DLTI_level);
	iris_pt_sr_set_i7(true,
		pcfg->pt_sr_hsize,
		pcfg->pt_sr_vsize);
	iris_sdr2hdr_set_degain();
}

void iris_debug_info_get_i7(u32 *value, u32 count)
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
		len = iris_debug_memc_option_get_i7(buf, count);
		break;
	case 3:
		len = iris_debug_pq_info_get(buf, count, true);
		break;
	default:
		break;
	}
	if (len < count)
		buf[len] = '\0';
}

int iris_configure_memc_i7(u32 type, u32 value)
{
	int rc = 0;

	switch (type) {
	case IRIS_MODE_SET:
		iris_emv_mode_switch_i7(value);
		break;
	case IRIS_OUT_FRAME_RATE_SET:
		iris_set_out_frame_rate_i7(value);
		break;
	case IRIS_DUAL_CH_CTRL:
		iris_dual_ch_ctrl_cmd_proc_i7(value);
		break;
	case IRIS_MEMC_CTRL:
		iris_memc_ctrl_cmd_proc_i7(value);
		break;
	case USER_DEMO_WND:
		iris_fi_demo_window_set_i7(value);
		break;
	case IRIS_OSD_AUTOREFRESH:
		iris_osd_auto_refresh_enable_i7(value);
		break;
	case IRIS_PANEL_TE:
		iris_set_panel_te_i7(value);
		break;
	case IRIS_N2M_ENABLE:
		if (value > 1) {
			IRIS_LOGI("%s(), enable N2M function in PT mode", __func__);
			iris_set_n2m_enable_i7(true, value);
			iris_dtg_te_n2m_ctrl_setting_send_i7(true);
		}
		if (value == 0) {
			IRIS_LOGI("%s(), disable N2M function in PT mode", __func__);
			iris_set_n2m_enable_i7(false, 1);
			iris_dtg_te_n2m_ctrl_setting_send_i7(false);
		}
		break;
	default:
		IRIS_LOGI("%s: type = %d value = %d", __func__, type, value);
		break;
	}

	return rc;
}

int iris_configure_ex_memc_i7(u32 type, u32 count, u32 *values)
{
	int rc = 0;

	switch (type) {
	case IRIS_SET_MVD_META:
		onConfigureMvdMeta_i7(count, values);
		break;
	case IRIS_MEMC_INFO_SET:
		iris_memc_info_set_i7(values);
		break;
	case IRIS_DEBUG_SET:
		iris_debug_memc_option_set_i7(values[0], values[1]);
		break;
	default:
		IRIS_LOGI("%s: type = %d count = %d", __func__, type, count);
		break;
	}

	return rc;
}

int iris_configure_get_memc_i7(u32 type, u32 count, u32 *values)
{
	int rc = 0;
	u32 get_op = 0;

	switch (type) {
	case IRIS_OSD_OVERFLOW_ST:
		*values = iris_osd_overflow_status_get_i7();
		break;
	case IRIS_KERNEL_STATUS_GET:
		get_op = *values;
		iris_kernel_multistatus_get_i7(get_op, count, values);
		break;
	case IRIS_DEBUG_GET:
		iris_debug_info_get_i7(values, count);
		break;
	default:
		IRIS_LOGI("%s: type = %d count = %d", __func__, type, count);
		break;
	}

	return rc;
}

void iris_init_memc_i7(void)
{
	//iris_frc_setting_init_i7();

	iris_memc_vfr_update_work_init_i7();
	iris_memc_status_clear_i7(true);
	iris_dual_status_clear_i7(true);
}

void iris_lightoff_memc_i7(void)
{
	iris_memc_status_clear_i7(true);
	iris_dual_status_clear_i7(true);

	iris_pwil0_efifo_setting_reset_i7();
	iris_dport_output_mode_reset_i7();
	iris_set_pwil_mode_i7(PT_MODE, false, DSI_CMD_SET_STATE_HS, false);

	iris_emv_on_lightoff_i7();
	iris_osd_comp_ready_pad_select_i7(false, false);
	iris_mcu_mode_reset_i7();
}

void iris_enable_memc_i7(struct dsi_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (panel->is_secondary) {
		if (pcfg->iris_osd_autorefresh_enabled) {
			IRIS_LOGI("reset iris_osd_autorefresh");
			iris_osd_auto_refresh_enable_i7(0);
		}
		iris_mipi1_enable_i7();
		pcfg->ap_mipi1_power_st = true;
		pcfg->frc_setting.hres_2nd = panel->cur_mode->timing.h_active;
		pcfg->frc_setting.vres_2nd = panel->cur_mode->timing.v_active;
		pcfg->frc_setting.refresh_rate_2nd = panel->cur_mode->timing.refresh_rate;
		pcfg->frc_setting.dsc_2nd = panel->cur_mode->priv_info->dsc_enabled;
		IRIS_LOGI("ap_mipi1_power_st: %d, timing %dx%d@%dHz dsc %d",
			pcfg->ap_mipi1_power_st,
			panel->cur_mode->timing.h_active,
			panel->cur_mode->timing.v_active,
			panel->cur_mode->timing.refresh_rate,
			panel->cur_mode->priv_info->dsc_enabled);
	}
}

void iris_dsi_rx_mode_switch_i7(u8 rx_mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;
	u32 ovs_dly_rfb;
	u32 *payload = NULL;

	IRIS_LOGI("%s: %d", __func__, rx_mode);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xf0, 3);
	if (rx_mode == DSI_OP_CMD_MODE)
		payload[0] |= 0x20001;
	else
		payload[0] &= ~0x20001;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xf0, 0xf0, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x70, 3);
	if (rx_mode == DSI_OP_CMD_MODE)
		payload[0] &= ~0x4000;
	else
		payload[0] |= 0x4000;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x70, 0x70, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x90, 2);
	if (rx_mode == DSI_OP_CMD_MODE)
		payload[0] &= ~0x800;
	else
		payload[0] |= 0x800;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x90, 0x90, 1);
	iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe6, 0xe6, 1);

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

void iris_memc_func_init_i7(struct iris_memc_func *memc_func)
{
	memc_func->register_osd_irq = iris_register_osd_irq_i7;
	memc_func->update_panel_ap_te = iris_update_panel_ap_te_i7;
	memc_func->inc_osd_irq_cnt = iris_inc_osd_irq_cnt_i7;
	memc_func->is_display1_autorefresh_enabled = iris_is_display1_autorefresh_enabled_i7;
	memc_func->pt_sr_set = iris_pt_sr_set_i7;
	memc_func->configure_memc = iris_configure_memc_i7;
	memc_func->configure_ex_memc = iris_configure_ex_memc_i7;
	memc_func->configure_get_memc = iris_configure_get_memc_i7;
	memc_func->init_memc = iris_init_memc_i7;
	memc_func->lightoff_memc = iris_lightoff_memc_i7;
	memc_func->enable_memc = iris_enable_memc_i7;
	memc_func->sr_update = iris_sr_update_i7;
	memc_func->frc_setting_init = iris_frc_setting_init_i7;
	memc_func->dbgfs_memc_init = iris_dbgfs_memc_init_i7;
	memc_func->parse_memc_param0 = iris_parse_memc_param0_i7;
	memc_func->parse_memc_param1 = iris_parse_memc_param1_i7;
	memc_func->frc_timing_setting_update = iris_frc_timing_setting_update_i7;
	memc_func->pt_sr_reset = iris_pt_sr_reset_i7;
	memc_func->mcu_state_set = iris_mcu_state_set_i7;
	memc_func->mcu_ctrl_set = iris_mcu_ctrl_set_i7;
	memc_func->memc_vfr_video_update_monitor = iris_memc_vfr_video_update_monitor_i7;
	memc_func->low_latency_mode_get = iris_low_latency_mode_get_i7;
	memc_func->health_care = iris_health_care_i7;
	memc_func->dsi_rx_mode_switch = iris_dsi_rx_mode_switch_i7;
}

void iris_pwil_idle_mask_update(bool enable)
{
	u32 cmd[4];
	u32 *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe3, 2);
	if (!payload)
		return;
	cmd[0] = IRIS_PWIL_ADDR + PWIL_IDLE_MASK;
	if (enable)
		cmd[1] = 0x000001E7;
	else
		cmd[1] = payload[0];

	cmd[2] = IRIS_PWIL_ADDR + PWIL_REG_UPDATE;
	cmd[3] = 0x00000100;

	iris_ocp_write_mult_vals(4, cmd);
}
