// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2021, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <linux/gcd.h>

#include <video/mipi_display.h>
#include <drm/drm_mipi_dsi.h>
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_log.h"
#include "dsi_iris_reg_i7.h"
#include "dsi_iris_memc_i7.h"
#include "dsi_iris_emv_i7.h"
#include "dsi_iris_dual_i7.h"
#include "dsi_iris_i3c.h"

#define ENALBE_STATUS_CHECK 0
//#define DEBUG_FPGA_LIMIT 1
#define IRIS_PATH_BW10	10

extern u32 iris_pq_disable;
static int iris_extmv_frc_mode;
static int iris_emv_meta_en;
static int iris_emv_skip_duplicate;
static int iris_emv_current_mode;
static int iris_emv_bmv_size;
static int iris_emv_fixed_bmvsize;
static int iris_emv_base4_8x8;
static int iris_emv_low_latency = 1;
static int iris_emv_path_bw;
static int iris_emv_layout;
static int iris_emv_game_landscape;
static struct iris_frc_setting *frc_setting;
static bool iris_emv_touched;
static int iris_emv_shaped;
static bool iris_emv_instant_send = true;
static bool iris_emv_healthcheck_done;
static int iris_emv_healthfailed_sim;
static int iris_emv_reset = 1;
static int iris_emv_dump_once;
static int iris_emv_esd_sim;
static int iris_emv_efifo_control = 1;
static int iris_emv_osd_resize_control = 1;
static bool iris_emv_esd_reviving;
static int iris_emv_esd_ignored;
static int32_t debug_emv_game_frc;
static int32_t debug_emv_new_dual;
static int iris_emv_formal = 1;
static int iris_emv_debug;
static int iris_emv_pq = 1;
static int iris_emv_hugemot = 1;
static int iris_emv_twobuffers = 1;
static int iris_emv_hold_dispmeta;
static int iris_emv_scaler_inpt;
static int iris_emv_scaler_infrc = 1;
static int iris_emv_capen_aftswap;
static int iris_emv_capen_inpt;
static int iris_emv_shock;
static int iris_emv_rescue_enable = 1;
static int iris_emv_sr = 1;
static int iris_emv_lli = 1;
static int iris_emv_batch = 1;
static u32 iris_emv_mv_addr_start;
static int iris_emv_limit = 2073600;//1080x1920
static bool last_reginfo_saved;
static bool last_graphicinfo_saved;
static u32 graphic_ctrl_1_saved;
static bool iris_emv_efifo_config;

int iris_emv_get_bmv_size_i7(void)
{
	if ((iris_emv_bmv_size != 4)
		&& (iris_emv_bmv_size != 8)
		&& (iris_emv_bmv_size != 16))
		iris_emv_bmv_size = 4;

	if (iris_emv_fixed_bmvsize > 0)
		iris_emv_bmv_size = iris_emv_fixed_bmvsize;

	return iris_emv_bmv_size;
}

int iris_emv_get_path_bitwidth_i7(void)
{
	if ((iris_emv_path_bw != 8)
		&& (iris_emv_path_bw != 10))
		iris_emv_path_bw = IRIS_PATH_BW10;

	return iris_emv_path_bw;
}

bool iris_emv_game_mode_enabled_i7(void)
{
	return (debug_emv_game_frc == 1);
}

bool iris_emv_new_dual_enabled_i7(void)
{
	return (debug_emv_new_dual == 1);
}

void iris_emv_frc_mode_en_set_i7(bool enable)
{
	debug_emv_game_frc = enable ? 1 : 0;
	if (enable)
		iris_emv_touched = true;
}

u32 iris_efifo_state_get_i7(void)
{
	u32 rc = 0;
	rc = iris_ocp_read(IRIS_PWIL_0_ADDR + DATA_PATH_CTRL0, DSI_CMD_SET_STATE_HS);
	return (rc&0x2);
}

u32 iris_fi_repeat_state_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_FRC_MIF_ADDR + FRCC_FI_HW_META0, DSI_CMD_SET_STATE_HS);
	pr_err("iris test FRCC_FI_HW_META0= %x\n", rc);
	rc = (rc >> 21) & 0x3;
	return (rc == 0x3 || rc == 0x2) ? 1 : 0;
}

void iris_pwil0_intr_raw_clear_i7(bool direct)
{
	if (direct)
		iris_ocp_write_val(IRIS_PWIL_0_ADDR + 0x1fff0, 0xffff);
	else
		iris_frc_reg_add_i7(IRIS_PWIL_0_ADDR + 0x1fff0, 0xffff, 0);
}

u32 iris_blending_flush_state_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_BLENDING + BLENDING_CTRL, DSI_CMD_SET_STATE_HS);
	pr_err("iris test BLENDING_CTRL = %x\n", rc);
	rc = (rc >> 2) & 0x7;
	return rc;
}

u32 iris_pwil_1_swrst_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_PWIL_1_ADDR + PWIL_CTRL, DSI_CMD_SET_STATE_HS);
	rc &= 0x1;
	return rc;
}

u32 iris_pwr_ctrl_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(0xf0000068, DSI_CMD_SET_STATE_HS);
	return rc;
}

u32 iris_pwil_1_capten_get_i7(void)
{
	u32 rc = 0;
	rc = iris_ocp_read(IRIS_PWIL_1_ADDR + PWIL_1_CAPT_CTRL, DSI_CMD_SET_STATE_HS);
	return rc;
}

u32 iris_pwil_0_capten_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_PWIL_ADDR + PWIL_CAPT_CTRL0, DSI_CMD_SET_STATE_HS);
	return rc;
}

void iris_pwil_0_capten_get_debug_i7(void)
{
	pr_err("iris test PWIL_0_CAPT_CTRL = %x\n", iris_pwil_0_capten_get_i7());
}

u32 iris_pwil_0_ctrl0_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_PWIL_0_ADDR + PWIL_CTRL0, DSI_CMD_SET_STATE_HS);
	return rc;
}

u32 iris_pwil_0_input_meta_ctrl_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_PWIL_0_ADDR + PWIL_INPUT_META_CTRL, DSI_CMD_SET_STATE_HS);
	return rc;
}

u32 iris_pwil_0_disp_ctrl0_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_PWIL_0_ADDR + PWIL_DISP_CTRL0, DSI_CMD_SET_STATE_HS);
	return rc;
}

void iris_dbg_info_get_i7(void)
{
	u32 rc[10] = {0};

	rc[0] = iris_ocp_read(IRIS_RX1_ADDR + RX_VDO_META, DSI_CMD_SET_STATE_HS);
	rc[1] = iris_ocp_read(0xf1a40354, DSI_CMD_SET_STATE_HS);
	rc[2] = iris_ocp_read(0xf1a00354, DSI_CMD_SET_STATE_HS);
	pr_err("iris test IRIS_RX1_VDO_META = 0x%x, FC1 0x%x, FC0 0x%x\n", rc[0], rc[1], rc[2]);
}

void iris_pwil_0_regdump_get_debug_i7(void)
{
	iris_blending_intstat_get_i7();
	iris_rx0_meta_get_i7();
	iris_dbg_info_get_i7();
	pr_err("iris test PWIL_0_CTRL0 = 0x%x\n", iris_pwil_0_ctrl0_get_i7());
	pr_err("iris test PWIL_0_INPUT_META = 0x%x\n", iris_pwil_0_input_meta_ctrl_get_i7());
	pr_err("iris test PWIL_0_disp_ctrl0 = 0x%x\n", iris_pwil_0_disp_ctrl0_get_i7());
	iris_pwil_0_capten_get_debug_i7();
}

void iris_rx0_meta_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_RX0_ADDR + RX0_VDO_META, DSI_CMD_SET_STATE_HS);
	pr_err("iris test IRIS_RX0_VDO_META = %x\n", rc);
}

u32 iris_rx0_result_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_RX0_ADDR + 0x204, DSI_CMD_SET_STATE_HS);
	rc &= 0xFF;
	if (rc > 0)
		pr_err("iris test IRIS_RX0_CMD_04 = %x\n", rc);
	if (rc != 0x02)
		rc = 0;
	return rc;
}

void iris_emv_pwil_lli_trigger_i7(void)
{
	struct iris_ctrl_opt arr[] = {
		{0x03, 0xe0, 0x01},
		{0x03, 0xe1, 0x01},
		{0x03, 0x80, 0x01},	// IRIS_IP_PWIL, pwil: update
		{0x11, 0xe6, 0x00},	// IRIS_IP_DMA channel 12, triggered
	};

	struct iris_ctrl_opt *opt_arr = arr;
	int len = sizeof(arr)/sizeof(struct iris_ctrl_opt);

	iris_send_assembled_pkt(opt_arr, len);
	IRIS_LOGI("%s, len: %d", __func__, len);
}

void iris_emv_pwil_exit_lli_trigger_i7(void)
{
	struct iris_ctrl_opt arr[] = {
		{0x03, 0xA0, 0x01},
		{0x03, 0xB0, 0x01},
		{0x03, 0x70, 0x01},
		{0x03, 0x60, 0x01},
		{0x11, 0xe6, 0x00},	// IRIS_IP_DMA channel 12, triggered
	};

	struct iris_ctrl_opt *opt_arr = arr;
	int len = sizeof(arr)/sizeof(struct iris_ctrl_opt);

	iris_send_assembled_pkt(opt_arr, len);
	IRIS_LOGI("%s, len: %d", __func__, len);
}

void iris_emv_pwil_update_lli_trigger_i7(void)
{
	struct iris_ctrl_opt arr[] = {
		{0x03, 0x80, 0x01},	// IRIS_IP_PWIL, pwil: update
		{0x11, 0xe6, 0x00},	// IRIS_IP_DMA channel 12, triggered
	};

	struct iris_ctrl_opt *opt_arr = arr;
	int len = sizeof(arr)/sizeof(struct iris_ctrl_opt);

	iris_send_assembled_pkt(opt_arr, len);
	IRIS_LOGI("%s, len: %d", __func__, len);
}

void iris_emv_pwil_vfr_disable_i7(bool lli, bool batch)
{
	u32 cmd[2];
	u32 ctrl[1];
	u32 *payload = NULL;

	//frc_ctrl frc_init
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x70, 2);
	if (!payload)
		return;
	ctrl[0] = payload[0];
	ctrl[0] &= ~(0x1);

	if (lli) {
		/* update cmdlist */
		payload[0] = ctrl[0];
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x70, 0x70, 0);
		iris_update_pq_opt(PATH_DSI, true);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_FRC_CTRL;
		cmd[1] = 0;
		iris_ocp_write_mult_vals(2, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_FRC_CTRL, 0x0, 0);
}

void iris_emv_pwil_frc_ctrl_i7(bool vfr, bool gmd, bool mvc, bool fmd, int numVdBuff, int numMvBuff, bool lli, bool batch)
{
	u32 cmd[4];
	u32 ctrl[2];
	u32 *payload = NULL;

	//frc_ctrl frc_init
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x70, 2);
	if (!payload)
		return;
	ctrl[0] = payload[0];
	ctrl[1] = payload[1];
	if (vfr)
		ctrl[0] |= 0x1;
	else
		ctrl[0] &= ~(0x1);

	if (mvc)
		ctrl[1] |= 0x1;
	else
		ctrl[1] &= ~(0x1);

	if (gmd)
		ctrl[1] |= 0x4;
	else
		ctrl[1] &= ~(0x4);

	if (fmd)
		ctrl[1] |= 0x4000;
	else
		ctrl[1] &= ~(0x4000);

	ctrl[1] &= ~(0x73f00);
	numVdBuff &= 0x7;
	numMvBuff &= 0x7;
	ctrl[1] |= (numVdBuff << 8)|(numVdBuff << 11)|(numMvBuff << 16);

	if (lli) {
		/* update cmdlist */
		payload[0] = ctrl[0];
		payload[1] = ctrl[1];
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x70, 0x70, 0);
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		if (iris_emv_instant_send)
			iris_emv_pwil_update_lli_trigger_i7();
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_FRC_CTRL;
		cmd[1] = 0;
		cmd[2] = IRIS_PWIL_ADDR + PWIL_FRC_INIT;
		cmd[3] = 0x00031b08;
		iris_ocp_write_mult_vals(4, cmd);
		return;
	}
	// disable pwil: mvc and gmd
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_FRC_INIT, 0x00031b08, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_REG_UPDATE, 0x00000100, 0);
}

void iris_emv_gmd_disable_i7(void)
{
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_ENABLE_0, 0x0025d596, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_REG_SHDW, 0x00000002, 0);
	iris_emv_pwil_frc_ctrl_i7(false, false, false, false,
			(iris_emv_twobuffers > 0) ? 2 : 3, 2,
			(iris_emv_lli > 0), (iris_emv_batch > 0));
#ifdef OLD_CODE
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_FRC_INIT, 0x00031b08, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_REG_UPDATE, 0x00000100, 0);
#endif
	iris_frc_reg_add_i7(IRIS_GMD_ADDR + GMD_CTRL, 0x00000000, 0);
}

void iris_emv_rx1_switch_input_timing_i7(int width, int height, bool dsc)
{
	u32 value = dsc ? (1 << 24) : 0;

	iris_frc_reg_add_i7(IRIS_RX1_ADDR + DSI_DT_04, value, 0);
	value = ((width - 1) << 16);
	iris_frc_reg_add_i7(IRIS_RX1_ADDR + DCS_CMD_2A_COL, value, 0);
	value = ((height - 1) << 16);
	iris_frc_reg_add_i7(IRIS_RX1_ADDR + DCS_CMD_2B_PAGE, value, 0);
}

void iris_emv_rx0_set_disp_mode_meta_i7(int mode, int pipe, bool compressed, bool dual)
{
	//mode=0: PT, 1: FRC;
	//pipe: 1: graphic, 2: video
	u32 value = dual ? 0x00000080 : 0;

	value |= (mode << 2);
	value |= compressed ? (1<<4) : 0;
	value |= (pipe<<8);
	iris_frc_reg_add_i7(IRIS_RX0_ADDR + RX0_VDO_META, value, 0);
}

void iris_emv_rx1_set_disp_mode_meta_i7(int mode, int pipe, bool compressed, bool dual)
{
	//mode=0: PT, 1: FRC;
	//pipe: 1: graphic, 2: video
	u32 value = dual ? 0x00000080 : 0;

	value |= (mode << 2);
	value |= compressed ? (1<<4) : 0;
	value |= (pipe<<8);
	iris_frc_reg_add_i7(IRIS_RX1_ADDR + RX_VDO_META, value, 0);
}

//comp_en=0 and set 2A 2B: FIX ME the size
void iris_emv_rx1_switch_timing_i7(int width, int height, bool dsc)
{
	iris_emv_rx1_set_disp_mode_meta_i7(0, 1, dsc, true);
	iris_emv_rx1_switch_input_timing_i7(width, height, dsc);
#ifdef OLD_REF
	iris_frc_reg_add_i7(IRIS_RX1_ADDR + RX_VDO_META, 0x00000180, 0);
	iris_frc_reg_add_i7(IRIS_RX1_ADDR + DSI_DT_04, 0x00000000, 0);
	iris_frc_reg_add_i7(IRIS_RX1_ADDR + DCS_CMD_2A_COL, 0x013f0000, 0);
	iris_frc_reg_add_i7(IRIS_RX1_ADDR + DCS_CMD_2B_PAGE, 0x01df0000, 0);
#endif
}

void iris_emv_sys_channels_swap_ctrl_i7(bool rx_ch_swap, bool batch)
{
	u32 cmd[4];
	u32 addr[2];
	u32 value[2];
	u32 *payload = NULL;

	//ec set channel swap shadow mode
	payload = iris_get_ipopt_payload_data(IRIS_IP_SYS, ID_SYS_TE_SWAP_I7, 2);
	if (!payload)
		return;
	addr[0] = payload[0];
	value[0] = payload[1];
	//value[0] |= 0x000e2000; // check mipi_rx, pwil_0, pwil_1 idle
	value[0] |= 0x000a2000; // check pwil_0, pwil_1 idle

	cmd[0] = IRIS_SYS_ADDR + ALT_CTRL2;
	cmd[1] = value[0];

	/* update cmdlist */
	payload[1] = value[0];
	iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_TE_SWAP_I7, ID_SYS_TE_SWAP_I7, 0);

	//e4
	payload = iris_get_ipopt_payload_data(IRIS_IP_SYS, ID_SYS_TE_BYPASS_I7, 2);
	if (!payload)
		return;
	addr[1] = payload[0];
	value[1] = payload[1];
	value[1] &= ~(0x00080000);
	value[1] |= (rx_ch_swap) ? 0x00080000 : 0x0;

	cmd[2] = IRIS_SYS_ADDR + ALT_CTRL0;
	cmd[3] = value[1];

	/* update cmdlist */
	payload[1] = value[1];
	iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_TE_BYPASS_I7, ID_SYS_TE_BYPASS_I7, 0);

	iris_update_pq_opt(PATH_DSI, true);

	if (batch) {
		iris_frc_reg_add_i7(IRIS_SYS_ADDR + ALT_CTRL2, cmd[1], 0);
		iris_frc_reg_add_i7(IRIS_SYS_ADDR + ALT_CTRL0, cmd[3], 0);
	} else {
		iris_ocp_write_mult_vals(4, cmd);
	}
}

//sys swap channels
void iris_emv_sys_channels_swap_i7(void)
{
	//iris_frc_reg_add_i7(IRIS_SYS_ADDR + ALT_CTRL0, 0x00080000, 0);
	//iris_frc_reg_add_i7(IRIS_SYS_ADDR + ALT_CTRL2, 0x000e2000, 0);
	iris_emv_sys_channels_swap_ctrl_i7(true, true);
}

void iris_emv_sys_channels_restore_i7(void)
{
	//iris_frc_reg_add_i7(IRIS_SYS_ADDR + ALT_CTRL0, 0x00000000, 0);
	//iris_frc_reg_add_i7(IRIS_SYS_ADDR + ALT_CTRL2, 0x000e2000, 0);
	iris_emv_sys_channels_swap_ctrl_i7(false, true);
}

void iris_emv_efifo_pre_config_i7(void)
{
	if (!iris_emv_game_mode_enabled_i7())
		return;

	IRIS_LOGI("%s(%d)", __func__, __LINE__);

	// psr rd3 buffer depth
	iris_init_update_ipopt_t(IRIS_IP_PSR_MIF, 0xF1, 0xF1, 1);
	iris_init_update_ipopt_t(IRIS_IP_PSR_MIF, 0x80, 0x80, 1);

	// pwil v1 osd buffer depth
	iris_init_update_ipopt_t(IRIS_IP_PWIL_2, 0xF2, 0xF2, 0);

	iris_emv_efifo_config = true;
}

void iris_emv_efifo_restore_config_i7(void)
{
	if (!iris_emv_efifo_config)
		return;

	IRIS_LOGI("%s(%d)", __func__, __LINE__);

	if ((iris_emv_osd_resize_control&0x01) == 0x1) {
		iris_emv_mode_switch_i7(IRIS_EMV_CLOSE_PIPE_0);
		iris_emv_mode_switch_i7(IRIS_EMV_HELPER_A);
		return;
	}

	// psr rd3 buffer depth
	iris_init_update_ipopt_t(IRIS_IP_PSR_MIF, 0xF0, 0xF0, 1);
	iris_init_update_ipopt_t(IRIS_IP_PSR_MIF, 0x80, 0x80, 1);

	// pwil v1 osd buffer depth
	iris_init_update_ipopt_t(IRIS_IP_PWIL_2, 0xF0, 0xF0, 0);
}

void iris_emv_efifo_enable_i7(bool enable)
{
	if (!iris_emv_efifo_config)
		return;

	IRIS_LOGI("%s(%d), enable: %s",
			__func__, __LINE__, enable ? "true" : "false");

	// trigger for psr
	iris_dma_trig(DMA_CH0, 1);

	if (!enable)
		iris_emv_efifo_config = false;
}

void iris_emv_efifo_enable_ctrl_i7(bool enable)
{
	u32 reg_val;
	u32 *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xf0, 4);
	reg_val = payload[0];

	if (enable)
		/* set DATA_PATH_CTRL0->EFIFO_EN,EFIFO_POSITION to 0x3 */
		payload[0] = BITS_SET(reg_val, 2, 1, 0x3);
	else
		/* set DATA_PATH_CTRL0->EFIFO_EN,EFIFO_POSITION to 0 */
		payload[0] = BITS_SET(reg_val, 2, 1, 0);

	IRIS_LOGI("%s(%d): enable %d, into 0x%08x",
		__func__, __LINE__, enable, payload[0]);

	iris_set_ipopt_payload_data(IRIS_IP_PWIL, 0xf0, 4, payload[0]);

	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xf0, 0xf0, 1);
	iris_set_ipopt_payload_data(IRIS_IP_PWIL, 0xfd, 2, payload[0]);
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xfd, 0xfd, 0);

	//iris_dma_trig(DMA_CH12, 0);
	iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
}

u32 iris_emv_pwil_efifo_buffer_info_get_i7(u32* addr, u32* depth, bool* en)
{
	u32 *payload = NULL;

	if (NULL == addr || NULL == depth || NULL == en)
		return -1;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xF0, 2);
	if (payload) {
		*addr = payload[11];
		*depth = payload[12];
		*en = (payload[2]&0x2);
	}
	if (!(*en)) {
		u32 rc = iris_ocp_read(IRIS_PWIL_0_ADDR + DATA_PATH_CTRL0, DSI_CMD_SET_STATE_HS);
		*en = (rc&0x2);
	}

	return 0;
}

u32 iris_emv_pwil_osd_buffer_info_get_i7(u32* addr, u32* depth)
{
	u32 *payload = NULL;

	if (NULL == addr || NULL == depth)
		return -1;

	//pwil_v11: init emv
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL_2, 0xF2, 2);
	if (payload) {
		*addr = payload[9];
		*depth = payload[10];
	}

	return 0;
}

#define PSR_RD3_BUF_DEPTH  0x1c
#define PSR_OSD_BASE_ADDR 0x120
#define PSR_RD3_M_BUF_DEPTH 0x128
void iris_emv_resize_osd_buffer_ctrl_i7(bool extend, bool lli, bool batch)
{
	u32 *payload = NULL;
	u32 *payload2 = NULL;
	u32 *payload3 = NULL;
	u32 *payload4 = NULL;
	u32 efifo_addr = 0;
	u32 efifo_depth = 0;
	bool efifo_en = false;
	u32 osd_addr = 0;
	u32 osd_depth = 0;
	u32 cmd[14];
	u32 ctrl[7];
	struct iris_cfg *pcfg = iris_get_cfg();
	bool dual_update = (pcfg->iris_pwil_blend_st && pcfg->dual_enabled);

	if (extend) {
		iris_emv_pwil_efifo_buffer_info_get_i7(&efifo_addr, &efifo_depth, &efifo_en);
		iris_emv_pwil_osd_buffer_info_get_i7(&osd_addr, &osd_depth);
		IRIS_LOGV("%s(%d): efifo: 0x%08x(%08x)[%d];osd:0x%08x(%08x),capt_en[%d, %d],swrst[%d],efifo_en[%d]blendflush[%d]",
			__func__, __LINE__,
			efifo_addr, efifo_depth, efifo_en, osd_addr, osd_depth,
			iris_pwil_0_capten_get_i7(), iris_pwil_1_capten_get_i7(),
			iris_pwil_1_swrst_get_i7(), iris_efifo_state_get_i7(), iris_blending_flush_state_get_i7());
		//expand osd size
		osd_addr = efifo_addr;
		osd_depth = osd_depth + efifo_depth;
		ctrl[0] = osd_addr;
		ctrl[1] = osd_depth;
		//pwil_v11: init
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL_2, 0xF0, 2);
		if (payload) {
			if (lli) {
				payload[9] = ctrl[0];
				payload[10] = ctrl[1];
			}
		}
		//psr_mif: init
		payload2 = iris_get_ipopt_payload_data(IRIS_IP_PSR_MIF, 0xF0, 2);
		if (payload2) {
			u32 rd_depth = (payload2[7]&0x0FFF0000) >> 16;
			rd_depth = (osd_depth/16/rd_depth);
			ctrl[2] = (payload2[7]&0xFFFF0000)|((rd_depth)&0x0000FFFF);
			ctrl[3] = osd_addr;
			ctrl[4] = (payload2[10]&0xFF000000)|(osd_depth&0x00FFFFFF);
			if (lli) {
				payload2[7] = ctrl[2];
				payload2[8] = ctrl[3];
				payload2[10] = ctrl[4];
			}
		}
	} else {
		IRIS_LOGV("%s(%d): capt_en[%d, %d],swrst[%d],efifo_en[%d]pwr[0x%x]blendflush[%d]",
			__func__, __LINE__,
			iris_pwil_0_capten_get_i7(), iris_pwil_1_capten_get_i7(),
			iris_pwil_1_swrst_get_i7(), iris_efifo_state_get_i7(),
			iris_pwr_ctrl_get_i7(), iris_blending_flush_state_get_i7());
		//restore
		//pwil_v11: init<=pwil_v11: init emv
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL_2, 0xF0, 2);
		payload2 = iris_get_ipopt_payload_data(IRIS_IP_PWIL_2, 0xF2, 2);
		if (payload && payload2) {
			ctrl[0] = payload2[9];
			ctrl[1] = payload2[10];
			if (lli) {
				payload[9] = ctrl[0];
				payload[10] = ctrl[1];
			}
		}
		//psr_mif: init<=psr_mif: init emv
		payload3 = iris_get_ipopt_payload_data(IRIS_IP_PSR_MIF, 0xF0, 2);
		payload4 = iris_get_ipopt_payload_data(IRIS_IP_PSR_MIF, 0xF1, 2);
		if (payload && payload2) {
			ctrl[2] = payload4[7];
			ctrl[3] = payload4[8];
			ctrl[4] = payload4[10];
			if (lli) {
				payload3[7] = ctrl[2];
				payload3[8] = ctrl[3];
				payload3[10] = ctrl[4];
			}
		}
	}

	IRIS_LOGI("%s(%d):extend %d, pwil2: osd:into 0x%08x(0x%08x);psr-mif:into 0x%08x, 0x%08x, 0x%08x",
		__func__, __LINE__, extend,
		ctrl[0], ctrl[1], ctrl[2], ctrl[3], ctrl[4]);

	IRIS_LOGV("%s(%d):extend %d, [irismipi1: %d, apmipi1: %d, blendst: %d][dual_en: %d, frc_en: %d, pwilmodestate: %d][instant %d]",
			__func__, __LINE__, extend,
			pcfg->iris_mipi1_power_st, pcfg->ap_mipi1_power_st, pcfg->iris_pwil_blend_st,
			pcfg->dual_enabled, pcfg->frc_enabled, pcfg->iris_pwil_mode_state, dual_update);

	if (lli) {
		// psr rd3 buffer depth
		iris_init_update_ipopt_t(IRIS_IP_PSR_MIF, 0xF0, 0xF0, 1);
		iris_init_update_ipopt_t(IRIS_IP_PSR_MIF, 0x80, 0x80, 1);

		// pwil v1 osd buffer depth
		iris_init_update_ipopt_t(IRIS_IP_PWIL_2, 0xF0, 0xF0, 0);

		//should not trigger CH5, otherwise, RX1 will be reset.
		//iris_dma_trig(DMA_CH0|DMA_CH5, 0);

		//send into Iris DMA
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		if (!iris_emv_instant_send)
			return;
	}
	ctrl[5] = 0x8;
	ctrl[6] = 0x1;

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_1_ADDR + PWIL_1_OSD_BASE_ADDR;
		cmd[1] = ctrl[0];
		cmd[2] = IRIS_PWIL_1_ADDR + PWIL_1_OSD_BUF_DEPTH;
		cmd[3] = ctrl[1];
		cmd[4] = IRIS_PSR_ADDR + PSR_RD3_BUF_DEPTH;
		cmd[5] = ctrl[2];
		cmd[6] = IRIS_PSR_ADDR + PSR_OSD_BASE_ADDR;
		cmd[7] = ctrl[3];
		cmd[8] = IRIS_PSR_ADDR + PSR_RD3_M_BUF_DEPTH;
		cmd[9] = ctrl[4];
		cmd[10] = IRIS_PWIL_1_ADDR + PWIL_1_REG_UPDATE;
                cmd[11] = ctrl[5];
                cmd[12] = IRIS_PSR_ADDR + PSR_SW_CONTROL;
                cmd[13] = ctrl[6];

		iris_ocp_write_mult_vals(14, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_1_ADDR + PWIL_1_OSD_BASE_ADDR, ctrl[0], 0);
	iris_frc_reg_add_i7(IRIS_PWIL_1_ADDR + PWIL_1_OSD_BUF_DEPTH, ctrl[1], 0);
	iris_frc_reg_add_i7(IRIS_PWIL_1_ADDR + PWIL_1_REG_UPDATE, ctrl[5], 0);
	iris_frc_reg_add_i7(IRIS_PSR_ADDR + PSR_RD3_BUF_DEPTH, ctrl[2], 0);
	iris_frc_reg_add_i7(IRIS_PSR_ADDR + PSR_OSD_BASE_ADDR, ctrl[3], 0);
	iris_frc_reg_add_i7(IRIS_PSR_ADDR + PSR_RD3_M_BUF_DEPTH, ctrl[4], 0);
	iris_frc_reg_add_i7(IRIS_PSR_ADDR + PSR_SW_CONTROL, ctrl[6], 0);
}

void iris_emv_resize_osd_buffer_i7(bool extend)
{
	iris_emv_resize_osd_buffer_ctrl_i7(extend, (iris_emv_lli > 0), (iris_emv_batch > 0));
}

void iris_emv_pwil_1_capt_enable_ctrl_i7(bool enable, bool lli, bool batch)
{
	u32 cmd[2];
	u32 value = 0x00000000|enable;
	u32 *payload = NULL;

	if (lli) {
		/* update cmdlist */
		//PWIL_CAPT_CTRL
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL_2, 0xF1, 4);
		payload[0] = value;
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xF1, 0xF1, 0);
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		if (!iris_emv_instant_send)
			return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_1_ADDR + PWIL_1_CAPT_CTRL;
		cmd[1] = value;
		iris_ocp_write_mult_vals(2, cmd);
		return;
	}
	iris_frc_reg_add_i7(IRIS_PWIL_1_ADDR + PWIL_1_CAPT_CTRL, 0x00000000|enable, 0);
}

void iris_emv_pwil_1_swrst_enable_ctrl_i7(bool enable, bool lli, bool batch)
{
	u32 cmd[2];
	u32 value = 0x00000a04|enable;

	if (lli) {
		struct iris_update_regval regval;
		u32 sw_rst = (enable ? 0x1 : 0x0);

		regval.ip = IRIS_IP_PWIL_2;
		regval.opt_id = 0xf1;
		regval.mask = 0x1;
		regval.value = sw_rst;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);
		if (iris_emv_instant_send) {
			iris_dma_trig(DMA_CH5, 0);
			iris_update_pq_opt(PATH_DSI, true);
		}
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_1_ADDR + PWIL_1_CAPT_CTRL;
		cmd[1] = value;
		iris_ocp_write_mult_vals(2, cmd);
		return;
	}
	iris_frc_reg_add_i7(IRIS_PWIL_1_ADDR + PWIL_CTRL, 0x00000a04|enable, 0);
}

void iris_emv_ext_dbg_i7(int mode)
{
	if ((0x10000&mode) == 0x10000) {
		return;
	}
	if ((0x20000&mode) == 0x20000) {
		int ctlcode = (0xFFFF&mode);

		IRIS_LOGI("%s: debug mode 0x%x", __func__, mode);

		iris_memc_cmd_payload_init_i7();

		switch (ctlcode) {
		case 0x01:
			iris_emv_efifo_enable_ctrl_i7(true);
			break;
		case 0x02:
			iris_emv_efifo_enable_ctrl_i7(false);
			break;
		case 0x03:
			iris_emv_resize_osd_buffer_i7(true);
			break;
		case 0x04:
			iris_emv_resize_osd_buffer_i7(false);
			break;
		default:
			break;
                }
		iris_memc_cmd_payload_send_i7();
		return;
	}
}

u32 iris_emv_pwil_video_ctrl1_get_i7(void)
{
	u32 *payload = NULL;

	//video_ctrl_1
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xB1, 2);
	if (payload)
		return payload[1];

	return 0;
}

u32 iris_emv_pwil_video_ctrl5_get_i7(void)
{
	u32 *payload = NULL;

	//video_ctrl
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xB1, 2);
	if (payload)
		return payload[5];

	return 0;
}

static u32 iris_emv_pwil_video_ctrl7_get_i7(bool emv)
{
	u32 *payload = NULL;
	u8 opt = emv ? 0xB2 : 0xB1;

	//video_ctrl
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, opt, 2);
	if (payload)
		return payload[7];//PWIL_VIDEO_CTRL7

	return 0;
}

//
//eMV game: buffer allocation
//EFIFO:
//OSD FIFO 20000 ~ 7FFFF
//EFIFO: 80000~A0000
//Video: A0000~271000
//MV: 271000~
//
void iris_emv_pwil_vidbuffer_alloc_i7(bool emv, bool lli, bool batch)
{
	u32 addr = iris_emv_pwil_video_ctrl7_get_i7(emv);
	u32 cmd[2];
	u32 ctrl[1];
	u32 *payload = NULL;

	//addr = (addr == 0) ? 0x00080000 : addr;

	if (lli) {
		//video_ctrl
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xB0, 2);
		if (!payload)
			return;
		ctrl[0] = addr;
		/* update cmdlist */
		payload[7] = ctrl[0];
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xB0, 0xB0, 0);
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL7;
		cmd[1] = addr;
		iris_ocp_write_mult_vals(2, cmd);
		return;
	}

	//PWIL_0
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL7, addr, 0);
}

int iris_emv_aligned_width_get_i7(int width, int bpp, int alignbits)
{
	int hstride = (width * bpp + alignbits - 1) / alignbits;

	return hstride*alignbits/bpp;
}

int iris_emv_frc_mvdbuffer_offset_get_i7(void)
{
	int offset = 0;

	if (frc_setting) {
		if (frc_setting->emv_hres > 0 && frc_setting->emv_vres > 0) {
			offset = iris_emv_aligned_width_get_i7(frc_setting->emv_hres, 24, 64);
			offset = offset*frc_setting->emv_vres*3;
			offset = ((offset >> 10) + 1) << 10;//1k-aligned
		}
	}
	return offset;
}

u32 iris_emv_pwil_mv1buffer_base_addr_get_i7(void)
{
	u32 addr = 0;
	u32 *payload = NULL;

	//get mv1 base addr2
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe1, 9);
	if (!payload)
		return 0;
	addr = payload[0];
	IRIS_LOGI("%s: MV1_BASE_ADDR2 = 0x%x", __func__, addr);
	if (addr == 0)
		addr = 0x00271000;
	if (iris_emv_mv_addr_start != 0)
		addr = iris_emv_mv_addr_start;

	return addr;
}

void iris_emv_pwil_mvdbuffer_alloc_i7(bool lli, bool batch)
{
	u32 cmd[8];
	u32 addr[4];
	u32 *payload = NULL;
	u32 mvstart = iris_emv_pwil_mv1buffer_base_addr_get_i7();
	u32 offset = iris_emv_frc_mvdbuffer_offset_get_i7();

	IRIS_LOGI("%s: MV1ADDR2 = 0x%x[0x%x][0x%x][0x%x], offset = 0x%x(mvd:0x%04x%04x)",
		__func__,
		mvstart,
		mvstart + offset,
		mvstart + 2*offset,
		mvstart + 3*offset,
		offset,
		frc_setting->emv_hres,
		frc_setting->emv_vres);

	addr[0] = mvstart;//mv1_2
	addr[1] = addr[0] + offset;//mv0_2
	addr[2] = addr[1] + offset;//mv1_3
	addr[3] = addr[2] + offset;//mv0_3

	cmd[0] = IRIS_PWIL_0_ADDR + PWIL_MV1_BASE_ADDR2;
	cmd[1] = addr[0];
	cmd[2] = IRIS_PWIL_0_ADDR + PWIL_MV0_BASE_ADDR2;
	cmd[3] = addr[1];
	cmd[4] = IRIS_PWIL_0_ADDR + PWIL_MV1_BASE_ADDR3;
	cmd[5] = addr[2];
	cmd[6] = IRIS_PWIL_0_ADDR + PWIL_MV0_BASE_ADDR3;
	cmd[7] = addr[3];

	/* update cmdlist */
	if (lli) {
		//MV0_BASE_ADDR0
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe1, 2);
		if (!payload)
			return;
		payload[0] = 0;
		payload[1] = 0;
		payload[2] = addr[1];//mv0_2
		payload[3] = addr[3];//mv0_3
		payload[5] = 0;
		payload[6] = 0;
		payload[7] = addr[0];//mv1_2
		payload[8] = addr[2];//mv1_3

		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xe1, 0xe1, 0);
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		return;
	}

	if (!batch && !lli) {
		iris_ocp_write_mult_vals(8, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV1_BASE_ADDR2, 0x00271000, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV0_BASE_ADDR2, 0x0027C000, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV1_BASE_ADDR3, 0x00287000, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV0_BASE_ADDR3, 0x00292000, 0);
}

void iris_emv_pwil_buffers_alloc_i7(bool emv)
{
	//iris_emv_pwil_osdbuffer_alloc();//3
	iris_emv_pwil_vidbuffer_alloc_i7(emv, (iris_emv_lli > 0), (iris_emv_batch > 0));//1
	iris_emv_pwil_mvdbuffer_alloc_i7((iris_emv_lli > 0), (iris_emv_batch > 0));//4
}

void iris_emv_pwil_mv_capt_enable_ctrl_i7(bool enable, bool use_meta, bool lli, bool batch)
{
	u32 cmd[2];
	u32 *payload = NULL;
	u32 value = (use_meta) ? 0x00000001 : 0x03010000;

	value = (iris_emv_meta_en > 0) ? 0x00000001 : value;
	value = enable ? value : 0x00000000;

	/* update cmdlist */
	if (lli) {
		//INPUT_META_CTRL
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe0, 2);
		if (!payload)
			return;
		payload[0] = value;

		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xe0, 0xe0, 0);
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_INPUT_META_CTRL;
		cmd[1] = value;
		iris_ocp_write_mult_vals(2, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_INPUT_META_CTRL, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_REG_UPDATE, 0x00000100, 0);
}

void iris_emv_pwil_mv_capt_enable_i7(bool enable, bool use_meta)
{
	iris_emv_pwil_mv_capt_enable_ctrl_i7(enable, use_meta,
			(iris_emv_lli > 0), (iris_emv_batch > 0));
}

void iris_emv_pwil_duplicate_frame_filter_ctrl_i7(bool enable, bool lli, bool batch)
{
	u32 cmd[2];
	u32 value = enable ? 0x0a : 0x02;
	u32 *payload = NULL;

	//PWIL_PIAD_BLEND_INFO
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x90, 2);
	if (!payload)
		return;
	value = payload[0];
	if (enable)
		value |= 0x8;
	else
		value &= ~(0x8);

	if (!((iris_emv_meta_en > 0) && (iris_emv_skip_duplicate > 0)))
		value &= ~(0x8);

	if (lli) {
		/* update cmdlist */
		payload[0] = value;
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x90, 0x90, 0);
		iris_update_pq_opt(PATH_DSI, true);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_PIAD_BLEND_INFO;
		cmd[1] = value;
		iris_ocp_write_mult_vals(2, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_PIAD_BLEND_INFO, value, 0);
}

void iris_emv_pwil_duplicate_frame_filter_i7(bool enable)
{
	iris_emv_pwil_duplicate_frame_filter_ctrl_i7(enable,
			(iris_emv_lli > 0), (iris_emv_batch > 0));
}

void iris_emv_pwil_display_mode_set_i7(int mode,
				int pipe,
				bool compressed,
				bool dual,
				bool repeat,
				bool swapped)
{
	//mode=0: PT, 1: FRC; pipe: 1: graphic, 2: video
	u32 value = dual ? 0x10000000 : 0;

	value |= repeat ? (1<<2) : 0;
	value |= (pipe<<16)|mode;
	value |= compressed ? (1<<3) : 0;

	if (iris_emv_lli == 0) {
		iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_DISP_CTRL0, value, 0);
		iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_REG_UPDATE, 0x00000100, 0);
		return;
	}

	if (swapped)
		iris_emv_rx1_set_disp_mode_meta_i7(mode, pipe, compressed, dual);
	else
		iris_emv_rx0_set_disp_mode_meta_i7(mode, pipe, compressed, dual);
}

void iris_emv_pwil_capt_enable_ctrl_i7(bool enable, bool lli, bool batch)
{
	u32 cmd[2];
	u32 value = 0x00000000|enable;
	u32 *payload = NULL;

	if (lli) {
		/* update cmdlist */
		//PWIL_CAPT_CTRL0
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xF0, 7);
		if (!payload)
			return;
		payload[0] = value;
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xF0, 0xF0, 0);
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		if (!iris_emv_instant_send)
			return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_CAPT_CTRL0;
		cmd[1] = value;
		iris_ocp_write_mult_vals(2, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_CAPT_CTRL0, 0x00000000|enable, 0);
}

void iris_emv_pwil_capt_enable_i7(int enable)
{
	iris_emv_pwil_capt_enable_ctrl_i7(enable, (iris_emv_lli > 0), (iris_emv_batch > 0));
	IRIS_LOGI("%s: main cap_en = %d", __func__, enable);
}

void iris_emv_pwil_1_capt_enable_i7(int enable)
{
	IRIS_LOGI("%s: osd cap_en = %d", __func__, enable);
	iris_emv_pwil_1_capt_enable_ctrl_i7(enable, /*(iris_emv_lli > 0)*/false, (iris_emv_batch > 0));
}

void iris_emv_pwil_1_swrst_enable_i7(int enable)
{
	IRIS_LOGI("%s: osd swrst = %d", __func__, enable);
	iris_emv_pwil_1_swrst_enable_ctrl_i7(enable, /*(iris_emv_lli > 0)*/false, (iris_emv_batch > 0));
}

void iris_emv_blending_flush_ctrl_i7(bool video, bool graphic, bool osd, bool batch)
{
	u32 cmd[4];
	u32 value;
	u32 *payload = NULL;
	int flush = 0;
	u32 valueCsrTo = 0;
	bool forPT = (IRIS_EMV_OFF_PT == iris_emv_mode_get_i7() || IRIS_EMV_OFF_FINAL == iris_emv_mode_get_i7());

	flush |= video ? 0x4 : 0;
	flush |= graphic ? 0x8 : 0;
	flush |= osd ? 0x10 : 0;

	if (forPT) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_BLEND, 0x10, 2);
		if (!payload)
			return;
		valueCsrTo = payload[0];
	} else {
		payload = iris_get_ipopt_payload_data(IRIS_IP_BLEND, 0x20, 2);
		if (!payload)
			return;
		valueCsrTo = payload[0];
	}

	payload = iris_get_ipopt_payload_data(IRIS_IP_BLEND, 0xF0, 2);
	if (!payload)
		return;
	value = payload[0];
	value &= ~(0x1C);
	value |= flush;

	cmd[0] = IRIS_BLENDING + BLENDING_CTRL;
	cmd[1] = value;
	cmd[2] = IRIS_BLENDING + CSR_TO;
	cmd[3] = valueCsrTo;

	/* update cmdlist */
	payload[0] = value;
	payload[6] = valueCsrTo;
	iris_init_update_ipopt_t(IRIS_IP_BLEND, 0xF0, 0xF0, 0);
	iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
	if (!iris_emv_instant_send)
		return;

	if (batch) {
		iris_frc_reg_add_i7(cmd[0], cmd[1], 0);
		iris_frc_reg_add_i7(cmd[2], cmd[3], 0);
	} else
		iris_ocp_write_mult_vals(4, cmd);

	IRIS_LOGI("%s:flush video %d, graphic %d, osd %d, 0x%x, csrto 0x%x(0x%x)", __func__,
			 video, graphic, osd, value, valueCsrTo, iris_emv_mode_get_i7());
}

void iris_emv_blending_flush_enable_i7(bool video, bool graphic, bool osd)
{
	iris_emv_blending_flush_ctrl_i7(video, graphic, osd, true);
}

void iris_emv_pwil_dport_control_i7(int count, bool lli, bool batch)
{
	u32 value = 0x04000000|((count&0x1F)<<27);
	u32 cmd[4];
	u32 *payload = NULL;

	//pwil: disp PWIL_DISP_CTRL1
	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xD0, 2);
	if (!payload)
		return;

	if (lli) {
		/* update cmdlist */
		payload[0] = value;
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xD0, 0xD0, 0);
		iris_update_pq_opt(PATH_DSI, true);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_DISP_CTRL1;
		cmd[1] = value;
		cmd[2] = IRIS_PWIL_ADDR + PWIL_REG_UPDATE;
		cmd[3] = 0x00000100;
		iris_ocp_write_mult_vals(4, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_DISP_CTRL1, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_REG_UPDATE, 0x00000100, 0);
}

void iris_emv_pwil_configure_cmd_ctrl_i7(int col, int width,
				int row, int height,
				bool lli, bool batch)
{
	u32 row_range = ((height + row - 1) << 16)|(row&0xFFFF);
	u32 col_range = ((width + col - 1) << 16)|(col&0xFFFF);
	//u32 value = (height<<16)|(width&0xFFFF);
	u32 cmd[4];
	u32 ctrl[2];
	u32 *payload = NULL;

	ctrl[0] = row_range;//PWIL_CMD_CTRL0
	ctrl[1] = col_range;//PWIL_CMD_CTRL1

	if (lli) {
		//cmd_ctrl
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x60, 2);
		if (!payload)
			return;

		/* update cmdlist */
		payload[0] = ctrl[0];
		payload[1] = ctrl[1];

		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x60, 0x60, 0);
		iris_update_pq_opt(PATH_DSI, true);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_CMD_CTRL0;
		cmd[1] = ctrl[0];
		cmd[2] = IRIS_PWIL_ADDR + PWIL_CMD_CTRL1;
		cmd[3] = ctrl[1];

		iris_ocp_write_mult_vals(4, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_CMD_CTRL0, row_range, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_CMD_CTRL1, col_range, 0);

#ifdef OLD_CODE
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL2, value, 0);
#endif
}

int iris_emv_pwil_stride_get_i7(int width, int bpp)
{
	int hstride = (width * bpp + 63) / 64;

	return hstride;
}

void iris_emv_pwil_configure_graphic_ctrl_i7(int width, int height, bool compressed, bool disp_dsc_en,
					bool lli, bool batch, bool emvEnter)
{
	u32 value = (height<<16)|(width&0xFFFF);
	u16 bpp = compressed ? 8 : 24;
	u32 hstride = 0;
	u32 cmd[12];
	u32 ctrl[8];
	u32 *payload = NULL;

	hstride = iris_emv_pwil_stride_get_i7(width, bpp);

	ctrl[2] = value;//PWIL_GRAPHIC_CTRL2
	ctrl[3] = 0;//PWIL_GRAPHIC_CTRL3
	ctrl[4] = value;//PWIL_GRAPHIC_CTRL4
	ctrl[5] = value;//PWIL_GRAPHIC_CTRL5
	ctrl[6] = hstride;//PWIL_GRAPHIC_CTRL6

	if (lli) {
		//graphic_ctrl
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xA0, 2);
		if (!payload)
			return;
		ctrl[0] = payload[0];
		if (!last_graphicinfo_saved && emvEnter) {
			graphic_ctrl_1_saved = payload[1];
			last_graphicinfo_saved = true;
		}

		ctrl[1] = emvEnter ? (disp_dsc_en ? 0x00040888 : 0x00000888) : graphic_ctrl_1_saved;
		if (ctrl[1] == 0)
			ctrl[1] = (payload[1] != 0) ? payload[1] : 0x00040888;

		/* update cmdlist */
		payload[0] = ctrl[0];
		payload[1] = ctrl[1];
		payload[2] = ctrl[2];
		payload[3] = ctrl[3];
		payload[4] = ctrl[4];
		payload[5] = ctrl[5];
		payload[6] = ctrl[6];

		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xA0, 0xA0, 0);
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL1;
		cmd[1] = ctrl[1];
		cmd[2] = IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL2;
		cmd[3] = ctrl[2];
		cmd[4] = IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL3;
		cmd[5] = ctrl[3];
		cmd[6] = IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL4;
		cmd[7] = ctrl[4];
		cmd[8] = IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL5;
		cmd[9] = ctrl[5];
		cmd[10] = IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL6;
		cmd[11] = ctrl[6];

		iris_ocp_write_mult_vals(12, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL2, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL4, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL5, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_GRAPHIC_CTRL6, 0x0000003c, 0);
}

void iris_emv_pwil_switch_to_pt_in_raw_graphic_i7(void)
{
	if (iris_emv_lli > 0) {
		iris_emv_pwil_display_mode_set_i7(0, 1, false, true, false, true);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_DISP_CTRL0, 0x10010000, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_REG_UPDATE, 0x00000100, 0);
}

void iris_emv_pwil_restore_video_ctrl_i7(bool send)
{
	u32 *payload = NULL;
	u32 *payload_orig = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xB0, 2);
	payload_orig = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xB1, 2);
	if (payload && payload_orig) {
		memcpy(payload, payload_orig, 9*sizeof(u32));
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xB0, 0xB0, 0);
		iris_update_pq_opt(PATH_DSI, send);
	}
}

void iris_emv_pwil_configure_video_ctrl_i7(int left, int top,
					int videoWidthCapt, int videoHeightCapt,
					int videoWidth, int videoHeight,
					int dispWidth, int dispHeight,
					bool compressed, bool disp_dsc, bool scalerdown, int numVdBuff,
					bool lli, bool batch, bool emvEnter)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u16 dsc_bpp = compressed ? pcfg->frc_setting.mv_coef : 24;
	u32 procSize = (videoHeight<<16)|(videoWidth&0xFFFF);
	u32 rawSize = (dispHeight<<16)|(dispWidth&0xFFFF);
	u32 captStart = (top<<16)|(left&0xFFFF);
	u32 captSize = (videoHeightCapt<<16)|(videoWidthCapt&0xFFFF);
	u32 hstride = 0;
	u32 cmd[10];
	u32 ctrl[8];
	u32 *payload = NULL;

	hstride = iris_frc_video_hstride_calc_i7(videoWidth, dsc_bpp, 1);

	if (!scalerdown)
		captSize = procSize;

	IRIS_LOGI("%s: dsc_bpp 0x%x, hstride %d", __func__, dsc_bpp, hstride);
	ctrl[1] = iris_emv_pwil_video_ctrl1_get_i7();
	ctrl[2] = rawSize;//PWIL_VIDEO_CTRL2
	ctrl[3] = captStart;//PWIL_VIDEO_CTRL3
	ctrl[4] = captSize;//PWIL_VIDEO_CTRL4
	ctrl[5] = procSize;//PWIL_VIDEO_CTRL5
	ctrl[6] = hstride;//PWIL_VIDEO_CTRL6

	if (lli) {
		//video_ctrl
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xB0, 2);
		if (!payload)
			return;
		ctrl[0] = payload[0];
		ctrl[0] &= (~0xf);
		ctrl[0] |= (numVdBuff&0xf);

		if (emvEnter) {
			ctrl[1] &= ~0x0007ffff;
			ctrl[1] |= 0x00000888;
			ctrl[1] |= compressed ? 0x00050000 : 0;
			ctrl[1] |= disp_dsc ? 0x00020000 : 0;
		}

		/* update cmdlist */
		payload[0] = ctrl[0];//PWIL_VIDEO_CTRL0
		payload[1] = ctrl[1];//PWIL_VIDEO_CTRL1
		payload[2] = ctrl[2];//PWIL_VIDEO_CTRL2
		payload[3] = ctrl[3];//PWIL_VIDEO_CTRL3
		payload[4] = ctrl[4];//PWIL_VIDEO_CTRL4
		payload[5] = ctrl[5];//PWIL_VIDEO_CTRL5
		payload[6] = ctrl[6];//PWIL_VIDEO_CTRL6

		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xB0, 0xB0, 0);
		iris_update_pq_opt(PATH_DSI, true);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_0_ADDR + PWIL_VIDEO_CTRL2;
		cmd[1] = ctrl[2];
		cmd[2] = IRIS_PWIL_0_ADDR + PWIL_VIDEO_CTRL3;
		cmd[3] = ctrl[3];
		cmd[4] = IRIS_PWIL_0_ADDR + PWIL_VIDEO_CTRL4;
		cmd[5] = ctrl[4];
		cmd[6] = IRIS_PWIL_0_ADDR + PWIL_VIDEO_CTRL5;
		cmd[7] = ctrl[5];
		cmd[8] = IRIS_PWIL_0_ADDR + PWIL_VIDEO_CTRL6;
		cmd[9] = ctrl[6];

		iris_ocp_write_mult_vals(10, cmd);
		return;
	}

	//iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL1, 0x00070888, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL2, rawSize, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL3, captStart, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL4, captSize, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL5, procSize, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL6, hstride, 0);

#ifdef OLD_CODE
	u32 value = (height<<16)|(width&0xFFFF);
	//u32 hstride = (compressed) ? width*8/64 : width*24/64;//ceiling

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL1, 0x00070888, 0);
	//iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL2, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL3, 0x005b0000, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL4, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL5, value, 0);
	//iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_VIDEO_CTRL6, 0x00000021, 0);
#endif
}

void iris_emv_pwil_configure_mv_ctrl_i7(int videoWidth, int videoHeight, bool lli, bool batch)
{
	int width = 0;
	int height = 0;
	int left = 0;
	int top = 0;
	u32 value = 0;
	u32 mv0_capt_start = 0;
	u32 mv1_capt_start = 0;
	u32 mv_hstride = 0;
	int bmvSize = iris_emv_get_bmv_size_i7();
	int hstride = 0;
	u32 cmd[18];
	u32 ctrl[9];
	u32 *payload = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s: bmv 0x%04x%04x", __func__, bmvSize, bmvSize);

	width = ((videoWidth + bmvSize - 1)/bmvSize)*2;
	height = (videoHeight + bmvSize - 1)/bmvSize;
	value = (height<<16)|(width&0xFFFF);
	left = 0;
	top = 1;

	if ((iris_emv_base4_8x8 > 0) && (iris_emv_fixed_bmvsize > 0))
		top = 1 + (videoHeight/4 - height);

	if (iris_emv_game_landscape > 0)
		top = 1;
	left = pcfg->emv_info.mvd0Left;
	top = pcfg->emv_info.mvd0Top;
	mv0_capt_start = (top<<16)|(left&0xFFFF);
	left = width;
	if ((iris_emv_base4_8x8 > 0) && (iris_emv_fixed_bmvsize > 0))
		left = 2*width;
	left = pcfg->emv_info.mvd1Left;
	top = pcfg->emv_info.mvd1Top;
	mv1_capt_start = (top<<16)|(left&0xFFFF);
	hstride = iris_emv_pwil_stride_get_i7(width, 24);
	mv_hstride = (hstride&0xFFFF);
	mv_hstride |= (mv_hstride<<16);
	mv_hstride |= (hstride&0xFFFF);

	ctrl[0] = value;//MV0_CTRL0
	ctrl[1] = mv0_capt_start;//MV0_CTRL1
	ctrl[2] = value;//MV0_CTRL2
	ctrl[3] = value;//MV0_CTRL3
	ctrl[4] = value;//MV1_CTRL0
	ctrl[5] = mv1_capt_start;//MV1_CTRL1
	ctrl[6] = value;//MV1_CTRL2
	ctrl[7] = value;//MV1_CTRL3
	ctrl[8] = mv_hstride;//MV_H_STRIDE

	cmd[0] = IRIS_PWIL_0_ADDR + PWIL_MV0_CTRL0;
	cmd[1] = ctrl[0];
	cmd[2] = IRIS_PWIL_0_ADDR + PWIL_MV0_CTRL1;
	cmd[3] = ctrl[1];
	cmd[4] = IRIS_PWIL_0_ADDR + PWIL_MV0_CTRL2;
	cmd[5] = ctrl[2];
	cmd[6] = IRIS_PWIL_0_ADDR + PWIL_MV0_CTRL3;
	cmd[7] = ctrl[3];
	cmd[8] = IRIS_PWIL_0_ADDR + PWIL_MV1_CTRL0;
	cmd[9] = ctrl[4];
	cmd[10] = IRIS_PWIL_0_ADDR + PWIL_MV1_CTRL1;
	cmd[11] = ctrl[5];
	cmd[12] = IRIS_PWIL_0_ADDR + PWIL_MV1_CTRL2;
	cmd[13] = ctrl[6];
	cmd[14] = IRIS_PWIL_0_ADDR + PWIL_MV1_CTRL3;
	cmd[15] = ctrl[7];
	cmd[16] = IRIS_PWIL_0_ADDR + PWIL_MV_H_STRIDE;
	cmd[17] = ctrl[8];

	if (lli) {
		//PWIL_MV0_CTRL0
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe0, 3);
		if (!payload)
			return;
		/* update cmdlist */
		payload[0] = ctrl[0];//MV0_CTRL0
		payload[1] = ctrl[1];//MV0_CTRL1
		payload[2] = ctrl[2];//MV0_CTRL2
		payload[3] = ctrl[3];//MV0_CTRL3
		payload[4] = ctrl[4];//MV1_CTRL0
		payload[5] = ctrl[5];//MV1_CTRL1
		payload[6] = ctrl[6];//MV1_CTRL2
		payload[7] = ctrl[7];//MV1_CTRL3
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xe0, 0xe0, 0);

		//PWIL_MV_H_STRIDE
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe1, 11);
		if (!payload)
			return;
		payload[0] = ctrl[8];//PWIL_MV_H_STRIDE
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xe1, 0xe1, 0);

		iris_update_pq_opt(PATH_DSI, true);
		return;
	}

	if (!batch && !lli) {
		iris_ocp_write_mult_vals(18, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV0_CTRL0, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV0_CTRL1, mv0_capt_start, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV0_CTRL2, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV0_CTRL3, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV1_CTRL0, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV1_CTRL1, mv1_capt_start, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV1_CTRL2, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV1_CTRL3, value, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_MV_H_STRIDE, mv_hstride, 0);
}

void iris_emv_pwil_configure_dma_ctrl_i7(bool emv, bool lli, bool batch)
{
	struct iris_update_regval regval;
	u32 water = (emv ? 0x3c : 0x30);

	regval.ip = IRIS_IP_PWIL;
	regval.opt_id = 0xdf;
	regval.mask = 0x1fff8000;
	regval.value = (water << 15)|(water << 22);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 1);
	if (iris_emv_instant_send) {
		iris_dma_trig(DMA_CH12, 0);
		iris_update_pq_opt(PATH_DSI, true);
	}
}

void iris_emv_pwil_shortpacket_mask_ctrl_i7(int enable, bool updatelli, bool batch)
{
	u32 cmd[2];
	u32 value;
	u32 *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xF0, 2);
	if (!payload)
		return;
	value = payload[0];
	if (enable)
		value |= 0x8;
	else
		value &= ~(0x8);

	cmd[0] = IRIS_PWIL_ADDR + PWIL_CTRL0;
	cmd[1] = value;

	/* update cmdlist */
	if (updatelli) {
		payload[0] = value;
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xF0, 0xF0, 0);
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		if (!iris_emv_instant_send)
			return;
	}

	if (batch)
		iris_frc_reg_add_i7(cmd[0], cmd[1], 0);
	else
		iris_ocp_write_mult_vals(2, cmd);
}

void iris_emv_pwil_shortpacket_mask_i7(int enable)
{
	iris_emv_pwil_shortpacket_mask_ctrl_i7(enable, true, true);
	//iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_CTRL0, 0x00094014|(enable<<3), 0);
	IRIS_LOGI("%s: mask = %d", __func__, enable);
}

void iris_emv_frc_ds_ctrl_i7(int dsy_out_en, int ppcy_out_en, bool update)
{
	iris_frc_reg_add_i7(IRIS_FRC_DS_ADDR + FRC_DS_CTRL,
		0x000a018a|(dsy_out_en<<2)|(ppcy_out_en<<4), 0);
	if (update)
		iris_frc_reg_add_i7(IRIS_FRC_DS_ADDR + 0x1ff00, 0x00000002, 0);
}

void iris_emv_pwil_configure_path_ctrl_0_i7(bool lli, bool batch)
{
	u32 cmd[2];
	u32 value = (iris_emv_get_path_bitwidth_i7() == IRIS_PATH_BW10) ? 0x84c00056 : 0x84800016;
	u32 *payload = NULL;

	if (lli) {
		// pwil: ctrl 0: DATA_PATH_CTRL0
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xF0, 4);
		if (!payload)
			return;

		if ((value&payload[0]) != value) {
			IRIS_LOGE("%s: Insufficient DATA_PATH_CTRL0 for emv = 0x%x",
					__func__,
					payload[0]);
			value |= payload[0];
			payload[0] = value;
		}
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xF0, 0xF0, 0);
		iris_update_pq_opt(PATH_DSI, true);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + DATA_PATH_CTRL0;
		cmd[1] = value;
		iris_ocp_write_mult_vals(2, cmd);
	}

	if (iris_emv_get_path_bitwidth_i7() == IRIS_PATH_BW10)
		iris_frc_reg_add_i7(IRIS_PWIL_ADDR + DATA_PATH_CTRL0, 0x84c00056, 0);
	else
		iris_frc_reg_add_i7(IRIS_PWIL_ADDR + DATA_PATH_CTRL0, 0x84800016, 0);
}

void iris_emv_pwil_configure_sliceinfo_ctrl_i7(bool enterEmv, bool lli, bool batch)
{
	u32 cmd[4];
	u32 ctrl[2];
	u32 *payload = NULL;

	if (!enterEmv) {
		ctrl[0] = 0x00f000f0;
		ctrl[1] = 0x00f000f0;
	} else {
		ctrl[0] = 0x04380001;
		ctrl[1] = 0x02400438;
	}

	/* update cmdlist */
	if (lli) {
		if (!enterEmv) {
			payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL_2, 0xF0, 9);
			if (!payload)
				return;
			ctrl[0] = payload[0];
			ctrl[1] = payload[1];
		}
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xF0, 8);
		if (!payload)
			return;
		payload[0] = ctrl[0];
		payload[1] = ctrl[1];
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xF0, 0xF0, 0);
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + PWIL_SLICE_INFO0;
		cmd[1] = ctrl[0];
		cmd[2] = IRIS_PWIL_ADDR + PWIL_SLICE_INFO1;
		cmd[3] = ctrl[1];
		iris_ocp_write_mult_vals(4, cmd);
		return;
	}

	if (!enterEmv) {
		iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_SLICE_INFO0, 0x00f000f0, 0);
		iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_SLICE_INFO1, 0x00f000f0, 0);
	} else {
		iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_SLICE_INFO0, 0x04380001, 0);
		iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_SLICE_INFO1, 0x02400438, 0);
	}
}

void iris_emv_pwil_path_ctrl_1_configure_i7(bool scalerup, bool scalerdown, bool sr, bool lli, bool batch)
{
	u32 cmd[2];
	u32 *payload = NULL;
	//DATA_PATH_CTRL1 enabled scale up, scale down and SR
	u32 value = 0x00000204;

	if (iris_emv_get_path_bitwidth_i7() == IRIS_PATH_BW10)
		value = 0x00000206;

	if (lli) {
		//DATA_PATH_CTRL1
		payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xF0, 5);
		if (!payload)
			return;
		value = payload[0];
	}

	if (scalerup)
		value &= ~(0x20);
	else
		value |= 0x20;
	if (scalerdown)
		value &= ~(0x80);
	else
		value |= 0x80;
	if (sr)
		value &= ~(0x200);
	else
		value |= 0x200;

	/* update cmdlist */
	if (lli) {
		payload[0] = value;
		iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xF0, 0xF0, 0);
		iris_update_pq_opt(PATH_DSI, iris_emv_instant_send);
		return;
	}

	if (!batch && !lli) {
		cmd[0] = IRIS_PWIL_ADDR + DATA_PATH_CTRL1;
		cmd[1] = value;
		iris_ocp_write_mult_vals(2, cmd);
		return;
	}

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + DATA_PATH_CTRL1, value, 0);
}

void iris_emv_pwil_configure_path_ctrl_1_i7(void)
{
#ifdef OLD_CODE
	//DATA_PATH_CTRL1 disable scale up, disable scale down and SR
	u32 value = 0x000002a4;

	if (iris_emv_get_path_bitwidth_i7() == IRIS_PATH_BW10)
		value = 0x000002a6;

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + DATA_PATH_CTRL1, value, 0);
#endif
	iris_emv_pwil_path_ctrl_1_configure_i7(false, false, false, (iris_emv_lli > 0), (iris_emv_batch > 0));
	//iris_emv_pwil_configure_sliceinfo_ctrl_i7(true, (iris_emv_lli > 0), (iris_emv_batch > 0));
#ifdef OLD_CODE
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_SLICE_INFO0, 0x04380001, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_SLICE_INFO1, 0x02400438, 0);
#endif
}

void iris_emv_pwil_configure_path_ctrl_1_enable_scaleup_i7(bool down)
{
#ifdef OLD_CODE
	//DATA_PATH_CTRL1 enabled scale up, disable scale down and SR
	u32 value = 0x00000284;

	if (iris_emv_get_path_bitwidth_i7() == IRIS_PATH_BW10)
		value = 0x00000286;

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + DATA_PATH_CTRL1, value, 0);
#endif
	iris_emv_pwil_path_ctrl_1_configure_i7(true, down, (iris_emv_sr > 0), (iris_emv_lli > 0), (iris_emv_batch > 0));
}

void iris_emv_pwil_configure_path_ctrl_1_enable_scalers_i7(void)
{
#ifdef OLD_CODE
	//DATA_PATH_CTRL1 enabled scale up, scale down and SR
	u32 value = 0x00000204;

	if (iris_emv_get_path_bitwidth_i7() == IRIS_PATH_BW10)
		value = 0x00000206;

	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + DATA_PATH_CTRL1, value, 0);
#endif
	iris_emv_pwil_path_ctrl_1_configure_i7(true, true, true, (iris_emv_lli > 0), (iris_emv_batch > 0));
	//iris_emv_pwil_configure_sliceinfo_ctrl_i7(false, (iris_emv_lli > 0), (iris_emv_batch > 0));
#ifdef OLD_CODE
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_SLICE_INFO0, 0x00f000f0, 0);
	iris_frc_reg_add_i7(IRIS_PWIL_ADDR + PWIL_SLICE_INFO1, 0x00f000f0, 0);
#endif
}

void iris_emv_frc_enable_i7(bool update, int videoWidth, int videoHeight, int numVdBuff, int numMvBuff)
{
	u32 mv_format = 0;//0--4x4, 1--8x8, 2--16x16
	u32 block_size = 0;
	u32 mv_hstride = 0;
	u32 mv_width = 0;
	u32 val_ctrl_reg0 = 0x001b0101;
	u32 val_frcc_reg6 = 0x00008000;//number of ext mv buffer set to 2
	u32 mvc_gen_cnt_thr, fi_meta_fast_entry_thr;
	u32 mv_base_start = iris_emv_pwil_mv1buffer_base_addr_get_i7();
	u32 offset = iris_emv_frc_mvdbuffer_offset_get_i7();
	u32 eco_resv_ctrl = (iris_emv_low_latency > 0) ? 0x00000c0c : 0x0000000c;

	IRIS_LOGI("%s:in", __func__);
	mvc_gen_cnt_thr = videoHeight >> 2;
	fi_meta_fast_entry_thr = mvc_gen_cnt_thr + 2;
	block_size = iris_emv_get_bmv_size_i7();
	mv_format = block_size/8;
	val_frcc_reg6 |= fi_meta_fast_entry_thr;
	val_frcc_reg6 |= (mv_format<<16);
	mv_width = ((videoWidth + block_size - 1)/block_size)*2;
	mv_hstride = iris_emv_pwil_stride_get_i7(mv_width, 24);
	if ((iris_emv_lli == 0) && (iris_emv_batch > 0))
		offset = 0x0000b000;

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_ENABLE_0, 0x0025d597, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_ENABLE_1, 0x94df9000, 0);
	val_ctrl_reg0 &= ~(0x003f0000);
	val_ctrl_reg0 |= ((numVdBuff&0x7) << 16);
	val_ctrl_reg0 |= ((numMvBuff&0x7) << 19);
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG0, val_ctrl_reg0, 0);
	//MV format 0: 4x4:w f2010020 0000805c f
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_REG6, val_frcc_reg6, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_EXT_MV_BSADR, mv_base_start, 0);
	//yuntian: mv hstride=ceiling(mv width*24/64) or (video width/blocksize)*48bit/64bit
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_EXT_MV_HSTRIDE, mv_hstride, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_EXT_MV_OFF, offset, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_ECO_LIST, 0x08, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_CTRL_ECO_REV_CTRL, eco_resv_ctrl, 0);
	if (update)
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_REG_SHDW, 0x00000002, 0);
}

void iris_emv_frc_fi_fastentry_enable_i7(bool enable, bool update)
{
	u32 value = 0x94df9000;

	//phase_fwd_en=1
	if (iris_emv_pq > 0)
		value = 0x94df9800;

	value |= enable ? (1<<2) : 0;
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_ENABLE_1, value, 0);
	if (update)
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_REG_SHDW, 0x00000002, 0);
}

void iris_emv_fi_searchrange_set_i7(bool update)
{
	//struct iris_cfg *pcfg = iris_get_cfg();
	u32 max_search_range, val_range_ctrl;
	u32 vrange_top, vrange_bot;
	u32 hres = (frc_setting->mv_hres / 4) * 4;
	u32 bpp = 2*iris_emv_get_path_bitwidth_i7();

	if (frc_setting->mv_hres % 4)
		hres += 4;
	max_search_range = 0x200000 / hres / bpp - 4;
	if (max_search_range > 510)
		max_search_range = 510;
	vrange_top = max_search_range / 2 - 1;
	vrange_bot = max_search_range / 2;

	if (hres < 480 && vrange_top > 128) {
		vrange_top = 128;
		vrange_bot = 128;
	}
	// vrange_bot should be odd number
	if (vrange_bot%2 == 0) {
		vrange_bot -= 1;
		vrange_top -= 1;
	}
	val_range_ctrl = vrange_top + (vrange_bot << 8);
	//w f2180040 00006564 f for 320x360
	if (iris_emv_formal > 0)
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_RANGE_CTRL, val_range_ctrl, 0);
	else
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_RANGE_CTRL, 0x00006564, 0);
	if (update)
		iris_frc_reg_add_i7(IRIS_FI_ADDR + FI_SHDW_CTRL, 0x00000001, 0);

	IRIS_LOGI("external MV frc: val_range_ctrl = 0x%x, mv algo 0x%04x%04x, hres %d, bpp %d",
		val_range_ctrl, frc_setting->mv_hres, frc_setting->mv_vres, hres, bpp);
}

void iris_emv_frc_phasemapping_disable_i7(bool update)
{
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_ENABLE_0, 0x0025c596, 0);
	if (update)
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_REG_SHDW, 0x00000002, 0);
}

void iris_emv_fi_force_repeat_enable_i7(bool enable, bool update)
{
	u32 value = enable ? 0x2 : 0;

	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_ECO_LIST, 0x08, 0);
	iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_MVF_CTRL_0, value, 0);
	if (update)
		iris_frc_reg_add_i7(IRIS_FRC_MIF_ADDR + FRCC_REG_SHDW, 0x00000001, 0);
}

void iris_emv_mvf_borderthreshold_i7(bool update)
{
	iris_frc_reg_add_i7(IRIS_MVF_ADDR + EMVSEL_CFG2, 0x42200480, 0);
	if (iris_emv_pq > 0) {
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + EMVSEL_CFG10, 0x10112134, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + EMVSEL_CFG13, 0x0f002780, 0);
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + EMVSEL_CFG19, 0x00411100, 0);
		if (iris_emv_hugemot > 0)
			iris_frc_reg_add_i7(IRIS_MVF_ADDR + EMVSEL_CFG15, 0x000280a0, 0);
		else
			iris_frc_reg_add_i7(IRIS_MVF_ADDR + EMVSEL_CFG15, 0x000294a5, 0);
	}
	if (update)
		iris_frc_reg_add_i7(IRIS_MVF_ADDR + MVF_SHDW_CTRL, 0x13, 0);
}

void iris_emv_swap_channels_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int hres_2nd = pcfg->frc_setting.hres_2nd;
	int vres_2nd = pcfg->frc_setting.vres_2nd;
	bool dsc_2nd = pcfg->frc_setting.dsc_2nd;

	iris_emv_pwil_shortpacket_mask_i7(1);
	iris_emv_pwil_capt_enable_i7(0);
	iris_emv_rx1_switch_timing_i7(hres_2nd, vres_2nd, dsc_2nd);//4
	iris_emv_sys_channels_swap_i7();//2
	//show osd(game+mouse), flush gmvd on video/graphic channel
	iris_emv_blending_flush_enable_i7(true, true, false);
	iris_emv_clockinout_i7(kt_flush_video);
	//iris_emv_pwil_1_capt_enable_i7(1);
	if (iris_emv_capen_aftswap > 0)
		iris_emv_pwil_capt_enable_i7(1);
	iris_emv_pwil_shortpacket_mask_i7(0);
}

void iris_emv_restore_channels_i7(void)
{
	iris_emv_sys_channels_restore_i7();
	iris_emv_blending_flush_enable_i7(false, false, true);
	iris_emv_clockinout_i7(kt_flush_osd);
}

void iris_emv_reverse_swap_channels_i7(void)
{
	if (iris_emv_instant_send) {
		iris_emv_pwil_shortpacket_mask_i7(1);
		iris_emv_pwil_capt_enable_i7(0);
		iris_emv_pwil_1_capt_enable_i7(0);
		iris_emv_rx1_set_disp_mode_meta_i7(0, 1, false, false);
		iris_emv_restore_channels_i7();
		iris_emv_pwil_display_mode_set_i7(0, 1, true, true, false, false);
		iris_emv_pwil_shortpacket_mask_i7(0);
	}

	if (iris_emv_capen_aftswap > 0)
		iris_emv_pwil_capt_enable_i7(1);
}

static bool iris_emv_pipelines_idle_i7(void);

void iris_emv_exit_prepare_i7(void)
{
	if (iris_emv_instant_send) {
		iris_emv_pwil_shortpacket_mask_i7(1);
		iris_emv_pwil_capt_enable_i7(0);
		//show osd, flush v/g
		iris_emv_blending_flush_enable_i7(true, true, false);
		iris_emv_pwil_configure_path_ctrl_1_i7();//no scale
		iris_emv_pwil_switch_to_pt_in_raw_graphic_i7();//pt,graphic,raw,dual
		if (iris_emv_lli > 0)
			iris_emv_pwil_update_lli_trigger_i7();
		iris_emv_pwil_shortpacket_mask_i7(0);
		iris_emv_pwil_capt_enable_i7(1);
	}
}

void iris_emv_exit_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int disp_hres = pcfg->frc_setting.disp_hres;
	int disp_vres = pcfg->frc_setting.disp_vres;
	bool disp_dsc = pcfg->frc_setting.disp_dsc;
	int vid_proc = iris_emv_pwil_video_ctrl5_get_i7();
	int vid_hres = vid_proc & 0xffff;
	int vid_vres = (vid_proc >> 16) & 0xffff;

	IRIS_LOGI("%s:in [0x%04x%04x][0x%04x%04x][0x%04x%04x][0x%04x%04x]", __func__,
		pcfg->frc_setting.disp_hres,
		pcfg->frc_setting.disp_vres,
		pcfg->frc_setting.hres_2nd,
		pcfg->frc_setting.vres_2nd,
		pcfg->frc_setting.mv_hres,
		pcfg->frc_setting.mv_vres,
		pcfg->emv_info.gameWidth,
		pcfg->emv_info.gameHeight);
	//show osd(game+mouse, opacity should already restored),
	//and flush gmvd on video/graphic channel
	if (iris_emv_instant_send)
		iris_emv_pwil_capt_enable_i7(0);
	iris_emv_pwil_configure_path_ctrl_1_enable_scalers_i7();
	iris_emv_pwil_configure_graphic_ctrl_i7(disp_hres, disp_vres, disp_dsc, disp_dsc,
					(iris_emv_lli > 0), (iris_emv_batch > 0), false);
	iris_emv_pwil_configure_video_ctrl_i7(0, 0, disp_hres, disp_vres,
					vid_hres, vid_vres,
					disp_hres, disp_vres, true, disp_dsc, true, 3,
					(iris_emv_lli > 0), (iris_emv_batch > 0), false);
	iris_emv_pwil_frc_ctrl_i7(false, true, true, false, 3, 3,
				(iris_emv_lli > 0), (iris_emv_batch > 0));
	iris_emv_pwil_configure_dma_ctrl_i7(false, (iris_emv_lli > 0), (iris_emv_batch > 0));
	iris_emv_pwil_buffers_alloc_i7(false);
	iris_emv_pwil_configure_cmd_ctrl_i7(0, disp_hres,
					0, disp_vres,
					(iris_emv_lli > 0), (iris_emv_batch > 0));
	if ((iris_emv_lli > 0) && iris_emv_instant_send)
		iris_emv_pwil_exit_lli_trigger_i7();

	if (iris_emv_debug > 0)
		iris_pwil_0_capten_get_debug_i7();

	if (iris_emv_instant_send) {
		bool idle = true;

		iris_emv_pwil_1_capt_enable_i7(0);
		iris_emv_restore_channels_i7();
		iris_emv_pwil_display_mode_set_i7(0, 1, true, true, false, false);
		idle = iris_emv_pipelines_idle_i7();
		IRIS_LOGI("%s(%d), GET_IRIS_PIPELINES IDLEs %d for channels swap revert", __func__, __LINE__, idle);
		if (idle == false)
			iris_emv_pwil_1_swrst_enable_i7(1);
	}

	if (iris_emv_debug > 0)
		iris_pwil_0_regdump_get_debug_i7();

	iris_emv_pwil_capt_enable_i7(1);
	last_reginfo_saved = false;
}

void iris_emv_enter_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int hres_2nd = pcfg->frc_setting.hres_2nd;
	int vres_2nd = pcfg->frc_setting.vres_2nd;
	bool dsc_2nd = pcfg->frc_setting.dsc_2nd;
	bool disp_dsc = pcfg->frc_setting.disp_dsc;
	int gameWidth = pcfg->emv_info.gameWidth;
	int gameHeight = pcfg->emv_info.gameHeight;
	int captLeft = pcfg->emv_info.gameLeft;
	int captTop = pcfg->emv_info.gameTop;
	int captWidth = pcfg->emv_info.gameWidthSrc;
	int captHeight = pcfg->emv_info.gameHeightSrc;
	int bmvSize = iris_emv_get_bmv_size_i7();
	int mvdHeight = 0;
	bool scaledown = (pcfg->emv_info.overflow == 0xe0);

	IRIS_LOGI("%s:in [0x%04x%04x][0x%04x%04x][0x%04x%04x][0x%04x%04x]", __func__,
		pcfg->frc_setting.disp_hres,
		pcfg->frc_setting.disp_vres,
		pcfg->frc_setting.hres_2nd,
		pcfg->frc_setting.vres_2nd,
		pcfg->emv_info.gameWidthSrc,
		pcfg->emv_info.gameHeightSrc,
		pcfg->emv_info.gameWidth,
		pcfg->emv_info.gameHeight);

	pcfg->frc_setting.emv_hres = ((gameWidth + bmvSize - 1)/bmvSize)*2;
	pcfg->frc_setting.emv_vres = (gameHeight + bmvSize - 1)/bmvSize;
	mvdHeight = pcfg->frc_setting.emv_vres;

	if ((iris_emv_base4_8x8 > 0) && (iris_emv_fixed_bmvsize > 0)) {
		pcfg->frc_setting.emv_hres = ((gameWidth+3)/4)*2;
		pcfg->frc_setting.emv_vres = (gameHeight + 3)/4;
		mvdHeight = pcfg->frc_setting.emv_vres;
	}

	if (iris_emv_hold_dispmeta > 0)
		iris_emv_pwil_shortpacket_mask_i7(1);
	iris_emv_pwil_capt_enable_i7(0);
	iris_emv_pwil_configure_path_ctrl_1_i7();//3
	iris_emv_pwil_configure_graphic_ctrl_i7(hres_2nd, vres_2nd, dsc_2nd, disp_dsc,
					(iris_emv_lli > 0), (iris_emv_batch > 0), true);//4
	iris_emv_pwil_configure_cmd_ctrl_i7(0, hres_2nd,
					0, vres_2nd,
					(iris_emv_lli > 0), (iris_emv_batch > 0));//2
	iris_emv_pwil_switch_to_pt_in_raw_graphic_i7();//2
	iris_emv_pwil_configure_path_ctrl_0_i7((iris_emv_lli > 0), (iris_emv_batch > 0));//1
	iris_emv_pwil_configure_video_ctrl_i7(captLeft, captTop,
					captWidth, captHeight,
					gameWidth, gameHeight,
					hres_2nd, vres_2nd,
					true, disp_dsc, scaledown, (iris_emv_twobuffers > 0) ? 2 : 3,
					(iris_emv_lli > 0), (iris_emv_batch > 0), true);
	iris_emv_pwil_configure_mv_ctrl_i7(gameWidth, gameHeight,
					(iris_emv_lli > 0), (iris_emv_batch > 0));//9
	iris_emv_pwil_configure_dma_ctrl_i7(true, (iris_emv_lli > 0), (iris_emv_batch > 0));//1
	iris_emv_pwil_buffers_alloc_i7(true);//
	if ((iris_emv_osd_resize_control&0x01) == 0x01) {
		iris_emv_efifo_enable_ctrl_i7(false);
		iris_emv_resize_osd_buffer_i7(true);
	}
	if (iris_emv_lli > 0)
		iris_emv_pwil_lli_trigger_i7();
	//capture from video pipe
	iris_emv_pwil_display_mode_set_i7(0, 2, false, true, false, true);
	iris_emv_pwil_mv_capt_enable_i7(true, false);//2
	iris_emv_frc_enable_i7(true, gameWidth, gameHeight, (iris_emv_twobuffers > 0) ? 2 : 3, 2);
	iris_emv_pwil_vfr_disable_i7((iris_emv_lli > 0), (iris_emv_batch > 0));
	if (iris_emv_scaler_inpt > 0)
		iris_emv_pwil_configure_path_ctrl_1_enable_scaleup_i7(scaledown);
	iris_emv_fi_searchrange_set_i7(true);
	iris_emv_gmd_disable_i7();//inlcuded iris_emv_pwil_update_lli_trigger_i7
	iris_emv_mvf_borderthreshold_i7(true);
	iris_emv_frc_fi_fastentry_enable_i7(true, true);
	iris_emv_frc_phasemapping_disable_i7(true);
	iris_emv_frc_ds_ctrl_i7(false, false, true);
	iris_emv_pwil_1_swrst_enable_i7(0);
	iris_emv_pwil_1_capt_enable_i7(1);//just in case
	iris_pwil0_intr_raw_clear_i7(true);
	if (iris_emv_capen_inpt > 0)
		iris_emv_pwil_capt_enable_i7(1);
	IRIS_LOGI("%s:out", __func__);
}

void irisSetDisableDsppPq_i7(bool enable)
{
	iris_pq_disable = enable ? 0x7 : 0;
	IRIS_LOGI("external MV frc meta: disable dspp pq: %d", iris_pq_disable);
}

int toGameWidth_i7(int mvdWidth, int blockSize)
{
	return (mvdWidth/2)*blockSize;
}

int toMvdWidth_i7(int gameWidth, int blockSize)
{
	return ((gameWidth + blockSize - 1)/blockSize)*2;
}

int toMvdHeight_i7(int gameHeight, int blockSize)
{
	return ((gameHeight + blockSize - 1)/blockSize);
}

int toMvdBlockSizeWidth_i7(int gameWidth, int mvdWidth)
{
	int mvdPoints = mvdWidth/2;

	return (gameWidth + mvdPoints - 1)/mvdPoints;
}

int toMvdBlockSizeHeight_i7(int gameHeight, int mvdHeight)
{
	return (gameHeight + mvdHeight - 1)/mvdHeight;
}

bool isValidMvdBlockSize_i7(int blockSize)
{
	return (blockSize == 4 || blockSize == 8 || blockSize == 16);
}

void irisSetExtMvFrc_i7(struct extmv_frc_meta *meta)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (meta == NULL) {
		IRIS_LOGE("meta is invalid!");
		return;
	}

	IRIS_LOGI("external MV frc meta: mode %d valid %d, bmvsize %d, orientation %d",
		meta->mode,
		meta->valid,
		meta->bmvSize,
		meta->orientation);

	if (meta->mode == 0xECEA) {
		int mvdHeight = 0;
		int mvdWidth = 0;
		int mvdBlockSize = 0;

		if ((meta->gameWidthSrc&(u32)0x1) != 0)
			meta->gameWidth = meta->gameWidthSrc + 1;
		else
			meta->gameWidth = meta->gameWidthSrc;
		if ((meta->gameHeightSrc&(u32)0x1) != 0)
			meta->gameHeight = meta->gameHeightSrc + 1;
		else
			meta->gameHeight = meta->gameHeightSrc;

		if ((meta->containerWidth != 0) && (meta->containerHeight != 0)) {
			if (meta->gameWidth == meta->gmvdWidthSrc
				|| meta->gameWidthSrc == meta->gmvdWidthSrc) {
				//layout 0 1 2
				iris_emv_layout = 0;

				if (meta->gmvdTop > 0)
					iris_emv_layout = 1;
				else if (meta->gmvdHeightSrc == meta->containerHeight)
					iris_emv_layout = 2;

				mvdHeight = meta->gmvdHeightSrc - meta->gameHeightSrc - 1;
				mvdBlockSize = toMvdBlockSizeHeight_i7(meta->gameHeight, mvdHeight);
				if (!isValidMvdBlockSize_i7(mvdBlockSize)) {
					if (iris_emv_layout != 2) {
						meta->mode = 0;
						IRIS_LOGE("i7:META: Fatal! Invalid EMV block size(%d) for layout %d",
							mvdBlockSize, iris_emv_layout);
						return;
					}
					mvdBlockSize = 4;//only 4x4 supported for layout2
					mvdHeight = toMvdHeight_i7(meta->gameHeight, mvdBlockSize);
				}
				mvdWidth = toMvdWidth_i7(meta->gameWidth, mvdBlockSize);
				meta->gameLeft = meta->gmvdLeft;
				meta->gameTop = meta->gmvdTop + meta->gmvdHeightSrc - meta->gameHeightSrc;
				meta->mvd0Left = meta->gmvdLeft;
				meta->mvd0Top = meta->gmvdTop + 1;
				if (iris_emv_layout == 2)
					meta->mvd0Top = meta->gameTop - mvdHeight;
				meta->mvd0Width = mvdWidth;
				meta->mvd0Height = mvdHeight;
				meta->mvd1Left = meta->mvd0Left + mvdWidth;
				meta->mvd1Top = meta->mvd0Top;
				meta->mvd1Width = mvdWidth;
				meta->mvd1Height = mvdHeight;
			} else if (meta->gmvdHeightSrc == meta->containerHeight) {
				iris_emv_layout = 3;
				mvdWidth = meta->gmvdWidthSrc - meta->gameWidthSrc;
				mvdBlockSize = toMvdBlockSizeWidth_i7(meta->gameWidth, mvdWidth);
				if (!isValidMvdBlockSize_i7(mvdBlockSize)) {
					meta->mode = 0;
					IRIS_LOGE("i7:META: Fatal! Invalid EMV block size(%d) for layout 3",
						mvdBlockSize);
					return;
				}
				mvdHeight = toMvdHeight_i7(meta->gameHeight, mvdBlockSize);
				meta->gameLeft = meta->gmvdLeft;
				meta->gameTop = meta->gmvdTop + meta->gmvdHeightSrc - meta->gameHeightSrc;
#ifdef MV0_AT_BOTTOM
				meta->mvd0Left = meta->gmvdLeft + meta->gameWidthSrc;
				meta->mvd0Top = meta->gmvdTop + meta->gmvdHeightSrc - mvdHeight;
				meta->mvd0Width = mvdWidth;
				meta->mvd0Height = mvdHeight;
				meta->mvd1Left = meta->mvd0Left;
				meta->mvd1Top = meta->mvd0Top - mvdHeight;
				meta->mvd1Width = mvdWidth;
				meta->mvd1Height = mvdHeight;
#else
				meta->mvd1Left = meta->gmvdLeft + meta->gameWidthSrc;
				meta->mvd1Top = meta->gmvdTop + meta->gmvdHeightSrc - mvdHeight;
				meta->mvd1Width = mvdWidth;
				meta->mvd1Height = mvdHeight;
				meta->mvd0Left = meta->mvd1Left;
				meta->mvd0Top = meta->mvd1Top - mvdHeight;
				meta->mvd0Width = mvdWidth;
				meta->mvd0Height = mvdHeight;
#endif
			}
		}

		meta->overflow = 0;
		if (iris_emv_layout == 0)
			mvdHeight = meta->gmvdHeightSrc - meta->gameHeightSrc - 1;

		if (mvdHeight <= 0) {
			meta->mode = 0;
			return;
		}
		if (meta->gameWidth*meta->gameHeight > iris_emv_limit) {
			int mvdBlockNumMax = 0;

			meta->gameHeight = mvdHeight*4;
			meta->gameWidth = iris_emv_limit/meta->gameHeight;
			mvdBlockNumMax = (meta->gameWidth + 3)/4;
			if (mvdBlockNumMax <= 0)
				return;
			mvdBlockSize = (meta->gameWidthSrc + mvdBlockNumMax - 1)/mvdBlockNumMax;
			if (mvdBlockSize <= 0)
				return;
			mvdWidth = (meta->gameWidthSrc + mvdBlockSize - 1)/mvdBlockSize;
			meta->gameWidth = mvdWidth*4;
			meta->overflow = 0xe0;
			IRIS_LOGI("i7:META: mvdH=%d,mvdW=%d,BlockMax=%d,BlockSize=%d, frc 0x%04x%04x",
			mvdHeight, mvdWidth,
			mvdBlockNumMax, mvdBlockSize,
			meta->gameWidth, meta->gameHeight);
			if (mvdHeight <= 0 || mvdWidth <= 0) {
				meta->mode = 0;
				meta->gameWidth = 160;
				meta->gameHeight = 160;
				IRIS_LOGE("i7:META: Fatal! Invalid EMV meta!");
			}
		}

		IRIS_LOGI("META:algo_s 0x%04x%04x,pa:%d,%d,gmvd:%d,%d,0x%04x%04x,"
		"game:%d,%d,0x%04x%04x,ov 0x%x,mv0:%d,%d,mv1:%d,%d,0x%04x%04x",
			meta->gameWidthSrc, meta->gameHeightSrc,
			meta->gameWidthSrc&(u32)0x1,
			meta->gameHeightSrc&(u32)0x1,
			meta->gmvdLeft, meta->gmvdTop,
			meta->gmvdWidthSrc, meta->gmvdHeightSrc,
			meta->gameLeft, meta->gameTop,
			meta->gameWidth, meta->gameHeight,
			meta->overflow,
			meta->mvd0Left, meta->mvd0Top,
			meta->mvd1Left, meta->mvd1Top,
			meta->mvd1Width, meta->mvd1Height);
	}
	memcpy(&pcfg->emv_info, meta, sizeof(struct extmv_frc_meta));
	if (meta->mode == 0xECEA) {
		iris_extmv_frc_mode = 1;
		iris_emv_bmv_size = meta->bmvSize;
		iris_emv_game_landscape = (meta->orientation > 0) ? 1 : 0;
		iris_emv_frc_mode_en_set_i7(true);
	} else {
		iris_extmv_frc_mode = 0;
		iris_emv_frc_mode_en_set_i7(false);
	}

	iris_pwil0_intr_raw_clear_i7(true);

	if (iris_emv_debug > 0)
		iris_pwil_0_regdump_get_debug_i7();
	if ((iris_emv_shock > 0) && (iris_emv_rescue_enable > 0))
		iris_emv_mode_switch_i7(IRIS_EMV_HEALTH_UP);
	else
		iris_health_care2_i7();
}

bool iris_emv_game_size_changed_i7(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if ((pcfg->frc_setting.mv_hres == pcfg->emv_info.gameWidth)
		&& (pcfg->frc_setting.mv_vres == pcfg->emv_info.gameHeight)
		&& (pcfg->emv_info.gmvdWidthSrcLast == pcfg->emv_info.gmvdWidthSrc)
		&& (pcfg->emv_info.gmvdHeightSrcLast == pcfg->emv_info.gmvdHeightSrc)
		&& (pcfg->emv_info.gameWidthSrcLast == pcfg->emv_info.gameWidthSrc)
		&& (pcfg->emv_info.gameHeightSrcLast == pcfg->emv_info.gameHeightSrc))
		return false;
	pcfg->emv_info.gmvdWidthSrcLast = pcfg->emv_info.gmvdWidthSrc;
	pcfg->emv_info.gmvdHeightSrcLast = pcfg->emv_info.gmvdHeightSrc;
	pcfg->emv_info.gameWidthSrcLast = pcfg->emv_info.gameWidthSrc;
	pcfg->emv_info.gameHeightSrcLast = pcfg->emv_info.gameHeightSrc;
	return true;
}

void iris_emv_clockinout_i7(int index)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->emv_perf.kt[index] = ktime_get();
}

uint32_t iris_emv_perf_dump_i7(int startIndex, int endIndex)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t timeus0 = 0;

	if (pcfg == NULL)
		return 0;

	timeus0 = (u32)ktime_to_us(pcfg->emv_perf.kt[endIndex]);
	timeus0 -= (u32)ktime_to_us(pcfg->emv_perf.kt[startIndex]);
	return timeus0;
}

void iris_emv_on_mipi1_up_i7(void)
{
	if (iris_emv_game_mode_enabled_i7()) {
		iris_emv_mode_switch_i7(IRIS_EMV_CLOSE_PIPE_1);
		iris_emv_clockinout_i7(kt_iris2nd_holdon);
		IRIS_LOGI("%s: capt_en close pipe 1", __func__);
		if (iris_emv_new_dual_enabled_i7()) {
			iris_emv_mode_switch_i7(IRIS_EMV_ON_PREPARE);
			iris_emv_mode_switch_i7(IRIS_EMV_ON_SWAP);
		}
	} else
		iris_osd_pipeline_switch_i7(false);
}

void iris_osd_pipeline_switch_i7(bool on)
{
	IRIS_LOGI("%s: osd pipeline switch to %d", __func__, on);
	if (on)
		iris_emv_mode_switch_i7(IRIS_EMV_OPEN_PIPE_1);
	else
		iris_emv_mode_switch_i7(IRIS_EMV_CLOSE_PIPE_1);
}

void iris_emv_on_mipi1_down_i7(void)
{
	if (iris_emv_game_mode_enabled_i7())
		iris_emv_mode_switch_i7(IRIS_EMV_OFF_FINAL);
}

void iris_emv_on_dual_open_i7(void)
{
	if (iris_emv_game_mode_enabled_i7() && (!iris_emv_new_dual_enabled_i7()))
		iris_emv_mode_switch_i7(IRIS_EMV_ON_PREPARE);
}

static u32 iris_pwil_status_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_PWIL_0_ADDR + PWIL_STATUS, DSI_CMD_SET_STATE_HS);
	IRIS_LOGI("%s(%d), PWIL_0_PWIL_STATUS = %#x", __func__, __LINE__, rc);

	return rc;
}

static bool iris_emv_pipelines_idle_i7(void)
{
	u32 rc = iris_pwil_status_get_i7();
	bool idles = (rc & 0x01);

	idles &= (rc >> 31) & 0x01;
	idles &= (rc >> 20) & 0x01;
	IRIS_LOGI("%s(%d), idles: %#x, pwil status: %#x", __func__, __LINE__, idles, rc);

	return idles;
}

bool iris_emv_on_dual_on_i7(bool success)
{
	static int timeout;

	if (success
		&& iris_emv_game_mode_enabled_i7()
		&& (!iris_emv_new_dual_enabled_i7())
		&& (iris_emv_mode_get_i7() == IRIS_EMV_ON_PREPARE)) {
		iris_emv_mode_switch_i7(IRIS_EMV_ON_SWAP);
		timeout = 3;
		pr_err("iris test GET_IRIS_BLEND_STATE: swap channels\n");
	}

	if ((iris_emv_mode_get_i7() == IRIS_EMV_ON_SWAP) && (timeout > 0)) {
		bool idle = iris_emv_pipelines_idle_i7();

		IRIS_LOGI("%s(%d), GET_IRIS_PIPELINES IDLEs %d for swap channels %d",
				__func__, __LINE__, idle, timeout);
		timeout--;
		if (idle && (iris_emv_debug > 0))
			iris_pwil_0_regdump_get_debug_i7();
		if ((timeout == 0) && (!idle)) {
			timeout = 1;
			if (iris_emv_debug > 0)
				iris_pwil_0_regdump_get_debug_i7();
			iris_health_care_i7();
		}
		return idle;
	}

	if (success
		&& (!iris_emv_game_mode_enabled_i7()))
		iris_osd_pipeline_switch_i7(true);

	return true;
}

void iris_emv_on_dual_closing_i7(void)
{
	if (iris_emv_game_mode_enabled_i7())
		iris_emv_mode_switch_i7(IRIS_EMV_OFF_PREPARE);
}

void iris_emv_on_dual_close_i7(void)
{
	if (iris_emv_game_mode_enabled_i7())
		iris_emv_mode_switch_i7(IRIS_EMV_OFF_PT);
}

bool iris_emv_set_frc_prepare_i7(void)
{
	if (iris_emv_game_mode_enabled_i7()) {
		iris_emv_mode_switch_i7(IRIS_EMV_ON_CONFIGURE);
		return true;
	}
	return false;
}

bool iris_emv_set_frc_on_i7(void)
{
	if (iris_emv_game_mode_enabled_i7()) {
		if (iris_blending_flush_state_get_i7() == 0)
			pr_err("iris MEMC_CTRL_PT2FRC: EMV ON: UNEXPECTED FLUSHST\n");
		pr_err("iris test MEMC_CTRL_PT2FRC: EMV ON\n");
		iris_emv_mode_switch_i7(IRIS_EMV_ON_FRC);
		return true;
	}
	return false;
}

void iris_emv_after_frc_on_i7(void)
{
	if (iris_emv_game_mode_enabled_i7()) {
		IRIS_LOGI("iris test MEMC_CTRL_PT2FRC: After EMV ON");
		iris_emv_mode_switch_i7(IRIS_EMV_ON_FRC_POST);
	}
}

bool iris_emv_set_frc_off_i7(void)
{
	if (iris_emv_game_mode_enabled_i7()) {
		pr_err("iris test MEMC_CTRL_PT_PREPARE: EMV OFF\n");
		iris_emv_clockinout_i7(kt_pt_prep_start);
		iris_emv_mode_switch_i7(IRIS_EMV_OFF_CONFIGURE);
		iris_emv_clockinout_i7(kt_pt_prep_end);
		return true;
	}
	return false;
}

int iris_emv_mode_get_i7(void)
{
	return iris_emv_current_mode;
}

u32 iris_blending_intstat_get_i7(void)
{
	u32 rc = 0;

	rc = iris_ocp_read(IRIS_BLENDING + 0x1ffe4, DSI_CMD_SET_STATE_HS);
	IRIS_LOGI("iris test BLENDING INTSTAT = %x", rc);
	rc &= 0x8;
	return (rc == 0x8);
}

void iris_blending_intstat_clear_i7(bool direct)
{
	if (direct)
		iris_ocp_write_val(IRIS_BLENDING + 0x1fff0, 0xf);
	else
		iris_frc_reg_add_i7(IRIS_BLENDING + 0x1fff0, 0xf, 0);
}

void iris_emv_pwil_swrst_enable_i7(int enable)
{
	u32 value = iris_pwil_0_ctrl0_get_i7();

	IRIS_LOGI("%s: %d, 0x%x", __func__, enable, value);
	if (enable > 0)
		value |= 0x3;
	else
		value &= ~(0x3);
	iris_frc_reg_add_i7(IRIS_PWIL_0_ADDR + PWIL_CTRL0, value, 0);
}

void iris_emv_shock_rescue_down_i7(void)
{
	iris_emv_pwil_shortpacket_mask_i7(1);
	iris_emv_pwil_capt_enable_i7(0);
	iris_emv_pwil_swrst_enable_i7(1);
	IRIS_LOGI("iris test rescue: sleep down");
}

void iris_emv_shock_rescue_up_i7(void)
{
	iris_emv_pwil_swrst_enable_i7(0);
	iris_emv_pwil_capt_enable_i7(1);
	iris_emv_pwil_shortpacket_mask_i7(0);
	iris_blending_intstat_clear_i7(false);
	iris_emv_shock = 0;
	if (iris_emv_rescue_enable > 100)
		iris_emv_rescue_enable = 100;
	iris_emv_rescue_enable++;
	IRIS_LOGI("iris test rescue: wakeup");
}

bool iris_emv_shock_check_i7(void)
{
	if (iris_emv_rescue_enable <= 0)
		return false;
	iris_emv_shock = (iris_blending_intstat_get_i7()) ? 1 : 0;
	return (iris_emv_shock > 0);
}

bool iris_health_care_i7(void)
{
	if (iris_emv_shock_check_i7()) {
		iris_emv_mode_switch_i7(IRIS_EMV_HEALTH_DOWN);
		udelay(3 * 1000);
		iris_emv_mode_switch_i7(IRIS_EMV_HEALTH_UP);
		return true;
	}
	return false;
}

bool iris_health_check_i7(void)
{
	if (iris_emv_healthfailed_sim > 0) {
		iris_emv_healthfailed_sim = 0;
		return false;
	}
	if (iris_rx0_result_get_i7() != 0)
		return false;
	if (iris_emv_shock_check_i7())
		return false;
	return true;
}

bool iris_health_care2_i7(void)
{
	if (!iris_health_check_i7()) {
		iris_emv_mode_switch_i7(IRIS_EMV_HEALTH_DOWN);
		udelay(3 * 1000);
		iris_emv_mode_switch_i7(IRIS_EMV_HEALTH_UP);
		return true;
	}
	return false;
}

bool iris_emv_on_display_mode_i7(u32 state)
{
	if (iris_emv_game_mode_enabled_i7()) {
		if ((state == 8)
			&& (iris_emv_mode_get_i7() == IRIS_EMV_ON_FRC)
			&& (!iris_emv_healthcheck_done)) {
			bool well = true;
			iris_emv_healthcheck_done = true;
			well = iris_health_check_i7();
			if (well)
				iris_emv_after_frc_on_i7();
			return well;
		}
	}
	return true;
}

void iris_emv_mode_switch_i7(int mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	frc_setting = &pcfg->frc_setting;
	if (mode < IRIS_EMV_MIN || mode > IRIS_EMV_MAX) {
		iris_emv_ext_dbg_i7(mode);
		return;
	}

	IRIS_LOGI("%s:from mode %d [shape %d] into mode %d", __func__,
		iris_emv_current_mode, iris_emv_shaped, mode);

	iris_emv_current_mode = mode;

	iris_memc_cmd_payload_init_i7();
	switch (mode) {
	case IRIS_EMV_ON_PREPARE:
		//dual PT:show osd(game+mouse) and flush gmvd
		iris_emv_blending_flush_enable_i7(false, false, true);
		iris_emv_clockinout_i7(kt_flush_osd);
		iris_sdr2hdr_allow(false);
		iris_emv_shaped = 1;
		iris_emv_healthcheck_done = false;
		break;
	case IRIS_EMV_ON_SWAP:
		//dual PT: swap channels
		iris_emv_swap_channels_i7();
		iris_emv_clockinout_i7(kt_dualon_swap);
		iris_emv_shaped = 2;
		break;
	case IRIS_EMV_ON_CONFIGURE:
		//frc: configure for gmvd
		iris_emv_enter_i7();
		iris_emv_shaped = 3;
		break;
	case IRIS_EMV_ON_FRC:
		//show gmvd + osd
		iris_emv_pwil_display_mode_set_i7(1, 2, false, true, false, true);
		if (iris_emv_scaler_infrc > 10) {
			iris_emv_pwil_configure_path_ctrl_1_enable_scaleup_i7(false);
			if (iris_emv_lli > 0)
				iris_emv_pwil_lli_trigger_i7();
		}
		if (iris_emv_lli > 0)
			iris_emv_pwil_shortpacket_mask_i7(0);
		iris_emv_pwil_duplicate_frame_filter_i7(false);
		iris_emv_pwil_capt_enable_i7(1);
		iris_emv_shaped = 4;
		break;
	case IRIS_EMV_ON_FRC_POST:
		if (iris_emv_scaler_infrc > 0 && iris_emv_scaler_infrc < 10) {
			iris_emv_pwil_configure_path_ctrl_1_enable_scaleup_i7(false);
			if (iris_emv_lli > 0)
				iris_emv_pwil_lli_trigger_i7();
		}
		break;
	case IRIS_EMV_ON_FRC_FINAL:
		iris_emv_blending_flush_enable_i7(false, false, false);
		iris_emv_clockinout_i7(kt_flush_none);
		iris_sdr2hdr_allow(true);
		break;
	case IRIS_EMV_OFF_PREPARE:
		if (iris_emv_shaped < 3) {
			//on abort
			IRIS_LOGE("%s: aborting ...", __func__);
			if (iris_emv_shaped == 2)
				iris_emv_reverse_swap_channels_i7();

			iris_emv_shaped = 5;
			break;
		}
		if (iris_emv_shaped >= 5) {
			IRIS_LOGI("%s: closing ...", __func__);
			break;
		}
		iris_sdr2hdr_allow(false);
		iris_emv_fi_force_repeat_enable_i7(true, true);
		iris_emv_pwil_duplicate_frame_filter_i7(false);
		iris_emv_pwil_mv_capt_enable_i7(false, false);
		if (iris_emv_lli > 0)
			iris_emv_pwil_lli_trigger_i7();
		iris_emv_shaped = 5;
		break;
	case IRIS_EMV_CLOSE_THE_SET:
		iris_emv_pwil_capt_enable_i7(0);
		iris_emv_pwil_1_capt_enable_i7(0);
		iris_emv_shaped = 6;
		break;
	case IRIS_EMV_OFF_CONFIGURE:
		iris_emv_exit_i7();
		iris_emv_shaped = 7;
		break;
	case IRIS_EMV_OFF_PT:
		if (iris_emv_shaped < 7)
			iris_emv_shock_check_i7();
		if (iris_emv_shaped < 5)
			IRIS_LOGI("%s: Warning! unexpected shape, please check!", __func__);
		if (iris_emv_debug > 0)
			iris_pwil_0_regdump_get_debug_i7();
		iris_emv_pwil_shortpacket_mask_i7(0);
		iris_emv_pwil_capt_enable_i7(1);
		iris_emv_shaped = 8;
		break;
	case IRIS_EMV_OFF_FINAL:
		iris_sdr2hdr_allow(true);
		iris_emv_blending_flush_enable_i7(false, false, false);
		iris_emv_current_mode = 0;
		iris_emv_clockinout_i7(kt_flush_none);
		iris_emv_shaped = 0;
		iris_emv_pwil_capt_enable_i7(1);
		if (iris_emv_shock_check_i7())
			iris_emv_shock_rescue_down_i7();
		break;
	case IRIS_EMV_CLOSE_PIPE_1:
		iris_emv_pwil_1_capt_enable_i7(0);
		iris_emv_pwil_1_swrst_enable_i7(1);
		break;
	case IRIS_EMV_OPEN_PIPE_1:
		iris_emv_pwil_1_swrst_enable_i7(0);
		iris_emv_pwil_1_capt_enable_i7(1);
		break;
	case IRIS_EMV_CLOSE_PIPE_0:
		iris_emv_pwil_capt_enable_i7(0);
		break;
	case IRIS_EMV_OPEN_PIPE_0:
		iris_emv_pwil_capt_enable_i7(1);
		break;
	case IRIS_EMV_OFF_PRECONFIG:
		iris_emv_exit_prepare_i7();
		break;
	case IRIS_EMV_HELPER_A:
		iris_emv_resize_osd_buffer_i7(false);
		break;
	case IRIS_EMV_DUMP:
		iris_emv_reverse_swap_channels_i7();
		break;
	case IRIS_EMV_HEALTH_DOWN:
		iris_emv_shock_rescue_down_i7();
		break;
	case IRIS_EMV_HEALTH_UP:
		iris_emv_shock_rescue_up_i7();
		break;
	default:
		break;
	}
	iris_memc_cmd_payload_send_i7();
}

void iris_emv_local_reset_i7(void)
{
	IRIS_LOGI("%s: in shape %d", __func__, iris_emv_shaped);
	iris_emv_instant_send = false;
	iris_emv_current_mode = IRIS_EMV_OFF_FINAL;
	iris_emv_pwil_mv_capt_enable_i7(false, false);
	iris_emv_exit_i7();
	iris_emv_pwil_shortpacket_mask_i7(0);
	iris_emv_blending_flush_enable_i7(false, false, false);
	iris_emv_instant_send = true;
	iris_emv_current_mode = 0;
	iris_emv_shaped = 0;
	iris_extmv_frc_mode = 0;
	iris_emv_frc_mode_en_set_i7(false);
	IRIS_LOGI("%s: out", __func__);
}

void iris_emv_on_lightoff_i7(void)
{
	IRIS_LOGI("%s: in touched %d [%d %d %d, %d %d]", __func__,
		iris_emv_touched, iris_emv_lli,
		iris_emv_shaped, iris_emv_reset,
		iris_extmv_frc_mode, debug_emv_game_frc);

	iris_emv_efifo_restore_config_i7();
	if (iris_emv_efifo_config)
		iris_emv_efifo_config = false;

	if (iris_emv_touched && (iris_emv_lli > 0)) {
		iris_emv_pwil_restore_video_ctrl_i7(false);
		if ((iris_emv_shaped > 0) && (iris_emv_reset > 0))
			iris_emv_local_reset_i7();
	}
	iris_emv_touched = false;
	iris_emv_frc_mode_en_set_i7(false);
	last_graphicinfo_saved = false;
}

bool iris_dump_once_i7(u32 flag)
{
	if (iris_emv_dump_once & flag) {
		iris_emv_dump_once &= ~flag;
		return true;
	}
	return false;
}

bool iris_esd_sim_once_i7(void)
{
	if (iris_emv_esd_sim > 0) {
		iris_emv_esd_sim = 0;
		iris_emv_esd_reviving = true;
		return true;
	}
	return false;
}

bool iris_esd_reviving_i7(void)
{
	return iris_emv_esd_reviving;
}

void iris_esd_revived_i7(void)
{
	iris_emv_esd_reviving = false;
}

bool iris_esd_check_ignored_i7(void)
{
	return (iris_emv_esd_ignored > 0);
}

void onConfigureMvdMeta_i7(u32 count, u32 *values)
{
	struct extmv_frc_meta meta;

	if (count < 17 || values == NULL)
		return;

	IRIS_LOGE("i7:SET_MVD_META: %d: [%x][%d, 0x%04x%04x, %d, %d][%d]",
		count, values[0],
		values[1], values[2], values[3], values[4], values[5],
		values[16]);
	IRIS_LOGE("iris7: SET_MVD_META:[%d, 0x%04x%04x, %d, %d][%d, 0x%04x%04x, %d, %d]",
		values[6], values[7], values[8], values[9], values[10],
		values[11], values[12], values[13], values[14], values[15]);
	irisSetDisableDsppPq_i7((values[0] > 0) ? true : false);
	meta.mode = values[0];
	meta.gamePixelFormat = values[1];
	meta.gameWidthSrc = values[2];
	meta.gameHeightSrc = values[3];
	meta.gameLeftSrc = values[4];
	meta.gameTopSrc = values[5];
	meta.mvd0PixelFormat = values[6];
	meta.mvd0Width = values[7];
	meta.mvd0Height = values[8];
	meta.mvd0Left = values[9];
	meta.mvd0Top = values[10];
	meta.mvd1PixelFormat = values[11];
	meta.mvd1Width = values[12];
	meta.mvd1Height = values[13];
	meta.mvd1Left = values[14];
	meta.mvd1Top = values[15];
	meta.valid = values[16];
	if (count >= 19) {
		meta.bmvSize = values[17];
		meta.orientation = values[18];
		meta.gmvdPixelFormat = values[19];
		meta.gmvdWidthSrc = values[20];  /*H-resolution*/
		meta.gmvdHeightSrc = values[21]; /*V-resolution*/
		meta.gmvdLeft = values[22];   /*start position X*/
		meta.gmvdTop = values[23];    /*start position Y*/
		if (count > 24) {
			meta.containerWidth = values[24];
			meta.containerHeight = values[25];
		}
		IRIS_LOGE("i7:SET_MVD_META:bmvSize %d,ori:%d,gmvd: format %d,"
		    "algo 0x%04x%04x(%d,%d), container[0x%04x%04x]",
			values[17], values[18], values[19],
			values[20], values[21],
			values[22], values[23], values[24], values[25]);
	}
	irisSetExtMvFrc_i7(&meta);
}

int iris_dbgfs_emv_init_i7(struct dsi_display *display)
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

	debugfs_create_u32("emv_meta_en", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_meta_en);
	debugfs_create_u32("emv_bmv_size", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_bmv_size);
	debugfs_create_u32("emv_fixed_bmvsize", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_fixed_bmvsize);
	debugfs_create_u32("emv_path_bw", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_path_bw);
	debugfs_create_u32("emv_layout", 0600, pcfg->dbg_root,
			(u32 *)&iris_emv_layout);
	debugfs_create_u32("emv_landscape", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_game_landscape);
	debugfs_create_u32("emv_skip_duplicate", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_skip_duplicate);
	debugfs_create_u32("pq_disable", 0644, pcfg->dbg_root,
			(u32 *)&iris_pq_disable);
	debugfs_create_u32("emv_game_frc", 0644, pcfg->dbg_root,
			(u32 *)&debug_emv_game_frc);
	debugfs_create_u32("emv_formal", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_formal);
	debugfs_create_u32("emv_debug", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_debug);
	debugfs_create_u32("emv_pq", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_pq);
	debugfs_create_u32("emv_hugemot", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_hugemot);
	debugfs_create_u32("emv_low_latency", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_low_latency);
	debugfs_create_u32("emv_sr", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_sr);
	debugfs_create_u32("emv_two_buffers", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_twobuffers);
	debugfs_create_u32("emv_hold_dispmeta", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_hold_dispmeta);
	debugfs_create_u32("emv_capen_aftswap", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_capen_aftswap);
	debugfs_create_u32("emv_scaler_inpt", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_scaler_inpt);
	debugfs_create_u32("emv_scaler_infrc", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_scaler_infrc);
	debugfs_create_u32("emv_dump_once", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_dump_once);
	debugfs_create_u32("emv_esd_once", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_esd_sim);
	debugfs_create_u32("emv_esd_ignored", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_esd_ignored);
	debugfs_create_u32("emv_lli", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_lli);
	debugfs_create_bool("emv_send", 0600, pcfg->dbg_root,
			&iris_emv_instant_send);
	debugfs_create_u32("emv_shaped", 0600, pcfg->dbg_root,
			(u32 *)&iris_emv_shaped);
	debugfs_create_bool("emv_touched", 0600, pcfg->dbg_root,
			&iris_emv_touched);
	debugfs_create_u32("emv_rescue", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_rescue_enable);
	debugfs_create_u32("emv_health_failed_sim", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_healthfailed_sim);
	debugfs_create_u32("emv_capen_inpt", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_capen_inpt);
	debugfs_create_u32("emv_efifo_control", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_efifo_control);
	debugfs_create_u32("emv_osd_resize", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_osd_resize_control);
	debugfs_create_u32("emv_reset", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_reset);
	debugfs_create_u32("emv_mv_start", 0644, pcfg->dbg_root,
	(u32 *)&iris_emv_mv_addr_start);
	debugfs_create_u32("emv_limit", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_limit);
	debugfs_create_u32("emv_batch", 0644, pcfg->dbg_root,
			(u32 *)&iris_emv_batch);
	debugfs_create_u32("emv_new_dual", 0644, pcfg->dbg_root,
			(u32 *)&debug_emv_new_dual);

	return 0;
}
