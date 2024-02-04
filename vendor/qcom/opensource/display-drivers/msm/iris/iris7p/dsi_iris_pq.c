// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <video/mipi_display.h>
#include <sde_encoder_phys.h>
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_lut.h"
#include "dsi_iris_ioctl.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_log.h"
#include "dsi_iris_memc.h"
#include "dsi_iris_reg_i7p.h"
#include "dsi_iris_dts_fw.h"

static u32 dwCSC2CoffValue[12];

static long nCSCCoffValue[18] = {
	0x000, 0x800, 0x000,
	0x000, 0x000, 0x800,
	0x800, 0x000, 0x000,
	0x000, 0x800, 0x000,
	0x000, 0x000, 0x800,
	0x800, 0x000, 0x000,
};

void iris_quality_setting_off_i7p(void)
{
	iris_setting.quality_cur.al_bl_ratio = 0;
	iris_setting.quality_cur.pq_setting.cmcolorgamut = 0;
	iris_cm_color_gamut_set_i7p(
			iris_setting.quality_cur.pq_setting.cmcolorgamut, true);
	iris_brightness_para_reset();
	iris_csc_para_reset();
	iris_csc2_para_reset();
	iris_dpp_precsc_enable(false, false);
}

void iris_end_dpp_i7p(bool bcommit)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_skip_dma) {
		if (!iris_dynamic_power_get()) {
			if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
				iris_dma_gen_ctrl(DMA_EVENT_CH13, DMA_EVENT_DTG_EVS_PRE,
						!bcommit);
			else
				iris_dma_gen_ctrl(DMA_EVENT_CH13, DMA_EVENT_DTG_TE, !bcommit);
		} else
			iris_init_update_ipopt_t(IRIS_IP_DPP, 0x80, 0x80, !bcommit);
		iris_update_pq_opt(iris_pq_update_path, bcommit);
	}
}

static void iris_end_pq(bool bcommit)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_skip_dma) {
		if (!iris_dynamic_power_get()) {
			if (pcfg->frc_enabled || pcfg->pwil_mode == FRC_MODE)
				iris_dma_gen_ctrl(DMA_EVENT_CH10, DMA_EVENT_DTG_EVS_PRE,
						!bcommit);
			else
				iris_dma_gen_ctrl(DMA_EVENT_CH10, DMA_EVENT_DTG_TE, !bcommit);
		} else
			iris_init_update_ipopt_t(IRIS_IP_DPORT, 0x80, 0x80, !bcommit);
		iris_update_pq_opt(iris_pq_update_path, bcommit);
	}
}

void iris_pq_parameter_init_i7p(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 index;

	/* no pxlw node */
	if (pcfg->valid <= PARAM_EMPTY) {
		IRIS_LOGW("no pxlw node");
		return;
	}

	iris_min_color_temp = pcfg->min_color_temp;
	iris_max_color_temp = pcfg->max_color_temp;

	index = (iris_min_color_temp-IRIS_CCT_MIN_VALUE)/IRIS_CCT_STEP;
	iris_min_x_value = iris_color_temp_x_get(index);

	index = (iris_max_color_temp-IRIS_CCT_MIN_VALUE)/IRIS_CCT_STEP;
	iris_max_x_value = iris_color_temp_x_get(index);

	pqlt_cur_setting->colortempvalue = 0;	// Use default color temperature from PCS.

	IRIS_LOGI("%s, iris_min_x_value=%d, iris_max_x_value = %d", __func__, iris_min_x_value, iris_max_x_value);
}

void iris_cm_ratio_set_i7p(void)
{
	u32 index;
	u32 index_default;
	u32 xvalue;
	u32 xvalue_default;
	u32 ratio;
	u32 value = 0;
	u32 value_default;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	struct iris_ip_opt *psopt;
	uint32_t *data = NULL;
	int i, k;
	//struct iris_cfg *pcfg = iris_get_cfg();
	uint16_t coefBuff_start = 0;
	uint32_t coefBuffIndex = pqlt_cur_setting->pq_setting.cmcolorgamut;
	uint32_t  *payload = NULL;

	//csc coef has 54 values + 27 precsc values + cct has 3 values.
	if (pqlt_cur_setting->pq_setting.cmcolorgamut >= 7)
		coefBuffIndex = 2;

	if (!iris_crstk_coef_buf) {
		IRIS_LOGE("iris_crstk_coef_buf is NULL");
		return;
	}
	coefBuff_start = coefBuffIndex * (CRSTK_COEF_SIZE/2 + CCT_VALUE_SIZE/2);
	iris_min_color_temp = iris_crstk_coef_buf[coefBuff_start+CRSTK_COEF_SIZE/2];
	value_default = iris_crstk_coef_buf[coefBuff_start+CRSTK_COEF_SIZE/2 + 1];
	iris_max_color_temp = iris_crstk_coef_buf[coefBuff_start+CRSTK_COEF_SIZE/2 + 2];

	if (iris_min_color_temp == 0)
		iris_min_color_temp = 2500;
	if (iris_max_color_temp == 0)
		iris_max_color_temp = 11000;
	if (value_default == 0)
		value_default = 6500;

	index = (iris_min_color_temp-IRIS_CCT_MIN_VALUE)/IRIS_CCT_STEP;
	iris_min_x_value = iris_color_temp_x_get(index);
	index = (iris_max_color_temp-IRIS_CCT_MIN_VALUE)/IRIS_CCT_STEP;
	iris_max_x_value = iris_color_temp_x_get(index);

	if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_MANUL) {
		if (pqlt_cur_setting->colortempvalue == 0)
			value = value_default;
		else
			value = pqlt_cur_setting->colortempvalue;
	} else if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_AUTO)
		value = pqlt_cur_setting->cctvalue;
	else
		value = value_default;

	if (value > iris_max_color_temp)
		value = iris_max_color_temp;
	else if (value < iris_min_color_temp)
		value = iris_min_color_temp;
	index = (value - IRIS_CCT_MIN_VALUE)/25;
	xvalue = iris_color_temp_x_get(index);

	if (value_default > iris_max_color_temp)
		value_default = iris_max_color_temp;
	else if (value_default < iris_min_color_temp)
		value_default = iris_min_color_temp;
	index_default = (value_default - IRIS_CCT_MIN_VALUE)/25;
	xvalue_default = iris_color_temp_x_get(index_default);

	IRIS_LOGD("cm color temperature default CCT=%d, xvalue_default = %d\n", value_default, xvalue_default);
	IRIS_LOGD("min_cct = %d, max_cct = %d\n", iris_min_color_temp, iris_max_color_temp);
	IRIS_LOGD("value = %d, index = %d, xvalue = %d\n", value, index, xvalue);
	IRIS_LOGD("cmcolorgamut=%d, coefBuff_start=%d, iris_max_x_value=%d, iris_min_x_value=%d\n",
		pqlt_cur_setting->pq_setting.cmcolorgamut, coefBuff_start, iris_max_x_value, iris_min_x_value);

	psopt = iris_find_ip_opt(IRIS_IP_DPP, 0x32);  //csc2 crstk coef
	if (!psopt) {
		IRIS_LOGE("can not find i_p=%x id=0x32", IRIS_IP_DPP);
		return;
	}
	data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
	if ((xvalue >= iris_max_x_value) && (xvalue < xvalue_default)) {
		ratio = ((xvalue - iris_max_x_value)*10000)/(xvalue_default - iris_max_x_value);

		IRIS_LOGD("ratio:%d, xvalue: %d, iris_max_x_value: %d", ratio, xvalue, iris_max_x_value);
		for (i = 0; i < 18; i++) {
			nCSCCoffValue[i] = (iris_crstk_coef_buf[coefBuff_start+18+i] * ratio +
						(10000 - ratio)*iris_crstk_coef_buf[coefBuff_start+i])/10000;
		}

	} else if ((xvalue <=  iris_min_x_value) && (xvalue >= xvalue_default)) {
		ratio = ((xvalue - xvalue_default)*10000)/(iris_min_x_value - xvalue_default);

		IRIS_LOGD("ratio:%d, xvalue: %d, iris_min_x_value: %d", ratio, xvalue, iris_min_x_value);
		for (i = 0; i < 18; i++) {
			nCSCCoffValue[i] = (iris_crstk_coef_buf[coefBuff_start+36+i]*ratio +
						iris_crstk_coef_buf[coefBuff_start+18+i]*(10000-ratio))/10000;
		}
	}

	data[2] = nCSCCoffValue[1] << 16 | nCSCCoffValue[0];
	data[3] = nCSCCoffValue[3] << 16 | nCSCCoffValue[2];
	data[4] = nCSCCoffValue[5] << 16 | nCSCCoffValue[4];
	data[5] = nCSCCoffValue[7] << 16 | nCSCCoffValue[6];
	data[6] = 0x0000 << 16 | nCSCCoffValue[8];
	data[7] = nCSCCoffValue[10] << 16 | nCSCCoffValue[9];
	data[8] = nCSCCoffValue[12] << 16 | nCSCCoffValue[11];
	data[9] = nCSCCoffValue[14] << 16 | nCSCCoffValue[13];
	data[10] = nCSCCoffValue[16] << 16 | nCSCCoffValue[15];
	data[11] = 0x0000 << 16 | nCSCCoffValue[17];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x32, 0x32, 0x01);

	psopt = iris_find_ip_opt(IRIS_IP_DPP, 0x33);  //csc2' coef
	if (!psopt) {
		IRIS_LOGE("could not find i_p=%d opt_id=0x33", IRIS_IP_DPP);
		return;
	}

	for (i = 0; i < 9; i++) {
		if (i/3 == 0)
			k = 1;
		else if (i/3 == 1)
			k = 5;
		else
			k = 6;
		if (dwCSC2CoffBuffer[i] > 0x4000) {
			dwCSC2CoffValue[i] = 0x8000 - dwCSC2CoffBuffer[i];
			dwCSC2CoffValue[i] = (dwCSC2CoffValue[i] * nCSCCoffValue[k]) / 0x800;
			dwCSC2CoffValue[i] = 0x8000 - dwCSC2CoffValue[i];
		} else {
			dwCSC2CoffValue[i] = (dwCSC2CoffBuffer[i] * nCSCCoffValue[k]) / 0x800;
		}
	}
	for (i = 9; i < 12; i++) {
		if (i == 9)
			k = 6;
		else if (i == 10)
			k = 1;
		else
			k = 5;
		if (dwCSC2CoffBuffer[i] > 0x100000) {
			dwCSC2CoffValue[i] = 0x200000 - dwCSC2CoffBuffer[i];
			dwCSC2CoffValue[i] = (dwCSC2CoffValue[i] * nCSCCoffValue[k]) / 0x800;
			dwCSC2CoffValue[i] = 0x200000 - dwCSC2CoffValue[i];
		} else
			dwCSC2CoffValue[i] = (dwCSC2CoffBuffer[i] * nCSCCoffValue[k]) / 0x800;
	}

	if (m_dpp_precsc_enable == false) {
		data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
		data[2] = dwCSC2CoffValue[1] << 16 | dwCSC2CoffValue[0];
		data[3] = dwCSC2CoffValue[3] << 16 | dwCSC2CoffValue[2];
		data[4] = dwCSC2CoffValue[5] << 16 | dwCSC2CoffValue[4];
		data[5] = dwCSC2CoffValue[7] << 16 | dwCSC2CoffValue[6];
		data[6] = 0x0000 << 16 | dwCSC2CoffValue[8];
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x33, 0x33, 0x01);

		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xc0, 2);
		if (!payload)
			return;
		for (i = 0; i < 3; i++)
			payload[i] = dwCSC2CoffValue[i + 9];
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0xc0, 0xc0, 0x01);
	} else {
		data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
		data[2] = nCSCCoffValue[1] << 16 | nCSCCoffValue[0];
		data[3] = nCSCCoffValue[3] << 16 | nCSCCoffValue[2];
		data[4] = nCSCCoffValue[5] << 16 | nCSCCoffValue[4];
		data[5] = nCSCCoffValue[7] << 16 | nCSCCoffValue[6];
		data[6] = 0x0000 << 16 | nCSCCoffValue[8];
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x33, 0x33, 0x01);

		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xc0, 2);
		if (!payload)
			return;
		for (i = 0; i < 3; i++)
			payload[i] = 0;
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0xc0, 0xc0, 0x01);
	}

	for (i = 0; i < 18; i++)
		IRIS_LOGD("nCSCCoffValue[%d] = 0x%04x", i, nCSCCoffValue[i]);

	IRIS_LOGD("cm color temperature value=%d", value);
}

void iris_cm_color_gamut_set_i7p(u32 level, bool bcommit)
{
	struct iris_update_regval regval;
	u32 gammalevel;
	uint32_t  *payload = NULL;
	uint32_t gammactrl = 0;
	uint32_t gammamode = 0;
	uint32_t currentmode;
	bool apl = 0;
	uint32_t lut3d_interp1, lut3d_interp2;
	uint32_t interp1_src = 0, interp1_src2;
	u16 aplstatus_value = iris_get_firmware_aplstatus_value();

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x50;

	if (level == 0) {  //3dlut bypass
		regval.mask = 0x00000023;
		regval.value = 0x0000020;
	} else {
		regval.mask = 0x00000023;
		//regval.value = 0x0 | ((level) << 10);
		regval.value = 0x0000003;
	}

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x50, 0x50, 0x01);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2);
	if (!payload)
		return;

	switch (level) {
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
		interp1_src = level;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 7://Vivid No CAM + Vivid High CAM
		interp1_src = 0x2; interp1_src2 = 0x3;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 8://Vivid high CAM x Native
		interp1_src = 0x3; interp1_src2 = 0x1;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 9://Vivid no CAM
		interp1_src = 0x2; interp1_src2 = 0x2;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	case 12://Vivid no CAM x Native
		interp1_src = 0x2; interp1_src2 = 0x1;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;

	case 13://Standard D65 sRGB x Native
		interp1_src = 0x4; interp1_src2 = 0x1;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;

	case 14://Standard D65 P3 x Native
		interp1_src = 0x5; interp1_src2 = 0x1;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src2 << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;

	default:
		interp1_src = 1;
		lut3d_interp1 = ((interp1_src << 10) | (interp1_src << 6) | 1);
		lut3d_interp1 = (payload[0] & 0xc0000000) | lut3d_interp1;
		lut3d_interp2 = 0;
		lut3d_interp2 = (payload[1] & 0xffff8000) | lut3d_interp2;
	break;
	}
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2, lut3d_interp1);
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x51, 3, lut3d_interp2);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x51, 0x51, 0x01);

	apl = (aplstatus_value & (0x1 << interp1_src)) ? 1 : 0;
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2);
	if (!payload)
		return;
	currentmode = payload[0] & 0x7;
	if (apl == 0) {
		gammactrl = payload[0] & 0xff0; //65bin gamma
		gammalevel = 0x00 + interp1_src;
	} else {
		gammamode = 2; //17bin gamma
		gammactrl = ((payload[0] & 0xff0) | gammamode | (0x1 << 3));
		//gammalevel = 0x20+level;
		gammalevel = 0xa0 + interp1_src;
	}

	if (level == 0) {
		gammactrl = payload[0] & 0xff0; //65bin gamma
		gammalevel = 0x00;
	}

	IRIS_LOGD("aplstauts: 0x%x, gammamctrl: %d, gammalevel: 0x%x", aplstatus_value, gammactrl, gammalevel);
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);
	iris_dpp_apl_enable(apl, 0x01);
	iris_update_ip_opt(GAMMA_LUT, gammalevel, 0x01);

	iris_cm_ratio_set_i7p();

	if (bcommit)
		iris_end_dpp_i7p(true);
	IRIS_LOGI("cm color gamut=%d", level);
}

void iris_lux_set_i7p(u32 level, bool update)
{
	uint32_t  *payload = NULL;

	level = level >> 1;
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x90, 10);
	if (!payload)
		return;
	payload[0] = (payload[0] & ~0xffff0000) | (level << 16);
	iris_update_ip_opt(IRIS_IP_DPP, 0x90, 1);
	if (update)
		iris_end_dpp_i7p(true);
	IRIS_LOGW("lux value =%d", level);
}

void iris_al_enable_i7p(bool enable)
{
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x90, 2);
	if (!payload)
		return;
	payload[0] &= ~0x00000001;
	if (enable == true)
		payload[0] |= 0x00000001;
	iris_update_ip_opt(IRIS_IP_DPP, 0x90, 1);
	iris_end_dpp_i7p(true);
	IRIS_LOGW("al enable =%d", enable);
}

void iris_pwil_dport_disable_i7p(bool enable, u32 value)
{
	u32 cmd[4];

	cmd[0] = IRIS_PWIL_ADDR + PWIL_DISP_CTRL1;
	cmd[1] = 0x00000000;
	cmd[1] |= (value << 27);
	if (enable)
		cmd[1] |= 0x04000000;
	cmd[2] = IRIS_PWIL_ADDR + PWIL_REG_UPDATE;
	cmd[3] = 0x00000100;

	iris_ocp_write_mult_vals(4, cmd);

	IRIS_LOGD("%s, pwil_dport_disable = %d, count = %d", __func__, enable, value);
}

int iris_update_backlight_i7p(u32 bl_lvl)
{
	int rc = 0;
	struct iris_cfg *pcfg = NULL;
	struct dsi_panel *panel;
	char led_pwm1[3] = {MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x0, 0x0};
	struct dsi_cmd_desc_pxlw backlight_cmd_pxlw = {
		{0, MIPI_DSI_DCS_LONG_WRITE, 0, 0, 0, sizeof(led_pwm1), led_pwm1, 0, NULL}, 1, 0
	};

	struct dsi_cmd_desc backlight_cmd;
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &backlight_cmd,
	};
	remap_to_qcom_style(&backlight_cmd, &backlight_cmd_pxlw, 1);

	pcfg = iris_get_cfg();
	panel = pcfg->panel;

	iris_setting.quality_cur.system_brightness = bl_lvl;

	if (panel->bl_config.bl_max_level > 255) {
		if (pcfg->switch_bl_endian) {
			led_pwm1[1] = (unsigned char)(bl_lvl >> 8);
			led_pwm1[2] = (unsigned char)(bl_lvl & 0xff);
		} else {
			led_pwm1[1] = (unsigned char)(bl_lvl & 0xff);
			led_pwm1[2] = (unsigned char)(bl_lvl >> 8);
		}
	} else {
		led_pwm1[1] = (unsigned char)(bl_lvl & 0xff);
		backlight_cmd.msg.tx_len = 2;
	}

	if (pcfg->abyp_ctrl.abypass_mode == PASS_THROUGH_MODE)
		rc = iris_pt_send_panel_cmd(panel, &cmdset);
	else
		rc = iris_abyp_send_panel_cmd(panel, &cmdset);
	return rc;
}

void iris_dom_set_i7p(int mode)
{
	uint32_t  *payload = NULL;
	uint32_t dport_ctrl0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->tx_mode == DSI_OP_VIDEO_MODE)
		return;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPORT, 0x01, 2);
	if (!payload)
		return;
	dport_ctrl0 = payload[0];
	dport_ctrl0 &= ~0xc000;
	dport_ctrl0 |= (mode & 0x3) << 14;
	iris_set_ipopt_payload_data(IRIS_IP_DPORT, 0x01, 2, dport_ctrl0);
	iris_init_update_ipopt_t(IRIS_IP_DPORT, 0x01, 0x01, 0x01);

	iris_end_pq(true);
}

void iris_csc2_para_set_i7p(uint32_t *values)
{
	uint32_t  *payload = NULL;
	int i, k;

	if (values == NULL) {
		IRIS_LOGE("brightness value is empty");
		return;
	}

	//pre csc
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x34, 2);
	if (!payload)
		return;
	for (i = 0; i < 5; i++)
		payload[i] = values[i];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x34, 0x34, 0x01);
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xd0, 2);
	if (!payload)
		return;
	for (i = 0; i < 3; i++)
		payload[i] = values[i + 5];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0xd0, 0xd0, 0x01);

	for (i = 0; i < 9; i++) {
		if (i%2 == 0)
			dwCSC2CoffBuffer[i] = values[i/2] & 0xffff;
		else
			dwCSC2CoffBuffer[i] = (values[i/2] & 0xffff0000) >> 16;
		if (i/3 == 0)
			k = 1;
		else if (i/3 == 1)
			k = 5;
		else
			k = 6;
		if (dwCSC2CoffBuffer[i] > 0x4000) {
			dwCSC2CoffValue[i] = 0x8000 - dwCSC2CoffBuffer[i];
			dwCSC2CoffValue[i] = (dwCSC2CoffValue[i] * nCSCCoffValue[k]) / 0x800;
			dwCSC2CoffValue[i] = 0x8000 - dwCSC2CoffValue[i];
		} else {
			dwCSC2CoffValue[i] = (dwCSC2CoffBuffer[i] * nCSCCoffValue[k]) / 0x800;
		}
	}
	for (i = 9; i < 12; i++) {
		dwCSC2CoffBuffer[i] = values[i-4];
		if (i == 9)
			k = 6;
		else if (i == 10)
			k = 1;
		else
			k = 5;
		if (dwCSC2CoffBuffer[i] > 0x100000) {
			dwCSC2CoffValue[i] = 0x200000 - dwCSC2CoffBuffer[i];
			dwCSC2CoffValue[i] = (dwCSC2CoffValue[i] * nCSCCoffValue[k]) / 0x800;
			dwCSC2CoffValue[i] = 0x200000 - dwCSC2CoffValue[i];
		} else
			dwCSC2CoffValue[i] = (dwCSC2CoffBuffer[i] * nCSCCoffValue[k]) / 0x800;
	}
	if (m_dpp_precsc_enable == false) {
		values[0] = dwCSC2CoffValue[1] << 16 | dwCSC2CoffValue[0];
		values[1] = dwCSC2CoffValue[3] << 16 | dwCSC2CoffValue[2];
		values[2] = dwCSC2CoffValue[5] << 16 | dwCSC2CoffValue[4];
		values[3] = dwCSC2CoffValue[7] << 16 | dwCSC2CoffValue[6];
		values[4] = 0x0000 << 16 | dwCSC2CoffValue[8];

		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x33, 2);
		if (!payload)
			return;
		for (i = 0; i < 5; i++)
			payload[i] = values[i];
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x33, 0x33, 0x01);
		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xc0, 2);
		if (!payload)
			return;
		for (i = 0; i < 3; i++)
			payload[i] = dwCSC2CoffValue[i + 9];
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0xc0, 0xc0, 0x01);
	}
	iris_end_dpp_i7p(true);
}

void iris_pwil_dpp_en_i7p(bool dpp_en)
{
	uint32_t  *payload = NULL;
	u32 cmd[4];

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x01, 4);
	if (!payload)
		return;
	if (dpp_en)
		payload[0] |= 0x10;
	else
		payload[0] &= ~0x10;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x01, 0x01, 0x01);

	if (iris_dynamic_power_get()) {
		cmd[0] = IRIS_PWIL_ADDR + DATA_PATH_CTRL0;
		cmd[1] = payload[0];
		cmd[2] = IRIS_PWIL_ADDR + PWIL_REG_UPDATE;
		cmd[3] = 0x00000100;

		iris_ocp_write_mult_vals(4, cmd);
	}
	iris_dma_trig(DMA_CH9, 0);
	iris_update_pq_opt(iris_pq_update_path, true);

	IRIS_LOGD("%s, dpp_en = %d", __func__, dpp_en);
}

void iris_scurve_enable_set(u32 level)
{
	u32 scurvelevel;
	u32 enable = 0;
	uint32_t  *payload = NULL;

	if (level > 0)
		enable = 1;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x70, 2);
	if (!payload)
		return;
	payload[0] &= ~0x00000001;
	payload[0] |= enable;
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x70, 0x70, 0x01);

	scurvelevel = 0xe0 + level;
	iris_init_update_ipopt_t(IRIS_IP_DPP, scurvelevel, scurvelevel, 0x01);

	iris_end_dpp_i7p(true);
	IRIS_LOGD("scurve level=%d", level);
}
