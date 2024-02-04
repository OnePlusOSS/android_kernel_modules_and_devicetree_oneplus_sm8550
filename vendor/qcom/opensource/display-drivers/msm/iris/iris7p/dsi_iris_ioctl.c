// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include "dsi_iris_api.h"
#include "dsi_iris.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_lut.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_ioctl.h"
#include "dsi_iris_i3c.h"
#include "dsi_iris_loop_back.h"
#include "dsi_iris_log.h"
#include "dsi_iris_memc.h"
#include "dsi_iris_memc_helper.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_cmpt.h"


// for game station settings via i2c
static uint32_t CM_CNTL[14] = {
	0xf0560000, 0x8020e000,
	0xf0560000, 0x820e000,
	0xf0560008, 0x00000000,
	0xf056000c, 0x6e,
	0xf056000c, 0x5f,
	0xf0560110, 0x00000000,
	0xf0560140, 0x00000100
};

// 0: mipi, 1: i2c
static int adb_type;

static bool iris_special_config(u32 type)
{
	switch (type) {
	case IRIS_OSD_ENABLE:
	case IRIS_OSD_AUTOREFRESH:
	case IRIS_OSD_OVERFLOW_ST:
	case IRIS_DBG_KERNEL_LOG_LEVEL:
	case IRIS_DBG_TIMING_SWITCH_LEVEL:
	case USER_DEMO_WND:
	case IRIS_MEMC_LEVEL:
	case IRIS_WAIT_VSYNC:
	case IRIS_CHIP_VERSION:
	case IRIS_FW_UPDATE:
	case IRIS_DEBUG_SET:
	case IRIS_LOOP_BACK_MODE:
	case IRIS_FRC_PQ_LEVEL:
	case IRIS_SET_DPP_APL_ABS:
	case IRIS_SET_DPP_APL_RES:
	case IRIS_ENABLE_DPP_APL:
	case IRIS_CLEAR_FRC_MIF_INT:
		return true;
	default:
		break;
	}

	return false;
}

static bool _iris_is_valid_type(u32 display, u32 type)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return false;

	if (!iris_special_config(type)
			&& type != IRIS_ANALOG_BYPASS_MODE
			&& pcfg->abyp_ctrl.abypass_mode == ANALOG_BYPASS_MODE)
		return false;

	if (type != IRIS_DBG_KERNEL_LOG_LEVEL
			&& pcfg->chip_ver == IRIS3_CHIP_VERSION)
		return false;

	return true;
}

static int _iris_configure(u32 display, u32 type, u32 value)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	bool dpp_precsc_enable;
	bool gamut_update = false;
	u32 regval;

	switch (type) {
	case IRIS_CM_COLOR_TEMP_MODE:
		if ((value & 0x3) > 2)
			goto error;

		pqlt_cur_setting->pq_setting.cmcolortempmode = value & 0x3;
		iris_cm_colortemp_mode_set(pqlt_cur_setting->pq_setting.cmcolortempmode, true);
		break;
	case IRIS_CM_COLOR_GAMUT:
		//if (value > 15)
		//	goto error;
		dpp_precsc_enable = ((value & 0x80) == 0x80) ? 1 : 0;

		gamut_update = (pqlt_cur_setting->pq_setting.cmcolorgamut != (value & 0x7f)) ? 1 : 0;
		if (gamut_update == false)
			gamut_update = (dpp_precsc_enable != m_dpp_precsc_enable) ? 1 : 0;

		pqlt_cur_setting->pq_setting.cmcolorgamut = value & 0x7f;
		if (pqlt_cur_setting->pq_setting.cmcolorgamut == 0)
			dpp_precsc_enable = 0;

		iris_dpp_precsc_set(dpp_precsc_enable);
		if (pqlt_cur_setting->pq_setting.cmcolortempmode) {
			if (gamut_update) {
				iris_dpp_precsc_enable(dpp_precsc_enable, false);
				iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut, true);
			} else {
				iris_dpp_precsc_enable(dpp_precsc_enable, true);
			}
		}
		break;
	case IRIS_AL_ENABLE:
		pqlt_cur_setting->pq_setting.alenable = value & 0x01;
		iris_al_enable(pqlt_cur_setting->pq_setting.alenable);
		break;
	case IRIS_DEMO_MODE:
		pqlt_cur_setting->pq_setting.demomode = value & 0x3;
		break;
	case IRIS_DYNAMIC_POWER_CTRL:
		if (value & 0x01) {
			IRIS_LOGI(" [%s, %d] open psr_mif osd first address eco.", __func__, __LINE__);
			iris_dynamic_power_set(value & 0x01, false);
		} else {
			IRIS_LOGI(" [%s, %d] close psr_mif osd first address eco.", __func__, __LINE__);
			iris_dynamic_power_set(value & 0x01, false);
		}
		break;
	case IRIS_DBP_MODE:
		iris_dbp_switch(value & 0x1, 0);
		break;
	case IRIS_READING_MODE:
		pqlt_cur_setting->pq_setting.readingmode = value & 0x1;
		iris_reading_mode_set(pqlt_cur_setting->pq_setting.readingmode);
		break;
	case IRIS_COLOR_TEMP_VALUE:
		pqlt_cur_setting->colortempvalue = value;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_MANUL)
			iris_cm_color_temp_set();
		break;
	case IRIS_CCT_VALUE:
		pqlt_cur_setting->cctvalue = value;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_AUTO)
			iris_cm_color_temp_set();
		break;
	case IRIS_LUX_VALUE:
		pqlt_cur_setting->luxvalue = value;
		iris_lux_set(pqlt_cur_setting->luxvalue, true);
		break;
	case IRIS_HDR_MAXCLL:
		pqlt_cur_setting->maxcll = value;
		break;
	case IRIS_ANALOG_BYPASS_MODE:
		if (value == pcfg->abyp_ctrl.abypass_mode) {
			IRIS_LOGD("Same bypass mode");
			break;
		}
		IRIS_LOGI("%s(), switch Iris mode to: %u", __func__, value);
		if (value == 0x10 || value == 0x20) {
			iris_abyp_mode_set(value >> 4);
			break;
		}
		if (value == ANALOG_BYPASS_MODE) {
			if (pcfg->dtg_ctrl_pt != 0)
				pcfg->dtg_ctrl_pt = 0;
			iris_quality_setting_off();
		}
		if (iris_abyp_switch_proc(pcfg->display, value) != 0)
			goto error;
		break;
	case IRIS_LOOP_BACK_MODE:
		iris_set_loopback_flag_i7p(value);
		break;
	case IRIS_HDR_PREPARE:
		iris_set_skip_dma(true);
		break;
	case IRIS_HDR_COMPLETE:
		iris_set_skip_dma(false);
		break;
	case IRIS_FW_UPDATE:
		// Need do multi-thread protection.
		if (value < LOAD_METHOD_CNT) {
			/* before parsing firmware, free ip & opt buffer which alloc for LUT,
			 * if loading firmware failed before, need realloc seq space after
			 * updating firmware
			 */
			//u8 firmware_status = iris_get_fw_load_status();

			iris_free_ipopt_buf(IRIS_LUT_PIP_IDX);
			iris_parse_lut_cmds(value);
			//if (firmware_status == FIRMWARE_LOAD_FAIL) {
				iris_free_seq_space();
				iris_alloc_seq_space();
			//}
			if (iris_get_fw_load_status() == FIRMWARE_LOAD_SUCCESS) {
				if (pcfg->abyp_ctrl.abypass_mode == PASS_THROUGH_MODE) {
					iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut, true);
					//iris_scaler_gamma_enable(false, 1);
				}
				iris_update_fw_load_status(FIRMWARE_IN_USING);
			}
		}
		break;
	case IRIS_DBG_KERNEL_LOG_LEVEL:
		iris_set_loglevel(value & 0xf);
		iris_set_trace_en((value >> 4) & 0xf);
		break;
	case IRIS_DBG_TIMING_SWITCH_LEVEL:
		iris_set_tm_sw_loglevel(value);
		break;
	case IRIS_WAIT_VSYNC:
		return iris_wait_vsync();
	case IRIS_CHIP_VERSION:
		pcfg->chip_value[0] = value;
		break;
	case IRIS_SET_METADATA:
		if (pcfg->metadata != 0) {
			IRIS_LOGI("last meta not sent!");
			goto error;
		}
		pcfg->metadata = value;
		IRIS_LOGD("metadata: %x", pcfg->metadata);
		break;
	case IRIS_SET_METADATA_LOCK:	// lock by panel_lock
		if (pcfg->metadata != 0)
			IRIS_LOGI("last meta not sent!");
		pcfg->metadata = value;
		IRIS_LOGD("metadata: %x", pcfg->metadata);
		iris_set_metadata(false);
		break;
	case IRIS_PWIL_DPORT_DISABLE:
		pqlt_cur_setting->pwil_dport_disable = value;
		iris_pwil_dport_disable(value, 2);
		break;
	case IRIS_MIPI_RX_VALIDATE:
		iris_ocp_write_val(0xf1a00008, 0xffff3a9a);
		break;
	case IRIS_S_CURVE:
		pqlt_cur_setting->scurvelevel = value;
		iris_scurve_enable_set(pqlt_cur_setting->scurvelevel);
		break;
	case IRIS_SCALER_PP_FILTER_LEVEL:
		iris_scl_sr1d_filter(1, &value);
		break;
	case IRIS_PT_SR_SET:
		iris_scl_ptsr_config(1, &value);
		break;
	case IRIS_FRC_PQ_LEVEL:
		iris_memc_set_pq_level(1, &value);
		break;
	case IRIS_SCL_CONFIG:
		iris_scl_config(1, &value);
		break;
	case IRIS_SCL_MODEL:
		iris_scl_change_model(1, &value);
		break;
	case IRIS_MEMC_DSC_CONFIG:
		iris_memc_dsc_config(1, &value);
		break;
	case IRIS_SET_DPP_APL_ABS:
		pcfg->iris_i2c_read(0xF0040038, &regval);
		regval &= ~0x000003ff;
		value &= 0x000003ff;
		regval |= value;
		pcfg->iris_i2c_write(0xF0040038, regval);
		break;
	case IRIS_SET_DPP_APL_RES:
		pcfg->iris_i2c_write(0x00003008, value);
		break;
	case IRIS_ENABLE_DPP_APL:
		pcfg->iris_i2c_read(0xF0040038, &regval);
		if (value == 1)
			regval |= 0x80000000;
		else
			regval &= ~0x80000000;
		pcfg->iris_i2c_write(0xf0040038, regval);
		break;
	case IRIS_MEMC_CTRL:
	case IRIS_MEMC_OSD:
	case USER_DEMO_WND:
	case IRIS_N2M_ENABLE:
		iris_configure_memc(type, value);
		break;
	case IRIS_MODE_SET:
	case IRIS_VIDEO_FRAME_RATE_SET:
	case IRIS_OUT_FRAME_RATE_SET:
	case IRIS_OSD_ENABLE:
	case IRIS_FRC_LOW_LATENCY:
	case IRIS_PANEL_TE:
	case IRIS_AP_TE:
	case IRIS_MEMC_LEVEL:
	case IRIS_OSD_LAYER_EMPTY:
		IRIS_LOGI("unused in iris7p");
		break;
	default:
		goto error;
	}

	return 0;

error:
	return -EINVAL;

}

int iris_configure_i7p(u32 display, u32 type, u32 value)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;
	ktime_t ktime = 0;

	if (type == IRIS_OSD_AUTOREFRESH || type == IRIS_OSD_OVERFLOW_ST) {
		IRIS_LOGV("%s(), display: %u, type: 0x%04x(%u), value: %#x(%u), current Iris mode: %d",
				__func__,
				display, type, type, value, value, pcfg->abyp_ctrl.abypass_mode);
	} else {
		IRIS_LOGI("%s(), display: %u, type: 0x%04x(%u), value: %#x(%u), current Iris mode: %d",
				__func__,
				display, type, type, value, value, pcfg->abyp_ctrl.abypass_mode);
	}

	if (!_iris_is_valid_type(display, type))
		return -EPERM;

	switch (type) {
	case IRIS_DEMO_MODE:
	case IRIS_HDR_MAXCLL:
	case IRIS_DEBUG_CAP:
	case IRIS_VIDEO_FRAME_RATE_SET:
	case IRIS_OUT_FRAME_RATE_SET:
	case IRIS_OSD_AUTOREFRESH:
	case IRIS_DBG_KERNEL_LOG_LEVEL:
	case IRIS_DBG_TIMING_SWITCH_LEVEL:
	case IRIS_FRC_LOW_LATENCY:
	case IRIS_N2M_ENABLE:
	case IRIS_WAIT_VSYNC:
	case IRIS_MEMC_LEVEL:
	case USER_DEMO_WND:
	case IRIS_CHIP_VERSION:
	case IRIS_OSD_LAYER_EMPTY:
	case IRIS_SET_METADATA:
	case IRIS_LOOP_BACK_MODE:
		/* don't lock panel_lock */
		return _iris_configure(display, type, value);
	}

	if (IRIS_IF_LOGI())
		ktime = ktime_get();

	mutex_lock(&pcfg->panel->panel_lock);
	rc = _iris_configure(display, type, value);
	mutex_unlock(&pcfg->panel->panel_lock);

	if (IRIS_IF_LOGI())
		IRIS_LOGI("%s(), spend %u us for type 0x%04x(%u) value %#x(%u)",
				__func__,
				(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime),
				type, type, value, value);

	return rc;
}

static int _iris_configure_ex(u32 display, u32 type, u32 count, u32 *values)
{
	int ret = -1;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	//uint8_t i = 0;
	u32 TempValue = 0;
	bool is_phone;
	u32 dump_reg_cnt, dump_read_cnt, i, j;
	u32 *dump_reg_addr, dump_reg_val[10];
	#define DUMP_REG_BUF_SIZE 100
	char dump_buf[DUMP_REG_BUF_SIZE];
	int len = 0;

	if (!_iris_is_valid_type(display, type))
		return -EPERM;

	switch (type) {
	case IRIS_LUX_VALUE:
		break;
	case IRIS_HDR_MAXCLL:
		break;
	case IRIS_CCF1_UPDATE:
		/* Nothing to do for Iirs5*/
		break;
	case IRIS_CCF2_UPDATE:
		/* Nothing to do for Iirs5*/
		break;
	case IRIS_HUE_SAT_ADJ:
		IRIS_LOGD("cm csc value: csc0 = 0x%x, csc1 = 0x%x, csc2 = 0x%x, csc3 = 0x%x, csc4 = 0x%x",
				values[0], values[1], values[2], values[3], values[4]);
		IRIS_LOGD("game mode %d", values[5]);
		/*
		 *if (values[5] == 1) {
		 *	for (i = 0; i <= 4; i++) {
		 *		if (pcfg->iris_i2c_write) {
		 *			if (pcfg->iris_i2c_write(CM_CNTL[10] + i*4, values[i]) < 0)
		 *				IRIS_LOGE("i2c set reg fails, reg=0x%x, val=0x%x",
		 *					CM_CNTL[10] + i*4, values[i]);
		 *		} else {
		 *			IRIS_LOGE("Game Station is not connected");
		 *		}
		 *	}
		 *} else {
		 *	iris_cm_csc_level_set(IRIS_IP_CM, &values[0]);
		 *}
		 */
		iris_csc_para_set(&values[0]);
		break;
	case IRIS_CONTRAST_DIMMING:
		IRIS_LOGI("dpp csc value: csc0 = 0x%x, csc1 = 0x%x, csc2 = 0x%x, csc3 = 0x%x, csc4 = 0x%x",
				values[0], values[1], values[2], values[3], values[4]);
		//iris_cm_csc_level_set(IRIS_IP_DPP, &values[0]);
		break;
	case IRIS_COLOR_TEMP_VALUE:
		is_phone = (count > 1) ? (values[1] == 0) : true;
		pqlt_cur_setting->colortempvalue = values[0];

		if (is_phone) {
			if (count > 3) {
				pqlt_cur_setting->min_colortempvalue = values[2];
				pqlt_cur_setting->max_colortempvalue = values[3];
			} else {
				pqlt_cur_setting->min_colortempvalue = 0;
				pqlt_cur_setting->max_colortempvalue = 0;
			}
			if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_MANUL)
				iris_cm_color_temp_set();
		} else {
			TempValue = iris_cm_ratio_set_for_iic();
			IRIS_LOGD("set reg=0x%x, val=0x%x", CM_CNTL[4], TempValue);
			if (pcfg->iris_i2c_write) {
				if (pcfg->iris_i2c_write(CM_CNTL[4], TempValue) < 0)
					IRIS_LOGE("i2c set reg fails, reg=0x%x, val=0x%x", CM_CNTL[4], TempValue);
			} else {
				IRIS_LOGE("Game Station is not connected");
			}
		}
		break;
	case IRIS_DBG_TARGET_REGADDR_VALUE_SET:
		if (adb_type == 0) {
			iris_ocp_write_val(values[0], values[1]);
		} else if (adb_type == 1) {
			if (pcfg->iris_i2c_write) {
				if (pcfg->iris_i2c_write(values[0], values[1]) < 0)
					IRIS_LOGE("i2c set reg fails, reg=0x%x, val=0x%x", values[0], values[1]);
			} else {
				IRIS_LOGE("Game Station is not connected");
			}
		}
		break;
	case IRIS_DBG_TARGET_REGADDR_VALUE_SET2:
		if (adb_type == 0) {
			IRIS_LOGD("%s,%d: path select dsi", __func__, __LINE__);
			iris_ocp_write_vals(values[0], values[1], count-2, values+2, DSI_CMD_SET_STATE_HS);
		} else {
			IRIS_LOGD("%s,%d: path select i2c, header = %x", __func__, __LINE__, values[0]);
			if ((values[0] & 0x0f) == 0x0c) {
				IRIS_LOGD("%s,%d: direct bus write", __func__, __LINE__);
				iris_i2c_write(&values[1], count-2, I3C_DIRECT_WR_OP, (values[0] >> 24) & 0x0F);
			} else if ((values[0] & 0x0f) == 0x00) {
				IRIS_LOGD("%s,%d: ocp burst write", __func__, __LINE__);
				iris_i2c_write(&values[1], count-2, I3C_OCP_BURST_WR_OP, (values[0] >> 24) & 0x0F);
			} else if ((values[0] & 0x0f) == 0x05) {
				IRIS_LOGD("%s,%d: bit enable write", __func__, __LINE__);
				iris_i2c_bit_en_op(values[1], values[2], values[3]);
			}
		}
		break;
	case IRIS_DBG_TARGET_REG_DUMP:
		if (!pcfg->iris_i2c_read) {
			IRIS_LOGE("iris_i2c_read is null");
			break;
		}
		if (count < 2 || count > 10) {
			IRIS_LOGE("wrong prameter count in reg dump: %d", count);
			break;
		}
		dump_read_cnt = values[0];
		dump_reg_addr = &values[1];
		dump_reg_cnt = count - 1;
		for (i = 0; i < dump_read_cnt; i++) {
			for (j = 0; j < dump_reg_cnt; j++)
				pcfg->iris_i2c_read(dump_reg_addr[j], &dump_reg_val[j]);

			memset(dump_buf,  0, sizeof(dump_buf));
			len = 0;
			for (j = 0; j < dump_reg_cnt; j++)
				len += snprintf(dump_buf+len, DUMP_REG_BUF_SIZE,
					", addr = 0x%08x, value = 0x%08x",
					dump_reg_addr[j], dump_reg_val[j]);
			IRIS_LOGI("reg dump, count: %3d%s", i, dump_buf);
		}
		break;
	case IRIS_CM_COLOR_TEMP_MODE:
		// phone
		pqlt_cur_setting->pq_setting.cmcolortempmode = values[0] & 0x3;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode > 2)
			goto error;

		iris_cm_colortemp_mode_set(pqlt_cur_setting->pq_setting.cmcolortempmode, true);

		// game station
		if (pcfg->iris_i2c_write) {
			if (pcfg->iris_i2c_write(CM_CNTL[6], values[1] ? CM_CNTL[9] : CM_CNTL[7]) < 0)
				IRIS_LOGE("i2c set reg fails, reg=0x%x", CM_CNTL[6]);
			else if (pcfg->iris_i2c_write(CM_CNTL[12], CM_CNTL[13]) < 0)
				IRIS_LOGE("i2c set reg fails, reg=0x%x", CM_CNTL[12]);
		} else {
			IRIS_LOGE("Game Station is not connected");
		}
		break;
	case IRIS_CSC_MATRIX:
		/*
		 *if (count > 9) {
		 *	if (values[0] == 1)
		 *		iris_cm_csc_level_set(IRIS_IP_CM, &values[2]);
		 *	else if (values[0] == 2)
		 *		iris_cm_csc_level_set(IRIS_IP_DPP, &values[2]);
		 *	else
		 *		return -EPERM;
		 *} else
		 *	return -EPERM;
		 */
		break;
	case IRIS_DBG_SEND_PACKAGE:
		ret = iris_send_ipopt_cmds(values[0], values[1]);
		IRIS_LOGD("iris config sends package: i_p: %#x, opt: %#x, send: %d.",
				values[0], values[1], ret);
		break;
	case IRIS_DPP_3DLUT_GAIN:
		iris_dpp_3dlut_gain(count, &values[0], false);
		break;
	case IRIS_GAMMA_MODE:
		if (count > 2)
			goto error;
		if (count == 1)
			iris_dpp_gammamode_set(values[0], 0);
		else
			iris_dpp_gammamode_set(values[0], values[1]);
		break;
	case IRIS_BRIGHTNESS_CHIP:
		iris_brightness_level_set(&values[0]);
		break;
	case IRIS_DC_DIMMING:
		iris_brightness_level_set(&values[0]);
		break;
	case IRIS_DPP_CSC_SET:
		iris_csc2_para_set(&values[0]);
		break;
	case IRIS_WAIT_VSYNC:
		if (count > 2)
			iris_set_pending_panel_brightness(values[0], values[1], values[2]);
		break;
	case IRIS_SCALER_FILTER_LEVEL:
		iris_scl_ioinc_filter(count, values);
		break;
	case IRIS_SCALER_PP_FILTER_LEVEL:
		iris_scl_sr1d_filter(count, values);
		break;
	case IRIS_PT_SR_SET:
		iris_scl_ptsr_config(count, values);
		break;
	case IRIS_FRC_PQ_LEVEL:
		iris_memc_set_pq_level(count, values);
		break;
	case IRIS_SCL_CONFIG:
		iris_scl_config(count, values);
		break;
	case IRIS_SCL_MODEL:
		iris_scl_change_model(count, values);
		break;
	case IRIS_MEMC_DSC_CONFIG:
		iris_memc_dsc_config(count, values);
		break;
	case IRIS_MEMC_INFO_SET:
	case IRIS_DEBUG_SET:
		iris_configure_ex_memc(type, count, values);
		break;
	case IRIS_MEMC_OSD_PROTECT:
	case IRIS_SET_MVD_META:
		IRIS_LOGI("unused in iris7p");
		break;
	default:
		IRIS_LOGE("%s(), unrecognized type: 0x%04x(%d)", __func__, type, type);
		goto error;
	}

	return 0;

error:
	return -EINVAL;
}

int iris_configure_ex_i7p(u32 display, u32 type, u32 count, u32 *values)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;

	IRIS_LOGI("%s(), type: 0x%04x(%d), value: %#x(%d), count: %d, abyp mode: %d",
			__func__,
			type, type, values[0], values[0], count, pcfg->abyp_ctrl.abypass_mode);
	if (!_iris_is_valid_type(display, type))
		return -EPERM;

	switch (type) {
	case IRIS_WAIT_VSYNC:
		/* don't lock panel_lock */
		return _iris_configure_ex(display, type, count, values);
	default:
		break;
	}

	mutex_lock(&pcfg->panel->panel_lock);
	rc = _iris_configure_ex(display, type, count, values);
	mutex_unlock(&pcfg->panel->panel_lock);
	return rc;
}

int iris_configure_get_i7p(u32 display, u32 type, u32 count, u32 *values)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 reg_addr, reg_val;

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -EINVAL;

	switch (type) {
	case IRIS_CM_COLOR_TEMP_MODE:
		*values = pqlt_cur_setting->pq_setting.cmcolortempmode;
		break;
	case IRIS_CM_COLOR_GAMUT:
		*values = pqlt_cur_setting->pq_setting.cmcolorgamut;
		break;
	case IRIS_AL_ENABLE:
		*values = pqlt_cur_setting->pq_setting.alenable;
		break;
	case IRIS_DEMO_MODE:
		*values = pqlt_cur_setting->pq_setting.demomode;
		break;
	case IRIS_LUX_VALUE:
		*values = pqlt_cur_setting->luxvalue;
		break;
	case IRIS_READING_MODE:
		*values = pqlt_cur_setting->pq_setting.readingmode;
		break;
	case IRIS_DYNAMIC_POWER_CTRL:
		*values = iris_dynamic_power_get();
		break;
	case IRIS_HDR_MAXCLL:
		*values = pqlt_cur_setting->maxcll;
		break;
	case IRIS_DBP_MODE:
		*values = pcfg->lp_ctrl.dbp_mode;
		break;
	case IRIS_ANALOG_BYPASS_MODE:
		iris_abyp_mode_get(count, values);
		break;
	case IRIS_LOOP_BACK_MODE:
		*values = iris_loop_back_validate_i7p();
		break;
	case IRIS_CCT_VALUE:
		*values = pqlt_cur_setting->cctvalue;
		break;
	case IRIS_COLOR_TEMP_VALUE:
		*values = pqlt_cur_setting->colortempvalue;
		break;
	case IRIS_CHIP_VERSION:
		if (*values == 1)
			*values = pcfg->chip_value[1];
		else {
			*values = 0;
			*values = iris_get_chip_caps();
			if (*values == 0)
				return -EFAULT;
		}
		break;
	case IRIS_PANEL_TYPE:
		*values = pcfg->panel_type;
		break;
	case IRIS_PANEL_NITS:
		*values = pcfg->panel_nits;
		break;
	case IRIS_MCF_DATA:
		mutex_lock(&pcfg->panel->panel_lock);
		/* get MCF from panel */
		mutex_unlock(&pcfg->panel->panel_lock);
		break;
	case IRIS_DBG_TARGET_REGADDR_VALUE_GET:
		IRIS_LOGV("%s:%d, pcfg->abyp_ctrl.abypass_mode = %d",
				__func__, __LINE__,
				pcfg->abyp_ctrl.abypass_mode);
		if ((pcfg->abyp_ctrl.abypass_mode == ANALOG_BYPASS_MODE) && (adb_type == 0))
			return -ENOTCONN;

		if (adb_type == 0) {
			mutex_lock(&pcfg->panel->panel_lock);
			*values = iris_ocp_read(*values, DSI_CMD_SET_STATE_HS);
			mutex_unlock(&pcfg->panel->panel_lock);
		} else if (adb_type == 1) {
			reg_addr = *values;
			if (pcfg->iris_i2c_read) {
				if (pcfg->iris_i2c_read(reg_addr, &reg_val) < 0)
					IRIS_LOGE("i2c read reg fails, reg=0x%x", reg_addr);
				else
					*values = reg_val;
			} else {
				IRIS_LOGE("Game Station is not connected");
			}
		}
		break;
	case IRIS_DBG_KERNEL_LOG_LEVEL:
		*values = iris_get_loglevel() | (iris_get_trace_en() << 4);
		break;
	case IRIS_DBG_TIMING_SWITCH_LEVEL:
		*values = iris_get_tm_sw_loglevel();
		break;
	case IRIS_VIDEO_FRAME_RATE_SET:
		*values = 0;
		break;
	case IRIS_OUT_FRAME_RATE_SET:
		*values = 0;
		break;
	case IRIS_MIPI2RX_PWRST:
		*values = 0;
		break;
	case IRIS_DUAL2SINGLE_ST:
		*values = 0;
		break;
	case IRIS_WORK_MODE:
		*values = DSI_OP_CMD_MODE;
		if (pcfg->panel)
			*values = pcfg->panel->panel_mode;
		break;
	case IRIS_PANEL_TE:
		*values = 0;
		break;
	case IRIS_AP_TE:
		*values = pcfg->ap_te;
		IRIS_LOGI("get IRIS_AP_TE: %d", pcfg->ap_te);
		break;
	case IRIS_MODE_SET:
		*values = 0;
		break;
	case IRIS_N2M_ENABLE:
		*values = pcfg->n2m_enable;
		break;
	case IRIS_MEMC_LEVEL:
		*values = pcfg->memc_info.memc_level;
		break;
	case IRIS_MEMC_OSD:
		*values = 0;
		break;
	case IRIS_PARAM_VALID:
		*values = pcfg->valid;
		break;
	case IRIS_GET_METADATA:
		*values = pcfg->metadata;
		break;
	case IRIS_PWIL_DPORT_DISABLE:
		*values = pqlt_cur_setting->pwil_dport_disable;
		break;
	case IRIS_MIPI_RX_VALIDATE:
		mutex_lock(&pcfg->panel->panel_lock);
		reg_val = iris_ocp_read(0xf1a00004, DSI_CMD_SET_STATE_HS);
		mutex_unlock(&pcfg->panel->panel_lock);
		if (reg_val & 0x0ffff3cf)
			*values = 1;
		else
			*values = 0;
		break;
	case IRIS_S_CURVE:
		*values = pqlt_cur_setting->scurvelevel;
		break;
	case IRIS_KERNEL_STATUS_GET:
	case IRIS_DEBUG_GET:
		iris_configure_get_memc(type, count, values);
		break;
	case IRIS_FW_UPDATE:
		*values = fw_calibrated_status;
		break;
	case IRIS_PT_SR_SET:
		iris_scl_ptsr_get(count, values);
		break;
	case IRIS_FRC_PQ_LEVEL:
		iris_memc_get_pq_level(count, values);
		break;
	case IRIS_GET_DPP_MCU_RES:
		mutex_lock(&pcfg->panel->panel_lock);
		*values = 0;
		if (atomic_read(&pcfg->iris_esd_flag) == 0) {
			u32 i = 0;
			u32 check_status = 0;
			u32 apl_res = 0;

			IRIS_LOGI("%s(), apl_res of front 2 frames as follows:\n", __func__);

			for (i = 0; i < 2; i++) {
				if (pcfg->iris_i2c_read(0x00003200 + i*4, &apl_res) != 0) {
					IRIS_LOGE("%s(%d), i2c read dpp apl_res frame[%d] failed\n",
					__func__, __LINE__, i);
					*values = 107;  //i2c read fail
				}
				IRIS_LOGI("%s(), frame[%2d] = 0x%08x\n", __func__, i, apl_res);
			}

			if (pcfg->iris_i2c_read(0x00003100, &apl_res) != 0) {
				IRIS_LOGE("%s(%d), i2c read dpp apl_res reference failed\n",
					__func__, __LINE__);
				*values = 107;  //i2c read fail
			}

			IRIS_LOGI("%s(), apl_res reference: 0x%08x\n", __func__, apl_res);

			if (pcfg->iris_i2c_read(0x00003000, &check_status) != 0) {
				IRIS_LOGE("%s(%d), i2c read dpp apl_res status failed\n",
					__func__, __LINE__);
				*values = 107;  //i2c read fail
			} else {
				if (check_status == 1) {
					*values = 1;
					if (pcfg->iris_i2c_read(0x00003004, &apl_res) != 0)
						IRIS_LOGE("%s(%d), i2c read dpp apl_res buffer failed\n",
							__func__, __LINE__);

					IRIS_LOGI("%s(), apl_res buffer: 0x%08x\n", __func__, apl_res);
				}
			}
		}
		mutex_unlock(&pcfg->panel->panel_lock);
		break;
	case IRIS_GET_DPP_APL_RES:
		mutex_lock(&pcfg->panel->panel_lock);
		*values = 0;
		{
			u32 apl_res = 0;

			if (pcfg->iris_i2c_read(0xf13003bc, &apl_res) != 0) {
				*values = 107;  //i2c read fail
				IRIS_LOGE("%s(%d), i2c read dpp apl_res register failed\n",
					__func__, __LINE__);
			}

			IRIS_LOGI("%s(), current frame apl_res register: 0x%08x\n", __func__, apl_res);

		}
		mutex_unlock(&pcfg->panel->panel_lock);
		break;
	case IRIS_DUMP_APL_PER_FRAME:
		mutex_lock(&pcfg->panel->panel_lock);
		*values = 0;
		{
			u32 i = 0;
			u32 apl_res = 0;
			u32 frame_count = 0;

			IRIS_LOGI("%s(), dump apl of all frames as follows:\n", __func__);

			if (pcfg->iris_i2c_read(0x00003104, &frame_count) != 0) {
				IRIS_LOGE("%s(%d), i2c read dpp apl_res reference failed\n",
					__func__, __LINE__);
				*values = 107;  //i2c read fail
			}
			IRIS_LOGI("%s(), frame count is: %d\n", __func__, frame_count);
			if (frame_count > 4096)
				frame_count = 4096;

			for (i = 0; i < frame_count; i++) {
				if (pcfg->iris_i2c_read(0x00003200 + i*4, &apl_res) != 0) {
					IRIS_LOGE("%s(%d), i2c read dpp apl_res frame[%d] failed\n",
					__func__, __LINE__, i);
					*values = 107;  //i2c read fail
				}
				IRIS_LOGI("%s(), frame[%4d] = 0x%08x\n", __func__, i, apl_res);
			}
		}
		mutex_unlock(&pcfg->panel->panel_lock);
		break;
	default:
		return -EFAULT;
	}

	if (type == IRIS_OSD_AUTOREFRESH || type == IRIS_OSD_OVERFLOW_ST || type == IRIS_DBG_TARGET_REGADDR_VALUE_GET) {
		IRIS_LOGV("%s(), type: 0x%04x(%d), value: %d",
				__func__,
				type, type, *values);
	} else {
		IRIS_LOGI("%s(), type: 0x%04x(%d), value: %d",
				__func__,
				type, type, *values);
	}
	return 0;
}

static ssize_t iris_adb_type_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	tot = scnprintf(bp, sizeof(bp), "%d\n", adb_type);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static ssize_t iris_adb_type_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	adb_type = val;

	return count;
}

static const struct file_operations iris_adb_type_write_fops = {
	.open = simple_open,
	.write = iris_adb_type_write,
	.read = iris_adb_type_read,
};

int iris_dbgfs_adb_type_init_i7p(struct dsi_display *display)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (debugfs_create_file("adb_type", 0644, pcfg->dbg_root, display,
				&iris_adb_type_write_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}
