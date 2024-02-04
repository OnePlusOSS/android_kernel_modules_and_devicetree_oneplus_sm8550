// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lut.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_log.h"

static DEFINE_MUTEX(fw_attach_cmd_lock);
static DEFINE_MUTEX(fw_status_lock);


int iris_parse_lut_cmds_i7p(uint32_t flag)
{
	int ret = 0;
	bool temp_flag = 0;
	struct iris_data data[] = { {NULL, 0}, {NULL, 0}, {NULL, 0}, {NULL, 0} };
	const struct firmware *fw = NULL;
	const struct firmware *ccf1_fw = NULL;
	const struct firmware *ccf2_fw = NULL;
	const struct firmware *ccf3_fw = NULL;
	struct iris_ip_index *pip_index = NULL;
	u8 firmware_state = 0;
	u16 fw_ccf_status = 0;
	char ccf1_name[MAX_FW_NAME_LEN] = {};
	char ccf2_name[MAX_FW_NAME_LEN] = {};
	char ccf3_name[MAX_FW_NAME_LEN] = {};
	struct iris_cfg *pcfg = iris_get_cfg();

	pip_index = iris_get_ip_idx(IRIS_LUT_PIP_IDX);

	if (flag == LOAD_CALIBRATED_ONLY) {
		firmware_state |= (1<<0);
		goto load_calibrated;
	}

	// Load "iris7.fw".
	ret = iris_request_firmware(&fw, IRIS_FIRMWARE_NAME_I7P);
	if (!ret) {
		firmware_state |= (1<<0);
		data[0].buf = fw->data;
		data[0].size = fw->size;
		IRIS_LOGI("%s(%d), request name: %s, size: %u.",
				__func__, __LINE__, IRIS_FIRMWARE_NAME_I7P, data[0].size);
	} else {
		IRIS_LOGE("%s(), failed to load: %s", __func__, IRIS_FIRMWARE_NAME_I7P);
		goto load_done;
	}

	if (flag == LOAD_CALIBRATED_OR_GOLDEN)
		goto load_calibrated;
	else if (flag == LOAD_GOLDEN_ONLY)
		goto load_golden;

load_calibrated:
	// Load "iris7p_ccf1.fw".
	strlcpy(ccf1_name, IRIS_CCF1_CALIBRATED_FIRMWARE_NAME_I7P, MAX_FW_NAME_LEN);
	ret = iris_request_firmware(&ccf1_fw, ccf1_name);
	if (!ret) {
		firmware_state |= (1<<1);
		data[1].buf = ccf1_fw->data;
		data[1].size = ccf1_fw->size;
		IRIS_LOGI("%s(%d), request calibrated fw, name: %s, size: %u.",
			__func__, __LINE__, ccf1_name, data[1].size);
	} else {
		if (flag == LOAD_CALIBRATED_OR_GOLDEN) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load gelden", __func__, ccf1_name);
			goto load_golden;
		} else if (flag == LOAD_CALIBRATED_ONLY) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load done", __func__, ccf1_name);
			goto load_done;
		}
	}

	// Load "iris7p_ccf2.fw".
	strlcpy(ccf2_name, IRIS_CCF2_CALIBRATED_FIRMWARE_NAME_I7P, MAX_FW_NAME_LEN);
	ret = iris_request_firmware(&ccf2_fw, ccf2_name);
	if (!ret) {
		firmware_state |= (1<<2);
		data[2].buf = ccf2_fw->data;
		data[2].size = ccf2_fw->size;
		IRIS_LOGI("%s(%d), request calibrated fw, name: %s, size: %u.",
			__func__, __LINE__, ccf2_name, data[2].size);
	} else {
		if (flag == LOAD_CALIBRATED_OR_GOLDEN) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load golden", __func__, ccf2_name);
			goto load_golden;
		} else if (flag == LOAD_CALIBRATED_ONLY) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load done", __func__, ccf2_name);
			goto load_done;
		}
	}

	// Load "iris7p_ccf3.fw".
	strlcpy(ccf3_name, IRIS_CCF3_CALIBRATED_FIRMWARE_NAME_I7P, MAX_FW_NAME_LEN);
	ret = iris_request_firmware(&ccf3_fw, ccf3_name);
	if (!ret) {
		//Temporary modify for lut load, will replace a determined value after usecase fixed
		const uint32_t ccf3_tail_size = 0;
		uint32_t ccf3_data_size = 0;

		temp_flag = ((ccf3_fw->size - ccf3_tail_size)
			>= ((CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * CRSTK_COEF_GROUP_I7P)) ? 1 : 0;
		if (temp_flag) {
			firmware_state |= (1<<3);
			ccf3_data_size = ccf3_fw->size;
			temp_flag &= iris_crst_coef_check(ccf3_fw->data, ccf3_fw->size, CRSTK_COEF_GROUP_I7P);
			IRIS_LOGI("%s(%d), request calibrated FW, name: %s, size: %u.",
				__func__, __LINE__, ccf3_name, ccf3_data_size);
		}
	}
	if (ret || !temp_flag) {
		if (!temp_flag) {
			IRIS_LOGE("%s(), invalid format for firmware %s",
				__func__, ccf3_name);
		}
		if (flag == LOAD_CALIBRATED_OR_GOLDEN) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load gelden", __func__, ccf3_name);
			goto load_golden;
		} else if (flag == LOAD_CALIBRATED_ONLY) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load done", __func__, ccf3_name);
			goto load_done;
		}
	}

	if (firmware_state == 0x0f) {
		fw_ccf_status = LOAD_CALIBRATED_FW;
		goto load_done;
	}

load_golden:
	firmware_state = firmware_state & 0x1;

	// Load golden ccf1.fw firmware.
	if (pcfg->ccf1_name)
		strlcpy(ccf1_name, pcfg->ccf1_name, MAX_FW_NAME_LEN);
	else
		strlcpy(ccf1_name, IRIS_CCF1_FIRMWARE_NAME_I7P, MAX_FW_NAME_LEN);
	iris_release_firmware(&ccf1_fw);
	ret = iris_request_firmware(&ccf1_fw, ccf1_name);
	if (!ret) {
		firmware_state |= (1<<1);
		data[1].buf = ccf1_fw->data;
		data[1].size = ccf1_fw->size;
		IRIS_LOGI("%s(%d), request golden FW, name: %s, size: %u.",
			__func__, __LINE__, ccf1_name, data[1].size);
	} else {
		IRIS_LOGE("%s(), failed to load golden FW: %s", __func__, ccf1_name);
	}

	// Load golden ccf2.fw firmware.
	if (pcfg->ccf2_name)
		strlcpy(ccf2_name, pcfg->ccf2_name, MAX_FW_NAME_LEN);
	else
		strlcpy(ccf2_name, IRIS_CCF2_FIRMWARE_NAME_I7P, MAX_FW_NAME_LEN);
	iris_release_firmware(&ccf2_fw);
	ret = iris_request_firmware(&ccf2_fw, ccf2_name);
	if (!ret) {
		firmware_state |= (1<<2);
		data[2].buf = ccf2_fw->data;
		data[2].size = ccf2_fw->size;
		IRIS_LOGI("%s(%d), request golden FW, name: %s, size: %u.",
			__func__, __LINE__, ccf2_name, data[2].size);
	} else {
		IRIS_LOGE("%s(), failed to load golden FW: %s", __func__, ccf2_name);
	}

	// Load golden ccf3.fw firmware.
	if (pcfg->ccf3_name)
		strlcpy(ccf3_name, pcfg->ccf3_name, MAX_FW_NAME_LEN);
	else
		strlcpy(ccf3_name, IRIS_CCF3_FIRMWARE_NAME_I7P, MAX_FW_NAME_LEN);
	iris_release_firmware(&ccf3_fw);
	ret = iris_request_firmware(&ccf3_fw, ccf3_name);
	if (!ret) {
		//Temporary modify for lut load, will replace a determined value after usecase fixed
		const uint32_t ccf3_tail_size = 0;
		uint32_t ccf3_data_size = 0;

		if ((ccf3_fw->size - ccf3_tail_size) >= ((CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * CRSTK_COEF_GROUP_I7P)) {
			firmware_state |= (1<<3);
			ccf3_data_size = ccf3_fw->size;
			iris_crst_coef_check(ccf3_fw->data, ccf3_fw->size, CRSTK_COEF_GROUP_I7P);
			IRIS_LOGI("%s(%d), request golden FW, name: %s, size: %u.",
				__func__, __LINE__, ccf3_name, ccf3_data_size);
		} else {
			IRIS_LOGE("%s(), invalid format for firmware %s",
				__func__, ccf3_name);
		}
	} else {
		IRIS_LOGE("%s(), failed to load golden FW: %s", __func__, ccf3_name);
	}

	fw_ccf_status = LOAD_GOLDEN_FW;

load_done:
	mutex_lock(&fw_status_lock);
	fw_calibrated_status = fw_ccf_status;
	fw_loaded_status = (firmware_state == 0x0f ? FIRMWARE_LOAD_SUCCESS : FIRMWARE_LOAD_FAIL);
	mutex_unlock(&fw_status_lock);
	IRIS_LOGI("%s(), load firmware: %s, state: %#x",
			__func__,
			fw_loaded_status == FIRMWARE_LOAD_SUCCESS ? "success" : "fail",
			firmware_state);
	if (fw_loaded_status == FIRMWARE_LOAD_SUCCESS) {
		mutex_lock(&fw_attach_cmd_lock);
		ret = iris_attach_cmd_to_ipidx(data, (sizeof(data))/(sizeof(data[0])),
				pip_index);
		if (ret)
			IRIS_LOGE("%s(), failed to load iris fw", __func__);

		iris_send_lut_for_dma();
		iris_parse_misc_info();
		mutex_unlock(&fw_attach_cmd_lock);
	}

	iris_release_firmware(&fw);
	iris_release_firmware(&ccf1_fw);
	iris_release_firmware(&ccf2_fw);
	iris_release_firmware(&ccf3_fw);

	return ret;
}

int iris_send_lut_i7p(u8 lut_type, u8 lut_table_index)
{
	u8 lut_opt_id = 0xfe;
	struct iris_cfg *pcfg = iris_get_cfg();

	switch (lut_type) {
	case DPP_3DLUT:
		if (lut_table_index >= 0x3f) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
					__func__, __LINE__, lut_table_index, lut_type);
			break;
		}

		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(): call DPP_3DLUT, index: %#x.", __func__, lut_table_index);
		break;

	case DPP_PRE_LUT:
		if (lut_table_index >= 0x3f) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
					__func__, __LINE__, lut_table_index, lut_type);
			break;
		}
		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(): call DPP_PRE_LUT, index: %#x.", __func__, lut_table_index);
		break;

	case DPP_DLV_LUT:
		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call DPP_DLV_LUT, index: %#x.", __func__, lut_table_index);
		break;

	case SR_LUT:
		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call SR_LUT, index: %#x.", __func__, lut_table_index);
	break;
	case IOINC1D_LUT:
	case IOINC1D_PP_LUT:
	case IOINC1D_LUT_SHARP:
	case IOINC1D_PP_LUT_SHARP:
	case IOINC1D_LUT_9TAP:
	case IOINC1D_LUT_SHARP_9TAP:
		if ((lut_table_index & 0x3f) >= SCALER1D_LUT_NUMBER) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
					__func__, __LINE__, lut_table_index, lut_type);
			break;
		}

		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGD("%s(), call SCALER1D, i_p: %#x, index: %d.", __func__,
				lut_type, lut_table_index);
		break;

	case GAMMA_LUT:
		if (lut_table_index > 0xaf) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
					__func__, __LINE__, lut_table_index, lut_type);
			break;
		}

		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGD("%s(), call GAMMA_LUT, index: %d.", __func__, lut_table_index);
		break;

	case FRC_PHASE_LUT:
		if (lut_table_index >= FRC_PHASE_TYPE_CNT) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
					__func__, __LINE__, lut_table_index, lut_type);
			break;
		}

		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call FRC_PHASE_LUT, index: %d.", __func__, lut_table_index);
		break;

	case APP_CODE_LUT:
		if (lut_table_index != 0) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
					__func__, __LINE__, lut_table_index, lut_type);
			break;
		}

		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call APP_CODE_LUT, index: %d.", __func__, lut_table_index);
		break;

	case DPP_DITHER_LUT:
		if (lut_table_index != 0) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
					__func__, __LINE__, lut_table_index, lut_type);
			break;
		}

		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call DPP_DITHER_LUT, index: %d.", __func__, lut_table_index);
		break;

	case DTG_PHASE_LUT:
		if (lut_table_index != 0) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
					__func__, __LINE__, lut_table_index, lut_type);
			break;
		}

		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call DTG_PHASE_LUT, index: %d.", __func__, lut_table_index);
		break;

	default:
		IRIS_LOGW("%s(), type of %d have no cmd.", __func__, lut_type);
		break;
	}

	IRIS_LOGD("%s(), lut type: %#x, lut index: %#x, cmd count: %d",
			__func__,
			lut_type, lut_table_index, pcfg->iris_cmds.cmds_index);

	return IRIS_SUCCESS;
}
