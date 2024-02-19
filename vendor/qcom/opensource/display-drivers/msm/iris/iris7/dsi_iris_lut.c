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

static u8 payload_size;
static uint32_t lut_lut2[LUT_LEN] = {};
static uint32_t LUT2_fw[LUT_LEN+LUT_LEN+LUT_LEN] = {};

static struct msmfb_iris_ambient_info iris_ambient_lut;
static struct msmfb_iris_maxcll_info iris_maxcll_lut;

static u8 *iris_ambient_lut_buf;
/* SDR2HDR_UVYGAIN_BLOCK_CNT > SDR2HDR_LUT2_BLOCK_CNT */
static struct dsi_cmd_desc *dynamic_lut_send_cmd;

static uint32_t lut_luty[LUT_LEN] = {};
static uint32_t lut_lutuv[LUT_LEN] = {};
static uint32_t LUTUVY_fw[LUT_LEN+LUT_LEN+LUT_LEN+LUT_LEN+LUT_LEN+LUT_LEN] = {};

static u8 *iris_maxcll_lut_buf;
static struct dsi_cmd_desc *dynamic_lutuvy_send_cmd;

struct lut_node {
	u32 lut_cmd_cnts_max;
	u32 hdr_lut2_pkt_cnt;
	u32 hdr_lutuvy_pkt_cnt;
};

static struct lut_node iris_lut_param;

static DEFINE_MUTEX(fw_attach_cmd_lock);
static DEFINE_SPINLOCK(fw_status_lock);

static void _iris_init_ambient_lut(void)
{
	iris_ambient_lut.ambient_lux = 0;
	iris_ambient_lut.ambient_bl_ratio = 0;
	iris_ambient_lut.lut_lut2_payload = &lut_lut2;

	if (iris_ambient_lut_buf != NULL) {
		vfree(iris_ambient_lut_buf);
		iris_ambient_lut_buf = NULL;
	}

	dynamic_lut_send_cmd = NULL;
}

static void _iris_init_maxcll_lut(void)
{
	iris_maxcll_lut.mMAXCLL = 2200;
	iris_maxcll_lut.lut_luty_payload = &lut_luty;
	iris_maxcll_lut.lut_lutuv_payload = &lut_lutuv;

	if (iris_maxcll_lut_buf != NULL) {
		vfree(iris_maxcll_lut_buf);
		iris_maxcll_lut_buf = NULL;
	}

	dynamic_lutuvy_send_cmd = NULL;
}

static void _iris_init_lut_buf(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	payload_size = pcfg->split_pkt_size;
	memset(&iris_lut_param, 0x00, sizeof(iris_lut_param));

	/* for HDR ambient light */
	_iris_init_ambient_lut();

	/* for HDR maxcll */
	_iris_init_maxcll_lut();

	iris_init_tm_points_lut();
}

int iris_change_dpp_lutrgb_type_addr(void)
{
	int i = 0;
	int rc = -EINVAL;
	u8 ip = IRIS_IP_DPP;
	u8 opt_id = 0xB0;
	struct iris_ip_opt *lut_popt = NULL;
	struct iris_ip_opt *popt = NULL;
	int32_t type = 0;
	uint8_t lutip = 0;
	uint8_t get_optid = 0;
	struct iris_ip_index *pip_index = NULL;

	IRIS_LOGD("%s(%d)", __func__, __LINE__);
	popt = iris_find_ip_opt(ip, opt_id);
	if (!popt) {
		IRIS_LOGE("%s(), cann't find valid option, input i_p: %#x, opt: %#x.",
				__func__, ip, opt_id);
		return rc;
	}

	type = IRIS_LUT_PIP_IDX;
	lutip = DPP_PRE_LUT - LUT_IP_START;

	pip_index = iris_get_ip_idx(type) + lutip;

	for (i = 0; i < pip_index->opt_cnt; i++) {
		lut_popt = pip_index->opt + i;
		get_optid = lut_popt->opt_id;
		if (get_optid != 0x10)
			rc = iris_change_lut_type_addr(lut_popt, popt);
	}

	return rc;
}

int iris_parse_lut_cmds_i7(uint32_t flag)
{
	int ret = 0;
	bool temp_flag = 0;
	struct iris_data data[] = { {NULL, 0}, {NULL, 0}, {NULL, 0}, {NULL, 0} };
	const struct firmware *fw = NULL;
	const struct firmware *ccf1_fw = NULL;
	const struct firmware *ccf2_fw = NULL;
	const struct firmware *ccf3_fw = NULL;
	const struct firmware *ccf4_fw = NULL;
	struct iris_ip_index *pip_index = NULL;
	u8 firmware_state = 0;
	u16 fw_ccf_status = 0;
	char ccf1_name[MAX_FW_NAME_LEN] = {};
	char ccf2_name[MAX_FW_NAME_LEN] = {};
	char ccf3_name[MAX_FW_NAME_LEN] = {};
	char ccf4_name[MAX_FW_NAME_LEN] = {};
	struct iris_cfg *pcfg = iris_get_cfg();

	pip_index = iris_get_ip_idx(IRIS_LUT_PIP_IDX);
	_iris_init_lut_buf();

	if (flag == LOAD_CALIBRATED_ONLY) {
		firmware_state |= (1<<0);
		goto load_calibrated;
	}

	// Load "iris7.fw".
	ret = iris_request_firmware(&fw, IRIS_FIRMWARE_NAME_I7);
	if (!ret) {
		firmware_state |= (1<<0);
		data[0].buf = fw->data;
		data[0].size = fw->size;
		IRIS_LOGI("%s(%d), request name: %s, size: %u.",
				__func__, __LINE__, IRIS_FIRMWARE_NAME_I7, data[0].size);
	} else {
		IRIS_LOGE("%s(), failed to load: %s", __func__, IRIS_FIRMWARE_NAME_I7);
		goto load_done;
	}

	if (flag == LOAD_CALIBRATED_OR_GOLDEN)
		goto load_calibrated;
	else if (flag == LOAD_GOLDEN_ONLY)
		goto load_golden;

load_calibrated:
	// Load "iris7_ccf1.fw".
	strlcpy(ccf1_name, IRIS_CCF1_CALIBRATED_FIRMWARE_NAME_I7, MAX_FW_NAME_LEN);
	ret = iris_request_firmware(&ccf1_fw, ccf1_name);
	if (!ret) {
		firmware_state |= (1<<1);
		data[1].buf = ccf1_fw->data;
		data[1].size = ccf1_fw->size;
		IRIS_LOGI("%s(%d), request calibrated fw, name: %s, size: %u.",
			__func__, __LINE__, ccf1_name, data[1].size);
	} else {
		if (flag == LOAD_CALIBRATED_OR_GOLDEN) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load golden", __func__, ccf1_name);
			goto load_golden;
		} else if (flag == LOAD_CALIBRATED_ONLY) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load done", __func__, ccf1_name);
			goto load_done;
		}
	}

	// Load "iris7_ccf2.fw".
	strlcpy(ccf2_name, IRIS_CCF2_CALIBRATED_FIRMWARE_NAME_I7, MAX_FW_NAME_LEN);
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

	// Load "iris7_ccf3.fw".
	strlcpy(ccf3_name, IRIS_CCF3_CALIBRATED_FIRMWARE_NAME_I7, MAX_FW_NAME_LEN);
	ret = iris_request_firmware(&ccf3_fw, ccf3_name);
	if (!ret) {
		//Temporary modify for lut load, will replace a determined value after usecase fixed
		const uint32_t ccf3_tail_size = 0;
		uint32_t ccf3_data_size = 0;
		uint32_t crstk_coef_group = 0;

		ccf3_data_size = ccf3_fw->size - ccf3_tail_size;
		crstk_coef_group = ccf3_data_size / (CRSTK_COEF_SIZE + CCT_VALUE_SIZE);
		temp_flag = (crstk_coef_group >= CRSTK_COEF_GROUP_I7) ? 1 : 0;
		if (temp_flag) {
			firmware_state |= (1<<3);
			temp_flag &= iris_crst_coef_check(ccf3_fw->data, ccf3_fw->size, crstk_coef_group);
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
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load golden", __func__, ccf3_name);
			goto load_golden;
		} else if (flag == LOAD_CALIBRATED_ONLY) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load done", __func__, ccf3_name);
			goto load_done;
		}
	}

	// Load "iris7_ccf4.fw".
	strlcpy(ccf4_name, IRIS_CCF4_CALIBRATED_FIRMWARE_NAME_I7, MAX_FW_NAME_LEN);
	ret = iris_request_firmware(&ccf4_fw, ccf4_name);
	if (ret) {
		// Load golden firmware.
		strlcpy(ccf4_name, IRIS_CCF4_FIRMWARE_NAME_I7, MAX_FW_NAME_LEN);
		ret = iris_request_firmware(&ccf4_fw, ccf4_name);
	}
	if (!ret) {
		firmware_state |= (1<<4);
		data[3].buf = ccf4_fw->data;
		data[3].size = ccf4_fw->size;
		IRIS_LOGI("%s(%d), request calibrated fw, name: %s, size: %u.",
			__func__, __LINE__, ccf4_name, data[3].size);
	} else {
		if (flag == LOAD_CALIBRATED_OR_GOLDEN) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load golden", __func__, ccf4_name);
			goto load_golden;
		} else if (flag == LOAD_CALIBRATED_ONLY) {
			IRIS_LOGE("%s(), failed to load calibrated FW: %s , goto load done", __func__, ccf4_name);
			goto load_done;
		}
	}

	if (firmware_state == 0x1f) {
		fw_ccf_status = LOAD_CALIBRATED_FW;
		goto load_done;
	}

load_golden:
	firmware_state = firmware_state & 0x1;

	// Load golden ccf1.fw firmware.
	if (pcfg->ccf1_name)
		strlcpy(ccf1_name, pcfg->ccf1_name, MAX_FW_NAME_LEN);
	else
		strlcpy(ccf1_name, IRIS_CCF1_FIRMWARE_NAME_I7, MAX_FW_NAME_LEN);
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
		strlcpy(ccf2_name, IRIS_CCF2_FIRMWARE_NAME_I7, MAX_FW_NAME_LEN);
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
		strlcpy(ccf3_name, IRIS_CCF3_FIRMWARE_NAME_I7, MAX_FW_NAME_LEN);
	iris_release_firmware(&ccf3_fw);
	ret = iris_request_firmware(&ccf3_fw, ccf3_name);
	if (!ret) {
		//Temporary modify for lut load, will replace a determined value after usecase fixed
		const uint32_t ccf3_tail_size = 0;
		uint32_t ccf3_data_size = 0;
		uint32_t crstk_coef_group = 0;

		ccf3_data_size = ccf3_fw->size - ccf3_tail_size;
		crstk_coef_group = ccf3_data_size / (CRSTK_COEF_SIZE + CCT_VALUE_SIZE);
		if (crstk_coef_group >= CRSTK_COEF_GROUP_I7) {
			firmware_state |= (1<<3);
			iris_crst_coef_check(ccf3_fw->data, ccf3_fw->size, crstk_coef_group);
			IRIS_LOGI("%s(%d), request golden FW, name: %s, size: %u.",
				__func__, __LINE__, ccf3_name, ccf3_data_size);
		} else {
			IRIS_LOGE("%s(), invalid format for firmware %s",
				__func__, ccf3_name);
		}
	} else {
		IRIS_LOGE("%s(), failed to load golden FW: %s", __func__, ccf3_name);
	}

	// Load golden ccf4.fw firmware.
	strlcpy(ccf4_name, IRIS_CCF4_FIRMWARE_NAME_I7, MAX_FW_NAME_LEN);
	iris_release_firmware(&ccf4_fw);
	ret = iris_request_firmware(&ccf4_fw, ccf4_name);
	if (!ret) {
		firmware_state |= (1<<4);
		data[3].buf = ccf4_fw->data;
		data[3].size = ccf4_fw->size;
		IRIS_LOGI("%s(%d), request golden FW, name: %s, size: %u.",
			__func__, __LINE__, ccf4_name, data[3].size);
	} else {
		IRIS_LOGE("%s(), failed to load golden FW: %s", __func__, ccf4_name);
	}
	fw_ccf_status = LOAD_GOLDEN_FW;

load_done:
	spin_lock(&fw_status_lock);
	fw_calibrated_status = fw_ccf_status;
	fw_loaded_status = (firmware_state == 0x1f ? FIRMWARE_LOAD_SUCCESS : FIRMWARE_LOAD_FAIL);
	spin_unlock(&fw_status_lock);
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
	iris_release_firmware(&ccf4_fw);

	return ret;
}

int iris_send_lut_i7(u8 lut_type, u8 lut_table_index)
{
	u8 lut_opt_id = 0xfe;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc *pdesc = pcfg->iris_cmds.iris_cmds_buf;

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
		IRIS_LOGD("%s(): call DPP_PRE_LUT, index: %#x.", __func__, lut_table_index);
		break;
	case SDR2HDR_LUT:
		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call SDR2HDR_LUT, index: %#x.", __func__, lut_table_index);
		break;

	case BLENDING_LUT:
		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call BLENDING_LUT, index: %#x.", __func__, lut_table_index);
		break;

	case DPP_DEMURA_LUT:
		lut_opt_id = lut_table_index & 0xff;
		iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call DPP_DEMURA_LUT, index: %#x.", __func__, lut_table_index);
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

	case AMBINET_HDR_GAIN:
		if (!iris_maxcll_lut_buf || !dynamic_lutuvy_send_cmd)
			break;

		memcpy(&pdesc[pcfg->iris_cmds.cmds_index], &dynamic_lutuvy_send_cmd[0],
				sizeof(struct dsi_cmd_desc)*iris_lut_param.hdr_lutuvy_pkt_cnt);

		pcfg->iris_cmds.cmds_index += iris_lut_param.hdr_lutuvy_pkt_cnt;
		IRIS_LOGI("%s(), ambinet hdr gain.", __func__);
		break;

	case AMBINET_SDR2HDR_LUT:
		if (!iris_ambient_lut_buf || !dynamic_lut_send_cmd)
			break;

		memcpy(&pdesc[pcfg->iris_cmds.cmds_index],
				&dynamic_lut_send_cmd[0],
				sizeof(struct dsi_cmd_desc)
				* iris_lut_param.hdr_lut2_pkt_cnt);

		pcfg->iris_cmds.cmds_index +=
			iris_lut_param.hdr_lut2_pkt_cnt;
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

	IRIS_LOGD("%s(), lut type: %#x, lut index: %#x, cmd count: %d, max count: %d",
			__func__,
			lut_type, lut_table_index,
			pcfg->iris_cmds.cmds_index, iris_lut_param.lut_cmd_cnts_max);

	return IRIS_SUCCESS;
}

void iris_update_ambient_lut(enum LUT_TYPE lutType, u32 lutpos)
{
	u32 len = 0;
	u32 hdr_payload_size = payload_size;
	u32 hdr_pkt_size = hdr_payload_size + DIRECT_BUS_HEADER_SIZE;
	u32 hdr_block_pkt_cnt =
		(SDR2HDR_LUT_BLOCK_SIZE/2 + hdr_payload_size - 1)
		/ hdr_payload_size;
	u32 iris_lut_buf_index, lut_block_index, lut_block_cnt;
	u32 lut_pkt_cmd_index;
	u32 temp_index, index_i;
	u32 dbus_addr_start;
	u32 lut_fw_index;
	u32 cmd_payload_len;
	struct ocp_header ocp_dbus_header;

	memset(&ocp_dbus_header, 0, sizeof(ocp_dbus_header));
	ocp_dbus_header.header = 0x0004000C;
	ocp_dbus_header.address = SDR2HDR_LUT2_ADDRESS;

	if (lutpos == 0xFFE00000)
		hdr_block_pkt_cnt =
			(SDR2HDR_LUT_BLOCK_SIZE + hdr_payload_size - 1)
			/ hdr_payload_size;

	if (lutType != AMBINET_SDR2HDR_LUT) {
		IRIS_LOGE("%s input lutType error %d", __func__, lutType);
		return;
	}

	if (lutpos == 0xFFE00000)
		dbus_addr_start = SDR2HDR_LUT2_ADDRESS;
	else
		dbus_addr_start = SDR2HDR_LUT2_ADDRESS + lutpos * SDR2HDR_LUT_BLOCK_SIZE / 2;
	lut_block_cnt = SDR2HDR_LUT2_BLOCK_CNT;

	// copy lut2 to the firmware format.
	//  lut2 is EVEN+ODD,
	//  LUT2_fw is  EVEN ODD EVEN ODD EVEN ODD
	for (index_i = 0; index_i < LUT_LEN; index_i++) {
		if (lutpos == 0xFFE00000) {
			lut_fw_index = index_i / 2;
			if (index_i % 2 != 0)  // ODD
				lut_fw_index += LUT_LEN / 2;
			LUT2_fw[lut_fw_index] = lut_lut2[index_i];
			LUT2_fw[lut_fw_index + LUT_LEN] = lut_lut2[index_i];
			LUT2_fw[lut_fw_index + LUT_LEN + LUT_LEN] = lut_lut2[index_i];
		} else {
			if (index_i % 2 == 0) {
				lut_fw_index = index_i / 4;
				if (index_i % 4 != 0) /* ODD */
					lut_fw_index += LUT_LEN / 4;
				LUT2_fw[lut_fw_index] = lut_lut2[index_i];
				LUT2_fw[lut_fw_index + LUT_LEN / 2] = lut_lut2[index_i];
				LUT2_fw[lut_fw_index + LUT_LEN / 2 + LUT_LEN / 2] =
					lut_lut2[index_i];
			}
		}
	}

	if (dynamic_lut_send_cmd == NULL) {
		len = sizeof(struct dsi_cmd_desc)
			* hdr_pkt_size * hdr_block_pkt_cnt
			* SDR2HDR_LUT2_BLOCK_NUMBER;
		dynamic_lut_send_cmd = vzalloc(len);
		if (dynamic_lut_send_cmd == NULL) {
			IRIS_LOGE("%s(), failed to alloc mem", __func__);
			return;
		}
		iris_lut_param.lut_cmd_cnts_max +=
			hdr_block_pkt_cnt * SDR2HDR_LUT2_BLOCK_NUMBER;
		iris_lut_param.hdr_lut2_pkt_cnt =
			hdr_block_pkt_cnt * SDR2HDR_LUT2_BLOCK_NUMBER;
	}

	if (iris_ambient_lut_buf)
		memset(iris_ambient_lut_buf, 0,
				hdr_pkt_size * iris_lut_param.hdr_lut2_pkt_cnt);

	if (!iris_ambient_lut_buf) {
		len = hdr_pkt_size * iris_lut_param.hdr_lut2_pkt_cnt;
		iris_ambient_lut_buf = vzalloc(len);
	}
	if (!iris_ambient_lut_buf) {
		vfree(dynamic_lut_send_cmd);
		dynamic_lut_send_cmd = NULL;
		return;
	}

	lut_fw_index = 0;
	/*parse LUT2*/
	for (lut_block_index = 0;
			lut_block_index < lut_block_cnt;
			lut_block_index++){

		ocp_dbus_header.address = dbus_addr_start
			+ lut_block_index
			* SDR2HDR_LUT_BLOCK_ADDRESS_INC;

		for (lut_pkt_cmd_index = 0;
				lut_pkt_cmd_index < hdr_block_pkt_cnt;
				lut_pkt_cmd_index++) {

			iris_lut_buf_index =
				lut_block_index * hdr_pkt_size
				* hdr_block_pkt_cnt
				+ lut_pkt_cmd_index * hdr_pkt_size;

			if (lut_pkt_cmd_index == hdr_block_pkt_cnt-1) {
				if (lutpos == 0xFFE00000)
					cmd_payload_len = SDR2HDR_LUT_BLOCK_SIZE
						- (hdr_block_pkt_cnt-1) * hdr_payload_size;
				else
					cmd_payload_len = SDR2HDR_LUT_BLOCK_SIZE/2
						- (hdr_block_pkt_cnt-1) * hdr_payload_size;
			} else
				cmd_payload_len = hdr_payload_size;

			temp_index = lut_pkt_cmd_index
				+ hdr_block_pkt_cnt * lut_block_index;
			dynamic_lut_send_cmd[temp_index].msg.type = 0x29;
			dynamic_lut_send_cmd[temp_index].msg.tx_len =
				cmd_payload_len + DIRECT_BUS_HEADER_SIZE;
			dynamic_lut_send_cmd[temp_index].post_wait_ms = 0;
			dynamic_lut_send_cmd[temp_index].msg.tx_buf =
				iris_ambient_lut_buf + iris_lut_buf_index;

			memcpy(&iris_ambient_lut_buf[iris_lut_buf_index],
					&ocp_dbus_header, DIRECT_BUS_HEADER_SIZE);
			iris_lut_buf_index += DIRECT_BUS_HEADER_SIZE;

			memcpy(&iris_ambient_lut_buf[iris_lut_buf_index],
					&LUT2_fw[lut_fw_index], cmd_payload_len);

			lut_fw_index += cmd_payload_len / 4;
			ocp_dbus_header.address += cmd_payload_len;
		}
	}
}

void iris_update_maxcll_lut(enum LUT_TYPE lutType, u32 lutpos)
{
	u32 hdr_payload_size = payload_size;
	u32 hdr_pkt_size = hdr_payload_size + DIRECT_BUS_HEADER_SIZE;
	u32 hdr_block_pkt_cnt = (SDR2HDR_LUT_BLOCK_SIZE / 2 + hdr_payload_size - 1) / hdr_payload_size;
	u32 iris_lut_buf_index, lut_block_index, lut_block_cnt, lut_pkt_cmd_index;
	u32 temp_index, index_i;
	u32 dbus_addr_start;
	u32 lut_fw_index;
	u32 cmd_payload_len;
	struct ocp_header ocp_dbus_header;

	memset(&ocp_dbus_header, 0, sizeof(ocp_dbus_header));
	ocp_dbus_header.header = 0x0004000C;
	ocp_dbus_header.address = SDR2HDR_LUTUVY_ADDRESS;

	if (lutpos == 0xFFFF0000)
		hdr_block_pkt_cnt = (SDR2HDR_LUT_BLOCK_SIZE + hdr_payload_size - 1) / hdr_payload_size;

	if (lutType != AMBINET_HDR_GAIN) {
		IRIS_LOGE("%s input lutType error %d", __func__, lutType);
		return;
	}

	dbus_addr_start = SDR2HDR_LUTUVY_ADDRESS;
	lut_block_cnt = SDR2HDR_LUTUVY_BLOCK_CNT;

	// copy lutuvy to the firmware format.
	// lutuvy is EVEN+ODD, LUT2_fw is  EVEN ODD EVEN ODD EVEN ODD
	for (index_i = 0; index_i < LUT_LEN; index_i++) {
		if (lutpos == 0xFFFF0000) {
			lut_fw_index = index_i / 2;
			if (index_i % 2 == 0) // ODD
				lut_fw_index += LUT_LEN / 2;
			LUTUVY_fw[lut_fw_index] = lut_lutuv[index_i];
			LUTUVY_fw[lut_fw_index + LUT_LEN] = lut_lutuv[index_i];
			LUTUVY_fw[lut_fw_index + 2 * LUT_LEN] = lut_lutuv[index_i];
			LUTUVY_fw[lut_fw_index + 3 * LUT_LEN] = lut_lutuv[index_i];
			LUTUVY_fw[lut_fw_index + 4 * LUT_LEN] = lut_luty[index_i];
			LUTUVY_fw[lut_fw_index + 5 * LUT_LEN] = lut_luty[index_i];
		} else {
			if (index_i % 2 == 0) {
				lut_fw_index = index_i / 4;
				if (index_i % 4 != 0) // ODD
					lut_fw_index += LUT_LEN / 4;
				LUTUVY_fw[lut_fw_index] = lut_lutuv[index_i];
				LUTUVY_fw[lut_fw_index + LUT_LEN / 2] = lut_lutuv[index_i];
				LUTUVY_fw[lut_fw_index + LUT_LEN] = lut_lutuv[index_i];
				LUTUVY_fw[lut_fw_index + 3 * LUT_LEN / 2] = lut_lutuv[index_i];
				LUTUVY_fw[lut_fw_index + 2 * LUT_LEN] = lut_luty[index_i];
				LUTUVY_fw[lut_fw_index + 5 * LUT_LEN / 2] = lut_luty[index_i];
			}
		}
	}

	if (dynamic_lutuvy_send_cmd == NULL) {
		dynamic_lutuvy_send_cmd = vzalloc(sizeof(struct dsi_cmd_desc)
					* hdr_pkt_size * hdr_block_pkt_cnt * SDR2HDR_LUTUVY_BLOCK_NUMBER);
		if (dynamic_lutuvy_send_cmd == NULL) {
			IRIS_LOGE("%s: failed to alloc mem", __func__);
			return;
		}
		iris_lut_param.lut_cmd_cnts_max += hdr_block_pkt_cnt * SDR2HDR_LUTUVY_BLOCK_NUMBER;
		iris_lut_param.hdr_lutuvy_pkt_cnt = hdr_block_pkt_cnt * SDR2HDR_LUTUVY_BLOCK_NUMBER;
	}

	if (iris_maxcll_lut_buf)
		memset(iris_maxcll_lut_buf, 0, hdr_pkt_size * iris_lut_param.hdr_lutuvy_pkt_cnt);

	if (!iris_maxcll_lut_buf)
		iris_maxcll_lut_buf = vzalloc(hdr_pkt_size * iris_lut_param.hdr_lutuvy_pkt_cnt);

	if (!iris_maxcll_lut_buf) {
		vfree(dynamic_lutuvy_send_cmd);
		dynamic_lutuvy_send_cmd = NULL;
		IRIS_LOGE("%s: failed to alloc mem", __func__);
		return;
	}

	lut_fw_index = 0;
	//parse LUTUVY
	for (lut_block_index = 0; lut_block_index < lut_block_cnt; lut_block_index++) {
		ocp_dbus_header.address = dbus_addr_start + lut_block_index*SDR2HDR_LUT_BLOCK_ADDRESS_INC;
		if (lutpos != 0xFFFF0000)
			ocp_dbus_header.address += lutpos * SDR2HDR_LUT_BLOCK_SIZE/2;
		for (lut_pkt_cmd_index = 0; lut_pkt_cmd_index < hdr_block_pkt_cnt; lut_pkt_cmd_index++) {
			iris_lut_buf_index = lut_block_index*hdr_pkt_size*hdr_block_pkt_cnt
						+ lut_pkt_cmd_index*hdr_pkt_size;
			if (lut_pkt_cmd_index == hdr_block_pkt_cnt-1) {
				cmd_payload_len = SDR2HDR_LUT_BLOCK_SIZE/2 - (hdr_block_pkt_cnt-1) * hdr_payload_size;
				if (lutpos == 0xFFFF0000)
					cmd_payload_len += SDR2HDR_LUT_BLOCK_SIZE/2;
			} else
				cmd_payload_len = hdr_payload_size;

			temp_index = lut_pkt_cmd_index + hdr_block_pkt_cnt * lut_block_index;
			dynamic_lutuvy_send_cmd[temp_index].msg.type = 0x29;
			dynamic_lutuvy_send_cmd[temp_index].msg.tx_len = cmd_payload_len + DIRECT_BUS_HEADER_SIZE;
			dynamic_lutuvy_send_cmd[temp_index].post_wait_ms = 0;
			dynamic_lutuvy_send_cmd[temp_index].msg.tx_buf = iris_maxcll_lut_buf + iris_lut_buf_index;

			memcpy(&iris_maxcll_lut_buf[iris_lut_buf_index], &ocp_dbus_header, DIRECT_BUS_HEADER_SIZE);
			iris_lut_buf_index += DIRECT_BUS_HEADER_SIZE;

			memcpy(&iris_maxcll_lut_buf[iris_lut_buf_index], &LUTUVY_fw[lut_fw_index], cmd_payload_len);
			lut_fw_index += cmd_payload_len/4;
			ocp_dbus_header.address += cmd_payload_len;
		}
	}
}
