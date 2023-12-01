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
#include "dsi_iris.h"

u8 fw_loaded_status = FIRMWARE_LOAD_FAIL;
u16 fw_calibrated_status;
u16 gamma_apl_status;

static DEFINE_MUTEX(fw_status_lock);

u8 iris_get_fw_load_status(void)
{
	u8 status = 0;

	mutex_lock(&fw_status_lock);
	status = fw_loaded_status;
	mutex_unlock(&fw_status_lock);

	return status;
}

void iris_update_fw_load_status(u8 value)
{
	mutex_lock(&fw_status_lock);
	fw_loaded_status = value;
	mutex_unlock(&fw_status_lock);
}

u16 iris_get_firmware_aplstatus_value(void)
{
	return gamma_apl_status;
}

int32_t iris_request_firmware(const struct firmware **fw,
		const uint8_t *name)
{
	int32_t rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct device *dev = &pcfg->display->pdev->dev;

	if (name == NULL) {
		IRIS_LOGE("%s(), firmware is null", __func__);
		return -EINVAL;
	}

	rc = request_firmware(fw, name, dev);
	if (rc) {
		IRIS_LOGE("%s(), failed to request firmware: %s, return: %d",
				__func__, name, rc);
		return rc;
	}

	IRIS_LOGI("%s(), request firmware [Success], name: %s, size: %zu bytes",
			__func__, name, (*fw)->size);

	return rc;
}

void iris_release_firmware(const struct firmware **fw)
{
	if (*fw) {
		release_firmware(*fw);
		*fw = NULL;
	}
}

int iris_change_lut_type_addr(
		struct iris_ip_opt *dest, struct iris_ip_opt *src)
{
	int rc = -EINVAL;
	struct dsi_cmd_desc *desc = NULL;

	if (!src || !dest) {
		IRIS_LOGE("%s(), src or dest is null", __func__);
		return rc;
	}

	desc = src->cmd;
	if (!desc) {
		IRIS_LOGE("%s(), invalid desc.", __func__);
		return rc;
	}

	IRIS_LOGD("%s(), desc len: %zu", __func__, desc->msg.tx_len);
	iris_change_type_addr(dest, src);

	return 0;
}

static int _iris_change_gamma_type_addr(void)
{
	int i = 0;
	int rc = -EINVAL;
	u8 ip = IRIS_IP_DPP;
	u8 opt_id = 0xFE;
	struct iris_ip_opt *lut_popt = NULL;
	struct iris_ip_opt *popt = NULL;
	uint8_t get_optid = 0;
	uint8_t lutip = 0;
	int32_t type = 0;
	struct iris_ip_index *pip_index = NULL;
	bool bApl = 0;
	uint8_t level = 0x0;
	IRIS_LOGD("%s(%d)", __func__, __LINE__);
	popt = iris_find_ip_opt(ip, opt_id);
	if (!popt) {
		IRIS_LOGE("%s(%d), can't find valid option for i_p: %#x, opt: %#x",
			__func__, __LINE__, ip, opt_id);
		return rc;
	}

	gamma_apl_status = 0;
	type = IRIS_LUT_PIP_IDX;
	lutip = GAMMA_LUT - LUT_IP_START;

	pip_index = iris_get_ip_idx(type) + lutip;

	for (i = 0; i < pip_index->opt_cnt; i++) {
		lut_popt = pip_index->opt + i;
		rc = iris_change_lut_type_addr(lut_popt, popt);
		get_optid = lut_popt->opt_id;
		level = get_optid & 0xf;
		bApl = (get_optid & 0x80) ? 1 : 0;
		if (bApl)
			gamma_apl_status |= (0x1 << level);

	}

	return rc;
}

int iris_send_lut_for_dma(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;

	/*register level*/
	if (rc)
		IRIS_LOGE("%s(%d), failed to change dbc type", __func__, __LINE__);
	rc = _iris_change_gamma_type_addr();
	if (rc)
		IRIS_LOGE("%s(%d), failed to change gamma type", __func__, __LINE__);

	if (pcfg->iris_chip_type == CHIP_IRIS7) {
		rc = iris_change_dpp_lutrgb_type_addr();
		if (rc)
			IRIS_LOGE("%s(%d), failed to change dpp pre_lutRGB type", __func__, __LINE__);
	}

	return rc;
}

void iris_parse_misc_info(void)
{
	uint8_t ip, opt;
	uint8_t i, j, v;
	char str[41];
	uint32_t *p = NULL;
	uint8_t *pc = NULL;
	uint32_t len = 0;
	uint8_t Change_Id[21];
	uint32_t pcs_ver;
	uint32_t date;
	uint8_t calibration_status;
	uint16_t panel_nits;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(),%d", __func__, __LINE__);
	/* iris7.fw: ip = MISC_INFO_LUT, opt = 0
	 * iris7_ccf1.fw: ip = MISC_INFO_LUT, opt = 1
	 * iris7_ccf2.fw: ip = MISC_INFO_LUT, opt = 2
	 */
	ip = MISC_INFO_LUT;
	opt = 0x1;

	if (iris_find_ip_opt(ip, opt) == NULL) {
		IRIS_LOGE("%s(%d), can not find misc info i_p-opt", __func__, __LINE__);
		return;
	}

	p = iris_get_ipopt_payload_data(ip, opt, 2);
	if (!p) {
		IRIS_LOGE("%s(%d), can not get misc info payload", __func__, __LINE__);
		return;
	}
	pc = (uint8_t *)p;

	len = iris_get_ipopt_payload_len(ip, opt, 2);
	if (len != 40) {
		IRIS_LOGE("%s(%d), invalid payload len %d", __func__, __LINE__, len);
		return;
	}

	for (i = 0; i < 8; i++)
		IRIS_LOGD("p[%d] = 0x%08x", i, p[i]);

	memcpy(Change_Id, pc, 21);
	pcs_ver = pc[21]<<24 | pc[22]<<16 | pc[23]<<8 | pc[24];
	date = pc[25]<<24 | pc[26]<<16 | pc[27]<<8 | pc[28];
	calibration_status = pc[29];
	panel_nits = pc[30]<<8 | pc[31];

	pcfg->panel_nits = panel_nits;

	str[0] = (char)Change_Id[0];
	for (i = 1; i < 21; i++) {
		for (j = 0; j < 2; j++) {
			if (j == 0)
				v = Change_Id[i]/16;
			else
				v = Change_Id[i]%16;
			if (v <= 9)
				str[(i-1)*2+j+1] = v + 48;
			else
				str[(i-1)*2+j+1] = v + 87;
		}
	}

	IRIS_LOGI("Change_Id: %s", str);
	IRIS_LOGI("pcs_ver = %08x", pcs_ver);
	IRIS_LOGI("date = %08x", date);
	IRIS_LOGI("calibration_status = %d", calibration_status);
	IRIS_LOGI("panel_nits = %04x", panel_nits);
}

int iris_parse_lut_cmds(uint32_t flag)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		return iris_parse_lut_cmds_i7(flag);
	else
		return iris_parse_lut_cmds_i7p(flag);
}

/*add lut cmds to bufs for sending*/
static void _iris_prepare_lut_cmds(struct iris_ip_opt *popt)
{
	int pos = 0;
	struct iris_cfg *pcfg = NULL;
	struct dsi_cmd_desc *pdesc = NULL;

	pcfg = iris_get_cfg();

	pdesc = pcfg->iris_cmds.iris_cmds_buf;
	pos = pcfg->iris_cmds.cmds_index;

	IRIS_LOGD("%s(), %p %p len: %d",
			__func__, &pdesc[pos], popt, popt->cmd_cnt);
	memcpy(&pdesc[pos], popt->cmd, sizeof(*pdesc) * popt->cmd_cnt);
	pos += popt->cmd_cnt;
	pcfg->iris_cmds.cmds_index = pos;
}

void iris_fomat_lut_cmds(u8 lut_type, u8 opt_id)
{
	struct iris_ip_opt *popt = NULL;

	popt = iris_find_ip_opt(lut_type, opt_id);
	if (!popt) {
		IRIS_LOGW("%s(%d), invalid opt id: %#x.",
				__func__, __LINE__, opt_id);
		return;
	}
	_iris_prepare_lut_cmds(popt);
}

int iris_send_lut(u8 lut_type, u8 lut_table_index)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		return iris_send_lut_i7(lut_type, lut_table_index);
	else
		return iris_send_lut_i7p(lut_type, lut_table_index);
}

void iris_update_gamma(void)
{
	if (iris_get_fw_load_status() == FIRMWARE_LOAD_FAIL)
		iris_scaler_gamma_enable(0);
	else {
		iris_scaler_gamma_enable(1);
		iris_update_fw_load_status(FIRMWARE_IN_USING);
	}
}

int iris_dbgfs_fw_calibrate_status_init(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u16("fw_calibrated_status", 0644, pcfg->dbg_root,
			&fw_calibrated_status);

	return 0;
}
