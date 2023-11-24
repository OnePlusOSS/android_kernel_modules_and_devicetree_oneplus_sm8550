// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <video/mipi_display.h>
#include <drm/drm_bridge.h>
#include <drm/drm_encoder.h>
#include "dsi_drm.h"
#include <sde_encoder.h>
#include <sde_encoder_phys.h>
#include <sde_trace.h>
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_gpio.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_log.h"
#include "dsi_iris_i3c.h"
#include "dsi_iris_dts_fw.h"
#include <linux/kobject.h>
#include "dsi_iris_reg_i7p.h"



void iris_global_var_init_i7p(void)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	pcfg->iris_dtg_addr = IRIS_DTG_ADDR;
	pcfg->dtg_ctrl = DTG_CTRL;
	pcfg->dtg_update = DTG_UPDATE;
	pcfg->ovs_dly = OVS_DLY;
}

/* control dma channels trigger
   input: channels -- bit 0, ch0; bit 1, ch1; bit 2, ch2; bit 3, ch3
          source -- trigger source selection
          chain -- send command with chain or not
 */
void _iris_dma_gen_ctrl(int channels, int source, bool chain, u8 opt)
{
	int mask, dma_ctrl;
	int value = 0;
	u32 *payload = NULL;

	if(channels & 0x1)
		value |= (0x20 | source);
	if(channels & 0x2)
		value |= ((0x20 | source) << 7);
	if(channels & 0x4)
		value |= ((0x20 | source) << 14);
	if(channels & 0x8)
		value |= ((0x20 | source) << 21);

	payload = iris_get_ipopt_payload_data(IRIS_IP_SYS, opt, 4);
	if (!payload) {
		IRIS_LOGE("%s(), can not get pwil SYS_DMA_GEN_CTRL property in sys setting", __func__);
		return;
	}
	mask = 0x0cf9f3e7; //DMA_CH* ONE_TIME_TRIG_EN, SRC_SEL
	dma_ctrl = payload[0] & (~mask);
	dma_ctrl |= (value & mask);

	IRIS_LOGD("%s: value 0x%x, dma_ctrl 0x%x",
		__func__, value, dma_ctrl);

	iris_set_ipopt_payload_data(IRIS_IP_SYS, opt, 4, dma_ctrl);
	iris_init_update_ipopt_t(IRIS_IP_SYS, opt, opt, chain);
}

void iris_dma_gen_ctrl_i7p(int channels, int source, bool chain)
{
	struct iris_cfg *pcfg;
	int next_channels = (channels >> 4) & 0x0F;

	pcfg = iris_get_cfg();

	if (source > 7) {
		IRIS_LOGE("%s, source %d is wrong!", __func__, source);
		return;
	}

	IRIS_LOGD("%s: channels 0x%x, source %d, chain %d",
		__func__, channels, source, chain);
	if (channels & 0x0F) {
		int wait = (!chain && next_channels) ? 1 : 0;

		_iris_dma_gen_ctrl((channels & 0xF), source, wait, pcfg->id_sys_dma_gen_ctrl);
	}

	if (next_channels)
		_iris_dma_gen_ctrl(next_channels, source, chain, ID_SYS_DMA_GEN_CTRL2);
}

void iris_bulksram_power_domain_proc_i7p(void)
{
	u32 i, value;
	int rc = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	for (i = 0; i < 5; i++) {
		usleep_range(1000, 1010);
		rc = pcfg->iris_i2c_write(0xf0000068, 0x13);
		if (rc < 0) {
			IRIS_LOGE("i2c enable bulksram power failed, rc:%d", rc);
			continue;
		}
		rc = pcfg->iris_i2c_read(0xf0000068, &value);
		if ((rc < 0) || value != 0x13) {
			IRIS_LOGE("i2c read bulksram power failed, value:%d, rc:%d", value, rc);
			continue;
		}
		rc = pcfg->iris_i2c_write(0xf0000068, 0x03);
		if (rc < 0) {
			IRIS_LOGE("i2c disable bulksram power failed, rc:%d", rc);
			continue;
		}
		rc = pcfg->iris_i2c_read(0xf0000068, &value);
		if ((rc < 0) || value != 0x03) {
			IRIS_LOGE("i2c read bulksram power failed, value:%d, rc:%d", value, rc);
			continue;
		}
		break;
	}
	IRIS_LOGI("%s %d", __func__, i);
}

int iris_esd_check_i7p(void)
{
	int rc = 1;
	unsigned int run_status = 0x00;
	struct iris_cfg *pcfg;

	char get_diag_result[1] = {0x0f};
	char rbuf[16] = {0};
	struct dsi_cmd_desc_pxlw cmds_pxlw = {
		{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0, 0,
			sizeof(get_diag_result), get_diag_result, 2, rbuf},
		1,
		0};
	struct dsi_cmd_desc cmds;
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &cmds,
	};

	remap_to_qcom_style(&cmds, &cmds_pxlw, 1);

	pcfg = iris_get_cfg();

	rc = iris_dsi_send_cmds(pcfg->panel, cmdset.cmds, cmdset.count,
							cmdset.state, pcfg->vc_ctrl.to_iris_vc_id);

	run_status = rbuf[1] & 0x7;
	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
		IRIS_LOGI("dsi read iris esd value: 0x%02x 0x%02x. run_status:0x%x. rc:%d.",
					rbuf[0], rbuf[1], run_status, rc);
	}
	if (rc) {
		IRIS_LOGI("%s dsi read iris esd err: %d", __func__, rc);
		rc = -1;
		goto exit;
	}

	if (iris_esd_ctrl_get() & 0x10) {
		run_status = 0xff;
		pcfg->lp_ctrl.esd_ctrl &= ~0x10;
		IRIS_LOGI("iris esd %s force trigger", __func__);
	}

	if (run_status != 0) {
		IRIS_LOGI("iris esd err status 0x%x. ctrl: %d", run_status, pcfg->lp_ctrl.esd_ctrl);
		iris_dump_status();
		rc = -2;
	} else {
		rc = 1;
	}

exit:
	if (rc < 0) {
		pcfg->lp_ctrl.esd_cnt_iris++;
		IRIS_LOGI("iris esd err cnt: %d. rc %d", pcfg->lp_ctrl.esd_cnt_iris, rc);
	}

	IRIS_LOGD("%s rc:%d", __func__, rc);

	return rc;
}

void iris_pwil_update_i7p(void)
{
	u32 cmd[6];
	u32 *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe3, 2);
	cmd[0] = IRIS_PWIL_ADDR + EMPTY_FRAME_TIMEOUT_CNT;
	cmd[1] = payload[0];

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xe5, 2);
	cmd[2] = IRIS_PWIL_ADDR + BUSY_DOMAIN_DLY;
	cmd[3] = payload[0];

	cmd[4] = IRIS_PWIL_ADDR + PWIL_REG_UPDATE;
	cmd[5] = 0x00000100;

	iris_ocp_write_mult_vals(6, cmd);
}

