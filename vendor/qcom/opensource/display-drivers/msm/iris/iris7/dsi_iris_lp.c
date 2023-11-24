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
#include "dsi_iris_reg_i7.h"



/* abyp debug option
 * bit[0]: 0 -- light up with PT, 1 -- light up with ABYP
 * bit[1]: 0 -- switch to ABYP after light up, 1 -- switch to PT after light up
 * bit[3]: 0 -- non force, 1 -- force abyp during panel switch
 * bit[6]: 0 -- default, 1 -- use dbp instead of abyp
 */


static bool _iris_hdr_power;


void iris_global_var_init_i7(void)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	pcfg->iris_dtg_addr = IRIS_DTG_ADDR;
	pcfg->dtg_ctrl = DTG_CTRL;
	pcfg->dtg_update = DTG_UPDATE;
	pcfg->ovs_dly = OVS_DLY;
}

/* power on & off HDR domain
 *   type: 0 -- power off HDR & HDR_COLOR
 *         1 -- power on HDR, power off HDR_COLOER
 *         2 -- power on HDR & HDR_COLOR
 */
int iris_pmu_hdr_set(bool on, bool chain)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;

	pcfg = iris_get_cfg();

	IRIS_LOGI("%s: on %d, cur pwr %d, chain %d", __func__, on, _iris_hdr_power, chain);

	if (_iris_hdr_power == on) {
		IRIS_LOGI("%s: same type %d", __func__, on);
		return 2;
	}

	regval.ip = IRIS_IP_DMA;
	regval.opt_id = 0xd0; //dpp ch10 power protection
	regval.mask = 0x3;
	regval.value = on ? 0x3 : 0x1;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	regval.ip = IRIS_IP_DMA;
	regval.opt_id = 0xd2; //pwil ch12 power protection
	regval.mask = 0x3;
	regval.value = on ? 0x3 : 0x1;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	_iris_hdr_power = on;

	iris_pmu_power_set(HDR_PWR, on, chain);

	return 0;
}

/* control dma channels trigger
   input: channels -- bit 0, ch0; bit 1, ch1; bit 2, ch2; bit 3, ch3
          source -- trigger source selection
          chain -- send command with chain or not
 */
void iris_dma_gen_ctrl_i7(int channels, int source, bool chain)
{
	int value = 0;
	uint32_t  *payload = NULL;
	int mask, dma_ctrl;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (source > 7) {
		IRIS_LOGE("%s, source %d is wrong!", __func__, source);
		return;
	}

	if(channels & 0x1)
		value |= (0x20 | source);
	if(channels & 0x2)
		value |= ((0x20 | source) << 7);
	if(channels & 0x4)
		value |= ((0x20 | source) << 14);
	if(channels & 0x8)
		value |= ((0x20 | source) << 21);

	payload = iris_get_ipopt_payload_data(IRIS_IP_SYS, pcfg->id_sys_dma_gen_ctrl, 4);
	if (!payload) {
		IRIS_LOGE("%s(), can not get pwil SYS_DMA_GEN_CTRL property in sys setting", __func__);
		return;
	}
	mask = 0x0cf9f3e7; //DMA_CH* ONE_TIME_TRIG_EN, SRC_SEL
	dma_ctrl = payload[0] & (~mask);
	dma_ctrl |= (value & mask);

	IRIS_LOGD("%s: channels 0x%x, source %d, chain %d, value 0x%x, dma_ctrl 0x%x",
		__func__, channels, source, chain, value, dma_ctrl);

	iris_set_ipopt_payload_data(IRIS_IP_SYS, pcfg->id_sys_dma_gen_ctrl, 4, dma_ctrl);
	iris_init_update_ipopt_t(IRIS_IP_SYS, pcfg->id_sys_dma_gen_ctrl, pcfg->id_sys_dma_gen_ctrl, chain);
}



void iris_disable_temp_sensor(void)
{
	if (iris_ioctl_i2c_write(0xf0010030, 1) < 0)
		IRIS_LOGE("i2c disable tempsensor failed, line:%d", __LINE__);
	if (iris_ioctl_i2c_write(0xf0000150, 3) < 0)
		IRIS_LOGE("i2c disable tempsensor failed, line:%d", __LINE__);
	if (iris_ioctl_i2c_write(0xf0000150, 0) < 0)
		IRIS_LOGE("i2c disable tempsensor failed, line:%d", __LINE__);
	if (iris_ioctl_i2c_write(0xf0010030, 0) < 0)
		IRIS_LOGE("i2c disable tempsensor failed, line:%d", __LINE__);
}

void iris_bulksram_power_domain_proc_i7(void)
{
	u32 i, value;
	int rc = 0;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	for (i = 0; i < 5; i++)
	{
		usleep_range(1000, 1010);
		rc = iris_ioctl_i2c_write(0xf0000068, 0x43);
		if (rc < 0)
		{
			IRIS_LOGE("i2c enable bulksram power failed, rc:%d", rc);
			continue;
		}
		rc = iris_ioctl_i2c_read(0xf0000068, &value);
		if ((rc < 0) || value != 0x43)
		{
			IRIS_LOGE("i2c read bulksram power failed, value:%d, rc:%d", value, rc);
			continue;
		}
		rc = iris_ioctl_i2c_write(0xf0000068, 0x03);
		if (rc < 0)
		{
			IRIS_LOGE("i2c disable bulksram power failed, rc:%d", rc);
			continue;
		}
		rc = iris_ioctl_i2c_read(0xf0000068, &value);
		if ((rc < 0) || value != 0x03)
		{
			IRIS_LOGE("i2c read bulksram power failed, value:%d, rc:%d", value, rc);
			continue;
		}
		break;
	}
	IRIS_LOGI("%s %d", __func__, i);
}



int iris_esd_check_i7(void)
{
	int rc = 1;
	unsigned int run_status = 0x00;
	struct iris_cfg *pcfg;
	u32 value = 0;

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

	if (pcfg->ocp_read_by_i2c && pcfg->iris_i2c_read) {
		if (pcfg->iris_i2c_read(0xf1a00204, &value) < 0) {
			IRIS_LOGE("%s(): iris i2c read fail", __func__);
			rc = -1;
			goto exit;
		}
		IRIS_LOGD("%s(), %d, value = 0x%08x", __func__, __LINE__, value);
		rbuf[0] = (value >> 8) & 0xff;
		rbuf[1] = (value >> 0) & 0xff;
		rc = 0;
	} else
		rc = iris_dsi_send_cmds(pcfg->panel, cmdset.cmds, cmdset.count,
							cmdset.state, pcfg->vc_ctrl.to_iris_vc_id);

	run_status = rbuf[1] & 0x7;
	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
		IRIS_LOGI("read iris esd value: 0x%02x 0x%02x. run_status:0x%x. rc:%d.",
					rbuf[0], rbuf[1], run_status, rc);
	}
	if (rc) {
		IRIS_LOGI("%s() read iris esd read err: %d", __func__, rc);
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

