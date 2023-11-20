// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <drm/drm_mipi_dsi.h>
#include <video/mipi_display.h>
#include <dsi_drm.h>
#include <sde_encoder_phys.h>
#include <sde_trace.h>
#include "dsi_parser.h"
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_ioctl.h"
#include "dsi_iris_lut.h"
#include "dsi_iris_loop_back.h"
#include "dsi_iris_gpio.h"
#include "dsi_iris_emv.h"
#include "dsi_iris_dual.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_log.h"
#include "dsi_iris_memc.h"
#include "dsi_iris_i3c.h"
#include "dsi_iris_cmpt.h"
#include "dsi_iris_dts_fw.h"

#define IRIS_CHIP_VER_0   0
#define IRIS_CHIP_VER_1   1
#define IRIS_OCP_HEADER_ADDR_LEN  8

#define calc_space_left(x, y) (x - y%x)
#define NON_EMBEDDED_BUF_SIZE (512*1024)  //512k

#define to_dsi_display(x) container_of(x, struct dsi_display, host)

enum {
	DSI_CMD_ONE_LAST_FOR_MULT_IPOPT = 0,
};

enum iris_op_type {
	IRIS_LIGHTUP_OP = 0,
	IRIS_PQUPDATE_OP,
};

enum iris_send_mode {
	DSI_NON_EMBEDDED_MODE = 0,
	DSI_EMBEDDED_NO_MA_MODE,
	DSI_EMBEDDED_MA_MODE,
};

/*use to parse dtsi cmd list*/
struct iris_cmd_header {
	uint32_t dsi_type;  /* dsi command type 0x23 0x29*/
	uint32_t last_pkt; /*last in chain*/
	uint32_t wait_us; /*wait time*/
	uint32_t ip_type; /*ip type*/
	uint32_t opt_and_link; /*ip option and lp or hs*/
	uint32_t payload_len; /*payload len*/
};

struct iris_cmd_comp {
	int32_t link_state;
	int32_t cnt;
	struct dsi_cmd_desc *cmd;
	enum iris_op_type op_type;
	enum iris_send_mode send_mode;
};

static struct iris_cfg gcfg = {
	.ccf1_name = NULL,
	.ccf2_name = NULL,
	.ccf3_name = NULL,
};
static uint8_t g_cont_splash_type = IRIS_CONT_SPLASH_NONE;
uint8_t iris_pq_update_path = PATH_DSI;

static bool iris_driver_registered = false;
static bool iris_driver_unregistered = false;

int iris_dbgfs_status_init(struct dsi_display *display);
static int _iris_dbgfs_cont_splash_init(struct dsi_display *display);
static void _iris_send_cont_splash_pkt(uint32_t type);
static int _iris_update_pq_seq(struct iris_update_ipopt *popt, int len);
static void _iris_update_desc_last(struct dsi_cmd_desc *pcmd,
		int count, bool last_cmd);
static int _iris_set_pkt_last(struct dsi_cmd_desc *cmd, int32_t cmd_cnt, uint32_t add_last_flag);

static int _iris_get_vreg(void)
{
	int rc = 0;
	int i;
	struct regulator *vreg = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_panel *panel = pcfg->panel;

	for (i = 0; i < pcfg->iris_power_info.count; i++) {
		vreg = devm_regulator_get(panel->parent,
				pcfg->iris_power_info.vregs[i].vreg_name);
		rc = IS_ERR(vreg);
		if (rc) {
			IRIS_LOGE("failed to get %s regulator",
					pcfg->iris_power_info.vregs[i].vreg_name);
			goto error_put;
		}
		pcfg->iris_power_info.vregs[i].vreg = vreg;
	}

	return rc;
error_put:
	for (i = i - 1; i >= 0; i--) {
		devm_regulator_put(pcfg->iris_power_info.vregs[i].vreg);
		pcfg->iris_power_info.vregs[i].vreg = NULL;
	}
	return rc;
}

static int _iris_put_vreg(void)
{
	int rc = 0;
	int i;
	struct iris_cfg *pcfg = iris_get_cfg();

	for (i = pcfg->iris_power_info.count - 1; i >= 0; i--)
		devm_regulator_put(pcfg->iris_power_info.vregs[i].vreg);

	return rc;
}

void iris_init(struct dsi_display *display, struct dsi_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(), for dispaly: %s, panel: %s",
			__func__,
			display->display_type, panel->name);

	if (iris_virtual_display(display)) {
		pcfg->display2 = display;
		pcfg->panel2 = panel;
		return;
	}

	pcfg->display = display;
	pcfg->panel = panel;
	pcfg->iris_i2c_read = iris_ioctl_i2c_read;
	pcfg->iris_i2c_write = iris_ioctl_i2c_write;
	pcfg->iris_i2c_burst_write = iris_ioctl_i2c_burst_write;
	pcfg->aod = false;
	pcfg->fod = false;
	pcfg->fod_pending = false;
	pcfg->platform_type = 1; //need to use ASIC
	pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE; //default abyp

	pcfg->pt_sr_enable = false;
	pcfg->pt_sr_enable_restore = false;
	pcfg->n2m_ratio = 1;
	pcfg->dtg_ctrl_pt = 0;
	pcfg->ocp_read_by_i2c = 1;
	pcfg->iris_mipi1_power_st = false;
	pcfg->ap_mipi1_power_st = false;
	pcfg->iris_pwil_blend_st = false;
	pcfg->iris_mipi1_power_on_pending = false;
	pcfg->iris_pwil_mode_state = 2;

	pcfg->frc_pq_guided_level = 1;
	pcfg->frc_pq_dejaggy_level = 1;
	pcfg->frc_pq_peaking_level = 1;
	pcfg->frc_pq_DLTI_level = 1;

	pcfg->frcgame_pq_guided_level = 0;
	pcfg->frcgame_pq_dejaggy_level = 0;
	pcfg->frcgame_pq_peaking_level = 0;
	pcfg->frcgame_pq_DLTI_level = 0;

	pcfg->osd_label = 0;
	pcfg->frc_label = 0;
	pcfg->frc_demo_window = 0;
	//iris_frc_setting_init();
	iris_memc_vfr_update_work_init(pcfg);

	iris_memc_status_clear(true);
	iris_dual_status_clear(true);

	atomic_set(&pcfg->fod_cnt, 0);
	atomic_set(&pcfg->iris_esd_flag, 0);
	iris_init_timing_switch();
	iris_lp_init();

#ifdef IRIS_EXT_CLK // skip ext clk
	pcfg->ext_clk = devm_clk_get(&display->pdev->dev, "div_clk");
#endif

	if (!iris_virtual_display(display)) {
		iris_dbgfs_lp_init(display);
		iris_dbgfs_pq_init(display);
		_iris_dbgfs_cont_splash_init(display);
		iris_dbgfs_memc_init(display);
		iris_loop_back_init(display);
		iris_dbgfs_adb_type_init(display);
		iris_dbgfs_fw_calibrate_status_init();
		iris_dbgfs_emv_init(display);
		iris_dbgfs_status_init(display);
		iris_dbg_gpio_init();
	}
	_iris_get_vreg();
	mutex_init(&pcfg->gs_mutex);
	mutex_init(&pcfg->ioctl_mutex);
	iris_driver_register();
}

void iris_deinit(struct dsi_display *display)
{
	struct iris_cfg *pcfg = NULL;
	int i;

	pcfg = iris_get_cfg();

	if (!iris_is_chip_supported())
		return;

	if (iris_virtual_display(display))
		return;

#ifdef IRIS_EXT_CLK // skip ext clk
	if (pcfg->ext_clk) {
		devm_clk_put(&display->pdev->dev, pcfg->ext_clk);
		pcfg->ext_clk = NULL;
	}
#endif

	for (i = 0; i < iris_get_cmd_list_cnt(); i++)
		iris_free_ipopt_buf(i);
	iris_free_ipopt_buf(IRIS_LUT_PIP_IDX);

	if (pcfg->pq_update_cmd.update_ipopt_array) {
		kfree(pcfg->pq_update_cmd.update_ipopt_array);
		pcfg->pq_update_cmd.update_ipopt_array = NULL;
		pcfg->pq_update_cmd.array_index = 0;
	}

	iris_free_seq_space();

	_iris_put_vreg();
	iris_sysfs_status_deinit();
	iris_deinit_timing_switch();
	iris_driver_unregister();
}

void iris_control_pwr_regulator(bool on)
{
	int rc = 0;
	struct iris_cfg *pcfg = NULL;

	if (!iris_is_chip_supported())
		return;

	pcfg = iris_get_cfg();
	rc = dsi_pwr_enable_regulator(&pcfg->iris_power_info, on);
	if (rc)
		IRIS_LOGE("failed to power %s iris", on ? "on" : "off");
}

void iris_power_on(struct dsi_panel *panel)
{
	if (!iris_is_chip_supported())
		return;

	IRIS_LOGI("%s(), for [%s] %s, secondary: %s",
			__func__,
			panel->name, panel->type,
			panel->is_secondary ? "true" : "false");

	if (panel->is_secondary)
		return;

	iris_control_pwr_regulator(true);

	if (iris_vdd_valid()) {
		iris_enable_vdd();
	} else { // No need to control vdd and clk
		IRIS_LOGW("%s(), vdd does not valid, use pmic", __func__);
//		iris_control_pwr_regulator(true);
	}
#ifdef IRIS_EXT_CLK
	iris_clk_enable(panel);
#endif
	usleep_range(5000, 5000);
}

void iris_reset(void)
{
	if (!iris_is_chip_supported())
		return;

	IRIS_LOGI("%s()", __func__);

	iris_init_one_wired();
	iris_reset_chip();
	return;
}

void iris_power_off(struct dsi_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!iris_is_chip_supported())
		return;

	IRIS_LOGI("%s(), for [%s] %s, secondary: %s",
			__func__,
			panel->name, panel->type,
			panel->is_secondary ? "true" : "false");

	if (panel->is_secondary) {
		pcfg->ap_mipi1_power_st = false;
		IRIS_LOGI("ap_mipi1_power_st: %d", pcfg->ap_mipi1_power_st);
		return;
	}

	iris_reset_off();
	usleep_range(500, 510);
#ifdef IRIS_EXT_CLK
	iris_clk_disable(panel);
#endif
	if (iris_vdd_valid())
		iris_disable_vdd();
//	else
//		iris_control_pwr_regulator(false);
	msleep(45);
	iris_control_pwr_regulator(false);

}

bool iris_virtual_display(const struct dsi_display *display)
{
	if (display && display->panel && display->panel->is_secondary)
		return true;

	return false;
}

bool iris_is_virtual_encoder_phys(void *phys_enc)
{
	struct sde_encoder_phys *phys_encoder = phys_enc;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;

	if (phys_encoder == NULL)
		return false;

	if (phys_encoder->connector == NULL)
		return false;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return false;

	display = c_conn->display;
	if (display == NULL)
		return false;

	if (!iris_virtual_display(display))
		return false;

	return true;
}

struct iris_cfg *iris_get_cfg(void)
{
	return &gcfg;
}

uint8_t iris_get_cont_splash_type(void)
{
	return g_cont_splash_type;
}

static struct iris_ctrl_seq *_iris_get_ctrl_seq_addr(
		struct iris_ctrl_seq *base, uint8_t chip_id)
{
	struct iris_ctrl_seq *pseq = NULL;

	switch (chip_id) {
	case IRIS_CHIP_VER_0:
		pseq = base;
		break;
	case IRIS_CHIP_VER_1:
		pseq = base + 1;
		break;
	default:
		IRIS_LOGE("unknown chip id: %d", chip_id);
		break;
	}
	return pseq;
}

static bool _iris_is_valid_ip(uint32_t ip)
{
	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return true;

	if (ip < IRIS_IP_END)
		return true;

	return false;
}

static struct iris_ctrl_seq *_iris_get_ctrl_seq_common(
		struct iris_cfg *pcfg, int32_t type)
{
	struct iris_ctrl_seq *pseq = NULL;

	if (type == IRIS_CONT_SPLASH_NONE)
		pseq = _iris_get_ctrl_seq_addr(pcfg->ctrl_seq, pcfg->chip_id);
	else if (type == IRIS_CONT_SPLASH_LK)
		pseq = _iris_get_ctrl_seq_addr(pcfg->ctrl_seq_cs, pcfg->chip_id);

	return pseq;
}

static struct iris_ctrl_seq *_iris_get_ctrl_seq(struct iris_cfg *pcfg)
{
	return _iris_get_ctrl_seq_common(pcfg, IRIS_CONT_SPLASH_NONE);
}

static struct iris_ctrl_seq *_iris_get_ctrl_seq_cs(struct iris_cfg *pcfg)
{
	return _iris_get_ctrl_seq_common(pcfg, IRIS_CONT_SPLASH_LK);
}

static bool _iris_is_lut(uint8_t ip)
{
	return ip >= LUT_IP_START ? true : false;
}

static uint32_t _iris_get_ocp_type(const uint8_t *payload)
{
	uint32_t *pval = (uint32_t *)payload;

	return cpu_to_le32(pval[0]);
}

static uint32_t _iris_get_ocp_base_addr(const uint8_t *payload)
{
	uint32_t *pval = (uint32_t *)payload;

	return cpu_to_le32(pval[1]);
}

static void _iris_set_ocp_type(const uint8_t *payload, uint32_t val)
{
	uint32_t *pval = (uint32_t *)payload;

	IRIS_LOGV("%s(), change ocp type from %#x to %#x.", __func__, pval[0], val);
	pval[0] = val;
}

static void _iris_set_ocp_base_addr(const uint8_t *payload, uint32_t val)
{
	uint32_t *pval = (uint32_t *)payload;

	IRIS_LOGV("%s(), change addr from %#x to %#x.", __func__, pval[1], val);
	pval[1] = val;
}

static void _iris_set_ocp_first_val(const uint8_t *payload, uint32_t val)
{
	uint32_t *pval  = (uint32_t *)payload;

	pval[2] = val;
}

static bool _iris_is_direct_bus(const uint8_t *payload)
{
	uint8_t val = _iris_get_ocp_type(payload) & 0x0f;

	//the last 4bit will show the ocp type
	if (val == 0x00 || val == 0x0c)
		return true;

	return false;
}

static int _iris_split_mult_pkt(const uint8_t *payload, int payload_size)
{
	uint32_t pkt_size = 0;
	int pkt_cnt = 1;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!_iris_is_direct_bus(payload))
		return pkt_cnt;

	pkt_size = pcfg->split_pkt_size;
	if (payload_size > pkt_size + IRIS_OCP_HEADER_ADDR_LEN)
		pkt_cnt =  (payload_size - IRIS_OCP_HEADER_ADDR_LEN + pkt_size - 1) / pkt_size;

	return pkt_cnt;
}

static void _iris_set_cont_splash_type(uint8_t type)
{
	g_cont_splash_type = type;
}

static bool _iris_is_valid_cmd_type(int32_t type)
{
	if ((type >= IRIS_DTSI_PIP_IDX_START && type < iris_get_cmd_list_cnt()) ||
			(type == IRIS_LUT_PIP_IDX))
		return true;

	return false;
}

static bool _iris_is_valid_dtsi_cmd_type(int32_t type)
{
	if (type >= IRIS_DTSI_PIP_IDX_START && type < iris_get_cmd_list_cnt())
		return true;

	return false;
}

struct iris_ip_index *iris_get_ip_idx(int32_t type)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!_iris_is_valid_cmd_type(type)) {
		IRIS_LOGE("%s, can not get pip idx for type: %u", __func__, type);
		return NULL;
	}

	return pcfg->ip_index_arr[type];
}

static int32_t _iris_get_ip_idx_type(const struct iris_ip_index *pip_index)
{
	int32_t type = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	for (type = IRIS_DTSI_PIP_IDX_START; type < iris_get_cmd_list_cnt(); type++) {
		if (pip_index == pcfg->ip_index_arr[type])
			return type;
	}
	if (pip_index == pcfg->ip_index_arr[IRIS_LUT_PIP_IDX])
		return IRIS_LUT_PIP_IDX;

	return -EINVAL;
}

static void _iris_init_ip_index(struct iris_ip_index *pip_index)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t cnt = 0;
	int32_t ip_cnt = IRIS_IP_CNT;

	if (_iris_get_ip_idx_type(pip_index) == IRIS_LUT_PIP_IDX)
		ip_cnt = LUT_IP_END - LUT_IP_START;

	for (i = 0; i < ip_cnt; i++) {
		cnt = pip_index[i].opt_cnt;
		for (j = 0; j < cnt; j++) {
			pip_index[i].opt[j].cmd = NULL;
			pip_index[i].opt[j].link_state = 0xff;
		}
	}
}

static int32_t _iris_alloc_pip_buf(struct iris_ip_index *pip_index)
{
	int i = 0;
	int j = 0;
	int opt_cnt = 0;
	int ip_cnt = IRIS_IP_CNT;

	if (_iris_get_ip_idx_type(pip_index) == IRIS_LUT_PIP_IDX)
		ip_cnt = LUT_IP_END - LUT_IP_START;

	for (i = 0; i < ip_cnt; i++) {
		opt_cnt = pip_index[i].opt_cnt;
		if (opt_cnt != 0) {
			pip_index[i].opt =
				kvzalloc(opt_cnt * sizeof(struct iris_ip_opt),
						GFP_KERNEL);
			if (!pip_index[i].opt) {
				IRIS_LOGE("%s:%d no space\n", __func__, __LINE__);
				/*free already malloc space*/
				for (j = 0; j < i; j++) {
					kvfree(pip_index[j].opt);
					pip_index[j].opt = NULL;
				}
				return -ENOMEM;
			}
		}
	}

	return 0;
}

static int32_t _iris_alloc_desc_buf(struct dsi_cmd_desc **cmds, int cmd_cnt)
{
	int cmd_size = 0;

	/*create dsi cmds*/
	if (cmd_cnt == 0) {
		IRIS_LOGE("%s(), invalid statics count", __func__);
		return -EINVAL;
	}

	cmd_size = cmd_cnt * sizeof(struct dsi_cmd_desc);
	*cmds = vzalloc(cmd_size);
	if (!(*cmds)) {
		IRIS_LOGE("%s(), failed to malloc space for dsi", __func__);
		return -ENOMEM;
	}
	IRIS_LOGI("%s(), alloc %p, count %d", __func__, *cmds, cmd_cnt);

	return 0;
}

static int32_t _iris_alloc_cmd_buf(struct dsi_cmd_desc **cmds,
		struct iris_ip_index *pip_index, int cmd_cnt)
{
	int32_t rc = 0;

	rc = _iris_alloc_desc_buf(cmds, cmd_cnt);
	if (rc)
		return rc;

	rc = _iris_alloc_pip_buf(pip_index);
	if (rc) {
		vfree(*cmds);
		*cmds = NULL;
	}

	return rc;
}

static int32_t _iris_write_ip_opt(struct dsi_cmd_desc *cmd,
		const struct iris_cmd_header *hdr, int32_t pkt_cnt,
		struct iris_ip_index *pip_index)
{
	uint8_t i = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	uint8_t cnt = 0;

	if (!hdr || !cmd || !pip_index) {
		IRIS_LOGE("%s(), invalid input parameter.", __func__);
		return -EINVAL;
	}

	ip = hdr->ip_type & 0xff;
	opt_id = hdr->opt_and_link & 0xff;

	if (ip >= LUT_IP_START)
		ip -= LUT_IP_START;

	cnt = pip_index[ip].opt_cnt;

	for (i = 0; i < cnt; i++) {
		if (pip_index[ip].opt[i].cmd == NULL) {
			pip_index[ip].opt[i].cmd = cmd;
			pip_index[ip].opt[i].cmd_cnt = pkt_cnt;
			pip_index[ip].opt[i].opt_id = opt_id;
			break;
		} else if (pip_index[ip].opt[i].opt_id == opt_id) {
			/*find the right opt_id*/
			pip_index[ip].opt[i].cmd_cnt += pkt_cnt;
			break;
		}
	}

	if (i == cnt) {
		IRIS_LOGE("%s(), find i_p opt fail, i_p = 0x%02x opt = 0x%02x.",
				__func__, ip, opt_id);
		return -EINVAL;
	}

	/*to set link state*/
	if (pip_index[ip].opt[i].link_state == 0xff
			&& pip_index[ip].opt[i].opt_id == opt_id) {
		uint8_t link_state = 0;

		link_state = (hdr->opt_and_link >> 8) & 0xff;
		pip_index[ip].opt[i].link_state =
			link_state ? DSI_CMD_SET_STATE_LP : DSI_CMD_SET_STATE_HS;
	}

	return 0;
}

static int32_t _iris_trans_section_hdr_to_desc(
		struct dsi_cmd_desc *cmd, const struct iris_cmd_header *hdr)
{
	memset(cmd, 0, sizeof(struct dsi_cmd_desc));

	cmd->msg.type = (hdr->dsi_type & 0xff);
	cmd->post_wait_ms = (hdr->wait_us & 0xff);
	cmd->last_command = ((hdr->last_pkt & 0xff) != 0);
	cmd->msg.tx_len = hdr->payload_len;

	IRIS_LOGV("%s(), type: %#x, wait: %#x, last: %s, len: %zu",
			__func__,
			cmd->msg.type, cmd->post_wait_ms,
			cmd->last_command ? "true" : "false", cmd->msg.tx_len);

	return cmd->msg.tx_len;
}

static void _iris_change_last_and_size(struct iris_cmd_header *dest,
		const struct iris_cmd_header *src, int index, const int pkt_cnt)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int pkt_size = pcfg->split_pkt_size;

	memcpy(dest, src, sizeof(*src));
	if (index == pkt_cnt - 1) {
		dest->payload_len = src->payload_len * sizeof(uint32_t) - (pkt_cnt - 1) * pkt_size;
		return;
	}

	dest->last_pkt = 0;
	dest->payload_len = (pkt_size + IRIS_OCP_HEADER_ADDR_LEN);
}

static int _iris_write_cmd_hdr(struct dsi_cmd_desc *cmd,
		const struct iris_cmd_header *phdr, int pkt_cnt)
{
	int i = 0;
	struct iris_cmd_header tmp_hdr;

	for (i = 0; i < pkt_cnt; i++) {
		_iris_change_last_and_size(&tmp_hdr, phdr, i, pkt_cnt);

		/*add cmds hdr information*/
		_iris_trans_section_hdr_to_desc(cmd + i, &tmp_hdr);
	}

	return 0;
}

static bool _iris_need_direct_send(const struct iris_cmd_header *hdr)
{
	if (hdr == NULL) {
		IRIS_LOGE("%s(), invalid input!", __func__);
		return false;
	}

	if (hdr->ip_type == APP_CODE_LUT)
		return true;

	return false;
}

static void _iris_create_cmd_payload(const struct iris_cmd_header *hdr,
		const uint8_t *payload, uint8_t *msg_buf, int32_t buf_size)
{
	int32_t i = 0;
	uint32_t *pval = NULL;
	uint32_t cnt = 0;

	if (_iris_need_direct_send(hdr)) {
		memcpy(msg_buf, payload, buf_size);
		return;
	}

	pval = (uint32_t *)payload;
	cnt = buf_size >> 2;
	for (i = 0; i < cnt; i++)
		*(uint32_t *)(msg_buf + (i << 2)) = cpu_to_le32(pval[i]);
}

static int _iris_write_cmd_payload(struct dsi_cmd_desc *pdesc,
		const struct iris_cmd_header *hdr, const char *payload, int pkt_cnt)
{
	int i = 0, j = 0;
	uint32_t dlen = 0;
	uint8_t *ptr = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t pkt_size = pcfg->split_pkt_size;
	uint32_t ocp_type = _iris_get_ocp_type(payload);
	uint32_t base_addr = _iris_get_ocp_base_addr(payload);

	if (pkt_cnt == 1) {
		dlen = pdesc->msg.tx_len;
		ptr = vzalloc(dlen);
		if (!ptr) {
			IRIS_LOGE("%s Failed to allocate memory", __func__);
			return -ENOMEM;
		}

		_iris_create_cmd_payload(hdr, payload, ptr, dlen);
		pdesc->msg.tx_buf = ptr;
	} else {
		/*remove header and base address*/
		payload += IRIS_OCP_HEADER_ADDR_LEN;
		for (i = 0; i < pkt_cnt; i++) {
			dlen = pdesc[i].msg.tx_len;

			ptr = vzalloc(dlen);
			if (!ptr) {
				for (j = 0; j < i; j++) {
					vfree(pdesc[j].msg.tx_buf);
					pdesc[j].msg.tx_buf = NULL;
				}
				IRIS_LOGE("can not allocate space");
				return -ENOMEM;
			}

			_iris_set_ocp_base_addr(ptr, base_addr + i * pkt_size);
			_iris_set_ocp_type(ptr, ocp_type);
			_iris_create_cmd_payload(hdr, payload,
					ptr + IRIS_OCP_HEADER_ADDR_LEN,
					dlen - IRIS_OCP_HEADER_ADDR_LEN);

			/* add payload */
			payload += (dlen - IRIS_OCP_HEADER_ADDR_LEN);
			pdesc[i].msg.tx_buf = ptr;
		}
	}

	if (IRIS_IF_LOGVV()) {
		int len = 0;

		for (i = 0; i < pkt_cnt; i++) {
			len = (pdesc[i].msg.tx_len > 16) ? 16 : pdesc[i].msg.tx_len;
			print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4,
					pdesc[i].msg.tx_buf, len, false);
		}
	}

	return 0;
}

void iris_change_type_addr(struct iris_ip_opt *dest, struct iris_ip_opt *src)
{
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t pkt_size = pcfg->split_pkt_size;
	const void *buf = src->cmd->msg.tx_buf;
	int pkt_cnt = dest->cmd_cnt;
	uint32_t ocp_type = _iris_get_ocp_type(buf);
	uint32_t base_addr = _iris_get_ocp_base_addr(buf);

	for (i = 0; i < pkt_cnt; i++) {
		buf = dest->cmd[i].msg.tx_buf;
		_iris_set_ocp_base_addr(buf, base_addr + i * pkt_size);
		_iris_set_ocp_type(buf, ocp_type);
		IRIS_LOGD_IF(i == 0, "%s(), change ocp type 0x%08x, change base addr to 0x%08x.",
					__func__, ocp_type, base_addr);
	}
}

struct iris_ip_opt *iris_find_specific_ip_opt(uint8_t ip, uint8_t opt_id, int32_t type)
{
	int32_t i = 0;
	struct iris_ip_opt *popt = NULL;
	struct iris_ip_index *pip_index = NULL;

	IRIS_LOGV("%s(), i_p: %#x, opt: %#x, type: %d", __func__, ip, opt_id, type);
	if (!_iris_is_valid_ip(ip)) {
		IRIS_LOGE("%s(), i_p %d is out of range", __func__, ip);
		return NULL;
	}

	if (!_iris_is_valid_cmd_type(type)) {
		IRIS_LOGE("%s(), type %d is invalid", __func__, type);
		return NULL;
	}

	if (ip >= LUT_IP_START) {
		type = IRIS_LUT_PIP_IDX;
		ip -= LUT_IP_START;
	}

	pip_index = iris_get_ip_idx(type) + ip;

	for (i = 0; i < pip_index->opt_cnt; i++) {
		popt = pip_index->opt + i;
		if (popt->opt_id == opt_id)
			return popt;
	}

	return NULL;
}

static struct iris_ip_opt *_iris_find_master_ip_opt(
		uint8_t ip, uint8_t opt_id, int32_t cur_type)
{
	int32_t i = 0;
	struct iris_ip_opt *popt = NULL;
	struct iris_ip_index *pip_index = NULL;
	int32_t master_type = 0;

	if (!iris_is_master_timing_supported())
		return NULL;

	if (cur_type == IRIS_LUT_PIP_IDX)
		return NULL;

	master_type = iris_get_master_timing_type();
	if (master_type == IRIS_DTSI_NONE || master_type == cur_type)
		return NULL;

	pip_index = iris_get_ip_idx(master_type) + ip;
	for (i = 0; i < pip_index->opt_cnt; i++) {
		popt = pip_index->opt + i;
		if (popt->opt_id == opt_id)
			return popt;
	}

	return NULL;
}

struct iris_ip_opt *iris_find_ip_opt(uint8_t ip, uint8_t opt_id)
{
	int32_t i = 0;
	struct iris_ip_opt *popt = NULL;
	struct iris_ip_index *pip_index = NULL;
	int32_t type = iris_get_cmd_list_index();

	IRIS_LOGV("%s(), i_p: %#x, opt: %#x", __func__, ip, opt_id);
	if (!_iris_is_valid_ip(ip)) {
		IRIS_LOGE("%s(), i_p %d is out of range", __func__, ip);
		return NULL;
	}

	if (ip >= LUT_IP_START) {
		type = IRIS_LUT_PIP_IDX;
		ip -= LUT_IP_START;
	}

	pip_index = iris_get_ip_idx(type) + ip;

	for (i = 0; i < pip_index->opt_cnt; i++) {
		popt = pip_index->opt + i;
		if (popt->opt_id == opt_id)
			return popt;
	}

	return _iris_find_master_ip_opt(ip, opt_id, type);
}

static void _iris_print_ipopt(struct iris_ip_index  *pip_index)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	int32_t ip_cnt = IRIS_IP_CNT;

	if (_iris_get_ip_idx_type(pip_index) == IRIS_LUT_PIP_IDX)
		ip_cnt = LUT_IP_END - LUT_IP_START;

	for (i = 0; i < ip_cnt; i++) {
		for (j = 0; j < pip_index[i].opt_cnt; j++) {
			struct iris_ip_opt *popt = &(pip_index[i].opt[j]);

			IRIS_LOGI("%s(%d), i_p: %02x, opt: %02x, cmd: %p, len: %d, link state: %#x",
					__func__, __LINE__,
					i, popt->opt_id, popt->cmd, popt->cmd_cnt, popt->link_state);
			for (k = 0; k < popt->cmd_cnt; k++)
				IRIS_LOGI("[%d] last_command = %02x", k, popt->cmd[k].last_command);
		}
	}
}

static void _iris_parse_appversion(
		const uint8_t *payload, const struct iris_cmd_header *phdr)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t *pval = NULL;
	uint32_t app_version = 0x0;
	uint32_t app_date;

	if (phdr->ip_type != APP_VERSION_LUT)
		return;

	pval = (uint32_t *)payload;

	app_version = *pval;
	app_date = *(pval + 1);
	pcfg->app_version = app_version;
	pcfg->app_date[0] = app_date & 0xff;
	pcfg->app_date[1] = (app_date >> 8) & 0xff;
	pcfg->app_date[2] = (app_date >> 16) & 0xff;
	pcfg->app_date[3] = (app_date >> 24) & 0xff;
	IRIS_LOGI("%s(), iris fw version: %d, [date]%d:%d:%d:%d",
			__func__,
			pcfg->app_version,
			pcfg->app_date[3],
			pcfg->app_date[2],
			pcfg->app_date[1],
			pcfg->app_date[0]);
}

static int32_t _iris_add_cmd_to_ipidx(const struct iris_data *data,
		struct dsi_cmd_desc *cmds, int cmd_pos, struct iris_ip_index *pip_index)
{
	int32_t span = 0;
	int32_t pkt_cnt = 0;
	int32_t total_size = 0;
	int32_t payload_size = 0;
	struct dsi_cmd_desc *pdesc = NULL;
	const uint8_t *payload = NULL;
	const struct iris_cmd_header *hdr = NULL;
	const uint8_t *buf_ptr = (uint8_t *)data->buf;
	int32_t data_size = data->size;
	int32_t cmd_index = cmd_pos;

	while (total_size < data_size) {
		hdr = (const struct iris_cmd_header *)buf_ptr;
		pdesc = &cmds[cmd_index];
		payload = buf_ptr + sizeof(struct iris_cmd_header);
		payload_size = hdr->payload_len * sizeof(uint32_t);
		total_size += sizeof(struct iris_cmd_header) + payload_size;

		_iris_parse_appversion(payload, hdr);

		pkt_cnt = _iris_split_mult_pkt(payload, payload_size);
		IRIS_LOGV_IF(pkt_cnt > 1, "%s(), pkt_cnt is: %d.", __func__, pkt_cnt);

		/*need to first write desc header and then write payload*/
		_iris_write_cmd_hdr(pdesc, hdr, pkt_cnt);
		_iris_write_cmd_payload(pdesc, hdr, payload, pkt_cnt);

		/*write cmd link information*/
		_iris_write_ip_opt(pdesc, hdr, pkt_cnt, pip_index);

		buf_ptr += sizeof(struct iris_cmd_header) + payload_size;
		cmd_index += pkt_cnt;
	}
	span = cmd_index - cmd_pos;

	if (IRIS_IF_LOGVV())
		_iris_print_ipopt(pip_index);

	return span;
}

static int32_t _iris_create_ipidx(const struct iris_data *data, int32_t data_cnt,
		struct iris_ip_index *pip_index, int32_t cmd_cnt)
{
	int32_t i = 0;
	int32_t rc = 0;
	int32_t cmd_pos = 0;
	struct dsi_cmd_desc *cmds = NULL;

	/*create dsi cmd list*/
	rc = _iris_alloc_cmd_buf(&cmds, pip_index, cmd_cnt);
	if (rc) {
		IRIS_LOGE("create dsi memory failed!");
		return -ENOMEM;
	}

	_iris_init_ip_index(pip_index);

	for (i = 0; i < data_cnt; i++) {
		if (data[i].size == 0) {
			IRIS_LOGW("data[%d] length is %d.", i, data[i].size);
			continue;
		}
		cmd_pos += _iris_add_cmd_to_ipidx(&data[i], cmds, cmd_pos, pip_index);
	}

	if (cmd_cnt != cmd_pos) {
		IRIS_LOGE("%s(), invalid desc, cmd count: %d, cmd pos: %d.",
				__func__, cmd_cnt, cmd_pos);
	}

	return 0;
}

static int32_t _iris_accum_section_desc_cnt(const struct iris_cmd_header *hdr,
		const uint8_t *payload, int32_t *pcmd_cnt)
{
	int pkt_cnt = 1;
	int32_t payload_size = 0;

	if (!hdr || !pcmd_cnt || !payload) {
		IRIS_LOGE("%s(%d), invalid input parameter!", __func__, __LINE__);
		return -EINVAL;
	}

	payload_size = hdr->payload_len * sizeof(uint32_t);
	pkt_cnt = _iris_split_mult_pkt(payload, payload_size);

	/* it will split to pkt_cnt dsi cmds
	 * add (pkt_cnt-1) ocp_header(4 bytes) and ocp_type(4 bytes)
	 */
	*pcmd_cnt += pkt_cnt;

	IRIS_LOGV("%s(), dsi cmd count: %d", __func__, *pcmd_cnt);

	return 0;
}

static int32_t _iris_accum_section_opt_cnt(const struct iris_cmd_header *hdr,
		struct iris_ip_index *pip_index)
{
	uint8_t last = 0;
	uint8_t ip = 0;

	if (!hdr || !pip_index) {
		IRIS_LOGE("%s(%d), invalid input parameter.", __func__, __LINE__);
		return -EINVAL;
	}

	last = hdr->last_pkt & 0xff;
	ip = hdr->ip_type & 0xff;

	if (last == 1) {
		if (ip >= LUT_IP_START)
			ip -= LUT_IP_START;
		pip_index[ip].opt_cnt++;
	}

	return 0;
}

static int32_t _iris_poll_each_section(const struct iris_cmd_header *hdr,
		const char *payload, struct iris_ip_index *pip_index, int32_t *pcmd_cnt)
{
	int32_t rc = 0;

	rc = _iris_accum_section_desc_cnt(hdr, payload, pcmd_cnt);
	if (rc)
		goto EXIT_VAL;

	rc = _iris_accum_section_opt_cnt(hdr, pip_index);
	if (rc)
		goto EXIT_VAL;

	return 0;

EXIT_VAL:

	IRIS_LOGE("%s(), cmd static is error!", __func__);
	return rc;
}

static int32_t _iris_verify_dtsi(const struct iris_cmd_header *hdr,
		struct iris_ip_index *pip_index)
{
	uint32_t *pval = NULL;
	uint8_t tmp = 0;
	int32_t rc = 0;
	int32_t type = _iris_get_ip_idx_type(pip_index);

	if (_iris_is_valid_dtsi_cmd_type(type)) {
		if (hdr->ip_type >= IRIS_IP_CNT) {
			IRIS_LOGE("hdr->ip_type is  0x%0x out of max i_p", hdr->ip_type);
			rc = -EINVAL;
		} else if (((hdr->opt_and_link >> 8) & 0xff)  > 1) {
			IRIS_LOGE("hdr->opt link state not right 0x%0x", hdr->opt_and_link);
			rc = -EINVAL;
		}
	} else {
		if (hdr->ip_type >= LUT_IP_END || hdr->ip_type < LUT_IP_START) {
			IRIS_LOGE("hdr->i_p_type is  0x%0x out of i_p range", hdr->ip_type);
			rc = -EINVAL;
		}
	}

	switch (hdr->dsi_type) {
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_DCS_LONG_WRITE:
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		break;
	case MIPI_DSI_GENERIC_LONG_WRITE:
		/*judge payload0 for iris header*/
		pval = (uint32_t *)hdr + (sizeof(*hdr) >> 2);
		tmp = *pval & 0x0f;
		if (tmp == 0x00 || tmp == 0x05 || tmp == 0x0c || tmp == 0x08) {
			break;
		} else if (tmp == 0x04) {
			if ((hdr->payload_len - 1) % 2 != 0) {
				IRIS_LOGE("dlen is not right = %d", hdr->payload_len);
				rc = -EINVAL;
			}
		} else {
			IRIS_LOGE("payload hdr is not right = %0x", *pval);
			rc = -EINVAL;
		}
		break;
	default:
		IRIS_LOGE("dsi type is not right %0x", hdr->dsi_type);
		rc = -EINVAL;
	}

	if (rc) {
		IRIS_LOGE("hdr info: %#x %#x %#x %#x %#x %#x",
				hdr->dsi_type, hdr->last_pkt,
				hdr->wait_us, hdr->ip_type,
				hdr->opt_and_link, hdr->payload_len);
	}

	return rc;
}

static int32_t _iris_parse_cmd_param_path(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u8 *key = "pxlw,cmd-param-from-firmware";
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	rc = p_dts_ops->read_u32(np, key, &(pcfg->cmd_param_from_fw));
	if (rc) {
		IRIS_LOGE("%s(), failed to %s, return: %d",
			__func__, key, rc);
		pcfg->cmd_param_from_fw = 0;
		return rc;
	}

	IRIS_LOGI("%s(), pxlw platform type: %#x", __func__, pcfg->cmd_param_from_fw);

	return rc;
}

static int32_t _iris_parse_platform_type(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	rc = p_dts_ops->read_u32(np, "pxlw,platform", &(pcfg->platform_type));
	if (rc) {
		IRIS_LOGE("%s(), failed to pxlw platform, return: %d",
			__func__, rc);
		return rc;
	}

	IRIS_LOGI("%s(), pxlw platform type: %#x", __func__, pcfg->platform_type);

	return rc;
}

static int32_t _iris_parse_panel_type(
		struct device_node *np, struct iris_cfg *pcfg)
{
	const char *data = NULL;
	u32 value = 0;
	u8 values[2] = {};
	int32_t rc = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return rc;

	data = p_dts_ops->get_property(np, "pxlw,panel-type", NULL);
	if (data) {
		if (!strcmp(data, "PANEL_LCD_SRGB"))
			pcfg->panel_type = PANEL_LCD_SRGB;
		else if (!strcmp(data, "PANEL_LCD_P3"))
			pcfg->panel_type = PANEL_LCD_P3;
		else if (!strcmp(data, "PANEL_OLED"))
			pcfg->panel_type = PANEL_OLED;
		else/*default value is 0*/
			pcfg->panel_type = PANEL_LCD_SRGB;
	} else { /*default value is 0*/
		pcfg->panel_type = PANEL_LCD_SRGB;
		IRIS_LOGW("parse panel type failed!");
	}

	rc = p_dts_ops->read_u32(np, "pxlw,panel-dimming-brightness", &value);
	if (rc == 0) {
		pcfg->panel_dimming_brightness = value;
	} else {
		/* for V30 panel, 255 may cause panel during exit HDR, and lost TE.*/
		pcfg->panel_dimming_brightness = 250;
		IRIS_LOGW("parse panel dimming brightness failed!");
	}

	rc = p_dts_ops->read_u8_array(np, "pxlw,panel-hbm", values, 2);
	if (rc == 0) {
		pcfg->panel_hbm[0] = values[0];
		pcfg->panel_hbm[1] = values[1];
	} else {
		pcfg->panel_hbm[0] = 0xff;
		pcfg->panel_hbm[1] = 0xff;
	}

	return 0;
}

static int32_t _iris_parse_chip_ver(
		struct device_node *np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return rc;

	rc = p_dts_ops->read_u32(np, "pxlw,chip-ver",
			&(pcfg->chip_ver));
	if (rc) {
		IRIS_LOGE("can not get property: pxlw, chip-ver");
		return rc;
	}
	IRIS_LOGW("pxlw,chip-version: %#x", pcfg->chip_ver);

	return rc;
}

static int32_t _iris_parse_lut_mode(
		struct device_node *np, struct iris_cfg *pcfg)
{
	const char *data;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	data = p_dts_ops->get_property(np, "pxlw,lut-mode", NULL);
	if (data) {
		if (!strcmp(data, "single"))
			pcfg->lut_mode = SINGLE_MODE;
		else if (!strcmp(data, "interpolation"))
			pcfg->lut_mode = INTERPOLATION_MODE;
		else/*default value is 0*/
			pcfg->lut_mode = INTERPOLATION_MODE;
	} else { /*default value is 0*/
		pcfg->lut_mode = INTERPOLATION_MODE;
		IRIS_LOGI("no lut mode set, use default");
	}

	IRIS_LOGI("pxlw,lut-mode: %d", pcfg->lut_mode);
	return 0;
}

static int32_t _iris_parse_virtual_channel_id(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u8 vals[3] = {0, 0, 0};
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	rc = p_dts_ops->read_u8_array(np, "pxlw,virtual-channel-enable", vals, VC_ST_MAX);
	if (rc)
		IRIS_LOGE("%s(), failed to parse pxlw,virtual-channel-enable, return: %d", __func__, rc);

	pcfg->vc_ctrl.vc_arr[VC_PT] = vals[0];
	pcfg->vc_ctrl.vc_arr[VC_FRC] = vals[1];
	pcfg->vc_ctrl.vc_enable = pcfg->vc_ctrl.vc_arr[VC_PT];

	rc = p_dts_ops->read_u8_array(np, "pxlw,virtual-channel-id", vals, 3);
	if (rc)
		IRIS_LOGE("%s(), failed to parse virtual channel id, return: %d", __func__, rc);

	pcfg->vc_ctrl.to_iris_vc_id = vals[0];
	pcfg->vc_ctrl.to_panel_hs_vc_id = vals[1];
	pcfg->vc_ctrl.to_panel_lp_vc_id = vals[2];

	IRIS_LOGI("%s: vc_enable [%d] vc_arr [%d %d]", __func__,
			  pcfg->vc_ctrl.vc_enable, pcfg->vc_ctrl.vc_arr[0], pcfg->vc_ctrl.vc_arr[1]);
	IRIS_LOGI("%s(), parse pxlw,virtual-channel-id [%d %d %d]",
			  __func__, vals[0], vals[1], vals[2]);

	return rc;
}

static int32_t _iris_parse_ovs_delay(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	u32 *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0x00, 2);
	if (!payload) {
		IRIS_LOGE("%s(), can not get ovs-delay property in dtg setting", __func__);
		return -EINVAL;
	}

	pcfg->ovs_delay = payload[15] + 1;
	pcfg->ovs_delay_frc = payload[29] + 1;
	pcfg->vsw_vbp_delay = payload[4] + payload[5];

	IRIS_LOGI("%s(), ovs_delay is [%d], ovs_delay_frc is [%d], vsw_vbp_delay is [%d]",
		__func__, pcfg->ovs_delay, pcfg->ovs_delay_frc, pcfg->vsw_vbp_delay);

	return 0;
}

static int32_t _iris_parse_pwil_eco(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	u32 *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x90, 2);
	if (!payload) {
		IRIS_LOGE("%s(), can not get eco property in pwil setting", __func__);
		return -EINVAL;
	}

	if (payload[0] & BIT(11))
		pcfg->dtg_eco_enabled = true;
	else
		pcfg->dtg_eco_enabled = false;

	IRIS_LOGI("%s(), pcfg->dtg_eco_enabled is [%d]", __func__, pcfg->dtg_eco_enabled);

	return 0;
}

static const char *_iris_dsi_trans_mode_name(uint8_t mode)
{
	const char *name = NULL;

	switch (mode) {
	case DSI_NON_EMBEDDED_MODE:
		name = "non-embedded, default ma";
		break;
	case DSI_EMBEDDED_NO_MA_MODE:
		name = "embedded w/o ma";
		break;
	case DSI_EMBEDDED_MA_MODE:
		name = "embedded w/ ma";
		break;
	default:
		name = "unknown mode";
	}

	return name;
}

static int32_t _iris_parse_split_pkt_info(
		struct device_node *np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	uint32_t length = 0;
	uint32_t arr_32[6] = {0, 0, 0, 0, 0, 0};
	uint8_t arr_8[2] = {DSI_NON_EMBEDDED_MODE, DSI_EMBEDDED_NO_MA_MODE};
	const uint32_t *arr;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;


	rc = p_dts_ops->read_u32(np, "pxlw,pkt-payload-size",
			&(pcfg->split_pkt_size));
	if (rc) {
		IRIS_LOGE("can not get property: pxlw,pkt-payload-size, using default 228");
		pcfg->split_pkt_size = 228;
	}
	IRIS_LOGI("pxlw,split-pkt-payload-size: %d", pcfg->split_pkt_size);

	rc = p_dts_ops->read_u8_array(np, "pxlw,dsi-trans-mode", arr_8, 2);
	if (rc)
		IRIS_LOGE("%s(), failed to parse embedded enable, using default mode", __func__);
	pcfg->dsi_trans_mode[0] = arr_8[0];
	pcfg->dsi_trans_mode[1] = arr_8[1];
	IRIS_LOGI("%s(): lightup using %s mode, ", __func__,
		_iris_dsi_trans_mode_name(pcfg->dsi_trans_mode[0]));
	IRIS_LOGI("%s(): pq update using %s mode, ", __func__,
		_iris_dsi_trans_mode_name(pcfg->dsi_trans_mode[1]));

	arr = p_dts_ops->get_property(np, "pxlw,dsi-trans-len", &length);
	if (!arr) {
		IRIS_LOGE("%s, dsi-trans-len not found\n", __func__);
		rc = -EINVAL;
		goto last;
	}
	length = length / sizeof(u32);
	if (length != 6) {
		IRIS_LOGE("%s, dsi-trans-len only have %d parameters\n", __func__, length);
		rc = -EINVAL;
		goto last;
	}
	rc = p_dts_ops->read_u32_array(np, "pxlw,dsi-trans-len",
					arr_32, length);
	if (rc) {
		IRIS_LOGE("%s, cannot read dsi-trans-len, rc = %d\n", __func__, rc);
		goto last;
	}

last:
	pcfg->dsi_trans_len[0][0] = arr_32[0];	  //non-embedded, lightup
	pcfg->dsi_trans_len[0][1] = arr_32[1];	  //non-embedded, pqupdate
	pcfg->dsi_trans_len[1][0] = arr_32[2];	  //embedded-w/o-ma, lightup
	pcfg->dsi_trans_len[1][1] = arr_32[3];	  //embedded-w/o-ma, pqupdate
	pcfg->dsi_trans_len[2][0] = arr_32[4];	  //embedded-w/-ma, lightup
	pcfg->dsi_trans_len[2][1] = arr_32[5];	  //embedded-w/-ma, pqupdate

	IRIS_LOGI("%s(), dsi trans len, non-embedded:<%d %d>, embedded-no-ma:<%d %d>, embedded-ma:<%d %d>",
		__func__,
		pcfg->dsi_trans_len[0][0], pcfg->dsi_trans_len[0][1],
		pcfg->dsi_trans_len[1][0], pcfg->dsi_trans_len[1][1],
		pcfg->dsi_trans_len[2][0], pcfg->dsi_trans_len[2][1]);


	return rc;
}

static int32_t _iris_poll_cmd_lists(const struct iris_data *data, int32_t data_cnt,
		struct iris_ip_index *pip_index, int32_t *pcmd_cnt)
{
	int32_t rc = 0;
	int32_t i = 0;
	int32_t payload_size = 0;
	int32_t data_len = 0;
	const uint8_t *buf_ptr = NULL;
	const struct iris_cmd_header *hdr = NULL;

	if (data == NULL || pip_index == NULL || pcmd_cnt == NULL) {
		IRIS_LOGE("%s(), invalid input!", __func__);
		return -EINVAL;
	}

	for (i = 0; i < data_cnt; i++) {
		if (data[i].size == 0) {
			IRIS_LOGW("data length is = %d", data[i].size);
			continue;
		}

		buf_ptr = data[i].buf;
		data_len = data[i].size;
		while (data_len >= sizeof(struct iris_cmd_header)) {
			hdr = (const struct iris_cmd_header *)buf_ptr;
			data_len -= sizeof(struct iris_cmd_header);
			if (hdr->payload_len > (data_len >> 2)) {
				IRIS_LOGE("%s: length error, i_p = 0x%02x opt=0x%02x, len=%d",
						__func__, hdr->ip_type, hdr->opt_and_link, hdr->payload_len);
				return -EINVAL;
			}

			if (IRIS_IF_LOGVV()) {
				rc = _iris_verify_dtsi(hdr, pip_index);
				if (rc) {
					IRIS_LOGE("%s(%d), verify dtis return: %d", __func__, __LINE__, rc);
					return rc;
				}
			}

			IRIS_LOGV("hdr info, type: 0x%02x, last: 0x%02x, wait: 0x%02x, i_p: 0x%02x, opt: 0x%02x, len: %d.",
					hdr->dsi_type, hdr->last_pkt, hdr->wait_us,
					hdr->ip_type, hdr->opt_and_link, hdr->payload_len);

			//payload
			buf_ptr += sizeof(struct iris_cmd_header);

			/*change to uint8_t length*/
			//hdr->payload_len *= sizeof(uint32_t);
			payload_size = hdr->payload_len * sizeof(uint32_t);

			rc = _iris_poll_each_section(hdr, buf_ptr, pip_index, pcmd_cnt);
			if (rc) {
				IRIS_LOGE("%s(), failed to poll section: %d, return: %d",
						__func__, hdr->ip_type, rc);
				return rc;
			}

			buf_ptr += payload_size;
			data_len -= payload_size;
		}
	}

	return rc;
}

static int32_t _iris_alloc_dtsi_cmd_buf(
		const struct device_node *np, const uint8_t *key, uint8_t **buf)
{
	int32_t cmd_size = 0;
	int32_t cmd_len = 0;
	const void *ret = NULL;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	ret = p_dts_ops->get_property(np, key, &cmd_len);
	if (!ret) {
		IRIS_LOGE("%s(), failed for parsing %s", __func__, key);
		return -EINVAL;
	}

	if (cmd_len % 4 != 0) {
		IRIS_LOGE("length = %d is not multpile of 4", cmd_len);
		return -EINVAL;
	}

	cmd_size = sizeof(char) * cmd_len;
	*buf = vzalloc(cmd_size);
	if (!*buf) {
		IRIS_LOGE("can not vzalloc memory");
		return  -ENOMEM;
	}

	return cmd_size;
}

static int32_t _iris_write_dtsi_cmd_to_buf(const struct device_node *np,
		const uint8_t *key, uint8_t **buf, int size)
{
	int32_t rc = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	rc = p_dts_ops->read_u32_array(np, key,
			(uint32_t *)(*buf), size >> 2);
	if (rc != 0) {
		IRIS_LOGE("%s(%d), read array is not right", __func__, __LINE__);
		return -EINVAL;
	}

	return rc;
}

static void _iris_free_dtsi_cmd_buf(uint8_t **buf)
{
	if (*buf) {
		vfree(*buf);
		*buf = NULL;
	}
}

static void _iris_save_cmd_count(const struct iris_ip_index *pip_index,
		const int cmd_cnt)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int32_t idx_type = _iris_get_ip_idx_type(pip_index);

	if (_iris_is_valid_dtsi_cmd_type(idx_type)
			&& pcfg->dtsi_cmds_cnt < cmd_cnt) {
		pcfg->dtsi_cmds_cnt = cmd_cnt;
		return;
	}

	if (idx_type == IRIS_LUT_PIP_IDX
			&& pcfg->lut_cmds_cnt < cmd_cnt) {
		pcfg->lut_cmds_cnt = cmd_cnt;
		return;
	}

	IRIS_LOGI("%s(), doesn't save count for type %#x pip index %p",
			__func__, idx_type, pip_index);
}

int32_t iris_attach_cmd_to_ipidx(const struct iris_data *data,
		int32_t data_cnt, struct iris_ip_index *pip_index)
{
	int32_t rc = 0;
	int32_t cmd_cnt = 0;

	if (pip_index == NULL) {
		IRIS_LOGE("%s(), pip index", __func__);
		return -EINVAL;
	}

	rc = _iris_poll_cmd_lists(data, data_cnt, pip_index, &cmd_cnt);
	if (rc) {
		IRIS_LOGE("fail to parse dtsi/lut cmd list!");
		return rc;
	}

	_iris_save_cmd_count(pip_index, cmd_cnt);

	rc = _iris_create_ipidx(data, data_cnt, pip_index, cmd_cnt);

	return rc;
}

int32_t iris_parse_dtsi_cmd(const struct device_node *lightup_node, uint32_t cmd_index)
{
	int32_t rc = 0;
	int32_t cmd_size = 0;
	int32_t data_cnt = 0;
	uint8_t *dtsi_buf = NULL;
	struct iris_ip_index *pip_index = NULL;
	struct iris_data data[1];
	const int32_t str_len = 24;
	uint8_t key[24] = "pxlw,iris-cmd-list";

	if (!_iris_is_valid_dtsi_cmd_type(cmd_index)) {
		IRIS_LOGE("%s(), invalid cmd index: %u", __func__, cmd_index);
		return -EINVAL;
	}

	if (cmd_index != IRIS_DTSI_PIP_IDX_START)
		snprintf(key, str_len, "pxlw,iris-cmd-list-%d", cmd_index);

	IRIS_LOGI("%s(), parse '%s' for cmd list %d", __func__, key, cmd_index);
	memset(data, 0x00, sizeof(data));

	// need to keep dtsi buf and release after used
	cmd_size = _iris_alloc_dtsi_cmd_buf(lightup_node, key, &dtsi_buf);
	if (cmd_size <= 0) {
		IRIS_LOGE("can not malloc space for dtsi cmd");
		return -ENOMEM;
	}

	rc = _iris_write_dtsi_cmd_to_buf(lightup_node, key, &dtsi_buf, cmd_size);
	if (rc) {
		IRIS_LOGE("cant not write dtsi cmd to buf");
		goto FREE_DTSI_BUF;
	}
	data[0].buf = dtsi_buf;
	data[0].size = cmd_size;

	pip_index = iris_get_ip_idx(cmd_index);
	data_cnt = ARRAY_SIZE(data);
	rc = iris_attach_cmd_to_ipidx(data, data_cnt, pip_index);

FREE_DTSI_BUF:
	_iris_free_dtsi_cmd_buf(&dtsi_buf);

	return rc;
}

static void _iris_add_cmd_seq(struct iris_ctrl_opt *ctrl_opt,
		int item_cnt, const uint8_t *pdata)
{
	int32_t i = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	uint8_t chain = 0;
	const int32_t span = 3;

	for (i = 0; i < item_cnt; i++) {
		ip = pdata[span * i];
		opt_id = pdata[span * i + 1];
		chain = pdata[span * i + 2];

		ctrl_opt[i].ip = ip & 0xff;
		ctrl_opt[i].opt_id = opt_id & 0xff;
		ctrl_opt[i].chain = chain & 0xff;

		if (IRIS_IF_LOGV()) {
			IRIS_LOGI("i_p = %d opt = %d  chain=%d",
					ip, opt_id, chain);
		}
	}
}

static int32_t _iris_alloc_cmd_seq(
		struct iris_ctrl_seq  *pctrl_seq, int32_t seq_cnt)
{
	pctrl_seq->ctrl_opt = vmalloc(seq_cnt * sizeof(struct iris_ctrl_seq));
	if (pctrl_seq->ctrl_opt == NULL) {
		IRIS_LOGE("can not malloc space for pctrl opt");
		return -ENOMEM;
	}
	pctrl_seq->cnt = seq_cnt;

	return 0;
}

static int32_t _iris_parse_cmd_seq_data(struct device_node *np,
		const uint8_t *key, const uint8_t **pval)
{
	const uint8_t *pdata = NULL;
	int32_t item_cnt = 0;
	int32_t seq_cnt = 0;
	int32_t span = 3;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	pdata = p_dts_ops->get_property(np, key, &item_cnt);
	if (!pdata) {
		IRIS_LOGI("%s(), can't get '%s'", __func__, key);
		return -EINVAL;
	}

	seq_cnt =  (item_cnt / span);
	if (item_cnt == 0 || item_cnt != span * seq_cnt) {
		IRIS_LOGE("parse %s len is not right = %d", key, item_cnt);
		return -EINVAL;
	}

	*pval = pdata;

	return seq_cnt;
}

static int32_t _iris_parse_cmd_seq_common(
		struct device_node *np, const uint8_t *pre_key,
		const uint8_t *key, struct iris_ctrl_seq *pctrl_seq)
{
	int32_t pre_seq_cnt = 0;
	int32_t seq_cnt = 0;
	int32_t sum = 0;
	int32_t rc = 0;
	const uint8_t *pdata = NULL;
	const uint8_t *pre_pdata = NULL;

	pre_seq_cnt = _iris_parse_cmd_seq_data(np, pre_key, &pre_pdata);
	if (pre_seq_cnt <= 0) {
		IRIS_LOGE("%s(), failed to parse pre '%s'.", __func__, pre_key);
		return -EINVAL;
	}

	seq_cnt = _iris_parse_cmd_seq_data(np, key, &pdata);
	if (seq_cnt <= 0) {
		IRIS_LOGE("%s(), failed to parse '%s'.", __func__, key);
		return -EINVAL;
	}

	sum = pre_seq_cnt + seq_cnt;

	rc = _iris_alloc_cmd_seq(pctrl_seq, sum);
	if (rc != 0)
		return rc;

	_iris_add_cmd_seq(pctrl_seq->ctrl_opt, pre_seq_cnt, pre_pdata);
	_iris_add_cmd_seq(&pctrl_seq->ctrl_opt[pre_seq_cnt], seq_cnt, pdata);

	return rc;
}

static int32_t _iris_parse_cmd_seq(
		struct device_node *np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	uint8_t *pre0_key = "pxlw,iris-lightup-sequence-pre0";
	uint8_t *pre1_key = "pxlw,iris-lightup-sequence-pre1";
	uint8_t *key = "pxlw,iris-lightup-sequence";

	rc = _iris_parse_cmd_seq_common(np, pre0_key, key, pcfg->ctrl_seq);
	if (rc != 0)
		return rc;

	return _iris_parse_cmd_seq_common(np, pre1_key, key, pcfg->ctrl_seq + 1);
}

int32_t iris_parse_optional_seq(struct device_node *np, const uint8_t *key,
		struct iris_ctrl_seq *pseq)
{
	int32_t rc = 0;
	int32_t seq_cnt = 0;
	const uint8_t *pdata = NULL;

	seq_cnt = _iris_parse_cmd_seq_data(np, key, &pdata);
	if (seq_cnt <= 0) {
		IRIS_LOGI("%s(), [optional] without sequence for '%s'", __func__, key);
		return 0;
	}

	rc = _iris_alloc_cmd_seq(pseq, seq_cnt);
	if (rc != 0) {
		IRIS_LOGE("%s(), failed to alloc for '%s' seq, return %d",
				__func__, key, rc);
		return rc;
	}

	_iris_add_cmd_seq(pseq->ctrl_opt, seq_cnt, pdata);

	return rc;
}

/*use for debug cont-splash lk part*/
static int32_t _iris_parse_cont_splash_cmd_seq(
		struct device_node *np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	uint8_t *pre0_key = "pxlw,iris-lightup-sequence-pre0";
	uint8_t *pre1_key = "pxlw,iris-lightup-sequence-pre1";
	uint8_t *key = "pxlw,iris-lightup-sequence-cont-splash";

	rc = _iris_parse_cmd_seq_common(np, pre0_key, key, pcfg->ctrl_seq_cs);
	if (rc != 0)
		return rc;

	return _iris_parse_cmd_seq_common(np, pre1_key,
			key, pcfg->ctrl_seq_cs + 1);
}

static int32_t _iris_parse_tx_mode(
		struct device_node *np,
		struct dsi_panel *panel, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u8 tx_mode;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return rc;

	pcfg->rx_mode = panel->panel_mode;
	pcfg->tx_mode = panel->panel_mode;
	IRIS_LOGI("%s, panel_mode = %d", __func__, panel->panel_mode);
	rc = p_dts_ops->read_u8(np, "pxlw,iris-tx-mode", &tx_mode);
	if (!rc) {
		IRIS_LOGI("get property: pxlw, iris-tx-mode: %d", tx_mode);
		//pcfg->tx_mode = tx_mode;
	}
	if (pcfg->rx_mode == pcfg->tx_mode)
		pcfg->pwil_mode = PT_MODE;
	else
		pcfg->pwil_mode = RFB_MODE;

	IRIS_LOGI("%s(), pwil mode: %d", __func__, pcfg->pwil_mode);
	return 0;
}

static int _iris_parse_pwr_entries(struct dsi_display *display)
{
	int32_t rc = 0;
	char *supply_name = NULL;
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();

	if (!display || !display->panel)
		return -EINVAL;

	if (!strcmp(display->panel->type, "primary")) {
		supply_name = "qcom,iris-supply-entries";

		rc = dsi_pwr_of_get_vreg_data(&display->panel->utils,
				&pcfg->iris_power_info, supply_name);
		if (rc) {
			rc = -EINVAL;
			IRIS_LOGE("%s pwr enters error", __func__);
		}
	}
	return rc;
}

static void _iris_parse_bl_endian(struct dsi_panel *panel, struct iris_cfg *pcfg)
{
	if (!panel || !pcfg)
		return;

	pcfg->switch_bl_endian = panel->utils.read_bool(panel->utils.data,
			"pxlw,switch-bl-endian");
	IRIS_LOGI("%s(), switch backlight endian: %s",
			__func__,
			pcfg->switch_bl_endian ? "true" : "false");
}

static void _iris_parse_default_aux_size(struct iris_cfg *pcfg)
{
	u32 *payload = NULL;
	u32 SET0_PPS6, SET0_PPS7, SET0_PPS8, SET0_PPS9;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DSC_DEC_AUX, 0xF0, 2);
	if (!payload)
		return;
	SET0_PPS6 = payload[4] >> 16 & 0xff;
	SET0_PPS7 = payload[4] >> 24 & 0xff;
	SET0_PPS8 = payload[5] >> 0 & 0xff;
	SET0_PPS9 = payload[5] >> 8 & 0xff;
	pcfg->aux_height_in_using = SET0_PPS6 << 8 | SET0_PPS7;
	pcfg->aux_width_in_using = SET0_PPS8 << 8 | SET0_PPS9;
}

static int _iris_parse_cmd_list(struct device_node *np)
{
	int cmd_idx = IRIS_DTSI_PIP_IDX_START;
	int rc = -EINVAL;

	do {
		rc = iris_parse_dtsi_cmd(np, cmd_idx);
		IRIS_LOGI_IF(rc != 0,
				"%s(), [optional] have not cmds list %d", __func__, cmd_idx);
		if (rc)
			return rc;
		cmd_idx++;
	} while (cmd_idx < iris_get_cmd_list_cnt());

	return rc;
}

int iris_parse_cmd_param(struct device_node *lightup_node)
{
	int rc = -EINVAL;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!lightup_node)
		IRIS_LOGI("lightup node is null, use firmware to parse cmd params");

	rc = _iris_parse_cmd_seq(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse cmd seq error");
		return rc;
	}

	rc = _iris_parse_cmd_list(lightup_node);
	if (rc) {
		IRIS_LOGE("_iris_parse_cmd_list error");
		return rc;
	}

	rc = _iris_parse_ovs_delay(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse ovs delay, return: %d",
				  __func__, rc);
		return rc;
	}

	rc = _iris_parse_pwil_eco(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse eco delay, return: %d",
				  __func__, rc);
		return rc;
	}
	iris_parse_mv_resolution();

	_iris_parse_default_aux_size(pcfg);
	return rc;
}


static int _iris_parse_subnode(struct dsi_display *display, void *node)
{
	int32_t rc = -EINVAL;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct device_node *lightup_node = node;

	if (!lightup_node)
		IRIS_LOGE("%s(), lightup node is null, so parse from fw", __func__);
	else
		IRIS_LOGI("%s(), lightup node: %s", __func__, lightup_node->name);

	rc = _iris_parse_split_pkt_info(lightup_node, pcfg);
	if (rc)
		IRIS_LOGI("%s(), failed to parse split pkt info, using default info", __func__);

	iris_parse_color_temp_range(lightup_node, pcfg);

	rc = _iris_parse_tx_mode(lightup_node, display->panel, pcfg);
	if (rc)
		IRIS_LOGE("no set iris tx mode!");

	rc = iris_parse_timing_switch_info(lightup_node);
	if (rc)
		IRIS_LOGI("%s, [optional] have not timing switch info", __func__);

	rc = iris_parse_default_pq_param(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse pq init error");
		return rc;
	}

	rc = _iris_parse_platform_type(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse platform type, return: %d",
			__func__, rc);
		return rc;
	}

	rc = _iris_parse_panel_type(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse panel type error");
		return rc;
	}

	_iris_parse_lut_mode(lightup_node, pcfg);

	rc = _iris_parse_chip_ver(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse chip ver error");
		return rc;
	}

	rc = iris_parse_lp_ctrl(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse low power control error");
		return rc;
	}

	rc = _iris_parse_virtual_channel_id(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse virtual channel id info, return: %d",
				  __func__, rc);
		return rc;
	}

	rc = _iris_parse_cont_splash_cmd_seq(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("parse cont splash cmd seq error");
		return rc;
	}

	iris_parse_ap_panel_te(lightup_node);

	iris_parse_frc_dec_initial_delay(lightup_node);

	rc = _iris_parse_pwr_entries(display);
	if (rc)
		IRIS_LOGE("pwr entries error\n");

	_iris_parse_bl_endian(display->panel, pcfg);

	_iris_parse_cmd_param_path(lightup_node, pcfg);
	if (!pcfg->cmd_param_from_fw)
		iris_parse_cmd_param(lightup_node);

	pcfg->valid = PARAM_PARSED;	/* parse ok */
	IRIS_LOGI("%s(%d), exit.", __func__, __LINE__);

	return 0;
}

static int _iris_parse_iris_golden_fw(struct device_node *np)
{
	const char *names_array[8];
	int index, count = 0;
	int rc;
	struct iris_cfg *pcfg = iris_get_cfg();

	rc = of_property_match_string(np, "iris-calibration-golden-fw-list", "iris7");
	if (unlikely(rc < 0)) {
		IRIS_LOGI("%s(), doesn't contain \"iris7\"", __func__);
		goto default_fw_name;
	} else {
		/* Read number of names from fw-list property */
		index = rc;
		count = of_property_count_strings(np, "iris-calibration-golden-fw-list");
		rc = of_property_read_string_array(np, "iris-calibration-golden-fw-list",
			names_array, count);
		if (rc < 0) {
			IRIS_LOGI("Failed to invoke of_property_read_string_array()");
			goto default_fw_name;
		}
		if (index + 3 < count) {
			pcfg->ccf1_name = names_array[index + 1];
			pcfg->ccf2_name = names_array[index + 2];
			pcfg->ccf3_name = names_array[index + 3];
			IRIS_LOGD("%s(), name: %s %s %s", __func__, pcfg->ccf1_name, pcfg->ccf2_name, pcfg->ccf3_name);
		} else {
			rc = -1;
			goto default_fw_name;
		}
	}
	return rc;

default_fw_name:
	pcfg->ccf1_name = NULL;
	pcfg->ccf2_name = NULL;
	pcfg->ccf3_name = NULL;
	IRIS_LOGI("can not parse golden fw name, using default");
	return rc;
}

int iris_parse_param(struct dsi_display *display)
{
	int32_t ret = 0;
	struct device_node *lightup_node = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(%d), enter.", __func__, __LINE__);
	if (!display || !display->pdev->dev.of_node || !display->panel_node) {
		IRIS_LOGE("the param is null");
		return -EINVAL;
	}
	if (display->panel->is_secondary)
		return 0;

	pcfg->valid = PARAM_EMPTY;	/* empty */
	spin_lock_init(&pcfg->iris_1w_lock);
	init_completion(&pcfg->frame_ready_completion);
	_iris_parse_iris_golden_fw(display->panel_node);

	lightup_node = of_parse_phandle(display->pdev->dev.of_node, "pxlw,iris-lightup-config", 0);
	if (lightup_node) {
		iris_set_dts_ops(DTS_CTX_FROM_IMG);
		ret = _iris_parse_subnode(display, lightup_node);
	}
	return ret;
}


struct iris_pq_ipopt_val *iris_get_cur_ipopt_val(uint8_t ip)
{
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_pq_init_val *pinit_val = &(pcfg->pq_init_val);

	for (i = 0; i < pinit_val->ip_cnt; i++) {
		struct iris_pq_ipopt_val *pq_ipopt_val = pinit_val->val + i;

		if (ip == pq_ipopt_val->ip)
			return pq_ipopt_val;
	}

	return NULL;
}

static void _iris_reset_out_cmds(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int sum = pcfg->dtsi_cmds_cnt + pcfg->lut_cmds_cnt;

	memset(pcfg->iris_cmds.iris_cmds_buf, 0x00,
			sum * sizeof(struct dsi_cmd_desc));
	pcfg->iris_cmds.cmds_index = 0;
}

static int32_t _iris_init_cmd_comp(int32_t ip,
		int32_t opt_index, struct iris_cmd_comp *pcmd_comp)
{
	struct iris_ip_opt *opt = NULL;

	if (!_iris_is_valid_ip(ip)) {
		IRIS_LOGE("%s(), invalid i_p: %#x", __func__, ip);
		return -EINVAL;
	}

	opt = iris_find_ip_opt(ip, opt_index);
	if (!opt) {
		IRIS_LOGE("%s(), can not find popt, i_p: %#x, opt: %#x",
				__func__, ip, opt_index);
		return -EINVAL;
	}

	pcmd_comp->cmd = opt->cmd;
	pcmd_comp->cnt = opt->cmd_cnt;
	pcmd_comp->link_state = opt->link_state;
	IRIS_LOGV("%s(), opt count: %d, link state: %#x",
			__func__, pcmd_comp->cnt, pcmd_comp->link_state);

	return 0;
}

void iris_print_desc_cmds(struct dsi_cmd_desc *p, int cmd_cnt, int state)
{
	int i = 0;
	int j = 0;
	int msg_len = 0;
	int dlen = 0;
	uint8_t *arr = NULL;
	uint8_t *ptr = NULL;
	uint8_t *ptr_tx = NULL;
	struct dsi_cmd_desc *pcmd = NULL;
	int str_len = 0;

	IRIS_LOGI("%s(), cmd len: %d, state: %s", __func__, cmd_cnt,
			(state == DSI_CMD_SET_STATE_HS) ? "high speed" : "low power");

	for (i = 0; i < cmd_cnt; i++) {
		pcmd = p + i;
		dlen = pcmd->msg.tx_len;
		msg_len = 3 * dlen + 23; //3* 7(dchdr) + 1(\n) + 1 (0)
		arr = vmalloc(msg_len * sizeof(uint8_t));
		if (!arr) {
			IRIS_LOGE("%s(), fail to malloc space", __func__);
			return;
		}
		memset(arr, 0x00, sizeof(uint8_t) * msg_len);

		ptr = arr;
		ptr_tx = (uint8_t *) pcmd->msg.tx_buf;
		str_len = snprintf(ptr, msg_len, "\" %02X", pcmd->msg.type);
		ptr += str_len;
		for (j = 0; j < dlen; j++) {
			str_len = snprintf(ptr, msg_len - (ptr - arr), " %02X", ptr_tx[j]);
			ptr += str_len;
		}
		snprintf(ptr, msg_len - (ptr - arr), "\\n\"");
		IRIS_LOGE("%s", arr);

		if (pcmd->post_wait_ms > 0)
			IRIS_LOGE("\" FF %02X\\n\"", pcmd->post_wait_ms);

		vfree(arr);
		arr = NULL;
	}
}

static void _iris_print_total_cmds(struct dsi_cmd_desc *cmds, int cmd_cnt)
{
	int i = 0;
	int j = 0;
	int value_count = 0;
	int print_count = 0;
	struct dsi_cmd_desc *pcmd = NULL;
	uint32_t *pval = NULL;

	if (IRIS_IF_NOT_LOGD())
		return;

	IRIS_LOGD("%s(), package count in cmd list: %d", __func__, cmd_cnt);
	for (i = 0; i < cmd_cnt; i++) {
		pcmd = cmds + i;
		value_count = pcmd->msg.tx_len/sizeof(uint32_t);
		print_count = value_count;

		pval = (uint32_t *)pcmd->msg.tx_buf;
		IRIS_LOGI("%s(), payload value count: %d, print count: %d, ocp type: 0x%08x, addr: 0x%08x",
			__func__, value_count, print_count, pval[0], pval[1]);

		for (j = 0; j < print_count; j++)
			IRIS_LOGI("0x%08x ", pval[j]);
	}
}

static void _iris_print_spec_cmds(struct dsi_cmd_desc *p, int cmd_cnt)
{
	int i = 0;
	int j = 0;
	int value_count = 0;
	int print_count = 0;
	struct dsi_cmd_desc *pcmd = NULL;
	uint32_t *pval = NULL;

	if (IRIS_IF_NOT_LOGD())
		return;

	IRIS_LOGD("%s(), package count in cmd list: %d", __func__, cmd_cnt);
	for (i = 0; i < cmd_cnt; i++) {
		pcmd = p + i;
		value_count = pcmd->msg.tx_len/sizeof(uint32_t);
		print_count = value_count;
		if (value_count > 16)
			print_count = 16;
		pval = (uint32_t *)pcmd->msg.tx_buf;
		if (i == 0 || i == cmd_cnt-1) {
			IRIS_LOGD("%s(), package: %d, type: 0x%02x, last: %s, channel: 0x%02x, flags: 0x%04x, wait: 0x%02x, send size: %zu.",
					__func__, i,
					pcmd->msg.type, pcmd->last_command?"true":"false", pcmd->msg.channel,
					pcmd->msg.flags, pcmd->post_wait_ms, pcmd->msg.tx_len);

			if (IRIS_IF_NOT_LOGV())
				continue;

			IRIS_LOGV("%s(), payload value count: %d, print count: %d, ocp type: 0x%08x, addr: 0x%08x",
					__func__, value_count, print_count, pval[0], pval[1]);
			for (j = 2; j < print_count; j++)
				IRIS_LOGV("0x%08x", pval[j]);

			if (i == cmd_cnt-1 && value_count > 4 && print_count != value_count) {
				IRIS_LOGV("%s(), payload tail: 0x%08x, 0x%08x, 0x%08x, 0x%08x.", __func__,
						pval[value_count-4], pval[value_count-3],
						pval[value_count-2], pval[value_count-1]);
			}
		}
	}
}

static void _iris_print_dtsi_cmds_for_lk(struct dsi_cmd_desc *cmds,
		int32_t cnt, int32_t wait, int32_t link_state)
{
	if (iris_get_cont_splash_type() != IRIS_CONT_SPLASH_LK)
		return;

	//restore the last cmd wait time
	if (wait != 0)
		cmds[cnt-1].post_wait_ms = 1;

	iris_print_desc_cmds(cmds, cnt, link_state);
}

static int32_t _iris_i2c_send_ocp_cmds(struct dsi_panel *panel,
		struct iris_cmd_comp *pcmd_comp)
{
	int ret = 0;
	int i = 0;
	struct iris_i2c_msg *msg = NULL;
	uint32_t msg_num = 0;

	msg_num = pcmd_comp->cnt;
	msg = kmalloc_array(msg_num, sizeof(struct iris_i2c_msg), GFP_KERNEL);
	if (msg == NULL) {
		IRIS_LOGE("%s(), failed to allocate memory", __func__);
		return -EINVAL;
	}

	for (i = 0; i < msg_num; i++) {
		msg[i].buf = (uint8_t *)pcmd_comp->cmd[i].msg.tx_buf;
		msg[i].len = pcmd_comp->cmd[i].msg.tx_len;
	}

	ret = iris_i2c_multi_write(msg, msg_num);

	kfree(msg);

	return ret;
}

static uint32_t _iris_calculate_delay_us(uint32_t payload_size, uint32_t cmd_num)
{
	uint32_t delay_us = 0;
	uint32_t panel_mbit_clk = 0;
	uint32_t lane_num = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGV("%s(%d), clk_rate_hz is %u, num_data_lanes is %d", __func__, __LINE__,
		pcfg->panel->cur_mode->timing.clk_rate_hz,
		pcfg->panel->host_config.num_data_lanes);

	if (pcfg->panel->cur_mode->timing.clk_rate_hz)
		panel_mbit_clk = pcfg->panel->cur_mode->timing.clk_rate_hz / 1000000;

	if (!panel_mbit_clk) {
		IRIS_LOGE("%s(%d), panel_mbit_clk is 0, default set to 50MHz.",
			__func__, __LINE__);
		panel_mbit_clk = 50;
	}

	lane_num = pcfg->panel->host_config.num_data_lanes;
	if (!lane_num) {
		/*default set to 4 lanes*/
		lane_num = 4;
	}

	/* follow:
	 *	8*(total_payload_size + total_command_num*6)*(1+inclk/pclk)/(lane_num*bitclk)
	 *	assume inclk/pclk = 2, this is the max value
	 */
	delay_us = 8 * (payload_size + cmd_num * 6) * (1+2) / (lane_num * panel_mbit_clk);

	return delay_us;
}

void iris_insert_delay_us(uint32_t payload_size, uint32_t cmd_num)
{
	uint32_t delay_us = 0;

	IRIS_LOGD("%s, payload_size is %d, cmd_num is %d",
		__func__, payload_size, cmd_num);

	if ((!payload_size) || (!cmd_num))
		return;

	if ((payload_size > 4096) || (cmd_num > 128))
		IRIS_LOGE("%s, it is risky to send such packets, payload_size %d, cmd_num %d",
			__func__, payload_size, cmd_num);
	/*embedded size is 240, non-embedded size is 256, use 240 default*/
	delay_us = _iris_calculate_delay_us(payload_size, payload_size/240 + 1);

	IRIS_LOGD("%s(%d): delay_us is %d", __func__, __LINE__, delay_us);

	if (delay_us)
		udelay(delay_us);
}

static int32_t _iris_dsi_single_addr_send(struct dsi_panel *panel, struct iris_cmd_comp *pcmd_comp)
{
	int32_t ret, i;
	uint32_t wait = 0;
	uint32_t trans_len, cont_pkt_len;
	struct dsi_cmd_desc *cmd = NULL;
	struct dsi_cmd_desc *cmd_cur = NULL;
	struct dsi_cmd_desc *cmd_next = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	const uint32_t split_cmd_len = pcfg->split_pkt_size + 8;

	if (!pcmd_comp) {
		IRIS_LOGE("%s(), cmd list is null.", __func__);
		return -EINVAL;
	}

	if (pcmd_comp->op_type == IRIS_LIGHTUP_OP)
		trans_len = pcfg->dsi_trans_len[1][0];
	else
		trans_len = pcfg->dsi_trans_len[1][1];

	IRIS_LOGD("%s(), trans_len is %d.", __func__, trans_len);

	if ((trans_len == 0) || (trans_len % split_cmd_len)) {
		/*fill last_command*/
		_iris_set_pkt_last(pcmd_comp->cmd, pcmd_comp->cnt,
			DSI_CMD_ONE_LAST_FOR_MULT_IPOPT);
	} else {
		_iris_set_pkt_last(pcmd_comp->cmd, pcmd_comp->cnt, 1);
		cont_pkt_len = 0;
		for (i = 0; i < pcmd_comp->cnt - 1; i++) {
			cmd_cur = pcmd_comp->cmd + i;
			cmd_next = pcmd_comp->cmd + i + 1;

			cont_pkt_len += cmd_cur->msg.tx_len;
			if (cont_pkt_len < trans_len && (cont_pkt_len + cmd_next->msg.tx_len) <= trans_len)
				_iris_update_desc_last(cmd_cur, 1, false);
			else
				cont_pkt_len = 0;
		}
	}

	if (IRIS_IF_LOGD()) {
		for (i = 0; i < pcmd_comp->cnt; i++) {
			cmd = pcmd_comp->cmd + i;
			IRIS_LOGD("i = %d, tx_len = %d, last = %d",
				i, cmd->msg.tx_len, cmd->last_command);
		}
	}

	/*use us than ms*/
	cmd = pcmd_comp->cmd + pcmd_comp->cnt - 1;
	wait = cmd->post_wait_ms;
	if (wait)
		cmd->post_wait_ms = 0;

	ret = iris_dsi_send_cmds(panel, pcmd_comp->cmd,
		pcmd_comp->cnt, pcmd_comp->link_state, pcfg->vc_ctrl.to_iris_vc_id);
	if (wait)
		udelay(wait);

	_iris_print_spec_cmds(pcmd_comp->cmd, pcmd_comp->cnt);
	_iris_print_dtsi_cmds_for_lk(pcmd_comp->cmd, pcmd_comp->cnt, wait, pcmd_comp->link_state);

	return ret;
}

static void __iris_mult_addr_pad(uint8_t **p, uint32_t *poff, uint32_t left_len)
{

	switch (left_len) {
	case 4:
		_iris_set_ocp_type(*p, 0x00000405);
		*p += 4;
		*poff += 4;
		break;
	case 8:
		_iris_set_ocp_type(*p, 0x00000805);
		_iris_set_ocp_base_addr(*p, 0x00000000);
		*p += 8;
		*poff += 8;
		break;
	case 12:
		_iris_set_ocp_type(*p, 0x00000c05);
		_iris_set_ocp_base_addr(*p, 0x00000000);
		_iris_set_ocp_first_val(*p, 0x00000000);
		*p += 12;
		*poff += 12;
		break;
	case 0:
		break;
	default:
		IRIS_LOGE("%s()%d, left len not aligh to 4.", __func__, __LINE__);
		break;
	}

}

static void __iris_mult_addr_sw_split(uint8_t **p, uint32_t *poff,
	uint32_t left_len, struct dsi_cmd_desc *pdesc)
{
	uint8_t *ptmp = NULL;
	uint32_t tmp_len, wc, ocp_header, base_addr;

	wc = pdesc->msg.tx_len;
	ocp_header = _iris_get_ocp_type(pdesc->msg.tx_buf);
	base_addr = _iris_get_ocp_base_addr(pdesc->msg.tx_buf);

	if (left_len%8 == 0)
		tmp_len = left_len - 4;
	else
		tmp_len = left_len;

	memcpy(*p, pdesc->msg.tx_buf, tmp_len);
	ocp_header &= 0xFF0000FF;
	ocp_header |= (tmp_len << 8) | 0x1;
	_iris_set_ocp_type(*p, ocp_header);
	*p += tmp_len;
	*poff += tmp_len;

	if (left_len%8 == 0)
		__iris_mult_addr_pad(p, poff, 4);

	ptmp = (uint8_t *)pdesc->msg.tx_buf;
	ptmp += tmp_len;
	ocp_header &= 0xFF0000FF;
	ocp_header |= (wc-tmp_len+4) << 8;

	_iris_set_ocp_type(*p, ocp_header);
	*p += 4;
	*poff += 4;
	memcpy(*p, ptmp, wc-tmp_len);
	*p += wc-tmp_len;
	*poff += wc-tmp_len;

}

static void __iris_mult_addr_bw_split(uint8_t **p, uint32_t *poff,
	uint32_t left_len, struct dsi_cmd_desc *pdesc)
{
	uint8_t *ptmp = NULL;
	uint32_t tmp_len, wc, ocp_header, base_addr;

	wc = pdesc->msg.tx_len;
	ocp_header = _iris_get_ocp_type(pdesc->msg.tx_buf);
	base_addr = _iris_get_ocp_base_addr(pdesc->msg.tx_buf);

	tmp_len = left_len;
	memcpy(*p, pdesc->msg.tx_buf, tmp_len);
	ocp_header &= 0xFF0000FF;
	ocp_header |= (tmp_len << 8) | 0x1;
	_iris_set_ocp_type(*p, ocp_header);
	*p += tmp_len;
	*poff += tmp_len;

	ptmp = (uint8_t *)pdesc->msg.tx_buf;
	ptmp += tmp_len;
	ocp_header &= 0xFF0000FF;
	ocp_header |= (wc-tmp_len+8) << 8;

	_iris_set_ocp_type(*p, ocp_header);
	_iris_set_ocp_base_addr(*p, base_addr + (tmp_len-8));
	*p += 8;
	*poff += 8;
	memcpy(*p, ptmp, wc-tmp_len);
	*p += wc-tmp_len;
	*poff += wc-tmp_len;

}

static int32_t _iris_dsi_mult_addr_send(struct dsi_panel *panel,
		struct iris_cmd_comp *pcmd_comp)
{
	int32_t ret = 0;
	int32_t link_state, i;
	uint32_t wait, wc, ocp_header, base_addr;
	uint32_t left_len, trans_len, trans_num = 0;
	uint32_t total_payload_len_prev, total_payload_len;
	uint8_t *ptr = NULL;
	uint32_t split_len = 256;
	const uint32_t pad_len = 12;
	struct dsi_cmd_desc *pdesc = NULL;
	struct mipi_dsi_msg *pmsg = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();


	if (!pcmd_comp) {
		IRIS_LOGE("%s(), cmd list is null.", __func__);
		return -EINVAL;
	}

	if (pcmd_comp->send_mode == DSI_NON_EMBEDDED_MODE) {
		if (pcmd_comp->op_type == IRIS_LIGHTUP_OP)
			trans_len = pcfg->dsi_trans_len[0][0];
		else
			trans_len = pcfg->dsi_trans_len[0][1];
		split_len = 256;
	} else {
		if (pcmd_comp->op_type == IRIS_LIGHTUP_OP)
			trans_len = pcfg->dsi_trans_len[2][0];
		else
			trans_len = pcfg->dsi_trans_len[2][1];
		split_len = pcfg->split_pkt_size + 8;
	}

	IRIS_LOGD("%s(%d), trans_len is %d, split_len is %d.",
		__func__, __LINE__, trans_len, split_len);

	total_payload_len_prev = 0;
	for (i = 0; i < pcmd_comp->cnt; i++) {
		pdesc = pcmd_comp->cmd + i;
		total_payload_len_prev += pdesc->msg.tx_len;
	}
	IRIS_LOGD("%s(%d), total_payload_len_prev is %d.",
		__func__, __LINE__, total_payload_len_prev);

	link_state = pcmd_comp->link_state;
	wait = pcmd_comp->cmd[pcmd_comp->cnt - 1].post_wait_ms;

	memset(pcfg->dsi_trans_buf, 0x00, NON_EMBEDDED_BUF_SIZE);
	total_payload_len = 0;
	ptr = pcfg->dsi_trans_buf;
	for (i = 0; i < pcmd_comp->cnt; i++) {
		pdesc = pcmd_comp->cmd + i;
		wc = pdesc->msg.tx_len;
		ocp_header = _iris_get_ocp_type(pdesc->msg.tx_buf);
		base_addr = _iris_get_ocp_base_addr(pdesc->msg.tx_buf);

		left_len = calc_space_left(split_len, total_payload_len);
		if (left_len <= pad_len)
			__iris_mult_addr_pad(&ptr, &total_payload_len, left_len);

		left_len = calc_space_left(split_len, total_payload_len);
		if (left_len >= wc) {
			memcpy(ptr, pdesc->msg.tx_buf, wc);
			ocp_header &= 0xFF0000FF;
			ocp_header |= (wc << 8) | 0x1;
			_iris_set_ocp_type(ptr, ocp_header);
			ptr += wc;
			total_payload_len += wc;
		} else {
			switch (ocp_header & 0xF) {
			case 0x4:
				__iris_mult_addr_sw_split(&ptr, &total_payload_len, left_len, pdesc);
				break;
			case 0x0:
			case 0xc:
				__iris_mult_addr_bw_split(&ptr, &total_payload_len, left_len, pdesc);
				break;
			default:
				IRIS_LOGE("%s(),%d, invalid cmd type.", __func__, __LINE__);
			}
		}
	}

	IRIS_LOGD("%s(),%d, total_payload_len after regroup is %d.", __func__, __LINE__, total_payload_len);

	if (pcmd_comp->send_mode == DSI_EMBEDDED_MA_MODE) {
		trans_num = (total_payload_len + split_len - 1) / split_len;
	} else {
		if (trans_len != 0) {
			trans_num = (total_payload_len + trans_len - 1)/trans_len;
			WARN_ON(trans_len > 0x1000000);
		} else {
			trans_num = 1;
			WARN_ON(total_payload_len > 0x1000000);
		}
	}

	IRIS_LOGD("%s(),%d, trans_len is %d, trans_num is %d.",
		__func__, __LINE__, trans_len, trans_num);

	WARN_ON(!trans_num);

	pdesc = vzalloc(trans_num * sizeof(struct dsi_cmd_desc));
	if (!pdesc) {
		IRIS_LOGE("%s(), failed to alloc desc cmd", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < trans_num; i++) {
		pdesc[i].post_wait_ms = 0;
		pdesc[i].last_command = 1;
		pmsg = &pdesc[i].msg;
		memcpy(pmsg, &pcmd_comp->cmd[0].msg, sizeof(struct mipi_dsi_msg));

		if (pcmd_comp->send_mode == DSI_EMBEDDED_MA_MODE) {
			if ((i == trans_num-1) && (total_payload_len%split_len))
				pmsg->tx_len = total_payload_len%split_len;
			else
				pmsg->tx_len = split_len;
			pmsg->tx_buf = pcfg->dsi_trans_buf + i*split_len;
		} else {
			if ((trans_len == 0) && (trans_num == 1)) {
				pmsg->tx_len = total_payload_len;
				pmsg->tx_buf = pcfg->dsi_trans_buf;
			} else {
				if ((i == trans_num-1) && (total_payload_len%trans_len))
					pmsg->tx_len = total_payload_len%trans_len;
				else
					pmsg->tx_len = trans_len;
				pmsg->tx_buf = pcfg->dsi_trans_buf + i*trans_len;
			}
		}
	}

	/*add last for embedded w/ ma mode*/
	if (pcmd_comp->send_mode == DSI_EMBEDDED_MA_MODE) {
		_iris_set_pkt_last(pdesc, trans_num, trans_len/split_len);
		if (IRIS_IF_LOGD()) {
			struct dsi_cmd_desc *cmd = NULL;

			for (i = 0; i < trans_num; i++) {
				cmd = pdesc + i;
				IRIS_LOGD("i = %d, tx_len = %d, last = %d",
					i, cmd->msg.tx_len, cmd->last_command);
			}
		}
		WARN_ON(trans_len%split_len != 0);
	}

	ret = iris_dsi_send_cmds(panel, pdesc,
			trans_num, link_state, pcfg->vc_ctrl.to_iris_vc_id);
	if (wait)
		udelay(wait);

	if (ret)
		IRIS_LOGE("%s(), [%d]transfer failed, ret = %d.", __func__, i, ret);

	_iris_print_total_cmds(pdesc, trans_num);
	_iris_print_total_cmds(pcmd_comp->cmd, pcmd_comp->cnt);

	vfree(pdesc);

	return ret;
}

static int32_t _iris_dsi_send_ocp_cmds(struct dsi_panel *panel,
		struct iris_cmd_comp *pcmd_comp)
{
	int32_t ret = 0;
	uint8_t dsi_trans_mode = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcmd_comp->op_type == IRIS_LIGHTUP_OP)
		dsi_trans_mode = pcfg->dsi_trans_mode[0];
	else
		dsi_trans_mode = pcfg->dsi_trans_mode[1];

	IRIS_LOGD("%s, op_type: %s, dsi_trans_mode: %s", __func__,
		pcmd_comp->op_type ? "pq update" : "light up",
		_iris_dsi_trans_mode_name(dsi_trans_mode));

	pcmd_comp->send_mode = dsi_trans_mode;

	switch (dsi_trans_mode) {
	case DSI_EMBEDDED_NO_MA_MODE:
		ret = _iris_dsi_single_addr_send(panel, pcmd_comp);
		break;
	case DSI_EMBEDDED_MA_MODE:
	case DSI_NON_EMBEDDED_MODE:
		ret = _iris_dsi_mult_addr_send(panel, pcmd_comp);
		break;
	default:
		IRIS_LOGW("%s(%d), invalid send mode [%d], using embedded w/o ma as default",
			__func__, __LINE__, dsi_trans_mode);
		ret = _iris_dsi_single_addr_send(panel, pcmd_comp);
		break;
	}

	return ret;

}

int32_t _iris_send_cmds(struct dsi_panel *panel,
		struct iris_cmd_comp *pcmd_comp, uint8_t path)
{
	int32_t ret = 0;

	IRIS_LOGD("%s,%d: path = %d", __func__, __LINE__, path);

	if (!pcmd_comp) {
		IRIS_LOGE("cmd list is null");
		return -EINVAL;
	}

	if (path == PATH_DSI)
		ret = _iris_dsi_send_ocp_cmds(panel, pcmd_comp);
	else if (path == PATH_I2C)
		ret = _iris_i2c_send_ocp_cmds(panel, pcmd_comp);
	else
		ret = -EINVAL;

	return ret;
}

static void _iris_send_panel_cmd(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmds)
{
	if (!cmds || !cmds->count) {
		IRIS_LOGE("cmds = %p or cmd_cnt = 0", cmds);
		return;
	}

	iris_pt_send_panel_cmd(panel, cmds);
}

int32_t iris_send_ipopt_cmds(int32_t ip, int32_t opt_id)
{
	int32_t rc = 0;
	struct iris_cmd_comp cmd_comp;
	struct dsi_cmd_desc *cmd;
	int32_t i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGD("%s(), i_p: %#x, opt: %#x.", __func__, ip, opt_id);
	rc = _iris_init_cmd_comp(ip, opt_id, &cmd_comp);
	if (rc) {
		IRIS_LOGE("%s(), can not find in seq for i_p: 0x%02x opt: 0x%02x.",
				__func__, ip, opt_id);
		return rc;
	}

	if (IRIS_IF_LOGVV()) {
		for (i = 0; i < cmd_comp.cnt; i++) {
			cmd = cmd_comp.cmd + i;
			IRIS_LOGD("%s(), i_p: %0x opt: %0x last: %d len: %zu",
			__func__, ip, opt_id, cmd->last_command, cmd->msg.tx_len);
		}
	}
	cmd_comp.op_type = IRIS_PQUPDATE_OP;
	return _iris_send_cmds(pcfg->panel, &cmd_comp, PATH_DSI);
}

/**********************************************
 * the API will only be called when suspend/resume and boot up.
 *
 ***********************************************/
static void _iris_send_spec_lut(uint8_t lut_table, uint8_t lut_idx)
{
	if (lut_table == AMBINET_HDR_GAIN
			|| lut_table == AMBINET_SDR2HDR_LUT)
		return;

	iris_send_lut(lut_table, lut_idx);
}

static void _iris_send_new_lut(uint8_t lut_table, uint8_t lut_idx)
{
	iris_send_lut(lut_table, lut_idx);
}

static void _iris_update_cmds(struct iris_cmd_comp *pcmd_comp,
		int32_t link_state, enum iris_op_type op)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	_iris_reset_out_cmds();

	memset(pcmd_comp, 0x00, sizeof(*pcmd_comp));
	pcmd_comp->cmd = pcfg->iris_cmds.iris_cmds_buf;
	pcmd_comp->link_state = link_state;
	pcmd_comp->cnt = pcfg->iris_cmds.cmds_index;
	pcmd_comp->op_type = op;
}

static void _iris_update_desc_last(struct dsi_cmd_desc *pcmd,
		int count, bool last_cmd)
{
	int i = 0;

	for (i = 0; i < count; i++)
		pcmd[i].last_command = last_cmd;
}

static void _iris_update_mult_pkt_last(struct dsi_cmd_desc *cmd,
		int cmd_cnt, uint32_t span)
{
	int i = 0;
	int num = 0;
	int tail = 0;

	num = cmd_cnt / span;
	tail = cmd_cnt % span;

	for (i = 0; i < num; i++) {
		_iris_update_desc_last(cmd + i*span, span - 1, false);
		_iris_update_desc_last(cmd + i*span + span - 1, 1, true);
	}

	if (tail) {
		_iris_update_desc_last(cmd + i*span, tail - 1, false);
		_iris_update_desc_last(cmd + i*span + tail - 1, 1, true);
	}
}

static int _iris_set_pkt_last(struct dsi_cmd_desc *cmd, int32_t cmd_cnt,
			uint32_t add_last_flag)
{
	if (add_last_flag == DSI_CMD_ONE_LAST_FOR_MULT_IPOPT) {
		_iris_update_desc_last(cmd, cmd_cnt, false);
		_iris_update_desc_last(cmd + cmd_cnt - 1, 1, true);
	} else {
		_iris_update_mult_pkt_last(cmd, cmd_cnt, add_last_flag);
	}

	return 0;
}

static int _iris_send_lut_pkt(
		struct iris_ctrl_opt *popt, struct iris_cmd_comp *pcomp,
		bool is_update, uint8_t path)
{
	int32_t cur = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	uint8_t ip = popt->ip;
	uint8_t opt_id = popt->opt_id;
	int32_t chain = popt->chain;
	int32_t prev = pcomp->cnt;

	IRIS_LOGD("%s(), i_p: %#x opt: %#x, chain: %d, update: %s",
			__func__, ip, opt_id, chain, is_update ? "true" : "false");

	pcfg->iris_cmds.cmds_index = prev;
	if (is_update)
		_iris_send_new_lut(ip, opt_id);
	else
		_iris_send_spec_lut(ip, opt_id);

	cur = pcfg->iris_cmds.cmds_index;
	if (cur == prev) {
		IRIS_LOGD("lut table is empty for i_p: %02x opt: %02x",
				popt->ip, opt_id);
		return 0;
	}

	pcomp->cnt = cur;

	if (!chain) {
		_iris_send_cmds(pcfg->panel, pcomp, path);
		_iris_update_cmds(pcomp, pcomp->link_state, pcomp->op_type);
	}

	return 0;
}

static int _iris_send_dtsi_pkt(
		struct iris_ctrl_opt *pip_opt, struct iris_cmd_comp *pcomp, uint8_t path)
{
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	int32_t prev = 0;
	int32_t cur = 0;
	int32_t chain = 0;
	int32_t rc = 0;
	struct iris_cfg *pcfg = NULL;
	struct iris_cmd_comp comp_priv;

	pcfg = iris_get_cfg();
	ip = pip_opt->ip;
	opt_id = pip_opt->opt_id;
	chain = pip_opt->chain;

	IRIS_LOGD("%s(), i_p: %#x opt: %#x, chain: %d.",
			__func__, ip, opt_id, chain);

	/*get single/multiple selection(s) according to option of ip*/
	rc = _iris_init_cmd_comp(ip, opt_id, &comp_priv);
	if (rc) {
		IRIS_LOGE("%s(), invalid i_p: %#x opt: %#x.", __func__, ip, opt_id);
		return -EINVAL;
	}

	if (pcomp->cnt == 0)
		pcomp->link_state = comp_priv.link_state;

	prev = pcomp->cnt;
	/*move single/multiples selection to one command*/

	memcpy(pcomp->cmd + pcomp->cnt, comp_priv.cmd,
			comp_priv.cnt * sizeof(*comp_priv.cmd));
	pcomp->cnt += comp_priv.cnt;

	cur = pcomp->cnt;

	/* if need to send or the last packet of sequence,
	 * it should send out to the MIPI
	 */
	if (!chain) {
		_iris_send_cmds(pcfg->panel, pcomp, path);
		_iris_update_cmds(pcomp, pcomp->link_state, pcomp->op_type);
	}

	return 0;
}

void iris_send_pkt(struct iris_ctrl_opt *arr, int seq_cnt,
		struct iris_cmd_comp *pcmd_comp)
{
	int i = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	int32_t rc = -1;

	for (i = 0; i < seq_cnt; i++) {
		ip = arr[i].ip;
		opt_id = arr[i].opt_id;
		IRIS_LOGV("%s(), i_p:%0x opt:%0x", __func__, ip, opt_id);

		/*lut table*/
		if (_iris_is_lut(ip))
			rc = _iris_send_lut_pkt(arr + i, pcmd_comp, false, PATH_DSI);
		else
			rc = _iris_send_dtsi_pkt(arr + i, pcmd_comp, PATH_DSI);

		if (rc)
			IRIS_LOGE("%s(), [FATAL ERROR] invalid i_p: %0x opt: %0x", __func__, ip, opt_id);
	}
}

void iris_send_assembled_pkt(struct iris_ctrl_opt *arr, int seq_cnt)
{
	struct iris_cmd_comp cmd_comp;

	_iris_update_cmds(&cmd_comp, DSI_CMD_SET_STATE_HS, IRIS_LIGHTUP_OP);

	iris_send_pkt(arr, seq_cnt, &cmd_comp);
}

static void _iris_send_lightup_pkt(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_ctrl_seq *pseq = _iris_get_ctrl_seq(pcfg);

	if (!pseq) {
		IRIS_LOGE("%s(), invalid pseq", __func__);
		return;
	}

	SDE_ATRACE_BEGIN("_iris_send_lightup_pkt");
	iris_send_assembled_pkt(pseq->ctrl_opt, pseq->cnt);
	SDE_ATRACE_END("_iris_send_lightup_pkt");
}

void iris_init_update_ipopt(
		struct iris_update_ipopt *popt, uint8_t ip,
		uint8_t opt_old, uint8_t opt_new, uint8_t chain)
{
	popt->ip = ip;
	popt->opt_old = opt_old;
	popt->opt_new = opt_new;
	popt->chain = chain;
}

int iris_init_update_ipopt_t(uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t chain)
{
	uint32_t index = 0;
	uint32_t max_cnt = 0;
	struct iris_update_ipopt *popt = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	index = pcfg->pq_update_cmd.array_index;
	max_cnt = pcfg->ip_opt_cnt;

	if (index >= max_cnt) {
		IRIS_LOGE("%s(), no empty space to install i_p: %#x, opt old: %#x, opt new: %#x",
				__func__, ip, opt_old, opt_new);
		return -EINVAL;
	}

	popt = pcfg->pq_update_cmd.update_ipopt_array + index;
	pcfg->pq_update_cmd.array_index++;

	iris_init_update_ipopt(popt, ip, opt_old, opt_new, chain);

	return (index + 1);
}

static int _iris_read_chip_id(void)
{
	uint32_t sys_pll_ro_status = 0xf0000010;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->chip_value[0] != 0)
		pcfg->chip_value[1] = iris_ocp_read(pcfg->chip_value[0], DSI_CMD_SET_STATE_HS);

	// FIXME: if chip version is set by sw, skip hw read chip id.
	if (pcfg->chip_ver == IRIS3_CHIP_VERSION)
		pcfg->chip_id = (iris_ocp_read(sys_pll_ro_status, DSI_CMD_SET_STATE_HS)) & 0xFF;
	else
		pcfg->chip_id = 0;

	IRIS_LOGI("%s(), chip version: %#x, chip id: %#x",
			__func__, pcfg->chip_ver, pcfg->chip_id);

	return pcfg->chip_id;
}

static void _iris_clean_status(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->metadata = 0;	// clean metadata
}

void iris_free_ipopt_buf(uint32_t ip_type)
{
	int ip_index = 0;
	int opt_index = 0;
	uint32_t desc_index = 0;
	int ip_cnt = IRIS_IP_CNT;
	struct dsi_cmd_desc *pdesc_addr = NULL;
	struct iris_ip_index *pip_index = iris_get_ip_idx(ip_type);

	if (ip_type == IRIS_LUT_PIP_IDX)
		ip_cnt = LUT_IP_END - LUT_IP_START;

	for (ip_index = 0; ip_index < ip_cnt; ip_index++) {
		if (pip_index[ip_index].opt_cnt == 0 || pip_index[ip_index].opt == NULL)
			continue;

		for (opt_index = 0; opt_index < pip_index[ip_index].opt_cnt; opt_index++) {
			if (pip_index[ip_index].opt[opt_index].cmd_cnt == 0
					|| pip_index[ip_index].opt[opt_index].cmd == NULL)
				continue;

			/* get desc cmd start address */
			if (pdesc_addr == NULL || pip_index[ip_index].opt[opt_index].cmd < pdesc_addr)
				pdesc_addr = pip_index[ip_index].opt[opt_index].cmd;

			for (desc_index = 0; desc_index < pip_index[ip_index].opt[opt_index].cmd_cnt; desc_index++) {
				if (pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_buf == NULL
						|| pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_len == 0)
					continue;

				/* free cmd payload, which alloc in "_iris_write_cmd_payload()" */
				vfree(pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_buf);
				pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_buf = NULL;
			}

			/* set each desc cmd to NULL first */
			pip_index[ip_index].opt[opt_index].cmd = NULL;
		}

		/* free opt buffer for each ip, which alloc in "_iris_alloc_pip_buf()" */
		kvfree(pip_index[ip_index].opt);
		pip_index[ip_index].opt = NULL;
		pip_index[ip_index].opt_cnt = 0;
	}

	/* free desc cmd buffer, which alloc in "_iris_alloc_desc_buf()", desc
	 * cmd buffer is continus memory, so only free once on start address
	 */
	if (pdesc_addr != NULL) {
		IRIS_LOGI("%s(), free desc cmd buffer %p, type %#x", __func__, pdesc_addr, ip_type);
		vfree(pdesc_addr);
		pdesc_addr = NULL;
	}
}

void iris_free_seq_space(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	/* free cmd to sent buffer, which alloc in "iris_alloc_seq_space()" */
	if (pcfg->iris_cmds.iris_cmds_buf != NULL) {
		IRIS_LOGI("%s()%d, free %p", __func__, __LINE__, pcfg->iris_cmds.iris_cmds_buf);
		vfree(pcfg->iris_cmds.iris_cmds_buf);
		pcfg->iris_cmds.iris_cmds_buf = NULL;
	}

	if (pcfg->dsi_trans_buf != NULL) {
		IRIS_LOGI("%s()%d, free %p", __func__, __LINE__, pcfg->dsi_trans_buf);
		vfree(pcfg->dsi_trans_buf);
		pcfg->dsi_trans_buf = NULL;
	}

}

void iris_alloc_seq_space(void)
{
	uint8_t *buf = NULL;
	struct dsi_cmd_desc *pdesc = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	int sum = pcfg->dtsi_cmds_cnt + pcfg->lut_cmds_cnt;

	IRIS_LOGI("%s(), dtsi = %u, lut = %u", __func__,
			pcfg->dtsi_cmds_cnt, pcfg->lut_cmds_cnt);

	sum *= sizeof(struct dsi_cmd_desc);
	pdesc = vmalloc(sum);
	if (!pdesc) {
		IRIS_LOGE("%s(), failed to alloc buffer", __func__);
		return;
	}
	pcfg->iris_cmds.iris_cmds_buf = pdesc;

	IRIS_LOGI("%s()%d, alloc %p", __func__, __LINE__, pcfg->iris_cmds.iris_cmds_buf);
	_iris_reset_out_cmds();

	buf = vmalloc(NON_EMBEDDED_BUF_SIZE);
	if (!buf) {
		vfree(pcfg->iris_cmds.iris_cmds_buf);
		pcfg->iris_cmds.iris_cmds_buf = NULL;
		IRIS_LOGE("%s(), failed to alloc non embedded buf", __func__);
		return;
	}
	pcfg->dsi_trans_buf = buf;
	IRIS_LOGI("%s()%d, alloc %p", __func__, __LINE__, pcfg->dsi_trans_buf);

	// Need to init PQ parameters here for video panel.
	iris_pq_parameter_init();
}

void iris_alloc_update_ipopt_space(void)
{
	struct iris_update_ipopt *p = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	int sum = 0, dtsi_opt_cnt = 0;
	int i = 0, j = 0;
	struct iris_ip_index *pip_index = NULL;

	for (i = IRIS_DTSI_PIP_IDX_START; i < iris_get_cmd_list_cnt(); i++) {
		pip_index = iris_get_ip_idx(i);
		dtsi_opt_cnt = 0;
		for (j = 0; j < IRIS_IP_CNT; j++)
			dtsi_opt_cnt += pip_index[j].opt_cnt;
	}
	if (dtsi_opt_cnt > sum)
		sum = dtsi_opt_cnt;

	pip_index = iris_get_ip_idx(IRIS_LUT_PIP_IDX);
	for (j = 0; j < LUT_IP_END - LUT_IP_START; j++)
		sum += pip_index[j].opt_cnt;

	pcfg->ip_opt_cnt = sum;
	IRIS_LOGI("%s(), i_p opt count: %u", __func__, pcfg->ip_opt_cnt);

	p = kcalloc(sum, sizeof(struct iris_update_ipopt), GFP_KERNEL);
	if (!p) {
		IRIS_LOGE("%s(), failed to allo update ipopt space", __func__);
		return;
	}

	pcfg->pq_update_cmd.update_ipopt_array = p;

	IRIS_LOGI("%s(), alloc %p", __func__, pcfg->pq_update_cmd.update_ipopt_array);

	iris_init_ipopt_t();

}

static void _iris_load_mcu(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 values[2] = {0xF00000C8, 0x1};
	struct iris_ctrl_opt ctrl_opt;

	IRIS_LOGI("%s(%d): load mcu, mcu_sw_reset:%d", __func__, __LINE__, values[1]);
	ctrl_opt.ip = APP_CODE_LUT;
	ctrl_opt.opt_id = 0;
	ctrl_opt.chain = 0;

	iris_send_assembled_pkt(&ctrl_opt, 1);
	iris_ocp_write_mult_vals(2, values);
	pcfg->mcu_code_downloaded = true;
}

static void _iris_pre_rx_cmd(void)
{
	/*send rx cmds first with low power*/
	iris_send_ipopt_cmds(IRIS_IP_RX, 0xF1);
}

static void _iris_pre_lightup(void)
{
	_iris_pre_rx_cmd();

	/*read chip_id*/
	_iris_read_chip_id();

	iris_pq_parameter_init();
	iris_frc_timing_setting_update();
	_iris_clean_status();
}

void _iris_read_power_mode(struct dsi_panel *panel)
{
	char get_power_mode[1] = {0x0a};
	char read_cmd_rbuf[16] = {0};
	struct dsi_cmd_desc_pxlw cmds_pxlw = {
		{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0, 0,
		sizeof(get_power_mode), get_power_mode, 1, read_cmd_rbuf}, 1, 0};
	struct dsi_cmd_desc cmds;
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &cmds,
	};
	struct iris_cfg *pcfg = iris_get_cfg();

	remap_to_qcom_style(&cmds, &cmds_pxlw, 1);

	iris_set_msg_flags(&cmds, READ_FLAG);
	IRIS_LOGI("%s(%d), abyp mode: %d", __func__, __LINE__,
			pcfg->abyp_ctrl.abypass_mode);
	if (pcfg->abyp_ctrl.abypass_mode == ANALOG_BYPASS_MODE) {
		iris_abyp_send_panel_cmd(panel, &cmdset);
	} else {
		iris_pt_send_panel_cmd(panel, &cmdset);
		IRIS_LOGI("[a]power mode: 0x%02x", read_cmd_rbuf[0]);
		read_cmd_rbuf[0] = 0;
		_iris_send_panel_cmd(panel, &cmdset);
	}
	pcfg->power_mode = read_cmd_rbuf[0];

	IRIS_LOGI("%s(), power mode: 0x%02x", __func__, pcfg->power_mode);
}

void iris_fpga_type_get(void)
{
#if defined(CONFIG_PXLW_FPGA_IRIS)
	int rc = 1;
	struct iris_cfg *pcfg = iris_get_cfg();
	char payload[1] = {0x06};
	char rbuf[16] = {0};
	struct dsi_cmd_desc_pxlw cmds_pxlw = {
		{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0, 0,
			sizeof(payload), payload, 1, rbuf}, 1, 0};
	struct dsi_cmd_desc cmds;
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &cmds,
	};

	remap_to_qcom_style(&cmds, &cmds_pxlw, 1);

	rc = iris_dsi_send_cmds(pcfg->panel, cmdset.cmds, cmdset.count,
							cmdset.state, pcfg->vc_ctrl.to_iris_vc_id);

	if (!rc)
		pcfg->proFPGA_detected = rbuf[0] & 0x1;
	IRIS_LOGI("%s(), fpga type: %d", __func__, pcfg->proFPGA_detected);
#endif
}

int iris_lightup(struct dsi_panel *panel)
{
	ktime_t ktime0;
	ktime_t ktime1;
	uint32_t timeus0 = 0;
	uint32_t timeus1 = 0;
	uint8_t type = 0;
	struct dsi_display *display = to_dsi_display(panel->host);
	int rc;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(), start +++, cmd list index: %u",
			__func__,
			iris_get_cmd_list_index());

	SDE_ATRACE_BEGIN("iris_lightup");
	ktime0 = ktime_get();
	rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_ON);
	if (rc) {
		IRIS_LOGE("%s(), failed to enable all DSI clocks for display: %s, return: %d",
				__func__, display->name, rc);
	}

	rc = iris_display_cmd_engine_enable(display);
	if (rc) {
		IRIS_LOGE("%s(), failed to enable cmd engine for display: %s, return: %d",
				__func__, display->name, rc);
	}

	_iris_pre_lightup();
	_iris_load_mcu();

	type = iris_get_cont_splash_type();

	/*use to debug cont splash*/
	if (type == IRIS_CONT_SPLASH_LK) {
		IRIS_LOGI("%s(%d), enter cont splash", __func__, __LINE__);
		_iris_send_cont_splash_pkt(IRIS_CONT_SPLASH_LK);
	} else {
		_iris_send_lightup_pkt();
		iris_update_gamma();
		iris_ioinc_filter_ratio_send();
	}


	if (&(panel->cur_mode->priv_info->cmd_sets[DSI_CMD_SET_PRE_ON]) != NULL) {
		SDE_ATRACE_BEGIN("iris_pt_send_panel_cmd");
		rc = iris_pt_send_panel_cmd(panel, &(panel->cur_mode->priv_info->cmd_sets[DSI_CMD_SET_PRE_ON]));
		SDE_ATRACE_END("iris_pt_send_panel_cmd");
		IRIS_LOGI("%s(%d), pre_on_cmds", __func__, __LINE__);
	}

	ktime1 = ktime_get();
	if (type == IRIS_CONT_SPLASH_LK)
		IRIS_LOGI("%s(), exit cont splash", __func__);
	else
		/*continuous splahs should not use dma setting low power*/
		iris_lp_enable_post();

	iris_tx_buf_to_vc_set(pcfg->vc_ctrl.vc_arr[VC_PT]);

	rc = iris_display_cmd_engine_disable(display);
	if (rc) {
		IRIS_LOGE("%s(), failed to disable cmd engine for display: %s, return: %d",
				__func__, display->name, rc);
	}
	rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_OFF);
	if (rc) {
		IRIS_LOGE("%s(), failed to disable all DSI clocks for display: %s, return: %d",
				__func__, display->name, rc);
	}

	iris_update_last_pt_timing(&panel->cur_mode->timing);
	iris_sdr2hdr_set_img_size(panel->cur_mode->timing.h_active,
			panel->cur_mode->timing.v_active);

	SDE_ATRACE_END("iris_lightup");

	timeus0 = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
	timeus1 = (u32) ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime1);
	IRIS_LOGI("%s() spend time0 %d us, time1 %d us.",
			__func__, timeus0, timeus1);

#ifdef IRIS_MIPI_TEST
	_iris_read_power_mode(panel);
#endif
	pcfg->abyp_ctrl.preloaded = true;
	IRIS_LOGI("%s(), end +++", __func__);

	return 0;
}

void iris_update_2nd_active_timing(struct dsi_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (panel == NULL || pcfg == NULL)
		return;

	if (panel->is_secondary && pcfg->ap_mipi1_power_st) {
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

int iris_enable(struct dsi_panel *panel, struct dsi_panel_cmd_set *on_cmds)
{
	int rc = 0;
	int abyp_status_gpio;
	int prev_mode;
	int lightup_opt = iris_lightup_opt_get();
	struct iris_cfg *pcfg = iris_get_cfg();
	ktime_t ktime0 = 0;
	ktime_t ktime1 = 0;
	ktime_t ktime2 = 0;
	ktime_t ktime3 = 0;
	ktime_t ktime4 = 0;
	uint32_t timeus0 = 0;
	uint32_t timeus1 = 0;
	uint32_t timeus2 = 0;
	uint32_t timeus3 = 0;
	uint32_t timeus4 = 0;


	if (pcfg->valid < PARAM_PREPARED) {
		if (on_cmds != NULL)
			rc = iris_abyp_send_panel_cmd(panel, on_cmds);
		goto end;
	}

	IRIS_LOGI_IF(panel != NULL,
			"%s(), for %s %s, secondary: %s",
			__func__,
			panel->name, panel->type,
			panel->is_secondary ? "true" : "false");

	if (IRIS_IF_LOGI())
		ktime0 = ktime_get();
	if (panel->is_secondary) {
		if (pcfg->iris_osd_autorefresh_enabled) {
			IRIS_LOGI("reset iris_osd_autorefresh");
			iris_osd_auto_refresh_enable(0);
		}
		iris_mipi1_enable();
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
		goto end;
	}

	SDE_ATRACE_BEGIN("iris_enable");
	iris_update_panel_timing(&panel->cur_mode->timing);
	iris_lp_enable_pre();

	/* Force Iris work in ABYP mode */
	if (iris_is_abyp_timing(&panel->cur_mode->timing))
		pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE;

	IRIS_LOGI("%s(), mode:%d, rate: %d, v: %d, on_opt:0x%x",
			__func__,
			pcfg->abyp_ctrl.abypass_mode,
			panel->cur_mode->timing.refresh_rate,
			panel->cur_mode->timing.v_active,
			lightup_opt);

	/* support lightup_opt */
	if (lightup_opt & 0x1) {
		if (on_cmds != NULL)
			rc = iris_abyp_send_panel_cmd(panel, on_cmds);
		IRIS_LOGI("%s(), force ABYP lightup.", __func__);
		SDE_ATRACE_END("iris_enable");
		goto end;
	}

	if (IRIS_IF_LOGI())
		ktime1 = ktime_get();

	_iris_bulksram_power_domain_proc();

	if (iris_is_sleep_abyp_mode()) {
		_iris_disable_temp_sensor();
		iris_sleep_abyp_power_down();

		if (IRIS_IF_LOGI())
			ktime2 = ktime_get();
		if (IRIS_IF_LOGI()) {
			timeus0 = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
			timeus1 = (u32) ktime_to_us(ktime2) - (u32)ktime_to_us(ktime1);
		}
		IRIS_LOGI("%s(), iris takes total %d us, prepare %d us, low power %d us",
				__func__, timeus0 + timeus1, timeus0, timeus1);
	} else {
		prev_mode = pcfg->abyp_ctrl.abypass_mode;

		abyp_status_gpio = iris_exit_abyp(true);
		if (abyp_status_gpio == 1) {
			IRIS_LOGE("%s(), failed to exit abyp!", __func__);
			SDE_ATRACE_END("iris_enable");
			goto end;
		}

		if (IRIS_IF_LOGI())
			ktime2 = ktime_get();

#if defined(CONFIG_PXLW_FPGA_IRIS)
		if (iris_platform_get() == IRIS_FPGA)
			iris_fpga_type_get();
#endif
		rc = iris_lightup(panel);
		pcfg->abyp_ctrl.abypass_mode = PASS_THROUGH_MODE;

		if (IRIS_IF_LOGI())
			ktime3 = ktime_get();

		if (on_cmds != NULL) {
			SDE_ATRACE_BEGIN("iris_pt_send_panel_cmd");
			rc = iris_pt_send_panel_cmd(panel, on_cmds);
			SDE_ATRACE_END("iris_pt_send_panel_cmd");
		}

		if (IRIS_IF_LOGI())
			ktime4 = ktime_get();

		//Switch back to ABYP mode if need
		if ((iris_platform_get() != IRIS_FPGA) && !(iris_lightup_opt_get() & 0x2)) {
			if (prev_mode == ANALOG_BYPASS_MODE)
				iris_abyp_switch_proc(pcfg->display, ANALOG_BYPASS_MODE);
		}

		if (IRIS_IF_LOGI()) {
			timeus0 = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
			timeus1 = (u32) ktime_to_us(ktime2) - (u32)ktime_to_us(ktime1);
			timeus2 = (u32) ktime_to_us(ktime3) - (u32)ktime_to_us(ktime2);
			timeus3 = (u32) ktime_to_us(ktime4) - (u32)ktime_to_us(ktime3);
			timeus4 = (u32) ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime4);
		}
		IRIS_LOGI("%s(), iris takes total %d us, prepare %d us, enter PT %d us,"
				" light up %d us, exit PT %d us.",
				__func__,
				timeus0 + timeus1 + timeus2 + timeus4,
				timeus0, timeus1, timeus2, timeus4);
		if (on_cmds != NULL) {
			IRIS_LOGI("Send panel cmd takes %d us.", timeus3);
		}
	}
	SDE_ATRACE_END("iris_enable");

end:
#ifdef IRIS_EXT_CLK
	iris_clk_disable(panel);
#endif
	return rc;
}

int iris_set_aod(struct dsi_panel *panel, bool aod)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg)
		return rc;

	if (panel->is_secondary)
		return rc;

	IRIS_LOGI("%s(%d), aod: %d", __func__, __LINE__, aod);
	if (pcfg->aod == aod) {
		IRIS_LOGI("[%s:%d] aod: %d no change", __func__, __LINE__, aod);
		return rc;
	}

	if (aod) {
		if (!pcfg->fod) {
			pcfg->abyp_prev_mode = pcfg->abyp_ctrl.abypass_mode;
			if (iris_get_abyp_mode(panel) == PASS_THROUGH_MODE)
				iris_abyp_switch_proc(pcfg->display, ANALOG_BYPASS_MODE);
		}
	} else {
		if (!pcfg->fod) {
			if (iris_get_abyp_mode(panel) == ANALOG_BYPASS_MODE &&
					pcfg->abyp_prev_mode == PASS_THROUGH_MODE &&
					!pcfg->fod) {
				iris_abyp_switch_proc(pcfg->display, PASS_THROUGH_MODE);
			}
		}
	}

	if (pcfg->fod_pending)
		pcfg->fod_pending = false;
	pcfg->aod = aod;

	return rc;
}

int iris_set_fod(struct dsi_panel *panel, bool fod)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg)
		return rc;

	if (panel->is_secondary)
		return rc;

	IRIS_LOGD("%s(%d), fod: %d", __func__, __LINE__, fod);
	if (pcfg->fod == fod) {
		IRIS_LOGD("%s(%d), fod: %d no change", __func__, __LINE__, fod);
		return rc;
	}

	if (!dsi_panel_initialized(panel)) {
		IRIS_LOGD("%s(%d), panel is not initialized fod: %d", __func__, __LINE__, fod);
		pcfg->fod_pending = true;
		atomic_set(&pcfg->fod_cnt, 1);
		pcfg->fod = fod;
		return rc;
	}

	if (fod) {
		if (!pcfg->aod) {
			pcfg->abyp_prev_mode = pcfg->abyp_ctrl.abypass_mode;
			if (iris_get_abyp_mode(panel) == PASS_THROUGH_MODE)
				iris_abyp_switch_proc(pcfg->display, ANALOG_BYPASS_MODE);
		}
	} else {
		/* pending until hbm off cmds sent in update_hbm 1->0 */
		pcfg->fod_pending = true;
		atomic_set(&pcfg->fod_cnt, 1);
	}

	pcfg->fod = fod;

	return rc;
}

int iris_post_fod(struct dsi_panel *panel)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg)
		return rc;

	if (panel->is_secondary)
		return rc;

	if (atomic_read(&pcfg->fod_cnt) > 0) {
		IRIS_LOGD("%s(%d), fod delay %d", __func__, __LINE__, atomic_read(&pcfg->fod_cnt));
		atomic_dec(&pcfg->fod_cnt);
		return rc;
	}

	IRIS_LOGD("%s(%d), fod: %d", __func__, __LINE__, pcfg->fod);

	if (pcfg->fod) {
		if (!pcfg->aod) {
			pcfg->abyp_prev_mode = pcfg->abyp_ctrl.abypass_mode;
			if (iris_get_abyp_mode(panel) == PASS_THROUGH_MODE)
				iris_abyp_switch_proc(pcfg->display, ANALOG_BYPASS_MODE);
		}
	} else {
		if (!pcfg->aod) {
			if (iris_get_abyp_mode(panel) == ANALOG_BYPASS_MODE &&
					pcfg->abyp_prev_mode == PASS_THROUGH_MODE) {
				iris_abyp_switch_proc(pcfg->display, PASS_THROUGH_MODE);
			}
		}
	}

	pcfg->fod_pending = false;

	return rc;
}

bool iris_get_aod(struct dsi_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!panel || !pcfg)
		return false;

	return pcfg->aod;
}

static void _iris_clear_aod_state(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->aod) {
		pcfg->aod = false;
		pcfg->abyp_ctrl.abypass_mode = pcfg->abyp_prev_mode;
	}
}

/*check whether it is in initial cont-splash packet*/
static bool _iris_check_cont_splash_ipopt(uint8_t ip, uint8_t opt_id)
{
	int i = 0;
	uint8_t cs_ip = 0;
	uint8_t cs_opt_id = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_ctrl_seq *pseq_cs = _iris_get_ctrl_seq_cs(pcfg);

	if (!pseq_cs) {
		IRIS_LOGE("%s(), invalid pseq_cs", __func__);
		return false;
	}

	for (i = 0; i < pseq_cs->cnt; i++) {
		cs_ip = pseq_cs->ctrl_opt[i].ip;
		cs_opt_id = pseq_cs->ctrl_opt[i].opt_id;

		if (ip == cs_ip && opt_id == cs_opt_id)
			return true;
	}

	return false;
}

/*select ip/opt to the opt_arr according to lightup stage type*/
static int _iris_select_cont_splash_ipopt(
		int type, struct iris_ctrl_opt *opt_arr)
{
	uint32_t i = 0;
	uint32_t j = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_ctrl_seq *pseq = _iris_get_ctrl_seq(pcfg);
	struct iris_ctrl_opt *pctrl_opt = NULL;

	if (!pseq) {
		IRIS_LOGE("%s(), invalid pseq", __func__);
		return -EINVAL;
	}

	for (i = 0; i < pseq->cnt; i++) {
		pctrl_opt = pseq->ctrl_opt + i;
		ip = pctrl_opt->ip;
		opt_id = pctrl_opt->opt_id;

		if (_iris_check_cont_splash_ipopt(ip, opt_id))
			continue;

		memcpy(opt_arr + j, pctrl_opt, sizeof(*pctrl_opt));
		j++;
	}

	IRIS_LOGD("%s(), real len: %d", __func__, j);
	return j;
}

static void _iris_send_cont_splash_pkt(uint32_t type)
{
	int seq_cnt = 0;
	uint32_t size = 0;
	const int iris_max_opt_cnt = 30;
	struct iris_ctrl_opt *opt_arr = NULL;
	struct iris_cfg *pcfg = NULL;
	struct iris_ctrl_seq *pseq_cs = NULL;

	size = IRIS_IP_CNT * iris_max_opt_cnt * sizeof(struct iris_ctrl_opt);
	opt_arr = vmalloc(size);
	if (opt_arr == NULL) {
		IRIS_LOGE("%s(), failed to malloc buffer!", __func__);
		return;
	}

	pcfg = iris_get_cfg();
	memset(opt_arr, 0xff, size);

	if (type == IRIS_CONT_SPLASH_LK) {
		pseq_cs = _iris_get_ctrl_seq_cs(pcfg);
		if (!pseq_cs) {
			IRIS_LOGE("%s() invalid pseq_cs", __func__);
			vfree(opt_arr);
			return;
		}
		iris_send_assembled_pkt(pseq_cs->ctrl_opt, pseq_cs->cnt);
	} else if (type == IRIS_CONT_SPLASH_KERNEL) {
		iris_lp_enable_pre();
		seq_cnt = _iris_select_cont_splash_ipopt(type, opt_arr);
		iris_send_assembled_pkt(opt_arr, seq_cnt);
		iris_lp_enable_post();
		_iris_read_chip_id();
	} else if (type == IRIS_CONT_SPLASH_BYPASS_PRELOAD) {
		iris_enable(pcfg->panel, NULL);
	}

	vfree(opt_arr);
}

void iris_send_cont_splash(struct dsi_display *display)
{
	struct dsi_panel *panel = display->panel;
	struct iris_cfg *pcfg = iris_get_cfg();
	int lightup_opt = iris_lightup_opt_get();
	uint32_t type;
	int rc = 0;

	if (!iris_is_chip_supported())
		return;

	if (panel->is_secondary)
		return;
#ifdef IRIS_EXT_CLK //skip ext clk
	iris_clk_enable(panel);
#endif
	rc = iris_set_pinctrl_state(true);
	if (rc) {
		IRIS_LOGE("%s() failed to set iris pinctrl, rc=%d\n", __func__, rc);
		return;
	}

	type = iris_get_cont_type_with_timing_switch(panel);

	if (lightup_opt & 0x1)
		type = IRIS_CONT_SPLASH_NONE;

	mutex_lock(&pcfg->panel->panel_lock);
	_iris_send_cont_splash_pkt(type);
	mutex_unlock(&pcfg->panel->panel_lock);
}

int iris_lightoff(struct dsi_panel *panel, bool dead,
		struct dsi_panel_cmd_set *off_cmds)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int lightup_opt = iris_lightup_opt_get();

	if (pcfg->valid < PARAM_PREPARED) {
		if (panel && !panel->is_secondary && off_cmds)
			iris_abyp_send_panel_cmd(panel, off_cmds);
		return 0;
	}

	pcfg->mipi2_pwr_st = false;
	pcfg->metadata = 0; // clean metadata
	pcfg->dtg_ctrl_pt = 0;
	iris_memc_status_clear(true);
	iris_dual_status_clear(true);

	if (!panel || panel->is_secondary) {
		IRIS_LOGD("no need to light off for 2nd panel.");
		return 0;
	}

	if ((lightup_opt & 0x10) == 0)
		pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE; //clear to ABYP mode

	IRIS_LOGI("%s(%d), panel %s, mode: %s(%d) ---", __func__, __LINE__,
			dead ? "dead" : "alive",
			pcfg->abyp_ctrl.abypass_mode == PASS_THROUGH_MODE ? "PT" : "ABYP",
			pcfg->abyp_ctrl.abypass_mode);
	if (off_cmds && (!dead)) {
		if (pcfg->abyp_ctrl.abypass_mode == PASS_THROUGH_MODE)
			iris_pt_send_panel_cmd(panel, off_cmds);
		else
			iris_abyp_send_panel_cmd(panel, off_cmds);
	}
	iris_quality_setting_off();
	iris_lp_setting_off();
	iris_pwil0_efifo_setting_reset();
	iris_dport_output_mode_reset();
	iris_pt_sr_reset();
	iris_emv_on_lightoff();
	iris_dtg_update_reset();
	iris_set_pwil_mode(PT_MODE, false, DSI_CMD_SET_STATE_HS, false);
	iris_osd_comp_ready_pad_select(false, false);
	_iris_clear_aod_state();
	pcfg->panel_pending = 0;
	iris_set_pinctrl_state(false);
	iris_mcu_mode_reset();

	IRIS_LOGI("%s(%d) ---", __func__, __LINE__);

	return 0;
}

int iris_disable(struct dsi_panel *panel, bool dead, struct dsi_panel_cmd_set *off_cmds)
{
	return iris_lightoff(panel, dead, off_cmds);
}

static void _iris_send_update_opt(
		struct iris_update_ipopt *popt,
		struct iris_cmd_comp *pasm_comp, uint8_t path)
{
	int32_t ip = 0;
	int32_t rc = 0;
	struct iris_ctrl_opt ctrl_opt;

	ip = popt->ip;
	ctrl_opt.ip = popt->ip;
	ctrl_opt.opt_id = popt->opt_new;
	ctrl_opt.chain = popt->chain;

	/*speical deal with lut table*/
	if (_iris_is_lut(ip))
		rc = _iris_send_lut_pkt(&ctrl_opt, pasm_comp, true, path);
	else
		rc = _iris_send_dtsi_pkt(&ctrl_opt, pasm_comp, path);

	if (rc) {
		IRIS_LOGE("%s(), [FATAL ERROR] invalid i_p: %#x, opt: %#x",
				__func__,
				ip, ctrl_opt.opt_id);
	}
}

static void _iris_send_pq_cmds(struct iris_update_ipopt *popt, int ipopt_cnt, uint8_t path)
{
	int32_t i = 0;
	struct iris_cmd_comp cmd_comp;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!popt || !ipopt_cnt) {
		IRIS_LOGE("%s(), invalid popt or ipopt_cnt", __func__);
		return;
	}

	memset(&cmd_comp, 0x00, sizeof(cmd_comp));
	cmd_comp.cmd =  pcfg->iris_cmds.iris_cmds_buf;
	cmd_comp.link_state = DSI_CMD_SET_STATE_HS;
	cmd_comp.cnt = pcfg->iris_cmds.cmds_index;
	cmd_comp.op_type = IRIS_PQUPDATE_OP;

	for (i = 0; i < ipopt_cnt; i++)
		_iris_send_update_opt(&popt[i], &cmd_comp, path);
}

static int _iris_update_pq_seq(struct iris_update_ipopt *popt, int ipopt_cnt)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t ip = 0;
	int32_t opt_id = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_ctrl_seq *pseq = _iris_get_ctrl_seq(pcfg);

	if (!pseq) {
		IRIS_LOGE("%s(), invalid pseq", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ipopt_cnt; i++) {
		/*need to update sequence*/
		if (popt[i].opt_new != popt[i].opt_old) {
			for (j = 0; j < pseq->cnt; j++) {
				ip = pseq->ctrl_opt[j].ip;
				opt_id = pseq->ctrl_opt[j].opt_id;

				if ((ip == popt[i].ip) && (ip == GAMMA_LUT))
					break;

				if (ip == popt[i].ip &&
						opt_id == popt[i].opt_old)
					break;
			}

			if (j == pseq->cnt) {
				IRIS_LOGE("%s(), failed to find i_p: %#x opt: %d",
						__func__,
						popt[i].ip, popt[i].opt_old);
				return -EINVAL;
			}

			pseq->ctrl_opt[j].opt_id = popt[i].opt_new;
		}
	}

	return 0;
}

void iris_update_pq_opt(uint8_t path, bool bcommit)
{
	int32_t rc = 0;
	int ipopt_cnt = 0;
	struct iris_update_ipopt *popt = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	popt = pcfg->pq_update_cmd.update_ipopt_array;
	ipopt_cnt = pcfg->pq_update_cmd.array_index;

	if (!popt || !ipopt_cnt) {
		IRIS_LOGE("%s(), invalid popt or ipopt_cnt", __func__);
		return;
	}

	rc = _iris_update_pq_seq(popt, ipopt_cnt);
	if ((!rc) && bcommit)
		_iris_send_pq_cmds(popt, ipopt_cnt, path);

	iris_init_ipopt_t();
}

static struct dsi_cmd_desc *_iris_get_desc_from_ipopt(uint8_t ip, uint8_t opt_id, int32_t pos)
{
	struct iris_ip_opt *popt = NULL;
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();
	popt = iris_find_ip_opt(ip, opt_id);
	if (popt == NULL) {
		IRIS_LOGE("%s(), can't find i_p opt for i_p: 0x%02x, opt: 0x%02x.",
				__func__, ip, opt_id);
		return NULL;
	}

	if (pos < 2) {
		IRIS_LOGE("%s(), invalid pos: %d", __func__, pos);
		return NULL;
	}

	return popt->cmd + (pos * 4 - IRIS_OCP_HEADER_ADDR_LEN) / pcfg->split_pkt_size;
}

struct dsi_cmd_desc *iris_get_specific_desc_from_ipopt(uint8_t ip,
		uint8_t opt_id, int32_t pos, uint32_t type)
{
	struct iris_ip_opt *popt = NULL;
	struct iris_cfg *pcfg = NULL;

	popt = iris_find_specific_ip_opt(ip, opt_id, type);
	if (popt == NULL) {
		IRIS_LOGE("%s(), can't find popt for i_p: 0x%02x, opt: 0x%02x, type: %u",
				__func__, ip, opt_id, type);
		return NULL;
	}

	if (pos < 2) {
		IRIS_LOGE("%s(), invalid pos: %d", __func__, pos);
		return NULL;
	}

	pcfg = iris_get_cfg();
	return popt->cmd + (pos * 4 - IRIS_OCP_HEADER_ADDR_LEN) / pcfg->split_pkt_size;
}

uint32_t *iris_get_ipopt_payload_data(
		uint8_t ip, uint8_t opt_id, int32_t pos)
{
	struct dsi_cmd_desc *pdesc = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	pdesc = _iris_get_desc_from_ipopt(ip, opt_id, pos);
	if (!pdesc) {
		IRIS_LOGE("%s(), failed to find desc!", __func__);
		return NULL;
	} /* else if (pos > pdesc->msg.tx_len) {
		IRIS_LOGE("%s(), pos %d is out of paload length %zu",
				__func__,
				pos, pdesc->msg.tx_len);
		return NULL;
	}
	*/

	return (uint32_t *)((uint8_t *)pdesc->msg.tx_buf + (pos * 4) % pcfg->split_pkt_size);
}

uint32_t iris_get_ipopt_payload_len(uint8_t ip, uint8_t opt_id, int32_t pos)
{
	struct dsi_cmd_desc *pdesc = _iris_get_desc_from_ipopt(ip, opt_id, pos);

	if (!pdesc) {
		IRIS_LOGE("%s(), can't find desc for i_p: 0x%02x, opt: 0x%02x, pos: %d",
		 __func__, ip, opt_id, pos);
		return 0;
	} else if (pos > pdesc->msg.tx_len) {
		IRIS_LOGE("%s(), pos %d is out of paload length %zu",
		 __func__, pos, pdesc->msg.tx_len);
		return 0;
	}

	return (uint32_t)(pdesc->msg.tx_len);
}

void iris_set_ipopt_payload_data(
		uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value)
{
	struct dsi_cmd_desc *pdesc = NULL;
	struct iris_cfg *pcfg = NULL;
	uint32_t *pvalue = NULL;

	pcfg = iris_get_cfg();
	pdesc = _iris_get_desc_from_ipopt(ip, opt_id, pos);
	if (!pdesc) {
		IRIS_LOGE("%s(), failed to find right desc.", __func__);
		return;
	}

	iris_sync_payload(ip, opt_id, pos, value);
	pvalue = (uint32_t *)((uint8_t *)pdesc->msg.tx_buf + (pos * 4) % pcfg->split_pkt_size);
	pvalue[0] = value;
}

void iris_update_bitmask_regval_nonread(
		struct iris_update_regval *pregval, bool is_commit)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t orig_val = 0;
	uint32_t *data = NULL;
	uint32_t val = 0;
	struct iris_ip_opt *popt = NULL;

	if (!pregval) {
		IRIS_LOGE("%s(), invalid input", __func__);
		return;
	}

	ip = pregval->ip;
	opt_id = pregval->opt_id;

	popt = iris_find_ip_opt(ip, opt_id);
	if (popt == NULL) {
		IRIS_LOGE("%s(), can't find i_p: 0x%02x opt: 0x%02x",
				__func__, ip, opt_id);
		return;
	} else if (popt->cmd_cnt != 1) {
		IRIS_LOGE("%s(), invalid bitmask, i_p: 0x%02x opt: 0x%02x popt len: %d",
				__func__,  ip, opt_id, popt->cmd_cnt);
		return;
	}

	data = (uint32_t *)popt->cmd[0].msg.tx_buf;

	orig_val = cpu_to_le32(data[2]);
	val = orig_val & (~pregval->mask);
	val |= (pregval->value & pregval->mask);
	data[2] = val;

	iris_sync_bitmask(pregval);

	pregval->value = val;

	if (is_commit)
		iris_send_ipopt_cmds(ip, opt_id);
}

uint32_t iris_get_regval_bitmask(int32_t ip, int32_t opt_id)
{
	uint32_t orig_val = 0;
	uint32_t *data = NULL;
	struct iris_ip_opt *popt = NULL;

	popt = iris_find_ip_opt(ip, opt_id);
	if (popt == NULL) {
		IRIS_LOGE("%s(), can't find i_p: 0x%02x opt: 0x%02x",
				__func__, ip, opt_id);
		return 0;
	} else if (popt->cmd_cnt != 1) {
		IRIS_LOGE("%s(), invalid bitmask, i_p: 0x%02x opt: 0x%02x popt len: %d",
				__func__,  ip, opt_id, popt->cmd_cnt);
		return 0;
	}

	data = (uint32_t *)popt->cmd[0].msg.tx_buf;
	orig_val = cpu_to_le32(data[2]);
	return orig_val;
}

uint32_t iris_schedule_line_no_get(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t schedule_line_no, panel_vsw_vbp;

	if ((pcfg->frc_enabled) || (pcfg->pwil_mode == FRC_MODE))
		schedule_line_no = pcfg->ovs_delay_frc;
	else
		schedule_line_no = pcfg->ovs_delay;

	panel_vsw_vbp = pcfg->panel->cur_mode->timing.v_back_porch +
			pcfg->panel->cur_mode->timing.v_sync_width;

	if (pcfg->vsw_vbp_delay > panel_vsw_vbp)
		schedule_line_no += pcfg->vsw_vbp_delay - panel_vsw_vbp;

	if (pcfg->dtg_eco_enabled)
		schedule_line_no += pcfg->vsw_vbp_delay;

	return schedule_line_no;
}

#ifdef IRIS_EXT_CLK
void iris_clk_enable(struct dsi_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (panel->is_secondary) {
		IRIS_LOGD("%s(), %d, skip enable clk in virtual channel", __func__, __LINE__);
		return;
	}

	if (pcfg->ext_clk && !pcfg->clk_enable_flag) {
		IRIS_LOGI("%s(), %d, enable ext clk", __func__, __LINE__);
		clk_prepare_enable(pcfg->ext_clk);
		pcfg->clk_enable_flag = true;
		usleep_range(5000, 5001);
	} else {
		if (!pcfg->ext_clk)
			IRIS_LOGE("%s(), %d, ext clk not exist!", __func__, __LINE__);
		if (pcfg->clk_enable_flag)
			IRIS_LOGI("%s(), %d, ext clk has enabled", __func__, __LINE__);
	}
}

void iris_clk_disable(struct dsi_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (panel->is_secondary) {
		IRIS_LOGD("%s(), %d, skip disable clk in virtual channel", __func__, __LINE__);
		return;
	}

	if (pcfg->ext_clk && pcfg->clk_enable_flag) {
		IRIS_LOGI("%s(), %d, disable ext clk", __func__, __LINE__);
		clk_disable_unprepare(pcfg->ext_clk);
		pcfg->clk_enable_flag = false;
	} else {
		if (!pcfg->ext_clk)
			IRIS_LOGE("%s(), %d, ext clk not exist!", __func__, __LINE__);
		if (!pcfg->clk_enable_flag)
			IRIS_LOGI("%s(), %d, ext clk not enabled", __func__, __LINE__);
	}
}
#endif

static ssize_t _iris_cont_splash_write(
		struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	_iris_set_cont_splash_type(val);

	if (val == IRIS_CONT_SPLASH_KERNEL) {
		struct iris_cfg *pcfg = iris_get_cfg();
		mutex_lock(&pcfg->panel->panel_lock);
		_iris_send_cont_splash_pkt(val);
		mutex_unlock(&pcfg->panel->panel_lock);
	} else if (val != IRIS_CONT_SPLASH_LK &&
			val != IRIS_CONT_SPLASH_NONE) {
		IRIS_LOGE("the value is %zu, need to be 1 or 2 3", val);
	}

	return count;
}

static ssize_t _iris_cont_splash_read(
		struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	uint8_t type;
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	type = iris_get_cont_splash_type();
	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", type);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_cont_splash_fops = {
	.open = simple_open,
	.write = _iris_cont_splash_write,
	.read = _iris_cont_splash_read,
};

int iris_wait_vsync(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct drm_encoder *drm_enc;

	if (pcfg->display == NULL || pcfg->display->bridge == NULL)
		return -ENOLINK;
	drm_enc = pcfg->display->bridge->base.encoder;
	if (!drm_enc || !drm_enc->crtc)
		return -ENOLINK;
	if (sde_encoder_is_disabled(drm_enc))
		return -EIO;

	sde_encoder_wait_for_event(drm_enc, MSM_ENC_VBLANK);

	return 0;
}

int iris_set_pending_panel_brightness(int32_t pending, int32_t delay, int32_t level)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg) {
		IRIS_LOGI("set pending panel %d, %d, %d", pending, delay, level);
		pcfg->panel_pending = pending;
		pcfg->panel_delay = delay;
		pcfg->panel_level = level;
	}

	return 0;
}

int iris_sync_panel_brightness(int32_t step, void *phys_enc)
{
	struct sde_encoder_phys *phys_encoder = phys_enc;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct iris_cfg *pcfg;
	int rc = 0;

	if (phys_encoder == NULL)
		return -EFAULT;
	if (phys_encoder->connector == NULL)
		return -EFAULT;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return -EFAULT;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	display = c_conn->display;
	if (display == NULL)
		return -EFAULT;

	pcfg = iris_get_cfg();

	if (pcfg->panel_pending == step) {
		IRIS_LOGI("sync pending panel %d %d,%d,%d",
				step, pcfg->panel_pending, pcfg->panel_delay,
				pcfg->panel_level);
		SDE_ATRACE_BEGIN("sync_panel_brightness");
		if (step <= 2) {
			rc = c_conn->ops.set_backlight(&c_conn->base,
					display, pcfg->panel_level);
			if (pcfg->panel_delay > 0)
				usleep_range(pcfg->panel_delay, pcfg->panel_delay + 1);
		} else {
			if (pcfg->panel_delay > 0)
				usleep_range(pcfg->panel_delay, pcfg->panel_delay + 1);
			rc = c_conn->ops.set_backlight(&c_conn->base,
					display, pcfg->panel_level);
		}
		if (c_conn->bl_device)
			c_conn->bl_device->props.brightness = pcfg->panel_level;
		pcfg->panel_pending = 0;
		SDE_ATRACE_END("sync_panel_brightness");
	}

	return rc;
}

static ssize_t _iris_chip_id_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int tot = 0;
	struct iris_cfg *pcfg = NULL;
	char bp[512];

	if (*ppos)
		return 0;

	pcfg = iris_get_cfg();

	tot = scnprintf(bp, sizeof(bp), "%u\n", pcfg->chip_id);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static const struct file_operations iris_chip_id_fops = {
	.open = simple_open,
	.read = _iris_chip_id_read,
};

static ssize_t _iris_power_mode_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int tot = 0;
	struct iris_cfg *pcfg = NULL;
	char bp[512];

	if (*ppos)
		return 0;

	pcfg = iris_get_cfg();

	tot = scnprintf(bp, sizeof(bp), "0x%02x\n", pcfg->power_mode);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static const struct file_operations iris_power_mode_fops = {
	.open = simple_open,
	.read = _iris_power_mode_read,
};

static ssize_t _iris_list_debug(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	uint8_t ip;
	uint8_t opt_id;
	int32_t pos;
	uint32_t value;
	char buf[64];
	uint32_t *payload = NULL;

	if (count >= sizeof(buf))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end if string */

	if (sscanf(buf, "%x %x %x %x", &ip, &opt_id, &pos, &value) != 4)
		return -EINVAL;

	payload = iris_get_ipopt_payload_data(ip, opt_id, 2);
	if (payload == NULL)
		return -EFAULT;

	IRIS_LOGI("%s: %x %x %x %x", __func__, ip, opt_id, pos, value);

	iris_set_ipopt_payload_data(ip, opt_id, pos, value);

	return count;
}

static const struct file_operations _iris_list_debug_fops = {
	.open = simple_open,
	.write = _iris_list_debug,
};

/* adb shell "echo 05 00 45 > d/iris/iris_cmd_payload"
 * parameters: ip(Hex) opt(Hex) pos(Dec)
 * check dump value on kernel log
 */
static ssize_t _iris_dump_cmd_payload(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	uint8_t ip;
	uint8_t opt_id;
	int32_t pos;
	char buf[64];
	uint32_t *payload = NULL;

	if (count >= sizeof(buf))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end if string */

	if (sscanf(buf, "%x %x %d", &ip, &opt_id, &pos) != 3) {
		if (sscanf(buf, "%x %x", &ip, &opt_id) == 2) {
			struct iris_ip_opt *popt = NULL;
			int i = 0;

			if (!_iris_is_valid_ip(ip)) {
				IRIS_LOGE("%s(), invalid i_p: %#x", __func__, ip);
				return -EINVAL;
			}

			popt = iris_find_ip_opt(ip, opt_id);
			if (popt == NULL) {
				IRIS_LOGE("%s(), invalid i_p 0x%02x, opt 0x%02x", __func__, ip, opt_id);
				return -EINVAL;
			}

			IRIS_LOGW("%s(), for i_p 0x%02x, opt 0x%02x", __func__, ip, opt_id);
			for (i = 0; i < popt->cmd_cnt; i++) {
				print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 32, 4,
						popt->cmd[i].msg.tx_buf, popt->cmd[i].msg.tx_len, false);
			}

			return count;
		}

		return -EINVAL;
	}

	payload = iris_get_ipopt_payload_data(ip, opt_id, 2);
	if (payload == NULL)
		return -EFAULT;

	IRIS_LOGW("%s(), for i_p: 0x%02X opt: 0x%02X, payload[%d] is: 0x%08X",
			__func__, ip, opt_id, pos, payload[pos]);

	return count;
}

static const struct file_operations iris_cmd_payload_fops = {
	.open = simple_open,
	.write = _iris_dump_cmd_payload,
};

static int _iris_dbgfs_cont_splash_init(struct dsi_display *display)
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
	if (debugfs_create_file("iris_cont_splash", 0644, pcfg->dbg_root, display,
				&iris_cont_splash_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("chip_id", 0644, pcfg->dbg_root, display,
				&iris_chip_id_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("power_mode", 0644, pcfg->dbg_root, display,
				&iris_power_mode_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	debugfs_create_u8("dsi_trans_mode_for_lightup", 0644, pcfg->dbg_root,
		&pcfg->dsi_trans_mode[0]);

	debugfs_create_u8("dsi_trans_mode_for_pqupdate", 0644, pcfg->dbg_root,
		&pcfg->dsi_trans_mode[1]);

	debugfs_create_u32("dsi_trans_len_non_embedded_for_lightup", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->dsi_trans_len[0][0]);

	debugfs_create_u32("dsi_trans_len_non_embedded_for_pqupdate", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->dsi_trans_len[0][1]);

	debugfs_create_u32("dsi_trans_len_embedded_no_ma_for_lightup", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->dsi_trans_len[1][0]);

	debugfs_create_u32("dsi_trans_len_embedded_no_ma_for_pqupdate", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->dsi_trans_len[1][1]);

	debugfs_create_u32("dsi_trans_len_embedded_ma_for_lightup", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->dsi_trans_len[2][0]);

	debugfs_create_u32("dsi_trans_len_embedded_ma_for_pqupdate", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->dsi_trans_len[2][1]);

	if (debugfs_create_file("iris_list_debug",	0644, pcfg->dbg_root, display,
				&_iris_list_debug_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("iris_cmd_payload", 0644, pcfg->dbg_root, display,
				&iris_cmd_payload_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file for 'iris_cmd_payload' failed",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	debugfs_create_u8("ocp_read_by_i2c", 0644, pcfg->dbg_root, &pcfg->ocp_read_by_i2c);

	return 0;
}

static void _iris_fw_create_name(struct dsi_display *display, char *name)
{
	u8 *np = name;
	int len = 0;
	char *key = "qcom,mdss_dsi";

	len = sprintf(np, "%s", "pxlw");
	np += len;
	len = sprintf(np, "%s.fw", display->name + strlen(key));
	IRIS_LOGI("%s dts fw name:%s", __func__, name);
}

static int _iris_fw_parse_dts(struct dsi_display *display)
{
	char fw_name[256] = {};

	if (iris_get_cfg()->cmd_param_from_fw) {
		iris_set_dts_ops(DTS_CTX_FROM_FW);

		_iris_fw_create_name(display, fw_name);

		return iris_parse_dts_ctx(fw_name);
	}
	return 0;
}

void iris_prepare(struct dsi_display *display)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	static bool is_boot = true;

	if (!iris_is_chip_supported())
		return;

	if (display->panel->is_secondary)
		return;

	if (is_boot) {
		is_boot = false;

		if (pcfg->valid == PARAM_PARSED) {
			if (_iris_fw_parse_dts(display)) {
				pcfg->valid = PARAM_EMPTY;
				pcfg->abyp_ctrl.abypass_mode = ANALOG_BYPASS_MODE;
				return;
			}
			iris_frc_setting_init();
			iris_parse_lut_cmds(LOAD_GOLDEN_ONLY);
			iris_alloc_seq_space();
			iris_alloc_update_ipopt_space();
			pcfg->valid = PARAM_PREPARED;	/* prepare ok */
		}
	}
}

static int _iris_dev_probe(struct platform_device *pdev)
{
	struct iris_cfg *pcfg;
	int rc;

	IRIS_LOGI("%s()", __func__);
	if (!pdev || !pdev->dev.of_node) {
		IRIS_LOGE("%s(), pdev not found", __func__);
		return -ENODEV;
	}

	pcfg = iris_get_cfg();
	pcfg->pdev = pdev;
	dev_set_drvdata(&pdev->dev, pcfg);

	rc = iris_enable_pinctrl(pdev, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to enable pinctrl, return: %d",
			__func__, rc);
	}

	rc = iris_parse_gpio(pdev, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse gpio, return: %d",
				__func__, rc);
		return rc;
	}

	iris_request_gpio();

	return 0;
}

static int _iris_dev_remove(struct platform_device *pdev)
{
	struct iris_cfg *pcfg = dev_get_drvdata(&pdev->dev);

	IRIS_LOGI("%s()", __func__);

	iris_release_gpio(pcfg);

	return 0;
}

static const struct of_device_id iris_dt_match[] = {
	{.compatible = "pxlw,iris"},
	{}
};

static struct platform_driver iris_driver = {
	.probe = _iris_dev_probe,
	.remove = _iris_dev_remove,
	.driver = {
		.name = "pxlw-iris",
		.of_match_table = iris_dt_match,
	},
};

int iris_driver_register(void)
{
	if (iris_driver_registered)
		return 0;
	else
		iris_driver_registered = true;

	return platform_driver_register(&iris_driver);
}

void iris_driver_unregister(void)
{
	if (iris_driver_unregistered || !iris_driver_registered)
		return;
	else
		iris_driver_unregistered = true;

	platform_driver_unregister(&iris_driver);
}

//module_platform_driver(iris_driver);
