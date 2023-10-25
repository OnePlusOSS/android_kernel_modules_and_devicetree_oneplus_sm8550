// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <linux/vmalloc.h>
#include <video/mipi_display.h>
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_log.h"
#include "dsi_iris_reg.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_i3c.h"
#include "dsi_iris_cmpt.h"

#define IRIS_TX_HV_PAYLOAD_LEN   120
#define IRIS_TX_PAYLOAD_LEN 124
#define IRIS_RD_PACKET_DATA  0xF13DC018
#define IRIS_TX_INTSTAT_RAW 0xF13DFFE4
#define IRIS_TX_READ_RESPONSE_RECEIVED 0x80000000
#define IRIS_TX_READ_ERR_MASK 0x6FEFFEFF
#define IRIS_TX_INTCLR 0xF13DFFF0
#define IRIS_RD_PACKET_DATA_I3  0xF0C1C018

static char iris_read_cmd_rbuf[16];
static struct iris_ocp_cmd ocp_cmd;
static struct iris_ocp_cmd ocp_test_cmd[DSI_CMD_CNT];
static struct dsi_cmd_desc iris_test_cmd[DSI_CMD_CNT];

static void _iris_add_cmd_addr_val(
		struct iris_ocp_cmd *pcmd, u32 addr, u32 val)
{
	*(u32 *)(pcmd->cmd + pcmd->cmd_len) = cpu_to_le32(addr);
	*(u32 *)(pcmd->cmd + pcmd->cmd_len + 4) = cpu_to_le32(val);
	pcmd->cmd_len += 8;
}

static void _iris_add_cmd_payload(struct iris_ocp_cmd *pcmd, u32 payload)
{
	*(u32 *)(pcmd->cmd + pcmd->cmd_len) = cpu_to_le32(payload);
	pcmd->cmd_len += 4;
}

void remap_to_qcom_style(struct dsi_cmd_desc *dst, struct dsi_cmd_desc_pxlw *src, u32 count)
{
	u32 i;

	for (i = 0; i < count; i++) {
		(dst + i)->msg.channel = (src + i)->msg.channel;
		(dst + i)->msg.type = (src + i)->msg.type;
		(dst + i)->msg.flags = (src + i)->msg.flags;
#if defined(CONFIG_ARCH_WAIPIO) || defined(CONFIG_ARCH_LAHAINA)
		(dst + i)->msg.ctrl = (src + i)->msg.ctrl;
		(dst + i)->msg.wait_ms = (src + i)->msg.wait_ms;
#endif
		(dst + i)->msg.tx_len = (src + i)->msg.tx_len;
		(dst + i)->msg.tx_buf = (src + i)->msg.tx_buf;
		(dst + i)->msg.rx_len = (src + i)->msg.rx_len;
		(dst + i)->msg.rx_buf = (src + i)->msg.rx_buf;
		(dst + i)->last_command = (src + i)->last_command;
		(dst + i)->post_wait_ms = (src + i)->post_wait_ms;
	}
}
void iris_ocp_write_val(u32 address, u32 value)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc_pxlw iris_ocp_cmd_pxlw[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0} };
	struct dsi_cmd_desc iris_ocp_cmd[1];

	remap_to_qcom_style(iris_ocp_cmd, iris_ocp_cmd_pxlw, 1);

	memset(&ocp_cmd, 0, sizeof(ocp_cmd));

	_iris_add_cmd_payload(&ocp_cmd, 0xFFFFFFF0 | OCP_SINGLE_WRITE_BYTEMASK);
	_iris_add_cmd_addr_val(&ocp_cmd, address, value);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

	IRIS_LOGD("%s(), addr: %#x, value: %#x", __func__, address, value);

	iris_dsi_send_cmds(pcfg->panel, iris_ocp_cmd, 1, DSI_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);
}

void iris_ocp_write_vals(u32 header, u32 address, u32 size, u32 *pvalues, enum dsi_cmd_set_state state)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc_pxlw iris_ocp_cmd_pxlw[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0} };
	struct dsi_cmd_desc iris_ocp_cmd[1];

	u32 max_size = CMD_PKT_SIZE / 4 - 2;
	u32 i;

	remap_to_qcom_style(iris_ocp_cmd, iris_ocp_cmd_pxlw, 1);

	while (size > 0) {
		memset(&ocp_cmd, 0, sizeof(ocp_cmd));

		_iris_add_cmd_payload(&ocp_cmd, header);
		_iris_add_cmd_payload(&ocp_cmd, address);
		if (size < max_size) {
			for (i = 0; i < size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			size = 0;
		} else {
			for (i = 0; i < max_size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			address += max_size * 4;
			pvalues += max_size;
			size -= max_size;
		}
		iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;
		IRIS_LOGD("%s(), header: %#x, addr: %#x, len: %zu", __func__,
				header, address, iris_ocp_cmd[0].msg.tx_len);

		iris_dsi_send_cmds(pcfg->panel, iris_ocp_cmd, 1, state, pcfg->vc_ctrl.to_iris_vc_id);
	}
}

void iris_ocp_bit_en_write(u32 address, u32 bit_en, u32 bit_val, enum dsi_cmd_set_state state)
{

	u32 vals[2] = {0};

	vals[0] = bit_en;
	vals[1] = bit_val;
	iris_ocp_write_vals(OCP_SINGLE_WRITE_BITEN, address, 2, vals, state);
}

/*pvalues need to be one address and one value*/
static void _iris_dsi_write_mult_vals(u32 size, u32 *pvalues)
{
	u32 i;
	/*need to remove one header length*/
	u32 max_size = 60; /*(244 -4)/4*/
	u32 header = 0xFFFFFFF4;
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc_pxlw iris_ocp_cmd_pxlw[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0} };
	struct dsi_cmd_desc iris_ocp_cmd[1];

	remap_to_qcom_style(iris_ocp_cmd, iris_ocp_cmd_pxlw, 1);

	if (size % 2 != 0) {
		IRIS_LOGE("%s(), need to be mult pair of address and value", __func__);
		return;
	}

	while (size > 0) {
		memset(&ocp_cmd, 0, sizeof(ocp_cmd));

		_iris_add_cmd_payload(&ocp_cmd, header);
		if (size < max_size) {
			for (i = 0; i < size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			size = 0;
		} else {
			for (i = 0; i < max_size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			pvalues += max_size;
			size -= max_size;
		}
		iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;
		IRIS_LOGD("%s(), header: 0x%08x, len: %zu",
				__func__,
				header, iris_ocp_cmd[0].msg.tx_len);

		iris_dsi_send_cmds(pcfg->panel, iris_ocp_cmd,
			1, DSI_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);
	}
}

/*pvalues need to be one address and one value*/
void iris_ocp_write_mult_vals(u32 size, u32 *pvalues)
{
	IRIS_LOGD("%s(%d), path select dsi", __func__, __LINE__);
	_iris_dsi_write_mult_vals(size, pvalues);
}

static void _iris_ocp_write_addr(u32 address, u32 mode)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc_pxlw iris_ocp_cmd_pxlw[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0} };
	struct dsi_cmd_desc iris_ocp_cmd[1];

	remap_to_qcom_style(iris_ocp_cmd, iris_ocp_cmd_pxlw, 1);

	/* Send OCP command.*/
	memset(&ocp_cmd, 0, sizeof(ocp_cmd));

	_iris_add_cmd_payload(&ocp_cmd, OCP_SINGLE_READ);
	_iris_add_cmd_payload(&ocp_cmd, address);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

	iris_dsi_send_cmds(pcfg->panel, iris_ocp_cmd, 1, mode, pcfg->vc_ctrl.to_iris_vc_id);
}

static u32 _iris_ocp_read_value(u32 mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	char pi_read[1] = {0x00};
	struct dsi_cmd_desc_pxlw pi_read_cmd_pxlw[] = {
		{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_REQ_ACK,
			 0, 0, sizeof(pi_read), pi_read, 0, NULL},
		1, 0} };
	struct dsi_cmd_desc pi_read_cmd[1];
	u32 response_value;

	remap_to_qcom_style(pi_read_cmd, pi_read_cmd_pxlw, 1);

	iris_set_msg_flags(pi_read_cmd, READ_FLAG);
	/* Read response.*/
	memset(iris_read_cmd_rbuf, 0, sizeof(iris_read_cmd_rbuf));
	pi_read_cmd[0].msg.rx_len = 4;
	pi_read_cmd[0].msg.rx_buf = iris_read_cmd_rbuf;
	iris_dsi_send_cmds(pcfg->panel, pi_read_cmd, 1, mode, pcfg->vc_ctrl.to_iris_vc_id);

	IRIS_LOGD("%s(), read register: 0x%02x 0x%02x 0x%02x 0x%02x", __func__,
			iris_read_cmd_rbuf[0], iris_read_cmd_rbuf[1],
			iris_read_cmd_rbuf[2], iris_read_cmd_rbuf[3]);

	response_value = iris_read_cmd_rbuf[0] | (iris_read_cmd_rbuf[1] << 8)
				| (iris_read_cmd_rbuf[2] << 16) | (iris_read_cmd_rbuf[3] << 24);

	return response_value;
}

u32 _iris_dsi_ocp_read(u32 address, u32 mode)
{
	u32 value = 0;

	_iris_ocp_write_addr(address, mode);

	value = _iris_ocp_read_value(mode);
	IRIS_LOGD("%s(), addr: %#x, value: %#x", __func__, address, value);

	return value;
}

u32 iris_ocp_read(u32 address, u32 mode)
{
	u32 value = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_ATRACE_BEGIN(__func__);
	if (pcfg->ocp_read_by_i2c && pcfg->iris_i2c_read) {
		IRIS_LOGD("%s(%d), path select i2c", __func__, __LINE__);
		if (pcfg->iris_i2c_read(address, &value) < 0)
			IRIS_LOGE("i2c read reg fails, reg addr=0x%x", address);
		IRIS_ATRACE_END(__func__);
		return value;
	}
	IRIS_LOGD("%s(%d), path select dsi", __func__, __LINE__);
	value = _iris_dsi_ocp_read(address, mode);
	IRIS_ATRACE_END(__func__);

	return value;
}

static void _iris_dump_packet(u8 *data, int size)
{
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4, data, size, false);
}

void iris_write_test(struct dsi_panel *panel, u32 iris_addr,
		int ocp_type, u32 pkt_size)
{
	union iris_ocp_cmd_header ocp_header;
	struct dsi_cmd_desc_pxlw iris_cmd_pxlw = {
		{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0};
	u32 test_value = 0xFFFF0000;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc iris_cmd;

	remap_to_qcom_style(&iris_cmd, &iris_cmd_pxlw, 1);

	memset(&ocp_header, 0, sizeof(ocp_header));
	ocp_header.header32 = 0xFFFFFFF0 | ocp_type;

	memset(&ocp_cmd, 0, sizeof(ocp_cmd));
	memcpy(ocp_cmd.cmd, &ocp_header.header32, OCP_HEADER);
	ocp_cmd.cmd_len = OCP_HEADER;

	switch (ocp_type) {
	case OCP_SINGLE_WRITE_BYTEMASK:
	case OCP_SINGLE_WRITE_BITMASK:
		for (; ocp_cmd.cmd_len <= (pkt_size - 8); ) {
			_iris_add_cmd_addr_val(&ocp_cmd, iris_addr, test_value);
			test_value++;
		}
		break;
	case OCP_BURST_WRITE:
		test_value = 0xFFFF0000;
		_iris_add_cmd_addr_val(&ocp_cmd, iris_addr, test_value);
		if (pkt_size <= ocp_cmd.cmd_len)
			break;
		test_value++;
		for (; ocp_cmd.cmd_len <= pkt_size - 4;) {
			_iris_add_cmd_payload(&ocp_cmd, test_value);
			test_value++;
		}
		break;
	default:
		break;
	}

	IRIS_LOGI("%s(), len: %d, iris addr: %#x, test value: %#x",
			__func__,
			ocp_cmd.cmd_len, iris_addr, test_value);
	iris_cmd.msg.tx_len = ocp_cmd.cmd_len;
	iris_dsi_send_cmds(panel, &iris_cmd, 1, DSI_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);

	if (IRIS_IF_LOGD())
		_iris_dump_packet(ocp_cmd.cmd, ocp_cmd.cmd_len);
}

void iris_write_test_muti_pkt(struct dsi_panel *panel,
		struct iris_ocp_dsi_tool_input *ocp_input)
{
	union iris_ocp_cmd_header ocp_header;
	u32 test_value = 0xFF000000;
	int cnt = 0;

	u32 iris_addr, ocp_type, pkt_size, total_cnt;
	struct iris_cfg *pcfg = iris_get_cfg();

	ocp_type = ocp_input->iris_ocp_type;
	test_value = ocp_input->iris_ocp_value;
	iris_addr = ocp_input->iris_ocp_addr;
	total_cnt = ocp_input->iris_ocp_cnt;
	pkt_size = ocp_input->iris_ocp_size;

	memset(iris_test_cmd, 0, sizeof(iris_test_cmd));
	memset(ocp_test_cmd, 0, sizeof(ocp_test_cmd));

	memset(&ocp_header, 0, sizeof(ocp_header));
	ocp_header.header32 = 0xFFFFFFF0 | ocp_type;

	switch (ocp_type) {
	case OCP_SINGLE_WRITE_BYTEMASK:
	case OCP_SINGLE_WRITE_BITMASK:
		for (cnt = 0; cnt < total_cnt; cnt++) {
			memcpy(ocp_test_cmd[cnt].cmd,
					&ocp_header.header32, OCP_HEADER);
			ocp_test_cmd[cnt].cmd_len = OCP_HEADER;

			test_value = 0xFF000000 | (cnt << 16);
			while (ocp_test_cmd[cnt].cmd_len <= (pkt_size - 8)) {
				_iris_add_cmd_addr_val(&ocp_test_cmd[cnt],
						(iris_addr + cnt * 4), test_value);
				test_value++;
			}

			iris_test_cmd[cnt].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
			iris_test_cmd[cnt].msg.tx_len = ocp_test_cmd[cnt].cmd_len;
			iris_test_cmd[cnt].msg.tx_buf = ocp_test_cmd[cnt].cmd;
		}
		iris_test_cmd[total_cnt - 1].last_command = true;
		break;
	case OCP_BURST_WRITE:
		for (cnt = 0; cnt < total_cnt; cnt++) {
			memcpy(ocp_test_cmd[cnt].cmd,
					&ocp_header.header32, OCP_HEADER);
			ocp_test_cmd[cnt].cmd_len = OCP_HEADER;
			test_value = 0xFF000000 | (cnt << 16);

			_iris_add_cmd_addr_val(&ocp_test_cmd[cnt],
					(iris_addr + cnt * 4), test_value);
			/* if(pkt_size <= ocp_test_cmd[cnt].cmd_len)
			 * break;
			 */
			test_value++;
			while (ocp_test_cmd[cnt].cmd_len <= pkt_size - 4) {
				_iris_add_cmd_payload(&ocp_test_cmd[cnt], test_value);
				test_value++;
			}

			iris_test_cmd[cnt].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
			iris_test_cmd[cnt].msg.tx_len = ocp_test_cmd[cnt].cmd_len;
			iris_test_cmd[cnt].msg.tx_buf = ocp_test_cmd[cnt].cmd;
		}
		iris_test_cmd[total_cnt - 1].last_command = true;
		break;
	default:
		break;
	}

	IRIS_LOGI("%s(), total cnt: %#x iris addr: %#x test value: %#x",
		__func__, total_cnt, iris_addr, test_value);

	iris_dsi_send_cmds(panel, iris_test_cmd, total_cnt, DSI_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);

	if (IRIS_IF_NOT_LOGV())
		return;

	for (cnt = 0; cnt < total_cnt; cnt++)
		_iris_dump_packet(ocp_test_cmd[cnt].cmd,
				ocp_test_cmd[cnt].cmd_len);
}

int iris_dsi_send_cmds(struct dsi_panel *panel, struct dsi_cmd_desc *cmds,
		u32 count, enum dsi_cmd_set_state state, u8 vc_id)
{
	int rc = 0;
	int i = 0;
	int cont_payload_size = 0;
	int cont_cmd_num = 0;
	ssize_t len;
	const struct mipi_dsi_host_ops *ops;
	struct iris_cfg *pcfg = NULL;
	struct dsi_display *display = NULL;
	u8 vc_id_bak;

	if (!panel || !panel->cur_mode)
		return -EINVAL;

	if (count == 0 || cmds == NULL) {
		IRIS_LOGD("%s(), panel %s no commands to be sent for state %d",
				__func__,
				panel->name, state);
		return -EINVAL;
	}

	pcfg = iris_get_cfg();
	IRIS_ATRACE_BEGIN(__func__);

	display = pcfg->display;

	ops = panel->host->ops;

	for (i = 0; i < count; i++) {
		vc_id_bak = cmds->msg.channel;
		cmds->msg.channel = vc_id;

		if (cmds->msg.flags & MIPI_DSI_MSG_CMD_DMA_SCHED)
			cmds->msg.flags = MIPI_DSI_MSG_CMD_DMA_SCHED;
		else
			cmds->msg.flags = 0;

		if (state == DSI_CMD_SET_STATE_LP)
			cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

		cmds->last_command ? iris_set_msg_flags(cmds, LAST_FLAG)
			: iris_set_msg_flags(cmds, BATCH_FLAG);

		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_ALL_CLKS, DSI_CLK_ON);
		WARN_ON(!mutex_is_locked(&panel->panel_lock));
		len = ops->transfer(panel->host, &cmds->msg);
		dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_ALL_CLKS, DSI_CLK_OFF);

		if (IRIS_IF_LOGVV())
			_iris_dump_packet((u8 *)cmds->msg.tx_buf, cmds->msg.tx_len);

		if (len < 0) {
			rc = len;
			IRIS_LOGE("%s(), failed to set cmds: %d, return: %d",
					__func__,
					cmds->msg.type, rc);
			dump_stack();
			goto error;
		}
		if (cmds->post_wait_ms)
			usleep_range(cmds->post_wait_ms * 1000,
					((cmds->post_wait_ms * 1000) + 10));

		cont_payload_size += cmds->msg.tx_len;
		cont_cmd_num++;
		if (cmds->last_command) {
			if (pcfg->vc_ctrl.vc_enable)
				if (pcfg->vc_ctrl.to_iris_vc_id == vc_id)
					iris_insert_delay_us(cont_payload_size, cont_cmd_num);
			cont_payload_size = 0;
			cont_cmd_num = 0;
		}
		cmds->msg.channel = vc_id_bak;
		cmds++;
	}
error:
	IRIS_ATRACE_END(__func__);
	return rc;
}

static u32 _iris_pt_get_split_pkt_cnt(int dlen)
{
	u32 sum = 1;

	if (dlen > IRIS_TX_HV_PAYLOAD_LEN)
		sum = (dlen - IRIS_TX_HV_PAYLOAD_LEN
				+ IRIS_TX_PAYLOAD_LEN - 1) / IRIS_TX_PAYLOAD_LEN + 1;
	return sum;
}

/*
 * @Description: use to do statitics for cmds which should not less than 252
 *      if the payload is out of 252, it will change to more than one cmds
 * the first payload need to be
 *	4 (ocp_header) + 8 (tx_addr_header + tx_val_header)
 *	+ 2* payload_len (TX_payloadaddr + payload_len)<= 252
 * the sequence payloader need to be
 *	4 (ocp_header) + 2* payload_len (TX_payloadaddr + payload_len)<= 252
 *	so the first payload should be no more than 120
 *	the second and sequence need to be no more than 124
 *
 * @Param: cmdset  cmds request
 * @return: the cmds number need to split
 **/
static u32 _iris_pt_calc_cmd_cnt(struct dsi_panel_cmd_set *cmdset)
{
	u32 i = 0;
	u32 sum = 0;
	u32 dlen = 0;

	for (i = 0; i < cmdset->count; i++) {
		dlen = cmdset->cmds[i].msg.tx_len;
		sum += _iris_pt_get_split_pkt_cnt(dlen);
	}

	return sum;
}

static int _iris_pt_alloc_cmds(
		struct dsi_panel_cmd_set *cmdset,
		struct dsi_cmd_desc **ptx_cmds,
		struct iris_ocp_cmd **pocp_cmds)
{
	int cmds_cnt = _iris_pt_calc_cmd_cnt(cmdset);

	IRIS_LOGD("%s(%d), cmds cnt: %d malloc len: %lu",
			__func__, __LINE__,
			cmds_cnt, cmds_cnt * sizeof(**ptx_cmds));
	*ptx_cmds = vmalloc(cmds_cnt * sizeof(**ptx_cmds));
	if (!(*ptx_cmds)) {
		IRIS_LOGE("%s(), failed to malloc buf, len: %lu",
				__func__,
				cmds_cnt * sizeof(**ptx_cmds));
		return -ENOMEM;
	}

	*pocp_cmds = vmalloc(cmds_cnt * sizeof(**pocp_cmds));
	if (!(*pocp_cmds)) {
		IRIS_LOGE("%s(), failed to malloc buf for pocp cmds", __func__);
		vfree(*ptx_cmds);
		*ptx_cmds = NULL;
		return -ENOMEM;
	}
	return cmds_cnt;
}

static void _iris_pt_init_tx_cmd_hdr(
		struct dsi_panel_cmd_set *cmdset, struct dsi_cmd_desc *dsi_cmd,
		union iris_mipi_tx_cmd_header *header)
{
	u8 dtype = dsi_cmd->msg.type;

	memset(header, 0x00, sizeof(*header));
	header->stHdr.dtype = dtype;
	header->stHdr.linkState = (cmdset->state == DSI_CMD_SET_STATE_LP) ? 1 : 0;
}

static void _iris_pt_set_cmd_hdr(
		union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd, bool is_write)
{
	u32 dlen = 0;
	u8 *ptr = NULL;

	if (!dsi_cmd)
		return;

	dlen = dsi_cmd->msg.tx_len;

	if (is_write)
		pheader->stHdr.writeFlag = 0x01;
	else
		pheader->stHdr.writeFlag = 0x00;

	if (pheader->stHdr.longCmdFlag == 0) {
		ptr = (u8 *)dsi_cmd->msg.tx_buf;
		if (dlen == 1) {
			pheader->stHdr.len[0] = ptr[0];
		} else if (dlen == 2) {
			pheader->stHdr.len[0] = ptr[0];
			pheader->stHdr.len[1] = ptr[1];
		}
	} else {
		pheader->stHdr.len[0] = dlen & 0xff;
		pheader->stHdr.len[1] = (dlen >> 8) & 0xff;
	}
}

static void _iris_pt_set_wrcmd_hdr(
		union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd)
{
	_iris_pt_set_cmd_hdr(pheader, dsi_cmd, true);
}

static void _iris_pt_set_rdcmd_hdr(
		union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd)
{
	_iris_pt_set_cmd_hdr(pheader, dsi_cmd, false);
}

static void _iris_pt_init_ocp_cmd(struct iris_ocp_cmd *pocp_cmd)
{
	union iris_ocp_cmd_header ocp_header;

	if (!pocp_cmd) {
		IRIS_LOGE("%s(), invalid pocp cmd!", __func__);
		return;
	}

	memset(pocp_cmd, 0x00, sizeof(*pocp_cmd));
	ocp_header.header32 = 0xfffffff0 | OCP_SINGLE_WRITE_BYTEMASK;
	memcpy(pocp_cmd->cmd, &ocp_header.header32, OCP_HEADER);
	pocp_cmd->cmd_len = OCP_HEADER;
}

static void _iris_add_tx_cmds(
		struct dsi_cmd_desc *ptx_cmd,
		struct iris_ocp_cmd *pocp_cmd, u8 wait, u16 flags)
{
	struct dsi_cmd_desc_pxlw desc_init_val_pxlw = {
		{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			CMD_PKT_SIZE, NULL, 0, NULL}, 0, 0};
	struct dsi_cmd_desc desc_init_val;

	remap_to_qcom_style(&desc_init_val, &desc_init_val_pxlw, 1);

	memcpy(ptx_cmd, &desc_init_val, sizeof(struct dsi_cmd_desc));
	ptx_cmd->msg.tx_buf = pocp_cmd->cmd;
	ptx_cmd->msg.tx_len = pocp_cmd->cmd_len;
	ptx_cmd->msg.flags = flags;
	ptx_cmd->last_command = iris_is_last_cmd(&ptx_cmd->msg);
	ptx_cmd->post_wait_ms = wait;
}

static u32 _iris_pt_short_write(
		struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS7_CHIP_VERSION ?
		IRIS_MIPI_TX_HEADER_ADDR : IRIS_MIPI_TX_HEADER_ADDR_I3;

	pheader->stHdr.longCmdFlag = 0x00;

	_iris_pt_set_wrcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: 0x%4x",
			__func__, __LINE__,
			pheader->hdr32);
	_iris_add_cmd_addr_val(pocp_cmd, address, pheader->hdr32);

	return sum;
}

static u32 _iris_pt_short_read(
		struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS7_CHIP_VERSION ?
		IRIS_MIPI_TX_HEADER_ADDR : IRIS_MIPI_TX_HEADER_ADDR_I3;

	pheader->stHdr.longCmdFlag = 0x00;
	_iris_pt_set_rdcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: 0x%4x",
			__func__, __LINE__,
			pheader->hdr32);
	_iris_add_cmd_addr_val(pocp_cmd, address, pheader->hdr32);

	return sum;
}

static u32 _iris_pt_get_split_pkt_len(u16 dlen, int sum, int k)
{
	u16 split_len = 0;

	if (k == 0)
		split_len = dlen <  IRIS_TX_HV_PAYLOAD_LEN
			? dlen : IRIS_TX_HV_PAYLOAD_LEN;
	else if (k == sum - 1)
		split_len = dlen - IRIS_TX_HV_PAYLOAD_LEN
			- (k - 1) * IRIS_TX_PAYLOAD_LEN;
	else
		split_len = IRIS_TX_PAYLOAD_LEN;

	return split_len;
}

static void _iris_pt_add_split_pkt_payload(
		struct iris_ocp_cmd *pocp_cmd, u8 *ptr, u16 split_len)
{
	u32 i = 0;
	union iris_mipi_tx_cmd_payload payload;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS7_CHIP_VERSION ?
		IRIS_MIPI_TX_PAYLOAD_ADDR : IRIS_MIPI_TX_PAYLOAD_ADDR_I3;

	memset(&payload, 0x00, sizeof(payload));
	for (i = 0; i < split_len; i += 4, ptr += 4) {
		if (i + 4 > split_len) {
			payload.pld32 = 0;
			memcpy(payload.p, ptr, split_len - i);
		} else
			payload.pld32 = *(u32 *)ptr;

		IRIS_LOGD("%s(), payload: %#x", __func__, payload.pld32);
		_iris_add_cmd_addr_val(pocp_cmd, address,
				payload.pld32);
	}
}

static u32 _iris_pt_long_write(
		struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd)
{
	u8 *ptr = NULL;
	u32 i = 0;
	u32 sum = 0;
	u16 dlen = 0;
	u32 split_len = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS7_CHIP_VERSION ?
		IRIS_MIPI_TX_HEADER_ADDR : IRIS_MIPI_TX_HEADER_ADDR_I3;

	dlen = dsi_cmd->msg.tx_len;

	pheader->stHdr.longCmdFlag = 0x1;
	_iris_pt_set_wrcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: %#x",
			__func__, __LINE__,
			pheader->hdr32);
	_iris_add_cmd_addr_val(pocp_cmd, address,
			pheader->hdr32);

	ptr = (u8 *)dsi_cmd->msg.tx_buf;
	sum = _iris_pt_get_split_pkt_cnt(dlen);

	while (i < sum) {
		ptr += split_len;
		split_len = _iris_pt_get_split_pkt_len(dlen, sum, i);
		_iris_pt_add_split_pkt_payload(pocp_cmd + i, ptr, split_len);

		i++;
		if (i < sum)
			_iris_pt_init_ocp_cmd(pocp_cmd + i);
	}
	return sum;
}

static u32 _iris_pt_add_cmd(
		struct dsi_cmd_desc *ptx_cmd, struct iris_ocp_cmd *pocp_cmd,
		struct dsi_cmd_desc *dsi_cmd, struct dsi_panel_cmd_set *cmdset)
{
	u32 i = 0;
	u16 dtype = 0;
	u32 sum = 0;
	u8 wait = 0;
	u16 flags = 0;
	union iris_mipi_tx_cmd_header header;

	_iris_pt_init_tx_cmd_hdr(cmdset, dsi_cmd, &header);

	dtype = iris_switch_cmd_type(dsi_cmd->msg.type);
	switch (dtype) {
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		sum = _iris_pt_short_read(pocp_cmd, &header, dsi_cmd);
		break;
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
		sum = _iris_pt_short_write(pocp_cmd, &header, dsi_cmd);
		break;
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
		sum = _iris_pt_long_write(pocp_cmd, &header, dsi_cmd);
		break;
	default:
		IRIS_LOGE("%s(), invalid type: %#x",
				__func__,
				dsi_cmd->msg.type);
		break;
	}

	for (i = 0; i < sum; i++) {
		wait = (i == sum - 1) ? dsi_cmd->post_wait_ms : 0;
		if (i != sum - 1)
			iris_set_msg_flags(dsi_cmd, BATCH_FLAG);
		flags = dsi_cmd->msg.flags;
		_iris_add_tx_cmds(ptx_cmd + i, pocp_cmd + i, wait, flags);
	}
	return sum;
}

static int _iris_pt_send_cmds(
		struct dsi_panel *panel,
		struct dsi_cmd_desc *ptx_cmds, u32 cmds_cnt)
{
	struct dsi_panel_cmd_set panel_cmds;
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;
	u8 vc_id = 0;
	memset(&panel_cmds, 0x00, sizeof(panel_cmds));

	panel_cmds.cmds = ptx_cmds;
	panel_cmds.count = cmds_cnt;
	panel_cmds.state = DSI_CMD_SET_STATE_HS;

	if (pcfg->vc_ctrl.vc_enable) {
		vc_id = (panel_cmds.state == DSI_CMD_SET_STATE_LP) ?
			pcfg->vc_ctrl.to_panel_lp_vc_id :
			pcfg->vc_ctrl.to_panel_hs_vc_id;
	}
	rc = iris_dsi_send_cmds(panel, panel_cmds.cmds,
			panel_cmds.count, panel_cmds.state, vc_id);

	if (iris_get_cont_splash_type() == IRIS_CONT_SPLASH_LK)
		iris_print_desc_cmds(panel_cmds.cmds, panel_cmds.count, panel_cmds.state);
	return rc;
}

static int _iris_pt_write_panel_cmd(
		struct dsi_panel *panel, struct dsi_panel_cmd_set *cmdset)
{
	u32 i = 0;
	u32 j = 0;
	int cmds_cnt = 0;
	u32 offset = 0;
	int rc = 0;
	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct dsi_cmd_desc *ptx_cmds = NULL;
	struct dsi_cmd_desc *dsi_cmds = NULL;

	if (!panel || !cmdset) {
		IRIS_LOGE("%s(), invalid panel or cmdset!", __func__);
		return -EINVAL;
	}

	if (cmdset->count == 0) {
		IRIS_LOGD("%s(), invalid cmdset count!", __func__);
		return -EINVAL;
	}

	cmds_cnt = _iris_pt_alloc_cmds(cmdset, &ptx_cmds, &pocp_cmds);
	if (cmds_cnt < 0) {
		IRIS_LOGE("%s(), invalid cmds count: %d", __func__, cmds_cnt);
		return -EINVAL;
	}

	for (i = 0; i < cmdset->count; i++) {
		/*initial val*/
		dsi_cmds = cmdset->cmds + i;
		_iris_pt_init_ocp_cmd(pocp_cmds + j);
		offset = _iris_pt_add_cmd(
				ptx_cmds + j, pocp_cmds + j, dsi_cmds, cmdset);
		j += offset;
	}

	if (j != (u32)cmds_cnt)
		IRIS_LOGE("%s(), invalid cmd count: %d, j: %d",
				__func__,
				cmds_cnt, j);
	else
		rc = _iris_pt_send_cmds(panel, ptx_cmds, (u32)cmds_cnt);

	vfree(pocp_cmds);
	vfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;
	return rc;
}

static void _iris_pt_switch_cmd(
		struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset,
		struct dsi_cmd_desc *dsi_cmd)
{
	if (!cmdset || !panel || !dsi_cmd) {
		IRIS_LOGE("%s(), invalid input param", __func__);
		return;
	}

	cmdset->cmds = dsi_cmd;
	cmdset->count = 1;
}

static int _iris_pt_write_max_pkt_size(
		struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset)
{
	u32 rlen = 0;
	int rc = 0;
	struct dsi_panel_cmd_set local_cmdset;
	static char max_pktsize[2] = {0x00, 0x00}; /* LSB tx first, 10 bytes */
	static struct dsi_cmd_desc_pxlw pkt_size_cmd_pxlw = {
		{0, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, MIPI_DSI_MSG_REQ_ACK,
			0, 0, sizeof(max_pktsize), max_pktsize, 0, NULL}, 1, 0};
	struct dsi_cmd_desc pkt_size_cmd;

	remap_to_qcom_style(&pkt_size_cmd, &pkt_size_cmd_pxlw, 1);

	rlen = cmdset->cmds[0].msg.rx_len;
	if (rlen > 128) {
		IRIS_LOGE("%s(), invalid len: %d", __func__, rlen);
		return -EINVAL;
	}

	max_pktsize[0] = (rlen & 0xFF);
	memset(&local_cmdset, 0x00, sizeof(local_cmdset));

	_iris_pt_switch_cmd(panel, &local_cmdset, &pkt_size_cmd);
	rc = _iris_pt_write_panel_cmd(panel, &local_cmdset);

	return rc;
}

static int _iris_pt_send_panel_rdcmd(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset)
{
	struct dsi_panel_cmd_set local_cmdset;
	struct dsi_cmd_desc *dsi_cmd = cmdset->cmds;
	int rc = 0;

	memset(&local_cmdset, 0x00, sizeof(local_cmdset));

	_iris_pt_switch_cmd(panel, &local_cmdset, dsi_cmd);

	/*passthrough write to panel*/
	rc = _iris_pt_write_panel_cmd(panel, &local_cmdset);
	return rc;
}

static int _iris_pt_remove_respond_hdr(char *ptr, int *offset)
{
	int rc = 0;
	char cmd;

	if (!ptr)
		return -EINVAL;

	cmd = ptr[0];
	IRIS_LOGV("%s(), cmd: 0x%02x", __func__, cmd);
	switch (cmd) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		IRIS_LOGD("%s(), rx ACK_ERR_REPORT", __func__);
		rc = -EINVAL;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		*offset = 1;
		rc = 1;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		*offset = 1;
		rc = 2;
		break;
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		*offset = 4;
		rc = ptr[1];
		break;
	default:
		rc = 0;
	}

	return rc;
}

static int _iris_pt_read(struct dsi_panel_cmd_set *cmdset, uint8_t path)
{
	u32 i = 0;
	u32 rlen = 0;
	u32 intstat = 0;
	int retry_cnt, frame_interval;
	u32 offset = 0;
	int rc = 0;
	union iris_mipi_tx_cmd_payload val;
	u8 *rbuf = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = 0;

	switch (pcfg->chip_ver) {
	case IRIS7_CHIP_VERSION:
	case IRIS5_CHIP_VERSION:
		address = IRIS_RD_PACKET_DATA;
		break;
	case IRIS3_CHIP_VERSION:
		address = IRIS_RD_PACKET_DATA_I3;
		break;
	default:
		IRIS_LOGE("chip version not supported!");
		break;
	}

	rbuf = (u8 *)cmdset->cmds[0].msg.rx_buf;
	rlen = cmdset->cmds[0].msg.rx_len;

	if (!rbuf || rlen <= 0) {
		IRIS_LOGE("%s(), rbuf: %p, rlen: %d", __func__, rbuf, rlen);
		return -EINVAL;
	}

	if (pcfg->panel->cur_mode->timing.refresh_rate != 0)
		frame_interval = 1000/pcfg->panel->cur_mode->timing.refresh_rate + 1;
	else
		frame_interval = 1000/60 + 1;

	retry_cnt = frame_interval * 2;
	IRIS_LOGD("%s, frame_interval is %d ms, retry_cnt %d", __func__, frame_interval, retry_cnt);

	/*read iris for data*/
	for (i = 0; i < retry_cnt; i++) {
		usleep_range(1000, 1001);
		intstat = iris_ocp_read(IRIS_TX_INTSTAT_RAW, cmdset->state);

		if (intstat & IRIS_TX_READ_RESPONSE_RECEIVED)
			break;
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
			IRIS_LOGI("%s retry: %d", __func__, (i + 1));
	}

	if (intstat & IRIS_TX_READ_ERR_MASK) {
		rc = -1;
		IRIS_LOGE("%s(), Tx read error 0x%x, rc: %d",
				  __func__, intstat, rc);
		return rc;
	}
	val.pld32 = iris_ocp_read(address, cmdset->state);

	rlen = _iris_pt_remove_respond_hdr(val.p, &offset);
	IRIS_LOGV("%s(), read len: %d", __func__, rlen);

	if (rlen <= 0) {
		rc = -1;
		IRIS_LOGE("%s(), failed to remove respond header, val: 0x%x, rlen: %d, rc: %d",
				  __func__, val.pld32, rlen, rc);
		return rc;
	}

	if (rlen <= 2) {
		if (offset < 4) {
			for (i = 0; i < rlen; i++)
				rbuf[i] = val.p[offset + i];
		} else {
			val.pld32 = iris_ocp_read(address, cmdset->state);
			for (i = 0; i < rlen; i++)
				rbuf[i] = val.p[i];
		}
	} else {
		int j = 0;
		int len = 0;
		int num = (rlen + 3) / 4;

		for (i = 0; i < num; i++) {
			len = (i == num - 1) ? rlen - 4 * i : 4;
			val.pld32 = iris_ocp_read(address, DSI_CMD_SET_STATE_HS);
			for (j = 0; j < len; j++)
				rbuf[i * 4 + j] = val.p[j];
		}
	}

	return rc;
}

int iris_pt_read_panel_cmd(struct dsi_panel *panel,
						   struct dsi_panel_cmd_set *cmdset)
{
	u8 vc_id = 0;
	int rc = 0;
	struct iris_cfg *pcfg = NULL;

	IRIS_LOGD("%s(), enter", __func__);

	if (!panel || !cmdset || cmdset->count != 1) {
		IRIS_LOGE("%s(), invalid input, cmdset: %p", __func__, cmdset);
		return -EINVAL;
	}

	pcfg = iris_get_cfg();


	if (!pcfg->vc_ctrl.vc_enable) {
		/*step1  write max packet size*/
		rc = _iris_pt_write_max_pkt_size(panel, cmdset);

		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			return rc;
		}
		iris_ocp_write_val(IRIS_TX_INTCLR, 0xFFFFFFFF);
		/*step2 write read cmd to panel*/
		rc = _iris_pt_send_panel_rdcmd(panel, cmdset);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			goto exit;
		}
		/*step3 read panel data*/
		rc = _iris_pt_read(cmdset, pcfg->read_path);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			goto exit;
		}
	} else {
		vc_id = (cmdset->state == DSI_CMD_SET_STATE_LP) ?
			pcfg->vc_ctrl.to_panel_lp_vc_id :
			pcfg->vc_ctrl.to_panel_hs_vc_id;
		rc = iris_dsi_send_cmds(panel, cmdset->cmds,
								cmdset->count, DSI_CMD_SET_STATE_HS, vc_id);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			goto exit;
		}
	}
exit:
	return rc;
}

int iris_pt_send_panel_cmd(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset)
{
	u8 vc_id = 0;
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	if (!cmdset || !panel) {
		IRIS_LOGE("%s(), invalid panel: %p or cmdset: %p",
			__func__, panel, cmdset);
		return -EINVAL;
	}

	if (cmdset->count == 1 && iris_is_read_cmd(&cmdset->cmds[0])) {
		ktime_t lp_ktime0;
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			lp_ktime0 = ktime_get();
		}
		rc = iris_pt_read_panel_cmd(panel, cmdset);
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			IRIS_LOGI("%s rc: %d, spend time %d us", __func__,
				rc,(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
		}
		return rc;
	}

	if (!pcfg->vc_ctrl.vc_enable) {
		IRIS_LOGD("using ocp type to panel");
		_iris_pt_write_panel_cmd(panel, cmdset);
	} else {
		vc_id = (cmdset->state == DSI_CMD_SET_STATE_LP) ?
			pcfg->vc_ctrl.to_panel_lp_vc_id :
			pcfg->vc_ctrl.to_panel_hs_vc_id;
		IRIS_LOGD("using aux channel %d", vc_id);
		iris_dsi_send_cmds(panel, cmdset->cmds,
			cmdset->count, DSI_CMD_SET_STATE_HS, vc_id);
	}

	return rc;
}

int iris_abyp_send_panel_cmd(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset)
{
	u8 vc_id = 0;

	if (!cmdset || !panel) {
		IRIS_LOGE("%s(), invalid panel: %p or cmdset: %p",
			__func__, panel, cmdset);
		return -EINVAL;
	}

	iris_dsi_send_cmds(panel, cmdset->cmds,
			cmdset->count, cmdset->state, vc_id);
	return 0;
}

void iris_set_pwil_mode(u8 mode, bool osd_enable, int state, bool commit)
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

int iris_platform_get(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return pcfg->platform_type;
}

void iris_dtg_eco(bool enable, bool chain)
{
	u32 *payload = NULL;

	IRIS_LOGI("%s: %d", __func__, enable);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x90, 2);
	if (!payload)
		return;
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

void iris_dsi_rx_mode_switch(uint8_t rx_mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;
	u32 ovs_dly_rfb;
	u32 *payload = NULL;

	IRIS_LOGI("%s: %d", __func__, rx_mode);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xf0, 3);
	if (!payload)
		return;
	if (rx_mode == DSI_OP_CMD_MODE)
		payload[0] |= 0x20001;
	else
		payload[0] &= ~0x20001;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0xf0, 0xf0, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x70, 3);
	if (!payload)
		return;
	if (rx_mode == DSI_OP_CMD_MODE)
		payload[0] &= ~0x4000;
	else
		payload[0] |= 0x4000;
	iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x70, 0x70, 1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0x90, 2);
	if (!payload)
		return;
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
		if (!payload)
			return;
		ovs_dly_rfb = payload[0];
		payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0xf8, 2);
		if (!payload)
			return;
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

void iris_dtg_update_reset(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;

	if ((pcfg->rx_mode == DSI_OP_VIDEO_MODE) && (pcfg->tx_mode == DSI_OP_VIDEO_MODE)) {
		regval.ip = IRIS_IP_DTG;
		regval.opt_id = 0xF0;
		regval.mask = 0x0000000F;
		regval.value = 0xF;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(IRIS_IP_DTG, 0xF0, 0xF0, 0);
	}
}

void iris_sw_te_enable(void)
{
	u32 *payload = NULL;
	u32 dtg_ctrl;
	u32 cmd[8];

	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, ID_DTG_TE_SEL, 2);
	if (!payload)
		return;
	dtg_ctrl = payload[0];
	cmd[0] = IRIS_DTG_ADDR + DTG_CTRL;
	cmd[1] = dtg_ctrl & ~0x800;
	cmd[2] = IRIS_DTG_ADDR + DTG_UPDATE;
	cmd[3] = 0x1;
	cmd[4] = IRIS_DTG_ADDR + DTG_CTRL;
	cmd[5] = dtg_ctrl | 0x800 | 0x14;
	cmd[6] = IRIS_DTG_ADDR + DTG_UPDATE;
	cmd[7] = 0x1;
	iris_ocp_write_mult_vals(8, cmd);
}

void iris_ovs_dly_change(bool enable)
{
	u32 *payload = NULL;
	u32 ovs_dly_pt, ovs_dly_rfb;
	u32 cmd[4];

	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0x00, 2);
	if (!payload)
		return;
	ovs_dly_pt = payload[15];
	payload = iris_get_ipopt_payload_data(IRIS_IP_DTG, 0xf5, 2);
	if (!payload)
		return;
	ovs_dly_rfb = payload[0];

	cmd[0] = IRIS_DTG_ADDR + OVS_DLY;
	if (enable)
		cmd[1] = ovs_dly_rfb;
	else
		cmd[1] =  ovs_dly_pt;
	cmd[2] = IRIS_DTG_ADDR + DTG_UPDATE;
	cmd[3] = 0x2;
	iris_ocp_write_mult_vals(4, cmd);
}
