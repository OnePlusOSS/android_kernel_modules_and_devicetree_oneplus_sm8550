// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */


#include <linux/i2c.h>
#include "dsi_display.h"
#include "dsi_iris_api.h"
#include "dsi_iris_i3c.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_log.h"

#define IRIS_COMPATIBLE_NAME  "pixelworks,iris"
#define IRIS_I2C_DRIVER_NAME  "pw-i3c"

/*iris i2c handle*/
static struct i2c_client  *iris_i2c_handle;

static void __iris_i2c_buf_init(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	memset(pcfg->iris_i2c_cfg.buf, 0x00, IRIS_I2C_BUF_LEN);
	pcfg->iris_i2c_cfg.buf_index = 0;
}

static void __iris_i2c_buf_reset(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->iris_i2c_cfg.buf_index = 0;
}


static uint8_t *__iris_i2c_alloc_buf(uint32_t byte_len)
{
	uint8_t *pbuf = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGV("%s: buf = %p, index = %d, byte_len = %d", __func__,
		pcfg->iris_i2c_cfg.buf, pcfg->iris_i2c_cfg.buf_index, byte_len);

	if ((pcfg->iris_i2c_cfg.buf_index + byte_len) > IRIS_I2C_BUF_LEN) {
		IRIS_LOGE("%s(): can not alloc i2c buf\n", __func__);
		return NULL;
	}

	pbuf = pcfg->iris_i2c_cfg.buf;
	pbuf += pcfg->iris_i2c_cfg.buf_index;
	pcfg->iris_i2c_cfg.buf_index += byte_len;

	IRIS_LOGV("%s: buf = %p, index = %d", __func__, pbuf, pcfg->iris_i2c_cfg.buf_index);

	return pbuf;
}


static void __iris_i2c_fill_buf(struct i3c_protocol_t *ph, u8 *pbuf)
{

	uint16_t i, len;

	len = ph->header.byte_len / sizeof(uint32_t);
	if (ph->header.type == I3C_OCP_READ_OP)
		len = 2;

	pbuf[0] = ph->header.val & 0xff;
	pbuf[1] = (ph->header.val >> 8) & 0xff;
	pbuf[2] = (ph->header.val >> 16) & 0xff;
	pbuf[3] = (ph->header.val >> 24) & 0xff;

	for (i = 1; i < len; i++) {
		pbuf[i*4] = ((ph->tx_buf[i-1]) & 0xff);
		pbuf[i*4 + 1] = ((ph->tx_buf[i-1] >> 8) & 0xff);
		pbuf[i*4 + 2] = ((ph->tx_buf[i-1] >> 16) & 0xff);
		pbuf[i*4 + 3] = ((ph->tx_buf[i-1] >> 24) & 0xff);
	}

}

static void __iris_convert_dsi_to_i3c(uint8_t *payload, uint32_t len)
{
	uint8_t slot;
	uint32_t header, address, offset;
	union i3c_header_t conn;
	uint32_t *pval = (uint32_t *)payload;

	conn.val = 0;
	header = cpu_to_le32(pval[0]);
	address = cpu_to_le32(pval[1]);
	IRIS_LOGD("%s,%d: header = 0x%08x, address = 0x%08x", __func__, __LINE__, pval[0], pval[1]);

	if ((header & 0xf) == 0xc) {
		conn.type = I3C_OCP_BURST_WR_OP;
		conn.slot = 0x000;
		conn.byte_len = len & 0xffff;

		slot = (header >> 24) & 0xf;
		switch (slot) {
		case 0:
			if (address < 0xF1090000) {
				offset = address - 0;
				address = 0xF1090000 + offset;
			}
			break;
		case 1:
			offset = address - 0;
			address = 0xF16C0400 + offset;
			break;
		case 2:
			offset = address - 0;
			address = 0xF1607200 + offset;
			break;
		case 3:
			if ((address >= 0x7c00) && (address < 0x8000)) {
				offset = address - 0x7c00;
				address = 0xF1607c00 + offset;
			} else if ((address >= 0x8000) && (address < 0x28000)) {
				offset = address - 0x8000;
				address = 0xF1608000 + offset;
			} else if (address >= 0x28000) {
				offset = address - 0x28000;
				address = 0xF1640400 + offset;
			} else {
				IRIS_LOGE("%s(): invalid address in slot 3\n", __func__);
				return;
			}
			break;
		case 4:
			offset = address - 0;
			address = 0xF1606800 + offset;
			break;
		case 5:
			offset = address - 0;
			address = 0xF1680400 + offset;
			break;
		default:
			IRIS_LOGE("%s(): invalid direct bus slot num %d\n", __func__, slot);
			return;
		}
		pval[0] = conn.val;
		pval[1] = address;
	} else if ((header & 0xf) == 0x0) {
		conn.type = I3C_OCP_BURST_WR_OP;
		conn.slot = 0x000;
		conn.byte_len = len & 0xffff;
		pval[0] = conn.val;
	} else if ((header & 0xf) == 0x4) {
		conn.type = I3C_OCP_SINGLE_WR_OP;
		conn.slot = 0xFFF;
		conn.byte_len = len & 0xffff;
		pval[0] = conn.val;
	}
	IRIS_LOGD("%s,%d: header = 0x%08x, address = 0x%08x", __func__, __LINE__, pval[0], pval[1]);
}

int iris_ioctl_i2c_read(uint32_t addr, uint32_t *pval)
{
	int ret = -1;

	ret = iris_i2c_read(addr, pval, 1);

	return ret;
}

int iris_ioctl_i2c_write(uint32_t addr, uint32_t val)
{
	int ret = -1;
	uint32_t buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	ret = iris_i2c_write(buf, 1, I3C_OCP_SINGLE_WR_OP, 0);

	return ret;
}

int iris_ioctl_i2c_burst_write(uint32_t addr, uint32_t *pval, uint16_t reg_num)
{
	int ret = -1;
	uint32_t *addr_val = NULL;

	// sub header and address
	if ((reg_num == 0) || (reg_num > (I2C_MSG_MAX_LEN-8)/4)) {
		IRIS_LOGE("%s: reg_num equal to 0 or too long\n", __func__);
		return -EINVAL;
	}

	addr_val = vmalloc((reg_num + 1) * sizeof(uint32_t));
	if (addr_val == NULL) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	addr_val[0] = addr;
	memcpy(&addr_val[1], pval, reg_num * sizeof(uint32_t));

	ret = iris_i2c_write(addr_val, reg_num, I3C_OCP_BURST_WR_OP, 0);

	vfree(addr_val);
	return ret;
}

int iris_i2c_read(uint32_t addr, uint32_t *pval, uint32_t reg_num)
{
	int i;
	int ret = -1;
	uint8_t tx_payload[8];
	uint8_t *rx_payload = NULL;
	uint16_t byte_count = 0;
	struct i2c_msg msgs[2];
	struct i3c_protocol_t ocp_rd_op;

	if ((reg_num == 0) || (reg_num > (I2C_MSG_MAX_LEN)/4)) {
		IRIS_LOGE("%s: reg_num equal to 0 or too long\n", __func__);
		return -EINVAL;
	}

	byte_count = reg_num * sizeof(uint32_t);
	memset(msgs, 0x00, 2 * sizeof(msgs[0]));
	memset(&ocp_rd_op, 0x00, sizeof(ocp_rd_op));
	memset(tx_payload, 0x00, 8);

	ocp_rd_op.header.type = I3C_OCP_READ_OP;
	ocp_rd_op.header.slot = 0x000;
	ocp_rd_op.header.byte_len = byte_count;
	ocp_rd_op.tx_buf = &addr;

	__iris_i2c_buf_reset();
	rx_payload = __iris_i2c_alloc_buf(byte_count);
	if (rx_payload == NULL) {
		__iris_i2c_buf_reset();
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	__iris_i2c_fill_buf(&ocp_rd_op, tx_payload);
	memset(rx_payload, 0x00, byte_count);

	msgs[0].addr = (iris_i2c_handle->addr) & 0xff;
	msgs[0].flags = 0;
	msgs[0].buf = tx_payload;
	msgs[0].len = 8;

	msgs[1].addr = (iris_i2c_handle->addr) & 0xff;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = rx_payload;
	msgs[1].len = byte_count;

	ret = i2c_transfer(iris_i2c_handle->adapter, &msgs[0], 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s, %d: i2c_transfer failed, write cmd, addr = 0x%08x, ret = %d\n",
			__func__, __LINE__, addr, ret);
	}

	if (ret == 0) {
		udelay(20);
		ret = i2c_transfer(iris_i2c_handle->adapter, &msgs[1], 1);
		if (ret == 1) {
			ret = 0;
		} else {
			ret = ret < 0 ? ret : -EIO;
			IRIS_LOGE("%s, %d: i2c_transfer failed, read cmd, addr = 0x%08x, ret = %d\n",
				__func__, __LINE__, addr, ret);
		}
	}

	if (ret == 0) {
		for (i = 0; i < reg_num; i++) {
			pval[i] = (rx_payload[i * 4] << 0) |
				(rx_payload[i * 4 + 1] << 8) |
				(rx_payload[i * 4 + 2] << 16)|
				(rx_payload[i * 4 + 3] << 24);
		}
	}

	__iris_i2c_buf_reset();
	rx_payload = NULL;

	return ret;

}

int iris_i2c_write(uint32_t *addr_val, uint32_t reg_num, enum i3c_op_type type, uint8_t dsi_slot)
{

	int ret = -1;
	uint8_t *payload = NULL;
	uint32_t byte_len = 0;
	struct i2c_msg msg;
	struct i3c_protocol_t ocp_wr_op;

	if (addr_val == NULL) {
		IRIS_LOGE("%s, %d: pbuf is NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if ((reg_num == 0) || (reg_num > (I2C_MSG_MAX_LEN-8)/4)) {
		IRIS_LOGE("%s: reg_num equal to 0 or too long\n", __func__);
		return -EINVAL;
	}

	memset(&msg, 0x00, sizeof(msg));
	memset(&ocp_wr_op, 0x00, sizeof(ocp_wr_op));

	if (type == I3C_OCP_SINGLE_WR_OP)
		byte_len = (2 * reg_num + 1) * sizeof(uint32_t);
	else if ((type == I3C_OCP_BURST_WR_OP) || (type == I3C_DIRECT_WR_OP))
		byte_len = (reg_num + 2) * sizeof(uint32_t);
	else
		return -EINVAL;

	ocp_wr_op.header.type = type;
	ocp_wr_op.header.slot = 0xFFF;
	ocp_wr_op.header.byte_len = byte_len;
	ocp_wr_op.tx_buf = addr_val;

	if (type == I3C_DIRECT_WR_OP)
		ocp_wr_op.header.slot = 1 << dsi_slot;

	__iris_i2c_buf_reset();
	payload = __iris_i2c_alloc_buf(byte_len);
	if (!payload) {
		__iris_i2c_buf_reset();
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	__iris_i2c_fill_buf(&ocp_wr_op, payload);

	msg.addr = (iris_i2c_handle->addr) & 0xff;
	msg.flags = 0;
	msg.buf = payload;
	msg.len = ocp_wr_op.header.byte_len;

	ret = i2c_transfer(iris_i2c_handle->adapter, &msg, 1);

	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s: i2c_transfer failed, ret=%d\n", __func__, ret);
	}

	__iris_i2c_buf_reset();
	payload = NULL;

	return ret;
}

int iris_i2c_bit_en_op(uint32_t addr, uint32_t bit_en, uint32_t val)
{

	int ret = -1;
	uint8_t payload[16] = {0};
	uint32_t tbuf[3] = {0};
	struct i2c_msg msg;
	struct i3c_protocol_t bit_en_op;

	memset(&msg, 0x00, sizeof(msg));
	memset(&bit_en_op, 0x00, sizeof(bit_en_op));

	bit_en_op.header.type = I3C_OCP_BIT_EN_OP;
	bit_en_op.header.slot = 0xFFF;
	bit_en_op.header.byte_len = 16;

	bit_en_op.tx_buf = tbuf;
	bit_en_op.tx_buf[0] = addr;
	bit_en_op.tx_buf[1] = bit_en;
	bit_en_op.tx_buf[2] = val;

	__iris_i2c_fill_buf(&bit_en_op, payload);

	msg.addr = (iris_i2c_handle->addr) & 0xff;
	msg.flags = 0;
	msg.buf = payload;
	msg.len = bit_en_op.header.byte_len;

	ret = i2c_transfer(iris_i2c_handle->adapter, &msg, 1);

	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s: i2c_transfer failed, ret=%d\n", __func__, ret);
	}

	bit_en_op.tx_buf = NULL;

	return ret;
}

int iris_i2c_multi_write(struct iris_i2c_msg *dsi_msg, uint32_t msg_num)
{

	int ret = -1;
	uint32_t i = 0;
	uint32_t byte_count = 0;
	uint32_t total_len = 0;
	struct i2c_msg *i2c_msg;

	if ((dsi_msg == NULL) || (msg_num == 0)) {
		IRIS_LOGE("%s, %d: pbuf is NULL or num = 0\n", __func__, __LINE__);
		return -EINVAL;
	}

	for (i = 0; i < msg_num; i++) {
		if (dsi_msg[i].len > I2C_MSG_MAX_LEN) {
			IRIS_LOGE("%s: msg len exceed max i2c xfer len\n", __func__);
			return -EINVAL;
		}

		total_len += dsi_msg[i].len;
		if (total_len > IRIS_I2C_BUF_LEN) {
			IRIS_LOGE("%s: total len exceed max i2c buf len\n", __func__);
			return -EINVAL;
		}
	}

	i2c_msg = vmalloc(sizeof(struct i2c_msg) * msg_num);
	if (!i2c_msg) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		return -ENOMEM;
	}

	__iris_i2c_buf_reset();
	for (i = 0; i < msg_num; i++) {
		byte_count = dsi_msg[i].len;
		i2c_msg[i].buf = __iris_i2c_alloc_buf(byte_count);
		if (!i2c_msg[i].buf) {
			IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
			ret = -ENOMEM;
			goto error;
		}

		i2c_msg[i].addr = (iris_i2c_handle->addr) & 0xff;
		i2c_msg[i].flags = 0;
		i2c_msg[i].len = byte_count;
		memcpy(i2c_msg[i].buf, dsi_msg[i].buf, byte_count);
		__iris_convert_dsi_to_i3c(i2c_msg[i].buf, byte_count);
	}

	ret = i2c_transfer(iris_i2c_handle->adapter, i2c_msg, msg_num);
	if (ret == msg_num) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		IRIS_LOGE("%s: i2c_transfer failed, ret=%d\n", __func__, ret);
	}

error:
	__iris_i2c_buf_reset();
	vfree(i2c_msg);
	i2c_msg = NULL;
	return ret;
}

static int iris_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_i2c_handle = client;
	IRIS_LOGI("%s,%d: %p\n", __func__, __LINE__, iris_i2c_handle);

	pcfg->iris_i2c_cfg.buf = kmalloc(IRIS_I2C_BUF_LEN, GFP_KERNEL);
	if (!pcfg->iris_i2c_cfg.buf) {
		IRIS_LOGE("%s, %d: allocate memory fails\n", __func__, __LINE__);
		rc = -ENOMEM;
		return rc;
	}

	IRIS_LOGI("%s,%d: %p\n", __func__, __LINE__, pcfg->iris_i2c_cfg.buf);
	__iris_i2c_buf_init();

	return 0;
}

static int iris_i2c_remove(struct i2c_client *client)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_i2c_handle = NULL;
	kfree(pcfg->iris_i2c_cfg.buf);
	pcfg->iris_i2c_cfg.buf = NULL;
	pcfg->iris_i2c_cfg.buf_index = 0;

	return 0;
}

static const struct i2c_device_id iris_i2c_id_table[] = {
	{IRIS_I2C_DRIVER_NAME, 0},
	{},
};

static const struct of_device_id iris_match_table[] = {
	{.compatible = IRIS_COMPATIBLE_NAME,},
	{ },
};

static struct i2c_driver plx_i2c_driver = {
	.driver = {
		.name = IRIS_I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = iris_match_table,
	},
	.probe = iris_i2c_probe,
	.remove =  iris_i2c_remove,
	.id_table = iris_i2c_id_table,
};

int iris_i2c_bus_init(void)
{
	int ret;

	IRIS_LOGD("%s()\n", __func__);
	iris_i2c_handle = NULL;
	ret = i2c_add_driver(&plx_i2c_driver);
	if (ret != 0)
		IRIS_LOGE("iris i2c add driver fail: %d\n", ret);
	return 0;
}

void iris_i2c_bus_exit(void)
{
	i2c_del_driver(&plx_i2c_driver);
	iris_i2c_remove(iris_i2c_handle);
}
