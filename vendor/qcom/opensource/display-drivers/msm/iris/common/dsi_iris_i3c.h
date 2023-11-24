/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef DSI_IRIS_I3C_H
#define DSI_IRIS_I3C_H

#define I2C_MSG_MAX_LEN (65535)
#define IRIS_I2C_BUF_LEN (256*1024)  //256k bytes

#define SYS_I3C_CTRL_ADDR (0xF0000100)

enum i3c_op_type {
	I3C_OCP_BURST_WR_OP = 0x0,
	I3C_OCP_SINGLE_WR_OP = 0x4,
	I3C_DIRECT_WR_OP = 0xc,
	I3C_OCP_BIT_EN_OP = 0x5,
	I3C_OCP_READ_OP = 0x8,
};

enum PATH_TYPE {
	PATH_I2C = 0,
	PATH_DSI,
};

union i3c_header_t {
	u32 val;
	struct {
		u32 type	: 4;
		u32 slot	: 12;
		u32 byte_len: 16;   //header + addr + payload
	};
};

struct i3c_protocol_t {
	union i3c_header_t header;
	u32 *tx_buf;
	u32 *rx_buf;
};

struct iris_i2c_msg {
	uint8_t *buf;
	uint32_t len;
};

int iris_i2c_read(uint32_t addr, uint32_t *pval, uint32_t reg_num);
int iris_i2c_write(uint32_t *addr_val, uint32_t reg_num, enum i3c_op_type type, uint8_t dsi_slot);
int iris_i2c_bit_en_op(uint32_t addr, uint32_t bit_en, uint32_t val);

int iris_ioctl_i2c_read(uint32_t addr, uint32_t *pval);
int iris_ioctl_i2c_write(uint32_t addr, uint32_t val);
int iris_ioctl_i2c_burst_write(uint32_t addr, uint32_t *pval, uint16_t reg_num);

int iris_i2c_multi_write(struct iris_i2c_msg *msg, uint32_t msg_num);

#endif
