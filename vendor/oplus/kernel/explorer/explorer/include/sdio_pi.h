/*
 * Copyright (c) 2021 ZEKU Technology(Shanghai) Corp.,Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Basecode Created :        2020/09/28 Author: wangman@zeku.com
 *
 */

#ifndef _EXPLORER_SDIO_PI_H
#define _EXPLORER_SDIO_PI_H

#define SDIO_VENDOR_ID 0x1919
#define SDIO_DEVICE_ID 0x9066
#define SDIO_ATU_CONFIG_NUM 64
#define SDIO_DRIVER_NAME "explorer_sdio"

#define CMD_RETRIES 3

#define FUNC0	0
#define FUNC1	1

#include <linux/list.h>
#include "sdio_zk.h"

/* In byte mode, the upper 12 bits of the device reorganized address are 0.
 * In block mode, the upper 3 bits and the low 9 bits of the device reoriganized address are 0.
 * Cause cmd52 can only modify 8 bits
 */
struct atu_reg_s {
	/* [7:0] */
	unsigned int atu0:8;
	/* [15:8] */
	unsigned int atu1:8;
	/* [23:16] */
	unsigned int atu2:8;
	/* [31:24] */
	unsigned int atu3:8;
};

union atu_reg_u {
	struct atu_reg_s item;
	unsigned int value;
};

/* Cause cmd52 can only modify 8 bits */
struct remap_reg_s {
	/* [7:0] */
	unsigned int remap0:8;
	/* [15:8] */
	unsigned int remap1:8;
	/* [23:16] */
	unsigned int remap2:8;
	/* [31:24] */
	unsigned int remap3:8;
};

union remap_reg_u{
	struct remap_reg_s item;
	unsigned int value;
};

struct atu_remap_item {
	u32 atu;
	u32 remap;
};

struct batch_data_item {
	u32 addr;
	u32 data;
};

struct current_atu_list {
	unsigned int index;
	struct list_head list;
};


/* ATU0-ATU13 is in use */
/* set ATU14 for ipc */
#define BYTE_FREE_LIST_INIT_STATE	0xffff8000
/* ATU32-ATU55 is in use */
#define BLOCK_FREE_LIST_INIT_STATE	0xff000000

/* index needs to be unsigned int */
#define INDEX_TO_FREE_STATE(index, byte)	(byte?(0x1 << (index)):(0x1 << (index - 32)))
#define GET_FREE_STATE_TO_INDEX(state, byte)	(byte?(ffs(state)-1):(ffs(state) + 31))
#define IS_BYTE_MODE(index)		((index) <= 31)
/* the type of pdata should be (struct explorer_sdio_data *) */
#define GET_FREE_LIST_ADDR(pdata, byte) (byte?(&(pdata)->byte_free_list):(&(pdata)->block_free_list))
#define GET_CUR_ATU_NUM_ADDR(pdata, byte) (byte?(&(pdata)->byte_current_atu_num):(&(pdata)->block_current_atu_num))

/*tuning fail*/
#define TUNING_FAIL_RECOVERABLE 1

struct explorer_sdio_data {
	unsigned int            index;
	struct sdio_func        *func;

	/* for atu and remap registers */
	/* 1. atu table */
	union atu_reg_u         atu_reg[64];
	union remap_reg_u       remap_reg[64];
	/* 2. current list */
	struct list_head byte_current_list;
	unsigned int byte_current_atu_num;
	struct list_head block_current_list;
	unsigned int block_current_atu_num;
	/* 3. free list */
	unsigned int	byte_free_list;
	unsigned int	block_free_list;

	/* The address of the data buffer should be continuous */
	u8 data[64];
};

int explorer_sdio_init(void);
void explorer_sdio_exit(void);
int sdio_check_card(void);


/*for CMD52 or CMD53*/
int sdio_read_data(struct explorer_sdio_data *sdio_data, unsigned int addr, void *buffer, int count);
int sdio_write_data(struct explorer_sdio_data *sdio_data, unsigned int addr, void *buffer, int count);

int sdio_batch_write_data(struct explorer_sdio_data *sdio_data, struct batch_data_item *src, int count);

int sdio_read_register(struct explorer_sdio_data *sdio_data, unsigned int regoffset, unsigned char *out, unsigned int funcnum);

int sdio_write_register(struct explorer_sdio_data *sdio_data, unsigned int regoffset, unsigned char input, unsigned int funcnum);
int sdio_read_register_no_claim(struct explorer_sdio_data *sdio_data, unsigned int regoffset, unsigned char *out, unsigned int funcnum);
int sdio_write_register_no_claim(struct explorer_sdio_data *sdio_data, unsigned int regoffset, unsigned char input, unsigned int funcnum);

int sdio_get_sdclk(struct explorer_sdio_data *sdio_data);
int sdio_set_sdclk(struct explorer_sdio_data *sdio_data, int sdclk_value);
int explorer_sdio_execute_tuning(struct explorer_sdio_data *pdata);

#endif

