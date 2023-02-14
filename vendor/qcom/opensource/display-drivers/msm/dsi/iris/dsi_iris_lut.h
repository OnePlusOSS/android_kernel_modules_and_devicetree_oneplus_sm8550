/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_LUT_H_
#define _DSI_IRIS_LUT_H_
#include <linux/firmware.h>

//iris7, coef includes crstk's coef and pre_csc's coef
#define CRSTK_COEF_SIZE        (18*3*2+9*3*2)  //crstk0_coef + crstk1_coef (uint16) * 3 for each group
#define CRSTK_COEF_GROUP       (9) // 11   //golden has 11 groups, calibrated would have 9 groups
#define CCT_VALUE_SIZE          (3*2) //warm, standard & cool

enum {
	LOAD_CALIBRATED_OR_GOLDEN = 0,
	LOAD_GOLDEN_ONLY,
	LOAD_CALIBRATED_ONLY,
	LOAD_METHOD_CNT
};

int iris_parse_lut_cmds(uint32_t flag);
int iris_send_lut(u8 lut_type, u8 lut_table_index);
void iris_update_ambient_lut(enum LUT_TYPE lutType, u32 lutPos);
void iris_update_maxcll_lut(enum LUT_TYPE lutType, u32 lutpos);
u8 iris_get_fw_load_status(void);
void iris_update_fw_load_status(u8 value);
void iris_update_gamma(void);
int iris_dbgfs_fw_calibrate_status_init(void);
void iris_crst_coef_check(const u8 *fw_data, size_t fw_size);
u16 iris_get_firmware_aplstatus_value(void);
int32_t iris_request_firmware(const struct firmware **fw,
		const uint8_t *name);
void iris_release_firmware(const struct firmware **fw);

#endif // _DSI_IRIS_LUT_H_
