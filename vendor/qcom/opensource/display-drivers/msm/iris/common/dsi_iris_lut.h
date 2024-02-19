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
#define CRSTK_COEF_GROUP_I7P       (7)
#define CRSTK_COEF_GROUP_I7        (9)
#define CCT_VALUE_SIZE          (3*2) //warm, standard & cool

#define MAX_FW_NAME_LEN  256

enum {
	LOAD_CALIBRATED_OR_GOLDEN = 0,
	LOAD_GOLDEN_ONLY,
	LOAD_CALIBRATED_ONLY,
	LOAD_METHOD_CNT
};

enum {
	LOAD_FW_FAIL = 0,
	LOAD_GOLDEN_FW,
	LOAD_CALIBRATED_FW
};

extern u8 fw_loaded_status;
extern u16 fw_calibrated_status;
extern u16 gamma_apl_status;

int iris_parse_lut_cmds(uint32_t flag);
int iris_parse_lut_cmds_i7(uint32_t flag);
int iris_parse_lut_cmds_i7p(uint32_t flag);
int iris_change_dpp_lutrgb_type_addr(void);
int iris_send_lut(u8 lut_type, u8 lut_table_index);
int iris_send_lut_i7(u8 lut_type, u8 lut_table_index);
int iris_send_lut_i7p(u8 lut_type, u8 lut_table_index);
u8 iris_get_fw_load_status(void);
void iris_update_fw_load_status(u8 value);
void iris_update_gamma(void);
int iris_dbgfs_fw_calibrate_status_init(void);
u16 iris_get_firmware_aplstatus_value(void);
int32_t iris_request_firmware(const struct firmware **fw,
		const uint8_t *name);
void iris_release_firmware(const struct firmware **fw);
void iris_init_lut_buf(void);
int iris_send_lut_for_dma(void);
void iris_parse_misc_info(void);
void iris_fomat_lut_cmds(u8 lut_type, u8 opt_id);
int iris_change_lut_type_addr(
		struct iris_ip_opt *dest, struct iris_ip_opt *src);
void iris_update_ambient_lut(enum LUT_TYPE lutType, u32 lutPos);
void iris_update_maxcll_lut(enum LUT_TYPE lutType, u32 lutpos);
#endif // _DSI_IRIS_LUT_H_
