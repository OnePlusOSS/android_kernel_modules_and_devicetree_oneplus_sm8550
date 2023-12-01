/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2022, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2022.
 */
#ifndef _IRIS_DTS_FW_H_
#define _IRIS_DTS_FW_H_

struct iris_dts_ops;

bool iris_is_printable_string(const void *data, int len);

void iris_print_data(const uint8_t *data, int len);

void iris_blob(const uint8_t *blob, uint32_t size);

struct iris_dts_ops *iris_get_dts_ops(void);
void iris_set_dts_ops(int id);
int iris_parse_dts_ctx(const uint8_t *fw_name);

#endif
