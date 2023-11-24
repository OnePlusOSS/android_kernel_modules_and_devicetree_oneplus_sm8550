// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2023, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2023.
 */

#include "dsi_iris_api.h"
#include "dsi_iris.h"
#include "dsi_iris_log.h"

enum iris_chip_type iris_get_chip_type(void)
{
	u32 chip_capabilities = 0;
	enum iris_chip_type rc = CHIP_UNKNOWN;

	chip_capabilities = iris_get_chip_caps();
	switch ((chip_capabilities >> 8) & 0xFF) {
	case 0x5:
		rc = CHIP_IRIS5;
		break;
	case 0x6:
		rc = CHIP_IRIS6;
		break;
	case 0x7:
		if (((chip_capabilities >> 30) & 0x3) > 1)
			rc = CHIP_IRIS7P;
		else
			rc = CHIP_IRIS7;
		break;
	default:
		rc = CHIP_UNKNOWN;
		IRIS_LOGE("%s(), chip type is unknown. chip_capabilities: 0x%x", __func__, chip_capabilities);
		break;
	}

	return rc;
}

int iris_need_update_pps_one_time(void)
{
	int rc = 0;
	static int flag = 1;

	if (flag == 1 && iris_get_chip_type() == CHIP_IRIS7P) {
		rc = 1;
		flag = 0;
		IRIS_LOGI("Iris force update pps for one time!");
	}
	return rc;
}
