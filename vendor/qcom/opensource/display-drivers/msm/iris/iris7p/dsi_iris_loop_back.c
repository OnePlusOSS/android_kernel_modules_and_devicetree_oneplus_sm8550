// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include "dsi_iris_api.h"
#include "dsi_iris_i2c.h"
#include "dsi_iris_loop_back.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_log.h"
#include "dsi_iris_lp.h"

enum LOOP_BACK_ERR_TYPE {
	ERR_NO_ERR	= 0,
	ERR_INTERNAL_PT = 1,
};

#define BIT_INTERNAL_PT		(1 << 0)

static uint32_t iris_loop_back_flag_i7p = (BIT_INTERNAL_PT);

void iris_set_loopback_flag_i7p(uint32_t val)
{
	iris_loop_back_flag_i7p = val;
}

uint32_t iris_get_loopback_flag_i7p(void)
{
	return iris_loop_back_flag_i7p;
}

/**
 *  internal pt loop back, case t055917
 */
#define INTERNAL_PT_REG_NUM  (52)
static uint32_t internal_pt_op_addr[INTERNAL_PT_REG_NUM] = {
0xf0000068, 0xf0000068,
0xf0000000, 0xf00000f8, 0xf00000c8, 0xf0000048, 0xf0000000,
0xf0000020, 0xf0010040, 0xf0000004, 0xf0000008, 0xf0000008,
0xf0000024, 0xf0000028, 0xf0000028, 0xf0010040, 0xf000004c,
0xf0000050, 0xf0000048, 0xf0010040, 0xf1300004, 0xf1300010,
0xf1300014, 0xf1300018, 0xf130001c, 0xf1300054, 0xf1300050,
0xf130000c, 0xf1300100, 0xf13000f8, 0xf13000e0, 0xf13c0028,
0xf13c002c, 0xf13c0030, 0xf13c0034, 0xf13c006c, 0xf13c0068,
0xf13c0004, 0xf159ff00, 0xf1500024, 0xf1500020, 0xf1500004,
0xf1500014, 0xf151ff00, 0xf1600000, 0xf1600018, 0xf160006c,
0xf1600008, 0xf160000c, 0xf16001c4, 0xf160106c, 0xf1601068
};

static uint32_t internal_pt_op_val[INTERNAL_PT_REG_NUM] = {
0x00000001, 0x0000001f,
0x00000000, 0x00000644, 0x00000000, 0x00000000, 0x00000000,
0x00000000, 0x00000020, 0x004a0239, 0x0000f811, 0x0000f011,
0x002a0238, 0x0000f811, 0x0000f011, 0x000000ff, 0x03331100,
0x00000000, 0x0002d490, 0x00000020, 0x80078030, 0x0e025901,
0xd1002000, 0x0c000700, 0x5c066706, 0x346cf43b, 0xb42b342b,
0x1c021c02, 0x00000000, 0x00000010, 0x005be021, 0x0e025901,
0xd1002000, 0x0c000700, 0x5c066706, 0x346cf43b, 0xb42b342b,
0x04ac0003, 0x00000001, 0x000a0000, 0x020308cc, 0x143c2085,
0x0007841e, 0x00000100, 0x0001400c, 0x021c021c, 0x00000aaa,
0x84400052, 0x00400500, 0x00000003, 0x00000200, 0x00000037
};

static uint32_t internal_pt_op_delay[INTERNAL_PT_REG_NUM] = {
0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0
};

static uint32_t internal_pt_checksum[3] = {0x4178cadf, 0x3116b3b1, 0x37850768};

static int iris_internal_pt_verify(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int ret = 0;
	int i = 0;
	uint32_t statis[3] = {0};

	IRIS_LOGD("%s(%d), start.", __func__, __LINE__);
	iris_set_esd_status(false);
	iris_loop_back_reset();
	mdelay(100);
	mutex_lock(&pcfg->panel->panel_lock);
	iris_set_two_wire0_enable();
	mutex_unlock(&pcfg->panel->panel_lock);

	for (i = 0; i < INTERNAL_PT_REG_NUM; i++) {
		ret = pcfg->iris_i2c_write(internal_pt_op_addr[i], internal_pt_op_val[i]);
		if (ret) {
			IRIS_LOGE("%s(%d), error, i = %d, ret = %d.", __func__, __LINE__, i, ret);
			return 0x11;
		}

		if (internal_pt_op_delay[i])
			mdelay(internal_pt_op_delay[i]);
	}

	mdelay(100);
	for (i = 0; i < 3; i++) {
		ret = pcfg->iris_i2c_read(0xf1600200 + i*4, &statis[i]);
		if (ret) {
			IRIS_LOGE("%s(%d), fail, i2c read statis fail.", __func__, __LINE__);
			return 0x12;
		}
	}
	iris_set_esd_status(true);
	IRIS_LOGI("%s(%d), statis = 0x%x 0x%x 0x%x.", __func__, __LINE__,
			statis[0], statis[1], statis[2]);

	if ((statis[0] == internal_pt_checksum[0]) &&
		(statis[1] == internal_pt_checksum[1]) &&
		(statis[2] == internal_pt_checksum[2])) {
		ret = ERR_NO_ERR;
		IRIS_LOGI("%s(%d), internal pt loopback validate success.", __func__, __LINE__);
	} else {
		IRIS_LOGE("%s(%d), fail, statis not equal to checksum.", __func__, __LINE__);
		return 0x13;
	}

	IRIS_LOGD("%s(%d), end.", __func__, __LINE__);

	return ret;
}

int iris_loop_back_validate_i7p(void)
{
	int ret = 0;
#ifdef IRIS_EXT_CLK
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_clk_enable(pcfg->panel);
#endif
	if (iris_loop_back_flag_i7p & BIT_INTERNAL_PT) {
		IRIS_LOGI("%s(%d), step 1, internal pt loop back verify!", __func__, __LINE__);
		ret = iris_internal_pt_verify();
		if (ret) {
			IRIS_LOGE("%s(%d) step1: internal pt loop back verify ret = %d", __func__, __LINE__, ret);
			return ERR_INTERNAL_PT;
		}
	}

	iris_loop_back_reset();
	mdelay(10);
	iris_bulksram_power_domain_proc_i7p();
	//iris_disable_temp_sensor();
	iris_sleep_abyp_power_down();

	IRIS_LOGI("%s(%d), loop back test all passed!", __func__, __LINE__);
#ifdef IRIS_EXT_CLK
	iris_clk_disable(pcfg->panel);
#endif
	return 0;
}

int iris_mipi_rx0_validate_i7p(void)
{
	int ret = 0;

	//TODO

	return ret;
}
