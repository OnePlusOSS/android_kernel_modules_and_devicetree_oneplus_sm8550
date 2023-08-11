// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <linux/kobject.h>
#include "dsi_iris_api.h"
#include "dsi_iris_i3c.h"
#include "dsi_iris_loop_back.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_log.h"
#include "dsi_iris_lp.h"
#include "sde_connector.h"

static void iris_set_esd_status(bool enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_display *display = pcfg->display;

	if (!display)
		return;

	if (!pcfg->panel || !display->drm_conn)
		return;

	if (!enable) {
		if (display->panel->esd_config.esd_enabled) {
			sde_connector_schedule_status_work(display->drm_conn, false);
			display->panel->esd_config.esd_enabled = false;
			IRIS_LOGD("disable esd work");
		}
	} else {
		if (!display->panel->esd_config.esd_enabled) {
			sde_connector_schedule_status_work(display->drm_conn, true);
			display->panel->esd_config.esd_enabled = true;
			IRIS_LOGD("enabled esd work");
		}
	}
}

static uint32_t iris_loop_back_flag = (BIT_PURE_LOOPBACK | BIT_DUAL_PT);

/**
 * pure loop back
 */
#define PURE_LOOP_BACK_REG_NUM  (51)
static uint32_t pure_loop_back_op_addr[PURE_LOOP_BACK_REG_NUM] = {
0xf00000f8, 0xf0000068, 0xf0000068, 0xf0000068, 0xf0000068,
0xf1480068, 0xf1480074, 0xf1480078, 0xf148007c, 0xf1480080,
0xf14800b8, 0xf14800b4, 0xf1480120, 0xf1a8001c, 0xf1a80020,
0xf1a80024, 0xf1a80028, 0xf1a80060, 0xf1a8005c, 0xf1a9ff00,
0xf1ac0000, 0xf1ac0010, 0xf1ac001c, 0xf1ac0020, 0xf1ac0024,
0xf1ac0028, 0xf1ac0060, 0xf1ac005c, 0xf1adff00, 0xf1980000,
0xf1980008, 0xf1980048, 0xf198007c, 0xf1980050, 0xf1980054,
0xf1981010, 0xf1940000, 0xf1940018, 0xf194006c, 0xf1940008,
0xf19401c4, 0xf194106c, 0xf18c0004, 0xf18d1000, 0xf178001c,
0xf1780128, 0xf1780124, 0xf179ff00, 0xf1700130, 0xf1711000,
0xf1941068
};

static uint32_t pure_loop_back_op_val[PURE_LOOP_BACK_REG_NUM] = {
0x00000644, 0x00000000, 0x00000001, 0x00000003, 0x000000ff,
0x80078030, 0x0e025901, 0xd1002000, 0x0c000700, 0x5c066706,
0x346cf43b, 0xb42b342b, 0x00000002, 0x0e025901, 0xd1002000,
0x0c000700, 0x5c066706, 0x346cf43b, 0xb42b342b, 0x00000100,
0x01480150, 0x80078030, 0x0e025901, 0xd1002000, 0x0c000700,
0x5c066706, 0x346cf43b, 0xb42b342b, 0x00000100, 0x00000a04,
0x00001800, 0x0438021c, 0x00000003, 0x00040000, 0x00040000,
0x00000010, 0x0001400c, 0x0438021c, 0x00000aaa, 0x84400050,
0x00000003, 0x00000200, 0x00000008, 0x00000100, 0x30080800,
0x01040000, 0x10100102, 0x00000100, 0x0a0a0664, 0x00000006,
0x00000037
};

static uint32_t pure_loop_back_op_delay[PURE_LOOP_BACK_REG_NUM] = {
0, 1, 1, 1, 1,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 20,
0
};

static uint32_t pure_loop_back_checksum[3] = {0x58f75b40, 0x57ee3d0d, 0x576d2193};

static int iris_pure_loop_back_verify(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int ret = 0;
	int i = 0;
	uint32_t statis[3] = {0};

	IRIS_LOGD("%s(%d), start.", __func__, __LINE__);
	iris_set_esd_status(false);
	iris_reset();
	mdelay(100);
	mutex_lock(&pcfg->panel->panel_lock);
	iris_set_two_wire0_enable();
	mutex_unlock(&pcfg->panel->panel_lock);

	for (i = 0; i < PURE_LOOP_BACK_REG_NUM; i++) {
		ret = iris_ioctl_i2c_write(pure_loop_back_op_addr[i], pure_loop_back_op_val[i]);
		if (ret) {
			IRIS_LOGE("%s(%d), error, i = %d, ret = %d.", __func__, __LINE__, i, ret);
			return 0x11;
		}

		if (pure_loop_back_op_delay[i])
			mdelay(pure_loop_back_op_delay[i]);
	}

	mdelay(100);
	for (i = 0; i < 3; i++) {
		iris_ioctl_i2c_read(0xf1940200 + i*4, &statis[i]);
		if (ret) {
			IRIS_LOGE("%s(%d), i2c read statis fail.", __func__, __LINE__);
			return 0x12;
		}
	}
	iris_set_esd_status(true);
	IRIS_LOGI("%s(%d), statis = 0x%x 0x%x 0x%x.", __func__, __LINE__,
			statis[0], statis[1], statis[2]);


	if ((statis[0] == pure_loop_back_checksum[0]) &&
		(statis[1] == pure_loop_back_checksum[1]) &&
		(statis[2] == pure_loop_back_checksum[2])) {
		ret = ERR_NO_ERR;
		IRIS_LOGI("%s(%d), loopback validate success.", __func__, __LINE__);
	} else {
		IRIS_LOGE("%s(%d), statis not equal to checksum.", __func__, __LINE__);
		return 0x13;
	}

	IRIS_LOGD("%s(%d), end.", __func__, __LINE__);

	return ret;

}

/**
 * dual pt
 */
#define DUAL_PT_REG_NUM  (47)
static uint32_t dual_pt_op_addr[DUAL_PT_REG_NUM] = {
0xf0000068, 0xf0000068,
0xf0000000, 0xf00000f8, 0xf1480068, 0xf1480074, 0xf1480078,
0xf148007c, 0xf1480080, 0xf14800b8, 0xf14800b4, 0xf1a8001c,
0xf1a80020, 0xf1a80024, 0xf1a80028, 0xf1a80060, 0xf1a8005c,
0xf1ac001c, 0xf1ac0020, 0xf1ac0024, 0xf1ac0028, 0xf1ac0000,
0xf1ac0010, 0xf1ac0060, 0xf1ac005c, 0xf1980000, 0xf1980048,
0xf1980050, 0xf1980054, 0xf1980008, 0xf18c0004, 0xf18d1000,
0xf178001c, 0xf1780124, 0xf1780128, 0xf179ff00, 0xf1700130,
0xf1711000, 0xf198007c, 0xf1981010, 0xf1940000, 0xf1940018,
0xf194006c, 0xf1940008, 0xf19401c4, 0xf194106c, 0xf1941068
};

static uint32_t dual_pt_op_val[DUAL_PT_REG_NUM] = {
0x00000001, 0x000000ff,
0x00000000, 0x00000644, 0x80078030, 0x0e025901, 0xd1002000,
0x0c000700, 0x5c066706, 0x346cf43b, 0xb42b342b, 0x0e025901,
0xd1002000, 0x0c000700, 0x5c066706, 0x346cf43b, 0xb42b342b,
0x0e025901, 0xd1002000, 0x0c000700, 0x5c066706, 0x01480150,
0x80078030, 0x346cf43b, 0xb42b342b, 0x00000a04, 0x0438021c,
0x00040000, 0x00040000, 0x00001800, 0x00000008, 0x00000100,
0x30080800, 0x10100102, 0x01040000, 0x00000100, 0x0a0a0664,
0x00000002, 0x00000003, 0x00000010, 0x0001400c, 0x0438021c,
0x00000aaa, 0x84400050, 0x00000003, 0x00000200, 0x00000037
};

static uint32_t dual_pt_op_delay[DUAL_PT_REG_NUM] = {
0, 1,
0, 1, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 10
};

static uint32_t dual_pt_checksum[3] = {0x58f75b40, 0x57ee3d0d, 0x576d2193};

static int iris_dual_pt_verify(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int ret = 0;
	int i = 0;
	uint32_t statis[3] = {0};

	IRIS_LOGD("%s(%d), start.", __func__, __LINE__);
	iris_set_esd_status(false);
	iris_reset();
	mdelay(100);
	mutex_lock(&pcfg->panel->panel_lock);
	iris_set_two_wire0_enable();
	mutex_unlock(&pcfg->panel->panel_lock);

	for (i = 0; i < DUAL_PT_REG_NUM; i++) {
		ret = iris_ioctl_i2c_write(dual_pt_op_addr[i], dual_pt_op_val[i]);
		if (ret) {
			IRIS_LOGE("%s(%d), error, i = %d, ret = %d.", __func__, __LINE__, i, ret);
			return 0x21;
		}

		if (dual_pt_op_delay[i])
			mdelay(dual_pt_op_delay[i]);
	}

	mdelay(100);
	for (i = 0; i < 3; i++) {
		iris_ioctl_i2c_read(0xf1940200 + i*4, &statis[i]);
		if (ret) {
			IRIS_LOGE("%s(%d), i2c read statis fail.", __func__, __LINE__);
			return 0x22;
		}
	}
	iris_set_esd_status(true);
	IRIS_LOGI("%s(%d), statis = 0x%x 0x%x 0x%x.", __func__, __LINE__,
			statis[0], statis[1], statis[2]);


	if ((statis[0] == dual_pt_checksum[0]) &&
		(statis[1] == dual_pt_checksum[1]) &&
		(statis[2] == dual_pt_checksum[2])) {
		ret = ERR_NO_ERR;
		IRIS_LOGI("%s(%d), loopback validate success.", __func__, __LINE__);
	} else {
		IRIS_LOGE("%s(%d), statis not equal to checksum.", __func__, __LINE__);
		return 0x23;
	}

	IRIS_LOGD("%s(%d), end.", __func__, __LINE__);

	return ret;
}

/**
 * atspeed-efifo pt
 */
#define ATSPEED_EFIFO_REG_NUM  (62)
static uint32_t atspeed_efifo_op_addr[ATSPEED_EFIFO_REG_NUM] = {
0xf0000068, 0xf0000068,
0xf0000000, 0xf00000f8, 0xf0000048, 0xf0000000, 0xf0000020,
0xf0000004, 0xf0000008, 0xf0000008, 0xf0000024, 0xf0000028,
0xf0000028, 0xf0010040, 0xf1480068, 0xf1480074, 0xf1480078,
0xf148007c, 0xf1480080, 0xf14800b8, 0xf14800b4, 0xf1a8001c,
0xf1a80020, 0xf1a80024, 0xf1a80028, 0xf1a80060, 0xf1a8005c,
0xf1ac001c, 0xf1ac0020, 0xf1ac0024, 0xf1ac0028, 0xf1ac0000,
0xf1ac0010, 0xf1ac0060, 0xf1ac005c, 0xf1980000, 0xf1980048,
0xf1980050, 0xf1980054, 0xf1980008, 0xf18c0004, 0xf18d1000,
0xf178001c, 0xf1780124, 0xf1780128, 0xf1780024, 0xf1780020,
0xf1780004, 0xf1780014, 0xf1780120, 0xf179ff00, 0xf1700130,
0xf1711000, 0xf198007c, 0xf1981010, 0xf1940000, 0xf1940018,
0xf194006c, 0xf1940008, 0xf19401c4, 0xf194106c, 0xf1941068
};

static uint32_t atspeed_efifo_op_val[ATSPEED_EFIFO_REG_NUM] = {
0x00000001, 0x000000ff,
0x00000000, 0x00000644, 0x0000b7a0, 0x00000000, 0x00000000,
0x0a2a022a, 0x0000f80d, 0x0000f00d, 0x0a2a022b, 0x0000f80d,
0x0000f00d, 0x000000ff, 0x80078030, 0x0e025901, 0xd1002000,
0x0c000700, 0x5c066706, 0x346cf43b, 0xb42b342b, 0x0e025901,
0xd1002000, 0x0c000700, 0x5c066706, 0x346cf43b, 0xb42b342b,
0x0e025901, 0xd1002000, 0x0c000700, 0x5c066706, 0x01480150,
0x80078030, 0x346cf43b, 0xb42b342b, 0x00000a04, 0x0438021c,
0x00000000, 0x00080000, 0x00001800, 0x00000008, 0x00000100,
0x30081000, 0x10100102, 0x01080000, 0x000a0000, 0x020308cc,
0x143c2085, 0x0007841e, 0x00000000, 0x00000100, 0x0a0a0664,
0x00000002, 0x00000003, 0x00000010, 0x0001400c, 0x0438021c,
0x00000aaa, 0x84400052, 0x00000003, 0x00000200, 0x00000037
};

static uint32_t atspeed_efifo_op_delay[ATSPEED_EFIFO_REG_NUM] = {
0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 1, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
0, 0, 0, 0, 8
};

static uint32_t atspeed_efifo_checksum[3] = {0x58f75b40, 0x57ee3d0d, 0x576d2193};

static int iris_atspeed_efifo_verify(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int ret = 0;
	int i = 0;
	uint32_t statis[3] = {0};

	IRIS_LOGD("%s(%d), start.", __func__, __LINE__);
	iris_reset();
	mdelay(100);
	mutex_lock(&pcfg->panel->panel_lock);
	iris_set_two_wire0_enable();
	mutex_unlock(&pcfg->panel->panel_lock);

	for (i = 0; i < ATSPEED_EFIFO_REG_NUM; i++) {
		ret = iris_ioctl_i2c_write(atspeed_efifo_op_addr[i], atspeed_efifo_op_val[i]);
		if (ret) {
			IRIS_LOGE("%s(%d), error, i = %d, ret = %d.", __func__, __LINE__, i, ret);
			return 0x31;
		}

		if (atspeed_efifo_op_delay[i])
			mdelay(atspeed_efifo_op_delay[i]);
	}

	mdelay(100);
	for (i = 0; i < 3; i++) {
		iris_ioctl_i2c_read(0xf1940200 + i*4, &statis[i]);
		if (ret) {
			IRIS_LOGE("%s(%d), i2c read statis fail.", __func__, __LINE__);
			return 0x32;
		}
	}

	IRIS_LOGI("%s(%d), statis = 0x%x 0x%x 0x%x.", __func__, __LINE__,
			statis[0], statis[1], statis[2]);


	if ((statis[0] == atspeed_efifo_checksum[0]) &&
		(statis[1] == atspeed_efifo_checksum[1]) &&
		(statis[2] == atspeed_efifo_checksum[2])) {
		ret = ERR_NO_ERR;
		IRIS_LOGI("%s(%d), loopback validate success.", __func__, __LINE__);
	} else {
		IRIS_LOGE("%s(%d), statis not equal to checksum.", __func__, __LINE__);
		return 0x33;
	}

	IRIS_LOGD("%s(%d), end.", __func__, __LINE__);

	return ret;
}

/**
 * smt_pmu
 */
static uint32_t addr_val_s4[8][2] = {
	{0xf0000120, 0x00044633},
	{0xf0000124, 0x0000f1f9},
	{0xf0000128, 0x00000200},
	{0xf000012C, 0x00000100},
	{0xf0000130, 0x00000100},
	{0xf0000134, 0x00000100},
	{0xf0000138, 0x00000100},
	{0xf0000140, 0x00000100},
};
static uint32_t addr_val_s5[1][2] = {
	{0xf0000068, 0x00000001},
};
static uint32_t addr_val_s6[1][2] = {
	{0xf0000120, 0x00040233},
};
static uint32_t addr_val_s7[1][2] = {
	{0xf0000068, 0x00000003},
};
static uint32_t addr_val_s8[1][2] = {
	{0xf0000124, 0x0000f1f9},
};
static uint32_t addr_val_s9[1][2] = {
	{0xf0000068, 0x0000000b},
};
static uint32_t addr_val_s10[1][2] = {
	{0xf000012c, 0x0000c8f8},
};
static uint32_t addr_val_s11[1][2] = {
	{0xf0000068, 0x0000000f},
};
static uint32_t addr_val_s12[1][2] = {
	{0xf0000128, 0x0000f1f9},
};
static uint32_t addr_val_s13[1][2] = {
	{0xf0000068, 0x0000004f},
};
static uint32_t addr_val_s14[1][2] = {
	{0xf0000140, 0x0001c8f8},
};
static uint32_t addr_val_s15[1][2] = {
	{0xf0000068, 0x0000005f},
};
static uint32_t addr_val_s16[1][2] = {
	{0xf0000134, 0x0000c8f8},
};
static uint32_t addr_val_s17[1][2] = {
	{0xf0000068, 0x0000007f},
};
static uint32_t addr_val_s18[1][2] = {
	{0xf0000138, 0x0000c8f8},
};
static uint32_t addr_val_s19[1][2] = {
	{0xf0000068, 0x000000ff},
};
static uint32_t addr_val_s20[1][2] = {
	{0xf0000134, 0x0000c8f8},
};
static uint32_t addr_val_s21[1][2] = {
	{0xf0000068, 0x00000001},
};
static uint32_t addr_val_s22[8][2] = {
	{0xf0000120, 0x00040233},
	{0xf0000124, 0x00000200},
	{0xf0000128, 0x00000200},
	{0xf000012c, 0x00000100},
	{0xf0000130, 0x00000100},
	{0xf0000134, 0x00000100},
	{0xf0000138, 0x00000100},
	{0xf0000140, 0x00000100},
};
static uint32_t addr_val_s23[1][2] = {
	{0xf0000068, 0x000000ff},
};
static uint32_t addr_val_s24[8][2] = {
	{0xf0000120, 0x0007fc33},
	{0xf0000124, 0x0000f1f9},
	{0xf0000128, 0x0000f1f9},
	{0xf000012c, 0x0000c8f8},
	{0xf0000130, 0x0000c8f8},
	{0xf0000134, 0x0000c8f8},
	{0xf0000138, 0x0000c8f8},
	{0xf0000140, 0x0001c8f8},
};

static int _iris_reg_check(uint32_t (*p1)[2], uint32_t cnt1, uint32_t (*p2)[2], uint32_t cnt2)
{
	int i, ret = 0;
	uint32_t val;

	if (!p2) {
		IRIS_LOGE("%s(), p2 is null", __func__);
		ret = -EINVAL;
		goto end;
	}

	IRIS_LOGD("%s, cnt1 = %d, cnt2 = %d", __func__, cnt1, cnt2);

	for (i = 0; i < cnt1; i++) {
		ret = iris_ioctl_i2c_write(p1[i][0], p1[i][1]);
		if (ret) {
			IRIS_LOGE("%s(%d), error, i = %d, ret = %d.", __func__, __LINE__, i, ret);
			goto end;
		}
	}

	for (i = 0; i < cnt2; i++) {
		iris_ioctl_i2c_read(p2[i][0], &val);
		if (val != p2[i][1]) {
			ret = i + 1;
			goto end;
		}
	}

end:
	IRIS_LOGD("%s, ret = %d", __func__, ret);
	return ret;
}

static int iris_smt_pmu_verify(void)
{
	int ret = 0;

	IRIS_LOGD("%s(%d), start.", __func__, __LINE__);
	iris_reset();
	mdelay(1);

	IRIS_LOGD("%s(%d), s4 check.", __func__, __LINE__);
	ret = _iris_reg_check(NULL, 0, addr_val_s4, sizeof(addr_val_s4)/sizeof(addr_val_s4[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s4 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), s5-6 check.", __func__, __LINE__);
	ret = _iris_reg_check(addr_val_s5, sizeof(addr_val_s5)/sizeof(addr_val_s5[0]),
		addr_val_s6, sizeof(addr_val_s6)/sizeof(addr_val_s6[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s5-6 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), s7-8 check.", __func__, __LINE__);
	ret = _iris_reg_check(addr_val_s7, sizeof(addr_val_s7)/sizeof(addr_val_s7[0]),
		addr_val_s8, sizeof(addr_val_s8)/sizeof(addr_val_s8[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s7-8 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), s9-10 check.", __func__, __LINE__);
	ret = _iris_reg_check(addr_val_s9, sizeof(addr_val_s9)/sizeof(addr_val_s9[0]),
		addr_val_s10, sizeof(addr_val_s10)/sizeof(addr_val_s10[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s9-10 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), s11-12 check.", __func__, __LINE__);
	ret = _iris_reg_check(addr_val_s11, sizeof(addr_val_s11)/sizeof(addr_val_s11[0]),
		addr_val_s12, sizeof(addr_val_s12)/sizeof(addr_val_s12[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s11-12 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), s13-14 check.", __func__, __LINE__);
	ret = _iris_reg_check(addr_val_s13, sizeof(addr_val_s13)/sizeof(addr_val_s13[0]),
		addr_val_s14, sizeof(addr_val_s14)/sizeof(addr_val_s14[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s13-14 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), s15-16 check.", __func__, __LINE__);
	ret = _iris_reg_check(addr_val_s15, sizeof(addr_val_s15)/sizeof(addr_val_s15[0]),
		addr_val_s16, sizeof(addr_val_s6)/sizeof(addr_val_s16[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s15-16 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), s17-18 check.", __func__, __LINE__);
	ret = _iris_reg_check(addr_val_s17, sizeof(addr_val_s17)/sizeof(addr_val_s17[0]),
		addr_val_s18, sizeof(addr_val_s18)/sizeof(addr_val_s18[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s17-18 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), s19-20 check.", __func__, __LINE__);
	ret = _iris_reg_check(addr_val_s19, sizeof(addr_val_s19)/sizeof(addr_val_s19[0]),
		addr_val_s20, sizeof(addr_val_s20)/sizeof(addr_val_s20[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s19-20 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), s21-22 check.", __func__, __LINE__);
	ret = _iris_reg_check(addr_val_s21, sizeof(addr_val_s21)/sizeof(addr_val_s21[0]),
		addr_val_s22, sizeof(addr_val_s22)/sizeof(addr_val_s22[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s21-22 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), s23-24 check.", __func__, __LINE__);
	ret = _iris_reg_check(addr_val_s23, sizeof(addr_val_s23)/sizeof(addr_val_s23[0]),
		addr_val_s24, sizeof(addr_val_s24)/sizeof(addr_val_s24[0]));
	if (ret) {
		IRIS_LOGE("%s(%d) s23-24 check failed, ret = %d", __func__, __LINE__, ret);
		return ERR_SMT_PMU;
	}

	IRIS_LOGD("%s(%d), end.", __func__, __LINE__);
	return 0;
}

int iris_loop_back_validate(void)
{
	int ret = 0;

#ifdef IRIS_EXT_CLK
	struct iris_cfg *pcfg = iris_get_cfg();

	iris_clk_enable(pcfg->panel);
#endif

	if (iris_loop_back_flag & BIT_PURE_LOOPBACK) {
		IRIS_LOGI("%s(%d), step 1, pure loop back verify!", __func__, __LINE__);
		ret = iris_pure_loop_back_verify();
		if (ret) {
			IRIS_LOGE("%s(%d) step1: pure loop back verify ret = %d", __func__, __LINE__, ret);
			return ERR_PURE_LOOP_BACK;
		}
	}

	if (iris_loop_back_flag & BIT_DUAL_PT) {
		IRIS_LOGI("%s(%d), step 2, dual pt verify!", __func__, __LINE__);
		ret = iris_dual_pt_verify();
		if (ret) {
			IRIS_LOGE("%s(%d) step2: dual pt verify ret = %d", __func__, __LINE__, ret);
			return ERR_DUAL_PT;
		}
	}

	if (iris_loop_back_flag & BIT_ATSPEED_EFIFO) {
		IRIS_LOGI("%s(%d), step 3, at speed efifo verify!", __func__, __LINE__);
		ret = iris_atspeed_efifo_verify();
		if (ret) {
			IRIS_LOGE("%s(%d) step3: at speed efifo verify ret = %d", __func__, __LINE__, ret);
			return ERR_ATSPEED_EFIFO;
		}
	}

	if (iris_loop_back_flag & BIT_SMT_PMU) {
		IRIS_LOGI("%s(%d), step 4, smt pmu verify!", __func__, __LINE__);
		ret = iris_smt_pmu_verify();
		if (ret) {
			IRIS_LOGE("%s(%d) step4: smt pmu verify ret = %d", __func__, __LINE__, ret);
			return ERR_SMT_PMU;
		}
	}

	iris_reset();
	mdelay(10);
	_iris_disable_temp_sensor();
	iris_sleep_abyp_power_down();

#ifdef IRIS_EXT_CLK
	iris_clk_disable(pcfg->panel);
#endif
	return 0;
}

static ssize_t _iris_dbg_loop_back_read(struct file *file,
		char __user *buff, size_t count, loff_t *ppos)
{
	int ret = -1;
	int tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	ret = iris_loop_back_validate();
	IRIS_LOGI("%s(%d) ret: %d", __func__, __LINE__, ret);

	tot = scnprintf(bp, sizeof(bp), "0x%x\n", ret);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_loop_back_fops = {
	.open = simple_open,
	.read = _iris_dbg_loop_back_read,
};


int iris_mipi_rx0_validate(void)
{
	int ret = 0;
	uint32_t reg_val = 0;

	iris_ocp_write_val(0xf1a00008, 0xffff3a9a);
	mdelay(100);
	reg_val = iris_ocp_read(0xf1a00004, DSI_CMD_SET_STATE_HS);
	if (reg_val & 0x0ffff3cf)
		ret = 1;
	else
		ret = 0;

	return ret;
}

static ssize_t _iris_mipi_rx_validate(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	int tot = 0;
	char bp[512];
	struct iris_cfg *pcfg = iris_get_cfg();

	if (*ppos)
		return 0;

	mutex_lock(&pcfg->panel->panel_lock);
	ret = iris_mipi_rx0_validate();
	mutex_unlock(&pcfg->panel->panel_lock);

	tot = scnprintf(bp, sizeof(bp), "0x%02x\n", ret);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;

}

static const struct file_operations iris_mipi_rx_validate_fops = {
	.open = simple_open,
	.write = NULL,
	.read = _iris_mipi_rx_validate,
};

static ssize_t iris_loop_back_show(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = iris_loop_back_validate();
	IRIS_LOGI("%s(%d) ret: %d", __func__, __LINE__, ret);

	return snprintf(buf, PAGE_SIZE, "0x%x\n", ret);
}

#define IRIS_ATTR(_name, _mode, _show, _store) \
	struct kobj_attribute iris_attr_##_name = __ATTR(_name, _mode, _show, _store)
static IRIS_ATTR(iris_loop_back, S_IRUGO, iris_loop_back_show, NULL);

static struct attribute *iris_dev_attrs[] = {
	&iris_attr_iris_loop_back.attr,
	NULL
};

static const struct attribute_group iris_attr_group = {
	.attrs = iris_dev_attrs,
};

int iris_dbgfs_loop_back_init(struct dsi_display *display)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int retval;

	if (pcfg->iris_kobj == NULL) {
		pcfg->iris_kobj = kobject_create_and_add(IRIS_SYSFS_TOP_DIR, kernel_kobj);
	}
	if (pcfg->iris_kobj) {
		/* Create the files associated with this kobject */
		retval = sysfs_create_group(pcfg->iris_kobj, &iris_attr_group);
		if (retval) {
			kobject_put(pcfg->iris_kobj);
			IRIS_LOGW("sysfs create group iris_loop_back_show error");
		}
	} else {
		IRIS_LOGW("sysfs create iris dir error");
	}

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("iris_loop_back_flag", 0644, pcfg->dbg_root,
			(u32 *)&iris_loop_back_flag);

	if (debugfs_create_file("iris_loop_back",	0644, pcfg->dbg_root, display,
				&iris_loop_back_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("iris_mipi_rx_validate",	0644, pcfg->dbg_root, display,
				&iris_mipi_rx_validate_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}
