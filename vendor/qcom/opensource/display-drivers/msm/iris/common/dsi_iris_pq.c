// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <video/mipi_display.h>
#include <sde_encoder_phys.h>
#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_lp.h"
#include "dsi_iris_pq.h"
#include "dsi_iris_lut.h"
#include "dsi_iris_ioctl.h"
#include "dsi_iris_timing_switch.h"
#include "dsi_iris_log.h"
#include "dsi_iris_memc.h"
#include "dsi_iris_dts_fw.h"
#include "dsi_iris.h"

struct iris_setting_info iris_setting;
bool iris_skip_dma;
bool m_dpp_precsc_enable;
u32 iris_min_color_temp;
u32 iris_max_color_temp;
u32 iris_min_x_value;
u32 iris_max_x_value;
static u32 last_filter_level;

u16 *iris_crstk_coef_buf;

#define IRIS_X_6500K			3128
#define IRIS_X_7500K			2991
#define IRIS_X_7700K			2969
#define IRIS_X_2500K			4637
#define IRIS_LAST_BIT_CTRL	1

u32 dwCSC2CoffBuffer[12] = {
	0x0000, 0x0800, 0x0000,
	0x0000, 0x0000, 0x0800,
	0x0800, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000,
};

static u32 dwCSCCoffBuffer[8] = {
	0x08000000, 0x00000000, 0x08000000, 0x00000800,
	0x00000000, 0x00000000, 0x00000000, 0x00000000
};

static u32 dwCSC2CoffDefault[12] = {
	0x0000, 0x0800, 0x0000,
	0x0000, 0x0000, 0x0800,
	0x0800, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000,
};

/*range is 2500~11000*/
static u32 iris_color_x_buf[] = {
	4637, 4626, 4615, 4603,
	4591, 4578, 4565, 4552,
	4538, 4524, 4510, 4496,
	4481, 4467, 4452, 4437,
	4422, 4407, 4392, 4377,
	4362, 4347, 4332, 4317,
	4302, 4287, 4272, 4257,
	4243, 4228, 4213, 4199,
	4184, 4170, 4156, 4141,
	4127, 4113, 4099, 4086,
	4072, 4058, 4045, 4032,
	4018, 4005, 3992, 3980,
	3967, 3954, 3942, 3929,
	3917, 3905, 3893, 3881,
	3869, 3858, 3846, 3835,
	3823, 3812, 3801, 3790,
	3779, 3769, 3758, 3748,
	3737, 3727, 3717, 3707,
	3697, 3687, 3677, 3668,
	3658, 3649, 3639, 3630,
	3621, 3612, 3603, 3594,
	3585, 3577, 3568, 3560,
	3551, 3543, 3535, 3527,
	3519, 3511, 3503, 3495,
	3487, 3480, 3472, 3465,
	3457, 3450, 3443, 3436,
	3429, 3422, 3415, 3408,
	3401, 3394, 3388, 3381,
	3375, 3368, 3362, 3356,
	3349, 3343, 3337, 3331,
	3325, 3319, 3313, 3307,
	3302, 3296, 3290, 3285,
	3279, 3274, 3268, 3263,
	3258, 3252, 3247, 3242,
	3237, 3232, 3227, 3222,
	3217, 3212, 3207, 3202,
	3198, 3193, 3188, 3184,
	3179, 3175, 3170, 3166,
	3161, 3157, 3153, 3149,
	3144, 3140, 3136, 3132,
	3128, 3124, 3120, 3116,
	3112, 3108, 3104, 3100,
	3097, 3093, 3089, 3085,
	3082, 3078, 3074, 3071,
	3067, 3064, 3060, 3057,
	3054, 3050, 3047, 3043,
	3040, 3037, 3034, 3030,
	3027, 3024, 3021, 3018,
	3015, 3012, 3009, 3006,
	3003, 3000, 2997, 2994,
	2991, 2988, 2985, 2982,
	2980, 2977, 2974, 2971,
	2969, 2966, 2963, 2961,
	2958, 2955, 2953, 2950,
	2948, 2945, 2943, 2940,
	2938, 2935, 2933, 2930,
	2928, 2926, 2923, 2921,
	2919, 2916, 2914, 2912,
	2910, 2907, 2905, 2903,
	2901, 2899, 2896, 2894,
	2892, 2890, 2888, 2886,
	2884, 2882, 2880, 2878,
	2876, 2874, 2872, 2870,
	2868, 2866, 2864, 2862,
	2860, 2858, 2856, 2854,
	2853, 2851, 2849, 2847,
	2845, 2844, 2842, 2840,
	2838, 2837, 2835, 2833,
	2831, 2830, 2828, 2826,
	2825, 2823, 2821, 2820,
	2818, 2817, 2815, 2813,
	2812, 2810, 2809, 2807,
	2806, 2804, 2803, 2801,
	2800, 2798, 2797, 2795,
	2794, 2792, 2791, 2789,
	2788, 2787, 2785, 2784,
	2782, 2781, 2780, 2778,
	2777, 2776, 2774, 2773,
	2772, 2770, 2769, 2768,
	2766, 2765, 2764, 2763,
	2761, 2760, 2759, 2758,
	2756, 2755, 2754, 2753,
	2751, 2750, 2749, 2748,
	2747, 2745, 2744, 2743,
	2742, 2741, 2740, 2739,
	2737,
};

struct iris_setting_info *iris_get_setting(void)
{
	return &iris_setting;
}

int iris_get_hdr_enable(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		return iris_get_hdr_enable_i7();
	else
		return 0;
}

bool iris_dspp_dirty(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (pqlt_cur_setting->dspp_dirty > 0) {
		IRIS_LOGI("DSPP is dirty");
		pqlt_cur_setting->dspp_dirty--;
		return true;
	}

	return false;
}

void iris_quality_setting_off(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_quality_setting_off_i7();
	else
		iris_quality_setting_off_i7p();

	last_filter_level = 0;
	iris_skip_dma = false;
	m_dpp_precsc_enable = false;
}

void iris_set_skip_dma(bool skip)
{
	IRIS_LOGD("skip_dma=%d", skip);
	iris_skip_dma = skip;
}

static void iris_end_dpp(bool bcommit)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_end_dpp_i7(bcommit);
	else
		iris_end_dpp_i7p(bcommit);
}

void iris_init_ipopt_t(void)
{
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	for (i = 0; i < pcfg->ip_opt_cnt; i++)
		pcfg->pq_update_cmd.update_ipopt_array[i].ip = 0xff;

	pcfg->pq_update_cmd.array_index = 0;
}

u32 iris_color_temp_x_get(u32 index)
{
	return iris_color_x_buf[index];
}

void iris_ioinc_filter_ratio_send(void)
{
	//u32 dwRatioDiff = 0;
	u32 i = 0;
	u32 len = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	bool video_mode = false;
	u32 level = 0x10;

	struct iris_ctrl_opt arr_input[] = {
		{IOINC1D_LUT, 0x00, 0x01},
		{IOINC1D_LUT, 0x40, 0x01},
		{IOINC1D_LUT, 0x80, 0x01},
		{IOINC1D_LUT, 0xc0, 0x01},
		{IOINC1D_LUT_SHARP, 0x00, 0x01},
		{IOINC1D_LUT_SHARP, 0x40, 0x01},
		{IOINC1D_LUT_SHARP, 0x80, 0x01},
		{IOINC1D_LUT_SHARP, 0xc0, 0x00},
	};

	struct iris_ctrl_opt arr_pp[] = {
		{IOINC1D_PP_LUT, 0x00, 0x01},
		{IOINC1D_PP_LUT, 0x40, 0x01},
		{IOINC1D_PP_LUT, 0x80, 0x01},
		{IOINC1D_PP_LUT, 0xc0, 0x01},
		{IOINC1D_PP_LUT_SHARP, 0x00, 0x01},
		{IOINC1D_PP_LUT_SHARP, 0x40, 0x01},
		{IOINC1D_PP_LUT_SHARP, 0x80, 0x01},
		{IOINC1D_PP_LUT_SHARP, 0xc0, 0x00},
	};

	if (pcfg->iris_chip_type == CHIP_IRIS7P)
		return;

	/*
	dwRatioDiff = pcfg->frc_setting.disp_hres * 10 / pcfg->frc_setting.memc_hres;

	if ((dwRatioDiff < 20) && (dwRatioDiff >= 18))
		dwScaleDownRatio = eScalerDown1_8;
	else if ((dwRatioDiff < 22) && (dwRatioDiff >= 20))
		dwScaleDownRatio = eScalerDown2_0;
	else if ((dwRatioDiff < 24) && (dwRatioDiff >= 22))
		dwScaleDownRatio = eScalerDown2_2;
	else if ((dwRatioDiff < 26) && (dwRatioDiff >= 24))
		dwScaleDownRatio = eScalerDown2_4;
	else if ((dwRatioDiff < 28) && (dwRatioDiff >= 26))
		dwScaleDownRatio = eScalerDown2_6;
	else if (dwRatioDiff >= 28)
		dwScaleDownRatio = eScalerDown2_8;
	*/

	if (pcfg->memc_info.memc_mode == MEMC_SINGLE_VIDEO_ENABLE ||
		pcfg->memc_info.memc_mode == MEMC_DUAL_VIDEO_ENABLE)
		video_mode = true;
	if (video_mode == true)
		level = 0x28;
	if (last_filter_level != 0 && level == last_filter_level) {
		IRIS_LOGI("%s, ioinc ratio=%d no need to update 1d filter.", __func__, level);
		return;
	}
	len = sizeof(arr_input)/sizeof(struct iris_ctrl_opt);
	for (i = 0; i < len; i++) {
		arr_input[i].opt_id += level;
		IRIS_LOGD("%s[%d], %02x %02x %02x", __func__, __LINE__,
			arr_input[i].ip, arr_input[i].opt_id, arr_input[i].chain);
	}

	if (video_mode) {
		iris_send_assembled_pkt(arr_input, len);
	} else {
		iris_send_assembled_pkt(arr_input, len);
	}
	last_filter_level = level;
	IRIS_LOGI("%s, ioinc ratio=%d last_filter_level=%d", __func__, level, last_filter_level);

	level = 0xb;
	len = sizeof(arr_pp)/sizeof(struct iris_ctrl_opt);
	for (i = 0; i < len; i++) {
		arr_pp[i].opt_id += level;
		IRIS_LOGD("%s[%d], %02x %02x %02x", __func__, __LINE__,
			arr_pp[i].ip, arr_pp[i].opt_id, arr_pp[i].chain);
	}
	if (video_mode) {
		iris_send_assembled_pkt(arr_pp, len);
	} else {
		iris_send_assembled_pkt(arr_pp, len);
	}
	IRIS_LOGI("%s, ioinc ratio=%d", __func__, level);

}

bool iris_crst_coef_check(const u8 *fw_data, size_t fw_size, u32 crstk_coef_group)
{
	u32 len = 0;
	bool ret = false;

	if (fw_size < ((CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * crstk_coef_group)) {
		IRIS_LOGE("fw_size should be = %d bytes", (CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * crstk_coef_group);
		return false;
	}
	else
		len = fw_size;

	kfree(iris_crstk_coef_buf);
	iris_crstk_coef_buf = NULL;
	IRIS_LOGI("kfree iris_crstk_coef_buf");

	if (iris_crstk_coef_buf == NULL) {
		iris_crstk_coef_buf = kzalloc(len, GFP_KERNEL);
		IRIS_LOGI("kzalloc iris_crstk_coef_buf addr: 0x%x", &iris_crstk_coef_buf[0]);
	}
	if (!iris_crstk_coef_buf) {
		IRIS_LOGE("%s:failed to alloc mem iris_crstk_coef_buf:%p",
			__func__, iris_crstk_coef_buf);
		return false;
	}
	IRIS_LOGI("crs_fw size: %d", len);
	memcpy(&iris_crstk_coef_buf[0], (fw_data), len);

	IRIS_LOGI("csc2 or precsc: 0x%x", iris_crstk_coef_buf[len/2 - 1]);

	if (iris_crstk_coef_buf[len/2 - 1] == 0xaaaa || iris_crstk_coef_buf[len/2 - 1] == 0x5555) {
		IRIS_LOGI("csc2 or precsc data check OK!");
		ret = true;
	} else {
		IRIS_LOGI("csc2 or precsc data check error!");
		ret = false;
	}

	return ret;
}


void iris_pq_parameter_init(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_pq_parameter_init_i7();
	else
		iris_pq_parameter_init_i7p();

}

static void iris_cm_ratio_set(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_cm_ratio_set_i7();
	else
		iris_cm_ratio_set_i7p();
}

u32 iris_cm_ratio_set_for_iic(void)
{
	u32 tablesel;
	u32 index;
	u32 xvalue;
	u32 ratio;
	u32 value;
	u32 regvalue = 0;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	value = pqlt_cur_setting->colortempvalue;

	if (value > iris_max_color_temp)
		value = iris_max_color_temp;
	else if (value < iris_min_color_temp)
		value = iris_min_color_temp;
	index = (value - IRIS_CCT_MIN_VALUE) / 25;
	xvalue = iris_color_temp_x_get(index);

	if (xvalue == iris_min_x_value) {
		tablesel = 0;
		regvalue = tablesel | 0x02;
	} else if ((xvalue < iris_min_x_value) && (xvalue >= IRIS_X_7700K)) {
		tablesel = 0;
		ratio = ((xvalue - IRIS_X_7700K) * 16383) / (iris_min_x_value - IRIS_X_7700K);
		regvalue = tablesel | (ratio << 16);
	} else if ((xvalue >= iris_max_x_value) && (xvalue < IRIS_X_7700K)) {
		tablesel = 1;
		ratio = ((xvalue - iris_max_x_value) * 16383) / (IRIS_X_7700K - iris_max_x_value);
		regvalue = tablesel | (ratio << 16);
	}

	IRIS_LOGI("cm color temperature value=%d", value);

	return regvalue;
}

void iris_cm_colortemp_mode_set(u32 mode, bool bcommit)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (mode == 0) {
		iris_dpp_precsc_enable(0, false);
		iris_cm_color_gamut_set(0, false);
	} else {
		iris_dpp_precsc_enable(m_dpp_precsc_enable, false);
		iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut, false);
	}

	if (bcommit)
		iris_end_dpp(true);
	IRIS_LOGD("cm color temperature mode=%d", mode);
}

void iris_cm_color_temp_set(void)
{
	iris_cm_ratio_set();
	iris_end_dpp(true);

	IRIS_LOGD("%s",  __func__);
}

void iris_dpp_apl_enable(bool enable, uint8_t chain)
{
	uint32_t  *payload = NULL;
	//struct iris_update_ipopt popt[IP_OPT_MAX];
	uint32_t apl_ctrl = 0;
	uint32_t csc_coef_ctrl = 0;
	//enable/disable apl_ctrl
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x21, 2);
	if (!payload)
		return;
	apl_ctrl = payload[0] & 0xffffffcc;
	if (enable)
		apl_ctrl = apl_ctrl | 0x3 << 4 | 0x3;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x21, 2, apl_ctrl);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x21, 0x21, chain);

	//csc_coef_ctrl, CSC2_COEF_UPDATE_EN = 1 to enable
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x31, 2);
	if (!payload)
		return;
	csc_coef_ctrl = payload[0] & 0xfffffffe;
	//if (enable)
	//	csc_coef_ctrl = csc_coef_ctrl | 0x1;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x31, 2, csc_coef_ctrl);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x31, 0x31, chain);

}

void iris_dpp_3dlut_gain(u32 count, u32 *values, bool bcommit)
{
	uint32_t  *payload = NULL;
	int len;
	uint32_t lut3d_interp3;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2);
	if (!payload)
		return;
	if (count == 1) {
		lut3d_interp3 = payload[0] & 0xffff0000;
		lut3d_interp3 = lut3d_interp3 | values[0];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2, lut3d_interp3);
	} else if (count == 2) {
		lut3d_interp3 = (values[1] << 16) | values[0];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2, lut3d_interp3);
	} else if (count == 3) {
		lut3d_interp3 = (values[1] << 16) | values[0];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2, lut3d_interp3);
		lut3d_interp3 = payload[0] & 0xffff0000;
		lut3d_interp3 = lut3d_interp3 | values[2];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 3, lut3d_interp3);
	} else if (count == 4) {
		lut3d_interp3 = (values[1] << 16) | values[0];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2, lut3d_interp3);
		lut3d_interp3 = (values[3] << 16) | values[2];
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 3, lut3d_interp3);
	}
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x52, 0x52, 0x01);
	if (bcommit)
		iris_end_dpp(true);
	IRIS_LOGD("3dlut interpolation, gain=0x%x, len=%d", values[0], len);
}

void iris_dpp_3dlut_send(u8 lut_optid, uint8_t chain)
{
	u8 lut_type = DPP_3DLUT;

	iris_init_update_ipopt_t(lut_type, lut_optid, lut_optid, chain);

	if (chain == 0)
		iris_end_dpp(true);

	IRIS_LOGD("3dlut send optid: 0x%x", lut_optid);
}

void iris_cm_color_gamut_set(u32 level, bool bcommit)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_cm_color_gamut_set_i7(level, bcommit);
	else
		iris_cm_color_gamut_set_i7p(level, bcommit);
}

void iris_dpp_gamma_set(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	u32 gammalevel;

	if (pqlt_cur_setting->pq_setting.cmcolortempmode
			== IRIS_COLOR_TEMP_OFF)
		gammalevel = 0; /*use liner gamma if cm lut disable*/
	else
		gammalevel = pqlt_cur_setting->pq_setting.cmcolorgamut + 1;

	iris_update_ip_opt(GAMMA_LUT,
			gammalevel, 0x01);

	iris_end_dpp(true);
}

void iris_dpp_precsc_enable(u32 enable, bool bcommit)
{
	uint32_t  *payload = NULL;
	uint32_t csc_ctrl = 0;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x30, 2);
	if (!payload)
		return;
	csc_ctrl = payload[0] & 0xdeff;
	if (enable)
		csc_ctrl = csc_ctrl | 0x1 << 8;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x30, 2, csc_ctrl);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x30, 0x30, 0x01);
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x30, 2);
	if (!payload)
		return;

	if (bcommit)
		iris_end_dpp(true);

	IRIS_LOGD("dpp precsc enable =%d", enable);
}

void iris_dpp_precsc_set(bool enable)
{
	m_dpp_precsc_enable = enable;
}

void iris_dpp_prelut_rgb_set(u32 lut_optid, bool bcommit)
{
	u8 lut_type = DPP_PRE_LUT;

	iris_init_update_ipopt_t(lut_type, lut_optid, lut_optid, 0x1);

	iris_end_dpp(bcommit);

	IRIS_LOGI("dpp pre lut rgb switch =%d", lut_optid);
}

void iris_dpp_gammamode_set(u32 gammamode, u32 gammaIndex)
{
	uint32_t  *payload = NULL;
	uint32_t currentmode = 0;
	uint32_t gammactrl = 0;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2);
	if (!payload)
		return;
	currentmode = payload[0] & 0x7;

	if ((gammamode == 0) && (currentmode != gammamode)) {
		//regval.value = 0;
		gammactrl = payload[0] & 0xff8;
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);

		iris_update_ip_opt(GAMMA_LUT, 0x00, 0x01);
	} else if (gammamode == 1) {
		gammactrl = ((payload[0]&0xfc0) | gammamode | (gammaIndex << 4));
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);

		if (currentmode != gammamode) {
			iris_update_ip_opt(GAMMA_LUT, 0x10, 0x01);
		}
	} else if (gammamode == 2) {
		gammactrl = ((payload[0]&0xfc0) | gammamode | (gammaIndex << 4));
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
		iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);

		if (currentmode != gammamode) {
			iris_update_ip_opt(GAMMA_LUT, 0x20, 0x01);
		}
	}

	iris_end_dpp(true);
}

void iris_reading_mode_set(u32 level)
{
	IRIS_LOGI("reading mode=%d", level);
}

void iris_lux_set(u32 level, bool update)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_lux_set_i7(level, update);
	else
		iris_lux_set_i7p(level, update);
}

void iris_al_enable(bool enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_al_enable_i7(enable);
	else
		iris_al_enable_i7p(enable);
}

void iris_pwil_dport_disable(bool enable, u32 value)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_pwil_dport_disable_i7(enable, value);
	else
		iris_pwil_dport_disable_i7p(enable, value);
}

void iris_scaler_gamma_enable(u32 level)
{
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x00, 2);
	if (!payload)
		return;

	payload[0] &= ~0x00000040;
	payload[0] |= (level << 6);

	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x00, 0x00, 0x01);

	iris_end_dpp(true);
	IRIS_LOGI("gamma enable=%d", level);
}

int iris_kickoff(void *phys_enc)
{
	struct sde_encoder_phys *phys_encoder = phys_enc;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct iris_cfg *pcfg;

	if (phys_encoder == NULL)
		return -EFAULT;
	if (phys_encoder->connector == NULL)
		return -EFAULT;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return -EFAULT;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	display = c_conn->display;
	if (display == NULL)
		return -EFAULT;

	pcfg = iris_get_cfg();
	if (pcfg->fod_pending)
		iris_post_fod(pcfg->panel);
	if (pcfg->memc_info.vfr_en && pcfg->iris_chip_type == CHIP_IRIS7)
		iris_memc_vfr_video_update_monitor(pcfg, display);
	if (iris_virtual_display(display) || pcfg->valid < PARAM_PARSED)
		return 0;

	complete(&pcfg->frame_ready_completion);
	return 0;
}

int32_t  iris_update_ip_opt(
		uint8_t ip, uint8_t opt_id, uint8_t chain)
{
	int i = 0;
	uint8_t old_opt;
	int32_t cnt = 0;

	struct iris_pq_ipopt_val *pq_ipopt_val = iris_get_cur_ipopt_val(ip);

	if (pq_ipopt_val == NULL) {
		IRIS_LOGI("can not get pq ipot val i_p = %02x, opt_id = %02x",
				ip, opt_id);
		return 1;
	}

	if (ip == GAMMA_LUT) {
		old_opt  = pq_ipopt_val->popt[i];
		pq_ipopt_val->popt[i] = opt_id;
		cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
		return cnt;
	}

	for (i = 0; i < pq_ipopt_val->opt_cnt; i++) {
		if ((opt_id & 0xf0) == ((pq_ipopt_val->popt[i]) & 0xf0)) {
			old_opt  = pq_ipopt_val->popt[i];
			pq_ipopt_val->popt[i] = opt_id;
			cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
			return cnt;
		}
	}
	return 1;
}

void iris_rx_meta_dma_list_send(u32 meta, bool commit)
{
	uint32_t *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_RX, 0x02, 3);
	if (!payload)
		return;

	payload[0] = meta;
	iris_init_update_ipopt_t(IRIS_IP_RX, 0x02, 0x02, 0);
	iris_update_pq_opt(iris_pq_update_path, commit);
	IRIS_LOGI("%s(), set pwil mode according to dma mode: %x", __func__, payload[0]);

}

static ssize_t iris_pq_config_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg_log {
		uint8_t type;
		char *str;
	};

	struct iris_cfg_log arr[] = {
		{IRIS_DEMO_MODE, "IRIS_DEMO_MODE"},
		{IRIS_SDR2HDR, "IRIS_SDR2HDR"},
		{IRIS_DYNAMIC_POWER_CTRL, "IRIS_DYNAMIC_POWER_CTRL"},
		{IRIS_HDR_MAXCLL, "IRIS_HDR_MAXCLL"},
		{IRIS_ANALOG_BYPASS_MODE, "IRIS_ANALOG_BYPASS_MODE"},
		{IRIS_CM_COLOR_TEMP_MODE, "IRIS_CM_COLOR_TEMP_MODE"},
		{IRIS_CM_COLOR_GAMUT, "IRIS_CM_COLOR_GAMUT"},
		{IRIS_CCT_VALUE, "IRIS_CCT_VALUE"},
		{IRIS_COLOR_TEMP_VALUE, "IRIS_COLOR_TEMP_VALUE"},
		{IRIS_AL_ENABLE, "IRIS_AL_ENABLE"},
		{IRIS_LUX_VALUE, "IRIS_LUX_VALUE"},
		{IRIS_READING_MODE, "IRIS_READING_MODE"},
		{IRIS_CHIP_VERSION, "IRIS_CHIP_VERSION"},
		{IRIS_PANEL_TYPE, "IRIS_PANEL_TYPE"},
	};
	u32 type;
	u32 value;
	int i = 0;
	uint32_t cfg_val = 0;
	int len = (sizeof(arr))/(sizeof(arr[0]));
	struct iris_cfg *pcfg = iris_get_cfg();

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	type = (val & 0xffff0000) >> 16;
	value = (val & 0xffff);

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_configure_i7(DSI_PRIMARY, type, value);
	else
		iris_configure_i7p(DSI_PRIMARY, type, value);

	for (i = 0; i < len; i++) {
		if (pcfg->iris_chip_type == CHIP_IRIS7)
			iris_configure_get_i7(DSI_PRIMARY, arr[i].type, 1, &cfg_val);
		else
			iris_configure_get_i7p(DSI_PRIMARY, arr[i].type, 1, &cfg_val);
		IRIS_LOGI("%s: %d", arr[i].str, cfg_val);
	}

	return count;
}

static const struct file_operations iris_pq_config_fops = {
	.open = simple_open,
	.write = iris_pq_config_write,
};


int iris_dbgfs_pq_init(struct dsi_display *display)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}
	if (debugfs_create_file("iris_pq_config", 0644, pcfg->dbg_root, display,
				&iris_pq_config_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}
	return 0;
}

int iris_update_backlight(u32 bl_lvl)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		return iris_update_backlight_i7(bl_lvl);
	else
		return iris_update_backlight_i7p(bl_lvl);
}

int iris_update_backlight_value(u32 bl_lvl)
{
	int rc = 0;
	struct iris_cfg *pcfg = NULL;
	struct dsi_panel *panel;

	pcfg = iris_get_cfg();
	panel = pcfg->panel;

	if (!pcfg->switch_bl_endian)
		bl_lvl = (((bl_lvl & 0xff) << 8) | (bl_lvl >> 8));

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_setting.quality_cur.ai_backlight = ((u32)pcfg->panel_nits*bl_lvl)/panel->bl_config.bl_max_level;

	return rc;
}

void iris_dom_set(int mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_dom_set_i7(mode);
	else
		iris_dom_set_i7p(mode);
}

static void iris_brightness_para_set(uint32_t *values)
{
	struct iris_update_regval regval;
	uint32_t dimmingGain = 0;
	uint32_t dimmingEnable = 0;

	if (values == NULL) {
		IRIS_LOGE("brightness value is empty");
		return;
	}

	dimmingEnable = values[4];
	IRIS_LOGD("dimming set: enable %d, values: 0x%x\n", dimmingEnable, values[1]);
	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x60;    //DIM_CTRL/FD
	regval.mask = 0x00000001;
	regval.value = dimmingEnable?0x1:0x0;

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x60, 0x60, 0x1);

	dimmingGain = ((values[1] << 16) | values[0]);
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x61, 2, dimmingGain);
	dimmingGain = values[2];
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x61, 3, dimmingGain);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x61, 0x61, 0x01);

}

void iris_brightness_para_reset(void)
{
	struct iris_update_regval regval;
	uint32_t  *payload = NULL;

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x60;    //DIM_CTRL/FD
	regval.mask = 0x00000001;
	regval.value = 0x0;

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x60, 0x60, 0x1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x61, 2);
	if (!payload)
		return;

	payload[0] = 0x40004000;
	payload[1] = 0x4000;
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x61, 0x61, 0x01);
	iris_end_dpp(true);
}

void iris_csc2_para_set(uint32_t *values)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_csc2_para_set_i7(values);
	else
		iris_csc2_para_set_i7p(values);
}

void iris_csc2_para_reset(void)
{
	uint32_t  *payload = NULL;
	int i;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x33, 2);
	if (!payload)
		return;
	for (i = 0; i < 5; i++)
		payload[i] = dwCSCCoffBuffer[i];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x33, 0x33, 0x01);
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xc0, 2);
	if (!payload)
		return;
	for (i = 0; i < 3; i++)
		payload[i] = dwCSCCoffBuffer[i + 5];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0xc0, 0xc0, 0x01);

	//pre csc
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x34, 2);
	if (!payload)
		return;
	for (i = 0; i < 5; i++)
		payload[i] = dwCSCCoffBuffer[i];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x34, 0x34, 0x01);
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0xd0, 2);
	if (!payload)
		return;
	for (i = 0; i < 3; i++)
		payload[i] = dwCSCCoffBuffer[i + 5];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0xd0, 0xd0, 0x01);
	iris_end_dpp(true);

	for (i = 0; i < 12; i++)
		dwCSC2CoffBuffer[i] = dwCSC2CoffDefault[i];
}

void iris_csc_para_set(uint32_t *values)
{
	uint32_t  *payload = NULL;
	int i;

	if (values == NULL) {
		IRIS_LOGE("brightness value is empty");
		return;
	}
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x40, 3);
	if (!payload)
		return;
	for (i = 0; i < 8; i++)
		payload[i] = values[i];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x40, 0x40, 0x01);
	iris_end_dpp(true);
}

void iris_csc_para_reset(void)
{
	uint32_t  *payload = NULL;
	int i;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x40, 3);
	if (!payload)
		return;
	for (i = 0; i < 8; i++)
		payload[i] = dwCSCCoffBuffer[i];
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x40, 0x40, 0x01);
	iris_end_dpp(true);
}

void iris_brightness_level_set(u32 *value)
{
	iris_brightness_para_set(value);
	iris_end_dpp(true);
}

int32_t iris_parse_color_temp_range(struct device_node *np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	/* 2500K~7500K for default */
	pcfg->min_color_temp = 2500;
	pcfg->max_color_temp = 7500;

	rc = p_dts_ops->read_u32(np, "pxlw,min-color-temp", &(pcfg->min_color_temp));
	if (rc) {
		IRIS_LOGE("can not get property: pxlw,min-color-temp");
		return rc;
	}
	IRIS_LOGI("pxlw,min-color-temp: %d", pcfg->min_color_temp);

	rc = p_dts_ops->read_u32(np, "pxlw,max-color-temp", &(pcfg->max_color_temp));
	if (rc) {
		IRIS_LOGE("can not get property:pxlw,max-color-temp");
		return rc;
	}
	IRIS_LOGI("pxlw,max-color-temp: %d", pcfg->max_color_temp);

	return rc;
}

static int32_t _iris_count_ip(const uint8_t *data, int32_t len, int32_t *pval)
{
	int tmp = 0;
	int i = 0;
	int j = 0;

	if (data == NULL || len == 0 || pval == NULL) {
		IRIS_LOGE("%s(), invalid data or pval or len", __func__);
		return -EINVAL;
	}

	tmp = data[0];
	len = len >> 1;

	for (i = 0; i < len; i++) {
		if (tmp == data[2 * i]) {
			pval[j]++;
		} else {
			tmp = data[2 * i];
			j++;
			pval[j]++;
		}
	}

	/*j begin from 0*/
	return j + 1;
}

static int32_t _iris_alloc_pq_init_space(struct iris_cfg *pcfg,
		const uint8_t *pdata, int32_t item_cnt)
{
	int32_t i = 0;
	int32_t size = 0;
	int32_t ip_cnt = 0;
	int32_t rc = 0;
	int32_t *ptr = NULL;
	struct iris_pq_init_val *pinit_val = &pcfg->pq_init_val;

	if (pdata == NULL || item_cnt == 0) {
		IRIS_LOGE("%s(), invalid input, data: %p, size: %d", __func__, pdata, item_cnt);
		return -EINVAL;
	}

	size = sizeof(*ptr) * (item_cnt >> 1);
	ptr = vmalloc(size);
	if (ptr == NULL) {
		IRIS_LOGE("can not malloc space for ptr");
		return -EINVAL;
	}
	memset(ptr, 0x00, size);

	ip_cnt = _iris_count_ip(pdata, item_cnt, ptr);
	if (ip_cnt <= 0) {
		IRIS_LOGE("can not static i_p option");
		rc = -EINVAL;
		goto EXIT_FREE;
	}

	pinit_val->ip_cnt = ip_cnt;
	size = sizeof(struct iris_pq_ipopt_val) * ip_cnt;
	pinit_val->val = vmalloc(size);
	if (pinit_val->val == NULL) {
		IRIS_LOGE("can not malloc pinit_val->val");
		rc = -EINVAL;
		goto EXIT_FREE;
	}

	for (i = 0; i < ip_cnt; i++) {
		pinit_val->val[i].opt_cnt = ptr[i];
		size = sizeof(uint8_t) * ptr[i];
		pinit_val->val[i].popt = vmalloc(size);
	}

EXIT_FREE:
	vfree(ptr);
	ptr = NULL;

	return rc;
}
int32_t iris_parse_default_pq_param(struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	int32_t item_cnt = 0;
	int32_t rc = 0;
	const uint8_t *key = "pxlw,iris-pq-default-val";
	const uint8_t *pdata = NULL;
	struct iris_pq_init_val *pinit_val = &pcfg->pq_init_val;
	struct iris_dts_ops *p_dts_ops = iris_get_dts_ops();

	if (!p_dts_ops)
		return -EINVAL;

	pdata = p_dts_ops->get_property(np, key, &item_cnt);
	if (!pdata) {
		IRIS_LOGE("%s pxlw,iris-pq-default-val fail", __func__);
		return -EINVAL;
	}

	rc = _iris_alloc_pq_init_space(pcfg, pdata, item_cnt);
	if (rc) {
		IRIS_LOGE("malloc error");
		return rc;
	}

	for (i = 0; i < pinit_val->ip_cnt; i++) {
		struct iris_pq_ipopt_val *pval = &(pinit_val->val[i]);

		pval->ip = pdata[k++];
		for (j = 0; j < pval->opt_cnt; j++) {
			pval->popt[j] = pdata[k];
			k += 2;
		}
		/*need to skip one*/
		k -= 1;
	}

	if (IRIS_IF_LOGV()) {
		IRIS_LOGE("ip_cnt = %0x", pinit_val->ip_cnt);
		for (i = 0; i < pinit_val->ip_cnt; i++) {
			char ptr[256];
			int32_t len = 0;
			int32_t sum = 256;
			struct iris_pq_ipopt_val *pval = &(pinit_val->val[i]);

			snprintf(ptr, sum, "i_p is %0x opt is ", pval->ip);
			for (j = 0; j < pval->opt_cnt; j++) {
				len = strlen(ptr);
				sum -= len;
				snprintf(ptr + len, sum, "%0x ", pval->popt[j]);
			}
			IRIS_LOGE("%s", ptr);
		}
	}

	return rc;
}

void iris_pwil_dpp_en(bool dpp_en)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		iris_pwil_dpp_en_i7(dpp_en);
	else
		iris_pwil_dpp_en_i7p(dpp_en);
}

int iris_EDR_backlight_ctrl(u32 hdr_nit, u32 ratio_panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int32_t rc = 0;

	if (pcfg->iris_chip_type == CHIP_IRIS7)
		rc = iris_EDR_backlight_ctrl_i7(hdr_nit, ratio_panel);
	else
		rc = -EINVAL;

	return rc;
}
