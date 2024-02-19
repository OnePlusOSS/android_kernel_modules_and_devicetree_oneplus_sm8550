/*
 * Copyright (c) 2021 ZEKU Technology(Shanghai) Corp.,Ltd, all rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Basecode Created :        2021/04/25 Author: zhanghong@zeku.com
 *
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include "../explorer/include/ap_boot.h"
#include "../explorer/include/main.h"
#include "../explorer/include/power.h"
#include "../explorer/include/uapi/explorer_uapi.h"
#include "include/slt.h"
#include "include/slt_loader.h"

#ifdef PBL_2nd
int explorer_load_stl_1st_pbl(struct explorer_plat_data *epd) {
	int ret = 0;
	char name[NAME_MAX_LEN] = {0};

	if (!epd) {
		pr_err("%s: Invalid parameter, epd is null\n", __func__);
		return -1;
	}
	if (epd->ebs.is_cc_sec_boot) {
		snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SEC_SLT_PBL_1ST_NAME);
	} else {
		snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SLT_PBL_1ST_NAME);
	}
	ret = explorer_load_fw(epd, name, FW_PBL_ADDR);
	return ret;
}

int explorer_load_stl_2nd_pbl(struct explorer_plat_data *epd) {
	int ret = 0;
	char name[NAME_MAX_LEN] = {0};

	if (!epd) {
		pr_err("%s: Invalid parameter, epd is null\n", __func__);
		return -1;
	}
	if (epd->ebs.is_cc_sec_boot) {
		snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SEC_SLT_PBL_2ND_NAME);
	} else {
		snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SLT_PBL_2ND_NAME);
	}
	ret = explorer_load_fw(epd, name, SLT_FW_2ND_PBL_ADDR);
	return ret;
}
#endif

int explorer_load_stl_normal_pbl(struct explorer_plat_data *epd)
{
	int ret = 0;
	char name[NAME_MAX_LEN] = {0};
	if (!epd) {
		pr_err("%s: Invalid parameter, epd is null\n", __func__);
		return -1;
	}
	if (epd->ebs.is_cc_sec_boot) {
		snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SEC_SLT_NORMAL_PBL_NAME);
		ret = explorer_load_fw(epd, name, FW_PBL_ADDR);
		if (ret < 0) {
			ret = explorer_load_fw(epd, PBL_SEC_NAME, FW_PBL_ADDR);
		}
	} else {
		snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SLT_NORMAL_PBL_NAME);
		ret = explorer_load_fw(epd, name, FW_PBL_ADDR);
		if (ret < 0) {
			ret = explorer_load_fw(epd, PBL_NAME, FW_PBL_ADDR);
		}
	}
	return ret;
}

int explorer_load_stl_pbl(struct explorer_plat_data *epd)
{
	int ret = 0;
#ifndef PBL_2nd
	char name[NAME_MAX_LEN] = {0};
#endif

	if (!epd) {
		pr_err("%s: Invalid parameter, epd is null\n", __func__);
		return -1;
	}
#ifdef PBL_2nd
	ret = explorer_load_stl_1st_pbl(epd);
	ret += explorer_load_stl_2nd_pbl(epd);
#else
	if (epd->ebs.is_cc_sec_boot) {
		snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SEC_SLT_PBL_NAME);
	} else {
		snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SLT_PBL_NAME);
	}
	ret = explorer_load_fw(epd, name, FW_PBL_ADDR);
#endif
	return ret;
}

int explorer_load_stl_sdi(struct explorer_plat_data *epd)
{
	int ret = 0;
	char name[NAME_MAX_LEN] = {0};
	if (!epd) {
		pr_err("%s: Invalid parameter, epd is null\n", __func__);
		return -1;
	}
	if (epd->ebs.is_cc_sec_boot) {
		ret = explorer_load_fw(epd, SDI_SEC_NAME, FW_SDI_ADDR);
		if (ret < 0) {
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SEC_SLT_SDI_NAME);
			ret = explorer_load_fw(epd, name, FW_SDI_ADDR);
		}
	} else {
		ret = explorer_load_fw(epd, SDI_NAME, FW_SDI_ADDR);
		if (ret < 0) {
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SLT_SDI_NAME);
			ret = explorer_load_fw(epd, name, FW_SDI_ADDR);
		}
	}
	return ret;
}

int explorer_load_stl_os(struct explorer_plat_data *epd)
{
	int ret = 0;
	char name[NAME_MAX_LEN] = {0};

	if (!epd) {
		pr_err("%s, invalid parameter, epd is null\n", __func__);
		return -EINVAL;
	}

	if (epd->ebs.is_cc_sec_boot) {
		if (epd->ebi.rmod == RTOS_SLT_AON)
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SEC_SLT_OS_AON_NAME);
		else if (epd->ebi.rmod == RTOS_SLT)
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SEC_SLT_OS_NAME);
		else {
			pr_err("%s, invalid ebi.rmod(%d).\n", __func__, epd->ebi.rmod);
			return -EINVAL;
		}
	} else {
		if (epd->ebi.rmod == RTOS_SLT_AON)
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SLT_OS_AON_NAME);
		else if (epd->ebi.rmod == RTOS_SLT)
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SLT_OS_NAME);
		else {
			pr_err("%s, invalid ebi.rmod(%d).\n", __func__, epd->ebi.rmod);
			return -EINVAL;
		}
	}
	ret = explorer_load_fw(epd, name, epd->ebi.rpos==RTOS_SRAM ? FW_OS_ADDR:FW_OS_DDR_ADDR);
	return ret;
}

int explorer_load_stl_se_fw(struct explorer_plat_data *epd) {
	int ret = 0;
	char name[NAME_MAX_LEN] = {0};

	if (!epd) {
		pr_err("%s: Invalid parameter, epd is null\n", __func__);
		return -1;
	}
	snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SLT_SE_DATA_NAME);
	ret = explorer_load_fw(epd, name, FW_SE_DATA_ADDR);
	return ret;
}

int explorer_load_stl_se_header(struct explorer_plat_data *epd) {
	int ret = 0;
	char name[NAME_MAX_LEN] = {0};

	if (!epd) {
		pr_err("%s: Invalid parameter, epd is null\n", __func__);
		return -1;
	}
	snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, SLT_SE_HEAD_NAME);
	ret = explorer_load_fw(epd, name, FW_SE_HEAD_ADDR);
	return ret;
}

int explorer_load_hash(struct explorer_plat_data *epd, const char *sub_str) {
	int ret = 0;
	char name[NAME_MAX_LEN] = {0};
	switch (epd->cmod) {
		case CAM_BZ_BK:
			snprintf(name, sizeof(name), "%s/isp/ud_%s_hash.bin", SLT_FIRMWARE_ROOT_DIR, sub_str);
			break;
		case CAM_LW_BK:
			snprintf(name, sizeof(name), "%s/isp/sn_%s_hash.bin", SLT_FIRMWARE_ROOT_DIR, sub_str);
			break;
		default:
			snprintf(name, sizeof(name), "%s/isp/%s_hash.bin", SLT_FIRMWARE_ROOT_DIR, sub_str);
			break;
	}
	pr_info("%s: load hash:%s\n", __func__, name);
	ret = explorer_load_fw(epd, name, SLT_FW_HASH_ADDR);
	return ret;
}

int explorer_load_npu_firmware_or_mode_set(struct explorer_plat_data *epd, enum slt_fw_type fw_type) {
	int ret = 0;
	char name[NAME_MAX_LEN] = {0};
	const char *sub_str = NULL;
	const char *hash_sub_str = NULL;
	const char *n1, *n2, *n3, *n4, *n5;
	u32  npu_addr1 = FW_NPU_ADDR1, npu_addr2 = FW_NPU_ADDR2;
	u32  npu_addr3 = FW_NPU_ADDR3, npu_addr4 = FW_NPU_ADDR4, npu_addr5 = FW_NPU_ADDR5;
	switch (fw_type) {
		case SLT_FW_NPU_UD:
			sub_str = "ud";
			hash_sub_str = BAYER_12;
			epd->cmod = CAM_BZ_BK;
			break;
		case SLT_FW_NPU_SN:
			sub_str = "sn";
			hash_sub_str = BAYER_12;
			epd->cmod = CAM_LW_BK;
			break;
		case SLT_FW_NPU_AON:
			sub_str = "aon";
			break;
		case SLT_FW_NO_NPU_NM:
			//only set camera mode
			hash_sub_str = BAYER_12;
			epd->cmod = CAM_NORMAL;
			break;
		default:
			pr_info("%s: no support NPU/mode type(%d), return\n", __func__, fw_type);
			break;
	}

	if (hash_sub_str != NULL) {
		ret = explorer_load_hash(epd, hash_sub_str);
		if (ret < 0) {
			pr_err("%s: load hash %s, fail\n", __func__, name);
			ret = -EINVAL;
			goto exit;
		}
	}

	if (sub_str == NULL) {
		goto exit;
	}
	if (epd->ebs.is_cc_sec_boot) {
		n1 = SEC_NPU_FW_1;
		n2 = SEC_NPU_FW_2;
		n3 = SEC_NPU_FW_3;
		n4 = SEC_NPU_FW_4;
		n5 = SEC_NPU_FW_5;
	} else {
		n1 = NPU_FW_1;
		n2 = NPU_FW_2;
		n3 = NPU_FW_3;
		n4 = NPU_FW_4;
		n5 = NPU_FW_5;
	}
	if (fw_type == SLT_FW_NPU_AON) {
		//overlay npu address
		npu_addr1 = AON_FW_NPU_ZMODEL_ADDR;
		npu_addr2 = AON_FW_NPU_CMEM_ADDR;
		npu_addr3 = AON_FW_NPU_VSP_ADDR;
		npu_addr4 = AON_FW_NPU_DDR_CMEM_ADDR;
		npu_addr5 = AON_FW_NPU_MMP_ADDR;
	}
	memset(name, 0x00, sizeof(name));
	snprintf(name, sizeof(name), "%s/npu/%s/%s", SLT_FIRMWARE_ROOT_DIR, sub_str, n1);
	ret = explorer_load_fw(epd, name, npu_addr1);
	if (ret < 0) {
		pr_err("%s: load %s fail, ret=%d\n", __func__, name, ret);
		ret = -EINVAL;
		goto exit;
	}

	memset(name, 0x00, sizeof(name));
	snprintf(name, sizeof(name), "%s/npu/%s/%s", SLT_FIRMWARE_ROOT_DIR, sub_str, n2);
	ret = explorer_load_fw(epd, name, npu_addr2);
	if (ret < 0) {
		pr_err("%s: load %s fail, ret=%d\n", __func__, name, ret);
		ret = -EINVAL;
		goto exit;
	}
	memset(name, 0x00, sizeof(name));
	snprintf(name, sizeof(name), "%s/npu/%s/%s", SLT_FIRMWARE_ROOT_DIR, sub_str, n3);
	ret = explorer_load_fw(epd, name, npu_addr3);
	if (ret < 0) {
		pr_err("%s: load %s fail, ret=%d\n", __func__, name, ret);
		ret = -EINVAL;
		goto exit;
	}
	memset(name, 0x00, sizeof(name));
	snprintf(name, sizeof(name), "%s/npu/%s/%s", SLT_FIRMWARE_ROOT_DIR, sub_str, n4);
	ret = explorer_load_fw(epd, name, npu_addr4);
	if (fw_type == SLT_FW_NPU_AON) {
		//workaround for aon ddr_cmem is 0 size;
		ret = 0;
	}

	if (ret < 0) {
		pr_err("%s: load %s fail, ret=%d\n", __func__, name, ret);
		ret = -EINVAL;
		goto exit;
	}
	memset(name, 0x00, sizeof(name));
	snprintf(name, sizeof(name), "%s/npu/%s/%s", SLT_FIRMWARE_ROOT_DIR, sub_str, n5);
	ret = explorer_load_fw(epd, name, npu_addr5);

	if (ret < 0) {
		pr_err("%s: load %s fail, ret=%d\n", __func__, name, ret);
		ret = -EINVAL;
		goto exit;
	}
exit:
	if (ret < 0) {
		//clean cmod
		epd->cmod = CAM_NORMAL;
	}
	return ret;
}

int explorer_load_isp_firmware(struct explorer_plat_data *epd, enum slt_fw_type fw_type, bool hash) {
	int ret = 0;
	char name[NAME_MAX_LEN] = {0};
	u32  addr = 0;
	u32  pic_offset = 0;
	u8   group_num = 0;
	u8   pic_num_per_group = 0;
	const char *sub_str = NULL;
	int i = 0;
	bool only_hash =  false;

	switch (fw_type) {
		case SLT_FW_ISP_BAYER_12B:
			addr = SLT_FW_ISP_BAYER_12B_ADDR_S;
			pic_offset = PIC_ALIGN_SIZE_4656_720;
			group_num = TYPE_BAYER_12_PIC_GROUP;
			pic_num_per_group = TYPE_BAYER_12_PIC_NUM_PER_GROUP;
			sub_str = BAYER_12;
			hash = false; //load hash when npu loading
			break;
		case SLT_FW_ISP_BAYER_14B:
#if 0
			addr = SLT_FW_ISP_BAYER_14B_ADDR_S;
			pic_offset = PIC_ALIGN_SIZE_4656_720;
			group_num = TYPE_BAYER_14_PIC_GROUP;
			pic_num_per_group = TYPE_BAYER_14_PIC_NUM_PER_GROUP;
#else
			only_hash = true;
#endif
			sub_str = BAYER_14;
			epd->cmod = CAM_NORMAL;
			break;
		case SLT_FW_ISP_NCFA:
			addr = SLT_FW_NCFA_ADDR_S;
			pic_offset = PIC_ALIGN_SIZE_4656_720_NCFA;
			group_num = TYPE_NCFA_PIC_GROUP;
			pic_num_per_group = TYPE_NCFA_PIC_NUM_PER_GROUP;
			sub_str = NCFA;
			epd->cmod = CAM_NORMAL;
			break;
		case SLT_FW_ISP_RGBW:
			addr = SLT_FW_RGBW_ADDR_S;
			pic_offset = PIC_ALIGN_SIZE_4656_720_RGBW;
			group_num = TYPE_RGBW_PIC_GROUP;
			pic_num_per_group = TYPE_RGBW_PIC_NUM_PER_GROUP;
			sub_str = RGBW;
			epd->cmod = CAM_NORMAL;
			break;
		case SLT_FW_ISP_RGBW_FULL:
			addr = SLT_FW_ISP_RGBW_F_ADDR_S;
			pic_offset = PIC_ALIGN_SIZE_8736_128;
			group_num = TYPE_RGBW_F_PIC_GROUP;
			pic_num_per_group = TYPE_RGBW_F_PIC_NUM_PER_GROUP;
			sub_str = RGBW_FULL;
			epd->cmod = CAM_NORMAL;
			break;
		case SLT_FW_ISP_RSGN:
			addr = SLT_FW_ISP_RSGN_ADDR_S;
			pic_offset = PIC_ALIGN_SIZE_4672_736;
			group_num = TYPE_RSGN_PIC_GROUP;
			pic_num_per_group = TYPE_RSGN_PIC_NUM_PER_GROUP;
			sub_str = RSGN;
			epd->cmod = CAM_NORMAL;
			break;
		case SLT_FW_ISP_AON:
			addr = SLT_FW_ISP_AON_ADDR_S;
			pic_offset = PIC_ALIGN_SIZE_416_416;
			group_num = TYPE_AON_PIC_GROUP;
			pic_num_per_group = TYPE_AON_PIC_NUM_PER_GROUP;
			sub_str = AON;
			epd->cmod = CAM_AON;
			break;
		/* default */
		default:
			pr_err("%s: unsupport SLT FW type(%d), do nothing and exit\n", __func__, fw_type);
			ret = -EINVAL;
			break;
	}
	if (ret < 0) {
		goto exit;
	}
	//load hash if need
	if (hash) {
		ret = explorer_load_hash(epd, sub_str);
		if (ret < 0) {
			pr_err("%s: load hash %s, fail\n", __func__, name);
			ret = -EINVAL;
			goto exit;
		}
	}

	if (only_hash) {
		pr_info("%s: only need load hash, so exit\n", __func__);
		goto exit;
	}
	//check parameters
	if (addr == 0) {
		pr_err("%s: Invalid load address, exit\n", __func__);
		ret = -EINVAL;
		goto exit;
	}
	if (sub_str == NULL) {
		pr_err("%s: Invalid sub str, exit\n", __func__);
		ret = -EINVAL;
		goto exit;
	}
	if ((group_num > ISP_MAX_GROUP)
		|| (group_num <= 0)
		|| (pic_num_per_group > ISP_MAX_NUM_PER_GROUP)
		|| (pic_num_per_group <= 0)) {
		pr_err("%s: Invalid group definition: group[%d], num_per_group[%d], exit\n", __func__, group_num, pic_num_per_group);
		ret = -EINVAL;
		goto exit;
	}
	//load
	pr_info("%s: Load ISP firmware start: type:%s, group num:%d, picture num per group:%d\n", __func__, sub_str, group_num, pic_num_per_group);
	switch (group_num) {
		case 1:
			for (i = 0; i < pic_num_per_group; i++) {
				memset(name, 0x00, sizeof(name));
				snprintf(name, sizeof(name), "%s/isp/%s/isp_%s_%s_%d.raw", SLT_FIRMWARE_ROOT_DIR, sub_str, sub_str, FRAME_L, i);
				ret = explorer_load_fw(epd, name, addr);
				if (ret < 0) {
					break;
				}
				addr += pic_offset;
			}
			break;
		case 2:
			for (i = 0; i < pic_num_per_group; i++) {
				memset(name, 0x00, sizeof(name));
				snprintf(name, sizeof(name), "%s/isp/%s/isp_%s_%s_%d.raw", SLT_FIRMWARE_ROOT_DIR, sub_str, sub_str, FRAME_L, i);
				ret = explorer_load_fw(epd, name, addr);
				if (ret < 0) {
					break;
				}
				addr += pic_offset;
				memset(name, 0x00, sizeof(name));
				snprintf(name, sizeof(name), "%s/isp/%s/isp_%s_%s_%d.raw", SLT_FIRMWARE_ROOT_DIR, sub_str, sub_str, FRAME_S, i);
				ret = explorer_load_fw(epd, name, addr);
				if (ret < 0) {
					break;
				}
				addr += pic_offset;
			}
			break;
		case 3:
			for (i = 0; i < pic_num_per_group; i++) {
				memset(name, 0x00, sizeof(name));
				snprintf(name, sizeof(name), "%s/isp/%s/isp_%s_%s_%d.raw", SLT_FIRMWARE_ROOT_DIR, sub_str, sub_str, FRAME_L, i);
				ret = explorer_load_fw(epd, name, addr);
				if (ret < 0) {
					break;
				}
				addr += pic_offset;
				memset(name, 0x00, sizeof(name));
				snprintf(name, sizeof(name), "%s/isp/%s/isp_%s_%s_%d.raw", SLT_FIRMWARE_ROOT_DIR, sub_str, sub_str, FRAME_M, i);
				ret = explorer_load_fw(epd, name, addr);
				if (ret < 0) {
					break;
				}
				addr += pic_offset;
				memset(name, 0x00, sizeof(name));
				snprintf(name, sizeof(name), "%s/isp/%s/isp_%s_%s_%d.raw", SLT_FIRMWARE_ROOT_DIR, sub_str, sub_str, FRAME_S, i);
				ret = explorer_load_fw(epd, name, addr);
				if (ret < 0) {
					break;
				}
				addr += pic_offset;
			}
			break;
		default:
			break;
	}
exit:
	return ret;
}

int explorer_load_firmware_only(struct explorer_plat_data *epd, enum slt_fw_type fw_type)
{
	int ret = 0;
	u32 load_addr = 0;
	char name[NAME_MAX_LEN] = {0};
	switch (fw_type) {
		/* DDR firmware */
		case SLT_FW_DDR_ATE:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_ATE_FW);
			ret = explorer_load_fw(epd, name, load_addr);
			break;
		case SLT_FW_DDR_DIAG_L:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_DIAG_FW_L);
			ret = explorer_load_fw(epd, name, load_addr);
			break;
		case SLT_FW_DDR_DIAG_M:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_DIAG_FW_M);
			ret = explorer_load_fw(epd, name, load_addr);
			break;
		case SLT_FW_DDR_DIAG_H:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_DIAG_FW_H);
			ret = explorer_load_fw(epd, name, load_addr);
			break;
		case SLT_FW_DDR_DIAG_C:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_DIAG_FW_C);
			ret = explorer_load_fw(epd, name, load_addr);
			break;
		case SLT_FW_DDR_FIRST_FULL:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			ret = explorer_load_fw(epd, DDR_1ST_NAME, load_addr);
			if (ret < 0) {
				snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_FIRST_BOOT_FW);
				ret = explorer_load_fw(epd, name, load_addr);
			}
			break;
		case SLT_FW_DDR_FIRST_L:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_FIRST_L_BOOT_FW);
			ret = explorer_load_fw(epd, name, load_addr);
			break;
		case SLT_FW_DDR_FIRST_M:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_FIRST_M_BOOT_FW);
			ret = explorer_load_fw(epd, name, load_addr);
			break;
		case SLT_FW_DDR_FIRST_H:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_FIRST_H_BOOT_FW);
			ret = explorer_load_fw(epd, name, load_addr);
			break;
		case SLT_FW_DDR_FIRST_C:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_FIRST_C_BOOT_FW);
			ret = explorer_load_fw(epd, name, load_addr);
			break;
		case SLT_FW_DDR_PATTERN:
#ifdef PBL_2nd
			load_addr = SLT_FW_2ND_DDR_ADDR;
#else
			load_addr = FW_DDR_ADDR;
#endif
			snprintf(name, sizeof(name), "%s/%s", SLT_FIRMWARE_ROOT_DIR, DDR_FIX_PATTERN);
			ret = explorer_load_fw(epd, name, load_addr);
			break;
		case SLT_FW_ISP_ALL:
			ret = explorer_load_isp_firmware(epd, SLT_FW_ISP_BAYER_12B, false);
			if (ret < 0) {
				break;
			}
			ret = explorer_load_isp_firmware(epd, SLT_FW_ISP_NCFA, false);
			if (ret < 0) {
				break;
			}
			ret = explorer_load_isp_firmware(epd, SLT_FW_ISP_RGBW, false);
			if (ret < 0) {
				break;
			}
			ret = explorer_load_isp_firmware(epd, SLT_FW_ISP_BAYER_14B, false);
			if (ret < 0) {
				break;
			}
			ret = explorer_load_isp_firmware(epd, SLT_FW_ISP_RGBW_FULL, false);
			if (ret < 0) {
				break;
			}
			ret = explorer_load_isp_firmware(epd, SLT_FW_ISP_RSGN, false);
			if (ret < 0) {
				break;
			}
			ret = explorer_load_isp_firmware(epd, SLT_FW_ISP_AON, false);
			break;
		/* default */
		default:
			ret = explorer_load_isp_firmware(epd, fw_type, true);
			break;
	}
	return ret;
}

int wait_firmware_on(struct explorer_plat_data *epd, struct explorer_slt_fw *target_fw)
{
	int ret = 0;
	struct explorer_boot_info *ebi;
	struct explorer_boot_status *ebs;

	if (!epd || !target_fw) {
		pr_err("%s: Invalid parameter, epd is null\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	ebi = &(epd->ebi);
	ebs = &(epd->ebs);

	if (target_fw->type >= MAX_SLT_FW_TYPE) {
		pr_err("%s: unsupport SLT FW type(%d)\n", __func__, target_fw->type);
		ret = -EINVAL;
		goto exit;
	}

	/* boot stage mode, directly reset pmic to  */
	if (target_fw->type <= SLT_BOOT_STAGE_TYPE) {
		//Clean ebi first for next boot;
		memset(&epd->ebi, 0x00, sizeof(struct explorer_boot_info));
		#ifdef HAPS
		/* Add warm reset by mailbox, todo */
		#else
		/* PMIC reset here */
		if (epd->is_pmic_pon && (ebs->slt_has_booted == true)) {
			pr_info("%s: target_fw->type(%d), current stage:%d, reset pmic start\n", __func__, target_fw->type, atomic_read(&ebs->current_stage));
			power_reset_explorer(epd);
		}
		#endif
	} else if (target_fw->type <= SLT_LAST_PBL_STAGE_FM) {
		/* check is in SLT PBL stage */
		if (atomic_read(&ebs->current_stage) != PBL_SLT) {
			pr_err("%s: FW type stage is not match(type:%d, stage:%d)\n", __func__, target_fw->type, atomic_read(&ebs->current_stage));
			ret = -EINVAL;
			goto exit;
		}
	} else if (target_fw->type <= SLT_LAST_OS_STAGE_FM) {
	    /* check is in SLT OS stage? */
		if ((ebs->cur_os_type != SLT_OS)
			&& (ebs->cur_os_type != SLT_OS_AON)) {
			pr_err("%s: FW type stage is not match(type:%d, stage:%d, os type:%d)\n", __func__, target_fw->type, atomic_read(&ebs->current_stage), ebs->cur_os_type);
			ret = -EINVAL;
			goto exit;
		}
	}
	/* load fw */
	switch(target_fw->type) {
		case SLT_FW_PBL:
			/* boot to SLT pbl stage */
			epd->ebi.target_stage = PBL_SLT;
			epd->ebi.boot_timeout = 1000; // pbl mode 1000ms
			epd->ebi.dmod = DDR_FIRST;
			epd->ebi.mmod = MIPI_NORMAL;
			epd->ebi.rmod = RTOS_NULL;
			ret = explorer_boot(epd);
			ebs->slt_has_booted = true;
			break;
		case SLT_FW_OS:
		case SLT_FW_OS_QUICK:
		case SLT_FW_OS_DDR:
		case SLT_FW_OS_DDR_QUICK:
			/* boot SLT OS stage */
			epd->ebi.target_stage = OS_OK;
			if (target_fw->type == SLT_FW_OS_QUICK
				|| target_fw->type == SLT_FW_OS_DDR_QUICK) {
				epd->ebi.boot_timeout = 2000; // quick rtos mode 2000ms
				// #ifndef OPLUS_FEATURE_CAMERA_COMMON
				// epd->ebi.dmod = DDR_QUICK;
				// #else OPLUS_FEATURE_CAMERA_COMMON
				epd->ebi.dmod = DDR_BUFFER;
				// #endif OPLUS_FEATURE_CAMERA_COMMON
			} else {
				epd->ebi.boot_timeout = 4000; // rtos mode 4000ms
				epd->ebi.dmod = DDR_FIRST;
			}
			epd->ebi.mmod = MIPI_NORMAL;
			epd->ebi.rmod = RTOS_SLT;
			if (target_fw->type == SLT_FW_OS
				|| target_fw->type == SLT_FW_OS_QUICK) {
				epd->ebi.rpos = RTOS_SRAM;
			} else {
				epd->ebi.rpos = RTOS_DDR;
			}
			ret = explorer_boot(epd);
			ebs->slt_has_booted = true;
			break;
		case SLT_FW_OS_AON:
		case SLT_FW_OS_AON_QUICK:
		case SLT_FW_OS_AON_DDR:
		case SLT_FW_OS_AON_DDR_QUICK:
			/* boot SLT OS stage */
			epd->ebi.target_stage = OS_OK;
			if (target_fw->type == SLT_FW_OS_AON_QUICK
				|| target_fw->type == SLT_FW_OS_AON_DDR_QUICK) {
				epd->ebi.boot_timeout = 2000; //quick aon 2000ms
				epd->ebi.dmod = DDR_QUICK;
			} else {
				epd->ebi.boot_timeout = 4000; // aon rtos mode 4000ms
				epd->ebi.dmod = DDR_FIRST;
			}
			epd->ebi.mmod = MIPI_NORMAL;
			epd->ebi.rmod = RTOS_SLT_AON;
			if (target_fw->type == SLT_FW_OS_AON
				|| target_fw->type == SLT_FW_OS_AON_QUICK) {
				epd->ebi.rpos = RTOS_SRAM;
			} else {
				epd->ebi.rpos = RTOS_DDR;
			}
			ret = explorer_boot(epd);
			ebs->slt_has_booted = true;
			break;
		case SLT_FW_NPU_UD:
		case SLT_FW_NPU_SN:
		case SLT_FW_NPU_AON:
		case SLT_FW_NO_NPU_NM:
			ret = explorer_load_npu_firmware_or_mode_set(epd, target_fw->type);
			break;
		/* load firmware only */
		default:
			ret = explorer_load_firmware_only(epd, target_fw->type);
			break;
	}
exit:
	return ret;
}
