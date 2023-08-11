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

#ifndef _SLT_LOADER_H
#define _SLT_LOADER_H

#define PBL_2nd   //default disable, not full support so far

#define SLT_FIRMWARE_ROOT_DIR   "explorer/slt"
#define NAME_MAX_LEN            128

/* SLT firmware name */
#define SLT_NORMAL_PBL_NAME     "pbl_slt.bin"
#define SEC_SLT_NORMAL_PBL_NAME "sec_pbl_slt.bin"

#ifndef PBL_2nd
#define SLT_PBL_NAME            "pbl_SLT.bin"
#define SEC_SLT_PBL_NAME        "sec_pbl_SLT.bin"
#else
#define SLT_PBL_1ST_NAME        "pbl_SLT_1st.bin"
#define SLT_PBL_2ND_NAME        "pbl_SLT_2nd.bin"
#define SEC_SLT_PBL_1ST_NAME    "sec_pbl_SLT_1st.bin"
#define SEC_SLT_PBL_2ND_NAME    "sec_pbl_SLT_2nd.bin"
#endif
#define SLT_OS_NAME             "rtthread_slt.bin"
#define SEC_SLT_OS_NAME         "sec_rtthread_slt.bin"
#define SLT_OS_AON_NAME         "rtthread_aon_slt.bin"
#define SEC_SLT_OS_AON_NAME     "sec_rtthread_aon_slt.bin"

#define SLT_SDI_NAME            "sdi.bin"
#define SEC_SLT_SDI_NAME        "sec_sdi.bin"

/* DDR firmware only used by SLT */
#define DDR_ATE_FW               "ddr/ddr_ate.bin"
#define DDR_FIRST_BOOT_FW        "ddr/ddr_first.bin"
#define DDR_FIRST_L_BOOT_FW      "ddr/ddr_first_l.bin"
#define DDR_FIRST_M_BOOT_FW      "ddr/ddr_first_m.bin"
#define DDR_FIRST_H_BOOT_FW      "ddr/ddr_first_h.bin"
#define DDR_FIRST_C_BOOT_FW      "ddr/ddr_first_c.bin"
#define DDR_DIAG_FW_L            "ddr/ddr_first_diag_l.bin"
#define DDR_DIAG_FW_M            "ddr/ddr_first_diag_m.bin"
#define DDR_DIAG_FW_H            "ddr/ddr_first_diag_h.bin"
#define DDR_DIAG_FW_C            "ddr/ddr_first_diag_c.bin"
#define DDR_FIX_PATTERN          "ddr/ddr_pattern.bin"

/* CE firmware only used by SLT */
#define SLT_SE_DATA_NAME          "se/se_fw.bin"
#define SLT_SE_HEAD_NAME          "se/se_header.bin"

/* SLT firmware address */
#ifdef PBL_2nd
#define SLT_FW_2ND_PBL_ADDR		    0x00109C00
/* DDR load address in 2nd PBL */
#define SLT_FW_2ND_DDR_END_ADDR		0x0016E000
#define SLT_DDR_SIZE                0x00040000
#define SLT_FW_2ND_DDR_ADDR         (SLT_FW_2ND_DDR_END_ADDR - SLT_DDR_SIZE)
#endif

/* ISP definition */
#define ISP_MAX_NUM_PER_GROUP                3
#define ISP_MAX_GROUP                        3

#define TYPE_BAYER_12_PIC_GROUP              3
#define TYPE_BAYER_12_PIC_NUM_PER_GROUP      3

#define TYPE_NCFA_PIC_GROUP                  1
#define TYPE_NCFA_PIC_NUM_PER_GROUP          3

#define TYPE_RGBW_PIC_GROUP                  1
#define TYPE_RGBW_PIC_NUM_PER_GROUP          3

#define TYPE_BAYER_14_PIC_GROUP              1
#define TYPE_BAYER_14_PIC_NUM_PER_GROUP      1

#define TYPE_RGBW_F_PIC_GROUP                1
#define TYPE_RGBW_F_PIC_NUM_PER_GROUP        1

#define TYPE_RSGN_PIC_GROUP                  1
#define TYPE_RSGN_PIC_NUM_PER_GROUP          3

#define TYPE_AON_PIC_GROUP                   1
#define TYPE_AON_PIC_NUM_PER_GROUP           3

/* Picture key string definition */
#define FRAME_L                   "L"
#define FRAME_M                   "M"
#define FRAME_S                   "S"

/* sub str definition */
#define BAYER_12                  "bayer"
#define NCFA                      "ncfa"
#define RGBW                      "rgbw"
#define RGBW_FULL                 "rgbw_full"
#define BAYER_14                  "bayer_14b"
#define RSGN                      "rsgn"
#define AON                       "aon"

/* Picture align size, total num */
#define PIC_ALIGN_SIZE_4656_720      0x4EC100
#define PIC_ALIGN_SIZE_4656_720_NCFA 0x9D8100
#define PIC_ALIGN_SIZE_4656_720_RGBW 0x816100
#define PIC_ALIGN_SIZE_8736_128      0x1A0100
#define PIC_ALIGN_SIZE_4672_736      0x508100
#define PIC_ALIGN_SIZE_416_416       0x34100

#define TYPE_BAYER_12_PIC_NUM     (TYPE_BAYER_12_PIC_GROUP * TYPE_BAYER_12_PIC_NUM_PER_GROUP)
#define TYPE_NCFA_PIC_NUM         (TYPE_NCFA_PIC_GROUP * TYPE_NCFA_PIC_NUM_PER_GROUP)
#define TYPE_RGBW_PIC_NUM         (TYPE_RGBW_PIC_GROUP * TYPE_RGBW_PIC_NUM_PER_GROUP)
#define TYPE_RGBW_FULL_PIC_NUM    (TYPE_RGBW_F_PIC_GROUP * TYPE_RGBW_F_PIC_NUM_PER_GROUP)
#define TYPE_BAYER_14_PIC_NUM     (TYPE_BAYER_14_PIC_GROUP * TYPE_BAYER_14_PIC_NUM_PER_GROUP)
#define TYPE_RSGN_PIC_NUM         (TYPE_RSGN_PIC_GROUP * TYPE_RSGN_PIC_NUM_PER_GROUP)
#define TYPE_AON_PIC_NUM          (TYPE_AON_PIC_GROUP * TYPE_AON_PIC_NUM_PER_GROUP)

/* ISP load address */
#define SLT_FW_ISP_BAYER_12B_ADDR_S   0x25826000        //4656 x 720
#define SLT_FW_NCFA_ADDR_S            (SLT_FW_ISP_BAYER_12B_ADDR_S + \
                                         TYPE_BAYER_12_PIC_NUM * PIC_ALIGN_SIZE_4656_720)
#define SLT_FW_RGBW_ADDR_S            0x2A1FAC00
#define SLT_FW_ISP_BAYER_14B_ADDR_S   (SLT_FW_RGBW_ADDR_S + \
                                         TYPE_RGBW_PIC_NUM * PIC_ALIGN_SIZE_4656_720)
#define SLT_FW_ISP_RGBW_F_ADDR_S      0x2DBEC600        //8736 x 128
#define SLT_FW_ISP_RSGN_ADDR_S        0x2DD8C700        //4672 x 736
#define SLT_FW_ISP_AON_ADDR_S         0x2BA3CF00        //416 x 416

#define SLT_FW_HASH_ADDR              0x2BFF0000

//npu image str
#define NPU_FW_1		"compiler.zmodel"
#define NPU_FW_2		"cmem.bin"
#define NPU_FW_3		"vsp_params.bin"
#define NPU_FW_4		"ddr_cmem.bin"
#define NPU_FW_5		"mmp.bin"

#define SEC_NPU_FW_1		"sec_compiler.zmodel"
#define SEC_NPU_FW_2		"sec_cmem.bin"
#define SEC_NPU_FW_3		"sec_vsp_params.bin"
#define SEC_NPU_FW_4		"sec_ddr_cmem.bin"
#define SEC_NPU_FW_5		"sec_mmp.bin"

#define AON_FW_NPU_ZMODEL_ADDR      0x20000000
#define AON_FW_NPU_MMP_ADDR         0x200d0000
#define AON_FW_NPU_CMEM_ADDR        0x200a0000
#define AON_FW_NPU_VSP_ADDR         0x20200000
#define AON_FW_NPU_DDR_CMEM_ADDR    0x200c0000

/* Function definition */
#ifdef PBL_2nd
int explorer_load_stl_1st_pbl(struct explorer_plat_data *epd);
int explorer_load_stl_2nd_pbl(struct explorer_plat_data *epd);
#endif
int explorer_load_stl_pbl(struct explorer_plat_data *epd);
int explorer_load_stl_os(struct explorer_plat_data *epd);
int explorer_load_stl_normal_pbl(struct explorer_plat_data *epd);
int explorer_load_stl_se_fw(struct explorer_plat_data *epd);
int explorer_load_stl_sdi(struct explorer_plat_data *epd);
int explorer_load_stl_se_header(struct explorer_plat_data *epd);

#endif /*_slt_LOADER_H*/
