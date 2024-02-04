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
 * Basecode Created :        2020/10/26 Author: zhangao@zeku.com
 *
 */

#ifndef _AP_BOOT_H_
#define _AP_BOOT_H_

#include "ipc.h"


/* Macros controls */
//#define AUTO_TRIG
#define BOOT_TIME_LOG
#define	BOOT_TIMEOUT_MAX			(200000)

#define SDU_CTL_BASE  0x0800
#define CSPI_CTL_BASE 0x0100

//SDU Control Regs
#define SDU_BOOT_CTRL_REG0     (SDU_CTL_BASE + 0x00) // [RW][7:0    ]
#define SDU_BOOT_RESET_DONE_ACK        (0x01 << 0)
#define SDU_JTAG_IMITATE_EN            (0x01 << 1)
#define SDU_JTAG_TRST                  (0x01 << 2)
#define SDU_JTAG_TDI                   (0x01 << 3)
#define SDU_JTAG_TMS                   (0x01 << 4)
#define SDU_JTAG_TCK                   (0x01 << 5)
#define SDU_RAMDUMP_EN_SDU             (0x01 << 6)
#define SDU_ABNORMAL_BOOT_EN_SDU       (0x01 << 7)

#define SDU_AP_CTRL_REG1	(SDU_CTL_BASE + 0x01)
#define DDR_FW_TYPE			(0x01 << 1)
#define DDR_FW_TYPE_VLD		(0x01 << 2)

#define SDU_AHB_CTRL_REG1      (SDU_CTL_BASE + 0x02) // [RW][23:16  ]
#define SDU_POWER_CTRL_REG0    (SDU_CTL_BASE + 0x03) // [RW][31:24  ]
#define SDU_POWER_PLL_AP_CFG_DONE      (0x01 << 3)
#define SDU_POWER_PLL_SPD_MASK         (0x03 << 1)
#define SDU_POWER_PLL_SPD_LOW          (0x00 << 1)
#define SDU_POWER_PLL_SPD_MID          (0x01 << 1)
#define SDU_POWER_PLL_SPD_HIGH         (0x02 << 1)
#define SDU_RST_CTRL_REG0      (SDU_CTL_BASE + 0x04) // [RW][39:32  ]
#define SDU_RST_MIPI_RX                (0x01 << 0)
#define SDU_RST_MIPI_TX                (0x01 << 1)
#define SDU_RST_DDR                    (0x01 << 2)
#define SDU_RST_NPU                    (0x01 << 3)
#define SDU_RST_ISP_BE                 (0x01 << 4)
#define SDU_RST_ISP_FE1                (0x01 << 5)
#define SDU_RST_ISP_FE0                (0x01 << 6)
#define SDU_RST_ISP_TOP                (0x01 << 7)
#define SDU_RST_CTRL_REG1      (SDU_CTL_BASE + 0x05) // [RW][47:40  ]
#define SDU_RST_SPMI                   (0x01 << 2)
#define SDU_RST_APBHS                  (0x01 << 3)
#define SDU_RST_APBLS                  (0x01 << 4)
#define SDU_RST_AHB                    (0x01 << 5)
#define SDU_RST_BUSNIC                 (0x01 << 6)
#define SDU_RST_CPUCORE                (0x01 << 7)
#define SDU_AP_CTRL_REG6       (SDU_CTL_BASE + 0x06) // [RW][55:48  ]
#define SDU_AP_CTRL_REG7	(SDU_CTL_BASE + 0x07) // [RW][63:56  ]
    #define MIPI_BP_EN          (0x01 << 6)
    #define MIPI_BP_EN_VLD      (0x01 << 7)
    #define PBL_LOG_BUF_STOP    (0x01 << 4)
    #define DDR_TRAIN_DATA_DONE (0x01 << 3)
    #define PLAT_TYPE_BIT_H     (0x01 << 2)
    #define PLAT_TYPE_BIT_L     (0x01 << 1)
    #define RTOS_LOAD_POS       (0x01 << 0)
#define SDU_AP_CTRL_REG8       (SDU_CTL_BASE + 0x08) // [RW][71:64  ]
#define SDU_AP_CTRL_REG9       (SDU_CTL_BASE + 0x09) // [RW][79:72  ]
#define SDU_AP_CTRL_REGA       (SDU_CTL_BASE + 0x0A) // [RW][87:80  ]
#define SDU_AP_CTRL_REGB       (SDU_CTL_BASE + 0x0B) // [RW][95:88  ]
#define SDU_AP_CTRL_REGC       (SDU_CTL_BASE + 0x0C) // [RW][103:96 ]
#define SDU_AP_CTRL_REGD       (SDU_CTL_BASE + 0x0D) // [RW][111:104]
#define SDU_UNIQ_CTRL_REG0     (SDU_CTL_BASE + 0x0E) // [RW][119:112]
#define SDU_UNIQ_CTRL_REG1     (SDU_CTL_BASE + 0x0F) // [RW][127:120]

//SDU Read-only Regs
#define SDU_BOOT_STAT_REG0     (SDU_CTL_BASE + 0x10) // [RO][7:0    ]
#define SDU_BOOT_STAT_MIPI_BP_OK       (0x01 << 0)
#define SDU_BOOT_STAT_MPU_GUARD        (0x01 << 1)
#define SDU_BOOT_STAT_REFCLK_384       ((0x00 & 0x03) << 2)
#define SDU_BOOT_STAT_REFCLK_260       ((0x01 & 0x03) << 2)
#define SDU_BOOT_STAT_REFCLK_192       ((0x20 & 0x03) << 2)
#define SDU_BOOT_STAT_ROM_BOOT         (0x01 << 7)

#define SDU_BOOT_STAT_REG1     (SDU_CTL_BASE + 0x11) // [RO][15:8   ]
#define SDU_BOOT_STAT_ABNORMAL_BOOT    (0x01 << 7)
#define SDU_BOOT_STAT_UART_DONE        (0x01 << 6)
#define SDU_BOOT_STAT_UART_SUCCESS     (0x01 << 5)
#define SDU_BOOT_STAT_SE_CRC_DONE      (0x01 << 2)
#define SDU_BOOT_STAT_SE_CRC_SUCCESS   (0x01 << 1)
#define SDU_BOOT_STAT_SECURE_BOOT      (0x01 << 0)

#define SDU_BOOT_STAT_REG2     (SDU_CTL_BASE + 0x12) // [RO][23:16  ]
#define SDU_BOOT_STAT_VERIFY_PASS_NPU  (0x01 << 0)
#define SDU_BOOT_STAT_VERIFY_PASS_RTOS (0x01 << 1)
#define SDU_BOOT_STAT_DDR_TRAINING_DONE (0x01 << 2)
#define SDU_BOOT_STAT_VERIFY_PASS_SDI  (0x01 << 3)
#define SDU_BOOT_STAT_CRC_CHK_PASS_SDI (0x01 << 4)
#define SDU_BOOT_STAT_VERIFY_PASS_PBL  (0x01 << 5)
#define SDU_BOOT_STAT_SE_INIT_SUCCESS  (0x01 << 6)
#define SDU_BOOT_STAT_SE_INIT_DONE     (0x01 << 7)

#define SDU_BUSMON_STAT_REG0   (SDU_CTL_BASE + 0x13) // [RO][31:24  ]
#define SDU_BUSMON_STAT_REG1   (SDU_CTL_BASE + 0x14) // [RO][39:32  ]
#define SDU_AHB_STAT_REG0      (SDU_CTL_BASE + 0x15) // [RO][47:40  ]
#define SDU_INT_STAT_REG0      (SDU_CTL_BASE + 0x16) // [RO][55:48  ]
#define SDU_INT_STAT_REG1      (SDU_CTL_BASE + 0x17) // [RO][63:56  ]
#define SDU_INT_RESET_DONE             (0x01 << 3)

#define SDU_BOOT_STAT_REG8     (SDU_CTL_BASE + 0x18) // [RO][71:64  ]
#define SDU_BOOT_STAT_PLL_DONE         (0x01 << 2)
#define SDU_BOOT_STAT_PLL_SPD          (0x03 << 0)
#define SDU_BOOT_STAT_PLL_LS           (0x00)
#define SDU_BOOT_STAT_PLL_MS           (0x01)
#define SDU_BOOT_STAT_PLL_HS           (0x02)

#define SDU_BOOT_STAT_REG9     (SDU_CTL_BASE + 0x19) // [RO][79:72  ]
#ifdef SLT_ENABLE
#define SDU_BOOT_STAT_VERIFY_PASS_SLT  (0x01 << 6)
#endif

#define SDU_AP_STAT_REGA       (SDU_CTL_BASE + 0x1a) // [RO][87:80  ]
#define SDU_AP_STAT_REGB       (SDU_CTL_BASE + 0x1b) // [RO][95:88  ]
#define SDU_AP_STAT_REGC       (SDU_CTL_BASE + 0x1c) // [RO][103:96 ]
#define SDU_STAT_SE_INIT_READY         (0x01 << 0)
#define SDU_AP_STAT_REGD       (SDU_CTL_BASE + 0x1d) // [RO][111:104]
#define ADU_AP_STA_REMEDY_NEED_RESEND  (0x01 << 6)
#define SDU_UNIQ_STAT_REG0     (SDU_CTL_BASE + 0x1e) // [RO][119:112]
#define SDU_UNIQ_STAT_REG1     (SDU_CTL_BASE + 0x1F) // [RO][127:120]

//CSPI Read-only Regs
#define CSPI_BOOT_STAT_REG0    (CSPI_CTL_BASE + 0x00) // [RO][7:0    ]
#define CSPI_BOOT_STAT_REG1    (CSPI_CTL_BASE + 0x01) // [RO][15:8   ]
#define CSPI_BOOT_STAT_REG2    (CSPI_CTL_BASE + 0x02) // [RO][23:16  ]
#define CSPI_BUSMON_STAT_REG0  (CSPI_CTL_BASE + 0x03) // [RO][31:24  ]
#define CSPI_BUSMON_STAT_REG1  (CSPI_CTL_BASE + 0x04) // [RO][39:32  ]
#define CSPI_AHB_STAT_REG0     (CSPI_CTL_BASE + 0x05) // [RO][47:40  ]
#define CSPI_INT_STAT_REG0     (CSPI_CTL_BASE + 0x06) // [RO][55:48  ]
#define CSPI_INT_STAT_REG1     (CSPI_CTL_BASE + 0x07) // [RO][63:56  ]
#define CSPI_PWR_STAT_REG0     (CSPI_CTL_BASE + 0x08) // [RO][71:64  ]
#define CSPI_BOOT_STEP_CC      (CSPI_CTL_BASE + 0x09) // [RO][79:72  ]
#define CSPI_AP_STAT_REGA      (CSPI_CTL_BASE + 0x0a) // [RO][87:80  ]
#define CSPI_AP_STAT_REGB      (CSPI_CTL_BASE + 0x0b) // [RO][95:88  ]
#define CSPI_AP_STAT_REGC      (CSPI_CTL_BASE + 0x0c) // [RO][103:96 ]
#define CSPI_AP_STAT_REGD      (CSPI_CTL_BASE + 0x0d) // [RO][111:104]
#define CSPI_UNIQ_STAT_REG0    (CSPI_CTL_BASE + 0x0e) // [RO][119:112]
#define CSPI_UNIQ_STAT_REG1    (CSPI_CTL_BASE + 0x0f) // [RO][127:120]

//CSPI Control Regs
#define CSPI_BOOT_CTRL_REG0    (CSPI_CTL_BASE + 0x10) // [RW][7:0    ]
#define CSPI_AHB_CTRL_REG0     (CSPI_CTL_BASE + 0x11) // [RW][15:8   ]
#define CSPI_AHB_CTRL_REG1     (CSPI_CTL_BASE + 0x12) // [RW][23:16  ]
#define CSPI_PWR_CTRL_REG0     (CSPI_CTL_BASE + 0x13) // [RW][31:24  ]
#define CSPI_RST_CTRL_REG0     (CSPI_CTL_BASE + 0x14) // [RW][39:32  ]
#define CSPI_RST_CTRL_REG1     (CSPI_CTL_BASE + 0x15) // [RW][47:40  ]
#define CSPI_AP_CTRL_REG6      (CSPI_CTL_BASE + 0x16) // [RW][55:48  ]
#define CSPI_AP_CTRL_REG7      (CSPI_CTL_BASE + 0x17) // [RW][63:56  ]
#define CSPI_AP_CTRL_REG8      (CSPI_CTL_BASE + 0x18) // [RW][71:64  ]
#define CSPI_AP_CTRL_REG9      (CSPI_CTL_BASE + 0x19) // [RW][79:72  ]
#define CSPI_AP_CTRL_REGA      (CSPI_CTL_BASE + 0x1a) // [RW][87:80  ]
#define CSPI_AP_CTRL_REGB      (CSPI_CTL_BASE + 0x1b) // [RW][95:88  ]
#define CSPI_AP_CTRL_REGC      (CSPI_CTL_BASE + 0x1c) // [RW][103:96 ]
#define CSPI_AP_CTRL_REGD      (CSPI_CTL_BASE + 0x1d) // [RW][111:104]
#define CSPI_UNIQ_CTRL_REG0    (CSPI_CTL_BASE + 0x1e) // [RW][119:112]
#define CSPI_UNIQ_CTRL_REG1    (CSPI_CTL_BASE + 0x1f) // [RW][127:120]

//Global Regs
#define GLB_AP_CTRL_BOOT_STEP	0x40020468
#define BOOT_TRANS_BIT_BOOTROM (0x01 << (0+16))
#define BOOT_TRANS_BIT_REMEDY  (0x01 << (1+16))
#define BOOT_TRANS_BIT_SE_FW   (0x01 << (2+16))
#define BOOT_TRANS_BIT_SDI_FW  (0x01 << (3+16))
#define BOOT_TRANS_BIT_PBL_FW  (0x01 << (4+16))
#define BOOT_TRANS_BIT_DDR_FW  (0x01 << (5+16))
#define BOOT_TRANS_BIT_OS_IMG  (0x01 << (6+16))
#define BOOT_TRANS_BIT_NPU_IMG (0x01 << (7+16))
#define BOOT_TRANS_BIT_ISP_IMG (0x01 << (8+16))
#define BOOT_TRANS_BIT_SE_HEAD_FW (0x01 << (25))
#define BOOT_STEP_BOOTROM_DONE  ((0x01 & 0x3F) << 26)
#define BOOT_STEP_REMEDY_DONE   ((0x02 & 0x3F) << 26)
#define BOOT_STEP_SE_DONE       ((0x04 & 0x3F) << 26)
#define BOOT_STEP_3_FW_DONE     ((0x08 & 0x3F) << 26)
#define BOOT_STEP_OS_DONE       ((0x10 & 0x3F) << 26)
#define BOOT_STEP_NPU_DONE      ((0x20 & 0x3F) << 26)

#ifdef SLT_ENABLE
#define BOOT_TRANS_BIT_2ND_PBL (0x01 << (16-1))
#endif

#define GLB_CPU_RELEASE_RESET		0x4002046C
#define CPU_RELEASE_RESET	(0x01)

//INTC reg
#define CC_INTC0_BASE   0x48018000
#define CC_INTC1_BASE   0x48019000
#define CC_INTC2_BASE   0x4801A000
#define CC_INTC3_BASE   0x4801B000
#define CC_INTC0_L_EN   (CC_INTC0_BASE  + 0x00)
#define CC_INTC0_L_MASK (CC_INTC0_BASE +  0x08)
#define CC_INTC1_L_EN   (CC_INTC1_BASE  + 0x00)
#define CC_INTC1_L_MASK (CC_INTC1_BASE +  0x08)
#define CC_INTC2_L_EN   (CC_INTC2_BASE  + 0x00)
#define CC_INTC2_L_MASK (CC_INTC2_BASE +  0x08)
#define CC_INTC3_L_EN   (CC_INTC3_BASE  + 0x00)
#define CC_INTC3_L_MASK (CC_INTC3_BASE +  0x08)
#define CC_INTC_RESET_EN   (0x01 << 0)
#define CC_INTC_RESET_MASK (0x01 << 0)

/* define ret values */
#define R_ABN_RBT_OK		1
#define R_BAD_ARG		1001
#define R_BAD_TSTG		1002
#define R_BAD_DMOD		1003
#define R_BAD_MMOD		1004
#define R_BAD_RMOD		1005
#define R_BAD_RPOS		1006
#define R_BAD_PMOD		1007
#define R_FW_REQ_FAIL		1008
#define R_REG_VAL_CHG		1009
#define R_SE_INIT_FAIL		1010
#define R_PLL_ENA_FAIL		1011
#define R_SE_INIT_TO		1012
#define R_PBL_VER_TO		1013
#define R_MIPI_BP_TO		1014
#define R_DDR_INIT_TO		1015
#define R_OS_VER_TO		1016
#define R_NPU_VER_TO		1017
#ifdef SLT_ENABLE
#define R_SLT_VER_TO		1018
#endif
#define R_BAD_CMOD		1019
#define R_REPEATED_BOOT		1020
#define R_PMIC_OC_FAIL		1021
#define R_DDR_INIT_FAIL		1022

#define FW_LOAD_RETRY		2

/* firmware name */
#define BOOTROM_NAME		"explorer/bootrom.bin"
#define REMEDY_NAME		"explorer/remedy.bin"
#define SE_HEAD_NAME		"explorer/se_header.bin"
#define SE_DATA_NAME		"explorer/se_fw.bin"
#define SDI_SEC_NAME		"explorer/sec_sdi.bin"
#define SDI_NAME		"explorer/sdi.bin"
#define PBL_SEC_NAME		"explorer/sec_pbl.bin"
#define PBL_NAME		"explorer/pbl.bin"
#define PROV_NAME		"explorer/pbl_prov.bin"
#define DDR_1ST_NAME		"explorer/ddr_first.bin"
#define DDR_QCK_NAME		"explorer/ddr_quick.bin"
#define DDR_QCK_DATA_NAME	"explorer/ddr_data.bin"
#define OS_SEC_NAME		"explorer/sec_rtthread.bin"
#define OS_NAME			"explorer/rtthread.bin"
#define OS_AON_SEC_NAME		"explorer/sec_rtthread_aon.bin"
#define OS_AON_NAME		"explorer/rtthread_aon.bin"
#define OS_PLAT_SEC_NAME	"explorer/sec_rtthread_plat.bin"
#define OS_PLAT_NAME		"explorer/rtthread_plat.bin"
#define OS_RLS_SEC_NAME		"explorer/sec_rtthread_release.bin"
#define OS_RLS_NAME		"explorer/rtthread_release.bin"
#define OS_DBG_NOPM_SEC_NAME	"explorer/sec_rtthread_nopm.bin"
#define OS_DBG_NOPM_NAME	"explorer/rtthread_nopm.bin"
#define OS_RLS_NOPM_SEC_NAME	"explorer/sec_rtthread_release_nopm.bin"
#define OS_RLS_NOPM_NAME	"explorer/rtthread_release_nopm.bin"

#define NPU_NAME		"explorer/npu_model/nmod.bin"
#define NPU_SEC_NAME		"explorer/npu_model/sec_nmod.lz4"

#define NPU_BZ_BK_NAME		"explorer/npu_model/bz/bk.bin"
#define NPU_BZ_FR_NAME		"explorer/npu_model/bz/fr.bin"
#define NPU_BZ_BK_BS_NAME	"explorer/npu_model/bz/bk_bs.bin"
#define NPU_BZ_FR_BS_NAME	"explorer/npu_model/bz/fr_bs.bin"

#define NPU_BZ_BK_SEC_NAME	"explorer/npu_model/bz/sec_bk.lz4"
#define NPU_BZ_FR_SEC_NAME	"explorer/npu_model/bz/sec_fr.lz4"
#define NPU_BZ_BK_BS_SEC_NAME	"explorer/npu_model/bz/sec_bk_bs.lz4"
#define NPU_BZ_FR_BS_SEC_NAME	"explorer/npu_model/bz/sec_fr_bs.lz4"

#define NPU_LW_BK_NAME		"explorer/npu_model/lw/bk.bin"
#define NPU_LW_FR_NAME		"explorer/npu_model/lw/fr.bin"
#define NPU_LW_BK_BS_NAME	"explorer/npu_model/lw/bk_bs.bin"
#define NPU_LW_FR_BS_NAME	"explorer/npu_model/lw/fr_bs.bin"

#define NPU_LW_BK_SEC_NAME	"explorer/npu_model/lw/sec_bk.lz4"
#define NPU_LW_FR_SEC_NAME	"explorer/npu_model/lw/sec_fr.lz4"
#define NPU_LW_BK_BS_SEC_NAME	"explorer/npu_model/lw/sec_bk_bs.lz4"
#define NPU_LW_FR_BS_SEC_NAME	"explorer/npu_model/lw/sec_fr_bs.lz4"

#define NPU_AON_NAME		"explorer/npu_model/aon/aon_combine_lz4.bin"
#define NPU_AON_SEC_NAME	"explorer/npu_model/aon/aon_combine_lz4.bin"

/* firmware address */
#define FW_BOOTROM_ADDR		0x00000000
#define FW_REMEDY_ADDR		0x00108000
#define FW_SE_HEAD_ADDR		0x00149800
#define FW_SE_DATA_ADDR		0x58104000
#define FW_SDI_ADDR		0x00100400
#define FW_PBL_ADDR		0x00178000
#define FW_DDR_ADDR		0x00109800
#define FW_DDR_DATA_ADDR	0x0013D800
#define FW_OS_ADDR		0x00108000
#define FW_OS_DDR_ADDR		0x2FE00000

#define FW_NPU_ADDR		0x20000000
#define FW_NPU_SEC_ADDR		0x20E00000
#define FW_NPU_AON_ADDR		0x20900000
#define FW_NPU_ADDR1		0x20300000
#define FW_NPU_ADDR2		0x203A0000
#define FW_NPU_ADDR3		0x20400000
#define FW_NPU_ADDR4		0x205B0000
#define FW_NPU_ADDR5		0x205C0000

#define SLT_PBL_LOG_ADDR	0x00119C00
#define SLT_PBL_LOG_LEN		0x14400 /* 12E000 - 119C00 */
#define NORMAL_PBL_LOG_ADDR	0x00107800
#define NORMAL_PBL_LOG_LEN	(FW_OS_ADDR - NORMAL_PBL_LOG_ADDR)  //0x800

#define DDR_START_ADDR      0x20000000
#define DDR_MAX_END_ADDR    0x2FFFFFFF

#define FW_WRITE_BUF_SIZE (512*511)

#define AP_BOOT_ERR(format, ...) \
	pr_err("ap_boot[%s][%d]" format, __FILE__, __LINE__, ##__VA_ARGS__)
#define AP_BOOT_INFO(format, ...) \
	pr_info("ap_boot[%s][%d]" format, __FILE__, __LINE__, ##__VA_ARGS__)
#define AP_BOOT_DBG(format, ...)  \
	pr_debug("ap_boot[%s][%d]" format, __FILE__, __LINE__, ##__VA_ARGS__)

int explorer_proc_bootrom_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data);
int explorer_proc_pbl_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data);

enum wait_stage {
    WAIT_SE_OK = 0,
    WAIT_PBL_OK,
#ifdef SLT_ENABLE
    WAIT_2ND_PBL_OK,
#endif
    WAIT_MIPI_OK,
    WAIT_DDR_OK,
    WAIT_OS_OK,
    WAIT_NPU_OK,
};

extern char *training_data_buf;
extern unsigned int training_data_size;

int explorer_set_glb_reg_bits(struct explorer_plat_data *epd, u32 addr, u32 val);
int explorer_clr_glb_reg_bits(struct explorer_plat_data *epd, u32 addr, u32 val);

extern int explorer_load_buffer(struct explorer_plat_data *epd, u32 addr);
extern int explorer_load_fw(struct explorer_plat_data *epd,
                           const char *firmware_name, u32 addr);
extern int explorer_boot(struct explorer_plat_data *epd);
int explorer_sdio_tuning(struct explorer_plat_data *epd);


#endif /* _AP_BOOT_H_ */
