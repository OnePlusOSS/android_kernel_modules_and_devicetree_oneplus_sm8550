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
 * Basecode Created :        2021/03/17 Author: wangqinyuan@zeku.com
 *
 */

#ifndef _EXPLORER_SLT_H
#define _EXPLORER_SLT_H

#define	SLT_CMD_LEN_MAX		(512)

#define	SLT_CASE_NAME_LEN_MIN		(3)
#define	SLT_CASE_NAME_LEN_MAX		(32)

#define SLT_OPTION_LEN_MAX		(SLT_CMD_LEN_MAX - SLT_CASE_NAME_LEN_MIN)

#define SLT_TOKEN_LEN_MIN		(1)
#define	SLT_TOKEN_NUM_MAX		(SLT_OPTION_LEN_MAX/SLT_TOKEN_LEN_MIN)
#define SLT_TOKEN_LEN_MAX		(32)
#define	SLT_TOKEN_NUM_MIN		(SLT_OPTION_LEN_MAX/SLT_TOKEN_LEN_MAX)

#define	SLT_RESP_MSG_LEN_MAX		(1024)
#define	SLT_LOCAL_CASE_NUM		(8)

enum slt_case_type {
	CASE_IN_AP = 1,
	CASE_IN_PBL,
	CASE_IN_RTOS,
};

/* slt buffer address */
#define	SLT_BUFFER_BASE			(0x170000U)

struct slt_token {
	char name[SLT_TOKEN_LEN_MAX];
	char value[SLT_TOKEN_LEN_MAX];
};

/**
 * SLT case structure
 */
struct slt_cast_node {
	char case_name[SLT_CASE_NAME_LEN_MAX];
	int (*run_case)(struct explorer_plat_data *epd, char *option);
};

struct slt_resp_msg {
	unsigned int checksum;
	unsigned short len;
	unsigned short reserve;
	unsigned short testcase_id;
	unsigned char test_result;
	unsigned char result_type;
	unsigned short msg_offset;
	unsigned short data_offset;
};

/**
 * SLT userland message.
 */
struct explorer_slt_msg_header {
	unsigned int len;
	unsigned int type;
};

/**
 * SLT FW definition, should match with RttBootSltMode
*/
enum slt_fw_type {
	SLT_NO_FW_ONLY_RESET,
	SLT_FW_PBL = 1,
	SLT_FW_OS_QUICK,
	SLT_FW_OS_AON_QUICK,
	SLT_FW_OS_DDR_QUICK,
	SLT_FW_OS_AON_DDR_QUICK,
	SLT_FW_OS,
	SLT_FW_OS_AON,
	SLT_FW_OS_DDR,
	SLT_FW_OS_AON_DDR,
	SLT_BOOT_STAGE_TYPE = SLT_FW_OS_AON_DDR,
	SLT_FW_DDR_ATE,
	SLT_FW_DDR_DIAG_L,
	SLT_FW_DDR_DIAG_M,
	SLT_FW_DDR_DIAG_H,
	SLT_FW_DDR_DIAG_C,
	SLT_FW_DDR_FIRST_L,
	SLT_FW_DDR_FIRST_M,
	SLT_FW_DDR_FIRST_H,
	SLT_FW_DDR_FIRST_C,    //4266
	SLT_FW_DDR_FIRST_FULL, //800+1600+3200+4266
	SLT_FW_DDR_PATTERN,
	SLT_LAST_PBL_STAGE_FM = SLT_FW_DDR_PATTERN, //last PBL firmware type
	SLT_FW_ISP_BAYER_12B,
	SLT_FW_ISP_BAYER_14B,
	SLT_FW_ISP_NCFA,
	SLT_FW_ISP_RGBW,
	SLT_FW_ISP_RGBW_FULL,
	SLT_FW_ISP_RSGN,
	SLT_FW_ISP_AON,
	SLT_FW_ISP_ALL,
	SLT_FW_NPU_UD,
	SLT_FW_NPU_SN,
	SLT_FW_NPU_AON,
	SLT_FW_NO_NPU_NM,
	SLT_LAST_OS_STAGE_FM = SLT_FW_NO_NPU_NM, //last OS firmware type
	MAX_SLT_FW_TYPE,
};

struct explorer_slt_fw {
	enum slt_fw_type type;
};

void explorer_slt_localcase_init(int index, const char *name, int (*run_test_case)(struct explorer_plat_data *epd, char *option));
int explorer_proc_slt_msg(struct explorer_plat_data *epd, void *ap_buffer, struct hal_comm_data *comm_data);
int explorer_proc_pbl_slt_msg(struct explorer_plat_data *epd, struct hal_comm_data *comm_data);
int explorer_slt_dispatch(struct explorer_plat_data *epd, char *slt_command,
			    enum slt_case_type case_type, size_t count);
int explorer_slt_parse_option(char *slt_option, char slt_tokens[][SLT_TOKEN_LEN_MAX]);
int explorer_slt_parse_token(char *slt_token, struct slt_token *param);
void explorer_sdio_slt_test_case_init(void);

/* load firmware function definition */
int wait_firmware_on(struct explorer_plat_data *epd, struct explorer_slt_fw *target_fw_type);

#endif /* _EXPLORER_SLT_H */
