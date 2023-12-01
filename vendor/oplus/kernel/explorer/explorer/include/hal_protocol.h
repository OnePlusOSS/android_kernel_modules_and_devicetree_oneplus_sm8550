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
 * Basecode Created :        2020/10/11 Author: zf@zeku.com
 *
 */

#ifndef _HAL_PROTOCOL_H
#define _HAL_PROTOCOL_H

/*
 * abstraction hardware specification
 */

/* CMD and DATA field length in byte */

#define	MBOX_CMD_LEN			(8)
#define	MBOX_DATA_LEN			(8)
#define MBOX_ITEM_LEN			(MBOX_CMD_LEN+MBOX_DATA_LEN)

/* hardware channel id */
typedef enum {
	HAL_CH_0 = 0,
	HAL_CH_1,
	HAL_CH_2,
	HAL_CH_3,
} eHAL_CH;

/* hardware independent communication protocol each field length*/

#define		CMD_IS_SYNC			(1)
#define		CMD_IS_REPLY			(1)
#define		CMD_IS_END			(1)
#define		CMD_CORE_ID			(3)
#define		CMD_PLD_POS			(2)
#define		CMD_MOD_ID			(8)
#define		CMD_SUB_ID			(8)
#define		CMD_SYNC_INDEX			(8)
#define		CMD_ASYNC_INDEX			(8)

#define		CMD_RSVD			(8)
#define		CMD_LEN_OR_RET			(24)

#define		COMM_BUF_LEN			(MBOX_CMD_LEN+MBOX_DATA_LEN)
#define		COMM_BUF_NUM			(COMM_BUF_LEN/sizeof(u32))

#define		CMD_ID_MASK			(0x1f)
#define		CMD_LEN_MASK			(0x00ffffff)
#define		CMD_INDEX_MASK			(0xff)

/* hardware independent communication protocol each field value */

#define		HAL_CMD_ASYNC			(0x0)
#define		HAL_CMD_SYNC			(0x1)

#define		HAL_CMD_SEND			(0x0)
#define		HAL_CMD_REPLY			(0x1)

#define		HAL_CMD_START			(0x0)
#define		HAL_CMD_END			(0x1)

#define		HAL_CMD_DATA_LEN_4		(0x4)
#define		HAL_CMD_DATA_LEN_8		(0x8)
#define		HAL_CMD_DUMMY_DATA		(0xAABBCCDD)
#define		HAL_CMD_DUMMY_INDEX		(0x0F)

#define		RETRY_NUM				(0x3)
#define		COMM_WAIT_TIME			(2) /* second */

/* HAL communication protocol cmd */

#define HAL_CMD_DEFINE(chan, id)  \
	(((chan) & 0xFF) << CMD_MOD_ID | ((id) & 0xFF))

/* HAL cmd: malloc operation */

#define HAL_CMD_MALLOC		\
	HAL_CMD_DEFINE(HAL_CH_0, 0x00)

/* HAL cmd: free operation */

#define HAL_CMD_FREE		\
	HAL_CMD_DEFINE(HAL_CH_0, 0x01)

/* HAL cmd: bootrom command */

#define HAL_CMD_BOOTROM		\
	HAL_CMD_DEFINE(HAL_CH_0, 0x02)

/* HAL cmd: pbl command */

#define HAL_CMD_PBL		\
	HAL_CMD_DEFINE(HAL_CH_0, 0x03)

/* HAL cmd: generic ipc data */

#define HAL_CMD_GEN		\
	HAL_CMD_DEFINE(HAL_CH_0, 0x04)

/* HAL cmd: read data in allocated address */

#define HAL_CMD_RDIAA		\
	HAL_CMD_DEFINE(HAL_CH_0, 0x05)

/* HAL cmd:  watchdog timeout */

#define HAL_CMD_WDT_TIMEOUT    \
	HAL_CMD_DEFINE(HAL_CH_0, 0x06)

/* HAL cmd: read data end */

#define HAL_CMD_RDE		\
	HAL_CMD_DEFINE(HAL_CH_0, 0x07)

/* HAL cmd: upload buffer */

#define HAL_CMD_UPLOAD_BUFFER       \
	HAL_CMD_DEFINE(HAL_CH_0, 0x08)

/* HAL cmd: report TP EVENT */
#define HAL_CMD_TPEVENT       \
	HAL_CMD_DEFINE(HAL_CH_0, 0x09)

/* HAL cmd: boot completed */
#define HAL_CMD_BOOT_COMPLETED       \
	HAL_CMD_DEFINE(HAL_CH_0, 0x0C)

/* HAL cmd:  runtime work executed */

#define HAL_CMD_SDIO_DETECTED        \
	HAL_CMD_DEFINE(HAL_CH_0, 0x0D)

/* HAL cmd: ap scratch log from cop */

#define HAL_CMD_SCRATCH_LOG   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x0F)

/* HAL cmd: send log buffer addr to ap */

#define HAL_CMD_SEND_LOG_ADDR   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x10)

/* HAL cmd: security cmd */

#define HAL_CMD_SEC    \
	HAL_CMD_DEFINE(HAL_CH_0, 0x11)

/* HAL cmd: isp cmd & data */

#define HAL_CMD_ISP   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x12)

/* HAL cmd: isp large data */

#define HAL_CMD_ISP_LDATA   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x13)

/* HAL cmd: isp cmd & data */

#define HAL_CMD_CAMERA   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x14)

#ifdef SLT_ENABLE
/* HAL cmd: slt */
#define HAL_CMD_SLT      \
	HAL_CMD_DEFINE(HAL_CH_0, 0x15)
#endif

/* HAL cmd: slt */
#define HAL_CMD_DDR_DATA      \
	HAL_CMD_DEFINE(HAL_CH_0, 0x16)

/* HAL non ipc mem cmd base */

#define HAL_CMD_NON_IPCMEM   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x1A)

/* HAL cmd: ram dump */

#define HAL_CMD_RAM_DUMP   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x1A)

/* HAL cmd: change hclk */

#define HAL_CMD_HCLK       \
	HAL_CMD_DEFINE(HAL_CH_0, 0x1B)

/* HAL cmd: change sdclk */
#define HAL_CMD_SDCLK      \
	HAL_CMD_DEFINE(HAL_CH_0, 0x1C)

/* HAL cmd: suspend/resume cop */

#define HAL_CMD_SUSPEND_RESUME   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x1E)

/* HAL cmd: gdb */
#define HAL_CMD_GDB   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x1F)

/* HAL cmd: exception */
#define HAL_CMD_EXCEPTION   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x20)
/* HAL cmd: ulog ctl*/
#define HAL_CMD_ULOG_CTL   \
	HAL_CMD_DEFINE(HAL_CH_0, 0x21)

/* HAL cmd: NPU-related ipc data */

#define HAL_CMD_NPU		\
	HAL_CMD_DEFINE(HAL_CH_0, 0x22)


#define HAL_CMD_SOFT_RESET		\
	HAL_CMD_DEFINE(HAL_CH_0, 0x23)

/* HAL cmd: max cmd id */
#define HAL_CMD_MAX	       \
	HAL_CMD_DEFINE(HAL_CH_0, 0xff)

#define EXT_CMD_ID(cmd_id)	\
	(((cmd_id) & 0xff))

#define CON_IPC_ID(mod_id, sub_id)	\
	(((sub_id) << 16) | (mod_id))

#define EXT_SUB_ID(cmd_id)	\
	((((cmd_id) >> 16) & 0xffff))

#define EXT_MOD_ID(cmd_id)	\
	(((cmd_id) & 0xffff))

struct hal_comm_data {
	unsigned int cmd_async_idx :
		CMD_ASYNC_INDEX;
	unsigned int cmd_sync_idx :
		CMD_SYNC_INDEX;
	unsigned int cmd_mod_id :
		CMD_MOD_ID;
	unsigned int cmd_core_id :
		CMD_CORE_ID;
	unsigned int cmd_pld_pos :
		CMD_PLD_POS;
	unsigned int cmd_is_end :
		CMD_IS_END;
	unsigned int cmd_is_reply :
		CMD_IS_REPLY;
	unsigned int cmd_is_sync :
		CMD_IS_SYNC;
	unsigned int data_len :
		CMD_LEN_OR_RET;
	unsigned int cmd_sub_id :
		CMD_SUB_ID;
	unsigned int data[2];
};

/* boot cmd data */
#define ID_SE_RLD_SET		0xFU
#define ID_SE_RLD_CLR		0x1U
#define ID_SDI_RLD_SET		0x2U
#define ID_SDI_RLD_CLR		0x3U
#define ID_PBL_RLD_SET		0x4U
#define ID_PBL_RLD_CLR		0x5U

#define ID_INFORM_CBOOT		0x10U
#define ID_INFORM_HBOOT		0x11U
#define ID_INFORM_AP_RST	0xBADU

/**
  * PBL sub command id
  */
#define	PBL_SUB_CMD_DFT		(0)	/* default */
#define	PBL_SUB_CMD_TD		(2)	/* DDR tranining data */
#ifdef SLT_ENABLE
#define	PBL_SUB_CMD_SLT		(3)
#endif

#define	PBL_SUB_CMD_MB		(4)	/* mipi bypass */

#ifdef SLT_ENABLE
#define ID_2ND_RLD_SET		0x4U
#define ID_2ND_RLD_CLR		0x5U
#endif
#define ID_DDRFW_RLD_SET	0x6U
#define ID_DDRFW_RLD_CLR	0x7U
#define ID_OS_RLD_SET		0x8U
#define ID_OS_RLD_CLR		0x9U
#define ID_NPU_RLD_SET		0xAU
#define ID_NPU_RLD_CLR		0xBU
#define ID_ISP_RLD_SET		0xCU
#define ID_ISP_RLD_CLR		0xDU
#define ID_SLT_BEGIN		0x10U
#define ID_PROV_DONE		0x11U
#define ID_PBL_BEGIN		0x12U
#define ID_MIPI_OK		0x13U
#define ID_DDR_OK		0x14U
#define ID_OS_OK		0x15U
#define ID_NPU_OK		0x16U
#define ID_SLT_RLD_CE	0x17U
#define ID_DDR_FAIL		0x18U
#define ID_PMIC_HW_OK		0x100U
#define ID_PMIC_HW_ERR		0x101U

#define	PBL_SUB_CMD_MAX		(32)

/**
  * NPU sub command id
  */
#define	NPU_SUB_CMD_TRANS_DONE	(0)	/* default */

#endif
