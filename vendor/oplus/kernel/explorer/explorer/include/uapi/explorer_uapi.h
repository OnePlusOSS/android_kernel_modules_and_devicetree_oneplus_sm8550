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
 * Basecode Created :        2020/01/20 Author: zhangao@zeku.com
 *
 */

#ifndef _EXPLORER_UAPI_H
#define _EXPLORER_UAPI_H

#if defined (QCOM_AON) || defined (MTK_AON)
#include "../../../aon/include/aon_uapi.h"
#endif

struct zeku_msg_hdr {
	unsigned short msg_id;
	unsigned short data_size;
};

struct zeku_msg_size_max {
	struct zeku_msg_hdr hdr;
	unsigned char data[512];
};

struct explorer_sync_reply {
	unsigned int size;
	void *buffer;
};

struct explorer_sync_msg {
	struct explorer_sync_reply reply;
	unsigned int msg_id;
	unsigned char data[512];
};

/**
 * Generic header of message from AP to Explorer.
 * If addr_alloc = IPC_DST_ADDR_MANUAL, dst_addr is address specified by usrland,
 * otherwise, it is ignored.
 */

/* core id list */
#define	IPC_GEN_COREID_AP			(0)
#define	IPC_GEN_COREID_EXPLORER			(1)

/* system id list */
#define	IPC_GEN_SYSID_BOOTROM			(0)
#define	IPC_GEN_SYSID_PBL			(1)
#define	IPC_GEN_SYSID_RTTHREAD			(2)
#define	IPC_GEN_SYSID_ANDROID			(3)

/* module id list */
#define	IPC_GEN_MSGID_BSP			(0)
#define	IPC_GEN_MSGID_ISP			(1)
#define	IPC_GEN_MSGID_TSS			(2)		/* timestamp sync */
#define	IPC_GEN_MSGID_ASO			(3)		/* AP sensorhub time offset */
#define	IPC_GEN_MSGID_THERMAL			(4)
#define	IPC_GEN_MSGID_TM1			(17)
#define	IPC_GEN_MSGID_TM2			(18)
#define	IPC_GEN_MSGID_PRJID			(19)

#define	IPC_GEN_MSGID_TEST			(0x5A)

/* xfer mode */
#define	IPC_SYNC_SEND_MODE			(0)	/* sync send */
#define	IPC_SYNC_REPLY_MODE			(1)	/* sync reply */
#define	IPC_ASYNC_SEND_MODE			(2)	/* async send */
#define	IPC_ASYNC_REPLY_MODE			(3)	/* async reply */
#define	IPC_GEN_DATA			(0)
#define	IPC_GEN_CMD			(1)
#define	IPC_SET_XFER_MODE(high,low)	(((high)<<16)+((low)&0xffff))
#define IPC_GET_DATA_MODE(mode)		(((mode)>>16)&0xffff)
#define IPC_GET_SYNC_MODE(mode)		((mode)&0xffff)

/* ipc feature address */
#define	IPC_AUTO_ALLOC_ADDR			(0xAABBCCDD)

/* endpoint info */
struct explorer_endpoint {
	unsigned int core_id;		/* core id */
	unsigned int sys_id;		/* system id */
	unsigned int mod_id;		/* module id */
};

/* generic data header */
struct explorer_gen_data_header {
	struct explorer_endpoint src_ep;
	struct explorer_endpoint dst_ep;
	unsigned int mode;		/* xfer mode */
	unsigned int dst_addr;		/* destination address */
	struct explorer_sync_reply reply;		/* reply data from Explorer */
	unsigned int psize;		/* payload size */
	void *payload;		/* point to payload buffer */
};

/* test structure */
struct explorer_gen_msg_test {
	struct explorer_gen_data_header header;
	unsigned char data[512];
};

/* tunning data bin file name max length  */
#define	TD_NAME_LEN_MAX			(128)
struct explorer_tunning_data_info {
	unsigned int addr;
	char name[TD_NAME_LEN_MAX+1];
};

/* npu model file name max length  */
#define	NMOD_NAME_LEN_MAX			(128)
struct explorer_npu_mod_info {
	unsigned int addr;
	unsigned int index;
	char name[NMOD_NAME_LEN_MAX+1];
};

struct explorer_rdmem_info {
	unsigned int id;
};

enum op_mode_t {
	OP_CONFIG_MIPI,
	OP_STOP_MIPI,
	OP_GOTO_RTOS,
	OP_MODE_MAX
};

struct mipi_params_t {
	uint8_t op_cmd;
	uint8_t mipi_packed_params[7];
};

enum boot_force_cfg {
	FORCE_NO = 0,
	FORCE_RLS,
	FORCE_DBG,
	FORCE_PLAT,
	FORCE_DBG_NOPM,
	FORCE_RLS_NOPM,
};

enum boot_stage {
	UNKNOWN = 0,
	BOOTROM,
	PBL_NORMAL,
	PBL_PROV,
	PBL_SLT,
	MIPI_BP_OK,
	DDR_OK,
	OS_OK,
	NPU_OK,
	STG_MAX,
};

enum explorer_ddr_mode {
	DDR_QUICK = 0,
	DDR_FIRST,
	DDR_BUFFER,
	DDR_MAX_MODE,
};

enum explorer_mipi_mode {
	MIPI_NORMAL = 0,
	MIPI_BYPASS,
	MIPI_MAX_MODE,
};

enum explorer_rtos_mode {
	RTOS_NULL = 0,
	RTOS_NORMAL,
	RTOS_AON, /* for aon */
	RTOS_PLAT,
	RTOS_RLS,
	RTOS_DBG_NOPM,
	RTOS_RLS_NOPM,
/* Hidden definition for Android Hal, must be appended at the end */
#ifdef SLT_ENABLE
	RTOS_SLT,
	RTOS_SLT_AON,
#endif
	RTOS_MAX
};

enum explorer_rtos_addr {
	RTOS_SRAM = 0,	/* 108000 */
	RTOS_DDR,	/* 2fe00000 */
	MAX_RTOS_ADDR_TYPE,
};

enum explorer_plat_mode {
	PLAT_QCOM = 0,	/* ref clk = 38.4 Mhz */
	PLAT_MTK,	/* ref clk = 26   MHz */
	MAX_PLAT,
};

enum explorer_cam_mode {
	CAM_NORMAL = 0,
	CAM_BZ_BK = 40,
	CAM_BZ_FR = 41,
	CAM_BZ_BK_BS = 50,
	CAM_BZ_FR_BS = 51,
	CAM_LW_BK = 60,
	CAM_LW_FR = 61,
	CAM_LW_BK_BS = 70,
	CAM_LW_FR_BS = 71,
	CAM_AON = 80,
	CAM_MAX = 256,
};

struct explorer_boot_info {
	enum boot_stage target_stage;
	enum explorer_ddr_mode dmod;
	enum explorer_mipi_mode mmod;
	enum explorer_rtos_mode rmod;
	enum explorer_rtos_addr rpos;
	enum explorer_plat_mode pmod;
	enum explorer_cam_mode rsv;
	unsigned boot_timeout; /* max timeout in msec */
	char buf_mb[8];
};

struct explorer_boot_info_override {
	enum boot_stage target_stage;
	enum explorer_rtos_mode rmod;
	enum explorer_rtos_addr rpos;
	enum explorer_cam_mode rsv;
	unsigned boot_timeout; /* max timeout in msec */
};

/* only used in explorer ap driver */
enum explorer_os_type {
	UNKNOWN_OS = 0,
	NORMAL_OS,
	AON_OS,
	PLAT_OS,
#ifdef SLT_ENABLE
	SLT_OS,
	SLT_OS_AON,
#endif
};

enum explorer_secure_state {
	STAT_UNKNOWN = 0,
	STAT_SECURE,
	STAT_NONSEC,
	STAT_MAX,
};

struct explorer_pbl_info {
	struct explorer_sync_reply reply;
	unsigned short sub_id;
	unsigned short size;
	unsigned int addr;
};

struct explorer_system_status {
	unsigned int ops;	/* 0: read, 1: write */
	void  *value;	/* read: address, write: value */
};

struct explorer_log_reply {
	unsigned int addr;
	unsigned int len;
};

struct explorer_training_data {
	unsigned int len;
	void *addr;
};
#define TRAINING_DATA_BUF_SIZE      (50*1024)
enum explorer_power_info {
	/* Power Actions */
	POWER_IOC_DATA_FIRST = 0x0,
	POWER_IOC_DATA_SUSPEND,
	POWER_IOC_DATA_RESUME,
	POWER_IOC_DATA_SUSPEND_FORCE,
	POWER_IOC_DATA_POWER_ON,
	POWER_IOC_DATA_POWER_OFF,
	POWER_IOC_DATA_LAST,
	/* Power State Set */
	POWER_IOC_STATE_FIRST = 0x10,
	POWER_IOC_STATE_STANDBY,
	POWER_IOC_STATE_BYPASS,
	POWER_IOC_STATE_AON,
	POWER_IOC_STATE_MISSION,
	POWER_IOC_STATE_LAST,
	/* Power State Get */
	POWER_IOC_STATE_GET,
#ifdef SLT_ENABLE
	/* SLT Test */
	POWER_IOC_SDIO_VDD_ON,
	POWER_IOC_SDIO_VDD_OFF,
#endif
};

enum explor_ulog_ctl_info {
	ULOG_DISABLED = 0x0,
	ULOG_ENABLED,
	ULOG_DUMP,
	ULOG_GET_BUF_INFO,
};

enum explorer_tuning {
	TUNING_OK = 0,
	TUNING_SDIO_ERR,
	TUNING_CLOCK_ERR,
	TUNING_BEGIN,
};

struct explorer_sdio_status {
	int sdio_tuning_status;
};

// Define sdi cmd subtype
enum RttSdiCmdType {
	SDI_ENABLE_CTRL     = 0,    /* ram dump enable ctrl */
	SDI_RAM_DUMP        = 1,    /* ram dump info */
	SDI_TRIGGER_DUMP    = 2,    /* trigger ram dump */
	SDI_DUMP_TYPE       = 3,    /* ram dump type */
	SDI_GET_DUMP_INFO   = 4,    /* get dump info, include address and len */
	SDI_CMD_MAX,
};

struct RttSdiMsg {
    enum RttSdiCmdType subtype;
    /* for SDI_ENABLE_CTRL, dataSize: 1, data: bool enable address;
       for SDI_RAM_DUMP, dataSize: sizeof(RttRamDumpMsg), data: RttRamDumpMsg address;
       for SDI_TRIGGER_DUMP, dataSize: 1, data: bool ddrlp2 result
       for SDI_TRIGGER_DUMP, dataSize: 0, data: NULL */
    int datasize;
    void* data;
};

struct RttRamDumpMsg {
    int source_addr;
    int dump_len;
    char* target_addr;
};

#define RAMDUMP_COPY_SIZE 0x400000

/* IOCTL commands nr */
#define	IOC_NR_WR_TEST			(1)
#define	IOC_NR_RD_TEST			(2)
#ifdef SDIO_PRESSURE_TEST
#define	IOC_NR_WRITE_TEST			(3)
#define	IOC_NR_READ_TEST			(4)
#endif
#define	IOC_NR_ISP_NOWAIT			(5)
#define	IOC_NR_CAMERA_NOWAIT			(6)
#define	IOC_NR_SECURITY			(7)
#define IOC_NR_SDCLK			(8)
#define IOC_NR_READDUMP			(9)
#define IOC_BATCH_WRITE_TEST			(10)
#define IOC_NR_BOOT			(11)
#define IOC_NR_POWER		(12)
#define IOC_NR_THERMAL_GET			(13)
#define IOC_NR_SYNC_TIME			(14)
#define IOC_NR_CTL_ULOG		(15)
#define IOC_NR_LOAD_NPU		(16)
#define IOC_NR_PBL_MIPI_BP	(17)
#define IOC_NR_BOOT_CONTINUE	(18)
#define IOC_NR_READ_REPLY			(19)
#define IOC_NR_WRITE_DATA			(20)
#define IOC_NR_GET_SEC_STATE		(21)
#define IOC_NR_SOFT_RESET		(22)
#define IOC_NR_SYS_STATUS		(23)

#define IOC_NR_GDB			(30)
#ifdef SLT_ENABLE
#define IOC_NR_SLT			(32)
#define IOC_NR_SLT_WAIT_FW			(33)
#endif

#define	IOC_NR_ISP_WAIT			(64)
#define	IOC_NR_CAMERA_WAIT			(65)
#define	IOC_NR_PBL			(66)
#define	IOC_NR_PBL_LOG_GET		(67)
#define	IOC_NR_PBL_LOG_CLEAN_FLAG   (68)
#ifndef ZEKU_EXPLORER_PLATFORM_RPI
#define IOC_NR_CAM_CONTROL			(70)
#endif

#define	IOC_NR_SEND_TUNNING_DATA			(128)

/* IOCTL commands */
#define	EXPLORER_IOC_MAGIC			(0xA5)

#define	EXPLORER_IOC_WR_TEST			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_WR_TEST, struct zeku_msg_size_max)
#define	EXPLORER_IOC_RD_TEST			_IOR(EXPLORER_IOC_MAGIC, IOC_NR_RD_TEST, struct zeku_msg_size_max)
#ifdef SDIO_PRESSURE_TEST
#define	EXPLORER_IOC_WRITE_TEST			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_WRITE_TEST, 0)
#define	EXPLORER_IOC_READ_TEST			_IOR(EXPLORER_IOC_MAGIC, IOC_NR_READ_TEST, 0)
#endif
#define	EXPLORER_IOC_ISP_NOWAIT			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_ISP_NOWAIT, struct zeku_msg_size_max)
#define	EXPLORER_IOC_CAMERA_NOWAIT			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_CAMERA_NOWAIT, 0)
#define EXPLORER_IOC_SDCLK			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_SDCLK, int)
#define	EXPLORER_IOC_BATCH_WRITE_TEST			_IOW(EXPLORER_IOC_MAGIC, IOC_BATCH_WRITE_TEST, 0)
#define EXPLORER_IOC_BOOT			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_BOOT, struct explorer_boot_info)
#define EXPLORER_IOC_POWER			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_POWER, enum explorer_power_info)
#define EXPLORER_IOC_THERMAL			_IOR(EXPLORER_IOC_MAGIC, IOC_NR_THERMAL_GET, 0)
#define EXPLORER_IOC_SYNC_TIME			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_SYNC_TIME, 0)
#define EXPLORER_IOC_CTL_ULOG			__IOW(EXPLORER_IOC_MAGIC, IOC_NR_CTL_ULOG, enum explor_ulog_ctl_info)
#define EXPLORER_IOC_WRITE_DATA			__IOW(EXPLORER_IOC_MAGIC, IOC_NR_WRITE_DATA, struct explorer_training_data)
#define EXPLORER_IOC_SOFT_RESET			__IOW(EXPLORER_IOC_MAGIC, IOC_NR_SOFT_RESET, 0)

#ifdef SLT_ENABLE
#define EXPLORER_IOC_SLT(size)			_IOC(_IOC_WRITE, EXPLORER_IOC_MAGIC, IOC_NR_SLT, size)
#define EXPLORER_IOC_SLT_WF			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_SLT_WAIT_FW, struct explorer_slt_fw)
#endif

#define	EXPLORER_IOC_ISP_WAIT			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_ISP_WAIT, struct explorer_sync_msg)
#define	EXPLORER_IOC_PBL			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_PBL, struct explorer_pbl_info)

#define	EXPLORER_IOC_SEND_TUNNING_DATA			_IOW(EXPLORER_IOC_MAGIC, IOC_NR_SEND_TUNNING_DATA, \
								struct explorer_tunning_data_info)

#define	EXPLORER_IOC_READ_DUMP			_IOR(EXPLORER_IOC_MAGIC, IOC_NR_READDUMP, struct RttSdiMsg)

#if defined (QCOM_AON) || defined (MTK_AON)
#define	EXPLORER_IOC_CAM_CONTROL		_IOWR(EXPLORER_IOC_MAGIC, IOC_NR_CAM_CONTROL, struct aon_control)
#endif

#define	EXPLORER_IOC_LEN_MAX			(4096)

#define	USER_SHARED_MEM_SIZE			(1024*1024)

#endif

