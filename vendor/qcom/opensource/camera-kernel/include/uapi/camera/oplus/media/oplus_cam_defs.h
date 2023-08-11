/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 */

#define CAM_OEM_COMMON_OPCODE_BASE              0x8000
#define CAM_GET_OIS_EIS_HALL                    (CAM_COMMON_OPCODE_BASE + 0xD)
#define CAM_WRITE_CALIBRATION_DATA              (CAM_OEM_COMMON_OPCODE_BASE + 0x2)
#define CAM_CHECK_CALIBRATION_DATA              (CAM_OEM_COMMON_OPCODE_BASE + 0x3)
#define CAM_WRITE_AE_SYNC_DATA                  (CAM_OEM_COMMON_OPCODE_BASE + 0x4)
#define CAM_OEM_IO_CMD                          (CAM_OEM_COMMON_OPCODE_BASE + 0x5)
#define CAM_OEM_GET_ID                          (CAM_OEM_COMMON_OPCODE_BASE + 0x6)
#define CAM_GET_DPC_DATA                        (CAM_OEM_COMMON_OPCODE_BASE + 0x7)
#define CAM_STORE_DUALOIS_GYRO_GAIN             (CAM_OEM_COMMON_OPCODE_BASE + 0x8)
#define CAM_WRITE_DUALOIS_GYRO_GAIN             (CAM_OEM_COMMON_OPCODE_BASE + 0x9)
#define CAM_DO_DUALOIS_GYRO_OFFSET              (CAM_OEM_COMMON_OPCODE_BASE + 0xA)
#define CAM_QUERY_DUALOIS_SMA_WIRE_STATUS       (CAM_OEM_COMMON_OPCODE_BASE + 0xB)
#define CAM_GET_DUALOIS_INITIAL_GYRO_GAIN       (CAM_OEM_COMMON_OPCODE_BASE + 0xC)
#define CAM_WRITE_SHIFT_OIS_REGISTER            (CAM_OEM_COMMON_OPCODE_BASE + 0xD)
#define CAM_FIRMWARE_CALI_GYRO_OFFSET           (CAM_OEM_COMMON_OPCODE_BASE + 0xE)
#define CAM_TELE_OIS_USE_MONITOR                (CAM_OEM_COMMON_OPCODE_BASE + 0xF)


#define CAM_OEM_CMD_READ_DEV                    0
#define CAM_OEM_CMD_WRITE_DEV                   1
#define CAM_OEM_OIS_CALIB                       2
#define CAM_OEM_RW_SIZE_MAX                    3000

struct cam_oem_i2c_reg_array {
	unsigned int    reg_addr;
	unsigned int    reg_data;
	unsigned int    delay;
	unsigned int    data_mask;
};

struct cam_oem_rw_ctl {
	signed int                cmd_code;
	unsigned long long        cam_regs_ptr;
	unsigned int              slave_addr;
	unsigned int              reg_data_type;
	signed int                reg_addr_type;
	signed short              num_bytes;
};

struct cam_oem_initsettings {
	struct cam_oem_i2c_reg_array reg_setting[CAM_OEM_RW_SIZE_MAX];
	signed short size;
	unsigned short addr_type;
	unsigned short data_type;
};

/*add for get hall dat for EIS*/
#define HALL_MAX_NUMBER 12
struct ois_hall_type {
	unsigned int    dataNum;
	unsigned int    mdata[HALL_MAX_NUMBER];
	unsigned int    timeStamp;
};

typedef struct dual_ois_calibration_t
{
	int LsGyroGain_X;
	int LsGyroGain_Y;
	int SsGyroGain_A;
	int SsGyroGain_B;
	int SsGyroGain_C;
	int mode;
	int successed;
} DUAL_OIS_CALI_RESULTS;

typedef struct ois_gyrogain_t
{
	int32_t GyroGain_X;
	int32_t GyroGain_Y;
} OIS_GYROGAIN;

typedef struct dual_ois_gyrogain_t
{
	unsigned int LsGyroGain_X;
	unsigned int LsGyroGain_Y;
	unsigned int SsGyroGain_A;
	unsigned int SsGyroGain_B;
	unsigned int SsGyroGain_C;
} DUAL_OIS_GYROGAIN;

#define VIDIOC_CAM_FTM_POWNER_UP 0
#define VIDIOC_CAM_FTM_POWNER_DOWN 1
#define VIDIOC_CAM_AON_POWNER_UP                  0x5000
#define VIDIOC_CAM_AON_POWNER_DOWN                0x5001
#define VIDIOC_CAM_AON_QUERY_INFO                 0x5002

#define VIDIOC_CAM_SENSOR_STATR 0x9000
#define VIDIOC_CAM_SENSOR_STOP 0x9001

