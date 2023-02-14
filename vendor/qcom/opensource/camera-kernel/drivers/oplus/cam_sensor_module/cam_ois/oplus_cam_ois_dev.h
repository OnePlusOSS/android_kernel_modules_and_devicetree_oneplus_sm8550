/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */
 
#include <linux/kfifo.h>

#define GET_HALL_DATA_VERSION_DEFUALT         0
#define GET_HALL_DATA_VERSION_V2              1
#define GET_HALL_DATA_VERSION_V3              2
struct cam_ois_hall_data_in_ois_aligned {
	uint16_t hall_data_cnt;
	uint32_t hall_data;
};

struct cam_ois_hall_data_in_driver {
	uint32_t high_dword;
	uint32_t low_dword;
	uint32_t hall_data;
};

#define SAMPLE_COUNT_IN_DRIVER        100
#define SAMPLE_COUNT_IN_OIS           34
#define SAMPLE_SIZE_IN_OIS            6
#define SAMPLE_SIZE_IN_OIS_ALIGNED    (sizeof(struct cam_ois_hall_data_in_ois_aligned))
#define SAMPLE_SIZE_IN_DRIVER         (sizeof(struct cam_ois_hall_data_in_driver))
#define CLOCK_TICKCOUNT_MS            19200
#define OIS_MAGIC_NUMBER              0x7777
#define OIS_MAX_COUNTER               36
#define SAMPLE_COUNT_IN_NCS_DATA      32
#define SAMPLE_COUNT_IN_NCS_DATA_TELE124  28

#define SAMPLE_SIZE_IN_OIS_NCS        144

#define ENABLE_OIS_DELAY_POWER_DOWN

#ifdef ENABLE_OIS_DELAY_POWER_DOWN
#define OIS_POWER_DOWN_DELAY 500//ms
enum cam_ois_power_down_thread_state {
	CAM_OIS_POWER_DOWN_THREAD_RUNNING,
	CAM_OIS_POWER_DOWN_THREAD_STOPPED,
};

enum cam_ois_power_state {
	CAM_OIS_POWER_ON,
	CAM_OIS_POWER_OFF,
};
#endif

enum cam_ois_type_vendor {
	CAM_OIS_MASTER,
	CAM_OIS_SLAVE,
	CAM_OIS_TYPE_MAX,
};

enum cam_ois_state_vendor {
	CAM_OIS_INVALID,
	CAM_OIS_FW_DOWNLOADED,
	CAM_OIS_READY,
};

enum cam_ois_control_cmd {
	CAM_OIS_START_POLL_THREAD,
	CAM_OIS_STOP_POLL_THREAD,
};

enum cam_ois_close_state {
	CAM_OIS_IS_OPEN,
	CAM_OIS_IS_DOING_CLOSE,
	CAM_OIS_IS_CLOSE,
};
enum cam_ois_download_fw_state {
	CAM_OIS_FW_NOT_DOWNLOAD,
	CAM_OIS_FW_DOWNLOAD_DONE,
};

