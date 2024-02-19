/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef _CAM_OIS_DEV_H_
#define _CAM_OIS_DEV_H_

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/cam_sensor.h>
#include <cam_sensor_i2c.h>
#include <cam_sensor_spi.h>
#include <cam_sensor_io.h>
#include <cam_cci_dev.h>
#include <cam_req_mgr_util.h>
#include <cam_req_mgr_interface.h>
#include <cam_mem_mgr.h>
#include <cam_subdev.h>
#include "cam_soc_util.h"
#include "cam_context.h"
#include <linux/kfifo.h>
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include "oplus_cam_ois_dev.h"
#endif

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

#define OIS_DRIVER_I2C "cam-i2c-ois"
#define OIS_DRIVER_I3C "i3c_camera_ois"

enum cam_ois_state {
	CAM_OIS_INIT,
	CAM_OIS_ACQUIRE,
	CAM_OIS_CONFIG,
	CAM_OIS_START,
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	CAM_OIS_LOCK,
#endif
};

/**
 * struct cam_ois_i2c_info_t - I2C info
 * @slave_addr      :   slave address
 * @i2c_freq_mode   :   i2c frequency mode
 *
 */
struct cam_ois_i2c_info_t {
	uint16_t slave_addr;
	uint8_t i2c_freq_mode;
};

/**
 * struct cam_ois_soc_private - ois soc private data structure
 * @ois_name        :   ois name
 * @i2c_info        :   i2c info structure
 * @power_info      :   ois power info
 *
 */
struct cam_ois_soc_private {
	const char *ois_name;
	struct cam_ois_i2c_info_t i2c_info;
	struct cam_sensor_power_ctrl_t power_info;
};

/**
 * struct cam_ois_intf_params - bridge interface params
 * @device_hdl   : Device Handle
 * @session_hdl  : Session Handle
 * @ops          : KMD operations
 * @crm_cb       : Callback API pointers
 */
struct cam_ois_intf_params {
	int32_t device_hdl;
	int32_t session_hdl;
	int32_t link_hdl;
	struct cam_req_mgr_kmd_ops ops;
	struct cam_req_mgr_crm_cb *crm_cb;
};

/**
 * struct cam_ois_ctrl_t - OIS ctrl private data
 * @device_name     :   ois device_name
 * @pdev            :   platform device
 * @ois_mutex       :   ois mutex
 * @soc_info        :   ois soc related info
 * @io_master_info  :   Information about the communication master
 * @cci_i2c_master  :   I2C structure
 * @v4l2_dev_str    :   V4L2 device structure
 * @is_i3c_device   :   A Flag to indicate whether this OIS is I3C Device or not.
 * @bridge_intf     :   bridge interface params
 * @i2c_fwinit_data :   ois i2c firmware init settings
 * @i2c_init_data   :   ois i2c init settings
 * @i2c_mode_data   :   ois i2c mode settings
 * @i2c_time_data   :   ois i2c time write settings
 * @i2c_calib_data  :   ois i2c calib settings
 * @ois_device_type :   ois device type
 * @cam_ois_state   :   ois_device_state
 * @ois_fw_flag     :   flag for firmware download
 * @is_ois_calib    :   flag for Calibration data
 * @opcode          :   ois opcode
 * @device_name     :   Device name
 *
 */
struct cam_ois_ctrl_t {
	char device_name[CAM_CTX_DEV_NAME_MAX_LENGTH];
	struct platform_device *pdev;
	struct mutex ois_mutex;
	struct cam_hw_soc_info soc_info;
	struct camera_io_master io_master_info;
	enum cci_i2c_master_t cci_i2c_master;
	enum cci_device_num cci_num;
	struct cam_subdev v4l2_dev_str;
	bool is_i3c_device;
	struct cam_ois_intf_params bridge_intf;
	struct i2c_settings_array i2c_fwinit_data;
	struct i2c_settings_array i2c_init_data;
	struct i2c_settings_array i2c_calib_data;
	struct i2c_settings_array i2c_mode_data;
	struct i2c_settings_array i2c_time_data;
	enum msm_camera_device_type_t ois_device_type;
	enum cam_ois_state cam_ois_state;
	char ois_name[32];
	uint8_t ois_fw_flag;
	uint8_t is_ois_calib;
	struct cam_ois_opcode opcode;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	enum cam_ois_type_vendor ois_type;  //Master or Slave
	uint8_t ois_gyro_position;          //Gyro positon
	uint8_t ois_gyro_vendor;            //Gyro vendor
	uint8_t ois_actuator_vendor;        //Actuator vendor
	uint8_t ois_module_vendor;          //Module vendor
	uint8_t ois_switch_spi_mode;
	uint8_t actuator_ois_eeprom_merge_flag;
	struct mutex ois_read_mutex;
	struct mutex *actuator_ois_eeprom_merge_mutex;
	bool ois_read_thread_start_to_read;
	struct task_struct *ois_read_thread;
	struct mutex ois_hall_data_mutex;
	struct mutex ois_poll_thread_mutex;
	bool ois_poll_thread_exit;
	enum cam_ois_control_cmd ois_poll_thread_control_cmd;
	struct task_struct *ois_poll_thread;
	struct kfifo ois_hall_data_fifo;
	struct kfifo ois_hall_data_fifoV2;
	bool isTeleOisUseMonitor;
	bool pre_isTeleOisUseMonitor;
	bool camera_ois_shake_detect_enable;
	enum cam_ois_state cam_ois_last_state;
	bool ois_hall_data_debug_enable;
	bool dump_hall_thread_run;
	struct task_struct *ois_print_hall_data_thread;
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
	struct mutex ois_power_down_mutex;
	enum cam_ois_power_down_thread_state ois_power_down_thread_state;
	enum cam_ois_power_state ois_power_state;
	bool ois_power_down_thread_exit;
#endif
	uint8_t ois_eis_function;
	uint8_t ois_change_cci;
	/*ois download in advance*/
	struct task_struct *ois_downloadfw_thread;
	struct mutex do_ioctl_ois;
	enum cam_ois_download_fw_state ois_download_fw_done;
	enum cam_ois_close_state ois_fd_have_close_state;
	int  cam_ois_download_fw_in_advance;
#endif
};

/**
 * @brief : API to register OIS hw to platform framework.
 * @return struct platform_device pointer on on success, or ERR_PTR() on error.
 */
int cam_ois_driver_init(void);

/**
 * @brief : API to remove OIS Hw from platform framework.
 */
void cam_ois_driver_exit(void);
#endif /*_CAM_OIS_DEV_H_ */
