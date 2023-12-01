// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */

#include "cam_ois_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_ois_soc.h"
#include "cam_ois_core.h"
#include "cam_debug_util.h"
#include "camera_main.h"
#include "oplus_cam_ois_soc.h"

void cam_ois_driver_soc_init_oem(struct cam_ois_ctrl_t *o_ctrl, struct device_node *of_node)
{
	const char                     *p = NULL;
	int                             ret = 0;
	int                             id;
	ret = of_property_read_u32(of_node, "ois_gyro,position", &id);
	if (ret) {
		o_ctrl->ois_gyro_position = 1;
		CAM_DBG(CAM_OIS, "get ois_gyro,position failed rc:%d, set default value to %d", ret, o_ctrl->ois_gyro_position);
	} else {
		o_ctrl->ois_gyro_position = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read ois_gyro,position success, value:%d", o_ctrl->ois_gyro_position);
	}

	ret = of_property_read_u32(of_node, "ois,type", &id);
	if (ret) {
		o_ctrl->ois_type = CAM_OIS_MASTER;
		CAM_DBG(CAM_OIS, "get ois,type failed rc:%d, default %d", ret, o_ctrl->ois_type);
	} else {
		o_ctrl->ois_type = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read ois,type success, value:%d", o_ctrl->ois_type);
	}

	ret = of_property_read_u32(of_node, "ois_gyro,type", &id);
	if (ret) {
		o_ctrl->ois_gyro_vendor = 0x02;
		CAM_DBG(CAM_OIS, "get ois_gyro,type failed rc:%d, default %d", ret, o_ctrl->ois_gyro_vendor);
	} else {
		o_ctrl->ois_gyro_vendor = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read ois_gyro,type success, value:%d", o_ctrl->ois_gyro_vendor);
	}

	ret = of_property_read_string_index(of_node, "ois,name", 0, (const char **)&p);
	if (ret) {
		CAM_DBG(CAM_OIS, "get ois,name failed rc:%d, set default value to %s", ret, o_ctrl->ois_name);
	} else {
		memcpy(o_ctrl->ois_name, p, sizeof(o_ctrl->ois_name));
		CAM_INFO(CAM_OIS, "read ois,name success, value:%s", o_ctrl->ois_name);
	}

	ret = of_property_read_u32(of_node, "ois_module,vendor", &id);
	if (ret) {
		o_ctrl->ois_module_vendor = 0x01;
		CAM_DBG(CAM_OIS, "get ois_module,vendor failed rc:%d, default %d", ret, o_ctrl->ois_module_vendor);
	} else {
		o_ctrl->ois_module_vendor = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read ois_module,vendor success, value:%d", o_ctrl->ois_module_vendor);
	}

	ret = of_property_read_u32(of_node, "ois_actuator,vednor", &id);
	if (ret) {
		o_ctrl->ois_actuator_vendor = 0x01;
		CAM_DBG(CAM_OIS, "get ois_actuator,vednor failed rc:%d, default %d", ret, o_ctrl->ois_actuator_vendor);
	} else {
		o_ctrl->ois_actuator_vendor = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read ois_actuator,vednor success, value:%d", o_ctrl->ois_actuator_vendor);
	}

	ret = of_property_read_u32(of_node, "ois,fw", &id);
	if (ret) {
		o_ctrl->ois_fw_flag = 0x01;
		CAM_DBG(CAM_OIS, "get ois,fw failed rc:%d, default %d", ret, o_ctrl->ois_fw_flag);
	} else {
		o_ctrl->ois_fw_flag = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read ois,fw success, value:%d", o_ctrl->ois_fw_flag);
	}

	ret = of_property_read_u32(of_node, "change_cci", &id);
	if (ret) {
		o_ctrl->ois_change_cci = 0x00;
		CAM_DBG(CAM_OIS, "get change_cci failed rc:%d, default %d", ret, o_ctrl->ois_change_cci);
	} else {
		o_ctrl->ois_change_cci = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read change_cci success, value:%d", o_ctrl->ois_change_cci);
	}

	ret = of_property_read_u32(of_node, "ois_eis_function", &id);
	if (ret) {
		o_ctrl->ois_eis_function = 0x00;
		CAM_DBG(CAM_OIS, "get ois_eis_function failed rc:%d, default %d", ret, o_ctrl->ois_eis_function);
	} else {
		o_ctrl->ois_eis_function = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read ois_eis_function success, value:%d", o_ctrl->ois_eis_function);
	}

	ret = of_property_read_u32(of_node, "download,fw", &id);
	if (ret) {
	    o_ctrl->cam_ois_download_fw_in_advance = 0;
		CAM_DBG(CAM_OIS, "get download,fw failed rc:%d, default %d", ret, o_ctrl->cam_ois_download_fw_in_advance);
	} else {
	    o_ctrl->cam_ois_download_fw_in_advance = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read download,fw success, value:%d", o_ctrl->cam_ois_download_fw_in_advance);
	}

	ret = of_property_read_u32(of_node, "ois_switch_spi_mode", &id);
	if (ret) {
		o_ctrl->ois_switch_spi_mode = 0;
		CAM_DBG(CAM_OIS, "get ois_switch_spi_mode failed rc:%d, default %d", ret, o_ctrl->ois_switch_spi_mode);
	} else {
		o_ctrl->ois_switch_spi_mode = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read ois_switch_spi_mode success, value:%d", o_ctrl->ois_switch_spi_mode);
	}

	ret = of_property_read_u32(of_node, "actuator_ois_eeprom_merge", &id);
	if (ret) {
		o_ctrl->actuator_ois_eeprom_merge_flag = 0;
		CAM_DBG(CAM_OIS, "get actuator_ois_eeprom_merge_flag failed rc:%d, default %d", ret, o_ctrl->actuator_ois_eeprom_merge_flag);
	} else {
		o_ctrl->actuator_ois_eeprom_merge_flag = (uint8_t)id;
		CAM_INFO(CAM_OIS, "read actuator_ois_eeprom_merge_flag success, value:%d", o_ctrl->actuator_ois_eeprom_merge_flag);

		o_ctrl->actuator_ois_eeprom_merge_mutex = &actuator_ois_eeprom_shared_mutex;
		if (!actuator_ois_eeprom_shared_mutex_init_flag) {
			mutex_init(o_ctrl->actuator_ois_eeprom_merge_mutex);
			actuator_ois_eeprom_shared_mutex_init_flag = true;
		}
	}
}
