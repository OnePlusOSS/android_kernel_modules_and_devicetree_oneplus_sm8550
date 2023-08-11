/********************************************************************
 * Copyright (c) 2020-2030 OPLUS Mobile Comm Corp.,Ltd. All Rights Reserved.
 * OPLUS_FEATURE_CAMERA_COMMON
 * File                  : oplus_cam_eeprom_dev.c
 * Version               : 1.0
 * Description           : add some oem code modifications for cam_eeprom_dev.c.
 * Date                  : 2021-07-15
 * -------------Rivision History-----------------------------------------
 * <version>      <date>       <author>       <desc>
 * 1.0            2021-07-15   huangqipeng    add for engineermode
 * OPLUS Mobile Comm Proprietary and Confidential.
 *********************************************************************/
#include "cam_eeprom_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_eeprom_soc.h"
#include "cam_eeprom_core.h"
#include "cam_debug_util.h"
#include "camera_main.h"
#include "oplus_cam_eeprom_dev.h"

void cam_eeprom_update_i2c_info_oem(struct cam_eeprom_ctrl_t * e_ctrl,
	struct cam_eeprom_i2c_info_t *i2c_info, bool chip_version_old) {
	if (e_ctrl->change_cci && (chip_version_old == FALSE)) {
		e_ctrl->io_master_info_ois.cci_client->sid = (i2c_info->slave_addr) >> 1;
		e_ctrl->io_master_info_ois.cci_client->retries = 3;
		e_ctrl->io_master_info_ois.cci_client->id_map = 0;
		e_ctrl->io_master_info_ois.cci_client->i2c_freq_mode = i2c_info->i2c_freq_mode;
	}
}

void cam_eeprom_component_bind_oem(struct cam_eeprom_ctrl_t * e_ctrl) {
	e_ctrl->io_master_info_ois.master_type = CCI_MASTER;
	e_ctrl->io_master_info_ois.cci_client = kzalloc(
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
}
