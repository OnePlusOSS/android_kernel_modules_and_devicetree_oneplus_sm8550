/********************************************************************
 * Copyright (c) 2020-2030 OPLUS Mobile Comm Corp.,Ltd. All Rights Reserved.
 * OPLUS_FEATURE_CAMERA_COMMON
 * File                  : oplus_cam_eeprom_dev.h
 * Version               : 1.0
 * Description           : add some Statements for cam_eeprom_dev.h.
 * Date                  : 2021-07-15
 * -------------Rivision History-----------------------------------------
 * <version>      <date>       <author>       <desc>
 * 1.0            2021-07-15   huangqipeng    add for engineermode
 * OPLUS Mobile Comm Proprietary and Confidential.
 *********************************************************************/
#ifndef _OPLUS_CAM_EEPROM_DEV_H_
#define _OPLUS_CAM_EEPROM_DEV_H_

#include "cam_eeprom_dev.h"
extern bool chip_version_old;
void cam_eeprom_update_i2c_info_oem(struct cam_eeprom_ctrl_t * e_ctrl,
	struct cam_eeprom_i2c_info_t *i2c_info, bool chip_version_old);
void cam_eeprom_component_bind_oem(struct cam_eeprom_ctrl_t * e_ctrl);
#endif/* _OPLUS_CAM_EEPROM_DEV_H_ */
