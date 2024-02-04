/********************************************************************
 * Copyright (c) 2020-2030 OPLUS Mobile Comm Corp.,Ltd. All Rights Reserved.
 * OPLUS_FEATURE_CAMERA_COMMON
 * File                  : oplus_cam_eeprom_soc.h
 * Version               : 1.0
 * Description           : add some Statements for cam_eeprom_soc.h.
 * Date                  : 2021-07-15
 * -------------Rivision History-----------------------------------------
 * <version>      <date>       <author>       <desc>
 * 1.0            2021-07-15   huangqipeng    add for engineermode
 * OPLUS Mobile Comm Proprietary and Confidential.
 *********************************************************************/
#ifndef _OPLUS_CAM_EEPROM_SOC_H_
#define _OPLUS_CAM_EEPROM_SOC_H_

#include "cam_eeprom_dev.h"

void cam_eeprom_parse_dt_oem(struct cam_eeprom_ctrl_t *e_ctrl, struct device_node *of_node);
#endif/* _OPLUS_CAM_EEPROM_SOC_H_ */
