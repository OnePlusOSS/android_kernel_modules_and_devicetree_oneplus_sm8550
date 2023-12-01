#ifndef _DOWNLOAD_OIS_FW_H_
#define _DOWNLOAD_OIS_FW_H_

#include <linux/module.h>
#include <linux/firmware.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_dev.h"
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"

#include <linux/string.h>
#include <linux/time.h>
#include <linux/types.h>


//int RamWrite32A(uint32_t addr, uint32_t data);
//int RamRead32A(uint32_t addr, uint32_t* data);
int RamWrite32A_oneplus(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t data);
int RamRead32A_oneplus(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data);
int DownloadFW(struct cam_ois_ctrl_t *o_ctrl);
int OISControl(struct cam_ois_ctrl_t *o_ctrl);
void ReadOISHALLData(struct cam_ois_ctrl_t *o_ctrl, void *data);
void ReadOISHALLDataV2(struct cam_ois_ctrl_t *o_ctrl, void *data);
void ReadOISHALLDataV3(struct cam_ois_ctrl_t *o_ctrl, void *data);
bool IsOISReady(struct cam_ois_ctrl_t *o_ctrl);
void InitOIS(struct cam_ois_ctrl_t *o_ctrl);
void DeinitOIS(struct cam_ois_ctrl_t *o_ctrl);
void InitOISResource(struct cam_ois_ctrl_t *o_ctrl);
int OIS_READ_HALL_DATA_TO_UMD (struct cam_ois_ctrl_t *o_ctrl,struct i2c_settings_array *i2c_settings);
int WRITE_QTIMER_TO_OIS (struct cam_ois_ctrl_t *o_ctrl);
int OIS_READ_HALL_DATA_TO_UMD_NEW (struct cam_ois_ctrl_t *o_ctrl,struct i2c_settings_array *i2c_settings);
int OIS_READ_HALL_DATA_TO_UMD_TELE124 (struct cam_ois_ctrl_t *o_ctrl,struct i2c_settings_array *i2c_settings);
int OIS_READ_HALL_DATA_TO_UMD_129 (struct cam_ois_ctrl_t *o_ctrl,struct i2c_settings_array *i2c_settings);
int OIS_READ_HALL_DATA_TO_UMD_Bu24721 (struct cam_ois_ctrl_t *o_ctrl,struct i2c_settings_array *i2c_settings);
int OIS_READ_HALL_DATA_TO_UMD_SEM1217S (struct cam_ois_ctrl_t *o_ctrl,struct i2c_settings_array *i2c_settings);

int StoreOisGyroGian(struct cam_ois_ctrl_t *o_ctrl);
int WriteOisGyroGianToRam(struct cam_ois_ctrl_t *o_ctrl,DUAL_OIS_CALI_RESULTS* current_gyro_gain);

int WriteOisGyroGian(struct cam_ois_ctrl_t *o_ctrl,OIS_GYROGAIN* current_gyro_gain);

int DoDualOisGyroOffset(struct cam_ois_ctrl_t *o_ctrl, uint32_t *gyro_data);
int DoBU24721GyroOffset(struct cam_ois_ctrl_t *o_ctrl, uint32_t *gyro_data);
int DoSEM1217SGyroOffset(struct cam_ois_ctrl_t *o_ctrl, uint32_t *gyro_data);

int QueryDualOisSmaWireStatus(struct cam_ois_ctrl_t *o_ctrl, uint32_t *sma_wire_broken);
int GetDualOisGyroGain(struct cam_ois_ctrl_t *o_ctrl,DUAL_OIS_GYROGAIN* initial_gyro_gain);
int WriteDualOisShiftRegister(struct cam_ois_ctrl_t *o_ctrl, uint32_t distance);

int32_t oplus_cam_ois_construct_default_power_setting(struct cam_sensor_power_ctrl_t *power_info);
int32_t oplus_cam_ois_construct_default_power_setting_129(struct cam_sensor_power_ctrl_t *power_info);
int32_t oplus_cam_ois_construct_default_power_setting_bu24721(struct cam_sensor_power_ctrl_t *power_info);
int32_t oplus_cam_ois_construct_default_power_setting_1217s(struct cam_sensor_power_ctrl_t *power_info);


int cam_ois_download_start(struct cam_ois_ctrl_t *o_ctrl);
void cam_ois_do_power_down(struct cam_ois_ctrl_t *o_ctrl);
void cam_set_ois_disable(struct cam_ois_ctrl_t *o_ctrl);

int RohmOisWrite(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t data);
int RohmOisRead(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data);
int Rohm_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl);
void spi_mode_switch(struct cam_ois_ctrl_t *o_ctrl);

uint8_t I2C_FW_8bit__read(uint32_t addr);
uint32_t I2C_FM_32bit__read(uint32_t addr);
uint32_t I2C_FM_16bit__read(uint32_t addr);

uint8_t sem1217_8bit_read(uint32_t addr);
uint16_t sem1217_16bit_read(uint32_t addr);
int sem1217_8bit_write(uint32_t addr, uint8_t data);
int sem1217_block_write(void* register_data,int size);

int I2C_FM_8bit_write( uint32_t addr, uint8_t data);
int I2C_FM_16bit_write( uint32_t addr, uint16_t data);
void I2C_FM_block_write(void* register_data,int size);



uint8_t I2C_OIS_8bit__read(uint32_t addr);
uint16_t I2C_OIS_16bit__read(uint32_t addr);
int I2C_OIS_32bit__read(uint32_t addr, uint32_t* data);

int I2C_OIS_8bit_write(uint32_t addr, uint8_t data);
int I2C_OIS_16bit_write(uint32_t addr, uint16_t data);
int I2C_OIS_32bit_write(uint32_t addr, uint32_t data);

void I2C_OIS_block_write(void* register_data,int size);
void OISCountinueRead(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, void *data, uint16_t size);


void Wait(int us);
int oplus_cam_ois_do_push_center(struct cam_ois_ctrl_t *o_ctrl);
int oplus_cam_ois_push_center(struct cam_ois_ctrl_t *o_ctrl, void *arg);
void oplus_cam_ois_sds_enable(struct v4l2_subdev *sd, void *arg);
int32_t oplus_cam_ois_lock(struct v4l2_subdev *sd);
int32_t oplus_cam_ois_unlock(struct v4l2_subdev *sd);

int32_t oplus_cam_ois_fill_power_setting(struct cam_ois_ctrl_t *o_ctrl, struct cam_sensor_power_ctrl_t *power_info);



#endif
/* _DOWNLOAD_OIS_FW_H_ */

