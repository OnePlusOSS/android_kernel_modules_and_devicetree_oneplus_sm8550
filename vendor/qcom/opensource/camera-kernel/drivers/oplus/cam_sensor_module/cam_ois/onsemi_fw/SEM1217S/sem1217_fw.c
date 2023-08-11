/////////////////////////////////////////////////////////////////////////////
// File Name    : bu24721_fw.c
// Function        : Various function for OIS control
//
// Copyright(c)    Rohm Co.,Ltd. All rights reserved
//
/***** ROHM Confidential ***************************************************/
//#define    _USE_MATH_DEFINES                            //

#ifndef SEM1217_FW_C
#define SEM1217_FW_C
#endif
#include <linux/types.h>
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "fw_download_interface.h"
#include "sem1217_fw.h"

static int block_write_count = 0;

int SEM1217S_Store_OIS_Cal_Data (void)
{
	uint8_t txdata[SEM1217S_TX_BUFFER_SIZE];
	uint8_t rxdata[SEM1217S_RX_BUFFER_SIZE];
	uint16_t repeatedCnt = 1000;
	int32_t data_error = 0xFFFF;

	rxdata[0] = sem1217_8bit_read(SEM1217S_REG_OIS_STS);
	CAM_INFO(CAM_OIS, "SEM1217S_REG_OIS_STS: %u", rxdata[0]);

	if (rxdata[0] != SEM1217S_STATE_READY)
	{
		txdata[0] = SEM1217S_OIS_OFF;
		sem1217_8bit_write(SEM1217S_REG_OIS_CTRL, txdata[0]);
	}

	txdata[0] = SEM1217S_OIS_INFO_EN; /* Set OIS_INFO_EN */
	/* Write 1 Byte to REG_INFO_BLK_UP_CTRL */
	sem1217_8bit_write(SEM1217S_REGINFO_BLK_UP_CTRL, txdata[0]);
	CAM_INFO(CAM_OIS, "write SEM1217S_REGINFO_BLK_UP_CTRL: %u", txdata[0]);
	//I2C_Write_Data(REG_INFO_BLK_UP_CTRL, 1, txdata);
	mdelay(100); /* Delay 100 ms */

	do
	{
		if (repeatedCnt == 0)
		{
			/* Abnormal Termination Error. */
			CAM_ERR(CAM_OIS, "REG_INFO_BLK_UP_CTRL failed: %u", rxdata[0]);
			return 0;
		}
		mdelay(50); /* Delay 50 ms */
		rxdata[0]=sem1217_8bit_read(SEM1217S_REGINFO_BLK_UP_CTRL);
		repeatedCnt--;
	} while ((rxdata[0] & SEM1217S_OIS_INFO_EN) == SEM1217S_OIS_INFO_EN);

	rxdata[0] = sem1217_8bit_read(SEM1217S_REG_OIS_ERR);
	rxdata[1] = sem1217_8bit_read(SEM1217S_REG_OIS_ERR + 1);
	data_error = (uint16_t)((rxdata[0]) | (rxdata[1] << 8 ));

	if ((data_error & SEM1217S_ERR_ODI) != SEM1217S_NO_ERROR)
	{
		/* Different INFORWRITE data on flash */
		CAM_ERR(CAM_OIS, "SEM1217S_ERR_ODI error %d", data_error);
		return 0;
	}

	return 1;
	/* INFORWRITE data on flash Success Process */
}

struct SEM1217S_FACT_ADJ SEM1217S_Gyro_offset_cal(void)
{
	uint8_t txdata[SEM1217S_TX_BUFFER_SIZE];
	uint8_t rxdata[SEM1217S_RX_BUFFER_SIZE];
	uint16_t repeatedCnt = 1000;
	uint16_t data_error = 0xFFFF;
	int rc = 0;

	struct SEM1217S_FACT_ADJ SEM1217S_FADJ_CAL = { 0x0000, 0x0000};

	rxdata[0] = sem1217_8bit_read(SEM1217S_REG_OIS_STS);
	CAM_INFO(CAM_OIS, "SEM1217S_REG_OIS_STS: %u", rxdata[0]);

	if (rxdata[0] != SEM1217S_STATE_READY)
	{
		txdata[0] = SEM1217S_OIS_OFF;
		sem1217_8bit_write(SEM1217S_REG_OIS_CTRL, txdata[0]);
	}

	rxdata[0] = sem1217_8bit_read(SEM1217S_REG_AF_STS);
	CAM_INFO(CAM_OIS, "SEM1217S_REG_AF_STS: %u", rxdata[0]);

	if (rxdata[0] != SEM1217S_STATE_READY)
	{
		txdata[0] = SEM1217S_AF_OFF;
		sem1217_8bit_write(SEM1217S_REG_AF_CTRL, txdata[0]);
	}

	txdata[0] = SEM1217S_G_OFFSET_EN;
	sem1217_8bit_write(SEM1217S_REG_GCAL_CTRL, txdata[0]);
	mdelay(50);

	do
	{
		if (repeatedCnt == 0)
		{
			return SEM1217S_FADJ_CAL;
		}
		mdelay(50);
		rxdata[0] = sem1217_8bit_read(SEM1217S_REG_GCAL_CTRL);
		CAM_INFO(CAM_OIS, "SEM1217S_REG_GCAL_CTRL: %u", rxdata[0]);
		repeatedCnt--;
	} while ((rxdata[0] & SEM1217S_G_OFFSET_EN) == SEM1217S_G_OFFSET_EN);

	rxdata[0] = sem1217_8bit_read(SEM1217S_REG_OIS_ERR);
	rxdata[1] = sem1217_8bit_read(SEM1217S_REG_OIS_ERR + 1);
	data_error = (uint16_t)((rxdata[0]) | (rxdata[1] << 8 ));

	if ((data_error & (SEM1217S_ERR_GCALX | SEM1217S_ERR_GCALY)) != SEM1217S_NO_ERROR)
	{
		CAM_ERR(CAM_OIS, "SEM1217S offset cal failed, data_error: 0x%x", data_error);
		SEM1217S_FADJ_CAL.gl_GX_OFS = 0;
		SEM1217S_FADJ_CAL.gl_GY_OFS = 0;
		return SEM1217S_FADJ_CAL;
	}

	rxdata[0] = sem1217_8bit_read(SEM1217S_REG_GX_OFFSET);
	rxdata[1] = sem1217_8bit_read(SEM1217S_REG_GX_OFFSET + 1);
	SEM1217S_FADJ_CAL.gl_GX_OFS = (uint16_t)((rxdata[0]) | (rxdata[1] << 8 ));

	rxdata[0] = sem1217_8bit_read(SEM1217S_REG_GY_OFFSET);
	rxdata[1] = sem1217_8bit_read(SEM1217S_REG_GY_OFFSET + 1);
	SEM1217S_FADJ_CAL.gl_GY_OFS = (uint16_t)((rxdata[0]) | (rxdata[1] << 8 ));
	CAM_INFO(CAM_OIS, "sem1217s x-offset: %u, y-offset: %u", SEM1217S_FADJ_CAL.gl_GX_OFS, SEM1217S_FADJ_CAL.gl_GY_OFS);

	rc = SEM1217S_Store_OIS_Cal_Data();

	return SEM1217S_FADJ_CAL;
}

int sem1217s_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint8_t txdata[SEM1217S_TX_BUFFER_SIZE + 2];
	uint8_t rxdata[SEM1217S_RX_BUFFER_SIZE];
	struct device_node *of_node = NULL;
	uint8_t* sem1217s_addr = NULL;
	uint16_t txBuffSize;
	uint32_t i, chkIdx;
	uint16_t subaddr_FLASH_DATA_BIN_1;
	uint32_t fw_data_size;
	struct cam_hw_soc_info *soc_info = &o_ctrl->soc_info;

	uint16_t idx = 0;
	uint16_t check_sum;
	uint32_t updated_ver;
	uint32_t new_fw_ver;
	uint32_t current_fw_ver;
	int rc = 0;

	of_node = soc_info->dev->of_node;

	fw_data_size = of_property_count_u8_elems(of_node, "fw_data");
	if (fw_data_size < 12)
	{
		CAM_ERR(CAM_OIS, "ptr->SizeFromCode(%d) < 12, ", fw_data_size);
		return -1;
	}
	CAM_INFO(CAM_OIS,"allocating fw_data_size: %u", fw_data_size);

	block_write_count = 0;
	sem1217s_addr = (uint8_t*)kzalloc(fw_data_size, GFP_KERNEL);
	if (!sem1217s_addr)
	{
		CAM_ERR(CAM_OIS, "Failed in allocating fw_data_size: %u", fw_data_size);
		return -1;
	}
	else
	{
		memset(sem1217s_addr, 0, fw_data_size);
	}

	rc = of_property_read_u8_array(of_node, "fw_data", sem1217s_addr, fw_data_size);
	if ( rc < 0)
	{
		CAM_ERR(CAM_OIS, "Invalid fw_data params");
	}

	rxdata[0] = sem1217_8bit_read(SEM1217S_REG_APP_VER);
	rxdata[1] = sem1217_8bit_read(SEM1217S_REG_APP_VER + 1);
	rxdata[2] = sem1217_8bit_read(SEM1217S_REG_APP_VER + 2);
	rxdata[3] = sem1217_8bit_read(SEM1217S_REG_APP_VER + 3);
	new_fw_ver = *(uint32_t *)&sem1217s_addr[fw_data_size - 12];
	current_fw_ver = ((uint32_t *)rxdata)[0];

	CAM_ERR(CAM_OIS, "current_fw_ver: 0x%x, new_fw_ver: 0x%x", current_fw_ver, new_fw_ver);
	if (current_fw_ver == new_fw_ver)
	{
		CAM_ERR(CAM_OIS, "version is the same, no need to update");
		return 0;
	}

	if (o_ctrl->actuator_ois_eeprom_merge_flag)
	{
		CAM_DBG(CAM_OIS, "before actuator_ois_eeprom_merge_flag lock");
		mutex_lock(o_ctrl->actuator_ois_eeprom_merge_mutex);
		CAM_DBG(CAM_OIS, "after actuator_ois_eeprom_merge_flag lock");
	}

	if (current_fw_ver != 0)
	{
		rxdata[0] = sem1217_8bit_read(SEM1217S_REG_OIS_STS);
		if (rxdata[0] != SEM1217S_STATE_READY)
		{
			txdata[0] = SEM1217S_OIS_OFF;
			rc = sem1217_8bit_write(SEM1217S_REG_OIS_CTRL, txdata[0]);
			if (rc != 0)
			{
				goto error_hand;
			}
		}

		rxdata[0] = sem1217_8bit_read(SEM1217S_REG_AF_STS);
		if (rxdata[0] != SEM1217S_STATE_READY)
		{
			txdata[0] = SEM1217S_AF_OFF;
			rc = sem1217_8bit_write(SEM1217S_REG_AF_CTRL, txdata[0]);
			if (rc != 0)
			{
				goto error_hand;
			}
		}
	}

	txBuffSize = SEM1217S_TX_SIZE_256_BYTE;
	txdata[0] = SEM1217S_FWUP_CTRL_256_SET;
	rc = sem1217_8bit_write(SEM1217S_REG_FWUP_CTRL, txdata[0]);
	if (rc != 0)
	{
		goto error_hand;
	}

	msleep(60);
	check_sum = 0;

	subaddr_FLASH_DATA_BIN_1 = SEM1217S_REG_DATA_BUF;
	for (i = 0; i < (SEM1217S_APP_FW_SIZE / txBuffSize); i++)
	{
		for (chkIdx = 0; chkIdx < txBuffSize; chkIdx += 2)
		{
			check_sum += ((sem1217s_addr[chkIdx + 1 + (txBuffSize * i)] << 8) | sem1217s_addr[chkIdx + (txBuffSize * i)]);
		}

		memcpy(txdata + 2, &sem1217s_addr[idx], txBuffSize);
		txdata[0] = (subaddr_FLASH_DATA_BIN_1 >> 8);
		txdata[1] = (subaddr_FLASH_DATA_BIN_1 & 0xFF);
		rc = sem1217_block_write(txdata, SEM1217S_TX_BUFFER_SIZE + 2);
		if (rc != 0)
		{
			goto error_hand;
		}
		CAM_DBG(CAM_OIS, "update ois fw blk_num: %d", i+1);
		idx += txBuffSize;
		mdelay(20);
	}

	((uint16_t*)txdata)[1] = check_sum;
	txdata[0] = (SEM1217S_REG_FWUP_CHKSUM >> 8);
	txdata[1] = (SEM1217S_REG_FWUP_CHKSUM & 0xFF);
	rc = sem1217_block_write(txdata, 4);
	if (rc != 0)
	{
		goto error_hand;
	}
	mdelay(200);

	rxdata[0] = sem1217_8bit_read(SEM1217S_REG_FWUP_ERR);
	if (rxdata[0] != SEM1217S_NO_ERROR)
	{
		CAM_ERR(CAM_OIS, "update fw erro");
		goto error_hand;
	}

	txdata[0] = SEM1217S_RESET_REQ;
	rc = sem1217_8bit_write(SEM1217S_REG_FWUP_CTRL,txdata[0]);
	if (rc != 0)
	{
		goto error_hand;
	}
	msleep(200);

	rxdata[0] = sem1217_8bit_read(SEM1217S_REG_APP_VER);
	rxdata[1] = sem1217_8bit_read(SEM1217S_REG_APP_VER + 1);
	rxdata[2] = sem1217_8bit_read(SEM1217S_REG_APP_VER + 2);
	rxdata[3] = sem1217_8bit_read(SEM1217S_REG_APP_VER + 3);

	if (o_ctrl->actuator_ois_eeprom_merge_flag)
	{
		mutex_unlock(o_ctrl->actuator_ois_eeprom_merge_mutex);
	}

	updated_ver = *(uint32_t *)rxdata;
	CAM_ERR(CAM_OIS, "updated_ver: 0x%x, new_fw_ver: 0x%x", updated_ver, new_fw_ver);
	if (updated_ver != new_fw_ver)
	{
		CAM_ERR(CAM_OIS, "update fw failed, update version is not equal with read");
		return -1;
	}
	return rc;

error_hand:
	mutex_unlock(o_ctrl->actuator_ois_eeprom_merge_mutex);
	CAM_ERR(CAM_OIS, "update fw failed, rc: %d", rc);
	return rc;
}

