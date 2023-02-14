#include <linux/kfifo.h>
#include <asm/arch_timer.h>
#include "fw_download_interface.h"
#include "LC898124/Ois.h"
#include "linux/proc_fs.h"
#include "BU24721/bu24721_fw.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
#ifndef _STRUCT_TIMESPEC
struct timeval {
	long tv_sec;
	long tv_usec;
};
#endif
#endif


extern unsigned char SelectDownload(uint8_t GyroSelect,
	uint8_t ActSelect,
	uint8_t MasterSlave,
	uint8_t FWType);
extern uint8_t FlashDownload128( uint8_t ModuleVendor, uint8_t ActVer, uint8_t MasterSlave, uint8_t FWType);
extern uint8_t FlashProgram129(uint8_t ModuleVendor,
	uint8_t ActVer,
	struct cam_ois_ctrl_t *o_ctrl );
extern uint8_t WrGyroGainData( uint8_t UcMode );
extern uint8_t WrGyroGainData_LS( uint8_t UcMode );
extern uint8_t WrGyAcOffsetData( uint8_t UcMode );
extern uint32_t MeasGyAcOffset(  void  );
extern uint32_t MeasGyAcOffset(  void  );
extern void Gyro_gain_set(OIS_UWORD X_gain, OIS_UWORD Y_gain);

extern void Update_Gyro_offset_gain_cal_from_flash(void);


#define MAX_DATA_NUM 64
extern bool chip_version_old;

struct mutex ois_mutex;
struct cam_ois_ctrl_t *ois_ctrl = NULL;
struct cam_ois_ctrl_t *ois_ctrls[CAM_OIS_TYPE_MAX] = {NULL};
enum cam_ois_state_vendor ois_state[CAM_OIS_TYPE_MAX] = {0};
struct proc_dir_entry *face_common_dir = NULL;
struct proc_dir_entry *proc_file_entry = NULL;
struct proc_dir_entry *proc_file_entry_tele = NULL;

#define OIS_REGISTER_SIZE 100
#define OIS_READ_REGISTER_DELAY 10
#define COMMAND_SIZE 255
static struct kobject *cam_ois_kobj;
bool dump_ois_registers = false;
uint32_t ois_registers_124[OIS_REGISTER_SIZE][2] = {
	{0xF010, 0x0000},//Servo On/Off
	{0xF012, 0x0000},//Enable/Disable OIS
	{0xF013, 0x0000},//OIS Mode
	{0xF015, 0x0000},//Select Gyro vendor
	{0x82B8, 0x0000},//Gyro Gain X
	{0x8318, 0x0000},//Gyro Gain Y
	{0x0338, 0x0000},//Gyro Offset X
	{0x033c, 0x0000},//Gyro Offset Y
	{0x01C0, 0x0000},//Hall Offset X
	{0x0214, 0x0000},//Hall Offset Y
	{0x0310, 0x0000},//Gyro Raw Data X
	{0x0314, 0x0000},//Gyro Raw Data Y
	{0x0268, 0x0000},//Hall Raw Data X
	{0x026C, 0x0000},//Hall Raw Data Y
	{0xF100, 0x0000},//OIS status
	{0xF112, 0x0000},//spi status
	{0x8000, 0x0000},
	{0x8004, 0x0000},//fw version
	{0x8880, 0x0000},
	{0x82A0, 0x0000},
	{0x8300, 0x0000},
	{0x02E8, 0x0000},
	{0x030C, 0x0000},
	{0x8884, 0x0000},
	{0x8888, 0x0000},//slow pan/tilt parameter.
	{0x84F8, 0x0000},//pan/tilt parameter
	{0x035C, 0x0000},
	{0x8360, 0x0000},
	{0x889C, 0x0000},
    {0x82E8, 0x0000},
    {0x82E0, 0x0000},
    {0x01c4, 0x0000},
    {0x0218, 0x0000},
	{0x0000, 0x0000},
};

uint32_t ois_registers_128[OIS_REGISTER_SIZE][2] = {
	{0xF010, 0x0000},//Servo On/Off
	{0xF018, 0x0000},//Damping detection On/Off
	{0xF012, 0x0000},//Enable/Disable OIS
	{0xF013, 0x0000},//OIS Mode
	{0xF015, 0x0000},//Select Gyro vendor
	{0x82B8, 0x0000},//Gyro Gain X
	{0x8318, 0x0000},//Gyro Gain Y
	{0x0240, 0x0000},//Gyro Offset X
	{0x0244, 0x0000},//Gyro Offset Y
	{0x00D8, 0x0000},//Hall Offset X
	{0x0128, 0x0000},//Hall Offset Y
	{0x0220, 0x0000},//Gyro Raw Data X
	{0x0224, 0x0000},//Gyro Raw Data Y
	{0x0178, 0x0000},//Hall Raw Data X
	{0x017C, 0x0000},//Hall Raw Data Y
	{0xF01D, 0x0000},//SPI IF read access command
	{0xF01E, 0x0000},//SPI IF Write access command
	{0xF100, 0x0000},//OIS status
	{0x01E0, 0x0000},
	{0x0204, 0x0000},
	{0x82B4, 0x0000},
	{0x8314, 0x0000},
	{0x0248, 0x0000},
	{0x024C, 0x0000},//Gyro limit X/Y
	{0x82A0, 0x0000},
	{0x8300, 0x0000},//Gyro coeff3 X/Y
	{0x82BC, 0x0000},
	{0x831C, 0x0000},//Gyro coeff4 X/Y
	{0x0258, 0x0000},//Gyro switch
	{0x01F8, 0x0000},
	{0x021C, 0x0000},//Gyro out X/Y
	{0x06CC, 0x0000},//spi status
	{0x8000, 0x0000},
	{0x8004, 0x0000},//fw version
	{0x0220, 0x0000},
	{0x0224, 0x0000},
	{0x01F8, 0x0000},
	{0x021C, 0x0000},
	{0x82B4, 0x0000},
	{0x8314, 0x0000},
	{0x0000, 0x0000},
};

uint32_t ois_registers_129[OIS_REGISTER_SIZE][2] = {
	{0xF010, 0x0000},//Servo On/Off
	{0xF011, 0x0000},//Damping detection On/Off
	{0xF012, 0x0000},//Enable/Disable OIS
	{0xF013, 0x0000},//OIS Mode
	{0xF015, 0x0000},//Select Gyro vendor
	{0xF016, 0x0000},//Select Gyro vendor
	{0xF01C, 0x0000},//Select Gyro vendor
	{0xF01D, 0x0000},//Select Gyro vendor
	{0xF01E, 0x0000},//Select Gyro vendor
	{0xF01F, 0x0000},//Select Gyro vendor
	{0xF100, 0x0000},//Select Gyro vendor

	{0x0480, 0x0000},//Gyro DC Offset Adj.
	{0x0484, 0x0000},//Gyro DC Offset Adj.
	{0x0628, 0x0000},//Gyro DC Offset Adj.

	{0x06D4, 0x0000},//Accel DC Offset Adj.
	{0x0700, 0x0000},//Accel DC Offset Adj.
	{0x072C, 0x0000},//Accel DC Offset Adj.

	{0x0460, 0x0000},//Input Gyro Raw Data X
	{0x0464, 0x0000},//Input Gyro Raw Data Y
	{0x061C, 0x0000},//Input Gyro Raw Data Z

	{0x06D0, 0x0000},//Input raw Accel
	{0x06FC, 0x0000},//Input raw Accel
	{0x0728, 0x0000},//Input raw Accel

	{0x87D8, 0x0000},//LS Gyro Gain X
	{0x8810, 0x0000},//LS Gyro Gain Y

	{0x8410, 0x0000},//SS Gyro Gain X
	{0x8470, 0x0000},//SS Gyro Gain Y
	{0x8768, 0x0000},//SS Gyro Gain Z

	{0x0268, 0x0000},//Hall-X
	{0x026C, 0x0000},//Hall-Y

	{0x03B8, 0x0000},//Hall-X
	{0x03BC, 0x0000},//Hall-Y
	{0x03C0, 0x0000},//Hall-Z

	{0x01E4, 0x0000},
	{0x0234, 0x0000},//fw version
};

/*
* ois bu24721 info
* {0x000,     0x0000,      0x0000     0x00/0x01}
* reg addr,   reg value,   reg type   read/write
*/

uint32_t ois_registers_bu24721[OIS_REGISTER_SIZE][4] = {

	{0xF000, 0x0000, 0x0001, 0x00},  // OIS IC Info
	{0xF004, 0x0000, 0x0001, 0x00},  // OIS IC Version
	{0xF01C, 0x0000, 0x0004, 0x00},  // OIS Program ID
	{0xF0E2, 0x0000, 0x0002, 0x00},  // OIS Gyro X
	{0xF0E4, 0x0000, 0x0002, 0x00},  // OIS Gyro Y
	{0xF09A, 0x0000, 0x0001, 0x00},  // OIS Sync Enable
	{0xF200, 0x0000, 0x0001, 0x00},  // Read Hall Data
	{0xF0AA, 0x0000, 0x0001, 0x00},  // OIS Centering Amount
	{0xF020, 0x0000, 0x0001, 0x00},  // OIS Control
	{0xF021, 0x0000, 0x0001, 0x00},  // OIS Mode
	{0xF023, 0x0000, 0x0001, 0x00},  // OIS Gyro Mode
	{0xF024, 0x0000, 0x0001, 0x00},  // OIS Status
	{0xF025, 0x0000, 0x0001, 0x00},  // OIS Angle Limit
	{0xF02A, 0x0000, 0x0001, 0x00},  // OIS SPI Mode Select
	{0xF02C, 0x0000, 0x0001, 0x00},  // OIS Gyro Control Address
	{0xF02D, 0x0000, 0x0001, 0x00},  // OIS Gyro Control Data
	{0xF050, 0x0000, 0x0001, 0x00},  // Exit Standy
	{0xF054, 0x0000, 0x0001, 0x00},  // Enter Standy
	{0xF058, 0x0000, 0x0001, 0x00},  // SW RESET
	{0xF07A, 0x0000, 0x0002, 0x00},  // OIS Gyro gain X
	{0xF07C, 0x0000, 0x0002, 0x00},  // OIS Gyro gain Y
	{0xF088, 0x0002, 0x0001, 0x01},  // OIS Gyro Offset Req
	{0xF08A, 0x0000, 0x0002, 0x00},  // OIS Gyro Offset
	{0xF088, 0x0003, 0x0001, 0x01},  // OIS Gyro Offset Req
	{0xF08A, 0x0000, 0x0002, 0x00},  // OIS Gyro Offset
	{0xF097, 0x0000, 0x0001, 0x00},  // Pre SW RESET
	{0xF09A, 0x0000, 0x0001, 0x00},  // OIS Sync Centering Enable
	{0xF09C, 0x0000, 0x0001, 0x00},  // OIS Gyro Offset Update/Diff Req
	{0xF09D, 0x0000, 0x0002, 0x00},  // OIS Gyro Offset Update/Diff
	{0xF0AA, 0x0000, 0x0001, 0x00},  // OIS Sync Centering Ratio
	{0xF18E, 0x0000, 0x0001, 0x00},  // OIS Pan/Tilt Setting
	{0xF1E2, 0x0000, 0x0002, 0x00},  // OIS Gyro Calib Flash Addr
	{0xF1E4, 0x0000, 0x0001, 0x00},  // OIS Gyro Calib Update

	{0xF0E4, 0x0000, 0x0002, 0x00},  // OIS Gyro X L
	{0xF0E5, 0x0000, 0x0002, 0x00},  // OIS Gyro X H
	{0xF0E2, 0x0000, 0x0002, 0x00},  // OIS Gyro Y L
	{0xF0E3, 0x0000, 0x0002, 0x00},  // OIS Gyro Y H

	{0xF060, 0x00,   0x0001, 0x01},  // OIS HALL
	{0xF062, 0x0000, 0x0002, 0x00},  // OIS HALL X
	{0xF060, 0x01,   0x0001, 0x01},  // OIS HAL
	{0xF062, 0x0000, 0x0002, 0x00},  // OIS HALL Y
	{0xF15E, 0x00,   0x0001, 0x01},  // OIS gyro target
	{0xF15C, 0x0000, 0x0002, 0x00},  // OIS gyro target X
	{0xF15E, 0x01,   0x0001, 0x01},  // OIS gyro target
	{0xF15C, 0x0000, 0x0002, 0x00},  // OIS gyro target Y

};


int RamWrite32A(    uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_ois_ctrl_t *o_ctrl = ois_ctrl;

	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x00,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD,
		.delay = 0x00,
	};

	if (o_ctrl == NULL)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,write 0x%04x failed, retry:%d",
				o_ctrl->ois_type,
				addr,
				i+1);
		}
		else
		{
			return rc;
		}
	}
	return rc;
}

int RamRead32A(    uint32_t addr, uint32_t* data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_ois_ctrl_t *o_ctrl = ois_ctrl;

	if (o_ctrl == NULL)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD, false);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,read 0x%04x failed, retry:%d",
				o_ctrl->ois_type,
				addr,
				i+1);
		}
		else
		{
			return rc;
		}
	}
	return rc;
}

static ssize_t ois_read(struct file *p_file,
	char __user *puser_buf, size_t count, loff_t *p_offset)
{
    return 1;
}

static ssize_t ois_write(struct file *p_file,
	const char __user *puser_buf,
	size_t count, loff_t *p_offset)
{
	char data[COMMAND_SIZE] = {0};
	char* const delim = " ";
	int iIndex = 0;
	char *token = NULL, *cur = NULL;
	uint32_t addr =0, value = 0;
	int result = 0;

	if(puser_buf)
	{
		if (count >= COMMAND_SIZE || copy_from_user(&data, puser_buf, count))
		{
			CAM_ERR(CAM_OIS, "copy from user buffer error");
			return -EFAULT;
		}
	}

	cur = data;
	while ((token = strsep(&cur, delim)))
	{
		//CAM_ERR(CAM_OIS, "string = %s iIndex = %d, count = %d", token, iIndex, count);
                int ret=0;
		if (iIndex  == 0)
		{
		    ret = kstrtouint(token, 16, &addr);
		}
		else if (iIndex == 1)
		{
		    ret = kstrtouint(token, 16, &value);
		}
		if(ret < 0)
		{
			CAM_ERR(CAM_OIS,"String conversion to unsigned int failed");
		}
		iIndex++;
	}
	if (ois_ctrls[CAM_OIS_MASTER] && addr != 0)
	{
		result = RamWrite32A_oneplus(ois_ctrls[CAM_OIS_MASTER], addr, value);
		if (result < 0)
		{
			CAM_ERR(CAM_OIS, "write addr = 0x%x, value = 0x%x fail", addr, value);
		}
		else
		{
			CAM_INFO(CAM_OIS, "write addr = 0x%x, value = 0x%x success", addr, value);
		}
	}
	return count;
}

static ssize_t ois_read_tele(struct file *p_file,
	char __user *puser_buf, size_t count, loff_t *p_offset)
{
    return 1;
}

static ssize_t ois_write_tele(struct file *p_file,
	const char __user *puser_buf,
	size_t count, loff_t *p_offset)
{
	char data[COMMAND_SIZE] = {0};
	char* const delim = " ";
	int iIndex = 0;
	char *token = NULL, *cur = NULL;
	uint32_t addr =0, value = 0;
	int result = 0;

	if(puser_buf)
	{
		if (count >= COMMAND_SIZE || copy_from_user(&data, puser_buf, count))
		{
			CAM_ERR(CAM_OIS, "copy from user buffer error");
			return -EFAULT;
		}
	}

	cur = data;
	while ((token = strsep(&cur, delim)))
	{
		//CAM_ERR(CAM_OIS, "string = %s iIndex = %d, count = %d", token, iIndex, count);
		int ret=0;
		if (iIndex  == 0)
		{
			ret = kstrtouint(token, 16, &addr);
		}
		else if (iIndex == 1)
		{
			ret = kstrtouint(token, 16, &value);
		}

		if(ret < 0)
		{
			CAM_ERR(CAM_OIS,"String conversion to unsigned int failed");
		}
		iIndex++;
	}

	if (ois_ctrls[CAM_OIS_SLAVE] && addr != 0)
	{
		result = RamWrite32A_oneplus(ois_ctrls[CAM_OIS_SLAVE], addr, value);
		if (result < 0)
		{
			CAM_ERR(CAM_OIS, "write addr = 0x%x, value = 0x%x fail", addr, value);
		}
		else
		{
			CAM_INFO(CAM_OIS, "write addr = 0x%x, value = 0x%x success", addr, value);
		}
	}
	return count;
}

static const struct proc_ops proc_file_fops = {
	.proc_read  = ois_read,
	.proc_write = ois_write,
};
static const struct proc_ops proc_file_fops_tele = {
	.proc_read  = ois_read_tele,
	.proc_write = ois_write_tele,
};

int ois_start_read(void *arg, bool start)
{
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;

	if (!o_ctrl)
	{
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	mutex_lock(&(o_ctrl->ois_read_mutex));
	o_ctrl->ois_read_thread_start_to_read = start;
	mutex_unlock(&(o_ctrl->ois_read_mutex));

	msleep(OIS_READ_REGISTER_DELAY);

	return 0;
}

int ois_read_thread(void *arg)
{
	int rc = 0;
	int i;
	char buf[OIS_REGISTER_SIZE*2*4 + 1] = {0};  // Add one more bytes to prevent out-of-bounds access
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
	int ois_reg_type = 0;

	if (!o_ctrl)
	{
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	CAM_ERR(CAM_OIS, "ois_read_thread created");

	msleep(500);

	while (!kthread_should_stop())
	{
		memset(buf, 0, sizeof(buf));
		mutex_lock(&(o_ctrl->ois_read_mutex));
		if (o_ctrl->ois_read_thread_start_to_read)
		{
			if (strstr(o_ctrl->ois_name, "124"))
			{
				for (i = 0; i < OIS_REGISTER_SIZE; i++)
				{
					if (ois_registers_124[i][0])
					{
						ois_registers_124[i][1] = 0;
						camera_io_dev_read(&(o_ctrl->io_master_info),
							(uint32_t)ois_registers_124[i][0],
							(uint32_t *)&ois_registers_124[i][1],
							CAMERA_SENSOR_I2C_TYPE_WORD,
							CAMERA_SENSOR_I2C_TYPE_DWORD, false);
					}
				}

				for (i = 0; i < OIS_REGISTER_SIZE; i++)
				{
					if (ois_registers_124[i][0])
					{
						snprintf(buf+strlen(buf), sizeof(buf), "0x%x,0x%x,",
							ois_registers_124[i][0],
							ois_registers_124[i][1]);
					}
				}
			}
			else if (strstr(o_ctrl->ois_name, "128"))
			{
				for (i = 0; i < OIS_REGISTER_SIZE; i++)
				{
					if (ois_registers_128[i][0])
					{
						ois_registers_128[i][1] = 0;
						camera_io_dev_read(&(o_ctrl->io_master_info),
							(uint32_t)ois_registers_128[i][0],
							(uint32_t *)&ois_registers_128[i][1],
							CAMERA_SENSOR_I2C_TYPE_WORD,
							CAMERA_SENSOR_I2C_TYPE_DWORD, false);
					}
				}

				for (i = 0; i < OIS_REGISTER_SIZE; i++)
				{
					if (ois_registers_128[i][0])
					{
						snprintf(buf+strlen(buf), sizeof(buf), "0x%x,0x%x,",
							ois_registers_128[i][0],
							ois_registers_128[i][1]);
					}
				}
			}
			else if (strstr(o_ctrl->ois_name, "129"))
			{
				for (i = 0; i < OIS_REGISTER_SIZE; i++)
				{
					if (ois_registers_129[i][0])
					{
						ois_registers_129[i][1] = 0;
						RamRead32A(ois_registers_129[i][0], & ois_registers_129[i][1] );
						//camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)ois_registers_129[i][0], (uint32_t *)&ois_registers_129[i][1],
						                   //CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
					}
				}

				for (i = 0; i < OIS_REGISTER_SIZE; i++)
				{
					if (ois_registers_129[i][0])
					{
						snprintf(buf+strlen(buf), sizeof(buf), "0x%x,0x%x,",
							ois_registers_129[i][0],
							ois_registers_129[i][1]);
					}
				}
			}
			else if (strstr(o_ctrl->ois_name, "bu24721_tele"))
			{
				for (i = 0; i < OIS_REGISTER_SIZE; i++) {
					if (ois_registers_bu24721[i][0]) {
						if(ois_registers_bu24721[i][2] != 0){
							ois_reg_type = ois_registers_bu24721[i][2];
						}
						if(0x00 == ois_registers_bu24721[i][3])
						{
							ois_registers_bu24721[i][1] = 0;
							rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)ois_registers_bu24721[i][0], (uint32_t *)&ois_registers_bu24721[i][1],
							                   CAMERA_SENSOR_I2C_TYPE_WORD, ois_reg_type, false);
						}
						else if(0x01 == ois_registers_bu24721[i][3])  //write
						{
							rc = I2C_OIS_8bit_write(ois_registers_bu24721[i][0],ois_registers_bu24721[i][1]);
							IsOISReady(o_ctrl);
						}
						if(rc < 0) {
							CAM_ERR(CAM_OIS, " ois name:%s rc: %s", o_ctrl->ois_name, rc);
						}
					}
				}

				for (i = 0; i < OIS_REGISTER_SIZE; i++) {
					if (ois_registers_bu24721[i][0]) {
						snprintf(buf+strlen(buf), sizeof(buf), "0x%x,0x%x,", ois_registers_bu24721[i][0], ois_registers_bu24721[i][1]);
					}
				}
			}
			CAM_ERR(CAM_OIS, "%s OIS register data: %s", o_ctrl->ois_name, buf);
		}
		mutex_unlock(&(o_ctrl->ois_read_mutex));

		msleep(OIS_READ_REGISTER_DELAY);
	}

	CAM_ERR(CAM_OIS, "ois_read_thread exist");

	return rc;
}

int ois_start_read_thread(void *arg, bool start)
{
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;

	if (!o_ctrl)
	{
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
		return -1;
	}

	if (start)
	{
		if (o_ctrl->ois_read_thread)
		{
			CAM_ERR(CAM_OIS, "ois_read_thread is already created, no need to create again.");
		}
		else
		{
			o_ctrl->ois_read_thread = kthread_run(ois_read_thread, o_ctrl, o_ctrl->ois_name);
			if (!o_ctrl->ois_read_thread)
			{
				CAM_ERR(CAM_OIS, "create ois read thread failed");
				mutex_unlock(&(o_ctrl->ois_read_mutex));
				return -2;
			}
		}
	}
	else
	{
		if (o_ctrl->ois_read_thread)
		{
			mutex_lock(&(o_ctrl->ois_read_mutex));
			o_ctrl->ois_read_thread_start_to_read = 0;
			mutex_unlock(&(o_ctrl->ois_read_mutex));
			kthread_stop(o_ctrl->ois_read_thread);
			o_ctrl->ois_read_thread = NULL;
		}
		else
		{
			CAM_ERR(CAM_OIS, "ois_read_thread is already stopped, no need to stop again.");
		}
	}

	return 0;
}

#define OIS_ATTR(_name, _mode, _show, _store) \
    struct kobj_attribute ois_attr_##_name = __ATTR(_name, _mode, _show, _store)

static ssize_t dump_registers_store(struct kobject *kobj,
                             struct kobj_attribute * attr,
                             const char * buf,
                             size_t count)
{
        unsigned int if_start_thread;
        CAM_WARN(CAM_OIS, "%s", buf);
        if (sscanf(buf, "%u", &if_start_thread) != 1)
		{
                return -1;
        }

        if(if_start_thread)
		{
                dump_ois_registers = true;
        }
		else
		{
                dump_ois_registers = false;
        }

        return count;
}

static OIS_ATTR(dump_registers, 0644, NULL, dump_registers_store);
static struct attribute *ois_node_attrs[] = {
        &ois_attr_dump_registers.attr,
        NULL,
};
static const struct attribute_group ois_common_group = {
        .attrs = ois_node_attrs,
};
static const struct attribute_group *ois_groups[] = {
        &ois_common_group,
        NULL,
};

void WitTim( uint16_t time)
{
	msleep(time);
}
void setI2cSlvAddr( uint8_t a , uint8_t b)
{
	CAM_ERR(CAM_OIS, "setI2cSlvAddr a:%d b:%d",a,b);
}

void CntRd(uint32_t addr, void *data, uint16_t size)
{
	int i = 0;
	int32_t rc = 0;
	int retry = 3;
	struct cam_ois_ctrl_t *o_ctrl = ois_ctrl;

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info), addr, (uint8_t *)data,
		                            CAMERA_SENSOR_I2C_TYPE_WORD,
		                            CAMERA_SENSOR_I2C_TYPE_BYTE,
		                            size);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,Continue read failed, rc:%d, retry:%d",o_ctrl->ois_type, rc, i+1);
		}
		else
		{
			break;
		}
	}
}

void CntWrt(  void *register_data, uint16_t size)
{
	uint8_t *data = (uint8_t *)register_data;
	int32_t rc = 0;
	int i = 0;
	int reg_data_cnt = size - 1;
	int continue_cnt = 0;
	int retry = 3;
	static struct cam_sensor_i2c_reg_array *i2c_write_setting_gl = NULL;

	struct cam_ois_ctrl_t *o_ctrl = ois_ctrl;

	struct cam_sensor_i2c_reg_setting i2c_write;

	if (o_ctrl == NULL)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return;
	}

	if (i2c_write_setting_gl == NULL)
	{
		i2c_write_setting_gl = (struct cam_sensor_i2c_reg_array *)kzalloc(
		                           sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2, GFP_KERNEL);
		if(!i2c_write_setting_gl)
		{
			CAM_ERR(CAM_OIS, "Alloc i2c_write_setting_gl failed");
			return;
		}
	}

	memset(i2c_write_setting_gl, 0, sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2);

	for(i = 0; i< reg_data_cnt; i++)
	{
		if (i == 0)
		{
			i2c_write_setting_gl[continue_cnt].reg_addr = data[0];
			i2c_write_setting_gl[continue_cnt].reg_data = data[1];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
		}
		else
		{
			i2c_write_setting_gl[continue_cnt].reg_data = data[i+1];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
		}
		continue_cnt++;
	}
	i2c_write.reg_setting = i2c_write_setting_gl;
	i2c_write.size = continue_cnt;
	i2c_write.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_write.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_write.delay = 0x00;

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		                                    &i2c_write, 1);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,Continue write failed, rc:%d, retry:%d",o_ctrl->ois_type, rc, i+1);
		}
		else
		{
			break;
		}
	}
}


// int RamWrite32A(    uint32_t addr, uint32_t data)
// {
// 	int32_t rc = 0;
// 	int retry = 3;
// 	int i;

// 	struct cam_ois_ctrl_t *o_ctrl = ois_ctrl;

// 	struct cam_sensor_i2c_reg_array i2c_write_setting = {
// 		.reg_addr = addr,
// 		.reg_data = data,
// 		.delay = 0x00,
// 		.data_mask = 0x00,
// 	};
// 	struct cam_sensor_i2c_reg_setting i2c_write = {
// 		.reg_setting = &i2c_write_setting,
// 		.size = 1,
// 		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
// 		.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD,
// 		.delay = 0x00,
// 	};

// 	if (o_ctrl == NULL) {
// 		CAM_ERR(CAM_OIS, "Invalid Args");
// 		return -EINVAL;
// 	}

// 	for(i = 0; i < retry; i++) {
// 		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
// 		if (rc < 0) {
// 			CAM_ERR(CAM_OIS, "ois type=%d,write 0x%04x failed, retry:%d",o_ctrl->ois_type, addr, i+1);
// 		} else {
// 			return rc;
// 		}
// 	}
// 	return rc;
// }

// int RamRead32A(    uint32_t addr, uint32_t* data)
// {
// 	int32_t rc = 0;
// 	int retry = 3;
// 	int i;

// 	struct cam_ois_ctrl_t *o_ctrl = ois_ctrl;

// 	if (o_ctrl == NULL) {
// 		CAM_ERR(CAM_OIS, "Invalid Args");
// 		return -EINVAL;
// 	}
// 	for(i = 0; i < retry; i++) {
// 		rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
// 		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
// 		if (rc < 0) {
// 			CAM_ERR(CAM_OIS, "ois type=%d,read 0x%04x failed, retry:%d",o_ctrl->ois_type, addr, i+1);
// 		} else {
// 			return rc;
// 		}
// 	}
// 	return rc;
// }

int RamWrite32A_oneplus(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x00,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD,
		.delay = 0x00,
	};

	if (o_ctrl == NULL)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,write 0x%04x failed, retry:%d",o_ctrl->ois_type, addr, i+1);
		}
		else
		{
			return rc;
		}
	}
	return rc;
}

int RamRead32A_oneplus(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	if (o_ctrl == NULL)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD, false);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,read 0x%04x failed, retry:%d",o_ctrl->ois_type, addr, i+1);
		}
		else
		{
			return rc;
		}
	}
	return rc;
}

int RohmOisWrite(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x01,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.delay = 0x00,
	};

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "ois type=%d,write 0x%04x failed, retry:%d",o_ctrl->ois_type, addr, i+1);
		} else {

        CAM_ERR(CAM_OIS, "write cci device=%d master=%d",o_ctrl->io_master_info.cci_client->cci_device,o_ctrl->io_master_info.cci_client->cci_i2c_master);
            CAM_ERR(CAM_OIS, "write success ois type=%d,write 0x%04x ,data=%d",o_ctrl->ois_type, addr,data);
			return rc;
		}
	}
	return rc;
}

int RohmOisRead(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

//    o_ctrl->io_master_info.cci_client->sid = 0x7C;
	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE, false);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "ois type=%d,read 0x%04x failed, retry:%d",o_ctrl->ois_type, addr, i+1);
			CAM_ERR(CAM_OIS, "cci device=%d master=%d sid=0x%x",o_ctrl->io_master_info.cci_client->cci_device,o_ctrl->io_master_info.cci_client->cci_i2c_master, o_ctrl->io_master_info.cci_client->sid);
		} else {
		    CAM_ERR(CAM_OIS, "cci device=%d master=%d sid=0x%x",o_ctrl->io_master_info.cci_client->cci_device,o_ctrl->io_master_info.cci_client->cci_i2c_master, o_ctrl->io_master_info.cci_client->sid);
            CAM_ERR(CAM_OIS, "read success ois type=%d,read 0x%04x data=%d",o_ctrl->ois_type, addr,*data);
			return rc;
		}
	}
//	o_ctrl->io_master_info.cci_client->sid = 0x3e;
	return rc;
}


void OISCountinueRead(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, void *data, uint16_t size)
{
	int i = 0;
	int32_t rc = 0;
	int retry = 3;

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info), addr, (uint8_t *)data,
		                            CAMERA_SENSOR_I2C_TYPE_WORD,
		                            CAMERA_SENSOR_I2C_TYPE_WORD,
		                            size);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,Continue read failed, rc:%d, retry:%d",o_ctrl->ois_type, rc, i+1);
		}
		else
		{
			break;
		}
	}
}

void OISCountinueWrite(  struct cam_ois_ctrl_t *o_ctrl, void *register_data, uint16_t size)
{
	uint32_t *data = (uint32_t *)register_data;
	int32_t rc = 0;
	int i = 0;
	int reg_data_cnt = size - 1;
	int continue_cnt = 0;
	int retry = 3;
	static struct cam_sensor_i2c_reg_array *i2c_write_setting_gl = NULL;

	struct cam_sensor_i2c_reg_setting i2c_write;

	if (o_ctrl == NULL)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return;
	}

	if (i2c_write_setting_gl == NULL)
	{
		i2c_write_setting_gl = (struct cam_sensor_i2c_reg_array *)kzalloc(
		                           sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2, GFP_KERNEL);
		if(!i2c_write_setting_gl)
		{
			CAM_ERR(CAM_OIS, "Alloc i2c_write_setting_gl failed");
			return;
		}
	}

	memset(i2c_write_setting_gl, 0, sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2);

	for(i = 0; i< reg_data_cnt; i++)
	{
		if (i == 0)
		{
			i2c_write_setting_gl[continue_cnt].reg_addr = data[0];
			i2c_write_setting_gl[continue_cnt].reg_data = data[1];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
		}
		else
		{
			i2c_write_setting_gl[continue_cnt].reg_data = data[i+1];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
		}
		continue_cnt++;
	}
	i2c_write.reg_setting = i2c_write_setting_gl;
	i2c_write.size = continue_cnt;
	i2c_write.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_write.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	i2c_write.delay = 0x00;

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		                                    &i2c_write, 1);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,Continue write failed, rc:%d, retry:%d",o_ctrl->ois_type, rc, i+1);
		}
		else
		{
			break;
		}
	}
	kfree(i2c_write_setting_gl);
}

int OISWrite(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x00,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD,
		.delay = 0x00,
	};

	if (o_ctrl == NULL)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,write 0x%04x failed, retry:%d",o_ctrl->ois_type, addr, i+1);
		}
		else
		{
			return rc;
		}
	}
	return rc;
}

int OISRead(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t* data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	if (o_ctrl == NULL)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), (uint32_t)addr, (uint32_t *)data,
		                        CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD, false);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,read 0x%04x failed, retry:%d",o_ctrl->ois_type, addr, i+1);
		}
		else
		{
			return rc;
		}
	}
	return rc;
}

void Set124Or128GyroAccelCoef(struct cam_ois_ctrl_t *o_ctrl)
{
	CAM_ERR(CAM_OIS, "SetGyroAccelCoef SelectAct 0x%x GyroPostion 0x%x",
		o_ctrl->ois_actuator_vendor,
		o_ctrl->ois_gyro_position);

	if (strstr(o_ctrl->ois_name, "124"))
	{
		if(o_ctrl->ois_gyro_position==3)
		{
			RamWrite32A( GCNV_XX, (UINT32) 0x00000000);
			RamWrite32A( GCNV_XY, (UINT32) 0x80000001);
			RamWrite32A( GCNV_YY, (UINT32) 0x00000000);
			RamWrite32A( GCNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( GCNV_ZP, (UINT32) 0x7FFFFFFF);

			RamWrite32A( ACNV_XX, (UINT32) 0x00000000);
			RamWrite32A( ACNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_YY, (UINT32) 0x00000000);
			RamWrite32A( ACNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_ZP, (UINT32) 0x80000001);
		}
		else if(o_ctrl->ois_gyro_position==2)
		{
			RamWrite32A( GCNV_XX, (UINT32) 0x00000000);
			RamWrite32A( GCNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A( GCNV_YY, (UINT32) 0x00000000);
			RamWrite32A( GCNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( GCNV_ZP, (UINT32) 0x7FFFFFFF);

			RamWrite32A( ACNV_XX, (UINT32) 0x00000000);
			RamWrite32A( ACNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_YY, (UINT32) 0x00000000);
			RamWrite32A( ACNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_ZP, (UINT32) 0x80000001);
		}
		else if(o_ctrl->ois_gyro_position==4)
		{
			RamWrite32A( GCNV_XX, (UINT32) 0x00000000);
			RamWrite32A( GCNV_XY, (UINT32) 0x80000001);
			RamWrite32A( GCNV_YY, (UINT32) 0x00000000);
			RamWrite32A( GCNV_YX, (UINT32) 0x80000001);
			RamWrite32A( GCNV_ZP, (UINT32) 0x7FFFFFFF);

			RamWrite32A( ACNV_XX, (UINT32) 0x00000000);
			RamWrite32A( ACNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_YY, (UINT32) 0x00000000);
			RamWrite32A( ACNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( ACNV_ZP, (UINT32) 0x80000001);
		}
		else if(o_ctrl->ois_gyro_position==5)
		{
			RamWrite32A( GCNV_XX, (UINT32) 0x00000000);
			RamWrite32A( GCNV_XY, (UINT32) 0x7FFFFFFF);
			RamWrite32A( GCNV_YY, (UINT32) 0x00000000);
			RamWrite32A( GCNV_YX, (UINT32) 0x7FFFFFFF);
			RamWrite32A( GCNV_ZP, (UINT32) 0x80000001);

			RamWrite32A( ACNV_XX, (UINT32) 0x00000000);
			RamWrite32A( ACNV_XY, (UINT32) 0x80000001 );
			RamWrite32A( ACNV_YY, (UINT32) 0x00000000);
			RamWrite32A( ACNV_YX, (UINT32) 0x80000001 );
			RamWrite32A( ACNV_ZP, (UINT32) 0x80000001);
		}
	}
	else if (strstr(o_ctrl->ois_name, "128"))
	{

	}
}

int StoreOisGyroGian(struct cam_ois_ctrl_t *o_ctrl)
{
	unsigned char rc = 0;
	ois_ctrl = o_ctrl;

	if (strstr(o_ctrl->ois_name, "129"))
	{
		CAM_ERR(CAM_OIS, "Entry 129 StoreOisGyroGian");

		rc = WrGyroGainData(1);
		if(rc != 0)
		{
			CAM_ERR(CAM_OIS, " 129 StoreOisGyroGian WrGyroGainData_SS failed");
		}
		msleep(10);
		WrGyroGainData_LS(1);
		if(rc != 0)
		{
			CAM_ERR(CAM_OIS, " 129 StoreOisGyroGian WrGyroGainData_LS failed");
		}
		msleep(10);
	}
	else if(strstr(o_ctrl->ois_name, "imx766_bu24721_tele"))
	{
		WriteGyroGainToFlash();
	}
	return rc;
}

int WriteOisGyroGianToRam(struct cam_ois_ctrl_t *o_ctrl,DUAL_OIS_CALI_RESULTS* current_gyro_gain)
{
	int rc = 0;
	uint32_t SsGyroA , SsGyroB , SsGyroC;
	uint32_t LsGyroX , LsGyroY;
	ois_ctrl = o_ctrl;

	CAM_ERR(CAM_OIS,"Engineer Camera set LsGyroGain_X = 0x%x LsGyroGain_Y:0x%x"
			"SsGyroGain_A:0x%x SsGyroGain_B:0x%x SsGyroGain_C:0x%x"
			"mode:%d successed:%d",
			current_gyro_gain->LsGyroGain_X,
			current_gyro_gain->LsGyroGain_Y,
			current_gyro_gain->SsGyroGain_A,
			current_gyro_gain->SsGyroGain_B,
			current_gyro_gain->SsGyroGain_C,
			current_gyro_gain->mode,
			current_gyro_gain->successed);

	switch(current_gyro_gain->mode)
	{
		case 1 :
			RamRead32A(0x87D8, & LsGyroX );
			RamRead32A(0x8810, & LsGyroY );
			CAM_ERR(CAM_OIS, "before write ls_gyro_gain_x = 0x%x ls_gyro_gain_y = 0x%x ,%f ,%f",
				LsGyroX,
				LsGyroY,
				LsGyroX,
				LsGyroY);

			RamWrite32A(0x87D8, (uint32_t)current_gyro_gain->LsGyroGain_X);
			RamWrite32A(0x8810, (uint32_t)current_gyro_gain->LsGyroGain_Y);

			RamRead32A(0x87D8, & LsGyroX );
			RamRead32A(0x8810, & LsGyroY );
			CAM_ERR(CAM_OIS, "after write ls_gyro_gain_x = 0x%x ls_gyro_gain_y = 0x%x ,%f ,%f",
				LsGyroX,
				LsGyroY,
				LsGyroX,
				LsGyroY);

			break;

		case 2 :
			RamRead32A(0x8410, & SsGyroA );
			RamRead32A(0x8470, & SsGyroB );
			CAM_ERR(CAM_OIS, "before write ss_gyro_gain_x = 0x%x ss_gyro_gain_y = 0x%x", SsGyroA, SsGyroB);

			RamWrite32A(0x8410, (uint32_t)current_gyro_gain->SsGyroGain_A);
			RamWrite32A(0x8470, (uint32_t)current_gyro_gain->SsGyroGain_B);

			RamRead32A(0x8410, & SsGyroA );
			RamRead32A(0x8470, & SsGyroB );
			CAM_ERR(CAM_OIS, "after write ss_gyro_gain_x = 0x%x ss_gyro_gain_y = 0x%x", SsGyroA, SsGyroB);

			break;

		case 3 :
			RamRead32A(0x8768, & SsGyroC );
			CAM_ERR(CAM_OIS, "before write ss_gyro_gain_z = 0x%x",SsGyroC);

			RamWrite32A(0x8768, (uint32_t)current_gyro_gain->SsGyroGain_C);

			RamRead32A(0x8768, & SsGyroC );
			CAM_ERR(CAM_OIS, "after write ss_gyro_gain_z = 0x%x",SsGyroC);

			break;

	}

	return rc;
}

int GetDualOisGyroGain(struct cam_ois_ctrl_t *o_ctrl,DUAL_OIS_GYROGAIN* initial_gyro_gain)
{
	int rc = 0;


	if (!o_ctrl)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if(strstr(o_ctrl->ois_name, "bu24721"))
	{
		CAM_INFO(CAM_OIS, "this is bu24721,no need get gyro gain!");
		return rc;
	}

	ois_ctrl = o_ctrl;

	RamRead32A(0x87D8,&initial_gyro_gain->LsGyroGain_X);
	RamRead32A(0x8810,&initial_gyro_gain->LsGyroGain_Y);
	CAM_INFO(CAM_OIS, "LS: Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x",
		initial_gyro_gain->LsGyroGain_X,
		initial_gyro_gain->LsGyroGain_Y);

	RamRead32A(0x8410,&initial_gyro_gain->SsGyroGain_A );
	RamRead32A(0x8470,&initial_gyro_gain->SsGyroGain_B );
	RamRead32A(0x8768,&initial_gyro_gain->SsGyroGain_C );
	CAM_INFO(CAM_OIS, "SS: Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x, Gyro_gain_Z:0x%x",
		initial_gyro_gain->SsGyroGain_A,
		initial_gyro_gain->SsGyroGain_B,
		initial_gyro_gain->SsGyroGain_C);

	return rc;
}

int QueryDualOisSmaWireStatus(struct cam_ois_ctrl_t *o_ctrl, uint32_t *sma_wire_broken)
{
	int rc = 0;
	if(strstr(o_ctrl->ois_name, "bu24721"))
	{
		CAM_ERR(CAM_OIS, "this is bu24721,no need QueryDualOisSmaWireStatus!");
		return rc;
	}

	RamRead32A(0xF121, sma_wire_broken );
	CAM_ERR(CAM_OIS, "QueryDualOisSmaWireStatus sma_wire_broken: %d", sma_wire_broken);
	return rc;
}


int WriteDualOisShiftRegister(struct cam_ois_ctrl_t *o_ctrl, uint32_t distance)
{
	int rc = 0;
	uint32_t value;
	RamRead32A(0x871C, &value);
	CAM_DBG(CAM_OIS, "before write shift register, the value is 0x%x, distance = 0x%x", value, distance);

	RamWrite32A(0x871C, distance);

	RamRead32A(0x871C, &value);
	CAM_DBG(CAM_OIS, "after write shift register, the value is 0x%x", value);
	return rc;
}

int DoDualOisGyroOffset(struct cam_ois_ctrl_t *o_ctrl, uint32_t *gyro_data)
{
	int rc = 0;
	uint32_t ReadGyroOffset_X = 0, ReadGyroOffset_Y = 0, ReadGyroOffset_Z = 0;

	if (!o_ctrl)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_ctrl = o_ctrl;

	RamRead32A(0x0480, & ReadGyroOffset_X );
	RamRead32A(0x0484, & ReadGyroOffset_Y );
	CAM_ERR(CAM_OIS, "before WriteOisGyroOffset GyroOffset_X = 0x%x GyroOffset_Y = 0x%x",
		ReadGyroOffset_X,
		ReadGyroOffset_Y);

	RamRead32A(0x010C, & ReadGyroOffset_X );
	RamRead32A(0x0110, & ReadGyroOffset_Y );
	RamRead32A(0x0114, & ReadGyroOffset_Z );
	CAM_ERR(CAM_OIS, "before WriteOisGyroOffset GyroOffset_X = 0x%x GyroOffset_Y = 0x%x ReadGyroOffset_Z = 0x%x",
		ReadGyroOffset_X,
		ReadGyroOffset_Y,
		ReadGyroOffset_Z);

	rc = MeasGyAcOffset();

	RamRead32A(0x0480, & ReadGyroOffset_X );
	RamRead32A(0x0484, & ReadGyroOffset_Y );
	CAM_ERR(CAM_OIS, "after MeasGyAcOffset ss_gyro_offset_x = 0x%x ss_gyro_offset_y = 0x%x rc:0x%x",
		ReadGyroOffset_X,
		ReadGyroOffset_Y,
		rc);


	rc = WrGyAcOffsetData(1);

	ReadGyroOffset_X = (( ReadGyroOffset_X >> 16) & 0x0000FFFF );
	ReadGyroOffset_Y = (( ReadGyroOffset_Y >> 16) & 0x0000FFFF );
	*gyro_data = ReadGyroOffset_Y << 16 | ReadGyroOffset_X;

	return rc;

}

int DoBU24721GyroOffset(struct cam_ois_ctrl_t *o_ctrl, uint32_t *gyro_data)
{
	int rc = 0;
	uint32_t ReadGyroOffset_X = 0, ReadGyroOffset_Y = 0;
	struct _FACT_ADJ FADJCAL = { 0x0000, 0x0000};

	if (!o_ctrl)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_ctrl = o_ctrl;

	FADJCAL= Gyro_offset_cal();

	ReadGyroOffset_X = (( FADJCAL.gl_GX_OFS >> 16) & 0x0000FFFF );
	ReadGyroOffset_Y = (( FADJCAL.gl_GY_OFS >> 16) & 0x0000FFFF );
	*gyro_data = ReadGyroOffset_Y << 16 | ReadGyroOffset_X;

	return rc;

}

int WriteOisGyroGian(struct cam_ois_ctrl_t *o_ctrl,OIS_GYROGAIN* current_gyro_gain)
{
	int rc = 0;
	if (!o_ctrl)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_ctrl = o_ctrl;
	Gyro_gain_set(current_gyro_gain->GyroGain_X , current_gyro_gain->GyroGain_Y);

	return rc;
}


static int Download124Or128FW(struct cam_ois_ctrl_t *o_ctrl)
{
	#define MAX_RETRY 10
	uint32_t download_retry = 0;
	uint32_t UlReadValX, UlReadValY, UlReadValZ;
	uint32_t spi_type;
	unsigned char rc = 0;
	struct timespec64 mStartTime, mEndTime, diff;
	uint64_t mSpendTime = 0;

	if (!o_ctrl)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_ctrl = o_ctrl;

	ktime_get_real_ts64(&mStartTime);

	CAM_INFO(CAM_OIS, "MasterSlave 0x%x, GyroVendor 0x%x, GyroPosition 0x%x, ModuleVendor 0x%x, ActVer 0x%x, FWType 0x%x ois_name:%s",
	         o_ctrl->ois_type,
			 o_ctrl->ois_gyro_vendor,
			 o_ctrl->ois_gyro_position,
			 o_ctrl->ois_module_vendor,
			 o_ctrl->ois_actuator_vendor,
			 o_ctrl->ois_fw_flag,
			 o_ctrl->ois_name);

	if (strstr(o_ctrl->ois_name, "124"))
	{
		rc = SelectDownload(o_ctrl->ois_gyro_vendor,
			o_ctrl->ois_actuator_vendor,
			o_ctrl->ois_type,
			o_ctrl->ois_fw_flag);

		if (0 == rc)
		{
			Set124Or128GyroAccelCoef(ois_ctrl);

			//remap
			RamWrite32A(0xF000, 0x00000000 );
			//msleep(120);

			//SPI-Master ( Act1 )  Check gyro signal
			RamRead32A(0x061C, & UlReadValX );
			RamRead32A(0x0620, & UlReadValY );
			CAM_INFO(CAM_OIS, "Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);

			spi_type = 0;
			RamRead32A(0x8880, & spi_type );
			CAM_INFO(CAM_OIS, "spi_status:0x%x", spi_type);

			//SPI-Master ( Act1 )  Check gyro gain
			RamRead32A(0x82b8, & UlReadValX );
			RamRead32A(0x8318, & UlReadValY );
			CAM_INFO(CAM_OIS, "Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x", UlReadValX, UlReadValY);

			//SPI-Master ( Act1 )  start gyro signal transfer. ( from Master to slave. )
			if (CAM_OIS_MASTER == o_ctrl->ois_type) {
				RamWrite32A(0x8970, 0x00000001 );
				//msleep(5);
				RamWrite32A(0xf111, 0x00000001 );
				//msleep(5);
			}
		}
		else
		{
			switch (rc)
			{
				case 0x01:
					CAM_ERR(CAM_OIS, "H/W error");
					break;
				case 0x02:
					CAM_ERR(CAM_OIS, "Table Data & Program download verify error");
					break;
				case 0xF0:
					CAM_ERR(CAM_OIS, "Download code select error");
					break;
				case 0xF1:
					CAM_ERR(CAM_OIS, "Download code information read error");
					break;
				case 0xF2:
					CAM_ERR(CAM_OIS, "Download code information disagreement");
					break;
				case 0xF3:
					CAM_ERR(CAM_OIS, "Download code version error");
					break;
				default:
					CAM_ERR(CAM_OIS, "Unkown error code");
					break;
			}
		}
	}
	else if (strstr(o_ctrl->ois_name, "128"))
	{
		rc = FlashDownload128(o_ctrl->ois_module_vendor, o_ctrl->ois_actuator_vendor, o_ctrl->ois_type, o_ctrl->ois_fw_flag);

		while(rc && (download_retry < MAX_RETRY))
		{
			rc = FlashDownload128(o_ctrl->ois_module_vendor, o_ctrl->ois_actuator_vendor, o_ctrl->ois_type, o_ctrl->ois_fw_flag);
			download_retry++;
			CAM_ERR(CAM_OIS, "Error: Download128 Failed: %d. now retry %d !",rc,download_retry);
		}

		if (0 == rc)
		{
			Set124Or128GyroAccelCoef(ois_ctrl);

			//LC898128 don't need to do remap
			//RamWrite32A(0xF000, 0x00000000 );
			//msleep(120);

			//select gyro vendor
			RamWrite32A(0xF015, o_ctrl->ois_gyro_vendor);
			msleep(10);

			//SPI-Master ( Act1 )  Check gyro signal
			RamRead32A(0x0220, & UlReadValX );
			RamRead32A(0x0224, & UlReadValY );
			CAM_INFO(CAM_OIS, "Gyro_X:0x%x, Gyro_Y:0x%x", UlReadValX, UlReadValY);
/*
			spi_type = 0;
			RamRead32A(0xf112, & spi_type );
			CAM_INFO(CAM_OIS, "spi_type:0x%x", spi_type);
*/
			//SPI-Master ( Act1 )  Check gyro gain
			/*RamWrite32A(0x82b8, 0xbe147a00 );
                        RamWrite32A(0x8318, 0xba1cab00 );*/
			RamRead32A(0x82b8, & UlReadValX );
			RamRead32A(0x8318, & UlReadValY );
			CAM_INFO(CAM_OIS, "Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x", UlReadValX, UlReadValY);

			//SPI-Master ( Act1 )  start gyro signal transfer. ( from Master to slave. )
			if (CAM_OIS_MASTER == o_ctrl->ois_type)
			{
				RamWrite32A(0xF017, 0x01);
                                mutex_lock(&(o_ctrl->ois_hall_data_mutex));
                                kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
                                kfifo_reset(&(o_ctrl->ois_hall_data_fifoV2));
                                mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
			}
		}
		else
		{
			switch (rc&0xF0)
			{
				case 0x00:
					CAM_ERR(CAM_OIS, "Error ; during the rom boot changing. Also including 128 power off issue.");
					break;
				case 0x20:
					CAM_ERR(CAM_OIS, "Error ; during Initial program for updating to program memory.");
					break;
				case 0x30:
					CAM_ERR(CAM_OIS, "Error ; during User Mat area erasing.");
					break;
				case 0x40:
					CAM_ERR(CAM_OIS, "Error ; during User Mat area programing.");
					break;
				case 0x50:
					CAM_ERR(CAM_OIS, "Error ; during the verification.");
					break;
				case 0x90:
					CAM_ERR(CAM_OIS, "Error ; during the drive offset confirmation.");
					break;
				case 0xA0:
					CAM_ERR(CAM_OIS, "Error ; during the MAT2 re-write process.");
					break;
				case 0xF0:
					if (rc == 0xF0)
					{
						CAM_ERR(CAM_OIS, "mistake of module vendor designation.");
					}
					else if (rc == 0xF1)
					{
						CAM_ERR(CAM_OIS, "mistake size of From Code.");
					}
					break;
				default:
					CAM_ERR(CAM_OIS, "Unkown error code");
					break;
			}
		}
	}else if (strstr(o_ctrl->ois_name, "129")) {

		//rc = FlashProgram129(o_ctrl->ois_module_vendor, o_ctrl->ois_actuator_vendor , o_ctrl);
		//msleep(40);
		//CAM_INFO(CAM_OIS, "ois FlashProgram129 rc=%d", rc);

		if (0 == rc)
		{

			//select gyro vendor
			RamWrite32A(0xF015, 0x00000000);

			//SPI-Master ( Act1 )  Check gyro signal
			RamRead32A(0x0460, & UlReadValX );
			RamRead32A(0x0464, & UlReadValY );
			RamRead32A(0x061C, & UlReadValZ );
			CAM_INFO(CAM_OIS, "Input_Raw_Gyro_X:0x%x, Input_Raw_Gyro_Y::0x%x, Input_Raw_Gyro_Z:0x%x",
				UlReadValX,
				UlReadValY,
				UlReadValZ);

			RamRead32A(0xF019, & UlReadValX );
			CAM_INFO(CAM_OIS, "0xF019:0x%x", UlReadValX);

			if(UlReadValX == 0)
			{
				RamWrite32A(0xF019, 0x00000001);
				IsOISReady(o_ctrl);
			}


			RamRead32A(0x1A00, & UlReadValX);
			CAM_INFO(CAM_OIS, "0x1A00:0x%x", UlReadValX);
			if(UlReadValX & (0x8000))
			{
				RamWrite32A(0xF010, 0x80000003);
				CAM_INFO(CAM_OIS, "set 0xF010 0x80000003");
				IsOISReady(o_ctrl);
			}

			RamRead32A(0xF010, & UlReadValX );
			CAM_INFO(CAM_OIS, "check 0xF010 servo on(0x00000033) :0x%x", UlReadValX);

			RamRead32A(0x0220, & UlReadValX );
			RamRead32A(0x01D0, & UlReadValY );
			CAM_INFO(CAM_OIS, "LS: Hall-X:0x%x, Hall-Y:0x%x", UlReadValX, UlReadValY);

			RamRead32A(0x87D8, & UlReadValX);
			RamRead32A(0x8810, & UlReadValY);
			CAM_INFO(CAM_OIS, "LS: Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x",
				UlReadValX,
				UlReadValY);

			RamRead32A(0x0350, & UlReadValX );
			RamRead32A(0x03A0, & UlReadValY );
			RamRead32A(0x0B98, & UlReadValZ );
			CAM_INFO(CAM_OIS, "SS: Hall-X:0x%x, Hall-Y:0x%x, Hall-Z:0x%x",
				UlReadValX,
				UlReadValY,
				UlReadValZ);

			RamRead32A(0x8410, & UlReadValX );
			RamRead32A(0x8470, & UlReadValY );
			RamRead32A(0x8768, & UlReadValZ );
			CAM_INFO(CAM_OIS, "SS: Gyro_gain_X:0x%x, Gyro_gain_Y:0x%x, Gyro_gain_Z:0x%x",
				UlReadValX,
				UlReadValY,
				UlReadValZ);

			RamWrite32A(0xF012, 0x00000011);
			IsOISReady(o_ctrl);
			CAM_INFO(CAM_OIS, "set 0xF012 0x00000011");
			RamWrite32A(0xF01C, 0x00000000);
			IsOISReady(o_ctrl);
			CAM_INFO(CAM_OIS, "set 0xF01C 0x00000000");
			RamWrite32A(0xF016, 0x00000000);
			IsOISReady(o_ctrl);
			CAM_INFO(CAM_OIS, "set 0xF016 0x00000000");


			//SPI-Master ( Act1 )  start gyro signal transfer. ( from Master to slave. )
			if (CAM_OIS_MASTER == o_ctrl->ois_type)
			{
				mutex_lock(&(o_ctrl->ois_hall_data_mutex));
				kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
				kfifo_reset(&(o_ctrl->ois_hall_data_fifoV2));
				mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
			}

		} else {
			switch (rc&0xF0)
			{
				case 0x00:
					CAM_ERR(CAM_OIS, "Error ; during the rom boot changing. Also including 128 power off issue.");
					break;
				case 0x20:
					CAM_ERR(CAM_OIS, "Error ; during Initial program for updating to program memory.");
					break;
				case 0x30:
					CAM_ERR(CAM_OIS, "Error ; during User Mat area erasing.");
					break;
				case 0x40:
					CAM_ERR(CAM_OIS, "Error ; during User Mat area programing.");
					break;
				case 0x50:
					CAM_ERR(CAM_OIS, "Error ; during the verification.");
					break;
				case 0x90:
					CAM_ERR(CAM_OIS, "Error ; during the drive offset confirmation.");
					break;
				case 0xA0:
					CAM_ERR(CAM_OIS, "Error ; during the MAT2 re-write process.");
					break;
				case 0xF0:
					if (rc == 0xF0)
					{
						CAM_ERR(CAM_OIS, "mistake of module vendor designation.");
					}
					else if (rc == 0xF1)
					{
						CAM_ERR(CAM_OIS, "mistake size of From Code.");
					}
					break;
				default:
					CAM_ERR(CAM_OIS, "Unkown error code");
					break;
			}
		}
	}
	else
	{
		CAM_ERR(CAM_OIS, "Unsupported OIS");
	}
	ktime_get_real_ts64(&mEndTime);
	diff = timespec64_sub(mEndTime, mStartTime);
	mSpendTime = (timespec64_to_ns(&diff))/1000000;

	CAM_INFO(CAM_OIS, "cam_ois_fw_download rc=%d, (Spend: %d ms)", rc, mSpendTime);

	return 0;
}

int DownloadFW(struct cam_ois_ctrl_t *o_ctrl)
{
	uint8_t rc = 0;

	if (o_ctrl)
	{
		mutex_lock(&ois_mutex);
		if(o_ctrl->opcode.prog == 1)
		{
			CAM_INFO(CAM_OIS, "update gyro vendor to =%d",o_ctrl->opcode.prog);
			if(o_ctrl->ois_type == CAM_OIS_MASTER)
			{
				o_ctrl->ois_gyro_vendor = 6;
			}
			else if(o_ctrl->ois_type == CAM_OIS_SLAVE)
			{
				o_ctrl->ois_gyro_vendor = 3;
			}
			ois_ctrls[CAM_OIS_MASTER]->ois_gyro_vendor = 6;
			if(ois_ctrls[CAM_OIS_SLAVE])
				ois_ctrls[CAM_OIS_SLAVE]->ois_gyro_vendor = 3;
		}
		else if(o_ctrl->opcode.prog == 3)
		{
			CAM_INFO(CAM_OIS, "update gyro vendor to =%d",o_ctrl->opcode.prog);
			if(o_ctrl->ois_type == CAM_OIS_MASTER)
			{
				o_ctrl->ois_gyro_vendor = 0;
			}
			else if(o_ctrl->ois_type == CAM_OIS_SLAVE)
			{
				o_ctrl->ois_gyro_vendor = 0;
			}
			if(ois_ctrls[CAM_OIS_MASTER])
			{
				ois_ctrls[CAM_OIS_MASTER]->ois_gyro_vendor = 0;
			}
			if(ois_ctrls[CAM_OIS_SLAVE])
				ois_ctrls[CAM_OIS_SLAVE]->ois_gyro_vendor = 0;
		}

		if (CAM_OIS_INVALID == ois_state[o_ctrl->ois_type])
		{

			if (CAM_OIS_MASTER == o_ctrl->ois_type)
			{
				rc = Download124Or128FW(ois_ctrls[CAM_OIS_MASTER]);
				if (rc)
				{
					CAM_ERR(CAM_OIS, "ois type=%d,Download %s FW failed",o_ctrl->ois_type, o_ctrl->ois_name);
				}
				else
				{
					if (dump_ois_registers && !ois_start_read_thread(ois_ctrls[CAM_OIS_MASTER], 1))
					{
						ois_start_read(ois_ctrls[CAM_OIS_MASTER], 1);
					}
				}
			}
			else if (CAM_OIS_SLAVE == o_ctrl->ois_type)
			{
				if (CAM_OIS_INVALID == ois_state[CAM_OIS_MASTER]  && !strstr(ois_ctrls[CAM_OIS_SLAVE]->ois_name, "bu24721"))
				{
					rc = Download124Or128FW(ois_ctrls[CAM_OIS_MASTER]);
				}
				if (rc)
				{
					CAM_ERR(CAM_OIS, "ois type=%d,Download %s FW failed",
						o_ctrl->ois_type,
						ois_ctrls[CAM_OIS_MASTER]->ois_name);
				}
				else
				{
					if(strstr(ois_ctrls[CAM_OIS_SLAVE]->ois_name, "bu24721"))
					{
						rc = Rohm_ois_fw_download(ois_ctrls[CAM_OIS_SLAVE]);
					}
					else
					{
						rc = Download124Or128FW(ois_ctrls[CAM_OIS_SLAVE]);
					}

					if (rc)
					{
						CAM_ERR(CAM_OIS, "ois type=%d,Download %s FW failed",
							o_ctrl->ois_type,
							o_ctrl->ois_name);
					}
					else
					{
						if (dump_ois_registers&&!ois_start_read_thread(ois_ctrls[CAM_OIS_SLAVE], 1))
						{
							ois_start_read(ois_ctrls[CAM_OIS_SLAVE], 1);
						}
						/*just for test tele start*/
						//if (dump_ois_registers&&!ois_start_read_thread(ois_ctrls[CAM_OIS_MASTER], 1)) {
						//ois_start_read(ois_ctrls[CAM_OIS_MASTER], 1);
						//}
						/*just for test tele end*/
					}
				}
			}
			ois_state[o_ctrl->ois_type] = CAM_OIS_FW_DOWNLOADED;
		}
		else
		{
			CAM_ERR(CAM_OIS, "ois type=%d,OIS state 0x%x is wrong",
				o_ctrl->ois_type,
				ois_state[o_ctrl->ois_type]);
		}
		mutex_unlock(&ois_mutex);
	}
	else
	{
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
	}

	return rc;
}

int OISPollThread124(void *arg)
{
#define SAMPLE_COUNT_IN_OIS_124 7
#define SAMPLE_INTERVAL     4000
	int32_t i = 0;
	uint32_t *data = NULL;
	uint32_t kfifo_in_len = 0;
	uint32_t fifo_size_in_ois = SAMPLE_COUNT_IN_OIS_124*SAMPLE_SIZE_IN_DRIVER;
	uint32_t fifo_size_in_driver = SAMPLE_COUNT_IN_DRIVER*SAMPLE_SIZE_IN_DRIVER;
	unsigned long long timestampQ = 0;

	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
	uint32_t ois_hall_registers[SAMPLE_COUNT_IN_OIS_124] = {0x89C4, 0x89C0, 0x89BC, 0x89B8, 0x89B4, 0x89B0, 0x89AC};

	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));

	data = kzalloc(fifo_size_in_ois, GFP_KERNEL);
	if (!data)
	{
		CAM_ERR(CAM_OIS, "failed to kzalloc");
		return -1;
	}

	CAM_DBG(CAM_OIS, "ois type=%d,OISPollThread124 creat",o_ctrl->ois_type);

	while(1)
	{
		mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
		if (o_ctrl->ois_poll_thread_exit)
		{
			mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
			goto exit;
		}
		mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
		timestampQ = __arch_counter_get_cntvct();
		//CAM_ERR(CAM_OIS, "trace timestamp:%lld in Qtime", timestampQ);

		memset(data, 0, fifo_size_in_ois);

		//Read OIS HALL data
		for (i = 0; i < SAMPLE_COUNT_IN_OIS_124; i++)
		{
			data[3*i] = timestampQ >> 32;
			data[3*i+1] = timestampQ & 0xFFFFFFFF;
			OISRead(o_ctrl, ois_hall_registers[i], &(data[3*i+2]));
			timestampQ -= 2*CLOCK_TICKCOUNT_MS;
		}

		for (i = SAMPLE_COUNT_IN_OIS_124 - 1; i >= 0; i--)
		{
			CAM_DBG(CAM_OIS, "ois type=%d,OIS HALL data %lld (0x%x 0x%x)",
				o_ctrl->ois_type,
				((uint64_t)data[3*i] << 32)+(uint64_t)data[3*i+1],
				data[3*i+2]&0xFFFF0000>>16, data[3*i+2]&0xFFFF);
		}

		mutex_lock(&(o_ctrl->ois_hall_data_mutex));
		if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + fifo_size_in_ois) > fifo_size_in_driver)
		{
			CAM_DBG(CAM_OIS, "ois type=%d,ois_hall_data_fifo is full, fifo size %d, file len %d, will reset FIFO",
				o_ctrl->ois_type,
				kfifo_size(&(o_ctrl->ois_hall_data_fifo)),
				kfifo_len(&(o_ctrl->ois_hall_data_fifo)));
			kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
		}

		if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + fifo_size_in_ois) <= fifo_size_in_driver)
		{
			kfifo_in_len = kfifo_in(&(o_ctrl->ois_hall_data_fifo), data, fifo_size_in_ois);
			if (kfifo_in_len != fifo_size_in_ois)
			{
				CAM_DBG(CAM_OIS, "ois type=%d,kfifo_in %d Bytes, FIFO maybe full, some OIS Hall sample maybe dropped.",
					o_ctrl->ois_type,
					kfifo_in_len);
			}
			else
			{
				CAM_DBG(CAM_OIS, "ois type=%d,kfifo_in %d Bytes",
					o_ctrl->ois_type,
					fifo_size_in_ois);
			}
		}
		mutex_unlock(&(o_ctrl->ois_hall_data_mutex));

		usleep_range(SAMPLE_COUNT_IN_OIS_124*SAMPLE_INTERVAL-5, SAMPLE_COUNT_IN_OIS_124*SAMPLE_INTERVAL);
	}

exit:
	kfree(data);
	CAM_DBG(CAM_OIS, "ois type=%d,OISPollThread124 exit",o_ctrl->ois_type);
	return 0;
}


int OISPollThread128(void *arg)
{
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t kfifo_in_len = 0;
	uint32_t fifo_size_in_ois = SAMPLE_COUNT_IN_OIS*SAMPLE_SIZE_IN_OIS;
	uint32_t fifo_size_in_ois_aligned = SAMPLE_COUNT_IN_OIS*SAMPLE_SIZE_IN_OIS_ALIGNED;
	uint32_t fifo_size_in_driver = SAMPLE_COUNT_IN_DRIVER*SAMPLE_SIZE_IN_DRIVER;
	uint16_t *p_hall_data_in_ois = NULL;
	struct cam_ois_hall_data_in_ois_aligned *p_hall_data_in_ois_aligned = NULL;
	struct cam_ois_hall_data_in_driver *p_hall_data_in_driver = NULL;
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
	uint64_t first_QTimer = 0;      // This will be used for the start QTimer to calculate the QTimer interval
	uint64_t prev_QTimer = 0;       // This is the last QTimer in the last CCI read
	uint64_t current_QTimer = 0;    // This will be used for the end QTimer to calculate the QTimer interval
	uint64_t interval_QTimer = 0;   // This is the QTimer interval between two sample
	uint64_t sample_offset = 0;     // This is timestamp offset between IC and system
	uint64_t readout_time = (((1+2+1+SAMPLE_SIZE_IN_OIS*SAMPLE_COUNT_IN_OIS)*8)/1000)*CLOCK_TICKCOUNT_MS;      // This is the time of CCI read
	uint16_t sample_count = 0;      // This is the sample count for one CCI read
	uint16_t sample_num = 0;        // This will be used to detect whehter some HALL data was dropped
	uint32_t total_sample_count = 0;// This will be used to calculate the QTimer interval
	uint16_t threshold = 2;         // This is the threshold to trigger Timestamp calibration, this means 2ms
	uint16_t tmp = 0;
	uint64_t real_QTimer;
	uint64_t real_QTimer_after;
	uint64_t i2c_read_offset;
	static uint64_t pre_real_QTimer = 0;
	static uint64_t pre_QTimer_offset =0 ;
	uint64_t estimate_QTimer = 0;	// This is the QTimer interval between two sample
	uint32_t vaild_cnt = 0;
	uint32_t is_add_Offset = 0;
	uint32_t offset_cnt;
        uint32_t data=0;

	p_hall_data_in_ois = kzalloc(fifo_size_in_ois, GFP_KERNEL);
	if (!p_hall_data_in_ois)
	{
		CAM_ERR(CAM_OIS, "failed to kzalloc p_hall_data_in_ois");
		return -1;
	}

	p_hall_data_in_ois_aligned = kzalloc(fifo_size_in_ois_aligned, GFP_KERNEL);
	if (!p_hall_data_in_ois_aligned)
	{
		CAM_ERR(CAM_OIS, "failed to kzalloc p_hall_data_in_ois_aligned");
		kfree(p_hall_data_in_ois);
		return -1;
	}

	p_hall_data_in_driver = kzalloc(SAMPLE_COUNT_IN_OIS*SAMPLE_SIZE_IN_DRIVER, GFP_KERNEL);
	if (!p_hall_data_in_driver)
	{
		CAM_ERR(CAM_OIS, "failed to kzalloc p_hall_data_in_driver");
		kfree(p_hall_data_in_ois);
		kfree(p_hall_data_in_ois_aligned);
		return -1;
	}

	CAM_DBG(CAM_OIS, "ois type=%d,OISPollThread128 creat",o_ctrl->ois_type);
#ifdef ENABLE_OIS_DELAY_POWER_DOWN
        mutex_lock(&(o_ctrl->ois_power_down_mutex));
        if (o_ctrl->ois_power_state == CAM_OIS_POWER_OFF)
		{
                mutex_unlock(&(o_ctrl->ois_power_down_mutex));
                goto exit;
        }
        mutex_unlock(&(o_ctrl->ois_power_down_mutex));
#endif

        RamWrite32A_oneplus(o_ctrl,0xF110, 0x0);//Clear buffer to all "0" & enable buffer update function.
        RamRead32A_oneplus(o_ctrl, 0x82B8, &data);
        CAM_ERR(CAM_OIS, "ois addr=0x82B8 data=0x%x",data);

        RamRead32A_oneplus(o_ctrl, 0x8318, &data);
        CAM_ERR(CAM_OIS, "ois addr=0X8318 data=0x%x",data);

        RamRead32A_oneplus(o_ctrl, 0x8800, &data);
        CAM_ERR(CAM_OIS, "ois addr=0x8800 data=0x%x",data);

        RamWrite32A_oneplus(o_ctrl,0x82B8, 0x40000000);
        RamWrite32A_oneplus(o_ctrl,0x8318, 0x40000000);
        RamWrite32A_oneplus(o_ctrl,0x8800, 0x10);

        RamRead32A_oneplus(o_ctrl, 0x82B8, &data);
        CAM_ERR(CAM_OIS, "ois addr=0x82B8 data=0x%x",data);

        RamRead32A_oneplus(o_ctrl, 0x8318, &data);
        CAM_ERR(CAM_OIS, "ois addr=0X8318 data=0x%x",data);

        RamRead32A_oneplus(o_ctrl, 0x8800, &data);
        CAM_ERR(CAM_OIS, "ois addr=0x8800 data=0x%x",data);


	while(1)
	{

		mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
		if (o_ctrl->ois_poll_thread_exit)
		{
			mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
			goto exit;
		}
		mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));

		sample_count = 0;
		tmp = sample_num;
		memset(p_hall_data_in_ois, 0, fifo_size_in_ois);
		memset(p_hall_data_in_ois_aligned, 0, fifo_size_in_ois_aligned);
		memset(p_hall_data_in_driver, 0, SAMPLE_COUNT_IN_OIS*SAMPLE_SIZE_IN_DRIVER);

		usleep_range(1995, 2000);

		real_QTimer = __arch_counter_get_cntvct();

		//Read OIS HALL data
		OISCountinueRead(o_ctrl, 0xF111, (void *)p_hall_data_in_ois, fifo_size_in_ois);
		real_QTimer_after = __arch_counter_get_cntvct();
		i2c_read_offset = real_QTimer_after - real_QTimer;
		//Covert the data from unaligned to aligned
		for(i = 0, j = 0; i < SAMPLE_COUNT_IN_OIS; i++)
		{
			if(((p_hall_data_in_ois[3*i] == 0) && (p_hall_data_in_ois[3*i+1] == 0) && (p_hall_data_in_ois[3*i+2] == 0)) || \
				(p_hall_data_in_ois[3*i] == OIS_MAGIC_NUMBER && p_hall_data_in_ois[3*i+1] == OIS_MAGIC_NUMBER))
			{
				CAM_DBG(CAM_OIS, "ois type=%d,OIS HALL RAW data %d %d (0x%x 0x%x)",o_ctrl->ois_type, i,
				        p_hall_data_in_ois[3*i],
				        p_hall_data_in_ois[3*i+1],
				        p_hall_data_in_ois[3*i+2]);
			}
			else
			{
				p_hall_data_in_ois_aligned[j].hall_data_cnt = p_hall_data_in_ois[3*i];
				p_hall_data_in_ois_aligned[j].hall_data = ((uint32_t)p_hall_data_in_ois[3*i+1] << 16) + p_hall_data_in_ois[3*i+2];
				CAM_DBG(CAM_OIS, "ois type=%d,OIS HALL RAW data %d %d (0x%x 0x%x)",o_ctrl->ois_type, i,
				        p_hall_data_in_ois_aligned[j].hall_data_cnt,
				        p_hall_data_in_ois_aligned[j].hall_data&0xFFFF0000>>16,
				        p_hall_data_in_ois_aligned[j].hall_data&0xFFFF);
				j++;
			}
		}

		sample_offset = (uint64_t)((p_hall_data_in_ois[3*(SAMPLE_COUNT_IN_OIS-1)+2] & 0xFF) * CLOCK_TICKCOUNT_MS * 2 / OIS_MAX_COUNTER);

		if(first_QTimer == 0)
		{
			//Init some parameters
			for(i = 0; i < SAMPLE_COUNT_IN_OIS; i++)
			{
				if((p_hall_data_in_ois_aligned[i].hall_data == 0)
					&& (p_hall_data_in_ois_aligned[i].hall_data_cnt == 0))
				{
					break;
				}
			}
			if ((i >= 1) && (i <= SAMPLE_COUNT_IN_OIS))
			{
				first_QTimer = __arch_counter_get_cntvct() - readout_time - sample_offset;
				prev_QTimer = first_QTimer;
				sample_num = p_hall_data_in_ois_aligned[i-1].hall_data_cnt;
			}
			continue;
		}
		else
		{
			vaild_cnt = 0;
			current_QTimer = __arch_counter_get_cntvct() - readout_time - sample_offset;
			//calculate sample_count and total_sample_count, and detect whether some hall data was dropped.
			for(i = 0; i < SAMPLE_COUNT_IN_OIS; i++)
			{
				if((p_hall_data_in_ois_aligned[i].hall_data != 0)
					|| (p_hall_data_in_ois_aligned[i].hall_data_cnt != 0))
				{
					total_sample_count++;
					sample_count++;
					while (++tmp != p_hall_data_in_ois_aligned[i].hall_data_cnt)
					{
						total_sample_count++;
						CAM_DBG(CAM_OIS, "ois type=%d,One sample was droped, %d %d %d",
							o_ctrl->ois_type,
							i,
							tmp,
							p_hall_data_in_ois_aligned[i].hall_data_cnt);
					}
				}
			}
			if(sample_count > 0)
			{
				if (total_sample_count > 1)
				{
					interval_QTimer = (current_QTimer - first_QTimer)/(total_sample_count - 1);
				}
				else if(total_sample_count == 1)
				{
					interval_QTimer = threshold*CLOCK_TICKCOUNT_MS;
				}

				//Calculate the TS for every sample, if some sample were dropped, the TS of this sample will still be calculated, but will not report to UMD.
				for(i = 0; i < SAMPLE_COUNT_IN_OIS; i++)
				{
					if((p_hall_data_in_ois_aligned[i].hall_data != 0)
						|| (p_hall_data_in_ois_aligned[i].hall_data_cnt != 0))
					{
						if (i == 0)
						{
							//p_hall_data_in_driver[i].timestamp = prev_QTimer;
							estimate_QTimer = prev_QTimer;
							while (++sample_num != p_hall_data_in_ois_aligned[i].hall_data_cnt)
							{
								//p_hall_data_in_driver[i].timestamp += interval_QTimer;
								estimate_QTimer += interval_QTimer;
							}
							//p_hall_data_in_driver[i].timestamp += interval_QTimer;
							estimate_QTimer += interval_QTimer;
							p_hall_data_in_driver[i].high_dword = estimate_QTimer >> 32;
							p_hall_data_in_driver[i].low_dword  = estimate_QTimer & 0xFFFFFFFF;
							p_hall_data_in_driver[i].hall_data  = p_hall_data_in_ois_aligned[i].hall_data;
						}
						else
						{
							//p_hall_data_in_driver[i].timestamp = p_hall_data_in_driver[i-1].timestamp;
							estimate_QTimer = ((uint64_t)p_hall_data_in_driver[i-1].high_dword << 32) + (uint64_t)p_hall_data_in_driver[i-1].low_dword;
							while (++sample_num != p_hall_data_in_ois_aligned[i].hall_data_cnt)
							{
								//p_hall_data_in_driver[i].timestamp += interval_QTimer;
								estimate_QTimer += interval_QTimer;
							}
							//p_hall_data_in_driver[i].timestamp += interval_QTimer;
							estimate_QTimer += interval_QTimer;
							p_hall_data_in_driver[i].high_dword = estimate_QTimer >> 32;
							p_hall_data_in_driver[i].low_dword  = estimate_QTimer & 0xFFFFFFFF;
							p_hall_data_in_driver[i].hall_data  = p_hall_data_in_ois_aligned[i].hall_data;
						}

						CAM_DBG(CAM_OIS, "ois type=%d,OIS HALL data %lld (0x%x 0x%x)",o_ctrl->ois_type,
						        ((uint64_t)p_hall_data_in_driver[i].high_dword << 32) + (uint64_t)p_hall_data_in_driver[i].low_dword,
						        (p_hall_data_in_driver[i].hall_data&0xFFFF0000)>>16,
						        p_hall_data_in_driver[i].hall_data&0xFFFF);
						vaild_cnt ++ ;
					}
					else
					{
						break;
					}
				}

				if ((i >= 1) && (i <= SAMPLE_COUNT_IN_OIS))
				{
					prev_QTimer = ((uint64_t)p_hall_data_in_driver[i-1].high_dword << 32) + (uint64_t)p_hall_data_in_driver[i-1].low_dword;
				}
				real_QTimer -= sample_offset;


				CAM_DBG(CAM_OIS, "OIS HALL data before %lld %lld",
							real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer,
							pre_real_QTimer - (real_QTimer - (vaild_cnt-1) * interval_QTimer) );

				if ( pre_real_QTimer != 0 &&  vaild_cnt > 0 &&
					((real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer <= CLOCK_TICKCOUNT_MS) ||
					 (pre_real_QTimer - (real_QTimer - (vaild_cnt-1) * interval_QTimer) <= CLOCK_TICKCOUNT_MS)))
				{
					real_QTimer += interval_QTimer;
				}

				for (i =0; i < 5; i++)
				{
					if ( pre_real_QTimer != 0 &&  vaild_cnt > 0 &&
						((int64_t)(real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer )< 0))
					{
						real_QTimer += interval_QTimer;
						is_add_Offset = 1;
					}
				}

				if ( pre_real_QTimer != 0 &&  vaild_cnt > 0 &&
					((real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer <= CLOCK_TICKCOUNT_MS) ||
					 (pre_real_QTimer - (real_QTimer - (vaild_cnt-1) * interval_QTimer) <= CLOCK_TICKCOUNT_MS)))
				{
					real_QTimer += interval_QTimer;

				}

				if ((pre_real_QTimer != 0)
					&& ((int64_t)(real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer) > 42000 
					|| (int64_t)(real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer) < 34000))
				{

					if (total_sample_count > 100 )
					{
						real_QTimer =  pre_real_QTimer + vaild_cnt * interval_QTimer;
						CAM_DBG(CAM_OIS, "OIS HALL data force calate  %d ",offset_cnt);
						offset_cnt ++ ;
						if (offset_cnt > 3)
						{
							is_add_Offset = 1;
						}
					}
				}
				else
				{
						offset_cnt = 0;
				}

				CAM_DBG(CAM_OIS, "OIS HALL data after %lld  %lld",
							real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer,
							pre_real_QTimer - (real_QTimer - (vaild_cnt-1) * interval_QTimer));

				pre_QTimer_offset = real_QTimer - pre_real_QTimer - (vaild_cnt-1) * interval_QTimer;

				for (i = 0; i < vaild_cnt ;i++)
				{
					p_hall_data_in_driver[vaild_cnt - i -1].high_dword = real_QTimer >> 32;
					p_hall_data_in_driver[vaild_cnt - i -1].low_dword  = real_QTimer & 0xFFFFFFFF;
					real_QTimer -= interval_QTimer;
				}

				for ( i = 0; i < vaild_cnt;i++)
				{
					CAM_DBG(CAM_OIS, "OIS HALL data %lld (0x%x 0x%x) pre :%lld offset reg:%d i2c_read_offset %lld",
						((uint64_t)p_hall_data_in_driver[i].high_dword << 32) + (uint64_t)p_hall_data_in_driver[i].low_dword,
						(p_hall_data_in_driver[i].hall_data&0xFFFF0000)>>16,
						p_hall_data_in_driver[i].hall_data&0xFFFF,
						        pre_real_QTimer,
						        (p_hall_data_in_ois[3*(SAMPLE_COUNT_IN_OIS-1)+2] & 0xFF),
						        i2c_read_offset);

				}
				if (!is_add_Offset)
				{
					pre_real_QTimer = ((uint64_t)p_hall_data_in_driver[vaild_cnt -1].high_dword << 32) |
						(uint64_t)p_hall_data_in_driver[vaild_cnt -1].low_dword;
				}
				else
				{
					pre_real_QTimer = 0;
					is_add_Offset = 0;
				}
				//Do Timestamp calibration
				//Put the HALL data into the FIFO
				mutex_lock(&(o_ctrl->ois_hall_data_mutex));
				if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + vaild_cnt*SAMPLE_SIZE_IN_DRIVER) > fifo_size_in_driver)
				{
					CAM_DBG(CAM_OIS, "ois type=%d,ois_hall_data_fifo is full, fifo size %d, file len %d, will reset FIFO",
						o_ctrl->ois_type,
						kfifo_size(&(o_ctrl->ois_hall_data_fifo)),
						kfifo_len(&(o_ctrl->ois_hall_data_fifo)));
					kfifo_reset(&(o_ctrl->ois_hall_data_fifo));
				}

				if ((kfifo_len(&(o_ctrl->ois_hall_data_fifoV2)) + vaild_cnt*SAMPLE_SIZE_IN_DRIVER) > fifo_size_in_driver)
				{
					CAM_DBG(CAM_OIS, "ois type=%d,ois_hall_data_fifoV2 is full, fifo size %d, file len %d, will reset FIFO",
						o_ctrl->ois_type,
						kfifo_size(&(o_ctrl->ois_hall_data_fifoV2)),
						kfifo_len(&(o_ctrl->ois_hall_data_fifoV2)));
					kfifo_reset(&(o_ctrl->ois_hall_data_fifoV2));
				}
				//Store ois data for EISV3
				if ((kfifo_len(&(o_ctrl->ois_hall_data_fifo)) + vaild_cnt*SAMPLE_SIZE_IN_DRIVER) <= fifo_size_in_driver)
				{
					kfifo_in_len = kfifo_in(&(o_ctrl->ois_hall_data_fifo),
						p_hall_data_in_driver,
						vaild_cnt*SAMPLE_SIZE_IN_DRIVER);

					if (kfifo_in_len != vaild_cnt*SAMPLE_SIZE_IN_DRIVER)
					{
						CAM_DBG(CAM_OIS, "ois type=%d,kfifo_in %d Bytes, FIFO maybe full, some OIS Hall sample maybe dropped.",
							o_ctrl->ois_type,
							kfifo_in_len);
					}
					else
					{
						CAM_DBG(CAM_OIS, "ois type=%d,kfifo_in %d Bytes",
							o_ctrl->ois_type,
							vaild_cnt*SAMPLE_SIZE_IN_DRIVER);
					}
				}
				//Store ois data for EISv2
				if ((kfifo_len(&(o_ctrl->ois_hall_data_fifoV2)) + vaild_cnt*SAMPLE_SIZE_IN_DRIVER) <= fifo_size_in_driver)
				{
					kfifo_in_len = kfifo_in(&(o_ctrl->ois_hall_data_fifoV2),
						p_hall_data_in_driver,
						vaild_cnt*SAMPLE_SIZE_IN_DRIVER);

					if (kfifo_in_len != vaild_cnt*SAMPLE_SIZE_IN_DRIVER)
					{
						CAM_DBG(CAM_OIS, "ois type=%d,kfifo_in %d Bytes, FIFOV2 maybe full, some OIS Hall sample maybe dropped.",
							o_ctrl->ois_type,
							kfifo_in_len);
					}
					else
					{
						CAM_DBG(CAM_OIS, "ois type=%d,kfifo_inV2 %d Bytes",
							o_ctrl->ois_type,
							vaild_cnt*SAMPLE_SIZE_IN_DRIVER);
					}
				}
				mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
			}
		}
	}

exit:
	pre_real_QTimer = 0;
	is_add_Offset = 0;
	total_sample_count = 0;
	kfree(p_hall_data_in_ois);
	kfree(p_hall_data_in_ois_aligned);
	kfree(p_hall_data_in_driver);
	CAM_DBG(CAM_OIS, "ois type=%d,OISPollThread128 exit",o_ctrl->ois_type);
	return 0;
}



#define OIS_HALL_DATA_SIZE   52
struct hall_info
{
    uint32_t timeStampSec;  //us
    uint32_t timeStampUsec;
    uint32_t mHalldata;
};

void timeval_add(struct timeval *tv,int32_t usec)
{
	if (usec > 0) {
		if (tv->tv_usec + usec >= 1000*1000) {
			tv->tv_sec += 1;
			tv->tv_usec = tv->tv_usec - 1000 * 1000 + usec;
		} else {
			tv->tv_usec += usec;
		}
	} else {
		if (tv->tv_usec < abs(usec)) {
			tv->tv_sec -= 1;
			tv->tv_usec = tv->tv_usec + 1000 * 1000 + usec;
		} else {
			tv->tv_usec += usec;
		}
	}
}

void do_gettimeofday(struct timeval *tv)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_usec = now.tv_nsec/1000;
}

int WRITE_QTIMER_TO_OIS (struct cam_ois_ctrl_t *o_ctrl)
{
	uint64_t qtime_ns = 0,qtime_ns_after=0,value=351000;
	int32_t  rc = 0,i=0,j=0;
	static struct cam_sensor_i2c_reg_array *i2c_write_setting_gl = NULL;
	struct cam_sensor_i2c_reg_setting i2c_write;
	uint32_t reg_addr_qtimer = 0x0;
	if(strstr(o_ctrl->ois_name, "128")){
		reg_addr_qtimer = 0xF112;
	}else if(strstr(o_ctrl->ois_name, "124")){
		reg_addr_qtimer = 0xF114;
	}
	CAM_DBG(CAM_OIS, "reg_adr_qtimer = 0x%x",reg_addr_qtimer);

	if (i2c_write_setting_gl == NULL)
	{
		i2c_write_setting_gl = (struct cam_sensor_i2c_reg_array *)kzalloc(
		                           sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2, GFP_KERNEL);
		if(!i2c_write_setting_gl)
		{
			CAM_ERR(CAM_OIS, "Alloc i2c_write_setting_gl failed");
			return -1;
		}
	}
	/*if(is_wirte==TRUE){
		CAM_ERR(CAM_OIS, "have write Qtimer");
		return 0;
	}*/
	memset(i2c_write_setting_gl, 0, sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2);
        while(value>350000  && j<5 ){
	        rc = cam_sensor_util_get_current_qtimer_ns(&qtime_ns);
	        if (rc < 0) {
		        CAM_ERR(CAM_OIS,
			        "Failed to get current qtimer value: %d",
			        rc);
		        return rc;
	        }
                CAM_DBG(CAM_OIS,"qtime_ns: ms=0x%x us=0x%x",(uint32_t)((qtime_ns>>32)&0xffffffff),(uint32_t)(qtime_ns&0xffffffff));

	        for(i = 0; i< 2; i++) {
		        if (i == 0) {
			        i2c_write_setting_gl[i].reg_addr = reg_addr_qtimer;
			        i2c_write_setting_gl[i].reg_data = (uint32_t)((qtime_ns>>32)&0xffffffff);
			        i2c_write_setting_gl[i].delay = 0X00;
			        i2c_write_setting_gl[i].data_mask = 0x00;
		        } else {
			        i2c_write_setting_gl[i].reg_data = (uint32_t)(qtime_ns&0xffffffff);
			        i2c_write_setting_gl[i].delay = 0x00;
			        i2c_write_setting_gl[i].data_mask = 0x00;
		        }
	        }
	        i2c_write.reg_setting = i2c_write_setting_gl;
	        i2c_write.size = 2;
	        i2c_write.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	        i2c_write.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	        i2c_write.delay = 0x00;
                rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),&i2c_write, 1);
	        if (rc < 0) {
		        CAM_ERR(CAM_OIS,
			        "Failed to write qtimer value: %d",
			        rc);
		        return rc;
	        }
	        rc = cam_sensor_util_get_current_qtimer_ns(&qtime_ns_after);
	        if (rc < 0) {
		        CAM_ERR(CAM_OIS,
			        "Failed to get current qtime_ns_after value: %d",
			        rc);
		        return rc;
	        }
                CAM_DBG(CAM_OIS,"qtime_ns_after: ms=0x%x us=0x%x",(uint32_t)((qtime_ns_after>>32)&0xffffffff),(uint32_t)(qtime_ns_after&0xffffffff));
                value = qtime_ns_after-qtime_ns;
                //is_wirte=TRUE;
                if(j>0)
                        CAM_DBG(CAM_OIS,"write time=%d",value);
                j++;
        }
        return rc;
}

int OIS_READ_HALL_DATA_TO_UMD_NEW (struct cam_ois_ctrl_t *o_ctrl,struct i2c_settings_array *i2c_settings) {

        int32_t                         rc = 0,i=0;
        struct i2c_settings_list       *i2c_list;
        uint8_t                        *read_buff = NULL;
        uint32_t                        buff_length = 0;
        uint32_t                        read_length = 0;
        uint64_t                        fifo_count = 0,qtime_ms=0,qtime_us=0;
        uint64_t                        preqtime_ms=0,newqtime=0;
        uint32_t                        hall_data_x = 0,hall_data_y = 0;
        int                             j=0;
        uint8_t                        *temp_buff = NULL;


        list_for_each_entry(i2c_list,
                        &(i2c_settings->list_head), list) {
                        read_buff = i2c_list->i2c_settings.read_buff;
                        buff_length = i2c_list->i2c_settings.read_buff_len;
                        if ((read_buff == NULL) || (buff_length == 0)) {
                                CAM_ERR(CAM_OIS,
                                        "Invalid input buffer, buffer: %pK, length: %d",
                                        read_buff, buff_length);
                                return -EINVAL;
                        }
                        read_length = i2c_list->i2c_settings.size;
                        temp_buff = kzalloc(read_length, GFP_KERNEL);
                        memset(temp_buff, 0, read_length);
                        CAM_DBG(CAM_OIS,"buffer: %pK, fbufer_length: %d  read_length=%d",read_buff, buff_length,read_length);
                        CAM_DBG(CAM_OIS,"start read");
                        OISCountinueRead(o_ctrl, 0xF111, (void *)temp_buff, read_length);
                        CAM_DBG(CAM_OIS,"read done");
                        /* ois data count is 144 Bytes and have max sample is 32; last Bytes is sample count,and 32/33 Bytes means Qtimer */

                        fifo_count = temp_buff[143];
                        read_buff[143]=temp_buff[143];
                        if(fifo_count > SAMPLE_COUNT_IN_NCS_DATA) {
                                CAM_ERR(CAM_OIS,"ois have drop data fifo_count=%d",fifo_count);
                                fifo_count = SAMPLE_COUNT_IN_NCS_DATA;
                        }

                        for(j=0;j<fifo_count;j++){
                                read_buff[j*4]=temp_buff[(fifo_count-j-1)*4];
                                read_buff[j*4+1]=temp_buff[(fifo_count-j-1)*4+1];
                                read_buff[j*4+2]=temp_buff[(fifo_count-j-1)*4+2];
                                read_buff[j*4+3]=temp_buff[(fifo_count-j-1)*4+3];
                        }

                        for(j=0;j<8;j++)
                                read_buff[128+j]=temp_buff[128+j];
                        CAM_DBG(CAM_OIS,"Qtimer = 0x%x,%x,%x,%x,%x,%x,%x,%x",read_buff[128],read_buff[129],read_buff[130],read_buff[131],read_buff[132],read_buff[133],read_buff[134],read_buff[135]);

                        if(temp_buff[128] == 0xff){
                            CAM_ERR(CAM_OIS,"Qtimer = 0x%x,%x,%x,%x,%x,%x,%x,%x",read_buff[128],read_buff[129],read_buff[130],read_buff[131],read_buff[132],read_buff[133],read_buff[134],read_buff[135]);
                            read_buff[143] = 0;
                            fifo_count = 0;
                        }

                        for ( j = 0; j < 8; j++)
                        {
                                 newqtime = (newqtime << 8) | (read_buff[128+j]);
                        }
                        qtime_ms = (((uint32_t)read_buff[128]<<24)+((uint32_t)read_buff[129]<<16)+((uint32_t)read_buff[130]<<8)+(uint32_t)read_buff[131]);
                        qtime_us = (((uint32_t)read_buff[132]<<24)+((uint32_t)read_buff[133]<<16)+((uint32_t)read_buff[134]<<8)+(uint32_t)read_buff[135]);
                        if((newqtime-(fifo_count-1)*2000000)<preqtime_ms)
                                CAM_ERR(CAM_OIS,"time error");
                        preqtime_ms=newqtime;
                        CAM_DBG(CAM_OIS,"READ fifo_count=%d",fifo_count);
                        CAM_DBG(CAM_OIS,"Rqtimer value: ms=0x%x us=0x%x",qtime_ms,qtime_us);

                        if(fifo_count > 0) {
                                for(i=0 ; i<SAMPLE_COUNT_IN_NCS_DATA; i++){
                                        hall_data_x=((read_buff[i*4]<<8)+read_buff[i*4+1]);
                                        hall_data_y=((read_buff[i*4+2]<<8)+read_buff[i*4+3]);
                                        CAM_DBG(CAM_OIS,"hall_data_x=0x%x hall_data_y=0x%x ",hall_data_x,hall_data_y);
                                }
                        }
        }
        kfree(temp_buff);
        return rc;
}

int OIS_READ_HALL_DATA_TO_UMD_TELE124 (struct cam_ois_ctrl_t *o_ctrl,struct i2c_settings_array *i2c_settings) {

        int32_t                         rc = 0,i=0;
        struct i2c_settings_list       *i2c_list;
        uint8_t                        *read_buff = NULL;
        uint32_t                        buff_length = 0;
        uint32_t                        read_length = 0;
        uint64_t                        fifo_count = 0,qtime_ms=0,qtime_us=0;
        uint64_t                        preqtime_ms=0,newqtime=0;
        uint32_t                        hall_data_x = 0,hall_data_y = 0;
        int                             j=0;
        uint8_t                        *temp_buff = NULL;


        list_for_each_entry(i2c_list,
                        &(i2c_settings->list_head), list) {
                        read_buff = i2c_list->i2c_settings.read_buff;
                        buff_length = i2c_list->i2c_settings.read_buff_len;
                        if ((read_buff == NULL) || (buff_length == 0)) {
                                CAM_ERR(CAM_OIS,
                                        "Invalid input buffer, buffer: %pK, length: %d",
                                        read_buff, buff_length);
                                return -EINVAL;
                        }
                        read_length = i2c_list->i2c_settings.size;
                        temp_buff = kzalloc(read_length, GFP_KERNEL);
                        memset(temp_buff, 0, read_length);
                        CAM_DBG(CAM_OIS,"buffer: %pK, fbufer_length: %d  read_length=%d",read_buff, buff_length,read_length);
                        CAM_DBG(CAM_OIS,"start read");
                        OISCountinueRead(o_ctrl, 0xF113, (void *)temp_buff, read_length);
                        CAM_DBG(CAM_OIS,"read done");
                        /* ois data count is 144 Bytes and have max sample is 32; last Bytes is sample count,and 32/33 Bytes means Qtimer */

                        fifo_count = temp_buff[127];
                        read_buff[127]=temp_buff[127];
                        if(fifo_count > SAMPLE_COUNT_IN_NCS_DATA_TELE124) {
                                CAM_ERR(CAM_OIS,"ois have drop data fifo_count=%d",fifo_count);
                                fifo_count = SAMPLE_COUNT_IN_NCS_DATA_TELE124;
                        }

                        for(j=0;j<fifo_count;j++){
                                read_buff[j*4]=temp_buff[(fifo_count-j-1)*4];
                                read_buff[j*4+1]=temp_buff[(fifo_count-j-1)*4+1];
                                read_buff[j*4+2]=temp_buff[(fifo_count-j-1)*4+2];
                                read_buff[j*4+3]=temp_buff[(fifo_count-j-1)*4+3];
                        }

                        for(j=0;j<8;j++){
                                read_buff[112+j]=temp_buff[112+j];
                        }
                        CAM_DBG(CAM_OIS,"Qtimer = 0x%x,%x,%x,%x,%x,%x,%x,%x",read_buff[112],read_buff[113],read_buff[114],read_buff[115],read_buff[116],read_buff[117],read_buff[118],read_buff[119]);

                        if(temp_buff[112] == 0xff){
                            CAM_ERR(CAM_OIS,"Qtimer = 0x%x,%x,%x,%x,%x,%x,%x,%x",read_buff[112],read_buff[113],read_buff[114],read_buff[115],read_buff[116],read_buff[117],read_buff[118],read_buff[119]);
                            read_buff[127] = 0;
                            fifo_count = 0;
                        }

                        for ( j = 0; j < 8; j++)
                        {
                                 newqtime = (newqtime << 8) | (read_buff[112+j]);
                        }
                        qtime_ms = (((uint32_t)read_buff[112]<<24)+((uint32_t)read_buff[113]<<16)+((uint32_t)read_buff[114]<<8)+(uint32_t)read_buff[115]);
                        qtime_us = (((uint32_t)read_buff[116]<<24)+((uint32_t)read_buff[117]<<16)+((uint32_t)read_buff[118]<<8)+(uint32_t)read_buff[119]);
                        if((newqtime-(fifo_count-1)*2000000)<preqtime_ms)
                                CAM_ERR(CAM_OIS,"time error");
                        preqtime_ms=newqtime;
                        CAM_DBG(CAM_OIS,"READ fifo_count=%d",fifo_count);

                        if(fifo_count > 0) {
                                for(i=0 ; i<SAMPLE_COUNT_IN_NCS_DATA_TELE124; i++){
                                        hall_data_x=((read_buff[i*4]<<8)+read_buff[i*4+1]);
                                        hall_data_y=((read_buff[i*4+2]<<8)+read_buff[i*4+3]);
                                        CAM_DBG(CAM_OIS,"hall_data_x=0x%x hall_data_y=0x%x ",hall_data_x,hall_data_y);
                                }
                        }
        }
        kfree(temp_buff);
        return rc;
}

int OIS_READ_HALL_DATA_TO_UMD (struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_settings)
{
	struct i2c_settings_list       *i2c_list;
	uint8_t                        *read_buff = NULL;
	uint32_t                        buff_length = 0;
	uint32_t                        read_length = 0;
	uint32_t                        read_fifo_length = 0;
	uint32_t                        fifo_len_in_ois_driver=0;
	uint8_t                        *temp_buff = NULL;
	uint8_t                         j=0,sample_count=0;
	uint64_t                        ticks = 0;
	uint64_t                        qtimer=0,pre_qtimer=0;

	temp_buff = kzalloc(SAMPLE_COUNT_IN_OIS*SAMPLE_SIZE_IN_DRIVER, GFP_KERNEL);
	memset(temp_buff, 0, SAMPLE_COUNT_IN_OIS*SAMPLE_SIZE_IN_DRIVER);

	list_for_each_entry(i2c_list,
		&(i2c_settings->list_head), list)
	{
		read_buff = i2c_list->i2c_settings.read_buff;
		buff_length = i2c_list->i2c_settings.read_buff_len;
		if ((read_buff == NULL) || (buff_length == 0))
		{
			CAM_ERR(CAM_OIS,
				"Invalid input buffer, buffer: %pK, length: %d",
				read_buff, buff_length);
			kfree(temp_buff);
			return -EINVAL;
		}
		read_length = i2c_list->i2c_settings.size;
		CAM_DBG(CAM_OIS,"read length: %d addr type=%d data type=%d",
			read_length,
			i2c_list->i2c_settings.addr_type,
			i2c_list->i2c_settings.data_type);
		mutex_lock(&(o_ctrl->ois_hall_data_mutex));
		fifo_len_in_ois_driver = kfifo_len(&(o_ctrl->ois_hall_data_fifoV2));
		CAM_DBG(CAM_OIS,"ffio length=%d",fifo_len_in_ois_driver);
		if (fifo_len_in_ois_driver > SAMPLE_SIZE_IN_DRIVER*SAMPLE_COUNT_IN_NCS_DATA) 
		{
			fifo_len_in_ois_driver = SAMPLE_SIZE_IN_DRIVER*SAMPLE_COUNT_IN_NCS_DATA;
		}

		if(fifo_len_in_ois_driver>0)
		{
			read_fifo_length = kfifo_out(&(o_ctrl->ois_hall_data_fifoV2),temp_buff, fifo_len_in_ois_driver);
                       sample_count=(read_fifo_length/SAMPLE_SIZE_IN_DRIVER);
                       CAM_DBG(CAM_OIS,"read sample count=%d", sample_count);
                       if(sample_count>0){
                            for(j=0;j<sample_count;j++){
                                read_buff[j*4]   = temp_buff[12*j+11];
                                read_buff[j*4+1] = temp_buff[12*j+10];
                                read_buff[j*4+2] = temp_buff[12*j+9];
                                read_buff[j*4+3] = temp_buff[12*j+8];
                                CAM_DBG(CAM_OIS,"hall data=0x%x 0x%x",
									((read_buff[j*4]<<8)+read_buff[j*4+1]),
									((read_buff[j*4+2]<<8)+read_buff[j*4+3]));
                            }
                            //set the Byte as sample count;all is 204ByTes
                            read_buff[143] = sample_count;
                            //set last 8 BYTES as last sample Qtimer
                            for ( j = 0; j < 8; j++)
                            {
                                if(j<4)
                                        ticks = (ticks << 8) | (temp_buff[(sample_count-1)*12+3-j]);
                                else
                                        ticks = (ticks << 8) | (temp_buff[(sample_count-1)*12+11-j]);
                            }

                            //ticks=((uint64_t)(hight<<32))+(uint64_t)low;

                            CAM_DBG(CAM_OIS,"ticks=%lld 0x%llx",ticks,ticks);
                            qtimer=mul_u64_u32_div(ticks,QTIMER_MUL_FACTOR, QTIMER_DIV_FACTOR);
                            CAM_DBG(CAM_OIS,"qtimer=%lld  0x%llx",qtimer,qtimer);
                            pre_qtimer = qtimer;

                            for (j = 0; j < 8; j++) {
                                    CAM_DBG(CAM_OIS, "time: reg_data[%d]: 0x%x",
                                            135-j, (qtimer & 0xFF));
                                    read_buff[135-j] =
                                            (qtimer & 0xFF);
                                    qtimer >>= 8;
                            }
                            CAM_DBG(CAM_OIS,"Qtimer=%x %x %x %x %x %x %x %x",
								read_buff[128],
								read_buff[129],
								read_buff[130],
								read_buff[131],
								read_buff[132],
								read_buff[133],
								read_buff[134],
								read_buff[135]);
                       }
                }else {
                        CAM_DBG(CAM_OIS,"no fifo data ,set Qtimer as before,sample count=0");
                        read_buff[143] = 0;
                        for (j = 0; j < 8; j++) {
                                    CAM_DBG(CAM_OIS, "time: reg_data[%d]: 0x%x",
                                            135-j, (pre_qtimer & 0xFF));
                                    read_buff[135-j] =
                                            (pre_qtimer & 0xFF);
                                    pre_qtimer >>= 8;
                        }
                }
                mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
        }
        kfree(temp_buff);
        return 0;
}

int OIS_READ_HALL_DATA_TO_UMD_129 (struct cam_ois_ctrl_t *o_ctrl,struct i2c_settings_array *i2c_settings) {

	int32_t 						rc = 0,i=0;
	struct i2c_settings_list	   *i2c_list;
	uint8_t 					   *read_buff = NULL;
	uint32_t						buff_length = 0;
	uint32_t						read_length = 0;
	uint64_t						fifo_count = 0,qtime_ms=0,qtime_us=0;
	uint64_t						preqtime_ms=0,newqtime=0;
	uint32_t						ls_hall_data_x = 0,ls_hall_data_y = 0;
	uint32_t						ss_hall_data_a = 0,ss_hall_data_b = 0,ss_hall_data_r = 0;
	int 							j=0;
	uint8_t 					   *temp_buff = NULL;



	list_for_each_entry(i2c_list,
		&(i2c_settings->list_head), list) {
		read_buff = i2c_list->i2c_settings.read_buff;
		buff_length = i2c_list->i2c_settings.read_buff_len;
		if ((read_buff == NULL) || (buff_length == 0)) {
				CAM_DBG(CAM_OIS,
						"Invalid input buffer, buffer: %pK, length: %d",
						read_buff, buff_length);
				return -EINVAL;
		}
		read_length = i2c_list->i2c_settings.size;
		temp_buff = kzalloc(read_length, GFP_KERNEL);
		memset(temp_buff, 0, read_length);
		CAM_DBG(CAM_OIS,"buffer: %pK, fbufer_length: %d  read_length=%d",read_buff, buff_length,read_length);
		CAM_DBG(CAM_OIS,"start read");
		OISCountinueRead(o_ctrl, 0xF111, (void *)temp_buff, read_length);
		CAM_DBG(CAM_OIS,"read done");
		/* ois data count is 2*6*16 = 192 Bytes and have max sample is 12;
							last Bytes is sample count,and 14/15 Bytes means Qtimer */

		fifo_count = temp_buff[191];
		read_buff[191]=temp_buff[191];

		if(fifo_count > 12) {
				CAM_DBG(CAM_OIS,"ois have drop data fifo_count=%d",fifo_count);
				fifo_count = 12;
				read_buff[191] = fifo_count;

		}

		for(j=0;j<fifo_count;j++)
		{
			for(i = 0; i < 12; i ++){
				read_buff[j*12 + i]=temp_buff[(fifo_count-j-1)*12 + i];
			}
		}
		for(j=0;j<4;j++){
				read_buff[144+j]=temp_buff[144+j];
				read_buff[148+j]=temp_buff[156+j];
		}

		for ( j = 0; j < 8; j++)
		{
				 newqtime = (newqtime << 8) | (read_buff[144+j]);
		}
		qtime_ms = (((uint32_t)read_buff[144]<<24)+((uint32_t)read_buff[145]<<16)+((uint32_t)read_buff[146]<<8)+(uint32_t)read_buff[147]);
		qtime_us = (((uint32_t)read_buff[156]<<24)+((uint32_t)read_buff[157]<<16)+((uint32_t)read_buff[158]<<8)+(uint32_t)read_buff[159]);
		if((newqtime-(fifo_count-1)*2000000)<preqtime_ms)
				CAM_DBG(CAM_OIS,"time error");
		preqtime_ms=newqtime;
		CAM_DBG(CAM_OIS,"READ fifo_count=%d",fifo_count);
		CAM_DBG(CAM_OIS,"Rqtimer value: ms=0x%x us=0x%x",qtime_ms,qtime_us);

		if(fifo_count > 0) {
			for(i=0 ; i<12; i++){
				ls_hall_data_x=((read_buff[i*12]<<8)+read_buff[i*12+1]);
				ls_hall_data_y=((read_buff[i*12+2]<<8)+read_buff[i*12+3]);
				ss_hall_data_a =((read_buff[i*12+4]<<8)+read_buff[i*12+5]);
				ss_hall_data_b = ((read_buff[i*12+6]<<8)+read_buff[i*12+7]);
				ss_hall_data_r =((read_buff[i*12+8]<<8)+read_buff[i*12+9]);
				CAM_DBG(CAM_OIS,"ls_hall_data_x=0x%x ls_hall_data_y=0x%x "
					"ss_hall_data_a=0x%x ss_hall_data_b=0x%x  ss_hall_data_r=0x%x",
					ls_hall_data_x,
					ls_hall_data_y,
					ss_hall_data_a,
					ss_hall_data_b,
					ss_hall_data_r);
			}
		}
	}
	kfree(temp_buff);
	return rc;
}

int OIS_READ_HALL_DATA_TO_UMD_Bu24721 (struct cam_ois_ctrl_t *o_ctrl,struct i2c_settings_array *i2c_settings) {

	int32_t 						rc = 0,i=0;
	struct i2c_settings_list	   *i2c_list;
	uint8_t 					   *read_buff = NULL;
	uint32_t						buff_length = 0;
	uint32_t						read_length = 0;
	uint64_t						fifo_count = 0;
	uint64_t						preqtime_ms=0,newqtime=0;
	uint32_t						hall_data_x = 0,hall_data_y = 0;
	int 							j=0;
	uint8_t 					   *temp_buff = NULL;
	uint32_t						delayCount = 0, delayTime = 0;
	uint64_t						qtimer_ns = 0;



	list_for_each_entry(i2c_list,
		&(i2c_settings->list_head), list) {
		read_buff = i2c_list->i2c_settings.read_buff;
		buff_length = i2c_list->i2c_settings.read_buff_len;
		if ((read_buff == NULL) || (buff_length == 0)) {
				CAM_DBG(CAM_OIS,
						"Invalid input buffer, buffer: %pK, length: %d",
						read_buff, buff_length);
				return -EINVAL;
		}
		read_length = OIS_HALL_DATA_SIZE;
		temp_buff = kzalloc(read_length, GFP_KERNEL);
		memset(temp_buff, 0, read_length);
		CAM_DBG(CAM_OIS,"buffer: %pK, fbufer_length: %d  read_length=%d",read_buff, buff_length,read_length);
		CAM_DBG(CAM_OIS,"start read");
		OISCountinueRead(o_ctrl, 0xF200, (void *)temp_buff, read_length);
		cam_sensor_util_get_current_qtimer_ns(&qtimer_ns);

		CAM_DBG(CAM_OIS,"read done");

		fifo_count = temp_buff[0] & 0xF;
		read_buff[56] = fifo_count;

		if(fifo_count > 12) {
				CAM_DBG(CAM_OIS,"ois have drop data fifo_count=%d",fifo_count);
				fifo_count = 12;
				read_buff[56] = fifo_count;
		}

		for(j=0; j<fifo_count * 4; j++)
		{
			read_buff[j]=temp_buff[j + 1];
		}

		delayCount = ((uint32_t)(temp_buff[j + 1] << 8) | temp_buff[j + 2]);
		delayTime = delayCount * 17800; //nano sec
		qtimer_ns -= delayTime;

		//set qtimer
		read_buff[48] = (qtimer_ns >> 56) & 0xFF;
		read_buff[49] = (qtimer_ns >> 48) & 0xFF;
		read_buff[50] = (qtimer_ns >> 40) & 0xFF;
		read_buff[51] = (qtimer_ns >> 32)& 0xFF;
		read_buff[52] = (qtimer_ns >> 24) & 0xFF;
		read_buff[53] = (qtimer_ns >> 16) & 0xFF;
		read_buff[54] = (qtimer_ns >> 8) & 0xFF;
		read_buff[55] = qtimer_ns & 0xFF;

		if((newqtime-(fifo_count-1)*2000000)<preqtime_ms)
				CAM_DBG(CAM_OIS,"time error");
		preqtime_ms=newqtime;
		CAM_DBG(CAM_OIS,"READ fifo_count=%d",fifo_count);
		CAM_DBG(CAM_OIS,"Rqtimer value: 0x%x", qtimer_ns);

		if(fifo_count > 0) {
			for(i=0 ; i<fifo_count; i++){
				hall_data_x=((read_buff[i*4]<<8)+read_buff[i*4+1]);
				hall_data_y=((read_buff[i*4+2]<<8)+read_buff[i*4+3]);
				CAM_DBG(CAM_OIS,"hall_data_x=0x%x hall_data_y=0x%x ",
					hall_data_x,
					hall_data_y);
			}
		}
	}
	kfree(temp_buff);
	return rc;
}

void ReadOISHALLData(struct cam_ois_ctrl_t *o_ctrl, void *data)
{
	uint32_t data_size = 0;
	uint32_t fifo_len_in_ois_driver;

	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	fifo_len_in_ois_driver = kfifo_len(&(o_ctrl->ois_hall_data_fifo));
	if (fifo_len_in_ois_driver > 0)
	{
		int ret;
		if (fifo_len_in_ois_driver > SAMPLE_SIZE_IN_DRIVER*SAMPLE_COUNT_IN_DRIVER)
		{
			fifo_len_in_ois_driver = SAMPLE_SIZE_IN_DRIVER*SAMPLE_COUNT_IN_DRIVER;
		}
		ret = kfifo_to_user(&(o_ctrl->ois_hall_data_fifo),
			data,
			fifo_len_in_ois_driver,
			&data_size);
		CAM_DBG(CAM_OIS, "ois type=%d,Copied %d Bytes to UMD with return value %d",
			o_ctrl->ois_type,
			data_size,ret);
	}
	else
	{
		CAM_DBG(CAM_OIS, "ois type=%d,fifo_len is %d, no need copy to UMD",
			o_ctrl->ois_type,
			fifo_len_in_ois_driver);
	}

	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
}

void ReadOISHALLDataV2(struct cam_ois_ctrl_t *o_ctrl, void *data)
{
	uint32_t data_size = 0;
	uint32_t fifo_len_in_ois_driver;

	mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	fifo_len_in_ois_driver = kfifo_len(&(o_ctrl->ois_hall_data_fifoV2));
	if (fifo_len_in_ois_driver > 0)
	{
		int ret;
		if (fifo_len_in_ois_driver > SAMPLE_SIZE_IN_DRIVER*SAMPLE_COUNT_IN_DRIVER)
		{
			fifo_len_in_ois_driver = SAMPLE_SIZE_IN_DRIVER*SAMPLE_COUNT_IN_DRIVER;
		}
		ret = kfifo_to_user(&(o_ctrl->ois_hall_data_fifoV2),
			data,
			fifo_len_in_ois_driver,
			&data_size);
		CAM_DBG(CAM_OIS, "ois type=%d,Copied %d Bytes to UMD EISv2 with return value %d",
			o_ctrl->ois_type,
			data_size,ret);
	}
	else
	{
		CAM_DBG(CAM_OIS, "ois type=%d,fifo_len is %d, no need copy to UMD EISv2",
			o_ctrl->ois_type,
			fifo_len_in_ois_driver);
	}

	mutex_unlock(&(o_ctrl->ois_hall_data_mutex));
}

void ReadOISHALLDataV3(struct cam_ois_ctrl_t *o_ctrl, void *data)
{

	//mutex_lock(&(o_ctrl->ois_hall_data_mutex));
	//mutex_unlock(&(o_ctrl->ois_hall_data_mutex));

}

int OISControl(struct cam_ois_ctrl_t *o_ctrl)
{
	if (o_ctrl == NULL){
		CAM_ERR(CAM_OIS, "Invalid Args");
		return 0;
	}
/*
	if (strstr(o_ctrl->ois_name, "bu24721"))
	{
		mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
		o_ctrl->ois_poll_thread = kthread_run(OISPollThreadBu24721, o_ctrl, o_ctrl->ois_name);
		mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
	}
*/
	if(o_ctrl->ois_eis_function != 1)
	{
		return 0;
	}

	if (o_ctrl && (o_ctrl->ois_type != CAM_OIS_MASTER))
	{
		CAM_INFO(CAM_OIS, "ois type=%d, don't create OIS thread",o_ctrl->ois_type);
		return 0;
	}
	if (o_ctrl && (CAM_OIS_READY == ois_state[o_ctrl->ois_type]))
	{
		switch (o_ctrl->ois_poll_thread_control_cmd)
		{
			case CAM_OIS_START_POLL_THREAD:
				mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
				if (o_ctrl->ois_poll_thread)
				{
					CAM_INFO(CAM_OIS, "ois type=%d,ois_poll_thread is already created, no need to create again.",o_ctrl->ois_type);
				}
				else
				{
					o_ctrl->ois_poll_thread_exit = false;
					if (strstr(o_ctrl->ois_name, "128"))
					{
						o_ctrl->ois_poll_thread = kthread_run(OISPollThread128, o_ctrl, o_ctrl->ois_name);
					}
					else if (strstr(o_ctrl->ois_name, "124"))
					{
						//o_ctrl->ois_poll_thread = kthread_run(OISPollThread124, o_ctrl, o_ctrl->ois_name);
					}
					if (!o_ctrl->ois_poll_thread)
					{
						o_ctrl->ois_poll_thread_exit = true;
						CAM_ERR(CAM_OIS, "ois type=%d,create ois poll thread failed",o_ctrl->ois_type);
					}
				}
				mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));

				break;
			case CAM_OIS_STOP_POLL_THREAD:
				mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
				if (o_ctrl->ois_poll_thread)
				{
					o_ctrl->ois_poll_thread_exit = true;
					o_ctrl->ois_poll_thread = NULL;
				}
				mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));

				break;
		}
	}
	else
	{
		CAM_ERR(CAM_OIS, "o_ctrl=%p,ois_type=%d ois_state=%d",
			o_ctrl,
			o_ctrl->ois_type,
			ois_state[o_ctrl->ois_type]);
		return -1;
	}

	return 0;
}

bool IsOISReady(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t temp, retry_cnt;
	retry_cnt = 10;

	if (o_ctrl)
	{
		do
		{
			if (strstr(o_ctrl->ois_name, "bu24721"))
			{
				RohmOisRead(o_ctrl,0xF024, &temp);
				if ((temp&0x01) == 1)
				{
					CAM_ERR(CAM_OIS, "OIS %d IsOISReady", o_ctrl->ois_type);
					ois_state[o_ctrl->ois_type] = CAM_OIS_READY;
					return true;
				}
				msleep(5);
				CAM_ERR(CAM_OIS, "OIS %d 0xF024 = 0x%x", o_ctrl->ois_type, temp);
			}
			else
			{
				RamRead32A_oneplus(o_ctrl,0xF100, &temp);
				if (temp == 0)
				{
					CAM_INFO(CAM_OIS, "OIS %d IsOISReady", o_ctrl->ois_type);
					ois_state[o_ctrl->ois_type] = CAM_OIS_READY;
					return true;
				}
				CAM_ERR(CAM_OIS, "OIS %d 0xF100 = 0x%x", o_ctrl->ois_type, temp);
			}
			retry_cnt--;
			msleep(5);
		}
		while(retry_cnt);

		return false;
	} else {
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
		return false;
	}
}

void InitOIS(struct cam_ois_ctrl_t *o_ctrl)
{
	if (o_ctrl)
	{
		if (o_ctrl->ois_type == CAM_OIS_MASTER)
		{
			ois_state[CAM_OIS_MASTER] = CAM_OIS_INVALID;
		}
		else if (o_ctrl->ois_type == CAM_OIS_SLAVE)
		{
			ois_state[CAM_OIS_SLAVE] = CAM_OIS_INVALID;  //add to init
			CAM_ERR(CAM_OIS, "reset master to CAM_OIS_INVALID");
			if (ois_ctrls[CAM_OIS_MASTER])
			{
				ois_state[CAM_OIS_MASTER] = CAM_OIS_INVALID;
				if(ois_ctrls[CAM_OIS_MASTER]->ois_change_cci)
				{
					if(chip_version_old)
					{
						ois_ctrls[CAM_OIS_MASTER]->io_master_info.cci_client->cci_device = CCI_DEVICE_0;
						ois_ctrls[CAM_OIS_MASTER]->io_master_info.cci_client->cci_i2c_master = MASTER_0;
						ois_ctrls[CAM_OIS_MASTER]->cci_i2c_master = MASTER_0;
						CAM_INFO(CAM_OIS, "change old module cci");
					}
				}
				if (camera_io_init(&(ois_ctrls[CAM_OIS_MASTER]->io_master_info)))
				{
					CAM_ERR(CAM_OIS, "cci_init failed");
				}
			}
		}
		else
		{
			CAM_ERR(CAM_OIS, "ois_type 0x%x is wrong", o_ctrl->ois_type);
		}
	}
	else
	{
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
	}
}

void DeinitOIS(struct cam_ois_ctrl_t *o_ctrl)
{
	if (o_ctrl)
	{
		if (strstr(o_ctrl->ois_name, "bu24721") != NULL)
		{
			mutex_lock(&(o_ctrl->ois_poll_thread_mutex));
			if (o_ctrl->ois_poll_thread) {
					o_ctrl->ois_poll_thread_exit = true;
					o_ctrl->ois_poll_thread = NULL;
			}
			mutex_unlock(&(o_ctrl->ois_poll_thread_mutex));
		}

		if (o_ctrl->ois_type == CAM_OIS_MASTER)
		{
			if (o_ctrl->ois_read_thread_start_to_read&&ois_ctrls[CAM_OIS_MASTER])
			{
				ois_start_read_thread(ois_ctrls[CAM_OIS_MASTER], 0);
			}
			ois_state[CAM_OIS_MASTER] = CAM_OIS_INVALID;
		}
		else if (o_ctrl->ois_type == CAM_OIS_SLAVE)
		{
			if (ois_ctrls[CAM_OIS_MASTER])
			{
				/*donot start main camera thread when switch tele  
				if(ois_ctrls[CAM_OIS_MASTER]->ois_read_thread_start_to_read) {
					ois_start_read_thread(ois_ctrls[CAM_OIS_MASTER], 0);
				}
				*/
				if (camera_io_release(&(ois_ctrls[CAM_OIS_MASTER]->io_master_info)))
				{
					CAM_ERR(CAM_OIS, "ois type=%d,cci_deinit failed",ois_ctrls[CAM_OIS_MASTER]->ois_type);
				}
			}
			if (ois_ctrls[CAM_OIS_SLAVE])
			{
				if(o_ctrl->ois_read_thread_start_to_read)
				{
					ois_start_read_thread(ois_ctrls[CAM_OIS_SLAVE], 0);
				}
			}
			ois_state[CAM_OIS_SLAVE] = CAM_OIS_INVALID;

		}
		else
		{
			CAM_ERR(CAM_OIS, "ois_type 0x%x is wrong", o_ctrl->ois_type);
		}
	}
	else
	{
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
	}
}
int update_ois_firmware_bu24721(void *arg)
{
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
	int rc = 0;
	CAM_INFO(CAM_OIS, "update_ois_firmware_bu24721");
	o_ctrl->io_master_info.master_type = CCI_MASTER;
	o_ctrl->io_master_info.cci_client->sid = 0x3E;
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
	o_ctrl->io_master_info.cci_client->retries = 3;
	o_ctrl->io_master_info.cci_client->id_map = 0;

	ois_ctrls[CAM_OIS_SLAVE] = o_ctrl;
	ois_ctrl = o_ctrl;

	cam_ois_power_up(o_ctrl);
	msleep(10);

	rc = Rohm_ois_fw_download(o_ctrl);

	cam_ois_power_down(o_ctrl);
	CAM_INFO(CAM_OIS, "exit update_ois_firmware_bu24721 rc:%d", rc);
	return rc;
}

int update_ois_firmware_129(void *arg)
{
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
	int rc = 0;
	CAM_INFO(CAM_OIS, "update_ois_firmware_129");
	o_ctrl->io_master_info.master_type = CCI_MASTER;
	o_ctrl->io_master_info.cci_client->sid = 0x24;
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
	o_ctrl->io_master_info.cci_client->retries = 3;
	o_ctrl->io_master_info.cci_client->id_map = 0;
	ois_ctrls[CAM_OIS_MASTER] = o_ctrl;
	ois_ctrl = o_ctrl;

	cam_ois_power_up(o_ctrl);
	msleep(10);

	rc = FlashProgram129(o_ctrl->ois_module_vendor, o_ctrl->ois_actuator_vendor , o_ctrl);
	if(rc == 0)
	{
		CAM_INFO(CAM_OIS, "FlashProgram129 download successed!");
	}
	else {
		switch (rc&0xF0)
		{
			case 0x00:
				CAM_ERR(CAM_OIS, "Error ; during the rom boot changing. Also including 128 power off issue.");
				break;
			case 0x20:
				CAM_ERR(CAM_OIS, "Error ; during Initial program for updating to program memory.");
				break;
			case 0x30:
				CAM_ERR(CAM_OIS, "Error ; during User Mat area erasing.");
				break;
			case 0x40:
				CAM_ERR(CAM_OIS, "Error ; during User Mat area programing.");
				break;
			case 0x50:
				CAM_ERR(CAM_OIS, "Error ; during the verification.");
				break;
			case 0x90:
				CAM_ERR(CAM_OIS, "Error ; during the drive offset confirmation.");
				break;
			case 0xA0:
				CAM_ERR(CAM_OIS, "Error ; during the MAT2 re-write process.");
				break;
			case 0xF0:
				if (rc == 0xF0)
				{
					CAM_ERR(CAM_OIS, "mistake of module vendor designation.");
				}
				else if (rc == 0xF1)
				{
					CAM_ERR(CAM_OIS, "mistake size of From Code.");
				}
				break;
			default:
				CAM_ERR(CAM_OIS, "Unkown error code");
				break;
		}
	}

	cam_ois_power_down(o_ctrl);
	CAM_INFO(CAM_OIS, "exit update_ois_firmware_129 rc:%d", rc);

	return rc;
}

void InitOISResource(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc;
	mutex_init(&ois_mutex);
	if (o_ctrl)
	{
		if (strstr(o_ctrl->ois_name, "129"))
		{
			CAM_INFO(CAM_OIS, "create ois download fw thread");
			o_ctrl->ois_downloadfw_thread = kthread_run(update_ois_firmware_129, o_ctrl, o_ctrl->ois_name);
		}

		if (strstr(o_ctrl->ois_name, "bu24721"))
		{
			CAM_INFO(CAM_OIS, "create ois download fw thread");
			o_ctrl->ois_downloadfw_thread = kthread_run(update_ois_firmware_bu24721, o_ctrl, o_ctrl->ois_name);
		}

		if (o_ctrl->ois_type == CAM_OIS_MASTER)
		{
			ois_ctrls[CAM_OIS_MASTER] = o_ctrl;
			//Hardcode the parameters of main OIS, and those parameters will be overrided when open main camera
			o_ctrl->io_master_info.cci_client->sid = 0x24;
			o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
			o_ctrl->io_master_info.cci_client->retries = 3;
			o_ctrl->io_master_info.cci_client->id_map = 0;
			CAM_INFO(CAM_OIS, "ois type=%d,ois_ctrls[%d] = %p",
				o_ctrl->ois_type,
				CAM_OIS_MASTER,
				ois_ctrls[CAM_OIS_MASTER]);
		}
		else if (o_ctrl->ois_type == CAM_OIS_SLAVE)
		{
			ois_ctrls[CAM_OIS_SLAVE] = o_ctrl;
			CAM_INFO(CAM_OIS, "ois type=%d,ois_ctrls[%d] = %p",
				o_ctrl->ois_type,
				CAM_OIS_SLAVE,
				ois_ctrls[CAM_OIS_SLAVE]);
		} 
		else
		{
			CAM_ERR(CAM_OIS, "ois_type 0x%x is wrong", o_ctrl->ois_type);
		}

		if(!cam_ois_kobj)
		{
			cam_ois_kobj = kobject_create_and_add("ois_control", kernel_kobj);
			rc = sysfs_create_groups(cam_ois_kobj, ois_groups);
			if (rc != 0)
			{
				CAM_ERR(CAM_OIS,"Error creating sysfs ois group");
				sysfs_remove_groups(cam_ois_kobj, ois_groups);
			}
		}
		else
		{
			CAM_INFO(CAM_OIS, "ois_control node exist");
		}
	}
	else
	{
		CAM_ERR(CAM_OIS, "o_ctrl is NULL");
	}

	//Create OIS control node
	if(face_common_dir == NULL)
	{
		face_common_dir =  proc_mkdir("OIS", NULL);
		if(!face_common_dir)
		{
			CAM_ERR(CAM_OIS, "create dir fail CAM_ERROR API");
			//return FACE_ERROR_GENERAL;
		}
	}

	if(proc_file_entry == NULL)
	{
		proc_file_entry = proc_create("OISControl",
			0777,
			face_common_dir,
			&proc_file_fops);
		if(proc_file_entry == NULL)
		{
			CAM_ERR(CAM_OIS, "Create fail");
		}
		else
		{
			CAM_INFO(CAM_OIS, "Create successs");
		}
	}

	if(proc_file_entry_tele == NULL)
	{
		proc_file_entry_tele = proc_create("OISControl_tele",
			0777,
			face_common_dir,
			&proc_file_fops_tele);
		if(proc_file_entry_tele == NULL)
		{
			CAM_ERR(CAM_OIS, "Create fail");
		}
		else
		{
			CAM_INFO(CAM_OIS, "Create successs");
		}
	}
}

int32_t oplus_cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 5;
	power_info->power_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting)*5,
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 0;

	power_info->power_setting[1].seq_type = SENSOR_VIO;
	power_info->power_setting[1].seq_val = CAM_VIO;
	power_info->power_setting[1].config_val = 1;
	power_info->power_setting[1].delay = 0;

	power_info->power_setting[2].seq_type = SENSOR_VANA;
	power_info->power_setting[2].seq_val = CAM_VANA;
	power_info->power_setting[2].config_val = 1;
	power_info->power_setting[2].delay = 0;

	power_info->power_setting[3].seq_type = SENSOR_CUSTOM_REG1;
	power_info->power_setting[3].seq_val = CAM_V_CUSTOM1;
	power_info->power_setting[3].config_val = 1;
	power_info->power_setting[3].delay = 0;

	power_info->power_setting[4].seq_type = SENSOR_CUSTOM_REG2;
	power_info->power_setting[4].seq_val = CAM_V_CUSTOM2;
	power_info->power_setting[4].config_val = 1;
	power_info->power_setting[4].delay = 10;

	power_info->power_down_setting_size = 5;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting)*5,
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	power_info->power_down_setting[1].seq_type = SENSOR_VIO;
	power_info->power_down_setting[1].seq_val = CAM_VIO;
	power_info->power_down_setting[1].config_val = 0;

	power_info->power_down_setting[2].seq_type = SENSOR_VANA;
	power_info->power_down_setting[2].seq_val = CAM_VANA;
	power_info->power_down_setting[2].config_val = 0;

	power_info->power_down_setting[3].seq_type = SENSOR_CUSTOM_REG2;
	power_info->power_down_setting[3].seq_val = CAM_V_CUSTOM2;
	power_info->power_down_setting[3].config_val = 0;

	power_info->power_down_setting[4].seq_type = SENSOR_CUSTOM_REG1;
	power_info->power_down_setting[4].seq_val = CAM_V_CUSTOM1;
	power_info->power_down_setting[4].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}

int32_t oplus_cam_ois_construct_default_power_setting_bu24721(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 5;
	power_info->power_setting = kzalloc(sizeof(struct cam_sensor_power_setting)*5, GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VIO;
	power_info->power_setting[0].seq_val = CAM_VIO;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 0;

	power_info->power_setting[1].seq_type = SENSOR_VDIG;
	power_info->power_setting[1].seq_val = CAM_VDIG;
	power_info->power_setting[1].config_val = 1;
	power_info->power_setting[1].delay = 0;

	power_info->power_setting[2].seq_type = SENSOR_CUSTOM_REG1;
	power_info->power_setting[2].seq_val = CAM_V_CUSTOM1;
	power_info->power_setting[2].config_val = 1;
	power_info->power_setting[2].delay = 0;

	power_info->power_setting[3].seq_type = SENSOR_CUSTOM_REG2;
	power_info->power_setting[3].seq_val = CAM_V_CUSTOM2;
	power_info->power_setting[3].config_val = 1;
	power_info->power_setting[3].delay = 10;

	power_info->power_setting[4].seq_type = SENSOR_VAF;
	power_info->power_setting[4].seq_val = CAM_VAF;
	power_info->power_setting[4].config_val = 1;
	power_info->power_setting[4].delay = 10;

	power_info->power_down_setting_size = 5;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting)*5,
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	power_info->power_down_setting[1].seq_type = SENSOR_CUSTOM_REG2;
	power_info->power_down_setting[1].seq_val = CAM_V_CUSTOM2;
	power_info->power_down_setting[1].config_val = 0;

	power_info->power_down_setting[2].seq_type = SENSOR_CUSTOM_REG1;
	power_info->power_down_setting[2].seq_val = CAM_V_CUSTOM1;
	power_info->power_down_setting[2].config_val = 0;

	power_info->power_down_setting[3].seq_type = SENSOR_VDIG;
	power_info->power_down_setting[3].seq_val = CAM_VDIG;
	power_info->power_down_setting[3].config_val = 0;

	power_info->power_down_setting[4].seq_type = SENSOR_VIO;
	power_info->power_down_setting[4].seq_val = CAM_VIO;
	power_info->power_down_setting[4].config_val = 0;


	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}

int32_t oplus_cam_ois_construct_default_power_setting_129(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 6;
	power_info->power_setting = kzalloc(sizeof(struct cam_sensor_power_setting)*6, GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VIO;
	power_info->power_setting[0].seq_val = CAM_VIO;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 0;

	power_info->power_setting[1].seq_type = SENSOR_VDIG;
	power_info->power_setting[1].seq_val = CAM_VDIG;
	power_info->power_setting[1].config_val = 1;
	power_info->power_setting[1].delay = 0;

	power_info->power_setting[2].seq_type = SENSOR_CUSTOM_REG1;
	power_info->power_setting[2].seq_val = CAM_V_CUSTOM1;
	power_info->power_setting[2].config_val = 1;
	power_info->power_setting[2].delay = 0;

	power_info->power_setting[3].seq_type = SENSOR_VAF;
	power_info->power_setting[3].seq_val = CAM_VAF;
	power_info->power_setting[3].config_val = 1;
	power_info->power_setting[3].delay = 10;

	power_info->power_setting[4].seq_type = SENSOR_VANA;
	power_info->power_setting[4].seq_val = CAM_VANA;
	power_info->power_setting[4].config_val = 1;
	power_info->power_setting[4].delay = 0;

	power_info->power_setting[5].seq_type = SENSOR_CUSTOM_REG2;
	power_info->power_setting[5].seq_val = CAM_V_CUSTOM2;
	power_info->power_setting[5].config_val = 1;
	power_info->power_setting[5].delay = 10;

	power_info->power_down_setting_size = 6;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting)*6,
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	power_info->power_down_setting[1].seq_type = SENSOR_VIO;
	power_info->power_down_setting[1].seq_val = CAM_VIO;
	power_info->power_down_setting[1].config_val = 0;

	power_info->power_down_setting[2].seq_type = SENSOR_VANA;
	power_info->power_down_setting[2].seq_val = CAM_VANA;
	power_info->power_down_setting[2].config_val = 0;

	power_info->power_down_setting[3].seq_type = SENSOR_CUSTOM_REG2;
	power_info->power_down_setting[3].seq_val = CAM_V_CUSTOM2;
	power_info->power_down_setting[3].config_val = 0;

	power_info->power_down_setting[4].seq_type = SENSOR_CUSTOM_REG1;
	power_info->power_down_setting[4].seq_val = CAM_V_CUSTOM1;
	power_info->power_down_setting[4].config_val = 0;

	power_info->power_down_setting[5].seq_type = SENSOR_VDIG;
	power_info->power_down_setting[5].seq_val = CAM_VDIG;
	power_info->power_down_setting[5].config_val = 0;


	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}

int cam_ois_slaveInfo_pkt_parser_oem(struct cam_ois_ctrl_t *o_ctrl)
{
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
	o_ctrl->io_master_info.cci_client->sid = (0x7c >> 1);
	o_ctrl->io_master_info.cci_client->retries = 3;
	o_ctrl->io_master_info.cci_client->id_map = 0;

	return 0;
}

int ois_download_fw_thread(void *arg)
{
	struct cam_ois_ctrl_t *o_ctrl = (struct cam_ois_ctrl_t *)arg;
	int rc = -1;
	mutex_lock(&(o_ctrl->ois_mutex));
	cam_ois_slaveInfo_pkt_parser_oem(o_ctrl);

	if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			mutex_lock(&(o_ctrl->ois_power_down_mutex));
			o_ctrl->ois_power_down_thread_exit = true;
			if (o_ctrl->ois_power_state == CAM_OIS_POWER_OFF){
					rc = cam_ois_power_up(o_ctrl);
					if(rc != 0) {
							CAM_ERR(CAM_OIS, "ois power up failed");
							mutex_unlock(&(o_ctrl->ois_power_down_mutex));
							mutex_unlock(&(o_ctrl->ois_mutex));
							return rc;
					}
			} else {
					CAM_ERR(CAM_OIS, "ois type=%d,OIS already power on, no need to power on again",o_ctrl->ois_type);
			}
			CAM_ERR(CAM_OIS, "ois[%s] type:%d  power up successful",o_ctrl->ois_type,o_ctrl->ois_name);
			o_ctrl->ois_power_state = CAM_OIS_POWER_ON;
			mutex_unlock(&(o_ctrl->ois_power_down_mutex));
	}

	o_ctrl->cam_ois_state = CAM_OIS_CONFIG;

	mutex_unlock(&(o_ctrl->ois_mutex));

	mutex_lock(&(o_ctrl->do_ioctl_ois));
	if(o_ctrl->ois_download_fw_done == CAM_OIS_FW_NOT_DOWNLOAD)
	{
		rc = DownloadFW(o_ctrl);
		if(rc != 0)
		{
			CAM_ERR(CAM_OIS, "ois download fw failed");
			o_ctrl->ois_download_fw_done = CAM_OIS_FW_NOT_DOWNLOAD;
			mutex_unlock(&(o_ctrl->do_ioctl_ois));
			return rc;
		}
		else
		{
			o_ctrl->ois_download_fw_done = CAM_OIS_FW_DOWNLOAD_DONE;
		}
	}
	RamWrite32A_oneplus(o_ctrl,0xf012,0x0);
	RamWrite32A_oneplus(o_ctrl,0xf010,0x0);
	mutex_unlock(&(o_ctrl->do_ioctl_ois));
	return rc;
}

int cam_ois_download_start(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	mutex_lock(&(o_ctrl->ois_mutex));
	mutex_lock(&(o_ctrl->ois_power_down_mutex));
	if (o_ctrl->ois_power_state == CAM_OIS_POWER_ON)
	{
		CAM_INFO(CAM_OIS, "do not need to create ois download fw thread");
		o_ctrl->ois_power_down_thread_exit = true;
		mutex_unlock(&(o_ctrl->ois_power_down_mutex));
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		mutex_unlock(&(o_ctrl->ois_mutex));
		return rc;
	}
	else
	{
		CAM_INFO(CAM_OIS, "create ois download fw thread");
		o_ctrl->ois_downloadfw_thread = kthread_run(ois_download_fw_thread, o_ctrl, o_ctrl->ois_name);
		if (!o_ctrl->ois_downloadfw_thread)
		{
			CAM_ERR(CAM_OIS, "create ois download fw thread failed");
			mutex_unlock(&(o_ctrl->ois_power_down_mutex));
			mutex_unlock(&(o_ctrl->ois_mutex));
			return -1;
		}
	}
	mutex_unlock(&(o_ctrl->ois_power_down_mutex));
	mutex_lock(&(o_ctrl->do_ioctl_ois));
	o_ctrl->ois_fd_have_close_state = CAM_OIS_IS_OPEN;
	mutex_unlock(&(o_ctrl->do_ioctl_ois));
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}

void cam_ois_do_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	mutex_lock(&(o_ctrl->ois_mutex));

	//when close ois,should be disable ois
	mutex_lock(&(o_ctrl->ois_power_down_mutex));
	if (o_ctrl->ois_power_state == CAM_OIS_POWER_ON)
	{
		RamWrite32A_oneplus(o_ctrl,0xf012,0x0);
	}
	mutex_unlock(&(o_ctrl->ois_power_down_mutex));
	mutex_lock(&(o_ctrl->do_ioctl_ois));
	o_ctrl->ois_fd_have_close_state = CAM_OIS_IS_DOING_CLOSE;
	mutex_unlock(&(o_ctrl->do_ioctl_ois));

	cam_ois_shutdown(o_ctrl);

	mutex_unlock(&(o_ctrl->ois_mutex));
}

void cam_set_ois_disable(struct cam_ois_ctrl_t *o_ctrl)
{
	if(strstr(o_ctrl->ois_name, "129"))
	{
		RamWrite32A(0xF016, 0x00000000 );
		IsOISReady(o_ctrl);
		CAM_INFO(CAM_OIS, "set 0xF016 0x00000000");

		RamWrite32A(0xF01C, 0x00000000 );
		IsOISReady(o_ctrl);
		CAM_INFO(CAM_OIS, "set 0xF01C 0x00000000");

		RamWrite32A(0xF012, 0x00000000 );
		IsOISReady(o_ctrl);
		CAM_INFO(CAM_OIS, "set 0xF012 0x00000000");

	}

}

/****************************   Rohm FW  function    ********************************/

int Rohm_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t     rc = 0;
	struct timespec64 mStartTime, mEndTime, diff;
	uint64_t mSpendTime = 0;
	ktime_get_real_ts64(&mStartTime);

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	if(strstr(o_ctrl->ois_name, "bu24721"))
	{
		//1.download Fw
		CAM_INFO(CAM_OIS, "Tele OIS download FW ...");
		rc = Rohm_bu24721_fw_download(o_ctrl->isTeleOisUseMonitor);
		if(!rc)
		{
			//2.checksum

			//3.config para
		}
		else
		{
			CAM_ERR(CAM_OIS, "Tele OIS download FW Failed rc = %d",rc);
		}
	}

	ktime_get_real_ts64(&mEndTime);
	diff = timespec64_sub(mEndTime, mStartTime);
	mSpendTime = (timespec64_to_ns(&diff))/1000000;

	CAM_INFO(CAM_OIS, "tele ois FW download rc=%d, (Spend: %d ms)", rc, mSpendTime);
	return rc;
}

uint8_t I2C_OIS_8bit__read(uint32_t addr)
{
	uint32_t data = 0xff;
	int32_t rc = 0;

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	rc = camera_io_dev_read(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info), addr,&data,
	               CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE, false);
	return (uint8_t)data;
}

uint16_t I2C_OIS_16bit__read(uint32_t addr)
{
	uint32_t data = 0xff;
	int32_t rc = 0;

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = camera_io_dev_read(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info), addr,&data,
	               CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD, false);
	return (uint16_t)data;
}

int  I2C_OIS_32bit__read(uint32_t addr, uint32_t* data)
{
	//uint32_t data = 0xff;
	int32_t rc = 0;

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = camera_io_dev_read(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info), addr, data,
	               CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD, false);
	return rc;
}

int I2C_OIS_8bit_write(uint32_t addr, uint8_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x00,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.delay = 0x00,
	};

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "ois type=%d,I2C_OIS_8bit_write 0x%04x failed, retry:%d",ois_ctrls[CAM_OIS_SLAVE]->ois_type, addr, i+1);
		} else {
            CAM_DBG(CAM_OIS, "I2C_OIS_8bit_write success ois type=%d,write 0x%04x ,data=%d",ois_ctrls[CAM_OIS_SLAVE]->ois_type, addr,data);
			return rc;
		}
	}
	return rc;
}

int I2C_OIS_16bit_write(uint32_t addr, uint16_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;

	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x00,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.delay = 0x00,
	};

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "ois type=%d,I2C_OIS_8bit_write 0x%04x failed, retry:%d",ois_ctrls[CAM_OIS_SLAVE]->ois_type, addr, i+1);
		} else {
            CAM_DBG(CAM_OIS, "I2C_OIS_8bit_write success ois type=%d,write 0x%04x ,data=%d",ois_ctrls[CAM_OIS_SLAVE]->ois_type, addr,data);
			return rc;
		}
	}
	return rc;
}

void I2C_OIS_block_write(void* register_data,int size)
{
	uint8_t *data =  (uint8_t *)register_data;
	int32_t rc = 0;
	int i = 0;
	int reg_data_cnt = size - 2;
	int continue_cnt = 0;
	int retry = 3;
	static struct cam_sensor_i2c_reg_array *i2c_write_setting_gl = NULL;

	struct cam_sensor_i2c_reg_setting i2c_write;

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return;
	}

	if (i2c_write_setting_gl == NULL)
	{
		i2c_write_setting_gl = (struct cam_sensor_i2c_reg_array *)kzalloc(
		                           sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2, GFP_KERNEL);
		if(!i2c_write_setting_gl)
		{
			CAM_ERR(CAM_OIS, "Alloc i2c_write_setting_gl failed");
			return;
		}
	}

	memset(i2c_write_setting_gl, 0, sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2);

	for(i = 0; i< reg_data_cnt; i++)
	{
		if (i == 0)
		{
			i2c_write_setting_gl[continue_cnt].reg_addr = ((data[0]&0xff)<<8)|(data[1]&0xff);
			i2c_write_setting_gl[continue_cnt].reg_data = data[2];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
			CAM_ERR(CAM_OIS, "i2c_write_setting_gl[%d].reg_addr = %04x data:0x%02x %02x %02x %02x",continue_cnt,i2c_write_setting_gl[continue_cnt].reg_addr,data[2],data[3],data[4],data[5]);
		}
		else
		{
			i2c_write_setting_gl[continue_cnt].reg_data = data[i+2];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
			//CAM_ERR(CAM_OIS, "i2c_write_setting_gl[%d].reg_addr = %x data:%x",continue_cnt,i2c_write_setting_gl[continue_cnt].reg_addr,i2c_write_setting_gl[continue_cnt].reg_data);
		}
		continue_cnt++;
	}
	i2c_write.reg_setting = i2c_write_setting_gl;
	i2c_write.size = continue_cnt;
	i2c_write.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_write.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_write.delay = 0x02;

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write_continuous(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info),
		                                    &i2c_write, 1);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,Continue write failed, rc:%d, retry:%d",ois_ctrls[CAM_OIS_SLAVE]->ois_type, rc, i+1);
		}
		else
		{
			break;
		}
	}
}


//FW

uint8_t I2C_FW_8bit__read(uint32_t addr)
{
	uint32_t data = 0xff;
	int32_t rc = 0;
	uint32_t sid_defult = 0;

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	sid_defult = ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid;
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = FLASH_SLVADR; //flash addr

	rc = camera_io_dev_read(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info), addr,&data,
	               CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE, false);
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = sid_defult;

	return (uint8_t)data;
}

uint32_t I2C_FM_32bit__read(uint32_t addr)
{
	uint32_t data = 0xff;
	int32_t rc = 0;
	uint32_t sid_defult = 0;

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	sid_defult = ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid;
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = FLASH_SLVADR; //flash addr
	rc = camera_io_dev_read(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info), addr,&data,
	               CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD, false);
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = sid_defult;
	return data;
}

uint32_t I2C_FM_16bit__read(uint32_t addr)
{
	uint32_t data = 0xff;
	int32_t rc = 0;
	uint32_t sid_defult = 0;

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	sid_defult = ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid;
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = FLASH_SLVADR; //flash addr
	rc = camera_io_dev_read(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info), addr,&data,
	               CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD, false);
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = sid_defult;
	return data;
}

int I2C_FM_8bit_write(uint32_t addr, uint8_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;
	uint32_t sid_defult = 0;

	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x00,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.delay = 0x00,
	};

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	sid_defult = ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid;
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = FLASH_SLVADR; //flash addr

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "ois type=%d,I2C_OIS_8bit_write 0x%04x failed, retry:%d",ois_ctrls[CAM_OIS_SLAVE]->ois_type, addr, i+1);
		}
		else
		{
			CAM_DBG(CAM_OIS, "I2C_OIS_8bit_write success ois type=%d,write 0x%04x ,data=%x",ois_ctrls[CAM_OIS_SLAVE]->ois_type, addr,data);
			ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = sid_defult;
			return rc;
		}
	}
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = sid_defult;
	return rc;
}

int I2C_FM_16bit_write(uint32_t addr, uint16_t data)
{
	int32_t rc = 0;
	int retry = 3;
	int i;
	uint32_t sid_defult = 0;

	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x00,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.delay = 0x00,
	};

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	sid_defult = ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid;
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = FLASH_SLVADR; //flash addr

	for(i = 0; i < retry; i++) {
		rc = camera_io_dev_write(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "ois type=%d,I2C_OIS_8bit_write 0x%04x failed, retry:%d",ois_ctrls[CAM_OIS_SLAVE]->ois_type, addr, i+1);
		} else {
			CAM_DBG(CAM_OIS, "I2C_OIS_8bit_write success ois type=%d,write 0x%04x ,data=%d",ois_ctrls[CAM_OIS_SLAVE]->ois_type, addr,data);
			ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = sid_defult;
			return rc;
		}
	}
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = sid_defult;
	return rc;
}

void I2C_FM_block_write(void* register_data,int size)
{
	//uint8_t *register_data_addr = (uint8_t *)register_data;
	uint8_t *data =  (uint8_t *)register_data;
	int32_t rc = 0;
	int i = 0;
	int reg_data_cnt = size - 2;
	int continue_cnt = 0;
	int retry = 3;
	uint32_t sid_defult = 0;
	static struct cam_sensor_i2c_reg_array *i2c_write_setting_gl = NULL;

	struct cam_sensor_i2c_reg_setting i2c_write;

	if (ois_ctrls[CAM_OIS_SLAVE] == NULL)
	{
		CAM_ERR(CAM_OIS, "Invalid Args");
		return;
	}

	if (i2c_write_setting_gl == NULL)
	{
		i2c_write_setting_gl = (struct cam_sensor_i2c_reg_array *)kzalloc(
		                           sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2, GFP_KERNEL);
		if(!i2c_write_setting_gl)
		{
			CAM_ERR(CAM_OIS, "Alloc i2c_write_setting_gl failed");
			return;
		}
	}

	memset(i2c_write_setting_gl, 0, sizeof(struct cam_sensor_i2c_reg_array)*MAX_DATA_NUM*2);

	for(i = 0; i< reg_data_cnt; i++)
	{
		if (i == 0)
		{
			i2c_write_setting_gl[continue_cnt].reg_addr = ((data[0]&0xff)<<8)|(data[1]&0xff);
			i2c_write_setting_gl[continue_cnt].reg_data = data[2];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
			//CAM_ERR(CAM_OIS, "i2c_write_setting_gl[%d].reg_addr = %04x data:0x%02x %02x %02x %02x",continue_cnt,i2c_write_setting_gl[continue_cnt].reg_addr,data[2],data[3],data[4],data[5]);
		}
		else
		{
			i2c_write_setting_gl[continue_cnt].reg_data = data[i+2];
			i2c_write_setting_gl[continue_cnt].delay = 0x00;
			i2c_write_setting_gl[continue_cnt].data_mask = 0x00;
			//CAM_ERR(CAM_OIS, "i2c_write_setting_gl[%d].reg_addr = %x data:%x",continue_cnt,i2c_write_setting_gl[continue_cnt].reg_addr,i2c_write_setting_gl[continue_cnt].reg_data);
		}
		continue_cnt++;
	}
	i2c_write.reg_setting = i2c_write_setting_gl;
	i2c_write.size = continue_cnt;
	i2c_write.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_write.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_write.delay = 0x01;

	sid_defult = ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid;
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = FLASH_SLVADR; //flash addr

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write_continuous(&(ois_ctrls[CAM_OIS_SLAVE]->io_master_info),
		                                    &i2c_write, 1);
		if (rc < 0)
		{
			CAM_ERR(CAM_OIS, "ois type=%d,Continue write failed, rc:%d, retry:%d",ois_ctrls[CAM_OIS_SLAVE]->ois_type, rc, i+1);
		}
		else
		{
			break;
		}
	}
	ois_ctrls[CAM_OIS_SLAVE]->io_master_info.cci_client->sid = sid_defult;
}




void Wait(int us)
{
	msleep((us+999)/1000);
}


