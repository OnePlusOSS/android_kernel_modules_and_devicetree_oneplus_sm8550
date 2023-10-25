/**************************************************************
 * Copyright (c)  2008- 2030  Oplus Mobile communication Corp.ltd.
 * File       : goodix_drivers_brl.c
 * Description: Source file for Goodix GT9897 driver
 * Version   : 1.0
 * Date        : 2019-08-27
 * Author    : bsp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 ****************************************************************/
#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>

#include "goodix_brl_core.h"

#define SPI_TRANS_PREFIX_LEN    1
#define REGISTER_WIDTH          4
#define SPI_READ_DUMMY_LEN      3
#define SPI_READ_PREFIX_LEN	\
		(SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH + SPI_READ_DUMMY_LEN)
#define SPI_WRITE_PREFIX_LEN	(SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH)

#define SPI_WRITE_FLAG		    0xF0
#define SPI_READ_FLAG		    0xF1

#define GOODIX_BUS_RETRY_TIMES  3

#define SPI_TRANS_PREFIX_LEN    1
#define REGISTER_WIDTH          4
#define SPI_READ_DUMMY_LEN      3
#define SPI_READ_PREFIX_LEN     (SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH + SPI_READ_DUMMY_LEN)
#define SPI_WRITE_PREFIX_LEN    (SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH)

#define SPI_WRITE_FLAG          0xF0
#define SPI_READ_FLAG           0xF1

/* shortcircuit info */
#define CHN_VDD					0xFF
#define CHN_GND					0x7F
#define DRV_CHANNEL_FLAG		0x80

#define MAX_TEST_TIME_MS        15000
#define DEFAULT_TEST_TIME_MS	7000

#define MAX_DRV_NUM_BRB				    	52
#define MAX_SEN_NUM_BRB				    	75
#define SHORT_TEST_TIME_REG_BRB				0x26AE0
#define DFT_ADC_DUMP_NUM_BRB				762
#define DFT_SHORT_THRESHOLD_BRB				100
#define DFT_DIFFCODE_SHORT_THRESHOLD_BRB	32
#define SHORT_TEST_STATUS_REG_BRB			0x20400
#define SHORT_TEST_RESULT_REG_BRB			0x20410
#define DRV_DRV_SELFCODE_REG_BRB			0x2049A
#define SEN_SEN_SELFCODE_REG_BRB 			0x21AF2
#define DRV_SEN_SELFCODE_REG_BRB			0x248A6
#define DIFF_CODE_DATA_REG_BRB				0x269E0

#define SHORT_TEST_FINISH_FLAG  			0x88
#define SHORT_TEST_THRESHOLD_REG			0x20402
#define SHORT_TEST_RUN_REG					0x10400
#define SHORT_TEST_RUN_FLAG					0xAA
#define INSPECT_FW_SWITCH_CMD				0x85

typedef struct __attribute__((packed)) {
	u8 result;
	u8 drv_drv_num;
	u8 sen_sen_num;
	u8 drv_sen_num;
	u8 drv_gnd_avdd_num;
	u8 sen_gnd_avdd_num;
	u16 checksum;
} test_result_t;


static int  goodix_reset(void *chip_data);

int goodix_spi_read(struct chip_data_brl *chip_info, unsigned int addr,
		    unsigned char *data, unsigned int len)
{
	struct spi_device *spi = chip_info->s_client;
	u8 *rx_buf = NULL;
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;

	rx_buf = kzalloc(SPI_READ_PREFIX_LEN + len, GFP_KERNEL);
	if (!rx_buf) {
		TPD_INFO("GT:rx_buf kzalloc error\n");
		ret = -ENOMEM;
		return ret;
	}
	tx_buf = kzalloc(SPI_READ_PREFIX_LEN + len, GFP_KERNEL);
	if (!tx_buf) {
		TPD_INFO("GT:tx_buf kzalloc error\n");
		ret = -ENOMEM;
		goto err_tx_buf_alloc;
	}

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	/*spi_read tx_buf format: 0xF1 + addr(4bytes) + data*/
	tx_buf[0] = SPI_READ_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	tx_buf[5] = 0xFF;
	tx_buf[6] = 0xFF;
	tx_buf[7] = 0xFF;

	xfers.tx_buf = tx_buf;
	xfers.rx_buf = rx_buf;
	xfers.len = SPI_READ_PREFIX_LEN + len;
	xfers.cs_change = 0;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		TPD_INFO("GT:spi transfer error:%d\n", ret);
		goto exit;
	}
	memcpy(data, &rx_buf[SPI_READ_PREFIX_LEN], len);

exit:
	kfree(tx_buf);
err_tx_buf_alloc:
	kfree(rx_buf);

	return ret;
}

int goodix_spi_write(struct chip_data_brl *chip_info, unsigned int addr,
		     unsigned char *data, unsigned int len)
{
	struct spi_device *spi = chip_info->s_client;
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;

	tx_buf = kzalloc(SPI_WRITE_PREFIX_LEN + len, GFP_KERNEL);
	if (!tx_buf) {
		TPD_INFO("GT:alloc tx_buf failed, size:%d\n",
			 SPI_WRITE_PREFIX_LEN + len);
		return -ENOMEM;
	}

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	tx_buf[0] = SPI_WRITE_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	memcpy(&tx_buf[SPI_WRITE_PREFIX_LEN], data, len);
	xfers.tx_buf = tx_buf;
	xfers.len = SPI_WRITE_PREFIX_LEN + len;
	xfers.cs_change = 0;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		TPD_INFO("GT:spi transfer error:%d\n", ret);
	}

	kfree(tx_buf);
	return ret;
}

static int goodix_reg_read(struct chip_data_brl *chip_info, u32 addr,
			   u8 *data, u32 len)
{
	return goodix_spi_read(chip_info, addr, data, len);
}

static int goodix_reg_write(struct chip_data_brl *chip_info, u32 addr,
			    u8 *data, u32 len)
{
	return goodix_spi_write(chip_info, addr, data, len);
}

static int goodix_reg_write_confirm(struct chip_data_brl *chip_info,
				    u32 addr, u8 *data, u32 len)
{
	u8 *cfm, cfm_buf[32];
	int r, i;

	if (len > sizeof(cfm_buf)) {
		cfm = kzalloc(len, GFP_KERNEL);
		if (!cfm) {
			TPD_INFO("GT:Mem alloc failed\n");
			return -ENOMEM;
		}
	} else {
		cfm = &cfm_buf[0];
	}

	for (i = 0; i < 3; i++) {
		r = goodix_reg_write(chip_info, addr, data, len);
		if (r < 0) {
			goto exit;
		}
		r = goodix_reg_read(chip_info, addr, cfm, len);
		if (r < 0) {
			goto exit;
		}

		if (memcmp(data, cfm, len)) {
			r = -EMEMCMP;
			continue;
		} else {
			r = 0;
			break;
		}
	}

exit:
	if (cfm != &cfm_buf[0]) {
		kfree(cfm);
	}
	return r;
}

/*****************************************************************************
* goodix_append_checksum
* @summary
*    Calcualte data checksum with the specified mode.
*
* @param data
*   data need to be calculate
* @param len
*   data length
* @param mode
*   calculate for u8 or u16 checksum
* @return
*   return the data checksum value.
*
*****************************************************************************/
u32 goodix_append_checksum(u8 *data, int len, int mode)
{
	u32 checksum = 0;
	int i;

	checksum = 0;
	if (mode == CHECKSUM_MODE_U8_LE) {
		for (i = 0; i < len; i++) {
			checksum += data[i];
		}
	} else {
		for (i = 0; i < len; i+=2) {
			checksum += (data[i] + (data[i+1] << 8));
		}
	}

	if (mode == CHECKSUM_MODE_U8_LE) {
		data[len] = checksum & 0xff;
		data[len + 1] = (checksum >> 8) & 0xff;
		return 0xFFFF & checksum;
	}
	data[len] = checksum & 0xff;
	data[len + 1] = (checksum >> 8) & 0xff;
	data[len + 2] = (checksum >> 16) & 0xff;
	data[len + 3] = (checksum >> 24) & 0xff;
	return checksum;
}

/* checksum_cmp: check data valid or not
 * @data: data need to be check
 * @size: data length need to be check(include the checksum bytes)
 * @mode: compare with U8 or U16 mode
 * */
int checksum_cmp(const u8 *data, int size, int mode)
{
	u32 cal_checksum = 0;
	u32 r_checksum = 0;
	u32 i;

	if (mode == CHECKSUM_MODE_U8_LE) {
		if (size < 2) {
			return 1;
		}
		for (i = 0; i < size - 2; i++) {
			cal_checksum += data[i];
		}

		r_checksum = data[size - 2] + (data[size - 1] << 8);
		return (cal_checksum & 0xFFFF) == r_checksum ? 0 : 1;
	}

	if (size < 4) {
		return 1;
	}
	for (i = 0; i < size - 4; i += 2) {
		cal_checksum += data[i] + (data[i + 1] << 8);
	}
	r_checksum = data[size - 4] + (data[size - 3] << 8) +
		     (data[size - 2] << 16) + (data[size - 1] << 24);
	return cal_checksum == r_checksum ? 0 : 1;
}

/* command ack info */
#define CMD_ACK_IDLE             0x01
#define CMD_ACK_BUSY             0x02
#define CMD_ACK_BUFFER_OVERFLOW  0x03
#define CMD_ACK_CHECKSUM_ERROR   0x04
#define CMD_ACK_OK               0x80

#define GOODIX_CMD_RETRY 6
static int brl_send_cmd(struct chip_data_brl *chip_info,
			struct goodix_ts_cmd *cmd)
{
	int ret, retry, i;
	struct goodix_ts_cmd cmd_ack;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;

	if (!misc->cmd_addr) {
		TPD_INFO("GT:%s: invalied cmd addr\n", __func__);
		return -1;
	}
	cmd->state = 0;
	cmd->ack = 0;
	goodix_append_checksum(&(cmd->buf[2]), cmd->len - 2,
			       CHECKSUM_MODE_U8_LE);
	TPD_DEBUG("GT:cmd data %*ph\n", cmd->len, &(cmd->buf[2]));

	retry = 0;
	while (retry++ < GOODIX_CMD_RETRY) {
		ret = goodix_reg_write(chip_info,
				       misc->cmd_addr, cmd->buf, sizeof(*cmd));
		if (ret < 0) {
			TPD_INFO("GT:%s: failed write command\n", __func__);
			return ret;
		}
		for (i = 0; i < GOODIX_CMD_RETRY; i++) {
			/* check command result */
			ret = goodix_reg_read(chip_info,
					      misc->cmd_addr, cmd_ack.buf, sizeof(cmd_ack));
			if (ret < 0) {
				TPD_INFO("GT:%s: failed read command ack, %d\n",
					 __func__, ret);
				return ret;
			}
			TPD_DEBUG("GT:cmd ack data %*ph\n",
				  (int)sizeof(cmd_ack), cmd_ack.buf);
			if (cmd_ack.ack == CMD_ACK_OK) {
				usleep_range(2000, 2100);
				return 0;
			}

			if (cmd_ack.ack == CMD_ACK_BUSY ||
			    cmd_ack.ack == 0x00) {
				usleep_range(1000, 1100);
				continue;
			}
			if (cmd_ack.ack == CMD_ACK_BUFFER_OVERFLOW) {
				usleep_range(10000, 11000);
			}
			usleep_range(1000, 1100);
			break;
		}
	}
	TPD_INFO("GT:%s, failed get valid cmd ack\n", __func__);
	return -1;
}

/* this is for send cmd with one byte cmd_data parameters */
static int goodix_send_cmd_simple(struct chip_data_brl *chip_info,
				  u8 cmd_type, u8 cmd_data)
{
	struct goodix_ts_cmd cmd;

	switch (cmd_type) {
	case GTP_CMD_NORMAL:
	case GTM_CMD_EDGE_LIMIT_VERTICAL:
		cmd.len = GTP_CMD_SPECIAL_LEN;
	break;
	default:
		cmd.len = GTP_CMD_NORMAL__LEN;
	break;
	}

	TPD_INFO("GT:%s, cmd:%u, len:%d, data:%u\n", __func__, cmd_type, cmd.len, cmd_data);

	cmd.cmd = cmd_type;
	cmd.data[0] = cmd_data;
	return brl_send_cmd(chip_info, &cmd);
}

/********* Start of function that work for oplus_touchpanel_operations callbacks***************/
static int goodix_clear_irq(void *chip_data)
{
	int ret = -1;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	u8 clean_flag = 0;

	if (!chip_info->rawdiff_mode) {
		ret = goodix_reg_write(chip_info,
				       chip_info->ic_info.misc.touch_data_addr, &clean_flag, 1);
		if (ret < 0) {
			TPD_INFO("GT:I2C write end_cmd  error!\n");
		}
	}

	return ret;
}

/*
static void getSpecialCornerPoint(uint8_t *buf, int n, struct Coordinate *point)
{
    // TODO
}

static int clockWise(uint8_t *buf, int n)
{
    // TODO
    //TPD_INFO("GT:ClockWise count = %d\n", count);
    return 1;
}
*/

static void goodix_esd_check_enable(struct chip_data_brl *chip_info, bool enable)
{
	TPD_INFO("GT:%s %s\n", __func__, enable ? "enable" : "disable");
	/* enable/disable esd check flag */
	chip_info->esd_check_enabled = enable;
}

static int goodix_enter_sleep(struct chip_data_brl *chip_info, bool enable)
{
	u8 sleep_cmd[] = {0x00, 0x00, 0x04, 0x84, 0x88, 0x00};
	u32 cmd_reg = chip_info->ic_info.misc.cmd_addr;

	TPD_INFO("GT:%s, sleep enable = %d\n", __func__, enable);

	if (enable) {
		goodix_spi_write(chip_info, cmd_reg, sleep_cmd, sizeof(sleep_cmd));
	} else {
		goodix_reset(chip_info);
	}

	chip_info->sleep_enable = !!enable;
	return 0;
}

static int goodix_enable_gesture(struct chip_data_brl *chip_info, bool enable)
{
	struct goodix_ts_cmd tmp_cmd;
	int ret = -1;

	TPD_INFO("GT:%s, gesture enable = %d\n", __func__, enable);

	if (enable) {
		if (chip_info->gesture_type == 0)
			chip_info->gesture_type = 0xFFFFFFFF;

		goodix_enter_sleep(chip_info, false);

		TPD_INFO("GT:%s, gesture_type:0x%08X, and disable sleep\n", __func__, chip_info->gesture_type);
		tmp_cmd.len = 8;
		tmp_cmd.cmd = 0xA6;
		tmp_cmd.data[0] = chip_info->gesture_type & 0xFF;
		tmp_cmd.data[1] = (chip_info->gesture_type >> 8) & 0xFF;
		tmp_cmd.data[2] = (chip_info->gesture_type >> 16) & 0xFF;
		tmp_cmd.data[3] = (chip_info->gesture_type >> 24) & 0xFF;
	} else {
		tmp_cmd.len = 4;
		tmp_cmd.cmd = 0xA7;
	}

	ret = brl_send_cmd(chip_info, &tmp_cmd);

	chip_info->gesture_enable = !!enable;
	return ret;
}

static int goodix_enable_edge_limit(struct chip_data_brl *chip_info, int state)
{
	int ret = -1;

	TPD_INFO("GT:%s, edge limit enable = %d\n", __func__, state);

	if (state == 1 || VERTICAL_SCREEN == chip_info->touch_direction) {
		ret = goodix_send_cmd_simple(chip_info, GTM_CMD_EDGE_LIMIT_VERTICAL, GTP_CMD_NORMAL);
	} else {
		if (LANDSCAPE_SCREEN_90 == chip_info->touch_direction) {
			ret = goodix_send_cmd_simple(chip_info, GTM_CMD_EDGE_LIMIT_LANDSCAPE, GTP_MASK_DISABLE);
		} else if (LANDSCAPE_SCREEN_270 == chip_info->touch_direction) {
			ret = goodix_send_cmd_simple(chip_info, GTM_CMD_EDGE_LIMIT_LANDSCAPE, GTP_MASK_ENABLE);
		}
	}

	return ret;
}

static int goodix_enable_charge_mode(struct chip_data_brl *chip_info, bool enable)
{
	int ret = -1;

	TPD_INFO("GT:%s, charge mode enable = %d\n", __func__, enable);

	if (enable) {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_CHARGER_ON, GTP_MASK_ENABLE);
	} else {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_CHARGER_OFF, GTP_MASK_DISABLE);
	}

	return ret;
}

static int goodix_enable_game_mode(struct chip_data_brl *chip_info, bool enable)
{
	int ret = 0;

	TPD_INFO("GT:%s, game mode enable = %d\n", __func__, enable);
	if (enable) {
		/* disable pen before game mode start when pen enable*/
		msleep(10);
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_GAME_MODE, GTP_MASK_ENABLE);
		TPD_INFO("GT:%s: GTP_CMD_ENTER_GAME_MODE\n", __func__);
	} else {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_GAME_MODE, GTP_MASK_DISABLE);
		TPD_INFO("GT:%s: GTP_CMD_EXIT_GAME_MODE\n", __func__);
		/* enable pen after game mode end when pen enable*/
		msleep(10);
	}

	chip_info->game_enable = !!enable;

	return ret;
}

#pragma  pack(1)
struct goodix_config_head {
	union {
		struct {
			u8 panel_name[8];
			u8 fw_pid[8];
			u8 fw_vid[4];
			u8 project_name[8];
			u8 file_ver[2];
			u32 cfg_id;
			u8 cfg_ver;
			u8 cfg_time[8];
			u8 reserved[15];
			u8 flag;
			u16 cfg_len;
			u8 cfg_num;
			u16 checksum;
		};
		u8 buf[64];
	};
};
#pragma pack()

#define CONFIG_CND_LEN          4
#define CONFIG_CMD_START        0x04
#define CONFIG_CMD_WRITE        0x05
#define CONFIG_CMD_EXIT         0x06
#define CONFIG_CMD_READ_START   0x07
#define CONFIG_CMD_READ_EXIT    0x08

#define CONFIG_CMD_STATUS_PASS  0x80
#define CONFIG_CMD_WAIT_RETRY   20

static int wait_cmd_status(struct chip_data_brl *chip_info, u8 target_status, int retry)
{
	struct goodix_ts_cmd cmd_ack;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;
	int i, ret;

	for (i = 0; i < retry; i++) {
		ret = goodix_reg_read(chip_info,
				      misc->cmd_addr, cmd_ack.buf, sizeof(cmd_ack));
		if (!ret && cmd_ack.state == target_status) {
			TPD_DEBUG("GT:status check pass\n");
			return 0;
		}

		TPD_INFO("GT:%s: cmd status not ready, retry %d, ack 0x%x, status 0x%x, ret %d\n", __func__,
			 i, cmd_ack.ack, cmd_ack.state, ret);
		TPD_INFO("GT:cmd buf %*ph\n", (int)sizeof(cmd_ack), cmd_ack.buf);
		usleep_range(15000, 15100);
	}
	return -1;
}

static int send_cfg_cmd(struct chip_data_brl *chip_info,
			struct goodix_ts_cmd *cfg_cmd)
{
	int ret;

	ret = brl_send_cmd(chip_info, cfg_cmd);
	if (ret) {
		TPD_INFO("GT:%s: failed write cfg prepare cmd %d\n", __func__, ret);
		return ret;
	}
	ret = wait_cmd_status(chip_info, CONFIG_CMD_STATUS_PASS,
			      CONFIG_CMD_WAIT_RETRY);
	if (ret) {
		TPD_INFO("GT:%s: failed wait for fw ready for config, %d\n",
			 __func__, ret);
		return ret;
	}
	return 0;
}

static s32 goodix_send_config(struct chip_data_brl *chip_info, u8 *cfg, int len)
{
	int ret;
	u8 *tmp_buf;
	struct goodix_ts_cmd cfg_cmd;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;

	if (len > misc->fw_buffer_max_len) {
		TPD_INFO("GT:%s: config len exceed limit %d > %d\n", __func__,
			 len, misc->fw_buffer_max_len);
		return -1;
	}

	tmp_buf = kzalloc(len, GFP_KERNEL);
	if (!tmp_buf) {
		TPD_INFO("GT:%s: failed alloc malloc\n", __func__);
		return -ENOMEM;
	}

	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_START;
	ret = send_cfg_cmd(chip_info, &cfg_cmd);
	if (ret) {
		TPD_INFO("GT:%s: failed write cfg prepare cmd %d\n",
			 __func__, ret);
		goto exit;
	}

	TPD_DEBUG("GT:try send config to 0x%x, len %d\n",
		  misc->fw_buffer_addr, len);
	ret = goodix_reg_write(chip_info,
			       misc->fw_buffer_addr, cfg, len);
	if (ret) {
		TPD_INFO("GT:%s: failed write config data, %d\n", __func__, ret);
		goto exit;
	}
	ret = goodix_reg_read(chip_info,
			      misc->fw_buffer_addr, tmp_buf, len);
	if (ret) {
		TPD_INFO("GT:%s: failed read back config data\n", __func__);
		goto exit;
	}

	if (memcmp(cfg, tmp_buf, len)) {
		TPD_INFO("GT:%s: config data read back compare file\n", __func__);
		ret = -1;
		goto exit;
	}
	/* notify fw for recive config */
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_WRITE;
	ret = send_cfg_cmd(chip_info, &cfg_cmd);
	if (ret) {
		TPD_INFO("GT:%s: failed send config data ready cmd %d\n", __func__, ret);
	}

exit:
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_EXIT;
	if (send_cfg_cmd(chip_info, &cfg_cmd)) {
		TPD_INFO("GT:%s: failed send config write end command\n", __func__);
		ret = -1;
	}

	if (!ret) {
		TPD_INFO("GT:success send config\n");
		msleep(100);
	}

	kfree(tmp_buf);
	return ret;
}

static int goodix_reset(void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("GT:%s IN\n", __func__);

	if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
		gpio_direction_output(chip_info->hw_res->reset_gpio, false);
		msleep(15);
		gpio_direction_output(chip_info->hw_res->reset_gpio, true);
		msleep(50);
		chip_info->halt_status = false; /*reset this flag when ic reset*/
	} else {
		TPD_INFO("GT:reset gpio is invalid\n");
	}

	msleep(100);

	chip_info->pen_state = PEN_UP;


	return 0;
}

/*********** End of function that work for oplus_touchpanel_operations callbacks***************/


/********* Start of implementation of oplus_touchpanel_operations callbacks********************/
static int goodix_ftm_process(void *chip_data)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("GT:%s is called!\n", __func__);
	ret = tp_powercontrol_avdd(chip_info->hw_res, false);
	if (ret) {
		TPD_INFO("GT:%s, avdd power control faild,ret = %d\n", __func__, ret);
		return -1;
	}
	ret = tp_powercontrol_vddi(chip_info->hw_res, false);
	if (ret) {
		TPD_INFO("GT:%s, vddi power control faild,ret = %d\n", __func__, ret);
		return -1;
	}
	if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
		gpio_direction_output(chip_info->hw_res->reset_gpio, false);
	}

	return 0;
}
/*
#define TS_DEFAULT_FIRMWARE         "GT9966.bin"
#define TS_DEFAULT_INSPECT_LIMIT    "gt9966_test_limit"
static void  goodix_util_get_vendor(struct hw_resource * hw_res,
		struct panel_info *panel_data)
{
	char manuf_version[MAX_DEVICE_VERSION_LENGTH] = "0202 ";
	char manuf_manufacture[MAX_DEVICE_MANU_LENGTH] = "GD_";
	char gt_fw_name[16] = TS_DEFAULT_FIRMWARE;
	char gt_test_limit_name[MAX_DEVICE_MANU_LENGTH] = TS_DEFAULT_INSPECT_LIMIT;

	memcpy (panel_data->manufacture_info.version,
		&manuf_version[0], MAX_DEVICE_VERSION_LENGTH);
	memcpy (panel_data->manufacture_info.manufacture,
		&manuf_manufacture[0], MAX_DEVICE_MANU_LENGTH);
	memcpy (panel_data->fw_name , &gt_fw_name [0],16);
	memcpy (panel_data->test_limit_name,
		&gt_test_limit_name[0], MAX_DEVICE_MANU_LENGTH);
}
*/
static int goodix_get_vendor(void *chip_data, struct panel_info *panel_data)
{
	int len = 0;
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
	char manu_temp[MAX_DEVICE_MANU_LENGTH] = "HD_";
#endif
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	len = strlen(panel_data->fw_name);
	chip_info->tp_type = panel_data->tp_type;
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
	chip_info->p_tp_fw = panel_data->manufacture_info.version;
	strlcat(manu_temp, panel_data->manufacture_info.manufacture,
		MAX_DEVICE_MANU_LENGTH);
	strncpy(panel_data->manufacture_info.manufacture, manu_temp,
		MAX_DEVICE_MANU_LENGTH);
#endif
	TPD_INFO("chip_info->tp_type = %d, panel_data->fw_name = %s\n",
		 chip_info->tp_type, panel_data->fw_name);

	return 0;
}


#define GOODIX_READ_VERSION_RETRY   5
#define FW_VERSION_INFO_ADDR        0x10014
#define GOODIX_NORMAL_PID           "9966"
/*
 * return: 0 for no error.
 * 	   -1 when encounter a bus error
 * 	   -EFAULT version checksum error
*/
static int brl_read_version(struct chip_data_brl *chip_info,
			    struct goodix_fw_version *version)
{
	int ret, i;
	u8 rom_pid[10] = {0};
	u8 patch_pid[10] = {0};
	u8 buf[sizeof(struct goodix_fw_version)] = {0};

	for (i = 0; i < GOODIX_READ_VERSION_RETRY; i++) {
		ret = goodix_reg_read(chip_info,
				      FW_VERSION_INFO_ADDR, buf, sizeof(buf));
		if (ret) {
			TPD_INFO("GT:read fw version: %d, retry %d\n", ret, i);
			ret = -1;
			usleep_range(5000, 5100);
			continue;
		}

		if (!checksum_cmp(buf, sizeof(buf), CHECKSUM_MODE_U8_LE)) {
			break;
		}

		TPD_INFO("GT:invalid fw version: checksum error!\n");
		TPD_INFO("GT:fw version:%*ph\n", (int)sizeof(buf), buf);
		ret = -EFAULT;
		usleep_range(10000, 10100);
	}
	if (ret) {
		TPD_INFO("GT:%s: failed get valied fw version, but continue\n", __func__);
		return 0;
	}
	memcpy(version, buf, sizeof(*version));
	memcpy(rom_pid, version->rom_pid, sizeof(version->rom_pid));
	memcpy(patch_pid, version->patch_pid, sizeof(version->patch_pid));
	TPD_INFO("GT:rom_pid:%s\n", rom_pid);
	TPD_INFO("GT:rom_vid:%*ph\n", (int)sizeof(version->rom_vid), version->rom_vid);
	TPD_INFO("GT:pid:%s\n", patch_pid);
	TPD_INFO("GT:vid:%*ph\n", (int)sizeof(version->patch_vid), version->patch_vid);
	TPD_INFO("GT:sensor_id:%d\n", version->sensor_id);

	return 0;
}

#define LE16_TO_CPU(x)  (x = le16_to_cpu(x))
#define LE32_TO_CPU(x)  (x = le32_to_cpu(x))
static int convert_ic_info(struct goodix_ic_info *info, const u8 *data)
{
	int i;
	struct goodix_ic_info_version *version = &info->version;
	struct goodix_ic_info_feature *feature = &info->feature;
	struct goodix_ic_info_param *parm = &info->parm;
	struct goodix_ic_info_misc *misc = &info->misc;

	info->length = le16_to_cpup((__le16 *)data);

	data += 2;
	memcpy(version, data, sizeof(*version));
	version->config_id = le32_to_cpu(version->config_id);

	data += sizeof(struct goodix_ic_info_version);
	memcpy(feature, data, sizeof(*feature));
	feature->freqhop_feature = le16_to_cpu(feature->freqhop_feature);
	feature->calibration_feature = le16_to_cpu(feature->calibration_feature);
	feature->gesture_feature = le16_to_cpu(feature->gesture_feature);
	feature->side_touch_feature = le16_to_cpu(feature->side_touch_feature);
	feature->stylus_feature = le16_to_cpu(feature->stylus_feature);

	data += sizeof(struct goodix_ic_info_feature);
	parm->drv_num = *(data++);
	parm->sen_num = *(data++);
	parm->button_num = *(data++);
	parm->force_num = *(data++);
	parm->active_scan_rate_num = *(data++);
	if (parm->active_scan_rate_num > MAX_SCAN_RATE_NUM) {
		TPD_INFO("GT:%s: invalid scan rate num %d > %d\n", __func__,
			 parm->active_scan_rate_num, MAX_SCAN_RATE_NUM);
		return -1;
	}
	for (i = 0; i < parm->active_scan_rate_num; i++) {
		parm->active_scan_rate[i] = le16_to_cpup((__le16 *)(data + i * 2));
	}

	data += parm->active_scan_rate_num * 2;
	parm->mutual_freq_num = *(data++);
	if (parm->mutual_freq_num > MAX_SCAN_FREQ_NUM) {
		TPD_INFO("GT:%s: invalid mntual freq num %d > %d\n", __func__,
			 parm->mutual_freq_num, MAX_SCAN_FREQ_NUM);
		return -1;
	}
	for (i = 0; i < parm->mutual_freq_num; i++) {
		parm->mutual_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));
	}

	data += parm->mutual_freq_num * 2;
	parm->self_tx_freq_num = *(data++);
	if (parm->self_tx_freq_num > MAX_SCAN_FREQ_NUM) {
		TPD_INFO("GT:%s: invalid tx freq num %d > %d\n", __func__,
			 parm->self_tx_freq_num, MAX_SCAN_FREQ_NUM);
		return -1;
	}
	for (i = 0; i < parm->self_tx_freq_num; i++) {
		parm->self_tx_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));
	}

	data += parm->self_tx_freq_num * 2;
	parm->self_rx_freq_num = *(data++);
	if (parm->self_rx_freq_num > MAX_SCAN_FREQ_NUM) {
		TPD_INFO("GT:%s: invalid rx freq num %d > %d\n", __func__,
			 parm->self_rx_freq_num, MAX_SCAN_FREQ_NUM);
		return -1;
	}

	for (i = 0; i < parm->self_rx_freq_num; i++) {
		parm->self_rx_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));
	}

	data += parm->self_rx_freq_num * 2;
	parm->stylus_freq_num = *(data++);
	if (parm->stylus_freq_num > MAX_FREQ_NUM_STYLUS) {
		TPD_INFO("GT:%s: invalid stylus freq num %d > %d\n", __func__,
			 parm->stylus_freq_num, MAX_FREQ_NUM_STYLUS);
		return -1;
	}
	for (i = 0; i < parm->stylus_freq_num; i++) {
		parm->stylus_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));
	}

	data += parm->stylus_freq_num * 2;
	memcpy(misc, data, sizeof(*misc));
	misc->cmd_addr = le32_to_cpu(misc->cmd_addr);
	misc->cmd_max_len = le16_to_cpu(misc->cmd_max_len);
	misc->cmd_reply_addr = le32_to_cpu(misc->cmd_reply_addr);
	misc->cmd_reply_len = le16_to_cpu(misc->cmd_reply_len);
	misc->fw_state_addr = le32_to_cpu(misc->fw_state_addr);
	misc->fw_state_len = le16_to_cpu(misc->fw_state_len);
	misc->fw_buffer_addr = le32_to_cpu(misc->fw_buffer_addr);
	misc->fw_buffer_max_len = le16_to_cpu(misc->fw_buffer_max_len);
	misc->frame_data_addr = le32_to_cpu(misc->frame_data_addr);
	misc->frame_data_head_len = le16_to_cpu(misc->frame_data_head_len);

	misc->fw_attr_len = le16_to_cpu(misc->fw_attr_len);
	misc->fw_log_len = le16_to_cpu(misc->fw_log_len);
	misc->stylus_struct_len = le16_to_cpu(misc->stylus_struct_len);
	misc->mutual_struct_len = le16_to_cpu(misc->mutual_struct_len);
	misc->self_struct_len = le16_to_cpu(misc->self_struct_len);
	misc->noise_struct_len = le16_to_cpu(misc->noise_struct_len);
	misc->touch_data_addr = le32_to_cpu(misc->touch_data_addr);
	misc->touch_data_head_len = le16_to_cpu(misc->touch_data_head_len);
	misc->point_struct_len = le16_to_cpu(misc->point_struct_len);
	LE32_TO_CPU(misc->mutual_rawdata_addr);
	LE32_TO_CPU(misc->mutual_diffdata_addr);
	LE32_TO_CPU(misc->mutual_refdata_addr);
	LE32_TO_CPU(misc->self_rawdata_addr);
	LE32_TO_CPU(misc->self_diffdata_addr);
	LE32_TO_CPU(misc->self_refdata_addr);
	LE32_TO_CPU(misc->iq_rawdata_addr);
	LE32_TO_CPU(misc->iq_refdata_addr);
	LE32_TO_CPU(misc->im_rawdata_addr);
	LE16_TO_CPU(misc->im_readata_len);
	LE32_TO_CPU(misc->noise_rawdata_addr);
	LE16_TO_CPU(misc->noise_rawdata_len);
	LE32_TO_CPU(misc->stylus_rawdata_addr);
	LE16_TO_CPU(misc->stylus_rawdata_len);
	LE32_TO_CPU(misc->noise_data_addr);
	LE32_TO_CPU(misc->esd_addr);

	return 0;
}

static void print_ic_info(struct goodix_ic_info *ic_info)
{
	struct goodix_ic_info_version *version = &ic_info->version;
	struct goodix_ic_info_feature *feature = &ic_info->feature;
	struct goodix_ic_info_param *parm = &ic_info->parm;
	struct goodix_ic_info_misc *misc = &ic_info->misc;

	TPD_INFO("GT:ic_info_length:                %d\n", ic_info->length);
	TPD_INFO("GT:info_customer_id:              0x%01X\n", version->info_customer_id);
	TPD_INFO("GT:info_version_id:               0x%01X\n", version->info_version_id);
	TPD_INFO("GT:ic_die_id:                     0x%01X\n", version->ic_die_id);
	TPD_INFO("GT:ic_version_id:                 0x%01X\n", version->ic_version_id);
	TPD_INFO("GT:config_id:                     0x%4X\n", version->config_id);
	TPD_INFO("GT:config_version:                0x%01X\n", version->config_version);
	TPD_INFO("GT:frame_data_customer_id:        0x%01X\n", version->frame_data_customer_id);
	TPD_INFO("GT:frame_data_version_id:         0x%01X\n", version->frame_data_version_id);
	TPD_INFO("GT:touch_data_customer_id:        0x%01X\n", version->touch_data_customer_id);
	TPD_INFO("GT:touch_data_version_id:         0x%01X\n", version->touch_data_version_id);

	TPD_INFO("GT:freqhop_feature:               0x%04X\n", feature->freqhop_feature);
	TPD_INFO("GT:calibration_feature:           0x%04X\n", feature->calibration_feature);
	TPD_INFO("GT:gesture_feature:               0x%04X\n", feature->gesture_feature);
	TPD_INFO("GT:side_touch_feature:            0x%04X\n", feature->side_touch_feature);
	TPD_INFO("GT:stylus_feature:                0x%04X\n", feature->stylus_feature);

	TPD_INFO("GT:Drv*Sen,Button,Force num:      %d x %d, %d, %d\n",
		 parm->drv_num, parm->sen_num, parm->button_num, parm->force_num);

	TPD_INFO("GT:Cmd:                           0x%04X, %d\n",
		 misc->cmd_addr, misc->cmd_max_len);
	TPD_INFO("GT:Cmd-Reply:                     0x%04X, %d\n",
		 misc->cmd_reply_addr, misc->cmd_reply_len);
	TPD_INFO("GT:FW-State:                      0x%04X, %d\n",
		 misc->fw_state_addr, misc->fw_state_len);
	TPD_INFO("GT:FW-Buffer:                     0x%04X, %d\n",
		 misc->fw_buffer_addr, misc->fw_buffer_max_len);
	TPD_INFO("GT:Touch-Data:                    0x%04X, %d\n",
		 misc->touch_data_addr, misc->touch_data_head_len);
	TPD_INFO("GT:point_struct_len:              %d\n", misc->point_struct_len);
	TPD_INFO("GT:mutual_rawdata_addr:           0x%04X\n",
		 misc->mutual_rawdata_addr);
	TPD_INFO("GT:mutual_diffdata_addr:          0x%04X\n",
		 misc->mutual_diffdata_addr);
	TPD_INFO("GT:self_rawdata_addr:             0x%04X\n",
		 misc->self_rawdata_addr);
	TPD_INFO("GT:self_rawdata_addr:             0x%04X\n",
		 misc->self_rawdata_addr);
	TPD_INFO("GT:stylus_rawdata_addr:           0x%04X, %d\n",
		 misc->stylus_rawdata_addr, misc->stylus_rawdata_len);
	TPD_INFO("GT:esd_addr:                      0x%04X\n", misc->esd_addr);
}

#define GOODIX_GET_IC_INFO_RETRY	3
#define GOODIX_IC_INFO_MAX_LEN		1024
#define GOODIX_IC_INFO_ADDR		    0x10070
static int brl_get_ic_info(struct chip_data_brl *chip_info,
			   struct goodix_ic_info *ic_info)
{
	int ret, i;
	u16 length = 0;
	u8 afe_data[GOODIX_IC_INFO_MAX_LEN] = {0};

	for (i = 0; i < GOODIX_GET_IC_INFO_RETRY; i++) {
		ret = goodix_reg_read(chip_info,
				      GOODIX_IC_INFO_ADDR,
				      (u8 *)&length, sizeof(length));
		if (ret) {
			TPD_INFO("GT:failed get ic info length, %d\n", ret);
			usleep_range(5000, 5100);
			continue;
		}
		length = le16_to_cpu(length);
		if (length >= GOODIX_IC_INFO_MAX_LEN) {
			TPD_INFO("GT:invalid ic info length %d, retry %d\n", length, i);
			continue;
		}

		ret = goodix_reg_read(chip_info,
				      GOODIX_IC_INFO_ADDR, afe_data, length);
		if (ret) {
			TPD_INFO("GT:failed get ic info data, %d\n", ret);
			usleep_range(5000, 5100);
			continue;
		}
		/* judge whether the data is valid */
		/*if (is_risk_data((const uint8_t *)afe_data, length)) {
			TPD_INFO("GT:fw info data invalid\n");
			usleep_range(5000, 5100);
			continue;
		}*/
		if (checksum_cmp((const uint8_t *)afe_data,
				 length, CHECKSUM_MODE_U8_LE)) {
			TPD_INFO("GT:fw info checksum error!\n");
			usleep_range(5000, 5100);
			continue;
		}
		break;
	}
	if (i == GOODIX_GET_IC_INFO_RETRY) {
		TPD_INFO("GT:%s: failed get ic info\n", __func__);
		/* set ic_info length =0 to indicate this is invalid */
		ic_info->length = 0;
		return -1;
	}

	ret = convert_ic_info(ic_info, afe_data);
	if (ret) {
		TPD_INFO("GT:%s: convert ic info encounter error\n", __func__);
		ic_info->length = 0;
		return ret;
	}
	print_ic_info(ic_info);
	/* check some key info */
	if (!ic_info->misc.cmd_addr || !ic_info->misc.fw_buffer_addr ||
	    !ic_info->misc.touch_data_addr) {
		TPD_INFO("GT:%s: cmd_addr fw_buf_addr and touch_data_addr is null\n", __func__);
		ic_info->length = 0;
		return -1;
	}
	TPD_INFO("GT:success get ic info %d\n", ic_info->length);
	return 0;
}

static int goodix_get_chip_info(void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	int ret;

	/* get ic info */
	ret = brl_get_ic_info(chip_info, &chip_info->ic_info);
	if (ret < 0) {
		TPD_INFO("GT:faile to get ic info, but continue to probe common!!\n");
		return 0;
	}

	/* get version info */
	ret = brl_read_version(chip_info, &chip_info->ver_info);
	if (ret < 0) {
		TPD_INFO("GT:failed to get version\n");
	}

	return ret;
}

static int goodix_power_control(void *chip_data, bool enable)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("GT:%s: enable:%d\n", __func__, enable);

	if (true == enable) {
		ret = tp_powercontrol_vddi(chip_info->hw_res, true);
		if (ret) {
			return -1;
		}

		usleep_range(10000, 11000);

		ret = tp_powercontrol_avdd(chip_info->hw_res, true);
		if (ret) {
			return -1;
		}

		usleep_range(10000, 11000);
		goodix_reset(chip_info);
	} else {
		if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
			gpio_direction_output(chip_info->hw_res->reset_gpio, false);
		}

		disable_irq(chip_info->irq);

		usleep_range(10000, 11000);

		ret = tp_powercontrol_vddi(chip_info->hw_res, false);
		if (ret) {
			return -1;
		}

		usleep_range(50000, 51000);

		ret = tp_powercontrol_avdd(chip_info->hw_res, false);
		if (ret) {
			return -1;
		}
	}

	return ret;
}

static fw_check_state goodix_fw_check(void *chip_data,
				      struct resolution_info *resolution_info,
				      struct panel_info *panel_data)
{
	u32 fw_ver_num = 0;
	u8 cfg_ver = 0;
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
	char dev_version[MAX_DEVICE_VERSION_LENGTH] = {0};
#endif
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_fw_version *fw_ver;

	/* TODO need confirm fw state check method*/
	fw_ver = &chip_info->ver_info;
	if (!isdigit(fw_ver->patch_pid[0]) || !isdigit(fw_ver->patch_pid[1]) ||
	    !isdigit(fw_ver->patch_pid[2]) || !isdigit(fw_ver->patch_pid[3])) {
		TPD_INFO("GT:%s: goodix abnormal patch id found: %s\n", __func__,
			 fw_ver->patch_pid);
		return FW_ABNORMAL;
	}
	fw_ver_num = le32_to_cpup((__le32 *)&fw_ver->patch_vid[0]);
	cfg_ver = chip_info->ic_info.version.config_version;

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
	if (panel_data->manufacture_info.version) {
		panel_data->tp_fw = (fw_ver_num >> 24);
		sprintf(dev_version, "%04x", panel_data->tp_fw);
		strlcpy(&(panel_data->manufacture_info.version[5]), dev_version, 5);
	}
#endif

	TPD_INFO("GT:%s: panel_data->tp_fw = 0x%x , fw_ver_num = 0x%x, fw_ver_num >> 24 = 0x%x\n",
		__func__, panel_data->tp_fw, fw_ver_num, fw_ver_num >> 24);
	return FW_NORMAL;
}

/*****************start of GT9886's update function********************/

#define FW_HEADER_SIZE				512
#define FW_SUBSYS_INFO_SIZE			10
#define FW_SUBSYS_INFO_OFFSET       42
#define FW_SUBSYS_MAX_NUM			47

#define ISP_MAX_BUFFERSIZE			(1024 * 4)

#define NO_NEED_UPDATE              99

#define FW_PID_LEN		    		8
#define FW_VID_LEN       			4
#define FLASH_CMD_LEN 				11

#define FW_FILE_CHECKSUM_OFFSET 	8
#define CONFIG_DATA_TYPE 			4

#define ISP_RAM_ADDR				0x57000
#define HW_REG_CPU_RUN_FROM			0x10000
#define FLASH_CMD_REG				0x13400
#define HW_REG_ISP_BUFFER			0x13410
#define CONFIG_DATA_ADDR			0x40000

#define HOLD_CPU_REG_W 				0x0002
#define HOLD_CPU_REG_R 				0x2000
#define MISCTL_REG				    0xD80B
#define WATCH_DOG_REG				0xD054
#define ENABLE_MISCTL				0x04
#define DISABLE_WATCH_DOG		    0x00

#define FLASH_CMD_TYPE_READ  			    0xAA
#define FLASH_CMD_TYPE_WRITE 			    0xBB
#define FLASH_CMD_ACK_CHK_PASS	    		0xEE
#define FLASH_CMD_ACK_CHK_ERROR     		0x33
#define FLASH_CMD_ACK_IDLE      		    0x11
#define FLASH_CMD_W_STATUS_CHK_PASS 		0x22
#define FLASH_CMD_W_STATUS_CHK_FAIL 		0x33
#define FLASH_CMD_W_STATUS_ADDR_ERR 		0x44
#define FLASH_CMD_W_STATUS_WRITE_ERR 		0x55
#define FLASH_CMD_W_STATUS_WRITE_OK 		0xEE


/**
 * fw_subsys_info - subsytem firmware infomation
 * @type: sybsystem type
 * @size: firmware size
 * @flash_addr: flash address
 * @data: firmware data
 */
struct fw_subsys_info {
	u8 type;
	u32 size;
	u32 flash_addr;
	const u8 *data;
};

/**
 *  firmware_summary
 * @size: fw total length
 * @checksum: checksum of fw
 * @hw_pid: mask pid string
 * @hw_pid: mask vid code
 * @fw_pid: fw pid string
 * @fw_vid: fw vid code
 * @subsys_num: number of fw subsystem
 * @chip_type: chip type
 * @protocol_ver: firmware packing
 *   protocol version
 * @bus_type: 0 represent I2C, 1 for SPI
 * @subsys: sybsystem info
 */
#pragma pack(1)
struct  firmware_summary {
	u32 size;
	u32 checksum;
	u8 hw_pid[6];
	u8 hw_vid[3];
	u8 fw_pid[FW_PID_LEN];
	u8 fw_vid[FW_VID_LEN];
	u8 subsys_num;
	u8 chip_type;
	u8 protocol_ver;
	u8 bus_type;
	u8 flash_protect;
	u8 reserved[8];
	struct fw_subsys_info subsys[FW_SUBSYS_MAX_NUM];
};
#pragma pack()

/**
 * firmware_data - firmware data structure
 * @fw_summary: firmware infomation
 * @firmware: firmware data structure
 */
struct firmware_data {
	struct  firmware_summary fw_summary;
	const struct firmware *firmware;
};

struct config_data {
	u8 *data;
	int size;
};

#pragma pack(1)
struct goodix_flash_cmd {
	union {
		struct {
			u8 status;
			u8 ack;
			u8 len;
			u8 cmd;
			u8 fw_type;
			u16 fw_len;
			u32 fw_addr;
			/*u16 checksum;*/
		};
		u8 buf[16];
	};
};
#pragma pack()

struct fw_update_ctrl {
	int mode;
	struct goodix_ts_config *ic_config;
	struct chip_data_brl *chip_info;
	struct firmware_data fw_data;
};

/**
 * goodix_parse_firmware - parse firmware header infomation
 *	and subsystem infomation from firmware data buffer
 *
 * @fw_data: firmware struct, contains firmware header info
 *	and firmware data.
 * return: 0 - OK, < 0 - error
 */
/* sizeof(length) + sizeof(checksum) */

static int goodix_parse_firmware(struct firmware_data *fw_data)
{
	const struct firmware *firmware;
	struct  firmware_summary *fw_summary;
	unsigned int i, fw_offset, info_offset;
	u32 checksum;
	int r = 0;

	fw_summary = &fw_data->fw_summary;

	/* copy firmware head info */
	firmware = fw_data->firmware;
	if (firmware->size < FW_SUBSYS_INFO_OFFSET) {
		TPD_INFO("GT:%s: Invalid firmware size:%zu\n",
			 __func__, firmware->size);
		r = -1;
		goto err_size;
	}
	memcpy(fw_summary, firmware->data, sizeof(*fw_summary));

	/* check firmware size */
	fw_summary->size = le32_to_cpu(fw_summary->size);
	if (firmware->size != fw_summary->size + FW_FILE_CHECKSUM_OFFSET) {
		TPD_INFO("GT:%s: Bad firmware, size not match, %zu != %d\n",
			 __func__, firmware->size, fw_summary->size + 6);
		r = -1;
		goto err_size;
	}

	for (i = FW_FILE_CHECKSUM_OFFSET, checksum = 0;
	     i < firmware->size; i+=2) {
		checksum += firmware->data[i] + (firmware->data[i+1] << 8);
	}

	/* byte order change, and check */
	fw_summary->checksum = le32_to_cpu(fw_summary->checksum);
	if (checksum != fw_summary->checksum) {
		TPD_INFO("GT:%s: Bad firmware, cheksum error\n", __func__);
		r = -1;
		goto err_size;
	}

	if (fw_summary->subsys_num > FW_SUBSYS_MAX_NUM) {
		TPD_INFO("GT:%s: Bad firmware, invalid subsys num: %d\n",
			 __func__, fw_summary->subsys_num);
		r = -1;
		goto err_size;
	}

	/* parse subsystem info */
	fw_offset = FW_HEADER_SIZE;
	for (i = 0; i < fw_summary->subsys_num; i++) {
		info_offset = FW_SUBSYS_INFO_OFFSET +
			      i * FW_SUBSYS_INFO_SIZE;

		fw_summary->subsys[i].type = firmware->data[info_offset];
		fw_summary->subsys[i].size =
			le32_to_cpup((__le32 *)&firmware->data[info_offset + 1]);

		fw_summary->subsys[i].flash_addr =
			le32_to_cpup((__le32 *)&firmware->data[info_offset + 5]);
		if (fw_offset > firmware->size) {
			TPD_INFO("GT:%s: Sybsys offset exceed Firmware size\n",
				 __func__);
			goto err_size;
		}

		fw_summary->subsys[i].data = firmware->data + fw_offset;
		fw_offset += fw_summary->subsys[i].size;
	}

	TPD_INFO("GT:Firmware package protocol: V%u\n", fw_summary->protocol_ver);
	TPD_INFO("GT:Fimware PID:GT%s\n", fw_summary->fw_pid);
	TPD_INFO("GT:Fimware VID:%*ph\n", 4, fw_summary->fw_vid);
	TPD_INFO("GT:Firmware chip type:%02X\n", fw_summary->chip_type);
	TPD_INFO("GT:Firmware bus type:%d\n", fw_summary->bus_type);
	TPD_INFO("GT:Firmware size:%u\n", fw_summary->size);
	TPD_INFO("GT:Firmware subsystem num:%u\n", fw_summary->subsys_num);

	for (i = 0; i < fw_summary->subsys_num; i++) {
		TPD_DEBUG("GT:------------------------------------------\n");
		TPD_DEBUG("GT:Index:%d\n", i);
		TPD_DEBUG("GT:Subsystem type:%02X\n", fw_summary->subsys[i].type);
		TPD_DEBUG("GT:Subsystem size:%u\n", fw_summary->subsys[i].size);
		TPD_DEBUG("GT:Subsystem flash_addr:%08X\n",
			  fw_summary->subsys[i].flash_addr);
		TPD_DEBUG("GT:Subsystem Ptr:%p\n", fw_summary->subsys[i].data);
	}

err_size:
	return r;
}

static int goodix_fw_update_reset(struct fw_update_ctrl *fwu_ctrl, int delay_ms)
{
	struct chip_data_brl *chip_info = fwu_ctrl->chip_info;

	if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
		gpio_direction_output(chip_info->hw_res->reset_gpio, 0);
		udelay(2000);
		gpio_direction_output(chip_info->hw_res->reset_gpio, 1);
		if (delay_ms < 20) {
			usleep_range(delay_ms * 1000, delay_ms * 1000 + 100);
		} else {
			msleep(delay_ms);
		}
	}
	return 0;
}

/* get config id form config file */
#define CONFIG_ID_OFFSET	    30
u32 goodix_get_file_config_id(u8 *ic_config)
{
	if (!ic_config) {
		return 0;
	}
	return le32_to_cpup((__le32 *)&ic_config[CONFIG_ID_OFFSET]);
}

/**
 * goodix_fw_version_compare - compare the active version with
 * firmware file version.
 * @fwu_ctrl: firmware infomation to be compared
 * return: 0 equal, < 0 unequal
 */
static int goodix_fw_version_compare(struct fw_update_ctrl *fwu_ctrl)
{
	int ret;
	struct goodix_fw_version fw_version;
	struct firmware_summary *fw_summary = &fwu_ctrl->fw_data.fw_summary;
	u32 file_cfg_id;
	u32 ic_cfg_id;

	/* compare fw_version */
	fw_version = fwu_ctrl->chip_info->ver_info;
	if (memcmp(fw_version.patch_pid, fw_summary->fw_pid, FW_PID_LEN)) {
		TPD_INFO("GT:%s: Product ID mismatch:%s:%s\n", __func__,
			 fw_version.patch_pid, fw_summary->fw_pid);
		return -1;
	}

	ret = memcmp(fw_version.patch_vid, fw_summary->fw_vid, FW_VID_LEN);
	if (ret) {
		TPD_INFO("GT:active firmware version:%*ph\n", FW_VID_LEN,
			 fw_version.patch_vid);
		TPD_INFO("GT:firmware file version: %*ph\n", FW_VID_LEN,
			 fw_summary->fw_vid);
		return -1;
	}
	TPD_INFO("GT:Firmware version equal\n");

	/* compare config id */
	if (fwu_ctrl->ic_config && fwu_ctrl->ic_config->length > 0) {
		file_cfg_id = goodix_get_file_config_id(fwu_ctrl->ic_config->data);
		ic_cfg_id = fwu_ctrl->chip_info->ic_info.version.config_id;
		if (ic_cfg_id != file_cfg_id) {
			TPD_INFO("GT:ic_cfg_id:0x%x != file_cfg_id:0x%x\n",
				 ic_cfg_id, file_cfg_id);
			return -1;
		}
		TPD_INFO("GT:Config_id equal\n");
	}

	return 0;
}

/**
 * goodix_load_isp - load ISP program to deivce ram
 * @dev: pointer to touch device
 * @fw_data: firmware data
 * return 0 ok, <0 error
 */
static int goodix_load_isp(struct fw_update_ctrl *fwu_ctrl)
{
	struct firmware_data *fw_data = &fwu_ctrl->fw_data;
	struct goodix_fw_version isp_fw_version = {{0}};
	struct fw_subsys_info *fw_isp;
	u8 reg_val[8] = {0x00};
	int r;

	fw_isp = &fw_data->fw_summary.subsys[0];

	TPD_INFO("GT:Loading ISP start\n");
	r = goodix_reg_write_confirm(fwu_ctrl->chip_info, ISP_RAM_ADDR,
				     (u8 *)fw_isp->data, fw_isp->size);
	if (r < 0) {
		TPD_INFO("GT:%s: Loading ISP error\n", __func__);
		return r;
	}

	TPD_INFO("GT:Success send ISP data\n");

	/* SET BOOT OPTION TO 0X55 */
	memset(reg_val, 0x55, 8);
	r = goodix_reg_write_confirm(fwu_ctrl->chip_info,
				     HW_REG_CPU_RUN_FROM, reg_val, 8);
	if (r < 0) {
		TPD_INFO("GT:%s: Failed set REG_CPU_RUN_FROM flag\n", __func__);
		return r;
	}
	TPD_INFO("GT:Success write [8]0x55 to 0x%x\n", HW_REG_CPU_RUN_FROM);

	if (goodix_fw_update_reset(fwu_ctrl, 100)) {
		TPD_INFO("GT:%s: reset abnormal\n", __func__);
	}
	/*check isp state */
	if (brl_read_version(fwu_ctrl->chip_info, &isp_fw_version)) {
		TPD_INFO("GT:%s: failed read isp version\n", __func__);
		return -2;
	}
	if (memcmp(&isp_fw_version.patch_pid[3], "ISP", 3)) {
		TPD_INFO("GT:%s: patch id error %c%c%c != %s\n", __func__,
			 isp_fw_version.patch_pid[3], isp_fw_version.patch_pid[4],
			 isp_fw_version.patch_pid[5], "ISP");
		return -3;
	}
	TPD_INFO("GT:ISP running successfully\n");
	return 0;
}

/**
 * goodix_update_prepare - update prepare, loading ISP program
 *  and make sure the ISP is running.
 * @fwu_ctrl: pointer to fimrware control structure
 * return: 0 ok, <0 error
 */
static int goodix_update_prepare(struct fw_update_ctrl *fwu_ctrl)
{
	u8 reg_val[4] = {0};
	u8 temp_buf[64] = {0};
	int retry = 20;
	int r;

	/*reset IC*/
	TPD_INFO("GT:firmware update, reset\n");
	if (goodix_fw_update_reset(fwu_ctrl, 5)) {
		TPD_INFO("GT:%s: reset abnormal\n", __func__);
	}

	retry = 100;
	/* Hold cpu*/
	do {
		reg_val[0] = 0x01;
		reg_val[1] = 0x00;
		r = goodix_reg_write(fwu_ctrl->chip_info,
				     HOLD_CPU_REG_W, reg_val, 2);
		r |= goodix_reg_read(fwu_ctrl->chip_info,
				     HOLD_CPU_REG_R, &temp_buf[0], 4);
		r |= goodix_reg_read(fwu_ctrl->chip_info,
				     HOLD_CPU_REG_R, &temp_buf[4], 4);
		r |= goodix_reg_read(fwu_ctrl->chip_info,
				     HOLD_CPU_REG_R, &temp_buf[8], 4);
		if (!r && !memcmp(&temp_buf[0], &temp_buf[4], 4) &&
		    !memcmp(&temp_buf[4], &temp_buf[8], 4) &&
		    !memcmp(&temp_buf[0], &temp_buf[8], 4)) {
			break;
		}
		usleep_range(1000, 1100);
		TPD_INFO("GT:retry hold cpu %d\n", retry);
		TPD_DEBUG("GT:data:%*ph\n", 12, temp_buf);
	} while (--retry);
	if (!retry) {
		TPD_INFO("GT:%s: Failed to hold CPU, return =%d\n", __func__, r);
		return -1;
	}
	TPD_INFO("GT:Success hold CPU\n");

	/* enable misctl clock */
	r = goodix_reg_read(fwu_ctrl->chip_info, MISCTL_REG, reg_val, 1);
	reg_val[0] |= ENABLE_MISCTL;
	r = goodix_reg_write(fwu_ctrl->chip_info, MISCTL_REG, reg_val, 1);
	TPD_INFO("GT:enbale misctl clock\n");

	/* disable watch dog */
	reg_val[0] = DISABLE_WATCH_DOG;
	r = goodix_reg_write(fwu_ctrl->chip_info, WATCH_DOG_REG, reg_val, 1);
	TPD_INFO("GT:disable watch dog\n");

	/* load ISP code and run form isp */
	r = goodix_load_isp(fwu_ctrl);
	if (r < 0) {
		TPD_INFO("GT:%s: Failed lode and run isp\n", __func__);
	}

	return r;
}

/* goodix_send_flash_cmd: send command to read or write flash data
 * @flash_cmd: command need to send.
 * */
static int goodix_send_flash_cmd(struct fw_update_ctrl *fwu_ctrl,
				 struct goodix_flash_cmd *flash_cmd)
{
	int i, ret, retry;
	struct goodix_flash_cmd tmp_cmd;

	TPD_DEBUG("GT:try send flash cmd:%*ph\n", (int)sizeof(flash_cmd->buf),
		  flash_cmd->buf);
	memset(tmp_cmd.buf, 0, sizeof(tmp_cmd));
	ret = goodix_reg_write(fwu_ctrl->chip_info, FLASH_CMD_REG,
			       flash_cmd->buf, sizeof(flash_cmd->buf));
	if (ret) {
		TPD_INFO("GT:%s: failed send flash cmd %d\n", __func__, ret);
		return ret;
	}

	retry = 5;
	for (i = 0; i < retry; i++) {
		ret = goodix_reg_read(fwu_ctrl->chip_info, FLASH_CMD_REG,
				      tmp_cmd.buf, sizeof(tmp_cmd.buf));
		if (!ret && tmp_cmd.ack == FLASH_CMD_ACK_CHK_PASS) {
			break;
		}
		usleep_range(5000, 5100);
		TPD_INFO("GT:flash cmd ack error retry %d, ack 0x%x, ret %d\n",
			 i, tmp_cmd.ack, ret);
	}
	if (tmp_cmd.ack != FLASH_CMD_ACK_CHK_PASS) {
		TPD_INFO("GT:%s: flash cmd ack error, ack 0x%x, ret %d\n",
			 __func__, tmp_cmd.ack, ret);
		TPD_INFO("GT:%s: data:%*ph\n", __func__,
			 (int)sizeof(tmp_cmd.buf), tmp_cmd.buf);
		return -1;
	}
	TPD_INFO("GT:flash cmd ack check pass\n");

	msleep(80);
	retry = 20;
	for (i = 0; i < retry; i++) {
		ret = goodix_reg_read(fwu_ctrl->chip_info, FLASH_CMD_REG,
				      tmp_cmd.buf, sizeof(tmp_cmd.buf));
		if (!ret && tmp_cmd.ack == FLASH_CMD_ACK_CHK_PASS &&
		    tmp_cmd.status == FLASH_CMD_W_STATUS_WRITE_OK) {
			TPD_INFO("GT:flash status check pass\n");
			return 0;
		}

		TPD_INFO("GT:flash cmd status not ready, retry %d, ack 0x%x, status 0x%x, ret %d\n",
			 i, tmp_cmd.ack, tmp_cmd.status, ret);
		usleep_range(15000, 15100);
	}

	TPD_INFO("GT:%s: flash cmd status error %d, ack 0x%x, status 0x%x, ret %d\n", __func__,
		 i, tmp_cmd.ack, tmp_cmd.status, ret);
	if (ret) {
		TPD_INFO("GT:reason: bus or paltform error\n");
		return -1;
	}

	switch (tmp_cmd.status) {
	case FLASH_CMD_W_STATUS_CHK_PASS:
		TPD_INFO("GT:%s: data check pass, but failed get follow-up results\n", __func__);
		return -EFAULT;
	case FLASH_CMD_W_STATUS_CHK_FAIL:
		TPD_INFO("GT:%s: data check failed, please retry\n", __func__);
		return -EAGAIN;
	case FLASH_CMD_W_STATUS_ADDR_ERR:
		TPD_INFO("GT:%s: flash target addr error, please check\n", __func__);
		return -EFAULT;
	case FLASH_CMD_W_STATUS_WRITE_ERR:
		TPD_INFO("GT:%s: flash data write err, please retry\n", __func__);
		return -EAGAIN;
	default:
		TPD_INFO("GT:%s: unknown status\n", __func__);
		return -EFAULT;
	}
}

static int goodix_flash_package(struct fw_update_ctrl *fwu_ctrl,
				u8 subsys_type, u8 *pkg, u32 flash_addr, u16 pkg_len)
{
	int ret, retry;
	struct goodix_flash_cmd flash_cmd;

	retry = 2;
	do {
		ret = goodix_reg_write(fwu_ctrl->chip_info,
				       HW_REG_ISP_BUFFER, pkg, pkg_len);
		if (ret < 0) {
			TPD_INFO("GT:%s: Failed to write firmware packet\n", __func__);
			return ret;
		}

		flash_cmd.status = 0;
		flash_cmd.ack = 0;
		flash_cmd.len = FLASH_CMD_LEN;
		flash_cmd.cmd = FLASH_CMD_TYPE_WRITE;
		flash_cmd.fw_type = subsys_type;
		flash_cmd.fw_len = cpu_to_le16(pkg_len);
		flash_cmd.fw_addr = cpu_to_le32(flash_addr);

		goodix_append_checksum(&(flash_cmd.buf[2]),
				       9, CHECKSUM_MODE_U8_LE);

		ret = goodix_send_flash_cmd(fwu_ctrl, &flash_cmd);
		if (!ret) {
			TPD_INFO("GT:success write package to 0x%x, len %d\n",
				 flash_addr, pkg_len - 4);
			return 0;
		}
	} while (ret == -EAGAIN && --retry);

	return ret;
}

/**
 * goodix_flash_subsystem - flash subsystem firmware,
 *  Main flow of flashing firmware.
 *	Each firmware subsystem is divided into several
 *	packets, the max size of packet is limited to
 *	@{ISP_MAX_BUFFERSIZE}
 * @dev: pointer to touch device
 * @subsys: subsystem infomation
 * return: 0 ok, < 0 error
 */
static int goodix_flash_subsystem(struct fw_update_ctrl *fwu_ctrl,
				  struct fw_subsys_info *subsys)
{
	u32 data_size, offset;
	u32 total_size;
	/*TODO: confirm flash addr ,<< 8??*/
	u32 subsys_base_addr = subsys->flash_addr;
	u8 *fw_packet = NULL;
	int r = 0;

	/*
	 * if bus(i2c/spi) error occued, then exit, we will do
	 * hardware reset and re-prepare ISP and then retry
	 * flashing
	 */
	total_size = subsys->size;
	fw_packet = kzalloc(ISP_MAX_BUFFERSIZE + 4, GFP_KERNEL);
	if (!fw_packet) {
		TPD_INFO("GT:%s: Failed alloc memory\n", __func__);
		return -ENOMEM;
	}

	offset = 0;
	while (total_size > 0) {
		data_size = total_size > ISP_MAX_BUFFERSIZE ?
			    ISP_MAX_BUFFERSIZE : total_size;
		TPD_INFO("GT:Flash firmware to %08x,size:%u bytes\n",
			 subsys_base_addr + offset, data_size);

		memcpy(fw_packet, &subsys->data[offset], data_size);
		/* set checksum for package data */
		goodix_append_checksum(fw_packet,
				       data_size, CHECKSUM_MODE_U16_LE);

		r = goodix_flash_package(fwu_ctrl, subsys->type, fw_packet,
					 subsys_base_addr + offset, data_size + 4);
		if (r) {
			TPD_INFO("GT:%s: failed flash to %08x,size:%u bytes\n",
				 __func__, subsys_base_addr + offset, data_size);
			break;
		}
		offset += data_size;
		total_size -= data_size;
	} /* end while */

	kfree(fw_packet);
	return r;
}

/**
 * goodix_flash_firmware - flash firmware
 * @dev: pointer to touch device
 * @fw_data: firmware data
 * return: 0 ok, < 0 error
 */
static int goodix_flash_firmware(struct fw_update_ctrl *fw_ctrl)
{
	struct firmware_data *fw_data = &fw_ctrl->fw_data;
	struct  firmware_summary  *fw_summary;
	struct fw_subsys_info *fw_x;
	struct fw_subsys_info subsys_cfg = {0};
	int retry = GOODIX_BUS_RETRY_TIMES;
	int i, r = 0, fw_num;

	/* start from subsystem 1,
	 * subsystem 0 is the ISP program */

	fw_summary = &fw_data->fw_summary;
	fw_num = fw_summary->subsys_num;

	/* flash config data first if we have */
	if (fw_ctrl->ic_config && fw_ctrl->ic_config->length) {
		subsys_cfg.data = fw_ctrl->ic_config->data;
		subsys_cfg.size = fw_ctrl->ic_config->length;
		subsys_cfg.flash_addr = CONFIG_DATA_ADDR;
		subsys_cfg.type = CONFIG_DATA_TYPE;
		r = goodix_flash_subsystem(fw_ctrl, &subsys_cfg);
		if (r) {
			TPD_INFO("GT:%s: failed flash config with ISP, %d\n",
				 __func__, r);
			return r;
		}
		TPD_INFO("GT:success flash config with ISP\n");
	}

	for (i = 1; i < fw_num && retry;) {
		TPD_INFO("GT:--- Start to flash subsystem[%d] ---\n", i);
		fw_x = &fw_summary->subsys[i];
		r = goodix_flash_subsystem(fw_ctrl, fw_x);
		if (r == 0) {
			TPD_INFO("GT:--- End flash subsystem[%d]: OK ---\n", i);
			i++;
		} else if (r == -EAGAIN) {
			retry--;
			TPD_INFO("GT:%s: --- End flash subsystem%d: Fail, errno:%d, retry:%d ---\n", __func__,
				 i, r, GOODIX_BUS_RETRY_TIMES - retry);
		} else if (r < 0) { /* bus error */
			TPD_INFO("GT:%s: --- End flash subsystem%d: Fatal error:%d exit ---\n", __func__,
				 i, r);
			goto exit_flash;
		}
	}

exit_flash:
	return r;
}

/**
 * goodix_update_finish - update finished, FREE resource
 *  and reset flags---
 * @fwu_ctrl: pointer to fw_update_ctrl structrue
 * return: 0 ok, < 0 error
 */
static int goodix_update_finish(struct fw_update_ctrl *fwu_ctrl)
{
	if (goodix_fw_update_reset(fwu_ctrl, 100)) {
		TPD_INFO("GT:%s: reset abnormal\n", __func__);
	}

	/* refresh chip_info */
	if (goodix_get_chip_info(fwu_ctrl->chip_info)) {
		return -1;
	}
	/* compare version */
	if (goodix_fw_version_compare(fwu_ctrl)) {
		return -1;
	}

	return 0;
}

u32 getUint(u8 *buffer, int len)
{
	u32 num = 0;
	int i = 0;
	for (i = 0; i < len; i++) {
		num <<= 8;
		num += buffer[i];
	}
	return num;
}

static int goodix_parse_cfg_data(const struct firmware *cfg_bin,
				 char *cfg_type, u8 *cfg, int *cfg_len, u8 sid)
{
	int i = 0, config_status = 0, one_cfg_count = 0;
	int cfg_pkg_len = 0;

	u8 bin_group_num = 0, bin_cfg_num = 0;
	u16 cfg_checksum = 0, checksum = 0;
	u8 sid_is_exist = GOODIX_NOT_EXIST;
	u16 cfg_offset = 0;
	u8 cfg_sid = 0;

	TPD_DEBUG("GT:%s run,sensor id:%d\n", __func__, sid);

	if (sid == GTP_SENSOR_ID_ERR) {
		sid = GTP_SENSOR_ID_DEFAULT;
		TPD_INFO("GT:%s sensor err set 0,need update to 255\n", __func__);
	}

	cfg_pkg_len = getU32(cfg_bin->data) + BIN_CFG_START_LOCAL;
	if (cfg_pkg_len > cfg_bin->size) {
		TPD_INFO("GT:%s:Bad firmware!,cfg package len:%d,firmware size:%d\n",
			 __func__, cfg_pkg_len, (int)cfg_bin->size);
		goto exit;
	}

	/* check firmware's checksum */
	cfg_checksum = getU16(&cfg_bin->data[4]);

	for (i = BIN_CFG_START_LOCAL; i < (cfg_pkg_len) ; i++) {
		checksum += cfg_bin->data[i];
	}

	if ((checksum) != cfg_checksum) {
		TPD_INFO("GT:%s:Bad firmware!(checksum: 0x%04X, header define: 0x%04X)\n",
			 __func__, checksum, cfg_checksum);
		goto exit;
	}
	/* check head end  */

	bin_group_num = cfg_bin->data[MODULE_NUM];
	bin_cfg_num = cfg_bin->data[CFG_NUM];
	TPD_DEBUG("GT:%s:bin_group_num = %d, bin_cfg_num = %d\n",
		  __func__, bin_group_num, bin_cfg_num);

	if (!strncmp(cfg_type, GOODIX_TEST_CONFIG, strlen(GOODIX_TEST_CONFIG))) {
		config_status = 0;
	} else if (!strncmp(cfg_type, GOODIX_NORMAL_CONFIG, strlen(GOODIX_NORMAL_CONFIG))) {
		config_status = 1;
	} else if (!strncmp(cfg_type, GOODIX_NORMAL_NOISE_CONFIG, strlen(GOODIX_NORMAL_NOISE_CONFIG))) {
		config_status = 2;
	} else if (!strncmp(cfg_type, GOODIX_GLOVE_CONFIG, strlen(GOODIX_GLOVE_CONFIG))) {
		config_status = 3;
	} else if (!strncmp(cfg_type, GOODIX_GLOVE_NOISE_CONFIG, strlen(GOODIX_GLOVE_NOISE_CONFIG))) {
		config_status = 4;
	} else if (!strncmp(cfg_type, GOODIX_HOLSTER_CONFIG, strlen(GOODIX_HOLSTER_CONFIG))) {
		config_status = 5;
	} else if (!strncmp(cfg_type, GOODIX_HOLSTER_NOISE_CONFIG, strlen(GOODIX_HOLSTER_NOISE_CONFIG))) {
		config_status = 6;
	} else if (!strncmp(cfg_type, GOODIX_NOISE_TEST_CONFIG, strlen(GOODIX_NOISE_TEST_CONFIG))) {
		config_status = 7;
	} else {
		TPD_INFO("GT:%s: invalid config text field\n", __func__);
		goto exit;
	}

	cfg_offset = CFG_HEAD_BYTES + bin_group_num * bin_cfg_num * CFG_INFO_BLOCK_BYTES;
	for (i = 0 ; i < bin_group_num * bin_cfg_num; i++) {
		/* find cfg's sid in cfg.bin */
		one_cfg_count = getU16(&cfg_bin->data[CFG_HEAD_BYTES + 2 + i * CFG_INFO_BLOCK_BYTES]);
		cfg_sid = cfg_bin->data[CFG_HEAD_BYTES + i * CFG_INFO_BLOCK_BYTES];
		if (sid == cfg_sid) {
			sid_is_exist = GOODIX_EXIST;
			if (config_status == (cfg_bin->data[CFG_HEAD_BYTES + 1 + i * CFG_INFO_BLOCK_BYTES])) {
				memcpy(cfg, &cfg_bin->data[cfg_offset], one_cfg_count);
				*cfg_len = one_cfg_count;
				TPD_DEBUG("GT:%s:one_cfg_count = %d, cfg_data1 = 0x%02x, cfg_data2 = 0x%02x\n",
					  __func__, one_cfg_count, cfg[0], cfg[1]);
				break;
			}
		}
		cfg_offset += one_cfg_count;
	}

	if (i >= bin_group_num * bin_cfg_num) {
		TPD_INFO("GT:%s:(not find config ,config_status: %d)\n", __func__, config_status);
		goto exit;
	}

	TPD_DEBUG("GT:%s exit\n", __func__);
	return NO_ERR;
exit:
	return RESULT_ERR;
}

static int goodix_get_cfg_data(void *chip_data_info, const struct firmware *cfg_bin,
			       char *config_name, struct goodix_ts_config *config)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data_info;
	u8 *cfg_data = NULL;
	int cfg_len = 0;
	int ret = NO_ERR;

	TPD_DEBUG("GT:%s run\n", __func__);

	cfg_data = kzalloc(GOODIX_CFG_MAX_SIZE, GFP_KERNEL);
	if (cfg_data == NULL) {
		TPD_INFO("GT:Memory allco err\n");
		goto exit;
	}

	config->length = 0;
	/* parse config data */
	ret = goodix_parse_cfg_data(cfg_bin, config_name, cfg_data,
				    &cfg_len, chip_info->ver_info.sensor_id);
	if (ret < 0) {
		TPD_INFO("GT:%s: parse %s data failed\n", __func__, config_name);
		ret = -1;
		goto exit;
	}

	TPD_INFO("GT:%s: %s  version:%d , size:%d\n", __func__,
		 config_name, cfg_data[0], cfg_len);
	memcpy(config->data, cfg_data, cfg_len);
	config->length = cfg_len;

	strncpy(config->name, config_name, MAX_STR_LEN);

exit:
	if (cfg_data) {
		kfree(cfg_data);
		cfg_data = NULL;
	}
	TPD_DEBUG("GT:%s exit\n", __func__);
	return ret;
}


static int goodix_get_cfg_parms(void *chip_data_info,
				const struct firmware *firmware)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data_info;
	int ret = 0;

	TPD_DEBUG("GT:%s run\n", __func__);
	if (firmware == NULL) {
		TPD_INFO("GT:%s: firmware is null\n", __func__);
		ret = -1;
		goto exit;
	}

	if (firmware->data == NULL) {
		TPD_INFO("GT:%s:Bad firmware!(config firmware data is null: )\n", __func__);
		ret = -1;
		goto exit;
	}

	TPD_INFO("GT:%s: cfg_bin_size:%d\n", __func__, (int)firmware->size);
	if (firmware->size > 56) {
		TPD_DEBUG("GT:cfg_bin head info:%*ph\n", 32, firmware->data);
		TPD_DEBUG("GT:cfg_bin head info:%*ph\n", 24, firmware->data + 32);
	}

	/* parse normal config data */
	ret = goodix_get_cfg_data(chip_info, firmware,
				  GOODIX_NORMAL_CONFIG, &chip_info->normal_cfg);
	if (ret < 0) {
		TPD_INFO("GT:%s: Failed to parse normal_config data:%d\n", __func__, ret);
	} else {
		TPD_INFO("GT:%s: parse normal_config data success\n", __func__);
	}

	ret = goodix_get_cfg_data(chip_info, firmware,
				  GOODIX_TEST_CONFIG, &chip_info->test_cfg);
	if (ret < 0) {
		TPD_INFO("GT:%s: Failed to parse test_config data:%d\n", __func__, ret);
	} else {
		TPD_INFO("GT:%s: parse test_config data success\n", __func__);
	}

	/* parse normal noise config data */
	ret = goodix_get_cfg_data(chip_info, firmware,
				  GOODIX_NORMAL_NOISE_CONFIG, &chip_info->normal_noise_cfg);
	if (ret < 0) {
		TPD_INFO("GT:%s: Failed to parse normal_noise_config data\n", __func__);
	} else {
		TPD_INFO("GT:%s: parse normal_noise_config data success\n", __func__);
	}

	/* parse noise test config data */
	ret = goodix_get_cfg_data(chip_info, firmware,
				  GOODIX_NOISE_TEST_CONFIG, &chip_info->noise_test_cfg);
	if (ret < 0) {
		memcpy(&chip_info->noise_test_cfg, &chip_info->normal_cfg,
		       sizeof(chip_info->noise_test_cfg));
		TPD_INFO("GT:%s: Failed to parse noise_test_config data,use normal_config data\n", __func__);
	} else {
		TPD_INFO("GT:%s: parse noise_test_config data success\n", __func__);
	}
exit:
	TPD_DEBUG("GT:%s exit:%d\n", __func__, ret);
	return ret;
}

/*	get fw firmware from firmware
	return value:
	0: operate success
	other: failed*/
static int goodix_get_fw_parms(void *chip_data_info,
			       const struct firmware *firmware, struct firmware *fw_firmware)
{
	/*struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data_info;*/
	int ret = 0;
	int cfg_pkg_len = 0;
	int fw_pkg_len = 0;

	TPD_DEBUG("GT:%s run\n", __func__);
	if (firmware == NULL) {
		TPD_INFO("GT:%s: firmware is null\n", __func__);
		ret = -1;
		goto exit;
	}

	if (firmware->data == NULL) {
		TPD_INFO("GT:%s:Bad firmware!(config firmware data si null)\n", __func__);
		ret = -1;
		goto exit;
	}

	if (fw_firmware == NULL) {
		TPD_INFO("GT:%s:fw_firmware is null\n", __func__);
		ret = -1;
		goto exit;
	}

	TPD_DEBUG("GT:clear fw_firmware\n");
	memset(fw_firmware, 0, sizeof(struct firmware));

	cfg_pkg_len = getU32(firmware->data) + BIN_CFG_START_LOCAL;
	TPD_DEBUG("GT:%s cfg package len:%d\n", __func__, cfg_pkg_len);

	if (firmware->size <= (cfg_pkg_len + 16)) {
		TPD_INFO("GT:%s current firmware does not contain goodix fw\n", __func__);
		TPD_INFO("GT:%s cfg package len:%d,firmware size:%d\n", __func__,
			 cfg_pkg_len, (int)firmware->size);
		ret = -1;
		goto exit;
	}

	if (!((firmware->data[cfg_pkg_len + 0] == 'G') &&
			(firmware->data[cfg_pkg_len + 1] == 'X') &&
			(firmware->data[cfg_pkg_len + 2] == 'F') &&
			(firmware->data[cfg_pkg_len + 3] == 'W'))) {
		TPD_INFO("GT:%s can't find fw package\n", __func__);
		TPD_INFO("GT:Data type:%c %c %c %c,dest type is:GXFW\n", firmware->data[cfg_pkg_len + 0],
			 firmware->data[cfg_pkg_len + 1], firmware->data[cfg_pkg_len + 2],
			 firmware->data[cfg_pkg_len + 3]);
		ret = -1;
		goto exit;
	}

	if (firmware->data[cfg_pkg_len + 4] != 1) {
		TPD_INFO("GT:%s can't support this ver:%d\n", __func__,
			 firmware->data[cfg_pkg_len + 4]);
		ret = -1;
		goto exit;
	}

	fw_pkg_len =  getU32(firmware->data + cfg_pkg_len + 8);

	TPD_DEBUG("GT:%s fw package len:%d\n", __func__, fw_pkg_len);
	if ((fw_pkg_len + 16 + cfg_pkg_len) > firmware->size) {
		TPD_INFO("GT:%s bad firmware,need len:%d,actual firmware size:%d\n",
			 __func__, fw_pkg_len + 16 + cfg_pkg_len, (int)firmware->size);
		ret = -1;
		goto exit;
	}

	fw_firmware->size = fw_pkg_len;
	fw_firmware->data = firmware->data + cfg_pkg_len + 16;

	TPD_DEBUG("GT:success get fw,len:%d\n", fw_pkg_len);
	TPD_DEBUG("GT:fw head info:%*ph\n", 32, fw_firmware->data);
	TPD_DEBUG("GT:fw tail info:%*ph\n", 4, &fw_firmware->data[fw_pkg_len - 4 - 1]);
	ret = 0;

exit:
	TPD_DEBUG("GT:%s exit:%d\n", __func__, ret);
	return ret;
}

static fw_update_state goodix_fw_update(void *chip_data,
					const struct firmware *cfg_fw_firmware, bool force)
{
#define FW_UPDATE_RETRY   2
	int retry0 = FW_UPDATE_RETRY;
	int retry1 = FW_UPDATE_RETRY;
	int r, ret;
	struct chip_data_brl *chip_info;
	struct fw_update_ctrl *fwu_ctrl = NULL;
	struct firmware fw_firmware;

	fwu_ctrl = kzalloc(sizeof(struct fw_update_ctrl), GFP_KERNEL);
	if (!fwu_ctrl) {
		TPD_INFO("GT:Failed to alloc memory for fwu_ctrl\n");
		return -ENOMEM;
	}
	chip_info = (struct chip_data_brl *)chip_data;
	fwu_ctrl->chip_info = chip_info;

	r = goodix_get_cfg_parms(chip_data, cfg_fw_firmware);
	if (r < 0) {
		TPD_INFO("GT:%s Failed get cfg from firmware\n", __func__);
	} else {
		TPD_INFO("GT:%s success get ic cfg from firmware\n", __func__);
	}

	r = goodix_get_fw_parms(chip_data, cfg_fw_firmware, &fw_firmware);
	if (r < 0) {
		TPD_INFO("GT:%s Failed get ic fw from firmware\n", __func__);
		goto err_parse_fw;
	} else {
		TPD_INFO("GT:%s success get ic fw from firmware\n", __func__);
	}

	fwu_ctrl->fw_data.firmware = &fw_firmware;
	fwu_ctrl->ic_config = &fwu_ctrl->chip_info->normal_cfg;
	r = goodix_parse_firmware(&fwu_ctrl->fw_data);
	if (r < 0) {
		goto err_parse_fw;
	}

	/* TODO: set force update flag*/
	if (force == false) {
		r = goodix_fw_version_compare(fwu_ctrl);
		if (!r) {
			TPD_INFO("GT:firmware upgraded\n");
			r = FW_NO_NEED_UPDATE;
			goto err_check_update;
		}
	}

start_update:
	do {
		ret = goodix_update_prepare(fwu_ctrl);
		if (ret) {
			TPD_INFO("GT:%s: failed prepare ISP, retry %d\n", __func__,
				 FW_UPDATE_RETRY - retry0);
		}
	} while (ret && --retry0 > 0);
	if (ret) {
		TPD_INFO("GT:%s: Failed to prepare ISP, exit update:%d\n",
			 __func__, ret);
		goto err_fw_prepare;
	}

	/* progress: 20%~100% */
	ret = goodix_flash_firmware(fwu_ctrl);
	if (ret < 0 && --retry1 > 0) {
		TPD_INFO("GT:%s: Bus error, retry firmware update:%d\n", __func__,
			 FW_UPDATE_RETRY - retry1);
		goto start_update;
	}
	if (ret) {
		TPD_INFO("GT:flash fw data enter error\n");
	} else {
		TPD_INFO("GT:flash fw data success, need check version\n");
	}

err_fw_prepare:
	ret = goodix_update_finish(fwu_ctrl);
	if (!ret) {
		TPD_INFO("GT:Firmware update successfully\n");
		r = FW_UPDATE_SUCCESS;
	} else {
		TPD_INFO("GT:%s: Firmware update failed\n", __func__);
		r = FW_UPDATE_ERROR;
	}
err_check_update:
err_parse_fw:
	if (fwu_ctrl) {
		kfree(fwu_ctrl);
		fwu_ctrl = NULL;
	}

	return r;
}

static u32 goodix_u32_trigger_reason(void *chip_data,
				     int gesture_enable, int is_suspended)
{
	int ret = -1;
	u8 touch_num = 0;
	u8 point_type = 0;
	u32 result_event = 0;
	int pre_read_len;
	u8 event_status;
	struct goodix_ic_info_misc *misc;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	memset(chip_info->touch_data, 0, MAX_GT_IRQ_DATA_LENGTH);
	if (chip_info->kernel_grip_support) {
		memset(chip_info->edge_data, 0, MAX_GT_EDGE_DATA_LENGTH);
	}

	misc = &chip_info->ic_info.misc;
	if (!misc->touch_data_addr) {
		TPD_DEBUG("GT:%s: invalied touch data address\n", __func__);
		return IRQ_IGNORE;
	}

	/* touch head + 2 fingers + checksum */
	pre_read_len = IRQ_EVENT_HEAD_LEN +
		       BYTES_PER_POINT * 2 + COOR_DATA_CHECKSUM_SIZE;

	ret = goodix_reg_read(chip_info, misc->touch_data_addr,
			      chip_info->touch_data, pre_read_len);
	if (ret < 0) {
		TPD_DEBUG("GT:%s: spi transfer error!\n", __func__);
		return IRQ_IGNORE;
	}

	TPD_DEBUG("GT:%s:check->flag:[%*ph]-data:[%*ph]\n", __func__,
		IRQ_EVENT_HEAD_LEN, chip_info->touch_data,
		IRQ_EVENT_HEAD_LEN * IRQ_EVENT_HEAD_LEN, chip_info->touch_data + IRQ_EVENT_HEAD_LEN);

	if (chip_info->touch_data[0] == 0x00) {
		TPD_DEBUG("GT:invalid touch head");
		return IRQ_IGNORE;
	}

	if (checksum_cmp(chip_info->touch_data,
			 IRQ_EVENT_HEAD_LEN, CHECKSUM_MODE_U8_LE)) {
		TPD_DEBUG("GT:%s: [checksum err !!]touch_head %*ph\n", __func__, IRQ_EVENT_HEAD_LEN,
			chip_info->touch_data);
		goto exit;
	}

	event_status = chip_info->touch_data[IRQ_EVENT_TYPE_OFFSET];
	if (event_status & GOODIX_TOUCH_EVENT) {
		touch_num = chip_info->touch_data[POINT_NUM_OFFSET] & 0x0F;
		if (touch_num > MAX_POINT_NUM) {
			TPD_DEBUG("GT:invalid touch num %d\n", touch_num);
			goto exit;
		}
		if (chip_info->kernel_grip_support == true) {
			chip_info->abnormal_grip_coor = false;
			if (event_status & GRIP_COOR_SUPPORT_FLAG) {
				chip_info->get_grip_coor = true;
				ret = goodix_reg_read(chip_info,
					EDGE_INPUT_COORD, chip_info->edge_data,
					EDGE_INPUT_OFFSET * touch_num + 2);
				if (ret < 0) {
					TPD_INFO("GT:%s:spi transfer error!\n", __func__);
					goto exit;
				}
				TPD_DEBUG("GT:grip bit->[0x%x]->girp[%*ph]\n",
					event_status & GRIP_COOR_SUPPORT_FLAG, 16, chip_info->edge_data);
				if (checksum_cmp(chip_info->edge_data,
						EDGE_INPUT_OFFSET * touch_num + 2,
						CHECKSUM_MODE_U8_LE) && touch_num > TOUCH_UP) {
					chip_info->abnormal_grip_coor = true;
						TPD_INFO("GT:%s: [checksum err!!] girp:[%*ph]\n", __func__,
							16, chip_info->edge_data);
				}
			} else {
				chip_info->get_grip_coor = false;
			}
		}

		if (unlikely(touch_num > 2)) {
			ret = goodix_reg_read(chip_info,
					misc->touch_data_addr + pre_read_len,
					&chip_info->touch_data[pre_read_len],
					(touch_num - 2) * BYTES_PER_POINT);
			if (ret < 0) {
				TPD_DEBUG("GT:read touch point data from coor_addr failed!\n");
				goto exit;
			}
		}

		point_type = chip_info->touch_data[IRQ_EVENT_HEAD_LEN] & 0x0F;
		if (point_type == POINT_TYPE_STYLUS ||
				point_type == POINT_TYPE_STYLUS_HOVER) {
			ret = checksum_cmp(&chip_info->touch_data[IRQ_EVENT_HEAD_LEN],
					BYTES_PER_POINT * 2 + 2, CHECKSUM_MODE_U8_LE);
		} else {
			ret = checksum_cmp(&chip_info->touch_data[IRQ_EVENT_HEAD_LEN],
					BYTES_PER_POINT * touch_num + 2, CHECKSUM_MODE_U8_LE);
		}
		if (ret < 0) {
			TPD_DEBUG("GT:touch data checksum error\n");
			goto exit;
		}
	} else if (!(event_status & (GOODIX_GESTURE_EVENT | GOODIX_FINGER_IDLE_EVENT))) {
		/*TODO check this event*/
	}

	if (gesture_enable && is_suspended &&
		(event_status & GOODIX_GESTURE_EVENT)) {
		result_event = IRQ_GESTURE;
		goto exit;
	} else if (is_suspended) {
		goto exit;
	}
	if (event_status & GOODIX_FINGER_IDLE_EVENT) {
		goto exit;
	}
	if (event_status & GOODIX_REQUEST_EVENT) {/*int request*/
		result_event = IRQ_FW_CONFIG;
		goto exit;
	}
	if (event_status & GOODIX_FINGER_STATUS_EVENT) {
		SET_BIT(result_event, IRQ_FW_HEALTH);
	}

	if (event_status & GOODIX_TOUCH_EVENT) {
		if (point_type == POINT_TYPE_STYLUS ||
				point_type == POINT_TYPE_STYLUS_HOVER) {
		} else if (chip_info->pen_state == PEN_UP) {
			SET_BIT(result_event, IRQ_TOUCH);
		}

		if ((event_status & GOODIX_FINGER_PRINT_EVENT) &&
		    !is_suspended && (chip_info->fp_down_flag == false)) {
			chip_info->fp_down_flag = true;
			SET_BIT(result_event, IRQ_FINGERPRINT);
		} else if (!is_suspended && (event_status & GOODIX_FINGER_PRINT_EVENT) &&
			   (chip_info->fp_down_flag == true)) {
			chip_info->fp_down_flag = false;
			SET_BIT(result_event, IRQ_FINGERPRINT);
		}
	}

exit:
	/* read done */
	goodix_clear_irq(chip_info);
	return result_event;
}


static int goodix_get_touch_points(void *chip_data,
			struct point_info *points, int max_num)
{
	int true_num;
	int touch_map = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	u8 touch_num = 0;
	u8 *coor_data = NULL;
	u8 *ew_data = NULL;
	s32 id = 0;

	touch_num = chip_info->touch_data[POINT_NUM_OFFSET] & 0x0F;

	TPD_DEBUG("GT:%s:check %d points data:[%*ph]\n", __func__,
		touch_num,
		TOUCH_POINT_OFFECT * touch_num + TOUCH_CHECKSUM_OFFECT,
		chip_info->touch_data + TOUCH_ALL_OFFECT);

	if (touch_num > MAX_POINT_NUM) {
		TPD_INFO("GT:%s:ERR!! touch_num OVER LIMIT->%d\n", __func__, touch_num);
		goto END_TOUCH;
	}

	chip_info->touch_state = TOUCH_DOWN;
	coor_data = &chip_info->touch_data[IRQ_EVENT_HEAD_LEN];
	ew_data   = &chip_info->edge_data[0];

	for (true_num = 0; true_num < touch_num; true_num++) {
		id = (coor_data[0] >> 4) & FINGER_CHECK;
		if (id >= MAX_POINT_NUM) {
			TPD_INFO("GT:%s ERR!!->invalidID:%d,abnormalNUM:%d\n",
				__func__, id, true_num);
			continue;
		}

		/* normal data */
		points[id].x      = le16_to_cpup((__le16 *)(coor_data + 2));
		points[id].y      = le16_to_cpup((__le16 *)(coor_data + 4));
		points[id].touch_major    = max(coor_data[6], coor_data[7]);
		points[id].width_major    = min(coor_data[6], coor_data[7]);
		/* touch status */
		points[id].status = TOUCH_DOWN;
		ew_data   += BYTES_PER_EDGE;
		coor_data += BYTES_PER_POINT;
		touch_map |= 0x01 << id;
	}

END_TOUCH:
	return touch_map;
}

static void goodix_gesture_coordiate(struct chip_data_brl *chip_info, struct gesture_info *gesture)
{
	u8 *coor_data = NULL;

	coor_data = &chip_info->touch_data[IRQ_EVENT_HEAD_LEN];
	gesture->Point_start.x = le16_to_cpup((__le16 *)(coor_data));
	gesture->Point_start.y = le16_to_cpup((__le16 *)(coor_data + GESTURE_DATA_ADDR_OFFECT));

	TPD_INFO("GT:%s:(X,Y)=>(%5d,%5d):reg_data=>[%*ph]",
		__func__,
		gesture->Point_start.x,
		gesture->Point_start.y,
		GESTURE_DATA_ADDR_SIZE, coor_data);
}

static int goodix_get_gesture_info(void *chip_data, struct gesture_info *gesture)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	u8 gesture_type;

	TPD_DEBUG("GT:%s:check gesture data = [%*ph]\n", __func__,
		IRQ_EVENT_HEAD_LEN, chip_info->touch_data);

	gesture_type = chip_info->touch_data[4];
	TPD_INFO("GT:%s: get gesture type:0x%x\n", __func__, gesture_type);

	switch (gesture_type) {
	case GOODIX_LEFT2RIGHT_SWIP:
		gesture->gesture_type = LEFT2RIGHT_SWIP;
	break;
	case GOODIX_RIGHT2LEFT_SWIP:
		gesture->gesture_type = RIGHT2LEFT_SWIP;
	break;
	case GOODIX_UP2DOWN_SWIP:
		gesture->gesture_type = UP2DOWN_SWIP;
	break;
	case GOODIX_DOWN2UP_SWIP:
		gesture->gesture_type = DOWN2UP_SWIP;
	break;
	case GOODIX_DOU_SWIP:
		gesture->gesture_type = DOU_SWIP;
	break;
	case GOODIX_DOU_TAP:
		gesture->gesture_type = DOU_TAP;
	break;
	case GOODIX_SINGLE_TAP:
		gesture->gesture_type = SINGLE_TAP;
	break;
	case GOODIX_UP_VEE:
		gesture->gesture_type = UP_VEE;
	break;
	case GOODIX_DOWN_VEE:
		gesture->gesture_type = DOWN_VEE;
	break;
	case GOODIX_LEFT_VEE:
		gesture->gesture_type = LEFT_VEE;
	break;
	case GOODIX_RIGHT_VEE:
		gesture->gesture_type = RIGHT_VEE;
	break;
	case GOODIX_CIRCLE_GESTURE:
		gesture->gesture_type = CIRCLE_GESTURE;
	break;
	case GOODIX_M_GESTRUE:
		gesture->gesture_type = M_GESTRUE;
	break;
	case GOODIX_W_GESTURE:
		gesture->gesture_type = W_GESTURE;
	break;
	default:
		TPD_INFO("GT:%s: unknown gesture type[0x%x]\n", __func__, gesture_type);
	break;
	}

	goodix_gesture_coordiate(chip_info, gesture);

	return 0;
}

static int goodix_mode_switch(void *chip_data, work_mode mode, int flag)
{
	int ret = -1;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	if (!chip_info->ic_info.length) {
		if ((mode == MODE_NORMAL) && (flag == true)) {
			TPD_INFO("GT:%s: goodix ic info invalid, but probe, continue\n", __func__);
			return 0;
		}
		TPD_INFO("GT:%s: goodix ic info invalid\n", __func__);
		return ret;
	}

	if (chip_info->halt_status && (mode != MODE_NORMAL)) {
		goodix_reset(chip_info);
	}

	switch (mode) {
	case MODE_NORMAL:
		ret = 0;
		break;

	case MODE_SLEEP:
		ret = goodix_enter_sleep(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT:%s: goodix enter sleep failed\n", __func__);
		}
		break;

	case MODE_GESTURE:
		ret = goodix_enable_gesture(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT:%s: goodix enable:(%d) gesture failed.\n", __func__, flag);
			return ret;
		}
		break;

	case MODE_EDGE:
		ret = goodix_enable_edge_limit(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT:%s: goodix enable:(%d) edge limit failed.\n", __func__, flag);
			return ret;
		}
		break;

	case MODE_CHARGE:
		ret = goodix_enable_charge_mode(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT:%s: enable charge mode : %d failed\n", __func__, flag);
		}
		break;

	case MODE_GAME:
		ret = goodix_enable_game_mode(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT:%s: enable game mode : %d failed\n", __func__, flag);
		}
		break;

	default:
		TPD_INFO("GT:%s: mode %d not support.\n", __func__, mode);
	}

	return ret;
}

static int goodix_esd_handle(void *chip_data)
{
	s32 ret = -1;
	u8 esd_buf = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;

	if (!chip_info->esd_check_enabled || !misc->esd_addr) {
		TPD_DEBUG("GT:%s: close\n", __func__);
		return 0;
	}

	ret = goodix_reg_read(chip_info, misc->esd_addr, &esd_buf, 1);
	if ((ret < 0) || esd_buf == 0xAA) {
		TPD_INFO("GT:%s: esd dynamic esd occur, ret = %d, esd_buf = %d.\n",
			 __func__, ret, esd_buf);
		TPD_INFO("GT:%s: IC works abnormally! Process esd reset.\n", __func__);
		disable_irq_nosync(chip_info->irq);

		goodix_power_control(chip_info, false);
		msleep(30);
		goodix_power_control(chip_info, true);
		usleep_range(10000, 10100);

		goodix_reset(chip_data);

		tp_touch_btnkey_release(chip_info->tp_index);

		enable_irq(chip_info->irq);
		TPD_INFO("GT:%s: Goodix esd reset over.\n", __func__);
		chip_info->esd_err_count++;
		return -1;
	} else {
		esd_buf = 0xAA;
		ret = goodix_reg_write(chip_info,
				       chip_info->ic_info.misc.esd_addr, &esd_buf, 1);
		if (ret < 0) {
			TPD_INFO("GT:%s: Failed to reset esd reg.\n", __func__);
		}
	}

	return 0;
}

static void goodix_enable_fingerprint_underscreen(void *chip_data, uint32_t enable)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("GT:%s, enable = %d\n", __func__, enable);
	if (enable) {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_FOD_FINGER_PRINT, GTP_MASK_DISABLE);
	} else {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_FOD_FINGER_PRINT, GTP_MASK_ENABLE);
	}

	return;
}

static void goodix_enable_gesture_mask(void *chip_data, uint32_t enable)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("GT:%s, enable = %d\n", __func__, enable);
	TPD_INFO("GT:%s: gesture_type:0x%08X\n", __func__, chip_info->gesture_type);
	/* if (enable) {
		enable all gesture type
		chip_info->gesture_type = 0xFFFFFFFF;
	} else {
		chip_info->gesture_type = 0x00000000;
	} */
}

static void goodix_set_gesture_state(void *chip_data, int state)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_DEBUG("GT:%s: ready change state & support_gesture_type -> 0x%2x & 0x%2x\n",
		__func__, state, chip_info->gesture_type);

	state = state & chip_info->support_gesture_type;

	TPD_DEBUG("GT:%s: changed state & support_gesture_type -> 0x%2x & 0x%2x\n",
		__func__, state, chip_info->gesture_type);

	SET_GESTURE_BIT(state, DOU_TAP, chip_info->gesture_type, GTP_GESTURE_DOU_TAP);
	SET_GESTURE_BIT(state, UP_VEE, chip_info->gesture_type, GTP_GESTURE_UP_VEE);
	SET_GESTURE_BIT(state, DOWN_VEE, chip_info->gesture_type, GTP_GESTURE_DOWN_VEE);
	SET_GESTURE_BIT(state, LEFT_VEE, chip_info->gesture_type, GTP_GESTURE_LEFT_VEE);
	SET_GESTURE_BIT(state, RIGHT_VEE, chip_info->gesture_type, GTP_GESTURE_RIGHT_VEE);
	SET_GESTURE_BIT(state, CIRCLE_GESTURE, chip_info->gesture_type, GTP_GESTURE_CIRCLE);
	SET_GESTURE_BIT(state, DOU_SWIP, chip_info->gesture_type, GTP_GESTURE_DOU_SWIP);
	SET_GESTURE_BIT(state, LEFT2RIGHT_SWIP, chip_info->gesture_type, GTP_GESTURE_L2R_SWIP);
	SET_GESTURE_BIT(state, RIGHT2LEFT_SWIP, chip_info->gesture_type, GTP_GESTURE_R2L_SWIP);
	SET_GESTURE_BIT(state, UP2DOWN_SWIP, chip_info->gesture_type, GTP_GESTURE_U2D_SWIP);
	SET_GESTURE_BIT(state, DOWN2UP_SWIP, chip_info->gesture_type, GTP_GESTURE_D2U_SWIP);
	SET_GESTURE_BIT(state, M_GESTRUE, chip_info->gesture_type, GTP_GESTURE_M);
	SET_GESTURE_BIT(state, W_GESTURE, chip_info->gesture_type, GTP_GESTURE_W);
	SET_GESTURE_BIT(state, SINGLE_TAP, chip_info->gesture_type, GTP_GESTURE_SIN_TAP);
	/* SET_GESTURE_BIT(state, HEART, chip_info->gesture_type, HEART); */
	/* SET_GESTURE_BIT(state, S_GESTURE, chip_info->gesture_type, S_GESTURE); */

	TPD_INFO("GT:%s: gesture_type:0x%08X\n", __func__, chip_info->gesture_type);
}

static void goodix_screenon_fingerprint_info(void *chip_data,
		struct fp_underscreen_info *fp_tpinfo)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	if (chip_info->fp_down_flag) {
		fp_tpinfo->x = chip_info->fp_coor_report.fp_x_coor;
		fp_tpinfo->y = chip_info->fp_coor_report.fp_y_coor;
		fp_tpinfo->area_rate = chip_info->fp_coor_report.fp_area;
		fp_tpinfo->touch_state = FINGERPRINT_DOWN_DETECT;
	} else {
		fp_tpinfo->x = chip_info->fp_coor_report.fp_x_coor;
		fp_tpinfo->y = chip_info->fp_coor_report.fp_y_coor;
		fp_tpinfo->area_rate = chip_info->fp_coor_report.fp_area;
		fp_tpinfo->touch_state = FINGERPRINT_UP_DETECT;
	}
}

static int goodix_request_event_handler(struct chip_data_brl *chip_info)
{
	int ret = -1;
	u8 rqst_code = 0;

	rqst_code = chip_info->touch_data[REQUEST_EVENT_TYPE_OFFSET];
	TPD_INFO("GT:%s: request state:0x%02x.\n", __func__, rqst_code);

	switch (rqst_code) {
	case GTP_RQST_CONFIG:
		TPD_INFO("GT:HW request config.\n");
		ret = goodix_send_config(chip_info, chip_info->normal_cfg.data,
					 chip_info->normal_cfg.length);
		if (ret) {
			TPD_INFO("GT:request config, send config faild.\n");
		}
		break;
	case GTP_RQST_RESET:
		TPD_INFO("GT:%s: HW requset reset.\n", __func__);
		goodix_reset(chip_info);
		break;
	default:
		TPD_INFO("GT:%s: Unknown hw request:%d.\n", __func__, rqst_code);
		break;
	}

	return 0;
}

static int goodix_fw_handle(void *chip_data)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	ret = goodix_request_event_handler(chip_info);

	return ret;
}

static void goodix_register_info_read(void *chip_data,
			uint16_t register_addr, uint8_t *result, uint8_t length)
{
	/*struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TODO need change oplus framework to support u32 address*/
}

static void goodix_set_touch_direction(void *chip_data, uint8_t dir)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	chip_info->touch_direction = dir;
}

static uint8_t goodix_get_touch_direction(void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	return chip_info->touch_direction;
}

static int goodix_specific_resume_operate(void *chip_data,
		struct specific_resume_data *p_resume_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	TPD_DEBUG("GT:%s call\n", __func__);
	goodix_esd_check_enable(chip_info, true);
	return 0;
}

static int goodix_set_smooth_lv_set(void *chip_data, int level)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	int ret = 0;

	TPD_INFO("GT:%s, %s smooth ->%d\n", __func__, level > 0 ? "enable" : "disable", level);

	if (level > 0) {
		if (goodix_send_cmd_simple(chip_info, GTP_SMOOTH_CMD, (u8)level) < 0) {
			TPD_INFO("GT:%s, fail set smooth ->%d\n", __func__, level);
			ret = -1;
		}
	} else if (level == 0) {
		if (goodix_send_cmd_simple(chip_info, GTP_SMOOTH_CMD, GTP_MASK_DISABLE) < 0) {
			TPD_INFO("GT:%s, fail disable smooth\n", __func__, level);
			ret = -1;
		}
	}

	return ret;
}

static int goodix_set_sensitive_lv_set(void *chip_data, int level)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
		int ret = 0;

	TPD_INFO("GT:%s, %s sensitive ->%d\n", __func__, level > 0 ? "enable" : "disable", level);

	if (level > 0) {
		if (goodix_send_cmd_simple(chip_info, GTP_SENSITIVE_CMD, (u8)level) < 0) {
			TPD_INFO("GT:%s, fail set sensitive ->%d\n", __func__, level);
			ret = -1;
		}
	} else if (level == 0) {
		if (goodix_send_cmd_simple(chip_info, GTP_SENSITIVE_CMD, GTP_MASK_DISABLE) < 0) {
			TPD_INFO("GT:%s, fail disable sensitive\n", __func__, level);
			ret = -1;
		}
	}

	return ret;
}


/* high frame default enable 60s */
static int goodix_set_high_frame_rate(void *chip_data, int level, int time)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	int ret = 0;

	TPD_INFO("GT:%s, %s high frame rate, %d\n", __func__, !!level > 0 ? "enable" : "disable", time);
	if (!!level) {
		ret = goodix_send_cmd_simple(chip_info, GTP_GAME_HIGH_FRAME, GTP_MASK_ENABLE);
	} else {
		ret = goodix_send_cmd_simple(chip_info, GTP_GAME_HIGH_FRAME, GTP_MASK_DISABLE);
	}

	return 0;
}

static int goodix_set_refresh_switch(void *chip_data, int fps)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	bool check_enable = false;
	int ret = 0;

	TPD_DEBUG("GT:%s: refresh_switch: %d HZ!\n", __func__, fps);
	if (chip_info == NULL) {
		return -1;
	}

	check_enable = chip_info->game_enable & \
		chip_info->sleep_enable & \
		chip_info->pen_enable & \
		chip_info->gesture_enable;

	chip_info->display_refresh_rate = fps;

	if ((!check_enable) && (chip_info->rate_num_support == true)) {
		switch (chip_info->display_refresh_rate) {
		case GOODIX_60_FPS:
			if (chip_info->default_rate_set) {
				ret = goodix_send_cmd_simple(chip_info, GTP_CMD_SIXTY_CMD, GTP_MASK_ENABLE);
				chip_info->default_rate_set = false;
			}
		break;
		default:
			if (!chip_info->default_rate_set) {
				ret = goodix_send_cmd_simple(chip_info, GTP_CMD_SIXTY_CMD, GTP_MASK_DISABLE);
				chip_info->default_rate_set = true;
			}
		break;
		}
	} else {
		TPD_INFO("GT:%s: not change rate, check enable->game:%d,pen:%d,gesture:%d,sleep:%d\n",
			__func__, chip_info->game_enable, chip_info->pen_enable, chip_info->gesture_enable, chip_info->sleep_enable);
	}

	return 0;
}

static int goodix_send_temperature(void *chip_data, int temp, bool normal_mode)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	int ret = 0;

	if (BIT_CHK(chip_info->check_now_state, SELF_TEST_ENABLE)) {
		TPD_INFO("GT:%s :self testing, not send temp to mcu, %d", __func__, chip_info->check_now_state);
		goto OUT;
	}

	if (normal_mode == true) {
		if (chip_info->temp_recorder_cnt == CNT_MAX) {
			chip_info->temp_recorder_cnt = CNT_CLR;
			if (temp != chip_info->gt_temperature[0] && temp != chip_info->gt_temperature[1]) {
				ret = goodix_send_cmd_simple(chip_info, GTP_CMD_TEMP_CMD, temp);
				TPD_INFO("GT:%s : cnt max, need temp:%d -> ic", __func__, temp);
			}
		}

		if (temp == chip_info->gt_temperature[0]) {
			ret = goodix_send_cmd_simple(chip_info, GTP_CMD_TEMP_CMD, chip_info->gt_temperature[0]);
			TPD_INFO("GT:%s : send temp %d -> ic", __func__, chip_info->gt_temperature[0]);
		} else if (temp == chip_info->gt_temperature[1]) {
			ret = goodix_send_cmd_simple(chip_info, GTP_CMD_TEMP_CMD, chip_info->gt_temperature[1]);
			TPD_INFO("GT:%s : send temp %d -> ic", __func__, chip_info->gt_temperature[1]);
		}

		chip_info->temp_recorder_cnt++;
	} else {
		TPD_INFO("%s : now resume, must send temp:%d to ic\n", __func__, temp);
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_TEMP_CMD, temp);
	}

OUT:
	return 0;
}

struct oplus_touchpanel_operations goodix_ops = {
	.ftm_process                 = goodix_ftm_process,
	.get_vendor                  = goodix_get_vendor,
	.get_chip_info               = goodix_get_chip_info,
	.reset                       = goodix_reset,
	.power_control               = goodix_power_control,
	.fw_check                    = goodix_fw_check,
	.fw_update                   = goodix_fw_update,
	.trigger_reason              = goodix_u32_trigger_reason,
	.get_touch_points            = goodix_get_touch_points,
	.get_gesture_info            = goodix_get_gesture_info,
	.mode_switch                 = goodix_mode_switch,
	.esd_handle                  = goodix_esd_handle,
	.fw_handle                   = goodix_fw_handle,
	.register_info_read          = goodix_register_info_read,
	.enable_fingerprint          = goodix_enable_fingerprint_underscreen,
	.enable_gesture_mask         = goodix_enable_gesture_mask,
	.screenon_fingerprint_info   = goodix_screenon_fingerprint_info,
	.set_gesture_state         	 = goodix_set_gesture_state,
	.set_touch_direction         = goodix_set_touch_direction,
	.get_touch_direction         = goodix_get_touch_direction,
	.specific_resume_operate     = goodix_specific_resume_operate,
	.set_high_frame_rate         = goodix_set_high_frame_rate,
	.smooth_lv_set               = goodix_set_smooth_lv_set,
	.sensitive_lv_set            = goodix_set_sensitive_lv_set,
	.tp_refresh_switch           = goodix_set_refresh_switch,
	.send_temperature            = goodix_send_temperature,
};
/********* End of implementation of oplus_touchpanel_operations callbacks**********************/

/************** Start of callback of proc/Goodix/config_version node**************************/

/* success return config_len, <= 0 failed */
int goodix_read_config(struct chip_data_brl *chip_info, u8 *cfg, int size)
{
	int ret;
	struct goodix_ts_cmd cfg_cmd;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;
	struct goodix_config_head cfg_head;

	if (!cfg) {
		return -1;
	}

	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_READ_START;
	ret = send_cfg_cmd(chip_info, &cfg_cmd);
	if (ret) {
		TPD_INFO("GT:%s: failed send config read prepare command\n", __func__);
		return ret;
	}

	ret = goodix_reg_read(chip_info, misc->fw_buffer_addr,
			      cfg_head.buf, sizeof(cfg_head));
	if (ret) {
		TPD_INFO("GT:%s: failed read config head %d\n", __func__, ret);
		goto exit;
	}

	if (checksum_cmp(cfg_head.buf, sizeof(cfg_head), CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("GT:%s: config head checksum error\n", __func__);
		ret = -1;
		goto exit;
	}

	cfg_head.cfg_len = le16_to_cpu(cfg_head.cfg_len);
	if (cfg_head.cfg_len > misc->fw_buffer_max_len ||
	    cfg_head.cfg_len > size) {
		TPD_INFO("GT:%s: cfg len exceed buffer size %d > %d\n", __func__, cfg_head.cfg_len,
			 misc->fw_buffer_max_len);
		ret = -1;
		goto exit;
	}

	memcpy(cfg, cfg_head.buf, sizeof(cfg_head));
	ret = goodix_reg_read(chip_info, misc->fw_buffer_addr + sizeof(cfg_head),
			      cfg + sizeof(cfg_head), cfg_head.cfg_len);
	if (ret) {
		TPD_INFO("GT:%s: failed read cfg pack, %d\n", __func__, ret);
		goto exit;
	}

	TPD_INFO("GT:config len %d\n", cfg_head.cfg_len);
	if (checksum_cmp(cfg + sizeof(cfg_head),
			 cfg_head.cfg_len, CHECKSUM_MODE_U16_LE)) {
		TPD_INFO("GT:%s: config body checksum error\n", __func__);
		ret = -1;
		goto exit;
	}
	TPD_INFO("GT:success read config data: len %zu\n",
		 cfg_head.cfg_len + sizeof(cfg_head));
exit:
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_READ_EXIT;
	if (send_cfg_cmd(chip_info, &cfg_cmd)) {
		TPD_INFO("GT:%s: failed send config read finish command\n", __func__);
		ret = -1;
	}
	if (ret) {
		return -1;
	}
	return cfg_head.cfg_len + sizeof(cfg_head);
}

static void goodix_config_info_read(struct seq_file *s, void *chip_data)
{
	int ret = 0, i = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_fw_version *fw_version = &chip_info->ver_info;
	struct goodix_ic_info *ic_info = &chip_info->ic_info;

	seq_printf(s, "==== Goodix default config setting in driver====\n");
	for (i = 0; i < TS_CFG_MAX_LEN && i < chip_info->normal_cfg.length; i++) {
		seq_printf(s, "0x%02X, ", chip_info->normal_cfg.data[i]);
		if ((i + 1) % 32 == 0) {
			seq_printf(s, "\n");
		}
	}
	seq_printf(s, "\n");
	seq_printf(s, "==== Goodix test cfg in driver====\n");
	for (i = 0; i < TS_CFG_MAX_LEN && i < chip_info->test_cfg.length; i++) {
		seq_printf(s, "0x%02X, ", chip_info->test_cfg.data[i]);
		if ((i + 1) % 32 == 0) {
			seq_printf(s, "\n");
		}
	}
	seq_printf(s, "\n");
	seq_printf(s, "==== Goodix noise test cfg in driver====\n");
	for (i = 0; i < TS_CFG_MAX_LEN && i < chip_info->noise_test_cfg.length; i++) {
		seq_printf(s, "0x%02X, ", chip_info->noise_test_cfg.data[i]);
		if ((i + 1) % 32 == 0) {
			seq_printf(s, "\n");
		}
	}
	seq_printf(s, "\n");

	seq_printf(s, "==== Goodix config read from chip====\n");

	ret = brl_get_ic_info(chip_info, ic_info);
	if (ret) {
		TPD_INFO("GT:%s: failed get ic info, ret %d\n", __func__, ret);
		goto exit;
	}
	ret = brl_read_version(chip_info, fw_version);
	if (ret) {
		TPD_INFO("GT:goodix_config_info_read goodix_read_config error:%d\n", ret);
		goto exit;
	}

	seq_printf(s, "\n");
	seq_printf(s, "==== Goodix Version Info ====\n");
	seq_printf(s, "ConfigVer: 0x%02X\n", ic_info->version.config_version);
	seq_printf(s, "ProductID: GT%s\n", fw_version->patch_pid);
	seq_printf(s, "PatchID: %*ph\n", (int)sizeof(fw_version->patch_pid), fw_version->patch_pid);
	seq_printf(s, "MaskID: %*ph\n", (int)sizeof(fw_version->rom_vid), fw_version->rom_vid);
	seq_printf(s, "SensorID: %d\n", fw_version->sensor_id);

exit:
	return;
}
/*************** End of callback of proc/Goodix/config_version node***************************/

struct goodix_proc_operations goodix_brl_proc_ops = {
	.goodix_config_info_read    = goodix_config_info_read,
};

static int goodix_gt9966_ts_probe(struct spi_device *spi)
{
	struct chip_data_brl *chip_info = NULL;
	struct touchpanel_data *ts = NULL;
	int ret = -1;

	TPD_INFO("GT:Goodix driver version: %s\n", GOODIX_DRIVER_VERSION);

	/* 2. Alloc chip_info */
	chip_info = kzalloc(sizeof(struct chip_data_brl), GFP_KERNEL);
	if (chip_info == NULL) {
		TPD_INFO("GT:chip info kzalloc error\n");
		ret = -ENOMEM;
		return ret;
	}

	/* 3. Alloc common ts */
	ts = common_touch_data_alloc();
	if (ts == NULL) {
		TPD_INFO("GT:ts kzalloc error\n");
		goto ts_malloc_failed;
	}

	/* 4. alloc touch data space */
	chip_info->touch_data = kzalloc(MAX_GT_IRQ_DATA_LENGTH, GFP_KERNEL);
	if (chip_info->touch_data == NULL) {
		TPD_INFO("GT:touch_data kzalloc error\n");
		goto err_register_driver;
	}

	chip_info->edge_data = kzalloc(MAX_GT_EDGE_DATA_LENGTH, GFP_KERNEL);
	if (chip_info->edge_data == NULL) {
		TPD_INFO("GT:edge_data kzalloc error\n");
		goto err_touch_data_alloc;
	}

	/* init spi_device */
	spi->mode          = SPI_MODE_0;
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret) {
		TPD_INFO("GT:failed set spi mode, %d\n", ret);
		goto err_edge_data_alloc;
	}

	ts->dev = &spi->dev;
	ts->s_client = spi;
	ts->irq = spi->irq;
	ts->chip_data = chip_info;
	spi_set_drvdata(spi, ts);

	chip_info->hw_res = &ts->hw_res;
	chip_info->s_client = spi;
	chip_info->goodix_ops = &goodix_brl_proc_ops;
	/* 6. file_operations callbacks binding */
	ts->ts_ops = &goodix_ops;

	chip_info->rate_num_support = false;
	chip_info->default_rate_set = true;
	ts->bus_type = TP_BUS_SPI;

	/* 8. register common touch device */
	ret = register_common_touch_device(ts);
	if (ret < 0) {
		goto err_edge_data_alloc;
	}

	chip_info->tp_index = ts->tp_index;

	chip_info->max_x = ts->resolution_info.max_x;
	chip_info->max_y = ts->resolution_info.max_y;
	chip_info->pen_support =      false;
	chip_info->game_enable =      false;
	chip_info->gesture_enable =   false;
	chip_info->pen_enable =       false;
	chip_info->sleep_enable =     false;
	chip_info->check_now_state =       0;
	chip_info->check_old_state =       0;
	chip_info->check_chg_state =       0;
	chip_info->pen_state =        PEN_UP;
	chip_info->ts = ts;

	mutex_init(&chip_info->debug_lock);

	goodix_esd_check_enable(chip_info, true);

	ts->tp_suspend_order = TP_LCD_SUSPEND;

	TPD_INFO("GT:%s, probe normal end\n", __func__);

	return 0;

err_edge_data_alloc:
	if (chip_info->edge_data) {
		kfree(chip_info->edge_data);
	}
	chip_info->edge_data = NULL;

err_touch_data_alloc:
	if (chip_info->touch_data) {
		kfree(chip_info->touch_data);
	}
	chip_info->touch_data = NULL;

err_register_driver:
	common_touch_data_free(ts);
	ts = NULL;

ts_malloc_failed:
	kfree(chip_info);
	chip_info = NULL;

	TPD_INFO("GT:%s, probe error\n", __func__);
	return ret;
}

static int goodix_ts_pm_suspend(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("GT:%s: is called\n", __func__);
	tp_pm_suspend(ts);
	return 0;
}

static int goodix_ts_pm_resume(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("GT:%s is called\n", __func__);
	tp_pm_resume(ts);
	return 0;
}

static void goodix_gt9966_tp_shutdown(struct spi_device *s_client)
{
	struct touchpanel_data *ts = spi_get_drvdata(s_client);

	TPD_INFO("GT:%s is called\n", __func__);

	tp_shutdown(ts);
}

static int __maybe_unused goodix_gt9966_ts_remove(struct spi_device *s_client)
{
	struct touchpanel_data *ts = spi_get_drvdata(s_client);
	struct chip_data_brl *chip_info = NULL;
	if (!ts) {
		TPD_INFO("GT:%s spi_get_drvdata(s_client) is null.\n", __func__);
		return -EINVAL;
	}

	chip_info = (struct chip_data_brl *)ts->chip_data;

	TPD_INFO("GT:%s is called\n", __func__);

	goodix_remove_proc(ts);
	gtx8_deinit_tool_node(ts);
	unregister_common_touch_device(ts);
	common_touch_data_free(ts);
	mutex_destroy(&chip_info->debug_lock);

	tp_kfree((void **)&ts);
	tp_kfree((void **)&chip_info);

	spi_set_drvdata(s_client, NULL);

	return 0;
}

static const struct dev_pm_ops gt9966_pm_ops = {
	.suspend = goodix_ts_pm_suspend,
	.resume = goodix_ts_pm_resume,
};

static const struct spi_device_id gt9966_id[] = {
	{ TPD_DEVICE, 0},
	{ }
};

static struct of_device_id gt9966_match_table[] = {
	{ .compatible = TPD_DEVICE, },
	{ .compatible = "goodix-gt9966", },
	{ }
};

static struct spi_driver gt9966_ts_driver = {
	.probe = goodix_gt9966_ts_probe,
	.remove = goodix_gt9966_ts_remove,
	.id_table   = gt9966_id,
	.shutdown   = goodix_gt9966_tp_shutdown,
	.driver = {
		.name   = TPD_DEVICE,
		.of_match_table = gt9966_match_table,
		.pm = &gt9966_pm_ops,
	},
};

/***********************Start of module init and exit****************************/
int __init tp_driver_init_gt9966(void)
{
	TPD_INFO("GT:%s is called\n", __func__);

	if (!tp_judge_ic_match(TPD_DEVICE)) {
		TPD_INFO("GT:%s not match\n", __func__);
		goto OUT;
	}

	if (spi_register_driver(&gt9966_ts_driver) != 0) {
		TPD_INFO("GT:%s : unable to add spi driver.\n", __func__);
		goto OUT;
	}

OUT:
	return 0;
}

void __exit tp_driver_exit_gt9966(void)
{
	TPD_INFO("GT:%s : Core layer exit", __func__);
	spi_unregister_driver(&gt9966_ts_driver);
}

#ifdef CONFIG_TOUCHPANEL_LATE_INIT
late_initcall(tp_driver_init_gt9966);
#else
module_init(tp_driver_init_gt9966);
#endif

module_exit(tp_driver_exit_gt9966);
/***********************End of module init and exit*******************************/
MODULE_AUTHOR("Goodix, Inc.");
MODULE_DESCRIPTION("GTP Touchpanel Driver");
MODULE_LICENSE("GPL v2");
