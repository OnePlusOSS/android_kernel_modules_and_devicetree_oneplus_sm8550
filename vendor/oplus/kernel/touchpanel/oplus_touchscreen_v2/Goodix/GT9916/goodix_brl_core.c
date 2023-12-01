// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#include <linux/thermal.h>
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

#define MAX_DRV_NUM_BRD				    	20
#define MAX_SEN_NUM_BRD				    	40
#define SHORT_TEST_TIME_REG_BRD				0x14D7A
#define SHORT_TEST_STATUS_REG_BRD			0x13400
#define SHORT_TEST_RESULT_REG_BRD			0x13408
#define DRV_DRV_SELFCODE_REG_BRD			0x1344E
#define SEN_SEN_SELFCODE_REG_BRD 			0x137E6
#define DRV_SEN_SELFCODE_REG_BRD			0x14556
#define DIFF_CODE_DATA_REG_BRD				0x14D00

#define SHORT_TEST_FINISH_FLAG  			0x88
#define SHORT_TEST_THRESHOLD_REG			0x20402
#define SHORT_TEST_RUN_REG					0x10400
#define SHORT_TEST_RUN_FLAG					0xAA
#define INSPECT_FW_SWITCH_CMD				0x85

static u8 brl_d_drv_map[] = {
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 50, 51, 52, 53, 54, 55,
	56, 57, 58, 59,
};

static u8 brl_d_sen_map[] = {
	0, 1, 2, 3, 4, 5, 6, 7,
	8, 9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
};

typedef struct __attribute__((packed)) {
	u8 result;
	u8 drv_drv_num;
	u8 sen_sen_num;
	u8 drv_sen_num;
	u8 drv_gnd_avdd_num;
	u8 sen_gnd_avdd_num;
	u16 checksum;
} test_result_t;

static int cal_cha_to_cha_res(int v1, int v2)
{
	return (v1 / v2 - 1) * 70 + 59;
}

static int cal_cha_to_avdd_res(int v1, int v2)
{
	return 64 * (2 * v2 - 25) * 93 / v1 - 20;
}

static int cal_cha_to_gnd_res(int v)
{
	return 145000 / v - 15;
}

static int  goodix_reset(void *chip_data);
static void goodix_state_verify(struct chip_data_brl *chip_info);
static void goodix_check_bit_set(struct chip_data_brl *chip_info, unsigned int bit, bool enable);

void start_time(struct chip_data_brl *chip_info, choice_one_hrtimer choice_one_hrtimer) {
	switch (choice_one_hrtimer) {
	case PEN__CHECK_HRTIMER:
		hrtimer_start(&chip_info->check_hrtimer, chip_info->check_time, HRTIMER_MODE_REL);
	break;
	default:
	break;
	}
}

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
		TPD_INFO("GT_brlD:rx_buf kzalloc error\n");
		ret = -ENOMEM;
		return ret;
	}
	tx_buf = kzalloc(SPI_READ_PREFIX_LEN + len, GFP_KERNEL);
	if (!tx_buf) {
		TPD_INFO("GT_brlD:tx_buf kzalloc error\n");
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
		TPD_INFO("GT_brlD:spi transfer error:%d\n", ret);
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
		TPD_INFO("GT_brlD:alloc tx_buf failed, size:%d\n",
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
		TPD_INFO("GT_brlD:spi transfer error:%d\n", ret);
	}

	kfree(tx_buf);
	return ret;
}

int goodix_reg_read(struct chip_data_brl *chip_info, u32 addr,
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
			TPD_INFO("GT_brlD:Mem alloc failed\n");
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

static void goodix_rotate_abcd2cbad(int tx, int rx, s16 *data)
{
	s16 *temp_buf = NULL;
	int size = tx * rx;
	int i;
	int j;
	int col;

	temp_buf = kcalloc(size, sizeof(s16), GFP_KERNEL);
	if (!temp_buf)
		return;

	for (i = 0, j = 0, col = 0; i < size; i++) {
		temp_buf[i] = data[j++ * rx + col];
		if (j == tx) {
			j = 0;
			col++;
		}
	}

	memcpy(data, temp_buf, size * sizeof(s16));
	kfree(temp_buf);
}

/* command ack info */
#define CMD_ACK_IDLE             0x01
#define CMD_ACK_BUSY             0x02
#define CMD_ACK_BUFFER_OVERFLOW  0x03
#define CMD_ACK_CHECKSUM_ERROR   0x04
#define CMD_ACK_OK               0x80

#define GOODIX_CMD_RETRY 6
int brl_send_cmd(struct chip_data_brl *chip_info,
			struct goodix_ts_cmd *cmd)
{
	int ret, retry, i;
	struct goodix_ts_cmd cmd_ack;
	struct goodix_ic_info_misc *misc = &chip_info->ic_info.misc;

	if (!misc->cmd_addr || cmd->len < 4) {
		TPD_INFO("GT:%s: invalied cmd addr or len %d\n", __func__, cmd->len);
		return -1;
	}
	cmd->state = 0;
	cmd->ack = 0;
	goodix_append_checksum(&(cmd->buf[2]), cmd->len - 2,
			       CHECKSUM_MODE_U8_LE);
	TPD_DEBUG("GT_brlD:cmd data %*ph\n", cmd->len, &(cmd->buf[2]));

	retry = 0;
	while (retry++ < GOODIX_CMD_RETRY) {
		ret = goodix_reg_write(chip_info,
				       misc->cmd_addr, cmd->buf, sizeof(*cmd));
		if (ret < 0) {
			TPD_INFO("GT_brlD:%s: failed write command\n", __func__);
			return ret;
		}
		for (i = 0; i < GOODIX_CMD_RETRY; i++) {
			/* check command result */
			ret = goodix_reg_read(chip_info,
					      misc->cmd_addr, cmd_ack.buf, sizeof(cmd_ack));
			if (ret < 0) {
				TPD_INFO("GT_brlD:%s: failed read command ack, %d\n",
					 __func__, ret);
				return ret;
			}
			TPD_DEBUG("GT_brlD:cmd ack data %*ph\n",
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
	TPD_INFO("GT_brlD:%s, failed get valid cmd ack\n", __func__);
	return -1;
}

/* this is for send cmd with one byte cmd_data parameters */
int goodix_send_cmd_simple(struct chip_data_brl *chip_info,
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

	TPD_INFO("GT_brlD:%s, cmd:%u, len:%d, data:%u\n", __func__, cmd_type, cmd.len, cmd_data);

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
			TPD_INFO("GT_brlD:I2C write end_cmd  error!\n");
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
    //TPD_INFO("GT_brlD:ClockWise count = %d\n", count);
    return 1;
}
*/

static void goodix_esd_check_enable(struct chip_data_brl *chip_info, bool enable)
{
	TPD_INFO("GT_brlD:%s %s\n", __func__, enable ? "enable" : "disable");
	/* enable/disable esd check flag */
	chip_info->esd_check_enabled = enable;
}

static int goodix_enter_sleep(struct chip_data_brl *chip_info, bool enable)
{
	u8 sleep_cmd[] = {0x00, 0x00, 0x04, 0x84, 0x88, 0x00};
	u32 cmd_reg = chip_info->ic_info.misc.cmd_addr;

	TPD_INFO("GT_brlD:%s, sleep enable = %d\n", __func__, enable);

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
	int ret = RESULT_ERR;
	TPD_INFO("GT_brlD:%s, gesture enable=%d\n", __func__, enable);

	if (enable) {
		if (chip_info->gesture_type  == 0 &&
			chip_info->gesture_type1 == 0) {
			TPD_INFO("GT_brlD:%s, [WARNING!!]invilid gesture,not enable\n", __func__);
			goto OUT;
		}
		goodix_enter_sleep(chip_info, false);
		tmp_cmd.len = GTP_CMD_GESTURE_LEN;
		tmp_cmd.cmd = GOODIX_GESTURE_CMD_ENABLE;
		tmp_cmd.data[0] = chip_info->gesture_type  & GOODIX_CLEAR_BYTE;
		tmp_cmd.data[1] = chip_info->gesture_type1 & GOODIX_CLEAR_BYTE;
		/* temporary set v ^ < > */
		tmp_cmd.data[0] = chip_info->gesture_type  | GOODIX_FORCE_SET_BIT;
		TPD_INFO("GT_brlD:%s->gesture:type0[%2x]type1[%2x]\n",
			__func__, chip_info->gesture_type, chip_info->gesture_type1);
	} else {
		tmp_cmd.len = GTP_CMD_SPECIAL_LEN;
		tmp_cmd.cmd = GOODIX_GESTURE_CMD_DISABE;
	}

	chip_info->gesture_enable = !!enable;
	ret = brl_send_cmd(chip_info, &tmp_cmd);
OUT:
	return ret;
}

static int goodix_enable_edge_limit(struct chip_data_brl *chip_info, int state)
{
	int ret = -1;

	TPD_INFO("GT_brlD:%s, edge limit enable = %d\n", __func__, state);

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

	TPD_INFO("GT_brlD:%s, charge mode enable = %d\n", __func__, enable);

	if (enable) {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_CHARGER, GTP_MASK_ENABLE);
	} else {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_CHARGER, GTP_MASK_DISABLE);
	}

	return ret;
}

static int goodix_enable_game_mode(struct chip_data_brl *chip_info, bool enable)
{
	int ret = 0;

	TPD_INFO("GT_brlD:%s, game mode enable = %d\n", __func__, enable);
	if (enable) {
		/* disable pen before game mode start when pen enable*/
		goodix_check_bit_set(chip_info, GAME_MODE_ENABLE, true);
		goodix_state_verify(chip_info);
		msleep(10);
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_GAME_MODE, GTP_MASK_ENABLE);
		TPD_INFO("GT_brlD:%s: GTP_CMD_ENTER_GAME_MODE\n", __func__);
	} else {
		ret = goodix_send_cmd_simple(chip_info, GTP_CMD_GAME_MODE, GTP_MASK_DISABLE);
		TPD_INFO("GT_brlD:%s: GTP_CMD_EXIT_GAME_MODE\n", __func__);
		/* enable pen after game mode end when pen enable*/
		msleep(10);
		goodix_check_bit_set(chip_info, GAME_MODE_ENABLE, false);
		goodix_state_verify(chip_info);
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
			TPD_DEBUG("GT_brlD:status check pass\n");
			return 0;
		}

		TPD_INFO("GT_brlD:%s: cmd status not ready, retry %d, ack 0x%x, status 0x%x, ret %d\n", __func__,
			 i, cmd_ack.ack, cmd_ack.state, ret);
		TPD_INFO("GT_brlD:cmd buf %*ph\n", (int)sizeof(cmd_ack), cmd_ack.buf);
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
		TPD_INFO("GT_brlD:%s: failed write cfg prepare cmd %d\n", __func__, ret);
		return ret;
	}
	ret = wait_cmd_status(chip_info, CONFIG_CMD_STATUS_PASS,
			      CONFIG_CMD_WAIT_RETRY);
	if (ret) {
		TPD_INFO("GT_brlD:%s: failed wait for fw ready for config, %d\n",
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
		TPD_INFO("GT_brlD:%s: config len exceed limit %d > %d\n", __func__,
			 len, misc->fw_buffer_max_len);
		return -1;
	}

	tmp_buf = kzalloc(len, GFP_KERNEL);
	if (!tmp_buf) {
		TPD_INFO("GT_brlD:%s: failed alloc malloc\n", __func__);
		return -ENOMEM;
	}

	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_START;
	ret = send_cfg_cmd(chip_info, &cfg_cmd);
	if (ret) {
		TPD_INFO("GT_brlD:%s: failed write cfg prepare cmd %d\n",
			 __func__, ret);
		goto exit;
	}

	TPD_DEBUG("GT_brlD:try send config to 0x%x, len %d\n",
		  misc->fw_buffer_addr, len);
	ret = goodix_reg_write(chip_info,
			       misc->fw_buffer_addr, cfg, len);
	if (ret) {
		TPD_INFO("GT_brlD:%s: failed write config data, %d\n", __func__, ret);
		goto exit;
	}
	ret = goodix_reg_read(chip_info,
			      misc->fw_buffer_addr, tmp_buf, len);
	if (ret) {
		TPD_INFO("GT_brlD:%s: failed read back config data\n", __func__);
		goto exit;
	}

	if (memcmp(cfg, tmp_buf, len)) {
		TPD_INFO("GT_brlD:%s: config data read back compare file\n", __func__);
		ret = -1;
		goto exit;
	}
	/* notify fw for recive config */
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_WRITE;
	ret = send_cfg_cmd(chip_info, &cfg_cmd);
	if (ret) {
		TPD_INFO("GT_brlD:%s: failed send config data ready cmd %d\n", __func__, ret);
	}

exit:
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_EXIT;
	if (send_cfg_cmd(chip_info, &cfg_cmd)) {
		TPD_INFO("GT_brlD:%s: failed send config write end command\n", __func__);
		ret = -1;
	}

	if (!ret) {
		TPD_INFO("GT_brlD:success send config\n");
		msleep(100);
	}

	kfree(tmp_buf);
	return ret;
}

static int goodix_send_temperature(void *chip_data, int temp, bool normal_mode);

#ifndef CONFIG_ARCH_QTI_VM
static int get_now_temp(struct chip_data_brl *chip_info)
{
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->s_client);
	int result = -40000;
	int rc = 0;

#ifdef CONFIG_TOUCHPANEL_TRUSTED_TOUCH
	if (atomic_read(&ts->trusted_touch_enabled) == 1) {
		TPD_INFO("GT_brlD:%s: Trusted touch is already enabled, do not get temp\n", __func__);
		return rc;
	}
#endif

	if (ts->is_suspended) {
		TPD_INFO("GT_brlD:%s :ERR!ts->is_suspended\n", __func__);
		goto OUT_ERR;
	}

	if (ts->temperature_detect_shellback_support) {
		ts->oplus_shell_themal = thermal_zone_get_zone_by_name("shell_back");
		if(IS_ERR(ts->oplus_shell_themal)) {
			TPD_INFO("GT_brlD:%s:ERR!ts->oplus_shell_themal\n", __func__);
			goto OUT_ERR;
		}
		rc = thermal_zone_get_temp(ts->oplus_shell_themal, &result);
		if (rc < 0) {
			TPD_INFO("GT_brlD:%s:ERR!can't get skin temp for shellback, rc=%d\n", __func__, rc);
			goto OUT_ERR;
		}
	} else {
		if (!ts->skin_therm_chan) {
			TPD_INFO("GT_brlD:%s:ERR!ts->skin_therm_chan\n", __func__);
			goto OUT_ERR;
		}
		rc = iio_read_channel_processed(ts->skin_therm_chan, &result);
		if (rc < 0) {
			TPD_INFO("GT_brlD:%s:ERR!can't get skin temp,rc=%d\n", __func__, rc);
			goto OUT_ERR;
		}
	}
	result = result / 1000;
	TPD_INFO("GT_brlD:%s:temp is %d\n", __func__, result);
	goodix_send_temperature(chip_info, result, false);

	return rc;
OUT_ERR:
	return RESULT_ERR;
}
#endif

static int goodix_reset(void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("GT_brlD:%s IN\n", __func__);

	if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
		gpio_direction_output(chip_info->hw_res->reset_gpio, false);
		msleep(15);
		gpio_direction_output(chip_info->hw_res->reset_gpio, true);
		msleep(50);
		chip_info->halt_status = false; /*reset this flag when ic reset*/
	} else {
		TPD_INFO("GT_brlD:reset gpio is invalid\n");
	}

	msleep(100);

	chip_info->pen_state = PEN_UP;

	if (chip_info->gt_temp_enable == true) {
#ifndef CONFIG_ARCH_QTI_VM
		get_now_temp(chip_info);
#endif
	}

	return 0;
}

/*********** End of function that work for oplus_touchpanel_operations callbacks***************/


/********* Start of implementation of oplus_touchpanel_operations callbacks********************/
static int goodix_ftm_process(void *chip_data)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("GT_brlD:%s is called!\n", __func__);
	ret = tp_powercontrol_avdd(chip_info->hw_res, false);
	if (ret) {
		TPD_INFO("GT_brlD:%s, avdd power control faild,ret = %d\n", __func__, ret);
		return -1;
	}
	ret = tp_powercontrol_vddi(chip_info->hw_res, false);
	if (ret) {
		TPD_INFO("GT_brlD:%s, vddi power control faild,ret = %d\n", __func__, ret);
		return -1;
	}
	if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
		gpio_direction_output(chip_info->hw_res->reset_gpio, false);
	}

	return 0;
}
/*
#define TS_DEFAULT_FIRMWARE         "GT9916.bin"
#define TS_DEFAULT_INSPECT_LIMIT    "gt9916_test_limit"
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
#define GOODIX_NORMAL_PID           "9916"
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
			TPD_INFO("GT_brlD:read fw version: %d, retry %d\n", ret, i);
			ret = -1;
			usleep_range(5000, 5100);
			continue;
		}

		if (!checksum_cmp(buf, sizeof(buf), CHECKSUM_MODE_U8_LE)) {
			break;
		}

		TPD_INFO("GT_brlD:invalid fw version: checksum error!\n");
		TPD_INFO("GT_brlD:fw version:%*ph\n", (int)sizeof(buf), buf);
		ret = -EFAULT;
		usleep_range(10000, 10100);
	}
	if (ret) {
		TPD_INFO("GT_brlD:%s: failed get valied fw version, but continue\n", __func__);
		return 0;
	}
	memcpy(version, buf, sizeof(*version));
	memcpy(rom_pid, version->rom_pid, sizeof(version->rom_pid));
	memcpy(patch_pid, version->patch_pid, sizeof(version->patch_pid));
	TPD_INFO("GT_brlD:rom_pid:%s\n", rom_pid);
	TPD_INFO("GT_brlD:rom_vid:%*ph\n", (int)sizeof(version->rom_vid), version->rom_vid);
	TPD_INFO("GT_brlD:pid:%s\n", patch_pid);
	TPD_INFO("GT_brlD:vid:%*ph\n", (int)sizeof(version->patch_vid), version->patch_vid);
	TPD_INFO("GT_brlD:sensor_id:%d\n", version->sensor_id);

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
		TPD_INFO("GT_brlD:%s: invalid scan rate num %d > %d\n", __func__,
			 parm->active_scan_rate_num, MAX_SCAN_RATE_NUM);
		return -1;
	}
	for (i = 0; i < parm->active_scan_rate_num; i++) {
		parm->active_scan_rate[i] = le16_to_cpup((__le16 *)(data + i * 2));
	}

	data += parm->active_scan_rate_num * 2;
	parm->mutual_freq_num = *(data++);
	if (parm->mutual_freq_num > MAX_SCAN_FREQ_NUM) {
		TPD_INFO("GT_brlD:%s: invalid mntual freq num %d > %d\n", __func__,
			 parm->mutual_freq_num, MAX_SCAN_FREQ_NUM);
		return -1;
	}
	for (i = 0; i < parm->mutual_freq_num; i++) {
		parm->mutual_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));
	}

	data += parm->mutual_freq_num * 2;
	parm->self_tx_freq_num = *(data++);
	if (parm->self_tx_freq_num > MAX_SCAN_FREQ_NUM) {
		TPD_INFO("GT_brlD:%s: invalid tx freq num %d > %d\n", __func__,
			 parm->self_tx_freq_num, MAX_SCAN_FREQ_NUM);
		return -1;
	}
	for (i = 0; i < parm->self_tx_freq_num; i++) {
		parm->self_tx_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));
	}

	data += parm->self_tx_freq_num * 2;
	parm->self_rx_freq_num = *(data++);
	if (parm->self_rx_freq_num > MAX_SCAN_FREQ_NUM) {
		TPD_INFO("GT_brlD:%s: invalid rx freq num %d > %d\n", __func__,
			 parm->self_rx_freq_num, MAX_SCAN_FREQ_NUM);
		return -1;
	}

	for (i = 0; i < parm->self_rx_freq_num; i++) {
		parm->self_rx_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));
	}

	data += parm->self_rx_freq_num * 2;
	parm->stylus_freq_num = *(data++);
	if (parm->stylus_freq_num > MAX_FREQ_NUM_STYLUS) {
		TPD_INFO("GT_brlD:%s: invalid stylus freq num %d > %d\n", __func__,
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

	TPD_INFO("GT_brlD:ic_info_length:                %d\n", ic_info->length);
	TPD_INFO("GT_brlD:info_customer_id:              0x%01X\n", version->info_customer_id);
	TPD_INFO("GT_brlD:info_version_id:               0x%01X\n", version->info_version_id);
	TPD_INFO("GT_brlD:ic_die_id:                     0x%01X\n", version->ic_die_id);
	TPD_INFO("GT_brlD:ic_version_id:                 0x%01X\n", version->ic_version_id);
	TPD_INFO("GT_brlD:config_id:                     0x%4X\n", version->config_id);
	TPD_INFO("GT_brlD:config_version:                0x%01X\n", version->config_version);
	TPD_INFO("GT_brlD:frame_data_customer_id:        0x%01X\n", version->frame_data_customer_id);
	TPD_INFO("GT_brlD:frame_data_version_id:         0x%01X\n", version->frame_data_version_id);
	TPD_INFO("GT_brlD:touch_data_customer_id:        0x%01X\n", version->touch_data_customer_id);
	TPD_INFO("GT_brlD:touch_data_version_id:         0x%01X\n", version->touch_data_version_id);

	TPD_INFO("GT_brlD:freqhop_feature:               0x%04X\n", feature->freqhop_feature);
	TPD_INFO("GT_brlD:calibration_feature:           0x%04X\n", feature->calibration_feature);
	TPD_INFO("GT_brlD:gesture_feature:               0x%04X\n", feature->gesture_feature);
	TPD_INFO("GT_brlD:side_touch_feature:            0x%04X\n", feature->side_touch_feature);
	TPD_INFO("GT_brlD:stylus_feature:                0x%04X\n", feature->stylus_feature);

	TPD_INFO("GT_brlD:Drv*Sen,Button,Force num:      %d x %d, %d, %d\n",
		 parm->drv_num, parm->sen_num, parm->button_num, parm->force_num);

	TPD_INFO("GT_brlD:Cmd:                           0x%04X, %d\n",
		 misc->cmd_addr, misc->cmd_max_len);
	TPD_INFO("GT_brlD:Cmd-Reply:                     0x%04X, %d\n",
		 misc->cmd_reply_addr, misc->cmd_reply_len);
	TPD_INFO("GT_brlD:FW-State:                      0x%04X, %d\n",
		 misc->fw_state_addr, misc->fw_state_len);
	TPD_INFO("GT_brlD:FW-Buffer:                     0x%04X, %d\n",
		 misc->fw_buffer_addr, misc->fw_buffer_max_len);
	TPD_INFO("GT_brlD:Touch-Data:                    0x%04X, %d\n",
		 misc->touch_data_addr, misc->touch_data_head_len);
	TPD_INFO("GT_brlD:point_struct_len:              %d\n", misc->point_struct_len);
	TPD_INFO("GT_brlD:mutual_rawdata_addr:           0x%04X\n",
		 misc->mutual_rawdata_addr);
	TPD_INFO("GT_brlD:mutual_diffdata_addr:          0x%04X\n",
		 misc->mutual_diffdata_addr);
	TPD_INFO("GT_brlD:self_rawdata_addr:             0x%04X\n",
		 misc->self_rawdata_addr);
	TPD_INFO("GT_brlD:self_rawdata_addr:             0x%04X\n",
		 misc->self_rawdata_addr);
	TPD_INFO("GT_brlD:stylus_rawdata_addr:           0x%04X, %d\n",
		 misc->stylus_rawdata_addr, misc->stylus_rawdata_len);
	TPD_INFO("GT_brlD:esd_addr:                      0x%04X\n", misc->esd_addr);
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
			TPD_INFO("GT_brlD:failed get ic info length, %d\n", ret);
			usleep_range(5000, 5100);
			continue;
		}
		length = le16_to_cpu(length);
		if (length >= GOODIX_IC_INFO_MAX_LEN) {
			TPD_INFO("GT_brlD:invalid ic info length %d, retry %d\n", length, i);
			continue;
		}

		ret = goodix_reg_read(chip_info,
				      GOODIX_IC_INFO_ADDR, afe_data, length);
		if (ret) {
			TPD_INFO("GT_brlD:failed get ic info data, %d\n", ret);
			usleep_range(5000, 5100);
			continue;
		}
		/* judge whether the data is valid */
		/*if (is_risk_data((const uint8_t *)afe_data, length)) {
			TPD_INFO("GT_brlD:fw info data invalid\n");
			usleep_range(5000, 5100);
			continue;
		}*/
		if (checksum_cmp((const uint8_t *)afe_data,
				 length, CHECKSUM_MODE_U8_LE)) {
			TPD_INFO("GT_brlD:fw info checksum error!\n");
			usleep_range(5000, 5100);
			continue;
		}
		break;
	}
	if (i == GOODIX_GET_IC_INFO_RETRY) {
		TPD_INFO("GT_brlD:%s: failed get ic info\n", __func__);
		/* set ic_info length =0 to indicate this is invalid */
		ic_info->length = 0;
		return -1;
	}

	ret = convert_ic_info(ic_info, afe_data);
	if (ret) {
		TPD_INFO("GT_brlD:%s: convert ic info encounter error\n", __func__);
		ic_info->length = 0;
		return ret;
	}
	print_ic_info(ic_info);
	/* check some key info */
	if (!ic_info->misc.cmd_addr || !ic_info->misc.fw_buffer_addr ||
	    !ic_info->misc.touch_data_addr) {
		TPD_INFO("GT_brlD:%s: cmd_addr fw_buf_addr and touch_data_addr is null\n", __func__);
		ic_info->length = 0;
		return -1;
	}
	TPD_INFO("GT_brlD:success get ic info %d\n", ic_info->length);
	return 0;
}

static int goodix_get_chip_info(void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	int ret;

	/* get ic info */
	ret = brl_get_ic_info(chip_info, &chip_info->ic_info);
	if (ret < 0) {
		TPD_INFO("GT_brlD:faile to get ic info, but continue to probe common!!\n");
		return 0;
	}

	/* get version info */
	ret = brl_read_version(chip_info, &chip_info->ver_info);
	if (ret < 0) {
		TPD_INFO("GT_brlD:failed to get version\n");
	}

	return ret;
}

static int goodix_power_control(void *chip_data, bool enable)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("GT_brlD:%s: enable:%d\n", __func__, enable);

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
		if (gpio_is_valid(chip_info->hw_res->reset_gpio))
			gpio_direction_output(chip_info->hw_res->reset_gpio, false);

		disable_irq(chip_info->irq);

		usleep_range(50000, 51000);
		ret = tp_powercontrol_avdd(chip_info->hw_res, false);
		if (ret)
			return -1;

		usleep_range(10000, 11000);
		ret = tp_powercontrol_vddi(chip_info->hw_res, false);
		if (ret)
			return -1;
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
		TPD_INFO("GT_brlD:%s: goodix abnormal patch id found: %s\n", __func__,
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

	TPD_INFO("GT_brlD:%s: panel_data->tp_fw = 0x%x , fw_ver_num = 0x%x, fw_ver_num >> 24 = 0x%x\n",
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

#define ISP_RAM_ADDR				0x23800
#define HW_REG_CPU_RUN_FROM			0x10000
#define FLASH_CMD_REG				0x12400
#define HW_REG_ISP_BUFFER			0x12410
#define CONFIG_DATA_ADDR			0x3E000

#define HOLD_CPU_REG_W 				0x0002
#define HOLD_CPU_REG_R 				0x2000
#define MISCTL_REG				    0xD804
#define WATCH_DOG_REG				0xD040
#define ENABLE_MISCTL				0x20700000
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
		TPD_INFO("GT_brlD:%s: Invalid firmware size:%zu\n",
			 __func__, firmware->size);
		r = -1;
		goto err_size;
	}
	memcpy(fw_summary, firmware->data, sizeof(*fw_summary));

	/* check firmware size */
	fw_summary->size = le32_to_cpu(fw_summary->size);
	if (firmware->size != fw_summary->size + FW_FILE_CHECKSUM_OFFSET) {
		TPD_INFO("GT_brlD:%s: Bad firmware, size not match, %zu != %d\n",
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
		TPD_INFO("GT_brlD:%s: Bad firmware, cheksum error\n", __func__);
		r = -1;
		goto err_size;
	}

	if (fw_summary->subsys_num > FW_SUBSYS_MAX_NUM) {
		TPD_INFO("GT_brlD:%s: Bad firmware, invalid subsys num: %d\n",
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
			TPD_INFO("GT_brlD:%s: Sybsys offset exceed Firmware size\n",
				 __func__);
			goto err_size;
		}

		fw_summary->subsys[i].data = firmware->data + fw_offset;
		fw_offset += fw_summary->subsys[i].size;
	}

	TPD_INFO("GT_brlD:Firmware package protocol: V%u\n", fw_summary->protocol_ver);
	TPD_INFO("GT_brlD:Fimware PID:GT%s\n", fw_summary->fw_pid);
	TPD_INFO("GT_brlD:Fimware VID:%*ph\n", 4, fw_summary->fw_vid);
	TPD_INFO("GT_brlD:Firmware chip type:%02X\n", fw_summary->chip_type);
	TPD_INFO("GT_brlD:Firmware bus type:%d\n", fw_summary->bus_type);
	TPD_INFO("GT_brlD:Firmware size:%u\n", fw_summary->size);
	TPD_INFO("GT_brlD:Firmware subsystem num:%u\n", fw_summary->subsys_num);

	for (i = 0; i < fw_summary->subsys_num; i++) {
		TPD_DEBUG("GT_brlD:------------------------------------------\n");
		TPD_DEBUG("GT_brlD:Index:%d\n", i);
		TPD_DEBUG("GT_brlD:Subsystem type:%02X\n", fw_summary->subsys[i].type);
		TPD_DEBUG("GT_brlD:Subsystem size:%u\n", fw_summary->subsys[i].size);
		TPD_DEBUG("GT_brlD:Subsystem flash_addr:%08X\n",
			  fw_summary->subsys[i].flash_addr);
		TPD_DEBUG("GT_brlD:Subsystem Ptr:%p\n", fw_summary->subsys[i].data);
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
		TPD_INFO("GT_brlD:%s: Product ID mismatch:%s:%s\n", __func__,
			 fw_version.patch_pid, fw_summary->fw_pid);
		return -1;
	}

	ret = memcmp(fw_version.patch_vid, fw_summary->fw_vid, FW_VID_LEN);
	if (ret) {
		TPD_INFO("GT_brlD:active firmware version:%*ph\n", FW_VID_LEN,
			 fw_version.patch_vid);
		TPD_INFO("GT_brlD:firmware file version: %*ph\n", FW_VID_LEN,
			 fw_summary->fw_vid);
		return -1;
	}
	TPD_INFO("GT_brlD:Firmware version equal\n");

	/* compare config id */
	if (fwu_ctrl->ic_config && fwu_ctrl->ic_config->length > 0) {
		file_cfg_id = goodix_get_file_config_id(fwu_ctrl->ic_config->data);
		ic_cfg_id = fwu_ctrl->chip_info->ic_info.version.config_id;
		if (ic_cfg_id != file_cfg_id) {
			TPD_INFO("GT_brlD:ic_cfg_id:0x%x != file_cfg_id:0x%x\n",
				 ic_cfg_id, file_cfg_id);
			return -1;
		}
		TPD_INFO("GT_brlD:Config_id equal\n");
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

	TPD_INFO("GT_brlD:Loading ISP start\n");
	r = goodix_reg_write_confirm(fwu_ctrl->chip_info, ISP_RAM_ADDR,
				     (u8 *)fw_isp->data, fw_isp->size);
	if (r < 0) {
		TPD_INFO("GT_brlD:%s: Loading ISP error\n", __func__);
		return r;
	}

	TPD_INFO("GT_brlD:Success send ISP data\n");

	/* SET BOOT OPTION TO 0X55 */
	memset(reg_val, 0x55, 8);
	r = goodix_reg_write_confirm(fwu_ctrl->chip_info,
				     HW_REG_CPU_RUN_FROM, reg_val, 8);
	if (r < 0) {
		TPD_INFO("GT_brlD:%s: Failed set REG_CPU_RUN_FROM flag\n", __func__);
		return r;
	}
	TPD_INFO("GT_brlD:Success write [8]0x55 to 0x%x\n", HW_REG_CPU_RUN_FROM);

	if (goodix_fw_update_reset(fwu_ctrl, 100)) {
		TPD_INFO("GT_brlD:%s: reset abnormal\n", __func__);
	}
	/*check isp state */
	if (brl_read_version(fwu_ctrl->chip_info, &isp_fw_version)) {
		TPD_INFO("GT_brlD:%s: failed read isp version\n", __func__);
		return -2;
	}
	if (memcmp(&isp_fw_version.patch_pid[3], "ISP", 3)) {
		TPD_INFO("GT_brlD:%s: patch id error %c%c%c != %s\n", __func__,
			 isp_fw_version.patch_pid[3], isp_fw_version.patch_pid[4],
			 isp_fw_version.patch_pid[5], "ISP");
		return -3;
	}
	TPD_INFO("GT_brlD:ISP running successfully\n");
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
	u32 misc_val = ENABLE_MISCTL;
	u8 reg_val[4] = {0};
	u8 temp_buf[64] = {0};
	int retry = 20;
	int r;

	/*reset IC*/
	TPD_INFO("GT_brlD:firmware update, reset\n");
	if (goodix_fw_update_reset(fwu_ctrl, 5)) {
		TPD_INFO("GT_brlD:%s: reset abnormal\n", __func__);
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
		TPD_INFO("GT_brlD:retry hold cpu %d\n", retry);
		TPD_DEBUG("GT_brlD:data:%*ph\n", 12, temp_buf);
	} while (--retry);
	if (!retry) {
		TPD_INFO("GT_brlD:%s: Failed to hold CPU, return =%d\n", __func__, r);
		return -1;
	}
	TPD_INFO("GT_brlD:Success hold CPU\n");

	/* enable misctl clock */
	r = goodix_reg_write(fwu_ctrl->chip_info, MISCTL_REG, (u8 *)&misc_val, 4);
	TPD_INFO("GT_brlD:enbale misctl clock\n");

	/* disable watch dog */
	reg_val[0] = DISABLE_WATCH_DOG;
	r = goodix_reg_write(fwu_ctrl->chip_info, WATCH_DOG_REG, reg_val, 1);
	TPD_INFO("GT_brlD:disable watch dog\n");

	/* load ISP code and run form isp */
	r = goodix_load_isp(fwu_ctrl);
	if (r < 0) {
		TPD_INFO("GT_brlD:%s: Failed lode and run isp\n", __func__);
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

	TPD_DEBUG("GT_brlD:try send flash cmd:%*ph\n", (int)sizeof(flash_cmd->buf),
		  flash_cmd->buf);
	memset(tmp_cmd.buf, 0, sizeof(tmp_cmd));
	ret = goodix_reg_write(fwu_ctrl->chip_info, FLASH_CMD_REG,
			       flash_cmd->buf, sizeof(flash_cmd->buf));
	if (ret) {
		TPD_INFO("GT_brlD:%s: failed send flash cmd %d\n", __func__, ret);
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
		TPD_INFO("GT_brlD:flash cmd ack error retry %d, ack 0x%x, ret %d\n",
			 i, tmp_cmd.ack, ret);
	}
	if (tmp_cmd.ack != FLASH_CMD_ACK_CHK_PASS) {
		TPD_INFO("GT_brlD:%s: flash cmd ack error, ack 0x%x, ret %d\n",
			 __func__, tmp_cmd.ack, ret);
		TPD_INFO("GT_brlD:%s: data:%*ph\n", __func__,
			 (int)sizeof(tmp_cmd.buf), tmp_cmd.buf);
		return -1;
	}
	TPD_INFO("GT_brlD:flash cmd ack check pass\n");

	msleep(80);
	retry = 20;
	for (i = 0; i < retry; i++) {
		ret = goodix_reg_read(fwu_ctrl->chip_info, FLASH_CMD_REG,
				      tmp_cmd.buf, sizeof(tmp_cmd.buf));
		if (!ret && tmp_cmd.ack == FLASH_CMD_ACK_CHK_PASS &&
		    tmp_cmd.status == FLASH_CMD_W_STATUS_WRITE_OK) {
			TPD_INFO("GT_brlD:flash status check pass\n");
			return 0;
		}

		TPD_INFO("GT_brlD:flash cmd status not ready, retry %d, ack 0x%x, status 0x%x, ret %d\n",
			 i, tmp_cmd.ack, tmp_cmd.status, ret);
		usleep_range(15000, 15100);
	}

	TPD_INFO("GT_brlD:%s: flash cmd status error %d, ack 0x%x, status 0x%x, ret %d\n", __func__,
		 i, tmp_cmd.ack, tmp_cmd.status, ret);
	if (ret) {
		TPD_INFO("GT_brlD:reason: bus or paltform error\n");
		return -1;
	}

	switch (tmp_cmd.status) {
	case FLASH_CMD_W_STATUS_CHK_PASS:
		TPD_INFO("GT_brlD:%s: data check pass, but failed get follow-up results\n", __func__);
		return -EFAULT;
	case FLASH_CMD_W_STATUS_CHK_FAIL:
		TPD_INFO("GT_brlD:%s: data check failed, please retry\n", __func__);
		return -EAGAIN;
	case FLASH_CMD_W_STATUS_ADDR_ERR:
		TPD_INFO("GT_brlD:%s: flash target addr error, please check\n", __func__);
		return -EFAULT;
	case FLASH_CMD_W_STATUS_WRITE_ERR:
		TPD_INFO("GT_brlD:%s: flash data write err, please retry\n", __func__);
		return -EAGAIN;
	default:
		TPD_INFO("GT_brlD:%s: unknown status\n", __func__);
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
			TPD_INFO("GT_brlD:%s: Failed to write firmware packet\n", __func__);
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
			TPD_INFO("GT_brlD:success write package to 0x%x, len %d\n",
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
		TPD_INFO("GT_brlD:%s: Failed alloc memory\n", __func__);
		return -ENOMEM;
	}

	offset = 0;
	while (total_size > 0) {
		data_size = total_size > ISP_MAX_BUFFERSIZE ?
			    ISP_MAX_BUFFERSIZE : total_size;
		TPD_INFO("GT_brlD:Flash firmware to %08x,size:%u bytes\n",
			 subsys_base_addr + offset, data_size);

		memcpy(fw_packet, &subsys->data[offset], data_size);
		/* set checksum for package data */
		goodix_append_checksum(fw_packet,
				       data_size, CHECKSUM_MODE_U16_LE);

		r = goodix_flash_package(fwu_ctrl, subsys->type, fw_packet,
					 subsys_base_addr + offset, data_size + 4);
		if (r) {
			TPD_INFO("GT_brlD:%s: failed flash to %08x,size:%u bytes\n",
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
			TPD_INFO("GT_brlD:%s: failed flash config with ISP, %d\n",
				 __func__, r);
			return r;
		}
		TPD_INFO("GT_brlD:success flash config with ISP\n");
	}

	for (i = 1; i < fw_num && retry;) {
		TPD_INFO("GT_brlD:--- Start to flash subsystem[%d] ---\n", i);
		fw_x = &fw_summary->subsys[i];
		r = goodix_flash_subsystem(fw_ctrl, fw_x);
		if (r == 0) {
			TPD_INFO("GT_brlD:--- End flash subsystem[%d]: OK ---\n", i);
			i++;
		} else if (r == -EAGAIN) {
			retry--;
			TPD_INFO("GT_brlD:%s: --- End flash subsystem%d: Fail, errno:%d, retry:%d ---\n", __func__,
				 i, r, GOODIX_BUS_RETRY_TIMES - retry);
		} else if (r < 0) { /* bus error */
			TPD_INFO("GT_brlD:%s: --- End flash subsystem%d: Fatal error:%d exit ---\n", __func__,
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
		TPD_INFO("GT_brlD:%s: reset abnormal\n", __func__);
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

	TPD_DEBUG("GT_brlD:%s run,sensor id:%d\n", __func__, sid);

	if (sid == GTP_SENSOR_ID_ERR) {
		sid = GTP_SENSOR_ID_DEFAULT;
		TPD_INFO("GT_brlD:%s sensor err set 0,need update to 255\n", __func__);
	}

	cfg_pkg_len = getU32(cfg_bin->data) + BIN_CFG_START_LOCAL;
	if (cfg_pkg_len > cfg_bin->size) {
		TPD_INFO("GT_brlD:%s:Bad firmware!,cfg package len:%d,firmware size:%d\n",
			 __func__, cfg_pkg_len, (int)cfg_bin->size);
		goto exit;
	}

	/* check firmware's checksum */
	cfg_checksum = getU16(&cfg_bin->data[4]);

	for (i = BIN_CFG_START_LOCAL; i < (cfg_pkg_len) ; i++) {
		checksum += cfg_bin->data[i];
	}

	if ((checksum) != cfg_checksum) {
		TPD_INFO("GT_brlD:%s:Bad firmware!(checksum: 0x%04X, header define: 0x%04X)\n",
			 __func__, checksum, cfg_checksum);
		goto exit;
	}
	/* check head end  */

	bin_group_num = cfg_bin->data[MODULE_NUM];
	bin_cfg_num = cfg_bin->data[CFG_NUM];
	TPD_DEBUG("GT_brlD:%s:bin_group_num = %d, bin_cfg_num = %d\n",
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
		TPD_INFO("GT_brlD:%s: invalid config text field\n", __func__);
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
				TPD_DEBUG("GT_brlD:%s:one_cfg_count = %d, cfg_data1 = 0x%02x, cfg_data2 = 0x%02x\n",
					  __func__, one_cfg_count, cfg[0], cfg[1]);
				break;
			}
		}
		cfg_offset += one_cfg_count;
	}

	if (i >= bin_group_num * bin_cfg_num) {
		TPD_INFO("GT_brlD:%s:(not find config ,config_status: %d)\n", __func__, config_status);
		goto exit;
	}

	TPD_DEBUG("GT_brlD:%s exit\n", __func__);
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

	TPD_DEBUG("GT_brlD:%s run\n", __func__);

	cfg_data = kzalloc(GOODIX_CFG_MAX_SIZE, GFP_KERNEL);
	if (cfg_data == NULL) {
		TPD_INFO("GT_brlD:Memory allco err\n");
		goto exit;
	}

	config->length = 0;
	/* parse config data */
	ret = goodix_parse_cfg_data(cfg_bin, config_name, cfg_data,
				    &cfg_len, chip_info->ver_info.sensor_id);
	if (ret < 0) {
		TPD_INFO("GT_brlD:%s: parse %s data failed\n", __func__, config_name);
		ret = -1;
		goto exit;
	}

	TPD_INFO("GT_brlD:%s: %s  version:%d , size:%d\n", __func__,
		 config_name, cfg_data[0], cfg_len);
	memcpy(config->data, cfg_data, cfg_len);
	config->length = cfg_len;

	strncpy(config->name, config_name, MAX_STR_LEN);

exit:
	if (cfg_data) {
		kfree(cfg_data);
		cfg_data = NULL;
	}
	TPD_DEBUG("GT_brlD:%s exit\n", __func__);
	return ret;
}


static int goodix_get_cfg_parms(void *chip_data_info,
				const struct firmware *firmware)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data_info;
	int ret = 0;

	TPD_DEBUG("GT_brlD:%s run\n", __func__);
	if (firmware == NULL) {
		TPD_INFO("GT_brlD:%s: firmware is null\n", __func__);
		ret = -1;
		goto exit;
	}

	if (firmware->data == NULL) {
		TPD_INFO("GT_brlD:%s:Bad firmware!(config firmware data is null: )\n", __func__);
		ret = -1;
		goto exit;
	}

	TPD_INFO("GT_brlD:%s: cfg_bin_size:%d\n", __func__, (int)firmware->size);
	if (firmware->size > 56) {
		TPD_DEBUG("GT_brlD:cfg_bin head info:%*ph\n", 32, firmware->data);
		TPD_DEBUG("GT_brlD:cfg_bin head info:%*ph\n", 24, firmware->data + 32);
	}

	/* parse normal config data */
	ret = goodix_get_cfg_data(chip_info, firmware,
				  GOODIX_NORMAL_CONFIG, &chip_info->normal_cfg);
	if (ret < 0) {
		TPD_INFO("GT_brlD:%s: Failed to parse normal_config data:%d\n", __func__, ret);
	} else {
		TPD_INFO("GT_brlD:%s: parse normal_config data success\n", __func__);
	}

	ret = goodix_get_cfg_data(chip_info, firmware,
				  GOODIX_TEST_CONFIG, &chip_info->test_cfg);
	if (ret < 0) {
		TPD_INFO("GT_brlD:%s: Failed to parse test_config data:%d\n", __func__, ret);
	} else {
		TPD_INFO("GT_brlD:%s: parse test_config data success\n", __func__);
	}

	/* parse normal noise config data */
	ret = goodix_get_cfg_data(chip_info, firmware,
				  GOODIX_NORMAL_NOISE_CONFIG, &chip_info->normal_noise_cfg);
	if (ret < 0) {
		TPD_INFO("GT_brlD:%s: Failed to parse normal_noise_config data\n", __func__);
	} else {
		TPD_INFO("GT_brlD:%s: parse normal_noise_config data success\n", __func__);
	}

	/* parse noise test config data */
	ret = goodix_get_cfg_data(chip_info, firmware,
				  GOODIX_NOISE_TEST_CONFIG, &chip_info->noise_test_cfg);
	if (ret < 0) {
		memcpy(&chip_info->noise_test_cfg, &chip_info->normal_cfg,
		       sizeof(chip_info->noise_test_cfg));
		TPD_INFO("GT_brlD:%s: Failed to parse noise_test_config data,use normal_config data\n", __func__);
	} else {
		TPD_INFO("GT_brlD:%s: parse noise_test_config data success\n", __func__);
	}
exit:
	TPD_DEBUG("GT_brlD:%s exit:%d\n", __func__, ret);
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

	TPD_DEBUG("GT_brlD:%s run\n", __func__);
	if (firmware == NULL) {
		TPD_INFO("GT_brlD:%s: firmware is null\n", __func__);
		ret = -1;
		goto exit;
	}

	if (firmware->data == NULL) {
		TPD_INFO("GT_brlD:%s:Bad firmware!(config firmware data si null)\n", __func__);
		ret = -1;
		goto exit;
	}

	if (fw_firmware == NULL) {
		TPD_INFO("GT_brlD:%s:fw_firmware is null\n", __func__);
		ret = -1;
		goto exit;
	}

	TPD_DEBUG("GT_brlD:clear fw_firmware\n");
	memset(fw_firmware, 0, sizeof(struct firmware));

	cfg_pkg_len = getU32(firmware->data) + BIN_CFG_START_LOCAL;
	TPD_DEBUG("GT_brlD:%s cfg package len:%d\n", __func__, cfg_pkg_len);

	if (firmware->size <= (cfg_pkg_len + 16)) {
		TPD_INFO("GT_brlD:%s current firmware does not contain goodix fw\n", __func__);
		TPD_INFO("GT_brlD:%s cfg package len:%d,firmware size:%d\n", __func__,
			 cfg_pkg_len, (int)firmware->size);
		ret = -1;
		goto exit;
	}

	if (!((firmware->data[cfg_pkg_len + 0] == 'G') &&
			(firmware->data[cfg_pkg_len + 1] == 'X') &&
			(firmware->data[cfg_pkg_len + 2] == 'F') &&
			(firmware->data[cfg_pkg_len + 3] == 'W'))) {
		TPD_INFO("GT_brlD:%s can't find fw package\n", __func__);
		TPD_INFO("GT_brlD:Data type:%c %c %c %c,dest type is:GXFW\n", firmware->data[cfg_pkg_len + 0],
			 firmware->data[cfg_pkg_len + 1], firmware->data[cfg_pkg_len + 2],
			 firmware->data[cfg_pkg_len + 3]);
		ret = -1;
		goto exit;
	}

	if (firmware->data[cfg_pkg_len + 4] != 1) {
		TPD_INFO("GT_brlD:%s can't support this ver:%d\n", __func__,
			 firmware->data[cfg_pkg_len + 4]);
		ret = -1;
		goto exit;
	}

	fw_pkg_len =  getU32(firmware->data + cfg_pkg_len + 8);

	TPD_DEBUG("GT_brlD:%s fw package len:%d\n", __func__, fw_pkg_len);
	if ((fw_pkg_len + 16 + cfg_pkg_len) > firmware->size) {
		TPD_INFO("GT_brlD:%s bad firmware,need len:%d,actual firmware size:%d\n",
			 __func__, fw_pkg_len + 16 + cfg_pkg_len, (int)firmware->size);
		ret = -1;
		goto exit;
	}

	fw_firmware->size = fw_pkg_len;
	fw_firmware->data = firmware->data + cfg_pkg_len + 16;

	TPD_DEBUG("GT_brlD:success get fw,len:%d\n", fw_pkg_len);
	TPD_DEBUG("GT_brlD:fw head info:%*ph\n", 32, fw_firmware->data);
	TPD_DEBUG("GT_brlD:fw tail info:%*ph\n", 4, &fw_firmware->data[fw_pkg_len - 4 - 1]);
	ret = 0;

exit:
	TPD_DEBUG("GT_brlD:%s exit:%d\n", __func__, ret);
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
		TPD_INFO("GT_brlD:Failed to alloc memory for fwu_ctrl\n");
		return -ENOMEM;
	}
	chip_info = (struct chip_data_brl *)chip_data;
	fwu_ctrl->chip_info = chip_info;

	r = goodix_get_cfg_parms(chip_data, cfg_fw_firmware);
	if (r < 0) {
		TPD_INFO("GT_brlD:%s Failed get cfg from firmware\n", __func__);
	} else {
		TPD_INFO("GT_brlD:%s success get ic cfg from firmware\n", __func__);
	}

	r = goodix_get_fw_parms(chip_data, cfg_fw_firmware, &fw_firmware);
	if (r < 0) {
		TPD_INFO("GT_brlD:%s Failed get ic fw from firmware\n", __func__);
		goto err_parse_fw;
	} else {
		TPD_INFO("GT_brlD:%s success get ic fw from firmware\n", __func__);
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
			TPD_INFO("GT_brlD:firmware upgraded\n");
			r = FW_NO_NEED_UPDATE;
			goto err_check_update;
		}
	}

start_update:
	do {
		ret = goodix_update_prepare(fwu_ctrl);
		if (ret) {
			TPD_INFO("GT_brlD:%s: failed prepare ISP, retry %d\n", __func__,
				 FW_UPDATE_RETRY - retry0);
		}
	} while (ret && --retry0 > 0);
	if (ret) {
		TPD_INFO("GT_brlD:%s: Failed to prepare ISP, exit update:%d\n",
			 __func__, ret);
		goto err_fw_prepare;
	}

	/* progress: 20%~100% */
	ret = goodix_flash_firmware(fwu_ctrl);
	if (ret < 0 && --retry1 > 0) {
		TPD_INFO("GT_brlD:%s: Bus error, retry firmware update:%d\n", __func__,
			 FW_UPDATE_RETRY - retry1);
		goto start_update;
	}
	if (ret) {
		TPD_INFO("GT_brlD:flash fw data enter error\n");
	} else {
		TPD_INFO("GT_brlD:flash fw data success, need check version\n");
	}

err_fw_prepare:
	ret = goodix_update_finish(fwu_ctrl);
	if (!ret) {
		TPD_INFO("GT_brlD:Firmware update successfully\n");
		r = FW_UPDATE_SUCCESS;
	} else {
		TPD_INFO("GT_brlD:%s: Firmware update failed\n", __func__);
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

static enum hrtimer_restart goodix_check_hrtimer(struct hrtimer *timer)
{
	struct chip_data_brl *chip_info = container_of(timer, struct chip_data_brl, check_hrtimer);

	if (chip_info->check_id == NO_TOUCH) {
		chip_info->check_start = OFF;
	}

	chip_info->check_hrtimer_over = TIME_CHECK_OVER;
	chip_info->check_palm_flag    = false;
	chip_info->ts->pen_mode_tp_state = DEFAULT;

	TPD_DEBUG("GT_brlD:%s: check_hrtimer_over:%d,check_id:%d,check_start:%d, palm:%d\n", \
		__func__,
		chip_info->check_hrtimer_over, chip_info->check_id, chip_info->check_start, chip_info->check_palm_flag);

	return HRTIMER_NORESTART;
}

static void goodix_read_differ(struct chip_data_brl * chip_info)
{
	int tx_num, rx_num;
	int ret;
	int i, j;
	u8 *rw_buf;
	s16 *diff_buf;
	u32 mutual_diffdata_addr;
	u32 self_diffdata_addr;

	tx_num = chip_info->hw_res->tx_num;
	rx_num = chip_info->hw_res->rx_num;
	mutual_diffdata_addr = chip_info->ic_info.misc.mutual_diffdata_addr;
	self_diffdata_addr = chip_info->ic_info.misc.self_diffdata_addr;
	rw_buf = chip_info->diff_rw_buf;
	diff_buf = chip_info->diff_buf;

	if (rw_buf == NULL || diff_buf == NULL) {
		TPD_INFO("%s:diff buf is NULL", __func__);
		return;
	}

	/*mutual diff data*/
	ret = goodix_reg_read(chip_info, mutual_diffdata_addr, rw_buf, tx_num*rx_num*2);
	if (ret < 0) {
		TPD_INFO("%s: read mutual diff data error", __func__);
		return;
	}

	for (i = 0; i < rx_num; ++i) {
		for (j = 0; j < tx_num; ++j) {
			diff_buf[i*tx_num+j] = rw_buf[j*rx_num*2 + i*2] + (rw_buf[j*rx_num*2 + i*2 + 1] << 8);
		}
	}

	/*self diff data*/
	rw_buf += tx_num*rx_num*2;
	diff_buf += tx_num*rx_num;
	ret = goodix_reg_read(chip_info, self_diffdata_addr, rw_buf, (tx_num+rx_num)*2);
	if (ret < 0) {
		TPD_INFO("%s: read self diff data error", __func__);
		return;
	}

	for (i = 0; i < (chip_info->diff_size - tx_num*rx_num); ++i) {
		diff_buf[i] = rw_buf[i*2] + (rw_buf[i*2+1] << 8);
	}
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
		TPD_DEBUG("GT_brlD:%s: invalied touch data address\n", __func__);
		return IRQ_IGNORE;
	}

	/* touch head + 2 fingers + checksum */
	pre_read_len = MAX_GT_IRQ_DATA_LENGTH;

	ret = goodix_reg_read(chip_info, misc->touch_data_addr,
			      chip_info->touch_data, pre_read_len);
	if (ret < 0) {
		TPD_DEBUG("GT_brlD:%s: spi transfer error!\n", __func__);
		return IRQ_IGNORE;
	}

	/*tp data record*/
	if (chip_info->enable_differ_mode) {
		goodix_read_differ(chip_info);
	}

	TPD_DEBUG("GT_brlD:%s:addr[0x%2x] flag[%*ph]:data[%*ph]\n", __func__,
		misc->touch_data_addr,
		IRQ_EVENT_HEAD_LEN, chip_info->touch_data,
		IRQ_EVENT_HEAD_LEN * IRQ_EVENT_HEAD_LEN, chip_info->touch_data + IRQ_EVENT_HEAD_LEN);

	if (chip_info->touch_data[0] == 0x00) {
		TPD_DEBUG("GT_brlD:invalid touch head");
		return IRQ_IGNORE;
	}

	if (checksum_cmp(chip_info->touch_data,
			 IRQ_EVENT_HEAD_LEN, CHECKSUM_MODE_U8_LE)) {
		TPD_DEBUG("GT_brlD:%s: [checksum err !!]touch_head %*ph\n", __func__, IRQ_EVENT_HEAD_LEN,
			chip_info->touch_data);
		goto exit;
	}

	event_status = chip_info->touch_data[IRQ_EVENT_TYPE_OFFSET];
	if (event_status & GOODIX_TOUCH_EVENT) {
		touch_num = chip_info->touch_data[POINT_NUM_OFFSET] & 0x0F;
		if (touch_num > MAX_POINT_NUM) {
			TPD_DEBUG("GT_brlD:invalid touch num %d\n", touch_num);
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
					TPD_INFO("GT_brlD:%s:spi transfer error!\n", __func__);
					goto exit;
				}
				TPD_DEBUG("GT_brlD:grip bit->[0x%x]->girp[%*ph]\n",
					event_status & GRIP_COOR_SUPPORT_FLAG, 16, chip_info->edge_data);
				if (checksum_cmp(chip_info->edge_data,
						EDGE_INPUT_OFFSET * touch_num + 2,
						CHECKSUM_MODE_U8_LE) && touch_num > TOUCH_UP) {
					chip_info->abnormal_grip_coor = true;
						TPD_INFO("GT_brlD:%s: [checksum err!!] girp:[%*ph]\n", __func__,
							16, chip_info->edge_data);
				}
			} else {
				chip_info->get_grip_coor = false;
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
			TPD_DEBUG("GT_brlD:touch data checksum error\n");
			goto exit;
		}
	} else if (!(event_status & (GOODIX_GESTURE_EVENT | GOODIX_FINGER_IDLE_EVENT))) {
		/*TODO check this event*/
	}

	if ((chip_info->touch_data[TOUCH_DATA_OFFECT_2] &
			GTP_PLAM_ADDR_FLAG) && (chip_info->probe_complete == true)) {
		if (chip_info->ts->palm_to_sleep_enable) {
			TPD_DEBUG("GT_brlD:%s:detect palm,now to sleep\n", __func__);
			result_event = IRQ_PALM;
			goto exit;
		}
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
		SET_BIT(result_event, IRQ_PEN_REPORT);
	}
	if (event_status & GOODIX_FINGER_STATUS_EVENT) {
		SET_BIT(result_event, IRQ_FW_HEALTH);
	}

	if (event_status & GOODIX_TOUCH_EVENT) {
		if ((((chip_info->pen_state == PEN_DOWN) && (touch_num == 0)) || point_type == POINT_TYPE_STYLUS ||
				point_type == POINT_TYPE_STYLUS_HOVER) && chip_info->pen_enable == true) {
			SET_BIT(result_event, IRQ_PEN);
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

/* Xa=O-K*n; n:0~370 (O:992,k:3) */

#define LEFT_MIN_X_AXIS    12000
#define LEFT_MAX_X_AXIS    5000
static void check_dynamic_limit(struct chip_data_brl *chip_info,
		struct point_info *points, s32 id)
{
	int now_x_pos = points[id].x;
	int dyna_pos = chip_info->motor_get_coord;
	int vir_ori = chip_info->virtual_origin;
	int new_x_axis = 0;
	int k = chip_info->dynamic_k_value;
	int prevent = chip_info->motor_prevent;
	int offect = chip_info->motor_offect;

	switch (chip_info->motor_runing) {
	case true:
		new_x_axis = vir_ori - (k * dyna_pos);
		TPD_DEBUG("GT_brlD:[ori:%d,dyna:%d,k:%d]-->new:[Xposi:%d %s Xaxis:%d]\n",
			vir_ori, dyna_pos, k, now_x_pos, (now_x_pos > (new_x_axis + offect)) ? ">" : "<=", new_x_axis);
		if (now_x_pos <= new_x_axis + offect) {
			TPD_DEBUG("GT_brlD: ----->!!!include limit, not to touch!!!<-----\n");
			points[id].tx_press = 0;
			points[id].rx_press = prevent;
			points[id].tx_er =    0;
			points[id].rx_er =    prevent;
		}
		chip_info->runing_x_coord = new_x_axis;
		chip_info->runing_x_offect = offect;
	break;
	case false:
	break;
	default: break;
	}
}

void goodix_clear_status(struct chip_data_brl *chip_info)
{
	chip_info->ts->pen_mode_tp_state = CANCLE_TP;
	tp_touch_btnkey_release(chip_info->tp_index);
}

static int goodix_check_finger(struct chip_data_brl *chip_info,
			struct point_info *points, int touch_status, int id, int touch_num)
{
	int ret = 0;

	switch (touch_status) {
	case TOUCH_DOWN:
		if (chip_info->check_start == OFF)
			goto OUT;

		if (chip_info->check_hrtimer_over == TIME_CHECK_START)
			chip_info->check_id = BIT_SET(id) | chip_info->check_id;
		else
			if (BIT_CHK(chip_info->check_id, touch_num)) {
				chip_info->check_start = OFF;
				chip_info->check_id    = TOUCH_UP;
				ret = -1;
			}
		if (BIT_CHK(chip_info->check_id, id)) {
			points[id].status = TOUCH_UP;
			ret = -1;
		}
	break;
	case TOUCH_UP:
		if((chip_info->touch_data[TOUCH_DATA_OFFECT_2] &
			GTP_PLAM_ADDR_FLAG)) {
			chip_info->check_palm_flag = true;
			chip_info->ts->pen_mode_tp_state = CANCLE_TP;
			start_time(chip_info, PEN__CHECK_HRTIMER);
		}

		if (chip_info->check_start == OFF)
			goto OUT;

		if (chip_info->check_hrtimer_over == TIME_CHECK_OVER) {
			chip_info->check_start = OFF;
			chip_info->check_id    = TOUCH_UP;
			ret = -1;
		}
	break;
	case PEN_DOWN:
		if (chip_info->check_start == OFF)
			goto OUT;
		chip_info->check_start = OFF;
		chip_info->check_id    = TOUCH_UP;
	default:
	break;
	}

OUT:
	TPD_DEBUG("GT_brlD:%s:%s->id[%d]check_id[%d]timer_over[%d]status[%d]touch_num[%d]ret[%d]palm[%d]",
		__func__,
		touch_status ? "TOUCH_DOWN" : "TOUCH_UP",
		id,
		chip_info->check_id,
		chip_info->check_hrtimer_over,
		points[id].status,
		touch_num,
		ret,
		chip_info->check_palm_flag);

	return ret;
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
	int ret = 0;

	touch_num = chip_info->touch_data[POINT_NUM_OFFSET] & 0x0F;

	TPD_DEBUG("GT_brlD:%s:check %d points data:[%*ph]\n", __func__,
		touch_num,
		TOUCH_POINT_OFFECT * touch_num + TOUCH_CHECKSUM_OFFECT,
		chip_info->touch_data + TOUCH_ALL_OFFECT);

	if (touch_num > MAX_POINT_NUM) {
		TPD_INFO("GT_brlD:%s:ERR!! touch_num OVER LIMIT->%d\n", __func__, touch_num);
		goto END_TOUCH;
	}

	if (touch_num == ALL_TOUCH_UP || chip_info->abnormal_grip_coor == true) { /*Up event*/
		chip_info->touch_state = TOUCH_UP;
		TPD_DEBUG("GT_brlD:UP abnormal_grip_coor:%d,pen_support:%d\n",
			chip_info->abnormal_grip_coor, chip_info->pen_support);
		if (chip_info->pen_enable)
			ret = goodix_check_finger(chip_info, points, TOUCH_UP, ALL_TOUCH_UP, touch_num);
		goto END_TOUCH;
	}

	if (chip_info->pen_enable && chip_info->check_palm_flag == true) {
		TPD_DEBUG("GT_brlD:%s:UP from palm mode\n", __func__);
		chip_info->touch_state = TOUCH_UP;
		chip_info->check_palm_flag = false;
		goto END_TOUCH;
	}

	chip_info->touch_state = TOUCH_DOWN;
	coor_data = &chip_info->touch_data[IRQ_EVENT_HEAD_LEN];
	ew_data   = &chip_info->edge_data[0];

	for (true_num = 0; true_num < touch_num; true_num++) {
		id = (coor_data[0] >> 4) & FINGER_CHECK;
		if (id >= MAX_POINT_NUM) {
			TPD_INFO("GT_brlD:%s ERR!!->invalidID:%d,abnormalNUM:%d\n",
				__func__, id, true_num);
			continue;
		}

		if (chip_info->pen_enable) {
			ret = goodix_check_finger(chip_info, points, TOUCH_DOWN, id, touch_num);
			if (ret < 0) {
				touch_map = 0;
				goto END_TOUCH;
			}
		}

		/* normal data */
		points[id].x      = le16_to_cpup((__le16 *)(coor_data + 2));
		points[id].y      = le16_to_cpup((__le16 *)(coor_data + 4));
		points[id].touch_major    = max(coor_data[6], coor_data[7]);
		points[id].width_major    = min(coor_data[6], coor_data[7]);
		/* edge data */
		points[id].tx_press = DATA_CHANGE(ew_data[0]); /* 0x1038A */
		points[id].rx_press = DATA_CHANGE(ew_data[1]); /* 0x1038B */
		points[id].tx_er    = DATA_CHANGE(ew_data[3]); /* 0x1038C */
		points[id].rx_er    = DATA_CHANGE(ew_data[2]); /* 0x1038D */
		/* touch status */
		points[id].status = TOUCH_DOWN;

		if (chip_info->motor_coord_support)
			check_dynamic_limit(chip_info, points, id);

		TPD_DEBUG("GT_brlD:[TP]:DOWN:%d:%s:(%6d,%6d).tp[%2d].rp[%2d].te[%2d].re[%2d]\n",
			id,
			points[id].x > (chip_info->max_x / 2) ? "right" : "left",
			points[id].x, points[id].y,
			points[id].tx_press, points[id].rx_press,
			points[id].tx_er, points[id].rx_er);

		ew_data   += BYTES_PER_EDGE;
		coor_data += BYTES_PER_POINT;
		touch_map |= 0x01 << id;
	}

END_TOUCH:
	if (chip_info->enable_differ_mode) {
		TPD_INFO("GT:%s:print differ data\n", __func__);
		goodix_print_differ(chip_info->diff_buf, chip_info->diff_size, chip_info->hw_res->tx_num, chip_info->hw_res->rx_num);
	}
	return touch_map;
}

static void goodix_gesture_coordiate(struct chip_data_brl *chip_info, struct gesture_info *gesture)
{
	u8 *coor_data = NULL;

	coor_data = &chip_info->touch_data[IRQ_EVENT_HEAD_LEN];
	gesture->Point_start.x = le16_to_cpup((__le16 *)(coor_data));
	gesture->Point_start.y = le16_to_cpup((__le16 *)(coor_data + GESTURE_DATA_ADDR_OFFECT));

	TPD_INFO("GT_brlD:%s:(X,Y)=>kernel[%5d,%5d] user[%4d,%4d]",
		__func__,
		gesture->Point_start.x,
		gesture->Point_start.y,
		gesture->Point_start.x / chip_info->resolution_ratio,
		gesture->Point_start.y / chip_info->resolution_ratio);
}

static int goodix_get_gesture_info(void *chip_data, struct gesture_info *gesture)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	u8 gesture_type;

	TPD_DEBUG("GT_brlD:%s:check gesture data = [%*ph]\n", __func__,
		IRQ_EVENT_HEAD_LEN, chip_info->touch_data);

	gesture_type = chip_info->touch_data[4];
	TPD_INFO("GT_brlD:%s: get gesture type:0x%x\n", __func__, gesture_type);

	switch (gesture_type) {
	case GOODIX_LEFT2RIGHT_SWIP:
		gesture->gesture_type = LEFT2RIGHT_SWIP;
	break;
	case GOODIX_RIGHT2LEFT_SWIP:
		gesture->gesture_type = RIGHT2LEFT_SWIP;
	break;
	case GOODIX_UP2DOWN_SWIP:
		gesture->gesture_type = DOU_SWIP;
	break;
	case GOODIX_DOWN2UP_SWIP:
		gesture->gesture_type = DOU_SWIP;
	break;
	/*
	case GOODIX_DOU_SWIP:
		gesture->gesture_type = DOU_SWIP;
	break; */
	case GOODIX_DOU_TAP:
		gesture->gesture_type = DOU_TAP;
	break;
	case GOODIX_SINGLE_TAP:
		gesture->gesture_type = SINGLE_TAP;
	break;
	case GOODIX_PENDETECT:
		gesture->gesture_type = PENDETECT;
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
		TPD_INFO("GT_brlD:%s: unknown gesture type[0x%x]\n", __func__, gesture_type);
	break;
	}

	goodix_gesture_coordiate(chip_info, gesture);

	return 0;
}

static void goodix_get_health_info(void *chip_data, struct monitor_data *mon_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_health_info_v2 *health_info;
	struct goodix_health_info_v2 *health_local = &chip_info->health_info;
	struct goodix_health_info_v2 health_data;
	int ret = 0;
	u8 clear_flag = 0;
	u32 health_info_addr = 0x10328;
	uint16_t freq_before, freq_after = 0;
	uint32_t hsync_freq_before, hsync_freq_after = 0;
	char *freq_str = NULL;

	/* ret = touch_i2c_read_block(chip_info->client, 0x10328, sizeof(struct goodix_health_info_v2), (unsigned char *)&health_data); */
	ret = goodix_reg_read(chip_info, health_info_addr, (unsigned char *)&health_data, sizeof(struct goodix_health_info_v2));

	if (ret < 0) {
		TPD_INFO("GT_brlD:%s: read debug log data i2c faild\n", __func__);
		return;
	}

	TPD_DEBUG("GT_brlD:GTP_REG_DEBUG:%*ph\n", (int)sizeof(struct goodix_health_info_v2), &health_data);

	health_info = &health_data;

	if (health_info->shield_water) {
		if (health_info->shield_water_state) {
			tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_SHIELD_WATER);
			TPD_DETAIL("%s: enter water mode\n", __func__);
		} else {
			TPD_DETAIL("%s: exit water mode\n", __func__);
		}
	}

	if (health_info->baseline_refresh) {
		switch (health_info->baseline_refresh_type) {
		case BASE_DC_COMPONENT:
			tp_healthinfo_report(mon_data, HEALTH_REPORT, "BASE_DC_COMPONENT");
			break;

		case BASE_SYS_UPDATE:
			tp_healthinfo_report(mon_data, HEALTH_REPORT, "BASE_SYS_UPDATE");
			break;

		case BASE_NEGATIVE_FINGER:
			tp_healthinfo_report(mon_data, HEALTH_REPORT, "base_negative_finger");
			break;

		case BASE_MONITOR_UPDATE:
			tp_healthinfo_report(mon_data, HEALTH_REPORT, "BASE_MONITOR_UPDATE");
			break;

		case BASE_CONSISTENCE:
			tp_healthinfo_report(mon_data, HEALTH_REPORT, "BASE_CONSISTENCE");
			break;

		case BASE_FORCE_UPDATE:
			tp_healthinfo_report(mon_data, HEALTH_REPORT, "BASE_FORCE_UPDATE");
			break;

		default:
			break;
		}

		TPD_DETAIL("%s: baseline refresh type: %d \n", __func__,
			   health_info->baseline_refresh_type);
	}

	if (health_info->shield_freq != 0) {
		if (health_info->charger_mode) {
			tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_NOISE_CHARGE);
		} else {
			tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_NOISE);
		}

		freq_before = health_info->freq_before_low + (health_info->freq_before_high << 8);
		freq_after = health_info->freq_after_low + (health_info->freq_after_high << 8);

		TPD_DETAIL("%s: freq before: %hu HZ, freq after: %hu HZ; noise level before: %d, noise level after: %d/n", __func__,
			   freq_before, freq_after, health_info->noise_level_before, health_info->noise_level_after);

		freq_str = kzalloc(25, GFP_KERNEL);
		if (!freq_str) {
			TPD_INFO("GT_brlD:freq_str kzalloc failed.\n");
		} else {
			if (health_info->charger_mode) {
				snprintf(freq_str, 25, "noise_%d_freq_%hu_c", health_info->noise_level_after, freq_after);
			} else {
				snprintf(freq_str, 25, "noise_%d_freq_%hu", health_info->noise_level_after, freq_after);
			}
			tp_healthinfo_report(mon_data, HEALTH_REPORT, freq_str);
			kfree(freq_str);
		}
	}

	goodix_get_pen_health_info(chip_info, mon_data);

	if (health_info->esd_rst != 0) {
		switch (health_info->esd_reason) {
		case ESD_RAM_ERROR:
			tp_healthinfo_report(mon_data, HEALTH_REPORT, "ESD_RAM_ERROR");
			break;

		case ESD_RAWDATA_ERROR:
			tp_healthinfo_report(mon_data, HEALTH_REPORT, "ESD_RAWDATA_ERROR");
			break;

		default:
			break;
		}

		TPD_DETAIL("%s: fw reset type : %d\n", __func__, health_info->esd_reason);
	}

	if (health_info->shield_palm != 0
		   && health_info->shield_palm != health_local->shield_palm) {
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_SHIELD_PALM);
		TPD_DETAIL("%s: enter palm mode\n", __func__);
	}

	if (health_info->charger_mode != 0
		   && health_info->charger_mode != health_local->charger_mode) {
		TPD_DETAIL("%s: enter charger mode\n", __func__);
	}

	if (health_info->low_temperature != 0
		   && health_info->low_temperature != health_local->low_temperature) {
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_TEMP_DRIFT);
		TPD_DETAIL("%s: enter low temperature\n", __func__);
	}

	if (health_info->broken_compensated != 0
		   && health_info->broken_compensated != health_local->broken_compensated) {
		tp_healthinfo_report(mon_data, HEALTH_REPORT, HEALTH_REPORT_CHANEL_FILL);
		TPD_DETAIL("%s: broken compensated occurred\n", __func__);
	}

	if (health_info->sync_error != 0) {
		switch (health_info->hsync_error) {
		case HSYNC_NO_INPUT:
			tp_healthinfo_report(mon_data, HEALTH_REPORT, "HSYNC_NO_INPUT");
			break;

		case HSYNC_FREQ_HOPPING:
			tp_healthinfo_report(mon_data, HEALTH_REPORT, "HSYNC_FREQ_HOPPING");
			break;

		default:
			break;
		}

		hsync_freq_before = health_info->hsync_freq_before_byte0 +
			   (health_info->hsync_freq_before_byte1 << 8) +
			   (health_info->hsync_freq_before_byte2 << 16) +
			   (health_info->hsync_freq_before_byte3 << 24);
		hsync_freq_after = health_info->hsync_freq_after_byte0 +
			   (health_info->hsync_freq_after_byte1 << 8) +
			   (health_info->hsync_freq_after_byte2 << 16) +
			   (health_info->hsync_freq_after_byte3 << 24);
		TPD_DETAIL("%s: Hsync error : %d, freq before: %d, freq after: %d\n",
			   __func__, health_info->esd_reason, hsync_freq_before, hsync_freq_after);
	}

	memcpy(health_local, health_info, sizeof(struct goodix_health_info_v2));

	/* ret = touch_i2c_write_block(chip_info->client,
				    chip_info->reg_info.GTP_REG_DEBUG, 1, &clear_flag); */
	ret = goodix_reg_write(chip_info, health_info_addr, &clear_flag, 1);

	if (ret < 0) {
		TPD_INFO("GT_brlD:%s: clear debug log data i2c faild\n", __func__);
	}

	return;
}
/*
static int goodix_read_coord_position(void *chip_data, char *page, size_t str)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("GT_brlD:%s: start to goodix_read_coord_position, str:%d\n", __func__, str);

	snprintf(page, str-1, "Xnow:%d,Xoffect:%d,Xk:%d,Mnow:%d,Moffect:%d,Mtotal:%d-%d~%d-%d\n",
			chip_info->runing_x_coord,
			chip_info->runing_x_offect,
			chip_info->dynamic_k_value,
			chip_info->motor_get_coord,
			chip_info->motor_max_max - chip_info->motor_max_min,
			chip_info->motor_min_min, chip_info->motor_min_max,
			chip_info->motor_max_min, chip_info->motor_max_max);

	return 0;
}
*/
static void set_new_offect(struct chip_data_brl *chip_info, int new_offect)
{
	chip_info->motor_max_min = chip_info->motor_max_max + new_offect;
	chip_info->motor_min_min = chip_info->motor_min_max + new_offect;
	TPD_INFO("GT_brlD:%s:new motor para:[%d,%d]~[%d,%d][offect:%d]\n", __func__,
		chip_info->motor_min_min, chip_info->motor_min_max,
		chip_info->motor_max_min, chip_info->motor_max_max,
		new_offect);
}

static void goodix_check_reg(struct chip_data_brl *chip_info, int addr, int lens)
{
	u8 buf[32] = {0};
	goodix_reg_read(chip_info, addr, buf, lens);
	TPD_INFO("GT_brlD:%s->[%*ph]\n", __func__, lens, buf);
	return;
}

static int goodix_set_coord_position(void *chip_data, int position)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	int ret = 0;


	if (position < 0) {
		set_new_offect(chip_info, position);
	}

	chip_info->motor_get_coord = position;

	if (position >= chip_info->motor_min_min &&
		position <= chip_info->motor_min_max &&
		chip_info->motor_max_limit == true) {
		ret = goodix_send_cmd_simple(chip_info, GTP_MOTOR_POSITON_MASK, GTP_MASK_DISABLE);
		goodix_check_reg(chip_info, GOODIX_CMD_REG, 8);
		ret = goodix_send_cmd_simple(chip_info, GTP_PLAM_MASK, GTP_MASK_ENABLE);
		goodix_check_reg(chip_info, GOODIX_CMD_REG, 8);
		chip_info->motor_max_limit = false;
		chip_info->motor_runing = false;

		TPD_INFO("GT_brlD:%s: MIN position:%d\n", __func__, chip_info->motor_get_coord);
	} else if (position > chip_info->motor_min_max &&
		position <= chip_info->motor_max_min) {
		chip_info->motor_runing = true;
		if (chip_info->motor_max_limit == false) {
			ret = goodix_send_cmd_simple(chip_info, GTP_MOTOR_POSITON_MASK, GTP_MASK_ENABLE);
			goodix_check_reg(chip_info, GOODIX_CMD_REG, 8);
			ret = goodix_send_cmd_simple(chip_info, GTP_PLAM_MASK, GTP_MASK_DISABLE);
			goodix_check_reg(chip_info, GOODIX_CMD_REG, 8);
			chip_info->motor_max_limit = true;
		TPD_INFO("GT_brlD:%s: RUNING position:%d\n", __func__, chip_info->motor_get_coord);
		}
	} else if (position > chip_info->motor_max_min &&
		position <= chip_info->motor_max_max &&
		chip_info->motor_runing == true) {
		chip_info->motor_runing = false;
		ret = goodix_send_cmd_simple(chip_info, GTP_PLAM_MASK, GTP_MASK_ENABLE);
		goodix_check_reg(chip_info, GOODIX_CMD_REG, 8);
		TPD_INFO("GT_brlD:%s: MAX position:%d\n", __func__, chip_info->motor_get_coord);
	}

	return ret;
}

static int goodix_mode_switch(void *chip_data, work_mode mode, int flag)
{
	int ret = -1;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	if (!chip_info->ic_info.length) {
		if ((mode == MODE_NORMAL) && (flag == true)) {
			TPD_INFO("GT_brlD:%s: goodix ic info invalid, but probe, continue\n", __func__);
			return 0;
		}
		TPD_INFO("GT_brlD:%s: goodix ic info invalid\n", __func__);
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
			TPD_INFO("GT_brlD:%s: goodix enter sleep failed\n", __func__);
		}
		break;

	case MODE_GESTURE:
		ret = goodix_enable_gesture(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT_brlD:%s: goodix enable:(%d) gesture failed.\n", __func__, flag);
			return ret;
		}
		break;

	case MODE_EDGE:
		ret = goodix_enable_edge_limit(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT_brlD:%s: goodix enable:(%d) edge limit failed.\n", __func__, flag);
			return ret;
		}
		break;

	case MODE_CHARGE:
		ret = goodix_enable_charge_mode(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT_brlD:%s: enable charge mode : %d failed\n", __func__, flag);
		}
		break;

	case MODE_GAME:
		ret = goodix_enable_game_mode(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT_brlD:%s: enable game mode : %d failed\n", __func__, flag);
		}
		break;

	case MODE_PEN_SCAN:
		ret = goodix_enable_pen_mode(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT_brlD:%s: enable pen mode : %d failed\n", __func__, flag);
		}
		if (chip_info->pen_state == PEN_UP)
			chip_info->pen_enable = !!flag;
		break;
	case MODE_PEN_CTL:
		ret = goodix_pen_control(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("GT_brlD:%s: enable pen control : %d failed\n", __func__, flag);
		}
		break;
	default:
		TPD_INFO("GT_brlD:%s: mode %d not support.\n", __func__, mode);
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
		TPD_DEBUG("GT_brlD:%s: close\n", __func__);
		return 0;
	}

	ret = goodix_reg_read(chip_info, misc->esd_addr, &esd_buf, 1);
	if ((ret < 0) || esd_buf == 0xAA) {
		TPD_INFO("GT_brlD:%s: esd dynamic esd occur, ret = %d, esd_buf = %d.\n",
			 __func__, ret, esd_buf);
		TPD_INFO("GT_brlD:%s: IC works abnormally! Process esd reset.\n", __func__);
		disable_irq_nosync(chip_info->irq);

		goodix_power_control(chip_info, false);
		msleep(30);
		goodix_power_control(chip_info, true);
		usleep_range(10000, 10100);

		goodix_reset(chip_data);

		tp_touch_btnkey_release(chip_info->tp_index);

		enable_irq(chip_info->irq);
		TPD_INFO("GT_brlD:%s: Goodix esd reset over.\n", __func__);
		chip_info->esd_err_count++;
		return -1;
	} else {
		esd_buf = 0xAA;
		ret = goodix_reg_write(chip_info,
				       chip_info->ic_info.misc.esd_addr, &esd_buf, 1);
		if (ret < 0) {
			TPD_INFO("GT_brlD:%s: Failed to reset esd reg.\n", __func__);
		}
	}

	return 0;
}

static void goodix_enable_fingerprint_underscreen(void *chip_data, uint32_t enable)
{
	int ret = 0;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_INFO("GT_brlD:%s, enable = %d\n", __func__, enable);
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
	TPD_INFO("GT_brlD:%s: abandon gesture_type:0x%08X\n",
		__func__, chip_info->gesture_type);
}

static void goodix_set_gesture_state(void *chip_data, int state)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	TPD_DEBUG("GT_brlD:%s:start change state[0x%x] support type[0x%x]\n",
		__func__, state, chip_info->support_gesture_type);

	state = state & chip_info->support_gesture_type;

	TPD_DEBUG("GT_brlD:%s:  end change state[0x%x] gesture type[0x%x]\n",
		__func__, state, chip_info->gesture_type);

	SET_GESTURE_BIT(state, DOU_TAP,        chip_info->gesture_type,  GTP_DATA0_GESTURE_DOU_TAP);
	/* all directions v , need change later*/
	SET_GESTURE_BIT(state, UP_VEE,         chip_info->gesture_type,  GTP_DATA0_GESTURE_VEE);
	/*
	SET_GESTURE_BIT(state, DOWN_VEE,       chip_info->gesture_type,  GTP_DATA0_GESTURE_VEE);
	SET_GESTURE_BIT(state, LEFT_VEE,       chip_info->gesture_type,  GTP_DATA0_GESTURE_VEE);
	SET_GESTURE_BIT(state, RIGHT_VEE,      chip_info->gesture_type,  GTP_DATA0_GESTURE_VEE); */
	SET_GESTURE_BIT(state, CIRCLE_GESTURE, chip_info->gesture_type,  GTP_DATA0_GESTURE_CIRCLE);
	SET_GESTURE_BIT(state, M_GESTRUE,      chip_info->gesture_type,  GTP_DATA0_GESTURE_M);
	SET_GESTURE_BIT(state, W_GESTURE,      chip_info->gesture_type,  GTP_DATA0_GESTURE_W);
	/* data1 cmd need set */
	SET_GESTURE_BIT(state, DOU_SWIP,       chip_info->gesture_type1, GTP_DATA1_GESTURE_DOUSWIP)
	SET_GESTURE_BIT(state, SINGLE_TAP,     chip_info->gesture_type1, GTP_DATA1_GESTURE_SINGLE);

	TPD_INFO("GT_brlD:%s: gesture_type :0x%x\n", __func__, chip_info->gesture_type);
	TPD_INFO("GT_brlD:%s: gesture_type1:0x%x\n", __func__, chip_info->gesture_type1);
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
	TPD_INFO("GT_brlD:%s: request state:0x%02x.\n", __func__, rqst_code);

	switch (rqst_code) {
	case GTP_RQST_CONFIG:
		TPD_INFO("GT_brlD:HW request config.\n");
		ret = goodix_send_config(chip_info, chip_info->normal_cfg.data,
					 chip_info->normal_cfg.length);
		if (ret) {
			TPD_INFO("GT_brlD:request config, send config faild.\n");
		}
		break;
	case GTP_RQST_RESET:
		TPD_INFO("GT_brlD:%s: HW requset reset.\n", __func__);
		goodix_reset(chip_info);
		break;
	default:
		TPD_INFO("GT_brlD:%s: Unknown hw request:%d.\n", __func__, rqst_code);
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
	TPD_DEBUG("GT_brlD:%s call\n", __func__);
	goodix_esd_check_enable(chip_info, true);
	return 0;
}

static int goodix_set_smooth_lv_set(void *chip_data, int level)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	int ret = 0;

	TPD_INFO("GT_brlD:%s, %s smooth ->%d\n", __func__, level > 0 ? "enable" : "disable", level);

	if (level > 0) {
		if (goodix_send_cmd_simple(chip_info, GTP_SMOOTH_CMD, (u8)level) < 0) {
			TPD_INFO("GT_brlD:%s, fail set smooth ->%d\n", __func__, level);
			ret = -1;
		}
	} else if (level == 0) {
		if (goodix_send_cmd_simple(chip_info, GTP_SMOOTH_CMD, GTP_MASK_DISABLE) < 0) {
			TPD_INFO("GT_brlD:%s, fail disable smooth\n", __func__);
			ret = -1;
		}
	}

	return ret;
}

static int goodix_set_sensitive_lv_set(void *chip_data, int level)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
		int ret = 0;

	TPD_INFO("GT_brlD:%s, %s sensitive ->%d\n", __func__, level > 0 ? "enable" : "disable", level);

	if (level > 0) {
		if (goodix_send_cmd_simple(chip_info, GTP_SENSITIVE_CMD, (u8)level) < 0) {
			TPD_INFO("GT_brlD:%s, fail set sensitive ->%d\n", __func__, level);
			ret = -1;
		}
	} else if (level == 0) {
		if (goodix_send_cmd_simple(chip_info, GTP_SENSITIVE_CMD, GTP_MASK_DISABLE) < 0) {
			TPD_INFO("GT_brlD:%s, fail disable sensitive\n", __func__);
			ret = -1;
		}
	}

	return ret;
}


/* high frame default enable 60s */
static int goodix_set_high_frame_rate(void *chip_data, int level, int time)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_ts_cmd tmp_cmd;
	int ret = 0;
	TPD_INFO("GT_brlD:%s, %s high frame rate, %d\n", __func__, !!level > 0 ? "enable" : "disable", time);
	if (!!level) {
		tmp_cmd.len     = GTP_CMD_HIGHFRAM_LEN;    /* 6 */
		tmp_cmd.cmd     = GTP_GAME_HIGH_FRAME;     /* 0x25*/
		tmp_cmd.data[0] = GTP_GAME_HIGH_TO_IDLE;   /* 60 */
		tmp_cmd.data[1] = GTP_GAME_ACTIVE_TO_HIGH; /* 1*/
		ret = brl_send_cmd(chip_info, &tmp_cmd);
	}
	return 0;
}

static void goodix_check_bit_chg(struct chip_data_brl *chip_info, unsigned int bit)
{
	if (((BIT_CHK(chip_info->check_old_state, bit) && !BIT_CHK(chip_info->check_now_state, bit)) ||
		(!BIT_CHK(chip_info->check_old_state, bit) && BIT_CHK(chip_info->check_now_state, bit))) &&
		!BIT_CHK(chip_info->check_chg_state, bit)) {
		chip_info->check_chg_state = BIT_SET(bit) | chip_info->check_chg_state;
	} else if (((!BIT_CHK(chip_info->check_old_state, bit) && !BIT_CHK(chip_info->check_now_state, bit)) ||
				(BIT_CHK(chip_info->check_old_state, bit) && BIT_CHK(chip_info->check_now_state, bit))) &&
				BIT_CHK(chip_info->check_chg_state, bit)) {
		chip_info->check_chg_state = BIT_CLR(bit) & chip_info->check_chg_state;
	}
	TPD_DEBUG("GT_brlD: %s-->[old:%d]-[now:%d]-[chg:%d]\n",
		__func__,
		chip_info->check_old_state, chip_info->check_now_state, chip_info->check_chg_state);
}

static void goodix_check_bit_set(struct chip_data_brl *chip_info, unsigned int bit, bool enable)
{
	if (enable == true) {
		chip_info->check_now_state = BIT_SET(bit) | chip_info->check_now_state;
		goodix_check_bit_chg(chip_info, bit);
		chip_info->check_old_state = BIT_SET(bit) | chip_info->check_now_state;
	} else {
		chip_info->check_now_state = BIT_CLR(bit) & chip_info->check_now_state;
		goodix_check_bit_chg(chip_info, bit);
		chip_info->check_old_state = BIT_CLR(bit) & chip_info->check_now_state;
	}
}

static void goodix_state_verify(struct chip_data_brl *chip_info)
{
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->s_client);
	int ret = 0;

	if (chip_info->check_chg_state &&
		chip_info->pen_enable && chip_info->pen_support &&
		!ts->is_suspended) {
		if (BIT_CHK(chip_info->check_now_state, GAME_MODE_ENABLE)) {
			TPD_INFO("GT_brlD: %s->game on->must disable pen\n", __func__);
			ret = goodix_enable_pen_mode(chip_info, false);

			if (BIT_CHK(chip_info->pen_ctl_para, PEN_CTL_SMALL_PALM_ENABLE)) {
				TPD_INFO("GT_brlD: %s->game on->must disable small palm\n", __func__);
				ret = goodix_pen_control_palm_inpen(chip_info, PEN_CTL_SMALL_PALM_CLOSE);
			}

		} else if (!BIT_CHK(chip_info->check_now_state, GAME_MODE_ENABLE)) {
			TPD_INFO("GT_brlD: %s->game off->must enable pen and pen ctl:%d\n", __func__, chip_info->pen_ctl_para);
			ret = goodix_enable_pen_mode(chip_info, true);
		}
	}
}

static int goodix_set_refresh_switch(void *chip_data, int fps)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	bool check_enable = false;
	int ret = 0;

	TPD_DEBUG("GT_brlD:%s: refresh_switch: %d HZ!\n", __func__, fps);
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
		TPD_INFO("GT_brlD:%s: not change rate, check enable->game:%d,pen:%d,gesture:%d,sleep:%d\n",
			__func__, chip_info->game_enable, chip_info->pen_enable, chip_info->gesture_enable, chip_info->sleep_enable);
	}

	return 0;
}

static int goodix_send_temperature(void *chip_data, int temp, bool normal_mode)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	int ret = 0;

	if (BIT_CHK(chip_info->check_now_state, SELF_TEST_ENABLE)) {
		TPD_INFO("GT_brlD:%s :self testing, not send temp to mcu, %d", __func__, chip_info->check_now_state);
		goto OUT;
	}

	if (normal_mode == true) {
		if (chip_info->temp_recorder_cnt == CNT_MAX) {
			chip_info->temp_recorder_cnt = CNT_CLR;
			if (temp != chip_info->gt_temperature[0] && temp != chip_info->gt_temperature[1]) {
				ret = goodix_send_cmd_simple(chip_info, GTP_CMD_TEMP_CMD, temp);
				TPD_INFO("GT_brlD:%s : cnt max, need temp:%d -> ic", __func__, temp);
			}
		}

		if (temp == chip_info->gt_temperature[0]) {
			ret = goodix_send_cmd_simple(chip_info, GTP_CMD_TEMP_CMD, chip_info->gt_temperature[0]);
			TPD_INFO("GT_brlD:%s : send temp %d -> ic", __func__, chip_info->gt_temperature[0]);
		} else if (temp == chip_info->gt_temperature[1]) {
			ret = goodix_send_cmd_simple(chip_info, GTP_CMD_TEMP_CMD, chip_info->gt_temperature[1]);
			TPD_INFO("GT_brlD:%s : send temp %d -> ic", __func__, chip_info->gt_temperature[1]);
		}
		TPD_DEBUG("GT_brlD:%s [cnt:%d][temp:%d][def_temp0:%d,def_temp1:%d]", __func__,
		chip_info->temp_recorder_cnt, temp, chip_info->gt_temperature[0], chip_info->gt_temperature[1]);
		chip_info->temp_recorder_cnt++;
	} else {
		TPD_INFO("GT_brlD:%s now resume, must send temp:%d to ic\n", __func__, temp);
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
	.get_pen_points              = goodix_get_pen_points,
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
	.health_report               = goodix_get_health_info,
	.set_high_frame_rate         = goodix_set_high_frame_rate,
	.smooth_lv_set               = goodix_set_smooth_lv_set,
	.sensitive_lv_set            = goodix_set_sensitive_lv_set,
	.tp_refresh_switch           = goodix_set_refresh_switch,
	.send_temperature            = goodix_send_temperature,
	.pen_uplink_msg              = goodix_pen_uplink_data,
	.pen_downlink_msg            = goodix_pen_downlink_data,
};
/********* End of implementation of oplus_touchpanel_operations callbacks**********************/
static void gt_brld_data_reset(struct chip_data_brl *chip_info,
	u32 addr, u8 clear_state, debug_type debug_type)
{
	int ret = 0;
	switch (debug_type) {
	case GTP_FW_STATUS:
			TPD_INFO("GT_brlD:%s:GTP_FW_STATUS->close debug info\n", __func__);
			ret = goodix_send_cmd_simple(chip_info, GTP_CMD_DEBUG_INFO, GTP_CMD_DEBUG_OFF);
			if (ret < 0) {
				TPD_INFO("GT_brlD:%s:close debug info failed!!\n", __func__);
				ret = RESULT_ERR;
				goto RETURN;
			}
		break;
	default:
		break;
	}

	TPD_INFO("GT_brlD:%s:reset to coor mode->close all debug mode\n", __func__);
	ret = goodix_send_cmd_simple(chip_info, GTP_CMD_UNIFY_CMD, GTP_CMD___NORMAL);
	if (ret < 0) {
		TPD_INFO("GT_brlD:%s:reset to coor mode->close all debug mode failed!!\n", __func__);
		ret = RESULT_ERR;
		goto RETURN;
	}
RETURN:
	ret = goodix_reg_write(chip_info, addr, &clear_state, GTP_CLEAR_STATE_DATA);
	ret = goodix_send_cmd_simple(chip_info, GOODIX_IDLE_CMD, GOODIX_IDLE_START);
	return;
}

static void gt_brld_data_get(struct seq_file *s,
	struct chip_data_brl *chip_info,
	u8 *kernel_buf, u32 addr, int tx_num, int rx_num, s16 data,
	debug_type debug_type)
{
	int i   = 0;
	int j   = 0;
	int ret = 0;

	ret = goodix_reg_read(chip_info, addr, kernel_buf, tx_num * rx_num * 2);
	usleep_range(5000, 5100);
	seq_printf(s, "[%s debug info]\n", TPD_DEVICE);

	switch (debug_type) {
	case GTP_RAWDATA:
			seq_printf(s, "---------------< rawdata >---------------\n");
		break;
	case GTP_DIFFDATA:
			seq_printf(s, "---------------< diffdata >---------------\n");
		break;
	case GTP_BASEDATA:
			seq_printf(s, "---------------< basedata >---------------\n");
		break;
	case GTP_FW_STATUS:
			seq_printf(s, "---------------< fwstatus >---------------\n");
		break;
	case GTP_NORMAL_LIZE:
			seq_printf(s, "---------------< nomalize >---------------\n");
		break;
	default:
		break;
	}
	seq_printf(s, "RX=%d TX=%d\n", rx_num, tx_num);
	seq_printf(s, "[TX] ");
	for (i = 0; i < tx_num; i++)
		seq_printf(s, "%5d ", i);
	seq_printf(s, "\n[RX]\n");
	for (i = 0; i < rx_num; i++) {
		seq_printf(s, "[%2d] ", i);
		for (j = 0; j < tx_num; j++) {
			data = kernel_buf[j * rx_num * 2 + i * 2] +
				  (kernel_buf[j * rx_num * 2 + i * 2 + 1] << 8);
			seq_printf(s, "%5d ", data);
		}
		seq_printf(s, "\n");
	}
	seq_printf(s, "---------------<   end   >---------------\n");
}

static int gt_brld_data_pretreatment(struct chip_data_brl *chip_info,
	u32 addr, u8 clear_state, u8 *kernel_buf, debug_type debug_type)
{
	int i   = 0;
	int ret = 0;

	ret = goodix_send_cmd_simple(chip_info, GOODIX_IDLE_CMD, GOODIX_IDLE_CLOSE);
	switch(debug_type) {
	case GTP_FW_STATUS:
			TPD_INFO("GT_brlD:%s:GTP_FW_STATUS->open debug info\n", __func__);
			ret = goodix_send_cmd_simple(chip_info, GTP_CMD_DEBUG_INFO, GTP_CMD_DEBUG_ON);
			if (ret < 0) {
				TPD_INFO("GT_brlD:%s:open debug info failed!!\n", __func__);
				ret = RESULT_ERR;
				goto RETURN;
			}
			TPD_INFO("GT_brlD:%s:GTP_CMD_DIFFDATA start ready\n", __func__);
			ret = goodix_send_cmd_simple(chip_info, GTP_CMD_UNIFY_CMD, GTP_CMD_DIFFDATA);
			if (ret < 0) {
				TPD_INFO("GT_brlD:%s:GTP_CMD_DIFFDATA ready failed!!\n", __func__);
				ret = RESULT_ERR;
				goto RETURN;
			}
		break;
	case GTP_DIFFDATA:
			TPD_INFO("GT_brlD:%s:only GTP_CMD_DIFFDATA start ready\n", __func__);
			ret = goodix_send_cmd_simple(chip_info, GTP_CMD_UNIFY_CMD, GTP_CMD_DIFFDATA);
			if (ret < 0) {
				TPD_INFO("GT_brlD:%s:only GTP_CMD_DIFFDATA ready failed!!\n", __func__);
				ret = RESULT_ERR;
				goto RETURN;
			}
		break;
	case GTP_RAWDATA:
			TPD_INFO("GT_brlD:%s:GTP_CMD__RAWDATA start ready\n", __func__);
			ret = goodix_send_cmd_simple(chip_info, GTP_CMD_UNIFY_CMD, GTP_CMD__RAWDATA);
			if (ret < 0) {
				TPD_INFO("GT_brlD:%s:GTP_CMD__RAWDATA ready failed!!\n", __func__);
				ret = RESULT_ERR;
				goto RETURN;
			}
		break;
	case GTP_BASEDATA:
			TPD_INFO("GT_brlD:%s:GTP_CMD_BASEDATA start ready\n", __func__);
			ret = goodix_send_cmd_simple(chip_info, GTP_CMD_UNIFY_CMD, GTP_CMD_BASEDATA);
			if (ret < 0) {
				TPD_INFO("GT_brlD:%s:GTP_CMD_BASEDATA ready failed!!\n", __func__);
				ret = RESULT_ERR;
				goto RETURN;
			}
		break;
	default:
		break;
	}

	msleep(20);
	ret = goodix_reg_write(chip_info, GTP_WAIT_DATA_ADDR, &clear_state, GTP_CLEAR_STATE_DATA);
	while (i++ < 10) {
		ret = goodix_reg_read(chip_info, GTP_WAIT_DATA_ADDR, kernel_buf, GTP_CLEAR_STATE_DATA);
		TPD_INFO("GT_brlD:ret=%d  kernel_buf=0x%2x\n", ret, kernel_buf[0]);
		if (!ret && (kernel_buf[0] & GTP_DEBUG_DATA_CHECK)) {
			TPD_INFO("GT_brlD:Data ready OK\n");
			break;
		}
		msleep(20);
	}
	if (i >= 10) {
		TPD_INFO("GT_brlD:data not ready:%d, quit!\n", i);
		ret = RESULT_ERR;
	}

RETURN:
	return ret;
}

/******** Start of implementation of debug_info_proc_operations callbacks*********************/
static void gt_brld_debug_info_read(struct seq_file *s,
				   void *chip_data, debug_type debug_type)
{
	int ret = RESULT_ERR;
	u8 *kernel_buf = NULL;
	u32 addr = 0;
	struct chip_data_brl  *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_ic_info *ic_info;
	int tx_num = 0;
	int rx_num = 0;
	s16 data   = 0;
	u8 clear_state = 0;

	chip_info->rawdiff_mode = ON;
	ic_info = &chip_info->ic_info;
	rx_num = ic_info->parm.sen_num;
	tx_num = ic_info->parm.drv_num;

	if (!tx_num || !rx_num) {
		TPD_INFO("GT_brlD:%s: error invalid tx %d or rx %d num\n",
			 __func__, tx_num, rx_num);
		return;
	}

	kernel_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (kernel_buf == NULL) {
		TPD_INFO("GT_brlD:%s kmalloc error\n", __func__);
		return;
	}

	mutex_lock(&chip_info->debug_lock);
	memset(kernel_buf, GTP_PAGE_CLEAR, PAGE_SIZE);
	addr = GTP_DEBUG_DATA_ADDR;

	TPD_INFO("GT_brlD:%s:ready pass [trx:%2d,%2d][addr:0x%2x]\n",
		__func__, tx_num, rx_num, addr);

	ret = gt_brld_data_pretreatment(chip_info, addr, clear_state, kernel_buf, debug_type);
	if (ret < 0) {
		TPD_INFO("GT_brlD:pretreatment failed, quit!\n");
		goto read_data_exit;
	}

	gt_brld_data_get(s, chip_info, kernel_buf, addr,
	                     tx_num, rx_num, data, debug_type);

read_data_exit:
	kfree(kernel_buf);
	mutex_unlock(&chip_info->debug_lock);
	gt_brld_data_reset(chip_info, addr, clear_state, debug_type);
	chip_info->rawdiff_mode = OFF;
	return;
}

static void gt_brld_delta_read(struct seq_file *s, void *chip_data)
{
	TPD_INFO("GT_brlD:%s: start\n", __func__);
	gt_brld_debug_info_read(s, chip_data, GTP_FW_STATUS);
	gt_brld_debug_info_read(s, chip_data, GTP_DIFFDATA);
	gt_brld_debug_info_read(s, chip_data, GTP_RAWDATA);
	gt_brld_debug_info_read(s, chip_data, GTP_BASEDATA);
}

static void goodix_baseline_read(struct seq_file *s, void *chip_data)
{
	gt_brld_debug_info_read(s, chip_data, GTP_RAWDATA);
}

static void goodix_main_register_read(struct seq_file *s, void *chip_data)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	struct goodix_ic_info_misc *ic_info_misc;
	int ret = 0;
	u8 touch_data[IRQ_EVENT_HEAD_LEN + BYTES_PER_POINT + COOR_DATA_CHECKSUM_SIZE] = {0};

	ic_info_misc = &chip_info->ic_info.misc;
	seq_printf(s, "====================================================\n");
	if (chip_info->p_tp_fw) {
		seq_printf(s, "tp fw = 0x%s\n", chip_info->p_tp_fw);
	}
	ret = goodix_reg_read(chip_info, ic_info_misc->touch_data_addr,
			      touch_data, sizeof(touch_data));
	if (ret < 0) {
		TPD_INFO("GT_brlD:%s: i2c transfer error!\n", __func__);
		goto out;
	}
	seq_printf(s, "cached touch_data: %*ph\n", (int)sizeof(touch_data), chip_info->touch_data);
	TPD_INFO("GT_brlD:%s: cached touch_data: %*ph\n", __func__, (int)sizeof(touch_data), chip_info->touch_data);
	seq_printf(s, "current touch_data: %*ph\n", (int)sizeof(touch_data), touch_data);
	TPD_INFO("GT_brlD:%s: current touch_data: %*ph\n", __func__, (int)sizeof(touch_data), touch_data);
	seq_printf(s, "====================================================\n");
out:
	return;
}

static void goodix_limit_write(void *chip_data, int32_t count)
{
	struct chip_data_brl *chip_info;
	int rx_num, tx_num;
	u32 diff_size;

	chip_info = (struct chip_data_brl *)chip_data;
	tx_num = chip_info->hw_res->tx_num;
	rx_num = chip_info->hw_res->rx_num;
	diff_size = tx_num*rx_num + tx_num+rx_num;

	if (count) {
		chip_info->diff_buf = kzalloc(diff_size*sizeof(s16), GFP_KERNEL);
		if (chip_info->diff_buf == NULL) {
			TPD_INFO("%s: kmalloc error\n", __func__);
			goto exit;
		}

		chip_info->diff_rw_buf = kzalloc(diff_size*2, GFP_KERNEL);
		if (chip_info->diff_rw_buf == NULL) {
			TPD_INFO("%s: kmalloc error\n", __func__);
			goto err_rw_buf_alloc;
		}

		memset(chip_info->diff_buf, 0, diff_size*sizeof(s16));
		memset(chip_info->diff_rw_buf, 0, diff_size*2);
		chip_info->diff_size = diff_size;
		chip_info->enable_differ_mode = !!count;
		goto exit;
	}
	else {
		chip_info->enable_differ_mode = false;
		goto diff_mode_off;
	}

diff_mode_off:
	kfree(chip_info->diff_rw_buf);
	chip_info->diff_rw_buf = NULL;
err_rw_buf_alloc:
	kfree(chip_info->diff_buf);
	chip_info->diff_buf = NULL;

exit:
	TPD_INFO("%s:tx is %d, rx is %d, differ mode is %s", __func__, tx_num, rx_num, chip_info->enable_differ_mode ? "enable" : "disable");
}

static struct debug_info_proc_operations debug_info_proc_ops = {
	/*    .limit_read         = goodix_limit_read,*/
	.baseline_blackscreen_read = goodix_baseline_read,
	.delta_read                = gt_brld_delta_read,
	.baseline_read             = goodix_baseline_read,
	.main_register_read        = goodix_main_register_read,
	.tp_limit_data_write	 = goodix_limit_write,
};
/********* End of implementation of debug_info_proc_operations callbacks**********************/

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
		TPD_INFO("GT_brlD:%s: failed send config read prepare command\n", __func__);
		return ret;
	}

	ret = goodix_reg_read(chip_info, misc->fw_buffer_addr,
			      cfg_head.buf, sizeof(cfg_head));
	if (ret) {
		TPD_INFO("GT_brlD:%s: failed read config head %d\n", __func__, ret);
		goto exit;
	}

	if (checksum_cmp(cfg_head.buf, sizeof(cfg_head), CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("GT_brlD:%s: config head checksum error\n", __func__);
		ret = -1;
		goto exit;
	}

	cfg_head.cfg_len = le16_to_cpu(cfg_head.cfg_len);
	if (cfg_head.cfg_len > misc->fw_buffer_max_len ||
	    cfg_head.cfg_len > size) {
		TPD_INFO("GT_brlD:%s: cfg len exceed buffer size %d > %d\n", __func__, cfg_head.cfg_len,
			 misc->fw_buffer_max_len);
		ret = -1;
		goto exit;
	}

	memcpy(cfg, cfg_head.buf, sizeof(cfg_head));
	ret = goodix_reg_read(chip_info, misc->fw_buffer_addr + sizeof(cfg_head),
			      cfg + sizeof(cfg_head), cfg_head.cfg_len);
	if (ret) {
		TPD_INFO("GT_brlD:%s: failed read cfg pack, %d\n", __func__, ret);
		goto exit;
	}

	TPD_INFO("GT_brlD:config len %d\n", cfg_head.cfg_len);
	if (checksum_cmp(cfg + sizeof(cfg_head),
			 cfg_head.cfg_len, CHECKSUM_MODE_U16_LE)) {
		TPD_INFO("GT_brlD:%s: config body checksum error\n", __func__);
		ret = -1;
		goto exit;
	}
	TPD_INFO("GT_brlD:success read config data: len %zu\n",
		 cfg_head.cfg_len + sizeof(cfg_head));
exit:
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_READ_EXIT;
	if (send_cfg_cmd(chip_info, &cfg_cmd)) {
		TPD_INFO("GT_brlD:%s: failed send config read finish command\n", __func__);
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
		TPD_INFO("GT_brlD:%s: failed get ic info, ret %d\n", __func__, ret);
		goto exit;
	}
	ret = brl_read_version(chip_info, fw_version);
	if (ret) {
		TPD_INFO("GT_brlD:goodix_config_info_read goodix_read_config error:%d\n", ret);
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

/************** Start of auto test func**************************/
#define ABS(val)			((val < 0)? -(val) : val)
#define MAX(a, b)			((a > b)? a : b)
static void goodix_cache_deltadata(struct chip_data_brl *chip_data)
{
	u32 data_size;
	struct goodix_ts_test *ts_test = chip_data->brl_test;
	int tx = chip_data->ic_info.parm.drv_num;
	int j;
	int max_val;
	int raw;
	int temp;

	data_size = ts_test->rawdata.size;

		for (j = 0; j < data_size; j++) {
			raw = ts_test->rawdata.data[j];
			max_val = 0;
			/* calcu delta with above node */
			if (j - tx >= 0) {
				temp = ts_test->rawdata.data[j - tx];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with bellow node */
			if (j + tx < data_size) {
				temp = ts_test->rawdata.data[j + tx];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with left node */
			if (j % tx) {
				temp = ts_test->rawdata.data[j - 1];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with right node */
			if ((j + 1) % tx) {
				temp = ts_test->rawdata.data[j + 1];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			ts_test->deltadata.data[j] = max_val * 1000 / raw;
		}
}

static int brl_auto_test_preoperation(struct seq_file *s,
				       void *chip_data,
				       struct auto_testdata *goodix_testdata,
				       struct test_item_info *p_test_item_info)
{
	struct chip_data_brl *cd = (struct chip_data_brl *)chip_data;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	int ret;

	TPD_INFO("GT_brlD:%s IN\n", __func__);

	goodix_check_bit_set(cd, SELF_TEST_ENABLE, true);

	cd->brl_test = kzalloc(sizeof(struct goodix_ts_test), GFP_KERNEL);
	cd->brl_test->rawdata.data = kzalloc(tx * rx * 2, GFP_KERNEL);
	cd->brl_test->rawdata.size = tx * rx;
	cd->brl_test->noisedata.data = kzalloc(tx * rx * 2, GFP_KERNEL);
	cd->brl_test->noisedata.size = tx * rx;

	cd->brl_test->deltadata.data = kzalloc(tx * rx * 2, GFP_KERNEL);
	cd->brl_test->deltadata.size = tx * rx;

	cd->brl_test->self_rawdata.data = kzalloc((tx + rx) * 2, GFP_KERNEL);
	cd->brl_test->self_rawdata.size = tx + rx;

	/* disabled irq */
	disable_irq(cd->irq);

	if (cd->test_cfg.length > 0) {
		ret = goodix_send_config(cd, cd->test_cfg.data, cd->test_cfg.length);
		if (ret < 0) {
			TPD_INFO("GT_brlD:send test config failed\n");
			return -EINVAL;
		}
	}

	TPD_INFO("GT_brlD:%s OUT\n", __func__);

	return 0;
}

static int brl_clk_test(struct seq_file *s,
				void *chip_data,
			    struct auto_testdata *goodix_testdata,
				struct test_item_info *p_test_item_info)
{
	struct chip_data_brl *cd = (struct chip_data_brl *)chip_data;
	u8 prepare_cmd[] = {0x00, 0x00, 0x04, 0x0D, 0x11, 0x00};
	u8 start_cmd[] = {0x00, 0x00, 0x04, 0x0E, 0x12, 0x00};
	u32 cmd_addr = cd->ic_info.misc.cmd_addr;
	u32 frame_buf_addr = cd->ic_info.misc.fw_buffer_addr;
	u8 rcv_buf[10];
	struct clk_test_parm clk_parm;
	u64 cal_freq;
	u64 main_clk;
	u64 clk_in_cnt;
	u64 clk_osc_cnt;
	u64 pen_osc_clk;
	u64 freq_delta;
	int retry;
	int ret = RESULT_ERR;

	TPD_INFO("GT_brlD:%s IN\n", __func__);

	if (cd->pen_support == false) {
		TPD_INFO("GT_brlD:%s pen_support disable, no need do OSC test!!\n", __func__);
		return 0;
	}

	if (cd->pen_osc_frequency == PEN_OSC_FREQ_64K)
		pen_osc_clk = EXT_OSC_FREQ_64K;
	else
		pen_osc_clk = EXT_OSC_FREQ_16K;

	TPD_INFO("GT_brlD:%s: osc clk is %llu\n", __func__, pen_osc_clk);

	goodix_reset(cd);

	goodix_spi_write(cd, cmd_addr, prepare_cmd, sizeof(prepare_cmd));

	retry = 10;
	while (retry--) {
		msleep(20);
		goodix_spi_read(cd, cmd_addr, rcv_buf, 2);
		if (rcv_buf[0] == 0x08 && rcv_buf[1] == 0x80)
			break;
	}
	if (retry < 0) {
		TPD_INFO("GT_brlD:%s: switch clk test mode failed, sta[%x] ack[%x]\n",
				__func__, rcv_buf[0], rcv_buf[1]);
		goto exit;
	}

	clk_parm.gio = 19;
	clk_parm.div = 1;
	clk_parm.is_64m = 1;
	clk_parm.en = 0;
	clk_parm.pll_prediv = 0;
	clk_parm.pll_fbdiv = 0;
	clk_parm.trigger_mode = 0;
	clk_parm.clk_in_num = 1000;
	goodix_append_checksum(clk_parm.buf, sizeof(clk_parm) - 2, CHECKSUM_MODE_U8_LE);
	goodix_spi_write(cd, frame_buf_addr, clk_parm.buf, sizeof(clk_parm));

	goodix_spi_write(cd, cmd_addr, start_cmd, sizeof(start_cmd));
	retry = 20;
	while (retry--) {
		msleep(50);
		goodix_spi_read(cd, cmd_addr, rcv_buf, 2);
		if (rcv_buf[0] == 0x0B && rcv_buf[1] == 0x80)
			break;
	}
	if (retry < 0) {
		TPD_INFO("GT_brlD:%s: wait clk test result failed, sta[%x] ack[%x]\n",
				__func__, rcv_buf[0], rcv_buf[1]);
		goto exit;
	}

	goodix_spi_read(cd, frame_buf_addr + sizeof(clk_parm), rcv_buf, sizeof(rcv_buf));
	if (checksum_cmp(rcv_buf, sizeof(rcv_buf), CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("GT_brlD:%s: clk test result checksum error, [%*ph]\n",
				__func__, (int)sizeof(rcv_buf), rcv_buf);
		goto exit;
	}

	main_clk = le16_to_cpup((__le16 *)&rcv_buf[0]);
	clk_in_cnt = le16_to_cpup((__le16 *)&rcv_buf[2]);
	clk_osc_cnt = le32_to_cpup((__le32 *)&rcv_buf[4]);
	cal_freq = clk_in_cnt * main_clk * 1000000 * 8 / clk_osc_cnt;
	TPD_INFO("GT_brlD:%s: main_clk:%lldM clk_in_cnt:%lld clk_osc_cnt:%lld cal_freq:%lld\n",
		__func__, main_clk, clk_in_cnt, clk_osc_cnt, cal_freq);

	if (pen_osc_clk > cal_freq)
		freq_delta = pen_osc_clk - cal_freq;
	else
		freq_delta = cal_freq - pen_osc_clk;
	if (freq_delta * 100 / pen_osc_clk > 2) {
		TPD_INFO("GT_brlD:%s: osc clk test failed\n", __func__);
	} else {
		TPD_INFO("GT_brlD:%s: osc clk test pass\n", __func__);
		ret = 0;
	}

exit:
	return ret;
}

static int brl_noisedata_test(struct seq_file *s,
				void *chip_data,
			    struct auto_testdata *goodix_testdata,
				struct test_item_info *p_test_item_info)
{
	struct chip_data_brl *cd = (struct chip_data_brl *)chip_data;
	struct goodix_ts_test *ts_test = cd->brl_test;
	struct goodix_ts_cmd temp_cmd;
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	u32 noise_data_addr;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	int ret;
	int retry;
	int discard_cnt = 5;
	u8 status;
	int val;
	int noise_threshold;
	int err_cnt = 0;
	int i;

	TPD_INFO("GT_brlD:%s IN\n", __func__);

	if (p_test_item_info->para_num && p_test_item_info->p_buffer[0]) {
		ts_test->is_item_support[TYPE_TEST1] = 1;
		noise_threshold = p_test_item_info->p_buffer[1];
		TPD_INFO("GT_brlD:noise_threshold:%d\n", noise_threshold);
	} else {
		TPD_INFO("GT_brlD:skip noisedata test\n");
		return 0;
	}

	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x82;
	temp_cmd.len = 5;
	ret = brl_send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		TPD_INFO("GT_brlD:%s switch noisedata mode failed\n", __func__);
		return RESULT_ERR;
	}

	status = 0;
	while (--discard_cnt) {
		msleep(20);
		goodix_reg_write(cd, sync_addr, &status, 1);
	}

	retry = 20;
	while (retry--) {
		usleep_range(5000, 5100);
		goodix_reg_read(cd, sync_addr, &status, 1);
		if (status == 0x80)
			break;
	}
	if (retry < 0) {
		TPD_INFO("GT_brlD:noisedata is not ready val:0x%02x exit", status);
		return RESULT_ERR;
	}

	noise_data_addr = sync_addr + cd->ic_info.misc.frame_data_head_len +
			cd->ic_info.misc.fw_attr_len +
			cd->ic_info.misc.fw_log_len + 8;

	goodix_reg_read(cd, noise_data_addr, (u8 *)ts_test->noisedata.data,
			ts_test->noisedata.size * 2);
	goodix_rotate_abcd2cbad(tx, rx, ts_test->noisedata.data);

	for (i = 0; i < ts_test->noisedata.size; i++) {
		val = ts_test->noisedata.data[i];
		val = ABS(val);

		if (val > noise_threshold) {
			TPD_INFO("GT_brlD:noisedata[%d]:%d > noise_threshold[%d]\n", i, val, noise_threshold);
			seq_printf(s, "noisedata[%d]:%d > noise_threshold[%d]\n", i, val, noise_threshold);
			err_cnt++;
		}
	}

	if (err_cnt > 0)
		return RESULT_ERR;

	ts_test->test_result[TYPE_TEST1] = GTP_TEST_OK;
	return 0;
}

static int brl_capacitance_test(struct seq_file *s,
				void *chip_data,
			    struct auto_testdata *goodix_testdata,
				struct test_item_info *p_test_item_info)
{
	struct chip_data_brl *cd = (struct chip_data_brl *)chip_data;
	struct goodix_ts_test *ts_test = cd->brl_test;
	struct goodix_ts_cmd temp_cmd;
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	u32 raw_data_addr;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	int ret;
	int retry;
	int discard_cnt = 5;
	u8 status;
	int i;
	int val;
	int err_cnt = 0;
	int32_t *max_limit;
	int32_t *min_limit;

	TPD_INFO("GT_brlD:%s IN\n", __func__);

	if (p_test_item_info->para_num && p_test_item_info->p_buffer[0]) {
		ts_test->is_item_support[TYPE_TEST2] = 1;
		if (p_test_item_info->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
				max_limit = (int32_t *)(goodix_testdata->fw->data +
								       p_test_item_info->top_limit_offset);
				min_limit = (int32_t *)(goodix_testdata->fw->data +
								       p_test_item_info->floor_limit_offset);
		} else {
			TPD_INFO("GT_brlD:item: %d get_test_item_info fail\n", LIMIT_TYPE_TX_RX_DATA);
			return RESULT_ERR;
		}
	} else {
		TPD_INFO("GT_brlD:skip rawdata test\n");
		return 0;
	}

	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x81;
	temp_cmd.len = 5;
	ret = brl_send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		TPD_INFO("GT_brlD:%s switch rawdata mode failed\n", __func__);
		return RESULT_ERR;
	}

	status = 0;
	while (--discard_cnt) {
		msleep(20);
		goodix_reg_write(cd, sync_addr, &status, 1);
	}

	retry = 20;
	while (retry--) {
		usleep_range(5000, 5100);
		goodix_reg_read(cd, sync_addr, &status, 1);
		if (status == 0x80)
			break;
	}
	if (retry < 0) {
		TPD_INFO("GT_brlD:rawdata is not ready val:0x%02x exit", status);
		return RESULT_ERR;
	}

	raw_data_addr = sync_addr + cd->ic_info.misc.frame_data_head_len +
			cd->ic_info.misc.fw_attr_len +
			cd->ic_info.misc.fw_log_len + 8;

	goodix_reg_read(cd, raw_data_addr, (u8 *)ts_test->rawdata.data,
			ts_test->rawdata.size * 2);
	goodix_rotate_abcd2cbad(tx, rx, ts_test->rawdata.data);

	for (i = 0; i < ts_test->rawdata.size; i++) {
		val = ts_test->rawdata.data[i];
		if (val > max_limit[i] || val < min_limit[i]) {
			TPD_INFO("GT_brlD:rawdata[%d] out of threshold[%d,%d]\n", val, min_limit[i], max_limit[i]);
			seq_printf(s, "rawdata[%d] out of threshold[%d,%d]\n", val, min_limit[i], max_limit[i]);
			err_cnt++;
		}
	}

	if (err_cnt > 0)
		return RESULT_ERR;

	ts_test->test_result[TYPE_TEST2] = GTP_TEST_OK;
	return 0;
}

static int brl_deltacapacitance(struct seq_file *s,
				void *chip_data,
			    struct auto_testdata *goodix_testdata,
				struct test_item_info *p_test_item_info)
{
	struct chip_data_brl *cd = (struct chip_data_brl *)chip_data;
	struct goodix_ts_test *ts_test = cd->brl_test;
	int i;
	int val;
	int err_cnt = 0;
	int32_t *delta_threshold;

	TPD_INFO("GT_brlD:%s IN\n", __func__);

	if (p_test_item_info->para_num && p_test_item_info->p_buffer[0]) {
		ts_test->is_item_support[TYPE_TEST3] = 1;
		if (p_test_item_info->item_limit_type == LIMIT_TYPE_TX_RX_DATA) {
			delta_threshold = (int32_t *)(goodix_testdata->fw->data +
								p_test_item_info->top_limit_offset);
		} else {
			TPD_INFO("GT_brlD:item: %d get_test_item_info fail\n", LIMIT_TYPE_TX_RX_DATA);
			return RESULT_ERR;
		}
	} else {
		TPD_INFO("GT_brlD:skip deltadata test\n");
		return 0;
	}

	goodix_cache_deltadata(cd);

	ts_test->test_result[TYPE_TEST3] = GTP_TEST_OK;
	for (i = 0; i < ts_test->deltadata.size; i++) {
		val = ts_test->deltadata.data[i];
		if (val > delta_threshold[i]) {
			TPD_INFO("GT_brlD:deltadata[%d] out of threshold[%d]\n", val, delta_threshold[i]);
			seq_printf(s, "deltadata[%d] out of threshold[%d]\n", val, delta_threshold[i]);
			err_cnt++;
		}
	}

	if (err_cnt > 0) {
		ts_test->test_result[TYPE_TEST3] = GTP_TEST_NG;
		return RESULT_ERR;
	}

	return 0;
}

static int brl_self_rawcapacitance(struct seq_file *s,
				void *chip_data,
			    struct auto_testdata *goodix_testdata,
				struct test_item_info *p_test_item_info)
{
	struct chip_data_brl *cd = (struct chip_data_brl *)chip_data;
	struct goodix_ts_test *ts_test = cd->brl_test;
	struct goodix_ts_cmd temp_cmd;
	u32 sync_addr = cd->ic_info.misc.frame_data_addr;
	u32 selfraw_data_addr;
	int ret;
	int retry;
	int discard_cnt = 5;
	u8 status;
	int i;
	int val;
	int err_cnt = 0;
	int32_t *max_limit;
	int32_t *min_limit;

	TPD_INFO("GT_brlD:%s IN\n", __func__);

	if (p_test_item_info->para_num && p_test_item_info->p_buffer[0]) {
		ts_test->is_item_support[TYPE_TEST4] = 1;

		if (p_test_item_info->item_limit_type == LIMIT_TYPE_SLEF_TX_RX_DATA) {
			max_limit = (int32_t *)(goodix_testdata->fw->data +
							    p_test_item_info->top_limit_offset);
			min_limit = (int32_t *)(goodix_testdata->fw->data +
							    p_test_item_info->floor_limit_offset);
		} else {
			TPD_INFO("GT_brlD:item: %d get_test_item_info fail\n", TYPE_TEST4);
			return RESULT_ERR;
		}
	} else {
		TPD_INFO("GT_brlD:skip self_rawdata test\n");
		return 0;
	}

	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x81;
	temp_cmd.len = 5;
	ret = brl_send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		TPD_INFO("GT_brlD:%s switch self rawdata mode failed\n", __func__);
		return RESULT_ERR;
	}

	status = 0;
	while (--discard_cnt) {
		msleep(20);
		goodix_reg_write(cd, sync_addr, &status, 1);
	}

	retry = 20;
	while (retry--) {
		usleep_range(5000, 5100);
		goodix_reg_read(cd, sync_addr, &status, 1);
		if (status == 0x80)
			break;
	}
	if (retry < 0) {
		TPD_INFO("GT_brlD:self rawdata is not ready val:0x%02x exit", status);
		return RESULT_ERR;
	}

	selfraw_data_addr = sync_addr + cd->ic_info.misc.frame_data_head_len +
			cd->ic_info.misc.fw_attr_len +
			cd->ic_info.misc.fw_log_len +
			cd->ic_info.misc.mutual_struct_len + 10;

	goodix_reg_read(cd, selfraw_data_addr, (u8 *)ts_test->self_rawdata.data,
			ts_test->self_rawdata.size * 2);

	for (i = 0; i < ts_test->self_rawdata.size; i++) {
		val = ts_test->self_rawdata.data[i];
		if (val > max_limit[i] || val < min_limit[i]) {
			TPD_INFO("GT_brlD:self_rawdata[%d] out of threshold[%d,%d]\n", val, min_limit[i], max_limit[i]);
			seq_printf(s, "self_rawdata[%d] out of threshold[%d,%d]\n", val, min_limit[i], max_limit[i]);
			err_cnt++;
		}
	}

	if (err_cnt > 0)
		return RESULT_ERR;

	ts_test->test_result[TYPE_TEST4] = GTP_TEST_OK;
	return 0;
}

static int brl_short_test_prepare(struct chip_data_brl *cd)
{
	struct goodix_ts_cmd tmp_cmd;
	int ret;
	int retry;
	int resend = 3;
	u8 status;

	TPD_INFO("GT_brlD:short test prepare IN\n");

	tmp_cmd.len = 4;
	tmp_cmd.cmd = INSPECT_FW_SWITCH_CMD;

resend_cmd:
	goodix_reset(cd);
	ret = brl_send_cmd(cd, &tmp_cmd);
	if (ret < 0) {
		TPD_INFO("GT_brlD:send test mode failed\n");
		return ret;
	}

	retry = 3;
	while (retry--) {
		msleep(40);
		ret = goodix_reg_read(cd, SHORT_TEST_RUN_REG, &status, 1);
		if (!ret && status == SHORT_TEST_RUN_FLAG)
			return 0;
		TPD_INFO("GT_brlD:short_mode_status=0x%02x ret=%d\n", status, ret);
	}

	if (resend--)
		goto resend_cmd;

	return -EINVAL;
}

static u32 map_die2pin(struct ts_test_params *test_params, u32 chn_num)
{
	int i = 0;
	u32 res = 255;

	if (chn_num & DRV_CHANNEL_FLAG)
		chn_num = (chn_num & ~DRV_CHANNEL_FLAG) + test_params->max_sen_num;

	for (i = 0; i < test_params->max_sen_num; i++) {
		if (test_params->sen_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	/* res != 255 mean found the corresponding channel num */
	if (res != 255)
		return res;
	/* if cannot find in SenMap try find in DrvMap */
	for (i = 0; i < test_params->max_drv_num; i++) {
		if (test_params->drv_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	if (i >= test_params->max_drv_num)
		TPD_INFO("GT_brlD:Faild found corrresponding channel num:%d\n", chn_num);
	else
		res |= DRV_CHANNEL_FLAG;

	return res;
}

static int gdix_check_tx_tx_shortcircut(struct chip_data_brl *cd,
        u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf;
	u32 data_reg = DRV_DRV_SELFCODE_REG_BRD;
	struct goodix_ts_test *ts_test = cd->brl_test;
	struct ts_test_params *test_params = &ts_test->test_params;
	int max_drv_num = test_params->max_drv_num;
	int max_sen_num = test_params->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_drv_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		return -ENOMEM;
	}
	/* drv&drv shortcircut check */
	for (i = 0; i < short_ch_num; i++) {
		ret = goodix_reg_read(cd, data_reg, data_buf, size);
		if (ret < 0) {
			TPD_INFO("GT_brlD:Failed read Drv-to-Drv short rawdata\n");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			TPD_INFO("GT_brlD:Drv-to-Drv adc data checksum error\n");
			err = -EINVAL;
			break;
		}

		r_threshold = test_params->r_drv_drv_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		short_die_num -= max_sen_num;
		if (short_die_num >= max_drv_num) {
			TPD_INFO("GT_brlD:invalid short pad num:%d\n",
				short_die_num + max_sen_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			TPD_INFO("GT_brlD:invalid self_capdata:0x%x\n", self_capdata);
			continue;
		}

		for (j = short_die_num + 1; j < max_drv_num; j++) {
			adc_signal = le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num =
					map_die2pin(test_params, short_die_num + max_sen_num);
				slave_pin_num =
					map_die2pin(test_params, j + max_sen_num);
				if (master_pin_num == 0xFF || slave_pin_num == 0xFF) {
					TPD_INFO("GT_brlD:WARNNING invalid pin\n");
					continue;
				}
				TPD_INFO("GT_brlD:short circut:R=%dK,R_Threshold=%dK",
							short_r, r_threshold);
				TPD_INFO("GT_brlD:%s%d--%s%d shortcircut\n",
					(master_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_rx_rx_shortcircut(struct chip_data_brl *cd,
        u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf;
	u32 data_reg = SEN_SEN_SELFCODE_REG_BRD;
	struct goodix_ts_test *ts_test = cd->brl_test;
	struct ts_test_params *test_params = &ts_test->test_params;
	int max_sen_num = test_params->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_sen_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		return -ENOMEM;
	}
	/* drv&drv shortcircut check */
	for (i = 0; i < short_ch_num; i++) {
		ret = goodix_reg_read(cd, data_reg, data_buf, size);
		if (ret) {
			TPD_INFO("GT_brlD:Failed read Sen-to-Sen short rawdata\n");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			TPD_INFO("GT_brlD:Sen-to-Sen adc data checksum error\n");
			err = -EINVAL;
			break;
		}

		r_threshold = test_params->r_sen_sen_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		if (short_die_num >= max_sen_num) {
			TPD_INFO("GT_brlD:invalid short pad num:%d\n", short_die_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			TPD_INFO("GT_brlD:invalid self_capdata:0x%x\n", self_capdata);
			continue;
		}

		for (j = short_die_num + 1; j < max_sen_num; j++) {
			adc_signal = le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(test_params, short_die_num);
				slave_pin_num = map_die2pin(test_params, j);
				if (master_pin_num == 0xFF || slave_pin_num == 0xFF) {
					TPD_INFO("GT_brlD:WARNNING invalid pin\n");
					continue;
				}
				TPD_INFO("GT_brlD:short circut:R=%dK,R_Threshold=%dK\n",
							short_r, r_threshold);
				TPD_INFO("GT_brlD:%s%d--%s%d shortcircut\n",
					(master_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_tx_rx_shortcircut(struct chip_data_brl *cd,
        u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf = NULL;
	u32 data_reg = DRV_SEN_SELFCODE_REG_BRD;
	struct goodix_ts_test *ts_test = cd->brl_test;
	struct ts_test_params *test_params = &ts_test->test_params;
	int max_drv_num = test_params->max_drv_num;
	int max_sen_num = test_params->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_drv_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		return -ENOMEM;
	}
	/* drv&sen shortcircut check */
	for (i = 0; i < short_ch_num; i++) {
		ret = goodix_reg_read(cd, data_reg, data_buf, size);
		if (ret) {
			TPD_INFO("GT_brlD:Failed read Drv-to-Sen short rawdata\n");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			TPD_INFO("GT_brlD:Drv-to-Sen adc data checksum error\n");
			err = -EINVAL;
			break;
		}

		r_threshold = test_params->r_drv_sen_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		if (short_die_num >= max_sen_num) {
			TPD_INFO("GT_brlD:invalid short pad num:%d\n", short_die_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			TPD_INFO("GT_brlD:invalid self_capdata:0x%x\n", self_capdata);
			continue;
		}

		for (j = 0; j < max_drv_num; j++) {
			adc_signal = le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(test_params, short_die_num);
				slave_pin_num = map_die2pin(test_params, j + max_sen_num);
				if (master_pin_num == 0xFF || slave_pin_num == 0xFF) {
					TPD_INFO("GT_brlD:WARNNING invalid pin\n");
					continue;
				}
				TPD_INFO("GT_brlD:short circut:R=%dK,R_Threshold=%dK\n",
							short_r, r_threshold);
				TPD_INFO("GT_brlD:%s%d--%s%d shortcircut\n",
					(master_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_resistance_to_gnd(struct chip_data_brl *cd,
        u16 adc_signal, u32 pos)
{
	long r = 0;
	u16 r_th = 0, avdd_value = 0;
	u16 chn_id_tmp = 0;
	u8 pin_num = 0;
	unsigned short short_type;
	struct goodix_ts_test *ts_test = cd->brl_test;
	struct ts_test_params *test_params = &ts_test->test_params;
	int max_drv_num = test_params->max_drv_num;
	int max_sen_num = test_params->max_sen_num;

	avdd_value = test_params->avdd_value;
	short_type = adc_signal & 0x8000;
	adc_signal &= ~0x8000;
	if (adc_signal == 0)
		adc_signal = 1;

	if (short_type == 0) {
		/* short to GND */
		r = cal_cha_to_gnd_res(adc_signal);
	} else {
		/* short to VDD */
		r = cal_cha_to_avdd_res(adc_signal, avdd_value);
	}

	if (pos < max_drv_num)
		r_th = test_params->r_drv_gnd_threshold;
	else
		r_th = test_params->r_sen_gnd_threshold;

	chn_id_tmp = pos;
	if (chn_id_tmp < max_drv_num)
		chn_id_tmp += max_sen_num;
	else
		chn_id_tmp -= max_drv_num;

	if (r < r_th) {
		pin_num = map_die2pin(test_params, chn_id_tmp);
		TPD_INFO("GT_brlD:%s%d shortcircut to %s,R=%ldK,R_Threshold=%dK\n",
				(pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
				(pin_num & ~DRV_CHANNEL_FLAG),
				short_type ? "VDD" : "GND",
				r, r_th);
		return -EINVAL;
	}

	return 0;
}

static int gdix_check_gndvdd_shortcircut(struct chip_data_brl *cd)
{
	int ret = 0, err = 0;
	int size = 0, i = 0;
	u16 adc_signal = 0;
	u32 data_reg = DIFF_CODE_DATA_REG_BRD;
	u8 *data_buf = NULL;
	struct goodix_ts_test *ts_test = cd->brl_test;
	int max_drv_num = ts_test->test_params.max_drv_num;
	int max_sen_num = ts_test->test_params.max_sen_num;

	size = (max_drv_num + max_sen_num) * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		return -ENOMEM;
	}
	/* read diff code, diff code will be used to calculate
		* resistance between channel and GND */
	ret = goodix_reg_read(cd, data_reg, data_buf, size);
	if (ret < 0) {
		TPD_INFO("GT_brlD:Failed read to-gnd rawdata\n");
		err = -EINVAL;
		goto err_out;
	}

	if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("GT_brlD:diff code checksum error\n");
		err = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < max_drv_num + max_sen_num; i++) {
		adc_signal = le16_to_cpup((__le16 *)&data_buf[i * 2]);
		ret = gdix_check_resistance_to_gnd(cd,
					adc_signal, i);
		if (ret != 0) {
			TPD_INFO("GT_brlD:Resistance to-gnd/vdd short\n");
			err = ret;
		}
	}

err_out:
	kfree(data_buf);
	return err;
}

static int brl_shortcircut_analysis(struct chip_data_brl *cd)
{
	int ret;
	int err = 0;
	test_result_t test_result;

	ret = goodix_reg_read(cd, SHORT_TEST_RESULT_REG_BRD,
		(u8 *)&test_result, sizeof(test_result));
	if (ret < 0) {
		TPD_INFO("GT_brlD:%s: Read TEST_RESULT_REG failed\n", __func__);
		return ret;
	}

	if (checksum_cmp((u8 *)&test_result, sizeof(test_result),
		CHECKSUM_MODE_U8_LE)) {
		TPD_INFO("GT_brlD:%s: shrot result checksum err\n", __func__);
		return -EINVAL;
	}

	TPD_INFO("GT_brlD:%s :short flag 0x%02x, drv&drv:%d, sen&sen:%d, drv&sen:%d, drv/GNDVDD:%d, sen/GNDVDD:%d\n",
		__func__,
		test_result.result, test_result.drv_drv_num, test_result.sen_sen_num,
		test_result.drv_sen_num, test_result.drv_gnd_avdd_num, test_result.sen_gnd_avdd_num);


	if (!(test_result.result & 0x0F)) {
		TPD_INFO("GT_brlD:%s: >>>>> No shortcircut\n", __func__);
		return 0;
	}

	if (test_result.drv_drv_num)
		err |= gdix_check_tx_tx_shortcircut(cd, test_result.drv_drv_num);
	if (test_result.sen_sen_num)
		err |= gdix_check_rx_rx_shortcircut(cd, test_result.sen_sen_num);
	if (test_result.drv_sen_num)
		err |= gdix_check_tx_rx_shortcircut(cd, test_result.drv_sen_num);
	if (test_result.drv_gnd_avdd_num || test_result.sen_gnd_avdd_num)
		err |= gdix_check_gndvdd_shortcircut(cd);

	TPD_INFO("GT_brlD:%s : >>>>> short check return 0x%x\n", __func__, err);

	return err;
}

static int brl_shortcircut_test(struct seq_file *s,
				void *chip_data,
			    struct auto_testdata *goodix_testdata,
				struct test_item_info *p_test_item_info)
{
	struct chip_data_brl *cd = (struct chip_data_brl *)chip_data;
	struct goodix_ts_test *ts_test = cd->brl_test;
	struct ts_test_params *test_params = &ts_test->test_params;
	u16 test_time;
	u8 status;
	int retry;
	int ret;

	TPD_INFO("GT_brlD:%s IN\n", __func__);

	if (p_test_item_info->para_num && p_test_item_info->p_buffer[0]) {
		ts_test->is_item_support[TYPE_TEST5] = 1;
		/* store data to test_parms */
		test_params->short_threshold = p_test_item_info->p_buffer[1];
		test_params->r_drv_drv_threshold = p_test_item_info->p_buffer[2];
		test_params->r_drv_sen_threshold = p_test_item_info->p_buffer[3];
		test_params->r_sen_sen_threshold = p_test_item_info->p_buffer[4];
		test_params->r_drv_gnd_threshold = p_test_item_info->p_buffer[5];
		test_params->r_sen_gnd_threshold = p_test_item_info->p_buffer[6];
		test_params->avdd_value = p_test_item_info->p_buffer[7];
		test_params->max_drv_num = MAX_DRV_NUM_BRD;
		test_params->max_sen_num = MAX_SEN_NUM_BRD;
		test_params->drv_map = brl_d_drv_map;
		test_params->sen_map = brl_d_sen_map;
	} else {
		TPD_INFO("GT_brlD:%s: skip shortcircuit test\n", __func__);
		return 0;
	}

	ret = brl_short_test_prepare(cd);
	if (ret < 0) {
		TPD_INFO("GT_brlD:Failed enter short test mode\n");
		return RESULT_ERR;
	}

	/* get short test time */
	ret = goodix_reg_read(cd, SHORT_TEST_TIME_REG_BRD, (u8 *)&test_time, 2);
	if (ret < 0) {
		TPD_INFO("GT_brlD:Failed to get test_time, default %dms\n", DEFAULT_TEST_TIME_MS);
		test_time = DEFAULT_TEST_TIME_MS;
	} else {
		if (test_time > MAX_TEST_TIME_MS) {
			TPD_INFO("GT_brlD:test time too long %d > %d\n",
				test_time, MAX_TEST_TIME_MS);
			test_time = MAX_TEST_TIME_MS;
		}
		TPD_INFO("GT_brlD:get test time %dms\n", test_time);
	}

	/* start short circuit test */
	status = 0;
	ret = goodix_reg_write(cd, SHORT_TEST_RUN_REG, &status, 1);

	/* wait short test finish */
	if (test_time > 0)
		msleep(test_time);

	retry = 50;
	while (retry--) {
		ret = goodix_reg_read(cd, SHORT_TEST_STATUS_REG_BRD, &status, 1);
		if (!ret && status == SHORT_TEST_FINISH_FLAG)
			break;
		msleep(50);
	}
	if (retry < 0) {
		TPD_INFO("GT_brlD:short test failed, status:0x%02x\n", status);
		return RESULT_ERR;
	}

	/* start analysis short result */
	TPD_INFO("GT_brlD:short_test finished, start analysis\n");
	ret = brl_shortcircut_analysis(cd);
	if (ret < 0)
		return RESULT_ERR;

	ts_test->test_result[TYPE_TEST5] = GTP_TEST_OK;
	return 0;
}

static void brl_put_test_result(struct seq_file *s, void *chip_data,
				 struct auto_testdata *p_testdata,
				 struct test_item_info *p_test_item_info)
{
	uint8_t  data_buf[64];
	int i;
	/*save test fail result*/
	struct chip_data_brl *cd = (struct chip_data_brl *)chip_data;
	struct goodix_ts_test *ts_test = cd->brl_test;

	test_result_t short_test;

	memset(data_buf, 0, sizeof(data_buf));
	memset(&short_test, 0, sizeof(short_test));

	if (ts_test->rawdata.size && ts_test->is_item_support[TYPE_TEST2]) {
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			snprintf(data_buf, 64, "%s\n", "[RAW DATA]");
			tp_test_write(p_testdata->fp, p_testdata->length,
				      data_buf, strlen(data_buf), p_testdata->pos);
		}

		for (i = 0; i < ts_test->rawdata.size; i++) {
			if (!IS_ERR_OR_NULL(p_testdata->fp)) {
				snprintf(data_buf, 64, "%d,", ts_test->rawdata.data[i]);
				tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
					      p_testdata->pos);

				if (!((i + 1) % p_testdata->rx_num) && (i != 0)) {
					snprintf(data_buf, 64, "\n");
					tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
						      p_testdata->pos);
				}
			}
		}
	}

	if (ts_test->noisedata.size && ts_test->is_item_support[TYPE_TEST1]) {
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			snprintf(data_buf, 64, "\n%s\n", "[NOISE DATA]");
			tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
				      p_testdata->pos);
		}

		for (i = 0; i < ts_test->noisedata.size; i++) {
			if (!IS_ERR_OR_NULL(p_testdata->fp)) {
				sprintf(data_buf, "%d,", ts_test->noisedata.data[i]);
				tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
					      p_testdata->pos);

				if (!((i + 1) % p_testdata->rx_num) && (i != 0)) {
					snprintf(data_buf, 64, "\n");
					tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
						      p_testdata->pos);
				}
			}
		}
	}

	if (ts_test->deltadata.size && ts_test->is_item_support[TYPE_TEST3]) {
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			snprintf(data_buf, 64, "\n%s\n", "[DELTA DATA]");
			tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
				      p_testdata->pos);
		}

		for (i = 0; i < ts_test->deltadata.size; i++) {
			if (!IS_ERR_OR_NULL(p_testdata->fp)) {
				sprintf(data_buf, "%d,", ts_test->deltadata.data[i]);
				tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
					      p_testdata->pos);

				if (!((i + 1) % p_testdata->rx_num) && (i != 0)) {
					snprintf(data_buf, 64, "\n");
					tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
						      p_testdata->pos);
				}
			}
		}
	}

	if (ts_test->self_rawdata.size && ts_test->is_item_support[TYPE_TEST4]) {
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			snprintf(data_buf, 64, "\n%s\n", "[SELF RAW DATA]");
			tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
				      p_testdata->pos);
		}

		for (i = 0; i < ts_test->self_rawdata.size; i++) {
			if (!IS_ERR_OR_NULL(p_testdata->fp)) {
				sprintf(data_buf, "%d,", ts_test->self_rawdata.data[i]);
				tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
					      p_testdata->pos);
			}
		}

		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			sprintf(data_buf, "\n");
			tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
				      p_testdata->pos);
		}
	}

	if (!IS_ERR_OR_NULL(p_testdata->fp)) {
		snprintf(data_buf, 64, "\nTX:%d,RX:%d\n", p_testdata->tx_num, p_testdata->rx_num);
		tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
			      p_testdata->pos);
	}

	if (!IS_ERR_OR_NULL(p_testdata->fp)) {
		snprintf(data_buf, 32, "config id:0x%4X\n", cd->ic_info.version.config_id);
		tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
			      p_testdata->pos);
	}

	if (ts_test->self_rawdata.size && ts_test->is_item_support[TYPE_TEST5]) {
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			TPD_INFO("GT_brlD:%s :short flag 0x%02x, drv&drv:%d, sen&sen:%d, drv&sen:%d, drv/GNDVDD:%d, sen/GNDVDD:%d\n",
			__func__,
			short_test.result, short_test.drv_drv_num, short_test.sen_sen_num,
			short_test.drv_sen_num, short_test.drv_gnd_avdd_num, short_test.sen_gnd_avdd_num);

			snprintf(data_buf, 32, "short test : %02x-%u-%u-%u-%u-%u\n",
				short_test.result, short_test.drv_drv_num, short_test.sen_sen_num,
				short_test.drv_sen_num, short_test.drv_gnd_avdd_num, short_test.sen_gnd_avdd_num);

			tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
					  p_testdata->pos);
		}
	}

	for (i = 0; i < MAX_TEST_ITEMS; i++) {
		/* if have tested, show result */
		if (!ts_test->is_item_support[i])
			continue;

		TPD_INFO("GT_brlD:test_result_info %s: %s\n", test_item_name[i],
				ts_test->test_result[i] ? "pass" : "fail");
		if (!IS_ERR_OR_NULL(p_testdata->fp)) {
			snprintf(data_buf, 64, "%s: %s\n", test_item_name[i],
					ts_test->test_result[i] ? "pass" : "fail");
			tp_test_write(p_testdata->fp, p_testdata->length, data_buf, strlen(data_buf),
						p_testdata->pos);
		}
	}

	TPD_INFO("GT_brlD:%s exit\n", __func__);
	return;
}

static int brl_auto_test_endoperation(struct seq_file *s,
				void *chip_data,
				struct auto_testdata *p_testdata,
				struct test_item_info *p_test_item_info)
{
	struct chip_data_brl *cd = (struct chip_data_brl *)chip_data;
	struct goodix_fw_version ic_ver;
	struct goodix_ic_info ic_info;
	u32 fw_ver = 0;
	u8 cfg_ver = 0;
	int ret;

	TPD_INFO("GT_brlD:%s IN\n", __func__);

	goodix_reset(cd);
	if (cd->normal_cfg.length > 0) {
		ret = goodix_send_config(cd, cd->normal_cfg.data, cd->normal_cfg.length);
		if (ret < 0)
			TPD_INFO("GT_brlD:send normal config failed\n");
	}

	/*read device fw version*/
	brl_read_version(cd, &ic_ver);
	fw_ver = le32_to_cpup((__le32 *)&ic_ver.patch_vid[0]);
	brl_get_ic_info(cd, &ic_info);
	cfg_ver = ic_info.version.config_version;
	p_testdata->dev_tp_fw = (fw_ver << 8) + cfg_ver;

	enable_irq(cd->irq);

	brl_put_test_result(s, chip_data, p_testdata, p_test_item_info);

	kfree(cd->brl_test->rawdata.data);
	kfree(cd->brl_test->noisedata.data);
	kfree(cd->brl_test->self_rawdata.data);
	kfree(cd->brl_test->deltadata.data);
	kfree(cd->brl_test);

	goodix_check_bit_set(cd, SELF_TEST_ENABLE, false);

	return 0;
}

/*************** End of atuo test func***************************/
static struct goodix_auto_test_operations goodix_test_ops = {
	.auto_test_preoperation = brl_auto_test_preoperation,
	.test1 = brl_noisedata_test,
	.test2 = brl_capacitance_test,
	.test3 = brl_deltacapacitance,
	.test4 = brl_self_rawcapacitance,
	.test5 = brl_shortcircut_test,
	.test6 = brl_clk_test,
	.auto_test_endoperation = brl_auto_test_endoperation,
};

static struct engineer_test_operations goodix_engineer_test_ops = {
	.auto_test                  = goodix_auto_test,
};

struct goodix_proc_operations goodix_brl_proc_ops = {
	.goodix_config_info_read    = goodix_config_info_read,
};

static void init_motor_para(struct chip_data_brl *chip_info)
{
	chip_info->motor_max_max = chip_info->motor_coord_para[1];
	chip_info->motor_max_min = chip_info->motor_coord_para[1] - chip_info->motor_coord_para[2];
	chip_info->motor_min_max = chip_info->motor_coord_para[0] + chip_info->motor_coord_para[2];
	chip_info->motor_min_min = chip_info->motor_coord_para[0];

	chip_info->virtual_origin = chip_info->motor_dynamic_limit[0];
	chip_info->motor_offect = chip_info->motor_dynamic_limit[1];
	chip_info->motor_prevent = chip_info->motor_dynamic_limit[2];
	chip_info->dynamic_k_value = chip_info->virtual_origin / chip_info->motor_coord_para[1];

	TPD_INFO("GT_brlD:motor parse OK:[%3d~%3d,%3d~%3d],virtual_origin:%d,k:%d,offect:%d,prevent:%d",
			chip_info->motor_min_min, chip_info->motor_min_max,
			chip_info->motor_max_min, chip_info->motor_max_max,
			chip_info->virtual_origin, chip_info->dynamic_k_value,
			chip_info->motor_offect, chip_info->motor_prevent);

	chip_info->motor_max_limit = false;
	chip_info->motor_runing = false;
	chip_info->get_grip_coor = false;
	return;
}

static void init_goodix_chip_dts(struct device *dev, void *chip_data)
{
	int rc;
	int i = 0, cnt = 0, ret = 0;
	struct device_node *np;
	struct device_node *chip_np;
	int temp_array[MAX_SIZE];

	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	np = dev->of_node;

	chip_info->snr_read_support = of_property_read_bool(np, "snr_read_support");

	chip_np = of_get_child_by_name(np, "GT9916");
	if (!chip_np) {
		TPD_INFO("GT_brlD:%s: fail get child node for gt9916", __func__);
		return;
	} else {
		TPD_INFO("GT_brlD:%s: success get child node for gt9916", __func__);
	}

	chip_info->motor_coord_support = false;

	rc = of_property_read_u32_array(chip_np, "gt9916_get_motor_coord", temp_array, MOTOR_COORD_PARA);
	if (rc) {
		TPD_INFO("GT_brlD:%s: fail get gt9916_get_motor_coord %d\n", __func__, rc);
	} else {
		chip_info->motor_coord_support = true;
		for (i = 0; i < MOTOR_COORD_PARA; i++) {
			chip_info->motor_coord_para[i] = temp_array[i];
			cnt += chip_info->motor_coord_para[i];
		}
		if (0 == cnt) {
			TPD_INFO("GT_brlD:%s: invalid data!! stop to parse\n", __func__);
			goto OUT_ERR;
		}
		ret = goodix_set_coord_position(chip_info, 1);
	}

	rc = of_property_read_u32_array(chip_np, "gt9916_dynamic_limit", temp_array, MOTOR_DYNA_LIMIT);
	if (rc) {
		TPD_INFO("GT_brlD:%s: fail get gt9916_dynamic_limit %d\n", __func__, rc);
	} else {
		for (i = 0; i < MOTOR_DYNA_LIMIT; i++) {
			chip_info->motor_dynamic_limit[i] = temp_array[i];
			cnt += chip_info->motor_dynamic_limit[i];
		}
		if (0 == cnt) {
			TPD_INFO("GT_brlD:%s: invalid data!! stop to parse\n", __func__);
			goto OUT_ERR;
		}
	}

	if (chip_info->motor_coord_support == true)
		init_motor_para(chip_info);

	if (of_property_read_u32(chip_np, "support_gesture_type", &ret) < 0) {
		TPD_INFO("GT_brlD:%s: not get support_gesture_type, need set default\n", __func__);
	} else {
		chip_info->support_gesture_type = ret;
		TPD_INFO("GT_brlD:%s: ok get support_gesture_type, set -> %u\n", __func__, chip_info->support_gesture_type);
	}

	if (of_property_read_u32(chip_np, "pen_osc_frequency", &ret) < 0) {
		chip_info->pen_osc_frequency = DEF_OSC_FREQ_16K;
		TPD_INFO("GT_brlD:%s: pen osc freq is default:%u\n", __func__, chip_info->pen_osc_frequency);
	} else {
		if (ret == PEN_OSC_FREQ_64K)
			chip_info->pen_osc_frequency = PEN_OSC_FREQ_64K;
		else
			chip_info->pen_osc_frequency = DEF_OSC_FREQ_16K;
		TPD_INFO("GT_brlD:%s: change pen osc freq :%u\n", __func__, chip_info->pen_osc_frequency);
	}

	chip_info->rate_num = of_property_count_u32_elems(chip_np, "fps_report_rate");
	if (chip_info->rate_num < 0) {
		TPD_INFO("GT_brlD:%s: not fps_report_rate\n", __func__);
	} else {
		TPD_INFO("GT_brlD:%s: find %d rate config", __func__, chip_info->rate_num);

		if (of_property_read_u32_array(chip_np, "fps_report_rate", temp_array, chip_info->rate_num)) {
			TPD_INFO("GT_brlD:%s: fps_report_rate not specified\n", __func__);
			for (i = 0; i < chip_info->rate_num; i++) {
				chip_info->rate_array[i] = 0;
				TPD_INFO("GT_brlD:%s: set default -> fps_report_rate is: %u\n", __func__, chip_info->rate_array[i]);
			}
		} else {
			for (i = 0; i < chip_info->rate_num; i++) {
				chip_info->rate_array[i] = temp_array[i];
				chip_info->rate_num_support = true;
				TPD_INFO("GT_brlD:%s: set dts para -> fps_report_rate is: %u\n", __func__, chip_info->rate_array[i]);
			}
		}
	}

	if (chip_info->gt_temperature[0] > 0 && chip_info->gt_temperature[1] > 0) {
		if (of_property_read_u32_array(chip_np, "thermal_detect_support", temp_array, GOODIX_MAX_TEMP)) {
			TPD_INFO("GT_brlD:%s: not to change temperature zone\n", __func__);
		} else {
			chip_info->gt_temperature[0] = temp_array[0];
			chip_info->gt_temperature[1] = temp_array[1];
			TPD_INFO("GT_brlD:%s: change temperature pass -> [%u],[%u]\n",
				__func__, chip_info->gt_temperature[0], chip_info->gt_temperature[1]);
		}
	}

	return;
OUT_ERR:
	chip_info->motor_coord_support = false;
	return;
}

static int goodix_gt9916_ts_probe(struct spi_device *spi)
{
	struct chip_data_brl *chip_info = NULL;
	struct touchpanel_data *ts = NULL;
	int ret = -1;

	TPD_INFO("GT_brlD:Goodix driver version: %s\n", GOODIX_DRIVER_VERSION);

	/* 2. Alloc chip_info */
	chip_info = kzalloc(sizeof(struct chip_data_brl), GFP_KERNEL);
	if (chip_info == NULL) {
		TPD_INFO("GT_brlD:chip info kzalloc error\n");
		ret = -ENOMEM;
		return ret;
	}

	/* 3. Alloc common ts */
	ts = common_touch_data_alloc();
	if (ts == NULL) {
		TPD_INFO("GT_brlD:ts kzalloc error\n");
		goto ts_malloc_failed;
	}

	/* 4. alloc touch data space */
	chip_info->touch_data = kzalloc(MAX_GT_IRQ_DATA_LENGTH, GFP_KERNEL);
	if (chip_info->touch_data == NULL) {
		TPD_INFO("GT_brlD:touch_data kzalloc error\n");
		goto err_register_driver;
	}

	chip_info->edge_data = kzalloc(MAX_GT_EDGE_DATA_LENGTH, GFP_KERNEL);
	if (chip_info->edge_data == NULL) {
		TPD_INFO("GT_brlD:edge_data kzalloc error\n");
		goto err_touch_data_alloc;
	}

	/* init spi_device */
	spi->mode          = SPI_MODE_0;
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret) {
		TPD_INFO("GT_brlD:failed set spi mode, %d\n", ret);
		goto err_edge_data_alloc;
	}

	ts->dev = &spi->dev;
	ts->s_client = spi;
	ts->irq = spi->irq;
	ts->chip_data = chip_info;
	spi_set_drvdata(spi, ts);
	/* add input_dev info */
	ts->id.bustype = BUS_SPI;
	ts->id.vendor = GOODIX;
	ts->id.product = GT9916;

	chip_info->hw_res = &ts->hw_res;
	chip_info->s_client = spi;
	chip_info->goodix_ops = &goodix_brl_proc_ops;
	ts->com_test_data.chip_test_ops = &goodix_test_ops;
	ts->engineer_ops = &goodix_engineer_test_ops;
	ts->debug_info_ops = &debug_info_proc_ops;
	/* 6. file_operations callbacks binding */
	ts->ts_ops = &goodix_ops;
	chip_info->probe_complete = false;
	chip_info->rate_num_support = false;
	chip_info->default_rate_set = true;
	ts->bus_type = TP_BUS_SPI;

	init_goodix_chip_dts(ts->dev, chip_info);

	/* 8. register common touch device */
	ret = register_common_touch_device(ts);
	if (ret < 0) {
		goto err_edge_data_alloc;
	}

	/* 9. create goodix tool node */
	gtx8_init_tool_node(ts, &chip_info->rawdiff_mode);

	chip_info->kernel_grip_support = ts->kernel_grip_support;
	chip_info->tp_index = ts->tp_index;
	chip_info->max_x = ts->resolution_info.max_x;
	chip_info->max_y = ts->resolution_info.max_y;
	chip_info->pen_support = ts->pen_support;
	chip_info->pen_support_opp = ts->pen_support_opp;
	chip_info->game_enable =      false;
	chip_info->gesture_enable =   false;
	chip_info->pen_enable =       false;
	chip_info->sleep_enable =     false;
	chip_info->check_now_state =       0;
	chip_info->check_old_state =       0;
	chip_info->check_chg_state =       0;
	chip_info->pen_state =        PEN_UP;
	chip_info->pen_input_state =  PEN_DOWN;
	chip_info->pen_min_x = 660;
	chip_info->pen_max_x = 18400;
	chip_info->pen_min_y = 660;
	chip_info->pen_max_y = 24180;
	chip_info->ts = ts;
	/* init pencil default status*/
	ts->is_pen_connected =        false;

	pen_init_debug_node(chip_info);

	mutex_init(&chip_info->debug_lock);

	if(chip_info->pen_support) {
		TPD_INFO("GT_brlD:%s, start check in pen hrtimer\n", __func__);
		hrtimer_init(&chip_info->check_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		chip_info->check_time = ktime_set(GOODIX_CHECK_HRTIMER_S, GOODIX_CHECK_HRTIMER_NS);
		chip_info->check_hrtimer.function = goodix_check_hrtimer;
		chip_info->check_hrtimer_over = NO_CHECK_TIME;
		INIT_WORK(&chip_info->check_pendown_work, pen_checkdown_work);
	}

	goodix_esd_check_enable(chip_info, true);
	ts->tp_suspend_order = TP_LCD_SUSPEND;

	if (chip_info->ts->temperature_detect_support ||
		chip_info->ts->temperature_detect_shellback_support) {
		chip_info->gt_temperature[0] = 5;
		chip_info->gt_temperature[1] = 15;
		chip_info->gt_temp_enable    = true;
		chip_info->temp_recorder_cnt = CNT_CLR;
		TPD_INFO("GT_brlD:%s:gt_temp_para set default control->[%u][%u]\n",
			__func__, chip_info->gt_temperature[0], chip_info->gt_temperature[1]);
	} else {
		chip_info->gt_temperature[0] = NO_DATA;
		chip_info->gt_temperature[1] = NO_DATA;
		chip_info->gt_temp_enable    = false;
		chip_info->temp_recorder_cnt = CNT_CLR;
		TPD_INFO("GT_brlD:%s:failed set temperature control->[%u][%u]\n",
			__func__, chip_info->gt_temperature[0], chip_info->gt_temperature[1]);
	}

	chip_info->resolution_ratio = \
		chip_info->ts->resolution_info.max_x / chip_info->ts->resolution_info.LCD_WIDTH;

	TPD_INFO("GT_brlD:%s,probe normal end\n", __func__);
	chip_info->probe_complete = true;
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

	TPD_INFO("GT_brlD:%s, probe failed.\n", __func__);
	return ret;
}

static int goodix_ts_pm_suspend(struct device *dev)
{
#ifndef CONFIG_ARCH_QTI_VM
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("GT_brlD:%s: is called\n", __func__);
	tp_pm_suspend(ts);
#endif
	return 0;
}

static int goodix_ts_pm_resume(struct device *dev)
{
#ifndef CONFIG_ARCH_QTI_VM
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("GT_brlD:%s is called\n", __func__);
	tp_pm_resume(ts);
#endif
	return 0;
}

static void goodix_gt9916_tp_shutdown(struct spi_device *s_client)
{
	struct touchpanel_data *ts = spi_get_drvdata(s_client);

	TPD_INFO("GT_brlD:%s is called\n", __func__);

	tp_shutdown(ts);
}

static int __maybe_unused goodix_gt9916_ts_remove(struct spi_device *s_client)
{
	struct touchpanel_data *ts = spi_get_drvdata(s_client);
	struct chip_data_brl *chip_info = NULL;
	if (!ts) {
		TPD_INFO("GT_brlD:%s spi_get_drvdata(s_client) is null.\n", __func__);
		return -EINVAL;
	}

	chip_info = (struct chip_data_brl *)ts->chip_data;

	TPD_INFO("GT_brlD:%s is called\n", __func__);

	if (chip_info->pen_healthinfo_node == true) {
		TPD_INFO("GT_brlD:%s need remove healthinfo node\n", __func__);
		remove_proc_entry(PENCIL_DATA_PATH, chip_info->ts->prEntry_tp);
	}

	goodix_remove_proc(ts);
	gtx8_deinit_tool_node(ts);
	unregister_common_touch_device(ts);
	common_touch_data_free(ts);
	mutex_destroy(&chip_info->debug_lock);

	tp_kfree((void **)&chip_info);

	spi_set_drvdata(s_client, NULL);

	return 0;
}

static const struct dev_pm_ops gt9916_pm_ops = {
	.suspend = goodix_ts_pm_suspend,
	.resume = goodix_ts_pm_resume,
};

static const struct spi_device_id gt9916_id[] = {
	{ TPD_DEVICE, 0},
	{ }
};

static struct of_device_id gt9916_match_table[] = {
	{ .compatible = TPD_DEVICE, },
	{ .compatible = "goodix-gt9916", },
	{ }
};

static struct spi_driver gt9916_ts_driver = {
	.probe = goodix_gt9916_ts_probe,
	.remove = goodix_gt9916_ts_remove,
	.id_table   = gt9916_id,
	.shutdown   = goodix_gt9916_tp_shutdown,
	.driver = {
		.name   = TPD_DEVICE,
		.of_match_table = gt9916_match_table,
		.pm = &gt9916_pm_ops,
	},
};

/***********************Start of module init and exit****************************/
int __init tp_driver_init_gt9916(void)
{
	TPD_INFO("GT_brlD:%s is called\n", __func__);

	if (!tp_judge_ic_match(TPD_DEVICE)) {
		TPD_INFO("GT_brlD:%s not match\n", __func__);
		goto OUT;
	}

	if (spi_register_driver(&gt9916_ts_driver) != 0) {
		TPD_INFO("GT_brlD:%s : unable to add spi driver.\n", __func__);
		goto OUT;
	}

OUT:
	return 0;
}

void __exit tp_driver_exit_gt9916(void)
{
	TPD_INFO("GT_brlD:%s : Core layer exit", __func__);
	spi_unregister_driver(&gt9916_ts_driver);
}

#ifdef CONFIG_TOUCHPANEL_LATE_INIT
late_initcall(tp_driver_init_gt9916);
#else
module_init(tp_driver_init_gt9916);
#endif

module_exit(tp_driver_exit_gt9916);
/***********************End of module init and exit*******************************/
MODULE_AUTHOR("Goodix, Inc.");
MODULE_DESCRIPTION("GTP Touchpanel Driver");
MODULE_LICENSE("GPL v2");
