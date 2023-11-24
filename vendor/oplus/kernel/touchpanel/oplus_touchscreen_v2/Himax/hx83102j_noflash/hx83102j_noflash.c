/* SPDX-License-Identifier: GPL-2.0-only*/
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#include<mt-plat/mtk_boot_common.h>
#else
#include <soc/oplus/system/boot_mode.h>
#endif

#include <linux/spi/spi.h>
#include "hx83102j_noflash.h"

#define OPLUS17001TRULY_TD4322_1080P_CMD_PANEL 29

/*******Part0:LOG TAG Declear********************/

#ifdef TPD_DEVICE
#undef TPD_DEVICE
#define TPD_DEVICE "himax,hx83102j_nf"
#else
#define TPD_DEVICE "himax,hx83102j_nf"
#endif

#ifdef PEN_BATTERY_LOW_NOTIFIER
#define TIME_24H_TO_SECONDS 86400     /*24h-----86400  12h------43200  6h-----21300*/
#define DEBUG_TIME_TO_SECONDS 60     /*1MIN-----60 */
#define PEN_BATTERY_LOW_NAME  "pen_battery_low_notifier"

#endif

static int hx83102j_read_FW_status(struct chip_data_hx83102j *chip_info, uint8_t *state_addr, uint8_t *tmp_addr);
static int hx83102j_sram_write_crc_check(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry, uint8_t *addr, int strt_idx, uint32_t len);
#ifdef PEN_BATTERY_LOW_NOTIFIER
static int pen_battery_low_init(struct handwrite_pen_battery *pen_batlow);
static void pen_battery_low_notifier_env(struct handwrite_pen_battery *pen_batlow);
/*static void clear_pen_batlow_flag(struct handwrite_pen_battery *pen_batlow);*/
static void pen_battery_low_exit(struct handwrite_pen_battery *pen_batlow);
#endif
/*******Part0: SPI Interface***************/
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
static const struct mtk_chip_config hx_spi_ctrdata = {
	.sample_sel = 0,
	.cs_setuptime = 100,
	.cs_holdtime = 60,
	.cs_idletime = 0,
	.tick_delay = 0,
};
#else
static const struct mt_chip_conf hx_spi_ctrdata = {
	.setuptime = 25,
	.holdtime = 25,
	.high_time = 3, /* 16.6MHz */
	.low_time = 3,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,

	.cpol = 0,
	.cpha = 0,

	.rx_mlsb = 1,
	.tx_mlsb = 1,

	.tx_endian = 0,
	.rx_endian = 0,

	.com_mod = DMA_TRANSFER,

	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif


static int32_t spi_read_write_himax(struct spi_device *client, uint8_t *buf, size_t len,
		       uint8_t *rbuf, SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};
	u8 *tx_buf = NULL;
	u8 *rx_buf = NULL;
	int status;

	switch (rw) {
	case SPIREAD:
		tx_buf = tp_devm_kzalloc(&client->dev, 3, GFP_KERNEL | GFP_DMA);

		if (!tx_buf) {
			status = -ENOMEM;
			goto spi_out;
		}

		memset(tx_buf, 0xFF, 3);
		memcpy(tx_buf, buf, 3);
		rx_buf = tp_devm_kzalloc(&client->dev, len + 3 + DUMMY_BYTES, GFP_KERNEL | GFP_DMA);

		if (!rx_buf) {
			status = -ENOMEM;
			goto spi_out;
		}

		memset(rx_buf, 0xFF, len + DUMMY_BYTES);
		t.tx_buf = tx_buf;
		t.rx_buf = rx_buf;
		t.len    = (len + 3 + DUMMY_BYTES);
		break;

	case SPIWRITE:
		tx_buf = tp_devm_kzalloc(&client->dev, len, GFP_KERNEL | GFP_DMA);

		if (!tx_buf) {
			status = -ENOMEM;
			goto spi_out;
		}

		memcpy(tx_buf, buf, len);
		t.tx_buf = tx_buf;
		break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);


	status = spi_sync(client, &m);

	if (status == 0) {
		if (rw == SPIREAD) {
			memcpy(rbuf, rx_buf+3, len);
		}
	}

spi_out:

	if (tx_buf) {
		tp_devm_kfree(&client->dev, (void **)&tx_buf, len + 3 + DUMMY_BYTES);
	}

	if (rx_buf) {
		tp_devm_kfree(&client->dev, (void **)&rx_buf, len + DUMMY_BYTES);
	}

	return status;
}


static ssize_t hx83102j_spi_sync(struct spi_device *spi, struct spi_message *message)
{
	int status;

	status = spi_sync(spi, message);

	return status;
}

static int hx83102j_spi_write(struct chip_data_hx83102j *chip_info, uint8_t *buf, uint32_t len)
{
	int status = 0;
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};
	u8 *tx_buf = NULL;


	tx_buf = tp_devm_kzalloc(&chip_info->hx_spi->dev, len, GFP_KERNEL | GFP_DMA);

	if (!tx_buf) {
		status = -ENOMEM;
		goto spi_out;
	}

	memcpy(tx_buf, buf, len);
	t.tx_buf = tx_buf;


	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	hx83102j_spi_sync(chip_info->hx_spi, &m);


spi_out:

	if (tx_buf) {
		tp_devm_kfree(&chip_info->hx_spi->dev, (void **)&tx_buf, len + DUMMY_BYTES);
	}

	return status;
}

static int hx83102j_bus_read(struct chip_data_hx83102j *chip_info, uint8_t command, uint32_t length, uint8_t *data)
{
	int result = 0;
	uint8_t spi_format_buf[3];


	mutex_lock(&(chip_info->spi_lock));
	spi_format_buf[0] = 0xF3;
	spi_format_buf[1] = command;
	spi_format_buf[2] = 0x00;


	spi_read_write_himax(chip_info->hx_spi, spi_format_buf, length, data, SPIREAD);


	mutex_unlock(&(chip_info->spi_lock));

	return result;
}

static int hx83102j_bus_write_with_addr(struct chip_data_hx83102j *chip_info, uint8_t command, uint8_t *addr, uint8_t *data, uint32_t length)
{
	int result = 0;
	uint8_t *spi_format_buf = NULL;
	int alloc_size = 0;
	uint8_t offset = 0;
	uint32_t tmp_len = length;


	alloc_size = length;

	mutex_lock(&(chip_info->spi_lock));
	if (spi_format_buf == NULL) {
		spi_format_buf = kzalloc((alloc_size + 2) * sizeof(uint8_t), GFP_KERNEL);
	}

	if (spi_format_buf == NULL) {
		TPD_INFO("%s: Can't allocate enough buf\n", __func__);
		goto FAIL;
	}
	spi_format_buf[0] = 0xF2;
	spi_format_buf[1] = command;
	offset = 2;
	if (addr != NULL) {
		memcpy(spi_format_buf+offset, addr, 4);
		offset += 4;
		tmp_len -= 4;
	}

	if (data != NULL)
		memcpy(spi_format_buf+offset, data, tmp_len);
#ifdef CONFIG_VMAP_STACK
	result = hx83102j_spi_write(chip_info, spi_format_buf, length + 2);
#else
	result = hx83102j_spi_write(chip_info, spi_format_buf, length + 2);
#endif /*CONFIG_VMAP_STACK*/

FAIL:
	if(spi_format_buf != NULL) {
		kfree(spi_format_buf);
		spi_format_buf = NULL;
	}

	mutex_unlock(&(chip_info->spi_lock));
	return result;
}

static int hx83102j_bus_write(struct chip_data_hx83102j *chip_info, uint8_t command, uint32_t length, uint8_t *data)
{
	/* uint8_t spi_format_buf[length + 2]; */
	int result = 0;
	static uint8_t *spi_format_buf;
	int alloc_size = 0;
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	alloc_size = MAX_TRANS_SZ + 14;
#else
	alloc_size = 49156;
#endif
	mutex_lock(&(chip_info->spi_lock));

	if (spi_format_buf == NULL) {
		spi_format_buf = kzalloc((alloc_size + 2) * sizeof(uint8_t), GFP_KERNEL);
	}

	if (spi_format_buf == NULL) {
		TPD_INFO("%s: Can't allocate enough buf\n", __func__);
		return -ENOMEM;
	}

	spi_format_buf[0] = 0xF2;
	spi_format_buf[1] = command;

	memcpy((uint8_t *)(&spi_format_buf[2]), data, length);
#ifdef CONFIG_VMAP_STACK
	result = hx83102j_spi_write(chip_info, spi_format_buf, length + 2);
#else
	result = hx83102j_spi_write(chip_info, spi_format_buf, length + 2);
#endif  /*CONFIG_VMAP_STACK*/

	if(spi_format_buf != NULL) {
		kfree(spi_format_buf);
		spi_format_buf = NULL;
	}
	mutex_unlock(&(chip_info->spi_lock));

	return result;
}

/*******Part1: Function Declearation*******/
static int hx83102j_power_control(void *chip_data, bool enable);
static int hx83102j_get_chip_info(void *chip_data);
static int hx83102j_mode_switch(void *chip_data, work_mode mode, int flag);
static uint32_t hx83102j_hw_check_crc(struct chip_data_hx83102j *chip_info, uint8_t *start_addr, int reload_length);
static fw_check_state hx83102j_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data);
static void hx83102j_read_FW_ver(struct chip_data_hx83102j *chip_info);
#ifdef HX_RST_PIN_FUNC
static int hx83102j_resetgpio_set(struct hw_resource *hw_res, bool on);
#endif
/*******Part2:Call Back Function implement*******/

/* add for himax */
static void hx83102j_flash_write_burst(struct chip_data_hx83102j *chip_info, uint8_t *reg_byte, uint8_t *write_data)
{
	uint8_t data_byte[8];
	int i = 0;
	int j = 0;

	for (i = 0; i < 4; i++) {
		data_byte[i] = reg_byte[i];
	}

	for (j = 4; j < 8; j++) {
		data_byte[j] = write_data[j - 4];
	}

	if (hx83102j_bus_write(chip_info, 0x00, 8, data_byte) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return;
	}
}

static void hx83102j_flash_write_burst_length(struct chip_data_hx83102j *chip_info, uint8_t *reg_byte, uint8_t *write_data, int length)
{
	uint8_t *data_byte;
	data_byte = kzalloc(sizeof(uint8_t) * (length + 4), GFP_KERNEL);

	if (data_byte == NULL) {
		TPD_INFO("%s: Can't allocate enough buf\n", __func__);
		return;
	}

	memcpy(data_byte, reg_byte, 4); /* assign addr 4bytes */
	memcpy(data_byte + 4, write_data, length); /* assign data n bytes */

	if (hx83102j_bus_write(chip_info, 0, length + 4, data_byte) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
	}
	if (data_byte)
		kfree(data_byte);
}

static void hx83102j_burst_enable(struct chip_data_hx83102j *chip_info, uint8_t auto_add_4_byte)
{
	uint8_t tmp_data[4];
	tmp_data[0] = 0x31;

	if (hx83102j_bus_write(chip_info, 0x13, 1, tmp_data) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return;
	}

	tmp_data[0] = (0x10 | auto_add_4_byte);
	if (hx83102j_bus_write(chip_info, 0x0D, 1, tmp_data) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return;
	}
}

static void hx83102j_register_read(struct chip_data_hx83102j *chip_info, uint8_t *read_addr, int read_length, uint8_t *read_data, bool cfg_flag)
{
	uint8_t tmp_data[4];
	int ret;
	if(chip_info->cfg_flag == false) {
		if(read_length > 256) {
			TPD_INFO("%s: read len over 256!\n", __func__);
			return;
		}
		if (read_length > 4) {
			hx83102j_burst_enable(chip_info, 1);
		} else {
			hx83102j_burst_enable(chip_info, 0);
		}

		tmp_data[0] = read_addr[0];
		tmp_data[1] = read_addr[1];
		tmp_data[2] = read_addr[2];
		tmp_data[3] = read_addr[3];
		ret = hx83102j_bus_write(chip_info, 0x00, 4, tmp_data);
		if (ret < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return;
		}
		tmp_data[0] = 0x00;
		ret = hx83102j_bus_write(chip_info, 0x0C, 1, tmp_data);
		if (ret < 0) {
			TPD_INFO("%s: bus1 access fail!\n", __func__);
			return;
		}

		if (hx83102j_bus_read(chip_info, 0x08, read_length, read_data) < 0) {
			TPD_INFO("%s: bus2 access fail!\n", __func__);
			return;
		}
		if (read_length > 4) {
			hx83102j_burst_enable(chip_info, 0);
		}
	} else if (chip_info->cfg_flag == true) {
		if(hx83102j_bus_read(chip_info, read_addr[0], read_length, read_data) < 0) {
			TPD_INFO("%s: bus3 access fail!\n", __func__);
			return;
		}
	} else {
		TPD_INFO("%s: chip_info->cfg_flag = %d, value is wrong!\n", __func__, chip_info->cfg_flag);
		return;
	}
}

static void hx83102j_register_write(struct chip_data_hx83102j *chip_info, uint8_t *write_addr, int write_length, uint8_t *write_data, bool cfg_flag)
{
	int i = 0;
	int address = 0;
	if (chip_info->cfg_flag == false) {
		address = (write_addr[3] << 24) + (write_addr[2] << 16) + (write_addr[1] << 8) + write_addr[0];

		for (i = address; i < address + write_length; i++) {
			if (write_length > 4) {
				hx83102j_burst_enable(chip_info, 1);
			} else {
				hx83102j_burst_enable(chip_info, 0);
			}
			hx83102j_flash_write_burst_length(chip_info, write_addr, write_data, write_length);
		}
	} else if (chip_info->cfg_flag == true) {
		if (hx83102j_bus_write(chip_info, write_addr[0], write_length, write_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return;
		}
	} else {
		TPD_INFO("%s: chip_info->cfg_flag = %d, value is wrong!\n", __func__, chip_info->cfg_flag);
		return;
	}
}

static int hx83102j_mcu_register_write(struct chip_data_hx83102j *chip_info, uint8_t *write_addr, uint32_t write_length, uint8_t *write_data, uint8_t cfg_flag)
{
	int total_read_times = 0;
	int max_bus_size = 128, test = 0;
	int total_size_temp = 0;
	int address = 0;
	int i = 0;

	uint8_t tmp_addr[4];
	uint8_t *tmp_data;

	total_size_temp = write_length;
	TPD_DETAIL("%s, Entering - total write size=%d\n", __func__, total_size_temp);

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	if (write_length > MAX_TRANS_SZ) {
		max_bus_size = MAX_TRANS_SZ;
	} else {
		max_bus_size = write_length;
	}

#else
	if (write_length > 49152) {
		max_bus_size = 49152;
	} else {
		max_bus_size = write_length;
	}
#endif

	hx83102j_burst_enable(chip_info, 1);

	tmp_addr[3] = write_addr[3];
	tmp_addr[2] = write_addr[2];
	tmp_addr[1] = write_addr[1];
	tmp_addr[0] = write_addr[0];
	TPD_DEBUG("%s, write addr = 0x%02X%02X%02X%02X\n", __func__,
		tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);

	tmp_data = kzalloc(sizeof(uint8_t) * max_bus_size, GFP_KERNEL);
	if (tmp_data == NULL) {
		TPD_INFO("%s: Can't allocate enough buf \n", __func__);
		return -1;
	}

	if (total_size_temp % max_bus_size == 0) {
		total_read_times = total_size_temp / max_bus_size;
	} else {
		total_read_times = total_size_temp / max_bus_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		if (total_size_temp >= max_bus_size) {
			memcpy(tmp_data, write_data + (i * max_bus_size), max_bus_size);
			hx83102j_flash_write_burst_length(chip_info, tmp_addr, tmp_data, max_bus_size);

			total_size_temp = total_size_temp - max_bus_size;
		} else {
			test = total_size_temp % max_bus_size;
			memcpy(tmp_data, write_data + (i * max_bus_size), test);
			TPD_DEBUG("last total_size_temp=%d\n", total_size_temp % max_bus_size);

			hx83102j_flash_write_burst_length(chip_info, tmp_addr, tmp_data, max_bus_size);
		}

		address = ((i + 1) * max_bus_size);
		tmp_addr[0] = write_addr[0] + (uint8_t) ((address) & 0x00FF);

		if (tmp_addr[0] <  write_addr[0]) {
			tmp_addr[1] = write_addr[1] + (uint8_t) ((address >> 8) & 0x00FF) + 1;
		} else {
			tmp_addr[1] = write_addr[1] + (uint8_t) ((address >> 8) & 0x00FF);
		}

		udelay(100);
	}
	TPD_DETAIL("%s, End \n", __func__);
	if (tmp_data)
		kfree(tmp_data);
	return 0;
}
static bool hx83102j_mcu_sys_reset(struct chip_data_hx83102j *chip_info)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int retry = 0;

	do {
		/*===========================================
			0x31 ==> 0x27
		===========================================*/
		tmp_data[0] = 0x27;
		if (hx83102j_bus_write(chip_info, 0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*===========================================
			0x32 ==> 0x95
		===========================================*/
		tmp_data[0] = 0x95;
		if (hx83102j_bus_write(chip_info, 0x32, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*===========================================
			0x31 ==> 0x00
		===========================================*/
		tmp_data[0] = 0x00;
		if (hx83102j_bus_write(chip_info, 0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		usleep_range(1000, 1001);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x98;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_addr[0] = 0xE4;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
	} while ((tmp_data[1] != 0x02 || tmp_data[0] != 0x00) && retry++ < 5);

	return true;
}
static void hx_check_fw_status(struct chip_data_hx83102j *chip_info)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	    tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 0x900000A8, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xE4;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 0x900000E4, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xF8;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 0x900000F8, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xE8;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 0x900000E8, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x40;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 0x10007f40, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
}

static bool hx83102j_sense_off(struct chip_data_hx83102j *chip_info)
{
	uint8_t cnt = 0;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	do {
		if (cnt == 0 || (tmp_data[0] != 0xA5 && tmp_data[0] != 0x00 && tmp_data[0] != 0x87)) {
			tmp_addr[3] = 0x90;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x5C;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0xA5;
			hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
		}

		usleep_range(20000, 20001);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		if (tmp_data[0] != 0x05) {
			TPD_INFO("%s: it already in safe mode=0x%02X\n", __func__, tmp_data[0]);
		break;
		}

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x40;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_DEBUG("%s: 10007F40: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_DEBUG("%s: 10000000: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x04;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_DEBUG("%s: 10007F04: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x02;
		tmp_addr[1] = 0x04;
		tmp_addr[0] = 0xF4;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_DEBUG("%s: 800204B4: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		usleep_range(20000, 20001);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x5C;
			hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

		cnt++;
		TPD_DEBUG("%s: save mode lock cnt = %d, data[0] = %2X!\n", __func__, cnt, tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (cnt < 10));


	cnt = 0;

	do {
		/*===========================================
			0x31 ==> 0x27
		===========================================*/
		tmp_data[0] = 0x27;
		if (hx83102j_bus_write(chip_info, 0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}
		/*===========================================
			0x32 ==> 0x95
		===========================================*/
		tmp_data[0] = 0x95;
		if (hx83102j_bus_write(chip_info, 0x32, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return false;
		}

		/*======================
			Check enter_save_mode
		======================*/
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

		TPD_DEBUG("%s: Check enter_save_mode data[0]=%X \n", __func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/*=====================================
				Reset TCON
			=====================================*/
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x04;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
			usleep_range(1000, 1001);
			return true;
		} else {
			usleep_range(10000, 10001);
#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(chip_info->hw_res, false);
		usleep_range(5000, 5001);
		hx83102j_resetgpio_set(chip_info->hw_res, true);
		usleep_range(5000, 5001);
#else
		hx83102j_mcu_sys_reset(chip_info);
#endif
		}
	} while (cnt++ < 15);

	return false;
}

static void hx83102j_interface_on(struct chip_data_hx83102j *chip_info)
{
	uint8_t tmp_data[5];
	uint8_t tmp_data2[2];
	int cnt = 0;

	/*Read a dummy register to wake up I2C.*/
	if (hx83102j_bus_read(chip_info, 0x08, 4, tmp_data) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return;
	}

	do {
		/*===========================================
			Enable continuous burst mode : 0x13 ==> 0x31
		===========================================*/
		tmp_data[0] = 0x31;
		if (hx83102j_bus_write(chip_info, 0x13, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return;
		}
		/*===========================================
			AHB address auto +4 : 0x0D ==> 0x11
			Do not AHB address auto +4 : 0x0D ==> 0x10
		===========================================*/
		tmp_data[0] = (0x10);
		if (hx83102j_bus_write(chip_info, 0x0D, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
			return;
		}

		/* Check cmd*/
		hx83102j_bus_read(chip_info, 0x13, 1, tmp_data);
		hx83102j_bus_read(chip_info, 0x0D, 1, tmp_data2);

		if (tmp_data[0] == 0x31 && tmp_data2[0] == 0x10) {
			break;
		}
		usleep_range(1000, 1001);
	} while (++cnt < 10);

	if (cnt > 0) {
		TPD_INFO("%s:Polling burst mode: %d times", __func__, cnt);
	}
}

static void hx83102j_diag_register_set(struct chip_data_hx83102j *chip_info, uint8_t diag_command)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	TPD_DEBUG("diag_command = %d\n", diag_command);

	hx83102j_interface_on(chip_info);

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x72;
	tmp_addr[0] = 0xEC;


	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = diag_command;
	hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

	hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
	TPD_DEBUG("%s: tmp_data[3] = 0x%02X, tmp_data[2] = 0x%02X, tmp_data[1] = 0x%02X, tmp_data[0] = 0x%02X!\n",
			 __func__, tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
}


static void hx83102j_hx83102j_reload_to_active(struct chip_data_hx83102j *chip_info)
{
	uint8_t addr[4] = {0};
	uint8_t data[4] = {0};
	uint8_t retry_cnt = 0;

	addr[3] = 0x90;
	addr[2] = 0x00;
	addr[1] = 0x00;
	addr[0] = 0x48;

	do {
		data[3] = 0x00;
		data[2] = 0x00;
		data[1] = 0x00;
		data[0] = 0xEC;
		hx83102j_register_write(chip_info, addr, 4, data, 0);
		usleep_range(1000, 1001);
		hx83102j_register_read(chip_info, addr, 4, data, 0);
		TPD_INFO("%s: data[1]=%d, data[0]=%d, retry_cnt=%d\n", __func__, data[1], data[0], retry_cnt);
		retry_cnt++;
	} while ((data[1] != 0x01 || data[0] != 0xEC) && retry_cnt < HIMAX_REG_RETRY_TIMES);
}

static void hx83102j_en_hw_crc(struct chip_data_hx83102j *chip_info, int en)
{
	uint8_t addr[DATA_LEN_4] = {0};
	uint8_t data[DATA_LEN_4] = {0};
	uint8_t wrt_data[DATA_LEN_4];
	uint8_t retry_cnt = 0;
	addr[3] = 0x80;
	addr[2] = 0x01;
	addr[1] = 0x00;
	addr[0] = 0x00;
	if (en) {
		data[3] = 0x00;
		data[2] = 0x00;
		data[1] = 0xEC;
		data[0] = 0xCE;
	} else {
		data[3] = 0x00;
		data[2] = 0x00;
		data[1] = 0x00;
		data[0] = 0x00;
	}
	do {
		wrt_data[3] = data[3];
		wrt_data[2] = data[2];
		wrt_data[1] = data[1];
		wrt_data[0] = data[0];
		hx83102j_register_write(chip_info, addr, DATA_LEN_4, wrt_data, 0);
		usleep_range(1000, 1100);
		hx83102j_register_read(chip_info, addr, DATA_LEN_4, data, 0);
		TPD_INFO("%s:opt hw crc data[1]=%d, data[0]=%d, retry_cnt=%d\n", __func__,
				data[1], data[0], retry_cnt);
		retry_cnt++;
	} while ((data[1] != wrt_data[1]
		|| data[0] != wrt_data[0])
		&& retry_cnt < HIMAX_REG_RETRY_TIMES);
}

#if defined(HX_CODE_OVERLAY)
static void hx83102j_sense_on_by_sys_rst(struct chip_data_hx83102j *chip_info)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	hx83102j_interface_on(chip_info);
	tmp_addr[3] = 0x90;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x5C;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);


	tmp_addr[3] = 0x90;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x18;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x55;
	hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

#ifdef HX_OPT_HW_CRC
	hx83102j_en_hw_crc(chip_info, 1);
#endif
	hx83102j_hx83102j_reload_to_active(chip_info);
}
#endif

static void hx83102j_sense_on(struct chip_data_hx83102j *chip_info, uint8_t FlashMode)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	TPD_DETAIL("Enter %s  \n", __func__);

	hx83102j_interface_on(chip_info);
	tmp_addr[3] = 0x90;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x5C;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

	usleep_range(10000, 11000);


	if (!FlashMode) {
#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(chip_info->hw_res, false);
		usleep_range(5000, 5001);
		hx83102j_resetgpio_set(chip_info->hw_res, true);
		usleep_range(5000, 5001);
#else
		chip_info->g_core_fp.fp_sys_reset(chip_info);
#endif
		chip_info->hx_esd_reset_activate = 1;
	} else {
		/* reset code*/
		tmp_data[0] = 0x00;
		if (hx83102j_bus_write(chip_info, 0x31, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
		}
		if (hx83102j_bus_write(chip_info, 0x32, 1, tmp_data) < 0) {
			TPD_INFO("%s: i2c access fail!\n", __func__);
		}
	}
#ifdef HX_OPT_HW_CRC
	hx83102j_en_hw_crc(chip_info, 1);
#endif
	hx83102j_hx83102j_reload_to_active(chip_info);
}

/**
 * hx83102j_enable_interrupt -   Device interrupt ability control.
 * @chip_info: struct include i2c resource.
 * @enable: disable or enable control purpose.
 * Return  0: succeed, -1: failed.
 */

static int hx83102j_enable_interrupt(struct chip_data_hx83102j *chip_info, bool enable)
{
	struct irq_desc *desc = NULL;


	TPD_INFO("%s: now irq depth=%d\n", __func__, desc);
	if (enable == true) {
		do {
			enable_irq(chip_info->hx_irq);
			desc = irq_to_desc(chip_info->hx_irq);
			if(desc->depth > 0)
				continue;
			else
				break;
			TPD_INFO("%s: after en irq depth=%d\n", __func__, desc->depth);
		} while (desc->depth > 0);
	} else if (enable == false) {
		disable_irq_nosync(chip_info->hx_irq);
	} else {
		TPD_DETAIL("irq is not pairing! enable= %d", enable);
	}

	return 0;
}

#ifdef HX_ZERO_FLASH

#if defined(HX_ALG_OVERLAY)
static int hx83102j_mcu_register_read(struct chip_data_hx83102j *chip_info, uint8_t *addr, uint8_t *buf, uint32_t len)
{
	int ret = -1;

	if (addr[0] == chip_info->pzf_op->addr_spi200_data[0]
	&& addr[1] == chip_info->pzf_op->addr_spi200_data[1]
	&& addr[2] == chip_info->pzf_op->addr_spi200_data[2]
	&& addr[3] == chip_info->pzf_op->addr_spi200_data[3])
		hx83102j_burst_enable(chip_info, 0);
	else if (len > DATA_LEN_4)
		hx83102j_burst_enable(chip_info, 1);
	else
		hx83102j_burst_enable(chip_info, 0);

	ret = hx83102j_bus_write_with_addr(chip_info, chip_info->pzf_op->addr_ahb_addr_byte_0[0], addr, NULL, 4);
	if (ret < 0) {
		TPD_INFO("%s: bus access fail!\n", __func__);

		return -1;
	}

	ret = hx83102j_bus_write_with_addr(chip_info, chip_info->pzf_op->addr_ahb_access_direction[0], NULL,
		&chip_info->pzf_op->data_ahb_access_direction_read[0], 1);
	if (ret < 0) {
		TPD_INFO("%s: bus access fail!\n", __func__);

		return -1;
	}

	ret = hx83102j_bus_read(chip_info, chip_info->pzf_op->addr_ahb_rdata_byte_0[0], len, buf);
	if (ret < 0) {
		TPD_INFO("%s: bus access fail!\n", __func__);

		return -1;
	}


	return NO_ERR;
}


static int hx83102j_mcu_assign_sorting_mode(struct chip_data_hx83102j *chip_info, uint8_t *tmp_data)
{
	uint8_t retry = 0;
	uint8_t rdata[4] = {0};

	TPD_INFO("%s:addr: 0x%02X%02X%02X%02X, write to:0x%02X%02X%02X%02X\n",
		__func__,
		chip_info->pzf_op->addr_sorting_mode_en[3],
		chip_info->pzf_op->addr_sorting_mode_en[2],
		chip_info->pzf_op->addr_sorting_mode_en[1],
		chip_info->pzf_op->addr_sorting_mode_en[0],
		tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);

	while (retry++ < 3) {
		hx83102j_mcu_register_write(chip_info, chip_info->pzf_op->addr_sorting_mode_en, DATA_LEN_4,
			tmp_data, 0);
		usleep_range(1000, 1100);
		hx83102j_mcu_register_read(chip_info, chip_info->pzf_op->addr_sorting_mode_en,
			rdata, DATA_LEN_4);

		if (rdata[3] == tmp_data[3] && rdata[2] == tmp_data[2]
		&& rdata[1] == tmp_data[1] && rdata[0] == tmp_data[0]) {
			TPD_INFO("%s: success to write sorting mode\n", __func__);
			return NO_ERR;
		}
		TPD_INFO("%s: fail to write sorting mode\n", __func__);
	}
	return -1;
}
#endif
#if defined(HX_ALG_OVERLAY)
static int alg_overlay(struct chip_data_hx83102j *chip_info, uint8_t alg_idx_t, struct zf_info *info,
	const struct firmware *fw)
{
	int ret = 0;
	int retry = 0;
	uint8_t tmp_addr[4] = {0xFC, 0x7F, 0x00, 0x10};
	uint8_t rdata[4] = {0};

	uint8_t i = 0;
	uint8_t alg_sdata[4] = {0xA5, 0x5A, 0x5A, 0xA5};

	uint8_t data[4] = {0x01, 0x00, 0x00, 0x00};

	uint8_t tmp_data[64];

	TPD_INFO("%s: Enter \n", __func__);

	if (alg_idx_t == 0 || info[alg_idx_t].write_size == 0) {
		TPD_INFO("%s: wrong alg overlay section[%d, %d]!\n", __func__,
			alg_idx_t, info[alg_idx_t].write_size);
		ret = FW_NOT_READY;
		goto ALOC_CFG_BUF_FAIL;
	}

	retry = 0;
	do {
		hx83102j_mcu_register_write(chip_info, tmp_addr, DATA_LEN_4, alg_sdata,
			0);
		usleep_range(1000, 1100);
		hx83102j_mcu_register_read(chip_info, tmp_addr, rdata,
			DATA_LEN_4);
	} while ((rdata[0] != alg_sdata[0]
	|| rdata[1] != alg_sdata[1]
	|| rdata[2] != alg_sdata[2]
	|| rdata[3] != alg_sdata[3])
	&& retry++ < HIMAX_REG_RETRY_TIMES);

	if (retry > HIMAX_REG_RETRY_TIMES) {
		TPD_INFO("%s: init handshaking data FAIL[%02X%02X%02X%02X]!!\n",
			__func__, rdata[0], rdata[1], rdata[2], rdata[3]);
	}

	alg_sdata[3] = OVL_ALG_REPLY;
	alg_sdata[2] = OVL_ALG_REPLY;
	alg_sdata[1] = OVL_ALG_REPLY;
	alg_sdata[0] = OVL_ALG_REPLY;

	chip_info->g_core_fp.fp_reload_disable(chip_info, 0);


	hx83102j_mcu_register_write(chip_info, chip_info->pzf_op->addr_raw_out_sel, sizeof(chip_info->pzf_op->data_clear),
		chip_info->pzf_op->data_clear, 0);
	/*DSRAM func initial*/
	hx83102j_mcu_assign_sorting_mode(chip_info, chip_info->pzf_op->data_clear);
	/* reset N frame back to default for normal mode */
	hx83102j_mcu_register_write(chip_info, chip_info->pzf_op->addr_set_frame_addr, 4, data, 0);
	/*FW reload done initial*/
	hx83102j_mcu_register_write(chip_info, chip_info->pzf_op->addr_fw_define_2nd_flash_reload, sizeof(chip_info->pzf_op->data_clear) ,
		chip_info->pzf_op->data_clear, 0);

	hx83102j_sense_on(chip_info, 0x00);

	retry = 0;
	do {
		usleep_range(3000, 3100);
		hx83102j_mcu_register_read(chip_info, tmp_addr, rdata, DATA_LEN_4);
	} while ((rdata[0] != OVL_ALG_REQUEST
	|| rdata[1] != OVL_ALG_REQUEST
	|| rdata[2] != OVL_ALG_REQUEST
	|| rdata[3] != OVL_ALG_REQUEST)
	&& retry++ < 30);

	if (retry > 30) {
		TPD_INFO("%s: fail req data = 0x%02X%02X%02X%02X\n", __func__,
			rdata[0], rdata[1], rdata[2], rdata[3]);
		/* monitor FW status for debug */
		for (i = 0; i < 10; i++) {
			usleep_range(10000, 10100);
			hx83102j_mcu_register_read(chip_info, tmp_addr, rdata, DATA_LEN_4);
			TPD_DEBUG("%s: req data = 0x%02X%02X%02X%02X\n",
				__func__, rdata[0], rdata[1], rdata[2],
				rdata[3]);
			chip_info->cmd_set[0] = 0x01;
			hx83102j_read_FW_status(chip_info, chip_info->cmd_set, tmp_data);
		}
		ret = 3;
		goto BURN_OVL_FAIL;
	}

	TPD_DEBUG("%s: upgrade alg overlay section[%d]\n", __func__, alg_idx_t);

	if (hx83102j_sram_write_crc_check(chip_info, fw, info[alg_idx_t].sram_addr,
		info[alg_idx_t].fw_addr,
		info[alg_idx_t].write_size) != 0) {
		TPD_INFO("%s: Alg Overlay HW crc FAIL\n", __func__);
		ret = 2;
	}

	retry = 0;
	do {
		hx83102j_mcu_register_write(chip_info, tmp_addr, DATA_LEN_4, alg_sdata, 0);
		usleep_range(1000, 1100);
		hx83102j_mcu_register_read(chip_info, tmp_addr, rdata, DATA_LEN_4);
	} while ((alg_sdata[3] != rdata[3]
	|| alg_sdata[2] != rdata[2]
	|| alg_sdata[1] != rdata[1]
	|| alg_sdata[0] != rdata[0])
	&& retry++ < HIMAX_REG_RETRY_TIMES);

	if (retry > HIMAX_REG_RETRY_TIMES) {
		TPD_INFO("%s: fail rpl data = 0x%02X%02X%02X%02X\n", __func__,
			rdata[0], rdata[1], rdata[2], rdata[3]);
	} else {
		TPD_INFO("%s: waiting for FW reload data", __func__);

		retry = 0;
		while (retry++ < 30) {
			hx83102j_mcu_register_read(chip_info, chip_info->pzf_op->addr_fw_define_2nd_flash_reload, data, DATA_LEN_4);

			/* use all 4 bytes to compare */
			if ((data[3] == 0x00 && data[2] == 0x00 &&
			data[1] == 0x72 && data[0] == 0xC0)) {
				TPD_INFO("%s: FW reload done\n", __func__);
					break;
			}
			TPD_INFO("%s: wait FW reload %d times\n", __func__, retry);
			chip_info->cmd_set[0] = 0x01;
			hx83102j_read_FW_status(chip_info, chip_info->cmd_set, tmp_data);
			usleep_range(10000, 11000);
		}
	}

BURN_OVL_FAIL:
ALOC_CFG_BUF_FAIL:
	TPD_INFO("%s: Exit \n", __func__);
	return ret;
}
#endif
static void hx83102j_in_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len)
{
	/*TPD_INFO("%s: Entering!\n", __func__);*/
	switch (len) {
	case 1:
		cmd[0] = addr;
		/*TPD_INFO("%s: cmd[0] = 0x%02X\n", __func__, cmd[0]);*/
		break;

	case 2:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		/*TPD_INFO("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X\n", __func__, cmd[0], cmd[1]);*/
		break;

	case 4:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		cmd[2] = (addr >> 16) % 0x100;
		cmd[3] = addr / 0x1000000;
		/*  TPD_INFO("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X,cmd[2] = 0x%02X,cmd[3] = 0x%02X\n",
			__func__, cmd[0], cmd[1], cmd[2], cmd[3]);*/
		break;

	default:
		TPD_INFO("%s: input length fault, len = %d!\n", __func__, len);
	}
}
/*
static void hx_update_dirly_0f(struct chip_data_hx83102j *chip_info)
{
	TPD_DEBUG("It will update fw after esd event in zero flash mode!\n");
	chip_info->g_core_fp.fp_0f_operation_dirly(chip_info);
}
*/
static int hx_dis_rload_0f(struct chip_data_hx83102j *chip_info, int disable)
{
	/*Diable Flash Reload*/
	int retry = 10;
	int check_val = 0;
	uint8_t tmp_data[4] = {0};

	TPD_DETAIL("%s: Entering !\n", __func__);

	do {
		hx83102j_flash_write_burst(chip_info, chip_info->pzf_op->addr_dis_flash_reload,  chip_info->pzf_op->data_dis_flash_reload);
		hx83102j_register_read(chip_info, chip_info->pzf_op->addr_dis_flash_reload, 4, tmp_data, false);
		TPD_DETAIL("Now data: tmp_data[3] = 0x%02X || tmp_data[2] = 0x%02X || tmp_data[1] = 0x%02X || tmp_data[0] = 0x%02X\n",
					tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
		if(tmp_data[3] != 0x00 || tmp_data[2] != 0x00 || tmp_data[1] != 0x9A || tmp_data[0] != 0xA9) {
			TPD_INFO("Now data: tmp_data[3] = 0x%02X || tmp_data[2] = 0x%02X || tmp_data[1] = 0x%02X || tmp_data[0] = 0x%02X\n",
						tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
			TPD_INFO("Not Same,Write Fail, there is %d retry times!\n", retry);
			usleep_range(5000, 5001);
		} else {
			check_val = 1;
			TPD_DETAIL("It's same! Write success!\n");
		}
	} while (check_val == 0 && retry-- > 0);

	TPD_DETAIL("%s: END !\n", __func__);

	return check_val;
}

static void hx83102j_mcu_clean_sram_0f(struct chip_data_hx83102j *chip_info, uint8_t *addr, int write_len, int type)
{
	int total_read_times = 0;
	int max_bus_size = MAX_TRANS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	int address = 0;
	int i = 0;
	uint8_t fix_data = 0x00;
	uint8_t tmp_addr[4];
	uint8_t *tmp_data;

	TPD_DETAIL("%s, Entering \n", __func__);
	tmp_data = kzalloc(sizeof(uint8_t) * MAX_TRANS_SZ, GFP_KERNEL);
	if (tmp_data == NULL) {
		TPD_INFO("%s: error tmp_data no memory !\n", __func__);
		return;
	}
	total_size = write_len;
	total_size_temp = write_len;

	if (total_size > MAX_TRANS_SZ) {
		max_bus_size = MAX_TRANS_SZ;
	}

	total_size_temp = write_len;

	hx83102j_burst_enable(chip_info, 1);

	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];
	TPD_DETAIL("%s, write addr tmp_addr[3] = 0x%2.2X, tmp_addr[2] = 0x%2.2X, tmp_addr[1] = 0x%2.2X, tmp_addr[0] = 0x%2.2X\n",
				__func__, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);

	switch (type) {
	case 0:
		fix_data = 0x00;
		break;
	case 1:
		fix_data = 0xAA;
		break;
	case 2:
		fix_data = 0xBB;
		break;
	default:
		TPD_INFO("%s: hx83102j_mcu_clean_sram_0f error type = %d \n", __func__, type);
		break;
	}

	for (i = 0; i < MAX_TRANS_SZ; i++) {
		tmp_data[i] = fix_data;
	}

	TPD_DETAIL("%s, total size=%d\n", __func__, total_size);

	if (total_size_temp % max_bus_size == 0) {
		total_read_times = total_size_temp / max_bus_size;
	} else {
		total_read_times = total_size_temp / max_bus_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		/*TPD_DETAIL("[log]write %d time start!\n", i);*/
		if (total_size_temp >= max_bus_size) {
			hx83102j_flash_write_burst_length(chip_info, tmp_addr, tmp_data,  max_bus_size);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			TPD_DETAIL("last total_size_temp=%d\n", total_size_temp);
			hx83102j_flash_write_burst_length(chip_info, tmp_addr, tmp_data,  total_size_temp % max_bus_size);
		}
		address = ((i + 1) * max_bus_size);
		tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF);
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);

		msleep(10);
	}
	if (tmp_data)
		kfree(tmp_data);
	TPD_DETAIL("%s, END \n", __func__);
}

static void hx83102j_mcu_write_sram_0f(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry,
					uint8_t *addr, int start_index, uint32_t write_len)
{
	int total_read_times = 0;
	int max_bus_size = MAX_TRANS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	uint32_t address = 0;
	int i = 0;
	uint8_t tmp_addr[4];
	uint32_t now_addr;

	TPD_DETAIL("%s, ---Entering \n", __func__);

	total_size = fw_entry->size;

	total_size_temp = write_len;
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	if (write_len > MAX_TRANS_SZ) {
		max_bus_size = MAX_TRANS_SZ;
	} else {
		max_bus_size = write_len;
	}
#else
#ifdef HIMAX_DBG
	if (write_len > 49152) {
		max_bus_size = 49152;
	} else {
		max_bus_size = write_len;
	}
#else
	if (write_len > HX64K) {
		max_bus_size = HX64K;
	} else {
		max_bus_size = write_len;
	}
#endif
#endif
	hx83102j_burst_enable(chip_info, 1);

	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];
	TPD_DETAIL("%s, write addr tmp_addr[3] = 0x%2.2X, tmp_addr[2] = 0x%2.2X, tmp_addr[1] = 0x%2.2X, tmp_addr[0] = 0x%2.2X\n",
			__func__, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);
	now_addr = (addr[3] << 24) + (addr[2] << 16) + (addr[1] << 8) + addr[0];
	TPD_DETAIL("now addr= 0x%08X\n", now_addr);

	TPD_DETAIL("%s,  total size=%d\n", __func__, total_size);

	if (chip_info->tmp_data == NULL) {
		chip_info->tmp_data = kzalloc(sizeof(uint8_t) * firmware_update_space, GFP_KERNEL);
		if (chip_info->tmp_data == NULL) {
			TPD_INFO("%s, alloc chip_info->tmp_data failed\n", __func__);
			return;
		}
	}
	memcpy(chip_info->tmp_data, fw_entry->data, total_size);
	/*
	for (i = 0;i < 10;i++) {
		TPD_INFO("[%d] 0x%2.2X", i, tmp_data[i]);
	}
	TPD_INFO("\n");
	*/
	if (total_size_temp % max_bus_size == 0) {
		total_read_times = total_size_temp / max_bus_size;
	} else {
		total_read_times = total_size_temp / max_bus_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		/*
		TPD_INFO("[log]write %d time start!\n", i);
		TPD_INFO("[log]addr[3] = 0x%02X, addr[2] = 0x%02X, addr[1] = 0x%02X, addr[0] = 0x%02X!\n", tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);
		*/
		if (total_size_temp >= max_bus_size) {
			hx83102j_flash_write_burst_length(chip_info, tmp_addr, &(chip_info->tmp_data[start_index + i * max_bus_size]),  max_bus_size);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			TPD_DETAIL("last total_size_temp=%d\n", total_size_temp);
			hx83102j_flash_write_burst_length(chip_info, tmp_addr, &(chip_info->tmp_data[start_index + i * max_bus_size]),  total_size_temp % max_bus_size);
		}

		/*TPD_INFO("[log]write %d time end!\n", i);*/
		address = ((i + 1) * max_bus_size)+now_addr;
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);

		tmp_addr[3] = (address>>24) & 0x00FF;
		tmp_addr[2] = (address>>16) & 0x00FF;
		tmp_addr[1] = (address>>8) & 0x00FF;
		tmp_addr[0] = address & 0x00FF;
		/*
		if (tmp_addr[0] < addr[0]) {
			tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF) + 1;
		} else {
			tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF);
		}*/


		udelay(100);
	}
	TPD_DETAIL("%s, ----END \n", __func__);

	memset(chip_info->tmp_data, 0, total_size);
}
static int hx83102j_sram_write_crc_check(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry, uint8_t *addr, int strt_idx, uint32_t len)
{
	int retry = 0;
	int crc = -1;

	TPD_DETAIL("%s, addr=0x%02X%02X%02X%02X, start_idx=%d=0x%08X, len=%d\n",
		__func__, addr[3], addr[2], addr[1], addr[0], strt_idx, strt_idx, len);

	if (len <= 0) {
		TPD_INFO("%s,len=%d, Fail!!\n", __func__, len);
		goto END;
	}


	do {
		chip_info->g_core_fp.fp_write_sram_0f(chip_info, fw_entry, addr, strt_idx, len);
		crc = hx83102j_hw_check_crc(chip_info, addr, len);
		retry++;
		TPD_DETAIL("%s, HW crc %s in %d time\n", __func__, (crc == 0)?"OK":"Fail", retry);
	} while (crc != 0 && retry < 10);

END:
	return crc;
}

static int hx83102j_mcu_Calculate_crc_with_AP(unsigned char *FW_content, int crc_from_FW, int len)
{
	int i, j, length = 0;
	int fw_data;
	int fw_data_2;
	int crc = 0xFFFFFFFF;
	int polynomial = 0x82F63B78;

	length = len / 4;

	for (i = 0; i < length; i++) {
		fw_data = FW_content[i * 4];

		for (j = 1; j < 4; j++) {
			fw_data_2 = FW_content[i * 4 + j];
			fw_data += (fw_data_2) << (8 * j);
		}

		crc = fw_data ^ crc;

		for (j = 0; j < 32; j++) {
			if ((crc % 2) != 0) {
				crc = ((crc >> 1) & 0x7FFFFFFF) ^ polynomial;
			} else {
				crc = (((crc >> 1) & 0x7FFFFFFF)/*& 0x7FFFFFFF*/);
			}
		}
		/*I("crc = %x, i = %d \n", crc, i);*/
	}

	return crc;
}

#if defined(HX_CODE_OVERLAY)
static int hx83102j_0f_overlay(struct chip_data_hx83102j *chip_info, int ovl_type, int mode)
{
	uint8_t count = 0;
	uint8_t count2 = 0;
	uint8_t part_num = 0;
	uint8_t buf[16];
	uint8_t handshaking_addr[4] = {0xFC, 0x7F, 0x00, 0x10};
	uint8_t ovl_idx_t;
	uint8_t request;
	uint8_t reply;
	uint8_t sram_addr[4] = {0};
	uint32_t offset = 0;
	uint32_t size = 0;
	uint8_t send_data[4] = {0};
	uint8_t recv_data[4] = {0};
	int ret = 0;
	uint32_t cfg_table_pos = chip_info->cfg_table_flash_addr;
	struct firmware *request_fw_headfile = NULL;
	const struct firmware *tmp_fw_entry = NULL;

	 /*1. check fw status*/
	TPD_INFO("Get FW from buffer or headfile\n");
	if (request_fw_headfile == NULL) {
		request_fw_headfile = kzalloc(sizeof(struct firmware), GFP_KERNEL);
	}
	if(request_fw_headfile == NULL) {
		TPD_INFO("%s kzalloc is failed!\n", __func__);
		ret = MEM_ALLOC_FAIL;
		goto FAIL_OUT1;
	}
	if (chip_info->g_fw_sta) {
		TPD_INFO("get from g_fw_buf\n");
		request_fw_headfile->size = chip_info->g_fw_len;
		request_fw_headfile->data = chip_info->g_fw_buf;
		tmp_fw_entry = request_fw_headfile;
	} else {
		TPD_INFO("get from g_fw_buf failed, get from headfile\n");
		if(chip_info->p_firmware_headfile->firmware_data) {
			request_fw_headfile->size = chip_info->p_firmware_headfile->firmware_size;
			request_fw_headfile->data = chip_info->p_firmware_headfile->firmware_data;
			tmp_fw_entry = request_fw_headfile;
			chip_info->using_headfile = true;
		} else {
			TPD_INFO("firmware_data is NULL! exit firmware update!\n");
			ret = FW_NOT_READY;
			goto FAIL_OUT2;
		}
	}

	/*2. check in self_test or not*/
	if (chip_info->in_self_test == 0) {
	/*1. get number of partition*/
			TPD_DETAIL("%s, cfg_table_pos=0x%08X\n", __func__, cfg_table_pos);
			part_num = tmp_fw_entry->data[cfg_table_pos + 12];
			if (part_num <= 1) {
				TPD_INFO("%s, size of cfg part failed! part_num = %d\n",
						__func__, part_num);
				ret = LENGTH_FAIL;
				goto FAIL_OUT2;
			}

			TPD_DETAIL("%s: overlay section %d\n", __func__, ovl_type-1);
			if (ovl_type == 2) {
				request = OVL_GESTURE_REQUEST;
				reply = OVL_GESTURE_REPLY;

			} else if (ovl_type == 3) {
				request = OVL_BORDER_REQUEST;
				reply = OVL_BORDER_REPLY;

			} else {
				TPD_INFO("%s: error overlay type %d\n", __func__, ovl_type);
				ret = HX_INIT_FAIL;
				goto FAIL_OUT2;
			}
			ovl_idx_t = ovl_idx[ovl_type - 1];
			memcpy(buf, &tmp_fw_entry->data[ovl_idx_t * 0x10 + cfg_table_pos], 16);
			memcpy(sram_addr, buf, 4);
			offset = buf[11] << 24 | buf[10] << 16 | buf[9] << 8 | buf[8];
			size = buf[7] << 24 | buf[6] << 16 | buf[5] << 8 | buf[4];

		hx83102j_enable_interrupt(chip_info, false);
		hx83102j_sense_off(chip_info);

			if (hx83102j_sram_write_crc_check(chip_info, tmp_fw_entry, sram_addr, offset, size) != 0)
				TPD_INFO("%s, Overlay HW crc FAIL\n", __func__);

			send_data[3] = 0x00;
			send_data[2] = 0x00;
			send_data[1] = 0x00;
			send_data[0] = reply;
			count2 = 0;
			do {
				hx83102j_register_write(chip_info, handshaking_addr, 4, send_data, false);
				msleep(1);
				hx83102j_register_read(chip_info, handshaking_addr, 4, recv_data, false);
			} while (recv_data[0] != reply && count2++ < 10);
	}

	TPD_DETAIL("%s: overlay request %d times; reply %d times\n", __func__,
			count, count2);

	/* rescue mechanism
*	if (count >= 10) {
*		chip_info->g_core_fp.fp_0f_operation_dirly();
*		chip_info->g_core_fp.fp_reload_disable(0);
*		hx83102j_sense_on(0x00);
*		hx83102j_enable_interrupt(chip_info, true);
*	}
*/
		ret = 0;
FAIL_OUT2:
	 if (request_fw_headfile != NULL) {
		kfree(request_fw_headfile);
		request_fw_headfile = NULL;
	}
FAIL_OUT1:
	hx83102j_sense_on_by_sys_rst(chip_info);
	hx83102j_enable_interrupt(chip_info, true);
	return ret;
}
#endif
static bool hx_bin_desc_data_get(struct chip_data_hx83102j *chip_info, uint32_t addr, uint8_t *flash_buf)
{
	uint8_t data_sz = 0x10;
	uint32_t i = 0, j = 0;
	uint16_t chk_end = 0;
	uint16_t chk_sum = 0;
	uint32_t map_code = 0;
	unsigned long flash_addr = 0;

	for (i = 0; i < FW_PAGE_SZ; i = i + data_sz) {
		for (j = i; j < (i + data_sz); j++) {
			chk_end |= flash_buf[j];
			chk_sum += flash_buf[j];
		}
		if (!chk_end) { /*1. Check all zero*/
			TPD_INFO("%s: End in %X\n",	__func__, i + addr);
			return false;
		} else if (chk_sum % 0x100) { /*2. Check sum*/
			TPD_DETAIL("%s: chk sum failed in %X\n",	__func__, i + addr);
		} else { /*3. get data*/
			map_code = flash_buf[i] + (flash_buf[i + 1] << 8)
			+ (flash_buf[i + 2] << 16) + (flash_buf[i + 3] << 24);
			flash_addr = flash_buf[i + 4] + (flash_buf[i + 5] << 8)
			+ (flash_buf[i + 6] << 16) + (flash_buf[i + 7] << 24);
			switch (map_code) {
			case FW_CID:
				chip_info->cid_ver_maj_flash_addr = flash_addr;
				chip_info->cid_ver_min_flash_addr = flash_addr + 1;
				TPD_DETAIL("%s: CID_VER in %lX\n", __func__,
				chip_info->cid_ver_maj_flash_addr);
				break;
			case FW_VER:
				chip_info->fw_ver_maj_flash_addr = flash_addr;
				chip_info->fw_ver_min_flash_addr = flash_addr + 1;
				TPD_DETAIL("%s: FW_VER in %lX\n", __func__,
				chip_info->fw_ver_maj_flash_addr);
				break;
			case CFG_VER:
				chip_info->cfg_ver_maj_flash_addr = flash_addr;
				chip_info->cfg_ver_min_flash_addr = flash_addr + 1;
				TPD_DETAIL("%s: CFG_VER in = %08lX\n", __func__,
				chip_info->cfg_ver_maj_flash_addr);
				break;
			case TP_CONFIG_TABLE:
				chip_info->cfg_table_flash_addr = flash_addr;
				TPD_DETAIL("%s: CONFIG_TABLE in %X\n",
				__func__, chip_info->cfg_table_flash_addr);
				break;
			default:
				TPD_INFO("%s: ERROR map_code  %d\n", __func__, map_code);
				break;
			}
		}
		chk_end = 0;
		chk_sum = 0;
	}

	return true;
}

static bool hx_mcu_bin_desc_get(struct chip_data_hx83102j *chip_info, unsigned char *fw, uint32_t max_sz)
{
	uint32_t addr_t = 0;
	unsigned char *fw_buf = NULL;
	bool keep_on_flag = false;
	bool g_bin_desc_flag = false;

	do {
		fw_buf = &fw[addr_t];

		/*Check bin is with description table or not*/
		if (!g_bin_desc_flag) {
			if (fw_buf[0x00] == 0x00 && fw_buf[0x01] == 0x00
			&& fw_buf[0x02] == 0x00 && fw_buf[0x03] == 0x00
			&& fw_buf[0x04] == 0x00 && fw_buf[0x05] == 0x00
			&& fw_buf[0x06] == 0x00 && fw_buf[0x07] == 0x00
			&& fw_buf[0x0E] == 0x87)
				g_bin_desc_flag = true;
		}
		if (!g_bin_desc_flag) {
			TPD_INFO("%s: fw_buf[0x00] = %2X, fw_buf[0x0E] = %2X\n",
			__func__, fw_buf[0x00], fw_buf[0x0E]);
			TPD_INFO("%s: No description table\n",	__func__);
			break;
		}

		/*Get related data*/
		keep_on_flag = hx_bin_desc_data_get(chip_info, addr_t, fw_buf);

		addr_t = addr_t + FW_PAGE_SZ;
	} while (max_sz > addr_t && keep_on_flag);

	return g_bin_desc_flag;
}
static bool hx_parse_bin_cfg_data(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry)
{
	uint32_t cfg_table_pos = chip_info->cfg_table_flash_addr;
	bool flag_1k_header = false;
	int part_num = 0;
	int i = 0;
	uint8_t buf[16];
	int i_max = 0;
	int i_min = 0;
	uint32_t dsram_base = 0xFFFFFFFF;
	uint32_t dsram_max = 0;

#if defined(HX_CODE_OVERLAY) || defined(HX_ALG_OVERLAY)
	int allovlidx = 0;

#endif

#if defined(HX_ALG_OVERLAY)

	 chip_info->g_alg_idx_t = 0;
	 chip_info->g_has_alg_overlay = false;
#endif
#if defined(HX_CODE_OVERLAY)
	uint8_t j = 0;

	if (ovl_idx == NULL) {
		ovl_idx = kzalloc(OVL_SECTION_NUM, GFP_KERNEL);
		if (ovl_idx == NULL) {
			TPD_INFO("%s, ovl_idx alloc failed!\n",
				__func__);
			return -ENOMEM;
		}
	} else {
		memset(ovl_idx, 0, OVL_SECTION_NUM);
	}
#endif

	/*0. check 1k header*/
	if (fw_entry->data[0x00] == 0x00
		&& fw_entry->data[0x01] == 0x00
		&& fw_entry->data[0x02] == 0x00
		&& fw_entry->data[0x03] == 0x00
		&& fw_entry->data[0x04] == 0x00
		&& fw_entry->data[0x05] == 0x00
		&& fw_entry->data[0x06] == 0x00
		&& fw_entry->data[0x07] == 0x00
		&& fw_entry->data[0x0E] == 0x87)
		flag_1k_header = true;
	else
		flag_1k_header = false;

	/*1. get number of partition*/
	part_num = fw_entry->data[cfg_table_pos + 12];


	if (part_num <= 1) {
		TPD_INFO("%s, size of cfg part failed! part_num = %d\n", __func__, part_num);
		return false;
	}
	/*2. initial struct of array*/
	if (chip_info->g_zf_info_arr == NULL)
		chip_info->g_zf_info_arr = kzalloc(part_num * sizeof(struct zf_info), GFP_KERNEL);
	if (chip_info->g_zf_info_arr == NULL) {
		TPD_INFO("%s, Allocate ZF info array failed!\n", __func__);
		return false;
	}

	for (i = 0; i < part_num; i++) {
		/*3. get all partition*/
		TPD_DETAIL("%s, i * 0x10 + cfg_table_pos =%d \n", __func__, i * 0x10 + cfg_table_pos);
		memcpy(buf, &fw_entry->data[i * 0x10 + cfg_table_pos], 16);
		TPD_DETAIL("%s, buf[7]=0x%x buf[6]=0x%x buf[5]=0x%x buf[4]=0x%x \n", __func__, buf[7], buf[6], buf[5], buf[4]);
		memcpy(chip_info->g_zf_info_arr[i].sram_addr, buf, 4);
		chip_info->g_zf_info_arr[i].write_size =  buf[7] << 24 | buf[6] << 16
				| buf[5] << 8 | buf[4];
		chip_info->g_zf_info_arr[i].fw_addr = buf[10] << 16 | buf[9] << 8 | buf[8];
		chip_info->g_zf_info_arr[i].cfg_addr = chip_info->g_zf_info_arr[i].sram_addr[0];
		chip_info->g_zf_info_arr[i].cfg_addr += chip_info->g_zf_info_arr[i].sram_addr[1] << 8;
		chip_info->g_zf_info_arr[i].cfg_addr += chip_info->g_zf_info_arr[i].sram_addr[2] << 16;
		chip_info->g_zf_info_arr[i].cfg_addr += chip_info->g_zf_info_arr[i].sram_addr[3] << 24;

#if defined(HX_CODE_OVERLAY)
		/*overlay section*/
		if ((buf[15] == 0x55 && buf[14] == 0x66)
		|| (buf[3] == 0x20 && buf[2] == 0x00
		&& buf[1] == 0x8C && buf[0] == 0xE0)) {
			TPD_INFO("%s: catch overlay section in index %d\n",
				__func__, i);

			/* record index of overlay section */
			allovlidx |= 1 << i;

			if (buf[15] == 0x55 && buf[14] == 0x66) {
				/* current mechanism */
				j = buf[13];
				if (j < OVL_SECTION_NUM)
					ovl_idx[j] = i;
			} else {
				/* previous mechanism */
				if (j < OVL_SECTION_NUM)
					ovl_idx[j++] = i;
			}

			continue;
		}
#endif
#if defined(HX_ALG_OVERLAY)
		if ((buf[15] == 0x77 && buf[14] == 0x88)) {
			TPD_INFO("%s: find alg overlay section in index %d\n",
				__func__, i);
			/* record index of alg overlay section */
			allovlidx |= 1 << i;
			chip_info->g_alg_idx_t = i;
			chip_info->g_has_alg_overlay = true;
			continue;
		}

# endif
		/*TPD_INFO("%s, [%d] SRAM addr = %08X\n", __func__, i, chip_info->g_zf_info_arr[i].cfg_addr);
		TPD_INFO("%s, [%d] fw_addr = %04X!\n", __func__, i, chip_info->g_zf_info_arr[i].fw_addr);
		TPD_INFO("%s, [%d] write_size = %d!\n", __func__, i, chip_info->g_zf_info_arr[i].write_size); */
		if (i == 0)
			continue;
		if (dsram_base > chip_info->g_zf_info_arr[i].cfg_addr) {
			dsram_base = chip_info->g_zf_info_arr[i].cfg_addr;
			i_min = i;
		} else if (dsram_max < chip_info->g_zf_info_arr[i].cfg_addr) {
			dsram_max = chip_info->g_zf_info_arr[i].cfg_addr;
			i_max = i;
		}
	}
	for (i = 0; i < 4; i++)
		chip_info->hx83102j_nf_sram_min[i] = chip_info->g_zf_info_arr[i_min].sram_addr[i];
	chip_info->hx83102j_nf_cfg_sz = (dsram_max - dsram_base) + chip_info->g_zf_info_arr[i_max].write_size;

	if (chip_info->hx83102j_nf_cfg_sz % 4 > 0)
		chip_info->hx83102j_nf_cfg_sz = chip_info->hx83102j_nf_cfg_sz + (chip_info->hx83102j_nf_cfg_sz % 4);


	TPD_INFO("%s, chip_info->hx83102j_nf_cfg_sz = %d!, dsram_base = %X, dsram_max = %X\n", __func__, chip_info->hx83102j_nf_cfg_sz, dsram_base, dsram_max);

	if (chip_info->hx83102j_nf_fw_buf == NULL)
		chip_info->hx83102j_nf_fw_buf = kzalloc(sizeof(unsigned char) * HX64K, GFP_KERNEL);
	if (chip_info->hx83102j_nf_fw_buf == NULL) {
		TPD_INFO("%s, chip_info->hx83102j_nf_fw_buf allocates fail\n", __func__);
		return false;
	}

	for (i = 1; i < part_num; i++) {
		if (chip_info->g_zf_info_arr[i].cfg_addr % 4 != 0)
			chip_info->g_zf_info_arr[i].cfg_addr = chip_info->g_zf_info_arr[i].cfg_addr - (chip_info->g_zf_info_arr[i].cfg_addr % 4);

#if defined(HX_CODE_OVERLAY)|| defined(HX_ALG_OVERLAY)
		/*overlay section*/
		if (allovlidx & (1 << i)) {
			TPD_INFO("%s: skip overlay section %d\n", __func__, i);
			continue;
		}
#endif
		memcpy(chip_info->hx83102j_nf_fw_buf + (chip_info->g_zf_info_arr[i].cfg_addr - dsram_base)
			, (unsigned char *)&fw_entry->data[chip_info->g_zf_info_arr[i].fw_addr], chip_info->g_zf_info_arr[i].write_size);
	}
	chip_info->hx83102j_nf_cfg_crc = hx83102j_mcu_Calculate_crc_with_AP(chip_info->hx83102j_nf_fw_buf, 0, chip_info->hx83102j_nf_cfg_sz);
	TPD_INFO("hx83102j_nf_cfg_crc = 0x%x\n", chip_info->hx83102j_nf_cfg_crc);
	return true;
}

static int hx83102j_nf_zf_part_info(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry)
{
	bool ret = false;
	bool flag_1k_header = false;
	int retry = 0;
	int crc = -1;
#if defined(HX_CODE_OVERLAY)
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);
	uint8_t tmp_addr[4] = {0xFC, 0x7F, 0x00, 0x10};
	uint8_t send_data[4] = {0};
	uint8_t recv_data[4] = {0};
	uint8_t ovl_idx_t = 0;
	uint8_t gesture_addr[4] = {0};
	uint8_t gesture_data[4] = {0};
#endif

	if (!hx_parse_bin_cfg_data(chip_info, fw_entry))
		TPD_INFO("%s, Parse cfg from bin failed\n", __func__);
#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(chip_info->hw_res, false);
		usleep_range(5000, 5001);
		hx83102j_resetgpio_set(chip_info->hw_res, true);
		usleep_range(5000, 5001);
#else
		chip_info->g_core_fp.fp_sys_reset(chip_info);
#endif
	hx83102j_sense_off(chip_info);
#ifdef HX_OPT_HW_CRC
	hx83102j_en_hw_crc(chip_info, 1);
#endif
	/*getnstimeofday(&timeStart);*/
	/* first 64K */

	/*0. check 1k header*/
	if (fw_entry->data[0x00] == 0x00
		&& fw_entry->data[0x01] == 0x00
		&& fw_entry->data[0x02] == 0x00
		&& fw_entry->data[0x03] == 0x00
		&& fw_entry->data[0x04] == 0x00
		&& fw_entry->data[0x05] == 0x00
		&& fw_entry->data[0x06] == 0x00
		&& fw_entry->data[0x07] == 0x00
		&& fw_entry->data[0x0E] == 0x87)
		flag_1k_header = true;
	else
		flag_1k_header = false;

	if (flag_1k_header == true)
		crc = hx83102j_sram_write_crc_check(chip_info, fw_entry, chip_info->pzf_op->data_sram_start_addr, HX1K,  chip_info->g_zf_info_arr[0].write_size);
	else
		crc = hx83102j_sram_write_crc_check(chip_info, fw_entry, chip_info->pzf_op->data_sram_start_addr, 0,  chip_info->g_zf_info_arr[0].write_size);

	ret = (crc == 0) ? true : false;
	if (crc != 0)
		TPD_INFO("size=%d crc Failed! crc = %X",  chip_info->g_zf_info_arr[0].write_size, crc);
	do {
		hx83102j_mcu_register_write(chip_info, chip_info->hx83102j_nf_sram_min, chip_info->hx83102j_nf_cfg_sz, chip_info->hx83102j_nf_fw_buf, 0);
		crc = hx83102j_hw_check_crc(chip_info, chip_info->hx83102j_nf_sram_min, chip_info->hx83102j_nf_cfg_sz);
		if (crc != chip_info->hx83102j_nf_cfg_crc)
			TPD_INFO("Config crc FAIL, HW crc = %X, SW crc = %X, retry time = %d", crc, chip_info->hx83102j_nf_cfg_crc, retry);
		retry++;
	} while (!ret && retry < 10);
	if(retry > 9) {
		if (chip_info->g_zf_info_arr)
			kfree(chip_info->g_zf_info_arr);
		chip_info->g_zf_info_arr = NULL;
		return -1;
	}
#if defined(HX_CODE_OVERLAY)
	if (ts->gesture_enable) {
			gesture_addr[3] = 0x10;
			gesture_addr[2] = 0x00;
			gesture_addr[1] = 0x7F;
			gesture_addr[0] = 0x10;
			gesture_data[3] = 0xA5;
			gesture_data[2] = 0x5A;
			gesture_data[1] = 0xA5;
			gesture_data[0] = 0x5A;
			hx83102j_register_write(chip_info, gesture_addr, 4, gesture_data, false);
		}
#endif
#if defined(HX_CODE_OVERLAY)
		/* ovl_idx[0] - sorting */
		/* ovl_idx[1] - gesture */
		/* ovl_idx[2] - border  */

if (chip_info->in_self_test == 1) {
			ovl_idx_t = ovl_idx[0];
			send_data[0] = OVL_SORTING_REPLY;
	} else {
			ovl_idx_t = ovl_idx[2];
			send_data[0] = OVL_BORDER_REPLY;
	}


		if (chip_info->g_zf_info_arr[ovl_idx_t].write_size == 0) {
			send_data[0] = OVL_FAULT;
			TPD_INFO("%s, WRONG overlay section, plese check FW!\n",
					__func__);
		} else {
			if (hx83102j_sram_write_crc_check(chip_info, fw_entry,
			chip_info->g_zf_info_arr[ovl_idx_t].sram_addr,
			chip_info->g_zf_info_arr[ovl_idx_t].fw_addr,
			chip_info->g_zf_info_arr[ovl_idx_t].write_size) != 0) {
				send_data[0] = OVL_FAULT;
				TPD_INFO("%s, Overlay HW crc FAIL\n", __func__);
			} else {
				TPD_INFO("%s, Overlay HW crc PASS\n", __func__);
			}
		}

		retry = 0;
		do {
			hx83102j_register_write(chip_info, tmp_addr, 4, send_data, false);
			hx83102j_register_read(chip_info, tmp_addr, 4, recv_data, false);
			retry++;
		} while ((send_data[3] != recv_data[3]
				|| send_data[2] != recv_data[2]
				|| send_data[1] != recv_data[1]
				|| send_data[0] != recv_data[0])
				&& retry < HIMAX_REG_RETRY_TIMES);
#endif
#if defined(HX_ALG_OVERLAY)
	if (chip_info->g_has_alg_overlay)
		alg_overlay(chip_info, chip_info->g_alg_idx_t, chip_info->g_zf_info_arr, fw_entry);
#endif
	chip_info->g_core_fp.fp_clean_sram_0f(chip_info, chip_info->pzf_op->data_mode_switch, 4, 0);
	if (chip_info->g_zf_info_arr)
		kfree(chip_info->g_zf_info_arr);
	chip_info->g_zf_info_arr = NULL;
	return 0;
}

static void hx83102j_mcu_firmware_update_0f(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry)
{
	int retry = 0;
	int crc = -1;
	int ret = 0;
	uint8_t temp_addr[4];
	uint8_t temp_data[4];
	struct firmware *request_fw_headfile = NULL;
	const struct firmware *tmp_fw_entry = NULL;
	bool reload = false;
#ifdef HIMAX_DBG
	char *aa = "Himax_firmware.bin";
#endif
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);
	TPD_DETAIL("%s, Entering \n", __func__);

fw_reload:
	if (fw_entry == NULL || reload) {
		TPD_INFO("Get FW from headfile\n");
		if (request_fw_headfile == NULL) {
			request_fw_headfile = kzalloc(sizeof(struct firmware), GFP_KERNEL);
		}
		if (request_fw_headfile == NULL) {
			TPD_INFO("%s kzalloc failed!\n", __func__);
			return;
		}
		if (chip_info->g_fw_sta) {
			TPD_INFO("request firmware failed, get from g_fw_buf\n");
			request_fw_headfile->size = chip_info->g_fw_len;
			request_fw_headfile->data = chip_info->g_fw_buf;
			tmp_fw_entry = request_fw_headfile;

		} else {
			TPD_INFO("request firmware failed, get from headfile\n");
			if(chip_info->p_firmware_headfile->firmware_data) {
				request_fw_headfile->size = chip_info->p_firmware_headfile->firmware_size;
				request_fw_headfile->data = chip_info->p_firmware_headfile->firmware_data;
				tmp_fw_entry = request_fw_headfile;
				chip_info->using_headfile = true;
			} else {
				TPD_INFO("firmware_data is NULL! exit firmware update!\n");

#ifdef HIMAX_DBG
				TPD_INFO("hx self update flow!\n");
				goto test_update;
#else
				if (ts->firmware_in_dts && ts->firmware_in_dts->data) {
					request_fw_headfile->size = ts->firmware_in_dts->size;
					request_fw_headfile->data = ts->firmware_in_dts->data;
					tmp_fw_entry = request_fw_headfile;
				} else {
					TPD_INFO("%s firmware_in_dts NULL\n", __func__);
					if(request_fw_headfile != NULL) {
						kfree(request_fw_headfile);
						request_fw_headfile = NULL;
					}
					return;
				}
#endif
			}
		}
	} else {
		tmp_fw_entry = fw_entry;
#ifdef HIMAX_DBG
		TPD_INFO("%s: There is fw from upper layer!\n", __func__);
		goto fw_done;
#endif
	}
/*
 #if defined(HX_BOOT_UPGRADE)
	ret = request_firmware(&tmp_fw_entry, g_fw_boot_upgrade_name, ts->dev);
	TPD_INFO("%s: ---request file %s finished\n", __func__, g_fw_boot_upgrade_name);
	if (ret < 0) {
		TPD_INFO("%s,%d: error code = %d\n", __func__, __LINE__, ret);
	}
#endif*/
#ifdef HIMAX_DBG
test_update:

	ret = request_firmware(&tmp_fw_entry, aa, ts->dev);
	TPD_INFO("%s request firmware:%s!\n", __func__, aa);
	if (ret < 0) {
		TPD_INFO("[Error] Fail request_firmware !\n");
		return;
	}
fw_done:
#endif


	chip_info->g_fw_entry = tmp_fw_entry;

	hx_mcu_bin_desc_get(chip_info, (unsigned char *)chip_info->g_fw_entry->data, HX1K);

	if ((int)tmp_fw_entry->size > HX64K) {
		ret = hx83102j_nf_zf_part_info(chip_info, tmp_fw_entry);
	} else {
#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(chip_info->hw_res, false);
		usleep_range(5000, 5001);
		hx83102j_resetgpio_set(chip_info->hw_res, true);
		usleep_range(5000, 5001);
#else
		chip_info->g_core_fp.fp_sys_reset(chip_info);
#endif
		hx83102j_sense_off(chip_info);
		/* first 64K */
		do {
			chip_info->g_core_fp.fp_write_sram_0f(chip_info, tmp_fw_entry, chip_info->pzf_op->data_sram_start_addr, 0, HX64K);
			crc = hx83102j_hw_check_crc(chip_info, chip_info->pzf_op->data_sram_start_addr, HX64K);

			temp_addr[3] = 0x08;
			temp_addr[2] = 0x00;
			temp_addr[1] = 0xBF;
			temp_addr[0] = 0xFC;
			hx83102j_register_read(chip_info, temp_addr, 4, temp_data, false);

			TPD_DETAIL("%s, 64K LAST 4 BYTES: data[3] = 0x%02X, data[2] = 0x%02X, data[1] = 0x%02X, data[0] = 0x%02X\n",
							__func__, temp_data[3], temp_data[2], temp_data[1], temp_data[0]);

			if (crc == 0) {
				TPD_DETAIL("%s, HW crc OK in %d time \n", __func__, retry);
				break;
			} else {
				TPD_INFO("%s, HW crc FAIL in %d time !\n", __func__, retry);
			}
			retry++;
		} while (((temp_data[3] == 0 && temp_data[2] == 0 && temp_data[1] == 0 && temp_data[0] == 0 &&
						retry < 80) || (crc != 0 && retry < 30)) && !(chip_info->using_headfile && retry < 3));

		if (crc != 0) {
			TPD_INFO("Last time crc Fail!\n");
			if(reload) {
				return;
			} else {
				reload = true;
				goto fw_reload;
			}
		}

		/* if chip_info->g_poweronof!= 1, it will be clean mode! */
		/*config and setting */
		/*config info*/
		if (chip_info->g_poweronof == 1) {
			retry = 0;
			do {
				chip_info->g_core_fp.fp_write_sram_0f(chip_info, tmp_fw_entry, chip_info->pzf_op->data_cfg_info, 0xC000, 128);
				crc = hx83102j_hw_check_crc(chip_info, chip_info->pzf_op->data_cfg_info, 128);
				if (crc == 0) {
					TPD_DETAIL("%s, config info ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, config info fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("config info crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			chip_info->g_core_fp.fp_clean_sram_0f(chip_info, chip_info->pzf_op->data_cfg_info, 128, 2);
		}
		/*FW config*/
		if (chip_info->g_poweronof == 1) {
			retry = 0;
			do {
				chip_info->g_core_fp.fp_write_sram_0f(chip_info, tmp_fw_entry, chip_info->pzf_op->data_fw_cfg_p1, 0xC100, 528);
				crc = hx83102j_hw_check_crc(chip_info, chip_info->pzf_op->data_fw_cfg_p1, 528);
				if (crc == 0) {
					TPD_DETAIL("%s, 1 FW config ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, 1 FW config fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("1 FW config crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			chip_info->g_core_fp.fp_clean_sram_0f(chip_info, chip_info->pzf_op->data_fw_cfg_p1, 528, 1);
		}

		if (chip_info->g_poweronof == 1) {
			retry = 0;
			do {
				chip_info->g_core_fp.fp_write_sram_0f(chip_info, tmp_fw_entry, chip_info->pzf_op->data_fw_cfg_p3, 0xCA00, 128);
				crc = hx83102j_hw_check_crc(chip_info, chip_info->pzf_op->data_fw_cfg_p3, 128);
				if (crc == 0) {
					TPD_DETAIL("%s, 3 FW config ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, 3 FW config fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("3 FW config crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			chip_info->g_core_fp.fp_clean_sram_0f(chip_info, chip_info->pzf_op->data_fw_cfg_p3, 128, 1);
		}

		/*ADC config*/
		if (chip_info->g_poweronof == 1) {
			retry = 0;
			do {
				chip_info->g_core_fp.fp_write_sram_0f(chip_info, tmp_fw_entry, chip_info->pzf_op->data_adc_cfg_1, 0xD640, 1200);
				crc = hx83102j_hw_check_crc(chip_info, chip_info->pzf_op->data_adc_cfg_1, 1200);
				if (crc == 0) {
					TPD_DETAIL("%s, 1 ADC config ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, 1 ADC config fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("1 ADC config crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			chip_info->g_core_fp.fp_clean_sram_0f(chip_info, chip_info->pzf_op->data_adc_cfg_1, 1200, 2);
		}

		if (chip_info->g_poweronof == 1) {
			retry = 0;
			do {
				chip_info->g_core_fp.fp_write_sram_0f(chip_info, tmp_fw_entry, chip_info->pzf_op->data_adc_cfg_2, 0xD320, 800);
				crc = hx83102j_hw_check_crc(chip_info, chip_info->pzf_op->data_adc_cfg_2, 800);
				if (crc == 0) {
					TPD_DETAIL("%s, 2 ADC config ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, 2 ADC config fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("2 ADC config crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			chip_info->g_core_fp.fp_clean_sram_0f(chip_info, chip_info->pzf_op->data_adc_cfg_2, 800, 2);
		}

		/*mapping table*/
		if (chip_info->g_poweronof == 1) {
			retry = 0;
			do {
				chip_info->g_core_fp.fp_write_sram_0f(chip_info, tmp_fw_entry, chip_info->pzf_op->data_map_table, 0xE000, 1536);
				crc = hx83102j_hw_check_crc(chip_info, chip_info->pzf_op->data_map_table, 1536);
				if (crc == 0) {
					TPD_DETAIL("%s, mapping table ok in %d time \n", __func__, retry);
					break;
				} else {
					TPD_INFO("%s, mapping table fail in %d time !\n", __func__, retry);
				}
				retry++;
			} while ((crc != 0 && retry < 30) || (chip_info->using_headfile && retry < 15));

			if (crc != 0) {
				TPD_INFO("mapping table crc Fail!\n");
				if (!reload) {
					reload = true;
					goto fw_reload;
				}
			}
		} else {
			chip_info->g_core_fp.fp_clean_sram_0f(chip_info, chip_info->pzf_op->data_map_table, 1536, 2);
		}
	}

	chip_info->first_download_finished = true;

	/* switch mode*/
	if (chip_info->g_poweronof == 1) {
		chip_info->g_core_fp.fp_write_sram_0f(chip_info, tmp_fw_entry, chip_info->pzf_op->data_mode_switch, 0xC30C, 4);
	} else {
		chip_info->g_core_fp.fp_clean_sram_0f(chip_info, chip_info->pzf_op->data_mode_switch, 4, 2);
	}

	hx83102j_fw_check(ts->chip_data, &ts->resolution_info, &ts->panel_data);

#ifdef HIMAX_DBG
	if (fw_entry == NULL) {
		TPD_INFO("%s, release fw \n", __func__);
		release_firmware(tmp_fw_entry);
	} else {
		TPD_INFO("%s, From Upper layer,skip relase firmware \n", __func__);
	}
#else
	if (request_fw_headfile != NULL) {
		kfree(request_fw_headfile);
		request_fw_headfile = NULL;
	}
#endif

	TPD_DETAIL("%s, END \n", __func__);
}
static int hx_0f_op_file_dirly(struct chip_data_hx83102j *chip_info, char *file_name)
{
	int err = NO_ERR;
	const struct firmware *fw_entry = NULL;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);


	TPD_INFO("%s, Entering \n", __func__);
	TPD_INFO("file name = %s\n", file_name);
#ifdef HIMAX_DBG
	err = request_firmware(&fw_entry, file_name, ts->dev);
#else
	if (ts->fw_update_app_support) {
		err = request_firmware_select(&fw_entry, file_name, ts->dev);
	} else {
		err = request_firmware(&fw_entry, file_name, ts->dev);
	}
#endif
	if (err < 0) {
		TPD_INFO("%s, fail in line%d error code=%d, file maybe fail\n", __func__, __LINE__, err);
		return err;
	}


	if(chip_info->g_f_0f_updat == 1) {
		TPD_INFO("%s:[Warning]Other thread is updating now!\n", __func__);
		release_firmware(fw_entry);
		err = -1;
		return err;
	} else {
		TPD_INFO("%s:Entering Update Flow!\n", __func__);
		chip_info->g_f_0f_updat = 1;
	}

	hx83102j_enable_interrupt(chip_info, false);

	/* trigger reset */
#ifdef HX_RST_PIN_FUNC
	hx83102j_resetgpio_set(chip_info->hw_res, false);
	usleep_range(5000, 5001);
	hx83102j_resetgpio_set(chip_info->hw_res, true);
	usleep_range(5000, 5001);
#else
	chip_info->g_core_fp.fp_sys_reset(chip_info);
#endif
	chip_info->g_core_fp.fp_firmware_update_0f(chip_info, fw_entry);
	release_firmware(fw_entry);

	chip_info->g_f_0f_updat = 0;
	TPD_INFO("%s, END \n", __func__);
	return err;
}
static int hx83102j_mcu_0f_operation_dirly(struct chip_data_hx83102j *chip_info)
{
	int err = NO_ERR;

	TPD_DETAIL("%s, Entering \n", __func__);

	if(chip_info->g_f_0f_updat == 1) {
		TPD_INFO("%s:[Warning]Other thread is updating now!\n", __func__);
		err = -1;
		return err;
	} else {
		TPD_DETAIL("%s:Entering Update Flow!\n", __func__);
		chip_info->g_f_0f_updat = 1;
	}


	chip_info->g_core_fp.fp_firmware_update_0f(chip_info, NULL);


	chip_info->g_f_0f_updat = 0;
	TPD_DETAIL("%s, END \n", __func__);
	return err;
}
/*
static int hx83102j_mcu_0f_operation_test_dirly(struct chip_data_hx83102j *chip_info, char *fw_name)
{
	int err = NO_ERR;
	const struct firmware *fw_entry = NULL;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	TPD_DETAIL("%s, Entering \n", __func__);
	TPD_DETAIL("file name = %s\n", fw_name);
	TPD_INFO("Request TP firmware.\n");
	err = request_firmware(&fw_entry, fw_name, ts->dev);
	if (err < 0) {
		TPD_INFO("%s, fail in line%d error code=%d, file maybe fail\n", __func__, __LINE__, err);
		if (fw_entry != NULL) {
			release_firmware(fw_entry);
			fw_entry = NULL;
		}
		return err;
	}

	chip_info->g_core_fp.fp_firmware_update_0f(chip_info, fw_entry);
	release_firmware(fw_entry);

	TPD_DETAIL("%s, END \n", __func__);
	return err;
}
*/
static void hx83102j_mcu_0f_operation(struct work_struct *work)
{
	struct chip_data_hx83102j *chip_info = container_of(work,
			struct chip_data_hx83102j, work_0f_update.work);
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);
	TPD_INFO("file name = %s\n", ts->panel_data.fw_name);

	if (chip_info->g_f_0f_updat == 1) {
		TPD_INFO("%s:[Warning]Other thread is updating now!\n", __func__);
		return;
	} else {
		TPD_INFO("%s:Entering Update Flow!\n", __func__);
		chip_info->g_f_0f_updat = 1;
	}

	hx83102j_enable_interrupt(chip_info, false);
	msleep(1);
	chip_info->g_core_fp.fp_firmware_update_0f(chip_info, NULL);

	chip_info->g_core_fp.fp_reload_disable(chip_info, 0);
	msleep(10);
	hx83102j_read_FW_ver(chip_info);
	msleep(10);
	hx83102j_sense_on(chip_info, 0x00);
	msleep(10);
	TPD_INFO("%s:End \n", __func__);

#ifdef CONFIG_OPLUS_TP_APK
	if(chip_info->debug_mode_sta) {
		if(ts->apk_op && ts->apk_op->apk_debug_set) {
			ts->apk_op->apk_debug_set(ts->chip_data, true);
		}
	}
#endif

	hx83102j_enable_interrupt(chip_info, true);

	chip_info->g_f_0f_updat = 0;
	TPD_INFO("%s, END \n", __func__);
	return;
}

#ifdef HX_0F_DEBUG
static void hx83102j_mcu_read_sram_0f(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry, uint8_t *addr, int start_index, int read_len)
{
	int total_read_times = 0;
	int max_bus_size = MAX_TRANS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	int address = 0;
	int i = 0, j = 0;
	int not_same = 0;

	uint8_t tmp_addr[4];
	uint8_t *temp_info_data;
	int *not_same_buff;

	TPD_INFO("%s, Entering \n", __func__);

	hx83102j_burst_enable(chip_info, 1);

	total_size = read_len;

	total_size_temp = read_len;
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	if (read_len > MAX_RECVS_SZ) {
		max_bus_size = MAX_RECVS_SZ;
	} else {
		max_bus_size = read_len;
	}
#else
	if (read_len > 2048) {
		max_bus_size = 2048;
	} else {
		max_bus_size = read_len;
	}
#endif
	temp_info_data = kzalloc(sizeof(uint8_t) * total_size, GFP_KERNEL);
	if (temp_info_data == NULL) {
		TPD_INFO("%s, temp_info_data get ram fail \n", __func__);
		return;
	}
	not_same_buff = kzalloc(sizeof(int) * total_size, GFP_KERNEL);
	if (not_same_buff == NULL) {
		if(temp_info_data) {
			kfree(temp_info_data);
		}
		TPD_INFO("%s, not_same_buff get ram fail \n", __func__);
		return;
	}

	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];

	TPD_INFO("%s, total size=%d\n", __func__, total_size);

	hx83102j_burst_enable(chip_info, 1);

	if (total_size % max_bus_size == 0) {
		total_read_times = total_size / max_bus_size;
	} else {
		total_read_times = total_size / max_bus_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		if (total_size_temp >= max_bus_size) {
			hx83102j_register_read(chip_info, tmp_addr, max_bus_size, &temp_info_data[i * max_bus_size], false);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			hx83102j_register_read(chip_info, tmp_addr, total_size_temp % max_bus_size, &temp_info_data[i * max_bus_size], false);
		}

		address = ((i + 1) * max_bus_size);
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);
		if (tmp_addr[0] < addr[0]) {
			tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF) + 1;
		} else {
			tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF);
		}

		msleep(10);
	}
	TPD_INFO("%s, READ Start \n", __func__);
	TPD_INFO("%s, start_index = %d \n", __func__, start_index);
	j = start_index;
	for (i = 0; i < read_len; i++, j++) {
		if (chip_info->g_fw_buf[j] != temp_info_data[i]) {
			not_same++;
			not_same_buff[i] = 1;
		}

		TPD_INFO("0x%2.2X, ", temp_info_data[i]);

		if (i > 0 && i % 16 == 15) {
			printk("\n");
		}
	}
	TPD_INFO("%s, READ END \n", __func__);
	TPD_INFO("%s, Not Same count=%d\n", __func__, not_same);
	if (not_same != 0) {
		j = start_index;
		for (i = 0; i < read_len; i++, j++) {
			if (not_same_buff[i] == 1) {
				TPD_INFO("bin = [%d] 0x%2.2X\n", i, chip_info->g_fw_buf[j]);
			}
		}
		for (i = 0; i < read_len; i++, j++) {
			if (not_same_buff[i] == 1) {
				TPD_INFO("sram = [%d] 0x%2.2X \n", i, temp_info_data[i]);
			}
		}
	}
	TPD_INFO("%s, READ END \n", __func__);
	TPD_INFO("%s, Not Same count=%d\n", __func__, not_same);
	TPD_INFO("%s, END \n", __func__);

	if (not_same_buff)
		kfree(not_same_buff);
	if (temp_info_data)
		kfree(temp_info_data);
}

static void hx83102j_mcu_read_all_sram(struct chip_data_hx83102j *chip_info, uint8_t *addr, int read_len)
{
	int total_read_times = 0;
	int max_bus_size = MAX_RECVS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	int address = 0;
	int i = 0;
	/*
	struct file *fn;
	struct filename *vts_name;
	*/

	uint8_t tmp_addr[4];
	uint8_t *temp_info_data;

	TPD_INFO("%s, Entering \n", __func__);

	hx83102j_burst_enable(chip_info, 1);

	total_size = read_len;

	total_size_temp = read_len;

	temp_info_data = kzalloc(sizeof(uint8_t) * total_size, GFP_KERNEL);
	if (temp_info_data == NULL) {
		TPD_INFO("%s, temp_info_data get ram fail \n", __func__);
		return;
	}


	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];

	TPD_INFO("%s, total size=%d\n", __func__, total_size);

	if (total_size % max_bus_size == 0) {
		total_read_times = total_size / max_bus_size;
	} else {
		total_read_times = total_size / max_bus_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		if (total_size_temp >= max_bus_size) {
			hx83102j_register_read(chip_info, tmp_addr, max_bus_size, &temp_info_data[i * max_bus_size], false);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			hx83102j_register_read(chip_info, tmp_addr, total_size_temp % max_bus_size, &temp_info_data[i * max_bus_size], false);
		}

		address = ((i + 1) * max_bus_size);
		tmp_addr[1] = addr[1] + (uint8_t) ((address >> 8) & 0x00FF);
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);

		msleep(10);
	}
	/*for (i = 0;i < read_len;i++) {
		TPD_INFO("0x%2.2X, ", temp_info_data[i]);

		if (i > 0 && i%16 == 15)
			printk("\n");
	}*/

	/* need modify
	TPD_INFO("Now Write File start!\n");
	vts_name = getname_kernel("/sdcard/dump_dsram.txt");
	fn = file_open_name(vts_name, O_CREAT | O_WRONLY, 0);
	if (!IS_ERR (fn)) {
		TPD_INFO("%s create file and ready to write\n", __func__);
		fn->f_op->write (fn, temp_info_data, read_len*sizeof (uint8_t), &fn->f_pos);
		filp_close (fn, NULL);
	}
	TPD_INFO("Now Write File End!\n");
	*/

	TPD_INFO("%s, END \n", __func__);
	if (temp_info_data)
		kfree(temp_info_data);
}

static void hx83102j_mcu_firmware_read_0f(struct chip_data_hx83102j *chip_info, const struct firmware *fw_entry, int type)
{
	uint8_t tmp_addr[4] = {0};

	TPD_INFO("%s, Entering \n", __func__);
	if (type == 0) { /* first 64K */
		chip_info->g_core_fp.fp_read_sram_0f(chip_info, fw_entry, chip_info->pzf_op->data_sram_start_addr, 0, HX64K);
		chip_info->g_core_fp.fp_read_all_sram(chip_info, tmp_addr, 0xC000);
	} else { /*last 16k*/
		chip_info->g_core_fp.fp_read_sram_0f(chip_info, fw_entry, chip_info->pzf_op->data_cfg_info, 0xC000, 132);
		chip_info->g_core_fp.fp_read_sram_0f(chip_info, fw_entry, chip_info->pzf_op->data_fw_cfg, 0xC0FE, 512);
		chip_info->g_core_fp.fp_read_sram_0f(chip_info, fw_entry, chip_info->pzf_op->data_adc_cfg_1, 0xD000, 376);
		chip_info->g_core_fp.fp_read_sram_0f(chip_info, fw_entry, chip_info->pzf_op->data_adc_cfg_2, 0xD178, 376);
		chip_info->g_core_fp.fp_read_sram_0f(chip_info, fw_entry, chip_info->pzf_op->data_adc_cfg_3, 0xD000, 376);
		chip_info->g_core_fp.fp_read_all_sram(chip_info, chip_info->pzf_op->data_sram_clean, HX_32K_SZ);
	}
	TPD_INFO("%s, END \n", __func__);
}

static void hx83102j_mcu_0f_operation_check(struct chip_data_hx83102j *chip_info, int type)
{
	TPD_INFO("%s, Entering \n", __func__);

	chip_info->g_core_fp.fp_firmware_read_0f(chip_info, NULL, type);

	TPD_INFO("%s, END \n", __func__);
	return;
}
#endif


static int hx_0f_init(struct chip_data_hx83102j *chip_info)
{
	if (chip_info->pzf_op == NULL)
		chip_info->pzf_op = kzalloc(sizeof(struct zf_operation), GFP_KERNEL);
	if (chip_info->pzf_op == NULL) {
		TPD_INFO("%s, chip_info->pzf_op get ram fail \n", __func__);
		return -1;
	}
	chip_info->g_core_fp.fp_reload_disable = hx_dis_rload_0f;
	chip_info->g_core_fp.fp_sys_reset = hx83102j_mcu_sys_reset;
	chip_info->g_core_fp.fp_clean_sram_0f = hx83102j_mcu_clean_sram_0f;
	chip_info->g_core_fp.fp_write_sram_0f = hx83102j_mcu_write_sram_0f;
	chip_info->g_core_fp.fp_firmware_update_0f = hx83102j_mcu_firmware_update_0f;
	chip_info->g_core_fp.fp_0f_operation = hx83102j_mcu_0f_operation;
	chip_info->g_core_fp.fp_0f_operation_dirly = hx83102j_mcu_0f_operation_dirly;
	chip_info->g_core_fp.fp_0f_op_file_dirly = hx_0f_op_file_dirly;
#ifdef HX_0F_DEBUG
	chip_info->g_core_fp.fp_read_sram_0f = hx83102j_mcu_read_sram_0f;
	chip_info->g_core_fp.fp_read_all_sram = hx83102j_mcu_read_all_sram;
	chip_info->g_core_fp.fp_firmware_read_0f = hx83102j_mcu_firmware_read_0f;
	chip_info->g_core_fp.fp_0f_operation_check = hx83102j_mcu_0f_operation_check;
#endif

	hx83102j_in_parse_assign_cmd(ZF_ADDR_DIS_FLASH_RELOAD, chip_info->pzf_op->addr_dis_flash_reload, sizeof(chip_info->pzf_op->addr_dis_flash_reload));
	hx83102j_in_parse_assign_cmd(ZF_DATA_DIS_FLASH_RELOAD, chip_info->pzf_op->data_dis_flash_reload, sizeof(chip_info->pzf_op->data_dis_flash_reload));
	hx83102j_in_parse_assign_cmd(ZF_ADDR_SYSTEM_RESET, chip_info->pzf_op->addr_system_reset, sizeof(chip_info->pzf_op->addr_system_reset));
	hx83102j_in_parse_assign_cmd(ZF_DATA_SYSTEM_RESET, chip_info->pzf_op->data_system_reset, sizeof(chip_info->pzf_op->data_system_reset));
	hx83102j_in_parse_assign_cmd(ZF_DATA_SRAM_START_ADDR, chip_info->pzf_op->data_sram_start_addr, sizeof(chip_info->pzf_op->data_sram_start_addr));
	hx83102j_in_parse_assign_cmd(ZF_DATA_SRAM_CLEAN, chip_info->pzf_op->data_sram_clean, sizeof(chip_info->pzf_op->data_sram_clean));
	hx83102j_in_parse_assign_cmd(ZF_DATA_CFG_INFO, chip_info->pzf_op->data_cfg_info, sizeof(chip_info->pzf_op->data_cfg_info));
	hx83102j_in_parse_assign_cmd(ZF_DATA_FW_CFG_P1, chip_info->pzf_op->data_fw_cfg_p1, sizeof(chip_info->pzf_op->data_fw_cfg_p1));
	hx83102j_in_parse_assign_cmd(ZF_DATA_FW_CFG_P2, chip_info->pzf_op->data_fw_cfg_p2, sizeof(chip_info->pzf_op->data_fw_cfg_p2));
	hx83102j_in_parse_assign_cmd(ZF_DATA_FW_CFG_P3, chip_info->pzf_op->data_fw_cfg_p3, sizeof(chip_info->pzf_op->data_fw_cfg_p3));
	hx83102j_in_parse_assign_cmd(ZF_DATA_ADC_CFG_1, chip_info->pzf_op->data_adc_cfg_1, sizeof(chip_info->pzf_op->data_adc_cfg_1));
	hx83102j_in_parse_assign_cmd(ZF_DATA_ADC_CFG_2, chip_info->pzf_op->data_adc_cfg_2, sizeof(chip_info->pzf_op->data_adc_cfg_2));
	hx83102j_in_parse_assign_cmd(ZF_DATA_ADC_CFG_3, chip_info->pzf_op->data_adc_cfg_3, sizeof(chip_info->pzf_op->data_adc_cfg_3));
	hx83102j_in_parse_assign_cmd(ZF_DATA_MAP_TABLE, chip_info->pzf_op->data_map_table, sizeof(chip_info->pzf_op->data_map_table));
	hx83102j_in_parse_assign_cmd(ZF_DATA_MODE_SWITCH, chip_info->pzf_op->data_mode_switch, sizeof(chip_info->pzf_op->data_mode_switch));

#if defined(HX_ALG_OVERLAY)
	hx83102j_in_parse_assign_cmd(DRIVER_ADDR_FW_DEFINE_2ND_FLASH_RELOAD, chip_info->pzf_op->addr_fw_define_2nd_flash_reload,
		sizeof(chip_info->pzf_op->addr_fw_define_2nd_flash_reload));
	hx83102j_in_parse_assign_cmd(FW_ADDR_RAW_OUT_SEL, chip_info->pzf_op->addr_raw_out_sel, sizeof(chip_info->pzf_op->addr_raw_out_sel));
	hx83102j_in_parse_assign_cmd(FW_ADDR_SET_FRAME_ADDR, chip_info->pzf_op->addr_set_frame_addr, sizeof(chip_info->pzf_op->addr_set_frame_addr));
	hx83102j_in_parse_assign_cmd(FW_DATA_CLEAR, chip_info->pzf_op->data_clear, sizeof(chip_info->pzf_op->data_clear));
	hx83102j_in_parse_assign_cmd(FW_ADDR_SORTING_MODE_EN,
		chip_info->pzf_op->addr_sorting_mode_en,
		sizeof(chip_info->pzf_op->addr_sorting_mode_en));
	hx83102j_in_parse_assign_cmd(FLASH_ADDR_SPI200_DATA,
		chip_info->pzf_op->addr_spi200_data,
		sizeof(chip_info->pzf_op->addr_spi200_data));
	hx83102j_in_parse_assign_cmd(IC_ADR_AHB_ADDR_BYTE_0,
		chip_info->pzf_op->addr_ahb_addr_byte_0,
		sizeof(chip_info->pzf_op->addr_ahb_addr_byte_0));
	hx83102j_in_parse_assign_cmd(IC_ADR_AHB_ACCESS_DIRECTION,
		chip_info->pzf_op->addr_ahb_access_direction,
		sizeof(chip_info->pzf_op->addr_ahb_access_direction));
	hx83102j_in_parse_assign_cmd(IC_CMD_AHB_ACCESS_DIRECTION_READ,
		chip_info->pzf_op->data_ahb_access_direction_read,
		sizeof(chip_info->pzf_op->data_ahb_access_direction_read));
	hx83102j_in_parse_assign_cmd(IC_ADR_AHB_RDATA_BYTE_0,
		chip_info->pzf_op->addr_ahb_rdata_byte_0,
		sizeof(chip_info->pzf_op->addr_ahb_rdata_byte_0));


#endif

	return 0;
}

#endif

static bool hx83102j_ic_package_check(struct chip_data_hx83102j *chip_info)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[8] = {0};
	uint8_t ret_data = 0x00;
	int i = 0;

#ifdef HX_RST_PIN_FUNC
	hx83102j_resetgpio_set(chip_info->hw_res, false);
	usleep_range(5000, 5001);
	hx83102j_resetgpio_set(chip_info->hw_res, true);
	usleep_range(5000, 5001);
#else
	hx83102j_mcu_sys_reset(chip_info);
#endif

	hx83102j_sense_off(chip_info);


	for (i = 0; i < 5; i++) {
		/* Product ID */
		/* Touch*/
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xD0;
		hx83102j_register_read(chip_info, tmp_addr, 8, tmp_data, false);


		TPD_INFO("%s: Read driver IC ID: tmp_data[0]=0x%02X, tmp_data[1]=0x%02X, tmp_data[2]=0x%02X, tmp_data[3]=0x%02X \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);


		/*if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x11) && ((tmp_data[1] == 0x2f) || (tmp_data[1] == 0x2f)))*/
		if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x10) && (tmp_data[1] == 0x29)) {
			chip_info->ic_type = HX_83102J_SERIES_PWON;
			chip_info->ic_checksum = HX_TP_BIN_CHECKSUM_CRC;
			hx_0f_init(chip_info);

			chip_info->fw_ver_maj_flash_addr   = 59397;
			chip_info->fw_ver_maj_flash_leng   = 1;
			chip_info->fw_ver_min_flash_addr   = 59398;
			chip_info->fw_ver_min_flash_leng   = 1;
			chip_info->cfg_ver_maj_flash_addr = 59648;
			chip_info->cfg_ver_maj_flash_leng = 1;
			chip_info->cfg_ver_min_flash_addr = 59649;
			chip_info->cfg_ver_min_flash_leng = 1;
			chip_info->cid_ver_maj_flash_addr = 59394;
			chip_info->cid_ver_maj_flash_leng = 1;
			chip_info->cid_ver_min_flash_addr = 59395;
			chip_info->cid_ver_min_flash_leng = 1;

#ifdef HX_AUTO_UPDATE_FW
			g_i_FW_VER = i_CTPM_FW[chip_info->fw_ver_maj_flash_addr] << 8 | i_CTPM_FW[chip_info->fw_ver_min_flash_addr];
			g_i_CFG_VER = i_CTPM_FW[chip_info->cfg_ver_maj_flash_addr] << 8 | i_CTPM_FW[chip_info->cfg_ver_min_flash_addr];
			g_i_CID_MAJ = i_CTPM_FW[chip_info->cid_ver_maj_flash_addr];
			g_i_CID_MIN = i_CTPM_FW[chip_info->cid_ver_min_flash_addr];
#endif
			TPD_INFO("Himax IC package 83102j_in\n");
			ret_data = true;
			break;
		} else {
			ret_data = false;
			TPD_INFO("%s:Read driver ID register Fail:\n", __func__);
		}
	}

	return ret_data;
}


static void hx83102j_power_on_init(struct chip_data_hx83102j *chip_info)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	TPD_INFO("%s\n", __func__);

	/*RawOut select initial*/
	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x02;
	tmp_addr[1] = 0x04;
	tmp_addr[0] = 0xF4;

	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, false);

	/*DSRAM func initial*/
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x07;
	tmp_addr[0] = 0xFC;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, false);
#ifndef HX_ZERO_FLASH
	hx83102j_sense_on(chip_info, 0x00);
#endif
}


static void hx83102j_read_FW_ver(struct chip_data_hx83102j *chip_info)
{
	uint8_t cmd[4];
	uint8_t data[64];
	uint8_t data2[64];
	int retry = 20;
	int reload_status = 0;
	u8 ver_len = 0;
	char dev_version[MAX_DEVICE_VERSION_LENGTH] = {0};
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	hx83102j_sense_on(chip_info, 0);

	while(reload_status == 0) {
		cmd[3] = 0x10;  /*oplus fw id bin address : 0xc014   , 49172    Tp ic address : 0x 10007014*/
		cmd[2] = 0x00;
		cmd[1] = 0x7f;
		cmd[0] = 0x00;
		hx83102j_register_read(chip_info, cmd, 4, data, false);

		cmd[3] = 0x10;
		cmd[2] = 0x00;
		cmd[1] = 0x72;
		cmd[0] = 0xc0;
		hx83102j_register_read(chip_info, cmd, 4, data2, false);

		if ((data[1] == 0x3A && data[0] == 0xA3) || (data2[1] == 0x72 && data2[0] == 0xc0)) {
			TPD_INFO("reload OK! \n");
			reload_status = 1;
			break;
		} else if (retry == 0) {
			TPD_INFO("reload 20 times! fail \n");
			return;
		} else {
			retry--;
			msleep(10);
			TPD_INFO("reload fail, delay 10ms retry=%d\n", retry);
		}
	}
	TPD_INFO("%s : data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n", __func__, data[0], data[1], data[2], data[3]);
	TPD_INFO("reload_status=%d\n", reload_status);

	hx83102j_sense_off(chip_info);

	/*=====================================
		Read FW version : 0x1000_7004  but 05,06 are the real addr for FW Version
	=====================================*/

	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x04;
	hx83102j_register_read(chip_info, cmd, 4, data, false);


	TPD_INFO("PANEL_VER : %X \n", data[0]);
	TPD_INFO("FW_VER : %X \n", data[1] << 8 | data[2]);

	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x84;
	hx83102j_register_read(chip_info, cmd, 4, data, false);

	TPD_INFO("CFG_VER : %X \n", data[2] << 8 | data[3]);
	TPD_INFO("TOUCH_VER : %X \n", data[2]);
	TPD_INFO("DISPLAY_VER : %X \n", data[3]);
	snprintf(dev_version, MAX_DEVICE_VERSION_LENGTH, "%02X", data[2]);
#ifndef HIMAX_DBG
	if (ts->panel_data.manufacture_info.version) {
		if (ts->panel_data.vid_len == 0) {
			strlcpy(&(ts->panel_data.manufacture_info.version[12]), dev_version, 3);
		} else {
			ver_len = ts->panel_data.vid_len;
			if (ver_len > MAX_DEVICE_VERSION_LENGTH - 4) {
				ver_len = MAX_DEVICE_VERSION_LENGTH - 4;
			}

			strlcpy(&ts->panel_data.manufacture_info.version[ver_len],
					dev_version, MAX_DEVICE_VERSION_LENGTH - ver_len);
		}
	}
	TPD_INFO("manufacture_info.version: %s\n", ts->panel_data.manufacture_info.version);
#endif

	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x00;
	hx83102j_register_read(chip_info, cmd, 4, data, false);
	TPD_INFO("CID_VER : %X \n", (data[2] << 8 | data[3]));
	return;
}
#ifdef HIMAX_DBG
static void hx83102j_check_reload(struct chip_data_hx83102j *chip_info)
{
	uint8_t cmd[4];
	uint8_t data[4];
	uint8_t data2[4];
	int retry = 20;
	int reload_status = 0;

	while (reload_status == 0) {
		cmd[3] = 0x10;	/* oplus fw id bin address : 0xc014	  , 49172	 Tp ic address : 0x 10007014*/
		cmd[2] = 0x00;
		cmd[1] = 0x7f;
		cmd[0] = 0x00;
		hx83102j_register_read(chip_info, cmd, 4, data, false);

		cmd[3] = 0x10;
		cmd[2] = 0x00;
		cmd[1] = 0x72;
		cmd[0] = 0xc0;
		hx83102j_register_read(chip_info, cmd, 4, data2, false);

		if ((data[1] == 0x3A && data[0] == 0xA3) || (data2[1] == 0x72
				&& data2[0] == 0xc0)) {
			TPD_INFO("reload OK! \n");
			reload_status = 1;
			break;

		} else if (retry == 0) {
			TPD_INFO("reload 20 times! fail \n");
			return;

		} else {
			retry--;
			msleep(10);
			TPD_INFO("reload fail, delay 10ms retry=%d\n", retry);
		}
	}

	TPD_INFO("%s : data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n",
		 __func__, data[0], data[1], data[2], data[3]);
	TPD_INFO("reload_status=%d\n", reload_status);
}
#endif
static void hx83102j_read_OPLUS_FW_ver(struct chip_data_hx83102j *chip_info)
{
	uint8_t cmd[4];
	uint8_t data[4];
	uint8_t data2[4];
	uint32_t touch_ver = 0;
	int retry = 200;
	int reload_status = 0;
	u8 ver_len = 0;
	char dev_version[MAX_DEVICE_VERSION_LENGTH] = {0};
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	hx83102j_sense_on(chip_info, 0);

	while (reload_status == 0) {
		cmd[3] = 0x10;  /* oplus fw id bin address : 0xc014   , 49172    Tp ic address : 0x 10007014*/
		cmd[2] = 0x00;
		cmd[1] = 0x7f;
		cmd[0] = 0x00;
		hx83102j_register_read(chip_info, cmd, 4, data, false);

		cmd[3] = 0x10;
		cmd[2] = 0x00;
		cmd[1] = 0x72;
		cmd[0] = 0xc0;
		hx83102j_register_read(chip_info, cmd, 4, data2, false);

		if ((data[1] == 0x3A && data[0] == 0xA3) || (data2[1] == 0x72
				&& data2[0] == 0xc0)) {
			TPD_INFO("reload OK! \n");
			reload_status = 1;
			break;

		} else if (retry == 0) {
			TPD_INFO("reload 20 times! fail \n");
			return;

		} else {
			retry--;
			msleep(10);
			TPD_INFO("reload fail, delay 10ms retry=%d\n", retry);
			hx_check_fw_status(chip_info);
		}
	}

	TPD_INFO("%s : data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n",
		 __func__, data[0], data[1], data[2], data[3]);
	TPD_INFO("reload_status=%d\n", reload_status);

	hx83102j_sense_off(chip_info);

	cmd[3] = 0x10;  /*oplus fw id bin address : 0xc014    Tp ic address : 0x 10007014*/
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x14;
	hx83102j_register_read(chip_info, cmd, 4, data, false);
	if (data[0] == 0xBD && data[1] == 0x12)
		chip_info->isbd12proj = true;
	if (data[0] == 0xEA && data[1] == 0x00 && (data[2] & 0xF0) == 0x60)
		chip_info->isea006proj = true;
	chip_info->vendor_proj_info = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
	TPD_INFO("%s : data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n", __func__, data[0], data[1], data[2], data[3]);

	cmd[3] = 0x10;  /*oplus fw id bin address : 0xc014    Tp ic address : 0x 10007014*/
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x84;
	hx83102j_register_read(chip_info, cmd, 4, data, false);
	touch_ver = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
	TPD_INFO("%s :touch_ver = 0x%08X\n", __func__, touch_ver);
	snprintf(dev_version, MAX_DEVICE_VERSION_LENGTH, "%02X", touch_ver>>8);
#ifndef HIMAX_DBG
	if (ts->panel_data.manufacture_info.version) {
		if (ts->panel_data.vid_len == 0) {
			strlcpy(&(ts->panel_data.manufacture_info.version[12]), dev_version, 3);
		} else {
			ver_len = ts->panel_data.vid_len;
			if (ver_len > MAX_DEVICE_VERSION_LENGTH - 4) {
				ver_len = MAX_DEVICE_VERSION_LENGTH - 4;
			}
			strlcpy(&ts->panel_data.manufacture_info.version[ver_len],
					dev_version, MAX_DEVICE_VERSION_LENGTH - ver_len);
		}
	}
	TPD_INFO("manufacture_info.version: %s\n", ts->panel_data.manufacture_info.version);
#endif

	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x00;
	hx83102j_register_read(chip_info, cmd, 4, data, false);
	chip_info->fw_cid_ver = data[2] << 8 | data[3];
	chip_info->touch_ver = touch_ver >> 8;
	TPD_INFO("%s :fw_cid_ver = 0x%04X touch_ver = 0x%04X\n", __func__, chip_info->fw_cid_ver, chip_info->touch_ver);
	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x04;
	hx83102j_register_read(chip_info, cmd, 4, data, false);
	chip_info->vendor_panel_ver = data[0];
	chip_info->vendor_fw_ver = data[1] << 8 | data[2];
	TPD_INFO("%s :vendor_panel_ver = 0x%04X vendor_fw_ver = 0x%04X\n", __func__, chip_info->vendor_panel_ver, chip_info->vendor_fw_ver);
	return;
}

static uint32_t hx83102j_hw_check_crc(struct chip_data_hx83102j *chip_info, uint8_t *start_addr, int reload_length)
{
	uint32_t result = 0;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int cnt = 0;
	int length = reload_length / 4;

	/*0x8005_0020 <= from, 0x8005_0028 <= 0x0099_length*/
	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x05;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x20;

	hx83102j_flash_write_burst(chip_info, tmp_addr, start_addr);

	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x05;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x28;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x99;
	tmp_data[1] = (length >> 8);
	tmp_data[0] = length;
	hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

	cnt = 0;
	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x05;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x00;
	do {
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

		if ((tmp_data[0] & 0x01) != 0x01) {
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x05;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x18;
			hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
			TPD_DETAIL("%s: tmp_data[3]=%X, tmp_data[2]=%X, tmp_data[1]=%X, tmp_data[0]=%X  \n", __func__, tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
			result = ((tmp_data[3] << 24) + (tmp_data[2] << 16) + (tmp_data[1] << 8) + tmp_data[0]);
			break;
		}
	} while (cnt++ < 100);

	return result;
}

#ifndef HX_ZERO_FLASH
static bool hx83102j_calculateChecksum(struct chip_data_hx83102j *chip_info, bool change_iref)
{
	uint8_t crc_result = 0;
	uint8_t tmp_data[4];

	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;

	crc_result = hx83102j_hw_check_crc(chip_info, tmp_data, FW_SIZE_64K);

	msleep(50);

	return !crc_result;
}
#endif
static int cal_data_len(int raw_cnt_rmd, int HX_MAX_PT, int raw_cnt_max)
{
	int rawdatalen;
	if (raw_cnt_rmd != 0x00) {
		rawdatalen = 128 - ((HX_MAX_PT + raw_cnt_max + 3) * 4) - 1;
	} else {
		rawdatalen = 128 - ((HX_MAX_PT + raw_cnt_max + 2) * 4) - 1;
	}
	return rawdatalen;
}


static int hx83102j_report_data_init(struct chip_data_hx83102j *chip_info, int max_touch_point, int tx_num, int rx_num)
{
	if (chip_info->hx_touch_data->hx_coord_buf != NULL) {
		kfree(chip_info->hx_touch_data->hx_coord_buf);
		chip_info->hx_touch_data->hx_coord_buf = NULL;
	}

	if (chip_info->hx_touch_data->diag_mutual != NULL) {
		kfree(chip_info->hx_touch_data->diag_mutual);
		chip_info->hx_touch_data->diag_mutual = NULL;
	}

	chip_info->hx_touch_data->event_size = 128;

	chip_info->hx_touch_data->touch_all_size = 128;

	chip_info->hx_touch_info_point_cnt = max_touch_point * 4;

	if ((max_touch_point % 4) == 0)
		chip_info->hx_touch_info_point_cnt += (max_touch_point / 4) * 4;
	else
		chip_info->hx_touch_info_point_cnt += ((max_touch_point / 4) + 1) * 4;

	chip_info->hx_touch_data->raw_cnt_max = max_touch_point / 4;
	chip_info->hx_touch_data->raw_cnt_rmd = max_touch_point % 4;

	if (chip_info->hx_touch_data->raw_cnt_rmd != 0x00) {
		chip_info->hx_touch_data->rawdata_size = cal_data_len(chip_info->hx_touch_data->raw_cnt_rmd, max_touch_point, chip_info->hx_touch_data->raw_cnt_max);
		chip_info->hx_touch_data->touch_info_size = (max_touch_point + chip_info->hx_touch_data->raw_cnt_max + 2) * 4;
	} else {
		chip_info->hx_touch_data->rawdata_size = cal_data_len(chip_info->hx_touch_data->raw_cnt_rmd, max_touch_point, chip_info->hx_touch_data->raw_cnt_max);
		chip_info->hx_touch_data->touch_info_size = (max_touch_point + chip_info->hx_touch_data->raw_cnt_max + 1) * 4;
	}
#if defined(HX_ALG_OVERLAY)
		chip_info->hx_touch_data->touch_info_size += STYLUS_INFO_SZ;
		chip_info->hx_touch_data->rawdata_size -= STYLUS_INFO_SZ;
#endif
	if ((tx_num * rx_num + tx_num + rx_num) % chip_info->hx_touch_data->rawdata_size == 0) {
		chip_info->hx_touch_data->rawdata_frame_size = (tx_num * rx_num + tx_num + rx_num) / chip_info->hx_touch_data->rawdata_size;
	} else {
		chip_info->hx_touch_data->rawdata_frame_size = (tx_num * rx_num + tx_num + rx_num) / chip_info->hx_touch_data->rawdata_size + 1;
	}
	TPD_INFO("%s: rawdata_frame_size = %d ", __func__, chip_info->hx_touch_data->rawdata_frame_size);
	TPD_INFO("%s: max_touch_point:%d, hx_raw_cnt_max:%d, hx_raw_cnt_rmd:%d, g_hx_rawdata_size:%d, chip_info->hx_touch_data->touch_info_size:%d\n",
			__func__, max_touch_point, chip_info->hx_touch_data->raw_cnt_max, chip_info->hx_touch_data->raw_cnt_rmd,
			chip_info->hx_touch_data->rawdata_size, chip_info->hx_touch_data->touch_info_size);

	if (chip_info->hx_touch_data->hx_coord_buf == NULL)
		chip_info->hx_touch_data->hx_coord_buf = kzalloc(sizeof(uint8_t) * (chip_info->hx_touch_data->touch_info_size), GFP_KERNEL);
	if (chip_info->hx_touch_data->hx_coord_buf == NULL) {
		goto mem_alloc_fail;
	}

	if (chip_info->hx_touch_data->diag_mutual == NULL)
		chip_info->hx_touch_data->diag_mutual = kzalloc(tx_num * rx_num * sizeof(int32_t), GFP_KERNEL);
	if (chip_info->hx_touch_data->diag_mutual == NULL) {
		goto mem_alloc_fail;
	}

	if (chip_info->hx_touch_data->hx_rawdata_buf == NULL)
		chip_info->hx_touch_data->hx_rawdata_buf =
			kzalloc(sizeof(uint8_t) * (chip_info->hx_touch_data->touch_all_size - chip_info->hx_touch_data->touch_info_size), GFP_KERNEL);
	if (chip_info->hx_touch_data->hx_rawdata_buf == NULL) {
		goto mem_alloc_fail;
	}

	if (chip_info->hx_touch_data->hx_event_buf == NULL)
		chip_info->hx_touch_data->hx_event_buf = kzalloc(sizeof(uint8_t) * (chip_info->hx_touch_data->event_size), GFP_KERNEL);
	if (chip_info->hx_touch_data->hx_event_buf == NULL) {
		goto mem_alloc_fail;
	}

	return NO_ERR;

mem_alloc_fail:
	if (chip_info->hx_touch_data->hx_coord_buf) {
		kfree(chip_info->hx_touch_data->hx_coord_buf);
		chip_info->hx_touch_data->hx_coord_buf = NULL;
	}

	if (chip_info->hx_touch_data->hx_rawdata_buf) {
		kfree(chip_info->hx_touch_data->hx_rawdata_buf);
		chip_info->hx_touch_data->hx_rawdata_buf = NULL;
	}

	if (chip_info->hx_touch_data->hx_event_buf) {
		kfree(chip_info->hx_touch_data->hx_event_buf);
		chip_info->hx_touch_data->hx_event_buf = NULL;
	}

	if (chip_info->hx_touch_data->diag_mutual) {
		kfree(chip_info->hx_touch_data->diag_mutual);
		chip_info->hx_touch_data->diag_mutual = NULL;
	}
	TPD_INFO("%s: Memory allocate fail!\n", __func__);
	return MEM_ALLOC_FAIL;
}

static bool hx83102j_read_event_stack(struct chip_data_hx83102j *chip_info, uint8_t *buf, uint8_t length)
{
	uint8_t cmd[4];

	/*AHB_I2C Burst Read Off*/
	cmd[0] = 0x00;
	if (hx83102j_bus_write(chip_info, 0x11, 1, cmd) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return 0;
	}

	hx83102j_bus_read(chip_info, 0x30, length, buf);

	/*AHB_I2C Burst Read On*/
	cmd[0] = 0x01;
	if (hx83102j_bus_write(chip_info, 0x11, 1, cmd) < 0) {
		TPD_INFO("%s: i2c access fail!\n", __func__);
		return 0;
	}

	return 1;
}

static int hx83102j_ic_esd_recovery(struct chip_data_hx83102j *chip_info, int hx_esd_event, int hx_zero_event, int length)
{
	if (hx_esd_event == length) {
		chip_info->g_zero_event_count = 0;
		goto checksum_fail;
	} else if (hx_zero_event == length) {
		chip_info->g_zero_event_count++;
		TPD_INFO("[HIMAX TP MSG]: ALL Zero event is %d times.\n", chip_info->g_zero_event_count);
		if (chip_info->g_zero_event_count > 2) {
			chip_info->g_zero_event_count = 0;
			TPD_INFO("[HIMAX TP MSG]: ESD event checked - ALL Zero.\n");
			goto checksum_fail;
		}
		goto err_workqueue_out;
	}

checksum_fail:
	return CHECKSUM_FAIL;
err_workqueue_out:
	return WORK_OUT;
}
/*Himax DB Start*/
/*
static int hx83102j_lcd_resetgpio_set(struct hw_resource *hw_res, bool on)
{
	int ret = 0;
		if (gpio_is_valid(hw_res->lcd_reset_gpio)) {
		TPD_DETAIL("Set the lcd_reset_gpio on=%d \n", on);
		ret = gpio_direction_output(hw_res->lcd_reset_gpio, on);
		if (ret) {
			TPD_INFO("Set the lcd_reset_gpio on=%d fail\n", on);
		} else {

		}
		usleep_range(5000, 5100);
		TPD_DETAIL("%s hw_res->lcd_reset_gpio = %d\n", __func__, hw_res->lcd_reset_gpio);
		gpio_free(hw_res->lcd_reset_gpio);
		TPD_INFO("%s: free lcd reset pin\n", __func__);
	}

	return ret;
}*/

/*Himax DB End*/

#ifdef HX_RST_PIN_FUNC
static int hx83102j_resetgpio_set(struct hw_resource *hw_res, bool on)
{
	int ret = 0;
	if (gpio_is_valid(hw_res->reset_gpio)) {
		TPD_DETAIL("Set the reset_gpio on=%d \n", on);
		ret = gpio_direction_output(hw_res->reset_gpio, on);
		if (ret) {
			TPD_INFO("Set the reset_gpio on=%d fail\n", on);
		}
		msleep(RESET_TO_NORMAL_TIME);
		TPD_DETAIL("%s hw_res->reset_gpio = %d\n", __func__, hw_res->reset_gpio);
	}

	return ret;
}
#endif

static void hx83102j_esd_hw_reset(struct chip_data_hx83102j *chip_info)
{
	int ret = 0;
	int load_fw_times = 10;

	TPD_DETAIL("START_Himax TP: ESD - Reset\n");
#ifdef HX_ZERO_FLASH
	mutex_lock(&(chip_info->fw_update_lock));
#endif
	hx83102j_enable_interrupt(chip_info, false);

	do {
	#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(chip_info->hw_res, false);
		usleep_range(5000, 5001);
		hx83102j_resetgpio_set(chip_info->hw_res, true);
		usleep_range(5000, 5001);
	#else
		chip_info->g_core_fp.fp_sys_reset(chip_info);
	#endif
		TPD_DETAIL("%s: ESD reset finished\n", __func__);

		TPD_DETAIL("It will update fw after esd event in zero flash mode!\n");

		load_fw_times--;
		chip_info->g_core_fp.fp_0f_operation_dirly(chip_info);
		ret = chip_info->g_core_fp.fp_reload_disable(chip_info, 0);
	} while (!ret && load_fw_times > 0);

	if (!load_fw_times) {
		TPD_INFO("%s: load_fw_times over 10 times\n", __func__);
	}
	hx83102j_read_OPLUS_FW_ver(chip_info);
	hx83102j_sense_on(chip_info, 0);
	msleep(10);
	/* need_modify*/
	/* report all leave event
	hx83102j_report_all_leave_event(ts);*/

	hx83102j_enable_interrupt(chip_info, true);

#ifdef HX_ZERO_FLASH
	mutex_unlock(&(chip_info->fw_update_lock));
#endif
}

void hx83102j_esd_wdt_reset(struct chip_data_hx83102j *chip_info)
{
	int retry = 0;
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};

	if (chip_info->wdt_event_count == 3) {
		hx83102j_sense_off(chip_info);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x9C;

		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0xDD;
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x28;

		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0xA5;
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

		tmp_addr[3] = 0x30;
		tmp_addr[2] = 0x0B;
		tmp_addr[1] = 0x90;
		tmp_addr[0] = 0x00;

		tmp_data[3] = 0x2E;
		tmp_data[2] = 0x10;
		tmp_data[1] = 0x83;
		tmp_data[0] = 0x00;
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

		tmp_addr[3] = 0x30;
		tmp_addr[2] = 0x0E;
		tmp_addr[1] = 0xB0;
		tmp_addr[0] = 0x00;

		tmp_data[3] = 0xCC;
		tmp_data[2] = 0x66;
		tmp_data[1] = 0x55;
		tmp_data[0] = 0x00;
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

		do {
			tmp_addr[3] = 0x30;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0xE0;
			tmp_addr[0] = 0x01;
			memset(tmp_data, 0, sizeof(tmp_data));
			hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

			TPD_INFO("Before trigger,retry:%d, R%02X%02X%02X%02XH = 0x%02X%02X%02X%02X\n",
					retry, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0],
					tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);

			tmp_addr[3] = 0x30;
			tmp_addr[2] = 0x03;
			tmp_addr[1] = 0x40;
			tmp_addr[0] = 0x00;

			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

			usleep_range(1000, 1100);

			tmp_addr[3] = 0x30;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0xE0;
			tmp_addr[0] = 0x01;
			memset(tmp_data, 0, sizeof(tmp_data));
			hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

			TPD_INFO("After trigger,retry:%d, R%02X%02X%02X%02XH = 0x%02X%02X%02X%02X\n",
					retry, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0],
					tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);

			if ((tmp_data[0] == tmp_data[1]) && (tmp_data[1] == tmp_data[2]) && (tmp_data[2] == tmp_data[3])) {
				if (tmp_data[0] == 0x00) {
					TPD_INFO("%s: EnteringTE!\n", __func__);
					chip_info->wdt_event_count = 0;
					break;
				}
			}
		} while ((retry++ < 5));
		hx83102j_sense_on(chip_info, 0x00);
	}
}

static int hx83102j_checksum_cal(struct chip_data_hx83102j *chip_info, uint8_t *buf, int ts_status)
{
	int hx_eb_event = 0;
	int hx_ec_event = 0;
	int hx_ed_event = 0;
	int hx_ee_event = 0;
	int hx_esd_event = 0;
	int hx_zero_event = 0;
	int shaking_ret = 0;

	uint16_t check_sum_cal = 0;
	int32_t loop_i = 0;
	int length = 0;
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};

	/* Normal */
	if (ts_status == HX_REPORT_COORD) {
		length = chip_info->hx_touch_data->touch_info_size;
	} else if (ts_status == HX_REPORT_SMWP_EVENT) {
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
	} else {
		TPD_INFO("%s, Neither Normal Nor SMWP error!\n", __func__);
	}
	if (chip_info->hx_esd_reset_activate) {
		/* drop 1st interrupts after chip reset */
		chip_info->hx_esd_reset_activate = 0;
		TPD_INFO("[hx_esd_reset_activate]:%s: Back from reset, ready to serve.\n", __func__);
		goto checksum_fail;
	}

	for (loop_i = 0; loop_i < length; loop_i++) {
		check_sum_cal += buf[loop_i];
		/* #ifdef HX_ESD_RECOVERY  */
		if (ts_status == HX_REPORT_COORD || ts_status == HX_REPORT_SMWP_EVENT) {
			/* case 1 ESD recovery flow */
			if (buf[loop_i] == 0xEB) {
				hx_eb_event++;
			} else if (buf[loop_i] == 0xEC) {
				hx_ec_event++;
			} else if (buf[loop_i] == 0xED) {
				hx_ed_event++;
			} else if (buf[loop_i] == 0x00) {/* case 2 ESD recovery flow-Disable */
				hx_zero_event++;
			} else {
				hx_eb_event = 0;
				hx_ec_event = 0;
				hx_ed_event = 0;
				hx_ee_event = 0;
				hx_zero_event = 0;
				chip_info->g_zero_event_count = 0;
				chip_info->wdt_event_count = 0;
			}

			if (hx_eb_event == length) {
				hx_esd_event = length;
				chip_info->hx_eb_event_flag++;
				TPD_INFO("[HIMAX TP MSG]: ESD event checked - ALL 0xEB.\n");
			} else if (hx_ec_event == length) {
				hx_esd_event = length;
				chip_info->hx_ec_event_flag++;
				TPD_INFO("[HIMAX TP MSG]: ESD event checked - ALL 0xEC.\n");
			} else if (hx_zero_event == length) {
				TPD_INFO("[HIMAX TP MSG]: ALL 0x00.\n");

				tmp_addr[3] = 0x90;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x00;
				tmp_addr[0] = 0xE4;
				hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
				TPD_INFO("%s: 0x900000E4, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

				if (tmp_data[0] == 0x10) {
					chip_info->g_zero_event_count = 0;
					chip_info->wdt_event_count++;
					TPD_INFO("[HIMAX TP MSG]: WDT event is %d times.\n", chip_info->wdt_event_count);
					hx83102j_esd_hw_reset(chip_info);
					hx83102j_esd_wdt_reset(chip_info);
					goto checksum_fail;
				} else {
					chip_info->wdt_event_count = 0;
				}

			} else if (hx_ed_event == length) {
				hx_esd_event = length;
				chip_info->hx_ed_event_flag++;
				TPD_INFO("[HIMAX TP MSG]: EXCPT event checked - ALL 0xED.\n");

			} else {
				hx_esd_event = 0;
			}
		}
		/* #endif */
	}

	if (ts_status == HX_REPORT_COORD) {
		if (hx_esd_event == length || hx_zero_event == length) {
			shaking_ret = hx83102j_ic_esd_recovery(chip_info, hx_esd_event, hx_zero_event, length);
			if (shaking_ret == CHECKSUM_FAIL) {
				hx83102j_esd_hw_reset(chip_info);
				goto checksum_fail;
			} else if (shaking_ret == ERR_WORK_OUT) {
				goto err_workqueue_out;
			} else {
				goto workqueue_out;
			}
		} else if (chip_info->hx_hw_reset_activate) {
			/* drop 1st interrupts after chip reset */
			chip_info->hx_hw_reset_activate = 0;
			TPD_INFO("[chip_info->hx_hw_reset_activate]:%s: Back from reset, ready to serve.\n", __func__);
			goto checksum_fail;
		}
	}

	if ((check_sum_cal % 0x100 != 0)) {
		TPD_INFO("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);
		goto workqueue_out;
	}

	/* TPD_INFO("%s:End\n",__func__); */
	return NO_ERR;

/*ready_to_serve:
	return READY_TO_SERVE;*/
checksum_fail:
	return CHECKSUM_FAIL;
err_workqueue_out:
	return ERR_WORK_OUT;
workqueue_out:
	return WORK_OUT;
}

static void hx83102j_log_touch_data(uint8_t *buf, struct chip_data_hx83102j *chip_info)
{
	int loop_i = 0;
	int print_size = 0;

	if (!chip_info->hx_touch_data->diag_cmd) {
		print_size = chip_info->hx_touch_data->touch_info_size;
	} else {
		print_size = chip_info->hx_touch_data->touch_all_size;
	}

	for (loop_i = 0; loop_i < print_size; loop_i++) {
		TPD_INFO("p[%d] = 0x%02X ", loop_i, buf[loop_i]);
		if((loop_i + 1) % 8 == 0) {
			TPD_INFO("\n");
		}
		if (loop_i == (print_size - 1)) {
			TPD_INFO("\n");
		}
	}
}
#ifndef HX_ZERO_FLASH
static void hx83102j_idle_mode(struct chip_data_hx83102j *chip_info, int disable)
{
	int retry = 20;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t switch_cmd = 0x00;

	TPD_INFO("%s:entering\n", __func__);
	do {
		TPD_INFO("%s,now %d times\n!", __func__, retry);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x70;
		tmp_addr[0] = 0x88;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

		if (disable)
			switch_cmd = 0x17;
		else
			switch_cmd = 0x1F;

		tmp_data[0] = switch_cmd;
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s:After turn ON/OFF IDLE Mode [0] = 0x%02X, [1] = 0x%02X, [2] = 0x%02X, [3] = 0x%02X\n", __func__,
				tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		retry--;
		msleep(10);
	} while ((tmp_data[0] != switch_cmd) && retry > 0);

	TPD_INFO("%s: setting OK!\n", __func__);
}
#endif
#ifndef HX_ZERO_FLASH
static void hx83102j_reload_disable(struct chip_data_hx83102j *chip_info, int on)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	TPD_INFO("%s:entering\n", __func__);

	if (on) {/*reload disable*/
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x00;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0xA5;
		tmp_data[0] = 0x5A;
	} else {/*reload enable*/
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x00;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;
	}

	hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

	TPD_INFO("%s: setting OK!\n", __func__);
}
#endif

static int hx83102j_test_data_pop_out(struct chip_data_hx83102j *chip_info, int data_type, char *g_Test_list_log,
			char *g_company_info_log, char *g_project_test_info_log, char *rslt_buf, char *filepath)
{
	struct file *raw_file = NULL;
	char *line = "=================================================\n";
	char *company = "Himax for OPLUS: Driver Seneor Test\n";
	char *info = "Test Info as follow\n";
	char *project_name_log = "OPLUS_";
	int ret_val = NO_ERR;
	int real_sz = 0;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	TPD_INFO("%s: Entering!\n", __func__);
	TPD_INFO("data size = 0x%04X\n", (uint32_t)strlen(rslt_buf));

	/*Company Info*/
	snprintf(g_company_info_log, 160, "%s%s%s%s", line, company, info, line);
	TPD_DETAIL("%s 000: %s \n", __func__, g_company_info_log);

	TPD_INFO("%s: try to mkdir!\n", __func__);
#ifndef HIMAX_DBG
#ifdef CONFIG_ARCH_HAS_SYSCALL_WRAPPER
	/*
	ksys_mkdir("/sdcard/TpTestReport", S_IRUGO | S_IWUSR);
	ksys_mkdir("/sdcard/TpTestReport/screenOn", S_IRUGO | S_IWUSR);
	ksys_mkdir("/sdcard/TpTestReport/screenOn/NG", S_IRUGO | S_IWUSR);
	ksys_mkdir("/sdcard/TpTestReport/screenOn/OK", S_IRUGO | S_IWUSR);
	ksys_mkdir("/sdcard/TpTestReport/screenOff", S_IRUGO | S_IWUSR);
	ksys_mkdir("/sdcard/TpTestReport/screenOff/NG", S_IRUGO | S_IWUSR);
	ksys_mkdir("/sdcard/TpTestReport/screenOff/OK", S_IRUGO | S_IWUSR);
	*/
#else

	sys_mkdir("/sdcard/TpTestReport", S_IRUGO | S_IWUSR);
	sys_mkdir("/sdcard/TpTestReport/screenOn", S_IRUGO | S_IWUSR);
	sys_mkdir("/sdcard/TpTestReport/screenOn/NG", S_IRUGO | S_IWUSR);
	sys_mkdir("/sdcard/TpTestReport/screenOn/OK", S_IRUGO | S_IWUSR);
	sys_mkdir("/sdcard/TpTestReport/screenOff", S_IRUGO | S_IWUSR);
	sys_mkdir("/sdcard/TpTestReport/screenOff/NG", S_IRUGO | S_IWUSR);
	sys_mkdir("/sdcard/TpTestReport/screenOff/OK", S_IRUGO | S_IWUSR);

#endif
#endif

	/*project Info*/
/*Himax_DB_Test Start*/
	/*
	snprintf(g_project_test_info_log, 118, "Project_name: %s%d\nFW_ID: %8X\nFW_Ver: %4X\nPanel Info: TX_Num=%d RX_Num=%d\nTest stage: Mobile\n",
			project_name_log, get_project(), chip_info->fw_id, chip_info->fw_ver, chip_info->hw_res->tx_num, chip_info->hw_res->rx_num);
	*/
	snprintf(g_project_test_info_log, 118, "Project_name: %s\nFW_ID: %8X\nFW_Ver: %4X\nPanel Info: TX_Num=%d RX_Num=%d\nTest stage: Mobile\n",
			project_name_log, chip_info->vendor_proj_info, chip_info->fw_cid_ver, chip_info->hw_res->tx_num, chip_info->hw_res->rx_num);

/*Himax_DB_Test End*/
	TPD_DETAIL("%s 001: %s \n", __func__, g_project_test_info_log);

	if (IS_ERR(raw_file)) {
		TPD_INFO("%s open file failed = %ld\n", __func__, PTR_ERR(raw_file));
		ret_val = -EIO;
		goto SAVE_DATA_ERR;
	}
	/*
	fs = get_fs();
	set_fs(get_ds());
	vfs_write(raw_file, g_company_info_log, (int)(strlen(g_company_info_log)), &pos);
	pos = pos + (int)(strlen(g_company_info_log));

	vfs_write(raw_file, g_project_test_info_log, (int)(strlen(g_project_test_info_log)), &pos);
	pos = pos + (int)(strlen(g_project_test_info_log));

	vfs_write(raw_file, g_test_list_log, (int)(strlen(g_test_list_log)), &pos);
	pos = pos + (int)(strlen(g_test_list_log));

	vfs_write(raw_file, rslt_buf, chip_info->g_1kind_raw_size * chip_info->hx_criteria_item * sizeof(char), &pos);
	if (raw_file != NULL)
		filp_close(raw_file, NULL);

	set_fs(fs);
	*/
	real_sz = chip_info->g_rslt_data_len;
	if (data_type == HX_AUTO_TEST) {
		ts->com_test_data.result_cur_len = chip_info->g_rslt_data_len;
		memcpy(&ts->com_test_data.result_data[0], &chip_info->g_rslt_data[0], real_sz * sizeof(char));
		TPD_INFO("%s: Auto test result, size=%d, content=%s\n", __func__, ts->com_test_data.result_cur_len, ts->com_test_data.result_data);
	} else if (data_type == HX_BS_TEST) {
		ts->com_test_data.bs_result_cur_len = chip_info->g_rslt_data_len;
		memcpy(&ts->com_test_data.bs_result_data[0], &chip_info->g_rslt_data[0], real_sz * sizeof(char));
		TPD_INFO("%s: BS test result, size=%d, content=%s\n", __func__, ts->com_test_data.bs_result_cur_len, ts->com_test_data.bs_result_data);
	}

SAVE_DATA_ERR:
	TPD_INFO("%s: End!\n", __func__);
	return ret_val;
}


static int hx_test_data_get(struct chip_data_hx83102j *chip_info, uint32_t RAW[], char *start_log, char *result, int now_item)
{
	uint32_t i;

	ssize_t len = 0;
	char *testdata = NULL;
	uint32_t SZ_SIZE = chip_info->g_1kind_raw_size;

	TPD_INFO("%s: Entering, Now type=%s!\n", __func__,
			 g_himax_inspection_mode[now_item]);

	testdata = kzalloc(sizeof(char) * SZ_SIZE, GFP_KERNEL);
	if (!testdata) {
		TPD_INFO("%s:%d testdata kzalloc buf error\n", __func__, __LINE__);
		return -ENOMEM;
	}
	len += snprintf((testdata + len), SZ_SIZE - len, "%s", result);
	len += snprintf((testdata + len), SZ_SIZE - len, "%s", start_log);
	for (i = 0; i < chip_info->hw_res->tx_num * chip_info->hw_res->rx_num; i++) {
		if (i > 1 && ((i + 1) % chip_info->hw_res->rx_num) == 0)
			len += snprintf((testdata + len), SZ_SIZE - len, "%5d,\n", RAW[i]);
		else
			len += snprintf((testdata + len), SZ_SIZE - len,
							"%5d,", RAW[i]);
	}

	memcpy(&chip_info->g_rslt_data[chip_info->g_rslt_data_len], testdata, len);
	chip_info->g_rslt_data_len += len;
	TPD_INFO("%s: chip_info->g_rslt_data_len=%d!\n", __func__, chip_info->g_rslt_data_len);

	if (testdata)
		kfree(testdata);
	TPD_INFO("%s: End!\n", __func__);
	return NO_ERR;
}


static int hx83102j_get_rawdata(struct chip_data_hx83102j *chip_info, uint32_t *RAW, uint32_t datalen)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t *tmp_rawdata;
	uint32_t retry = 0;
	uint16_t checksum_cal;
	uint32_t i = 0;

	uint8_t max_i2c_size = MAX_RECVS_SZ;
	int address = 0;
	int total_read_times = 0;
	int total_size = datalen * 2 + 4;
	int total_size_temp;
	uint32_t j = 0;
	uint32_t index = 0;
	uint32_t Min_DATA = 0xFFFFFFFF;
	uint32_t Max_DATA = 0x00000000;

	TPD_INFO("%s: Entering!\n", __func__);
	/*1 Set Data Ready PWD*/
	while (retry < 200) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = Data_PWD1;
		tmp_data[0] = Data_PWD0;
		hx83102j_flash_write_burst_length(chip_info, tmp_addr, tmp_data, 4);
		usleep_range(1000, 1001);
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: [3]=0x%02X,[2]=0x%02X,[1]=0x%02X,[0]=0x%02X\n", __func__,
			tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
		if ((tmp_data[0] == Data_PWD0 && tmp_data[1] == Data_PWD1) ||
			(tmp_data[0] == Data_PWD1 && tmp_data[1] == Data_PWD0)) {
			TPD_INFO("%s: write pwd OK!\n", __func__);
			break;
		} else {
			TPD_INFO("[ERR]%s: 000waiting pwd fail!\n", __func__);
		}

		retry++;
		msleep(1);
	}

	if (retry >= 200) {
		TPD_INFO("[ERR]%s: write pwd fail!\n", __func__);
		return RESULT_ERR;
	}

	retry = 0;

	while (retry < 400) {
		TPD_INFO("%s: wait times=%d, [3]=0x%02X,[2]=0x%02X,[1]=0x%02X,[0]=0x%02X\n", __func__, retry,
			tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
		if (tmp_data[0] == Data_PWD1 && tmp_data[1] == Data_PWD0) {
			TPD_INFO("%s: waiting pwd OK!\n", __func__);
			break;
		}

		retry++;
		msleep(1);
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
	}

	if (retry >= 400) {
		TPD_INFO("[ERR]%s: waiting pwd fail!\n", __func__);
		return RESULT_ERR;
	} else {
		retry = 0;
	}

	TPD_INFO("%s: allocate tmp_rawdata\n", __func__);
	tmp_rawdata = kzalloc(sizeof(uint8_t) * (total_size + 8), GFP_KERNEL);
	if (!tmp_rawdata) {
		TPD_INFO("[ERR]%s: rawdata buf allocate fail!\n", __func__);
		return RESULT_ERR;
	}

	/*2 Read Data from SRAM*/
	TPD_INFO("%s: re-arrange rawdata\n", __func__);
	while (retry < 10) {
		checksum_cal = 0;
		total_size_temp = total_size;
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;

		if (total_size % max_i2c_size == 0) {
			total_read_times = total_size / max_i2c_size;
		} else {
			total_read_times = total_size / max_i2c_size + 1;
		}

		for (i = 0; i < (total_read_times); i++) {
			if (total_size_temp >= max_i2c_size) {
				hx83102j_register_read(chip_info, tmp_addr, max_i2c_size, &tmp_rawdata[i * max_i2c_size], false);
				total_size_temp = total_size_temp - max_i2c_size;
			} else {
				hx83102j_register_read(chip_info, tmp_addr, total_size_temp % max_i2c_size, &tmp_rawdata[i * max_i2c_size], false);
			}

			address = ((i + 1) * max_i2c_size);
			tmp_addr[1] = (uint8_t)((address >> 8) & 0x00FF);
			tmp_addr[0] = (uint8_t)((address) & 0x00FF);
		}

		/*3 Check Checksum*/
		for (i = 2; i < total_size; i = i + 2) {
			checksum_cal += tmp_rawdata[i + 1] * 256 + tmp_rawdata[i];
		}

		if (checksum_cal == 0) {
			break;
		}

		retry++;
	}
	if (retry >= 10) {
		TPD_INFO("Retry over 10 times: do recovery\n");
		if (tmp_rawdata)
			kfree(tmp_rawdata);
		return RESULT_RETRY;
	}

	/*4 Copy Data*/
	TPD_INFO("%s: assign rawdata to target\n", __func__);
	for (i = 0; i < chip_info->hw_res->rx_num * chip_info->hw_res->tx_num; i++) {
		RAW[i] = ((int8_t)tmp_rawdata[(i * 2) + 1 + 4] * 256) + tmp_rawdata[(i * 2) + 4];
	}
/* cancel print information */
/*
	for (j = 0; j < g_chip_info->hw_res->tx_num; j++) {
		if (j == 0) {
			printk("	  RX%2d", j + 1);
		} else {
			printk("  RX%2d", j + 1);
		}
	}
	printk("\n");
*/
	for (i = 0; i < chip_info->hw_res->rx_num; i++) {
		/*printk("TX%2d", i + 1);*/
		for (j = 0; j < chip_info->hw_res->tx_num; j++) {
			index = ((chip_info->hw_res->rx_num * chip_info->hw_res->tx_num - i) - chip_info->hw_res->rx_num * j) - 1;

			/*printk("%6d", RAW[index]);*/

			if (RAW[index] > Max_DATA) {
				Max_DATA = RAW[index];
			}

			if (RAW[index] < Min_DATA) {
				Min_DATA = RAW[index];
			}
		}
		/*printk("\n");*/
	}

	TPD_INFO("Max = %5d, Min = %5d \n", Max_DATA, Min_DATA);

	if (tmp_rawdata)
		kfree(tmp_rawdata);

	return RESULT_OK;
}

static void hx83102j_switch_data_type(struct chip_data_hx83102j *chip_info, uint8_t checktype)
{
	uint8_t datatype = 0x00;

	switch (checktype) {
	case HIMAX_INSPECTION_OPEN:
		datatype = DATA_OPEN;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		datatype = DATA_MICRO_OPEN;
		break;
	case HIMAX_INSPECTION_SHORT:
		datatype = DATA_SHORT;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		datatype = DATA_RAWDATA;
		break;
	case HIMAX_INSPECTION_NOISE:
		datatype = DATA_NOISE;
		break;
	case HIMAX_INSPECTION_BACK_NORMAL:
		datatype = DATA_BACK_NORMAL;
		break;
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
		datatype = DATA_LPWUG_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWUG_NOISE:
		datatype = DATA_LPWUG_NOISE;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
		datatype = DATA_LPWUG_IDLE_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		datatype = DATA_LPWUG_IDLE_NOISE;
		break;
	default:
		TPD_INFO("Wrong type=%d\n", checktype);
		break;
	}
	hx83102j_diag_register_set(chip_info, datatype);
}

static int hx83102j_switch_mode(struct chip_data_hx83102j *chip_info, int mode)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	TPD_INFO("%s: Entering\n", __func__);

	/*Stop Handshaking*/
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x00;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	hx83102j_flash_write_burst_length(chip_info, tmp_addr, tmp_data, 4);

	/*Swtich Mode*/
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x7F;
	tmp_addr[0] = 0x04;
	switch (mode) {
	case HIMAX_INSPECTION_SORTING:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_SORTING_START;
		tmp_data[0] = PWD_SORTING_START;
		break;
	case HIMAX_INSPECTION_OPEN:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_OPEN_START;
		tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_OPEN_START;
		tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_SHORT:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_SHORT_START;
		tmp_data[0] = PWD_SHORT_START;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_RAWDATA_START;
		tmp_data[0] = PWD_RAWDATA_START;
		break;
	case HIMAX_INSPECTION_NOISE:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_NOISE_START;
		tmp_data[0] = PWD_NOISE_START;
		break;
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_LPWUG_START;
		tmp_data[0] = PWD_LPWUG_START;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = PWD_LPWUG_IDLE_START;
		tmp_data[0] = PWD_LPWUG_IDLE_START;
		break;
	default:
		TPD_INFO("%s: ERROR Wrong mode %d\n", __func__, mode);
		break;
	}
	hx83102j_flash_write_burst_length(chip_info, tmp_addr, tmp_data, 4);

	TPD_INFO("%s: End of setting!\n", __func__);

	return 0;
}


static void hx83102j_set_N_frame(struct chip_data_hx83102j *chip_info, uint16_t Nframe, uint8_t checktype)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x72;
	tmp_addr[0] = 0x94;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = (uint8_t)((Nframe & 0xFF00) >> 8);
	tmp_data[0] = (uint8_t)(Nframe & 0x00FF);
	hx83102j_flash_write_burst_length(chip_info, tmp_addr, tmp_data, 4);

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70;
	tmp_addr[0] = 0xF4;
	hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

	switch (checktype) {
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		tmp_data[0] = BS_LPWUG;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		tmp_data[0] = BS_LPWUG_IDLE;
		break;
	case HIMAX_INSPECTION_RAWDATA:
	case HIMAX_INSPECTION_NOISE:
		tmp_data[0] = BS_RAWDATANOISE;
		break;
	default:
		tmp_data[0] = BS_OPENSHORT;
		break;
	}
	hx83102j_flash_write_burst_length(chip_info, tmp_addr, tmp_data, 4);
}


static uint32_t hx83102j_check_mode(struct chip_data_hx83102j *chip_info, uint8_t checktype)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t wait_pwd[2];

	wait_pwd[0] = PWD_NONE;
	wait_pwd[1] = PWD_NONE;

	switch (checktype) {
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		wait_pwd[0] = PWD_LPWUG_END;
		wait_pwd[1] = PWD_LPWUG_END;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWUG_IDLE_END;
		wait_pwd[1] = PWD_LPWUG_IDLE_END;
		break;
	default:
		TPD_INFO("Wrong type=%d\n", checktype);
		break;
	}

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x7F;
	tmp_addr[0] = 0x04;
	hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
	TPD_INFO("%s: hx83102j_wait_sorting_mode, tmp_data[0]=%x, tmp_data[1]=%x\n", __func__, tmp_data[0], tmp_data[1]);

	if (wait_pwd[0] == tmp_data[0] && wait_pwd[1] == tmp_data[1]) {
		TPD_INFO("Change to mode=%s\n", g_himax_inspection_mode[checktype]);
		return 0;
	} else
		return 1;
}

static void hx83102j_get_noise_base(struct chip_data_hx83102j *chip_info)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t ratio, threshold, threshold_LPWUG;

	/*tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70;
	tmp_addr[0] = 0x8C;
	*/
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70;
	tmp_addr[0] = 0x94; /*ratio*/
	hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
	ratio = tmp_data[1];

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70;
	tmp_addr[0] = 0xA0; /*threshold_LPWUG*/
	hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
	threshold_LPWUG = tmp_data[0];

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70;
	tmp_addr[0] = 0x9C; /*threshold*/
	hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
	threshold = tmp_data[0];

	/*TPD_INFO("tmp_data[0]=0x%x tmp_data[1]=0x%x tmp_data[2]=0x%x tmp_data[3]=0x%x\n",
			  tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
	*/
	/*NOISEMAX = tmp_data[3]*(NOISE_P/256);*/
	g_noisemax = ratio * threshold;
	g_lpwug_noisemax = ratio * threshold_LPWUG;
	TPD_INFO("NOISEMAX=%d LPWUG_NOISE_MAX=%d \n", g_noisemax, g_lpwug_noisemax);
}

static int16_t hx83102j_get_noise_weight(struct chip_data_hx83102j *chip_info)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int16_t weight;

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x72;
	tmp_addr[0] = 0xC8;
	hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
	weight = ((int8_t)tmp_data[1] << 8) | tmp_data[0];
	TPD_INFO("%s: weight = %d ", __func__, weight);

	return weight;
}

static uint32_t hx83102j_wait_sorting_mode(struct chip_data_hx83102j *chip_info, uint8_t checktype)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t wait_pwd[2];
	uint8_t count = 0;

	wait_pwd[0] = PWD_NONE;
	wait_pwd[1] = PWD_NONE;

	switch (checktype) {
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		wait_pwd[0] = PWD_LPWUG_END;
		wait_pwd[1] = PWD_LPWUG_END;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWUG_IDLE_END;
		wait_pwd[1] = PWD_LPWUG_IDLE_END;
		break;
	default:
		TPD_INFO("Wrong type=%d\n", checktype);
		break;
	}

	do {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x04;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: hx83102j_wait_sorting_mode, tmp_data[0]=%x, tmp_data[1]=%x\n", __func__, tmp_data[0], tmp_data[1]);

		if (wait_pwd[0] == tmp_data[0] && wait_pwd[1] == tmp_data[1]) {
			TPD_INFO("%s: change mode done\n", __func__);
			return 0;
		} else
			TPD_INFO("%s: change mode fail in %d\n", __func__, count);

		hx_check_fw_status(chip_info);

		if (wait_pwd[0] == tmp_data[0] && wait_pwd[1] == tmp_data[1]) {
			return 0;
		}


		TPD_INFO("Now retry %d times!\n", count++);
		msleep(50);
	} while (count < 200);

	return 1;
}

static int hx83102j_check_notch(struct chip_data_hx83102j *chip_info, int index)
{
	int tx = chip_info->hw_res->tx_num;
	int rx = chip_info->hw_res->rx_num;
	if (((tx - 4) / 2 <= index / 36 && index / 36 <= (tx - 4) / 2 + 3) && (index % 36 >= rx - 1)) {
		return 1;
	} else {
		return 0;
	}
}

static int mpTestFunc(struct chip_data_hx83102j *chip_info, uint8_t checktype, uint32_t datalen)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint32_t i = 0;
	int fail_idx = -9487;
	int16_t weight = 0;
	int32_t *RAW = NULL;
#ifdef RAWDATA_NOISE
	uint32_t RAW_Rawdata[datalen];
#endif
	char *rslt_log = NULL;
	char *start_log = NULL;
	int ret = 0;
	int criteria_rawdata_min = RAWMIN;
	int criteria_rawdata_max = RAWMAX;
	int criteria_lpwug_rawdata_max = LPWUG_RAWDATA_MAX;
	int criteria_lpwug_idle_rawdata_max = LPWUG_IDLE_RAWDATA_MAX;

	if (chip_info->isbd12proj) {
		criteria_rawdata_min = RAWMIN_BD12;
		criteria_rawdata_max = RAWMAX_BD12;
		criteria_lpwug_rawdata_max = LPWUG_RAWDATA_MAX_BD12;
		criteria_lpwug_idle_rawdata_max = LPWUG_IDLE_RAWDATA_MAX_BD12;
	}

	if (hx83102j_check_mode(chip_info, checktype)) {
		TPD_INFO("Need Change Mode, target=%s", g_himax_inspection_mode[checktype]);
#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(chip_info->hw_res, false);
		usleep_range(5000, 5001);
		hx83102j_resetgpio_set(chip_info->hw_res, true);
		usleep_range(5000, 5001);
#else
		hx83102j_mcu_sys_reset(chip_info);
#endif
		hx83102j_sense_off(chip_info);
#ifndef HX_ZERO_FLASH
		hx83102j_reload_disable(chip_info, 1);
#endif

		hx83102j_switch_mode(chip_info, checktype);

		if (checktype == HIMAX_INSPECTION_NOISE) {
			hx83102j_set_N_frame(chip_info, NOISEFRAME, checktype);
			/*hx83102j_get_noise_base();*/
		} else if (checktype >= HIMAX_INSPECTION_LPWUG_RAWDATA) {
			TPD_INFO("N frame = %d\n", 1);
			hx83102j_set_N_frame(chip_info, 1, checktype);
		} else {
			hx83102j_set_N_frame(chip_info, 2, checktype);
		}


		hx83102j_sense_on(chip_info, 1);

		ret = hx83102j_wait_sorting_mode(chip_info, checktype);
		if (ret) {
			TPD_INFO("%s: hx83102j_wait_sorting_mode FAIL\n", __func__);
			return ret;
		}
	}

	hx83102j_switch_data_type(chip_info, checktype);

	RAW = kcalloc(datalen, sizeof(int32_t), GFP_KERNEL);
	if (RAW == NULL) {
			TPD_INFO("%s, Failed to allocate memory\n", __func__);
			return -1;
	}

	ret = hx83102j_get_rawdata(chip_info, RAW, datalen);
	if (ret) {
		TPD_INFO("%s: hx83102j_get_rawdata FAIL\n", __func__);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 900000A8: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x40;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 10007F40: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 10000000: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x04;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 10007F04: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x02;
		tmp_addr[1] = 0x04;
		tmp_addr[0] = 0xF4;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 800204B4: data[0]=0x%02X, data[1]=0x%02X, data[2]=0x%02X, data[3]=0x%02X, \n",
			__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		if (RAW) {
			kfree(RAW);
		}

		return ret;
	}

	/* back to normal */
	hx83102j_switch_data_type(chip_info, HIMAX_INSPECTION_BACK_NORMAL);

	rslt_log = kzalloc(256 * sizeof(char), GFP_KERNEL);
	if (!rslt_log) {
		TPD_INFO("%s:%d rslt_log kzalloc buf error\n", __func__, __LINE__);
		goto RET_OUT;
	}
	start_log = kzalloc(256 * sizeof(char), GFP_KERNEL);
	if (!start_log) {
		TPD_INFO("%s:%d	 start_log kzalloc buf error\n", __func__, __LINE__);
		goto RET_OUT;
	}
	switch (checktype) {
	case HIMAX_INSPECTION_OPEN:
		for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
			if (hx83102j_check_notch(chip_info, i)) {
				continue;
			}
			if(chip_info->isread_csv == false) {
				if (RAW[i] > OPENMAX || RAW[i] < OPENMIN) {
					fail_idx = i;
					TPD_INFO("%s: open test FAIL\n", __func__);
					ret = RESULT_ERR;
				}
			} else {
				if (RAW[i] > chip_info->hx83102j_nf_inspection_criteria[IDX_OPENMAX][i] ||
					RAW[i] < chip_info->hx83102j_nf_inspection_criteria[IDX_OPENMIN][i]) {
					fail_idx = i;
					TPD_INFO("%s: open test FAIL\n", __func__);
					ret = RESULT_ERR;
				}
			}
		}
		TPD_INFO("%s: open test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_MICRO_OPEN:
		for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
			if (hx83102j_check_notch(chip_info, i)) {
				continue;
			}
			if(chip_info->isread_csv == false) {
				if (RAW[i] > M_OPENMAX || RAW[i] < M_OPENMIN) {
					fail_idx = i;
					TPD_INFO("%s: micro open test FAIL\n", __func__);
					ret =  RESULT_ERR;
				}
			} else {
				if (RAW[i] > chip_info->hx83102j_nf_inspection_criteria[IDX_M_OPENMAX][i] ||
					RAW[i] < chip_info->hx83102j_nf_inspection_criteria[IDX_M_OPENMIN][i]) {
					fail_idx = i;
					TPD_INFO("%s: micro open test FAIL\n", __func__);
					TPD_INFO("M_OPENMAX = %d, M_OPENMIN = %d\n",
					chip_info->hx83102j_nf_inspection_criteria[IDX_M_OPENMAX][1],
					chip_info->hx83102j_nf_inspection_criteria[IDX_M_OPENMIN][1]);
					ret =  RESULT_ERR;
				}
			}
		}

		TPD_INFO("%s: micro open test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_SHORT:
		for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
			if (hx83102j_check_notch(chip_info, i)) {
				continue;
			}
			if(chip_info->isread_csv == false) {
				if (RAW[i] > SHORTMAX || RAW[i] < SHORTMIN) {
					fail_idx = i;
					TPD_INFO("%s: short test FAIL\n", __func__);
					ret = RESULT_ERR;
				}
			} else {
				if (RAW[i] > chip_info->hx83102j_nf_inspection_criteria[IDX_SHORTMAX][i] ||
					RAW[i] < chip_info->hx83102j_nf_inspection_criteria[IDX_SHORTMIN][i]) {
					fail_idx = i;
					TPD_INFO("%s: short test FAIL\n", __func__);
					ret = RESULT_ERR;
				}
			}
		}
		TPD_INFO("%s: short test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_RAWDATA:
		for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
				if (hx83102j_check_notch(chip_info, i)) {
				continue;
			}
			if(chip_info->isread_csv == false) {
				if (RAW[i] > criteria_rawdata_max || RAW[i] < criteria_rawdata_min) {
					fail_idx = i;
					TPD_INFO("%s: rawdata test FAIL\n", __func__);
					ret = RESULT_ERR;
				}
			} else {
				if (RAW[i] > chip_info->hx83102j_nf_inspection_criteria[IDX_RAWDATA_MAX][i] ||
					RAW[i] < chip_info->hx83102j_nf_inspection_criteria[IDX_RAWDATA_MIN][i]) {
					fail_idx = i;
					TPD_INFO("%s: rawdata test FAIL\n", __func__);
					ret = RESULT_ERR;
				}
			}
		}
		TPD_INFO("%s: rawdata test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_NOISE:
		/*for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
			if (hx83102j_check_notch(i)) {
				continue;
			}
			if (RAW[i] > g_noisemax) {
				TPD_INFO("%s: noise test FAIL\n", __func__);
				return RESULT_ERR;
			}
		}*/
		hx83102j_get_noise_base(chip_info);
		if (chip_info->isbd12proj || chip_info->isea006proj) {
			snprintf(start_log, 256 * sizeof(char), "NOISE TEST\n");
			for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
					if (hx83102j_check_notch(chip_info, i)) {
					continue;
				}
				if (RAW[i] > g_noisemax) {
					fail_idx = i;
					TPD_INFO("%s: noise test FAIL\n", __func__);
					TPD_INFO("%s: noise test FAIL in %d, value=%d\n", __func__, i, RAW[i]);
					ret = RESULT_ERR;
				}
			}
		} else {
			weight = hx83102j_get_noise_weight(chip_info);
			if (weight > g_noisemax) {
				TPD_INFO("%s: noise test FAIL\n", __func__);
				ret = RESULT_ERR;
				fail_idx = RESULT_ERR;
				snprintf(start_log, 256 * sizeof(char), " Test Fail Threshold = %d, weight=%d\n", g_noisemax, weight);
			} else {
				snprintf(start_log, 256 * sizeof(char), " Test Pass Threshold = %d, weight=%d\n", g_noisemax, weight);
			}
		}
#ifdef RAWDATA_NOISE
		TPD_INFO("[MP_RAW_TEST_RAW]\n");

		hx83102j_switch_data_type(chip_info, HIMAX_INSPECTION_RAWDATA);
		ret = hx83102j_get_rawdata(chip_info, RAW, datalen);
		if (ret == RESULT_ERR) {
			TPD_INFO("%s: hx83102j_get_rawdata FAIL\n", __func__);
			ret = RESULT_ERR;
		}
#endif
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
		for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
			if (hx83102j_check_notch(chip_info, i)) {
				continue;
			}
			if(chip_info->isread_csv == false) {
				if (RAW[i] > criteria_lpwug_rawdata_max || RAW[i] < LPWUG_RAWDATA_MIN) {
					fail_idx = i;
					TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_RAWDATA FAIL\n", __func__);
					ret = THP_AFE_INSPECT_ERAW;
				}
			} else {
				if (RAW[i] > chip_info->hx83102j_nf_inspection_criteria[IDX_LPWUG_RAWDATA_MAX][i] ||
					RAW[i] < chip_info->hx83102j_nf_inspection_criteria[IDX_LPWUG_RAWDATA_MIN][i]) {
					fail_idx = i;
					TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_RAWDATA FAIL\n", __func__);
					ret = THP_AFE_INSPECT_ERAW;
				}
			}
		}
		TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_NOISE:
		if(chip_info->isbd12proj || chip_info->isea006proj) {
			for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
				if (hx83102j_check_notch(chip_info, i)) {
					continue;
				}
				if(chip_info->isread_csv == false) {
					if (RAW[i] > LPWUG_NOISE_MAX || RAW[i] < LPWUG_NOISE_MIN) {
						fail_idx = i;
						TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_NOISE FAIL\n", __func__);
						ret = THP_AFE_INSPECT_ENOISE;
					}
				} else {
					if (RAW[i] > chip_info->hx83102j_nf_inspection_criteria[IDX_LPWUG_NOISE_MAX][i] ||
						RAW[i] < chip_info->hx83102j_nf_inspection_criteria[IDX_LPWUG_NOISE_MIN][i]) {
						fail_idx = i;
						TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_NOISE FAIL\n", __func__);
						ret = THP_AFE_INSPECT_ENOISE;
					}
				}
			}
		} else {
			hx83102j_get_noise_base(chip_info);
			weight = hx83102j_get_noise_weight(chip_info);
			if(chip_info->isread_csv == false) {
				if (weight > g_lpwug_noisemax || weight < LPWUG_NOISE_MIN) {
					TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_NOISE FAIL\n", __func__);
					ret = THP_AFE_INSPECT_ENOISE;
				}
			} else {
				if (weight > g_lpwug_noisemax || weight < chip_info->hx83102j_nf_inspection_criteria[IDX_LPWUG_NOISE_MIN][i]) {
					fail_idx = i;
					TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_NOISE FAIL\n", __func__);
					ret = THP_AFE_INSPECT_ENOISE;
				}
			}
		}
		TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_NOISE PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
		for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
			if (hx83102j_check_notch(chip_info, i)) {
				continue;
			}
			if(chip_info->isread_csv == false) {
				if (RAW[i] > criteria_lpwug_idle_rawdata_max || RAW[i] < LPWUG_IDLE_RAWDATA_MIN) {
					fail_idx = i;
					TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA FAIL\n", __func__);
					ret = THP_AFE_INSPECT_ERAW;
				}
			} else {
				if (RAW[i] > chip_info->hx83102j_nf_inspection_criteria[IDX_LPWUG_IDLE_RAWDATA_MAX][i] ||
					RAW[i] < chip_info->hx83102j_nf_inspection_criteria[IDX_LPWUG_IDLE_RAWDATA_MIN][i]) {
					fail_idx = i;
					TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA FAIL\n", __func__);
					ret = THP_AFE_INSPECT_ERAW;
				}
			}
		}
		TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
			if (hx83102j_check_notch(chip_info, i)) {
				continue;
			}
			if(chip_info->isread_csv == false) {
				if (RAW[i] > LPWUG_IDLE_NOISE_MAX || RAW[i] < LPWUG_IDLE_NOISE_MIN) {
					fail_idx = i;
					TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_IDLE_NOISE FAIL\n", __func__);
					ret = THP_AFE_INSPECT_ENOISE;
				}
			} else {
				if (RAW[i] > chip_info->hx83102j_nf_inspection_criteria[IDX_LPWUG_IDLE_NOISE_MAX][i] ||
					RAW[i] < chip_info->hx83102j_nf_inspection_criteria[IDX_LPWUG_IDLE_NOISE_MIN][i]) {
					fail_idx = i;
					TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_IDLE_NOISE FAIL\n", __func__);
					ret = THP_AFE_INSPECT_ENOISE;
				}
			}
		}
		TPD_INFO("%s: HIMAX_INSPECTION_LPWUG_IDLE_NOISE PASS\n", __func__);
		break;

	default:
		TPD_INFO("Wrong type=%d\n", checktype);
		break;
	}
	if (fail_idx == -9487) {
		snprintf(rslt_log, 256 * sizeof(char), "%s%s\n",
			 g_himax_inspection_mode[checktype], " Test Pass!");
	} else {
		snprintf(rslt_log, 256 * sizeof(char), "%s, Test Fail in RAW[%d]=%d\n",
			 g_himax_inspection_mode[checktype],
			 fail_idx, RAW[fail_idx]);
	}

	hx_test_data_get(chip_info, RAW, start_log, rslt_log, checktype);

	if (RAW) {
		kfree(RAW);
	}
	if (rslt_log) {
		kfree(rslt_log);
	}
	if (start_log) {
		kfree(start_log);
	}
	if (ret) {
		return ret;
	} else {
		return RESULT_OK;
	}

RET_OUT:
	if (RAW) {
		kfree(RAW);
	}
	if (rslt_log) {
		kfree(rslt_log);
	}
	return RESULT_ERR;
}
#ifdef HX_CSV_CRITERIA
static int hx83102j_saperate_comma(const struct firmware *file_entry,
								char **result, int str_size)
{
	int count = 0;
	int str_count = 0; /* now string*/
	int ch_count = 0; /* now char count in string*/

	do {
		switch (file_entry->data[count]) {
		case ASCII_COMMA:
		case ACSII_SPACE:
		case ASCII_CR:
		case ASCII_LF:
			count++;
			/* If end of line as above condifiton,
			* differencing the count of char.
			* If ch_count != 0
			* it's meaning this string is parsing over .
			* The Next char is belong to next string
			*/
			if (ch_count != 0) {
				ch_count = 0;
				str_count++;
			}
			break;
		default:
			result[str_count][ch_count++] =
				file_entry->data[count];
			count++;
			break;
		}
	} while (count < file_entry->size && str_count < str_size);

	return str_count;
}

static int hx_diff_str(char *str1, char *str2)
{
	int i = 0;
	int result = 0; /* zero is all same, non-zero is not same index*/
	int str1_len = strlen(str1);
	int str2_len = strlen(str2);

	if (str1_len != str2_len) {
		TPD_DEBUG("%s:Size different!\n", __func__);
		return LENGTH_FAIL;
	}

	for (i = 0; i < str1_len; i++) {
		if (str1[i] != str2[i]) {
			result = i + 1;
			TPD_INFO("%s: different in %d!\n", __func__, result);
			return result;
		}
	}

	return result;
}

/* get idx of criteria whe parsing file */
static int hx_find_crtra_id(char *input)
{
	int i = 0;
	int result = 0;

	for (i = 0 ; i < chip_info->hx_criteria_size ; i++) {
		if (hx_diff_str(g_hx_inspt_crtra_name[i], input) == 0) {
			result = i;
			TPD_INFO("find the str=%s, idx=%d\n",
					 g_hx_inspt_crtra_name[i], i);
			break;
		}
	}
	if (i > (chip_info->hx_criteria_size - 1)) {
		TPD_INFO("%s: find Fail!\n", __func__);
		return LENGTH_FAIL;
	}

	return result;
}

/* claculate 10's power function */
static int hx83102j_power_cal(int pow, int number)
{
	int i = 0;
	int result = 1;

	for (i = 0; i < pow; i++)
		result *= 10;
	result = result * number;

	return result;
}

/* String to int */
static int hiamx_parse_str2int(char *str)
{
	int i = 0;
	int temp_cal = 0;
	int result = 0;
	int str_len = strlen(str);
	int negtive_flag = 0;

	for (i = 0; i < strlen(str); i++) {
		if (str[i] != '-' && str[i] > '9' && str[i] < '0') {
			TPD_INFO("%s: Parsing fail!\n", __func__);
			result = -9487;
			negtive_flag = 0;
			break;
		}
		if (str[i] == '-') {
			negtive_flag = 1;
			continue;
		}
		temp_cal = str[i] - '0';
		result += hx83102j_power_cal(str_len - i - 1, temp_cal);
		/* str's the lowest char is the number's the highest number
		 * So we should reverse this number before using the power
		 * function
		 * -1: starting number is from 0 ex:10^0 = 1, 10^1 = 10
		 */
	}

	if (negtive_flag == 1)
		result = 0 - result;

	return result;
}

static int hx_print_crtra_after_parsing(struct chip_data_hx83102j *chip_info)
{
	int i = 0, j = 0;
	int all_mut_len = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;

	for (i = 0; i < chip_info->hx_criteria_size; i++) {
		TPD_DETAIL("Now is %s\n", g_hx_inspt_crtra_name[i]);
		if (chip_info->hx83102j_nf_inspt_crtra_flag[i] == 1) {
			for (j = 0; j < all_mut_len; j++) {
				TPD_DEBUG("%d, ", chip_info->hx83102j_nf_inspection_criteria[i][j]);
				if (j % 16 == 15)
					TPD_DEBUG("\n");
			}
		} else {
			TPD_DEBUG("No this Item in this criteria file!\n");
		}
		TPD_DEBUG("\n");
	}

	return 0;
}

static int hx_get_crtra_by_name(struct chip_data_hx83102j *chip_info, char **result, int size_of_result_str)
{
	int i = 0;
	/* count of criteria type */
	int count_type = 0;
	/* count of criteria data */
	int count_data = 0;
	int err = THP_AFE_INSPECT_OK;
	int all_mut_len = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;
	int temp = 0;

	/* get criteria and assign to a global array(2-Dimensional/int) */
	/* basiclly the size of criteria will be
	 * (crtra_count * (all_mut_len) + crtra_count)
	 * but we use file size to be the end of counter
	 */
	for (i = 0; i < size_of_result_str && result[i] != NULL; i++) {
		/* It have get one page(all mutual) criteria data!
		 * And we should skip the string of criteria name!
		 */
		if (i == 0 || i == ((i / (all_mut_len)) + (i / (all_mut_len) * (all_mut_len)))) {
			count_data = 0;

			TPD_DEBUG("Now find str=%s, idx=%d\n", result[i], i);

			/* check the item of criteria is in criteria file
			* or not
			*/
			count_type = hx_find_crtra_id(result[i]);
			if (count_type < 0) {
				TPD_INFO("1. %s:Name Not match!\n", __func__);
				/* E("can recognize[%d]=%s\n", count_type,
				 * g_hx_inspt_crtra_name[count_type]);
				 */
				TPD_INFO("get from file[%d]=%s\n", i, result[i]);
				TPD_INFO("Please check criteria file again!\n");
				err = THP_AFE_INSPECT_EFILE;
				goto END_FUNCTION;
			} else {
				TPD_INFO("Now str=%s, idx=%d\n",
						 g_hx_inspt_crtra_name[count_type], count_type);
				chip_info->hx83102j_nf_inspt_crtra_flag[count_type] = 1;
			}
			continue;
		}
		/* change string to int*/
		temp = hiamx_parse_str2int(result[i]);
		if (temp != -9487) {
			chip_info->hx83102j_nf_inspection_criteria[count_type][count_data] = temp;
		} else {
			TPD_INFO("%s: Parsing Fail in %d\n", __func__, i);
			TPD_INFO("in range:[%d]=%s\n", count_type,
					 g_hx_inspt_crtra_name[count_type]);
			TPD_INFO("btw, get from file[%d]=%s\n", i, result[i]);
			break;
		}
		/* dbg
		 * I("[%d]chip_info->hx83102j_nf_inspection_criteria[%d][%d]=%d\n",
		 * i, count_type, count_data,
		 * chip_info->hx83102j_nf_inspection_criteria[count_type][count_data]);
		 */
		count_data++;
	}

	/* dbg:print all of criteria from parsing file */
	hx_print_crtra_after_parsing(chip_info);

	TPD_INFO("Total loop=%d\n", i);
END_FUNCTION:
	return err;
}

static int hx83102j_parse_criteria_file(struct chip_data_hx83102j *chip_info)
{
	int err = THP_AFE_INSPECT_OK;
	const struct firmware *file_entry = NULL;
	static char **result;
	int i = 0;
	int j = 0;
	int crtra_count = chip_info->hx_criteria_size;
	int data_size = 0; /* The maximum of number Data*/
	int all_mut_len = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;
	int str_max_len = 0;
	int result_all_len = 0;
	int file_size = 0;
	int size_of_result_str = 0;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	TPD_INFO("%s, Entering\n", __func__);
	TPD_INFO("file name = %s\n", chip_info->test_limit_name);


	if (ts->com_test_data.limit_fw == NULL) {
		TPD_INFO("%s, There is no criteria file\n", __func__);
		err = -1;
	} else {
		TPD_INFO("%s: assign limit file from global!\n", __func__);
	file_entry = ts->com_test_data.limit_fw;
	}
	if (err < 0) {
		TPD_INFO("%s, fail in line%d error code=%d\n", __func__, __LINE__, err);
		err = THP_AFE_INSPECT_EFILE;
		chip_info->isread_csv = false;
		goto END_FUNC_REQ_FAIL;
	}
	/* size of criteria include name string */
	data_size = ((all_mut_len) * crtra_count) + crtra_count;

	/* init the array which store original criteria and include
	 *  name string
	 */
	while (g_hx_inspt_crtra_name[j] != NULL) {
		if (strlen(g_hx_inspt_crtra_name[j]) > str_max_len)
			str_max_len = strlen(g_hx_inspt_crtra_name[j]);
		j++;
	}

	if (result == NULL) {
		TPD_INFO("%s: result is NULL, alloc memory.\n", __func__);
		result = kcalloc(data_size, sizeof(char *), GFP_KERNEL);
		if (result != NULL) {
			for (i = 0 ; i < data_size; i++) {
				result[i] = kcalloc(str_max_len, sizeof(char), GFP_KERNEL);
				if (result[i] == NULL) {
					TPD_INFO("%s: rst_arr Memory allocation falied!\n", __func__);
					goto rst_arr_mem_alloc_failed;
				}
			}
		} else {
			TPD_INFO("%s: Memory allocation falied!\n", __func__);
			goto rst_mem_alloc_failed;
		}
	} else {
		for (i = 0 ; i < data_size; i++)
			memset(result[i], 0x00, str_max_len * sizeof(char));
	}

	result_all_len = data_size;
	file_size = file_entry->size;
	TPD_INFO("Now result_all_len=%d\n", result_all_len);
	TPD_INFO("Now file_size=%d\n", file_size);

	/* dbg */
	TPD_DEBUG("first 4 bytes 0x%2X, 0x%2X, 0x%2X, 0x%2X !\n",
			  file_entry->data[0], file_entry->data[1],
			  file_entry->data[2], file_entry->data[3]);

	/* parse value in to result array(1-Dimensional/String) */
	size_of_result_str =
		hx83102j_saperate_comma(file_entry, result, data_size);

	TPD_INFO("%s: now size_of_result_str=%d\n", __func__, size_of_result_str);

	err = hx_get_crtra_by_name(chip_info, result, size_of_result_str);
	if (err != THP_AFE_INSPECT_OK) {
		TPD_INFO("%s:Load criteria from file fail, go end!\n", __func__);
	}

	goto END_FUNC;

rst_arr_mem_alloc_failed:
	for (i = 0 ; i < data_size; i++) {
		if (result[i] != NULL) {
			kfree(result[i]);
		}
	}
	if (result)
		kfree(result);
rst_mem_alloc_failed:
END_FUNC:
END_FUNC_REQ_FAIL:
	TPD_INFO("%s, END\n", __func__);
	return err;
	/* parsing Criteria end */
}
#endif
#ifdef HX_OPV2_CRITERIA
static int hx83102j_parse_criteria_file(struct chip_data_hx83102j *chip_info)
{
	int err = THP_AFE_INSPECT_OK;
	/*const struct firmware *file_entry = NULL;*/
	/*char *file_name = "hx_criteria.csv";*/
	/*static char **result;*/
	/*int crtra_count = chip_info->hx_criteria_size;*/
	/*int data_size = 0;*/ /* The maximum of number Data*/
	int all_mut_len = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;
	int all_header_size = 13 * 4;
	int item_header_size = 7 * 4;

	/* ih = test item header */
	/*int offset_ih_name = 0x00;*/ /* 0 ~ 7 bytes*/
	int offset_ih_type = 0x08; /* 8 ~ 9 bytes*/
	/*int offset_ih_data = 0x0A;*/ /* 10 ~ 11 bytes*/
	/* 12 ~ 15 bytes unkown*/
	/*int offset_ih_end_idx_top = 0x10;*/ /* 16 ~ 19 bytes*/
	/*int offset_ih_has_top = 0x14;*/ /* 20 ~ 23 bytes*/
	/*int offset_ih_has_floor = 0x18;*/ /* 24 ~ 27 bytes*/

	uint32_t p = 0; /** location **/
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t file_size;
	struct auto_test_header *ph = NULL;
	uint32_t *p_item_offset = NULL;

	uint8_t *limit_entry;
	char header_str[128] = {0};
	int type = 0;
	int criteria_type = 0;
	int type_list_check = 0;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	TPD_INFO("%s, Entering\n", __func__);
	TPD_INFO("file name = %s\n", chip_info->test_limit_name);

	/* default path is /system/etc/firmware */
	/* err = request_firmware(&file_entry, chip_info->test_limit_name, ts->dev);*/

	if (ts->com_test_data.limit_fw == NULL) {
		TPD_INFO("%s, There is no criteria file\n", __func__);
		err = -1;
		goto END;
	}

	ph = (struct auto_test_header *)(ts->com_test_data.limit_fw->data);
	limit_entry = (uint8_t *)ts->com_test_data.limit_fw->data;
	file_size = ts->com_test_data.limit_fw->size;


	p_item_offset = (uint32_t *)(ts->com_test_data.limit_fw->data + 16);
	for (i = 0; i < all_header_size; i++) {
		if (i < 8)
			header_str[i] = limit_entry[i];
		p++;
	}
	header_str[8] = '\0';
	TPD_INFO("%s: header=%s, p=%d\n", __func__, header_str, p);

	for (i = 0; i < chip_info->hx_criteria_size; i+=2) {
		if (p + 1 >= file_size) {
			TPD_INFO("%s: Over size, skip\n", __func__);
			break;
		}
		type = (limit_entry[p + offset_ih_type] - 1);
		criteria_type = type * 2;
		TPD_INFO("%s [%i]Now type %d = %s\n", __func__, i, type, g_op_hx_inspection_mode[type]);
		TPD_INFO("%s Now criteria_type %d = %s\n", __func__, type, g_hx_inspt_crtra_name[criteria_type]);
		p += item_header_size;
		TPD_INFO("%s now start's p = %d\n", __func__, p);
		if ((type + 1) >= OPLUS_TEST_IDX_OPEN
			&& (type + 1) <= OPLUS_TEST_IDX_LPWUG_IDLE_NOISE) {
			TPD_INFO("%s Now type=%s, p=%d start criteria\n", __func__, g_hx_inspt_crtra_name[criteria_type], p);
			for(j = 0; j < all_mut_len; j++) {
					chip_info->hx83102j_nf_inspection_criteria[criteria_type][j]
						= limit_entry[p + 3] << 24 |
							limit_entry[p + 2] << 16 |
							limit_entry[p + 1] << 8 |
							limit_entry[p];
					p+=4;
			}
			type_list_check++;
			TPD_INFO("%s, parse type=%s, p=%d, OK\n", __func__, g_hx_inspt_crtra_name[criteria_type], p);
			TPD_INFO("Now %s [0]=%d, [1]=%d,[2]=%d,[3]=%d\n", g_hx_inspt_crtra_name[criteria_type],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][0],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][1],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][2],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][3]);
			TPD_INFO("Now %s [last3]=%d, [last2]=%d,[last1]=%d,[last0]=%d\n", g_hx_inspt_crtra_name[criteria_type],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][all_mut_len - 4],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][all_mut_len - 3],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][all_mut_len - 2],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][all_mut_len - 1]);

			criteria_type++;
			TPD_INFO("%s Now type=%s, p=%d start criteria\n", __func__, g_hx_inspt_crtra_name[criteria_type], p);
			for(j = 0; j < all_mut_len; j++) {
					chip_info->hx83102j_nf_inspection_criteria[criteria_type][j]
						= limit_entry[p + 3] << 24 |
							limit_entry[p + 2] << 16 |
							limit_entry[p + 1] << 8 |
							limit_entry[p];
					p+=4;
			}
			type_list_check++;
			TPD_INFO("%s, parse type=%s, p=%d, OK\n", __func__, g_hx_inspt_crtra_name[criteria_type], p);
			TPD_INFO("Now %s [0]=%d, [1]=%d,[2]=%d,[3]=%d\n", g_hx_inspt_crtra_name[criteria_type],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][0],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][1],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][2],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][3]);
			TPD_INFO("Now %s [last3]=%d, [last2]=%d,[last1]=%d,[last0]=%d\n", g_hx_inspt_crtra_name[criteria_type],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][all_mut_len - 4],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][all_mut_len - 3],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][all_mut_len - 2],
				chip_info->hx83102j_nf_inspection_criteria[criteria_type][all_mut_len - 1]);
		}
	}

	if (type_list_check != chip_info->hx_criteria_size) {
		err = -1;
		TPD_INFO("[Error] Now check=%d is no equal all item=%d", type_list_check, chip_info->hx_criteria_item);
	}
END:
	TPD_INFO("%s, END\n", __func__);
	return err;
	/* parsing Criteria end */
}

#endif
/* Get sub-string from original string by using some charaters
 * return size of result
 */

static int hx_get_size_str_arr(char **input)
{
	int i = 0;
	int result = 0;

	while (input[i] != NULL)
		i++;

	result = i;
	TPD_DEBUG("There is %d in [0]=%s\n", result, input[0]);

	return result;
}


static int hx83102j_self_test_data_init(struct chip_data_hx83102j *chip_info)
{
	int ret = THP_AFE_INSPECT_OK;
	int i = 0;

	/* get test item and its items of criteria*/
	chip_info->hx_criteria_item = hx_get_size_str_arr(g_himax_inspection_mode);
	chip_info->hx_criteria_size = hx_get_size_str_arr(g_hx_inspt_crtra_name);
	TPD_INFO("There is %d chip_info->hx_criteria_item and %d chip_info->hx_criteria_size\n",
			 chip_info->hx_criteria_item, chip_info->hx_criteria_size);

	/* init criteria data*/
	if (chip_info->hx83102j_nf_inspt_crtra_flag == NULL)
		chip_info->hx83102j_nf_inspt_crtra_flag = kzalloc(chip_info->hx_criteria_size * sizeof(int), GFP_KERNEL);
	if (chip_info->hx83102j_nf_inspection_criteria == NULL)
		chip_info->hx83102j_nf_inspection_criteria = kzalloc(sizeof(int *)*chip_info->hx_criteria_size, GFP_KERNEL);
	if (chip_info->hx83102j_nf_inspt_crtra_flag == NULL || chip_info->hx83102j_nf_inspection_criteria == NULL) {
		TPD_INFO("%s: %d, Memory allocation falied!\n", __func__, __LINE__);
		return MEM_ALLOC_FAIL;
	}

	for (i = 0; i < chip_info->hx_criteria_size; i++) {
		if (chip_info->hx83102j_nf_inspection_criteria[i] == NULL)
			chip_info->hx83102j_nf_inspection_criteria[i] = kcalloc(
				(chip_info->hw_res->tx_num * chip_info->hw_res->rx_num), sizeof(int), GFP_KERNEL);
		if (chip_info->hx83102j_nf_inspection_criteria[i] == NULL) {
			TPD_INFO("%s: %d, Memory allocation falied!\n", __func__, __LINE__);
			return MEM_ALLOC_FAIL;
		}
	}

	/* parsing criteria from file*/
	ret = hx83102j_parse_criteria_file(chip_info);

	/* print get criteria string */
	for (i = 0 ; i < chip_info->hx_criteria_size ; i++) {
		if (chip_info->hx83102j_nf_inspt_crtra_flag[i] != 0)
			TPD_DEBUG("%s: [%d]There is String=%s\n", __func__, i, g_hx_inspt_crtra_name[i]);
	}

	return ret;
}

static int hx83102j_black_screen_test(void *chip_data, char *message, int msg_size, struct auto_testdata *hx_testdata)
{
	int error = 0;
	int error_num = 0;
	int retry_cnt = 3;
	char *buf = NULL;
	char *g_file_name_ok = NULL;
	char *g_file_name_ng = NULL;
	char *g_test_list_log = NULL;
	char *g_project_test_info_log = NULL;
	char *g_company_info_log = NULL;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	int i = 0;

	TPD_INFO("%s\n", __func__);


	chip_info->in_self_test = 1;


	buf = kzalloc(sizeof(char) * 128, GFP_KERNEL);
	if (!buf) {
		TPD_INFO("%s:%d buf kzalloc error\n", __func__, __LINE__);
		goto RET_OUT;
	}
	g_file_name_ok = kzalloc(sizeof(char) * 64, GFP_KERNEL);
	if (!g_file_name_ok) {
		TPD_INFO("%s:%d g_file_name_ok kzalloc error\n", __func__, __LINE__);
		goto RET_OUT;
	}
	g_file_name_ng = kzalloc(sizeof(char) * 64, GFP_KERNEL);
	if (!g_file_name_ng) {
		TPD_INFO("%s:%d g_file_name_ng kzalloc error\n", __func__, __LINE__);
		goto RET_OUT;
	}
	chip_info->g_rslt_data_len = 0;

	/*init criteria data*/
	error = hx83102j_self_test_data_init(chip_info);
	if (error != THP_AFE_INSPECT_OK) {
			TPD_INFO("%s:%d criteria error\n", __func__, __LINE__);
			goto RET_OUT;
	}
	/*init criteria data*/

	/*Init Log Data */
	chip_info->g_1kind_raw_size = 5 * chip_info->hw_res->tx_num * chip_info->hw_res->rx_num * 2;
	g_company_info_log = kcalloc(256, sizeof(char), GFP_KERNEL);
	g_test_list_log = kcalloc(256, sizeof(char), GFP_KERNEL);
	g_project_test_info_log = kcalloc(256, sizeof(char), GFP_KERNEL);
	if ((g_company_info_log == NULL) || (g_test_list_log == NULL) || (g_project_test_info_log == NULL)) {
		TPD_INFO("%s:%d g_company_info_log g_test_list_log g_project_test_info_log kzalloc buf error\n", __func__, __LINE__);
		goto RET_OUT;
	}
	chip_info->hx83102j_nf_fail_write_count = 0;
	chip_info->g_file_path_ok = kcalloc(256, sizeof(char), GFP_KERNEL);
	if (!chip_info->g_file_path_ok) {
		TPD_INFO("%s:%d chip_info->g_file_path_ok kzalloc buf error\n", __func__, __LINE__);
		goto RET_OUT;
	}
	chip_info->g_file_path_ng = kcalloc(256, sizeof(char), GFP_KERNEL);
	if (!chip_info->g_file_path_ng) {
		TPD_INFO("%s:%d chip_info->g_file_path_ng kzalloc buf error\n", __func__, __LINE__);
		goto RET_OUT;
	}

	if (chip_info->g_rslt_data == NULL) {
		TPD_INFO("chip_info->g_rslt_data is NULL");
		chip_info->g_rslt_data = kcalloc(chip_info->g_1kind_raw_size * chip_info->hx_criteria_item,
							  sizeof(char), GFP_KERNEL);
		if (!chip_info->g_rslt_data) {
			TPD_INFO("%s:%d chip_info->g_rslt_data kzalloc buf error\n", __func__, __LINE__);
			goto RET_OUT;
		}
	} else {
		memset(chip_info->g_rslt_data, 0x00, chip_info->g_1kind_raw_size * chip_info->hx_criteria_item *
			   sizeof(char));
	}


	/* return to safe mode*/
	hx83102j_enable_interrupt(chip_info, false);

	/*Init Log Data */

	/*6. LPWUG RAWDATA*/
	retry_cnt = 3;
	TPD_INFO("[MP_LPWUG_TEST_NOISE]\n");
	do {
		error = mpTestFunc(chip_info, HIMAX_INSPECTION_LPWUG_NOISE, (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num) +
				chip_info->hw_res->tx_num + chip_info->hw_res->rx_num);
		retry_cnt--;
	} while ((error == RESULT_RETRY) && (retry_cnt > 0));
	snprintf(buf, 128, "7. MP_LPWUG_TEST_NOISE: %s\n", error ? "Error" : "Ok");
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 32,
				"7. MP_LPWUG_TEST_NOISE: %s\n", error ? "NG" : "Ok");
	TPD_INFO("%s", buf);
	if (error != 0)
		error_num++;

	/*7. LPWUG NOISE*/
	retry_cnt = 3;
	TPD_INFO("[MP_LPWUG_TEST_RAW]\n");
	do {
		error = mpTestFunc(chip_info, HIMAX_INSPECTION_LPWUG_RAWDATA, (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num) +
							chip_info->hw_res->tx_num + chip_info->hw_res->rx_num);
		retry_cnt--;
	} while ((error == RESULT_RETRY) && (retry_cnt > 0));
	snprintf(buf, 128, "6. MP_LPWUG_TEST_RAW: %s\n", error ? "Error" : "Ok");
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 15, "test Item:\n");
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 32,
				"6. MP_LPWUG_TEST_RAW: %s\n", error ? "NG" : "Ok");
	TPD_INFO("%s", buf);
	if (error != 0)
		error_num++;

	/*8. LPWUG IDLE RAWDATA*/
	retry_cnt = 3;
	TPD_INFO("[MP_LPWUG_IDLE_TEST_NOISE]\n");
	do {
		error = mpTestFunc(chip_info, HIMAX_INSPECTION_LPWUG_IDLE_NOISE, (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num) +
							chip_info->hw_res->tx_num + chip_info->hw_res->rx_num);
		retry_cnt--;
	} while ((error == RESULT_RETRY) && (retry_cnt > 0));
	snprintf(buf, 128, "9. MP_LPWUG_IDLE_TEST_NOISE: %s\n", error ? "Error" : "Ok");
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 32,
								"9. HIMAX_INSPECTION_LPWUG_IDLE_NOISE: %s\n", error ? "NG" : "Ok");
	TPD_INFO("%s", buf);
	if (error != 0)
		error_num++;

	retry_cnt = 3;
	TPD_INFO("[MP_LPWUG_IDLE_TEST_RAW]\n");
	do {
		error = mpTestFunc(chip_info, HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA, (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num) +
							chip_info->hw_res->tx_num + chip_info->hw_res->rx_num);
		retry_cnt--;
	} while ((error == RESULT_RETRY) && (retry_cnt > 0));
	snprintf(buf, 128, "8. MP_LPWUG_IDLE_TEST_RAW: %s\n", error ? "Error" : "Ok");
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 32,
				"8. MP_LPWUG_IDLE_TEST_RAW: %s\n", error ? "NG" : "Ok");
	TPD_INFO("%s", buf);
	if (error != 0)
		error_num++;

	/*Save Log Data */
	/*
	sprintf(g_file_name_ok, "tp_testlimit_gesture_OK_%02d%02d%02d-%02d%02d%02d-utc.csv",
			(rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
			rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
	sprintf(g_file_name_ng, "tp_testlimit_gesture_NG_%02d%02d%02d-%02d%02d%02d-utc.csv",
			(rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
			rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
	*/
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 22,
				"Final_result: %s\n", error_num ? "Fail" : "Pass");
	if (error) {
		snprintf(chip_info->g_file_path_ng,
				(int)(strlen(HX_GES_RSLT_OUT_PATH_NG) + strlen(g_file_name_ng) + 1),
				"%s%s", HX_GES_RSLT_OUT_PATH_NG, g_file_name_ng);
		hx83102j_test_data_pop_out(chip_info, HX_BS_TEST, g_test_list_log, g_company_info_log, g_project_test_info_log,
			chip_info->g_rslt_data, chip_info->g_file_path_ng);
	} else {
		snprintf(chip_info->g_file_path_ok,
				(int)(strlen(HX_GES_RSLT_OUT_PATH_OK) + strlen(g_file_name_ok) + 1), "%s%s", HX_GES_RSLT_OUT_PATH_OK, g_file_name_ok);
		hx83102j_test_data_pop_out(chip_info, HX_BS_TEST, g_test_list_log, g_company_info_log, g_project_test_info_log,
				chip_info->g_rslt_data, chip_info->g_file_path_ok);
	}
	/*Save Log Data */
	/*tp_test_write(hx_testdata->fp, hx_testdata->length, chip_info->g_rslt_data, (size_t)strlen(chip_info->g_rslt_data), hx_testdata->pos);*/

	snprintf(buf, 128, "%d errors. %s", error_num, error_num ? "" : "All test passed.");
	TPD_INFO("%d errors. %s\n", error_num, error_num ? "" : "All test passed.");

#ifndef HX_ZERO_FLASH
	hx83102j_sense_off(chip_info);

	hx83102j_reload_disable(chip_info, 0);

	hx83102j_sense_on(chip_info, 0);
#endif

	chip_info->in_self_test = 0;

RET_OUT:
	chip_info->in_self_test = 0;
	if (chip_info->hx83102j_nf_inspection_criteria != NULL) {
		for (i = 0; i < chip_info->hx_criteria_size; i++) {
			if (chip_info->hx83102j_nf_inspection_criteria[i] != NULL) {
				kfree(chip_info->hx83102j_nf_inspection_criteria[i]);
				chip_info->hx83102j_nf_inspection_criteria[i] = NULL;
			}
		}
		if (chip_info->hx83102j_nf_inspection_criteria)
			kfree(chip_info->hx83102j_nf_inspection_criteria);
		chip_info->hx83102j_nf_inspection_criteria = NULL;
		TPD_INFO("Now it have free the chip_info->hx83102j_nf_inspection_criteria!\n");
	} else {
		TPD_INFO("No Need to free chip_info->hx83102j_nf_inspection_criteria!\n");
	}

	if (chip_info->hx83102j_nf_inspt_crtra_flag) {
		kfree(chip_info->hx83102j_nf_inspt_crtra_flag);
		chip_info->hx83102j_nf_inspt_crtra_flag = NULL;
	}
	/*
	if (chip_info->g_rslt_data) {
		kfree(chip_info->g_rslt_data);
		chip_info->g_rslt_data = NULL;
	}
	*/
	if (chip_info->g_file_path_ok) {
		kfree(chip_info->g_file_path_ok);
		chip_info->g_file_path_ok = NULL;
	}
	if (chip_info->g_file_path_ng) {
		kfree(chip_info->g_file_path_ng);
		chip_info->g_file_path_ng = NULL;
	}
	if (g_test_list_log) {
		kfree(g_test_list_log);
		g_test_list_log = NULL;
	}
	if (g_project_test_info_log) {
		kfree(g_project_test_info_log);
		g_project_test_info_log = NULL;
	}
	if (g_company_info_log) {
		kfree(g_company_info_log);
		g_company_info_log = NULL;
	}
	if (buf) {
		kfree(buf);
		buf = NULL;
	}
	if (g_file_name_ok) {
		kfree(g_file_name_ok);
		g_file_name_ok = NULL;
	}
	if (g_file_name_ng) {
		kfree(g_file_name_ng);
		g_file_name_ng = NULL;
	}
	return error_num;
}
static void hx83102j_test_finish(void *chip_data)
{
#ifndef HX_ZERO_FLASH
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	hx83102j_sense_off(chip_info);
	hx83102j_reload_disable(chip_info, 0);
	hx83102j_sense_on(chip_info, 0);
#endif
}
static int hx83102j_chip_self_test(struct seq_file *s, struct chip_data_hx83102j *chip_info, char *g_test_list_log)
{
	int error = 0;
	int error_num = 0;
	char *buf;
	/*uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t back_data[4];*/
	uint8_t retry_cnt = 3;

	TPD_INFO("%s:Entering\n", __func__);

	buf = kzalloc(sizeof(char) * 128, GFP_KERNEL);
	if (!buf) {
		TPD_INFO("%s:%d buf kzalloc error\n", __func__, __LINE__);
		error_num = -ENOMEM;
		goto RET_OUT;
	}

	TPD_INFO("[MP_OPEN_TEST_RAW]\n");
	do {
		error = mpTestFunc(chip_info, HIMAX_INSPECTION_OPEN, (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num) +
							chip_info->hw_res->tx_num + chip_info->hw_res->rx_num);
		retry_cnt--;
	} while ((error == RESULT_RETRY) && (retry_cnt > 0));
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 15, "test Item:\n");
	snprintf(buf, 128, "1. Open Test: %s\n", error ? "Error" : "Ok");
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 32, "1. Open Test: %s\n", error ? "NG" : "Ok");
	TPD_INFO("%s", buf);
	if (error != 0)
		error_num++;


	retry_cnt = 3;
	TPD_INFO("[MP_MICRO_OPEN_TEST_RAW]\n");
	do {
		error = mpTestFunc(chip_info, HIMAX_INSPECTION_MICRO_OPEN, (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num) +
							chip_info->hw_res->tx_num + chip_info->hw_res->rx_num);
		retry_cnt--;
	} while ((error == RESULT_RETRY) && (retry_cnt > 0));
	snprintf(buf, 128, "2. Micro Open Test: %s\n", error ? "Error" : "Ok");
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 32,
				"2. Micro Open Test: %s\n", error ? "NG" : "Ok");
	TPD_INFO("%s", buf);
	if (error != 0)
		error_num++;

	retry_cnt = 3;
	TPD_INFO("[MP_SHORT_TEST_RAW]\n");
	do {
		error = mpTestFunc(chip_info, HIMAX_INSPECTION_SHORT, (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num) +
							chip_info->hw_res->tx_num + chip_info->hw_res->rx_num);
		retry_cnt--;
	} while ((error == RESULT_RETRY) && (retry_cnt > 0));
	snprintf(buf, 128, "3. Short Test: %s\n", error ? "Error" : "Ok");
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 32, "3. Short Test: %s\n", error ? "NG" : "Ok");
	TPD_INFO("%s", buf);
	if (error != 0)
		error_num++;


	retry_cnt = 3;
	TPD_INFO("[MP_NOISE_TEST_RAW]\n");
	do {
		error = mpTestFunc(chip_info, HIMAX_INSPECTION_NOISE, (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num) +
							chip_info->hw_res->tx_num + chip_info->hw_res->rx_num);
		retry_cnt--;
	} while ((error == RESULT_RETRY) && (retry_cnt > 0));
	snprintf(buf, 128, "5. Noise Test: %s\n", error ? "Error" : "Ok");
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count, 32, "5. Noise Test: %s\n", error ? "NG" : "Ok");
	TPD_INFO("%s", buf);
	if (error != 0)
		error_num++;


#ifndef RAWDATA_NOISE
	/*5. RawData Test*/
	retry_cnt = 3;
	TPD_INFO("[MP_RAW_TEST_RAW]\n");
	do {
		error = mpTestFunc(chip_info, HIMAX_INSPECTION_RAWDATA, (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num) +
							chip_info->hw_res->tx_num + chip_info->hw_res->rx_num);
		retry_cnt--;
	} while ((error == RESULT_RETRY) && (retry_cnt > 0));
	snprintf(buf, 128, "4. Raw data Test: %s\n", error ? "Error" : "Ok");
	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count,
				32, "4. Raw data Test: %s\n", error ? "NG" : "Ok");
	TPD_INFO("%s", buf);
	if (error != 0)
		error_num++;
#endif

	chip_info->in_self_test = 0;

	chip_info->hx83102j_nf_fail_write_count += snprintf(g_test_list_log + chip_info->hx83102j_nf_fail_write_count,
				22, "Final_result: %s\n", error_num ? "Fail" : "Pass");
	TPD_INFO("%s:End", __func__);
RET_OUT:
	if(buf)
		kfree(buf);
	return error_num;
}

static size_t hx83102j_proc_register_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
	size_t ret = 0;
	uint16_t loop_i;
	int i = 0;
	int times = 0;
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	int max_bus_size = MAX_RECVS_SZ;
#else
	int max_bus_size = 128;
#endif
	uint8_t *data;
	uint32_t now_address;
	char *temp_buf;
	uint8_t now_reg_cmd[4];
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)
			ts->chip_data;


	data = kzalloc(sizeof(uint8_t) * 128, GFP_KERNEL);
	if(!data) {
		TPD_INFO("%s: [data]Can't allocate enough data\n", __func__);
		ret = -ENOMEM;
		goto RET_OUT;
	}

	if (!chip_info->hx_proc_send_flag) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(!temp_buf) {
			TPD_INFO("%s: [tmp_buf]Can't allocate enough data\n", __func__);
			ret = -ENOMEM;
			if (data) {
				kfree(data);
			}
			goto RET_OUT;
		}

		TPD_INFO("hx83102j_register_show: %02X, %02X, %02X, %02X\n", chip_info->register_command[3],
			chip_info->register_command[2], chip_info->register_command[1], chip_info->register_command[0]);

		/*times = (128 % max_bus_size) > 0 ? 128 / max_bus_size + 1 : 128 / max_bus_size;*/
	#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
		times = 128 / max_bus_size + 1;
	#else
		times = 128 / max_bus_size;
	#endif
		for (i = 0 ; i < times; i++) {
			now_address = (chip_info->register_command[3] << 24) + (chip_info->register_command[2] << 16)
							+ (chip_info->register_command[1] << 8) + (chip_info->register_command[0]);
			now_address += (max_bus_size * i);
			TPD_INFO("Now times %d: reg=0x%08X\n", i, now_address);
			now_reg_cmd[3] = (now_address >> 24) & 0xFF;
			now_reg_cmd[2] = (now_address >> 16) & 0xFF;
			now_reg_cmd[1] = (now_address >> 8) & 0xFF;
			now_reg_cmd[0] = now_address & 0xFF;
			if (i == (times - 1) && i != 0)
				hx83102j_register_read(chip_info, now_reg_cmd, 128 % max_bus_size, &data[i * max_bus_size], chip_info->cfg_flag);
			else
				hx83102j_register_read(chip_info, now_reg_cmd, max_bus_size, &data[i * max_bus_size], chip_info->cfg_flag);
			now_address = 0;
		}
		ret += snprintf(temp_buf + ret, len - ret, "command:  %02X, %02X, %02X, %02X\n", chip_info->register_command[3],
							chip_info->register_command[2], chip_info->register_command[1], chip_info->register_command[0]);
		for (loop_i = 0; loop_i < 128; loop_i++) {
			ret += snprintf(temp_buf + ret, len - ret, "0x%2.2X ", data[loop_i]);
			if ((loop_i % 16) == 15) {
				ret += snprintf(temp_buf + ret, len - ret, "\n");
			}
		}
		ret += snprintf(temp_buf + ret, len - ret, "\n");
		if (copy_to_user(buf, temp_buf, len)) {
			TPD_INFO("%s, here:%d\n", __func__, __LINE__);
		}
		if (temp_buf)
			kfree(temp_buf);
		chip_info->hx_proc_send_flag = 1;
	} else {
		chip_info->hx_proc_send_flag = 0;
	}
	if (data)
		kfree(data);
RET_OUT:
	return ret;
}


static size_t hx83102j_proc_register_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
	char buf[81] = {0};
	char buf_tmp[16];
	uint8_t length = 0;
	unsigned long result = 0;
	uint8_t loop_i = 0;
	uint16_t base = 2;
	char *data_str = NULL;
	uint8_t w_data[20];
	uint8_t x_pos[20];
	uint8_t count = 0;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)
			ts->chip_data;

	if (len >= 80) {
		TPD_INFO("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}
	buf[len] = '\0';

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(w_data, 0x0, sizeof(w_data));
	memset(x_pos, 0x0, sizeof(x_pos));

	TPD_INFO("himax %s \n", buf);

	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' && buf[2] == 'x') {
		length = strlen(buf);


		for (loop_i = 0; loop_i < length; loop_i++) {
			if (buf[loop_i] == 'x') {
				x_pos[count] = loop_i;
				count++;
			}
		}

		data_str = strrchr(buf, 'x');
		TPD_INFO("%s: %s.\n", __func__, data_str);
		length = strlen(data_str + 1) - 1;

		if (buf[0] == 'r') {
			if (buf[3] == 'F' && buf[4] == 'E' && length == 4) {
				length = length - base;
				chip_info->cfg_flag = true;
				memcpy(buf_tmp, data_str + base + 1, length);
			} else {
				chip_info->cfg_flag = false;
				memcpy(buf_tmp, data_str + 1, length);
			}

			chip_info->byte_length = length / 2;
			if (!kstrtoul(buf_tmp, 16, &result)) {
				for (loop_i = 0 ; loop_i < chip_info->byte_length ; loop_i++) {
					chip_info->register_command[loop_i] = (uint8_t)(result >> loop_i * 8);
				}
			}
		} else if (buf[0] == 'w') {
			if (buf[3] == 'F' && buf[4] == 'E') {
				chip_info->cfg_flag = true;
				memcpy(buf_tmp, buf + base + 3, length);
			} else {
				chip_info->cfg_flag = false;
				memcpy(buf_tmp, buf + 3, length);
			}
			if (count < 3) {
				chip_info->byte_length = length / 2;
				if (!kstrtoul(buf_tmp, 16, &result)) {
					for (loop_i = 0 ; loop_i < chip_info->byte_length ; loop_i++) {
						chip_info->register_command[loop_i] = (uint8_t)(result >> loop_i * 8);
					}
				}
				if (!kstrtoul(data_str + 1, 16, &result)) {
					for (loop_i = 0 ; loop_i < chip_info->byte_length ; loop_i++) {
						w_data[loop_i] = (uint8_t)(result >> loop_i * 8);
					}
				}
				hx83102j_register_write(chip_info, chip_info->register_command, chip_info->byte_length, w_data, chip_info->cfg_flag);
			} else {
				chip_info->byte_length = x_pos[1] - x_pos[0] - 2;
				for (loop_i = 0; loop_i < count; loop_i++) {
					memcpy(buf_tmp, buf + x_pos[loop_i] + 1, chip_info->byte_length);

					if (!kstrtoul(buf_tmp, 16, &result)) {
						if (loop_i == 0) {
							chip_info->register_command[loop_i] = (uint8_t)(result);
						} else {
							w_data[loop_i - 1] = (uint8_t)(result);
						}
					}
				}
				chip_info->byte_length = count - 1;
				hx83102j_register_write(chip_info, chip_info->register_command, chip_info->byte_length, &w_data[0], chip_info->cfg_flag);
			}
		} else {
			return len;
		}
	}
	return len;
}

static void hx83102j_return_event_stack(struct chip_data_hx83102j *chip_info)
{
	int retry = 20;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	TPD_INFO("%s:entering\n", __func__);
	do {
		TPD_INFO("%s, now %d times\n!", __func__, retry);
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		retry--;
	} while ((tmp_data[1] != 0x00 && tmp_data[0] != 0x00) && retry > 0);

	TPD_INFO("%s: End of setting!\n", __func__);
}
/*IC_BASED_END*/

static int hx83102j_write_read_reg(struct chip_data_hx83102j *chip_info, uint8_t *tmp_addr, uint8_t *tmp_data, uint8_t hb, uint8_t lb)
{
	int cnt = 0;

	do {
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

		msleep(20);
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s:Now tmp_data[0] = 0x%02X, [1] = 0x%02X, [2] = 0x%02X, [3] = 0x%02X\n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
	} while ((tmp_data[1] != hb && tmp_data[0] != lb) && cnt++ < 100);

	if (cnt >= 99) {
		TPD_INFO("hx83102j_write_read_reg ERR Now register 0x%08X : high byte = 0x%02X, low byte = 0x%02X\n", tmp_addr[3], tmp_data[1], tmp_data[0]);
		return -1;
	}

	TPD_INFO("Now register 0x%08X : high byte = 0x%02X, low byte = 0x%02X\n", tmp_addr[3], tmp_data[1], tmp_data[0]);
	return NO_ERR;
}

static void hx83102j_get_DSRAM_data(struct chip_data_hx83102j *chip_info, uint8_t *info_data, uint8_t x_num, uint8_t y_num)
{
	int i = 0;
	unsigned char tmp_addr[4];
	unsigned char tmp_data[4];
	uint8_t max_i2c_size = MAX_RECVS_SZ;
	int m_key_num = 0;
	int total_size = (x_num * y_num + x_num + y_num) * 2 + 4;
	int total_size_temp;
	int data_size = (x_num * y_num + x_num + y_num) * 2;
	int total_read_times = 0;
	int address = 0;
	uint8_t *temp_info_data;
	uint32_t check_sum_cal = 0;
	int fw_run_flag = -1;


	temp_info_data = kzalloc(sizeof(uint8_t) * (total_size + 8), GFP_KERNEL);
	if (temp_info_data == NULL) {
		TPD_INFO("%s: [temp_info_data]Can't allocate enough data\n", __func__);
		return;
	}
	/*1. Read number of MKey R100070E8H to determin data size*/
	m_key_num = 0;

	total_size += m_key_num * 2;

	/* 2. Start DSRAM Rawdata and Wait Data Ready */
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x00;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x5A;
	tmp_data[0] = 0xA5;
	fw_run_flag = hx83102j_write_read_reg(chip_info, tmp_addr, tmp_data, 0xA5, 0x5A);
	if (fw_run_flag < 0) {
		TPD_INFO("%s Data NOT ready => bypass \n", __func__);
		if (temp_info_data)
			kfree(temp_info_data);
		return;
	}

	/* 3. Read RawData */
	total_size_temp = total_size;
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x00;

	if (total_size % max_i2c_size == 0) {
		total_read_times = total_size / max_i2c_size;
	} else {
		total_read_times = total_size / max_i2c_size + 1;
	}

	for (i = 0; i < (total_read_times); i++) {
		if (total_size_temp >= max_i2c_size) {
			hx83102j_register_read(chip_info, tmp_addr, max_i2c_size, &temp_info_data[i * max_i2c_size], false);
			total_size_temp = total_size_temp - max_i2c_size;
		} else {
			hx83102j_register_read(chip_info, tmp_addr, total_size_temp % max_i2c_size,
								&temp_info_data[i * max_i2c_size], false);
		}

		address = ((i + 1) * max_i2c_size);
		tmp_addr[1] = (uint8_t)((address >> 8) & 0x00FF);
		tmp_addr[0] = (uint8_t)((address) & 0x00FF);
	}

	/* 4. FW stop outputing */

	if (chip_info->dsram_flag == false) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
	} else {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		tmp_data[3] = 0x11;
		tmp_data[2] = 0x22;
		tmp_data[1] = 0x33;
		tmp_data[0] = 0x44;
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
	}

	/* 5. Data Checksum Check */
	for (i = 2; i < total_size; i = i + 2) { /* 2:PASSWORD NOT included */
		check_sum_cal += (temp_info_data[i + 1] * 256 + temp_info_data[i]);
		printk("0x%2x:0x%4x ", temp_info_data[i], check_sum_cal);
		if (i % 32 == 0)
			printk("\n");
	}

	if (check_sum_cal % 0x10000 != 0) {
		memcpy(info_data, &temp_info_data[4], data_size * sizeof(uint8_t));
		TPD_INFO("%s check_sum_cal fail=%2x \n", __func__, check_sum_cal);
		if (temp_info_data)
			kfree(temp_info_data);
		return;
	} else {
		memcpy(info_data, &temp_info_data[4], data_size * sizeof(uint8_t));
		TPD_INFO("%s checksum PASS \n", __func__);
	}
	if (temp_info_data)
		kfree(temp_info_data);
}

static void hx83102j_ts_diag_func(struct chip_data_hx83102j *chip_info, int32_t *mutual_data)
{
	int i = 0;
	int j = 0;
	unsigned int index = 0;
	int total_size = (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num +
				 chip_info->hw_res->tx_num + chip_info->hw_res->rx_num) * 2;
	int mutual_size = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;
	int self_size = chip_info->hw_res->tx_num + chip_info->hw_res->rx_num;
	uint8_t *info_data = NULL;
	int32_t new_data;
	/* 1:common dsram,2:100 frame Max,3:N-(N-1)frame */
	int dsram_type = 0;
	char *write_buf = NULL;

	info_data = kcalloc(total_size, sizeof(uint32_t), GFP_KERNEL);
	if (info_data == NULL) {
			TPD_INFO("%s, Failed to allocate memory\n", __func__);
			return;
	}

	write_buf = kcalloc(total_size * 3, sizeof(uint32_t), GFP_KERNEL);
	if (write_buf == NULL) {
			TPD_INFO("%s, Failed to allocate memory\n", __func__);
			if (info_data)
				kfree(info_data);
			return;
	}

	memset(write_buf, 0, total_size * 3);

	dsram_type = chip_info->g_diag_command / 10;

	TPD_INFO("%s:Entering g_diag_command=%d\n!", __func__, chip_info->g_diag_command);

	if (dsram_type == 8) {
		dsram_type = 1;
		TPD_INFO("%s Sorting Mode run sram type1 ! \n", __func__);
	}

	hx83102j_burst_enable(chip_info, 1);
	hx83102j_get_DSRAM_data(chip_info, info_data, chip_info->hw_res->rx_num, chip_info->hw_res->tx_num);

	index = 0;
	for (i = 0; i < chip_info->hw_res->tx_num; i++) {
		for (j = 0; j < chip_info->hw_res->rx_num; j++) {
			new_data = ((int8_t)info_data[index + 1] << 8 | info_data[index]);
			mutual_data[i * chip_info->hw_res->rx_num + j] = new_data;
			index += 2;
		}
	}

	index = mutual_size;
	for (i = 0; i < self_size; i++) {
			new_data = ((int8_t)info_data[index + 1] << 8 | info_data[index]);
			chip_info->diag_self[i] = new_data;
			index += 2;
	}

	if (info_data)
		kfree(info_data);
	if (write_buf)
		kfree(write_buf);
}

static void diag_parse_raw_data(struct chip_data_hx83102j *chip_info, int mul_num, int self_num, uint8_t diag_cmd, int32_t *mutual_data, int32_t *self_data)
{
	int rawdatalen_word;
	int index = 0;
	int temp1, temp2, i;

	if (chip_info->hx_touch_data->hx_rawdata_buf[0] == 0x3A
		&& chip_info->hx_touch_data->hx_rawdata_buf[1] == 0xA3
		&& chip_info->hx_touch_data->hx_rawdata_buf[2] > 0
		&& chip_info->hx_touch_data->hx_rawdata_buf[3] == diag_cmd) {
		rawdatalen_word = chip_info->hx_touch_data->rawdata_size / 2;
		index = (chip_info->hx_touch_data->hx_rawdata_buf[2] - 1) * rawdatalen_word;

		for (i = 0; i < rawdatalen_word; i++) {
			temp1 = index + i;

			if (temp1 < mul_num) {
				mutual_data[index + i] = ((int8_t)chip_info->hx_touch_data->hx_rawdata_buf[i * 2 + 4 + 1])* 256 +
							chip_info->hx_touch_data->hx_rawdata_buf[i * 2 + 4];
			} else {
				temp1 = i + index;
				temp2 = self_num + mul_num;

				if (temp1 >= temp2) {
					break;
				}
				self_data[i + index - mul_num] = (((int8_t)chip_info->hx_touch_data->hx_rawdata_buf[i * 2 + 4 + 1]) << 8) |
							chip_info->hx_touch_data->hx_rawdata_buf[i * 2 + 4];
			}
		}
	}
}

static bool diag_check_sum(struct chip_data_hx83102j *chip_info) /*return checksum value  */
{
	uint16_t check_sum_cal = 0;
	int i;

	/*Check 128th byte crc*/
	check_sum_cal = 0;
	for (i = 0; i < (chip_info->hx_touch_data->touch_all_size - chip_info->hx_touch_data->touch_info_size); i = i + 2) {
		check_sum_cal += (chip_info->hx_touch_data->hx_rawdata_buf[i + 1] * 256 + chip_info->hx_touch_data->hx_rawdata_buf[i]);
	}
	if (check_sum_cal % 0x10000 != 0) {
		TPD_INFO("%s fail=%2X \n", __func__, check_sum_cal);
		return 0;
	}

	return 1;
}

static size_t hx83102j_proc_diag_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
	char messages[80] = {0};
	uint8_t command[2] = {0x00, 0x00};
	uint8_t receive[1];

	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	/* 0: common, other: dsram*/
	int storage_type = 0;
	/* 1:IIR, 2:DC, 3:Bank, 4:IIR2, 5:IIR2_N, 6:FIR2, 7:Baseline, 8:dump coord */
	int rawdata_type = 0;

	memset(receive, 0x00, sizeof(receive));

	if (len >= 80) {
		TPD_INFO("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	if (messages[1] == 0x0A) {
		chip_info->g_diag_command = messages[0] - '0';
	} else {
		chip_info->g_diag_command = (messages[0] - '0') * 10 + (messages[1] - '0');
	}

	storage_type = chip_info->g_diag_command / 10;
	rawdata_type = chip_info->g_diag_command % 10;

	TPD_INFO(" messages       = %s\n"
			 " g_diag_command = 0x%x\n"
			 " storage_type   = 0x%x\n"
			 " rawdata_type   = 0x%x\n",
			 messages, chip_info->g_diag_command, storage_type, rawdata_type);

	if (chip_info->g_diag_command > 0 && rawdata_type == 0) {
		TPD_INFO("[Himax]g_diag_command = 0x%x, storage_type=%d, rawdata_type=%d! Maybe no support!\n", chip_info->g_diag_command, storage_type, rawdata_type);
		chip_info->g_diag_command = 0x00;
	} else {
		TPD_INFO("[Himax]g_diag_command = 0x%x, storage_type=%d, rawdata_type=%d\n", chip_info->g_diag_command, storage_type, rawdata_type);
	}

	if (storage_type == 0 && rawdata_type > 0 && rawdata_type < 8) {
		TPD_INFO("%s, common\n", __func__);
		if (chip_info->dsram_flag) {
			/*(1) Clear DSRAM flag*/
			chip_info->dsram_flag = false;
			/*(2) Enable ISR*/
			hx83102j_enable_interrupt(chip_info, true);
			/*(3) FW leave sram and return to event stack*/
			hx83102j_return_event_stack(chip_info);
		}

		command[0] = chip_info->g_diag_command;
		hx83102j_diag_register_set(chip_info, command[0]);
	} else if (storage_type > 0 && storage_type < 8 && rawdata_type > 0 && rawdata_type < 8) {
		TPD_INFO("%s, dsram\n", __func__);

		/*0. set diag flag*/
		if (chip_info->dsram_flag) {
			/*(1) Clear DSRAM flag*/
			chip_info->dsram_flag = false;
			/*(2) Enable ISR*/
			hx83102j_enable_interrupt(chip_info, true);
			/*(3) FW leave sram and return to event stack*/
			hx83102j_return_event_stack(chip_info);
		}

		switch(rawdata_type) {
		case 1:
			command[0] = 0x09; /*IIR*/
			break;

		case 2:
			command[0] = 0x0A;/*RAWDATA*/
			break;

		case 3:
			command[0] = 0x08;/*Baseline*/
			break;

		default:
			command[0] = 0x00;
			TPD_INFO("%s: Sram no support this type !\n", __func__);
			break;
		}
		hx83102j_diag_register_set(chip_info, command[0]);
		TPD_INFO("%s: Start get raw data in DSRAM\n", __func__);
		/*1. Disable ISR*/
		hx83102j_enable_interrupt(chip_info, false);

		/*2. Set DSRAM flag*/
		chip_info->dsram_flag = true;
	} else {
		/*set diag flag*/
		if (chip_info->dsram_flag) {
			TPD_INFO("return and cancel sram thread!\n");
			/*(1) Clear DSRAM flag*/
			chip_info->dsram_flag = false;
			/*(2) Enable ISR*/
			hx83102j_enable_interrupt(chip_info, true);
			hx83102j_return_event_stack(chip_info);
		}
		command[0] = 0x00;
		chip_info->g_diag_command = 0x00;
		hx83102j_diag_register_set(chip_info, command[0]);
		TPD_INFO("return to normal g_diag_command = 0x%x\n", chip_info->g_diag_command);
	}
	return len;
}


static size_t hx83102j_proc_diag_read(struct seq_file *s, void *v)
{
	size_t ret = 0;
	uint16_t mutual_num;
	uint16_t self_num;
	uint16_t width;
	int dsram_type = 0;
	int data_type = 0;
	int i = 0;
	int j = 0;
	int k = 0;

	struct touchpanel_data *ts = s->private;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)
			ts->chip_data;

	mutual_num = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;
	self_num = chip_info->hw_res->tx_num + chip_info->hw_res->rx_num;
	width = chip_info->hw_res->rx_num;


	s->size = (mutual_num + self_num) * 6 + 256;
	s->buf = kcalloc(s->size, sizeof(char), GFP_KERNEL);
	if (s->buf == NULL) {
		TPD_INFO("[ERROR]%s,%d: Memory allocation falied!\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	memset(s->buf, 0, s->size * sizeof(char));


	dsram_type = chip_info->g_diag_command / 10;
	data_type = chip_info->g_diag_command % 10;


	seq_printf(s, "ChannelStart (rx tx) : %4d, %4d\n\n", chip_info->hw_res->rx_num, chip_info->hw_res->tx_num);

		/* start to show out the raw data in adb shell */
		if ((data_type >= 1 && data_type <= 7)) {
			if (dsram_type > 0)
				hx83102j_ts_diag_func(chip_info, chip_info->hx_touch_data->diag_mutual);

		for (j = 0; j < chip_info->hw_res->rx_num ; j++) {
			for (i = 0; i < chip_info->hw_res->tx_num; i++) {
				k = ((mutual_num - j) - chip_info->hw_res->rx_num * i) - 1;
				seq_printf(s, "%6d", chip_info->hx_touch_data->diag_mutual[k]);
			}
			seq_printf(s, " %6d\n", chip_info->diag_self[j]);
		}

		seq_puts(s, "\n");
		for (i = 0; i < chip_info->hw_res->tx_num; i++) {
			seq_printf(s, "%6d", chip_info->diag_self[i + chip_info->hw_res->rx_num]);
		}
	}

	seq_puts(s, "\n");
	seq_puts(s, "ChannelEnd");
	seq_puts(s,  "\n");

	/* print Mutual/Slef Maximum and Minimum */

	for (i = 0; i < (chip_info->hw_res->tx_num * chip_info->hw_res->rx_num); i++) {
		if (chip_info->hx_touch_data->diag_mutual[i] > chip_info->g_max_mutual) {
			chip_info->g_max_mutual = chip_info->hx_touch_data->diag_mutual[i];
		}
		if (chip_info->hx_touch_data->diag_mutual[i] < chip_info->g_min_mutual) {
			chip_info->g_min_mutual = chip_info->hx_touch_data->diag_mutual[i];
		}
	}

	for (i = 0; i < (chip_info->hw_res->tx_num + chip_info->hw_res->rx_num); i++) {
		if (chip_info->diag_self[i] > chip_info->g_max_self) {
			chip_info->g_max_self = chip_info->diag_self[i];
		}
		if (chip_info->diag_self[i] < chip_info->g_min_self) {
			chip_info->g_min_self = chip_info->diag_self[i];
		}
	}

	seq_printf(s, "Mutual Max:%3d, Min:%3d\n", chip_info->g_max_mutual, chip_info->g_min_mutual);
	seq_printf(s, "Self Max:%3d, Min:%3d\n", chip_info->g_max_self, chip_info->g_min_self);

	/* recovery status after print */
	chip_info->g_max_mutual = 0;
	chip_info->g_min_mutual = 0xFFFF;
	chip_info->g_max_self = 0;
	chip_info->g_min_self = 0xFFFF;

	return ret;
}

static uint8_t hx83102j_read_DD_status(struct chip_data_hx83102j *chip_info, uint8_t *cmd_set, uint8_t *tmp_data)
{
	int cnt = 0;
	uint8_t req_size = chip_info->cmd_set[0];
	uint8_t cmd_addr[4] = {0xFC, 0x00, 0x00, 0x90};
	uint8_t tmp_addr[4] = {0x80, 0x7F, 0x00, 0x10};

	chip_info->cmd_set[3] = 0xAA;
	hx83102j_register_write(chip_info, cmd_addr, 4, chip_info->cmd_set, 0);

	TPD_INFO("%s: chip_info->cmd_set[0] = 0x%02X, chip_info->cmd_set[1] = 0x%02X, chip_info->cmd_set[2] = 0x%02X, chip_info->cmd_set[3] = 0x%02X\n",
		__func__, chip_info->cmd_set[0], chip_info->cmd_set[1], chip_info->cmd_set[2], chip_info->cmd_set[3]);

	for (cnt = 0; cnt < 100; cnt++) {
		hx83102j_register_read(chip_info, cmd_addr, 4, tmp_data, false);

		msleep(10);
		if (tmp_data[3] == 0xBB) {
			TPD_INFO("%s Data ready goto moving data\n", __func__);
			break;
		} else if (cnt >= 99) {
			TPD_INFO("%s Data not ready in FW \n", __func__);
			return FW_NOT_READY;
		}
	}
	hx83102j_register_read(chip_info, tmp_addr, req_size, tmp_data, false);
	return NO_ERR;
}

static size_t hx83102j_proc_DD_debug_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
	int ret = 0;
	uint8_t tmp_data[64];
	uint8_t loop_i = 0;
	char *temp_buf;

	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	if (!chip_info->hx_proc_send_flag) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if (temp_buf == NULL) {
			TPD_INFO("%s: [tmp_buf]Can't allocate enough data\n", __func__);
			ret = -ENOMEM;
			return ret;
		}

		if (chip_info->mutual_set_flag == 1) {
			if (hx83102j_read_DD_status(chip_info, chip_info->cmd_set, tmp_data) == NO_ERR) {
				for (loop_i = 0; loop_i < chip_info->cmd_set[0]; loop_i++) {
					if ((loop_i % 8) == 0) {
						ret += snprintf(temp_buf + ret, len - ret, "0x%02X : ", loop_i);
					}
					ret += snprintf(temp_buf + ret, len - ret, "0x%02X ", tmp_data[loop_i]);
					if ((loop_i % 8) == 7)
						ret += snprintf(temp_buf + ret, len - ret, "\n");
				}
			}
		}

		ret += snprintf(temp_buf + ret, len - ret, "\n");
		if (copy_to_user(buf, temp_buf, len))
			TPD_INFO("%s, here:%d\n", __func__, __LINE__);
		if (temp_buf)
			kfree(temp_buf);
		chip_info->hx_proc_send_flag = 1;
	} else
		chip_info->hx_proc_send_flag = 0;
	return ret;
}

static size_t hx83102j_proc_DD_debug_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
	uint8_t i = 0;
	uint8_t cnt = 2;
	unsigned long result = 0;
	char buf_tmp[20];
	char buf_tmp2[4];
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	if (len >= 20) {
		TPD_INFO("%s: no command exceeds 20 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf_tmp, buff, len)) {
		return -EFAULT;
	}
	memset(buf_tmp2, 0x0, sizeof(buf_tmp2));

	if (buf_tmp[2] == 'x' && buf_tmp[6] == 'x' && buf_tmp[10] == 'x') {
		chip_info->mutual_set_flag = 1;
		for (i = 3; i < 12; i = i + 4) {
			memcpy(buf_tmp2, buf_tmp + i, 2);
			if (!kstrtoul(buf_tmp2, 16, &result))
				chip_info->cmd_set[cnt] = (uint8_t)result;
			else
				TPD_INFO("String to oul is fail in cnt = %d, buf_tmp2 = %s", cnt, buf_tmp2);
			cnt--;
		}
		TPD_INFO("cmd_set[2] = %02X, cmd_set[1] = %02X, cmd_set[0] = %02X\n", chip_info->cmd_set[2], chip_info->cmd_set[1], chip_info->cmd_set[0]);
	} else
		chip_info->mutual_set_flag = 0;

	return len;
}

static int hx83102j_read_FW_status(struct chip_data_hx83102j *chip_info, uint8_t *state_addr, uint8_t *tmp_addr)
{
	uint8_t req_size = 0;
	uint8_t status_addr[4] = {0x44, 0x7F, 0x00, 0x10};
	uint8_t cmd_addr[4] = {0xF8, 0x00, 0x00, 0x90};

	if (state_addr[0] == 0x01) {
		state_addr[1] = 0x04;
		state_addr[2] = status_addr[0];
		state_addr[3] = status_addr[1];
		state_addr[4] = status_addr[2];
		state_addr[5] = status_addr[3];
		req_size = 0x04;
		hx83102j_sense_off(chip_info);
		hx83102j_register_read(chip_info, status_addr, req_size, tmp_addr, false);
		hx83102j_sense_on(chip_info, 1);
	} else if (state_addr[0] == 0x02) {
		state_addr[1] = 0x30;
		state_addr[2] = cmd_addr[0];
		state_addr[3] = cmd_addr[1];
		state_addr[4] = cmd_addr[2];
		state_addr[5] = cmd_addr[3];
		req_size = 0x30;
		hx83102j_register_read(chip_info, cmd_addr, req_size, tmp_addr, false);
	}

	return NO_ERR;
}

static size_t hx83102j_proc_FW_debug_read(struct file *file, char *buf,
		size_t len, loff_t *pos)
{
	int ret = 0;
	uint8_t tmp_data[4] = {0};
	uint8_t tmp_addr[4] = {0};
	char *temp_buf;
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	if (!chip_info->hx_proc_send_flag) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if (temp_buf == NULL) {
			TPD_INFO("%s: [tmp_buf]Can't allocate enough data\n", __func__);
			ret = -ENOMEM;
			return ret;
		}

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 0x900000A8, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		ret += snprintf(temp_buf + ret, len - ret, "%s: 0x900000A8, [0]=0x%02X, [1]=0x%02X, [2]=0x%02X, [3]=0x%02X\n",
						__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xE4;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 0x900000E4, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		ret += snprintf(temp_buf + ret, len - ret, "%s: 0x900000E4, [0]=0x%02X, [1]=0x%02X, [2]=0x%02X, [3]=0x%02X\n",
						__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xF8;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 0x900000F8, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		ret += snprintf(temp_buf + ret, len - ret, "%s: 0x900000F8, [0]=0x%02X, [1]=0x%02X, [2]=0x%02X, [3]=0x%02X\n", __func__,
				tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xE8;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 0x900000E8, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		ret += snprintf(temp_buf + ret, len - ret, "%s: 0x900000E8, [0]=0x%02X, [1]=0x%02X, [2]=0x%02X, [3]=0x%02X\n",
						__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x40;
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: 0x10007f40, tmp_data[0]=%x, tmp_data[1]=%x, tmp_data[2]=%x, tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		ret += snprintf(temp_buf + ret, len - ret, "%s: 0x10007f40, [0]=0x%02X, [1]=0x%02X, [2]=0x%02X, [3]=0x%02X\n",
						__func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		ret += snprintf(temp_buf + ret, len - ret, "Himax Touch Driver Version:  %s\n", HIMAX_DRIVER_VER);

		if (copy_to_user(buf, temp_buf, len))
			TPD_INFO("%s, here:%d\n", __func__, __LINE__);
		if (temp_buf)
			kfree(temp_buf);
		chip_info->hx_proc_send_flag = 1;
	} else
		chip_info->hx_proc_send_flag = 0;
	return ret;
}

static int hx83102j_configuration_init(struct chip_data_hx83102j *chip_info, bool config)
{
	int ret = 0;
	TPD_INFO("%s, configuration init = %d\n", __func__, config);
	return ret;
}

static int hx83102j_ic_reset(struct chip_data_hx83102j *chip_info, uint8_t loadconfig, uint8_t int_off)
{
	int ret = 0;
	chip_info->hx_hw_reset_activate = 1;

	TPD_INFO("%s, status: loadconfig=%d, int_off=%d\n", __func__, loadconfig, int_off);

	if (chip_info->hw_res->reset_gpio) {
		if (int_off) {
			ret = hx83102j_enable_interrupt(chip_info, false);
			if (ret < 0) {
				TPD_INFO("%s: hx83102j enable interrupt failed.\n", __func__);
				return ret;
			}
		}
	#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(chip_info->hw_res, false);
		usleep_range(5000, 5001);
		hx83102j_resetgpio_set(chip_info->hw_res, true);
		usleep_range(5000, 5001);
	#else
		chip_info->g_core_fp.fp_sys_reset(chip_info);
	#endif
		if (loadconfig) {
			ret = hx83102j_configuration_init(chip_info, false);
			if (ret < 0) {
				TPD_INFO("%s: hx83102j configuration init failed.\n", __func__);
				return ret;
			}
			ret = hx83102j_configuration_init(chip_info, true);
			if (ret < 0) {
				TPD_INFO("%s: hx83102j configuration init failed.\n", __func__);
				return ret;
			}
		}
		if (int_off) {
			ret = hx83102j_enable_interrupt(chip_info, true);
			if (ret < 0) {
				TPD_INFO("%s: hx83102j enable interrupt failed.\n", __func__);
				return ret;
			}
		}
	}
	return 0;
}

static size_t hx83102j_proc_reset_write(struct file *file, const char *buff,
										size_t len, loff_t *pos)
{
	char buf_tmp[12];
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	if (len >= 12) {
		TPD_INFO("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf_tmp, buff, len)) {
		return -EFAULT;
	}
	if (buf_tmp[0] == '1')
		hx83102j_ic_reset(chip_info, false, false);
	else if (buf_tmp[0] == '2')
		hx83102j_ic_reset(chip_info, false, true);
	else if (buf_tmp[0] == '3')
		hx83102j_ic_reset(chip_info, true, false);
	else if (buf_tmp[0] == '4')
		hx83102j_ic_reset(chip_info, true, true);
	else if ((buf_tmp[0] == 'z') && (buf_tmp[1] == 'r')) {
		hx83102j_mcu_0f_operation_check(chip_info, 0);
		hx83102j_mcu_0f_operation_check(chip_info, 1);
	} else if ((buf_tmp[0] == 'z') && (buf_tmp[1] == 'w'))
		hx83102j_mcu_0f_operation_dirly(chip_info);
	return len;
}

static size_t hx83102j_proc_sense_on_off_write(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	char buf[80] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)
			ts->chip_data;

	if (len >= 80) {
		TPD_INFO("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	if (buf[0] == '0') {
		hx83102j_sense_off(chip_info);
		TPD_INFO("Sense off \n");
	} else if (buf[0] == '1') {
		if (buf[1] == 's') {
			hx83102j_sense_on(chip_info, 0x00);
			TPD_INFO("Sense on re-map on, run sram \n");
		} else {
			hx83102j_sense_on(chip_info, 0x01);
			TPD_INFO("Sense on re-map off, run flash \n");
		}
	} else {
		TPD_INFO("Do nothing \n");
	}
	return len;
}

#ifdef CONFIG_OPLUS_TP_APK
static void hx83102j_gesture_debug_mode_set(bool on_off)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	char buf[80] = {0};
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x7F;
	tmp_addr[0] = 0xF8;

	if (on_off) {
		chip_info->switch_algo = buf[0];
		chip_info->check_point_format = 1;
		tmp_data[3] = 0xA1;
		tmp_data[2] = 0x1A;
		tmp_data[1] = 0xA1;
		tmp_data[0] = 0x1A;
		hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, 0);
		TPD_INFO("%s: Report 40 trajectory coordinate points .\n", __func__);
	}  else {
		chip_info->switch_algo = 0;
		chip_info->check_point_format = 0;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;

		hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, 0);
		TPD_INFO("%s: close FW enter algorithm switch.\n", __func__);
	}
}


static void hx83102j_debug_mode_set(bool on_off)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	char buf[80] = {0};
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x7F;
	tmp_addr[0] = 0xF8;

	if (on_off) {
		chip_info->switch_algo = buf[0];
		chip_info->check_point_format = 0;
		tmp_data[3] = 0xA5;
		tmp_data[2] = 0x5A;
		tmp_data[1] = 0xA5;
		tmp_data[0] = 0x5A;
		hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, 0);
		TPD_INFO("%s: open FW enter algorithm switch.\n", __func__);
	}  else {
		chip_info->switch_algo = 0;
		chip_info->check_point_format = 0;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;

		hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, 0);
		TPD_INFO("%s: close FW enter algorithm switch.\n", __func__);
	}
}

static void hx83102j_debug_sta_judge(struct chip_data_hx83102j *chip_info)
{
	static struct hx83102j_fw_debug_info last_sta;
	struct hx83102j_fw_debug_info sta;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	memcpy(&sta, &chip_info->hx_touch_data->hx_state_info[3], sizeof(sta));

	if (last_sta.recal0 != sta.recal0) {
		if (sta.recal0) {
			log_buf_write(ts, 1);
		} else {
			log_buf_write(ts, 2);
		}
	}

	if (last_sta.recal1 != sta.recal1) {
		if (sta.recal1) {
			log_buf_write(ts, 4);
		} else {
			log_buf_write(ts, 3);
		}
	}

	if (last_sta.paseline != sta.paseline) {
		if (sta.paseline) {
			log_buf_write(ts, 5);
		} else {
		}
	}

	if (last_sta.palm != sta.palm) {
		if (sta.palm) {
			log_buf_write(ts, 7);
		} else {
			log_buf_write(ts, 6);
		}
	}
	if (last_sta.idle != sta.idle) {
		if (sta.idle) {
			log_buf_write(ts, 9);
		} else {
			log_buf_write(ts, 8);
		}
	}

	if (last_sta.water != sta.water) {
		if (sta.water) {
			log_buf_write(ts, 11);
		} else {
			log_buf_write(ts, 10);
		}
	}

	if (last_sta.hopping != sta.hopping) {
		if (sta.hopping) {
			log_buf_write(ts, 13);
		} else {
			log_buf_write(ts, 12);
		}
	}

	if (last_sta.noise != sta.noise) {
		if (sta.noise) {
			log_buf_write(ts, 15);
		} else {
			log_buf_write(ts, 14);
		}
	}

	if (last_sta.glove != sta.glove) {
		if (sta.glove) {
			log_buf_write(ts, 17);
		} else {
			log_buf_write(ts, 16);
		}
	}

	if (last_sta.border != sta.border) {
		if (sta.border) {
			log_buf_write(ts, 19);
		} else {
			log_buf_write(ts, 18);
		}
	}

	if (last_sta.vr != sta.vr) {
		if (sta.vr) {
			log_buf_write(ts, 21);
		} else {
			log_buf_write(ts, 20);
		}
	}

	if (last_sta.big_small != sta.big_small) {
		if (sta.big_small) {
			log_buf_write(ts, 23);
		} else {
			log_buf_write(ts, 22);
		}
	}

	if (last_sta.one_block != sta.one_block) {
		if (sta.one_block) {
			log_buf_write(ts, 25);
		} else {
			log_buf_write(ts, 24);
		}
	}

	if (last_sta.blewing != sta.blewing) {
		if (sta.blewing) {
			log_buf_write(ts, 27);
		} else {
			log_buf_write(ts, 26);
		}
	}

	if (last_sta.thumb_flying != sta.thumb_flying) {
		if (sta.thumb_flying) {
			log_buf_write(ts, 29);
		} else {
			log_buf_write(ts, 28);
		}
	}

	if (last_sta.border_extend != sta.border_extend) {
		if (sta.border_extend) {
			log_buf_write(ts, 31);
		} else {
			log_buf_write(ts, 30);
		}
	}

	memcpy(&last_sta, &sta, sizeof(last_sta));

	if (tp_debug > 0) {
		TPD_INFO("The sta  is = 0x%02X,0x%02X\n",
				 chip_info->hx_touch_data->hx_state_info[3],
				 chip_info->hx_touch_data->hx_state_info[4]);
	}

	return;
}


#endif
#if defined(HX_PEN_V2)
#define READ_VAR_BIT(var, nb)			(((var) >> (nb)) & 0x1)
static bool wgp_pen_id_crc(uint8_t *p_id)
{
	uint64_t pen_id, input;
	uint8_t hash_id, devidend;
	uint8_t pol = 0x43;
	int i = 0;

	pen_id = (uint64_t)p_id[0] | ((uint64_t)p_id[1] << 8) |
		((uint64_t)p_id[2] << 16) | ((uint64_t)p_id[3] << 24) |
		((uint64_t)p_id[4] << 32) | ((uint64_t)p_id[5] << 40) |
		((uint64_t)p_id[6] << 48);
	hash_id = p_id[7];


	input = pen_id << 6;
	devidend = input >> (44 + 6);

	for (i = (44 + 6 - 1); i >= 0; i--) {
		if (READ_VAR_BIT(devidend, 6))
			devidend = devidend ^ pol;
		devidend = devidend << 1 | READ_VAR_BIT(input, i);
	}
	if (READ_VAR_BIT(devidend, 6))
		devidend = devidend ^ pol;


	if (devidend == hash_id) {
		return 1;
	}

	return 0;
}
#endif

static int hx83102j_ts_work(void *chip_data)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);
	char *buf;
	uint16_t mutual_num;
	uint16_t self_num;
	int ret = 0;
	int check_sum_cal;
	int ts_status = HX_REPORT_COORD;
	int hx_point_num;

	uint8_t hx_state_info_pos;
#if defined(HX_ALG_OVERLAY)
	uint8_t p_hover = 0, p_btn = 0, p_btn2 = 0;
	int8_t p_tilt_x = 0, p_tilt_y = 0;
	int p_x = 0, p_y = 0, p_w = 0;
	int base = 0;
#if defined(HX_PEN_V2)
	uint8_t p_id[8];
	uint8_t p_id_en = 0, p_id_sel = 0;
#endif
#endif

	if (!chip_info->hx_touch_data) {
		TPD_INFO("%s:%d chip_info->hx_touch_data is NULL\n", __func__, __LINE__);
		return 0;
	}

	if (!chip_info->hx_touch_data->hx_coord_buf) {
		TPD_INFO("%s:%d chip_info->hx_touch_data->hx_coord_buf is NULL\n", __func__, __LINE__);
		return 0;
	}

	buf = kzalloc(sizeof(char) * 128, GFP_KERNEL);
	if (!buf) {
		TPD_INFO("%s:%d buf kzalloc error\n", __func__, __LINE__);
		return -ENOMEM;
	}
	chip_info->g_pen_num = chip_info->hx_pen_num;
	chip_info->hx_pen_num = 0;

	hx83102j_burst_enable(chip_info, 0);

	if (chip_info->g_diag_command)
		ret = hx83102j_read_event_stack(chip_info, buf, 128);
	else
		ret = hx83102j_read_event_stack(chip_info, buf, chip_info->hx_touch_data->touch_info_size);
	if (!ret) {
		TPD_INFO("%s: can't read data from chip in normal!\n", __func__);
		goto checksum_fail;
	}

	if (LEVEL_DEBUG == tp_debug) {
		hx83102j_log_touch_data(buf, chip_info);
	}

	check_sum_cal = hx83102j_checksum_cal(chip_info, buf, ts_status);/*????checksum*/

	if (check_sum_cal == CHECKSUM_FAIL) {
		goto checksum_fail;

	} else if (check_sum_cal == ERR_WORK_OUT) {
		goto err_workqueue_out;

	} else if (check_sum_cal == WORK_OUT) {
		goto workqueue_out;
	}

	/*hx83102j_assign_touch_data(buf,ts_status); ??buf??, ??hx_coord_buf*/

	hx_state_info_pos = chip_info->hx_touch_data->touch_info_size - 6;
#if defined(HX_ALG_OVERLAY)
	hx_state_info_pos -= STYLUS_INFO_SZ;
	base = chip_info->hx_touch_data->touch_info_size - STYLUS_INFO_SZ;
#endif

	if(ts_status == HX_REPORT_COORD) {
		memcpy(chip_info->hx_touch_data->hx_coord_buf, &buf[0], chip_info->hx_touch_data->touch_info_size);
		if(buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF) {
			memcpy(chip_info->hx_touch_data->hx_state_info, &buf[hx_state_info_pos], 5);
#ifdef CONFIG_OPLUS_TP_APK
			if (chip_info->debug_mode_sta) {
				hx83102j_debug_sta_judge(chip_info);
			}
#endif
		} else {
			memset(chip_info->hx_touch_data->hx_state_info, 0x00, sizeof(chip_info->hx_touch_data->hx_state_info));
		}
	}

#if defined(HX_ALG_OVERLAY)
#if defined(HX_PEN_V2)
	p_x = chip_info->hx_touch_data->hx_coord_buf[base] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 1];
	p_y = (chip_info->hx_touch_data->hx_coord_buf[base + 2] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 3]);
	p_w = (chip_info->hx_touch_data->hx_coord_buf[base + 4] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 5]);
	p_tilt_x = (int8_t)chip_info->hx_touch_data->hx_coord_buf[base + 6];
	p_tilt_y = (int8_t)chip_info->hx_touch_data->hx_coord_buf[base + 7];
	p_hover = chip_info->hx_touch_data->hx_coord_buf[base + 8] & 0x01;
	p_btn = chip_info->hx_touch_data->hx_coord_buf[base + 8] & 0x02;
	p_btn2 = chip_info->hx_touch_data->hx_coord_buf[base + 8] & 0x04;
	p_id_en = chip_info->hx_touch_data->hx_coord_buf[base + 8] & 0x08;
	if (!p_id_en) {
	} else {
		p_id_sel =
		(chip_info->hx_touch_data->hx_coord_buf[base + 8] & 0xF0) >> 4;
		p_id[p_id_sel*2] =
		chip_info->hx_touch_data->hx_coord_buf[base + 9];
		p_id[p_id_sel*2 + 1] =
		chip_info->hx_touch_data->hx_coord_buf[base + 10];

		if (p_id_sel == 3) {
			ret = wgp_pen_id_crc(p_id);
			if (!ret)
				TPD_INFO("Pen_ID crc not match\n");
		}
	}
#else
	p_x = chip_info->hx_touch_data->hx_coord_buf[base] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 1];
	p_y = (chip_info->hx_touch_data->hx_coord_buf[base + 2] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 3]);
	p_w = (chip_info->hx_touch_data->hx_coord_buf[base + 4] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 5]);
	p_tilt_x = (int8_t)chip_info->hx_touch_data->hx_coord_buf[base + 6];
	p_hover = chip_info->hx_touch_data->hx_coord_buf[base + 7];
	p_btn = chip_info->hx_touch_data->hx_coord_buf[base + 8];
	p_btn2 = chip_info->hx_touch_data->hx_coord_buf[base + 9];
	p_tilt_y = (int8_t)chip_info->hx_touch_data->hx_coord_buf[base + 10];
#endif
#endif

	if (chip_info->g_diag_command) {
		mutual_num = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;
		self_num = chip_info->hw_res->tx_num + chip_info->hw_res->rx_num;
		TPD_INFO("chip_info->hx_touch_data->touch_all_size= %d chip_info->hx_touch_data->touch_info_size = %d, %d\n",
				chip_info->hx_touch_data->touch_all_size, chip_info->hx_touch_data->touch_info_size,
				chip_info->hx_touch_data->touch_all_size - chip_info->hx_touch_data->touch_info_size);
		memcpy(chip_info->hx_touch_data->hx_rawdata_buf, &buf[chip_info->hx_touch_data->touch_info_size],
				chip_info->hx_touch_data->touch_all_size - chip_info->hx_touch_data->touch_info_size);
		if (!diag_check_sum(chip_info)) {
			goto err_workqueue_out;
		}
		diag_parse_raw_data(chip_info, mutual_num, self_num, chip_info->g_diag_command, chip_info->hx_touch_data->diag_mutual, chip_info->diag_self);
	}

	if (chip_info->hx_touch_data->hx_coord_buf[chip_info->hx_touch_info_point_cnt] == 0xff)
		hx_point_num = 0;
	else
		hx_point_num = chip_info->hx_touch_data->hx_coord_buf[chip_info->hx_touch_info_point_cnt] & 0x0f;

	if (p_x >= 0 &&
	p_x <= (ts->resolution_info.max_x - 1) && p_y >= 0 &&
	p_y <= (ts->resolution_info.max_y - 1)) {
		chip_info->hx_pen_num++;
	}
	if (chip_info->hx_pen_num != 0 || chip_info->g_pen_num !=0) {
#ifndef HIMAX_DBG
		ret = IRQ_PEN;
#else
		ret = IRQ_TOUCH;
#endif
	} else {
		ret = IRQ_TOUCH;
	}
	if (buf)
		kfree(buf);
	return ret;

checksum_fail:
err_workqueue_out:
workqueue_out:
	if (buf)
		kfree(buf);
	return -EINVAL;
}
#ifndef HIMAX_DBG
static void hx83102j_get_pen_points(void *chip_data, struct pen_info *points)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);
	uint8_t hx_state_info_pos;
#if defined(HX_ALG_OVERLAY)
	uint8_t p_hover = 0, p_btn = 0, p_btn2 = 0, p_battery = 0;
	int8_t p_tilt_x = 0, p_tilt_y = 0;
	int p_x = 0, p_y = 0, p_w = 0;
	int base = 0;
#if defined(HX_PEN_V2)
	uint8_t p_id[8];
	uint8_t p_id_en = 0, p_id_sel = 0;
#endif
#endif
	hx_state_info_pos = chip_info->hx_touch_data->touch_info_size - 6;
#if defined(HX_ALG_OVERLAY)
	hx_state_info_pos -= STYLUS_INFO_SZ;
	base = chip_info->hx_touch_data->touch_info_size - STYLUS_INFO_SZ;
#endif
#if defined(HX_PEN_V2)
	p_x = chip_info->hx_touch_data->hx_coord_buf[base] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 1];
	p_y = (chip_info->hx_touch_data->hx_coord_buf[base + 2] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 3]);
	p_w = (chip_info->hx_touch_data->hx_coord_buf[base + 4] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 5]);
	p_tilt_x = (int8_t)chip_info->hx_touch_data->hx_coord_buf[base + 6];
	p_tilt_y = (int8_t)chip_info->hx_touch_data->hx_coord_buf[base + 7];
	p_hover = chip_info->hx_touch_data->hx_coord_buf[base + 8] & 0x01;
	p_btn = chip_info->hx_touch_data->hx_coord_buf[base + 8] & 0x02;
	p_btn2 = chip_info->hx_touch_data->hx_coord_buf[base + 8] & 0x04;
	p_id_en = chip_info->hx_touch_data->hx_coord_buf[base + 8] & 0x08;
	if (!p_id_en) {
	} else {
		p_id_sel =
		(chip_info->hx_touch_data->hx_coord_buf[base + 8] & 0xF0) >> 4;
		p_id[p_id_sel*2] =
		chip_info->hx_touch_data->hx_coord_buf[base + 9];
		p_id[p_id_sel*2 + 1] =
		chip_info->hx_touch_data->hx_coord_buf[base + 10];

		if (p_id_sel == 3) {
			ret = wgp_pen_id_crc(p_id);
			if (!ret)
				TPD_INFO("Pen_ID crc not match\n");
		}
	}
#else
	p_x = chip_info->hx_touch_data->hx_coord_buf[base] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 1];
	p_y = (chip_info->hx_touch_data->hx_coord_buf[base + 2] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 3]);
	p_w = (chip_info->hx_touch_data->hx_coord_buf[base + 4] << 8
		| chip_info->hx_touch_data->hx_coord_buf[base + 5]);
	p_tilt_x = (int8_t)chip_info->hx_touch_data->hx_coord_buf[base + 6];
	p_hover = chip_info->hx_touch_data->hx_coord_buf[base + 7] & 0x01;
	p_battery = (chip_info->hx_touch_data->hx_coord_buf[base + 7] & 0x02) >> 1;
	p_btn = chip_info->hx_touch_data->hx_coord_buf[base + 8];
	p_btn2 = chip_info->hx_touch_data->hx_coord_buf[base + 9];
	p_tilt_y = (int8_t)chip_info->hx_touch_data->hx_coord_buf[base + 10];
#endif

	if(p_x >= 0 && p_x <= ts->resolution_info.max_x && p_y >= 0 && p_y <= ts->resolution_info.max_y) {
		points->x = p_x;
		points->y = p_y;
		points->z = p_w;
		points->tilt_x = p_tilt_x;
		points->tilt_y = p_tilt_y;
		points->d = p_hover;
		points->btn1 = p_btn;
		points->btn2 = p_btn2;
		points->battery = p_battery;
		points->status = 1;
#ifdef PEN_BATTERY_LOW_NOTIFIER

		if ((points->battery == 1)&&(points->d == 0)&&(tp_debug == 0)) {
			pen_battery_low_notifier_env(&chip_info->pen_battery);
		} else if ((points->battery == 0)&&(points->d == 0)&&(tp_debug == 0)) {
			/* clear_pen_batlow_flag(&chip_info->pen_battery); */    /* cacel----clear battery low flag */
		} else if ((points->battery == 1)&&(points->d == 0)&&(tp_debug == 1)) {
			pen_battery_low_notifier_env(&chip_info->pen_battery);
		} else if (tp_debug == 2) {
			pen_battery_low_notifier_env(&chip_info->pen_battery);
		}
#endif/*end of PEN_BATTERY_LOW_NOTIFIER*/
	}

/* report coordinates */
/*else {
	TPD_INFO("%s: enter!MMCDEBUG pen333\n", __func__);
	points->x = 0;
	points->y = 0;
	points->z = 0;
	points->tilt_x = 0;
	points->tilt_y = 0;
	points->d = 0;
	points->btn1 = 0;
	points->btn2 = 0;
	points->battery = 0;
	points->status = 0;
	}*/
}
#endif
static int hx83102j_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
	int i, x, y, z = 1, obj_attention = 0;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	for (i = 0; i < ts->max_num; i++) {
		x = chip_info->hx_touch_data->hx_coord_buf[i * 4] << 8 | chip_info->hx_touch_data->hx_coord_buf[i * 4 + 1];
		y = (chip_info->hx_touch_data->hx_coord_buf[i * 4 + 2] << 8 | chip_info->hx_touch_data->hx_coord_buf[i * 4 + 3]);
		z = chip_info->hx_touch_data->hx_coord_buf[i + 4 * max_num];
		if(x >= 0 && x <= ts->resolution_info.max_x && y >= 0 && y <= ts->resolution_info.max_y) {
			points[i].x = x;
			points[i].y = y;
			points[i].width_major = z;
			points[i].touch_major = z;
			points[i].status = 1;
			obj_attention = obj_attention | (0x0001 << i);
		}
	}
	/*TPD_DEBUG("%s:%d  obj_attention = 0x%x\n", __func__, __LINE__, obj_attention);*/

	return obj_attention;
}

static int hx83102j_ftm_process(void *chip_data)
{
#ifdef HX_RST_PIN_FUNC
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	hx83102j_resetgpio_set(chip_info->hw_res, false);
#endif
	return 0;
}

static int hx83102j_get_vendor(void *chip_data, struct panel_info *panel_data)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

#ifdef HIMAX_DBG
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);
	panel_data->test_limit_name = "Himax_limit.img";
	ts->panel_data.test_limit_name = "Himax_limit.img";
	panel_data->fw_name = "Himax_firmware.bin";
#endif

	chip_info->tp_type = panel_data->tp_type;
	chip_info->p_tp_fw = &panel_data->tp_fw;
	TPD_INFO("chip_info->tp_type = %d, panel_data->test_limit_name = %s, panel_data->fw_name = %s\n",
			 chip_info->tp_type, panel_data->test_limit_name, panel_data->fw_name);
	return 0;
}


static int hx83102j_get_chip_info(void *chip_data)
{
	return 1;
}

/**
 * hx83102j_get_fw_id -   get device fw id.
 * @chip_info: struct include i2c resource.
 * Return fw version result.
 */
static uint32_t hx83102j_get_fw_id(struct chip_data_hx83102j *chip_info)
{
	uint32_t current_firmware = 0;
	uint8_t cmd[4];
	uint8_t data[64];

	cmd[3] = 0x10;  /* oplus fw id bin address : 0xc014   , 49172    Tp ic address : 0x 10007014*/
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x14;
	hx83102j_register_read(chip_info, cmd, 4, data, false);

	TPD_DEBUG("%s : data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n", __func__, data[0], data[1], data[2], data[3]);

	current_firmware = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	TPD_INFO("CURRENT_FIRMWARE_ID = 0x%x\n", current_firmware);

	return current_firmware;
}


/*
static void __init get_lcd_vendor(void)
{
	if (strstr(boot_command_line, "1080p_dsi_vdo-1-fps")) {
		chip_info->g_lcd_vendor = 1;
	} else if (strstr(boot_command_line, "1080p_dsi_vdo-2-fps")) {
		chip_info->g_lcd_vendor = 2;
	} else if (strstr(boot_command_line, "1080p_dsi_vdo-3-fps")) {
		chip_info->g_lcd_vendor = 3;
	} else if (strstr(boot_command_line, "1080p_dsi_vdo-7-fps")) {
		chip_info->g_lcd_vendor = 7;
	} else if (strstr(boot_command_line, "1080p_dsi_vdo-8-fps")) {
		chip_info->g_lcd_vendor = 8;
	} else if (strstr(boot_command_line, "1080p_dsi_vdo-9-fps")) {
		chip_info->g_lcd_vendor = 9;
	}
}*/


static fw_check_state hx83102j_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;


	panel_data->tp_fw = hx83102j_get_fw_id(chip_info);
#ifndef HIMAX_DBG
	if (panel_data->manufacture_info.version)
		sprintf(panel_data->manufacture_info.version, "0x%x-%03X", panel_data->tp_fw, chip_info->touch_ver);
#endif

	return FW_NORMAL;
}

static u32 hx83102j_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
	int ret = IRQ_IGNORE;
	/*TPD_INFO("%s.\n", __func__);*/

	if ((gesture_enable == 1) && is_suspended) {
		return IRQ_GESTURE;
	} else {
		ret = hx83102j_ts_work(chip_data);
		if (ret <= IRQ_IGNORE) {
			TPD_INFO("ts work process irq ignore\n");
			ret = IRQ_IGNORE;
		}
	}
	/* TPD_INFO("hx83102j_trigger_reason ret=%d\n", ret);*/
	return ret;
}
/*
static int hx83102j_reset_for_prepare(void *chip_data)
{
	int ret = -1;
	int i2c_error_number = 0;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	TPD_INFO("%s.\n", __func__);
	hx83102j_resetgpio_set(chip_info->hw_res, true);

	return ret;
}
*/
/*
static void hx83102j_resume_prepare(void *chip_data)
{
	hx83102j_reset_for_prepare(chip_data);
	#ifdef HX_ZERO_FLASH
	TPD_DETAIL("It will update fw,if there is power-off in suspend!\n");

	g_zero_event_count = 0;

	hx83102j_enable_interrupt(chip_info, false);

	 trigger reset
	hx83102j_resetgpio_set(chip_info->hw_res, false);
	hx83102j_resetgpio_set(chip_info->hw_res, true);

	chip_info->g_core_fp.fp_0f_operation_dirly();
	chip_info->g_core_fp.fp_reload_disable(0);
	hx83102j_sense_on(0x00);
	 need_modify
	 report all leave event
	hx83102j_report_all_leave_event(ts);

	hx83102j_enable_interrupt(chip_info, true);
	#endif
}
*/
static void hx83102j_exit_esd_mode(void *chip_data)
{
	TPD_INFO("exit esd mode ok\n");
	return;
}

/*
 * return success: 0 ; fail : negative
 */
static int hx83102j_reset(void *chip_data)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	int ret = 0;
	int load_fw_times = 10;

	TPD_INFO("%s.\n", __func__);
#ifdef HIMAX_DBG
	chip_info->first_download_finished = true;
#endif
	if (!chip_info->first_download_finished) {
		TPD_INFO("%s:First download has not finished, don't do reset.\n", __func__);
		return 0;
	}

	chip_info->g_zero_event_count = 0;


	if (!chip_info->is_auto_test) {
		TPD_INFO("%s: Now update fw\n", __func__);
#ifdef HX_ZERO_FLASH
	mutex_lock(&(chip_info->fw_update_lock));
#endif


	hx83102j_enable_interrupt(chip_info, false);


	do {
		load_fw_times--;
		chip_info->g_core_fp.fp_0f_operation_dirly(chip_info);
		ret = chip_info->g_core_fp.fp_reload_disable(chip_info, 0);
	} while (!ret && load_fw_times > 0);

	if (!load_fw_times) {
		TPD_INFO("%s: load_fw_times over 10 times\n", __func__);
	}

	usleep_range(5000, 5100);
	hx83102j_read_OPLUS_FW_ver(chip_info);
	usleep_range(5000, 5100);

	hx83102j_sense_on(chip_info, 0x00);

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	hx83102j_enable_interrupt(chip_info, true);
#endif
#ifdef HX_ZERO_FLASH
	mutex_unlock(&(chip_info->fw_update_lock));
#endif
	} else {
		TPD_INFO("%s: No need to update, is auto test=%d\n", __func__, chip_info->is_auto_test);
		chip_info->is_auto_test = 0;
	}
	return ret;
}

static void hx83102j_ultra_enter(struct chip_data_hx83102j *chip_info)
{
	uint8_t tmp_data[4];
	int rtimes = 0;

	TPD_INFO("%s:entering\n", __func__);

	/* 34 -> 11 */
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:1/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0x11;
		if (hx83102j_bus_write(chip_info, 0x34, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (hx83102j_bus_read(chip_info, 0x34, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x34, correct 0x11 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0x11);

	/* 33 -> 33 */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:2/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0x33;
		if (hx83102j_bus_write(chip_info, 0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (hx83102j_bus_read(chip_info, 0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x33, correct 0x33 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0x33);

	/* 34 -> 22 */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:3/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0x22;
		if (hx83102j_bus_write(chip_info, 0x34, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (hx83102j_bus_read(chip_info, 0x34, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x34, correct 0x22 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0x22);

	/* 33 -> AA */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:4/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0xAA;
		if (hx83102j_bus_write(chip_info, 0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (hx83102j_bus_read(chip_info, 0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x33, correct 0xAA = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0xAA);

	/* 33 -> 33 */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:5/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0x33;
		if (hx83102j_bus_write(chip_info, 0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (hx83102j_bus_read(chip_info, 0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x33, correct 0x33 = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0x33);

	/* 33 -> AA */
	rtimes = 0;
	do {
		if (rtimes > 10) {
			TPD_INFO("%s:6/6 retry over 10 times!\n", __func__);
			return;
		}
		tmp_data[0] = 0xAA;
		if (hx83102j_bus_write(chip_info, 0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi write fail!\n", __func__);
			continue;
		}
		tmp_data[0] = 0x00;
		if (hx83102j_bus_read(chip_info, 0x33, 1, tmp_data) < 0) {
			TPD_INFO("%s: spi read fail!\n", __func__);
			continue;
		}

		TPD_INFO("%s:retry times %d, addr = 0x33, correct 0xAA = current 0x%2.2X\n", __func__, rtimes, tmp_data[0]);
		rtimes++;
	} while (tmp_data[0] != 0xAA);

	TPD_INFO("%s:END\n", __func__);
}

static int hx83102j_enable_black_gesture(struct chip_data_hx83102j *chip_info, bool enable)
{
	int ret = 0;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);
	static char ovl_done = 0;
	int retry_cnt = 0;
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint8_t back_data[4] = {0};

	TPD_INFO("%s:enable=%d, ts->is_suspended=%d \n", __func__, enable, ts->is_suspended);

	if (ts->is_suspended) {
		/*status in suspend*/
		chip_info->hx_esd_reset_activate = 0;
		TPD_INFO("%s: now is in suspend! chip_info->hx_esd_reset_activate new = %d\n", __func__, chip_info->hx_esd_reset_activate);
		if (ovl_done == 0) {
			retry_cnt = 0;
			do {
				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x10;
				tmp_data[3] = 0xA5;
				tmp_data[2] = 0x5A;
				tmp_data[1] = 0xA5;
				tmp_data[0] = 0x5A;
				hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
				back_data[3] = 0xA5;
				back_data[2] = 0x5A;
				back_data[1] = 0xA5;
				back_data[0] = 0x5A;
				hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
				TPD_INFO("%s: tmp_data[0] = 0x%02X, retry_cnt=%d \n", __func__, tmp_data[0], retry_cnt);
				retry_cnt++;
			} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1] ||
							tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);

			if (chip_info->gflagautotest == 1) {
				TPD_INFO("%s:Testing, skip overlay\n", __func__);
			}
#ifdef HX_CODE_OVERLAY
			else
				hx83102j_0f_overlay(chip_info, 2, 1);
#endif
		}
		if (enable) {
			TPD_INFO("%s: now entering SMWP mode!\n", __func__);
			if (ovl_done == 1) {
#ifdef HX_RST_PIN_FUNC
				hx83102j_resetgpio_set(chip_info->hw_res, false);
				usleep_range(5000, 5001);
				hx83102j_resetgpio_set(chip_info->hw_res, true);
				usleep_range(5000, 5001);
#else
				chip_info->g_core_fp.fp_sys_reset(chip_info);
#endif
#ifdef HX_OPT_HW_CRC
				hx83102j_en_hw_crc(chip_info, 1);
#endif
				hx83102j_hx83102j_reload_to_active(chip_info);
			}
#ifdef CONFIG_OPLUS_TP_APK
			if (chip_info->debug_gesture_sta) {
					hx83102j_gesture_debug_mode_set(true);
			}
#endif

		} else {
			/* psensor mode*/
			TPD_INFO("%s: now entering utlra mode!\n", __func__);
			retry_cnt = 0;
			do {
				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x10;
				tmp_data[3] = 0x00;
				tmp_data[2] = 0x00;
				tmp_data[1] = 0x00;
				tmp_data[0] = 0x00;
				hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
				back_data[3] = 0x00;
				back_data[2] = 0x00;
				back_data[1] = 0x00;
				back_data[0] = 0x00;
				hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
				TPD_INFO("%s: tmp_data[0] = 0x%02X, retry_cnt=%d \n", __func__, tmp_data[0], retry_cnt);
				retry_cnt++;
			} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1] ||
						tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);
			hx83102j_ultra_enter(chip_info);
		}
		if (ovl_done == 0)
			ovl_done = 1;
	} else {
		TPD_INFO("%s: Leave suspend and back to normal!\n", __func__);
		/* return back to normal*/
		retry_cnt = 0;
		do {
				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x10;
				tmp_data[3] = 0x00;
				tmp_data[2] = 0x00;
				tmp_data[1] = 0x00;
				tmp_data[0] = 0x00;
				hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
				back_data[3] = 0x00;
				back_data[2] = 0x00;
				back_data[1] = 0x00;
				back_data[0] = 0x00;
				hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
				TPD_INFO("%s: tmp_data[0] = 0x%02X, retry_cnt=%d \n", __func__, tmp_data[0], retry_cnt);
				retry_cnt++;
		} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1]	||
					tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);
		if (!chip_info->first_download_finished) {
			 TPD_INFO("%s:need do overlay.\n", __func__);

			if (chip_info->gflagautotest == 1) {
				TPD_INFO("%s:Testing, skip overlay\n", __func__);
				chip_info->gflagautotest = 0;
			}
#if defined(HX_CODE_OVERLAY)
			else
				hx83102j_0f_overlay(chip_info, 3, 1);
#endif
		}
		ovl_done = 0;
	}
	return ret;
}
/*
static int hx83102j_enable_edge_limit(struct chip_data_hx83102j *chip_info, bool enable)
{
	int ret = 0;
	return ret;
}
*/

static int hx83102j_enable_charge_mode(struct chip_data_hx83102j *chip_info, bool enable)
{
	int ret = 0;
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	TPD_INFO("%s, charge mode enable = %d\n", __func__, enable);

	/*Enable:0x10007F38 = 0xA55AA55A  */
	if (enable) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x38;
		tmp_data[3] = 0xA5;
		tmp_data[2] = 0x5A;
		tmp_data[1] = 0xA5;
		tmp_data[0] = 0x5A;
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
	} else {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0x38;
		tmp_data[3] = 0x77;
		tmp_data[2] = 0x88;
		tmp_data[1] = 0x77;
		tmp_data[0] = 0x88;
		hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
	}

	return ret;
}

/*on = 1:on   0:off */
static int hx83102j_jitter_switch (struct chip_data_hx83102j *chip_info, bool on)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int rtimes = 0;
	int ret = 0;

	TPD_INFO("%s:entering\n", __func__);

	if (!on) {
		do {
			if (rtimes > 10) {
				TPD_INFO("%s:retry over 10, jitter off failed!\n", __func__);
				TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x5A, 0xA5, 0x5A, 0xA5\n", __func__);
				ret = -1;
				break;
			}

			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0xE0;
			tmp_data[3] = 0xA5;
			tmp_data[2] = 0x5A;
			tmp_data[1] = 0xA5;
			tmp_data[0] = 0x5A;
			hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

			hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

			TPD_INFO("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
					 rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
			rtimes++;
		} while (tmp_data[3] != 0xA5 || tmp_data[2] != 0x5A
				 || tmp_data[1] != 0xA5 || tmp_data[0] != 0x5A);
		TPD_INFO("%s:jitter off success!\n", __func__);
	} else {
		do {
			if (rtimes > 10) {
				TPD_INFO("%s:retry over 10, jitter on failed!\n", __func__);
				TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x00, 0x00, 0x00, 0x00\n", __func__);
				ret = -1;
				break;
			}

			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0xE0;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

			hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

			TPD_INFO("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
					 rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
			rtimes++;
		} while (tmp_data[3] == 0xA5 && tmp_data[2] == 0x5A
				 && tmp_data[1] == 0xA5 && tmp_data[0] == 0x5A);
		TPD_INFO("%s:jitter on success!\n", __func__);
	}
	TPD_INFO("%s:END\n", __func__);
	return ret;
}

static int hx83102j_enable_headset_mode(struct chip_data_hx83102j *chip_info, bool enable)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int rtimes = 0;
	int ret = 0;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	if (ts->headset_pump_support) {
		if (enable) {/* insert headset */
			do {
				if (rtimes > 10) {
					TPD_INFO("%s:insert headset failed!\n", __func__);
					TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x5A, 0xA5, 0x5A, 0xA5\n", __func__);
					ret = -1;
					break;
				}

				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0xE8;
				tmp_data[3] = 0xA5;
				tmp_data[2] = 0x5A;
				tmp_data[1] = 0xA5;
				tmp_data[0] = 0x5A;
				hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

				hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
				TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
						   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
				rtimes++;
			} while (tmp_data[3] != 0xA5 || tmp_data[2] != 0x5A
					 || tmp_data[1] != 0xA5 || tmp_data[0] != 0x5A);

			TPD_INFO("%s:insert headset success!\n", __func__);
		} else {/* remove headset  */
			do {
				if (rtimes > 10) {
					TPD_INFO("%s:remove headset failed!\n", __func__);
					TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x5A, 0xA5, 0x5A, 0xA5\n", __func__);
					ret = -1;
					break;
				}

				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0xE8;
				tmp_data[3] = 0x00;
				tmp_data[2] = 0x00;
				tmp_data[1] = 0x00;
				tmp_data[0] = 0x00;
				hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

				hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
				TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
						   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
				rtimes++;
			} while (tmp_data[3] != 0x00 || tmp_data[2] != 0x00
					 || tmp_data[1] != 0x00 || tmp_data[0] != 0x00);

			TPD_INFO("%s:remove headset success!\n", __func__);
		}
	}
	return ret;
}

/* mode = 0:off   1:normal   2:turn right    3:turn left*/
static int hx83102j_rotative_switch(struct chip_data_hx83102j *chip_info, int mode)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int rtimes = 0;
	int ret = 0;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);

	TPD_DETAIL("%s:entering\n", __func__);

	if (ts->fw_edge_limit_support) {
		if (mode == 1 || VERTICAL_SCREEN == chip_info->touch_direction) {/* vertical */
			do {
				if (rtimes > 10) {
					TPD_INFO("%s:rotative normal failed!\n", __func__);
					TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x5A, 0xA5, 0x5A, 0xA5\n", __func__);
					ret = -1;
					break;
				}

				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x3C;
				tmp_data[3] = 0xA5;
				tmp_data[2] = 0x5A;
				tmp_data[1] = 0xA5;
				tmp_data[0] = 0x5A;
				hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

				hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
				TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
						   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
				rtimes++;
			} while (tmp_data[3] != 0xA5 || tmp_data[2] != 0x5A
					 || tmp_data[1] != 0xA5 || tmp_data[0] != 0x5A);

			TPD_INFO("%s:rotative normal success!\n", __func__);

		} else {
			rtimes = 0;
			if (LANDSCAPE_SCREEN_270 == chip_info->touch_direction) { /*turn right*/
				do {
					if (rtimes > 10) {
						TPD_INFO("%s:rotative right failed!\n", __func__);
						TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x3A, 0xA3, 0x3A, 0xA3\n", __func__);
						ret = -1;
						break;
					}

					tmp_addr[3] = 0x10;
					tmp_addr[2] = 0x00;
					tmp_addr[1] = 0x7F;
					tmp_addr[0] = 0x3C;
					tmp_data[3] = 0xA3;
					tmp_data[2] = 0x3A;
					tmp_data[1] = 0xA3;
					tmp_data[0] = 0x3A;
					hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

					hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

					TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
							   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
					rtimes++;
				} while (tmp_data[3] != 0xA3 || tmp_data[2] != 0x3A
						 || tmp_data[1] != 0xA3 || tmp_data[0] != 0x3A);

				TPD_INFO("%s:rotative right success!\n", __func__);

			} else if (LANDSCAPE_SCREEN_90 == chip_info->touch_direction) { /*turn left*/
				do {
					if (rtimes > 10) {
						TPD_INFO("%s:rotative left failed!\n", __func__);
						TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x1A, 0xA1, 0x1A, 0xA1\n", __func__);
						ret = -1;
						break;
					}

					tmp_addr[3] = 0x10;
					tmp_addr[2] = 0x00;
					tmp_addr[1] = 0x7F;
					tmp_addr[0] = 0x3C;
					tmp_data[3] = 0xA1;
					tmp_data[2] = 0x1A;
					tmp_data[1] = 0xA1;
					tmp_data[0] = 0x1A;
					hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

					hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);

					TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n",
						__func__, rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
					rtimes++;
				} while (tmp_data[3] != 0xA1 || tmp_data[2] != 0x1A
						 || tmp_data[1] != 0xA1 || tmp_data[0] != 0x1A);

				TPD_INFO("%s:rotative left success!\n", __func__);
			}
		}
	} else {
		if (mode) {/*open*/
			do {
				if (rtimes > 10) {
					TPD_INFO("%s:open edge limit failed!\n", __func__);
					TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x5A, 0xA5, 0x5A, 0xA5\n", __func__);
					ret = -1;
					break;
				}

				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x3C;
				tmp_data[3] = 0xA5;
				tmp_data[2] = 0x5A;
				tmp_data[1] = 0xA5;
				tmp_data[0] = 0x5A;
				hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

				hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
				TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
						   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
				rtimes++;
			} while (tmp_data[3] != 0xA5 || tmp_data[2] != 0x5A
					 || tmp_data[1] != 0xA5 || tmp_data[0] != 0x5A);

			TPD_INFO("%s:open edge limit success!\n", __func__);

		} else {/*close*/
			do {
				if (rtimes > 10) {
					TPD_INFO("%s:close edge limit failed!\n", __func__);
					TPD_INFO("%s:correct tmp_data[0, 1, 2, 3] = 0x9A, 0xA9, 0x9A, 0xA9\n", __func__);
					ret = -1;
					break;
				}

				tmp_addr[3] = 0x10;
				tmp_addr[2] = 0x00;
				tmp_addr[1] = 0x7F;
				tmp_addr[0] = 0x3C;
				tmp_data[3] = 0xA9;
				tmp_data[2] = 0x9A;
				tmp_data[1] = 0xA9;
				tmp_data[0] = 0x9A;
				hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

				hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
				TPD_DETAIL("%s:retry times %d, current tmp_data[0, 1, 2, 3] = 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", __func__,
						   rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
				rtimes++;
			} while (tmp_data[3] != 0xA9 || tmp_data[2] != 0x9A
					 || tmp_data[1] != 0xA9 || tmp_data[0] != 0x9A);

			TPD_INFO("%s:close edge limit success!\n", __func__);
		}
	}
	TPD_DETAIL("%s:END\n", __func__);
	return ret;
}

static int hx83102j_mode_switch(void *chip_data, work_mode mode, int flag)
{
	int ret = -1;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	switch(mode) {
	case MODE_NORMAL:
		ret = hx83102j_configuration_init(chip_info, true);
		if (ret < 0) {
			TPD_INFO("%s: hx83102j configuration init failed.\n", __func__);
			return ret;
		}

		break;

	case MODE_SLEEP:
		/*device control: sleep mode*/
		ret = hx83102j_configuration_init(chip_info, false);
		if (ret < 0) {
			TPD_INFO("%s: hx83102j configuration init failed.\n", __func__);
			return ret;
		}

		break;

	case MODE_GESTURE:
		ret = hx83102j_enable_black_gesture(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("%s: hx83102j enable gesture failed.\n", __func__);
			return ret;
		}

		break;
#ifndef HIMAX_DBG
	case MODE_GLOVE:

		break;
#endif
	case MODE_EDGE:
		/*ret = hx83102j_enable_edge_limit(chip_info, flag);*/
		ret = hx83102j_rotative_switch(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("%s: hx83102j enable edg & corner limit failed.\n", __func__);
			return ret;
		}

		break;

	case MODE_CHARGE:
		ret = hx83102j_enable_charge_mode(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("%s: enable charge mode : %d failed\n", __func__, flag);
		}
		break;

	case MODE_HEADSET:
		ret = hx83102j_enable_headset_mode(chip_info, flag);
		if (ret < 0) {
			TPD_INFO("%s: enable headset mode : %d failed\n", __func__, flag);
		}
		break;
#ifndef HIMAX_DBG
	case MODE_GAME:
		ret = hx83102j_jitter_switch(chip_info, !flag);
		if (ret < 0) {
			TPD_INFO("%s: enable game mode : %d failed\n", __func__, !flag);
		}
		break;
#endif
	default:
		TPD_INFO("%s: Wrong mode.\n", __func__);
	}

	return ret;
}
#ifdef HIMAX_DBG
static void hx83102j_set_SMWP_enable(struct chip_data_hx83102j *chip_info, uint8_t SMWP_enable, bool suspended)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint8_t back_data[4];
	uint8_t retry_cnt = 0;

	hx83102j_sense_off(chip_info);

	/* Enable:0x10007F10 = 0xA55AA55A */
	do {
		if (SMWP_enable) {
			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0x10;
			tmp_data[3] = 0xA5;
			tmp_data[2] = 0x5A;
			tmp_data[1] = 0xA5;
			tmp_data[0] = 0x5A;
			hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
			back_data[3] = 0XA5;
			back_data[2] = 0X5A;
			back_data[1] = 0XA5;
			back_data[0] = 0X5A;
		} else {
			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0x10;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);
			back_data[3] = 0X00;
			back_data[2] = 0X00;
			back_data[1] = 0X00;
			back_data[0] = 0x00;
		}
		hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
		TPD_INFO("%s: tmp_data[0]=%d, SMWP_enable=%d, retry_cnt=%d \n", __func__, tmp_data[0], SMWP_enable, retry_cnt);
		retry_cnt++;
	} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1] || tmp_data[0] != back_data[0]) && retry_cnt < 10);

	hx83102j_sense_on(chip_info, 0);
}
#endif
static int hx_id_match_oplus(int get_id)
{
	int result = 0xff;
	int i = 0;

	for (i = 0; i < HX_GESUTRE_SZ; i++) {
		if (get_id == hx_common_gesture_id[i]) {
			TPD_INFO("Found it, idx=%d, common=0x%02X, oplus=0x%02X\n",
				i, hx_common_gesture_id[i], hx_oplus_gesture_id[i]);
			result = hx_oplus_gesture_id[i];
			break;
		}
	}

	return result;
}

static int hx83102j_get_gesture_info(void *chip_data, struct gesture_info *gesture)
{
	int i = 0;
	int gesture_sign = 0;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);
	uint8_t *buf;
	int gest_len;
	int check_fc = 0;
	int check_sum_cal;
	int ts_status = HX_REPORT_SMWP_EVENT;


	buf = kzalloc(chip_info->hx_touch_data->event_size * sizeof(uint8_t), GFP_KERNEL);
	if (!buf) {
		TPD_INFO("%s:%d kzalloc buf error\n", __func__, __LINE__);
		return -1;
	}

	hx83102j_burst_enable(chip_info, 0);
	if (!hx83102j_read_event_stack(chip_info, buf, chip_info->hx_touch_data->event_size)) {
		TPD_INFO("%s: can't read data from chip in gesture!\n", __func__);
		if (buf)
			kfree(buf);
		return -1;
	}

	if (LEVEL_DEBUG == tp_debug) {
		hx83102j_log_touch_data(buf, chip_info);
	}

/* cancel print information */
/*
	for (i = 0; i < 128; i++) {
		if (!i) {
			printk("%s: gesture buf data\n", __func__);
		}
		printk("%02d ", buf[i]);
		if ((i + 1) % 8 == 0) {
			printk("\n");
		}
		if (i == (128 - 1)) {
			printk("\n");
		}
	}
*/
	check_sum_cal = hx83102j_checksum_cal(chip_info, buf, ts_status);
	if (check_sum_cal == CHECKSUM_FAIL) {
		if (buf)
			kfree(buf);
		return -1;
	} else if (check_sum_cal == ERR_WORK_OUT) {
		goto err_workqueue_out;
	}

	for (i = 0; i < 4; i++) {
		if (check_fc == 0) {
			if ((buf[0] != 0x00) && ((buf[0] <= 0xFF))) {
				check_fc = 1;
				gesture_sign = buf[i];
			} else {
				check_fc = 0;

				break;
			}
		} else {
			if (buf[i] != gesture_sign) {
				check_fc = 0;

				break;
			}
		}
	}


	if (buf[GEST_PTLG_ID_LEN] != GEST_PTLG_HDR_ID1 ||
		buf[GEST_PTLG_ID_LEN + 1] != GEST_PTLG_HDR_ID2) {
		goto RET_OUT;
	}

	if (buf[GEST_PTLG_ID_LEN] == GEST_PTLG_HDR_ID1 &&
		buf[GEST_PTLG_ID_LEN + 1] == GEST_PTLG_HDR_ID2) {
		gest_len = buf[GEST_PTLG_ID_LEN + 2];
		if (gest_len > 52) {
			gest_len = 52;
		}


		i = 0;
		chip_info->gest_pt_cnt = 0;

#ifdef CONFIG_OPLUS_TP_APK
		if(chip_info->check_point_format == 0) {
#endif
			while (i < (gest_len + 1) / 2) {
				if (i == 6) {
					chip_info->gest_pt_x[chip_info->gest_pt_cnt] = buf[GEST_PTLG_ID_LEN + 4 + i * 2];
				} else {
					chip_info->gest_pt_x[chip_info->gest_pt_cnt] = buf[GEST_PTLG_ID_LEN + 4 + i * 2] * ts->resolution_info.max_x / 255;
				}
				chip_info->gest_pt_y[chip_info->gest_pt_cnt] = buf[GEST_PTLG_ID_LEN + 4 + i * 2 + 1] * ts->resolution_info.max_y / 255;
				i++;

				chip_info->gest_pt_cnt += 1;
			}
#ifdef CONFIG_OPLUS_TP_APK
		} else {
			int j = 0;
			int nn;
			int n = 24;
			int m = 26;
			int pt_num;
			chip_info->gest_pt_cnt = 40;
			if (ts->gesture_buf) {
				pt_num = gest_len + buf[126];
				if (pt_num > 104) {
					pt_num = 104;
				}
				ts->gesture_buf[0] = gesture_sign;
				ts->gesture_buf[1] = buf[127];

				if (ts->gesture_buf[0] == 0x07) {
					for(j = 0; j < gest_len * 2; j = j + 2) {
						ts->gesture_buf[3 + j] = buf[n];
						ts->gesture_buf[3 + j + 1] = buf[n + 1];
						n = n + 4;
					}

					for(nn = 0; nn < (pt_num - gest_len)   * 2 ; nn = nn + 2) {
						ts->gesture_buf[3 + j + nn] = buf[m];
						ts->gesture_buf[3 + j + nn + 1] = buf[m + 1];
						m = m + 4;
					}
					ts->gesture_buf[2] = pt_num;
				} else {
					ts->gesture_buf[2] = gest_len;
					memcpy(&ts->gesture_buf[3], &buf[24], 80);
				}
			}
		}
#endif
		if (gesture_sign == hx_common_gesture_id[0])
			chip_info->gest_pt_cnt = 1;
		if (chip_info->gest_pt_cnt) {
			gesture->gesture_type = hx_id_match_oplus(gesture_sign);/* id */
			gesture->Point_start.x = chip_info->gest_pt_x[0];/* start x */
			gesture->Point_start.y = chip_info->gest_pt_y[0];/* start y */
			gesture->Point_end.x = chip_info->gest_pt_x[1];/* end x */
			gesture->Point_end.y = chip_info->gest_pt_y[1];/* end y */
			gesture->Point_1st.x = chip_info->gest_pt_x[2]; /* 1 */
			gesture->Point_1st.y = chip_info->gest_pt_y[2];
			gesture->Point_2nd.x = chip_info->gest_pt_x[3];/* 2 */
			gesture->Point_2nd.y = chip_info->gest_pt_y[3];
			gesture->Point_3rd.x = chip_info->gest_pt_x[4];/* 3 */
			gesture->Point_3rd.y = chip_info->gest_pt_y[4];
			gesture->Point_4th.x = chip_info->gest_pt_x[5];/* 4 */
			gesture->Point_4th.y = chip_info->gest_pt_y[5];
			gesture->clockwise = chip_info->gest_pt_x[6]; /*  1, 0 */

			/*for (i = 0; i < 6; i++)
			   TPD_DEBUG("%d [ %d  %d ]\n", i, gest_pt_x[i], gest_pt_y[i]);*/
		}
	}


RET_OUT:
	if (buf) {
		kfree(buf);
	}
	return 0;

err_workqueue_out:
	if (buf) {
		kfree(buf);
	}
	return -1;
}

static int hx83102j_power_control(void *chip_data, bool enable)
{
	int ret = 0;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	if (true == enable) {
		/*
		ret = tp_powercontrol_2v8(chip_info->hw_res, true);
		if (ret)
			return -1;
		ret = tp_powercontrol_1v8(chip_info->hw_res, true);
		if (ret)
			return -1;
		*/

	#ifdef HX_RST_PIN_FUNC
		ret = hx83102j_resetgpio_set(chip_info->hw_res, true);
		if (ret)
			return -1;
	#endif
	} else {
	#ifdef HX_RST_PIN_FUNC
		ret = hx83102j_resetgpio_set(chip_info->hw_res, false);
		if (ret)
			return -1;
	#endif
	/*
		ret = tp_powercontrol_1v8(chip_info->hw_res, false);
		if (ret)
			return -1;
		ret = tp_powercontrol_2v8(chip_info->hw_res, false);
		if (ret)
			return -1;
	*/
	}
	return ret;
}
/*
static void store_to_file(int fd, char *format, ...)
{
	va_list args;
	char buf[64] = {0};

	va_start(args, format);
	vsnprintf(buf, 64, format, args);
	va_end(args);

	if (fd >= 0) {
		sys_write(fd, buf, strlen(buf));
	}
}
*/


static int hx83102j_int_pin_test(struct seq_file *s, void *chip_data, struct auto_testdata *syna_testdata)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	struct touchpanel_data *ts = spi_get_drvdata(chip_info->hx_spi);
	int eint_status, eint_count = 0, read_gpio_num = 10;

	TPD_INFO("%s, step 0: begin to check INT-GND short item\n", __func__);
	while (read_gpio_num--) {
		msleep(5);
		eint_status = gpio_get_value(ts->hw_res.irq_gpio);
		if (eint_status == 1)
			eint_count--;
		else
			eint_count++;
		TPD_INFO("%s eint_count = %d  eint_status = %d\n", __func__, eint_count, eint_status);
	}
	TPD_INFO("TP EINT PIN direct short! eint_count = %d\n", eint_count);
	if (eint_count == 10) {
		TPD_INFO("error :  TP EINT PIN direct short!\n");
		seq_printf(s, "TP EINT direct stort\n");

		eint_count = 0;
		return TEST_FAIL;
	}

	return TEST_PASS;
}

static int hx83102j_auto_test(struct seq_file *s, void *chip_data, struct auto_testdata *syna_testdata)
{
	int error_count = 0;
	int ret = THP_AFE_INSPECT_OK;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	char g_file_name_ok[64];
	char g_file_name_ng[64];
	char *g_test_list_log = NULL;
	char *g_project_test_info_log = NULL;
	char *g_company_info_log = NULL;
	int i = 0;
	chip_info->in_self_test = 1;
	chip_info->g_rslt_data_len = 0;
	chip_info->is_auto_test = true;


	ret = hx83102j_self_test_data_init(chip_info);
	if (ret != THP_AFE_INSPECT_OK) {
		TPD_INFO("%s:%d criteria error\n", __func__, __LINE__);
		goto RET_OUT;
	}

	chip_info->g_1kind_raw_size = 5 * chip_info->hw_res->tx_num * chip_info->hw_res->rx_num * 2;
	g_company_info_log = kcalloc(256, sizeof(char), GFP_KERNEL);
	g_test_list_log = kcalloc(256, sizeof(char), GFP_KERNEL);
	g_project_test_info_log = kcalloc(256, sizeof(char), GFP_KERNEL);
	if ((g_company_info_log == NULL) || (g_test_list_log == NULL) || (g_project_test_info_log == NULL)) {
		TPD_INFO("%s:%d g_company_info_log|g_test_list_log|g_project_test_info_log kzalloc buf error\n", __func__, __LINE__);
		goto RET_OUT;
	}
	chip_info->hx83102j_nf_fail_write_count = 0;
	chip_info->g_file_path_ok = kcalloc(256, sizeof(char), GFP_KERNEL);
	if (!chip_info->g_file_path_ok) {
		TPD_INFO("%s:%d chip_info->g_file_path_ok kzalloc buf error\n", __func__, __LINE__);
		goto RET_OUT;
	}
	chip_info->g_file_path_ng = kcalloc(256, sizeof(char), GFP_KERNEL);
	if (!chip_info->g_file_path_ng) {
		TPD_INFO("%s:%d chip_info->g_file_path_ng kzalloc buf error\n", __func__, __LINE__);
		goto RET_OUT;
	}

	if (chip_info->g_rslt_data == NULL) {
		TPD_INFO("chip_info->g_rslt_data is NULL");
		chip_info->g_rslt_data = kcalloc(chip_info->g_1kind_raw_size * chip_info->hx_criteria_item,
							  sizeof(char), GFP_KERNEL);
		if (!chip_info->g_rslt_data) {
			TPD_INFO("%s:%d chip_info->g_rslt_data kzalloc buf error\n", __func__, __LINE__);
			goto RET_OUT;
		}
	} else {
		memset(chip_info->g_rslt_data, 0x00, chip_info->g_1kind_raw_size * chip_info->hx_criteria_item *
			   sizeof(char));
	}

	error_count += hx83102j_chip_self_test(s, chip_info, g_test_list_log);


	if (error_count) {
		snprintf(chip_info->g_file_path_ng,
				 (int)(strlen(HX_RSLT_OUT_PATH_NG) + strlen(g_file_name_ng) + 1),
				 "%s%s", HX_RSLT_OUT_PATH_NG, g_file_name_ng);
		hx83102j_test_data_pop_out(chip_info, HX_AUTO_TEST, g_test_list_log, g_company_info_log,
						g_project_test_info_log, chip_info->g_rslt_data, chip_info->g_file_path_ng);

	} else {
		snprintf(chip_info->g_file_path_ok,
				 (int)(strlen(HX_RSLT_OUT_PATH_OK) + strlen(g_file_name_ok) + 1),
				 "%s%s", HX_RSLT_OUT_PATH_OK, g_file_name_ok);
		hx83102j_test_data_pop_out(chip_info, HX_AUTO_TEST, g_test_list_log, g_company_info_log,
						g_project_test_info_log, chip_info->g_rslt_data, chip_info->g_file_path_ok);
	}

	/*tp_test_write(syna_testdata->fp, syna_testdata->length, chip_info->g_rslt_data, (size_t)strlen(chip_info->g_rslt_data), syna_testdata->pos);*/

	seq_printf(s, "%d errors. %s\n", error_count, error_count ? "" : "All test passed.");
	TPD_INFO(" TP auto test %d error(s). %s\n", error_count, error_count ? "" : "All test passed.");

	/** Need to let it touch in end of test **/
	hx83102j_set_N_frame(chip_info, 1, HIMAX_INSPECTION_NOISE);
	/* return Normal from dsram rawdata mode*/
	hx83102j_diag_register_set(chip_info, 0);
	hx83102j_sense_off(chip_info);
	hx83102j_sense_on(chip_info, 1);
	ret = hx83102j_enable_interrupt(chip_info, true);
	chip_info->in_self_test = 0;

RET_OUT:
	chip_info->in_self_test = 0;
	if (chip_info->hx83102j_nf_inspection_criteria != NULL) {
		for (i = 0; i < chip_info->hx_criteria_size; i++) {
			if (chip_info->hx83102j_nf_inspection_criteria[i] != NULL) {
				kfree(chip_info->hx83102j_nf_inspection_criteria[i]);
				chip_info->hx83102j_nf_inspection_criteria[i] = NULL;
			}
		}
		if (chip_info->hx83102j_nf_inspection_criteria)
			kfree(chip_info->hx83102j_nf_inspection_criteria);
		chip_info->hx83102j_nf_inspection_criteria = NULL;
		TPD_INFO("Now it have free the chip_info->hx83102j_nf_inspection_criteria!\n");
	} else {
		TPD_INFO("No Need to free chip_info->hx83102j_nf_inspection_criteria!\n");
	}

	if (chip_info->hx83102j_nf_inspt_crtra_flag) {
		kfree(chip_info->hx83102j_nf_inspt_crtra_flag);
		chip_info->hx83102j_nf_inspt_crtra_flag = NULL;
	}

	if (chip_info->g_file_path_ok) {
		kfree(chip_info->g_file_path_ok);
		chip_info->g_file_path_ok = NULL;
	}
	if (chip_info->g_file_path_ng) {
		kfree(chip_info->g_file_path_ng);
		chip_info->g_file_path_ng = NULL;
	}
	if (g_test_list_log) {
		kfree(g_test_list_log);
		g_test_list_log = NULL;
	}
	if (g_project_test_info_log) {
		kfree(g_project_test_info_log);
		g_project_test_info_log = NULL;
	}
	if (g_company_info_log) {
		kfree(g_company_info_log);
		g_company_info_log = NULL;
	}
	return error_count;
}

static void hx83102j_read_debug_data(struct seq_file *s, void *chip_data, int debug_data_type)
{
	uint16_t mutual_num;
	uint16_t self_num;
	uint16_t width;
	int i = 0;
	int j = 0;
	int k = 0;
	int32_t *data_mutual_sram;
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};

	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	if (!chip_info)
		return;

	data_mutual_sram = kzalloc(chip_info->hw_res->tx_num * chip_info->hw_res->rx_num * sizeof(int32_t), GFP_KERNEL);
	if (!data_mutual_sram) {
		goto RET_OUT;
	}

	mutual_num = chip_info->hw_res->tx_num * chip_info->hw_res->rx_num;
	self_num = chip_info->hw_res->tx_num + chip_info->hw_res->rx_num; /*don't add KEY_COUNT*/
	width = chip_info->hw_res->rx_num;
	seq_printf(s, "ChannelStart (rx tx) : %4d, %4d\n\n", chip_info->hw_res->rx_num, chip_info->hw_res->tx_num);

	/*start to show debug data*/
	switch (debug_data_type) {
	case DEBUG_DATA_BASELINE:
		seq_printf(s, "Baseline data: \n");
		TPD_INFO("Baseline data: \n");
		break;

	case DEBUG_DATA_RAW:
		seq_printf(s, "Raw data: \n");
		TPD_INFO("Raw data: \n");
		break;

	case DEBUG_DATA_DELTA:
		seq_printf(s, "Delta data: \n");
		TPD_INFO("Delta data: \n");
		break;

	case DEBUG_DATA_DOWN:
		seq_printf(s, "Finger down data: \n");
		TPD_INFO("Finger down data: \n");
		break;

	default :
		seq_printf(s, "No this debug datatype \n");
		TPD_INFO("No this debug datatype \n");
		goto RET_OUT;
		break;
	}
	if (debug_data_type == DEBUG_DATA_DOWN) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xF8;

		tmp_data[3] = 0xA5;
		tmp_data[2] = 0x5A;
		tmp_data[1] = 0xA5;
		tmp_data[0] = 0x5A;
		hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, 0);
		hx83102j_diag_register_set(chip_info, DEBUG_DATA_DELTA);
	} else {
		hx83102j_diag_register_set(chip_info, debug_data_type);
	}
	TPD_INFO("%s: Start get debug data in DSRAM\n", __func__);
	chip_info->dsram_flag = true;

	hx83102j_ts_diag_func(chip_info, data_mutual_sram);

	for (j = 0; j < chip_info->hw_res->rx_num; j++) {
		for (i = 0; i < chip_info->hw_res->tx_num; i++) {
			k = ((mutual_num - j) - chip_info->hw_res->rx_num * i) - 1;
			seq_printf(s, "%6d", data_mutual_sram[k]);
		}
		seq_printf(s, " %6d\n", chip_info->diag_self[j]);
	}

	seq_printf(s, "\n");
	for (i = 0; i < chip_info->hw_res->tx_num; i++) {
		seq_printf(s, "%6d", chip_info->diag_self[i]);
	}
	/*Clear DSRAM flag*/
	hx83102j_diag_register_set(chip_info, 0x00);
	chip_info->dsram_flag = false;
	hx83102j_return_event_stack(chip_info);

	seq_printf(s, "\n");
	seq_printf(s, "ChannelEnd");
	seq_printf(s, "\n");

	TPD_INFO("%s, here:%d\n", __func__, __LINE__);

	if (debug_data_type == DEBUG_DATA_DOWN) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xF8;

		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;
		hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, 0);
	}

RET_OUT:
	if (data_mutual_sram) {
		kfree(data_mutual_sram);
	}

	return;
}

static void hx83102j_baseline_read(struct seq_file *s, void *chip_data)
{
	hx83102j_read_debug_data(s, chip_data, DEBUG_DATA_BASELINE);
	hx83102j_read_debug_data(s, chip_data, DEBUG_DATA_RAW);
	return;
}

static void hx83102j_delta_read(struct seq_file *s, void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	hx83102j_read_debug_data(s, chip_data, DEBUG_DATA_DELTA);
#ifdef CONFIG_OPLUS_TP_APK
	if (chip_info->debug_mode_sta) {
		hx83102j_read_debug_data(s, chip_data, DEBUG_DATA_DOWN);
	}
#endif /*end of CONFIG_OPLUS_TP_APK*/
	return;
}

static void hx83102j_main_register_read(struct seq_file *s, void *chip_data)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	seq_printf(s, "PANEL_VER : %X\n", chip_info->vendor_panel_ver);
	seq_printf(s, "FW_VER : %X\n", chip_info->vendor_fw_ver);

	seq_printf(s, "CID_VER : %X\n", (chip_info->fw_cid_ver));

	seq_printf(s, "Project ID = %X\n", chip_info->vendor_proj_info);
	return;
}

/*Reserved node*/
static void hx83102j_reserve_read(struct seq_file *s, void *chip_data)
{
	return;
}

static fw_update_state hx83102j_fw_update(void *chip_data, const struct firmware *fw, bool force)
{
	uint32_t CURRENT_FIRMWARE_ID = 0, FIRMWARE_ID = 0;
	uint8_t cmd[4];
	uint8_t data[64];
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	const uint8_t *p_fw_id = NULL;

	if (fw) {
		if (chip_info->g_fw_buf) {
			chip_info->g_fw_len = fw->size;
			memcpy(chip_info->g_fw_buf, fw->data, fw->size);
			chip_info->g_fw_sta = true;
		}
	}
	if (fw == NULL) {
		TPD_INFO("fw is NULL\n");
#ifndef HIMAX_DBG
		return FW_NO_NEED_UPDATE;
#else
		goto update_dirly;
#endif
	}

	p_fw_id = fw->data + 49172;

	if (!chip_info) {
		TPD_INFO("Chip info is NULL\n");
		return 0;
	}

	TPD_INFO("%s is called\n", __func__);



	CURRENT_FIRMWARE_ID = (*p_fw_id << 24) | (*(p_fw_id + 1) << 16) | (*(p_fw_id + 2) << 8) | *(p_fw_id + 3);


	cmd[3] = 0x10;  /*oplus fw id bin address : 0xc014   , 49172    Tp ic address : 0x 10007014*/
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x14;
	hx83102j_register_read(chip_info, cmd, 4, data, false);
	FIRMWARE_ID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	TPD_INFO("CURRENT TP FIRMWARE ID is 0x%x, FIRMWARE IMAGE ID is 0x%x\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID);


	/*step 3:Get into program mode*/
	/********************get into prog end************/
	/*step 4:flash firmware zone*/
	TPD_INFO("update-----------------firmware ------------------update!\n");
#ifndef HIMAX_DBG
	chip_info->g_core_fp.fp_firmware_update_0f(chip_info, fw);
#else
update_dirly:
	TPD_INFO("update----------firmware --------update--------dirly!\n");
	chip_info->g_core_fp.fp_firmware_update_0f(chip_info, NULL);
#endif
	chip_info->g_core_fp.fp_reload_disable(chip_info, 0);
	msleep(10);

	hx83102j_read_OPLUS_FW_ver(chip_info);
	hx83102j_sense_on(chip_info, 0x00);
	msleep(10);

	hx83102j_enable_interrupt(chip_info, true);


	chip_info->first_download_finished = true;
	return FW_UPDATE_SUCCESS;
}


/*
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM

static int hx83102j_get_usb_state(void)
{
	return 0;
}
#else
static int hx83102j_get_usb_state(void)
{
	return 0;
}
#endif
*/

static int hx83102j_reset_gpio_control(void *chip_data, bool enable)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;
	if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
		TPD_INFO("%s: set reset state %d\n", __func__, enable);
	#ifdef HX_RST_PIN_FUNC
		hx83102j_resetgpio_set(chip_info->hw_res, enable);
	#endif
		TPD_DETAIL("%s: set reset state END\n", __func__);
	}
	return 0;
}

static void hx83102j_set_touch_direction(void *chip_data, uint8_t dir)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	chip_info->touch_direction = dir;
}

static uint8_t hx83102j_get_touch_direction(void *chip_data)
{
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	return chip_info->touch_direction;
}
/*Himax_DB_Test Start*/
void hx83102j_freq_hop_trigger(void *chip_data)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	int rtimes = 0;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	TPD_INFO("send cmd to tigger frequency hopping here!!!\n");
	chip_info->hx83102j_freq_point = 1 - chip_info->hx83102j_freq_point;
	if (chip_info->hx83102j_freq_point) {/*hop to frequency 130K*/
		do {
			if (rtimes > 10) {
				TPD_INFO("%s:frequency hopping failed!\n", __func__);
				TPD_INFO("%s:correct tmp_data[0,1,2,3] = 0x5A,0xA5,0x5A,0xA5\n", __func__);
				break;
			}

			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0xC4;
			tmp_data[3] = 0xA5;
			tmp_data[2] = 0x5A;
			tmp_data[1] = 0xA5;
			tmp_data[0] = 0x5A;
			hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

			hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
			TPD_DETAIL("%s:retry times %d, current tmp_data[0,1,2,3] = 0x%2.2X,0x%2.2X,0x%2.2X,0x%2.2X\n",
				__func__, rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
			rtimes++;
		} while (tmp_data[3] != 0xA5 || tmp_data[2] != 0x5A
				 || tmp_data[1] != 0xA5 || tmp_data[0] != 0x5A);

		if (rtimes <= 10) {
			TPD_INFO("%s:hopping frequency to 130K success!\n", __func__);
		}
	} else {/*hop to frequency 75K*/
		do {
			if (rtimes > 10) {
				TPD_INFO("%s:frequency hopping failed!\n", __func__);
				TPD_INFO("%s:correct tmp_data[0,1,2,3] = 0x3A,0xA3,0x3A,0xA3\n", __func__);
				break;
			}

			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0xC4;
			tmp_data[3] = 0xA3;
			tmp_data[2] = 0x3A;
			tmp_data[1] = 0xA3;
			tmp_data[0] = 0x3A;
			hx83102j_flash_write_burst(chip_info, tmp_addr, tmp_data);

			hx83102j_register_read(chip_info, tmp_addr, 4, tmp_data, false);
			TPD_DETAIL("%s:retry times %d, current tmp_data[0,1,2,3] = 0x%2.2X,0x%2.2X,0x%2.2X,0x%2.2X\n",
				__func__, rtimes, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
			rtimes++;
		} while (tmp_data[3] != 0xA3 || tmp_data[2] != 0x3A
				 || tmp_data[1] != 0xA3 || tmp_data[0] != 0x3A);

		if (rtimes <= 10) {
			TPD_INFO("%s:hopping frequency to 75K success!\n", __func__);
		}
	}
}
static int hx83102j_test_prepare(void *chip_data, struct auto_testdata *hx_testdata)
{
	int ret = 0;
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)chip_data;

	/*init criteria data*/
	hx_testdata->tp_fw = chip_info->vendor_proj_info;
	hx_testdata->dev_tp_fw = chip_info->fw_cid_ver;
	/*init criteria data*/

	if (hx_testdata->fw_test) {
#ifdef HX_ZERO_FLASH
		mutex_lock(&(chip_info->fw_update_lock));
#endif
		TPD_INFO("%s: update MPFW\n", __func__);
		chip_info->g_core_fp.fp_firmware_update_0f(chip_info, hx_testdata->fw_test);
		usleep_range(5000, 5100);
		chip_info->g_core_fp.fp_reload_disable(chip_info, 0);
		usleep_range(5000, 5100);
		hx83102j_read_OPLUS_FW_ver(chip_info);
		msleep(5);
		hx83102j_sense_on(chip_info, 0x00);
	}

	ret = hx83102j_enable_interrupt(chip_info, false);
#ifdef HX_ZERO_FLASH
	mutex_unlock(&(chip_info->fw_update_lock));
#endif
	return 0;
}
/*Himax_DB_Test End*/

static struct oplus_touchpanel_operations hx83102j_ops = {
	.ftm_process      = hx83102j_ftm_process,
	.get_vendor       = hx83102j_get_vendor,
	.get_chip_info    = hx83102j_get_chip_info,
	.reset            = hx83102j_reset,
	.power_control    = hx83102j_power_control,
	.fw_check         = hx83102j_fw_check,
	.fw_update        = hx83102j_fw_update,
	.trigger_reason   = hx83102j_trigger_reason,
	.get_touch_points = hx83102j_get_touch_points,
#ifndef HIMAX_DBG
	.get_pen_points	  = hx83102j_get_pen_points,
#endif
	.get_gesture_info = hx83102j_get_gesture_info,
	.mode_switch      = hx83102j_mode_switch,
	.exit_esd_mode    = hx83102j_exit_esd_mode,
	/*.resume_prepare = hx83102j_resume_prepare,*/
	/*.get_usb_state    = hx83102j_get_usb_state,*/
	/*.black_screen_test = hx83102j_black_screen_test,*/
	.reset_gpio_control = hx83102j_reset_gpio_control,
	.set_touch_direction    = hx83102j_set_touch_direction,
	.get_touch_direction    = hx83102j_get_touch_direction,
/*Himax_DB_Test Start*/
	.freq_hop_trigger = hx83102j_freq_hop_trigger,
/*Himax_DB_Test End*/
};

static struct himax_proc_operations hx83102j_proc_ops = {
	.test_prepare = hx83102j_test_prepare,
	.test_finish = hx83102j_test_finish,
	.int_pin_test = hx83102j_int_pin_test,
	.self_test = hx83102j_auto_test,
	.blackscreen_test = hx83102j_black_screen_test,
	.himax_proc_register_write =  hx83102j_proc_register_write,
	.himax_proc_register_read =  hx83102j_proc_register_read,
	.himax_proc_diag_write =  hx83102j_proc_diag_write,
	.himax_proc_diag_read =  hx83102j_proc_diag_read,
	.himax_proc_DD_debug_read =  hx83102j_proc_DD_debug_read,
	.himax_proc_DD_debug_write =  hx83102j_proc_DD_debug_write,
	.himax_proc_FW_debug_read =  hx83102j_proc_FW_debug_read,
	.himax_proc_reset_write =  hx83102j_proc_reset_write,
	.himax_proc_sense_on_off_write =  hx83102j_proc_sense_on_off_write,
#ifdef HX_ENTER_ALGORITHM_NUMBER
	/*.himax_proc_enter_algorithm_switch_write = hx83102j_enter_algorithm_number_write,*/
	/*.himax_proc_enter_algorithm_switch_read  = hx83102j_enter_algorithm_number_read,*/
#endif
};

static struct engineer_test_operations hx_engineer_test_ops = {
	.auto_test				  = hx_auto_test,
	.black_screen_test		  = hx_black_screen_test,
};

static struct debug_info_proc_operations debug_info_proc_ops = {
	/*.limit_read    = hx83102j_limit_read,*/
	.delta_read    = hx83102j_delta_read,
	.baseline_read = hx83102j_baseline_read,
	.main_register_read = hx83102j_main_register_read,
	.reserve_read = hx83102j_reserve_read,
};

#ifdef CONFIG_OPLUS_TP_APK

static void hx83102j_enter_hopping_write(bool on_off)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};


	if (on_off) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xF8;

		tmp_data[3] = 0xA5;
		tmp_data[2] = 0x5A;
		tmp_data[1] = 0xA5;
		tmp_data[0] = 0x5A;
		hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, 0);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xC4;

		tmp_data[3] = 0xA1;
		tmp_data[2] = 0x1A;
		tmp_data[1] = 0xA1;
		tmp_data[0] = 0x1A;
		hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, 0);
		TPD_INFO("%s: open himax enter hopping write.\n", __func__);
	} else {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xF8;

		tmp_data[3] = 0;
		tmp_data[2] = 0;
		tmp_data[1] = 0;
		tmp_data[0] = 0;
		hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, 0);

		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F;
		tmp_addr[0] = 0xC4;

		tmp_data[3] = 0;
		tmp_data[2] = 0;
		tmp_data[1] = 0;
		tmp_data[0] = 0;
		hx83102j_register_write(chip_info, tmp_addr, 4, tmp_data, 0);

		TPD_INFO("%s: close himax hopping write.\n", __func__);
	}
}


static void hx83102j_apk_game_set(void *chip_data, bool on_off)
{
	hx83102j_mode_switch(chip_data, MODE_GAME, (int)on_off);
}

static bool hx83102j_apk_game_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	return chip_info->lock_point_status;
}

static void hx83102j_apk_debug_set(void *chip_data, bool on_off)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	hx83102j_debug_mode_set(on_off);
	chip_info->debug_mode_sta = on_off;
}

static bool hx83102j_apk_debug_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	return chip_info->debug_mode_sta;
}

static void hx83102j_apk_gesture_debug(void *chip_data, bool on_off)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	chip_info->debug_gesture_sta = on_off;
}

static bool  hx83102j_apk_gesture_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	return chip_info->debug_gesture_sta;
}

static int  hx83102j_apk_gesture_info(void *chip_data, char *buf, int len)
{
	int ret = 0;
	int i;
	int num;
	u8 temp;
	struct chip_data_hx83102j *chip_info;
	struct touchpanel_data *ts;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	ts = spi_get_drvdata(chip_info->hx_spi);

	if(len < 2) {
		return 0;
	}
	buf[0] = 255;

	temp = ts->gesture_buf[0];
	if (temp == 0x00) {
		temp = ts->gesture_buf[1] | 0x80;
	}
	buf[0] = temp;


	num = ts->gesture_buf[2];

	if(num > 40) {
		num = 40;
	}
	ret = 2;

	buf[1] = num;

	for (i = 0; i < num; i++) {
		int x;
		int y;
		x = ts->gesture_buf[i * 2 + 3];
		x = x * ts->resolution_info.max_x / 255;

		y = ts->gesture_buf[i * 2 + 4];
		y = y * ts->resolution_info.max_y / 255;




		if (len < i * 4 + 2) {
			break;
		}
		buf[i * 4 + 2] = x & 0xFF;
		buf[i * 4 + 3] = (x >> 8) & 0xFF;
		buf[i * 4 + 4] = y & 0xFF;
		buf[i * 4 + 5] = (y >> 8) & 0xFF;
		ret += 4;
	}

	return ret;
}


static void hx83102j_apk_earphone_set(void *chip_data, bool on_off)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	hx83102j_mode_switch(chip_data, MODE_HEADSET, (int)on_off);
	chip_info->earphone_sta = on_off;
}

static bool hx83102j_apk_earphone_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	return chip_info->earphone_sta;
}

static void hx83102j_apk_charger_set(void *chip_data, bool on_off)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	hx83102j_mode_switch(chip_data, MODE_CHARGE, (int)on_off);
	chip_info->plug_status = on_off;
}

static bool hx83102j_apk_charger_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	return chip_info->plug_status;
}

static void hx83102j_apk_noise_set(void *chip_data, bool on_off)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	hx83102j_enter_hopping_write(on_off);
	chip_info->noise_sta = on_off;
}

static bool hx83102j_apk_noise_get(void *chip_data)
{
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;

	return chip_info->noise_sta;
}


static int  hx83102j_apk_tp_info_get(void *chip_data, char *buf, int len)
{
	int ret;
	struct chip_data_hx83102j *chip_info;
	chip_info = (struct chip_data_hx83102j *)chip_data;
	ret = snprintf(buf, len, "IC:HIMAX%06X\nFW_VER:0x%04X\nCH:%dX%d\n",
				   0x83102J,
				   chip_info->fw_cid_ver,
				   chip_info->hw_res->tx_num,
				   chip_info->hw_res->rx_num);
	if (ret > len) {
		ret = len;
	}

	return ret;
}

static void hx83102j_init_oplus_apk_op(struct touchpanel_data *ts)
{
	if (ts->apk_op == NULL)
		ts->apk_op = kzalloc(sizeof(APK_OPERATION), GFP_KERNEL);
	if(ts->apk_op) {
		ts->apk_op->apk_game_set = hx83102j_apk_game_set;
		ts->apk_op->apk_game_get = hx83102j_apk_game_get;
		ts->apk_op->apk_debug_set = hx83102j_apk_debug_set;
		ts->apk_op->apk_debug_get = hx83102j_apk_debug_get;
		/*apk_op->apk_proximity_set = ili_apk_proximity_set;*/
		/*apk_op->apk_proximity_dis = ili_apk_proximity_dis;*/
		ts->apk_op->apk_noise_set = hx83102j_apk_noise_set;
		ts->apk_op->apk_noise_get = hx83102j_apk_noise_get;
		ts->apk_op->apk_gesture_debug = hx83102j_apk_gesture_debug;
		ts->apk_op->apk_gesture_get = hx83102j_apk_gesture_get;
		ts->apk_op->apk_gesture_info = hx83102j_apk_gesture_info;
		ts->apk_op->apk_earphone_set = hx83102j_apk_earphone_set;
		ts->apk_op->apk_earphone_get = hx83102j_apk_earphone_get;
		ts->apk_op->apk_charger_set = hx83102j_apk_charger_set;
		ts->apk_op->apk_charger_get = hx83102j_apk_charger_get;
		ts->apk_op->apk_tp_info_get = hx83102j_apk_tp_info_get;
		/*apk_op->apk_data_type_set = ili_apk_data_type_set;*/
		/*apk_op->apk_rawdata_get = ili_apk_rawdata_get;*/
		/*apk_op->apk_diffdata_get = ili_apk_diffdata_get;*/
		/*apk_op->apk_basedata_get = ili_apk_basedata_get;*/
		/*ts->apk_op->apk_backdata_get = nova_apk_backdata_get;*/
		/*apk_op->apk_debug_info = ili_apk_debug_info;*/

	} else {
		TPD_INFO("Can not kzalloc apk op.\n");
	}
}
#endif /*end of CONFIG_OPLUS_TP_APK*/

static ssize_t tp_info_read(struct file *file, char *buf,
							size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	char buf_tmp[1024] = {0};
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)
			ts->chip_data;

	if (*pos)
		return 0;

	ret = snprintf(buf_tmp, sizeof(buf_tmp),
				"[VENDOR] = TIANMA, [IC] = HX83102J, [FW_VER] = 0x%02X\n", chip_info->touch_ver);
	if (clear_user(buf, len)) {
		TPD_INFO("clear error");
		return -EIO;
	}
	if (copy_to_user(buf, buf_tmp, (len > 1024)?1024:len))
		TPD_INFO("%s,here:%d\n", __func__, __LINE__);

	*pos += ret;
	return ret;
}

#if LINUX_VERSION_CODE>= KERNEL_VERSION(5, 10, 0)

static const struct proc_ops tp_info_ops = {
	.proc_open = simple_open,
	.proc_read = tp_info_read,
};
#else
static const struct file_operations tp_info_ops = {
	.owner = THIS_MODULE,
	.read = tp_info_read,
};
#endif
/*
* Modify and fix by Himax(HX)
* Version: 2020020701
*/
static int __maybe_unused hx83102j_tp_probe(struct spi_device *spi)
{
	struct chip_data_hx83102j *chip_info = NULL;
	struct touchpanel_data *ts = NULL;
	struct proc_dir_entry *prEntry_tmp = NULL;
	int ret = -1;

	TPD_INFO("%s  is called\n", __func__);

	/*step1:Alloc chip_info*/
	chip_info = kzalloc(sizeof(struct chip_data_hx83102j), GFP_KERNEL);
	if (chip_info == NULL) {
		TPD_INFO("chip info kzalloc error\n");
		ret = -ENOMEM;
		return ret;
	}
	memset(chip_info, 0, sizeof(*chip_info));

	/* allocate himax report data */
	chip_info->hx_touch_data = kzalloc(sizeof(struct himax_report_data), GFP_KERNEL);
	if (chip_info->hx_touch_data == NULL) {
		ret = -ENOMEM;
		goto err_register_driver;
	}

	/*step2:Alloc common ts*/
	ts = common_touch_data_alloc();
	if (ts == NULL) {
		TPD_INFO("ts kzalloc error\n");
		goto err_register_driver;
	}
	memset(ts, 0, sizeof(*ts));

	chip_info->g_fw_buf = vmalloc(255 * 1024);
	if (chip_info->g_fw_buf == NULL) {
		TPD_INFO("fw buf vmalloc error\n");
		ret = -ENOMEM;
		goto err_g_fw_buf;
	}

	/*step3:binding dev for easy operate*/
	chip_info->hx_spi = spi;
	chip_info->syna_ops = &hx83102j_proc_ops;
	ts->debug_info_ops = &debug_info_proc_ops;
	ts->s_client = spi;
	chip_info->hx_irq = spi->irq;
	ts->irq = spi->irq;
	spi_set_drvdata(spi, ts);
	ts->dev = &spi->dev;
	ts->chip_data = chip_info;
	chip_info->hw_res = &ts->hw_res;
#ifdef HX_ZERO_FLASH
	mutex_init(&(chip_info->spi_lock));
	mutex_init(&(chip_info->fw_update_lock));
#endif
	chip_info->touch_direction = VERTICAL_SCREEN;
	chip_info->using_headfile = false;
	chip_info->first_download_finished = false;

	if (ts->s_client->master->flags & SPI_MASTER_HALF_DUPLEX) {
		TPD_INFO("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_spi_setup;
	}
	ts->s_client->bits_per_word = 8;
	ts->s_client->mode = SPI_MODE_3;
	ts->s_client->chip_select = 0;


#ifdef HIMAX_DBG
	TPD_INFO("%s: request firmware for limit!\n", __func__);
	ts->panel_data.test_limit_name = "Himax_limit.img";
	if (request_firmware(&ts->com_test_data.limit_fw, "Himax_limit.img", ts->dev) < 0) {
		TPD_INFO("[Error]request limit file fail!\n");
	}
#endif

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	/* new usage of MTK spi API */
	memcpy(&chip_info->hx_spi_mcc, &hx_spi_ctrdata, sizeof(struct mtk_chip_config));
	ts->s_client->controller_data = (void *)&chip_info->hx_spi_mcc;
#else
	/* old usage of MTK spi API */
	/*memcpy(&chip_info->hx_spi_mcc, &hx_spi_ctrdata, sizeof(struct mt_chip_conf));*/
	/*ts->s_client->controller_data = (void *)&chip_info->hx_spi_mcc;*/

	ret = spi_setup(ts->s_client);
	if (ret < 0) {
		TPD_INFO("Failed to perform SPI setup\n");
		goto err_spi_setup;
	}
#endif
	/*chip_info->p_spuri_fp_touch = &(ts->spuri_fp_touch);*/

	/*disable_irq_nosync(chip_info->hx_irq);*/

	/*step4:file_operations callback binding*/
	ts->ts_ops = &hx83102j_ops;
	ts->engineer_ops = &hx_engineer_test_ops;

#ifdef HX_ZERO_FLASH
	chip_info->pzf_op = NULL;
	chip_info->g_auto_update_flag = false;
	chip_info->g_poweronof = 1;
#endif
#ifdef CONFIG_OPLUS_TP_APK
	hx83102j_init_oplus_apk_op(ts);
#endif /* end of CONFIG_OPLUS_TP_APK*/

	/*step5:register common touch*/
	ret = register_common_touch_device(ts);
	if (ret < 0) {
		goto err_register_driver;
	}
	hx83102j_enable_interrupt(chip_info, false);
	if (hx83102j_ic_package_check(chip_info) == false) {
		TPD_INFO("Himax chip doesn NOT EXIST");
		ret = -1;
		goto err_register_driver;
	}
	chip_info->test_limit_name = ts->panel_data.test_limit_name;
#ifdef HX_ZERO_FLASH
	chip_info->p_firmware_headfile = &ts->panel_data.firmware_headfile;
	chip_info->g_auto_update_flag = true;
	chip_info->himax_0f_update_wq = create_singlethread_workqueue("HMX_0f_update_reuqest");
	INIT_DELAYED_WORK(&chip_info->work_0f_update, hx83102j_mcu_0f_operation);
	/*queue_delayed_work(chip_info->himax_0f_update_wq, &chip_info->work_0f_update, msecs_to_jiffies(2000));*/
#else
	hx83102j_read_FW_ver(chip_info);
	hx83102j_calculateChecksum(chip_info, false);
#endif

	hx83102j_power_on_init(chip_info);

	/*touch data init*/
	ret = hx83102j_report_data_init(chip_info, ts->max_num, ts->hw_res.tx_num, ts->hw_res.rx_num);
	if (ret) {
		goto err_register_driver;
	}

	ts->tp_suspend_order = TP_LCD_SUSPEND;
	ts->tp_resume_order = LCD_TP_RESUME;
	ts->skip_suspend_operate = true;
#ifdef HIMAX_DBG
	ts->skip_reset_in_resume = false;
#else
	ts->skip_reset_in_resume = false;
#endif


	/*step7:create hx83102j related proc files*/
	himax_create_proc(ts, chip_info->syna_ops);
	chip_info->irq_en_cnt = 1;
	TPD_INFO("%s, probe normal end\n", __func__);

	prEntry_tmp = proc_create_data("tp_info", 0666, NULL, &tp_info_ops, ts);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_INFO("%s: Couldn't create proc/tp_info proc entry, %d\n", __func__, __LINE__);
	}

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
	if (ts->boot_mode == RECOVERY_BOOT) {
#else
	if (ts->boot_mode == MSM_BOOT_MODE__RECOVERY) {
#endif
#ifndef HX_ZERO_FLASH
		hx83102j_enable_interrupt(chip_info, true);
#endif
		TPD_INFO("In Recovery mode, no-flash download fw by headfile\n");
		/*queue_delayed_work(chip_info->himax_0f_update_wq, &chip_info->work_0f_update, msecs_to_jiffies(500));*/
	}
	/*if (is_oem_unlocked()) {*/
	TPD_INFO("Replace system image for cts, download fw by headfile\n");
	queue_delayed_work(chip_info->himax_0f_update_wq, &chip_info->work_0f_update, msecs_to_jiffies(3000));
	/*}*/

	if (ts->fw_update_in_probe_with_headfile) {
#ifndef HX_ZERO_FLASH
		hx83102j_enable_interrupt(chip_info, true);
#endif
		TPD_INFO("It's fw_update_in_probe_with_headfile\n");
		/*queue_delayed_work(chip_info->himax_0f_update_wq, &chip_info->work_0f_update, msecs_to_jiffies(5000));*/
		}
#ifdef PEN_BATTERY_LOW_NOTIFIER
	chip_info->pen_battery.dev_num = 0;
	chip_info->pen_battery.pen_batlow_flag = false;
	pen_battery_low_init(&chip_info->pen_battery);
#endif
	chip_info->hx_criteria_item = 4;
#ifdef HX_ALG_OVERLAY
	chip_info->g_alg_idx_t = 0;
	chip_info->hx_pen_num = 0;
	chip_info->g_pen_num = 0xff;
#endif
	chip_info->isea006proj = false;
	chip_info->isbd12proj = false;
	chip_info->isread_csv = true;
	chip_info->hx_hw_reset_activate = 0;
	chip_info->hx_touch_info_point_cnt = 0;
	chip_info->g_lcd_vendor = 0;
	chip_info->irq_en_cnt = 0;
	chip_info->g_1kind_raw_size = 0;
	chip_info->hx83102j_nf_cfg_crc = -1;
	chip_info->in_self_test = 0;
	chip_info->hx_esd_reset_activate = false;
	chip_info->byte_length = 0;
	chip_info->hx83102j_freq_point = 0;

	return 0;
err_spi_setup:
	if (chip_info->g_fw_buf) {
		vfree(chip_info->g_fw_buf);
	}
err_g_fw_buf:
err_register_driver:


#if defined(CONFIG_DRM_MSM)
	if (&ts->fb_notif) {
	}
#elif defined(CONFIG_FB)
	if (&ts->fb_notif) {
		fb_unregister_client(&ts->fb_notif);
	}
#endif/*CONFIG_FB*/

	hx83102j_enable_interrupt(chip_info, false);

	common_touch_data_free(ts);
	ts = NULL;

	if (chip_info->hx_touch_data) {
		kfree(chip_info->hx_touch_data);
	}

	if (chip_info) {
		kfree(chip_info);
	}


	TPD_INFO("%s, probe error\n", __func__);

	return ret;
}

static int __maybe_unused hx83102j_tp_remove(struct spi_device *spi)
{
	struct touchpanel_data *ts = spi_get_drvdata(spi);

	TPD_INFO("%s is called\n", __func__);

	if (ts) {
		unregister_common_touch_device(ts);
		common_touch_data_free(ts);
	}

	return 0;
}

static void hx83102j_tp_shutdown(struct spi_device *spi)
{
	struct touchpanel_data *ts = spi_get_drvdata(spi);
#ifdef PEN_BATTERY_LOW_NOTIFIER
	struct chip_data_hx83102j *chip_info = (struct chip_data_hx83102j *)ts->chip_data;

	pen_battery_low_exit(&chip_info->pen_battery);
#endif
	TPD_INFO("%s is called\n", __func__);
	tp_shutdown(ts);
}

static int hx83102j_spi_suspend(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("%s: is called gesture_enable =%d\n", __func__, ts->gesture_enable);
	tp_pm_suspend(ts);

	return 0;
}

static int hx83102j_spi_resume(struct device *dev)
{
	struct touchpanel_data *ts = dev_get_drvdata(dev);

	TPD_INFO("%s is called\n", __func__);
	tp_pm_resume(ts);
	return 0;
}

#ifdef PEN_BATTERY_LOW_NOTIFIER
static ssize_t penbattery_debug_level(struct device *dev,
							struct device_attribute *attr,
							const char *buf, size_t count)
{
	TPD_INFO("%s enter\n", __func__);

	return count;
}

static DEVICE_ATTR(penbattery_debug_level, S_IRUGO | S_IWUSR, NULL, penbattery_debug_level);

static const struct attribute *penbattery_notifier_attr[] = {
	&dev_attr_penbattery_debug_level.attr,
	NULL,
};

static const struct attribute_group penbattery_notifier_attr_group = {
	.attrs = (struct attribute **) penbattery_notifier_attr,
};

static const struct file_operations pen_ops =
{
	.owner              = THIS_MODULE,
};
/*
static void clear_pen_batlow_flag(struct handwrite_pen_battery *pen_batlow)
{
	pen_batlow->pen_batlow_flag = false;
}
*/
static void pen_battery_low_notifier_env(struct handwrite_pen_battery *pen_batlow)
{
	char *envp[2] = {NULL, NULL};
	char pen_battery_low[20] = {0};
	static struct timespec64 ts_start, ts_end;
	static struct timespec64 ts_delta;
	int ret = 0;

	if (pen_batlow->pen_batlow_flag == true) {
		ktime_get_boottime_ts64(&ts_end);
		ts_delta = timespec64_sub(ts_end, ts_start);
		if (tp_debug != 0) {
			if (ts_delta.tv_sec < DEBUG_TIME_TO_SECONDS)
				return;
		}
		else if (ts_delta.tv_sec < TIME_24H_TO_SECONDS)
			return;
	}
	pen_batlow->pen_batlow_flag = true;
	ktime_get_boottime_ts64(&ts_start);
	strcpy(pen_battery_low, "pencil_low_battery");
	envp[0] = pen_battery_low;
	ret = kobject_uevent_env(&pen_batlow->pen_battery_low_dev->kobj, KOBJ_CHANGE, envp);
	TPD_INFO("%s envp[0]:%s ret = %d \n", __func__, envp[0], ret);
	return;
}

static int pen_battery_low_init(struct handwrite_pen_battery *pen_batlow)
{
	int ret = 0;

	TPD_INFO("%s enter\n", __func__);
	ret = alloc_chrdev_region(&pen_batlow->dev_num, 0, 1, PEN_BATTERY_LOW_NAME);
	if (ret < 0) {
		TPD_INFO("%s: failed to alloc chrdev region\n", __func__);
		return 0;
	}
	pen_batlow->penbattery_notifier_class = class_create(THIS_MODULE, PEN_BATTERY_LOW_NAME);
	if (IS_ERR(pen_batlow->penbattery_notifier_class)) {
		TPD_INFO("pen_battery_low_init: class_register fail\n");
		return 0;
	}

	cdev_init(&pen_batlow->pen_cdev, &pen_ops);
	ret = cdev_add(&pen_batlow->pen_cdev, pen_batlow->dev_num, 1);
	if (ret < 0) {
		pr_err("%s: failed to add cdev\n", __func__);
		goto out_class;
	}

	pen_batlow->pen_battery_low_dev = device_create(pen_batlow->penbattery_notifier_class, NULL, pen_batlow->dev_num, NULL, PEN_BATTERY_LOW_NAME);
	if (pen_batlow->pen_battery_low_dev) {
			return 0;
	} else {
		TPD_INFO("pen_battery_low_notifier:device_create fail\n");
		ret = -1;
		goto out_class;
		return 0;
	}

out_class:
	class_unregister(pen_batlow->penbattery_notifier_class);

	return 0;
}

static void pen_battery_low_exit(struct handwrite_pen_battery *pen_batlow)
{
	if (pen_batlow->pen_battery_low_dev) {
		class_unregister(pen_batlow->penbattery_notifier_class);
	}
}
#endif /* PEN_BATTERY_LOW_NOTIFIER */

static const struct spi_device_id tp_id[] = {
#ifdef CONFIG_TOUCHPANEL_MULTI_NOFLASH
	{ "oplus,tp_noflash", 0 },
#else
	{ TPD_DEVICE, 0 },
#endif
	{ }
};

static struct of_device_id tp_match_table[] = {
#ifdef CONFIG_TOUCHPANEL_MULTI_NOFLASH
	{ .compatible = "oplus,tp_noflash", },
#else
	{ .compatible = TPD_DEVICE, },
#endif
	{ },
};

static const struct dev_pm_ops tp_pm_ops = {
	.suspend = hx83102j_spi_suspend,
	.resume = hx83102j_spi_resume,
};


static struct spi_driver hx83102j_common_driver = {
	.probe      = hx83102j_tp_probe,
	.remove     = hx83102j_tp_remove,
	.shutdown   = hx83102j_tp_shutdown,
	.id_table   = tp_id,
	.driver = {
		.name = TPD_DEVICE,
		.owner = THIS_MODULE,
		.of_match_table = tp_match_table,
		.pm = &tp_pm_ops,
	},
};

static int __init tp_driver_init(void)
{
	int status = 0;

	TPD_INFO("%s is called\n", __func__);
#ifndef HIMAX_DBG
	if (!tp_judge_ic_match(TPD_DEVICE)) {
		return 0;
	}

	get_oem_verified_boot_state();
#endif
	status = spi_register_driver(&hx83102j_common_driver);
	if (status < 0) {
		TPD_INFO("%s, Failed to register SPI driver.\n", __func__);
		return 0;
	}

	return status;
}

/* should never be called */
static void __exit tp_driver_exit(void)
{
	spi_unregister_driver(&hx83102j_common_driver);
	return;
}

module_init(tp_driver_init);
module_exit(tp_driver_exit);

MODULE_DESCRIPTION("Touchscreen Driver");
MODULE_LICENSE("GPL");
