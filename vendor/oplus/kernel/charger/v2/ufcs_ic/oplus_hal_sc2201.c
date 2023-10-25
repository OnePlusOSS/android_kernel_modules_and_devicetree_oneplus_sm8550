// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[SC2201]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/proc_fs.h>
#include <trace/events/sched.h>
#include <linux/ktime.h>
#include <uapi/linux/sched/types.h>

#include <ufcs_class.h>
#include <oplus_chg_ic.h>
#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_mms.h>
#include <oplus_mms_gauge.h>
#include <oplus_impedance_check.h>

#include "oplus_hal_sc2201.h"

struct oplus_sc2201 {
	struct device *dev;
	struct ufcs_dev *ufcs;
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex i2c_rw_lock;

	int ufcs_en_gpio;
	int ufcs_int_gpio;
	int ufcs_int_irq;

	struct pinctrl *pinctrl;
	struct pinctrl_state *ufcs_int_default;
	struct pinctrl_state *ufcs_en_active;
	struct pinctrl_state *ufcs_en_sleep;

	atomic_t suspended;
};

static int sc2201_read_byte(struct oplus_sc2201 *chip, u16 addr, u8 *data)
{
	u8 addr_buf[2] = {addr & 0xff, addr >> 8};
	int rc = 0;

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_master_send(chip->client, addr_buf, 2);
	if (rc < 2) {
		chg_err("write 0x%04x error, rc = %d \n", addr, rc);
		rc = rc < 0 ? rc : -EIO;
	    goto error;
	}

	rc = i2c_master_recv(chip->client, data, 1);
	if (rc < 1) {
		chg_err("read 0x%04x error, rc = %d \n", addr, rc);
		rc = rc < 0 ? rc : -EIO;
		/* TODO: oplus_ufcs_protocol_track_upload_i2c_err_info(rc, addr); */
		goto error;
	}
	mutex_unlock(&chip->i2c_rw_lock);
	/* TODO: oplus_ufcs_protocol_i2c_err_clr(); */
	return 0;

error:
	mutex_unlock(&chip->i2c_rw_lock);
	/* TODO: oplus_ufcs_protocol_i2c_err_inc(); */
	return rc;
}

static int sc2201_read_data(struct oplus_sc2201 *chip, u16 addr,
			   u8 *buf, int len)
{
	u8 addr_buf[2] = {addr & 0xff, addr >> 8};
	int rc = 0;

	mutex_lock(&chip->i2c_rw_lock);
	rc = i2c_master_send(chip->client, addr_buf, 2);
	if (rc < 2) {
		chg_err("read 0x%04x error, rc=%d\n", addr, rc);
		rc = rc < 0 ? rc : -EIO;
		goto error;
	}

	rc = i2c_master_recv(chip->client, buf, len);
	if (rc < len) {
		chg_err("read 0x%04x error, rc=%d\n", addr, rc);
		rc = rc < 0 ? rc : -EIO;
		/* TODO: oplus_ufcs_protocol_track_upload_i2c_err_info(rc, addr); */
		goto error;
	}
	mutex_unlock(&chip->i2c_rw_lock);
	/* TODO: oplus_ufcs_protocol_i2c_err_clr(); */
	return 0;

error:
	mutex_unlock(&chip->i2c_rw_lock);
	/* TODO: oplus_ufcs_protocol_i2c_err_inc(); */
	return rc;
}

static int sc2201_write_byte(struct oplus_sc2201 *chip, u16 addr, u8 data)
{
	u8 buf[3] = {addr & 0xff, addr >> 8, data};
	int rc = 0;

	mutex_lock(&chip->i2c_rw_lock);
	rc  = i2c_master_send(chip->client, buf, 3);
	if (rc < 3) {
		chg_err("write 0x%04x error, rc = %d \n", addr, rc);
		/* TODO: oplus_ufcs_protocol_track_upload_i2c_err_info(rc, addr); */
		mutex_unlock(&chip->i2c_rw_lock);
		/* TODO: oplus_ufcs_protocol_i2c_err_inc(); */
		rc = rc < 0 ? rc : -EIO;
		return rc;
	}
	mutex_unlock(&chip->i2c_rw_lock);
	/* TODO: oplus_ufcs_protocol_i2c_err_clr(); */
	return 0;
}

static int sc2201_write_data(struct oplus_sc2201 *chip, u16 addr, u16 length, u8 *data)
{
	u8 *buf;
	int rc = 0;

	buf = kzalloc(length + 2, GFP_KERNEL);
	if (!buf) {
		chg_err("alloc memorry for i2c buffer error\n");
		return -ENOMEM;
	}

	buf[0] = addr & 0xff;
	buf[1] = addr >> 8;
	memcpy(&buf[2], data, length);

	mutex_lock(&chip->i2c_rw_lock);
	rc  = i2c_master_send(chip->client, buf, length + 2);
	if (rc < length + 2) {
		chg_err("write 0x%04x error, ret = %d \n", addr, rc);
		/* TODO: oplus_ufcs_protocol_track_upload_i2c_err_info(rc, addr); */
		mutex_unlock(&chip->i2c_rw_lock);
		kfree(buf);
		/* TODO: oplus_ufcs_protocol_i2c_err_inc(); */
		rc = rc < 0? rc : -EIO;
		return rc;
	}
	mutex_unlock(&chip->i2c_rw_lock);
	kfree(buf);
	/* TODO: oplus_ufcs_protocol_i2c_err_clr(); */
	return 0;
}

static int sc2201_write_bit_mask(struct oplus_sc2201 *chip, u16 addr,
				u8 mask, u8 data)
{
	u8 temp = 0;
	int rc = 0;

	rc = sc2201_read_byte(chip, addr, &temp);
	if (rc < 0)
		return rc;

	temp = (data & mask) | (temp & (~mask));

	rc = sc2201_write_byte(chip, addr, temp);
	if (rc < 0)
		return rc;

	return 0;
}

static int sc2201_ufcs_write_msg(struct ufcs_dev *ufcs, unsigned char *buf, int len)
{
	struct oplus_sc2201 *chip = ufcs->drv_data;

	if (!chip)
		return -ENODEV;

	sc2201_write_byte(chip, SC2201_ADDR_TX_LENGTH, len);
	sc2201_write_data(chip, SC2201_ADDR_TX_BUFFER0, len, buf);
	sc2201_write_bit_mask(chip, SC2201_ADDR_UFCS_CTRL0, SC2201_MASK_SND_CMP, SC2201_CMD_SND_CMP);
	return 0;
}

static int sc2201_ufcs_read_msg(struct ufcs_dev *ufcs, unsigned char *buf, int len)
{
	u8 rx_length[2] = {0};
	int tmp;
	int rc = 0;
	struct oplus_sc2201 *chip = ufcs->drv_data;

	if (!chip)
		return -ENODEV;

	rc = sc2201_read_data(chip, SC2201_ADDR_RX_LENGTH_L, rx_length, 2);
	if (rc < 0) {
		chg_err("can't read rx buf len, rc=%d\n", rc);
		return rc;
	}

	tmp = rx_length[1] << 8 | rx_length[0];
	if (tmp > len) {
		chg_err("rx_buf_len = %d, limit to %d\n", tmp, len);
		tmp = len;
	}

	rc = sc2201_read_data(chip, SC2201_ADDR_RX_BUFFER0, buf, tmp);
	if (rc < 0) {
		chg_err("can't read rx buf, rc=%d\n", rc);
		return rc;
	}

	return tmp;
}

static int sc2201_read_flags(struct oplus_sc2201 *chip)
{
	struct ufcs_error_info *reg = &chip->ufcs->err_info;
	int rc = 0;
	u8 flag_buf[SC2201_FLAG_NUM] = {0};

	if (!chip || !chip->ufcs)
		return -ENODEV;

	rc = sc2201_read_data(chip, SC2201_ADDR_GENERAL_INT_FLAG, flag_buf, SC2201_FLAG_NUM);
	if (rc < 0) {
		chg_err("failed to read flag register\n");
		return rc;
	}

	reg->hd_error.dp_ovp = flag_buf[3] & SC2201_FLAG_DP_OVP;
	reg->hd_error.dm_ovp = flag_buf[3] & SC2201_FLAG_DM_OVP;
	reg->hd_error.temp_shutdown = flag_buf[0] & SC2201_FLAG_TEMP_SHUTDOWN;
	reg->hd_error.wtd_timeout = flag_buf[0] & SC2201_FLAG_WATCHDOG_TIMEOUT;
	reg->hd_error.hardreset = flag_buf[8] & SC2201_FLAG_HARD_RESET;

	reg->rcv_error.sent_cmp = flag_buf[7] & SC2201_FLAG_SENT_PACKET_COMPLETE;
	reg->rcv_error.msg_trans_fail = flag_buf[7] & SC2201_FLAG_MSG_TRANS_FAIL;
	reg->rcv_error.ack_rcv_timeout = flag_buf[7] & SC2201_FLAG_ACK_RECEIVE_TIMEOUT;
	reg->rcv_error.data_rdy = flag_buf[7] & SC2201_FLAG_DATA_READY;

	reg->commu_error.baud_error = flag_buf[8] & SC2201_FLAG_BAUD_RATE_ERROR;
	reg->commu_error.training_error = flag_buf[8] & SC2201_FLAG_TRAINING_BYTE_ERROR;
	reg->commu_error.start_fail = flag_buf[8] & SC2201_FLAG_START_FAIL;
	reg->commu_error.byte_timeout = flag_buf[8] & SC2201_FLAG_DATA_BYTE_TIMEOUT;
	reg->commu_error.rx_len_error = flag_buf[8] & SC2201_FLAG_LENGTH_ERROR;
	reg->commu_error.rx_overflow = flag_buf[7] & SC2201_FLAG_RX_OVERFLOW;
	reg->commu_error.crc_error = flag_buf[8] & SC2201_FLAG_CRC_ERROR;
	reg->commu_error.stop_error = flag_buf[8] & SC2201_FLAG_STOP_ERROR;
	reg->commu_error.baud_change = flag_buf[9] & SC2201_FLAG_BAUD_RATE_CHANGE;
	reg->commu_error.bus_conflict = flag_buf[9] & SC2201_FLAG_BUS_CONFLICT;

	if (chip->ufcs->handshake_state == UFCS_HS_WAIT) {
		if ((flag_buf[7] & SC2201_FLAG_HANDSHAKE_SUCCESS) &&
		    !(flag_buf[7] & SC2201_FLAG_HANDSHAKE_FAIL))
			chip->ufcs->handshake_state = UFCS_HS_SUCCESS;
	}
	chg_info("[0/3/7/8] = [0x%x, 0x%x, 0x%x, 0x%x]\n", flag_buf[0], flag_buf[3], flag_buf[7], flag_buf[8]);

	return 0;
}

static int sc2201_dump_registers(struct oplus_sc2201 *chip)
{
	int rc = 0;
	u16 addr = 0x0;
	u8 val_buf[16] = {0x0};

	if (!chip)
		return -ENODEV;

	if (atomic_read(&chip->suspended) == 1) {
		chg_err("sc2201 is suspend!\n");
		return -ENODEV;
	}

	for (addr = 0x0; addr <= 0x000F; addr++) {
		rc = sc2201_read_byte(chip, addr, &val_buf[addr]);
		if (rc < 0) {
			chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
			break;
		}
	}

	chg_err(":[0~4][0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n",
		val_buf[0], val_buf[1], val_buf[2], val_buf[3], val_buf[4]);
	chg_err(":[5~9][0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n",
		val_buf[5], val_buf[6], val_buf[7], val_buf[8], val_buf[9]);
	chg_err(":[a~f][0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]\n",
		val_buf[10], val_buf[11], val_buf[12], val_buf[13], val_buf[14], val_buf[15]);

	return 0;
}

static int sc2201_ufcs_init(struct ufcs_dev *ufcs)
{
	return 0;
}

static int sc2201_ufcs_enable(struct ufcs_dev *ufcs)
{
	struct oplus_sc2201 *chip = ufcs->drv_data;
	u16 addr_buf[SC2201_ENABLE_REG_NUM] = {SC2201_ADDR_GENERAL_CTRL, SC2201_ADDR_GENERAL_CTRL, SC2201_ADDR_UFCS_CTRL0,
		SC2201_ADDR_UFCS_CTRL1, SC2201_ADDR_UFCS_INT_MASK0, SC2201_ADDR_UFCS_INT_MASK1, SC2201_ADDR_DP_VH};
	u8 cmd_buf[SC2201_ENABLE_REG_NUM] = {SC2201_CMD_REG_RST, SC2201_CMD_DPDM_EN, SC2201_CMD_EN_CHIP,
		SC2201_CMD_CLR_TX_RX, SC2201_CMD_MASK_ACK_TIMEOUT, SC2201_FLAG_MASK_TRANING_BYTE_ERROR, SC2201_MASK_DP_VH}, value = 0;
	int rc = 0;
	int i = 0;

	if (!chip)
		return -ENODEV;

	if (IS_ERR_OR_NULL(chip) || IS_ERR_OR_NULL(chip->pinctrl)
			|| IS_ERR_OR_NULL(chip->ufcs_en_active)) {
		chg_err("global pointer chip error\n");
		return -ENODEV;
	}

	if (atomic_read(&chip->suspended) == 1) {
		chg_err("sc2201 is suspend!\n");
		return -ENODEV;
	}

	gpio_direction_output(chip->ufcs_en_gpio, 1);
	rc = pinctrl_select_state(chip->pinctrl, chip->ufcs_en_active);
	if (rc < 0) {
		chg_err("can't set the en gpio high! \n");
		return rc;
	}

	msleep(10);
	chg_err("gpio_get_value(chip->ufcs_en_gpio)  gpio_value = %d !\n", gpio_get_value(chip->ufcs_en_gpio));
	for (i = 0; i < SC2201_ENABLE_REG_NUM; i++) {
		rc = sc2201_write_byte(chip, addr_buf[i], cmd_buf[i]);
		if (rc < 0) {
			chg_err("write i2c failed!\n");
			return rc;
		}
	}
	for (i = 0; i < SC2201_ENABLE_REG_NUM; i++) {
		rc = sc2201_read_byte(chip, addr_buf[i], &value);
		chg_err(" read 0x%x value = 0x%x\n", addr_buf[i], value);
	}

	return 0;
}

static int sc2201_disable(struct oplus_sc2201 *chip)
{
	int rc = 0;

	if (!chip)
		return -ENODEV;

	if (IS_ERR_OR_NULL(chip) || IS_ERR_OR_NULL(chip->pinctrl)
			|| IS_ERR_OR_NULL(chip->ufcs_en_sleep)) {
		chg_err("sleep pinctrl error \n");
		return -ENODEV;
	}
	rc = sc2201_write_byte(chip, SC2201_ADDR_GENERAL_CTRL, SC2201_CMD_DPDM_DIS);
	if (rc < 0) {
		chg_err("write GENERAL_CTRL i2c failed\n");
		return rc;
	}

	rc = sc2201_write_byte(chip, SC2201_ADDR_UFCS_CTRL0, SC2201_CMD_DIS_CHIP);
	if (rc < 0) {
		chg_err("write UFCS_CTRL0 i2c failed\n");
		return rc;
	}

	msleep(10);
	gpio_direction_output(chip->ufcs_en_gpio, 0);
	rc = pinctrl_select_state(chip->pinctrl, chip->ufcs_en_sleep);
	if (rc < 0) {
		chg_err("can't set the en gpio low! \n");
		return -ENODEV;
	}

	return 0;
}

static int sc2201_ufcs_disable(struct ufcs_dev *ufcs)
{
	int rc = 0;
	struct oplus_sc2201 *chip = ufcs->drv_data;

	if (!chip)
		return -ENODEV;

	if (IS_ERR_OR_NULL(chip) || IS_ERR_OR_NULL(chip->pinctrl)
			|| IS_ERR_OR_NULL(chip->ufcs_en_sleep)) {
		chg_err("sleep pinctrl error \n");
		return -ENODEV;
	}
	rc = sc2201_write_byte(chip, SC2201_ADDR_GENERAL_CTRL, SC2201_CMD_DPDM_DIS);
	if (rc < 0) {
		chg_err("write GENERAL_CTRL i2c failed\n");
		return rc;
	}

	rc = sc2201_write_byte(chip, SC2201_ADDR_UFCS_CTRL0, SC2201_CMD_DIS_CHIP);
	if (rc < 0) {
		chg_err("write UFCS_CTRL0 i2c failed\n");
		return rc;
	}

	msleep(10);
	gpio_direction_output(chip->ufcs_en_gpio, 0);
	rc = pinctrl_select_state(chip->pinctrl, chip->ufcs_en_sleep);
	if (rc < 0) {
		chg_err("can't set the en gpio low! \n");
		return -ENODEV;
	}

	return 0;
}

static int sc2201_ufcs_handshake(struct ufcs_dev *ufcs)
{
	int rc = 0;
	struct oplus_sc2201 *chip = ufcs->drv_data;

	if (!chip)
		return -ENODEV;

	rc = sc2201_write_bit_mask(chip, SC2201_ADDR_UFCS_CTRL0, SC2201_MASK_EN_HANDSHAKE,
	                SC2201_CMD_EN_HANDSHAKE);
	return rc;
}

static int sc2201_ufcs_set_baud_rate(struct ufcs_dev *ufcs, enum ufcs_baud_rate baud)
{
	int rc = 0;
	struct oplus_sc2201 *chip = ufcs->drv_data;

	if (!chip)
		return -ENODEV;

	rc = sc2201_write_bit_mask(chip, SC2201_ADDR_UFCS_CTRL0, SC2201_FLAG_BAUD_RATE_VALUE,
		(baud << SC2201_FLAG_BAUD_NUM_SHIFT));

	return rc;
}

static int sc2201_ufcs_source_hard_reset(struct ufcs_dev *ufcs)
{
	int rc = 0;
	struct oplus_sc2201 *chip = ufcs->drv_data;

	if (!chip)
		return -ENODEV;

	rc = sc2201_write_bit_mask(chip, SC2201_ADDR_UFCS_CTRL0, SC2201_SEND_SOURCE_HARDRESET,
	                SC2201_SEND_SOURCE_HARDRESET);

	return rc;
}

static int sc2201_ufcs_cable_hard_reset(struct ufcs_dev *ufcs)
{
	int rc = 0;
	struct oplus_sc2201 *chip = ufcs->drv_data;

	if (!chip)
		return -ENODEV;

	rc = sc2201_write_bit_mask(chip, SC2201_ADDR_UFCS_CTRL0, SC2201_SEND_CABLE_HARDRESET,
					SC2201_SEND_CABLE_HARDRESET);

	return rc;
}

static irqreturn_t sc2201_int_handler(int irq, void *dev_id)
{
	struct oplus_sc2201 *chip = dev_id;

	ufcs_clr_error_flag(chip->ufcs);
	sc2201_read_flags(chip);
	ufcs_msg_handler(chip->ufcs);
	return IRQ_HANDLED;
}

static int sc2201_gpio_init(struct oplus_sc2201 *chip)
{
	int rc = 0;
	int gpio_value = 0;
	struct device_node *node = chip->dev->of_node;

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -ENODEV;
	}

	chip->ufcs_int_gpio = of_get_named_gpio(node, "oplus,ufcs_int-gpio", 0);
	if (!gpio_is_valid(chip->ufcs_int_gpio)) {
		chg_err("ufcs_int gpio not specified\n");
		return -ENODEV;
	}
	rc = gpio_request(chip->ufcs_int_gpio, "ufcs_int-gpio");
	if (rc < 0) {
		chg_err("ufcs_int gpio request error, rc=%d\n", rc);
		return rc;
	}
	chip->ufcs_int_default = pinctrl_lookup_state(chip->pinctrl, "ufcs_int_default");
	if (IS_ERR_OR_NULL(chip->ufcs_int_default)) {
		chg_err("get ufcs_int_default fail\n");
		goto free_int_gpio;
	}
	gpio_direction_input(chip->ufcs_int_gpio);
	pinctrl_select_state(chip->pinctrl, chip->ufcs_int_default);
	chip->ufcs_int_irq = gpio_to_irq(chip->ufcs_int_gpio);
	rc = request_threaded_irq(chip->ufcs_int_irq, NULL, sc2201_int_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ufcs_int_irq", chip);
	if (rc < 0) {
		chg_err("ufcs_int_irq request error, rc=%d\n", rc);
		goto free_int_gpio;
	}
	enable_irq_wake(chip->ufcs_int_irq);

	chip->ufcs_en_gpio = of_get_named_gpio(node, "oplus,ufcs_en-gpio", 0);
	if (!gpio_is_valid(chip->ufcs_en_gpio)) {
		chg_err("ufcs_en_gpio not specified\n");
		goto disable_int_irq;
	}
	rc = gpio_request(chip->ufcs_en_gpio, "ufcs_en-gpio");
	if (rc < 0) {
		chg_err("ufcs_en_gpio request error, rc=%d\n", rc);
		goto disable_int_irq;
	}
	chip->ufcs_en_active = pinctrl_lookup_state(chip->pinctrl, "ufcs_en_active");
	if (IS_ERR_OR_NULL(chip->ufcs_en_active)) {
		chg_err("get ufcs_en_active fail\n");
		goto free_en_gpio;
	}
	chip->ufcs_en_sleep = pinctrl_lookup_state(chip->pinctrl, "ufcs_en_sleep");
	if (IS_ERR_OR_NULL(chip->ufcs_en_sleep)) {
		chg_err("get ufcs_en_sleep fail\n");
		goto free_en_gpio;
	}
	gpio_direction_output(chip->ufcs_en_gpio, 0);
	pinctrl_select_state(chip->pinctrl, chip->ufcs_en_sleep);

	gpio_value = gpio_get_value(chip->ufcs_en_gpio);
	chg_err("gpio_get_value(chip->ufcs_en_gpio)  gpio_value = %d !\n", gpio_value);

	return 0;

free_en_gpio:
	if (!gpio_is_valid(chip->ufcs_en_gpio))
		gpio_free(chip->ufcs_en_gpio);
disable_int_irq:
	disable_irq(chip->ufcs_int_irq);
free_int_gpio:
	if (!gpio_is_valid(chip->ufcs_int_gpio))
		gpio_free(chip->ufcs_int_gpio);
	return rc;
}

static int sc2201_charger_choose(struct oplus_sc2201 *chip)
{
	int rc = 0;
	u16 addr = 0x0;
	u8 val_buf = 0x0;

	rc = sc2201_read_byte(chip, addr, &val_buf);
	if (rc < 0) {
		chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		return -EPROBE_DEFER;
	} else
		return 1;
}

static int sc2201_hardware_init(struct oplus_sc2201 *chip)
{
	int rc = 0;

	gpio_direction_output(chip->ufcs_en_gpio, 1);
	pinctrl_select_state(chip->pinctrl, chip->ufcs_en_active);
	msleep(10);
	chg_err("gpio_get_value(chip->ufcs_en_gpio)  gpio_value = %d !\n", gpio_get_value(chip->ufcs_en_gpio));
	rc = sc2201_charger_choose(chip);
	if (rc <= 0) {
		gpio_direction_output(chip->ufcs_en_gpio, 0);
		pinctrl_select_state(chip->pinctrl, chip->ufcs_en_sleep);
		return rc;
	}

	sc2201_dump_registers(chip);
	gpio_direction_output(chip->ufcs_en_gpio, 0);
	pinctrl_select_state(chip->pinctrl, chip->ufcs_en_sleep);
	return rc;
}

static int sc2201_parse_dt(struct oplus_sc2201 *chip)
{
	struct device_node *node = NULL;

	if (!chip) {
		return -1;
	}
	node = chip->dev->of_node;
	return 0;
}

static void register_ufcs_devinfo(void)
{
#ifndef CONFIG_DISABLE_OPLUS_FUNCTION
	int rc = 0;
	char *version;
	char *manufacture;

	version = "sc2201";
	manufacture = "SouthChip";

	rc = register_device_proc("sc2201", version, manufacture);
	if (rc)
		chg_err("register_ufcs_devinfo fail\n");
#endif
}

static bool sc2201_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static struct regmap_config sc2201_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = SC2201_MAX_REG,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = sc2201_is_volatile_reg,
};

static struct ufcs_dev_ops sc2201_ufcs_ops = {
	.init = sc2201_ufcs_init,
	.write_msg = sc2201_ufcs_write_msg,
	.read_msg = sc2201_ufcs_read_msg,
	.handshake = sc2201_ufcs_handshake,
	.source_hard_reset = sc2201_ufcs_source_hard_reset,
	.cable_hard_reset = sc2201_ufcs_cable_hard_reset,
	.set_baud_rate = sc2201_ufcs_set_baud_rate,
	.enable = sc2201_ufcs_enable,
	.disable = sc2201_ufcs_disable,
};

static struct ufcs_config sc2201_ufcs_config = {
	.check_crc = false,
	.reply_ack = false,
	.msg_resend = false,
	.ic_vendor_id = SC2201_VENDOR_ID,
};

static int sc2201_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct oplus_sc2201 *chip_ic;

	int rc;

	chip_ic = devm_kzalloc(&client->dev, sizeof(struct oplus_sc2201), GFP_KERNEL);
	if (!chip_ic) {
		chg_err("failed to allocate oplus_sc2201\n");
		return -ENOMEM;
	}

	chip_ic->regmap = devm_regmap_init_i2c(client, &sc2201_regmap_config);
	if (!chip_ic->regmap) {
		rc = -ENODEV;
		goto regmap_init_err;
	}

	chip_ic->dev = &client->dev;
	chip_ic->client = client;
	i2c_set_clientdata(client, chip_ic);
	mutex_init(&chip_ic->i2c_rw_lock);


	rc = sc2201_gpio_init(chip_ic);
	if (rc < 0) {
		chg_err("sc2201 gpio init failed, rc = %d!\n", rc);
		goto gpio_init_err;
	}

	msleep(10);

	rc = sc2201_hardware_init(chip_ic);
	if (rc < 0) {
		chg_err("sc2201 ic init failed, rc = %d!\n", rc);
		goto gpio_init_err;
	}
	sc2201_parse_dt(chip_ic);

	chip_ic->ufcs = ufcs_device_register(chip_ic->dev, &sc2201_ufcs_ops, chip_ic, &sc2201_ufcs_config);
	if (IS_ERR_OR_NULL(chip_ic->ufcs)) {
		chg_err("ufcs device register error\n");
		rc = -ENODEV;
		goto reg_chg_err;
	}

	register_ufcs_devinfo();

	chg_info("call end!\n");

	return 0;

reg_chg_err:
gpio_init_err:
	if (!gpio_is_valid(chip_ic->ufcs_en_gpio))
		gpio_free(chip_ic->ufcs_en_gpio);
	disable_irq(chip_ic->ufcs_int_irq);
	if (!gpio_is_valid(chip_ic->ufcs_int_gpio))
		gpio_free(chip_ic->ufcs_int_gpio);
regmap_init_err:
	devm_kfree(&client->dev, chip_ic);
	return rc;
}

static int sc2201_pm_resume(struct device *dev_chip)
{
	struct i2c_client *client  = container_of(dev_chip, struct i2c_client, dev);
	struct oplus_sc2201 *chip = i2c_get_clientdata(client);

	if (chip == NULL)
		return 0;

	atomic_set(&chip->suspended, 0);

	return 0;
}

static int sc2201_pm_suspend(struct device *dev_chip)
{
	struct i2c_client *client  = container_of(dev_chip, struct i2c_client, dev);
	struct oplus_sc2201 *chip = i2c_get_clientdata(client);

	if (chip == NULL)
		return 0;

	atomic_set(&chip->suspended, 1);

	return 0;
}

static const struct dev_pm_ops sc2201_pm_ops = {
	.resume = sc2201_pm_resume,
	.suspend = sc2201_pm_suspend,
};

static void sc2201_driver_remove(struct i2c_client *client)
{
	struct oplus_sc2201 *chip = i2c_get_clientdata(client);

	if(chip == NULL)
		return;

	ufcs_device_unregister(chip->ufcs);
	if (!gpio_is_valid(chip->ufcs_en_gpio))
		gpio_free(chip->ufcs_en_gpio);
	disable_irq(chip->ufcs_int_irq);
	if (!gpio_is_valid(chip->ufcs_int_gpio))
		gpio_free(chip->ufcs_int_gpio);
	devm_kfree(&client->dev, chip);
}

static void sc2201_shutdown(struct i2c_client *chip_client)
{
	struct oplus_sc2201 *chip = i2c_get_clientdata(chip_client);

	sc2201_disable(chip);
}

static const struct of_device_id sc2201_match[] = {
	{.compatible = "oplus,sc2201-ufcs"},
	{},
};

static const struct i2c_device_id sc2201_id[] = {
	{"oplus,sc2201-ufcs", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sc2201_id);


static struct i2c_driver sc2201_i2c_driver = {
	.driver		= {
		.name = "sc2201-ufcs",
		.owner	= THIS_MODULE,
		.of_match_table = sc2201_match,
		.pm = &sc2201_pm_ops,
	},
	.probe		= sc2201_driver_probe,
	.remove		= sc2201_driver_remove,
	.id_table	= sc2201_id,
	.shutdown	= sc2201_shutdown,
};

static __init int sc2201_i2c_driver_init(void)
{
#if __and(IS_BUILTIN(CONFIG_OPLUS_CHG), IS_BUILTIN(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return 0;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return 0;
#endif /* CONFIG_OPLUS_CHG_V2 */
	return i2c_add_driver(&sc2201_i2c_driver);
}

static __exit void sc2201_i2c_driver_exit(void)
{
#if (IS_ENABLED(CONFIG_OPLUS_CHG) && IS_ENABLED(CONFIG_OPLUS_CHG_V2))
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if (node == NULL)
		return;
	if (!of_property_read_bool(node, "oplus,chg_framework_v2"))
		return;
#endif /* CONFIG_OPLUS_CHG_V2 */
	i2c_del_driver(&sc2201_i2c_driver);
}

oplus_chg_module_register(sc2201_i2c_driver);

MODULE_DESCRIPTION("SC UFCS Driver");
MODULE_LICENSE("GPL v2");
