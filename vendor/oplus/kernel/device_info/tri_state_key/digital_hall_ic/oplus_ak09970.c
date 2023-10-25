#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/init.h>
#include "oplus_ak09970.h"
#include "../oplus_tri_key.h"

#define DHALL_NUM 1
#define TRI_KEY_TAG                  "[tri_state_key] "
#define TRI_KEY_ERR(fmt, args...)\
	pr_err(TRI_KEY_TAG" %s : "fmt, __func__, ##args)
#define TRI_KEY_LOG(fmt, args...)\
	pr_err(TRI_KEY_TAG" %s : "fmt, __func__, ##args)


static struct oplus_dhall_chip *g_chip;

static struct hall_srs ak09970_ranges[] = {
	{"36_04mT", AK09970_HIGH_SENSITIVITY, false},
	{"101_57mT", AK09970_WIDE_RANGE, false},
};

static DEFINE_MUTEX(ak09970_i2c_mutex);

static int ak09970_i2c_read_block(struct oplus_dhall_chip *chip, u8 addr, u8 *data, u8 len)
{
	u8 reg_addr = addr;
	int err = 0;
	struct i2c_client *client = chip->client;
	struct i2c_msg msgs[2] = {{0}, {0}};

	if (!client) {
		TRI_KEY_LOG("client null\n");
		return -EINVAL;
	} else if (len > AK09970_I2C_REG_MAX_SIZE) {
		TRI_KEY_LOG(" length %d exceeds %d\n", len, AK09970_I2C_REG_MAX_SIZE);
		return -EINVAL;
	}
	mutex_lock(&ak09970_i2c_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg_addr;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	err = i2c_transfer(client->adapter, msgs, (sizeof(msgs) / sizeof(msgs[0])));
	/*TRI_KEY_LOG("----ak09970_i2c_read_block: (0x%02X %p %d)\n",addr, data, len);
	dump_stack();*/

	if (err < 0) {
		TRI_KEY_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	mutex_unlock(&ak09970_i2c_mutex);

	return err;
}

static int ak09970_i2c_write_block(struct oplus_dhall_chip *chip, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	int idx = 0;
	int num = 0;
	char buf[AK09970_I2C_REG_MAX_SIZE] = {0};
	struct i2c_client *client = chip->client;

	if (!client) {
		TRI_KEY_LOG("client null\n");
		return -EINVAL;
	} else if (len >= AK09970_I2C_REG_MAX_SIZE) {
		TRI_KEY_LOG(" length %d exceeds %d\n", len, AK09970_I2C_REG_MAX_SIZE);
		return -EINVAL;
	}

	mutex_lock(&ak09970_i2c_mutex);

	buf[num++] = addr;
	for (idx = 0; idx < len; idx++) {
		buf[num++] = data[idx];
	}

	/*TRI_KEY_LOG("----ak09970_i2c_write_block: (0x%02X %p %d)\n",addr, data, len);
	/dump_stack();*/
	err = i2c_master_send(client, buf, num);

	if (err < 0) {
		TRI_KEY_LOG("send command error!! %d\n", err);
	}

	mutex_unlock(&ak09970_i2c_mutex);
	return err;
}

static int mysqrt(long x)
{
	long temp = 0;
	int res = 0;
	int count = 0;
	if (x == 1 || x == 0)
		return x;
	temp = x / 2;
	while (1) {
		long a = temp;
		count++;
		temp = (temp + x / temp) / 2;
		if (count > MAX_SQRT_TIME) {
			res = temp;
			TRI_KEY_LOG("count = %d, over the max time\n", count);
			return res;
		}
		if (((a - temp) < 2) && ((a - temp) > -2)) {
			res = temp;
			return res;
		}
	}
}

static int ak09970_get_data(struct dhall_data_xyz *data)
{
	int err = 0;
	int irqval = 0;
	u8 buf[7] = {0};
	short value_x = 0;
	short value_y = 0;
	short value_z = 0;
	long value_v = 0;
	u8 st = 0;
	TRI_KEY_LOG("%s:called", __func__);
	if (g_chip== NULL) {
		TRI_KEY_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	msleep(35);

	/* (1) read data */
	err = ak09970_i2c_read_block(g_chip, AK09970_REG_ST1_ZYX/*0x17*/, buf, sizeof(buf));
	if (err < 0) {
		TRI_KEY_LOG(" fail %d \n", err);
		return err;
	}

	/* (2) collect data*/
	st = buf[0];
	if (buf[0] & 0x01) {
		value_x = (short)((u16)(buf[5] << 8) + buf[6]);
		value_y = (short)((u16)(buf[3] << 8) + buf[4]);
		value_z = (short)((u16)(buf[1] << 8) + buf[2]);
	} else {
		TRI_KEY_LOG("ak09970 hall: st1(0x%02X%02X) is not DRDY.\n",  buf[0], buf[1]);
		data->hall_x = value_x;
		data->hall_y = value_y;
		data->hall_z = value_z;
		data->st = st;
		return err;
	}
	err = ak09970_i2c_read_block(g_chip, AK09970_REG_ST1_V, buf, sizeof(buf));
	if (err < 0) {
		TRI_KEY_LOG(" ak09970_i2c_read_block AK09970_REG_ST1_V fail %d \n", err);
		return err;
	}
	if (buf[0] & 0x01) {
		value_v = (long)(value_x * value_x) +(long)(value_y * value_y) + (long)(value_z * value_z);
		TRI_KEY_LOG("v data %d is been changed\n", value_v);
	} else {
		value_v = (long)((u32)(buf[1] << 24) +(u32)(buf[2] << 16) + (u32)(buf[3] << 8) + (u32)buf[4]);
		TRI_KEY_LOG("new hall value  v is %d\n", value_v);
	}
	data->hall_x = value_x;
	data->hall_y = value_y;
	data->hall_z = value_z;
	data->hall_v = mysqrt(value_v);
	irqval = gpio_get_value(g_chip->irq_gpio);
	TRI_KEY_LOG("new hall value is x %d, y %d, z %d v %d \n",
			   value_x, value_y, value_z, data->hall_v);
	TRI_KEY_LOG("%s  read irq is %d\n" , __func__, irqval);
	return 0;
}

static void ak09970_dump_reg(u8 *buf)
{
	if (g_chip == NULL) {
		TRI_KEY_LOG("g_chip NULL \n");
		return;
	}

	sprintf(buf, "Not support\n");
	TRI_KEY_LOG("%s \n", buf);
}

static int ak09970_set_reg(int reg, int val)
{
	u8 data = (u8)val;

	if (g_chip == NULL) {
		TRI_KEY_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	ak09970_i2c_write_block(g_chip, (u8)reg, &data, 1);

	return 0;
}

static bool ak09970_is_power_on(void)
{
	if (g_chip == NULL) {
		TRI_KEY_LOG("g_chip NULL \n");
		return false;
	}

	return g_chip->is_power_on;
}


static void ak09970_gpio_set_value(int gpio, int value)
{
	if (gpio_is_valid(gpio)) {
		gpio_set_value(gpio, value);
	}
}


/* vdd / vid power control */
static int ak09970_set_power(struct oplus_dhall_chip *chip, bool on)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->power_2v8)) {
		TRI_KEY_LOG("vdd_2v8 invalid\n");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(chip->power_1v8)) {
		TRI_KEY_LOG("vdd1v8 invalid\n");
		return -EINVAL;
	}

	if (on) {
		if (regulator_count_voltages(chip->power_2v8) > 0) {
			ret = regulator_set_voltage(chip->power_2v8, 2856000, 3104000);
			if (ret) {
				TRI_KEY_LOG("Regulator set_vtg failed vdd ret=%d\n", ret);
				return ret;
			}

			ret = regulator_set_load(chip->power_2v8, CURRENT_LOAD_UA);
			if (ret) {
				TRI_KEY_LOG("Regulator set_vtg failed vdd ret=%d\n", ret);
				return ret;
			}
		}
		if (regulator_count_voltages(chip->power_1v8) > 0) {
			ret = regulator_set_voltage(chip->power_1v8, 1800000, 1800000);
			if (ret) {
				TRI_KEY_LOG("Regulator set_vtg failed vcc_i2c ret=%d\n", ret);
				return ret;
			}
		}
		/*enable the 2v8 power*/
		ret = regulator_enable(chip->power_2v8);
		if (ret) {
			TRI_KEY_LOG("Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		/*should enable the 1v8 power*/
		msleep(5);

		ret = regulator_enable(chip->power_1v8);
		if (ret) {
			TRI_KEY_LOG("Regulator vcc_i2c enable failed ret=%d\n", ret);
			regulator_disable(chip->power_2v8);
			return ret;
		}

		chip->is_power_on = true;

	} else {
		ret = regulator_disable(chip->power_1v8);
		if (ret) {
			TRI_KEY_LOG("Regulator vcc_i2c disable failed ret=%d\n", ret);
			ret = regulator_enable(chip->power_2v8);
			return ret;
		}

		msleep(1);
		ret = regulator_disable(chip->power_2v8);
		if (ret) {
			TRI_KEY_LOG("Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}
	}

	return 0;
}

static void ak09970_inttobuff(u8* th, int low, int high)
{
	th[0] = (u8)(((u16)high) >> 8);
	th[1] = (u8)(((u16)high) & 0xFF);
	th[2] = (u8)(((u16)low) >> 8);
	th[3] = (u8)(((u16)low) & 0xFF);
	TRI_KEY_LOG("%s call ,th buf is 0x%02X%02X%02X%02X\n" , __func__, th[0], th[1], th[2], th[3]);
	return;
}


static bool ak09970_update_threshold(int position, short lowthd, short highthd, struct dhall_data_xyz *halldata, int interf)
{
	u8 th[4] = {0};
	u8 sth[4] = {0};
	u8 vth[4] = {0};
	int err = 0;
	u8 data[2] = {0};
	int irqval = 0;
	int second_low = 0;
	int second_high = 0;
	int vlow = 0;
	int vhigh = 0;
	if (g_chip == NULL) {
		TRI_KEY_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	if (position >= 6) {
		return false;
	}
	irqval = gpio_get_value(g_chip->irq_gpio);
	TRI_KEY_LOG("%s  read irq1 is %d\n" , __func__, irqval);
	err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1, th, 4);
	if (err < 0) {
		TRI_KEY_LOG("%s: clear AK09970_REG_SWX1 fail %d \n", __func__, err);
		return err;
	}
	err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1+1, sth, 4);
	if (err < 0) {
		TRI_KEY_LOG("%s: clear AK09970_REG_SWY1 fail %d \n", __func__, err);
		return err;
	}
	err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1+3, vth, 4);
	if (err < 0) {
		TRI_KEY_LOG("%s: clear AK09970_REG_SWX1 fail %d \n", __func__, err);
		return err;
	}
	switch (position) {
	case UP_STATE:
		lowthd = halldata->hall_x +YBRP_TOL;
	    highthd = halldata->hall_x +YBOP_TOL;
		break;
	case DOWN_STATE:
		lowthd = halldata->hall_x -YBOP_TOL;
		highthd = halldata->hall_x -YBRP_TOL;
		break;
	case MID_STATE:
		break;
	default:
		break;
	}
	ak09970_inttobuff(th, lowthd, highthd);
	TRI_KEY_LOG("lowthd = %d, highthd=%d.\n", lowthd, highthd);
	switch (position) {
	case UP_STATE:
		data[0] = 0x02;
		data[1] = 0x16;
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_CNTL1, data, 2);
		if (err < 0) {
			TRI_KEY_LOG("UP_STATE interupt fail %d \n", err);
			return err;
		}
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1, th, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: clear AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		second_low = halldata->hall_y - 2701;
		second_high = halldata->hall_y - 2700;
		ak09970_inttobuff(sth, second_low, second_high);
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1+1, sth, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: write AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		vlow = halldata->hall_v + XBRP_TOL;
		vhigh = halldata->hall_v + XBOP_TOL;
		ak09970_inttobuff(vth, vlow, vhigh);
		TRI_KEY_LOG("UP_STATE xlow=%d,xhigh=%d, ylow=%d,yhigh=%d, vlow=%d, vhigh = %d\n", lowthd, highthd, second_low, second_high, vlow, vhigh);
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1+3, vth, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: write AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		err = ak09970_i2c_read_block(g_chip, AK09970_REG_CNTL1, data, 2);
		TRI_KEY_LOG("%s UP_STATE AK09970_REG_CNTL1 0x20 data is 0x%02X%02X \n", __func__, data[0], data[1]);
		break;
	case DOWN_STATE:
		data[0] = 0x03;
		data[1] = 0x16;
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_CNTL1, data, 2);
		if (err < 0) {
			TRI_KEY_LOG("%s: write AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1, th, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: write AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		second_low = halldata->hall_y - YBOP_TOL;
		second_high = halldata->hall_y - YBRP_TOL;
		ak09970_inttobuff(sth, second_low, second_high);
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1+1, sth, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: write AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		vlow = halldata->hall_v + XBRP_TOL;
		vhigh = halldata->hall_v + XBOP_TOL;
		ak09970_inttobuff(vth, vlow, vhigh);
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1+3, vth, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: write AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		err = ak09970_i2c_read_block(g_chip, AK09970_REG_SWX1, th, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: write AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		TRI_KEY_LOG("DOWN_STATE xlow=%d,xhigh=%d, ylow=%d,yhigh=%d, vlow=%d, vhigh = %d\n", lowthd, highthd, second_low, second_high, vlow, vhigh);
		err = ak09970_i2c_read_block(g_chip, AK09970_REG_CNTL1, data, 2);
		if (err < 0) {
			TRI_KEY_LOG("%s: read AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		TRI_KEY_LOG("%s DOWN_STATE AK09970_REG_CNTL1 0x20 data is 0x%02X%02X \n", __func__, data[0], data[1]);
		err = ak09970_i2c_read_block(g_chip, AK09970_REG_SWX1+3, vth, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: read AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		break;
	case MID_STATE:
		if((halldata->hall_x < 0)&& (halldata->hall_y < 0)) {
			data[0] = 0x00;
			second_low = halldata->hall_x + XBRP_TOL;
			second_high = halldata->hall_x + XBOP_TOL;
			lowthd = halldata->hall_y +YBRP_TOL;
			highthd = halldata->hall_y +YBOP_TOL;
		} else if ((halldata->hall_x < 0) && (halldata->hall_y >= 0)) {
			data[0] = 0x02;
			second_low = halldata->hall_x + XBRP_TOL;
			second_high = halldata->hall_x + XBOP_TOL;
			lowthd = halldata->hall_y - YBOP_TOL;
			highthd = halldata->hall_y - YBRP_TOL;
		} else if ((halldata->hall_x >= 0) && (halldata->hall_y < 0)) {
			data[0] = 0x01;
			second_low = halldata->hall_x - YBOP_TOL;
			second_high = halldata->hall_x - YBRP_TOL;
			lowthd = halldata->hall_y + YBRP_TOL;
			highthd = halldata->hall_y + YBOP_TOL;
		} else {
			data[0] = 0x03;
			second_low = halldata->hall_x - YBOP_TOL;
			second_high = halldata->hall_x - YBRP_TOL;
			lowthd = halldata->hall_y - YBOP_TOL;
			highthd = halldata->hall_y - YBRP_TOL;
		}
		data[1] = 0x16;
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_CNTL1, data, 2);
		if (err < 0) {
			TRI_KEY_LOG("MID_STATE interupt fail %d \n", err);
			return err;
		}
		ak09970_inttobuff(th, lowthd, highthd);
		err = ak09970_i2c_read_block(g_chip, AK09970_REG_CNTL1, data, 2);
		if (err < 0) {
			TRI_KEY_LOG("%s: read AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1 + 1, th, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: clear AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		ak09970_inttobuff(sth, second_low, second_high);
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1, sth, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: clear AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		vlow = halldata->hall_v + XBRP_TOL;
		vhigh = halldata->hall_v + XBOP_TOL;
		ak09970_inttobuff(vth, vlow, vhigh);
		TRI_KEY_LOG("DOWN_STATE xlow=%d,xhigh=%d, ylow=%d,yhigh=%d, vlow=%d, vhigh = %d\n", lowthd, highthd, second_low, second_high, vlow, vhigh);
		err = ak09970_i2c_write_block(g_chip, AK09970_REG_SWX1+3, vth, 4);
		if (err < 0) {
			TRI_KEY_LOG("%s: clear AK09970_REG_SWX1 fail %d \n", __func__, err);
			return err;
		}
		break;
	default:
		break;
	}
	irqval = gpio_get_value(g_chip->irq_gpio);
	TRI_KEY_LOG("%s  read irq3 is %d\n" , __func__, irqval);
	if (err < 0) {
		TRI_KEY_LOG("fail %d\n", err);
		return false;
	} else {
		return true;
	}
}

/* functions for interrupt handler */
static irqreturn_t ak09970_irq_handler(int irq, void *dev_id)
{
	TRI_KEY_LOG("call \n");

	if (!g_chip) {
		TRI_KEY_LOG("g_chip NULL \n");
		return -EINVAL;
	}
	msleep(80);
	threeaxis_hall_irq_handler(DHALL_0);

	return IRQ_HANDLED;
}




int ak09970_setup_eint(struct oplus_dhall_chip *chip)
{
	int ret = 0;

	if (gpio_is_valid(chip->irq_gpio)) {
		if (chip->id < DHALL_NUM) {
			ret = gpio_request(chip->irq_gpio, "ak09970-irq");
			if (ret) {
				TRI_KEY_LOG("unable to request gpio [%d]\n", chip->irq_gpio);
				return -EINVAL;
			}

			ret = gpio_direction_input(chip->irq_gpio);

			msleep(50);

			chip->irq = gpio_to_irq(chip->irq_gpio);
		} else {
			TRI_KEY_LOG("hall id is invalid %d.\n", chip->id);
		}
	} else {
		chip->irq = -EINVAL;
		TRI_KEY_LOG("irq_gpio is invalid\n");
	}

	TRI_KEY_LOG("GPIO %d irq:%d \n", chip->irq_gpio, chip->irq);

	return 0;
}


static int ak09970_set_detection_mode(u8 mode)
{
	u8 data[2] = {0};
	int err = 0;
	u8 sdata[4] = {0};
	TRI_KEY_LOG("%s call.\n", __func__);
	if (g_chip== NULL) {
		TRI_KEY_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	if (mode == DETECTION_MODE_INTERRUPT) { /*interrupt mode*/
			TRI_KEY_LOG("%s DETECTION_MODE_INTERRUPT.\n", __func__);
			err = ak09970_i2c_read_block(g_chip, AK09970_REG_SWX1, sdata, 4);
			if (err < 0) {
				TRI_KEY_LOG("config interupt fail %d \n", err);
				return err;
			}
			TRI_KEY_LOG("%s  AK09970_REG_SWX1 0x22 data is 0x%02X%02X%02X%02X \n", __func__, sdata[0], sdata[1], sdata[2], sdata[3]);
			/* requst irq */
			if (0 == g_chip->irq_enabled) {
				if (g_chip->irq > 0) {
					TRI_KEY_LOG("%s g_chip->irq > 0.\n", __func__);
					err = request_threaded_irq(g_chip->irq, NULL,&ak09970_irq_handler,IRQ_TYPE_LEVEL_LOW | IRQF_ONESHOT,\
									"ak09970_0", (void *)g_chip->client);
					if (err < 0) {
						TRI_KEY_LOG("IRQ LINE NOT AVAILABLE!!\n");
						return -EINVAL;
					}
				} else {
					TRI_KEY_LOG("IRQ LINE NOT AVAILABLE!!");
					return -EINVAL;
				}
				irq_set_irq_wake(g_chip->irq, 1);

				g_chip->irq_enabled = 1;
				TRI_KEY_LOG("%s g_chip->irq_enabled = 1.\n", __func__);
			}
	} else {
			TRI_KEY_LOG("%s not DETECTION_MODE_INTERRUPT.\n", __func__);
			err = ak09970_i2c_write_block(g_chip, AK09970_REG_CNTL1, data, 2);
			if (err < 0) {
				TRI_KEY_LOG("config interupt fail %d \n", err);
				return err;
			}
			/*irq_set_irq_wake(g_chip->irq, 0);
			disable_irq(g_chip->irq);
			free_irq(g_chip->irq, (void *)g_chip->client);
			g_chip->irq_enabled = 0;*/
	}

	err = ak09970_i2c_read_block(g_chip, AK09970_REG_CNTL1, data, 2);
	TRI_KEY_LOG("%s AK09970_REG_CNTL1 0x20 data is 0x%02X%02X \n", __func__, data[0], data[1]);
	return 0;
}

static int ak09970_enable_irq(bool enable)
{
	if (g_chip== NULL) {
		TRI_KEY_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	if (enable) {
		enable_irq(g_chip->irq);
	} else {
		disable_irq_nosync(g_chip->irq);
	}
	return 0;
}

static int ak09970_clear_irq(void)
{
	if (g_chip== NULL) {
		TRI_KEY_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	/*ak09970_clear_interrupt(g_chip);*/

	return 0;
}

static int ak09970_get_irq_state(void)
{
	if (g_chip== NULL) {
		TRI_KEY_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	return g_chip->irq_enabled;
}

static void ak09970_set_sensitivity(char *value)
{
	int i = 0;
	struct hall_srs *srs = NULL;

	TRI_KEY_LOG("%s\n", __func__);

	for (i = 0; i < sizeof(ak09970_ranges) / sizeof(struct hall_srs); i++) {
		srs = &ak09970_ranges[i];
		if (!strncmp(srs->name, value, strlen(srs->name))) {
			break;
		} else {
			srs = NULL;
		}
	}

	if (!srs) {
		TRI_KEY_LOG("%s not match\n", value);
		return;
	}
}

static int ak09970_reset_device(struct oplus_dhall_chip *chip)
{
	int err = 0;
	u8 data = 0;
	u8 rdata = 0;
	TRI_KEY_LOG("%s call \n", __func__);
	data = AK09970_VAL_SRST_RESET;
	err = ak09970_i2c_write_block(chip, AK09970_REG_SRST, &data, 1);
	if (err < 0) {
		TRI_KEY_LOG("sw-reset failed(%d)", err);
		return err;
	}
	msleep(5); /* wait 5ms*/
	rdata = 0x0E;
	TRI_KEY_LOG("%s  sensor mode is %d\n" , __func__, rdata);
	err = ak09970_i2c_write_block(chip, AK09970_REG_CNTL2, &rdata, 1);
	if (err < 0) {
		TRI_KEY_LOG("AK09970_REG_CNTL2 failed(%d)", err);
		return err;
	}

	if (err < 0) {
		TRI_KEY_LOG("ak09970 failed to set hidden\n");
	}

	return err;
}

static void ak09970_parse_dts(struct oplus_dhall_chip *chip)
{
	struct device_node *np = NULL;
	int rc = 0;
	/*uint32_t data_range;*/
	uint32_t value;

	np = chip->client->dev.of_node;
	chip->irq_gpio = of_get_named_gpio(np, "dhall,irq-gpio", 0);

	chip->reset_gpio = of_get_named_gpio(np, "dhall,reset-gpio", 0);
	TRI_KEY_LOG("reset gpio is %d.\n", chip->reset_gpio);
	if (!gpio_is_valid(chip->reset_gpio)) {
		TRI_KEY_LOG("AK09970 pdn pin(%u) is valid\n", chip->reset_gpio);
		chip->reset_gpio = -1;
	} else {
		rc = gpio_request(chip->reset_gpio, "dhall,reset-gpio");
		TRI_KEY_LOG("gpio_request reset = %d\n", rc);
		gpio_direction_output(chip->reset_gpio, 0);
	}

	chip->power_2v8 = regulator_get(&chip->client->dev, "vdd_2v8");
	if (IS_ERR_OR_NULL(chip->power_2v8)) {
		TRI_KEY_LOG("Regulator get failed vdd_2v8\n");
	}

	chip->power_1v8 = regulator_get(&chip->client->dev, "vcc_1v8");
	if (IS_ERR_OR_NULL(chip->power_1v8)) {
		TRI_KEY_LOG("Regulator get failed vcc_1v8\n");
	}

	chip->pctrl = devm_pinctrl_get(&chip->client->dev);
	if (IS_ERR_OR_NULL(chip->pctrl)) {
		TRI_KEY_LOG("failed to get pinctrl\n");
		return;
	}

	chip->irq_state = pinctrl_lookup_state(chip->pctrl, "hall_interrupt_input");
	if (IS_ERR_OR_NULL(chip->irq_state)) {
		rc = PTR_ERR(chip->irq_state);
		TRI_KEY_LOG("pinctrl_lookup_state, err:%d\n", rc);
	} else {
		pinctrl_select_state(chip->pctrl, chip->irq_state);
	}

	chip->enable_hidden = of_property_read_bool(np, "hall,bias_support");
	if (chip->enable_hidden) {
		rc = of_property_read_u32(np, "hall,bias-ratio", &value);
		if (rc) {
			chip->bias_ratio = 100;
		} else {
			chip->bias_ratio = value;
		}
	}
}


struct dhall_operations  ak09970_ops = {
	.enable_irq = ak09970_enable_irq,
	.clear_irq = ak09970_clear_irq,
	.get_irq_state = ak09970_get_irq_state,
	.set_detection_mode = ak09970_set_detection_mode,
	.update_threeaxis_threshold = ak09970_update_threshold,
	.dump_regs = ak09970_dump_reg,
	.set_reg = ak09970_set_reg,
	.is_power_on = ak09970_is_power_on,
	.set_sensitivity = ak09970_set_sensitivity,
	.get_threeaxis_data  = ak09970_get_data,
};

static int ak09970_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct oplus_dhall_chip *chip = NULL;
	struct extcon_dev_data	*hall_dev = NULL;
	int err = 0;

	TRI_KEY_LOG("call \n");

	chip = devm_kzalloc(&client->dev, sizeof(struct oplus_dhall_chip), GFP_KERNEL);
	if (!chip) {
		TRI_KEY_LOG("kernel memory alocation was failed \n");
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TRI_KEY_LOG("i2c unsupported\n");
		return -EOPNOTSUPP;
	}

	hall_dev = kzalloc(sizeof(struct extcon_dev_data), GFP_KERNEL);
	if (!hall_dev) {
		TRI_KEY_ERR("kernel memory alocation was failed\n");
		return -ENOMEM;
	}

	chip->client = client;
	hall_dev->client = client;
	i2c_set_clientdata(client, hall_dev);
	hall_dev->dev = &client->dev;
	ak09970_parse_dts(chip);

	ak09970_set_power(chip, 1);
	ak09970_gpio_set_value(chip->reset_gpio, 0);
	mdelay(1);
	ak09970_gpio_set_value(chip->reset_gpio, 1);
	mdelay(1);
	err = ak09970_reset_device(chip);
	if (err < 0) {
		TRI_KEY_LOG("ak09970_reset_device fail \n");
		goto fail;
	}

	err = ak09970_setup_eint(chip);
	if (err < 0) {
		TRI_KEY_LOG("ak09970_setup_eint failed, %d\n", chip->id);
	}

	g_chip = chip;
	oplus_register_hall("three_axis_hall", &ak09970_ops, hall_dev);

	TRI_KEY_LOG("success. \n");
	return 0;
fail:
	TRI_KEY_LOG("fail. \n");
	devm_kfree(&client->dev, chip);
	return -ENXIO;
}

static int ak09970_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int ak09970_i2c_suspend(struct device *dev)
{
	return 0;
}

static int ak09970_i2c_resume(struct device *dev)
{
	return 0;
}

static const struct of_device_id ak09970_match[] = {
	{ .compatible = "oplus,dhall-ak09970"},
	{},
};

static const struct i2c_device_id ak09970_id[] = {
	{"dhall-ak09970", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ak09970_id);

static const struct dev_pm_ops ak09970_pm_ops = {
	.suspend = ak09970_i2c_suspend,
	.resume = ak09970_i2c_resume,
};

static struct i2c_driver ak09970_i2c_driver = {
	.driver = {
		.name	= "dhall-ak09970",
		.of_match_table =  ak09970_match,
		.pm = &ak09970_pm_ops,
	},
	.probe		= ak09970_i2c_probe,
	.remove		= ak09970_i2c_remove,
	.id_table	= ak09970_id,
};

static int __init ak09970_init(void)
{
	TRI_KEY_LOG("call\n");
	i2c_add_driver(&ak09970_i2c_driver);
	return 0;
}

static void __exit ak09970_exit(void)
{
	TRI_KEY_LOG("call\n");
}

module_init(ak09970_init);
module_exit(ak09970_exit);

MODULE_SOFTDEP("pre: pwm-qti-lpg");
MODULE_DESCRIPTION("AK09970 hallswitch driver");
MODULE_LICENSE("GPL");

