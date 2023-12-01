#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_mux.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/version.h>

#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
#include "mux_switch.h"
#endif

#include "tcpm.h"
#include "tcpci.h"

#include "oplus_typec_switch.h"

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
/*2022/12/09, Add for typec_switch err feedback.*/
#include <soc/oplus/system/oplus_mm_kevent_fb.h>
#endif

/* #define DEBUG */

#define VNAME(name) (#name)

#define CHARGER_TYPE_TIMEOUT (500)  // ms

#define USB_TYPEC_NORMAL  (0)
#define USB_TYPEC_REVERSE (1)

static struct regulator *vio28_reg = NULL;
static int err_status = 0;

struct typec_switch_priv {
	struct regmap *regmap;
	struct device *dev;
	struct tcpc_device *tcpc_dev;
	struct notifier_block pd_nb;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct blocking_notifier_head typec_switch_notifier;
	struct mutex notification_lock;
	unsigned int hs_det_pin;
	//#ifdef OPLUS_ARCH_EXTENDS
	/*2021/01/04, add for fsa4480 hs_det_level*/
	int hs_det_level;
	//#endif /*OPLUS_ARCH_EXTENDS*/
	enum typec_switch_vendor vendor;
	bool plug_state;
	bool b_dynamic_sense_to_ground;
	/* add start for DP */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	struct typec_mux_dev *mux;
#else
	struct typec_mux *mux;
#endif
	/* add end for DP */
	//2021/12/02, check the headset pluging state when probe, fix headset detect bug
	struct delayed_work hp_work;

	int charger_plugged;
	struct completion resume_ack;
};

#define RESUME_TIMEDOUT_MS	1000
static int typec_switch_wait_resume(struct typec_switch_priv *switch_priv)
{
	int rc;

	rc = wait_for_completion_timeout(&switch_priv->resume_ack, msecs_to_jiffies(RESUME_TIMEDOUT_MS));
	if (rc < 0) {
		if (rc == -ETIMEDOUT)
			pr_err("wait resume timedout\n");
		else
			pr_err("Unknown completion err, rc=%d\n", rc);
	}
	return rc;
}

struct typec_switch_reg_val {
	u8 reg;
	u8 val;
};

static const struct regmap_config typec_switch_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = DEFAULT_REG_MASK,
};

static const struct typec_switch_reg_val dio4483_i2c_reg[] = {
	{DIO4483_REG_SWITCH_SELECT, 0x18},
	{DIO4483_REG_SLOW_L, 0x00},
	{DIO4483_REG_SLOW_R, 0x00},
	{DIO4483_REG_SLOW_MIC, 0x00},
	{DIO4483_REG_SLOW_SENSE, 0x00},
	{DIO4483_REG_SLOW_GND, 0x00},
	{DIO4483_REG_DELAY_L_R, 0x00},
	{DIO4483_REG_DELAY_L_MIC, 0x00},
	{DIO4483_REG_DELAY_L_SENSE, 0x00},
	{DIO4483_REG_DELAY_L_AGND, 0x09},
	{DIO4483_REG_SWITCH_SETTINGS, 0x98},
	{DIO4483_REG_FUN_EN, 0x00},
};

static const struct typec_switch_reg_val default_i2c_reg[] = {
	{DEFAULT_REG_SWITCH_SELECT, 0x18},
	{DEFAULT_REG_SLOW_L, 0x00},
	{DEFAULT_REG_SLOW_R, 0x00},
	{DEFAULT_REG_SLOW_MIC, 0x00},
	{DEFAULT_REG_SLOW_SENSE, 0x00},
	{DEFAULT_REG_SLOW_GND, 0x00},
	{DEFAULT_REG_DELAY_L_R, 0x00},
	{DEFAULT_REG_DELAY_L_MIC, 0x00},
	{DEFAULT_REG_DELAY_L_SENSE, 0x00},
	{DEFAULT_REG_DELAY_L_AGND, 0x09},
	{DEFAULT_REG_SWITCH_SETTINGS, 0x98},
	{DEFAULT_REG_FUN_EN, 0x08},
};

//2020/11/04, support mic and ground switch to fix headset detect bug
struct typec_switch_priv *g_typec_switch_priv = NULL;
static int typec_switch_status = 0;

#define I2C_RETRIES 50
#define I2C_RETRY_DELAY 5
int typec_switch_write_register(struct regmap *regmap, unsigned int addr, unsigned int reg_val) {
	int ret = 0;
	int retries = I2C_RETRIES;

	do {
		ret = regmap_write(regmap, addr, reg_val);
		if (ret < 0) {
			if (retries == I2C_RETRIES) {
				pr_err("%s, %d, regmap write 0x%x failed, err=%d!\n", __func__, __LINE__, addr, ret);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
				mm_fb_audio_fatal_delay(10047, MM_FB_KEY_RATELIMIT_5MIN, \
							FEEDBACK_DELAY_60S, "payload@@regulator first time regmap write failed, addr=0x%x, ret=0x%x",
							addr, ret);
#endif /*CONFIG_OPLUS_FEATURE_MM_FEEDBACK*/
			}
			retries--;
			if (retries == 0) {
				pr_err("%s, %d, regmap write 0x%x failed, after retry %d times, err=%d!\n", __func__, __LINE__, addr, I2C_RETRIES, ret);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
				mm_fb_audio_fatal_delay(10047, MM_FB_KEY_RATELIMIT_5MIN,
							FEEDBACK_DELAY_60S, "payload@@regulator regmap write failed after retry %d times, addr=0x%x, ret=0x%x",
							addr, I2C_RETRIES, ret);
#endif /*CONFIG_OPLUS_FEATURE_MM_FEEDBACK*/
			} else {
				msleep(I2C_RETRY_DELAY);
			}
		}
	} while (retries > 0 && ret < 0);

	return ret;
}

#ifdef DEBUG
/*2021/04/12, add for type-c switch debug*/
static unsigned int debug_reg[32];
static void typec_switch_dump_reg(void)
{
	struct typec_switch_priv *switch_priv = g_typec_switch_priv;
	int i = 0;

	dev_info(switch_priv->dev,"%s, %d dump reg >>>\n",__func__, __LINE__);

	for (i = 0 ;i <= 0x1f ;i++) {
		regmap_read(switch_priv->regmap, i, &debug_reg[i]);
	}

	dev_info(switch_priv->dev,"reg:0x00:0x%02x,0x01:0x%02x,0x02:0x%02x,0x03:0x%02x\n", debug_reg[0],debug_reg[1],debug_reg[2],debug_reg[3]);
	dev_info(switch_priv->dev,"reg:0x04:0x%02x,0x05:0x%02x,0x06:0x%02x,0x07:0x%02x\n", debug_reg[4],debug_reg[5],debug_reg[6],debug_reg[7]);
	dev_info(switch_priv->dev,"reg:0x08:0x%02x,0x09:0x%02x,0x0A:0x%02x,0x0B:0x%02x\n", debug_reg[8],debug_reg[9],debug_reg[10],debug_reg[11]);
	dev_info(switch_priv->dev,"reg:0x0C:0x%02x,0x0D:0x%02x,0x0E:0x%02x,0x0F:0x%02x\n", debug_reg[12],debug_reg[13],debug_reg[14],debug_reg[15]);
	dev_info(switch_priv->dev,"reg:0x10:0x%02x,0x11:0x%02x,0x12:0x%02x,0x13:0x%02x\n", debug_reg[16],debug_reg[17],debug_reg[18],debug_reg[19]);
	dev_info(switch_priv->dev,"reg:0x14:0x%02x,0x15:0x%02x,0x16:0x%02x,0x17:0x%02x\n", debug_reg[20],debug_reg[21],debug_reg[22],debug_reg[23]);
	dev_info(switch_priv->dev,"reg:0x18:0x%02x,0x19:0x%02x,0x1A:0x%02x,0x1B:0x%02x\n", debug_reg[24],debug_reg[25],debug_reg[26],debug_reg[27]);
	dev_info(switch_priv->dev,"reg:0x1C:0x%02x,0x1D:0x%02x,0x1E:0x%02x,0x1F:0x%02x\n", debug_reg[28],debug_reg[29],debug_reg[30],debug_reg[31]);

	dev_info(switch_priv->dev,"%s, %d dump reg <<<\n",__func__, __LINE__);
}
#endif

// TODO,No need
static void typec_switch_usbc_update_settings(struct typec_switch_priv *switch_priv,
		u32 switch_control, u32 switch_enable)
{
	unsigned int reg_val = 0;

	if (!switch_priv->regmap) {
		dev_info(switch_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	if (switch_priv->vendor == DIO4480) {
		return;
	}

	if (switch_priv->vendor == DIO4483) {
		return;
	}

	if (switch_priv->vendor == WAS4783) {
		return;
	}

	reg_val = 0;
	SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DEVICE_ENABLE);
	dev_info(switch_priv->dev, "%s, %d, reg_val = 0x%02x", __func__, __LINE__, reg_val);
	typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
	typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SELECT, switch_control);
	/* FSA4480 chip hardware requirement */
	usleep_range(50, 55);
	typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, switch_enable);
}

int typec_switch_to_fast_charger(int to_fast_charger)
{
	struct typec_switch_priv *switch_priv = g_typec_switch_priv;
	struct device *dev;
	int ret = 0;
	unsigned int reg_val = 0;

	unsigned int dn_l_status = 0;
	unsigned int dp_r_status = 0;
	unsigned int width = 2;

	if (!switch_priv) {
		pr_err("%s, switch_priv is NULL", __func__);

		return -EINVAL;
	}

	dev = switch_priv->dev;
	if (!dev) {
		pr_err("%s, switch_priv->dev is NULL", __func__);

		return -EINVAL;
	}

	if (switch_priv->vendor != DIO4483 && switch_priv->vendor != WAS4783) {
		dev_err(dev, "%s, %d, current chip 0x%02x, is not supported!", __func__, __LINE__, switch_priv->vendor);

		return -EINVAL;
	}

	dev_info(dev, "%s: charger_plugged = %d, to_fast_charger = %d\n", __func__, switch_priv->charger_plugged, to_fast_charger);
	typec_switch_wait_resume(switch_priv);
#ifdef DEBUG
	typec_switch_dump_reg();
#endif

	if (switch_priv->vendor == DIO4483) {
		regmap_read(switch_priv->regmap, DIO4483_REG_SWITCH_STATUS0, &reg_val);
		dn_l_status = GET_BITS(reg_val, DIO4483_SWITCH_STATUS0_DN_L_SWITCH_STATUS_L, width);
		dp_r_status = GET_BITS(reg_val, DIO4483_SWITCH_STATUS0_DP_R_SWITCH_STATUS_L, width);
	} else {
		regmap_read(switch_priv->regmap, DEFAULT_REG_SWITCH_STATUS0, &reg_val);
		dn_l_status = GET_BITS(reg_val, DEFAULT_SWITCH_STATUS0_DN_L_SWITCH_STATUS_L, width);
		dp_r_status = GET_BITS(reg_val, DEFAULT_SWITCH_STATUS0_DP_R_SWITCH_STATUS_L, width);
	}

	if (dn_l_status == 0x02 || dp_r_status == 0x02) {
		dev_info(dev,"%s, %d , switching from headphone to charger is prohibited!\n",__func__, __LINE__);

		return -EPERM;
	}

	if (to_fast_charger) {
		if (switch_priv->vendor == DIO4483) {
			reg_val = 0;
			SET_BIT(reg_val, DIO4483_SWITCH_SELECT_USB2_SWITCH);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DIO4483_REG_SWITCH_SELECT, reg_val);
			ret |= typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SELECT, reg_val);

			reg_val = 0;
			SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DEVICE_ENABLE);
			SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DN_L_TO_DN_or_L);
			SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DN_R_TO_DP_or_R);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DIO4483_REG_SWITCH_SETTINGS, reg_val);
			ret |= typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SETTINGS, reg_val);
			usleep_range(10000, 10005);

			dev_info(dev, "%s, %d, charger plugin. set to switch mode", __func__, __LINE__);
		} else {//WAS4783
			reg_val = 0;
			SET_BIT(reg_val, DEFAULT_SWITCH_SELECT_USB2_SWITCH);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SELECT, reg_val);
			ret |= typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SELECT, reg_val);

			reg_val = 0;
			SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DEVICE_ENABLE);
			SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DN_L_TO_DN_or_L);
			SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DN_R_TO_DP_or_R);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
			ret |= typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
			usleep_range(10000, 10005);

			dev_info(dev, "%s, %d, charger plugin. set to switch mode", __func__, __LINE__);
		}
	} else {
		if (switch_priv->vendor == DIO4483) {
			reg_val = 0;
			SET_BIT(reg_val, DIO4483_SWITCH_SELECT_DN_L_TO_DN_or_L);
			SET_BIT(reg_val, DIO4483_SWITCH_SELECT_DN_R_TO_DP_or_R);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DIO4483_REG_SWITCH_SELECT, reg_val);
			ret |= typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SELECT, reg_val);

			reg_val = 0;
			SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DEVICE_ENABLE);
			SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DN_L_TO_DN_or_L);
			SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DN_R_TO_DP_or_R);

			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DIO4483_REG_SWITCH_SETTINGS, reg_val);
			ret |= typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SETTINGS, reg_val);

			dev_info(dev, "%s, %d, charger plugout. set to usb mode", __func__, __LINE__);
		} else {//WAS4783
			reg_val = 0;
			SET_BIT(reg_val, DEFAULT_SWITCH_SELECT_DN_L_TO_DN_or_L);
			SET_BIT(reg_val, DEFAULT_SWITCH_SELECT_DN_R_TO_DP_or_R);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SELECT, reg_val);
			ret |= typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SELECT, reg_val);

			reg_val = 0;
			SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DEVICE_ENABLE);
			SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DN_L_TO_DN_or_L);
			SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DN_R_TO_DP_or_R);

			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
			ret |= typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
			dev_info(dev, "%s, %d, charger plugout. set to usb mode", __func__, __LINE__);
		}
	}

	if (ret != 0) {
		dev_err(dev, "%s, %d, regmap write fail, ret = %d!", __func__, __LINE__, ret);
		err_status = 1;
		if (to_fast_charger == 0) {
			reg_val = 0;
			SET_BIT(reg_val, DEFAULT_I2C_RESET);
			typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_RESET, reg_val);//reset
		}
	}

	return ret;
}
EXPORT_SYMBOL(typec_switch_to_fast_charger);

/* bit3~0:
* 0000 DPDM, 0001 fast charge, 0010 Headphone 0100 unknown
*
* bit7~4:
* 0000 not support 1to3 switch
* 0001 support 1to3 switch

* bit11~bit8:
* 0000 noram
* 0001 iic err
* 0010 invalid param */
int typec_switch_status0(void)
{
	struct typec_switch_priv *switch_priv = g_typec_switch_priv;

	int rc = 0;
	struct device *dev = NULL;

	unsigned int reg_status = 0, reg_status1 = 0, reg_val = 0;
	unsigned int reg_l_shift = 0, reg_r_shift = 0;
	unsigned int width = 2;
	unsigned int dn_l_status = 0;
	unsigned int dp_r_status = 0;


	if (!switch_priv) {
		pr_err("%s, switch_priv is NULL", __func__);
		rc |= TYPEC_AUDIO_SWITCH_STATE_INVALID_PARAM;
		goto err_handler;
	}

	dev = switch_priv->dev;
	if (!dev) {
		pr_err("%s, switch_priv->dev is NULL", __func__);
		rc |= TYPEC_AUDIO_SWITCH_STATE_INVALID_PARAM;
		goto err_handler;
	}

	if (switch_priv->vendor == DIO4480) {
		reg_status1 = DIO4480_REG_SWITCH_STATUS1;
		reg_status = DIO4480_REG_SWITCH_STATUS0;
		reg_l_shift = DIO4483_SWITCH_STATUS0_DN_L_SWITCH_STATUS_L;
		reg_r_shift = DIO4483_SWITCH_STATUS0_DP_R_SWITCH_STATUS_L;
	} else if (switch_priv->vendor == DIO4483) {
		reg_status1 = DIO4483_REG_SWITCH_STATUS1;
		reg_status = DIO4483_REG_SWITCH_STATUS0;
		reg_l_shift = DIO4483_SWITCH_STATUS0_DN_L_SWITCH_STATUS_L;
		reg_r_shift = DIO4483_SWITCH_STATUS0_DP_R_SWITCH_STATUS_L;
	} else {
		reg_status1 = DEFAULT_REG_SWITCH_STATUS1;
		reg_status = DEFAULT_REG_SWITCH_STATUS0;
		reg_l_shift = DEFAULT_SWITCH_STATUS0_DN_L_SWITCH_STATUS_L;
		reg_r_shift = DEFAULT_SWITCH_STATUS0_DP_R_SWITCH_STATUS_L;
		SET_BIT(rc, TYPEC_AUDIO_SWITCH_STATE_SUPPORT);//support 1to3
	}

	regmap_read(switch_priv->regmap, reg_status1, &reg_val);
	dev_info(dev, "%s, reg[0x%02x] = 0x%02x", __func__, reg_status1, reg_val);//old status

	regmap_read(switch_priv->regmap, reg_status, &reg_val);
	dev_info(dev, "%s, reg[0x%02x] = 0x%02x", __func__, reg_status, reg_val);

	dn_l_status = GET_BITS(reg_val, reg_l_shift, width);
	dp_r_status = GET_BITS(reg_val, reg_r_shift, width);

	dev_info(dev, "%s, dn_l_status = 0x%02x, dp_r_status = 0x%02x", __func__, dn_l_status, dp_r_status);

	switch (dn_l_status) {
	case 0x00:
		dev_info(dev, "%s, DN_L Switch Open/Not Connected", __func__);
		break;
	case 0x01:
		dev_info(dev, "%s, DN_L connected to DN1", __func__);
		break;
	case 0x02: // Headphone
		dev_info(dev, "%s, DN_L connected to L", __func__);
		break;
	case 0x03: // Fast Charger
		dev_info(dev, "%s, DN_L connected to DN2", __func__);
		break;
	default:
		break;
	}

	switch (dp_r_status) {
	case 0x00:
		dev_info(dev, "%s, DP_R Switch Open/Not Connected", __func__);
		break;
	case 0x01:
		dev_info(dev, "%s, DP_R connected to DP1", __func__);
		break;
	case 0x02: // Headphone
		dev_info(dev, "%s, DP_R connected to R", __func__);
		break;
	case 0x03: // Fast Charger
		dev_info(dev, "%s, DP_R connected to DP2", __func__);
		break;
	default:
		break;
	}

	if ((dn_l_status == 0x01) && (dp_r_status == 0x01)) { // Charger
		rc &= ~(0x1 << TYPEC_AUDIO_SWITCH_STATE_DPDM);
	} else if ((dn_l_status == 0x03) && (dp_r_status == 0x03)) { // Fast Charger
		rc |= TYPEC_AUDIO_SWITCH_STATE_FAST_CHG;
	} else if ((dn_l_status == 0x02) && (dp_r_status == 0x02)) { // Headphone
		rc |= TYPEC_AUDIO_SWITCH_STATE_AUDIO;
	} else { // Unknown
		rc |= TYPEC_AUDIO_SWITCH_STATE_UNKNOW;
		pr_err("%s, Typec audio switch state is unknow", __func__);
	}

err_handler:
	if (err_status) {
		rc |= TYPEC_AUDIO_SWITCH_STATE_I2C_ERR;
		err_status = 0;//reset status
	}

	return rc;
}
EXPORT_SYMBOL(typec_switch_status0);

static int typec_switch_usbc_event_changed(struct notifier_block *nb,
				unsigned long evt, void *ptr)
{
	struct typec_switch_priv *switch_priv =
			container_of(nb, struct typec_switch_priv, pd_nb);
	struct device *dev;
	struct tcp_notify *noti = ptr;
	int state = 0;
	int rc = 0;

	if (!switch_priv) {
		pr_err("%s, switch_priv is NULL", __func__);
		return -EINVAL;
	}

	dev = switch_priv->dev;
	if (!dev) {
		pr_err("%s, switch_priv->dev is NULL", __func__);
		return -EINVAL;
	}

	if (evt == TCP_NOTIFY_TYPEC_STATE) {
		dev_info(dev, "%s: typeC event = %lu, plug_state = %d\n", __func__, evt, switch_priv->plug_state);
	}

	switch (evt) {
	case TCP_NOTIFY_TYPEC_STATE:// 14
		dev_info(dev, "%s: old_state: %d, new_state: %d\n",
			__func__, noti->typec_state.old_state, noti->typec_state.new_state);
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SNK) {
			/* SNK plug in */
			dev_err(dev, "%s: SNK  has been plugged in\n", __func__);
			switch_priv->charger_plugged = 1;
		} else if (switch_priv->charger_plugged != 0 && noti->typec_state.old_state == TYPEC_ATTACHED_SNK
			&& noti->typec_state.new_state == TYPEC_UNATTACHED) {
			/* Charger plug out */
			dev_err(dev, "%s: SNK has been plugged out\n", __func__);
			switch_priv->charger_plugged = 0;
		} else if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO) {
			/* AUDIO plug in */
			dev_info(dev, "%s: audio plug in\n", __func__);
			switch_priv->plug_state = true;
			pm_stay_awake(switch_priv->dev);
			cancel_work_sync(&switch_priv->usbc_analog_work);
			schedule_work(&switch_priv->usbc_analog_work);
		} else if (switch_priv->plug_state == true
			&& noti->typec_state.new_state == TYPEC_UNATTACHED) {
			/* AUDIO plug out */
			dev_info(dev, "%s: audio plug out\n", __func__);
			switch_priv->plug_state = false;
			pm_stay_awake(switch_priv->dev);
			cancel_work_sync(&switch_priv->usbc_analog_work);
			schedule_work(&switch_priv->usbc_analog_work);
		}
		break;
	case TCP_NOTIFY_PD_STATE:
		dev_info(dev, "%s, %d, noti->pd_state.connected = %d\n",
			__func__, __LINE__, noti->pd_state.connected);
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			break;
		case PD_CONNECT_TYPEC_ONLY_SNK_DFT:
			break;
		case PD_CONNECT_HARD_RESET:
			break;
		default:
			break;
		}
		break;
#ifdef OPLUS_FEATURE_CHG_BASIC
	case TCP_NOTIFY_CHRDET_STATE:
		dev_info(dev, "%s, %d, noti->chrdet_state.chrdet = %d\n",
			__func__, __LINE__, noti->chrdet_state.chrdet);
		break;

#if IS_ENABLED(CONFIG_SND_SOC_OPLUS_TYPEC_SWITCH)
	case TCP_NOTIFY_SWITCH_GET_STATE:
		state = typec_switch_status0();
		if (noti->switch_get_status.pfunc) {
			dev_info(dev, "%s, %d, return state = %d\n", __func__, __LINE__, state);
			noti->switch_get_status.pfunc(state);
		}
		break;

	case TCP_NOTIFY_SWITCH_SET_STATE:
		state = noti->switch_set_status.state;
		rc = typec_switch_to_fast_charger(state);
		if (noti->switch_set_status.pfunc) {
			dev_info(dev, "%s, %d, return state = %d\n", __func__, __LINE__, rc);
			noti->switch_set_status.pfunc(rc);
		}
		break;
#endif
#endif
	case TCP_NOTIFY_PLUG_OUT:
		dev_info(dev, "%s: typec plug out\n", __func__);
		break;
	default:
		break;
	};

	return NOTIFY_OK;
}

static int typec_switch_usbc_analog_setup_switches(struct typec_switch_priv *switch_priv)
{
	int ret = 0;
	struct device *dev;
	unsigned int switch_status = 0;
	unsigned int jack_status = 0;
	unsigned int reg_val = 0;
	int state;
	int i = 0;

	if (!switch_priv) {
		pr_err("%s, switch_priv is NULL", __func__);

		return -EINVAL;
	}
	dev = switch_priv->dev;
	if (!dev) {
		pr_err("%s, switch_priv->dev is NULL", __func__);

		return -EINVAL;
	}
	typec_switch_wait_resume(switch_priv);
	mutex_lock(&switch_priv->notification_lock);

	dev_info(dev, "%s: plug_state %d\n", __func__, switch_priv->plug_state);

#ifdef DEBUG
	typec_switch_dump_reg();
#endif

	if (switch_priv->plug_state) {
		pr_info("plugin regulator_get_voltage(%d)\n", regulator_get_voltage(vio28_reg));
		// TODO
		if (switch_priv->vendor == DIO4480) {
			/* activate switches */
			ret = typec_switch_write_register(switch_priv->regmap, DIO4480_REG_RESET, 0x01);//reset DIO4480

			usleep_range(1000, 1005);
			ret = typec_switch_write_register(switch_priv->regmap, DIO4480_REG_FUN_EN, 0x45);

			// TODO
			for (i = 0;i < 100 ;i++) {
				usleep_range(10*1000, 10*1005);
				regmap_read(switch_priv->regmap, 0x18, &reg_val);
				if (reg_val & 0x4) {
					dev_info(dev, "%s: Audio jack detection and configuration has occurred.\n", __func__);

					break;
				}
			}

			dev_info(dev, "%s, %d, set reg[0x%02x] done.\n", __func__, __LINE__, DIO4480_REG_FUN_EN);
			typec_switch_status = 0;
		} else if (switch_priv->vendor == DIO4483) {
			/* activate switches */
			reg_val = 0;
			SET_BIT(reg_val, DIO4483_I2C_RESET);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DIO4483_REG_FUN_EN, reg_val);

			ret = typec_switch_write_register(switch_priv->regmap, DIO4483_REG_RESET, reg_val); // reset DIO4483
			usleep_range(1000, 1005);

			reg_val = 0x40; // 4.6V
			SET_BIT(reg_val, DIO4483_FUNCTION_MIC_AUTO_TURN_OUT);
			SET_BIT(reg_val, DIO4483_FUNCTION_AUDIO_JACK_DECTION);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DIO4483_REG_FUN_EN, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DIO4483_REG_FUN_EN, reg_val);

			for (i = 0;i < 100 ;i++) {
				usleep_range(10*1000, 10*1005);
				regmap_read(switch_priv->regmap, 0x18, &reg_val);
				if (GET_BIT(reg_val, DIO4483_DETECTION_FLAG_AUDIO_JACK_DETECTION_CONFIGURATION_OCCURRED)) {
					dev_info(dev, "%s: Audio jack detection and configuration has occurred.\n", __func__);

					break;
				}
			}

			dev_info(dev, "%s, %d, set reg[0x%02x] done.\n", __func__, __LINE__, DIO4483_REG_FUN_EN);
			typec_switch_status = 0;
		} else if (switch_priv->vendor == WAS4783) {
			reg_val = 0x00;
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SELECT, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SELECT, reg_val);

			reg_val = 0x99;
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, reg_val);

			reg_val = 0x07; //set 19h for fix 18h Audio jack detection and switch occurred slow issue
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, 0x19, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, 0x19, reg_val);

			reg_val = 0x00; // 4.4V
			SET_BIT(reg_val, DEFAULT_FUNCTION_SLOW_TURN_ON);
			SET_BIT(reg_val, DEFAULT_FUNCTION_MIC_AUTO_TURN_OUT);
			SET_BIT(reg_val, DEFAULT_FUNCTION_AUDIO_JACK_DECTION);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_FUN_EN, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_FUN_EN, reg_val);

			for (i = 0;i < 100 ;i++) {
				usleep_range(10*1000, 10*1005);
				regmap_read(switch_priv->regmap, 0x18, &reg_val);
				dev_info(dev, "%s, %d, i = %d, reg_val = 0x%02x", __func__, __LINE__, i, reg_val);
				if (GET_BIT(reg_val, DEFAULT_DETECTION_FLAG_AUDIO_JACK_DETECTION_CONFIGURATION_OCCURRED)) {
					dev_info(dev, "%s: Audio jack detection and configuration has occurred.\n", __func__);

					break;
				}
			}

			dev_info(dev, "%s, %d, set reg[0x%02x] done.\n", __func__, __LINE__, DEFAULT_REG_FUN_EN);
			typec_switch_status = 0;

		} else {
			reg_val = 0x9F;
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
			usleep_range(1000, 1005);

			reg_val = 0x40; // 4.6V
			SET_BIT(reg_val, DEFAULT_FUNCTION_SLOW_TURN_ON);
			SET_BIT(reg_val, DEFAULT_FUNCTION_AUDIO_JACK_DECTION);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_FUN_EN, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_FUN_EN, reg_val);
			usleep_range(4000, 4005);

			dev_info(dev, "%s, %d, set reg[0x%02x] done.\n", __func__, __LINE__, DEFAULT_REG_FUN_EN);
			typec_switch_status = 0;
		}

		regmap_read(switch_priv->regmap, DEFAULT_REG_JACK_STATUS, &jack_status);
		dev_info(dev, "%s, %d, read reg[0x%02x] = 0x%02x\n", __func__, __LINE__, DEFAULT_REG_JACK_STATUS, jack_status);
		// ZZZ No need?
		if ((jack_status & 0x2) && (switch_priv->vendor != DIO4480 && switch_priv->vendor != DIO4483 && switch_priv->vendor != WAS4783)) {
			//for 3 pole, mic switch to SBU2
			dev_info(dev, "%s: set mic to sbu2 for 3 pole.\n", __func__);
			typec_switch_usbc_update_settings(switch_priv, 0x00, 0x9F);
			usleep_range(4000, 4005);
		}

		regmap_read(switch_priv->regmap, DEFAULT_REG_SWITCH_STATUS0, &switch_status);
		dev_info(dev, "%s, %d, read reg[0x%02x] = 0x%02x\n", __func__, __LINE__, DEFAULT_REG_SWITCH_STATUS0, switch_status);
		regmap_read(switch_priv->regmap, DEFAULT_REG_SWITCH_STATUS1, &switch_status);
		dev_info(dev, "%s, %d, read reg[0x%02x] = 0x%02x\n", __func__, __LINE__, DEFAULT_REG_SWITCH_STATUS1, switch_status);

		if (switch_priv->vendor == DIO4480) {
			regmap_read(switch_priv->regmap, DIO4483_REG_JACK_STATUS, &jack_status);
			dev_info(dev, "%s: reg[0x17]=0x%02x.\n", __func__, jack_status);

			if (jack_status == 0x01) {
				dev_info(dev, "%s: error status,swap MIC_GND\n", __func__);
				typec_switch_write_register(switch_priv->regmap, DIO4480_REG_RESET, 0x01);//reset DIO4480
				usleep_range(1000, 1005);
				typec_switch_write_register(switch_priv->regmap, DIO4480_REG_SWITCH_SELECT, 0x00);//GND - GSBU1, MIC - SBU2
				typec_switch_write_register(switch_priv->regmap, DIO4480_REG_SWITCH_SETTINGS, 0x9f);
				usleep_range(10000, 10005);
				typec_switch_status = 1;
			}
		} else if (switch_priv->vendor == DIO4483) {
			regmap_read(switch_priv->regmap, DIO4483_REG_JACK_STATUS, &jack_status);
			dev_info(dev, "%s, %d, read reg[0x%02x] = 0x%02x.\n", __func__, __LINE__, DIO4483_REG_JACK_STATUS, jack_status);

			if (jack_status == 0x01) {
				dev_info(dev, "%s: error status,swap MIC_GND\n", __func__);
				reg_val = 0;
				SET_BIT(reg_val, DIO4483_I2C_RESET);
				dev_info(dev, "%s, %d, reg_val = 0x%02x", __func__, __LINE__, reg_val);
				typec_switch_write_register(switch_priv->regmap, DIO4483_REG_RESET, reg_val);//reset DIO4483
				usleep_range(1000, 1005);
				typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SELECT, 0x00);//GND - GSBU1, MIC - SBU2

				reg_val = 0;
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DEVICE_ENABLE);
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DN_L_TO_DN_or_L);
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DN_R_TO_DP_or_R);
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_SENSE_TO_GSBUx);
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_MIC_TO_SBUx);
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_AGND_TO_SBUx);
				dev_info(dev, "%s, %d, reg_val = 0x%02x", __func__, __LINE__, reg_val);
				typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SETTINGS, reg_val);
				usleep_range(10000, 10005);
				typec_switch_status = 1;
			}
		} else {
			regmap_read(switch_priv->regmap, DEFAULT_REG_JACK_STATUS, &jack_status);
			dev_info(dev, "%s, %d, reg[0x%02x] = 0x%02x.\n", __func__, __LINE__, DEFAULT_REG_JACK_STATUS, jack_status);

			if (jack_status == 0x01) {
				dev_info(dev, "%s: error status,swap MIC_GND\n", __func__);

				ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SELECT, 0x00);//GND - GSBU1, MIC - SBU2

				reg_val = 0;
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DEVICE_ENABLE);
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DN_L_TO_DN_or_L);
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DN_R_TO_DP_or_R);
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_SENSE_TO_GSBUx);
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_MIC_TO_SBUx);
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_AGND_TO_SBUx);
				dev_info(dev, "%s, %d, reg_val = 0x%02x", __func__, __LINE__, reg_val);
				ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
				usleep_range(10000, 10005);
				typec_switch_status = 1;
			}

			if (jack_status == 0x02) {
				regmap_read(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, &reg_val);
				dev_info(dev, "%s, %d, reg[0x%02x] = 0x%02x.\n", __func__, __LINE__, DEFAULT_REG_SWITCH_SETTINGS, reg_val);

				regmap_read(switch_priv->regmap, DEFAULT_REG_SWITCH_SELECT, &reg_val);
				dev_info(dev, "%s, %d, reg[0x%02x] = 0x%02x.\n", __func__, __LINE__, DEFAULT_REG_SWITCH_SELECT, reg_val);

				reg_val = 0x9B;
				dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
				ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
				usleep_range(10000, 10005);

				reg_val = 0x00;
				dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SELECT, reg_val);
				ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SELECT, reg_val);
				usleep_range(10000, 10005);

				regmap_read(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, &reg_val);
				dev_info(dev, "%s, %d, reg[0x%02x] = 0x%02x.\n", __func__, __LINE__, DEFAULT_REG_SWITCH_SETTINGS, reg_val);

				regmap_read(switch_priv->regmap, DEFAULT_REG_SWITCH_SELECT, &reg_val);
				dev_info(dev, "%s, %d, reg[0x%02x] = 0x%02x.\n", __func__, __LINE__, DEFAULT_REG_SWITCH_SELECT, reg_val);

			}

		}

		if (gpio_is_valid(switch_priv->hs_det_pin)) {
			dev_info(dev, "%s, %d: set hs_det_pin %d to enable.\n", __func__, __LINE__, switch_priv->hs_det_pin);
			state = gpio_get_value(switch_priv->hs_det_pin);
			dev_info(dev, "%s: before hs_det_pin state = %d.\n", __func__, state);
			usleep_range(1000, 1005);
			gpio_direction_output(switch_priv->hs_det_pin, switch_priv->hs_det_level);
			state = gpio_get_value(switch_priv->hs_det_pin);
			dev_info(dev, "%s: after hs_det_pin state = %d.\n", __func__, state);
		}
	} else {
		pr_info("plugout regulator_get_voltage(%d)\n", regulator_get_voltage(vio28_reg));
		if (gpio_is_valid(switch_priv->hs_det_pin)) {
			dev_info(dev, "%s: set hs_det_pin to disable.\n", __func__);
			state = gpio_get_value(switch_priv->hs_det_pin);
			dev_info(dev, "%s: before hs_det_pin state = %d.\n", __func__, state);
			gpio_direction_output(switch_priv->hs_det_pin, !switch_priv->hs_det_level);
			state = gpio_get_value(switch_priv->hs_det_pin);
			dev_info(dev, "%s: after hs_det_pin state = %d.\n", __func__, state);
		}

		if (switch_priv->vendor == DIO4480) {
			ret = typec_switch_write_register(switch_priv->regmap, DIO4480_REG_RESET, 0x01);//reset DIO4480
			usleep_range(1000, 1005);
			ret = typec_switch_write_register(switch_priv->regmap, DIO4480_REG_SWITCH_SELECT, 0x18);
			ret = typec_switch_write_register(switch_priv->regmap, DIO4480_REG_SWITCH_SETTINGS, 0x98);
			dev_info(dev, "%s: plugout. set to usb mode\n", __func__);
		} else if (switch_priv->vendor == DIO4483) {
			reg_val = 0;
			SET_BIT(reg_val, DIO4483_I2C_RESET);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DIO4483_REG_RESET, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DIO4483_REG_RESET, reg_val);//reset DIO4483
			usleep_range(1000, 1005);

			reg_val = 0;
			SET_BIT(reg_val, DIO4483_SWITCH_SELECT_DN_L_TO_DN_or_L);
			SET_BIT(reg_val, DIO4483_SWITCH_SELECT_DN_R_TO_DP_or_R);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DIO4483_REG_SWITCH_SELECT, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SELECT, reg_val);

			reg_val = 0;
			SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DEVICE_ENABLE);
			SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DN_L_TO_DN_or_L);
			SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DN_R_TO_DP_or_R);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DIO4483_REG_SWITCH_SETTINGS, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SETTINGS, reg_val);
			dev_info(dev, "%s, %d, plugout. set to usb mode\n", __func__, __LINE__);
		} else {
			reg_val = 0;
			SET_BIT(reg_val, DEFAULT_I2C_RESET);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_RESET, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_RESET, reg_val);//reset WAS4783
			usleep_range(1000, 1005);

			reg_val = 0;
			SET_BIT(reg_val, DEFAULT_SWITCH_SELECT_DN_L_TO_DN_or_L);
			SET_BIT(reg_val, DEFAULT_SWITCH_SELECT_DN_R_TO_DP_or_R);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SELECT, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SELECT, reg_val);

			reg_val = 0;
			SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DEVICE_ENABLE);
			SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DN_L_TO_DN_or_L);
			SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DN_R_TO_DP_or_R);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
			dev_info(dev, "%s, %d, plugout. set to usb mode\n", __func__, __LINE__);

			reg_val = 0; // 4.6V
			SET_BIT(reg_val, DEFAULT_FUNCTION_SLOW_TURN_ON);
			dev_info(dev, "%s, %d, write 0x%02x = 0x%02x", __func__, __LINE__, DEFAULT_REG_FUN_EN, reg_val);
			ret = typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_FUN_EN, reg_val);
		}
		if (ret != 0) {
			reg_val = 0;
			SET_BIT(reg_val, DEFAULT_I2C_RESET);
			typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_RESET, reg_val);//reset
		}
	}
	if (ret != 0) {
		dev_err(dev, "%s, %d, regmap write fail, ret = %d!", __func__, __LINE__, ret);
		err_status = 1;
	}

	mutex_unlock(&switch_priv->notification_lock);
	return ret;
}

static int typec_switch_validate_display_port_settings(struct typec_switch_priv *switch_priv)
{
	u32 switch_status = 0;

	if (!switch_priv->regmap) {
		pr_err("switch_priv->regmap is NULL\n");
		return -EINVAL;
	}

	regmap_read(switch_priv->regmap, DEFAULT_REG_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		pr_err("AUX SBU1/2 switch status is invalid = 0x%02x\n",
				switch_status);
		return -EIO;
	}

	return 0;
}
/*
 * typec_switch_event - configure FSA switch position based on event
 *
 * @node - phandle node to fsa4480 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int typec_switch_event(struct device_node *node,
			 enum typec_switch_function event)
{
	int switch_control = 0;
	unsigned int reg_val = 0;

#ifndef OPLUS_ARCH_EXTENDS
//2020/11/04, support mic and ground switch to fix headset detect bug
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct typec_switch_priv *switch_priv;

	if (!client) {
		pr_err("%s, client is NULL", __func__);
		return -EINVAL;
	}

	switch_priv = (struct typec_switch_priv *)i2c_get_clientdata(client);
	if (!switch_priv) {
		pr_err("%s, switch_priv is NULL", __func__);
		return -EINVAL;
	}
#else
	struct typec_switch_priv *switch_priv = g_typec_switch_priv;
	if (!switch_priv) {
		pr_err("switch_priv is NULL\n");
		return -EINVAL;
	}
#endif
	if (!switch_priv->regmap) {
		pr_err("switch_priv->regmap is NULL\n");
		return -EINVAL;
	}

	pr_info("%s - switch event: %d\n", __func__, event);

	switch (event) {
	case TYPEC_SWITCH_MIC_GND_SWAP:
		if (switch_priv->vendor == DIO4480) {
			if (typec_switch_status) {
				pr_info("%s - switch event: %d delay 0ms\n", __func__, event);
				typec_switch_write_register(switch_priv->regmap, DIO4483_REG_RESET, 0x01);
				usleep_range(1000, 1005);
				typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SELECT, 0x07); //GND - GSBU2, MIC - SBU1
				typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SETTINGS, 0x9f);
				typec_switch_status = 0;
			}
		} else if (switch_priv->vendor == DIO4483) {
			if (typec_switch_status) {
				pr_info("%s - switch event: %d delay 0ms\n", __func__, event);
				reg_val = 0;
				SET_BIT(reg_val, DIO4483_I2C_RESET);
				dev_info(switch_priv->dev, "%s, %d, reg_val = 0x%02x", __func__, __LINE__, reg_val);
				typec_switch_write_register(switch_priv->regmap, DIO4483_REG_RESET, reg_val);
				usleep_range(1000, 1005);

				// TODO Readonly?
				typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SELECT, 0x07); //GND - GSBU2, MIC - SBU1

				reg_val = 0;
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DEVICE_ENABLE);
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DN_L_TO_DN_or_L);
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_DN_R_TO_DP_or_R);
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_SENSE_TO_GSBUx);
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_MIC_TO_SBUx);
				SET_BIT(reg_val, DIO4483_SWITCH_SETTINGS_AGND_TO_SBUx);
				dev_info(switch_priv->dev, "%s, %d, reg_val = 0x%02x", __func__, __LINE__, reg_val);
				typec_switch_write_register(switch_priv->regmap, DIO4483_REG_SWITCH_SETTINGS, reg_val);
				typec_switch_status = 0;
			}
		} else {
			if (typec_switch_status) {
				// TODO Readonly?
				typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SELECT, 0x07); //GND - GSBU2, MIC - SBU1

				reg_val = 0;
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DEVICE_ENABLE);
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DN_L_TO_DN_or_L);
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_DN_R_TO_DP_or_R);
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_SENSE_TO_GSBUx);
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_MIC_TO_SBUx);
				SET_BIT(reg_val, DEFAULT_SWITCH_SETTINGS_AGND_TO_SBUx);
				dev_info(switch_priv->dev, "%s, %d, reg_val = 0x%02x", __func__, __LINE__, reg_val);
				typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_SWITCH_SETTINGS, reg_val);
				typec_switch_status = 0;
			}

		}
		typec_switch_usbc_update_settings(switch_priv, switch_control, 0x9F);
		break;
	case TYPEC_SWITCH_USBC_ORIENTATION_CC1:
		typec_switch_usbc_update_settings(switch_priv, 0x18, 0xF8);
		return typec_switch_validate_display_port_settings(switch_priv);
	case TYPEC_SWITCH_USBC_ORIENTATION_CC2:
		typec_switch_usbc_update_settings(switch_priv, 0x78, 0xF8);
		return typec_switch_validate_display_port_settings(switch_priv);
	case TYPEC_SWITCH_USBC_DISPLAYPORT_DISCONNECTED:
		typec_switch_usbc_update_settings(switch_priv, 0x18, 0x98);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(typec_switch_event);

static int typec_switch_parse_dt(struct typec_switch_priv *switch_priv,
	struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	int ret = 0;
	int hs_det_level = 0;
	int state = 0;
	int sense_to_ground = 0;

/*2020/08/20, adjust the voltage of Headset DET */
	vio28_reg = regulator_get(dev, "dio_audio");

	if (IS_ERR_OR_NULL(vio28_reg)) {
		pr_info("%s: VIO28_reg is NULL\n", __func__);
	} else {
		/*set 3v*/
		ret = regulator_set_voltage(vio28_reg, 3000000, 3000000);
		if (ret) {
			pr_err("regulator_set_voltage(%d) failed!\n", ret);
		}

		pr_info("regulator_get_voltage(%d)\n", regulator_get_voltage(vio28_reg));
	}

	if (dNode == NULL) {
		pr_err("%s, dNode is NULL", __func__);
		return -ENODEV;
	}

	if (!switch_priv) {
		pr_info("%s: switch_priv is NULL\n", __func__);
		return -ENOMEM;
	}

	switch_priv->hs_det_pin = of_get_named_gpio(dNode,
			"oplus,hs-det-gpio", 0);
	pr_info("%s,%d, hs-det-gpio = %d, ARCH_NR_GPIOS = %d\n", __func__, __LINE__, switch_priv->hs_det_pin, ARCH_NR_GPIOS);
	if (!gpio_is_valid(switch_priv->hs_det_pin)) {
		pr_info("%s: hs-det-gpio in dt node is missing\n", __func__);
		//return -ENODEV;
	}

	/*2021/01/04, add for fsa4480 hs_det_level*/
	ret = of_property_read_u32(dNode,
			"oplus,hs-det-level", &hs_det_level);
	if (ret) {
		pr_info("%s: hs-det-level request fail\n", __func__);
		switch_priv->hs_det_level = 0;
	} else {
		switch_priv->hs_det_level = hs_det_level;
		pr_debug("%s,%d, hs_det_level = %d\n", __func__, __LINE__, switch_priv->hs_det_level);
	}

	/*2021/07/20, supporting dynamic sense to ground, fix leakage bug */
	ret = of_property_read_u32(dNode,
			"fsa4480,dynamic-sense-to-gnd", &sense_to_ground);
	if (ret) {
		pr_info("%s: read prop sense-to-gnd fail\n", __func__);
		switch_priv->b_dynamic_sense_to_ground = false;
	} else {
		switch_priv->b_dynamic_sense_to_ground = sense_to_ground;
	}

	if (gpio_is_valid(switch_priv->hs_det_pin)) {
		ret = gpio_request(switch_priv->hs_det_pin, "typec_switch_hs_det");
		if (ret) {
			pr_err("%s: hs-det-gpio request fail\n", __func__);
			//return ret;
		}
		dev_info(dev, "%s: hs_det_pin gpio = %d.\n", __func__, switch_priv->hs_det_pin);

		/*2021/03/02, add for fsa4480 hs_det_level*/
		gpio_direction_output(switch_priv->hs_det_pin, !switch_priv->hs_det_level);

		state = gpio_get_value(switch_priv->hs_det_pin);
		dev_info(dev, "%s: init hs_det_pin state = %d.\n", __func__, state);
	} else {
		pr_info("%s, %d, switch_priv->hs_det_pin = %d , do nothing!\n", __func__, __LINE__, switch_priv->hs_det_pin);
	}

	return ret;
}

static void typec_switch_usbc_analog_work_fn(struct work_struct *work)
{
	struct typec_switch_priv *switch_priv =
		container_of(work, struct typec_switch_priv, usbc_analog_work);

	if (!switch_priv) {
		pr_err("%s: switch_priv is NULL\n", __func__);

		pm_relax(switch_priv->dev);

		return;
	}
	typec_switch_usbc_analog_setup_switches(switch_priv);
	pm_relax(switch_priv->dev);

	typec_switch_status0();
}

// ZZZ TODO
static void dio4483_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	for (i = 0; i < ARRAY_SIZE(dio4483_i2c_reg); i++) {
		typec_switch_write_register(regmap, dio4483_i2c_reg[i].reg,
				dio4483_i2c_reg[i].val);
	}
}

static void default_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	for (i = 0; i < ARRAY_SIZE(default_i2c_reg); i++) {
		typec_switch_write_register(regmap, default_i2c_reg[i].reg,
				default_i2c_reg[i].val);
	}
}

/* add start for DP */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static int typec_switch_mux_set(struct typec_mux_dev *mux, struct typec_mux_state *state)
#else
static int typec_switch_mux_set(struct typec_mux *mux, struct typec_mux_state *state)
#endif
{
	struct typec_switch_priv *switch_priv = typec_mux_get_drvdata(mux);
	struct tcp_notify *data = state->data;
	int ret = 0;
	//enum typec_orientation orientation;
	int orientation;
	unsigned int switch_status0 = 0;
	unsigned int switch_status1 = 0;

	/* Debug Message
	 dev_info(switch_priv->dev, "typec_switch_mux_set\n");
	 dev_info(switch_priv->dev, "state->mode : %d\n", state->mode);
	 dev_info(switch_priv->dev, "data-> polarity : %d\n", data->ama_dp_state.polarity);
	 dev_info(switch_priv->dev, "data-> signal : %d\n", data->ama_dp_state.signal);
	 dev_info(switch_priv->dev, "data-> pin_assignment : %d\n", data->ama_dp_state.pin_assignment);
	 dev_info(switch_priv->dev, "data-> active : %d\n", data->ama_dp_state.active);
	 */

	if (state->mode == TCP_NOTIFY_AMA_DP_STATE) {
		dev_info(switch_priv->dev, "%s DP state = %d\n", __func__, data->ama_dp_state.polarity);
		orientation = data->ama_dp_state.polarity;
		switch (orientation) {
		case USB_TYPEC_NORMAL:
			/* switch cc1 side */
			typec_switch_usbc_update_settings(switch_priv, 0x18, 0xF8);
			return typec_switch_validate_display_port_settings(switch_priv);
		case USB_TYPEC_REVERSE:
			typec_switch_usbc_update_settings(switch_priv, 0x78, 0xF8);
			return typec_switch_validate_display_port_settings(switch_priv);
			break;
		default:
			break;
		}
	} else if (state->mode == TCP_NOTIFY_TYPEC_STATE) {
		if ((data->typec_state.old_state == TYPEC_ATTACHED_SRC
			|| data->typec_state.old_state == TYPEC_ATTACHED_SNK)
			&& data->typec_state.new_state == TYPEC_UNATTACHED) {
			regmap_read(switch_priv->regmap, DEFAULT_REG_SWITCH_STATUS0, &switch_status0);
			regmap_read(switch_priv->regmap, DEFAULT_REG_SWITCH_STATUS1, &switch_status1);
			dev_info(switch_priv->dev, "%s: reg[0x%x]=0x%x, reg[0x%x]=0x%x.\n", __func__, DEFAULT_REG_SWITCH_STATUS0,
				 switch_status0, DEFAULT_REG_SWITCH_STATUS1, switch_status1);
			if (((switch_status0&0x7F) == 0x05)&& ((switch_status1&0x3F) == 0x00)) {
				dev_info(switch_priv->dev, "switch already be usb state.\n");
				return ret;
			}
			typec_switch_usbc_update_settings(switch_priv, 0x18, 0x98);
			dev_info(switch_priv->dev, "Plug out\n");
		}
	}
	return ret;
}
/* add end for DP */

#ifdef OPLUS_ARCH_EXTENDS
//2021/12/02, check the headset pluging state when probe, fix headset detect bug
static void hp_work_callback(struct work_struct *work)
{
	struct typec_switch_priv *switch_priv = g_typec_switch_priv;

	if (switch_priv->plug_state == true) {
		pr_info("%s: headphone is inserted already\n", __func__);

		return;
	}

	if (tcpm_inquire_typec_attach_state(switch_priv->tcpc_dev) == TYPEC_ATTACHED_AUDIO) {
		pr_info("%s: TYPEC_ATTACHED_AUDIO is inserted\n", __func__);
		switch_priv->plug_state = true;
		pm_stay_awake(switch_priv->dev);
		cancel_work_sync(&switch_priv->usbc_analog_work);
		schedule_work(&switch_priv->usbc_analog_work);
	} else if (tcpm_inquire_typec_attach_state(switch_priv->tcpc_dev) == TYPEC_ATTACHED_SNK) {
		pr_info("%s: SNK has been inserted\n", __func__);
	} else {
		pr_info("%s: TYPEC_ATTACHED_AUDIO is not inserted\n", __func__);
	}
}
#endif

static int typec_switch_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct typec_switch_priv *switch_priv;
	int rc = 0;
	/* add start for DP */
	struct device *dev = &i2c->dev;
	struct typec_mux_desc mux_desc;
	/* add end for DP */
	int i = 0;

	unsigned int chip_id = 0;
	unsigned int reg_val = 0;

	int ret = 0;
	switch_priv = devm_kzalloc(&i2c->dev, sizeof(*switch_priv),
				GFP_KERNEL);
	if (!switch_priv) {
		pr_err("%s, alloc memory failed!", __func__);
		return -ENOMEM;
	}

	switch_priv->dev = &i2c->dev;

	init_completion(&switch_priv->resume_ack);
	complete_all(&switch_priv->resume_ack);

	typec_switch_parse_dt(switch_priv, &i2c->dev);

	switch_priv->regmap = devm_regmap_init_i2c(i2c, &typec_switch_regmap_config);
	if (IS_ERR_OR_NULL(switch_priv->regmap)) {
		dev_err(switch_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!switch_priv->regmap) {
			rc = -EINVAL;
			goto err_data;
		}
		rc = PTR_ERR(switch_priv->regmap);
		goto err_data;
	}

	//reset device
	reg_val = 0;
	SET_BIT(reg_val, DEFAULT_I2C_RESET);
	dev_info(dev, "%s, %d, reg_val = 0x%02x", __func__, __LINE__, reg_val);
	typec_switch_write_register(switch_priv->regmap, DEFAULT_REG_RESET, reg_val);//reset DIO4483
	usleep_range(1*1000, 1*1005);
	// check device id
	ret = regmap_read(switch_priv->regmap, DIO_REG_DEVICE_ID, &chip_id);
	dev_info(switch_priv->dev, "%s: device chip_id = 0x%02x\n", __func__, chip_id);
	//2022/01/07, add for DIO4480 device id check error
	for (i = 0 ; i < 4 && (ret < 0); i++){
		ret = regmap_read(switch_priv->regmap, DIO_REG_DEVICE_ID, &chip_id);//reset DIO4480
		pr_err("%s: %d read, device id reg value: 0x%02x\n", __func__, i, chip_id);
	}

	switch (chip_id) {
	case DIO_CHIP_4480:
		switch_priv->vendor = DIO4480;
		break;
	case DIO_CHIP_4483:
		switch_priv->vendor = DIO4483;
		break;
	case DIO_CHIP_4483_2:
		switch_priv->vendor = DIO4483;
		break;
	case WAS_CHIP_4783:
		switch_priv->vendor = WAS4783;
		break;
	default:
		goto err_data;
	}

	if (switch_priv->vendor == DIO4483) {
		dio4483_update_reg_defaults(switch_priv->regmap);
	}

	if (switch_priv->vendor == WAS4783) {
		default_update_reg_defaults(switch_priv->regmap);
	}

	switch_priv->plug_state = false;
	switch_priv->charger_plugged = 0;
	switch_priv->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!switch_priv->tcpc_dev) {
		pr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		goto err_data;
	}

	switch_priv->pd_nb.notifier_call = typec_switch_usbc_event_changed;
	switch_priv->pd_nb.priority = 0;
	rc = register_tcp_dev_notifier(switch_priv->tcpc_dev, &switch_priv->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (rc < 0) {
		pr_err("%s: register tcpc notifer fail\n", __func__);
		goto err_data;
	}

	mutex_init(&switch_priv->notification_lock);

	dev_info(switch_priv->dev,"typec_switch: begin typec PD register action\n");
	/* add start for DP part */

	/* Setting MUX callback */
	mux_desc.drvdata = switch_priv;
	mux_desc.fwnode = dev->fwnode;
	mux_desc.set = typec_switch_mux_set;
#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
	switch_priv->mux = mtk_typec_mux_register(dev, &mux_desc);
#else
	switch_priv->mux = typec_switch_register(dev, &mux_desc);
#endif
	if (IS_ERR(switch_priv->mux)) {
		dev_err(dev, "error registering typec mux: %ld\n",
			PTR_ERR(switch_priv->mux));

		unregister_tcp_dev_notifier(switch_priv->tcpc_dev, &switch_priv->pd_nb, TCP_NOTIFY_TYPE_ALL);
		goto err_data;
	}

	/* add end for DP part */
	i2c_set_clientdata(i2c, switch_priv);

	INIT_WORK(&switch_priv->usbc_analog_work, typec_switch_usbc_analog_work_fn);

	switch_priv->typec_switch_notifier.rwsem =
		(struct rw_semaphore)__RWSEM_INITIALIZER
		((switch_priv->typec_switch_notifier).rwsem);
	switch_priv->typec_switch_notifier.head = NULL;

	//2020/11/04, support mic and ground switch to fix headset detect bug
	g_typec_switch_priv = switch_priv;

	//2021/12/02, check the headset pluging state when probe, fix headset detect bug
	INIT_DELAYED_WORK(&switch_priv->hp_work, hp_work_callback);
	schedule_delayed_work(&switch_priv->hp_work, msecs_to_jiffies(4000));

	dev_info(switch_priv->dev, "probed successfully!\n");

	return 0;

err_data:
	if (gpio_is_valid(switch_priv->hs_det_pin)) {
		gpio_free(switch_priv->hs_det_pin);
	}
	devm_kfree(&i2c->dev, switch_priv);
	dev_set_drvdata(&i2c->dev, NULL);

	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void typec_switch_remove(struct i2c_client *i2c)
#else
static int typec_switch_remove(struct i2c_client *i2c)
#endif
{
	struct typec_switch_priv *switch_priv =
			(struct typec_switch_priv *)i2c_get_clientdata(i2c);

	if (!switch_priv) {
		pr_err("%s, switch_priv is NULL!", __func__);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
		return;
#else
		return -EINVAL;
#endif
	}

	cancel_delayed_work(&switch_priv->hp_work);

	typec_switch_usbc_update_settings(switch_priv, 0x18, 0x98);
	cancel_work_sync(&switch_priv->usbc_analog_work);

	pm_relax(switch_priv->dev);
	mutex_destroy(&switch_priv->notification_lock);

#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
	mtk_typec_mux_unregister(switch_priv->mux);
#else
	typec_switch_unregister(switch_priv->mux);
#endif

	unregister_tcp_dev_notifier(switch_priv->tcpc_dev, &switch_priv->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (gpio_is_valid(switch_priv->hs_det_pin)) {
		gpio_free(switch_priv->hs_det_pin);
	}
	devm_kfree(&i2c->dev, switch_priv);

	dev_set_drvdata(&i2c->dev, NULL);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	return;
#else
	return 0;
#endif
}

static int typec_switch_pm_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct typec_switch_priv *switch_priv;

	if (!client) {
		pr_err("%s, client is NULL", __func__);
		return -EINVAL;
	}

	switch_priv = (struct typec_switch_priv *)i2c_get_clientdata(client);

	if (!switch_priv) {
		pr_err("switch_priv is null\n");
		return 0;
	}

	complete_all(&switch_priv->resume_ack);
	return 0;
}

static int typec_switch_pm_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct typec_switch_priv *switch_priv;

	if (!client) {
		pr_err("%s, client is NULL", __func__);
		return -EINVAL;
	}

	switch_priv = (struct typec_switch_priv *)i2c_get_clientdata(client);

	if (!switch_priv) {
		pr_err("switch_priv is null\n");
		return 0;
	}

	reinit_completion(&switch_priv->resume_ack);
	return 0;
}

static void typec_switch_shutdown(struct i2c_client *i2c) {
	struct typec_switch_priv *switch_priv;
	if (!i2c) {
		pr_err("%s, client is NULL", __func__);
		return;
	}

	switch_priv = (struct typec_switch_priv *)i2c_get_clientdata(i2c);

	pr_info("%s: recover all register while shutdown\n", __func__);

	if (switch_priv->vendor == DIO4480 || switch_priv->vendor == DIO4483) {
		typec_switch_write_register(switch_priv->regmap, DIO4483_REG_RESET, 0x01);//reset DIO4483
		return;
	}

	default_update_reg_defaults(switch_priv->regmap);

	return;
}

static const struct dev_pm_ops typec_switch_pm_ops = {
	.resume = typec_switch_pm_resume,
	.suspend = typec_switch_pm_suspend,
};

static const struct of_device_id typec_switch_i2c_dt_match[] = {
	{
		.compatible = "oplus,typec-switch-i2c",
	},
	{
		.compatible = "qcom,dio4480-i2c",
	},
	{
		.compatible = "qcom,dio4483-i2c",
	},
	{
		.compatible = "mtk,was4783-i2c",
	},
	{}
};

static const struct i2c_device_id typec_switch_i2c_id[] = {
	{ "typec_switch", 0 },
	{ "dio4480", 0 },
	{ "dio4483", 0 },
	{ "was4783", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, typec_switch_i2c_id);

static struct i2c_driver typec_switch_i2c_driver = {
	.driver = {
		.name = TYPEC_SWITCH_I2C_NAME,
		.of_match_table = typec_switch_i2c_dt_match,
		.pm = &typec_switch_pm_ops,
	},
	.probe = typec_switch_probe,
	.remove = typec_switch_remove,
	.shutdown = typec_switch_shutdown,
	.id_table = typec_switch_i2c_id,
};

static int __init typec_switch_init(void)
{
	int rc;

	pr_info("typec_switch: try to register I2C driver\n");

	rc = i2c_add_driver(&typec_switch_i2c_driver);
	if (rc) {
		pr_err("Failed to register I2C driver %s, rc = %d", VNAME(typec_switch_i2c_driver), rc);
	} else {
		pr_info("typec_switch: success to register I2C driver\n");
	}
	return rc;
}

static void __exit typec_switch_exit(void)
{
	i2c_del_driver(&typec_switch_i2c_driver);
}
module_init(typec_switch_init);
module_exit(typec_switch_exit);

MODULE_DESCRIPTION("TypeC Switch I2C driver");
MODULE_LICENSE("GPL v2");
