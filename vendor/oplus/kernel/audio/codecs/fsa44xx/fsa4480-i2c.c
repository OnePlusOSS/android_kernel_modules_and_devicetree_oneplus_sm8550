// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 */

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

#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
#include "../../../../drivers/misc/mediatek/typec/mux/mux_switch.h"
#endif


#define OPLUS_ARCH_EXTENDS

//#ifdef OPLUS_ARCH_EXTENDS
#include <linux/regulator/consumer.h>
//#endif /*OPLUS_ARCH_EXTENDS*/

#include "fsa4480-i2c.h"
#include "../../../../drivers/misc/mediatek/typec/tcpc/inc/tcpm.h"

#define FSA4480_I2C_NAME	"fsa4480-driver"

#define HL5280_DEVICE_REG_VALUE 0x49

#define FSA4480_DEVICE_ID  0x00
#define FSA4480_SWITCH_SETTINGS 0x04
#define FSA4480_SWITCH_CONTROL  0x05
#define FSA4480_SWITCH_STATUS0  0x06
#define FSA4480_SWITCH_STATUS1  0x07
#define FSA4480_SLOW_L          0x08
#define FSA4480_SLOW_R          0x09
#define FSA4480_SLOW_MIC        0x0A
#define FSA4480_SLOW_SENSE      0x0B
#define FSA4480_SLOW_GND        0x0C
#define FSA4480_DELAY_L_R       0x0D
#define FSA4480_DELAY_L_MIC     0x0E
#define FSA4480_DELAY_L_SENSE   0x0F
#define FSA4480_DELAY_L_AGND    0x10
#define FSA4480_FUN_EN          0x12
#define FSA4480_JACK_STATUS     0x17
#define FSA4480_RESET           0x1E
#define FSA4480_MASK            0x30

#undef dev_dbg
#define dev_dbg dev_info

#ifndef OPLUS_ARCH_EXTENDS
#define OPLUS_ARCH_EXTENDS
#endif

#define USB_TYPEC_NORMAL  (0)
#define USB_TYPEC_REVERSE (1)

#ifdef OPLUS_ARCH_EXTENDS
// add for check and reset MT6338_TOP_INT_CON0 bit7 for headset can't be detected issue
extern bool mt6338_accdet_irq_check_and_set(void);
#endif /* OPLUS_ARCH_EXTENDS */

enum switch_vendor {
    FSA4480 = 0,
    HL5280,
    DIO4480
};

#ifdef OPLUS_ARCH_EXTENDS
static unsigned int debug_reg[32];
#endif /*OPLUS_ARCH_EXTENDS*/

static struct regulator *vio28_reg = NULL;

struct fsa4480_priv {
	struct regmap *regmap;
	struct device *dev;
	struct tcpc_device *tcpc_dev;
	struct notifier_block pd_nb;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct blocking_notifier_head fsa4480_notifier;
	struct mutex notification_lock;
	unsigned int hs_det_pin;
	//#ifdef OPLUS_ARCH_EXTENDS
	int hs_det_level;
	//#endif /*OPLUS_ARCH_EXTENDS*/
	enum switch_vendor vendor;
	bool plug_state;
	bool b_dynamic_sense_to_ground;
	/* add start for DP */
	struct typec_mux *mux;
	/* add end for DP */
#ifdef OPLUS_ARCH_EXTENDS
	struct delayed_work hp_work;
#endif
};

struct fsa4480_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config fsa4480_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FSA4480_MASK,
};

static const struct fsa4480_reg_val fsa_reg_i2c_defaults[] = {
	{FSA4480_SWITCH_CONTROL, 0x18},
	{FSA4480_SLOW_L, 0x00},
	{FSA4480_SLOW_R, 0x00},
	{FSA4480_SLOW_MIC, 0x00},
	{FSA4480_SLOW_SENSE, 0x00},
	{FSA4480_SLOW_GND, 0x00},
	{FSA4480_DELAY_L_R, 0x00},
	{FSA4480_DELAY_L_MIC, 0x00},
	{FSA4480_DELAY_L_SENSE, 0x00},
	{FSA4480_DELAY_L_AGND, 0x09},
	{FSA4480_SWITCH_SETTINGS, 0x98},
};

#ifdef OPLUS_ARCH_EXTENDS
struct fsa4480_priv *g_fsa_priv = NULL;
static int dio_status = 0;
#endif

static void fsa4480_usbc_update_settings(struct fsa4480_priv *fsa_priv,
		u32 switch_control, u32 switch_enable)
{
	if (!fsa_priv->regmap) {
		dev_info(fsa_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

    if (fsa_priv->vendor == DIO4480) {
        return;
    }
	
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x80);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, switch_control);
	/* FSA4480 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, switch_enable);
}

static int fsa4480_usbc_event_changed(struct notifier_block *nb,
				      unsigned long evt, void *ptr)
{
	struct fsa4480_priv *fsa_priv =
			container_of(nb, struct fsa4480_priv, pd_nb);
	struct device *dev;
	struct tcp_notify *noti = ptr;

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	if (fsa_priv->vendor == HL5280) {
		dev_info(dev, "%s: switch chip is HL5280\n", __func__);
	}

	dev_info(dev, "%s: typeC event: %d plug_state: %d\n", __func__, evt, fsa_priv->plug_state);

	switch (evt) {
	case TCP_NOTIFY_TYPEC_STATE:
		dev_info(dev, "%s: old_state: %d, new_state: %d\n",
			__func__, noti->typec_state.old_state, noti->typec_state.new_state);
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO) {
			/* AUDIO plug in */
			dev_info(dev, "%s: audio plug in\n", __func__);
			fsa_priv->plug_state = true;
			pm_stay_awake(fsa_priv->dev);
			cancel_work_sync(&fsa_priv->usbc_analog_work);
			schedule_work(&fsa_priv->usbc_analog_work);
		} else if (fsa_priv->plug_state == true
			&& noti->typec_state.new_state == TYPEC_UNATTACHED) {
			/* AUDIO plug out */
			dev_info(dev, "%s: audio plug out\n", __func__);
			fsa_priv->plug_state = false;
			pm_stay_awake(fsa_priv->dev);
			cancel_work_sync(&fsa_priv->usbc_analog_work);
			schedule_work(&fsa_priv->usbc_analog_work);
		}
		break;
	case TCP_NOTIFY_PLUG_OUT:
		dev_info(dev, "%s: typec plug out\n", __func__);
		break;
	default:
		break;
	};

	return NOTIFY_OK;
}

static int fsa4480_usbc_analog_setup_switches(struct fsa4480_priv *fsa_priv)
{
	int rc = 0;
	struct device *dev;
	unsigned int switch_status = 0;
	unsigned int jack_status = 0;
	int state;
	int i = 0;

	if (!fsa_priv)
		return -EINVAL;
	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	mutex_lock(&fsa_priv->notification_lock);

	dev_info(dev, "%s: plug_state %d\n", __func__, fsa_priv->plug_state);

	for (i = 0 ;i <= 0x1f ;i++) {
		regmap_read(fsa_priv->regmap, i, &debug_reg[i]);
	}

	dev_info(dev,"%s dump reg:0x00:%02x,0x01:%02x,0x02:%02x,0x03:%02x,0x04:%02x,0x05:%02x,0x06:%02x,0x07:%02x\n",__func__,debug_reg[0],debug_reg[1],debug_reg[2],debug_reg[3],debug_reg[4],debug_reg[5],debug_reg[6],debug_reg[7]);
	dev_info(dev,"%s dump reg:0x08:%02x,0x09:%02x,0x0A:%02x,0x0B:%02x,0x0C:%02x,0x0D:%02x,0x0E:%02x,0x0F:%02x\n",__func__,debug_reg[8],debug_reg[9],debug_reg[10],debug_reg[11],debug_reg[12],debug_reg[13],debug_reg[14],debug_reg[15]);
	dev_info(dev,"%s dump reg:0x10:%02x,0x11:%02x,0x12:%02x,0x13:%02x,0x14:%02x,0x15:%02x,0x16:%02x,0x17:%02x\n ",__func__,debug_reg[16],debug_reg[17],debug_reg[18],debug_reg[19],debug_reg[20],debug_reg[21],debug_reg[22],debug_reg[23]);
	dev_info(dev,"%s dump reg:0x18:%02x,0x19:%02x,0x1A:%02x,0x1B:%02x,0x1C:%02x,0x1D:%02x,0x1E:%02x,0x1F:%02x\n",__func__,debug_reg[24],debug_reg[25],debug_reg[26],debug_reg[27],debug_reg[28],debug_reg[29],debug_reg[30],debug_reg[31]);

	if (fsa_priv->plug_state) {
		pr_info("plugin regulator_get_voltage(%d)\n", regulator_get_voltage(vio28_reg));
		if (fsa_priv->vendor == DIO4480) {
			/* activate switches */
			regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);//reset DIO4480
			usleep_range(1000, 1005);
			regmap_write(fsa_priv->regmap, FSA4480_FUN_EN, 0x45);

			for (i = 0;i < 100 ;i++) {
				usleep_range(10*1000, 10*1005);
				regmap_read(fsa_priv->regmap, 0x18, &switch_status);
				if (switch_status & 0x4) {
					dev_info(dev, "%s: Audio jack detection and configuration has occurred.\n", __func__);
					i = 100;
				}
			}

			dev_info(dev, "%s: set reg[0x%x] done.\n", __func__, FSA4480_FUN_EN);
			dio_status = 0;

		} else {
			/* activate switches */
			fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
			usleep_range(1000, 1005);
			regmap_write(fsa_priv->regmap, FSA4480_FUN_EN, 0x45);
			usleep_range(4000, 4005);
			dev_info(dev, "%s: set reg[0x%x] done.\n", __func__, FSA4480_FUN_EN);
		}

		regmap_read(fsa_priv->regmap, FSA4480_JACK_STATUS, &jack_status);
		dev_info(dev, "%s: reg[0x%x]=0x%x.\n", __func__, FSA4480_JACK_STATUS, jack_status);
		if ((jack_status & 0x2)&&(fsa_priv->vendor != DIO4480)) {
			//for 3 pole, mic switch to SBU2
			dev_info(dev, "%s: set mic to sbu2 for 3 pole.\n", __func__);
			fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
			usleep_range(4000, 4005);
		}

		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS0, &switch_status);
		dev_info(dev, "%s: reg[0x%x]=0x%x.\n", __func__, FSA4480_SWITCH_STATUS0, switch_status);
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);
		dev_info(dev, "%s: reg[0x%x]=0x%x.\n", __func__, FSA4480_SWITCH_STATUS1, switch_status);

		if (fsa_priv->vendor == DIO4480) {
			regmap_read(fsa_priv->regmap, FSA4480_JACK_STATUS, &switch_status);
			dev_info(dev, "%s: reg[0x17]=0x%x.\n", __func__, switch_status);

			if (switch_status == 0x01) {
				dev_info(dev, "%s: error status,swap MIC_GND\n", __func__);
				regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);//reset DIO4480
				usleep_range(1000, 1005);
				regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, 0x00);//GND - GSBU1, MIC - SBU2
				regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9f);
				usleep_range(10000, 10005);
				dio_status = 1;
			}

		}
		if (gpio_is_valid(fsa_priv->hs_det_pin)) {
			// Check the bit 7: audio irq enable
			if (!mt6338_accdet_irq_check_and_set()) {
				dev_err(dev, "%s: MT6338_TOP_INT_CON0 bit7 is 0.\n", __func__);
			}
			usleep_range(2000, 2005);

			dev_info(dev, "%s: set hs_det_pin to enable.\n", __func__);
			state = gpio_get_value(fsa_priv->hs_det_pin);
			dev_info(dev, "%s: before hs_det_pin state = %d.\n", __func__, state);
			usleep_range(1000, 1005);
			//dev_info(dev, "%s: panzhao test sleep 300ms\n", __func__);
			gpio_direction_output(fsa_priv->hs_det_pin, fsa_priv->hs_det_level);
			state = gpio_get_value(fsa_priv->hs_det_pin);
			dev_info(dev, "%s: after hs_det_pin state = %d.\n", __func__, state);
		}
	} else {
		pr_info("plugout regulator_get_voltage(%d)\n", regulator_get_voltage(vio28_reg));
		if (gpio_is_valid(fsa_priv->hs_det_pin)) {
			// Check the bit 7: audio irq enable
			if (!mt6338_accdet_irq_check_and_set()) {
				dev_err(dev, "%s: MT6338_TOP_INT_CON0 bit7 is 0.\n", __func__);
			}
			usleep_range(2000, 2005);

			dev_info(dev, "%s: set hs_det_pin to disable.\n", __func__);
			state = gpio_get_value(fsa_priv->hs_det_pin);
			dev_info(dev, "%s: before hs_det_pin state = %d.\n", __func__, state);
			gpio_direction_output(fsa_priv->hs_det_pin, !fsa_priv->hs_det_level);
			state = gpio_get_value(fsa_priv->hs_det_pin);
			dev_info(dev, "%s: after hs_det_pin state = %d.\n", __func__, state);
		}

		if (fsa_priv->vendor == DIO4480) {
			regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);//reset DIO4480
			usleep_range(1000, 1005);
			regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, 0x18);
			regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x98);
			dev_info(dev, "%s: plugout. set to usb mode\n", __func__);
		} else {
			/* deactivate switches */
			fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
			//#ifdef OPLUS_ARCH_EXTENDS
			//regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9D);
			//#endif /*OPLUS_ARCH_EXTENDS*/
		}
	}

	mutex_unlock(&fsa_priv->notification_lock);
	return rc;
}

/*
 * fsa4480_reg_notifier - register notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on success, or error code
 */
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	rc = blocking_notifier_chain_register
				(&fsa_priv->fsa4480_notifier, nb);
	if (rc)
		return rc;

	/*
	 * as part of the init sequence check if there is a connected
	 * USB C analog adapter
	 */
	dev_dbg(fsa_priv->dev, "%s: verify if USB adapter is already inserted\n",
		__func__);
	rc = fsa4480_usbc_analog_setup_switches(fsa_priv);

	return rc;
}
EXPORT_SYMBOL(fsa4480_reg_notifier);

/*
 * fsa4480_unreg_notifier - unregister notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on pass, or error code
 */
int fsa4480_unreg_notifier(struct notifier_block *nb,
			     struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	return blocking_notifier_chain_unregister
					(&fsa_priv->fsa4480_notifier, nb);
}
EXPORT_SYMBOL(fsa4480_unreg_notifier);

static int fsa4480_validate_display_port_settings(struct fsa4480_priv *fsa_priv)
{
	u32 switch_status = 0;

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		pr_info("AUX SBU1/2 switch status is invalid = %u\n",
				switch_status);
		return -EIO;
	}

	return 0;
}
/*
 * fsa4480_switch_event - configure FSA switch position based on event
 *
 * @node - phandle node to fsa4480 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event)
{
	int switch_control = 0;
#ifndef OPLUS_ARCH_EXTENDS
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;
#else
	struct fsa4480_priv *fsa_priv = g_fsa_priv;
	if (!fsa_priv) {
		pr_info("fsa_priv is NULL\n");
		return -EINVAL;
	}
#endif
	if (!fsa_priv->regmap)
		return -EINVAL;

	pr_info("%s - switch event: %d\n", __func__, event);
	if(fsa_priv->vendor != DIO4480) {
		return -EINVAL;
	}

	switch (event) {
	case FSA_MIC_GND_SWAP:
		if (fsa_priv->vendor == DIO4480) {
			if (dio_status) {
				pr_info("%s - switch event: %d delay 0ms\n", __func__, event);
				regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);
				usleep_range(1000, 1005);
				regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, 0x07); //GND - GSBU2, MIC - SBU1
				regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9f);
				dio_status = 0;
			}
		} else {
			regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL,
					&switch_control);
			if ((switch_control & 0x07) == 0x07)
				switch_control = 0x0;
			else
				switch_control = 0x7;
		}
		fsa4480_usbc_update_settings(fsa_priv, switch_control, 0x9F);
		break;
	case FSA_USBC_ORIENTATION_CC1:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_ORIENTATION_CC2:
		fsa4480_usbc_update_settings(fsa_priv, 0x78, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_DISPLAYPORT_DISCONNECTED:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_event);

#ifdef OPLUS_ARCH_EXTENDS
int fsa4480_sense_to_ground(bool bstate)
{
    struct fsa4480_priv *fsa_priv = g_fsa_priv;
	if (!fsa_priv) {
		pr_info("fsa_priv is NULL\n");
		return -EINVAL;
	}
    
    if (!fsa_priv->b_dynamic_sense_to_ground) {
        pr_info("Not support dynamic sense to ground\n");
        return 0;
	}

	if (fsa_priv->vendor != FSA4480) {
		pr_info("Chip is not FSA4480, not support sense to ground\n");
		return 0;
	}

	if (fsa_priv->plug_state) {
		pr_info("Headset is plug in, do nothing\n");
		return 0;
	}

	pr_info("%s: bstate = %d", __func__, bstate);

	if (bstate) {
		regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9D);
	} else {
		regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x98);
	}

	return 0;
}

EXPORT_SYMBOL(fsa4480_sense_to_ground);
#endif

static int fsa4480_parse_dt(struct fsa4480_priv *fsa_priv,
	struct device *dev)
{
    struct device_node *dNode = dev->of_node;
    int ret = 0;
    int hs_det_level = 0;
    int state = 0;
    int sense_to_ground = 0;

//#ifdef OPLUS_ARCH_EXTENDS
    vio28_reg = regulator_get(dev, "fsa_audio");

    if(IS_ERR_OR_NULL(vio28_reg))
    {
        pr_info("%s: VIO28_reg is NULL\n", __func__);
    }

    /*set 3v*/
    ret = regulator_set_voltage(vio28_reg, 3000000, 3000000);
    if(ret)
   {
      pr_info("regulator_set_voltage(%d) failed!\n", ret);
   }
    
	pr_info("regulator_get_voltage(%d)\n", regulator_get_voltage(vio28_reg));
//#endif /*OPLUS_ARCH_EXTENDS*/

    if (dNode == NULL) {
        return -ENODEV;
    }

	if (!fsa_priv) {
		pr_info("%s: fsa_priv is NULL\n", __func__);
		return -ENOMEM;
	}

	fsa_priv->hs_det_pin = of_get_named_gpio(dNode,
	        "fsa4480,hs-det-gpio", 0);
	if (!gpio_is_valid(fsa_priv->hs_det_pin)) {
	    pr_info("%s: hs-det-gpio in dt node is missing\n", __func__);
	    return -ENODEV;
	}

	//#ifdef OPLUS_ARCH_EXTENDS
	ret = of_property_read_u32(dNode,
			"fsa4480,hs-det-level", &hs_det_level);
	if (ret) {
		pr_info("%s: hs-det-level request fail\n", __func__);
		fsa_priv->hs_det_level = 0;
	} else {
		fsa_priv->hs_det_level = hs_det_level;
	}
	//#endif /*OPLUS_ARCH_EXTENDS*/

	//#ifdef OPLUS_ARCH_EXTENDS
	ret = of_property_read_u32(dNode,
			"fsa4480,dynamic-sense-to-gnd", &sense_to_ground);
	if (ret) {
		pr_info("%s: read prop sense-to-gnd fail\n", __func__);
		fsa_priv->b_dynamic_sense_to_ground = false;
	} else {
		fsa_priv->b_dynamic_sense_to_ground = sense_to_ground;
	}
	//#endif /*OPLUS_ARCH_EXTENDS*/

	ret = gpio_request(fsa_priv->hs_det_pin, "fsa4480_hs_det");
	if (ret) {
		pr_info("%s: hs-det-gpio request fail\n", __func__);
		return ret;
	}
	dev_info(dev, "%s: hs_det_pin gpio = %d.\n", __func__, fsa_priv->hs_det_pin);

	//#ifdef OPLUS_ARCH_EXTENDS
	gpio_direction_output(fsa_priv->hs_det_pin, !fsa_priv->hs_det_level);
	//#else /*OPLUS_ARCH_EXTENDS*/
	//gpio_direction_output(fsa_priv->hs_det_pin, 1);
	//#endif /*OPLUS_ARCH_EXTENDS*/

	state = gpio_get_value(fsa_priv->hs_det_pin);
	dev_info(dev, "%s: init hs_det_pin state = %d.\n", __func__, state);

	return ret;
}

static void fsa4480_usbc_analog_work_fn(struct work_struct *work)
{
	struct fsa4480_priv *fsa_priv =
		container_of(work, struct fsa4480_priv, usbc_analog_work);

	if (!fsa_priv) {
		pr_info("%s: fsa container invalid\n", __func__);
		return;
	}
	fsa4480_usbc_analog_setup_switches(fsa_priv);
	pm_relax(fsa_priv->dev);
}

static void fsa4480_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
		regmap_write(regmap, fsa_reg_i2c_defaults[i].reg,
				   fsa_reg_i2c_defaults[i].val);
}

/* add start for DP */
static int fsa4480_mux_set(struct typec_mux *mux, struct typec_mux_state *state)
{
	struct fsa4480_priv *fsa_priv = typec_mux_get_drvdata(mux);
	struct tcp_notify *data = state->data;
	int ret = 0;
	//enum typec_orientation orientation;
	int orientation;
	unsigned int switch_status0 = 0;
	unsigned int switch_status1 = 0;

	/* Debug Message
	 dev_info(fsa_priv->dev, "fsa4480_mux_set\n");
	 dev_info(fsa_priv->dev, "state->mode : %d\n", state->mode);
	 dev_info(fsa_priv->dev, "data-> polarity : %d\n", data->ama_dp_state.polarity);
	 dev_info(fsa_priv->dev, "data-> signal : %d\n", data->ama_dp_state.signal);
	 dev_info(fsa_priv->dev, "data-> pin_assignment : %d\n", data->ama_dp_state.pin_assignment);
	 dev_info(fsa_priv->dev, "data-> active : %d\n", data->ama_dp_state.active);
	 */

	if (state->mode == TCP_NOTIFY_AMA_DP_STATE) {
		dev_info(fsa_priv->dev, "%s DP state = %d\n", __func__, data->ama_dp_state.polarity);
		orientation = data->ama_dp_state.polarity;
		switch (orientation) {
		case USB_TYPEC_NORMAL:
			/* switch cc1 side */
			fsa4480_usbc_update_settings(fsa_priv, 0x18, 0xF8);
			return fsa4480_validate_display_port_settings(fsa_priv);
		case USB_TYPEC_REVERSE:
			fsa4480_usbc_update_settings(fsa_priv, 0x78, 0xF8);
			return fsa4480_validate_display_port_settings(fsa_priv);
			break;
		default:
			break;
		}
	} else if (state->mode == TCP_NOTIFY_TYPEC_STATE) {
		if ((data->typec_state.old_state == TYPEC_ATTACHED_SRC
			|| data->typec_state.old_state == TYPEC_ATTACHED_SNK)
			&& data->typec_state.new_state == TYPEC_UNATTACHED) {
				regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS0, &switch_status0);
				regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status1);
				dev_info(fsa_priv->dev, "%s: reg[0x%x]=0x%x, reg[0x%x]=0x%x.\n", __func__, FSA4480_SWITCH_STATUS0,
					switch_status0, FSA4480_SWITCH_STATUS1, switch_status1);
				if (((switch_status0&0x7F) == 0x05)&& ((switch_status1&0x3F) == 0x00)) {
					dev_info(fsa_priv->dev, "switch already be usb state.\n");
					return ret;
				}
				fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
				dev_info(fsa_priv->dev, "Plug out, switch set to usb.\n");
			}
	}
	return ret;
}
/* add end for DP */

#ifdef OPLUS_ARCH_EXTENDS
static void hp_work_callback(struct work_struct *work)
{
	struct fsa4480_priv *fsa_priv = g_fsa_priv;

	if (fsa_priv->plug_state == true) {
		return;
	}

	if (tcpm_inquire_typec_attach_state(fsa_priv->tcpc_dev) == TYPEC_ATTACHED_AUDIO) {
		pr_info("%s: TYPEC_ATTACHED_AUDIO is inserted\n", __func__);
		fsa_priv->plug_state = true;
		pm_stay_awake(fsa_priv->dev);
		cancel_work_sync(&fsa_priv->usbc_analog_work);
		schedule_work(&fsa_priv->usbc_analog_work);
	} else {
		pr_info("%s: TYPEC_ATTACHED_AUDIO is not inserted\n", __func__);
	}
}
#endif

static int fsa4480_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct fsa4480_priv *fsa_priv;
	int rc = 0;
	unsigned int reg_value = 0;
	/* add start for DP */
	struct device *dev = &i2c->dev;
	struct typec_mux_desc mux_desc;
        /* add end for DP */
	#ifdef OPLUS_ARCH_EXTENDS
	int i = 0;
	#endif
	int ret = 0;

	fsa_priv = devm_kzalloc(&i2c->dev, sizeof(*fsa_priv),
				GFP_KERNEL);
	if (!fsa_priv)
		return -ENOMEM;

	fsa_priv->dev = &i2c->dev;

	fsa4480_parse_dt(fsa_priv, &i2c->dev);

	fsa_priv->regmap = devm_regmap_init_i2c(i2c, &fsa4480_regmap_config);
	if (IS_ERR_OR_NULL(fsa_priv->regmap)) {
		dev_info(fsa_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!fsa_priv->regmap) {
			rc = -EINVAL;
			goto err_data;
		}
		rc = PTR_ERR(fsa_priv->regmap);
		goto err_data;
	}


	ret = regmap_read(fsa_priv->regmap, FSA4480_DEVICE_ID, &reg_value);
	dev_info(fsa_priv->dev, "%s: device id reg value: 0x%x\n", __func__, reg_value);
        #ifdef OPLUS_ARCH_EXTENDS
	//for (i = 0 ; i < 5 ; i++){
	//	pr_err("%s: device id reg value: 0x%x\n", __func__, reg_value);
	//	regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);//reset DIO4480
	//}
	for (i = 0 ; i < 4 && (ret < 0); i++){
		ret = regmap_read(fsa_priv->regmap, FSA4480_DEVICE_ID, &reg_value);//reset DIO4480
		pr_err("%s: %d read, device id reg value: 0x%x\n", __func__, i, reg_value);
	}
	#endif
	if (HL5280_DEVICE_REG_VALUE == reg_value) {
		dev_info(fsa_priv->dev, "%s: switch chip is HL5280\n", __func__);
		fsa_priv->vendor = HL5280;
        } else if (0xF1 == reg_value) {
		dev_info(fsa_priv->dev, "%s: switch chip is DIO4480\n", __func__);
		fsa_priv->vendor = DIO4480;
	} else {
		dev_info(fsa_priv->dev, "%s: switch chip is FSA4480\n", __func__);
		fsa_priv->vendor = FSA4480;
	}

    if (fsa_priv->vendor != DIO4480) {
	    fsa4480_update_reg_defaults(fsa_priv->regmap);
		//#ifdef OPLUS_ARCH_EXTENDS
		//regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9D);
		//#endif /*OPLUS_ARCH_EXTENDS*/

	}

	if (fsa_priv->vendor == DIO4480) {
		regmap_write(fsa_priv->regmap, 0x1e, 0x01);//reset DIO4480
		usleep_range(1*1000, 1*1005);
	}

	fsa_priv->plug_state = false;
	fsa_priv->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!fsa_priv->tcpc_dev) {
		pr_info("%s get tcpc device type_c_port0 fail\n", __func__);
		goto err_data;
	}

	fsa_priv->pd_nb.notifier_call = fsa4480_usbc_event_changed;
	fsa_priv->pd_nb.priority = 0;
	rc = register_tcp_dev_notifier(fsa_priv->tcpc_dev, &fsa_priv->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (rc < 0) {
		pr_info("%s: register tcpc notifer fail\n", __func__);
		goto err_data;
	}

	mutex_init(&fsa_priv->notification_lock);

	dev_info(fsa_priv->dev,"fsa4480: begin typec PD register action\n");
	/* add start for DP part */

	/* Setting MUX callback */
	mux_desc.drvdata = fsa_priv;
	mux_desc.fwnode = dev->fwnode;
	mux_desc.set = fsa4480_mux_set;
#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
	fsa_priv->mux = mtk_typec_mux_register(dev, &mux_desc);
#else
	fsa_priv->mux = typec_switch_register(dev, &mux_desc);
#endif
	if (IS_ERR(fsa_priv->mux)) {
		dev_info(dev, "error registering typec mux: %ld\n",
			PTR_ERR(fsa_priv->mux));

		mutex_destroy(&fsa_priv->notification_lock);

		unregister_tcp_dev_notifier(fsa_priv->tcpc_dev, &fsa_priv->pd_nb, TCP_NOTIFY_TYPE_ALL);

		goto err_data;
	}
	/* add end for DP part */
	i2c_set_clientdata(i2c, fsa_priv);

	INIT_WORK(&fsa_priv->usbc_analog_work,
		  fsa4480_usbc_analog_work_fn);

	fsa_priv->fsa4480_notifier.rwsem =
		(struct rw_semaphore)__RWSEM_INITIALIZER
		((fsa_priv->fsa4480_notifier).rwsem);
	fsa_priv->fsa4480_notifier.head = NULL;
#ifdef OPLUS_ARCH_EXTENDS
	g_fsa_priv = fsa_priv;
#endif
#ifdef OPLUS_ARCH_EXTENDS
	INIT_DELAYED_WORK(&fsa_priv->hp_work, hp_work_callback);
	schedule_delayed_work(&fsa_priv->hp_work, msecs_to_jiffies(2000));
#endif
	dev_info(fsa_priv->dev,"fsa4480: probe ok\n");

	return 0;

err_data:
	if (gpio_is_valid(fsa_priv->hs_det_pin)) {
		gpio_free(fsa_priv->hs_det_pin);
	}
	devm_kfree(&i2c->dev, fsa_priv);
	return rc;
}

static int fsa4480_remove(struct i2c_client *i2c)
{
	struct fsa4480_priv *fsa_priv =
			(struct fsa4480_priv *)i2c_get_clientdata(i2c);

	if (!fsa_priv)
		return -EINVAL;

	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	cancel_work_sync(&fsa_priv->usbc_analog_work);
	pm_relax(fsa_priv->dev);
	mutex_destroy(&fsa_priv->notification_lock);

#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
	mtk_typec_mux_unregister(fsa_priv->mux);
#else
	typec_switch_unregister(fsa_priv->mux);
#endif

#ifdef OPLUS_ARCH_EXTENDS
	unregister_tcp_dev_notifier(fsa_priv->tcpc_dev, &fsa_priv->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (gpio_is_valid(fsa_priv->hs_det_pin)) {
		gpio_free(fsa_priv->hs_det_pin);
	}
	devm_kfree(&i2c->dev, fsa_priv);
#endif

	dev_set_drvdata(&i2c->dev, NULL);
	return 0;
}

static void fsa4480_shutdown(struct i2c_client *i2c) {
	struct fsa4480_priv *fsa_priv =
			(struct fsa4480_priv *)i2c_get_clientdata(i2c);

	if (!fsa_priv) {
		return;
	}

	pr_info("%s: recover all register while shutdown\n", __func__);

	if (fsa_priv->vendor == DIO4480) {
		regmap_write(fsa_priv->regmap, 0x1e, 0x01);//reset DIO4480
		return;
	}

	fsa4480_update_reg_defaults(fsa_priv->regmap);

	return;
}

static const struct of_device_id fsa4480_i2c_dt_match[] = {
	{
		.compatible = "qcom,fsa4480-i2c",
	},
	{}
};

static const struct i2c_device_id fsa4480_i2c_id[] = {
	{ "fsa4480", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fsa4480_i2c_id);

static struct i2c_driver fsa4480_i2c_driver = {
	.driver = {
		.name = FSA4480_I2C_NAME,
		.of_match_table = fsa4480_i2c_dt_match,
	},
	.probe = fsa4480_probe,
	.remove = fsa4480_remove,
	.shutdown = fsa4480_shutdown,
	.id_table = fsa4480_i2c_id,
};

static int __init fsa4480_init(void)
{
	int rc;
	
	pr_info("fsa4480: try to register I2C driver\n");

	rc = i2c_add_driver(&fsa4480_i2c_driver);
	if (rc)
		pr_info("fsa4480: Failed to register I2C driver: %d\n", rc);
	else
		pr_info("fsa4480: success to register I2C driver\n");
	return rc;
}
//late_initcall_sync(fsa4480_init);

static void __exit fsa4480_exit(void)
{
	i2c_del_driver(&fsa4480_i2c_driver);
}
module_init(fsa4480_init);
module_exit(fsa4480_exit);

MODULE_DESCRIPTION("FSA4480 I2C driver");
MODULE_LICENSE("GPL v2");
