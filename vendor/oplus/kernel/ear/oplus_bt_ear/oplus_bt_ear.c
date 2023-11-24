// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

/*
 * EAR Power Switch Module
 * controls power to external Ear device
 * with interface to power management device
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include "oplus_bt_ear.h"
#include <linux/of_device.h>
#include "soc/oplus/system/oplus_project.h"
#include <linux/fs.h>
#define LOG_TAG "[oplus_bt_ear] %s line:%d "
#define debug(fmt, args...) printk(LOG_TAG fmt, __FUNCTION__, __LINE__, ##args)

static const char* const ear_pin_str[PIN_STATE_MAX] = {
    "ear_porst_low", "ear_porst_high", "ear_dvdd_disable", "ear_dvdd_enable", "ear_avdd_disable", "ear_avdd_enable",
};

static const struct of_device_id ear_power_match_table[] = {
    {.compatible = "zeku,b3",},
    {}
};
MODULE_DEVICE_TABLE(of, ear_power_match_table);

static struct earpower_platform_data* ear_power_pdata;
static int pwr_state;
static struct class* ear_class;
static int ear_power_major;
static bool probe_finished;

static int switch_pinctl_state(int state) {
    int rc = 0;
    pr_debug("%s: state: %d\n", __func__, state);
    if ((state < 0) || (state >= PIN_STATE_MAX)) {
        return -1;
    }

    if (!ear_power_pdata) {
        pr_err("%s: ear_power_pdata is null for state %d\n", __func__, state);
        return -1;
    }

    if (!IS_ERR(ear_power_pdata->pin_states[state])) {
        rc = pinctrl_select_state(ear_power_pdata->pinctrl,
                ear_power_pdata->pin_states[state]);
        if (rc) {
            pr_err("%s failed to select state %s return %d\n",
                __func__, ear_pin_str[state], rc);
            return rc;
        }
    }
    return rc;
}

static int handle_dvdd_1p8_in_supply(int on) {
    int rc = 0;

    pr_debug("%s: on: %d\n", __func__, on);

    if (!ear_power_pdata || !ear_power_pdata->dvdd_1p8_parent) {
        pr_err("%s: ear_power_pdata null for dvdd_1p8_parent \n", __func__);
        return -1;
    }

    rc = regulator_set_voltage(ear_power_pdata->dvdd_1p8_parent,
            1800000,
            3300000);
    if (rc < 0) {
        pr_err("%s: regulator_set_voltage(dvdd_1p8_parent) failed rc=%d\n",
            __func__, rc);
        goto out;
    }

    if (on == 1) {
        rc = regulator_enable(ear_power_pdata->dvdd_1p8_parent);
        if (rc < 0) {
            pr_err("%s: regulator_enable(dvdd_1p8_parent) failed. rc=%d\n",
                __func__, rc);
            goto out;
        }

    } else {
        rc = regulator_disable(ear_power_pdata->dvdd_1p8_parent);
        if (rc < 0) {
            pr_err("%s: regulator_diable(dvdd_1p8_parent) failed. rc=%d\n",
                __func__, rc);
            goto out;
        }

    }

out:
    return rc;

}


static int handle_avdd_1p8_in_supply(int on) {
    int rc = 0;

    pr_debug("%s: on: %d\n", __func__, on);

    if (!ear_power_pdata || !ear_power_pdata->avdd_1p8_parent) {
        pr_err("%s: ear_power_pdata null for avdd_1p8_parent \n", __func__);
        return -1;
    }

    rc = regulator_set_voltage(ear_power_pdata->avdd_1p8_parent,
            1800000,
            3300000);
    if (rc < 0) {
        pr_err("%s: regulator_set_voltage(avdd_1p8_parent) failed rc=%d\n",
            __func__, rc);
        goto out;
    }

    if (on == 1) {
        rc = regulator_enable(ear_power_pdata->avdd_1p8_parent);
        if (rc < 0) {
            pr_err("%s: regulator_enable(avdd_1p8_parent) failed. rc=%d\n",
                __func__, rc);
            goto out;
        }

    } else {
        rc = regulator_disable(ear_power_pdata->avdd_1p8_parent);
        if (rc < 0) {
            pr_err("%s: regulator_diable(avdd_1p8_parent) failed. rc=%d\n",
                __func__, rc);
            goto out;
        }
    }

out:
    return rc;

}

static int handle_dvdd_1p8(int on) {
    int rc = 0;
    pr_debug("%s: on: %d\n", __func__, on);

    if (!ear_power_pdata) {
        pr_err("%s: ear_power_pdata null\n", __func__);
        return -1;
    }

    if (on == 1) {
        if (handle_dvdd_1p8_in_supply(on)) {
            pr_err("%s: Unable to handle_dvdd_1p8_in_supply\n", __func__);
            return -1;
        }

        rc = switch_pinctl_state(PIN_STATE_EAR_DVDD_ENABLE);
        if (rc) {
            pr_err("%s: Unable to switch to PIN_STATE_EAR_DVDD_ENABLE \n", __func__);
        }
    } else if (on == 0) {
        rc = switch_pinctl_state(PIN_STATE_EAR_DVDD_DISABLE);
        if (rc) {
            pr_err("%s: Unable to switch to PIN_STATE_EAR_DVDD_DISABLE \n", __func__);
        }

        if (handle_dvdd_1p8_in_supply(on)) {
            pr_err("%s: Unable to handle_dvdd_1p8_in_supply\n", __func__);
        }
    }

    return rc;
}

static int handle_dvdd_0p6(int on) {
    int rc = 0;

    pr_debug("%s: on: %d\n", __func__, on);

    if (!ear_power_pdata || !ear_power_pdata->dvdd_0p6) {
        pr_err("%s: ear_power_pdata null for handle_dvdd_0p6 \n", __func__);
        return -1;
    }
    rc = regulator_set_voltage(ear_power_pdata->dvdd_0p6,
            600000,
            600000);
    if (rc < 0) {
        pr_err("%s: regulator_set_voltage(dvdd_0p6) failed rc=%d\n",
            __func__, rc);
        goto out;
    }

    if (on == 1) {
        rc = regulator_enable(ear_power_pdata->dvdd_0p6);
        if (rc < 0) {
            pr_err("%s: regulator_enable(dvdd_0p6) failed. rc=%d\n",
                __func__, rc);
            goto out;
        }

    } else {
        rc = regulator_disable(ear_power_pdata->dvdd_0p6);
        if (rc < 0) {
            pr_err("%s: regulator_diable(dvdd_0p6) failed. rc=%d\n",
                __func__, rc);
            goto out;
        }

    }


out:
    return rc;

}

static int handle_avdd_0p9(int on) {
    int rc = 0;

    pr_debug("%s: on: %d\n", __func__, on);

    if (!ear_power_pdata || !ear_power_pdata->avdd_0p9) {
        pr_err("%s: ear_power_pdata null for handle_avdd_0p9 \n", __func__);
        return -1;
    }
    rc = regulator_set_voltage(ear_power_pdata->avdd_0p9,
            900000,
            900000);
    if (rc < 0) {
        pr_err("%s: regulator_set_voltage(avdd_0p9) failed rc=%d\n",
            __func__, rc);
        goto out;
    }
    if (on == 1) {
        rc = regulator_enable(ear_power_pdata->avdd_0p9);
        if (rc < 0) {
            pr_err("%s: regulator_enable(avdd_0p9) failed. rc=%d\n",
                __func__, rc);
            goto out;
        }
    } else {

        rc = regulator_disable(ear_power_pdata->avdd_0p9);
        if (rc < 0) {
            pr_err("%s: regulator_disable(avdd_0p9) failed. rc=%d\n",
                __func__, rc);
            goto out;
        }

    }

out:
    return rc;

}

static int handle_avdd_1p8(int on) {
    int rc = 0;
    pr_debug("%s: on: %d\n", __func__, on);

    if (!ear_power_pdata) {
        pr_err("%s: ear_power_pdata null\n", __func__);
        return -1;
    }

    if (on == 1) {
        if (handle_avdd_1p8_in_supply(on)) {
            pr_err("%s: Unable to handle_avdd_1p8_in_supply\n", __func__);
            return -1;
        }

        rc = switch_pinctl_state(PIN_STATE_EAR_AVDD_ENABLE);
        if (rc) {
            pr_err("%s: Unable to switch to PIN_STATE_EAR_AVDD_ENABLE \n", __func__);
        }
    } else if (on == 0) {
        rc = switch_pinctl_state(PIN_STATE_EAR_AVDD_DISABLE);
        if (rc) {
            pr_err("%s: Unable to switch to PIN_STATE_EAR_AVDD_DISABLE \n", __func__);
        }

        if (handle_avdd_1p8_in_supply(on)) {
            pr_err("%s: Unable to handle_avdd_1p8_in_supply\n", __func__);
        }
    }

    return rc;
}

static int handle_dvdd_0p75(int on) {
    int rc = 0;

    pr_debug("%s: on: %d\n", __func__, on);

    if (!ear_power_pdata || !ear_power_pdata->dvdd_0p75) {
        pr_err("%s: ear_power_pdata null for dvdd_0p75 \n", __func__);
        return -1;
    }
    rc = regulator_set_voltage(ear_power_pdata->dvdd_0p75,
            750000,
            750000);
    if (rc < 0) {
        pr_err("%s: regulator_set_voltage(dvdd_0p75) failed rc=%d\n",
            __func__, rc);
        goto out;
    }

    if (on == 1) {
        rc = regulator_enable(ear_power_pdata->dvdd_0p75);
        if (rc < 0) {
            pr_err("%s: regulator_enable(dvdd_0p75) failed. rc=%d\n",
                __func__, rc);
        }
    } else {
        rc = regulator_disable(ear_power_pdata->dvdd_0p75);
        if (rc < 0) {
            pr_err("%s: regulator_disable(dvdd_0p75) failed. rc=%d\n",
                __func__, rc);
        }

    }
out:
    return rc;

}

static int handle_spdt(int on) {
    int rc = 0;

    pr_debug("%s: on: %d\n", __func__, on);

    if (!ear_power_pdata || !ear_power_pdata->spdt) {
        pr_err("%s: ear_power_pdata null for handle_spdt \n", __func__);
        return -1;
    }
    rc = regulator_set_voltage(ear_power_pdata->spdt,
            2800000,
            2800000);
    if (rc < 0) {
        pr_err("%s: regulator_set_voltage(spdt) failed rc=%d\n",
            __func__, rc);
        goto out;
    }

    if (on == 1) {
        rc = regulator_enable(ear_power_pdata->spdt);
        if (rc < 0) {
            pr_err("%s: regulator_enable(spdt) failed. rc=%d\n",
                __func__, rc);
            goto out;
        }

    } else {
        rc = regulator_disable(ear_power_pdata->spdt);
        if (rc < 0) {
            pr_err("%s: regulator_diable(spdt) failed. rc=%d\n",
                __func__, rc);
            goto out;
        }

    }

out:
    return rc;

}

static int handle_porrst(int on) {
    int rc = 0;
    pr_debug("%s: on: %d\n", __func__, on);

    if (!ear_power_pdata) {
        pr_err("%s: ear_power_pdata null\n", __func__);
        return -1;
    }

    if (on == 1) {
        rc = switch_pinctl_state(PIN_STATE_EAR_PORST_HIGH);
        if (rc) {
            pr_err("%s: Unable to switch to PIN_STATE_EAR_PORST_HIGH \n", __func__);
        }
    } else if (on == 0) {
        rc = switch_pinctl_state(PIN_STATE_EAR_PORST_LOW);
        if (rc) {
            pr_err("%s: Unable to switch to PIN_STATE_EAR_PORST_LOW \n", __func__);
        }
    }

    return rc;

}



static int ear_power(int on) {
    int rc = 0;
    pr_debug("%s: on: %d\n", __func__, on);
    if (on == 1) {
        rc = handle_dvdd_1p8(on);
        if (rc < 0) {
            pr_err("%s: ear_power handle_dvdd_1p8 config failed\n",
                __func__);
            goto dvdd_1p8_fail;
        }
        msleep(10);
        rc = handle_dvdd_0p6(on);

        if (rc < 0) {
            pr_err("%s: ear_power handle_dvdd_0p6 config failed\n",
                __func__);
            goto dvdd_0p6_fail;
        }
        msleep(10);

        rc = handle_avdd_0p9(on);

        if (rc < 0) {
            pr_err("%s: ear_power handle_avdd_0p9 config failed\n",
                __func__);
            goto avdd_0p9_fail;
        }
        msleep(10);

        rc = handle_avdd_1p8(on);

        if (rc < 0) {
            pr_err("%s: ear_power handle_avdd_1p8 config failed\n",
                __func__);
            goto avdd_1p8_fail;
        }
        msleep(10);

        rc = handle_dvdd_0p75(on);

        if (rc < 0) {
            pr_err("%s: ear_power handle_dvdd_0p75 config failed\n",
                __func__);
            goto dvdd_0p75_fail;
        }
        msleep(10);

        rc = handle_porrst(on);

        if (rc < 0) {
            pr_err("%s: ear_power handle_porrest config failed\n",
                __func__);
            goto porrst_fail;
        }
        msleep(5);
        pr_err("%s: ear_power sleep 10 after handle_porrst on\n",
            __func__);
    } else if (on == 0) {
        // Power Off
    porrst_fail:
        rc = handle_porrst(0);

        if (rc < 0) {
            pr_err("%s: ear_power handle_porrest config failed\n",
                __func__);
            goto porrst_fail;
        }

    dvdd_0p75_fail:
        rc = handle_dvdd_0p75(0);

        if (rc < 0) {
            pr_err("%s: ear_power handle_dvdd_0p75 config failed\n",
                __func__);
        }
    avdd_1p8_fail:
        rc = handle_avdd_1p8(0);

        if (rc < 0) {
            pr_err("%s: ear_power handle_avdd_1p8 config failed\n",
                __func__);
        }
    avdd_0p9_fail:
        rc = handle_avdd_0p9(0);
        if (rc < 0) {
            pr_err("%s: ear_power handle_avdd_0p9 config failed\n",
                __func__);
            goto avdd_0p9_fail;
        }

    dvdd_0p6_fail:
        rc = handle_dvdd_0p6(0);
        if (rc < 0) {
            pr_err("%s: ear_power handle_dvdd_0p6 config failed\n",
                __func__);
        }
    dvdd_1p8_fail:
        if (on == 0) {
            pr_err("%s: ear_power sleep 10 before dvdd1.8 power down\n",
                __func__);
            msleep(10);
        }
        rc = handle_dvdd_1p8(0);
        if (rc < 0) {
            pr_err("%s: ear_power handle_dvdd_1p8 config failed\n",
                __func__);
        }

    } else {
        pr_err("%s: Invalid power mode: %d\n", __func__, on);
        rc = -1;
    }
    return rc;
}


static int ear_power_vreg_init(struct platform_device* pdev) {
    int ret = 0;
    pr_debug("%s\n", __func__);

    ear_power_pdata->dvdd_0p6 = devm_regulator_get(&pdev->dev, "zeku,dvdd_0p6");
    if (IS_ERR(ear_power_pdata->dvdd_0p6)) {
        ret = PTR_ERR(ear_power_pdata->dvdd_0p6);
        ear_power_pdata->dvdd_0p6 = NULL;
        pr_warn("%s: failed to get dvdd_0p6 error:%d\n", __func__, ret);
        return ret;
    }

    pr_debug("after dvdd_0p6");
    ear_power_pdata->dvdd_0p75 = devm_regulator_get(&pdev->dev, "zeku,dvdd_0p75");
    if (IS_ERR(ear_power_pdata->dvdd_0p75)) {
        ret = PTR_ERR(ear_power_pdata->dvdd_0p75);
        ear_power_pdata->dvdd_0p75 = NULL;
        pr_warn("%s: failed to get dvdd_0p75 error:%d\n", __func__, ret);
        return ret;
    }

    pr_debug("after dvdd_0p75");

    ear_power_pdata->avdd_0p9 = devm_regulator_get(&pdev->dev, "zeku,avdd_0p9");
    if (IS_ERR(ear_power_pdata->avdd_0p9)) {
        ret = PTR_ERR(ear_power_pdata->avdd_0p9);
        ear_power_pdata->avdd_0p9 = NULL;
        pr_warn("%s: failed to get avdd_0p9 error:%d\n", __func__, ret);
        return ret;
    }
    pr_debug("after avdd_0p9");
    ear_power_pdata->avdd_1p8_parent = devm_regulator_get(&pdev->dev, "zeku,avdd_1p8_parent");
    if (IS_ERR(ear_power_pdata->avdd_1p8_parent)) {
        ret = PTR_ERR(ear_power_pdata->avdd_1p8_parent);
        ear_power_pdata->avdd_1p8_parent = NULL;
        pr_warn("%s: failed to get avdd_1p8_parent error:%d\n", __func__, ret);
        return ret;
    }
    pr_debug("after avdd_1p8_parent");
    ear_power_pdata->dvdd_1p8_parent = devm_regulator_get(&pdev->dev, "zeku,dvdd_1p8_parent");
    if (IS_ERR(ear_power_pdata->dvdd_1p8_parent)) {
        ret = PTR_ERR(ear_power_pdata->dvdd_1p8_parent);
        ear_power_pdata->dvdd_1p8_parent = NULL;
        pr_warn("%s: failed to get dvdd_1p8_parent error:%d\n", __func__, ret);
        return ret;
    }
    pr_debug("after dvdd_1p8_parent");
    if (get_PCB_Version() == EVT1) {
        ear_power_pdata->spdt = devm_regulator_get(&pdev->dev, "zeku,spdt");
        if (IS_ERR(ear_power_pdata->spdt)) {
            ret = PTR_ERR(ear_power_pdata->spdt);
            ear_power_pdata->spdt = NULL;
            pr_warn("%s: failed to get spdt error:%d\n", __func__, ret);
        } else {
            ret = handle_spdt(1);
            if (ret < 0) {
                pr_err("%s: ear_power_vreg_init  config spdt failed\n",
                    __func__);
            }
        }
        pr_debug("after spdt supply enable");
    } else {
        pr_debug("don`t need spdt control");
    }
    return ret;

}

static int ear_power_pinctrl_init(struct platform_device* pdev) {
    int rc = 0, i = 0;
    pr_debug("%s\n", __func__);
    if (!ear_power_pdata) {
        pr_err("%s: ear_power_pdata is NULL\n", __func__);
    }

    ear_power_pdata->pinctrl = devm_pinctrl_get(&(pdev->dev));
    if (IS_ERR_OR_NULL(ear_power_pdata->pinctrl)) {
        pr_warn("%s: devm_pinctrl_get failed\n", __func__);
        return -1;
    }

    for (i = 0; i < PIN_STATE_MAX; i++) {
        ear_power_pdata->pin_states[i] = pinctrl_lookup_state(ear_power_pdata->pinctrl,
                ear_pin_str[i]);
        if (IS_ERR(ear_power_pdata->pin_states[i])) {
            rc = PTR_ERR(ear_power_pdata->pin_states[i]);
            pr_err("%s Can't find pin state %s %d\n",
                __func__, ear_pin_str[i], rc);
            return rc;
        }
    }

    if (switch_pinctl_state(PIN_STATE_EAR_PORST_LOW) < 0) {
        return -1;
    }

    if (switch_pinctl_state(PIN_STATE_EAR_DVDD_DISABLE) < 0) {
        return -1;
    }

    if (switch_pinctl_state(PIN_STATE_EAR_AVDD_DISABLE) < 0) {
        return -1;
    }

    return rc;
}

static int ear_power_populate_dt_pinfo(struct platform_device* pdev) {
    int rc = 0;

    pr_debug("%s\n", __func__);

    if (!ear_power_pdata) {
        return -ENOMEM;
    }

    if (pdev->dev.of_node) {
        rc = ear_power_vreg_init(pdev);
        if (rc) {
            pr_err("after ear_power_vreg_get");
            return rc;
        }
        pr_err("after ear_power_vreg_get");
        rc = ear_power_pinctrl_init(pdev);
        if (rc) {
            pr_err("after ear_power_vreg_get");
            return rc;
        }
        pr_debug("after ear_gpio_dvdd");
    }
    return rc;
}

static int ear_power_probe(struct platform_device* pdev) {
    int rc = 0;

    pr_debug("%s\n", __func__);

    ear_power_pdata = kzalloc(sizeof(*ear_power_pdata), GFP_KERNEL);

    if (!ear_power_pdata) {
        return -ENOMEM;
    }

    ear_power_pdata->pdev = pdev;
    if (pdev->dev.of_node) {
        rc = ear_power_populate_dt_pinfo(pdev);
        if (rc < 0) {
            pr_err("%s, Failed to populate device tree info\n",
                __func__);
            goto free_pdata;
        }
        pdev->dev.platform_data = ear_power_pdata;
    } else {
        pr_err("%s: Failed to get platform data\n", __func__);
        goto free_pdata;
    }

    probe_finished = true;
    return 0;

free_pdata:
    kfree(ear_power_pdata);
    return rc;
}

static int ear_power_remove(struct platform_device* pdev) {
    dev_dbg(&pdev->dev, "%s\n", __func__);
    handle_spdt(0);
    probe_finished = false;
    kfree(ear_power_pdata);

    return 0;
}



static long ear_power_ioctl(struct file* file, unsigned int cmd, unsigned long arg) {
    int ret = 0, pwr_cntrl = 0;
    if (!ear_power_pdata || !probe_finished) {
        pr_err("%s: Ear Power Probing Pending.Try Again\n", __func__);
        return -EAGAIN;
    }

    switch (cmd) {
    case EAR_CMD_PWR_CTRL:
        pwr_cntrl = (int)arg;
        pr_err("%s: EAR_CMD_PWR_CTRL pwr_cntrl: %d\n",
            __func__, pwr_cntrl);
        if (pwr_state != pwr_cntrl) {
            ret = ear_power(pwr_cntrl);
            if (!ret) {
                pwr_state = pwr_cntrl;
            }
        } else {
            pr_err("%s: Ear chip state is already: %d no change\n",
                __func__, pwr_state);
            ret = 0;
        }
        break;

    default:
        return -ENOIOCTLCMD;
    }
    return ret;
}

static struct platform_driver ear_power_driver = {
    .probe = ear_power_probe,
    .remove = ear_power_remove,
    .driver = {
        .name = "oplus_ear",
        .owner = THIS_MODULE,
        .of_match_table = ear_power_match_table,
    },
};

static const struct file_operations ear_dev_fops = {
    .unlocked_ioctl = ear_power_ioctl,
    .compat_ioctl = ear_power_ioctl,
};

static int __init earpower_init(void) {
    int ret = 0;

    probe_finished = false;
    ret = platform_driver_register(&ear_power_driver);

    if (ret) {
        pr_err("%s: failed to register platform driver\n", __func__);
        goto driver_err;
    }

    ear_power_major = register_chrdev(0, "ear", &ear_dev_fops);
    if (ear_power_major < 0) {
        pr_err("%s: failed to allocate char dev\n", __func__);
        ret = -1;
        goto chrdev_err;
    }

    ear_class = class_create(THIS_MODULE, "ear-dev");
    if (IS_ERR(ear_class)) {
        pr_err("%s: coudn't create class\n", __func__);
        ret = -1;
        goto class_err;
    }

    if (device_create(ear_class, NULL, MKDEV(ear_power_major, 0),
            NULL, "btear") == NULL) {
        pr_err("%s: failed to create device\n", __func__);
        goto device_err;
    }
    pr_debug("%s: out \n", __func__);
    return 0;

device_err:
    class_destroy(ear_class);
class_err:
    unregister_chrdev(ear_power_major, "ear");
chrdev_err:
    platform_driver_unregister(&ear_power_driver);
driver_err:
    return ret;
}

static void __exit earpower_exit(void) {
    device_destroy(ear_class, MKDEV(ear_power_major, 0));
    class_destroy(ear_class);
    unregister_chrdev(ear_power_major, "ear");
    platform_driver_unregister(&ear_power_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Oplus Ear power control driver");


module_init(earpower_init);
module_exit(earpower_exit);
