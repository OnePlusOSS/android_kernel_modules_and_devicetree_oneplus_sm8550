/******************************************************************************
 ** Copyright (C), 2022-2022, Oplus Mobile Comm Corp., Ltd
 ** File: - oplus_wifismartantenna.c
 ** Description: wifismartantenna.c (wsa)
 **
 ** Version: 1.0
 ** Date : 2020/08/14
 ** TAG: OPLUS_FEATURE_WIFI_SMARTANTENNA
 ** ------------------------------- Revision History: ----------------------------
 ** <author>                                <data>        <version>       <desc>
 ** ------------------------------------------------------------------------------
 *******************************************************************************/

#include <linux/types.h>
#include <linux/ip.h>
#include <linux/netfilter.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <linux/sysctl.h>
#include <net/route.h>
#include <net/ip.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/version.h>
#include <net/tcp.h>
#include <linux/random.h>
#include <net/sock.h>
#include <net/dst.h>
#include <linux/file.h>
#include <net/tcp_states.h>
#include <net/sch_generic.h>
#include <net/pkt_sched.h>
#include <net/netfilter/nf_queue.h>
#include <linux/netfilter/xt_state.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_owner.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/ipv4/nf_conntrack_ipv4.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <net/genetlink.h>
#include <soc/oplus/system/oplus_project.h>

#define LOG_TAG "[oplus_wsa] %s line:%d "
#define debug(fmt, args...) printk(LOG_TAG fmt, __FUNCTION__, __LINE__, ##args)

/* gpio number and gpio level*/
static long oplus_wsa_gpiolevel = 0;

struct pinctrl *wsa_pinctl = NULL;
struct pinctrl_state *pinctrl_state_high = NULL;
struct pinctrl_state *pinctrl_state_low = NULL;
struct pinctrl_state *pa_pinctrl_state_high = NULL;
struct pinctrl_state *pa_pinctrl_state_low = NULL;

struct regulator *vdd_reg = NULL;

enum {
        OPLUS_WSA_PRIMARY_ANT = 0x13,
        OPLUS_WSA_SECONDARY_ANT = 0x14,
};

enum {
        WSA_ATTR_UNSPEC,
        WSA_ATTR_ANT_IDX,  /* u32 */
        __WSA_ATTR_MAX,
};
#define WSA_ATTR_MAX   (__WSA_ATTR_MAX - 1)

enum {
        WSA_CMD_UNSPEC,
        WSA_CMD_SWITCH_ANT,
        __WSA_CMD_MAX,
};
#define WSA_CMD_MAX    (__WSA_CMD_MAX - 1)

static struct nla_policy wsa_cmd_policy[WSA_ATTR_MAX + 1] = {
        [WSA_ATTR_ANT_IDX] = {.type =  NLA_U32, },
};

static bool oplus_default_state_is_high(void) {
        int project_id = get_project();
        debug("%s project id is %d", __func__, project_id);
        if (project_id == 22811 || project_id == 22861) {
                return true;
        }
        return false;
}

static int oplus_switch_to_secondary_antenna(void)
{
#ifdef OPLUS_FEATURE_WIFI_SMARTANTENNA
        if (!wsa_pinctl || !pinctrl_state_high || !pinctrl_state_low) {
                debug("pinctrl is NULL");
                return -ENOENT;
        }
        if (oplus_default_state_is_high()) {
                debug("%s the seconcdary ant is low state", __func__);
                pinctrl_select_state(wsa_pinctl, pinctrl_state_low);
        } else {
                debug("%s the seconcdary ant is high state", __func__);
                /* toggle antenna switch to secondary antenna */
                pinctrl_select_state(wsa_pinctl, pinctrl_state_high);
        }

        /* delay 1 us to wait antenna switch */
        udelay(1);
#else
        if (!wsa_pinctl || !pinctrl_state_high || !pa_pinctrl_state_high || !pa_pinctrl_state_low) {
                debug("pinctrl is NULL");
                return -ENOENT;
        }

        debug("%s is high", __func__);
        /* pa switch need to be toggle to 50ohm */
        pinctrl_select_state(wsa_pinctl, pa_pinctrl_state_high);

        /* delay 1 us to wait pa switch */
        udelay(1);

        /* toggle antenna switch to secondary antenna */
        pinctrl_select_state(wsa_pinctl, pinctrl_state_high);

        /* delay 1 us to wait antenna switch */
        udelay(1);

        /* turn pa switch to the path */
        pinctrl_select_state(wsa_pinctl, pa_pinctrl_state_low);
#endif

        return 0;
}

static int oplus_switch_to_prime_antenna(void)
{
#ifdef OPLUS_FEATURE_WIFI_SMARTANTENNA
        if (!wsa_pinctl || !pinctrl_state_high || !pinctrl_state_low) {
                debug("pinctrl is NULL");
                return -ENOENT;
        }

        if (oplus_default_state_is_high()) {
                debug("%s the prime ant is high state", __func__);
                pinctrl_select_state(wsa_pinctl, pinctrl_state_high);
        } else {
                debug("%s the prime ant is low state", __func__);
                /* toggle antenna switch to secondary antenna */
                pinctrl_select_state(wsa_pinctl, pinctrl_state_low);
        }

        /* delay 1 us to wait antenna switch */
        udelay(1);
#else
        if (!wsa_pinctl || !pinctrl_state_low || !pa_pinctrl_state_high || !pa_pinctrl_state_low) {
                debug("pinctrl is NULL");
                return -ENOENT;
        }

        debug("%s is low", __func__);
        /* pa switch need to be toggle to 50ohm */
        pinctrl_select_state(wsa_pinctl, pa_pinctrl_state_high);

        /* delay 1 us to wait pa switch */
        udelay(1);

        /* toggle antenna switch to prime antenna */
        pinctrl_select_state(wsa_pinctl, pinctrl_state_low);

        /* delay 1 us to wait antenna switch */
        udelay(1);

        /* turn pa switch to the path */
        pinctrl_select_state(wsa_pinctl, pa_pinctrl_state_low);
#endif

        return 0;
}

static int wsa_doit(struct sk_buff *skb, struct genl_info *info)
{
        u32 user_cmd = 0;
        int ret = 0;

        if (info->attrs[WSA_ATTR_ANT_IDX]) {
                user_cmd = nla_get_u32(info->attrs[WSA_ATTR_ANT_IDX]);
        }

        debug("user_cmd: 0x%x", user_cmd);
        switch (user_cmd) {
        case OPLUS_WSA_SECONDARY_ANT:
		ret = oplus_switch_to_secondary_antenna();
                break;
        case OPLUS_WSA_PRIMARY_ANT:
                ret = oplus_switch_to_prime_antenna();
                break;
        default:
                return -EINVAL;
        }

        return ret;
}

static struct genl_ops wsa_ops[] = {
        {
                .doit = wsa_doit,
                .cmd = WSA_CMD_SWITCH_ANT,
        },
};

/* The netlink family */
static struct genl_family wsa_family = {
        .name = "WSA_GEN_NL",
        .hdrsize = 0,           /* no private header */
        .version = 1,           /* no particular meaning now */
        .maxattr = WSA_ATTR_MAX,
        .policy = wsa_cmd_policy,
        .module = THIS_MODULE,
        .ops = wsa_ops,
        .n_ops = ARRAY_SIZE(wsa_ops),
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
        .resv_start_op = __WSA_CMD_MAX,
#endif
};

/* register for /sys/module/oplus_wifismartantenna/parameters/oplus_wsa */
static int oplus_wsa_sysmodule_ops_set(const char *kmessage, const struct kernel_param *kp)
{
        if (!kmessage) {
                debug("%s error: kmessage == null!", __func__);
                return -1;
        }
        debug("%s: %s", __func__, kmessage);

        if (kstrtol(kmessage, 10, &oplus_wsa_gpiolevel) ||
                        (oplus_wsa_gpiolevel != 0 && oplus_wsa_gpiolevel != 1)) {
                debug("%s error: gpiolevel parsing error!", __func__);
                return -1;
        }

        debug("%s level = %ld", __func__, oplus_wsa_gpiolevel);

        if (!wsa_pinctl) {
                debug("pinctrl is NULL");
                return -1;
        }

        if (oplus_wsa_gpiolevel == 1) {
                oplus_switch_to_secondary_antenna();
        } else {
                oplus_switch_to_prime_antenna();
        }

        return 0;
}

static const struct kernel_param_ops oplus_wsa_sysmodule_ops = {
        .set = oplus_wsa_sysmodule_ops_set,
        .get = param_get_int,
};

module_param_cb(oplus_wsa, &oplus_wsa_sysmodule_ops, &oplus_wsa_gpiolevel,
                S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

static int oplus_wsa_probe(struct platform_device *pdev)
{
        int ret = 0;
        struct device *dev;

        debug("%s: enter\n", __func__);

        dev = &pdev->dev;
        if (dev->of_node == NULL) {
                debug("%s: Can't find compatible node in device tree\n", __func__);
                return -ENOENT;
        }

        wsa_pinctl = devm_pinctrl_get(dev);
        if (IS_ERR_OR_NULL(wsa_pinctl)) {
                ret = PTR_ERR(wsa_pinctl);
                debug("%s: Failed to get pinctrl, err = %d\n", __func__, ret);
                wsa_pinctl = NULL;
                goto out;
        }

        pinctrl_state_high = pinctrl_lookup_state(wsa_pinctl, "cnss_wlan_wsa_high");
        if (IS_ERR_OR_NULL(pinctrl_state_high)) {
                ret = PTR_ERR(pinctrl_state_high);
                debug("%s: Fail to get wsa pinctrl high state, ret=%d", __func__, ret);
                pinctrl_state_high = NULL;
                goto out;
        }

        pinctrl_state_low = pinctrl_lookup_state(wsa_pinctl, "cnss_wlan_wsa_low");
        if (IS_ERR_OR_NULL(pinctrl_state_low)) {
                ret = PTR_ERR(pinctrl_state_low);
                debug("%s: Fail to get wsa pinctrl low state, ret=%d", __func__, ret);
                pinctrl_state_low = NULL;
                goto out;
        }

#ifndef OPLUS_FEATURE_WIFI_SMARTANTENNA
        pa_pinctrl_state_high = pinctrl_lookup_state(wsa_pinctl, "cnss_pa_switch_high");
        if (IS_ERR_OR_NULL(pa_pinctrl_state_high)) {
                ret = PTR_ERR(pa_pinctrl_state_high);
                debug("%s: Fail to get pa switch pinctrl high state, ret=%d", __func__, ret);
                pa_pinctrl_state_high = NULL;
                goto out;
        }

        pa_pinctrl_state_low = pinctrl_lookup_state(wsa_pinctl, "cnss_pa_switch_low");
        if (IS_ERR_OR_NULL(pa_pinctrl_state_low)) {
                ret = PTR_ERR(pa_pinctrl_state_low);
                debug("%s: Fail to get pa switch pinctrl low state, ret=%d", __func__, ret);
                pa_pinctrl_state_low = NULL;
                goto out;
        }

        if (of_find_property(dev->of_node, "vdd-supply", NULL)) {
                vdd_reg = devm_regulator_get(dev, "vdd");
                if (IS_ERR(vdd_reg)) {
                        ret = PTR_ERR(vdd_reg);
                        debug("%s: Fail to get vdd-supply, ret=%d", __func__, ret);
                }
        }

        if (vdd_reg) {
                ret = regulator_enable(vdd_reg);
                if (ret < 0) {
                        debug("%s: Fail to enable vdd-supply, ret=%d", __func__, ret);
                }
        }
#endif

        if (oplus_default_state_is_high()) {
                debug("%s init as high state!", __func__);
                pinctrl_select_state(wsa_pinctl, pinctrl_state_high);
        } else {
                debug("%s init as low state!", __func__);
                pinctrl_select_state(wsa_pinctl, pinctrl_state_low);
        }
        return 0;
out:
        return ret;
}

static int oplus_wsa_remove(struct platform_device *pdev)
{
        if (wsa_pinctl) {
                devm_pinctrl_put(wsa_pinctl);
        }

        if (vdd_reg) {
                int ret = regulator_disable(vdd_reg);
                if (ret < 0) {
                        debug("%s: Fail to disable vdd-supply, ret=%d", __func__, ret);
                }
        }

        return 0;
}

static const struct of_device_id oplus_wsa_dt_ids[] = {
        { .compatible = "oplus,wlan-wsa" },
        {},
};
MODULE_DEVICE_TABLE(of, oplus_wsa_dt_ids);


static struct platform_driver oplus_wsa_driver = {
        .probe = oplus_wsa_probe,
        .remove = oplus_wsa_remove,
        .driver = {
                .name = "oplus_wsa",
                .of_match_table = of_match_ptr(oplus_wsa_dt_ids),
        },
};

static int __init oplus_wsa_init(void)
{
        int ret = 0;

        platform_driver_register(&oplus_wsa_driver);
        ret = genl_register_family(&wsa_family);
        if (ret) {
                debug("Failed to register wsa netlink family: %d\n", ret);
        }

        return ret;
}

static void __exit oplus_wsa_fini(void)
{
        genl_unregister_family(&wsa_family);
        platform_driver_unregister(&oplus_wsa_driver);
}


module_init(oplus_wsa_init);
module_exit(oplus_wsa_fini);
MODULE_LICENSE("Dual BSD/GPL");
