/***********************************************************
** Copyright (C), 2008-2019, OPLUS Mobile Comm Corp., Ltd.
** File: hans.c
** Description: Add for hans freeze manager
**
** Version: 1.0
** Date : 2019/09/23
**
** ------------------ Revision History:------------------------
** <author>      <data>      <version >       <desc>
** Kun Zhou    2019/09/23      1.0       OPLUS_ARCH_EXTENDS
** Kun Zhou    2020/05/27      1.1       OPLUS_FEATURE_HANS_FREEZE
** Qingxin Guo 2021/08/04      1.2       GENERIC NETLINK
****************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/genetlink.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <net/genetlink.h>
#include "hans.h"

#define NETLINK_PORT_HANS        (0x15356)
#define PRINT_LIMIT              (1000)

static atomic_t hans_deamon_port;
static unsigned long log_jiffies = 0;
static int hans_handler(struct sk_buff *skb, struct genl_info *info);

static const struct genl_ops hans_genl_ops[] = {
        {
                .cmd = HANS_CMD_GENL,
                .flags = 0,
                .doit = hans_handler,
                .dumpit = NULL,
        },
};

static struct genl_family hans_genl_family = {
        .id = GENL_ID_GENERATE,
        .hdrsize = 0,
        .name = HANS_FAMILY,
        .version = HANS_FAMILY_VERSION,
        .maxattr = HANS_ATTR_MSG_MAX,
        .ops = hans_genl_ops,
        .n_ops = ARRAY_SIZE(hans_genl_ops),
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
        .resv_start_op = HANS_CMD_GENL + 1,
#endif
};


static inline int genl_msg_prepare_usr_msg(unsigned char cmd, size_t size,
                                                        pid_t pid, struct sk_buff **skbp)
{
        struct sk_buff *skb;
        /* create a new netlink msg */
        skb = genlmsg_new(size, GFP_ATOMIC);
        if (skb == NULL) {
                pr_err("oplus_hans: new genlmsg alloc failed\n");
                return -ENOMEM;
        }
        /* Add a new netlink message to an skb */
        genlmsg_put(skb, pid, 0, &hans_genl_family, 0, cmd);
        *skbp = skb;
        return 0;
}

static inline int genl_msg_mk_usr_msg(struct sk_buff *skb, int type, void *data, int len)
{
        int ret = 0;

        /* add a netlink attribute to a socket buffer */
        ret = nla_put(skb, type, len, data);

        return ret;
}

/*
 *reuse LOOPBACK and FROZEN_TRANS channel to notify framework whether kernel support cgroupv2 or not
 */
static void hans_kern_support_cgrpv2(void) {
	/*notify framework that kernel support cgroupv2*/
	hans_report(PKG, -1, -1, -1, -1, "PKG", HANS_USE_CGRPV2);
	printk(KERN_ERR "%s: hans support cgroupv2\n", __func__);
}

/*
 * netlink report function to tell HANS native deamon unfreeze process info
 * if the parameters is empty, fill it with (pid/uid with -1)
 */
int hans_report(enum message_type type, int caller_pid, int caller_uid, int target_pid, int target_uid, const char *rpc_name, int code)
{
        int len = 0;
        int ret = 0;
        void *head;
        size_t size;
        struct hans_message data;
        struct sk_buff *skb = NULL;

        if (atomic_read(&hans_deamon_port) == -1) {
                pr_err("%s: hans_deamon_port invalid!\n", __func__);
                return HANS_ERROR;
        }

        if (type >= TYPE_MAX) {
                pr_err("%s: type = %d invalid!\n", __func__, type);
                return HANS_ERROR;
        }

        len = sizeof(struct hans_message);
        size = nla_total_size(len);

        ret = genl_msg_prepare_usr_msg(HANS_CMD_GENL, size, (pid_t)atomic_read(&hans_deamon_port), &skb);
        if (ret) {
                pr_err("%s: genl_msg_prepare_usr_msg failed!\n", __func__);
                return HANS_ERROR;
        }

        data.type = type;
        data.port = NETLINK_PORT_HANS;
        data.caller_pid = caller_pid;
        data.caller_uid = caller_uid;
        data.target_pid = target_pid;
        data.target_uid = target_uid;
        data.pkg_cmd = -1; /* invalid package cmd */
        data.code = code;
        strlcpy(data.rpc_name, rpc_name, INTERFACETOKEN_BUFF_SIZE);

        ret = genl_msg_mk_usr_msg(skb, HANS_ATTR_MSG_GENL, &data, len);
        if (ret) {
                pr_err("%s: genl_msg_mk_usr_msg failed!\n", __func__);
                kfree_skb(skb);
                return HANS_ERROR;
        }

        head = genlmsg_data(nlmsg_data(nlmsg_hdr(skb)));
        genlmsg_end(skb, head);

        if ((ret = genlmsg_unicast(&init_net, skb, (u32)atomic_read(&hans_deamon_port))) < 0) {
                pr_err("%s: genlmsg_unicast failed! err = %d\n", __func__ , ret);
                return HANS_ERROR;
        }

        return HANS_NOERROR;
}

/* HANS kernel module handle the message from HANS native deamon */
static int hans_handler(struct sk_buff *skb, struct genl_info *info)
{
        struct hans_message *data = NULL;
        struct nlmsghdr *nlh = NULL;
        struct genlmsghdr *genlhdr = NULL;
        struct nlattr *nla = NULL;

        unsigned int len  = 0;
        int uid = -1;

        if (!skb) {
                pr_err("%s: recv skb NULL!\n", __func__);
                return HANS_ERROR;
        }

        /* safety check */
        uid = (*NETLINK_CREDS(skb)).uid.val;
        /* only allow native deamon talk with HANS kernel. */
        /*if (uid != 1000) {
                pr_err("%s: uid: %d, permission denied\n", __func__, uid);
                return;
        }*/

        if (skb->len >= NLMSG_SPACE(0)) {
                nlh = nlmsg_hdr(skb);
                genlhdr = nlmsg_data(nlh);
                nla = genlmsg_data(genlhdr);

                if (nla->nla_type == HANS_ATTR_MSG_GENL) {
                        len = NLA_PAYLOAD(nla->nla_len);
                        data = (struct hans_message *)NLA_DATA(nla);

                        /* native daemon in ofreezer 1.0 has no 'int persistent' in the message structure */
                        if (len < (sizeof(struct hans_message) - sizeof(int))) {
                                pr_err("%s: hans_message len check faied! len = %d	min_expected_len = %lu!\n",
                                        __func__, len, sizeof(struct hans_message) - sizeof(int));
                                return HANS_ERROR;
                        }

                        if (data->port < 0) {
                                pr_err("%s: portid = %d invalid!\n", __func__, data->port);
                                return HANS_ERROR;
                        }
                        if (data->type >= TYPE_MAX) {
                                pr_err("%s: type = %d invalid!\n", __func__, data->type);
                                return HANS_ERROR;
                        }
                        if (atomic_read(&hans_deamon_port) == -1 && data->type != LOOP_BACK) {
                                pr_err("%s: handshake not setup, type = %d!\n", __func__, data->type);
                                return HANS_ERROR;
                        }

                        switch (data->type) {
                        case LOOP_BACK:  /*Loop back message, only for native deamon and kernel handshake*/
                                atomic_set(&hans_deamon_port, data->port);
                                hans_report(LOOP_BACK, -1, -1, -1, -1, "loop back", CPUCTL_VERSION);
                                printk(KERN_ERR "%s: --> LOOP_BACK, port = %d\n", __func__, data->port);
				hans_kern_support_cgrpv2();
                                break;

                        case PKG:
                                if (len < sizeof(struct hans_message)) {
                                        /*
                                        printk(KERN_ERR "%s: --> PKG, native daemon 1.0 without 'int persistent' in the message, len=%d, expected len=%d\n",
                                                __func__, len, sizeof(struct hans_message));
                                        */
                                        /* native daemon in ofreezer 1.0 has no 'int persistent' in the message structure */
                                        printk(KERN_ERR "%s: --> PKG, ofreezer 1.0 native, uid = %d, pkg_cmd = %d\n",
                                                __func__, data->target_uid, data->pkg_cmd);
					hans_network_cmd_parse(data->target_uid, 0 /* persistent */, data->pkg_cmd);
                                        break;
                                }
				if (printk_timed_ratelimit(&log_jiffies, PRINT_LIMIT)) {
					printk(KERN_ERR "%s: --> PKG, uid = %d, persistent = %d, pkg_cmd = %d\n",
						__func__, data->target_uid, data->persistent, data->pkg_cmd);
				}
                                hans_network_cmd_parse(data->target_uid, data->persistent, data->pkg_cmd);
                                break;

                        case FROZEN_TRANS:
                        case CPUCTL_TRANS:
				if (CHECK_KERN_SUPPORT_CGRPV2 == data->target_uid) {
					hans_kern_support_cgrpv2();
				} else {
					if (printk_timed_ratelimit(&log_jiffies, PRINT_LIMIT)) {
						printk(KERN_ERR "%s: --> FROZEN_TRANS, uid = %d, frozen_check_type = %d\n", __func__, data->target_uid, data->persistent);
					}
					hans_check_frozen_transcation(data->target_uid, data->type, data->persistent);
				}
                                break;

                        default:
                                pr_err("%s: hans_messag type invalid %d\n", __func__, data->type);
                                break;
                        }
                }
        }
        return HANS_NOERROR;
}

int register_hans_vendor_hooks(void)
{
	int rc = 0;

	rc = register_trace_android_vh_binder_preset(binder_preset_handler, NULL);
	if (rc != 0) {
		pr_err("register_trace_android_vh_binder_preset failed, rc=%d\n", rc);
		return rc;
	}

	rc = register_trace_android_vh_binder_trans(binder_trans_handler, NULL);
	if (rc != 0) {
		pr_err("register_trace_android_vh_binder_trans failed,  rc=%d\n", rc);
		return rc;
	}

	rc = register_trace_android_vh_binder_reply(binder_reply_handler, NULL);
	if (rc != 0) {
		pr_err("register_trace_android_vh_binder_reply failed, rc=%d\n", rc);
		return rc;
	}

	rc = register_trace_android_vh_binder_alloc_new_buf_locked(binder_alloc_handler, NULL);
	if (rc != 0) {
		pr_err("register_trace_android_vh_binder_alloc_new_buf_locked failed, rc=%d\n", rc);
		return rc;
	}

	rc = register_trace_android_vh_do_send_sig_info(send_signal_handler, NULL);
	if (rc != 0) {
		pr_err("register_trace_android_vh_do_send_sig_info failed, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

void unregister_hans_vendor_hooks(void)
{
	unregister_trace_android_vh_binder_preset(binder_preset_handler, NULL);
	unregister_trace_android_vh_binder_trans(binder_trans_handler, NULL);
	unregister_trace_android_vh_binder_reply(binder_reply_handler, NULL);
	unregister_trace_android_vh_binder_alloc_new_buf_locked(binder_alloc_handler, NULL);
	unregister_trace_android_vh_do_send_sig_info(send_signal_handler, NULL);
}

static int __init hans_core_init(void)
{
        if (genl_register_family(&hans_genl_family) != 0) {
                pr_err("%s: genl_register_family error!\n", __func__);
		goto genl_failed;
        }

	if(register_hans_vendor_hooks() != 0) {
		pr_err("%s: hans vendor hook register failed!\n", __func__);
		goto vh_failed;
	}

	atomic_set(&hans_deamon_port, -1);

	if (hans_netfilter_init() == HANS_ERROR) {
		pr_err("%s: netfilter init failed!\n", __func__);
		goto nf_failed;
	}

	printk(KERN_INFO "%s: -\n", __func__);
	return HANS_NOERROR;

nf_failed:
vh_failed:
	unregister_hans_vendor_hooks();
genl_failed:
	genl_unregister_family(&hans_genl_family);
	return HANS_ERROR;
}

static void __exit hans_core_exit(void)
{
	genl_unregister_family(&hans_genl_family);
	unregister_hans_vendor_hooks();
	hans_netfilter_deinit();
	printk(KERN_INFO "%s: -\n", __func__);
}

module_init(hans_core_init);
module_exit(hans_core_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("zhoukun1 <zhoukun1>");

