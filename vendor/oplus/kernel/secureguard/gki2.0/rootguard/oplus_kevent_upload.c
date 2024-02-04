// SPDX-License-Identifier: GPL-2.0-only
/**************************************************************
* Copyright (c)  2008- 2030  OPLUS Mobile communication Corp.ltd All rights reserved.
*
* File       	: oplus_kevent_upload.c
* Description	: oplus_kevent_upload
* Version   	: 1.0
* Date        	: 2019-12-19
* Author    	:
* TAG         	:
****************************************************************/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/netlink.h>
#include <net/net_namespace.h>
#include <linux/proc_fs.h>
#include <net/sock.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <net/genetlink.h>

//#include <trace/hooks/secureguard.h>  /*used to vendor_hook*/

#include "oplus_kevent.h"
#include "oplus_secure_debug.h"
#include "oplus_secure_guard_netlink.h"

static int oplus_security_keventupload_flag = 0;
static volatile unsigned int kevent_pid;

#define OPLUS_KEVENT_MAX_UP_PALOAD_LEN			2048

static void oplus_kevent_send_to_user(void *data, struct kernel_packet_info *userinfo, int *p_retval);

static int security_keventupload_sendpid_cmd(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na = NULL;
	unsigned int * p_data = NULL;

	pr_info("[ROOTCHECK-KEVENTLD-INFO]:kernel recv cmd \n");
	if (info->attrs[SECURE_GUARD_CMD_ATTR_MSG]){
		na = info->attrs[SECURE_GUARD_CMD_ATTR_MSG];
		PRINT_FORMAT(nla_data(na),  nla_len(na));
		pr_info("[ROOTCHECK-KEVENTLD-INFO]:nla_len(na) is %d  \n", nla_len(na));
		p_data = nla_data(na);
		kevent_pid = *p_data;
		pr_info("[ROOTCHECK-KEVENTLD-INFO]:kevent_pid is 0x%x  \n", kevent_pid);
        }

	return 0;
}

static const struct genl_ops oplus_security_ops[] = {
	{
		.cmd		= SECURE_GUARD_CMD_GENL_SENDPID,
		.doit		= security_keventupload_sendpid_cmd,
		//.policy		= taskstats_cmd_get_policy,
		//.flags		= GENL_ADMIN_PERM,
	},
};


static struct genl_family oplus_security_family __ro_after_init = {
	.name		= OPLUS_SECURE_GUARD_PROTOCAL_NAME,
	.version	= OPLUS_SECURE_GUARD_GENL_VERSION,
	.maxattr	= SECURE_GUARD_CMD_ATTR_MAX,
	.module		= THIS_MODULE,
	.ops		= oplus_security_ops,
	.n_ops		= ARRAY_SIZE(oplus_security_ops),
};


static inline int genl_msg_prepare_usr_msg(unsigned char cmd, size_t size, pid_t pid, struct sk_buff **skbp)
{
    struct sk_buff *skb;

    /* create a new netlink msg */
    skb = genlmsg_new(size, GFP_KERNEL|GFP_ATOMIC);
    if (skb == NULL) {
        return -ENOMEM;
    }

    /* Add a new netlink message to an skb */
    genlmsg_put(skb, pid, 0, &oplus_security_family, 0, cmd);

    *skbp = skb;
    return 0;
}

static inline int genl_msg_mk_usr_msg(struct sk_buff *skb, int type, void *data, int len)
{
    int ret;

    /* add a netlink attribute to a socket buffer */
    if ((ret = nla_put(skb, type, len, data)) != 0) {
        return ret;
    }
    return 0;
}


static void oplus_kevent_send_to_user(void *data, struct kernel_packet_info *userinfo, int *p_retval)
{
#ifdef CONFIG_DEBUG_ATOMIC_SLEEP
	(void)data;
	(void)userinfo;
	(void)p_retval;
	pr_info("userdebug mode & CONFIG_DEBUG_ATOMIC_SLEEP == y, no report.\n");
	return;
#else
	int ret = 0;

	struct sk_buff *skbuff = NULL;
	void * head = NULL;
	size_t data_len = 0;
	size_t attr_len = 0;
        *p_retval = 0;
	/*max_len */
	/*pr_info("[ROOTCHECK-KEVENTLD-INFO]:oplus_kevent_send_to_user\n");*/

	if (userinfo->payload_length >= OPLUS_KEVENT_MAX_UP_PALOAD_LEN){
        pr_err("[ROOTCHECK-KEVENTLD-ERROR]:kevent_send_to_user: payload_length out of range\n");
		*p_retval = -1;
		return ;
	}

	data_len = userinfo->payload_length + sizeof(struct kernel_packet_info);
	attr_len = nla_total_size(data_len);
	/*pr_info("[OPLUS_SECURITY DEBUG]:data_len is %u, attr_len is %u\n", data_len, attr_len);*/

	ret = genl_msg_prepare_usr_msg(SECURE_GUARD_CMD_GENL_UPLOAD, attr_len, kevent_pid, &skbuff);
        if (ret){
        pr_err("[ROOTCHECK-KEVENTLD-ERROR]:genl_msg_prepare_usr_msg err, ret is %d \n");
		*p_retval = -1;
		return ;
	}

        ret = genl_msg_mk_usr_msg(skbuff, SECURE_GUARD_CMD_ATTR_MSG, userinfo, data_len);
        if (ret) {
        kfree_skb(skbuff);
		*p_retval = ret;
		return ;
        }

        head = genlmsg_data(nlmsg_data(nlmsg_hdr(skbuff)));

        genlmsg_end(skbuff, head);

        ret = genlmsg_unicast(&init_net, skbuff, kevent_pid);
        if (ret < 0) {
		*p_retval = ret;
		return ;
        }

	*p_retval = 0;
        return ;
#endif
}

int kevent_send_to_user(struct kernel_packet_info *userinfo)
{
    int ret = 0;

	oplus_kevent_send_to_user(NULL, userinfo, &ret);

	return ret;
}
EXPORT_SYMBOL_GPL(kevent_send_to_user);

static int __init oplus_security_keventupload_init(void)
{
	int ret = 0;

	/*register gen_link family*/
	ret = genl_register_family(&oplus_security_family);
	if (ret){
		goto exit;
	}
	oplus_security_keventupload_flag = 1;
	pr_info("[ROOTCHECK-KEVENTLD-INFO]registered gen_link family %s OK \n", OPLUS_SECURE_GUARD_PROTOCAL_NAME);

exit:
    if (ret){
        if (oplus_security_keventupload_flag){
	        genl_unregister_family(&oplus_security_family);
	    }
	}
	return ret;
}


static void __exit oplus_security_keventupload_exit()
{
	if (oplus_security_keventupload_flag){
		genl_unregister_family(&oplus_security_family);
	}
	return ;
}

module_init(oplus_security_keventupload_init);
module_exit(oplus_security_keventupload_exit);

MODULE_LICENSE("GPL");


