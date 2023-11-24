// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/netlink.h>
#include <net/net_namespace.h>
#include <linux/proc_fs.h>
#include <net/sock.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/genetlink.h>
#include <net/genetlink.h>
#include "powerkey_monitor.h"
#include "theia_kevent_kernel.h"

#define NLA_DATA(na) ((char *)((char *)(na) + NLA_HDRLEN))
#define NLA_PAYLOAD(len) ((len) - NLA_HDRLEN)

static u32 theia_kevent_pid = -1;
static DEFINE_MUTEX(theia_kevent_mutex);

#define THEIA_KEVENT_DEBUG_PRINTK(a, arg...)\
	do {\
		printk("[theia_kevent]: " a, ##arg);\
	} while (0)

static int theia_kevent_receive_from_user(struct sk_buff *skb,
	struct genl_info *info);

static const struct genl_ops theia_genl_ops[] = {
	{
		.cmd = THEIA_CMD_DOWNLINK,
		.flags = 0,
		.doit = theia_kevent_receive_from_user,
		.dumpit = NULL,
	},
};

static struct genl_family theia_genl_family = {
	.id = GENL_ID_GENERATE,
	.hdrsize = 0,
	.name = THEIA_FAMILY_NAME,
	.version = THEIA_FAMILY_VERSION,
	.maxattr = THEIA_ATTR_TYPE_MAX,
	.ops = theia_genl_ops,
	.n_ops = ARRAY_SIZE(theia_genl_ops),
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
	.resv_start_op = __THEIA_CMD_MAX,
#endif
};

/* kernel receive message from userspace */
static int theia_kevent_receive_from_user(struct sk_buff *skb,
	struct genl_info *info)
{
	struct nlmsghdr *nlhdr;
	struct genlmsghdr *genlhdr;
	struct nlattr *nla;
	char *pdata = NULL;
	char *payload = NULL;
	uint32_t size = 0x0;

	THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_receive_from_user called\n");

	if (skb == NULL) {
		THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_receive_from_user skb invalid\n");
		return -EINVAL;
	}

	if (skb->len < NLMSG_HDRLEN) {
		THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_receive_from_user skb len invalid\n");
		return -EINVAL;
	}

	nlhdr = nlmsg_hdr(skb);
	genlhdr = nlmsg_data(nlhdr);
	nla = genlmsg_data(genlhdr);
	payload = NLA_DATA(nla);

	size = NLA_PAYLOAD(nla->nla_len);
	if (size == 0) {
		THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_receive_from_user payload len zero\n");
		return -EINVAL;
	}

	pdata = (char *)kmalloc(size + 1, GFP_KERNEL);
	if (pdata == NULL) {
		THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_receive_from_user kmalloc failed\n");
		return -ENOMEM;
	}

	memcpy(pdata, payload, size);
	pdata[size] = 0x0;
	THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_receive_from_user, pdata:%s\n", pdata);

	if (!strcmp(pdata, THEIA_KEVENT_MODULE)) {
		theia_kevent_pid = nlhdr->nlmsg_pid;
		THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_receive_from_user, theia_kevent_pid is %u ..\n", theia_kevent_pid);
	}

	if (pdata)
		kfree(pdata);

	return THEIA_OK;
}

static inline int genl_msg_prepare_usr_msg(unsigned char cmd,
	size_t size, pid_t pid, struct sk_buff **skbp)
{
	struct sk_buff *skb;

	/* create a new netlink msg */
	skb = genlmsg_new(size, GFP_ATOMIC);
	if (skb == NULL) {
		THEIA_KEVENT_DEBUG_PRINTK("create a new netlink msg failed\n");
		return -ENOMEM;
	}

	/* add a new netlink msg to a skb */
	genlmsg_put(skb, pid, 0, &theia_genl_family, 0, cmd);
	*skbp = skb;
	return 0;
}

static inline int genl_msg_mk_usr_msg(struct sk_buff *skb, int type,
	void *data, int len)
{
	/* add a netlink attribute to a socket buffer */
	return nla_put(skb, type, len, data);
}

/* kernel send message to userspace */
static int theia_kevent_send_to_user(struct theia_kevent_packet *userinfo,
	enum theia_attr_type type)
{
	int ret;
	size_t payload_size;
	size_t size;
	struct sk_buff *skb;
	void *head;
	char *payload;

	THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_send_to_user called\n");

	if (theia_kevent_pid == -1) {
		THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_send_to_user theia_kevent_pid is -1, return\n");
		return -1;
	}

	/* protect payload too long problem */
	if (userinfo->len >= MAX_PAYLOAD_DATASIZE) {
		THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_send_to_user payload_length out of range\n");
		return -1;
	}

	payload_size = sizeof(struct theia_kevent_packet) + userinfo->len;
	size = nla_total_size(payload_size);
	ret = genl_msg_prepare_usr_msg(THEIA_CMD_UPLINK, size, theia_kevent_pid, &skb);
	if (ret) {
		THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_send_to_user genl msg alloc failed\n");
		return ret;
	}

	payload = (char *)userinfo;
	ret = genl_msg_mk_usr_msg(skb, type, payload, payload_size);
	if (ret) {
		THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_send_to_user add netlink attribute to socket buffer failed\n");
		kfree_skb(skb);
		return ret;
	}

	head = genlmsg_data(nlmsg_data(nlmsg_hdr(skb)));
	genlmsg_end(skb, head);

	/* send data */
	ret = genlmsg_unicast(&init_net, skb, theia_kevent_pid);
	if (ret < 0) {
		THEIA_KEVENT_DEBUG_PRINTK("theia_kevent_send_to_user send fail = %d, theia_kevent_pid = %d \n", ret, theia_kevent_pid);
		return -1;
	}

	return 0;
}

/* send msg to userspace */
void SendTheiaKevent(int type, char *log_tag, char *event_id, char *payload)
{
	struct theia_kevent_packet *user_msg_info;
	void *buffer = NULL;
	int len;
	size_t size;
	int ret;

	THEIA_KEVENT_DEBUG_PRINTK("SendTheiaKevent begin\n");

	mutex_lock(&theia_kevent_mutex);

	/* alloc memory */
	len = strlen(payload);
	size = sizeof(struct theia_kevent_packet) + len + 1;
	buffer = kmalloc(size, GFP_ATOMIC);
	memset(buffer, 0, size);
	user_msg_info = (struct theia_kevent_packet *)buffer;

	/* setup tag */
	memcpy(user_msg_info->tag, log_tag, strlen(log_tag) + 1);

	/* setup event id */
	memcpy(user_msg_info->event_id, event_id, strlen(event_id) + 1);

	/* setup payload */
	user_msg_info->len = len + 1;
	memcpy(user_msg_info->data, payload, len + 1);

	/* send to userspace */
	ret = theia_kevent_send_to_user(user_msg_info, type);
	if (ret)
		THEIA_KEVENT_DEBUG_PRINTK("SendTheiaKevent failed :%d\n", ret);
	else
		THEIA_KEVENT_DEBUG_PRINTK("SendTheiaKevent success, user_msg_info->data:%s\n", user_msg_info->data);

	kfree(buffer);
	mutex_unlock(&theia_kevent_mutex);
	return;
}

void SendDcsTheiaKevent(char *log_tag, char *event_id, char *logmap)
{
	SendTheiaKevent(THEIA_ATTR_TYPE_DCS_MSG, log_tag, event_id, logmap);
}

int theia_kevent_module_init(void) {
	int ret;

	ret = genl_register_family(&theia_genl_family);
	if (ret) {
		THEIA_KEVENT_DEBUG_PRINTK("genl register family failed, ret = %d\n", ret);
		return ret;
	} else {
		THEIA_KEVENT_DEBUG_PRINTK("genl register family success, ret = %d\n", ret);
	}

	return 0;
}

void theia_kevent_module_exit(void) {
	genl_unregister_family(&theia_genl_family);
	THEIA_KEVENT_DEBUG_PRINTK("theia_kevent exit\n");
}
