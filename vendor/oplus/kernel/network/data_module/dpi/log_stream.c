/***********************************************************
** Copyright (C), 2008-2022, oplus Mobile Comm Corp., Ltd.
** File: log_stream.c
** Description: add logkit stream identify
**
** Version: 1.0
** Date : 2022/10/13
** Author: Zhangpeng
**
** ------------------ Revision History:------------------------
** <author> <data> <version > <desc>
** Zhangpeng 2022/10/13 1.0 build this module
****************************************************************/

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/icmp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netlink.h>
#include <linux/random.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/tcp.h>
#include <linux/types.h>
#include <linux/version.h>
#include <net/dst.h>
#include <net/genetlink.h>
#include <net/inet_connection_sock.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/route.h>
#include <net/tcp.h>
#include <net/tcp_states.h>
#include <net/udp.h>
#include <linux/netfilter_ipv6.h>

#include "dpi_core.h"
#include "log_stream.h"

#include "../include/netlink_api.h"
#include "../include/comm_def.h"
#include "../include/dpi_api.h"

static int s_debug = 0;

#define LOG_TAG "LOG-KIT"
#define logt(fmt, args...) LOG(LOG_TAG, fmt, ##args)
#define logi(fmt, args...) do { \
	if (s_debug) { \
		LOG(LOG_TAG, fmt, ##args); \
	} \
} while (0)

#define MAX_IP_COUNT 10
#define MAX_HOSTNAME_LENGTH 128


u32 s_log_kit_uid = 0;

static char s_hostname[MAX_HOSTNAME_LENGTH];
static u32 s_ip_addr[MAX_IP_COUNT];
static int s_ip_count = 0;



static u32 get_log_service_addr(struct iphdr *iph, int dir)
{
	if (dir) {
		return (u32)ntohl(iph->daddr);
	} else {
		return (u32)ntohl(iph->saddr);
	}
}

static struct iphdr *get_ip_header(struct sk_buff *skb, int dir)
{
	if (skb->protocol != htons(ETH_P_IP)) {
		return NULL;
	}
	return ip_hdr(skb);
}

static int is_log_stream(struct iphdr *iph, int dir, dpi_match_data_t *data)
{
	u32 log_server_addr = get_log_service_addr(iph, dir);
	size_t cur = 0;
	u32* ipAddrCur = NULL;
	ipAddrCur = s_ip_addr;
	for (; cur < s_ip_count && *ipAddrCur; cur++, ipAddrCur++) {
		if (*ipAddrCur == log_server_addr) {
			return 1;
		}
	}
	return 0;
}

static int log_kit_set_ip_addr(u32 eventid, Netlink__Proto__RequestMessage *requestMsg, char **rsp_data, u32 *rsp_len)
{
	if ((requestMsg->request_data_case != NETLINK__PROTO__REQUEST_MESSAGE__REQUEST_DATA_REQUEST_SET_LOG_STEAM_IP)
		|| (!requestMsg->requestsetlogsteamip)) {
		return COMM_NETLINK_ERR_PARAM;
	}
	memset(s_hostname, '\0', sizeof(s_hostname));
	memset(s_ip_addr, '\0', sizeof(s_ip_addr));
	strncpy(s_hostname, requestMsg->requestsetlogsteamip->hostname, (MAX_HOSTNAME_LENGTH - 1));
	s_ip_count = requestMsg->requestsetlogsteamip->n_ipaddr;
	if(s_ip_count > MAX_IP_COUNT) {
		s_ip_count = MAX_IP_COUNT;
	}
	if(s_ip_count > 0) {
		memcpy(s_ip_addr, requestMsg->requestsetlogsteamip->ipaddr, s_ip_count * sizeof(uint32_t));
	}
	do {
		size_t len = 0, pack_len = 0;
		char *buf = NULL;
		NETLINK_RSP_DATA_DECLARE(rsp_name, requestMsg->header->requestid, requestMsg->header->eventid, COMM_NETLINK_SUCC);
		len = netlink__proto__response_message__get_packed_size(&rsp_name);
		buf = kmalloc(len, GFP_ATOMIC);
		if (!buf) {
			logt("malloc size %lu failed", len);
			return COMM_NETLINK_ERR_MEMORY;
		}
		pack_len = netlink__proto__response_message__pack(&rsp_name, buf);
		logi("request_log_kit_set_ip_addr pack len %lu  buf len %lu", pack_len, len);
		*rsp_data = buf;
		*rsp_len = len;
	} while (0);
	return COMM_NETLINK_SUCC;
}

static int log_stream_match(struct sk_buff *skb, int dir, dpi_match_data_t *data)
{
	struct iphdr *iph = NULL;
	iph = get_ip_header(skb, dir);
	if (!iph || iph->protocol != IPPROTO_TCP) {
		data->dpi_result = DPI_ID_SYSTEM_APP;
		data->state = DPI_MATCH_STATE_COMPLETE;
		return 0;
	}
	if (is_log_stream(iph, dir, data)) {
		logt("match log stream success");
		data->dpi_result = DPI_ID_LOG_KIT_STREAM_DATA;
		data->state = DPI_MATCH_STATE_COMPLETE;
		return 0;
	}
	data->dpi_result = DPI_ID_SYSTEM_APP;
	data->state = DPI_MATCH_STATE_COMPLETE;
	return 0;
}

int set_system_uid(u32 uid)
{
	int ret = 0;
	u32 old_uid = s_log_kit_uid;
	s_log_kit_uid = uid;
	logt("dpi uid : %u", uid);
	if (s_log_kit_uid != old_uid) {
		if (old_uid == 0) {
			ret = dpi_register_app_match(s_log_kit_uid, log_stream_match);
			logt("dpi_register_app_match log_kit uid %u return %d", s_log_kit_uid, ret);
		} else if (s_log_kit_uid == 0) {
			ret = dpi_unregister_app_match(s_log_kit_uid);
			logt("dpi_unregister_app_match log_kit uid %u return %d", s_log_kit_uid, ret);
		} else {
			ret |= dpi_unregister_app_match(old_uid);
			ret |= dpi_register_app_match(s_log_kit_uid, log_stream_match);
			logt("dpi app uid change! log_kit uid %u %u return %d", s_log_kit_uid, old_uid, ret);
		}
	}
	return ret;
}

static void data_free(void *data)
{
	if (data) {
		kfree(data);
	}
}

static struct ctl_table oplus_log_stream_sysctl_table[] = {
	{
		.procname   = "debug",
		.data       = &s_debug,
		.maxlen     = sizeof(int),
		.mode       = 0644,
		.proc_handler   = proc_dointvec,
	},
	{}
};

static struct ctl_table_header *oplus_log_stream_table_hdr = NULL;

static int oplus_stats_calc_sysctl_init(void)
{
	oplus_log_stream_table_hdr = register_net_sysctl(&init_net, "net/oplus_log_stream", oplus_log_stream_sysctl_table);
	return oplus_log_stream_table_hdr == NULL ? -ENOMEM : 0;
}

int log_stream_init(void)
{
	int ret = 0;
	ret = register_netlink_request(COMM_NETLINK_EVENT_SET_LOG_STREAM_IPADDR, log_kit_set_ip_addr, data_free);
	if (ret < 0) {
		logt("register cmd COMM_NETLINK_EVENT_SET_LOG_STREAM_IPADDR failed, ret=%d", ret);
		return ret;
	} else {
		logi("init netlink successfully.");
	}
	ret = oplus_stats_calc_sysctl_init();
	if(ret < 0) {
		logt("register net sysctl failed, ret=%d", ret);
		unregister_netlink_request(COMM_NETLINK_EVENT_SET_LOG_STREAM_IPADDR);
		return ret;
	}
	else {
		logi("init sysctl successfully.");
	}
	return ret;
}

void log_stream_fini(void)
{
	int ret = 0;
	unregister_netlink_request(COMM_NETLINK_EVENT_SET_LOG_STREAM_IPADDR);
	if (s_log_kit_uid) {
		ret = dpi_unregister_app_match(s_log_kit_uid);
		logt("dpi_unregister_app_match log kit uid %u return %d", s_log_kit_uid, ret);
		s_log_kit_uid = 0;
	}
	if (oplus_log_stream_table_hdr) {
		unregister_net_sysctl_table(oplus_log_stream_table_hdr);
	}
}
