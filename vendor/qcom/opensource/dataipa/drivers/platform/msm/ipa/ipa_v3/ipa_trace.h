/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM ipa
#define TRACE_INCLUDE_FILE ipa_trace

#if !defined(_IPA_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _IPA_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(
	intr_to_poll3,

	TP_PROTO(unsigned long client),

	TP_ARGS(client),

	TP_STRUCT__entry(
		__field(unsigned long,	client)
	),

	TP_fast_assign(
		__entry->client = client;
	),

	TP_printk("client=%lu", __entry->client)
);

TRACE_EVENT(
	poll_to_intr3,

	TP_PROTO(unsigned long client),

	TP_ARGS(client),

	TP_STRUCT__entry(
		__field(unsigned long,	client)
	),

	TP_fast_assign(
		__entry->client = client;
	),

	TP_printk("client=%lu", __entry->client)
);

TRACE_EVENT(
	idle_sleep_enter3,

	TP_PROTO(unsigned long client),

	TP_ARGS(client),

	TP_STRUCT__entry(
		__field(unsigned long,	client)
	),

	TP_fast_assign(
		__entry->client = client;
	),

	TP_printk("client=%lu", __entry->client)
);

TRACE_EVENT(
	idle_sleep_exit3,

	TP_PROTO(unsigned long client),

	TP_ARGS(client),

	TP_STRUCT__entry(
		__field(unsigned long,	client)
	),

	TP_fast_assign(
		__entry->client = client;
	),

	TP_printk("client=%lu", __entry->client)
);

TRACE_EVENT(
	rmnet_ipa_netifni3,

	TP_PROTO(unsigned long rx_pkt_cnt),

	TP_ARGS(rx_pkt_cnt),

	TP_STRUCT__entry(
		__field(unsigned long,	rx_pkt_cnt)
	),

	TP_fast_assign(
		__entry->rx_pkt_cnt = rx_pkt_cnt;
	),

	TP_printk("rx_pkt_cnt=%lu", __entry->rx_pkt_cnt)
);

TRACE_EVENT(
	rmnet_ipa_netifrx3,

	TP_PROTO(unsigned long rx_pkt_cnt),

	TP_ARGS(rx_pkt_cnt),

	TP_STRUCT__entry(
		__field(unsigned long,	rx_pkt_cnt)
	),

	TP_fast_assign(
		__entry->rx_pkt_cnt = rx_pkt_cnt;
	),

	TP_printk("rx_pkt_cnt=%lu", __entry->rx_pkt_cnt)
);

TRACE_EVENT(
	rmnet_ipa_netif_rcv_skb3,

	TP_PROTO(const struct sk_buff *skb, unsigned long rx_pkt_cnt),

	TP_ARGS(skb, rx_pkt_cnt),

	TP_STRUCT__entry(
		__string(name,			skb->dev->name)
		__field(const void *,	skbaddr)
		__field(u16,			protocol)
		__field(unsigned int,	len)
		__field(unsigned int,	data_len)
		__field(unsigned long,	rx_pkt_cnt)
	),

	TP_fast_assign(
		__assign_str(name, skb->dev->name);
		__entry->skbaddr = skb;
		__entry->protocol = ntohs(skb->protocol);
		__entry->len = skb->len;
		__entry->data_len = skb->data_len;
		__entry->rx_pkt_cnt = rx_pkt_cnt;
	),

	TP_printk("dev=%s skbaddr=%p protocol=0x%04x len=%u data_len=%u rx_pkt_cnt=%lu",
		__get_str(name),
		__entry->skbaddr,
		__entry->protocol,
		__entry->len,
		__entry->data_len,
		__entry->rx_pkt_cnt)
);

TRACE_EVENT(
	ipa3_napi_rx_poll_num,

	TP_PROTO(unsigned long client, int poll_num),

	TP_ARGS(client, poll_num),

	TP_STRUCT__entry(
		__field(unsigned long,	client)
		__field(int,	poll_num)
	),

	TP_fast_assign(
		__entry->client = client;
		__entry->poll_num = poll_num;
	),

	TP_printk("client=%lu each_poll_aggr_pkt_num=%d",
		__entry->client,
		__entry->poll_num)
);

TRACE_EVENT(
	ipa3_napi_rx_poll_cnt,

	TP_PROTO(unsigned long client, int poll_num),

	TP_ARGS(client, poll_num),

	TP_STRUCT__entry(
		__field(unsigned long,	client)
		__field(int,	poll_num)
	),

	TP_fast_assign(
		__entry->client = client;
		__entry->poll_num = poll_num;
	),

	TP_printk("client=%lu napi_overall_poll_pkt_cnt=%d",
		__entry->client,
		__entry->poll_num)
);

TRACE_EVENT(
	ipa3_napi_schedule,

	TP_PROTO(unsigned long client),

	TP_ARGS(client),

	TP_STRUCT__entry(
		__field(unsigned long,	client)
	),

	TP_fast_assign(
		__entry->client = client;
	),

	TP_printk("client=%lu", __entry->client)
);

TRACE_EVENT(
	ipa3_napi_poll_entry,

	TP_PROTO(unsigned long client),

	TP_ARGS(client),

	TP_STRUCT__entry(
		__field(unsigned long,	client)
	),

	TP_fast_assign(
		__entry->client = client;
	),

	TP_printk("client=%lu", __entry->client)
);


TRACE_EVENT(
	ipa3_napi_poll_exit,

	TP_PROTO(unsigned long client),

	TP_ARGS(client),

	TP_STRUCT__entry(
		__field(unsigned long,	client)
	),

	TP_fast_assign(
		__entry->client = client;
	),

	TP_printk("client=%lu", __entry->client)
);

TRACE_EVENT(
	ipa3_tx_dp,

	TP_PROTO(const struct sk_buff *skb, unsigned long client),

	TP_ARGS(skb, client),

	TP_STRUCT__entry(
		__string(name,			skb->dev->name)
		__field(const void *,	skbaddr)
		__field(u16,			protocol)
		__field(unsigned int,	len)
		__field(unsigned int,	data_len)
		__field(unsigned long,	client)
	),

	TP_fast_assign(
		__assign_str(name, skb->dev->name);
		__entry->skbaddr = skb;
		__entry->protocol = ntohs(skb->protocol);
		__entry->len = skb->len;
		__entry->data_len = skb->data_len;
		__entry->client = client;
	),

	TP_printk("dev=%s skbaddr=%p protocol=0x%04x len=%u data_len=%u client=%lu",
		__get_str(name),
		__entry->skbaddr,
		__entry->protocol,
		__entry->len,
		__entry->data_len,
		__entry->client)
);

TRACE_EVENT(
	ipa3_tx_done,

	TP_PROTO(unsigned long client),

	TP_ARGS(client),

	TP_STRUCT__entry(
		__field(unsigned long,	client)
	),

	TP_fast_assign(
		__entry->client = client;
	),

	TP_printk("client=%lu", __entry->client)
);

TRACE_EVENT(
	ipa3_replenish_rx_page_recycle,

	TP_PROTO(u32 i, struct page *p, bool is_tmp_alloc),

	TP_ARGS(i, p, is_tmp_alloc),

	TP_STRUCT__entry(
		__field(u32, i)
		__field(struct page *,	p)
		__field(bool,		is_tmp_alloc)
		__field(unsigned long,	pfn)
	),

	TP_fast_assign(
		__entry->i = i;
		__entry->p = p;
		__entry->is_tmp_alloc = is_tmp_alloc;
		__entry->pfn = page_to_pfn(p);
	),

	TP_printk("wan_cons type=%u: page=0x%pK pfn=0x%lx tmp=%s",
		__entry->i, __entry->p, __entry->pfn,
		__entry->is_tmp_alloc ? "true" : "false")
);

TRACE_EVENT(
	handle_page_completion,

	TP_PROTO(struct page *p, struct sk_buff *skb, u16 len,
		 bool is_tmp_alloc, enum ipa_client_type client),

	TP_ARGS(p, skb, len, is_tmp_alloc, client),

	TP_STRUCT__entry(
		__field(struct page *,		p)
		__field(struct sk_buff *,	skb)
		__field(u16,			len)
		__field(bool,			is_tmp_alloc)
		__field(unsigned long,		pfn)
		__field(enum ipa_client_type,	client)
	),

	TP_fast_assign(
		__entry->p = p;
		__entry->skb = skb;
		__entry->len = len;
		__entry->is_tmp_alloc = is_tmp_alloc;
		__entry->pfn = page_to_pfn(p);
		__entry->client = client;
	),

	TP_printk("%s: page=0x%pK pfn=0x%lx skb=0x%pK len=%u tmp=%s",
		(__entry->client == IPA_CLIENT_APPS_WAN_CONS) ? "WAN_CONS"
							      : "WAN_COAL_CONS",
		__entry->p, __entry->pfn, __entry->skb, __entry->len,
		__entry->is_tmp_alloc ? "true" : "false")
);

TRACE_EVENT(
	ipa3_rx_napi_chain,

	TP_PROTO(struct sk_buff *first_skb, struct sk_buff *prev_skb,
		 struct sk_buff *rx_skb),

	TP_ARGS(first_skb, prev_skb, rx_skb),

	TP_STRUCT__entry(
		__field(struct sk_buff *,	first_skb)
		__field(struct sk_buff *,	prev_skb)
		__field(struct sk_buff *,	rx_skb)
	),

	TP_fast_assign(
		__entry->first_skb = first_skb;
		__entry->prev_skb = prev_skb;
		__entry->rx_skb = rx_skb;
	),

	TP_printk("first_skb=0x%pK prev_skb=0x%pK rx_skb=0x%pK",
		__entry->first_skb, __entry->prev_skb, __entry->rx_skb)
);

#endif /* _IPA_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#ifdef CONFIG_IPA_VENDOR_DLKM
#define TRACE_INCLUDE_PATH ../../../../vendor/qcom/opensource/dataipa/drivers/platform/msm/ipa/ipa_v3
#else
#define TRACE_INCLUDE_PATH ../../techpack/dataipa/drivers/platform/msm/ipa/ipa_v3
#endif
#include <trace/define_trace.h>
