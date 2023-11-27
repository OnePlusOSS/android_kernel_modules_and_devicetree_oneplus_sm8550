/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2012-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _IPAHAL_REG_H_
#define _IPAHAL_REG_H_

#include <linux/ipa.h>

/*
 * Registers names
 *
 * NOTE:: Any change to this enum, need to change to ipareg_name_to_str
 *	array as well.
 */
enum ipahal_reg_name {
	IPA_ROUTE,
	IPA_IRQ_STTS_EE_n,
	IPA_IRQ_EN_EE_n,
	IPA_IRQ_CLR_EE_n,
	IPA_SUSPEND_IRQ_INFO_EE_n,
	IPA_SUSPEND_IRQ_EN_EE_n,
	IPA_SUSPEND_IRQ_CLR_EE_n,
	IPA_HOLB_DROP_IRQ_INFO_EE_n,
	IPA_HOLB_DROP_IRQ_EN_EE_n,
	IPA_HOLB_DROP_IRQ_CLR_EE_n,
	IPA_BCR,
	IPA_ENABLED_PIPES,
	IPA_VERSION,
	IPA_TAG_TIMER,
	IPA_NAT_TIMER,
	IPA_COMP_HW_VERSION,
	IPA_COMP_CFG,
	IPA_STATE_TX_WRAPPER,
	IPA_STATE_TX1,
	IPA_STATE_FETCHER,
	IPA_STATE_FETCHER_MASK,
	IPA_STATE_FETCHER_MASK_0,
	IPA_STATE_FETCHER_MASK_1,
	IPA_STATE_DFETCHER,
	IPA_STATE_ACL,
	IPA_STATE,
	IPA_STATE_RX_ACTIVE,
	IPA_STATE_TX0,
	IPA_STATE_TSP,
	IPA_STATE_AGGR_ACTIVE,
	IPA_COUNTER_CFG,
	IPA_STATE_GSI_TLV,
	IPA_STATE_GSI_AOS,
	IPA_STATE_GSI_IF,
	IPA_STATE_GSI_SKIP,
	IPA_STATE_GSI_IF_CONS,
	IPA_STATE_DPL_FIFO,
	IPA_STATE_COAL_MASTER,
	IPA_GENERIC_RAM_ARBITER_PRIORITY,
	IPA_STATE_NLO_AGGR,
	IPA_STATE_COAL_MASTER_1,
	IPA_ENDP_INIT_HDR_n,
	IPA_ENDP_INIT_HDR_EXT_n,
	IPA_ENDP_INIT_AGGR_n,
	IPA_AGGR_FORCE_CLOSE,
	IPA_ENDP_INIT_ROUTE_n,
	IPA_ENDP_INIT_MODE_n,
	IPA_ENDP_INIT_NAT_n,
	IPA_ENDP_INIT_CONN_TRACK_n,
	IPA_ENDP_INIT_CTRL_n,
	IPA_ENDP_INIT_CTRL_SCND_n,
	IPA_ENDP_INIT_CTRL_STATUS_n,
	IPA_ENDP_INIT_HOL_BLOCK_EN_n,
	IPA_ENDP_INIT_HOL_BLOCK_TIMER_n,
	IPA_ENDP_INIT_DEAGGR_n,
	IPA_ENDP_INIT_SEQ_n,
	IPA_DEBUG_CNT_REG_n,
	IPA_ENDP_INIT_CFG_n,
	IPA_IRQ_EE_UC_n,
	IPA_ENDP_INIT_HDR_METADATA_MASK_n,
	IPA_ENDP_INIT_HDR_METADATA_n,
	IPA_ENDP_INIT_PROD_CFG_n,
	IPA_ENDP_INIT_RSRC_GRP_n,
	IPA_SHARED_MEM_SIZE,
	IPA_SW_AREA_RAM_DIRECT_ACCESS_n,
	IPA_DEBUG_CNT_CTRL_n,
	IPA_UC_MAILBOX_m_n,
	IPA_FILT_ROUT_HASH_FLUSH,
	IPA_FILT_ROUT_HASH_EN,
	IPA_SINGLE_NDP_MODE,
	IPA_QCNCM,
	IPA_SYS_PKT_PROC_CNTXT_BASE,
	IPA_LOCAL_PKT_PROC_CNTXT_BASE,
	IPA_ENDP_STATUS_n,
	IPA_ENDP_YELLOW_RED_MARKER_CFG_n,
	IPA_ENDP_FILTER_ROUTER_HSH_CFG_n,
	IPA_SRC_RSRC_GRP_01_RSRC_TYPE_n,
	IPA_SRC_RSRC_GRP_23_RSRC_TYPE_n,
	IPA_SRC_RSRC_GRP_45_RSRC_TYPE_n,
	IPA_SRC_RSRC_GRP_67_RSRC_TYPE_n,
	IPA_DST_RSRC_GRP_01_RSRC_TYPE_n,
	IPA_DST_RSRC_GRP_23_RSRC_TYPE_n,
	IPA_DST_RSRC_GRP_45_RSRC_TYPE_n,
	IPA_DST_RSRC_GRP_67_RSRC_TYPE_n,
	IPA_RSRC_GRP_CFG,
	IPA_RSRC_GRP_CFG_EXT,
	IPA_RX_HPS_CLIENTS_MIN_DEPTH_0,
	IPA_RX_HPS_CLIENTS_MIN_DEPTH_1,
	IPA_RX_HPS_CLIENTS_MAX_DEPTH_0,
	IPA_RX_HPS_CLIENTS_MAX_DEPTH_1,
	IPA_HPS_FTCH_ARB_QUEUE_WEIGHT,
	IPA_QSB_MAX_WRITES,
	IPA_QSB_MAX_READS,
	IPA_TX_CFG,
	IPA_IDLE_INDICATION_CFG,
	IPA_DPS_SEQUENCER_FIRST,
	IPA_DPS_SEQUENCER_LAST,
	IPA_HPS_SEQUENCER_FIRST,
	IPA_HPS_SEQUENCER_LAST,
	IPA_CLKON_CFG,
	IPA_QTIME_TIMESTAMP_CFG,
	IPA_TIMERS_PULSE_GRAN_CFG,
	IPA_TIMERS_XO_CLK_DIV_CFG,
	IPA_STAT_QUOTA_BASE_n,
	IPA_STAT_QUOTA_MASK_n,
	IPA_STAT_TETHERING_BASE_n,
	IPA_STAT_TETHERING_MASK_n,
	IPA_STAT_FILTER_IPV4_BASE,
	IPA_STAT_FILTER_IPV6_BASE,
	IPA_STAT_ROUTER_IPV4_BASE,
	IPA_STAT_ROUTER_IPV6_BASE,
	IPA_STAT_FILTER_IPV4_START_ID,
	IPA_STAT_FILTER_IPV6_START_ID,
	IPA_STAT_ROUTER_IPV4_START_ID,
	IPA_STAT_ROUTER_IPV6_START_ID,
	IPA_STAT_FILTER_IPV4_END_ID,
	IPA_STAT_FILTER_IPV6_END_ID,
	IPA_STAT_ROUTER_IPV4_END_ID,
	IPA_STAT_ROUTER_IPV6_END_ID,
	IPA_STAT_DROP_CNT_BASE_n,
	IPA_STAT_DROP_CNT_MASK_n,
	IPA_SNOC_FEC_EE_n,
	IPA_FEC_ADDR_EE_n,
	IPA_FEC_ADDR_MSB_EE_n,
	IPA_FEC_ATTR_EE_n,
	IPA_ENDP_GSI_CFG1_n,
	IPA_ENDP_GSI_CFG_AOS_n,
	IPA_ENDP_GSI_CFG_TLV_n,
	IPA_COAL_EVICT_LRU,
	IPA_COAL_QMAP_CFG,
	IPA_FLAVOR_0,
	IPA_FLAVOR_9,
	IPA_STATE_AGGR_ACTIVE_n,
	IPA_AGGR_FORCE_CLOSE_n,
	IPA_STAT_QUOTA_MASK_EE_n_REG_k,
	IPA_SUSPEND_IRQ_INFO_EE_n_REG_k,
	IPA_SUSPEND_IRQ_CLR_EE_n_REG_k,
	IPA_SUSPEND_IRQ_EN_EE_n_REG_k,
	IPA_STAT_TETHERING_MASK_EE_n_REG_k,
	IPA_STAT_DROP_CNT_MASK_EE_n_REG_k,
	IPA_FILT_ROUT_CACHE_FLUSH,
	IPA_FILTER_CACHE_CFG_n,
	IPA_ROUTER_CACHE_CFG_n,
	IPA_NAT_UC_EXTERNAL_CFG,
	IPA_NAT_UC_LOCAL_CFG,
	IPA_NAT_UC_SHARED_CFG,
	IPA_CONN_TRACK_UC_EXTERNAL_CFG,
	IPA_CONN_TRACK_UC_LOCAL_CFG,
	IPA_CONN_TRACK_UC_SHARED_CFG,
	IPA_ULSO_CFG_IP_ID_MIN_VALUE_n,
	IPA_ULSO_CFG_IP_ID_MAX_VALUE_n,
	IPA_ENDP_INIT_ULSO_CFG_n,
	IPA_ENDP_INIT_NAT_EXC_SUPPRESS_n,
	IPA_TSP_QM_EXTERNAL_BADDR_LSB,
	IPA_TSP_QM_EXTERNAL_BADDR_MSB,
	IPA_TSP_QM_EXTERNAL_SIZE,
	IPA_TSP_INGRESS_POLICING_CFG,
	IPA_TSP_EGRESS_POLICING_CFG,
	IPA_STAT_TSP_DROP_BASE,
	IPA_STATE_QMNGR_QUEUE_NONEMPTY,
	IPA_RAM_INGRESS_POLICER_DB_BASE_ADDR,
	IPA_RAM_EGRESS_SHAPING_PROD_DB_BASE_ADDR,
	IPA_RAM_EGRESS_SHAPING_TC_DB_BASE_ADDR,
	IPA_COAL_MASTER_CFG,
	IPA_IPV4_NAT_EXC_SUPPRESS_ROUT_TABLE_INDX,
	IPA_IPV6_CONN_TRACK_EXC_SUPPRESS_ROUT_TABLE_INDX,
	IPA_REG_MAX,
};

/*
 * struct ipahal_reg_route - IPA route register
 * @route_dis: route disable
 * @route_def_pipe: route default pipe
 * @route_def_hdr_table: route default header table
 * @route_def_hdr_ofst: route default header offset table
 * @route_frag_def_pipe: Default pipe to route fragmented exception
 *    packets and frag new rule statues, if source pipe does not have
 *    a notification status pipe defined.
 * @route_def_retain_hdr: default value of retain header. It is used
 *    when no rule was hit
 */
struct ipahal_reg_route {
	u32 route_dis;
	u32 route_def_pipe;
	u32 route_def_hdr_table;
	u32 route_def_hdr_ofst;
	u8  route_frag_def_pipe;
	u32 route_def_retain_hdr;
};

/*
 * struct ipahal_reg_endp_init_route - IPA ENDP_INIT_ROUTE_n register
 * @route_table_index: Default index of routing table (IPA Consumer).
 */
struct ipahal_reg_endp_init_route {
	u32 route_table_index;
};

/*
 * struct ipahal_reg_endp_init_rsrc_grp - IPA_ENDP_INIT_RSRC_GRP_n register
 * @rsrc_grp: Index of group for this ENDP. If this ENDP is a source-ENDP,
 *	index is for source-resource-group. If destination ENPD, index is
 *	for destination-resoruce-group.
 */
struct ipahal_reg_endp_init_rsrc_grp {
	u32 rsrc_grp;
};

/*
 * struct ipahal_reg_endp_init_mode - IPA ENDP_INIT_MODE_n register
 * @dst_pipe_number: This parameter specifies destination output-pipe-packets
 *	will be routed to. Valid for DMA mode only and for Input
 *	Pipes only (IPA Consumer)
 */
struct ipahal_reg_endp_init_mode {
	u32 dst_pipe_number;
	struct ipa_ep_cfg_mode ep_mode;
};

/*
 * struct ipahal_reg_shared_mem_size - IPA_SHARED_MEM_SIZE register
 * @shared_mem_sz: Available size [in 8Bytes] of SW partition within
 *	IPA shared memory.
 * @shared_mem_baddr: Offset of SW partition within IPA
 *	shared memory[in 8Bytes]. To get absolute address of SW partition,
 *	add this offset to IPA_SW_AREA_RAM_DIRECT_ACCESS_n baddr.
 */
struct ipahal_reg_shared_mem_size {
	u32 shared_mem_sz;
	u32 shared_mem_baddr;
};

/*
 * struct ipahal_reg_ep_cfg_status - status configuration in IPA end-point
 * @status_en: Determines if end point supports Status Indications. SW should
 *	set this bit in order to enable Statuses. Output Pipe - send
 *	Status indications only if bit is set. Input Pipe - forward Status
 *	indication to STATUS_ENDP only if bit is set. Valid for Input
 *	and Output Pipes (IPA Consumer and Producer)
 * @status_ep: Statuses generated for this endpoint will be forwarded to the
 *	specified Status End Point. Status endpoint needs to be
 *	configured with STATUS_EN=1 Valid only for Input Pipes (IPA
 *	Consumer)
 * @status_location: Location of PKT-STATUS on destination pipe.
 *	If set to 0 (default), PKT-STATUS will be appended before the packet
 *	for this endpoint. If set to 1, PKT-STATUS will be appended after the
 *	packet for this endpoint. Valid only for Output Pipes (IPA Producer)
 * @status_pkt_suppress: Disable notification status, when statistics is enabled
 */
struct ipahal_reg_ep_cfg_status {
	bool status_en;
	u8 status_ep;
	bool status_location;
	u8 status_pkt_suppress;
};

/*
 * struct ipahal_reg_clkon_cfg-  Enables SW bypass clock-gating for the IPA core
 *
 * @all: Enables SW bypass clock-gating controls for this sub-module;
 *	0: CGC is enabled by internal logic, 1: No CGC (clk is always 'ON').
 *	sub-module affected is based on var name -> ex: open_rx refers
 *	to IPA_RX sub-module and open_global refers to global IPA 1x clock
 */
struct ipahal_reg_clkon_cfg {
	bool open_dpl_fifo;
	bool open_global_2x_clk;
	bool open_global;
	bool open_gsi_if;
	bool open_weight_arb;
	bool open_qmb;
	bool open_ram_slaveway;
	bool open_aggr_wrapper;
	bool open_qsb2axi_cmdq_l;
	bool open_fnr;
	bool open_tx_1;
	bool open_tx_0;
	bool open_ntf_tx_cmdqs;
	bool open_dcmp;
	bool open_h_dcph;
	bool open_d_dcph;
	bool open_ack_mngr;
	bool open_ctx_handler;
	bool open_rsrc_mngr;
	bool open_dps_tx_cmdqs;
	bool open_hps_dps_cmdqs;
	bool open_rx_hps_cmdqs;
	bool open_dps;
	bool open_hps;
	bool open_ftch_dps;
	bool open_ftch_hps;
	bool open_ram_arb;
	bool open_misc;
	bool open_tx_wrapper;
	bool open_proc;
	bool open_rx;
};

/*
 * struct ipahal_reg_qtime_timestamp_cfg - IPA timestamp configuration
 *  Relevant starting IPA 4.5.
 *  IPA timestamps are based on QTIMER which is 56bit length which is
 *  based on XO clk running at 19.2MHz (52nsec resolution).
 *  Specific timestamps (TAG, NAT, DPL) my require lower resolution.
 *  This can be achieved by omitting LSB bits from 56bit QTIMER.
 *  e.g. if we omit (shift) 24 bit then we get (2^24)*(52n)=0.87sec resolution.
 *
 * @dpl_timestamp_lsb: Shifting Qtime value. Value will be used as LSB of
 *  DPL timestamp.
 * @dpl_timestamp_sel: if false, DPL timestamp will be based on legacy
 *  DPL_TIMER which counts in 1ms. if true, it will be based on QTIME
 *  value shifted by dpl_timestamp_lsb.
 * @tag_timestamp_lsb: Shifting Qtime value. Value will be used as LSB of
 *  TAG timestamp.
 * @nat_timestamp_lsb: Shifting Qtime value. Value will be used as LSB of
 *  NAT timestamp.
 */
struct ipahal_reg_qtime_timestamp_cfg {
	u32 dpl_timestamp_lsb;
	bool dpl_timestamp_sel;
	u32 tag_timestamp_lsb;
	u32 nat_timestamp_lsb;
};

/*
 * enum ipa_timers_time_gran_type - Time granularity to be used with timers
 *
 * e.g. for HOLB and Aggregation timers
 */
enum ipa_timers_time_gran_type {
	IPA_TIMERS_TIME_GRAN_10_USEC,
	IPA_TIMERS_TIME_GRAN_20_USEC,
	IPA_TIMERS_TIME_GRAN_50_USEC,
	IPA_TIMERS_TIME_GRAN_100_USEC,
	IPA_TIMERS_TIME_GRAN_1_MSEC,
	IPA_TIMERS_TIME_GRAN_10_MSEC,
	IPA_TIMERS_TIME_GRAN_100_MSEC,
	IPA_TIMERS_TIME_GRAN_NEAR_HALF_SEC, /* 0.65536s */
	IPA_TIMERS_TIME_GRAN_MAX,
};

/*
 * struct ipahal_reg_timers_pulse_gran_cfg - Counters tick granularities
 *  Relevant starting IPA 4.5.
 *  IPA timers are based on XO CLK running 19.2MHz (52ns resolution) deviced
 *  by clock divider (see IPA_TIMERS_XO_CLK_DIV_CFG) - default 100Khz (10usec).
 *  IPA timers instances (e.g. HOLB or AGGR) may require different resolutions.
 *  There are 3 global pulse generators with configurable granularity. Each
 *  timer instance can choose one of the three generators to work with.
 *  Each generator granularity can be one of supported ones.
 *
 * @gran_X: granularity tick of counterX
 */
struct ipahal_reg_timers_pulse_gran_cfg {
	enum ipa_timers_time_gran_type gran_0;
	enum ipa_timers_time_gran_type gran_1;
	enum ipa_timers_time_gran_type gran_2;
	enum ipa_timers_time_gran_type gran_3;
};

/*
 * struct ipahal_reg_timers_xo_clk_div_cfg - IPA timers clock divider
 * Used to control clock divider which gets XO_CLK of 19.2MHz as input.
 * Output of CDIV is used to generate IPA timers granularity
 *
 * @enable: Enable of the clock divider for all IPA and GSI timers.
 *  clock is disabled by default, and need to be enabled when system is up.
 * @value: Divided value to be used by CDIV. POR value is set to 191
 *  to generate 100KHz clk based on XO_CLK.
 *  Values of ipahal_reg_timers_pulse_gran_cfg are based on this default.
 */
struct ipahal_reg_timers_xo_clk_div_cfg {
	bool enable;
	u32 value;
};

/*
 * struct ipahal_reg_comp_cfg- IPA Core QMB/Master Port selection
 *
 * @enable / @ipa_dcmp_fast_clk_en: are not relevant starting IPA4.5
 * @ipa_full_flush_wait_rsc_closure_en: relevant starting IPA4.5
 */
struct ipahal_reg_comp_cfg {
	bool gen_qmb_0_dynamic_asize;
	bool gen_qmb_1_dynamic_asize;
	bool ipa_full_flush_wait_rsc_closure_en;
	u8 ipa_atomic_fetcher_arb_lock_dis;
	bool gsi_if_out_of_buf_stop_reset_mask_enable;
	bool genqmb_aooowr;
	bool qmb_ram_rd_cache_disable;
	bool ipa_qmb_select_by_address_global_en;
	bool gsi_multi_axi_masters_dis;
	bool gsi_snoc_cnoc_loop_protection_disable;
	bool gen_qmb_0_snoc_cnoc_loop_protection_disable;
	bool gen_qmb_1_multi_inorder_wr_dis;
	bool gen_qmb_0_multi_inorder_wr_dis;
	bool gen_qmb_1_multi_inorder_rd_dis;
	bool gen_qmb_0_multi_inorder_rd_dis;
	bool gsi_multi_inorder_wr_dis;
	bool gsi_multi_inorder_rd_dis;
	bool ipa_qmb_select_by_address_prod_en;
	bool ipa_qmb_select_by_address_cons_en;
	bool ipa_dcmp_fast_clk_en;
	bool gen_qmb_1_snoc_bypass_dis;
	bool gen_qmb_0_snoc_bypass_dis;
	bool gsi_snoc_bypass_dis;
	bool ram_arb_priority_client_samp_fix_disable;
	bool enable;
};

/*
 * struct ipahal_reg_tx_wrapper- IPA TX Wrapper state information
 */
struct ipahal_reg_tx_wrapper {
	bool tx0_idle;
	bool tx1_idle;
	bool ipa_prod_ackmngr_db_empty;
	bool ipa_prod_ackmngr_state_idle;
	bool ipa_prod_prod_bresp_empty;
	bool ipa_prod_prod_bresp_toggle_idle;
	bool ipa_mbim_pkt_fms_idle;
	u8 mbim_direct_dma;
	bool trnseq_force_valid;
	bool pkt_drop_cnt_idle;
	u8 nlo_direct_dma;
	u8 coal_direct_dma;
	bool coal_slave_idle;
	bool coal_slave_ctx_idle;
	u8 coal_slave_open_frame;
};

/*
 * struct ipa_hash_tuple - Hash tuple members for flt and rt
 *  the fields tells if to be masked or not
 * @src_id: pipe number for flt, table index for rt
 * @src_ip_addr: IP source address
 * @dst_ip_addr: IP destination address
 * @src_port: L4 source port
 * @dst_port: L4 destination port
 * @protocol: IP protocol field
 * @meta_data: packet meta-data
 *
 */
struct ipahal_reg_hash_tuple {
	/* src_id: pipe in flt, tbl index in rt */
	bool src_id;
	bool src_ip_addr;
	bool dst_ip_addr;
	bool src_port;
	bool dst_port;
	bool protocol;
	bool meta_data;
};

/*
 * struct ipahal_reg_fltrt_hash_tuple - IPA hash tuple register
 * @flt: Hash tuple info for filtering
 * @rt: Hash tuple info for routing
 * @undefinedX: Undefined/Unused bit fields set of the register
 */
struct ipahal_reg_fltrt_hash_tuple {
	struct ipahal_reg_hash_tuple flt;
	struct ipahal_reg_hash_tuple rt;
	u32 undefined1;
	u32 undefined2;
};

/*
* struct ipahal_reg_fltrt_cache_tuple - IPA cache tuple register
* @flt: cache tuple info for flt\rt
* @undefinedX: Undefined/Unused bit fields set of the register
*/
struct ipahal_reg_fltrt_cache_tuple {
	struct ipahal_reg_hash_tuple tuple;
	u32 undefined;
};

/*
 * enum ipahal_reg_dbg_cnt_type - Debug Counter Type
 * DBG_CNT_TYPE_IPV4_FLTR - Count IPv4 filtering rules
 * DBG_CNT_TYPE_IPV4_ROUT - Count IPv4 routing rules
 * DBG_CNT_TYPE_GENERAL - General counter
 * DBG_CNT_TYPE_IPV6_FLTR - Count IPv6 filtering rules
 * DBG_CNT_TYPE_IPV4_ROUT - Count IPv6 routing rules
 */
enum ipahal_reg_dbg_cnt_type {
	DBG_CNT_TYPE_IPV4_FLTR,
	DBG_CNT_TYPE_IPV4_ROUT,
	DBG_CNT_TYPE_GENERAL,
	DBG_CNT_TYPE_IPV6_FLTR,
	DBG_CNT_TYPE_IPV6_ROUT,
};

/*
 * struct ipahal_reg_debug_cnt_ctrl - IPA_DEBUG_CNT_CTRL_n register
 * @en - Enable debug counter
 * @type - Type of debugging couting
 * @product - False->Count Bytes . True->Count #packets
 * @src_pipe - Specific Pipe to match. If FF, no need to match
 *	specific pipe
 * @rule_idx_pipe_rule - Global Rule or Pipe Rule. If pipe, then indicated by
 *	src_pipe. Starting at IPA V3_5,
 *	no support on Global Rule. This field will be ignored.
 * @rule_idx - Rule index. Irrelevant for type General
 */
struct ipahal_reg_debug_cnt_ctrl {
	bool en;
	enum ipahal_reg_dbg_cnt_type type;
	bool product;
	u8 src_pipe;
	bool rule_idx_pipe_rule;
	u16 rule_idx;
};

/*
 * struct ipahal_reg_rsrc_grp_xy_cfg - Min/Max values for two rsrc groups
 * @x_min - first group min value
 * @x_max - first group max value
 * @y_min - second group min value
 * @y_max - second group max value
 */
struct ipahal_reg_rsrc_grp_xy_cfg {
	u32 x_min;
	u32 x_max;
	u32 y_min;
	u32 y_max;
};

/*
 * struct ipahal_reg_rsrc_grp_cfg - General configuration of resource group behavior
 * @src_grp_index - Index of special source resource group
 * @src_grp_valid - Set to 1 if a special source resrouce group exists
 * @dst_pipe_index - Index of special destination pipe
 * @dst_pipe_valid - Set to 1 if a special destination pipe exists
 * @dst_grp_index - Index of special destination resource group
 * @dst_grp_valid - Set to 1 if a special destination resrouce group exists
 */
struct ipahal_reg_rsrc_grp_cfg {
	u8 src_grp_index;
	bool src_grp_valid;
	u8 dst_pipe_index;
	bool dst_pipe_valid;
	u8 dst_grp_index;
	bool dst_grp_valid;
};

/*
 * struct ipahal_reg_rsrc_grp_cfg_ext - General configuration of resource group behavior extended
 * @index - Index of 2nd-priority special source resource group.
 * 	Will be chosen only in case 1st-level priority group is not requesting service.
 * @valid - Set to 1 if a 2nd-priority special source resrouce group exists
 */
struct ipahal_reg_rsrc_grp_cfg_ext {
	u8 index;
	bool valid;
};

/*
 * struct ipahal_reg_rx_hps_clients - Min or Max values for RX HPS clients
 * @client_minmax - Min or Max values. In case of depth 0 the 4 or 5 values
 *	are used. In case of depth 1, only the first 2 values are used
 */
struct ipahal_reg_rx_hps_clients {
	u32 client_minmax[5];
};

/*
 * struct ipahal_reg_rx_hps_weights - weight values for RX HPS clients
 * @hps_queue_weight_0 - 4 bit Weight for RX_HPS_CMDQ #0 (3:0)
 * @hps_queue_weight_1 - 4 bit Weight for RX_HPS_CMDQ #1 (7:4)
 * @hps_queue_weight_2 - 4 bit Weight for RX_HPS_CMDQ #2 (11:8)
 * @hps_queue_weight_3 - 4 bit Weight for RX_HPS_CMDQ #3 (15:12)
 */
struct ipahal_reg_rx_hps_weights {
	u32 hps_queue_weight_0;
	u32 hps_queue_weight_1;
	u32 hps_queue_weight_2;
	u32 hps_queue_weight_3;
};

/*
 * struct ipahal_reg_counter_cfg - granularity of counter registers
 * @aggr_granularity  -Defines the granularity of AGGR timers
 *	granularity [msec]=(x+1)/(32)
 */
struct ipahal_reg_counter_cfg {
	enum {
		GRAN_VALUE_125_USEC = 3,
		GRAN_VALUE_250_USEC = 7,
		GRAN_VALUE_500_USEC = 15,
		GRAN_VALUE_MSEC = 31,
	} aggr_granularity;
};


/*
 * struct ipahal_reg_valmask - holding values and masking for registers
 *	HAL application may require only value and mask of it for some
 *	register fields.
 * @val - The value
 * @mask - Tha mask of the value
 */
struct ipahal_reg_valmask {
	u32 val;
	u32 mask;
};

/*
 * struct ipahal_reg_fltrt_hash_flush - Flt/Rt flush configuration
 * @v6_rt - Flush IPv6 Routing cache
 * @v6_flt - Flush IPv6 Filtering cache
 * @v4_rt - Flush IPv4 Routing cache
 * @v4_flt - Flush IPv4 Filtering cache
 */
struct ipahal_reg_fltrt_hash_flush {
	bool v6_rt;
	bool v6_flt;
	bool v4_rt;
	bool v4_flt;
};

/*
* struct ipahal_reg_fltrt_cache_flush - Flt/Rt flush configuration
* @rt - Flush Routing cache
* @flt - Flush Filtering cache
*/
struct ipahal_reg_fltrt_cache_flush {
	bool rt;
	bool flt;
};

/*
 * struct ipahal_reg_single_ndp_mode - IPA SINGLE_NDP_MODE register
 * @single_ndp_en: When set to '1', IPA builds MBIM frames with up to 1
 *	NDP-header.
 * @unused: undefined bits of the register
 */
struct ipahal_reg_single_ndp_mode {
	bool single_ndp_en;
	u32 undefined;
};

/*
 * struct ipahal_reg_qcncm - IPA QCNCM register
 * @mode_en: When QCNCM_MODE_EN=1, IPA will use QCNCM signature.
 * @mode_val: Used only when QCNCM_MODE_EN=1 and sets SW Signature in
 *	the NDP header.
 * @unused: undefined bits of the register
 */
struct ipahal_reg_qcncm {
	bool mode_en;
	u32 mode_val;
	u32 undefined;
};

/*
 * struct ipahal_reg_qsb_max_writes - IPA QSB Max Writes register
 * @qmb_0_max_writes: Max number of outstanding writes for GEN_QMB_0
 * @qmb_1_max_writes: Max number of outstanding writes for GEN_QMB_1
 */
struct ipahal_reg_qsb_max_writes {
	u32 qmb_0_max_writes;
	u32 qmb_1_max_writes;
};

/*
 * struct ipahal_reg_qsb_max_reads - IPA QSB Max Reads register
 * @qmb_0_max_reads: Max number of outstanding reads for GEN_QMB_0
 * @qmb_1_max_reads: Max number of outstanding reads for GEN_QMB_1
 * @qmb_0_max_read_beats: Max number of outstanding read beats for GEN_QMB_0
 * @qmb_1_max_read_beats: Max number of outstanding read beats for GEN_QMB_1
 */
struct ipahal_reg_qsb_max_reads {
	u32 qmb_0_max_reads;
	u32 qmb_1_max_reads;
	u32 qmb_0_max_read_beats;
	u32 qmb_1_max_read_beats;
};

/*
 * struct ipahal_reg_tx_cfg - IPA TX_CFG register
 * @tx0_prefetch_disable: Disable prefetch on TX0
 * @tx1_prefetch_disable: Disable prefetch on TX1
 * @tx0_prefetch_almost_empty_size: Prefetch almost empty size on TX0
 * @tx1_prefetch_almost_empty_size: Prefetch almost empty size on TX1
 * @dmaw_scnd_outsd_pred_threshold: threshold for DMAW_SCND_OUTSD_PRED_EN
 * @dmaw_max_beats_256_dis:
 * @dmaw_scnd_outsd_pred_en:
 * @pa_mask_en:
 * @dual_tx_enable: When 1 TX0 and TX1 are enabled. When 0 only TX0 is enabled
 * @sspnd_pa_no_start_state: When 1 sspnd_req does not take inco account
			     PA FSM state START.
			     When 0 sspnd_req_ will not be answered
			     on that state.
 *  Relevant starting IPA4.5
 */
struct ipahal_reg_tx_cfg {
	bool tx0_prefetch_disable;
	bool tx1_prefetch_disable;
	u32 tx0_prefetch_almost_empty_size;
	u32 tx1_prefetch_almost_empty_size;
	u32 dmaw_scnd_outsd_pred_threshold;
	u32 dmaw_max_beats_256_dis;
	u32 dmaw_scnd_outsd_pred_en;
	u32 pa_mask_en;
	bool dual_tx_enable;
	bool sspnd_pa_no_start_state;
	bool holb_sticky_drop_en;
};

/*
 * struct ipahal_reg_idle_indication_cfg - IPA IDLE_INDICATION_CFG register
 * @const_non_idle_enable: enable the asserting of the IDLE value and DCD
 * @enter_idle_debounce_thresh:  configure the debounce threshold
 */
struct ipahal_reg_idle_indication_cfg {
	u16 enter_idle_debounce_thresh;
	bool const_non_idle_enable;
};

/*
 * struct ipa_ep_cfg_ctrl_scnd - IPA_ENDP_INIT_CTRL_SCND_n register
 * @endp_delay: delay endpoint
 */
struct ipahal_ep_cfg_ctrl_scnd {
	bool endp_delay;
};

/*
 * struct ipahal_reg_state_coal_master- IPA_STATE_COAL_MASTER register
 * @vp_timer_expired: VP bitmap. If set, Vp aggregation timer has expired
 * @lru_cp: least recently used VP index
 * @init_vp_fsm_state: init VP FSM current state
 * @check_fir_fsm_state: check fir FMS current state
 * @hash_calc_fsm_state: hash calculation FSM current state
 * @find_open_fsm_state: find open VP FSM current state
 * @main_fsm_state: main coalescing master state FSM current state
 * @vp_vld: VP bitmap. If set, VP is valid, and coalescing frame is open.
 */
struct ipahal_reg_state_coal_master {
	u32 vp_timer_expired;
	u32 lru_vp;
	u32 init_vp_fsm_state;
	u32 check_fir_fsm_state;
	u32 hash_calc_fsm_state;
	u32 find_open_fsm_state;
	u32 main_fsm_state;
	u32 vp_vld;
};

/*
 * struct ipahal_reg_coal_evict_lru - IPA_COAL_EVICT_LRU register
 * @coal_vp_lru_thrshld: Connection that is opened below  this val
 *			 will not get evicted. valid till v5_2.
 * @coal_eviction_en: Enable eviction
 * @coal_vp_lru_gran_sel: select the appropiate granularity out of 4 options
 * Valid from v5_5.
 * @coal_vp_lru_udp_thrshld: Coalescing eviction threshold. LRU VP
 * stickness/inactivity defined by this threshold fot UDP connectiom.
 * 0 mean all UDP's non sticky. Valid from v5_5.
 * @coal_vp_lru_tcp_thrshld: Coalescing eviction threshold. LRU VP
 * stickness/inactivity defined by this threshold fot TCP connection.
 * 0 mean all TCP's non sticky. Valid from v5_5.
 * @coal_vp_lru_udp_thrshld_en: Coalescing eviction enable for UDP connections
 * when UDP pacjet arrived. 0-disable these evictions. Valid from v5_5.
 * @coal_vp_lru_tcp_thrshld_en: Coalescing eviction enable for TCP connections
 * when TCP pacjet arrived. 0-disable these evictions. Valid from v5_5.
 * @coal_vp_lru_tcp_num: configured TCP NUM value , SW define when TCP/UDP will
 * treat as exceed during eviction process. Valid from v5_5.
 */
struct ipahal_reg_coal_evict_lru {
	u32 coal_vp_lru_thrshld;
	bool coal_eviction_en;
	u8 coal_vp_lru_gran_sel;
	u8 coal_vp_lru_udp_thrshld;
	u8 coal_vp_lru_tcp_thrshld;
	bool coal_vp_lru_udp_thrshld_en;
	bool coal_vp_lru_tcp_thrshld_en;
	bool coal_vp_lru_tcp_num;
};

/*
 * struct ipahal_reg_coal_qmap_cfg - IPA_COAL_QMAP_CFG register
 * @mux_id_byte_sel: MUX_ID field in the QMAP portion in COALESCING header is
 * taken from injected packet metadata field in PKT_CTX.
 * Metadata consists of 4 bytes, configuring value 0 to MUX_ID_BYTE_SEL will
 * take bits 7:0 from metadata field, value 1 will take bits 15:8 and so on ...
 */
struct ipahal_reg_coal_qmap_cfg {
	u32 mux_id_byte_sel;
};

/*
 * struct ipahal_reg_coal_master_cfg - IPA_COAL_MASTER_CFG register
 * @coal_ipv4_id_ignore: 1 - global ignore IPV4 ID checks regardles DF,
 * val 0 -keep checks according to DF/MF  conditions.
 * @coal_enhanced_ipv4_id_en: 1 - if (DF == 1 && MF == 0 && frag_offset == 0)
 * Coalescingwill ignore IPv4 identification field, else legacy behaviour
 * is used.
 * 0 - Coalescing will use original legacy IPv4 identification field check.
 * @coal_force_to_default: 1 - force any new packet that arrives to coal master
 * to default pipe, and close any open frames with the same tuple
 * 0 - regular coalescing activity.
 */
struct ipahal_reg_coal_master_cfg {
	bool coal_ipv4_id_ignore;
	bool coal_enhanced_ipv4_id_en;
	bool coal_force_to_default;
};

/*
 * struct ipahal_ipa_flavor_0 - IPA_FLAVOR_0 register
 * @ipa_pipes: Number of supported pipes
 * @ipa_cons_pipes: Number of consumer pipes
 * @ipa_prod_pipes: Number of producer pipes
 * @ipa_prod_lowest: Number of first producer pipe
 */
struct ipahal_ipa_flavor_0 {
	u8 ipa_pipes;
	u8 ipa_cons_pipes;
	u8 ipa_prod_pipes;
	u8 ipa_prod_lowest;
};

/*
 * struct ipahal_ipa_flavor_9 - IPA_FLAVOR_9 register
 * @ipa_tsp_max_ingr_tc: Maximal number of ingress (consumer-based) traffic-classes.
 *                       Does not include invalid traffic-class 0x00.
 * @ipa_tsp_max_egr_tc: Maximal number of egress (producer-based) traffic-classes.
 *                      Does not include invalid traffic-class 0x00.
 * @ipa_tsp_max_prod: Maximal number of TSP-enabled producers.
 * @reserved: Reserved
 */
struct ipahal_ipa_flavor_9 {
	u8 ipa_tsp_max_ingr_tc;
	u8 ipa_tsp_max_egr_tc;
	u8 ipa_tsp_max_prod;
	u8 reserved;
};

/*
 * struct ipahal_ipa_state_tsp - TSP engine state register
 * @traffic_shaper_idle: Traffic-Shaper module IDLE indication
 * @traffic_shaper_fifo_empty: Traffic-Shaper FIFO empty indication
 * @queue_mngr_idle: QMNGR overall IDLE indication
 * @queue_mngr_head_idle: QMNGR head module IDLE indication
 * @queue_mngr_shared_idle: QMNGR shared module IDLE indication
 * @queue_mngr_tail_idle: QMNGR tail module IDLE indication
 * @queue_mngr_block_ctrl_idle: Block control module IDLE indication
 */
struct ipahal_ipa_state_tsp {
	bool traffic_shaper_idle;
	bool traffic_shaper_fifo_empty;
	bool queue_mngr_idle;
	bool queue_mngr_head_idle;
	bool queue_mngr_shared_idle;
	bool queue_mngr_tail_idle;
	bool queue_mngr_block_ctrl_idle;
};

/*
 * struct ipahal_reg_nat_uc_local_cfg -  IPA_NAT_UC_EXTERNAL_CFG register
 * @nat_uc_external_table_addr_lsb: 32 LSb bits of system-memory address of
 * external UC-activation entry table.
 */
struct ipahal_reg_nat_uc_external_cfg {
	u32 nat_uc_external_table_addr_lsb;
};

/*
 * struct ipahal_reg_nat_uc_local_cfg - IPA_NAT_UC_LOCAL_CFG register
 * @nat_uc_local_table_addr_lsb: 32 LSb bits of local address of local
 * UC-activation entry table. Address is memory-map based,
 * i.e. includes IPA address from chip level.
 */
struct ipahal_reg_nat_uc_local_cfg {
	u32 nat_uc_local_table_addr_lsb;
};

/*
 * struct ipahal_reg_nat_uc_shared_cfg -  IPA_NAT_UC_SHARED_CFG register
 * @nat_uc_external_table_addr_msb: 16 MSb of external UC-ativation entry table.
 * @nat_uc_local_table_addr_msb: 16 MSb bits of local UC-ativation entry table.
 */
struct ipahal_reg_nat_uc_shared_cfg {
	u32 nat_uc_local_table_addr_msb;
	u32 nat_uc_external_table_addr_msb;
};

/*
 * struct ipahal_reg_conn_track_uc_local_cfg - IPA_conn_track_UC_EXTERNAL_CFG
 * register
 * @conn_track_uc_external_table_addr_lsb: 32 LSb bits of system-memory address
 * of external UC-activation entry table.
 */
struct ipahal_reg_conn_track_uc_external_cfg {
	u32 conn_track_uc_external_table_addr_lsb;
};

/*
 * struct ipahal_reg_conn_track_uc_local_cfg - IPA_conn_track_UC_LOCAL_CFG
 * register
 * @conn_track_uc_local_table_addr_lsb: 32 LSb bits of local address of local
 * UC-activation entry table. Address is memory-map based,
 * i.e. includes IPA address from chip level.
 */
struct ipahal_reg_conn_track_uc_local_cfg {
	u32 conn_track_uc_local_table_addr_lsb;
};

/*
 * struct ipahal_reg_conn_track_uc_shared_cfg -  IPA_conn_track_UC_SHARED_CFG
 * register
 * @conn_track_uc_external_table_addr_msb: 16 MSb of external UC-ativation
 * entry table.
 * @conn_track_uc_local_table_addr_msb: 16 MSb bits of local UC-ativation
 * entry table.
 */
struct ipahal_reg_conn_track_uc_shared_cfg {
	u16 conn_track_uc_local_table_addr_msb;
	u16 conn_track_uc_external_table_addr_msb;
};

/*
 * ipahal_print_all_regs() - Loop and read and print all the valid registers
 *  Parameterized registers are also printed for all the valid ranges.
 *  Print to dmsg and IPC logs
 */
void ipahal_print_all_regs(bool print_to_dmesg);

/*
 * ipahal_reg_name_str() - returns string that represent the register
 * @reg_name: [in] register name
 */
const char *ipahal_reg_name_str(enum ipahal_reg_name reg_name);

/*
 * ipahal_read_reg_n() - Get the raw value of n parameterized reg
 */
u32 ipahal_read_reg_n(enum ipahal_reg_name reg, u32 n);

/*
 * ipahal_read_reg_mn() - Get mn parameterized reg value
 */
u32 ipahal_read_reg_mn(enum ipahal_reg_name reg, u32 m, u32 n);

/*
* ipahal_read_reg_nk() - Read from n/k parameterized reg
*/
static inline u32 ipahal_read_reg_nk(enum ipahal_reg_name reg, u32 n, u32 k)
{
	return ipahal_read_reg_mn(reg, k, n);
}

/*
* ipahal_read_ep_reg_n() - Get n parameterized reg value according to ep
*/
u32 ipahal_read_ep_reg_n(enum ipahal_reg_name reg, u32 n, u32 ep_num);

/*
 * ipahal_write_reg_mn() - Write to m/n parameterized reg a raw value
 */
void ipahal_write_reg_mn(enum ipahal_reg_name reg, u32 m, u32 n, u32 val);

/*
* ipahal_write_reg_nk() - Write to n/k parameterized reg a raw value
*/
static inline void ipahal_write_reg_nk(
	enum ipahal_reg_name reg, u32 n, u32 k, u32 val)
{
	ipahal_write_reg_mn(reg, k, n, val);
}

/*
 * ipahal_write_reg_n() - Write to n parameterized reg a raw value
 */
static inline void ipahal_write_reg_n(enum ipahal_reg_name reg,
	u32 n, u32 val)
{
	ipahal_write_reg_mn(reg, 0, n, val);
}

/*
 * ipahal_read_reg_n_fields() - Get the parsed value of n parameterized reg
 */
u32 ipahal_read_reg_n_fields(enum ipahal_reg_name reg, u32 n, void *fields);

/*
 * ipahal_write_reg_n_fields() - Write to n parameterized reg a prased value
 */
void ipahal_write_reg_n_fields(enum ipahal_reg_name reg, u32 n,
	const void *fields);

/*
 * ipahal_read_reg() - Get the raw value of a reg
 */
static inline u32 ipahal_read_reg(enum ipahal_reg_name reg)
{
	return ipahal_read_reg_n(reg, 0);
}

/*
 * ipahal_read_ep_reg() - Get the raw value of a ep reg
 */
u32 ipahal_read_ep_reg(enum ipahal_reg_name reg, u32 ep_num);

/*
 * ipahal_write_reg() - Write to reg a raw value
 */
static inline void ipahal_write_reg(enum ipahal_reg_name reg,
	u32 val)
{
	ipahal_write_reg_mn(reg, 0, 0, val);
}

/*
 * ipahal_write_reg_mask() - Overwrite a masked raw value in reg
 */
static inline void ipahal_write_reg_mask(enum ipahal_reg_name reg, u32 val, u32 mask)
{
	u32 new_val = ipahal_read_reg(reg);
	new_val &= !mask;
	new_val &= (val & mask);
	ipahal_write_reg(reg, val);
}

/*
 * ipahal_read_reg_fields() - Get the parsed value of a reg
 */
static inline u32 ipahal_read_reg_fields(enum ipahal_reg_name reg, void *fields)
{
	return ipahal_read_reg_n_fields(reg, 0, fields);
}

/*
 * ipahal_write_reg_fields() - Write to reg a parsed value
 */
static inline void ipahal_write_reg_fields(enum ipahal_reg_name reg,
	const void *fields)
{
	ipahal_write_reg_n_fields(reg, 0, fields);
}

/*
 * ipahal_write_ep_reg() - Write to ep reg a raw value
 */
void ipahal_write_ep_reg(enum ipahal_reg_name reg, u32 ep_num, u32 val);

/*
 * ipahal_write_ep_reg_n() - Write to ep reg a raw value
 */
void ipahal_write_ep_reg_n(enum ipahal_reg_name reg, u32 n, u32 ep_num, u32 val);

/*
 * Get the offset of a m/n parameterized register
 */
u32 ipahal_get_reg_mn_ofst(enum ipahal_reg_name reg, u32 m, u32 n);

/*
* Get the offset of a n,k register
*/
u32 ipahal_get_reg_nk_offset(enum ipahal_reg_name reg, u32 n, u32 l);

/*
 * Get the offset of a n parameterized register
 */
static inline u32 ipahal_get_reg_n_ofst(enum ipahal_reg_name reg, u32 n)
{
	return ipahal_get_reg_mn_ofst(reg, 0, n);
}

/*
 * Get the offset of a ep register according to ep index
 */
u32 ipahal_get_ep_reg_offset(enum ipahal_reg_name reg, u32 ep_num);

/*
 * Get the offset of a register
 */
static inline u32 ipahal_get_reg_ofst(enum ipahal_reg_name reg)
{
	return ipahal_get_reg_mn_ofst(reg, 0, 0);
}

/*
 * Get the register base address
 */
u32 ipahal_get_reg_base(void);

/*
 * Specific functions
 * These functions supply specific register values for specific operations
 *  that cannot be reached by generic functions.
 * E.g. To disable aggregation, need to write to specific bits of the AGGR
 *  register. The other bits should be untouched. This operation is very
 *  specific and cannot be generically defined. For such operations we define
 *  these specific functions.
 */
u32 ipahal_aggr_get_max_byte_limit(void);
u32 ipahal_aggr_get_max_pkt_limit(void);
void ipahal_get_aggr_force_close_valmask(int ep_idx,
	struct ipahal_reg_valmask *valmask);
void ipahal_get_fltrt_hash_flush_valmask(
	struct ipahal_reg_fltrt_hash_flush *flush,
	struct ipahal_reg_valmask *valmask);

void ipahal_get_fltrt_cache_flush_valmask(
	struct ipahal_reg_fltrt_cache_flush *flush,
	struct ipahal_reg_valmask *valmask);

#endif /* _IPAHAL_REG_H_ */
