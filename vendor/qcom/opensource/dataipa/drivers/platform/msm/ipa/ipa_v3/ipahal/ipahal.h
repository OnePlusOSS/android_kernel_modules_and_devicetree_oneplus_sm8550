/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _IPAHAL_H_
#define _IPAHAL_H_

#include "ipa_defs.h"
#include "ipa_common_i.h"

/*
 * Immediate command names
 *
 * NOTE:: Any change to this enum, need to change to ipahal_imm_cmd_name_to_str
 *	array as well.
 */
enum ipahal_imm_cmd_name {
	IPA_IMM_CMD_IP_V4_FILTER_INIT,
	IPA_IMM_CMD_IP_V6_FILTER_INIT,
	IPA_IMM_CMD_IP_V4_NAT_INIT,
	IPA_IMM_CMD_IP_V4_ROUTING_INIT,
	IPA_IMM_CMD_IP_V6_ROUTING_INIT,
	IPA_IMM_CMD_HDR_INIT_LOCAL,
	IPA_IMM_CMD_HDR_INIT_SYSTEM,
	IPA_IMM_CMD_REGISTER_WRITE,
	IPA_IMM_CMD_REGISTER_READ,
	IPA_IMM_CMD_NAT_DMA,
	IPA_IMM_CMD_IP_PACKET_INIT,
	IPA_IMM_CMD_DMA_SHARED_MEM,
	IPA_IMM_CMD_IP_PACKET_TAG_STATUS,
	IPA_IMM_CMD_DMA_TASK_32B_ADDR,
	IPA_IMM_CMD_TABLE_DMA,
	IPA_IMM_CMD_IP_V6_CT_INIT,
	IPA_IMM_CMD_IP_PACKET_INIT_EX,
	IPA_IMM_CMD_MAX,
};

/* Immediate commands abstracted structures */

/*
 * struct ipahal_imm_cmd_ip_v4_filter_init - IP_V4_FILTER_INIT cmd payload
 * Inits IPv4 filter block.
 * @hash_rules_addr: Addr in sys mem where ipv4 hashable flt tbl starts
 * @nhash_rules_addr: Addr in sys mem where ipv4 non-hashable flt tbl starts
 * @hash_rules_size: Size in bytes of the hashable tbl to cpy to local mem
 * @hash_local_addr: Addr in shared mem where ipv4 hashable flt tbl should
 *  be copied to
 * @nhash_rules_size: Size in bytes of the non-hashable tbl to cpy to local mem
 * @nhash_local_addr: Addr in shared mem where ipv4 non-hashable flt tbl should
 *  be copied to
 */
struct ipahal_imm_cmd_ip_v4_filter_init {
	u64 hash_rules_addr;
	u64 nhash_rules_addr;
	u32 hash_rules_size;
	u32 hash_local_addr;
	u32 nhash_rules_size;
	u32 nhash_local_addr;
};

/*
 * struct ipahal_imm_cmd_ip_v6_filter_init - IP_V6_FILTER_INIT cmd payload
 * Inits IPv6 filter block.
 * @hash_rules_addr: Addr in sys mem where ipv6 hashable flt tbl starts
 * @nhash_rules_addr: Addr in sys mem where ipv6 non-hashable flt tbl starts
 * @hash_rules_size: Size in bytes of the hashable tbl to cpy to local mem
 * @hash_local_addr: Addr in shared mem where ipv6 hashable flt tbl should
 *  be copied to
 * @nhash_rules_size: Size in bytes of the non-hashable tbl to cpy to local mem
 * @nhash_local_addr: Addr in shared mem where ipv6 non-hashable flt tbl should
 *  be copied to
 */
struct ipahal_imm_cmd_ip_v6_filter_init {
	u64 hash_rules_addr;
	u64 nhash_rules_addr;
	u32 hash_rules_size;
	u32 hash_local_addr;
	u32 nhash_rules_size;
	u32 nhash_local_addr;
};

/*
 * struct ipahal_imm_cmd_nat_ipv6ct_init_common - NAT/IPv6CT table init command
 *                                                common part
 * @base_table_addr: Address in sys/shared mem where base table start
 * @expansion_table_addr: Address in sys/shared mem where expansion table
 *  starts. Entries that result in hash collision are located in this table.
 * @base_table_addr_shared: base_table_addr in shared mem (if not, then sys)
 * @expansion_table_addr_shared: expansion_rules_addr in
 *  shared mem (if not, then sys)
 * @size_base_table: Num of entries in the base table
 * @size_expansion_table: Num of entries in the expansion table
 * @table_index: For future support of multiple tables
 */
struct ipahal_imm_cmd_nat_ipv6ct_init_common {
	u64 base_table_addr;
	u64 expansion_table_addr;
	bool base_table_addr_shared;
	bool expansion_table_addr_shared;
	u16 size_base_table;
	u16 size_expansion_table;
	u8 table_index;
};

/*
 * struct ipahal_imm_cmd_ip_v4_nat_init - IP_V4_NAT_INIT cmd payload
 * Inits IPv4 NAT block. Initiate NAT table with it dimensions, location
 *  cache address and other related parameters.
 * @table_init: table initialization parameters
 * @index_table_addr: Addr in sys/shared mem where index table, which points
 *  to NAT table starts
 * @index_table_expansion_addr: Addr in sys/shared mem where expansion index
 *  table starts
 * @index_table_addr_shared: index_table_addr in shared mem (if not, then sys)
 * @index_table_expansion_addr_shared: index_table_expansion_addr in
 *  shared mem (if not, then sys)
 * @public_addr_info: Public IP addresses info suitable to the IPA H/W version
 *                    IPA H/W >= 4.0 - PDN config table offset in SMEM
 *                    IPA H/W < 4.0  - The public IP address
 */
struct ipahal_imm_cmd_ip_v4_nat_init {
	struct ipahal_imm_cmd_nat_ipv6ct_init_common table_init;
	u64 index_table_addr;
	u64 index_table_expansion_addr;
	bool index_table_addr_shared;
	bool index_table_expansion_addr_shared;
	u32 public_addr_info;
};

/*
 * struct ipahal_imm_cmd_ip_v6_ct_init - IP_V6_CONN_TRACK_INIT cmd payload
 * Inits IPv6CT block. Initiate IPv6CT table with it dimensions, location
 *  cache address and other related parameters.
 * @table_init: table initialization parameters
 */
struct ipahal_imm_cmd_ip_v6_ct_init {
	struct ipahal_imm_cmd_nat_ipv6ct_init_common table_init;
};

/*
 * struct ipahal_imm_cmd_ip_v4_routing_init - IP_V4_ROUTING_INIT cmd payload
 * Inits IPv4 routing table/structure - with the rules and other related params
 * @hash_rules_addr: Addr in sys mem where ipv4 hashable rt tbl starts
 * @nhash_rules_addr: Addr in sys mem where ipv4 non-hashable rt tbl starts
 * @hash_rules_size: Size in bytes of the hashable tbl to cpy to local mem
 * @hash_local_addr: Addr in shared mem where ipv4 hashable rt tbl should
 *  be copied to
 * @nhash_rules_size: Size in bytes of the non-hashable tbl to cpy to local mem
 * @nhash_local_addr: Addr in shared mem where ipv4 non-hashable rt tbl should
 *  be copied to
 */
struct ipahal_imm_cmd_ip_v4_routing_init {
	u64 hash_rules_addr;
	u64 nhash_rules_addr;
	u32 hash_rules_size;
	u32 hash_local_addr;
	u32 nhash_rules_size;
	u32 nhash_local_addr;
};

/*
 * struct ipahal_imm_cmd_ip_v6_routing_init - IP_V6_ROUTING_INIT cmd payload
 * Inits IPv6 routing table/structure - with the rules and other related params
 * @hash_rules_addr: Addr in sys mem where ipv6 hashable rt tbl starts
 * @nhash_rules_addr: Addr in sys mem where ipv6 non-hashable rt tbl starts
 * @hash_rules_size: Size in bytes of the hashable tbl to cpy to local mem
 * @hash_local_addr: Addr in shared mem where ipv6 hashable rt tbl should
 *  be copied to
 * @nhash_rules_size: Size in bytes of the non-hashable tbl to cpy to local mem
 * @nhash_local_addr: Addr in shared mem where ipv6 non-hashable rt tbl should
 *  be copied to
 */
struct ipahal_imm_cmd_ip_v6_routing_init {
	u64 hash_rules_addr;
	u64 nhash_rules_addr;
	u32 hash_rules_size;
	u32 hash_local_addr;
	u32 nhash_rules_size;
	u32 nhash_local_addr;
};

/*
 * struct ipahal_imm_cmd_hdr_init_local - HDR_INIT_LOCAL cmd payload
 * Inits hdr table within local mem with the hdrs and their length.
 * @hdr_table_addr: Word address in sys mem where the table starts (SRC)
 * @size_hdr_table: Size of the above (in bytes)
 * @hdr_addr: header address in IPA sram (used as DST for memory copy)
 * @rsvd: reserved
 */
struct ipahal_imm_cmd_hdr_init_local {
	u64 hdr_table_addr;
	u32 size_hdr_table;
	u32 hdr_addr;
};

/*
 * struct ipahal_imm_cmd_hdr_init_system - HDR_INIT_SYSTEM cmd payload
 * Inits hdr table within sys mem with the hdrs and their length.
 * @hdr_table_addr: Word address in system memory where the hdrs tbl starts.
 */
struct ipahal_imm_cmd_hdr_init_system {
	u64 hdr_table_addr;
};

/*
 * struct ipahal_imm_cmd_table_dma - TABLE_DMA cmd payload
 * Perform DMA operation on NAT and IPV6 connection tracking related mem
 * addresses. Copy data into different locations within IPv6CT and NAT
 * associated tbls. (For add/remove NAT rules)
 * @offset: offset in bytes from base addr to write 'data' to
 * @data: data to be written
 * @table_index: NAT tbl index. Defines the tbl on which to perform DMA op.
 * @base_addr: Base addr to which the DMA operation should be performed.
 */
struct ipahal_imm_cmd_table_dma {
	u32 offset;
	u16 data;
	u8 table_index;
	u8 base_addr;
};

/*
 * struct ipahal_imm_cmd_ip_packet_init - IP_PACKET_INIT cmd payload
 * Configuration for specific IP pkt. Shall be called prior to an IP pkt
 *  data. Pkt will not go through IP pkt processing.
 * @destination_pipe_index: Destination pipe index  (in case routing
 *  is enabled, this field will overwrite the rt  rule)
 */
struct ipahal_imm_cmd_ip_packet_init {
	u32 destination_pipe_index;
};

/*
 * struct ipahal_imm_cmd_ip_packet_init_ex - IP_PACKET_INIT_EX cmd payload
 * @frag_disable: true - disabled. overrides IPA_ENDP_CONFIG_n:FRAG_OFFLOAD_EN
 * @filter_disable: true - disabled, false - enabled
 * @nat_disable: true - disabled, false - enabled
 * @route_disable: true - disabled, false - enabled
 * @hdr_removal_insertion_disable: true - disabled, false - enabled
 * @cs_disable: true - disabled, false - enabled
 * @quota_tethering_stats_disable: true - disabled, false - enabled
 * fields @flt_rt_tbl_idx - @flt_retain_hdr are a logical software translation
 * of ipa5_0_flt_rule_hw_hdr/ipa5_5_flt_rule_hw_hdr
 * fields @rt_pipe_dest_idx - @rt_system are a logical software translation
 * ipa5_0_rt_rule_hw_hdr/ipa5_5_flt_rule_hw_hdr
 * @dpl_disable: true - disabled, false - enabled, valid from IPAv5_5.
 * @flt_ext_hdr: true - flt ext_hdr enabled, false - disabled. Note all fields of
 * ext header are valid in immediate command irrespective of this flag.
 * fields @flt_ttl - @flt_qos_class are a logical software translation
 * of ipa5_5_flt_rule_hw_hdr_ext.
 * @rt_ext_hdr: true - rt ext_hdr enabled, false - disabled. Note all fields of
 * ext header are valid in immediate command irrespective of this flag.
 * fields @rt_ttl - @rt_skip_ingress are a logical software translation
 * ipa5_5_rt_rule_hw_hdr_ext
 */
struct ipahal_imm_cmd_ip_packet_init_ex {
	bool frag_disable;
	bool filter_disable;
	bool nat_disable;
	bool route_disable;
	bool hdr_removal_insertion_disable;
	bool cs_disable;
	bool quota_tethering_stats_disable;
	u8 flt_rt_tbl_idx;
	u8 flt_stats_cnt_idx;
	u8 flt_priority;
	bool flt_close_aggr_irq_mod;
	u8 flt_action;
	u8 flt_pdn_idx;
	bool flt_set_metadata;
	bool flt_retain_hdr;
	u8 rt_pipe_dest_idx;
	u8 rt_stats_cnt_idx;
	u8 rt_priority;
	bool rt_close_aggr_irq_mod;
	u16 rt_hdr_offset;
	bool rt_proc_ctx;
	bool rt_retain_hdr;
	bool rt_system;
	bool dpl_disable;
	bool flt_ext_hdr;
	bool flt_ttl;
	u8 flt_qos_class;
	bool rt_ext_hdr;
	bool rt_ttl;
	u8 rt_qos_class;
	bool rt_skip_ingress;
};

/*
 * enum ipa_pipeline_clear_option - Values for pipeline clear waiting options
 * @IPAHAL_HPS_CLEAR: Wait for HPS clear. All queues except high priority queue
 *  shall not be serviced until HPS is clear of packets or immediate commands.
 *  The high priority Rx queue / Q6ZIP group shall still be serviced normally.
 *
 * @IPAHAL_SRC_GRP_CLEAR: Wait for originating source group to be clear
 *  (for no packet contexts allocated to the originating source group).
 *  The source group / Rx queue shall not be serviced until all previously
 *  allocated packet contexts are released. All other source groups/queues shall
 *  be serviced normally.
 *
 * @IPAHAL_FULL_PIPELINE_CLEAR: Wait for full pipeline to be clear.
 *  All groups / Rx queues shall not be serviced until IPA pipeline is fully
 *  clear. This should be used for debug only.
 */
enum ipahal_pipeline_clear_option {
	IPAHAL_HPS_CLEAR,
	IPAHAL_SRC_GRP_CLEAR,
	IPAHAL_FULL_PIPELINE_CLEAR
};

/*
 * struct ipahal_imm_cmd_register_write - REGISTER_WRITE cmd payload
 * Write value to register. Allows reg changes to be synced with data packet
 *  and other immediate commands. Can be used to access the sram
 * @offset: offset from IPA base address - Lower 16bit of the IPA reg addr
 * @value: value to write to register
 * @value_mask: mask specifying which value bits to write to the register
 * @skip_pipeline_clear: if to skip pipeline clear waiting (don't wait)
 * @pipeline_clear_option: options for pipeline clear waiting
 */
struct ipahal_imm_cmd_register_write {
	u32 offset;
	u32 value;
	u32 value_mask;
	bool skip_pipeline_clear;
	enum ipahal_pipeline_clear_option pipeline_clear_options;
};

/*
 * struct ipahal_imm_cmd_register_read - REGISTER_READ cmd payload
 * Read value from register. Allows reg changes to be synced with data packet
 *  and other immediate commands. Can be used to access the sram
 * @offset: offset from IPA base address - Lower 16bit of the IPA reg addr
 * @sys_addr: Address in system memory for storing register value
 * @skip_pipeline_clear: if to skip pipeline clear waiting (don't wait)
 * @pipeline_clear_option: options for pipeline clear waiting
 */
struct ipahal_imm_cmd_register_read {
	u32 offset;
	u32 sys_addr;
	bool skip_pipeline_clear;
	enum ipahal_pipeline_clear_option pipeline_clear_options;
};

/*
 * struct ipahal_imm_cmd_dma_shared_mem - DMA_SHARED_MEM cmd payload
 * Perform mem copy into or out of the SW area of IPA local mem
 * @system_addr: Address in system memory
 * @size: Size in bytes of data to copy. Expected size is up to 2K bytes
 * @local_addr: Address in IPA local memory
 * @clear_after_read: Clear local memory at the end of a read operation allows
 *  atomic read and clear if HPS is clear. Ignore for writes.
 * @is_read: Read operation from local memory? If not, then write.
 * @skip_pipeline_clear: if to skip pipeline clear waiting (don't wait)
 * @pipeline_clear_option: options for pipeline clear waiting
 */
struct ipahal_imm_cmd_dma_shared_mem {
	u64 system_addr;
	u32 size;
	u32 local_addr;
	bool clear_after_read;
	bool is_read;
	bool skip_pipeline_clear;
	enum ipahal_pipeline_clear_option pipeline_clear_options;
};

/*
 * struct ipahal_imm_cmd_ip_packet_tag_status - IP_PACKET_TAG_STATUS cmd payload
 * This cmd is used for to allow SW to track HW processing by setting a TAG
 *  value that is passed back to SW inside Packet Status information.
 *  TAG info will be provided as part of Packet Status info generated for
 *  the next pkt transferred over the pipe.
 *  This immediate command must be followed by a packet in the same transfer.
 * @tag: Tag that is provided back to SW
 */
struct ipahal_imm_cmd_ip_packet_tag_status {
	u64 tag;
};

/*
 * struct ipahal_imm_cmd_dma_task_32b_addr - IPA_DMA_TASK_32B_ADDR cmd payload
 * Used by clients using 32bit addresses. Used to perform DMA operation on
 *  multiple descriptors.
 *  The Opcode is dynamic, where it holds the number of buffer to process
 * @cmplt: Complete flag: If true, IPA interrupt SW when the entire
 *  DMA related data was completely xfered to its destination.
 * @eof: Enf Of Frame flag: If true, IPA assert the EOT to the
 *  dest client. This is used used for aggr sequence
 * @flsh: Flush flag: If true pkt will go through the IPA blocks but
 *  will not be xfered to dest client but rather will be discarded
 * @lock: Lock pipe flag: If true, IPA will stop processing descriptors
 *  from other EPs in the same src grp (RX queue)
 * @unlock: Unlock pipe flag: If true, IPA will stop exclusively
 *  servicing current EP out of the src EPs of the grp (RX queue)
 * @size1: Size of buffer1 data
 * @addr1: Pointer to buffer1 data
 * @packet_size: Total packet size. If a pkt send using multiple DMA_TASKs,
 *  only the first one needs to have this field set. It will be ignored
 *  in subsequent DMA_TASKs until the packet ends (EOT). First DMA_TASK
 *  must contain this field (2 or more buffers) or EOT.
 */
struct ipahal_imm_cmd_dma_task_32b_addr {
	bool cmplt;
	bool eof;
	bool flsh;
	bool lock;
	bool unlock;
	u32 size1;
	u32 addr1;
	u32 packet_size;
};

/*
 * struct ipahal_imm_cmd_pyld - Immediate cmd payload information
 * @len: length of the buffer
 * @opcode: opcode of the immediate command
 * @data: buffer contains the immediate command payload. Buffer goes
 *  back to back with this structure
 */
struct ipahal_imm_cmd_pyld {
	u16 len;
	u16 opcode;
	u8 data[0];
};


/* Immediate command Function APIs */

/*
 * ipahal_imm_cmd_name_str() - returns string that represent the imm cmd
 * @cmd_name: [in] Immediate command name
 */
const char *ipahal_imm_cmd_name_str(enum ipahal_imm_cmd_name cmd_name);

/*
 * ipahal_construct_imm_cmd() - Construct immdiate command
 * This function builds imm cmd bulk that can be be sent to IPA
 * The command will be allocated dynamically.
 * After done using it, call ipahal_destroy_imm_cmd() to release it
 */
struct ipahal_imm_cmd_pyld *ipahal_construct_imm_cmd(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx);

/*
 * ipahal_modify_imm_cmd() - Modify immdiate command in an existing buffer
 * This function modifies an existing imm cmd buffer
 * @cmd_name: [in] Immediate command name
 * @cmd_data: [in] Constructed immediate command buffer data
 * @params: [in] Structure with specific IMM params
 * @params_mask: [in] Same structure, but the fields filled with 0,
 *  if they should not be changed, or any non-zero for fields to be updated
 */
int ipahal_modify_imm_cmd(
	enum ipahal_imm_cmd_name cmd,
	const void *cmd_data,
	const void *params,
	const void *params_mask);

/*
 * ipa_imm_cmd_modify_ip_packet_init_ex_dest_pipe() -
 *   Modify ip_packet_init_ex immdiate command pipe_dest_idx field
 * This function modifies an existing imm cmd buffer
 * @cmd_data: [in] Constructed immediate command buffer data
 * @pipe_dest_idx: [in] destination pipe index to set
 */
void ipa_imm_cmd_modify_ip_packet_init_ex_dest_pipe(
	const void *cmd_data,
	u64 pipe_dest_idx);

/*
 * ipahal_construct_nop_imm_cmd() - Construct immediate comamnd for NO-Op
 * Core driver may want functionality to inject NOP commands to IPA
 *  to ensure e.g., PIPLINE clear before someother operation.
 * The functionality given by this function can be reached by
 *  ipahal_construct_imm_cmd(). This function is helper to the core driver
 *  to reach this NOP functionlity easily.
 * @skip_pipline_clear: if to skip pipeline clear waiting (don't wait)
 * @pipline_clr_opt: options for pipeline clear waiting
 * @is_atomic_ctx: is called in atomic context or can sleep?
 */
struct ipahal_imm_cmd_pyld *ipahal_construct_nop_imm_cmd(
	bool skip_pipline_clear,
	enum ipahal_pipeline_clear_option pipline_clr_opt,
	bool is_atomic_ctx);

/*
 * ipahal_destroy_imm_cmd() - Destroy/Release bulk that was built
 *  by the construction functions
 */
static inline void ipahal_destroy_imm_cmd(struct ipahal_imm_cmd_pyld *pyld)
{
	kfree(pyld);
}


/* IPA Status packet Structures and Function APIs */

/*
 * enum ipahal_pkt_status_opcode - Packet Status Opcode
 * @IPAHAL_STATUS_OPCODE_PACKET_2ND_PASS: Packet Status generated as part of
 *  IPA second processing pass for a packet (i.e. IPA XLAT processing for
 *  the translated packet).
 */
enum ipahal_pkt_status_opcode {
	IPAHAL_PKT_STATUS_OPCODE_PACKET = 0,
	IPAHAL_PKT_STATUS_OPCODE_NEW_FRAG_RULE,
	IPAHAL_PKT_STATUS_OPCODE_DROPPED_PACKET,
	IPAHAL_PKT_STATUS_OPCODE_SUSPENDED_PACKET,
	IPAHAL_PKT_STATUS_OPCODE_LOG,
	IPAHAL_PKT_STATUS_OPCODE_DCMP,
	IPAHAL_PKT_STATUS_OPCODE_PACKET_2ND_PASS,
};

/*
 * enum ipahal_pkt_status_exception - Packet Status exception type
 * @IPAHAL_PKT_STATUS_EXCEPTION_PACKET_LENGTH: formerly IHL exception.
 *
 * Note: IPTYPE, PACKET_LENGTH and PACKET_THRESHOLD exceptions means that
 *  partial / no IP processing took place and corresponding Status Mask
 *  fields should be ignored. Flt and rt info is not valid.
 *
 * NOTE:: Any change to this enum, need to change to
 *	ipahal_pkt_status_exception_to_str array as well.
 */
enum ipahal_pkt_status_exception {
	IPAHAL_PKT_STATUS_EXCEPTION_NONE = 0,
	IPAHAL_PKT_STATUS_EXCEPTION_DEAGGR,
	IPAHAL_PKT_STATUS_EXCEPTION_IPTYPE,
	IPAHAL_PKT_STATUS_EXCEPTION_PACKET_LENGTH,
	IPAHAL_PKT_STATUS_EXCEPTION_PACKET_THRESHOLD,
	IPAHAL_PKT_STATUS_EXCEPTION_TTL,
	IPAHAL_PKT_STATUS_EXCEPTION_FRAG_RULE_MISS,
	IPAHAL_PKT_STATUS_EXCEPTION_SW_FILT,
	/*
	 * NAT and IPv6CT have the same value at HW.
	 * NAT for IPv4 and IPv6CT for IPv6 exceptions
	 */
	IPAHAL_PKT_STATUS_EXCEPTION_NAT,
	IPAHAL_PKT_STATUS_EXCEPTION_IPV6CT,
	IPAHAL_PKT_STATUS_EXCEPTION_UCP,
	IPAHAL_PKT_STATUS_EXCEPTION_INVALID_PIPE,
	IPAHAL_PKT_STATUS_EXCEPTION_RQOS,
	IPAHAL_PKT_STATUS_EXCEPTION_HDRI,
	IPAHAL_PKT_STATUS_EXCEPTION_CSUM,
	IPAHAL_PKT_STATUS_EXCEPTION_MAX,
};

/*
 * enum ipahal_pkt_status_mask - Packet Status bitmask shift values of
 *  the contained flags. This bitmask indicates flags on the properties of
 *  the packet as well as IPA processing it may had.
 * @FRAG_PROCESS: Frag block processing flag: Was pkt processed by frag block?
 *  Also means the frag info is valid unless exception or first frag
 * @FILT_PROCESS: Flt block processing flag: Was pkt processed by flt block?
 *  Also means that flt info is valid.
 * @NAT_PROCESS: NAT block processing flag: Was pkt processed by NAT block?
 *  Also means that NAT info is valid, unless exception.
 * @ROUTE_PROCESS: Rt block processing flag: Was pkt processed by rt block?
 *  Also means that rt info is valid, unless exception.
 * @TAG_VALID: Flag specifying if TAG and TAG info valid?
 * @FRAGMENT: Flag specifying if pkt is IP fragment.
 * @FIRST_FRAGMENT: Flag specifying if pkt is first fragment. In this case, frag
 *  info is invalid
 * @V4: Flag specifying pkt is IPv4 or IPv6
 * @CKSUM_PROCESS: CSUM block processing flag: Was pkt processed by csum block?
 *  If so, csum trailer exists
 * @AGGR_PROCESS: Aggr block processing flag: Was pkt processed by aggr block?
 * @DEST_EOT: Flag specifying if EOT was asserted for the pkt on dest endp
 * @DEAGGR_PROCESS: Deaggr block processing flag: Was pkt processed by deaggr
 *  block?
 * @DEAGG_FIRST: Flag specifying if this is the first pkt in deaggr frame
 * @SRC_EOT: Flag specifying if EOT asserted by src endp when sending the buffer
 * @PREV_EOT: Flag specifying if EOT was sent just before the pkt as part of
 *  aggr hard-byte-limit
 * @BYTE_LIMIT: Flag specifying if pkt is over a configured byte limit.
 */
enum ipahal_pkt_status_mask {
	IPAHAL_PKT_STATUS_MASK_FRAG_PROCESS_SHFT = 0,
	IPAHAL_PKT_STATUS_MASK_FILT_PROCESS_SHFT,
	IPAHAL_PKT_STATUS_MASK_NAT_PROCESS_SHFT,
	IPAHAL_PKT_STATUS_MASK_ROUTE_PROCESS_SHFT,
	IPAHAL_PKT_STATUS_MASK_TAG_VALID_SHFT,
	IPAHAL_PKT_STATUS_MASK_FRAGMENT_SHFT,
	IPAHAL_PKT_STATUS_MASK_FIRST_FRAGMENT_SHFT,
	IPAHAL_PKT_STATUS_MASK_V4_SHFT,
	IPAHAL_PKT_STATUS_MASK_CKSUM_PROCESS_SHFT,
	IPAHAL_PKT_STATUS_MASK_AGGR_PROCESS_SHFT,
	IPAHAL_PKT_STATUS_MASK_DEST_EOT_SHFT,
	IPAHAL_PKT_STATUS_MASK_OPENED_FRAME_SHFT =
		IPAHAL_PKT_STATUS_MASK_DEST_EOT_SHFT,
	IPAHAL_PKT_STATUS_MASK_DEAGGR_PROCESS_SHFT,
	IPAHAL_PKT_STATUS_MASK_DEAGG_FIRST_SHFT,
	IPAHAL_PKT_STATUS_MASK_SRC_EOT_SHFT,
	IPAHAL_PKT_STATUS_MASK_PREV_EOT_SHFT,
	IPAHAL_PKT_STATUS_MASK_RQOS_NAS_SHFT =
		IPAHAL_PKT_STATUS_MASK_PREV_EOT_SHFT,
	IPAHAL_PKT_STATUS_MASK_BYTE_LIMIT_SHFT,
	IPAHAL_PKT_STATUS_MASK_RQOS_AS_SHFT =
		IPAHAL_PKT_STATUS_MASK_BYTE_LIMIT_SHFT,
};

/*
 * Returns boolean value representing a property of the a packet.
 * @__flag_shft: The shift value of the flag of the status bitmask of
 * @__status: Pointer to abstracrted status structure
 *  the needed property. See enum ipahal_pkt_status_mask
 */
#define IPAHAL_PKT_STATUS_MASK_FLAG_VAL(__flag_shft, __status) \
	(((__status)->status_mask) & ((u32)0x1<<(__flag_shft)) ? true : false)

/*
 * enum ipahal_pkt_status_nat_type - Type of NAT
 */
enum ipahal_pkt_status_nat_type {
	IPAHAL_PKT_STATUS_NAT_NONE,
	IPAHAL_PKT_STATUS_NAT_SRC,
	IPAHAL_PKT_STATUS_NAT_DST,
};

/*
 * struct ipahal_pkt_status - IPA status packet abstracted payload.
 *  This structure describes the status packet fields for the
 *   following statuses: IPA_STATUS_PACKET, IPA_STATUS_DROPPED_PACKET,
 *   IPA_STATUS_SUSPENDED_PACKET.
 *  Other statuses types has different status packet structure.
 * @tag_info: S/W defined value provided via immediate command
 * @status_opcode: The Type of the status (Opcode).
 * @exception: The first exception that took place.
 *  In case of exception, src endp and pkt len are always valid.
 * @status_mask: Bit mask for flags on several properties on the packet
 *  and processing it may passed at IPA. See enum ipahal_pkt_status_mask
 * @pkt_len: Pkt pyld len including hdr and retained hdr if used. Does
 *  not include padding or checksum trailer len.
 * @metadata: meta data value used by packet
 * @flt_local: Filter table location flag: Does matching flt rule belongs to
 *  flt tbl that resides in lcl memory? (if not, then system mem)
 * @flt_hash: Filter hash hit flag: Does matching flt rule was in hash tbl?
 * @flt_global: Global filter rule flag: Does matching flt rule belongs to
 *  the global flt tbl? (if not, then the per endp tables)
 * @flt_ret_hdr: Retain header in filter rule flag: Does matching flt rule
 *  specifies to retain header?
 *  Starting IPA4.5, this will be true only if packet has L2 header.
 * @flt_miss: Filtering miss flag: Was their a filtering rule miss?
 *   In case of miss, all flt info to be ignored
 * @rt_local: Route table location flag: Does matching rt rule belongs to
 *  rt tbl that resides in lcl memory? (if not, then system mem)
 * @rt_hash: Route hash hit flag: Does matching rt rule was in hash tbl?
 * @ucp: UC Processing flag
 * @rt_miss: Routing miss flag: Was their a routing rule miss?
 * @nat_hit: NAT hit flag: Was their NAT hit?
 * @nat_type: Defines the type of the NAT operation:
 * @time_of_day_ctr: running counter from IPA clock
 * @hdr_local: Header table location flag: In header insertion, was the header
 *  taken from the table resides in local memory? (If no, then system mem)
 * @frag_hit: Frag hit flag: Was their frag rule hit in H/W frag table?
 * @flt_rule_id: The ID of the matching filter rule (if no miss).
 *  This info can be combined with endp_src_idx to locate the exact rule.
 * @rt_rule_id: The ID of the matching rt rule. (if no miss). This info
 *  can be combined with rt_tbl_idx to locate the exact rule.
 * @nat_entry_idx: Index of the NAT entry used of NAT processing
 * @hdr_offset: Offset of used header in the header table
 * @endp_src_idx: Source end point index.
 * @endp_dest_idx: Destination end point index.
 *  Not valid in case of exception
 * @rt_tbl_idx: Index of rt tbl that contains the rule on which was a match
 * @seq_num: Per source endp unique packet sequence number
 * @frag_rule: Frag rule index in H/W frag table in case of frag hit
 * @frag_rule_idx: Frag rule index value.
 * @tbl_idx: Table index valid or not.
 * @src_ip_addr: Source packet IP address.
 * @dest_ip_addr: Destination packet IP address.
 * @protocol: Protocal number.
 * @ip_id: IP packet IP ID number.
 * @tlated_ip_addr: IP address.
 * @ip_cksum_diff: IP packet checksum difference.
 * @hdr_ret: l2 header retained flag, indicates whether l2 header is retained
 * or not.
 * @ll: low latency indication.
 * @tsp: Traffic shaping policing flag, indicates traffic class info
 * overwrites tag info.
 * @ttl_dec: ttl update flag, indicates whether ttl is updated.
 * @nat_exc_suppress: nat exception supress flag, indicates whether
 * nat exception is suppressed.
 * @ingress_tc: Ingress traffic class index.
 * @egress_tc: Egress traffic class index.
 * @pd: router disabled ingress policer.
 */
struct ipahal_pkt_status {
	u64 tag_info;
	enum ipahal_pkt_status_opcode status_opcode;
	enum ipahal_pkt_status_exception exception;
	u32 status_mask;
	u32 pkt_len;
	u32 metadata;
	bool flt_local;
	bool flt_hash;
	bool flt_global;
	bool flt_ret_hdr;
	bool flt_miss;
	bool rt_local;
	bool rt_hash;
	bool ucp;
	bool rt_miss;
	bool nat_hit;
	enum ipahal_pkt_status_nat_type nat_type;
	u32 time_of_day_ctr;
	bool hdr_local;
	bool frag_hit;
	u16 flt_rule_id;
	u16 rt_rule_id;
	u16 nat_entry_idx;
	u16 hdr_offset;
	u8 endp_src_idx;
	u8 endp_dest_idx;
	u8 rt_tbl_idx;
	u8 seq_num;
	u8 frag_rule;
	u8 frag_rule_idx;
	bool tbl_idx;
	u32 src_ip_addr;
	u32 dest_ip_addr;
	u8 protocol;
	u16 ip_id;
	u32 tlated_ip_addr;
	u16 ip_cksum_diff;
	bool hdr_ret;
	bool ll;
	bool tsp;
	bool ttl_dec;
	bool nat_exc_suppress;
	u8 ingress_tc;
	u8 egress_tc;
	bool pd;
};

/*
 * struct ipahal_pkt_status_thin - this struct is used to parse only
 *  a few fields from the status packet, needed for LAN optimization.
 * @exception: The first exception that took place.
 * @metadata: meta data value used by packet
 * @endp_src_idx: Source end point index.
 * @ucp: UC Processing flag
 */
struct ipahal_pkt_status_thin {
	enum ipahal_pkt_status_exception exception;
	u32 metadata;
	u8 endp_src_idx;
	bool ucp;
};

/*
 * ipahal_pkt_status_get_size() - Get H/W size of packet status
 */
u32 ipahal_pkt_status_get_size(void);

/*
 * ipahal_pkt_status_parse() - Parse Packet Status payload to abstracted form
 * @unparsed_status: Pointer to H/W format of the packet status as read from H/W
 * @status: Pointer to pre-allocated buffer where the parsed info will be stored
 */
void ipahal_pkt_status_parse(const void *unparsed_status,
	struct ipahal_pkt_status *status);

/*
 * ipahal_pkt_status_parse_thin() - Parse some of the packet status fields
 * for specific usage in the LAN rx data path where parsing needs to be done
 * but only for specific fields.
 * @unparsed_status: Pointer to H/W format of the packet status as read from HW
 * @status: Pointer to pre-allocated buffer where the parsed info will be
 * stored
 */
void ipahal_pkt_status_parse_thin(const void *unparsed_status,
	struct ipahal_pkt_status_thin *status);

/*
 * ipahal_pkt_status_exception_str() - returns string represents exception type
 * @exception: [in] The exception type
 */
const char *ipahal_pkt_status_exception_str(
	enum ipahal_pkt_status_exception exception);

/*
 * ipahal_cp_hdr_to_hw_buff() - copy header to hardware buffer according to
 * base address and offset given.
 * @base: dma base address
 * @offset: offset from base address where the data will be copied
 * @hdr: the header to be copied
 * @hdr_len: the length of the header
 */
void ipahal_cp_hdr_to_hw_buff(void *base, u32 offset, u8 *hdr, u32 hdr_len);

/*
 * ipahal_cp_proc_ctx_to_hw_buff() - copy processing context to
 * base address and offset given.
 * @type: type of header processing context
 * @base: dma base address
 * @offset: offset from base address where the data will be copied
 * @hdr_len: the length of the header
 * @hdr_base_addr: base address in table
 * @offset_entry: offset from hdr_base_addr in table
 * @l2tp_params: l2tp parameters
 * @eogre_params: eogre parameters
 * @generic_params: generic proc_ctx params
 * @is_64: Indicates whether header base address/dma base address is 64 bit.
 */
int ipahal_cp_proc_ctx_to_hw_buff(enum ipa_hdr_proc_type type,
		void *base, u32 offset, u32 hdr_len,
		u64 hdr_base_addr,
		struct ipa_hdr_offset_entry *offset_entry,
		struct ipa_l2tp_hdr_proc_ctx_params *l2tp_params,
		struct ipa_eogre_hdr_proc_ctx_params *eogre_params,
		struct ipa_eth_II_to_eth_II_ex_procparams *generic_params,
		bool is_64);

/*
 * ipahal_get_proc_ctx_needed_len() - calculates the needed length for addition
 * of header processing context according to the type of processing context
 * @type: header processing context type (no processing context,
 *	IPA_HDR_PROC_ETHII_TO_ETHII etc.)
 */
int ipahal_get_proc_ctx_needed_len(enum ipa_hdr_proc_type type);

int ipahal_init(enum ipa_hw_type ipa_hw_type, void __iomem *base,
    u32 ipa_cfg_offset, struct device *ipa_pdev);
void ipahal_destroy(void);
void ipahal_free_dma_mem(struct ipa_mem_buffer *mem);

/*
* ipahal_test_ep_bit() - return true if a ep bit is set
*/
bool ipahal_test_ep_bit(u32 reg_val, u32 ep_num);

/*
* ipahal_get_ep_bit() - get ep bit set in the right offset
*/
u32 ipahal_get_ep_bit(u32 ep_num);

/*
* ipahal_get_ep_reg_idx() - get ep reg index according to ep num
*/
u32 ipahal_get_ep_reg_idx(u32 ep_num);

/*
 * ***************************************************************
 *
 * To follow, a generalized qmap header manipulation API.
 *
 * ***************************************************************
 */
/**
 * qmap_hdr_v4_5 -
 *
 * @cd -
 * @qmap_next_hdr -
 * @pad -
 * @mux_id -
 * @packet_len_with_pad -
 * @hdr_type -
 * @coal_next_hdr -
 * @zero_checksum -
 *
 * The following bit layout is when the data are in host order.
 *
 * FIXME FINDME Need to be reordered properly to reflect network
 *              ordering as seen by little endian host (qmap_hdr_v5_5
 *              below proplerly done).
 */
struct qmap_hdr_v4_5 {
	/*
	 * 32 bits of qmap header to follow
	 */
	u64 cd: 1;
	u64 qmap_next_hdr: 1;
	u64 pad: 6;
	u64 mux_id: 8;
	u64 packet_len_with_pad: 16;
	/*
	 * 32 bits of coalescing frame header to follow
	 */
	u64 hdr_type: 7;
	u64 coal_next_hdr: 1;
	u64 zero_checksum: 1;
	u64 rsrvd1: 7;
	u64 rsrvd2: 16;
} __packed;

/**
 * qmap_hdr_v5_0 -
 *
 * @cd -
 * @qmap_next_hdr -
 * @pad -
 * @mux_id -
 * @packet_len_with_pad -
 * @hdr_type -
 * @coal_next_hdr -
 * @ip_id_cfg -
 * @zero_checksum -
 * @additional_hdr_size -
 * @segment_size -
 *
 * The following bit layout is when the data are in host order.
 *
 * FIXME FINDME Need to be reordered properly to reflect network
 *              ordering as seen by little endian host (qmap_hdr_v5_5
 *              below proplerly done).
 */
struct qmap_hdr_v5_0 {
	/*
	 * 32 bits of qmap header to follow
	 */
	u64 cd: 1;
	u64 qmap_next_hdr: 1;
	u64 pad: 6;
	u64 mux_id: 8;
	u64 packet_len_with_pad: 16;
	/*
	 * 32 bits of coalescing frame header to follow
	 */
	u64 hdr_type: 7;
	u64 coal_next_hdr: 1;
	u64 ip_id_cfg: 1;
	u64 zero_checksum: 1;
	u64 rsrvd: 1;
	u64 additional_hdr_size: 5;
	u64 segment_size: 16;
} __packed;

/**
 * qmap_hdr_v5_5 -
 *
 * @cd -
 * @qmap_next_hdr -
 * @pad -
 * @mux_id -
 * @packet_len_with_pad -
 * @hdr_type -
 * @coal_next_hdr -
 * @chksum_valid -
 * @num_nlos -
 * @inc_ip_id -
 * @rnd_ip_id -
 * @close_value -
 * @close_type -
 * @vcid -
 *
 * NOTE:
 *
 *   The layout below is different when compared against
 *   documentation, which shows the fields as they are in network byte
 *   order - and network byte order is how we receive the data from
 *   the IPA.  To avoid using cycles converting from network to host
 *   order, we've defined the stucture below such that we can access
 *   the correct fields while the data are still in network order.
 */
struct qmap_hdr_v5_5 {
	/*
	 * 32 bits of qmap header to follow
	 */
	u8 pad: 6;
	u8 qmap_next_hdr: 1;
	u8 cd: 1;
	u8 mux_id;
	u16 packet_len_with_pad;
	/*
	 * 32 bits of coalescing frame header to follow
	 */
	u8 coal_next_hdr: 1;
	u8 hdr_type: 7;
	u8 rsrvd1: 2;
	u8 rnd_ip_id: 1;
	u8 inc_ip_id: 1;
	u8 num_nlos: 3;
	u8 chksum_valid: 1;

	u8 close_type: 4;
	u8 close_value: 4;
	u8 rsrvd2: 4;
	u8 vcid: 4;
} __packed;

/**
 * qmap_hdr_u -
 *
 * The following is a union of all of the qmap versions above.
 *
 * NOTE WELL: REMEMBER to keep it in sync with the bit strucure
 *            definitions above.
 */
union qmap_hdr_u {
	struct qmap_hdr_v4_5 qmap4_5;
	struct qmap_hdr_v5_0 qmap5_0;
	struct qmap_hdr_v5_5 qmap5_5;
	u32                  words[2]; /* these used to flip from ntoh and hton */
} __packed;

/**
 * qmap_hdr_data -
 *
 * The following is an aggregation of the qmap header bit structures
 * above.
 *
 * NOTE WELL: REMEMBER to keep it in sync with the bit structure
 *            definitions above.
 */
struct qmap_hdr_data {
	/*
	 * Data from qmap header to follow
	 */
	u8 cd;
	u8 qmap_next_hdr;
	u8 pad;
	u8 mux_id;
	u16 packet_len_with_pad;
	/*
	 * Data from coalescing frame header to follow
	 */
	u8 hdr_type;
	u8 coal_next_hdr;
	u8 ip_id_cfg;
	u8 zero_checksum;
	u8 additional_hdr_size;
	u16 segment_size;
	u8 chksum_valid;
	u8 num_nlos;
	u8 inc_ip_id;
	u8 rnd_ip_id;
	u8 close_value;
	u8 close_type;
	u8 vcid;
};

/**
 * FUNCTION: ipahal_qmap_parse()
 *
 * The following function to be called when version specific qmap parsing is
 * required.
 *
 * ARGUMENTS:
 *
 *   unparsed_qmap
 *
 *     The QMAP header off of a freshly recieved data packet.  As per
 *     the architecture documentation, the data contained herein will
 *     be in network order.
 *
 *   qmap_data_rslt
 *
 *     A location to store the parsed data from unparsed_qmap above.
 */
int ipahal_qmap_parse(
	const void*           unparsed_qmap,
	struct qmap_hdr_data* qmap_data_rslt);


/**
 * FUNCTION: ipahal_qmap_ntoh()
 *
 * The following function will take a QMAP header, which you know is
 * in network order, and convert it to host order.
 *
 * NOTE WELL: Once in host order, the data will align with the bit
 *            descriptions in the headers above.
 *
 * ARGUMENTS:
 *
 *   src_data_from_packet
 *
 *     The QMAP header off of a freshly recieved data packet.  As per
 *     the architecture documentation, the data contained herein will
 *     be in network order.
 *
 *  dst_result
 *
 *    A location to where the original data will be copied, then
 *    converted to host order.
 */
static inline void ipahal_qmap_ntoh(
	const void*       src_data_from_packet,
	union qmap_hdr_u* dst_result)
{
	/*
	 * Nothing to do, since we define the bit fields in the
	 * structure, such that we can access them correctly while
	 * keeping the data in network order...
	 */
	if (src_data_from_packet && dst_result) {
		memcpy(
			dst_result,
			src_data_from_packet,
			sizeof(union qmap_hdr_u));
	}
}

/**
 * FUNCTION: ipahal_qmap_hton()
 *
 * The following function will take QMAP data, that you've assembled
 * in host otder (ie. via using the bit structures definitions above),
 * and convert it to network order.
 *
 * This function is to be used for QMAP data destined for network
 * transmission.
 *
 * ARGUMENTS:
 *
 *   src_data_from_host
 *
 *     QMAP data in host order.
 *
 *  dst_result
 *
 *    A location to where the host ordered data above will be copied,
 *    then converted to network order.
 */
static inline void ipahal_qmap_hton(
	union qmap_hdr_u* src_data_from_host,
	void*             dst_result)
{
	if (src_data_from_host && dst_result) {
		memcpy(
			dst_result,
			src_data_from_host,
			sizeof(union qmap_hdr_u));
		/*
		 * Reusing variable below to do the host to network swap...
		 */
		src_data_from_host = (union qmap_hdr_u*) dst_result;
		src_data_from_host->words[0] = htonl(src_data_from_host->words[0]);
		src_data_from_host->words[1] = htonl(src_data_from_host->words[1]);
	}
}

#endif /* _IPAHAL_H_ */
