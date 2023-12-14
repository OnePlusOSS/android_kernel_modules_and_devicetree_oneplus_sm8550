/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 */

#ifndef _IPAHAL_FLTRT_I_H_
#define _IPAHAL_FLTRT_I_H_

/*
 * enum ipa_fltrt_equations - RULE equations
 *  These are names values to the equations that can be used
 *  The HAL layer holds mapping between these names and H/W
 *  presentation.
 */
enum ipa_fltrt_equations {
	IPA_TOS_EQ,
	IPA_PROTOCOL_EQ,
	IPA_TC_EQ,
	IPA_OFFSET_MEQ128_0,
	IPA_OFFSET_MEQ128_1,
	IPA_OFFSET_MEQ32_0,
	IPA_OFFSET_MEQ32_1,
	IPA_IHL_OFFSET_MEQ32_0,
	IPA_IHL_OFFSET_MEQ32_1,
	IPA_METADATA_COMPARE,
	IPA_IHL_OFFSET_RANGE16_0,
	IPA_IHL_OFFSET_RANGE16_1,
	IPA_IHL_OFFSET_EQ_32,
	IPA_IHL_OFFSET_EQ_16,
	IPA_FL_EQ,
	IPA_IS_FRAG,
	IPA_IS_PURE_ACK,
	IPA_EQ_MAX,
};

/* Width and Alignment values for H/W structures.
 * Specific for IPA version.
 */
#define IPA3_0_HW_TBL_SYSADDR_ALIGNMENT (127)
#define IPA3_0_HW_TBL_LCLADDR_ALIGNMENT (7)
#define IPA3_0_HW_TBL_BLK_SIZE_ALIGNMENT (127)
#define IPA3_0_HW_TBL_WIDTH (8)
#define IPA3_0_HW_TBL_HDR_WIDTH (8)
#define IPA3_0_HW_TBL_ADDR_MASK (127)
#define IPA3_0_HW_RULE_BUF_SIZE (256)
#define IPA3_0_HW_RULE_START_ALIGNMENT (7)
#define IPA3_0_HW_RULE_PREFETCH_BUF_SIZE (128)


/*
 * Rules Priority.
 * Needed due to rules classification to hashable and non-hashable.
 * Higher priority is lower in number. i.e. 0 is highest priority
 */
#define IPA3_0_RULE_MAX_PRIORITY (0)
#define IPA3_0_RULE_MIN_PRIORITY (1023)

#define IPA5_0_RULE_MAX_PRIORITY (0)
#define IPA5_0_RULE_MIN_PRIORITY (255)

/*
 * RULE ID, bit length (e.g. 10 bits).
 */
#define IPA3_0_RULE_ID_BIT_LEN (10)
#define IPA3_0_LOW_RULE_ID (1)

/*
 * COUNTER ID, LOW COUNTER ID.
 */
#define IPA4_5_LOW_CNT_ID (1)

/**
 * struct ipa3_0_rt_rule_hw_hdr - HW header of IPA routing rule
 * @word: routing rule header properties
 * @en_rule: enable rule - Equation bit fields
 * @pipe_dest_idx: destination pipe index
 * @system: Is referenced header is lcl or sys memory
 * @hdr_offset: header offset
 * @proc_ctx: whether hdr_offset points to header table or to
 *	header processing context table
 * @priority: Rule priority. Added to distinguish rules order
 *  at the integrated table consisting from hashable and
 *  non-hashable parts
 * @rsvd1: reserved bits
 * @retain_hdr: added to add back to the packet the header removed
 *  as part of header removal. This will be done as part of
 *  header insertion block.
 * @rule_id: rule ID that will be returned in the packet status
 * @rsvd2: reserved bits
 */
struct ipa3_0_rt_rule_hw_hdr {
	union {
		u64 word;
		struct {
			u64 en_rule:16;
			u64 pipe_dest_idx:5;
			u64 system:1;
			u64 hdr_offset:9;
			u64 proc_ctx:1;
			u64 priority:10;
			u64 rsvd1:5;
			u64 retain_hdr:1;
			u64 rule_id:10;
			u64 rsvd2:6;
		} hdr;
	} u;
};

/**
 * struct ipa4_5_rt_rule_hw_hdr - HW header of IPA routing rule
 * @word: routing rule header properties
 * @en_rule: enable rule - Equation bit fields
 * @pipe_dest_idx: destination pipe index
 * @system: Is referenced header is lcl or sys memory
 * @hdr_offset: header offset
 * @proc_ctx: whether hdr_offset points to header table or to
 *	header processing context table
 * @priority: Rule priority. Added to distinguish rules order
 *  at the integrated table consisting from hashable and
 *  non-hashable parts
 * @stats_cnt_idx_msb: stats cnt index msb
 * @rsvd2: reserved bits
 * @retain_hdr: added to add back to the packet the header removed
 *  as part of header removal. This will be done as part of
 *  header insertion block.
 * @rule_id: rule ID that will be returned in the packet status
 * @stats_cnt_idx_lsb: stats cnt index lsb
 */
struct ipa4_5_rt_rule_hw_hdr {
	union {
		u64 word;
		struct {
			u64 en_rule:16;
			u64 pipe_dest_idx:5;
			u64 system:1;
			u64 hdr_offset:9;
			u64 proc_ctx:1;
			u64 priority:10;
			u64 stats_cnt_idx_msb : 2;
			u64 rsvd2 : 3;
			u64 retain_hdr:1;
			u64 rule_id:10;
			u64 stats_cnt_idx_lsb : 6;
		} hdr;
	} u;
};

/**
 * struct ipa5_0_rt_rule_hw_hdr - HW header of IPA routing rule
 * @word: routing rule header properties
 * @en_rule: enable rule - Equation bit fields
 * @pipe_dest_idx: destination pipe index
 * @stats_cnt_idx_lsb: stats cnt index
 * @priority: Rule priority. Added to distinguish rules order
 *  at the integrated table consisting from hashable and
 *  non-hashable parts
 * @rsvd: reserved bit
 * @close_aggr_irq_mod: close aggregation/coalescing and close GSI
 *  interrupt moderation
 * @rule_id: rule ID that will be returned in the packet status
 * @hdr_offset: header offset
 * @proc_ctx: whether hdr_offset points to header table or to
 *	header processing context table
 * @retain_hdr: added to add back to the packet the header removed
 *  as part of header removal. This will be done as part of
 *  header insertion block.
 * @system: Is referenced header is lcl or sys memory
 */
struct ipa5_0_rt_rule_hw_hdr {
	union {
		u64 word;
		struct {
			u64 en_rule : 16;
			u64 pipe_dest_idx : 8;
			u64 stats_cnt_idx : 8;
			u64 priority : 8;
			u64 rsvd : 1;
			u64 close_aggr_irq_mod : 1;
			u64 rule_id : 10;
			u64 hdr_offset : 9;
			u64 proc_ctx : 1;
			u64 retain_hdr : 1;
			u64 system : 1;
		} hdr;
	} u;
};

/**
 * struct ipa5_5_rt_rule_hw_hdr - HW header of IPA routing rule
 * @word: routing rule header properties
 * @en_rule: enable rule - Equation bit fields
 * @pipe_dest_idx: destination pipe index
 * @stats_cnt_idx_lsb: stats cnt index
 * @priority: Rule priority. Added to distinguish rules order
 *  at the integrated table consisting from hashable and
 *  non-hashable parts
 * @ext_hdr: indicates whethere extention header is present or not.
 * @close_aggr_irq_mod: close aggregation/coalescing and close GSI
 *  interrupt moderation
 * @rule_id: rule ID that will be returned in the packet status
 * @hdr_offset: header offset
 * @proc_ctx: whether hdr_offset points to header table or to
 *	header processing context table
 * @retain_hdr: added to add back to the packet the header removed
 *  as part of header removal. This will be done as part of
 *  header insertion block.
 * @system: Is referenced header is lcl or sys memory
 */
struct ipa5_5_rt_rule_hw_hdr {
	union {
		u64 word;
		struct {
			u64 en_rule : 16;
			u64 pipe_dest_idx : 8;
			u64 stats_cnt_idx : 8;
			u64 priority : 8;
			u64 ext_hdr : 1;
			u64 close_aggr_irq_mod : 1;
			u64 rule_id : 10;
			u64 hdr_offset : 9;
			u64 proc_ctx : 1;
			u64 retain_hdr : 1;
			u64 system : 1;
		} hdr;
	} u;
};

/**
 * struct ipa5_5_rt_rule_hw_hdr_ext - HW header of IPA routing rule
 * extention
 * @word: routing rule extention header properties
 * @ttl: enable ttl decrement.
 * @qos_class: qos classification value.
 * @skip_ingress: Skip ingress policing.
 * @rsvd: Reserved bits
 */
struct ipa5_5_rt_rule_hw_hdr_ext {
	union {
		u16 word;
		struct {
			u16 ttl : 1;
			u16 qos_class : 6;
			u16 skip_ingress : 1;
			u16 rsvd : 8;
		} hdr;
	} u;
};


/**
 * struct ipa3_0_flt_rule_hw_hdr - HW header of IPA filter rule
 * @word: filtering rule properties
 * @en_rule: enable rule
 * @action: post filtering action
 * @rt_tbl_idx: index in routing table
 * @retain_hdr: added to add back to the packet the header removed
 *  as part of header removal. This will be done as part of
 *  header insertion block.
 * @rsvd1: reserved bits
 * @priority: Rule priority. Added to distinguish rules order
 *  at the integrated table consisting from hashable and
 *  non-hashable parts
 * @rsvd2: reserved bits
 * @rule_id: rule ID that will be returned in the packet status
 * @rsvd3: reserved bits
 */
struct ipa3_0_flt_rule_hw_hdr {
	union {
		u64 word;
		struct {
			u64 en_rule:16;
			u64 action:5;
			u64 rt_tbl_idx:5;
			u64 retain_hdr:1;
			u64 rsvd1:5;
			u64 priority:10;
			u64 rsvd2:6;
			u64 rule_id:10;
			u64 rsvd3:6;
		} hdr;
	} u;
};

/**
 * struct ipa4_0_flt_rule_hw_hdr - HW header of IPA filter rule
 * @word: filtering rule properties
 * @en_rule: enable rule
 * @action: post filtering action
 * @rt_tbl_idx: index in routing table
 * @retain_hdr: added to add back to the packet the header removed
 *  as part of header removal. This will be done as part of
 *  header insertion block.
 * @pdn_idx: in case of go to src nat action possible to input the pdn index to
 *  the NAT block
 * @set_metadata: enable metadata replacement in the NAT block
 * @priority: Rule priority. Added to distinguish rules order
 *  at the integrated table consisting from hashable and
 *  non-hashable parts
 * @rsvd2: reserved bits
 * @rule_id: rule ID that will be returned in the packet status
 * @rsvd3: reserved bits
 */
struct ipa4_0_flt_rule_hw_hdr {
	union {
		u64 word;
		struct {
			u64 en_rule : 16;
			u64 action : 5;
			u64 rt_tbl_idx : 5;
			u64 retain_hdr : 1;
			u64 pdn_idx : 4;
			u64 set_metadata : 1;
			u64 priority : 10;
			u64 rsvd2 : 6;
			u64 rule_id : 10;
			u64 rsvd3 : 6;
		} hdr;
	} u;
};

/**
 * struct ipa4_5_flt_rule_hw_hdr - HW header of IPA filter rule
 * @word: filtering rule properties
 * @en_rule: enable rule
 * @action: post filtering action
 * @rt_tbl_idx: index in routing table
 * @retain_hdr: added to add back to the packet the header removed
 *  as part of header removal. This will be done as part of
 *  header insertion block.
 * @pdn_idx: in case of go to src nat action possible to input the pdn index to
 *  the NAT block
 * @set_metadata: enable metadata replacement in the NAT block
 * @priority: Rule priority. Added to distinguish rules order
 *  at the integrated table consisting from hashable and
 *  non-hashable parts
 * @stats_cnt_idx_msb: stats cnt index msb
 * @rsvd2: reserved bits
 * @rule_id: rule ID that will be returned in the packet status
 * @stats_cnt_idx_lsb: stats cnt index lsb
 */
struct ipa4_5_flt_rule_hw_hdr {
	union {
		u64 word;
		struct {
			u64 en_rule : 16;
			u64 action : 5;
			u64 rt_tbl_idx : 5;
			u64 retain_hdr : 1;
			u64 pdn_idx : 4;
			u64 set_metadata : 1;
			u64 priority : 10;
			u64 stats_cnt_idx_msb : 2;
			u64 rsvd2 : 4;
			u64 rule_id : 10;
			u64 stats_cnt_idx_lsb : 6;
		} hdr;
	} u;
};

/**
 * struct ipa5_0_flt_rule_hw_hdr - HW header of IPA filter rule
 * @word: filtering rule properties
 * @en_rule: enable rule
 * @rt_tbl_idx: index in routing table
 * @stats_cnt_idx: stats cnt index
 * @priority: Rule priority. Added to distinguish rules order
 *  at the integrated table consisting from hashable and
 *  non-hashable parts
 * @close_aggr_irq_mod: close aggregation/coalescing and close GSI
 *  interrupt moderation
 * @rule_id: rule ID that will be returned in the packet status
 * @action: post filtering action
 * @pdn_idx: in case of go to src nat action possible to input the pdn index to
 *  the NAT block
 * @set_metadata: enable metadata replacement in the NAT block
 * @retain_hdr: added to add back to the packet the header removed
 *  as part of header removal. This will be done as part of
 *  header insertion block.
 * @rsvd1\rsvd2: reserved bits
 */
struct ipa5_0_flt_rule_hw_hdr {
	union {
		u64 word;
		struct {
			u64 en_rule : 16;
			u64 rt_tbl_idx : 8;
			u64 stats_cnt_idx : 8;
			u64 priority : 8;
			u64 rsvd1 : 1;
			u64 close_aggr_irq_mod : 1;
			u64 rule_id : 10;
			u64 action : 5;
			u64 pdn_idx : 4;
			u64 set_metadata : 1;
			u64 retain_hdr : 1;
			u64 rsvd2 : 1;
		} hdr;
	} u;
};

/**
 * struct ipa5_5_flt_rule_hw_hdr - HW header of IPA filter rule
 * @word: filtering rule properties
 * @en_rule: enable rule
 * @rt_tbl_idx: index in routing table
 * @stats_cnt_idx: stats cnt index
 * @priority: Rule priority. Added to distinguish rules order
 *  at the integrated table consisting from hashable and
 *  non-hashable parts
 * @ext_hdr: indicates whethere extention header is present or not.
 * @close_aggr_irq_mod: close aggregation/coalescing and close GSI
 *  interrupt moderation
 * @rule_id: rule ID that will be returned in the packet status
 * @action: post filtering action
 * @pdn_idx: in case of go to src nat action possible to input the pdn index to
 *  the NAT block
 * @set_metadata: enable metadata replacement in the NAT block
 * @retain_hdr: added to add back to the packet the header removed
 *  as part of header removal. This will be done as part of
 *  header insertion block.
 * @rsvd: reserved bits
 */
struct ipa5_5_flt_rule_hw_hdr {
	union {
		u64 word;
		struct {
			u64 en_rule : 16;
			u64 rt_tbl_idx : 8;
			u64 stats_cnt_idx : 8;
			u64 priority : 8;
			u64 ext_hdr : 1;
			u64 close_aggr_irq_mod : 1;
			u64 rule_id : 10;
			u64 action : 5;
			u64 pdn_idx : 4;
			u64 set_metadata : 1;
			u64 retain_hdr : 1;
			u64 rsvd : 1;
		} hdr;
	} u;
};

/**
 * struct ipa5_5_flt_rule_hw_hdr_ext - HW header of IPA filter rule
 * extention
 * @word: filtering rule extention header properties
 * @ttl: enable ttl decrement.
 * @qos_class: qos classification value.
 * @rsvd: Reserved bits
 */
struct ipa5_5_flt_rule_hw_hdr_ext {
	union {
		u16 word;
		struct {
			u16 ttl : 1;
			u16 qos_class : 6;
			u16 rsvd : 9;
		} hdr;
	} u;
};

int ipahal_fltrt_init(enum ipa_hw_type ipa_hw_type);
void ipahal_fltrt_destroy(void);

#endif /* _IPAHAL_FLTRT_I_H_ */
