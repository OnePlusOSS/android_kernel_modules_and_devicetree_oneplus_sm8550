// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include "ipahal.h"
#include "ipahal_i.h"
#include "ipahal_reg_i.h"
#include "ipahal_fltrt_i.h"
#include "ipahal_hw_stats_i.h"
#include "ipahal_nat_i.h"

#define CHECK_SET_PARAM(member, p_cmd_data, p_params, p_params_mask) \
	if ((p_params_mask)->member) {\
		(p_cmd_data)->member = (p_params)->member;\
	}

struct ipahal_context *ipahal_ctx;

static const char *ipahal_imm_cmd_name_to_str[IPA_IMM_CMD_MAX] = {
	__stringify(IPA_IMM_CMD_IP_V4_FILTER_INIT),
	__stringify(IPA_IMM_CMD_IP_V6_FILTER_INIT),
	__stringify(IPA_IMM_CMD_IP_V4_NAT_INIT),
	__stringify(IPA_IMM_CMD_IP_V4_ROUTING_INIT),
	__stringify(IPA_IMM_CMD_IP_V6_ROUTING_INIT),
	__stringify(IPA_IMM_CMD_HDR_INIT_LOCAL),
	__stringify(IPA_IMM_CMD_HDR_INIT_SYSTEM),
	__stringify(IPA_IMM_CMD_REGISTER_WRITE),
	__stringify(IPA_IMM_CMD_REGISTER_READ),
	__stringify(IPA_IMM_CMD_NAT_DMA),
	__stringify(IPA_IMM_CMD_IP_PACKET_INIT),
	__stringify(IPA_IMM_CMD_DMA_SHARED_MEM),
	__stringify(IPA_IMM_CMD_IP_PACKET_TAG_STATUS),
	__stringify(IPA_IMM_CMD_DMA_TASK_32B_ADDR),
	__stringify(IPA_IMM_CMD_TABLE_DMA),
	__stringify(IPA_IMM_CMD_IP_V6_CT_INIT),
	__stringify(IPA_IMM_CMD_IP_PACKET_INIT_EX),
};

static const char *ipahal_pkt_status_exception_to_str
	[IPAHAL_PKT_STATUS_EXCEPTION_MAX] = {
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_NONE),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_DEAGGR),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_IPTYPE),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_PACKET_LENGTH),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_PACKET_THRESHOLD),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_TTL),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_FRAG_RULE_MISS),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_SW_FILT),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_NAT),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_IPV6CT),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_UCP),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_INVALID_PIPE),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_HDRI),
	__stringify(IPAHAL_PKT_STATUS_EXCEPTION_CSUM),
};

/*
 * Forward declarations.
 */
static u16 ipahal_imm_cmd_get_opcode(enum ipahal_imm_cmd_name cmd);
static int ipahal_qmap_init(enum ipa_hw_type ipa_hw_type);

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_dma_task_32b_addr(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_dma_task_32b_addr *data;
	struct ipahal_imm_cmd_dma_task_32b_addr *dma_params =
		(struct ipahal_imm_cmd_dma_task_32b_addr *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld))
		return pyld;

	/* Currently supports only one packet */
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd) + (1 << 8);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_dma_task_32b_addr *)pyld->data;

	if (unlikely(dma_params->size1 & ~0xFFFF)) {
		WARN(1, "Size1 is bigger than 16bit width 0x%x\n",
			dma_params->size1);
	}
	if (unlikely(dma_params->packet_size & ~0xFFFF)) {
		WARN(1, "Pkt size is bigger than 16bit width 0x%x\n",
			dma_params->packet_size);
	}
	data->cmplt = dma_params->cmplt ? 1 : 0;
	data->eof = dma_params->eof ? 1 : 0;
	data->flsh = dma_params->flsh ? 1 : 0;
	data->lock = dma_params->lock ? 1 : 0;
	data->unlock = dma_params->unlock ? 1 : 0;
	data->size1 = dma_params->size1;
	data->addr1 = dma_params->addr1;
	data->packet_size = dma_params->packet_size;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_packet_tag_status(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_packet_tag_status *data;
	struct ipahal_imm_cmd_ip_packet_tag_status *tag_params =
		(struct ipahal_imm_cmd_ip_packet_tag_status *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_packet_tag_status *)pyld->data;

	if (unlikely(tag_params->tag & ~0xFFFFFFFFFFFF)) {
		IPAHAL_ERR("tag is bigger than 48bit width 0x%llx\n",
			tag_params->tag);
		WARN_ON(1);
	}
	data->tag = tag_params->tag;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_dma_shared_mem(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_dma_shared_mem *data;
	struct ipahal_imm_cmd_dma_shared_mem *mem_params =
		(struct ipahal_imm_cmd_dma_shared_mem *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld))
		return pyld;

	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_dma_shared_mem *)pyld->data;

	if (unlikely(mem_params->size & ~0xFFFF)) {
		WARN(1, "Size is bigger than 16bit width 0x%x\n",
			mem_params->size);
	}
	if (unlikely(mem_params->local_addr & ~0xFFFF)) {
		WARN(1, "Local addr is bigger than 16bit width 0x%x\n",
			mem_params->local_addr);
	}
	data->direction = mem_params->is_read ? 1 : 0;
	data->size = mem_params->size;
	data->local_addr = mem_params->local_addr;
	data->system_addr = mem_params->system_addr;
	data->skip_pipeline_clear = mem_params->skip_pipeline_clear ? 1 : 0;
	switch (mem_params->pipeline_clear_options) {
	case IPAHAL_HPS_CLEAR:
		data->pipeline_clear_options = 0;
		break;
	case IPAHAL_SRC_GRP_CLEAR:
		data->pipeline_clear_options = 1;
		break;
	case IPAHAL_FULL_PIPELINE_CLEAR:
		data->pipeline_clear_options = 2;
		break;
	default:
		IPAHAL_ERR("unsupported pipline clear option %d\n",
			mem_params->pipeline_clear_options);
		WARN_ON(1);
	}

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_dma_shared_mem_v_4_0(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_dma_shared_mem_v_4_0 *data;
	struct ipahal_imm_cmd_dma_shared_mem *mem_params =
		(struct ipahal_imm_cmd_dma_shared_mem *)params;

	if (unlikely(mem_params->size & ~0xFFFF)) {
		IPAHAL_ERR("Size is bigger than 16bit width 0x%x\n",
			mem_params->size);
		WARN_ON(1);
		return NULL;
	}
	if (unlikely(mem_params->local_addr & ~0xFFFF)) {
		IPAHAL_ERR("Local addr is bigger than 16bit width 0x%x\n",
			mem_params->local_addr);
		WARN_ON(1);
		return NULL;
	}

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		WARN_ON(1);
		return pyld;
	}

	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_dma_shared_mem_v_4_0 *)pyld->data;

	data->direction = mem_params->is_read ? 1 : 0;
	data->clear_after_read = mem_params->clear_after_read;
	data->size = mem_params->size;
	data->local_addr = mem_params->local_addr;
	data->system_addr = mem_params->system_addr;
	pyld->opcode |= (mem_params->skip_pipeline_clear ? 1 : 0) << 8;
	switch (mem_params->pipeline_clear_options) {
	case IPAHAL_HPS_CLEAR:
		break;
	case IPAHAL_SRC_GRP_CLEAR:
		pyld->opcode |= (1 << 9);
		break;
	case IPAHAL_FULL_PIPELINE_CLEAR:
		pyld->opcode |= (2 << 9);
		break;
	default:
		IPAHAL_ERR("unsupported pipline clear option %d\n",
			mem_params->pipeline_clear_options);
		WARN_ON(1);
	}

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_register_write(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_register_write *data;
	struct ipahal_imm_cmd_register_write *regwrt_params =
		(struct ipahal_imm_cmd_register_write *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_register_write *)pyld->data;

	if (unlikely(regwrt_params->offset & ~0xFFFF)) {
		IPAHAL_ERR("Offset is bigger than 16bit width 0x%x\n",
			regwrt_params->offset);
		WARN_ON(1);
	}
	data->offset = regwrt_params->offset;
	data->value = regwrt_params->value;
	data->value_mask = regwrt_params->value_mask;

	data->skip_pipeline_clear = regwrt_params->skip_pipeline_clear ? 1 : 0;
	switch (regwrt_params->pipeline_clear_options) {
	case IPAHAL_HPS_CLEAR:
		data->pipeline_clear_options = 0;
		break;
	case IPAHAL_SRC_GRP_CLEAR:
		data->pipeline_clear_options = 1;
		break;
	case IPAHAL_FULL_PIPELINE_CLEAR:
		data->pipeline_clear_options = 2;
		break;
	default:
		IPAHAL_ERR("unsupported pipline clear option %d\n",
			regwrt_params->pipeline_clear_options);
		WARN_ON(1);
	}

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_register_write_v_4_0(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_register_write_v_4_0 *data;
	struct ipahal_imm_cmd_register_write *regwrt_params =
		(struct ipahal_imm_cmd_register_write *)params;

	if (unlikely(regwrt_params->offset & ~0xFFFF)) {
		IPAHAL_ERR("Offset is bigger than 16bit width 0x%x\n",
			regwrt_params->offset);
		WARN_ON(1);
		return NULL;
	}

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		WARN_ON(1);
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_register_write_v_4_0 *)pyld->data;

	data->offset = regwrt_params->offset;
	data->offset_high = regwrt_params->offset >> 16;
	data->value = regwrt_params->value;
	data->value_mask = regwrt_params->value_mask;

	pyld->opcode |= (regwrt_params->skip_pipeline_clear ? 1 : 0) << 8;
	switch (regwrt_params->pipeline_clear_options) {
	case IPAHAL_HPS_CLEAR:
		break;
	case IPAHAL_SRC_GRP_CLEAR:
		pyld->opcode |= (1 << 9);
		break;
	case IPAHAL_FULL_PIPELINE_CLEAR:
		pyld->opcode |= (2 << 9);
		break;
	default:
		IPAHAL_ERR("unsupported pipline clear option %d\n",
			regwrt_params->pipeline_clear_options);
		WARN_ON(1);
	}

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_register_read(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_register_read *data;
	struct ipahal_imm_cmd_register_read *regrd_params =
		(struct ipahal_imm_cmd_register_read *)params;

	if (unlikely(regrd_params->offset & ~0xFFFF)) {
		IPAHAL_ERR("Offset is bigger than 16bit width 0x%x\n",
			regrd_params->offset);
		WARN_ON(1);
		return NULL;
	}

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		WARN_ON(1);
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_register_read *)pyld->data;

	data->offset = regrd_params->offset;
	data->offset_high = regrd_params->offset >> 16;
	data->sys_addr = regrd_params->sys_addr;

	pyld->opcode |= (regrd_params->skip_pipeline_clear ? 1 : 0) << 8;
	switch (regrd_params->pipeline_clear_options) {
	case IPAHAL_HPS_CLEAR:
		break;
	case IPAHAL_SRC_GRP_CLEAR:
		pyld->opcode |= (1 << 9);
		break;
	case IPAHAL_FULL_PIPELINE_CLEAR:
		pyld->opcode |= (2 << 9);
		break;
	default:
		IPAHAL_ERR("unsupported pipline clear option %d\n",
			regrd_params->pipeline_clear_options);
		WARN_ON(1);
	}

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_packet_init(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_packet_init *data;
	struct ipahal_imm_cmd_ip_packet_init *pktinit_params =
		(struct ipahal_imm_cmd_ip_packet_init *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_packet_init *)pyld->data;

	if (unlikely(pktinit_params->destination_pipe_index & ~0x1F)) {
		IPAHAL_ERR("Dst pipe idx is bigger than 5bit width 0x%x\n",
			pktinit_params->destination_pipe_index);
		WARN_ON(1);
	}
	data->destination_pipe_index = pktinit_params->destination_pipe_index;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_packet_init_v_5_0(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_packet_init_v_5_0 *data;
	struct ipahal_imm_cmd_ip_packet_init *pktinit_params =
		(struct ipahal_imm_cmd_ip_packet_init *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_packet_init_v_5_0 *)pyld->data;

	if (unlikely(pktinit_params->destination_pipe_index & ~0xFF)) {
		IPAHAL_ERR("Dst pipe idx is bigger than 8bit width 0x%x\n",
			pktinit_params->destination_pipe_index);
		WARN_ON(1);
	}
	data->destination_pipe_index = pktinit_params->destination_pipe_index;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_packet_init_ex(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_packet_init_ex *data;
	struct ipahal_imm_cmd_ip_packet_init_ex *packet_init_ex_params =
		(struct ipahal_imm_cmd_ip_packet_init_ex *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_packet_init_ex *)pyld->data;

	data->frag_disable = packet_init_ex_params->frag_disable;
	data->filter_disable = packet_init_ex_params->filter_disable;
	data->nat_disable = packet_init_ex_params->nat_disable;
	data->route_disable = packet_init_ex_params->route_disable;
	data->hdr_removal_insertion_disable =
	packet_init_ex_params->hdr_removal_insertion_disable;
	data->cs_disable = packet_init_ex_params->cs_disable;
	data->quota_tethering_stats_disable =
	packet_init_ex_params->quota_tethering_stats_disable;
	data->flt_rt_tbl_idx = packet_init_ex_params->flt_rt_tbl_idx;
	data->flt_stats_cnt_idx = packet_init_ex_params->flt_stats_cnt_idx;
	data->flt_priority = packet_init_ex_params->flt_priority;
	data->flt_close_aggr_irq_mod =
	packet_init_ex_params->flt_close_aggr_irq_mod;
	/* rule id value of 0x3FF is required */
	/*  (if not set correctly, filtering stats may be updated) */
	data->flt_rule_id = 0x3FF;
	data->flt_action = packet_init_ex_params->flt_action;
	data->flt_pdn_idx = packet_init_ex_params->flt_pdn_idx;
	data->flt_set_metadata = packet_init_ex_params->flt_set_metadata;
	data->flt_retain_hdr = packet_init_ex_params->flt_retain_hdr;
	data->rt_pipe_dest_idx = packet_init_ex_params->rt_pipe_dest_idx;
	data->rt_stats_cnt_idx = packet_init_ex_params->rt_stats_cnt_idx;
	data->rt_priority = packet_init_ex_params->rt_priority;
	data->rt_close_aggr_irq_mod =
		packet_init_ex_params->rt_close_aggr_irq_mod;
	/* rule id value of 0x3FF is required */
	/*  (if not set correctly, filtering stats may be updated) */
	data->rt_rule_id = 0x3FF;
	data->rt_hdr_offset = packet_init_ex_params->rt_hdr_offset;
	data->rt_proc_ctx = packet_init_ex_params->rt_proc_ctx;
	data->rt_retain_hdr = packet_init_ex_params->rt_retain_hdr;
	data->rt_system = packet_init_ex_params->rt_system;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_packet_init_ex_v5_5(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_packet_init_ex_v5_5 *data;
	struct ipahal_imm_cmd_ip_packet_init_ex *packet_init_ex_params =
		(struct ipahal_imm_cmd_ip_packet_init_ex *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_packet_init_ex_v5_5 *)pyld->data;

	data->frag_disable = packet_init_ex_params->frag_disable;
	data->filter_disable = packet_init_ex_params->filter_disable;
	data->nat_disable = packet_init_ex_params->nat_disable;
	data->route_disable = packet_init_ex_params->route_disable;
	data->hdr_removal_insertion_disable =
	packet_init_ex_params->hdr_removal_insertion_disable;
	data->cs_disable = packet_init_ex_params->cs_disable;
	data->quota_tethering_stats_disable =
	packet_init_ex_params->quota_tethering_stats_disable;
	data->dpl_disable = packet_init_ex_params->dpl_disable;
	data->flt_rt_tbl_idx = packet_init_ex_params->flt_rt_tbl_idx;
	data->flt_stats_cnt_idx = packet_init_ex_params->flt_stats_cnt_idx;
	data->flt_priority = packet_init_ex_params->flt_priority;
	data->flt_ext_hdr = packet_init_ex_params->flt_ext_hdr;
	data->flt_close_aggr_irq_mod =
	packet_init_ex_params->flt_close_aggr_irq_mod;
	/* rule id value of 0x3FF is required */
	/*  (if not set correctly, filtering stats may be updated) */
	data->flt_rule_id = 0x3FF;
	data->flt_action = packet_init_ex_params->flt_action;
	data->flt_pdn_idx = packet_init_ex_params->flt_pdn_idx;
	data->flt_set_metadata = packet_init_ex_params->flt_set_metadata;
	data->flt_retain_hdr = packet_init_ex_params->flt_retain_hdr;
	data->flt_ttl = packet_init_ex_params->flt_ttl;
	data->flt_qos_class = packet_init_ex_params->flt_qos_class;
	data->rt_pipe_dest_idx = packet_init_ex_params->rt_pipe_dest_idx;
	data->rt_stats_cnt_idx = packet_init_ex_params->rt_stats_cnt_idx;
	data->rt_priority = packet_init_ex_params->rt_priority;
	data->rt_ext_hdr = packet_init_ex_params->rt_ext_hdr;
	data->rt_close_aggr_irq_mod =
		packet_init_ex_params->rt_close_aggr_irq_mod;
	/* rule id value of 0x3FF is required */
	/*  (if not set correctly, filtering stats may be updated) */
	data->rt_rule_id = 0x3FF;
	data->rt_hdr_offset = packet_init_ex_params->rt_hdr_offset;
	data->rt_proc_ctx = packet_init_ex_params->rt_proc_ctx;
	data->rt_retain_hdr = packet_init_ex_params->rt_retain_hdr;
	data->rt_system = packet_init_ex_params->rt_system;
	data->rt_ttl = packet_init_ex_params->rt_ttl;
	data->rt_qos_class = packet_init_ex_params->rt_qos_class;
	data->rt_skip_ingress = packet_init_ex_params->rt_skip_ingress;
	return pyld;
}

int ipa_imm_cmd_modify_ip_packet_init_ex(
	enum ipahal_imm_cmd_name cmd,
	const void *cmd_data,
	const void *params,
	const void *params_mask)
{
	struct ipa_imm_cmd_hw_ip_packet_init_ex *data =
		(struct ipa_imm_cmd_hw_ip_packet_init_ex *)cmd_data;
	struct ipahal_imm_cmd_ip_packet_init_ex *mask =
		(struct ipahal_imm_cmd_ip_packet_init_ex *)params_mask;
	struct ipahal_imm_cmd_ip_packet_init_ex *prms =
		(struct ipahal_imm_cmd_ip_packet_init_ex *)params;

	CHECK_SET_PARAM(frag_disable, data, prms, mask);
	CHECK_SET_PARAM(filter_disable, data, prms, mask);
	CHECK_SET_PARAM(nat_disable, data, prms, mask);
	CHECK_SET_PARAM(route_disable, data, prms, mask);
	CHECK_SET_PARAM(hdr_removal_insertion_disable, data, prms, mask);
	CHECK_SET_PARAM(cs_disable, data, prms, mask);
	CHECK_SET_PARAM(quota_tethering_stats_disable, data, prms, mask);
	CHECK_SET_PARAM(flt_rt_tbl_idx, data, prms, mask);
	CHECK_SET_PARAM(flt_stats_cnt_idx, data, prms, mask);
	CHECK_SET_PARAM(flt_priority, data, prms, mask);
	CHECK_SET_PARAM(flt_close_aggr_irq_mod, data, prms, mask);
	CHECK_SET_PARAM(flt_action, data, prms, mask);
	CHECK_SET_PARAM(flt_pdn_idx, data, prms, mask);
	CHECK_SET_PARAM(flt_set_metadata, data, prms, mask);
	CHECK_SET_PARAM(flt_retain_hdr, data, prms, mask);
	CHECK_SET_PARAM(rt_pipe_dest_idx, data, prms, mask);
	CHECK_SET_PARAM(rt_stats_cnt_idx, data, prms, mask);
	CHECK_SET_PARAM(rt_priority, data, prms, mask);
	CHECK_SET_PARAM(rt_close_aggr_irq_mod, data, prms, mask);
	CHECK_SET_PARAM(rt_hdr_offset, data, prms, mask);
	CHECK_SET_PARAM(rt_proc_ctx, data, prms, mask);
	CHECK_SET_PARAM(rt_retain_hdr, data, prms, mask);
	CHECK_SET_PARAM(rt_system, data, prms, mask);

	return 0;
}

static int ipa_imm_cmd_modify_ip_packet_init_ex_v5_5(
	enum ipahal_imm_cmd_name cmd,
	const void *cmd_data,
	const void *params,
	const void *params_mask)
{
	struct ipa_imm_cmd_hw_ip_packet_init_ex_v5_5 *data =
		(struct ipa_imm_cmd_hw_ip_packet_init_ex_v5_5 *)cmd_data;
	struct ipahal_imm_cmd_ip_packet_init_ex *mask =
		(struct ipahal_imm_cmd_ip_packet_init_ex *)params_mask;
	struct ipahal_imm_cmd_ip_packet_init_ex *prms =
		(struct ipahal_imm_cmd_ip_packet_init_ex *)params;

	CHECK_SET_PARAM(frag_disable, data, prms, mask);
	CHECK_SET_PARAM(filter_disable, data, prms, mask);
	CHECK_SET_PARAM(nat_disable, data, prms, mask);
	CHECK_SET_PARAM(route_disable, data, prms, mask);
	CHECK_SET_PARAM(hdr_removal_insertion_disable, data, prms, mask);
	CHECK_SET_PARAM(cs_disable, data, prms, mask);
	CHECK_SET_PARAM(quota_tethering_stats_disable, data, prms, mask);
	CHECK_SET_PARAM(dpl_disable, data, prms, mask);
	CHECK_SET_PARAM(flt_rt_tbl_idx, data, prms, mask);
	CHECK_SET_PARAM(flt_stats_cnt_idx, data, prms, mask);
	CHECK_SET_PARAM(flt_priority, data, prms, mask);
	CHECK_SET_PARAM(flt_close_aggr_irq_mod, data, prms, mask);
	CHECK_SET_PARAM(flt_action, data, prms, mask);
	CHECK_SET_PARAM(flt_pdn_idx, data, prms, mask);
	CHECK_SET_PARAM(flt_set_metadata, data, prms, mask);
	CHECK_SET_PARAM(flt_retain_hdr, data, prms, mask);
	CHECK_SET_PARAM(rt_pipe_dest_idx, data, prms, mask);
	CHECK_SET_PARAM(rt_stats_cnt_idx, data, prms, mask);
	CHECK_SET_PARAM(rt_priority, data, prms, mask);
	CHECK_SET_PARAM(rt_close_aggr_irq_mod, data, prms, mask);
	CHECK_SET_PARAM(rt_hdr_offset, data, prms, mask);
	CHECK_SET_PARAM(rt_proc_ctx, data, prms, mask);
	CHECK_SET_PARAM(rt_retain_hdr, data, prms, mask);
	CHECK_SET_PARAM(rt_system, data, prms, mask);
	CHECK_SET_PARAM(flt_ext_hdr, data, prms, mask);
	CHECK_SET_PARAM(flt_ttl, data, prms, mask);
	CHECK_SET_PARAM(flt_qos_class, data, prms, mask);
	CHECK_SET_PARAM(rt_ext_hdr, data, prms, mask);
	CHECK_SET_PARAM(rt_ttl, data, prms, mask);
	CHECK_SET_PARAM(rt_qos_class, data, prms, mask);
	CHECK_SET_PARAM(rt_skip_ingress, data, prms, mask);
	return 0;
}

inline void ipa_imm_cmd_modify_ip_packet_init_ex_dest_pipe_v5_5(
	const void *cmd_data,
	u64 pipe_dest_idx)
{
	((struct ipa_imm_cmd_hw_ip_packet_init_ex_v5_5 *)cmd_data)->rt_pipe_dest_idx
		= pipe_dest_idx;
}

inline void ipa_imm_cmd_modify_ip_packet_init_ex_dest_pipe(
	const void *cmd_data,
	u64 pipe_dest_idx)
{
	if (ipahal_ctx->hw_type >= IPA_HW_v5_5)
		return ipa_imm_cmd_modify_ip_packet_init_ex_dest_pipe_v5_5(cmd_data,
			pipe_dest_idx);
	else
		((struct ipa_imm_cmd_hw_ip_packet_init_ex *)cmd_data)->rt_pipe_dest_idx
			= pipe_dest_idx;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_nat_dma(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_nat_dma *data;
	struct ipahal_imm_cmd_table_dma *nat_params =
		(struct ipahal_imm_cmd_table_dma *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_nat_dma *)pyld->data;

	data->table_index = nat_params->table_index;
	data->base_addr = nat_params->base_addr;
	data->offset = nat_params->offset;
	data->data = nat_params->data;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_table_dma_ipav4(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_table_dma_ipav4 *data;
	struct ipahal_imm_cmd_table_dma *nat_params =
		(struct ipahal_imm_cmd_table_dma *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_table_dma_ipav4 *)pyld->data;

	data->table_index = nat_params->table_index;
	data->base_addr = nat_params->base_addr;
	data->offset = nat_params->offset;
	data->data = nat_params->data;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_hdr_init_system(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_hdr_init_system *data;
	struct ipahal_imm_cmd_hdr_init_system *syshdr_params =
		(struct ipahal_imm_cmd_hdr_init_system *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_hdr_init_system *)pyld->data;

	data->hdr_table_addr = syshdr_params->hdr_table_addr;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_hdr_init_local(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_hdr_init_local *data;
	struct ipahal_imm_cmd_hdr_init_local *lclhdr_params =
		(struct ipahal_imm_cmd_hdr_init_local *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_hdr_init_local *)pyld->data;

	if (unlikely(lclhdr_params->size_hdr_table & ~0xFFF)) {
		IPAHAL_ERR("Hdr tble size is bigger than 12bit width 0x%x\n",
			lclhdr_params->size_hdr_table);
		WARN_ON(1);
	}
	data->hdr_table_addr = lclhdr_params->hdr_table_addr;
	data->size_hdr_table = lclhdr_params->size_hdr_table;
	data->hdr_addr = lclhdr_params->hdr_addr;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_v6_routing_init(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_v6_routing_init *data;
	struct ipahal_imm_cmd_ip_v6_routing_init *rt6_params =
		(struct ipahal_imm_cmd_ip_v6_routing_init *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_v6_routing_init *)pyld->data;

	data->hash_rules_addr = rt6_params->hash_rules_addr;
	data->hash_rules_size = rt6_params->hash_rules_size;
	data->hash_local_addr = rt6_params->hash_local_addr;
	data->nhash_rules_addr = rt6_params->nhash_rules_addr;
	data->nhash_rules_size = rt6_params->nhash_rules_size;
	data->nhash_local_addr = rt6_params->nhash_local_addr;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_v4_routing_init(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_v4_routing_init *data;
	struct ipahal_imm_cmd_ip_v4_routing_init *rt4_params =
		(struct ipahal_imm_cmd_ip_v4_routing_init *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_v4_routing_init *)pyld->data;

	data->hash_rules_addr = rt4_params->hash_rules_addr;
	data->hash_rules_size = rt4_params->hash_rules_size;
	data->hash_local_addr = rt4_params->hash_local_addr;
	data->nhash_rules_addr = rt4_params->nhash_rules_addr;
	data->nhash_rules_size = rt4_params->nhash_rules_size;
	data->nhash_local_addr = rt4_params->nhash_local_addr;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_v4_nat_init(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_v4_nat_init *data;
	struct ipahal_imm_cmd_ip_v4_nat_init *nat4_params =
		(struct ipahal_imm_cmd_ip_v4_nat_init *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_v4_nat_init *)pyld->data;

	data->ipv4_rules_addr = nat4_params->table_init.base_table_addr;
	data->ipv4_expansion_rules_addr =
		nat4_params->table_init.expansion_table_addr;
	data->index_table_addr = nat4_params->index_table_addr;
	data->index_table_expansion_addr =
		nat4_params->index_table_expansion_addr;
	data->table_index = nat4_params->table_init.table_index;
	data->ipv4_rules_addr_type =
		nat4_params->table_init.base_table_addr_shared ? 1 : 0;
	data->ipv4_expansion_rules_addr_type =
		nat4_params->table_init.expansion_table_addr_shared ? 1 : 0;
	data->index_table_addr_type =
		nat4_params->index_table_addr_shared ? 1 : 0;
	data->index_table_expansion_addr_type =
		nat4_params->index_table_expansion_addr_shared ? 1 : 0;
	data->size_base_tables = nat4_params->table_init.size_base_table;
	data->size_expansion_tables =
		nat4_params->table_init.size_expansion_table;
	data->public_addr_info = nat4_params->public_addr_info;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_v6_ct_init(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_v6_ct_init *data;
	struct ipahal_imm_cmd_ip_v6_ct_init *ipv6ct_params =
		(struct ipahal_imm_cmd_ip_v6_ct_init *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld))
		return pyld;
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_v6_ct_init *)pyld->data;

	data->table_addr = ipv6ct_params->table_init.base_table_addr;
	data->expansion_table_addr =
		ipv6ct_params->table_init.expansion_table_addr;
	data->table_index = ipv6ct_params->table_init.table_index;
	data->table_addr_type =
		ipv6ct_params->table_init.base_table_addr_shared ? 1 : 0;
	data->expansion_table_addr_type =
		ipv6ct_params->table_init.expansion_table_addr_shared ? 1 : 0;
	data->size_base_table = ipv6ct_params->table_init.size_base_table;
	data->size_expansion_table =
		ipv6ct_params->table_init.size_expansion_table;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_v6_filter_init(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_v6_filter_init *data;
	struct ipahal_imm_cmd_ip_v6_filter_init *flt6_params =
		(struct ipahal_imm_cmd_ip_v6_filter_init *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_v6_filter_init *)pyld->data;

	data->hash_rules_addr = flt6_params->hash_rules_addr;
	data->hash_rules_size = flt6_params->hash_rules_size;
	data->hash_local_addr = flt6_params->hash_local_addr;
	data->nhash_rules_addr = flt6_params->nhash_rules_addr;
	data->nhash_rules_size = flt6_params->nhash_rules_size;
	data->nhash_local_addr = flt6_params->nhash_local_addr;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_ip_v4_filter_init(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_pyld *pyld;
	struct ipa_imm_cmd_hw_ip_v4_filter_init *data;
	struct ipahal_imm_cmd_ip_v4_filter_init *flt4_params =
		(struct ipahal_imm_cmd_ip_v4_filter_init *)params;

	pyld = IPAHAL_MEM_ALLOC(sizeof(*pyld) + sizeof(*data), is_atomic_ctx);
	if (unlikely(!pyld)) {
		IPAHAL_ERR("kzalloc err\n");
		return pyld;
	}
	pyld->opcode = ipahal_imm_cmd_get_opcode(cmd);
	pyld->len = sizeof(*data);
	data = (struct ipa_imm_cmd_hw_ip_v4_filter_init *)pyld->data;

	data->hash_rules_addr = flt4_params->hash_rules_addr;
	data->hash_rules_size = flt4_params->hash_rules_size;
	data->hash_local_addr = flt4_params->hash_local_addr;
	data->nhash_rules_addr = flt4_params->nhash_rules_addr;
	data->nhash_rules_size = flt4_params->nhash_rules_size;
	data->nhash_local_addr = flt4_params->nhash_local_addr;

	return pyld;
}

static struct ipahal_imm_cmd_pyld *ipa_imm_cmd_construct_dummy(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	IPAHAL_ERR("no construct function for IMM_CMD=%s, IPA ver %d\n",
		ipahal_imm_cmd_name_str(cmd), ipahal_ctx->hw_type);
	WARN_ON(1);
	return NULL;
}

static int ipa_imm_cmd_modify_dummy(
	enum ipahal_imm_cmd_name cmd,
	const void *cmd_data,
	const void *params,
	const void *params_mask)
{
	IPAHAL_ERR("no modify function for IMM_CMD=%s, IPA ver %d\n",
		ipahal_imm_cmd_name_str(cmd), ipahal_ctx->hw_type);
	WARN_ON(1);
	return -EINVAL;
}

/*
 * struct ipahal_imm_cmd_obj - immediate command H/W information for
 *  specific IPA version
 * @construct - CB to construct imm command payload from abstracted structure
 * @modify - CB to modify imm command payload from abstracted structure
 * @opcode - Immediate command OpCode
 */
struct ipahal_imm_cmd_obj {
	struct ipahal_imm_cmd_pyld *(*construct)(enum ipahal_imm_cmd_name cmd,
		const void *params, bool is_atomic_ctx);
	int (*modify)(enum ipahal_imm_cmd_name cmd,
		      const void *cmd_data,
		      const void *params,
		      const void *params_mask);
	u16 opcode;
};

/*
 * This table contains the info regard each immediate command for IPAv3
 *  and later.
 * Information like: opcode and construct functions.
 * All the information on the IMM on IPAv3 are statically defined below.
 * If information is missing regard some IMM on some IPA version,
 *  the init function will fill it with the information from the previous
 *  IPA version.
 * Information is considered missing if all of the fields are 0
 * If opcode is -1, this means that the IMM is removed on the
 *  specific version
 */
static struct ipahal_imm_cmd_obj
		ipahal_imm_cmd_objs[IPA_HW_MAX][IPA_IMM_CMD_MAX] = {
	/* IPAv3 */
	[IPA_HW_v3_0][IPA_IMM_CMD_IP_V4_FILTER_INIT] = {
		ipa_imm_cmd_construct_ip_v4_filter_init,
		ipa_imm_cmd_modify_dummy,
		3},
	[IPA_HW_v3_0][IPA_IMM_CMD_IP_V6_FILTER_INIT] = {
		ipa_imm_cmd_construct_ip_v6_filter_init,
		ipa_imm_cmd_modify_dummy,
		4},
	[IPA_HW_v3_0][IPA_IMM_CMD_IP_V4_NAT_INIT] = {
		ipa_imm_cmd_construct_ip_v4_nat_init,
		ipa_imm_cmd_modify_dummy,
		5},
	[IPA_HW_v3_0][IPA_IMM_CMD_IP_V4_ROUTING_INIT] = {
		ipa_imm_cmd_construct_ip_v4_routing_init,
		ipa_imm_cmd_modify_dummy,
		7},
	[IPA_HW_v3_0][IPA_IMM_CMD_IP_V6_ROUTING_INIT] = {
		ipa_imm_cmd_construct_ip_v6_routing_init,
		ipa_imm_cmd_modify_dummy,
		8},
	[IPA_HW_v3_0][IPA_IMM_CMD_HDR_INIT_LOCAL] = {
		ipa_imm_cmd_construct_hdr_init_local,
		ipa_imm_cmd_modify_dummy,
		9},
	[IPA_HW_v3_0][IPA_IMM_CMD_HDR_INIT_SYSTEM] = {
		ipa_imm_cmd_construct_hdr_init_system,
		ipa_imm_cmd_modify_dummy,
		10},
	[IPA_HW_v3_0][IPA_IMM_CMD_REGISTER_WRITE] = {
		ipa_imm_cmd_construct_register_write,
		ipa_imm_cmd_modify_dummy,
		12},
	[IPA_HW_v3_0][IPA_IMM_CMD_NAT_DMA] = {
		ipa_imm_cmd_construct_nat_dma,
		ipa_imm_cmd_modify_dummy,
		14},
	[IPA_HW_v3_0][IPA_IMM_CMD_IP_PACKET_INIT] = {
		ipa_imm_cmd_construct_ip_packet_init,
		ipa_imm_cmd_modify_dummy,
		16},
	[IPA_HW_v3_0][IPA_IMM_CMD_DMA_TASK_32B_ADDR] = {
		ipa_imm_cmd_construct_dma_task_32b_addr,
		ipa_imm_cmd_modify_dummy,
		17},
	[IPA_HW_v3_0][IPA_IMM_CMD_DMA_SHARED_MEM] = {
		ipa_imm_cmd_construct_dma_shared_mem,
		ipa_imm_cmd_modify_dummy,
		19},
	[IPA_HW_v3_0][IPA_IMM_CMD_IP_PACKET_TAG_STATUS] = {
		ipa_imm_cmd_construct_ip_packet_tag_status,
		ipa_imm_cmd_modify_dummy,
		20},

	/* IPAv4 */
	[IPA_HW_v4_0][IPA_IMM_CMD_REGISTER_WRITE] = {
		ipa_imm_cmd_construct_register_write_v_4_0,
		ipa_imm_cmd_modify_dummy,
		12},
	/* NAT_DMA was renamed to TABLE_DMA for IPAv4 */
	[IPA_HW_v4_0][IPA_IMM_CMD_NAT_DMA] = {
		ipa_imm_cmd_construct_dummy,
		ipa_imm_cmd_modify_dummy,
		-1},
	[IPA_HW_v4_0][IPA_IMM_CMD_TABLE_DMA] = {
		ipa_imm_cmd_construct_table_dma_ipav4,
		ipa_imm_cmd_modify_dummy,
		14},
	[IPA_HW_v4_0][IPA_IMM_CMD_DMA_SHARED_MEM] = {
		ipa_imm_cmd_construct_dma_shared_mem_v_4_0,
		ipa_imm_cmd_modify_dummy,
		19},
	[IPA_HW_v4_0][IPA_IMM_CMD_IP_V6_CT_INIT] = {
		ipa_imm_cmd_construct_ip_v6_ct_init,
		ipa_imm_cmd_modify_dummy,
		23},

	/* IPAv5 */
	[IPA_HW_v5_0][IPA_IMM_CMD_IP_PACKET_INIT] = {
		ipa_imm_cmd_construct_ip_packet_init_v_5_0,
		ipa_imm_cmd_modify_dummy,
		16},
	[IPA_HW_v5_0][IPA_IMM_CMD_IP_PACKET_INIT_EX] = {
		ipa_imm_cmd_construct_ip_packet_init_ex,
		ipa_imm_cmd_modify_ip_packet_init_ex,
		18},

	[IPA_HW_v5_1][IPA_IMM_CMD_REGISTER_READ] = {
		ipa_imm_cmd_construct_register_read,
		ipa_imm_cmd_modify_dummy,
		13},
	/* IPAv5_5 */
	[IPA_HW_v5_5][IPA_IMM_CMD_IP_PACKET_INIT_EX] = {
		ipa_imm_cmd_construct_ip_packet_init_ex_v5_5,
		ipa_imm_cmd_modify_ip_packet_init_ex_v5_5,
		18},
};

/*
 * ipahal_imm_cmd_init() - Build the Immediate command information table
 *  See ipahal_imm_cmd_objs[][] comments
 */
static int ipahal_imm_cmd_init(enum ipa_hw_type ipa_hw_type)
{
	int i;
	int j;
	struct ipahal_imm_cmd_obj zero_obj;

	IPAHAL_DBG_LOW("Entry - HW_TYPE=%d\n", ipa_hw_type);

	if ((ipa_hw_type < 0) || (ipa_hw_type >= IPA_HW_MAX)) {
		IPAHAL_ERR("invalid IPA HW type (%d)\n", ipa_hw_type);
		return -EINVAL;
	}

	memset(&zero_obj, 0, sizeof(zero_obj));
	for (i = IPA_HW_v3_0 ; i < ipa_hw_type ; i++) {
		for (j = 0; j < IPA_IMM_CMD_MAX ; j++) {
			if (!memcmp(&ipahal_imm_cmd_objs[i+1][j], &zero_obj,
				sizeof(struct ipahal_imm_cmd_obj))) {
				memcpy(&ipahal_imm_cmd_objs[i+1][j],
					&ipahal_imm_cmd_objs[i][j],
					sizeof(struct ipahal_imm_cmd_obj));
			} else {
				/*
				 * explicitly overridden immediate command.
				 * Check validity
				 */
				if (!ipahal_imm_cmd_objs[i+1][j].opcode) {
					IPAHAL_ERR(
					  "imm_cmd=%s with zero opcode ipa_ver=%d\n",
					  ipahal_imm_cmd_name_str(j), i+1);
					WARN_ON(1);
				}
				if (!ipahal_imm_cmd_objs[i+1][j].construct) {
					IPAHAL_ERR(
					  "imm_cmd=%s with NULL construct func ipa_ver=%d\n",
					  ipahal_imm_cmd_name_str(j), i+1);
					WARN_ON(1);
				}
			}
		}
	}

	return 0;
}

/*
 * ipahal_imm_cmd_name_str() - returns string that represent the imm cmd
 * @cmd_name: [in] Immediate command name
 */
const char *ipahal_imm_cmd_name_str(enum ipahal_imm_cmd_name cmd_name)
{
	if (cmd_name < 0 || cmd_name >= IPA_IMM_CMD_MAX) {
		IPAHAL_ERR("requested name of invalid imm_cmd=%d\n", cmd_name);
		return "Invalid IMM_CMD";
	}

	return ipahal_imm_cmd_name_to_str[cmd_name];
}

/*
 * ipahal_imm_cmd_get_opcode() - Get the fixed opcode of the immediate command
 */
static u16 ipahal_imm_cmd_get_opcode(enum ipahal_imm_cmd_name cmd)
{
	u32 opcode;

	if (cmd >= IPA_IMM_CMD_MAX) {
		IPAHAL_ERR("Invalid immediate command imm_cmd=%u\n", cmd);
		ipa_assert();
		return -EFAULT;
	}

	IPAHAL_DBG_LOW("Get opcode of IMM_CMD=%s\n",
		ipahal_imm_cmd_name_str(cmd));
	opcode = ipahal_imm_cmd_objs[ipahal_ctx->hw_type][cmd].opcode;
	if (opcode == -1) {
		IPAHAL_ERR("Try to get opcode of obsolete IMM_CMD=%s\n",
			ipahal_imm_cmd_name_str(cmd));
		ipa_assert();
		return -EFAULT;
	}

	return opcode;
}

/*
 * ipahal_construct_imm_cmd() - Construct immdiate command
 * This function builds imm cmd bulk that can be be sent to IPA
 * The command will be allocated dynamically.
 * After done using it, call ipahal_destroy_imm_cmd() to release it
 */
struct ipahal_imm_cmd_pyld *ipahal_construct_imm_cmd(
	enum ipahal_imm_cmd_name cmd, const void *params, bool is_atomic_ctx)
{
	if (!params) {
		IPAHAL_ERR("Input error: params=%pK\n", params);
		ipa_assert();
		return NULL;
	}

	if (cmd >= IPA_IMM_CMD_MAX) {
		IPAHAL_ERR("Invalid immediate command %u\n", cmd);
		return NULL;
	}

	IPAHAL_DBG_LOW("construct IMM_CMD:%s\n", ipahal_imm_cmd_name_str(cmd));
	return ipahal_imm_cmd_objs[ipahal_ctx->hw_type][cmd].construct(
		cmd, params, is_atomic_ctx);
}

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
	const void *params_mask)
{
	if (!cmd_data || !params || !params_mask) {
		WARN_ONCE(true,
			"Input error: cmd_data=%pK params=%pK params_mask=%pK\n",
			cmd_data, params, params_mask);
		return -EINVAL;
	}

	if (cmd >= IPA_IMM_CMD_MAX) {
		IPAHAL_ERR("Invalid immediate command %u\n", cmd);
		return -EINVAL;
	}

	IPAHAL_DBG_LOW("Modify IMM_CMD:%s\n", ipahal_imm_cmd_name_str(cmd));
	return ipahal_imm_cmd_objs[ipahal_ctx->hw_type][cmd].modify(
		cmd,
		cmd_data,
		params,
		params_mask);
}

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
	bool is_atomic_ctx)
{
	struct ipahal_imm_cmd_register_write cmd;
	struct ipahal_imm_cmd_pyld *cmd_pyld;

	memset(&cmd, 0, sizeof(cmd));
	cmd.skip_pipeline_clear = skip_pipline_clear;
	cmd.pipeline_clear_options = pipline_clr_opt;
	cmd.value_mask = 0x0;

	cmd_pyld = ipahal_construct_imm_cmd(IPA_IMM_CMD_REGISTER_WRITE,
		&cmd, is_atomic_ctx);

	if (!cmd_pyld)
		IPAHAL_ERR("failed to construct register_write imm cmd\n");

	return cmd_pyld;
}


/* IPA Packet Status Logic */

#define IPA_PKT_STATUS_SET_MSK(__hw_bit_msk, __shft) \
	(status->status_mask |= \
		((hw_status_mask) & (__hw_bit_msk) ? 1 : 0) << (__shft))

static enum ipahal_pkt_status_exception pkt_status_parse_exception(
	bool is_ipv6, u64 exception)
{
	enum ipahal_pkt_status_exception exception_type = 0;

	switch (exception) {
	case 0:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_NONE;
		break;
	case 1:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_DEAGGR;
		break;
	case 4:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_IPTYPE;
		break;
	case 8:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_PACKET_LENGTH;
		break;
	case 10:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_TTL;
		break;
	case 16:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_FRAG_RULE_MISS;
		break;
	case 32:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_SW_FILT;
		break;
	case 64:
		if (is_ipv6)
			exception_type = IPAHAL_PKT_STATUS_EXCEPTION_IPV6CT;
		else
			exception_type = IPAHAL_PKT_STATUS_EXCEPTION_NAT;
		break;
	case 128:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_UCP;
		break;
	case 129:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_INVALID_PIPE;
		break;
	case 131:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_RQOS;
		break;
	case 136:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_HDRI;
		break;
	case 229:
		exception_type = IPAHAL_PKT_STATUS_EXCEPTION_CSUM;
		break;
	default:
		IPAHAL_ERR("unsupported Status Exception type 0x%x\n",
			exception);
		WARN_ON(1);
	}

	return exception_type;
}

static void __ipa_parse_gen_pkt(struct ipahal_pkt_status *status,
				const void *unparsed_status)
{
	bool is_ipv6;
	union ipa_pkt_status_hw *hw_status =
		(union ipa_pkt_status_hw *)unparsed_status;

	is_ipv6 = (hw_status->ipa_pkt.status_mask & 0x80) ? false : true;
	status->pkt_len = hw_status->ipa_pkt.pkt_len;
	status->endp_src_idx = hw_status->ipa_pkt.endp_src_idx;
	status->endp_dest_idx = hw_status->ipa_pkt.endp_dest_idx;
	status->metadata = hw_status->ipa_pkt.metadata;
	status->flt_local = hw_status->ipa_pkt.flt_local;
	status->flt_hash = hw_status->ipa_pkt.flt_hash;
	status->flt_global = hw_status->ipa_pkt.flt_hash;
	status->flt_ret_hdr = hw_status->ipa_pkt.flt_ret_hdr;
	status->flt_miss = (hw_status->ipa_pkt.rt_rule_id ==
			IPAHAL_PKT_STATUS_FLTRT_RULE_MISS_ID);
	status->flt_rule_id = hw_status->ipa_pkt.flt_rule_id;
	status->rt_local = hw_status->ipa_pkt.rt_local;
	status->rt_hash = hw_status->ipa_pkt.rt_hash;
	status->ucp = hw_status->ipa_pkt.ucp;
	status->rt_tbl_idx = hw_status->ipa_pkt.rt_tbl_idx;
	status->rt_miss = (hw_status->ipa_pkt.rt_rule_id ==
			IPAHAL_PKT_STATUS_FLTRT_RULE_MISS_ID);
	status->rt_rule_id = hw_status->ipa_pkt.rt_rule_id;
	status->nat_hit = hw_status->ipa_pkt.nat_hit;
	status->nat_entry_idx = hw_status->ipa_pkt.nat_entry_idx;
	status->tag_info = hw_status->ipa_pkt.tag_info;
	status->seq_num = hw_status->ipa_pkt.seq_num;
	status->time_of_day_ctr = hw_status->ipa_pkt.time_of_day_ctr;
	status->hdr_local = hw_status->ipa_pkt.hdr_local;
	status->hdr_offset = hw_status->ipa_pkt.hdr_offset;
	status->frag_hit = hw_status->ipa_pkt.frag_hit;
	status->frag_rule = hw_status->ipa_pkt.frag_rule;
	status->nat_type = hw_status->ipa_pkt.nat_type;

	status->exception = pkt_status_parse_exception(is_ipv6,
			hw_status->ipa_pkt.exception);
}

static void __ipa_parse_frag_pkt(struct ipahal_pkt_status *status,
				const void *unparsed_status)
{
	union ipa_pkt_status_hw *hw_status =
		(union ipa_pkt_status_hw *)unparsed_status;

	status->frag_rule_idx = hw_status->frag_pkt.frag_rule_idx;
	status->tbl_idx = hw_status->frag_pkt.tbl_idx;
	status->src_ip_addr = hw_status->frag_pkt.src_ip_addr;
	status->dest_ip_addr = hw_status->frag_pkt.dest_ip_addr;
	status->protocol = hw_status->frag_pkt.protocol;
	status->ip_id = hw_status->frag_pkt.ip_id;
	status->tlated_ip_addr = hw_status->frag_pkt.tlated_ip_addr;
	status->ip_cksum_diff = hw_status->frag_pkt.ip_cksum_diff;
	status->endp_src_idx = hw_status->frag_pkt.endp_src_idx;
	status->endp_dest_idx = hw_status->frag_pkt.endp_dest_idx;
	status->metadata = hw_status->frag_pkt.metadata;
	status->seq_num = hw_status->frag_pkt.seq_num;
	status->hdr_local = hw_status->frag_pkt.hdr_local;
	status->hdr_offset = hw_status->frag_pkt.hdr_offset;
	status->exception = hw_status->frag_pkt.exception;
	status->nat_type = hw_status->frag_pkt.nat_type;
}

static void __ipa_parse_gen_pkt_v5_0(struct ipahal_pkt_status *status,
				const void *unparsed_status)
{
	bool is_ipv6;
	union ipa_pkt_status_hw_v5_0 *hw_status =
		(union ipa_pkt_status_hw_v5_0 *)unparsed_status;

	is_ipv6 = (hw_status->ipa_pkt.status_mask & 0x80) ? false : true;
	status->pkt_len = hw_status->ipa_pkt.pkt_len;
	status->endp_src_idx = hw_status->ipa_pkt.endp_src_idx;
	status->endp_dest_idx = hw_status->ipa_pkt.endp_dest_idx;
	status->metadata = hw_status->ipa_pkt.metadata;
	status->flt_local = hw_status->ipa_pkt.flt_local;
	status->flt_hash = hw_status->ipa_pkt.flt_hash;
	status->flt_global = hw_status->ipa_pkt.flt_hash;
	status->flt_ret_hdr = hw_status->ipa_pkt.flt_ret_hdr;
	status->flt_miss = (hw_status->ipa_pkt.rt_rule_id ==
			IPAHAL_PKT_STATUS_FLTRT_RULE_MISS_ID);
	status->flt_rule_id = hw_status->ipa_pkt.flt_rule_id;
	status->rt_local = hw_status->ipa_pkt.rt_local;
	status->rt_hash = hw_status->ipa_pkt.rt_hash;
	status->ucp = hw_status->ipa_pkt.ucp;
	status->rt_tbl_idx = hw_status->ipa_pkt.rt_tbl_idx;
	status->rt_miss = (hw_status->ipa_pkt.rt_rule_id ==
			IPAHAL_PKT_STATUS_FLTRT_RULE_MISS_ID);
	status->rt_rule_id = hw_status->ipa_pkt.rt_rule_id;
	status->nat_hit = hw_status->ipa_pkt.nat_hit;
	status->nat_entry_idx = hw_status->ipa_pkt.nat_entry_idx;
	status->tag_info = hw_status->ipa_pkt.tag_info;
	status->seq_num = hw_status->ipa_pkt.seq_num;
	status->time_of_day_ctr = hw_status->ipa_pkt.time_of_day_ctr;
	status->hdr_local = hw_status->ipa_pkt.hdr_local;
	status->hdr_offset = hw_status->ipa_pkt.hdr_offset;
	status->frag_hit = hw_status->ipa_pkt.frag_hit;
	status->frag_rule = hw_status->ipa_pkt.frag_rule;
	status->nat_type = hw_status->ipa_pkt.nat_type;

	status->exception = pkt_status_parse_exception(is_ipv6,
			hw_status->ipa_pkt.exception);
}

static void __ipa_parse_frag_pkt_v5_0(struct ipahal_pkt_status *status,
				const void *unparsed_status)
{
	union ipa_pkt_status_hw_v5_0 *hw_status =
		(union ipa_pkt_status_hw_v5_0 *)unparsed_status;

	status->frag_rule_idx = hw_status->frag_pkt.frag_rule_idx;
	status->tbl_idx = hw_status->frag_pkt.tbl_idx;
	status->src_ip_addr = hw_status->frag_pkt.src_ip_addr;
	status->dest_ip_addr = hw_status->frag_pkt.dest_ip_addr;
	status->protocol = hw_status->frag_pkt.protocol;
	status->ip_id = hw_status->frag_pkt.ip_id;
	status->tlated_ip_addr = hw_status->frag_pkt.tlated_ip_addr;
	status->ip_cksum_diff = hw_status->frag_pkt.ip_cksum_diff;
	status->endp_src_idx = hw_status->frag_pkt.endp_src_idx;
	status->endp_dest_idx = hw_status->frag_pkt.endp_dest_idx;
	status->metadata = hw_status->frag_pkt.metadata;
	status->seq_num = hw_status->frag_pkt.seq_num;
	status->hdr_local = hw_status->frag_pkt.hdr_local;
	status->hdr_offset = hw_status->frag_pkt.hdr_offset;
	status->exception = hw_status->frag_pkt.exception;
	status->nat_type = hw_status->frag_pkt.nat_type;
}

static void __ipa_parse_gen_pkt_v5_5(struct ipahal_pkt_status *status,
				const void *unparsed_status)
{
	bool is_ipv6;
	union ipa_pkt_status_hw_v5_5 *hw_status =
		(union ipa_pkt_status_hw_v5_5 *)unparsed_status;

	is_ipv6 = (hw_status->ipa_pkt.status_mask & 0x80) ? false : true;
	status->pkt_len = hw_status->ipa_pkt.pkt_len;
	status->endp_src_idx = hw_status->ipa_pkt.endp_src_idx;
	status->endp_dest_idx = hw_status->ipa_pkt.endp_dest_idx;
	status->metadata = hw_status->ipa_pkt.metadata;
	status->flt_local = hw_status->ipa_pkt.flt_local;
	status->flt_hash = hw_status->ipa_pkt.flt_hash;
	status->flt_global = hw_status->ipa_pkt.flt_hash;
	status->flt_ret_hdr = hw_status->ipa_pkt.flt_ret_hdr;
	status->flt_miss = (hw_status->ipa_pkt.rt_rule_id ==
			IPAHAL_PKT_STATUS_FLTRT_RULE_MISS_ID);
	status->flt_rule_id = hw_status->ipa_pkt.flt_rule_id;
	status->rt_local = hw_status->ipa_pkt.rt_local;
	status->rt_hash = hw_status->ipa_pkt.rt_hash;
	status->ucp = hw_status->ipa_pkt.ucp;
	status->rt_tbl_idx = hw_status->ipa_pkt.rt_tbl_idx;
	status->rt_miss = (hw_status->ipa_pkt.rt_rule_id ==
			IPAHAL_PKT_STATUS_FLTRT_RULE_MISS_ID);
	status->rt_rule_id = hw_status->ipa_pkt.rt_rule_id;
	status->nat_hit = hw_status->ipa_pkt.nat_hit;
	status->nat_entry_idx = hw_status->ipa_pkt.nat_entry_idx;
	status->tag_info = hw_status->ipa_pkt.tag_info;
	status->egress_tc = hw_status->ipa_pkt.egress_tc;
	status->ingress_tc = hw_status->ipa_pkt.ingress_tc;
	status->seq_num = hw_status->ipa_pkt.seq_num;
	status->time_of_day_ctr = hw_status->ipa_pkt.time_of_day_ctr;
	status->hdr_local = hw_status->ipa_pkt.hdr_local;
	status->hdr_offset = hw_status->ipa_pkt.hdr_offset;
	status->frag_hit = hw_status->ipa_pkt.frag_hit;
	status->frag_rule = hw_status->ipa_pkt.frag_rule;
	status->nat_type = hw_status->ipa_pkt.nat_type;
	status->nat_exc_suppress = hw_status->ipa_pkt.nat_exc_suppress;
	status->tsp = hw_status->ipa_pkt.tsp;
	status->ttl_dec = hw_status->ipa_pkt.ttl_dec;

	status->exception = pkt_status_parse_exception(is_ipv6,
			hw_status->ipa_pkt.exception);
}


static void __ipa_parse_frag_pkt_v5_5(struct ipahal_pkt_status *status,
				const void *unparsed_status)
{
	union ipa_pkt_status_hw_v5_5 *hw_status =
		(union ipa_pkt_status_hw_v5_5 *)unparsed_status;

	status->frag_rule_idx = hw_status->frag_pkt.frag_rule_idx;
	status->tbl_idx = hw_status->frag_pkt.tbl_idx;
	status->src_ip_addr = hw_status->frag_pkt.src_ip_addr;
	status->dest_ip_addr = hw_status->frag_pkt.dest_ip_addr;
	status->protocol = hw_status->frag_pkt.protocol;
	status->ip_id = hw_status->frag_pkt.ip_id;
	status->tlated_ip_addr = hw_status->frag_pkt.tlated_ip_addr;
	status->ip_cksum_diff = hw_status->frag_pkt.ip_cksum_diff;
	status->endp_src_idx = hw_status->frag_pkt.endp_src_idx;
	status->endp_dest_idx = hw_status->frag_pkt.endp_dest_idx;
	status->metadata = hw_status->frag_pkt.metadata;
	status->seq_num = hw_status->frag_pkt.seq_num;
	status->hdr_local = hw_status->frag_pkt.hdr_local;
	status->hdr_offset = hw_status->frag_pkt.hdr_offset;
	status->exception = hw_status->frag_pkt.exception;
	status->nat_type = hw_status->frag_pkt.nat_type;
	status->hdr_ret = hw_status->frag_pkt.ret;
	status->ll = hw_status->frag_pkt.ll;
	status->ingress_tc = hw_status->frag_pkt.ingress_tc;
	status->egress_tc = hw_status->frag_pkt.egress_tc;
	status->pd = hw_status->frag_pkt.pd;
}


static void ipa_pkt_status_parse(
	const void *unparsed_status, struct ipahal_pkt_status *status);
static void ipa_pkt_status_parse_thin(const void *unparsed_status,
	struct ipahal_pkt_status_thin *status);
static void ipa_pkt_status_parse_thin_v5_0(const void *unparsed_status,
	struct ipahal_pkt_status_thin *status);
static void ipa_pkt_status_parse_thin_v5_5(const void *unparsed_status,
	struct ipahal_pkt_status_thin *status);
static void ipa_pkt_status_parse(
	const void *unparsed_status, struct ipahal_pkt_status *status);
static void ipa_pkt_status_parse_v5_0(
	const void *unparsed_status, struct ipahal_pkt_status *status);
static void ipa_pkt_status_parse_v5_5(
	const void *unparsed_status, struct ipahal_pkt_status *status);
/*
 * struct ipahal_pkt_status_obj - Pakcet Status H/W information for
 *  specific IPA version
 * @size: H/W size of the status packet
 * @parse: CB that parses the H/W packet status into the abstracted structure
 * @parse_thin: light weight CB that parses only some of the fields for
 * data path optimization
 */
struct ipahal_pkt_status_obj {
	u32 size;
	void (*parse)(const void *unparsed_status,
		struct ipahal_pkt_status *status);
	void (*parse_thin)(const void *unparsed_status,
			struct ipahal_pkt_status_thin *status);
	void (*__parse_gen_pkt)(struct ipahal_pkt_status *status,
				const void *unparsed_status);
	void (*__parse_frag_pkt)(struct ipahal_pkt_status *status,
				const void *unparsed_status);
};

/*
 * This table contains the info regard packet status for IPAv3 and later
 * Information like: size of packet status and parsing function
 * All the information on the pkt Status on IPAv3 are statically defined below.
 * If information is missing regard some IPA version, the init function
 *  will fill it with the information from the previous IPA version.
 * Information is considered missing if all of the fields are 0
 */
static struct ipahal_pkt_status_obj ipahal_pkt_status_objs[IPA_HW_MAX] = {
	/* IPAv3 */
	[IPA_HW_v3_0] = {
		IPA3_0_PKT_STATUS_SIZE,
		ipa_pkt_status_parse,
		ipa_pkt_status_parse_thin,
		__ipa_parse_gen_pkt,
		__ipa_parse_frag_pkt,
		},
	/* IPAv5 */
	[IPA_HW_v5_0] = {
		IPA3_0_PKT_STATUS_SIZE,
		ipa_pkt_status_parse_v5_0,
		ipa_pkt_status_parse_thin_v5_0,
		__ipa_parse_gen_pkt_v5_0,
		__ipa_parse_frag_pkt_v5_0,
		},
	/* IPAv5.5 */
	[IPA_HW_v5_5] = {
		IPA3_0_PKT_STATUS_SIZE,
		ipa_pkt_status_parse_v5_5,
		ipa_pkt_status_parse_thin_v5_5,
		__ipa_parse_gen_pkt_v5_5,
		__ipa_parse_frag_pkt_v5_5,
		},
};

static inline enum ipahal_pkt_status_opcode ipa_hw_opcode_to_opcode(
	const u8 hw_opcode)
{
	enum ipahal_pkt_status_opcode opcode = 0;

	switch (hw_opcode) {
	case 0x1:
		opcode = IPAHAL_PKT_STATUS_OPCODE_PACKET;
		break;
	case 0x2:
		opcode = IPAHAL_PKT_STATUS_OPCODE_NEW_FRAG_RULE;
		break;
	case 0x4:
		opcode = IPAHAL_PKT_STATUS_OPCODE_DROPPED_PACKET;
		break;
	case 0x8:
		opcode = IPAHAL_PKT_STATUS_OPCODE_SUSPENDED_PACKET;
		break;
	case 0x10:
		opcode = IPAHAL_PKT_STATUS_OPCODE_LOG;
		break;
	case 0x20:
		opcode = IPAHAL_PKT_STATUS_OPCODE_DCMP;
		break;
	case 0x40:
		opcode = IPAHAL_PKT_STATUS_OPCODE_PACKET_2ND_PASS;
		break;
	default:
		IPAHAL_ERR_RL("unsupported Status Opcode 0x%x\n", hw_opcode);
	}

	return opcode;
}

static inline enum ipahal_pkt_status_nat_type ipa_hw_nat_type_to_nat_type(
	const enum ipahal_pkt_status_nat_type hw_nat_type)
{
	enum ipahal_pkt_status_nat_type nat_type = IPAHAL_PKT_STATUS_NAT_NONE;

	switch (hw_nat_type) {
	case 0:
		nat_type = IPAHAL_PKT_STATUS_NAT_NONE;
		break;
	case 1:
		nat_type = IPAHAL_PKT_STATUS_NAT_SRC;
		break;
	case 2:
		nat_type = IPAHAL_PKT_STATUS_NAT_DST;
		break;
	default:
		IPAHAL_ERR_RL("unsupported Status NAT type 0x%x\n",hw_nat_type);
	}

	return nat_type;
}

static inline void ipa_set_pkt_status_mask(const u16 hw_status_mask,
	struct ipahal_pkt_status *status)
{
	IPA_PKT_STATUS_SET_MSK(0x1, IPAHAL_PKT_STATUS_MASK_FRAG_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x2, IPAHAL_PKT_STATUS_MASK_FILT_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x4, IPAHAL_PKT_STATUS_MASK_NAT_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x8, IPAHAL_PKT_STATUS_MASK_ROUTE_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x10, IPAHAL_PKT_STATUS_MASK_TAG_VALID_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x20, IPAHAL_PKT_STATUS_MASK_FRAGMENT_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x40,
		IPAHAL_PKT_STATUS_MASK_FIRST_FRAGMENT_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x80, IPAHAL_PKT_STATUS_MASK_V4_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x100,
		IPAHAL_PKT_STATUS_MASK_CKSUM_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x200, IPAHAL_PKT_STATUS_MASK_AGGR_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x400, IPAHAL_PKT_STATUS_MASK_DEST_EOT_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x800,
		IPAHAL_PKT_STATUS_MASK_DEAGGR_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x1000, IPAHAL_PKT_STATUS_MASK_DEAGG_FIRST_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x2000, IPAHAL_PKT_STATUS_MASK_SRC_EOT_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x4000, IPAHAL_PKT_STATUS_MASK_PREV_EOT_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x8000, IPAHAL_PKT_STATUS_MASK_BYTE_LIMIT_SHFT);
	status->status_mask &= 0xFFFF;
}

static inline void ipa_set_pkt_status_mask_v5_5(const u16 hw_status_mask,
	struct ipahal_pkt_status *status)
{
	IPA_PKT_STATUS_SET_MSK(0x1, IPAHAL_PKT_STATUS_MASK_FRAG_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x2, IPAHAL_PKT_STATUS_MASK_FILT_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x4, IPAHAL_PKT_STATUS_MASK_NAT_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x8, IPAHAL_PKT_STATUS_MASK_ROUTE_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x10, IPAHAL_PKT_STATUS_MASK_TAG_VALID_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x20, IPAHAL_PKT_STATUS_MASK_FRAGMENT_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x40,
		IPAHAL_PKT_STATUS_MASK_FIRST_FRAGMENT_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x80, IPAHAL_PKT_STATUS_MASK_V4_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x100,
		IPAHAL_PKT_STATUS_MASK_CKSUM_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x200, IPAHAL_PKT_STATUS_MASK_AGGR_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x400, IPAHAL_PKT_STATUS_MASK_OPENED_FRAME_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x800,
		IPAHAL_PKT_STATUS_MASK_DEAGGR_PROCESS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x1000, IPAHAL_PKT_STATUS_MASK_DEAGG_FIRST_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x2000, IPAHAL_PKT_STATUS_MASK_SRC_EOT_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x4000, IPAHAL_PKT_STATUS_MASK_RQOS_NAS_SHFT);
	IPA_PKT_STATUS_SET_MSK(0x8000, IPAHAL_PKT_STATUS_MASK_RQOS_AS_SHFT);
	status->status_mask &= 0xFFFF;
}


static void ipa_pkt_status_parse(
	const void *unparsed_status, struct ipahal_pkt_status *status)
{
	union ipa_pkt_status_hw *hw_status =
		(union ipa_pkt_status_hw *)unparsed_status;

	status->status_opcode =
		ipa_hw_opcode_to_opcode(hw_status->ipa_pkt.status_opcode);

	if (status->status_opcode == IPAHAL_PKT_STATUS_OPCODE_NEW_FRAG_RULE)
		ipahal_pkt_status_objs[ipahal_ctx->hw_type].\
			__parse_frag_pkt(status, unparsed_status);
	else
		ipahal_pkt_status_objs[ipahal_ctx->hw_type].\
			__parse_gen_pkt(status, unparsed_status);

	status->nat_type = ipa_hw_nat_type_to_nat_type(status->nat_type);

	ipa_set_pkt_status_mask((u16)(hw_status->ipa_pkt.status_mask), status);
}

static void ipa_pkt_status_parse_v5_0(
	const void *unparsed_status, struct ipahal_pkt_status *status)
{
	union ipa_pkt_status_hw_v5_0 *hw_status =
		(union ipa_pkt_status_hw_v5_0 *)unparsed_status;

	status->status_opcode =
		ipa_hw_opcode_to_opcode(hw_status->ipa_pkt.status_opcode);

	if (status->status_opcode == IPAHAL_PKT_STATUS_OPCODE_NEW_FRAG_RULE)
		ipahal_pkt_status_objs[ipahal_ctx->hw_type].\
			__parse_frag_pkt(status, unparsed_status);
	else
		ipahal_pkt_status_objs[ipahal_ctx->hw_type].\
			__parse_gen_pkt(status, unparsed_status);

	status->nat_type = ipa_hw_nat_type_to_nat_type(status->nat_type);

	ipa_set_pkt_status_mask((u16)(hw_status->ipa_pkt.status_mask), status);
}

static void ipa_pkt_status_parse_v5_5(
	const void *unparsed_status, struct ipahal_pkt_status *status)
{
	union ipa_pkt_status_hw_v5_5 *hw_status =
		(union ipa_pkt_status_hw_v5_5 *)unparsed_status;

	status->status_opcode =
		ipa_hw_opcode_to_opcode(hw_status->ipa_pkt.status_opcode);

	if (status->status_opcode == IPAHAL_PKT_STATUS_OPCODE_NEW_FRAG_RULE)
		ipahal_pkt_status_objs[ipahal_ctx->hw_type].\
			__parse_frag_pkt(status, unparsed_status);
	else
		ipahal_pkt_status_objs[ipahal_ctx->hw_type].\
			__parse_gen_pkt(status, unparsed_status);

	status->nat_type = ipa_hw_nat_type_to_nat_type(status->nat_type);

	ipa_set_pkt_status_mask_v5_5((u16)(hw_status->ipa_pkt.status_mask), status);
}

/*
 * ipa_pkt_status_parse_thin() - Parse some of the packet status fields
 * for specific usage in the LAN rx data path where parsing needs to be done
 * but only for specific fields.
 * @unparsed_status: Pointer to H/W format of the packet status as read from HW
 * @status: Pointer to pre-allocated buffer where the parsed info will be
 * stored
 */
static void ipa_pkt_status_parse_thin(const void *unparsed_status,
	struct ipahal_pkt_status_thin *status)
{
	union ipa_pkt_status_hw *hw_status =
		(union ipa_pkt_status_hw *)unparsed_status;
	bool is_ipv6 = (hw_status->ipa_pkt.status_mask & 0x80) ? false : true;

	IPAHAL_DBG_LOW("Parse Thin Status Packet\n");
	status->metadata = hw_status->ipa_pkt.metadata;
	status->endp_src_idx = hw_status->ipa_pkt.endp_src_idx;
	status->ucp = hw_status->ipa_pkt.ucp;
	status->exception = pkt_status_parse_exception(is_ipv6,
						hw_status->ipa_pkt.exception);
}

/*
 * ipa_pkt_status_parse_thin_v5_0() - Parse some of the v5.0 packet status
 * fields for specific usage in the LAN rx data path where parsing needs
 * to be done but only for specific fields.
 * @unparsed_status: Pointer to H/W format of the packet status as read from HW
 * @status: Pointer to pre-allocated buffer where the parsed info will be
 * stored
 */
static void ipa_pkt_status_parse_thin_v5_0(const void *unparsed_status,
	struct ipahal_pkt_status_thin *status)
{
	union ipa_pkt_status_hw_v5_0 *hw_status =
		(union ipa_pkt_status_hw_v5_0 *)unparsed_status;
	bool is_ipv6 =
		(hw_status->ipa_pkt.status_mask & 0x80) ? false : true;

	IPAHAL_DBG_LOW("Parse Thin Status Packet\n");
	status->metadata = hw_status->ipa_pkt.metadata;
	status->endp_src_idx = hw_status->ipa_pkt.endp_src_idx;
	status->ucp = hw_status->ipa_pkt.ucp;
	status->exception = pkt_status_parse_exception(is_ipv6,
					hw_status->ipa_pkt.exception);
}

/*
 * ipa_pkt_status_parse_thin_v5_5() - Parse some of the v5.5 packet status
 * fields for specific usage in the LAN rx data path where parsing needs
 * to be done but only for specific fields.
 * @unparsed_status: Pointer to H/W format of the packet status as read from HW
 * @status: Pointer to pre-allocated buffer where the parsed info will be
 * stored
 */
static void ipa_pkt_status_parse_thin_v5_5(const void *unparsed_status,
	struct ipahal_pkt_status_thin *status)
{
	union ipa_pkt_status_hw_v5_5 *hw_status =
		(union ipa_pkt_status_hw_v5_5 *)unparsed_status;
	bool is_ipv6 =
		(hw_status->ipa_pkt.status_mask & 0x80) ? false : true;

	IPAHAL_DBG_LOW("Parse Thin Status Packet\n");
	status->metadata = hw_status->ipa_pkt.metadata;
	status->endp_src_idx = hw_status->ipa_pkt.endp_src_idx;
	status->ucp = hw_status->ipa_pkt.ucp;
	status->exception = pkt_status_parse_exception(is_ipv6,
					hw_status->ipa_pkt.exception);
}

/*
 * ipahal_pkt_status_init() - Build the packet status information array
 *  for the different IPA versions
 *  See ipahal_pkt_status_objs[] comments
 */
static int ipahal_pkt_status_init(enum ipa_hw_type ipa_hw_type)
{
	int i;
	struct ipahal_pkt_status_obj zero_obj;

	IPAHAL_DBG_LOW("Entry - HW_TYPE=%d\n", ipa_hw_type);

	if ((ipa_hw_type < 0) || (ipa_hw_type >= IPA_HW_MAX)) {
		IPAHAL_ERR("invalid IPA HW type (%d)\n", ipa_hw_type);
		return -EINVAL;
	}

	/*
	 * Since structure alignment is implementation dependent,
	 * add test to avoid different and incompatible data layouts.
	 * If test fails it also means that ipahal_pkt_status_parse_thin
	 * need to be checked.
	 *
	 * In case new H/W has different size or structure of status packet,
	 * add a compile time validty check for it like below (as well as
	 * the new defines and/or the new strucutre in the internal header).
	 */
	BUILD_BUG_ON(sizeof(union ipa_pkt_status_hw) !=
		IPA3_0_PKT_STATUS_SIZE);

	memset(&zero_obj, 0, sizeof(zero_obj));
	for (i = IPA_HW_v3_0 ; i < ipa_hw_type ; i++) {
		if (!memcmp(&ipahal_pkt_status_objs[i+1], &zero_obj,
			sizeof(struct ipahal_pkt_status_obj))) {
			memcpy(&ipahal_pkt_status_objs[i+1],
				&ipahal_pkt_status_objs[i],
				sizeof(struct ipahal_pkt_status_obj));
		} else {
			/*
			 * explicitly overridden Packet Status info
			 * Check validity
			 */
			if (!ipahal_pkt_status_objs[i+1].size) {
				IPAHAL_ERR(
				  "Packet Status with zero size ipa_ver=%d\n",
				  i+1);
				WARN_ON(1);
			}
			if (!ipahal_pkt_status_objs[i+1].parse) {
				IPAHAL_ERR(
				  "Packet Status without Parse func ipa_ver=%d\n",
				  i+1);
				WARN_ON(1);
			}
			if (!ipahal_pkt_status_objs[i+1].parse_thin) {
				IPAHAL_ERR(
				  "Packet Status without Parse_thin func ipa_ver=%d\n",
				  i+1);
				WARN_ON(1);
			}
			if (!ipahal_pkt_status_objs[i+1].__parse_gen_pkt) {
				IPAHAL_ERR(
				  "Packet Status without parse_gen func ipa_ver=%d\n",
				  i+1);
				WARN_ON(1);
			}
			if (!ipahal_pkt_status_objs[i+1].__parse_frag_pkt) {
				IPAHAL_ERR(
				  "Packet Status without parse_frag func ipa_ver=%d\n",
				  i+1);
				WARN_ON(1);
			}
		}
	}

	return 0;
}

/*
 * ipahal_pkt_status_get_size() - Get H/W size of packet status
 */
u32 ipahal_pkt_status_get_size(void)
{
	return ipahal_pkt_status_objs[ipahal_ctx->hw_type].size;
}

/*
 * ipahal_pkt_status_parse() - Parse Packet Status payload to abstracted form
 * @unparsed_status: Pointer to H/W format of the packet status as read from H/W
 * @status: Pointer to pre-allocated buffer where the parsed info will be stored
 */
void ipahal_pkt_status_parse(const void *unparsed_status,
	struct ipahal_pkt_status *status)
{
	if (!unparsed_status || !status) {
		IPAHAL_ERR("Input Error: unparsed_status=%pK status=%pK\n",
			unparsed_status, status);
		return;
	}

	IPAHAL_DBG_LOW("Parse Status Packet\n");
	memset(status, 0, sizeof(*status));
	ipahal_pkt_status_objs[ipahal_ctx->hw_type].parse(unparsed_status,
		status);
}

/*
 * ipahal_pkt_status_parse_thin() - Similar to ipahal_pkt_status_parse,
 * the difference is it only parses some of the status packet fields
 * used for TP optimization.
 * @unparsed_status: Pointer to H/W format of the packet status as read from H/W
 * @status: Pointer to pre-allocated buffer where the parsed info will be stored
 */
void ipahal_pkt_status_parse_thin(const void *unparsed_status,
	struct ipahal_pkt_status_thin *status)
{
	if (!unparsed_status || !status) {
		IPAHAL_ERR("Input Error: unparsed_status=%pK status=%pK\n",
			unparsed_status, status);
		return;
	}
	IPAHAL_DBG_LOW("Parse_thin Status Packet\n");
	ipahal_pkt_status_objs[ipahal_ctx->hw_type].parse_thin(unparsed_status,
				status);
}

/*
 * ipahal_pkt_status_exception_str() - returns string represents exception type
 * @exception: [in] The exception type
 */
const char *ipahal_pkt_status_exception_str(
	enum ipahal_pkt_status_exception exception)
{
	if (exception < 0 || exception >= IPAHAL_PKT_STATUS_EXCEPTION_MAX) {
		IPAHAL_ERR(
			"requested string of invalid pkt_status exception=%d\n",
			exception);
		return "Invalid PKT_STATUS_EXCEPTION";
	}

	return ipahal_pkt_status_exception_to_str[exception];
}

#ifdef CONFIG_DEBUG_FS
static void ipahal_debugfs_init(void)
{
	ipahal_ctx->dent = debugfs_create_dir("ipahal", 0);
	if (!ipahal_ctx->dent || IS_ERR(ipahal_ctx->dent)) {
		IPAHAL_ERR("fail to create ipahal debugfs folder\n");
		goto fail;
	}

	return;
fail:
	debugfs_remove_recursive(ipahal_ctx->dent);
	ipahal_ctx->dent = NULL;
}

static void ipahal_debugfs_remove(void)
{
	if (!ipahal_ctx)
		return;

	if (IS_ERR(ipahal_ctx->dent)) {
		IPAHAL_ERR("ipahal debugfs folder was not created\n");
		return;
	}

	debugfs_remove_recursive(ipahal_ctx->dent);
}
#else /* CONFIG_DEBUG_FS */
static void ipahal_debugfs_init(void) {}
static void ipahal_debugfs_remove(void) {}
#endif /* CONFIG_DEBUG_FS */

/*
 * ipahal_cp_hdr_to_hw_buff_v3() - copy header to hardware buffer according to
 * base address and offset given.
 * @base: dma base address
 * @offset: offset from base address where the data will be copied
 * @hdr: the header to be copied
 * @hdr_len: the length of the header
 */
static void ipahal_cp_hdr_to_hw_buff_v3(void *const base, u32 offset,
		u8 *const hdr, u32 hdr_len)
{
	memcpy(base + offset, hdr, hdr_len);
}

/* Header address update logic. */
#define IPAHAL_CP_PROC_CTX_HEADER_UPDATE(hdr_lsb, hdr_msb, addr) \
	do { \
		hdr_lsb = lower_32_bits(addr); \
		hdr_msb = upper_32_bits(addr); \
	} while (0)

/*
 * ipahal_cp_proc_ctx_to_hw_buff_v3() - copy processing context to
 * base address and offset given.
 * @type: header processing context type (no processing context,
 *	IPA_HDR_PROC_ETHII_TO_ETHII etc.)
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
static int ipahal_cp_proc_ctx_to_hw_buff_v3(enum ipa_hdr_proc_type type,
		void *const base, u32 offset,
		u32 hdr_len, u64 hdr_base_addr,
		struct ipa_hdr_offset_entry *offset_entry,
		struct ipa_l2tp_hdr_proc_ctx_params *l2tp_params,
		struct ipa_eogre_hdr_proc_ctx_params *eogre_params,
		struct ipa_eth_II_to_eth_II_ex_procparams *generic_params,
		bool is_64)
{
	u64 hdr_addr;

	if (type == IPA_HDR_PROC_NONE) {
		struct ipa_hw_hdr_proc_ctx_add_hdr_seq *ctx;

		ctx = (struct ipa_hw_hdr_proc_ctx_add_hdr_seq *)
			(base + offset);
		ctx->hdr_add.tlv.type = IPA_PROC_CTX_TLV_TYPE_HDR_ADD;
		ctx->hdr_add.tlv.length = 2;
		ctx->hdr_add.tlv.value = hdr_len;
		hdr_addr = hdr_base_addr + offset_entry->offset;
		IPAHAL_DBG("header address 0x%llx\n",
			hdr_addr);
		IPAHAL_CP_PROC_CTX_HEADER_UPDATE(ctx->hdr_add.hdr_addr,
			ctx->hdr_add.hdr_addr_hi, hdr_addr);
		if (!is_64)
			ctx->hdr_add.hdr_addr_hi = 0;
		ctx->end.type = IPA_PROC_CTX_TLV_TYPE_END;
		ctx->end.length = 0;
		ctx->end.value = 0;
	} else if ((type == IPA_HDR_PROC_L2TP_HEADER_ADD) ||
		(type == IPA_HDR_PROC_L2TP_UDP_HEADER_ADD)) {
		struct ipa_hw_hdr_proc_ctx_add_l2tp_hdr_cmd_seq *ctx;

		ctx = (struct ipa_hw_hdr_proc_ctx_add_l2tp_hdr_cmd_seq *)
			(base + offset);
		ctx->hdr_add.tlv.type = IPA_PROC_CTX_TLV_TYPE_HDR_ADD;
		ctx->hdr_add.tlv.length = 2;
		ctx->hdr_add.tlv.value = hdr_len;
		hdr_addr = hdr_base_addr + offset_entry->offset;
		IPAHAL_DBG("header address 0x%llx\n",
			hdr_addr);
		IPAHAL_CP_PROC_CTX_HEADER_UPDATE(ctx->hdr_add.hdr_addr,
			ctx->hdr_add.hdr_addr_hi, hdr_addr);
		if (!is_64)
			ctx->hdr_add.hdr_addr_hi = 0;
		ctx->l2tp_params.tlv.type = IPA_PROC_CTX_TLV_TYPE_PROC_CMD;
		ctx->l2tp_params.tlv.length = 1;
		if (type == IPA_HDR_PROC_L2TP_HEADER_ADD)
			ctx->l2tp_params.tlv.value =
					IPA_HDR_UCP_L2TP_HEADER_ADD;
		else
			ctx->l2tp_params.tlv.value =
					IPA_HDR_UCP_L2TP_UDP_HEADER_ADD;
		ctx->l2tp_params.l2tp_params.second_pass =
			l2tp_params->hdr_add_param.second_pass;
		ctx->l2tp_params.l2tp_params.eth_hdr_retained =
			l2tp_params->hdr_add_param.eth_hdr_retained;
		ctx->l2tp_params.l2tp_params.input_ip_version =
			l2tp_params->hdr_add_param.input_ip_version;
		ctx->l2tp_params.l2tp_params.output_ip_version =
			l2tp_params->hdr_add_param.output_ip_version;

		IPAHAL_DBG("command id %d\n", ctx->l2tp_params.tlv.value);
		ctx->end.type = IPA_PROC_CTX_TLV_TYPE_END;
		ctx->end.length = 0;
		ctx->end.value = 0;
	} else if (type == IPA_HDR_PROC_L2TP_HEADER_REMOVE) {
		struct ipa_hw_hdr_proc_ctx_remove_l2tp_hdr_cmd_seq *ctx;

		ctx = (struct ipa_hw_hdr_proc_ctx_remove_l2tp_hdr_cmd_seq *)
			(base + offset);
		ctx->hdr_add.tlv.type = IPA_PROC_CTX_TLV_TYPE_HDR_ADD;
		ctx->hdr_add.tlv.length = 2;
		ctx->hdr_add.tlv.value = hdr_len;
		hdr_addr = hdr_base_addr + offset_entry->offset;
		IPAHAL_DBG("header address 0x%llx length %d\n",
			hdr_addr, ctx->hdr_add.tlv.value);
		IPAHAL_CP_PROC_CTX_HEADER_UPDATE(ctx->hdr_add.hdr_addr,
			ctx->hdr_add.hdr_addr_hi, hdr_addr);
		if (!is_64)
			ctx->hdr_add.hdr_addr_hi = 0;
		ctx->l2tp_params.tlv.type = IPA_PROC_CTX_TLV_TYPE_PROC_CMD;
		ctx->l2tp_params.tlv.length = 1;
		ctx->l2tp_params.tlv.value =
					IPA_HDR_UCP_L2TP_HEADER_REMOVE;
		ctx->l2tp_params.l2tp_params.hdr_len_remove =
			l2tp_params->hdr_remove_param.hdr_len_remove;
		ctx->l2tp_params.l2tp_params.eth_hdr_retained =
			l2tp_params->hdr_remove_param.eth_hdr_retained;
		ctx->l2tp_params.l2tp_params.hdr_ofst_pkt_size_valid =
			l2tp_params->hdr_remove_param.hdr_ofst_pkt_size_valid;
		ctx->l2tp_params.l2tp_params.hdr_ofst_pkt_size =
			l2tp_params->hdr_remove_param.hdr_ofst_pkt_size;
		ctx->l2tp_params.l2tp_params.hdr_endianness =
			l2tp_params->hdr_remove_param.hdr_endianness;
		IPAHAL_DBG("hdr ofst valid: %d, hdr ofst pkt size: %d\n",
			ctx->l2tp_params.l2tp_params.hdr_ofst_pkt_size_valid,
			ctx->l2tp_params.l2tp_params.hdr_ofst_pkt_size);
		IPAHAL_DBG("endianness: %d\n",
			ctx->l2tp_params.l2tp_params.hdr_endianness);

		IPAHAL_DBG("command id %d\n", ctx->l2tp_params.tlv.value);
		ctx->end.type = IPA_PROC_CTX_TLV_TYPE_END;
		ctx->end.length = 0;
		ctx->end.value = 0;
	} else if (type == IPA_HDR_PROC_L2TP_UDP_HEADER_REMOVE) {
		struct ipa_hw_hdr_proc_ctx_remove_l2tp_hdr_cmd_seq *ctx;

		ctx = (struct ipa_hw_hdr_proc_ctx_remove_l2tp_hdr_cmd_seq *)
			(base + offset);
		ctx->hdr_add.tlv.type = IPA_PROC_CTX_TLV_TYPE_HDR_ADD;
		ctx->hdr_add.tlv.length = 2;
		if (l2tp_params->hdr_remove_param.eth_hdr_retained) {
			ctx->hdr_add.tlv.value = hdr_len;
			hdr_addr = hdr_base_addr + offset_entry->offset;
			IPAHAL_DBG("header address 0x%llx length %d\n",
				hdr_addr, ctx->hdr_add.tlv.value);
			IPAHAL_CP_PROC_CTX_HEADER_UPDATE(ctx->hdr_add.hdr_addr,
				ctx->hdr_add.hdr_addr_hi, hdr_addr);
			if (!is_64)
				ctx->hdr_add.hdr_addr_hi = 0;
		} else {
			ctx->hdr_add.tlv.value = 0;
		}
		ctx->l2tp_params.tlv.type = IPA_PROC_CTX_TLV_TYPE_PROC_CMD;
		ctx->l2tp_params.tlv.length = 1;
		ctx->l2tp_params.tlv.value =
				IPA_HDR_UCP_L2TP_UDP_HEADER_REMOVE;
		ctx->l2tp_params.l2tp_params.hdr_len_remove =
			l2tp_params->hdr_remove_param.hdr_len_remove;
		ctx->l2tp_params.l2tp_params.eth_hdr_retained =
			l2tp_params->hdr_remove_param.eth_hdr_retained;
		ctx->l2tp_params.l2tp_params.hdr_ofst_pkt_size_valid =
			l2tp_params->hdr_remove_param.hdr_ofst_pkt_size_valid;
		ctx->l2tp_params.l2tp_params.hdr_ofst_pkt_size =
			l2tp_params->hdr_remove_param.hdr_ofst_pkt_size;
		ctx->l2tp_params.l2tp_params.hdr_endianness =
			l2tp_params->hdr_remove_param.hdr_endianness;
		IPAHAL_DBG("hdr ofst valid: %d, hdr ofst pkt size: %d\n",
			ctx->l2tp_params.l2tp_params.hdr_ofst_pkt_size_valid,
			ctx->l2tp_params.l2tp_params.hdr_ofst_pkt_size);
		IPAHAL_DBG("endianness: %d\n",
			ctx->l2tp_params.l2tp_params.hdr_endianness);

		IPAHAL_DBG("command id %d\n", ctx->l2tp_params.tlv.value);
		ctx->end.type = IPA_PROC_CTX_TLV_TYPE_END;
		ctx->end.length = 0;
		ctx->end.value = 0;
	} else if (type == IPA_HDR_PROC_ETHII_TO_ETHII_EX) {
		struct ipa_hw_hdr_proc_ctx_add_hdr_cmd_seq_ex *ctx;

		ctx = (struct ipa_hw_hdr_proc_ctx_add_hdr_cmd_seq_ex *)
			(base + offset);

		ctx->hdr_add.tlv.type = IPA_PROC_CTX_TLV_TYPE_HDR_ADD;
		ctx->hdr_add.tlv.length = 2;
		ctx->hdr_add.tlv.value = hdr_len;
		hdr_addr = hdr_base_addr + offset_entry->offset;
		IPAHAL_DBG("header address 0x%x\n",
			ctx->hdr_add.hdr_addr);
		IPAHAL_CP_PROC_CTX_HEADER_UPDATE(ctx->hdr_add.hdr_addr,
			ctx->hdr_add.hdr_addr_hi, hdr_addr);
		if (!is_64)
			ctx->hdr_add.hdr_addr_hi = 0;

		ctx->hdr_add_ex.tlv.type = IPA_PROC_CTX_TLV_TYPE_PROC_CMD;
		ctx->hdr_add_ex.tlv.length = 1;
		ctx->hdr_add_ex.tlv.value = IPA_HDR_UCP_ETHII_TO_ETHII_EX;

		ctx->hdr_add_ex.params.input_ethhdr_negative_offset =
			generic_params->input_ethhdr_negative_offset;
		ctx->hdr_add_ex.params.output_ethhdr_negative_offset =
			generic_params->output_ethhdr_negative_offset;
		ctx->hdr_add_ex.params.reserved = 0;

		ctx->end.type = IPA_PROC_CTX_TLV_TYPE_END;
		ctx->end.length = 0;
		ctx->end.value = 0;
	} else if (type == IPA_HDR_PROC_EoGRE_HEADER_ADD) {
		struct ipa_hw_hdr_proc_ctx_add_eogre_hdr_cmd_seq *ctx =
			(struct ipa_hw_hdr_proc_ctx_add_eogre_hdr_cmd_seq *)
			(base + offset);

		ctx->hdr_add.tlv.type = IPA_PROC_CTX_TLV_TYPE_HDR_ADD;
		ctx->hdr_add.tlv.length = 2;
		ctx->hdr_add.tlv.value = hdr_len;
		hdr_addr = hdr_base_addr + offset_entry->offset;
		IPAHAL_DBG("header address 0x%llx\n",
			hdr_addr);
		IPAHAL_CP_PROC_CTX_HEADER_UPDATE(ctx->hdr_add.hdr_addr,
			ctx->hdr_add.hdr_addr_hi, hdr_addr);
		if (!is_64)
			ctx->hdr_add.hdr_addr_hi = 0;
		ctx->eogre_params.tlv.type = IPA_PROC_CTX_TLV_TYPE_PROC_CMD;
		ctx->eogre_params.tlv.length = 1;
		ctx->eogre_params.tlv.value = IPA_HDR_UCP_EoGRE_HEADER_ADD;
		ctx->eogre_params.eogre_params.eth_hdr_retained =
			eogre_params->hdr_add_param.eth_hdr_retained;
		ctx->eogre_params.eogre_params.input_ip_version =
			eogre_params->hdr_add_param.input_ip_version;
		ctx->eogre_params.eogre_params.output_ip_version =
			eogre_params->hdr_add_param.output_ip_version;
		ctx->eogre_params.eogre_params.second_pass =
			eogre_params->hdr_add_param.second_pass;
		IPAHAL_DBG("command id %d\n", ctx->eogre_params.tlv.value);
		IPAHAL_DBG("eth_hdr_retained %d input_ip_version %d output_ip_version %d second_pass %d\n",
			eogre_params->hdr_add_param.eth_hdr_retained,
			eogre_params->hdr_add_param.input_ip_version,
			eogre_params->hdr_add_param.output_ip_version,
			eogre_params->hdr_add_param.second_pass);
		ctx->end.type = IPA_PROC_CTX_TLV_TYPE_END;
		ctx->end.length = 0;
		ctx->end.value = 0;
	} else if (type == IPA_HDR_PROC_EoGRE_HEADER_REMOVE) {
		struct ipa_hw_hdr_proc_ctx_remove_eogre_hdr_cmd_seq *ctx =
			(struct ipa_hw_hdr_proc_ctx_remove_eogre_hdr_cmd_seq *)
			(base + offset);

		ctx->hdr_add.tlv.type = IPA_PROC_CTX_TLV_TYPE_HDR_ADD;
		ctx->hdr_add.tlv.length = 2;
		ctx->hdr_add.tlv.value = hdr_len;
		hdr_addr = hdr_base_addr + offset_entry->offset;
		IPAHAL_DBG("header address 0x%llx length %d\n",
				   hdr_addr, ctx->hdr_add.tlv.value);
		IPAHAL_CP_PROC_CTX_HEADER_UPDATE(
			ctx->hdr_add.hdr_addr,
			ctx->hdr_add.hdr_addr_hi, hdr_addr);
		if (!is_64)
			ctx->hdr_add.hdr_addr_hi = 0;
		ctx->eogre_params.tlv.type = IPA_PROC_CTX_TLV_TYPE_PROC_CMD;
		ctx->eogre_params.tlv.length = 1;
		ctx->eogre_params.tlv.value = IPA_HDR_UCP_EoGRE_HEADER_REMOVE;
		ctx->eogre_params.eogre_params.hdr_len_remove =
			eogre_params->hdr_remove_param.hdr_len_remove;
		ctx->end.type = IPA_PROC_CTX_TLV_TYPE_END;
		ctx->end.length = 0;
		ctx->end.value = 0;
	} else {
		struct ipa_hw_hdr_proc_ctx_add_hdr_cmd_seq *ctx;

		ctx = (struct ipa_hw_hdr_proc_ctx_add_hdr_cmd_seq *)
			(base + offset);
		ctx->hdr_add.tlv.type = IPA_PROC_CTX_TLV_TYPE_HDR_ADD;
		ctx->hdr_add.tlv.length = 2;
		ctx->hdr_add.tlv.value = hdr_len;
		hdr_addr = hdr_base_addr + offset_entry->offset;
		IPAHAL_DBG("header address 0x%llx\n",
			hdr_addr);
		IPAHAL_CP_PROC_CTX_HEADER_UPDATE(ctx->hdr_add.hdr_addr,
			ctx->hdr_add.hdr_addr_hi, hdr_addr);
		if (!is_64)
			ctx->hdr_add.hdr_addr_hi = 0;
		ctx->cmd.type = IPA_PROC_CTX_TLV_TYPE_PROC_CMD;
		ctx->cmd.length = 0;
		switch (type) {
		case IPA_HDR_PROC_ETHII_TO_ETHII:
			ctx->cmd.value = IPA_HDR_UCP_ETHII_TO_ETHII;
			break;
		case IPA_HDR_PROC_ETHII_TO_802_3:
			ctx->cmd.value = IPA_HDR_UCP_ETHII_TO_802_3;
			break;
		case IPA_HDR_PROC_802_3_TO_ETHII:
			ctx->cmd.value = IPA_HDR_UCP_802_3_TO_ETHII;
			break;
		case IPA_HDR_PROC_802_3_TO_802_3:
			ctx->cmd.value = IPA_HDR_UCP_802_3_TO_802_3;
			break;
		case IPA_HDR_PROC_SET_DSCP:
			ctx->cmd.value = IPA_HDR_UCP_SET_DSCP;
			break;
		default:
			IPAHAL_ERR("unknown ipa_hdr_proc_type %d", type);
			WARN_ON(1);
			return -EINVAL;
		}
		IPAHAL_DBG("command id %d\n", ctx->cmd.value);
		ctx->end.type = IPA_PROC_CTX_TLV_TYPE_END;
		ctx->end.length = 0;
		ctx->end.value = 0;
	}

	return 0;
}

/*
 * ipahal_get_proc_ctx_needed_len_v3() - calculates the needed length for
 * addition of header processing context according to the type of processing
 * context.
 * @type: header processing context type (no processing context,
 *	IPA_HDR_PROC_ETHII_TO_ETHII etc.)
 */
static int ipahal_get_proc_ctx_needed_len_v3(enum ipa_hdr_proc_type type)
{
	int ret;

	switch (type) {
	case IPA_HDR_PROC_NONE:
		ret = sizeof(struct ipa_hw_hdr_proc_ctx_add_hdr_seq);
		break;
	case IPA_HDR_PROC_ETHII_TO_ETHII:
	case IPA_HDR_PROC_ETHII_TO_802_3:
	case IPA_HDR_PROC_802_3_TO_ETHII:
	case IPA_HDR_PROC_802_3_TO_802_3:
		ret = sizeof(struct ipa_hw_hdr_proc_ctx_add_hdr_cmd_seq);
		break;
	case IPA_HDR_PROC_L2TP_HEADER_ADD:
		ret = sizeof(struct ipa_hw_hdr_proc_ctx_add_l2tp_hdr_cmd_seq);
		break;
	case IPA_HDR_PROC_L2TP_HEADER_REMOVE:
		ret =
		sizeof(struct ipa_hw_hdr_proc_ctx_remove_l2tp_hdr_cmd_seq);
		break;
	case IPA_HDR_PROC_L2TP_UDP_HEADER_ADD:
		ret = sizeof(struct ipa_hw_hdr_proc_ctx_add_l2tp_hdr_cmd_seq);
		break;
	case IPA_HDR_PROC_L2TP_UDP_HEADER_REMOVE:
		ret =
		sizeof(struct ipa_hw_hdr_proc_ctx_remove_l2tp_hdr_cmd_seq);
		break;
	case IPA_HDR_PROC_ETHII_TO_ETHII_EX:
		ret = sizeof(struct ipa_hw_hdr_proc_ctx_add_hdr_cmd_seq_ex);
		break;
	case IPA_HDR_PROC_EoGRE_HEADER_ADD:
		ret = sizeof(struct ipa_hw_hdr_proc_ctx_add_eogre_hdr_cmd_seq);
		break;
	case IPA_HDR_PROC_EoGRE_HEADER_REMOVE:
		ret =
		sizeof(struct ipa_hw_hdr_proc_ctx_remove_eogre_hdr_cmd_seq);
		break;
	default:
		/* invalid value to make sure failure */
		IPAHAL_ERR_RL("invalid ipa_hdr_proc_type %d\n", type);
		ret = -1;
	}

	return ret;
}

/*
 * struct ipahal_hdr_funcs - headers handling functions for specific IPA
 * version
 * @ipahal_cp_hdr_to_hw_buff - copy function for regular headers
 */
struct ipahal_hdr_funcs {
	void (*ipahal_cp_hdr_to_hw_buff)(void *const base, u32 offset,
			u8 *const hdr, u32 hdr_len);

	int (*ipahal_cp_proc_ctx_to_hw_buff)(enum ipa_hdr_proc_type type,
			void *const base, u32 offset, u32 hdr_len,
			u64 hdr_base_addr,
			struct ipa_hdr_offset_entry *offset_entry,
			struct ipa_l2tp_hdr_proc_ctx_params *l2tp_params,
			struct ipa_eogre_hdr_proc_ctx_params *eogre_params,
			struct ipa_eth_II_to_eth_II_ex_procparams
			*generic_params,
			bool is_64);

	int (*ipahal_get_proc_ctx_needed_len)(enum ipa_hdr_proc_type type);
};

static struct ipahal_hdr_funcs hdr_funcs;

static void ipahal_hdr_init(enum ipa_hw_type ipa_hw_type)
{

	IPAHAL_DBG("Entry - HW_TYPE=%d\n", ipa_hw_type);

	/*
	 * once there are changes in HW and need to use different case, insert
	 * new case for the new h/w. put the default always for the latest HW
	 * and make sure all previous supported versions have their cases.
	 */
	switch (ipa_hw_type) {
	case IPA_HW_v3_0:
	default:
		hdr_funcs.ipahal_cp_hdr_to_hw_buff =
				ipahal_cp_hdr_to_hw_buff_v3;
		hdr_funcs.ipahal_cp_proc_ctx_to_hw_buff =
				ipahal_cp_proc_ctx_to_hw_buff_v3;
		hdr_funcs.ipahal_get_proc_ctx_needed_len =
				ipahal_get_proc_ctx_needed_len_v3;
	}
	IPAHAL_DBG("Exit\n");
}

/*
 * ipahal_cp_hdr_to_hw_buff() - copy header to hardware buffer according to
 * base address and offset given.
 * @base: dma base address
 * @offset: offset from base address where the data will be copied
 * @hdr: the header to be copied
 * @hdr_len: the length of the header
 */
void ipahal_cp_hdr_to_hw_buff(void *base, u32 offset, u8 *const hdr,
		u32 hdr_len)
{
	IPAHAL_DBG_LOW("Entry\n");
	IPAHAL_DBG("base %pK, offset %d, hdr %pK, hdr_len %d\n", base,
			offset, hdr, hdr_len);
	if (!base || !hdr) {
		IPAHAL_ERR("failed on validating params\n");
		return;
	}

	hdr_funcs.ipahal_cp_hdr_to_hw_buff(base, offset, hdr, hdr_len);

	IPAHAL_DBG_LOW("Exit\n");
}

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
		void *const base, u32 offset, u32 hdr_len,
		u64 hdr_base_addr, struct ipa_hdr_offset_entry *offset_entry,
		struct ipa_l2tp_hdr_proc_ctx_params *l2tp_params,
		struct ipa_eogre_hdr_proc_ctx_params *eogre_params,
		struct ipa_eth_II_to_eth_II_ex_procparams *generic_params,
		bool is_64)
{
	IPAHAL_DBG(
		"type %d, base %pK, offset %d, hdr_len %d, hdr_base_addr %llu, offset_entry %pK, bool %d\n"
			, type, base, offset, hdr_len, hdr_base_addr, offset_entry, is_64);

	if (!base || !offset_entry || !hdr_base_addr) {
		IPAHAL_ERR(
			"invalid input: hdr_len:%u hdr_base_addr:%llu offset_entry:%pK\n",
			hdr_len, hdr_base_addr, offset_entry);
		return -EINVAL;
	}

	return hdr_funcs.ipahal_cp_proc_ctx_to_hw_buff(type, base, offset,
			hdr_len, hdr_base_addr, offset_entry, l2tp_params,
			eogre_params, generic_params, is_64);
}

/*
 * ipahal_get_proc_ctx_needed_len() - calculates the needed length for
 * addition of header processing context according to the type of processing
 * context
 * @type: header processing context type (no processing context,
 *	IPA_HDR_PROC_ETHII_TO_ETHII etc.)
 */
int ipahal_get_proc_ctx_needed_len(enum ipa_hdr_proc_type type)
{
	int res;

	IPAHAL_DBG("entry\n");

	res = hdr_funcs.ipahal_get_proc_ctx_needed_len(type);

	IPAHAL_DBG("Exit\n");

	return res;
}

int ipahal_init(enum ipa_hw_type ipa_hw_type, void __iomem *base,
	u32 ipa_cfg_offset, struct device *ipa_pdev)
{
	int result;

	IPAHAL_DBG("Entry - IPA HW TYPE=%d base=%pK ipa_pdev=%pK\n",
		ipa_hw_type, base, ipa_pdev);

	ipahal_ctx = kzalloc(sizeof(*ipahal_ctx), GFP_KERNEL);
	if (!ipahal_ctx) {
		IPAHAL_ERR("kzalloc err for ipahal_ctx\n");
		result = -ENOMEM;
		goto bail_err_exit;
	}

	if (ipa_hw_type < IPA_HW_v3_0) {
		IPAHAL_ERR("ipahal supported on IPAv3 and later only\n");
		result = -EINVAL;
		goto bail_free_ctx;
	}

	if (ipa_hw_type >= IPA_HW_MAX) {
		IPAHAL_ERR("invalid IPA HW type (%d)\n", ipa_hw_type);
		result = -EINVAL;
		goto bail_free_ctx;
	}

	if (!base) {
		IPAHAL_ERR("invalid memory io mapping addr\n");
		result = -EINVAL;
		goto bail_free_ctx;
	}

	if (!ipa_pdev) {
		IPAHAL_ERR("invalid IPA platform device\n");
		result = -EINVAL;
		goto bail_free_ctx;
	}

	ipahal_ctx->hw_type = ipa_hw_type;
	ipahal_ctx->base = base;
	ipahal_ctx->ipa_cfg_offset = ipa_cfg_offset;
	ipahal_ctx->ipa_pdev = ipa_pdev;

	if (ipahal_reg_init(ipa_hw_type)) {
		IPAHAL_ERR("failed to init ipahal reg\n");
		result = -EFAULT;
		goto bail_free_ctx;
	}

	if (ipahal_imm_cmd_init(ipa_hw_type)) {
		IPAHAL_ERR("failed to init ipahal imm cmd\n");
		result = -EFAULT;
		goto bail_free_ctx;
	}

	if (ipahal_pkt_status_init(ipa_hw_type)) {
		IPAHAL_ERR("failed to init ipahal pkt status\n");
		result = -EFAULT;
		goto bail_free_ctx;
	}

	if (ipahal_qmap_init(ipa_hw_type)) {
		IPAHAL_ERR("failed to init ipahal qmap\n");
		result = -EFAULT;
		goto bail_free_ctx;
	}

	ipahal_hdr_init(ipa_hw_type);

	if (ipahal_fltrt_init(ipa_hw_type)) {
		IPAHAL_ERR("failed to init ipahal flt rt\n");
		result = -EFAULT;
		goto bail_free_ctx;
	}

	if (ipahal_hw_stats_init(ipa_hw_type)) {
		IPAHAL_ERR("failed to init ipahal hw stats\n");
		result = -EFAULT;
		goto bail_free_fltrt;
	}

	if (ipahal_nat_init(ipa_hw_type)) {
		IPAHAL_ERR("failed to init ipahal NAT\n");
		result = -EFAULT;
		goto bail_free_fltrt;
	}

	/* create an IPC buffer for the registers dump */
	ipahal_ctx->regdumpbuf = ipc_log_context_create(IPAHAL_IPC_LOG_PAGES,
		"ipa_regs", MINIDUMP_MASK);
	if (ipahal_ctx->regdumpbuf == NULL)
		IPAHAL_ERR("failed to create IPA regdump log, continue...\n");

	ipahal_debugfs_init();

	return 0;

bail_free_fltrt:
	ipahal_fltrt_destroy();
bail_free_ctx:
	if (ipahal_ctx->regdumpbuf)
		ipc_log_context_destroy(ipahal_ctx->regdumpbuf);
	kfree(ipahal_ctx);
	ipahal_ctx = NULL;
bail_err_exit:
	return result;
}

void ipahal_destroy(void)
{
	IPAHAL_DBG("Entry\n");
	ipahal_fltrt_destroy();
	ipahal_debugfs_remove();
	kfree(ipahal_ctx);
	ipahal_ctx = NULL;
}

void ipahal_free_dma_mem(struct ipa_mem_buffer *mem)
{
	if (likely(mem)) {
		dma_free_coherent(ipahal_ctx->ipa_pdev, mem->size, mem->base,
			mem->phys_base);
		mem->size = 0;
		mem->base = NULL;
		mem->phys_base = 0;
	}
}

/*
 * ***************************************************************
 *
 * To follow, a generalized qmap header manipulation API.
 *
 * The functions immediately following this comment are version
 * specific qmap parsing functions.  The referred to in the
 * ipahal_qmap_parse_tbl below.
 *
 * ***************************************************************
 */
void ipa_qmap_hdr_parse_v4_5(
	union qmap_hdr_u*     qmap_hdr,
	struct qmap_hdr_data* qmap_data_rslt )
{
	qmap_data_rslt->cd = qmap_hdr->qmap5_0.cd;
	qmap_data_rslt->qmap_next_hdr = qmap_hdr->qmap5_0.qmap_next_hdr;
	qmap_data_rslt->pad = qmap_hdr->qmap5_0.pad;
	qmap_data_rslt->mux_id = qmap_hdr->qmap5_0.mux_id;
	qmap_data_rslt->packet_len_with_pad = qmap_hdr->qmap5_0.packet_len_with_pad;

	qmap_data_rslt->hdr_type = qmap_hdr->qmap5_0.hdr_type;
	qmap_data_rslt->coal_next_hdr = qmap_hdr->qmap5_0.coal_next_hdr;
	qmap_data_rslt->zero_checksum = qmap_hdr->qmap5_0.zero_checksum;
}

void ipa_qmap_hdr_parse_v5_0(
	union qmap_hdr_u*     qmap_hdr,
	struct qmap_hdr_data* qmap_data_rslt )
{
	qmap_data_rslt->cd = qmap_hdr->qmap5_0.cd;
	qmap_data_rslt->qmap_next_hdr = qmap_hdr->qmap5_0.qmap_next_hdr;
	qmap_data_rslt->pad = qmap_hdr->qmap5_0.pad;
	qmap_data_rslt->mux_id = qmap_hdr->qmap5_0.mux_id;
	qmap_data_rslt->packet_len_with_pad = qmap_hdr->qmap5_0.packet_len_with_pad;

	qmap_data_rslt->hdr_type = qmap_hdr->qmap5_0.hdr_type;
	qmap_data_rslt->coal_next_hdr = qmap_hdr->qmap5_0.coal_next_hdr;
	qmap_data_rslt->ip_id_cfg = qmap_hdr->qmap5_0.ip_id_cfg;
	qmap_data_rslt->zero_checksum = qmap_hdr->qmap5_0.zero_checksum;
	qmap_data_rslt->additional_hdr_size = qmap_hdr->qmap5_0.additional_hdr_size;
	qmap_data_rslt->segment_size = qmap_hdr->qmap5_0.segment_size;
}

void ipa_qmap_hdr_parse_v5_5(
	union qmap_hdr_u*     qmap_hdr,
	struct qmap_hdr_data* qmap_data_rslt )
{
	qmap_data_rslt->cd = qmap_hdr->qmap5_5.cd;
	qmap_data_rslt->qmap_next_hdr = qmap_hdr->qmap5_5.qmap_next_hdr;
	qmap_data_rslt->pad = qmap_hdr->qmap5_5.pad;
	qmap_data_rslt->mux_id = qmap_hdr->qmap5_5.mux_id;
	qmap_data_rslt->packet_len_with_pad = ntohs(qmap_hdr->qmap5_5.packet_len_with_pad);

	qmap_data_rslt->hdr_type = qmap_hdr->qmap5_5.hdr_type;
	qmap_data_rslt->coal_next_hdr = qmap_hdr->qmap5_5.coal_next_hdr;
	qmap_data_rslt->chksum_valid = qmap_hdr->qmap5_5.chksum_valid;
	qmap_data_rslt->num_nlos = qmap_hdr->qmap5_5.num_nlos;
	qmap_data_rslt->inc_ip_id = qmap_hdr->qmap5_5.inc_ip_id;
	qmap_data_rslt->rnd_ip_id = qmap_hdr->qmap5_5.rnd_ip_id;
	qmap_data_rslt->close_value = qmap_hdr->qmap5_5.close_value;
	qmap_data_rslt->close_type = qmap_hdr->qmap5_5.close_type;
	qmap_data_rslt->vcid = qmap_hdr->qmap5_5.vcid;
}

/*
 * Structure used to describe a version specific qmap parsing table.
 */
struct ipahal_qmap_parse_s {
	/*
	 * Function prototype for a version specific qmap parsing
	 * function.
	 */
	void (*parse)(
		union qmap_hdr_u*     qmap_hdr,
		struct qmap_hdr_data* qmap_data_rslt );
};

/*
 * Table used to contain and drive version specific qmap parsing
 * functions.
 */
static struct ipahal_qmap_parse_s ipahal_qmap_parse_tbl[IPA_HW_MAX] = {
	/* IPAv4.5 */
	[IPA_HW_v4_5] = {
		ipa_qmap_hdr_parse_v4_5
	},
	/* IPAv5.0 */
	[IPA_HW_v5_0] = {
		ipa_qmap_hdr_parse_v5_0
	},
	/* IPAv5.5 */
	[IPA_HW_v5_5] = {
		ipa_qmap_hdr_parse_v5_5
	},
};

static int ipahal_qmap_init(
	enum ipa_hw_type ipa_hw_type)
{
	struct ipahal_qmap_parse_s zero_obj;
	int i;

	IPAHAL_DBG_LOW("Entry - HW_TYPE=%d\n", ipa_hw_type);

	if (ipa_hw_type < 0 || ipa_hw_type >= IPA_HW_MAX) {
		IPAHAL_ERR("invalid IPA HW type (%d)\n", ipa_hw_type);
		return -EINVAL;
	}

	memset(&zero_obj, 0, sizeof(zero_obj));

	for (i = IPA_HW_v4_5; i < ipa_hw_type; i++) {

		if (memcmp(&ipahal_qmap_parse_tbl[i+1],
				   &zero_obj,
				   sizeof(struct ipahal_qmap_parse_s)) == 0 ) {
			memcpy(
				&ipahal_qmap_parse_tbl[i+1],
				&ipahal_qmap_parse_tbl[i],
				sizeof(struct ipahal_qmap_parse_s));
		} else {
			if (ipahal_qmap_parse_tbl[i+1].parse == 0) {
				IPAHAL_ERR(
					"QMAP parse table missing parse function ipa_ver=%d\n",
					i+1);
				WARN_ON(1);
			}
		}
	}

	return 0;
}

/*
 * FUNCTION: ipahal_qmap_parse()
 *
 * The following Function to be called when version specific qmap parsing is
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
	struct qmap_hdr_data* qmap_data_rslt )
{
	union qmap_hdr_u qmap_hdr;

	IPAHAL_DBG_LOW("Parse qmap/coal header\n");

	if (!unparsed_qmap || !qmap_data_rslt) {
		IPAHAL_ERR(
			"Input Error: unparsed_qmap=%pK qmap_data_rslt=%pK\n",
			unparsed_qmap, qmap_data_rslt);
		return -EINVAL;
	}

	if (ipahal_ctx->hw_type < IPA_HW_v4_5) {
		IPAHAL_ERR(
			"Unsupported qmap parse for IPA HW type (%d)\n",
			ipahal_ctx->hw_type);
		return -EINVAL;
	}

	ipahal_qmap_ntoh(unparsed_qmap, &qmap_hdr);

	ipahal_qmap_parse_tbl[ipahal_ctx->hw_type].parse(&qmap_hdr, qmap_data_rslt);

	return 0;
}
