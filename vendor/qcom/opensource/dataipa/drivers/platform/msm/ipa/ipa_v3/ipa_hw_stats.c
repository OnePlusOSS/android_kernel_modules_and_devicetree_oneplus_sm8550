// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include "ipa_i.h"
#include "ipahal.h"
#include "ipahal_hw_stats.h"

#define IPA_INIT_DROP_STATS_MAX_CMD_NUM 5
#define IPA_INIT_TETH_STATS_MAX_CMD_NUM 5
#define IPA_INIT_QUOTA_STATS_MAX_CMD_NUM 5

static inline u32 ipa_hw_stats_get_ep_bit_n_idx(enum ipa_client_type client,
	u32 *reg_idx)
{
	int ep = ipa3_get_ep_mapping(client);

	if (ep == IPA_EP_NOT_ALLOCATED)
		return 0;

	*reg_idx = ipahal_get_ep_reg_idx(ep);
	return ipahal_get_ep_bit(ep);
}

int ipa_hw_stats_init(void)
{
	int ret = 0, ep_index;
	struct ipa_teth_stats_endpoints *teth_stats_init;
	u32 reg_idx;
	u32 mask = 0;

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_0)
		return 0;

	ipa3_ctx->hw_stats = kzalloc(sizeof(*ipa3_ctx->hw_stats), GFP_KERNEL);
	if (!ipa3_ctx->hw_stats) {
		IPAERR("mem allocated failed!\n");
		return -ENOMEM;
	}

	/* initialize stats here */
	ipa3_ctx->hw_stats->enabled = true;

	/* for IPA_HW_v5_0, reserved teth_stats sram for flt-tbls */
	if (ipa3_ctx->ipa_hw_type == IPA_HW_v5_0)
		return 0;

	teth_stats_init = kzalloc(sizeof(*teth_stats_init), GFP_KERNEL);
	if (!teth_stats_init) {
		IPAERR("mem allocated failed!\n");
		return -ENOMEM;
	}
	/* enable prod mask */
	if (ipa3_ctx->platform_type == IPA_PLAT_TYPE_APQ) {
		mask = ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_MHI_PRIME_TETH_PROD,
			&reg_idx);
		teth_stats_init->prod_mask[reg_idx] = mask;

		mask = ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_USB_PROD,
			&reg_idx);
		teth_stats_init->prod_mask[reg_idx] |= mask;

		if (ipa3_ctx->ipa_wdi3_over_gsi) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WLAN2_PROD,
				&reg_idx);
			teth_stats_init->prod_mask[reg_idx] |= mask;
		} else {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WLAN1_PROD,
				&reg_idx);
			teth_stats_init->prod_mask[reg_idx] |= mask;
		}

		mask = ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_WIGIG_PROD,
			&reg_idx);
		teth_stats_init->prod_mask[reg_idx] |= mask;

		if (ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_MHI_PRIME_TETH_PROD,
			&reg_idx)) {
			ep_index = ipa3_get_ep_mapping(
				IPA_CLIENT_MHI_PRIME_TETH_PROD);
			if (ep_index == -1) {
				IPAERR("Invalid client.\n");
				ret = -EINVAL;
				goto fail_free_stats_ctx;
			}

			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_USB_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] = mask;

			if (ipa3_ctx->ipa_wdi3_over_gsi) {
				mask = ipa_hw_stats_get_ep_bit_n_idx(
					IPA_CLIENT_WLAN2_CONS,
					&reg_idx);
				teth_stats_init->dst_ep_mask[ep_index][reg_idx]
					|= mask;
			} else {
				mask = ipa_hw_stats_get_ep_bit_n_idx(
					IPA_CLIENT_WLAN1_CONS,
					&reg_idx);
				teth_stats_init->dst_ep_mask[ep_index][reg_idx]
					|= mask;
			}

			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG1_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG2_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG3_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG4_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		}
	} else {
		mask = ipa_hw_stats_get_ep_bit_n_idx(IPA_CLIENT_Q6_WAN_PROD,
			&reg_idx);
		teth_stats_init->prod_mask[reg_idx] = mask;

		mask = ipa_hw_stats_get_ep_bit_n_idx(IPA_CLIENT_USB_PROD,
			&reg_idx);
		teth_stats_init->prod_mask[reg_idx] |= mask;

		if (ipa3_ctx->ipa_wdi3_over_gsi) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WLAN2_PROD,
				&reg_idx);
			teth_stats_init->prod_mask[reg_idx] |= mask;
		} else {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WLAN1_PROD,
				&reg_idx);
			teth_stats_init->prod_mask[reg_idx] |= mask;
		}

		mask = ipa_hw_stats_get_ep_bit_n_idx(IPA_CLIENT_WIGIG_PROD,
			&reg_idx);
		teth_stats_init->prod_mask[reg_idx] |= mask;

		if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_5) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_Q6_DL_NLO_DATA_PROD,
				&reg_idx);
			teth_stats_init->prod_mask[reg_idx] |= mask;
		}

		if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_1) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_Q6_DL_NLO_LL_DATA_PROD,
				&reg_idx);
			teth_stats_init->prod_mask[reg_idx] |= mask;
		}

		if (ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_Q6_WAN_PROD,
			&reg_idx)) {
			ep_index = ipa3_get_ep_mapping(IPA_CLIENT_Q6_WAN_PROD);
			if (ep_index == -1) {
				IPAERR("Invalid client.\n");
				ret = -EINVAL;
				goto fail_free_stats_ctx;
			}

			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_USB_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] = mask;

			if (ipa3_ctx->ipa_wdi3_over_gsi) {
				mask = ipa_hw_stats_get_ep_bit_n_idx(
					IPA_CLIENT_WLAN2_CONS,
					&reg_idx);
				teth_stats_init->dst_ep_mask[ep_index][reg_idx]
					|= mask;
			} else {
				mask = ipa_hw_stats_get_ep_bit_n_idx(
					IPA_CLIENT_WLAN1_CONS,
					&reg_idx);
				teth_stats_init->dst_ep_mask[ep_index][reg_idx]
					|= mask;
			}

			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG1_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG2_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG3_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG4_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		}

		if (ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_Q6_DL_NLO_DATA_PROD,
			&reg_idx) && (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_5)) {
			ep_index = ipa3_get_ep_mapping(
				IPA_CLIENT_Q6_DL_NLO_DATA_PROD);
			if (ep_index == -1) {
				IPAERR("Invalid client.\n");
				ret = -EINVAL;
				goto fail_free_stats_ctx;
			}
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_USB_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] = mask;

			if (ipa3_ctx->ipa_wdi3_over_gsi) {
				mask = ipa_hw_stats_get_ep_bit_n_idx(
					IPA_CLIENT_WLAN2_CONS,
					&reg_idx);
				teth_stats_init->dst_ep_mask[ep_index][reg_idx]
					|= mask;
			} else {
				mask = ipa_hw_stats_get_ep_bit_n_idx(
					IPA_CLIENT_WLAN1_CONS,
					&reg_idx);
				teth_stats_init->dst_ep_mask[ep_index][reg_idx]
					|= mask;
			}

			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG1_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG2_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG3_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG4_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		}

		if (ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_Q6_DL_NLO_LL_DATA_PROD,
			&reg_idx) && (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_0)) {
			ep_index = ipa3_get_ep_mapping(
					IPA_CLIENT_Q6_DL_NLO_LL_DATA_PROD);
			if (ep_index == -1) {
				IPAERR("Invalid client.\n");
				ret = -EINVAL;
				goto fail_free_stats_ctx;
			}
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_USB_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] = mask;

			if (ipa3_ctx->ipa_wdi3_over_gsi) {
				mask = ipa_hw_stats_get_ep_bit_n_idx(
					IPA_CLIENT_WLAN2_CONS,
					&reg_idx);
				teth_stats_init->dst_ep_mask[ep_index][reg_idx]
					|= mask;
			} else {
				mask = ipa_hw_stats_get_ep_bit_n_idx(
					IPA_CLIENT_WLAN1_CONS,
					&reg_idx);
				teth_stats_init->dst_ep_mask[ep_index][reg_idx]
					|= mask;
			}

			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG1_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG2_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG3_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WIGIG4_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		}
	}

	if (ipa_hw_stats_get_ep_bit_n_idx(
		IPA_CLIENT_USB_PROD,
		&reg_idx)) {
		ep_index = ipa3_get_ep_mapping(IPA_CLIENT_USB_PROD);
		if (ep_index == -1) {
			IPAERR("Invalid client.\n");
			ret = -EINVAL;
			goto fail_free_stats_ctx;
		}

		mask = ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_Q6_WAN_CONS,
			&reg_idx);
		teth_stats_init->dst_ep_mask[ep_index][reg_idx] = mask;

		/* enable additional pipe monitoring for pcie modem */
		if (ipa3_ctx->platform_type == IPA_PLAT_TYPE_APQ) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_MHI_PRIME_TETH_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		} else if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_5) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_Q6_UL_NLO_DATA_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		}
	}

	if (ipa_hw_stats_get_ep_bit_n_idx(
		IPA_CLIENT_WLAN1_PROD,
		&reg_idx)) {
		ep_index = ipa3_get_ep_mapping(IPA_CLIENT_WLAN1_PROD);
		if (ep_index == -1) {
			IPAERR("Invalid client.\n");
			ret = -EINVAL;
			goto fail_free_stats_ctx;
		}

		mask = ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_Q6_WAN_CONS,
			&reg_idx);
		teth_stats_init->dst_ep_mask[ep_index][reg_idx] = mask;

		/* enable additional pipe monitoring for pcie modem*/
		if (ipa3_ctx->platform_type == IPA_PLAT_TYPE_APQ) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_MHI_PRIME_TETH_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		} else if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_5) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_Q6_UL_NLO_DATA_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		}
	}

	if (ipa_hw_stats_get_ep_bit_n_idx(
		IPA_CLIENT_WLAN2_PROD,
		&reg_idx)) {
		ep_index = ipa3_get_ep_mapping(IPA_CLIENT_WLAN2_PROD);
		if (ep_index == -1) {
			IPAERR("Invalid client.\n");
			ret = -EINVAL;
			goto fail_free_stats_ctx;
		}

		mask = ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_Q6_WAN_CONS,
			&reg_idx);
		teth_stats_init->dst_ep_mask[ep_index][reg_idx] = mask;

		/* enable additional pipe monitoring for pcie modem*/
		if (ipa3_ctx->platform_type == IPA_PLAT_TYPE_APQ) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_MHI_PRIME_TETH_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		} else if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_5) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_Q6_UL_NLO_DATA_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		}
	}

	if (ipa_hw_stats_get_ep_bit_n_idx(
		IPA_CLIENT_WIGIG_PROD,
		&reg_idx)) {
		ep_index = ipa3_get_ep_mapping(IPA_CLIENT_WIGIG_PROD);
		if (ep_index == -1) {
			IPAERR("Invalid client.\n");
			ret = -EINVAL;
			goto fail_free_stats_ctx;
		}

		mask = ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_Q6_WAN_CONS,
			&reg_idx);
		teth_stats_init->dst_ep_mask[ep_index][reg_idx] = mask;

		/* enable additional pipe monitoring for pcie modem */
		if (ipa3_ctx->platform_type == IPA_PLAT_TYPE_APQ) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_MHI_PRIME_TETH_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		} else if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_5) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_Q6_UL_NLO_DATA_CONS,
				&reg_idx);
			teth_stats_init->dst_ep_mask[ep_index][reg_idx] |= mask;
		}
	}


	ret = ipa_init_teth_stats(teth_stats_init);
	if (ret != 0) {
		IPAERR("init teth stats fails\n");
		goto fail_free_stats_ctx;
	}

	ipa3_ctx->hw_stats->teth_stats_enabled = true;
	kfree(teth_stats_init);
	return ret;

fail_free_stats_ctx:
	kfree(teth_stats_init);
	kfree(ipa3_ctx->hw_stats);
	ipa3_ctx->hw_stats = NULL;
	return ret;
}

static void ipa_close_coal_frame(struct ipahal_imm_cmd_pyld **coal_cmd_pyld)
{
	int i;
	struct ipahal_reg_valmask valmask;
	struct ipahal_imm_cmd_register_write reg_write_coal_close;
	u32 offset = 0;

	i = ipa3_get_ep_mapping(IPA_CLIENT_APPS_WAN_COAL_CONS);
	reg_write_coal_close.skip_pipeline_clear = false;
	reg_write_coal_close.pipeline_clear_options = IPAHAL_HPS_CLEAR;
	if (ipa3_ctx->ipa_hw_type < IPA_HW_v5_0)
		offset = ipahal_get_reg_ofst(
			IPA_AGGR_FORCE_CLOSE);
	else
		offset = ipahal_get_ep_reg_offset(
			IPA_AGGR_FORCE_CLOSE_n, i);
	reg_write_coal_close.offset = offset;
	ipahal_get_aggr_force_close_valmask(i, &valmask);
	reg_write_coal_close.value = valmask.val;
	reg_write_coal_close.value_mask = valmask.mask;
	*coal_cmd_pyld = ipahal_construct_imm_cmd(
		IPA_IMM_CMD_REGISTER_WRITE,
		&reg_write_coal_close, false);
}

static bool ipa_validate_quota_stats_sram_size(u32 needed_len)
{
	u32 sram_size;

	/* Starting IPA4.5 Quota stats is split between Q6 and AP */

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5) {
		IPAERR("Not supported ipa_ver=%d\n", ipa3_ctx->ipa_hw_type);
		return false;
	}

	sram_size = IPA_MEM_PART(stats_quota_ap_size);
	if (needed_len > sram_size) {
		IPAERR("SRAM partition too small: %u needed %u\n",
			sram_size, needed_len);
		return false;
	}

	return true;
}

int ipa_init_quota_stats(u32 *pipe_bitmask)
{
	struct ipahal_stats_init_pyld *pyld;
	struct ipahal_imm_cmd_dma_shared_mem cmd = { 0 };
	struct ipahal_imm_cmd_pyld *cmd_pyld;
	struct ipahal_imm_cmd_register_write quota_base = {0};
	struct ipahal_imm_cmd_pyld *quota_base_pyld;
	struct ipahal_imm_cmd_register_write quota_mask = {0};
	struct ipahal_imm_cmd_pyld *quota_mask_pyld[IPA5_PIPE_REG_NUM] = {0};
	struct ipahal_imm_cmd_pyld *coal_cmd_pyld = NULL;
	struct ipa3_desc desc[IPA_INIT_QUOTA_STATS_MAX_CMD_NUM] = { {0} };
	dma_addr_t dma_address;
	int ret;
	int num_cmd = 0;
	int ipa_ep_idx = IPA_EP_NOT_ALLOCATED;
	int i;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	if (!pipe_bitmask)
		return -EPERM;

	/* reset driver's cache */
	memset(&ipa3_ctx->hw_stats->quota, 0, sizeof(ipa3_ctx->hw_stats->quota));
	for (i = 0; i < IPA5_PIPE_REG_NUM; i++) {
		ipa3_ctx->hw_stats->quota.init.enabled_bitmask[i] =
			pipe_bitmask[i];
		IPADBG_LOW("pipe_bitmask[%d]=0x%x\n", i, pipe_bitmask[i]);
	}

	pyld = ipahal_stats_generate_init_pyld(IPAHAL_HW_STATS_QUOTA,
		&ipa3_ctx->hw_stats->quota.init, false);
	if (!pyld) {
		IPAERR("failed to generate pyld\n");
		return -EPERM;
	}

	if (!ipa_validate_quota_stats_sram_size(pyld->len)) {
		ret = -EPERM;
		goto destroy_init_pyld;
	}

	dma_address = dma_map_single(ipa3_ctx->pdev,
		pyld->data,
		pyld->len,
		DMA_TO_DEVICE);
	if (dma_mapping_error(ipa3_ctx->pdev, dma_address)) {
		IPAERR("failed to DMA map\n");
		ret = -EPERM;
		goto destroy_init_pyld;
	}

	/* IC to close the coal frame before HPS Clear if coal is enabled */
	ipa_ep_idx = ipa3_get_ep_mapping(IPA_CLIENT_APPS_WAN_COAL_CONS);
	if (ipa_ep_idx != IPA_EP_NOT_ALLOCATED && !ipa3_ctx->ulso_wa) {
		ipa_close_coal_frame(&coal_cmd_pyld);
		if (!coal_cmd_pyld) {
			IPAERR("failed to construct coal close IC\n");
			ret = -ENOMEM;
			goto unmap;
		}
		ipa3_init_imm_cmd_desc(&desc[num_cmd], coal_cmd_pyld);
		++num_cmd;
	}

	/* setting the registers and init the stats pyld are done atomically */
	quota_mask.skip_pipeline_clear = false;
	quota_mask.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	if (ipa3_ctx->ipa_hw_type < IPA_HW_v5_0) {
		quota_mask.offset = ipahal_get_reg_n_ofst(IPA_STAT_QUOTA_MASK_n,
			ipa3_ctx->ee);
		quota_mask.value = pipe_bitmask[0];
		quota_mask.value_mask = ~0;
		quota_mask_pyld[0] = ipahal_construct_imm_cmd(
			IPA_IMM_CMD_REGISTER_WRITE,
			&quota_mask, false);
		if (!quota_mask_pyld[0]) {
			IPAERR("failed to construct register_write imm cmd\n");
			ret = -ENOMEM;
			goto destroy_coal_cmd;
		}
		desc[num_cmd].opcode = quota_mask_pyld[0]->opcode;
		desc[num_cmd].pyld = quota_mask_pyld[0]->data;
		desc[num_cmd].len = quota_mask_pyld[0]->len;
		desc[num_cmd].type = IPA_IMM_CMD_DESC;
		num_cmd++;
	} else {
		for (i = 0; i < IPA5_PIPE_REG_NUM; i++) {
			quota_mask.value = pipe_bitmask[i];
			quota_mask.value_mask = ~0;
			quota_mask.offset = ipahal_get_reg_nk_offset(
				IPA_STAT_QUOTA_MASK_EE_n_REG_k,
				ipa3_ctx->ee, i);
			quota_mask_pyld[i] = ipahal_construct_imm_cmd(
				IPA_IMM_CMD_REGISTER_WRITE,
				&quota_mask, false);
			if (!quota_mask_pyld[i]) {
				int j;

				IPAERR(
					"failed to construct register_write imm cmd\n"
				);
				for (j = i - 1; j >= 0; j--)
					ipahal_destroy_imm_cmd(
						quota_mask_pyld[j]);
				ret = -ENOMEM;
				goto destroy_coal_cmd;
			}
			desc[num_cmd].opcode = quota_mask_pyld[i]->opcode;
			desc[num_cmd].pyld = quota_mask_pyld[i]->data;
			desc[num_cmd].len = quota_mask_pyld[i]->len;
			desc[num_cmd].type = IPA_IMM_CMD_DESC;
			num_cmd++;
		}
	}

	quota_base.skip_pipeline_clear = false;
	quota_base.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	quota_base.offset = ipahal_get_reg_n_ofst(IPA_STAT_QUOTA_BASE_n,
		ipa3_ctx->ee);
	quota_base.value = ipa3_ctx->smem_restricted_bytes +
		IPA_MEM_PART(stats_quota_ap_ofst);
	quota_base.value_mask = ~0;
	quota_base_pyld = ipahal_construct_imm_cmd(IPA_IMM_CMD_REGISTER_WRITE,
		&quota_base, false);
	if (!quota_base_pyld) {
		IPAERR("failed to construct register_write imm cmd\n");
		ret = -ENOMEM;
		goto destroy_quota_mask;
	}
	desc[num_cmd].opcode = quota_base_pyld->opcode;
	desc[num_cmd].pyld = quota_base_pyld->data;
	desc[num_cmd].len = quota_base_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	num_cmd++;

	cmd.is_read = false;
	cmd.skip_pipeline_clear = false;
	cmd.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	cmd.size = pyld->len;
	cmd.system_addr = dma_address;
	cmd.local_addr = ipa3_ctx->smem_restricted_bytes +
		IPA_MEM_PART(stats_quota_ap_ofst);
	cmd_pyld = ipahal_construct_imm_cmd(
		IPA_IMM_CMD_DMA_SHARED_MEM, &cmd, false);
	if (!cmd_pyld) {
		IPAERR("failed to construct dma_shared_mem imm cmd\n");
		ret = -ENOMEM;
		goto destroy_quota_base;
	}
	desc[num_cmd].opcode = cmd_pyld->opcode;
	desc[num_cmd].pyld = cmd_pyld->data;
	desc[num_cmd].len = cmd_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	num_cmd++;

	ret = ipa3_send_cmd(num_cmd, desc);
	if (ret) {
		IPAERR("failed to send immediate command (error %d)\n", ret);
		goto destroy_imm;
	}

	ret = 0;

destroy_imm:
	ipahal_destroy_imm_cmd(cmd_pyld);
destroy_quota_base:
	ipahal_destroy_imm_cmd(quota_base_pyld);
destroy_quota_mask:
	for (i = 0; i < IPA5_PIPE_REG_NUM; i++)
		if (quota_mask_pyld[i])
			ipahal_destroy_imm_cmd(quota_mask_pyld[i]);
destroy_coal_cmd:
	ipahal_destroy_imm_cmd(coal_cmd_pyld);
unmap:
	dma_unmap_single(ipa3_ctx->pdev, dma_address, pyld->len, DMA_TO_DEVICE);
destroy_init_pyld:
	ipahal_destroy_stats_init_pyld(pyld);
	return ret;
}

int ipa_get_quota_stats(struct ipa_quota_stats_all *out)
{
	int i;
	int ret;
	struct ipahal_stats_get_offset_quota get_offset = { { 0 } };
	struct ipahal_stats_offset offset = { 0 };
	struct ipahal_imm_cmd_dma_shared_mem cmd = { 0 };
	struct ipahal_imm_cmd_pyld *cmd_pyld[2];
	struct ipa_mem_buffer mem;
	struct ipa3_desc desc[2];
	struct ipahal_stats_quota_all *stats;
	int num_cmd = 0;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	memset(desc, 0, sizeof(desc));
	memset(cmd_pyld, 0, sizeof(cmd_pyld));

	get_offset.init = ipa3_ctx->hw_stats->quota.init;
	ret = ipahal_stats_get_offset(IPAHAL_HW_STATS_QUOTA, &get_offset,
		&offset);
	if (ret) {
		IPAERR("failed to get offset from hal %d\n", ret);
		return ret;
	}

	IPADBG_LOW("offset = %d size = %d\n", offset.offset, offset.size);

	if (offset.size == 0)
		return 0;

	mem.size = offset.size;
	mem.base = dma_alloc_coherent(ipa3_ctx->pdev,
		mem.size,
		&mem.phys_base,
		GFP_KERNEL);
	if (!mem.base) {
		IPAERR("fail to alloc DMA memory");
		return ret;
	}

	/* IC to close the coal frame before HPS Clear if coal is enabled */
	if (ipa3_get_ep_mapping(IPA_CLIENT_APPS_WAN_COAL_CONS) !=
		IPA_EP_NOT_ALLOCATED && !ipa3_ctx->ulso_wa) {
		ipa_close_coal_frame(&cmd_pyld[num_cmd]);
		if (!cmd_pyld[num_cmd]) {
			IPAERR("failed to construct coal close IC\n");
			ret = -ENOMEM;
			goto free_dma_mem;
		}
		ipa3_init_imm_cmd_desc(&desc[num_cmd], cmd_pyld[num_cmd]);
		++num_cmd;
	}

	cmd.is_read = true;
	cmd.clear_after_read = true;
	cmd.skip_pipeline_clear = false;
	cmd.pipeline_clear_options = IPAHAL_HPS_CLEAR;
	cmd.size = mem.size;
	cmd.system_addr = mem.phys_base;
	cmd.local_addr = ipa3_ctx->smem_restricted_bytes +
		IPA_MEM_PART(stats_quota_ap_ofst) + offset.offset;
	cmd_pyld[num_cmd] = ipahal_construct_imm_cmd(
		IPA_IMM_CMD_DMA_SHARED_MEM, &cmd, false);
	if (!cmd_pyld[num_cmd]) {
		IPAERR("failed to construct dma_shared_mem imm cmd\n");
		ret = -ENOMEM;
		goto free_dma_mem;
	}
	ipa3_init_imm_cmd_desc(&desc[num_cmd], cmd_pyld[num_cmd]);
	++num_cmd;

	ret = ipa3_send_cmd(num_cmd, desc);
	if (ret) {
		IPAERR("failed to send immediate command (error %d)\n", ret);
		goto destroy_imm;
	}

	stats = kzalloc(sizeof(*stats), GFP_KERNEL);
	if (!stats) {
		ret = -ENOMEM;
		goto destroy_imm;
	}

	ret = ipahal_parse_stats(IPAHAL_HW_STATS_QUOTA,
		&ipa3_ctx->hw_stats->quota.init, mem.base, stats);
	if (ret) {
		IPAERR("failed to parse stats (error %d)\n", ret);
		goto free_stats;
	}

	/*
	 * update driver cache.
	 * the stats were read from hardware with clear_after_read meaning
	 * hardware stats are 0 now
	 */
	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		int ep_idx = ipa3_get_ep_mapping(i);

		if (ep_idx == -1 || ep_idx >= ipa3_get_max_num_pipes())
			continue;

		if (ipa3_ctx->ep[ep_idx].client != i)
			continue;

		ipa3_ctx->hw_stats->quota.stats.client[i].num_ipv4_bytes +=
			stats->stats[ep_idx].num_ipv4_bytes;
		ipa3_ctx->hw_stats->quota.stats.client[i].num_ipv4_pkts +=
			stats->stats[ep_idx].num_ipv4_pkts;
		ipa3_ctx->hw_stats->quota.stats.client[i].num_ipv6_bytes +=
			stats->stats[ep_idx].num_ipv6_bytes;
		ipa3_ctx->hw_stats->quota.stats.client[i].num_ipv6_pkts +=
			stats->stats[ep_idx].num_ipv6_pkts;
	}

	/* copy results to out parameter */
	if (out)
		*out = ipa3_ctx->hw_stats->quota.stats;
	ret = 0;
free_stats:
	kfree(stats);
destroy_imm:
	for (i = 0; i < num_cmd; i++)
		ipahal_destroy_imm_cmd(cmd_pyld[i]);
free_dma_mem:
	dma_free_coherent(ipa3_ctx->pdev, mem.size, mem.base, mem.phys_base);
	return ret;

}

int ipa_reset_quota_stats(enum ipa_client_type client)
{
	int ret;
	struct ipa_quota_stats *stats;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	if (client >= IPA_CLIENT_MAX) {
		IPAERR("invalid client %d\n", client);
		return -EINVAL;
	}

	/* reading stats will reset them in hardware */
	ret = ipa_get_quota_stats(NULL);
	if (ret) {
		IPAERR("ipa_get_quota_stats failed %d\n", ret);
		return ret;
	}

	/* reset driver's cache */
	stats = &ipa3_ctx->hw_stats->quota.stats.client[client];
	memset(stats, 0, sizeof(*stats));
	return 0;
}

int ipa_reset_all_quota_stats(void)
{
	int ret;
	struct ipa_quota_stats_all *stats;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	/* reading stats will reset them in hardware */
	ret = ipa_get_quota_stats(NULL);
	if (ret) {
		IPAERR("ipa_get_quota_stats failed %d\n", ret);
		return ret;
	}

	/* reset driver's cache */
	stats = &ipa3_ctx->hw_stats->quota.stats;
	memset(stats, 0, sizeof(*stats));
	return 0;
}

int ipa_init_teth_stats(struct ipa_teth_stats_endpoints *in)
{
	struct ipahal_stats_init_pyld *pyld;
	struct ipahal_imm_cmd_dma_shared_mem cmd = { 0 };
	struct ipahal_imm_cmd_pyld *cmd_pyld;
	struct ipahal_imm_cmd_register_write teth_base = {0};
	struct ipahal_imm_cmd_pyld *teth_base_pyld;
	struct ipahal_imm_cmd_register_write teth_mask = { 0 };
	struct ipahal_imm_cmd_pyld *teth_mask_pyld[IPA5_PIPE_REG_NUM] = {0};
	struct ipahal_imm_cmd_pyld *coal_cmd_pyld = NULL;
	struct ipa3_desc desc[IPA_INIT_TETH_STATS_MAX_CMD_NUM] = { {0} };
	dma_addr_t dma_address;
	int ret;
	int i, j;
	int reg_idx;
	int num_cmd = 0;


	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	if (!in || (!in->prod_mask[0] && !in->prod_mask[1])) {
		IPAERR("invalid params\n");
		return -EINVAL;
	}

	reg_idx = 0;
	for (i = 0; i < IPA5_PIPES_NUM; i++) {
		if (i > 0 && !(i % IPA_STATS_MAX_PIPE_BIT)) {
			reg_idx++;
		}
		if (in->prod_mask[reg_idx] & ipahal_get_ep_bit(i)) {
			bool has_cons = false;

			for (j = 0; j < IPA5_PIPE_REG_NUM; j++) {
				if (in->dst_ep_mask[i][j])
					has_cons = true;
			}
			if (!has_cons) {
				IPAERR("prod %d doesn't have cons\n", i);
				return -EINVAL;
			}
		}
	}

	IPADBG("prod_mask=[0x%x][0x%x]\n",
		in->prod_mask[0], in->prod_mask[1]);

	/* reset driver's cache */
	memset(&ipa3_ctx->hw_stats->teth.init, 0,
		sizeof(ipa3_ctx->hw_stats->teth.init));
	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		memset(&ipa3_ctx->hw_stats->teth.prod_stats_sum[i], 0,
			sizeof(ipa3_ctx->hw_stats->teth.prod_stats_sum[i]));
		memset(&ipa3_ctx->hw_stats->teth.prod_stats[i], 0,
			sizeof(ipa3_ctx->hw_stats->teth.prod_stats[i]));
	}
	for (i = 0; i < IPA5_PIPE_REG_NUM; i++) {
		ipa3_ctx->hw_stats->teth.init.prod_bitmask[i] = in->prod_mask[i];
	}

	memcpy(ipa3_ctx->hw_stats->teth.init.cons_bitmask, in->dst_ep_mask,
		sizeof(ipa3_ctx->hw_stats->teth.init.cons_bitmask));

	pyld = ipahal_stats_generate_init_pyld(IPAHAL_HW_STATS_TETHERING,
		&ipa3_ctx->hw_stats->teth.init, false);
	if (!pyld) {
		IPAERR("failed to generate pyld\n");
		return -EPERM;
	}

	if (pyld->len > IPA_MEM_PART(stats_tethering_size)) {
		IPAERR("SRAM partition too small: %d needed %d\n",
			IPA_MEM_PART(stats_tethering_size), pyld->len);
		ret = -EPERM;
		goto destroy_init_pyld;
	}

	dma_address = dma_map_single(ipa3_ctx->pdev,
		pyld->data,
		pyld->len,
		DMA_TO_DEVICE);
	if (dma_mapping_error(ipa3_ctx->pdev, dma_address)) {
		IPAERR("failed to DMA map\n");
		ret = -EPERM;
		goto destroy_init_pyld;
	}

	/* IC to close the coal frame before HPS Clear if coal is enabled */
	if (ipa3_get_ep_mapping(IPA_CLIENT_APPS_WAN_COAL_CONS) !=
		IPA_EP_NOT_ALLOCATED && !ipa3_ctx->ulso_wa) {
		ipa_close_coal_frame(&coal_cmd_pyld);
		if (!coal_cmd_pyld) {
			IPAERR("failed to construct coal close IC\n");
			ret = -ENOMEM;
			goto unmap;
		}
		ipa3_init_imm_cmd_desc(&desc[num_cmd], coal_cmd_pyld);
		++num_cmd;
	}

	/* setting the registers and init the stats pyld are done atomically */
	teth_mask.skip_pipeline_clear = false;
	teth_mask.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	teth_mask.value_mask = ~0;
	if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_0) {
		for (i = 0; i < IPA5_PIPE_REG_NUM; i++) {
			teth_mask.offset = ipahal_get_reg_nk_offset(
				IPA_STAT_TETHERING_MASK_EE_n_REG_k,
				ipa3_ctx->ee, i);
			teth_mask.value = in->prod_mask[i];
			teth_mask_pyld[i] = ipahal_construct_imm_cmd(
				IPA_IMM_CMD_REGISTER_WRITE,
				&teth_mask, false);
			if (!teth_mask_pyld[i]) {
				IPAERR(
				"failed to construct register_write imm cmd\n");
				for (j = i - 1; j >= 0; j--) {
					ipahal_destroy_imm_cmd(
						teth_mask_pyld[j]);
					teth_mask_pyld[j] = NULL;
				}
				ret = -ENOMEM;
				goto destroy_coal_cmd;
			}
			desc[num_cmd].opcode = teth_mask_pyld[i]->opcode;
			desc[num_cmd].pyld = teth_mask_pyld[i]->data;
			desc[num_cmd].len = teth_mask_pyld[i]->len;
			desc[num_cmd].type = IPA_IMM_CMD_DESC;
			++num_cmd;
		}

	} else {
		teth_mask.offset = ipahal_get_reg_n_ofst(
			IPA_STAT_TETHERING_MASK_n,
			ipa3_ctx->ee);
		teth_mask.value = in->prod_mask[0];
		teth_mask_pyld[0] = ipahal_construct_imm_cmd(
			IPA_IMM_CMD_REGISTER_WRITE,
			&teth_mask, false);
		if (!teth_mask_pyld[0]) {
			IPAERR("failed to construct register_write imm cmd\n");
			ret = -ENOMEM;
			goto destroy_coal_cmd;
		}
		desc[num_cmd].opcode = teth_mask_pyld[0]->opcode;
		desc[num_cmd].pyld = teth_mask_pyld[0]->data;
		desc[num_cmd].len = teth_mask_pyld[0]->len;
		desc[num_cmd].type = IPA_IMM_CMD_DESC;
		++num_cmd;
	}

	teth_base.skip_pipeline_clear = false;
	teth_base.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	teth_base.offset = ipahal_get_reg_n_ofst(IPA_STAT_TETHERING_BASE_n,
		ipa3_ctx->ee);
	teth_base.value = ipa3_ctx->smem_restricted_bytes +
		IPA_MEM_PART(stats_tethering_ofst);
	teth_base.value_mask = ~0;
	teth_base_pyld = ipahal_construct_imm_cmd(IPA_IMM_CMD_REGISTER_WRITE,
		&teth_base, false);
	if (!teth_base_pyld) {
		IPAERR("failed to construct register_write imm cmd\n");
		ret = -ENOMEM;
		goto destroy_teth_mask;
	}
	desc[num_cmd].opcode = teth_base_pyld->opcode;
	desc[num_cmd].pyld = teth_base_pyld->data;
	desc[num_cmd].len = teth_base_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	++num_cmd;

	cmd.is_read = false;
	cmd.skip_pipeline_clear = false;
	cmd.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	cmd.size = pyld->len;
	cmd.system_addr = dma_address;
	cmd.local_addr = ipa3_ctx->smem_restricted_bytes +
			IPA_MEM_PART(stats_tethering_ofst);
	cmd_pyld = ipahal_construct_imm_cmd(
		IPA_IMM_CMD_DMA_SHARED_MEM, &cmd, false);
	if (!cmd_pyld) {
		IPAERR("failed to construct dma_shared_mem imm cmd\n");
		ret = -ENOMEM;
		goto destroy_teth_base;
	}
	desc[num_cmd].opcode = cmd_pyld->opcode;
	desc[num_cmd].pyld = cmd_pyld->data;
	desc[num_cmd].len = cmd_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	++num_cmd;

	ret = ipa3_send_cmd(num_cmd, desc);
	if (ret) {
		IPAERR("failed to send immediate command (error %d)\n", ret);
		goto destroy_imm;
	}

	ret = 0;

destroy_imm:
	ipahal_destroy_imm_cmd(cmd_pyld);
destroy_teth_base:
		ipahal_destroy_imm_cmd(teth_base_pyld);
destroy_teth_mask:
	for (i = 0; i < IPA5_PIPE_REG_NUM; i++) {
		if (teth_mask_pyld[i])
			ipahal_destroy_imm_cmd(teth_mask_pyld[i]);
	}
destroy_coal_cmd:
	if (coal_cmd_pyld)
		ipahal_destroy_imm_cmd(coal_cmd_pyld);
unmap:
	dma_unmap_single(ipa3_ctx->pdev, dma_address, pyld->len, DMA_TO_DEVICE);
destroy_init_pyld:
	ipahal_destroy_stats_init_pyld(pyld);
	return ret;
}

int ipa_get_teth_stats(void)
{
	int i, j;
	int prod_reg, cons_reg;
	int ret;
	struct ipahal_stats_get_offset_tethering get_offset;
	struct ipahal_stats_offset offset = {0};
	struct ipahal_imm_cmd_dma_shared_mem cmd = { 0 };
	struct ipahal_imm_cmd_pyld *cmd_pyld[2];
	struct ipa_mem_buffer mem;
	struct ipa3_desc desc[2];
	struct ipahal_stats_tethering_all *stats_all;
	struct ipa_hw_stats_teth *sw_stats;
	struct ipahal_stats_tethering *stats;
	struct ipa_quota_stats *quota_stats;
	struct ipahal_stats_init_tethering *init;
	int num_cmd = 0;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled &&
		ipa3_ctx->hw_stats->teth_stats_enabled))
		return 0;

	sw_stats = &ipa3_ctx->hw_stats->teth;
	init = (struct ipahal_stats_init_tethering *)
			&ipa3_ctx->hw_stats->teth.init;

	memset(desc, 0, sizeof(desc));
	memset(cmd_pyld, 0, sizeof(cmd_pyld));
	memset(&get_offset, 0, sizeof(get_offset));

	get_offset.init = ipa3_ctx->hw_stats->teth.init;
	ret = ipahal_stats_get_offset(IPAHAL_HW_STATS_TETHERING, &get_offset,
		&offset);
	if (ret) {
		IPAERR("failed to get offset from hal %d\n", ret);
		return ret;
	}

	IPADBG_LOW("offset = %d size = %d\n", offset.offset, offset.size);

	if (offset.size == 0)
		return 0;

	mem.size = offset.size;
	mem.base = dma_alloc_coherent(ipa3_ctx->pdev,
		mem.size,
		&mem.phys_base,
		GFP_KERNEL);
	if (!mem.base) {
		IPAERR("fail to alloc DMA memory\n");
		return ret;
	}

	/* IC to close the coal frame before HPS Clear if coal is enabled */
	if (ipa3_get_ep_mapping(IPA_CLIENT_APPS_WAN_COAL_CONS) !=
		IPA_EP_NOT_ALLOCATED && !ipa3_ctx->ulso_wa) {
		ipa_close_coal_frame(&cmd_pyld[num_cmd]);
		if (!cmd_pyld[num_cmd]) {
			IPAERR("failed to construct coal close IC\n");
			ret = -ENOMEM;
			goto free_dma_mem;
		}
		ipa3_init_imm_cmd_desc(&desc[num_cmd], cmd_pyld[num_cmd]);
		++num_cmd;
	}

	cmd.is_read = true;
	cmd.clear_after_read = true;
	cmd.skip_pipeline_clear = false;
	cmd.pipeline_clear_options = IPAHAL_HPS_CLEAR;
	cmd.size = mem.size;
	cmd.system_addr = mem.phys_base;
	cmd.local_addr = ipa3_ctx->smem_restricted_bytes +
		IPA_MEM_PART(stats_tethering_ofst) + offset.offset;
	cmd_pyld[num_cmd] = ipahal_construct_imm_cmd(
		IPA_IMM_CMD_DMA_SHARED_MEM, &cmd, false);
	if (!cmd_pyld[num_cmd]) {
		IPAERR("failed to construct dma_shared_mem imm cmd\n");
		ret = -ENOMEM;
		goto destroy_imm;
	}
	ipa3_init_imm_cmd_desc(&desc[num_cmd], cmd_pyld[num_cmd]);
	++num_cmd;

	ret = ipa3_send_cmd(num_cmd, desc);
	if (ret) {
		IPAERR("failed to send immediate command (error %d)\n", ret);
		goto destroy_imm;
	}

	stats_all = kzalloc(sizeof(*stats_all), GFP_KERNEL);
	if (!stats_all) {
		IPADBG("failed to alloc memory\n");
		ret = -ENOMEM;
		goto destroy_imm;
	}

	ret = ipahal_parse_stats(IPAHAL_HW_STATS_TETHERING,
		&ipa3_ctx->hw_stats->teth.init, mem.base, stats_all);
	if (ret) {
		IPAERR("failed to parse stats_all (error %d)\n", ret);
		goto free_stats;
	}

	/* reset prod_stats cache */
	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		memset(&ipa3_ctx->hw_stats->teth.prod_stats[i], 0,
			sizeof(ipa3_ctx->hw_stats->teth.prod_stats[i]));
	}

	/*
	 * update driver cache.
	 * the stats were read from hardware with clear_after_read meaning
	 * hardware stats are 0 now
	 */
	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		for (j = 0; j < IPA_CLIENT_MAX; j++) {
			int prod_idx = ipa3_get_ep_mapping(i);
			int cons_idx = ipa3_get_ep_mapping(j);

			if (prod_idx == -1 ||
				prod_idx >= ipa3_get_max_num_pipes())
				continue;

			if (cons_idx == -1 ||
				cons_idx >= ipa3_get_max_num_pipes())
				continue;

			prod_reg = ipahal_get_ep_reg_idx(prod_idx);
			cons_reg = ipahal_get_ep_reg_idx(cons_idx);

			/* save hw-query result */
			if ((init->prod_bitmask[prod_reg] &
				ipahal_get_ep_bit(prod_idx)) &&
				(init->cons_bitmask[prod_idx][cons_reg]
					& ipahal_get_ep_bit(cons_idx))) {
				IPADBG_LOW("prod %d cons %d\n",
					prod_idx, cons_idx);
				stats = &stats_all->stats[prod_idx][cons_idx];
				IPADBG_LOW("num_ipv4_bytes %lld\n",
					stats->num_ipv4_bytes);
				IPADBG_LOW("num_ipv4_pkts %lld\n",
					stats->num_ipv4_pkts);
				IPADBG_LOW("num_ipv6_pkts %lld\n",
					stats->num_ipv6_pkts);
				IPADBG_LOW("num_ipv6_bytes %lld\n",
					stats->num_ipv6_bytes);

				/* update stats*/
				quota_stats =
					&sw_stats->prod_stats[i].client[j];
				quota_stats->num_ipv4_bytes =
					stats->num_ipv4_bytes;
				quota_stats->num_ipv4_pkts =
					stats->num_ipv4_pkts;
				quota_stats->num_ipv6_bytes =
					stats->num_ipv6_bytes;
				quota_stats->num_ipv6_pkts =
					stats->num_ipv6_pkts;

				/* Accumulated stats */
				quota_stats =
					&sw_stats->prod_stats_sum[i].client[j];
				quota_stats->num_ipv4_bytes +=
					stats->num_ipv4_bytes;
				quota_stats->num_ipv4_pkts +=
					stats->num_ipv4_pkts;
				quota_stats->num_ipv6_bytes +=
					stats->num_ipv6_bytes;
				quota_stats->num_ipv6_pkts +=
					stats->num_ipv6_pkts;
			}
		}
	}

	ret = 0;
free_stats:
	kfree(stats_all);
	stats = NULL;
destroy_imm:
	for (i = 0; i < num_cmd; i++)
		ipahal_destroy_imm_cmd(cmd_pyld[i]);
free_dma_mem:
	dma_free_coherent(ipa3_ctx->pdev, mem.size, mem.base, mem.phys_base);
	return ret;

}

int ipa_query_teth_stats(enum ipa_client_type prod,
	struct ipa_quota_stats_all *out, bool reset)
{
	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled &&
		ipa3_ctx->hw_stats->teth_stats_enabled))
		return 0;

	if (!IPA_CLIENT_IS_PROD(prod) || ipa3_get_ep_mapping(prod) == -1) {
		IPAERR("invalid prod %d\n", prod);
		return -EINVAL;
	}

	/* copy results to out parameter */
	if (reset)
		*out = ipa3_ctx->hw_stats->teth.prod_stats[prod];
	else
		*out = ipa3_ctx->hw_stats->teth.prod_stats_sum[prod];
	return 0;
}

int ipa_reset_teth_stats(enum ipa_client_type prod, enum ipa_client_type cons)
{
	int ret;
	struct ipa_quota_stats *stats;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled &&
		ipa3_ctx->hw_stats->teth_stats_enabled))
		return 0;

	if (!IPA_CLIENT_IS_PROD(prod) || !IPA_CLIENT_IS_CONS(cons)) {
		IPAERR("invalid prod %d or cons %d\n", prod, cons);
		return -EINVAL;
	}

	/* reading stats will reset them in hardware */
	ret = ipa_get_teth_stats();
	if (ret) {
		IPAERR("ipa_get_teth_stats failed %d\n", ret);
		return ret;
	}

	/* reset driver's cache */
	stats = &ipa3_ctx->hw_stats->teth.prod_stats_sum[prod].client[cons];
	memset(stats, 0, sizeof(*stats));
	return 0;
}

int ipa_reset_all_cons_teth_stats(enum ipa_client_type prod)
{
	int ret;
	int i;
	struct ipa_quota_stats *stats;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled &&
		ipa3_ctx->hw_stats->teth_stats_enabled))
		return 0;

	if (!IPA_CLIENT_IS_PROD(prod)) {
		IPAERR("invalid prod %d\n", prod);
		return -EINVAL;
	}

	/* reading stats will reset them in hardware */
	ret = ipa_get_teth_stats();
	if (ret) {
		IPAERR("ipa_get_teth_stats failed %d\n", ret);
		return ret;
	}

	/* reset driver's cache */
	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		stats = &ipa3_ctx->hw_stats->teth.prod_stats_sum[prod].client[i];
		memset(stats, 0, sizeof(*stats));
	}

	return 0;
}

int ipa_reset_all_teth_stats(void)
{
	int i;
	int ret;
	struct ipa_quota_stats_all *stats;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled &&
		ipa3_ctx->hw_stats->teth_stats_enabled))
		return 0;

	/* reading stats will reset them in hardware */
	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		if (IPA_CLIENT_IS_PROD(i) && ipa3_get_ep_mapping(i) != -1) {
			ret = ipa_get_teth_stats();
			if (ret) {
				IPAERR("ipa_get_teth_stats failed %d\n", ret);
				return ret;
			}
			/* a single iteration will reset all hardware stats */
			break;
		}
	}

	/* reset driver's cache */
	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		stats = &ipa3_ctx->hw_stats->teth.prod_stats_sum[i];
		memset(stats, 0, sizeof(*stats));
	}

	return 0;
}

int ipa_init_flt_rt_stats(void)
{
	struct ipahal_stats_init_pyld *pyld;
	int smem_ofst, smem_size;
	int stats_base_flt_v4, stats_base_flt_v6;
	int stats_base_rt_v4, stats_base_rt_v6;
	struct ipahal_imm_cmd_dma_shared_mem cmd = { 0 };
	struct ipahal_imm_cmd_pyld *cmd_pyld;
	struct ipahal_imm_cmd_register_write flt_v4_base = {0};
	struct ipahal_imm_cmd_pyld *flt_v4_base_pyld;
	struct ipahal_imm_cmd_register_write flt_v6_base = {0};
	struct ipahal_imm_cmd_pyld *flt_v6_base_pyld;
	struct ipahal_imm_cmd_register_write rt_v4_base = {0};
	struct ipahal_imm_cmd_pyld *rt_v4_base_pyld;
	struct ipahal_imm_cmd_register_write rt_v6_base = {0};
	struct ipahal_imm_cmd_pyld *rt_v6_base_pyld;
	struct ipahal_imm_cmd_pyld *coal_cmd_pyld = NULL;
	struct ipa3_desc desc[6] = { {0} };
	dma_addr_t dma_address;
	int ret;
	int num_cmd = 0;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	smem_ofst = IPA_MEM_PART(stats_fnr_ofst);
	smem_size = IPA_MEM_PART(stats_fnr_size);

	pyld = ipahal_stats_generate_init_pyld(IPAHAL_HW_STATS_FNR,
		(void *)(uintptr_t)(IPA_MAX_FLT_RT_CNT_INDEX), false);
	if (!pyld) {
		IPAERR("failed to generate pyld\n");
		return -EPERM;
	}

	if (pyld->len > smem_size) {
		IPAERR("SRAM partition too small: %d needed %d\n",
			smem_size, pyld->len);
		ret = -EPERM;
		goto destroy_init_pyld;
	}

	dma_address = dma_map_single(ipa3_ctx->pdev,
		pyld->data,
		pyld->len,
		DMA_TO_DEVICE);
	if (dma_mapping_error(ipa3_ctx->pdev, dma_address)) {
		IPAERR("failed to DMA map\n");
		ret = -EPERM;
		goto destroy_init_pyld;
	}

	/* IC to close the coal frame before HPS Clear if coal is enabled */
	if (ipa3_get_ep_mapping(IPA_CLIENT_APPS_WAN_COAL_CONS) !=
		IPA_EP_NOT_ALLOCATED && !ipa3_ctx->ulso_wa) {
		ipa_close_coal_frame(&coal_cmd_pyld);
		if (!coal_cmd_pyld) {
			IPAERR("failed to construct coal close IC\n");
			ret = -ENOMEM;
			goto unmap;
		}
		ipa3_init_imm_cmd_desc(&desc[num_cmd], coal_cmd_pyld);
		++num_cmd;
	}

	stats_base_flt_v4 = ipahal_get_reg_ofst(IPA_STAT_FILTER_IPV4_BASE);
	stats_base_flt_v6 = ipahal_get_reg_ofst(IPA_STAT_FILTER_IPV6_BASE);
	stats_base_rt_v4 = ipahal_get_reg_ofst(IPA_STAT_ROUTER_IPV4_BASE);
	stats_base_rt_v6 = ipahal_get_reg_ofst(IPA_STAT_ROUTER_IPV6_BASE);

	/* setting the registers and init the stats pyld are done atomically */
	/* set IPA_STAT_FILTER_IPV4_BASE */
	flt_v4_base.skip_pipeline_clear = false;
	flt_v4_base.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	flt_v4_base.offset = stats_base_flt_v4;
	flt_v4_base.value = ipa3_ctx->smem_restricted_bytes +
		smem_ofst;
	flt_v4_base.value_mask = ~0;
	flt_v4_base_pyld = ipahal_construct_imm_cmd(IPA_IMM_CMD_REGISTER_WRITE,
		&flt_v4_base, false);
	if (!flt_v4_base_pyld) {
		IPAERR("failed to construct register_write imm cmd\n");
		ret = -ENOMEM;
		goto destroy_coal_cmd;
	}
	desc[num_cmd].opcode = flt_v4_base_pyld->opcode;
	desc[num_cmd].pyld = flt_v4_base_pyld->data;
	desc[num_cmd].len = flt_v4_base_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	++num_cmd;

	/* set IPA_STAT_FILTER_IPV6_BASE */
	flt_v6_base.skip_pipeline_clear = false;
	flt_v6_base.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	flt_v6_base.offset = stats_base_flt_v6;
	flt_v6_base.value = ipa3_ctx->smem_restricted_bytes +
		smem_ofst;
	flt_v6_base.value_mask = ~0;
	flt_v6_base_pyld = ipahal_construct_imm_cmd(IPA_IMM_CMD_REGISTER_WRITE,
		&flt_v6_base, false);
	if (!flt_v6_base_pyld) {
		IPAERR("failed to construct register_write imm cmd\n");
		ret = -ENOMEM;
		goto destroy_flt_v4_base;
	}
	desc[num_cmd].opcode = flt_v6_base_pyld->opcode;
	desc[num_cmd].pyld = flt_v6_base_pyld->data;
	desc[num_cmd].len = flt_v6_base_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	++num_cmd;

	/* set IPA_STAT_ROUTER_IPV4_BASE */
	rt_v4_base.skip_pipeline_clear = false;
	rt_v4_base.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	rt_v4_base.offset = stats_base_rt_v4;
	rt_v4_base.value = ipa3_ctx->smem_restricted_bytes +
		smem_ofst;
	rt_v4_base.value_mask = ~0;
	rt_v4_base_pyld = ipahal_construct_imm_cmd(IPA_IMM_CMD_REGISTER_WRITE,
		&rt_v4_base, false);
	if (!rt_v4_base_pyld) {
		IPAERR("failed to construct register_write imm cmd\n");
		ret = -ENOMEM;
		goto destroy_flt_v6_base;
	}
	desc[num_cmd].opcode = rt_v4_base_pyld->opcode;
	desc[num_cmd].pyld = rt_v4_base_pyld->data;
	desc[num_cmd].len = rt_v4_base_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	++num_cmd;

	/* set IPA_STAT_ROUTER_IPV6_BASE */
	rt_v6_base.skip_pipeline_clear = false;
	rt_v6_base.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	rt_v6_base.offset = stats_base_rt_v6;
	rt_v6_base.value = ipa3_ctx->smem_restricted_bytes +
		smem_ofst;
	rt_v6_base.value_mask = ~0;
	rt_v6_base_pyld = ipahal_construct_imm_cmd(IPA_IMM_CMD_REGISTER_WRITE,
		&rt_v6_base, false);
	if (!rt_v6_base_pyld) {
		IPAERR("failed to construct register_write imm cmd\n");
		ret = -ENOMEM;
		goto destroy_rt_v4_base;
	}
	desc[num_cmd].opcode = rt_v6_base_pyld->opcode;
	desc[num_cmd].pyld = rt_v6_base_pyld->data;
	desc[num_cmd].len = rt_v6_base_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	++num_cmd;

	cmd.is_read = false;
	cmd.skip_pipeline_clear = false;
	cmd.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	cmd.size = pyld->len;
	cmd.system_addr = dma_address;
	cmd.local_addr = ipa3_ctx->smem_restricted_bytes +
			smem_ofst;
	cmd_pyld = ipahal_construct_imm_cmd(
		IPA_IMM_CMD_DMA_SHARED_MEM, &cmd, false);
	if (!cmd_pyld) {
		IPAERR("failed to construct dma_shared_mem imm cmd\n");
		ret = -ENOMEM;
		goto destroy_rt_v6_base;
	}
	desc[num_cmd].opcode = cmd_pyld->opcode;
	desc[num_cmd].pyld = cmd_pyld->data;
	desc[num_cmd].len = cmd_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	++num_cmd;

	ret = ipa3_send_cmd(num_cmd, desc);
	if (ret) {
		IPAERR("failed to send immediate command (error %d)\n", ret);
		goto destroy_imm;
	}

	ret = 0;

destroy_imm:
	ipahal_destroy_imm_cmd(cmd_pyld);
destroy_rt_v6_base:
	ipahal_destroy_imm_cmd(rt_v6_base_pyld);
destroy_rt_v4_base:
	ipahal_destroy_imm_cmd(rt_v4_base_pyld);
destroy_flt_v6_base:
	ipahal_destroy_imm_cmd(flt_v6_base_pyld);
destroy_flt_v4_base:
	ipahal_destroy_imm_cmd(flt_v4_base_pyld);
destroy_coal_cmd:
	if (coal_cmd_pyld)
		ipahal_destroy_imm_cmd(coal_cmd_pyld);
unmap:
	dma_unmap_single(ipa3_ctx->pdev, dma_address, pyld->len, DMA_TO_DEVICE);
destroy_init_pyld:
	ipahal_destroy_stats_init_pyld(pyld);
	return ret;
}

static int __ipa_get_flt_rt_stats(struct ipa_ioc_flt_rt_query *query)
{
	int ret;
	int smem_ofst;
	bool clear = query->reset;
	struct ipahal_stats_get_offset_flt_rt_v4_5 *get_offset;
	struct ipahal_stats_offset offset = { 0 };
	struct ipahal_imm_cmd_dma_shared_mem cmd = { 0 };
	struct ipahal_imm_cmd_pyld *cmd_pyld[2];
	struct ipa_mem_buffer mem;
	struct ipa3_desc desc[2];
	int num_cmd = 0;
	int i;

	memset(desc, 0, sizeof(desc));
	memset(cmd_pyld, 0, sizeof(cmd_pyld));

	get_offset = kzalloc(sizeof(*get_offset), GFP_KERNEL);
	if (!get_offset) {
		IPADBG("no mem\n");
		return -ENOMEM;
	}

	smem_ofst = IPA_MEM_PART(stats_fnr_ofst);

	get_offset->start_id = query->start_id;
	get_offset->end_id = query->end_id;

	ret = ipahal_stats_get_offset(IPAHAL_HW_STATS_FNR, get_offset,
		&offset);
	if (ret) {
		IPAERR("failed to get offset from hal %d\n", ret);
		goto free_offset;
	}

	IPADBG("offset = %d size = %d\n", offset.offset, offset.size);

	if (offset.size == 0) {
		ret = 0;
		goto free_offset;
	}

	mem.size = offset.size;
	mem.base = dma_alloc_coherent(ipa3_ctx->pdev,
		mem.size,
		&mem.phys_base,
		GFP_KERNEL);
	if (!mem.base) {
		IPAERR("fail to alloc DMA memory\n");
		goto free_offset;
	}

	/* IC to close the coal frame before HPS Clear if coal is enabled */
	if (ipa3_get_ep_mapping(IPA_CLIENT_APPS_WAN_COAL_CONS) !=
		IPA_EP_NOT_ALLOCATED && !ipa3_ctx->ulso_wa) {
		ipa_close_coal_frame(&cmd_pyld[num_cmd]);
		if (!cmd_pyld[num_cmd]) {
			IPAERR("failed to construct coal close IC\n");
			ret = -ENOMEM;
			goto free_dma_mem;
		}
		ipa3_init_imm_cmd_desc(&desc[num_cmd], cmd_pyld[num_cmd]);
		++num_cmd;
	}

	cmd.is_read = true;
	cmd.clear_after_read = clear;
	cmd.skip_pipeline_clear = false;
	cmd.pipeline_clear_options = IPAHAL_HPS_CLEAR;
	cmd.size = mem.size;
	cmd.system_addr = mem.phys_base;
	cmd.local_addr = ipa3_ctx->smem_restricted_bytes +
		smem_ofst + offset.offset;
	cmd_pyld[num_cmd] = ipahal_construct_imm_cmd(
		IPA_IMM_CMD_DMA_SHARED_MEM, &cmd, false);
	if (!cmd_pyld[num_cmd]) {
		IPAERR("failed to construct dma_shared_mem imm cmd\n");
		ret = -ENOMEM;
		goto destroy_imm;
	}
	ipa3_init_imm_cmd_desc(&desc[num_cmd], cmd_pyld[num_cmd]);
	++num_cmd;

	ret = ipa3_send_cmd(num_cmd, desc);
	if (ret) {
		IPAERR("failed to send immediate command (error %d)\n", ret);
		goto destroy_imm;
	}

	ret = ipahal_parse_stats(IPAHAL_HW_STATS_FNR,
		NULL, mem.base, query);
	if (ret) {
		IPAERR("failed to parse stats (error %d)\n", ret);
		goto destroy_imm;
	}
	ret = 0;

destroy_imm:
	for (i = 0; i < num_cmd; i++)
		ipahal_destroy_imm_cmd(cmd_pyld[i]);
free_dma_mem:
	dma_free_coherent(ipa3_ctx->pdev, mem.size, mem.base, mem.phys_base);
free_offset:
	kfree(get_offset);
	return ret;
}

int ipa_get_flt_rt_stats(struct ipa_ioc_flt_rt_query *query)
{
	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled)) {
		IPAERR("hw_stats is not enabled\n");
		return 0;
	}

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5) {
		IPAERR("FnR stats not supported in %d hw_type\n",
			ipa3_ctx->ipa_hw_type);
		return 0;
	}

	if (query->start_id == 0 || query->end_id == 0) {
		IPAERR("Invalid start_id/end_id, must be not 0\n");
		IPAERR("start_id %d, end_id %d\n",
			query->start_id, query->end_id);
		return -EINVAL;
	}

	if (query->start_id > IPA_MAX_FLT_RT_CNT_INDEX) {
		IPAERR("start_cnt_id %d out of range\n", query->start_id);
		return -EINVAL;
	}

	if (query->end_id > IPA_MAX_FLT_RT_CNT_INDEX) {
		IPAERR("end_cnt_id %d out of range\n", query->end_id);
		return -EINVAL;
	}

	if (query->end_id < query->start_id) {
		IPAERR("end_id %d < start_id %d\n",
			query->end_id, query->start_id);
		return -EINVAL;
	}

	if (query->stats_size > sizeof(struct ipa_flt_rt_stats)) {
		IPAERR("stats_size %d > ipa_flt_rt_stats %d\n",
			query->stats_size, sizeof(struct ipa_flt_rt_stats));
		return -EINVAL;
	}

	return __ipa_get_flt_rt_stats(query);
}


static int __ipa_set_flt_rt_stats(int index, struct ipa_flt_rt_stats stats)
{
	int ret;
	int smem_ofst;
	struct ipahal_stats_get_offset_flt_rt_v4_5 *get_offset;
	struct ipahal_stats_offset offset = { 0 };
	struct ipahal_imm_cmd_dma_shared_mem cmd = { 0 };
	struct ipahal_imm_cmd_pyld *cmd_pyld;
	struct ipa_mem_buffer mem;
	struct ipa3_desc desc = { 0 };

	get_offset = kzalloc(sizeof(*get_offset), GFP_KERNEL);
	if (!get_offset) {
		IPADBG("no mem\n");
		return -ENOMEM;
	}

	smem_ofst = IPA_MEM_PART(stats_fnr_ofst);

	get_offset->start_id = index;
	get_offset->end_id = index;

	ret = ipahal_stats_get_offset(IPAHAL_HW_STATS_FNR, get_offset,
		&offset);
	if (ret) {
		IPAERR("failed to get offset from hal %d\n", ret);
		goto free_offset;
	}

	IPADBG("offset = %d size = %d\n", offset.offset, offset.size);

	if (offset.size == 0) {
		ret = 0;
		goto free_offset;
	}

	mem.size = offset.size;
	mem.base = dma_alloc_coherent(ipa3_ctx->pdev,
		mem.size,
		&mem.phys_base,
		GFP_KERNEL);
	if (!mem.base) {
		IPAERR("fail to alloc DMA memory\n");
		goto free_offset;
	}
	ipahal_set_flt_rt_sw_stats(mem.base, stats);

	cmd.is_read = false;
	cmd.skip_pipeline_clear = false;
	cmd.pipeline_clear_options = IPAHAL_HPS_CLEAR;
	cmd.size = mem.size;
	cmd.system_addr = mem.phys_base;
	cmd.local_addr = ipa3_ctx->smem_restricted_bytes +
		smem_ofst + offset.offset;
	cmd_pyld = ipahal_construct_imm_cmd(
		IPA_IMM_CMD_DMA_SHARED_MEM, &cmd, false);
	if (!cmd_pyld) {
		IPAERR("failed to construct dma_shared_mem imm cmd\n");
		ret = -ENOMEM;
		goto free_dma_mem;
	}
	desc.opcode = cmd_pyld->opcode;
	desc.pyld = cmd_pyld->data;
	desc.len = cmd_pyld->len;
	desc.type = IPA_IMM_CMD_DESC;

	ret = ipa3_send_cmd(1, &desc);
	if (ret) {
		IPAERR("failed to send immediate command (error %d)\n", ret);
		goto destroy_imm;
	}

	ret = 0;

destroy_imm:
	ipahal_destroy_imm_cmd(cmd_pyld);
free_dma_mem:
	dma_free_coherent(ipa3_ctx->pdev, mem.size, mem.base, mem.phys_base);
free_offset:
	kfree(get_offset);
	return ret;
}

int ipa_set_flt_rt_stats(int index, struct ipa_flt_rt_stats stats)
{
	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled)) {
		IPAERR("hw_stats is not enabled\n");
		return 0;
	}

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5) {
		IPAERR("FnR stats not supported in %d hw_type\n",
			ipa3_ctx->ipa_hw_type);
		return 0;
	}

	if (index > IPA_MAX_FLT_RT_CNT_INDEX) {
		IPAERR("index %d out of range\n", index);
		return -EINVAL;
	}

	if (index <= IPA_FLT_RT_HW_COUNTER) {
		IPAERR("index %d invalid, only support sw counter set\n",
			index);
		return -EINVAL;
	}

	return __ipa_set_flt_rt_stats(index, stats);
}

int ipa_drop_stats_init(void)
{
	u32 reg_idx;
	u32 mask, pipe_bitmask[IPA_EP_ARR_SIZE] = {0};

	mask = ipa_hw_stats_get_ep_bit_n_idx(
		IPA_CLIENT_USB_CONS,
		&reg_idx);
	pipe_bitmask[reg_idx] |= mask;

	if (ipa3_ctx->ipa_wdi3_5g_holb_timeout || ipa3_ctx->uc_ctx.ipa_use_uc_holb_monitor) {
		mask = ipa_hw_stats_get_ep_bit_n_idx(
			IPA_CLIENT_WLAN2_CONS,
			&reg_idx);
		pipe_bitmask[reg_idx] |= mask;
	}

	if (ipa3_ctx->platform_type == IPA_PLAT_TYPE_MDM) {
		if (ipa3_ctx->ipa_wdi3_2g_holb_timeout) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_WLAN2_CONS1,
				&reg_idx);
			pipe_bitmask[reg_idx] |= mask;
		}

		if (ipa3_ctx->use_tput_est_ep) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_TPUT_CONS,
				&reg_idx);
			pipe_bitmask[reg_idx] |= mask;

		}
	} else {
		/* ADPL pipe hw stats is now taken care by IPA Q6 */
		if (ipa3_ctx->ipa_hw_type < IPA_HW_v5_0) {
			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_USB_DPL_CONS,
				&reg_idx);
			pipe_bitmask[reg_idx] |= mask;

			mask = ipa_hw_stats_get_ep_bit_n_idx(
				IPA_CLIENT_ODL_DPL_CONS,
				&reg_idx);
			pipe_bitmask[reg_idx] |= mask;
		}
	}

	/* Currently we have option to enable drop stats using debugfs.
	 * To enable drop stats for a different pipe, first user needs
	 * to query drop stats to get the current stats and enable.
	 * TODO: to support dynamically caching drop stats.
	 */

	return ipa_init_drop_stats(pipe_bitmask);
}

int ipa_init_drop_stats(u32 *pipe_bitmask)
{
	struct ipahal_stats_init_pyld *pyld;
	struct ipahal_imm_cmd_dma_shared_mem cmd = { 0 };
	struct ipahal_imm_cmd_pyld *cmd_pyld;
	struct ipahal_imm_cmd_register_write drop_base = {0};
	struct ipahal_imm_cmd_pyld *drop_base_pyld;
	struct ipahal_imm_cmd_register_write drop_mask = {0};
	struct ipahal_imm_cmd_pyld *drop_mask_pyld[IPAHAL_IPA5_PIPE_REG_NUM] =
		{0};
	struct ipahal_imm_cmd_pyld *coal_cmd_pyld = NULL;
	struct ipa3_desc *desc = NULL;
	struct ipa_hw_stats_drop tmp_drop;
	dma_addr_t dma_address;
	int ret, i;
	int num_cmd = 0;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	if (!pipe_bitmask)
		return -EPERM;

	desc = kzalloc(sizeof(*desc) * IPA_INIT_DROP_STATS_MAX_CMD_NUM, GFP_KERNEL);
	if (!desc) {
		IPAERR("failed to allocate memory\n");
		return -ENOMEM;
	}

	/* check if IPA has enough space for # of pipes drop stats enabled*/
	memset(&tmp_drop, 0, sizeof(tmp_drop));
	for (i = 0; i < IPA5_PIPE_REG_NUM; i++) {
		tmp_drop.init.enabled_bitmask[i] = pipe_bitmask[i];
		IPADBG_LOW("pipe_bitmask[%d]=0x%x\n", i, pipe_bitmask[i]);
	}

	pyld = ipahal_stats_generate_init_pyld(IPAHAL_HW_STATS_DROP,
		&tmp_drop.init, false);
	if (!pyld) {
		IPAERR("failed to generate pyld\n");
		ret = -EPERM;
		goto fail_free_desc;
	}

	if (pyld->len > IPA_MEM_PART(stats_drop_size)) {
		IPAERR("SRAM partition too small: %d bytes (%d pipes)."
			"Tried to add %d bytes (%d pipes)."
			"Please disable some stats before adding new ones.\n",
			IPA_MEM_PART(stats_drop_size), IPA_MEM_PART(stats_drop_size)/8,
			pyld->len, pyld->len/8);
		ret = -EPERM;
		goto destroy_init_pyld;
	}

	/* reset driver's cache and copy the bitmask of new drop enabled pipes */
	memset(&ipa3_ctx->hw_stats->drop, 0, sizeof(ipa3_ctx->hw_stats->drop));
	ipa3_ctx->hw_stats->drop = tmp_drop;

	dma_address = dma_map_single(ipa3_ctx->pdev,
		pyld->data,
		pyld->len,
		DMA_TO_DEVICE);
	if (dma_mapping_error(ipa3_ctx->pdev, dma_address)) {
		IPAERR("failed to DMA map\n");
		ret = -EPERM;
		goto destroy_init_pyld;
	}

	/* IC to close the coal frame before HPS Clear if coal is enabled */
	if (ipa3_get_ep_mapping(IPA_CLIENT_APPS_WAN_COAL_CONS) !=
		IPA_EP_NOT_ALLOCATED && !ipa3_ctx->ulso_wa) {
		ipa_close_coal_frame(&coal_cmd_pyld);
		if (!coal_cmd_pyld) {
			IPAERR("failed to construct coal close IC\n");
			ret = -ENOMEM;
			goto unmap;
		}
		ipa3_init_imm_cmd_desc(&desc[num_cmd], coal_cmd_pyld);
		++num_cmd;
	}

	/* setting the registers and init the stats pyld are done atomically */
	drop_mask.skip_pipeline_clear = false;
	drop_mask.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	drop_mask.value_mask = ~0;
	if (ipa3_ctx->ipa_hw_type < IPA_HW_v5_0) {
		drop_mask.offset = ipahal_get_reg_n_ofst(
			IPA_STAT_DROP_CNT_MASK_n,
			ipa3_ctx->ee);
		drop_mask.value = pipe_bitmask[0];
		drop_mask_pyld[0] = ipahal_construct_imm_cmd(
			IPA_IMM_CMD_REGISTER_WRITE,
			&drop_mask, false);
		if (!drop_mask_pyld[0]) {
			IPAERR("failed to construct register_write imm cmd\n");
			ret = -ENOMEM;
			goto destroy_coal_cmd;
		}
		desc[num_cmd].opcode = drop_mask_pyld[0]->opcode;
		desc[num_cmd].pyld = drop_mask_pyld[0]->data;
		desc[num_cmd].len = drop_mask_pyld[0]->len;
		desc[num_cmd].type = IPA_IMM_CMD_DESC;
		++num_cmd;
	} else {
		for (i = 0; i < IPA5_PIPE_REG_NUM; i++) {
			drop_mask.offset = ipahal_get_reg_nk_offset(
				IPA_STAT_DROP_CNT_MASK_EE_n_REG_k,
				ipa3_ctx->ee, i);
			drop_mask.value = pipe_bitmask[i];
			drop_mask_pyld[i] = ipahal_construct_imm_cmd(
				IPA_IMM_CMD_REGISTER_WRITE,
				&drop_mask, false);
			if (!drop_mask_pyld[i]) {
				int j;

				IPAERR(
					"failed to construct register_write imm cmd\n"
				);
				for (j = i - 1; j >= 0; j--)
					ipahal_destroy_imm_cmd(
						drop_mask_pyld[j]);
				ret = -ENOMEM;
				goto destroy_coal_cmd;
			}
			desc[num_cmd].opcode = drop_mask_pyld[i]->opcode;
			desc[num_cmd].pyld = drop_mask_pyld[i]->data;
			desc[num_cmd].len = drop_mask_pyld[i]->len;
			desc[num_cmd].type = IPA_IMM_CMD_DESC;
			++num_cmd;
		}
	}

	drop_base.skip_pipeline_clear = false;
	drop_base.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	drop_base.offset = ipahal_get_reg_n_ofst(IPA_STAT_DROP_CNT_BASE_n,
		ipa3_ctx->ee);
	drop_base.value = ipa3_ctx->smem_restricted_bytes +
		IPA_MEM_PART(stats_drop_ofst);
	drop_base.value_mask = ~0;
	drop_base_pyld = ipahal_construct_imm_cmd(IPA_IMM_CMD_REGISTER_WRITE,
		&drop_base, false);
	if (!drop_base_pyld) {
		IPAERR("failed to construct register_write imm cmd\n");
		ret = -ENOMEM;
		goto destroy_drop_mask;
	}
	desc[num_cmd].opcode = drop_base_pyld->opcode;
	desc[num_cmd].pyld = drop_base_pyld->data;
	desc[num_cmd].len = drop_base_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	++num_cmd;

	cmd.is_read = false;
	cmd.skip_pipeline_clear = false;
	cmd.pipeline_clear_options = IPAHAL_FULL_PIPELINE_CLEAR;
	cmd.size = pyld->len;
	cmd.system_addr = dma_address;
	cmd.local_addr = ipa3_ctx->smem_restricted_bytes +
			IPA_MEM_PART(stats_drop_ofst);
	cmd_pyld = ipahal_construct_imm_cmd(
		IPA_IMM_CMD_DMA_SHARED_MEM, &cmd, false);
	if (!cmd_pyld) {
		IPAERR("failed to construct dma_shared_mem imm cmd\n");
		ret = -ENOMEM;
		goto destroy_drop_base;
	}
	desc[num_cmd].opcode = cmd_pyld->opcode;
	desc[num_cmd].pyld = cmd_pyld->data;
	desc[num_cmd].len = cmd_pyld->len;
	desc[num_cmd].type = IPA_IMM_CMD_DESC;
	++num_cmd;

	ret = ipa3_send_cmd(num_cmd, desc);
	if (ret) {
		IPAERR("failed to send immediate command (error %d)\n", ret);
		goto destroy_imm;
	}

	ret = 0;

destroy_imm:
	ipahal_destroy_imm_cmd(cmd_pyld);
destroy_drop_base:
	ipahal_destroy_imm_cmd(drop_base_pyld);
destroy_drop_mask:
	for (i = 0; i < IPA5_PIPE_REG_NUM; i++)
		if (drop_mask_pyld[i])
			ipahal_destroy_imm_cmd(drop_mask_pyld[i]);
destroy_coal_cmd:
	if (coal_cmd_pyld)
		ipahal_destroy_imm_cmd(coal_cmd_pyld);
unmap:
	dma_unmap_single(ipa3_ctx->pdev, dma_address, pyld->len, DMA_TO_DEVICE);
destroy_init_pyld:
	ipahal_destroy_stats_init_pyld(pyld);
fail_free_desc:
		kfree(desc);
	return ret;
}

int ipa_get_drop_stats(struct ipa_drop_stats_all *out)
{
	int i;
	int ret;
	struct ipahal_stats_get_offset_drop get_offset = { { 0 } };
	struct ipahal_stats_offset offset = { 0 };
	struct ipahal_imm_cmd_dma_shared_mem cmd = { 0 };
	struct ipahal_imm_cmd_pyld *cmd_pyld[2];
	struct ipa_mem_buffer mem;
	struct ipa3_desc desc[2];
	struct ipahal_stats_drop_all *stats;
	int num_cmd = 0;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	memset(desc, 0, sizeof(desc));
	memset(cmd_pyld, 0, sizeof(cmd_pyld));

	get_offset.init = ipa3_ctx->hw_stats->drop.init;
	ret = ipahal_stats_get_offset(IPAHAL_HW_STATS_DROP, &get_offset,
		&offset);
	if (ret) {
		IPAERR("failed to get offset from hal %d\n", ret);
		return ret;
	}

	IPADBG_LOW("offset = %d size = %d\n", offset.offset, offset.size);

	if (offset.size == 0)
		return 0;

	mem.size = offset.size;
	mem.base = dma_alloc_coherent(ipa3_ctx->pdev,
		mem.size,
		&mem.phys_base,
		GFP_KERNEL);
	if (!mem.base) {
		IPAERR("fail to alloc DMA memory\n");
		return ret;
	}

	/* IC to close the coal frame before HPS Clear if coal is enabled */
	if (ipa3_get_ep_mapping(IPA_CLIENT_APPS_WAN_COAL_CONS) !=
		IPA_EP_NOT_ALLOCATED && !ipa3_ctx->ulso_wa) {
		ipa_close_coal_frame(&cmd_pyld[num_cmd]);
		if (!cmd_pyld[num_cmd]) {
			IPAERR("failed to construct coal close IC\n");
			ret = -ENOMEM;
			goto free_dma_mem;
		}
		ipa3_init_imm_cmd_desc(&desc[num_cmd], cmd_pyld[num_cmd]);
		++num_cmd;
	}

	cmd.is_read = true;
	cmd.clear_after_read = true;
	cmd.skip_pipeline_clear = false;
	cmd.pipeline_clear_options = IPAHAL_HPS_CLEAR;
	cmd.size = mem.size;
	cmd.system_addr = mem.phys_base;
	cmd.local_addr = ipa3_ctx->smem_restricted_bytes +
		IPA_MEM_PART(stats_drop_ofst) + offset.offset;
	cmd_pyld[num_cmd] = ipahal_construct_imm_cmd(
		IPA_IMM_CMD_DMA_SHARED_MEM, &cmd, false);
	if (!cmd_pyld[num_cmd]) {
		IPAERR("failed to construct dma_shared_mem imm cmd\n");
		ret = -ENOMEM;
		goto destroy_imm;
	}
	ipa3_init_imm_cmd_desc(&desc[num_cmd], cmd_pyld[num_cmd]);
	++num_cmd;

	ret = ipa3_send_cmd(num_cmd, desc);
	if (ret) {
		IPAERR("failed to send immediate command (error %d)\n", ret);
		goto destroy_imm;
	}

	stats = kzalloc(sizeof(*stats), GFP_KERNEL);
	if (!stats) {
		ret = -ENOMEM;
		goto destroy_imm;
	}

	ret = ipahal_parse_stats(IPAHAL_HW_STATS_DROP,
		&ipa3_ctx->hw_stats->drop.init, mem.base, stats);
	if (ret) {
		IPAERR("failed to parse stats (error %d)\n", ret);
		goto free_stats;
	}

	/*
	 * update driver cache.
	 * the stats were read from hardware with clear_after_read meaning
	 * hardware stats are 0 now
	 */
	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		int ep_idx = ipa3_get_ep_mapping(i);

		if (ep_idx == -1 || ep_idx >= ipa3_get_max_num_pipes())
			continue;

		if (ipa3_ctx->ep[ep_idx].client != i)
			continue;

		ipa3_ctx->hw_stats->drop.stats.client[i].drop_byte_cnt +=
			stats->stats[ep_idx].drop_byte_cnt;
		ipa3_ctx->hw_stats->drop.stats.client[i].drop_packet_cnt +=
			stats->stats[ep_idx].drop_packet_cnt;
	}


	if (!out) {
		ret = 0;
		goto free_stats;
	}

	/* copy results to out parameter */
	*out = ipa3_ctx->hw_stats->drop.stats;

	ret = 0;
free_stats:
	kfree(stats);
destroy_imm:
	for (i = 0; i < num_cmd; i++)
		ipahal_destroy_imm_cmd(cmd_pyld[i]);
free_dma_mem:
	dma_free_coherent(ipa3_ctx->pdev, mem.size, mem.base, mem.phys_base);
	return ret;

}

int ipa_reset_drop_stats(enum ipa_client_type client)
{
	int ret;
	struct ipa_drop_stats *stats;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	if (client >= IPA_CLIENT_MAX) {
		IPAERR("invalid client %d\n", client);
		return -EINVAL;
	}

	/* reading stats will reset them in hardware */
	ret = ipa_get_drop_stats(NULL);
	if (ret) {
		IPAERR("ipa_get_drop_stats failed %d\n", ret);
		return ret;
	}

	/* reset driver's cache */
	stats = &ipa3_ctx->hw_stats->drop.stats.client[client];
	memset(stats, 0, sizeof(*stats));
	return 0;
}

int ipa_reset_all_drop_stats(void)
{
	int ret;
	struct ipa_drop_stats_all *stats;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	/* reading stats will reset them in hardware */
	ret = ipa_get_drop_stats(NULL);
	if (ret) {
		IPAERR("ipa_get_drop_stats failed %d\n", ret);
		return ret;
	}

	/* reset driver's cache */
	stats = &ipa3_ctx->hw_stats->drop.stats;
	memset(stats, 0, sizeof(*stats));
	return 0;
}


#ifndef CONFIG_DEBUG_FS
int ipa_debugfs_init_stats(struct dentry *parent) { return 0; }
#else
#define IPA_MAX_MSG_LEN 4096
static char dbg_buff[IPA_MAX_MSG_LEN];

static ssize_t ipa_debugfs_reset_quota_stats(struct file *file,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	s8 client = 0;
	int ret;

	mutex_lock(&ipa3_ctx->lock);

	ret = kstrtos8_from_user(ubuf, count, 0, &client);
	if (ret)
		goto bail;

	if (client == -1)
		ipa_reset_all_quota_stats();
	else
		ipa_reset_quota_stats(client);

	ret = count;
bail:
	mutex_unlock(&ipa3_ctx->lock);
	return ret;
}

static ssize_t ipa_debugfs_print_quota_stats(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes = 0;
	struct ipa_quota_stats_all *out;
	int i, reg_idx;
	int res;

	out = kzalloc(sizeof(*out), GFP_KERNEL);
	if (!out)
		return -ENOMEM;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	mutex_lock(&ipa3_ctx->lock);
	res = ipa_get_quota_stats(out);
	if (res) {
		mutex_unlock(&ipa3_ctx->lock);
		kfree(out);
		return res;
	}
	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		int ep_idx = ipa3_get_ep_mapping(i);

		if (ep_idx == -1)
			continue;

		if (IPA_CLIENT_IS_TEST(i))
			continue;

		reg_idx = ipahal_get_ep_reg_idx(ep_idx);
		if (!(ipa3_ctx->hw_stats->quota.init.enabled_bitmask[reg_idx] &
			ipahal_get_ep_bit(ep_idx)))
			continue;

		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"%s:\n",
			ipa_clients_strings[i]);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"num_ipv4_bytes=%llu\n",
			out->client[i].num_ipv4_bytes);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"num_ipv6_bytes=%llu\n",
			out->client[i].num_ipv6_bytes);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"num_ipv4_pkts=%u\n",
			out->client[i].num_ipv4_pkts);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"num_ipv6_pkts=%u\n",
			out->client[i].num_ipv6_pkts);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"\n");

	}
	mutex_unlock(&ipa3_ctx->lock);
	kfree(out);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa_debugfs_reset_tethering_stats(struct file *file,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	s8 client = 0;
	int ret;

	mutex_lock(&ipa3_ctx->lock);

	ret = kstrtos8_from_user(ubuf, count, 0, &client);
	if (ret)
		goto bail;

	if (client == -1)
		ipa_reset_all_teth_stats();
	else
		ipa_reset_all_cons_teth_stats(client);

	ret = count;
bail:
	mutex_unlock(&ipa3_ctx->lock);
	return ret;
}

static ssize_t ipa_debugfs_print_tethering_stats(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes = 0;
	struct ipa_quota_stats_all *out;
	int i, j, prod_reg, cons_reg;
	int res;

	out = kzalloc(sizeof(*out), GFP_KERNEL);
	if (!out)
		return -ENOMEM;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled &&
		ipa3_ctx->hw_stats->teth_stats_enabled))
		return 0;

	mutex_lock(&ipa3_ctx->lock);

	res = ipa_get_teth_stats();
	if (res) {
		mutex_unlock(&ipa3_ctx->lock);
		kfree(out);
		return res;
	}

	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		int ep_idx = ipa3_get_ep_mapping(i);

		if (ep_idx == -1)
			continue;

		if (!IPA_CLIENT_IS_PROD(i))
			continue;

		if (IPA_CLIENT_IS_TEST(i))
			continue;

		prod_reg = ipahal_get_ep_reg_idx(ep_idx);
		if (!(ipa3_ctx->hw_stats->teth.init.prod_bitmask[prod_reg] &
			ipahal_get_ep_bit(ep_idx)))
			continue;

		res = ipa_query_teth_stats(i, out, false);
		if (res) {
			mutex_unlock(&ipa3_ctx->lock);
			kfree(out);
			return res;
		}

		for (j = 0; j < IPA_CLIENT_MAX; j++) {
			int cons_idx = ipa3_get_ep_mapping(j);

			if (cons_idx == -1)
				continue;

			if (IPA_CLIENT_IS_TEST(j))
				continue;

			cons_reg = ipahal_get_ep_reg_idx(cons_idx);
			if (!(ipa3_ctx->hw_stats->teth.init.
				cons_bitmask[ep_idx][cons_reg]
				& ipahal_get_ep_bit(cons_idx)))
				continue;

			nbytes += scnprintf(dbg_buff + nbytes,
				IPA_MAX_MSG_LEN - nbytes,
				"%s->%s:\n",
				ipa_clients_strings[i],
				ipa_clients_strings[j]);
			nbytes += scnprintf(dbg_buff + nbytes,
				IPA_MAX_MSG_LEN - nbytes,
				"num_ipv4_bytes=%llu\n",
				out->client[j].num_ipv4_bytes);
			nbytes += scnprintf(dbg_buff + nbytes,
				IPA_MAX_MSG_LEN - nbytes,
				"num_ipv6_bytes=%llu\n",
				out->client[j].num_ipv6_bytes);
			nbytes += scnprintf(dbg_buff + nbytes,
				IPA_MAX_MSG_LEN - nbytes,
				"num_ipv4_pkts=%u\n",
				out->client[j].num_ipv4_pkts);
			nbytes += scnprintf(dbg_buff + nbytes,
				IPA_MAX_MSG_LEN - nbytes,
				"num_ipv6_pkts=%u\n",
				out->client[j].num_ipv6_pkts);
			nbytes += scnprintf(dbg_buff + nbytes,
				IPA_MAX_MSG_LEN - nbytes,
				"\n");
		}
	}
	mutex_unlock(&ipa3_ctx->lock);
	kfree(out);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa_debugfs_control_flt_rt_stats(struct file *file,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct ipa_ioc_flt_rt_query *query;
	unsigned long missing;
	int pyld_size = 0;
	int ret;

	query = kzalloc(sizeof(struct ipa_ioc_flt_rt_query),
		GFP_KERNEL);
	if (!query)
		return -ENOMEM;
	query->stats_size = sizeof(struct ipa_flt_rt_stats);
	pyld_size = IPA_MAX_FLT_RT_CNT_INDEX *
		sizeof(struct ipa_flt_rt_stats);
	query->stats = (uint64_t)kzalloc(pyld_size, GFP_KERNEL);
	if (!query->stats) {
		kfree(query);
		return -ENOMEM;
	}

	mutex_lock(&ipa3_ctx->lock);
	if (count >= sizeof(dbg_buff)) {
		ret = -EFAULT;
		goto bail;
	}

	missing = copy_from_user(dbg_buff, ubuf, count);
	if (missing) {
		ret = -EFAULT;
		goto bail;
	}

	dbg_buff[count] = '\0';
	if (strcmp(dbg_buff, "reset\n") == 0) {
		query->reset = 1;
		query->start_id = 1;
		query->end_id = IPA_MAX_FLT_RT_CNT_INDEX;
		ipa_get_flt_rt_stats(query);
	} else {
		IPAERR("unsupport flt_rt command\n");
	}

	ret = count;
bail:
	kfree((void *)(uintptr_t)(query->stats));
	kfree(query);
	mutex_unlock(&ipa3_ctx->lock);
	return ret;
}

static ssize_t ipa_debugfs_print_flt_rt_stats(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes = 0;
	int i;
	int res;
	int pyld_size = 0;
	struct ipa_ioc_flt_rt_query *query;

	query = kzalloc(sizeof(struct ipa_ioc_flt_rt_query),
		GFP_KERNEL);
	if (!query)
		return -ENOMEM;
	query->start_id = 1;
	query->end_id = IPA_MAX_FLT_RT_CNT_INDEX;
	query->reset = false;
	query->stats_size = sizeof(struct ipa_flt_rt_stats);
	pyld_size = IPA_MAX_FLT_RT_CNT_INDEX *
		sizeof(struct ipa_flt_rt_stats);
	query->stats = (uint64_t)kzalloc(pyld_size, GFP_KERNEL);
	if (!query->stats) {
		kfree(query);
		return -ENOMEM;
	}
	mutex_lock(&ipa3_ctx->lock);
	res = ipa_get_flt_rt_stats(query);
	if (res) {
		mutex_unlock(&ipa3_ctx->lock);
		kfree((void *)(uintptr_t)(query->stats));
		kfree(query);
		return res;
	}
	for (i = 0; i < IPA_MAX_FLT_RT_CNT_INDEX; i++) {
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"cnt_id: %d\n", i + 1);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"num_pkts: %d\n",
			((struct ipa_flt_rt_stats *)
			query->stats)[i].num_pkts);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"num_pkts_hash: %d\n",
			((struct ipa_flt_rt_stats *)
			query->stats)[i].num_pkts_hash);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"num_bytes: %lld\n",
			((struct ipa_flt_rt_stats *)
			query->stats)[i].num_bytes);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"\n");
	}
	mutex_unlock(&ipa3_ctx->lock);
	kfree((void *)(uintptr_t)(query->stats));
	kfree(query);
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa_debugfs_reset_drop_stats(struct file *file,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	s8 client = 0;
	int ret;

	mutex_lock(&ipa3_ctx->lock);

	ret = kstrtos8_from_user(ubuf, count, 0, &client);
	if (ret)
		goto bail;

	if (client == -1)
		ipa_reset_all_drop_stats();
	else
		ipa_reset_drop_stats(client);

	ret = count;
bail:
	mutex_unlock(&ipa3_ctx->lock);
	return count;
}

static ssize_t ipa_debugfs_print_drop_stats(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes = 0;
	struct ipa_drop_stats_all *out;
	int i, reg_idx;
	int res;

	out = kzalloc(sizeof(*out), GFP_KERNEL);
	if (!out)
		return -ENOMEM;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	mutex_lock(&ipa3_ctx->lock);
	res = ipa_get_drop_stats(out);
	if (res) {
		mutex_unlock(&ipa3_ctx->lock);
		kfree(out);
		return res;
	}

	for (i = 0; i < IPA_CLIENT_MAX; i++) {
		int ep_idx = ipa3_get_ep_mapping(i);

		if (ep_idx == -1)
			continue;

		if (!IPA_CLIENT_IS_CONS(i))
			continue;

		if (IPA_CLIENT_IS_TEST(i))
			continue;

		reg_idx = ipahal_get_ep_reg_idx(ep_idx);
		if (!(ipa3_ctx->hw_stats->drop.init.enabled_bitmask[reg_idx] &
			ipahal_get_ep_bit(ep_idx)))
			continue;

		/* Use more descriptive names for WLAN2_CONS pipes */
		if(i == IPA_CLIENT_WLAN2_CONS) {
			nbytes += scnprintf(dbg_buff + nbytes,
				IPA_MAX_MSG_LEN - nbytes,
				"IPA_CLIENT_WLAN2_HIGHSPEED_CONS:\n");
		} else if(i == IPA_CLIENT_WLAN2_CONS1) {
			nbytes += scnprintf(dbg_buff + nbytes,
				IPA_MAX_MSG_LEN - nbytes,
				" IPA_CLIENT_WLAN2_LOWSPEED_CONS:\n");
		} else {
			nbytes += scnprintf(dbg_buff + nbytes,
				IPA_MAX_MSG_LEN - nbytes,
				"%s:\n",
				ipa_clients_strings[i]);
		}

		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"drop_byte_cnt=%u\n",
			out->client[i].drop_byte_cnt);

		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"drop_packet_cnt=%u\n",
			out->client[i].drop_packet_cnt);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"\n");
	}
	mutex_unlock(&ipa3_ctx->lock);
	kfree(out);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa_debugfs_enable_disable_drop_stats(struct file *file,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	unsigned long missing;
	unsigned int pipe_num = 0;
	bool enable_pipe = true;
	u32 pipe_bitmask[IPAHAL_IPA5_PIPE_REG_NUM] = {0};
	u32 pipe_ep_reg_idx = 0;
	u32 pipe_ep_reg_bit = 0;
	char seprator = ',';
	int i, j;
	bool is_pipe = false;
	ssize_t ret;
	int pipe_num_temp;

	if (ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled) {
		for (i = 0; i < IPAHAL_IPA5_PIPE_REG_NUM; i++) {
			pipe_bitmask[i] =
				ipa3_ctx->hw_stats->drop.init.enabled_bitmask[i];
		}
	}

	mutex_lock(&ipa3_ctx->lock);
	if (count >= sizeof(dbg_buff)) {
		ret = -EFAULT;
		goto bail;
	}

	missing = copy_from_user(dbg_buff, ubuf, count);
	if (missing) {
		ret = -EFAULT;
		goto bail;
	}
	dbg_buff[count] = '\0';
	IPADBG("data is %s", dbg_buff);

	i = 0;
	while (dbg_buff[i] != ' ' && i < count)
		i++;
	j = i;
	i++;
	if (i < count) {
		if (dbg_buff[i] == '0') {
			enable_pipe = false;
			IPADBG("Drop stats will be disabled for pipes:");
		}
	}

	for (i = 0; i < j; i++) {
		if (dbg_buff[i] >= '0' && dbg_buff[i] <= '9') {
			pipe_num = (pipe_num * 10) + (dbg_buff[i] - '0');
			pipe_ep_reg_idx = ipahal_get_ep_reg_idx(pipe_num);
			pipe_ep_reg_bit = ipahal_get_ep_bit(pipe_num);
			is_pipe = true;
		}
		pipe_num_temp = ipa3_get_client_by_pipe(pipe_num);
		if (dbg_buff[i] == seprator) {
			/* Removing ADPL and ODL stats as Q6 supports it from IPA_5_0 */
			if ((pipe_num_temp == IPA_CLIENT_USB_DPL_CONS ||
				pipe_num_temp == IPA_CLIENT_ODL_DPL_CONS) &&
				ipa3_ctx->ipa_hw_type >= IPA_HW_v5_0) {
				pipe_num = 0;
				is_pipe = false;
				continue;
			}

			else if (pipe_num >= 0 && pipe_num < ipa3_ctx->ipa_num_pipes
				&& pipe_num_temp < IPA_CLIENT_MAX) {
				IPADBG("pipe number %u\n", pipe_num);
				if (enable_pipe)
					pipe_bitmask[pipe_ep_reg_idx] |=
						pipe_ep_reg_bit;
				else
					pipe_bitmask[pipe_ep_reg_idx] &=
						~pipe_ep_reg_bit;
			}
			pipe_num = 0;
			is_pipe = false;
		}
	}
	pipe_num_temp = ipa3_get_client_by_pipe(pipe_num);
	/* Removing ADPL and ODL stats as Q6 supports it from IPA_5_0 */
	if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_0 &&
		(pipe_num_temp == IPA_CLIENT_USB_DPL_CONS ||
		pipe_num_temp == IPA_CLIENT_ODL_DPL_CONS)) {
		IPAERR("Enable/Disable hw stats on DPL is not supported");
	} else if (is_pipe && pipe_num >= 0 && pipe_num < ipa3_ctx->ipa_num_pipes &&
		ipa3_get_client_by_pipe(pipe_num) < IPA_CLIENT_MAX) {
		IPADBG("pipe number %u\n", pipe_num);
		if (enable_pipe)
			pipe_bitmask[pipe_ep_reg_idx] |= pipe_ep_reg_bit;
		else
			pipe_bitmask[pipe_ep_reg_idx] &= ~pipe_ep_reg_bit;
	}

	ipa_init_drop_stats(pipe_bitmask);
	ret = count;
bail:
	mutex_unlock(&ipa3_ctx->lock);
	return ret;
}

static const struct file_operations ipa3_quota_ops = {
	.read = ipa_debugfs_print_quota_stats,
	.write = ipa_debugfs_reset_quota_stats,
};

static const struct file_operations ipa3_tethering_ops = {
	.read = ipa_debugfs_print_tethering_stats,
	.write = ipa_debugfs_reset_tethering_stats,
};

static const struct file_operations ipa3_flt_rt_ops = {
	.read = ipa_debugfs_print_flt_rt_stats,
	.write = ipa_debugfs_control_flt_rt_stats,
};

static const struct file_operations ipa3_drop_ops = {
	.read = ipa_debugfs_print_drop_stats,
	.write = ipa_debugfs_reset_drop_stats,
};

static const struct file_operations ipa3_enable_drop_ops = {
	.write = ipa_debugfs_enable_disable_drop_stats,
};

int ipa_debugfs_init_stats(struct dentry *parent)
{
	const mode_t read_write_mode = 0664;
	const mode_t write_mode = 0220;
	struct dentry *file;
	struct dentry *dent;

	if (!(ipa3_ctx->hw_stats && ipa3_ctx->hw_stats->enabled))
		return 0;

	dent = debugfs_create_dir("hw_stats", parent);
	if (IS_ERR_OR_NULL(dent)) {
		IPAERR("fail to create folder in debug_fs\n");
		return -EFAULT;
	}

	file = debugfs_create_file("quota", read_write_mode, dent, NULL,
		&ipa3_quota_ops);
	if (IS_ERR_OR_NULL(file)) {
		IPAERR("fail to create file %s\n", "quota");
		goto fail;
	}

	file = debugfs_create_file("drop", read_write_mode, dent, NULL,
		&ipa3_drop_ops);
	if (IS_ERR_OR_NULL(file)) {
		IPAERR("fail to create file %s\n", "drop");
		goto fail;
	}

	file = debugfs_create_file("enable_drop_stats", write_mode, dent, NULL,
		&ipa3_enable_drop_ops);
	if (IS_ERR_OR_NULL(file)) {
		IPAERR("fail to create file %s\n", "enable_drop_stats");
		goto fail;
	}

	file = debugfs_create_file("tethering", read_write_mode, dent, NULL,
		&ipa3_tethering_ops);
	if (IS_ERR_OR_NULL(file)) {
		IPAERR("fail to create file %s\n", "tethering");
		goto fail;
	}

	file = debugfs_create_file("flt_rt", read_write_mode, dent, NULL,
		&ipa3_flt_rt_ops);
	if (IS_ERR_OR_NULL(file)) {
		IPAERR("fail to create file flt_rt\n");
		goto fail;
	}

	return 0;
fail:
	debugfs_remove_recursive(dent);
	return -EFAULT;
}
#endif
