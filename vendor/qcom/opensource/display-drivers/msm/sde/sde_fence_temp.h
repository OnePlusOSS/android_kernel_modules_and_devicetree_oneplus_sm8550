/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 */

#ifndef _SDE_FENCE_TEMP_H_
#define _SDE_FENCE_TEMP_H_

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/soc/qcom/msm_hw_fence.h>

#define MAX_SDE_HFENCE_OUT_SIGNAL_PING_PONG 2

/**
 * struct sde_hw_fence_data - contains the information of each display-client of the hw-fences
 *                       to communicate with the fence controller.
 * @client_id: client_id enum for the display driver.
 * @hw_fence_client_id: client_id enum for the hw-fence driver.
 * @mem_descriptor: memory descriptor with the hfi for the rx/tx queues mapping.
 * @ipcc_in_client: ipcc client triggering the signal: IN_CLIENT (APPS) -> DPU
 * @ipcc_in_signal: ipcc signal triggered from client to dpu: IN_SIGNAL (APPS) -> DPU
 * @ipcc_out_signal_pp: output signal from dpu to fctl, ping-pongs between two signals
 * @ipcc_out_signal_pp_idx: index of the output signal ping-pong
 * @ipcc_out_client: destination client id (APPS for the FCTL)
 * @ipcc_this_client: ipcc dpu client id (For Waipio: APPS, For Kailua: DPU HW)
 */
struct sde_hw_fence_data {
	int client_id;
	enum hw_fence_client_id hw_fence_client_id;
	void *hw_fence_handle;
	struct msm_hw_fence_mem_addr mem_descriptor;
	u32 ipcc_in_client;
	u32 ipcc_in_signal;
	u32 ipcc_out_signal_pp[MAX_SDE_HFENCE_OUT_SIGNAL_PING_PONG];
	u32 ipcc_out_signal_pp_idx;
	u32 ipcc_out_client;
	u32 ipcc_this_client;
};

#endif /* _SDE_FENCE_TEMP_H_ */
