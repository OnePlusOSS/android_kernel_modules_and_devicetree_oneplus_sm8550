/*
 * Copyright (c) 2019-2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */
/**
 * DOC: Target interface file for denylist manager component to
 * declare api's which shall be used by denylist manager component
 * in target if internally.
 */

#ifndef __TARGET_IF_DLM_H
#define __TARGET_IF_DLM_H

#include "wlan_dlm_public_struct.h"

#if defined(WLAN_FEATURE_ROAM_OFFLOAD)
/**
 * target_if_dlm_send_reject_ap_list() - API to send reject ap list to FW
 * @pdev: pdev object
 * @reject_params: This contains the reject ap list, and the num of BSSIDs
 *
 * This API will send the reject ap list to the target for it to handle roaming
 * case scenarios.
 *
 * Return: Qdf status
 */
QDF_STATUS
target_if_dlm_send_reject_ap_list(struct wlan_objmgr_pdev *pdev,
				  struct reject_ap_params *reject_params);

/**
 * target_if_dlm_register_tx_ops() - Register dlm tx ops
 * @dlm_tx_ops: DLM tx ops
 *
 * This API will register the tx ops used by the DLM to send commands to the
 * target.
 *
 * Return: void
 */
void target_if_dlm_register_tx_ops(struct wlan_dlm_tx_ops *dlm_tx_ops);
#else
static inline void target_if_dlm_register_tx_ops(
	struct wlan_dlm_tx_ops *dlm_tx_ops)
{
}
#endif //WLAN_FEATURE_ROAM_OFFLOAD

#endif //__TARGET_IF_DLM_H
