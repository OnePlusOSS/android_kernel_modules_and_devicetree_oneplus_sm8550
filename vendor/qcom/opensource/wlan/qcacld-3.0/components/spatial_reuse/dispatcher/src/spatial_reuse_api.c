/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.

 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC : contains interface prototypes for spatial_reuse api
 */
#include <spatial_reuse_api.h>
#include <target_if_spatial_reuse.h>

struct sr_cb {
	sr_osif_event_cb send_osif_event;
} sr_cb;

QDF_STATUS wlan_spatial_reuse_config_set(struct wlan_objmgr_vdev *vdev,
					 uint8_t sr_ctrl,
					 uint8_t non_srg_max_pd_offset)
{
	struct wlan_lmac_if_tx_ops *tx_ops;
	struct wlan_objmgr_psoc *psoc = wlan_vdev_get_psoc(vdev);

	if (!psoc)
		return QDF_STATUS_E_NULL_VALUE;

	tx_ops = wlan_psoc_get_lmac_if_txops(psoc);
	if (!tx_ops)
		return QDF_STATUS_E_NULL_VALUE;

	if (tx_ops->spatial_reuse_tx_ops.send_cfg)
		return tx_ops->spatial_reuse_tx_ops.send_cfg(vdev, sr_ctrl,
							non_srg_max_pd_offset);

	return QDF_STATUS_E_NULL_VALUE;
}

QDF_STATUS wlan_spatial_reuse_he_siga_val15_allowed_set(
					struct wlan_objmgr_vdev *vdev,
					bool he_siga_va15_allowed)
{
	struct wlan_lmac_if_tx_ops *tx_ops;
	struct wlan_objmgr_psoc *psoc = wlan_vdev_get_psoc(vdev);

	if (!psoc)
		return QDF_STATUS_E_NULL_VALUE;

	tx_ops = wlan_psoc_get_lmac_if_txops(psoc);
	if (!tx_ops)
		return QDF_STATUS_E_NULL_VALUE;

	if (tx_ops->spatial_reuse_tx_ops.send_sr_prohibit_cfg)
		return tx_ops->spatial_reuse_tx_ops.send_sr_prohibit_cfg(
							vdev,
							he_siga_va15_allowed);
	return QDF_STATUS_E_NULL_VALUE;
}

QDF_STATUS
wlan_sr_setup_req(struct wlan_objmgr_vdev *vdev, struct wlan_objmgr_pdev *pdev,
		  bool is_sr_enable, int32_t pd_threshold) {
	struct wlan_lmac_if_tx_ops *tx_ops;
	QDF_STATUS status = QDF_STATUS_E_FAILURE;

	tx_ops = wlan_psoc_get_lmac_if_txops(wlan_pdev_get_psoc(pdev));
	if (tx_ops &&
	    tx_ops->spatial_reuse_tx_ops.target_if_set_sr_enable_disable) {
		status =
		tx_ops->spatial_reuse_tx_ops.target_if_set_sr_enable_disable(
					vdev, pdev, is_sr_enable, pd_threshold);
		return status;
	}
	return status;
}

QDF_STATUS wlan_spatial_reuse_pdev_init(struct wlan_objmgr_pdev *pdev)
{
	struct pdev_params pparam;
	wmi_unified_t wmi_handle;
	bool sr_supported;
	bool sr_per_ppdu_supported;

	wmi_handle = GET_WMI_HDL_FROM_PDEV(pdev);
	if (!wmi_handle) {
		mlme_err("Failed to get WMI handle!");
		return QDF_STATUS_E_INVAL;
	}
	sr_supported =
		wmi_service_enabled(wmi_handle,
				    wmi_service_srg_srp_spatial_reuse_support);
	sr_per_ppdu_supported =
		wmi_service_enabled(wmi_handle,
				    wmi_service_obss_per_packet_sr_support);
	if (!sr_per_ppdu_supported && !sr_supported) {
		mlme_err("FW doesn't support");
		return QDF_STATUS_E_NOSUPPORT;
	}

	qdf_mem_zero(&pparam, sizeof(pparam));
	pparam.param_id = WMI_PDEV_PARAM_SET_CMD_OBSS_PD_THRESHOLD;
	QDF_SET_BITS(pparam.param_value, NON_SRG_SPR_ENABLE_POS,
		     NON_SRG_SPR_ENABLE_SIZE, NON_SRG_SPR_ENABLE);
	QDF_SET_BITS(pparam.param_value, SR_PARAM_VAL_DBM_POS,
		     NON_SRG_PARAM_VAL_DBM_SIZE, SR_PARAM_VAL_DBM_UNIT);
	QDF_SET_BITS(pparam.param_value, NON_SRG_MAX_PD_OFFSET_POS,
		     NON_SRG_MAX_PD_OFFSET_SIZE, NON_SR_PD_THRESHOLD_DISABLED);

	return wmi_unified_pdev_param_send(wmi_handle, &pparam,
					   WILDCARD_PDEV_ID);
}

void wlan_sr_register_callback(struct wlan_objmgr_psoc *psoc,
			       sr_osif_event_cb cb)
{
	if (!psoc)
		return;
	sr_cb.send_osif_event = cb;
}

void wlan_spatial_reuse_osif_event(struct wlan_objmgr_vdev *vdev,
				   enum sr_osif_operation sr_osif_oper,
				   enum sr_osif_reason_code sr_osif_rc)
{
	if (sr_cb.send_osif_event)
		sr_cb.send_osif_event(vdev, sr_osif_oper, sr_osif_rc);
}
