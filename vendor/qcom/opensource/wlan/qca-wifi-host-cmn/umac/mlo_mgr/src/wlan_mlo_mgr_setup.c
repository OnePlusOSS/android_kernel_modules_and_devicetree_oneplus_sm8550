/* Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * DOC: contains MLO manager ap related functionality
 */
#include "wlan_mlo_mgr_cmn.h"
#include "wlan_mlo_mgr_main.h"
#ifdef WLAN_MLO_MULTI_CHIP
#include "wlan_lmac_if_def.h"
#include <cdp_txrx_mlo.h>
#endif
#include <wlan_mgmt_txrx_rx_reo_utils_api.h>

#ifdef WLAN_MLO_MULTI_CHIP
bool mlo_is_ml_soc(struct wlan_objmgr_psoc *psoc)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	uint8_t chip_idx;

	if (!mlo_ctx)
		return false;

	for (chip_idx = 0; chip_idx < MAX_MLO_CHIPS; chip_idx++)
		if (mlo_ctx->setup_info.soc_list[chip_idx] == psoc)
			return true;

	return false;
}

qdf_export_symbol(mlo_is_ml_soc);

void mlo_get_soc_list(struct wlan_objmgr_psoc **soc_list)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	uint8_t chip_idx;

	if (!mlo_ctx) {
		for (chip_idx = 0; chip_idx < MAX_MLO_CHIPS; chip_idx++)
			soc_list[chip_idx] = NULL;

		return;
	}

	for (chip_idx = 0; chip_idx < MAX_MLO_CHIPS; chip_idx++)
		soc_list[chip_idx] = mlo_ctx->setup_info.soc_list[chip_idx];
}

qdf_export_symbol(mlo_get_soc_list);

void mlo_cleanup_asserted_soc_setup_info(struct wlan_objmgr_psoc *psoc)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	uint8_t link_idx;
	struct wlan_objmgr_pdev *pdev;

	if (!mlo_ctx)
		return;

	if (!mlo_ctx->setup_info.num_links)
		return;

	if (!psoc) {
		qdf_info("NULL psoc");
		return;
	}

	for (link_idx = 0; link_idx < MAX_MLO_LINKS; link_idx++) {
		pdev = mlo_ctx->setup_info.pdev_list[link_idx];
		if (pdev) {
			if (wlan_pdev_get_psoc(pdev) == psoc) {
				mlo_ctx->setup_info.pdev_list[link_idx] = NULL;
				mlo_ctx->setup_info.state[link_idx] =
					MLO_LINK_TEARDOWN;
				mlo_ctx->setup_info.num_links--;
			}
		}
	}
}

qdf_export_symbol(mlo_cleanup_asserted_soc_setup_info);

void mlo_setup_update_total_socs(uint8_t tot_socs)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();

	if (!mlo_ctx)
		return;

	mlo_ctx->setup_info.tot_socs = tot_socs;
}

qdf_export_symbol(mlo_setup_update_total_socs);

static QDF_STATUS mlo_find_pdev_idx(struct wlan_objmgr_pdev *pdev,
				    uint8_t *link_idx)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	uint8_t idx;

	if (!mlo_ctx)
		return QDF_STATUS_E_FAILURE;

	if (!link_idx)
		return QDF_STATUS_E_FAILURE;

	for (idx = 0; idx < mlo_ctx->setup_info.tot_links; idx++) {
		if (mlo_ctx->setup_info.pdev_list[idx] == pdev) {
			*link_idx = idx;
			return QDF_STATUS_SUCCESS;
		}
	}

	return QDF_STATUS_E_FAILURE;
}

#define WLAN_SOC_ID_NOT_INITIALIZED -1
bool mlo_vdevs_check_single_soc(struct wlan_objmgr_vdev **wlan_vdev_list,
				uint8_t vdev_count)
{
	int i;
	uint8_t soc_id = WLAN_SOC_ID_NOT_INITIALIZED;

	for (i = 0; i < vdev_count; i++) {
		uint8_t vdev_soc_id = wlan_vdev_get_psoc_id(wlan_vdev_list[i]);

		if (i == 0)
			soc_id = vdev_soc_id;
		else if (soc_id != vdev_soc_id)
			return false;
	}

	return true;
}

qdf_export_symbol(mlo_vdevs_check_single_soc);

static void mlo_check_state(struct wlan_objmgr_psoc *psoc,
			    void *obj, void *args)
{
	struct wlan_objmgr_pdev *pdev;
	uint8_t link_idx;
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	struct mlo_state_params *params = (struct mlo_state_params *)args;

	pdev = (struct wlan_objmgr_pdev *)obj;

	if (!mlo_ctx)
		return;

	if (mlo_find_pdev_idx(pdev, &link_idx) != QDF_STATUS_SUCCESS) {
		qdf_info("Failed to find pdev");
		return;
	}

	if (mlo_ctx->setup_info.state[link_idx] != params->check_state)
		params->link_state_fail = 1;
}

QDF_STATUS mlo_check_all_pdev_state(struct wlan_objmgr_psoc *psoc,
				    enum MLO_LINK_STATE state)
{
	QDF_STATUS status = QDF_STATUS_E_INVAL;
	struct mlo_state_params params = {0};

	params.check_state = state;

	wlan_objmgr_iterate_obj_list(psoc, WLAN_PDEV_OP,
				     mlo_check_state, &params,
				     0, WLAN_MLME_NB_ID);

	if (params.link_state_fail)
		status = QDF_STATUS_E_INVAL;
	else
		status = QDF_STATUS_SUCCESS;

	return status;
}

void mlo_setup_init(void)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();

	if (!mlo_ctx)
		return;

	if (qdf_event_create(&mlo_ctx->setup_info.event) !=
						QDF_STATUS_SUCCESS) {
		mlo_err("Unable to create teardown event");
	}
}

qdf_export_symbol(mlo_setup_init);

void mlo_setup_deinit(void)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();

	if (!mlo_ctx)
		return;

	qdf_event_destroy(&mlo_ctx->setup_info.event);
}

qdf_export_symbol(mlo_setup_deinit);

void mlo_setup_update_num_links(struct wlan_objmgr_psoc *psoc,
				uint8_t num_links)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();

	if (!mlo_ctx)
		return;

	mlo_ctx->setup_info.tot_links += num_links;
}

qdf_export_symbol(mlo_setup_update_num_links);

void mlo_setup_update_soc_ready(struct wlan_objmgr_psoc *psoc)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	uint8_t chip_idx, tot_socs = 0;

	if (!mlo_ctx || !mlo_ctx->setup_info.tot_socs)
		return;

	tot_socs = mlo_ctx->setup_info.tot_socs;
	chip_idx = wlan_psoc_get_id(psoc);

	if (!(chip_idx < MAX_MLO_CHIPS)) {
		qdf_err("Invalid chip index, SoC setup failed");
		return;
	}

	mlo_ctx->setup_info.soc_list[chip_idx] = psoc;
	mlo_ctx->setup_info.num_soc++;
	qdf_debug("soc updated to mld list, id %d num soc %d",
		  chip_idx, mlo_ctx->setup_info.num_soc);

	if (mlo_ctx->setup_info.num_soc != mlo_ctx->setup_info.tot_socs)
		return;

	for (chip_idx = 0; chip_idx < MAX_MLO_CHIPS; chip_idx++) {
		struct wlan_objmgr_psoc *tmp_soc =
			mlo_ctx->setup_info.soc_list[chip_idx];
		if (tmp_soc)
			cdp_soc_mlo_soc_setup(wlan_psoc_get_dp_handle(tmp_soc),
					      mlo_ctx->dp_handle);
	}

	cdp_mlo_setup_complete(wlan_psoc_get_dp_handle(psoc),
			       mlo_ctx->dp_handle);
}

qdf_export_symbol(mlo_setup_update_soc_ready);

void mlo_setup_link_ready(struct wlan_objmgr_pdev *pdev)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	uint8_t link_idx;
	uint16_t link_id;

	if (!mlo_ctx || !mlo_ctx->setup_info.tot_links)
		return;

	if (mlo_find_pdev_idx(pdev, &link_idx) == QDF_STATUS_SUCCESS) {
		qdf_debug("pdev already part of list link idx %d", link_idx);
		return;
	}

	for (link_idx = 0; link_idx < mlo_ctx->setup_info.tot_links; link_idx++)
		if (!mlo_ctx->setup_info.pdev_list[link_idx])
			break;

	if (link_idx >= mlo_ctx->setup_info.tot_links) {
		qdf_err("Exceeding max total mld links");
		return;
	}

	mlo_ctx->setup_info.pdev_list[link_idx] = pdev;
	mlo_ctx->setup_info.state[link_idx] = MLO_LINK_SETUP_INIT;
	mlo_ctx->setup_info.num_links++;

	link_id = wlan_mlo_get_pdev_hw_link_id(pdev);
	if (link_id == INVALID_HW_LINK_ID) {
		qdf_err("Invalid HW link id for the pdev");
		return;
	}
	mlo_ctx->setup_info.valid_link_bitmap |= (1 << link_id);

	qdf_debug("pdev updated to mld link %d num_links %d",
		  link_idx, mlo_ctx->setup_info.num_links);

	qdf_assert_always(link_idx < MAX_MLO_LINKS);

	if (mlo_ctx->setup_info.num_links == mlo_ctx->setup_info.tot_links &&
	    mlo_ctx->setup_info.num_soc == mlo_ctx->setup_info.tot_socs) {
		struct wlan_objmgr_psoc *psoc;
		struct wlan_lmac_if_tx_ops *tx_ops;
		QDF_STATUS status;

		psoc = wlan_pdev_get_psoc(pdev);
		tx_ops = wlan_psoc_get_lmac_if_txops(psoc);

		status = wlan_mgmt_rx_reo_validate_mlo_link_info(psoc);
		if (QDF_IS_STATUS_ERROR(status)) {
			mlo_err("Failed to validate MLO HW link info");
			qdf_assert_always(0);
		}

		qdf_debug("Trigger MLO Setup request");
		if (tx_ops && tx_ops->mops.target_if_mlo_setup_req) {
			tx_ops->mops.target_if_mlo_setup_req(
					mlo_ctx->setup_info.pdev_list,
					mlo_ctx->setup_info.num_links,
					mlo_ctx->setup_info.ml_grp_id);
		}
	}
}

qdf_export_symbol(mlo_setup_link_ready);

void mlo_link_setup_complete(struct wlan_objmgr_pdev *pdev)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	uint8_t link_idx;

	if (!mlo_ctx)
		return;

	for (link_idx = 0; link_idx < mlo_ctx->setup_info.tot_links; link_idx++)
		if (mlo_ctx->setup_info.pdev_list[link_idx] == pdev) {
			mlo_ctx->setup_info.state[link_idx] =
							MLO_LINK_SETUP_DONE;
			break;
		}

	for (link_idx = 0; link_idx < mlo_ctx->setup_info.tot_links; link_idx++)
		if (mlo_ctx->setup_info.state[link_idx] == MLO_LINK_SETUP_DONE)
			continue;
		else
			break;

	if (link_idx == mlo_ctx->setup_info.tot_links) {
		struct wlan_objmgr_psoc *psoc;
		struct wlan_lmac_if_tx_ops *tx_ops;

		psoc = wlan_pdev_get_psoc(pdev);
		tx_ops = wlan_psoc_get_lmac_if_txops(psoc);
		/* Trigger MLO ready */
		if (tx_ops && tx_ops->mops.target_if_mlo_ready) {
			tx_ops->mops.target_if_mlo_ready(
					mlo_ctx->setup_info.pdev_list,
					mlo_ctx->setup_info.num_links);
		}
	}
}

qdf_export_symbol(mlo_link_setup_complete);

static void mlo_setup_link_down(struct wlan_objmgr_psoc *psoc,
				void *obj, void *args)
{
	struct wlan_objmgr_pdev *pdev;
	uint8_t link_idx;
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	uint16_t link_id;

	pdev = (struct wlan_objmgr_pdev *)obj;

	if (mlo_find_pdev_idx(pdev, &link_idx) != QDF_STATUS_SUCCESS) {
		qdf_info("Failed to find pdev");
		return;
	}

	mlo_ctx->setup_info.pdev_list[link_idx] = NULL;
	mlo_ctx->setup_info.state[link_idx] = MLO_LINK_UNINITIALIZED;
	mlo_ctx->setup_info.num_links--;

	link_id = wlan_mlo_get_pdev_hw_link_id(pdev);
	if (link_id == INVALID_HW_LINK_ID) {
		qdf_err("Invalid HW link id for the pdev");
		return;
	}
	mlo_ctx->setup_info.valid_link_bitmap &= ~(1 << link_id);

	qdf_debug("link down link_idx %d num_links %d",
		  link_idx, mlo_ctx->setup_info.num_links);
}

void mlo_setup_update_soc_down(struct wlan_objmgr_psoc *psoc)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	uint8_t chip_idx;

	if (!mlo_ctx)
		return;

	if (!mlo_ctx->setup_info.num_links) {
		qdf_debug("Links are already down");
		return;
	}

	wlan_objmgr_iterate_obj_list(psoc, WLAN_PDEV_OP,
				     mlo_setup_link_down, NULL,
				     0, WLAN_MLME_NB_ID);

	chip_idx = wlan_psoc_get_id(psoc);

	if (!(chip_idx < MAX_MLO_CHIPS)) {
		qdf_err("Invalid chip index, SoC setup down failed");
		return;
	}

	mlo_ctx->setup_info.soc_list[chip_idx] = NULL;
	mlo_ctx->setup_info.num_soc--;

	qdf_debug("Soc down, num soc %d num links %d",
		  mlo_ctx->setup_info.num_soc,
		  mlo_ctx->setup_info.num_links);
}

qdf_export_symbol(mlo_setup_update_soc_down);

void mlo_link_teardown_complete(struct wlan_objmgr_pdev *pdev)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	uint8_t link_idx;
	struct wlan_objmgr_psoc *soc;
	uint8_t chip_idx;

	if (!mlo_ctx)
		return;

	if (!mlo_ctx->setup_info.num_links) {
		qdf_err("Delayed response ignore");
		return;
	}

	if (mlo_find_pdev_idx(pdev, &link_idx) != QDF_STATUS_SUCCESS) {
		qdf_info("Failed to find pdev");
		return;
	}

	qdf_debug("Teardown link idx = %d", link_idx);
	mlo_ctx->setup_info.state[link_idx] = MLO_LINK_TEARDOWN;

	/* Waiting for teardown on other links */
	for (link_idx = 0; link_idx < mlo_ctx->setup_info.tot_links; link_idx++)
		if (mlo_ctx->setup_info.state[link_idx] != MLO_LINK_TEARDOWN)
			return;

	qdf_info("Teardown complete");

	for (chip_idx = 0; chip_idx < MAX_MLO_CHIPS; chip_idx++) {
		soc = mlo_ctx->setup_info.soc_list[chip_idx];
		if (soc)
			cdp_soc_mlo_soc_teardown(wlan_psoc_get_dp_handle(soc),
						 mlo_ctx->dp_handle);
	}

	qdf_event_set(&mlo_ctx->setup_info.event);
}

qdf_export_symbol(mlo_link_teardown_complete);

static void mlo_force_teardown(void)
{
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();
	struct wlan_objmgr_psoc *soc;
	uint8_t link_idx = 0;
	uint8_t chip_idx;

	if (!mlo_ctx)
		return;

	for (link_idx = 0; link_idx < mlo_ctx->setup_info.tot_links; link_idx++)
		mlo_ctx->setup_info.state[link_idx] = MLO_LINK_TEARDOWN;

	for (chip_idx = 0; chip_idx < MAX_MLO_CHIPS; chip_idx++) {
		soc = mlo_ctx->setup_info.soc_list[chip_idx];
		if (soc)
			cdp_soc_mlo_soc_teardown(wlan_psoc_get_dp_handle(soc),
						 mlo_ctx->dp_handle);
	}
}

#define MLO_MGR_TEARDOWN_TIMEOUT 3000
QDF_STATUS mlo_link_teardown_link(struct wlan_objmgr_psoc *psoc,
				  uint32_t reason)
{
	struct wlan_lmac_if_tx_ops *tx_ops;
	QDF_STATUS status;
	struct mlo_mgr_context *mlo_ctx = wlan_objmgr_get_mlo_ctx();

	if (!mlo_ctx)
		return QDF_STATUS_E_FAILURE;

	qdf_debug("Teardown req with num_soc %d num_link %d",
		  mlo_ctx->setup_info.num_soc,
		  mlo_ctx->setup_info.num_links);

	if (!mlo_ctx->setup_info.num_soc)
		return QDF_STATUS_SUCCESS;

	if (!mlo_check_all_pdev_state(psoc, MLO_LINK_TEARDOWN))
		return QDF_STATUS_SUCCESS;

	tx_ops = wlan_psoc_get_lmac_if_txops(psoc);
	/* Trigger MLO teardown */
	if (tx_ops && tx_ops->mops.target_if_mlo_teardown_req) {
		tx_ops->mops.target_if_mlo_teardown_req(
				mlo_ctx->setup_info.pdev_list,
				mlo_ctx->setup_info.num_links,
				reason);
	}

	if (reason == WMI_MLO_TEARDOWN_REASON_SSR) {
		/* do not wait for teardown event completion here for SSR */
		return QDF_STATUS_SUCCESS;
	}

	status = qdf_wait_for_event_completion(
			&mlo_ctx->setup_info.event,
			MLO_MGR_TEARDOWN_TIMEOUT);
	if (status != QDF_STATUS_SUCCESS) {
		qdf_info("Teardown timeout");
		mlo_force_teardown();
	}

	return status;
}

qdf_export_symbol(mlo_link_teardown_link);
#endif /*WLAN_MLO_MULTI_CHIP*/
