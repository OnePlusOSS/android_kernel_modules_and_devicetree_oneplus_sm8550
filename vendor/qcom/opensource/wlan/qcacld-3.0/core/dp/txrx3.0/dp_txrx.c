/*
 * Copyright (c) 2017-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

#include <wlan_objmgr_pdev_obj.h>
#include <dp_txrx.h>
#include <dp_types.h>
#include <dp_internal.h>
#include <cdp_txrx_cmn.h>
#include <cdp_txrx_misc.h>
#include <dp_tx_desc.h>
#include <dp_rx.h>
#include <ce_api.h>
#include <ce_internal.h>
#include <wlan_cfg.h>

/**
 * dp_rx_refill_thread_schedule() - Schedule rx refill thread
 * @soc: ol_txrx_soc_handle object
 *
 */
#ifdef WLAN_FEATURE_RX_PREALLOC_BUFFER_POOL
static void dp_rx_refill_thread_schedule(ol_txrx_soc_handle soc)
{
	struct dp_rx_refill_thread *rx_thread;
	struct dp_txrx_handle *dp_ext_hdl;

	if (!soc)
		return;

	dp_ext_hdl = cdp_soc_get_dp_txrx_handle(soc);
	if (!dp_ext_hdl)
		return;

	rx_thread = &dp_ext_hdl->refill_thread;
	qdf_set_bit(RX_REFILL_POST_EVENT, &rx_thread->event_flag);
	qdf_wake_up_interruptible(&rx_thread->wait_q);
}
#else
static void dp_rx_refill_thread_schedule(ol_txrx_soc_handle soc)
{
}
#endif

/**
 * dp_get_rx_threads_num() - Get number of threads in use
 * @soc: ol_txrx_soc_handle object
 *
 * Return: number of threads
 */
static uint8_t dp_get_rx_threads_num(ol_txrx_soc_handle soc)
{
	return cdp_get_num_rx_contexts(soc);
}

QDF_STATUS dp_txrx_init(ol_txrx_soc_handle soc, uint8_t pdev_id,
			struct dp_txrx_config *config)
{
	struct dp_txrx_handle *dp_ext_hdl;
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	uint8_t num_dp_rx_threads;
	struct dp_pdev *pdev;
	struct dp_soc *dp_soc;

	if (qdf_unlikely(!soc)) {
		dp_err("soc is NULL");
		return 0;
	}

	pdev = dp_get_pdev_from_soc_pdev_id_wifi3(cdp_soc_t_to_dp_soc(soc),
						  pdev_id);
	if (!pdev) {
		dp_err("pdev is NULL");
		return 0;
	}

	dp_ext_hdl = qdf_mem_malloc(sizeof(*dp_ext_hdl));
	if (!dp_ext_hdl) {
		QDF_ASSERT(0);
		return QDF_STATUS_E_NOMEM;
	}

	dp_info("dp_txrx_handle allocated");
	dp_ext_hdl->soc = soc;
	dp_ext_hdl->pdev = dp_pdev_to_cdp_pdev(pdev);
	cdp_soc_set_dp_txrx_handle(soc, dp_ext_hdl);
	qdf_mem_copy(&dp_ext_hdl->config, config, sizeof(*config));
	dp_ext_hdl->rx_tm_hdl.txrx_handle_cmn =
				dp_txrx_get_cmn_hdl_frm_ext_hdl(dp_ext_hdl);

	dp_soc = cdp_soc_t_to_dp_soc(soc);
	if (wlan_cfg_is_rx_refill_buffer_pool_enabled(dp_soc->wlan_cfg_ctx)) {
		dp_ext_hdl->refill_thread.soc = soc;
		dp_ext_hdl->refill_thread.enabled = true;
		qdf_status =
			dp_rx_refill_thread_init(&dp_ext_hdl->refill_thread);
		if (qdf_status != QDF_STATUS_SUCCESS) {
			dp_err("Failed to initialize RX refill thread status:%d",
			       qdf_status);
			return qdf_status;
		}
		cdp_register_rx_refill_thread_sched_handler(soc,
						dp_rx_refill_thread_schedule);
	}

	num_dp_rx_threads = dp_get_rx_threads_num(soc);
	dp_info("%d RX threads in use", num_dp_rx_threads);

	if (dp_ext_hdl->config.enable_rx_threads) {
		qdf_status = dp_rx_tm_init(&dp_ext_hdl->rx_tm_hdl,
					   num_dp_rx_threads);
	}

	return qdf_status;
}

QDF_STATUS dp_txrx_deinit(ol_txrx_soc_handle soc)
{
	struct dp_txrx_handle *dp_ext_hdl;
	struct dp_soc *dp_soc;

	if (!soc)
		return QDF_STATUS_E_INVAL;

	dp_ext_hdl = cdp_soc_get_dp_txrx_handle(soc);
	if (!dp_ext_hdl)
		return QDF_STATUS_E_FAULT;

	dp_soc = cdp_soc_t_to_dp_soc(soc);
	if (wlan_cfg_is_rx_refill_buffer_pool_enabled(dp_soc->wlan_cfg_ctx)) {
		dp_rx_refill_thread_deinit(&dp_ext_hdl->refill_thread);
		dp_ext_hdl->refill_thread.soc = NULL;
		dp_ext_hdl->refill_thread.enabled = false;
	}

	if (dp_ext_hdl->config.enable_rx_threads)
		dp_rx_tm_deinit(&dp_ext_hdl->rx_tm_hdl);

	qdf_mem_free(dp_ext_hdl);
	dp_info("dp_txrx_handle_t de-allocated");

	cdp_soc_set_dp_txrx_handle(soc, NULL);

	return QDF_STATUS_SUCCESS;
}

/**
 * dp_rx_tm_get_pending() - get number of frame in thread
 * nbuf queue pending
 * @soc: ol_txrx_soc_handle object
 *
 * Return: number of frames
 */
#ifdef FEATURE_WLAN_DP_RX_THREADS
int dp_rx_tm_get_pending(ol_txrx_soc_handle soc)
{
	int i;
	int num_pending = 0;
	struct dp_rx_thread *rx_thread;
	struct dp_txrx_handle *dp_ext_hdl;
	struct dp_rx_tm_handle *rx_tm_hdl;

	if (!soc)
		return 0;

	dp_ext_hdl = cdp_soc_get_dp_txrx_handle(soc);
	if (!dp_ext_hdl)
		return 0;

	rx_tm_hdl = &dp_ext_hdl->rx_tm_hdl;

	for (i = 0; i < rx_tm_hdl->num_dp_rx_threads; i++) {
		rx_thread = rx_tm_hdl->rx_thread[i];
		if (!rx_thread)
			continue;
		num_pending += qdf_nbuf_queue_head_qlen(&rx_thread->nbuf_queue);
	}

	if (num_pending)
		dp_debug("pending frames in thread queue %d", num_pending);

	return num_pending;
}
#else
int dp_rx_tm_get_pending(ol_txrx_soc_handle soc)
{
	return 0;
}
#endif
