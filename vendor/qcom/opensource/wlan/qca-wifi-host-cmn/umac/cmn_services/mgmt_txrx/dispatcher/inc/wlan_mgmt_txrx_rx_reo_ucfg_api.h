/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

#ifndef _WLAN_MGMT_TXRX_RX_REO_UCFG_API_H_
#define _WLAN_MGMT_TXRX_RX_REO_UCFG_API_H_

/**
 * DOC: wlan_mgmt_txrx_rx_reo_ucfg_api.h
 * This file contains mgmt rx-reorder ucfg layer related APIs
 */

#include <wlan_mgmt_txrx_rx_reo_utils_api.h>

#ifdef WLAN_MGMT_RX_REO_SUPPORT
/**
 * ucfg_wlan_mgmt_rx_reo_sim_start() - Helper API to start mgmt rx reorder
 * simulation
 *
 * This API starts the simulation framework which mimics the management frame
 * generation by target. MAC HW is modelled as a kthread. FW and host layers
 * are modelled as an ordered work queues.
 *
 * Return: QDF_STATUS
 */
QDF_STATUS
ucfg_wlan_mgmt_rx_reo_sim_start(void);

/**
 * ucfg_wlan_mgmt_rx_reo_sim_stop() - Helper API to stop mgmt rx reorder
 * simulation
 *
 * This API stops the simulation framework which mimics the management frame
 * generation by target. MAC HW is modelled as a kthread. FW and host layers
 * are modelled as an ordered work queues.
 * Return: QDF_STATUS
 */
QDF_STATUS
ucfg_wlan_mgmt_rx_reo_sim_stop(void);

/**
 * ucfg_wlan_mgmt_rx_reo_is_simulation_in_progress() - API to check whether
 * simulation is in progress
 *
 * Return: true if simulation is in progress, else false
 */
bool
ucfg_wlan_mgmt_rx_reo_is_simulation_in_progress(void);

#else
/**
 * ucfg_wlan_mgmt_rx_reo_sim_start() - Helper API to start mgmt rx
 * reorder simulation
 *
 * Error print is added to indicate that simulation framework is not compiled.
 * Return: QDF_STATUS_E_INVAL
 */
static inline QDF_STATUS
ucfg_wlan_mgmt_rx_reo_sim_start(void)
{
	mgmt_txrx_err("Mgmt rx reo simulation is not compiled");

	return QDF_STATUS_E_INVAL;
};

/**
 * ucfg_wlan_mgmt_rx_reo_sim_stop() - Helper API to stop mgmt rx
 * reorder simulation
 *
 * Error print is added to indicate that simulation framework is not compiled.
 * Return: QDF_STATUS_E_INVAL
 */
static inline QDF_STATUS
ucfg_wlan_mgmt_rx_reo_sim_stop(void)
{
	mgmt_txrx_err("Mgmt rx reo simulation is not compiled");

	return QDF_STATUS_E_INVAL;
}
#endif /* WLAN_MGMT_RX_REO_SUPPORT */
#endif /* _WLAN_MGMT_TXRX_RX_REO_UCFG_API_H_ */
