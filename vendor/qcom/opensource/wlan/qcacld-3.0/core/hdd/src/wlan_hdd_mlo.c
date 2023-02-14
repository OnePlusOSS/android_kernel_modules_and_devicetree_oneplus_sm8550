/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

/**
 * DOC : wlan_hdd_mlo.c
 *
 * WLAN Host Device Driver file for 802.11be (Extremely High Throughput)
 * support.
 *
 */
#include "wlan_hdd_main.h"
#include "wlan_hdd_mlo.h"
#include "osif_vdev_sync.h"
#include "wlan_osif_features.h"
#include "wlan_dp_ucfg_api.h"
#include "wlan_psoc_mlme_ucfg_api.h"

#if defined(CFG80211_11BE_BASIC)
#ifdef CFG80211_IFTYPE_MLO_LINK_SUPPORT
static
void wlan_hdd_register_ml_link(struct hdd_adapter *sta_adapter,
			       struct hdd_adapter *link_adapter)
{
	int ret;

	link_adapter->wdev.iftype = NL80211_IFTYPE_MLO_LINK;
	mutex_lock(&sta_adapter->wdev.mtx);
	ret = cfg80211_register_sta_mlo_link(&sta_adapter->wdev,
					     &link_adapter->wdev);
	mutex_unlock(&sta_adapter->wdev.mtx);

	if (ret) {
		hdd_err("Failed to register ml link wdev %d", ret);
		return;
	}
}

static
void wlan_hdd_unregister_ml_link(struct hdd_adapter *link_adapter,
				 bool rtnl_held)
{
	if (rtnl_held)
		rtnl_unlock();

	cfg80211_unregister_wdev(&link_adapter->wdev);

	if (rtnl_held)
		rtnl_lock();
}
#else
static
void wlan_hdd_register_ml_link(struct hdd_adapter *sta_adapter,
			       struct hdd_adapter *link_adapter)
{
}

static
void wlan_hdd_unregister_ml_link(struct hdd_adapter *link_adapter,
				 bool rtnl_held)
{
}
#endif

void hdd_register_wdev(struct hdd_adapter *sta_adapter,
		       struct hdd_adapter *link_adapter,
		       struct hdd_adapter_create_param *adapter_params)
{
	int  i;

	hdd_enter_dev(sta_adapter->dev);
	/* Set the relation between adapters*/
	wlan_hdd_register_ml_link(sta_adapter, link_adapter);
	sta_adapter->mlo_adapter_info.is_ml_adapter = true;
	sta_adapter->mlo_adapter_info.is_link_adapter = false;
	link_adapter->mlo_adapter_info.is_link_adapter = true;
	link_adapter->mlo_adapter_info.is_ml_adapter = false;
	link_adapter->mlo_adapter_info.ml_adapter = sta_adapter;
	link_adapter->mlo_adapter_info.associate_with_ml_adapter =
				      adapter_params->associate_with_ml_adapter;
	qdf_set_bit(WDEV_ONLY_REGISTERED, &link_adapter->event_flags);

	for (i = 0; i < WLAN_MAX_MLD; i++) {
		if (sta_adapter->mlo_adapter_info.link_adapter[i])
			continue;
		sta_adapter->mlo_adapter_info.link_adapter[i] = link_adapter;
		break;
	}

	qdf_mem_copy(link_adapter->mld_addr.bytes, sta_adapter->mld_addr.bytes,
		     QDF_MAC_ADDR_SIZE);
	hdd_exit();
}

static
void hdd_mlo_close_adapter(struct hdd_adapter *link_adapter, bool rtnl_held)
{
	struct osif_vdev_sync *vdev_sync;

	vdev_sync = osif_vdev_sync_unregister(link_adapter->dev);
	if (vdev_sync)
		osif_vdev_sync_wait_for_ops(vdev_sync);

	hdd_check_for_net_dev_ref_leak(link_adapter);
	wlan_hdd_release_intf_addr(link_adapter->hdd_ctx,
				   link_adapter->mac_addr.bytes);
	policy_mgr_clear_concurrency_mode(link_adapter->hdd_ctx->psoc,
					  link_adapter->device_mode);
	link_adapter->wdev.netdev = NULL;

	wlan_hdd_unregister_ml_link(link_adapter, rtnl_held);
	free_netdev(link_adapter->dev);

	if (vdev_sync)
		osif_vdev_sync_destroy(vdev_sync);
}

QDF_STATUS hdd_wlan_unregister_mlo_interfaces(struct hdd_adapter *adapter,
					      bool rtnl_held)
{
	int i;
	struct hdd_mlo_adapter_info *mlo_adapter_info;
	struct hdd_adapter *link_adapter;
	struct qdf_mac_addr adapter_mac;

	mlo_adapter_info = &adapter->mlo_adapter_info;

	if (mlo_adapter_info->is_link_adapter) {
		qdf_copy_macaddr(&adapter_mac, &adapter->mac_addr);
		ucfg_dp_destroy_intf(adapter->hdd_ctx->psoc, &adapter_mac);
		hdd_remove_front_adapter(adapter->hdd_ctx, &adapter);
		return QDF_STATUS_E_AGAIN;
	}

	for (i = 0; i < WLAN_MAX_MLD; i++) {
		link_adapter = mlo_adapter_info->link_adapter[i];
		if (!link_adapter)
			continue;
		qdf_copy_macaddr(&adapter_mac, &link_adapter->mac_addr);
		ucfg_dp_destroy_intf(link_adapter->hdd_ctx->psoc, &adapter_mac);
		hdd_remove_adapter(link_adapter->hdd_ctx, link_adapter);
		hdd_mlo_close_adapter(link_adapter, rtnl_held);
	}

	return QDF_STATUS_SUCCESS;
}

void hdd_wlan_register_mlo_interfaces(struct hdd_context *hdd_ctx)
{
	uint8_t *mac_addr;
	struct hdd_adapter_create_param params = {0};
	QDF_STATUS status;

	mac_addr = wlan_hdd_get_intf_addr(hdd_ctx, QDF_STA_MODE);
	if (mac_addr) {
		/* if target supports MLO create a new dev */
		params.only_wdev_register = true;
		params.associate_with_ml_adapter = false;
		status = hdd_open_adapter_no_trans(hdd_ctx, QDF_STA_MODE,
						   "null", mac_addr, &params);
		if (QDF_IS_STATUS_ERROR(status))
			hdd_err("Failed to register link adapter:%d", status);
	}

	qdf_mem_zero(&params, sizeof(struct hdd_adapter_create_param));
	params.only_wdev_register  = true;
	params.associate_with_ml_adapter = true;
	mac_addr = wlan_hdd_get_intf_addr(hdd_ctx, QDF_STA_MODE);
	if (mac_addr) {
		/* if target supports MLO create a new dev */
		status = hdd_open_adapter_no_trans(hdd_ctx, QDF_STA_MODE,
						   "null", mac_addr, &params);
		if (QDF_IS_STATUS_ERROR(status))
			hdd_err("Failed to register link adapter:%d", status);
	}
}

#ifdef CFG80211_MLD_MAC_IN_WDEV
static inline
void wlan_hdd_populate_mld_address(struct hdd_adapter *adapter,
				   uint8_t *mld_addr)
{
	qdf_mem_copy(adapter->wdev.mld_address, mld_addr,
		     QDF_NET_MAC_ADDR_MAX_LEN);
}
#else
static inline
void wlan_hdd_populate_mld_address(struct hdd_adapter *adapter,
				   uint8_t *mld_addr)
{
}
#endif
void
hdd_adapter_set_ml_adapter(struct hdd_adapter *adapter)
{
	adapter->mlo_adapter_info.is_ml_adapter = true;
}

struct hdd_adapter *hdd_get_ml_adater(struct hdd_context *hdd_ctx)
{
	struct hdd_adapter *adapter, *next_adapter = NULL;
	wlan_net_dev_ref_dbgid dbgid = NET_DEV_HOLD_GET_ADAPTER_BY_VDEV;

	hdd_for_each_adapter_dev_held_safe(hdd_ctx, adapter, next_adapter,
					   dbgid) {
		if (hdd_adapter_is_ml_adapter(adapter)) {
			hdd_adapter_dev_put_debug(adapter, dbgid);
			if (next_adapter)
				hdd_adapter_dev_put_debug(next_adapter,
							  dbgid);
			return adapter;
		}
		hdd_adapter_dev_put_debug(adapter, dbgid);
	}

	return NULL;
}

#ifdef WLAN_FEATURE_DYNAMIC_MAC_ADDR_UPDATE
int hdd_update_vdev_mac_address(struct hdd_context *hdd_ctx,
				struct hdd_adapter *adapter,
				struct qdf_mac_addr mac_addr)
{
	int i, ret = 0;
	struct hdd_mlo_adapter_info *mlo_adapter_info;
	struct hdd_adapter *link_adapter;
	bool eht_capab;

	ucfg_psoc_mlme_get_11be_capab(hdd_ctx->psoc, &eht_capab);
	if (hdd_adapter_is_ml_adapter(adapter) && eht_capab) {
		mlo_adapter_info = &adapter->mlo_adapter_info;

		for (i = 0; i < WLAN_MAX_MLD; i++) {
			link_adapter = mlo_adapter_info->link_adapter[i];
			if (!link_adapter)
				continue;
			ret = hdd_dynamic_mac_address_set(hdd_ctx, link_adapter,
							  mac_addr);
			if (ret)
				return ret;
		}
	} else {
		ret = hdd_dynamic_mac_address_set(hdd_ctx, adapter, mac_addr);
	}

	return ret;
}
#endif
#endif
