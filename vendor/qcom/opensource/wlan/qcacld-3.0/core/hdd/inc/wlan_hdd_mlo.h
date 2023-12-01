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
 * DOC : wlan_hdd_mlo.h
 *
 * WLAN Host Device Driver file for 802.11be (Extremely High Throughput)
 * support.
 *
 */
#if !defined(WLAN_HDD_MLO_H)
#define WLAN_HDD_MLO_H
#include <wlan_hdd_main.h>
#include "wlan_osif_features.h"

/**
 * struct hdd_adapter_create_param - adapter create parameters
 * @only_wdev_register:  Register only the wdev not the netdev
 * @associate_with_ml_adapter: Vdev points to the same netdev adapter
 * @is_ml_adapter: is a ml adapter with associated netdev
 * @is_add_virtual_iface: is netdev create request from add virtual interface
 */
struct hdd_adapter_create_param {
	uint32_t only_wdev_register:1,
		 associate_with_ml_adapter:1,
		 is_ml_adapter:1,
		 is_add_virtual_iface:1,
		 unused:28;
};

#if defined(WLAN_FEATURE_11BE_MLO) && defined(CFG80211_11BE_BASIC)
#define hdd_adapter_is_link_adapter(x) ((x)->mlo_adapter_info.is_link_adapter)
#define hdd_adapter_is_ml_adapter(x)   ((x)->mlo_adapter_info.is_ml_adapter)
#define hdd_adapter_is_associated_with_ml_adapter(x) \
			   ((x)->mlo_adapter_info.associate_with_ml_adapter)
#define hdd_adapter_get_mlo_adapter_from_link(x) \
			   ((x)->mlo_adapter_info.ml_adapter)
#else
#define hdd_adapter_is_link_adapter(x) (0)
#define hdd_adapter_is_ml_adapter(x)   (0)
#define hdd_adapter_is_associated_with_ml_adapter(x) (0)
#define hdd_adapter_get_mlo_adapter_from_link(x) (NULL)
#endif

#if defined(WLAN_FEATURE_11BE_MLO) && defined(CFG80211_11BE_BASIC)

/**
 * struct hdd_mlo_adapter_info - Mlo specific adapter information
 * @is_ml_adapter: Whether this is the main ml adaper attached to netdev
 * @is_link_adapter: Whether this a link adapter without netdev
 * @associate_with_ml_adapter: adapter which shares the vdev object with the ml
 * adapter
 * num_of_vdev_links: Num of vdevs/links part of the association
 * @ml_adapter: ML adapter backpointer
 * @link_adapter: backpointers to link adapters part of association
 */
struct hdd_mlo_adapter_info {
	uint32_t is_ml_adapter:1,
		 is_link_adapter:1,
		 associate_with_ml_adapter:1,
		 num_of_vdev_links:2,
		 unused:27;
	struct hdd_adapter *ml_adapter;
	struct hdd_adapter *link_adapter[WLAN_MAX_MLD];
};

/**
 * hdd_register_wdev() - Function to register only wdev
 * @sta_adapter : Station adapter linked with netdevice
 * @link_adapter: Link adapter
 * @adapter_params: Adapter params
 *
 * Function to register only the wdev not the netdev
 * Return: none
 */
void hdd_register_wdev(struct hdd_adapter *sta_adapter,
		       struct hdd_adapter *link_adapter,
		       struct hdd_adapter_create_param *adapter_params);
/**
 * hdd_wlan_unregister_mlo_interfaces() - Function to unregister mlo
 * interfaces
 * @adapter: Link adapter
 * @rtnl_held: RTNL held or not
 *
 * Function to unregister only the link adapter/wdev.
 * Return: none
 */
QDF_STATUS hdd_wlan_unregister_mlo_interfaces(struct hdd_adapter *adapter,
					      bool rtnl_held);

/**
 * hdd_wlan_register_mlo_interfaces() - Function to register mlo wdev interfaces
 * @hdd_ctx: hdd context
 *
 * Function to register mlo wdev interfaces.
 * Return: none
 */
void hdd_wlan_register_mlo_interfaces(struct hdd_context *hdd_ctx);

/**
 * hdd_adapter_set_ml_adapter() - set adapter as ml adapter
 * @adapter: HDD adapter
 *
 * This function sets adapter as ml adapter
 * Return: None
 */
void hdd_adapter_set_ml_adapter(struct hdd_adapter *adapter);

/**
 * hdd_get_ml_adater() - get an ml adapter
 * @adapter: HDD adapter
 *
 * This function returns ml adapter from adapter list
 * Return: adapter or NULL
 */
struct hdd_adapter *hdd_get_ml_adater(struct hdd_context *hdd_ctx);

/**
 * hdd_get_assoc_link_adapter() - get assoc link adapter
 * @ml_adapter: ML adapter
 *
 * This function returns assoc link adapter
 * Return: adapter or NULL
 */
struct hdd_adapter *hdd_get_assoc_link_adapter(struct hdd_adapter *ml_adapter);
#else
static inline
QDF_STATUS hdd_wlan_unregister_mlo_interfaces(struct hdd_adapter *adapter,
					      bool rtnl_held)
{
	return QDF_STATUS_SUCCESS;
}

static inline
void hdd_register_wdev(struct hdd_adapter *sta_adapter,
		       struct hdd_adapter *link_adapter,
		       struct hdd_adapter_create_param *adapter_params)
{
}

static inline
void hdd_wlan_register_mlo_interfaces(struct hdd_context *hdd_ctx)
{
}

static inline void
hdd_adapter_set_ml_adapter(struct hdd_adapter *adapter)
{
}

static inline
struct hdd_adapter *hdd_get_ml_adater(struct hdd_context *hdd_ctx)
{
	return NULL;
}

static inline
struct hdd_adapter *hdd_get_assoc_link_adapter(struct hdd_adapter *ml_adapter)
{
	return NULL;
}
#endif
#endif
