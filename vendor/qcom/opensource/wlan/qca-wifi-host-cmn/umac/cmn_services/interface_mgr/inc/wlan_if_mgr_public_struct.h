/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
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

/*
 * DOC: contains interface manager structure definitions
 */
#ifndef __IF_MGR_PUBLIC_STRUCTS_H
#define __IF_MGR_PUBLIC_STRUCTS_H

#include <scheduler_api.h>
#include <wlan_objmgr_psoc_obj.h>
#include <wlan_objmgr_pdev_obj.h>
#include <wlan_objmgr_vdev_obj.h>
#include <qdf_time.h>
#include <qdf_list.h>
#include <qdf_atomic.h>

/**
 * enum wlan_if_mgr_evt: interface manager events
 * @WLAN_IF_MGR_EV_CONNECT_START:Event to handle connect start request
 * @WLAN_IF_MGR_EV_CONNECT_COMPLETE:Event to handle connect start complete
 * @WLAN_IF_MGR_EV_DISCONNECT_START:Event to handle disconnect start request
 * @WLAN_IF_MGR_EV_DISCONNECT_COMPLETE:Event to handle disconnect start complete
 * @WLAN_IF_MGR_EV_VALIDATE_CANDIDATE:Event to validate candidate
 * @WLAN_IF_MGR_EV_AP_START_BSS:Event to handle start bss request
 * @WLAN_IF_MGR_EV_AP_START_BSS_COMPLETE:Event to handle start bss complete
 * @WLAN_IF_MGR_EV_AP_STOP_BSS:Event to handle stop bss request
 * @WLAN_IF_MGR_EV_AP_STOP_BSS_COMPLETE:Event to stop bss complete
 * WLAN_IF_MGR_EV_AP_START_ACS: Event to handle ACS start
 * WLAN_IF_MGR_EV_AP_STOP_ACS: Event to handle ACS stop
 * WLAN_IF_MGR_EV_AP_DONE_ACS: Event to handle ACS completion
 * WLAN_IF_MGR_EV_AP_CANCEL_ACS: Event to handle ACS cancel
 * WLAN_IF_MGR_EV_AP_START_HT40: Event to handle HT40 scan start
 * WLAN_IF_MGR_EV_AP_STOP_HT40: Event to handle HT40 scan stop
 * WLAN_IF_MGR_EV_AP_DONE_HT40: Event to handle HT40 scan completion
 * WLAN_IF_MGR_EV_AP_CANCEL_HT40: Event to handle HT40 scan cancel
 * WLAN_IF_MGR_EV_CSA_COMPLETE: Event to handle csa complete
 */
enum wlan_if_mgr_evt {
	WLAN_IF_MGR_EV_CONNECT_START = 0,
	WLAN_IF_MGR_EV_CONNECT_COMPLETE = 1,
	WLAN_IF_MGR_EV_DISCONNECT_START = 2,
	WLAN_IF_MGR_EV_DISCONNECT_COMPLETE = 3,
	WLAN_IF_MGR_EV_VALIDATE_CANDIDATE = 4,
	WLAN_IF_MGR_EV_AP_START_BSS = 5,
	WLAN_IF_MGR_EV_AP_START_BSS_COMPLETE = 6,
	WLAN_IF_MGR_EV_AP_STOP_BSS = 7,
	WLAN_IF_MGR_EV_AP_STOP_BSS_COMPLETE = 8,
	WLAN_IF_MGR_EV_AP_START_ACS = 9,
	WLAN_IF_MGR_EV_AP_STOP_ACS = 10,
	WLAN_IF_MGR_EV_AP_DONE_ACS = 11,
	WLAN_IF_MGR_EV_AP_CANCEL_ACS = 12,
	WLAN_IF_MGR_EV_AP_START_HT40 = 13,
	WLAN_IF_MGR_EV_AP_STOP_HT40 = 14,
	WLAN_IF_MGR_EV_AP_DONE_HT40 = 15,
	WLAN_IF_MGR_EV_AP_CANCEL_HT40 = 16,
	WLAN_IF_MGR_EV_CSA_COMPLETE = 17,
	WLAN_IF_MGR_EV_MAX = 18,
};

/**
 * struct validate_bss_data - interface manager validate candidate data
 * @peer_addr: MAC address of the BSS
 * @chan_freq: Frequency of the potential BSS connection
 * @beacon_interval: beacon interval of BSS
 * @is_mlo: indicate whether MLO is supported by the BSS or not
 * @scan_entry: scan entry data
 */
struct validate_bss_data {
	struct qdf_mac_addr peer_addr;
	qdf_freq_t chan_freq;
	uint16_t beacon_interval;
#ifdef WLAN_FEATURE_11BE_MLO
	bool is_mlo;
	struct scan_cache_entry *scan_entry;
#endif
};

/**
 * struct if_mgr_event_data - interface manager event data
 * @status: qdf status used to indicate if connect,disconnect,
 *	    start bss,stop bss event is success/failure.
 * @validate_bss_info: struct to hold the validate candidate information
 * @data: event data
 */
struct if_mgr_event_data {
	QDF_STATUS status;
	struct validate_bss_data validate_bss_info;
	void *data;
};

#endif
