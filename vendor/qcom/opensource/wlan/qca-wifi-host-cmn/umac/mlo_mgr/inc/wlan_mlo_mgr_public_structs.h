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

/*
 * DOC: contains mlo manager structure definitions
 */
#ifndef __MLO_MGR_PUBLIC_STRUCTS_H
#define __MLO_MGR_PUBLIC_STRUCTS_H

#include <wlan_objmgr_cmn.h>
#include <qdf_list.h>
#include <qdf_atomic.h>
#include <qdf_nbuf.h>
#include <wlan_cmn_ieee80211.h>
#include <wlan_cmn.h>
#include <wlan_objmgr_global_obj.h>
#if defined(WLAN_FEATURE_11BE_MLO) && defined(WLAN_MLO_MULTI_CHIP)
#include <qdf_event.h>
#endif

/* MAX MLO dev support */
#ifndef WLAN_UMAC_MLO_MAX_VDEVS
#define WLAN_UMAC_MLO_MAX_VDEVS 2
#endif

/* MAX instances of ML devices */
#ifndef WLAN_UMAC_MLO_MAX_DEV
#define WLAN_UMAC_MLO_MAX_DEV 2
#endif

/* Max PEER support */
#define MAX_MLO_PEER 512

struct mlo_mlme_ext_ops;
struct vdev_mlme_obj;

/* Max LINK PEER support */
#define MAX_MLO_LINK_PEERS WLAN_UMAC_MLO_MAX_VDEVS

#define MAX_MLO_PEER_ID 2048
#define MLO_INVALID_PEER_ID 0xFFFF

/* IE nomenclature */
#define ID_POS 0
#define TAG_LEN_POS 1
#define IDEXT_POS 2
#define MIN_IE_LEN 2
#define MULTI_LINK_CTRL_1 3
#define MULTI_LINK_CTRL_2 4
#define STA_CTRL_1 2
#define STA_CTRL_2 3
#define STA_PROFILE_SUB_ELEM_ID 0
#define PER_STA_PROF_MAC_ADDR_START 4

#ifdef WLAN_MLO_MULTI_CHIP
/*
 * enum MLO_LINK_STATE – MLO link state enums
 * @MLO_LINK_SETUP_INIT - MLO link SETUP exchange not yet done
 * @MLO_LINK_SETUP_DONE - MLO link SETUP exchange started
 * @MLO_LINK_READY - MLO link SETUP done and READY sent
 * @MLO_LINK_TEARDOWN - MLO teardown done.
 * @MLO_LINK_UNINITIALIZED - MLO link in blank state
 */
enum MLO_LINK_STATE {
	MLO_LINK_SETUP_INIT,
	MLO_LINK_SETUP_DONE,
	MLO_LINK_READY,
	MLO_LINK_TEARDOWN,
	MLO_LINK_UNINITIALIZED,
};

/**
 * struct mlo_setup_info: MLO setup status per link
 * @ml_grp_id: Unique id for ML grouping of Pdevs/links
 * @tot_socs: Total number of soc participating in ML group
 * @num_soc: Number of soc ready or probed
 * @tot_links: Total links in ML group
 * @num_links: Number of links probed in ML group
 * @pdev_list[MAX_MLO_LINKS]: pdev pointers belonging to this group
 * @soc_list[MAX_MLO_CHIPS]: psoc pointers belonging to this group
 * @state[MAX_MLO_LINKS]: MLO link state
 * @valid_link_bitmap: valid MLO link bitmap
 * @state_lock: lock to protect access to link state
 * @qdf_event_t: event for teardown completion
 */
#define MAX_MLO_LINKS 6
#define MAX_MLO_CHIPS 5
struct mlo_setup_info {
	uint8_t ml_grp_id;
	uint8_t tot_socs;
	uint8_t num_soc;
	uint8_t tot_links;
	uint8_t num_links;
	struct wlan_objmgr_pdev *pdev_list[MAX_MLO_LINKS];
	struct wlan_objmgr_psoc *soc_list[MAX_MLO_CHIPS];
	enum MLO_LINK_STATE state[MAX_MLO_LINKS];
	uint16_t valid_link_bitmap;
	qdf_spinlock_t state_lock;
	qdf_event_t event;
};

/**
 * struct mlo_state_params: MLO state params for pdev iteration
 * @link_state_fail: Flag to check when pdev not in expected state
 * @check_state: State on against which pdev is to be expected
 */
struct mlo_state_params {
	bool link_state_fail;
	enum MLO_LINK_STATE check_state;
};

#define MAX_MLO_GROUP 1
#endif

/*
 * struct mlo_mgr_context - MLO manager context
 * @ml_dev_list_lock: ML DEV list lock
 * @aid_lock: AID global lock
 * @ml_peerid_lock: ML peer ID global lock
 * @context: Array of MLO device context
 * @mlo_peer_id_bmap: bitmap to allocate MLO Peer ID
 * @max_mlo_peer_id: Max MLO Peer ID
 * @info: MLO setup info
 * @mlme_ops: MLO MLME callback function pointers
 * @msgq_ctx: Context switch mgr
 * @mlo_is_force_primary_umac: Force Primary UMAC enable
 * @mlo_forced_primary_umac_id: Force Primary UMAC ID
 * @dp_handle: pointer to DP ML context
 */
struct mlo_mgr_context {
#ifdef WLAN_MLO_USE_SPINLOCK
	qdf_spinlock_t ml_dev_list_lock;
	qdf_spinlock_t aid_lock;
	qdf_spinlock_t ml_peerid_lock;
#else
	qdf_mutex_t ml_dev_list_lock;
	qdf_mutex_t aid_lock;
	qdf_mutex_t ml_peerid_lock;
#endif
	qdf_list_t ml_dev_list;
	qdf_bitmap(mlo_peer_id_bmap, MAX_MLO_PEER_ID);
	uint16_t max_mlo_peer_id;
#ifdef WLAN_MLO_MULTI_CHIP
	struct mlo_setup_info setup_info;
#endif
	struct mlo_mlme_ext_ops *mlme_ops;
	struct ctxt_switch_mgr *msgq_ctx;
	bool mlo_is_force_primary_umac;
	uint8_t mlo_forced_primary_umac_id;
	void *dp_handle;
};

/*
 * struct wlan_ml_vdev_aid_mgr – ML AID manager
 * @aid_bitmap: AID bitmap array
 * @start_aid: start of AID index
 * @max_aid: Max allowed AID
 * @aid_mgr[]:  Array of link vdev aid mgr
 */
struct wlan_ml_vdev_aid_mgr {
	qdf_bitmap(aid_bitmap, WLAN_UMAC_MAX_AID);
	uint16_t start_aid;
	uint16_t max_aid;
	struct wlan_vdev_aid_mgr *aid_mgr[WLAN_UMAC_MLO_MAX_VDEVS];
};

/*
 * struct wlan_mlo_key_mgmt - MLO key management
 * @link_mac_address: list of vdevs selected for connection with the MLAP
 * @vdev_id: vdev id value
 * @keys_saved: keys saved bool
 */
struct wlan_mlo_key_mgmt {
	struct qdf_mac_addr link_mac_address;
	uint8_t vdev_id;
	bool keys_saved;
};

/**
 * struct mlo_sta_csa _params - CSA request parameters in mlo mgr
 * @csa_param: csa parameters
 * @link_id: the link index of AP which triggers CSA
 * @mlo_csa_synced: Before vdev is up, csa information is only saved but not
 *                  handled, and this value is false. Once vdev is up, the saved
 *                  csa information is handled, and this value is changed to
 *                  true. Note this value will be true if the vdev is doing
 *                  restart.
 * @csa_offload_event_recvd: True if WMI_CSA_HANDLING_EVENTID is already
 *                           received. False if this is the first
 *                           WMI_CSA_HANDLING_EVENTID.
 * @valid_csa_param: True once csa_param is filled.
 */
struct mlo_sta_csa_params {
	struct csa_offload_params csa_param;
	uint8_t link_id;
	bool mlo_csa_synced;
	bool csa_offload_event_recvd;
	bool valid_csa_param;
};

/*
 * struct mlo_sta_quiet_status - MLO sta quiet status
 * @link_id: link id
 * @quiet_status: true if corresponding ap in quiet status
 * @valid_status: true if mlo_sta_quiet_status is filled
 */
struct mlo_sta_quiet_status {
	uint8_t link_id;
	bool quiet_status;
	bool valid_status;
};

/*
 * struct wlan_mlo_sta - MLO sta additional info
 * @wlan_connect_req_links: list of vdevs selected for connection with the MLAP
 * @wlan_connected_links: list of vdevs associated with this MLO connection
 * @connect req: connect params
 * @copied_conn_req: original connect req
 * @copied_conn_req_lock: lock for the original connect request
 * @assoc_rsp: Raw assoc response frame
 * @mlo_csa_param: CSA request parameters for mlo sta
 * @disconn_req: disconnect req params
 */
struct wlan_mlo_sta {
	qdf_bitmap(wlan_connect_req_links, WLAN_UMAC_MLO_MAX_VDEVS);
	qdf_bitmap(wlan_connected_links, WLAN_UMAC_MLO_MAX_VDEVS);
	struct wlan_mlo_key_mgmt key_mgmt[WLAN_UMAC_MLO_MAX_VDEVS - 1];
	struct wlan_cm_connect_req *connect_req;
	struct wlan_cm_connect_req *copied_conn_req;
#ifdef WLAN_MLO_USE_SPINLOCK
	qdf_spinlock_t copied_conn_req_lock;
#else
	qdf_mutex_t copied_conn_req_lock;
#endif
	struct element_info assoc_rsp;
	struct mlo_sta_quiet_status mlo_quiet_status[WLAN_UMAC_MLO_MAX_VDEVS];
	struct mlo_sta_csa_params mlo_csa_param[WLAN_UMAC_MLO_MAX_VDEVS];
	struct wlan_cm_disconnect_req *disconn_req;
};

/*
 * struct wlan_mlo_ap - MLO AP related info
 * @num_ml_vdevs: number of vdevs to form MLD
 * @ml_aid_mgr: ML AID mgr
 * @mlo_vdev_quiet_bmap: Bitmap of vdevs for which quiet ie needs to enabled
 */
struct wlan_mlo_ap {
	uint8_t num_ml_vdevs;
	struct wlan_ml_vdev_aid_mgr *ml_aid_mgr;
	qdf_bitmap(mlo_vdev_quiet_bmap, WLAN_UMAC_MLO_MAX_VDEVS);
};

/*
 * struct wlan_mlo_peer_list - MLO peer list entry
 * @peer_hash: MLO peer hash code
 * @peer_list_lock: lock to access members of structure
 */
struct wlan_mlo_peer_list {
	qdf_list_t peer_hash[WLAN_PEER_HASHSIZE];
#ifdef WLAN_MLO_USE_SPINLOCK
	qdf_spinlock_t peer_list_lock;
#else
	qdf_mutex_t peer_list_lock;
#endif
};

/*
 * struct wlan_mlo_dev_context - MLO device context
 * @node: QDF list node member
 * @mld_id: MLD id
 * @mld_addr: MLO device MAC address
 * @wlan_vdev_list: list of vdevs associated with this MLO connection
 * @wlan_vdev_count: number of elements in the vdev list
 * @mlo_peer: list peers in this MLO connection
 * @wlan_max_mlo_peer_count: peer count across the links of specific MLO
 * @mlo_dev_lock: lock to access struct
 * @tsf_recalculation_lock: Lock to protect TSF (re)calculation
 * @ref_cnt: reference count
 * @ref_id_dbg: Reference count debug information
 * @sta_ctx: MLO STA related information
 * @ap_ctx: AP related information
 */
struct wlan_mlo_dev_context {
	qdf_list_node_t node;
	uint8_t mld_id;
	struct qdf_mac_addr mld_addr;
	struct wlan_objmgr_vdev *wlan_vdev_list[WLAN_UMAC_MLO_MAX_VDEVS];
	uint16_t wlan_vdev_count;
	struct wlan_mlo_peer_list mlo_peer_list;
	uint16_t wlan_max_mlo_peer_count;
#ifdef WLAN_MLO_USE_SPINLOCK
	qdf_spinlock_t mlo_dev_lock;
	qdf_spinlock_t tsf_recalculation_lock;
#else
	qdf_mutex_t mlo_dev_lock;
	qdf_mutex_t tsf_recalculation_lock;
#endif
	qdf_atomic_t ref_cnt;
	qdf_atomic_t ref_id_dbg[WLAN_REF_ID_MAX];
	struct wlan_mlo_sta *sta_ctx;
	struct wlan_mlo_ap *ap_ctx;
};

/*
 * struct wlan_mlo_link_peer_entry – Link peer entry
 * @link_peer: Object manager peer
 * @link_addr: MAC address of link peer
 * @link_ix: Link index
 * @is_primary: sets true if the peer is primary UMAC’s peer
 * @hw_link_id: HW Link id of peer
 * @assoc_rsp_buf: Assoc resp buffer
 */
struct wlan_mlo_link_peer_entry {
	struct wlan_objmgr_peer *link_peer;
	struct qdf_mac_addr link_addr;
	uint8_t link_ix;
	bool is_primary;
	uint8_t hw_link_id;
	qdf_nbuf_t assoc_rsp_buf;
};

/*
 * enum mlo_peer_state – MLO peer state
 * @ML_PEER_CREATED:     Initial state
 * @ML_PEER_ASSOC_DONE:  ASSOC sent on assoc link
 * @ML_PEER_DISCONN_INITIATED: Disconnect initiated on one of the links
 */
enum mlo_peer_state {
	ML_PEER_CREATED,
	ML_PEER_ASSOC_DONE,
	ML_PEER_DISCONN_INITIATED,
};

#ifdef UMAC_SUPPORT_MLNAWDS
/*
 * struct mlnawds_config - MLO NAWDS configuration
 * @caps: Bandwidth & NSS capabilities to be configured on NAWDS peer
 * @puncture_bitmap: puncture bitmap to be configured on NAWDS peer
 * @mac: MAC address of the NAWDS peer to which the caps & puncture bitmap is
 * to be configured.
 */
struct mlnawds_config {
	uint64_t caps;
	uint16_t puncture_bitmap;
	uint8_t  mac[QDF_MAC_ADDR_SIZE];
};
#endif

/*
 * struct mlpeer_auth_params - Deferred Auth params
 * @vdev_id:  VDEV ID
 * @psoc_id:  PSOC ID
 * @link_addr: MAC address
 * @mldmacaddr: MLD MAC address
 * @algo:  Auth algorithm
 * @seq: Auth sequence number
 * @status_code: Auth status
 * @challenge: Auth Challenge
 * @challenge_length: Auth Challenge length
 * @wbuf:  Auth wbuf
 * @rs: Rx stats
 */
struct mlpeer_auth_params {
	uint8_t vdev_id;
	uint8_t psoc_id;
	struct qdf_mac_addr link_addr;
	struct qdf_mac_addr mldaddr;
	uint16_t algo;
	uint16_t seq;
	uint16_t status_code;
	uint8_t *challenge;
	uint8_t challenge_length;
	qdf_nbuf_t wbuf;
	void *rs;
};

#ifdef WLAN_FEATURE_11BE

/**
 * enum wlan_t2lm_direction - Indicates the direction for which TID-to-link
 * mapping is available.
 * @WLAN_T2LM_DL_DIRECTION: Downlink
 * @WLAN_T2LM_UL_DIRECTION: Uplink
 * @WLAN_T2LM_BIDI_DIRECTION: Both downlink and uplink
 * @WLAN_T2LM_MAX_DIRECTION: Max direction, this is used only internally
 * @WLAN_T2LM_INVALID_DIRECTION: Invalid, this is used only internally to check
 * if the mapping present in wlan_t2lm_of_tids structure is valid or not.
 */
enum wlan_t2lm_direction {
	WLAN_T2LM_DL_DIRECTION,
	WLAN_T2LM_UL_DIRECTION,
	WLAN_T2LM_BIDI_DIRECTION,
	WLAN_T2LM_MAX_DIRECTION,
	WLAN_T2LM_INVALID_DIRECTION,
};

/* Total 8 TIDs are supported, TID 0 to TID 7 */
#define T2LM_MAX_NUM_TIDS 8

/**
 * enum wlan_t2lm_category - T2LM category
 *
 * @WLAN_T2LM_CATEGORY_NONE: none
 * @WLAN_T2LM_CATEGORY_REQUEST: T2LM request
 * @WLAN_T2LM_CATEGORY_RESPONSE: T2LM response
 * @WLAN_T2LM_CATEGORY_TEARDOWN: T2LM teardown
 * @WLAN_T2LM_CATEGORY_INVALID: Invalid
 */
enum wlan_t2lm_category {
	WLAN_T2LM_CATEGORY_NONE = 0,
	WLAN_T2LM_CATEGORY_REQUEST = 1,
	WLAN_T2LM_CATEGORY_RESPONSE = 2,
	WLAN_T2LM_CATEGORY_TEARDOWN = 3,
	WLAN_T2LM_CATEGORY_INVALID,
};

/**
 * enum wlan_t2lm_tx_status - Status code applicable for the T2LM frames
 * transmitted by the current peer.
 *
 * @WLAN_T2LM_TX_STATUS_NONE: Status code is not applicable
 * @WLAN_T2LM_TX_STATUS_SUCCESS: AP/STA successfully transmitted the T2LM frame
 * @WLAN_T2LM_TX_STATUS_FAILURE: Tx failure received from the FW.
 * @WLAN_T2LM_TX_STATUS_RX_TIMEOUT: T2LM response frame not received from the
 *                              peer for the transmitted T2LM request frame.
 * @WLAN_T2LM_TX_STATUS_INVALID: Invalid status code
 */
enum wlan_t2lm_tx_status {
	WLAN_T2LM_TX_STATUS_NONE = 0,
	WLAN_T2LM_TX_STATUS_SUCCESS = 1,
	WLAN_T2LM_TX_STATUS_FAILURE = 2,
	WLAN_T2LM_TX_STATUS_RX_TIMEOUT = 3,
	WLAN_T2LM_TX_STATUS_INVALID,
};

/**
 * enum wlan_t2lm_resp_frm_type - T2LM status corresponds to T2LM response frame
 *
 * @WLAN_T2LM_RESP_TYPE_SUCCESS: T2LM mapping provided in the T2LM request is
 *                       accepted either by the AP or STA
 * @WLAN_T2LM_RESP_TYPE_DENIED_TID_TO_LINK_MAPPING: T2LM Request denied because
 *                       the requested TID-to-link mapping is unacceptable.
 * @WLAN_T2LM_RESP_TYPE_PREFERRED_TID_TO_LINK_MAPPING: T2LM Request rejected and
 *                       preferred TID-to-link mapping is suggested.
 * @WLAN_T2LM_RESP_TYPE_INVALID: Status code is not applicable.
 */
enum wlan_t2lm_resp_frm_type {
	WLAN_T2LM_RESP_TYPE_SUCCESS = 0,
	WLAN_T2LM_RESP_TYPE_DENIED_TID_TO_LINK_MAPPING = 133,
	WLAN_T2LM_RESP_TYPE_PREFERRED_TID_TO_LINK_MAPPING = 134,
	WLAN_T2LM_RESP_TYPE_INVALID,
};

/**
 * enum wlan_t2lm_enable - TID-to-link negotiation supported by the mlo peer
 *
 * @WLAN_T2LM_NOT_SUPPORTED: T2LM is not supported by the MLD
 * @WLAN_MAP_EACH_TID_TO_SAME_OR_DIFFERENET_LINK_SET: MLD supports the mapping
 *             of each TID to the same or different link set (Disjoint mapping).
 * @WLAN_MAP_ALL_TIDS_TO_SAME_LINK_SET: MLD only supports the mapping of all
 *             TIDs to the same link set.
 * @WLAN_T2LM_ENABLE_INVALID: invalid
 */
enum wlan_t2lm_enable {
	WLAN_T2LM_NOT_SUPPORTED = 0,
	WLAN_MAP_EACH_TID_TO_SAME_OR_DIFFERENET_LINK_SET = 1,
	WLAN_MAP_ALL_TIDS_TO_SAME_LINK_SET = 2,
	WLAN_T2LM_ENABLE_INVALID,
};

/**
 * struct wlan_t2lm_of_tids - TID-to-Link mapping information for the frames
 * transmitted on the uplink, downlink and bidirectional.
 *
 * @is_homogeneous_mapping: The t2lm_provisioned_links is homogeneous mapping
 * @direction:  0 - Downlink, 1 - uplink 2 - Both uplink and downlink
 * @default_link_mapping: value 1 indicates the default T2LM, where all the TIDs
 *                        are mapped to all the links.
 *                        value 0 indicates the preferred T2LM mapping
 * @t2lm_provisioned_links: Indicates TID to link mapping of all the TIDS.
 */
struct wlan_t2lm_of_tids {
	bool is_homogeneous_mapping;
	enum wlan_t2lm_direction direction;
	bool default_link_mapping;
	uint16_t t2lm_provisioned_links[T2LM_MAX_NUM_TIDS];
};

/**
 * struct wlan_prev_t2lm_negotiated_info - Previous successful T2LM negotiation
 * is saved here.
 *
 * @dialog_token: Save the dialog token used in T2LM request and response frame.
 * @t2lm_info: Provides the TID to LINK mapping information
 */
struct wlan_prev_t2lm_negotiated_info {
	uint16_t dialog_token;
	struct wlan_t2lm_of_tids t2lm_info[WLAN_T2LM_MAX_DIRECTION];
};

/**
 * struct wlan_t2lm_onging_negotiation_info - Current ongoing T2LM negotiation
 * (information about transmitted T2LM request/response frame)
 *
 * @category: T2LM category as T2LM request frame
 * @dialog_token: Save the dialog token used in T2LM request and response frame.
 * @t2lm_info: Provides the TID-to-link mapping info for UL/DL/BiDi
 * @t2lm_tx_status: Status code corresponds to the transmitted T2LM frames
 * @t2lm_resp_type: T2LM status corresponds to T2LM response frame.
 */
struct wlan_t2lm_onging_negotiation_info {
	enum wlan_t2lm_category category;
	uint8_t dialog_token;
	struct wlan_t2lm_of_tids t2lm_info[WLAN_T2LM_MAX_DIRECTION];
	enum wlan_t2lm_tx_status t2lm_tx_status;
	enum wlan_t2lm_resp_frm_type t2lm_resp_type;
};

/**
 * struct wlan_mlo_peer_t2lm_policy - TID-to-link mapping information
 *
 * @self_gen_dialog_token: self generated dialog token used to send T2LM request
 *                         frame;
 * @t2lm_enable_val: TID-to-link enable value supported by this peer.
 * @t2lm_negotiated_info: Previous successful T2LM negotiation is saved here.
 * @ongoing_tid_to_link_mapping: This has the ongoing TID-to-link mapping info
 *                               transmitted by this peer to the connected peer.
 */
struct wlan_mlo_peer_t2lm_policy {
	uint8_t self_gen_dialog_token;
	enum wlan_t2lm_enable t2lm_enable_val;
	struct wlan_prev_t2lm_negotiated_info t2lm_negotiated_info;
	struct wlan_t2lm_onging_negotiation_info ongoing_tid_to_link_mapping;
};
#endif /* WLAN_FEATURE_11BE */

/**
 * struct wlan_mlo_eml_cap - EML capabilities of MLD
 * @emlsr_supp: eMLSR Support
 * @emlsr_pad_delay: eMLSR Padding Delay
 * @emlsr_trans_delay: eMLSR transition delay
 * @emlmr_supp: eMLMR Support
 * @emlmr_delay: eMLMR Delay
 * @trans_timeout: Transition Timeout
 * @reserved: Reserved
 */
struct wlan_mlo_eml_cap {
	uint16_t emlsr_supp:1,
		 emlsr_pad_delay:3,
		 emlsr_trans_delay:3,
		 emlmr_supp:1,
		 emlmr_delay:3,
		 trans_timeout:4,
		 reserved:1;
};

/**
 * struct wlan_mlo_msd_cap - MSD capabilities of MLD
 * @medium_sync_duration: Medium Sync Duration
 * @medium_sync_ofdm_ed_thresh: MSD threshold value
 * @medium_sync_max_txop_num: Max number of TXOP
 */
struct wlan_mlo_msd_cap {
	uint16_t medium_sync_duration:8,
		 medium_sync_ofdm_ed_thresh:4,
		 medium_sync_max_txop_num:4;
};

/**
 * struct wlan_mlo_mld_cap - MLD capabilities of MLD
 * @max_simult_link: Maximum number of simultaneous links
 * @srs_support: SRS support
 * @tid2link_neg_support: TID to Link Negotiation Support
 * @str_freq_sep: Frequency separation suggested by STR non-AP MLD
 *                OR Type of AP-MLD
 * @aar_support: AAR Support
 * @reserved: Reserved
 */
struct wlan_mlo_mld_cap {
	uint16_t max_simult_link:4,
		 srs_support:1,
		 tid2link_neg_support:2,
		 str_freq_sep:5,
		 aar_support:1,
		 reserved:3;
};

/*
 * struct wlan_mlo_peer_context - MLO peer context
 *
 * @peer_node:     peer list node for ml_dev qdf list
 * @peer_list: list of peers on the MLO link
 * @link_peer_cnt: Number of link peers attached
 * @max_links: Max links for this ML peer
 * @mlo_peer_id: unique ID for the peer
 * @peer_mld_addr: MAC address of MLD link
 * @mlo_ie: MLO IE struct
 * @mlo_peer_lock: lock to access peer structure
 * @assoc_id: Assoc ID derived by MLO manager
 * @ref_cnt: Reference counter to avoid use after free
 * @ml_dev: MLO dev context
 * @mlpeer_state: MLO peer state
 * @avg_link_rssi: avg RSSI of ML peer
 * @is_nawds_ml_peer: flag to indicate if ml_peer is NAWDS configured
 * @nawds_config: eack link peer's NAWDS configuration
 * @pending_auth: Holds pending auth request
 * @t2lm_policy: TID-to-link mapping information
 * @msd_cap_present: Medium Sync Capability present bit
 * @mlpeer_emlcap: EML capability information for ML peer
 * @mlpeer_msdcap: Medium Sync Delay capability information for ML peer
 */
struct wlan_mlo_peer_context {
	qdf_list_node_t peer_node;
	struct wlan_mlo_link_peer_entry peer_list[MAX_MLO_LINK_PEERS];
	uint8_t link_peer_cnt;
	uint8_t max_links;
	uint32_t mlo_peer_id;
	struct qdf_mac_addr peer_mld_addr;
	uint8_t *mlo_ie;
#ifdef WLAN_MLO_USE_SPINLOCK
	qdf_spinlock_t mlo_peer_lock;
#else
	qdf_mutex_t mlo_peer_lock;
#endif
	uint16_t assoc_id;
	uint8_t primary_umac_psoc_id;
	qdf_atomic_t ref_cnt;
	struct wlan_mlo_dev_context *ml_dev;
	enum mlo_peer_state mlpeer_state;
	int8_t avg_link_rssi;
#ifdef UMAC_SUPPORT_MLNAWDS
	bool is_nawds_ml_peer;
	struct mlnawds_config nawds_config[MAX_MLO_LINK_PEERS];
#endif
#ifdef UMAC_MLO_AUTH_DEFER
	struct mlpeer_auth_params *pending_auth[MAX_MLO_LINK_PEERS];
#endif
#ifdef WLAN_FEATURE_11BE
	struct wlan_mlo_peer_t2lm_policy t2lm_policy;
#endif
	bool msd_cap_present;
	struct wlan_mlo_eml_cap mlpeer_emlcap;
	struct wlan_mlo_msd_cap mlpeer_msdcap;
};

/*
 * struct mlo_link_info – ML link info
 * @link_addr: link mac address
 * @link_id: link index
 * @chan_freq: Operating channel frequency
 * @nawds_config: peer's NAWDS configurarion
 * @vdev_id: VDEV ID
 */
struct mlo_link_info {
	struct qdf_mac_addr link_addr;
	uint8_t link_id;
	uint16_t chan_freq;
#ifdef UMAC_SUPPORT_MLNAWDS
	struct mlnawds_config nawds_config;
#endif
	uint8_t vdev_id;
};

/*
 * struct mlo_partner_info – mlo partner link info
 * @num_partner_links: no. of partner links
 * @partner_link_info: per partner link info
 * @t2lm_enable_val: enum wlan_t2lm_enable
 */
struct mlo_partner_info {
	uint8_t num_partner_links;
	struct mlo_link_info partner_link_info[WLAN_UMAC_MLO_MAX_VDEVS];
#ifdef WLAN_FEATURE_11BE
	enum wlan_t2lm_enable t2lm_enable_val;
#endif
};

/*
 * struct mlo_probereq_info – mlo probe req link info
 * @num_links: no. of link info in probe req
 * @link_id: target link id of APs
 */
struct mlo_probereq_info {
	uint8_t mlid;
	uint8_t num_links;
	uint8_t link_id[WLAN_UMAC_MLO_MAX_VDEVS];
};

/*
 * struct mlo_tgt_link_info – ML target link info
 * @vdev_id: link peer vdev id
 * @hw_mld_link_id: HW link id
 */
struct mlo_tgt_link_info {
	uint8_t vdev_id;
	uint8_t hw_mld_link_id;
};

/*
 * struct mlo_tgt_partner_info – mlo target partner link info
 * @num_partner_links: no. of partner links
 * @link_info: per partner link info
 */
struct mlo_tgt_partner_info {
	uint8_t num_partner_links;
	struct mlo_tgt_link_info link_info[WLAN_UMAC_MLO_MAX_VDEVS];
};

/*
 * struct mlo_mlme_ext_ops - MLME callback functions
 * @mlo_mlme_ext_validate_conn_req: Callback to validate connect request
 * @mlo_mlme_ext_create_link_vdev: Callback to create link vdev for ML STA
 * @mlo_mlme_ext_peer_create: Callback to create link peer
 * @mlo_mlme_ext_peer_assoc: Callback to initiate peer assoc
 * @mlo_mlme_ext_peer_assoc_fail: Callback to notify peer assoc failure
 * @mlo_mlme_ext_peer_delete: Callback to initiate link peer delete
 * @mlo_mlme_ext_assoc_resp: Callback to initiate assoc resp
 * @mlo_mlme_get_link_assoc_req: Callback to get link assoc req buffer
 * @mlo_mlme_ext_deauth: Callback to initiate deauth
 * @mlo_mlme_ext_clone_security_param: Callback to clone mlo security params
 * @mlo_mlme_ext_peer_process_auth: Callback to process pending auth
 * @mlo_mlme_ext_handle_sta_csa_param: Callback to handle sta csa param
 */
struct mlo_mlme_ext_ops {
	QDF_STATUS (*mlo_mlme_ext_validate_conn_req)(
		    struct vdev_mlme_obj *vdev_mlme, void *ext_data);
	QDF_STATUS (*mlo_mlme_ext_create_link_vdev)(
		    struct vdev_mlme_obj *vdev_mlme, void *ext_data);
	QDF_STATUS (*mlo_mlme_ext_peer_create)(struct wlan_objmgr_vdev *vdev,
					struct wlan_mlo_peer_context *ml_peer,
					struct qdf_mac_addr *addr,
					qdf_nbuf_t frm_buf);
	void (*mlo_mlme_ext_peer_assoc)(struct wlan_objmgr_peer *peer);
	void (*mlo_mlme_ext_peer_assoc_fail)(struct wlan_objmgr_peer *peer);
	void (*mlo_mlme_ext_peer_delete)(struct wlan_objmgr_peer *peer);
	void (*mlo_mlme_ext_assoc_resp)(struct wlan_objmgr_peer *peer);
	qdf_nbuf_t (*mlo_mlme_get_link_assoc_req)(struct wlan_objmgr_peer *peer,
						  uint8_t link_ix);
	void (*mlo_mlme_ext_deauth)(struct wlan_objmgr_peer *peer);
	QDF_STATUS (*mlo_mlme_ext_clone_security_param)(
		    struct vdev_mlme_obj *vdev_mlme,
		    struct wlan_cm_connect_req *req);
#ifdef UMAC_MLO_AUTH_DEFER
	void (*mlo_mlme_ext_peer_process_auth)(
	      struct mlpeer_auth_params *auth_param);
#endif
	void (*mlo_mlme_ext_handle_sta_csa_param)(
				struct wlan_objmgr_vdev *vdev,
				struct csa_offload_params *csa_param);
	QDF_STATUS (*mlo_mlme_ext_sta_op_class)(
			struct vdev_mlme_obj *vdev_mlme,
			uint8_t *ml_ie);

};

/* maximum size of vdev bitmap array for MLO link set active command */
#define MLO_VDEV_BITMAP_SZ 2

/* maximum size of link number param array for MLO link set active command */
#define MLO_LINK_NUM_SZ 2

/**
 * enum mlo_link_force_mode: MLO link force modes
 * @MLO_LINK_FORCE_MODE_ACTIVE:
 *  Force specific links active
 * @MLO_LINK_FORCE_MODE_INACTIVE:
 *  Force specific links inactive
 * @MLO_LINK_FORCE_MODE_ACTIVE_NUM:
 *  Force active a number of links, firmware to decide which links to inactive
 * @MLO_LINK_FORCE_MODE_INACTIVE_NUM:
 *  Force inactive a number of links, firmware to decide which links to inactive
 * @MLO_LINK_FORCE_MODE_NO_FORCE:
 *  Cancel the force operation of specific links, allow firmware to decide
 */
enum mlo_link_force_mode {
	MLO_LINK_FORCE_MODE_ACTIVE       = 1,
	MLO_LINK_FORCE_MODE_INACTIVE     = 2,
	MLO_LINK_FORCE_MODE_ACTIVE_NUM   = 3,
	MLO_LINK_FORCE_MODE_INACTIVE_NUM = 4,
	MLO_LINK_FORCE_MODE_NO_FORCE     = 5,
};

/**
 * enum mlo_link_force_reason: MLO link force reasons
 * @MLO_LINK_FORCE_REASON_CONNECT:
 *  Set force specific links because of new connection
 * @MLO_LINK_FORCE_REASON_DISCONNECT:
 *  Set force specific links because of new dis-connection
 */
enum mlo_link_force_reason {
	MLO_LINK_FORCE_REASON_CONNECT    = 1,
	MLO_LINK_FORCE_REASON_DISCONNECT = 2,
};

/**
 * struct mlo_link_set_active_resp: MLO link set active response structure
 * @status: Return status, 0 for success, non-zero otherwise
 * @active_sz: size of current active vdev bitmap array
 * @active: current active vdev bitmap array
 * @inactive_sz: size of current inactive vdev bitmap array
 * @inactive: current inactive vdev bitmap array
 */
struct mlo_link_set_active_resp {
	uint32_t status;
	uint32_t active_sz;
	uint32_t active[MLO_VDEV_BITMAP_SZ];
	uint32_t inactive_sz;
	uint32_t inactive[MLO_VDEV_BITMAP_SZ];
};

/**
 * struct mlo_link_num_param: MLO link set active number params
 * @num_of_link: number of links to active/inactive
 * @vdev_type: type of vdev
 * @vdev_subtype: subtype of vdev
 * @home_freq: home frequency of the link
 */
struct mlo_link_num_param {
	uint32_t num_of_link;
	uint32_t vdev_type;
	uint32_t vdev_subtype;
	uint32_t home_freq;
};

/**
 * struct mlo_link_set_active_param: MLO link set active params
 * @force_mode: operation to take (enum mlo_link_force_mode)
 * @reason: reason for the operation (enum mlo_link_force_reason)
 * @num_link_entry: number of the valid entries for link_num
 * @num_vdev_bitmap: number of the valid entries for vdev_bitmap
 * @link_num: link number param array
 *  It's present only when force_mode is MLO_LINK_FORCE_MODE_ACTIVE_NUM or
 *  MLO_LINK_FORCE_MODE_INACTIVE_NUM
 * @vdev_bitmap: active/inactive vdev bitmap array
 *  It will be present when force_mode is MLO_LINK_FORCE_MODE_ACTIVE,
 *  MLO_LINK_FORCE_MODE_INACTIVE, MLO_LINK_FORCE_MODE_NO_FORCE,
 *  MLO_LINK_FORCE_MODE_ACTIVE_NUM or MLO_LINK_FORCE_MODE_INACTIVE_NUM
 */
struct mlo_link_set_active_param {
	uint32_t force_mode;
	uint32_t reason;
	uint32_t num_link_entry;
	uint32_t num_vdev_bitmap;
	struct mlo_link_num_param link_num[MLO_LINK_NUM_SZ];
	uint32_t vdev_bitmap[MLO_VDEV_BITMAP_SZ];
};

/*
 * struct mlo_link_set_active_ctx - Context for MLO link set active request
 * @vdev: pointer to vdev on which the request issued
 * @set_mlo_link_cb: callback function for MLO link set active request
 * @validate_set_mlo_link_cb: callback to validate set link request
 * @cb_arg: callback context
 */
struct mlo_link_set_active_ctx {
	struct wlan_objmgr_vdev *vdev;
	void (*set_mlo_link_cb)(struct wlan_objmgr_vdev *vdev, void *arg,
				struct mlo_link_set_active_resp *evt);
	QDF_STATUS (*validate_set_mlo_link_cb)(
			struct wlan_objmgr_psoc *psoc,
			struct mlo_link_set_active_param *param);
	void *cb_arg;
};

/*
 * struct mlo_link_set_active_req - MLO link set active request
 * @ctx: context for MLO link set active request
 * @param: MLO link set active params
 */
struct mlo_link_set_active_req {
	struct mlo_link_set_active_ctx ctx;
	struct mlo_link_set_active_param param;
};

/*
 * enum mlo_chip_recovery_type - MLO chip recovery types
 * @MLO_RECOVERY_MODE_0: CRASH_PARTNER_CHIPS & recover all chips
 * @MLO_RECOVERY_MODE_1: Crash & recover asserted chip alone
 * @MLO_RECOVERY_MODE_MAX: Max limit for recovery types
 */
enum mlo_chip_recovery_type {
	MLO_RECOVERY_MODE_0 = 1,
	MLO_RECOVERY_MODE_1 = 2,

	/* Add new types above */
	MLO_RECOVERY_MODE_MAX = 0xf
};
#endif
