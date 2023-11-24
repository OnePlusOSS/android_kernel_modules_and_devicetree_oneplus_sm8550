/******************************************************************************
** Copyright (C), 2019-2029, Oplus Mobile Comm Corp., Ltd
** Oplus_EDIT, All rights reserved.
** File: - wlan_oplus_smartmcc.c
** Description: wlan oplus smartmcc
**
** Version: 1.0
** Date : 2022/8/20
** TAG: OPLUS_FEATURE_WIFI_OPLUSWFD_SMARTMCC
** ------------------------------- Revision History: ----------------------------
** <author>                                <data>        <version>       <desc>
** ------------------------------------------------------------------------------
 *******************************************************************************/
#ifdef OPLUS_FEATURE_WIFI_OPLUSWFD_SMARTMCC
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/wireless.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/preempt.h>
#include <net/arp.h>
#include <net/cfg80211.h>
#include <net/mac80211.h>
#include <linux/nl80211.h>

/* QCom specific */
#include "wlan_hdd_main.h"
#include <wlan_hdd_includes.h>
#include "wlan_policy_mgr_ucfg.h"
#include "wma.h"
#include "lim_session_utils.h"
#include "wmi_tlv_helper.h"
#include "osif_vdev_sync.h"
#include "wmi_unified_priv.h"
#include "wlan_policy_mgr_api.h"
#include "wlan_hdd_p2p.h"
#include "wlan_p2p_mcc_quota_public_struct.h"

#define LOG_TAG_DEBUG "[OplusSmartMCCDebug] %s line:%d "
#define oplus_debug(fmt, args...) printk(LOG_TAG_DEBUG fmt, __FUNCTION__, __LINE__, ##args)
#define LOG_TAG_ERROR "[OplusSmartMCCError] %s line:%d "
#define oplus_error(fmt, args...) printk(LOG_TAG_ERROR fmt, __FUNCTION__, __LINE__, ##args)
bool check_private_miracast_cmd(uint8_t *sub_command, int *sta_quota);
int handle_private_miracast_cmd(struct hdd_adapter *adapter, int sta_quota);
void wlan_hdd_update_private_mcc_p2p_quota(struct hdd_adapter *adapter, int sta_quota);

/* only support from 8550T, FW may modify quota even in MAS disable mode due to TBTT or leaky AP issue
 * which will let the real MCC quota differnt from the set one
 * Ex: when with leaky AP, quota may change from 30/70=>75/25, when TBTT issue, may from 30/70=>60/150 etc...
 * It will let delay become high and hard to debug, this feature will easy for FWK debug
 */
#define SIZE_OF_FW_UPDATE_QUOTA_OUTPUT 128
int drv_cmd_smartmcc_get_fw_update_quota(struct hdd_adapter *adapter,
			  struct hdd_context *hdd_ctx,
			  uint8_t *command, uint8_t command_len,
			  struct hdd_priv_data *priv_data);
struct channel_quota fw_update_quota[MAX_MCC_QUOTA_CH_NUM];

bool check_private_miracast_cmd(uint8_t *sub_command, int *sta_quota)
{
	int sub_command_head;
	if (!sub_command) {
		oplus_debug("check_private_miracast_cmd fail: sub_command null");
		return false;
	}
	if (sscanf(sub_command, "%d %d", &sub_command_head, sta_quota) != 2) {
		oplus_debug("check_private_miracast_cmd fail: cmd format check fail, private sub cmd:%s", sub_command);
		return false;
	}
	if (sub_command_head != -1) {
		oplus_debug("check_private_miracast_cmd fail: sub cmd head check fail, private sub cmd:%s", sub_command);
		return false;
	}

	oplus_debug("private sub cmd:%s, sub_command_value(>=10 stands for sta_quota):%d", sub_command, *sta_quota);

	return true;
}

/* Qcom default code 5G is MAS enable, 2.4G MAS disable + force 30/70
 * when use default MIRACAST 0/1/2, only can control 2.4G from MAS enable <=> MAS disable + force 30/70
 * when use private cmd MIRACAST -1 -1/0/sta_quota, it can both control 2.4G/5G MAS enable/disable
 * since Qcom 2G 5G different design and have no default function to restore to default scenario
 * so restore to default scenario is complex:
 * 5G WFD on restore(MAS enable): MIRACAST -1 0
 * 5G WFD off resetore(MAS enable): MIRACAT -1 0 (MIRACAST 0 will default send to restore scan & low latency policy)
 * 2G WFD on restore(30/70 fixed): MIRACAST 1/2
 * 2G WFD off restore(MAS enable): needn't, MIRACAST 0 will default send, so can align with 5G WFD off
 * PS: MIRACAST -1 -1: check for driver smart mcc support or not
 */
int handle_private_miracast_cmd(struct hdd_adapter *adapter, int sta_quota)
{
	if (sta_quota == -1) {
		return 0;
	} else {
		// reach here stands FWK change the quota(set/restore), clear the fw_update_quota
		memset(fw_update_quota, '\0', sizeof(fw_update_quota));
		return wlan_hdd_set_mas(adapter, sta_quota);
	}
}

// Function to update private P2P quota, clone from wlan_hdd_update_mcc_p2p_quota
void wlan_hdd_update_private_mcc_p2p_quota(struct hdd_adapter *adapter, int sta_quota)
{
	uint8_t i;
	oplus_debug("Set STA quota: %d", sta_quota);
	if (sta_quota) {
		if (adapter->device_mode == QDF_STA_MODE)
			wlan_hdd_set_mcc_p2p_quota(adapter,
				sta_quota
			);
		else if (adapter->device_mode == QDF_P2P_GO_MODE)
			wlan_hdd_go_set_mcc_p2p_quota(adapter,
				100 - sta_quota);
		else
			wlan_hdd_set_mcc_p2p_quota(adapter,
				100 - sta_quota);
	} else {
		if (adapter->device_mode == QDF_P2P_GO_MODE)
			wlan_hdd_go_set_mcc_p2p_quota(adapter,
				HDD_RESET_MCC_P2P_QUOTA);
		else
			wlan_hdd_set_mcc_p2p_quota(adapter,
				HDD_RESET_MCC_P2P_QUOTA);
		for (i = 0; i < sizeof(fw_update_quota) / sizeof(fw_update_quota[0]); i++) {
			fw_update_quota[i].chan_mhz = 0;
			fw_update_quota[i].channel_time_quota = 0;
		}
	}
}

/**
* drv_cmd_smartmcc_get_fw_update_quota() - Helper function to get fw update quota
* @adapter: pointer to adapter on which request is received
* @hdd_ctx: pointer to hdd context
* @command: command name
* @command_len: command buffer length
* @priv_data: output pointer to hold fw update quota or default 0
*
* Return: On success 0, negative value on error.
*/
int drv_cmd_smartmcc_get_fw_update_quota(struct hdd_adapter *adapter,
			  struct hdd_context *hdd_ctx,
			  uint8_t *command, uint8_t command_len,
			  struct hdd_priv_data *priv_data)
{
	uint8_t buf[SIZE_OF_FW_UPDATE_QUOTA_OUTPUT] = {0};
	int ret = 0, len = 0, i;
	int need_print = 0;

	for (i = 0; i < sizeof(fw_update_quota) / sizeof(fw_update_quota[0]); i++) {
		if (fw_update_quota[i].chan_mhz > 0) {
			need_print = 1;
		}
		len += scnprintf(buf + len, sizeof(buf) - len, "%s %d %s %d %s %d ",
		    "chan:", i, "mhz:", fw_update_quota[i].chan_mhz, "quota:", fw_update_quota[i].channel_time_quota);
	}

	if (need_print) {
		oplus_debug("buf = %s", buf);
	}
	len = QDF_MIN(priv_data->total_len, len + 1);
	if (copy_to_user(priv_data->buf, buf, len)) {
	    hdd_err("failed to copy data to user buffer");
	    ret = -EFAULT;
	}

	return ret;
}
#endif
