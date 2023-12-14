// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/stringify.h>
#include "ipa_i.h"
#include "ipa_rm_i.h"
#include "ipahal_reg.h"
#include "ipahal_nat.h"
#include "ipa_odl.h"
#include "ipa_qmi_service.h"
#if defined(CONFIG_IPA_TSP)
/* The following line should be removed once TSP feature is POR */
#include "ipa_test_module_tsp.h"
#include "ipahal_tsp.h"
#endif
#define IPA_MAX_ENTRY_STRING_LEN 500
#define IPA_MAX_MSG_LEN 4096
#define IPA_DBG_MAX_RULE_IN_TBL 128
#define IPA_DBG_ACTIVE_CLIENT_BUF_SIZE ((IPA3_ACTIVE_CLIENTS_LOG_LINE_LEN \
	* IPA3_ACTIVE_CLIENTS_LOG_BUFFER_SIZE_LINES) + IPA_MAX_MSG_LEN)

#define IPA_DUMP_STATUS_FIELD(f) \
	pr_err(#f "=0x%x\n", status->f)

#define IPA_READ_ONLY_MODE  0444
#define IPA_READ_WRITE_MODE 0664
#define IPA_WRITE_ONLY_MODE 0220

struct ipa3_debugfs_file {
	const char *name;
	umode_t mode;
	void *data;
	const struct file_operations fops;
};

static const char * const ipa_eth_clients_strings[] = {
	__stringify(AQC107),
	__stringify(AQC113),
	__stringify(RTK8111K),
	__stringify(RTK8125B),
	__stringify(NTN),
	__stringify(NTN3),
	__stringify(EMAC),
};

const char *ipa3_event_name[IPA_EVENT_MAX_NUM] = {
	__stringify(WLAN_CLIENT_CONNECT),
	__stringify(WLAN_CLIENT_DISCONNECT),
	__stringify(WLAN_CLIENT_POWER_SAVE_MODE),
	__stringify(WLAN_CLIENT_NORMAL_MODE),
	__stringify(SW_ROUTING_ENABLE),
	__stringify(SW_ROUTING_DISABLE),
	__stringify(WLAN_AP_CONNECT),
	__stringify(WLAN_AP_DISCONNECT),
	__stringify(WLAN_STA_CONNECT),
	__stringify(WLAN_STA_DISCONNECT),
	__stringify(WLAN_CLIENT_CONNECT_EX),
	__stringify(WLAN_SWITCH_TO_SCC),
	__stringify(WLAN_SWITCH_TO_MCC),
	__stringify(WLAN_WDI_ENABLE),
	__stringify(WLAN_WDI_DISABLE),
	__stringify(WAN_UPSTREAM_ROUTE_ADD),
	__stringify(WAN_UPSTREAM_ROUTE_DEL),
	__stringify(WAN_EMBMS_CONNECT),
	__stringify(WAN_XLAT_CONNECT),
	__stringify(ECM_CONNECT),
	__stringify(ECM_DISCONNECT),
	__stringify(IPA_TETHERING_STATS_UPDATE_STATS),
	__stringify(IPA_TETHERING_STATS_UPDATE_NETWORK_STATS),
	__stringify(IPA_QUOTA_REACH),
	__stringify(IPA_SSR_BEFORE_SHUTDOWN),
	__stringify(IPA_SSR_AFTER_POWERUP),
	__stringify(ADD_VLAN_IFACE),
	__stringify(DEL_VLAN_IFACE),
	__stringify(ADD_L2TP_VLAN_MAPPING),
	__stringify(DEL_L2TP_VLAN_MAPPING),
	__stringify(IPA_PER_CLIENT_STATS_CONNECT_EVENT),
	__stringify(IPA_PER_CLIENT_STATS_DISCONNECT_EVENT),
	__stringify(ADD_BRIDGE_VLAN_MAPPING),
	__stringify(DEL_BRIDGE_VLAN_MAPPING),
	__stringify(WLAN_FWR_SSR_BEFORE_SHUTDOWN),
	__stringify(IPA_GSB_CONNECT),
	__stringify(IPA_GSB_DISCONNECT),
	__stringify(IPA_COALESCE_ENABLE),
	__stringify(IPA_COALESCE_DISABLE),
	__stringify(IPA_SET_MTU),
	__stringify_1(WIGIG_CLIENT_CONNECT),
	__stringify_1(WIGIG_FST_SWITCH),
	__stringify(IPA_PDN_DEFAULT_MODE_CONFIG),
	__stringify(IPA_PDN_IP_COLLISION_MODE_CONFIG),
	__stringify(IPA_PDN_IP_PASSTHROUGH_MODE_CONFIG),
	__stringify(IPA_MAC_FLT_EVENT),
	__stringify(IPA_SOCKV5_ADD),
	__stringify(IPA_SOCKV5_DEL),
	__stringify(IPA_SW_FLT_EVENT),
	__stringify(IPA_PKT_THRESHOLD_EVENT),
	__stringify(IPA_MOVE_NAT_TABLE),
	__stringify(IPA_EoGRE_UP_EVENT),
	__stringify(IPA_EoGRE_DOWN_EVENT),
	__stringify(IPA_IPPT_SW_FLT_EVENT),
	__stringify(IPA_MACSEC_ADD_EVENT),
	__stringify(IPA_MACSEC_DEL_EVENT),
};

const char *ipa3_hdr_l2_type_name[] = {
	__stringify(IPA_HDR_L2_NONE),
	__stringify(IPA_HDR_L2_ETHERNET_II),
	__stringify(IPA_HDR_L2_802_3),
	__stringify(IPA_HDR_L2_802_1Q),
};

const char *ipa3_hdr_proc_type_name[] = {
	__stringify(IPA_HDR_PROC_NONE),
	__stringify(IPA_HDR_PROC_ETHII_TO_ETHII),
	__stringify(IPA_HDR_PROC_ETHII_TO_802_3),
	__stringify(IPA_HDR_PROC_802_3_TO_ETHII),
	__stringify(IPA_HDR_PROC_802_3_TO_802_3),
	__stringify(IPA_HDR_PROC_L2TP_HEADER_ADD),
	__stringify(IPA_HDR_PROC_L2TP_HEADER_REMOVE),
	__stringify(IPA_HDR_PROC_ETHII_TO_ETHII_EX),
	__stringify(IPA_HDR_PROC_L2TP_UDP_HEADER_ADD),
	__stringify(IPA_HDR_PROC_L2TP_UDP_HEADER_REMOVE),
	__stringify(IPA_HDR_PROC_SET_DSCP),
	__stringify(IPA_HDR_PROC_EoGRE_HEADER_ADD),
	__stringify(IPA_HDR_PROC_EoGRE_HEADER_REMOVE),
};

static struct dentry *dent;
static struct dentry *dent_eth;
static char dbg_buff[IPA_MAX_MSG_LEN + 1];
static char *active_clients_buf;

static s8 ep_reg_idx;
static void *ipa_ipc_low_buff;


static ssize_t ipa3_read_gen_reg(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	int nbytes;
	struct ipahal_reg_shared_mem_size smem_sz;

	memset(&smem_sz, 0, sizeof(smem_sz));

	IPA_ACTIVE_CLIENTS_INC_SIMPLE();

	ipahal_read_reg_fields(IPA_SHARED_MEM_SIZE, &smem_sz);
	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"IPA_VERSION=0x%x\n"
			"IPA_COMP_HW_VERSION=0x%x\n"
			"IPA_ROUTE=0x%x\n"
			"IPA_SHARED_MEM_RESTRICTED=0x%x\n"
			"IPA_SHARED_MEM_SIZE=0x%x\n"
			"IPA_QTIME_TIMESTAMP_CFG=0x%x\n"
			"IPA_TIMERS_PULSE_GRAN_CFG=0x%x\n"
			"IPA_TIMERS_XO_CLK_DIV_CFG=0x%x\n",
			ipahal_read_reg(IPA_VERSION),
			ipahal_read_reg(IPA_COMP_HW_VERSION),
			ipahal_read_reg(IPA_ROUTE),
			smem_sz.shared_mem_baddr,
			smem_sz.shared_mem_sz,
			ipahal_read_reg(IPA_QTIME_TIMESTAMP_CFG),
			ipahal_read_reg(IPA_TIMERS_PULSE_GRAN_CFG),
			ipahal_read_reg(IPA_TIMERS_XO_CLK_DIV_CFG));

	IPA_ACTIVE_CLIENTS_DEC_SIMPLE();

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa3_write_ep_holb(struct file *file,
		const char __user *buf, size_t count, loff_t *ppos)
{
	struct ipa_ep_cfg_holb holb;
	u32 en;
	u32 tmr_val;
	u32 ep_idx;
	unsigned long missing;
	char *sptr, *token;

	if (count >= sizeof(dbg_buff))
		return -EFAULT;

	missing = copy_from_user(dbg_buff, buf, count);
	if (missing)
		return -EFAULT;

	dbg_buff[count] = '\0';

	sptr = dbg_buff;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 0, &ep_idx))
		return -EINVAL;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 0, &en))
		return -EINVAL;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 0, &tmr_val))
		return -EINVAL;

	holb.en = en;
	holb.tmr_val = tmr_val;

	ipa3_cfg_ep_holb(ep_idx, &holb);

	return count;
}

static ssize_t ipa3_write_holb_monitor_client(struct file *file,
		const char __user *buf, size_t count, loff_t *ppos)
{
	struct ipa_uc_holb_client_info holb_client;
	u32 max_stuck_cnt;
	u16 gsi_ch;
	u8 set_client;
	unsigned long missing;
	char *sptr, *token;

	if (count >= sizeof(dbg_buff))
		return -EFAULT;

	missing = copy_from_user(dbg_buff, buf, count);
	if (missing)
		return -EFAULT;

	dbg_buff[count] = '\0';

	sptr = dbg_buff;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou16(token, 0, &gsi_ch))
		return -EINVAL;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 0, &max_stuck_cnt))
		return -EINVAL;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou8(token, 0, &set_client))
		return -EINVAL;

	holb_client.gsi_chan_hdl = gsi_ch;
	holb_client.debugfs_param = set_client;
	holb_client.max_stuck_cnt = max_stuck_cnt;
	holb_client.action_mask = HOLB_MONITOR_MASK;
	holb_client.ee = IPA_EE_AP;


	ipa3_set_holb_client_by_ch(holb_client);

	return count;
}

static ssize_t ipa3_write_holb_monitor_client_add_del(struct file *file,
		const char __user *buf, size_t count, loff_t *ppos)
{
	u32 max_stuck_cnt, action_mask;
	u16 gsi_ch;
	u8 ee, add_client;

	unsigned long missing;
	char *sptr, *token;

	if (count >= sizeof(dbg_buff))
		return -EFAULT;

	missing = copy_from_user(dbg_buff, buf, count);
	if (missing)
		return -EFAULT;

	dbg_buff[count] = '\0';

	sptr = dbg_buff;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou16(token, 0, &gsi_ch))
		return -EINVAL;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 0, &action_mask))
		return -EINVAL;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 0, &max_stuck_cnt))
		return -EINVAL;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou8(token, 0, &ee))
		return -EINVAL;


	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou8(token, 0, &add_client))
		return -EINVAL;

	if (add_client)
		ipa3_uc_client_add_holb_monitor(gsi_ch, action_mask,
			max_stuck_cnt, ee);
	else
		ipa3_uc_client_del_holb_monitor(gsi_ch, ee);

	return count;
}
static ssize_t ipa3_write_ep_reg(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	s8 option;
	int ret;

	ret = kstrtos8_from_user(buf, count, 0, &option);
	if (ret)
		return ret;

	if (option >= ipa3_ctx->ipa_num_pipes) {
		IPAERR("bad pipe specified %u\n", option);
		return count;
	}

	ep_reg_idx = option;

	return count;
}

/**
 * _ipa_read_ep_reg_v3_0() - Reads and prints endpoint configuration registers
 *
 * Returns the number of characters printed
 */
int _ipa_read_ep_reg_v3_0(char *buf, int max_len, int pipe)
{
	return scnprintf(
		dbg_buff, IPA_MAX_MSG_LEN,
		"IPA_ENDP_INIT_NAT_%u=0x%x\n"
		"IPA_ENDP_INIT_HDR_%u=0x%x\n"
		"IPA_ENDP_INIT_HDR_EXT_%u=0x%x\n"
		"IPA_ENDP_INIT_MODE_%u=0x%x\n"
		"IPA_ENDP_INIT_AGGR_%u=0x%x\n"
		"IPA_ENDP_INIT_ROUTE_%u=0x%x\n"
		"IPA_ENDP_INIT_CTRL_%u=0x%x\n"
		"IPA_ENDP_INIT_HOL_EN_%u=0x%x\n"
		"IPA_ENDP_INIT_HOL_TIMER_%u=0x%x\n"
		"IPA_ENDP_INIT_DEAGGR_%u=0x%x\n"
		"IPA_ENDP_INIT_CFG_%u=0x%x\n"
		"IPA_ENDP_INIT_PROD_CFG_%u=0x%x\n",
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_NAT_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_HDR_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_HDR_EXT_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_MODE_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_AGGR_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_ROUTE_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_CTRL_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_HOL_BLOCK_EN_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_HOL_BLOCK_TIMER_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_DEAGGR_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_CFG_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_PROD_CFG_n, pipe));
}

/**
 * _ipa_read_ep_reg_v4_0() - Reads and prints endpoint configuration registers
 *
 * Returns the number of characters printed
 * Removed IPA_ENDP_INIT_ROUTE_n from v3
 */
int _ipa_read_ep_reg_v4_0(char *buf, int max_len, int pipe)
{
	return scnprintf(
		dbg_buff, IPA_MAX_MSG_LEN,
		"IPA_ENDP_INIT_NAT_%u=0x%x\n"
		"IPA_ENDP_INIT_CONN_TRACK_n%u=0x%x\n"
		"IPA_ENDP_INIT_HDR_%u=0x%x\n"
		"IPA_ENDP_INIT_HDR_EXT_%u=0x%x\n"
		"IPA_ENDP_INIT_MODE_%u=0x%x\n"
		"IPA_ENDP_INIT_AGGR_%u=0x%x\n"
		"IPA_ENDP_INIT_CTRL_%u=0x%x\n"
		"IPA_ENDP_INIT_HOL_EN_%u=0x%x\n"
		"IPA_ENDP_INIT_HOL_TIMER_%u=0x%x\n"
		"IPA_ENDP_INIT_DEAGGR_%u=0x%x\n"
		"IPA_ENDP_INIT_CFG_%u=0x%x\n",
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_NAT_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_CONN_TRACK_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_HDR_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_HDR_EXT_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_MODE_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_AGGR_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_CTRL_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_HOL_BLOCK_EN_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_HOL_BLOCK_TIMER_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_DEAGGR_n, pipe),
		pipe, ipahal_read_reg_n(IPA_ENDP_INIT_CFG_n, pipe));
}

static ssize_t ipa3_read_ep_reg(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	int nbytes;
	int i;
	int start_idx;
	int end_idx;
	int size = 0;
	int ret;
	loff_t pos;

	/* negative ep_reg_idx means all registers */
	if (ep_reg_idx < 0) {
		start_idx = 0;
		end_idx = ipa3_ctx->ipa_num_pipes;
	} else {
		start_idx = ep_reg_idx;
		end_idx = start_idx + 1;
	}
	pos = *ppos;
	IPA_ACTIVE_CLIENTS_INC_SIMPLE();
	for (i = start_idx; i < end_idx; i++) {

		nbytes = ipa3_ctx->ctrl->ipa3_read_ep_reg(dbg_buff,
				IPA_MAX_MSG_LEN, i);

		*ppos = pos;
		ret = simple_read_from_buffer(ubuf, count, ppos, dbg_buff,
					      nbytes);
		if (ret < 0) {
			IPA_ACTIVE_CLIENTS_DEC_SIMPLE();
			return ret;
		}

		size += ret;
		ubuf += nbytes;
		count -= nbytes;
	}
	IPA_ACTIVE_CLIENTS_DEC_SIMPLE();

	*ppos = pos + size;
	return size;
}

static ssize_t ipa3_set_clk_index(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	s8 option = 0;
	int ret;
	uint32_t bw_idx = 0;

	ret = kstrtos8_from_user(buf, count, 0, &option);
	if (ret)
		return ret;

	switch (option) {
	case 0:
		bw_idx = 0;
		break;
	case 1:
		bw_idx = 1;
		break;
	case 2:
		bw_idx = 2;
		break;
	case 3:
		bw_idx = 3;
		break;
	case 4:
		bw_idx = 4;
		break;
	default:
		pr_err("Not support this vote (%d)\n", option);
		return -EFAULT;
	}
	pr_info("Make sure some client connected before scaling the BW\n");
	ipa3_ctx->enable_clock_scaling = 1;
	if (ipa3_set_clock_plan_from_pm(bw_idx)) {
		IPAERR("Failed to vote for bus BW (%u)\n", bw_idx);
		return -EFAULT;
	}
	ipa3_ctx->enable_clock_scaling = 0;
	IPAERR("Clock scaling is done sucessful\n");

	return count;
}

static ssize_t ipa3_write_keep_awake(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	s8 option = 0;
	int ret;

	ret = kstrtos8_from_user(buf, count, 0, &option);
	if (ret)
		return ret;

	if (option == 0) {
		if (ipa_pm_remove_dummy_clients()) {
			pr_err("Failed to remove dummy clients\n");
			return -EFAULT;
		}
	} else {
		if (ipa_pm_add_dummy_clients(option - 1)) {
			pr_err("Failed to add dummy clients\n");
			return -EFAULT;
		}
	}

	return count;
}

static ssize_t ipa3_read_keep_awake(struct file *file, char __user *ubuf,
	size_t count, loff_t *ppos)
{
	int nbytes;

	mutex_lock(&ipa3_ctx->ipa3_active_clients.mutex);
	if (atomic_read(&ipa3_ctx->ipa3_active_clients.cnt))
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"IPA APPS power state is ON\n");
	else
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"IPA APPS power state is OFF\n");
	mutex_unlock(&ipa3_ctx->ipa3_active_clients.mutex);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa3_read_mpm_ring_size_dl(struct file *file, char __user *ubuf,
	size_t count, loff_t *ppos)
{
	int nbytes;

	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"IPA_MPM_RING_SIZE_DL = %d\n",
			ipa3_ctx->mpm_ring_size_dl);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa3_read_mpm_ring_size_ul(struct file *file, char __user *ubuf,
	size_t count, loff_t *ppos)
{
	int nbytes;

	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"IPA_MPM_RING_SIZE_UL = %d\n",
			ipa3_ctx->mpm_ring_size_ul);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa3_read_mpm_uc_thresh(struct file *file, char __user *ubuf,
	size_t count, loff_t *ppos)
{
	int nbytes;

	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"IPA_MPM_UC_THRESH = %d\n", ipa3_ctx->mpm_uc_thresh);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa3_read_mpm_teth_aggr_size(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes;

	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"IPA_MPM_TETH_AGGR_SIZE = %d\n",
			ipa3_ctx->mpm_teth_aggr_size);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa3_write_mpm_ring_size_dl(struct file *file,
	const char __user *buf,
	size_t count, loff_t *ppos)
{
	s8 option = 0;
	int ret;

	ret = kstrtos8_from_user(buf, count, 0, &option);
	if (ret)
		return ret;
	/* as option is type s8, max it can take is 127 */
	if ((option > 0) && (option <= IPA_MPM_MAX_RING_LEN))
		ipa3_ctx->mpm_ring_size_dl = option;
	else
		IPAERR("Invalid dl ring size =%d: range is 1 to %d\n",
			option, IPA_MPM_MAX_RING_LEN);
	return count;
}

static ssize_t ipa3_write_mpm_ring_size_ul(struct file *file,
	const char __user *buf,
	size_t count, loff_t *ppos)
{
	s8 option = 0;
	int ret;

	ret = kstrtos8_from_user(buf, count, 0, &option);
	if (ret)
		return ret;
	/* as option is type s8, max it can take is 127 */
	if ((option > 0) && (option <= IPA_MPM_MAX_RING_LEN))
		ipa3_ctx->mpm_ring_size_ul = option;
	else
		IPAERR("Invalid ul ring size =%d: range is 1 to %d\n",
			option, IPA_MPM_MAX_RING_LEN);
	return count;
}

static ssize_t ipa3_write_mpm_uc_thresh(struct file *file,
	const char __user *buf,
	size_t count, loff_t *ppos)
{
	s8 option = 0;
	int ret;

	ret = kstrtos8_from_user(buf, count, 0, &option);
	if (ret)
		return ret;
	/* as option is type s8, max it can take is 127 */
	if ((option > 0) && (option <= IPA_MPM_MAX_UC_THRESH))
		ipa3_ctx->mpm_uc_thresh = option;
	else
		IPAERR("Invalid uc thresh =%d: range is 1 to %d\n",
			option, IPA_MPM_MAX_UC_THRESH);
	return count;
}

static ssize_t ipa3_write_mpm_teth_aggr_size(struct file *file,
	const char __user *buf,
	size_t count, loff_t *ppos)
{
	s8 option = 0;
	int ret;

	ret = kstrtos8_from_user(buf, count, 0, &option);
	if (ret)
		return ret;
	/* as option is type s8, max it can take is 127 */
	if ((option > 0) && (option <= IPA_MAX_TETH_AGGR_BYTE_LIMIT))
		ipa3_ctx->mpm_teth_aggr_size = option;
	else
		IPAERR("Invalid agg byte limit =%d: range is 1 to %d\n",
			option, IPA_MAX_TETH_AGGR_BYTE_LIMIT);
	return count;
}

static ssize_t ipa3_read_holb_events(struct file *file, char __user *ubuf, size_t count,
		loff_t *ppos)
{
	int nbytes = 0;
	int client_idx;
	int event_id;
	bool enable;
	int num_clients = ipa3_ctx->uc_ctx.holb_monitor.num_holb_clients;
	struct ipa_uc_holb_client_info *holb_client;
	uint32_t qtimer_lsb;
	uint32_t qtimer_msb;

	mutex_lock(&ipa3_ctx->lock);
	for (client_idx = 0; client_idx < num_clients; client_idx++) {
		holb_client =
			&(ipa3_ctx->uc_ctx.holb_monitor.client[client_idx]);
		event_id = holb_client->current_idx;
		nbytes += scnprintf(
			dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"========================\n");
		nbytes += scnprintf(
			dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"GSI ch %d cur event_id %d ",
			holb_client->gsi_chan_hdl, holb_client->current_idx);
		nbytes += scnprintf(
			dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"enable cnt %d disable cnt %d\n",
			holb_client->enable_cnt, holb_client->disable_cnt);
		for (event_id = 0; event_id < IPA_HOLB_EVENT_LOG_MAX; event_id++) {
			enable = holb_client->events[event_id].enable;
			qtimer_lsb = holb_client->events[event_id].qTimerLSB;
			qtimer_msb = holb_client->events[event_id].qTimerMSB;
			nbytes += scnprintf(
				dbg_buff + nbytes,
				IPA_MAX_MSG_LEN - nbytes,
				"event id %d: %s QTimer %u %u\n",
				event_id,
				enable ? "Bad Periph event" : "Recovered Periph event",
				qtimer_lsb,
				qtimer_msb);
		}
		nbytes += scnprintf(
			dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"===============\n");
	}

	mutex_unlock(&ipa3_ctx->lock);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa3_read_hdr(struct file *file, char __user *ubuf, size_t count,
		loff_t *ppos)
{
	int nbytes = 0;
	int i = 0;
	struct ipa3_hdr_entry *entry;
	enum hdr_tbl_storage hdr_tbl;
	struct ipa_hdr_offset_entry *offset_entry;
	unsigned int offset_count;

	mutex_lock(&ipa3_ctx->lock);

	for (hdr_tbl = HDR_TBL_LCL; hdr_tbl < HDR_TBLS_TOTAL; hdr_tbl++) {
		if (hdr_tbl == HDR_TBL_LCL)
			pr_err("Table on local memory:\n");
		else
			pr_err("Table on system (ddr) memory:\n");

		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN, "Used offsets: ");
		for (i = 0; i < IPA_HDR_BIN_MAX; i++){
			offset_count = 0;
			list_for_each_entry(offset_entry,
					    &ipa3_ctx->hdr_tbl[hdr_tbl].head_offset_list[i],
					    link)
				offset_count++;
			if (offset_count)
				nbytes += scnprintf(dbg_buff + nbytes,
						    IPA_MAX_MSG_LEN - nbytes,
						    "%u * %u bytes, ",
						    offset_count,
						    ipa3_get_hdr_bin_size(i));
		}
		pr_err("%s", dbg_buff);

		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN, "Free offsets: ");
		for (i = 0; i < IPA_HDR_BIN_MAX; i++){
			offset_count = 0;
			list_for_each_entry(offset_entry,
					    &ipa3_ctx->hdr_tbl[hdr_tbl].head_free_offset_list[i],
					    link)
				offset_count++;
			if (offset_count)
				nbytes += scnprintf(dbg_buff + nbytes,
						    IPA_MAX_MSG_LEN - nbytes,
						    "%u * %u bytes, ",
						    offset_count,
						    ipa3_get_hdr_bin_size(i));
		}
		pr_err("%s", dbg_buff);

		list_for_each_entry(entry, &ipa3_ctx->hdr_tbl[hdr_tbl].head_hdr_entry_list,
				link) {
			if (entry->cookie != IPA_HDR_COOKIE)
				continue;
			nbytes = scnprintf(
				dbg_buff,
				IPA_MAX_MSG_LEN,
				"name:%s len=%d ref=%d partial=%d type=%s ofst=%u ",
				entry->name,
				entry->hdr_len,
				entry->ref_cnt,
				entry->is_partial,
				ipa3_hdr_l2_type_name[entry->type],
				entry->offset_entry->offset >> 2);

			for (i = 0; i < entry->hdr_len; i++) {
				scnprintf(dbg_buff + nbytes + i * 2,
					  IPA_MAX_MSG_LEN - nbytes - i * 2,
					  "%02x", entry->hdr[i]);
			}
			scnprintf(dbg_buff + nbytes + entry->hdr_len * 2,
				  IPA_MAX_MSG_LEN - nbytes - entry->hdr_len * 2,
				  "\n");
			pr_err("%s", dbg_buff);
		}
	}
	mutex_unlock(&ipa3_ctx->lock);

	return 0;
}

static int ipa3_attrib_dump(struct ipa_rule_attrib *attrib,
		enum ipa_ip_type ip)
{
	uint32_t addr[4];
	uint32_t mask[4];
	int i;

	if (attrib->attrib_mask & IPA_FLT_IS_PURE_ACK)
		pr_cont("is_pure_ack ");

	if (attrib->attrib_mask & IPA_FLT_TOS)
		pr_cont("tos:%d ", attrib->u.v4.tos);

	if (attrib->attrib_mask & IPA_FLT_TOS_MASKED) {
		pr_cont("tos_value:%d ", attrib->tos_value);
		pr_cont("tos_mask:%d ", attrib->tos_mask);
	}

	if (attrib->attrib_mask & IPA_FLT_PROTOCOL)
		pr_cont("protocol:%d ", attrib->u.v4.protocol);

	if (attrib->attrib_mask & IPA_FLT_SRC_ADDR) {
		if (ip == IPA_IP_v4) {
			addr[0] = htonl(attrib->u.v4.src_addr);
			mask[0] = htonl(attrib->u.v4.src_addr_mask);
			pr_cont(
				"src_addr:%pI4 src_addr_mask:%pI4 ",
				addr + 0, mask + 0);
		} else if (ip == IPA_IP_v6) {
			for (i = 0; i < 4; i++) {
				addr[i] = htonl(attrib->u.v6.src_addr[i]);
				mask[i] = htonl(attrib->u.v6.src_addr_mask[i]);
			}
			pr_cont(
			   "src_addr:%pI6 src_addr_mask:%pI6 ",
			   addr + 0, mask + 0);
		}
	}
	if (attrib->attrib_mask & IPA_FLT_DST_ADDR) {
		if (ip == IPA_IP_v4) {
			addr[0] = htonl(attrib->u.v4.dst_addr);
			mask[0] = htonl(attrib->u.v4.dst_addr_mask);
			pr_cont(
					   "dst_addr:%pI4 dst_addr_mask:%pI4 ",
					   addr + 0, mask + 0);
		} else if (ip == IPA_IP_v6) {
			for (i = 0; i < 4; i++) {
				addr[i] = htonl(attrib->u.v6.dst_addr[i]);
				mask[i] = htonl(attrib->u.v6.dst_addr_mask[i]);
			}
			pr_cont(
			   "dst_addr:%pI6 dst_addr_mask:%pI6 ",
			   addr + 0, mask + 0);
		}
	}
	if (attrib->attrib_mask & IPA_FLT_SRC_PORT_RANGE) {
		pr_cont("src_port_range:%u %u ",
				   attrib->src_port_lo,
			     attrib->src_port_hi);
	}
	if (attrib->attrib_mask & IPA_FLT_DST_PORT_RANGE) {
		pr_cont("dst_port_range:%u %u ",
			attrib->dst_port_lo,
			attrib->dst_port_hi);
	}
	if (attrib->attrib_mask & IPA_FLT_TYPE)
		pr_cont("type:%d ", attrib->type);

	if (attrib->attrib_mask & IPA_FLT_CODE)
		pr_cont("code:%d ", attrib->code);

	if (attrib->attrib_mask & IPA_FLT_SPI)
		pr_cont("spi:%x ", attrib->spi);

	if (attrib->attrib_mask & IPA_FLT_SRC_PORT)
		pr_cont("src_port:%u ", attrib->src_port);

	if (attrib->attrib_mask & IPA_FLT_DST_PORT)
		pr_cont("dst_port:%u ", attrib->dst_port);

	if (attrib->attrib_mask & IPA_FLT_TC)
		pr_cont("tc:%d ", attrib->u.v6.tc);

	if (attrib->attrib_mask & IPA_FLT_FLOW_LABEL)
		pr_cont("flow_label:%x ", attrib->u.v6.flow_label);

	if (attrib->attrib_mask & IPA_FLT_NEXT_HDR)
		pr_cont("next_hdr:%d ", attrib->u.v6.next_hdr);

	if (attrib->ext_attrib_mask & IPA_FLT_EXT_NEXT_HDR)
		pr_err("next_hdr:%d ", attrib->u.v6.next_hdr);

	if (attrib->attrib_mask & IPA_FLT_META_DATA) {
		pr_cont(
			"metadata:%x metadata_mask:%x ",
			attrib->meta_data, attrib->meta_data_mask);
	}

	if (attrib->attrib_mask & IPA_FLT_FRAGMENT)
		pr_cont("frg ");

	if ((attrib->attrib_mask & IPA_FLT_MAC_SRC_ADDR_ETHER_II) ||
		(attrib->attrib_mask & IPA_FLT_MAC_SRC_ADDR_802_3) ||
		(attrib->attrib_mask & IPA_FLT_MAC_SRC_ADDR_802_1Q)) {
		pr_cont("src_mac_addr:%pM ", attrib->src_mac_addr);
	}

	if ((attrib->attrib_mask & IPA_FLT_MAC_DST_ADDR_ETHER_II) ||
		(attrib->attrib_mask & IPA_FLT_MAC_DST_ADDR_802_3) ||
		(attrib->attrib_mask & IPA_FLT_MAC_DST_ADDR_L2TP) ||
		(attrib->attrib_mask & IPA_FLT_MAC_DST_ADDR_802_1Q) ||
		(attrib->attrib_mask & IPA_FLT_L2TP_UDP_INNER_MAC_DST_ADDR)) {
		pr_cont("dst_mac_addr:%pM ", attrib->dst_mac_addr);
	}

	if (attrib->ext_attrib_mask & IPA_FLT_EXT_MTU)
		pr_err("Payload Length:%d ", attrib->payload_length);

	if (attrib->attrib_mask & IPA_FLT_MAC_ETHER_TYPE ||
		attrib->ext_attrib_mask & IPA_FLT_EXT_L2TP_UDP_INNER_ETHER_TYPE)
		pr_cont("ether_type:%x ", attrib->ether_type);

	if (attrib->attrib_mask & IPA_FLT_VLAN_ID)
		pr_cont("vlan_id:%x ", attrib->vlan_id);

	if (attrib->attrib_mask & IPA_FLT_TCP_SYN)
		pr_cont("tcp syn ");

	if (attrib->attrib_mask & IPA_FLT_TCP_SYN_L2TP ||
		attrib->ext_attrib_mask & IPA_FLT_EXT_L2TP_UDP_TCP_SYN)
		pr_cont("tcp syn l2tp ");

	if (attrib->attrib_mask & IPA_FLT_L2TP_INNER_IP_TYPE)
		pr_cont("l2tp inner ip type: %d ", attrib->type);

	if (attrib->attrib_mask & IPA_FLT_L2TP_INNER_IPV4_DST_ADDR) {
		addr[0] = htonl(attrib->u.v4.dst_addr);
		mask[0] = htonl(attrib->u.v4.dst_addr_mask);
		pr_cont("dst_addr:%pI4 dst_addr_mask:%pI4 ", addr, mask);
	}

	pr_err("\n");
	return 0;
}

static int ipa3_attrib_dump_eq(struct ipa_ipfltri_rule_eq *attrib)
{
	uint8_t addr[16];
	uint8_t mask[16];
	int i;
	int j;

	if (attrib->tos_eq_present) {
		if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_5)
			pr_err("pure_ack ");
		else
			pr_err("tos:%d ", attrib->tos_eq);
	}

	if (attrib->protocol_eq_present)
		pr_err("protocol:%d ", attrib->protocol_eq);

	if (attrib->tc_eq_present)
		pr_err("tc:%d ", attrib->tc_eq);

	if (attrib->num_offset_meq_128 > IPA_IPFLTR_NUM_MEQ_128_EQNS) {
		IPAERR_RL("num_offset_meq_128  Max %d passed value %d\n",
		IPA_IPFLTR_NUM_MEQ_128_EQNS, attrib->num_offset_meq_128);
		return -EPERM;
	}

	for (i = 0; i < attrib->num_offset_meq_128; i++) {
		for (j = 0; j < 16; j++) {
			addr[j] = attrib->offset_meq_128[i].value[j];
			mask[j] = attrib->offset_meq_128[i].mask[j];
		}
		pr_err(
			"(ofst_meq128: ofst:%d mask:%pI6 val:%pI6) ",
			attrib->offset_meq_128[i].offset,
			mask, addr);
	}

	if (attrib->num_offset_meq_32 > IPA_IPFLTR_NUM_MEQ_32_EQNS) {
		IPAERR_RL("num_offset_meq_32  Max %d passed value %d\n",
		IPA_IPFLTR_NUM_MEQ_32_EQNS, attrib->num_offset_meq_32);
		return -EPERM;
	}

	for (i = 0; i < attrib->num_offset_meq_32; i++)
		pr_err(
		   "(ofst_meq32: ofst:%u mask:0x%x val:0x%x) ",
		   attrib->offset_meq_32[i].offset,
		   attrib->offset_meq_32[i].mask,
		   attrib->offset_meq_32[i].value);

	if (attrib->num_ihl_offset_meq_32 > IPA_IPFLTR_NUM_IHL_MEQ_32_EQNS) {
		IPAERR_RL("num_ihl_offset_meq_32  Max %d passed value %d\n",
		IPA_IPFLTR_NUM_IHL_MEQ_32_EQNS, attrib->num_ihl_offset_meq_32);
		return -EPERM;
	}

	for (i = 0; i < attrib->num_ihl_offset_meq_32; i++)
		pr_err(
			"(ihl_ofst_meq32: ofts:%d mask:0x%x val:0x%x) ",
			attrib->ihl_offset_meq_32[i].offset,
			attrib->ihl_offset_meq_32[i].mask,
			attrib->ihl_offset_meq_32[i].value);

	if (attrib->metadata_meq32_present)
		pr_err(
			"(metadata: ofst:%u mask:0x%x val:0x%x) ",
			attrib->metadata_meq32.offset,
			attrib->metadata_meq32.mask,
			attrib->metadata_meq32.value);

	if (attrib->num_ihl_offset_range_16 >
			IPA_IPFLTR_NUM_IHL_RANGE_16_EQNS) {
		IPAERR_RL("num_ihl_offset_range_16  Max %d passed value %d\n",
			IPA_IPFLTR_NUM_IHL_RANGE_16_EQNS,
			attrib->num_ihl_offset_range_16);
		return -EPERM;
	}

	for (i = 0; i < attrib->num_ihl_offset_range_16; i++)
		pr_err(
		   "(ihl_ofst_range16: ofst:%u lo:%u hi:%u) ",
		   attrib->ihl_offset_range_16[i].offset,
		   attrib->ihl_offset_range_16[i].range_low,
		   attrib->ihl_offset_range_16[i].range_high);

	if (attrib->ihl_offset_eq_32_present)
		pr_err(
			"(ihl_ofst_eq32:%d val:0x%x) ",
			attrib->ihl_offset_eq_32.offset,
			attrib->ihl_offset_eq_32.value);

	if (attrib->ihl_offset_eq_16_present)
		pr_err(
			"(ihl_ofst_eq16:%d val:0x%x) ",
			attrib->ihl_offset_eq_16.offset,
			attrib->ihl_offset_eq_16.value);

	if (attrib->fl_eq_present)
		pr_err("flow_label:%d ", attrib->fl_eq);

	if (attrib->ipv4_frag_eq_present)
		pr_err("frag ");

	pr_err("\n");
	return 0;
}

static int ipa3_open_dbg(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t ipa3_read_rt(struct file *file, char __user *ubuf, size_t count,
		loff_t *ppos)
{
	int i = 0;
	struct ipa3_rt_tbl *tbl;
	struct ipa3_rt_entry *entry;
	struct ipa3_rt_tbl_set *set;
	enum ipa_ip_type ip = (enum ipa_ip_type)file->private_data;
	u32 ofst;
	u32 ofst_words;

	set = &ipa3_ctx->rt_tbl_set[ip];

	mutex_lock(&ipa3_ctx->lock);

	pr_err("==== Routing Tables Start ====\n");
	if (ipa3_ctx->rt_tbl_hash_lcl[ip])
		pr_err("Hashable table resides on local memory\n");
	else
		pr_err("Hashable table resides on system (ddr) memory\n");
	if (ipa3_ctx->rt_tbl_nhash_lcl[ip])
		pr_err("Non-Hashable table resides on local memory\n");
	else
		pr_err("Non-Hashable table resides on system (ddr) memory\n");

	list_for_each_entry(tbl, &set->head_rt_tbl_list, link) {
		i = 0;
		list_for_each_entry(entry, &tbl->head_rt_rule_list, link) {
			pr_err("tbl_idx:%d tbl_name:%s tbl_ref:%u ",
				entry->tbl->idx, entry->tbl->name,
				entry->tbl->ref_cnt);
			if (entry->proc_ctx &&
				(!ipa3_check_idr_if_freed(entry->proc_ctx))) {
				ofst = entry->proc_ctx->offset_entry->offset;
				ofst_words =
					(ofst +
					ipa3_ctx->hdr_proc_ctx_tbl.start_offset)
					>> 5;
				pr_err("rule_idx:%d dst:%d ep:%d S:%u ",
					i, entry->rule.dst,
					ipa3_get_ep_mapping(entry->rule.dst),
					!ipa3_ctx->hdr_proc_ctx_tbl_lcl);
				pr_err("proc_ctx[32B]:%u attrib_mask:%08x ",
					ofst_words,
					entry->rule.attrib.attrib_mask);
			} else {
				if (entry->hdr)
					ofst = entry->hdr->offset_entry->offset;
				else
					ofst = 0;
				pr_err("rule_idx:%d dst:%d ep:%d S:%u ",
					i, entry->rule.dst,
					ipa3_get_ep_mapping(entry->rule.dst),
					!(entry->hdr && entry->hdr->is_lcl));
				pr_err("hdr_ofst[words]:%u attrib_mask:%08x ",
					ofst >> 2,
					entry->rule.attrib.attrib_mask);
			}
			pr_err("rule_id:%u max_prio:%u prio:%u ",
				entry->rule_id, entry->rule.max_prio,
				entry->prio);
			pr_err("enable_stats:%u counter_id:%u ",
				entry->rule.enable_stats,
				entry->rule.cnt_idx);
			pr_err("hashable:%u retain_hdr:%u ",
				entry->rule.hashable,
				entry->rule.retain_hdr);
			if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_0)
				pr_err("close_aggr_irq_mod: %u\n",
					entry->rule.close_aggr_irq_mod);
			if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_5)
				pr_err("ttl_update: %u\n", entry->rule.ttl_update);

			ipa3_attrib_dump(&entry->rule.attrib, ip);
			i++;
		}
	}
	pr_err("==== Routing Tables End ====\n");
	mutex_unlock(&ipa3_ctx->lock);

	return 0;
}

static ssize_t ipa3_read_rt_hw(struct file *file, char __user *ubuf,
	size_t count, loff_t *ppos)
{
	enum ipa_ip_type ip = (enum ipa_ip_type)file->private_data;
	int tbls_num;
	int rules_num;
	int tbl;
	int rl;
	int res = 0;
	struct ipahal_rt_rule_entry *rules = NULL;

	switch (ip) {
	case IPA_IP_v4:
		tbls_num = IPA_MEM_PART(v4_rt_num_index);
		break;
	case IPA_IP_v6:
		tbls_num = IPA_MEM_PART(v6_rt_num_index);
		break;
	default:
		IPAERR("ip type error %d\n", ip);
		return -EINVAL;
	}

	IPADBG("Tring to parse %d H/W routing tables - IP=%d\n", tbls_num, ip);

	rules = kzalloc(sizeof(*rules) * IPA_DBG_MAX_RULE_IN_TBL, GFP_KERNEL);
	if (!rules) {
		IPAERR("failed to allocate mem for tbl rules\n");
		return -ENOMEM;
	}

	IPA_ACTIVE_CLIENTS_INC_SIMPLE();
	mutex_lock(&ipa3_ctx->lock);

	for (tbl = 0 ; tbl < tbls_num ; tbl++) {
		pr_err("=== Routing Table %d = Hashable Rules ===\n", tbl);
		rules_num = IPA_DBG_MAX_RULE_IN_TBL;
		res = ipa3_rt_read_tbl_from_hw(tbl, ip, true, rules,
			&rules_num);
		if (res) {
			pr_err("ERROR - Check the logs\n");
			IPAERR("failed reading tbl from hw\n");
			goto bail;
		}
		if (!rules_num)
			pr_err("-->No rules. Empty tbl or modem system table\n");

		for (rl = 0 ; rl < rules_num ; rl++) {
			pr_err("rule_idx:%d dst ep:%d L:%u ",
				rl, rules[rl].dst_pipe_idx, rules[rl].hdr_lcl);

			if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_0)
				pr_err("close_aggr_irq_mod: %u ",
					rules[rl].close_aggr_irq_mod);

			if (rules[rl].hdr_type == IPAHAL_RT_RULE_HDR_PROC_CTX)
				pr_err("proc_ctx:%u attrib_mask:%08x ",
					rules[rl].hdr_ofst,
					rules[rl].eq_attrib.rule_eq_bitmap);
			else
				pr_err("hdr_ofst:%u attrib_mask:%08x ",
					rules[rl].hdr_ofst,
					rules[rl].eq_attrib.rule_eq_bitmap);

			pr_err("rule_id:%u cnt_id:%hhu prio:%u retain_hdr:%u\n",
				rules[rl].id, rules[rl].cnt_idx,
				rules[rl].priority, rules[rl].retain_hdr);
			res = ipa3_attrib_dump_eq(&rules[rl].eq_attrib);
			if (res) {
				IPAERR_RL("failed read attrib eq\n");
				goto bail;
			}
		}

		pr_err("=== Routing Table %d = Non-Hashable Rules ===\n", tbl);
		rules_num = IPA_DBG_MAX_RULE_IN_TBL;
		res = ipa3_rt_read_tbl_from_hw(tbl, ip, false, rules,
			&rules_num);
		if (res) {
			pr_err("ERROR - Check the logs\n");
			IPAERR("failed reading tbl from hw\n");
			goto bail;
		}
		if (!rules_num)
			pr_err("-->No rules. Empty tbl or modem system table\n");

		for (rl = 0 ; rl < rules_num ; rl++) {
			pr_err("rule_idx:%d dst ep:%d L:%u ",
				rl, rules[rl].dst_pipe_idx, rules[rl].hdr_lcl);

			if (rules[rl].hdr_type == IPAHAL_RT_RULE_HDR_PROC_CTX)
				pr_err("proc_ctx:%u attrib_mask:%08x ",
					rules[rl].hdr_ofst,
					rules[rl].eq_attrib.rule_eq_bitmap);
			else
				pr_err("hdr_ofst:%u attrib_mask:%08x ",
					rules[rl].hdr_ofst,
					rules[rl].eq_attrib.rule_eq_bitmap);

			pr_err("rule_id:%u cnt_id:%hhu prio:%u retain_hdr:%u\n",
				rules[rl].id, rules[rl].cnt_idx,
				rules[rl].priority, rules[rl].retain_hdr);
			res = ipa3_attrib_dump_eq(&rules[rl].eq_attrib);
			if (res) {
				IPAERR_RL("failed read attrib eq\n");
				goto bail;
			}
		}
		pr_err("\n");
	}

bail:
	mutex_unlock(&ipa3_ctx->lock);
	IPA_ACTIVE_CLIENTS_DEC_SIMPLE();
	kfree(rules);
	return res;
}

static ssize_t ipa3_read_proc_ctx(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	int nbytes = 0;
	struct ipa3_hdr_proc_ctx_tbl *tbl;
	struct ipa3_hdr_proc_ctx_entry *entry;
	u32 ofst_words;

	tbl = &ipa3_ctx->hdr_proc_ctx_tbl;

	mutex_lock(&ipa3_ctx->lock);

	if (ipa3_ctx->hdr_proc_ctx_tbl_lcl)
		pr_info("Table resides on local memory\n");
	else
		pr_info("Table resides on system(ddr) memory\n");

	list_for_each_entry(entry, &tbl->head_proc_ctx_entry_list, link) {
		ofst_words = (entry->offset_entry->offset +
			ipa3_ctx->hdr_proc_ctx_tbl.start_offset)
			>> 5;
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"id:%u hdr_proc_type:%s proc_ctx[32B]:%u ",
			entry->id,
			ipa3_hdr_proc_type_name[entry->type],
			ofst_words);
		nbytes += scnprintf(dbg_buff + nbytes,
			IPA_MAX_MSG_LEN - nbytes,
			"hdr[words]:%u\n",
			entry->hdr->offset_entry->offset >> 2);
	}
	mutex_unlock(&ipa3_ctx->lock);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa3_read_flt(struct file *file, char __user *ubuf, size_t count,
		loff_t *ppos)
{
	int i;
	int j;
	struct ipa3_flt_tbl *tbl;
	struct ipa3_flt_entry *entry;
	enum ipa_ip_type ip = (enum ipa_ip_type)file->private_data;
	struct ipa3_rt_tbl *rt_tbl;
	u32 rt_tbl_idx;
	u32 bitmap;
	bool eq;
	int res = 0;

	mutex_lock(&ipa3_ctx->lock);

	pr_err("==== Filtering Tables Start ====\n");
	if (ipa3_ctx->flt_tbl_hash_lcl[ip])
		pr_err("Hashable table resides on local memory\n");
	else
		pr_err("Hashable table resides on system (ddr) memory\n");
	if (ipa3_ctx->flt_tbl_nhash_lcl[ip])
		pr_err("Non-Hashable table resides on local memory\n");
	else
		pr_err("Non-Hashable table resides on system (ddr) memory\n");

	for (j = 0; j < ipa3_ctx->ipa_num_pipes; j++) {
		if (!ipa_is_ep_support_flt(j))
			continue;
		tbl = &ipa3_ctx->flt_tbl[j][ip];
		i = 0;
		list_for_each_entry(entry, &tbl->head_flt_rule_list, link) {
			if (entry->cookie != IPA_FLT_COOKIE)
				continue;
			if (entry->rule.eq_attrib_type) {
				rt_tbl_idx = entry->rule.rt_tbl_idx;
				bitmap = entry->rule.eq_attrib.rule_eq_bitmap;
				eq = true;
			} else {
				rt_tbl = ipa3_id_find(entry->rule.rt_tbl_hdl);
				if (rt_tbl == NULL ||
					rt_tbl->cookie != IPA_RT_TBL_COOKIE)
					rt_tbl_idx =  ~0;
				else
					rt_tbl_idx = rt_tbl->idx;
				bitmap = entry->rule.attrib.attrib_mask;
				eq = false;
			}
			pr_err("ep_idx:%d rule_idx:%d act:%d rt_tbl_idx:%d ",
				j, i, entry->rule.action, rt_tbl_idx);
			pr_err("attrib_mask:%08x retain_hdr:%d eq:%d ",
				bitmap, entry->rule.retain_hdr, eq);
			pr_err("hashable:%u rule_id:%u max_prio:%u prio:%u ",
				entry->rule.hashable, entry->rule_id,
				entry->rule.max_prio, entry->prio);
			if (entry->rule.hashable)
				pr_err("hash in_sys_preffer:%d, force: %d ",
					tbl->in_sys[IPA_RULE_HASHABLE],
					tbl->force_sys[IPA_RULE_HASHABLE]);
			else
				pr_err("non-hash in_sys_preffer:%d, force: %d ",
					tbl->in_sys[IPA_RULE_NON_HASHABLE],
					tbl->force_sys[IPA_RULE_NON_HASHABLE]);
			pr_err("enable_stats:%u counter_id:%u\n",
				entry->rule.enable_stats,
				entry->rule.cnt_idx);
			if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_0)
				pr_err("pdn index %d, set metadata %d ",
					entry->rule.pdn_idx,
					entry->rule.set_metadata);
			if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_0)
				pr_err("close_aggr_irq_mod %u ",
					entry->rule.close_aggr_irq_mod);
			if (eq) {
				res = ipa3_attrib_dump_eq(
						&entry->rule.eq_attrib);
				if (res) {
					IPAERR_RL("failed read attrib eq\n");
					goto bail;
				}
			} else
				ipa3_attrib_dump(
					&entry->rule.attrib, ip);
			i++;
			if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_5)
				pr_err("ttl_update %u ", entry->rule.ttl_update);
		}
	}
bail:
	pr_err("==== Filtering Tables End ====\n");
	mutex_unlock(&ipa3_ctx->lock);

	return res;
}

static ssize_t ipa3_read_flt_hw(struct file *file, char __user *ubuf,
	size_t count, loff_t *ppos)
{
	int pipe;
	int rl;
	int rules_num;
	struct ipahal_flt_rule_entry *rules;
	enum ipa_ip_type ip = (enum ipa_ip_type)file->private_data;
	u32 rt_tbl_idx;
	u32 bitmap;
	int res = 0;

	IPADBG("Tring to parse %d H/W filtering tables - IP=%d\n",
		ipa3_ctx->ep_flt_num, ip);

	rules = kzalloc(sizeof(*rules) * IPA_DBG_MAX_RULE_IN_TBL, GFP_KERNEL);
	if (!rules)
		return -ENOMEM;

	IPA_ACTIVE_CLIENTS_INC_SIMPLE();
	mutex_lock(&ipa3_ctx->lock);

	if (ipa3_ctx->flt_tbl_hash_lcl[ip])
		pr_err("Hashable table resides on local memory\n");
	else
		pr_err("Hashable table resides on system (ddr) memory\n");
	if (ipa3_ctx->flt_tbl_nhash_lcl[ip])
		pr_err("Non-Hashable table resides on local memory\n");
	else
		pr_err("Non-Hashable table resides on system (ddr) memory\n");

	for (pipe = 0; pipe < ipa3_ctx->ipa_num_pipes; pipe++) {
		if (!ipa_is_ep_support_flt(pipe))
			continue;
		pr_err("=== Filtering Table ep:%d = Hashable Rules ===\n",
			pipe);
		rules_num = IPA_DBG_MAX_RULE_IN_TBL;
		res = ipa3_flt_read_tbl_from_hw(pipe, ip, true, rules,
			&rules_num);
		if (res) {
			pr_err("ERROR - Check the logs\n");
			IPAERR("failed reading tbl from hw\n");
			goto bail;
		}
		if (!rules_num)
			pr_err("-->No rules. Empty tbl or modem sys table\n");

		for (rl = 0; rl < rules_num; rl++) {
			rt_tbl_idx = rules[rl].rule.rt_tbl_idx;
			bitmap = rules[rl].rule.eq_attrib.rule_eq_bitmap;
			pr_err("ep_idx:%d rule_idx:%d act:%d rt_tbl_idx:%d ",
				pipe, rl, rules[rl].rule.action, rt_tbl_idx);
			pr_err("attrib_mask:%08x retain_hdr:%d ",
				bitmap, rules[rl].rule.retain_hdr);
			pr_err("rule_id:%u cnt_id:%hhu prio:%u\n",
				rules[rl].id, rules[rl].cnt_idx,
				rules[rl].priority);
			if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_0)
				pr_err("close_aggr_irq_mod %u\n",
					rules[rl].rule.close_aggr_irq_mod);
			if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_0)
				pr_err("pdn: %u, set_metadata: %u ",
					rules[rl].rule.pdn_idx,
					rules[rl].rule.set_metadata);
			res = ipa3_attrib_dump_eq(&rules[rl].rule.eq_attrib);
			if (res) {
				IPAERR_RL("failed read attrib eq\n");
				goto bail;
			}
		}

		pr_err("=== Filtering Table ep:%d = Non-Hashable Rules ===\n",
			pipe);
		rules_num = IPA_DBG_MAX_RULE_IN_TBL;
		res = ipa3_flt_read_tbl_from_hw(pipe, ip, false, rules,
			&rules_num);
		if (res) {
			IPAERR("failed reading tbl from hw\n");
			goto bail;
		}
		if (!rules_num)
			pr_err("-->No rules. Empty tbl or modem sys table\n");
		for (rl = 0; rl < rules_num; rl++) {
			rt_tbl_idx = rules[rl].rule.rt_tbl_idx;
			bitmap = rules[rl].rule.eq_attrib.rule_eq_bitmap;
			pr_err("ep_idx:%d rule_idx:%d act:%d rt_tbl_idx:%d ",
				pipe, rl, rules[rl].rule.action, rt_tbl_idx);
			pr_err("attrib_mask:%08x retain_hdr:%d ",
				bitmap, rules[rl].rule.retain_hdr);
			pr_err("rule_id:%u cnt_id:%hhu prio:%u\n",
				rules[rl].id, rules[rl].cnt_idx,
				rules[rl].priority);
			if (ipa3_ctx->ipa_hw_type >= IPA_HW_v5_0)
				pr_err("close_aggr_irq_mod %u\n",
					rules[rl].rule.close_aggr_irq_mod);
			if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_0)
				pr_err("pdn: %u, set_metadata: %u ",
					rules[rl].rule.pdn_idx,
					rules[rl].rule.set_metadata);
			res = ipa3_attrib_dump_eq(&rules[rl].rule.eq_attrib);
			if (res) {
				IPAERR_RL("failed read attrib eq\n");
				goto bail;
			}
		}
		pr_err("\n");
	}

bail:
	mutex_unlock(&ipa3_ctx->lock);
	kfree(rules);
	IPA_ACTIVE_CLIENTS_DEC_SIMPLE();
	return res;
}

static ssize_t ipa3_read_stats(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	int nbytes;
	int i;
	int cnt = 0;
	uint connect = 0;

	for (i = 0; i < ipa3_ctx->ipa_num_pipes; i++)
		connect |= (ipa3_ctx->ep[i].valid << i);

	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
		"sw_tx=%u\n"
		"hw_tx=%u\n"
		"tx_non_linear=%u\n"
		"tx_compl=%u\n"
		"wan_rx=%u\n"
		"stat_compl=%u\n"
		"lan_aggr_close=%u\n"
		"wan_aggr_close=%u\n"
		"act_clnt=%u\n"
		"con_clnt_bmap=0x%x\n"
		"wan_rx_empty=%u\n"
		"wan_rx_empty_coal=%u\n"
		"wan_repl_rx_empty=%u\n"
		"rmnet_ll_rx_empty=%u\n"
		"rmnet_ll_repl_rx_empty=%u\n"
		"lan_rx_empty=%u\n"
		"lan_repl_rx_empty=%u\n"
		"flow_enable=%u\n"
		"flow_disable=%u\n"
		"rx_page_drop_cnt=%u\n"
		"lower_order=%u\n"
		"rmnet_notifier_enabled=%u\n"
		"num_buff_above_thresh_for_def_pipe_notified=%u\n"
		"num_buff_below_thresh_for_def_pipe_notified=%u\n"
		"num_buff_above_thresh_for_coal_pipe_notified=%u\n"
		"num_buff_below_thresh_for_coal_pipe_notified=%u\n"
		"num_buff_above_thresh_for_ll_pipe_notified=%u\n"
		"num_buff_below_thresh_for_ll_pipe_notified=%u\n"
		"num_free_page_task_scheduled=%u\n"
		"pipe_setup_fail_cnt=%u\n"
		"ttl_count=%u\n",
		ipa3_ctx->stats.tx_sw_pkts,
		ipa3_ctx->stats.tx_hw_pkts,
		ipa3_ctx->stats.tx_non_linear,
		ipa3_ctx->stats.tx_pkts_compl,
		ipa3_ctx->stats.rx_pkts,
		ipa3_ctx->stats.stat_compl,
		ipa3_ctx->stats.aggr_close,
		ipa3_ctx->stats.wan_aggr_close,
		atomic_read(&ipa3_ctx->ipa3_active_clients.cnt),
		connect,
		ipa3_ctx->stats.wan_rx_empty,
		ipa3_ctx->stats.wan_rx_empty_coal,
		ipa3_ctx->stats.wan_repl_rx_empty,
		ipa3_ctx->stats.rmnet_ll_rx_empty,
		ipa3_ctx->stats.rmnet_ll_repl_rx_empty,
		ipa3_ctx->stats.lan_rx_empty,
		ipa3_ctx->stats.lan_repl_rx_empty,
		ipa3_ctx->stats.flow_enable,
		ipa3_ctx->stats.flow_disable,
		ipa3_ctx->stats.rx_page_drop_cnt,
		ipa3_ctx->stats.lower_order,
		ipa3_ctx->ipa_rmnet_notifier_enabled,
		atomic_read(&ipa3_ctx->stats.num_buff_above_thresh_for_def_pipe_notified),
		atomic_read(&ipa3_ctx->stats.num_buff_below_thresh_for_def_pipe_notified),
		atomic_read(&ipa3_ctx->stats.num_buff_above_thresh_for_coal_pipe_notified),
		atomic_read(&ipa3_ctx->stats.num_buff_below_thresh_for_coal_pipe_notified),
		atomic_read(&ipa3_ctx->stats.num_buff_above_thresh_for_ll_pipe_notified),
		atomic_read(&ipa3_ctx->stats.num_buff_below_thresh_for_ll_pipe_notified),
		atomic_read(&ipa3_ctx->stats.num_free_page_task_scheduled),
		ipa3_ctx->stats.pipe_setup_fail_cnt,
		ipa3_ctx->stats.ttl_cnt
		);
	cnt += nbytes;

	for (i = 0; i < IPAHAL_PKT_STATUS_EXCEPTION_MAX; i++) {
		nbytes = scnprintf(dbg_buff + cnt,
			IPA_MAX_MSG_LEN - cnt,
			"lan_rx_excp[%u:%20s]=%u\n", i,
			ipahal_pkt_status_exception_str(i),
			ipa3_ctx->stats.rx_excp_pkts[i]);
		cnt += nbytes;
	}

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_odlstats(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	int nbytes;
	int cnt = 0;

	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"ODL received pkt =%u\n"
			"ODL processed pkt to DIAG=%u\n"
			"ODL dropped pkt =%u\n"
			"ODL packet in queue  =%u\n",
			ipa3_odl_ctx->stats.odl_rx_pkt,
			ipa3_odl_ctx->stats.odl_tx_diag_pkt,
			ipa3_odl_ctx->stats.odl_drop_pkt,
			atomic_read(&ipa3_odl_ctx->stats.numer_in_queue));

	cnt += nbytes;

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}


static ssize_t ipa3_read_page_recycle_stats(struct file *file,
		char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes;
	int cnt = 0, i = 0, k = 0;

	nbytes = scnprintf(
		dbg_buff, IPA_MAX_MSG_LEN,
		"COAL   : Total number of packets replenished =%llu\n"
		"COAL   : Number of page recycled packets  =%llu\n"
		"COAL   : Number of tmp alloc packets  =%llu\n"
		"COAL   : Number of times tasklet scheduled  =%llu\n"

		"DEF    : Total number of packets replenished =%llu\n"
		"DEF    : Number of page recycled packets =%llu\n"
		"DEF    : Number of tmp alloc packets  =%llu\n"
		"DEF    : Number of times tasklet scheduled  =%llu\n"

		"COMMON : Number of page recycled in tasklet  =%llu\n"
		"COMMON : Number of times free pages not found in tasklet =%llu\n",

		ipa3_ctx->stats.page_recycle_stats[0].total_replenished,
		ipa3_ctx->stats.page_recycle_stats[0].page_recycled,
		ipa3_ctx->stats.page_recycle_stats[0].tmp_alloc,
		ipa3_ctx->stats.num_sort_tasklet_sched[0],

		ipa3_ctx->stats.page_recycle_stats[1].total_replenished,
		ipa3_ctx->stats.page_recycle_stats[1].page_recycled,
		ipa3_ctx->stats.page_recycle_stats[1].tmp_alloc,
		ipa3_ctx->stats.num_sort_tasklet_sched[1],

		ipa3_ctx->stats.page_recycle_cnt_in_tasklet,
		ipa3_ctx->stats.num_of_times_wq_reschd);

	cnt += nbytes;

	for (k = 0; k < 2; k++) {
		for (i = 0; i < ipa3_ctx->page_poll_threshold; i++) {
			nbytes = scnprintf(
				dbg_buff + cnt, IPA_MAX_MSG_LEN,
				"COMMON  : Page replenish efficiency[%d][%d]  =%llu\n",
				k, i, ipa3_ctx->stats.page_recycle_cnt[k][i]);
			cnt += nbytes;
		}
	}

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_lan_coal_stats(
	struct file *file,
	char __user *ubuf,
	size_t       count,
	loff_t      *ppos)
{
	int nbytes=0, cnt=0;
	u32 i;
	char buf[1024];

	*buf = '\0';

	for ( i = 0;
		  i < sizeof(ipa3_ctx->stats.coal.coal_veid) /
			  sizeof(ipa3_ctx->stats.coal.coal_veid[0]);
		  i++ ) {

		nbytes += scnprintf(
			buf         + nbytes,
			sizeof(buf) - nbytes,
			"(%u/%llu) ",
			i,
			ipa3_ctx->stats.coal.coal_veid[i]);
	}

	nbytes = scnprintf(
		dbg_buff, IPA_MAX_MSG_LEN,
		"LAN COAL rx            = %llu\n"
		"LAN COAL pkts          = %llu\n"
		"LAN COAL left as is    = %llu\n"
		"LAN COAL reconstructed = %llu\n"
		"LAN COAL hdr qmap err  = %llu\n"
		"LAN COAL hdr nlo err   = %llu\n"
		"LAN COAL hdr pkt err   = %llu\n"
		"LAN COAL csum err      = %llu\n"

		"LAN COAL ip invalid    = %llu\n"
		"LAN COAL trans invalid = %llu\n"
		"LAN COAL tcp           = %llu\n"
		"LAN COAL tcp bytes     = %llu\n"
		"LAN COAL udp           = %llu\n"
		"LAN COAL udp bytes     = %llu\n"
		"LAN COAL (veid/cnt)...(veid/cnt) = %s\n",

		ipa3_ctx->stats.coal.coal_rx,
		ipa3_ctx->stats.coal.coal_pkts,
		ipa3_ctx->stats.coal.coal_left_as_is,
		ipa3_ctx->stats.coal.coal_reconstructed,
		ipa3_ctx->stats.coal.coal_hdr_qmap_err,
		ipa3_ctx->stats.coal.coal_hdr_nlo_err,
		ipa3_ctx->stats.coal.coal_hdr_pkt_err,
		ipa3_ctx->stats.coal.coal_csum_err,
		ipa3_ctx->stats.coal.coal_ip_invalid,
		ipa3_ctx->stats.coal.coal_trans_invalid,
		ipa3_ctx->stats.coal.coal_tcp,
		ipa3_ctx->stats.coal.coal_tcp_bytes,
		ipa3_ctx->stats.coal.coal_udp,
		ipa3_ctx->stats.coal.coal_udp_bytes,
		buf);

	cnt += nbytes;

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_cache_recycle_stats(
	struct file *file,
	char __user *ubuf,
	size_t       count,
	loff_t      *ppos)
{
	int nbytes;
	int cnt = 0;

	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"COAL  (cache) : Total number of pkts replenished =%llu\n"
			"COAL  (cache) : Number of pkts alloced  =%llu\n"
			"COAL  (cache) : Number of pkts not alloced  =%llu\n"

			"DEF   (cache) : Total number of pkts replenished =%llu\n"
			"DEF   (cache) : Number of pkts alloced  =%llu\n"
			"DEF   (cache) : Number of pkts not alloced  =%llu\n"

			"OTHER (cache) : Total number of packets replenished =%llu\n"
			"OTHER (cache) : Number of pkts alloced  =%llu\n"
			"OTHER (cache) : Number of pkts not alloced  =%llu\n",

			ipa3_ctx->stats.cache_recycle_stats[0].tot_pkt_replenished,
			ipa3_ctx->stats.cache_recycle_stats[0].pkt_allocd,
			ipa3_ctx->stats.cache_recycle_stats[0].pkt_found,

			ipa3_ctx->stats.cache_recycle_stats[1].tot_pkt_replenished,
			ipa3_ctx->stats.cache_recycle_stats[1].pkt_allocd,
			ipa3_ctx->stats.cache_recycle_stats[1].pkt_found,

			ipa3_ctx->stats.cache_recycle_stats[2].tot_pkt_replenished,
			ipa3_ctx->stats.cache_recycle_stats[2].pkt_allocd,
			ipa3_ctx->stats.cache_recycle_stats[2].pkt_found);

	cnt += nbytes;

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_wstats(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{

#define HEAD_FRMT_STR "%25s\n"
#define FRMT_STR "%25s %10u\n"
#define FRMT_STR1 "%25s %10u\n\n"

	int cnt = 0;
	int nbytes;
	int ipa_ep_idx;
	enum ipa_client_type client = IPA_CLIENT_WLAN1_PROD;
	struct ipa3_ep_context *ep;

	do {
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			HEAD_FRMT_STR, "Client IPA_CLIENT_WLAN1_PROD Stats:");
		cnt += nbytes;

		ipa_ep_idx = ipa3_get_ep_mapping(client);
		if (ipa_ep_idx == -1) {
			nbytes = scnprintf(dbg_buff + cnt,
				IPA_MAX_MSG_LEN - cnt, HEAD_FRMT_STR, "Not up");
			cnt += nbytes;
			break;
		}

		ep = &ipa3_ctx->ep[ipa_ep_idx];
		if (ep->valid != 1) {
			nbytes = scnprintf(dbg_buff + cnt,
				IPA_MAX_MSG_LEN - cnt, HEAD_FRMT_STR, "Not up");
			cnt += nbytes;
			break;
		}

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR, "Avail Fifo Desc:",
			atomic_read(&ep->avail_fifo_desc));
		cnt += nbytes;

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR, "Rx Pkts Rcvd:", ep->wstats.rx_pkts_rcvd);
		cnt += nbytes;

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR, "Rx Pkts Status Rcvd:",
			ep->wstats.rx_pkts_status_rcvd);
		cnt += nbytes;

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR, "Rx DH Rcvd:", ep->wstats.rx_hd_rcvd);
		cnt += nbytes;

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR, "Rx DH Processed:",
			ep->wstats.rx_hd_processed);
		cnt += nbytes;

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR, "Rx DH Sent Back:", ep->wstats.rx_hd_reply);
		cnt += nbytes;

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR, "Rx Pkt Leak:", ep->wstats.rx_pkt_leak);
		cnt += nbytes;

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR1, "Rx DP Fail:", ep->wstats.rx_dp_fail);
		cnt += nbytes;

	} while (0);

	client = IPA_CLIENT_WLAN1_CONS;
	nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt, HEAD_FRMT_STR,
		"Client IPA_CLIENT_WLAN1_CONS Stats:");
	cnt += nbytes;
	while (1) {
		ipa_ep_idx = ipa3_get_ep_mapping(client);
		if (ipa_ep_idx == -1) {
			nbytes = scnprintf(dbg_buff + cnt,
				IPA_MAX_MSG_LEN - cnt, HEAD_FRMT_STR, "Not up");
			cnt += nbytes;
			goto nxt_clnt_cons;
		}

		ep = &ipa3_ctx->ep[ipa_ep_idx];
		if (ep->valid != 1) {
			nbytes = scnprintf(dbg_buff + cnt,
				IPA_MAX_MSG_LEN - cnt, HEAD_FRMT_STR, "Not up");
			cnt += nbytes;
			goto nxt_clnt_cons;
		}

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR, "Tx Pkts Received:", ep->wstats.tx_pkts_rcvd);
		cnt += nbytes;

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR, "Tx Pkts Sent:", ep->wstats.tx_pkts_sent);
		cnt += nbytes;

		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			FRMT_STR1, "Tx Pkts Dropped:",
			ep->wstats.tx_pkts_dropped);
		cnt += nbytes;
		if (ep->sys) {
			nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
				FRMT_STR1, "sys len:",
				ep->sys->len);
			cnt += nbytes;
			nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
				FRMT_STR1, "rx_pool_sz:",
				ep->sys->rx_pool_sz);
			cnt += nbytes;
		}

nxt_clnt_cons:
			switch (client) {
			case IPA_CLIENT_WLAN1_CONS:
				client = IPA_CLIENT_WLAN2_CONS;
				nbytes = scnprintf(dbg_buff + cnt,
					IPA_MAX_MSG_LEN - cnt, HEAD_FRMT_STR,
					"Client IPA_CLIENT_WLAN2_CONS Stats:");
				cnt += nbytes;
				continue;
			case IPA_CLIENT_WLAN2_CONS:
				client = IPA_CLIENT_WLAN2_CONS1;
				nbytes = scnprintf(dbg_buff + cnt,
					IPA_MAX_MSG_LEN - cnt, HEAD_FRMT_STR,
					"Client IPA_CLIENT_WLAN2_CONS1 Stats:");
				cnt += nbytes;
				continue;
			case IPA_CLIENT_WLAN2_CONS1:
				client = IPA_CLIENT_WLAN3_CONS;
				nbytes = scnprintf(dbg_buff + cnt,
					IPA_MAX_MSG_LEN - cnt, HEAD_FRMT_STR,
					"Client IPA_CLIENT_WLAN3_CONS Stats:");
				cnt += nbytes;
				continue;
			case IPA_CLIENT_WLAN3_CONS:
				client = IPA_CLIENT_WLAN4_CONS;
				nbytes = scnprintf(dbg_buff + cnt,
					IPA_MAX_MSG_LEN - cnt, HEAD_FRMT_STR,
					"Client IPA_CLIENT_WLAN4_CONS Stats:");
				cnt += nbytes;
				continue;
			case IPA_CLIENT_WLAN4_CONS:
			default:
				break;
			}
		break;
	}

	nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
		"\n"HEAD_FRMT_STR, "All Wlan Consumer pipes stats:");
	cnt += nbytes;

	nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt, FRMT_STR,
		"Tx Comm Buff Allocated:",
		ipa3_ctx->wc_memb.wlan_comm_total_cnt);
	cnt += nbytes;

	nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt, FRMT_STR,
		"Tx Comm Buff Avail:", ipa3_ctx->wc_memb.wlan_comm_free_cnt);
	cnt += nbytes;

	nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt, FRMT_STR1,
		"Total Tx Pkts Freed:", ipa3_ctx->wc_memb.total_tx_pkts_freed);
	cnt += nbytes;

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_ntn(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
#define TX_STATS(x, y) \
	stats.tx_ch_stats[x].y
#define RX_STATS(x, y) \
	stats.rx_ch_stats[x].y

	struct Ipa3HwStatsNTNInfoData_t stats;
	int nbytes;
	int cnt = 0, i = 0;

	if (!ipa3_get_ntn_stats(&stats)) {
		for (i = 0; i < IPA_UC_MAX_NTN_TX_CHANNELS; i++) {
			nbytes = scnprintf(dbg_buff + cnt,
				IPA_MAX_MSG_LEN - cnt,
				"TX%d num_pkts_psr=%u\n"
				"TX%d ringFull=%u\n"
				"TX%d ringEmpty=%u\n"
				"TX%d ringUsageHigh=%u\n"
				"TX%d ringUsageLow=%u\n"
				"TX%d RingUtilCount=%u\n"
				"TX%d bamFifoFull=%u\n"
				"TX%d bamFifoEmpty=%u\n"
				"TX%d bamFifoUsageHigh=%u\n"
				"TX%d bamFifoUsageLow=%u\n"
				"TX%d bamUtilCount=%u\n"
				"TX%d num_db=%u\n"
				"TX%d num_qmb_int_handled=%u\n"
				"TX%d ipa_pipe_number=%u\n",
				i, TX_STATS(i, num_pkts_processed),
				i, TX_STATS(i, ring_stats.ringFull),
				i, TX_STATS(i, ring_stats.ringEmpty),
				i, TX_STATS(i, ring_stats.ringUsageHigh),
				i, TX_STATS(i, ring_stats.ringUsageLow),
				i, TX_STATS(i, ring_stats.RingUtilCount),
				i, TX_STATS(i, gsi_stats.bamFifoFull),
				i, TX_STATS(i, gsi_stats.bamFifoEmpty),
				i, TX_STATS(i, gsi_stats.bamFifoUsageHigh),
				i, TX_STATS(i, gsi_stats.bamFifoUsageLow),
				i, TX_STATS(i, gsi_stats.bamUtilCount),
				i, TX_STATS(i, num_db),
				i, TX_STATS(i, num_qmb_int_handled),
				i, TX_STATS(i, ipa_pipe_number));
			cnt += nbytes;
		}

		for (i = 0; i < IPA_UC_MAX_NTN_RX_CHANNELS; i++) {
			nbytes = scnprintf(dbg_buff + cnt,
				IPA_MAX_MSG_LEN - cnt,
				"RX%d num_pkts_psr=%u\n"
				"RX%d ringFull=%u\n"
				"RX%d ringEmpty=%u\n"
				"RX%d ringUsageHigh=%u\n"
				"RX%d ringUsageLow=%u\n"
				"RX%d RingUtilCount=%u\n"
				"RX%d bamFifoFull=%u\n"
				"RX%d bamFifoEmpty=%u\n"
				"RX%d bamFifoUsageHigh=%u\n"
				"RX%d bamFifoUsageLow=%u\n"
				"RX%d bamUtilCount=%u\n"
				"RX%d num_db=%u\n"
				"RX%d num_qmb_int_handled=%u\n"
				"RX%d ipa_pipe_number=%u\n",
				i, RX_STATS(i, num_pkts_processed),
				i, RX_STATS(i, ring_stats.ringFull),
				i, RX_STATS(i, ring_stats.ringEmpty),
				i, RX_STATS(i, ring_stats.ringUsageHigh),
				i, RX_STATS(i, ring_stats.ringUsageLow),
				i, RX_STATS(i, ring_stats.RingUtilCount),
				i, RX_STATS(i, gsi_stats.bamFifoFull),
				i, RX_STATS(i, gsi_stats.bamFifoEmpty),
				i, RX_STATS(i, gsi_stats.bamFifoUsageHigh),
				i, RX_STATS(i, gsi_stats.bamFifoUsageLow),
				i, RX_STATS(i, gsi_stats.bamUtilCount),
				i, RX_STATS(i, num_db),
				i, RX_STATS(i, num_qmb_int_handled),
				i, RX_STATS(i, ipa_pipe_number));
			cnt += nbytes;
		}
	} else {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"Fail to read NTN stats\n");
		cnt += nbytes;
	}

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_wdi(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct IpaHwStatsWDIInfoData_t stats;
	int nbytes;
	int cnt = 0;
	struct IpaHwStatsWDITxInfoData_t *tx_ch_ptr;

	if (!ipa3_get_wdi_stats(&stats)) {
		tx_ch_ptr = &stats.tx_ch_stats;
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"TX num_pkts_processed=%u\n"
			"TX copy_engine_doorbell_value=%u\n"
			"TX num_db_fired=%u\n"
			"TX ringFull=%u\n"
			"TX ringEmpty=%u\n"
			"TX ringUsageHigh=%u\n"
			"TX ringUsageLow=%u\n"
			"TX RingUtilCount=%u\n"
			"TX bamFifoFull=%u\n"
			"TX bamFifoEmpty=%u\n"
			"TX bamFifoUsageHigh=%u\n"
			"TX bamFifoUsageLow=%u\n"
			"TX bamUtilCount=%u\n"
			"TX num_db=%u\n"
			"TX num_unexpected_db=%u\n"
			"TX num_bam_int_handled=%u\n"
			"TX num_bam_int_in_non_running_state=%u\n"
			"TX num_qmb_int_handled=%u\n"
			"TX num_bam_int_handled_while_wait_for_bam=%u\n",
			tx_ch_ptr->num_pkts_processed,
			tx_ch_ptr->copy_engine_doorbell_value,
			tx_ch_ptr->num_db_fired,
			tx_ch_ptr->tx_comp_ring_stats.ringFull,
			tx_ch_ptr->tx_comp_ring_stats.ringEmpty,
			tx_ch_ptr->tx_comp_ring_stats.ringUsageHigh,
			tx_ch_ptr->tx_comp_ring_stats.ringUsageLow,
			tx_ch_ptr->tx_comp_ring_stats.RingUtilCount,
			tx_ch_ptr->bam_stats.bamFifoFull,
			tx_ch_ptr->bam_stats.bamFifoEmpty,
			tx_ch_ptr->bam_stats.bamFifoUsageHigh,
			tx_ch_ptr->bam_stats.bamFifoUsageLow,
			tx_ch_ptr->bam_stats.bamUtilCount,
			tx_ch_ptr->num_db,
			tx_ch_ptr->num_unexpected_db,
			tx_ch_ptr->num_bam_int_handled,
			tx_ch_ptr->num_bam_int_in_non_running_state,
			tx_ch_ptr->num_qmb_int_handled,
			tx_ch_ptr->num_bam_int_handled_while_wait_for_bam);
		cnt += nbytes;
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			"RX max_outstanding_pkts=%u\n"
			"RX num_pkts_processed=%u\n"
			"RX rx_ring_rp_value=%u\n"
			"RX ringFull=%u\n"
			"RX ringEmpty=%u\n"
			"RX ringUsageHigh=%u\n"
			"RX ringUsageLow=%u\n"
			"RX RingUtilCount=%u\n"
			"RX bamFifoFull=%u\n"
			"RX bamFifoEmpty=%u\n"
			"RX bamFifoUsageHigh=%u\n"
			"RX bamFifoUsageLow=%u\n"
			"RX bamUtilCount=%u\n"
			"RX num_bam_int_handled=%u\n"
			"RX num_db=%u\n"
			"RX num_unexpected_db=%u\n"
			"RX num_pkts_in_dis_uninit_state=%u\n"
			"RX num_ic_inj_vdev_change=%u\n"
			"RX num_ic_inj_fw_desc_change=%u\n"
			"RX num_qmb_int_handled=%u\n"
			"RX reserved1=%u\n"
			"RX reserved2=%u\n",
			stats.rx_ch_stats.max_outstanding_pkts,
			stats.rx_ch_stats.num_pkts_processed,
			stats.rx_ch_stats.rx_ring_rp_value,
			stats.rx_ch_stats.rx_ind_ring_stats.ringFull,
			stats.rx_ch_stats.rx_ind_ring_stats.ringEmpty,
			stats.rx_ch_stats.rx_ind_ring_stats.ringUsageHigh,
			stats.rx_ch_stats.rx_ind_ring_stats.ringUsageLow,
			stats.rx_ch_stats.rx_ind_ring_stats.RingUtilCount,
			stats.rx_ch_stats.bam_stats.bamFifoFull,
			stats.rx_ch_stats.bam_stats.bamFifoEmpty,
			stats.rx_ch_stats.bam_stats.bamFifoUsageHigh,
			stats.rx_ch_stats.bam_stats.bamFifoUsageLow,
			stats.rx_ch_stats.bam_stats.bamUtilCount,
			stats.rx_ch_stats.num_bam_int_handled,
			stats.rx_ch_stats.num_db,
			stats.rx_ch_stats.num_unexpected_db,
			stats.rx_ch_stats.num_pkts_in_dis_uninit_state,
			stats.rx_ch_stats.num_ic_inj_vdev_change,
			stats.rx_ch_stats.num_ic_inj_fw_desc_change,
			stats.rx_ch_stats.num_qmb_int_handled,
			stats.rx_ch_stats.reserved1,
			stats.rx_ch_stats.reserved2);
		cnt += nbytes;
	} else {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"Fail to read WDI stats\n");
		cnt += nbytes;
	}

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_write_dbg_cnt(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	u32 option = 0;
	struct ipahal_reg_debug_cnt_ctrl dbg_cnt_ctrl;
	int ret;

	if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_0) {
		IPAERR("IPA_DEBUG_CNT_CTRL is not supported in IPA 4.0\n");
		return -EPERM;
	}

	ret = kstrtou32_from_user(buf, count, 0, &option);
	if (ret)
		return ret;

	memset(&dbg_cnt_ctrl, 0, sizeof(dbg_cnt_ctrl));
	dbg_cnt_ctrl.type = DBG_CNT_TYPE_GENERAL;
	dbg_cnt_ctrl.product = true;
	dbg_cnt_ctrl.src_pipe = 0xff;
	dbg_cnt_ctrl.rule_idx_pipe_rule = false;
	dbg_cnt_ctrl.rule_idx = 0;
	if (option == 1)
		dbg_cnt_ctrl.en = true;
	else
		dbg_cnt_ctrl.en = false;

	IPA_ACTIVE_CLIENTS_INC_SIMPLE();
	ipahal_write_reg_n_fields(IPA_DEBUG_CNT_CTRL_n, 0, &dbg_cnt_ctrl);
	IPA_ACTIVE_CLIENTS_DEC_SIMPLE();

	return count;
}

static ssize_t ipa3_read_dbg_cnt(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	int nbytes;
	u32 regval;

	if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_0) {
		IPAERR("IPA_DEBUG_CNT_REG is not supported in IPA 4.0\n");
		return -EPERM;
	}

	IPA_ACTIVE_CLIENTS_INC_SIMPLE();
	regval =
		ipahal_read_reg_n(IPA_DEBUG_CNT_REG_n, 0);
	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"IPA_DEBUG_CNT_REG_0=0x%x\n", regval);
	IPA_ACTIVE_CLIENTS_DEC_SIMPLE();

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa3_read_msg(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	int nbytes;
	int cnt = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(ipa3_event_name); i++) {
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
				"msg[%u:%27s] W:%u R:%u\n", i,
				ipa3_event_name[i],
				ipa3_ctx->stats.msg_w[i],
				ipa3_ctx->stats.msg_r[i]);
		cnt += nbytes;
	}

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static void ipa3_read_table(
	char *table_addr,
	u32 table_size,
	u32 *total_num_entries,
	u32 *rule_id,
	enum ipahal_nat_type nat_type)
{
	int result;
	char *entry;
	size_t entry_size;
	bool entry_zeroed;
	bool entry_valid;
	u32 i, num_entries = 0, id = *rule_id;
	char *buff;
	size_t buff_size = 2 * IPA_MAX_ENTRY_STRING_LEN;

	IPADBG("In\n");

	if (table_addr == NULL) {
		pr_err("NULL NAT table\n");
		goto bail;
	}

	result = ipahal_nat_entry_size(nat_type, &entry_size);

	if (result) {
		IPAERR("Failed to retrieve size of %s entry\n",
			ipahal_nat_type_str(nat_type));
		goto bail;
	}

	buff = kzalloc(buff_size, GFP_KERNEL);

	if (!buff) {
		IPAERR("Out of memory\n");
		goto bail;
	}

	for (i = 0, entry = table_addr;
		i < table_size;
		++i, ++id, entry += entry_size) {

		result = ipahal_nat_is_entry_zeroed(nat_type, entry,
			&entry_zeroed);

		if (result) {
			IPAERR("Undefined if %s entry is zero\n",
				   ipahal_nat_type_str(nat_type));
			goto free_buf;
		}

		if (entry_zeroed)
			continue;

		result = ipahal_nat_is_entry_valid(nat_type, entry,
			&entry_valid);

		if (result) {
			IPAERR("Undefined if %s entry is valid\n",
				   ipahal_nat_type_str(nat_type));
			goto free_buf;
		}

		if (entry_valid) {
			++num_entries;
			pr_err("\tEntry_Index=%d\n", id);
		} else
			pr_err("\tEntry_Index=%d - Invalid Entry\n", id);

		ipahal_nat_stringify_entry(nat_type, entry,
			buff, buff_size);

		pr_err("%s\n", buff);

		memset(buff, 0, buff_size);
	}

	if (num_entries)
		pr_err("\n");
	else
		pr_err("\tEmpty\n\n");

free_buf:
	kfree(buff);
	*rule_id = id;
	*total_num_entries += num_entries;

bail:
	IPADBG("Out\n");
}

static void ipa3_start_read_memory_device(
	struct ipa3_nat_ipv6ct_common_mem *dev,
	enum ipahal_nat_type nat_type,
	u32 *num_ddr_ent_ptr,
	u32 *num_sram_ent_ptr)
{
	u32 rule_id = 0;

	if (dev->is_ipv6ct_mem) {

		IPADBG("In: v6\n");

		pr_err("%s_Table_Size=%d\n",
			   dev->name, dev->table_entries + 1);

		pr_err("%s_Expansion_Table_Size=%d\n",
			   dev->name, dev->expn_table_entries);

		pr_err("\n%s Base Table:\n", dev->name);

		if (dev->base_table_addr)
			ipa3_read_table(
				dev->base_table_addr,
				dev->table_entries + 1,
				num_ddr_ent_ptr,
				&rule_id,
				nat_type);

		pr_err("%s Expansion Table:\n", dev->name);

		if (dev->expansion_table_addr)
			ipa3_read_table(
				dev->expansion_table_addr,
				dev->expn_table_entries,
				num_ddr_ent_ptr,
				&rule_id,
				nat_type);
	}

	if (dev->is_nat_mem) {
		struct ipa3_nat_mem *nm_ptr = (struct ipa3_nat_mem *) dev;
		struct ipa3_nat_mem_loc_data *mld_ptr = NULL;
		u32 *num_ent_ptr;
		const char *type_ptr;

		IPADBG("In: v4\n");

		if (nm_ptr->active_table == IPA_NAT_MEM_IN_DDR &&
			nm_ptr->ddr_in_use) {

			mld_ptr     = &nm_ptr->mem_loc[IPA_NAT_MEM_IN_DDR];
			num_ent_ptr = num_ddr_ent_ptr;
			type_ptr    = "DDR based table";
		}

		if (nm_ptr->active_table == IPA_NAT_MEM_IN_SRAM &&
			nm_ptr->sram_in_use) {

			mld_ptr     = &nm_ptr->mem_loc[IPA_NAT_MEM_IN_SRAM];
			num_ent_ptr = num_sram_ent_ptr;
			type_ptr    = "SRAM based table";
		}

		if (mld_ptr) {
			pr_err("(%s) %s_Table_Size=%d\n",
				   type_ptr,
				   dev->name,
				   mld_ptr->table_entries + 1);

			pr_err("(%s) %s_Expansion_Table_Size=%d\n",
				   type_ptr,
				   dev->name,
				   mld_ptr->expn_table_entries);

			pr_err("\n(%s) %s_Base Table:\n",
				   type_ptr,
				   dev->name);

			if (mld_ptr->base_table_addr)
				ipa3_read_table(
					mld_ptr->base_table_addr,
					mld_ptr->table_entries + 1,
					num_ent_ptr,
					&rule_id,
					nat_type);

			pr_err("(%s) %s_Expansion Table:\n",
				   type_ptr,
				   dev->name);

			if (mld_ptr->expansion_table_addr)
				ipa3_read_table(
					mld_ptr->expansion_table_addr,
					mld_ptr->expn_table_entries,
					num_ent_ptr,
					&rule_id,
					nat_type);
		}
	}

	IPADBG("Out\n");
}

static void ipa3_finish_read_memory_device(
	struct ipa3_nat_ipv6ct_common_mem *dev,
	u32 num_ddr_entries,
	u32 num_sram_entries)
{
	IPADBG("In\n");

	if (dev->is_ipv6ct_mem) {
		pr_err("Overall number %s entries: %u\n\n",
			   dev->name,
			   num_ddr_entries);
	} else {
		struct ipa3_nat_mem *nm_ptr = (struct ipa3_nat_mem *) dev;

		if (num_ddr_entries)
			pr_err("%s: Overall number of DDR entries: %u\n\n",
				   dev->name,
				   num_ddr_entries);

		if (num_sram_entries)
			pr_err("%s: Overall number of SRAM entries: %u\n\n",
				   dev->name,
				   num_sram_entries);

		pr_err("%s: Driver focus changes to DDR(%u) to SRAM(%u)\n",
			   dev->name,
			   nm_ptr->switch2ddr_cnt,
			   nm_ptr->switch2sram_cnt);
	}

	IPADBG("Out\n");
}

static void ipa3_read_pdn_table(void)
{
	int i, result;
	char *pdn_entry;
	size_t pdn_entry_size;
	bool entry_zeroed;
	bool entry_valid;
	char *buff;
	size_t buff_size = 128;

	IPADBG("In\n");

	if (ipa3_ctx->nat_mem.pdn_mem.base) {

		result = ipahal_nat_entry_size(
			IPAHAL_NAT_IPV4_PDN, &pdn_entry_size);

		if (result) {
			IPAERR("Failed to retrieve size of PDN entry");
			goto bail;
		}

		buff = kzalloc(buff_size, GFP_KERNEL);
		if (!buff) {
			IPAERR("Out of memory\n");
			goto bail;
		}

		for (i = 0, pdn_entry = ipa3_ctx->nat_mem.pdn_mem.base;
			 i < ipa3_get_max_pdn();
			 ++i, pdn_entry += pdn_entry_size) {

			result = ipahal_nat_is_entry_zeroed(
				IPAHAL_NAT_IPV4_PDN,
				pdn_entry, &entry_zeroed);

			if (result) {
				IPAERR("ipahal_nat_is_entry_zeroed() fail\n");
				goto free;
			}

			if (entry_zeroed)
				continue;

			result = ipahal_nat_is_entry_valid(
				IPAHAL_NAT_IPV4_PDN,
				pdn_entry, &entry_valid);

			if (result) {
				IPAERR(
					"Failed to determine whether the PDN entry is valid\n");
				goto free;
			}

			ipahal_nat_stringify_entry(
				IPAHAL_NAT_IPV4_PDN,
				pdn_entry, buff, buff_size);

			if (entry_valid)
				pr_err("PDN %d: %s\n", i, buff);
			else
				pr_err("PDN %d - Invalid: %s\n", i, buff);

			memset(buff, 0, buff_size);
		}
		pr_err("\n");
free:
		kfree(buff);
	}
bail:
	IPADBG("Out\n");
}

static ssize_t ipa3_read_nat4(
	struct file *file,
	char __user *ubuf,
	size_t count,
	loff_t *ppos)
{
	struct ipa3_nat_ipv6ct_common_mem *dev = &ipa3_ctx->nat_mem.dev;
	struct ipa3_nat_mem *nm_ptr = (struct ipa3_nat_mem *) dev;
	struct ipa3_nat_mem_loc_data *mld_ptr = NULL;

	u32  rule_id = 0;

	u32 *num_ents_ptr;
	u32  num_ddr_ents = 0;
	u32  num_sram_ents = 0;

	u32 *num_index_ents_ptr;
	u32  num_ddr_index_ents = 0;
	u32  num_sram_index_ents = 0;

	const char *type_ptr;

	bool any_table_active = (nm_ptr->ddr_in_use || nm_ptr->sram_in_use);

	pr_err("==== NAT Tables Start ====\n");

	if (!dev->is_dev_init) {
		pr_err("NAT hasn't been initialized or not supported\n");
		goto ret;
	}

	mutex_lock(&dev->lock);

	if (!dev->is_hw_init || !any_table_active) {
		pr_err("NAT H/W and/or S/W not initialized\n");
		goto bail;
	}

	if (nm_ptr->sram_in_use) {
		IPADBG("SRAM based table with client 0, enable clk\n");
		IPA_ACTIVE_CLIENTS_INC_SPECIAL("SRAM");
	}

	if (ipa3_ctx->ipa_hw_type >= IPA_HW_v4_0) {
		ipa3_read_pdn_table();
	} else {
		pr_err("NAT Table IP Address=%pI4h\n\n",
			   &ipa3_ctx->nat_mem.public_ip_addr);
	}

	ipa3_start_read_memory_device(
		dev,
		IPAHAL_NAT_IPV4,
		&num_ddr_ents,
		&num_sram_ents);

	if (nm_ptr->active_table == IPA_NAT_MEM_IN_DDR &&
		nm_ptr->ddr_in_use) {

		mld_ptr            = &nm_ptr->mem_loc[IPA_NAT_MEM_IN_DDR];
		num_ents_ptr       = &num_ddr_ents;
		num_index_ents_ptr = &num_ddr_index_ents;
		type_ptr           = "DDR based table";
	}

	if (nm_ptr->active_table == IPA_NAT_MEM_IN_SRAM &&
		nm_ptr->sram_in_use) {

		mld_ptr            = &nm_ptr->mem_loc[IPA_NAT_MEM_IN_SRAM];
		num_ents_ptr       = &num_sram_ents;
		num_index_ents_ptr = &num_sram_index_ents;
		type_ptr           = "SRAM based table";
	}

	if (mld_ptr) {
		/* Print Index tables */
		pr_err("(%s) ipaNatTable Index Table:\n", type_ptr);

		ipa3_read_table(
			mld_ptr->index_table_addr,
			mld_ptr->table_entries + 1,
			num_index_ents_ptr,
			&rule_id,
			IPAHAL_NAT_IPV4_INDEX);

		pr_err("(%s) ipaNatTable Expansion Index Table:\n", type_ptr);

		ipa3_read_table(
			mld_ptr->index_table_expansion_addr,
			mld_ptr->expn_table_entries,
			num_index_ents_ptr,
			&rule_id,
			IPAHAL_NAT_IPV4_INDEX);

		if (*num_ents_ptr != *num_index_ents_ptr)
			IPAERR(
				"(%s) Base Table vs Index Table entry count differs (%u vs %u)\n",
				type_ptr, *num_ents_ptr, *num_index_ents_ptr);
	}

	ipa3_finish_read_memory_device(
		dev,
		num_ddr_ents,
		num_sram_ents);

	if (nm_ptr->sram_in_use) {
		IPADBG("SRAM based table with client 0, disable clk\n");
		IPA_ACTIVE_CLIENTS_DEC_SPECIAL("SRAM");
	}

bail:
	pr_err("==== NAT Tables End ====\n");
	mutex_unlock(&dev->lock);

ret:
	IPADBG("Out\n");

	return 0;
}

static ssize_t ipa3_read_ipv6ct(
	struct file *file,
	char __user *ubuf,
	size_t count,
	loff_t *ppos)
{
	struct ipa3_nat_ipv6ct_common_mem *dev = &ipa3_ctx->ipv6ct_mem.dev;

	u32 num_ddr_ents, num_sram_ents;

	num_ddr_ents = num_sram_ents = 0;

	IPADBG("In\n");

	pr_err("\n");

	if (!dev->is_dev_init) {
		pr_err("IPv6 Conntrack not initialized or not supported\n");
		goto bail;
	}

	if (!dev->is_hw_init) {
		pr_err("IPv6 connection tracking H/W hasn't been initialized\n");
		goto bail;
	}

	mutex_lock(&dev->lock);

	ipa3_start_read_memory_device(
		dev,
		IPAHAL_NAT_IPV6CT,
		&num_ddr_ents,
		&num_sram_ents);

	ipa3_finish_read_memory_device(
		dev,
		num_ddr_ents,
		num_sram_ents);

	mutex_unlock(&dev->lock);

bail:
	IPADBG("Out\n");

	return 0;
}

static ssize_t ipa3_pm_read_stats(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	int result, cnt = 0;

	result = ipa_pm_stat(dbg_buff, IPA_MAX_MSG_LEN);
	if (result < 0) {
		cnt += scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
				"Error in printing PM stat %d\n", result);
		goto ret;
	}
	cnt += result;
ret:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_pm_ex_read_stats(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	int result, cnt = 0;

	result = ipa_pm_exceptions_stat(dbg_buff, IPA_MAX_MSG_LEN);
	if (result < 0) {
		cnt += scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
				"Error in printing PM stat %d\n", result);
		goto ret;
	}
	cnt += result;
ret:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_ipahal_regs(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	IPA_ACTIVE_CLIENTS_INC_SIMPLE();
	ipahal_print_all_regs(true);
	IPA_ACTIVE_CLIENTS_DEC_SIMPLE();

	return 0;
}

static ssize_t ipa3_read_wdi_gsi_stats(struct file *file,
		char __user *ubuf, size_t count, loff_t *ppos)
{
	struct ipa_uc_dbg_ring_stats stats;
	int nbytes;
	int cnt = 0;

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"This feature only support on IPA4.5+\n");
		cnt += nbytes;
		goto done;
	}

	if (!ipa3_get_wdi_gsi_stats(&stats)) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"TX ringFull=%u\n"
			"TX ringEmpty=%u\n"
			"TX ringUsageHigh=%u\n"
			"TX ringUsageLow=%u\n"
			"TX RingUtilCount=%u\n",
			stats.u.ring[1].ringFull,
			stats.u.ring[1].ringEmpty,
			stats.u.ring[1].ringUsageHigh,
			stats.u.ring[1].ringUsageLow,
			stats.u.ring[1].RingUtilCount);
		cnt += nbytes;
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			"RX ringFull=%u\n"
			"RX ringEmpty=%u\n"
			"RX ringUsageHigh=%u\n"
			"RX ringUsageLow=%u\n"
			"RX RingUtilCount=%u\n",
			stats.u.ring[0].ringFull,
			stats.u.ring[0].ringEmpty,
			stats.u.ring[0].ringUsageHigh,
			stats.u.ring[0].ringUsageLow,
			stats.u.ring[0].RingUtilCount);
		cnt += nbytes;
	} else {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"Fail to read WDI GSI stats\n");
		cnt += nbytes;
	}
done:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_wdi3_gsi_stats(struct file *file,
		char __user *ubuf, size_t count, loff_t *ppos)
{
	struct ipa_uc_dbg_ring_stats stats;
	int nbytes;
	int cnt = 0;

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"This feature only support on IPA4.5+\n");
		cnt += nbytes;
		goto done;
	}
	if (!ipa3_get_wdi3_gsi_stats(&stats)) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"TX ringFull=%u\n"
			"TX ringEmpty=%u\n"
			"TX ringUsageHigh=%u\n"
			"TX ringUsageLow=%u\n"
			"TX RingUtilCount=%u\n",
			stats.u.ring[1].ringFull,
			stats.u.ring[1].ringEmpty,
			stats.u.ring[1].ringUsageHigh,
			stats.u.ring[1].ringUsageLow,
			stats.u.ring[1].RingUtilCount);
		cnt += nbytes;
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			"TX1 ringFull=%u\n"
			"TX1 ringEmpty=%u\n"
			"TX1 ringUsageHigh=%u\n"
			"TX1 ringUsageLow=%u\n"
			"TX1 RingUtilCount=%u\n",
			stats.u.ring[2].ringFull,
			stats.u.ring[2].ringEmpty,
			stats.u.ring[2].ringUsageHigh,
			stats.u.ring[2].ringUsageLow,
			stats.u.ring[2].RingUtilCount);
		cnt += nbytes;
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			"RX ringFull=%u\n"
			"RX ringEmpty=%u\n"
			"RX ringUsageHigh=%u\n"
			"RX ringUsageLow=%u\n"
			"RX RingUtilCount=%u\n",
			stats.u.ring[0].ringFull,
			stats.u.ring[0].ringEmpty,
			stats.u.ring[0].ringUsageHigh,
			stats.u.ring[0].ringUsageLow,
			stats.u.ring[0].RingUtilCount);
		cnt += nbytes;
	} else {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"Fail to read WDI GSI stats\n");
		cnt += nbytes;
	}

done:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_11ad_gsi_stats(struct file *file,
		char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes;
	int cnt = 0;

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"This feature only support on IPA4.5+\n");
		cnt += nbytes;
		goto done;
	}
	return 0;
done:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_aqc_gsi_stats(struct file *file,
		char __user *ubuf, size_t count, loff_t *ppos)
{
	struct ipa_uc_dbg_ring_stats stats;
	int nbytes;
	int cnt = 0;

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"This feature only support on IPA4.5+\n");
		cnt += nbytes;
		goto done;
	}
	if (!ipa3_get_aqc_gsi_stats(&stats)) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"TX ringFull=%u\n"
			"TX ringEmpty=%u\n"
			"TX ringUsageHigh=%u\n"
			"TX ringUsageLow=%u\n"
			"TX RingUtilCount=%u\n",
			stats.u.ring[1].ringFull,
			stats.u.ring[1].ringEmpty,
			stats.u.ring[1].ringUsageHigh,
			stats.u.ring[1].ringUsageLow,
			stats.u.ring[1].RingUtilCount);
		cnt += nbytes;
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			"RX ringFull=%u\n"
			"RX ringEmpty=%u\n"
			"RX ringUsageHigh=%u\n"
			"RX ringUsageLow=%u\n"
			"RX RingUtilCount=%u\n",
			stats.u.ring[0].ringFull,
			stats.u.ring[0].ringEmpty,
			stats.u.ring[0].ringUsageHigh,
			stats.u.ring[0].ringUsageLow,
			stats.u.ring[0].RingUtilCount);
		cnt += nbytes;
	} else {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"Fail to read AQC GSI stats\n");
		cnt += nbytes;
	}
done:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_mhip_gsi_stats(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	struct ipa_uc_dbg_ring_stats stats;
	int nbytes;
	int cnt = 0;

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"This feature only support on IPA4.5+\n");
		cnt += nbytes;
		goto done;
	}
	if (!ipa3_get_mhip_gsi_stats(&stats)) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"IPA_CLIENT_MHI_PRIME_TETH_CONS ringFull=%u\n"
			"IPA_CLIENT_MHI_PRIME_TETH_CONS ringEmpty=%u\n"
			"IPA_CLIENT_MHI_PRIME_TETH_CONS ringUsageHigh=%u\n"
			"IPA_CLIENT_MHI_PRIME_TETH_CONS ringUsageLow=%u\n"
			"IPA_CLIENT_MHI_PRIME_TETH_CONS RingUtilCount=%u\n",
			stats.u.ring[1].ringFull,
			stats.u.ring[1].ringEmpty,
			stats.u.ring[1].ringUsageHigh,
			stats.u.ring[1].ringUsageLow,
			stats.u.ring[1].RingUtilCount);
		cnt += nbytes;
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			"IPA_CLIENT_MHI_PRIME_TETH_PROD ringFull=%u\n"
			"IPA_CLIENT_MHI_PRIME_TETH_PROD ringEmpty=%u\n"
			"IPA_CLIENT_MHI_PRIME_TETH_PROD ringUsageHigh=%u\n"
			"IPA_CLIENT_MHI_PRIME_TETH_PROD ringUsageLow=%u\n"
			"IPA_CLIENT_MHI_PRIME_TETH_PROD RingUtilCount=%u\n",
			stats.u.ring[0].ringFull,
			stats.u.ring[0].ringEmpty,
			stats.u.ring[0].ringUsageHigh,
			stats.u.ring[0].ringUsageLow,
			stats.u.ring[0].RingUtilCount);
		cnt += nbytes;
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			"IPA_CLIENT_MHI_PRIME_RMNET_CONS ringFull=%u\n"
			"IPA_CLIENT_MHI_PRIME_RMNET_CONS ringEmpty=%u\n"
			"IPA_CLIENT_MHI_PRIME_RMNET_CONS ringUsageHigh=%u\n"
			"IPA_CLIENT_MHI_PRIME_RMNET_CONS ringUsageLow=%u\n"
			"IPA_CLIENT_MHI_PRIME_RMNET_CONS RingUtilCount=%u\n",
			stats.u.ring[3].ringFull,
			stats.u.ring[3].ringEmpty,
			stats.u.ring[3].ringUsageHigh,
			stats.u.ring[3].ringUsageLow,
			stats.u.ring[3].RingUtilCount);
		cnt += nbytes;
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			"IPA_CLIENT_MHI_PRIME_RMNET_PROD ringFull=%u\n"
			"IPA_CLIENT_MHI_PRIME_RMNET_PROD ringEmpty=%u\n"
			"IPA_CLIENT_MHI_PRIME_RMNET_PROD ringUsageHigh=%u\n"
			"IPA_CLIENT_MHI_PRIME_RMNET_PROD ringUsageLow=%u\n"
			"IPA_CLIENT_MHI_PRIME_RMNET_PROD RingUtilCount=%u\n",
			stats.u.ring[2].ringFull,
			stats.u.ring[2].ringEmpty,
			stats.u.ring[2].ringUsageHigh,
			stats.u.ring[2].ringUsageLow,
			stats.u.ring[2].RingUtilCount);
		cnt += nbytes;
	} else {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"Fail to read WDI GSI stats\n");
		cnt += nbytes;
	}

done:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_usb_gsi_stats(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	struct ipa_uc_dbg_ring_stats stats;
	int nbytes;
	int cnt = 0;

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"This feature only support on IPA4.5+\n");
		cnt += nbytes;
		goto done;
	}
	if (!ipa3_get_usb_gsi_stats(&stats)) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"TX ringFull=%u\n"
			"TX ringEmpty=%u\n"
			"TX ringUsageHigh=%u\n"
			"TX ringUsageLow=%u\n"
			"TX RingUtilCount=%u\n",
			stats.u.ring[1].ringFull,
			stats.u.ring[1].ringEmpty,
			stats.u.ring[1].ringUsageHigh,
			stats.u.ring[1].ringUsageLow,
			stats.u.ring[1].RingUtilCount);
		cnt += nbytes;
		nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
			"RX ringFull=%u\n"
			"RX ringEmpty=%u\n"
			"RX ringUsageHigh=%u\n"
			"RX ringUsageLow=%u\n"
			"RX RingUtilCount=%u\n",
			stats.u.ring[0].ringFull,
			stats.u.ring[0].ringEmpty,
			stats.u.ring[0].ringUsageHigh,
			stats.u.ring[0].ringUsageLow,
			stats.u.ring[0].RingUtilCount);
		cnt += nbytes;
	} else {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"Fail to read WDI GSI stats\n");
		cnt += nbytes;
	}

done:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static ssize_t ipa3_read_app_clk_vote(
	struct file *file,
	char __user *ubuf,
	size_t count,
	loff_t *ppos)
{
	int cnt =
		scnprintf(
			dbg_buff,
			IPA_MAX_MSG_LEN,
			"%u\n",
			ipa3_ctx->app_clock_vote.cnt);

	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static void ipa_dump_status(struct ipahal_pkt_status *status)
{
	IPA_DUMP_STATUS_FIELD(status_opcode);
	IPA_DUMP_STATUS_FIELD(exception);
	IPA_DUMP_STATUS_FIELD(status_mask);
	IPA_DUMP_STATUS_FIELD(pkt_len);
	IPA_DUMP_STATUS_FIELD(endp_src_idx);
	IPA_DUMP_STATUS_FIELD(endp_dest_idx);
	IPA_DUMP_STATUS_FIELD(metadata);
	IPA_DUMP_STATUS_FIELD(flt_local);
	IPA_DUMP_STATUS_FIELD(flt_hash);
	IPA_DUMP_STATUS_FIELD(flt_global);
	IPA_DUMP_STATUS_FIELD(flt_ret_hdr);
	IPA_DUMP_STATUS_FIELD(flt_miss);
	IPA_DUMP_STATUS_FIELD(flt_rule_id);
	IPA_DUMP_STATUS_FIELD(rt_local);
	IPA_DUMP_STATUS_FIELD(rt_hash);
	IPA_DUMP_STATUS_FIELD(ucp);
	IPA_DUMP_STATUS_FIELD(rt_tbl_idx);
	IPA_DUMP_STATUS_FIELD(rt_miss);
	IPA_DUMP_STATUS_FIELD(rt_rule_id);
	IPA_DUMP_STATUS_FIELD(nat_hit);
	IPA_DUMP_STATUS_FIELD(nat_entry_idx);
	IPA_DUMP_STATUS_FIELD(nat_type);
	pr_err("tag = 0x%llx\n", (u64)status->tag_info & 0xFFFFFFFFFFFF);
	IPA_DUMP_STATUS_FIELD(seq_num);
	IPA_DUMP_STATUS_FIELD(time_of_day_ctr);
	IPA_DUMP_STATUS_FIELD(hdr_local);
	IPA_DUMP_STATUS_FIELD(hdr_offset);
	IPA_DUMP_STATUS_FIELD(frag_hit);
	IPA_DUMP_STATUS_FIELD(frag_rule);
	IPA_DUMP_STATUS_FIELD(ttl_dec);
}

static ssize_t ipa_status_stats_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct ipa3_status_stats *stats;
	int i, j;

	stats = kzalloc(sizeof(*stats), GFP_KERNEL);
	if (!stats)
		return -EFAULT;

	for (i = 0; i < ipa3_ctx->ipa_num_pipes; i++) {
		if (!ipa3_ctx->ep[i].sys || !ipa3_ctx->ep[i].sys->status_stat)
			continue;

		memcpy(stats, ipa3_ctx->ep[i].sys->status_stat, sizeof(*stats));
		pr_err("Statuses for pipe %d\n", i);
		for (j = 0; j < IPA_MAX_STATUS_STAT_NUM; j++) {
			pr_err("curr=%d\n", stats->curr);
			ipa_dump_status(&stats->status[stats->curr]);
			pr_err("\n\n\n");
			stats->curr = (stats->curr + 1) %
				IPA_MAX_STATUS_STAT_NUM;
		}
	}

	kfree(stats);
	return 0;
}

static ssize_t ipa3_print_active_clients_log(struct file *file,
		char __user *ubuf, size_t count, loff_t *ppos)
{
	int cnt;
	int table_size;

	if (active_clients_buf == NULL) {
		IPAERR("Active Clients buffer is not allocated");
		return 0;
	}
	memset(active_clients_buf, 0, IPA_DBG_ACTIVE_CLIENT_BUF_SIZE);
	mutex_lock(&ipa3_ctx->ipa3_active_clients.mutex);
	cnt = ipa3_active_clients_log_print_buffer(active_clients_buf,
			IPA_DBG_ACTIVE_CLIENT_BUF_SIZE - IPA_MAX_MSG_LEN);
	table_size = ipa3_active_clients_log_print_table(active_clients_buf
			+ cnt, IPA_MAX_MSG_LEN);
	mutex_unlock(&ipa3_ctx->ipa3_active_clients.mutex);

	return simple_read_from_buffer(ubuf, count, ppos,
			active_clients_buf, cnt + table_size);
}

static ssize_t ipa3_clear_active_clients_log(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	ipa3_active_clients_log_clear();

	return count;
}

static ssize_t ipa3_enable_ipc_low(struct file *file,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	s8 option = 0;
	int ret;

	ret = kstrtos8_from_user(ubuf, count, 0, &option);
	if (ret)
		return ret;

	mutex_lock(&ipa3_ctx->lock);
	if (option) {
		if (!ipa_ipc_low_buff) {
			ipa_ipc_low_buff =
				ipc_log_context_create(IPA_IPC_LOG_PAGES,
					"ipa_low", MINIDUMP_MASK);
		}
			if (ipa_ipc_low_buff == NULL)
				IPADBG("failed to get logbuf_low\n");
		ipa3_ctx->logbuf_low = ipa_ipc_low_buff;
	} else {
		ipa3_ctx->logbuf_low = NULL;
	}
	mutex_unlock(&ipa3_ctx->lock);

	return count;
}

static ssize_t ipa3_read_ipa_max_napi_sort_page_thrshld(struct file *file,
	char __user *buf, size_t count, loff_t *ppos) {

	int nbytes;
	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"page max napi without free page = %d\n",
				ipa3_ctx->ipa_max_napi_sort_page_thrshld);
	return simple_read_from_buffer(buf, count, ppos, dbg_buff, nbytes);

}

static ssize_t ipa3_write_ipa_max_napi_sort_page_thrshld(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos) {

	int ret;
	u8 ipa_max_napi_sort_page_thrshld = 0;

	if (count >= sizeof(dbg_buff))
		return -EFAULT;

	ret = kstrtou8_from_user(buf, count, 0, &ipa_max_napi_sort_page_thrshld);
	if(ret)
		return ret;

	ipa3_ctx->ipa_max_napi_sort_page_thrshld = ipa_max_napi_sort_page_thrshld;

	IPADBG("napi cnt without prealloc pages = %d", ipa3_ctx->ipa_max_napi_sort_page_thrshld);

	return count;
}

static ssize_t ipa3_read_page_wq_reschd_time(struct file *file,
	char __user *buf, size_t count, loff_t *ppos) {

	int nbytes;
	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"Page WQ reschduule time = %d\n",
				ipa3_ctx->page_wq_reschd_time);
	return simple_read_from_buffer(buf, count, ppos, dbg_buff, nbytes);

}

static ssize_t ipa3_write_page_wq_reschd_time(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos) {

	int ret;
	u8 page_wq_reschd_time = 0;

	if (count >= sizeof(dbg_buff))
		return -EFAULT;

	ret = kstrtou8_from_user(buf, count, 0, &page_wq_reschd_time);
	if(ret)
		return ret;

	ipa3_ctx->page_wq_reschd_time = page_wq_reschd_time;

	IPADBG("Updated page WQ reschedule time = %d", ipa3_ctx->page_wq_reschd_time);

	return count;
}

static ssize_t ipa3_read_page_poll_threshold(struct file *file,
	char __user *buf, size_t count, loff_t *ppos) {

	int nbytes;
	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"Page Poll Threshold = %d\n",
				ipa3_ctx->page_poll_threshold);
	return simple_read_from_buffer(buf, count, ppos, dbg_buff, nbytes);

}
static ssize_t ipa3_write_page_poll_threshold(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos) {

	int ret;
	u8 page_poll_threshold =0;

	if (count >= sizeof(dbg_buff))
		return -EFAULT;

	ret = kstrtou8_from_user(buf, count, 0, &page_poll_threshold);
	if(ret)
		return ret;

	if(page_poll_threshold != 0 &&
		page_poll_threshold <= IPA_PAGE_POLL_THRESHOLD_MAX)
		ipa3_ctx->page_poll_threshold = page_poll_threshold;
	else
		IPAERR("Invalid value \n");

	IPADBG("Updated page poll threshold = %d", ipa3_ctx->page_poll_threshold);

	return count;
}

static void ipa3_nat_move_free_cb(void *buff, u32 len, u32 type)
{
	kfree(buff);
}

static ssize_t ipa3_write_nat_table_move(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	u32 direction;
	unsigned long missing;
	char *sptr, *token;
	struct ipa_move_nat_req_msg_v01 *req_data;
	struct ipa_msg_meta msg_meta;

	if (count >= sizeof(dbg_buff))
		return -EFAULT;

	missing = copy_from_user(dbg_buff, buf, count);
	if (missing)
		return -EFAULT;

	dbg_buff[count] = '\0';

	sptr = dbg_buff;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou32(token, 0, &direction))
		return -EINVAL;

	if (direction) {
		pr_err("moving to DDR\n");
		direction = QMI_IPA_MOVE_NAT_TO_DDR_V01;
	} else {
		pr_err("moving to SRAM\n");
		direction = QMI_IPA_MOVE_NAT_TO_SRAM_V01;
	}

	req_data = kzalloc(sizeof(struct ipa_move_nat_req_msg_v01),
		GFP_KERNEL);
	if (!req_data) {
		pr_err("allocation failed\n");
		return EFAULT;
	}

	memset(&msg_meta, 0, sizeof(struct ipa_msg_meta));
	msg_meta.msg_type = IPA_MOVE_NAT_TABLE;
	msg_meta.msg_len = sizeof(struct ipa_move_nat_req_msg_v01);

	req_data->nat_move_direction = direction;

	ipa3_disable_move_nat_resp();
	pr_err("disabled QMI\n");
	/* make sure QMI is disabled before message sent to IPACM */
	wmb();
	if (ipa_send_msg(&msg_meta, req_data, ipa3_nat_move_free_cb)) {
		pr_err("ipa_send_msg failed\nn");
	}
	pr_err("message sent\n");

	return count;
}
#if defined(CONFIG_IPA_TSP)
static ssize_t ipa3_read_tsp(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int i, nbytes = 0;
	struct ipahal_ipa_state_tsp state_tsp;
	u32 qm_non_empty;
	struct ipa_ioc_tsp_ingress_class_params ingr_tc;
	struct ipa_ioc_tsp_egress_prod_params egr_ep;
	struct ipa_ioc_tsp_egress_class_params egr_tc;

	/* Print the global TSP state flags */
	IPA_ACTIVE_CLIENTS_INC_SIMPLE();
	ipahal_read_reg_fields(IPA_STATE_TSP, &state_tsp);
	ipahal_read_reg_fields(IPA_STATE_QMNGR_QUEUE_NONEMPTY, &qm_non_empty);
	IPA_ACTIVE_CLIENTS_DEC_SIMPLE();

	if (state_tsp.traffic_shaper_idle)
		nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"Traffic-Sahper module IDLE\n");
	if (state_tsp.traffic_shaper_fifo_empty)
		nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"Traffic-Sahper FIFO empty\n");
	if (state_tsp.queue_mngr_idle)
		nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"QMNGR overall IDLE\n");
	if (state_tsp.queue_mngr_head_idle)
		nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"QMNGR head module IDLE\n");
	if (state_tsp.queue_mngr_shared_idle)
		nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"QMNGR shared module IDLE\n");
	if (state_tsp.queue_mngr_tail_idle)
		nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"QMNGR tail module IDLE\n");
	if (state_tsp.queue_mngr_block_ctrl_idle)
		nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"Block control module IDLE\n");

	nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"QM non-empty bitmask: 0x%08X\n", qm_non_empty);

	/* Dump Ingress Class, Egress Producer and Egress Class tables */
	nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"Ingress Trafic Class Table:\n");
	nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"TC Index\tMax Rate\tMax Burst\tInclude L2\n");
	for (i = 1; i <= ipa3_ctx->tsp.ingr_tc_max; i++) {
		ipahal_tsp_parse_hw_ingr_tc(ipa3_ctx->tsp.ingr_tc_tbl.base, i, &ingr_tc);
		nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"%02d:\t\t%u\t\t%u\t\t%u\n",
			i, ingr_tc.max_rate, ingr_tc.max_burst, ingr_tc.include_l2_len);
	}

	nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"Egress Producer Table:\n");
	nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"EP Index\tClient\tMax Rate\tMax Burst\n");
	for (i = 0; i < ipa3_ctx->tsp.egr_ep_max; i++) {
		ipahal_tsp_parse_hw_egr_ep(ipa3_ctx->tsp.egr_ep_tbl.base, i, &egr_ep);
		nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"%d:\t\t%d\t%u\t\t%u\n",
			i, ipa3_ctx->tsp.egr_ep_config[i], egr_ep.max_rate, egr_ep.max_burst);
	}

	nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"Egress Trafic Class Table:\n");
	nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"TC Index\tMax Rate\tMax Burst\tG. Rate\tG. Burst\n");
	for (i = 1; i <= ipa3_ctx->tsp.egr_tc_max; i++) {
		ipahal_tsp_parse_hw_egr_tc(ipa3_ctx->tsp.egr_tc_tbl.base, i, &egr_tc);
		nbytes += scnprintf(dbg_buff + nbytes, IPA_MAX_MSG_LEN - nbytes,
			"%02d:\t\t%u\t\t%u\t\t%u\t%u\n",
			i, egr_tc.max_rate, egr_tc.max_burst,
			egr_tc.guaranteed_rate, egr_tc.guaranteed_burst);
	}

	return simple_read_from_buffer(buf, count, ppos, dbg_buff, nbytes);
}

static ssize_t ipa3_write_tsp(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos) {

	int ret;
	u8 option = 0;

	if (count >= sizeof(dbg_buff))
		return -EFAULT;

	ret = kstrtou8_from_user(buf, count, 0, &option);
	if(ret)
		return ret;

	pr_err("TSP write is not implemented.\n");

	return count;
}
#endif
static const struct ipa3_debugfs_file debugfs_files[] = {
	{
		"gen_reg", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_gen_reg
		}
	}, {
		"active_clients", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_print_active_clients_log,
			.write = ipa3_clear_active_clients_log
		}
	}, {
		"ep_reg", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_ep_reg,
			.write = ipa3_write_ep_reg,
		}
	}, {
		"keep_awake", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_keep_awake,
			.write = ipa3_write_keep_awake,
		}
	}, {
		"mpm_ring_size_dl", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_mpm_ring_size_dl,
			.write = ipa3_write_mpm_ring_size_dl,
		}
	}, {
		"mpm_ring_size_ul", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_mpm_ring_size_ul,
			.write = ipa3_write_mpm_ring_size_ul,
		}
	}, {
		"mpm_uc_thresh", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_mpm_uc_thresh,
			.write = ipa3_write_mpm_uc_thresh,
		}
	}, {
		"mpm_teth_aggr_size", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_mpm_teth_aggr_size,
			.write = ipa3_write_mpm_teth_aggr_size,
		}
	}, {
		"set_clk_idx", IPA_READ_WRITE_MODE, NULL, {
			.write = ipa3_set_clk_index,
		}
	}, {
		"holb", IPA_WRITE_ONLY_MODE, NULL, {
			.write = ipa3_write_ep_holb,
		}
	}, {
		"holb_monitor_client_param", IPA_WRITE_ONLY_MODE, NULL, {
			.write = ipa3_write_holb_monitor_client,
		}
	}, {
		"holb_monitor_client_add_del", IPA_WRITE_ONLY_MODE, NULL, {
			.write = ipa3_write_holb_monitor_client_add_del,
		}
	}, {
		"holb_events", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_holb_events,
		}
	}, {
		"hdr", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_hdr,
		}
	}, {
		"proc_ctx", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_proc_ctx,
		}
	}, {
		"ip4_rt", IPA_READ_ONLY_MODE, (void *)IPA_IP_v4, {
			.read = ipa3_read_rt,
			.open = ipa3_open_dbg,
		}
	}, {
		"ip4_rt_hw", IPA_READ_ONLY_MODE, (void *)IPA_IP_v4, {
			.read = ipa3_read_rt_hw,
			.open = ipa3_open_dbg,
		}
	}, {
		"ip6_rt", IPA_READ_ONLY_MODE, (void *)IPA_IP_v6, {
			.read = ipa3_read_rt,
			.open = ipa3_open_dbg,
		}
	}, {
		"ip6_rt_hw", IPA_READ_ONLY_MODE, (void *)IPA_IP_v6, {
			.read = ipa3_read_rt_hw,
			.open = ipa3_open_dbg,
		}
	}, {
		"ip4_flt", IPA_READ_ONLY_MODE, (void *)IPA_IP_v4, {
			.read = ipa3_read_flt,
			.open = ipa3_open_dbg,
		}
	}, {
		"ip4_flt_hw", IPA_READ_ONLY_MODE, (void *)IPA_IP_v4, {
			.read = ipa3_read_flt_hw,
			.open = ipa3_open_dbg,
		}
	}, {
		"ip6_flt", IPA_READ_ONLY_MODE, (void *)IPA_IP_v6, {
			.read = ipa3_read_flt,
			.open = ipa3_open_dbg,
		}
	}, {
		"ip6_flt_hw", IPA_READ_ONLY_MODE, (void *)IPA_IP_v6, {
			.read = ipa3_read_flt_hw,
			.open = ipa3_open_dbg,
		}
	}, {
		"stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_stats,
		}
	}, {
		"wstats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_wstats,
		}
	}, {
		"odlstats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_odlstats,
		}
	}, {
		"page_recycle_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_page_recycle_stats,
		}
	}, {
		"lan_coal_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_lan_coal_stats,
		}
	}, {
		"cache_recycle_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_cache_recycle_stats,
		}
	}, {
		"wdi", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_wdi,
		}
	}, {
		"ntn", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_ntn,
		}
	}, {
		"dbg_cnt", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_dbg_cnt,
			.write = ipa3_write_dbg_cnt,
		}
	}, {
		"msg", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_msg,
		}
	}, {
		"ip4_nat", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_nat4,
		}
	}, {
		"ipv6ct", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_ipv6ct,
		}
	}, {
		"pm_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_pm_read_stats,
		}
	}, {
		"pm_ex_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_pm_ex_read_stats,
		}
	}, {
		"status_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa_status_stats_read,
		}
	}, {
		"enable_low_prio_print", IPA_WRITE_ONLY_MODE, NULL, {
			.write = ipa3_enable_ipc_low,
		}
	}, {
		"ipa_dump_regs", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_ipahal_regs,
		}
	}, {
		"wdi_gsi_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_wdi_gsi_stats,
		}
	}, {
		"wdi3_gsi_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_wdi3_gsi_stats,
		}
	}, {
		"11ad_gsi_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_11ad_gsi_stats,
		}
	}, {
		"aqc_gsi_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_aqc_gsi_stats,
		}
	}, {
		"mhip_gsi_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_mhip_gsi_stats,
		}
	}, {
		"usb_gsi_stats", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_usb_gsi_stats,
		}
	}, {
		"app_clk_vote_cnt", IPA_READ_ONLY_MODE, NULL, {
			.read = ipa3_read_app_clk_vote,
		}
	}, {
		"page_poll_threshold", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_page_poll_threshold,
			.write = ipa3_write_page_poll_threshold,
		}
	}, {
		"move_nat_table_to_ddr", IPA_WRITE_ONLY_MODE, NULL,{
			.write = ipa3_write_nat_table_move,
		}
	}, {
		"page_wq_reschd_time", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_page_wq_reschd_time,
			.write = ipa3_write_page_wq_reschd_time,
		}
	}, {
		"ipa_max_napi_sort_page_thrshld", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_ipa_max_napi_sort_page_thrshld,
			.write = ipa3_write_ipa_max_napi_sort_page_thrshld,
		}
#if defined(CONFIG_IPA_TSP)
	}, {
		"tsp", IPA_READ_WRITE_MODE, NULL, {
			.read = ipa3_read_tsp,
			.write = ipa3_write_tsp,
		}
#endif
	},
};

void ipa3_debugfs_init(void)
{
	const size_t debugfs_files_num =
		sizeof(debugfs_files) / sizeof(struct ipa3_debugfs_file);
	size_t i;
	struct dentry *file;

	dent = debugfs_create_dir("ipa", NULL);
	if (IS_ERR(dent)) {
		IPAERR("fail to create folder in debug_fs.\n");
		return;
	}

	debugfs_create_u32("hw_type", IPA_READ_ONLY_MODE,
		dent, &ipa3_ctx->ipa_hw_type);

	for (i = 0; i < debugfs_files_num; ++i) {
		const struct ipa3_debugfs_file *curr = &debugfs_files[i];

		file = debugfs_create_file(curr->name, curr->mode, dent,
			curr->data, &curr->fops);
		if (!file || IS_ERR(file)) {
			IPAERR("fail to create file for debug_fs %s\n",
				curr->name);
			goto fail;
		}
	}

	active_clients_buf = NULL;
	active_clients_buf = kzalloc(IPA_DBG_ACTIVE_CLIENT_BUF_SIZE,
			GFP_KERNEL);
	if (active_clients_buf == NULL)
		goto fail;

	debugfs_create_u32("enable_clock_scaling", IPA_READ_WRITE_MODE,
		dent, &ipa3_ctx->enable_clock_scaling);

	debugfs_create_u32("tx_wrapper_cache_max_size",
		IPA_READ_WRITE_MODE,
		dent, &ipa3_ctx->tx_wrapper_cache_max_size);

	debugfs_create_u32("enable_napi_chain", IPA_READ_WRITE_MODE,
		dent, &ipa3_ctx->enable_napi_chain);

	debugfs_create_u32("clock_scaling_bw_threshold_nominal_mbps",
		IPA_READ_WRITE_MODE, dent,
		&ipa3_ctx->ctrl->clock_scaling_bw_threshold_nominal);

	debugfs_create_u32("clock_scaling_bw_threshold_turbo_mbps",
			IPA_READ_WRITE_MODE, dent,
			&ipa3_ctx->ctrl->clock_scaling_bw_threshold_turbo);

	debugfs_create_u32("clk_rate", IPA_READ_ONLY_MODE,
		dent, &ipa3_ctx->curr_ipa_clk_rate);

	ipa_debugfs_init_stats(dent);

	ipa3_wigig_init_debugfs_i(dent);

	return;

fail:
	debugfs_remove_recursive(dent);
}

void ipa3_debugfs_remove(void)
{
	if (IS_ERR(dent)) {
		IPAERR("Debugfs:folder was not created.\n");
		return;
	}
	if (active_clients_buf != NULL) {
		kfree(active_clients_buf);
		active_clients_buf = NULL;
	}
	debugfs_remove_recursive(dent);
}

struct dentry *ipa_debugfs_get_root(void)
{
	return dent;
}
EXPORT_SYMBOL(ipa_debugfs_get_root);

static ssize_t ipa3_eth_read_status(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes;
	int cnt = 0;
	int i, j, k, type;
	struct ipa3_eth_info eth_info;

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"This feature only support on IPA4.5+\n");
		cnt += nbytes;
		goto done;
	}

	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"%15s|%10s|%10s|%30s|%10s|%10s\n", "protocol",
			"instance", "pipe_hdl", "pipe_enum",
			"pipe_id", "ch_id");
	cnt += nbytes;
	for (i = 0; i < IPA_ETH_CLIENT_MAX; i++) {
		for (j = 0; j < IPA_ETH_INST_ID_MAX; j++) {
			eth_info = ipa3_ctx->eth_info[i][j];
			for (k = 0; k < eth_info.num_ch; k++) {
				if (eth_info.map[j].valid) {
					type = eth_info.map[k].type;
					nbytes = scnprintf(dbg_buff + cnt,
						IPA_MAX_MSG_LEN - cnt,
						"%15s|%10d|%10d|%30s|%10d|%10d\n",
						ipa_eth_clients_strings[i],
						j,
						eth_info.map[k].pipe_hdl,
						ipa_clients_strings[type],
						eth_info.map[k].pipe_id,
						eth_info.map[k].ch_id);
					cnt += nbytes;
				}
			}
		}
	}
done:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static const struct file_operations fops_ipa_eth_status = {
	.read = ipa3_eth_read_status,
};

void ipa3_eth_debugfs_init(void)
{
	struct dentry *file;

	if (IS_ERR_OR_NULL(dent)) {
		IPAERR("debugs root not created\n");
		return;
	}
	dent_eth = debugfs_create_dir("eth", dent);
	if (IS_ERR(dent)) {
		IPAERR("fail to create folder in debug_fs.\n");
		return;
	}
	file = debugfs_create_file("status", IPA_READ_ONLY_MODE,
		dent_eth, NULL, &fops_ipa_eth_status);
	if (!file) {
		IPAERR("could not create status\n");
		goto fail;
	}
	return;

fail:
	debugfs_remove_recursive(dent_eth);
}
EXPORT_SYMBOL(ipa3_eth_debugfs_init);

static ssize_t ipa3_eth_read_perf_status(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes;
	int cnt = 0;
	struct ipa_eth_client *client;
	struct ipa_uc_dbg_ring_stats stats;
	int tx_ep, rx_ep;
	int ret;

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5
		&& (ipa3_ctx->ipa_hw_type != IPA_HW_v4_1
		|| ipa3_ctx->platform_type != IPA_PLAT_TYPE_APQ)) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"This feature only support on IPA4.5+\n");
		cnt += nbytes;
		goto done;
	}
	client = (struct ipa_eth_client *)file->private_data;
	switch (client->client_type) {
	case IPA_ETH_CLIENT_AQC107:
	case IPA_ETH_CLIENT_AQC113:
	case IPA_ETH_CLIENT_NTN:
		if (client->client_type == IPA_ETH_CLIENT_NTN) {
			ret = ipa3_get_ntn_gsi_stats(&stats);
			tx_ep = IPA_CLIENT_ETHERNET_CONS;
			rx_ep = IPA_CLIENT_ETHERNET_PROD;
		} else {
			ret = ipa3_get_aqc_gsi_stats(&stats);
			tx_ep = IPA_CLIENT_AQC_ETHERNET_CONS;
			rx_ep = IPA_CLIENT_AQC_ETHERNET_PROD;
		}
		if (!ret) {
			nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"%s_ringFull=%u\n"
			"%s_ringEmpty=%u\n"
			"%s_ringUsageHigh=%u\n"
			"%s_ringUsageLow=%u\n"
			"%s_RingUtilCount=%u\n",
			ipa_clients_strings[tx_ep],
			stats.u.ring[1].ringFull,
			ipa_clients_strings[tx_ep],
			stats.u.ring[1].ringEmpty,
			ipa_clients_strings[tx_ep],
			stats.u.ring[1].ringUsageHigh,
			ipa_clients_strings[tx_ep],
			stats.u.ring[1].ringUsageLow,
			ipa_clients_strings[tx_ep],
			stats.u.ring[1].RingUtilCount);
			cnt += nbytes;
			nbytes = scnprintf(dbg_buff + cnt,
			IPA_MAX_MSG_LEN - cnt,
			"%s_ringFull=%u\n"
			"%s_ringEmpty=%u\n"
			"%s_ringUsageHigh=%u\n"
			"%s_ringUsageLow=%u\n"
			"%s_RingUtilCount=%u\n",
			ipa_clients_strings[rx_ep],
			stats.u.ring[0].ringFull,
			ipa_clients_strings[rx_ep],
			stats.u.ring[0].ringEmpty,
			ipa_clients_strings[rx_ep],
			stats.u.ring[0].ringUsageHigh,
			ipa_clients_strings[rx_ep],
			stats.u.ring[0].ringUsageLow,
			ipa_clients_strings[rx_ep],
			stats.u.ring[0].RingUtilCount);
			cnt += nbytes;
		} else {
			nbytes = scnprintf(dbg_buff,
				IPA_MAX_MSG_LEN,
				"Fail to read [%s][%s] GSI stats\n",
				ipa_clients_strings[rx_ep],
				ipa_clients_strings[tx_ep]);
			cnt += nbytes;
		}
		break;
	case IPA_ETH_CLIENT_RTK8111K:
	case IPA_ETH_CLIENT_RTK8125B:
		ret = ipa3_get_rtk_gsi_stats(&stats);
		tx_ep = IPA_CLIENT_RTK_ETHERNET_CONS;
		rx_ep = IPA_CLIENT_RTK_ETHERNET_PROD;
		if (!ret) {
			nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
			"%s_ringFull=%u\n"
			"%s_ringEmpty=%u\n"
			"%s_ringUsageHigh=%u\n"
			"%s_ringUsageLow=%u\n"
			"%s_RingUtilCount=%u\n"
			"%s_trCount=%u\n"
			"%s_erCound=%u\n"
			"%s_totalAoSCount=%u\n"
			"%s_busytime=%llu\n",
			ipa_clients_strings[tx_ep],
			stats.u.rtk[1].commStats.ringFull,
			ipa_clients_strings[tx_ep],
			stats.u.rtk[1].commStats.ringEmpty,
			ipa_clients_strings[tx_ep],
			stats.u.rtk[1].commStats.ringUsageHigh,
			ipa_clients_strings[tx_ep],
			stats.u.rtk[1].commStats.ringUsageLow,
			ipa_clients_strings[tx_ep],
			stats.u.rtk[1].commStats.RingUtilCount,
			ipa_clients_strings[tx_ep],
			stats.u.rtk[1].trCount,
			ipa_clients_strings[tx_ep],
			stats.u.rtk[1].erCount,
			ipa_clients_strings[tx_ep],
			stats.u.rtk[1].totalAosCount,
			ipa_clients_strings[tx_ep],
			stats.u.rtk[1].busyTime);
			cnt += nbytes;
			nbytes = scnprintf(dbg_buff + cnt,
			IPA_MAX_MSG_LEN - cnt,
			"%s_ringFull=%u\n"
			"%s_ringEmpty=%u\n"
			"%s_ringUsageHigh=%u\n"
			"%s_ringUsageLow=%u\n"
			"%s_RingUtilCount=%u\n"
			"%s_trCount=%u\n"
			"%s_erCount=%u\n"
			"%s_totalAoSCount=%u\n"
			"%s_busytime=%llu\n",
			ipa_clients_strings[rx_ep],
			stats.u.rtk[0].commStats.ringFull,
			ipa_clients_strings[rx_ep],
			stats.u.rtk[0].commStats.ringEmpty,
			ipa_clients_strings[rx_ep],
			stats.u.rtk[0].commStats.ringUsageHigh,
			ipa_clients_strings[rx_ep],
			stats.u.rtk[0].commStats.ringUsageLow,
			ipa_clients_strings[rx_ep],
			stats.u.rtk[0].commStats.RingUtilCount,
			ipa_clients_strings[rx_ep],
			stats.u.rtk[0].trCount,
			ipa_clients_strings[rx_ep],
			stats.u.rtk[0].erCount,
			ipa_clients_strings[rx_ep],
			stats.u.rtk[0].totalAosCount,
			ipa_clients_strings[rx_ep],
			stats.u.rtk[0].busyTime);
			cnt += nbytes;
		} else {
			nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"Fail to read RTK GSI stats\n");
			cnt += nbytes;
		}
		break;
	default:
		ret = -EFAULT;
	}

done:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

#if IPA_ETH_API_VER >= 2
static void __ipa_ntn3_client_stats_read(int *cnt, struct ipa_ntn3_client_stats *s,
	const char *str_client_tx, const char *str_client_rx)
{
	int nbytes;

	nbytes = scnprintf(dbg_buff + *cnt, IPA_MAX_MSG_LEN - *cnt,
		"%s_RP=0x%x\n"
		"%s_WP=0x%x\n"
		"%s_ntn_pending_db_after_rollback:%u\n"
		"%s_msi_db_idx_val:%u\n"
		"%s_tx_derr_counter:%u\n"
		"%s_ntn_tx_oob_counter:%u\n"
		"%s_ntn_accumulated_tres_handled:%u\n"
		"%s_ntn_rollbacks_counter:%u\n"
		"%s_ntn_msi_db_count:%u\n",
		str_client_tx, s->tx_stats.rp,
		str_client_tx, s->tx_stats.wp,
		str_client_tx, s->tx_stats.pending_db_after_rollback,
		str_client_tx, s->tx_stats.msi_db_idx,
		str_client_tx, s->tx_stats.derr_cnt,
		str_client_tx, s->tx_stats.oob_cnt,
		str_client_tx, s->tx_stats.tres_handled,
		str_client_tx, s->tx_stats.rollbacks_cnt,
		str_client_tx, s->tx_stats.msi_db_cnt);
	*cnt += nbytes;
	nbytes = scnprintf(dbg_buff + *cnt, IPA_MAX_MSG_LEN - *cnt,
		"%s_RP=0x%x\n"
		"%s_WP=0x%x\n"
		"%s_ntn_pending_db_after_rollback:%u\n"
		"%s_msi_db_idx_val:%u\n"
		"%s_ntn_rx_chain_counter:%u\n"
		"%s_ntn_rx_err_counter:%u\n"
		"%s_ntn_accumulated_tres_handled:%u\n"
		"%s_ntn_rollbacks_counter:%u\n"
		"%s_ntn_msi_db_count:%u\n",
		str_client_rx, s->rx_stats.rp,
		str_client_rx, s->rx_stats.wp,
		str_client_rx, s->rx_stats.pending_db_after_rollback,
		str_client_rx, s->rx_stats.msi_db_idx,
		str_client_rx, s->rx_stats.chain_cnt,
		str_client_rx, s->rx_stats.err_cnt,
		str_client_rx, s->rx_stats.tres_handled,
		str_client_rx, s->rx_stats.rollbacks_cnt,
		str_client_rx, s->rx_stats.msi_db_cnt);
	*cnt += nbytes;
}
#endif

static ssize_t ipa3_eth_read_err_status(struct file *file,
	char __user *ubuf, size_t count, loff_t *ppos)
{
	int nbytes;
	int cnt = 0;
	struct ipa_eth_client *client;
	int tx_ep, rx_ep;
	struct ipa3_eth_error_stats tx_stats;
	struct ipa3_eth_error_stats rx_stats;
	int scratch_num;
#if IPA_ETH_API_VER >= 2
	struct ipa_ntn3_client_stats ntn3_stats;
	const char *str_client_tx, *str_client_rx;
#endif

	memset(&tx_stats, 0, sizeof(struct ipa3_eth_error_stats));
	memset(&rx_stats, 0, sizeof(struct ipa3_eth_error_stats));

	if (ipa3_ctx->ipa_hw_type < IPA_HW_v4_5
		&& (ipa3_ctx->ipa_hw_type != IPA_HW_v4_1
		|| ipa3_ctx->platform_type != IPA_PLAT_TYPE_APQ)) {
		nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
				"This feature only support on IPA4.5+\n");
		cnt += nbytes;
		goto done;
	}
	client = (struct ipa_eth_client *)file->private_data;

	switch (client->client_type) {
	case IPA_ETH_CLIENT_AQC107:
	case IPA_ETH_CLIENT_AQC113:
		tx_ep = IPA_CLIENT_AQC_ETHERNET_CONS;
		rx_ep = IPA_CLIENT_AQC_ETHERNET_PROD;
		scratch_num = 7;
	case IPA_ETH_CLIENT_RTK8111K:
	case IPA_ETH_CLIENT_RTK8125B:
		tx_ep = IPA_CLIENT_RTK_ETHERNET_CONS;
		rx_ep = IPA_CLIENT_RTK_ETHERNET_PROD;
		scratch_num = 5;
		break;
	case IPA_ETH_CLIENT_NTN:
		tx_ep = IPA_CLIENT_ETHERNET_CONS;
		rx_ep = IPA_CLIENT_ETHERNET_PROD;
		scratch_num = 6;
#if IPA_ETH_API_VER >= 2
	case IPA_ETH_CLIENT_NTN3:

		memset(&ntn3_stats, 0, sizeof(ntn3_stats));
		if (strstr(file->f_path.dentry->d_name.name, "0_status")) {
			ipa_eth_ntn3_get_status(&ntn3_stats, 0);
			str_client_tx = ipa_clients_strings[IPA_CLIENT_ETHERNET_CONS];
			str_client_rx = ipa_clients_strings[IPA_CLIENT_ETHERNET_PROD];
		} else {
			ipa_eth_ntn3_get_status(&ntn3_stats, 1);
			str_client_tx = ipa_clients_strings[IPA_CLIENT_ETHERNET2_CONS];
			str_client_rx = ipa_clients_strings[IPA_CLIENT_ETHERNET2_PROD];
		}
		__ipa_ntn3_client_stats_read(&cnt, &ntn3_stats, str_client_tx, str_client_rx);
		goto done;
#endif
	default:
		IPAERR("Not supported\n");
		return 0;
	}
	ipa3_eth_get_status(tx_ep, scratch_num, &tx_stats);
	ipa3_eth_get_status(rx_ep, scratch_num, &rx_stats);

	nbytes = scnprintf(dbg_buff, IPA_MAX_MSG_LEN,
		"%s_RP=0x%x\n"
		"%s_WP=0x%x\n"
		"%s_err:%u (scratch %d)\n",
		ipa_clients_strings[tx_ep],
		tx_stats.rp,
		ipa_clients_strings[tx_ep],
		tx_stats.wp,
		ipa_clients_strings[tx_ep],
		tx_stats.err, scratch_num);
	cnt += nbytes;
	nbytes = scnprintf(dbg_buff + cnt, IPA_MAX_MSG_LEN - cnt,
		"%s_RP=0x%x\n"
		"%s_WP=0x%x\n"
		"%s_err:%u (scratch %d)\n",
		ipa_clients_strings[rx_ep],
		rx_stats.rp,
		ipa_clients_strings[rx_ep],
		rx_stats.wp,
		ipa_clients_strings[rx_ep],
		rx_stats.err, scratch_num);
	cnt += nbytes;
done:
	return simple_read_from_buffer(ubuf, count, ppos, dbg_buff, cnt);
}

static const struct file_operations fops_ipa_eth_stats = {
	.read = ipa3_eth_read_perf_status,
	.open = ipa3_open_dbg,
};
static const struct file_operations fops_ipa_eth_client_status = {
	.read = ipa3_eth_read_err_status,
	.open = ipa3_open_dbg,
};
void ipa3_eth_debugfs_add_node(struct ipa_eth_client *client)
{
	struct dentry *file = NULL;
	int type, inst_id;
	char name[IPA_RESOURCE_NAME_MAX];

	if (IS_ERR_OR_NULL(dent_eth)) {
		IPAERR("debugs eth root not created\n");
		return;
	}

	if (client == NULL) {
		IPAERR_RL("invalid input\n");
		return;
	}

	type = client->client_type;
	inst_id = client->inst_id;
	if (type < IPA_ETH_CLIENT_MAX) {
		snprintf(name, IPA_RESOURCE_NAME_MAX,
			"%s_%d_stats", ipa_eth_clients_strings[type], inst_id);
		file = debugfs_create_file(name, IPA_READ_ONLY_MODE,
			dent_eth, (void *)client, &fops_ipa_eth_stats);
	}
	if (!file) {
		IPAERR("could not create hw_type file\n");
		return;
	}
	if (type < IPA_ETH_CLIENT_MAX) {
		snprintf(name, IPA_RESOURCE_NAME_MAX,
			"%s_%d_status", ipa_eth_clients_strings[type], inst_id);
		file = debugfs_create_file(name, IPA_READ_ONLY_MODE,
			dent_eth, (void *)client, &fops_ipa_eth_client_status);
	}
	if (!file) {
		IPAERR("could not create hw_type file\n");
		goto fail;
	}
	return;
fail:
	debugfs_remove_recursive(dent_eth);
}
EXPORT_SYMBOL(ipa3_eth_debugfs_add_node);

#else /* !CONFIG_DEBUG_FS */
#define INVALID_NO_OF_CHAR (-1)
void ipa3_debugfs_init(void) {}
void ipa3_debugfs_remove(void) {}
int _ipa_read_ep_reg_v3_0(char *buf, int max_len, int pipe)
{
	return INVALID_NO_OF_CHAR;
}
int _ipa_read_ep_reg_v4_0(char *buf, int max_len, int pipe)
{
	return INVALID_NO_OF_CHAR;
}
void ipa3_eth_debugfs_init(void) {}
void ipa3_eth_debugfs_add(struct ipa_eth_client *client) {}
#endif
