// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */

#ifndef __OPLUS_CHG_UFCS_H__
#define __OPLUS_CHG_UFCS_H__

#include <oplus_mms.h>

enum ufcs_topic_item {
	UFCS_ITEM_ONLINE,
	UFCS_ITEM_CHARGING,
	UFCS_ITEM_ADAPTER_ID,
	UFCS_ITEM_OPLUS_ADAPTER,
};

enum ufcs_fastchg_type {
	UFCS_FASTCHG_TYPE_UNKOWN,
	UFCS_FASTCHG_TYPE_THIRD = 0x8211,
	UFCS_FASTCHG_TYPE_V1 = 0x4211,
	UFCS_FASTCHG_TYPE_V2,
	UFCS_FASTCHG_TYPE_V3,
	UFCS_FASTCHG_TYPE_OTHER,
};

enum ufcs_power_type {
	UFCS_POWER_TYPE_UNKOWN,
	UFCS_POWER_TYPE_THIRD = 18,
	UFCS_POWER_TYPE_V1 = 33,
	UFCS_POWER_TYPE_V2 = 65,
	UFCS_POWER_TYPE_OTHER = 240,
};

enum ufcs_protocol_type {
	PROTOCOL_CHARGING_UNKNOWN = 0,
	PROTOCOL_CHARGING_PPS_OPLUS,
	PROTOCOL_CHARGING_PPS_THIRD,
	PROTOCOL_CHARGING_UFCS_THIRD,
	PROTOCOL_CHARGING_UFCS_OPLUS,
	PROTOCOL_CHARGING_SVOOC_OPLUS,
	PROTOCOL_CHARGING_SVOOC_THIRD,
	PROTOCOL_CHARGING_MAX = 100,
};

int oplus_ufcs_current_to_level(int curr);
enum ufcs_protocol_type oplus_ufcs_adapter_id_to_protocol_type(u32 id);
int oplus_ufcs_adapter_id_to_power(u32 id);

#endif /* __OPLUS_CHG_UFCS_H__ */
