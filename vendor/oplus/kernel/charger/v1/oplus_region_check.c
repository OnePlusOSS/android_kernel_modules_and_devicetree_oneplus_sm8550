// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2023 Oplus. All rights reserved.
 */

#include <linux/of.h>
#include "oplus_region_check.h"

#define PPS_REGION_MAX 16

static u8 region_id = 0x00;
static u32 pps_region_list[PPS_REGION_MAX] = {0};
static int pps_list_length = 0;
static u32 pps_priority_region_list[PPS_REGION_MAX] = {0};
static int pps_priority_list_length = 0;

static bool get_regionid_from_cmdline(void)
{
	struct device_node *np;
	const char *bootparams = NULL;
	char *str;
	int temp_region = 0;
	int ret = 0;

	np = of_find_node_by_path("/chosen");
	if (np) {
		ret = of_property_read_string(np, "bootargs", &bootparams);
		if (!bootparams || ret < 0) {
			chg_err("failed to get bootargs property\n");
			return false;
		}

		str = strstr(bootparams, "oplus_region=");
		if (str) {
			str += strlen("oplus_region=");
			ret = get_option(&str, &temp_region);
			if (ret == 1)
				region_id = temp_region & 0xFF;
			else
				region_id = 0;
			chg_err("oplus_region=0x%02x\n", region_id);
			return true;
		}
	}
	return false;
}

static void oplus_parse_pps_region_list(struct oplus_chg_chip *chip)
{
	int len = 0;
	int rc = 0;
	struct device_node *node;

	node = chip->dev->of_node;

	rc = of_property_count_elems_of_size(
		node, "oplus,pps_region_list", sizeof(u32));
	if (rc >= 0) {
		len = rc <= PPS_REGION_MAX ? rc : PPS_REGION_MAX;
		rc = of_property_read_u32_array(node, "oplus,pps_region_list",
						pps_region_list, len);
		if (rc < 0) {
			chg_err("parse pps_region_list failed, rc=%d\n", rc);
		}
	} else {
		len = 0;
		chg_err("parse pps_region_list_length failed, rc=%d\n", rc);
	}
	pps_list_length = len;
	for (len = 0; len < pps_list_length; len++)
		chg_err("pps_region_list[%d]=0x%08x", len, pps_region_list[len]);

	rc = of_property_count_elems_of_size(
		node, "oplus,pps_priority_list", sizeof(u32));
	if (rc >= 0) {
		len = rc <= PPS_REGION_MAX ? rc : PPS_REGION_MAX;
		rc = of_property_read_u32_array(node, "oplus,pps_priority_list",
						pps_priority_region_list, len);
		if (rc < 0) {
			chg_err("parse pps_region_list failed, rc=%d\n", rc);
		}
	} else {
		len = 0;
		chg_err("parse pps_region_list_length failed, rc=%d\n", rc);
	}
	pps_priority_list_length = len;
	for (len = 0; len < pps_priority_list_length; len++)
		chg_err("pps_priority_region_list[%d]=0x%08x", len, pps_priority_region_list[len]);
}

static bool find_id_in_list(u8 id, u32 *id_list, int list_length)
{
	int index = 0;
	u8 region_id_tmp = 0;

	if (id == 0x00 || list_length == 0)
		return false;

	while (index < list_length) {
		region_id_tmp = id_list[index] & 0xFF;
		chg_err("pps_list[%d]=0x%x", index, id_list[index]);
		if (id == region_id_tmp) {
			return true;
		}
		index++;
	}

	return false;
}

bool third_pps_supported_from_nvid(void)
{
	return find_id_in_list(region_id, pps_region_list, pps_list_length);
}

bool third_pps_priority_than_svooc(void)
{
	return find_id_in_list(region_id, pps_priority_region_list, pps_priority_list_length);
}

void oplus_chg_region_check_init(struct oplus_chg_chip *chip)
{
	bool ret = 0;

	if (chip == NULL)
		return;

	ret = get_regionid_from_cmdline();
	if (ret)
		oplus_parse_pps_region_list(chip);
}

