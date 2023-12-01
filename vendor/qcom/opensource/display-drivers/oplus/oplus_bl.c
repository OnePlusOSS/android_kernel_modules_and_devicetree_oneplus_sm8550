/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
**
** File : oplus_bl.c
** Description : oplus display backlight
** Version : 2.0
** Date : 2021/02/22
** Author : Display
******************************************************************/

#include "oplus_bl.h"
#include "oplus_display_interface.h"
#include "oplus_display_panel_common.h"
#if defined(CONFIG_PXLW_IRIS)
#include "dsi_iris_api.h"
#endif

char oplus_global_hbm_flags = 0x0;
static int enable_hbm_enter_dly_on_flags = 0;
static int enable_hbm_exit_dly_on_flags = 0;
extern u32 oplus_last_backlight;
static u32 pwm_switch_cmd_restore = 0;
static int backlight_onepulse_normal_buf[];
static int backlight_onepulse_max_buf[];

int oplus_panel_parse_bl_config(struct dsi_panel *panel)
{
	int rc = 0;
	u32 val = 0;
	struct dsi_parser_utils *utils = &panel->utils;

#if defined(CONFIG_PXLW_IRIS)
	if (iris_is_chip_supported() && (!strcmp(panel->type, "secondary"))) {
		LCD_INFO("iris secondary panel no need config\n");
		return 0;
	}
#endif

	rc = utils->read_u32(utils->data, "oplus,dsi-bl-normal-max-level", &val);
	if (rc) {
		LCD_INFO("[%s] oplus,dsi-bl-normal-max-level undefined, default to bl max\n",
				panel->oplus_priv.vendor_name);
		panel->bl_config.bl_normal_max_level = panel->bl_config.bl_max_level;
	} else {
		panel->bl_config.bl_normal_max_level = val;
	}
	LCD_INFO("[%s] bl_max_level=%d\n", panel->oplus_priv.vendor_name,
			panel->bl_config.bl_max_level);

	rc = utils->read_u32(utils->data, "oplus,dsi-brightness-normal-max-level",
		&val);
	if (rc) {
		LCD_INFO("[%s] oplus,dsi-brightness-normal-max-level undefined, default to brightness max\n",
				panel->oplus_priv.vendor_name);
		panel->bl_config.brightness_normal_max_level = panel->bl_config.brightness_max_level;
	} else {
		panel->bl_config.brightness_normal_max_level = val;
	}
	LCD_INFO("[%s] brightness_normal_max_level=%d\n",
			panel->oplus_priv.vendor_name,
			panel->bl_config.brightness_normal_max_level);

	rc = utils->read_u32(utils->data, "oplus,dsi-brightness-default-level", &val);
	if (rc) {
		LCD_INFO("[%s] oplus,dsi-brightness-default-level undefined, default to brightness normal max\n",
				panel->oplus_priv.vendor_name);
		panel->bl_config.brightness_default_level = panel->bl_config.brightness_normal_max_level;
	} else {
		panel->bl_config.brightness_default_level = val;
	}
	LCD_INFO("[%s] brightness_default_level=%d\n",
			panel->oplus_priv.vendor_name,
			panel->bl_config.brightness_default_level);

	rc = utils->read_u32(utils->data, "oplus,dsi-dc-backlight-threshold", &val);
	if (rc) {
		LCD_INFO("[%s] oplus,dsi-dc-backlight-threshold undefined, default to 260\n",
				panel->oplus_priv.vendor_name);
		panel->bl_config.dc_backlight_threshold = 260;
		panel->bl_config.oplus_dc_mode = false;
	} else {
		panel->bl_config.dc_backlight_threshold = val;
		panel->bl_config.oplus_dc_mode = true;
	}
	LCD_INFO("[%s] dc_backlight_threshold=%d, oplus_dc_mode=%d\n",
			panel->oplus_priv.vendor_name,
			panel->bl_config.dc_backlight_threshold,
			panel->bl_config.oplus_dc_mode);

	rc = utils->read_u32(utils->data, "oplus,dsi-global-hbm-case-id", &val);
	if (rc) {
		LCD_INFO("[%s] oplus,dsi-global-hbm-case-id undefined, default to 0\n",
				panel->oplus_priv.vendor_name);
		val = GLOBAL_HBM_CASE_NONE;
	} else if (val >= GLOBAL_HBM_CASE_MAX) {
		LCD_ERR("[%s] oplus,dsi-global-hbm-case-id is invalid:%d\n",
				panel->oplus_priv.vendor_name, val);
		val = GLOBAL_HBM_CASE_NONE;
	}
	panel->bl_config.global_hbm_case_id = val;
	LCD_INFO("[%s] global_hbm_case_id=%d\n",
			panel->oplus_priv.vendor_name,
			panel->bl_config.global_hbm_case_id);

	rc = utils->read_u32(utils->data, "oplus,dsi-global-hbm-threshold", &val);
	if (rc) {
		LCD_INFO("[%s] oplus,dsi-global-hbm-threshold undefined, default to brightness normal max + 1\n",
				panel->oplus_priv.vendor_name);
		panel->bl_config.global_hbm_threshold = panel->bl_config.brightness_normal_max_level + 1;
	} else {
		panel->bl_config.global_hbm_threshold = val;
	}
	LCD_INFO("[%s] global_hbm_threshold=%d\n",
			panel->oplus_priv.vendor_name,
			panel->bl_config.global_hbm_threshold);

	panel->bl_config.global_hbm_scale_mapping = utils->read_bool(utils->data,
			"oplus,dsi-global-hbm-scale-mapping");
	LCD_INFO("oplus,dsi-global-hbm-scale-mapping: %s\n",
			panel->bl_config.global_hbm_scale_mapping ? "true" : "false");

	rc = utils->read_u32(utils->data, "oplus,pwm-switch-backlight-threshold", &val);
	if (rc) {
		panel->oplus_priv.pwm_switch_support = false;
	} else {
		panel->bl_config.pwm_bl_threshold = val;
	}
	panel->pwm_power_on = false;
	panel->pwm_hbm_state = false;
	LCD_INFO("[%s] oplus,pwm-switch-backlight-threshold=%d\n",
			panel->oplus_priv.vendor_name,
			panel->bl_config.pwm_bl_threshold);

	rc = utils->read_u32(utils->data, "oplus,pwm-onepulse-backlight-threshold", &val);
	if (rc) {
		panel->oplus_priv.pwm_onepulse_support = false;
	} else {
		panel->bl_config.pwm_bl_onepulse_threshold = val;
	}
	LCD_INFO("[%s] oplus,pwm-onepulse-backlight-threshold=%d\n",
			panel->oplus_priv.vendor_name,
			panel->bl_config.pwm_bl_onepulse_threshold);

	return 0;
}

static int oplus_display_panel_dly(struct dsi_panel *panel, bool hbm_switch)
{
	if (hbm_switch) {
		if (enable_hbm_enter_dly_on_flags)
			enable_hbm_enter_dly_on_flags++;
		if (0 == oplus_global_hbm_flags) {
			if (dsi_panel_tx_cmd_set(panel, DSI_CMD_DLY_ON)) {
				return 0;
			}
			enable_hbm_enter_dly_on_flags = 1;
		} else if (4 == enable_hbm_enter_dly_on_flags) {
			if (dsi_panel_tx_cmd_set(panel, DSI_CMD_DLY_OFF)) {
				return 0;
			}
			enable_hbm_enter_dly_on_flags = 0;
		}
	} else {
		if (oplus_global_hbm_flags == 1) {
			if (dsi_panel_tx_cmd_set(panel, DSI_CMD_DLY_ON)) {
				return 0;
			}
			enable_hbm_exit_dly_on_flags = 1;
		} else {
			if (enable_hbm_exit_dly_on_flags)
				enable_hbm_exit_dly_on_flags++;
			if (3 == enable_hbm_exit_dly_on_flags) {
				enable_hbm_exit_dly_on_flags = 0;
				if (dsi_panel_tx_cmd_set(panel, DSI_CMD_DLY_OFF)) {
					return 0;
				}
			}
		}
	}
	return 0;
}

int oplus_panel_global_hbm_mapping(struct dsi_panel *panel, u32 *backlight_level)
{
	int rc = 0;
	u32 bl_lvl = *backlight_level;
	u32 global_hbm_switch_cmd = 0;
	bool global_hbm_dly = false;
	struct dsi_cmd_desc *cmds;
	size_t tx_len;
	u8 *tx_buf;
	u32 count;
	int i;

	if (bl_lvl > panel->bl_config.bl_normal_max_level) {
		if (!oplus_global_hbm_flags) {
			global_hbm_switch_cmd = DSI_CMD_HBM_ENTER_SWITCH;
		}
	} else if (oplus_global_hbm_flags) {
		global_hbm_switch_cmd = DSI_CMD_HBM_EXIT_SWITCH;
	}

	switch (panel->bl_config.global_hbm_case_id) {
	case GLOBAL_HBM_CASE_1:
		break;
	case GLOBAL_HBM_CASE_2:
		if (bl_lvl > panel->bl_config.bl_normal_max_level) {
			if (panel->bl_config.global_hbm_scale_mapping) {
				bl_lvl = (bl_lvl - panel->bl_config.bl_normal_max_level) * 100000
						/ (panel->bl_config.bl_max_level - panel->bl_config.bl_normal_max_level)
						* (panel->bl_config.bl_max_level - panel->bl_config.global_hbm_threshold)
						/ 100000 + panel->bl_config.global_hbm_threshold;
			} else if (bl_lvl < panel->bl_config.global_hbm_threshold) {
				bl_lvl = panel->bl_config.global_hbm_threshold;
			}
		}
		break;
	case GLOBAL_HBM_CASE_3:
		if (bl_lvl > panel->bl_config.bl_normal_max_level) {
			bl_lvl = bl_lvl + panel->bl_config.global_hbm_threshold
					- panel->bl_config.bl_normal_max_level - 1;
		}
		break;
	case GLOBAL_HBM_CASE_4:
		global_hbm_switch_cmd = 0;
		if (bl_lvl <= PANEL_MAX_NOMAL_BRIGHTNESS) {
			if (oplus_global_hbm_flags) {
				global_hbm_switch_cmd = DSI_CMD_HBM_EXIT_SWITCH;
			}
			bl_lvl = backlight_buf[bl_lvl];
		} else if (bl_lvl > HBM_BASE_600NIT) {
			if (!oplus_global_hbm_flags) {
				global_hbm_switch_cmd = DSI_CMD_HBM_ENTER_SWITCH;
			}
			global_hbm_dly = true;
			bl_lvl = backlight_600_800nit_buf[bl_lvl - HBM_BASE_600NIT];
		} else if (bl_lvl > PANEL_MAX_NOMAL_BRIGHTNESS) {
			if (oplus_global_hbm_flags) {
				global_hbm_switch_cmd = DSI_CMD_HBM_EXIT_SWITCH;
			}
			bl_lvl = backlight_500_600nit_buf[bl_lvl - PANEL_MAX_NOMAL_BRIGHTNESS];
		}
		break;
	default:
		global_hbm_switch_cmd = 0;
		break;
	}

	if (oplus_panel_pwm_onepulse_is_enabled(panel) && strcmp(panel->oplus_priv.vendor_name, "TM_NT37705")) {
		if (bl_lvl > panel->bl_config.pwm_bl_threshold && bl_lvl < panel->bl_config.pwm_bl_onepulse_threshold) {
			bl_lvl = panel->bl_config.pwm_bl_threshold + 1;
		} else if (bl_lvl >= panel->bl_config.pwm_bl_onepulse_threshold && bl_lvl <= panel->bl_config.brightness_normal_max_level) {
			bl_lvl = backlight_onepulse_normal_buf[bl_lvl - panel->bl_config.pwm_bl_onepulse_threshold];
		} else if (bl_lvl > panel->bl_config.brightness_normal_max_level) {
			bl_lvl = backlight_onepulse_max_buf[bl_lvl - panel->bl_config.brightness_normal_max_level];
		}
	}

	bl_lvl = bl_lvl < panel->bl_config.bl_max_level ? bl_lvl :
			panel->bl_config.bl_max_level;

	if (global_hbm_switch_cmd > 0) {
		/* Update the 0x51 value when sending hbm enter/exit command */
		cmds = panel->cur_mode->priv_info->cmd_sets[global_hbm_switch_cmd].cmds;
		count = panel->cur_mode->priv_info->cmd_sets[global_hbm_switch_cmd].count;
		for (i = 0; i < count; i++) {
			tx_len = cmds[i].msg.tx_len;
			tx_buf = (u8 *)cmds[i].msg.tx_buf;
			if ((3 == tx_len) && (0x51 == tx_buf[0])) {
				tx_buf[1] = (bl_lvl >> 8) & 0xFF;
				tx_buf[2] = bl_lvl & 0xFF;
				break;
			}
		}

		if (global_hbm_dly) {
			oplus_display_panel_dly(panel, true);
		}

		rc = dsi_panel_tx_cmd_set(panel, global_hbm_switch_cmd);
		oplus_global_hbm_flags = (global_hbm_switch_cmd == DSI_CMD_HBM_ENTER_SWITCH);
	}

	*backlight_level = bl_lvl;
	return 0;
}

int oplus_display_panel_get_global_hbm_status(void)
{
	return oplus_global_hbm_flags;
}

void oplus_display_panel_set_global_hbm_status(int global_hbm_status)
{
	oplus_global_hbm_flags = global_hbm_status;
	LCD_INFO("set oplus_global_hbm_flags = %d\n", global_hbm_status);
}

int oplus_hbm_pwm_state(struct dsi_panel *panel, bool hbm_state)
{
	if (!panel) {
		LCD_ERR("Invalid panel params\n");
		return -EINVAL;
	}

	if (panel->oplus_priv.pwm_switch_support && hbm_state) {
		if (oplus_panel_pwm_onepulse_is_enabled(panel)) {
			oplus_panel_event_data_notifier_trigger(panel, DRM_PANEL_EVENT_PWM_TURBO, 1, true);
		} else {
			oplus_panel_event_data_notifier_trigger(panel, DRM_PANEL_EVENT_PWM_TURBO, !hbm_state, true);
		}
	}

	if (panel->oplus_priv.pwm_switch_support) {
		panel->pwm_hbm_state = hbm_state;

		if (!hbm_state) {
			panel->pwm_power_on = true;
		}
	}
	LCD_INFO("set oplus pwm_hbm_state = %d\n", hbm_state);
	return 0;
}

void oplus_pwm_disable_duty_set_work_handler(struct work_struct *work)
{
	int rc = 0;
	struct dsi_panel *panel = container_of(work, struct dsi_panel, oplus_pwm_disable_duty_set_work);
	unsigned int refresh_rate = panel->cur_mode->timing.refresh_rate;

	oplus_sde_early_wakeup(panel);
	oplus_wait_for_vsync(panel);
	if (refresh_rate == 60) {
		oplus_need_to_sync_te(panel);
	}

	mutex_lock(&panel->panel_lock);
	if (panel->power_mode != SDE_MODE_DPMS_ON || !panel->panel_initialized) {
		LCD_WARN("display panel in off status\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}
	usleep_range(120, 120);
	if (!panel->oplus_priv.pwm_switch_restore_support) {
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_DISABLE_PWM_BACKLIGHT_COMPENSATION);
	} else {
		rc = dsi_panel_tx_cmd_set(panel, pwm_switch_cmd_restore);
	}

	mutex_unlock(&panel->panel_lock);

	if (rc) {
		LCD_ERR("[%s:%d]failed to send DSI_CMD_DISABLE_PWM_BACKLIGHT_COMPENSATION cmds rc = %d\n", panel->name, rc);
	}

	return;
}

int oplus_panel_pwm_switch_backlight(struct dsi_panel *panel, u32 bl_lvl)
{
	int rc = 0;
	u32 pwm_switch_state_last = panel->oplus_pwm_switch_state;
	u32 pwm_switch_cmd = 0;
	int pulse = 0;

	if (!panel->oplus_priv.pwm_switch_support)
		return rc;

	if (panel->pwm_hbm_state) {
		LCD_INFO("panel pwm_hbm_state true disable pwm switch!\n");
		return rc;
	}

	if (bl_lvl == 0 || bl_lvl == 1)
		return rc;

	if (bl_lvl > panel->bl_config.pwm_bl_threshold) {
		panel->oplus_pwm_switch_state = PWM_SWITCH_HIGH_STATE;
		pwm_switch_cmd = DSI_CMD_PWM_SWITCH_HIGH;
		pwm_switch_cmd_restore = DSI_CMD_PWM_SWITCH_HIGH_RESTORE;
		if (panel->pwm_power_on || panel->post_power_on) {
			if (!strcmp(panel->name, "AA551 P 3 A0004 dsc cmd mode panel"))
				pwm_switch_cmd = DSI_CMD_POWER_ON_PWM_SWITCH_HIGH;
			else
				pwm_switch_cmd = DSI_CMD_TIMMING_PWM_SWITCH_HIGH;
		}
		pulse = 0;
	} else {
		panel->oplus_pwm_switch_state = PWM_SWITCH_LOW_STATE;
		pwm_switch_cmd = DSI_CMD_PWM_SWITCH_LOW;
		pwm_switch_cmd_restore = DSI_CMD_PWM_SWITCH_LOW_RESTORE;
		if (panel->pwm_power_on || panel->post_power_on) {
			if (!strcmp(panel->name, "AA551 P 3 A0004 dsc cmd mode panel"))
				pwm_switch_cmd = DSI_CMD_POWER_ON_PWM_SWITCH_LOW;
			else
				pwm_switch_cmd = DSI_CMD_TIMMING_PWM_SWITCH_LOW;
		}
		pulse = 1;
	}
	if (oplus_panel_pwm_onepulse_is_enabled(panel)) {
		pulse = 1;
	}
	if (!strcmp(panel->name, "AC052 P 3 A0003 dsc cmd mode panel")
		|| !strcmp(panel->name, "AC052 S 3 A0001 dsc cmd mode panel")
		|| !strcmp(panel->name, "AA551 P 3 A0004 dsc cmd mode panel")
		|| !strcmp(panel->name, "AA536 P 3 A0001 dsc cmd mode panel")
		|| !strcmp(panel->name, "zonda tm nt37705 dsc cmd mode panel")) {
		oplus_panel_pwm_switch_wait_te_tx_cmd(panel, pwm_switch_cmd, pwm_switch_state_last);
	} else {
		if (pwm_switch_state_last != panel->oplus_pwm_switch_state ||
			panel->post_power_on || panel->pwm_power_on) {
			panel->pwm_power_on = false;
			panel->post_power_on = false;
			rc = dsi_panel_tx_cmd_set(panel, pwm_switch_cmd);
		}
	}
	oplus_panel_event_data_notifier_trigger(panel, DRM_PANEL_EVENT_PWM_TURBO, pulse, true);
	return 0;
}

int oplus_panel_pwm_switch_timing_switch(struct dsi_panel *panel)
{
	int rc = 0;
	u32 pwm_switch_cmd = DSI_CMD_TIMMING_PWM_SWITCH_LOW;

	if (!panel->oplus_priv.pwm_switch_support)
		return rc;

	if (panel->pwm_hbm_state) {
		LCD_INFO("panel pwm_hbm_state true disable pwm switch!\n");
		return rc;
	}


	if (panel->oplus_pwm_switch_state  == PWM_SWITCH_HIGH_STATE) {
		pwm_switch_cmd = DSI_CMD_TIMMING_PWM_SWITCH_HIGH;
	}

	rc = dsi_panel_tx_cmd_set(panel, pwm_switch_cmd);

	return rc;
}
int oplus_panel_pwm_switch_wait_te_tx_cmd(struct dsi_panel *panel, u32 pwm_switch_cmd, u32 pwm_switch_state_last)
{
	int rc = 0;
	unsigned int refresh_rate = panel->cur_mode->timing.refresh_rate;

	if (panel->pwm_power_on == true || panel->post_power_on) {
		panel->pwm_power_on = false;
		panel->post_power_on = false;
		rc = dsi_panel_tx_cmd_set(panel, pwm_switch_cmd);
		return rc;
	}

	if (pwm_switch_state_last != panel->oplus_pwm_switch_state) {
		oplus_sde_early_wakeup(panel);
		oplus_wait_for_vsync(panel);
		if (refresh_rate == 60) {
			oplus_need_to_sync_te(panel);
		}
		usleep_range(120, 120);

		if (oplus_panel_pwm_onepulse_is_enabled(panel)) {
			if (refresh_rate == 90) {
				oplus_need_to_sync_te(panel);
			}
		}

		rc = dsi_panel_tx_cmd_set(panel, pwm_switch_cmd);

		if (panel->oplus_priv.pwm_create_thread) {
			queue_work(panel->oplus_pwm_disable_duty_set_wq, &panel->oplus_pwm_disable_duty_set_work);
		}
	}
	return rc;
}

void oplus_panel_backlight_demura_dbv_switch(struct dsi_panel *panel, u32 bl_lvl)
{
	int rc = 0;
	u32 bl_demura_last_mode = panel->oplus_priv.bl_demura_mode;
	u32 bl_demura_mode = DSI_CMD_DEMURA_DBV_MODE0;

	if (!panel->oplus_priv.oplus_bl_demura_dbv_support)
		return;

	if (bl_lvl == 0 || bl_lvl == 1)
		return;

	if (bl_lvl < 688) {
		panel->oplus_priv.bl_demura_mode = 0;
		bl_demura_mode = DSI_CMD_DEMURA_DBV_MODE0;
	} else if (688 <= bl_lvl && bl_lvl < 1292) {
		panel->oplus_priv.bl_demura_mode = 1;
		bl_demura_mode = DSI_CMD_DEMURA_DBV_MODE1;
	} else if (1292 <= bl_lvl) {
		panel->oplus_priv.bl_demura_mode = 2;
		bl_demura_mode = DSI_CMD_DEMURA_DBV_MODE2;
	}

	if (panel->oplus_priv.bl_demura_mode != bl_demura_last_mode && panel->power_mode == SDE_MODE_DPMS_ON)
		rc = dsi_panel_tx_cmd_set(panel, bl_demura_mode);
	if (rc) {
		DSI_ERR("[%s] failed to send bl_demura_mode, rc=%d\n", panel->name, rc);
		return;
	}
}


static int backlight_onepulse_normal_buf[] = {
	1329, 1330, 1331, 1332, 1333, 1334, 1335, 1336, 1337, 1338, 1339, 1340, 1341, 1342, 1343, 1344, 1345, 1346, 1347, 1348, \
	1349, 1350, 1351, 1352, 1353, 1354, 1355, 1356, 1357, 1358, 1359, 1360, 1361, 1362, 1363, 1364, 1365, 1366, 1367, 1368, \
	1369, 1370, 1371, 1372, 1373, 1374, 1375, 1376, 1377, 1378, 1379, 1380, 1381, 1382, 1383, 1384, 1385, 1386, 1387, 1388, \
	1389, 1390, 1391, 1392, 1393, 1394, 1395, 1396, 1397, 1397, 1397, 1397, 1398, 1399, 1400, 1401, 1402, 1403, 1404, 1405, \
	1406, 1407, 1408, 1409, 1410, 1411, 1412, 1413, 1414, 1415, 1416, 1417, 1418, 1419, 1420, 1421, 1422, 1423, 1424, 1425, \
	1426, 1427, 1428, 1429, 1430, 1431, 1432, 1433, 1434, 1435, 1436, 1437, 1438, 1439, 1440, 1441, 1442, 1443, 1444, 1445, \
	1446, 1447, 1448, 1449, 1450, 1451, 1452, 1453, 1454, 1455, 1456, 1457, 1458, 1459, 1460, 1461, 1462, 1463, 1464, 1465, \
	1466, 1467, 1468, 1469, 1470, 1471, 1472, 1473, 1474, 1475, 1476, 1477, 1478, 1478, 1479, 1480, 1481, 1482, 1483, 1484, \
	1485, 1486, 1487, 1488, 1489, 1490, 1491, 1492, 1493, 1494, 1495, 1496, 1497, 1498, 1499, 1500, 1501, 1502, 1502, 1502, \
	1503, 1504, 1505, 1506, 1507, 1508, 1509, 1510, 1511, 1512, 1513, 1514, 1515, 1516, 1517, 1518, 1519, 1520, 1521, 1522, \
	1523, 1524, 1525, 1526, 1527, 1528, 1529, 1530, 1531, 1532, 1533, 1534, 1535, 1536, 1537, 1538, 1538, 1538, 1538, 1539, \
	1540, 1541, 1542, 1543, 1544, 1545, 1546, 1547, 1548, 1549, 1550, 1551, 1552, 1553, 1554, 1555, 1556, 1557, 1558, 1559, \
	1560, 1561, 1562, 1563, 1564, 1565, 1566, 1567, 1568, 1569, 1570, 1571, 1572, 1573, 1574, 1575, 1576, 1577, 1578, 1579, \
	1580, 1581, 1582, 1583, 1584, 1585, 1586, 1587, 1588, 1589, 1590, 1591, 1592, 1593, 1594, 1595, 1596, 1597, 1598, 1599, \
	1600, 1601, 1602, 1603, 1604, 1605, 1606, 1607, 1608, 1609, 1610, 1611, 1612, 1613, 1614, 1615, 1616, 1617, 1618, 1619, \
	1620, 1621, 1622, 1623, 1624, 1625, 1626, 1627, 1628, 1629, 1630, 1631, 1632, 1633, 1634, 1634, 1634, 1635, 1636, 1637, \
	1638, 1639, 1640, 1641, 1642, 1643, 1644, 1645, 1646, 1647, 1648, 1649, 1650, 1651, 1652, 1653, 1654, 1655, 1656, 1657, \
	1658, 1659, 1660, 1661, 1662, 1663, 1664, 1665, 1666, 1667, 1668, 1669, 1670, 1671, 1672, 1673, 1674, 1675, 1676, 1677, \
	1678, 1679, 1680, 1681, 1682, 1683, 1684, 1685, 1686, 1687, 1688, 1689, 1690, 1691, 1692, 1693, 1694, 1695, 1696, 1697, \
	1698, 1699, 1700, 1701, 1702, 1703, 1704, 1705, 1706, 1707, 1708, 1709, 1710, 1711, 1712, 1713, 1714, 1715, 1716, 1717, \
	1718, 1719, 1720, 1721, 1722, 1723, 1724, 1725, 1726, 1727, 1728, 1729, 1730, 1731, 1732, 1733, 1734, 1735, 1736, 1737, \
	1738, 1739, 1740, 1741, 1742, 1743, 1744, 1745, 1746, 1747, 1748, 1749, 1750, 1751, 1752, 1753, 1754, 1754, 1755, 1756, \
	1757, 1758, 1759, 1760, 1761, 1762, 1763, 1764, 1765, 1766, 1767, 1768, 1769, 1770, 1771, 1772, 1773, 1774, 1775, 1776, \
	1777, 1778, 1779, 1780, 1781, 1782, 1783, 1783, 1784, 1785, 1786, 1787, 1788, 1789, 1790, 1791, 1792, 1793, 1794, 1795, \
	1796, 1797, 1798, 1799, 1800, 1801, 1802, 1803, 1804, 1805, 1806, 1807, 1808, 1809, 1810, 1811, 1811, 1811, 1812, 1813, \
	1814, 1815, 1816, 1817, 1818, 1819, 1820, 1821, 1822, 1823, 1824, 1825, 1826, 1827, 1828, 1829, 1830, 1831, 1832, 1832, \
	1833, 1834, 1835, 1836, 1837, 1838, 1839, 1840, 1841, 1842, 1843, 1844, 1845, 1846, 1847, 1848, 1849, 1850, 1851, 1852, \
	1853, 1854, 1855, 1856, 1857, 1858, 1859, 1860, 1861, 1862, 1863, 1864, 1865, 1866, 1867, 1868, 1869, 1870, 1871, 1872, \
	1873, 1874, 1875, 1875, 1876, 1877, 1878, 1879, 1880, 1881, 1882, 1883, 1884, 1885, 1886, 1887, 1888, 1889, 1890, 1891, \
	1892, 1893, 1894, 1895, 1896, 1897, 1898, 1899, 1900, 1901, 1902, 1903, 1904, 1905, 1906, 1907, 1908, 1909, 1910, 1911, \
	1912, 1913, 1914, 1915, 1916, 1917, 1918, 1919, 1920, 1921, 1922, 1923, 1924, 1925, 1926, 1927, 1928, 1929, 1930, 1931, \
	1932, 1933, 1934, 1935, 1936, 1937, 1938, 1939, 1940, 1941, 1942, 1943, 1944, 1945, 1946, 1947, 1948, 1949, 1950, 1951, \
	1952, 1953, 1954, 1955, 1956, 1957, 1958, 1959, 1960, 1961, 1962, 1963, 1964, 1965, 1966, 1967, 1968, 1969, 1970, 1971, \
	1972, 1973, 1974, 1975, 1976, 1977, 1977, 1978, 1979, 1980, 1981, 1982, 1983, 1984, 1985, 1986, 1987, 1988, 1989, 1990, \
	1991, 1992, 1993, 1994, 1995, 1996, 1997, 1998, 1999, 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009, 2010, \
	2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020, 2021, 2022, 2023, 2024, 2025, 2026, 2027, 2028, 2029, 2030, \
	2031, 2032, 2033, 2034, 2035, 2036, 2037, 2038, 2039, 2040, 2041, 2042, 2043, 2044, 2045, 2046, 2047, 2048, 2048, 2049, \
	2050, 2051, 2052, 2053, 2054, 2055, 2056, 2057, 2058, 2059, 2060, 2061, 2062, 2063, 2064, 2065, 2066, 2067, 2068, 2069, \
	2070, 2071, 2072, 2073, 2074, 2075, 2076, 2077, 2078, 2079, 2080, 2081, 2082, 2083, 2084, 2085, 2086, 2087, 2088, 2089, \
	2090, 2091, 2092, 2093, 2094, 2095, 2096, 2097, 2098, 2099, 2100, 2101, 2102, 2103, 2104, 2105, 2106, 2107, 2108, 2109, \
	2110, 2111, 2112, 2113, 2114, 2115, 2116, 2117, 2118, 2118, 2119, 2120, 2121, 2122, 2123, 2124, 2125, 2126, 2127, 2128, \
	2129, 2130, 2131, 2132, 2133, 2133, 2134, 2135, 2136, 2137, 2138, 2139, 2140, 2141, 2142, 2143, 2144, 2145, 2146, 2147, \
	2148, 2149, 2150, 2151, 2152, 2153, 2154, 2155, 2156, 2157, 2158, 2159, 2160, 2161, 2162, 2163, 2164, 2165, 2166, 2167, \
	2168, 2169, 2170, 2171, 2172, 2173, 2174, 2175, 2176, 2177, 2178, 2179, 2180, 2181, 2182, 2183, 2184, 2185, 2186, 2187, \
	2187, 2188, 2189, 2190, 2191, 2192, 2193, 2194, 2195, 2196, 2197, 2198, 2199, 2200, 2201, 2202, 2202, 2202, 2203, 2204, \
	2205, 2206, 2207, 2208, 2209, 2210, 2211, 2212, 2213, 2214, 2215, 2216, 2217, 2218, 2219, 2220, 2221, 2222, 2223, 2224, \
	2225, 2226, 2227, 2228, 2229, 2230, 2231, 2232, 2233, 2234, 2235, 2236, 2237, 2238, 2239, 2240, 2241, 2242, 2243, 2244, \
	2245, 2246, 2247, 2248, 2249, 2250, 2251, 2252, 2253, 2254, 2255, 2256, 2257, 2258, 2259, 2260, 2261, 2261, 2262, 2263, \
	2264, 2265, 2266, 2267, 2268, 2269, 2270, 2271, 2272, 2273, 2274, 2275, 2276, 2277, 2278, 2279, 2280, 2281, 2282, 2283, \
	2283, 2283, 2284, 2285, 2286, 2287, 2288, 2289, 2290, 2291, 2292, 2293, 2294, 2295, 2296, 2297, 2298, 2299, 2300, 2301, \
	2301, 2302, 2303, 2304, 2305, 2306, 2307, 2308, 2309, 2310, 2311, 2312, 2313, 2314, 2315, 2316, 2317, 2318, 2319, 2320, \
	2321, 2322, 2323, 2324, 2325, 2326, 2327, 2328, 2329, 2330, 2331, 2332, 2333, 2334, 2335, 2336, 2337, 2338, 2338, 2339, \
	2340, 2341, 2342, 2343, 2344, 2345, 2346, 2347, 2348, 2349, 2350, 2351, 2352, 2353, 2354, 2355, 2356, 2357, 2358, 2359, \
	2360, 2361, 2362, 2363, 2364, 2365, 2366, 2367, 2368, 2369, 2370, 2371, 2372, 2373, 2374, 2375, 2376, 2377, 2378, 2379, \
	2380, 2381, 2382, 2383, 2384, 2385, 2385, 2386, 2387, 2388, 2389, 2390, 2391, 2392, 2393, 2394, 2395, 2396, 2397, 2398, \
	2399, 2400, 2401, 2402, 2403, 2404, 2405, 2406, 2407, 2408, 2409, 2410, 2411, 2412, 2413, 2414, 2415, 2416, 2417, 2418, \
	2419, 2420, 2421, 2422, 2423, 2424, 2425, 2426, 2427, 2428, 2429, 2429, 2430, 2431, 2431, 2432, 2433, 2434, 2435, 2436, \
	2437, 2438, 2439, 2440, 2441, 2442, 2443, 2444, 2445, 2446, 2447, 2448, 2448, 2449, 2450, 2451, 2452, 2453, 2454, 2455, \
	2456, 2457, 2458, 2459, 2460, 2461, 2462, 2463, 2464, 2465, 2466, 2467, 2468, 2469, 2469, 2469, 2469, 2470, 2471, 2472, \
	2473, 2474, 2475, 2475, 2475, 2475, 2476, 2477, 2478, 2479, 2480, 2481, 2482, 2483, 2484, 2485, 2486, 2487, 2488, 2489, \
	2490, 2491, 2492, 2493, 2493, 2493, 2493, 2493, 2493, 2493, 2494, 2495, 2496, 2497, 2498, 2499, 2500, 2501, 2502, 2503, \
	2504, 2505, 2506, 2507, 2508, 2509, 2510, 2511, 2512, 2513, 2514, 2514, 2515, 2516, 2517, 2518, 2519, 2520, 2520, 2520, \
	2521, 2522, 2523, 2524, 2525, 2526, 2527, 2528, 2529, 2530, 2531, 2532, 2532, 2532, 2532, 2533, 2534, 2535, 2536, 2537, \
	2538, 2539, 2540, 2541, 2542, 2543, 2544, 2545, 2546, 2547, 2548, 2549, 2549, 2549, 2549, 2550, 2551, 2552, 2553, 2554, \
	2555, 2556, 2557, 2558, 2559, 2560, 2561, 2561, 2561, 2562, 2563, 2564, 2565, 2566, 2567, 2568, 2569, 2570, 2571, 2572, \
	2573, 2574, 2575, 2575, 2576, 2577, 2578, 2579, 2580, 2580, 2581, 2582, 2583, 2584, 2585, 2586, 2587, 2588, 2589, 2590, \
	2591, 2592, 2593, 2594, 2595, 2596, 2597, 2598, 2598, 2599, 2600, 2601, 2602, 2603, 2604, 2605, 2606, 2607, 2608, 2609, \
	2610, 2611, 2612, 2613, 2614, 2615, 2616, 2616, 2616, 2617, 2618, 2619, 2620, 2621, 2622, 2623, 2624, 2625, 2626, 2627, \
	2628, 2629, 2630, 2631, 2632, 2633, 2634, 2635, 2636, 2637, 2638, 2639, 2640, 2641, 2642, 2643, 2644, 2645, 2646, 2647, \
	2648, 2649, 2650, 2651, 2652, 2653, 2654, 2655, 2656, 2657, 2658, 2659, 2660, 2661, 2662, 2663, 2664, 2665, 2666, 2667, \
	2668, 2669, 2670, 2671, 2672, 2673, 2674, 2675, 2676, 2676, 2677, 2678, 2679, 2680, 2681, 2682, 2683, 2684, 2685, 2686, \
	2687, 2688, 2689, 2690, 2691, 2692, 2693, 2694, 2695, 2696, 2697, 2698, 2699, 2700, 2701, 2702, 2703, 2704, 2705, 2706, \
	2707, 2708, 2708, 2708, 2709, 2710, 2711, 2712, 2713, 2714, 2715, 2716, 2717, 2718, 2719, 2720, 2721, 2722, 2723, 2724, \
	2725, 2726, 2727, 2728, 2729, 2730, 2731, 2732, 2733, 2734, 2735, 2736, 2737, 2738, 2739, 2740, 2741, 2742, 2743, 2744, \
	2745, 2746, 2747, 2748, 2749, 2750, 2751, 2752, 2753, 2754, 2755, 2756, 2757, 2758, 2759, 2760, 2761, 2762, 2763, 2764, \
	2765, 2766, 2767, 2768, 2769, 2770, 2771, 2772, 2773, 2774, 2775, 2776, 2777, 2778, 2779, 2780, 2781, 2782, 2783, 2784, \
	2785, 2786, 2787, 2788, 2789, 2790, 2791, 2792, 2793, 2794, 2795, 2796, 2797, 2798, 2799, 2800, 2801, 2802, 2803, 2804, \
	2805, 2806, 2807, 2808, 2809, 2810, 2811, 2812, 2813, 2814, 2815, 2816, 2817, 2818, 2819, 2820, 2821, 2822, 2823, 2824, \
	2825, 2826, 2827, 2828, 2829, 2830, 2831, 2832, 2833, 2834, 2835, 2836, 2837, 2838, 2839, 2840, 2841, 2842, 2843, 2844, \
	2845, 2846, 2847, 2848, 2849, 2850, 2851, 2852, 2853, 2854, 2855, 2856, 2857, 2858, 2859, 2860, 2861, 2862, 2863, 2864, \
	2865, 2865, 2865, 2866, 2867, 2868, 2869, 2870, 2870, 2871, 2872, 2873, 2874, 2875, 2876, 2877, 2877, 2878, 2879, 2880, \
	2881, 2882, 2883, 2884, 2885, 2886, 2887, 2888, 2889, 2890, 2891, 2892, 2893, 2894, 2895, 2896, 2897, 2898, 2899, 2900, \
	2901, 2901, 2902, 2903, 2904, 2905, 2906, 2906, 2906, 2907, 2908, 2909, 2910, 2911, 2912, 2913, 2914, 2915, 2916, 2917, \
	2918, 2919, 2920, 2921, 2922, 2923, 2924, 2925, 2926, 2927, 2928, 2929, 2930, 2931, 2932, 2933, 2934, 2935, 2936, 2937, \
	2938, 2939, 2940, 2941, 2942, 2943, 2944, 2945, 2946, 2947, 2948, 2949, 2950, 2951, 2952, 2953, 2954, 2955, 2956, 2957, \
	2958, 2959, 2960, 2961, 2962, 2963, 2964, 2965, 2966, 2967, 2968, 2969, 2970, 2971, 2972, 2973, 2974, 2975, 2976, 2977, \
	2978, 2979, 2980, 2981, 2982, 2983, 2984, 2985, 2986, 2987, 2988, 2989, 2989, 2989, 2990, 2991, 2992, 2993, 2994, 2995, \
	2996, 2997, 2998, 2999, 3000, 3001, 3002, 3003, 3004, 3005, 3006, 3007, 3008, 3009, 3010, 3011, 3012, 3013, 3014, 3015, \
	3016, 3017, 3018, 3019, 3020, 3021, 3022, 3023, 3024, 3025, 3026, 3027, 3028, 3029, 3030, 3031, 3032, 3033, 3034, 3035, \
	3036, 3037, 3038, 3039, 3040, 3041, 3042, 3043, 3044, 3045, 3046, 3047, 3048, 3049, 3050, 3051, 3052, 3053, 3054, 3055, \
	3056, 3057, 3058, 3059, 3060, 3061, 3062, 3063, 3064, 3065, 3066, 3067, 3068, 3069, 3070, 3071, 3072, 3073, 3074, 3075, \
	3076, 3077, 3078, 3079, 3080, 3081, 3082, 3083, 3084, 3085, 3086, 3087, 3088, 3089, 3090, 3091, 3092, 3093, 3094, 3095, \
	3096, 3097, 3098, 3099, 3100, 3101, 3102, 3103, 3104, 3105, 3106, 3107, 3108, 3109, 3110, 3111, 3112, 3113, 3114, 3115, \
	3116, 3117, 3118, 3119, 3120, 3121, 3122, 3123, 3124, 3125, 3126, 3127, 3128, 3129, 3130, 3131, 3132, 3133, 3134, 3135, \
	3136, 3137, 3138, 3139, 3140, 3141, 3142, 3143, 3144, 3145, 3146, 3147, 3148, 3149, 3150, 3151, 3152, 3153, 3154, 3155, \
	3156, 3157, 3158, 3159, 3160, 3161, 3162, 3163, 3164, 3165, 3166, 3167, 3168, 3169, 3170, 3171, 3172, 3173, 3174, 3175, \
	3176, 3177, 3178, 3179, 3180, 3181, 3182, 3183, 3183, 3184, 3185, 3186, 3187, 3188, 3189, 3190, 3191, 3192, 3193, 3194, \
	3195, 3196, 3197, 3198, 3199
};

static int backlight_onepulse_max_buf[] = {
	3200, 3200, 3200, 3201, 3202, 3204, 3205, 3206, 3207, 3209, 3210, 3211, 3212, 3213, \
	3213, 3213, 3213, 3214, 3215, 3216, 3217, 3218, 3219, 3220, 3221, 3221, 3221, 3221, 3225, 3228, 3229, 3230, 3230, 3230, \
	3231, 3234, 3235, 3236, 3237, 3238, 3239, 3240, 3241, 3242, 3242, 3242, 3246, 3246, 3246, 3246, 3246, 3247, 3248, 3249, \
	3250, 3252, 3252, 3252, 3253, 3254, 3255, 3256, 3257, 3258, 3259, 3260, 3261, 3262, 3263, 3264, 3265, 3266, 3267, 3268, \
	3269, 3270, 3271, 3272, 3273, 3274, 3275, 3276, 3277, 3278, 3279, 3280, 3281, 3282, 3283, 3284, 3285, 3286, 3287, 3288, \
	3289, 3290, 3291, 3292, 3293, 3294, 3294, 3295, 3296, 3297, 3298, 3299, 3300, 3301, 3302, 3303, 3304, 3305, 3306, 3307, \
	3308, 3309, 3310, 3312, 3313, 3314, 3315, 3316, 3317, 3318, 3319, 3320, 3322, 3323, 3324, 3327, 3328, 3330, 3333, 3334, \
	3338, 3339, 3340, 3341, 3344, 3345, 3346, 3347, 3350, 3354, 3359, 3360, 3361, 3364, 3365, 3366, 3367, 3370, 3371, 3372, \
	3375, 3378, 3379, 3380, 3383, 3386, 3389, 3390, 3391, 3392, 3393, 3398, 3400, 3401, 3405, 3408, 3411, 3412, 3413, 3418, \
	3419, 3425, 3428, 3429, 3430, 3431, 3432, 3434, 3436, 3437, 3438, 3441, 3445, 3446, 3447, 3448, 3449, 3450, 3453, 3456, \
	3460, 3461, 3462, 3463, 3467, 3469, 3470, 3472, 3476, 3477, 3479, 3482, 3483, 3489, 3490, 3492, 3493, 3494, 3495, 3496, \
	3497, 3500, 3503, 3504, 3505, 3510, 3511, 3512, 3513, 3514, 3515, 3516, 3517, 3518, 3519, 3520, 3521, 3522, 3523, 3525, \
	3526, 3527, 3528, 3529, 3530, 3531, 3532, 3533, 3534, 3535, 3536, 3537, 3538, 3539, 3540, 3541, 3542, 3543, 3544, 3545, \
	3546, 3547, 3547, 3548, 3548, 3549, 3549, 3550, 3551, 3552, 3553, 3555, 3558, 3559, 3560, 3561, 3562, 3563, 3564, 3565, \
	3566, 3567, 3568, 3569, 3570, 3571, 3572, 3573, 3574, 3575, 3576, 3577, 3578, 3579, 3579, 3580, 3581, 3582, 3583, 3584, \
	3585, 3586, 3587, 3588, 3589, 3590, 3591, 3592, 3593, 3594, 3595, 3596, 3597, 3598, 3599, 3600, 3601, 3602, 3603, 3604, \
	3605, 3606, 3607, 3608, 3609, 3610, 3611, 3611, 3612, 3613, 3614, 3614, 3615, 3616, 3617, 3618, 3619, 3621, 3622, 3623, \
	3624, 3625, 3626, 3627, 3628, 3629, 3630, 3631, 3632, 3633, 3634, 3635, 3636, 3637, 3638, 3639, 3640, 3641, 3641, 3641, \
	3642, 3643, 3645, 3646, 3647, 3648, 3649, 3650, 3651, 3652, 3653, 3654, 3655, 3656, 3657, 3658, 3659, 3660, 3661, 3662, \
	3663, 3664, 3665, 3666, 3667, 3668, 3669, 3670, 3671, 3672, 3673, 3674, 3675, 3676, 3677, 3678, 3679, 3680, 3681, 3682, \
	3683, 3684, 3685, 3686, 3687, 3688, 3688, 3688, 3689, 3690, 3691, 3692, 3693, 3694, 3695, 3696, 3698, 3699, 3700, 3701, \
	3702, 3703, 3703, 3703, 3704, 3705, 3706, 3707, 3708, 3709, 3710, 3711, 3712, 3713, 3719, 3720, 3721, 3722, 3723, 3723, \
	3724, 3725, 3726, 3727, 3728, 3729, 3730, 3732, 3733, 3734, 3735, 3736, 3736, 3737, 3738, 3739, 3739, 3740, 3741, 3742, \
	3743, 3744, 3745, 3746, 3747, 3748, 3749, 3750, 3751, 3752, 3753, 3754, 3755, 3756, 3757, 3758, 3759, 3760, 3761, 3762, \
	3763, 3764, 3765, 3766, 3767, 3768, 3769, 3770, 3771, 3772, 3773, 3774, 3775, 3776, 3777, 3778, 3779, 3780, 3781, 3782, \
	3783, 3784, 3785, 3786, 3787, 3788, 3789, 3790, 3791, 3792, 3793, 3794, 3795, 3796, 3797, 3798, 3799, 3800, 3800, 3800, \
	3802, 3802, 3802, 3802, 3802, 3803, 3803, 3804, 3805, 3806, 3807, 3808, 3809, 3810, 3811, 3812, 3813, 3815, 3816, 3816, \
	3816, 3817, 3818, 3819, 3820, 3821, 3822, 3823, 3824, 3825, 3826, 3827, 3828, 3829, 3830, 3831, 3832, 3833, 3834, 3835, \
	3836, 3836, 3836, 3837, 3838, 3839, 3840, 3842, 3843, 3844, 3845, 3846, 3847, 3848, 3849, 3850, 3851, 3852, 3853, 3854, \
	3855, 3857, 3858, 3859, 3860, 3860, 3860, 3862, 3864, 3865, 3866, 3867, 3868, 3869, 3870, 3871, 3872, 3873, 3875, 3876, \
	3877, 3878, 3879, 3880, 3880, 3880, 3881, 3882, 3882, 3883, 3884, 3885, 3886, 3887, 3888, 3889, 3889, 3890, 3891, 3892, \
	3893, 3894, 3895, 3896, 3897, 3898, 3899, 3900, 3901, 3902, 3903, 3904, 3905, 3906, 3908, 3909, 3910, 3911, 3912, 3913, \
	3914, 3916, 3917, 3918, 3920, 3921, 3922, 3923, 3924, 3926, 3927, 3927, 3928, 3930, 3931, 3932, 3933, 3934, 3935, 3936, \
	3937, 3938, 3940, 3941, 3942, 3943, 3943, 3943, 3944, 3945, 3946, 3946, 3947, 3948, 3949, 3950, 3951, 3952, 3953, 3954, \
	3955, 3956, 3957, 3958, 3959, 3960, 3960, 3961, 3962, 3963, 3964, 3965, 3966, 3967, 3968, 3969, 3970, 3971, 3972, 3973, \
	3974, 3975, 3976, 3977, 3978, 3979, 3980, 3981, 3981, 3981, 3981, 3981, 3981, 3981, 3981, 3982, 3982, 3982, 3983, 3983, \
	3983, 3983, 3983, 3983, 3983, 3983, 3983
};
