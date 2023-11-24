// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include "goodix_brl_core.h"

static u8 brl_pen_set_cmd_map[PEN_DOWN_CMD_MAX] = {
	GTP_PEN_SET_CMD_ID,
	GTP_PEN_SET_CMD_CON_STA,
	GTP_PEN_SET_CMD_SIZE,
	GTP_PEN_SET_CMD_REQ_CFG,
	GTP_PEN_SET_CMD_CFG_ACK,
	GTP_PEN_SET_CMD_CFG_ACK,
	GTP_PEN_SET_CMD_PRESS,
	GTP_PEN_SET_CMD_SPEED_ON,
	GTP_PEN_DOWN_CMD_FRQ,
};

/* pencil healthinfo start */
static void pencil_data_record(struct chip_data_brl *chip_info, int type)
{
	switch (type) {
	case CODING:
		if (chip_info->pen_err_coding_cnt >= PENCIL_DATA_MAX)
			chip_info->pen_err_coding_cnt = 0;
		chip_info->pen_err_coding[chip_info->pen_err_coding_cnt] = chip_info->checkdata[CODING];
		chip_info->pen_err_coding_cnt++;
		break;
	case ADC:
		if (chip_info->pen_err_adc_cnt >= PENCIL_DATA_MAX)
			chip_info->pen_err_adc_cnt = 0;
		chip_info->pen_err_adc[chip_info->pen_err_adc_cnt] = chip_info->checkdata[ADC];
		chip_info->pen_err_adc_cnt++;
		break;
	case BASE:
		if (chip_info->pen_err_base_cnt >= PENCIL_DATA_MAX)
			chip_info->pen_err_base_cnt = 0;
		chip_info->pen_err_base[chip_info->pen_err_base_cnt] = chip_info->checkdata[BASE];
		chip_info->pen_err_base_cnt++;
		break;
	default:
		break;
	}
}

static void pencil_data_operation(struct chip_data_brl *chip_info)
{
	int cnt = 0;
	int cnt_max = 0;
	bool pen_base = false;
	u16 match_map[] = {0x9B0, 0X1B58, 0X28F0, 0X6F54, 0X733C, 0X751C, 0X790};
	cnt_max = sizeof(match_map) / 2;

	for (cnt = 0; cnt < cnt_max; cnt++)
		if (chip_info->checkdata[CODING] == match_map[cnt])
			pen_base = true;

	if (pen_base == false)
		pencil_data_record(chip_info, CODING);

	if (chip_info->checkdata[ADC] < PENCIL_LIMIT_MIN || chip_info->checkdata[ADC] > PENCIL_LIMIT_MAX)
		pencil_data_record(chip_info, ADC);

	if (chip_info->checkdata[BASE] < PENCIL_LIMIT_MIN || chip_info->checkdata[BASE] > PENCIL_LIMIT_MAX)
		pencil_data_record(chip_info, BASE);

	chip_info->pen_err_press = chip_info->checkdata[PRESS];
}

static int pencil_parse_data(struct chip_data_brl *chip_info, char *buf, int cnt)
{
	char tmp[PENCIL_DATA_OFFECT + 1] = {0};
	u16 uch_value = 0;

	if (cnt >= PENCIL_DATA_GROUP) {
		TPD_INFO("GT_brlD:%s: cnt:%d over size!!\n", __func__, cnt);
		goto ERR;
	}

	strncpy(tmp, buf + (PENCIL_DATA_OFFECT * cnt), PENCIL_DATA_OFFECT);
	tmp[PENCIL_DATA_OFFECT] = PENCIL_DATA_LAST;

	if (kstrtou16(tmp, PENCIL_STR_TO_HEX, &uch_value)) {
		TPD_INFO("GT_brlD:%s: kstrtoint error\n", __func__);
		goto ERR;
	}

	chip_info->checkdata[cnt] = uch_value;
	TPD_DEBUG("GT_brlD:%s:tmp[%s]checkdata[%d]hex[0x%4x]dec[%hu]\n", __func__,
		tmp, cnt, chip_info->checkdata[cnt], chip_info->checkdata[cnt]);

	return 0;
ERR:
	return -1;
}

static int pencil_data_handle(struct chip_data_brl *chip_info, char *buf, int addr_tag)
{
	int cnt = 0;

	switch (addr_tag) {
	case P_ADDR2:
		for (cnt = 0; cnt < PENCIL_DATA_GROUP; cnt++)
			if (pencil_parse_data(chip_info, buf, cnt) < 0)
				return -1;
		/* coding check, adc check and press base check  */
		pencil_data_operation(chip_info);
		break;
	default:
		break;
	}
	return 0;
}

static void pencil_check_data(struct chip_data_brl *chip_info, char *check_buf)
{
	if (!strncmp(check_buf, PENCIL_DATA_ADDR2, PENCIL_DATA_MATCH) && strlen(check_buf) > PENCIL_DATA_MATCH)
		if (pencil_data_handle(chip_info, (check_buf + PENCIL_DATA_MATCH), P_ADDR2) < 0)
			TPD_INFO("GT_brlD:%s:operation failed!!\n", __func__);
}

static ssize_t proc_pencil_healthinfo_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct touchpanel_data *ts = PDE_DATA(file_inode(file));
	struct chip_data_brl *chip_info = (struct chip_data_brl *)ts->chip_data;
	char get_buf[MAX_PENCIL_DATA] = {0};

	if (ts->is_suspended) {
		TPD_INFO("GT_brlD:%s:now suspend, not to set", __func__);
		goto OUT;
	}

	if (!chip_info->pen_enable) {
		TPD_DEBUG("GT_brlD:%s:now pen disable, not set", __func__);
		goto OUT;
	}

	if (count != PENCIL_CHECK_COUNT) {
		TPD_DEBUG("GT_brlD:%s count not %ld\n", __func__, count);
		goto OUT;
	}

	if (tp_copy_from_user(get_buf, sizeof(get_buf), buffer, count, MAX_PENCIL_DATA)) {
		TPD_INFO("GT_brlD:%s: read proc input error.\n", __func__);
		goto OUT;
	}

	get_buf[count] = '\0';

	TPD_DEBUG("GT_brlD:%s:pen register->char:[%s], count:%ld\n", __func__, get_buf, count);

	pencil_check_data(chip_info, get_buf);

OUT:
	return count;
}

DECLARE_PROC_OPS(proc_pencil_healthinfo_fops, simple_open, NULL, proc_pencil_healthinfo_write, NULL);

void pen_init_debug_node(struct chip_data_brl *chip_info)
{
	struct proc_dir_entry *prEntry_tmp = NULL;

	if (chip_info->pen_support == true) {
		prEntry_tmp = proc_create_data(PENCIL_DATA_PATH, PENCIL_DATA_CHMOD,
				chip_info->ts->prEntry_tp, &proc_pencil_healthinfo_fops, chip_info->ts);

		if (prEntry_tmp == NULL) {
			chip_info->pen_healthinfo_node = false;
			TPD_INFO("GT_brlD:%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
		} else {
			chip_info->pen_healthinfo_node = true;
			TPD_INFO("GT_brlD:%s: create proc %s pass, %d\n", __func__, PENCIL_DATA_PATH, __LINE__);
			chip_info->pen_err_coding_cnt = 0;
			chip_info->pen_err_adc_cnt    = 0;
			chip_info->pen_err_base_cnt   = 0;
		}
	}
}
/* pencil healthinfo end */

void pen_checkdown_work(struct work_struct *work)
{
	struct chip_data_brl *chip_info =
		container_of(work, struct chip_data_brl, check_pendown_work);
	int retry = 0;
	int ret   = 0;
	do {
		if (chip_info->pen_state == PEN_DOWN) {
			TPD_INFO("GT_brlD:%s,detect now penDown\n", __func__);
			chip_info->pen_input_state = PEN_UP;
			msleep(PENCIL_RETYR_TIME);
		}
		TPD_INFO("GT_brlD:%s,check pen state:%d\n", __func__, chip_info->pen_state);
		if (chip_info->pen_state == PEN_UP) {
			TPD_INFO("GT_brlD:%s,pen up success\n", __func__);
			ret = goodix_send_cmd_simple(chip_info, GTP_PEN_ENABLE_MASK, GTP_MASK_DISABLE);
			if (BIT_CHK(chip_info->pen_ctl_para, PEN_CTL_SMALL_PALM_ENABLE)) {
				TPD_INFO("GT_brlD: %s->game on->must disable small palm\n", __func__);
				ret = goodix_pen_control_palm_inpen(chip_info, PEN_CTL_SMALL_PALM_CLOSE);
			}
			chip_info->pen_enable = !!GTP_MASK_DISABLE;
			return;
		}
		msleep(PENCIL_RETYR_TIME);
		retry+=1;
	} while (retry <= PENCIL_RETYR_CNT);
	return;
}

int goodix_enable_pen_mode(struct chip_data_brl *chip_info, bool enable)
{
	int ret = 0;

	if (chip_info->ts == NULL) {
		PEN_ERR("chip_info->ts is NULL\n");
		return -EINVAL;
	}

	if (enable) {
		ret = goodix_send_cmd_simple(chip_info, GTP_PEN_ENABLE_MASK, GTP_MASK_ENABLE);
		chip_info->pen_input_state = PEN_DOWN;
		tp_healthinfo_report(&chip_info->ts->monitor_data, HEALTH_REPORT, "pen mode enable");
	} else {
		if (chip_info->pen_state == PEN_DOWN) {
			TPD_INFO("GT_brlD:now penDown,start work to detect penUp\n", __func__);
			schedule_work(&chip_info->check_pendown_work);
		} else if (chip_info->pen_state == PEN_UP) {
			ret = goodix_send_cmd_simple(chip_info, GTP_PEN_ENABLE_MASK, GTP_MASK_DISABLE);
			if (BIT_CHK(chip_info->pen_ctl_para, PEN_CTL_SMALL_PALM_ENABLE)) {
				TPD_INFO("GT_brlD: %s->game on->must disable small palm\n", __func__);
				ret = goodix_pen_control_palm_inpen(chip_info, PEN_CTL_SMALL_PALM_CLOSE);
			}
		}
		tp_healthinfo_report(&chip_info->ts->monitor_data, HEALTH_REPORT, "pen mode disable");
	}

	TPD_INFO("GT:%s, now %s pen\n", __func__, enable > 0 ? "enable" : "disable");
	return ret;
}

static int goodix_pen_control_vibrator(struct chip_data_brl *chip_info, int ctl_cmd)
{
	u8 data = chip_info->pen_support_opp ? BIT_CHK(ctl_cmd, PEN_CTL_VIBRATOR_ENABLE) : !BIT_CHK(ctl_cmd, PEN_CTL_VIBRATOR_ENABLE);
	u8 cmd_type = chip_info->pen_support_opp ? PEN_OPP_CMD_VIBRATOR : PEN_CMD_VIBRATOR;

	if (goodix_send_cmd_simple(chip_info, cmd_type, data) < 0) {
		TPD_INFO("GT_brlD:%s fail to set vibrator cmd:%d\n", __func__, ctl_cmd & PEN_CTL_VIBRATOR_ENABLE);
		return -1;
	}

	chip_info->pen_ctl_para = BIT_SET(PEN_CTL_VIBRATOR) | chip_info->pen_ctl_para;

	if (BIT_CHK(ctl_cmd, PEN_CTL_VIBRATOR_ENABLE))
		chip_info->pen_ctl_para = BIT_SET(PEN_CTL_VIBRATOR_ENABLE) | chip_info->pen_ctl_para;
	else
		chip_info->pen_ctl_para = BIT_CLR(PEN_CTL_VIBRATOR_ENABLE) & chip_info->pen_ctl_para;

	TPD_INFO("GT_brlD:%s find vibrator cmd and set cmd:%d, data %d, cmd_type 0x%x, opp %d\n", __func__, ctl_cmd,
		data, cmd_type, chip_info->pen_support_opp);
	return 0;
}

int goodix_pen_control_palm_inpen(struct chip_data_brl *chip_info, int ctl_cmd)
{
	TPD_INFO("GT_brlD:%s find palm cmd and set cmd:%d\n", __func__, BIT_CHK(ctl_cmd, PEN_CTL_SMALL_PALM_ENABLE));

	if (goodix_send_cmd_simple(chip_info, PEN_CMD_SMALL_PALM, BIT_CHK(ctl_cmd, PEN_CTL_SMALL_PALM_ENABLE)) < 0)
		return -1;

	chip_info->pen_ctl_para = BIT_SET(PEN_CTL_SMALL_PALM) | chip_info->pen_ctl_para;

	if (BIT_CHK(ctl_cmd, PEN_CTL_SMALL_PALM_ENABLE)) {
		TPD_INFO("GT_brlD:%s small palm open ok\n", __func__);
		chip_info->pen_ctl_para = BIT_SET(PEN_CTL_SMALL_PALM_ENABLE) | chip_info->pen_ctl_para;
	} else {
		TPD_INFO("GT_brlD:%s small palm close ok\n", __func__);
		chip_info->pen_ctl_para = BIT_CLR(PEN_CTL_SMALL_PALM_ENABLE) & chip_info->pen_ctl_para;
	}

	return 0;
}

int goodix_pen_control(struct chip_data_brl *chip_info, int ctl_cmd)
{
	TPD_INFO("GT_brlD:%s, pen control cmd is [%2d]-[0x%x]\n", __func__, ctl_cmd, ctl_cmd);

	if (ctl_cmd == PEN_CTL_FEEDBACK) {
		TPD_INFO("GT_brlD:%s flag is feedback, return now control para\n", __func__);
		goto REPORT;
	}

	if (!ctl_cmd) {
		TPD_INFO("GT_brlD:%s invaled data!!\n", __func__);
		goto ERR;
	}

	if (BIT_CHK(ctl_cmd, PEN_CTL_VIBRATOR))
		if (!!goodix_pen_control_vibrator(chip_info, ctl_cmd))
			goto ERR;

	if (BIT_CHK(ctl_cmd, PEN_CTL_SMALL_PALM) && chip_info->pen_enable)
		if (!!goodix_pen_control_palm_inpen(chip_info, ctl_cmd))
			goto ERR;

REPORT:
	return chip_info->pen_ctl_para;
ERR:
	return -1;
}

int goodix_pen_downlink_data_package(u32 cmd, u32 buf_len, u8 *buf, u32 w_len, u8 *write_buf)
{
	int i;

	/* head */
	write_buf[GTP_SET_CMD_STATUS_OFFSET] = 0;
	write_buf[GTP_SET_CMD_ACK_OFFSET] = 0;
	write_buf[GTP_SET_CMD_LEN_OFFSET] = 2 + buf_len + 2;
	write_buf[GTP_SET_CMD_CMD_OFFSET] = brl_pen_set_cmd_map[cmd];

	/* data */
	if (cmd == PEN_DOWN_CMD_TIMING_CFG_ACK) {
		write_buf[GTP_SET_CMD_DATA0_OFFSET] = buf[0] << 7 | 1; /* timing */
	} else if (cmd == PEN_DOWN_CMD_HOPFRQ_CFG_ACK) {
		write_buf[GTP_SET_CMD_DATA0_OFFSET] = buf[0] << 7 | 2; /* hop frq */
	} else {
		for (i = 0; i < buf_len; i++) {
			write_buf[GTP_SET_CMD_DATA0_OFFSET+ i] = buf[i];
		}
	}

	return	4 + buf_len + 2;
}

static void goodix_pen_write_press_status(struct chip_data_brl *chip_info, u16 old_pval, u16 cur_pval)
{
	int ret;
	u8 cur_press_status = 0;
	u8 old_press_status = 0;
	u8 press_buf[2] = {0};
	u8 write_buf[MAX_CMD_BUF_LEN] = {0};

	old_press_status = !!old_pval;
	cur_press_status = !!cur_pval;

	if (cur_press_status != old_press_status) {
		press_buf[0] = cur_press_status;
		PEN_INFO("write cur_press_status %d old %d cur %d\n", cur_press_status, old_pval, cur_pval);
		goodix_pen_downlink_data_package(PEN_DOWN_CMD_PRESS, 2, press_buf, MAX_CMD_BUF_LEN, write_buf);
		ret = brl_send_cmd(chip_info, (struct goodix_ts_cmd *)write_buf);
	}

	return;
}

static void goodix_get_pen_info(struct chip_data_brl *chip_info, struct pen_info *pen_info)
{
	int pen_num = chip_info->pen_num;

	if (pen_num != ALL_PEN_UP && chip_info->pen_input_state != PEN_UP) {
		pen_info->status = 1;
	}
	pen_info->point_type = chip_info->point_type;
	pen_info->x = chip_info->pen_x;
	pen_info->y = chip_info->pen_y;
	pen_info->min_x = chip_info->pen_min_x;
	pen_info->min_y = chip_info->pen_min_y;
	pen_info->max_x = chip_info->pen_max_x;
	pen_info->max_y = chip_info->pen_max_y;
}

static void goodix_pen_downlink_cmd_healthinfo(struct chip_data_brl *chip_info, u32 cmd)
{
	struct monitor_data *mon_data = &chip_info->ts->monitor_data;

	switch (cmd) {
	case PEN_DOWN_CMD_CFG_ID:
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_DOWN_CMD_CFG_ID));
		break;
	case PEN_DOWN_CMD_CON_STA:
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_DOWN_CMD_CON_STA));
		break;
	case PEN_DOWN_CMD_SIZE_INFO:
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_DOWN_CMD_SIZE_INFO));
		break;
	case PEN_DOWN_CMD_REQ_CFG:
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_DOWN_CMD_REQ_CFG));
		break;
	case PEN_DOWN_CMD_TIMING_CFG_ACK:
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_DOWN_CMD_TIMING_CFG_ACK));
		break;
	case PEN_DOWN_CMD_HOPFRQ_CFG_ACK:
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_DOWN_CMD_HOPFRQ_CFG_ACK));
		break;
	case PEN_DOWN_CMD_SPEED_SWITCH:
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_DOWN_CMD_SPEED_SWITCH));
		break;
	case PEN_DOWN_CMD_FRQ:
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_DOWN_CMD_FRQ));
		break;
	default:
		break;
	}
}

int goodix_pen_downlink_data(void *chip_data, u32 cmd, u32 buf_len, u8 *buf)
{
	int ret = 0;
	u16 press_val;
	u16 old_pval;
	u16 frq_val;
	u32 write_len = 0;
	u8 frq_buf[2] = {0};
	u8 write_buf[MAX_CMD_BUF_LEN] = {0};
	struct pen_info pen_info;
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	if (chip_info->ts == NULL) {
		PEN_ERR("chip_info->ts is NULL\n");
		return -EINVAL;
	}
	if (cmd >= PEN_DOWN_CMD_MAX) {
		PEN_ERR("invaild cmd %d\n", cmd);
		return -EINVAL;
	}

	switch (cmd) {
	case PEN_DOWN_CMD_CFG_ID:
	case PEN_DOWN_CMD_CON_STA:
	case PEN_DOWN_CMD_SIZE_INFO:
	case PEN_DOWN_CMD_REQ_CFG:
	case PEN_DOWN_CMD_TIMING_CFG_ACK:
	case PEN_DOWN_CMD_HOPFRQ_CFG_ACK:
	case PEN_DOWN_CMD_SPEED_SWITCH:
		write_len = goodix_pen_downlink_data_package(cmd, buf_len, buf, MAX_CMD_BUF_LEN, write_buf);
		ret = brl_send_cmd(chip_info, (struct goodix_ts_cmd *)write_buf);
		break;
	case PEN_DOWN_CMD_PRESS:
		/* Write pressure status only when it changes */
		press_val = le16_to_cpup((__le16 *)buf);
		goodix_get_pen_info(chip_info, &pen_info);
		press_val = touch_pen_press_debounce(&pen_info, press_val, chip_info->pen_last_press);
		touch_pen_press_smooth_pre(press_val, &chip_info->pen_last_press);
		old_pval = chip_info->pen_press;
		chip_info->pen_press = press_val;
		goodix_pen_write_press_status(chip_info, old_pval, press_val);
		PEN_INFO("set press:cmd 0x%x, old_pval %d new press_val %d\n", cmd, old_pval, chip_info->pen_press);
		break;

	case PEN_DOWN_CMD_FRQ:
		/* Write frq*/
		frq_buf[0] = buf[0];
		frq_buf[1] = buf[1];
		frq_val = le16_to_cpup((__le16 *)buf);
		chip_info->pen_frq_val = frq_val;
		PEN_INFO("set frq:cmd 0x%x, frq_val %d\n", cmd, frq_val);
		write_len = goodix_pen_downlink_data_package(cmd, 2, frq_buf, MAX_CMD_BUF_LEN, write_buf);
		ret = brl_send_cmd(chip_info, (struct goodix_ts_cmd *)write_buf);
		break;
	default:
		PEN_ERR("invaild cmd %d\n", cmd);
		return -EINVAL;
	}

	if (ret) {
		PEN_ERR("goodix_reg_write failed ret %d cmd %d buf[%*ph] write_buf[%*ph]\n",
			ret, cmd, buf_len, buf, write_len, write_buf);
		return ret;
	}

	goodix_pen_downlink_cmd_healthinfo(chip_info, cmd);
	PEN_INFO("set cmd success:cmd 0x%x, buf[%*ph] write_buf[%*ph]\n",
		 cmd, buf_len, buf, write_len, write_buf);

	return 0;
}

void goodix_get_pen_points(void *chip_data, struct pen_info *pen_info)
{
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;
	int pen_num = ALL_PEN_UP;
	u8 *coor_data;
	u8 cur_key_map;
	int16_t x_angle, y_angle;
	u16 press = 0;
	u16 old_press = 0;

	TPD_DEBUG("GT_brlD:%s:check pen_data=[%*ph]\n", __func__,
		IRQ_EVENT_HEAD_LEN, chip_info->touch_data);

	pen_num = chip_info->touch_data[POINT_NUM_OFFSET] & 0x0F;
	chip_info->pen_num = pen_num;

	if (pen_num == ALL_PEN_UP || chip_info->pen_input_state == PEN_UP) { /* PEN Up */
		chip_info->pen_state          = PEN_UP;
		chip_info->check_start        = ON;
		chip_info->check_hrtimer_over = TIME_CHECK_START;
		start_time(chip_info, PEN__CHECK_HRTIMER);
		TPD_DEBUG("GT_brlD:%s:pen UP\n", __func__);
		return;
	}

	chip_info->pen_state = PEN_DOWN;
	if (chip_info->touch_state == TOUCH_DOWN) {
		TPD_DEBUG("GT_brlD:%s:detect touch down, must release all touch!!\n", __func__);
		chip_info->touch_state = TOUCH_UP;
		goodix_clear_status(chip_info);
	}

	coor_data = &chip_info->touch_data[IRQ_EVENT_HEAD_LEN];
	chip_info->point_type = chip_info->touch_data[IRQ_EVENT_HEAD_LEN] & 0xf;
	chip_info->pen_x = le16_to_cpup((__le16 *)(coor_data + 2));
	chip_info->pen_y = le16_to_cpup((__le16 *)(coor_data + 4));

	pen_info->status = 1;
	pen_info->point_type = chip_info->point_type;
	pen_info->x = chip_info->pen_x;
	pen_info->y = chip_info->pen_y;
	pen_info->min_x = chip_info->pen_min_x;
	pen_info->min_y = chip_info->pen_min_y;
	pen_info->max_x = chip_info->pen_max_x;
	pen_info->max_y = chip_info->pen_max_y;

	press = chip_info->pen_press;
	old_press = chip_info->pen_press;
	touch_pen_up_optimize(pen_info, &press, &chip_info->pen_last_press);
	goodix_pen_write_press_status(chip_info, old_press, press);
	chip_info->pen_press = press;
	press = touch_pen_press_smooth(press);
	pen_info->z = (chip_info->pen_support_opp) ? press : le16_to_cpup((__le16 *)(coor_data + 6));
	pen_info->d = (pen_info->z == 0) ? 1 : 0;
	x_angle = le16_to_cpup((__le16 *)(coor_data + 8));
	y_angle = le16_to_cpup((__le16 *)(coor_data + 10));
	pen_info->speed = le16_to_cpup((__le16 *)(coor_data + 12));

	pen_info->tilt_x = x_angle / 100;
	pen_info->tilt_y = y_angle / 100;

	cur_key_map = (chip_info->touch_data[3] & 0x0F) >> 1;
	if (cur_key_map & 0x01)
		pen_info->btn1 = 1;
	if (cur_key_map & 0x02)
		pen_info->btn2 = 1;
}

static u8 goodix_pen_get_cmd(struct chip_data_brl *chip_info, u8 request_type)
{
	u8 cmd;
	struct monitor_data *mon_data = &chip_info->ts->monitor_data;

	switch (request_type) {
	case GTP_RQST_TIMING:
		cmd = PEN_UP_CMD_TIMING;
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_UP_CMD_TIMING));
		break;
	case GTP_RQST_HOP_FREQ:
		cmd = PEN_UP_CMD_HOP_FRQ;
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_UP_CMD_HOP_FRQ));
		break;
	case GTP_RQST_PEN_SIZE:
		cmd = PEN_UP_CMD_GET_SIZE;
		tp_healthinfo_report(mon_data, HEALTH_REPORT, CMD_STR(PEN_UP_CMD_GET_SIZE));
		break;
	default:
		PEN_ERR("invaild request_type %d!\n", request_type);
		return -EINVAL;
	}

	return cmd;
}

static u32 goodix_pen_get_data_addr_offset(u8 event, u8 point_type, u8 touch_num)
{
	if ((event & GOODIX_TOUCH_EVENT) && (event & GOODIX_REQUEST_EVENT)) {
		if ((point_type == POINT_TYPE_STYLUS || point_type == POINT_TYPE_STYLUS_HOVER)) {
			return 16 + 2; /* pen touch */
		} else {
			return (touch_num == 0) ? 0: (touch_num * 8 + 2); /* hand touch */
		}
	} else {
		return 0;
	}
}

int goodix_pen_uplink_data(void *chip_data, u32 buf_len, u8 *buf, u32 *out_len)
{
	int ret, i;
	u8 cmd = 0;
	u8 event, point_type, touch_num, request_cmd;
	u32 offset, req_data_len, data_len;
	u8 data_buf[64] = {0};
	struct chip_data_brl *chip_info = (struct chip_data_brl *)chip_data;

	if (chip_info->ts == NULL) {
		PEN_ERR("chip_info->ts is NULL\n");
		return -EINVAL;
	}

	event = chip_info->touch_data[IRQ_EVENT_TYPE_OFFSET];
	point_type = chip_info->touch_data[IRQ_EVENT_HEAD_LEN] & 0x0F;
	touch_num = chip_info->touch_data[POINT_NUM_OFFSET] & 0x0F;
	req_data_len = chip_info->touch_data[GTP_PEN_DATA_LEN_OFFSET];
	offset = goodix_pen_get_data_addr_offset(event, point_type, touch_num);

	ret = goodix_reg_read(chip_info, GTP_PEN_START_REG_ADDR + offset, data_buf, req_data_len);
	if (ret < 0) {
		PEN_ERR("GT:%s: spi transfer error!\n", __func__);
		return ret;
	}

	if (checksum_cmp(data_buf, req_data_len, CHECKSUM_MODE_U8_LE)) {
		PEN_ERR("event 0x%x, point_type 0x%x, touch_num %d, offset %d req_data_len %d\n",
			event, point_type, touch_num, offset, req_data_len);
		PEN_ERR("addr 0x%x, pen data checksum err !!, data_buf[%*ph]\n",
			GTP_PEN_START_REG_ADDR, req_data_len, data_buf);
		return -EINVAL;
	}

	request_cmd = data_buf[0];
	cmd = goodix_pen_get_cmd(chip_info, request_cmd);
	data_len = req_data_len - 3;

	buf[0] = cmd;
	buf[1] = data_len;
	if (data_len > 0) {
		for (i = 0; i < data_len; i++) {
		    buf[2 + i] = data_buf[1 + i];
		}
	}

	*out_len = data_len + 2;

	PEN_INFO("cmd:%d, req type 0x%x, len %d, event 0x%x, point_type 0x%x, touch_num %d, offset %d\n",
		 cmd, request_cmd, data_len, event, point_type, touch_num, offset);

	return 0;
}

void goodix_get_pen_health_info(struct chip_data_brl *chip_info, struct monitor_data *mon_data)
{
	char *pen_str  = NULL;

	if (chip_info->pen_enable) {
		pen_str = kzalloc(MAX_PENCIL_DATA, GFP_KERNEL);
		if (pen_str == NULL) {
			TPD_INFO("GT:pen_str kzalloc failed.\n");
			return;
		}
		/* recorder pencil coding*/
		snprintf(pen_str, MAX_PENCIL_DATA, "PenCodingStart_%x_%x_%x_%x_%x_%x_%x_%x_%x_%x_End",
			chip_info->pen_err_coding[0],
			chip_info->pen_err_coding[1],
			chip_info->pen_err_coding[2],
			chip_info->pen_err_coding[3],
			chip_info->pen_err_coding[4],
			chip_info->pen_err_coding[5],
			chip_info->pen_err_coding[6],
			chip_info->pen_err_coding[7],
			chip_info->pen_err_coding[8],
			chip_info->pen_err_coding[9]);
		tp_healthinfo_report(mon_data, HEALTH_REPORT, pen_str);

		/* recorder pencil adc*/
		memset(pen_str, 0, MAX_PENCIL_DATA);
		snprintf(pen_str, MAX_PENCIL_DATA, "PenAdcStart_%hu_%hu_%hu_%hu_%hu_%hu_%hu_%hu_%hu_%hu_End",
			chip_info->pen_err_adc[0],
			chip_info->pen_err_adc[1],
			chip_info->pen_err_adc[2],
			chip_info->pen_err_adc[3],
			chip_info->pen_err_adc[4],
			chip_info->pen_err_adc[5],
			chip_info->pen_err_adc[6],
			chip_info->pen_err_adc[7],
			chip_info->pen_err_adc[8],
			chip_info->pen_err_adc[9]);
		tp_healthinfo_report(mon_data, HEALTH_REPORT, pen_str);

		/* recorder pencil base and press*/
		memset(pen_str, 0, MAX_PENCIL_DATA);
		snprintf(pen_str, MAX_PENCIL_DATA, "PenBaseStart_%hu_%hu_%hu_%hu_%hu_%hu_%hu_%hu_%hu_%hu_and_press_%hu_End",
			chip_info->pen_err_base[0],
			chip_info->pen_err_base[1],
			chip_info->pen_err_base[2],
			chip_info->pen_err_base[3],
			chip_info->pen_err_base[4],
			chip_info->pen_err_base[5],
			chip_info->pen_err_base[6],
			chip_info->pen_err_base[7],
			chip_info->pen_err_base[8],
			chip_info->pen_err_base[9],
			chip_info->pen_err_press);
		tp_healthinfo_report(mon_data, HEALTH_REPORT, pen_str);

		kfree(pen_str);
	}
}

