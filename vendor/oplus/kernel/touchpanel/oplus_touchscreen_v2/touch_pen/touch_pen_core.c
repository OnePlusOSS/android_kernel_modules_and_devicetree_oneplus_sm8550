#include <linux/module.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/thermal.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/iio/consumer.h>
#include <linux/alarmtimer.h>
#include <linux/timekeeping.h>

#include "touchpanel_common.h"
#include "touch_pen_core.h"

int touch_pen_init(struct touchpanel_data *ts)
{
    if (ts == NULL) {
        PEN_ERR("ts is NULL");
        return -EINVAL;
    }

    ts->pen_msg_list = init_message_list(PEN_MSG_NUM, PEN_MSG_BLOCK_BIT, TOUCH_PEN_NAME);
    if (ts->pen_msg_list == NULL) {
        PEN_ERR("ts is NULL");
        return -EINVAL;
    }
    ts->pen_init_success = 1;
    PEN_INFO("touch_pen_init, index %d", ts->tp_index);
    return 0;
}

void touch_pen_uninit(struct touchpanel_data *ts)
{
    if (ts == NULL) {
        PEN_ERR("ts is NULL");
        return;
    }
    clear_message_list(&ts->pen_msg_list);
    ts->pen_init_success = 0;
    PEN_INFO("touch_pen_uninit, index %d", ts->tp_index);
}

long touch_pen_uplink_msg_ioctl(struct touchpanel_data *ts, unsigned long arg)
{
    int ret = 0;
    struct pen_ioc_uplk_msg pen_msg = {0};
    struct message_node *tp_msg = NULL;

    if (ts->pen_init_success == 0) {
        PEN_INFO("pen not initialized, tp id %d", ts->tp_index);
        return -EINVAL;
    }

    if (copy_from_user(&pen_msg, (struct pen_ioc_uplk_msg *)arg, sizeof(struct pen_ioc_uplk_msg))) {
        PEN_ERR("copy_from_user failed");
        return -EINVAL;
    }

    tp_msg = get_message(ts->pen_msg_list, pen_msg.timeout);
    if (tp_msg == NULL) {
        PEN_ERR("tp_msg is NULL");
        return -EINVAL;
    }

    pen_msg.device_id = ts->tp_index;
    pen_msg.cmd = tp_msg->msg[0];
    pen_msg.len = tp_msg->msg[1];
    ret = copy_to_user((void *)arg, &pen_msg, sizeof(struct pen_ioc_uplk_msg));
    if (ret) {
        PEN_ERR("Fail to copy to user, len %d\n", tp_msg->len);
        ret = -EINVAL;
        goto out;
    }

    if (pen_msg.len > 0) {
        ret = copy_to_user((void *)pen_msg.buf, &tp_msg->msg[2], pen_msg.len);
        if (ret) {
            PEN_ERR("Fail to copy to user, len %d\n", tp_msg->len);
            ret = -EINVAL;
            goto out;
        }
    }

    TPD_DEBUG("tp id %d, cmd %d, len %d, get msg:[%*ph]", ts->tp_index,
        pen_msg.cmd, pen_msg.len, tp_msg->len, tp_msg->msg);
out:
    delete_message_node(ts->pen_msg_list, &tp_msg);
    return ret;
}

long touch_pen_downlink_msg_ioctl(struct touchpanel_data *ts, unsigned long arg)
{
    int ret = 0;
    struct pen_ioc_downlk_msg pen_msg = {0};
    u8 buf[PEN_BUF_SIZE] = {0};

    if (ts->pen_init_success == 0) {
        PEN_INFO("pen not initialized, tp id %d", ts->tp_index);
        return -EINVAL;
    }

    if (ts->is_suspended) {
        TPD_DEBUG("tp is suspended, device id %d\n", ts->tp_index);
        return 0;
    }

    if (copy_from_user(&pen_msg, (struct pen_ioc_downlk_msg *)arg, sizeof(struct pen_ioc_downlk_msg))) {
        PEN_ERR("copy_from_user failed");
        return -EINVAL;
    }

    if (pen_msg.cmd >= PEN_DOWN_CMD_MAX || pen_msg.len > PEN_BUF_SIZE) {
        PEN_ERR("tp id %d, check failed, cmd %d, len %d", ts->tp_index, pen_msg.cmd, pen_msg.len);
        return -EINVAL;
    }

    if (copy_from_user(buf, (void *)pen_msg.buf, pen_msg.len)) {
        PEN_ERR("copy_from_user failed");
        return -EINVAL;
    }

    if (ts->ts_ops->pen_downlink_msg == NULL) {
        PEN_ERR("check failed, set_pen_cmd is NULL");
        return -EINVAL;
    }

    ret = ts->ts_ops->pen_downlink_msg(ts->chip_data, pen_msg.cmd, pen_msg.len, buf);
    if (ret) {
        PEN_ERR("tp id %d, set cmd failed, ret %d, len %d", ts->tp_index, ret, pen_msg.len);
        return ret;
    }

    TPD_DEBUG("tp id %d, cmd %d len %d", ts->tp_index, pen_msg.cmd, pen_msg.len);
    return 0;
}

void touch_pen_uplink_msg_handle(struct touchpanel_data *ts)
{
    int ret = 0;
    u32 out_len = 0;
    u8 buf[PEN_BUF_SIZE] = {0};

    if (ts->pen_init_success == 0) {
        PEN_INFO("pen not initialized, tp id %d", ts->tp_index);
        return;
    }

    if (ts->ts_ops->pen_uplink_msg == NULL) {
        PEN_ERR("not support get_pen_cmd\n");
        return;
    }

    ret = ts->ts_ops->pen_uplink_msg(ts->chip_data, PEN_BUF_SIZE, buf, &out_len);
    if (ret) {
        PEN_ERR("tp id %d, get_pen_cmd failed, ret %d", ts->tp_index, ret);
        return;
    }

    post_message(ts->pen_msg_list, out_len, PEN_TYPE_GET_CMD, buf);
}

void touch_pen_speed_handle(struct touchpanel_data *ts, u16 speed)
{
    u32 len = 0;
    u8 buf[PEN_BUF_SIZE] = {0};
    static u16 old_speed = 0xffff;

    if (ts->pen_init_success == 0) {
        PEN_INFO("pen not initialized, tp id %d", ts->tp_index);
        return;
    }
    buf[0] = PEN_UP_CMD_SPEED;
    buf[1] = 2;
    buf[2] = speed & 0xff;
    buf[3] = speed >> 8;

    len = 4;

    if (old_speed != speed) {
        post_message(ts->pen_msg_list, len, PEN_TYPE_GET_CMD, buf);
        old_speed = speed;
    }
}

