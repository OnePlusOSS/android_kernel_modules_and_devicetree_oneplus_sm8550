#include "touch_pen_core.h"

u8  press_smooth_time = 0;
u16  press_smooth_step = 0;
u8  press_smooth_direction = 0;
u16  press_smooth_last_val = 0;
u8  press_clear_flg = 0;

#define POINT_TYPE_STYLUS_HOVER      0x01
#define POINT_TYPE_STYLUS            0x03

int touch_pen_press_debounce(struct pen_info *info, u16 press_val, u16 last_press_val)
{
    u16 cur_press_val = press_val;
    static u8 press_filter_time = 0;

    PEN_DEBUG("start:press_val = %d,cur_press_val %d, point_type= %d status %d,"
        " last_press_val %d press_smooth_step %d press_clear_flg %d\n",
        press_val, cur_press_val, info->point_type, info->status,
        last_press_val, press_smooth_step, press_clear_flg);

    press_clear_flg = (press_val != 0 && last_press_val == 0) ? 0 : 1;
    if(info->status == 1) {
        if(press_val != 0 || last_press_val == 0) {
            press_filter_time = 0;
        } else {
            if ((press_filter_time < 3) && (info->point_type == POINT_TYPE_STYLUS) && (last_press_val < 200) &&
                (press_smooth_step < 40)) {
                cur_press_val = last_press_val;
                if((info->x > info->min_x) && (info->x < info->max_x) &&
                    (info->y > info->min_y) && (info->y < info->max_y)) {
                    press_filter_time++;
                } else {
                    press_filter_time += 2;
                }
            }else{
                press_filter_time = 0;
            }
        }
    } else {
        press_filter_time = 0;
    }

    PEN_DEBUG("end:press_val = %d,cur_press_val %d, point_type= %d status %d,"
        " last_press_val %d press_smooth_step %d press_clear_flg %d\n",
        press_val, cur_press_val, info->point_type, info->status,
        last_press_val, press_smooth_step, press_clear_flg);

    return cur_press_val;
}
EXPORT_SYMBOL(touch_pen_press_debounce);

void touch_pen_up_optimize(struct pen_info *info, u16 *cur_press_val, u16 *last_press_val)
{
    PEN_DEBUG("cur_press_val %d, last_press_val %d, point_type= %d press_clear_flg %d\n", *cur_press_val, *last_press_val,
        info->point_type, press_clear_flg);
    if (info->point_type == POINT_TYPE_STYLUS_HOVER && press_clear_flg == 1) {
        *cur_press_val = 0;
        *last_press_val = 0;
        press_smooth_last_val = 0;
    }
    press_clear_flg = 1;
    return;
}
EXPORT_SYMBOL(touch_pen_up_optimize);

void touch_pen_press_smooth_pre(u16 cur_press_val, u16 *last_press_val)
{
    PEN_DEBUG("start:cur_press_val = %d,last_press_val %d, press_smooth_time =%d val %d direction %d \n", cur_press_val, *last_press_val,
        press_smooth_time, press_smooth_step, press_smooth_direction);
    if (cur_press_val != 0 && *last_press_val != 0) {
        if (cur_press_val < *last_press_val) {
            press_smooth_direction = 0;
            press_smooth_step = (*last_press_val - cur_press_val) / 5;
        } else {
            press_smooth_direction = 1;
            press_smooth_step = (cur_press_val - *last_press_val) / 5;
        }
        if (press_smooth_step > PEN_SMOOTH_STEP_MAX) {
            press_smooth_step = PEN_SMOOTH_STEP_MAX;
        }
    } else {
        press_smooth_direction = 1;
        press_smooth_step = 0;
    }
    press_smooth_time = 0;

    *last_press_val = cur_press_val;
    PEN_DEBUG("end:cur_press_val = %d,last_press_val %d, press_smooth_time =%d val %d direction %d \n", cur_press_val, *last_press_val,
        press_smooth_time, press_smooth_step, press_smooth_direction);
}
EXPORT_SYMBOL(touch_pen_press_smooth_pre);

int touch_pen_press_smooth(u16 press_val)
{
    u32 smooth_step = 0;
    u16 cur_press_val = press_val;

    PEN_DEBUG("start:press_val = %d, press_smooth_time =%d val %d direction %d last_val %d\n", press_val,
        press_smooth_time, press_smooth_step, press_smooth_direction, press_smooth_last_val);

    if (press_val == 0) {
        press_smooth_last_val = 0;
        return 0;
    }
    if (press_smooth_time < PEN_SMOOTH_TIME_MAX) {
        smooth_step = (press_smooth_time > 0) ? press_smooth_step * press_smooth_time : 0;
        if(press_smooth_direction == 0) {
            if(cur_press_val > smooth_step) {
                cur_press_val -= smooth_step;
            }
            if (press_smooth_last_val < cur_press_val) {
                cur_press_val = press_smooth_last_val;
            }
        } else {
            cur_press_val += smooth_step;
            cur_press_val = (cur_press_val > PEN_PRESS_MAX) ? PEN_PRESS_MAX : cur_press_val;
            if (press_smooth_last_val > cur_press_val) {
                cur_press_val = press_smooth_last_val;
            }
        }
        press_smooth_last_val = cur_press_val;
        press_smooth_time++;
    } else {
        cur_press_val = press_smooth_last_val;
    }

    PEN_DEBUG("end:cur_press_val = %d, press_smooth_time =%d val %d direction %d last_val %d\n", cur_press_val,
        press_smooth_time, press_smooth_step, press_smooth_direction, press_smooth_last_val);
    return cur_press_val;
}
EXPORT_SYMBOL(touch_pen_press_smooth);
