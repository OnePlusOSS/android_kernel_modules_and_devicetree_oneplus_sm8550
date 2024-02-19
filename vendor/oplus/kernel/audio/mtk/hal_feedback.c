/************************************************************************************
** File: -
** Copyright (C), 2020-2025, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**     add hal feedback
** Version: 1.0
** --------------------------- Revision History: --------------------------------
**               <author>                                <date>          <desc>
**
************************************************************************************/
#include <sound/control.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define HAL_FEEDBACK_MAX_BYTES         (512)
#define AUDIO_EVENTID_HAL_ERR          (10008)
#define OPLUS_FB_HAL_ERR_RATELIMIT     (300*1000)

#define MM_FB_EVENTID_LEN              (5)
#define IS_DIGITAL(x) (((x) >= '0') && ((x) <= '9'))

enum {
	TAG_ID = 0,
	TAG_LEVEL,
	TAG_DELAY,
	TAG_LIMIT,
	TAG_FN_LN,
	TAG_MAX
};

int hal_feedback_config_get(struct snd_kcontrol *kcontrol,
			unsigned int __user *bytes,
			unsigned int size)
{
	return 0;
}
EXPORT_SYMBOL(hal_feedback_config_get);

int hal_feedback_config_set(struct snd_kcontrol *kcontrol,
			const unsigned int __user *bytes,
			unsigned int size)
{
	int ret = 0;
	char info[HAL_FEEDBACK_MAX_BYTES + 1] = {0};
	unsigned int len = size;
	unsigned int event_id = 0;
	int lp = 0;
	mm_fb_param param;
	char *p1 = NULL;
	char *p2 = NULL;
	unsigned int tags[TAG_MAX] = {0, 0, 0, 0};
	unsigned int tag_num = 0;
	char tag_fn_ln[MAX_FUNC_LINE_SIZE] = {0};
	unsigned int fn_ln_len = 0;
	unsigned int offset = 0;

	if ((len < (MM_FB_EVENTID_LEN + 2)) || (len > HAL_FEEDBACK_MAX_BYTES)) {
		len = HAL_FEEDBACK_MAX_BYTES;
		pr_info("%s(), size(%d) out of range [%d, %d]",
				__func__, size, MM_FB_EVENTID_LEN + 2, HAL_FEEDBACK_MAX_BYTES);
	}

	if (copy_from_user(info, bytes, len)) {
		pr_err("%s(), Fail copy to user Ptr:(%p),len:%d",
				__func__, bytes, len);
		ret = -EFAULT;
		return ret;
	}
	info[len] = '\0';

	pr_info("%s(), %s", __func__, info);

	/* new format: "#event_id:level:delay:limit:func-line#payload info" */
	if (info[0] == '#') {
		p1 = info + 1;
		p2 = strchr(info + 1, '#');
		if (!p2 || ((p2 - p1 + 2) > len)) {
			printk(KERN_INFO "%s: format invalid, need #event_id:level:delay:limit:func-line#\n", __func__);
			goto exit;
		}
		offset = p2 - p1 + 2;

		while ((p1 < p2) && (tag_num < TAG_MAX)) {
			if (IS_DIGITAL(*p1) && (tag_num < TAG_FN_LN)) {
				tags[tag_num] = tags[tag_num] * 10 + *p1 - '0';
			} else if ((*p1 == ':') && (tag_num < TAG_FN_LN)) {
				tag_num++;
			} else if (tag_num == TAG_FN_LN) {
				fn_ln_len = p2 - p1;
				fn_ln_len = fn_ln_len > (MAX_FUNC_LINE_SIZE - 1) ? (MAX_FUNC_LINE_SIZE - 1) : fn_ln_len;
				memcpy(tag_fn_ln, p1, fn_ln_len);
				tag_fn_ln[fn_ln_len] = '\0';
				fn_ln_len += 1;
				break;
			} else {
				printk(KERN_INFO "%s: format invalid, need #event_id:level:delay:limit:func-line#\n", __func__);
				goto exit;
			}
			p1++;
		}

		if (tag_num != TAG_FN_LN) {
			printk(KERN_INFO "%s: format invalid, need #event_id:level:delay:limit:func-line#\n", __func__);
			goto exit;
		}
		if ((tags[TAG_ID] < OPLUS_AUDIO_EVENTID_ADSP_CRASH) || (tags[TAG_ID] > OPLUS_MM_EVENTID_MAX)) {
			printk(KERN_INFO "%s: event_id=%u invalid\n", __func__, tags[TAG_ID]);
			goto exit;
		}
		if (tags[TAG_LEVEL] >= FB_LEVEL_MAX) {
			printk(KERN_INFO "%s: level=%u invalid, set error level\n", __func__, tags[TAG_LEVEL]);
			tags[TAG_LEVEL] = FB_ERROR;
		}
		if (tags[TAG_LIMIT] == 0) {
			printk(KERN_INFO "%s: limit=%u invalid, set default limit\n", __func__, tags[TAG_LIMIT]);
			tags[TAG_LIMIT] = OPLUS_FB_HAL_ERR_RATELIMIT;
		}
		param.module = OPLUS_MM_DIRVER_FB_EVENT_AUDIO;
		param.event_id = tags[TAG_ID];
		param.level = tags[TAG_LEVEL];
		param.delay_s = tags[TAG_DELAY];
		param.limit_ms = tags[TAG_LIMIT] * 1000;
		memcpy(param.fn_ln, tag_fn_ln, fn_ln_len);
		memcpy(param.payload, info + offset, len - offset);
		param.payload[len - offset] = '\0';
		ret = upload_mm_fb_kevent_limit(&param);
	} else { /* old format: "payload info" or "event_id:payload info" */
		for (lp = 0; lp < MM_FB_EVENTID_LEN; lp++) {
			if (IS_DIGITAL(info[lp])) {
				event_id = event_id*10 + info[lp] - '0';
			} else {
				event_id = 0;
				break;
			}
		}

		if (event_id == 0) {
			ret = upload_mm_fb_kevent_to_atlas_limit(AUDIO_EVENTID_HAL_ERR, info,
					OPLUS_FB_HAL_ERR_RATELIMIT);
		} else {
			/* info format: (event_id + ":" + errorinfo), such as: (10008:payload@@assert error)*/
			ret = upload_mm_fb_kevent_to_atlas_limit(event_id, info + MM_FB_EVENTID_LEN + 1,
					OPLUS_FB_HAL_ERR_RATELIMIT);
		}
	}

exit:
	return ret;
}
EXPORT_SYMBOL(hal_feedback_config_set);

MODULE_LICENSE("GPL v2");

