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
#define OPLUS_FB_HAL_ERR_RATELIMIT     (60*1000)

#define MM_FB_EVENTID_LEN              (5)
#define IS_DIGITAL(x) (((x) >= '0') && ((x) <= '9'))


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

	return ret;
}
EXPORT_SYMBOL(hal_feedback_config_set);

MODULE_LICENSE("GPL v2");

