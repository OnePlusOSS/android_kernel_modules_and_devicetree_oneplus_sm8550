/************************************************************************************
** File: - vendor\oplus\kernel\audio\include\oplus_audio_kernel_fb.h
** CONFIG_OPLUS_FEATURE_MM_FEEDBACK
** Copyright (C), 2022-2024, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**     add oplus audio kernel feedback
** Version: 1.0
** --------------------------- Revision History: --------------------------------
**      <author>                                        <date>              <desc>
************************************************************************************/
#ifndef OPLUS_AUDIO_KERNEL_FB_H
#define OPLUS_AUDIO_KERNEL_FB_H

#include <linux/device.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define AUDIO_KERNEL_FB_VERSION                "1.0.1"

#define OPLUS_AUDIO_EVENTID_AUDIO_KERNEL_ERR   (10047)
#define FEEDBACK_RATELIMIT_INTERVAL            (300 * HZ)
#define FEEDBACK_RATELIMIT_BURST               1

#define audio_kernel_fb(level, delay, fmt, ...) \
	do { \
		static DEFINE_RATELIMIT_STATE(_rs, \
						FEEDBACK_RATELIMIT_INTERVAL, \
						FEEDBACK_RATELIMIT_BURST); \
		if (__ratelimit(&_rs)) \
			mm_fb_audio(OPLUS_AUDIO_EVENTID_AUDIO_KERNEL_ERR, \
					MM_FB_KEY_RATELIMIT_30MIN, delay, level, fmt, ##__VA_ARGS__); \
	} while (0)

#define ratelimited_fb(fmt, ...) \
	audio_kernel_fb(FB_ERROR, 0, fmt, ##__VA_ARGS__)

/* replace "pr_err" and feedback */
#define pr_err_base(level, delay, fmt, ...) \
	do { \
		printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__); \
		audio_kernel_fb(level, delay, "payload@@"fmt, ##__VA_ARGS__); \
	} while (0)

#define pr_err_fb(fmt, ...) \
	pr_err_base(FB_ERROR, 0, fmt, ##__VA_ARGS__)

/* print error log and delay 60s to feedback error */
#define pr_err_fb_delay(fmt, ...) \
	pr_err_base(FB_ERROR, FEEDBACK_DELAY_60S, fmt, ##__VA_ARGS__)

#define pr_err_fb_fatal_delay(fmt, ...) \
	pr_err_base(FB_FATAL, FEEDBACK_DELAY_60S, fmt, ##__VA_ARGS__)

#define pr_err_ratelimited_fb(fmt, ...) \
	do { \
		printk_ratelimited(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__); \
		audio_kernel_fb(FB_ERROR, 0, "payload@@"fmt, ##__VA_ARGS__); \
	} while (0)

/* just print err log and not feedback, for skip some err log*/
#define pr_err_not_fb(fmt, ...) \
			printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__);


/* replace "dev_err" and feedback */
#define dev_err_base(level, delay, dev, fmt, ...) \
	do { \
		_dev_err(dev, dev_fmt(fmt), ##__VA_ARGS__); \
		audio_kernel_fb(level, delay, "payload@@%s %s:"dev_fmt(fmt), \
				dev_driver_string(dev), dev_name(dev), ##__VA_ARGS__); \
	} while (0)

#define dev_err_fb(dev, fmt, ...) \
	dev_err_base(FB_ERROR, 0, dev, fmt, ##__VA_ARGS__)

#define dev_err_fb_delay(dev, fmt, ...) \
	dev_err_base(FB_ERROR, FEEDBACK_DELAY_60S, dev, fmt, ##__VA_ARGS__)

#define dev_err_fb_fatal_delay(dev, fmt, ...) \
	dev_err_base(FB_FATAL, FEEDBACK_DELAY_60S, dev, fmt, ##__VA_ARGS__)

#define dev_err_not_fb(dev, fmt, ...) \
			_dev_err(dev, dev_fmt(fmt), ##__VA_ARGS__);


/* replace "dev_err_once" and feedback */
#define dev_err_once_base(level, delay, dev, fmt, ...) \
	do { \
		dev_level_once(dev_err, dev, fmt, ##__VA_ARGS__); \
		audio_kernel_fb(level, delay, "payload@@%s %s:"dev_fmt(fmt), \
				dev_driver_string(dev), dev_name(dev), ##__VA_ARGS__); \
	} while (0)

#define dev_err_once_fb(dev, fmt, ...) \
	dev_err_once_base(FB_ERROR, 0, dev, fmt, ##__VA_ARGS__)

#define dev_err_once_fb_delay(dev, fmt, ...) \
	dev_err_once_base(FB_ERROR, FEEDBACK_DELAY_60S, dev, fmt, ##__VA_ARGS__)

#define dev_err_once_not_fb(dev, fmt, ...) \
			dev_level_once(dev_err, dev, fmt, ##__VA_ARGS__);


/* replace "dev_err_ratelimited" and feedback */
#define dev_err_ratelimited_base(level, delay, dev, fmt, ...) \
	do { \
		dev_level_ratelimited(dev_err, dev, fmt, ##__VA_ARGS__); \
		audio_kernel_fb(level, delay, "payload@@%s %s:"dev_fmt(fmt), \
				dev_driver_string(dev), dev_name(dev), ##__VA_ARGS__); \
	} while (0)

#define dev_err_ratelimited_fb(dev, fmt, ...) \
	dev_err_ratelimited_base(FB_ERROR, 0, dev, fmt, ##__VA_ARGS__)

#define dev_err_ratelimited_fb_delay(dev, fmt, ...) \
	dev_err_ratelimited_base(FB_ERROR, FEEDBACK_DELAY_60S, dev, fmt, ##__VA_ARGS__)

#define dev_err_ratelimited_not_fb(dev, fmt, ...) \
			dev_level_ratelimited(dev_err, dev, fmt, ##__VA_ARGS__);

#endif /* OPLUS_AUDIO_KERNEL_FB_H */

