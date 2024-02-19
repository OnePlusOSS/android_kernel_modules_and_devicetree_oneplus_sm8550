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
#include <linux/atomic.h>
#include <linux/types.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define AUDIO_KERNEL_FB_VERSION                "1.0.2"

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

/* continue error for several times trigger feedback */
#define audio_kernel_count_fb(is_err, cnt, level, delay, fmt, ...) \
	do { \
		static atomic_t g_cnt = ATOMIC_INIT(0); \
		static DEFINE_RATELIMIT_STATE(_rs, \
						FEEDBACK_RATELIMIT_INTERVAL, \
						FEEDBACK_RATELIMIT_BURST); \
		if (is_err) { \
			atomic_inc(&g_cnt); \
			if ((atomic_read(&g_cnt) > cnt) && __ratelimit(&_rs)) { \
				mm_fb_audio(OPLUS_AUDIO_EVENTID_AUDIO_KERNEL_ERR, \
						MM_FB_KEY_RATELIMIT_30MIN, delay, level, fmt, ##__VA_ARGS__); \
				atomic_set(&g_cnt, 0); \
			} \
		} else { \
			atomic_set(&g_cnt, 0); \
		} \
	} while (0)

#define ratelimited_count_fb(is_err, cnt, fmt, ...) \
	audio_kernel_count_fb(is_err, cnt, FB_ERROR, 0, fmt, ##__VA_ARGS__)

#define pr_err_count_base(is_err, cnt, level, delay, fmt, ...) \
	do { \
		printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__); \
		audio_kernel_count_fb(is_err, cnt, level, delay, "payload@@"fmt, ##__VA_ARGS__); \
	} while (0)

#define pr_err_count_fb(is_err, cnt, fmt, ...) \
	pr_err_count_base(is_err, cnt, FB_ERROR, 0, fmt, ##__VA_ARGS__)

/* print error log and delay 60s to feedback error */
#define pr_err_count_fb_delay(is_err, cnt, fmt, ...) \
	pr_err_count_base(is_err, cnt, FB_ERROR, FEEDBACK_DELAY_60S, fmt, ##__VA_ARGS__)

#define pr_err_count_fb_fatal_delay(is_err, cnt, fmt, ...) \
	pr_err_count_base(is_err, cnt, FB_FATAL, FEEDBACK_DELAY_60S, fmt, ##__VA_ARGS__)

#define pr_err_count_ratelimited_fb(is_err, cnt, fmt, ...) \
	do { \
		printk_ratelimited(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__); \
		audio_kernel_count_fb(is_err, cnt, FB_ERROR, 0, "payload@@"fmt, ##__VA_ARGS__); \
	} while (0)

/* replace "dev_err" and feedback */
#define dev_err_count_base(is_err, cnt, level, delay, dev, fmt, ...) \
	do { \
		_dev_err(dev, dev_fmt(fmt), ##__VA_ARGS__); \
		audio_kernel_count_fb(is_err, cnt, level, delay, "payload@@%s %s:"dev_fmt(fmt), \
				dev_driver_string(dev), dev_name(dev), ##__VA_ARGS__); \
	} while (0)

#define dev_err_count_fb(is_err, cnt, dev, fmt, ...) \
	dev_err_count_base(is_err, cnt, FB_ERROR, 0, dev, fmt, ##__VA_ARGS__)

#define dev_err_count_fb_delay(is_err, cnt, dev, fmt, ...) \
	dev_err_count_base(is_err, cnt, FB_ERROR, FEEDBACK_DELAY_60S, dev, fmt, ##__VA_ARGS__)

#define dev_err_count_fb_fatal_delay(is_err, cnt, dev, fmt, ...) \
	dev_err_count_base(is_err, cnt, FB_FATAL, FEEDBACK_DELAY_60S, dev, fmt, ##__VA_ARGS__)

/* replace "dev_err_once" and feedback */
#define dev_err_count_once_base(is_err, cnt, level, delay, dev, fmt, ...) \
	do { \
		dev_level_once(dev_err, dev, fmt, ##__VA_ARGS__); \
		audio_kernel_count_fb(is_err, cnt, level, delay, "payload@@%s %s:"dev_fmt(fmt), \
				dev_driver_string(dev), dev_name(dev), ##__VA_ARGS__); \
	} while (0)

#define dev_err_count_once_fb(is_err, cnt, dev, fmt, ...) \
	dev_err_count_once_base(is_err, cnt, FB_ERROR, 0, dev, fmt, ##__VA_ARGS__)

#define dev_err_count_once_fb_delay(is_err, cnt, dev, fmt, ...) \
	dev_err_count_once_base(is_err, cnt, FB_ERROR, FEEDBACK_DELAY_60S, dev, fmt, ##__VA_ARGS__)

/* replace "dev_err_ratelimited" and feedback */
#define dev_err_count_ratelimited_base(is_err, cnt, level, delay, dev, fmt, ...) \
	do { \
		dev_level_ratelimited(dev_err, dev, fmt, ##__VA_ARGS__); \
		audio_kernel_count_fb(is_err, cnt, level, delay, "payload@@%s %s:"dev_fmt(fmt), \
				dev_driver_string(dev), dev_name(dev), ##__VA_ARGS__); \
	} while (0)

#define dev_err_count_ratelimited_fb(is_err, cnt, dev, fmt, ...) \
	dev_err_count_ratelimited_base(is_err, cnt, FB_ERROR, 0, dev, fmt, ##__VA_ARGS__)

#define dev_err_count_ratelimited_fb_delay(is_err, cnt, dev, fmt, ...) \
	dev_err_count_ratelimited_base(is_err, cnt, FB_ERROR, FEEDBACK_DELAY_60S, dev, fmt, ##__VA_ARGS__)

#define dev_err_count_ratelimited_not_fb(is_err, cnt, dev, fmt, ...) \
			dev_level_count_ratelimited(is_err, cnt, dev_err, dev, fmt, ##__VA_ARGS__);


#define audio_kernel_count_and_limit_time_fb(is_err, cnt, limit_ms, level, delay, fmt, ...) \
	do { \
		static atomic_t g_cnt = ATOMIC_INIT(0); \
		static ktime_t g_time = 0; \
		static DEFINE_RATELIMIT_STATE(_rs, \
						FEEDBACK_RATELIMIT_INTERVAL, \
						FEEDBACK_RATELIMIT_BURST); \
		if (is_err) { \
			atomic_inc(&g_cnt); \
			if ((atomic_read(&g_cnt) > cnt) && \
				(g_time != 0) && ktime_after(ktime_get(), ktime_add_ms(g_time, limit_ms)) && \
				__ratelimit(&_rs)) { \
				mm_fb_audio(OPLUS_AUDIO_EVENTID_AUDIO_KERNEL_ERR, \
						MM_FB_KEY_RATELIMIT_30MIN, delay, level, fmt, ##__VA_ARGS__); \
				atomic_set(&g_cnt, 0); \
				g_time = 0; \
			} else if (g_time == 0) { \
				g_time = ktime_get(); \
			} \
		} else { \
			atomic_set(&g_cnt, 0); \
			g_time = 0; \
		} \
	} while (0)

#define ratelimited_count_limit_fb(is_err, cnt, limit_ms, fmt, ...) \
	audio_kernel_count_and_limit_time_fb(is_err, cnt, limit_ms, FB_ERROR, 0, fmt, ##__VA_ARGS__)

#endif /* OPLUS_AUDIO_KERNEL_FB_H */

