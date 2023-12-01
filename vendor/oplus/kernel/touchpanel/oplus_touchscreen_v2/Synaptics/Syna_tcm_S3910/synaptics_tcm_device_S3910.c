// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include "synaptics_tcm_S3910.h"

#define CHAR_DEVICE_NAME "tcm"
#define PLATFORM_DRIVER_NAME "synaptics_tcm_S3910"
#define CONCURRENT true

#define DEVICE_IOC_MAGIC 's'
#define DEVICE_IOC_RESET _IO(DEVICE_IOC_MAGIC, 0) /* 0x00007300 */
#define DEVICE_IOC_IRQ _IOW(DEVICE_IOC_MAGIC, 1, int) /* 0x40047301 */
#define DEVICE_IOC_RAW _IOW(DEVICE_IOC_MAGIC, 2, int) /* 0x40047302 */
#define DEVICE_IOC_CONCURRENT _IOW(DEVICE_IOC_MAGIC, 3, int) /* 0x40047303 */

#ifdef EXTERNAL_DEBUG_LOGGING
#define STD_GET_FRAME_ID		(0x15)
#define STD_SEND_MESSAGE_ID		(0x16)
#define STD_SET_REPORTS_ID		(0x17)
#define STD_CLEAN_OUT_FRAMES_ID (0x19)

#define DEVICE_IOC_GET_FRAME _IOR(DEVICE_IOC_MAGIC, STD_GET_FRAME_ID, struct syna_tcm_ioctl_data *) /* 0x80087315 */
#define DEVICE_IOC_SEND_MESSAGE _IOW(DEVICE_IOC_MAGIC, STD_SEND_MESSAGE_ID, struct syna_tcm_ioctl_data *) /* 0x40087316 */
#define DEVICE_IOC_SET_REPORTS _IOW(DEVICE_IOC_MAGIC, STD_SET_REPORTS_ID, struct syna_tcm_ioctl_data *) /* 0x40087317 */
#define DEVICE_IOC_CLEAN_OUT_FRAMES _IOW(DEVICE_IOC_MAGIC, STD_CLEAN_OUT_FRAMES_ID, struct syna_tcm_ioctl_data *) /* 0x40087319 */

#define FIFO_QUEUE_MAX_FRAMES	(240)

struct fifo_queue {
	struct list_head next;
	unsigned char *fifo_data;
	unsigned int data_length;
};
#endif

static struct device_hcd *g_device_hcd[TP_SUPPORT_MAX] = {NULL};

static void device_capture_touch_report(struct device_hcd *device_hcd,
					unsigned int count)
{
	int retval;
	unsigned char id;
	unsigned int idx;
	unsigned int size;
	unsigned char *data;
	struct syna_tcm_data *tcm_info = device_hcd->tcm_info;
	static bool report;
	static unsigned int offset;
	static unsigned int remaining_size;

	if (count < 2) {
		return;
	}

	data = &device_hcd->resp.buf[0];

	if (data[0] != MESSAGE_MARKER) {
		return;
	}

	id = data[1];
	size = 0;

	LOCK_BUFFER(device_hcd->report);

	switch (id) {
	case REPORT_TOUCH:
		if (count >= 4) {
			remaining_size = le2_to_uint(&data[2]);

		} else {
			report = false;
			goto exit;
		}

		retval = syna_tcm_alloc_mem(&device_hcd->report, remaining_size);

		if (retval < 0) {
			TPD_INFO("Failed to allocate memory for device_hcd->report.buf\n");
			report = false;
			goto exit;
		}

		idx = 4;
		size = count - idx;
		offset = 0;
		report = true;
		break;

	case STATUS_CONTINUED_READ:
		if (report == false) {
			goto exit;
		}

		if (count >= 2) {
			idx = 2;
			size = count - idx;
		}

		break;

	default:
		goto exit;
	}

	if (size) {
		size = MIN(size, remaining_size);
		retval = tp_memcpy(&device_hcd->report.buf[offset],
				   device_hcd->report.buf_size - offset,
				   &data[idx],
				   count - idx,
				   size);

		if (retval < 0) {
			TPD_INFO("Failed to copy touch report data\n");
			report = false;
			goto exit;

		} else {
			offset += size;
			remaining_size -= size;
			device_hcd->report.data_length += size;
		}
	}

	if (remaining_size) {
		goto exit;
	}

	LOCK_BUFFER(tcm_info->report.buffer);

	tcm_info->report.buffer.buf = device_hcd->report.buf;
	tcm_info->report.buffer.buf_size = device_hcd->report.buf_size;
	tcm_info->report.buffer.data_length = device_hcd->report.data_length;

	device_hcd->report_touch(tcm_info);

	UNLOCK_BUFFER(tcm_info->report.buffer);

	report = false;

exit:
	UNLOCK_BUFFER(device_hcd->report);

	return;
}

static int device_capture_touch_report_config(struct device_hcd *device_hcd,
		unsigned int count)
{
	int retval;
	unsigned int size;
	unsigned char *data;
	struct syna_tcm_data *tcm_info = device_hcd->tcm_info;

	if (device_hcd->raw_mode) {
		if (count < 3) {
			TPD_INFO("Invalid write data\n");
			return -EINVAL;
		}

		size = le2_to_uint(&device_hcd->out.buf[1]);

		if (count - 3 < size) {
			TPD_INFO("Incomplete write data\n");
			return -EINVAL;
		}

		if (!size) {
			return 0;
		}

		data = &device_hcd->out.buf[3];

	} else {
		size = count - 1;

		if (!size) {
			return 0;
		}

		data = &device_hcd->out.buf[1];
	}

	LOCK_BUFFER(tcm_info->config);

	retval = syna_tcm_alloc_mem(&tcm_info->config, size);

	if (retval < 0) {
		TPD_INFO("Failed to allocate memory for tcm_info->config.buf\n");
		UNLOCK_BUFFER(tcm_info->config);
		return retval;
	}

	retval = tp_memcpy(tcm_info->config.buf,
			   tcm_info->config.buf_size,
			   data,
			   size,
			   size);

	if (retval < 0) {
		TPD_INFO("Failed to copy touch report config data\n");
		UNLOCK_BUFFER(tcm_info->config);
		return retval;
	}

	tcm_info->config.data_length = size;

	UNLOCK_BUFFER(tcm_info->config);

	return 0;
}

#ifdef EXTERNAL_DEBUG_LOGGING
static int device_ioctl_get_frame(struct device_hcd *tcm_hcd,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int *frame_size)
{
	int retval = 0;
	int timeout = 0;
	unsigned char timeout_data[4] = {0};
	struct fifo_queue *pfifo_data = NULL;
	struct syna_tcm_data *tcm_info = tcm_hcd->tcm_info;

	if (buf_size < sizeof(timeout_data)) {
		retval = -EINVAL;
		goto exit;
	}
	retval = copy_from_user(timeout_data, ubuf_ptr, sizeof(timeout_data));
	if (retval) {
		retval = -EBADE;
		goto exit;
	}

	/* get the waiting duration */
	timeout = le4_to_uint(&timeout_data[0]);
	if (list_empty(&tcm_info->frame_fifo_queue)) {
		retval = wait_event_interruptible_timeout(tcm_info->wait_frame,
				(tcm_info->fifo_remaining_frame > 0),
				msecs_to_jiffies(timeout));
		if (retval == 0) {
			retval = -ETIMEDOUT;
			*frame_size = 0;
			goto exit;
		}
	}

	/* confirm the queue status */
	if (list_empty(&tcm_info->frame_fifo_queue)) {
		TPD_INFO("Is queue empty? The remaining frame = %d\n",
			tcm_info->fifo_remaining_frame);
		retval = -ENODATA;
		goto exit;
	}

	mutex_lock(&tcm_info->fifo_mutex);

	pfifo_data = list_first_entry(&tcm_info->frame_fifo_queue,
			struct fifo_queue, next);

	if (pfifo_data == NULL) {
		TPD_INFO("pfifo_data is NULL error\n");
		mutex_unlock(&tcm_info->fifo_mutex);
		retval = -ENODATA;
		goto exit;
	}

	TPD_INFO("Pop data from the queue, data length = %d\n",
		pfifo_data->data_length);

	if (buf_size >= pfifo_data->data_length) {
		retval = copy_to_user((void *)ubuf_ptr,
				pfifo_data->fifo_data,
				pfifo_data->data_length);
		if (retval) {
			TPD_INFO("Fail to copy data to user space, size:%d\n", retval);
			retval = -EBADE;
		}

		*frame_size = pfifo_data->data_length;

	} else {
		TPD_INFO("No enough space for data copy, buf_size:%d data:%d\n",
			buf_size, pfifo_data->data_length);

		mutex_unlock(&tcm_info->fifo_mutex);
		retval = -EOVERFLOW;
		goto exit;
	}

	TPD_INFO("From FIFO: (0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
		pfifo_data->fifo_data[0], pfifo_data->fifo_data[1],
		pfifo_data->fifo_data[2], pfifo_data->fifo_data[3]);

	if (retval >= 0)
		retval = pfifo_data->data_length;

	list_del(&pfifo_data->next);
	kfree(pfifo_data->fifo_data);
	kfree(pfifo_data);
	if (tcm_info->fifo_remaining_frame != 0)
		tcm_info->fifo_remaining_frame--;

	mutex_unlock(&tcm_info->fifo_mutex);

exit:
	return retval;
}

static void device_clean_queue(struct device_hcd *tcm_hcd)
{
	struct fifo_queue *pfifo_data = NULL;

	struct syna_tcm_data *tcm_info = tcm_hcd->tcm_info;

	mutex_lock(&tcm_info->fifo_mutex);

	while (!list_empty(&tcm_info->frame_fifo_queue)) {
		pfifo_data = list_first_entry(&tcm_info->frame_fifo_queue,
				struct fifo_queue, next);
		if (pfifo_data == NULL) {
			TPD_INFO("pfifo_data is NULL error\n");
			mutex_unlock(&tcm_info->fifo_mutex);
			return;
		}

		list_del(&pfifo_data->next);
		kfree(pfifo_data->fifo_data);
		kfree(pfifo_data);
		if (tcm_info->fifo_remaining_frame != 0)
			tcm_info->fifo_remaining_frame--;
	}

	TPD_INFO("Queue cleaned, frame: %d\n", tcm_info->fifo_remaining_frame);

	mutex_unlock(&tcm_info->fifo_mutex);
}

static int device_insert_fifo(struct syna_tcm_data *tcm_info,
		unsigned char *buf_ptr, unsigned int length)
{
	int retval = 0;
	struct fifo_queue *pfifo_data = NULL;
	struct fifo_queue *pfifo_data_temp = NULL;
	static int pre_remaining_frames = -1;

	mutex_lock(&tcm_info->fifo_mutex);

	/* check queue buffer limit */
	if (tcm_info->fifo_remaining_frame >= FIFO_QUEUE_MAX_FRAMES) {
		if (tcm_info->fifo_remaining_frame != pre_remaining_frames)
			TPD_INFO("Reached %d and drop FIFO first frame\n",
				tcm_info->fifo_remaining_frame);

		pfifo_data_temp = list_first_entry(&tcm_info->frame_fifo_queue,
						struct fifo_queue, next);

		list_del(&pfifo_data_temp->next);
		kfree(pfifo_data_temp->fifo_data);
		kfree(pfifo_data_temp);
		pre_remaining_frames = tcm_info->fifo_remaining_frame;
		tcm_info->fifo_remaining_frame--;
	} else if (pre_remaining_frames >= FIFO_QUEUE_MAX_FRAMES) {
		TPD_INFO("Reached limit, dropped oldest frame, remaining:%d\n",
			tcm_info->fifo_remaining_frame);
		pre_remaining_frames = tcm_info->fifo_remaining_frame;
	} else {
		TPD_INFO("Queued frames:%d\n",
			tcm_info->fifo_remaining_frame);
	}

	pfifo_data = kmalloc(sizeof(*pfifo_data), GFP_KERNEL);
	if (!(pfifo_data)) {
		TPD_INFO("Allocation size = %zu\n", (sizeof(*pfifo_data)));
		retval = -ENOMEM;
		goto exit;
	}

	pfifo_data->fifo_data = kmalloc(length, GFP_KERNEL);
	if (!(pfifo_data->fifo_data)) {
		TPD_INFO("Failed to allocate memory, size = %d\n", length);
		kfree(pfifo_data);
		retval = -ENOMEM;
		goto exit;
	}

	pfifo_data->data_length = length;

	memcpy((void *)pfifo_data->fifo_data, (void *)buf_ptr, length);
	/* append the data to the tail for FIFO queueing */
	list_add_tail(&pfifo_data->next, &tcm_info->frame_fifo_queue);
	tcm_info->fifo_remaining_frame++;
	retval = 0;
exit:
	mutex_unlock(&tcm_info->fifo_mutex);
	return retval;
}

void device_update_report_queue(struct syna_tcm_data *tcm_info,
		unsigned char code, struct syna_tcm_buffer *pevent_data)
{
	int retval;
	unsigned char *frame_buffer = NULL;
	unsigned int frame_length = 0;

	if ((pevent_data == NULL) ||
		(pevent_data->buf == NULL)) {
		TPD_INFO("Returned, invalid event data pointer\n");
		return;
	}
	if ((pevent_data->data_length == 0) &&
		(tcm_info->payload_length == 0)) {
		TPD_INFO("Returned, invalid event data length = 0\n");
		return;
	}
	TPD_INFO("Report ID = 0x%x\n", (int)code);
	frame_length = (tcm_info->payload_length + 3);
	frame_buffer = &pevent_data->buf[1];
	TPD_INFO("The overall queuing data length = %d\n", frame_length);

	retval = device_insert_fifo(tcm_info, frame_buffer, frame_length);
	if (retval < 0) {
		TPD_INFO("Fail to insert data to fifo\n");
		goto exit;
	}
	TPD_INFO("Pushed to fifo: (0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
		frame_buffer[0], frame_buffer[1],
		frame_buffer[2], frame_buffer[3]);
	wake_up_interruptible(&(tcm_info->wait_frame));

exit:
	return;
}

static int device_ioctl_set_reports(struct device_hcd *tcm_hcd,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int report_size)
{
	int retval = 0;
	unsigned char data[REPORT_TYPES] = {0};
	unsigned int reports = 0;
	unsigned int report_set = 0;

	struct syna_tcm_data *tcm_info = tcm_hcd->tcm_info;

	if (buf_size < sizeof(data)) {
		TPD_INFO("Invalid sync data size, buf_size:%d, expected:%d\n",
			buf_size, (unsigned int)sizeof(data));
		return -EINVAL;
	}

	if (report_size == 0) {
		TPD_INFO("Invalid written size\n");
		return -EINVAL;
	}

	retval = copy_from_user(data, ubuf_ptr, report_size);
	if (retval) {
		TPD_INFO("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	memcpy((void *)tcm_info->report_to_queue, (void *)data, REPORT_TYPES);
	for (reports = 0 ; reports < REPORT_TYPES ; reports++) {
		if (tcm_info->report_to_queue[reports] == EDL_ENABLE) {
			report_set++;
			TPD_INFO("Set report 0x%02x for queue\n", reports);
		}
	}
	TPD_INFO("Forward %d types of reports to the Queue.\n", report_set);
	retval = report_set;

exit:
	return retval;
}

static int device_ioctl_config_report(struct device_hcd *tcm_hcd,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int *msg_size)
{
	int retval = 0;
	unsigned int payload_length = 0;
	unsigned char *data = NULL;
	unsigned char cmd = 0;
	struct syna_tcm_data *tcm_info = tcm_hcd->tcm_info;

	if (buf_size < 2) {
		TPD_INFO("Invalid sync data size, buf_size:%d\n", buf_size);
		return -EINVAL;
	}

	if (*msg_size == 0 || *msg_size < 3) {
		TPD_INFO("Invalid message length, msg size: %d\n", *msg_size);
		return -EINVAL;
	}
	TPD_INFO("Message size:%d\n", *msg_size);
	data = kmalloc(*msg_size, GFP_KERNEL);
	if (!data) {
		TPD_INFO("Failed to allocate memory, size = %d\n", *msg_size);
		return -ENOMEM;
	}

	LOCK_BUFFER(tcm_hcd->out);

	retval = copy_from_user(data, ubuf_ptr, *msg_size);
	if (retval) {
		TPD_INFO("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		UNLOCK_BUFFER(tcm_hcd->out);
		kfree(data);
		return retval;
	}
	TPD_INFO("data[0]:%d data[1]:%d data[2]:%d\n",
			data[0], data[1], data[2]);

	cmd = data[0];
	payload_length = le2_to_uint(&data[1]);
	if (payload_length > 0 && ((buf_size - 3) >= payload_length)) {
		retval = syna_tcm_alloc_mem(&tcm_hcd->out, payload_length);
		if (retval < 0) {
			TPD_INFO("Failed to allocate memory for testing_hcd->out.buf\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			kfree(data);
			return retval;
		}

		memcpy((void *)tcm_hcd->out.buf, (void *)&data[3], payload_length);
	} else {
		TPD_INFO("Invaild payload length: %d, buf_size: %d\n",
					payload_length, buf_size);
	}
	if (data)
		kfree(data);

	TPD_INFO("CMD:%02x, payload_length:%d\n", cmd, payload_length);
	if (payload_length > 0) {
		TPD_INFO("Payload[0]:%02x\n", tcm_hcd->out.buf[0]);
	}

	LOCK_BUFFER(tcm_hcd->resp);

	retval = tcm_hcd->write_message(tcm_info,
			cmd,
			tcm_hcd->out.buf,
			payload_length,
			&tcm_hcd->resp.buf,
			&tcm_hcd->resp.buf_size,
			&tcm_hcd->resp.data_length,
			0);
	if (retval < 0) {
		TPD_INFO("Failed to write command %s\n",
				STR(cmd));
		UNLOCK_BUFFER(tcm_hcd->resp);
		UNLOCK_BUFFER(tcm_hcd->out);
		return retval;
	}

	UNLOCK_BUFFER(tcm_hcd->resp);
	UNLOCK_BUFFER(tcm_hcd->out);
	return retval;
}

static int device_ioctl_dispatch(struct device_hcd *tcm_hcd,
		unsigned int code, const unsigned char *ubuf_ptr,
		unsigned int ubuf_size, unsigned int *data_size)
{
	int retval = 0;
	switch (code) {
	case STD_GET_FRAME_ID:
		TPD_INFO("STD_GET_FRAME_ID called\n");
		retval = device_ioctl_get_frame(tcm_hcd,
				ubuf_ptr, ubuf_size, data_size);
		break;
	case STD_SEND_MESSAGE_ID:
		TPD_INFO("STD_SEND_MESSAGE_ID called\n");
		retval = device_ioctl_config_report(tcm_hcd,
				ubuf_ptr, ubuf_size, data_size);
		break;
	case STD_SET_REPORTS_ID:
		TPD_INFO("STD_SET_REPORTS_ID called\n");
		retval = device_ioctl_set_reports(tcm_hcd,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_CLEAN_OUT_FRAMES_ID:
		TPD_INFO("STD_CLEAN_OUT_FRAMES_ID called\n");
		device_clean_queue(tcm_hcd);
		retval = 0;
		break;
	default:
		TPD_INFO("Unknown IOCTL operation: 0x%x\n", code);
		return -EINVAL;
	}

	return retval;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
#ifdef HAVE_UNLOCKED_IOCTL
static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int device_ioctl(struct inode *inp, struct file *filp, unsigned int cmd,
			unsigned long arg)
#endif
#endif
{
	int retval = 0;
	struct device_hcd *device_hcd  = NULL;
	struct syna_tcm_data *tcm_info = NULL;

#ifdef EXTERNAL_DEBUG_LOGGING
	struct syna_tcm_ioctl_data ioc_data;
	unsigned char *ptr = NULL;
#endif

	device_hcd = filp->private_data;
	tcm_info = device_hcd->tcm_info;
	mutex_lock(&device_hcd->extif_mutex);

	switch (cmd) {
	case DEVICE_IOC_RESET:
		retval = device_hcd->reset(tcm_info);
		break;

	case DEVICE_IOC_IRQ:
		TPD_INFO("%s:in DEVICE_IOC_IRQ, arg = %lu, device_hcd->flag = %d, device_hcd->irq = %d\n", __func__, arg, device_hcd->flag, device_hcd->irq);
		if (arg == 0) {
			if (device_hcd->flag == 1) {
				disable_irq(device_hcd->irq);
				device_hcd->flag = 0;
			}

		} else if (arg == 1) {
			if (device_hcd->flag == 0) {
				enable_irq(device_hcd->irq);
				device_hcd->flag = 1;
			}
		}

		break;

	case DEVICE_IOC_RAW:
		if (arg == 0) {
			device_hcd->raw_mode = false;

		} else if (arg == 1) {
			device_hcd->raw_mode = true;
		}

		break;

	case DEVICE_IOC_CONCURRENT:
		if (arg == 0) {
			device_hcd->concurrent = false;

		} else if (arg == 1) {
			device_hcd->concurrent = true;
		}

		break;
#ifdef EXTERNAL_DEBUG_LOGGING
	case DEVICE_IOC_GET_FRAME:
	case DEVICE_IOC_SEND_MESSAGE:
	case DEVICE_IOC_SET_REPORTS:
	case DEVICE_IOC_CLEAN_OUT_FRAMES:
		retval = copy_from_user(&ioc_data,
				(void __user *) arg,
				sizeof(struct syna_tcm_ioctl_data));
		if (retval) {
			TPD_INFO("Fail to copy ioctl_data from user space, size:%d\n", retval);
			retval = -EBADE;
			goto exit;
		}

		ptr = ioc_data.buf;

		retval = device_ioctl_dispatch(device_hcd,
				(unsigned int)_IOC_NR(cmd),
				(const unsigned char *)ptr,
				ioc_data.buf_size,
				&ioc_data.data_length);
		if (retval < 0) {
			if (retval != -ETIMEDOUT) {
				TPD_INFO("Fail to do ioctl dispatch, retval:%d\n", retval);
			}
			goto exit;
		}

		retval = copy_to_user((void __user *) arg,
				&ioc_data,
				sizeof(struct syna_tcm_ioctl_data));
		if (retval) {
			TPD_INFO("Fail to update ioctl_data to user space, size:%d\n", retval);
			retval = -EBADE;
			goto exit;
		}
		break;
#endif

	default:
		retval = -ENOTTY;
		break;
	}
#ifdef EXTERNAL_DEBUG_LOGGING
	exit:
#endif
	mutex_unlock(&device_hcd->extif_mutex);

	return retval;
}

static loff_t device_llseek(struct file *filp, loff_t off, int whence)
{
	return -EINVAL;
}

static ssize_t device_read(struct file *filp, char __user *buf,
			   size_t count, loff_t *f_pos)
{
	int retval;
	struct device_hcd *device_hcd  = NULL;
	struct syna_tcm_data *tcm_info = NULL;

	if (count == 0) {
		return 0;
	}

	device_hcd = filp->private_data;
	tcm_info = device_hcd->tcm_info;

	mutex_lock(&device_hcd->extif_mutex);

	LOCK_BUFFER(device_hcd->resp);

	if (device_hcd->raw_mode) {
		retval = syna_tcm_alloc_mem(&device_hcd->resp, count);

		if (retval < 0) {
			TPD_INFO("%s:Failed to allocate memory for device_hcd->resp.buf\n", __func__);
			UNLOCK_BUFFER(device_hcd->resp);
			goto exit;
		}

		retval = device_hcd->read_message(tcm_info,
						  device_hcd->resp.buf,
						  count);

		if (retval < 0) {
			TPD_INFO("%s:Failed to read message\n", __func__);
			UNLOCK_BUFFER(device_hcd->resp);
			goto exit;
		}

	} else {
		if (count != device_hcd->resp.data_length) {
			TPD_INFO("%s:Invalid length information\n", __func__);
			UNLOCK_BUFFER(device_hcd->resp);
			retval = -EINVAL;
			goto exit;
		}
	}

	if (copy_to_user(buf, device_hcd->resp.buf, count)) {
		TPD_INFO("%s:Failed to copy data to user space\n", __func__);
		UNLOCK_BUFFER(device_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	if (!device_hcd->concurrent) {
		goto skip_concurrent;
	}

	if (device_hcd->report_touch == NULL) {
		TPD_INFO("%s:Unable to report touch\n", __func__);
		device_hcd->concurrent = false;
		goto skip_concurrent;
	}

	if (device_hcd->raw_mode) {
		device_capture_touch_report(device_hcd, count);
	}

skip_concurrent:
	UNLOCK_BUFFER(device_hcd->resp);

	retval = count;

exit:
	mutex_unlock(&device_hcd->extif_mutex);

	return retval;
}

static ssize_t device_write(struct file *filp, const char __user *buf,
			    size_t count, loff_t *f_pos)
{
	int retval;
	struct device_hcd *device_hcd  = NULL;
	struct syna_tcm_data *tcm_info = NULL;

	if (count == 0) {
		return 0;
	}

	device_hcd = filp->private_data;
	tcm_info = device_hcd->tcm_info;

	mutex_lock(&device_hcd->extif_mutex);

	LOCK_BUFFER(device_hcd->out);

	retval = syna_tcm_alloc_mem(&device_hcd->out, count == 1 ? count + 1 : count);

	if (retval < 0) {
		TPD_INFO("%s:Failed to allocate memory for device_hcd->out.buf\n", __func__);
		UNLOCK_BUFFER(device_hcd->out);
		goto exit;
	}

	if (copy_from_user(device_hcd->out.buf, buf, count)) {
		TPD_INFO("%s:Failed to copy data from user space\n", __func__);
		UNLOCK_BUFFER(device_hcd->out);
		retval = -EINVAL;
		goto exit;
	}

	LOCK_BUFFER(device_hcd->resp);

	TPD_INFO("%s: cmd 0x%x\n", __func__, device_hcd->out.buf[0]);

	if (device_hcd->raw_mode) {
		retval = device_hcd->write_message(tcm_info,
						   device_hcd->out.buf[0],
						   &device_hcd->out.buf[1],
						   count == 1 ? count : count - 1,
						   NULL,
						   NULL,
						   NULL,
						   0);

	} else {
		mutex_lock(&tcm_info->reset_mutex);
		retval = device_hcd->write_message(tcm_info,
						   device_hcd->out.buf[0],
						   &device_hcd->out.buf[1],
						   count == 1 ? count : count - 1,
						   &device_hcd->resp.buf,
						   &device_hcd->resp.buf_size,
						   &device_hcd->resp.data_length,
						   0);
		mutex_unlock(&tcm_info->reset_mutex);
	}

	if (device_hcd->out.buf[0] == CMD_ERASE_FLASH) {
		msleep(500);
	}

	if (retval < 0) {
		TPD_INFO("%s:Failed to write command 0x%02x\n",
		       __func__, device_hcd->out.buf[0]);
		UNLOCK_BUFFER(device_hcd->resp);
		UNLOCK_BUFFER(device_hcd->out);
		goto exit;
	}

	if (count && device_hcd->out.buf[0] == CMD_SET_TOUCH_REPORT_CONFIG) {
		retval = device_capture_touch_report_config(device_hcd, count);

		if (retval < 0) {
			TPD_INFO("%s:Failed to capture touch report config\n", __func__);
		}
	}

	UNLOCK_BUFFER(device_hcd->out);

	if (device_hcd->raw_mode) {
		retval = count;

	} else {
		retval = device_hcd->resp.data_length;
	}

	UNLOCK_BUFFER(device_hcd->resp);

exit:
	mutex_unlock(&device_hcd->extif_mutex);

	return retval;
}

static int device_open(struct inode *inode, struct file *filp)
{
	int retval;
	struct device_hcd *device_hcd =
		container_of(inode->i_cdev, struct device_hcd, char_dev);

	filp->private_data = device_hcd;

	mutex_lock(&device_hcd->extif_mutex);

	if (device_hcd->ref_count < 1) {
		device_hcd->ref_count++;
		retval = 0;

	} else {
		retval = -EACCES;
	}

	device_hcd->flag = 1;

	mutex_unlock(&device_hcd->extif_mutex);

	return retval;
}

static int device_release(struct inode *inode, struct file *filp)
{
	struct device_hcd *device_hcd =
		container_of(inode->i_cdev, struct device_hcd, char_dev);

	mutex_lock(&device_hcd->extif_mutex);

	if (device_hcd->ref_count) {
		device_hcd->ref_count--;
	}

	mutex_unlock(&device_hcd->extif_mutex);

	return 0;
}

static char *device_devnode(struct device *dev, umode_t *mode)
{
	if (!mode) {
		return NULL;
	}

	*mode = (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

	return kasprintf(GFP_KERNEL, "%s/%s", PLATFORM_DRIVER_NAME,
			 dev_name(dev));
}

static int device_create_class(struct device_hcd *device_hcd)
{
	if (device_hcd->class != NULL) {
		return 0;
	}

	device_hcd->class = class_create(THIS_MODULE, PLATFORM_DRIVER_NAME);

	if (IS_ERR(device_hcd->class)) {
		TPD_INFO("Failed to create class\n");
		return -ENODEV;
	}

	device_hcd->class->devnode = device_devnode;

	return 0;
}

static const struct file_operations device_fops = {
	.owner = THIS_MODULE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	.unlocked_ioctl = device_ioctl,
#else
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = device_ioctl,
#ifdef HAVE_COMPAT_IOCTL
	.compat_ioctl = device_ioctl,
#endif
#else
	.ioctl = device_ioctl,
#endif
#endif
	.llseek = device_llseek,
	.read = device_read,
	.write = device_write,
	.open = device_open,
	.release = device_release,
};

static int device_init(struct syna_tcm_data *tcm_info)
{
	int retval;
	dev_t dev_num;
	struct device_hcd *device_hcd = NULL;

	device_hcd = kzalloc(sizeof(*device_hcd), GFP_KERNEL);

	if (!device_hcd) {
		TPD_INFO("Failed to allocate memory for device_hcd\n");
		return -ENOMEM;
	}
	device_hcd->rmidev_major_num = 0;

	mutex_init(&device_hcd->extif_mutex);
	device_hcd->tp_index = tcm_info->tp_index;

	device_hcd->tcm_info = tcm_info;
	device_hcd->concurrent = CONCURRENT;

	INIT_BUFFER(device_hcd->out, false);
	INIT_BUFFER(device_hcd->resp, false);
	INIT_BUFFER(device_hcd->report, false);

	if (device_hcd->rmidev_major_num) {
		dev_num = MKDEV(device_hcd->rmidev_major_num, device_hcd->tp_index);
		retval = register_chrdev_region(dev_num, 1,
						PLATFORM_DRIVER_NAME);

		if (retval < 0) {
			TPD_INFO("Failed to register char device\n");
			goto err_register_chrdev_region;
		}

	} else {
		retval = alloc_chrdev_region(&dev_num, device_hcd->tp_index, 1,
					     PLATFORM_DRIVER_NAME);

		if (retval < 0) {
			TPD_INFO("Failed to allocate char device\n");
			goto err_alloc_chrdev_region;
		}

		device_hcd->rmidev_major_num = MAJOR(dev_num);
	}

	device_hcd->dev_num = dev_num;

	cdev_init(&device_hcd->char_dev, &device_fops);

	retval = cdev_add(&device_hcd->char_dev, dev_num, 1);

	if (retval < 0) {
		TPD_INFO("Failed to add char device\n");
		goto err_add_chardev;
	}

	retval = device_create_class(device_hcd);

	if (retval < 0) {
		TPD_INFO("Failed to create class\n");
		goto err_create_class;
	}

	device_hcd->device = device_create(device_hcd->class, NULL,
					   device_hcd->dev_num, NULL, CHAR_DEVICE_NAME"%d",
					   MINOR(device_hcd->dev_num));

	if (IS_ERR(device_hcd->device)) {
		TPD_INFO("Failed to create device\n");
		retval = -ENODEV;
		goto err_create_device;
	}

	g_device_hcd[device_hcd->tp_index] = device_hcd;
	return 0;

err_create_device:
	class_destroy(device_hcd->class);

err_create_class:
	cdev_del(&device_hcd->char_dev);

err_add_chardev:
	unregister_chrdev_region(dev_num, 1);

err_alloc_chrdev_region:
err_register_chrdev_region:
	RELEASE_BUFFER(device_hcd->report);
	RELEASE_BUFFER(device_hcd->resp);
	RELEASE_BUFFER(device_hcd->out);

	kfree(g_device_hcd[device_hcd->tp_index]);
	g_device_hcd[device_hcd->tp_index] = NULL;

	return retval;
}

struct device_hcd *syna_remote_device_S3010_init(struct syna_tcm_data *tcm_info)
{
	device_init(tcm_info);

	return g_device_hcd[tcm_info->tp_index];
}

int syna_remote_device_S3910_destory(struct syna_tcm_data *tcm_info)
{
	struct device_hcd *device_hcd = NULL;
	device_hcd = g_device_hcd[tcm_info->tp_index];

	if (!device_hcd) {
		return 0;
	}

	device_destroy(device_hcd->class, device_hcd->dev_num);

	class_destroy(device_hcd->class);

	cdev_del(&device_hcd->char_dev);

	unregister_chrdev_region(device_hcd->dev_num, 1);

	RELEASE_BUFFER(device_hcd->report);
	RELEASE_BUFFER(device_hcd->resp);
	RELEASE_BUFFER(device_hcd->out);

	kfree(device_hcd);
	g_device_hcd[tcm_info->tp_index] = NULL;

	return 0;
}

