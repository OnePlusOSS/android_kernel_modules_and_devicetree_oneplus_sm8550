/**************************************************** *******************************
 * Device simulation and emulation, the function of configuration driver is turned on
 *
 * File: pseudo_sensor.c
 * FileID:
 * Content Summary: Device simulation and emulation,
					realizing the function of data communication with the SensorHub
 * Other Instructions:
 * Current Version: 1.1
 * Author: 80353364
 * Completion Date: 2021-12-01
 *
 * Revision History
 * Modification Version Modifier 	Content
 * 2021-12-01 	 1.0 	 80353364 	function implementation
 * 2021-01-15 	 1.1 	 80353364 	To facilitate the loopback BUF processing of shared memory,
 *									change the relevant address counter to 32bit
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 ******************************************************************************/
#define pr_fmt(fmt) "<debug_kit>" fmt
#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/param.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include "hf_manager.h"
#include "scp.h"

#define DBG                                 pr_debug
#define INF                                 pr_info
#define ERR                                 pr_err

#define SENS_PSEUDO_SENSOR_MEM_ID           SENS_FB_MEM_ID
#define CFG_BUFSIZE                         512
#define CFG_MAX_SENSORS                     16          // Support 16 SENSOR (data channels)
#define CFG_SMEM_SZ                         SZ_64K      // The default memory allocation is 64KB
#define SZ_6K                               0x1800
#define MIN(a, b)                           ((a) <= (b) ? (a) : (b))

#define PSEUDO_THREAD_WAKEUP                0
#define PSEUDO_THREAD_SLEEP                 1

#define DEBUG_KIT_SENSOR_SET_MEM_ADDR       10
#define DEBUG_KIT_SENSOR_UPDATE_MEM         11
#define DEBUG_KIT_SENSOR_SET_DEBUG_KIT_DATA 12

#define DATA_BUFFER_SIZE                    16
#define SMEM_PSEUDO_SENSOR_DEBUG_KIT_CH     14


// Set the default data size, measured in 32KB, pay attention to subtract the space occupied by smem_head
// All allocated space size needs to be aligned with 64Byte, because the data sequencer is aligned with 0x40
// The final default memory allocation is 64KB

static uint16_t g_def_sz[CFG_MAX_SENSORS] = {
	SZ_6K,  SZ_6K,  SZ_6K,  SZ_6K,
	SZ_1K,  SZ_1K,  SZ_1K,  SZ_1K,
	SZ_512, SZ_512, SZ_512, SZ_512,
	SZ_512, SZ_512, SZ_512, SZ_256
};

static char *g_chn_type[CFG_MAX_SENSORS] = {
	"acc1",    "gyro1",    "mag",      "temp",
	"press",   "als1",     "proxi",    "halls",
	"acc2",    "gyro2",    "als2",     "als3",
	"n/a",     "n/a",      "debug",    "halla"
};

#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
typedef struct _sensor_debug_kit_data_t{
	int32_t fault_code;
	int32_t frequency;
	int32_t repeat;
	int32_t test_type;
}sensor_debug_kit_data_t;
#endif

struct psensor_data {
	uint64_t phy_address;
	uint64_t vir_address;
	uint32_t size;
	int16_t enable;
	int16_t cur_chn;
#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
	sensor_debug_kit_data_t debug_kit_data;
#endif
	spinlock_t rw_lock;
	wait_queue_head_t wq;
	struct task_struct *report_task; 			// kernel thread
	struct task_struct *poll_task;
	uint16_t node_type;
	unsigned long wakeup_flag;
	struct hf_client *client;
	int32_t time_left[CFG_MAX_SENSORS]; 		// timeleft
	struct delayed_work time_left_work;			// timer can NOT use mutex
	struct proc_dir_entry   *proc;
};

typedef struct smem_data_seq
{
	float begin;
	float end;
	float delta;
	float rand;
}   smem_data_seq;


// Data generator: supports up to 3 data
typedef struct smem_data_type1 {
	uint32_t magic;                              // magic number
	uint32_t times;                              // data execution times
	uint32_t times_run;                          // Data Remaining Execution Times
	uint32_t type:16;                            // data type, fixed at 1
	uint32_t axis:16;                            // The number of data reported at a time
	smem_data_seq seq[3];
}   smem_data_type1;


typedef struct smem_head {
	uint32_t bp[CFG_MAX_SENSORS];               // The starting offset of the current channel memory
	uint32_t size[CFG_MAX_SENSORS];             // The memory size allocated to the specified SENSOR,
												// loopback buffer used must not time out for this address
												// The unit is sizeof(float) (4)
	uint32_t wp[CFG_MAX_SENSORS];               // offset of the currently written data, sizeof(float)
	uint32_t rp[CFG_MAX_SENSORS];               // offset of the currently read data, sizeof(float)
	// 256B Above
	// The following 128B can be expanded to 256B
	// so 256-32=224B will be reserved for debug_kit
	uint8_t type[CFG_MAX_SENSORS];              // datatype
												// 0: raw data
												// 1: data generator
												// 2: fixed data
	uint8_t start[CFG_MAX_SENSORS];             // data start flag
												// 0: data is not ready
												// 1: data is ready
												// 2: data is being read
												// 3: The data has been fetched
	uint16_t sst[CFG_MAX_SENSORS];              // sensor working status:
												// 0: initial state
												// 1 normally enabled
												// 2: close manually
												// 3: enable error testing function
	uint32_t ecode[CFG_MAX_SENSORS];            // error code feedback
}   smem_head;


static struct psensor_data *g_data = NULL;

#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
int debug_kit_polling_thread(void *arg);
#endif


static int mtk_send_cmd_to_scp(uint8_t cmd, uint32_t *data, int size)
{
	int i;
	int len;
	int ret;
	struct custom_cmd cust_cmd;

#ifdef DEBUG
	char cmd_data[256];
	char *pdata = cmd_data;
	int total = 0;
	uint32_t *pcmd = (uint32_t*)&cust_cmd;
#endif

	memset(&cust_cmd, 0, sizeof(cust_cmd));
	cust_cmd.command = cmd;
	cust_cmd.rx_len = 0;
	cust_cmd.tx_len = size << 2;

	for (i = 0; i < size; i++) {
		cust_cmd.data[i] = data[i];
	}

#ifdef DEBUG
	memset(cmd_data, 0, sizeof(cmd_data));
	len = snprintf(pdata, 256 - total, "mtk_send_cmd_to_scp data:");
	pdata += len;
	total += len;
	for (i = 0; i < (size + 1); i++) {
		len = snprintf(pdata, 256 - total, " %08X", pcmd[i]);
		pdata += len;
		total += len;
	}
	INF("%s", cmd_data);
#endif

	ret = hf_client_custom_cmd(g_data->client, SENSOR_TYPE_DEBUG_KIT, &cust_cmd);

	if (ret < 0) {
		ERR("mtk_send_cmd_to_scp failed: %d", ret);
		return -EINVAL;
	}
	return 0;
}


static int mtk_get_smem(int mem_id)
{
	int ret;
	uint32_t data[3];
	phys_addr_t phys_addr, vir_address, size;

	phys_addr = scp_get_reserve_mem_phys(mem_id);
	vir_address = scp_get_reserve_mem_virt(mem_id);
	size = scp_get_reserve_mem_size(mem_id);

	INF("mtk_get_smem SENS_DEBUG_KIT_MEM_ID = %u\n", mem_id);

	if (size > CFG_SMEM_SZ) {
		phys_addr += (size - CFG_SMEM_SZ);
		vir_address += (size - CFG_SMEM_SZ);
		size = CFG_SMEM_SZ;
	}
	data[0] = 0x00;
	data[1] = phys_addr;
	data[2] = size;

	ret = mtk_send_cmd_to_scp(DEBUG_KIT_SENSOR_SET_MEM_ADDR, data, 3);
	if (ret < 0) {
		return -EINVAL;
	}

	g_data->phy_address = phys_addr;
	g_data->vir_address = vir_address;
	g_data->size = size;

	return 0;
}


static int mtk_put_smem(void)
{
	int ret;
	uint32_t data[3];

	INF("mtk_put_smem SENS_DEBUG_KIT_MEM_ID\n");

	data[0] = 0x01;
	data[1] = 0x00;
	data[2] = 0x00;

	ret = mtk_send_cmd_to_scp(DEBUG_KIT_SENSOR_SET_MEM_ADDR, data, 3);
	if (ret < 0) {
		return -EINVAL;
	}

	g_data->phy_address = 0;
	g_data->vir_address = 0;
	g_data->size = 0;

	return 0;
}


static int mtk_set_chn(uint32_t chn, int enabled)
{
	int ret;
	uint32_t data[3];

	if (enabled) {
		INF("mtk_open_chn 0x%08X\n", chn);
		data[0] = 0x02;
	} else {
		INF("mtk_close_chn 0x%08X\n", chn);
		data[0] = 0x03;
	}

	data[1] = chn;
	data[2] = 0x00;

	ret = mtk_send_cmd_to_scp(DEBUG_KIT_SENSOR_SET_MEM_ADDR, data, 3);
	if (ret < 0) {
		return -EINVAL;
	}

	return 0;
}

static int enable_pseudo_sensor(struct psensor_data *pseudo_sensor_cxt)
{
	int ret = 0;
	struct hf_manager_cmd cmd;
	struct hf_manager_batch *batch = NULL;
	pseudo_sensor_cxt->client = hf_client_create();
	if (!pseudo_sensor_cxt->client) {
		ERR("hf_client_create fail\n");
		return -ENOMEM;
	}

	ret = hf_client_find_sensor(pseudo_sensor_cxt->client, SENSOR_TYPE_DEBUG_KIT);
	if (ret < 0) {
		ERR("hf_client_find_sensor debug kit sensor fail\n");
		return -EINVAL;
	}

#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
	if (!pseudo_sensor_cxt->poll_task) {
		pseudo_sensor_cxt->poll_task = kthread_run(debug_kit_polling_thread, (void *)pseudo_sensor_cxt,
								  "debug_kit_polling_thread");
		if (IS_ERR(pseudo_sensor_cxt->poll_task)) {
			ERR("poll debug kit thread create failed\n");
			return -ENOMEM;
		}
	}
#endif

	memset(&cmd, 0, sizeof(cmd));
	cmd.sensor_type = SENSOR_TYPE_DEBUG_KIT;
	cmd.action = HF_MANAGER_SENSOR_ENABLE;
	batch = (struct hf_manager_batch *)cmd.data;
	batch->delay = 20000000;
	batch->latency = 0;
	ret = hf_client_control_sensor(pseudo_sensor_cxt->client, &cmd);
	if (ret < 0) {
		ERR("hf_client_control_sensor debug kit sensor fail\n");
		return -EINVAL;
	}

	return 0;
}

#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
int debug_kit_polling_thread(void *arg)
{
	struct psensor_data *pseudo_sensor_cxt = (struct psensor_data *)arg;
	struct hf_client *client = pseudo_sensor_cxt->client;
	struct hf_manager_event data[4];
	uint16_t adsp_event_counts = 0;
	int size = 0, i = 0;

	if (!client) {
		return -EINVAL;
	}
	while (!kthread_should_stop()) {
		memset(data, 0, sizeof(data));

		size = hf_client_poll_sensor(client, data, ARRAY_SIZE(data));
		if (size < 0)
			continue;
		for(i = 0; i < size; ++i) {
			if (data[i].sensor_type == SENSOR_TYPE_DEBUG_KIT
				&& data[i].action == DATA_ACTION) {
				adsp_event_counts = data[i].word[0];
				INF("recv data, state = %d, report_count = %d\n",
						adsp_event_counts, data[i].word[1]);
			}
		}
	}
	return 0;
}

static size_t update_debug_kit_data(struct psensor_data *pseudo_sensor_cxt)
{
	struct custom_cmd cust_cmd;
	size_t data_size = sizeof(pseudo_sensor_cxt->debug_kit_data);
	int ret;
	memset(&cust_cmd, 0, sizeof(cust_cmd));
	cust_cmd.command = DEBUG_KIT_SENSOR_SET_DEBUG_KIT_DATA;
	cust_cmd.rx_len = 0;
	cust_cmd.tx_len = data_size;
	cust_cmd.data[0] = pseudo_sensor_cxt->debug_kit_data.fault_code;
	cust_cmd.data[1] = pseudo_sensor_cxt->debug_kit_data.frequency;
	cust_cmd.data[2] = pseudo_sensor_cxt->debug_kit_data.repeat;
	cust_cmd.data[3] = pseudo_sensor_cxt->debug_kit_data.test_type;

	INF("update_debug_kit_data, cmd = 0x%x, data_size = %d, test_type=%d, fault_code=%d, frequency=%d, repeat=%d!\n",
		cust_cmd.command, data_size, pseudo_sensor_cxt->debug_kit_data.test_type, pseudo_sensor_cxt->debug_kit_data.fault_code,
		pseudo_sensor_cxt->debug_kit_data.frequency, pseudo_sensor_cxt->debug_kit_data.repeat);
	ret = hf_client_custom_cmd(pseudo_sensor_cxt->client, SENSOR_TYPE_DEBUG_KIT, &cust_cmd);
	if (ret < 0) {
		ERR("hf_client_custom_cmd debug kit sensor fail\n");
		data_size = 0;
	}

	return data_size;
}

static size_t sensor_debug_kit_update_data(struct psensor_data *pseudo_sensor_cxt)
{
#if 1
	return update_debug_kit_data(pseudo_sensor_cxt);
#else
	char *p;
	size_t count = sizeof(g_data->debug_kit_data);
	smem_head *smem = (smem_head*)g_data->vir_address;

	if(g_data->vir_address == 0) {
		ERR("vir_address not bind yet, do bind first!\n");
		return -1;
	}
	p = (char*)g_data->vir_address;
	p += smem->bp[SMEM_PSEUDO_SENSOR_DEBUG_KIT_CH] << 2;

	memcpy((void *)p, (void *)&(g_data->debug_kit_data), count);
	smem->start[SMEM_PSEUDO_SENSOR_DEBUG_KIT_CH] = 1;
	INF("smem ready, test_type=%d, fault_code=%d, frequency=%d, repeat=%d!\n",
		g_data->debug_kit_data.test_type, g_data->debug_kit_data.fault_code,
		g_data->debug_kit_data.frequency, g_data->debug_kit_data.repeat);
	return count;
#endif
}

static ssize_t proc_fault_code_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[DATA_BUFFER_SIZE] = {0};
	int len = 0;

	if (*ppos > 0 || count < DATA_BUFFER_SIZE)
		return 0;

	len = snprintf(data, DATA_BUFFER_SIZE, "%d\n", g_data->debug_kit_data.fault_code);
	if(copy_to_user(buf, data, len))
		return -EFAULT;

	*ppos = len;
	return len;
}

static ssize_t proc_fault_code_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len = 0;
	int cur_fault_code = 0;
	char data[DATA_BUFFER_SIZE] = {0};

	memset(data, 0, sizeof(data));
	len = MIN(count, DATA_BUFFER_SIZE);
	if (copy_from_user(data, buf, len)) {
		ERR("fault code val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &cur_fault_code)) {
		INF("fault code val input: %d\n", cur_fault_code);
	} else {
		ERR("fault code val input invalid\n");
		return -EINVAL;
	}

	if (cur_fault_code < 0 || cur_fault_code > 10002) {
		ERR("fault code val out range\n");
		return -EINVAL;
	}

	g_data->debug_kit_data.fault_code = cur_fault_code;
	return count;
}

static ssize_t proc_frequency_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[DATA_BUFFER_SIZE] = {0};
	int len = 0;

	if (*ppos > 0 || count < DATA_BUFFER_SIZE)
		return 0;

	len = snprintf(data, DATA_BUFFER_SIZE, "%d\n", g_data->debug_kit_data.frequency);
	if(copy_to_user(buf, data, len))
		return -EFAULT;

	*ppos = len;
	return len;
}

static ssize_t proc_frequency_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len = 0;
	int cur_frequency = 0;
	char data[DATA_BUFFER_SIZE] = {0};

	memset(data, 0, sizeof(data));
	len = MIN(count, DATA_BUFFER_SIZE);
	if (copy_from_user(data, buf, len)) {
		ERR("frequency val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &cur_frequency)) {
		INF("frequency val input: %d\n", cur_frequency);
	} else {
		ERR("frequency val input invalid\n");
		return -EINVAL;
	}

	g_data->debug_kit_data.frequency = cur_frequency;
	return count;
}

static ssize_t proc_repeat_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[DATA_BUFFER_SIZE] = {0};
	int len = 0;

	if (*ppos > 0 || count < DATA_BUFFER_SIZE)
		return 0;

	len = snprintf(data, DATA_BUFFER_SIZE, "%d\n", g_data->debug_kit_data.repeat);
	if(copy_to_user(buf, data, len))
		return -EFAULT;

	*ppos = len;
	return len;
}

static ssize_t proc_repeat_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len = 0;
	int cur_repeat = 0;
	char data[DATA_BUFFER_SIZE] = {0};

	memset(data, 0, sizeof(data));
	len = MIN(count, DATA_BUFFER_SIZE);
	if (copy_from_user(data, buf, len)) {
		ERR("repeat val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &cur_repeat)) {
		INF("repeat val input: %d\n", cur_repeat);
	} else {
		ERR("repeat val input invalid\n");
		return -EINVAL;
	}

	g_data->debug_kit_data.repeat = cur_repeat;
	return count;
}

static ssize_t proc_test_type_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[DATA_BUFFER_SIZE] = {0};
	int len = 0;

	if (*ppos > 0 || count < DATA_BUFFER_SIZE)
		return 0;

	len = snprintf(data, DATA_BUFFER_SIZE, "%d\n", g_data->debug_kit_data.test_type);
	if(copy_to_user(buf, data, len))
		return -EFAULT;

	*ppos = len;
	return len;
}
static ssize_t proc_test_type_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len = 0;
	int cur_test_type = 0;
	char data[DATA_BUFFER_SIZE] = {0};

	memset(data, 0, sizeof(data));
	len = MIN(count, DATA_BUFFER_SIZE);
	if (copy_from_user(data, buf, len)) {
		ERR("test type val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &cur_test_type)) {
		INF("test type val input: %d\n", cur_test_type);
	} else {
		ERR("test type val input invalid\n");
		return -EINVAL;
	}

	g_data->debug_kit_data.test_type = cur_test_type;
	return count;
}

static ssize_t proc_sync_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len = 0;
	int sync_flag = 0;
	char data[DATA_BUFFER_SIZE] = {0};

	memset(data, 0, sizeof(data));
	len = MIN(count, DATA_BUFFER_SIZE);
	if (copy_from_user(data, buf, len)) {
		ERR("test type val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &sync_flag)) {
		INF("sync val input: %d\n", sync_flag);
	} else {
		ERR("sync val input invalid\n");
		return -EINVAL;
	}

	if (sync_flag == 1) {
		sensor_debug_kit_update_data(g_data);
	}
	return count;
}
#endif


static void psensor_timer_left(struct work_struct *dwork)
{
	int i;
	int ret;
	int cnt = 0;
	uint32_t chns = 0;
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (0 == g_data->vir_address) {
		return;
	}

	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		if (g_data->time_left[i] == 0)  {
			continue;
		} else {
			g_data->time_left[i]--;
			cnt++;
		}

		if (g_data->time_left[i] == 0)  {
			smem->start[i] = 1;
			chns |= 1U << i;
			cnt--;
		}
	}

	if (chns) {
		ret = mtk_set_chn(chns, 1);
		if (ret < 0) {
			ERR("mtk set chn failed: 0x%08X\n", chns);
		}
	}

	if (cnt > 0) {
		schedule_delayed_work(&g_data->time_left_work, msecs_to_jiffies(1000));
	}
}


// Initialize the shared memory space
// According to the total size and the length of the sz array, the space is divided into smem
// Note that size is measured in bytes, sz is also measured in bytes, but in smem it is measured in float
static int psensor_smem_init(smem_head *smem, uint32_t size, uint16_t *sz)
{
	int i;
	uint32_t offset = sizeof(smem_head);
	uint32_t total_sz = 0;

	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		total_sz += sz[i];
	}

	if (0 == total_sz) {
		ERR("allocate size total is 0, failed\n");
		return -EPERM;
	}

	if (total_sz > size - offset) {
		ERR("allocate size(%X) more than free size %X\n", total_sz, size - offset);
		return -ENOMEM;
	}

	// Why divide by 4? It is because of conversion to float number
	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		smem->size[i] = sz[i] >> 2;
		smem->bp[i] = offset >> 2;
		smem->wp[i] = 0;
		smem->rp[i] = 0;
		offset = offset + sz[i];
		smem->type[i] = 0;
		smem->start[i] = 0;
	}
	return 0;
}


// Copy the user mode memory to the shared memory path, wp == rp is not allowed when copying
// smem: start address of shared memory
// chn: shared memory channel number
// buf: user space buf
// count: bytes of data transmitted by user space, note that the data is 4Byte aligned (float32)
static int psensor_smem_put(smem_head *smem, int chn, const char __user *buf, size_t count)
{
	char *p;
	uint32_t len = 0;
	uint32_t size = count >> 2;           // 数据为  4Byte 的整数倍
	size = MIN(size, smem->size[chn] - smem->wp[chn] + smem->rp[chn]);
	len = MIN(size, smem->size[chn] - (smem->wp[chn] % smem->size[chn]));

	p = (char*)g_data->vir_address;
	p += smem->bp[chn] << 2;
	p += (smem->wp[chn] % smem->size[chn]) << 2;

	if (copy_from_user(p, buf, len << 2)) {
		ERR("ecode val copy from user error\n");
		return -EIO;
	}

	if (size - len > 0) {
		p = (char*)g_data->vir_address;
		p += smem->bp[chn] << 2;
		if (copy_from_user(p, buf + len, (size - len) << 2)) {
			ERR("ecode val copy from user error\n");
			return -EIO;
		}
	}

	smem->wp[chn] += size;
	return size << 2;
}


static ssize_t proc_data_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[2048];
	int i, len, total = 0;
	char *d = (char *)&data[0];
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (*ppos > 0 || count < 2048) {
		return 0;
	}

	if (0 == g_data->vir_address) {
		total = snprintf(d, 2048, "memory invalid, plese enable first\n");
		ERR("memory invalid, plese enable first\n");
		if (copy_to_user(buf, data, total)) {
			return -EFAULT;
		}
		*ppos = total;
		return total;
	}

	len = snprintf(d, 2048-total, "addr\t\t: %016lX\n", g_data->vir_address);
	d += len;
	total += len;
	len = snprintf(d, 2048-total, "size\t\t: %X\n", g_data->size);
	d += len;
	total += len;
	len = snprintf(d, 2048-total, "current chn\t: %d\n", g_data->cur_chn);
	d += len;
	total += len;
	len = snprintf(d, 2048-total,
				   "chn sst name\toff.f\tsize.f\tdtype\tstart\twp.f\trp.f\tecode\ttimeleft\n");
	d += len;
	total += len;

	for(i = 0; i < CFG_MAX_SENSORS; i++) {
		if (i == g_data->cur_chn) {
			len = snprintf(d, 2048-total,
						   "%02d< %-4X%s\t%X\t%X\t%X\t%X\t%X\t%X\t%X\t%d\n",
							i, smem->sst[i], g_chn_type[i], smem->bp[i],
							smem->size[i], smem->type[i], smem->start[i],
							smem->wp[i],
							smem->rp[i], smem->ecode[i], g_data->time_left[i]);
		} else {
			len = snprintf(d, 2048-total,
						   "%02d  %-4X%s\t%X\t%X\t%X\t%X\t%X\t%X\t%X\t%d\n",
						   i, smem->sst[i], g_chn_type[i], smem->bp[i],
						   smem->size[i], smem->type[i], smem->start[i],
						   smem->wp[i], smem->rp[i], smem->ecode[i],
						   g_data->time_left[i]);

		}
		d += len;
		total += len;
	}

	if (copy_to_user(buf, data, total)) {
		return -EFAULT;
	}

	*ppos = total;
	return total;
}


static uint32_t IEEE754_INT_TO_FLOAT(int v)
{
	uint32_t data = 0;
	uint32_t e = 23;

	if (v == 0) {
		return 0x0;
	}
	if (v == -1) {
		return 0xBF800000;
	}
	if (v == 1) {
		return 0x3F800000;
	}

	if (v < 0) {
		data = 1 << 31;
	}
	v = v < 0 ? -v: v;

	if (v >= (1<<23)) {
		v = (1 << 23);
	}

	while ((v & (1 << 23)) == 0x0) {
		v = v << 1;
		e--;
	}
	v = v & 0x7FFFFF;
	e = e + 127;
	data = data | e << 23 | v;

	return data;
}


static ssize_t proc_data_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char data[128];
	int v[8];
	uint32_t *p;                 // Actually float
	int i;
	int ret = 0;
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;
	int len = MIN(count, 128);

	if (0 == g_data->vir_address) {
		ERR("no share memory, plese bind share memory first\n");
		return -ENOMEM;
	}

	if (2 == smem->type[chn]) {
		if (smem->size[chn] <= 16) {
			return -ENOMEM;
		}

		memset(data, 0, sizeof(data));
		memset(v, 0, sizeof(v));

		if (copy_from_user(data, buf, len)) {
			ERR("sizes val copy from user error\n");
			return -EIO;
		}
		// sscanf in the kernel code does not support float reading, so only integer data can be used
		ret = sscanf(data,
					 "%d,%d,%d,%d,%d,%d,%d,%d",
					 &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]);
		INF("get %d values\n", ret);

		smem->rp[chn] = 0;
		smem->wp[chn] = ret;
		p = (uint32_t*)g_data->vir_address;       // 其实是 float
		p += smem->bp[chn];
		for (i = 0; i < ret; i++) {
			*p++ = IEEE754_INT_TO_FLOAT(v[i]);
		}

		ret = count;
	} else {
		ret = psensor_smem_put(smem, chn, buf, count);
	}
	if (ret < 0) {
		ERR("data copy failed, ret = %d\n", ret);
	}

	return ret;
}


static ssize_t proc_sizes_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[512];
	int chn, i, len = 0;
	char *d = (char *)&data[0];

	if (*ppos > 0 || count < sizeof(data)) {
		return 0;
	}

	for(chn = 0; chn < CFG_MAX_SENSORS; chn++) {
		i = snprintf(d, 512-len, "chn%2d :\t%04X\n", chn, g_def_sz[chn]);
		d += i;
		len += i;
	}

	if (copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


static ssize_t proc_sizes_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int ret;
	int i;
	char data[128];
	uint16_t s[CFG_MAX_SENSORS];
	int len = MIN(count, 128);

	if (0 == g_data->vir_address) {
		ERR("no share memory, plese bind share memory first\n");
		return -ENOMEM;
	}

	memset(data, 0, sizeof(data));
	if (copy_from_user(data, buf, len)) {
		ERR("sizes val copy from user error\n");
		return -EIO;
	}

	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		s[i] = 0;
	}

	// Read data, note that the data unit is KB
	ret = sscanf(data,
				 "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
				 &s[0], &s[1], &s[2], &s[3], &s[4], &s[5], &s[6], &s[7],
				 &s[8], &s[9], &s[10],&s[11],&s[12],&s[13],&s[14],&s[15]);

	INF("get %d valid data\n", ret);
	for (i = 0; i < ret; i++) {
		s[i] <<= 10;
	}

	ret = mtk_set_chn(-1U, 0);
	if (ret < 0) {
		ERR("mtk set chn failed: 0xFFFFFFFF\n");
		return -EIO;
	}

	// Try to allocate space for each channel and return the allocation result
	ret = psensor_smem_init((smem_head*)g_data->vir_address, g_data->size, s);

	// If the allocation is successful, write the data to g_def_sz.
	// In fact, there is no problem if you don’t write it, it’s just for possible follow-up operations
	if (ret >= 0) {
		for (i = 0; i < CFG_MAX_SENSORS; i++) {
			g_def_sz[i] = s[i];
		}
	} else {
		return ret;
	}

	return count;
}


static ssize_t proc_datareset_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char *p;
	char data[16];
	int len;
	int ret;
	int reset = 0;
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (0 == g_data->vir_address) {
		ERR("invalid memory address\n");
		return -EIO;
	}

	memset(data, 0, sizeof(data));

	len = MIN(count, 16);

	if (copy_from_user(data, buf, len)) {
		ERR("datareset val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &reset)) {
		INF("datareset val input: %d\n", reset);
	} else {
		ERR("datareset val input invalid\n");
		return -EINVAL;
	}

	if (1 != reset) {
		return count;
	}

	p = (char*)g_data->vir_address;
	p += smem->bp[chn] << 2;
	memset(p, 0, smem->size[chn] << 2);
	smem->wp[chn] = 0;
	smem->rp[chn] = 0;
	smem->type[chn] = 0;
	smem->start[chn] = 0;

	ret = mtk_set_chn(1UL << chn, 0);
	if (ret < 0) {
		ERR("mtk set chn failed: 0x%08X\n", 1UL << chn);
		return -EIO;
	}
	return count;
}


static ssize_t proc_timeleft_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[512];
	int chn, i, len = 0;
	char *d = (char *)&data[0];

	if (*ppos > 0 || count < 512) {
		return 0;
	}

	for(chn = 0; chn < CFG_MAX_SENSORS; chn++) {
		i = snprintf(d, 512-len, "chn%2d timeleft\t: %d\n", chn, g_data->time_left[chn]);
		d += i;
		len += i;
	}

	if (copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}

// Write format 1: Directly write data means use the current channel
// Write format 2: "0xFF,4" > timeleft The channel number specified by the previous one,
// and the number of seconds after the next one is enabled
static ssize_t proc_timeleft_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int i;
	int ret;
	char data[32];
	int timeleft = 0;
	u32 chns = 0;
	smem_head *smem = NULL;
	int len = MIN(count, sizeof(data));

	memset(data, 0, sizeof(data));

	if (copy_from_user(data, buf, len)) {
		ERR("timeleft val copy from user error\n");
		return -EIO;
	}

	if (2 == sscanf(data, "0x%X,%d", &chns, &timeleft)) {
		INF("timeleft val input: chn: 0x%X, %d\n", chns, timeleft);
	} else if (2 == sscanf(data, "0x%X %d", &chns, &timeleft)) {
		INF("timeleft val input: chn: 0x%X, %d\n", chns, timeleft);
	} else if (2 == sscanf(data, "0x%X, %d", &chns, &timeleft)) {
		INF("timeleft val input: chn: 0x%X, %d\n", chns, timeleft);
	} else if (1 == sscanf(data, "%d", &timeleft)) {
		INF("timeleft val input: %d\n", timeleft);
	} else {
		ERR("input invalid, it like: echo \"0xFF 10\" > timeleft\n");
		return -EINVAL;
	}

	if (0 == chns) {
		chns = 1U << g_data->cur_chn;
	}

	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		if (chns & (1U << i)) {
			g_data->time_left[i] = timeleft;
		}
	}

	if (0 == timeleft && g_data->vir_address) {
		// start immediately
		smem = (smem_head*)g_data->vir_address;
		for (i = 0; i < CFG_MAX_SENSORS; i++) {
			if (chns & (1U << i)) {
				smem->start[i] = 1;
			}
		}
		ret = mtk_set_chn(chns, 1);
		if (ret < 0) {
			ERR("mtk set chn failed: 0x%08X\n", chns);
		}
	} else {
		schedule_delayed_work(&g_data->time_left_work, msecs_to_jiffies(1000));
	}

	return count;
}

static ssize_t proc_sst_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	static char *mod_txt[4] = {"init", "valid", "invalid", "error"};

	char data[512];
	int i, len = 0;
	char *d = (char *)&data[0];
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (*ppos > 0 || count < 512) {
		return 0;
	}

	i = snprintf(d, 512-len, "chn\tstatus\tmode\n");
	d += i;
	len += i;

	for(chn = 0; chn < CFG_MAX_SENSORS; chn++) {
		i = snprintf(d, 512-len, "%d\t%u\t%s\n", chn, smem->sst[chn], mod_txt[smem->sst[chn]]);
		d += i;
		len += i;
	}

	if (copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


static ssize_t proc_sst_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char data[32];
	int st = 0;
	smem_head *smem = (smem_head*)g_data->vir_address;
	int len = MIN(count, sizeof(data));

	memset(data, 0, sizeof(data));

	if (copy_from_user(data, buf, len)) {
		ERR("timeleft val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &st)) {
		INF("sensor status val input: %d\n", st);
	} else {
		ERR("input invalid, it like: echo 1 > sst\n");
		return -EINVAL;
	}

// Note that the value setting range of sst is:
// 0: SENSOR does not work (initial state)
// 1: SENSOR works in normal mode
// 2: SENSOR is manually closed
// 3: SENSOR works in exception simulation mode, and the ecode value only takes effect at this time.
	if (0 <= st && st < 4) {
		smem->sst[g_data->cur_chn] = st;
	} else {
		ERR("input value verify invalid\n");
		return -EINVAL;
	}
	return count;
}

static ssize_t proc_ecode_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[512];
	int i, len = 0;
	char *d = (char *)&data[0];
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (*ppos > 0 || count < 512) {
		return 0;
	}

	for(chn = 0; chn < CFG_MAX_SENSORS; chn++) {
		i = snprintf(d, 512-len,
					"chn%2d ecode\t: %08X(%u)\n",
					chn, smem->ecode[chn], smem->ecode[chn]);
		d += i;
		len += i;
	}

	if (copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


static ssize_t proc_ecode_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char data[32];
	int ecode = 0;
	smem_head *smem = (smem_head*)g_data->vir_address;
	int len = MIN(count, sizeof(data));

	memset(data, 0, sizeof(data));

	if (copy_from_user(data, buf, len)) {
		ERR("ecode val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &ecode)) {
		INF("ecode val input: %d\n", ecode);
	} else {
		ERR("input invalid, it like: echo 1 > ecode\n");
		return -EINVAL;
	}

	if (ecode >= 0) {
		smem->ecode[g_data->cur_chn] = ecode;
	} else {
		ERR("input value verify invalid\n");
		return -EINVAL;
	}
	return count;
}

static ssize_t proc_datatype_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len;
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE,"%u\n", smem->type[chn]);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}

static ssize_t proc_datatype_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char data[16];
	int len;
	int type = 0;
	int chn = (int)g_data->cur_chn;
	smem_head *smem = (smem_head*)g_data->vir_address;

	memset(data, 0, sizeof(data));

	len = MIN(count, 16);

	if (copy_from_user(data, buf, len)) {
		ERR("datatype val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &type)) {
		INF("datatype val input: %d\n", type);
	} else {
		ERR("datatype val input invalid\n");
		return -EINVAL;
	}

	if (type < 0 || type > 2) {
		ERR("datatype val out range\n");
		return -EINVAL;
	}

	smem->type[chn] = type;
	return count;
}

static ssize_t proc_phy_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "0x%016lX\n", g_data->phy_address);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}

static ssize_t proc_vir_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "0x%016lX\n", g_data->vir_address);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}

static ssize_t proc_freemem_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int i;
	int len = 0;
	char *d = &data[0];

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(d, CFG_BUFSIZE - len, "id\tphy\t \t \tsize\n");

	for (i = 0; i < 16; i++) {
		phys_addr_t phy = scp_get_reserve_mem_phys(i);
		phys_addr_t sz = scp_get_reserve_mem_size(i);
		d = &data[len];
		len += snprintf(d, CFG_BUFSIZE - len, "%d\t%016lX\t%X\n", i, phy, sz);
	}

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


static ssize_t proc_current_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "%d\n", g_data->cur_chn);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}

static ssize_t proc_current_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int len, cur_chn;
	char data[16];

	memset(data, 0, sizeof(data));

	len = MIN(count, 16);

	if (copy_from_user(data, buf, len)) {
		ERR("current val copy from user error\n");
		return -EIO;
	}

	if (1 == sscanf(data, "%d", &cur_chn)) {
		INF("current val input: %d\n", cur_chn);
	} else {
		ERR("current val input invalid\n");
		return -EINVAL;
	}

	if (cur_chn < 0 || cur_chn > (CFG_MAX_SENSORS - 1)) {
		ERR("current val out range\n");
		return -EINVAL;
	}

	g_data->cur_chn = cur_chn;
	return count;
}


static ssize_t proc_bind_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "%d\n", g_data->enable);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


static ssize_t proc_bind_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	void *vmem;
	int len;
	int ret;
	size_t size = 0;
	int enable = 0;
	char data[16];

	memset(data, 0, sizeof(data));

/*     if (0 != g_data->vir_address) {
		ERR("alread bind\n");
		return -EBUSY;
	} */

	len = MIN(count, 16);

	if (copy_from_user(data, buf, len)) {
		ERR("read bind val input error\n");
		return -EIO;
	}

	if (1 != sscanf(data, "%d", &enable)) {
		ERR("invalid input\n");
		return -EINVAL;
	}

	enable = enable & 0x0F;

	switch(enable) {
		case 0x00:
			// Disable shared memory
			ret = mtk_put_smem();
			if (ret < 0) {
				ERR("put share memory failed\n");
				return ret;
			}

			// TODO: disconnect client
			break;
		case 0x01:
			if (0 != g_data->vir_address) {
				ERR("alread bind\n");
				return -EBUSY;
			}

			ret = enable_pseudo_sensor(g_data);
			if (ret < 0) {
				ERR("enable debugkit sensor failed\n");
				return ret;
			}

			ret = mtk_get_smem(SENS_PSEUDO_SENSOR_MEM_ID);
			if (ret < 0) {
				ERR("get share memory failed\n");
				return ret;
			}

			size = (size_t)g_data->size;
			vmem = (void*)g_data->vir_address;
			INF("mtk share memory info: phy: %016lX, vir: %016lX, size: %08lX\n",
				g_data->phy_address, g_data->vir_address, size);

			if ((NULL == vmem) || (0 == size)) {
				ERR("memory malloc failed\n");
				return -ENOMEM;
			}

			memset(vmem, 0, size);
			psensor_smem_init((smem_head*)g_data->vir_address, g_data->size, g_def_sz);
			INF("memory malloc 0x%XByte at %016lX(physical: %016lX)\n",
				size, g_data->vir_address, g_data->phy_address);
			break;
		default:
			INF("operation number not support: %d\n", enable);
	}

	g_data->enable = enable;
	return count;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0))

static const struct file_operations proc_data_fops =
{
	.read = proc_data_r,
	.write = proc_data_w,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static const struct file_operations proc_sizes_fops =
{
	.read = proc_sizes_r,
	.write = proc_sizes_w,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static const struct file_operations proc_datareset_fops =
{
	.write = proc_datareset_w,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static const struct file_operations proc_timeleft_fops =
{
	.owner = THIS_MODULE,
	.read = proc_timeleft_r,
	.write = proc_timeleft_w,
	.open = simple_open,
};


static const struct file_operations proc_sst_fops =
{
	.owner = THIS_MODULE,
	.read = proc_sst_r,
	.write = proc_sst_w,
	.open = simple_open,
};


static const struct file_operations proc_ecode_fops =
{
	.owner = THIS_MODULE,
	.read = proc_ecode_r,
	.write = proc_ecode_w,
	.open = simple_open,
};


static const struct file_operations proc_datatype_fops =
{
	.owner = THIS_MODULE,
	.read = proc_datatype_r,
	.write = proc_datatype_w,
	.open = simple_open,
};


static const struct file_operations proc_phy_fops =
{
	.owner = THIS_MODULE,
	.read = proc_phy_r,
	.open = simple_open,
};


static const struct file_operations proc_vir_fops =
{
	.owner = THIS_MODULE,
	.read = proc_vir_r,
	.open = simple_open,
};


static const struct file_operations proc_freemem_fops =
{
	.owner = THIS_MODULE,
	.read = proc_freemem_r,
	.open = simple_open,
};


static const struct file_operations proc_current_fops =
{
	.read = proc_current_r,
	.write = proc_current_w,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static const struct file_operations proc_bind_fops =
{
	.read = proc_bind_r,
	.write = proc_bind_w,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
static const struct file_operations proc_fault_code_fops =
{
	.read       = proc_fault_code_r,
	.write      = proc_fault_code_w,
	.open       = simple_open,
	.owner      = THIS_MODULE,
};
static const struct file_operations proc_frequency_fops =
{
	.read       = proc_frequency_r,
	.write      = proc_frequency_w,
	.open       = simple_open,
	.owner      = THIS_MODULE,
};
static const struct file_operations proc_repeat_fops =
{
	.read       = proc_repeat_r,
	.write      = proc_repeat_w,
	.open       = simple_open,
	.owner      = THIS_MODULE,
};
static const struct file_operations proc_test_type_fops =
{
	.read       = proc_test_type_r,
	.write      = proc_test_type_w,
	.open       = simple_open,
	.owner      = THIS_MODULE,
};
static const struct file_operations proc_sync_fops =
{
	.write      = proc_sync_w,
	.open       = simple_open,
	.owner      = THIS_MODULE,
};
#endif
#else

static const struct proc_ops proc_data_fops =
{
	.proc_read = proc_data_r,
	.proc_write = proc_data_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_sizes_fops =
{
	.proc_read = proc_sizes_r,
	.proc_write = proc_sizes_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_datareset_fops =
{
	.proc_write = proc_datareset_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_timeleft_fops =
{
	.proc_read = proc_timeleft_r,
	.proc_write = proc_timeleft_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_sst_fops =
{
	.proc_read = proc_sst_r,
	.proc_write = proc_sst_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_ecode_fops =
{
	.proc_read = proc_ecode_r,
	.proc_write = proc_ecode_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_datatype_fops =
{
	.proc_read = proc_datatype_r,
	.proc_write = proc_datatype_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_phy_fops =
{
	.proc_read = proc_phy_r,
	.proc_open = simple_open,
};
static const struct proc_ops proc_vir_fops =
{
	.proc_read = proc_vir_r,
	.proc_open = simple_open,
};


static const struct proc_ops proc_freemem_fops =
{
	.proc_read = proc_freemem_r,
	.proc_open = simple_open,
};


static const struct proc_ops proc_current_fops =
{
	.proc_read = proc_current_r,
	.proc_write = proc_current_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_bind_fops =
{
	.proc_read = proc_bind_r,
	.proc_write = proc_bind_w,
	.proc_open = simple_open,
};
#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
static const struct proc_ops proc_fault_code_fops =
{
	.proc_read       = proc_fault_code_r,
	.proc_write      = proc_fault_code_w,
	.proc_open       = simple_open,
};
static const struct proc_ops proc_frequency_fops =
{
	.proc_read       = proc_frequency_r,
	.proc_write      = proc_frequency_w,
	.proc_open       = simple_open,
};
static const struct proc_ops proc_repeat_fops =
{
	.proc_read       = proc_repeat_r,
	.proc_write      = proc_repeat_w,
	.proc_open       = simple_open,
};
static const struct proc_ops proc_test_type_fops =
{
	.proc_read       = proc_test_type_r,
	.proc_write      = proc_test_type_w,
	.proc_open       = simple_open,
};
static const struct proc_ops proc_sync_fops =
{
	.proc_write      = proc_sync_w,
	.proc_open       = simple_open,
};

#endif
#endif


static int sensor_pseudo_report_thread(void *arg)
{
	int ret = 0;
	struct psensor_data *pseudo_sensor_cxt = (struct psensor_data *)arg;
	uint16_t node_type = 0;
	INF("sensor_pseudo_report_thread step1!\n");

	while (!kthread_should_stop()) {
		wait_event_interruptible(pseudo_sensor_cxt->wq, test_bit(PSEUDO_THREAD_WAKEUP,
				(unsigned long *)&pseudo_sensor_cxt->wakeup_flag));

		clear_bit(PSEUDO_THREAD_WAKEUP, (unsigned long *)&pseudo_sensor_cxt->wakeup_flag);
		set_bit(PSEUDO_THREAD_SLEEP, (unsigned long *)&pseudo_sensor_cxt->wakeup_flag);
		spin_lock(&pseudo_sensor_cxt->rw_lock);
		node_type = pseudo_sensor_cxt->node_type;
		spin_unlock(&pseudo_sensor_cxt->rw_lock);
	}

	INF("step2 ret =%s\n", ret);
	return ret;
}

static int __init pseduo_sensor_init(void)
{
	struct proc_dir_entry *dir;
	int err = 0;

	if (g_data != NULL) {
		ERR("driver already exist\n");
		return -EBUSY;
	} else {
		g_data = (struct psensor_data *)kmalloc(sizeof(struct psensor_data), GFP_KERNEL);
		if (NULL == g_data) {
			ERR("driver malloc memory failed\n");
			return -ENOMEM;
		}
	}

	memset(g_data, 0, sizeof(struct psensor_data));

	spin_lock_init(&g_data->rw_lock);
	init_waitqueue_head(&g_data->wq);
	set_bit(PSEUDO_THREAD_SLEEP, (unsigned long *)&g_data->wakeup_flag);

	dir = proc_mkdir("pseudo-sensor", NULL);
	if (!dir) {
		ERR("mkdir /proc/psensor_dir failed\n");
		err = -ENOMEM;
		goto debug_kit_init_failed;
	}

	g_data->proc = dir;
	proc_create_data("bind", 0664, dir,  &proc_bind_fops, NULL);
	proc_create_data("sizes", 0664, dir,  &proc_sizes_fops, NULL);
	proc_create_data("current", 0664, dir,  &proc_current_fops, NULL);
	proc_create_data("data", 0664, dir,  &proc_data_fops, NULL);
	proc_create_data("datatype", 0664, dir,  &proc_datatype_fops, NULL);
	proc_create_data("datareset", 0220, dir,  &proc_datareset_fops, NULL);
	proc_create_data("phyaddr", 0444, dir,  &proc_phy_fops, NULL);
	proc_create_data("viraddr", 0444, dir,  &proc_vir_fops, NULL);
	proc_create_data("frees", 0444, dir,  &proc_freemem_fops, NULL);
	proc_create_data("timeleft", 0664, dir,  &proc_timeleft_fops, NULL);
	proc_create_data("sst", 0664, dir,  &proc_sst_fops, NULL);
	proc_create_data("ecode", 0664, dir,  &proc_ecode_fops, NULL);
#ifdef OPLUS_FEATURE_SENSOR_DEBUG_KIT
	proc_create_data("fault_code", 0664, dir, &proc_fault_code_fops, NULL);
	proc_create_data("frequency", 0664, dir, &proc_frequency_fops, NULL);
	proc_create_data("repeat", 0664, dir, &proc_repeat_fops, NULL);
	proc_create_data("test_type", 0664, dir, &proc_test_type_fops, NULL);
	proc_create_data("sync", 0664, dir, &proc_sync_fops, NULL);
#endif

	/*create sensor_feedback_task thread*/
	g_data->report_task = kthread_create(sensor_pseudo_report_thread,
			(void *)g_data,
			"sensor_debug_kit_task");
	if (IS_ERR(g_data->report_task)) {
		ERR("kthread_create failed\n");
		err = PTR_ERR(g_data->report_task);
		goto debug_kit_init_failed;
	}

	/*wake up thread of report_task*/
	wake_up_process(g_data->report_task);

	INIT_DELAYED_WORK(&g_data->time_left_work, psensor_timer_left);

	INF("pseduo sensor initialize successed\n");
	return 0;

debug_kit_init_failed:
	kfree(g_data);
	g_data = NULL;
	return 0;
}


static void __exit pseduo_sensor_exit(void)
{
	if (NULL == g_data) {
		ERR("malloc memory failed?\n");
		return;
	}

	proc_remove(g_data->proc);

	kfree(g_data);
	g_data = NULL;
}


module_init(pseduo_sensor_init);
module_exit(pseduo_sensor_exit);


MODULE_AUTHOR("weng.yunfeng");
MODULE_DESCRIPTION("Pseudo Sensor Driver");
MODULE_LICENSE("GPL");
