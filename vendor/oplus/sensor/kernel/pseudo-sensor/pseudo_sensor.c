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
 * 2022-01-15 	 1.1 	 80353364 	to facilitate the loopback BUF processing of shared memory,
 *									change the relevant address counter to 32bit
 * 2023-03-25	 1.2	 80353364	optimize the code design of the QCOM platform,
 *									incorporate fault injection logic,
 *									and NOT support kernels below 5.0
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

#if defined(CFG_OPLUS_ARCH_IS_QCOM)

#include <linux/soc/qcom/smem.h>
#include <linux/qrtr.h>
#include <linux/net.h>
#include <linux/completion.h>
#include <linux/idr.h>
#include <linux/string.h>
#include <net/sock.h>
#include <linux/soc/qcom/qmi.h>

#endif


#if defined(CFG_OPLUS_ARCH_IS_MTK)

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include "hf_manager.h"
#include "scp.h"

#endif


#define DRV_TAG                             "<debug_kit>"
#define DBG(fmt, ...)                       pr_debug(DRV_TAG fmt, ##__VA_ARGS__);
#define INF(fmt, ...)                       pr_info(DRV_TAG fmt, ##__VA_ARGS__);
#define ERR(fmt, ...)                       pr_err(DRV_TAG fmt, ##__VA_ARGS__);


#if defined(CFG_OPLUS_ARCH_IS_QCOM)
#define OPLUS_PSEUDO_SENSOR_MEM_ID           490
#elif defined(CFG_OPLUS_ARCH_IS_MTK)
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,1,0)
#define OPLUS_PSEUDO_SENSOR_MEM_ID          SENS_FB_MEM_ID
#else
#define OPLUS_PSEUDO_SENSOR_MEM_ID          SENS_PSEUDO_SENSOR_MEM_ID
#endif
#endif

#define DEBUG_KIT_SENSOR_SET_MEM_ADDR       10
#define DEBUG_KIT_SENSOR_SET_DEBUG_KIT_DATA 12

#define CFG_BUFSIZE                         512
#define CFG_MAX_SENSORS                     16			// Support 16 SENSOR (data channels)
#define CFG_SMEM_SZ							SZ_64K
#define MIN(a, b)                           ((a) <= (b) ? (a) : (b))

/*******************************************************************************
 * driver data defined
 ******************************************************************************/
// Set the default data size, measured in 32KB, pay attention to subtract the space occupied by smem_head
// All allocated space size needs to be aligned with 64Byte, because the data sequencer is aligned with 0x40
// The final default memory allocation is 64KB
static uint32_t g_def_sz[CFG_MAX_SENSORS] = {
	SZ_16K,	SZ_16K,	SZ_16K,	SZ_1K,
	SZ_1K,  SZ_1K,  SZ_1K,  SZ_1K,
	SZ_1K,  SZ_1K, 	SZ_1K, 	SZ_1K,
	SZ_1K, 	SZ_1K, 	SZ_1K, 	SZ_1K
};


static char *g_chn_type[CFG_MAX_SENSORS] = {
	"acc1",    "gyro1",    "mag1",     "temp",
	"press",   "als1",     "proxi",    "halls",
	"acc2",    "gyro2",    "mag2",     "n/a",
	"n/a",     "als2",     "debug",    "halla"
};


#if defined(CFG_OPLUS_ARCH_IS_QCOM)
/*******************************************************************************
 * QCOM PLATFORM DEFINE
 * type definition
 ******************************************************************************/
struct qmi_data {
	unsigned host;
	struct qmi_handle qmi;
	struct mutex lock;
};
#endif


struct psensor_data {
	uint64_t vir_address;
	uint32_t size;
	int16_t enable;
	int16_t cur_chn;
	int32_t time_left[CFG_MAX_SENSORS];         // timeleft
	struct delayed_work time_left_work;			// timer can NOT use mutex
	struct proc_dir_entry   *proc;
#if defined(CFG_OPLUS_ARCH_IS_QCOM)
	struct qmi_data client;
#elif defined(CFG_OPLUS_ARCH_IS_MTK)
	struct hf_client *client;
#else
	uint32_t rev;
#endif
};


typedef struct smem_head {
	// Total is 1024 Byte, then CFG_MAX_SENSORS need <= 24
	//<!---------------------------------    0B    ---------------------------------!>//
	uint32_t bp[CFG_MAX_SENSORS];               // starting offset of the current channel memory
	uint32_t size[CFG_MAX_SENSORS];             // memory size allocated to the specified SENSOR,
												// ring-buf used must not time out for this address
												// unit is float
	uint32_t wp[CFG_MAX_SENSORS];               // offset of currently written data, unit is float
	uint32_t rp[CFG_MAX_SENSORS];               // offset of currently read data, unit is float
	uint8_t type[CFG_MAX_SENSORS];              // data type
												// 0: data
												// 2: constant value
	uint8_t start[CFG_MAX_SENSORS];             // data start flag
												// 0: data not ready
												// 1: data is ready
												// 2: data is being read
												// 3: data has been retrieved
	uint32_t rev[(512 - (18 * CFG_MAX_SENSORS)) >> 2];
												// data reserved: a total of 512B above, 512B below
	//<!---------------------------------   512B   ---------------------------------!>//
	int32_t inject_start;						// start flag
	int32_t rev2[3];							// data reserved2
	int32_t inject_data[124];					// data for fault injection: 704 BYTE
	//<!---------------------------------   1024B  ---------------------------------!>//
}   smem_head;


static struct psensor_data g_data;

/*******************************************************************************
 * QCOM PLATFORM DEFINE
 * Driver module parameters
 * ADSP: 34
 * SLPI: 49
 ******************************************************************************/
#define ADSP_INSTANCE  					34
#define SLPI_INSTANCE  					49

#if defined(CFG_OPLUS_ARCH_IS_QCOM)
/*******************************************************************************
 * QCOM PLATFORM DEFINE
 * message definition
 * Due to the MTK definition, the QCOM platform is best roughly compatible with MTK
 * magic_number type value1
 * 0x50534F00 	0 	 NA			Open shared memory, or set the first address of the memory
 * 0x50534F00 	1 	 NA			Disable shared memory
 * 0x50534F00 	2 	 X			Open memory access
 * 0x50534F00 	3 	 X			Disable memory access
 ******************************************************************************/
#define DATA_REQ1_TLV_TYPE				0x1
#define DATA_RESP1_TLV_TYPE				0x2
#define DATA_OPT1_TLV_TYPE				0x10
#define DATA_OPT2_TLV_TYPE				0x11
#define TEST_MED_DATA_SIZE_V01			8192
#define TEST_MAX_NAME_SIZE_V01			255
#define TEST_DATA_REQ_MSG_ID_V01		0x21
#define TEST_DATA_REQ_MAX_MSG_LEN_V01	8456
#define SNS_CLIENT_REQ_LEN_MAX_V01 		1000


struct test_name_type_v01 {
	u32 name_len;
	char name[TEST_MAX_NAME_SIZE_V01];
};


struct test_data_req_msg_v01 {
	u32 data_len;
	u8 data[TEST_MED_DATA_SIZE_V01];

	u8 client_name_valid;
	struct test_name_type_v01 client_name;
};


struct test_data_resp_msg_v01 {
	struct qmi_response_type_v01 resp;

	u8 data_valid;
	u32 data_len;
	u8 data[TEST_MED_DATA_SIZE_V01];

	u8 service_name_valid;
	struct test_name_type_v01 service_name;
};


static struct qmi_elem_info test_name_type_v01_ei[] = {
	{
		.data_type	= QMI_DATA_LEN,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
		.offset		= offsetof(struct test_name_type_v01, name_len),
	},
	{
		.data_type	= QMI_UNSIGNED_1_BYTE,
		.elem_len	= TEST_MAX_NAME_SIZE_V01,
		.elem_size	= sizeof(char),
		.array_type	= VAR_LEN_ARRAY,
		.tlv_type	= QMI_COMMON_TLV_TYPE,
		.offset		= offsetof(struct test_name_type_v01, name),
	},
	{}
};


static struct qmi_elem_info test_data_req_msg_v01_ei[] = {
	{
		.data_type	= QMI_DATA_LEN,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= DATA_REQ1_TLV_TYPE,
		.offset		= offsetof(struct test_data_req_msg_v01, data_len),
	},
	{
		.data_type	= QMI_UNSIGNED_1_BYTE,
		.elem_len	= TEST_MED_DATA_SIZE_V01,
		.elem_size	= sizeof(u8),
		.array_type	= VAR_LEN_ARRAY,
		.tlv_type	= DATA_REQ1_TLV_TYPE,
		.offset		= offsetof(struct test_data_req_msg_v01, data),
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= DATA_OPT1_TLV_TYPE,
		.offset		= offsetof(struct test_data_req_msg_v01, client_name_valid),
	},
	{
		.data_type	= QMI_STRUCT,
		.elem_len	= 1,
		.elem_size	= sizeof(struct test_name_type_v01),
		.array_type	= NO_ARRAY,
		.tlv_type	= DATA_OPT1_TLV_TYPE,
		.offset		= offsetof(struct test_data_req_msg_v01, client_name),
		.ei_array	= test_name_type_v01_ei,
	},
	{}
};


static struct qmi_elem_info test_data_resp_msg_v01_ei[] = {
	{
		.data_type	= QMI_STRUCT,
		.elem_len	= 1,
		.elem_size	= sizeof(struct qmi_response_type_v01),
		.array_type	= NO_ARRAY,
		.tlv_type	= DATA_RESP1_TLV_TYPE,
		.offset		= offsetof(struct test_data_resp_msg_v01, resp),
		.ei_array	= qmi_response_type_v01_ei,
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= DATA_OPT1_TLV_TYPE,
		.offset		= offsetof(struct test_data_resp_msg_v01, data_valid),
	},
	{
		.data_type	= QMI_DATA_LEN,
		.elem_len	= 1,
		.elem_size	= sizeof(u32),
		.array_type	= NO_ARRAY,
		.tlv_type	= DATA_OPT1_TLV_TYPE,
		.offset		= offsetof(struct test_data_resp_msg_v01, data_len),
	},
	{
		.data_type	= QMI_UNSIGNED_1_BYTE,
		.elem_len	= TEST_MED_DATA_SIZE_V01,
		.elem_size	= sizeof(u8),
		.array_type	= VAR_LEN_ARRAY,
		.tlv_type	= DATA_OPT1_TLV_TYPE,
		.offset		= offsetof(struct test_data_resp_msg_v01, data),
	},
	{
		.data_type	= QMI_OPT_FLAG,
		.elem_len	= 1,
		.elem_size	= sizeof(u8),
		.array_type	= NO_ARRAY,
		.tlv_type	= DATA_OPT2_TLV_TYPE,
		.offset		= offsetof(struct test_data_resp_msg_v01, service_name_valid),
	},
	{
		.data_type	= QMI_STRUCT,
		.elem_len	= 1,
		.elem_size	= sizeof(struct test_name_type_v01),
		.array_type	= NO_ARRAY,
		.tlv_type	= DATA_OPT2_TLV_TYPE,
		.offset		= offsetof(struct test_data_resp_msg_v01, service_name),
		.ei_array	= test_name_type_v01_ei,
	},
	{}
};


/*******************************************************************************
 * internal function
 * ADSP: 34
 * SLPI: 49
 ******************************************************************************/
static int qmi_new_server(struct qmi_handle *qmi, struct qmi_service *service)
{
	// QMI_PING_SERVICE_INSTANCE: 34 on adsp and 49 on slpi
	int ret = 0;
	unsigned int instance = service->instance;
	struct sockaddr_qrtr sq = { AF_QIPCRTR, service->node, service->port};

	INF("service: %d %d %d \n", service->service, service->version, instance);

	if ((ADSP_INSTANCE != instance) && (SLPI_INSTANCE != instance)) {
		INF("not sensor qmi service, exit\n");
		return -EPERM;
	} else if (SLPI_INSTANCE == instance) {
		// is SLPI
		g_data.client.host = 3;
	}

	ret = kernel_connect(qmi->sock, (struct sockaddr *)&sq, sizeof(sq), 0);
	if (ret < 0) {
		ERR("failed to bind to remote service port: %d@%d\n", service->node, service->port);
		return ret;
	} else {
		INF("bind to remote service port successed: %d@%d\n", service->node, service->port);
	}

	service->priv = &g_data.client;

	return 0;
}

static void qmi_del_server(struct qmi_handle *qmi, struct qmi_service *service)
{
	INF("enter qmi_del_server\n");
}


static struct qmi_ops qmi_lookup_ops = {
	.new_server = qmi_new_server,
	.del_server = qmi_del_server,
};


static int qcom_qmi_service_init(void)
{
	int ret;
	struct qmi_handle *qmi = &g_data.client.qmi;

	memset(&g_data.client, 0, sizeof(g_data.client));
	mutex_init(&g_data.client.lock);
	g_data.client.host = 2;				// ADSP

	ret = qmi_handle_init(qmi, TEST_DATA_REQ_MAX_MSG_LEN_V01, &qmi_lookup_ops, NULL);

	if (ret < 0) {
		ERR("qmi_handle_init failed1: %d\n", ret);
		return ret;
	}

	ret = qmi_add_lookup(qmi, 15, 1, ADSP_INSTANCE);
	if (ret < 0) {
		ERR("qmi_add_lookup failed2: %d\n", ret);
		return ret;
	}

	ret = qmi_add_lookup(qmi, 15, 1, SLPI_INSTANCE);
	if (ret < 0) {
		ERR("qmi_add_lookup failed3: %d\n", ret);
		return ret;
	}

	return ret;
}


static int qcom_send_msg(uint8_t cmd, char *data, int len)
{
	struct test_data_resp_msg_v01 *resp;
	struct test_data_req_msg_v01 *req;
	struct qmi_txn txn;
	int ret = -1;
	struct qmi_handle *qmi = &g_data.client.qmi;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req) {
		ERR("init request memory error\n");
		return -ENOMEM;
	} else {
		INF("init request memory successed\n");
	}

	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		kfree(req);
		ERR("init response memory error\n");
		return -ENOMEM;
	} else {
		INF("init response memory successed\n");
	}

	mutex_lock(&g_data.client.lock);

	req->data_len = min_t(uint32_t, sizeof(req->data), len);
	memcpy(req->data, data, req->data_len);

	ERR("send data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
		req->data[0], req->data[1], req->data[2], req->data[3],
		req->data[4], req->data[5], req->data[6], req->data[7]);

	ret = qmi_txn_init(qmi, &txn, test_data_resp_msg_v01_ei, resp);
	if (ret < 0) {
		ERR("init response data error: %d\n", ret);
		goto out;
	} else {
		INF("init response data successed\n");
	}

	ret = qmi_send_request(	qmi,
							NULL,
							&txn,
							TEST_DATA_REQ_MSG_ID_V01,
							TEST_DATA_REQ_MAX_MSG_LEN_V01,
							test_data_req_msg_v01_ei,
							req);
	if (ret < 0) {
		qmi_txn_cancel(&txn);
		ERR("send msg faild: %d\n", ret);
		goto out;
	} else {
		INF("send msg successed: %d\n", ret);
	}

	ret = qmi_txn_wait(&txn, 2 * HZ);

	if (ret < 0) {
		ERR("get msg response faild: %d\n", ret);
		goto out;
	}

	INF("resp result: %d, error: %d\n", resp->resp.result, resp->resp.error);

	ERR("resp data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
		resp->data[0], resp->data[1], resp->data[2], resp->data[3],
		resp->data[4], resp->data[5], resp->data[6], resp->data[7]);

	if (resp->data_valid && resp->data[0] == '0') {
		INF("resp result successed\n");
		ret = 0;
	} else {
		INF("resp result failed\n");
		ret = -EINVAL;
	}

out:
	mutex_unlock(&g_data.client.lock);
	kfree(resp);
	kfree(req);

	return ret;
}


static int qcom_send_fatal_msg(char* data, int len)
{
	uint32_t cust_cmd[4];

	len = MIN(16, len);

	memset(&cust_cmd[0], 0, sizeof(cust_cmd));
	memcpy(&cust_cmd[0], data, len);

//	cust_cmd[0] = (cust_cmd[0] & 0xFFFF) | 0x50540000;
	cust_cmd[0] = (cust_cmd[0] & 0xFF) | 0x50544E00;

	return qcom_send_msg(DEBUG_KIT_SENSOR_SET_DEBUG_KIT_DATA, (char*)&cust_cmd[0], len);
}


static int qcom_qmi_service_free(void)
{
	struct qmi_handle *qmi = &g_data.client.qmi;

	qmi_handle_release(qmi);

	return 0;
}

static int qcom_get_smem(void)
{
	int ret;
	void *vmem;
	size_t size;
	uint32_t cust_cmd[2];

	cust_cmd[0] = 0x50534F00;
	cust_cmd[1] = OPLUS_PSEUDO_SENSOR_MEM_ID;
	ret = qcom_send_msg(0, (char*)&cust_cmd, 8);
	if (ret < 0) {
		ERR("bp malloc memory failed\n");
		return ret;
	}

	vmem = qcom_smem_get(g_data.client.host, OPLUS_PSEUDO_SENSOR_MEM_ID, &size);

	if (IS_ERR(vmem)) {
		ERR("qcom_smem_alloc failed, host: %d type: %d\n",
			g_data.client.host,
			OPLUS_PSEUDO_SENSOR_MEM_ID);

		g_data.vir_address = 0;
		g_data.size = 0;
		return -ENOMEM;
	}

	INF("qcom_smem_alloc successed, host: %d type: %d size: %ldB\n",
		g_data.client.host,
		OPLUS_PSEUDO_SENSOR_MEM_ID,
		size);

	g_data.vir_address = (uint64_t)vmem;
	g_data.size = size;
	return 0;
}


static int qcom_put_smem(void)
{
	int ret = 0;
	uint32_t cust_cmd[2];
	cust_cmd[0] = 0x50534F01;
	cust_cmd[1] = 0;
	ret = qcom_send_msg(0, (char*)&cust_cmd, 8);
	if (ret >= 0) {
		g_data.vir_address = 0;
	}
	return ret;
}


static int qcom_set_chn(uint32_t chn_data, int enabled)
{
	int ret;
	uint32_t cust_cmd[2];

	if (enabled) {
		INF("open data channels: 0x%08X\n", chn_data);
		cust_cmd[0] = 0x50534F02;
	} else {
		INF("close data channels: 0x%08X\n", chn_data);
		cust_cmd[0] = 0x50534F03;
	}

	cust_cmd[1] = chn_data;

	ret = qcom_send_msg(0, (char*)&cust_cmd, 8);
	if (ret < 0) {
		return -EINVAL;
	}

	return 0;
}

#endif


/*******************************************************************************
 * MTK PLATFORM DEFINE
 ******************************************************************************/
#if defined(CFG_OPLUS_ARCH_IS_MTK)
static int mtk_send_msg(uint8_t cmd, char *data, int len)
{
	int ret;
	struct custom_cmd cust_cmd;

	memset(&cust_cmd, 0, sizeof(cust_cmd));
	cust_cmd.command = cmd;
	cust_cmd.rx_len = 0;
	cust_cmd.tx_len = len;

	memcpy(&cust_cmd.data[0], data, len);

	ret = hf_client_custom_cmd(g_data.client, SENSOR_TYPE_DEBUG_KIT, &cust_cmd);

	if (ret < 0) {
		ERR("mtk_send_msg failed: %d", ret);
		return -EINVAL;
	}
	return 0;
}


static int mtk_send_fatal_msg(char* data, int len)
{
	return mtk_send_msg(DEBUG_KIT_SENSOR_SET_DEBUG_KIT_DATA, data, len);
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

	ret = mtk_send_msg(DEBUG_KIT_SENSOR_SET_MEM_ADDR, (char*)&data[0], 12);
	if (ret < 0) {
		return -EINVAL;
	}

	return 0;
}


static int mtk_get_smem(void)
{
	int ret;
	uint32_t data[3];
	phys_addr_t phys_addr, vir_address, size;

	int mem_id = OPLUS_PSEUDO_SENSOR_MEM_ID;

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

	ret = mtk_send_msg(DEBUG_KIT_SENSOR_SET_MEM_ADDR, (char*)&data[0], 12);
	if (ret < 0) {
		return -EINVAL;
	}

	g_data.vir_address = vir_address;
	g_data.size = size;

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

	ret = mtk_send_msg(DEBUG_KIT_SENSOR_SET_MEM_ADDR, (char*)&data[0], 12);
	if (ret < 0) {
		return -EINVAL;
	}

	g_data.vir_address = 0;
	g_data.size = 0;

	return 0;
}


static int mtk_scp_sensor_init(void)
{
	int ret = 0;
	struct hf_manager_cmd cmd;
	struct hf_manager_batch *batch = NULL;
	g_data.client = hf_client_create();
	if (!g_data.client) {
		ERR("hf_client_create fail\n");
		return -ENOMEM;
	}

	ret = hf_client_find_sensor(g_data.client, SENSOR_TYPE_DEBUG_KIT);
	if (ret < 0) {
		ERR("hf_client_find_sensor debug kit sensor fail\n");
		return -EINVAL;
	}

	memset(&cmd, 0, sizeof(cmd));
	cmd.sensor_type = SENSOR_TYPE_DEBUG_KIT;
	cmd.action = HF_MANAGER_SENSOR_ENABLE;
	batch = (struct hf_manager_batch *)cmd.data;
	batch->delay = 20000000;
	batch->latency = 0;
	ret = hf_client_control_sensor(g_data.client, &cmd);
	if (ret < 0) {
		ERR("hf_client_control_sensor debug kit sensor fail\n");
		return -EINVAL;
	}

	return 0;
}


static int mtk_scp_sensor_free(void)
{
	if (!g_data.client) {
		ERR("hf_client_create fail\n");
		return -ENOMEM;
	}

	hf_client_destroy(g_data.client);

	g_data.client = NULL;

	return 0;
}

#endif



/*******************************************************************************
 * Platform-dependent uniformly named functions
 ******************************************************************************/
#if defined(CFG_OPLUS_ARCH_IS_QCOM)

#define platform_set_chn		qcom_set_chn
#define platform_get_smem		qcom_get_smem
#define platform_put_smem		qcom_put_smem
#define platform_send_fatal_msg	qcom_send_fatal_msg
#define platform_client_init	qcom_qmi_service_init
#define platform_client_release	qcom_qmi_service_free

#elif defined(CFG_OPLUS_ARCH_IS_MTK)

#define platform_set_chn		mtk_set_chn
#define platform_get_smem		mtk_get_smem
#define platform_put_smem		mtk_put_smem
#define platform_send_fatal_msg	mtk_send_fatal_msg
#define platform_client_init	mtk_scp_sensor_init
#define platform_client_release	mtk_scp_sensor_free

#else

#error PLATFORM_NOT_DEFINED: problem with makefile?

#endif



/*******************************************************************************
 * driver common function
 ******************************************************************************/
static int fault_inject_append(int32_t fault_code, int32_t frequency, int32_t repeat, int32_t test_type)
{
	smem_head *smem = (smem_head*)g_data.vir_address;
	if (NULL == smem) {
		return -ENOMEM;
	}

	smem->inject_data[0] = fault_code;
	smem->inject_data[1] = frequency;
	smem->inject_data[2] = repeat;
	smem->inject_data[3] = test_type;

	return 0;
}


static int fault_inject_reset(void)
{
	smem_head *smem = (smem_head*)g_data.vir_address;
	if (NULL == smem) {
		return -ENOMEM;
	}

	smem->inject_start = 0;
	memset(smem->inject_data, 0, sizeof(smem->inject_data));
	return 0;
}


static int fault_inject_start(void)
{
	int ret;
	smem_head *smem = (smem_head*)g_data.vir_address;
	if (NULL == smem) {
		return -ENOMEM;
	}

	ret = platform_send_fatal_msg((char*)&smem->inject_data[0], 16);

	if (ret >= 0) {
		smem->inject_start = 1;
	}

	return ret;
}


static void psensor_timer_left(struct work_struct *dwork)
{
	int i;
	int ret;
	int cnt = 0;
	uint32_t chns = 0;
	smem_head *smem = (smem_head*)g_data.vir_address;

	if (0 == g_data.vir_address) {
		return;
	}

	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		if (g_data.time_left[i] == 0)  {
			continue;
		} else {
			g_data.time_left[i]--;
			cnt++;
		}

		if (g_data.time_left[i] == 0)  {
			smem->start[i] = 1;
			chns |= 1U << i;
			cnt--;
		}
	}

	if (chns) {
		ret = platform_set_chn(chns, 1);
		if (ret < 0) {
			ERR("mtk set chn failed: 0x%08X\n", chns);
		}
	}

	if (cnt > 0) {
		schedule_delayed_work(&g_data.time_left_work, msecs_to_jiffies(1000));
	}
}


// Initialize the shared memory space
// According to the total size and the length of the sz array, the space is divided into smem
// Note that size is measured in bytes, sz is also measured in bytes, but in smem it is measured in float
static int psensor_smem_init(smem_head *smem, uint32_t size, uint32_t *sz)
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
	uint32_t size = count >> 2;
	size = MIN(size, smem->size[chn] - smem->wp[chn] + smem->rp[chn]);
	len = MIN(size, smem->size[chn] - (smem->wp[chn] % smem->size[chn]));

	p = (char*)g_data.vir_address;
	p += smem->bp[chn] << 2;
	p += (smem->wp[chn] % smem->size[chn]) << 2;

	if (copy_from_user(p, buf, len << 2)) {
		ERR("copy data from user error1\n");
		return -EIO;
	}


	if (size - len > 0) {
		p = (char*)g_data.vir_address;
		p += smem->bp[chn] << 2;
		if (copy_from_user(p, buf + len, (size - len) << 2)) {
			ERR("copy data from user error2\n");
			return -EIO;
		}
	}

	smem->wp[chn] += size;
	return size << 2;
}


static ssize_t proc_data_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
#define MLV 1792

	char data[MLV];
	int i, len, total = 0;
	char *d = (char *)&data[0];
	smem_head *smem = (smem_head*)g_data.vir_address;

	if (*ppos > 0 || count < sizeof(data)) {
		return 0;
	}

	if (0 == g_data.vir_address) {
		total = snprintf(d, MLV, "memory invalid, plese enable first\n");
		ERR("memory invalid, plese enable first\n");
		if (copy_to_user(buf, data, total)) {
			return -EFAULT;
		}
		*ppos = total;
		return total;
	}

	len = snprintf(d, MLV - total, "addr\t\t: %016llX\n", g_data.vir_address);
	d += len;
	total += len;
	len = snprintf(d, MLV - total, "size\t\t: %X\n", g_data.size);
	d += len;
	total += len;
	len = snprintf(d, MLV - total, "chn\t\t: %d\n", g_data.cur_chn);
	d += len;
	total += len;
	len = snprintf(d, MLV - total,
				   "chn name\toff.f\tsize.f\tdtype\tstart\twp.f\trp.f\ttimeleft\n");
	d += len;
	total += len;

	for(i = 0; i < CFG_MAX_SENSORS; i++) {
		if (i == g_data.cur_chn) {
			len = snprintf(d, MLV - total,
						   "%02d< %-4s\t%X\t%X\t%X\t%X\t%X\t%X\t%d\n",
							i,
							g_chn_type[i],
							smem->bp[i],
							smem->size[i],
							smem->type[i],
							smem->start[i],
							smem->wp[i],
							smem->rp[i],
							g_data.time_left[i]);
		} else {
			len = snprintf(d, MLV - total,
						   "%02d  %-4s\t%X\t%X\t%X\t%X\t%X\t%X\t%d\n",
						   i,
						   g_chn_type[i],
						   smem->bp[i],
						   smem->size[i],
						   smem->type[i],
						   smem->start[i],
						   smem->wp[i],
						   smem->rp[i],
						   g_data.time_left[i]);

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
	int chn = (int)g_data.cur_chn;
	smem_head *smem = (smem_head*)g_data.vir_address;
	int len = MIN(count, sizeof(data) - 1);

	if (0 == g_data.vir_address) {
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
		p = (uint32_t*)g_data.vir_address;
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
		i = snprintf(d, 512-len, "chn%2d :\t%08X\n", chn, g_def_sz[chn]);
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
	uint32_t s[CFG_MAX_SENSORS];
	int len = MIN(count, sizeof(data) - 1);

	if (0 == g_data.vir_address) {
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

	ret = platform_set_chn(-1U, 0);
	if (ret < 0) {
		ERR("set chn failed: 0xFFFFFFFF\n");
		return -EIO;
	}

	// Try to allocate space for each channel and return the allocation result
	ret = psensor_smem_init((smem_head*)g_data.vir_address, g_data.size, s);

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
	int chn = (int)g_data.cur_chn;
	smem_head *smem = (smem_head*)g_data.vir_address;

	if (0 == g_data.vir_address) {
		ERR("invalid memory address\n");
		return -EIO;
	}

	memset(data, 0, sizeof(data));

	len = MIN(count, sizeof(data) - 1);

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

	p = (char*)g_data.vir_address;
	p += smem->bp[chn] << 2;
	memset(p, 0, smem->size[chn] << 2);
	smem->wp[chn] = 0;
	smem->rp[chn] = 0;
	smem->type[chn] = 0;
	smem->start[chn] = 0;

	ret = platform_set_chn(1U << chn, 0);
	if (ret < 0) {
		ERR("set chn failed: 0x%08X\n", 1U << chn);
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
		i = snprintf(d, 512-len, "chn%2d timeleft\t: %d\n", chn, g_data.time_left[chn]);
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
	int len = MIN(count, sizeof(data) - 1);

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
		chns = 1U << g_data.cur_chn;
	}

	for (i = 0; i < CFG_MAX_SENSORS; i++) {
		if (chns & (1U << i)) {
			g_data.time_left[i] = timeleft;
		}
	}

	if (0 == timeleft && g_data.vir_address) {
		// start immediately
		smem = (smem_head*)g_data.vir_address;
		for (i = 0; i < CFG_MAX_SENSORS; i++) {
			if (chns & (1U << i)) {
				smem->start[i] = 1;
			}
		}
		ret = platform_set_chn(chns, 1);
		if (ret < 0) {
			ERR("set chn failed: 0x%08X\n", chns);
		}
	} else {
		schedule_delayed_work(&g_data.time_left_work, msecs_to_jiffies(1000));
	}

	return count;
}


static ssize_t proc_datatype_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len;
	int chn = (int)g_data.cur_chn;
	smem_head *smem = (smem_head*)g_data.vir_address;

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
	int chn = (int)g_data.cur_chn;
	smem_head *smem = (smem_head*)g_data.vir_address;

	memset(data, 0, sizeof(data));

	len = MIN(count, sizeof(data) - 1);

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


static ssize_t proc_vir_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "0x%016llX\n", g_data.vir_address);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}

#if defined(CFG_OPLUS_ARCH_IS_QCOM)

static ssize_t proc_freemem_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	uint32_t bytes;
	int len = 0;
	char *d = &data[0];

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	bytes = (uint32_t)qcom_smem_get_free_space(-1);
	len += snprintf(d, CFG_BUFSIZE - len, "[zone comm]\t: 0x%08X\n", bytes);
	d = &data[len];

	bytes = (uint32_t)qcom_smem_get_free_space(0);
	len += snprintf(d, CFG_BUFSIZE - len, "[zone apss]\t: 0x%08X\n", bytes);
	d = &data[len];

	bytes = (uint32_t)qcom_smem_get_free_space(1);
	len += snprintf(d, CFG_BUFSIZE - len, "[zone mpss]\t: 0x%08X\n", bytes);
	d = &data[len];

	bytes = (uint32_t)qcom_smem_get_free_space(2);
	len += snprintf(d, CFG_BUFSIZE - len, "[zone adsp]\t: 0x%08X\n", bytes);
	d = &data[len];

	bytes = (uint32_t)qcom_smem_get_free_space(3);
	len += snprintf(d, CFG_BUFSIZE - len, "[zone slpi]\t: 0x%08X\n", bytes);

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}

#elif defined(CFG_OPLUS_ARCH_IS_MTK)

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
		len += snprintf(d, CFG_BUFSIZE - len, "%d\t%016llX\t%08X\n", i, phy, (uint32_t)sz);
	}

	if(copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}

#else

#error PLATFORM_NOT_DEFINED: problem with makefile?

#endif


static ssize_t proc_current_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "%d\n", g_data.cur_chn);

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

	len = MIN(count, sizeof(data) - 1);

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

	g_data.cur_chn = cur_chn;
	return count;
}


static ssize_t proc_bind_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[CFG_BUFSIZE];
	int len = 0;

	if (*ppos > 0 || count < CFG_BUFSIZE) {
		return 0;
	}

	len = snprintf(data, CFG_BUFSIZE, "%d\n", g_data.enable);

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
	int ret = 0;
	size_t size = 0;
	int enable = 0;
	char data[16];

	memset(data, 0, sizeof(data));

	len = MIN(count, sizeof(data) - 1);

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
			if (0 == g_data.vir_address) {
				ERR("can not free share memory that not malloc\n");
				return -ENOMEM;
			}
			ret = platform_put_smem();
			if (ret < 0) {
				ERR("put share memory failed\n");
				return ret;
			}

			break;
		case 0x01:
			if (0 != g_data.vir_address) {
				ERR("alread bind\n");
				return -EBUSY;
			}

			ret = platform_get_smem();
			if (ret < 0) {
				ERR("get share memory failed\n");
				return ret;
			}

			size = (size_t)g_data.size;
			vmem = (void*)g_data.vir_address;
			INF("share memory info: vir: %016llX, size: %08lX\n",
				g_data.vir_address, size);

			if ((NULL == vmem) || (0 == size)) {
				ERR("memory malloc failed\n");
				return -ENOMEM;
			}

			memset(vmem, 0, size);
			psensor_smem_init((smem_head*)g_data.vir_address, g_data.size, g_def_sz);
			INF("memory malloc 0x%XByte at %016llX\n",
				(uint32_t)size, g_data.vir_address);
			break;
		default:
			INF("operation number not support: %d\n", enable);
	}

	g_data.enable = enable;
	return count;
}


static ssize_t proc_fault_inject_r(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[512];
	int i, wlen, len = 0;
	char *d = (char *)&data[0];
	smem_head *smem = (smem_head*)g_data.vir_address;

	if(g_data.vir_address == 0) {
		ERR("vir_address not bind yet, do bind first!\n");
		return -EIO;
	}

	if (*ppos > 0 || count < sizeof(data)) {
		return 0;
	}

	wlen = snprintf(d, 512-len, "stat %d\n", smem->inject_start);
	d += wlen;
	len += wlen;

	for (i = 0; i < 16; i+=4) {
		wlen = snprintf(d,
						512-len,
						"%04X %08X %08X %08X %08X\n",
						i << 2,
						smem->inject_data[i],
						smem->inject_data[i+1],
						smem->inject_data[i+2],
						smem->inject_data[i+3]);
		d += wlen;
		len += wlen;
	}

	if (copy_to_user(buf, data, len)) {
		return -EFAULT;
	}

	*ppos = len;
	return len;
}


static ssize_t proc_fault_inject_w(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int ret;
	char data[128];
	int32_t s[4];
	int len = MIN(count, sizeof(data) - 1);

	if (0 == g_data.vir_address) {
		ERR("no share memory, plese bind share memory first\n");
		return -ENOMEM;
	}

	memset(data, 0, sizeof(data));
	memset(s, 0, sizeof(s));

	if (copy_from_user(data, buf, len)) {
		ERR("fault injection val copy from user error\n");
		return -EIO;
	}

	if (0 == strncmp(data, "start", 5)) {
		fault_inject_start();
		return count;
	} else if (0 == strncmp(data, "reset", 5)) {
		fault_inject_reset();
		return count;
	}

	ret = sscanf(data, "%d,%d,%d,%d", &s[0], &s[1], &s[2], &s[3]);
	if (4 != ret) {
		ret = sscanf(data, "%X,%X,%X,%X", &s[0], &s[1], &s[2], &s[3]);
		if (4 != ret) {
			ERR("get %d valid data but need 4 data\n", ret);
			return -EINVAL;
		}
	}

	if (fault_inject_append(s[0], s[1], s[2], s[3]) < 0) {
		ERR("append fault inject data failed: %08X %08X %08X %08X\n", s[0], s[1], s[2], s[3]);
		return -EIO;
	} else {
		INF("append fault inject data successed: %08X %08X %08X %08X\n", s[0], s[1], s[2], s[3]);
	}

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

static const struct file_operations proc_fault_inject_fops =
{
	.owner = THIS_MODULE,
	.read = proc_fault_inject_r,
	.write = proc_fault_inject_w,
	.open = simple_open,
};

static const struct file_operations proc_datatype_fops =
{
	.owner = THIS_MODULE,
	.read = proc_datatype_r,
	.write = proc_datatype_w,
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

static const struct proc_ops proc_fault_inject_fops =
{
	.proc_read = proc_fault_inject_r,
	.proc_write = proc_fault_inject_w,
	.proc_open = simple_open,
};


static const struct proc_ops proc_datatype_fops =
{
	.proc_read = proc_datatype_r,
	.proc_write = proc_datatype_w,
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

#endif


static int __init pseduo_sensor_init(void)
{
	struct proc_dir_entry *dir;

	memset((void*)&g_data, 0, sizeof(struct psensor_data));

	dir = proc_mkdir("pseudo-sensor", NULL);

	if (!dir) {
		ERR("mkdir /proc/psensor_dir failed\n");
		return -ENOMEM;
	}

	g_data.proc = dir;

	proc_create_data("bind", 0664, dir,  &proc_bind_fops, NULL);
	proc_create_data("sizes", 0664, dir,  &proc_sizes_fops, NULL);
	proc_create_data("current", 0664, dir,  &proc_current_fops, NULL);
	proc_create_data("data", 0664, dir,  &proc_data_fops, NULL);
	proc_create_data("datatype", 0664, dir,  &proc_datatype_fops, NULL);
	proc_create_data("datareset", 0220, dir,  &proc_datareset_fops, NULL);
	proc_create_data("viraddr", 0444, dir,  &proc_vir_fops, NULL);
	proc_create_data("frees", 0444, dir,  &proc_freemem_fops, NULL);
	proc_create_data("timeleft", 0664, dir,  &proc_timeleft_fops, NULL);
	proc_create_data("fault_inject", 0664, dir,  &proc_fault_inject_fops, NULL);

	platform_client_init();

	INIT_DELAYED_WORK(&g_data.time_left_work, psensor_timer_left);

	INF("pseduo sensor initialize successed\n");

	return 0;
}


static void __exit pseduo_sensor_exit(void)
{
	platform_client_release();

	cancel_delayed_work(&g_data.time_left_work);

	proc_remove(g_data.proc);

	if (0 != g_data.vir_address) {
	}
}


module_init(pseduo_sensor_init);
module_exit(pseduo_sensor_exit);


MODULE_AUTHOR("weng.yunfeng");
MODULE_DESCRIPTION("Pseudo Sensor Driver");
MODULE_LICENSE("GPL");
