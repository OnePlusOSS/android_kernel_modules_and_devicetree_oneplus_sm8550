/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: i_qdf_qmi.h
 * This file provides OS dependent QMI APIs.
 */

#ifndef _I_QDF_QMI_H
#define _I_QDF_QMI_H

#ifdef WLAN_QMI

#include <linux/soc/qcom/qmi.h>
#include <linux/net.h>
#include <linux/types.h>
#include <qdf_types.h>
#include <qdf_trace.h>

typedef struct qmi_handle __qdf_qmi_handle;

typedef struct qmi_ops __qdf_qmi_ops;

typedef struct qmi_msg_handler __qdf_qmi_msg_handler;

typedef struct qmi_service __qdf_qmi_service;

typedef struct qmi_txn __qdf_qmi_txn;

typedef struct qmi_elem_info __qdf_qmi_elem_info;

typedef struct sockaddr_qrtr __qdf_sockaddr_qrtr;

/**
 * __qdf_qmi_handle_init() - Initialize QMI handle
 * @qmi_hdl:        QMI handle to initialize
 * @recv_buf_size: maximum size of incoming message
 * @ops: reference to callbacks for QRTR notifications
 * @handlers:   NULL-terminated list of QMI message handlers
 *
 * Returns: QDF status
 */
static inline
QDF_STATUS __qdf_qmi_handle_init(struct qmi_handle *qmi_hdl,
				 size_t recv_buf_size,
				 const struct qmi_ops *ops,
				 const struct qmi_msg_handler *qmi_msg_handlers)
{
	int ret;

	ret = qmi_handle_init(qmi_hdl, recv_buf_size, ops, qmi_msg_handlers);
	if (ret < 0)
		return QDF_STATUS_QMI_HANDLE_INIT_FAILED;

	return QDF_STATUS_SUCCESS;
}

/**
 * __qdf_qmi_handle_release() - Release QMI handle
 * @qm_hdl: QMI handle to release
 *
 * Returns: None
 */
static inline void __qdf_qmi_handle_release(struct qmi_handle *qmi_hdl)
{
	qmi_handle_release(qmi_hdl);
}

/**
 * __qdf_qmi_add_lookup() - Register a new lookup with the name service
 * @qmi_hdl:        qmi handle
 * @service:    service id of the request
 * @instance:   instance id of the request
 * @version:    version number of the request
 *
 * Return: QDF status
 */
static inline
QDF_STATUS __qdf_qmi_add_lookup(struct qmi_handle *qmi_hdl,
				unsigned int service, unsigned int version,
				unsigned int instance)
{
	int ret;

	ret = qmi_add_lookup(qmi_hdl, service, version, instance);
	if (ret < 0)
		return QDF_STATUS_QMI_ADD_LOOKUP_FAILED;

	return QDF_STATUS_SUCCESS;
}

/**
 * __qdf_qmi_connect_to_svc() - Connect to QMI service
 * @qmi_hdl: QMI handle
 * @qmi_svc: QMI service handle
 *
 * Return: QDF status
 */
static inline
QDF_STATUS __qdf_qmi_connect_to_svc(struct qmi_handle *qmi_hdl,
				    struct qmi_service *qmi_svc)
{
	struct sockaddr_qrtr sq = { 0 };
	int ret = 0;

	qdf_info("QMI server arriving: node %u port %u", qmi_svc->node,
		 qmi_svc->port);

	sq.sq_family = AF_QIPCRTR;
	sq.sq_node = qmi_svc->node;
	sq.sq_port = qmi_svc->port;

	ret = kernel_connect(qmi_hdl->sock, (struct sockaddr *)&sq,
			     sizeof(sq), 0);
	if (ret < 0) {
		qdf_err("Failed to connect to QMI remote service");
		return QDF_STATUS_QMI_SVC_CONNECT_FAILED;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * __qdf_qmi_txn_init() - Initialize QMI transaction
 * @qmi_hdl: QMI handle
 * @qmi_txn: QMI transaction handle
 * @qmi_ei: description of how to decode a matching response (optional)
 * @qmi_resp: pointer to the object to decode the response into (optional)
 *
 * Return: QDF status
 */
static inline
QDF_STATUS __qdf_qmi_txn_init(struct qmi_handle *qmi_hdl,
			      struct qmi_txn *qmi_txn,
			      struct qmi_elem_info *qmi_ei, void *resp)
{
	int ret;

	ret = qmi_txn_init(qmi_hdl, qmi_txn, qmi_ei, resp);
	if (ret < 0) {
		qdf_info("QMI transaction init failed");
		return QDF_STATUS_QMI_TXN_INIT_FAILED;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * __qdf_qmi_send_request() - Connect to QMI service
 * @qmi_hdl: QMI handle
 * @sq: destination sockaddr
 * @qmi_txn: QMI transaction handle
 * @msg_id: message id
 * @len: max length of the QMI message
 * @ei: QMI message description
 * @msg: message to be encoded
 *
 * Return: QDF status
 */
static inline
QDF_STATUS __qdf_qmi_send_request(struct qmi_handle *qmi_hdl,
				  struct sockaddr_qrtr *sq,
				  struct qmi_txn *qmi_txn, int msg_id,
				  uint32_t len, struct qmi_elem_info *ei,
				  const void *req)
{
	int ret;

	ret = qmi_send_request(qmi_hdl, sq, qmi_txn, msg_id, len, ei, req);
	if (ret < 0) {
		qdf_info("QMI send request failed");
		return QDF_STATUS_QMI_SEND_REQ_FAILED;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * __qdf_qmi_txn_wait() - Wait for transaction response
 * @qmi_txn: QMI transaction handle
 * @timeout: Timeout value in jiffies
 *
 * Return: QDF status
 */
static inline
QDF_STATUS __qdf_qmi_txn_wait(struct qmi_txn *qmi_txn, unsigned long timeout)
{
	int ret;

	ret = qmi_txn_wait(qmi_txn, timeout);
	if (ret < 0) {
		qdf_info("QMI Failed to wait for response");
		return QDF_STATUS_QMI_TXN_WAIT_FAILED;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * __qdf_qmi_txn_cancel() - Cancel the QMI transaction
 * @qmi_txn: QMI transaction handle
 *
 * Return: None
 */
static inline void __qdf_qmi_txn_cancel(struct qmi_txn *qmi_txn)
{
	qmi_txn_cancel(qmi_txn);
}
#endif
#endif
