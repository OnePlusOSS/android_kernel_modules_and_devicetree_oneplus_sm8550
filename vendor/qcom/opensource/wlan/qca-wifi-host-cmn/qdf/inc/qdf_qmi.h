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
 * DOC: qdf_qmi.h
 * This file defines the QMI abstraction.
 */

#ifndef _QDF_QMI_H
#define _QDF_QMI_H

#ifdef WLAN_QMI

#include <i_qdf_qmi.h>

typedef __qdf_qmi_handle qdf_qmi_handle;

typedef __qdf_qmi_ops qdf_qmi_ops;

typedef __qdf_qmi_msg_handler qdf_qmi_msg_handler;

typedef __qdf_qmi_service qdf_qmi_service;

typedef __qdf_qmi_txn qdf_qmi_txn;

typedef __qdf_qmi_elem_info qdf_qmi_elem_info;

typedef __qdf_sockaddr_qrtr qdf_sockaddr_qrtr;

/**
 * qdf_qmi_handle_init() - Initialize QMI handle
 * @qmi_hdl:        QMI handle to initialize
 * @recv_buf_size: maximum size of incoming message
 * @ops: reference to callbacks for QRTR notifications
 * @qmi_msg_handlers:   NULL-terminated list of QMI message handlers
 *
 * Returns: QDF status
 */
static inline
QDF_STATUS qdf_qmi_handle_init(qdf_qmi_handle *qmi_hdl, uint32_t recv_buf_size,
			       qdf_qmi_ops *ops,
			       qdf_qmi_msg_handler *qmi_msg_handlers)
{
	return __qdf_qmi_handle_init(qmi_hdl, recv_buf_size, ops,
				     qmi_msg_handlers);
}

/**
 * qdf_qmi_handle_release() - Release QMI handle
 * @qm_hdl: QMI handle to release
 *
 * Returns: None
 */
static inline
void qdf_qmi_handle_release(qdf_qmi_handle *qmi_hdl)
{
	return __qdf_qmi_handle_release(qmi_hdl);
}

/**
 * qdf_qmi_add_lookup() - Register a new lookup with the name service
 * @qmi_hdl: QMI handle
 * @service: service id of the request
 * @version: version number of the request
 * @instance: instance id of the request
 *
 * Return: QDF status
 */
static inline
QDF_STATUS qdf_qmi_add_lookup(qdf_qmi_handle *qmi_hdl, unsigned int service,
			      unsigned int version, unsigned int instance)
{
	return __qdf_qmi_add_lookup(qmi_hdl, service, version, instance);
}

/**
 * qdf_qmi_connect_to_svc() - Connect to QMI service
 * @qmi_hdl: QMI handle
 * @qmi_svc: QMI service handle
 *
 * Return: QDF status
 */
static inline
QDF_STATUS qdf_qmi_connect_to_svc(qdf_qmi_handle *qmi_hdl,
				  qdf_qmi_service *qmi_svc)
{
	return __qdf_qmi_connect_to_svc(qmi_hdl, qmi_svc);
}

/**
 * qdf_qmi_txn_init() - Initialize QMI transaction handle
 * @qmi_hdl: QMI handle
 * @qmi_txn: QMI transaction handle
 * @qmi_ei: description of how to decode a matching response (optional)
 * @qmi_resp: pointer to the object to decode the response into (optional)
 *
 * Return: QDF status
 */
static inline
QDF_STATUS qdf_qmi_txn_init(qdf_qmi_handle *qmi_hdl, qdf_qmi_txn *qmi_txn,
			    qdf_qmi_elem_info *qmi_ei, void *qmi_resp)
{
	return __qdf_qmi_txn_init(qmi_hdl, qmi_txn, qmi_ei, qmi_resp);
}

/**
 * qdf_qmi_send_request() - Send QMI request
 * @qmi_hdl: QMI handle
 * @sq: destination sockaddr
 * @qmi_txn: QMI transaction handle
 * @msg_id: message id
 * @len: max length of the QMI message
 * @ei: QMI message description
 * @req: message to be encoded
 *
 * Return: QDF status
 */
static inline
QDF_STATUS qdf_qmi_send_request(qdf_qmi_handle *qmi_hdl, qdf_sockaddr_qrtr *sq,
				qdf_qmi_txn *qmi_txn, int msg_id, uint32_t len,
				qdf_qmi_elem_info *ei, const void *req)
{
	return __qdf_qmi_send_request(qmi_hdl, sq, qmi_txn, msg_id, len, ei,
				      req);
}

/**
 * qdf_qmi_txn_wait() - Wait for transaction response
 * @qmi_txn: QMI transaction handle
 * @timeout: Timeout value in jiffies
 *
 * Return: QDF status
 */
static inline
QDF_STATUS qdf_qmi_txn_wait(qdf_qmi_txn *qmi_txn, unsigned long timeout)
{
	return __qdf_qmi_txn_wait(qmi_txn, timeout);
}

/**
 * qdf_qmi_txn_cancel() - Cancel the QMI transaction
 * @qmi_txn: QMI transaction handle
 *
 * Return: None
 */
static inline
void qdf_qmi_txn_cancel(qdf_qmi_txn *qmi_txn)
{
	__qdf_qmi_txn_cancel(qmi_txn);
}
#endif
#endif
