/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

#ifndef __UAPI_CAM_SYNC_H__
#define __UAPI_CAM_SYNC_H__

#include <linux/videodev2.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/media.h>

#define CAM_SYNC_DEVICE_NAME                     "cam_sync_device"

/* V4L event which user space will subscribe to */
#define CAM_SYNC_V4L_EVENT                       (V4L2_EVENT_PRIVATE_START + 0)
#define CAM_SYNC_V4L_EVENT_V2                    (V4L2_EVENT_PRIVATE_START + 1)

/* Specific event ids to get notified in user space */
#define CAM_SYNC_V4L_EVENT_ID_CB_TRIG            0

/* Size of opaque payload sent to kernel for safekeeping until signal time */
#define CAM_SYNC_USER_PAYLOAD_SIZE               2

/* Device type for sync device needed for device discovery */
#define CAM_SYNC_DEVICE_TYPE                     (MEDIA_ENT_F_OLD_BASE)

#define CAM_SYNC_GET_PAYLOAD_PTR(ev, type)       \
	(type *)((char *)ev.u.data + sizeof(struct cam_sync_ev_header))

#define CAM_SYNC_GET_HEADER_PTR(ev)              \
	((struct cam_sync_ev_header *)ev.u.data)

#define CAM_SYNC_GET_PAYLOAD_PTR_V2(ev, type)       \
	(type *)((char *)ev.u.data + sizeof(struct cam_sync_ev_header_v2))

#define CAM_SYNC_GET_HEADER_PTR_V2(ev)              \
	((struct cam_sync_ev_header_v2 *)ev.u.data)

#define CAM_SYNC_STATE_INVALID                   0
#define CAM_SYNC_STATE_ACTIVE                    1
#define CAM_SYNC_STATE_SIGNALED_SUCCESS          2
#define CAM_SYNC_STATE_SIGNALED_ERROR            3
#define CAM_SYNC_STATE_SIGNALED_CANCEL           4

/* Top level common sync event reason types */
#define CAM_SYNC_COMMON_EVENT_START       0
#define CAM_SYNC_COMMON_EVENT_UNUSED      (CAM_SYNC_COMMON_EVENT_START + 0)
#define CAM_SYNC_COMMON_EVENT_SUCCESS     (CAM_SYNC_COMMON_EVENT_START + 1)
#define CAM_SYNC_COMMON_EVENT_FLUSH       (CAM_SYNC_COMMON_EVENT_START + 2)
#define CAM_SYNC_COMMON_EVENT_STOP        (CAM_SYNC_COMMON_EVENT_START + 3)
#define CAM_SYNC_COMMON_EVENT_SYNX        (CAM_SYNC_COMMON_EVENT_START + 4)
#define CAM_SYNC_COMMON_REG_PAYLOAD_EVENT (CAM_SYNC_COMMON_EVENT_START + 5)
#define CAM_SYNC_COMMON_SYNC_SIGNAL_EVENT (CAM_SYNC_COMMON_EVENT_START + 6)
#define CAM_SYNC_COMMON_RELEASE_EVENT     (CAM_SYNC_COMMON_EVENT_START + 7)
#define CAM_SYNC_COMMON_EVENT_END         (CAM_SYNC_COMMON_EVENT_START + 50)

/* ISP Sync event reason types */
#define CAM_SYNC_ISP_EVENT_START                     (CAM_SYNC_COMMON_EVENT_END + 1)
#define CAM_SYNC_ISP_EVENT_UNKNOWN                   (CAM_SYNC_ISP_EVENT_START + 0)
#define CAM_SYNC_ISP_EVENT_BUBBLE                    (CAM_SYNC_ISP_EVENT_START + 1)
#define CAM_SYNC_ISP_EVENT_OVERFLOW                  (CAM_SYNC_ISP_EVENT_START + 2)
#define CAM_SYNC_ISP_EVENT_P2I_ERROR                 (CAM_SYNC_ISP_EVENT_START + 3)
#define CAM_SYNC_ISP_EVENT_VIOLATION                 (CAM_SYNC_ISP_EVENT_START + 4)
#define CAM_SYNC_ISP_EVENT_BUSIF_OVERFLOW            (CAM_SYNC_ISP_EVENT_START + 5)
#define CAM_SYNC_ISP_EVENT_FLUSH                     (CAM_SYNC_ISP_EVENT_START + 6)
#define CAM_SYNC_ISP_EVENT_HW_STOP                   (CAM_SYNC_ISP_EVENT_START + 7)
#define CAM_SYNC_ISP_EVENT_RECOVERY_OVERFLOW         (CAM_SYNC_ISP_EVENT_START + 8)
#define CAM_SYNC_ISP_EVENT_CSID_OUTPUT_FIFO_OVERFLOW (CAM_SYNC_ISP_EVENT_START + 9)
#define CAM_SYNC_ISP_EVENT_CSID_RX_ERROR             (CAM_SYNC_ISP_EVENT_START + 10)
#define CAM_SYNC_ISP_EVENT_CSID_SENSOR_SWITCH_ERROR  (CAM_SYNC_ISP_EVENT_START + 11)
#define CAM_SYNC_ISP_EVENT_END                       (CAM_SYNC_ISP_EVENT_START + 50)

/* ICP Sync event reason types */
#define CAM_SYNC_ICP_EVENT_START                 (CAM_SYNC_ISP_EVENT_END + 1)
#define CAM_SYNC_ICP_EVENT_UNKNOWN               (CAM_SYNC_ICP_EVENT_START + 0)
#define CAM_SYNC_ICP_EVENT_FRAME_PROCESS_FAILURE (CAM_SYNC_ICP_EVENT_START + 1)
#define CAM_SYNC_ICP_EVENT_CONFIG_ERR            (CAM_SYNC_ICP_EVENT_START + 2)
#define CAM_SYNC_ICP_EVENT_NO_MEMORY             (CAM_SYNC_ICP_EVENT_START + 3)
#define CAM_SYNC_ICP_EVENT_BAD_STATE             (CAM_SYNC_ICP_EVENT_START + 4)
#define CAM_SYNC_ICP_EVENT_BAD_PARAM             (CAM_SYNC_ICP_EVENT_START + 5)
#define CAM_SYNC_ICP_EVENT_BAD_ITEM              (CAM_SYNC_ICP_EVENT_START + 6)
#define CAM_SYNC_ICP_EVENT_INVALID_FORMAT        (CAM_SYNC_ICP_EVENT_START + 7)
#define CAM_SYNC_ICP_EVENT_UNSUPPORTED           (CAM_SYNC_ICP_EVENT_START + 8)
#define CAM_SYNC_ICP_EVENT_OUT_OF_BOUND          (CAM_SYNC_ICP_EVENT_START + 9)
#define CAM_SYNC_ICP_EVENT_TIME_OUT              (CAM_SYNC_ICP_EVENT_START + 10)
#define CAM_SYNC_ICP_EVENT_ABORTED               (CAM_SYNC_ICP_EVENT_START + 11)
#define CAM_SYNC_ICP_EVENT_HW_VIOLATION          (CAM_SYNC_ICP_EVENT_START + 12)
#define CAM_SYNC_ICP_EVENT_CMD_ERROR             (CAM_SYNC_ICP_EVENT_START + 13)
#define CAM_SYNC_ICP_EVENT_HFI_ERR_COMMAND_SIZE  (CAM_SYNC_ICP_EVENT_START + 14)
#define CAM_SYNC_ICP_EVENT_HFI_ERR_MESSAGE_SIZE  (CAM_SYNC_ICP_EVENT_START + 15)
#define CAM_SYNC_ICP_EVENT_HFI_ERR_QUEUE_EMPTY   (CAM_SYNC_ICP_EVENT_START + 16)
#define CAM_SYNC_ICP_EVENT_HFI_ERR_QUEUE_FULL    (CAM_SYNC_ICP_EVENT_START + 17)
#define CAM_SYNC_ICP_EVENT_END                   (CAM_SYNC_ICP_EVENT_START + 50)

/* JPEG Sync event reason types */
#define CAM_SYNC_JPEG_EVENT_START               (CAM_SYNC_ICP_EVENT_END + 1)
#define CAM_SYNC_JPEG_EVENT_UNKNOWN             (CAM_SYNC_JPEG_EVENT_START + 0)
#define CAM_SYNC_JPEG_EVENT_INVLD_CMD           (CAM_SYNC_JPEG_EVENT_START + 1)
#define CAM_SYNC_JPEG_EVENT_SET_IRQ_CB          (CAM_SYNC_JPEG_EVENT_START + 2)
#define CAM_SYNC_JPEG_EVENT_HW_RESET_FAILED     (CAM_SYNC_JPEG_EVENT_START + 3)
#define CAM_SYNC_JPEG_EVENT_CDM_CHANGE_BASE_ERR (CAM_SYNC_JPEG_EVENT_START + 4)
#define CAM_SYNC_JPEG_EVENT_CDM_CONFIG_ERR      (CAM_SYNC_JPEG_EVENT_START + 5)
#define CAM_SYNC_JPEG_EVENT_START_HW_ERR        (CAM_SYNC_JPEG_EVENT_START + 6)
#define CAM_SYNC_JPEG_EVENT_START_HW_HANG       (CAM_SYNC_JPEG_EVENT_START + 7)
#define CAM_SYNC_JPEG_EVENT_MISR_CONFIG_ERR     (CAM_SYNC_JPEG_EVENT_START + 8)
#define CAM_SYNC_JPEG_EVENT_END                 (CAM_SYNC_JPEG_EVENT_START + 50)

/* FD Sync event reason types */
#define CAM_SYNC_FD_EVENT_START           (CAM_SYNC_JPEG_EVENT_END + 1)
#define CAM_SYNC_FD_EVENT_UNKNOWN         (CAM_SYNC_FD_EVENT_START + 0)
#define CAM_SYNC_FD_EVENT_IRQ_FRAME_DONE  (CAM_SYNC_FD_EVENT_START + 1)
#define CAM_SYNC_FD_EVENT_IRQ_RESET_DONE  (CAM_SYNC_FD_EVENT_START + 2)
#define CAM_SYNC_FD_EVENT_HALT            (CAM_SYNC_FD_EVENT_START + 3)
#define CAM_SYNC_FD_EVENT_END             (CAM_SYNC_FD_EVENT_START + 50)

/* LRME Sync event reason types */
#define CAM_SYNC_LRME_EVENT_START           (CAM_SYNC_FD_EVENT_END + 1)
#define CAM_SYNC_LRME_EVENT_UNKNOWN         (CAM_SYNC_LRME_EVENT_START + 0)
#define CAM_SYNC_LRME_EVENT_CB_ERROR        (CAM_SYNC_LRME_EVENT_START + 1)
#define CAM_SYNC_LRME_EVENT_END             (CAM_SYNC_LRME_EVENT_START + 50)

/* OPE Sync event reason types */
#define CAM_SYNC_OPE_EVENT_START              (CAM_SYNC_LRME_EVENT_END + 1)
#define CAM_SYNC_OPE_EVENT_UNKNOWN            (CAM_SYNC_OPE_EVENT_START + 0)
#define CAM_SYNC_OPE_EVENT_PAGE_FAULT         (CAM_SYNC_OPE_EVENT_START + 1)
#define CAM_SYNC_OPE_EVENT_HW_HANG            (CAM_SYNC_OPE_EVENT_START + 2)
#define CAM_SYNC_OPE_EVENT_HALT               (CAM_SYNC_OPE_EVENT_START + 3)
#define CAM_SYNC_OPE_EVENT_CONFIG_ERR         (CAM_SYNC_OPE_EVENT_START + 4)
#define CAM_SYNC_OPE_EVENT_HW_FLUSH           (CAM_SYNC_OPE_EVENT_START + 5)
#define CAM_SYNC_OPE_EVENT_HW_RESUBMIT        (CAM_SYNC_OPE_EVENT_START + 6)
#define CAM_SYNC_OPE_EVENT_HW_RESET_DONE      (CAM_SYNC_OPE_EVENT_START + 7)
#define CAM_SYNC_OPE_EVENT_HW_ERROR           (CAM_SYNC_OPE_EVENT_START + 8)
#define CAM_SYNC_OPE_EVENT_INVLD_CMD          (CAM_SYNC_OPE_EVENT_START + 9)
#define CAM_SYNC_OPE_EVENT_HW_RESET_FAILED    (CAM_SYNC_OPE_EVENT_START + 10)
#define CAM_SYNC_OPE_EVENT_END                (CAM_SYNC_OPE_EVENT_START + 50)

/* CRE Sync event reason types */
#define CAM_SYNC_CRE_EVENT_START               (CAM_SYNC_OPE_EVENT_END + 1)
#define CAM_SYNC_CRE_EVENT_UNKNOWN             (CAM_SYNC_CRE_EVENT_START + 0)
#define CAM_SYNC_CRE_EVENT_CONFIG_ERR          (CAM_SYNC_CRE_EVENT_START + 1)
#define CAM_SYNC_CRE_EVENT_INVLD_CMD           (CAM_SYNC_CRE_EVENT_START + 2)
#define CAM_SYNC_CRE_EVENT_SET_IRQ_CB          (CAM_SYNC_CRE_EVENT_START + 3)
#define CAM_SYNC_CRE_EVENT_HW_RESET_FAILED     (CAM_SYNC_CRE_EVENT_START + 4)
#define CAM_SYNC_CRE_EVENT_HW_ERR              (CAM_SYNC_CRE_EVENT_START + 5)
#define CAM_SYNC_CRE_EVENT_END                 (CAM_SYNC_CRE_EVENT_START + 50)

#define CAM_SYNC_EVENT_MAX         8
#define CAM_SYNC_EVENT_REASON_CODE_INDEX  0

/* Fence types supported by the driver */
#define CAM_GENERIC_FENCE_TYPE_SYNC_OBJ     0x1
#define CAM_GENERIC_FENCE_TYPE_DMA_FENCE    0x2

/**
 * struct cam_sync_ev_header - Event header for sync event notification
 *
 * @sync_obj: Sync object
 * @status:   Status of the object
 */
struct cam_sync_ev_header {
	__s32 sync_obj;
	__s32 status;
};

/**
 * struct cam_sync_ev_header_v2 - Event header for sync event notification
 *
 * @sync_obj:    Sync object
 * @status:      Status of the object
 * @version:     sync driver version
 * @evt_param:   event parameter
 */
struct cam_sync_ev_header_v2 {
	__s32 sync_obj;
	__s32 status;
	uint32_t version;
	uint32_t evt_param[CAM_SYNC_EVENT_MAX];
};

/**
 * struct cam_sync_info - Sync object creation information
 *
 * @name:       Optional string representation of the sync object
 * @sync_obj:   Sync object returned after creation in kernel
 */
struct cam_sync_info {
	char  name[64];
	__s32 sync_obj;
};

/**
 * struct cam_sync_signal - Sync object signaling struct
 *
 * @sync_obj:   Sync object to be signaled
 * @sync_state: State of the sync object to which it should be signaled
 */
struct cam_sync_signal {
	__s32 sync_obj;
	__u32 sync_state;
};

/**
 * struct cam_sync_merge - Merge information for sync objects
 *
 * @sync_objs:  Pointer to sync objects
 * @num_objs:   Number of objects in the array
 * @merged:     Merged sync object
 */
struct cam_sync_merge {
	__u64 sync_objs;
	__u32 num_objs;
	__s32 merged;
};

/**
 * struct cam_sync_userpayload_info - Payload info from user space
 *
 * @sync_obj:   Sync object for which payload has to be registered for
 * @reserved:   Reserved
 * @payload:    Pointer to user payload
 */
struct cam_sync_userpayload_info {
	__s32 sync_obj;
	__u32 reserved;
	__u64 payload[CAM_SYNC_USER_PAYLOAD_SIZE];
};

/**
 * struct cam_sync_wait - Sync object wait information
 *
 * @sync_obj:   Sync object to wait on
 * @reserved:   Reserved
 * @timeout_ms: Timeout in milliseconds
 */
struct cam_sync_wait {
	__s32    sync_obj;
	__u32    reserved;
	uint64_t timeout_ms;
};

/**
 * struct cam_generic_fence_config - Fence config
 *                    Based on the operation, fields could be
 *                    input/output
 *
 * @version:          Struct version
 * @name:             Optional string representation [used for create/import]
 * @fence_sel_mask:   Fence select mask, if set for fence types other than the type
 *                    this input is processed for, the corresponding types would be
 *                    processed as well. For example if one wants to import a sync
 *                    object for an existing dma fence, set mask |= CAM_GENERIC_FENCE_TYPE_SYNC_OBJ,
 *                    a new sync object would be returned in sync_obj linked to an
 *                    existing dma_fence_fd.
 * @sync_obj:         Sync object
 * @dma_fence_fd:     DMA fence fd
 * @synx_obj:         Synx object
 * @reason_code:      Indicates if the operation was successful or not
 *                    for this set of fences. It is the responsibility of
 *                    the caller to clean up any partially processed batched
 *                    fences
 * @num_valid_params: Valid number of params being used
 * @valid_param_mask: Mask to indicate the field types in params
 * @params:           Additional params
 */
struct cam_generic_fence_config {
	__u32 version;
	char name[64];
	__u32 fence_sel_mask;
	__s32 sync_obj;
	__s32 dma_fence_fd;
	__s32 synx_obj;
	__s32 reason_code;
	__u32 num_valid_params;
	__u32 valid_param_mask;
	__u32 params[4];
};

/**
 * struct cam_dma_fence_signal - DMA fence signaling info
 *
 * @version:          Struct version
 * @dma_fence_fd:     DMA fence to be signaled
 * @status:           Any status if applicable, 0 for success
 * @num_valid_params: Valid number of params being used
 * @valid_param_mask: Mask to indicate the field types in params
 * @params:           Additional params
 */
struct cam_dma_fence_signal {
	__u32 version;
	__s32 dma_fence_fd;
	__s32 status;
	__u32 num_valid_params;
	__u32 valid_param_mask;
	__s32 params[3];
};

/**
 * struct cam_generic_fence_input_info - Parent structure that
 *                    provides info on fence batching
 *
 * @version:                Struct version
 * @num_fences_requested:   Number of fences to process
 * @num_fences_processed:   Number of fences processed
 *                          If userland requests 5 fences to be created
 *                          and it fails on the 3rd, num processed will be
 *                          3. This can be used by userspace to clean
 *                          partially batched fences
 * @num_valid_params:       Valid number of params being used
 * @valid_param_mask:       Mask to indicate the field types in params
 * @params:                 Additional params
 * @fence_info:             Variable length fence info input based on num_fences
 */
struct cam_generic_fence_input_info {
	__u32 version;
	__u32 num_fences_requested;
	__u32 num_fences_processed;
	__u32 num_valid_params;
	__u32 valid_param_mask;
	__s32 params[3];
	struct cam_generic_fence_config fence_cfg[1];
};

/**
 * struct cam_generic_fence_cmd_args - Generic fence cmd args
 *
 * @version:           Struct version
 * @fence_type:        Type of fence the ioctl cmd is for [dma/sync/synx]
 * @input_handle_type: Type of the fence input handle [user handle is expected]
 * @input_data_size:   Size of the data pointed to by input_handle
 * @input_handle:      Handle to the fence input data [create/signal/import...]
 *                     corresponding to the fence_type
 * @num_valid_params:  Valid number of reserved params being used
 * @valid_param_mask:  Mask to indicate the field types in params
 * @params:            Additional params
 */
struct cam_generic_fence_cmd_args {
	__u32 version;
	__u32 fence_type;
	__u32 input_handle_type;
	__u32 input_data_size;
	__u64 input_handle;
	__u32 num_valid_params;
	__u32 valid_param_mask;
	__u32 params[6];
};

/**
 * struct cam_private_ioctl_arg - Sync driver ioctl argument
 *
 * @id:         IOCTL command id
 * @size:       Size of command payload
 * @result:     Result of command execution
 * @reserved:   Reserved
 * @ioctl_ptr:  Pointer to user data
 */
struct cam_private_ioctl_arg {
	__u32 id;
	__u32 size;
	__u32 result;
	__u32 reserved;
	__u64 ioctl_ptr;
};

#define CAM_PRIVATE_IOCTL_CMD \
	_IOWR('V', BASE_VIDIOC_PRIVATE, struct cam_private_ioctl_arg)

/* Exclusive sync object IOCTL cmds */
#define CAM_SYNC_CREATE                          0
#define CAM_SYNC_DESTROY                         1
#define CAM_SYNC_SIGNAL                          2
#define CAM_SYNC_MERGE                           3
#define CAM_SYNC_REGISTER_PAYLOAD                4
#define CAM_SYNC_DEREGISTER_PAYLOAD              5
#define CAM_SYNC_WAIT                            6

/* Generic fence [sync/dma/synx] IOCTL cmds */
#define CAM_GENERIC_FENCE_CREATE                 11
#define CAM_GENERIC_FENCE_RELEASE                12
#define CAM_GENERIC_FENCE_IMPORT                 13
#define CAM_GENERIC_FENCE_SIGNAL                 14

#endif /* __UAPI_CAM_SYNC_H__ */
