/*
 * Copyright (c) 2021 ZEKU Technology(Shanghai) Corp.,Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Basecode Created :        2020/10/11 Author: zf@zeku.com
 *
 */

#ifndef _EXPLORER_IPC_H
#define _EXPLORER_IPC_H

#include "hal_protocol.h"
#include "sdio_pi.h"
#include "uapi/explorer_uapi.h"

/* #define	IPC_SOFT_DEBUG */
#define	IPC_API_VERSION			("0.9.0")

/*
  * ipc mem index
  */
enum ipc_mempool_index {
	IPC_MEM_AP2CC_CMD	=	0,
	IPC_MEM_AP2CC_DATA	=	1,
	IPC_MEM_CC2AP_CMD	=	2,
	IPC_MEM_CC2AP_DATA	=	3,
	IPC_MEM_MAX	=	4,
};

/*
  * ipc memory pool.
  */
#define	IPC_MEM_AP2CC_CMD_BASE		(0x17C800)
#define	IPC_MEM_AP2CC_CMD_PAGE_SIZE		(0x80)		/* 128B per ipc cmd buffer */
#define	IPC_MEM_AP2CC_CMD_PAGE_POS		(0)		/* bitmap pos 0 */
#define	IPC_MEM_AP2CC_CMD_PAGE_NUM		(0x20)		/* 32 ipc cmd buffer */
#define	IPC_MEM_AP2CC_CMD_SIZE		(IPC_MEM_AP2CC_CMD_PAGE_SIZE*IPC_MEM_AP2CC_CMD_PAGE_NUM)	/* 4096B */

#define	IPC_MEM_AP2CC_DATA_BASE		(IPC_MEM_AP2CC_CMD_BASE+IPC_MEM_AP2CC_CMD_SIZE)	/* base+0x1000 */
#define	IPC_MEM_AP2CC_DATA_PAGE_SIZE		(0x100)		/* 256B per ipc data buffer */
#define	IPC_MEM_AP2CC_DATA_PAGE_POS		(32)		/* bitmap pos 32 */
#define	IPC_MEM_AP2CC_DATA_PAGE_NUM		(0x18)		/* 24 ipc data buffer */
#define	IPC_MEM_AP2CC_DATA_SIZE		(IPC_MEM_AP2CC_DATA_PAGE_SIZE*IPC_MEM_AP2CC_DATA_PAGE_NUM)	/* 6144B */

#define	IPC_MEM_CC2AP_CMD_BASE		(IPC_MEM_AP2CC_DATA_BASE+IPC_MEM_AP2CC_DATA_SIZE)	/* base+0x2800 */
#define	IPC_MEM_CC2AP_CMD_PAGE_SIZE		(0x80)		/* 128B per ipc cmd buffer */
#define	IPC_MEM_CC2AP_CMD_PAGE_POS		(64)		/* bitmap pos 64 */
#define	IPC_MEM_CC2AP_CMD_PAGE_NUM		(0x20)		/* 32 ipc cmd buffer */
#define	IPC_MEM_CC2AP_CMD_SIZE		(IPC_MEM_CC2AP_CMD_PAGE_SIZE*IPC_MEM_CC2AP_CMD_PAGE_NUM)	/* 4096B */

#define	IPC_MEM_CC2AP_DATA_BASE		(IPC_MEM_CC2AP_CMD_BASE+IPC_MEM_CC2AP_CMD_SIZE)	/* base+0x3800 */
#define	IPC_MEM_CC2AP_DATA_PAGE_SIZE		(0x100)		/* 256B per ipc data buffer */
#define	IPC_MEM_CC2AP_DATA_PAGE_POS		(96)		/* bitmap pos 96 */
#define	IPC_MEM_CC2AP_DATA_PAGE_NUM		(0x18)		/* 24 ipc cmd buffer */
#define	IPC_MEM_CC2AP_DATA_SIZE		(IPC_MEM_CC2AP_DATA_PAGE_SIZE*IPC_MEM_CC2AP_DATA_PAGE_NUM)	/* 6144B */

#define	IPC_MEM_AP2CC_BITMAP_LEN	((IPC_MEM_AP2CC_CMD_PAGE_NUM+IPC_MEM_AP2CC_DATA_PAGE_NUM+32-1)/32*4)
#define	IPC_MEM_CC2AP_BITMAP_LEN	((IPC_MEM_CC2AP_CMD_PAGE_NUM+IPC_MEM_CC2AP_DATA_PAGE_NUM+32-1)/32*4)

#define	IPC_MP_DATA_MAX_LENGTH		(IPC_MEM_AP2CC_DATA_SIZE)

#define	IPC_MEM_FREE_WATERMARK		(4096)

/* generic data max size */
#define	IPC_GEN_DATA_SIZE_MAX		(16*1024*1024)

/* syscall read data max size */
#define	SC_READ_DATA_SIZE_MAX		(2*1024*1024)

/**
  * ipc address range
 */
enum ipc_memrange_index {
	IPC_MEM_RANGE0	=	0,
	IPC_MEM_RANGE1	=	1,
	IPC_MEM_RANGE_MAX	=	2,
};

#define	IPC_SRAM_RANGE_MIN		(0x100000)
#define	IPC_SRAM_RANGE_MAX		(0x180000)

#define	IPC_DDR_RANGE_MIN		(0x20000000)
#define	IPC_DDR_RANGE_MAX		(0x30000000)

/* ipc retry count */
#define	IPC_RETRY_COUNT			(3)

struct ipc_mempage {
	u32 pool_index	:	8;
	u32 page_index	:	8;
	u32 page_num	:	8;
	u32 reserved	:	8;
};

struct ipc_mempool {
	u32 base_addr;
	enum ipc_mempool_index pool_index;
	u32 page_pos;
	u32 page_num;
	u32 page_size;
	u32 bitmap_cached;
	u32 bitmap_mask;
	void *private_data;
	int (*alloc_page)(struct ipc_mempool *, u32, struct ipc_mempage *);	/* allocate page for self */
	int (*free_page)(struct ipc_mempool *, u32, struct ipc_mempage *);	/* free page for remote */
	int (*set_cached_bitmap)(struct ipc_mempool *);
	int (*get_free_space)(struct ipc_mempool *);
};

struct ipc_memrange {
	u32 addr_min;
	u32 addr_max;
};

/*
  * ipc buffer channel set/clear/status register
  */
#define	IPC_BUFCHAN_SET_BASE		(0x40020400)

#define	IPC_BUFCHAN_CLEAR_BASE		(0x40020410)
#define	IPC_BUFCHAN_CC2AP_CLEAR		(0x40020418)

#define	IPC_BUFCHAN_STATUS_BASE		(0x40020420)
#define	IPC_BUFCHAN_NUM		(0x4)
#define	IPC_BUFCHAN_LEN		(IPC_BUFCHAN_NUM*4)

/*
  * mbox data item
  */
union mbox_data {
	struct hal_comm_data comm_data;
	u32 fifo[COMM_BUF_NUM];
};

struct mbox_work {
	struct work_struct		worker;
	u32		work_id;
	u32		msg_num;
	void		*userdata;
};

/**
  * ipc sync reply
  */
 enum ipc_reply_addr_type {
	IPC_KERNEL	=	0,
	IPC_USER	=	1,
};

struct ipc_sync_reply {
	unsigned int size;
	void *buffer;
	enum ipc_reply_addr_type aflag;
};

/**
  * ipc xfer mode
  */
 enum ipc_xfer_mode {
	IPC_SINGLE_XFER	=	0,
	IPC_BATCH_XFER	=	1,
};

/**
  * ipc netlink header to userspace
  */
struct ipc_genl_header {
	unsigned int id;
	unsigned int size;
};

/**
  * ipc payload data positon
  */
#define	IPC_PLD_IN_MEMPOOL		(0)
#define	IPC_PLD_IN_MEMORY		(1)
#define	IPC_PLD_IN_MBOXFIFO		(2)

/*
  * mbox memory mapping
  */
#define	MBOX_BASE				(0x50000000)
#define	MBOX_INT_CLR_OFFSET				(0x4)
#define	MBOX_CPU_MSGNUM_OFFSET				(0x30)
#define	MBOX_AP_MSGNUM_OFFSET				(0x38)
#define	MBOX_WR_AP2CPU_OFFSET				(0x50)
#define	MBOX_RD_AP_OFFSET				(0xf0)

#define	MBOX_INT_CLR				(MBOX_BASE+MBOX_INT_CLR_OFFSET)
#define	MBOX_INT_CLR_AP2CC_INT				(0x4)

#define	MBOX_CPU_MSGNUM				(MBOX_BASE+MBOX_CPU_MSGNUM_OFFSET)

#define	MBOX_AP_MSGNUM				(MBOX_BASE+MBOX_AP_MSGNUM_OFFSET)
#define	MBOX_WR_AP2CPU				(MBOX_BASE+MBOX_WR_AP2CPU_OFFSET)
#define	MBOX_RD_AP				(MBOX_BASE+MBOX_RD_AP_OFFSET)

/* max work count */
#define	MBOX_WORK_MAX_NUM		(16)

#define	MBOX_FIFO_DEPTH_MAX		(32)
#define	MBOX_RETRY_MAX		(5000)

#define FULLDUMP_SIZE 0x10000000
#define SDI_DDRLP2_MSG 0x5a5a5a5a

/**
  * generic netlink maximum payload size
  */
#define	GENL_PAYLOAD_TOTAL_MAX_LENGTH		(1024*1024)

/* for 8350,  default max length per frame is 2068, including 24B nl header */
#define	GENL_PAYLOAD_FRAME_MAX_LENGTH		(2000)

struct explorer_plat_data;
int explorer_init_ipc(struct explorer_plat_data *epd);
int explorer_mbox_irq_handler(struct explorer_plat_data *epd);
int explorer_ipc_set_top_bitmap(struct explorer_plat_data *epd, struct ipc_mempage *free_page);
int explorer_ipc_clear_top_bitmap(struct explorer_plat_data *epd, struct ipc_mempage *free_page);
int explorer_hal_sync_write(struct explorer_plat_data *epd, u32 cc_addr, void *ap_buffer, u32 len);
int explorer_hal_sync_read(struct explorer_plat_data *epd, u32 cc_addr, void *ap_buffer, u32 len);
int explorer_hal_sync_write_internal_nolock(struct explorer_plat_data *epd,u32 regoffset, u8 in);
int explorer_hal_sync_read_internal_nolock(struct explorer_plat_data *epd,u32 regoffset, u8 *out);
int explorer_hal_sync_write_internal(struct explorer_plat_data *epd,u32 regoffset, u8 in);
int explorer_hal_sync_read_internal(struct explorer_plat_data *epd,u32 regoffset, u8 *out);
bool explorer_hal_get_hw_status(struct explorer_plat_data *epd);
int explorer_write_mbox(struct explorer_plat_data *epd, union mbox_data *data, int num);
int explorer_read_mbox(struct explorer_plat_data *epd, union mbox_data *data, int num);
int explorer_send_mbox_nowait(struct explorer_plat_data *epd, u32 mod_id, u32 sub_id, void *data);
int explorer_send_mbox_wait(struct explorer_plat_data *epd, u32 mod_id, u32 sub_id,
				  void *data, struct ipc_sync_reply *reply);
int explorer_reply_mbox(struct explorer_plat_data *epd, u32 mod_id, u32 sub_id, void *data);
int explorer_write_ipc_data_wait(struct explorer_plat_data *epd, u32 ipc_id,
				     void *ap_buffer, u32 len, struct explorer_sync_reply *reply);
int explorer_write_ipc_data_nowait(struct explorer_plat_data *epd, u32 ipc_id, void *ap_buffer, u32 len);
int explorer_write_data_wait(struct explorer_plat_data *epd, u32 ipc_id, u32 cc_addr,
				void *ap_buffer, u32 len, struct explorer_sync_reply *reply);
int explorer_write_data_nowait(struct explorer_plat_data *epd, u32 ipc_id, u32 cc_addr,
				   void *ap_buffer, u32 len);
int explorer_write_generic_data(struct explorer_plat_data *epd, void *ap_buffer,
				    u32 len, struct explorer_gen_data_header *gen_header);
int explorer_write_generic_cmd(struct explorer_plat_data *epd, void *ap_buffer,
				    u32 len, struct explorer_gen_data_header *gen_header);
int explorer_write_generic_cmd_kernel(struct explorer_plat_data *epd, void *ap_buffer,
				    u32 len, struct explorer_gen_data_header *gen_header);
int explorer_write_hclk_data(struct explorer_plat_data *epd, unsigned int data);
int explorer_write_power_data(struct explorer_plat_data *epd, unsigned int data0, unsigned int data1);
void explorer_dinit_ipc(struct explorer_plat_data *epd);
int explorer_hal_sync_batch_write(struct explorer_plat_data *epd, struct batch_data_item *ap_buffer, u32 count);

int explorer_genetlink_init(struct explorer_plat_data *epd);
int explorer_genetlink_exit(struct explorer_plat_data *epd);
int explorer_genl_mcast_data(struct explorer_plat_data *epd, u32 id, void *ap_buffer, u32 size);
int explorer_execute_tuning(struct explorer_plat_data *explorer_data);
#ifndef ZEKU_EXPLORER_PLATFORM_RPI
u32 explorer_get_project(void);
#endif
int explorer_send_project_id(struct explorer_plat_data *epd);
int explorer_write_project_id(struct explorer_plat_data *epd);

#endif /* _EXPLORER_IPC_H */
