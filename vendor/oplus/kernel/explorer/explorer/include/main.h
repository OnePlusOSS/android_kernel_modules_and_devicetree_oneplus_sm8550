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
 * Basecode Created :        2020/1/20 Author: zf@zeku.com
 *
 */

#ifndef _EXPLORER_MAIN_H
#define _EXPLORER_MAIN_H

#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#ifndef OPLUS_EXPLORER_PLATFORM_QCOM
#include <linux/time.h>
#endif
#include <linux/rtc.h>
#include "spi.h"
#include "ipc.h"
#include "irq.h"
#include "sdio_pi.h"
#include "hal_protocol.h"
#include "isp.h"
#include "exception.h"
#include "uapi/explorer_uapi.h"

#ifdef MTK_AON
#include "../../aon/mtk/include/mtk_aon_sensor_core.h"
#endif

#ifdef QCOM_AON
#include "../../aon/qcom/include/qcom_aon_sensor_core.h"
#endif

/*
 * mmap shared buffer total size 256 pages, layout:
 *  - isp tunning data, 150 pages
 *  - camera sending data, 2 pages
 *  - log buffer, 2 pages
 */
#define	USER_BUFFER_PAGES		(8)	/* 256 pages */

#define	SHMEM_SYNC_REPLY_BASE(base)		((u8 *)base)
#define	IPC_SYNC_REPLY_SIZE		(16*PAGE_SIZE)	/* 16 pages */

#define	SHMEM_IPC_BATCH_OFFSET		(IPC_SYNC_REPLY_SIZE)
#define	SHMEM_IPC_BATCH_BASE(base)		((u8 *)base+SHMEM_IPC_BATCH_OFFSET)
#define	IPC_BATCH_SIZE		(16*PAGE_SIZE)	/* 16 pages */

#define	SHMEM_RDBUF_OFFSET		(IPC_SYNC_REPLY_SIZE+IPC_BATCH_SIZE)
#define	SHMEM_RDBUF_BASE(base)		((u8 *)base+SHMEM_RDBUF_OFFSET)
#define	SC_READ_BUF_SIZE		(64*PAGE_SIZE)	/* 64 pages */
#define RAMDUMP_BUFFER_SIZE 0x80000

/* bus type */
enum ipc_bus_type {
	IPC_BUS_CSPI = 0,
	IPC_BUS_SDIO = 1,
	IPC_BUS_MAX = 2,
};

enum pw_mode {
	PW_MODE_POWEROFF,
	PW_MODE_MIPI_BYPASS,
	PW_MODE_DSP_DISABLED,
};

struct explorer_plat_data;

#ifdef OPLUS_EXPLORER_NO_PROJECT
struct timeval {
	__kernel_old_time_t     tv_sec;         /* seconds */
	__kernel_suseconds_t    tv_usec;        /* microseconds */
};

static inline void rtc_time_to_tm(unsigned long time, struct rtc_time *tm)
{
	rtc_time64_to_tm(time, tm);
}

static inline int rtc_tm_to_time(struct rtc_time *tm, unsigned long *time)
{
	*time = rtc_tm_to_time64(tm);

	return 0;
}
#endif

struct explorer_ops{
	int (*send_uevent)(struct explorer_plat_data *osd, char *uevent_buf);
	u64 (*get_timestamp)(void);
	int (*load_firmware)(struct explorer_plat_data *hpd, const char *firmware_name,
	                     u32 addr);
	int (*load_and_start_dsp)(struct explorer_plat_data *hpd);
	int (*power_on)(struct explorer_plat_data *plat_priv, enum pw_mode mode);
	void (*power_off)(struct explorer_plat_data *plat_priv);
	bool (*is_power_on)(struct explorer_plat_data *plat_priv);
	int (*power_on_and_off)(struct explorer_plat_data *plat_priv);
	int (*send_user_data)(struct explorer_plat_data *hpd, void *buffer, uint32_t aligned_size);
};

struct explorer_boot_status {
	atomic_t current_stage;
	atomic_t rtos_on;
	enum explorer_os_type cur_os_type;
	bool is_cc_sec_boot;
	atomic_t flashable;
	unsigned long start_jiffies; /*start Jiffies of this boot*/
#ifdef SLT_ENABLE
	bool slt_has_booted;
#endif
};

#ifndef OPLUS_EXPLORER_PLATFORM_QCOM
//typedef __s64 time64_t;
struct timeval_sdio {
	time64_t tv_sec;
	long tv_usec;
};
#endif

struct explorer_plat_data {
	struct platform_device *plat_dev;
	struct miscdevice dev;
	struct explorer_ops *ops;
	struct cspi_data *cspi;
	struct explorer_sdio_data *sdio_data;
#if defined (QCOM_AON) || defined (MTK_AON)
	struct aon_sensor_ctrl_t *aon_data;
#endif
	enum ipc_bus_type bus_type;
	bool genl;
	struct mutex genl_lock;		/* netlink mutex */

	/* sysfs */
	struct class *ns_class;
	struct device *explorer_dev;
	struct semaphore semphore;
	u32 pbs1_gpio;
	u32 pbs2_gpio;
	u32 pwoff_gpio;
	struct clk *rf_clk2;
	enum pw_mode pw_stat;

	int (*explorer_intc0_ivt[INTC0_MAX_IRQ])(struct explorer_plat_data *epd);	/* intc interrupt vector table */

	struct workqueue_struct *mbox_msg_wq;	/* workqueue to process mbox msg */
	struct mbox_work mbox_msg_work[MBOX_WORK_MAX_NUM];	/* schedule this work to process mbox msg */
	struct	mutex		mbox_rlock;		/* protect mbox */
	struct	mutex		mbox_wlock;		/* protect mbox */
	atomic_t mbox_ssindex[HAL_CMD_MAX];	/* mbox send sync cmd index */
	atomic_t mbox_saindex[HAL_CMD_MAX];	/* mbox send async cmd index */
	atomic_t mbox_rindex[HAL_CMD_MAX];	/* mbox recv cmd index */
	u32 mbox_cached_count;

	struct ipc_mempool ipc_mem_pool[IPC_MEM_MAX];
	u32 ipc_mem_bitmap[IPC_BUFCHAN_NUM];
	struct ipc_memrange ipc_mem_range[IPC_MEM_RANGE_MAX];
	spinlock_t ipc_page_slock;
	struct mutex comm_lock;		/* communication mutex */
	struct mutex ipc_sync_lock;
	void *sync_reply;	/* reply data of sync cmd */
	u32 sync_reply_size;
	enum ipc_xfer_mode ipc_mode;
	u32 project_id;

	void *shared_buffer;	/* shared buffer between kernel and usrland */
	struct	mutex		sc_rd_lock;		/* protect mbox */
	char *trans_buf;	/* transfer buffer for sdio interface */
	char *ap_buffer;	/* buffer for ramdump */

	int bsp_irq_gpio;				/* gpio number for bsp irq */
	int bsp_irq;					/* bsp irq number */
	int bsp_debug_gpio;				/* gpio number for bsp debug */
	int pmic_pon_gpio;				/* gpio number for pmic pon_1 */
	int pmic_reset_gpio;			/* gpio number for pmic reset */
	int explorer_sleep_gpio;		/* gpio number for Explorer dsleep, not used */
#ifndef ZEKU_EXPLORER_PLATFORM_RPI
#ifdef OPLUS_EXPLORER_PLATFORM_QCOM
 	struct clk *clk_ref;			/* Explorer ref clock */
#else
	const char *clk_ref; 			/* clk_ref for MTK platform */
#endif
#endif
	struct regulator *vcc_sdio;		/* Explorer sdio vcc */

	bool ignore_pmic;
	bool ignore_dsleep;

	bool firmware_loaded;		    /* whether cc already on */
	bool hw_probe_done;		/* whether hw(sdio/cspi) probe done */
	int tuning_retry_num;	/* times of tuning failures */

	atomic_t is_explorer_on;		/* Set to true when explorer on */
	bool is_pmic_pon;          /* Set to true when pmic power on */
	bool is_clock_on;          /* Set to true when clock on */
	bool is_vcc_sdio_on;       /* Set to true when vcc_sdio is on */
	bool is_sdu_clk_switched;  /* Set in hclk->sdclk and Clear in sdclk->hclk */
	bool is_cc_standby;        /* Set in sleep int and Clear in sdclk->hclk*/
	bool is_poweroff_skip;     /* Set for debug purpose */
	bool is_heartbeat_poweroff_skip;     /* Set for debug purpose */
	struct mutex power_sync_lock;
	struct completion sleep_completion;
	struct completion wake_completion;
	struct completion power_state_completion;    /* Completion used for power state switch */
	struct completion sdio_remove_completion;
	struct completion sdi_ddrlp2_completion;
	struct completion get_dumpinfo_completion;
	dump_info ramdump_info;
	bool is_get_dumpinfo;
	unsigned int completed_power_state;          /* Record the state of power state switching */
	unsigned int action_when_pmic_oc;        	/* Set true to power off explorer when pmic oc*/
	atomic_t is_pmic_oc;						/* Set to true when PMIC oc in PBL */
	atomic_t is_ddr_failed;						/* Set to true when ddr fail in PBL */
	bool heartbeat_started;
	struct delayed_work	heartbeat_detect_work;

	struct explorer_boot_info ebi; /*boot_info was passed down*/
	struct explorer_boot_status ebs; /*boot status was changed in kernel*/
	struct explorer_sdio_status est; /*sdio status was changed in kernel*/
	enum explorer_cam_mode cmod;
	struct explorer_npu_mod_info *nmod_info;
	enum boot_force_cfg boot_force;

	u32 cc_clk_drive_strength; /* explorer cc clk drive strength */
	u32 cc_cmd_drive_strength; /* explorer cc cmd drive strength */
	u32 cc_data_drive_strength; /* explorer cc cmd drive strength */
	uintptr_t status;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
        struct wake_lock suspend_lock;
#else
        struct wakeup_source *suspend_ws;
#endif

};

int explorer_send_uevent(struct explorer_plat_data *epd, char *uevent_buf);
extern int sdio_clock;
extern int tuning_clock;
extern int sdio_width;
extern int tuning_width;
extern int sdio_timing;
extern int tuning_timing;

#ifndef OPLUS_EXPLORER_PLATFORM_QCOM
static inline void rtc_time_to_tm(unsigned long time, struct rtc_time *tm)
{
	rtc_time64_to_tm(time, tm);
}
#endif

#endif /* _EXPLORER_MAIN_H */
