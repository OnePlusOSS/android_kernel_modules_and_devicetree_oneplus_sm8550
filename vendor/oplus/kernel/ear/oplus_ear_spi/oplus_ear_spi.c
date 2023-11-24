// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *    Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 */
#define dev_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/uio.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>

#include <linux/timekeeping.h>

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */

#define N_SPI_MINORS        32    /* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);

static_assert(N_SPI_MINORS > 0 && N_SPI_MINORS <= 256);
#define CLASS_NAME           "earspi"
static int SPIDEV_MAJOR;
/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *    is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK    (SPI_MODE_X_MASK | SPI_CS_HIGH \
        | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
        | SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
        | SPI_TX_QUAD | SPI_TX_OCTAL | SPI_RX_DUAL \
        | SPI_RX_QUAD | SPI_RX_OCTAL)
#define SPIDEV_MAX_DEVICE_NODE_COUNT        2

char *spi_device_node_info[SPIDEV_MAX_DEVICE_NODE_COUNT] = {
    "zeku_ear_spi0",
    "zeku_ear_spi1",
};
#define SPIDEV_IOCTL_PMGET				0x6BFF
#define SPIDEV_IOCTL_PMPUT				0x6BFE

#define SPIDEV_RW_TRANSFER_ARRAY_MAX    10
#define SPIDEV_RW_TRANSFER_BUF_LEN      4096

#define SPIDEV_RING_SLOT_NUM            16 * 2
#define SPIDEV_RING_SLOT_LEN            4096
#define SPIDEV_MALLOC_ALIGN             sizeof(void *)

#define SPIDEV_DEFAULT_SPEED            40000000
#define SPI_CSPI_VALUE_OFFSET           8
#define SPI_CSPI_FRAM_LEN               12
//#define EAR_AUDIO_FIFO_ADDRESS          0x2a000000
#define EAR_AUDIO_FIFO_ADDRESS          0x2a010000
#define EAR_AUDIO_FIFO_LEN              sizeof(fifo_t)
#define EAR_AUDIO_FIFO_OFFSET           sizeof(struct spi_message) + 3 * sizeof(struct spi_transfer) + (SPI_CSPI_FRAM_LEN  * 2) + EAR_AUDIO_FIFO_LEN + 4
#define EAR_AUDIO_FIFO_ASYNC_THRESHOLD  11532 * 3

#define EAR_AUDIO_IRQ_ADDRESS           0x4f030200
#define EAR_AUDIO_VALUE_LEN             8
#define EAR_AUDIO_IRQ_SRC_CORE          2
#define EAR_AUDIO_IRQ_SRC_CORE_SHIFT    5
#define EAR_AUDIO_MBOXMSG_CMD_SHIFT     8
#define MBOXMSG_CMD_HIFI_SPI            12
#define READ_EAR_AUDIO_FIFO_COUNT       4

typedef enum {
    FIFO_OK = 0,
    FIFO_ERR_START = 0,
    FIFO_ERR_PARA,
    FIFO_ERR_MEM,
    FIFO_ERR_FREE_NOT_ENOUGH,
    FIFO_ERR_DATA_NOT_ENOUGH,
} fifo_err_t;

struct spi_rw_transfer {
 __u64 tx;
 __u64 rx;
 __u64 ext_tx;
 __u16 tx_len;
 __u16 ext_tx_len;
 __u8 rx_offset;
 __u8 rx_ext_space:1;
 __u8 cs_change:1;
 __u8 res:6;
};

typedef struct {
    volatile int32_t wr_offset;
    volatile int32_t rd_offset;
    volatile int32_t len;
    volatile int32_t full_flag; // 1 -full
    unsigned int data;
} fifo_t;

struct ear_irq_msg {
    union {
    struct {
        u16 data0;
        u8 msg_type;
        u8 rev : 5;
        u8 src_core : 3;
    };
    u32 cmd;
    };
    u32 data;
};
struct spidev_ear_audio {
    struct spi_message *m;
    fifo_t fifo;
    struct mutex fifo_lock;
    struct completion m_completion;
    bool async;
} ear_audio;

struct spidev_data {
    dev_t        devt[SPIDEV_MAX_DEVICE_NODE_COUNT];
    spinlock_t    spi_lock;
    struct spi_device    *spi;
    struct list_head    device_entry;

    /* TX/RX buffers are NULL unless this device is open (users > 0) */
    struct mutex    buf_lock;
    unsigned    users;
    u8        *tx_buffer;
    u8        *rx_buffer;
    u8  *rw_transfer_buf;
    u32        speed_hz;
    unsigned int head;
    unsigned int tail;
    unsigned int ring_size;
    struct spidev_buff *bufs;
    struct mutex    ring_lock;
    
};

struct spidev_buf_operations {
    void (*release)(struct spidev_data *, struct spidev_buff *);
};

struct spidev_buff {
    void *buff;
    u32 offset;
    u32 len;
    const struct spidev_buf_operations *ops;
};

struct spidev_message_context {
    struct spi_message m;
    struct spidev_data *spidev;
    struct timespec64 ts_begin;
    struct timespec64 ts_end;
};

enum SpiOperationType {
    SPI_OPERATE_READ_INTER_REG,
    SPI_OPERATE_WRITE_INTER_REG,
    SPI_OPERATE_READ_EXT_SING_REG,
    SPI_OPERATE_READ_EXT_MULTI_DATA,
    SPI_OPERATE_WRITE_EXT_MULTI_DATA,
    SPI_OPERATE_WRITE_EXT_SING_REG,
    SPI_OPERATE_MAX,
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static size_t timeoutCount;
static size_t receiveAudioCount = 0;
static bool first = true;

#define SPIDEV_ERR(spidev, fmt, ...)    \
    dev_err(&spidev->dev, fmt, ##__VA_ARGS__)

#define SPIDEV_WARN(spidev, fmt, ...)    \
	dev_warn(&spidev->dev, fmt, ##__VA_ARGS__)


#define SPIDEV_INFO(spidev, fmt, ...)     \
    dev_info(&spidev->dev, fmt, ##__VA_ARGS__)

#define SPIDEV_DBG(spidev, fmt, ...)     \
    dev_info(&spidev->dev, fmt, ##__VA_ARGS__)

#ifdef XUEYING_CSPI_BIG_ENDIAN

#define SPIDEV_HTONL(buf, len) {};

#define SPIDEV_NTOHL(buf, len) {};

#else
#define SPIDEV_HTONL(buf, len)                                                          \
{                                                                                       \
int i;                                                                                  \
do {                                                                                    \
    for (i = 0; i < (len); i += 4) {                                                    \
        unsigned int *value = (unsigned int *)((buf) + i);                              \
        *value = htonl(*value);                                                         \
    }                                                                                   \
} while(0);                                                                             \
}
#define SPIDEV_NTOHL(buf, len)                                                          \
{                                                                                       \
int i;                                                                                  \
do {                                                                                    \
    for (i = 0; i < (len); i += 4) {                                                    \
        unsigned int *value = (unsigned int *)((buf) + i);                              \
        *value = ntohl(*value);                                                         \
    }                                                                                   \
} while(0);                                                                             \
}                            
#endif

/*-------------------------------------------------------------------------*/

static void spidev_free_ring(struct spidev_data    *spidev);
static void spidev_free_txrx(struct spidev_data * file_priv);

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
    int status;
    struct spi_device *spi;

    spin_lock_irq(&spidev->spi_lock);
    spi = spidev->spi;
    spin_unlock_irq(&spidev->spi_lock);

    if (spi == NULL)
    status = -ESHUTDOWN;
    else
    status = spi_sync(spi, message);

    if (status == 0)
    status = message->actual_length;

    return status;
}
static int spidev_async(struct spidev_data *spidev, struct spi_message *message)
{
    struct spi_device *spi;

    spi = spidev->spi;

    return spi_async(spi, message);
}


static void __attribute__((unused)) spidev_hex_dump(const char *string, const char *src, size_t length)
{
    int num = 0;
    printk("%s buf info(hex): \n", string);
    while (length--) {
    printk(KERN_CONT "%X ", *(src + num));
    num++;
    }
}

static void spidev_initCspi(enum SpiOperationType type, unsigned int addr, int len, u8 *cspi)
{
    switch (type)
    {
        case SPI_OPERATE_READ_EXT_MULTI_DATA:
        {
            cspi[0] = 0x10;
            cspi[1] = 0x00;
            break;
        }
    case SPI_OPERATE_READ_INTER_REG: {
    cspi[0] = 0x40;
    cspi[1] = 0x00;
    break;
    }
    case SPI_OPERATE_WRITE_EXT_SING_REG:
    case SPI_OPERATE_WRITE_EXT_MULTI_DATA: {
    cspi[0] = 0x80;
    cspi[1] = 0x00;
    break;
    }
    default:
    break;
    }

    cspi[2] = ((len >> 2) & 0xFF00) >> 8;
    cspi[3] = (len >> 2) & 0x00FF;
    cspi[4] = (addr & 0xFF000000) >> 24;
    cspi[5] = (addr & 0x00FF0000) >> 16;
    cspi[6] = (addr & 0x0000FF00) >> 8;
    cspi[7] = (addr & 0x000000FF);
    return;
}

/****************************************************************************************
 * @get fifo used length
 * param[in]fifo - fifo ptr
 * @return used length
 ***************************************************************************************/
static int32_t fifo_get_used_length(fifo_t *fifo)
{
    int32_t len;
    if ((fifo->full_flag == 1) && (fifo->wr_offset == fifo->rd_offset)) {
    len = fifo->len;
    } else {
    len = fifo->wr_offset - fifo->rd_offset;
    if (len < 0) {
        len += fifo->len;
    }
    }
    return len;
}

/****************************************************************************************
 *@get fifo free length
 *param[in]fifo - fifo ptr
 *@return free length
 ***************************************************************************************/
static int32_t fifo_get_free_length(fifo_t *fifo)
{
    int32_t used = fifo_get_used_length(fifo);
    return (fifo->len - used);
}
static void spidev_free_ear_audio(void)
{
    if (ear_audio.m) {
    kfree(ear_audio.m);
        ear_audio.m = NULL;
    }    
    return;
}

/****************************************************************************************
 * @move fifo wr ptr
 * param[in]fifo - fifo ptr
 * param[in]len - data len
 * @return void
 ***************************************************************************************/
static int32_t fifo_set_wr_offset(fifo_t *fifo, int32_t len)
{
    int32_t len_free = fifo_get_free_length(fifo);

    if (len > len_free) {
    return FIFO_ERR_FREE_NOT_ENOUGH;
    }

    fifo->wr_offset += len;
    if (fifo->wr_offset > fifo->len) {
    fifo->wr_offset -= fifo->len;
    } else if (fifo->wr_offset < 0) {
    fifo->wr_offset += fifo->len;
    }
    if ((fifo->wr_offset == fifo->rd_offset) && (len == len_free)) {
    fifo->full_flag = 1;
    }
    return FIFO_OK;
}

static bool spidev_check_read_ear_fifo(const struct spi_device *spidev, const bool isOpen,
                                                const bool isAsync, const fifo_t *before, fifo_t *after)
{
    if (!isOpen) {
        if ((after->data != before->data) ||
            (after->len != before->len)) {
            goto invalid_ear_fifo;
        }
        if (!isAsync && (before->wr_offset != after->wr_offset)) {
            SPIDEV_WARN(spidev, "Invalid ear wr offset value. wr: %u Sync read from B3 wr: %u",
                         before->wr_offset, after->wr_offset);
        }
    } else {
        if (after->wr_offset != after->rd_offset) {
            SPIDEV_WARN(spidev, "The B3 fifo is not empty. Sync read from B3 wr: %u rd: %u",
                         after->wr_offset, after->rd_offset);
        }
        if (!after->data || (after->len <= 0)) {
            goto invalid_ear_fifo;
        }
    }
    if (after->full_flag & 0xFFFFFFFE) {
        goto invalid_ear_fifo;
    }
    if ((after->rd_offset < 0) ||
        (after->rd_offset > after->len)) {
        goto invalid_ear_fifo;
    }
    if ((after->wr_offset < 0) ||
        (after->wr_offset > after->len)) {
        SPIDEV_WARN(spidev, "Invalid ear wr offset value. %s read from B3 wr: %u len: %u",
                    (isAsync? "Async" : "Sync"), after->wr_offset, after->len);
    }
    return true;
invalid_ear_fifo:

    SPIDEV_ERR(spidev,
            "Invalid ear fifo value. "
            "wr: %u rd: %u len: %u full: %d addr: 0x%x. "
            "%s read from B3 wr: %u rd: %u len: %u full: %d addr: 0x%x.",
            before->wr_offset, before->rd_offset, before->len, before->full_flag, before->data,
            (isAsync? "Async" : "Sync"), after->wr_offset, after->rd_offset, after->len, after->full_flag, after->data);
    return false;
}


static struct spi_message *
spidev_read_ear_fifo_message(struct spidev_data *spidev)
{
    struct spi_message *m;
    struct spi_transfer *t;
    u8 *buf_pos;
    int m_len = sizeof(struct spi_message) +
        3 * sizeof(struct spi_transfer) +
        (SPI_CSPI_FRAM_LEN + EAR_AUDIO_FIFO_LEN + 4) * 2;
    u8 *ear_message = kmalloc(m_len, GFP_KERNEL);

    if (!ear_message) {
    return NULL;
    }
    memset(ear_message, 0, m_len);
    m = (struct spi_message *)ear_message;
    t = (struct spi_transfer *)(m + 1);
    buf_pos = (u8 *)(t + 3);

    spi_message_init(m);

    spidev_initCspi(SPI_OPERATE_READ_EXT_MULTI_DATA, EAR_AUDIO_FIFO_ADDRESS,
        EAR_AUDIO_FIFO_LEN, buf_pos);
    t->tx_buf = buf_pos;
    t->len = SPI_CSPI_FRAM_LEN;
    t->speed_hz = spidev->speed_hz;
    t->cs_change = 1;
    buf_pos += t->len;
    spi_message_add_tail(t, m);
    t++;

    spidev_initCspi(SPI_OPERATE_READ_INTER_REG, 0x4e00d030, 4, buf_pos);
    t->tx_buf = buf_pos;
    t->len = SPI_CSPI_FRAM_LEN;
    t->speed_hz = spidev->speed_hz;
    t->cs_change = 1;
    buf_pos += t->len;
    spi_message_add_tail(t, m);
    t++;

    buf_pos[0] = 0x20;
    buf_pos[1] = 0x00;
    buf_pos[2] = ((EAR_AUDIO_FIFO_LEN >> 2) & 0xFF00) >> 8;
    buf_pos[3] = (EAR_AUDIO_FIFO_LEN >> 2) & 0x00FF;
    t->tx_buf = buf_pos;
    t->len = EAR_AUDIO_FIFO_LEN + 4;
    buf_pos += t->len;
    t->rx_buf = buf_pos;
    t->speed_hz = spidev->speed_hz;
    spi_message_add_tail(t, m);

    return m;
}

static void spidev_ear_fifo_ntoh(fifo_t *ear_fifo, char *data)
{
    fifo_t *fifo = (fifo_t *)data;
#ifdef XUEYING_CSPI_BIG_ENDIAN

    ear_fifo->wr_offset = fifo->wr_offset;
    ear_fifo->rd_offset = fifo->rd_offset;
    ear_fifo->len = fifo->len;
    ear_fifo->full_flag = fifo->full_flag;
    ear_fifo->data = fifo->data;
#else
    ear_fifo->wr_offset = ntohl(fifo->wr_offset);
    ear_fifo->rd_offset = ntohl(fifo->rd_offset);
    ear_fifo->len = ntohl(fifo->len);
    ear_fifo->full_flag = ntohl(fifo->full_flag);
    ear_fifo->data = ntohl(fifo->data);
#endif
    return;
}

static void spidev_ear_fifo_update(bool isAsync, fifo_t *ear_fifo, fifo_t *read_fifo)
{
    if (!isAsync) {
        ear_fifo->wr_offset = read_fifo->wr_offset;
    }
    ear_fifo->rd_offset = read_fifo->rd_offset;
    ear_fifo->len = read_fifo->len;
    ear_fifo->full_flag = read_fifo->full_flag;
    ear_fifo->data = read_fifo->data;
    return;
}


static ssize_t spidev_sync_read_ear_fifo(struct spidev_data *spidev, const bool isOpen,
                     fifo_t *ear_fifo)
{
    ssize_t status = -ENOMEM;
    struct spi_message *m = ear_audio.m;
    fifo_t read_fifo;
    int i;
    if (!m) {
        m = spidev_read_ear_fifo_message(spidev);
        if (!m) {
            return status;
        }
        ear_audio.m = m;
    }
    for (i = 0; (i < READ_EAR_AUDIO_FIFO_COUNT) && (status < 0); i++) {
        if (ear_audio.async) {
            mutex_unlock(&ear_audio.fifo_lock);
            wait_for_completion(&ear_audio.m_completion);
            mutex_lock(&ear_audio.fifo_lock);
            status = ear_audio.m->status;
        } else {
            status = spidev_sync(spidev, m);
            if (0 < status) {
                spidev_ear_fifo_ntoh(&read_fifo,
                             (char *)(ear_audio.m) +
                                 EAR_AUDIO_FIFO_OFFSET + 4);
            }
    		if (spidev_check_read_ear_fifo(spidev->spi, isOpen, false, &(ear_audio.fifo), &read_fifo)) {
                spidev_ear_fifo_update(false, &(ear_audio.fifo), &read_fifo);
            } else {
                status = -EAGAIN;
            }
        }
        if (status < 0) {
            SPIDEV_ERR(spidev->spi, "Retry read ear audio info.index: %d Status: %zd",
                        i, status);
        }
    }
    return status;
}



static void spidev_async_ear_fifo_complete(void *context)
{
	struct spi_message *m = (struct spi_message *)context;
	fifo_t read_fifo;
	mutex_lock(&ear_audio.fifo_lock);
	ear_audio.async = false;
	if (m->status < 0) {
		SPIDEV_ERR(m->spi,
			"Failed to read ear fifo info. ErrCode: %d", m->status);
	} else {
    	spidev_ear_fifo_ntoh(&read_fifo,
    			     (char *)(ear_audio.m) + EAR_AUDIO_FIFO_OFFSET + 4);
    	if (spidev_check_read_ear_fifo(m->spi, false, true, &ear_audio.fifo, &read_fifo)) {
            spidev_ear_fifo_update(true, &ear_audio.fifo, &read_fifo);
    	} else {
            m->status = -EAGAIN;
    	}
	}
	mutex_unlock(&ear_audio.fifo_lock);
	complete(&ear_audio.m_completion);
	return;
}
static ssize_t spidev_async_read_ear_fifo(struct spidev_data *spidev)
{
    struct spi_message *m = ear_audio.m;
    if (!m) {
    m = spidev_read_ear_fifo_message(spidev);
    if (!m) {
        return -ENOMEM;
    }
    ear_audio.m = m;
    }
    if (!ear_audio.async) {
    m->complete = spidev_async_ear_fifo_complete;
    m->context = m;
    if (0 <= spidev_async(spidev, m)) {
        reinit_completion(&ear_audio.m_completion);
        ear_audio.async = true;
    }
    }
    return 0;
}

static int spidev_transfer_message(struct spidev_data *spidev, struct spi_message *m,
            struct spi_transfer *k_transfer,
            struct spi_rw_transfer *u_transfer,
            unsigned n_count, unsigned next)
{
    int index = 0;
    struct spi_transfer *k_tmp;
    u8 *tx_buf, *rx_buf;
    struct spi_rw_transfer *u_tmp = u_transfer + next;

    int total_tx = 0;

    tx_buf = spidev->tx_buffer;
    rx_buf = spidev->rx_buffer;

    for (index = next, k_tmp = k_transfer; index < n_count;
         u_tmp++, index++, k_tmp++) {
    total_tx += u_tmp->tx_len + u_tmp->ext_tx_len;

    if (total_tx > bufsiz) {
        break;
    }
    if (u_tmp->tx) {
        if (_copy_from_user(
            tx_buf,
            (const u8 __user *)(uintptr_t)u_tmp->tx,
            u_tmp->tx_len)) {
        return 0;
        }

            k_tmp->tx_buf = tx_buf;
            SPIDEV_HTONL(tx_buf, u_tmp->tx_len);
            tx_buf += u_tmp->tx_len;
        }

        if (u_tmp->ext_tx) {
            if (_copy_from_user(
                    tx_buf,
                    (const u8 __user *)(uintptr_t)u_tmp->ext_tx,
                    u_tmp->ext_tx_len)) {
                return 0;
            }
            SPIDEV_HTONL(tx_buf, u_tmp->ext_tx_len);
            tx_buf += u_tmp->ext_tx_len;
        }

    if (u_tmp->rx_ext_space) {
        if (!k_tmp->tx_buf) {
        k_tmp->tx_buf = tx_buf;
        }
        memset(tx_buf, 0, u_tmp->ext_tx_len);
        tx_buf[0] = 0x20;
        tx_buf[1] = 0x00;
            tx_buf[2] = (((u_tmp->ext_tx_len - 4) >> 2) & 0xFF00) >> 8;
        tx_buf[3] = (((u_tmp->ext_tx_len - 4) >> 2) & 0x00FF);
        tx_buf += u_tmp->ext_tx_len;
    }
    if (u_tmp->rx) {
        k_tmp->rx_buf = rx_buf;
        rx_buf += u_tmp->ext_tx_len + u_tmp->tx_len;
    }
    k_tmp->len = u_tmp->ext_tx_len + u_tmp->tx_len;
    k_tmp->speed_hz = spidev->speed_hz;
    k_tmp->cs_change = u_tmp->cs_change;
    spi_message_add_tail(k_tmp, m);
    }
    return index - next;
}

static ssize_t spidev_transfer_read(struct spidev_data *spidev,
             struct spi_rw_transfer *u_transfer,
             unsigned n_count)

{
    struct spi_message m;
    struct spi_transfer *k_transfer;
    struct spi_rw_transfer *u_rw_tmp;
    int status = -EFAULT;
    int m_count;
    int t_count = 0;
    int index;
    u8 *rx_buf;
    unsigned long missing;
    int rx_len;
    ssize_t total_read = 0;
    const int r_array = 3;

    k_transfer = kcalloc(r_array, sizeof(*k_transfer), GFP_KERNEL);
    if (!k_transfer) {
    return -ENOMEM;
    }
    do {
    spi_message_init(&m);
    memset(k_transfer, 0, r_array * sizeof(*k_transfer));
    m_count = spidev_transfer_message(spidev, &m, k_transfer,
              u_transfer, n_count, t_count);
    if (!m_count) {
        return -EMSGSIZE;
    }
    status = spidev_sync(spidev, &m);
    if (status < 0) {
        if (!total_read) {
        total_read = status;
        }
        break;
    }
    rx_buf = spidev->rx_buffer;
    for (index = 0, u_rw_tmp = (u_transfer + t_count);
         index < m_count; index++, u_rw_tmp++) {
        rx_len = u_rw_tmp->tx_len + u_rw_tmp->ext_tx_len;
        if (u_rw_tmp->rx) {
            SPIDEV_NTOHL(rx_buf + u_rw_tmp->rx_offset, rx_len - u_rw_tmp->rx_offset);
        missing = copy_to_user(
            (u8 __user *)(uintptr_t)(u_rw_tmp->rx),
            rx_buf + u_rw_tmp->rx_offset,
            rx_len - u_rw_tmp->rx_offset);
        if (missing == (rx_len - u_rw_tmp->rx_offset)) {
            break;
        }
        total_read +=
            rx_len - u_rw_tmp->rx_offset - missing;
        rx_buf += rx_len;
        }
    }
    t_count += m_count;
    } while (t_count < n_count);

    kfree(k_transfer);

    return total_read;
}

static ssize_t spidev_transfer_write(struct spidev_data *spidev,
              struct spi_rw_transfer *u_transfer,
              unsigned n_count)

{
    struct spi_message m;
    struct spi_transfer *k_transfer;
    int status = -EFAULT;
    int m_count;
    int t_count = 0;
    ssize_t total_write = 0;
    const int w_array = 1;
    
    k_transfer = kcalloc(w_array, sizeof(*k_transfer), GFP_KERNEL);
    if (!k_transfer) {
    return -ENOMEM;
    }
    do {
    spi_message_init(&m);
    memset(k_transfer, 0, w_array * sizeof(*k_transfer));
    m_count = spidev_transfer_message(spidev, &m, k_transfer,
              u_transfer, n_count, t_count);
    if (!m_count) {
        return -EMSGSIZE;
    }
    status = spidev_sync(spidev, &m);
    if (status < 0) {
        if (!total_write) {
        total_write = status;
        }
        break;
    }
    //跳过每个头部的8个字节
    total_write += status - (8 * m_count);
    t_count += m_count;
    } while (t_count < n_count);

    kfree(k_transfer);

    return total_write;
}

//static char rw_transfer_buf[SPIDEV_RW_TRANSFER_ARRAY_MAX * sizeof(struct spi_rw_transfer)];

static struct spi_rw_transfer *
spidev_get_rw_transfer(struct spidev_data *spidev, struct spi_rw_transfer __user *buf, size_t len,
           unsigned *n_count)
{
    /* copy into scratch area */
    struct spi_rw_transfer *rw_transfer =
    (struct spi_rw_transfer *)spidev->rw_transfer_buf;
    int copy_len = min_t(size_t, SPIDEV_RW_TRANSFER_BUF_LEN, len);

    *n_count = copy_len / sizeof(struct spi_rw_transfer);

    if (0 != *n_count) {
    if (copy_from_user(rw_transfer, buf, copy_len)) {
        return NULL;
    }
    }
    else
    {
        return NULL;
    }
    return rw_transfer;
}



/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t spidev_read(struct file *filp, char __user *buf, size_t count,
           loff_t *f_pos)
{
    struct spidev_data *spidev;
    ssize_t status;
    struct spi_rw_transfer *user_transfer;
    unsigned n_count = 0;
    int send_count = 0;
    int total_count = 0;
    ssize_t total_r = 0;
    /* chipselect only toggles at start or end of operation */
    /*
    if (count > SPIDEV_RW_TRANSFER_BUF_LEN)
    return -EMSGSIZE;
    */
    spidev = filp->private_data;
    total_count = count / sizeof(struct spi_rw_transfer);
    mutex_lock(&spidev->buf_lock);
    while (total_count != send_count) {
        user_transfer = spidev_get_rw_transfer(spidev, (struct spi_rw_transfer *)buf + send_count,
                       count, &n_count);
        if (user_transfer) {
            status = spidev_transfer_read(spidev, user_transfer, n_count);
            if (status < 0)
            {
                total_r = total_r? total_r : status;
                break;
            }
        } else {
            total_r = total_r? total_r : -EFAULT;
            break;
        }
        send_count += n_count;
        count -= n_count * sizeof(struct spi_rw_transfer);
        total_r += status;
    }
    mutex_unlock(&spidev->buf_lock);

    //kfree(user_transfer);

    return total_r;
}

/* Write-only message with current device setup */
static ssize_t spidev_write(struct file *filp, const char __user *buf,
            size_t count, loff_t *f_pos)
{
    struct spidev_data *spidev;
    ssize_t status = 0;
    struct spi_rw_transfer *tmp_transfer;
    int n_count = 0;
    int send_count = 0;
    int total_count = 0;
    ssize_t total_w = 0;
    /* chipselect only toggles at start or end of operation */
    /*
    if (count > SPIDEV_RW_TRANSFER_BUF_LEN)
    return -EMSGSIZE;
    */
    spidev = filp->private_data;
    total_count = count / sizeof(struct spi_rw_transfer);
    mutex_lock(&spidev->buf_lock);
    while (total_count != send_count)
    {
        tmp_transfer = spidev_get_rw_transfer(spidev, (struct spi_rw_transfer *)buf + send_count,
                      count, &n_count);
        if (tmp_transfer) {
        status = spidev_transfer_write(spidev, tmp_transfer, n_count);
            if (status < 0)
            {
                total_w = total_w? total_w : status;
                break;
            }
        } else {
        total_w = total_w? total_w : -EFAULT;
            break;
        }
        send_count += n_count;
        count -= n_count * sizeof(struct spi_rw_transfer);
        total_w += status;
    }
    mutex_unlock(&spidev->buf_lock);
    return total_w;
}

static void spidev_buf_release(struct spidev_data *spidev,
        struct spidev_buff *spidev_buf)
{
    kfree(spidev_buf->buff);
    return;
}

static const struct spidev_buf_operations spidev_buf_ops = {
    .release = spidev_buf_release,
};

static bool spidev_ring_empty(unsigned int head, unsigned int tail)
{
    return head == tail;
}

static bool spidev_ring_full(unsigned int head, unsigned int tail,
             unsigned int limit)
{
    return (head - tail) >= limit;
}

static void *spidev_malloc_from_ring(struct spidev_data *spidev, size_t size)
{
    unsigned int head;
    unsigned int mask;
    int offset;
    struct spidev_buff *spidev_buf;
    bool was_empty = false;
	size_t align_size = ALIGN(size, SPIDEV_MALLOC_ALIGN);
	void *ret;

    head = spidev->head;
    mask = spidev->ring_size - 1;
    was_empty = spidev_ring_empty(head, spidev->tail);

	if (SPIDEV_RING_SLOT_LEN < align_size) {
		return NULL;
	}
	if (!was_empty) {
		spidev_buf = &spidev->bufs[(head - 1) & mask];
		offset = spidev_buf->offset + spidev_buf->len;
		if ((offset + align_size) <= SPIDEV_RING_SLOT_LEN) {
			spidev_buf->len += align_size;
			ret = (void *)((char *)spidev_buf->buff + offset);
			if (((unsigned long)ret) & 0x07) {
                SPIDEV_ERR(spidev->spi, "The malloc address is not align for 8");
			}
			return ret;
		}
	}

	if (!spidev_ring_full(head, spidev->tail, spidev->ring_size)) {
		spidev->head = head + 1;
		spidev_buf = &spidev->bufs[head & mask];

		spidev_buf->offset = 0;
		spidev_buf->len = align_size;
		ret = (void *)spidev_buf->buff;
		if (((unsigned long)ret) & 0x07) {
            SPIDEV_ERR(spidev->spi, "The malloc address is not align for 8");
		}
		return ret;
	}
	return NULL;
}

static void spidev_free_to_ring(struct spidev_data *spidev, size_t size)
{
    unsigned int tail;
    unsigned int total_len;
    unsigned int mask;
    int len;
    struct spidev_buff *spidev_buf;

    tail = spidev->tail;
    total_len = ALIGN(size, SPIDEV_MALLOC_ALIGN);
    mask = spidev->ring_size - 1;
	while (total_len) {
		spidev_buf = &spidev->bufs[tail & mask];
		len = min(spidev_buf->len, total_len);
		total_len -= len;
		spidev_buf->len -= len;
		spidev_buf->offset += len;
		if (!spidev_buf->len) {
			spidev_buf->offset = 0;
			tail++;
		}
	}
	spidev->tail = tail;
	return;
}

static void spidev_audio_transfer_free(struct spidev_data *spidev, struct spi_transfer *t)
{

    if (t->tx_buf) {
        spidev_free_to_ring(spidev, t->len);
    }
    if (t->rx_buf) {
        spidev_free_to_ring(spidev, t->len);
    }
    spidev_free_to_ring(spidev, sizeof(struct spi_transfer));
    return;
}

static void spidev_audio_message_free(struct spidev_data *spidev, struct spi_message *msg)
{
    struct spi_transfer *t;
    struct spi_transfer *t_tmp;
    list_for_each_entry_safe(t, t_tmp, &(msg->transfers), transfer_list) {
        spidev_audio_transfer_free(spidev, t);
    }
    return;
}


static void spidev_async_message_complete(void *context)
{
	struct spi_message *m = (struct spi_message *)context;
	struct spidev_message_context *ct =
		container_of(m, struct spidev_message_context, m);
	struct spidev_data *spidev = ct->spidev;
	struct timespec64 ts_delta;
	bool empty;
	bool dofree;

	ktime_get_boottime_ts64(&ct->ts_end);
	ts_delta = timespec64_sub(ct->ts_end, ct->ts_begin);
	if (timespec64_to_ns(&ts_delta) > (5 * NSEC_PER_MSEC)) {
        /*
		dev_warn(&spidev->spi->dev,
			 "speed %lld (ns) time to transfer %u byte. speed: %u Hz",
			 timespec64_to_ns(&ts_delta), m->frame_length, spidev->speed_hz);
        */
        timeoutCount++;
		//printk_async_write_time(ct);
	}
	if (m->status < 0) {
		SPIDEV_ERR(spidev->spi,
			"Failed to transfer %u byte. ErrCode: %d",
			m->frame_length, m->status);
	}

	mutex_lock(&spidev->ring_lock);
	spidev_audio_message_free(spidev, m);
	spidev_free_to_ring(spidev, sizeof(*ct));
	empty = spidev_ring_empty(spidev->head, spidev->tail);
	mutex_unlock(&spidev->ring_lock);

	mutex_lock(&device_list_lock);
	spin_lock_irq(&spidev->spi_lock);
	dofree = spidev->spi == NULL;
	spin_unlock_irq(&spidev->spi_lock);
	spidev->users--;
	if (!spidev->users && empty) {
		spidev_free_txrx(spidev);
		spidev_free_ring(spidev);
        if (dofree) {
            kfree(spidev);
            spidev_free_ear_audio();
        }
    }
    mutex_unlock(&device_list_lock);
    return;
}

static struct spi_transfer *
spidev_audio_irq_transfer(struct spidev_data *spidev)
{
    struct spi_transfer *t;
    struct ear_irq_msg *irq_msg;
    u8 *buf_pos;
    int m_len = sizeof(struct spi_transfer) + 16;
    t = spidev_malloc_from_ring(spidev, m_len);

    if (!t) {
    return NULL;
    }
    memset(t, 0, m_len);
    buf_pos = (u8 *)(t + 1);

    spidev_initCspi(SPI_OPERATE_WRITE_EXT_MULTI_DATA, EAR_AUDIO_IRQ_ADDRESS,
        8, buf_pos);

    irq_msg = (struct ear_irq_msg *)(buf_pos + SPI_CSPI_VALUE_OFFSET);
    irq_msg->msg_type = MBOXMSG_CMD_HIFI_SPI;
    irq_msg->src_core = EAR_AUDIO_IRQ_SRC_CORE;
#ifdef XUEYING_CSPI_BIG_ENDIAN
    irq_msg->cmd = htonl(irq_msg->cmd);
#endif
    t->tx_buf = buf_pos;
    t->len = 16;
    t->speed_hz = spidev->speed_hz;
    t->cs_change = 0;
    return t;
}

static struct spi_transfer *
spidev_update_fifo_transfer(struct spidev_data *spidev, int wr_offset)
{
    struct spi_transfer *t;
    u8 *buf_pos;
    int m_len = sizeof(struct spi_transfer) + SPI_CSPI_FRAM_LEN;
    t = (struct spi_transfer *)spidev_malloc_from_ring(spidev, m_len);

    if (!t) {
    return NULL;
    }
    memset(t, 0, m_len);
    buf_pos = (u8 *)(t + 1);

    spidev_initCspi(SPI_OPERATE_WRITE_EXT_SING_REG, EAR_AUDIO_FIFO_ADDRESS,
        4, buf_pos);
#ifdef XUEYING_CSPI_BIG_ENDIAN
    *(unsigned int *)(buf_pos + SPI_CSPI_VALUE_OFFSET) = wr_offset;
#else
    *(unsigned int *)(buf_pos + SPI_CSPI_VALUE_OFFSET) = htonl(wr_offset);
#endif
    t->tx_buf = buf_pos;
    t->len = SPI_CSPI_FRAM_LEN;
    t->speed_hz = spidev->speed_hz;
    t->cs_change = 1;

    return t;
}

static ssize_t spidev_check_ear_fifo_free_space(struct spidev_data *spidev,
            int len)
{
    int fifo_free;
    ssize_t status;

    fifo_free = fifo_get_free_length(&(ear_audio.fifo));
    if (fifo_free < len) {
        status = spidev_sync_read_ear_fifo(spidev, false, &ear_audio.fifo);
        if (status < 0) {
			SPIDEV_ERR(spidev->spi,
				"Sync read ear fifo failed. ErrCode: %zd",
				status);
            return status;
        }
        fifo_free = fifo_get_free_length(&(ear_audio.fifo));
        if (fifo_free < len) {
			SPIDEV_ERR(spidev->spi,
				"Ear does not have enough space.free: %d, write: %d."
				"wr: %u rd: %u len: %u full: %d addr: 0x%x",
			       fifo_free, len, ear_audio.fifo.wr_offset,
			       ear_audio.fifo.rd_offset, ear_audio.fifo.len, ear_audio.fifo.full_flag,
			       ear_audio.fifo.data);
            return -ENOSPC;
        }
    }
    return 0;
}

static ssize_t spidev_async_write_audio(struct kiocb *iocb,
            struct iov_iter *iter)
{
    struct file *filp = iocb->ki_filp;
    struct spidev_data *spidev = (struct spidev_data *)filp->private_data;
    size_t len;
    size_t frame_len = 0;
    unsigned int head;
    unsigned int mask;
    struct spidev_buff *spi_buf;
    unsigned int remain;
    struct spi_message *m;
    struct spi_transfer *t;
    unsigned int addr;
    u8 *tx_buf;
    struct spidev_message_context *context;
    ssize_t status = 0;
    unsigned int pre_head = 0;
    int prev_len = 0;
    bool isEmpty = false;
    int ret;
    unsigned int max_addr;
    u8 audio_head_tmp[12];
    static struct timespec64 ts_begin;
    static struct timespec64 ts_end;
    static unsigned short seq_num = 0;

	if (unlikely(iter->nr_segs & 0x1)) {
		SPIDEV_ERR(spidev->spi, "Invalid parament. the vector(%lu) is not a multiple of 2.",
		            iter->nr_segs);
        return -EINVAL;
    }
	if (unlikely(!(filp->f_flags & O_NONBLOCK))) {
		SPIDEV_ERR(spidev->spi, "Please open with O_NONBLOCK flag.");
		return -EAGAIN;
	}

    (void)copy_from_user(audio_head_tmp, iter->iov->iov_base, sizeof(audio_head_tmp));
    if (first) {
        first = false;
        ktime_get_boottime_ts64(&ts_begin);
    } else {
        if ((*(unsigned short *)(audio_head_tmp + 4)) == seq_num) {
            SPIDEV_INFO(spidev->spi, "Retransmit message. seq num: %hu", seq_num);
        }
        else if ((*(unsigned short *)(audio_head_tmp + 4)) != (unsigned short)(seq_num + 1)) {
            SPIDEV_INFO(spidev->spi, "The seq num is no continuously. last seq num: %hu, current seq num: %hu",
                    seq_num, *(unsigned short *)(audio_head_tmp + 4));
        }
    }
    seq_num = *(unsigned short *)(audio_head_tmp + 4);
    receiveAudioCount++;
    ktime_get_boottime_ts64(&ts_end);
    if (!(receiveAudioCount % 1000)) {
        struct timespec64 ts_delta;
        ts_delta = timespec64_sub(ts_end, ts_begin);
        SPIDEV_DBG(spidev->spi, " transfer 1000 packets in %llu (ns)", timespec64_to_ns(&ts_delta));
        ts_begin = ts_end;
    }

    mutex_lock(&ear_audio.fifo_lock);
    status = spidev_check_ear_fifo_free_space(spidev, iov_iter_count(iter));
    if (status < 0) {
        mutex_unlock(&ear_audio.fifo_lock);
		SPIDEV_ERR(spidev->spi, "Discard the audio data,because fifo is not enough space. seq num: %hu", seq_num);
        return status;
    }
    max_addr = ear_audio.fifo.data + ear_audio.fifo.len;
    mutex_lock(&spidev->ring_lock);

    mask = spidev->ring_size - 1;
    while (iov_iter_count(iter)) {
        len = frame_len = iter->iov[0].iov_len + iter->iov[1].iov_len;
        if (unlikely(len & 0x3)) {
            SPIDEV_ERR(spidev->spi, "discard the audio data,because the length is invalid. seq num: %hu", seq_num);
            iov_iter_advance(iter, len);
            continue;
        }
        addr = ear_audio.fifo.data + ear_audio.fifo.wr_offset;
        ret = fifo_set_wr_offset(&(ear_audio.fifo), frame_len);
        if (unlikely(FIFO_OK != ret)) {
            SPIDEV_ERR(spidev->spi, "update ear fifo write offset failed. seq num: %hu", seq_num);
        }
        head = pre_head = spidev->head;
        isEmpty = spidev_ring_empty(head, spidev->tail);
        if (!isEmpty) {
            prev_len = spidev->bufs[(head - 1) & mask].len;
        }

        context = spidev_malloc_from_ring(
            spidev, sizeof(struct spidev_message_context));
        if (!context) {
            goto revert_ring;
        }
        m = &context->m;
        spi_message_init(m);

    while (len) {
        t = spidev_malloc_from_ring(
        spidev, sizeof(struct spi_transfer));
        if (!t) {
				SPIDEV_ERR(spidev->spi, "The ring is full");
        goto revert_ring;
        }
        memset(t, 0, sizeof(*t));

        head = spidev->head;
        spi_buf = &spidev->bufs[(head - 1) &
            mask]; //此时head一定不等于tail
        remain = SPIDEV_RING_SLOT_LEN -
         (spi_buf->offset + spi_buf->len);
        if (remain <= SPI_CSPI_VALUE_OFFSET) {
        if (!spidev_ring_full(head, spidev->tail,
                  spidev->ring_size)) {
            spidev->head = head + 1;
            spi_buf = &spidev->bufs[head & mask];
            spi_buf->offset = 0;
            spi_buf->len = 0;
            remain = SPIDEV_RING_SLOT_LEN;
        } else {
					SPIDEV_ERR(spidev->spi,
						"The ring is full");
            goto revert_ring;
        }
        }
        if (addr == max_addr) {
        addr = ear_audio.fifo.data;
        }
        remain = min3((size_t)(remain - SPI_CSPI_VALUE_OFFSET),
              len, (size_t)(max_addr - addr));
            if (unlikely(remain & 0x3)) {
                SPIDEV_ERR(spidev->spi, "The seq num of %hu len is not align for 4, remain:%u len:%lu max_addr:0x%x addr:0x%x",
                        seq_num, remain, len, max_addr, addr);
            }
        tx_buf = spidev_malloc_from_ring(
        spidev, remain + SPI_CSPI_VALUE_OFFSET);

        spidev_initCspi(SPI_OPERATE_WRITE_EXT_MULTI_DATA, addr,
            remain, tx_buf);

        (void)copy_from_iter(tx_buf + SPI_CSPI_VALUE_OFFSET,
                 remain, iter);
            SPIDEV_HTONL(tx_buf + SPI_CSPI_VALUE_OFFSET, remain);
        t->tx_buf = tx_buf;
        t->speed_hz = spidev->speed_hz;
        t->cs_change = 1;
        t->len = remain + SPI_CSPI_VALUE_OFFSET;

        addr += remain;
        len -= remain;
        spi_message_add_tail(t, m);
    }

    t = spidev_update_fifo_transfer(spidev,
            ear_audio.fifo.wr_offset);
    if (!t) {
		    SPIDEV_ERR(spidev->spi,
						"Update fifo wr_offset failed");
        goto revert_ring;
    }
    spi_message_add_tail(t, m);

    t = spidev_audio_irq_transfer(spidev);
    if (!t) {
		    SPIDEV_ERR(spidev->spi,
						"Send irq to audio failed");
        goto revert_ring;
    }
    spi_message_add_tail(t, m);

    context->spidev = spidev;

    m->complete = spidev_async_message_complete;
    m->context = m;
    ktime_get_boottime_ts64(&context->ts_begin);
    ret = spidev_async(spidev, m);
    if (ret < 0) {
        status = status ? status : ret;
			SPIDEV_ERR(spidev->spi,
						"Add the message to async queue failed. ErrCode: %d", ret);
        goto revert_ring;
    }
    status += frame_len;
    mutex_lock(&device_list_lock);
    spidev->users++;
    mutex_unlock(&device_list_lock);
    }
    mutex_unlock(&spidev->ring_lock);

    if (fifo_get_free_length(&ear_audio.fifo) <=
        EAR_AUDIO_FIFO_ASYNC_THRESHOLD) {
        spidev_async_read_ear_fifo(spidev);
    }

    mutex_unlock(&ear_audio.fifo_lock);

    return status;

revert_ring:

    SPIDEV_ERR(spidev->spi, "Discard the audio data,because ring is not enough space. seq num: %hu", *(unsigned short *)(audio_head_tmp + 4));
	SPIDEV_DBG(spidev->spi,
		 "Revert ring: head: %u, len: %u frame_len: %lu", pre_head,
		 prev_len, frame_len);
	spidev->head = pre_head;
	if (!isEmpty) {
		spidev->bufs[(pre_head - 1) & mask].len = prev_len;
	} else {
		spidev->bufs[pre_head & mask].len = prev_len;
	}
	mutex_unlock(&spidev->ring_lock);
	fifo_set_wr_offset(&(ear_audio.fifo), -frame_len);
	mutex_unlock(&ear_audio.fifo_lock);

    status = status ? status : -ENOSPC;
    return status;
}

static int spidev_message(struct spidev_data *spidev,
    struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
    struct spi_message    msg;
    struct spi_transfer    *k_xfers;
    struct spi_transfer    *k_tmp;
    struct spi_ioc_transfer *u_tmp;
    unsigned    n, total, tx_total, rx_total;
    u8        *tx_buf, *rx_buf;
    int        status = -EFAULT;

    spi_message_init(&msg);
    k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
    if (k_xfers == NULL)
    return -ENOMEM;

    /* Construct spi_message, copying any tx data to bounce buffer.
     * We walk the array of user-provided transfers, using each one
     * to initialize a kernel version of the same transfer.
     */
    tx_buf = spidev->tx_buffer;
    rx_buf = spidev->rx_buffer;
    total = 0;
    tx_total = 0;
    rx_total = 0;
    for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
        n;
        n--, k_tmp++, u_tmp++) {
    /* Ensure that also following allocations from rx_buf/tx_buf will meet
     * DMA alignment requirements.
     */
    unsigned int len_aligned = ALIGN(u_tmp->len, ARCH_KMALLOC_MINALIGN);

    k_tmp->len = u_tmp->len;

    total += k_tmp->len;
    /* Since the function returns the total length of transfers
     * on success, restrict the total to positive int values to
     * avoid the return value looking like an error.  Also check
     * each transfer length to avoid arithmetic overflow.
     */
    if (total > INT_MAX || k_tmp->len > INT_MAX) {
        status = -EMSGSIZE;
        goto done;
    }

    if (u_tmp->rx_buf) {
        /* this transfer needs space in RX bounce buffer */
        rx_total += len_aligned;
        if (rx_total > bufsiz) {
        status = -EMSGSIZE;
        goto done;
        }
        k_tmp->rx_buf = rx_buf;
        rx_buf += len_aligned;
    }
    if (u_tmp->tx_buf) {
        /* this transfer needs space in TX bounce buffer */
        tx_total += len_aligned;
        if (tx_total > bufsiz) {
        status = -EMSGSIZE;
        goto done;
        }
        k_tmp->tx_buf = tx_buf;
        if (copy_from_user(tx_buf, (const u8 __user *)
            (uintptr_t) u_tmp->tx_buf,
            u_tmp->len))
        goto done;
        tx_buf += len_aligned;
    }

    k_tmp->cs_change = !!u_tmp->cs_change;
    k_tmp->tx_nbits = u_tmp->tx_nbits;
    k_tmp->rx_nbits = u_tmp->rx_nbits;
    k_tmp->bits_per_word = u_tmp->bits_per_word;
    k_tmp->delay.value = u_tmp->delay_usecs;
    k_tmp->delay.unit = SPI_DELAY_UNIT_USECS;
    k_tmp->speed_hz = u_tmp->speed_hz;
    k_tmp->word_delay.value = u_tmp->word_delay_usecs;
    k_tmp->word_delay.unit = SPI_DELAY_UNIT_USECS;
    if (!k_tmp->speed_hz)
        k_tmp->speed_hz = spidev->speed_hz;
#ifdef VERBOSE
    dev_dbg(&spidev->spi->dev,
        "  xfer len %u %s%s%s%dbits %u usec %u usec %uHz\n",
        k_tmp->len,
        k_tmp->rx_buf ? "rx " : "",
        k_tmp->tx_buf ? "tx " : "",
        k_tmp->cs_change ? "cs " : "",
        k_tmp->bits_per_word ? : spidev->spi->bits_per_word,
        k_tmp->delay.value,
        k_tmp->word_delay.value,
        k_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
    spi_message_add_tail(k_tmp, &msg);
    }

    status = spidev_sync(spidev, &msg);
    if (status < 0)
    goto done;

    /* copy any rx data out of bounce buffer */
    for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
        n;
        n--, k_tmp++, u_tmp++) {
    if (u_tmp->rx_buf) {
        if (copy_to_user((u8 __user *)
            (uintptr_t) u_tmp->rx_buf, k_tmp->rx_buf,
            u_tmp->len)) {
        status = -EFAULT;
        goto done;
        }
    }
    }
    status = total;

done:
    kfree(k_xfers);
    return status;
}

static struct spi_ioc_transfer *
spidev_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
    unsigned *n_ioc)
{
    u32    tmp;

    /* Check type, command number and direction */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC
        || _IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
        || _IOC_DIR(cmd) != _IOC_WRITE)
    return ERR_PTR(-ENOTTY);

    tmp = _IOC_SIZE(cmd);
    if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
    return ERR_PTR(-EINVAL);
    *n_ioc = tmp / sizeof(struct spi_ioc_transfer);
    if (*n_ioc == 0)
    return NULL;

    /* copy into scratch area */
    return memdup_user(u_ioc, tmp);
}

static int spidev_vote_on(struct spi_device *spi)
{
	int status;
	status = pm_runtime_get_sync(spi->controller->dev.parent);
	SPIDEV_DBG(spi, "vote spi master(%s) on", dev_name(spi->controller->dev.parent));
	return status;
}
static int spidev_vote_off(struct spi_device *spi)
{
	int status;
	status = pm_runtime_put(spi->controller->dev.parent);
	SPIDEV_DBG(spi, "vote spi master(%s) on", dev_name(spi->controller->dev.parent));
	return status;
}
static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int        retval = 0;
    struct spidev_data    *spidev;
    struct spi_device    *spi;
    u32        tmp;
    unsigned    n_ioc;
    struct spi_ioc_transfer    *ioc;

    /* Check type and command number */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
    return -ENOTTY;

    /* guard against device removal before, or while,
     * we issue this ioctl.
     */
    spidev = filp->private_data;
    spin_lock_irq(&spidev->spi_lock);
    spi = spi_dev_get(spidev->spi);
    spin_unlock_irq(&spidev->spi_lock);

    if (spi == NULL)
    return -ESHUTDOWN;

    /* use the buffer lock here for triple duty:
     *  - prevent I/O (from us) so calling spi_setup() is safe;
     *  - prevent concurrent SPI_IOC_WR_* from morphing
     *    data fields while SPI_IOC_RD_* reads them;
     *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
     */
    mutex_lock(&spidev->buf_lock);

    switch (cmd) {
    /* read requests */
    case SPI_IOC_RD_MODE:
    retval = put_user(spi->mode & SPI_MODE_MASK,
            (__u8 __user *)arg);
    break;
    case SPI_IOC_RD_MODE32:
    retval = put_user(spi->mode & SPI_MODE_MASK,
            (__u32 __user *)arg);
    break;
    case SPI_IOC_RD_LSB_FIRST:
    retval = put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
            (__u8 __user *)arg);
    break;
    case SPI_IOC_RD_BITS_PER_WORD:
    retval = put_user(spi->bits_per_word, (__u8 __user *)arg);
    break;
    case SPI_IOC_RD_MAX_SPEED_HZ:
    retval = put_user(spidev->speed_hz, (__u32 __user *)arg);
    break;

    /* write requests */
    case SPI_IOC_WR_MODE:
    case SPI_IOC_WR_MODE32:
    if (cmd == SPI_IOC_WR_MODE)
        retval = get_user(tmp, (u8 __user *)arg);
    else
        retval = get_user(tmp, (u32 __user *)arg);
    if (retval == 0) {
        struct spi_controller *ctlr = spi->controller;
        u32    save = spi->mode;

        if (tmp & ~SPI_MODE_MASK) {
        retval = -EINVAL;
        break;
        }

        if (ctlr->use_gpio_descriptors && ctlr->cs_gpiods &&
            ctlr->cs_gpiods[spi->chip_select])
        tmp |= SPI_CS_HIGH;

        tmp |= spi->mode & ~SPI_MODE_MASK;
        spi->mode = tmp & SPI_MODE_USER_MASK;
        retval = spi_setup(spi);
        if (retval < 0)
        spi->mode = save;
        else
        dev_dbg(&spi->dev, "spi mode %x\n", tmp);
    }
    break;
    case SPI_IOC_WR_LSB_FIRST:
    retval = get_user(tmp, (__u8 __user *)arg);
    if (retval == 0) {
        u32    save = spi->mode;

        if (tmp)
        spi->mode |= SPI_LSB_FIRST;
        else
        spi->mode &= ~SPI_LSB_FIRST;
        retval = spi_setup(spi);
        if (retval < 0)
        spi->mode = save;
        else
        dev_dbg(&spi->dev, "%csb first\n",
            tmp ? 'l' : 'm');
    }
    break;
    case SPI_IOC_WR_BITS_PER_WORD:
    retval = get_user(tmp, (__u8 __user *)arg);
    if (retval == 0) {
        u8    save = spi->bits_per_word;

        spi->bits_per_word = tmp;
        retval = spi_setup(spi);
        if (retval < 0)
        spi->bits_per_word = save;
        else
        dev_dbg(&spi->dev, "%d bits per word\n", tmp);
    }
    break;
    case SPI_IOC_WR_MAX_SPEED_HZ: {
    u32 save;

    retval = get_user(tmp, (__u32 __user *)arg);
    if (retval)
        break;
    if (tmp == 0) {
        retval = -EINVAL;
        break;
    }

    save = spi->max_speed_hz;

    spi->max_speed_hz = tmp;
    retval = spi_setup(spi);
    if (retval == 0) {
        spidev->speed_hz = tmp;
        dev_dbg(&spi->dev, "%d Hz (max)\n", spidev->speed_hz);
    }

    spi->max_speed_hz = save;

    break;
    }
	case SPIDEV_IOCTL_PMGET:
	{
		retval = spidev_vote_on(spi);
		break;
	}
	case SPIDEV_IOCTL_PMPUT:
	{
		retval = spidev_vote_off(spi);
		break;
	}
    default:
    /* segmented and/or full-duplex I/O request */
    /* Check message and copy into scratch area */
    ioc = spidev_get_ioc_message(cmd,
        (struct spi_ioc_transfer __user *)arg, &n_ioc);
    if (IS_ERR(ioc)) {
        retval = PTR_ERR(ioc);
        break;
    }
    if (!ioc)
        break;    /* n_ioc is also 0 */

    /* translate to spi_message, execute */
    retval = spidev_message(spidev, ioc, n_ioc);
    kfree(ioc);
    break;
    }

    mutex_unlock(&spidev->buf_lock);
    spi_dev_put(spi);
    return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioc_message(struct file *filp, unsigned int cmd,
    unsigned long arg)
{
    struct spi_ioc_transfer __user    *u_ioc;
    int        retval = 0;
    struct spidev_data    *spidev;
    struct spi_device    *spi;
    unsigned        n_ioc, n;
    struct spi_ioc_transfer    *ioc;

    u_ioc = (struct spi_ioc_transfer __user *) compat_ptr(arg);

    /* guard against device removal before, or while,
     * we issue this ioctl.
     */
    spidev = filp->private_data;
    spin_lock_irq(&spidev->spi_lock);
    spi = spi_dev_get(spidev->spi);
    spin_unlock_irq(&spidev->spi_lock);

    if (spi == NULL)
    return -ESHUTDOWN;

    /* SPI_IOC_MESSAGE needs the buffer locked "normally" */
    mutex_lock(&spidev->buf_lock);

    /* Check message and copy into scratch area */
    ioc = spidev_get_ioc_message(cmd, u_ioc, &n_ioc);
    if (IS_ERR(ioc)) {
    retval = PTR_ERR(ioc);
    goto done;
    }
    if (!ioc)
    goto done;    /* n_ioc is also 0 */

    /* Convert buffer pointers */
    for (n = 0; n < n_ioc; n++) {
    ioc[n].rx_buf = (uintptr_t) compat_ptr(ioc[n].rx_buf);
    ioc[n].tx_buf = (uintptr_t) compat_ptr(ioc[n].tx_buf);
    }

    /* translate to spi_message, execute */
    retval = spidev_message(spidev, ioc, n_ioc);
    kfree(ioc);

done:
    mutex_unlock(&spidev->buf_lock);
    spi_dev_put(spi);
    return retval;
}

static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    if (_IOC_TYPE(cmd) == SPI_IOC_MAGIC
        && _IOC_NR(cmd) == _IOC_NR(SPI_IOC_MESSAGE(0))
        && _IOC_DIR(cmd) == _IOC_WRITE)
    return spidev_compat_ioc_message(filp, cmd, arg);

    return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static void spidev_free_ring(struct spidev_data    *spidev)
{
    
    int index;
    struct spidev_buff *spidev_buf;

    if(!spidev->bufs)
    {
        return;
    }
    for (index = 0; index < spidev->ring_size; index++)
    {
        spidev_buf = &spidev->bufs[index];
        if (spidev_buf->buff)
        {
            spidev_buf->ops->release(spidev, spidev_buf);
        }
    }
    kfree(spidev->bufs);
    spidev->bufs = NULL;
    spidev->head = spidev->tail = 0;
    spidev->ring_size = 0;
    return;
}

static void *spidev_alloc_ring(struct spidev_data    *spidev)
{
    int ring_size = SPIDEV_RING_SLOT_NUM;
    int index;
    u8 *buf;
    spidev->bufs = kcalloc(ring_size, sizeof(struct spidev_buff), GFP_KERNEL_ACCOUNT);
    if (!spidev->bufs)
    {
       return NULL;
    }
    for (index = 0; index < ring_size; index++)
    {
        buf = kmalloc(SPIDEV_RING_SLOT_LEN, GFP_KERNEL);
        if (unlikely(!buf))
        {
            
            goto free_bufs;
        }
        spidev->bufs[index].buff = buf;
        spidev->bufs[index].offset = 0;
        spidev->bufs[index].len    = 0;
        spidev->bufs[index].ops    = &spidev_buf_ops;
    }
    spidev->ring_size = ring_size;
    spidev->head = spidev->tail = 0;
    return spidev->bufs;
    
free_bufs:
    spidev_free_ring(spidev);
    return NULL;
}

static void spidev_free_txrx(struct spidev_data *spidev)
{
    if (spidev->tx_buffer) {
        kfree(spidev->tx_buffer);
        spidev->tx_buffer = NULL;
    }
    
    if (spidev->rx_buffer) {
        kfree(spidev->rx_buffer);
        spidev->rx_buffer = NULL;
    }
    
    if (spidev->rw_transfer_buf) {
        vfree(spidev->rw_transfer_buf);
        spidev->rw_transfer_buf = NULL;
    }
    return;
}

static  void *spidev_alloc_txrx(struct spidev_data    *spidev)
{

    spidev->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
    if (!spidev->tx_buffer) {
        goto err_alloc_tx_buf;
    }
    
    spidev->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
    if (!spidev->rx_buffer) {
        goto err_alloc_rx_buf;
    }
    spidev->rw_transfer_buf = vmalloc(SPIDEV_RW_TRANSFER_BUF_LEN);
    if (!spidev->rw_transfer_buf) {
        goto err_alloc_transfer_buf;
    }
    return spidev;
    
err_alloc_transfer_buf:
    kfree(spidev->rx_buffer);

err_alloc_rx_buf:
    kfree(spidev->tx_buffer);
    
err_alloc_tx_buf:
    return NULL;
}


static int spidev_open(struct inode *inode, struct file *filp)
{
    struct spidev_data    *spidev = NULL, *iter;
    int        status = -ENXIO;
    int         index;
    
    mutex_lock(&device_list_lock);

    list_for_each_entry (iter, &device_list, device_entry) {
    for (index = 0; index < SPIDEV_MAX_DEVICE_NODE_COUNT; index++) {
        if (iter->devt[index] == inode->i_rdev) {
        status = 0;
        spidev = iter;
        break;
        }
    }
    }
    if (!spidev) {
    pr_debug("spidev: nothing for minor %d\n", iminor(inode));

		goto err_find;
    }

    if (filp->f_flags & O_NONBLOCK) {
        if (!spidev->bufs && !spidev_alloc_ring(spidev)) {
			SPIDEV_ERR(spidev->spi, "Falied malloc space for ring");
        status = -ENOMEM;
        	goto err_alloc;
    }
    mutex_lock(&ear_audio.fifo_lock);
		memset(&ear_audio.fifo, 0, sizeof(ear_audio.fifo));
        status = spidev_sync_read_ear_fifo(spidev, true, &(ear_audio.fifo));
    if (status < 0) {
        mutex_unlock(&ear_audio.fifo_lock);
			SPIDEV_ERR(spidev->spi, "Error sync read ear fifo. ErrCode:%d", status);
        	goto err_sync_fifo;
    }
		SPIDEV_DBG(spidev->spi, "Read ear fifo info: wr: %u rd: %u full: %d addr: 0x%x",
		            ear_audio.fifo.wr_offset, ear_audio.fifo.rd_offset, ear_audio.fifo.full_flag,
		            ear_audio.fifo.data);
    mutex_unlock(&ear_audio.fifo_lock);
        timeoutCount = 0;
        receiveAudioCount = 0;
        first = true;
    }else {
        if (!spidev->tx_buffer && !spidev_alloc_txrx(spidev)) {
            SPIDEV_ERR(spidev->spi, "Falied malloc space for txrx");
            status = -ENOMEM;
            goto err_alloc;
        }
    }
    spidev->users++;
    filp->private_data = spidev;
    stream_open(inode, filp);

    mutex_unlock(&device_list_lock);
    return 0;
err_sync_fifo:
    if (!spidev->users) {
        spidev_free_ring(spidev);
    }
err_alloc:
err_find:
    mutex_unlock(&device_list_lock);
    return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
    struct spidev_data    *spidev;
    int        dofree;

    mutex_lock(&device_list_lock);
    spidev = filp->private_data;
    filp->private_data = NULL;

    spin_lock_irq(&spidev->spi_lock);
    /* ... after we unbound from the underlying device? */
    dofree = (spidev->spi == NULL);
    spin_unlock_irq(&spidev->spi_lock);

    SPIDEV_DBG(spidev->spi,
			 "[zeku_ear_spi%u] %lu packets speed more time to transfer. speed: %u Hz",
			 iminor(inode), timeoutCount, spidev->speed_hz);
	/* last close? */
	spidev->users--;
	if (!spidev->users) {

    spidev_free_txrx(spidev);

        spidev_free_ring(spidev);
		
    if (dofree) {
        kfree(spidev);
            spidev_free_ear_audio();
        }
    }
#ifdef CONFIG_SPI_SLAVE
    if (!dofree)
    spi_slave_abort(spidev->spi);
#endif
    mutex_unlock(&device_list_lock);

    return 0;
}

static const struct file_operations spidev_fops = {
    .owner =    THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =    spidev_write,
    .read =    spidev_read,
    .unlocked_ioctl = spidev_ioctl,
    .compat_ioctl = spidev_compat_ioctl,
    .open =    spidev_open,
    .release =    spidev_release,
    .llseek =    no_llseek,
    .write_iter = spidev_async_write_audio,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

static const struct spi_device_id spidev_spi_ids[] = {
    { .name = "zeku_ear_spi" },
    {},
};
MODULE_DEVICE_TABLE(spi, spidev_spi_ids);

/*
 * spidev should never be referenced in DT without a specific compatible string,
 * it is a Linux implementation thing rather than a description of the hardware.
 */

static const struct of_device_id spidev_dt_ids[] = {
    { .compatible = "zeku,zeku_ear_spi" },
    {},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);

/* Dummy SPI devices not to be used in production systems */
static int spidev_acpi_check(struct device *dev)
{
    dev_warn(dev, "do not use this driver in production systems!\n");
    return 0;
}

static const struct acpi_device_id spidev_acpi_ids[] = {
    /*
     * The ACPI SPT000* devices are only meant for development and
     * testing. Systems used in production should have a proper ACPI
     * description of the connected peripheral and they should also use
     * a proper driver instead of poking directly to the SPI bus.
     */
    { "SPT0001", (kernel_ulong_t)&spidev_acpi_check },
    { "SPT0002", (kernel_ulong_t)&spidev_acpi_check },
    { "SPT0003", (kernel_ulong_t)&spidev_acpi_check },
    {},
};
MODULE_DEVICE_TABLE(acpi, spidev_acpi_ids);

static int spidev_create_device_node(int pos, char *device_name,
             struct spidev_data *spidev)
{
    unsigned long minor;
    struct spi_device *spi;
    int status;

    spi = spidev->spi;

    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
    struct device *dev;

    spidev->devt[pos] = MKDEV(SPIDEV_MAJOR, minor);
    dev = device_create(spidev_class, &spi->dev, spidev->devt[pos],
            spidev, "%s", device_name);
    status = PTR_ERR_OR_ZERO(dev);
    } else {
    dev_dbg(&spi->dev, "no minor number available!\n");
    spidev->devt[pos] = 0;
    status = -ENODEV;
    }
    if (status == 0) {
    set_bit(minor, minors);
    }
    return status;
}

static void spidev_destroy_device_node(dev_t devt)
{
    if (unlikely(0 == devt)) {
    return;
    }
    device_destroy(spidev_class, devt);
    clear_bit(MINOR(devt), minors);
    return;
}

static void spidev_create_destroy_mutil_device_node(int count,
                struct spidev_data *spidev)
{
    int index;
    for (index = 0; index < count; index++) {
    if (0 != spidev->devt[index]) {
        spidev_destroy_device_node(spidev->devt[index]);
    }
    }
    return;
}

static int spidev_create_mutil_device_node(char **device_array, int count,
               struct spidev_data *spidev)
{
    int index;
    
    int status;

    for (index = 0; (index < count) && (NULL != device_array[index]); index++) {
    status = spidev_create_device_node(index, device_array[index],
               spidev);
    if (0 != status) {
        spidev_create_destroy_mutil_device_node(index, spidev);
        break;
    }
    }
    return status;
}

/*-------------------------------------------------------------------------*/

static int spidev_probe(struct spi_device *spi)
{
    int (*match)(struct device *dev);
    struct spidev_data    *spidev;
    int        status;

    match = device_get_match_data(&spi->dev);
    if (match) {
    status = match(&spi->dev);
    if (status)
        return status;
    }

    /* Allocate driver data */
    spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
    if (!spidev)
    return -ENOMEM;

    /* Initialize the driver data */
    spidev->spi = spi;
    spin_lock_init(&spidev->spi_lock);
    mutex_init(&spidev->buf_lock);

    INIT_LIST_HEAD(&spidev->device_entry);

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    status = spidev_create_mutil_device_node(
    spi_device_node_info, SPIDEV_MAX_DEVICE_NODE_COUNT, spidev);
    if (0 == status) {
    list_add(&spidev->device_entry, &device_list);
    }
    mutex_unlock(&device_list_lock);

    spidev->speed_hz = SPIDEV_DEFAULT_SPEED;
    if (status == 0)
    spi_set_drvdata(spi, spidev);
    else
    kfree(spidev);

    mutex_init(&spidev->ring_lock);
    mutex_init(&ear_audio.fifo_lock);
    init_completion(&ear_audio.m_completion);
    return status;
}

static int spidev_remove(struct spi_device *spi)
{
    struct spidev_data    *spidev = spi_get_drvdata(spi);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    /* make sure ops on existing fds can abort cleanly */
    spin_lock_irq(&spidev->spi_lock);
    spidev->spi = NULL;
    spin_unlock_irq(&spidev->spi_lock);

    list_del(&spidev->device_entry);
    spidev_create_destroy_mutil_device_node(SPIDEV_MAX_DEVICE_NODE_COUNT,
            spidev);
    if (spidev->users == 0) {
    kfree(spidev);
        spidev_free_ear_audio();
    }    
    mutex_unlock(&device_list_lock);
    return 0;
}

static struct spi_driver spidev_spi_driver = {
    .driver = {
    .name =    "earspi",
    .of_match_table = spidev_dt_ids,
    .acpi_match_table = spidev_acpi_ids,
    },
    .probe =    spidev_probe,
    .remove =    spidev_remove,
    .id_table =    spidev_spi_ids,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
    int status;

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
    if (status < 0) {
        pr_err("%s: failed to register_chrdev %d",
            __func__, status);
        return status;
    }

    SPIDEV_MAJOR = status;
    spidev_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(spidev_class)) {
        pr_err("%s: failed to class_create",
            __func__);
        unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
        return PTR_ERR(spidev_class);
    }

    status = spi_register_driver(&spidev_spi_driver);
    if (status < 0) {
        pr_err("%s: failed to spi_register_driver %d",
            __func__, status);
        class_destroy(spidev_class);
        unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
    }
    return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
    spi_unregister_driver(&spidev_spi_driver);
    class_destroy(spidev_class);
    unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
