#ifndef __IO_METRICS_ENTRY_H__
#define __IO_METRICS_ENTRY_H__
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/blkdev.h>
#include <linux/percpu.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/export.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/trace_clock.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/namei.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/tracepoint.h>
#include <linux/fs.h>
#include <linux/bio.h>
#include <linux/printk.h>

#define io_metrics_print(fmt, arg...) \
    printk("[IO_METRICS] [%-16s] %20s:%-4d "fmt, current->comm, __func__, __LINE__, ##arg)

/* 统计周期 */
enum sample_cycle_type {
//    CYCLE_SECOND_10 = 0,
//    CYCLE_MINUTES_1,
//    CYCLE_MINUTES_10,
//    CYCLE_HOUR_1,
//    CYCLE_DAY_1,
//    CYCLE_WEEK_1,
    /* 记录总的流量 */
    CYCLE_FOREVER = 0,
    CYCLE_MAX
};

#define LAT_0_TO_100U_MASK      100000
#define LAT_100U_TO_200U_MASK   200000
#define LAT_200U_TO_500U_MASK   500000
#define LAT_500U_TO_2M_MASK     2000000
#define LAT_2M_TO_20M_MASK      20000000
#define LAT_20M_TO_100M_MASK    100000000
#define LAT_100M_TO_500M_MASK   500000000

/* 延迟分布 */
enum lat_range {
    LAT_0_TO_100U = 0,   /* (0, 100us]      */
    LAT_100U_TO_200U,    /* (100us, 200us]  */
    LAT_200U_TO_500U,    /* (200us, 500us]  */
    LAT_500U_TO_2M,      /* (500us, 2ms)    */
    LAT_2M_TO_20M,       /* [2ms, 20ms)     */
    LAT_20M_TO_100M,     /* [20ms, 200ms)   */
    LAT_100M_TO_500M,    /* [100ms, 500ms)  */
    LAT_500M_TO_MAX,     /* [500ms, +∞)     */
};

#define lat_range_check(elapsed, lat_range) \
do { \
    if (likely(elapsed <= LAT_0_TO_100U_MASK)) {   \
        lat_range = LAT_0_TO_100U;         \
    } else if (elapsed <= LAT_100U_TO_200U_MASK) { \
        lat_range = LAT_100U_TO_200U;      \
    } else if (elapsed <= LAT_200U_TO_500U_MASK) { \
        lat_range = LAT_200U_TO_500U;      \
    } else if (elapsed <= LAT_500U_TO_2M_MASK) {   \
        lat_range = LAT_500U_TO_2M;        \
    } else if (elapsed <= LAT_2M_TO_20M_MASK) {    \
        lat_range = LAT_2M_TO_20M;         \
    } else if (elapsed <= LAT_20M_TO_100M_MASK) {  \
        lat_range = LAT_20M_TO_100M;       \
    } else if (elapsed <= LAT_100M_TO_500M_MASK) { \
        lat_range = LAT_100M_TO_500M;      \
    }  else {                                        \
        lat_range = LAT_500M_TO_MAX;       \
    }                                                \
} while (0)


struct sample_cycle {
    enum sample_cycle_type value;
    const char * tag;
    const u64 cycle_value;
};

extern bool io_metrics_enabled;
extern bool io_metrics_debug_enabled;

#endif /* __IO_METRICS_ENTRY_H__ */