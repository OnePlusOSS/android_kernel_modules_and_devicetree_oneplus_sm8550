// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/types.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/limits.h>
#include <linux/printk.h>      /* for pr_err, pr_info etc */
#include <linux/mutex.h>
#include <linux/fcntl.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/atomic.h>
#if IS_ENABLED(CONFIG_OPLUS_SYSTEM_KERNEL_MTK) && (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
#include <../drivers/soc/oplus/cpu/sched/sched_assist/sa_common.h>
#else
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#endif
#define FDLEAK_CHECK_LOG_TAG "[fdleak_check]"
#define FD_MAX 32768
#define DEFAULT_THRESHOLD (FD_MAX/2)
#define DEFAULT_DUMP_THRESHOLD (DEFAULT_THRESHOLD + 500)
#define FDLEAK_ALREADY_TRIGGER_FLAG 0x55
#define FDLEAK_ALREADY_DUMP_FLAG 0x56
#define SIG_FDLEAK_CHECK_TRIGGER (SIGRTMIN + 10)
#define BIONIC_SIGNAL_FDTRACK (SIGRTMIN + 7)
#define TASK_COMM_LEN			16
#define THRESHOLD_LEN                   10
#define MAX_SYMBOL_LEN 64
#define TASK_WHITE_LIST_MAX  128

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
static char symbol[MAX_SYMBOL_LEN] = "__alloc_fd";
#else
static char symbol[MAX_SYMBOL_LEN] = "get_unused_fd_flags";
#endif

module_param_string(symbol, symbol, sizeof(symbol), 0644);
int load_threshold = DEFAULT_THRESHOLD;
int dump_threshold = DEFAULT_DUMP_THRESHOLD;

struct fdleak_white_list_struct {
	char comm[TASK_COMM_LEN];
	int load_threshold;
	int dump_threshold;
};

static struct fdleak_white_list_struct white_list[TASK_WHITE_LIST_MAX] = {
	{"fdleak_example", 2048, 2560},
	{"composer", 19500, 20000},
	{"surfaceflinger", 19500, 20000},
};

static ssize_t fdleak_proc_read(struct file *file, char __user *buf,
		size_t count, loff_t *off)
{
	char page[2048] = {0};
	int len = 0;
	int i;

	for(i = 0; i < ARRAY_SIZE(white_list); i++) {
                if (!strlen(white_list[i].comm))
                        break;
		len += snprintf(&page[len], 2048 - len, "fdleak_detect_task = %s load_threshold = %d dump_threshold = %d\n",
					white_list[i].comm, white_list[i].load_threshold, white_list[i].dump_threshold);
	}

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf, page, (len < count ? len : count))) {
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static ssize_t fdleak_proc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *off)
{
	int tmp_load_threshold = 0;
	int tmp_dump_threshold = 0;
	char tmp_task[TASK_COMM_LEN] = {0};
	int ret = 0;
	char buffer[64] = {0};
	int max_len[] = {TASK_COMM_LEN, THRESHOLD_LEN, THRESHOLD_LEN};
	int part;
	char delim[] = {' ', ' ', '\n'};
	char *start, *end;
	int i;

	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;

	if (copy_from_user(buffer, buf, count)) {
		pr_err(FDLEAK_CHECK_LOG_TAG "%s: read proc input error.\n", __func__);
		return count;
	}

	buffer[count] = '\0';

	/* validate the length of each of the 3 parts */
	start = buffer;
	for (part = 0; part < 3; part++) {
		end = strchr(start, delim[part]);
		if (end == NULL || (end - start) > max_len[part]) {
			return count;
		}
		start = end + 1;
	}

	ret = sscanf(buffer, "%s %d %d", tmp_task, &tmp_load_threshold, &tmp_dump_threshold);
	if(ret <= 0) {
		pr_err(FDLEAK_CHECK_LOG_TAG "%s: input error\n", __func__);
		return count;
	}
	for (i = 0; i < ARRAY_SIZE(white_list); i++) {
		if (strlen(white_list[i].comm) && !strcmp(white_list[i].comm, tmp_task)) {
			white_list[i].load_threshold = tmp_load_threshold;
			white_list[i].dump_threshold = tmp_dump_threshold;
			break;
		} else if (strlen(white_list[i].comm)) {
			continue;
		} else {
			strncpy(white_list[i].comm, tmp_task, strlen(tmp_task));
			white_list[i].load_threshold = tmp_load_threshold;
			white_list[i].dump_threshold = tmp_dump_threshold;
			break;
		}
	}
	return count;
}

static struct proc_ops fdleak_proc_pops = {
	.proc_read = fdleak_proc_read,
	.proc_write = fdleak_proc_write,
	.proc_lseek = default_llseek,
};

/* used for handle_fdleak_error serialize to avoid race condition */
static atomic_t error_is_handling;

static inline void handle_fdleak_error(struct task_struct *task)
{
	get_task_struct(task);
	send_sig(SIG_FDLEAK_CHECK_TRIGGER, task, 0);

	put_task_struct(task);
	atomic_set(&error_is_handling, 0);
}

int white_list_check(struct task_struct *p) {
        int i;

        for (i = 0; i < ARRAY_SIZE(white_list); i++) {
			if (!strlen(white_list[i].comm))
				break;

			if (strnstr(p->comm, white_list[i].comm, strlen(white_list[i].comm))) {
				load_threshold = white_list[i].load_threshold;
				dump_threshold = white_list[i].dump_threshold;
				return 0;
			}
        }

        load_threshold = DEFAULT_THRESHOLD;
        dump_threshold = DEFAULT_DUMP_THRESHOLD;
        return -1;
}

static void fdleak_check(int fd)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(current);

	if (IS_ERR_OR_NULL(ots))
		return;

        if (current->sighand == NULL) {
		return;
        }

        if (current->sighand->action[SIG_FDLEAK_CHECK_TRIGGER - 1].sa.sa_handler == SIG_DFL) {
		return;
        }

	/* already fdleak, return, not check */
	if (ots->fdleak_flag == FDLEAK_ALREADY_DUMP_FLAG || current->pid != current->tgid) {
		return;
	} else if (ots->fdleak_flag == FDLEAK_ALREADY_TRIGGER_FLAG && !white_list_check(current) && fd >= dump_threshold) {
		send_sig(BIONIC_SIGNAL_FDTRACK, current, 0);
		ots->fdleak_flag = FDLEAK_ALREADY_DUMP_FLAG;
	} else if (ots->fdleak_flag != FDLEAK_ALREADY_TRIGGER_FLAG && !white_list_check(current) && fd >= load_threshold) {
		if (atomic_cmpxchg(&error_is_handling, 0, 1) != 0)
			return;

		ots->fdleak_flag = FDLEAK_ALREADY_TRIGGER_FLAG;
	    pr_err(FDLEAK_CHECK_LOG_TAG "current : %s fd: %d \n", current->comm, fd);
		handle_fdleak_error(current);
	} else {
		return;
        }
}

static int ret_handler(struct kretprobe_instance *kri, struct pt_regs *regs)
{
	int fd;
	fd = regs_return_value(regs);
	if (fd < 0) {
		return -1;
	}

	fdleak_check(fd);
	return 0;
}

/* For each probe you need to allocate a kprobe structure */
static struct kretprobe g_krp = {
	.handler = ret_handler,
	.maxactive = 10,
};

static int __init fdleak_check_init(void)
{
	int ret;

	g_krp.kp.symbol_name = symbol;
	if(!proc_create("fdleak_detect", 0666, NULL, &fdleak_proc_pops)) {
		pr_err(FDLEAK_CHECK_LOG_TAG "proc node fdleak_detect create failed\n");
		return -ENOENT;
	}
	ret = register_kretprobe(&g_krp);
	if (ret < 0) {
		pr_err(FDLEAK_CHECK_LOG_TAG "oplus_fdleak_check, register_kretprobe failed, return %d\n", ret);
		remove_proc_entry("fdleak_detect", NULL);
		return ret;
	}
	pr_info(FDLEAK_CHECK_LOG_TAG "oplus_fdleak_check, planted kretprobe at %p\n", g_krp.kp.addr);


	return 0;
}

static void __exit fdleak_chekc_exit(void)
{
	unregister_kretprobe(&g_krp);
	remove_proc_entry("fdleak_detect", NULL);
	pr_info("oplus_fdleak_check, kretprobe at %p unregistered\n", g_krp.kp.addr);
}

MODULE_DESCRIPTION("oplus fdleak check");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wei.Li");

module_init(fdleak_check_init);
module_exit(fdleak_chekc_exit);

