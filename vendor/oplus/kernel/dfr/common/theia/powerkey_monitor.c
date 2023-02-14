// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include "powerkey_monitor.h"
#include "theia_kevent_kernel.h"

#define POWER_MONITOR_DEBUG_PRINTK(a, arg...)\
	do {\
		printk("[powerkey_monitor]: " a, ##arg);\
	} while (0)

static char *flow_buf = NULL;
static char *flow_buf_curr = NULL;
static int flow_index = 0;
static int stage_start = 0;
#define FLOW_SIZE 16
#define STAGE_BRIEF_SIZE 64
#define STAGE_TOTAL_SIZE ((STAGE_BRIEF_SIZE) * (FLOW_SIZE))

#define PROC_PWK_MONITOR_PARAM "pwkMonitorParam"
#define PROC_PWK_REPORT "theiaPwkReport"

static struct task_struct *block_thread = NULL;
static bool timer_started = false;
static int systemserver_pid = -1;
static bool g_system_boot_completed = false;

int get_systemserver_pid(void)
{
	return systemserver_pid;
}

static ssize_t powerkey_monitor_param_proc_read(struct file *file,
	char __user *buf, size_t count, loff_t *off)
{
	char page[512] = {0};
	int len = 0;

	len = sprintf(&page[len], "status=%d timeout_ms=%u is_panic=%d get_log=%d systemserver_pid=%d boot_completed=%d\n",
		g_black_data.status, g_black_data.timeout_ms, g_black_data.is_panic,
		g_black_data.get_log, systemserver_pid, g_system_boot_completed);

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buf, page, (len < count ? len : count)))
		return -EFAULT;

	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static bool handle_param_setup(char *key, char *value)
{
	bool ret = true;

	POWER_MONITOR_DEBUG_PRINTK("%s: setup param key:%s, value:%s\n", __func__, key, value);
	if (!strcmp(key, "state")) {
		int state;
		if (sscanf(value, "%d", &state) == 1)
			g_black_data.status = g_bright_data.status = state;
	} else if (!strcmp(key, "timeout")) {
		int timeout;
		if (sscanf(value, "%d", &timeout) == 1)
			g_black_data.timeout_ms = g_bright_data.timeout_ms = timeout;
	} else if (!strcmp(key, "log")) {
		int get_log;
		if (sscanf(value, "%d", &get_log) == 1)
			g_black_data.get_log = g_bright_data.get_log = get_log;
	} else if (!strcmp(key, "panic")) {
		int is_panic;
		if (sscanf(value, "%d", &is_panic) == 1)
			g_black_data.is_panic = g_bright_data.is_panic = is_panic;
	} else if (!strcmp(key, "systemserver_pid")) {
		int s_pid;
		if (sscanf(value, "%d", &s_pid) == 1)
			systemserver_pid = s_pid;
	} else if (!strcmp(key, "boot-completed")) {
		int is_boot_completed;
		if (sscanf(value, "%d", &is_boot_completed) == 1)
			g_system_boot_completed = !!is_boot_completed;
	} else {
		ret = false;
	}

	return ret;
}

bool is_system_boot_completed(void)
{
	return g_system_boot_completed;
}

/*
param format:
state 4;timeout 20000;panic 0;log 1
systemserver_pid 32639
*/
static ssize_t powerkey_monitor_param_proc_write(struct file *file,
	const char __user *buf, size_t count, loff_t *off)
{
	char buffer[256] = {0};
	char *pBuffer = NULL;
	char *param;
	int ret = 0;

	if (count > 255) {
		count = 255;
    }

	if (copy_from_user(buffer, buf, count)) {
		POWER_MONITOR_DEBUG_PRINTK("%s: read proc input error.\n", __func__);
		return count;
	}
	buffer[count] = '\0';
	pBuffer = buffer;

	POWER_MONITOR_DEBUG_PRINTK("%s: buffer:%s\n", __func__, buffer);

	while ((param = strsep(&pBuffer, ";"))) {
		char key[64] = {0}, value[64] = {0};
		ret = sscanf(param, "%s %s", key, value);
		POWER_MONITOR_DEBUG_PRINTK("%s: param:%s ret:%d key:%s value:%s\n", __func__, param, ret, key, value);
		if (ret == 2) {
			if (!handle_param_setup(key, value))
				POWER_MONITOR_DEBUG_PRINTK("%s: setup param fail! key:%s, value:%s\n", __func__, key, value);
		}
	}

	return count;
}

static int powerkey_monitor_param_proc_show(struct seq_file *seq_file, void *data)
{
	seq_printf(seq_file, "%s called\n", __func__);
	return 0;
}

static int powerkey_monitor_param_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, powerkey_monitor_param_proc_show, NULL);
}

static const struct proc_ops powerkey_monitor_param_proc_fops = {
	.proc_open = powerkey_monitor_param_proc_open,
	.proc_read = powerkey_monitor_param_proc_read,
	.proc_write = powerkey_monitor_param_proc_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

ssize_t get_last_pwkey_stage(char *buf)
{
	if (stage_start != flow_index) {
		int last_index = (flow_index == 0) ? (FLOW_SIZE - 1) : (flow_index - 1);
		snprintf(buf, 64, (last_index * STAGE_BRIEF_SIZE + flow_buf));
	} else {
		sprintf(buf, "");
	}

	return strlen(buf);
}

ssize_t get_pwkey_stages(char *buf)
{
	char *buf_curr = NULL;
	int start_index = stage_start;
	int end_index = flow_index;

	if (start_index == end_index)
		return 0;

	buf_curr = start_index * STAGE_BRIEF_SIZE + flow_buf;
	POWER_MONITOR_DEBUG_PRINTK("get_pwkey_stages start_index:%d end_index:%d\n", start_index, end_index);

	while (start_index!= end_index) {
		strcat(buf, buf_curr);
		strcat(buf, ",");
		POWER_MONITOR_DEBUG_PRINTK("get_pwkey_stages buf:%s\n", buf);

		buf_curr += STAGE_BRIEF_SIZE;

		/* w lock index */
		start_index++;
		if(start_index == FLOW_SIZE) {
			start_index = 0;
			buf_curr = flow_buf;
		}
	}

	return strlen(buf);
}

static ssize_t theia_powerkey_report_proc_read(struct file *file,
	char __user *buf, size_t count, loff_t *off)
{
	char stages[STAGE_TOTAL_SIZE] = {0};
	int stages_len;

	POWER_MONITOR_DEBUG_PRINTK("enter theia_powerkey_report_proc_read %d  %d", count, *off);

	stages_len = get_pwkey_stages(stages);

	return simple_read_from_buffer(buf, count, off, stages, stages_len);
}

void record_stage(const char *buf)
{
	if (!timer_started)
		return;

	POWER_MONITOR_DEBUG_PRINTK("%s: buf:%s\n", __func__, buf);

	memset(flow_buf_curr, 0, STAGE_BRIEF_SIZE);
	snprintf(flow_buf_curr, STAGE_BRIEF_SIZE, buf);
	flow_buf_curr += STAGE_BRIEF_SIZE;

	/* w lock index */
	flow_index++;
	if(flow_index == FLOW_SIZE) {
		flow_index = 0;
		flow_buf_curr = flow_buf;
	}
}

static ssize_t theia_powerkey_report_proc_write(struct file *file,
	const char __user *buf, size_t count, loff_t *off)
{
	char buffer[STAGE_BRIEF_SIZE] = {0};

	if (g_black_data.status == BLACK_STATUS_INIT || g_black_data.status == BLACK_STATUS_INIT_FAIL) {
		POWER_MONITOR_DEBUG_PRINTK("%s init not finish: status = %d\n", __func__, g_black_data.status);
		return count;
	}

	if (count >= STAGE_BRIEF_SIZE)
		count = STAGE_BRIEF_SIZE - 1;

	if (copy_from_user(buffer, buf, count)) {
		POWER_MONITOR_DEBUG_PRINTK("%s: read proc input error.\n", __func__);
		return count;
	}

	record_stage(buffer);
	return count;
}

static int theia_powerkey_report_proc_show(struct seq_file *seq_file, void *data)
{
	seq_printf(seq_file, "%s called\n", __func__);
	return 0;
}

static int theia_powerkey_report_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, theia_powerkey_report_proc_show, NULL);
}

static const struct proc_ops theia_powerkey_report_proc_fops = {
	.proc_open = theia_powerkey_report_proc_open,
	.proc_read = theia_powerkey_report_proc_read,
	.proc_write = theia_powerkey_report_proc_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static ssize_t theia_powerkey_test_node_proc_read(struct file *file,
	char __user *buf, size_t count, loff_t *off)
{
	return 0;
}

static ssize_t theia_powerkey_test_node_proc_write(struct file *file,
	const char __user *buf, size_t count, loff_t *off)
{
	char buffer[128] = {0};
	if (count > 127 || copy_from_user(buffer, buf, count)) {
		POWER_MONITOR_DEBUG_PRINTK("%s: read proc input error.\n", __func__);
		return count;
	}

    POWER_MONITOR_DEBUG_PRINTK("theia_powerkey_test_node_proc_write buffer:%s\n", buffer);
	if (!strncmp(buffer, "test_d_block\n", 13)) {
        block_thread = get_current();
         POWER_MONITOR_DEBUG_PRINTK("theia_powerkey_test_node_proc_write set TASK_UNINTERRUPTIBLE block_thread pid:%d\n", block_thread->pid);
        set_current_state(TASK_UNINTERRUPTIBLE);
         schedule();
         POWER_MONITOR_DEBUG_PRINTK("theia_powerkey_test_node_proc_write set TASK_UNINTERRUPTIBLE, after schedule\n");
         block_thread = NULL;
	} else if (!strncmp(buffer, "test_d_unblock\n", 15)) {
        if (block_thread != NULL) {
            POWER_MONITOR_DEBUG_PRINTK("theia_powerkey_test_node_proc_write call wake_up_process pid:%d\n", block_thread->pid);
            wake_up_process(block_thread);
        }
	} else if (!strncmp(buffer, "test_d_unblock_with_kill\n", 25)) {
/*
        if (block_thread != NULL) {
            POWER_MONITOR_DEBUG_PRINTK("theia_powerkey_test_node_proc_write call wake_up_process with kill pid:%d\n", block_thread->pid);
            block_thread->flags |= PF_KILLING;
            do_send_sig_info(SIGKILL, SEND_SIG_FORCED, block_thread, true);
            wake_up_process(block_thread);
        }
*/
	} else if (!strncmp(buffer, "test_blackscreen_dcs\n", 21)) {
        send_black_screen_dcs_msg();
	} else if (!strncmp(buffer, "test_kevent\n", 12)) {
        SendTheiaKevent(THEIA_ATTR_TYPE_COMMON_STRING, "logTagTest", "eventIDTest", "Powerkey test kevent!");
    }

	return count;
}

static int theia_powerkey_test_node_proc_show(struct seq_file *seq_file, void *data)
{
	seq_printf(seq_file, "%s called\n", __func__);
	return 0;
}

static int theia_powerkey_test_node_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, theia_powerkey_test_node_proc_show, NULL);
}

static const struct proc_ops theia_powerkey_test_node_proc_fops = {
	.proc_open = theia_powerkey_test_node_proc_open,
	.proc_read = theia_powerkey_test_node_proc_read,
	.proc_write = theia_powerkey_test_node_proc_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

void theia_pwk_stage_start(char *reason)
{
	POWER_MONITOR_DEBUG_PRINTK("theia_pwk_stage_start start %x:  %x   %x   flow_buf\n", flow_buf, flow_buf_curr, flow_index);
	stage_start = flow_index;
	timer_started = true;
	record_stage(reason);
}

void theia_pwk_stage_end(char *reason)
{
	if (timer_started) {
		POWER_MONITOR_DEBUG_PRINTK("theia_pwk_stage_end, reason:%s\n", reason);
		record_stage(reason);
		timer_started = false;
	}
}

static bool is_zygote_process(struct task_struct *t)
{
	const struct cred *tcred = __task_cred(t);
	if (!strcmp(t->comm, "main") && (tcred->uid.val == 0) &&
		(t->parent != 0 && !strcmp(t->parent->comm, "init")))
		return true;
	else
		return false;
}

static void show_coretask_state(void)
{
	struct task_struct *g, *p;

	rcu_read_lock();
	for_each_process_thread(g, p) {
		if (is_zygote_process(p) || !strncmp(p->comm, "system_server", TASK_COMM_LEN)
			|| !strncmp(p->comm, "surfaceflinger", TASK_COMM_LEN)) {
#if IS_MODULE(CONFIG_OPLUS_FEATURE_THEIA)
			touch_nmi_watchdog();
#endif
			sched_show_task(p);
		}
	}

	rcu_read_unlock();
}

void doPanic(void)
{
	/* 2. show all D task */
#if IS_MODULE(CONFIG_OPLUS_FEATURE_THEIA)
	handle_sysrq('w');
#else
	show_state_filter(TASK_UNINTERRUPTIBLE);
#endif
	/* 3. show system_server zoygot surfacefliger state */
	show_coretask_state();
	/* 4.current cpu registers :skip for minidump */
	panic("bright screen detected, force panic");
}

int __init powerkey_monitor_init(void)
{
	POWER_MONITOR_DEBUG_PRINTK("%s called\n", __func__);

	flow_buf = vmalloc(STAGE_TOTAL_SIZE);
	if (!flow_buf) {
		POWER_MONITOR_DEBUG_PRINTK("vmalloc flow_buf failed\n");
		return -ENOMEM;
	}
	memset(flow_buf, 0, STAGE_TOTAL_SIZE);
	flow_buf_curr = flow_buf;

	theia_kevent_module_init();
	black_screen_check_init();
	bright_screen_check_init();
	theia_send_event_init();

	/* a node for param setup */
	if (proc_create(PROC_PWK_MONITOR_PARAM, S_IRWXUGO, NULL, &powerkey_monitor_param_proc_fops) == NULL)
		POWER_MONITOR_DEBUG_PRINTK("pwkMonitorParam proc node create failed\n");

	/* a node for normal stage record */
	if (proc_create(PROC_PWK_REPORT, S_IRWXUGO, NULL, &theia_powerkey_report_proc_fops) == NULL)
		POWER_MONITOR_DEBUG_PRINTK("theiaPwkReport proc node create failed\n");

	/* a node fo test */
	/* proc_create("theiaPwkTestNode", S_IRWXUGO, NULL, &theia_powerkey_test_node_proc_fops); */

	return 0;
}

void __exit powerkey_monitor_exit(void)
{
	POWER_MONITOR_DEBUG_PRINTK("%s called\n", __func__);

	remove_proc_entry(PROC_PWK_MONITOR_PARAM, NULL);
	remove_proc_entry(PROC_PWK_REPORT, NULL);

	theia_send_event_exit();
	bright_screen_exit();
	black_screen_exit();
	theia_kevent_module_exit();

	if (!flow_buf) {
		vfree(flow_buf);
		flow_buf = NULL;
		flow_buf_curr = NULL;
	}
}
module_init(powerkey_monitor_init);
module_exit(powerkey_monitor_exit);

MODULE_DESCRIPTION("powerkey_monitor@2.0");
MODULE_VERSION("2.0");
MODULE_LICENSE("GPL v2");
