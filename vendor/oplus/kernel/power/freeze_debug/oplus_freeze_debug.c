#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <trace/events/power.h>
#include <trace/hooks/power.h>

#define OPLUS_FREEZE_PROC_DIR  "freeze"

static bool freeze_todo_logging_on = false;
static struct proc_dir_entry *freeze_proc = NULL;


static void android_vh_try_to_freeze_todo_logging(void *unused, bool *logging_on)
{
	*logging_on = freeze_todo_logging_on;
}

static ssize_t freeze_logging_on_write(struct file *filp,
		const char __user *buff, size_t len, loff_t *data)
{
	char buf[10] = {0};
	unsigned int val = 0;

	if (len > sizeof(buf))
		return -EFAULT;

	if (copy_from_user((char *)buf, buff, len))
		return -EFAULT;

	if (kstrtouint(buf, sizeof(buf), &val))
		return -EINVAL;

	freeze_todo_logging_on = !!(val);

	return len;
}

static int freeze_logging_on_show(struct seq_file *seq_filp, void *v)
{
	seq_printf(seq_filp, "%d\n", freeze_todo_logging_on);
	return 0;
}

static int freeze_logging_on_open(struct inode *inode, struct file *file)
{
	int ret;

	ret = single_open(file, freeze_logging_on_show, NULL);

	return ret;
}

static const struct proc_ops freeze_logging_on_fops = {
	.proc_open		= freeze_logging_on_open,
	.proc_write		= freeze_logging_on_write,
	.proc_read		= seq_read,
};

static int __init freeze_logging_init(void)
{
	freeze_proc = proc_mkdir(OPLUS_FREEZE_PROC_DIR, NULL);
	proc_create("freezing_logging_on", 0666, freeze_proc, &freeze_logging_on_fops);

	register_trace_android_vh_try_to_freeze_todo_logging(android_vh_try_to_freeze_todo_logging, NULL);

	return 0;
}
module_init(freeze_logging_init);

static void __exit freeze_logging_exit(void)
{
	unregister_trace_android_vh_try_to_freeze_todo_logging(android_vh_try_to_freeze_todo_logging, NULL);

	remove_proc_entry("freezing_logging_on", freeze_proc);
	remove_proc_entry(OPLUS_FREEZE_PROC_DIR, NULL);
}
module_exit(freeze_logging_exit);

MODULE_AUTHOR("Colin.Liu");
MODULE_DESCRIPTION("Freeze debug logging on module");
MODULE_LICENSE("GPL v2");
