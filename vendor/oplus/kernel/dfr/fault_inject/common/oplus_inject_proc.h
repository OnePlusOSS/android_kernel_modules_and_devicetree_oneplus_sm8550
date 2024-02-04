#ifndef _OPLUS_FAULT_PROC_H_
#define _OPLUS_FAULT_PROC_H_

#include <linux/version.h>
#include <linux/seq_file.h>
#include <linux/file.h>
#include <linux/uaccess.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
#define FILE_NODE(file) file->f_dentry->d_inode
#else
#define FILE_NODE(file) file->f_path.dentry->d_inode
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
#define PDE_DATA(node)   (PDE(node)->data)
#endif

#define proc_ops_name(name) proc_ops_##name
#define proc_ops_open(name)  proc_open_##name
#define proc_ops_show(name)  proc_show_##name
#define proc_ops_write(name)  proc_write_##name
#define proc_ops_init(name) {\
        .open   = proc_ops_open(name),\
        .read           = seq_read,\
        .write  = proc_ops_write(name),\
        .llseek = seq_lseek,\
        .release        = single_release,\
}

#define proc_ops_define(name) \
        static struct file_operations proc_ops_name(name) =  proc_ops_init(name)

/* proc node tree struct
 * name record the node
 * list record the node sub info
 * type record the node is dir or file node
 * proc_node record the proc register ptr
 */
struct proc_tree_node {
	struct list_head entry;
	struct list_head sublist;
	struct proc_tree_node *parent;
	int type;
	char *name;
	void *proc_node;
};

/* forbit using IC name to create node name */
struct proc_tree_node* oplus_proc_inject_init(char *name, const struct proc_ops *ops, void *data);
void oplus_proc_inject_exit(struct proc_tree_node* node);
#endif
