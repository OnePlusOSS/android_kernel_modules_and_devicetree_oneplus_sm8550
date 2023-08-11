// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2024 Oplus. All rights reserved.
 */
/***************************************************************
** OPLUS_FEATURE_DEVICE_FAULT_INJECT
** File : oplus_device_fault_inject.c
** Description : framework for device fault inject test
** Version : 1.0
******************************************************************/

/*
 * depend on msm export symbol:
 *
*/

#include <linux/kthread.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <uapi/linux/sched/types.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include "soc/oplus/system/oplus_project.h"
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/string.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
#endif

#include "oplus_inject_proc.h"


enum PROC_INJECT_TYPE {
	PROC_INJECT_DIR,
	PROC_INJECT_FILE
};

#define OPLUS_INJECT_NAME  "oplus_fault_inject"
#ifndef MAX_PATH
#define MAX_PATH  256
#endif
#define MAX_INJECT_NOD_NAME 32

static struct proc_tree_node oplus_fault_root_proc_dir = {
	.name = OPLUS_INJECT_NAME,
};

static struct proc_tree_node *g_oplus_inject_root_proc =
		&oplus_fault_root_proc_dir;

static int valid_path_name(char *name)
{
	int val = 0;
	int i;
	unsigned char invchar[] = {':', ' ', '*', '?', '<', '>', '|'};

	if (unlikely(!name)) {
		val = -1;

	} else {
		if (strlen(name) > MAX_PATH) {
			return val;
		}

		for (i = 0; i < sizeof(invchar); i++) {
			if (strchr(name, invchar[i]) != NULL) {
				val = -1;
				break;
			}
		}
	}

	return val;
}

static int get_dir_cnt(char *name)
{
	int cnt = 0;
	int i;

	for (i = 0; i < strlen(name); i++) {
		if (name[i] == '/') {
			cnt++;
		}
	}

	return cnt;
}

static struct proc_tree_node *oplus_proc_dir_create(char *name, int type,
		struct proc_tree_node *pdir_parent, const struct proc_ops *ops, void *data)
{
	struct proc_tree_node *pnode, *tmp;
	struct proc_tree_node *pdir = NULL;

	/* scan the proc name check whether already create */
	list_for_each_entry_safe(pnode, tmp, &pdir_parent->sublist, sublist) {
		if (strcmp(pnode->name, name) == 0) {
			return pnode->proc_node;
		}
	}

	pdir = kzalloc(sizeof(struct proc_tree_node), GFP_KERNEL);

	if (!pdir) {
		pr_err("alloc memory failed");
		return ERR_PTR(-ENOMEM);
	}

	pdir->name = kzalloc(MAX_INJECT_NOD_NAME, GFP_KERNEL);

	if (!pdir) {
		kfree(pdir);
		pr_err("name alloc memory failed");
		return ERR_PTR(-ENOMEM);
	}

	strncpy(pdir->name, name, strlen(name));

	pdir->parent = pdir_parent;
	pdir->type = type;
	INIT_LIST_HEAD(&pdir->sublist);
	INIT_LIST_HEAD(&pdir->entry);

	if (type == PROC_INJECT_DIR) {
		pdir->proc_node = proc_mkdir(pdir->name, pdir_parent->proc_node);

		if (!pdir->proc_node) {
			kfree(pdir);
			pdir = NULL;
			pr_err("register dir failed");
			return NULL;
		}

	} else {
		pdir->proc_node = proc_create_data(pdir->name, S_IFREG | 0660,
						   pdir_parent->proc_node, ops, data);

		if (!pdir->proc_node) {
			kfree(pdir);
			pdir = NULL;
			pr_err("register %s file node failed", pdir->name);
			return NULL;
		}
	}

	list_add(&pdir_parent->sublist, &pdir->entry);
	return pdir;
}

static struct proc_tree_node *register_proc_node(char *name, int type,
		struct proc_tree_node *p_proc, const struct proc_ops *ops, void *data)
{
	struct proc_tree_node *pdir = NULL;
	struct proc_tree_node *pdir_parent = p_proc;

	if (!g_oplus_inject_root_proc->proc_node) {
		g_oplus_inject_root_proc->proc_node = proc_mkdir(g_oplus_inject_root_proc->name,
						      NULL);

		if (!g_oplus_inject_root_proc->proc_node) {
			pr_err("%s register root dir failed\n", __func__);
			return ERR_PTR(-ENOMEM);
		}

		INIT_LIST_HEAD(&g_oplus_inject_root_proc->sublist);
		INIT_LIST_HEAD(&g_oplus_inject_root_proc->entry);
	}

	if (!pdir_parent) {
		pdir_parent = g_oplus_inject_root_proc;
	}

	pdir = oplus_proc_dir_create(name, type, pdir_parent, ops, data);

	return pdir;
}

static struct proc_tree_node *create_proc_path(char *name,
		const struct proc_ops *ops, void *data)
{
	char path[MAX_PATH] = {0};
	const char *split = "/";
	char *token = NULL;
	char *ptr = NULL;
	struct proc_tree_node  *pnode = NULL;
	int cnt = get_dir_cnt(name);

	strncpy(path, name, strlen(name));
	ptr = path;

	pr_info("name %s ", ptr);
	token = strsep((char **)&ptr, split);

	while (cnt > 0 && token != NULL) {
		/* create dir */
		pr_info("get dir %s", token);
		pnode = register_proc_node(token, PROC_INJECT_DIR, pnode, NULL, NULL);

		if (!pnode) {
			pr_err("name failed");
                        return NULL;
		}

		token = strsep(&ptr, split);
		cnt--;
	}

	if (token != NULL) {
		pr_info("get file %s ", token);
		/* create filenode */
		pnode = register_proc_node(token, PROC_INJECT_FILE,  pnode, ops, data);

	} else {
		/* the name invalid only have  dir TBD need unregister the proc node */
		pr_err("name invalid nuly have dir");
		return NULL;
	}

	return pnode;
}

/* create proc tree */
struct proc_tree_node* oplus_proc_inject_init(char *name, const struct proc_ops *ops, void *data)
{
        struct proc_tree_node*  pnode = NULL;
	if (valid_path_name(name)) {
		pr_err("invalid path name");
		return NULL;
	}

	pnode = create_proc_path(name, ops, data);
	if (!pnode) {
		pr_err("create proc path failed");
	}

	return pnode;
}
EXPORT_SYMBOL(oplus_proc_inject_init);

static void remove_from_node(struct proc_tree_node *node)
{
	struct proc_tree_node *pnode, *safe;
	list_for_each_entry_safe(pnode, safe, &node->sublist, entry) {
		pr_info("kfree node %s ptr %llx", pnode->name, pnode);
		list_del(&pnode->entry);
		/* remove sublist */
		remove_from_node(pnode);
                proc_remove(pnode->proc_node);
		kfree(pnode->name);
		kfree(pnode);
	}
}

void oplus_proc_inject_exit(struct proc_tree_node *node)
{
	if (node) {
		pr_info("release proc path");
		remove_from_node(node);
                if (node == g_oplus_inject_root_proc) {
                        proc_remove(g_oplus_inject_root_proc->proc_node);
                        pr_info("release root proc path");
                }
	}
}
EXPORT_SYMBOL(oplus_proc_inject_exit);

/* test case for create proc and release proc node */
static ssize_t oplus_fault_test_read(struct file *file, char __user *buf,
				     size_t count, loff_t *off)
{
	char *str = "hello test case";
	return simple_read_from_buffer(buf, count, off, str, strlen(str));
}

static struct proc_ops oplus_fault_test_pops = {
	.proc_open = simple_open,
	.proc_read = oplus_fault_test_read,
	.proc_lseek = seq_lseek,
};

static ssize_t oplus_fault_inject_read(struct file *file, char __user *buf,
				       size_t count, loff_t *off)
{
	return -1;
}

static ssize_t oplus_fault_inject_write(struct file *file,
					const char __user *buf,
					size_t count, loff_t *off)
{
	char buffer[MAX_PATH] = {0};
	int length = (count < MAX_PATH) ? count : MAX_PATH;

	if (copy_from_user(buffer, buf, length)) {
		pr_err("%s: read proc input error.\n", __func__);
		return count;
	}

	buffer[length - 1]  = 0;

	if (strncmp(buffer, "cleanall", 8) == 0) {
		remove_from_node(g_oplus_inject_root_proc);
		pr_err("%s: free all file node except test node.\n", __func__);
	}

	if (valid_path_name(buffer)) {
		pr_err("invalid path name");
		return -1;
	}

	if (!create_proc_path(buffer, &oplus_fault_test_pops, NULL)) {
		pr_err("create test proc path failed");
		return -1;
	}

	return count;
}

static struct proc_ops oplus_fault_inject_pops = {
	.proc_open = simple_open,
	.proc_lseek = seq_lseek,
	.proc_read = oplus_fault_inject_read,
	.proc_write = oplus_fault_inject_write,
};

static int oplus_inject_proc_init(void)
{
	if (!g_oplus_inject_root_proc->proc_node) {
		g_oplus_inject_root_proc->proc_node = proc_mkdir(g_oplus_inject_root_proc->name,
						      NULL);

		if (!g_oplus_inject_root_proc->proc_node) {
			pr_err("%s register root dir failed\n", __func__);
			return -ENOMEM;
		}

		INIT_LIST_HEAD(&g_oplus_inject_root_proc->sublist);

		pr_info("%s register success\n", __func__);

                oplus_proc_inject_init("test", &oplus_fault_inject_pops, NULL);
	}

	return 0;
}

static void oplus_inject_proc_exit(void)
{
	oplus_proc_inject_exit(g_oplus_inject_root_proc);
}

module_init(oplus_inject_proc_init);
module_exit(oplus_inject_proc_exit);
MODULE_DESCRIPTION("OPLUS device fault inject driver");
MODULE_LICENSE("GPL v2");
