/* SPDX-License-Identifier: GPL-2.0-only */
/**************************************************************
* Copyright (c)  2019- 2039  Oplus. All rights reserved..
*
* File       : oplus_secure_harden.h
* Description:Security harden for Oplus kernel
* Version   : 1.0
* Date        : 2021-2-25
* Author    :
* TAG         :
****************************************************************/
#ifndef __CONFIG_OPLUS_FEATURE_VDSO_CHECK__
#define __CONFIG_OPLUS_FEATURE_VDSO_CHECK__
#include <linux/const.h>


#define CHECK_ROOT_CREDS(x)	(uid_eq(x->cred->uid, GLOBAL_ROOT_UID) || \
							gid_eq(x->cred->gid, GLOBAL_ROOT_GID) || \
							uid_eq(x->cred->euid, GLOBAL_ROOT_UID) || \
							gid_eq(x->cred->egid, GLOBAL_ROOT_GID))

#define CHECK_SHELL_CREDS(x)     (uid_eq(x->cred->uid, GLOBAL_SHELL_UID) || \
                                                        gid_eq(x->cred->gid, GLOBAL_SHELL_GID) || \
                                                        uid_eq(x->cred->euid, GLOBAL_SHELL_UID) || \
                                                        gid_eq(x->cred->egid, GLOBAL_SHELL_GID))

/* ---CONFIG_OPLUS_HP_CHECK --- */
#define MCAST_MSFILTER		48
#define MCAST_JOIN_GROUP	42
#define IP_MSFILTER			41

#define HEAPSPARY_TIME_LIMIT 10
#define HEAPSPART_COUNT_LIMIT 200
#if 0
enum HeapSpary_type{
       KEVENT_ROOT_EVENT = 0,
       KEVENT_STRING,
       KEVENT_REMOUNT_EVENT,
       KEVENT_EXEC_EVENT,
       RESERVER,                /*reserved*/
       KEVENT_HARDEN_EVENT,     /* = 5*/
       CPU_INFO,                /* = 6 */
       SET_XATTR,               /* = 7 */
       MCAST_MSFILTER_IP4,      /* = 8 */
       MCAST_JOIN_GROUP_IP4,    /* = 9 */
       IP_MSFILTER_IP4,         /* = 10 */
       MCAST_JOIN_GROUP_IP6,    /* = 11 */
       MCAST_MSFILTER_IP6,      /* = 12 */
       SEPOLICY_RL,			 	/* = 13 */
       MEM_RW,	 				/* = 14 */
       EXEC2_EVENT,			 	/* = 15 */
};
#endif
enum kernel_kevent_type {
    KEVENT_ROOT_EVENT = 0,
    KEVENT_STRING,
    KEVENT_REMOUNT_EVENT,
    KEVENT_EXEC_EVENT,
    RESERVER,                   /*reserved*/
    KEVENT_HARDEN_EVENT,        /* = 5 */
    CPU_INFO,                   /* = 6 */
    SET_XATTR,                  /* = 7 */
    MCAST_MSFILTER_IP4,         /* = 8 */
    MCAST_JOIN_GROUP_IP4,       /* = 9 */
    IP_MSFILTER_IP4,            /* = 10 */
    MCAST_JOIN_GROUP_IP6,       /* = 11 */
    MCAST_MSFILTER_IP6,         /* = 12 */
    SEPOLICY_RL,                /* = 13 */
    EXEC2_EVENT,                /* = 14 */
};

/*Record 3 sets of data of different api frequencies*/
extern unsigned int heapspary_cpuinfo[3];
extern unsigned int heapspary_xttr[3];
extern unsigned int heapspary_ip4[10];
extern unsigned int heapspary_ip6[10];


void oplus_heapspray_check(unsigned int type);



#ifdef CONFIG_OPLUS_SECURE_VDSO_CHECK
#define SET_MASK_RW 0
#define CLEAR_MASK_RW 128
#if 0
/*For MEM_RW*/
#define MODULES_VADDR	_AC(0x0000000010000000,UL)
#define MODULES_LEN		_AC(0x00000000e0000000,UL)
#define MODULES_END		_AC(0x00000000f0000000,UL)
extern char vdso_start[], vdso_end[];
#endif
int oplus_check_selinux_load(void);
int oplus_check_set_procattr(const char *value, size_t size);
void report_secuiry_event(const char* event_name, unsigned int event_type, const char* more);

#endif /* CONFIG_OPLUS_SECURE_VDSO_CHECK */
#endif /* __CONFIG_OPLUS_FEATURE_VDSO_CHECK__ */
