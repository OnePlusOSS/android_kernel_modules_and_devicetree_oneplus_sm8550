#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/cred.h>

#include <linux/socket.h>

#include "oplus_harden_hook.h"
#include "oplus_kevent.h"
#include "oplus_guard_general.h"

/*kevent string*/
#define OPLUS_HARDEN_CAPABILITY_EVENT_NAME    "kernel_event"
#define OPLUS_HARDEN_CAPABILITY_EVENT_ID    "capa_harden"
#define OPLUS_HARDEN_CAPABILITY_EVENT_TYPE    5

#define OPLUS_ANDROID_THIRD_PART_APK_UID     10000
#define OPLUS_ANDROID_ROOT_UID    0

#define OPLUS_SETID_ID	1
#define OPLUS_SETID_RE	2
#define OPLUS_SETID_RES	4


#define OPLUS_HARDEN_UID                         0
#define OPLUS_HARDEN_EUID                        1
#define OPLUS_HARDEN_RESUID                      2
#define OPLUS_HARDEN_GID                         3
#define OPLUS_HARDEN_EGID                        4
#define OPLUS_HARDEN_RESGID                      5
#define OPLUS_HARDEN_NET_RAW                     0xf0
#define OPLUS_HARDEN_NET_RAW_FLAG                0xff

extern bool capable(int cap);
extern int kevent_send_to_user(struct kernel_packet_info *userinfo);
static int selinux_enabled = 1;

static void oplus_secure_harden_kevent(uid_t new_id, uid_t new_eid, uid_t new_sid, int flag, unsigned char uid_or_gid)
{
	int ret = -1;

	struct kernel_packet_info *dcs_event;
	char dcs_stack[sizeof(struct kernel_packet_info) + 256];
	char* dcs_event_payload = NULL;

	char comm[TASK_COMM_LEN];
	memset(comm, 0, TASK_COMM_LEN);

	dcs_event = (struct kernel_packet_info*)dcs_stack;
	dcs_event->type = OPLUS_HARDEN_CAPABILITY_EVENT_TYPE;
	strncpy(dcs_event->log_tag, OPLUS_HARDEN_CAPABILITY_EVENT_NAME, sizeof(dcs_event->log_tag));
	strncpy(dcs_event->event_id, OPLUS_HARDEN_CAPABILITY_EVENT_ID, sizeof(dcs_event->event_id));
	dcs_event_payload = kmalloc(256, GFP_ATOMIC);
	if (NULL == dcs_event_payload){
		return;
	}
	memset(dcs_event_payload, 0, 256);

	if ((uid_or_gid >= OPLUS_HARDEN_UID)&&(uid_or_gid <= OPLUS_HARDEN_RESUID)) {
		dcs_event->payload_length = snprintf(dcs_event_payload, 256,
	    "%d$$new_euid@@%d$$new_suid@@%d$$set_id_flag@@%d$$addr_limit@@%lx$$curr_uid@@%d$$curr_euid@@%d$$curr_suid@@%d$$curr_name@@%s$$enforce@@%d\n",
	    new_id, new_eid, new_sid, flag,
	    get_fs(), current_uid().val, current_euid().val, current_suid().val, get_task_comm(comm, current), selinux_enabled);
	}
	else if ((uid_or_gid >= OPLUS_HARDEN_GID)&&(uid_or_gid <= OPLUS_HARDEN_RESGID)){
		dcs_event->payload_length = snprintf(dcs_event_payload, 256,
	    "%d$$new_egid@@%d$$new_sgid@@%d$$set_id_flag@@%d$$addr_limit@@%lx$$curr_gid@@%d$$curr_egid@@%d$$curr_sgid@@%d$$curr_name@@%s$$enforce@@%d\n",
	    new_id, new_eid, new_sid, flag,
	    get_fs(), current_gid().val, current_egid().val, current_sgid().val, get_task_comm(comm, current), selinux_enabled);
	}
	else {
		dcs_event->payload_length = snprintf(dcs_event_payload, 256,
	    "%d$$type@@%d$$protocol@@%d$$set_id_flag@@%d$$addr_limit@@%lx$$curr_gid@@%d$$curr_egid@@%d$$curr_sgid@@%d$$curr_name@@%s$$enforce@@%d\n",
	    new_id, new_eid, new_sid, flag,
	    get_fs(), current_gid().val, current_egid().val, current_sgid().val, get_task_comm(comm, current), selinux_enabled);
	}

	pr_info("[ROOTCHECK-CAP-INFO]:oplus_root_check_succ,payload:%s\n", dcs_event_payload);

	memcpy(dcs_event->payload, dcs_event_payload, strlen(dcs_event_payload));

	ret = kevent_send_to_user(dcs_event);
	if (ret != 0 ){
		printk(KERN_INFO "[ROOTCHECK-EXEC2-ERROR]:Send to user failed\n");
	}

	kfree(dcs_event_payload);

	return;
}
/*
There are 2 blocking solution:
1.Intercept knowledge and raise the right to increase the system permission.
2.Only the interception permission is raised to ROOT.(Currently used)
*/
/*UID*/
static int oplus_harden_setuid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_uid = 0;
	unsigned int new_uid = 0;
	cur_uid = current_uid().val;
	new_uid = regs->regs[0];

	if (capable(CAP_SETUID) == true) {
        if (cur_uid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {

	        if (new_uid == OPLUS_ANDROID_ROOT_UID) {
                pr_err("[ROOTCHECK-CAP-ERROR]:CAP security incident detected, old->uid.val is %u, new->uid.val is %u\n",
			           cur_uid, new_uid);
	    oplus_secure_harden_kevent(new_uid, new_uid, new_uid, OPLUS_SETID_ID, OPLUS_HARDEN_UID);
            /*restore the uid*/	
            //regs->regs[0] = cur_uid;	
		}
	    }
    }
    return ret;
}

/*EUID*/
static int oplus_harden_setreuid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_uid = 0;
	unsigned int cur_euid = 0;
	unsigned int new_uid = 0;
	unsigned int new_euid = 0;
	cur_uid = current_uid().val;
	cur_euid = current_euid().val;

	new_uid = regs->regs[0];
	new_euid = regs->regs[1];

	if (capable(CAP_SETUID) == true) {
		if (cur_uid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if ((new_uid == OPLUS_ANDROID_ROOT_UID)||( new_euid == OPLUS_ANDROID_ROOT_UID)) {
				pr_err("[ROOTCHECK-CAP-ERROR]:CAP security incident detected, old->uid.val is %u, new->uid.val is %u, new->euid.val is %u \n",
					   cur_uid, new_uid, new_euid);
				oplus_secure_harden_kevent(new_uid, new_euid, new_uid, OPLUS_SETID_RE, OPLUS_HARDEN_EUID);
				/*restore the uid*/
				//regs->regs[0] = cur_uid;
				//regs->regs[1] = cur_euid;
			}
		}
	}
    return ret;
}

/*RESUID*/
static int oplus_harden_setresuid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_uid = 0;
	unsigned int cur_euid = 0;
	unsigned int cur_suid = 0;
	unsigned int new_uid = 0;
	unsigned int new_euid = 0;
	unsigned int new_suid = 0;
	cur_uid = current_uid().val;
	cur_euid = current_euid().val;
	cur_suid = current_suid().val;

	new_uid = regs->regs[0];
	new_euid = regs->regs[1];
	new_suid = regs->regs[2];

	if (capable(CAP_SETUID) == true) {
		if (cur_uid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if ((new_uid == OPLUS_ANDROID_ROOT_UID) || (new_euid == OPLUS_ANDROID_ROOT_UID) || (new_suid == OPLUS_ANDROID_ROOT_UID)) {
				pr_err("[ROOTCHECK-CAP-ERROR]:CAP security incident detected, old->uid.val is %u, new->uid.val is %u, new->euid.val is %u, new->suid.val is %u \n",
					   cur_uid, new_uid, new_euid, new_suid);
				oplus_secure_harden_kevent(new_uid, new_euid, new_suid, OPLUS_SETID_RES, OPLUS_HARDEN_RESUID);
				/*restore the uid*/
				//regs->regs[0] = cur_uid;
				//regs->regs[1] = cur_euid;
				//regs->regs[2] = cur_suid;
			}
		}
	}
    return ret;
}

/*GID*/
static int oplus_harden_setgid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_gid = 0;
	unsigned int new_gid = 0;
	cur_gid = current_gid().val;
	new_gid = regs->regs[0];

	if (capable(CAP_SETGID) == true) {
		if (cur_gid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if (new_gid == OPLUS_ANDROID_ROOT_UID) {
				pr_err("[ROOTCHECK-CAP-ERROR]:CAP security incident detected, old->gid.val is %u, new->gid.val is %u\n",
					   cur_gid, new_gid);
				oplus_secure_harden_kevent(new_gid, new_gid, new_gid, OPLUS_SETID_ID, OPLUS_HARDEN_GID);
				/*restore the gid*/
				//regs->regs[0] = cur_gid;	
			}
		}
	}
    return ret;
}

/*EGID*/
static int oplus_harden_setregid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_gid = 0;
	unsigned int cur_egid = 0;
	unsigned int new_gid = 0;
	unsigned int new_egid = 0;
	cur_gid = current_uid().val;
	cur_egid = current_euid().val;
	new_gid = regs->regs[0];
	new_egid = regs->regs[1];

	if (capable(CAP_SETGID) == true) {
		if (cur_gid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if ((new_gid == OPLUS_ANDROID_ROOT_UID) || (new_egid == OPLUS_ANDROID_ROOT_UID)) {
				pr_err("[ROOTCHECK-CAP-ERROR]:CAP security incident detected, old->gid.val is %u, new->gid.val is %u, new->egid.val is %u \n",
					   cur_gid, new_gid, new_egid);
				oplus_secure_harden_kevent(new_gid, new_egid, new_gid, OPLUS_SETID_RE, OPLUS_HARDEN_EGID);
				/*restore the gid*/
				//regs->regs[0] = cur_gid;
				//regs->regs[1] = cur_egid;
			}
		}
	}
    return ret;
}

/*RESGID*/
static int oplus_harden_setresgid(struct pt_regs *regs)
{
	int ret = 0;
	unsigned int cur_gid = 0;
	unsigned int cur_egid = 0;
	unsigned int cur_sgid = 0;
	unsigned int new_gid = 0;
	unsigned int new_egid = 0;
	unsigned int new_sgid = 0;
	cur_gid = current_gid().val;
	cur_egid = current_egid().val;
	cur_sgid = current_sgid().val;
	new_gid = regs->regs[0];
	new_egid = regs->regs[1];
	new_sgid = regs->regs[2];

	if (capable(CAP_SETGID) == true) {
		if (cur_gid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
			if ((new_gid == OPLUS_ANDROID_ROOT_UID) || (new_egid == OPLUS_ANDROID_ROOT_UID) || (new_sgid == OPLUS_ANDROID_ROOT_UID)) {
				pr_err("[ROOTCHECK-CAP-ERROR]:CAP security incident detected, old->gid.val is %u, new->gid.val is %u, new->egid.val is %u, new->sgid.val is %u \n",
					   cur_gid, new_gid, new_egid, new_sgid);
				oplus_secure_harden_kevent(new_gid, new_egid, new_sgid, OPLUS_SETID_RES, OPLUS_HARDEN_RESGID);
				/*restore the gid*/
				//regs->regs[0] = cur_gid;
				//regs->regs[1] = cur_egid;
				//regs->regs[2] = cur_sgid;
			}
		}
	}
    return ret;
}

void oplus_harden_pre_handler(void *data, struct pt_regs *regs, long id)
{
	int scno;
	int ret = 0;
	scno = regs->syscallno;
  
	if(is_unlocked())
		return;

	switch (scno)
	{
	    case __NR_setregid:
		    ret = oplus_harden_setregid(regs);
	        break;
	    case __NR_setgid:
		    ret = oplus_harden_setgid(regs);
	        break;
		case __NR_setreuid:
		    ret = oplus_harden_setreuid(regs);
	        break;
		case __NR_setuid:
		    ret = oplus_harden_setuid(regs);
	        break;
	    case __NR_setresuid:
		    ret = oplus_harden_setresuid(regs);
	        break;
		case __NR_setresgid:
			ret = oplus_harden_setresgid(regs);
	        break;
		default:
		    break;
	}
	
	return;
}

/*SOCKET*/
static int oplus_harden_socket_create(struct pt_regs *regs)
{
	unsigned int cur_uid = 0;
	int family;
	int type;
	int protocol;

	cur_uid = current_uid().val;
	family = regs->regs[0];
	type = regs->regs[1];
	protocol = regs->regs[2];

    if (cur_uid >= OPLUS_ANDROID_THIRD_PART_APK_UID) {
	    if ((family != AF_NETLINK)&&(type == SOCK_RAW)&&!capable(CAP_NET_RAW)) {
		    oplus_secure_harden_kevent((uid_t)family, (uid_t)type, (uid_t)protocol, OPLUS_HARDEN_NET_RAW_FLAG, OPLUS_HARDEN_NET_RAW);
	        pr_err("[ROOTCHECK-CAP-ERROR]:Missing CAP_NET_RAW for SOCK_RAW. family=%d, proto=%d\n", family, protocol);
            return -EPERM;
		}
	}

	return 0;
}

void oplus_harden_post_handler(void *data, struct pt_regs *regs, long ret)
{
	int scno;
	int ret_val = 0;
	scno = regs->syscallno;

	if(is_unlocked())
		return;

	/*if socket has no cap return err*/
	if (scno == __NR_socket) {
	    ret_val = oplus_harden_socket_create(regs);
	    if (ret_val) {
                /* regs->regs[0] = -EPERM;*/
	    }
	}
}
