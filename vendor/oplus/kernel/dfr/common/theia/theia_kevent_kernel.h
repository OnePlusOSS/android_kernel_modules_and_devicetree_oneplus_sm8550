// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2018-2020 Oplus. All rights reserved.
*/
#ifndef __THEIA_KEVENT_KERNEL_H_
#define __THEIA_KEVENT_KERNEL_H_

#define MAX_PAYLOAD_TAG_SIZE            128
#define MAX_PAYLOAD_EVENTID_SIZE        128
#define MAX_PAYLOAD_DATASIZE            1024

#define THEIA_OK 0

/* adapt using generic netlink */
#define THEIA_FAMILY_VERSION  1
#define THEIA_FAMILY_NAME  "theia_monitor"
#define GENL_ID_GENERATE 0

/* attribute type define */
enum theia_attr_type{
	THEIA_ATTR_TYPE_UNDEFINED,
	/* uplink, theia_kevent_packet.data: string */
	THEIA_ATTR_TYPE_COMMON_STRING,
	/*
	 * uplink, theia_kevent_packet.tag: dcs tag
	 * theia_kevent_packet.event_id: dcs event id
	 * theia_kevent_packet.data: is logmap format: logmap{key1:value1;key2:value2;key3:value3 ...}
	 */
	THEIA_ATTR_TYPE_DCS_MSG,
	THEIA_ATTR_TYPE_SEND_EVENT,
	__THEIA_ATTR_TYPE_MAX
};
#define THEIA_ATTR_TYPE_MAX (__THEIA_ATTR_TYPE_MAX - 1)

/* cmd type define */
enum {
	THEIA_CMD_UNSPEC,
	THEIA_CMD_DOWNLINK,
	THEIA_CMD_UPLINK,
	__THEIA_CMD_MAX
};
#define THEIA_CMD_MAX (__THEIA_CMD_MAX - 1)

#define THEIA_KEVENT_MODULE 	"theia_kevent"

struct theia_kevent_packet {
	int type;  /* reserve for userspace compatible for genl. 1: common string 2: dcs message for compatible */
	char tag[MAX_PAYLOAD_TAG_SIZE];            /* tag */
	char event_id[MAX_PAYLOAD_EVENTID_SIZE];     /* eventID */
	size_t len;                 /* Length of packet data */
	unsigned char data[0];            /* Optional packet data */
}__attribute__((packed));

int theia_kevent_module_init(void);
void theia_kevent_module_exit(void);
void SendTheiaKevent(int type, char *log_tag, char *event_id, char *payload);
void SendDcsTheiaKevent(char *log_tag, char *event_id, char *logmap);
#endif /* __THEIA_KEVENT_KERNEL_H_ */
