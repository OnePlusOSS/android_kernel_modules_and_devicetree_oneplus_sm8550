#ifndef __OPLUS_TYPEC_SWITCH_H__
#define __OPLUS_TYPEC_SWITCH_H__
//-----------------------------------------------------------------------------
#include "dio4480.h"
#include "dio4483.h"
#include "common_reg.h"

//-----------------------------------------------------------------------------
#define OPLUS_ARCH_EXTENDS
//-----------------------------------------------------------------------------
#define TYPEC_SWITCH_I2C_NAME	"oplus-typec-switch-driver"
//-----------------------------------------------------------------------------
#define GET_BIT(x, bit)				((x & (1 << bit)) >> bit)		/* Get bit x */
#define GET_BITS(_var, _index, _width)                                  \
	(((_var) >> (_index)) & ((0x1 << (_width)) - 1))
#define CLEAR_BIT(x, bit)			(x &= ~(1 << bit))				/* Reset bit x */
#define SET_BIT(x, bit)				(x |= (1 << bit))				/* Set bit x */
//-----------------------------------------------------------------------------
#define DIO_REG_DEVICE_ID		0x00
//-----------------------------------------------------------------------------
enum TYPEC_AUDIO_SWITCH_STATE {
	TYPEC_AUDIO_SWITCH_STATE_DPDM = 0x0,
	TYPEC_AUDIO_SWITCH_STATE_FAST_CHG = 0x1,
	TYPEC_AUDIO_SWITCH_STATE_AUDIO = 0x1 << 1,
	TYPEC_AUDIO_SWITCH_STATE_UNKNOW = 0x1 << 2,
	TYPEC_AUDIO_SWITCH_STATE_SUPPORT = 0x1 << 4,
	TYPEC_AUDIO_SWITCH_STATE_NO_RAM = 0x1 << 5,
	TYPEC_AUDIO_SWITCH_STATE_I2C_ERR = 0x1 << 8,
	TYPEC_AUDIO_SWITCH_STATE_INVALID_PARAM = 0x1 << 9,
};

enum typec_switch_function {
	TYPEC_SWITCH_MIC_GND_SWAP,
	TYPEC_SWITCH_USBC_ORIENTATION_CC1,
	TYPEC_SWITCH_USBC_ORIENTATION_CC2,
	TYPEC_SWITCH_USBC_DISPLAYPORT_DISCONNECTED,
	TYPEC_SWITCH_EVENT_MAX,
};

enum TYPEC_SWITCH_CHIP {
	DIO_CHIP_4480 		= 0xF1,
	DIO_CHIP_4483 		= 0xF3,
	DIO_CHIP_4483_2 	= 0xF5,
	WAS_CHIP_4783		= 0x31,
	TYPEC_SWITCH_CHIP_MAX 	= 0xFF,
};

enum typec_switch_vendor {
	DIO4480,
	DIO4483,
	WAS4783,
	DIO_MAX
};
//-----------------------------------------------------------------------------
#endif
