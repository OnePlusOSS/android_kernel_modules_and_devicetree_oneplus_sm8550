#ifndef __AK09970_H__
#define __AK09970_H__

#include <linux/ioctl.h>
#include <linux/mutex.h>

#define AK09970_REG_STATUS2			    (0x11)
#define AK09970_REG_STATUS3			    (0x19)
/*100ma*/
#define CURRENT_LOAD_UA					(100000)

/* feature of ic revision */
#define ak09970_REV_0_2					(0x02)
#define ak09970_REV_1_0					(0x10)
#define ak09970_REV					ak09970_REV_1_0
#define AK09970_I2C_REG_MAX_SIZE	(8)

/* property of driver */
#define ak09970_DRIVER_NAME				"ak09970"
#define ak09970_IRQ_NAME					"ak09970-irq"
#define ak09970_PATH					"/dev/ak09970"
#define ak09970_SLAVE_ADDR				(0x18)

/* register map */
#define ak09970_REG_PERSINT				(0x00)
#define ak09970_VAL_PERSINT_COUNT				(0x80)
#define ak09970_VAL_PERSINT_INTCLR			(0x01)
/*
[7:4]	PERS		: interrupt persistence count
[0]	INTCLR	= 1	: interrupt clear
*/

#define AK09970_REG_CNTL1			(0x20)
#define AK09970_DRDYEN              (0x01)
#define AK09970_SWX1EN              (0x02)
#define AK09970_SWX2EN              (0x04)
#define AK09970_SWY1EN              (0x08)
#define AK09970_SWY2EN              (0x10)
#define AK09970_SWZ1EN              (0x20)
#define AK09970_SWZ2EN              (0x40)
#define AK09970_ALL_EN              (0x7F)

#define AK09970_OINTEN              (0x04)

#define ak09970_VAL_INTSRS_INT_ON			(0x80)
#define ak09970_DETECTION_MODE_INTERRUPT		ak09970_VAL_INTSRS_INT_ON
#define ak09970_VAL_INTSRS_INT_OFF		(0x00)
#define ak09970_DETECTION_MODE_POLLING		ak09970_VAL_INTSRS_INT_OFF
#define ak09970_VAL_INTSRS_INTTYPE_BESIDE		(0x00)
#define ak09970_VAL_INTSRS_INTTYPE_WITHIN		(0x10)

/*0 HIGH_SENSITIVITY,1 WIDE_RANGE*/
#define AK09970_SMR_BIT             (5)

#define AK09970_HIGH_SENSITIVITY	(0x00)
#define AK09970_WIDE_RANGE	        (0x01)

/*OH, OL, RH, RL*/
#define AK09970_REG_SWX1                        0x22

#define ak09970_REG_I2CDIS					(0x06)
#define ak09970_VAL_I2CDISABLE					(0x37)
/*[7:0] I2CDIS	: disable i2c */

/*SRSTL*/
#define AK09970_REG_SRST                        0x30

#define AK09970_VAL_SRST_RESET					(0x01)
/*[0]	SRST	= 1	: soft reset*/

#define ak09970_REG_HIDDEN					(0x6F)
#define ak09970_REG_BIAS						(0x5C)
#define ak09970_VAL_HIDDEN_EANBLE				(0x6E)

#define AK09970_REG_CNTL2						(0x21)
#define AK09970_MODE_POWER_DOWN                 (0x00)
#define AK09970_MODE_SNG_MEASURE                (0x01)
/*0.25Hz  5Hz*/
#define AK09970_MODE_CONT_MEASURE_MODE1         (0x02)
/*0.5Hz  10Hz*/
#define AK09970_MODE_CONT_MEASURE_MODE2         (0x04)
/*1Hz  20Hz*/
#define AK09970_MODE_CONT_MEASURE_MODE3         (0x06)
/*10Hz  50Hz*/
#define AK09970_MODE_CONT_MEASURE_MODE4         (0x08)
/*20Hz  100Hz*/
#define AK09970_MODE_CONT_MEASURE_MODE5         (0x0A)
/*50Hz  500Hz*/
#define AK09970_MODE_CONT_MEASURE_MODE6         (0x0C)
/*100Hz  1000Hz*/
#define AK09970_MODE_CONT_MEASURE_MODE7         (0x0E)
/*Test  2000Hz*/
#define AK09970_MODE_SELF_TEST                  (0x10)

#define ak09970_VAL_OPF_EFRD_ON					(0x08)
#define ak09970_VAL_OPF_BIT_8						(0x02)
#define ak09970_VAL_OPF_BIT_10					(0x00)
#define ak09970_VAL_OPF_HSSON_ON					(0x01)
/*
[6:4]	OPF	: operation frequency
		000		: 20	(Hz)
		001		: 10	(Hz)
		010		: 6.7	(Hz)
		011		: 5		(Hz)
		100		: 80	(Hz)
		101		: 40	(Hz)
		110		: 26.7	(Hz)
		111		: 20	(Hz)
[3]		EFRD	= 0	: keep data without accessing eFuse
[3]		EFRD	= 1	: update data after accessing eFuse
[1]		BIT		= 0	: 10 bit resolution
[1]		BIT		= 1	: 8 bit resolution
[0]		HSSON	= 0 : Off power down mode
[0]		HSSON	= 1 : On power down mode
*/

#define AK09970_REG_DID						(0x00)
#define AK09970_WIA_VAL                      (0x48C1)

/*[7:0] DID	: Device ID */

#define ak09970_REG_INFO						(0x0A)
/*[7:0] INFO	: Information about IC */

#define ak09970_REG_ASA						(0x0B)
/*[7:0] ASA	: Hall Sensor sensitivity adjustment*/

#define AK09970_REG_ST1						(0x10)
#define AK09970_REG_ST1_ZYX						(0x17)
#define AK09970_REG_ST1_V						(0x18)
#define ak09970_VAL_ST1_DRDY					(0x01)
/*
[4]	INTM	: status of interrupt mode
[1]	BITM	: status of resolution
[0]	DRDY	: status of data ready
*/

#define ak09970_REG_HSL						(0x11)
/* [7:0]HSL	: low byte of hall sensor measurement data*/


#define ak09970_REG_HSH						(0x12)
/*[7:6] HSL	: high 2bits of hall sensor measurement data with sign*/

/* event property */
#define DEFAULT_EVENT_TYPE			EV_ABS
#define DEFAULT_EVENT_CODE			ABS_X
#define DEFAULT_EVENT_DATA_CAPABILITY_MIN	(-32768)
#define DEFAULT_EVENT_DATA_CAPABILITY_MAX	(32767)

/* delay property (ms)*/
#define ak09970_DELAY_MAX				(200)
#define ak09970_DELAY_MIN				(50)
#define ak09970_DELAY_FOR_READY			(60)

/*threeaxis hall algorithm para */
#define MAX_SQRT_TIME			1024
#define YBOP_TOL			        3001
#define YBRP_TOL			        3000
#define XBOP_TOL			        2001
#define XBRP_TOL			        2000


#define ak09970_DETECTION_MODE				ak09970_DETECTION_MODE_INTERRUPT
#define ak09970_INTERRUPT_TYPE				ak09970_VAL_INTSRS_INTTYPE_WITHIN
#define ak09970_SENSITIVITY_TYPE				ak09970_VAL_INTSRS_SRS_0_04mT
#define ak09970_PERSISTENCE_COUNT				ak09970_VAL_PERSINT_COUNT
#define AK09970_OPERATION_FREQUENCY			AK09970_MODE_SNG_MEASURE
#define ak09970_OPERATION_RESOLUTION			ak09970_VAL_OPF_BIT_10


struct oplus_dhall_chip {
	struct i2c_client	*client;
	struct pinctrl *pctrl;
	struct pinctrl_state *power_state;
	struct pinctrl_state *irq_state;
	bool			irq_enabled;
	unsigned int		id;
	int			calibrated_data;
	int			irq_source;
	short			value_30degree;
	short			value_70degree;
	short			thrhigh;
	short			thrlow;
	bool			last_state;
	struct delayed_work	work;

	struct regulator 	*power_1v8;
	struct regulator 	*power_2v8;
	int			irq_gpio;
	int         reset_gpio;
	int			irq;
	unsigned int		power_gpio;
	bool			is_power_on;
	bool enable_hidden;
	unsigned int bias_ratio;

	bool irq_wake;
	wait_queue_head_t wait;
	struct wakeup_source *ws;                           /*Qualcomm KBA-211220012446, To make power manager stay awake*/
};

#endif  /* __AK09970_H__ */

