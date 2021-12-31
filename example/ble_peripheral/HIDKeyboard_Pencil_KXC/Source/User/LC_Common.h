/**
*	@file	LC_Common.h
*	@date	10/20/2020
*	@version	1.0.1
*
*/

/*!
 * 	@defgroup	LC_Common
 *	@brief
 *	@{*/
#ifndef		LC_COMMON_H_
#define		LC_COMMON_H_
/*------------------------------------------------------------------*/
/*						C++ guard macro								*/
/*------------------------------------------------------------------*/
#ifdef	__cplusplus
	 extern  "C" {
#endif
/*------------------------------------------------------------------*/
/* 				 		head files include 							*/
/*------------------------------------------------------------------*/

#include "att.h"
#include "bcomdef.h"
#include "gapbondmgr.h"
#include "gapgattserver.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "global_config.h"
#include "hci.h"
#include "hci_tl.h"
#include "linkdb.h"
#include "ll.h"
#include "ll_def.h"
#include "ll_hw_drv.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "ota_app_service.h"
#include "peripheral.h"
#include "pwm.h"
#include "pwrmgr.h"
#include "rf_phy_driver.h"
#include "Hidkbd.h"
#include "battservice.h"
#include "hiddev.h"
#include "osal_snv.h"
#include "flash.h"
#include "gpio.h"
#include "i2c.h"
#include "adc.h"
#include	"LC_Event_Handler.h"
/*------------------------------------------------------------------*/
/*						Pins definitions							*/
/*------------------------------------------------------------------*/
#define		GPIO_OUT_DCDC_EN	P0

#define		GPIO_CHGR_CHECK		P7
#define		GPIO_BAT_CHECK		P20
#define		GPIO_USB_CHECK		P18

/*------------------------------------------------------------------*/
/*						MACROS										*/
/*------------------------------------------------------------------*/
//	Battery State Value


//	Reserve date in FS
#define		SNV_DEV_SOFT_RESET			0xF1

#define		SNV_DEV_MAC_ADDR			0xF2
//	Timer interval
#define		TIMER_1S					1
#define		TIMER_2S					2
#define		TIMER_3S					3
#define		TIMER_6S					6

#define		TIMER_500MS					7

#define		LC_TIMER_INTERVAL			TIMER_1S
#if (LC_TIMER_INTERVAL == TIMER_6S)
#define		LC_DEV_TIMER_DISCON_PWROFF	( 5*10 + 1)
#define		LC_DEV_TIMER_POWEROFF		(10*10 + 1)
#define		LC_DEV_TIMER_SUSPEND		(10*10 + 1)
#elif (LC_TIMER_INTERVAL == TIMER_1S)
#define		LC_DEV_TIMER_DISCON_PWROFF	( 5*60 + 1)
#define		LC_DEV_TIMER_CON_PWROFF		(60*60 + 1)
#define		LC_DEV_TIMER_POWEROFF		(10*60 + 1)
#define		LC_DEV_TIMER_SUSPEND		(10*60 + 1)
#elif (LC_TIMER_INTERVAL == TIMER_500MS)
#define		LC_DEV_TIMER_DISCON_PWROFF	(60 + 1)
#define		LC_DEV_TIMER_POWEROFF		(10*60 + 1)
#define		LC_DEV_TIMER_SUSPEND		(10*60 + 1)
#endif

//	DCDC enable/disable
#define		DCDC_ENABLE()		hal_gpio_write(GPIO_OUT_DCDC_EN, 0)
#define		DCDC_DISABLE()		hal_gpio_write(GPIO_OUT_DCDC_EN, 1)
/*------------------------------------------------------------------*/
/*						UI Task Events definitions					*/
/*------------------------------------------------------------------*/


/*------------------------------------------------------------------*/
/* 					 	Data structures							 	*/
/*------------------------------------------------------------------*/
typedef uint8_t				u8;
typedef uint16_t			u16;
typedef uint32_t			u32;
typedef signed   char		int8_t;		//!< Signed 8 bit integer
typedef unsigned char		uint8_t;		//!< Unsigned 8 bit integer
typedef signed   short		int16_t;		//!< Signed 16 bit integer
typedef unsigned short		uint16_t;		//!< Unsigned 16 bit integer
typedef signed   int		int32_t;		//!< Signed 32 bit integer
typedef unsigned int		uint32_t;		//!< Unsigned 32 bit integer
typedef signed   char		int8;			//!< Signed 8 bit integer
typedef unsigned char		uint8;			//!< Unsigned 8 bit integer
typedef signed   short		int16;			//!< Signed 16 bit integer
typedef unsigned short		uint16;		//!< Unsigned 16 bit integer
typedef signed   long		int32;			//!< Signed 32 bit integer
typedef unsigned long		uint32;		//!< Unsigned 32 bit integer

typedef		enum
{
	LC_DEV_BLE_DISCONNECTION	=	0,
	LC_DEV_BLE_CONNECTION,
}lc_dev_ble_state;

typedef		enum
{
	State_Off	=	0,
	State_On	=	1,
}lc_state_bool;

typedef		enum
{
	SYSTEM_STANDBY	=	0,
	SYSTEM_WORKING	=	1,
	SYSTEM_SUSPEND	=	2,
	SYSTEM_CHARGINE	=	3,
}lc_sys_run_t;

typedef	enum
{
	OLD_PHONE	=	0,
	NEW_PHONE	=	1,
}phone_type_t;

typedef struct
{
	uint32			dev_timeout_poweroff_cnt;		//	power off time
	uint8			dev_timer_poweroff_flag;		//	power off timer enable flag			:1	enable,		0	disable
	uint8			dev_poweron_switch_flag;		//	power on switch exist or not		:1	no switch,	0	
	uint8			dev_power_flag;					//	device working flag					:1	working,	0	power off
	uint8			dev_lowpower_flag;				//	lower power mode flag				:1	
	uint8			dev_ble_con_state;				//	BLE	connection state				:1	connected,	0	disconnected
	phone_type_t	dev_phone_type;					//	phone type	
	uint8			dev_charging_flag;				//	1	charging,	0	no charging
	uint8			dev_charge_full;				//	1	charge full,0

}lc_dev_sys_param;


/*------------------------------------------------------------------*/
/* 					 external variables							 	*/
/*------------------------------------------------------------------*/
extern		lc_dev_sys_param	LC_Dev_System_Param;
extern		uint8				hidDevTaskId;
extern		uint8	ownPublicAddr[LL_DEVICE_ADDR_LEN];
/*------------------------------------------------------------------*/
/* 					 User function prototypes					 	*/
/*------------------------------------------------------------------*/
uint32	clock_time_exceed_func			(uint32 ref,		uint32 span_ms	);
void	LC_Common_ProcessOSALMsg	 	(osal_event_hdr_t *pMsg				);
void	LC_Timer_Start					(void);
void	LC_Timer_Stop					(void);

#ifdef	__cplusplus
}
#endif

#endif	/* LC_COMMON_H_ */
/** @}*/
