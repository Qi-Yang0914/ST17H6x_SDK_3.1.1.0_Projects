/**
*	@file	LC_Key.h
*	@date	09/17/2020
*	@version	1.0.0
*
*/

/*!
 * 	@defgroup	LC_Key
 *	@brief
 *	@{*/

#ifndef LC_KEY_H_
#define LC_KEY_H_

/*------------------------------------------------------------------*/
/*						C++ guard macro								*/
/*------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif
/*------------------------------------------------------------------*/
/*						head files include 							*/
/*------------------------------------------------------------------*/
#include "LC_Common.h"
#include "LC_UI_Led_Buzzer.h"
/*------------------------------------------------------------------*/
/*						Pins definitions							*/
/*------------------------------------------------------------------*/
#define		MY_KEY_NO1_GPIO		P23
#define		MY_KEY_NO2_GPIO		P26

#define		IIC_GPIO_SDA		P14
#define		IIC_GPIO_SCL		P11
#define		GSENSOR_ID			(0x26)

/**		G-Sensor DA260 Register		**/
#define		SPI_CONFIG			(0x00)
#define		CHIPID				(0x01)
#define		ACC_X_LSB			(0x02)
#define		ACC_X_MSB			(0x03)
#define		ACC_Y_LSB			(0x04)
#define		ACC_Y_MSB			(0x05)
#define		ACC_Z_LSB			(0x06)
#define		ACC_Z_MSB			(0x07)
#define		MOTION_FLAG			(0x09)
#define		NEWDATA_FLAG		(0x0A)
#define		TAP_ACTIVE_STATUS	(0x0B)
#define		ORIENT_STATUS		(0x0C)
#define		RESOLUTION_RANGE	(0x0F)
#define		ODR_AXIS			(0x10)
#define		MODE_BW				(0x11)
#define		SWAP_POLARITY		(0x12)
#define		INT_SET1			(0x16)
#define		INT_SET2			(0x17)
#define		INT_MAP1			(0x19)
#define		INT_MAP2			(0x1A)
#define		INT_MAP3			(0x1B)
#define		INT_CONFIG			(0x20)
#define		INT_LATCH			(0x21)
#define		FREEFALL_DUR		(0x22)
#define		FREEFALL_THS		(0x23)
#define		FREEFALL_HYST		(0x24)
#define		ACTIVE_DUR			(0x27)
#define		ACTIVE_THS			(0x28)
#define		TAP_DUR				(0x2A)
#define		TAP_THS				(0x2B)
/*------------------------------------------------------------------*/
/*						UI Task Events definitions					*/
/*------------------------------------------------------------------*/
#define		KEY_EVENT_LEVEL1	0x0001
#define		KEY_SYS_RESET		0x0002
#define		KEY_EVENT_SCANF		0x0004

#define		KEY_SCANF_EVT		0x0008
#define		KEY_STOPSCANF_EVT	0x0010
#define		KEY_CHARG_CHECK_EVT	0x0020

/*------------------------------------------------------------------*/
/*						Data structures								*/
/*------------------------------------------------------------------*/
typedef struct
{
	uint32 key_down_sys_tick; //	system time when key down
	uint8 key_down_flag;      //	1:key down	0:key release
	uint8 key_repeated_num;   //	times of key down
} lc_key_struct_data;

/*------------------------------------------------------------------*/
/*						external variables							*/
/*------------------------------------------------------------------*/
extern uint8 LC_Key_TaskID;
extern lc_key_struct_data LC_Key_Param;
/*------------------------------------------------------------------*/
/*						User function prototypes					*/
/*------------------------------------------------------------------*/
void	LC_Key_Gpio_Init(void);
void	LC_Key_Task_Init(uint8 task_id);
uint16	LC_Key_ProcessEvent(uint8 task_id, uint16 events);

#ifdef __cplusplus
}
#endif

#endif /* LC_KEY_H_ */
/** @}*/
