/**
 *	@file		LC_IICDev.h
 *	@date		01/07/2022
 *	@version	1.0.0
 */

/*!
 *	@defgroup	LC_IICDev
 *	@brief
 *	@{*/

#ifndef		LC_IICDEV_H_
#define		LC_IICDEV_H_
/*------------------------------------------------------------------*/
/*						C++ guard macro								*/
/*------------------------------------------------------------------*/
#ifdef	__cplusplus
	extern "C"	{
#endif
/*------------------------------------------------------------------*/
/*						head files include 							*/
/*------------------------------------------------------------------*/
#include "LC_Common.h"
/*------------------------------------------------------------------*/
/*						Pins definitions							*/
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/*						MACROS										*/
/*------------------------------------------------------------------*/
#define		GSENSOR_ID			(0x4E)

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

/*------------------------------------------------------------------*/
/*						Data structures								*/
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/*						external variables							*/
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/*						User function prototypes					*/
/*------------------------------------------------------------------*/
void	LC_SensorInit(void);
void	LC_SensorSuspend(void);
#ifdef	__cplusplus
	}
#endif

#endif	/**	LC_IICDev.h **/
/**	@}*/

