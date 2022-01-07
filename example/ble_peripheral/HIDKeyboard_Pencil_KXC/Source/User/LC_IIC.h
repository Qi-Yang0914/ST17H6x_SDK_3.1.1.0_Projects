/**
 *	@file		LC_IIC.h
 *	@author		YQ
 *	@data		11/11/2020
 *	@version	1.0.0
 */

/*!
 *	@defgroup	LC_IIC
 *	@brief
 *	@{*/

#ifndef		LC_IIC_H_
#define		LC_IIC_H_
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
#define		LC_IIC_GPIO_SDA			P14
#define		LC_IIC_GPIO_SCL			P11
/*------------------------------------------------------------------*/
/*						MACROS										*/
/*------------------------------------------------------------------*/

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
extern	void	LC_Gpio_IIC_Init				(void										);
extern	void	LC_EEPROM_Read_Byte_Sequence	(uint8 id, uint8 addr, uint8 *p,uint8 len	);
extern	void	LC_EEPROM_Write_Byte_Sequence	(uint8 id, uint8 addr, uint8 *p, uint8 len	);
extern	void	LC_EEPROM_Write_One_Byte		(uint8 id, uint8 addr, uint8 data			);
extern	uint8	LC_EEPROM_Read_One_Byte			(uint8 id, uint8 addr						);
#ifdef	__cplusplus
	}
#endif

#endif	/**	LC_IIC.h **/
/**	@}*/

