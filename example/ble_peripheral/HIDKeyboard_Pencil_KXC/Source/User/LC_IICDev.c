/**
 *	@file		LC_IICDev.c
 *	@author		YQ
 *	@date		01/20/2021
 *	@brief		drivers of ICM40607A.
 */

/*!
 *	@defgroup	LC_IICDev
 *	@brief
 *	@{*/

/*------------------------------------------------------------------*/
/*						head files include 							*/
/*------------------------------------------------------------------*/
#include "LC_IIC.h"
#include "LC_IICDev.h"
/*------------------------------------------------------------------*/
/* 					 	local variables			 					*/
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* 					 	public variables		 					*/
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* 					 	local functions			 					*/
/*------------------------------------------------------------------*/
static	void	LC_SensorDeinit(void)
{
	hal_gpio_pin_init(LC_IIC_GPIO_SCL, IE);
	hal_gpio_pull_set(LC_IIC_GPIO_SCL, FLOATING);

	hal_gpio_pin_init(LC_IIC_GPIO_SDA, IE);
	hal_gpio_pull_set(LC_IIC_GPIO_SDA, FLOATING);
}
/*------------------------------------------------------------------*/
/* 					 	public functions		 					*/
/*------------------------------------------------------------------*/
void	LC_SensorInit(void)
{
	LC_Gpio_IIC_Init();
	uint8	data[1]	=	{0};

	data[0]	=	LC_EEPROM_Read_One_Byte(GSENSOR_ID, CHIPID);
	LOG("chip id = %x\n",data[0]);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, SPI_CONFIG, 0x24);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, RESOLUTION_RANGE, 0x00);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, ODR_AXIS, 0x07);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, INT_CONFIG, 0x02);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, INT_LATCH, 0x0E);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, TAP_DUR, 0x02);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, TAP_THS, 0x1E);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, INT_MAP1, 0x20);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, INT_SET1, 0x20);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, MODE_BW, 0x50);
	LOG("set g-sensor finish\n");
	LC_SensorDeinit();
}

void	LC_SensorSuspend(void)
{
	LC_Gpio_IIC_Init();
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, MODE_BW, 0xDE);
	LOG("set g-sensor sleep\n");
	LC_SensorDeinit();
}

/** @}*/

