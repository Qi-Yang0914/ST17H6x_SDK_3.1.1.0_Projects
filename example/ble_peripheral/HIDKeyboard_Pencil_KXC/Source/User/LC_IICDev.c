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

	data[0]	=	LC_EEPROM_Read_One_Byte(GSENSOR_ID, CHIPID);		//	DA260 CHIPID:0x13
	LOG("chip id = %x\n",data[0]);
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, SPI_CONFIG, 0x24);			//	soft reset
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, RESOLUTION_RANGE, 0x00);	//	disable high pass filter, full scale:+/-2g
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, ODR_AXIS, 0x07);			//	enable X.Y.Z axis, ODR 125Hz
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, INT_CONFIG, 0x02);			//	select OD output for INT1
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, INT_LATCH, 0x0E);			//	INT1 latch 100ms
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, TAP_DUR, 0x01);			//	tap duration for second shock:100ms
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, TAP_THS, 0x14);			//	threshold of tap interrupt = 0x14*62.5mg = 1250mg
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, INT_MAP1, 0x20);			//	mapping single tap interrupt to INT1
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, INT_SET1, 0x20);			//	enable single tap interrupt
	LC_EEPROM_Write_One_Byte(GSENSOR_ID, MODE_BW, 0x50);			//	
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

