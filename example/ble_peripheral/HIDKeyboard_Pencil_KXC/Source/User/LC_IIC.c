/**
 *	@file		LC_IIC.c
 *	@author		YQ
 *	@date		11/11/2020
 *	@version	1.0.0
 */

/*!
 *	@defgroup	LC_IIC
 *	@brief
 *	@{*/
 
/*------------------------------------------------------------------*/
/* 					 head files include		 						*/
/*------------------------------------------------------------------*/
#include "LC_IIC.h"

/*------------------------------------------------------------------*/
/* 					 IIC Drivers(simulated)			 				*/
/*------------------------------------------------------------------*/
static	inline	void	LC_IIC_Wait(void)
{
	;;;;;;;;;;;
}
static	inline	void	LC_IIC_Long_Wait(void)
{
	WaitUs(1);
}
static	inline	void	LC_IIC_Scl_Init(void)
{
	hal_gpio_fmux(LC_IIC_GPIO_SCL, Bit_DISABLE);
}
static	inline	void	LC_IIC_Scl_Idel(void)
{
	hal_gpio_pin_init(LC_IIC_GPIO_SCL, OEN);
	hal_gpio_write(LC_IIC_GPIO_SCL, 0);
}
static	inline	void	LC_IIC_Scl_Out(uint8 v)
{
	if(v){
		hal_gpio_pin_init(LC_IIC_GPIO_SCL, IE);
	}else{
		hal_gpio_pin_init(LC_IIC_GPIO_SCL, OEN);
	}
}
static	inline	void	LC_IIC_Sda_Init(void)
{
	hal_gpio_fmux(LC_IIC_GPIO_SDA, Bit_DISABLE);
	hal_gpio_pin_init(LC_IIC_GPIO_SDA, IE);
}
static	inline	void	LC_IIC_Sda_Idel(void)
{
	hal_gpio_pin_init(LC_IIC_GPIO_SDA, OEN);
	hal_gpio_write(LC_IIC_GPIO_SDA, 0);
}
static	inline	void	LC_IIC_Sda_Out(uint8 v)
{
	if(v){
		hal_gpio_pin_init(LC_IIC_GPIO_SDA, IE);
	}else{
		hal_gpio_pin_init(LC_IIC_GPIO_SDA, OEN);
	}
}
static	inline	uint8	LC_IIC_Sda_In(void)
{
	return	hal_gpio_read(LC_IIC_GPIO_SDA);
}

static	void	LC_IIC_Start(void)
{
	LC_IIC_Scl_Init();
	LC_IIC_Sda_Init();
	LC_IIC_Sda_Idel();
	LC_IIC_Scl_Idel();
	LC_IIC_Sda_Out(0);
	LC_IIC_Wait();
}
static	void	LC_IIC_Stop(void)
{
	LC_IIC_Sda_Out(0);
	LC_IIC_Wait();
	LC_IIC_Scl_Out(0);
	LC_IIC_Wait();
	LC_IIC_Scl_Out(1);
	LC_IIC_Wait();
	LC_IIC_Sda_Out(1);
}
static	uint8	LC_IIC_Wait_Ack(void)
{
	uint8	temp = 0;

	LC_IIC_Sda_Out(1);
	LC_IIC_Wait();
	LC_IIC_Scl_Out(1);
	LC_IIC_Wait();
	LC_IIC_Sda_In();
	LC_IIC_Wait();
	while(LC_IIC_Sda_In()){
		temp++;
		if(temp>254){
			LC_IIC_Stop();
			return	1;
		}
	}
	LC_IIC_Scl_Out(0);
	return 0;
}
static	void	LC_IIC_Write_Bit(int bit)
{
	LC_IIC_Scl_Out(0);
	LC_IIC_Sda_Out(bit);
	LC_IIC_Long_Wait();
	LC_IIC_Scl_Out(1);
}
static	int		LC_IIC_Read_Bit(void)
{
	LC_IIC_Write_Bit(1);
	return LC_IIC_Sda_In();
}
static	int		LC_IIC_Write_Byte(uint8 data)
{
	int	i = 0x80;
	while(i){
		LC_IIC_Write_Bit((data&i));
		i = i >> 1;
	}
	return LC_IIC_Read_Bit();
}
static	uint8	LC_IIC_Read_Byte(int last)
{
	uint8	dat =	0;
	for(uint8 i = 0;i<8;i++)
	{
		LC_IIC_Write_Bit(1);
		if(LC_IIC_Sda_In()){
			dat = (dat << 1) | 0x01;
		}else{
			dat = dat << 1;
		}
	}
	LC_IIC_Write_Bit(last);
	return dat;	
}
/*------------------------------------------------------------------*/
/* 					 PUBLIC FUNCTIONS		 						*/
/*------------------------------------------------------------------*/

/*!
 *	@fn			LC_Gpio_IIC_Init
 *	@brief		Initlialize the IIC pins.
 *	@param[in]	none.
 *	@return		none.
 */
void	LC_Gpio_IIC_Init(void)
{
	hal_gpio_pin_init(LC_IIC_GPIO_SDA, IE);
	hal_gpio_pull_set(LC_IIC_GPIO_SDA, STRONG_PULL_UP);

	hal_gpio_pin_init(LC_IIC_GPIO_SCL, IE);
	hal_gpio_pull_set(LC_IIC_GPIO_SCL, STRONG_PULL_UP);
}
/*!
 *	@fn			LC_EEPROM_Read_Byte_Sequence.
 *	@brief		iic read continously.
 *	@param[in]	id:		device ID.
 *	@param[in]	addr:	initial address.
 *	@param[in]	p:		data buffer.
 *	@param[in]	len:	length of reading.
 *	@return	none.
 */
void	LC_EEPROM_Read_Byte_Sequence(uint8 id, uint8 addr, uint8 *p,uint8 len)
{
	LC_IIC_Start();

	LC_IIC_Write_Byte(id);
	LC_IIC_Write_Byte(addr);
	LC_IIC_Stop();
	LC_IIC_Start();

	LC_IIC_Write_Byte(id |1);
	for(int k = 0;k<len;k++){
		*p++ = LC_IIC_Read_Byte(k == (len -1));
	}
	LC_IIC_Stop();
}
/*!
 *	@fn			LC_EEPROM_Write_Byte_Sequence.
 *	@brief		iic write continously.
 *	@param[in]	id:		device ID.
 *	@param[in]	addr:	initial address.
 *	@param[in]	*p:		data buffer.
 *	@param[in]	len:	length of writing.
 *	@return	none.
 */
void	LC_EEPROM_Write_Byte_Sequence(uint8 id, uint8 addr, uint8 *p, uint8 len)
{
	LC_IIC_Start();
	LC_IIC_Write_Byte(id);
	LC_IIC_Write_Byte(addr);
	for(int i = 0;i<len;i++){
		LC_IIC_Write_Byte(*p++);
	}
	LC_IIC_Stop();
}
/*!
 *	@fn			LC_EEPROM_Write_One_Byte.
 *	@brief		eeprom write one byte.
 *	@param[in]	id:		device ID.
 *	@param[in]	addr:	initial address.
 *	@param[in]	data:	.
 *	@return	none.
 */
void	LC_EEPROM_Write_One_Byte(uint8 id, uint8 addr, uint8 data)
{
	LC_IIC_Start();
	LC_IIC_Write_Byte(id);
	LC_IIC_Write_Byte(addr);
	LC_IIC_Write_Byte(data);
	LC_IIC_Stop();
}
/*!
 *	@fn			LC_EEPROM_Read_One_Byte.
 *	@brief		eeprom read one byte.
 *	@param[in]	id:		device ID.
 *	@param[in]	addr:	initial address.
 *	@return	data.
 */
uint8	LC_EEPROM_Read_One_Byte(uint8 id, uint8 addr)
{
	uint8 temp = 0;
	LC_EEPROM_Read_Byte_Sequence(id,addr,&temp,1);
	return temp;
}
 /** @}*/

