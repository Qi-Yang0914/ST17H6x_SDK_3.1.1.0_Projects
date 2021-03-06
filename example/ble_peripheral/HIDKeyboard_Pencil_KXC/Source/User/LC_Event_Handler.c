/**
 *	@file		LC_Event_Handler.c
 *	@author		YQ
 *	@date		01/20/2021
 *	@brief		IRQ handler.
 */

/*!
 *	@defgroup	LC_Event_Handler
 *	@brief		DO NOT put Interrupt code in XIP flash!!!
 *	@{*/

/*------------------------------------------------------------------*/
/*						head files include 							*/
/*------------------------------------------------------------------*/
#include	"LC_Event_Handler.h"
#include	"LC_UI_led_buzzer.h"
#include	"LC_Key.h"

/*------------------------------------------------------------------*/
/* 					 	local variables			 					*/
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* 					 	public variables		 					*/
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* 					 	public functions		 					*/
/*------------------------------------------------------------------*/
/**
 *	@fn			LC_RGB_Valeu_Deal
 *	@brief		Callback of timer handler.
 *	@param[in]	evt		:IRQ event.
 *	@return		none.
 */
void	__ATTR_SECTION_SRAM__  __attribute__((used))	LC_RGB_Valeu_Deal(uint8 evt)
{
	if(evt == HAL_EVT_TIMER_5)
	{
		osal_set_event(LC_Ui_Led_Buzzer_TaskID, DEV_ENABLE_PWMOUT);
	}
}
/**
 *	@fn			LC_IR_Analysis_Data
 *	@brief		analysis NEC,get user code and commond code.
 *	@param[in]	nTimeL		:interval of a negtivate and a posetive edge.	
 *	@return		none.
 */
void	__ATTR_SECTION_SRAM__  __attribute__((used))	LC_IR_Analysis_Data(uint32	nTimeL)
{

}
/*!
 *	@fn			LC_Key_Pin_IntHandler
 *	@brief		Callback of key Pin interrupt.
 *	@param[in]	pin		:pin of IR.
 *	@param[in]	type	:type of interrupe.
 *	@return		none.
 */
void	__ATTR_SECTION_SRAM__  __attribute__((used))	LC_Key_Pin_IntHandler(GPIO_Pin_e pin, IO_Wakeup_Pol_e type)
{
    switch (pin)
    {
		case MY_KEY_NO1_GPIO:
			if (type == NEGEDGE)
			{
				LOG("KEY interrupt\n");
				hal_gpioin_register(MY_KEY_NO1_GPIO, NULL, NULL);
				osal_start_timerEx(LC_Key_TaskID, KEY_SCANF_EVT, 20);
			}
			break;

		case MY_KEY_NO2_GPIO:
			if(type == NEGEDGE)
			{
				hal_gpioin_register(MY_KEY_NO2_GPIO, NULL, NULL);
				osal_start_timerEx(LC_Key_TaskID, KEY_SCANF_EVT, 20);
			}
			break;
			
		case	GPIO_USB_CHECK:
			if(type == POSEDGE)
			{
				LOG("Start Charging\n");
				hal_gpioin_register(GPIO_USB_CHECK, NULL, NULL);
				LC_Dev_System_Param.dev_charging_flag		=	1;
				LC_Dev_System_Param.dev_timer_poweroff_flag	=	State_Off;
				battPowerState	=	0xBB;
				osal_start_timerEx(LC_Ui_Led_Buzzer_TaskID, CHARGE_BREATH_INIT, 1000);
				osal_start_timerEx(LC_Key_TaskID, KEY_CHARG_CHECK_EVT, 1000);
			}
			break;
		default:

			break;
    }
}
/*!
 *	@fn			LC_Gpio_IR_IntHandler
 *	@brief		Callback of IR Pin interrupt.
 *	@param[in]	pin		:pin of IR.
 *	@param[in]	type	:type of interrupe.
 *	@return		none.
 */
void	__ATTR_SECTION_SRAM__  __attribute__((used))	LC_Gpio_IR_IntHandler(GPIO_Pin_e pin, IO_Wakeup_Pol_e type)
{

}

/** @}*/

