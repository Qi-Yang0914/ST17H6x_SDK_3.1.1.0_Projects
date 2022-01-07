/**
*	@file	LC_UI_led_buzzer.h
*	@date	09/16/2020
*	@version	1.0.0
*
*/

/*!
 * 	@defgroup	LC_UI_led_buzzer
 *	@brief
 *	@{*/
 
#include "LC_UI_Led_Buzzer.h"
#include "LC_Event_Handler.h"
#include "LC_IICDev.h"

/*------------------------------------------------------------------*/
/* 					 	public variables		 					*/
/*------------------------------------------------------------------*/
uint8	LC_Ui_Led_Buzzer_TaskID;
lc_ui_mode_para	Led_No1_Mode_Ms[]	=	{
	//off			on		cnt 	next mode
	{{0,			0}, 	0,		0},
	{{0, 			4500},	0x01,	2},
	{{1000,			1300},	0xff,	2}, 
	{{0,			200},	0x01,	0},
	{{0,			6000},	0x01,	0},
	{{1000,			60},	0x28,	0},
	{{180,			60},	0xff,	0},
};
lc_ui_mode_para	Led_No2_Mode_Ms[]	=	{
	//off			on		cnt 	next mode
	{{0,			0}, 	0,		0},
	{{900, 			100},	0x05,	0},
	{{0,			200},	0x19,	0}, 
	{{0,			100},	0x01,	0},
};
lc_ui_mode_para	Led_No3_Mode_Ms[]	=	{
	//off			on		cnt 	next mode
	{{0,			0}, 	0,		0},
	{{1900, 		100},	0xff,	1},
	{{0,			200},	0x19,	0}, 
	{{0,			100},	0x01,	0},
};
lc_ui_mode_para	Buzzer_Mode_Ms[]	=	{
	//off			on		cnt 	next mode
	{{0,			0}, 	0,		0},    
	{{400,			100},	0x02,	0}, 
	{{0,			50},	0x01,	0}, 	
	{{300,			100},	0xff,	3}, 
};
lc_ui_run_para	Led_No1_Param	=	{Led_No1_Mode_Ms};
lc_ui_run_para	Led_No2_Param	=	{Led_No2_Mode_Ms};
lc_ui_run_para	Led_No3_Param	=	{Led_No3_Mode_Ms};
lc_ui_run_para	Buzzer_Param	=	{Buzzer_Mode_Ms};
uint8	Breath_Duty				=	255;
uint8	Breath_Increase_Flag	=	0;
/*------------------------------------------------------------------*/
/* 					 	local functions			 					*/
/*------------------------------------------------------------------*/
/**
 * @brief	Initilize PWM pins.
 * 
 */
static	void	LC_PWM_Pin_Init(void)
{
	hal_gpio_pin_init(GPIO_PWM_P33, OEN);
	hal_gpio_write(GPIO_PWM_P33, 0);

	hal_gpio_pin_init(GPIO_PWM_P34, OEN);
	hal_gpio_write(GPIO_PWM_P34, 0);
}
/**
 * @brief 	Initlize PWM channel.No output before start PWM.
 * 
 */
static	void	LC_PWMChannelInit(void)
{
	hal_pwm_init(PWM_CH0, PWM_CLK_NO_DIV, PWM_CNT_UP, PWM_POLARITY_FALLING);
	hal_pwm_open_channel(PWM_CH0, GPIO_PWM_P33);

	hal_pwm_init(PWM_CH1, PWM_CLK_NO_DIV, PWM_CNT_UP, PWM_POLARITY_FALLING);
	hal_pwm_open_channel(PWM_CH1, GPIO_PWM_P34);

	hal_pwm_init(PWM_CH2, PWM_CLK_NO_DIV, PWM_CNT_UP, PWM_POLARITY_FALLING);
	hal_pwm_open_channel(PWM_CH2, MY_GPIO_LED_NO2);
}
/**
 * @brief 	Set RGB value to PWM register.
 * 
 */
static	void	LC_PWMSetRGBValue(void)
{
	hal_pwm_set_count_val(PWM_CH0, PWM_G_TEST, PWM_MAX_G);
	hal_pwm_set_count_val(PWM_CH1, PWM_R_TEST, PWM_MAX_R);
	hal_pwm_set_count_val(PWM_CH2, PWM_BREATH_MAX, PWM_BREATH_MAX);
}
static	void	LC_LedPWMChannelInit(void)
{
	Breath_Increase_Flag		=	0;
	Breath_Duty					=	PWM_BREATH_MAX;

	hal_gpio_pin_init(MY_GPIO_LED_NO2, OEN);
	hal_gpio_write(MY_GPIO_LED_NO2, 1);
	hal_pwm_init(PWM_CH2, PWM_CLK_NO_DIV, PWM_CNT_UP, PWM_POLARITY_FALLING);
	hal_pwm_open_channel(PWM_CH2, MY_GPIO_LED_NO2);
	hal_pwm_set_count_val(PWM_CH2, PWM_BREATH_MAX, PWM_BREATH_MAX);
	hal_pwm_start();
}
static	void	Led_ChargeBreath(void)
{
	{
		if(Breath_Increase_Flag == 0)
		{
			Breath_Duty--;
			if(Breath_Duty == 0)
			{
				Breath_Increase_Flag	=	1;
			}
		}
		else
		{
			Breath_Duty++;
			if(Breath_Duty == PWM_BREATH_MAX)
			{
				Breath_Increase_Flag	=	0;
			}
		}
		// LOG("breath %d\n",Breath_Duty);
		hal_pwm_set_count_val(PWM_CH2, Breath_Duty, PWM_BREATH_MAX);
	}
}
static	void	LC_Working_Timer(void)
{
	if(LC_Dev_System_Param.dev_timer_poweroff_flag == State_On)
	{
		LC_Dev_System_Param.dev_timeout_poweroff_cnt--;
		LOG("system tiemr WORKING = [%d]*[%d]s\n",LC_Dev_System_Param.dev_timeout_poweroff_cnt,LC_TIMER_INTERVAL);
		if(LC_Dev_System_Param.dev_timeout_poweroff_cnt == 0)
		{
			LC_Dev_System_Param.dev_power_flag	=	SYSTEM_STANDBY;
		}
	}
}

void	LC_PWM_OUT_Switch(uint8 onoff)
{
	if(onoff == State_On)
	{
		LC_PWM_Pin_Init();
		LC_PWMChannelInit();
		LC_PWMSetRGBValue();
		hal_pwm_start();
	}
	else
	{		
		// hal_pwm_module_deinit();
		hal_pwm_close_channel(PWM_CH0);
		hal_pwm_close_channel(PWM_CH1);
		LC_PWM_Pin_Init();
		// hal_pwm_stop();
	}
}
/*!
 *	@fn			LC_UI_Led_Buzzer_Gpio_Init
 *	@brief		Initialize the LED and Buzzer pins. 
 *	@param[in]	none.
 *	@return		none.
 */
void	LC_UI_Led_Buzzer_Gpio_Init(void)
{
	hal_gpio_pin_init(MY_GPIO_LED_NO1, OEN);
	LC_LED_BLUE_OFF();

	hal_gpio_pin_init(MY_GPIO_LED_NO2, OEN);
	LC_LED_GREEN_OFF();
}
/*!
 *	@fn			LC_Switch_Poweron
 *	@brief		press switch to power on.
 *	@param[in]	cur_state	:
 *	@param[in]	power_start_tick	:set time for long press to poweron,
 *									power_start_tick*25ms
 *	@return		none.
 */
void LC_Switch_Poweron(uint8 cur_state, uint8 power_start_tick)
{
	if(LC_Dev_System_Param.dev_poweron_switch_flag)
	{
		LC_Dev_System_Param.dev_power_flag		=	SYSTEM_WORKING;
		return;
	}
	uint8	poweron_start_num	=	power_start_tick;

	if(!cur_state)
	{
		while(poweron_start_num)
		{
			WaitMs(10);
			if(hal_gpio_read(MY_KEY_NO1_GPIO) == 0)
			{
				poweron_start_num--;
				LOG("press first %d\n", poweron_start_num);
				if(poweron_start_num == 0)
				{
					LOG("long once press\n");
					LC_Dev_System_Param.dev_power_flag	=	SYSTEM_STANDBY;
					LC_Dev_Poweroff();
					return;
				}
			}
			else
			{
				if(poweron_start_num!=power_start_tick)
				{	
					poweron_start_num	=	0;
					LOG("release first\n");
				}
				else
				{
					LC_Dev_System_Param.dev_power_flag	=	SYSTEM_STANDBY;
					LC_Dev_Poweroff();
					return;
				}
			}
		}
		WaitMs(5);
		if(hal_gpio_read(MY_KEY_NO1_GPIO) != 0)
		{
			if(poweron_start_num != power_start_tick)
			{
				poweron_start_num	=	30;
				while(poweron_start_num)
				{
					WaitMs(10);
					poweron_start_num--;
					LOG("check second press %d\n", poweron_start_num);
					if(hal_gpio_read(MY_KEY_NO1_GPIO) == 0)
					{
						LOG("press second %d\n",poweron_start_num);
						poweron_start_num	=	0;
						// WaitMs(10);
						// while(hal_gpio_read(MY_KEY_NO1_GPIO) == 0)	;
						LC_Dev_System_Param.dev_power_flag		=	SYSTEM_WORKING;
						return;
					}
				}
			}
			poweron_start_num	=	power_start_tick;
			LC_Dev_System_Param.dev_power_flag		=	SYSTEM_STANDBY;
			LC_Dev_Poweroff();
		}
	}
}
/*!
 *	@fn			LC_Dev_Poweroff
 *	@brief		the process of power off,need to disable adv and all events.
 *	@param[in]	none.
 *	@return		none.
 */
void LC_Dev_Poweroff(void)
{
	LOG("POWER OFF[%d]\n", LC_Dev_System_Param.dev_power_flag);
	LC_SensorSuspend();
	hal_gpio_pin_init(MY_KEY_NO2_GPIO, IE);
	hal_gpio_pull_set(MY_KEY_NO2_GPIO, STRONG_PULL_UP);
	hal_gpio_pin_init(GPIO_OUT_DCDC_EN, IE);
	hal_gpio_pull_set(GPIO_OUT_DCDC_EN, GPIO_PULL_UP_S);

	hal_gpio_pin_init(MY_GPIO_LED_NO1, IE);
	hal_gpio_pin_init(MY_GPIO_LED_NO2, IE);
	hal_gpio_pull_set(MY_GPIO_LED_NO1, GPIO_PULL_UP_S);
	hal_gpio_pull_set(MY_GPIO_LED_NO2, GPIO_PULL_UP_S);

	pwroff_cfg_t	User_Set_Wakeup[2];
	User_Set_Wakeup[0].pin	=	MY_KEY_NO1_GPIO;
	User_Set_Wakeup[0].type	=	NEGEDGE;
	User_Set_Wakeup[0].on_time	=	0;

	User_Set_Wakeup[1].pin	=	GPIO_USB_CHECK;
	User_Set_Wakeup[1].type	=	POSEDGE;
	User_Set_Wakeup[1].on_time	=	5;
	hal_pwrmgr_unlock(MOD_USR8);

	AP_WDT->CRR	=	0x76;	//	feed watch dog
	while(hal_gpio_read(MY_KEY_NO1_GPIO) == 0){
		WaitUs(10*1000);
		AP_WDT->CRR	=	0x76;	//	feed watch dog
	}
	hal_pwrmgr_poweroff(&User_Set_Wakeup[0], 2);
}

/*!
 *	@fn			LC_UI_Led_Buzzer_Task_Init 
 *	@brief		Initialize function for the UI_LED_BUZZER Task. 
 *	@param[in]	task_id			:the ID assigned by OSAL,
 *								used to send message and set timer.
 *	@retrurn	none.
 */
void LC_UI_Led_Buzzer_Task_Init(uint8 task_id)
{
	LC_Ui_Led_Buzzer_TaskID	=	task_id;
	LC_UI_Led_Buzzer_Gpio_Init();
}
/*!
 *	@fn			LC_UI_Led_Buzzer_ProcessEvent
 *	@brief		UI_LED_BUZZER Task event processor.This function
 *				is called to processs all events for the task.Events
 *				include timers,messages and any other user defined events.
 *	@param[in]	task_id			:The OSAL assigned task ID.
 *	@param[in]	events			:events to process.This is a bit map and can
 *									contain more than one event.
 */
uint16	LC_UI_Led_Buzzer_ProcessEvent(uint8 task_id, uint16 events)
{
	VOID task_id;	// OSAL required parameter that isn't used in this function
	static	uint8	disconnection_led_flag	=	0;
	if(events & SYS_EVENT_MSG)
	{
		uint8	*pMsg;
		if((pMsg = osal_msg_receive(LC_Ui_Led_Buzzer_TaskID)) != NULL)
		{
			LC_Common_ProcessOSALMsg((osal_event_hdr_t *)pMsg);
            // Release the OSAL message
			VOID osal_msg_deallocate(pMsg);
		}
		return(events ^ SYS_EVENT_MSG);
	}

	if(events & UI_EVENT_LEVEL1)
	{
		if(LC_Dev_System_Param.dev_power_flag == SYSTEM_WORKING)
		{
			if(LC_Dev_System_Param.dev_ble_con_state == LC_DEV_BLE_DISCONNECTION)
			{
				if(LC_Dev_System_Param.dev_charging_flag == 0)
				{
					//	disconnection led action
					if(disconnection_led_flag == 0)
					{
						LC_LED_BLUE_ON();
						disconnection_led_flag	=	1;
					}
					else
					{
						LC_LED_BLUE_OFF();
						disconnection_led_flag	=	0;
					}
				}
			}
			else 
			{
				battNotifyLevel();
			}

			osal_start_timerEx(LC_Ui_Led_Buzzer_TaskID, UI_EVENT_LEVEL1, 1*1000);
			LC_Working_Timer();
		}
		else if(LC_Dev_System_Param.dev_power_flag == SYSTEM_STANDBY)
		{
			LC_Dev_Poweroff();
		}
		return(events ^ UI_EVENT_LEVEL1);
	}


	if(events & DEV_ENABLE_PWMOUT)
	{
		Led_ChargeBreath();
		return(events ^ DEV_ENABLE_PWMOUT);
	}

	if(events & CHARGE_BREATH_INIT)
	{
		LC_Timer_Start();
		LC_LedPWMChannelInit();
		if(LC_Dev_System_Param.dev_ble_con_state == LC_DEV_BLE_CONNECTION)
		{
			LOG("charging disable PWM output\n");
			DCDC_DISABLE();
			LC_PWM_OUT_Switch(State_Off);
		}
		LC_LED_BLUE_OFF();
		return(events ^ CHARGE_BREATH_INIT);
	}

    // Discard unknown events
    return 0;
}
/** @}*/

