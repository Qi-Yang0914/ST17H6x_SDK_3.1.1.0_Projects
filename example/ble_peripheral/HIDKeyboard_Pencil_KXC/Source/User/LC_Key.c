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
/*------------------------------------------------------------------*/
/*						head files include 							*/
/*------------------------------------------------------------------*/
#include "LC_Key.h"
#include "LC_UI_led_buzzer.h"
#include "LC_Event_Handler.h"
#include "hidkbd.h"
#include "LC_IICDev.h"
/*------------------------------------------------------------------*/
/* 					 	local variables			 					*/
/*------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* 					 	public variables		 					*/
/*------------------------------------------------------------------*/
uint8 LC_Key_TaskID;
lc_key_struct_data LC_Key_Param = {
    .key_down_sys_tick = 0,
    .key_down_flag = 0,
    .key_repeated_num = 0,

};
/*------------------------------------------------------------------*/
/* 					 	local functions			 					*/
/*------------------------------------------------------------------*/
static void LC_KeyScanf(void)
{
    uint8 lc_keynewvalue = 0;
    static uint8 lc_keyoldvalue = 0;

    if(hal_gpio_read(MY_KEY_NO1_GPIO) == 0)
    {
        lc_keynewvalue = 1;
    }
	else if(hal_gpio_read(MY_KEY_NO2_GPIO) == 0)
	{
		lc_keynewvalue	=	2;
	}

    if (lc_keynewvalue)
    {
        if ((lc_keynewvalue == 0) || (lc_keyoldvalue != lc_keynewvalue))
        {
            lc_keyoldvalue = lc_keynewvalue;
            LC_Key_Param.key_down_flag = lc_keynewvalue;
            osal_start_timerEx(LC_Key_TaskID, KEY_EVENT_LEVEL1, 10);
            osal_stop_timerEx(LC_Key_TaskID, KEY_STOPSCANF_EVT);
        }
    }
    else
    {
        if (lc_keyoldvalue != 0)
        {
            lc_keyoldvalue = 0;
            LC_Key_Param.key_down_flag = 0;
            osal_start_timerEx(LC_Key_TaskID, KEY_EVENT_LEVEL1, 10);
            osal_start_timerEx(LC_Key_TaskID, KEY_STOPSCANF_EVT, 500);
        }
    }
}
/*------------------------------------------------------------------*/
/* 					 	public functions		 					*/
/*------------------------------------------------------------------*/
void Key_Pin_Config(void)
{
    hal_gpio_pin_init(MY_KEY_NO1_GPIO, IE);               //set gpio input
    hal_gpio_pull_set(MY_KEY_NO1_GPIO, STRONG_PULL_UP);   //pull up 150k

	hal_gpio_pin_init(MY_KEY_NO2_GPIO, IE);
	hal_gpio_pull_set(MY_KEY_NO2_GPIO, STRONG_PULL_UP);
	hal_gpioin_register(MY_KEY_NO2_GPIO, NULL, LC_Key_Pin_IntHandler);


	hal_gpio_pin_init(GPIO_USB_CHECK, IE);
	hal_gpio_pull_set(GPIO_USB_CHECK, PULL_DOWN);

	hal_gpio_pin_init(GPIO_CHGR_CHECK, IE);
	hal_gpio_pull_set(GPIO_CHGR_CHECK, STRONG_PULL_UP);

	hal_gpio_pin_init(GPIO_OUT_DCDC_EN, OEN);
	DCDC_DISABLE();
}

/*!
 *	@fn			LC_Key_Gpio_Init
 *	@brief		Initialize the key pins. 
 *	@param[in]	none.
 *	@return		none.
 */
void LC_Key_Gpio_Init(void)
{
    Key_Pin_Config();

    uint32 Power_Flash_Saved_Flag = g_system_reset_cause;
    LOG("Power_Flash_Saved_Flag = [%x]\n", Power_Flash_Saved_Flag);

    hal_pwrmgr_register(MOD_USR8, NULL, NULL);
    hal_pwrmgr_lock(MOD_USR8);

	if(hal_gpio_read(GPIO_USB_CHECK) == 1)
	{
		LC_Dev_System_Param.dev_poweron_switch_flag		=	1;
		LC_Dev_System_Param.dev_charging_flag			=	1;
		LC_Dev_System_Param.dev_timer_poweroff_flag		=	State_Off;
	}

    LC_Switch_Poweron(0, 30);
}
/*!
 *	@fn			LC_Key_Task_Init 
 *	@brief		Initialize function for the KEY Task. 
 *	@param[in]	task_id		: 	the ID assigned by OSAL,
 *								used to send message and set timer.
 *	@return	none.
 */
void LC_Key_Task_Init(uint8 task_id)
{
    LC_Key_TaskID = task_id;
	if(LC_Dev_System_Param.dev_power_flag == SYSTEM_WORKING)
	{
		LOG("power on led\n");
		osal_start_timerEx(LC_Ui_Led_Buzzer_TaskID, UI_EVENT_LEVEL1, 500);
		hal_gpioin_register(MY_KEY_NO1_GPIO, NULL, LC_Key_Pin_IntHandler);
	}

	if(hal_gpio_read(GPIO_USB_CHECK) == 1)
	{
		LOG("Start Charging Poweron\n");
		battPowerState	=	0xBB;
		osal_start_timerEx(LC_Ui_Led_Buzzer_TaskID, CHARGE_BREATH_INIT, 200);
		osal_start_timerEx(LC_Key_TaskID, KEY_CHARG_CHECK_EVT, 1000);
	}
	else
	{
		hal_gpioin_register(GPIO_USB_CHECK, LC_Key_Pin_IntHandler, NULL);
	}
	LC_SensorInit();
}
/*!
 *	@fn			LC_Key_ProcessEvent
 *	@brief		KEY Task event processor.This function
 *				is called to processs all events for the task.Events
 *				include timers,messages and any other user defined events.
 *	@param[in]	task_id			:The OSAL assigned task ID.
 *	@param[in]	events			:events to process.This is a bit map and can
 *									contain more than one event.
 */
uint16 LC_Key_ProcessEvent(uint8 task_id, uint16 events)
{
    VOID task_id; // OSAL required parameter that isn't used in this function
    if (events & SYS_EVENT_MSG)
    {
        uint8 *pMsg;
        if ((pMsg = osal_msg_receive(LC_Key_TaskID)) != NULL)
        {
            LC_Common_ProcessOSALMsg((osal_event_hdr_t *)pMsg);
            // Release the OSAL message
            VOID osal_msg_deallocate(pMsg);
        }
        return (events ^ SYS_EVENT_MSG);
    }
    if (events & KEY_EVENT_LEVEL1)
    {
        static	uint8	LC_last_button_pressed = 0;
        static	uint8	LC_last_button_numbale = 0;

        static	uint32	LC_last_button_press_time = 0;
        static	uint32	LC_last_button_release_time = 0;
        static	uint32	LC_key_time_temp	= 0;

		// static	uint8	Key_Press_Twice_Enable		=	0;
		static	uint8	Key_Value_Reserved			=	0;
        LC_key_time_temp = hal_systick() | 1;


        if (LC_Key_Param.key_down_flag)
        {
            if (!LC_last_button_pressed && clock_time_exceed_func(LC_last_button_release_time, 20))
            {
                LC_last_button_pressed = 1;
                LC_last_button_press_time = LC_key_time_temp;
                LC_last_button_numbale = LC_Key_Param.key_down_flag;
            }
        }
        else
        {
            if (LC_last_button_pressed && clock_time_exceed_func(LC_last_button_press_time, 20))
            {
                LC_last_button_release_time = LC_key_time_temp;
                LC_last_button_pressed = 0;
            }
        }
        if (LC_Key_Param.key_repeated_num && LC_Key_Param.key_down_sys_tick && clock_time_exceed_func(LC_Key_Param.key_down_sys_tick, 300))
        {
            LOG("Key total Kick num: %d\n", LC_Key_Param.key_repeated_num);

			if(LC_Dev_System_Param.dev_charging_flag == 0)
			{
				if(LC_Key_Param.key_repeated_num == 2)
				{
					if(Key_Value_Reserved == 1)
					{
						LC_Dev_System_Param.dev_power_flag	=	SYSTEM_STANDBY;
					}
					else if(Key_Value_Reserved == 2)
					{
						if(LC_Dev_System_Param.dev_ble_con_state == LC_DEV_BLE_CONNECTION)
						{
							LOG("keyboard E\n");
							LL_PLUS_DisableSlaveLatency(0);
							osal_stop_timerEx(hidKbdTaskId, HID_LATENCY_TURNOFF_EVT);
							hidKbdSendReport(0, HID_KEYBOARD_E);
							hidKbdSendReport(0, HID_KEYBOARD_RESERVED);
							osal_start_timerEx(hidKbdTaskId, HID_LATENCY_TURNOFF_EVT, 500);
						}
					}
				}
			}

            LC_Key_Param.key_down_sys_tick = 0;
            LC_Key_Param.key_repeated_num = 0;
            Key_Value_Reserved = 0;
        }
        if (LC_last_button_numbale && !LC_Key_Param.key_down_flag && clock_time_exceed_func(LC_last_button_press_time, 20))
        {
            LC_Key_Param.key_repeated_num++ ;
            LC_Key_Param.key_down_sys_tick = LC_key_time_temp;
			Key_Value_Reserved	=	LC_last_button_numbale;
            LOG("key time num: %d, key is%d\n", LC_Key_Param.key_repeated_num, LC_last_button_numbale);
            LC_last_button_numbale = 0;
        }
        if (LC_Key_Param.key_down_flag || LC_Key_Param.key_repeated_num)
        {
            osal_start_timerEx(LC_Key_TaskID, KEY_EVENT_LEVEL1, 20);
        }
        return (events ^ KEY_EVENT_LEVEL1);
    }

    if (events & KEY_SCANF_EVT)
    {
        osal_start_timerEx(LC_Key_TaskID, KEY_SCANF_EVT, 40);
        LC_KeyScanf();
        return (events ^ KEY_SCANF_EVT);
    }

    if (events & KEY_STOPSCANF_EVT)
    {
        LOG("stop scanf \n");
        hal_gpioin_register(MY_KEY_NO1_GPIO, NULL, LC_Key_Pin_IntHandler);
		hal_gpioin_register(MY_KEY_NO2_GPIO, NULL, LC_Key_Pin_IntHandler);
        osal_stop_timerEx(LC_Key_TaskID, KEY_SCANF_EVT);
        return (events ^ KEY_STOPSCANF_EVT);
    }

	if(events & KEY_CHARG_CHECK_EVT)
	{
		osal_start_reload_timer(LC_Key_TaskID, KEY_CHARG_CHECK_EVT, 500);
		if(hal_gpio_read(GPIO_CHGR_CHECK) == 1)
		{
			LOG("charge finish\n");
			LC_Timer_Stop();
			hal_pwm_stop();
			hal_gpio_pin_init(MY_GPIO_LED_NO2, OEN);
			LC_LED_GREEN_ON();
			LC_Dev_System_Param.dev_charge_full	=	1;
			if(battLevel < 100)
			{
				battLevel	+=	1;
			}
			else
			{
				battLevel	=	100;
			}
		}

		if(hal_gpio_read(GPIO_USB_CHECK) == 0)
		{
			LOG("charge stop\n");
			LC_Dev_System_Param.dev_power_flag	=	SYSTEM_STANDBY;
			osal_start_timerEx(LC_Ui_Led_Buzzer_TaskID, UI_EVENT_LEVEL1, 200);
			osal_stop_timerEx(LC_Key_TaskID, KEY_CHARG_CHECK_EVT);
		}
		return(events ^ KEY_CHARG_CHECK_EVT);
	}
    // Discard unknown events
    return 0;
}

/** @}*/
