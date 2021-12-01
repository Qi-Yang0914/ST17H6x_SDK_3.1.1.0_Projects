/**
*	@file	LC_Common.h
*	@date	09/17/2020
*	@version	1.0.0
*
*/

/*!
 * 	@defgroup	LC_Common
 *	@brief
 *	@{*/
 
#include "LC_Common.h"

lc_dev_sys_param	LC_Dev_System_Param	=	
{
	.dev_timeout_poweroff_cnt	=	LC_DEV_TIMER_DISCON_PWROFF,
	.dev_poweron_switch_flag	=	0,
	.dev_power_flag				=	0,
	.dev_lowpower_flag			=	0,
	.dev_ble_con_state			=	0,
	.dev_batt_value				=	0,

	.dev_charging_flag			=	0,
	.dev_batnoty_enable			=	0,
};
/*!
 *	@fn			clock_time_exceed_func
 *	@brief		
 */
uint32 clock_time_exceed_func(uint32 ref, uint32 span_ms)
{
#if 0
	u32 deltTick ,T0 ;
	T0 = hal_read_current_time();
	deltTick =TIME_DELTA(T0,ref);
	if(deltTick>span_ms){
		return 1 ;
	}else {
		return 0 ;
	}
#else 
	uint32 deltTick  = 0 ;
	deltTick = hal_ms_intv(ref) ;
	if(deltTick>span_ms){
		return 1 ;
	}else {
		return 0 ;
	}	
#endif
}
/*!
 *	@fn			LC_Common_ProcessOSALMsg
 *	@brief		Process an incoming task message,nothing.
 *	@param[in]	pMsg	:message to process
 *	@return		none.
 */
void LC_Common_ProcessOSALMsg(osal_event_hdr_t *pMsg)
{
	switch(pMsg->event)
	{
		default:
			// do nothing
		break;
	}
}


void LC_Timer_Start(void)
{
	// hal_timer_init(LC_RGB_Valeu_Deal);
	// hal_timer_set(AP_TIMER_ID_5, 100*1000);
	// hal_timer_set(AP_TIMER_ID_6, 5*1000);
//	LOG("Start timer:\n");
}
void LC_Timer_Stop(void)
{
	// hal_timer_stop(AP_TIMER_ID_5);
	// hal_timer_stop(AP_TIMER_ID_6);
//	LOG("Stop timer\n");
}

/** @}*/

