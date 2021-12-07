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

#ifndef	LC_UI_LED_BUZZER_H_
#define	LC_UI_LED_BUZZER_H_
/*------------------------------------------------------------------*/
/*						C++ guard macro								*/
/*------------------------------------------------------------------*/
#ifdef	__cplusplus
extern	"C"	{
#endif
/*------------------------------------------------------------------*/
/*						head files include 							*/
/*------------------------------------------------------------------*/
#include "LC_Common.h"
#include "LC_Key.h"
#include "LC_ADC.h"
/*------------------------------------------------------------------*/
/*						Pins definitions							*/
/*------------------------------------------------------------------*/
#define		MY_GPIO_LED_NO1		P1				//	blue LED
#define		MY_GPIO_LED_NO2		P31				//	green LED
#define		MY_GPIO_LED_NO3		GPIO_DUMMY
#define		MY_GPIO_PWM_NO1		GPIO_DUMMY
#define		MY_GPIO_PWM_NO2		GPIO_DUMMY
#define		MY_GPIO_PWM_NO3		GPIO_DUMMY

#define		BUZZER_FREQ			740
#define		BUZZER_DUTY			370

#define		GPIO_PWM_P33		P33
#define		GPIO_PWM_P34		P34

#define		PWM_MAX_G			(34-1)	//	16,000,000/34=470588	P33
#define		PWM_MAX_R			(33-1)	//	16,000,000/33=484848	P34

#define		PWM_G_TEST			(16)	//	duty=(16+1)/34=0.5		P33
#define		PWM_R_TEST			(16)	//	duty=(16+1)/33=0.51		P34

#define		PWM_BREATH_MAX		(255)	//	16,000,000/4/256	=	15,625Hz
/*------------------------------------------------------------------*/
/*						MACROS										*/
/*------------------------------------------------------------------*/
#define		LC_LED_BLUE_ON()	hal_gpio_write(MY_GPIO_LED_NO1, 0)
#define		LC_LED_BLUE_OFF()	hal_gpio_write(MY_GPIO_LED_NO1, 1)

#define		LC_LED_GREEN_ON()	hal_gpio_write(MY_GPIO_LED_NO2, 0)
#define		LC_LED_GREEN_OFF()	hal_gpio_write(MY_GPIO_LED_NO2, 1)
/*------------------------------------------------------------------*/
/*						UI Task Events definitions					*/
/*------------------------------------------------------------------*/
#define		UI_EVENT_LEVEL1		0x0001
#define		DEV_ENABLE_PWMOUT	0x0002
#define		CHARGE_BREATH_INIT	0x0004

/*------------------------------------------------------------------*/
/*						Data structures								*/
/*------------------------------------------------------------------*/
typedef struct
{
	uint16			offOn_Ms[2];
	uint8			offOn_cnt;
	uint8			next_mode;
}lc_ui_mode_para;

typedef struct
{
	lc_ui_mode_para *ui_type;
	uint8			cur_mode;
	uint8			cur_cnt;
	uint8			cur_state;
	uint32			next_wakeup_tick;
}lc_ui_run_para;
/*------------------------------------------------------------------*/
/*						external variables							*/
/*------------------------------------------------------------------*/
extern uint8	LC_Ui_Led_Buzzer_TaskID;
/*------------------------------------------------------------------*/
/*						User function prototypes					*/
/*------------------------------------------------------------------*/
void	LC_PWM_OUT_Switch					(uint8 onoff);
void	LC_Switch_Poweron					(uint8 cur_state, uint8 power_start_tick);
void 	LC_Dev_Poweroff						(void		);
void 	LC_UI_Led_Buzzer_Task_Init			(uint8 task_id);
uint16	LC_UI_Led_Buzzer_ProcessEvent		(uint8 task_id, uint16 events);


#ifdef	__cplusplus
}
#endif

#endif	/*	LC_UI_LED_BUZZER_H_	*/
/**	@}*/
