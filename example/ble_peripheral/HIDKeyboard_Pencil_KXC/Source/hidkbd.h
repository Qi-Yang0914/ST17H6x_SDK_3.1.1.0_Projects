/**************************************************************************************************
*******
**************************************************************************************************/

/******************************************************************************

 *****************************************************************************/

#ifndef HIDEMUKBD_H
#define HIDEMUKBD_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Task Events
#define START_DEVICE_EVT								0x0001
// #define	HID_BAT_CHECK_EVT								0x0002
// #define	HID_BAT_UPDATE_EVT								0x0004
#define HID_LATENCY_TURNOFF_EVT							0x0008
/*********************************************************************
 * MACROS
 */


typedef struct 
{
	uint8 len;
	uint8 name_str[16];
}lc_selfie_name_info;

/*********************************************************************
 * FUNCTIONS
 */
  
/*********************************************************************
 * GLOBAL VARIABLES
 */

extern uint8 hidKbdTaskId;
extern uint8 g_instant_cnt;

/*
 * Task Initialization for the BLE Application
 */
extern void HidKbd_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern	uint16	HidKbd_ProcessEvent				( uint8 task_id, uint16 events );
extern	void	hidKbdSendReport				(uint8 combine_enabel, uint8 keycode );
extern	void 	hidCCSendReport					( uint8 cmd, bool keyPressed, uint8 keyRepeated );
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /*HIDEMUKBD_H */
