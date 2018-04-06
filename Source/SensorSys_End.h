/**************************************************************************************************
  Filename:       SensorSys.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Generic Application definitions.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/
#ifndef END_H
#define END_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>
#include "SensorSys_Tools.h"
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */
#define MY_DEVICE   BUTTON_TYPE_ID|MOTOR_TYPE_ID|SWITCH_TYPE_ID


#define APP_INIT                           0
#define APP_START                          1

#define SYS_PROFID             0x0F04
#define SYS_DEVICEID           0x0001
#define SYS_DEVICE_VERSION     0
#define SYS_FLAGS              0

// Send Message Timeout
#define SYS_SEND_MSG_TIMEOUT   5000     // Every 5 seconds

// Application Events (OSAL) - These are bit weighted definitions.
#define SYS_SEND_MSG_EVT       0x0001
#define MATCH_BIND_EVT         0x0002
#define CLOSE_LED3_EVT         0x0004
#define CLOSE_LIGHT_EVT        0x0008
#define CONFIG_OPTION_EVT      0x0010

#define ALLOW_BIND_TIMER       0x0040

// EndPoint MAX
#define BUTTON_NUM_MAX         3
#define MOTOR_NUM_MAX          2
#define SWITCH_NUM_MAX         5
#define OBSERVE_NUM_MAX        1
/*********************************************************************
 * MACROS
 */
extern byte Sys_TaskID;
extern byte Button_TaskID;
extern byte Motor_TaskID;
extern byte Switch_TaskID;

extern uint8 ZDAppTaskID;

extern endPointDesc_t Sys_epDesc;
extern endPointDesc_t Button_epDesc[BUTTON_NUM_MAX];
extern endPointDesc_t Motor_epDesc[MOTOR_NUM_MAX];
extern endPointDesc_t Switch_epDesc[SWITCH_NUM_MAX];

extern uint8 buttonCnt;
extern uint8 motorCnt;
extern uint8 swCnt;
/*********************************************************************
 * FUNCTIONS
 */

extern void Sys_AllowBind ( uint8 timeout );
extern void Sys_AllowBindConfirm( uint16 source );
/*
 * Task Initialization for the Generic Application
 */
extern void Sys_Init( byte task_id );
extern void Button_Init( byte task_id );
extern void Motor_Init( byte task_id );
extern void Switch_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 Sys_ProcessEvent( byte task_id, UINT16 events );
extern UINT16 Button_ProcessEvent( byte task_id, UINT16 events );
extern UINT16 Motor_ProcessEvent( byte task_id, UINT16 events );
extern UINT16 Switch_ProcessEvent( byte task_id, UINT16 events );

extern void osalAddTasks( void );

uint8 Type2EP(uint8 type);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSORSYS_H */
