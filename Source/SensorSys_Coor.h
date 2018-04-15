
#include <stdio.h>
#include <string.h>

#ifndef COOR_H
#define COOR_H      TRUE

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

#define MY_DEVICE   KEY_TYPE_ID


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
#define CLOSE_BIND_EVT         0x0004
#define CLOSE_LIGHT_EVT        0x0008
#define CONFIG_OPTION_EVT      0x0010
#define MY_START_EVT           0x0011    // I dont kown whether it is here

#define KEY_BIND_TIMER      0x0021
#define MOTOR_BIND_TIMER       0x0021
#define SWITCH_BIND_TIMER         0x0021
#define OBSERVE_BIND_TIMER     0x0021
  
// EndPoint MAX
#define KEY_NUM_MAX         50
#define MOTOR_NUM_MAX          10   // can be expand to 20
#define SWITCH_NUM_MAX            20   // should be 50
#define OBSERVE_NUM_MAX        40
  
  
// SAPI SendDataRequest destinations
#define SYS_BINDING_ADDR                   INVALID_NODE_ADDR
#define SYS_BROADCAST_ADDR                 0xffff
  
/*********************************************************************
 * STRUCTS
 */
  
typedef struct
{
  osal_event_hdr_t hdr;
  uint16 data;
} sys_CbackEvent_t;  

/*********************************************************************
 * MACROS
 */
extern byte Sys_TaskID;
extern byte Key_TaskID;
extern byte Motor_TaskID;
extern byte Switch_TaskID;

extern uint8 ZDAppTaskID;
extern endPointDesc_t Sys_epDesc;
extern endPointDesc_t *Key_epDesc[KEY_NUM_MAX];
extern endPointDesc_t Motor_epDesc[MOTOR_NUM_MAX];
extern endPointDesc_t Switch_epDesc[SWITCH_NUM_MAX];

extern uint8 sysSeqNumber;

extern uint8 keyCnt;
extern uint8 motorCnt;
extern uint8 swCnt;
/*********************************************************************
 * FUNCTIONS
 */

extern void Sensor_AllowBind ( uint8 timeout );

/*
 * Task Initialization for the Generic Application
 */
extern void Sys_Init( byte task_id );
extern void Key_Init( byte task_id );
extern void Motor_Init( byte task_id );
extern void Switch_Init( byte task_id );
/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 Sys_ProcessEvent( byte task_id, UINT16 events );
extern UINT16 Key_ProcessEvent( byte task_id, UINT16 events );
extern UINT16 Motor_ProcessEvent( byte task_id, UINT16 events );
extern UINT16 Switch_ProcessEvent( byte task_id, UINT16 events );


extern void Sys_SendPreBindMessage( byte type_id);
extern void Sys_SendDataRequest ( uint16 destination, endPointDesc_t *epDesc, uint16 commandId, uint8 len,
                          uint8 *pData, uint8 handle, uint8 ack, uint8 radius );
// extern void Key_BindDevice ( uint8 create, uint16 commandId, uint8 *pDestination );

extern void osalAddTasks( void );
extern void Sys_BindConfirm( uint16 commandId, uint8 status );

#endif