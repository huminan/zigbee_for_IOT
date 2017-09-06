
#include <stdio.h>
#include <string.h>

#ifndef COOR_H
#define COOR_H

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
#define MY_DEVICE   BUTTON_TYPE_ID


#define APP_INIT                           0
#define APP_START                          1

// These constants are only for example and should be changed to the
// device's needs

#define SYS_ENDPOINT           2
#define ZB_ENDPOINT            3
#define BUTTON_ENDPOINT        10

  
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

#define ALLOW_BIND_TIMER       0x0020


// Cluster IDs
#define SYS_MAX_CLUSTERS       1
#define BUTTON_MAX_CLUSTERS    2
#define ZB_MAX_CLUSTERS      4
  
#define SYS_CLUSTERID          0x0001
#define ZB_CLUSTERID         0x0002
#define BUTTON_CLUSTERID       0x0003
  
// Define the Command ID's used in this application
#define BUTTON_CMD_ID          1

// Terminator Type ID
#define BUTTON_TYPE_ID         0x01
#define MOTOR_TYPE_ID          0x02
  //...   0x04 0x08 0x10 ...

/*********************************************************************
 * MACROS
 */
extern byte Sys_TaskID;
extern byte Button_TaskID;
extern uint8 ZDAppTaskID;
extern endPointDesc_t Button_epDesc;

extern uint16 sensor_bindInProgress;
extern uint8 sysSeqNumber;
/*********************************************************************
 * FUNCTIONS
 */

extern void Sensor_AllowBind ( uint8 timeout );

/*
 * Task Initialization for the Generic Application
 */
extern void Sys_Init( byte task_id );
extern void Button_Init( byte task_id );
/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 Sys_ProcessEvent( byte task_id, UINT16 events );
extern UINT16 Button_ProcessEvent( byte task_id, UINT16 events );

extern void Sys_SendPreBindMessage( byte type_id );
extern void Sensor_BindDevice ( uint8 create, uint16 commandId, uint8 *pDestination );

extern void osalAddTasks( void );

#endif