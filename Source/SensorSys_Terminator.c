/**************************************************************************************************
	Filename:       SensorSys.c
	Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
	Revision:       $Revision: 19453 $

	Description:    Generic Application (no Profile).


	Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

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
	PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

/*********************************************************************
	This application isn't intended to do anything useful, it is
	intended to be a simple example of an application's structure.

	This application sends "Hello World" to another "Generic"
	application every 15 seconds.  The application will also
	receive "Hello World" packets.

	The "Hello World" messages are sent/received as MSG type message.

	This applications doesn't have a profile, so it handles everything
	directly - itself.

	Key control:
		SW1:
		SW2:  initiates end device binding
		SW3:
		SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "AF.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "sapi.h"

#include "SensorSys_End.h"
#include "device.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
	#include "OnBoard.h"
#endif

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

// for SendDataRequest func
#include "NLMEDE.h"
#include "nwk_util.h"

#include "nwk.h"
#include "APS.h"
#include "ZDApp.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "ZComDef.h"
#include "hal_drivers.h"

/*********************************************************************
 * MACROS
 */
// Message ID's for application user messages must be in 0xE0-0xEF range
//#define ZB_USER_MSG                       0xE0
#define SYSCB_DATA_CNF   0xE0
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 sysSeqNumber = 0;    // 在端点间数据交流时被使用zb_SendDataRequest
uint8 AppTitle[]="SensorAPP"; //应用程序名称

// Sys 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
const cId_t Sys_ClusterList[SYS_MAX_CLUSTERS] =
{
  SYS_CLUSTERID
};

const cId_t Zb_ClusterList[ZB_MAX_CLUSTERS] =
{
  ZB_CLUSTERID
};

// Sys 端点简单描述符
const SimpleDescriptionFormat_t Sys_SimpleDesc =
{
	SYS_ENDPOINT,           //  int Endpoint;
	SYS_PROFID,                //  uint16 AppProfId[2];
	SYS_DEVICEID,              //  uint16 AppDeviceId[2];
	SYS_DEVICE_VERSION,        //  int   AppDevVer:4;
	SYS_FLAGS,                 //  int   AppFlags:4;
	SYS_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)Sys_ClusterList,  //  byte *pAppInClusterList;
	SYS_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)Sys_ClusterList   //  byte *pAppInClusterList;
};

const SimpleDescriptionFormat_t zb_SimpleDesc =
{
	ZB_ENDPOINT,           //  int Endpoint;
	SYS_PROFID,                //  uint16 AppProfId[2];
	SYS_DEVICEID,              //  uint16 AppDeviceId[2];
	SYS_DEVICE_VERSION,        //  int   AppDevVer:4;
	SYS_FLAGS,                 //  int   AppFlags:4;
	ZB_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)Zb_ClusterList,  //  byte *pAppInClusterList;
	ZB_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)Zb_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in Key_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t Sys_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 myAppState = APP_INIT;
uint8 keys_shift = 0;
uint8 type_join;		// Which type of EndPoint want to match

byte Sys_TaskID;

devStates_t Sys_NwkState;   // 节点现在的网络状态

byte Sys_TransID;  // 数据包的发送ID，每发一个自增1

// afAddrType_t Broadcast_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Sys_Init( byte task_id );

UINT16 Sys_ProcessEvent( byte task_id, UINT16 events );

void Sys_AllowBind ( uint8 timeout );

void Sys_MessageMSGCB( afIncomingMSGPacket_t *pckt, byte task_id );
void Sys_AllowBindConfirm( uint16 source );
void Sys_SendDataConfirm( uint8 handle, uint8 status );
void Sys_SendCback( uint8 event, uint8 status, uint16 data );
// void Sys_SendMessage( byte task_id );

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Sys_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Sys_Init( byte task_id )
{
  Sys_TaskID = task_id;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  // Broadcast_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  // Broadcast_DstAddr.endPoint = SYS_ENDPOINT;
  // Broadcast_DstAddr.addr.shortAddr = 0xFFFF;

  // Fill out the endpoint description.
  Sys_epDesc.endPoint = SYS_ENDPOINT;
  Sys_epDesc.task_id = &Sys_TaskID;
  Sys_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Sys_SimpleDesc;
  Sys_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &Sys_epDesc );

  type_join = 0;
  // Set device as Enddevice
  //zgDeviceLogicalType = ZG_DEVICETYPE_ENDDEVICE;

  // To Update the display...
  HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
  HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
  HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );
}

/*********************************************************************
 * @fn      Sys_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 Sys_ProcessEvent( byte task_id, UINT16 events )
{
	afIncomingMSGPacket_t *MSGpkt = NULL;
	afDataConfirm_t *afDataConfirm;

	// Data Confirmation message fields
	byte sentEP;
	ZStatus_t sentStatus;
	byte sentTransID;       // This should match the value sent

	if ( events & SYS_EVENT_MSG )
	{
		MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( task_id );
		while ( MSGpkt )
		{
			switch ( MSGpkt->hdr.event )
			{
				case AF_DATA_CONFIRM_CMD: // 判断信息是否发送成功
					// This message is received as a confirmation of a data packet sent.
					// The status is of ZStatus_t type [defined in ZComDef.h]
					// The message fields are defined in AF.h
					afDataConfirm = (afDataConfirm_t *)MSGpkt;
					sentEP = afDataConfirm->endpoint;
					sentStatus = afDataConfirm->hdr.status;
					sentTransID = afDataConfirm->transID;
					(void)sentEP;
					(void)sentTransID;

					// Action taken when confirmation is received.
					if ( sentStatus != ZSuccess )
					{
						// The data wasn't delivered -- Do something
					}
					break;

				case AF_INCOMING_MSG_CMD:   // 接收信息
					Sys_MessageMSGCB( MSGpkt, task_id );
					break;

				case ZDO_STATE_CHANGE:      // 设备网络状态改变
					Sys_NwkState = (devStates_t)(MSGpkt->hdr.status);
					if ( (Sys_NwkState == DEV_ROUTER)
							|| (Sys_NwkState == DEV_END_DEVICE) )
					{
						;
					}
					break;
                                case SYSCB_DATA_CNF:        // Data sent CoNFirm CallBack function
                                      Sys_SendDataConfirm( (uint8)((sys_CbackEvent_t *)MSGpkt)->data,
                                                                ((sys_CbackEvent_t *)MSGpkt)->hdr.status );
                                      break;
                  
				default:
					break;
			}

			// Release the memory
			osal_msg_deallocate( (uint8 *)MSGpkt );

			// Next
			MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Sys_TaskID );
		}
	  // return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}
	if(events & CLOSE_LED3_EVT)
	{
		HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);  //Close LED3
		return (events ^ CLOSE_LED3_EVT);
	}

	if(events & CLOSE_LIGHT_EVT)
	{
            HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
            HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
            HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );
	    return (events ^ CLOSE_LIGHT_EVT);
	}
	// Discard unknown events
	return 0;
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      Type2EP
 *
 * @brief   query for device type's corresponding endpoint number
 *
 * @param   type : which device type want to bind
 *
 * @return  Corresponding Endpoint number
 */
uint8 Type2EP(uint8 type)
{
    switch(type)
    {
        case KEY_TYPE_ID:
          {
                // 10~59
                if(keyCnt == KEY_NUM_MAX)
                {
                    return 0;     // error
                }
                keyCnt++;
                return (KEY_ENDPOINT+keyCnt-1);
          }
        case SWITCH_TYPE_ID:
          {
                // 60~109
                if(swCnt == SWITCH_NUM_MAX)
                {
                    return 0;     // error
                }
                swCnt++;
                return (SWITCH_ENDPOINT+swCnt-1);
          }
        case MOTOR_TYPE_ID:
          {
                // 110~119
                if(motorCnt == MOTOR_NUM_MAX)
                {
                  return 0;     // error
                }
                motorCnt++;
                return (MOTOR_ENDPOINT+motorCnt-1);
          }
    }
    return 0;
}


/*********************************************************************
 * @fn      Sys_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void Sys_MessageMSGCB( afIncomingMSGPacket_t *pkt, byte task_id )
{
	switch ( pkt->clusterId )
	{
		case SYS_CLUSTERID:
                {
			char flag[4];
			osal_memcpy( flag, pkt->cmd.Data, 4 );
			if( osal_memcmp( flag, "bind", 4) )
			{
				if( pkt->cmd.Data[4] & MY_DEVICE )
				{
                                        type_join = Type2EP(pkt->cmd.Data[4]);  // query for this type's corresponding endpoint number
                                        if(!type_join)
                                        {
                                            // faild to add endpoint
                                        }
                                        else
                                        {
                                            HalLedSet ( HAL_LED_3, HAL_LED_MODE_OFF );  // OPEN LED3
                                            osal_start_timerEx(Sys_TaskID, CLOSE_LED3_EVT, 5000);    // Close LED3 after 5s
       					    keys_shift = 1;
                                        }
				}
			}
#if defined( WIN32 )
			WPRINTSTR( pkt->cmd.Data );
#endif
			break;
                }
	}
}

/******************************************************************************
 * @fn          Sys_AllowBind
 *
 * @brief       The zb_AllowBind function puts the device into the
 *              Allow Binding Mode for a given period of time.  A peer device
 *              can establish a binding to a device in the Allow Binding Mode
 *              by calling zb_BindDevice with a destination address of NULL
 *
 * @param       timeout - The number of seconds to remain in the allow binding
 *                        mode.  Valid values range from 1 through 65.
 *                        If 0, the Allow Bind mode will be set false without TO
 *                        If greater than 64, the Allow Bind mode will be true
 *
 * @return      ZB_SUCCESS if the device entered the allow bind mode, else
 *              an error code.
 */

void Sys_AllowBind ( uint8 timeout )
{
  if ( timeout == 0 )
  {
    afSetMatch(type_join, FALSE);
  }
  else
  {
    afSetMatch(type_join, TRUE);
    if ( timeout != 0xFF )
    {
      if ( timeout > 64 )
      {
        timeout = 64;
      }
      HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );    // Close LED3
    }
  }
  return;
}

/******************************************************************************
 * @fn          Sys_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *              Shut Up allowbind after 1s.
 *
 * @param
 *
 * @return      none
 */
void Sys_AllowBindConfirm( uint16 source )
{
    type_join = 0;
    HalLedSet ( HAL_LED_3, HAL_LED_MODE_OFF );
    
    osal_start_timerEx(Sys_TaskID, CLOSE_LED3_EVT, 5000);    // Close LED3 after 3s
}

/*********************************************************************
 * @fn      Sys_SendMessage
 *
 * @brief   Send message.
 *
 * @param   none
 *
 * @return  none
 *//*
void Sys_SendMessage( void )
{
  afAddrType_t dstAddr;

  dstAddr.addr.shortAddr = 0x0000;      // Send to Coordinator
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  dstAddr.endPoint = Sys_epDesc.simpleDesc->EndPoint;
  
	char theMessageData[] = "";

	if ( AF_DataRequest( &dstAddr, &Sys_epDesc,
                                             SYS_CLUSTERID,
                                             (byte)osal_strlen( theMessageData ) + 1,
                                             (byte *)&theMessageData,
                                             &Sys_TransID,
                                             AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
	{
		// Successfully requested to be sent.
	}
	else
	{
		// Error occurred in request to send.
	}
}
*/
/******************************************************************************
 * @fn          Sys_SendDataRequest
 *
 * @brief       The Sys_SendDataRequest function initiates transmission of data
 *              to a peer device
 *
 * @param       destination - The destination of the data.  The destination can
 *                            be one of the following:
 *                            - 16-Bit short address of device [0-0xfffD]
 *                            - ZB_BROADCAST_ADDR sends the data to all devices
 *                              in the network.
 *                            - ZB_BINDING_ADDR sends the data to a previously
 *                              bound device.
 *
 *              commandId - The command ID to send with the message.  If the
 *                          SYS_BINDING_ADDR destination is used, this parameter
 *                          also indicates the binding to use.
 *
 *              len - The size of the pData buffer in bytes
 *              handle - A handle used to identify the send data request.
 *              txOptions - TRUE if requesting acknowledgement from the destination.
 *              radius - The max number of hops the packet can travel through
 *                       before it is dropped.
 *
 * @return      none
 */
void Sys_SendDataRequest ( uint16 destination, endPointDesc_t *epDesc, uint16 commandId, uint8 len,
                          uint8 *pData, uint8 handle, uint8 txOptions, uint8 radius )
{
  afStatus_t status;
  afAddrType_t dstAddr;

  txOptions |= AF_DISCV_ROUTE;

  // Set the destination address
  if (destination == SYS_BINDING_ADDR)
  {
    // Binding
    dstAddr.addrMode = afAddrNotPresent;
  } 
  else
  {
    // Use short address
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddr16Bit;

    if ( ADDR_NOT_BCAST != NLME_IsAddressBroadcast( destination ) )
    {
      txOptions &= ~AF_ACK_REQUEST;
    }
  }

  dstAddr.panId = 0;                                    // Not an inter-pan message.
  dstAddr.endPoint = epDesc->simpleDesc->EndPoint;  // Set the endpoint.

  
  // Send the message
  status = AF_DataRequest(&dstAddr, epDesc, commandId, len,
                          pData, &handle, txOptions, radius);

  if (status != afStatus_SUCCESS)
  {
    Sys_SendCback( SYSCB_DATA_CNF, status, handle );
  }
}

/*********************************************************************
 * CBack FUNCTIONS
 */

/*********************************************************************
 * @fn      SAPI_SendCback
 *
 * @brief   Sends a message to the sapi task ( itself ) so that a
 *           callback can be generated later.
 *
 * @return  none
 */
void Sys_SendCback( uint8 event, uint8 status, uint16 data )
{
  sapi_CbackEvent_t *pMsg;

  pMsg = (sapi_CbackEvent_t *)osal_msg_allocate( sizeof(sapi_CbackEvent_t) );
  if( pMsg )
  {
    pMsg->hdr.event = event;
    pMsg->hdr.status = status;
    pMsg->data = data;

    osal_msg_send( Sys_TaskID, (uint8 *)pMsg );
  }

}

/*********************************************************************
*********************************************************************/
/******************************************************************************
 * @fn          zb_StartConfirm
 *
 * @brief       The zb_StartConfirm callback is called by the ZigBee stack
 *              after a start request operation completes
 *
 * @param       status - The status of the start operation.  Status of
 *                       ZB_SUCCESS indicates the start operation completed
 *                       successfully.  Else the status is an error code.
 *
 * @return      none
 */
void zb_StartConfirm( uint8 status )
{
  if ( status == ZB_SUCCESS )
  {
        myAppState = APP_START;
  }
  else
  {
  }
}

/******************************************************************************
 * @fn          zb_FindDeviceConfirm
 *
 * @brief       The zb_FindDeviceConfirm callback function is called by the
 *              ZigBee stack when a find device operation completes.
 *
 * @param       searchType - The type of search that was performed.
 *              searchKey - Value that the search was executed on.
 *              result - The result of the search.
 *
 * @return      none
 */
void zb_FindDeviceConfirm( uint8 searchType, uint8 *searchKey, uint8 *result )
{
}

void zb_HandleOsalEvent( uint16 event )
{

}
/******************************************************************************
 * @fn          Sys_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void Sys_SendDataConfirm( uint8 handle, uint8 status )
{
}
// The order in this table must be identical to the task initialization calls below in osalInitTask.

const pTaskEventHandlerFn tasksArr[] = {
  macEventLoop,
  nwk_event_loop,
  Hal_ProcessEvent,
#if defined( MT_TASK )
  MT_ProcessEvent,
#endif
  APS_event_loop,
  ZDApp_event_loop,

  Sys_ProcessEvent,
  Key_ProcessEvent,
  Motor_ProcessEvent,
  Switch_ProcessEvent,
  SAPI_ProcessEvent
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;



/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks( void )
{
  uint8 taskID = 0;

  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

  macTaskInit( taskID++ );
  nwk_init( taskID++ );
  Hal_Init( taskID++ );
#if defined( MT_TASK )
  MT_TaskInit( taskID++ );
#endif
  APS_Init( taskID++ );
  ZDApp_Init( taskID++ );
  Sys_Init( taskID++ );
  Key_Init( taskID++ );
  Motor_Init( taskID++ );
  Switch_Init( taskID++ );
  SAPI_Init( taskID );
}