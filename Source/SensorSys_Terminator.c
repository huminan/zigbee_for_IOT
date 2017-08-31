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
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "sapi.h"

#include "SensorSys.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
	#include "OnBoard.h"
#endif

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
#define MY_DEVICE		BUTTON_TYPE_ID

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 AppTitle[]="SensorAPP"; //应用程序名称

// Sys 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
const cId_t Sys_ClusterList[SYS_MAX_CLUSTERS] =
{
  SYS_CLUSTERID
};

// Button 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
const cId_t Button_ClusterList[BUTTON_MAX_CLUSTERS] =
{
	BUTTON_CLUSTERID
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

// Button 端点简单描述符
const SimpleDescriptionFormat_t Button_SimpleDesc =
{
	BUTTON_ENDPOINT,           //  int Endpoint;
	SYS_PROFID,                //  uint16 AppProfId[2];
	SYS_DEVICEID,              //  uint16 AppDeviceId[2];
	SYS_DEVICE_VERSION,        //  int   AppDevVer:4;
	SYS_FLAGS,                 //  int   AppFlags:4;
	0,          			   //  byte  AppNumInClusters;
	NULL,  					   //  byte *pAppInClusterList;
	BUTTON_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)Button_ClusterList   //  byte *pAppInClusterList;
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
// filled-in in Button_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t Sys_epDesc;
endPointDesc_t Button_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 myAppState = APP_INIT;
static uint8 keys_shift = 0;

byte Sys_TaskID;
byte Button_TaskID;   // Task ID for internal task/event processing
										// This variable will be received when
										// Button_Init() is called.

devStates_t SensorSys_NwkState;   // 节点现在的网络状态


byte SensorSys_TransID;  // 数据包的发送ID，每发一个自增1

afAddrType_t Broadcast_DstAddr;
afAddrType_t Button_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Sys_Init( byte task_id );
void Button_Init( byte task_id );
void Button_HandleKeys( byte keys );
UINT16 Sys_ProcessEvent( byte task_id, UINT16 events );
void Sys_MessageMSGCB( afIncomingMSGPacket_t *pckt, byte task_id );
// void Sys_SendMessage( byte task_id );
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  );

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
	if ( myAppState == APP_INIT )
  {
  	uint8 startOptions;
 		uint8 logicalType;
    // In the init state, keys are used to indicate the logical mode.
    // The Switch device is always an end-device
    logicalType = ZG_DEVICETYPE_ENDDEVICE;
    zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);

    // Do more configuration if necessary and then restart device with auto-start bit set

    zb_ReadConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
    startOptions = ZCD_STARTOPT_AUTO_START;
    zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
    zb_SystemReset();
  }


  Sys_TaskID = task_id;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  Broadcast_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  Broadcast_DstAddr.endPoint = SYS_ENDPOINT;
  Broadcast_DstAddr.addr.shortAddr = 0xFFFF;

  // Fill out the endpoint description.
  Sys_epDesc.endPoint = SYS_ENDPOINT;
  Sys_epDesc.task_id = &Sys_TaskID;
  Sys_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Sys_SimpleDesc;
  Sys_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &Sys_epDesc );

  HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
  HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
  HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );
  HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );

  // To Update the display...
}

/*********************************************************************
 * @fn      Button_Init
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
void Button_Init( byte task_id )
{
	Button_TaskID = task_id;
	SensorSys_NwkState = DEV_INIT;    // added to main()
	SensorSys_TransID = 0;            // added to main()

	// Device hardware initialization can be added here or in main() (Zmain.c).
	// If the hardware is application specific - add it here.
	// If the hardware is other parts of the device add it in main().

	Button_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
	Button_DstAddr.endPoint = 0;
	Button_DstAddr.addr.shortAddr = 0;

	// Fill out the endpoint description.
	Button_epDesc.endPoint = BUTTON_ENDPOINT;
	Button_epDesc.task_id = &Button_TaskID;
	Button_epDesc.simpleDesc
						= (SimpleDescriptionFormat_t *)&Button_SimpleDesc;
	Button_epDesc.latencyReq = noLatencyReqs;

	// Register the endpoint description with the AF
	afRegister( &Button_epDesc );

	// Register for all key events - This app will handle all key events
	RegisterForKeys( Button_TaskID );
 
//	ZDO_RegisterForZDOMsg( Button_TaskID, End_Device_Bind_rsp );
	ZDO_RegisterForZDOMsg( Button_TaskID, Match_Desc_rsp );
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
	(void)task_id;  // Intentionally unreferenced parameter

	if ( events & SYS_EVENT_MSG )
	{
		MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( task_id );
		// 其它 task_id 同上
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
					if(myAppState == APP_INIT) {
						myAppState = APP_START;
					}

					SensorSys_NwkState = (devStates_t)(MSGpkt->hdr.status);
					if ( (SensorSys_NwkState == DEV_ZB_COORD)
							|| (SensorSys_NwkState == DEV_ROUTER)
							|| (SensorSys_NwkState == DEV_END_DEVICE) )
					{
						;
					}

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

	if(events & CLOSE_BIND_EVT)
	{
		HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
		return (events ^ CLOSE_BIND_EVT);
	}
	// Discard unknown events
	return 0;
}


UINT16 Button_ProcessEvent( byte task_id, UINT16 events )
{
	afIncomingMSGPacket_t *MSGpkt = NULL;

	(void)task_id;  // Intentionally unreferenced parameter

	if ( events & SYS_EVENT_MSG )
	{
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( task_id );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
      	case KEY_CHANGE:
      		Button_HandleKeys( ((keyChange_t *)MSGpkt)->keys );
      	break;
      }
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Button_TaskID );
    }
    return (events ^ SYS_EVENT_MSG);
	}
  return 0;
}


/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      Button_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void Button_HandleKeys( byte keys )
{
	zAddrType_t dstAddr;
	
	// Shift is used to make each button/switch dual purpose.
	if ( keys_shift )
	{
		// Allow Binding
		if ( keys & HAL_KEY_SW_1 )
		{
			zb_AllowBind(10);
			HalLedSet(HAL_LED_1, HAL_LED_MODE_FLASH);
			osal_start_timerEx(Sys_TaskID, CLOSE_BIND_EVT, 10000);
			
			keys_shift = 0;
		}
		if ( keys & HAL_KEY_SW_2 )
		{
		}
		if ( keys & HAL_KEY_SW_3 )
		{
		}
		if ( keys & HAL_KEY_SW_4 )
		{
		}
	}
	else
	{
		if ( keys & HAL_KEY_SW_1 )
		{
		}

		if ( keys & HAL_KEY_SW_2 )
		{/*
			HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

			// 想绑定另一个端点
			dstAddr.addrMode = Addr16Bit;
			dstAddr.addr.shortAddr = 0x0000; 
			ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(), 
														Button_epDesc.endPoint,
														SENSORSYS_PROFID,
														SENSORSYS_MAX_CLUSTERS, (cId_t *)Button_ClusterList,
														SENSORSYS_MAX_CLUSTERS, (cId_t *)Button_ClusterList,
														FALSE );*/
		}

		if ( keys & HAL_KEY_SW_3 )
		{
		}
		
		if ( keys & HAL_KEY_SW_4 )
		{
		/*	HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
			dstAddr.addrMode = AddrBroadcast;
			dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
			ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
												SENSORSYS_PROFID,
												SENSORSYS_MAX_CLUSTERS, (cId_t *)Button_ClusterList,
												SENSORSYS_MAX_CLUSTERS, (cId_t *)Button_ClusterList,
												FALSE );
		*/
		}
	}
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

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
			memcpy(flag, pkt->cmd.Data, 4);
			if( !strcmp( flag, "bind") )
			{
				if( pkt->cmd.Data[4] & MY_DEVICE )
				{
					HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);		// Set Red LED ON
					osal_start_timerEx(Sys_TaskID, CLOSE_BIND_EVT, 5000);		// Bind operation 5s timeout
					keys_shift = 1;
				}
			}
#if defined( WIN32 )
			WPRINTSTR( pkt->cmd.Data );
#endif
			break;
                }
	}
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
	char theMessageData[] = "";

	if ( AF_DataRequest( &Button_DstAddr, &Button_epDesc,
											 button_CLUSTERID,
											 (byte)osal_strlen( theMessageData ) + 1,
											 (byte *)&theMessageData,
											 &SensorSys_TransID,
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
 * @fn          zb_ReceiveDataIndication
 * 
 * @brief       The zb_ReceiveDataIndication callback function is called
 *              asynchronously by the ZigBee stack to notify the application
 *              when data is received from a peer device.
 *
 * @param       source - The short address of the peer device that sent the data
 *              command - The commandId associated with the data
 *              len - The number of bytes in the pData parameter
 *              pData - The data sent by the peer device
 *
 * @return      none
 */
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{
  if (command == BUTTON_CMD_ID)
  {
    // Received application command to toggle the LED
    HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK);
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
    // Try again later with a delay
    
    HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
  }
}
/******************************************************************************
 * @fn          zb_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
}
/******************************************************************************
 * @fn          zb_BindConfirm
 *
 * @brief       The zb_BindConfirm callback is called by the ZigBee stack
 *              after a bind operation completes.
 *
 * @param       commandId - The command ID of the binding being confirmed.
 *              status - The status of the bind operation.
 *
 * @return      none
 */
void zb_BindConfirm( uint16 commandId, uint8 status )
{
}
/******************************************************************************
 * @fn          zb_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void zb_AllowBindConfirm( uint16 source )
{
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