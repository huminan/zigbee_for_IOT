/**************************************************************************************************
  Filename:       Sys.c
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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 AppTitle[]="SensorAPP"; //Ӧ�ó�������

// Sys �˵�Ĵ�ID
// This list should be filled with Application specific Cluster IDs.
const cId_t Sys_ClusterList[SYS_MAX_CLUSTERS] =
{
  SYS_CLUSTERID
};

// Button �˵�Ĵ�ID
// This list should be filled with Application specific Cluster IDs.
const cId_t Button_ClusterList[BUTTON_MAX_CLUSTERS] =
{
  BUTTON_CLUSTERID
};

// Sys �˵��������
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

// Button �˵��������
const SimpleDescriptionFormat_t Button_SimpleDesc =
{
	BUTTON_ENDPOINT,           //  int Endpoint;
	SYS_PROFID,                //  uint16 AppProfId[2];
	SYS_DEVICEID,              //  uint16 AppDeviceId[2];
	SYS_DEVICE_VERSION,        //  int   AppDevVer:4;
	SYS_FLAGS,                 //  int   AppFlags:4;
	BUTTON_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)Button_ClusterList,  //  byte *pAppInClusterList;
	BUTTON_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)Button_ClusterList   //  byte *pAppInClusterList;
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

byte Sys_TaskID;
byte Button_TaskID;   // Task ID for internal task/event processing
                        // This variable will be received when
                        // Button_Init() is called.

devStates_t Sys_NwkState;   // �ڵ����ڵ�����״̬


byte Sys_TransID;  // ���ݰ��ķ���ID��ÿ��һ������1

afAddrType_t Broadcast_DstAddr;
afAddrType_t Button_DstAddr;

static uint8 sysSeqNumber = 0;    // �ڶ˵�����ݽ���ʱ��ʹ��zb_SendDataRequest

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Sys_Init( byte task_id );
void Button_Init( byte task_id );

UINT16 Sys_ProcessEvent( byte task_id, UINT16 events );
void Sys_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg, byte task_id );
void Button_HandleKeys( byte shift, byte keys );
void Sys_MessageMSGCB( afIncomingMSGPacket_t *pckt, byte task_id );
void Sys_SendPreBindMessage( byte type_id );

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

  // To Update the display...
 
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
    if(task_id == Sys_TaskID)
    {
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( task_id );
      while ( MSGpkt )
      {
        switch ( MSGpkt->hdr.event )
        {
          case ZDO_CB_MSG:  // �յ����󶨽ڵ��rsp
            Sys_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt, task_id );
            break;

          case AF_DATA_CONFIRM_CMD:
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

          case AF_INCOMING_MSG_CMD:
            Sys_MessageMSGCB( MSGpkt, task_id );
            break;

          case ZDO_STATE_CHANGE:
            Sys_NwkState = (devStates_t)(MSGpkt->hdr.status);
            if ( (Sys_NwkState == DEV_ZB_COORD) )
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
    }

    if(task_id == Button_TaskID)
    {
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( task_id );
      while ( MSGpkt )
      {
        switch ( MSGpkt->hdr.event )
        {
            case KEY_CHANGE:
               Button_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
            break;
        }
        // Release the memory
        osal_msg_deallocate( (uint8 *)MSGpkt );

        // Next
        MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Button_TaskID );
      }
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

   if( events & MATCH_BIND_EVT ) // �㲥 match ��
   {
      if(task_id == Button_TaskID)
      {
         zb_BindDevice(TRUE, BUTTON_CMD_ID, NULL);
         HalLedSet(HAL_LED_2, HAL_LED_MODE_ON); // ���̵�
      }
      return (events ^ MATCH_BIND_EVT);
   }


  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      Sys_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void Sys_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg , byte task_id )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            switch (task_id)
            {
              case Button_TaskID:
              {
                Button_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
                Button_DstAddr.addr.shortAddr = pRsp->nwkAddr;
                // Take the first endpoint, Can be changed to search through endpoints
                Button_DstAddr.endPoint = pRsp->epList[0];

                HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
              }

              // case Led_TaskID:
            }
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

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
void Button_HandleKeys( byte shift, byte keys )
{
  zAddrType_t dstAddr;
  
  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
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
      Sys_SendPreBindMessage(BUTTON_TYPE_ID);
    }

    // ��Զ�̶˵㷢������
    if ( keys & HAL_KEY_SW_2 )
    {
      //( uint16 destination, uint16 commandId, uint8 len,
      //  uint8 *pData, uint8 handle, uint8 txOptions, uint8 radius )
      zb_SendDataRequest( 0xFFFE,  BUTTON_CMD_ID, 0,
                        (uint8 *)NULL, sysSeqNumber, 0, 0 );
      // 0xFFFE -- û��Ŀ�ĵ�ַ��dstaddr(Match �� addr)
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    // ������������û�ж�������ƥ���
    if ( keys & HAL_KEY_SW_4 )
    {/*
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        SYS_PROFID,
                        SYS_MAX_CLUSTERS, (cId_t *)Button_ClusterList,
                        SYS_MAX_CLUSTERS, (cId_t *)Button_ClusterList,
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
    case BUTTON_CLUSTERID:
      // "the" message
#if defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}

/*********************************************************************
 * @fn      Sys_SendPreBindMessage
 * * @brief   Broadcast prebind message.
 *
 *
 * @param   type_id : ��Ҫ�󶨵��ն�����
 *
 * @return  none
 */
void Sys_SendPreBindMessage( byte type_id )
{
  char theMessageData[5] = "bind";
  theMessageData[4] = type_id;

  if ( AF_DataRequest( &Boardcast_DstAddr, &Sys_epDesc,
                       SYS_CLUSTERID,
                       (byte)osal_strlen( theMessageData ) + 1,
                       (byte *)&theMessageData,
                       &Sys_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
   //if(��Ҫ��ʲô�Ͷ�Ӧ�� TypeID)
   if(type_id == BUTTON_TYPE_ID)
      osal_start_timerEx(Button_TaskID, MATCH_BIND_EVT, 5000);

  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
*********************************************************************/