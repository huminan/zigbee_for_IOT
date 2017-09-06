/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "sapi.h"

#include "SensorSys_Coor.h"
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

// Button 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
const cId_t Button_ClusterList[BUTTON_MAX_CLUSTERS] =
{
  BUTTON_CLUSTERID
};

// Button 端点简单描述符
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

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 myAppState;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte Button_TaskID;
endPointDesc_t Button_epDesc;

afAddrType_t Button_DstAddr;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Button_Init( byte task_id );
void Button_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void Button_HandleKeys( byte shift, byte keys );


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
 * @fn      Button_ProcessEvent
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
        case ZDO_CB_MSG:  // 收到被绑定节点的rsp
          Button_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt);
          break;

        case KEY_CHANGE:
          Button_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
        break;
      }
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Button_TaskID );
    }
   return (events ^ SYS_EVENT_MSG);
  }

  if( events & MATCH_BIND_EVT ) // 广播 match 绑定
  {
    Sensor_BindDevice(TRUE, BUTTON_CMD_ID, NULL);
    // HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF); // 亮D2
    return (events ^ MATCH_BIND_EVT);
  }
  return 0;
}

/*********************************************************************
 * @fn      Button_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void Button_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case Match_Desc_rsp:
      {
        zAddrType_t dstAddr;
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );

        if ( sensor_bindInProgress != 0xffff )
        {
          // Create a binding table entry
          dstAddr.addrMode = Addr16Bit;
          dstAddr.addr.shortAddr = pRsp->nwkAddr;

          if ( APSME_BindRequest( Button_epDesc.simpleDesc->EndPoint,
                     sensor_bindInProgress, &dstAddr, pRsp->epList[0] ) == ZSuccess )
          {
            osal_stop_timerEx(sapi_TaskID,  ZB_BIND_TIMER);
            osal_start_timerEx( ZDAppTaskID, ZDO_NWK_UPDATE_NV, 250 );

            // Find IEEE addr
            ZDP_IEEEAddrReq( pRsp->nwkAddr, ZDP_ADDR_REQTYPE_SINGLE, 0, 0 );
            // Send bind confirm callback to application
#if ( SAPI_CB_FUNC )
            zb_BindConfirm( sensor_bindInProgress, ZB_SUCCESS );
#endif
            sensor_bindInProgress = 0xffff;
          }
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
      if(myAppState == APP_INIT)
        Sys_SendPreBindMessage(BUTTON_TYPE_ID);
      else
      {
        HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
        HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
        HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );
      }
    }

    // 对远程端点发送命令
    if ( keys & HAL_KEY_SW_2 )
    {

//      zb_SendDataRequest( 0xFFFE,  BUTTON_CMD_ID, 0,
//                        (uint8 *)NULL, sysSeqNumber, 0, 0 );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    // 是用来查找有没有东西可以匹配的
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