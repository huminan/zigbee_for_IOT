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
#include "device.h"
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
byte Button_TaskID;
uint8 buttonCnt;

// Button 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
cId_t Button_ClusterList[BUTTON_MAX_CLUSTERS] =
{
  BUTTON_OPEN,
  BUTTON_CLOSE,
  BUTTON_TRIGGER
};

// Button 端点简单描述符
SimpleDescriptionFormat_t Button_SimpleDesc[BUTTON_NUM_MAX] =
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

endPointDesc_t Button_epDesc[BUTTON_NUM_MAX];

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

afAddrType_t Button_DstAddr;
uint16 button_bindInProgress;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Button_Init( byte task_id );
void Button_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void Button_HandleKeys( byte shift, byte keys );

void Button_BindDevice ( uint8 create, uint8 endpoint, uint16 *commandId, uint8 *pDestination );

static void SAPI_BindConfirm( uint16 commandId, uint8 status );
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
  char i;
  Button_TaskID = task_id;
  buttonCnt = 0;
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  Button_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  Button_DstAddr.endPoint = 0;
  Button_DstAddr.addr.shortAddr = 0;

    for( i=0; i<BUTTON_NUM_MAX; i++)
    {
        // Fill out the endpoint description.
        Button_epDesc[i].endPoint = BUTTON_ENDPOINT+i;
        Button_epDesc[i].task_id = &Button_TaskID;
        Button_SimpleDesc[i] = Button_SimpleDesc[0];
        Button_epDesc[i].simpleDesc
                                            = (SimpleDescriptionFormat_t *)&(Button_SimpleDesc[i]);
        Button_SimpleDesc[i].EndPoint += i;
        Button_epDesc[i].latencyReq = noLatencyReqs;
        
        // Register the endpoint description with the AF
        afRegister( &(Button_epDesc[i]) );
    }

  button_bindInProgress = 0xffff;

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
    if(buttonCnt == BUTTON_NUM_MAX)
    {
        // do something
    }
    else
    {
        Button_BindDevice(TRUE, BUTTON_ENDPOINT+buttonCnt, Button_ClusterList, NULL); 
    }
    return (events ^ MATCH_BIND_EVT);
  }
  
  if ( events & BUTTON_BIND_TIMER )
  {
    // Send bind confirm callback to application
    SAPI_BindConfirm( button_bindInProgress, ZB_TIMEOUT );
    button_bindInProgress = 0xffff;

    return (events ^ BUTTON_BIND_TIMER);
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

        if ( button_bindInProgress != 0xffff )
        {
          uint8 ret = 0;
          // Create a binding table entry
          dstAddr.addrMode = Addr16Bit;
          dstAddr.addr.shortAddr = pRsp->nwkAddr;
          
           for(char i=0; i<BUTTON_MAX_CLUSTERS; i++)
          {
                if ( APSME_BindRequest( Button_epDesc[buttonCnt].simpleDesc->EndPoint,
                     button_bindInProgress+i, &dstAddr, pRsp->epList[0] ) != ZSuccess )
                {
                    ret = 1;
                    break;
                }
          }
          if(ret == ZSuccess)
          {
            osal_stop_timerEx(Button_TaskID,  BUTTON_BIND_TIMER);
            osal_start_timerEx( ZDAppTaskID, ZDO_NWK_UPDATE_NV, 250 );
            // Find IEEE addr
            ZDP_IEEEAddrReq( pRsp->nwkAddr, ZDP_ADDR_REQTYPE_SINGLE, 0, 0 );
            // Send bind confirm callback to application
            Sys_BindConfirm( button_bindInProgress, ZB_SUCCESS );
            button_bindInProgress = 0xffff;
            buttonCnt++;
          }
          else
          {
                // ClusterID wrong
          }
        }
      }
    break;
  }
}

/******************************************************************************
 * @fn          Button_BindDevice
 *
 * @brief       The zb_BindDevice function establishes or removes a ��binding? *              between two devices.  Once bound, an application can send
 *              messages to a device by referencing the commandId for the
 *              binding.
 *
 * @param       create - TRUE to create a binding, FALSE to remove a binding
 *              commandId - The identifier of the binding
 *              pDestination - The 64-bit IEEE address of the device to bind to
 *
 * @return      The status of the bind operation is returned in the
 *              Sys_BindConfirm callback.
 */
void Button_BindDevice ( uint8 create, uint8 endpoint, uint16 *commandId, uint8 *pDestination )
{
  zAddrType_t destination;
  uint8 ret = ZB_ALREADY_IN_PROGRESS;

  if ( create )
  {
    if (button_bindInProgress == 0xffff)
    {
      if ( pDestination )
      {
        destination.addrMode = Addr64Bit;
        osal_cpyExtAddr( destination.addr.extAddr, pDestination );
          // srcEndpoint, dstEndpoint
        ret = APSME_BindRequest( endpoint, commandId[0],
                                            &destination, endpoint );

        if ( ret == ZSuccess )
        {
          // Find nwk addr
          ZDP_NwkAddrReq(pDestination, ZDP_ADDR_REQTYPE_SINGLE, 0, 0 );
          osal_start_timerEx( ZDAppTaskID, ZDO_NWK_UPDATE_NV, 250 );
        }
      }
      else
      {
        ret = ZB_INVALID_PARAMETER;
        destination.addrMode = Addr16Bit;
        destination.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
        if ( ZDO_AnyClusterMatches( BUTTON_MAX_CLUSTERS, commandId, Button_epDesc[buttonCnt].simpleDesc->AppNumOutClusters,
                                                Button_epDesc[buttonCnt].simpleDesc->pAppOutClusterList ) )
        {
          // Try to match with a device in the allow bind mode
          ret = ZDP_MatchDescReq( &destination, NWK_BROADCAST_SHORTADDR,
              Button_epDesc[buttonCnt].simpleDesc->AppProfId, BUTTON_MAX_CLUSTERS, commandId, BUTTON_MAX_CLUSTERS, commandId, 0 );
        }
        else if ( ZDO_AnyClusterMatches( BUTTON_MAX_CLUSTERS, commandId, Button_epDesc[buttonCnt].simpleDesc->AppNumInClusters,
                                                Button_epDesc[buttonCnt].simpleDesc->pAppInClusterList ) )
        {
          ret = ZDP_MatchDescReq( &destination, NWK_BROADCAST_SHORTADDR,
              Button_epDesc[buttonCnt].simpleDesc->AppProfId, BUTTON_MAX_CLUSTERS, commandId, BUTTON_MAX_CLUSTERS, commandId, 0 );
        }

        if ( ret == ZB_SUCCESS )
        {
          // Set a timer to make sure bind completes
#if ( ZG_BUILD_RTR_TYPE )
          osal_start_timerEx(Button_TaskID, BUTTON_BIND_TIMER, AIB_MaxBindingTime);
#else
          // AIB_MaxBindingTime is not defined for an End Device
          osal_start_timerEx(Button_TaskID, BUTTON_BIND_TIMER, zgApsDefaultMaxBindingTime);
#endif
          button_bindInProgress = commandId[0];
          return; // dont send cback event
        }
      }
    }

  //  SAPI_SendCback( SAPICB_BIND_CNF, ret, commandId );
  }
  else
  {
    // Remove local bindings for the commandId
    BindingEntry_t *pBind;

    // Loop through bindings an remove any that match the cluster
    while ( pBind = bindFind( endpoint, commandId[0], 0 ) )
    {
      bindRemoveEntry(pBind);
    }
    osal_start_timerEx( ZDAppTaskID, ZDO_NWK_UPDATE_NV, 250 );
  }
  return;
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
    if ( keys & HAL_KEY_SW_6 )
    {
       Sys_SendDataRequest( 0xFFFE, &Motor_epDesc[motorCnt-1], MOTOR_STOP, 0,
                           (uint8 *)NULL, sysSeqNumber, 0, 0 );
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      if(myAppState == APP_INIT) {
        Sys_SendPreBindMessage(MOTOR_TYPE_ID, 1);
        myAppState = APP_START;
      }
      else
      {
        uint8 motorAction[] = "200,5 ,120,30,";
        Sys_SendDataRequest( 0xFFFE, &Motor_epDesc[motorCnt-1], MOTOR_FORWARD, (byte)osal_strlen( motorAction ) + 1,
                           motorAction, sysSeqNumber, 0, 0 );
      }
    }

    if ( keys & HAL_KEY_SW_2 )
    {
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    // 是用来查找有没有东西可以匹配�?    if ( keys & HAL_KEY_SW_4 )
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


/*********************************
 * CALLBACK FUNCTIONS   */
/******************************************************************************
 * @fn          SAPI_BindConfirm
 *
 * @brief       The SAPI_BindConfirm callback is called by the ZigBee stack
 *              after a bind operation completes.
 *
 * @param       commandId - The command ID of the binding being confirmed.
 *              status - The status of the bind operation.
 *              allowBind - TRUE if the bind operation was initiated by a call
 *                          to zb_AllowBindRespones.  FALSE if the operation
 *                          was initiated by a call to ZB_BindDevice
 *
 * @return      none
 */
void SAPI_BindConfirm( uint16 commandId, uint8 status )
{
    Sys_BindConfirm( commandId, status );
}
