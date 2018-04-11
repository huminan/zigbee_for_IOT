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
byte Key_TaskID;
uint8 keyCnt;

// Key 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
cId_t Key_ClusterList[KEY_MAX_CLUSTERS] =
{
  PORT_INIT_CLUSTER,
    OPERATE_CLUSTER,
    DELETE_CLUSTER
};

// Key 端点简单描述符
SimpleDescriptionFormat_t Key_SimpleDesc[KEY_NUM_MAX] =
{
	KEY_ENDPOINT,           //  int Endpoint;
	SYS_PROFID,                //  uint16 AppProfId[2];
	SYS_DEVICEID,              //  uint16 AppDeviceId[2];
	SYS_DEVICE_VERSION,        //  int   AppDevVer:4;
	SYS_FLAGS,                 //  int   AppFlags:4;
	KEY_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)Key_ClusterList,  //  byte *pAppInClusterList;
	KEY_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)Key_ClusterList   //  byte *pAppInClusterList;
};

endPointDesc_t Key_epDesc[KEY_NUM_MAX];

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

afAddrType_t Key_DstAddr;
uint16 key_bindInProgress;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Key_Init( byte task_id );
void Key_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void Key_HandleKeys( byte shift, byte keys );

void Key_BindDevice ( uint8 create, uint8 endpoint, uint16 *commandId, uint8 *pDestination );

static void Key_ReceiveDataIndication( uint16 source, uint8 endPoint, uint16 command, uint16 len, uint8 *pData  );
static void SAPI_BindConfirm( uint16 commandId, uint8 status );
/*********************************************************************
 * @fn      Key_Init
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
void Key_Init( byte task_id )
{
  char i;
  Key_TaskID = task_id;
  keyCnt = 0;
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  Key_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  Key_DstAddr.endPoint = 0;
  Key_DstAddr.addr.shortAddr = 0;

    for( i=0; i<KEY_NUM_MAX; i++)
    {
        // Fill out the endpoint description.
        Key_epDesc[i].endPoint = KEY_ENDPOINT+i;
        Key_epDesc[i].task_id = &Key_TaskID;
        Key_SimpleDesc[i] = Key_SimpleDesc[0];
        Key_epDesc[i].simpleDesc
                                            = (SimpleDescriptionFormat_t *)&(Key_SimpleDesc[i]);
        Key_SimpleDesc[i].EndPoint += i;
        Key_epDesc[i].latencyReq = noLatencyReqs;
        
        // Register the endpoint description with the AF
        afRegister( &(Key_epDesc[i]) );
    }

  key_bindInProgress = 0xffff;

  // Register for all key events - This app will handle all key events
  RegisterForKeys( Key_TaskID );

  // To Update the display...
  ZDO_RegisterForZDOMsg( Key_TaskID, Match_Desc_rsp );
}

/*********************************************************************
 * @fn      Key_ProcessEvent
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
UINT16 Key_ProcessEvent( byte task_id, UINT16 events )
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
          Key_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt);
          break;

        case KEY_CHANGE:
          Key_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
          
        case AF_INCOMING_MSG_CMD:
          Key_ReceiveDataIndication( MSGpkt->srcAddr.addr.shortAddr, MSGpkt->endPoint, MSGpkt->clusterId,
                                    MSGpkt->cmd.DataLength, MSGpkt->cmd.Data);
          break;
      }
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Key_TaskID );
    }
   return (events ^ SYS_EVENT_MSG);
  }

  if( events & MATCH_BIND_EVT ) // 广播 match 绑定
  {
    if(keyCnt == KEY_NUM_MAX)
    {
        // do something
    }
    else
    {
        Key_BindDevice(TRUE, KEY_ENDPOINT+keyCnt, Key_ClusterList, NULL); 
    }
    return (events ^ MATCH_BIND_EVT);
  }
  
  if ( events & KEY_BIND_TIMER )
  {
    // Send bind confirm callback to application
    SAPI_BindConfirm( key_bindInProgress, ZB_TIMEOUT );
    key_bindInProgress = 0xffff;

    return (events ^ KEY_BIND_TIMER);
  }
  
  return 0;
}

/*********************************************************************
 * @fn      Key_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void Key_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case Match_Desc_rsp:
      {
        zAddrType_t dstAddr;
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );

        if ( key_bindInProgress != 0xffff )
        {
          uint8 ret = 0;
          // Create a binding table entry
          dstAddr.addrMode = Addr16Bit;
          dstAddr.addr.shortAddr = pRsp->nwkAddr;
          
           for(char i=0; i<KEY_MAX_CLUSTERS; i++)
          {
                if ( APSME_BindRequest( Key_epDesc[keyCnt].simpleDesc->EndPoint,
                     key_bindInProgress+i, &dstAddr, pRsp->epList[0] ) != ZSuccess )
                {
                    ret = 1;
                    break;
                }
          }
          if(ret == ZSuccess)
          {
            osal_stop_timerEx(Key_TaskID,  KEY_BIND_TIMER);
            osal_start_timerEx( ZDAppTaskID, ZDO_NWK_UPDATE_NV, 250 );
            // Find IEEE addr
            ZDP_IEEEAddrReq( pRsp->nwkAddr, ZDP_ADDR_REQTYPE_SINGLE, 0, 0 );
            // Send bind confirm callback to application
            Sys_BindConfirm( key_bindInProgress, ZB_SUCCESS );
            key_bindInProgress = 0xffff;
            keyCnt++;
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
 * @fn          Key_BindDevice
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
void Key_BindDevice ( uint8 create, uint8 endpoint, uint16 *commandId, uint8 *pDestination )
{
  zAddrType_t destination;
  uint8 ret = ZB_ALREADY_IN_PROGRESS;

  if ( create )
  {
    if (key_bindInProgress == 0xffff)
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
        if ( ZDO_AnyClusterMatches( KEY_MAX_CLUSTERS, commandId, Key_epDesc[keyCnt].simpleDesc->AppNumOutClusters,
                                                Key_epDesc[keyCnt].simpleDesc->pAppOutClusterList ) )
        {
          // Try to match with a device in the allow bind mode
          ret = ZDP_MatchDescReq( &destination, NWK_BROADCAST_SHORTADDR,
              Key_epDesc[keyCnt].simpleDesc->AppProfId, KEY_MAX_CLUSTERS, commandId, KEY_MAX_CLUSTERS, commandId, 0 );
        }
        else if ( ZDO_AnyClusterMatches( KEY_MAX_CLUSTERS, commandId, Key_epDesc[keyCnt].simpleDesc->AppNumInClusters,
                                                Key_epDesc[keyCnt].simpleDesc->pAppInClusterList ) )
        {
          ret = ZDP_MatchDescReq( &destination, NWK_BROADCAST_SHORTADDR,
              Key_epDesc[keyCnt].simpleDesc->AppProfId, KEY_MAX_CLUSTERS, commandId, KEY_MAX_CLUSTERS, commandId, 0 );
        }

        if ( ret == ZB_SUCCESS )
        {
          // Set a timer to make sure bind completes
#if ( ZG_BUILD_RTR_TYPE )
          osal_start_timerEx(Key_TaskID, KEY_BIND_TIMER, AIB_MaxBindingTime);
#else
          // AIB_MaxBindingTime is not defined for an End Device
          osal_start_timerEx(Key_TaskID, KEY_BIND_TIMER, zgApsDefaultMaxBindingTime);
#endif
          key_bindInProgress = commandId[0];
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
 * @fn      Key_HandleKeys
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
void Key_HandleKeys( byte shift, byte keys )
{
  // Shift is used to make each key/switch dual purpose.
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
       
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      if(myAppState == APP_INIT) {
        
        myAppState = APP_START;
      }
      else
      {
        
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
                        SYS_MAX_CLUSTERS, (cId_t *)Key_ClusterList,
                        SYS_MAX_CLUSTERS, (cId_t *)Key_ClusterList,
                        FALSE );
                        */
    }
  }
}


/******************************************************************************
 * @fn          Key_ReceiveDataIndication
 *
 * @brief       The SAPI_ReceiveDataIndication callback function is called
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
void Key_ReceiveDataIndication( uint16 source, uint8 endPoint, uint16 command, uint16 len, uint8 *pData  )
{
  uint8 *buff = (uint8 *)osal_mem_alloc(sizeof(uint8)*(len+1));
  uint8 i;
  buff[0] = endPoint - KEY_ENDPOINT;
  for(i=1; i<len+1; i++,pData++)
  {
    buff[i] = *pData;
  }
  if(command == OPERATE_CLUSTER)
  {
    HalUARTWrite(0, buff, (byte)osal_strlen( buff ) + 1);
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
