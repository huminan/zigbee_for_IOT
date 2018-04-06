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
byte Motor_TaskID;
uint8 motorCnt;


// Motor 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
cId_t Motor_ClusterList[MOTOR_MAX_CLUSTERS] =
{
    PORT_INIT_CLUSTER,
    OPERATE_CLUSTER,
    LOOP_OPERATE_CLUSTER,
    DELETE_CLUSTER
};

// Motor 端点简单描述符
SimpleDescriptionFormat_t Motor_SimpleDesc[MOTOR_NUM_MAX] =
{
	MOTOR_ENDPOINT,           //  int Endpoint;
	SYS_PROFID,                //  uint16 AppProfId[2];
	SYS_DEVICEID,              //  uint16 AppDeviceId[2];
	SYS_DEVICE_VERSION,        //  int   AppDevVer:4;
	SYS_FLAGS,                 //  int   AppFlags:4;
	0,                          //  byte  AppNumInClusters;
	NULL,                       //  byte *pAppInClusterList;
	MOTOR_MAX_CLUSTERS,          //  byte  AppNumOutClusters;
	(cId_t *)Motor_ClusterList   //  byte *pAppOutClusterList;
};

endPointDesc_t Motor_epDesc[MOTOR_NUM_MAX];

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t Motor_DstAddr;
uint16 motor_bindInProgress;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Motor_Init( byte task_id );
void Motor_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );

void Motor_BindDevice ( uint8 create, uint8 endpoint, uint16 *commandId, uint8 *pDestination );

static void SAPI_BindConfirm( uint16 commandId, uint8 status );
/*********************************************************************
 * @fn      Motor_Init
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
void Motor_Init( byte task_id )
{
  char i;
  Motor_TaskID = task_id;
  motorCnt = 0;
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  Motor_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  Motor_DstAddr.endPoint = 0;
  Motor_DstAddr.addr.shortAddr = 0;

    for( i=0; i<MOTOR_NUM_MAX; i++)
    {
        // Fill out the endpoint description.
        Motor_epDesc[i].endPoint = MOTOR_ENDPOINT+i;
        Motor_epDesc[i].task_id = &Motor_TaskID;
        Motor_SimpleDesc[i] = Motor_SimpleDesc[0];
        Motor_epDesc[i].simpleDesc
                                            = (SimpleDescriptionFormat_t *)&(Motor_SimpleDesc[i]);
        Motor_SimpleDesc[i].EndPoint += i;
        Motor_epDesc[i].latencyReq = noLatencyReqs;
        
        // Register the endpoint description with the AF
        afRegister( &(Motor_epDesc[i]) );
    }

  motor_bindInProgress = 0xffff;

  ZDO_RegisterForZDOMsg( Motor_TaskID, Match_Desc_rsp );
}

/*********************************************************************
 * @fn      Motor_ProcessEvent
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
UINT16 Motor_ProcessEvent( byte task_id, UINT16 events )
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
          Motor_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt);
          break;
      }
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Motor_TaskID );
    }
   return (events ^ SYS_EVENT_MSG);
  }

  if( events & MATCH_BIND_EVT ) // 广播 match 绑定
  {
    if(motorCnt == MOTOR_NUM_MAX)
    {
        // do something
    }
    else
    {
        Motor_BindDevice(TRUE, MOTOR_ENDPOINT+motorCnt, Motor_ClusterList, NULL); 
    }
    return (events ^ MATCH_BIND_EVT);
  }
  
  if ( events & MOTOR_BIND_TIMER )
  {
    // Send bind confirm callback to application
    SAPI_BindConfirm( motor_bindInProgress, ZB_TIMEOUT );
    motor_bindInProgress = 0xffff;

    return (events ^ MOTOR_BIND_TIMER);
  }
  
  return 0;
}

/*********************************************************************
 * @fn      Motor_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void Motor_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case Match_Desc_rsp:
      {
        zAddrType_t dstAddr;
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );

        if ( motor_bindInProgress != 0xffff )
        {
          uint8 ret = 0;
          // Create a binding table entry
          dstAddr.addrMode = Addr16Bit;
          dstAddr.addr.shortAddr = pRsp->nwkAddr;

          for(char i=0; i<MOTOR_MAX_CLUSTERS; i++)
          {
                if ( APSME_BindRequest( Motor_epDesc[motorCnt].simpleDesc->EndPoint,
                     motor_bindInProgress+i, &dstAddr, pRsp->epList[0] ) != ZSuccess )
                {
                    ret = 1;
                    break;
                }
          }
          if(ret == ZSuccess)
          {
            osal_stop_timerEx(Motor_TaskID,  MOTOR_BIND_TIMER);
            osal_start_timerEx( ZDAppTaskID, ZDO_NWK_UPDATE_NV, 250 );
            // Find IEEE addr
            ZDP_IEEEAddrReq( pRsp->nwkAddr, ZDP_ADDR_REQTYPE_SINGLE, 0, 0 );
            // Send bind confirm callback to application
            Sys_BindConfirm( motor_bindInProgress, ZB_SUCCESS );
            motor_bindInProgress = 0xffff;
            motorCnt++;
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
 * @fn          Motor_BindDevice
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
void Motor_BindDevice ( uint8 create, uint8 endpoint, uint16 *commandId, uint8 *pDestination )
{
  zAddrType_t destination;
  uint8 ret = ZB_ALREADY_IN_PROGRESS;

  if ( create )
  {
    if (motor_bindInProgress == 0xffff)
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
        if ( ZDO_AnyClusterMatches( MOTOR_MAX_CLUSTERS, commandId, Motor_epDesc[motorCnt].simpleDesc->AppNumOutClusters,
                                                Motor_epDesc[motorCnt].simpleDesc->pAppOutClusterList ) )
        {
          // Try to match with a device in the allow bind mode
          ret = ZDP_MatchDescReq( &destination, NWK_BROADCAST_SHORTADDR,
              Motor_epDesc[motorCnt].simpleDesc->AppProfId, MOTOR_MAX_CLUSTERS, commandId, 0, (cId_t *)NULL, 0 );
        }
        else if ( ZDO_AnyClusterMatches( MOTOR_MAX_CLUSTERS, commandId, Motor_epDesc[motorCnt].simpleDesc->AppNumInClusters,
                                                Motor_epDesc[motorCnt].simpleDesc->pAppInClusterList ) )
        {
          ret = ZDP_MatchDescReq( &destination, NWK_BROADCAST_SHORTADDR,
              Motor_epDesc[motorCnt].simpleDesc->AppProfId, 0, (cId_t *)NULL, MOTOR_MAX_CLUSTERS, commandId, 0 );
        }

        if ( ret == ZB_SUCCESS )
        {
          // Set a timer to make sure bind completes
#if ( ZG_BUILD_RTR_TYPE )
          osal_start_timerEx(Motor_TaskID, MOTOR_BIND_TIMER, AIB_MaxBindingTime);
#else
          // AIB_MaxBindingTime is not defined for an End Device
          osal_start_timerEx(Motor_TaskID, MOTOR_BIND_TIMER, zgApsDefaultMaxBindingTime);
#endif
          motor_bindInProgress = commandId[0];
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
