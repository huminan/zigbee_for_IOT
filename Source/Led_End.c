#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
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

byte Led_TaskID;
uint8 ledCnt;


// Led 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
cId_t Led_ClusterList[LED_MAX_CLUSTERS] =
{
    LED_LIGHT,
    LED_DIM,
    LED_FLASH
};

// Led 端点简单描述符
SimpleDescriptionFormat_t Led_SimpleDesc[LED_NUM_MAX] =
{
	LED_ENDPOINT,           //  int Endpoint;
	SYS_PROFID,                //  uint16 AppProfId[2];
	SYS_DEVICEID,              //  uint16 AppDeviceId[2];
	SYS_DEVICE_VERSION,        //  int   AppDevVer:4;
	SYS_FLAGS,                 //  int   AppFlags:4;
	LED_MAX_CLUSTERS,        //  byte  AppNumInClusters;
	(cId_t *)Led_ClusterList,  //  byte *pAppInClusterList;
	0,                          //  byte  AppNumInClusters;
	NULL                         //  byte *pAppInClusterList;
};

endPointDesc_t Led_epDesc[LED_NUM_MAX];

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 myAppState;
extern uint8 keys_shift;
extern uint8 type_join;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

afAddrType_t Led_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Led_Init( byte task_id );
UINT16 Led_ProcessEvent( byte task_id, UINT16 events );

static void Led_ReceiveDataIndication( uint16 source, uint8 endPoint, 
                              uint16 command, uint16 len, uint8 *pData  );
static void Led_AllowBindConfirm( uint16 source );
/*********************************************************************
 * @fn      Led_Init
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
void Led_Init( byte task_id )
{
        char i;
	Led_TaskID = task_id;

	// Device hardware initialization can be added here or in main() (Zmain.c).
	// If the hardware is application specific - add it here.
	// If the hardware is other parts of the device add it in main().

	Led_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
	Led_DstAddr.endPoint = 0;
	Led_DstAddr.addr.shortAddr = 0;
        
        for( i=0; i<LED_NUM_MAX; i++)
        {
            // Fill out the endpoint description.
            Led_epDesc[i].endPoint = LED_ENDPOINT+i;
            Led_epDesc[i].task_id = &Led_TaskID;
            Led_SimpleDesc[i] = Led_SimpleDesc[0];
            Led_epDesc[i].simpleDesc
						= (SimpleDescriptionFormat_t *)&(Led_SimpleDesc[i]);
            Led_SimpleDesc[i].EndPoint += i;
	    Led_epDesc[i].latencyReq = noLatencyReqs;
            
            // Register the endpoint description with the AF
	    afRegister( &(Led_epDesc[i]) );
        }
 
	//	ZDO_RegisterForZDOMsg( Led_TaskID, End_Device_Bind_rsp );
	ZDO_RegisterForZDOMsg( Led_TaskID, Match_Desc_rsp );
}

/*********************************************************************
 * @fn      Led_ProcessEvent
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
UINT16 Led_ProcessEvent( byte task_id, UINT16 events )
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
            case AF_INCOMING_MSG_CMD:
                Led_ReceiveDataIndication( MSGpkt->srcAddr.addr.shortAddr, MSGpkt->endPoint, MSGpkt->clusterId,
                                        MSGpkt->cmd.DataLength, MSGpkt->cmd.Data);
                break;
              
            case ZDO_MATCH_DESC_RSP_SENT:
                Led_AllowBindConfirm( ((ZDO_MatchDescRspSent_t *)MSGpkt)->nwkAddr );
                break;
          }
          // Release the memory
          osal_msg_deallocate( (uint8 *)MSGpkt );
    
          // Next
          MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Led_TaskID );
        }
        return (events ^ SYS_EVENT_MSG);
    }
    return 0;
}

/******************************************************************************
 * @fn          Led_ReceiveDataIndication
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
void Led_ReceiveDataIndication( uint16 source, uint8 endPoint, uint16 command, uint16 len, uint8 *pData  )
{
  // LedAction(endPoint-LED_ENDPOINT, command, pData);
  
  switch (endPoint)
  {
    case LED_ENDPOINT:
      {
        if (command == LED_LIGHT)
        {
             zb_SystemReset();
        }
        if (command == LED_DIM)
        {
             zb_SystemReset();
        }
      }
    case (LED_ENDPOINT+1):
      {
        if (command == LED_LIGHT)
        {
            zb_SystemReset();
        }
      }
    case (LED_ENDPOINT+2):
      {
        if (command == LED_LIGHT)
        {
            
        }
      }
  }
}

/******************************************************************************
 * @fn          Led_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void Led_AllowBindConfirm( uint16 source )
{
     Sys_AllowBindConfirm(source);
}