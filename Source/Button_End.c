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

uint8 buttonCnt = 0;




// Button 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
const cId_t Button_ClusterList[BUTTON_MAX_CLUSTERS] =
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
extern uint8 keys_shift;
extern uint8 type_join;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */


byte Button_TaskID;

afAddrType_t Button_DstAddr;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Button_Init( byte task_id );
UINT16 Button_ProcessEvent( byte task_id, UINT16 events );
void Button_HandleKeys( byte shift, byte keys );

static void Button_ReceiveDataIndication( uint16 source, uint8 endPoint, 
                              uint16 command, uint16 len, uint8 *pData  );
static void Button_AllowBindConfirm( uint16 source );
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

	// Register for all key events - This app will handle all key events
	RegisterForKeys( Button_TaskID );
 
	//	ZDO_RegisterForZDOMsg( Button_TaskID, End_Device_Bind_rsp );
	ZDO_RegisterForZDOMsg( Button_TaskID, Match_Desc_rsp );

	HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
        HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
        HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );

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
            case KEY_CHANGE:
                Button_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
                break;
        
            case AF_INCOMING_MSG_CMD:
                Button_ReceiveDataIndication( MSGpkt->srcAddr.addr.shortAddr, MSGpkt->endPoint, MSGpkt->clusterId,
                                        MSGpkt->cmd.DataLength, MSGpkt->cmd.Data);
                break;
              
            case ZDO_MATCH_DESC_RSP_SENT:
                Button_AllowBindConfirm( ((ZDO_MatchDescRspSent_t *)MSGpkt)->nwkAddr );
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
  if(shift)
  {
        if ( keys & HAL_KEY_SW_1 )
        {
          zb_SystemReset();
        }
        if ( keys & HAL_KEY_SW_2 )
        {
          zb_SystemReset();
        }
        if ( keys & HAL_KEY_SW_3 )
        {
          zb_SystemReset();
        }
        if ( keys & HAL_KEY_SW_4 )
        {
        }
        if ( keys & HAL_KEY_SW_6 )  // Physical Button S1
        {
            // Send to Watch Endpoint ? Send to Button Endpoint ?
        }
  }
  
  else
  {
//	uint8 startOptions;
//	uint8 logicalType;
	// Shift is used to make each button/switch dual purpose.
	if ( keys_shift )
	{
		// Allow Binding
		if ( keys & HAL_KEY_SW_1 )
		{
			if(type_join)
			{
				Sys_AllowBind(10);
                                
			// osal_start_timerEx(Sys_TaskID, CLOSE_BIND_EVT, 10000);
			
				keys_shift = 0;
			}
			else
			{
				// wrong endpoint the presend send to
			}
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
                  
                  /*
		    if ( myAppState == APP_INIT )
		    {
		        // In the init state, keys are used to indicate the logical mode.
		        // The Switch device is always an end-device
		        logicalType = ZG_DEVICETYPE_ENDDEVICE;
		        zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);

		        // Do more configuration if necessary and then restart device with auto-start bit set

		        zb_ReadConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
		        startOptions = ZCD_STARTOPT_AUTO_START;
		        zb_WriteConfiguration( ZCD_NV_STARTUP_OPTION, sizeof(uint8), &startOptions );
		        zb_SystemReset();

    		    }*/
		  //  else
		//    {

	//	    }
		}

		if ( keys & HAL_KEY_SW_2 )
		{
		/*
			HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

			// 想绑定另一个端�?			dstAddr.addrMode = Addr16Bit;
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
}

/******************************************************************************
 * @fn          Button_ReceiveDataIndication
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
void Button_ReceiveDataIndication( uint16 source, uint8 endPoint, uint16 command, uint16 len, uint8 *pData  )
{
  // ButtonAction(endPoint-BUTTON_ENDPOINT, command);
  switch (endPoint)
  {
    case BUTTON_ENDPOINT:
      {
        if (command == BUTTON_OPEN)
        {
            Button_HandleKeys( 1, HAL_KEY_SW_1 );
        }
        if (command == BUTTON_CLOSE)
        {
            Button_HandleKeys( 1, HAL_KEY_SW_2 );
        }
        if (command == BUTTON_TRIGGER)
        {
            Button_HandleKeys( 1, HAL_KEY_SW_3 );
        }
      }
    case (BUTTON_ENDPOINT+1):
      {
        if (command == BUTTON_OPEN)
        {
            Button_HandleKeys( 1, HAL_KEY_SW_2 );
        }
        if (command == BUTTON_CLOSE)
        {
            Button_HandleKeys( 1, HAL_KEY_SW_2 );
        }
        if (command == BUTTON_TRIGGER)
        {
            Button_HandleKeys( 1, HAL_KEY_SW_3 );
        }
      }
    case (BUTTON_ENDPOINT+2):
      {
        if (command == BUTTON_OPEN)
        {
            Button_HandleKeys( 1, HAL_KEY_SW_3 );
        } 
        if (command == BUTTON_CLOSE)
        {
            Button_HandleKeys( 1, HAL_KEY_SW_2 );
        }
        if (command == BUTTON_TRIGGER)
        {
            Button_HandleKeys( 1, HAL_KEY_SW_3 );
        }
      }
  }
}

/******************************************************************************
 * @fn          Button_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void Button_AllowBindConfirm( uint16 source )
{
    Sys_AllowBindConfirm(source);
}