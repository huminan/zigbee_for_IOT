#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "sapi.h"

#include "SensorSys_End.h"
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

endPointDesc_t Button_epDesc;

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
static uint8 keys_shift = 0;
static uint8 type_join;

byte Button_TaskID;

afAddrType_t Button_DstAddr;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Button_Init( byte task_id );
UINT16 Button_ProcessEvent( byte task_id, UINT16 events );
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
	uint8 startOptions;
	uint8 logicalType;
	// Shift is used to make each button/switch dual purpose.
	if ( keys_shift )
	{
		// Allow Binding
		if ( keys & HAL_KEY_SW_1 )
		{
			if(type_join)
			{
				Sensor_AllowBind(10);
				HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
			// osal_start_timerEx(Sys_TaskID, CLOSE_BIND_EVT, 10000);
			
				keys_shift = 0;
				type_join = 0;
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

    		}
			else
			{
				HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
	 			HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
				HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );
				HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
			}
		}

		if ( keys & HAL_KEY_SW_2 )
		{
		/*
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
