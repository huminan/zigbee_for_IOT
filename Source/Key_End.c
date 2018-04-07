#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "sapi.h"

#include "SensorSys_End.h"
#include "device.h"
#include "SensorSys_Tools.h"
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
#define KEY_INIT_MSG_MAX    2

// #define KEY_INIT_POS_PORT   0
// #define KEY_INIT_POS_TOGGLE 1

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 keyCnt = 0;




// Key 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
const cId_t Key_ClusterList[KEY_MAX_CLUSTERS] =
{
    PORT_INIT_CLUSTER,
    TOGGLE_INIT_CLUSTER,
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
extern uint8 keys_shift;
extern uint8 type_join;
SensorObserve_t *KeyObserve; 
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */


byte Key_TaskID;

afAddrType_t Key_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Key_Init( byte task_id );
UINT16 Key_ProcessEvent( byte task_id, UINT16 events );
void Key_HandleKeys( byte shift, byte keys );
static void KeyAction( uint8 key, uint16 command, uint16 len, uint8 *pData );

static void Key_ReceiveDataIndication( uint16 source, uint8 endPoint, 
                              uint16 command, uint16 len, uint8 *pData  );
static void Key_AllowBindConfirm( uint16 source );

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
void KeySend2Coor(uint8 dev_num, uint16 commandId, uint8 *pData);


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

        KeyObserve = NULL;
	// Register for all key events - This app will handle all key events
	RegisterForKeys( Key_TaskID );
 
	//	ZDO_RegisterForZDOMsg( Key_TaskID, End_Device_Bind_rsp );
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
            case KEY_CHANGE:
                Key_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
                break;
        
            case AF_INCOMING_MSG_CMD:
                Key_ReceiveDataIndication( MSGpkt->srcAddr.addr.shortAddr, MSGpkt->endPoint, MSGpkt->clusterId,
                                        MSGpkt->cmd.DataLength, MSGpkt->cmd.Data);
                break;
              
            case ZDO_MATCH_DESC_RSP_SENT:
                Key_AllowBindConfirm( ((ZDO_MatchDescRspSent_t *)MSGpkt)->nwkAddr );
                break;
          }
          // Release the memory
          osal_msg_deallocate( (uint8 *)MSGpkt );
    
          // Next
          MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Key_TaskID );
        }
        return (events ^ SYS_EVENT_MSG);
    }
    return 0;
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
        if ( keys & HAL_KEY_SW_6 )  // Physical Key S1
        {
            // Send to Watch Endpoint ? Send to Key Endpoint ?
        }
  }
  
  else
  {
//	uint8 startOptions;
//	uint8 logicalType;
	// Shift is used to make each key/switch dual purpose.
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
		}

		if ( keys & HAL_KEY_SW_3 )
		{
		}
		
		if ( keys & HAL_KEY_SW_4 )
		{
		}
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
  KeyAction(endPoint-KEY_ENDPOINT, command, len, pData);
}

/******************************************************************************
 * @fn          SwitchAction
 *
 * @brief       Switch Action Decide
 *
 * @param
 *
 * @return      none
 */
void KeyAction( uint8 key, uint16 command, uint16 len, uint8 *pData )
{
  if(command == PORT_INIT_CLUSTER)
  {
/*    uint8 i;
    uint8 port[KEY_INIT_MSG_MAX];
    for(i=0; i<KEY_INIT_MSG_MAX; i++, pData++)
    {
        port[i] = *pData;
    }*/
    uint8 port;
    port = *pData;
  //  P2: 1 2 3 4 ; P1: 2 3 4 5 6 7
  //-----------------------------------
  //port: 0 1 2 3       4 5 6 7 8 9
    if(port < KEY_NUM_MAX)
    {
        if(port < P2_KEY_MAX)
        {
            P2SEL &= ~(0x01 << (port + 1));
            P2DIR &= ~(0x01 << (port + 1));
            P2IEN |= 0x01 << (port + 1);
            P2IFG = 0;
        }
        else
        {
            P1SEL &= ~(0x01 << (port - 2));
            P1DIR &= ~(0x01 << (port - 2));
            P1IEN |= 0x01 << (port - 2);
            P1IFG = 0;
        }
        if(KeyObserve == NULL)
        {
          KeyObserve = (SensorObserve_t *)osal_mem_alloc(sizeof(SensorObserve_t));
          KeyObserve->port = port;
          KeyObserve->next = NULL;
        }
        else
            ss_AddObserveList(KeyObserve, port);
        return;
    }
    else
    {
        // send error back
        return;
    }
  }
  if(command == TOGGLE_INIT_CLUSTER)
  {
    if(key > 3) // not exist
    {
        //send error
        return;
    }
    uint8 edge;
    edge = *pData;
    if(edge == KEY_RISE_EDGE)
    {
        PICTL &= ~(0x01<<key);
    }
    else
    {
        PICTL |= 0x01<<key;
    }
  }
}

void KeySend2Coor(uint8 dev_num, uint16 commandId, uint8 *pData)
{
    Sys_SendDataRequest( 0xFFFE, &Key_epDesc[dev_num], commandId, (uint8)osal_strlen( pData ),
                           pData, sysSeqNumber, 0, 0 );
}

/******************************************************************************
 * @fn          Key_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void Key_AllowBindConfirm( uint16 source )
{
    Sys_AllowBindConfirm(source);
}


