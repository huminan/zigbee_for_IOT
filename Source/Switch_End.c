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
#define SW_UPDATE_EVT   0x5000

// PORT NUMBERS
#define P2_SWITCH_NUM       4
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

byte Switch_TaskID;
uint8 swCnt;


// Switch 端点的簇ID
// This list should be filled with Application specific Cluster IDs.
cId_t Switch_ClusterList[SWITCH_MAX_CLUSTERS] =
{
    PORT_INIT_CLUSTER,
    OPERATE_CLUSTER,
    LOOP_OPERATE_CLUSTER,
    DELETE_CLUSTER
};

// Switch 端点简单描述符
SimpleDescriptionFormat_t Switch_SimpleDesc[SWITCH_NUM_MAX] =
{
	SWITCH_ENDPOINT,           //  int Endpoint;
	SYS_PROFID,                //  uint16 AppProfId[2];
	SYS_DEVICEID,              //  uint16 AppDeviceId[2];
	SYS_DEVICE_VERSION,        //  int   AppDevVer:4;
	SYS_FLAGS,                 //  int   AppFlags:4;
	SWITCH_MAX_CLUSTERS,        //  byte  AppNumInClusters;
	(cId_t *)Switch_ClusterList,  //  byte *pAppInClusterList;
	0,                          //  byte  AppNumInClusters;
	NULL                         //  byte *pAppInClusterList;
};

endPointDesc_t Switch_epDesc[SWITCH_NUM_MAX];

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

afAddrType_t Switch_DstAddr;
SensorControl_t SwitchControl[SWITCH_NUM_MAX]; 
uint8 swTimenow;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Switch_Init( byte task_id );
UINT16 Switch_ProcessEvent( byte task_id, UINT16 events );

static void Switch_ReceiveDataIndication( uint16 source, uint8 endPoint, 
                              uint16 command, uint16 len, uint8 *pData  );
static void Switch_AllowBindConfirm( uint16 source );
static void SwitchAction( uint8 sw, uint16 command, uint16 len, uint8 *pData );
static void SwitchUpdate(uint8 sw, uint8 first_boot);
static void Send2Coor(uint8 dev_num, uint16 commandId, uint8 *pData);
/*********************************************************************
 * @fn      Switch_Init
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
void Switch_Init( byte task_id )
{
        char i;
	Switch_TaskID = task_id;

	// Device hardware initialization can be added here or in main() (Zmain.c).
	// If the hardware is application specific - add it here.
	// If the hardware is other parts of the device add it in main().

	Switch_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
	Switch_DstAddr.endPoint = 0;
	Switch_DstAddr.addr.shortAddr = 0;
        
        for( i=0; i<SWITCH_NUM_MAX; i++)
        {
            // Fill out the endpoint description.
            Switch_epDesc[i].endPoint = SWITCH_ENDPOINT+i;
            Switch_epDesc[i].task_id = &Switch_TaskID;
            Switch_SimpleDesc[i] = Switch_SimpleDesc[0];
            Switch_epDesc[i].simpleDesc
						= (SimpleDescriptionFormat_t *)&(Switch_SimpleDesc[i]);
            Switch_SimpleDesc[i].EndPoint += i;
	    Switch_epDesc[i].latencyReq = noLatencyReqs;
            
            // Register the endpoint description with the AF
	    afRegister( &(Switch_epDesc[i]) );
        }
 
	//	ZDO_RegisterForZDOMsg( Switch_TaskID, End_Device_Bind_rsp );
	ZDO_RegisterForZDOMsg( Switch_TaskID, Match_Desc_rsp );
}

/*********************************************************************
 * @fn      Switch_ProcessEvent
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
UINT16 Switch_ProcessEvent( byte task_id, UINT16 events )
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
                Switch_ReceiveDataIndication( MSGpkt->srcAddr.addr.shortAddr, MSGpkt->endPoint, MSGpkt->clusterId,
                                        MSGpkt->cmd.DataLength, MSGpkt->cmd.Data);
                break;
              
            case ZDO_MATCH_DESC_RSP_SENT:
                Switch_AllowBindConfirm( ((ZDO_MatchDescRspSent_t *)MSGpkt)->nwkAddr );
                break;
          }
          // Release the memory
          osal_msg_deallocate( (uint8 *)MSGpkt );
    
          // Next
          MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Switch_TaskID );
        }
        return (events ^ SYS_EVENT_MSG);
    }
    if( (events & 0xFF00) & SW_UPDATE_EVT )
    {
        uint8 sw;
        sw = (uint8)(events);
        uint8 status_m;
        status_m = SwitchControl[sw].status;
        if(SwitchControl[sw].msg[status_m].hour)
        {
            if(SwitchControl[sw].msg[status_m].min == 0)
            {
                (SwitchControl[sw].msg[status_m].hour)--;
                SwitchControl[sw].msg[status_m].min = 59;
            }
            else
              (SwitchControl[sw].msg[status_m].min)--;
            osal_start_timerEx(Switch_TaskID, events, 60000); // calc 1min
        }
        else
        {
            if(SwitchControl[sw].msg[status_m].min == 0)
            {
                if(SwitchControl[sw].msg[status_m].sec == 0)
                    SwitchUpdate(sw,0);
                else
                {
                    osal_start_timerEx(Switch_TaskID, events, (SwitchControl[sw].msg[status_m].sec)*1000);
                    SwitchControl[sw].msg[status_m].sec = 0;
                }
            }
            else
            {
              (SwitchControl[sw].msg[status_m].min)--;
              osal_start_timerEx(Switch_TaskID, events, 60000); // calc 1min
            }
        }
        return (events ^ events);
    }
    
    return 0;
}

/******************************************************************************
 * @fn          Switch_ReceiveDataIndication
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
void Switch_ReceiveDataIndication( uint16 source, uint8 endPoint, uint16 command, uint16 len, uint8 *pData  )
{
     SwitchAction(endPoint-SWITCH_ENDPOINT, command, len, pData);
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
void SwitchAction( uint8 sw, uint16 command, uint16 len, uint8 *pData )
{
  if(command == PORT_INIT_CLUSTER)
  {
    uint8 port;
    port = *pData;
    
  //  P2: 1 2 3 4 ; P1: 2 3 4 5 6 7
  //-----------------------------------
  //port: 0 1 2 3       4 5 6 7 8 9
    if(port < SWITCH_NUM_MAX)
    {
        if(port < P2_SWITCH_NUM)
        {
            P2SEL &= ~(0x01 << (port + 1));
            P2DIR |= 0x01 << (port+1);
        }
        else
        {
            P1SEL &= ~(0x01 << (port - 2));
            P1DIR |= 0x01 << (port - 2);
        }
        SwitchControl[sw].port = port;
        return;
    }
    else
    {
        // send error back
        return;
    }
  }
  
  
  if(sw>=SWITCH_NUM_MAX)return;   // data error
  if( len % OPERATE_MSG_NUM )return;  // data error: Must be 6 data each group

  uint16 len_t;
  sensor_msg_t *msg_t = NULL;
  len_t = len/OPERATE_MSG_NUM;
  msg_t = (sensor_msg_t *)osal_mem_alloc(sizeof(sensor_msg_t) * len_t);
  uint8 i;
  for(i=0; i<len_t; i++)
  {    
      if(pData[i*OPERATE_MSG_NUM] > 1)      // ON/OFF
        msg_t[i].value = 1;
      else
        msg_t[i].value = 0;
      msg_t[i].level = pData[i*OPERATE_MSG_NUM+1];   // NO USE
      msg_t[i].hour = pData[i*OPERATE_MSG_NUM+3];
      msg_t[i].min = pData[i*OPERATE_MSG_NUM+4];
      msg_t[i].sec = pData[i*OPERATE_MSG_NUM+5];
  }
  SwitchControl[sw].msg = msg_t;
  SwitchControl[sw].total = len_t;
  SwitchControl[sw].status = 0;
  SwitchControl[sw].command = command;
  osal_mem_free(msg_t);
    uint16 event_t;
    event_t = SW_UPDATE_EVT | sw;       // 0x5000 | 0x00??
  
    osal_stop_timerEx(Switch_TaskID, event_t);
    SwitchUpdate(sw,1);
}

void SwitchUpdate(uint8 sw, uint8 first_boot)
{
    uint8 status_m;
    status_m = SwitchControl[sw].status;
    if(status_m == (SwitchControl[sw].total)-1)
    {
        if(SwitchControl[sw].command == LOOP_OPERATE_CLUSTER)
        {
            SwitchControl[sw].status = 0x00;
            status_m = 0xff;
        }
        else
        {
            osal_mem_free(SwitchControl[sw].msg);
            return;
        }
    }
    else
    {
      if(!first_boot)
      {
          if(status_m == 0xff)
          {
            status_m = 0;
          }
          else
          {
            (SwitchControl[sw].status)++;
            status_m++;
          }
      }
    }
    
    uint8 port;
    port = SwitchControl[sw].port;
    if( port < P2_SWITCH_NUM )
    {
        if(SwitchControl[sw].msg[status_m].value)
        {
            P2 |= 0x01 << (port+1);
        }
        else
        {
            P2 &= ~(0x01 << (port+1));
        }
    }
    else
    {
        if(SwitchControl[sw].msg[status_m].value)
        {
            P1 |= 0x01 << (port-2);
        }
        else
        {
            P1 &= ~(0x01 << (port-2));
        }
    }
    
    uint16 event_t;
    event_t = SW_UPDATE_EVT | sw;       // 0x5000 | 0x00??
    
    osal_set_event(Switch_TaskID, event_t);
}


/******************************************************************************
 * @fn          Switch_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void Switch_AllowBindConfirm( uint16 source )
{
     Sys_AllowBindConfirm(source);
}

void Send2Coor(uint8 dev_num, uint16 commandId, uint8 *pData)
{
    Sys_SendDataRequest( 0xFFFE, &Switch_epDesc[dev_num], commandId, (uint8)osal_strlen( pData ),
                           pData, sysSeqNumber, 0, 0 );
}