#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "sapi.h"

#include "SensorSys_End.h"
#include "device.h"
#include "Motor_End.h"

#include "DebugTrace.h"

#if !defined( WIN32 )
	#include "OnBoard.h"
#endif

/* HAL */
#include "hal_led.h"
#include "hal_timer.h"
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
	MOTOR_MAX_CLUSTERS,        //  byte  AppNumInClusters;
	(cId_t *)Motor_ClusterList,  //  byte *pAppInClusterList;
	0,                          //  byte  AppNumInClusters;
	NULL                         //  byte *pAppInClusterList;
};

endPointDesc_t Motor_epDesc[MOTOR_NUM_MAX];

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

afAddrType_t Motor_DstAddr;

SensorControl_t MotorControl[MOTOR_NUM_MAX]; 

static uint16 motorTimeout[MOTOR_NUM_MAX] = {0, 0};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Motor_Init( byte task_id );
UINT16 Motor_ProcessEvent( byte task_id, UINT16 events );

static void Motor_ReceiveDataIndication( uint16 source, uint8 endPoint, 
                              uint16 command, uint16 len, uint8 *pData  );
static void Motor_AllowBindConfirm( uint16 source );

static void Send2Coor(uint8 dev_num, uint16 commandId, uint8 *pData);
static void MotorAction( uint8 motor, uint16 command, uint16 len, uint8 *pData );
static void MotorUpdate(uint8 motor, uint8 first_boot);
static uint8 MotorDone(void);
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

        // Initalize P0 IO direction
        P0SEL &= ~0xf0;
        P0DIR |= 0xf0;
        
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
            
            // NULL -> pointer
            MotorControl[i].msg = NULL;
            
            // Register the endpoint description with the AF
	    afRegister( &(Motor_epDesc[i]) );
        }
 
	//	ZDO_RegisterForZDOMsg( Motor_TaskID, End_Device_Bind_rsp );
	ZDO_RegisterForZDOMsg( Motor_TaskID, Match_Desc_rsp );
        
// Init timer3 for Motor
        HalTimerInit(5);
        halTimerIntConnect(Timer3_uSec);
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
            case AF_INCOMING_MSG_CMD:
                Motor_ReceiveDataIndication( MSGpkt->srcAddr.addr.shortAddr, MSGpkt->endPoint, MSGpkt->clusterId,
                                        MSGpkt->cmd.DataLength, MSGpkt->cmd.Data);
                break;
              
            case ZDO_MATCH_DESC_RSP_SENT:
                Motor_AllowBindConfirm( ((ZDO_MatchDescRspSent_t *)MSGpkt)->nwkAddr );
                break;
          }
          // Release the memory
          osal_msg_deallocate( (uint8 *)MSGpkt );
    
          // Next
          MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Motor_TaskID );
        }
        return (events ^ SYS_EVENT_MSG);
    }
    if ( (events & 0xFF00) & MOTOR_UPDATE_EVT )
    {
        uint8 motor;
        motor = (uint8)(events);
        uint8 status_m;
        status_m = MotorControl[motor].status;
        if(MotorControl[motor].msg[status_m].hour)
        {
            if(MotorControl[motor].msg[status_m].min == 0)
            {
                (MotorControl[motor].msg[status_m].hour)--;
                MotorControl[motor].msg[status_m].min = 59;
            }
            else
              (MotorControl[motor].msg[status_m].min)--;
            osal_start_timerEx(Motor_TaskID, MOTOR_UPDATE_EVT, 60000); // calc 1min
        }
        else
        {
            if(MotorControl[motor].msg[status_m].min == 0)
            {
                if(MotorControl[motor].msg[status_m].sec == 0)
                {
                    MotorUpdate(motor,0);
                    return 0;
                }
                else
                {
                    osal_start_timerEx(Motor_TaskID, MOTOR_UPDATE_EVT, (MotorControl[motor].msg[status_m].sec)*1000);
                    MotorControl[motor].msg[status_m].sec = 0;
                }
            }
            else
            {
              (MotorControl[motor].msg[status_m].min)--;
              osal_start_timerEx(Motor_TaskID, MOTOR_UPDATE_EVT, 60000); // calc 1min
            }
        }
        return 0;
    }
    return 0;
}

/******************************************************************************
 * @fn          Motor_ReceiveDataIndication
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
void Motor_ReceiveDataIndication( uint16 source, uint8 endPoint, uint16 command, uint16 len, uint8 *pData  )
{
  MotorAction(endPoint-MOTOR_ENDPOINT, command, len, pData);
}

/******************************************************************************
 * @fn          Motor_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void Motor_AllowBindConfirm( uint16 source )
{
     motorCnt++;
     Sys_AllowBindConfirm(source);
}

/******************************************************************************
 * @fn          MotorAction
 *
 * @brief       Motor Action Decide
 *
 * @param
 *
 * @return      none
 */
void MotorAction( uint8 motor, uint16 command, uint16 len, uint8 *pData )
{
  // pData contains : " velocity(r/min), delay(0xffff: never)(sec), velocity, delay, ... "
  if(command == PORT_INIT_CLUSTER)
  {
    uint8 port;
    port = *pData;
    if(port < MOTOR_NUM_MAX)    // Port: 0, 1
    {
        P1DIR |= 0x03 << (port*2);
        MotorControl[motor].port = port;
        return;
    }
    else
    {
        // send error back
        return;
    }
  }
  
  
  if(motor>=MOTOR_NUM_MAX)return;   // data error
  if( len % OPERATE_MSG_NUM )return;  // data error: Must be 6 data each group

  uint16 len_t;
  sensor_msg_t *msg_t = NULL;
  len_t = len/OPERATE_MSG_NUM;
  msg_t = (sensor_msg_t *)osal_mem_alloc(sizeof(sensor_msg_t) * len_t);
  uint16 v;
  uint8 i;
  for(i=0; i<len_t; i++)
  {
      if(pData[i*OPERATE_MSG_NUM] >= 1)      // direction
        msg_t[i].value = 1;
      else
        msg_t[i].value = 0;
      
      v = (pData[i*OPERATE_MSG_NUM+1]<<8) | pData[i*OPERATE_MSG_NUM+2];
      msg_t[i].level = 150000/TIMER3_INT_DELAY/v;  // velocity
      msg_t[i].hour = pData[i*OPERATE_MSG_NUM+3];
      msg_t[i].min = pData[i*OPERATE_MSG_NUM+4];
      msg_t[i].sec = pData[i*OPERATE_MSG_NUM+5];
  }
  MotorControl[motor].msg = msg_t;
  MotorControl[motor].total = len_t;
  MotorControl[motor].status = 0;
  MotorControl[motor].command = command;
  osal_mem_free(msg_t);
  /**************************** use ',' devide datas
  // devide word between No.3 and No.4 ','
  if(motor>=MOTOR_NUM_MAX)return;   // data error
  uint8 i;
  uint8 posx1, posx2;
  uint8 num = Num_Pos(len, pData);
  uint8 motorVelocity_arr[5], motorDelay_arr[5];
  if(num%2)return;  // data error
  if(num/2 > MOTOR_MAX_OPERATION)num = MOTOR_MAX_OPERATION*2;   // data too long
  
  posx1 = 0;
  for(i=0; i<num; i++)
  {
      posx2 = Locate_Pos(pData, i+1);
      if(posx2 - posx1 == 0)
        return; //data error
      if(i%2)
      {
        mid(motorDelay_arr, pData, posx2-posx1-1,posx1);
        MotorControl[motor].delay[i/2] = atoi(motorDelay_arr);
      }
      else
      {
        mid(motorVelocity_arr, pData, posx2-posx1-1,posx1);
        MotorControl[motor].velocity[i/2] = 150000/TIMER3_INT_DELAY/atoi(motorVelocity_arr);
      }
      posx1 = posx2;
  }
  MotorControl[motor].total = num/2;
  MotorControl[motor].direct = command;
  MotorControl[motor].status = 0;
  */
  
  uint16 event_t;
  event_t = MOTOR_UPDATE_EVT | motor;       // 0x5000 | 0x00??
  
  osal_stop_timerEx(Motor_TaskID, event_t);
  MotorUpdate(motor, 1);
}

void MotorUpdate(uint8 motor, uint8 first_boot)
{
    uint8 status_m;
    status_m = MotorControl[motor].status;
    if(status_m == (MotorControl[motor].total)-1)
    {
        if(MotorControl[motor].command == LOOP_OPERATE_CLUSTER)
        {
            MotorControl[motor].status = 0x00;
            status_m = 0xff;
        }
        else
        {
            // STOP
            // Sleep Pin -> low
            (MotorControl[motor].status)++;
            motorTimeout[motor] = 0;
            if(MotorDone())
            {
                HalTimerStop();
                osal_mem_free(MotorControl[motor].msg);
            }
            return;
        }
    }
    if(MotorDone())   // first boot
    {
      motorTimeout[motor] = MotorControl[motor].msg[status_m].level;
      HalTimerStart();
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
            MotorControl[motor].status = status_m+1;
            status_m++;
          }
      }
      motorTimeout[motor] = MotorControl[motor].msg[status_m].level;
    }
    
    switch (MotorControl[motor].port)
    {
      case 0:
        MOTOR_DIR_1 = MotorControl[motor].msg[status_m].value;
        break;
      case 1:
        MOTOR_DIR_2 = MotorControl[motor].msg[status_m].value;
        break;
    }
    
    uint16 event_t;
    event_t = MOTOR_UPDATE_EVT | motor;       // 0x5000 | 0x00??
    /*
    if(MotorControl[motor].msg[status_m].hour || MotorControl[motor].msg[status_m].min)
      osal_start_timerEx(Motor_TaskID, event_t, 60000); // calc 1min
    else
      osal_start_timerEx(Motor_TaskID, event_t, (MotorControl[motor].msg[status_m].sec)*1000);
      MotorControl[motor].msg[status_m].sec = 0;*/
    osal_set_event(Motor_TaskID, event_t);
}

void Timer3_uSec()
{
    for(char i = 0; i<MOTOR_NUM_MAX; i++)
    {
        if(motorTimeout[i] == 0)
        {
          continue;
        }
        motorTimeout[i]--;
        if(motorTimeout[i] == 0)
        {
            // reverse
            switch(MotorControl[i].port)
            {
            case 0:
              MOTOR_STEP_1 ^= 1;
              break;
            case 1:
              MOTOR_STEP_2 ^= 1;
              break;
            }
            motorTimeout[i] = MotorControl[i].msg[MotorControl[i].status].level;
        }
    }
}


// Whether all of motor is stop: 1 -> yes
uint8 MotorDone(void)
{
    char i=0;
    for(i; i<MOTOR_NUM_MAX; i++)
    {
        if(motorTimeout[i]!=0)    
        { 
          // if not 0 : running
            return 0;   // not first
        }
    }
    // all is 0 : all is Stop
    return 1;
}


void Send2Coor(uint8 dev_num, uint16 commandId, uint8 *pData)
{
    Sys_SendDataRequest( 0xFFFE, &Motor_epDesc[dev_num], commandId, (uint8)osal_strlen( pData ),
                           pData, sysSeqNumber, 0, 0 );
}