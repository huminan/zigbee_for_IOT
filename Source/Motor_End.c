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
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
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

typedef struct {
  uint16 velocity[MOTOR_MAX_OPERATION];
  uint16 delay[MOTOR_MAX_OPERATION];
  uint8 total;      /* how much commands */
  uint8 direct;     /* rotate direction */
  uint8 status;     /* what is doing now */
} MotorControl_t;


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

MotorControl_t MotorControl[MOTOR_NUM_MAX]; 

static uint16 motorTimeout[MOTOR_NUM_MAX] = {0, 0};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Motor_Init( byte task_id );
UINT16 Motor_ProcessEvent( byte task_id, UINT16 events );

static void Motor_ReceiveDataIndication( uint16 source, uint8 endPoint, 
                              uint16 command, uint16 len, uint8 *pData  );
static void Motor_AllowBindConfirm( uint16 source );
static void MotorAction( uint8 motor, uint16 command, uint16 len, uint8 *pData );
static void MotorUpdate(uint8 motor);
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
    if ( events & MOTOR1_UPDATE_EVT )
    {
        MotorUpdate(0);
        return (events ^ MOTOR1_UPDATE_EVT);
    }
    if ( events & MOTOR2_UPDATE_EVT )
    {
        MotorUpdate(1);
        return (events ^ MOTOR2_UPDATE_EVT);
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
  
  switch(motor)
  {
  case 0:
    osal_stop_timerEx(Motor_TaskID, MOTOR1_UPDATE_EVT);
    osal_set_event(Motor_TaskID, MOTOR1_UPDATE_EVT);
    break;
  case 1:
    osal_stop_timerEx(Motor_TaskID, MOTOR2_UPDATE_EVT);
    osal_set_event(Motor_TaskID, MOTOR2_UPDATE_EVT);
    break;
  }
}

void MotorUpdate(uint8 motor)
{
    if(MotorControl[motor].status == (MotorControl[motor].total)-1)
    {
        // STOP
        // Sleep Pin -> low
        (MotorControl[motor].status)++;
        motorTimeout[motor] = 0;
        if(MotorDone())
        {
            HalTimerStop();
        }
        return;
    }

    if(MotorDone())   // first boot
    {
        MOTOR_DIR_1 = MotorControl[motor].direct;
        motorTimeout[motor] = MotorControl[motor].velocity[MotorControl[motor].status];
        HalTimerStart();
    }
    else
    {
        (MotorControl[motor].status)++;
        motorTimeout[motor] = MotorControl[motor].velocity[MotorControl[motor].status];
    }
    
    switch(motor)
    {
    case 0:
        osal_start_timerEx(Motor_TaskID, MOTOR1_UPDATE_EVT, (MotorControl[motor].delay[MotorControl[motor].status])*1000);
        break;
    case 1:
        osal_start_timerEx(Motor_TaskID, MOTOR2_UPDATE_EVT, (MotorControl[motor].delay[MotorControl[motor].status])*1000);
        break;
    }
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
            switch(i)
            {
            case 0:
              MOTOR_STEP_1 ^= 1;
              break;
            case 1:
              MOTOR_STEP_2 ^= 1;
              break;
            }
            motorTimeout[i] = MotorControl[i].velocity[MotorControl[i].status];
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