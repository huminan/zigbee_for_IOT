/*********************************************************************
 * INCLUDES
 */
#include "AF.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "stdlib.h"

#include "SensorSys_Coor.h"
#include "device.h"
#include "Bluetooth.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_led.h"
#include "hal_uart.h"
#include "MT_UART.h"
#include "MT_RPC.h"

#include "nwk.h"

#include "APS.h"
#include "ZDApp.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "ZComDef.h"
#include "hal_drivers.h"
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
uint8 uartbuf[20];
/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * MT_UART need to configure: 
 * baudrate: MT_UART_DEFAULT_BAUDRATE   9600
 * flow control: MT_UART_DEFAULT_FLOW   FALSE
 * which IO to use: in option complier preprocessor add: ZTOOL_P1
 * INT_HEAP_LEN -> 2072

 * CallBack Function is in MT_UART.c -> void MT_UartProcessZToolData ( uint8 port, uint8 event )
 */
void BluetoothInit(byte task_id)
{
  
  MT_UartInit();
  MT_UartRegisterTaskID(task_id);

}

void Bluetooth_Handle(byte *msg)
{
    switch(msg[MT_RPC_POS_CMD0])
    {
        case BT_CMD_BIND:
        {
           Sys_SendPreBindMessage(msg[MT_RPC_POS_CMD1]);
           
           break;
        }    
        case BT_CMD_PORT:
        {
           byte dev_num, dev_port;
           dev_num = msg[MT_RPC_FRAME_HDR_SZ];
           dev_port = msg[MT_RPC_FRAME_HDR_SZ+1];
           Sys_SendDataRequest( 0xFFFE, &Motor_epDesc[dev_num], PORT_INIT_CLUSTER, (uint8)osal_strlen( &dev_port ),
                           &dev_port, sysSeqNumber, 0, 0 );     // send to latest motor device
           break;
        }
        case BT_CMD_OPERATE:
        {
          // CMDs
           byte dev_num;
           dev_num = msg[MT_RPC_FRAME_HDR_SZ];
           
          // DATAs
           byte *data_p = NULL;
           uint8 len_t = msg[MT_RPC_POS_LEN]-1;
           data_p = (byte *)malloc(sizeof(byte) * len_t);
           for(uint8 i = 0; i<len_t; i++)
           {
                data_p[i] = msg[MT_RPC_FRAME_HDR_SZ+i+1];
           }
           //        uint8 motorAction[] = "200,5 ,120,30,";
           Sys_SendDataRequest( 0xFFFE, &Motor_epDesc[dev_num], OPERATE_CLUSTER, len_t,
                           data_p, sysSeqNumber, 0, 0 );       // send to latest motor device
           free(data_p);
           break;
        }   
        case BT_CMD_LOOP:
        {
          // CMDs
           byte dev_num;
           dev_num = msg[MT_RPC_FRAME_HDR_SZ];
           
          // DATAs
           byte *data_p = NULL;
           uint8 len_t = msg[MT_RPC_POS_LEN]-1;
           data_p = (byte *)malloc(sizeof(byte) * len_t);
           for(uint8 i = 0; i<len_t; i++)
           {
                data_p[i] = msg[MT_RPC_FRAME_HDR_SZ+i+1];
           }
           //        uint8 motorAction[] = "200,5 ,120,30,";
           Sys_SendDataRequest( 0xFFFE, &Motor_epDesc[dev_num], LOOP_OPERATE_CLUSTER, len_t,
                           data_p, sysSeqNumber, 0, 0 );       // send to latest motor device
           free(data_p);
           
           break;
        }
    }
}