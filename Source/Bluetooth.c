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
    if( msg[MT_RPC_POS_CMD0] == BT_CMD_BIND )
    {
       Sys_SendPreBindMessage(msg[MT_RPC_POS_CMD1]);
    }
    else if( msg[MT_RPC_POS_CMD0] == BT_CMD_TOGGLE )
    {
       if(msg[MT_RPC_POS_CMD1] != KEY_TYPE_ID)
       {
         // return error
         return;
       }
       byte *data_p = NULL;
       uint8 len_t = msg[MT_RPC_POS_LEN];
       data_p = (byte *)osal_mem_alloc(sizeof(byte) * len_t);
       for(uint8 i = 0; i<len_t; i++)
       {
            data_p[i] = msg[MT_RPC_FRAME_HDR_SZ+i];
       }
       Sys_SendDataRequest( 0xFFFE, Key_epDesc[0], msg[MT_RPC_POS_CMD0], len_t,
                           data_p, sysSeqNumber, 0, 0 );   // 端点默认第一个，无所�?       osal_mem_free(data_p);
    }
    else
    {
       // CMDs
       byte dev_num;           
       dev_num = msg[MT_RPC_FRAME_HDR_SZ];
       
       // DATAs
       byte *data_p = NULL;
       uint8 len_t = msg[MT_RPC_POS_LEN]-1;
       data_p = (byte *)osal_mem_alloc(sizeof(byte) * len_t);
       for(uint8 i = 0; i<len_t; i++)
       {
            data_p[i] = msg[MT_RPC_FRAME_HDR_SZ+i+1];
       }
       switch(msg[MT_RPC_POS_CMD1])
       {
           case KEY_TYPE_ID:
             Sys_SendDataRequest( 0xFFFE, Key_epDesc[dev_num], msg[MT_RPC_POS_CMD0], len_t,
                           data_p, sysSeqNumber, 0, 0 );
             break;
             
           case MOTOR_TYPE_ID:
             Sys_SendDataRequest( 0xFFFE, &Motor_epDesc[dev_num], msg[MT_RPC_POS_CMD0], len_t,
                           data_p, sysSeqNumber, 0, 0 );
             break;
             
           case SWITCH_TYPE_ID:
             Sys_SendDataRequest( 0xFFFE, Switch_epDesc[dev_num], msg[MT_RPC_POS_CMD0], len_t,
                           data_p, sysSeqNumber, 0, 0 );
             break;
       }
       osal_mem_free(data_p);
    }
}