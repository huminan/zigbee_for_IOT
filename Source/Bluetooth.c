/*********************************************************************
 * INCLUDES
 */
#include "AF.h"
#include "ZDObject.h"
#include "ZDProfile.h"

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
static void rxCB(uint8 port,uint8 event);

void BluetoothInit(void)
{
    HalUARTInit();
    halUARTCfg_t uartConfig;
    
    uartConfig.configured=TRUE;
    uartConfig.baudRate = HAL_UART_BR_9600;
    uartConfig.flowControl = FALSE;
    uartConfig.callBackFunc = rxCB;

    HalUARTOpen(0,&uartConfig);
    
//    HalUARTWrite(0,uartbuf,16);
}

static void rxCB(uint8 port,uint8 event)
{

    if(event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT))
    {
        HalUARTRead(0, uartbuf, Hal_UART_RxBufLen(0));
    }
}