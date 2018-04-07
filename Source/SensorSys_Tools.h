#ifndef SYS_TOOLS
#define SYS_TOOLS

#include <string.h>
#include <stdlib.h>
#include "hal_types.h"
#include "device.h"

extern uint8 * mid(uint8 *dst,uint8 *src, int n,int m);
extern uint8 Locate_Pos(uint8 *buf,uint8 cx);
extern uint8 Num_Pos(uint16 len, uint8 *buf);

extern SensorObserve_t *ss_FindObserveList( SensorObserve_t *ob, uint8 num);
extern SensorObserve_t *ss_EndObserveList( SensorObserve_t *ob);
extern void ss_AddObserveList( SensorObserve_t *ob, uint8 port);
extern void ss_KeyDetermine(SensorObserve_t *ob);
#endif