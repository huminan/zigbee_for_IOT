#include <stdio.h>
#include <string.h>
#include "SensorSys_Tools.h"
#include "ZComDef.h"
#include "ioCC2530.h"
#ifndef COOR
    #include "SensorSys_End.h"
#endif
uint8 * mid(uint8 *dst,uint8 *src, int n,int m)
{
    uint8 *p = src;
    uint8 *q = dst;
    int len = strlen(src);
    if(n>len) n = len-m;
    if(m<0) m=0;
    if(m>len) return NULL;
    p += m;
    while(n--) *(q++) = *(p++);
    *(q++)='\0';
    return dst;
}
						  
uint8 Locate_Pos(uint8 *buf,uint8 cx)
{	 		    
	uint8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}

uint8 Num_Pos(uint16 len, uint8 *buf)
{
    uint8 num = 0;
    uint16 length = len;
//    uint8 *p=buf;
    while(length--)
    {		 
        if(*buf==',')num++;
        buf++;
    }
    return num;	
}

void ss_AddObserveList( SensorObserve_t *ob, uint8 port)
{
        SensorObserve_t *new_p, *p;
        new_p = (SensorObserve_t *)osal_mem_alloc(sizeof(SensorObserve_t));
        new_p->port = port;
        new_p->next = NULL;
        p = ss_EndObserveList(ob);
        p->next = new_p;
}


SensorObserve_t *ss_EndObserveList( SensorObserve_t *ob)
{
    SensorObserve_t *p;
    p = ob;
    while(p->next != NULL)
    {
        p = p->next;
    }
    return p;
}


SensorObserve_t *ss_FindObserveList( SensorObserve_t *ob, uint8 num)
{
    SensorObserve_t *p;
    p = ob;
    uint8 i;
    for(i=0; i<num; i++)
    {
        p = ob->next;
    }
    return p;
}

#ifndef COOR
void ss_KeyDetermine(SensorObserve_t *ob)
{
    if(ob ==NULL)
    {
        return;
    }
// estimate *ob first
    SensorObserve_t *p;
    p = ob;
    uint8 count = 0;
    do{
        if(p->port < P2_KEY_MAX)
        {
            if(PICTL & 0x08)    // P2 in falling edge.
            {
                if(P2 & (0x01 << (p->port + 1)))
                {
                    break;
                }
            }
            else        // P2 in rising edge.
            {
                if(~(P2 | (0xFF &(0x01 << (p->port + 1)))))
                {
                    break;
                }
            }
        }
        else if(p->port < P2_KEY_MAX + P1_KEY_MAXL)
        {
            if(PICTL & 0x02)    // P1 L falling edge.
            {
                if(P1 & (0x01 << (p->port - 2)))
                {
                    break;
                }
            }
            else        // P1 L rising edge.
            {
                if(~(P1 | (0xFF &(0x01 << (p->port - 2)))))
                {
                    break;
                }
            }
        }
        else
        {
            if(PICTL & 0x04)    // P1 H falling edge.
            {
                if(P1 & (0x01 << (p->port - 2)))
                {
                    break;
                }
            }
            else        // P1 H rising edge.
            {
                if(~(P1 | (0xFF &(0x01 << (p->port - 2)))))
                {
                    break;
                }
            }
        }
        p = p->next;
        count++;
    }while(p == NULL);
    KeySend2Coor(count, OPERATE_CLUSTER, NULL);
}
#endif