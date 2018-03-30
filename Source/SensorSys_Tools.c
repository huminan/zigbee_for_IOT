#include <stdio.h>
#include <string.h>
#include "SensorSys_Tools.h"
#include "ZComDef.h"

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