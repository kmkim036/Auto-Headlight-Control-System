/***************************************************************************
** COPYRIGHT: Copyright (c) 2008-2011               
** file 		: D:\Programs\uip-1.0\stm32\src\app_call.h
** summary		: 
** 
** version		: v0.1
** author		: gang.cheng
** Date 		: 2013--3--14
** * Change Logs: 
** Date		Version		Author		Notes
***************************************************************************/

#ifndef __APP_CALL_H__
#define __APP_CALL_H__

#include <stdint.h>

#ifndef UIP_APPCALL
#define UIP_APPCALL                 uip_appcall //It is just tcp_appcall
#endif
typedef uint32_t uip_tcp_appstate_t;

#ifndef UIP_UDP_APPCALL
#define UIP_UDP_APPCALL             udp_appcall
#endif
typedef uint32_t uip_udp_appstate_t;

void uip_appcall(void);
void udp_appcall(void);

#endif

