#ifndef __YEELINK_CLIENT_H__
#define __YEELINK_CLIENT_H__

#include "uipopt.h"

#define WEBCLIENT_CONF_MAX_URLLEN 400

#define YEELINK_HOST        "www.yeelink.net"
#define YEELINK_PORT        80

typedef struct  {
    u8_t timer;
    u8_t httpflag;

    u16_t port;
    char  host[40];
    char  data[WEBCLIENT_CONF_MAX_URLLEN];
    u16_t get_request_ptr;
    u16_t get_request_left;
} yeelink_client_state_t;

typedef yeelink_client_state_t uip_tcp_appstate_t;

#define UIP_APPCALL yeelink_client_appcall

void yeelink_init(void);

u8_t yeelink_connect(char *host, u16_t port);

void yeelink_client_appcall(void);

#endif
