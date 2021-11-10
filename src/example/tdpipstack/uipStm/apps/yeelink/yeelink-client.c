#include "uip.h"
#include "uiplib.h"
#include "yeelink-client.h"
#include "resolv.h"

#include <string.h>
#include <stdio.h>

#define WEBCLIENT_TIMEOUT 100


static const u8_t http_get_request[] =
    "GET /v1.0/device/7907/sensor/12537/datapoints HTTP/1.1\r\n"
    "U-ApiKey:90ff838ed4fa41e28091f18254a82073\r\n"
    "Host: api.yeelink.net\r\n\r\n";

static yeelink_client_state_t yeelink_client;

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */

/*---------------------------------------------------------------------------*/

void yeelink_init(void)
{
    uip_ipaddr_t ipaddr;
    resolv_init();
    uip_ipaddr(ipaddr, 221, 228, 255, 1); // DNS
    resolv_conf(ipaddr);
    resolv_query(YEELINK_HOST);
}

static void yeelink_struct_init(void)
{
    yeelink_client.get_request_left = sizeof(http_get_request) - 1;
    yeelink_client.get_request_ptr = 0;
}

// api.yeelink.net IP : 42.96.164.52
u8_t yeelink_connect(char *host, u16_t port)
{
    struct uip_conn *conn = NULL;
    uip_ipaddr_t *ipaddr = NULL;
    static uip_ipaddr_t addr;

    if (uiplib_ipaddrconv(host, (unsigned char *)&addr) == 0) {
        ipaddr = (uip_ipaddr_t *)resolv_lookup(host);

        if (ipaddr == NULL) {
            return 0;
        }
    } else {
        ipaddr = &addr;
    }


    conn = uip_connect(ipaddr, htons(port));
    if (conn == NULL) {
        return 0;
    }

    yeelink_client.port = port;
    strncpy(yeelink_client.host, host, sizeof(yeelink_client.host));

    yeelink_client.timer = 0;
    
    yeelink_struct_init();

    return 1;
}

static u8_t *copy_string(u8_t *dest, const u8_t *src, u8_t len)
{
    strncpy((char *)dest, (const char *)src, len);
    return dest + len;
}

static void yeelink_request_get(void)
{
    u16_t len;
    u8_t *getrequest = NULL;
    u8_t *cptr = NULL;

    yeelink_client.timer = 0;

    if (yeelink_client.get_request_left > 0) {
        cptr = getrequest = uip_appdata;

        cptr = copy_string(cptr, http_get_request, sizeof(http_get_request) - 1);
        len = yeelink_client.get_request_left > uip_mss() ?
              uip_mss() :
              yeelink_client.get_request_left;

        uip_send(&(getrequest[yeelink_client.get_request_ptr]), len);
    }
}

static void yeelink_request_acked(void)
{
    u16_t len;

    yeelink_client.timer = 0;
    if (yeelink_client.get_request_left > 0) {
        len = yeelink_client.get_request_left > uip_mss() ?
              uip_mss() :
              yeelink_client.get_request_left;
        yeelink_client.get_request_left -= len;
        yeelink_client.get_request_ptr += len;
    }
}

static void yeelink_newdata(void)
{
    yeelink_client.timer = 0;

    u16_t len = uip_datalen();

    memcpy(yeelink_client.data, uip_appdata, len);

    //{"timestamp":"2014-03-25T13:40:47","value":0}
    u8_t *value_info = (u8_t *)strstr(yeelink_client.data, "\"value\"");
    if (value_info != NULL) {   // 分片传送的
        len = strlen("\"value\":");
        u8_t status = *(value_info + len);

        if (status == '0') {
            printf("yeelink_client: switch is close\r\n");
        } else {
            printf("yeelink_client: switch is open\r\n");
        }

        memset(yeelink_client.data, 0x00 , WEBCLIENT_CONF_MAX_URLLEN);

        /*uip_close();*/
        yeelink_struct_init();
    }
}

static u8_t yeelink_poll(void)
{
    ++yeelink_client.timer;
    if (yeelink_client.timer == WEBCLIENT_TIMEOUT) {
        yeelink_request_get();
        return 1;
    } else {
        return 0;
    }
}

static void yeelink_closed(void)
{
    if (resolv_lookup(YEELINK_HOST) == NULL) {
        resolv_query(YEELINK_HOST);
    } else {
        yeelink_connect(YEELINK_HOST, YEELINK_PORT);
    }
}

void yeelink_client_appcall(void)
{
    if (uip_connected()) {
        yeelink_request_get();
        printf("yeelink_client: connected, waiting for data...\r\n");
        return;
    }

    if (uip_aborted()) {
        printf("yeelink_client: connection aborted\r\n");
    }

    if (uip_timedout()) {
        printf("yeelink_client: connection timed out\r\n");
    }

    if (uip_acked()) {
        yeelink_request_acked();
    }

    if ( uip_rexmit() || uip_newdata()) {
        yeelink_newdata();
        return;
    }

    if (uip_poll()) {
        if (yeelink_poll()) {
            return;
        }
    }

    if (uip_closed()) {
        yeelink_closed();
        printf("yeelink_client: connection closed\r\n");
    }
}









