/*
 * Copyright (c) 2001, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Adam Dunkels.
 * 4. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * $Id: main.c 30804 2009-07-04 13:00:24Z anderslu $
 *
 */
#include "uipStm/uip/uip.h"
#include "uipStm/uip/uip_arp.h"
#include "tapdev.h"
#include "uipStm/uip/uip_timer.h"
#include "stm32f10x_it.h"
#include <stdio.h>
//#include <uart.h>

#define TICK_PER_SECOND     1000        // 1sec for 1000ticks
#define ethBUF ((struct uip_eth_hdr *)&uip_buf[0])

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */


/*---------------------------------------------------------------------------*/
void uip_log(char *m)
{
    printf("uIP log message: %s\r\n", m);
}
void resolv_found(char *name, u16_t *ipaddr)
{
    if (ipaddr == NULL) {
        printf("Host '%s' not found.\r\n", name);
    } else {
        printf("Found name '%s' = %d.%d.%d.%d\r\n", name,
               htons(ipaddr[0]) >> 8,
               htons(ipaddr[0]) & 0xff,
               htons(ipaddr[1]) >> 8,
               htons(ipaddr[1]) & 0xff);
        //yeelink_connect(YEELINK_HOST, YEELINK_PORT);
    }
}
#ifdef __DHCPC_H__
void
dhcpc_configured(const struct dhcpc_state *s)
{
    uip_sethostaddr(s->ipaddr);
    uip_setnetmask(s->netmask);
    uip_setdraddr(s->default_router);
    resolv_conf(s->dnsaddr);
}
#endif /* __DHCPC_H__ */
void
smtp_done(unsigned char code)
{
    printf("SMTP done with code %d\n", code);
}
void
webclient_closed(void)
{
    printf("Webclient: connection closed\n");
}
void
webclient_aborted(void)
{
    printf("Webclient: connection aborted\n");
}
void
webclient_timedout(void)
{
    printf("Webclient: connection timed out\n");
}
void
webclient_connected(void)
{
    printf("Webclient: connected, waiting for data...\n");
}
void
webclient_datahandler(char *data, u16_t len)
{
    printf("Webclient: got %d bytes of data.\n", len);
}

//__IO uint32_t g_RunTime = 0; --> g_tickCnt
//void SysTick_Handler(void){	g_RunTime++;	// 10ms}

/*---------------------------------------------------------------------------*/
int stmUipHttpdLoop(void) //int main(void)
{
    int i;
    uip_ipaddr_t ipaddr;
    struct timer periodic_timer, arp_timer;

    timer_set(&periodic_timer, CLOCK_SECOND / 2);
    timer_set(&arp_timer, CLOCK_SECOND * 10);

    //uart_init();
    SysTick_Config(SystemCoreClock / TICK_PER_SECOND);

    //============================
    stmENC28J60Init(600, uip_ethaddr.addr); //enc28j60Init(uip_ethaddr.addr); //tapdev_init();
	stmENC28J60ShowPhyRegs();
	stmEnc28WaitForLinkUp();
	//==================

    uip_arp_init();

    __enable_irq();

    uip_init();
    uip_ipaddr(ipaddr, 192, 168, 10, 111);   uip_sethostaddr(ipaddr); 	//my ip address
    uip_ipaddr(ipaddr, 192, 168, 10, 1);     uip_setdraddr(ipaddr); 	//default router's ip addr
    uip_ipaddr(ipaddr, 255, 255, 255, 0);    uip_setnetmask(ipaddr);

    //yeelink_init();
    httpd_init();
    /*  telnetd_init();*/
    /*  hello_world_init();*/
    /*  {
        u8_t mac[6] = {1,2,3,4,5,6};
        dhcpc_init(&mac, 6);
    }*/
    /*    webclient_init();
        resolv_init();
        uip_ipaddr(ipaddr, 205,252,144,288);
        resolv_conf(ipaddr);
        resolv_query("www.baidu.com");*/

    printf("\r\nSTM32F103 Web Server with UIP is running...\r\n");
    printf("You can access with http://192.168.10.111\r\n");
    while (1) {
    	uip_len = stmENC28J60packetReceive(UIP_BUFSIZE + 2, uip_buf);  //uip_len = enc28j60PacketReceive(UIP_BUFSIZE + 2, uip_buf); //uip_len = tapdev_read(uip_buf);
        if (uip_len > 0) {
        	printf("RX>etype=%04x(l=%d)\r\n",htons(ethBUF->type),uip_len);
            if (ethBUF->type == htons(UIP_ETHTYPE_IP)) {
                uip_arp_ipin();
                uip_input();
                // If the above function invocation resulted in data that should be sent out on the network, the global variable uip_len is set to a value > 0.
                if (uip_len > 0) {
                    uip_arp_out();
                    stmENC28J60packetSend(uip_buf, uip_len); //tapdev_send(uip_buf, uip_len);
                }
            } else if (ethBUF->type == htons(UIP_ETHTYPE_ARP)) {
                uip_arp_arpin();
                // If the above function invocation resulted in data that
                //   should be sent out on the network, the global variable
                //  uip_len is set to a value > 0.
                if (uip_len > 0) {
                	stmENC28J60packetSend(uip_buf, uip_len); //enc28j60PacketSend(uip_len, uip_buf); //tapdev_send(uip_buf, uip_len); //response ARP
                }
            }
        } else if (timer_expired(&periodic_timer)) {
            timer_reset(&periodic_timer);
            for (i = 0; i < UIP_CONNS; i++) {
                uip_periodic(i);
                // If the above function invocation resulted in data that
                //   should be sent out on the network, the global variable
                //   uip_len is set to a value > 0.
                if (uip_len > 0) {
                    uip_arp_out();
                    stmENC28J60packetSend(uip_buf, uip_len); //tapdev_send(uip_buf, uip_len);
                }
            }

#if UIP_UDP
            for (i = 0; i < UIP_UDP_CONNS; i++) {
                uip_udp_periodic(i);
                // If the above function invocation resulted in data that
                //   should be sent out on the network, the global variable
                //   uip_len is set to a value > 0.
                if (uip_len > 0) {
                    uip_arp_out();
                    stmENC28J60packetSend(uip_buf, uip_len); // tapdev_send(uip_buf, uip_len);
                }
            }
#endif

            // Call the ARP timer function every 10 seconds.
            if (timer_expired(&arp_timer)) {
                timer_reset(&arp_timer);
                uip_arp_timer();
            }
        }
    }

    return 0;
}
