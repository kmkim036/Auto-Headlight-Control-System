//YOON
// Add for 802.1as

#include <stdio.h>
#include <string.h>
#include "ethopts.h"
#include "yInc.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "misc.h"
//#include "constants.h"
#include "etharp.h"
#include "lwipopts.h"
#include "lwip/include/opt.h"

#if (USE_LWIP)
#include "datatypes.h"
#include "ptpstm/include/ptpd.h"
//#include "lwip/include/lwipopts.h"
/* net.c -- https://github.com/mpthompson/stm32_f4_ptpd/blob/master/libraries/ptpd-2.0.0/src/dep/net.c*/
//EVENT PORT 319 for SYNC,Delay_Req, Pdelay_Req and Pdelay_resp.
//General 320 : Announce, Follow_Up, Delay_Resp, Pdelay_Resp_Follow_Up,
// Management and Signaling messages are members of the general message
//THIS "net" prefix means that the below UDP layer is the "net"work in perspective of 1588 applications.

extern PtpClock ptpClock;
// Initialize network queue.
static void netQInit(BufQueue *queue, unsigned short qtype)
{
	queue->head = 0;
	queue->tail = 0;
	queue->qtype = qtype;
	//sys_mutex_new(&queue->mutex);
}

// Put data to the APP queue.
static bool netQPut(BufQueue *queue, void *pbuf)
{
	bool retval = FALSE;

	//sys_mutex_lock(&queue->mutex);
	// Is there room on the queue for the buffer?
	if (((queue->head + 1) & PBUF_QUEUE_MASK) != queue->tail){
		// Place the buffer in the queue.
		queue->head = (queue->head + 1) & PBUF_QUEUE_MASK;
		queue->pbuf[queue->head] = pbuf;
		retval = TRUE;
		//printf("PUT at (%d,%d)\r\n",queue->qtype, queue->head);
	}else{
		printf("PUT FULL\r\n");
	}
	//ptpClock.netPath.qseq = (ptpClock.netPath.qseq + 1) % 256; //add YOON

	//sys_mutex_unlock(&queue->mutex);

	return retval;
}

// Get data from the APP queue.
static void *netQGet(BufQueue *queue)
{
	void *pbuf = NULL;

	//sys_mutex_lock(&queue->mutex);

	// Is there a buffer on the queue?
	if (queue->tail != queue->head)	{
		// Get the buffer from the queue.
		queue->tail = (queue->tail + 1) & PBUF_QUEUE_MASK;
		pbuf = queue->pbuf[queue->tail];
		//printf("GET from (%d, %d)\r\n", queue->qtype, queue->tail);
	}else{
			//printf("GET EMPTY\r\n");
		}
	//sys_mutex_unlock(&queue->mutex);

	return pbuf;
}
//YOON ADD
void *netQPeek(BufQueue *queue){
	void *pbuf = NULL;
	int16_t tmptail;

	//sys_mutex_lock(&queue->mutex);
	if (queue->tail != queue->head)	{// Is there a buffer on the queue?
		// Peek the buffer from the queue.
		tmptail = (queue->tail + 1) & PBUF_QUEUE_MASK;
		pbuf = queue->pbuf[tmptail];
	}
	//sys_mutex_unlock(&queue->mutex);

	return pbuf;
}

/* Free any remaining pbufs in the queue. */
static void netQEmpty(BufQueue *queue)
{
	//sys_mutex_lock(&queue->mutex);

	// Free each remaining buffer in the queue.
	while (queue->tail != queue->head){
		// Get the buffer from the queue.
		queue->tail = (queue->tail + 1) & PBUF_QUEUE_MASK;
		pbuf_free(queue->pbuf[queue->tail]);
	}

	//sys_mutex_unlock(&queue->mutex);
}

/* Check if something is in the queue */
static bool netQCheck(BufQueue  *queue)
{
	bool  retval = FALSE;

	//sys_mutex_lock(&queue->mutex);

	if (queue->tail != queue->head) retval = TRUE;

	//sys_mutex_unlock(&queue->mutex);

	return retval;
}

// Wait for a packet  to come in on either port.  For now, there is no wait.
// Simply check to  see if a packet is available on either port and return 1, otherwise return 0.
int netSelect(NetPath *netPath, const TimeInternal *timeout)
{
	/* Check the packet queues.  If there is data, return TRUE. */
	if (netQCheck(&netPath->eventQ) || netQCheck(&netPath->generalQ)) return 1;

	return 0;
}

/* Delete all waiting packets in event queue. */
void netEmptyEventQ(NetPath *netPath)
{
	netQEmpty(&netPath->eventQ);
}

//Get the packet
static ssize_t netRecv(octet_t *buf, TimeInternal *time, BufQueue *msgQueue)
{
	int i;
	int j;
	u16_t length;
	struct pbuf *p;
	struct pbuf *pcopy;

	/* Get the next buffer from the APP queue. */
	if ((p = (struct pbuf*) netQGet(msgQueue)) == NULL)	{
		return 0;
	}

	/* Verify that we have enough space to store the contents. */
	if (p->tot_len > PACKET_SIZE)	{
		ERROR("netRecv: received truncated message\n");
		pbuf_free(p);
		return 0;
	}

	/* Verify there is contents to copy. */
	if (p->tot_len == 0){
		ERROR("netRecv: received empty packet\n");
		pbuf_free(p);
		return 0;
	}

	if (time != NULL){
#if USE_LWIP_PTP
		time->seconds = p->time_sec;
		time->nanoseconds = p->time_nsec;
#else
		getTimeFromReg(time);
#endif
	}

	/* Get the length of the buffer to copy. */
	length = p->tot_len;

	/* Copy the pbuf payload into the buffer. */
	pcopy = p;
	j = 0;
	for (i = 0; i < length; i++){ // Copy the next byte in the payload.
		buf[i] = ((u8_t *)pcopy->payload)[j++];
		// Skip to the next buffer in the payload?
		if (j == pcopy->len)	{	// Move to the next buffer.
			pcopy = pcopy->next;
			j = 0;
		}
	}

	/* Free up the pbuf (chain). */
	pbuf_free(p);

	return length;
}

ssize_t netRecvEvent(NetPath *netPath, octet_t *buf, TimeInternal *time)
{
	//struct sockaddr_in from_addr;
	//memset(&from_addr, 0, sizeof(from_addr));
	//netPath->lastDestAddr = 0;
	return netRecv(buf, time, &netPath->eventQ);
}

ssize_t netRecvGeneral(NetPath *netPath, octet_t *buf, TimeInternal *time)
{
	return netRecv(buf, time, &netPath->generalQ);
}

/* Find interface to  be used.  uuid should be filled with MAC address of the interface.
	 Will return the IPv4 address of  the interface. */
static int32_t findIface(const octet_t *ifaceName, octet_t *uuid, NetPath *netPath)
{
	struct netif *iface;

	iface = netif_default;
	memcpy(uuid, iface->hwaddr, iface->hwaddr_len);

	return iface->ip_addr.addr;
}

//============= Project Depedent ================================

#if (PROJ_FOR == PROJ_FOR_RSTP)

/* Processing an incoming message on the BPDU port. */
static void netRecvBpduCallback(void *arg, struct avb_pcb *pcb, struct pbuf *p,
  struct eth_addr *addr, u16_t etype)
{
	NetPath *netPath = (NetPath *) arg;

	// Place the incoming message on the 8021 QUEUE.
	if (!netQPut(&netPath->ieee8021Q, p))
	{
		pbuf_free(p);
		ERROR("netRecvEventCallback: queue full\n");
		return;
	}

	//ptpd_alert();// Alert the PTP thread there is now something to do.

}

static ssize_t netSendLink(const octet_t *buf, int16_t  length,
		TimeInternal *time, const int32_t * addr,
		struct rawavb_pcb * pcb)
{
	err_t result;
	struct pbuf *p;

	// Allocate the tx pbuf based on the current size.
	p = pbuf_alloc(PBUF_LINK, length, PBUF_RAM);
	//p = (struct rawavb_pcb *)memp_malloc(MEMP_RAW_PCB);
	if (NULL == p){
		ERROR("netSend: Failed to allocate Tx Buffer\r\n");
		goto fail01;
	}

	// Copy the incoming data into the pbuf payload.
	result = pbuf_take(p, buf, length);
	if (ERR_OK != result){
		ERROR("netSend: Failed to copy data to Pbuf (%d)\r\n", result);
		goto fail02;
	}
	// move pointer for Ethernet header
	if (pbuf_header(p, sizeof(struct eth_hdr)) != 0) {
		  printf("ieee8023raw_output(): pbuf_header Error\r\n");
		  return ERR_BUF;
	}
	// send the buffer.
	//result = lwLink_sendto(pcb, p, (void *)addr, pcb->etype);
	result = lwLink_sendto(pcb, p);//, pcb->etype);
	if (ERR_OK != result)
	{
		ERROR("netSend: Failed to send data (%d)\r\n", result);
		goto fail02;
	}

	if (time != NULL)
	{
#if USE_LWIP_PTP
		time->seconds = p->time_sec;
		time->nanoseconds = p->time_nsec;
#else
		/* TODO: use of loopback mode */
		/*
		time->seconds = 0;
		time->nanoseconds = 0;
		*/
		getTimeFromReg(time);
#endif
		DBGV("netSend: %d.%09d sec\r\n", time->seconds, time->nanoseconds);
	} else {
		DBGV("netSend\r\n");
	}
fail02:
	pbuf_free(p);

fail01:
	return length;
}

size_t netSendBpdu(NetPath *netPath, const octet_t *buf, int16_t  length, TimeInternal *time)
{
	return netSendLink(buf, length, time, &netPath->multicastAddr, netPath->ieee8021Pcb);
}

/* Start  all of the 802.1Q stuff */
bool netInit(NetPath *netPath, PtpClock *ptpClock)
{
	struct in_addr netAddr;
	struct ip_addr interfaceAddr;
	char addrStr[NET_ADDRESS_LENGTH];

	/* Initialize the buffer queues. */
	//netQInit(&netPath->eventQ, 319);
	//netQInit(&netPath->generalQ, 320);
	netQInit(&netPath->ieee8021Q, 0); //added

	/* Find a network interface */
	interfaceAddr.addr = findIface(ptpClock->rtOpts->ifaceName, ptpClock->portUuidField, netPath);
	if (!(interfaceAddr.addr)){
			ERROR("netInit: Failed to find interface address\r\n");
			goto fail01;
	}

	// (1) Open 8021 raw interfaces for the rstp.
	netPath->ieee8021Pcb = rawavb_new(ETHTYPE_8021BPDU); //0x8100
	if (NULL == netPath->ieee8021Pcb){
			ERROR("netInit: Failed to open 802.1 PCB\n");
			goto fail04;
	}

	//== Establish the appropriate raw bindings/connections for BPDU
	rawavb_recv(netPath->ieee8021Pcb, netRecvBpduCallback, netPath);
	rawavb_bind(netPath->ieee8021Pcb, IFTYPE_IEEE8021BPDU, netif_default);

	return TRUE;
fail04:
	rawavb_remove(netPath->ieee8021Pcb);
fail01:
	return FALSE;
}
#elif (PROJ_FOR == PROJ_FOR_PTP)

#if (PTP_PROTOCOL == IEEE1588V2)

/* Process an incoming message on the Event port. */
static void netRecvEventCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
								 struct ip_addr *addr, u16_t port)
{
	NetPath *netPath = (NetPath *) arg;

	/* Place the incoming message on the Event Port QUEUE. */
	if (!netQPut(&netPath->eventQ, p)){
		pbuf_free(p);
		ERROR("netRecvEventCallback: queue full\n");
		return;
	}

	/* Alert the PTP thread there is now something to do. */
	ptpd_alert();
}

/* Process an incoming message on the General port. */
static void netRecvGeneralCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
									struct ip_addr *addr, u16_t port)
{
	NetPath *netPath = (NetPath *) arg;

	/* Place the incoming message on the Event Port QUEUE. */
	if (!netQPut(&netPath->generalQ, p)){
		pbuf_free(p);
		ERROR("netRecvGeneralCallback: queue full\n");
		return;
	}

	/* Alert the PTP thread there is now something to do. */
	//ptpd_alert();
}

/* Start  all of the UDP stuff */
bool netInit(NetPath *netPath, PtpClock *ptpClock)
{
	struct in_addr netAddr;
	struct ip_addr interfaceAddr;
	char addrStr[NET_ADDRESS_LENGTH];

	/* Initialize the buffer queues. */
	netQInit(&netPath->eventQ, 319);
	netQInit(&netPath->generalQ, 320);

	/* Find a network interface */
	interfaceAddr.addr = findIface(ptpClock->rtOpts->ifaceName, ptpClock->portUuidField, netPath);
	if (!(interfaceAddr.addr)){
			ERROR("netInit: Failed to find interface address\r\n");
			goto fail01;
	}

	// (1) Open lwIP raw udp interfaces for the event port(319).
	netPath->eventPcb = lw_udp_new();
	if (NULL == netPath->eventPcb){
			ERROR("netInit: Failed to open Event UDP PCB\n");
			goto fail02;
	}
#if IP_SOF_BROADCAST //YOON 2016.9.30
	netPath->eventPcb->so_options |= SOF_BROADCAST;
#endif /* IP_SOF_BROADCAST */

	// (2) Open lwIP raw udp interfaces for the general port(320).
	netPath->generalPcb = lw_udp_new();
	if (NULL == netPath->generalPcb){
			ERROR("netInit: Failed to open General UDP PCB\n");
			goto fail03;
	}
#if IP_SOF_BROADCAST //YOON 2016.9.30
	netPath->generalPcb->so_options |= SOF_BROADCAST;
#endif /* IP_SOF_BROADCAST */


	/* Configure network (broadcast/unicast) addresses. */
	netPath->unicastAddr = 0; /* disable unicast */

	/* Init General multicast IP address */
	memcpy(addrStr, DEFAULT_PTP_DOMAIN_ADDRESS, NET_ADDRESS_LENGTH); //224.0.1.129
	if (!inet_aton(addrStr, &netAddr)){
			ERROR("netInit: failed to encode multi-cast address: %s\n", addrStr);
			goto fail04;
	}
	netPath->multicastAddr = netAddr.s_addr;

	/* Join multicast group (for receiving) on specified interface for General multicast IP address */
	igmp_joingroup(&interfaceAddr, (struct ip_addr *)&netAddr);

	/* (2) Init Peer multicast IP address for Event multicast IP address */
	memcpy(addrStr, PEER_PTP_DOMAIN_ADDRESS, NET_ADDRESS_LENGTH);
	if (!inet_aton(addrStr, &netAddr)){
			ERROR("netInit: failed to encode peer multi-cast address: %s\n", addrStr);
			goto fail04;
	}
	netPath->peerMulticastAddr = netAddr.s_addr;

	/* Join peer multicast group (for receiving) on specified interface */
	igmp_joingroup(&interfaceAddr, (struct ip_addr *) &netAddr);

	/* Multicast send only on specified interface. */
	netPath->eventPcb->multicast_ip.addr = netPath->multicastAddr;
	netPath->generalPcb->multicast_ip.addr = netPath->multicastAddr;

	//==Establish the appropriate UDP bindings/connections for events.
	lw_udp_recv(netPath->eventPcb, netRecvEventCallback, netPath);
	lw_udp_bind(netPath->eventPcb, IP_ADDR_ANY, PTP_EVENT_PORT);
	//udp_connect(netPath->eventPcb, &netAddr, PTP_EVENT_PORT);  //ADD YOON
	*(Integer16*)ptpClock->event_port_address = PTP_EVENT_PORT;//ADD YOON

	//== Establish the appropriate UDP bindings/connections for general.
	lw_udp_recv(netPath->generalPcb, netRecvGeneralCallback, netPath);
	lw_udp_bind(netPath->generalPcb, IP_ADDR_ANY, PTP_GENERAL_PORT);
	//udp_connect(netPath->generalPcb, &netAddr, PTP_GENERAL_PORT); //ADD YOON
	*(Integer16*)ptpClock->general_port_address = PTP_GENERAL_PORT;//ADD YOON

#if (PROJ_FOR == PROJ_FOR_RSTP)

	netQInit(&netPath->ieee8021Q, 0); //added -- RSTP
	// (3) Open 8021 raw interfaces for the rstp.
	netPath->ieee8021Pcb = rawavb_new(ETHTYPE_8021BPDU); //0x8100
	if (NULL == netPath->ieee8021Pcb){
			ERROR("netInit: Failed to open 802.1 PCB\n");
			lw_udp_remove(netPath->generalPcb);
			lw_udp_remove(netPath->eventPcb);
			return FALSE;
	}
	//== Establish the appropriate raw bindings/connections for BPDU
	rawavb_recv(netPath->ieee8021Pcb, netRecvBpduCallback, netPath);
	rawavb_bind(netPath->ieee8021Pcb, IFTYPE_IEEE8021BPDU, netif_default);
#endif

	return TRUE;

fail04:
	lw_udp_remove(netPath->generalPcb);
fail03:
	lw_udp_remove(netPath->eventPcb);
fail02:
fail01:
	return FALSE;
}

/* Shut down  the UDP and network stuff */
bool netShutdown(NetPath *netPath)
{
	struct ip_addr multicastAaddr;

	DBG("netShutdown\r\n");

	/* leave multicast group */
	multicastAaddr.addr = netPath->multicastAddr;
	igmp_leavegroup(IP_ADDR_ANY, &multicastAaddr);

	/* Disconnect and close the Event UDP interface */
	if (netPath->eventPcb)
	{
		lw_udp_disconnect(netPath->eventPcb);
		lw_udp_remove(netPath->eventPcb);
		netPath->eventPcb = NULL;
	}

	/* Disconnect and close the General UDP interface */
	if (netPath->generalPcb)
	{
		lw_udp_disconnect(netPath->generalPcb);
		lw_udp_remove(netPath->generalPcb);
		netPath->generalPcb = NULL;
	}

	// Disconnect and close the ieee8021 interface
	if(netPath->ieee8021Pcb)
	{
	   rawavb_remove(netPath->ieee8021Pcb);
	}

	/* Clear the network addresses. */
	netPath->multicastAddr = 0;
	netPath->unicastAddr = 0;

	/* Return a success code. */
	return TRUE;
}

static ssize_t netSend(const octet_t *buf, int16_t  length, TimeInternal *time, const int32_t * addr, struct udp_pcb * pcb)
{
	err_t result;
	struct pbuf * p;

	/* Allocate the tx pbuf based on the current size. */
	p = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
	if (NULL == p)
	{
		ERROR("netSend: Failed to allocate Tx Buffer\r\n");
		goto fail01;
	}

	/* Copy the incoming data into the pbuf payload. */
	result = pbuf_take(p, buf, length);
	if (ERR_OK != result)
	{
		ERROR("netSend: Failed to copy data to Pbuf (%d)\r\n", result);
		goto fail02;
	}

	/* send the buffer. */
	result = lw_udp_sendto(pcb, p, (void *)addr, pcb->local_port);
	if (ERR_OK != result)
	{
		ERROR("netSend: Failed to send data (%d)\r\n", result);
		goto fail02;
	}

	if (time != NULL)
	{
#if USE_LWIP_PTP
		time->seconds = p->time_sec;
		time->nanoseconds = p->time_nsec;
#else
		/* TODO: use of loopback mode */
		/*
		time->seconds = 0;
		time->nanoseconds = 0;
		*/
		getTimeFromReg(time);
#endif
		DBGV("netSend: %d.%09d sec\r\n", time->seconds, time->nanoseconds);
	} else {
		DBGV("netSend\r\n");
	}


fail02:
	pbuf_free(p);

fail01:
	return length;

	/*  return (0 == result) ? length : 0; */
}


ssize_t netSendEvent(NetPath *netPath, const octet_t *buf, int16_t  length, TimeInternal *time)
{
	return netSend(buf, length, time, &netPath->multicastAddr, netPath->eventPcb);
}

ssize_t netSendGeneral(NetPath *netPath, const octet_t *buf, int16_t  length)
{
	return netSend(buf, length, NULL, &netPath->multicastAddr, netPath->generalPcb);
}

ssize_t netSendPeerGeneral(NetPath *netPath, const octet_t *buf, int16_t  length)
{
	return netSend(buf, length, NULL, &netPath->peerMulticastAddr, netPath->generalPcb);
}

ssize_t netSendPeerEvent(NetPath *netPath, const octet_t *buf, int16_t  length, TimeInternal* time)
{
	return netSend(buf, length, time, &netPath->peerMulticastAddr, netPath->eventPcb);
}



#elif (PTP_PROTOCOL == IEEE8021AS)
//============= 802.1as ==================
/* Start  all of the UDP stuff */
bool netInit(NetPath *netPath, PtpClock *ptpClock)
{
	struct in_addr netAddr;
	struct ip_addr interfaceAddr;
	char addrStr[NET_ADDRESS_LENGTH];


	/* Find a network interface */
	interfaceAddr.addr = findIface(ptpClock->rtOpts->ifaceName, ptpClock->portUuidField, netPath);
	if (!(interfaceAddr.addr)){
			ERROR("netInit: Failed to find interface address\r\n");
			goto fail01;
	}

	// (1) Open lwIP raw udp interfaces for the event port(319).
	netPath->eventPcb = lw_udp_new();
	if (NULL == netPath->eventPcb){
			ERROR("netInit: Failed to open Event UDP PCB\n");
			goto fail02;
	}
#if IP_SOF_BROADCAST //YOON 2016.9.30
	netPath->eventPcb->so_options |= SOF_BROADCAST;
#endif /* IP_SOF_BROADCAST */

	// (2) Open lwIP raw udp interfaces for the general port(320).
	netPath->generalPcb = lw_udp_new();
	if (NULL == netPath->generalPcb){
			ERROR("netInit: Failed to open General UDP PCB\n");
			goto fail03;
	}
#if IP_SOF_BROADCAST //YOON 2016.9.30
	netPath->generalPcb->so_options |= SOF_BROADCAST;
#endif /* IP_SOF_BROADCAST */

	// (3) Open 8021 raw interfaces for the rstp.
	netPath->ieee8021Pcb = rawavb_new(ETHTYPE_8021BPDU); //0x8100
	if (NULL == netPath->ieee8021Pcb){
			ERROR("netInit: Failed to open 802.1 PCB\n");
			goto fail04;
	}

	/* Configure network (broadcast/unicast) addresses. */
	netPath->unicastAddr = 0; /* disable unicast */

	/* Init General multicast IP address */
	memcpy(addrStr, DEFAULT_PTP_DOMAIN_ADDRESS, NET_ADDRESS_LENGTH); //224.0.1.129
	if (!inet_aton(addrStr, &netAddr)){
			ERROR("netInit: failed to encode multi-cast address: %s\n", addrStr);
			goto fail05;
	}
	netPath->multicastAddr = netAddr.s_addr;

	/* Join multicast group (for receiving) on specified interface for General multicast IP address */
	igmp_joingroup(&interfaceAddr, (struct ip_addr *)&netAddr);

	/* (2) Init Peer multicast IP address for Event multicast IP address */
	memcpy(addrStr, PEER_PTP_DOMAIN_ADDRESS, NET_ADDRESS_LENGTH);
	if (!inet_aton(addrStr, &netAddr)){
			ERROR("netInit: failed to encode peer multi-cast address: %s\n", addrStr);
			goto fail05;
	}
	netPath->peerMulticastAddr = netAddr.s_addr;

	/* Join peer multicast group (for receiving) on specified interface */
	igmp_joingroup(&interfaceAddr, (struct ip_addr *) &netAddr);

	/* Multicast send only on specified interface. */
	netPath->eventPcb->multicast_ip.addr = netPath->multicastAddr;
	netPath->generalPcb->multicast_ip.addr = netPath->multicastAddr;

	//==Establish the appropriate UDP bindings/connections for events.
	lw_udp_recv(netPath->eventPcb, netRecvEventCallback, netPath);
	lw_udp_bind(netPath->eventPcb, IP_ADDR_ANY, PTP_EVENT_PORT);
	//udp_connect(netPath->eventPcb, &netAddr, PTP_EVENT_PORT);  //ADD YOON
	*(Integer16*)ptpClock->event_port_address = PTP_EVENT_PORT;//ADD YOON

	//== Establish the appropriate UDP bindings/connections for general.
	lw_udp_recv(netPath->generalPcb, netRecvGeneralCallback, netPath);
	lw_udp_bind(netPath->generalPcb, IP_ADDR_ANY, PTP_GENERAL_PORT);
	//udp_connect(netPath->generalPcb, &netAddr, PTP_GENERAL_PORT); //ADD YOON
	*(Integer16*)ptpClock->general_port_address = PTP_GENERAL_PORT;//ADD YOON

#if (PROJ_FOR == PROJ_FOR_RSTP)
	//== Establish the appropriate raw bindings/connections for BPDU
	rawavb_recv(netPath->ieee8021Pcb, netRecvBpduCallback, netPath);
	rawavb_bind(netPath->ieee8021Pcb, IFTYPE_IEEE8021BPDU, netif_default);
#endif

	return TRUE;
fail05:
	rawavb_remove(netPath->ieee8021Pcb);
fail04:
	lw_udp_remove(netPath->generalPcb);
fail03:
	lw_udp_remove(netPath->eventPcb);
fail02:
fail01:
	return FALSE;
}
/* Processing an incoming message on the Event port. */
static void netRecvEventCallback8021(void *arg, struct avb_pcb *pcb, struct pbuf *p,
  struct eth_addr *addr, u16_t etype)
{
	NetPath *netPath = (NetPath *) arg;

	/* Place the incoming message on the Event Port QUEUE. */
	if (!netQPut(&netPath->eventQ, p))
	{
		pbuf_free(p);
		ERROR("netRecvEventCallback: queue full\n");
		return;
	}
	/* Alert the PTP thread there is now something to do. */
	ptpd_alert();

}

/* Process an incoming message on the General port. */
static void netRecvGeneralCallback8021(void *arg, struct avb_pcb *pcb, struct pbuf *p,
	struct eth_addr *addr, u16_t etype)
{
	NetPath *netPath = (NetPath *) arg;

	/* Place the incoming message on the Event Port QUEUE. */
	if (!netQPut(&netPath->generalQ, p))
	{
		pbuf_free(p);
		ERROR("netRecvGeneralCallback: queue full\n");
		return;
	}

	/* Alert the PTP thread there is now something to do. */
	//ptpd_alert();
}

bool netInit8021(NetPath *netPath, PtpClock *ptpClock)
{
  int i;
  struct in_addr netAddr;
  struct ip_addr interfaceAddr;
  char addrStr[NET_ADDRESS_LENGTH];

  // Initialize the buffer queues.
  netQInit(&netPath->eventQ, 319);
  netQInit(&netPath->generalQ, 320);

  // Find a network interface
  findIface(ptpClock->rtOpts->ifaceName, ptpClock->portUuidField, netPath);
  // Open lwIP raw avb interfaces for the event port.
	netPath->eventPcb = rawavb_new(ETHTYPE_8021AS);//,IEEE8021AS_EVENTTYPE, netif_default);
	if (NULL == netPath->eventPcb)
	{
			ERROR("netInit: Failed to open Event raw avb PCB\n");
			goto fail02;
	}
	// Open lwIP raw avb interfaces for the general port. */
	netPath->generalPcb = rawavb_new(ETHTYPE_8021AS);// ,IEEE8021AS_GENERALTYPE, netif_default);
	if (NULL == netPath->generalPcb){
			ERROR("netInit: Failed to open General raw avb PCB\n");
			goto fail03;
	}

	//netPath->unicastAddrAvb = 0; //disable unicast
  /*
	if(!lookupSubdomainAddress(rtOpts->subdomainName, addrStr)) {
		rawavb_remove(netPath->eventPcb);
		rawavb_remove(netPath->generalPcb);
		pbuf_free(netPath->eventTxBuf);
		pbuf_free(netPath->generalTxBuf);
		return FALSE;
	}
	*/

	//netPath->multicastAddrAvb = netAddr.s_addr;
	// Setup subdomain address string.
	//for(i = 0; i < SUBDOMAIN_ADDRESS_LENGTH; ++i) {
	//	ptpClock->subdomain_address[i] = (netAddr.s_addr >> (i * 8)) & 0xff;
	//}

	// Register the appropriate avb bindings/connections for the event PTP packet.
	rawavb_recv(netPath->eventPcb, netRecvEventCallback8021, netPath); //Set a receive callback for the 802.1AS PCB
	rawavb_bind(netPath->eventPcb, IFTYPE_IEEE8021AS_EVENT, netif_default);//PTP_EVENT_PORT); //TO BE MODIFIED
	//rawavb_connect(netPath->eventPcb, IP_ADDR_ANY, PTP_EVENT_PORT);
  //lw_udp_bind(netPath->eventPcb, IP_ADDR_ANY, PTP_EVENT_PORT);
  //lw_udp_connect(netPath->eventPcb, IP_ADDR_ANY, PTP_EVENT_PORT);
	//*(Integer16*)ptpClock->event_port_address = 0x88f7;//PTP_EVENT_PORT; //DNI

	//Register the appropriate UDP bindings/connections for the general PTP packet.
	rawavb_recv(netPath->generalPcb, netRecvGeneralCallback8021, netPath);
	rawavb_bind(netPath->generalPcb, IFTYPE_IEEE8021AS_GENERAL, netif_default);//PTP_EVENT_PORT);//TO BE MODIFIED
	//rawavb_connect(netPath->generalPcb, IP_ADDR_ANY, PTP_EVENT_PORT);
	//lw_udp_bind(netPath->generalPcb, IP_ADDR_ANY, PTP_GENERAL_PORT);
	//lw_udp_connect(netPath->generalPcb, IP_ADDR_ANY, PTP_GENERAL_PORT);
	//*(Integer16*)ptpClock->general_port_address = 0x88f7;//PTP_GENERAL_PORT;//DNI

	/* Return a success code. */
	return TRUE;
fail04:
		rawavb_remove(netPath->generalPcb);
fail03:
		rawavb_remove(netPath->eventPcb);
fail02:
fail01:
	return FALSE;

}

/* shut down the UDP stuff */
Boolean netShutdown8021as(NetPath *netPath)
{
  /* Disconnect and close the Event interface */
  if(netPath->eventPcb)
  {
    //udp_disconnect(netPath->eventPcb);
	  rawavb_remove(netPath->eventPcb);
  }

  /* Disconnect and close the General interface */
  if(netPath->generalPcb)
  {
    //udp_disconnect(netPath->generalPcb);
    rawavb_remove(netPath->generalPcb);
  }

  // Free up the Event and General Tx PBUFs.
/*  if(netPath->eventTxBuf) {
    pbuf_free(netPath->eventTxBuf);
  }
  if(netPath->generalTxBuf)  {
    pbuf_free(netPath->generalTxBuf);
  }
*/
  /* Clear the network addresses. */
  netPath->multicastAddr = 0;
  netPath->unicastAddr = 0;

  /* Return a success code. */
  return TRUE;
}

static ssize_t netSend8021as(const octet_t *buf, int16_t  length,
		TimeInternal *time, const int32_t * addr,
		struct rawavb_pcb * pcb)
{
	err_t result;
	struct pbuf *p;

	// Allocate the tx pbuf based on the current size.
	p = pbuf_alloc(PBUF_LINK, length, PBUF_RAM);
	//p = (struct rawavb_pcb *)memp_malloc(MEMP_RAW_PCB);
	if (NULL == p){
		ERROR("netSend: Failed to allocate Tx Buffer\r\n");
		goto fail01;
	}

	// Copy the incoming data into the pbuf payload.
	result = pbuf_take(p, buf, length);
	if (ERR_OK != result){
		ERROR("netSend: Failed to copy data to Pbuf (%d)\r\n", result);
		goto fail02;
	}
	// move pointer for Ethernet header
	if (pbuf_header(p, sizeof(struct eth_hdr)) != 0) {
		  printf("ieee8023raw_output(): pbuf_header Error\r\n");
		  return ERR_BUF;
	}
	//pbuf_header(pcopy, -PBUF_LINK_HLEN); //YOON
	//pbuf_header(pcopy, PBUF_LINK_HLEN); //YOON
	// send the buffer.
	//result = lwLink_sendto(pcb, p, (void *)addr, pcb->etype);
	result = lwLink_sendto(pcb, p);//, pcb->etype);
	if (ERR_OK != result)
	{
		ERROR("netSend: Failed to send data (%d)\r\n", result);
		goto fail02;
	}

	if (time != NULL)
	{
#if USE_LWIP_PTP
		time->seconds = p->time_sec;
		time->nanoseconds = p->time_nsec;
#else
		/* TODO: use of loopback mode */
		/*
		time->seconds = 0;
		time->nanoseconds = 0;
		*/
		getTimeFromReg(time);
#endif
		DBGV("netSend: %d.%09d sec\r\n", time->seconds, time->nanoseconds);
	} else {
		DBGV("netSend\r\n");
	}
fail02:
	pbuf_free(p);

fail01:
	return length;
}
/*
// Transmit a packet on the General Port.
size_t netSendGeneral8021as(NetPath *netPath, Octet *buf, UInteger16 length) //buf = ptpClock->msgObuf (without MAC)
{
  int i, j;
  struct pbuf *pcopy;
  struct pbuf *tmp;
  struct pbuf *ypbuf; //

  //tmp = ( struct pbuf *)netPath->generalTxBuf; //YOON

  // Reallocate the tx pbuf based on the current size.
  //if(length>60) //YOON
  //pbuf_realloc(netPath->generalTxBuf, length+2); //ORG

  //tmp->tot_len = length; //YOON
  //tmp->len = length; //YOON
  pcopy = (struct pbuf *)(netPath->generalPcb); //pcopy = (struct pbuf *)(netPath->generalTxBuf);
  pcopy->tot_len = length;
  pbuf_header(pcopy, -PBUF_LINK_HLEN); //YOON
  // Copy the the buf into the generalTxBuf payload.
  j = 0;
  for(i = 0; i < length; i++)
  {
    ((u8_t *)pcopy->payload)[j++] = buf[i];

    if(j == pcopy->len)
    {
      pcopy = pcopy->next;
      j = 0;
    }
    //UARTprintf("[%d]=%02x ",i,((u8_t *)pcopy->payload)[i]);
  }
  //pbuf_header(netPath->generalTxBuf, PBUF_LINK_HLEN); //YOON init for next use
  // send the buffer.
  pbuf_header(pcopy, PBUF_LINK_HLEN); //YOON
  lwLink_sendto(netPath->generalPcb,pcopy);// netPath->generalTxBuf);//,  (void *)&netPath->multicastAddr, PTP_GENERAL_PORT);

  return(length);

}
*/
/* Transmit a packet on the Event Port. */
size_t netSendEvent8021as(NetPath *netPath, const octet_t *buf, int16_t  length, TimeInternal *time)
{
	return netSend8021as(buf, length, time, &netPath->multicastAddr, netPath->eventPcb);
}

ssize_t netSendGeneral8021as(NetPath *netPath, const octet_t *buf, int16_t  length)
{
	return netSend8021as(buf, length, NULL, &netPath->multicastAddr, netPath->generalPcb);
}

ssize_t netSendPeerGeneral8021as(NetPath *netPath, const octet_t *buf, int16_t  length)
{
	return netSend8021as(buf, length, NULL, &netPath->peerMulticastAddr, netPath->generalPcb);
}

ssize_t netSendPeerEvent8021as(NetPath *netPath, const octet_t *buf, int16_t  length, TimeInternal* time)
{
	return netSend8021as(buf, length, time, &netPath->peerMulticastAddr, netPath->eventPcb);
}
#endif
#endif //(PROJ_FOR == PROJ_FOR_PTP)
#endif //USE_LWIP
