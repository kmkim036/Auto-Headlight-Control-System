/*
 * [1] include paths
 *  Go to Build -> Configuration, and add library path to c:\Program Files\CodeSourcery\Sourcery G++ Lite\arm-none-eabi\include
 *   and add library path to c:\Program Files\CodeSourcery\Sourcery G++ Lite\lib\gcc\arm-none-eabi\4.5.2\include     ?�것?�?stdarg.h ?�문??추�???
 * ChongHo Yoon, KAU, 2011
 * Fix bug in void handleDelayReq() in protocol.c
 * Fix bug in void handleDelayResp() in protocol.c
 *
 * Option Settings : opt.h overrides all options of lwipopts.h
 */

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"

#include "ethopts.h"
#include "yInc.h"
//#include "lwip/include/udp.h"
//#include "lwip/include/def.h"
#include "lwip/include/memp.h"
#include "raw.h"

#if (PROJ_FOR == PROJ_FOR_PTP) && (PTP_PROTOCOL == IEEE8021AS)
#include "ptpstm/include/ptpd.h"

//#if (PTP_PROTOCOL == IEEE8021AS)
extern uint32_t g_ptpLocalTime_in_msec ; //extern __IO uint32_t g_ptpLocalTime_in_msec ;
extern err_t etharp_input(struct pbuf *p, struct netif *netif);

//*****************************************************************************
// Statically allocated runtime options and parameters for PTPd.
//*****************************************************************************
extern RunTimeOpts rtOpts;
extern PtpClock ptpClock;
extern ForeignMasterRecord ptpForeignRecords[DEFAULT_MAX_FOREIGN_RECORDS];
extern struct netif g_netif;

err_t lwLink_sendto(struct rawavb_pcb *pcb, struct pbuf *p);
unsigned char rawavb_input(struct pbuf *p, struct netif *inp, unsigned short etype);


unsigned long g_ulFlags=0;
unsigned char g_x =0;
unsigned char  Multicast8021as[6]={0x01,0x80,0xc2,0x00,0x00,0x0e};//LLDP ....was {0x01,0x00,0x5e,0x00,0x01,0x81};
struct rawavb_pcb *rawavb_pcbs = NULL;

#define ETHTYPE_8021AS 0x88f7

//===YOON
err_t as_input(struct pbuf *p, //p = removed Eth Header
		struct netif *inp)
{
  struct netif *netif;

  if(!netif_is_up(inp)){
		pbuf_free(p);
		return ERR_OK;
  }
  // send to upper layers
  if(!rawavb_input(p, inp, ETHTYPE_8021AS)) //If not 88F7, the frame did not ate by the upper layer. Thus we should eat the frame.
    pbuf_free(p);

  return ERR_OK;
}

void lwIP_Init_8021as(unsigned char macaddr){

	  // Initializes the dynamic memory heap defined by MEM_SIZE.
	  mem_init();
	  // Initializes the memory pools defined by MEMP_NUM_x.
	  memp_init();

	#ifdef USE_LCD

	   sprintf((char*)iptxt, "  %d.%d.%d.%d", iptab[3], iptab[2], iptab[1], iptab[0]);

	   LCD_DisplayStringLine(Line8, (uint8_t*)"  Static IP address   ");
	   LCD_DisplayStringLine(Line9, iptxt);
	#endif

	  /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
	            struct ip_addr *netmask, struct ip_addr *gw,
	            void *state, err_t (* init)(struct netif *netif),
	            err_t (* input)(struct pbuf *p, struct netif *netif))

	   Adds your network interface to the netif_list. Allocate a struct
	  netif and pass a pointer to this structure as the first argument.
	  Give pointers to cleared ip_addr structures when using DHCP,
	  or fill them with sane numbers otherwise. The state pointer may be NULL.

	  The init function pointer must point to a initialization function for
	  your ethernet netif interface. The following code illustrates it's use.*/
	  //netif_add(&g_netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &etharp_input);
	  netif_add(&g_netif, NULL, NULL, NULL, NULL, &ethernetif_init, &etharp_input);


	  /*  Registers the default network interface.*/
	  netif_set_default(&g_netif);

	  /*  When the netif is fully configured this function must be called.*/
	  netif_set_up(&g_netif);

	  printf("AS:lwip Init Done.\r\n");
}

void ieee8021as_thread()
{
	TimeInternal currentTod;

	//(1) Initialize run-time options to default values.
	rtOpts.slaveOnly = 0;//SLAVE_ONLY; //-----------------------------------YOON
	//=== SET DEFAULT CONFIGURATION ====
	//PROTOCOL
	rtOpts.announceInterval = 	DEFAULT_ANNOUNCE_INTERVAL; //1 --> 2sec
	rtOpts.syncInterval = 		DEFAULT_SYNC_INTERVAL;//0=1sec...-3=125msec;//DEFAULT_SYNC_INTERVAL; //-3= 125msec; 0 = 1sec
	rtOpts.logMinPdelayReqInterval = DEFAULT_PDELAYREQ_INTERVAL;
	//rtOpts.initial_delayreq = DEFAULT_DELAYREQ_INTERVAL;
	rtOpts.logMinDelayReqInterval = DEFAULT_DELAYREQ_INTERVAL;
	//rtOpts.autoDelayReqInterval = TRUE;
	rtOpts.masterRefreshInterval = 60;
	rtOpts.syncSequenceChecking = FALSE;// by default we don't check Sync message sequence continuity
	rtOpts.announceReceiptTimeout  = DEFAULT_ANNOUNCE_RECEIPT_TIMEOUT;

	rtOpts.delayMechanism = P2P;//DEFAULT_DELAY_MECHANISM; //E2E or P2P

	// maximum values for unicast negotiation
    rtOpts.logMaxPdelayReqInterval = 5;
	rtOpts.logMaxDelayReqInterval = 5;
	rtOpts.logMaxSyncInterval = 5;
	rtOpts.logMaxAnnounceInterval = 5;

	//Network
	rtOpts.portNumber = 1;// NUMBER_PORTS;
	rtOpts.portDisabled	 = FALSE; //Enable
	rtOpts.transport = IEEE_802_3; //UDP_IPV4;
	rtOpts.ipMode = IPMODE_MULTICAST;
	rtOpts.dot1AS = TRUE; //FALSE;
	rtOpts.disableUdpChecksums = TRUE;
	rtOpts.inboundLatency.nanoseconds = 1000; //DEFAULT_INBOUND_LATENCY; //7700ns
	rtOpts.outboundLatency.nanoseconds = 1000;//DEFAULT_OUTBOUND_LATENCY;//7700ns
	rtOpts.ofmShift.seconds = 0;
	rtOpts.ofmShift.nanoseconds = 250; //YOON.- IF YOU NEED SOME OFFSET AS A SLAVE, SET IT in NSEC.

	//Servo
	rtOpts.servo.noAdjust = FALSE; //rtOpts.servo.noAdjust = NO_ADJUST; //FALSE --> DO ADJUST.
	rtOpts.servo.noResetClock = FALSE;//DEFAULT_NO_RESET_CLOCK; ////FALSE --> CAN RESET.
	rtOpts.servo.sDelay = DEFAULT_DELAY_S; //6
	rtOpts.servo.sOffset = DEFAULT_OFFSET_S; //1
	rtOpts.servo.ap = DEFAULT_AP; //2
	rtOpts.servo.ai = DEFAULT_AI; //16
	rtOpts.servo.servoMaxPpb = ADJ_FREQ_MAX / 1000;// ADJ_FREQ_MAX by default
	rtOpts.servo.stepOnce	 = FALSE;
	rtOpts.servo.stepForce	 = FALSE;
	rtOpts.servo.noResetClock     = DEFAULT_NO_RESET_CLOCK; //"Do not step the clock - only slew"

	rtOpts.clockUpdateTimeout = 0;//If set to non-zero, timeout in seconds, after which the slave resets if no clock updates made.

	//BMC
	rtOpts.anyDomain = FALSE;
	rtOpts.clockQuality.clockAccuracy = DEFAULT_CLOCK_ACCURACY;
#if (ETHIF_ROLE == PTP_GRANDMASTERCLOCK)
	rtOpts.clockQuality.clockClass = DEFAULT_CLOCK_CLASS-1;
#else
	rtOpts.clockQuality.clockClass = DEFAULT_CLOCK_CLASS+1;
#endif
	rtOpts.clockQuality.offsetScaledLogVariance = DEFAULT_CLOCK_VARIANCE;//==5000 /* 7.6.3.3 */ //???? YOON //
	rtOpts.priority1 = DEFAULT_PRIORITY1; //248
	rtOpts.priority2 = DEFAULT_PRIORITY2; //248
	rtOpts.domainNumber = DEFAULT_DOMAIN_NUMBER; //==0
	rtOpts.maxForeignRecords = sizeof(ptpForeignRecords) / sizeof(ptpForeignRecords[0]);

	// timePropertiesDS
	rtOpts.timeProperties.currentUtcOffsetValid = DEFAULT_UTC_VALID;
	rtOpts.timeProperties.currentUtcOffset = DEFAULT_UTC_OFFSET;//rtOpts.currentUtcOffset = DEFAULT_UTC_OFFSET;
	rtOpts.timeProperties.timeSource = INTERNAL_OSCILLATOR;
	rtOpts.timeProperties.timeTraceable = FALSE;
	rtOpts.timeProperties.frequencyTraceable = FALSE;
	rtOpts.timeProperties.ptpTimescale = TRUE;

	// currentUtcOffsetValid compatibility flags
	rtOpts.alwaysRespectUtcOffset = TRUE;
	rtOpts.preferUtcValid = FALSE;
	rtOpts.requireUtcValid = FALSE;

	// panic mode options
	rtOpts.enablePanicMode = FALSE;
	rtOpts.panicModeDuration = 2;
	rtOpts.panicModeExitThreshold = 0;

	///
	rtOpts.maxDelay = 0; //Do accept master to slave delay (Tms_offset - from Sync message) or slave to master delay (delaySM - from Delay messages)
	//if greater than this value (nanoseconds).
	//0 = not used.", RANGECHECK_RANGE,0,NANOSECONDS_MAX);

	rtOpts.maxDelayMaxRejected = 0; //Maximum number of consecutive delay measurements exceeding maxDelay threshold. 0= Not used.
	//rtOpts.stats = PTP_TEXT_STATS;


	//(2) Set run time options.
	if (ptpdStartup(&ptpClock, &rtOpts, ptpForeignRecords) != 0)
	{
		printf("PTPD: startup failed");
		return;
	}

	//(3)bring up network interface and ip/udp stack. -- YOON 2016.10.12
	if (!netInit8021(&ptpClock.netPath, &ptpClock)){
		printf("ERROR: ieee8021as_thread: failed to initialize network\r\n");
		return;
	}

	//(4) ============= Set Current Time of Day (2015.12.19. 2:20:35 GMT) ============================
	currentTod.seconds = 1450491635;
	currentTod.nanoseconds = 681000000;
	setTimeToReg(&currentTod); //call ethernetif_PTPTime_SetTime()

	//(5) Start PTP timers ============================================================================
	ptpd_timerStart(YOON_TIMER,2000,ptpClock.itimer);	//YOON

	//(6) loop ptp statemachine forever.
	while(1){
		// Process the current state.
		if(ptpClock.disabled && (ptpClock.portDS.portState != PTP_DISABLED)) {
			stm_ptpd_toState(&ptpClock, PTP_DISABLED);
		}

		if (ptpClock.portDS.portState == PTP_INITIALIZING) {
			// DO NOT shut down once started. We have to "wait intelligently", that is keep processing signals.
			// If init failed, wait for n seconds until next retry, do not exit. Wait in chunks so SIGALRM can interrupt.
		    //if(ptpClock->initFailure) {
			    //ms_sleep(10000);
			    //ptpClock->initFailureTimeout--;
		    //}

			//if(!ptpClock->initFailure || ptpClock->initFailureTimeout <= 0) {
				if(!ptp_doInit(&ptpClock)) { //after some init and transition to PTP_LISTENING state.

					printf("PTPd init failed\r\n");// - will retry in %d seconds\n", DEFAULT_FAILURE_WAITTIME);
					//writeStatusFile(ptpClock, rtOpts, TRUE);
					//ptpClock->initFailure = TRUE;
					//ptpClock->initFailureTimeout = 100 * DEFAULT_FAILURE_WAITTIME;
					//		SET_ALARM(ALRM_NETWORK_FLT, TRUE);
					//	} else {
					//		ptpClock->initFailure = FALSE;
					//		ptpClock->initFailureTimeout = 0;
					//		SET_ALARM(ALRM_NETWORK_FLT, FALSE);
					//	}
				}
		} else {
			stm_ptpd_doState(&ptpClock);
		}
		ptpd_timerUpdate(ptpClock.itimer); //YOON Added

		 // handle periodic timers for LwIP
		//LwIP_Periodic_Handle(g_ptpLocalTime_in_msec);

		//BroadR_Periodic_Handle(g_ptpLocalTime_in_msec); //check link status -- Only For BroadR-Reach
		if (ptpd_timerExpired(YOON_TIMER,ptpClock.itimer)){
			displayStats(&ptpClock);
			ptpd_timerStart(YOON_TIMER,1000,ptpClock.itimer);	//YOON
		}
	}
}

//void ieee8021as_init_and_loop(){
//	ieee8021as_thread();
//}

/**
 * Process an incoming 802.1as frame.
 *
 * Given an incoming 802.1as frame (as a chain of pbufs) this function
 * finds a corresponding RAW PCB and hands over the pbuf to the pcbs
 * recv function. If no pcb is found or the frame is incorrect, the
 * pbuf is freed.
 *
 * @param p pbuf to be demultiplexed to a RAW PCB.
 * @param inp network interface on which the frame was received.
 *
 */

unsigned char rawavb_input(
		struct pbuf *p, //skip the eth header
		struct netif *inp,
		unsigned short etype)
{
  struct ieee8021as_hdr *ashdr;
  struct rawavb_pcb *pcb, *prev;
  u8_t iftype;
  s16_t proto;
  u8_t eaten = 0;

  if(etype == ETHTYPE_8021BPDU){
	  iftype  = IFTYPE_IEEE8021BPDU;
  }else{
	  ashdr = (struct ieee8021as_hdr *)p->payload;

	  if(((ashdr->_ts_msgType) & 0x0f) < 8)
		  iftype  = IFTYPE_IEEE8021AS_EVENT;
	  else
		  iftype  = IFTYPE_IEEE8021AS_GENERAL;
  }

  	  //prev = NULL;
  	  //pcb = rawavb_pcbs;

    // Iterate through the rawavb pcb list for a matching pcb.
    /* loop through all rawavb pcbs until the packet is eaten by one */
    /* this allows multiple pcbs to match against the packet by design */
/*    while ((eaten == 0) && (pcb != NULL)) {
      if (pcb->etype == etype) {
      	if(pcb->iftype != iftype)
      		continue;
          // receive callback function available?
          if (pcb->recv != NULL) {
            // the receive callback function did not eat the packet?
            if (pcb->recv(pcb->recv_arg, pcb, p) != 0) { //call rawavb_recv()
              //receive function ate the packet
              p = NULL;
              eaten = 1;
              if (prev != NULL) {
              // move the pcb to the front of rawavb_pcbs so that is found faster next time
                prev->next = pcb->next;
                pcb->next = rawavb_pcbs;
                rawavb_pcbs = pcb;
              }
            }
          }
          // no receive callback function was set for this rawavb PCB
          // drop the packet
      }
      prev = pcb;
      pcb = pcb->next;
    }
*/
  	prev = NULL;
  	for (pcb = rawavb_pcbs; pcb != NULL; pcb = pcb->next) {
  		if((pcb->etype == etype) && (pcb->iftype == iftype)){
            //=== matched
            if (pcb->recv != NULL) {// receive callback function available?
              if (pcb->recv(pcb->recv_arg, pcb, p) != 0) { //call netRecvEventCallback8021() to store in Event or General buffers
            	  //the receive function ate the packet.
            	   eaten = 1;
            	   p = NULL;

            	  if (prev != NULL) {
            		  // move the pcb to the front of rawavb_pcbs so that is found faster next time
            		  prev->next = pcb->next;
            		  pcb->next = rawavb_pcbs;
            		  rawavb_pcbs = pcb;
            	  }
              }
            }
            //======
  		}
  	}
  	//if the receive callback function did not eat the packet, the eaten is 0.
    return eaten;
}

/**
 * Send the rawavb  packet to the address given by rawavb_connect()
 *
 * @param pcb the rawavb pcb which to send
 * @param p the IP payload to send
 *
 */
err_t
rawavb_send(struct rawavb_pcb *pcb, struct pbuf *p)
{
  return lwLink_sendto(pcb, p);//, &pcb->remote_ip);
}

/**
 * Send the rawavb IP packet to the given address. Note that actually you cannot
 * modify the IP headers (this is inconsistent with the receive callback where
 * you actually get the IP headers), you can only specify the IP payload here.
 * It requires some more changes in lwIP. (there will be a rawavb_send() function
 * then.)
 *
 * @param pcb the rawavb pcb which to send
 * @param p the IP payload to send
 * @param ipaddr the destination address of the IP packet
 *
 */
//q = pbuf + ETHER_HEADER
err_t lwLink_sendto(struct rawavb_pcb *pcb, struct pbuf *p)//ip_addr_t *ipaddr)
{
  err_t err;
  struct netif *netif;
  struct pbuf *q; /* q will be sent down the stack */
  struct eth_hdr *ethhdr;
  u16_t *plen;
  int i;
  u8_t *pt;

 // LWIP_DEBUGF(RAWRAW_DEBUG | LWIP_DBG_TRACE, ("lwLink_sendto\n"));
  DBGV("lwLink_sendto> tot_len=%d\r\n",p->tot_len);// printf("lwLink_sendto> tot_len=%d\r\n",p->tot_len);

  //assemble Ethernet Header
  pt = (u8_t *)p->payload;
  memcpy(pt, Multicast8021as,6);
  memcpy(pt+6, ptpClock.portUuidField,6);  //MyMacAddress
  *(pt+12)=0x88;
  *(pt + 13)=0xf7;

  netif = pcb->netif ;

  err = netif->linkoutput(netif, p);

  return err;
}

/**
 * Bind an rawavb PCB.
 *
 * @param pcb rawavb PCB to be bound with etype.
  * @return lwIP error code.
 * - ERR_OK. Successful. No error occured.
 * - ERR_USE. The specified ipaddr and port are already bound to by another UDP PCB.
 *
 * @see udp_disconnect()
 */
//#define MAX_RAWS 10

err_t rawavb_bind(struct rawavb_pcb *pcb, u8_t iftype, struct netif *netif)
{
	pcb->iftype = iftype; //YOON
    pcb->netif = netif; //YOON
	return ERR_OK;
}
/* Connect an rawavb PCB.
 *This will associate the UDP PCB with the remote address.
 *
 * @param pcb UDP PCB to be connected with remote address ipaddr and port.
 * @param ipaddr remote IP address to connect with.
 * @param port remote UDP port to connect with.
 *
 * @return lwIP error code
 *
 * The udp pcb is bound to a random local port if not already bound.
 *
 * @see udp_disconnect()
 */
/*
err_t rawavb_connect(struct udp_pcb *pcb, struct ip_addr *ipaddr, u16_t port)
{
  struct udp_pcb *ipcb;

  if (pcb->local_port == 0) {
    err_t err = lw_udp_bind(pcb, &pcb->local_ip, pcb->local_port);
    if (err != ERR_OK)
      return err;
  }

  ip_addr_set(&pcb->remote_ip, ipaddr);
  pcb->remote_port = port;
  pcb->flags |= UDP_FLAGS_CONNECTED;
  // TODO: this functionality belongs in upper layers
  // Insert UDP PCB into the list of active UDP PCBs.
  for (ipcb = udp_pcbs; ipcb != NULL; ipcb = ipcb->next) {
    if (pcb == ipcb) {
      // already on the list, just return
      return ERR_OK;
    }
  }
  // PCB not yet on the list, add PCB now
  pcb->next = udp_pcbs;
  udp_pcbs = pcb;
  return ERR_OK;
}
*/
/*

err_t
raw_bind(struct raw_pcb *pcb, ip_addr_t *ipaddr)
{
  ip_addr_set(&pcb->local_ip, ipaddr);
  return ERR_OK;
}


err_t
raw_connect(struct raw_pcb *pcb, ip_addr_t *ipaddr)
{
  ip_addr_set(&pcb->remote_ip, ipaddr);
  return ERR_OK;
}

 */
// Disconnect a UDP PCB
// @param pcb the udp pcb to disconnect.
/*
void rawavb_disconnect(struct rawavb_pcb *pcb)
{
  // reset remote address association
  ip_addr_set(&pcb->remote_ip, IP_ADDR_ANY);
  pcb->remote_port = 0;
  // mark PCB as unconnected
  pcb->flags &= ~UDP_FLAGS_CONNECTED;
}
*/
/**
 * Set a receive callback for a UDP PCB
 *
 * This callback will be called when receiving a datagram for the pcb.
 *
 * @param pcb the pcb for wich to set the recv callback
 * @param recv function pointer of the callback function
 * @param recv_arg additional argument to pass to the callback function
 */
void rawavb_recv(struct rawavb_pcb *pcb, rawavb_recv_fn recv, void *recv_arg) //called by netInit8021as()
//rawavb_recv(struct rawavb_pcb *pcb, void (* recv)(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port),     void *recv_arg)
{
	  /* remember recv() callback and user data */
	  pcb->recv = recv; //eventRecv8021as() or generalRecv8021as
	  pcb->recv_arg = recv_arg; //netPath
}

/**
 * Remove an rawavb PCB.
 *
 * @param pcb rawavb PCB to be removed. The PCB is removed from the list of
 * rawavb PCB's and the data structure is freed from memory.
 *
 * @see rawavb_new()
 */
void rawavb_remove(struct rawavb_pcb *pcb)
{
  struct rawavb_pcb *pcb2;
  /* pcb to be removed is first in list? */
  if (rawavb_pcbs == pcb) {
    /* make list start at 2nd pcb */
    rawavb_pcbs = rawavb_pcbs->next;
    /* pcb not 1st in list */
  } else {
    for(pcb2 = rawavb_pcbs; pcb2 != NULL; pcb2 = pcb2->next) {
      /* find pcb in rawavb_pcbs list */
      if (pcb2->next != NULL && pcb2->next == pcb) {
        /* remove pcb from list */
        pcb2->next = pcb->next;
      }
    }
  }
  memp_free(MEMP_RAW_PCB, pcb);
}

/** Create a rawavb PCB.
 * @return The rawavb PCB which was created. NULL if the PCB data structure
 * could not be allocated.
 * @see rawavb_remove()
 */
struct rawavb_pcb *rawavb_new(unsigned short etype)//, u8_t iftype, struct netif *netif)
{
  struct rawavb_pcb *pcb;
  pcb = (struct rawavb_pcb *)memp_malloc(MEMP_RAW_PCB);

  /* could allocate rawavb PCB? */
  if (pcb != NULL) {
    /* initialize PCB to all zeroes */
    memset(pcb, 0, sizeof(struct rawavb_pcb));
    pcb->etype = etype;
    //pcb->netif = netif; //YOON
    //pcb->iftype = iftype; //YOON
    pcb->next = rawavb_pcbs;

    rawavb_pcbs = pcb;
  }
  return pcb;
}


#endif

