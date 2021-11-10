/**
 * @file
 * Ethernet Interface for standalone applications (without RTOS) - works only for 
 * ethernet polling mode (polling for ethernet frame reception)
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#include "yInc.h"
#if(PROCESSOR == STM32F103C8)
#else
#include <stdint.h>
#include "lwip/include/mem.h"
#include "lwip/include/etharp.h"
#include "stm32f4xx.h"
#include "stm32f4x7_eth.h"
#include "ethernetif.h"
#include "misc.h"
#include "core_cm4.h"
#include "stm32f4x7_eth_bsp.h"
#include <string.h>
#include "ptpd.h"
#include "lwip/include/lwipopts.h"

/* Network interface name */
#define IFNAME0 'S'
#define IFNAME1 'T'

//(1)=== DMA Descriptors for Ethernet MAC in STM MCU. Ethernet Rx & Tx DMA Descriptors
extern ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB], DMATxDscrTab[ETH_TXBUFNB];
// Global pointers to track current transmit and receive descriptors
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;
// Global pointer for last received frame infos
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;


//(2) Ethernet Driver Buffers in MCU Memory
extern uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE];//(RX)
extern uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; //(TX)

//(3) One may use one of two Get Receive Frame Methods. We choose Interrupt Method.
extern FrameTypeDef ETH_Get_Received_Frame_byPollingMethod(void);
extern FrameTypeDef ETH_Get_Received_Frame_byInterrupt(void);


//================== ADD FOR PTP YOON
#if LWIP_PTP
static void ETH_PTPStart(uint32_t UpdateMethod, uint32_t pps_freq);
#endif

u32_t ETH_PTPSubSecond2NanoSecond(u32_t SubSecondValue)
{
  uint64_t val = SubSecondValue * 1000000000ll;
  val >>=31;
  return val;
}


u32_t ETH_PTPNanoSecond2SubSecond(u32_t SubSecondValue)
{
  uint64_t val = SubSecondValue * 0x80000000ll;
  val /= 1000000000;
  return val;
}

/*
 * Enhanced descriptors (enabled with EDFE=1, ETHDMABMR bit 7), must be used if time
stamping is activated (TSE=1, ETH_PTPTSCR bit 0) or if IPv4 checksum offload is
activated (IPCO=1, ETH_MACCR bit 10).

Enhanced descriptors comprise eight 32-bit words, twice the size of normal descriptors.
RDES0, RDES1, RDES2 and RDES3 have the same definitions as for normal receive
descriptors (refer to Normal Rx DMA descriptors). RDES4 contains extended status while
RDES6 and RDES7 hold the time stamp. RDES4, RDES5, RDES6 and RDES7 are defined
below.

When the Enhanced descriptor mode is selected, the software needs to allocate 32 bytes (8
DWORDS) of memory for every descriptor. When time stamping or IPv4 checksum offload
are not being used, the enhanced descriptor format may be disabled and the software can
use normal descriptors with the default size of 16 bytes.
 */
//The value in this field has the subsecond time representation, with 0.46 ns accuracy
void ETH_PTPTime_GetTime(struct ptptime_t * timestamp)
{
	int tmp_sec;
	int tmp_nsec;
	//timestamp->tv_sec = (*(__IO uint32_t *)(ETH_MAC_BASE + ETH_PTPTSHR));//ETH_GetPTPRegister(ETH_PTPTSHR);//Ethernet PTP time stamp high update register
	//timestamp->tv_nsec = (*(__IO uint32_t *)(ETH_MAC_BASE + ETH_PTPTSLR));//ETH_PTPSubSecond2NanoSecond(ETH_GetPTPRegister(ETH_PTPTSLR)); //Ethernet PTP time stamp low register
	//timestamp->tv_nsec = ETH_PTPSubSecond2NanoSecond(timestamp->tv_nsec);

	//Get Sec
	timestamp->tv_sec = ETH_GetPTPRegister(ETH_PTPTSHR);//Ethernet PTP time stamp high update register
	//Get nSec : It may consume some delay.
	timestamp->tv_nsec = ETH_PTPSubSecond2NanoSecond(ETH_GetPTPRegister(ETH_PTPTSLR)); //Ethernet PTP time stamp low register
	//Get Sec again to check whether the 1sec advance occurs during while we get the nanosec.
	tmp_sec = ETH_GetPTPRegister(ETH_PTPTSHR);
	if(timestamp->tv_sec < tmp_sec){ //If it has happened 1sec advance during get nanosec.
		//timestamp->tv_sec = tmp_sec;
		timestamp->tv_nsec = 1000000000-5; //YOON -- We regards it. Is 5 nec reasonable?
	}
}




/*
 *     #ifdef USE_ENHANCED_DMA_DESCRIPTORS
        timestamp->tv_nsec = ETH_PTPSubSecond2NanoSecond(frame->descriptor->TimeStampLow);
        timestamp->tv_sec = frame->descriptor->TimeStampHigh;
    #else
        timestamp->tv_nsec = ETH_PTPSubSecond2NanoSecond(frame->descriptor->Buffer1Addr);
        timestamp->tv_sec = frame->descriptor->Buffer2NextDescAddr;
    #endif
 */





//YOON : Modified stm32_low_level_init()----
/* In this function, the hardware should be initialized. Called from ethernetif_init().
// [REF] stm32_f4_ptpd/libraries/lwip-1.4.1/port/STM32F4x7/arch/ethernetif.c
// [NOTE] netif should be already initialized with lwip network interface structure.
 */
void stm32_low_level_init(struct netif *netif) //ORG=low_level_init()
{
  int i;

  // maximum transfer unit
  netif->mtu = 1500;

  // device capabilities : IGMP=Accept multicast traffic.
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;//| NETIF_FLAG_LINK_UP;

  // set MAC hardware address
  netif->hwaddr_len = ETHARP_HWADDR_LEN;// set MAC hardware address length
  netif->hwaddr[0] =  MAC_ADDR0;
  netif->hwaddr[1] =  MAC_ADDR1;
  netif->hwaddr[2] =  MAC_ADDR2;
  netif->hwaddr[3] =  MAC_ADDR3;
  netif->hwaddr[4] =  MAC_ADDR4;
  netif->hwaddr[5] =  MAC_ADDR5;
  ETH_MACAddressConfig(ETH_MAC_Address0, netif->hwaddr);// Initialize MAC address in ethernet MAC

  //(2)Setup MAC DMA Descriptors
  ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);// Initialize Tx Descriptors list: Chain Mode (1524 x 4 buffers)
  ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);  // Initialize Rx Descriptors list: Chain Mode  (5 buffers)

  //DMA IRQ Enable (RX ONLY)
  for( i = 0; i < ETH_RXBUFNB; i++ )  ETH_DMARxDescReceiveITConfig( &DMARxDscrTab[i], ENABLE );
  //for( i = 0; i < ETH_TXBUFNB; i++) ETH_DMATxDescTransmitITConfig( &DMATxDscrTab[i], ENABLE ); //WE DO NOT USE TX DMA INTERRUPTS

#ifdef CHECKSUM_BY_HARDWARE
  // Enable the TCP, UDP and ICMP checksum insertion for the Tx frames
  for(i=0; i<ETH_TXBUFNB; i++)  {
      ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
  }
#endif
   // Note: TCP, UDP, ICMP checksum checking for received frame are enabled in DMA config.

#if LWIP_PTP
  // Enable PTP Timestamping : WE USE FINE METHODE. -- YOON
  ETH_PTPStart(ETH_PTP_FineUpdate, YPTP_PPS_FREQ); // ETH_PTPStart(ETH_PTP_CoarseUpdate); //working faulty.
#endif

  //delayms(10);
  // Enable MAC and DMA transmission and reception
  ETH_Start();

}
/** ORIGINAL stm32_low_level_init()
 * In this function, the hardware should be initialized. Called from ethernetif_init().
 * @param netif the already initialized lwip network interface structure  for this ethernetif
 */
/*
static void stm32_low_level_init(struct netif *netif)
{
#ifdef CHECKSUM_BY_HARDWARE
  int i;
#endif
  // set MAC hardware address length
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  // set MAC hardware address
  netif->hwaddr[0] =  MAC_ADDR0;
  netif->hwaddr[1] =  MAC_ADDR1;
  netif->hwaddr[2] =  MAC_ADDR2;
  netif->hwaddr[3] =  MAC_ADDR3;
  netif->hwaddr[4] =  MAC_ADDR4;
  netif->hwaddr[5] =  MAC_ADDR5;

  // initialize MAC address in ethernet MAC
  ETH_MACAddressConfig(ETH_MAC_Address0, netif->hwaddr);

  // maximum transfer unit
  netif->mtu = 1500;

  // device capabilities : don't set NETIF_FLAG_ETHARP if this device is not an ethernet one
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

  printf("low_level_init:  init Tx/Rx Descriptors list: Chain Mode\r\n");
  // Initialize Tx Descriptors list: Chain Mode
  ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
  // Initialize Rx Descriptors list: Chain Mode
  ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

  //printf("low_level_init: enable ETH_DMA_TX,RX IRQs\r\n");
  //YOON: Enable ETH_DMATxDescTransmit IRQ
  //for( i = 0; i < ETH_TXBUFNB; i++ )  {      ETH_DMATxDescTransmitITConfig( &DMATxDscrTab[i], ENABLE );  }
  //YOON: Enable Ethernet Rx interrrupt
  for( i = 0; i < ETH_RXBUFNB; i++ )  {      ETH_DMARxDescReceiveITConfig( &DMARxDscrTab[i], ENABLE );  }

#ifdef CHECKSUM_BY_HARDWARE
  // Enable the TCP, UDP and ICMP checksum insertion for the Tx frames
  //for(i=0; i<ETH_TXBUFNB; i++)    {
  //    ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
  //}
#endif

   //Note: TCP, UDP, ICMP checksum checking for received frame are enabled in DMA config
  // Enable MAC and DMA transmission and reception
  ETH_Start();

}
*/
/**===========================================================================
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p){
  struct pbuf *q;
  err_t retval = ERR_OK;
  int framelength = 0;
#if LWIP_PTP
	ETH_TimeStamp timeStamp;
#endif
  u8 *buffer =  (u8 *)(DMATxDescToSet->Buffer1Addr);
  
  // copy frame from pbufs to MAC driver's TX buffers (in the memory) for DMA.
  for(q = p; q != NULL; q = q->next){
    memcpy((u8_t*)&buffer[framelength], q->payload, q->len);
	framelength = framelength + q->len;
  }
  // Note: padding and CRC for transmitted frame are automatically inserted by DMA
#if LWIP_PTP
		// Actual transmission of the packet with MAC DMA. The meaning of Prepare may be considered as 'Fire'.
 	  	// [NOTE]After transmitting, the DMA will return its precise timestamp.
		if (ETH_Prepare_Transmit_Descriptors_TimeStamp(framelength, &timeStamp) != ETH_SUCCESS){
			retval = ERR_IF;
		}else{
			// Fill in the time stamp information into its pbuf.
			p->time_sec = timeStamp.TimeStampHigh;
			p->time_nsec = ETH_PTPSubSecond2NanoSecond(timeStamp.TimeStampLow);
		}
#else
  // Prepare transmit descriptors to give to DMA
  if(ETH_Prepare_Transmit_Descriptors(framelength) != ETH_SUCCESS)
	{
		retval = ERR_IF;
	}
#endif
  return retval; //ERR_OK;
}
//================================== Frame Receive =====================================================
// On the ETH-IRQHandler(), we will move the rx frame(stored in RXBUF via ETH_DMA) into pbuf via memory copy.
// After checking the rx event, We use ETH_Get_Received_Frame_interrupt() to scan RX DMA Descriptor.
// Each received frame has its precise rx timestamp in frame.descriptor.
/* Should allocate a pbuf and transfer the bytes of the incoming packet from the interface into the pbuf.
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
//static
struct pbuf * low_level_input(struct netif *netif){
  struct pbuf *p = NULL, *q;
  u16_t len =0;
  int l =0;
  FrameTypeDef frame;
  u8 *buffer;
	uint32_t bufferoffset = 0;//LHD
	uint32_t payloadoffset = 0;//LHD
	uint32_t byteslefttocopy = 0;//LHD
  uint32_t i=0;
  __IO ETH_DMADESCTypeDef *DMARxDesc;//DMARxNextDesc;
  char str[10];
  
  //printf("low_level_input():\r\n");
  
  p = NULL;
  
  //We use Interrupt Method. This function does scanning of Rx descriptors to get the receive frame.
  frame = ETH_Get_Received_Frame_interrupt();//frame = ETH_Get_Received_Frame_byPollingMethod();

  len = frame.length;
  buffer = (u8 *)frame.buffer;

  if(len >0)  p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
  if(p == NULL) return NULL;

  //If we have a frame, we copy received frame to pbuf chain.
  DMARxDesc = frame.descriptor;
  bufferoffset = 0;
  for(q = p; q != NULL; q = q->next){
			byteslefttocopy = q->len;
		    payloadoffset = 0;

		    // Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size
			while( (byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE ){
				// Copy data to pbuf
				memcpy( (u8_t*)((u8_t*)q->payload + payloadoffset), (u8_t*)((u8_t*)buffer + bufferoffset), (ETH_RX_BUF_SIZE - bufferoffset));

				// Point to next descriptor
				DMARxDesc = (ETH_DMADESCTypeDef *)(DMARxDesc->Buffer2NextDescAddr);
				buffer = (unsigned char *)(DMARxDesc->Buffer1Addr);

				byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
				payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
				bufferoffset = 0;
			}

			// At last segment, we copy remaining data in pbuf
		    memcpy( (u8_t*)((u8_t*)q->payload + payloadoffset), (u8_t*)((u8_t*)buffer + bufferoffset), byteslefttocopy);
		    bufferoffset = bufferoffset + byteslefttocopy;
  }

  //We can get its precise rx timestamp.
#if LWIP_PTP
			{
				p->time_sec = frame.descriptor->TimeStampHigh;
				p->time_nsec = ETH_PTPSubSecond2NanoSecond(frame.descriptor->TimeStampLow);
			}
#endif

  // Release descriptors to the DMA Controller

  // Check if frame with multiple DMA buffer segments
  if (DMA_RX_FRAME_infos->Seg_Count > 1)  {
    DMARxDesc = DMA_RX_FRAME_infos->FS_Rx_Desc;
  }else {
    DMARxDesc = frame.descriptor;
  }

  // Set Own bit in Rx descriptors: gives the buffers back to DMA Controller
  for (i=0; i<DMA_RX_FRAME_infos->Seg_Count; i++) {
    DMARxDesc->Status = ETH_DMARxDesc_OWN;
    DMARxDesc = (ETH_DMADESCTypeDef *)(DMARxDesc->Buffer2NextDescAddr);
  }
  
  // Clear Segment_Count
  DMA_RX_FRAME_infos->Seg_Count =0;

  // When Rx Buffer unavailable flag is set: clear it and resume reception.
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET) {
    ETH->DMASR = ETH_DMASR_RBUS;// Clear RBUS ETHERNET DMA flag
    ETH->DMARPDR = 0;    // Resume DMA reception
  }
  return p;
}

/**
 * This function should be called when a packet is ready to be read from the interface.
 * It uses the above function "low_level_input()" that should handle the actual reception of bytes from the network interface.
 * Then the type of the received packet is determined and the appropriate input function is called.
 */
err_t ethernetif_input(struct netif *netif)
{
  err_t err;
  struct pbuf *p;

  // move received packet in the RX buf into a new pbuf.
  p = low_level_input(netif);

  if (p == NULL){  // no packet could be read, silently ignore this
	    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: no packet could be read\n"));
	  return ERR_MEM;
  }

  // entry point to the LwIP stack
  err = netif->input(p, netif); //jump to "ethernet_input()" in etharp.c
  
  if (err != ERR_OK) {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
    pbuf_free(p);
    p = NULL;
  }
  return err;
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));
  
  printf("ethernetif_init: add etharp_output and low_level_output. and do low_level_init()\r\n");
#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output; //assemble ethernet frame before calling the linkoutput
  netif->linkoutput = low_level_output; //actual output with Ethernet.

  /* initialize the hardware */
  stm32_low_level_init(netif); //ETH_Start() sometimes failed to start mac YOON

  //etharp_init();
  ETH_Start();// YOON: why we call it again?

  //ETH->MACCR | = (ETH_MACCR_RE | ETH_MACCR_TE);//YOON ADDED

  return ERR_OK;
}

//== ADDED FOR PTP

#if LWIP_PTP

/*******************************************************************************
* Function Name  : ETH_PTPStart
* Description    : Initialize timestamping ability of ETH
* Input          : UpdateMethod:
*                       ETH_PTP_FineUpdate   : Fine Update method
*                       ETH_PTP_CoarseUpdate : Coarse Update method
* Output         : None
* Return         : None
* MODIFIED BY YOON : support pps_freq.
*******************************************************************************/

extern void cs2300Config();

static void ETH_PTPStart(uint32_t UpdateMethod, uint32_t pps_freq)
{
  /* Check the parameters */
  assert_param(IS_ETH_PTP_UPDATE(UpdateMethod));

  //Bit 29  TSTS: Time stamp trigger status
  //This bit indicates an interrupt event in the MAC core's Time stamp generator block.
  //The software must read the MAC core’s status register, clearing its source (bit 9),
  // to reset this bit  to 0. When this bit is high an interrupt is generated if enabled.
  /* Mask the Time stamp trigger interrupt by setting bit 9 in the MACIMR register. */
  ETH_MACITConfig(ETH_MAC_IT_TST, DISABLE);

  /* Program Time stamp register bit 0 to enable time stamping. */
  ETH_PTPTimeStampCmd(ENABLE);
  ETH_PTPPPSfreqConfig(pps_freq); //YOON added
  //cs2300Config(); //YOON added

  /* Program the Subsecond increment register based on the PTP clock frequency. */
  ETH_SetPTPSubSecondIncrement(ADJ_FREQ_BASE_INCREMENT); /* to achieve 20 ns accuracy, the value is ~ 43 */

  if (UpdateMethod == ETH_PTP_FineUpdate)
	{
    /* If we are using the Fine correction method, program the Time stamp addend register
     * and set Time stamp control register bit 5 (addend register update). */
    ETH_SetPTPTimeStampAddend(ADJ_FREQ_BASE_ADDEND);
    ETH_EnablePTPTimeStampAddend();
    delayms(10);

    // Poll the Time stamp control register until bit 5 is cleared.
    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSARU) == SET); //YOON. The program may be halted here. The reason will be analyzed. YOON =====
  }

  /* To select the Fine correction method (if required), program Time stamp control register  bit 1. */
  ETH_PTPUpdateMethodConfig(UpdateMethod);

  /* Program the Time stamp high update and Time stamp low update registers with the appropriate time value. */
  ETH_SetPTPTimeStampUpdate(ETH_PTP_PositiveTime, 0, 0);

  /* Set Time stamp control register bit 2 (Time stamp init). */
  ETH_InitializePTPTimeStamp();

  /* The enhanced descriptor format is enabled and the descriptor size is increased to 32 bytes (8 DWORDS). This is required when time stamping is activated above. */
  ETH_EnhancedDescriptorCmd(ENABLE);

  /* The Time stamp counter starts operation as soon as it is initialized
   * with the value written in the Time stamp update register. */
}

/*******************************************************************************
* Function Name  : ETH_PTPTimeStampAdjFreq
* Description    : Updates time stamp addend register
* Input          : Correction value in thousandth of ppm (Adj*10^9)
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t g_prev_addend = 0; //YOON
uint32_t g_prev_Adj = 0; //YOON

void ETH_PTPTime_AdjFreq(int32_t Adj)
{
	uint32_t addend;

	/* calculate the rate by which you want to speed up or slow down the system time increments */

	/* precise */
	/*
	int64_t addend;
	addend = Adj;
	addend *= ADJ_FREQ_BASE_ADDEND;
	addend /= 1000000000-Adj;
	addend += ADJ_FREQ_BASE_ADDEND;
	*/

	/* 32bit estimation
	ADJ_LIMIT = ((1l<<63)/275/ADJ_FREQ_BASE_ADDEND) = 11258181 = 11 258 ppm*/
	//if( Adj > 5120000) Adj = 5120000; //ADJ_FREQ_MAX
	//if( Adj < -5120000) Adj = -5120000;
	if( Adj > 512000) Adj = 512000; //ADJ_FREQ_MAX
	if( Adj < -512000) Adj = -512000;

	if(abs(Adj-g_prev_Adj) > 5){
		addend = ((((275LL * Adj)>>8) * (ADJ_FREQ_BASE_ADDEND>>24))>>6) + ADJ_FREQ_BASE_ADDEND;
		// Reprogram the Time stamp addend register with new Rate value and set ETH_TPTSCR
		if(g_prev_addend != addend){ //YOON
			ETH_SetPTPTimeStampAddend((uint32_t)addend);
			ETH_EnablePTPTimeStampAddend();
		}
		g_prev_addend = addend;
		g_prev_Adj = Adj;

		if(addend < ADJ_FREQ_BASE_ADDEND )
			printf("ETH_PTPTime_AdjFreq:Adj=%d, -addend=0x%x < 0x4C19EF00:\r\n",Adj,addend);
		else if(addend > ADJ_FREQ_BASE_ADDEND )
			printf("ETH_PTPTime_AdjFreq:Adj=%d, +addend=0x%x > 0x4C19EF00:\r\n",Adj,addend);
	}

}

/*******************************************************************************
* Function Name  : ETH_PTPTimeStampUpdateOffset
* Description    : Updates time base offset. Updates the current clock by the relative difference.
* Input          : Time offset with sign
* Output         : None
* Return         : None
* [NOTE] Used for only Coarse Method.
*******************************************************************************/
void ETH_PTPTime_UpdateOffset(struct ptptime_t * timeoffset)
{
	uint32_t Sign;
	uint32_t SecondValue;
	uint32_t NanoSecondValue;
	uint32_t SubSecondValue;
	uint32_t addend;

	/* determine sign and correct Second and Nanosecond values */
	if(timeoffset->tv_sec < 0 || (timeoffset->tv_sec == 0 && timeoffset->tv_nsec < 0))
	{
		Sign = ETH_PTP_NegativeTime;
		SecondValue = -timeoffset->tv_sec;
		NanoSecondValue = -timeoffset->tv_nsec;
	}
	else
	{
		Sign = ETH_PTP_PositiveTime;
		SecondValue = timeoffset->tv_sec;
		NanoSecondValue = timeoffset->tv_nsec;
	}

	/* convert nanosecond to subseconds */
	SubSecondValue = ETH_PTPNanoSecond2SubSecond(NanoSecondValue);

	/* read old addend register value*/
	addend = ETH_GetPTPRegister(ETH_PTPTSAR);

	while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTU) == SET);
	while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTI) == SET);

	/* Write the offset (positive or negative) in the Time stamp update high and low registers. */
	ETH_SetPTPTimeStampUpdate(Sign, SecondValue, SubSecondValue);

	/* Set bit 3 (TSSTU) in the Time stamp control register. */
	ETH_EnablePTPTimeStampUpdate();// Time stamp system time update Bit in Ethernet PTP time stamp control register: ETH->PTPTSCR |= ETH_PTPTSCR_TSSTU;
    //When this bit is set, the system time is updated (added to or subtracted from) with the value
	//specified in the Time stamp high update and Time stamp low update registers. Both the
	//TSSTU and TSSTI bits must be read as zero before you can set this bit. Once the update is
	//completed in hardware, this bit is cleared.

	/* The value in the Time stamp update registers is added to or subtracted from the system */
	/* time when the TSSTU bit is cleared. */
	while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTU) == SET);

	/* Write back old addend register value. */
	ETH_SetPTPTimeStampAddend(addend);
	ETH_EnablePTPTimeStampAddend(); //set TSARU bit in ETH_PTPTSCR.

	printf("ofm=%d:%d\r\n",SecondValue,NanoSecondValue);
}

/*******************************************************************************
* Function Name  : ETH_PTPTimeStampSetTime
* Description    : Initialize time base. Set the absolute current local TOD.
* Input          : Time with sign
* Output         : None
* Return         : None
*******************************************************************************/
void ETH_PTPTime_SetTime(struct ptptime_t * timestamp)
{
	uint32_t Sign;
	uint32_t SecondValue;
	uint32_t NanoSecondValue;
	uint32_t SubSecondValue;

	/* determine sign and correct Second and Nanosecond values */
	if(timestamp->tv_sec < 0 || (timestamp->tv_sec == 0 && timestamp->tv_nsec < 0))
	{
		Sign = ETH_PTP_NegativeTime;
		SecondValue = -timestamp->tv_sec;
		NanoSecondValue = -timestamp->tv_nsec;
	}
	else
	{
		Sign = ETH_PTP_PositiveTime;
		SecondValue = timestamp->tv_sec;
		NanoSecondValue = timestamp->tv_nsec;
	}

	// convert nanosecond to subseconds
	SubSecondValue = ETH_PTPNanoSecond2SubSecond(NanoSecondValue);

	/* Write the offset (positive or negative) in the Time stamp update high and low registers. */
	ETH_SetPTPTimeStampUpdate(Sign, SecondValue, SubSecondValue);
	/* Set Time stamp control register bit 2 (Time stamp init). */
	ETH_InitializePTPTimeStamp();
	/* The Time stamp counter starts operation as soon as it is initialized
	 * with the value written in the Time stamp update register. */
	while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTI) == SET);
}

#endif /* LWIP_PTP */

#endif //PROCESSOR


