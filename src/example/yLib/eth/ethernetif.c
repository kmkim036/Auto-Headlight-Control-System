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

#include "yLib/eth/include/ethopts.h"//#include "lwip/include/lwipopts.h"
#include "yInc.h"
#include "err.h"
#if (PROCESSOR == PROCESSOR_STM32F107VCT)
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f107_eth.h"
#include "misc.h"
#include "cmdline.h"
#include "etharp.h"
#include "netif.h"
//#include "lwip/include/ip_addr.h"
//#include "netconf.h"
#include "ethernetif.h" //#include "lwip/include/ethernetif.h"
#include "stm32f1x7_eth_bsp.h"

extern unsigned char g_macaddrLsb;
//extern err_t etharp_output(struct netif *netif, struct pbuf *q, struct ip_addr *ipaddr);

//(1)=== DMA Descriptors for Ethernet MAC in STM MCU. Ethernet Rx & Tx DMA Descriptors
extern volatile ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB], DMATxDscrTab[ETH_TXBUFNB];
#if USE_LWIP_PTP
extern  volatile ETH_DMADESCTypeDef  DMAPTPRxDscrTab[ETH_RXBUFNB], DMAPTPTxDscrTab[ETH_TXBUFNB]; //107
#endif

// Global pointers to track current transmit and receive descriptors
extern ETH_DMADESCTypeDef  *gp_DMATxDescToSet;
extern volatile ETH_DMADESCTypeDef  *gp_DMARxDescToGet;

extern ETH_DMADESCTypeDef  *DMAPTPTxDescToSet; //TimeStamp Value Stored after TX
extern volatile ETH_DMADESCTypeDef  *DMAPTPRxDescToGet; //TimeStamp Value Stored after RX
// Global pointer for last received frame infos
extern volatile ETH_DMA_Rx_Frame_infos *gp_DMA_RX_FRAME_infos;

//(2) Ethernet Driver Buffers in MCU Memory
extern unsigned char Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE];//(RX)
extern unsigned char Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; //(TX)

//(3) One may use one of two Get Receive Frame Methods. We choose Interrupt Method.
extern FrameTypeDef ETH_Get_Received_Frame_byPollingMethod(void);
extern FrameTypeDef ETH_Get_Received_Frame_byInterrupt(void);


//(4) For simple PHY
#if (ETHIF_ROLE == SIMPLE_ETH)
#if defined (__GNUC__) //
uint8_t g_rstp_Rx_Buff[ETH_RX_BUF_SIZE] __attribute__ ((aligned (4))); // An Ethernet Receive Buffer
#endif
#endif

//======================= simple ==================
int rxnum = 0;

//================================================SIMPLE ETHERNET FOR EDUCATIONAL PURPOSE ==================================================================
#if (PROJ_FOR != PROJ_FOR_PTP) && (PROJ_FOR != PROJ_FOR_RSTP_PTP)
void ethernetif_low_level_init(void) //see also void stm32_low_level_init(struct netif *netif)
{
  int i;
  unsigned char hwaddr[6];

  // set MAC hardware address
  hwaddr[0] =  0x00;		//MAC_ADDR0; MSB
  hwaddr[1] =  MAC_ADDR1;
  hwaddr[2] =  MAC_ADDR2;
  hwaddr[3] =  MAC_ADDR3;
  hwaddr[4] =  MAC_ADDR4;
  hwaddr[5] =  g_macaddrLsb;// MAC_ADDR5; LSB

  //(1) initialize MAC address @ ETH_MAC_ADDR_LBASE reg.
  ETH_MACAddressConfig(ETH_MAC_Address0, hwaddr);

  //(2) Init TX/RX DMA with Chain Mode.
  //(2-1) Setup Tx Descriptors list: Chain Mode (1524 x 2 buffers)
  ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
  delayms(10);

  //(2-2) Setup Rx Descriptors list: Chain Mode  (2 buffers)
  ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
  delayms(10);

  //Enable ETH_DMATxDescTransmit IRQ -- Not USED.
/*  for( i = 0; i < ETH_TXBUFNB; i++ )  {
	  ETH_DMATxDescTransmitITConfig( &DMATxDscrTab[i], ENABLE );
  }
*/
  //Enable Ethernet Rx interrrupt
  for( i = 0; i < ETH_RXBUFNB; i++ )  {
	  ETH_DMARxDescReceiveITConfig( &DMARxDscrTab[i], ENABLE );
  }

  // Enable MAC and DMA transmission and reception with DMA
  ETH_Start();

}
#else
// In this function, the hardware should be initialized. Called from ethernetif_init().
// [NOTE] netif should be already initialized with lwip network interface structure.
//was void stm32_low_level_init(struct netif *netif) //ORG=low_level_init()
void ethernetif_low_level_init(struct netif *netif)
{
  int i;

  // maximum transfer unit
  netif->mtu = 1500;

  // device capabilities : IGMP=Accept multicast traffic.
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;//| NETIF_FLAG_LINK_UP;

  // set MAC hardware address
  netif->hwaddr_len = 6;// set MAC hardware address length
  netif->hwaddr[0] =  MAC_ADDR0;
  netif->hwaddr[1] =  MAC_ADDR1;
  netif->hwaddr[2] =  MAC_ADDR2;
  netif->hwaddr[3] =  MAC_ADDR3;
  netif->hwaddr[4] =  MAC_ADDR4;
  netif->hwaddr[5] =  g_macaddrLsb; //MAC_ADDR5;

  //(1) initialize MAC address @ ETH_MAC_ADDR_LBASE reg.
  ETH_MACAddressConfig(ETH_MAC_Address0, netif->hwaddr);
  printf("MyMAC=%02x%02x%02x:%02x%02x%02x\r\n",MAC_ADDR0,MAC_ADDR1,MAC_ADDR2,MAC_ADDR3,MAC_ADDR4,g_macaddrLsb );

  //(2) Init TX/RX DMA with Chain Mode.
  //(2-1) Setup Tx Descriptors list: Chain Mode (1524 x 2 buffers)

#if USE_LWIP_PTP
  ETH_DMAPTPTxDescChainInit(DMATxDscrTab, &DMAPTPTxDscrTab[0], &Tx_Buff[0][0], ETH_TXBUFNB);// 107.. Initialize Tx Descriptors list: Chain Mode (1524 x 4 buffers)
#else
  ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);// Initialize Tx Descriptors list: Chain Mode (1524 x 4 buffers)
#endif
  delayms(10);
  //(2-2) Setup Rx Descriptors list: Chain Mode  (2 buffers)
#if USE_LWIP_PTP
  ETH_DMAPTPRxDescChainInit(DMARxDscrTab,  DMAPTPRxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB); //107  // Initialize Rx Descriptors list: Chain Mode  (5 buffers)
#else
  ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);  // Initialize Rx Descriptors list: Chain Mode  (5 buffers)
#endif
  delayms(10);
  //(2-3) Added for PTP
  for( i = 0; i < ETH_TXBUFNB; i++) ETH_DMATxDescShortFramePaddingCmd(&DMATxDscrTab[i], ENABLE );//ADDED BY YOON.

  //(3) DMA IRQ Enable (RX ONLY)
  for( i = 0; i < ETH_RXBUFNB; i++ )
	  ETH_DMARxDescReceiveITConfig( &DMARxDscrTab[i], ENABLE );

#ifdef CHECKSUM_BY_HARDWARE
  // Enable the TCP, UDP and ICMP checksum insertion for the Tx frames
  for(i=0; i<ETH_TXBUFNB; i++)  {
      ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
  }
#endif
   // Note: TCP, UDP, ICMP checksum checking for received frame are enabled in DMA config.

#if USE_LWIP_PTP
  // Enable PTP Timestamping : WE USE FINE METHODE. -- YOON
  ethernetif_PTPStart(ETH_PTP_FineUpdate, YPTP_PPS_FREQ); // ETH_PTPStart(ETH_PTP_CoarseUpdate); //working faulty.
#endif
  //delayms(10);
  // Enable MAC and DMA transmission and reception
  ETH_Start();
}
#endif

//========== Receive ================
//#if (USE_LWIP_PTP == 0)
void ethernetif_low_level_input_VerySimple()
{
  unsigned short len;
  int l =0;
  FrameTypeDef frame;
  unsigned char *buf;
  uint32_t i=0;
  volatile ETH_DMADESCTypeDef *DMARxNextDesc;
  char str[40];

  // get received frame
  frame = ETH_Get_Received_Frame_byPollingMethod();

  len = frame.length;
  buf = (unsigned char *)frame.buffer;

  sprintf(str,"R>Type=%02x%02x(%d), %u.%u",buf[12], buf[13],rxnum, gp_DMA_RX_FRAME_infos->TimeStampHigh, gp_DMA_RX_FRAME_infos->TimeStampLow);
  printf("%s\r\n", str);
  //stmOzOLED_printString(str,0,4,16);
  rxnum++;
  // Release descriptors to DMA
  // Check if frame with multiple DMA buffer segments
  if (gp_DMA_RX_FRAME_infos->Seg_Count > 1)  {
    DMARxNextDesc = gp_DMA_RX_FRAME_infos->FS_Rx_Desc;
  }else {
    DMARxNextDesc = frame.descriptor;
  }

  // Set Own bit in Rx descriptors: gives the buffers back to DMA
  for (i=0; i<gp_DMA_RX_FRAME_infos->Seg_Count; i++) {
    DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
    DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
  }

  // Clear Segment_Count
  gp_DMA_RX_FRAME_infos->Seg_Count =0;

  // When Rx Buffer unavailable flag is set: clear it and resume reception
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET) {
    // Clear RBUS ETHERNET DMA flag
    ETH->DMASR = ETH_DMASR_RBUS;
    // Resume DMA reception
    ETH->DMARPDR = 0;
  }
}
//#elif USE_LWIP
//static
struct pbuf * ethernetif_low_level_input_withNetif(struct netif *netif)
{
  struct pbuf *p = NULL, *q;
  unsigned len =0;
  int l =0;
  FrameTypeDef frame;
  unsigned char *buffer;
  uint32_t bufferoffset = 0;//LHD
  uint32_t payloadoffset = 0;//LHD
  uint32_t byteslefttocopy = 0;//LHD
  uint32_t i=0;
  volatile ETH_DMADESCTypeDef *lpDMARxDesc;//DMARxNextDesc;
  char str[10];
  ETH_TimeStamp timeStamp;

  //printf("ethernetif_low_level_input_withNetif>\r\n");

#if USE_LWIP_PTP

  //len = ETH_HandlePTPRxPkt((uint8_t *)p->payload, &timeStamp);
  //if(len == ETH_ERROR){	  pbuf_free(p);	  p = NULL;	  return NULL;  }

  //(1) Get Frame
  //frame = ETH_Get_Received_Frame_byInterrupt();//
  frame 	= ETH_Get_Received_Frame_byPollingMethod(); //  ETH_RxPkt_ChainMode();
  len 		= frame.length; //Why the length is 8193, too big.
  buffer 	= (unsigned char *)frame.buffer;

  printf("R>Type=%02x%02x(%d)@%u.%u\r\n",buffer[12], buffer[13],rxnum, gp_DMA_RX_FRAME_infos->TimeStampHigh, gp_DMA_RX_FRAME_infos->TimeStampLow);

  //(2)
  p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL); //p = pbuf_alloc(PBUF_RAW, 1524, PBUF_POOL);
  if(p == NULL) return NULL;

  p->time_sec = gp_DMA_RX_FRAME_infos->TimeStampHigh;
  p->time_nsec = ETH_PTPSubSecond2NanoSecond(gp_DMA_RX_FRAME_infos->TimeStampLow);
  //printf("---------input> sec=%u,nsec=%u\r\n",p->time_sec,p->time_nsec );

  //(3)For the frame, we copy received frame to pbuf chain.
  lpDMARxDesc = frame.descriptor;
  bufferoffset = 0;
  for(q = p; q != NULL; q = q->next){
	  byteslefttocopy = q->len;
	  payloadoffset = 0;

	  // Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size
	  // Never happened for big rx buffer.
	  while( (byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE ){
		  // Copy data to pbuf
		  memcpy( (unsigned char*)((unsigned char*)q->payload + payloadoffset), (unsigned char*)((unsigned char*)buffer + bufferoffset), (ETH_RX_BUF_SIZE - bufferoffset));
		  // Point to next descriptor
		  lpDMARxDesc = (ETH_DMADESCTypeDef *)(lpDMARxDesc->Buffer2NextDescAddr); //TIME....for PTP
		  buffer = (unsigned char *)(lpDMARxDesc->Buffer1Addr);
		  byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
		  payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
		  bufferoffset = 0;
	  }
	  // At last segment, we copy remaining data in pbuf
	  memcpy( (unsigned char*)((unsigned char*)q->payload + payloadoffset), (unsigned char*)((unsigned char*)buffer + bufferoffset), byteslefttocopy);
	  bufferoffset = bufferoffset + byteslefttocopy;
  }

  //(4) Release descriptors to DMA
  //(4-1) Check if frame with multiple DMA buffer segments
  if (gp_DMA_RX_FRAME_infos->Seg_Count > 1)  {
    lpDMARxDesc = gp_DMA_RX_FRAME_infos->FS_Rx_Desc;
  }else {
    lpDMARxDesc = frame.descriptor;
  }

  //(4-2) Set Own bit in Rx descriptors: gives the buffers back to DMA
  for (i=0; i<gp_DMA_RX_FRAME_infos->Seg_Count; i++) {
    lpDMARxDesc->Status = ETH_DMARxDesc_OWN;
    lpDMARxDesc = (ETH_DMADESCTypeDef *)(lpDMARxDesc->Buffer2NextDescAddr);
  }

  //(4-3) Clear Segment_Count
  gp_DMA_RX_FRAME_infos->Seg_Count =0;

  //(4-4) When Rx Buffer unavailable flag is set: clear it and resume reception
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET) {
    // Clear RBUS ETHERNET DMA flag
    ETH->DMASR = ETH_DMASR_RBUS;
    // Resume DMA reception
    ETH->DMARPDR = 0;
  }


#else

  //frame = ETH_Get_Received_Frame_byInterrupt();//
  frame = ETH_Get_Received_Frame_byPollingMethod(); //  ETH_RxPkt_ChainMode();
  len = frame.length; //Why the length is 8193, too big.
  buffer = (unsigned char *)frame.buffer;

  //If we have a frame, we copy received frame to pbuf chain.
  lpDMARxDesc = frame.descriptor;
  bufferoffset = 0;
  for(q = p; q != NULL; q = q->next){
	  byteslefttocopy = q->len;
	  payloadoffset = 0;

	  // Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size
	  while( (byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE ){
		  // Copy data to pbuf
		  memcpy( (unsigned char*)((unsigned char*)q->payload + payloadoffset), (unsigned char*)((unsigned char*)buffer + bufferoffset), (ETH_RX_BUF_SIZE - bufferoffset));
		  // Point to next descriptor
		  lpDMARxDesc = (ETH_DMADESCTypeDef *)(lpDMARxDesc->Buffer2NextDescAddr); //TIME....for PTP
		  buffer = (unsigned char *)(lpDMARxDesc->Buffer1Addr);
		  byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
		  payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
		  bufferoffset = 0;
	  }
	  // At last segment, we copy remaining data in pbuf
	  memcpy( (unsigned char*)((unsigned char*)q->payload + payloadoffset), (unsigned char*)((unsigned char*)buffer + bufferoffset), byteslefttocopy);
	  bufferoffset = bufferoffset + byteslefttocopy;
  }
#endif
  //We can get its precise rx timestamp.
#if 0
	p->time_sec = frame.descriptor->Buffer2NextDescAddr;//TimeStampHigh;//407
	p->time_nsec = ETH_PTPSubSecond2NanoSecond(frame.descriptor->Buffer1Addr);//TimeStampLow);//407
	printf("sec=%u, nsec=%u\r\n", p->time_sec,p->time_nsec);


  // Release descriptors to the DMA Controller
  // Check if frame with multiple DMA buffer segments
  if (gp_DMA_RX_FRAME_infos->Seg_Count > 1)  {
	  DMARxDesc = gp_DMA_RX_FRAME_infos->FS_Rx_Desc;
  }else {
	  DMARxDesc = frame.descriptor;
  }

  // Set Own bit in Rx descriptors: gives the buffers back to DMA Controller
  for (i=0; i<gp_DMA_RX_FRAME_infos->Seg_Count; i++) {
	  DMARxDesc->Status = ETH_DMARxDesc_OWN;
	  DMARxDesc = (ETH_DMADESCTypeDef *)(DMARxDesc->Buffer2NextDescAddr);
  }

  // Clear Segment_Count
  gp_DMA_RX_FRAME_infos->Seg_Count =0;

  // When Rx Buffer unavailable flag is set: clear it and resume reception.
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
  {
	  ETH->DMASR = ETH_DMASR_RBUS;// Clear RBUS ETHERNET DMA flag
	  ETH->DMARPDR = 0;    // Resume DMA reception
  }
#endif
  return p;
}
//#endif
//============ Transmit =====================
#if (USE_LWIP_PTP == 0)
/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

err_t ethernetif_low_level_output(volatile u8 *p,int framelength)
{
	  char errval;
	  uint32_t i=0;
	  u8 *buffer =  (u8 *)(gp_DMATxDescToSet->Buffer1Addr);

      //Copy data to Tx buffer
      for(i=0;i<framelength;i++){
    	  buffer[i] = *p;
    	  p++;
      }
	  // Note: padding and CRC for transmitted frame are automatically inserted by DMA
	  // Prepare transmit descriptors to give to DMA
	  if(ETH_Prepare_Transmit_Descriptors(framelength) != ERR_OK){
		  printf("----Tx Error\r\n");
		  return -1;//ERR_ETHER;
	  }

	  errval = ERR_OK;

	  // When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission
	  if ((ETH->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET) {
	      // Clear TUS ETHERNET DMA flag
	      ETH->DMASR = ETH_DMASR_TUS;
	      // Resume DMA transmission
	      ETH->DMATPDR = 0;
	  }
	  printf("----Tx OK(len=%d)\r\n",framelength);
	  return errval;
}
#elif USE_LWIP
#if (PROCESSOR == PROCESSOR_STM32F107VCT)
extern uint32_t ETH_HandlePTPTxPkt(uint8_t *ppkt, uint16_t FrameLength, uint32_t *PTPTxTab);

// Note: padding and CRC for transmitted frame are automatically inserted by DMA
static err_t ethernetif_low_level_output(struct netif *netif, struct pbuf *p)
{
  struct pbuf *q;
  err_t retval = ERR_OK;
  int framelength = 0;
  int i;

  ETH_TimeStamp timeStamp;

  //printf("ethernetif_low_level_output> len=%u\r\n",p->len);

  //(1) Prepare DMATxDescToSet.
  gp_DMATxDescToSet->Buffer1Addr 			= DMAPTPTxDescToSet->Buffer1Addr;
  gp_DMATxDescToSet->Buffer2NextDescAddr 	= DMAPTPTxDescToSet->Buffer2NextDescAddr;

  //(2) Fill tx frame @gp_DMATxDescToSet->Buffer1Addr from pbuf.
  unsigned char *buffer1 =  (unsigned char *)(gp_DMATxDescToSet->Buffer1Addr);//== ETH_GetCurrentTxBuffer();//the address of the buffer pointed by the current descritor
  for(q = p; q != NULL; q = q->next){
	  mempcpy((unsigned char*)&buffer1[framelength], q->payload, q->len);
	  //memcpy((unsigned char*)&buffer[framelength], q->payload, q->len);
	  //bpayload = (unsigned char *)q->payload;
	  //for(i=0;i<q->len;i++)  buffer[i] = bpayload[i];
	  framelength = framelength + q->len;
  }

  //(3) Tx [NOTE]After transmitting, the DMA will return its precise timestamp.
  ETH_HandlePTPTxPkt(buffer1, framelength, &timeStamp);  //ETH_HandlePTPTxPkt((uint8_t *)p->payload, p->len, &timeStamp);

  //(4) Get timestamp
  p->time_sec = timeStamp.TimeStampHigh;
  p->time_nsec = ETH_PTPSubSecond2NanoSecond(timeStamp.TimeStampLow);
  printf("output> @(%u.%09u)\r\n",p->time_sec,p->time_nsec );

/*		if (ETH_Prepare_Transmit_Descriptors_TimeStamp(framelength, &timeStamp) != ETH_SUCCESS){
			retval = ERR_IF;
		}else{
			// Fill in the time stamp information into its pbuf.
			p->time_sec = timeStamp.TimeStampHigh;
			p->time_nsec = ETH_PTPSubSecond2NanoSecond(timeStamp.TimeStampLow);
		}
*/
  return retval; //ERR_OK;
}
#else
static err_t ethernetif_low_level_output(struct netif *netif, struct pbuf *p)
{
	printf("Not Support");
}
#endif //Processor
#endif //lwip
//=====================
#if USE_LWIP
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
  netif->linkoutput = ethernetif_low_level_output; //actual output with Ethernet.

  /* initialize the hardware */
  ethernetif_low_level_init(netif);//stm32_low_level_init(netif); //ETH_Start() sometimes failed to start mac YOON

  //etharp_init();
  ETH_Start();// YOON: why we call it again?

  //ETH->MACCR | = (ETH_MACCR_RE | ETH_MACCR_TE);//YOON ADDED

  return ERR_OK;
}

//================================== Frame Receive =====================================================
// On the ETH-IRQHandler(), we will move the rx frame(stored in RXBUF via ETH_DMA) into pbuf via memory copy.
// After checking the rx event, We use ETH_Get_Received_Frame_byInterrupt() to scan RX DMA Descriptor.
// Each received frame has its precise rx timestamp in frame.descriptor.
/* Should allocate a pbuf and transfer the bytes of the incoming packet from the interface into the pbuf.
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */

/**
 * This function should be called when a packet is ready to be read from the interface.
 * It uses the above function "ethernetif_low_level_input_withNetif()" that should handle the actual reception of bytes from the network interface.
 * Then the type of the received packet is determined and the appropriate input function is called.
 */
err_t ethernetif_input(struct netif *netif)
{
  err_t err;
  struct pbuf *p;

  // move received packet in the RX buf into a new pbuf.
  p = ethernetif_low_level_input_withNetif(netif);

  if (p == NULL){  // no packet could be read, silently ignore this
	  LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: no packet could be read\n"));
	  return ERR_MEM;
  }

  // entry point to the LwIP stack
  err = netif->input(p, netif); //jump to "etharp_input()" in etharp.c

  if (err != ERR_OK) {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
    pbuf_free(p);
    p = NULL;
  }
  return err;
}
#endif //USE_LWIP

#if (PROJ_FOR == PROJ_FOR_RSTP)
//== ADDED FOR RSTP
unsigned short ethernetif_low_level_input_withRxbuf(unsigned char *prxbuf)
{
  unsigned short len;
  int l =0;
  FrameTypeDef *frame;
  unsigned char *pbuf;
  uint32_t i=0;
  volatile ETH_DMADESCTypeDef *DMARxNextDesc;

  // get received frame
  frame = ETH_Get_Received_Frame_byInterrupt();//  was  frame = ETH_Get_Received_Frame_byPollingMethod();

  len = frame->length;
  pbuf = (unsigned char *)frame->buffer;

  memcpy(prxbuf, pbuf, len);

  printf("e>(len=%u) From=%02x%02x%02x:%02x%02x%02x - Type/Len=0x%02x%02x(%d)\r\n",len, prxbuf[6], prxbuf[7], prxbuf[8], prxbuf[9], prxbuf[10], prxbuf[11], prxbuf[12], prxbuf[13],rxnum); //Normal
/*
  if(pbuf[12] == 0x81){ //IP175D Tagging for DSA
	  if(pbuf[13] != 0x00){
		  printf("R>(len=%u) Port=%d, Len/Type=0x%02x%02x\r\n",len, (pbuf[13]&0x07)-1, pbuf[16], pbuf[17]); //IP175D DSA Tag
	  }else
		  printf("R>(len=%u) VID = %02x%02x, Type/Len=%02x%02x\r\n",len, pbuf[14], pbuf[15],pbuf[16], pbuf[17]); //VLAN tag
  }else{
	  printf("R>(len=%u) Type/Len=%02x%02x(%d)\r\n",len, pbuf[12], pbuf[13],rxnum); //Normal
  }
*/
  rxnum++;

  // Release descriptors to DMA
  // Check if frame with multiple DMA buffer segments
  if (gp_DMA_RX_FRAME_infos->Seg_Count > 1)  {
    DMARxNextDesc = gp_DMA_RX_FRAME_infos->FS_Rx_Desc;
  }else {
    DMARxNextDesc = frame->descriptor;
  }

  /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
  for (i=0; i<gp_DMA_RX_FRAME_infos->Seg_Count; i++) {
    DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
    DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
  }

  /* Clear Segment_Count */
  gp_DMA_RX_FRAME_infos->Seg_Count =0;

  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET) {
    /* Clear RBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_RBUS;
    /* Resume DMA reception */
    ETH->DMARPDR = 0;
  }
  return len;
}
#endif

#if USE_LWIP_PTP

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
extern void ETH_PTPPPSfreqConfig(uint32_t pps_freq);

void ethernetif_PTPStart(unsigned UpdateMethod, unsigned pps_freq)
{
	printf("ETH_PTPStart\r\n");

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
    delayms(100);

    // Poll the Time stamp control register until bit 5 is cleared.(TSARU = Addend Register Update)
    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSARU) == SET); //YOON. The program may be halted here. The reason will be analyzed. YOON =====
  }

  /* To select the Fine correction method (if required), program Time stamp control register  bit 1. */
  ETH_PTPUpdateMethodConfig(UpdateMethod);

  /* Program the Time stamp high update and Time stamp low update registers with the appropriate time value. */
  ETH_SetPTPTimeStampUpdate(ETH_PTP_PositiveTime, 0, 0);

  /* Set Time stamp control register bit 2 (Time stamp init). */
  ETH_InitializePTPTimeStamp();

#ifdef USE_ENHANCED_DMA_DESCRIPTORS
  /* The enhanced descriptor format is enabled and the descriptor size is increased to 32 bytes (8 DWORDS). This is required when time stamping is activated above. */
  ETH_EnhancedDescriptorCmd(ENABLE);
#endif
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

void ethernetif_PTPTime_AdjFreq(int Adj)
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
			printf("AdjFreq:Adj=%d, -addend=0x%x < 0x4C19EF00:\r\n",Adj,addend);
		else if(addend > ADJ_FREQ_BASE_ADDEND )
			printf("AdjFreq:Adj=%d, +addend=0x%x > 0x4C19EF00:\r\n",Adj,addend);
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
void ethernetif_PTPTime_UpdateOffset(struct ptptime_t * timeoffset)
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
void ethernetif_PTPTime_SetTime(struct ptptime_t * timestamp)
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
#endif //

