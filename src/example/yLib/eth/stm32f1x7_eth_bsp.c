/**
  ******************************************************************************
  * @file    stm32f1x7_eth_bsp.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011 
  * @brief   STM32F1x7 Ethernet hardware configuration.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>

  *  BROADR-Reach BCM89810 Driver
  *  PHYADDR = 0x18
  *  NORMAL MII Mode
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
 #include <stdint.h>
#include "yLib/eth/include/ethopts.h"//#include "lwip/include/lwipopts.h"
#include "yInc.h"
#if (PROCESSOR == PROCESSOR_STM32F107VCT)
#include "cmsis_boot/stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include "stm32f107_eth.h"
#include "misc.h"
#include "cmdline.h"
#include "core_cm3.h"
#include "yLib/eth/include/stm32f1x7_eth_bsp.h"
//#include "lwip/include/ip_addr.h"
//#include "netconf.h"
#include "yLib/eth/include/ethernetif.h" //#include "lwip/include/ethernetif.h"
#include "err.h"
//#include "lwip/include/lwipmain.h"

__IO uint32_t  gEthInitStatus = 0;
__IO uint8_t EthLinkStatus = 0;
__IO uint32_t EthStatus = 0; //Add YOON -- NOT USED ANY..
extern unsigned char g_ip_addr3;//

/* Private function prototypes -----------------------------------------------*/
void ETH_GPIO_Config(void);
void ETH_NVIC_Config(void);
void ETH_MACDMA_Config(unsigned short PHYAddr);
extern struct netif g_netif;

//+-----------------+-----------+-----------+---------+
//| TJA1100         | 407-M35   | 407-M36   |407VZT   |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| ULED            |PE15       | <==       | PG7
//+-----------------+-----------+-----------+-----------+---------+---------+
//| BUTTON          |PD11(index)| PD11(L)   |
//+-----------------+-----------+-----------+-----------+---------+
//| BEEP            |PD14       | <==       |
//+-----------------+-----------+-----------+-----------+---------+
//| ChipEnalbe      |PE14(DNI)  |<==
//+-----------------+-----------+-----------+-----------+---------+
//| nIRQ            |PA3        |<==
//+-----------------+-----------+-----------+-----------+---------+
//| M/S Select      |PE1        |<==
//+-----------------+-----------+-----------+-----------+---------+
//| FORCE/AN        |PE3        |<==
//+-----------------+-----------+-----------+-----------+---------+
//| MCO1            |PA8        |<==
//+-----------------+-----------+-----------+-----------+---------+

/**
  * @brief  Configures the nested vectored interrupt controller.
  */
void ETH_NVIC_Config(void)
{
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Set the Vector Table base location at 0x08000000 */
  //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_DisableIRQ(ETH_IRQn);
  // Enable the Ethernet global Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_EnableIRQ(ETH_IRQn);

}

#if (ETH_IF_MODE != ETH_IF_SPI_MODE)
void ETH_GPIO_Config(void) //void stm32f107EthInitGpio(void) //NetInterface *interface)
{

	 /**
	  * @brief GPIO configuration
	  * @param[in] interface Underlying network interface
	  **/
	 /*
	         ETH_MDIO -------------------------> PA2 +
	         ETH_MDC --------------------------> PC1 +
	         ETH_PPS_OUT ----------------------> PB5 --- STM32 PPS
	         -ETH_MII_CRS ----------------------> PA0  -- Dont cared by MAC for Full Duplex Mode.
	         -ETH_MII_COL ----------------------> PA3  -- Dont cared by MAC for Full Duplex Mode.
	         -ETH_MII_RX_ER --------------------> PB10
	         ETH_MII_RXD2 ---------------------> PB0
	         ETH_MII_RXD3 ---------------------> PB1
	         ETH_MII_TX_CLK -------------------> PC3 +
	         ETH_MII_TXD2 ---------------------> PC2 +
	         ETH_MII_TXD3 ---------------------> PB8(STM32F107) //PE2
	         ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1 +
	         ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7 +
	         ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4 +
	         ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5 +
	         ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PB11
	         ETH_MII_TXD0/ETH_RMII_TXD0 -------> PB12
	         ETH_MII_TXD1/ETH_RMII_TXD1 -------> PB13

	         -nPHYRESET ------------------------> PC8 (NOT USED)
	                                                   */
	    GPIO_InitTypeDef GPIO_InitStructure;

	    //Enable AFIO clock
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	    //Enable GPIO clocks
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
	       RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

	    //Configure MCO (PA8) as an output
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);

	#if (ETH_IF_MODE == ETH_IF_MII_MODE) //ETH_IF_RMII_MODE

	    //Configure MCO pin to output the HSE clock (25MHz)
	    RCC_MCOConfig(RCC_MCO_HSE);

	    //Select MII interface mode
	    GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_MII);

	#elif (ETH_IF_MODE == ETH_IF_RMII_MODE)
	    //Configure PLL3 to output a 50MHz clock
	    RCC_PLL3Config(RCC_PLL3Mul_10);
	    //Enable PLL3
	    RCC_PLL3Cmd(ENABLE);

	    //Wait for the PLL3 to lock
	    while(RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET);

	    //Configure MCO pin to output the PLL3 clock
	    RCC_MCOConfig(RCC_MCO_PLL3CLK);

	    //Select RMII interface mode
	    GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_RMII);
	#endif

	    //Configure MII_MDIO (PA2) - AF Output Push Pull
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);

	    //Configure MII_PPS_OUT (PB5), ETH_MII_TXD3 (PB8), MII_TX_EN (PB11),
	    //MII_TXD0 (PB12) and MII_TXD1 (PB13) - AF Output Push Pull
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_PinRemapConfig(GPIO_Remap_PTP_PPS, ENABLE);//YOON : Added for STM32F107VCT .. PB5 PPS

	    //Configure MII_MDC (PC1) and MII_TXD2 (PC2) - AF Output Push Pull
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	    GPIO_Init(GPIOC, &GPIO_InitStructure);

	    //--- INPUT --------

	    //Configure ETH_MII_TX_CLK (PC3) -- INPUT
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //GPIO_Mode_AF_PP;
	    GPIO_Init(GPIOC, &GPIO_InitStructure);

	    // RX - INPUT
	    //Configure
	    //ETH_MII_RXD0 (PC4),
	    //ETH_MII_RXD1 (PC5),
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_Init(GPIOC, &GPIO_InitStructure);

	    //ETH_MII_RXD2 (PB0)
	    //ETH_MII_RXD3 (PB1)
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);

	    //Configure
	    //ETH_MII_CRS (PA0)-- X
	    //ETH_MII_RX_CLK (PA1)
	    //ETH_MII_COL (PA3) -- X
	    //ETH_MII_RX_DV (PA7),
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_7; //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);

	    //Configure ETH_MII_RX_ER (PB10)
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);

	    //Remap Ethernet pins (No need for remap)
	    GPIO_PinRemapConfig(GPIO_Remap_ETH, DISABLE);

	    delayms(100);
}
#elif (ETH_IF_MODE == ETH_IF_SPI_MODE)
void ETH_GPIO_Config_WO_MDIOMDC(void) //For KSZ Switches... Those are controlled with not MDIO/MDC but SPI.
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE,  ENABLE);

	//Enable SysCfg Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*
	// Configure nRESETPHY (PC8). Issue nPHY_RESET
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//GPIO_ResetBits(GPIOC, GPIO_Pin_8);//SET MII mode while in RESET ????
	*/

  /* MII/RMII Media interface selection ---PTP
   * -----------------------------------*/

#if (MII_RMII == MII_MODE)
  //Select MII mode : Setting mode is allowed under reset or before enabling MAC clocks.
/*
#ifdef USE_MCO1_FOR_PHY_CLOCK_SRC
   // Output HSE clock (25MHz) on MCO pin (PA8) to clock the PHY
	MCO1_Config_25MHz();
	printf("***Using MCO1 25MHz Output ***\r\n");
#endif
*/
  //Select MII mode : Setting mode is allowed under reset or before enabling MAC clocks.
  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_MII); //0 = MII/ 1= RMII
  printf("***MII***\r\n");

#elif (MII_RMII == RMII_MODE)  /* Mode RMII with STM324xG-EVAL */
//#error using RMII_MODE !!!!!
  	  /* Output PLL clock divided by 2 (50MHz) on MCO pin (PA8) to clock the PHY */
  	  //RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_2);
  	  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);
  	printf("***RMII***\r\n");
#endif

  delayms(800);
  //GPIO_SetBits(GPIOC, GPIO_Pin_8); //nRESET_PHY TO HIGH
  //delayms(10);

/* Ethernet pins configuration (YOON)************************************************/
   /*
        ETH_MDIO -------------------------> PA2 +
        ETH_MDC --------------------------> PC1 +
        ETH_PPS_OUT ----------------------> PB5 --- STM32 PPS
        -ETH_MII_CRS ----------------------> PA0  -- Dont cared by MAC for Full Duplex Mode.
        -ETH_MII_COL ----------------------> PA3  -- Dont cared by MAC for Full Duplex Mode.
        -ETH_MII_RX_ER --------------------> PB10
        ETH_MII_RXD2 ---------------------> PB0
        ETH_MII_RXD3 ---------------------> PB1
        ETH_MII_TX_CLK -------------------> PC3 +
        ETH_MII_TXD2 ---------------------> PC2 +
        ETH_MII_TXD3 ---------------------> PE2 //PB8 +
        ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1 +
        ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7 +
        ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4 +
        ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5 +
        ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PB11
        ETH_MII_TXD0/ETH_RMII_TXD0 -------> PB12
        ETH_MII_TXD1/ETH_RMII_TXD1 -------> PB13

        -nPHYRESET ------------------------> PC8 (NOT USED)
                                                  */
  /* Enable GPIOs clocks for Ethernet */
  // Configure PA1, PA2(MDIO), and PA7
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //?? NO GPIO_OType_AF
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_7; //exclude MDIO
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2  | GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH);
  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH); //MDIO
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH);

  //Continue... Configure PB0,PB1,PB5,PB11,PB12 and PB13
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_11  | GPIO_Pin_12 |  GPIO_Pin_13;  //exclude PB5 PPS
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_ETH);

  //In addition for PPS
  ////For LAN9355, We disable PPS Function of STM32F407. The PIN PB5 will be used for PPS output with GPIO OUTPUT which is driven by LAN9355's 1588 Event Trigger.
//#if (SWITCH_ID == SWITCH_LAN9355)
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;  //PB5 PPS
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//#else
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;  //PB5 PPS
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_ETH); //PPS - The PPS output is enabled through a GPIO alternate function. (GPIO_AFR register).
//#endif

  //Continue... Configure PC1(MDC), PC2, PC3, PC4 and PC5
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; //exclude MDC
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH); //MDC
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH);

  /* Configure PE2 (for TXD3) */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource2, GPIO_AF_ETH);

  delayms(800);

}
#endif

void ETH_MACDMA_Config(unsigned short PHYAddress)
{
	ETH_InitTypeDef ETH_InitStructure;
	//After GPIO Config, we start MAC Clocking
#if USE_MCU_PTP
  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx | RCC_AHBPeriph_ETH_MAC_Rx | RCC_AHBPeriph_ETH_MAC_PTP, ENABLE); //PTP Support case
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx | RCC_AHBPeriph_ETH_MAC_Rx, ENABLE); //PTP Support case --- 107
#else
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx | RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);
#endif
  delayms(10); //should be ...

  /* Reset ETHERNET on AHB Bus */
  ETH_DeInit(); //Deinitializes the ETHERNET peripheral registers to their default reset values.

  //If not working, comment out.
  ETH_SoftwareReset(); //resets all MAC subsystem internal registers and logic
  while (ETH_GetSoftwareResetStatus() == SET); //Never return 0. YOON....

  // ETHERNET Configuration ===============
  ETH_StructInit(&ETH_InitStructure);

  // Fill several interesting ETH_InitStructure parameters
  ETH_InitStructure.ETH_AutoNegotiation 			= ETH_AutoNegotiation_Disable;//Critical - SHould be Disabled for test. ETH_AutoNegotiation_Enable;// ETH_AutoNegotiation_Disable;
  ETH_InitStructure.ETH_Speed 						= ETH_Speed_100M;
  ETH_InitStructure.ETH_InterFrameGap 				= ETH_InterFrameGap_96Bit; //Added
  ETH_InitStructure.ETH_ReceiveOwn 					= ETH_ReceiveOwn_Disable; //ETH_ReceiveOwn_Enable; //Added for 10Mbps
  ETH_InitStructure.ETH_Mode 						= ETH_Mode_FullDuplex;
  ETH_InitStructure.ETH_CarrierSense 				= ETH_CarrierSense_Disable; //YOON : VERY CRITICAL
  ETH_InitStructure.ETH_LoopbackMode 				= ETH_LoopbackMode_Disable;//ETH_LoopbackMode_Enable;// Need RXCLK for Loopback.
  ETH_InitStructure.ETH_RetryTransmission 			= ETH_RetryTransmission_Disable;
  ETH_InitStructure.ETH_AutomaticPadCRCStrip 		= ETH_AutomaticPadCRCStrip_Disable;//Enable;
  ETH_InitStructure.ETH_ReceiveAll 					= ETH_ReceiveAll_Enable; //Unicast and Multicast Frames. Promiscuous Mode?
  ETH_InitStructure.ETH_BroadcastFramesReception 	= ETH_BroadcastFramesReception_Enable;
  ETH_InitStructure.ETH_PromiscuousMode 			= ETH_PromiscuousMode_Enable;//YOON : VERY CRITICAL FOR MULTICAST
  ETH_InitStructure.ETH_MulticastFramesFilter 		= ETH_MulticastFramesFilter_None;//ETH_MulticastFramesFilter_Perfect;
  ETH_InitStructure.ETH_UnicastFramesFilter 		= ETH_UnicastFramesFilter_Perfect;
  ETH_InitStructure.ETH_VLANTagComparison 			= ETH_VLANTagComparison_16Bit;
  ETH_InitStructure.ETH_VLANTagIdentifier 			= 0x00;//NO.    0x02; //For AVB

#ifdef CHECKSUM_BY_HARDWARE
  ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
#endif

  /*------------------------   DMA   -----------------------------------*/
  /* When we use the Checksum offload feature, we need to enable the Store and Forward mode:
  the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum,
  if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
  ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Disable;//ETH_DropTCPIPChecksumErrorFrame_Enable;
  ETH_InitStructure.ETH_ReceiveStoreForward 		= ETH_ReceiveStoreForward_Disable;//ETH_ReceiveStoreForward_Enable;
  ETH_InitStructure.ETH_TransmitStoreForward 		= ETH_TransmitStoreForward_Disable;//ETH_TransmitStoreForward_Enable;
  ETH_InitStructure.ETH_ForwardErrorFrames 			= ETH_ForwardErrorFrames_Disable;
  ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Enable;//Disable;
  ETH_InitStructure.ETH_SecondFrameOperate 			= ETH_SecondFrameOperate_Enable;
  ETH_InitStructure.ETH_AddressAlignedBeats 		= ETH_AddressAlignedBeats_Enable;
  ETH_InitStructure.ETH_FixedBurst 					= ETH_FixedBurst_Enable;
  ETH_InitStructure.ETH_RxDMABurstLength 			= ETH_RxDMABurstLength_8Beat;//ETH_RxDMABurstLength_32Beat;//ETH_RxDMABurstLength_8Beat;//ETH_RxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_TxDMABurstLength 			= ETH_RxDMABurstLength_8Beat;//ETH_TxDMABurstLength_32Beat;//ETH_RxDMABurstLength_8Beat;//ETH_TxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_DMAArbitration 				= ETH_DMAArbitration_RoundRobin_RxTx_2_1;//ETH_DMAArbitration_RoundRobin_RxTx_1_1;//ETH_DMAArbitration_RoundRobin_RxTx_2_1;
  ETH_InitStructure.ETH_DescriptorSkipLength 		= 0x0;// Added
  gEthInitStatus = ETH_Init(&ETH_InitStructure, PHYAddress);//IP101A_PHY_ADDRESS);//BROADR_PHY_ADDRESS);//IP101A_PHY_ADDRESS);//was KSZ8051_PHY_ADDRESS);

  //Disable MAC Interrupt(TimeStamp and P...)
  //ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T, DISABLE); //NIS=Normal interrupt summary; IT_R=Receive interrupt; T=Tx Interrupt(YOON add)

  // Enable the the RX DMA Interrupts
  ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R , ENABLE); //NIS=Normal interrupt summary; IT_R=Receive interrupt; T=Tx Interrupt(YOON add)

  //ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T , ENABLE); //NIS=Normal interrupt summary; IT_R=Receive interrupt; T=Tx Interrupt(YOON add)
  //ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T | ETH_DMA_IT_TBU, ENABLE); //NIS=Normal interrupt summary; IT_R=Receive interrupt; T=Tx Interrupt(YOON add)

  printf("ETH_MACDMA_Config Pass\r\n");
}
//==PHY DEPENDENT ===================================================
#if (PHYCHIP == TJA1100) //and others(PHY Mode Switches)
void ETH_TJA1100_BSP_Config(void) --> Consolidated in void ETH_X_BSP_Config(unsigned char PhyAddr)
#elif (PHYCHIP == RTL9K) //and others(PHY Mode Switches)
void ETH_RTL9K_BSP_Config(void) -->  Consolidated in void ETH_X_BSP_Config(unsigned char PhyAddr)
#else //and others(PHY Mode Switches) ==============================================================================
void ETH_X_BSP_Config(unsigned char PhyAddr, char *str)
{
	int i;
	uint16_t retv;
	unsigned int result;

	printf("%s Config\r\n",str);

	//(1) GPIO and MCO1 configuration
	ETH_GPIO_Config();

	//(2) Config Ethernet MAC interrupts
	ETH_NVIC_Config();

	//(3) Configure the Ethernet peripheral and DMA
	ETH_MACDMA_Config(PhyAddr);

	printf("ETH_MACDMA_Config...ok \r\n");
	printf("EthInitStatus: %d\r\n",gEthInitStatus);
	delayms(10);

	if (gEthInitStatus == 0) {
		while(1){
	    	printf("EthInitStatus=0 --> Error\r\n"); delayms(100);
		}
	 }else
		  printf("ok\r\n");

	 //(4) Get PHY Product ID
	 // Read PHY ID register2 and 3: Should be 0x0180 and 0xdc48
	 retv = ETH_ReadPHYRegister(PhyAddr, 0x02);
	 printf("PHYID Reg2 = 0x%04x and ",retv);

	 retv = ETH_ReadPHYRegister(PhyAddr, 0x03);
	 printf("PHYID Reg3 = 0x%04x \r\n",retv);

	 // Read PHY status register: Get Ethernet link status
	 retv = ETH_ReadPHYRegister(PhyAddr, 0x01);
	 if(retv & 0x04)  { //LRE Status Register(01:2) == 1-> Link UP
		 EthLinkStatus |= ETH_LINK_FLAG;  //EthStatus |= ETH_LINK_FLAG;
	 }

	 //(5) Walk through PHY Registers with SPI or I2C
	 for(i=0;i<16;i++){
		retv = ETH_ReadPHYRegister(PhyAddr, i);
		printf("ret(%d)=0x%04x\r\n",i,retv);
		delayms(10);
	 }

	 printf("ETH_BSP_Config --> PASSED\r\n");
}

#endif // ==============================================================================
#if 0
/**
 * @brief Receive a packet
 * @param[in] interface Underlying network interface
 * @return Error code
 **/
extern int rxnum;
//ethernetif_low_level_input_VerySimple(){//
err_t stm107_EthReceivePacket()//NetInterface *interface)
{

   err_t error;
   int len;
   unsigned char *frame;

   printf("R>\r\n");


   if (ETH_CheckFrameReceived())   {// process received ethernet packet
   			printf("=====main: ETH_CheckFrameReceived\r\n");
   			//seq++;
   			//LwIP_Pkt_Handle(); //move FIFO into pbuf, and handles it.
   			ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
   			ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
   	}

/*
   //The current buffer is available for reading?
   if(!(rxCurDmaDesc->rdes0 & ETH_RDES0_OWN))
   {
      //FS and LS flags should be set
      if((rxCurDmaDesc->rdes0 & ETH_RDES0_FS) && (rxCurDmaDesc->rdes0 & ETH_RDES0_LS))
      {
         //Make sure no error occurred
         if(!(rxCurDmaDesc->rdes0 & ETH_RDES0_ES))
         {
            //Retrieve the length of the frame
            len = (rxCurDmaDesc->rdes0 & ETH_RDES0_FL) >> 16;

            //Limit the number of data to read
            len = (len > ETH_RX_BUFFER_SIZE) ? ETH_RX_BUFFER_SIZE : len;
            frame = (uint8_t *) rxCurDmaDesc->rdes2;

            //Pass the packet to the upper layer
            //nicProcessPacket(interface, (uint8_t *) rxCurDmaDesc->rdes2, n);
            printf("R>Type=%02x%02x(%d)",frame[12], frame[13],rxnum);
            //printf("%s\r\n", str);
            rxnum++;
            //Valid packet received
            error = ERR_OK;
         }
         else
         {
            //The received packet contains an error
            error = ERR_INVALID_PACKET;
         }
      }
      else
      {
         //The packet is not valid
         error = ERR_INVALID_PACKET;
      }

      //Give the ownership of the descriptor back to the DMA
      rxCurDmaDesc->rdes0 = ETH_RDES0_OWN;
      //Point to the next descriptor in the list
      rxCurDmaDesc = (Stm32f107RxDmaDesc *) rxCurDmaDesc->rdes3;
   }
   else
   {
      //No more data in the receive buffer
      error = ERR_BUFFER_EMPTY;
   }

   //Clear RBUS flag to resume processing
   ETH->DMASR = ETH_DMASR_RBUS;
   //Instruct the DMA to poll the receive descriptor list
   ETH->DMARPDR = 0;
*/
   //Return status code
   return error;
}
#endif

bool g_TargetTimeReached= 0;

//=================For STM, any received frame will be served in this IRQ Handler ====
extern uint8_t g_rstp_Rx_Buff[ETH_RX_BUF_SIZE];
extern unsigned short rstp_eth_input(u8 *prxbuf);//extern unsigned short eth_input_withRxbuf(u8 *rxbuf);

void ETH_IRQHandler( void )
{
#ifdef USE_TARGETTIME
	if( SET == ETH_GetMACITStatus(ETH_MAC_IT_TST)){
		g_TargetTimeReached = 1;
	}
#endif

	//Read DMA Status register MDASR
    if( SET == ETH_GetDMAITStatus( ETH_DMA_IT_R )){

    	//Disable RIE Interrupt
    	//ETH->DMAIER &= ~ETH_DMAIER_RIE;

		while (ETH_CheckFrameReceived())   {// check if any packet received
#if (ETHIF_ROLE == SIMPLE_ETH)
			printf("---EthIRQ\r\n");
			//Appropriate Handler
	#if (PROJ_FOR == PROJ_FOR_EDU)
			ethernetif_low_level_input_VerySimple();
	#elif (PROJ_FOR == PROJ_FOR_RSTP)
			rstp_eth_input(g_rstp_Rx_Buff);
	#endif

#else
			//printf("R ");
   			//ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
   			//ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);

   			//ethernetif_low_level_input_VerySimple();

			LwIP_Pkt_Handle(); //avb_ethernetif_input()//move FIFO into pbuf, and handles it.
#endif
		}//while
        ETH_DMAClearITPendingBit( ETH_DMA_IT_R );    //Clear the Eth DMA Rx IT pending bits
    }//if

    if( SET == ETH_GetDMAITStatus( ETH_DMA_IT_T ) ){
        ETH_DMAClearITPendingBit( ETH_DMA_IT_T);
    }

    if( SET == ETH_GetDMAITStatus( ETH_DMA_IT_NIS ) ){ //Normal interrupt summary
        ETH_DMAClearITPendingBit( ETH_DMA_IT_NIS);
    }

    //ETH_DMAClearITPendingBit( ETH_DMA_IT_R | ETH_DMA_IT_T | ETH_DMA_IT_NIS);    // Clear the Eth DMA Rx IT pending bits
    //Bit 16 NIS: Normal interrupt summary. The normal interrupt summary bit value is the logical OR of the following when the
	//	corresponding interrupt bits are enabled in the ETH_DMAIER register:
	//	?ETH_DMASR [0]: Transmit interrupt
	//	?ETH_DMASR [2]: Transmit buffer unavailable
	//	?ETH_DMASR [6]: Receive interrupt
	//	?ETH_DMASR [14]: Early receive interrupt
	//	Only unmasked bits affect the normal interrupt summary bit. This is a sticky bit and it must be cleared (by writing a 1 to this bit) each time a corresponding
	//	Bit 16 is only reset in ETH_IRQHandler and ETH_IRQHandler never being executed.
    //

    //if( SET == ETH_GetDMAITStatus( ETH_DMA_IT_TBU ) ) {//YOON
    	//ETH_SetDMATxDescOwnBit(gp_DMATxDescToSet);
        //ETH_DMAClearITPendingBit( ETH_DMA_IT_TBU);
        //printf("---TBU_IRQ");
    //}

}


/*
void ETH_IRQHandler( void )
{

#ifdef USE_TARGETTIME
	if( SET == ETH_GetMACITStatus(ETH_MAC_IT_TST)){
		g_TargetTimeReached = 1;
	}
#endif

    if( SET == ETH_GetDMAITStatus( ETH_DMA_IT_R )){
		while (ETH_CheckFrameReceived())   {// check if any packet received
#if (ETHIF_ROLE == SIMPLE_ETH)
			//printf("---EthIRQ\r\n");

			//Appropriate Handler
	#if (PROJ_FOR == PROJ_FOR_EDU)
			stm107_EthReceivePacket();//ethernetif_low_level_input_VerySimple();//ethernetif_input_VerySimple(); //
	#elif (PROJ_FOR == PROJ_FOR_RSTP)
			rstp_eth_input(g_rstp_Rx_Buff);
	#endif

#else
			LwIP_Pkt_Handle(); //avb_ethernetif_input()//move FIFO into pbuf, and handles it.
#endif
		}
        ETH_DMAClearITPendingBit( ETH_DMA_IT_R );    //Clear the Eth DMA Rx IT pending bits
    }
    if( SET == ETH_GetDMAITStatus( ETH_DMA_IT_T ) ){
        ETH_DMAClearITPendingBit( ETH_DMA_IT_T);
    }
    if( SET == ETH_GetDMAITStatus( ETH_DMA_IT_NIS ) ){ //Normal interrupt summary
        ETH_DMAClearITPendingBit( ETH_DMA_IT_NIS);
    }

    //ETH_DMAClearITPendingBit( ETH_DMA_IT_R | ETH_DMA_IT_T | ETH_DMA_IT_NIS);    // Clear the Eth DMA Rx IT pending bits
    //Bit 16 NIS: Normal interrupt summary. The normal interrupt summary bit value is the logical OR of the following when the
	//	corresponding interrupt bits are enabled in the ETH_DMAIER register:
	//	– ETH_DMASR [0]: Transmit interrupt
	//	– ETH_DMASR [2]: Transmit buffer unavailable
	//	– ETH_DMASR [6]: Receive interrupt
	//	– ETH_DMASR [14]: Early receive interrupt
	//	Only unmasked bits affect the normal interrupt summary bit. This is a sticky bit and it must be cleared (by writing a 1 to this bit) each time a corresponding
	//	Bit 16 is only reset in ETH_IRQHandler and ETH_IRQHandler never being executed.
    //

    //if( SET == ETH_GetDMAITStatus( ETH_DMA_IT_TBU ) ) {//YOON
    	//ETH_SetDMATxDescOwnBit(gp_DMATxDescToSet);
        //ETH_DMAClearITPendingBit( ETH_DMA_IT_TBU);
        //printf("---TBU_IRQ");
    //}

}
*/


//=====================================================Reference Only =============
#if (PHYCHIP == DP83848)

void ETH_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clocks */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE,  ENABLE);

   //Enable SysCfg Clock
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure MCO (PA8) */ //Someone reports the MCO should be used even if not used for Ethernet. ??? Really? And PA8 should be NC.
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  //GPIO_Init(GPIOA, &GPIO_InitStructure);

#if (MII_RMII == MII_MODE)
 #ifdef USE_MCO1_FOR_PHY_CLOCK_SRC
  /* Output HSE clock (25MHz) on MCO pin (PA8) to clock the PHY */
  RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
	printf("***Using MCO1 25MHz Output ***\r\n");
 #endif /* USE_MCO1_FOR_PHY_CLOCK_SRC */
  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_MII); ////0 = MII/ 1= RMII
#else
  /* Output PLL clock divided by 2 (50MHz) on MCO pin (PA8) to clock the PHY */
  //RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_2);
  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);
#endif

  /* Configure nRESETPHY (PC8). Issue nPHY_RESET */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   //SET MII mode while in RESET ????
   GPIO_ResetBits(GPIOC, GPIO_Pin_8);

  delayms(800);
  GPIO_SetBits(GPIOC, GPIO_Pin_8); //nRESET_PHY TO HIGH
  delayms(10);

/* Ethernet pins configuration (YOON)************************************************/
   /*
        ETH_MDIO -------------------------> PA2 +
        ETH_MDC --------------------------> PC1 +
        ETH_PPS_OUT ----------------------> PB5 --- STM32 PPS
        ETH_MII_CRS ----------------------> PA0  -- Dont cared by MAC for Full Duplex Mode.
        ETH_MII_COL ----------------------> PA3  -- Dont cared by MAC for Full Duplex Mode.
        ETH_MII_RX_ER --------------------> PB10
        ETH_MII_RXD2 ---------------------> PB0
        ETH_MII_RXD3 ---------------------> PB1
        ETH_MII_TX_CLK -------------------> PC3 +
        ETH_MII_TXD2 ---------------------> PC2 +
        ETH_MII_TXD3 ---------------------> PE2 //PB8 +
        ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1 +
        ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7 +
        ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4 +
        ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5 +
        ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PB11
        ETH_MII_TXD0/ETH_RMII_TXD0 -------> PB12
        ETH_MII_TXD1/ETH_RMII_TXD1 -------> PB13

        nPHYRESET ------------------------> PC8

 	 	 // Configure PA0, PA1, PA2, PA3, and PA7
  	  	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  	  	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //?? NO GPIO_OType_AF
  	  	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	  	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	  	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_7;
  	  	  GPIO_Init(GPIOA, &GPIO_InitStructure);

  	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH); //AF11

  	  	  // Configure PB0,PB1,PB5,PB10,PB11,PB12 and PB13
  	  	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 |  GPIO_Pin_13 | GPIO_Pin_12 ;
  	  	  GPIO_Init(GPIOB, &GPIO_InitStructure);

  	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_ETH); //PPS - The PPS output is enabled through a GPIO alternate function. (GPIO_AFR register).
  	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_ETH);

  	  	  // Configure PC1, PC2, PC3, PC4 and PC5
  	  	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  	  	  GPIO_Init(GPIOC, &GPIO_InitStructure);
  	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH);
  	  	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH);

  	  	  // Configure PE2 (for TXD3)
  	  	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
  	  	  GPIO_Init(GPIOE, &GPIO_InitStructure);
  	  	  GPIO_PinAFConfig(GPIOE, GPIO_PinSource2, GPIO_AF_ETH);
                                                  */
  /* RMII
       ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1
       ETH_MDIO -------------------------> PA2
       ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7

       ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PB11
       ETH_MII_TXD0/ETH_RMII_TXD0 -------> PB12
       ETH_MII_TXD1/ETH_RMII_TXD1 -------> PB13

       ETH_MDC --------------------------> PC1
       ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4
       ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5
                                                 */
  /* Enable GPIOs clocks for Ethernet */
  /* Configure PA1, PA2 and PA7 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH);

  /* Configure PB11, PB12 and PB13 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_ETH);

  /* Configure PC1, PC4 and PC5 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH);
}


void ETH_BSP_Config(void)
{
  RCC_ClocksTypeDef RCC_Clocks;

    /***************************************************************************
    NOTE:
         When using Systick to manage the delay in Ethernet driver, the Systick
         must be configured before Ethernet initialization and, the interrupt
         priority should be the highest one.
  *****************************************************************************/

  /* Configure Systick clock source as HCLK */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

  /* SystTick configuration: an interrupt every 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

  /* Set Systick interrupt priority to 0*/
  NVIC_SetPriority (SysTick_IRQn, 0);

  /* Configure the GPIO ports for ethernet pins */
  ETH_GPIO_Config();

  /* Configure the Ethernet MAC/DMA */
  ETH_MACDMA_Config();

  /* Read PHY status register: Get Ethernet link status */
  //if(ETH_ReadPHYRegister(DP83848_PHY_ADDRESS, PHY_SR_DP83848) & 1) {
  //  EthStatus |= ETH_LINK_FLAG;
  //}

  /* Configure LED mode to MODE 1 */
  //Eth_Link_PHY_LEDConfig(DP83848_PHY_ADDRESS);

  /* Configure the PHY to generate an interrupt on change of link status */
  //Eth_Link_PHYITConfig(DP83848_PHY_ADDRESS);

  /* Configure the EXTI for Ethernet link status. */
  //Eth_Link_EXTIConfig();
}
/**
  * @brief  Configure the PHY to generate an interrupt on change of link status.
  * @param PHYAddress: external PHY address
  * @retval None
  */
uint32_t Eth_Link_PHYITConfig(uint16_t PHYAddress)
{
#if 0   // 20140822 hwpark. disable PHY Interrupt
  uint16_t tmpreg = 0;

  /* Read MICR register */
  tmpreg = ETH_ReadPHYRegister(PHYAddress, PHY_MICR_DP83848);

  /* Enable output interrupt events to signal via the INT pin */
  tmpreg |= (uint16_t)(PHY_MICR_INT_EN | PHY_MICR_INT_OE);
  if(!(ETH_WritePHYRegister(PHYAddress, PHY_MICR_DP83848, tmpreg)))
  {
    /* Return ERROR in case of write timeout */
    return ETH_ERROR;
  }

  /* Read MISR register */
  tmpreg = ETH_ReadPHYRegister(PHYAddress, PHY_MISR_DP83848);

  /* Enable Interrupt on change of link status */
  tmpreg |= (uint16_t)PHY_MISR_LINK_INT_EN;
  if(!(ETH_WritePHYRegister(PHYAddress, PHY_MISR_DP83848, tmpreg)))
  {
    /* Return ERROR in case of write timeout */
    return ETH_ERROR;
  }
#endif
  /* Return SUCCESS */
  return ETH_SUCCESS;
}

/**
  * @brief  EXTI configuration for Ethernet link status.
  * @param PHYAddress: external PHY address
  * @retval None
  */
void Eth_Link_EXTIConfig(void)
{
#if 1   // 20140822 hwpark. disable PHY Interrupt
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the INT (PB14) Clock */
  RCC_AHB1PeriphClockCmd(ETH_LINK_GPIO_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure INT pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = ETH_LINK_PIN;
  GPIO_Init(ETH_LINK_GPIO_PORT, &GPIO_InitStructure);

  /* Connect EXTI Line to INT Pin */
  SYSCFG_EXTILineConfig(ETH_LINK_EXTI_PORT_SOURCE, ETH_LINK_EXTI_PIN_SOURCE);

  /* Configure EXTI line */
  EXTI_InitStructure.EXTI_Line = ETH_LINK_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;        // 20140822 hwpark. Link status change interrupt from Link LED.
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set the EXTI interrupt to priority 1*/
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif
}

/**
  * @brief  This function handles Ethernet link status.
  * @param  None
  * @retval None
  */
void Eth_Link_ITHandler(uint16_t PHYAddress)
{
#if 1   // 20140822 hwpark. disable PHY Interrupt
  if((ETH_ReadPHYRegister(PHYAddress, PHY_SR_DP83848) & 1))  {
    netif_set_link_up(&g_netif);
  } else  {
    netif_set_link_down(&g_netif);
  }
#endif
#if 0
  /* Check whether the link interrupt has occurred or not */
  if(((ETH_ReadPHYRegister(PHYAddress, PHY_MISR_DP83848)) & PHY_LINK_STATUS) != 0)
  {
    if((ETH_ReadPHYRegister(PHYAddress, PHY_SR_DP83848) & 1))
    {
      netif_set_link_up(&g_netif);
    }
    else
    {
      netif_set_link_down(&g_netif);
    }
  }
#endif
}
void ETH_link_callback(struct netif *netif);
/**
  * @brief  Link callback function, this function is called on change of link status.
  * @param  The network interface
  * @retval None
  */
void ETH_link_callback(struct netif *netif)
{
  __IO uint32_t timeout = 0;
 uint32_t tmpreg;
 uint16_t RegValue;
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;
#ifndef USE_DHCP
  uint8_t iptab[4] = {0};
  uint8_t iptxt[20];
#endif /* USE_DHCP */

  if(netif_is_link_up(netif))
  {
    /* Restart the auto-negotiation */
    if(ETH_InitStructure.ETH_AutoNegotiation != ETH_AutoNegotiation_Disable)
    {
      /* Reset Timeout counter */
      timeout = 0;

      /* Enable auto-negotiation */
      ETH_WritePHYRegister(DP83848_PHY_ADDRESS, PHY_BCR, PHY_AutoNegotiation);

      /* Wait until the auto-negotiation will be completed */
      do
      {
        timeout++;
      } while (!(ETH_ReadPHYRegister(DP83848_PHY_ADDRESS, PHY_BSR, &result) & PHY_AutoNego_Complete) && (timeout < (uint32_t)PHY_READ_TO));

      /* Reset Timeout counter */
      timeout = 0;

      /* Read the result of the auto-negotiation */
      RegValue = ETH_ReadPHYRegister(DP83848_PHY_ADDRESS, PHY_SR_DP83848);

      /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
      if((RegValue & PHY_DUPLEX_STATUS) != (uint16_t)RESET)
      {
        /* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
        ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
      }
      else
      {
        /* Set Ethernet duplex mode to Half-duplex following the auto-negotiation */
        ETH_InitStructure.ETH_Mode = ETH_Mode_HalfDuplex;
      }
      /* Configure the MAC with the speed fixed by the auto-negotiation process */
      if(RegValue & PHY_SPEED_STATUS)
      {
        /* Set Ethernet speed to 10M following the auto-negotiation */
        ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
      }
      else
      {
        /* Set Ethernet speed to 100M following the auto-negotiation */
        ETH_InitStructure.ETH_Speed = ETH_Speed_100M;
      }

      /*------------------------ ETHERNET MACCR Re-Configuration --------------------*/
      /* Get the ETHERNET MACCR value */
      tmpreg = ETH->MACCR;

      /* Set the FES bit according to ETH_Speed value */
      /* Set the DM bit according to ETH_Mode value */
      tmpreg |= (uint32_t)(ETH_InitStructure.ETH_Speed | ETH_InitStructure.ETH_Mode);

      /* Write to ETHERNET MACCR */
      ETH->MACCR = (uint32_t)tmpreg;

      _eth_delay_(ETH_REG_WRITE_DELAY);
      tmpreg = ETH->MACCR;
      ETH->MACCR = tmpreg;
    }

    /* Restart MAC interface */
    ETH_Start();

#ifdef USE_DHCP
    ipaddr.addr = 0;
    netmask.addr = 0;
    gw.addr = 0;
    DHCP_state = DHCP_START;
#else
    IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, g_ip_addr3);//IP_ADDR3);
    IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
    IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif /* USE_DHCP */

    netif_set_addr(&g_netif, &ipaddr , &netmask, &gw);

    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&g_netif);

#ifdef SERIAL_DEBUG
    printf("# Network Calbe is now connected\r\n");
#ifndef USE_DHCP
    iptab[0] = g_ip_addr3;//IP_ADDR3;
    iptab[1] = IP_ADDR2;
    iptab[2] = IP_ADDR1;
    iptab[3] = IP_ADDR0;
    printf("# Static IP address\r\n");
    printf("#  : %d.%d.%d.%d\r\n", iptab[3], iptab[2], iptab[1], iptab[0]);
#endif
#endif

  }
  else
  {
    ETH_Stop();
#ifdef USE_DHCP
    DHCP_state = DHCP_LINK_DOWN;
    dhcp_stop(netif);
#endif /* USE_DHCP */

    /*  When the netif link is down this function must be called.*/
    netif_set_down(&g_netif);
#ifdef SERIAL_DEBUG
    printf("# Network Cable is unplugged\r\n");
#endif
  }
}

uint32_t Eth_Link_PHY_LEDConfig(uint16_t PHYAddress)
{
    uint16_t tmpreg = 0;

    /* Read CR register */
    tmpreg = ETH_ReadPHYRegister(PHYAddress, PHY_CR);

    /* Set LED Config to MODE 1 */
    tmpreg |= PHY_LED_CNFG;

    if(!(ETH_WritePHYRegister(PHYAddress, PHY_CR, tmpreg)))
    {
    /* Return ERROR in case of write timeout */
    return ETH_ERROR;
    }

    /* Return SUCCESS */
    return ETH_SUCCESS;

}
//===========================================================================================
#elif (PHYCHIP == BCM89810)
void ETH_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clocks */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE,  ENABLE);

   //Enable SysCfg Clock
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

   /* Configure nRESETPHY (PC8). Issue nPHY_RESET */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   //SET MII mode while in RESET ????
   GPIO_ResetBits(GPIOC, GPIO_Pin_8);


  /* Configure MCO (PA8) */ //Someone reports the MCO should be used even if not used for Ethernet. ??? Really? And PA8 should be NC.
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //GPIOD->BSRRH = 0x00;


  /* MII/RMII Media interface selection --------------------------------------*/
#if (MII_RMII == MII_MODE) /* Mode MII with STM324xG-EVAL  */

  //Select MII mode : Setting mode is allowed under reset or before enabling MAC clocks.
  //RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_ETH_MAC, ENABLE);

#ifdef USE_MCO1_FOR_PHY_CLOCK_SRC
 //#error using USE_MCO1_FOR_PHY_CLOCK_SRC !!!
   /* Output HSE clock (25MHz) on MCO pin (PA8) to clock the PHY */
   RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1); //--HOWEVER WE DO NOT USE FOR BROADR-REACH MODULE.
	printf("***Using MCO1 25MHz Output ***\r\n");
#endif /* USE_MCO1_FOR_PHY_CLOCK_SRC */
  //Select MII mode : Setting mode is allowed under reset or before enabling MAC clocks.
  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_MII); //0 = MII/ 1= RMII


#elif (MII_RMII == RMII_MODE)  /* Mode RMII with STM324xG-EVAL */
#error using RMII_MODE !!!!!
  /* Output PLL clock divided by 2 (50MHz) on MCO pin (PA8) to clock the PHY */
  //RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_2);
  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);
#endif

  delayms(800);
  GPIO_SetBits(GPIOC, GPIO_Pin_8); //nRESET_PHY TO HIGH

	  //RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_ETH_MAC, DISABLE);

  delayms(10);

/* Ethernet pins configuration (YOON)************************************************/
   /*
        ETH_MDIO -------------------------> PA2 +
        ETH_MDC --------------------------> PC1 +
        ETH_PPS_OUT ----------------------> PB5 --- STM32 PPS
        -ETH_MII_CRS ----------------------> PA0  -- Dont cared by MAC for Full Duplex Mode.
        -ETH_MII_COL ----------------------> PA3  -- Dont cared by MAC for Full Duplex Mode.
        -ETH_MII_RX_ER --------------------> PB10
        ETH_MII_RXD2 ---------------------> PB0
        ETH_MII_RXD3 ---------------------> PB1
        ETH_MII_TX_CLK -------------------> PC3 +
        ETH_MII_TXD2 ---------------------> PC2 +
        ETH_MII_TXD3 ---------------------> PE2 //PB8 +
        ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1 +
        ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7 +
        ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4 +
        ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5 +
        ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PB11
        ETH_MII_TXD0/ETH_RMII_TXD0 -------> PB12
        ETH_MII_TXD1/ETH_RMII_TXD1 -------> PB13

        -nPHYRESET ------------------------> PC8 (Optional)
                                                  */

  /* Enable GPIOs clocks for Ethernet */
  /* Configure PA0, PA1, PA2, PA3, and PA7 */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //?? NO GPIO_OType_AF
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH); //AF11

  /* Configure PB0,PB1,PB5,PB10,PB11,PB12 and PB13 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_11 |  GPIO_Pin_13 | GPIO_Pin_12 ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_ETH); //PPS - The PPS output is enabled through a GPIO alternate function. (GPIO_AFR register).
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_ETH);

  /* Configure PC1, PC2, PC3, PC4 and PC5 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH);

  /* Configure PE2 (for TXD3) */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource2, GPIO_AF_ETH);

}


void ETH_BROADR_BSP_Config(void)
{
	int i;
	uint16_t ret;
	unsigned int result;
	    /***************************************************************************
	    NOTE:
	         When using Systick to manage the delay in Ethernet driver, the Systick
	         must be configured before Ethernet initialization and, the interrupt
	         priority should be the highest one.
	  *****************************************************************************/

  // Configure the GPIO ports for ethernet pins
  ETH_GPIO_Config(); //Setup GPIO and MII Mode before MAC Clocking.
  printf("ETH_GPIO_Config...ok\r\n");
  delayms(10);

  // Config NVIC for Ethernet
  ETH_NVIC_Config();
  printf("ETH_NVIC_Config...ok\r\n");
  delayms(10);

  //Start MAC Clocking
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx | RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx | RCC_AHB1Periph_ETH_MAC_Rx | RCC_AHB1Periph_ETH_MAC_PTP, ENABLE);
  //add RCC_AHB1Periph_ETH_MAC_PTP. YOON   if user wants to use PTP option (in 'stm32f4x7_eth_bsp.c')

  //++++Setup Normal MII Mode of BroadR PHY++++++
  y811BroadR_SetupNormalMiiMode(); //LiteMode
  delayms(10);

  // Configure the Ethernet MAC/DMA
  ETH_MACDMA_Config();
  printf("ETH_MACDMA_Config...ok\r\n");
  printf("EthInitStatus: %d\r\n",gEthInitStatus);
  delayms(10);
  if (gEthInitStatus == 0) {
    while(1){
    	printf("EthInitStatus=0 --> Error\r\n"); delayms(100);
    }
  }
  printf("ETH_BSP_Config --> PASSED\r\n");

  //==== BroadR Specific ===================================
  // Read PHY status register: Get Ethernet link status
  if(ETH_ReadPHYRegister(BROADR_PHY_ADDRESS, 0x01) & 0x04)  { //LRE Status Register(01:2) == 1-> Link UP
	  EthLinkStatus |= ETH_LINK_FLAG;  //EthStatus |= ETH_LINK_FLAG;
  }
  // Configure the PHY to generate an interrupt on change of link status : NOT SUPPORT ON BR89810 PHY --YOON
  //Eth_Link_PHYITConfig(BROADR_PHY_ADDRESS);  //Eth_Link_PHYITConfig(0x18);//IP101A_PHY_ADDRESS);//KSZ8051_PHY_ADDRESS);
  // Configure the EXTI for Ethernet link status.
  //Eth_Link_EXTIConfig();

  //For AN
  //ETH_WritePHYRegister(yPHYADDR, 0, 0x1000);//0x1200	//UARTprintf(">AutoNeg.\r\n");
  //For FORCED MASTER
  //ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0, 0x0208);
  //printf(">Forced Master\r\n");
  //delayms(10);

  //Walk through PHY Registers
	for(i=0;i<16;i++){
		ret = ETH_ReadPHYRegister(BROADR_PHY_ADDRESS, i);
		printf("ret(%d)=0x%04x\r\n",i,ret);
		delayms(10);
	}
}

void y811BroadR_SetupNormalMiiMode(){
	u32 ret, result;


	ret = ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0x18, 0xf067);//	mdio_write(0x18, 0x18, 0xf067); //GMII/MII Mode Setup (use Shadow 7)
	delayms(10);
	if(ret == ETH_SUCCESS) printf("Suc 1\n");
	ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0x1c, 0xac01); //3.3V MII Pad Setup
	delayms(10);
	if(ret == ETH_SUCCESS) printf("Suc 2\n");
	ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0x17, 0x0f0e); //select expansion register of 0x0e
	delayms(10);
	if(ret == ETH_SUCCESS) printf("Suc 3\n");
	ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0x15, 0x0800); //MII-Lite Mode Setup
	delayms(10);
	if(ret == ETH_SUCCESS) printf("Suc 4\n");
}
#endif

#if 0
//Historical
void ETH_SimpleOrSwitchPHY_BSP_Config(unsigned char PhyAddr)
{
	int i;
	uint16_t retv;

  // Configure the GPIO ports for ethernet pins
#if (SWITCH_ID == SWITCH_KSZ8794)
  ETH_GPIO_Config_WO_MDIOMDC(); //Setup GPIO and MII Mode with SPI before MAC Clocking.
#else
  ETH_GPIO_Config(); 			//Setup GPIO and MII Mode before MAC Clocking.
#endif
  printf("ETH_GPIO_Config...ok \r\n");
  delayms(10);

  // Config NVIC for Ethernet
  ETH_NVIC_Config();
  printf("ETH_NVIC_Config...ok \r\n");
  delayms(10);

  //Start MAC Clocking -- will do in ETH_MACDMA_Config()
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx | RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx | RCC_AHB1Periph_ETH_MAC_Rx | RCC_AHB1Periph_ETH_MAC_PTP, ENABLE);
  //add RCC_AHB1Periph_ETH_MAC_PTP. YOON   if user wants to use PTP option (in 'stm32f4x7_eth_bsp.c')

  delayms(10);

  // Configure the Ethernet MAC/DMA
  ETH_MACDMA_Config(PhyAddr);//BROADR_PHY_ADDRESS);

  printf("ETH_MACDMA_Config...ok \r\n");
  printf("EthInitStatus: %d\r\n",gEthInitStatus);
  delayms(10);

  if (gEthInitStatus == 0) {
    while(1){
    	printf("EthInitStatus=0 --> Error \r\n"); delayms(100);
    }
  }
  printf("EthInitStatus=OK\r\n");

  //==== Get PHY Product ID
    // Read PHY ID register2 and 3: Should be 0x0180 and 0xdc48
  retv = ETH_ReadPHYRegister(PhyAddr, 0x02);
  printf("PHYID Reg2 = 0x%04x and ",retv);

  retv = ETH_ReadPHYRegister(PhyAddr, 0x03);
  printf("PHYID Reg3 = 0x%04x \r\n",retv);


  if(MII_RMII == MII_MODE){
	  printf("ETH_BSP_Config --> MII\r\n");
  }else
	  printf("ETH_BSP_Config --> RMII\r\n");
  printf("ETH_BSP_Config --> PASSED\r\n");
  //Walk through PHY Registers with SPI or I2C
/*	for(i=0;i<16;i++){
		retv = ETH_ReadPHYRegister(IP101A_PHY_ADDRESS, i);
		printf("ret(%d)=0x%04x\r\n",i,retv);
		delayms(10);
	}
*/
}

#endif
#endif //PROCESSOR
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
#if (ETHIF_ROLE == SIMPLE_ETH)
/*
void ethernetif_low_level_input_VerySimple()
{

  unsigned short len;
  int l =0;
  FrameTypeDef frame;
  unsigned char *buf;
  uint32_t i=0;
  __IO ETH_DMADESCTypeDef *DMARxNextDesc;
  char str[10];

  // get received frame
  frame = ETH_Get_Received_Frame_byPollingMethod();

  len = frame.length;
  buf = (unsigned char *)frame.buffer;

  sprintf(str,"R>Type=%02x%02x(%d)",buf[12], buf[13],rxnum);
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

  //When Rx Buffer unavailable flag is set: clear it and resume reception
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET) {
    // Clear RBUS ETHERNET DMA flag
    ETH->DMASR = ETH_DMASR_RBUS;
    // Resume DMA reception
    ETH->DMARPDR = 0;
  }

}
  */
#endif


