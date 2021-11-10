/**
  ******************************************************************************
  * @file    stm32f4x7_eth_bsp.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   Header for stm32f4x7_eth_bsp.c file.
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
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4x7_ETH_BSP_H
#define __STM32F4x7_ETH_BSP_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_exti.h"
#include "lwip/include/lwipopts.h"
  /* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#if (PTP_ROLE == PTP_GRANDMASTERCLOCK)
 	#define MAC_ADDR0   'B'
 	#define MAC_ADDR1   'G'
 	#define MAC_ADDR2   'A'
 	#define MAC_ADDR3   'R'
 	#define MAC_ADDR4   'M'
 	#define MAC_ADDR5   'I'
#elif (PTP_ROLE == PTP_OC)
 	#if(SWTICH_ID == SWITCH_KSZ8794)
 		#define MAC_ADDR0   'D'
 		#define MAC_ADDR1   'K'
 		#define MAC_ADDR2   'A'
 		#define MAC_ADDR3   'R'
 		#define MAC_ADDR4   'A'
 		#define MAC_ADDR5   'M'
 	#elif(SWTICH_ID == SWITCH_LAN9355)
 		#define MAC_ADDR0   'F'
 		#define MAC_ADDR1   'L'
 		#define MAC_ADDR2   'A'
 		#define MAC_ADDR3   'R'
 		#define MAC_ADDR4   'A'
 		#define MAC_ADDR5   'M'
 	#else
 		#define MAC_ADDR0   'H'
 		#define MAC_ADDR1   'I'
 		#define MAC_ADDR2   'R'
 		#define MAC_ADDR3   'A'
 		#define MAC_ADDR4   'M'
  		#define MAC_ADDR5   ((unsigned char)randNum32)
 	#endif
#else
		#define MAC_ADDR0   'P'
		#define MAC_ADDR1   'I'
		#define MAC_ADDR2   'R'
		#define MAC_ADDR3   'A'
		#define MAC_ADDR4   'M'
		#define MAC_ADDR5   'I'
#endif

//#define KSZ8051_PHY_ADDRESS       0x00
#define DP83848_PHY_ADDRESS       ((uint16_t) 0x01)
#define IP101A_PHY_ADDRESS       0x02//0x00 //0x10 - RTL8201
#define BROADR_PHY_ADDRESS       0x18//0x19
#define RTL9K_PHY_ADDRESS       0x01
#define TJA1100_PHY_ADDRESS     0x5//00//5

 //==================================================================================================
 /* Uncomment the line below when using time stamping and/or IPv4 checksum offload */
 #define USE_ENHANCED_DMA_DESCRIPTORS

 // Uncomment the line below if you want to use user defined Delay function (for using SysTick), otherwise default _eth_delay_ function defined within
 //   the Ethernet driver is used (less precise timing)
 //#define USE_Delay

 #ifdef USE_Delay
   #define _eth_delay_    delayms     /* User can provide more timing precise _eth_delay_ function
                                       ex. use Systick with time base of 10 ms (as done in the provided
                                       STM32F4x7xx demonstrations) */
 #else
   #define _eth_delay_    ETH_Delay /* Default _eth_delay_ function with less precise timing */
 #endif

#if (PHYCHIP == DP83848)
 /* Uncomment the line below to allow custom configuration of the Ethernet driver buffers */
 #define CUSTOM_DRIVER_BUFFERS_CONFIG
 #ifdef  CUSTOM_DRIVER_BUFFERS_CONFIG
 /* Redefinition of the Ethernet driver buffers size and count */
  #define ETH_RX_BUF_SIZE    ETH_MAX_PACKET_SIZE /* buffer size for receive */
  #define ETH_TX_BUF_SIZE    ETH_MAX_PACKET_SIZE /* buffer size for transmit */
  #define ETH_RXBUFNB        4 //20                  /* 20 Rx buffers of size ETH_RX_BUF_SIZE */
  #define ETH_TXBUFNB        4 //5                   /* 5  Tx buffers of size ETH_TX_BUF_SIZE */
 #endif
#endif

 /* PHY configuration section **************************************************/
 #ifdef USE_Delay
 /* PHY Reset delay */
 #define PHY_RESET_DELAY    ((uint32_t)0x000000FF)
 /* PHY Configuration delay */
 #define PHY_CONFIG_DELAY   ((uint32_t)0x00000FFF)
 /* Delay when writing to Ethernet registers*/
 #define ETH_REG_WRITE_DELAY ((uint32_t)0x00000001)
 #else
 /* PHY Reset delay */
 #define PHY_RESET_DELAY    ((uint32_t)0x000FFFFF)
 /* PHY Configuration delay */
 #define PHY_CONFIG_DELAY   ((uint32_t)0x00FFFFFF)
 /* Delay when writing to Ethernet registers*/
 #define ETH_REG_WRITE_DELAY ((uint32_t)0x0000FFFF)
 #endif

 /*******************  PHY Extended Registers section : ************************/

 /* These values are relatives to DP83848 PHY and change from PHY to another,
    so the user have to update this value depending on the used external PHY */
#if (PHYCHIP == DP83848)
 /* The DP83848 PHY Control Register */
 #define PHY_CR                 ((uint16_t)0x19)
 #define PHY_LED_CNFG           ((uint16_t)0x0020)       // LED Configuration. 0 : Mode 1, 1 : Mode 2

 /* The DP83848 PHY status register  */
 #define PHY_SR_DP83848                 ((uint16_t)0x10) /* PHY status register Offset */
 #define PHY_SPEED_STATUS       ((uint16_t)0x0002) /* PHY Speed mask */
 #define PHY_DUPLEX_STATUS      ((uint16_t)0x0004) /* PHY Duplex mask */

 /* The DP83848 PHY: MII Interrupt Control Register  */
 #define PHY_MICR_DP83848               ((uint16_t)0x11) /* MII Interrupt Control Register */
 #define PHY_MICR_INT_EN        ((uint16_t)0x0002) /* PHY Enable interrupts */
 #define PHY_MICR_INT_OE        ((uint16_t)0x0001) /* PHY Enable output interrupt events */

 /* The DP83848 PHY: MII Interrupt Status and Misc. Control Register */
 #define PHY_MISR_DP83848               ((uint16_t)0x12) /* MII Interrupt Status and Misc. Control Register */
 #define PHY_MISR_LINK_INT_EN   ((uint16_t)0x0020) /* Enable Interrupt on change of link status */
 #define PHY_LINK_STATUS        ((uint16_t)0x2000) /* PHY link status interrupt mask */


 /* Specific defines for EXTI line, used to manage Ethernet link status */
 #define ETH_LINK_EXTI_LINE             EXTI_Line14
 #define ETH_LINK_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOB
 #define ETH_LINK_EXTI_PIN_SOURCE       EXTI_PinSource14
 #define ETH_LINK_EXTI_IRQn             EXTI15_10_IRQn
 /* PB14 */
 #define ETH_LINK_PIN                   GPIO_Pin_14
 #define ETH_LINK_GPIO_PORT             GPIOB
 #define ETH_LINK_GPIO_CLK              RCC_AHB1Periph_GPIOB

 /* Ethernet Flags for EthStatus variable */
 #define ETH_INIT_FLAG           0x01 /* Ethernet Init Flag */
 #define ETH_LINK_FLAG           0x10 /* Ethernet Link Flag */
#endif




/* Specific defines for EXTI line, used to manage Ethernet link status (PC0)*/
#define ETH_LINK_EXTI_LINE             EXTI_Line14 //0
#define ETH_LINK_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOB //C
#define ETH_LINK_EXTI_PIN_SOURCE       EXTI_PinSource14 //0
#define ETH_LINK_EXTI_IRQn             EXTI15_10_IRQn //EXTI0_IRQn
/* PC0 */
#define ETH_LINK_PIN                   GPIO_Pin_14 //0
#define ETH_LINK_GPIO_PORT             GPIOB //C
#define ETH_LINK_GPIO_CLK              RCC_AHB1Periph_GPIOB //C

 /* Ethernet Flags for EthStatus variable : DP83848*/
 #define ETH_INIT_FLAG           0x01 /* Ethernet Init Flag */
 #define ETH_LINK_FLAG           0x10 /* Ethernet Link Flag */

 /* PHY registers */
#define PHY_IRQ_STAT_CTRL                  0x1b
#define PHY_IRQ_LINK_DOWN         ((uint16_t)(1<<10)) /* link down interrupt enable*/
#define PHY_IRQ_LINK_UP           ((uint16_t)(1<<8)) /* link up interrupt enable*/

#define PHY_LINK_DOWN             ((uint16_t)(1<<2)) /* link down mask */
#define PHY_LINK_UP             ((uint16_t)(1<<0)) /* link up mask */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void  ETH_BSP_Config(void);
uint32_t Eth_Link_PHYITConfig(uint16_t PHYAddress);
void Eth_Link_EXTIConfig(void);
void Eth_Link_ITHandler(uint16_t PHYAddress);

//PHY DP83848

extern uint32_t Eth_Link_PHY_LEDConfig(uint16_t PHYAddress);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4x7_ETH_BSP_H */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
