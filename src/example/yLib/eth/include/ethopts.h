/**
  ******************************************************************************
  * @file    lwipopts.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   lwIP Options Configuration.
  *          This file is based on Utilities\lwip_v1.3.2\src\include\lwip\opt.h 
  *          and contains the lwIP configuration for the STM32F4x7 demonstration.
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
#ifndef __ETHOPTS_H__
#define __ETHOPTS_H__
//======================= (1) HW CONFIG ===============================================================
#include "yLib/inet/ipconf.h"

//========= ETHERNET CONST =========================
//MII/RMII
#define ETH_IF_MII_MODE 	1
#define ETH_IF_RMII_MODE 	2
#define ETH_IF_SPI_MODE  	3

#define SWITCH_FAMILY_NOSWITCH  0
#define SWITCH_FAMILY_KSZ 		1
#define SWITCH_FAMILY_SMSC 		2
#define SWITCH_FAMILY_REALTEK 	3
#define SWITCH_FAMILY_ICPLUS 	4
#define SWITCH_FAMILY_NXP 		5
#define SWITCH_FAMILY_TI 		6
#define SWITCH_FAMILY_BC 		7

//PHYs
#define DP83848  			1
#define BCM89810 			2
#define IP101   			3 //Also we can support RTL8201CP
#define RTL9K       		4
#define TJA1100 			5
#define VPHY_SWITCH2MCU 	10
#define VMAC_SWITCH2MCU 	11

//#define IP101G 4
//#define RTL8201 5
//#define KSZ9021RN 6
//#define LAN9355SWITCH_BR_BR 3
//#define LAN9355SWITCH_RTL_BR 4
//#define LAN9355SWITCH_RTL_RTL 5
//#define RTL9047SWITCH 6
//#define GENERALPHY 7

//SWITCH CONTROLLERS
#define SWITCH_ID_NOSWITCH 0
#define SWITCH_ID_KSZ8794  1
#define SWITCH_ID_LAN9355  2
#define SWITCH_ID_IP175D   3
#define SWITCH_ID_KSZ8463  4 //3-port Switch for PTP
#define SWITCH_ID_KSZ8567  5 //7-port Switch for PTP/AVB
#define SWITCH_ID_SJA1105  8 //5-port Switch for PTP/AVB

//for MDIO==============

//=== PHY Access Method for MDIO
#define SMIIF 1
#define I2CIF 2
#define SPIIF 3

#define MDIO_FOR_EDU 			0
#define MDIO_FOR_BROADCOM 		1
#define MDIO_FOR_REALTEK9K 		2
#define MDIO_FOR_RTL9K 			2
#define MDIO_FOR_RTL9047SWITCH 	3
#define MDIO_FOR_NXP 			4
//#define MDIO_FOR MDIO_FOR_IP101G //EDU//RTL9K
//#define MDIO_FOR MDIO_FOR_LAN9355SWITCH_RTL_BR //BROADCOM //RTL9K //RTL9047SWITCH //RTL9K
//#define MDIO_FOR MDIO_FOR_RTL9K
//#define MDIO_FOR MDIO_FOR_RTL8201
extern unsigned char 	g_ip_addr3;//
//#define MAC_ADDR5   	g_macaddrLsb//g_ip_addr3

//#define KSZ8051_PHY_ADDRESS       0x00
#define DP83848_PHY_ADDRESS       	((uint16_t) 0x01)
#define IP101A_PHY_ADDRESS       	0x02//0x00 //0x10 - RTL8201
#define BROADR_PHY_ADDRESS       	0x18//0x19
#define RTL9K_PHY_ADDRESS       	0x01
#define TJA1100_PHY_ADDRESS     	0x5//00//5

//============ ETH OPTIONS ========
#define USE_TARGETTIME  1
#undef 	USE_TARGETTIME

#define YPTP_PPS_FREQ 	0//10 //freq = 2**10 = 1024Hz. Default = 0(=1Hz)

//==== PTP PROTOCOLS
#define IEEE1588V2 1
#define IEEE8021AS 2

//==== Clock Source
#define CLOCKSOURCE_LOC 0
#define CLOCKSOURCE_GPS 1

//=== PTP ROLES
#define SIMPLE_ETH 				0
#define PTP_OC 					1
#define PTP_GRANDMASTERCLOCK 	2
#define RSTP_ETH 					4
//SIMPLE_ETH_IP101 //PTP_TC_9355 //PTP_GRANDMASTERCLOCK //SHOULD BE SET
//#if ((PHYCHIP == SWITCHPHY) ||  (MII_RMII == RMII_MODE))
//#undef USE_MCO1_FOR_PHY_CLOCK_SRC
//#endif
#endif /* __ETHOPTS_H__ */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
