/*
  Ksz8567R Ethernet Switch
  [REF] KSZ9477S (with Ring Redundancy)
  Author: YOON
  */
/*
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "yInc.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
*/
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#include "yInc.h"
#include "ySpiDrv.h"
//I2C
#define OLED_ADDRESS		0x78 //0x3C = 7bit address --> 0x78 (8 bit addr)
//#define LAN9355_ADDRESS	0x14 //0x0A(000 1010) 7bit address --> 0x14(8 bit addr)
#define I2C_400KHZ			1	// 0 to use default 100Khz, 1 for 400Khz
//SPI
#if ((PROCESSOR == STM32F103C8) || (PROCESSOR == STM32F107VCT6))
	//Ksz8567 uses SPI.(nCS0=PB12)--STM32F103
#define nCS_KSZ8567_H nCS0_H
#define nCS_KSZ8567_L nCS0_L
#else
#endif
//Ksz8567R Registers -- Refer to ksz_9477.h
	#define KS_PORT_M			0x7//1F
//	#define KS_PRIO_M			0x3
	#define KS_PRIO_S			4//2

//====== Global REG ======================
#define REG8567_ID0_1			0x0000 //RO 0x00 -- Operation
#define REG8567_ID1_1			0x0001 //RO 0x85
	#define FAMILY_ID				0x85
#define REG8567_ID2_1			0x0002 //RO 0x67
	#define CHIP_ID_67				0x67
#define REG8567_ID3_1			0x0003 //7:4(REVID); 3:1(RSVD); 0(Software Reset: 0=Normal)
	#define SW_REVISION_M			0x0F//E
	#define SW_REVISION_S			4//1
	#define SW_RESET				0x01
#define	REG8567_SW_PME_CTRL		0x0006
	#define PME_ENABLE				BIT(1)
	#define PME_POLARITY			BIT(0)
#define REG8567_GLOBAL_INT_STATUS_4	0x0010
#define REG8567_GLOBAL_INT_MASK_4	0x0014
	#define LUE_INT					BIT(31) //Lookup Engine Interrupt Status
	#define TRIG_TS_INT				BIT(30)	//GPIO Pin Output Trigger/Timestamp Unit Interrupt Status
	//#define APB_TIMEOUT_INT			BIT(29)
	//#define SWITCH_INT_MASK				(TRIG_TS_INT | APB_TIMEOUT_INT)
#define REG8567_GLOBAL_PORT_INT_STATUS_4 0x0018
#define REG8567_GLOBAL_PORT_INT_MASK_4 	0x001C
#define REG8567_SW_PHY_INT_STATUS 	0x0020
//#define REG8567_SW_PHY_INT_ENABLE 	0x0024

//------------ 0x100 (Global IO Control Registers ----------
#define REG8567_SERIAL_IO_CONTROL_1				0x0100
#define REG8567_OUTPUT_CLOCK_CONTROL_1			0x0103 	//SYNCLKO Source Selection
#define REG8567_INBANDMGMT_CONTROL_4			0x0104	//
#define REG8567_IO_DRIVE_STRENGTH_1				0x010D
#define REG8567_INBANDMGMT_OPERATION_STATUS_4	0x0110
#define REG8567_LED_OVERRIDE_4					0x0120
#define REG8567_LED_OUTPUT_4					0x0124
#define REG8567_LED2_0_LED2_1_SOURCE_4			0x0128
//---------- 0x200(Global PHY Control Registers -------------
#define REG8567_POWER_DOWN_CONTROL0_1			0x0201
	#define SW_PLL_POWER_DOWN		(1 << 5)
	#define SW_POWER_MANAGEMENT_MODE_M	0x3
	#define SW_POWER_MANAGEMENT_MODE_S	3
	#define SW_POWER_NORMAL				0
	#define SW_ENERGY_DETECTION			1
	#define SW_SOFTWARE_POWER_DOWN		2
#define REG8567_LED_CONFIG_STRAP_4				0x0210
//---------- 0x300(Global SWITCH Control Registers -------------
#define REG8567_SWITCH_OPERATION_1				0x0300	//Tag
	#define SW_DOULBE_TAG				BIT(7)
	#define SW_SOFT_HARDWARE_RESET		BIT(1)
	#define SW_START_SWITCH				BIT(0)
#define REG8567_SWITCH_MAC_ADDR0_1				0x0302
#define REG8567_SWITCH_MAC_ADDR1_1				0x0303
#define REG8567_SWITCH_MAC_ADDR2_1				0x0304
#define REG8567_SWITCH_MAC_ADDR3_1				0x0305
#define REG8567_SWITCH_MAC_ADDR4_1				0x0306
#define REG8567_SWITCH_MAC_ADDR5_1				0x0307

#define REG8567_SWITCH_MAX_TX_UNIT_2			0x0308
#define REG8567_SWITCH_ISP_TPID_2				0x030a
#define REG8567_AVB_CBS_STATEGY_2				0x030e  //Credit-based shaper.  (use default)
#define REG8567_SWITCH_LOOKUP_ENGINE_CONTROL0_1	0x0310	//VLAN-Enable,..
#define REG8567_SWITCH_LOOKUP_ENGINE_CONTROL1_1	0x0311	//Unicast Learning enable, ...Flush,..
#define REG8567_SWITCH_LOOKUP_ENGINE_CONTROL2_1	0x0312	//Dynamic Entry Egress VLAN Filtering,...
#define REG8567_SWITCH_LOOKUP_ENGINE_CONTROL3_1	0x0313	//Age Period Set(default=0x4B) (sec)
#define REG8567_ADDR_LOOKUP_TABLE_INTERRPUT_1	0x0314	//
#define REG8567_ADDR_LOOKUP_TABLE_MASK_1		0x0315	//
#define REG8567_ADDR_LOOKUP_TABLE_ENTRY_IDX0_2	0x0316	//
#define REG8567_ADDR_LOOKUP_TABLE_ENTRY_IDX1_2	0x0318	//
#define REG8567_UNKNOWN_UNICAST_CONTROL_4		0x0320	//
//...
#define REG8567_GLOBAL_PORT_MIRRORING_SNOOPING_CONTROL_1	0x0370	//
#define REG8567_WRED_DIFFSERV_COLOR_MAPPING_1	0x0378	// Red-Yellow-Green.
#define REG8567_PTP_EVENT_MSG_PRIO_1			0x037C	//
#define REG8567_PTP_NONEVENT_MSG_PRIO_1			0x037D	//
#define REG8567_QUEUE_MGMT_CONTROL0_4			0x0390	//
//..
//----------- Global switch lookup engine control registers
#define REG8567_VLAN_TABLE_ENTRY0_4				0x0400	//
//..
//----------- Global switch PTP control registers
#define REG8567_GLOBAL_PTP_CLOCK_CONTROL_2				0x0500	//
//..
#define REG8567_GLOBAL_PTP_MSG_CONFIG1_2				0x0514	//Enable 802.3As(x), 1588, ...P2P/E2E(*), Master/Slave(*), One(*)/2-step
#define REG8567_GLOBAL_PTP_MSG_CONFIG2_2				0x0516	//
#define REG8567_GLOBAL_PTP_DOMAIN_VERSION_2				0x0518	//
//..
//==== PORT Registers =====(N=1..7)====================
//(PORT1)
#define REG8567_PORT1_PORT_DEFAULT_TAG0_1				0x1000	//VLANID[11:8]
#define REG8567_PORT1_PORT_DEFAULT_TAG1_1				0x1001 	//VLANID[7:0]
//...
#define REG8567_PORT1_PORT_OPERATION_CONTRL0_1			0x1020 	//Local Loopback, TailTag Enable,...
#define REG8567_PORT1_PORT_STATUS_1						0x1030 	//Speed, Duplex,
#define REG8567_PORT1_PHY_BASIC_CONTROL_2				0x1100 	//PHY Software Reset, Loopback Mode, Speed,...
#define REG8567_PORT1_PHY_BASIC_STATUS_2				0x1102 	//Speed, Duplex,
#define REG8567_PORT1_PORT_MAC_CONTROL0_1				0x1400 	//BroadcastStorm, ...
#define REG8567_PORT1_PORT_MAC_CONTROL1_1				0x1401 	//Backpressure, pass all frames
//...
#define REG8567_PORT1_PORT_TXQ_CONTROL0_1				0x1914 	//SchedulerMode(Strict/WRR) + ShaperMode(CBS,TAS)
#define REG8567_PORT1_PORT_TXQ_CONTROL1_1				0x1915 	//QueueWeightForWRRscheduling
#define REG8567_PORT1_PORT_TX_CBS_CONTROL0_2	0x1916 	//High Water Mark (0x0534)
#define REG8567_PORT1_PORT_TX_CBS_CONTROL1_2	0x1918 	//Low Water Mark (0x05F2)
#define REG8567_PORT1_PORT_TX_CBS_CONTROL2_2	0x191A 	//PortQueue Credit Increment 12.5%(0x2000)
#define REG8567_PORT1_PORT_TAS_CONTROL_1		0x1920 	//Cut-through Enable, Restricted TAS, RefTimeSelect for start time T0
#define REG8567_PORT1_PORT_TAS_EVENT_IDX_1		0x1923 	//
#define REG8567_PORT1_PORT_TAS_EVENT_4			0x1924 	//Event(Repeat, open, guardband start, close) + time(cycle count)
//(PORT2)
#define REG8567_PORT2_PORT_DEFAULT_TAG0_1				0x2000	//VLANID[11:8]
#define REG8567_PORT2_PORT_DEFAULT_TAG1_1				0x2001 	//VLANID[7:0]
//...
#define REG8567_PORT2_PORT_OPERATION_CONTRL0_1			0x2020 	//Local Loopback, TailTag Enable,...
#define REG8567_PORT2_PORT_STATUS_1						0x2030 	//Speed, Duplex,
#define REG8567_PORT2_PHY_BASIC_CONTROL_2				0x2100 	//PHY Software Reset, Loopback Mode, Speed,...
#define REG8567_PORT2_PHY_BASIC_STATUS_2				0x2102 	//Speed, Duplex,
#define REG8567_PORT1_PORT_MAC_CONTROL0_1				0x2400 	//BroadcastStorm, ...
#define REG8567_PORT1_PORT_MAC_CONTROL1_1				0x2401 	//Backpressure, pass all frames
//...
#define REG8567_PORT2_PORT_TXQ_CONTROL0_1				0x2914 	//SchedulerMode(Strict/WRR) + ShaperMode(CBS,TAS)
#define REG8567_PORT2_PORT_TXQ_CONTROL1_1				0x2915 	//QueueWeightForWRRscheduling
#define REG8567_PORT2_PORT_TX_CBS_CONTROL0_2			0x2916 	//High Water Mark (0x0534)
#define REG8567_PORT2_PORT_TX_CBS_CONTROL1_2			0x2918 	//Low Water Mark (0x05F2)
#define REG8567_PORT2_PORT_TX_CBS_CONTROL2_2			0x291A 	//PortQueue Credit Increment 12.5%(0x2000)
#define REG8567_PORT2_PORT_TAS_CONTROL_1				0x2920 	//Cut-through Enable, Restricted TAS, RefTimeSelect for start time T0
#define REG8567_PORT2_PORT_TAS_EVENT_IDX_1				0x2923 	//
#define REG8567_PORT2_PORT_TAS_EVENT_4					0x2924 	//Event(Repeat, open, guardband start, close) + time(cycle count)

//(PORT5)
#define REG8567_PORT5_PORT_DEFAULT_TAG0_1				0x5000	//VLANID[11:8]
#define REG8567_PORT5_PORT_DEFAULT_TAG1_1				0x5001 	//VLANID[7:0]
//...
#define REG8567_PORT5_PORT_OPERATION_CONTRL0_1			0x5020 	//Local Loopback, TailTag Enable,...
#define REG8567_PORT5_PORT_STATUS_1						0x5030 	//Speed, Duplex,
#define REG8567_PORT5_PHY_BASIC_CONTROL_2				0x5100 	//PHY Software Reset, Loopback Mode, Speed,...
#define REG8567_PORT5_PHY_BASIC_STATUS_2				0x5102 	//Speed, Duplex,
#define REG8567_PORT1_PORT_MAC_CONTROL0_1				0x5400 	//BroadcastStorm, ...
#define REG8567_PORT1_PORT_MAC_CONTROL1_1				0x5401 	//Backpressure, pass all frames
//...
#define REG8567_PORT5_PORT_TXQ_CONTROL0_1				0x5914 	//SchedulerMode(Strict/WRR) + ShaperMode(CBS,TAS)
#define REG8567_PORT5_PORT_TXQ_CONTROL1_1				0x5915 	//QueueWeightForWRRscheduling
#define REG8567_PORT5_PORT_TX_CBS_CONTROL0_2			0x5916 	//High Water Mark (0x0534)
#define REG8567_PORT5_PORT_TX_CBS_CONTROL1_2			0x5918 	//Low Water Mark (0x05F2)
#define REG8567_PORT5_PORT_TX_CBS_CONTROL2_2			0x591A 	//PortQueue Credit Increment 12.5%(0x2000)
#define REG8567_PORT5_PORT_TAS_CONTROL_1				0x5920 	//Cut-through Enable, Restricted TAS, RefTimeSelect for start time T0
#define REG8567_PORT5_PORT_TAS_EVENT_IDX_1				0x5923 	//
#define REG8567_PORT5_PORT_TAS_EVENT_4					0x5924 	//Event(Repeat, open, guardband start, close) + time(cycle count)

//--- PORT 6/7 Only (MAC Port)
#define REG8567_PORT6_XMII_PORT_CONTROL0_1				0x6300 	//Speed, Duplex,

#define REG8567_PORT1_PORT_MAC_CONTROL0_1				0x6400 	//BroadcastStorm, ...
#define REG8567_PORT1_PORT_MAC_CONTROL1_1				0x6401 	//Backpressure, pass all frames


/*


#define REG8567_SW_CTRL_0			0x02
	#define SW_NEW_BACKOFF			(1 << 7)
	#define SW_GLOBAL_RESET			(1 << 6)
	#define SW_FLUSH_DYN_MAC_TABLE		(1 << 5)
	#define SW_FLUSH_STA_MAC_TABLE		(1 << 4)
	#define SW_LINK_AUTO_AGING		(1 << 0)

#define REG8567_SW_CTRL_1			0x03
	#define SW_HUGE_PACKET			(1 << 6)
	#define SW_TX_FLOW_CTRL_DISABLE		(1 << 5)
	#define SW_RX_FLOW_CTRL_DISABLE		(1 << 4)
	#define SW_CHECK_LENGTH			(1 << 3)
	#define SW_AGING_ENABLE			(1 << 2)
	#define SW_FAST_AGING			(1 << 1)
	#define SW_AGGR_BACKOFF			(1 << 0)

#define REG8567_SW_CTRL_2			0x04
	#define UNICAST_VLAN_BOUNDARY		(1 << 7)
	#define MULTICAST_STORM_DISABLE		(1 << 6)
	#define SW_BACK_PRESSURE		(1 << 5)
	#define FAIR_FLOW_CTRL			(1 << 4)
	#define NO_EXC_COLLISION_DROP		(1 << 3)
	#define SW_LEGAL_PACKET_DISABLE		(1 << 1)

#define REG8567_SW_CTRL_3			0x05
 	 #define WEIGHTED_FAIR_QUEUE_ENABLE	(1 << 3)
	#define SW_VLAN_ENABLE			(1 << 7)
	#define SW_IGMP_SNOOP			(1 << 6)
	#define SW_MIRROR_RX_TX			(1 << 0)

#define REG8567_SW_CTRL_4			0x06
	#define SW_HALF_DUPLEX_FLOW_CTRL	(1 << 7)
	#define SW_HALF_DUPLEX			(1 << 6)
	#define SW_FLOW_CTRL			(1 << 5)
	#define SW_10_MBIT			(1 << 4)
	#define SW_REPLACE_VID			(1 << 3)
	#define BROADCAST_STORM_RATE_HI		0x07

#define REG8567_SW_CTRL_5			0x07
	#define BROADCAST_STORM_RATE_LO		0xFF
	#define BROADCAST_STORM_RATE		0x07FF

#define REG8567_SW_CTRL_6			0x08
	#define SW_MIB_COUNTER_FLUSH		(1 << 7)
	#define SW_MIB_COUNTER_FREEZE		(1 << 6)
	#define SW_MIB_COUNTER_CTRL_ENABLE	KS_PORT_M

#define REG8567_SW_CTRL_9			0x0B
	#define SPI_CLK_125_MHZ			0x80
	#define SPI_CLK_62_5_MHZ		0x40
	#define SPI_CLK_31_25_MHZ		0x00

#define SW_LED_MODE_M			0x3
#define SW_LED_MODE_S			4
#define SW_LED_LINK_ACT_SPEED		0
#define SW_LED_LINK_ACT			1
#define SW_LED_LINK_ACT_DUPLEX		2
#define SW_LED_LINK_DUPLEX		3

#define REG8567_SW_CTRL_10			0x0C

#define SW_TAIL_TAG_ENABLE		(1 << 1)
#define SW_PASS_PAUSE			(1 << 0)

#define REG8567_SW_CTRL_11			0x0D

#define REG8567_POWER_MANAGEMENT_1		0x0E

#define REG8567_POWER_MANAGEMENT_2		0x0F


#define REG8567_PORT_1_CTRL_0		0x10
#define REG8567_PORT_2_CTRL_0		0x20
#define REG8567_PORT_3_CTRL_0		0x30
#define REG8567_PORT_4_CTRL_0		0x40
#define REG8567_PORT_5_CTRL_0		0x50

#define PORT_BROADCAST_STORM		(1 << 7)
#define PORT_DIFFSERV_ENABLE		(1 << 6)
#define PORT_802_1P_ENABLE		(1 << 5)
#define PORT_BASED_PRIO_S		3
#define PORT_BASED_PRIO_M		(KS_PRIO_M << PORT_BASED_PRIO_S)
#define PORT_PORT_PRIO_0		0
#define PORT_PORT_PRIO_1		1
#define PORT_PORT_PRIO_2		2
#define PORT_PORT_PRIO_3		3
#define PORT_INSERT_TAG			(1 << 2)
#define PORT_REMOVE_TAG			(1 << 1)
#define PORT_QUEUE_SPLIT_L		(1 << 0)

#define REG8567_PORT_1_CTRL_1		0x11
#define REG8567_PORT_2_CTRL_1		0x21
#define REG8567_PORT_3_CTRL_1		0x31
#define REG8567_PORT_4_CTRL_1		0x41
#define REG8567_PORT_5_CTRL_1		0x51

#define PORT_MIRROR_SNIFFER		(1 << 7)
#define PORT_MIRROR_RX			(1 << 6)
#define PORT_MIRROR_TX			(1 << 5)
#define PORT_VLAN_MEMBERSHIP		KS_PORT_M

#define REG8567_PORT_1_CTRL_2		0x12
#define REG8567_PORT_2_CTRL_2		0x22
#define REG8567_PORT_3_CTRL_2		0x32
#define REG8567_PORT_4_CTRL_2		0x42
#define REG8567_PORT_5_CTRL_2		0x52

#define PORT_802_1P_REMAPPING		(1 << 7)
#define PORT_INGRESS_FILTER		(1 << 6)
#define PORT_DISCARD_NON_VID		(1 << 5)
#define PORT_FORCE_FLOW_CTRL		(1 << 4)
#define PORT_BACK_PRESSURE		(1 << 3)
#define PORT_TX_ENABLE			(1 << 2)
#define PORT_RX_ENABLE			(1 << 1)
#define PORT_LEARN_DISABLE		(1 << 0)

#define REG8567_PORT_1_CTRL_3		0x13
#define REG8567_PORT_2_CTRL_3		0x23
#define REG8567_PORT_3_CTRL_3		0x33
#define REG8567_PORT_4_CTRL_3		0x43
#define REG8567_PORT_5_CTRL_3		0x53
#define REG8567_PORT_1_CTRL_4		0x14
#define REG8567_PORT_2_CTRL_4		0x24
#define REG8567_PORT_3_CTRL_4		0x34
#define REG8567_PORT_4_CTRL_4		0x44
#define REG8567_PORT_5_CTRL_4		0x54

#define PORT_DEFAULT_VID		0x0001

#define REG8567_PORT_1_CTRL_5		0x15
#define REG8567_PORT_2_CTRL_5		0x25
#define REG8567_PORT_3_CTRL_5		0x35
#define REG8567_PORT_4_CTRL_5		0x45
#define REG8567_PORT_5_CTRL_5		0x55

#define PORT_ACL_ENABLE			(1 << 2)
#define PORT_AUTHEN_MODE		0x3
#define PORT_AUTHEN_PASS		0
#define PORT_AUTHEN_BLOCK		1
#define PORT_AUTHEN_TRAP		2

#define REG8567_PORT_5_CTRL_6		0x56

#define PORT_MII_INTERNAL_CLOCK		(1 << 7)
#define PORT_GMII_1GPS_MODE		(1 << 6)
#define PORT_RGMII_ID_IN_ENABLE		(1 << 4)
#define PORT_RGMII_ID_OUT_ENABLE	(1 << 3)
#define PORT_GMII_MAC_MODE		(1 << 2)
#define PORT_INTERFACE_TYPE		0x3
#define PORT_INTERFACE_MII		0
#define PORT_INTERFACE_RMII		1
#define PORT_INTERFACE_GMII		2
#define PORT_INTERFACE_RGMII		3

#define REG8567_PORT_1_CTRL_7		0x17
#define REG8567_PORT_2_CTRL_7		0x27
#define REG8567_PORT_3_CTRL_7		0x37
#define REG8567_PORT_4_CTRL_7		0x47

#define PORT_AUTO_NEG_ASYM_PAUSE	(1 << 5)
#define PORT_AUTO_NEG_SYM_PAUSE		(1 << 4)
#define PORT_AUTO_NEG_100BTX_FD		(1 << 3)
#define PORT_AUTO_NEG_100BTX		(1 << 2)
#define PORT_AUTO_NEG_10BT_FD		(1 << 1)
#define PORT_AUTO_NEG_10BT		(1 << 0)

#define REG8567_PORT_1_STATUS_0		0x18
#define REG8567_PORT_2_STATUS_0		0x28
#define REG8567_PORT_3_STATUS_0		0x38
#define REG8567_PORT_4_STATUS_0		0x48

#define PORT_REMOTE_ASYM_PAUSE		(1 << 5)
#define PORT_REMOTE_SYM_PAUSE		(1 << 4)
#define PORT_REMOTE_100BTX_FD		(1 << 3)
#define PORT_REMOTE_100BTX		(1 << 2)
#define PORT_REMOTE_10BT_FD		(1 << 1)
#define PORT_REMOTE_10BT		(1 << 0)

#define REG8567_PORT_1_STATUS_1		0x19
#define REG8567_PORT_2_STATUS_1		0x29
#define REG8567_PORT_3_STATUS_1		0x39
#define REG8567_PORT_4_STATUS_1		0x49

#define PORT_HP_MDIX			(1 << 7)
#define PORT_REVERSED_POLARITY		(1 << 5)
#define PORT_TX_FLOW_CTRL		(1 << 4)
#define PORT_RX_FLOW_CTRL		(1 << 3)
#define PORT_STAT_SPEED_100MBIT		(1 << 2)
#define PORT_STAT_FULL_DUPLEX		(1 << 1)

#define PORT_REMOTE_FAULT		(1 << 0)

#define REG8567_PORT_1_LINK_MD_CTRL		0x1A
#define REG8567_PORT_2_LINK_MD_CTRL		0x2A
#define REG8567_PORT_3_LINK_MD_CTRL		0x3A
#define REG8567_PORT_4_LINK_MD_CTRL		0x4A

#define PORT_CABLE_10M_SHORT		(1 << 7)
#define PORT_CABLE_DIAG_RESULT_M	0x3
#define PORT_CABLE_DIAG_RESULT_S	5
#define PORT_CABLE_STAT_NORMAL		0
#define PORT_CABLE_STAT_OPEN		1
#define PORT_CABLE_STAT_SHORT		2
#define PORT_CABLE_STAT_FAILED		3
#define PORT_START_CABLE_DIAG		(1 << 4)
#define PORT_FORCE_LINK			(1 << 3)
#define PORT_POWER_SAVING		(1 << 2)
#define PORT_PHY_REMOTE_LOOPBACK	(1 << 1)
#define PORT_CABLE_FAULT_COUNTER_H	0x01

#define REG8567_PORT_1_LINK_MD_RESULT	0x1B
#define REG8567_PORT_2_LINK_MD_RESULT	0x2B
#define REG8567_PORT_3_LINK_MD_RESULT	0x3B
#define REG8567_PORT_4_LINK_MD_RESULT	0x4B

#define PORT_CABLE_FAULT_COUNTER_L	0xFF
#define PORT_CABLE_FAULT_COUNTER	0x1FF

#define REG8567_PORT_1_CTRL_9		0x1C
#define REG8567_PORT_2_CTRL_9		0x2C
#define REG8567_PORT_3_CTRL_9		0x3C
#define REG8567_PORT_4_CTRL_9		0x4C

#define PORT_AUTO_NEG_DISABLE		(1 << 7)
#define PORT_FORCE_100_MBIT		(1 << 6)
#define PORT_FORCE_FULL_DUPLEX		(1 << 5)

#define REG8567_PORT_1_CTRL_10		0x1D
#define REG8567_PORT_2_CTRL_10		0x2D
#define REG8567_PORT_3_CTRL_10		0x3D
#define REG8567_PORT_4_CTRL_10		0x4D

#define PORT_LED_OFF			(1 << 7)
#define PORT_TX_DISABLE			(1 << 6)
#define PORT_AUTO_NEG_RESTART		(1 << 5)
#define PORT_POWER_DOWN			(1 << 3)
#define PORT_AUTO_MDIX_DISABLE		(1 << 2)
#define PORT_FORCE_MDIX			(1 << 1)
#define PORT_MAC_LOOPBACK		(1 << 0)

#define REG8567_PORT_1_STATUS_2		0x1E
#define REG8567_PORT_2_STATUS_2		0x2E
#define REG8567_PORT_3_STATUS_2		0x3E
#define REG8567_PORT_4_STATUS_2		0x4E

#define PORT_MDIX_STATUS		(1 << 7)
#define PORT_AUTO_NEG_COMPLETE		(1 << 6)
#define PORT_STAT_LINK_GOOD		(1 << 5)

#define REG8567_PORT_1_STATUS_3		0x1F
#define REG8567_PORT_2_STATUS_3		0x2F
#define REG8567_PORT_3_STATUS_3		0x3F
#define REG8567_PORT_4_STATUS_3		0x4F

#define PORT_PHY_LOOPBACK		(1 << 7)
#define PORT_PHY_ISOLATE		(1 << 5)
#define PORT_PHY_SOFT_RESET		(1 << 4)
#define PORT_PHY_FORCE_LINK		(1 << 3)
#define PORT_PHY_MODE_M			0x7
#define PHY_MODE_IN_AUTO_NEG		1
#define PHY_MODE_10BT_HALF		2
#define PHY_MODE_100BT_HALF		3
#define PHY_MODE_10BT_FULL		5
#define PHY_MODE_100BT_FULL		6
#define PHY_MODE_ISOLDATE		7

#define REG8567_PORT_CTRL_0			0x00
#define REG8567_PORT_CTRL_1			0x01
#define REG8567_PORT_CTRL_2			0x02
#define REG8567_PORT_CTRL_VID		0x03

#define REG8567_PORT_CTRL_5			0x05

#define REG8567_PORT_CTRL_7			0x07
#define REG8567_PORT_STATUS_0		0x08
#define REG8567_PORT_STATUS_1		0x09
#define REG8567_PORT_LINK_MD_CTRL		0x0A
#define REG8567_PORT_LINK_MD_RESULT		0x0B
#define REG8567_PORT_CTRL_9			0x0C
#define REG8567_PORT_CTRL_10		0x0D
#define REG8567_PORT_STATUS_2		0x0E
#define REG8567_PORT_STATUS_3		0x0F

#define REG8567_PORT_CTRL_12		0xA0
#define REG8567_PORT_CTRL_13		0xA1
#define REG8567_PORT_RATE_CTRL_3		0xA2
#define REG8567_PORT_RATE_CTRL_2		0xA3
#define REG8567_PORT_RATE_CTRL_1		0xA4
#define REG8567_PORT_RATE_CTRL_0		0xA5
#define REG8567_PORT_RATE_LIMIT		0xA6
#define REG8567_PORT_IN_RATE_0		0xA7
#define REG8567_PORT_IN_RATE_1		0xA8
#define REG8567_PORT_IN_RATE_2		0xA9
#define REG8567_PORT_IN_RATE_3		0xAA
#define REG8567_PORT_OUT_RATE_0		0xAB
#define REG8567_PORT_OUT_RATE_1		0xAC
#define REG8567_PORT_OUT_RATE_2		0xAD
#define REG8567_PORT_OUT_RATE_3		0xAE

#define PORT_CTRL_ADDR(port, addr)		\
	(addr = REG_PORT_1_CTRL_0 + (port) *	\
		(REG_PORT_2_CTRL_0 - REG_PORT_1_CTRL_0))


#define REG8567_SW_MAC_ADDR_0		0x68
#define REG8567_SW_MAC_ADDR_1		0x69
#define REG8567_SW_MAC_ADDR_2		0x6A
#define REG8567_SW_MAC_ADDR_3		0x6B
#define REG8567_SW_MAC_ADDR_4		0x6C
#define REG8567_SW_MAC_ADDR_5		0x6D

#define REG8567_IND_CTRL_0			0x6E

#define TABLE_EXT_SELECT_S		5
#define TABLE_EEE			(1 << TABLE_EXT_SELECT_S)
#define TABLE_ACL			(2 << TABLE_EXT_SELECT_S)
#define TABLE_PME			(4 << TABLE_EXT_SELECT_S)
#define TABLE_LINK_MD			(5 << TABLE_EXT_SELECT_S)
#define TABLE_READ			(1 << 4)
#define TABLE_SELECT_S			2
#define TABLE_STATIC_MAC		(0 << TABLE_SELECT_S)
#define TABLE_VLAN			(1 << TABLE_SELECT_S)
#define TABLE_DYNAMIC_MAC		(2 << TABLE_SELECT_S)
#define TABLE_MIB			(3 << TABLE_SELECT_S)

#define REG8567_IND_CTRL_1			0x6F

#define TABLE_ENTRY_MASK		0x03FF
#define TABLE_EXT_ENTRY_MASK		0x0FFF

#define REG8567_IND_DATA_8			0x70
#define REG8567_IND_DATA_7			0x71
#define REG8567_IND_DATA_6			0x72
#define REG8567_IND_DATA_5			0x73
#define REG8567_IND_DATA_4			0x74
#define REG8567_IND_DATA_3			0x75
#define REG8567_IND_DATA_2			0x76
#define REG8567_IND_DATA_1			0x77
#define REG8567_IND_DATA_0			0x78

#define REG8567_IND_DATA_PME_EEE_ACL	0xA0

#define REG8567_IND_DATA_CHECK		REG_IND_DATA_6
#define REG8567_IND_MIB_CHECK		REG_IND_DATA_4
#define REG8567_IND_DATA_HI			REG_IND_DATA_7
#define REG8567_IND_DATA_LO			REG_IND_DATA_3

#define REG8567_INT_STATUS			0x7C
#define REG8567_INT_ENABLE			0x7D

#define INT_PME				(1 << 4)

#define REG8567_ACL_INT_STATUS		0x7E
#define REG8567_ACL_INT_ENABLE		0x7F

#define INT_PORT_5			(1 << 4)
#define INT_PORT_4			(1 << 3)
#define INT_PORT_3			(1 << 2)
#define INT_PORT_2			(1 << 1)
#define INT_PORT_1			(1 << 0)

#define REG8567_SW_CTRL_12			0x80
#define REG8567_SW_CTRL_13			0x81

#define SWITCH_802_1P_MASK		3
#define SWITCH_802_1P_BASE		3
#define SWITCH_802_1P_SHIFT		2

#define SW_802_1P_MAP_M			KS_PRIO_M
#define SW_802_1P_MAP_S			KS_PRIO_S

#define REG8567_SWITCH_CTRL_14		0x82

#define SW_PRIO_MAPPING_M		KS_PRIO_M
#define SW_PRIO_MAPPING_S		6
#define SW_PRIO_MAP_3_HI		0
#define SW_PRIO_MAP_2_HI		2
#define SW_PRIO_MAP_0_LO		3

#define REG8567_SW_CTRL_15			0x83
#define REG8567_SW_CTRL_16			0x84
#define REG8567_SW_CTRL_17			0x85
#define REG8567_SW_CTRL_18			0x86

#define SW_SELF_ADDR_FILTER_ENABLE	(1 << 6)

#define REG8567_SW_UNK_UCAST_CTRL		0x83
#define REG8567_SW_UNK_MCAST_CTRL		0x84
#define REG8567_SW_UNK_VID_CTRL		0x85
#define REG8567_SW_UNK_IP_MCAST_CTRL	0x86

#define SW_UNK_FWD_ENABLE		(1 << 5)
#define SW_UNK_FWD_MAP			KS_PORT_M

#define REG8567_SW_CTRL_19			0x87

#define SW_IN_RATE_LIMIT_PERIOD_M	0x3
#define SW_IN_RATE_LIMIT_PERIOD_S	4
#define SW_IN_RATE_LIMIT_16_MS		0
#define SW_IN_RATE_LIMIT_64_MS		1
#define SW_IN_RATE_LIMIT_256_MS		2
#define SW_QUEUE_BASED_OUT_RATE_LIMIT	(1 << 3)
#define SW_INS_TAG_ENABLE		(1 << 2)

#define REG8567_TOS_PRIO_CTRL_0		0x90
#define REG8567_TOS_PRIO_CTRL_1		0x91
#define REG8567_TOS_PRIO_CTRL_2		0x92
#define REG8567_TOS_PRIO_CTRL_3		0x93
#define REG8567_TOS_PRIO_CTRL_4		0x94
#define REG8567_TOS_PRIO_CTRL_5		0x95
#define REG8567_TOS_PRIO_CTRL_6		0x96
#define REG8567_TOS_PRIO_CTRL_7		0x97
#define REG8567_TOS_PRIO_CTRL_8		0x98
#define REG8567_TOS_PRIO_CTRL_9		0x99
#define REG8567_TOS_PRIO_CTRL_10		0x9A
#define REG8567_TOS_PRIO_CTRL_11		0x9B
#define REG8567_TOS_PRIO_CTRL_12		0x9C
#define REG8567_TOS_PRIO_CTRL_13		0x9D
#define REG8567_TOS_PRIO_CTRL_14		0x9E
#define REG8567_TOS_PRIO_CTRL_15		0x9F

#define TOS_PRIO_M			KS_PRIO_M
#define TOS_PRIO_S			KS_PRIO_S

#define REG8567_SW_CTRL_20			0xA3

#define SW_GMII_DRIVE_STRENGTH_S	4
#define SW_DRIVE_STRENGTH_M		0x7
#define SW_DRIVE_STRENGTH_2MA		0
#define SW_DRIVE_STRENGTH_4MA		1
#define SW_DRIVE_STRENGTH_8MA		2
#define SW_DRIVE_STRENGTH_12MA		3
#define SW_DRIVE_STRENGTH_16MA		4
#define SW_DRIVE_STRENGTH_20MA		5
#define SW_DRIVE_STRENGTH_24MA		6
#define SW_DRIVE_STRENGTH_28MA		7
#define SW_MII_DRIVE_STRENGTH_S		0

#define REG8567_SW_CTRL_21			0xA4

#define SW_IPV6_MLD_OPTION		(1 << 3)
#define SW_IPV6_MLD_SNOOP		(1 << 2)


#define REG8567_PORT_1_CTRL_12		0xB0
#define REG8567_PORT_2_CTRL_12		0xC0
#define REG8567_PORT_3_CTRL_12		0xD0
#define REG8567_PORT_4_CTRL_12		0xE0
#define REG8567_PORT_5_CTRL_12		0xF0

#define PORT_PASS_ALL			(1 << 6)
#define PORT_INS_TAG_FOR_PORT_5_S	3
#define PORT_INS_TAG_FOR_PORT_5		(1 << 3)
#define PORT_INS_TAG_FOR_PORT_4		(1 << 2)
#define PORT_INS_TAG_FOR_PORT_3		(1 << 1)
#define PORT_INS_TAG_FOR_PORT_2		(1 << 0)

#define REG8567_PORT_1_CTRL_13		0xB1
#define REG8567_PORT_2_CTRL_13		0xC1
#define REG8567_PORT_3_CTRL_13		0xD1
#define REG8567_PORT_4_CTRL_13		0xE1
#define REG8567_PORT_5_CTRL_13		0xF1

#define PORT_QUEUE_SPLIT_H		(1 << 1)
#define PORT_QUEUE_SPLIT_1		0
#define PORT_QUEUE_SPLIT_2		1
#define PORT_QUEUE_SPLIT_4		2
#define PORT_DROP_TAG			(1 << 0)

#define REG8567_PORT_1_CTRL_14		0xB2
#define REG8567_PORT_2_CTRL_14		0xC2
#define REG8567_PORT_3_CTRL_14		0xD2
#define REG8567_PORT_4_CTRL_14		0xE2
#define REG8567_PORT_5_CTRL_14		0xF2
#define REG8567_PORT_1_CTRL_15		0xB3
#define REG8567_PORT_2_CTRL_15		0xC3
#define REG8567_PORT_3_CTRL_15		0xD3
#define REG8567_PORT_4_CTRL_15		0xE3
#define REG8567_PORT_5_CTRL_15		0xF3
#define REG8567_PORT_1_CTRL_16		0xB4
#define REG8567_PORT_2_CTRL_16		0xC4
#define REG8567_PORT_3_CTRL_16		0xD4
#define REG8567_PORT_4_CTRL_16		0xE4
#define REG8567_PORT_5_CTRL_16		0xF4
#define REG8567_PORT_1_CTRL_17		0xB5
#define REG8567_PORT_2_CTRL_17		0xC5
#define REG8567_PORT_3_CTRL_17		0xD5
#define REG8567_PORT_4_CTRL_17		0xE5
#define REG8567_PORT_5_CTRL_17		0xF5

#define REG8567_PORT_1_RATE_CTRL_3		0xB2
#define REG8567_PORT_1_RATE_CTRL_2		0xB3
#define REG8567_PORT_1_RATE_CTRL_1		0xB4
#define REG8567_PORT_1_RATE_CTRL_0		0xB5
#define REG8567_PORT_2_RATE_CTRL_3		0xC2
#define REG8567_PORT_2_RATE_CTRL_2		0xC3
#define REG8567_PORT_2_RATE_CTRL_1		0xC4
#define REG8567_PORT_2_RATE_CTRL_0		0xC5
#define REG8567_PORT_3_RATE_CTRL_3		0xD2
#define REG8567_PORT_3_RATE_CTRL_2		0xD3
#define REG8567_PORT_3_RATE_CTRL_1		0xD4
#define REG8567_PORT_3_RATE_CTRL_0		0xD5
#define REG8567_PORT_4_RATE_CTRL_3		0xE2
#define REG8567_PORT_4_RATE_CTRL_2		0xE3
#define REG8567_PORT_4_RATE_CTRL_1		0xE4
#define REG8567_PORT_4_RATE_CTRL_0		0xE5
#define REG8567_PORT_5_RATE_CTRL_3		0xF2
#define REG8567_PORT_5_RATE_CTRL_2		0xF3
#define REG8567_PORT_5_RATE_CTRL_1		0xF4
#define REG8567_PORT_5_RATE_CTRL_0		0xF5

#define RATE_CTRL_ENABLE		(1 << 7)
#define RATE_RATIO_M			((1 << 7) - 1)

#define PORT_OUT_RATE_ENABLE		(1 << 7)

#define REG8567_PORT_1_RATE_LIMIT		0xB6
#define REG8567_PORT_2_RATE_LIMIT		0xC6
#define REG8567_PORT_3_RATE_LIMIT		0xD6
#define REG8567_PORT_4_RATE_LIMIT		0xE6
#define REG8567_PORT_5_RATE_LIMIT		0xF6

#define PORT_IN_PORT_BASED_S		6
#define PORT_RATE_PACKET_BASED_S	5
#define PORT_IN_FLOW_CTRL_S		4
#define PORT_IN_LIMIT_MODE_M		0x3
#define PORT_IN_LIMIT_MODE_S		2
#define PORT_COUNT_IFG_S		1
#define PORT_COUNT_PREAMBLE_S		0
#define PORT_IN_PORT_BASED		(1 << PORT_IN_PORT_BASED_S)
#define PORT_RATE_PACKET_BASED		(1 << PORT_RATE_PACKET_BASED_S)
#define PORT_IN_FLOW_CTRL		(1 << PORT_IN_FLOW_CTRL_S)
#define PORT_IN_ALL			0
#define PORT_IN_UNICAST			1
#define PORT_IN_MULTICAST		2
#define PORT_IN_BROADCAST		3
#define PORT_COUNT_IFG			(1 << PORT_COUNT_IFG_S)
#define PORT_COUNT_PREAMBLE		(1 << PORT_COUNT_PREAMBLE_S)

#define REG8567_PORT_1_IN_RATE_0		0xB7
#define REG8567_PORT_2_IN_RATE_0		0xC7
#define REG8567_PORT_3_IN_RATE_0		0xD7
#define REG8567_PORT_4_IN_RATE_0		0xE7
#define REG8567_PORT_5_IN_RATE_0		0xF7
#define REG8567_PORT_1_IN_RATE_1		0xB8
#define REG8567_PORT_2_IN_RATE_1		0xC8
#define REG8567_PORT_3_IN_RATE_1		0xD8
#define REG8567_PORT_4_IN_RATE_1		0xE8
#define REG8567_PORT_5_IN_RATE_1		0xF8
#define REG8567_PORT_1_IN_RATE_2		0xB9
#define REG8567_PORT_2_IN_RATE_2		0xC9
#define REG8567_PORT_3_IN_RATE_2		0xD9
#define REG8567_PORT_4_IN_RATE_2		0xE9
#define REG8567_PORT_5_IN_RATE_2		0xF9
#define REG8567_PORT_1_IN_RATE_3		0xBA
#define REG8567_PORT_2_IN_RATE_3		0xCA
#define REG8567_PORT_3_IN_RATE_3		0xDA
#define REG8567_PORT_4_IN_RATE_3		0xEA
#define REG8567_PORT_5_IN_RATE_3		0xFA

#define PORT_IN_RATE_ENABLE		(1 << 7)
#define PORT_RATE_LIMIT_M		((1 << 7) - 1)

#define REG8567_PORT_1_OUT_RATE_0		0xBB
#define REG8567_PORT_2_OUT_RATE_0		0xCB
#define REG8567_PORT_3_OUT_RATE_0		0xDB
#define REG8567_PORT_4_OUT_RATE_0		0xEB
#define REG8567_PORT_5_OUT_RATE_0		0xFB
#define REG8567_PORT_1_OUT_RATE_1		0xBC
#define REG8567_PORT_2_OUT_RATE_1		0xCC
#define REG8567_PORT_3_OUT_RATE_1		0xDC
#define REG8567_PORT_4_OUT_RATE_1		0xEC
#define REG8567_PORT_5_OUT_RATE_1		0xFC
#define REG8567_PORT_1_OUT_RATE_2		0xBD
#define REG8567_PORT_2_OUT_RATE_2		0xCD
#define REG8567_PORT_3_OUT_RATE_2		0xDD
#define REG8567_PORT_4_OUT_RATE_2		0xED
#define REG8567_PORT_5_OUT_RATE_2		0xFD
#define REG8567_PORT_1_OUT_RATE_3		0xBE
#define REG8567_PORT_2_OUT_RATE_3		0xCE
#define REG8567_PORT_3_OUT_RATE_3		0xDE
#define REG8567_PORT_4_OUT_RATE_3		0xEE
#define REG8567_PORT_5_OUT_RATE_3		0xFE

//---PME

#define SW_PME_OUTPUT_ENABLE		(1 << 1)
#define SW_PME_ACTIVE_HIGH		(1 << 0)

#define PORT_MAGIC_PACKET_DETECT	(1 << 2)
#define PORT_LINK_UP_DETECT		(1 << 1)
#define PORT_ENERGY_DETECT		(1 << 0)

// ACL

#define ACL_FIRST_RULE_M		0xF

#define ACL_MODE_M			0x3
#define ACL_MODE_S			4
#define ACL_MODE_DISABLE		0
#define ACL_MODE_LAYER_2		1
#define ACL_MODE_LAYER_3		2
#define ACL_MODE_LAYER_4		3
#define ACL_ENABLE_M			0x3
#define ACL_ENABLE_S			2
#define ACL_ENABLE_2_COUNT		0
#define ACL_ENABLE_2_TYPE		1
#define ACL_ENABLE_2_MAC		2
#define ACL_ENABLE_2_BOTH		3
#define ACL_ENABLE_3_IP			1
#define ACL_ENABLE_3_SRC_DST_COMP	2
#define ACL_ENABLE_4_PROTOCOL		0
#define ACL_ENABLE_4_TCP_PORT_COMP	1
#define ACL_ENABLE_4_UDP_PORT_COMP	2
#define ACL_ENABLE_4_TCP_SEQN_COMP	3
#define ACL_SRC				(1 << 1)
#define ACL_EQUAL			(1 << 0)

#define ACL_MAX_PORT			0xFFFF

#define ACL_MIN_PORT			0xFFFF
#define ACL_IP_ADDR			0xFFFFFFFF
#define ACL_TCP_SEQNUM			0xFFFFFFFF

#define ACL_RESERVED			0xF8
#define ACL_PORT_MODE_M			0x3
#define ACL_PORT_MODE_S			1
#define ACL_PORT_MODE_DISABLE		0
#define ACL_PORT_MODE_EITHER		1
#define ACL_PORT_MODE_IN_RANGE		2
#define ACL_PORT_MODE_OUT_OF_RANGE	3

#define ACL_TCP_FLAG_ENABLE		(1 << 0)

#define ACL_TCP_FLAG_M			0xFF

#define ACL_TCP_FLAG			0xFF
#define ACL_ETH_TYPE			0xFFFF
#define ACL_IP_M			0xFFFFFFFF

#define ACL_PRIO_MODE_M			0x3
#define ACL_PRIO_MODE_S			6
#define ACL_PRIO_MODE_DISABLE		0
#define ACL_PRIO_MODE_HIGHER		1
#define ACL_PRIO_MODE_LOWER		2
#define ACL_PRIO_MODE_REPLACE		3
#define ACL_PRIO_M			0x7
#define ACL_PRIO_S			3
#define ACL_VLAN_PRIO_REPLACE		(1 << 2)
#define ACL_VLAN_PRIO_M			0x7
#define ACL_VLAN_PRIO_HI_M		0x3

#define ACL_VLAN_PRIO_LO_M		0x8
#define ACL_VLAN_PRIO_S			7
#define ACL_MAP_MODE_M			0x3
#define ACL_MAP_MODE_S			5
#define ACL_MAP_MODE_DISABLE		0
#define ACL_MAP_MODE_OR			1
#define ACL_MAP_MODE_AND		2
#define ACL_MAP_MODE_REPLACE		3
#define ACL_MAP_PORT_M			0x1F

#define ACL_CNT_M			((1 << 11) - 1)
#define ACL_CNT_S			5
#define ACL_MSEC_UNIT			(1 << 4)
#define ACL_INTR_MODE			(1 << 3)

#define REG8567_PORT_ACL_BYTE_EN_MSB	0x10

#define ACL_BYTE_EN_MSB_M		0x3F

#define REG8567_PORT_ACL_BYTE_EN_LSB	0x11

#define ACL_BYTE_ENABLE			((ACL_BYTE_EN_MSB_M << 8) | 0xFF)
#define ACL_MODE_ENABLE			(0x10 << 8)

#define REG8567_PORT_ACL_CTRL_0		0x12

#define PORT_ACL_WRITE_DONE		(1 << 6)
#define PORT_ACL_READ_DONE		(1 << 5)
#define PORT_ACL_WRITE			(1 << 4)
#define PORT_ACL_INDEX_M		0xF

#define REG8567_PORT_ACL_CTRL_1		0x13

#define PORT_ACL_FORCE_DLR_MISS		(1 << 0)

#ifndef PHY_REG_CTRL
#define PHY_REG_CTRL			0

#define PHY_RESET			(1 << 15)
#define PHY_LOOPBACK			(1 << 14)
#define PHY_SPEED_100MBIT		(1 << 13)
#define PHY_AUTO_NEG_ENABLE		(1 << 12)
#define PHY_POWER_DOWN			(1 << 11)
#define PHY_MII_DISABLE			(1 << 10)
#define PHY_AUTO_NEG_RESTART		(1 << 9)
#define PHY_FULL_DUPLEX			(1 << 8)
#define PHY_COLLISION_TEST_NOT		(1 << 7)
#define PHY_HP_MDIX			(1 << 5)
#define PHY_FORCE_MDIX			(1 << 4)
#define PHY_AUTO_MDIX_DISABLE		(1 << 3)
#define PHY_REMOTE_FAULT_DISABLE	(1 << 2)
#define PHY_TRANSMIT_DISABLE		(1 << 1)
#define PHY_LED_DISABLE			(1 << 0)

#define PHY_REG_STATUS			1

#define PHY_100BT4_CAPABLE		(1 << 15)
#define PHY_100BTX_FD_CAPABLE		(1 << 14)
#define PHY_100BTX_CAPABLE		(1 << 13)
#define PHY_10BT_FD_CAPABLE		(1 << 12)
#define PHY_10BT_CAPABLE		(1 << 11)
#define PHY_MII_SUPPRESS_CAPABLE_NOT	(1 << 6)
#define PHY_AUTO_NEG_ACKNOWLEDGE	(1 << 5)
#define PHY_REMOTE_FAULT		(1 << 4)
#define PHY_AUTO_NEG_CAPABLE		(1 << 3)
#define PHY_LINK_STATUS			(1 << 2)
#define PHY_JABBER_DETECT_NOT		(1 << 1)
#define PHY_EXTENDED_CAPABILITY		(1 << 0)

#define PHY_REG_ID_1			2
#define PHY_REG_ID_2			3

#define KSZ8795_ID_HI			0x0022
#define KSZ8795_ID_LO			0x1550

#define PHY_REG_AUTO_NEGOTIATION	4

#define PHY_AUTO_NEG_NEXT_PAGE_NOT	(1 << 15)
#define PHY_AUTO_NEG_REMOTE_FAULT_NOT	(1 << 13)
#define PHY_AUTO_NEG_SYM_PAUSE		(1 << 10)
#define PHY_AUTO_NEG_100BT4		(1 << 9)
#define PHY_AUTO_NEG_100BTX_FD		(1 << 8)
#define PHY_AUTO_NEG_100BTX		(1 << 7)
#define PHY_AUTO_NEG_10BT_FD		(1 << 6)
#define PHY_AUTO_NEG_10BT		(1 << 5)
#define PHY_AUTO_NEG_SELECTOR		0x001F
#define PHY_AUTO_NEG_802_3		0x0001

#define PHY_REG_REMOTE_CAPABILITY	5

#define PHY_REMOTE_NEXT_PAGE_NOT	(1 << 15)
#define PHY_REMOTE_ACKNOWLEDGE_NOT	(1 << 14)
#define PHY_REMOTE_REMOTE_FAULT_NOT	(1 << 13)
#define PHY_REMOTE_SYM_PAUSE		(1 << 10)
#define PHY_REMOTE_100BTX_FD		(1 << 8)
#define PHY_REMOTE_100BTX		(1 << 7)
#define PHY_REMOTE_10BT_FD		(1 << 6)
#define PHY_REMOTE_10BT			(1 << 5)

#define PHY_REG_LINK_MD			0x1D

#define PHY_START_CABLE_DIAG		(1 << 15)
#define PHY_CABLE_DIAG_RESULT		0x6000
#define PHY_CABLE_STAT_NORMAL		0x0000
#define PHY_CABLE_STAT_OPEN		0x2000
#define PHY_CABLE_STAT_SHORT		0x4000
#define PHY_CABLE_STAT_FAILED		0x6000
#define PHY_CABLE_10M_SHORT		(1 << 12)
#define PHY_CABLE_FAULT_COUNTER		0x01FF

#define PHY_REG_PHY_CTRL		0x1F

#define PHY_MODE_M			0x7
#define PHY_MODE_S			8
#define PHY_STAT_REVERSED_POLARITY	(1 << 5)
#define PHY_STAT_MDIX			(1 << 4)
#define PHY_FORCE_LINK			(1 << 3)
#define PHY_POWER_SAVING_ENABLE		(1 << 2)
#define PHY_REMOTE_LOOPBACK		(1 << 1)
#endif


//Default values are used in ksz_sw.h if these are not defined.
#define PRIO_QUEUES			4

#define KS_PRIO_IN_REG			4

#define SWITCH_PORT_NUM			4

#define KSZ8795_COUNTER_NUM		0x20
#define TOTAL_KSZ8795_COUNTER_NUM	(KSZ8795_COUNTER_NUM + 4)

#define SWITCH_COUNTER_NUM		KSZ8795_COUNTER_NUM
#define TOTAL_SWITCH_COUNTER_NUM	TOTAL_KSZ8795_COUNTER_NUM

// Required for common switch control in ksz_sw.c

#define SW_D				u8
#define SW_R(sw, addr)			(sw)->reg->r8(sw, addr)
#define SW_W(sw, addr, val)		(sw)->reg->w8(sw, addr, val)
#define SW_SIZE				(1)
#define SW_SIZE_STR			"%02x"
#define port_r				port_r8
#define port_w				port_w8


#define P_BCAST_STORM_CTRL		REG_PORT_CTRL_0
#define P_PRIO_CTRL			REG_PORT_CTRL_0
#define P_TAG_CTRL			REG_PORT_CTRL_0
#define P_MIRROR_CTRL			REG_PORT_CTRL_1
#define P_802_1P_CTRL			REG_PORT_CTRL_2
#define P_STP_CTRL			REG_PORT_CTRL_2
#define P_LOCAL_CTRL			REG_PORT_CTRL_7
#define P_REMOTE_STATUS			REG_PORT_STATUS_0
#define P_FORCE_CTRL			REG_PORT_CTRL_9
#define P_NEG_RESTART_CTRL		REG_PORT_CTRL_10
#define P_SPEED_STATUS			REG_PORT_STATUS_1
#define P_LINK_STATUS			REG_PORT_STATUS_2
#define P_PASS_ALL_CTRL			REG_PORT_CTRL_12
#define P_INS_SRC_PVID_CTRL		REG_PORT_CTRL_12
#define P_DROP_TAG_CTRL			REG_PORT_CTRL_13
#define P_RATE_LIMIT_CTRL		REG_PORT_RATE_LIMIT

#define S_UNKNOWN_DA_CTRL		REG_SWITCH_CTRL_12
#define S_FORWARD_INVALID_VID_CTRL	REG_FORWARD_INVALID_VID

#define S_FLUSH_TABLE_CTRL		REG_SW_CTRL_0
#define S_LINK_AGING_CTRL		REG_SW_CTRL_0
#define S_HUGE_PACKET_CTRL		REG_SW_CTRL_1
#define S_MIRROR_CTRL			REG_SW_CTRL_3
#define S_REPLACE_VID_CTRL		REG_SW_CTRL_4
#define S_PASS_PAUSE_CTRL		REG_SW_CTRL_10
#define S_TAIL_TAG_CTRL			REG_SW_CTRL_10
#define S_802_1P_PRIO_CTRL		REG_SW_CTRL_12
#define S_TOS_PRIO_CTRL			REG_TOS_PRIO_CTRL_0
#define S_IPV6_MLD_CTRL			REG_SW_CTRL_21

#define IND_ACC_TABLE(table)		((table) << 8)

#define TAIL_TAG_OVERRIDE		(1 << 6)
#define TAIL_TAG_LOOKUP			(1 << 7)
*/

// SPI_MODE_0
// Max 48MHz


// READ/WRITE SEQUENCE
// [ADDR][R/W] - [RegAddr(A9~A2)] - [DATA][DATA][DATA][DATA]
//
//NOTE
//i) To check I2C Slave functions,we must poll BYTE_TEST
//ii) To check I2C Slave full configuration,we must poll READY bit of HW_CFG reg.
//(iii) READ: [ADDR][W(0)] - [RegAddr(A9~A2)] -[ADDR][R(1)] - [DATA][DATA][DATA][DATA]
//(iv) WRITE: [ADDR][W(0)] - [RegAddr(A9~A2)] -[DATA][DATA][DATA][DATA]
//==============================
//extern int I2C_ySendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
//SlaveAddr(W)| Reg | SetSlaveAddr(R) | BurstReceiveStart for Repeated Start | GetData |
unsigned char Ksz8567_getReg(unsigned short reg);

/* KSZ8567R Register Address Mapping
 * Register Address = 16 bits
 * -- port --------|- Page ----| Register -------
 * bit15  14 13 12 - 11 10 9 8 - 7 6 5 4  3 2 1 0
 *     Z   0  0  0   0~F : Global
 *     Z   N  N  N   0~F : Operation/PHY/RGMII/MAC/MIB....
  */

//u8 regbuf[80];
//#if(PROCESSOR == STM32F103C8)
//#else
// =================== High Level ===========================
void Ksz8567_init(){
	unsigned char reta = 0x00;
	unsigned char retb, retc;
	char str[10];

	//yI2C_Init(I2C1, 400000);//400Kbps
	//OzOLED_init(">Ksz8567.KAU<");
/*	while(1){
		printf(">Ksz8567.KAU<(%u)\r\n",reta);
		delayms(1000);
		reta++;
	}
*/
#if ((PROCESSOR == STM32F103C8) || (PROCESSOR == STM32F107VCT6))
	//Ksz8567 uses SPI.(nCS0=PB12)--STM32F103
    //Use nCS0(PB12), Mode 3, 8bit Mode,
    stmSPI2_Config(
    		1, //1Mbps
    		0, //use nCS0 on PB12
    		0, //Mode 0
    		8); //data 8 bit (or 16)

	//stmSPI2_Config(0);
#else
	stmSPI3_Config(0);
#endif

	//Check CHIP ID REGISTER (0x00)
	while(reta != 0x85){
		reta = Ksz8567_getReg(REG8567_ID1_1);
		delayms(100);
	}
	retb = Ksz8567_getReg(REG8567_ID2_1); //retb=0x67.
	if(retb == 0x67){
		retc = Ksz8567_getReg(REG8567_ID3_1);
		if(retc & 0x01){
			//OzOLED_printString("Init:Pass+",0,2,10);
			printf("KSZ8567 is Reseting..\r\n");
		}
		else{
			//OzOLED_printString("Init:Pass+",0,2,10);
			printf("KSZ8567 is Normally Operated\r\n");
		}

	}

	return;

}
// SPI frame opcodes
#define SPI_ADDR_S			12
#define SPI_ADDR_M			((1 << SPI_ADDR_S) - 1)
#define SPI_TURNAROUND_S		1

#define KS_SPIOP_RD			3
#define KS_SPIOP_WR			2

//3bit cmd + 24 bit addr + 5 bit T/R
unsigned char Ksz8567_getReg(unsigned short reg){
	char str[10];
	unsigned char sendbuf[10];
	unsigned char retb;

	//READ : 011 | A23...A0 | TA bits(5bits) | D7..D0
	//       011 A23 A22 A21 A20 A19
	//       A18 A17 A16 - A15 A14 A13 A12 A11
	//       A10 A9 A8 A7 A6 A5 A4 A3
	//       A2 A1 A0 TA TA TA TA TA
	//       D7 ... D0
	//WRIT : 010 | A23...A0 | TA bits(5bits) | D7..D0

	sendbuf[0]=	0x60;
	sendbuf[1]= (unsigned char)((reg & 0xf800) >> 11);
	sendbuf[2]= (unsigned char)((reg & 0x07f8) >> 3);
	sendbuf[3]= (unsigned char)((reg & 0x0007) << 5);

	nCS_KSZ8567_L;
#if ((PROCESSOR == STM32F103C8) || (PROCESSOR == STM32F107VCT6))
	stmSpi2WrByte(sendbuf[0]);
	stmSpi2WrByte(sendbuf[1]);
	stmSpi2WrByte(sendbuf[2]);
	stmSpi2WrByte(sendbuf[3]);

	retb = stmSpi2RdByte();
#else
	stmSpi3WrByte(sendbuf[0]);
	stmSpi3WrByte(sendbuf[1]);
	stmSpi3WrByte(sendbuf[2]);
	stmSpi3WrByte(sendbuf[3]);

	retb = stmSpi3RdByte();
#endif
	nCS_KSZ8567_H;
	//sprintf(str,"%02x>%02x",reg,retb);
	//OzOLED_printString(str,0,4,12);
	printf("[Reg%x] = 0x%02x (%02x %02x %02x %02x)\r\n",reg,retb,sendbuf[0],sendbuf[1],sendbuf[2],sendbuf[3]);
	return retb;
}
unsigned char Ksz8567_setReg(unsigned short reg, unsigned int value, unsigned char valen)
{
	char str[10];
	unsigned char sendbuf[10];
	unsigned char retb;

	//READ : 011 | A23...A0 | TA bits(5bits) | D7..D0
	//       011 A23 A22 A21 A20 A19
	//       A18 A17 A16 - A15 A14 A13 A12 A11
	//       A10 A9 A8 A7 A6 A5 A4 A3
	//       A2 A1 A0 TA TA TA TA TA
	//       D7 ... D0
	//WRIT : 010 | A23...A0 | TA bits(5bits) | D7..D0

	sendbuf[0]=	0x40; //Write Code
	//Register Address Setup
	sendbuf[1]= (unsigned char)((reg & 0xf800) >> 11);
	sendbuf[2]= (unsigned char)((reg & 0x07f8) >> 3);
	sendbuf[3]= (unsigned char)((reg & 0x0007) << 5);
	//Value
	if(valen  == 4)	{//Value-Burst
		sendbuf[4]= (unsigned char)((value & 0xff000000)>>24);
		sendbuf[5]= (unsigned char)((value & 0xff0000)>>16);
		sendbuf[6]= (unsigned char)((value & 0xff00)>>8);
		sendbuf[7]= (unsigned char)((value & 0xff));
	}else{
		sendbuf[4]= (unsigned char)value;
	}

	nCS_KSZ8567_L;
#if ((PROCESSOR == STM32F103C8) || (PROCESSOR == STM32F107VCT6))
	stmSpi2WrByte(sendbuf[0]);
	stmSpi2WrByte(sendbuf[1]);
	stmSpi2WrByte(sendbuf[2]);
	stmSpi2WrByte(sendbuf[3]);

	if(valen  == 4)	{
		stmSpi2WrByte(sendbuf[4]);
		stmSpi2WrByte(sendbuf[5]);
		stmSpi2WrByte(sendbuf[6]);
		stmSpi2WrByte(sendbuf[7]);
	}else{
		stmSpi2WrByte(sendbuf[4]);//Value
	}
#else
	stmSpi3WrByte(sendbuf[0]);
	stmSpi3WrByte(sendbuf[1]);
	stmSpi3WrByte(sendbuf[2]);
	stmSpi3WrByte(sendbuf[3]);
#endif
	nCS_KSZ8567_H;
	//sprintf(str,"%02x>%02x",reg,retb);
	//OzOLED_printString(str,0,4,12);
	printf("[Reg%x] = 0x%02x (%02x %02x %02x %02x)\r\n",reg,retb,sendbuf[0],sendbuf[1],sendbuf[2],sendbuf[3]);
	return retb;
}
void Ksz8567_Loop(){
	unsigned short i,j;
	char str[10];
	long dval = 0;


	Ksz8567_init();
	Ksz8567_setReg(0x103, 0x02 | 0x04, 1); //Set SYNCLKO Source frm Port 1 recoved Clock

	i=0;
	while(1){
		for(i=0;i<=5;i++){
			for(j=0;j<0x20;j++){
				Ksz8567_getReg(i*256 + j);//Ksz8567_REGS[i]);
				delayms(2000);
			}

		}
	};
}
//#endif //PROCESSOR
