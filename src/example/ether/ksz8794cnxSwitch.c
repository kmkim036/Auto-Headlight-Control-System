/*
  KSZ8794CNX Ethernet Switch
  Author: YOON
  REF: micrel_switch_usage_guide.pdf

 The Micrel switch is usually connected to a MAC controller through a MII/RGMII interface.
 From the MAC controller point of view, it can be just a PHY.
 Therefore the switch device is implemented as a Linux phy device to the network driver.
 As the switch is actually accessed through SPI or I2C rather than MIIM, the driver creates a virtual MDIO bus for the phy devices
to be created.
 - The first PHY 0 is for the whole switch,
 - PHY 1 is for port 1, PHY 2 for port 2, and so on.
 The switch driver can also simulate the standard PHY registers if required,
 but that is generally not needed if the provided APIs are used.

 The first step to make the Micrel switch work is to make sure the MII interface is connected
 correctly on the system board and the correct MII mode is used.
 - The switch port connected with that MII interface is generally the last port of the switch,
   and is called the host port.

The default operation of the switch driver is to act as a PHY to the host network driver.
The only functions provided then are to notify the host for link change and change all the port speed to a
specific one.
The driver can report individual port speed, but the one got from the network driver is the fixed one from the MII interface,
as most network devices use that speed information to program the registers for proper operation.
If that is the only requirement needed the network driver does not need much modification.

For some situations it requires each port to be treated as a network device.
As the network devices are created in the host network driver, it requires modifications to that driver to support these
special functions.

The simplest way is to make each port a network device.
To act as a single unit a bridge needs to be created to bind those network ports.

Each port is treated as a network device for use in STP application,
but it is still preferred to have a main device to run some other applications.
In this case there is a main device for a whole  switch, and child devices for each port.

A bridge is then created to run STP. There are some issues in this method as the received frame is passed to both child and parent devices.
The driver tries to minimize these problems by passing only STP related frames to the child device.

There is a problem for Precision Time Protocol (PTP) support when using STP. As the PTP
event messages cannot be blindly forwarded by a software bridge, the standard software operation does not work.

Some applications like to have the control to send packets to specific port but do not want a
bridge. In this case a virtual VLAN device is created for each port so the application has an
option to use the VLAN device or the main device. There is a performance hit to pass two
frames to both VLAN and main devices, so the driver only does this for certain situations, like
handling PTP message or MRP frames.

[Distributed Switch Architecture]
An alternate way is to use Distributed Switch Architecture (DSA), a network layer to utilize the
ports as network devices. It requires almost no modifications to the host network driver.
However, as the host network driver is impervious to a switch in being used, everything is
forwarded by software and the switch hardware is not utilized.
Micrel switches use a tail tagging feature to know the received port and specify the destination
port. Once enabled the tail tag is required in all frames to be transmitted.
The DSA driver automatically adds the steps to prepare a frame with proper tail tag when sent from DSA ports,
but if the host network driver needs to send frames through its main network device, it needs to
call the special tail_xmit function to add a tail tag.
It requires modifying some kernel files related to DSA to add Micrel DSA support.
The kernel configuration CONFIG_NET_DSA_TAIL_TAG is defined when Micrel DSA support is
selected.

-- DLR : Device Level Ring (DLR)
-- HSR :

-TOTAL_PORT_NUM
-SW_PORT_NUM

[TBD]
KSZ8462HL(2 port SW) Source address filtering for implementing ring topologies
 */
#include "yInc.h"
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "yLib/include/yInc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
//#include "lwip/include/lwipopts.h"
#include "stm32f4x7_eth_bsp.h"
#include "stp_bridge.h"
#include "stp.h"
#include "yLib/eth/include/ethopts.h"

#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "ySpiDrv.h"
extern void stmSpi2_Config(unsigned char nCS);
extern unsigned char stmSpi2RdByte();
extern void stmSpi2WrByte(unsigned char inbyte);
//use nCS0 of PB12 : 103
#define nCS0_KSZ_1 nCS0_H
#define nCS0_KSZ_0 nCS0_L

#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
extern void stmSPI1_Config(unsigned sckMbps,unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern unsigned char stmSpi1RdByte();
extern void stmSpi1WrByte(unsigned char inbyte);
#else

#endif

#if	(SWITCH_ID == SWITCH_KSZ8794)
#include "ether/include/ksz_common.h"
#include "ether/include/ksz8794.h"

//struct ksz_sw g_sw;
//struct ksz_port;
//==========================================
unsigned long Ksz_getReg(unsigned char reg, unsigned char len);

// Required for common switch control in ksz_sw.c
//#define SW_D u16
//#define SW_R(addr) Ksz_getReg16(addr)
//#define SW_W(addr,val) Ksz_setReg16(addr,val)

#define SW_D				u8
//#define SW_R( addr)			(sw)->reg->r8( addr)
//#define SW_W( addr, val)		(sw)->reg->w8( addr, val)
#define SW_R8(addr)			Ksz_getReg(addr,1) //reg->r8(addr)
#define SW_W8(addr, val)	Ksz_setReg(addr,val, 1) //	reg->w8(addr, val)

#define SW_SIZE				(1)
#define SW_SIZE_STR			"%02x"
//#define Ksz_port_r8				Ksz_port_r8 //#define Ksz_port_r8 Ksz_port_r816
//#define Ksz_port_w8				Ksz_port_w8

//============ GET/PUT for KSZ over SPI. =====================

unsigned char Ksz_getReg8(unsigned char reg){
	char str[10];
	unsigned char sendbuf[10];
	unsigned char retb;

	sendbuf[0]=0x60 | ((reg & 0x80) >> 7); //011-zzzz-A7
	sendbuf[1]= ((reg & 0x7f) << 1) | 1; //A6...A0-TR(?)

	nCS0_KSZ_0;
  	stmSpiWrByte(sendbuf[0]);//stmSpi3WrByte(sendbuf[0]);
  	stmSpiWrByte(sendbuf[1]);//stmSpi3WrByte(sendbuf[1]);
  	retb = stmSpiRdByte();//retb = stmSpi3RdByte();
	nCS0_KSZ_1;	//GPIOA->BSRRL |= GPIO_Pin_15; // set PA15 (CS) high

	return retb;
}

unsigned short Ksz_getReg16(unsigned char reg){
	char str[10];
	unsigned char sendbuf[10];
	unsigned short rets;

	sendbuf[0]=0x60 | ((reg & 0x80) >> 7); //011-zzzz-A7
	sendbuf[1]= ((reg & 0x7f) << 1) | 1; //A6...A0-TR(?)

	nCS0_KSZ_0;//GPIOA->BSRRH |= GPIO_Pin_15; // set (CS) low
  	stmSpiWrByte(sendbuf[0]);
	stmSpiWrByte(sendbuf[1]);
	rets = (unsigned short)stmSpiRdByte();
	rets = rets << 8;

	rets = rets | stmSpiRdByte();
	nCS0_KSZ_1;//GPIOA->BSRRL |= GPIO_Pin_15; // set (CS) high

	return rets;
}
unsigned long Ksz_getReg32(unsigned char reg){
	char str[10];
	unsigned char sendbuf[10];
	unsigned long retl;

	sendbuf[0]=0x60 | ((reg & 0x80) >> 7); //011-zzzz-A7
	sendbuf[1]= ((reg & 0x7f) << 1) | 1; //A6...A0-TR(?)

	nCS0_KSZ_0;//set (CS) low
  	stmSpiWrByte(sendbuf[0]);
	stmSpiWrByte(sendbuf[1]);
	retl = (unsigned long)stmSpiRdByte();
	retl = retl << 8;

	retl |= (unsigned long) stmSpiRdByte();
	retl = retl << 8;

	retl |= (unsigned long) stmSpiRdByte();
	retl << 8;

	retl = retl | (unsigned long)stmSpiRdByte();
	nCS0_KSZ_1; // set PB12 (CS) high

	return retl;
}


//3bit RDcmd(011) + 12 bit addr + 1 bit T/R
unsigned long Ksz_getReg(unsigned char reg, unsigned char len){
	unsigned long ret;

  	switch(len){
  	case 1:
  		ret = Ksz_getReg8(reg);
  		break;
  	case 2:
  		ret = Ksz_getReg16(reg);
  		break;
  	case 4:
  		ret = Ksz_getReg32(reg);
  		break;
  	}
	return ret;
}


void Ksz_setReg8(unsigned char reg, unsigned char val){
	char str[10];
	unsigned char sendbuf[10];

	sendbuf[0]=0x40 | ((reg & 0x80) >> 7); //010-zzzz-A7
	sendbuf[1]= ((reg & 0x7f) << 1) | 1; //A6...A0-TR(?)

	nCS0_KSZ_0;//GPIOA->BSRRH |= GPIO_Pin_15; // set PA15 (CS) low
  	stmSpiWrByte(sendbuf[0]);
	stmSpiWrByte(sendbuf[1]);
	stmSpiWrByte(val);
	nCS0_KSZ_1;//GPIOA->BSRRL |= GPIO_Pin_15; // set PA15 (CS) high
}

void Ksz_setReg16(unsigned char reg, unsigned short val){
	char str[10];
	unsigned char sendbuf[10];

	sendbuf[0]=0x40 | ((reg & 0x80) >> 7); //010-zzzz-A7
	sendbuf[1]= ((reg & 0x7f) << 1) | 1; //A6...A0-TR(?)

	nCS0_KSZ_0;//GPIOA->BSRRH |= GPIO_Pin_15; // set PA15 (CS) low
  	stmSpiWrByte(sendbuf[0]);
  	stmSpiWrByte(sendbuf[1]);
	stmSpiWrByte((val & 0xff00) >> 8); //msb
	stmSpiWrByte(val & 0xff); //lsb
	nCS0_KSZ_1;//GPIOA->BSRRL |= GPIO_Pin_15; // set PA15 (CS) high
}
void Ksz_setReg32(unsigned char reg, unsigned long val){
	char str[10];
	unsigned char sendbuf[10];

	sendbuf[0]=0x40 | ((reg & 0x80) >> 7); //010-zzzz-A7
	sendbuf[1]= ((reg & 0x7f) << 1) | 1; //A6...A0-TR(?)

	nCS0_KSZ_0; //GPIOA->BSRRH |= GPIO_Pin_15; // set PA15 (CS) low
  	stmSpiWrByte(sendbuf[0]);
  	stmSpiWrByte(sendbuf[1]);
	stmSpiWrByte((val & 0xff000000) >> 24); //msb
	stmSpiWrByte((val & 0x00ff0000) >> 16);
	stmSpiWrByte((val & 0x0000ff00) >> 8);
	stmSpiWrByte(val & 0x000000ff); //lsb
	nCS0_KSZ_1;//GPIOA->BSRRL |= GPIO_Pin_15; // set PA15 (CS) high
}

//3bit WRcmd(010) + 12 bit addr + 1 bit T/R
void Ksz_setReg(unsigned char reg, unsigned char val,unsigned char len){

  	switch(len){
  	case 1:
  		Ksz_setReg8(reg,(unsigned char)val);
  		break;
  	case 2:
  		Ksz_setReg16(reg,(unsigned short)val);
  		break;
  	case 4:
  		Ksz_setReg32(reg,val);
  		break;
  	}
}
/*
u32 ksz_ptp_read(struct Ksz_ptp_info *ptp, int addr, u32 reg)
{
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);
	u32 ret = 0xffffffff;

	switch (addr) {
	case ADDR_16:
		ret = sw_r16( reg);
		break;
	case ADDR_32:
		ret = sw_r32( reg);
		break;
	case ADDR_8:
		ret = sw_r8( reg);
		break;
	}
	return ret;
}

void ksz_ptp_write(struct Ksz_ptp_info *ptp, int addr, u32 reg, u32 val)
{
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);

	switch (addr) {
	case ADDR_16:
		sw_w16( reg, val);
		break;
	case ADDR_32:
		sw_w32( reg, val);
		break;
	case ADDR_8:
		sw_w8( reg, val);
		break;
	}
}
*/



//================ switch functions ======================================
//bitwise write for switch.
void Ksz_sw_wCfg(unsigned int addr, unsigned char bits, int set){
	SW_D data;
	data = SW_R8(addr);
	if(set) data |= bits;
	else     data &= ~bits;
	SW_W8(addr,data);
}

/**
 * sw_chk - check switch register bits
 * @sw:		The switch instance.
 * @addr:	The address of the switch register.
 * @bits:	The data bits to check.
 *
 * This function checks whether the specified bits of the switch register are
 * set or not.
 *
 * Return 0 if the bits are not set.
 */
int Ksz_sw_chk(u32 addr, SW_D bits)
{
	SW_D data;

	data = SW_R8(addr);
	return (data & bits) == bits;
}

/**
 * sw_w_ext_table - write a byte of data to switch table
 * @sw:		The switch instance.
 * @table:	The table selector.
 * @addr:	The address of the table entry.
 * @offset:	The offset of the extended table.
 * @data:	Data to be written.
 *
 * This routine writes a byte of data to the extended table of the switch.
 * Hardware is locked to minimize corruption of written data.
 */
static void Ksz_sw_w_ext_table(int table, u16 addr, u16 offset,	u8 data)
{
	u16 ctrl_addr;
	ctrl_addr = IND_ACC_TABLE(table | addr) | offset; //Indirect Write
	Ksz_setReg16(REG_IND_CTRL_0, ctrl_addr);
	Ksz_setReg8(REG_IND_DATA_PME_EEE_ACL, data);
}

/**
 * sw_p_ext_table - write a number of bytes of data to switch table
 * @sw:		The switch instance.
 * @table:	The table selector.
 * @addr:	The address of the table entry.
 * @offset:	The offset of the extended table.
 * @data:	Data to be written.
 * @cnt:	Numbe of bytes to write.
 *
 * This routine writes several bytes of data to the extended table of the
 * switch.
 * Hardware is locked to minimize corruption of written data.
 */
static void Ksz_sw_p_ext_table(struct ksz_sw *sw, int table, u16 addr, u16 offset,
	u8 *data, int cnt)
{
	u16 ctrl_addr;
	int i;

	ctrl_addr = IND_ACC_TABLE(table | addr) | offset;

	Ksz_setReg16(REG_IND_CTRL_0, ctrl_addr);
	Ksz_setReg8(REG_IND_DATA_PME_EEE_ACL, data[0]);
	for (i = 1; i < cnt; i++) {
		++offset;
		Ksz_setReg8(REG_IND_CTRL_1, offset);
		Ksz_setReg8(REG_IND_DATA_PME_EEE_ACL, data[i]);
	}
}

#define KSZSW_REGS_SIZE			0x100

static struct Ksz_sw_regs {
	int start;
	int end;
} sw_regs_range[] = {
	{ 0x0000, 0x0100 },
	{ 0, 0 }
};

int Ksz_check_sw_reg_range(unsigned addr)
{
	struct Ksz_sw_regs *range = sw_regs_range;

	while (range->end > range->start) {
		if (range->start <= addr && addr < range->end)
			return true;
		range++;
	}
	return false;
}

int Ksz_sw_regs_get( u32 reg, size_t count, char *buf)
{
	size_t i;
	SW_D *addr;

	addr = (SW_D *) buf;
	for (i = 0; i < count; i += SW_SIZE, reg += SW_SIZE, addr++) {
		*addr = 0;
		if (Ksz_check_sw_reg_range(reg))
			*addr = SW_R8( reg);
	}
	return i;
}

int Ksz_sw_regs_set( u32 reg, size_t count, char *buf)
{
	size_t i;
	SW_D *addr;

	addr = (SW_D *) buf;
	for (i = 0; i < count; i += SW_SIZE, reg += SW_SIZE, addr++) {
		if (Ksz_check_sw_reg_range(reg))
			SW_W8( reg, *addr);
	}
	return i;
}
//================ port functions ========================================
void Ksz_port_r8(unsigned int port, int offset, unsigned char *retVal){
	unsigned int addr;
	SW_D data;

	PORT_CTRL_ADDR(port,addr);
	addr += offset;
	*retVal = SW_R8(addr);
}

Ksz_port_w8(unsigned int port, int offset, unsigned char val){
	unsigned int addr;

	PORT_CTRL_ADDR(port,addr);
	addr += offset;

	SW_W8(addr,val);
}

//bitwise write per port for set or reset.
void Ksz_port_w8Cfg(unsigned int port, int offset, unsigned char bits, int set){
	unsigned int addr;
	SW_D data;

	PORT_CTRL_ADDR(port,addr);
	addr += offset;
	data = SW_R8(addr);

	if(set) data |= bits;
	else     data &= ~bits;
	SW_W8(addr,data);
}

/**
 * port_chk - check port register bits
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the port register.
 * @bits:	The data bits to check.
 *
 * This function checks whether the specified bits of the port register are set
 * or not.
 *
 * Return 0 if the bits are not set.
 */
int Ksz_port_chk(int port, int offset, SW_D bits)
{
	u32 addr;
	SW_D data;

	PORT_CTRL_ADDR(port, addr);
	addr += offset;
	data = SW_R8(addr);
	return (data & bits) == bits;
}



/* Switch features and bug fixes. */
#define STP_SUPPORT			(1 << 0)
#define VLAN_PORT			(1 << 1)
#define VLAN_PORT_REMOVE_TAG		(1 << 2)
#define VLAN_PORT_TAGGING		(1 << 3)
#define VLAN_PORT_START			200
#define SW_VLAN_DEV			(1 << 4)

#define USE_FEWER_PORTS			(1 << 18)
#define DLR_HW				(1 << 24)
#define HSR_HW				(1 << 25)

#define DSA_SUPPORT			(1 << 28)
#define DIFF_MAC_ADDR			(1 << 30)

/* Software overrides. */
#define PAUSE_FLOW_CTRL			(1 << 0)
#define FAST_AGING			(1 << 1)

#define ACL_INTR_MONITOR		(1 << 17)

#define TAIL_PRP_0			(1 << 24)
#define TAIL_PRP_1			(1 << 25)

#define TAG_REMOVE			(1 << 30)
#define TAIL_TAGGING			(1 << 31)


enum phy_state {
	PHY_DOWN = 0,
	PHY_STARTING,
	PHY_READY,
	PHY_PENDING,
	PHY_UP,
	PHY_AN,
	PHY_RUNNING,
	PHY_NOLINK,
	PHY_FORCING,
	PHY_CHANGELINK,
	PHY_HALTED,
	PHY_RESUMING
};
typedef enum {
	PHY_INTERFACE_MODE_NA,
	PHY_INTERFACE_MODE_MII,
	PHY_INTERFACE_MODE_GMII,
	PHY_INTERFACE_MODE_SGMII,
	PHY_INTERFACE_MODE_TBI,
	PHY_INTERFACE_MODE_REVMII,
	PHY_INTERFACE_MODE_RMII,
	PHY_INTERFACE_MODE_RGMII,
	PHY_INTERFACE_MODE_RGMII_ID,
	PHY_INTERFACE_MODE_RGMII_RXID,
	PHY_INTERFACE_MODE_RGMII_TXID,
	PHY_INTERFACE_MODE_RTBI,
	PHY_INTERFACE_MODE_SMII,
	PHY_INTERFACE_MODE_XGMII,
	PHY_INTERFACE_MODE_MOCA,
	PHY_INTERFACE_MODE_QSGMII,
	PHY_INTERFACE_MODE_MAX,
} phy_interface_t;



struct ksz_sw_cached_regs {
	u16 ptp_clk_ctrl;
};

// Switch features and bug fixes.
#define STP_SUPPORT			(1 << 0)
#define VLAN_PORT			(1 << 1)
#define VLAN_PORT_REMOVE_TAG		(1 << 2)
#define VLAN_PORT_TAGGING		(1 << 3)
#define VLAN_PORT_START			200
#define SW_VLAN_DEV			(1 << 4)
#define MRP_SUPPORT			(1 << 5)
#define DLR_HW				(1 << 24)
#define HSR_HW				(1 << 25)
#define DSA_SUPPORT			(1 << 28)
#define DIFF_MAC_ADDR			(1 << 30)
#ifdef CONFIG_1588_PTP
#define PTP_HW				(1 << 31)
#endif

// Software overrides.
#define PAUSE_FLOW_CTRL			(1 << 0)
#define FAST_AGING			(1 << 1)
#define TAIL_PRP_0			(1 << 24)
#define TAIL_PRP_1			(1 << 25)
#define TAG_REMOVE			(1 << 30)
#define TAIL_TAGGING			(1 << 31)



#define STATIC_MAC_TABLE_ADDR		0x0000FFFF
#define STATIC_MAC_TABLE_FWD_PORTS	0x00070000
#define STATIC_MAC_TABLE_VALID		0x00080000
#define STATIC_MAC_TABLE_OVERRIDE	0x00100000
#define STATIC_MAC_TABLE_USE_FID	0x00200000
#define STATIC_MAC_TABLE_FID		0x03C00000

#define STATIC_MAC_FWD_PORTS_SHIFT	16
#define STATIC_MAC_FID_SHIFT		22

#define DYNAMIC_MAC_TABLE_ADDR		0x0000FFFF
#define DYNAMIC_MAC_TABLE_FID		0x000F0000
#define DYNAMIC_MAC_TABLE_SRC_PORT	0x00300000
#define DYNAMIC_MAC_TABLE_TIMESTAMP	0x00C00000
#define DYNAMIC_MAC_TABLE_ENTRIES	0xFF000000

#define DYNAMIC_MAC_TABLE_ENTRIES_H	0x03
#define DYNAMIC_MAC_TABLE_MAC_EMPTY	0x04
#define DYNAMIC_MAC_TABLE_RESERVED	0x78
#define DYNAMIC_MAC_TABLE_NOT_READY	0x80

#define DYNAMIC_MAC_FID_SHIFT		16
#define DYNAMIC_MAC_SRC_PORT_SHIFT	20
#define DYNAMIC_MAC_TIMESTAMP_SHIFT	22
#define DYNAMIC_MAC_ENTRIES_SHIFT	24
#define DYNAMIC_MAC_ENTRIES_H_SHIFT	8


//=== port functions =======

Ksz_port_cfg_force_flow_ctrl( int p, int set){
	Ksz_port_w8Cfg(p,P_STP_CTRL, PORT_FORCE_FLOW_CTRL, set);
}
//== mirroring
/* Mirroring */

inline void Ksz_port_cfg_mirror_sniffer( int p, int set)
{
	Ksz_port_w8Cfg(p,P_MIRROR_CTRL, PORT_MIRROR_SNIFFER, set);
}

inline void Ksz_port_cfg_mirror_rx( int p, int set)
{
	Ksz_port_w8Cfg(p, P_MIRROR_CTRL, PORT_MIRROR_RX, set);
}

inline void Ksz_port_cfg_mirror_tx( int p, int set)
{
	Ksz_port_w8Cfg(p, P_MIRROR_CTRL, PORT_MIRROR_TX, set);
}

inline void Ksz_sw_cfg_mirror_rx_tx( int set)
{
	//Ksz_sw_cfg( S_MIRROR_CTRL, SW_MIRROR_RX_TX, set);
}


static int Ksz_sw_get_Family_Chip_id(struct ksz_sw *sw, u8 *id1, u8 *id2)
{
	int id;
	int i;
	int j;

	//Check CHIP ID REGISTER (@0x00)
	id = Ksz_getReg16(REG_CHIP_ID0);
	i = id;
	j = i & 0xf0;
	i >>= 8;
	*id1 = (u8) i;
	*id2 = (u8) j;
	return id;
}

void Ksz_sw_cfg_tail_tag(int enable)
{
	Ksz_sw_wCfg(S_TAIL_TAG_CTRL, SW_TAIL_TAG_ENABLE, enable);
}

extern void stmSPI_Config(unsigned char whichSPI, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);

boolean Ksz_ConfigSPI_CheckWeHave(unsigned char whichSPI, unsigned char familyID, unsigned char chipID)
{
	unsigned char reta = 0x00;
	unsigned char retb;
	char str[10];

	//For KSZ8794, SPI Mode is 0.
	stmSPI_Config(whichSPI, 0, 0, 8);	//KSZ8794 uses SPI2 in KONG.(nCS0=PB12)//was KSZ8794 uses SPI3.(nCS0=PA15)

	//Check CHIP ID REGISTER (@0x00)
	reta = Ksz_getReg8(REG_CHIP_ID0);
	printf("reta = %02x\r\n",reta);
	yAssert (reta == familyID); //KSZ8794 = 0x87 , KSZ8463 = 0x84

	retb = Ksz_getReg8(REG_CHIP_ID1); //ChipID | RevID | StartSwitchBit
	printf("Reg0/1 = %02x/%02x\r\n",reta,retb);
	if((retb & 0xf0) == 0x60){
		printf("We have KSZ8794\r\n");
	}else{
		printf("??%02x\r\n",retb & 0xf0);
	}
	delayms(100);
	//Check Start Switch
	if(retb & 0x01){
		stmOzOLED_printString("Init>Pass+",0,2,10);
		printf("Init>Pass+\r\n");
		return true;
	}
	return false;
}

void Ksz8794_Basic_Loop(){
	int i;
	char str[10];
	long dval = 0;


	Ksz_ConfigSPI_CheckWeHave(USE_SPI2,0x87,0x94); //KSZ8794


	//Ksz_sw_setup();

	i=0;

	int portIndex;
	unsigned char reg;

	while(1){
		for (portIndex = 0; portIndex < 3; portIndex++){
		//check link status
		reg = Ksz_getReg(0x1e + (portIndex<<4), 1); //0x1e(PORT1), 0x2e(PORT2) Port n Status 2 Registers
		if (reg & (1 << 5)) //bit 5 Link Good.
		{
			// link is now good
			printf("Port %d> Link Good\r\n", portIndex+1);
		}
		else if (!(reg & (1 << 5)))
		{
			// link is now down
			printf("Port %d> Link Down\r\n", portIndex+1);
		}
		}
		delayms(1000);
	}

	while(1){
			Ksz_getReg(i, 1);//KSZ8794_REGS[i]);
			delayms(1000);
			i++;
			if(i==0xff)	i=0;
	};

}

#if 0

/**
 * sw_cfg_port_base_vlan - configure port-based VLAN membership
 * @sw:		The switch instance.
 * @port:	The port index.
 * @member:	The port-based VLAN membership.
 *
 * This routine configures the port-based VLAN membership of the port.
 */
static void Ksz_sw_cfg_port_base_vlan(int port, u8 member)
{
	SW_D data;

	Ksz_port_r8(port, P_MIRROR_CTRL, &data);
	data &= ~PORT_VLAN_MEMBERSHIP;
	data |= (member & g_sw.PORT_MASK);
	Ksz_port_w8(port, P_MIRROR_CTRL, data);

	g_sw.info.port_cfg[port].member = member;
}  /* sw_cfg_port_base_vlan */
static void Ksz_sw_cfg_each_port(int p, int cpu)
{
	if (!cpu)
		Ksz_sw_cfg_port_base_vlan(p, g_sw.HOST_MASK | (1 << p));
	else
		Ksz_sw_cfg_port_base_vlan(p, g_sw.PORT_MASK);
}

inline int Ksz_port_chk_mirror_sniffer( int p)
{
	return Ksz_port_chk(p,P_MIRROR_CTRL, PORT_MIRROR_SNIFFER);
}

inline int Ksz_port_chk_mirror_rx( int p)
{
	return Ksz_port_chk( p,P_MIRROR_CTRL, PORT_MIRROR_RX);
}

inline int Ksz_port_chk_mirror_tx( int p)
{
	return Ksz_port_chk(p,	P_MIRROR_CTRL, PORT_MIRROR_TX);
}

inline void Ksz_port_cfg_rmv_tag(int p, int remove)
{
	Ksz_port_w8Cfg( p, P_TAG_CTRL, PORT_REMOVE_TAG, remove);
}

inline int Ksz_sw_chk_mirror_rx_tx()
{
	return Ksz_sw_chk(S_MIRROR_CTRL, SW_MIRROR_RX_TX);
}

void Ksz_sw_setup_mirror()
{
	int port;

	for (port = 0; port < TOTAL_PORT_NUM; port++) {
		Ksz_port_cfg_mirror_sniffer( port, 0);
		Ksz_port_cfg_mirror_rx( port, 0);
		Ksz_port_cfg_mirror_tx( port, 0);
	}
	Ksz_sw_cfg_mirror_rx_tx( 0);
}

//ACL
inline void Ksz_port_cfg_acl(int p, int set)
{
	Ksz_port_w8Cfg(p,REG_PORT_CTRL_5, PORT_ACL_ENABLE, set);
}

inline int Ksz_port_chk_acl(int p)
{
	return Ksz_port_chk(p,	REG_PORT_CTRL_5, PORT_ACL_ENABLE);
}
/**
 * sw_w_acl_table - write to ACL table
 * @sw:		The switch instance.
 * @port:	The port index.
 * @addr:	The address of the table entry.
 * @acl:	The ACL entry.
 *
 * This routine writes an entry of the ACL table of the port.
 */
void Ksz_sw_w_acl_table(int port, u16 addr,	struct ksz_acl_table *acl)
{
	u8 data[20];

	if (ACL_MODE_LAYER_2 == acl->mode && ACL_ENABLE_2_COUNT == acl->enable)	return;
	acl->data[0] = 0xff;
	memset(data, 0, sizeof(data));

	/* TBD
	wait_for_acl_table(sw, port);
	set_acl_action_info(acl, data);
	set_acl_table_info(acl, data);
	set_acl_ruleset_info(acl, data);
	sw_w_acl_hw(sw, port, addr, data, ACL_BYTE_ENABLE);
	memcpy(acl->data, data, ACL_TABLE_LEN);
	 */
}
static void Ksz_sw_reset_acl()
{
	struct ksz_port_cfg *cfg;
	int port;

	for (port = 0; port < g_sw.mib_port_cnt; port++) {
		cfg = &g_sw.info.port_cfg[port];
		memset(cfg->acl_info, 0, sizeof(struct ksz_acl_table) * 	ACL_TABLE_ENTRIES);
		cfg->acl_index = cfg->acl_act_index = cfg->acl_rule_index = 0;
	}
}
static void Ksz_sw_reset_acl_hw()
{
	struct ksz_port_cfg *cfg;
	struct ksz_acl_table *acl;
	int i;
	int acl_on;
	int port;

	Ksz_sw_reset_acl();

	for (port = 0; port < g_sw.mib_port_cnt; port++) {
		//port = chk_last_port(sw, port);
		acl_on = Ksz_port_chk_acl(port);
		if (!acl_on)
			Ksz_port_cfg_acl(port, true);

		cfg = &g_sw.info.port_cfg[port];
		for (i = 0; i < ACL_TABLE_ENTRIES; i++) {
			acl = &cfg->acl_info[i];
			acl->mode = 0;
			acl->ruleset = 0;
			Ksz_sw_w_acl_table(port, i, acl);
		}

		if (!acl_on)
			Ksz_port_cfg_acl(port, false);
	}
}

//==========================
//inline void Ksz_sw_reset()
void Ksz_sw_reset()
{
	int i;
	Ksz_setReg8(REG_POWER_MANAGEMENT_1,	SW_SOFTWARE_POWER_DOWN << SW_POWER_MANAGEMENT_MODE_S);
	Ksz_setReg8(REG_POWER_MANAGEMENT_1,	0);
	for (i = 1; i < g_sw.mib_port_cnt; i++) {
		g_sw.info.port_cfg[i].acl_byte_enable = ACL_BYTE_ENABLE;
		Ksz_sw_w_ext_table(TABLE_ACL, i + 1, REG_PORT_ACL_BYTE_EN_MSB, ACL_BYTE_EN_MSB_M);
		Ksz_sw_w_ext_table(TABLE_ACL, i + 1,REG_PORT_ACL_BYTE_EN_LSB, 0xFF);
	}
	Ksz_sw_reset_acl_hw();
}

//============= micrel RSTP ============
void Ksz_port_cfg_mstp(int p, u8 mstp)
{
//	Ksz_port_w8(p, REG_PORT_LUE_MSTP_INDEX, mstp);
}
// This routine configures the spanning tree state of the port.



Ksz_sw_fast_aging(unsigned char set){
	Ksz_sw_wCfg(REG_SW_CTRL_1, SW_FAST_AGING,set);
}

Ksz_port_w8Cfg_dis_learn(unsigned char port, unsigned char set){
	Ksz_port_w8Cfg(port, P_STP_CTRL, PORT_LEARN_DISABLE,set); //Port n Control 2 reg. (0x12,22,32,52)
}

Ksz_port_w8Cfg_learn(unsigned char port, unsigned char endis){
}

Ksz_port_w8Cfg_forwarding(unsigned char port, unsigned char endis){
}
Ksz_port_w8Cfg_rx(unsigned char port, unsigned char set){
	Ksz_port_w8Cfg(port, P_STP_CTRL, PORT_RX_ENABLE,set);
}
Ksz_port_w8Cfg_tx(unsigned char port, unsigned char set){
	Ksz_port_w8Cfg(port, P_STP_CTRL, PORT_TX_ENABLE,set);
}

inline int Ksz_port_chk_dis_learn(int p)
{
	return Ksz_port_chk(p,P_STP_CTRL, PORT_LEARN_DISABLE);
}

inline int Ksz_port_chk_rx( int p)
{
	return Ksz_port_chk( p,P_STP_CTRL, PORT_RX_ENABLE);
}

inline int Ksz_port_chk_tx( int p)
{
	return Ksz_port_chk( p,P_STP_CTRL, PORT_TX_ENABLE);
}

//register 2(0x02) Global control 0. bit5.
Ksz_sw_flush_dyn_mac_table(unsigned char port)
{
	int cnt;
	int first;
	int index;
	int learn_disable[TOTAL_PORT_NUM];

	if ((uint) port < TOTAL_PORT_NUM) {
		first = port;
		cnt = port + 1;
	} else {
		first = 0;
		cnt = TOTAL_PORT_NUM;
	}
	for (index = first; index < cnt; index++) {
		learn_disable[index] = Ksz_port_chk_dis_learn( index);
		if (!learn_disable[index])
			Ksz_port_cfg_dis_learn( index, 1);
	}
	//SW_FLUSH_DYN_MAC_TABLE == differs from othres. It is 5th bit of 0x02 register
	Ksz_sw_wCfg(S_FLUSH_TABLE_CTRL, 1<<5, 1);//SW_FLUSH_DYN_MAC_TABLE, 1);
	for (index = first; index < cnt; index++) {
		if (!learn_disable[index])
			Ksz_port_cfg_dis_learn( index, 0);
	}
	Ksz_sw_wCfg(S_FLUSH_TABLE_CTRL, 1<<5, 1);//Ksz_sw_wCfg(S_FLUSH_TABLE_CTRL,SW_FLUSH_DYN_MAC_TABLE, 1);
}

void Ksz_port_set_stp_state( int port, int state)
{
	SW_D data;
	struct ksz_port_cfg *port_cfg;
	int member = -1;

	port_cfg = &g_sw.info.port_cfg[port];
	Ksz_port_r8(port, P_STP_CTRL, &data);//get current//Reg2
	switch (state) {
	case STP_STATE_DISABLED:
		data &= ~(PORT_TX_ENABLE | PORT_RX_ENABLE);
		data |= PORT_LEARN_DISABLE;
		if (port < SW_PORT_NUM)
			member = 0;
		break;
	case STP_STATE_LISTENING:
/*
 * No need to turn on transmit because of port direct mode.
 * Turning on receive is required if MAC table is not setup.
 */
		data &= ~PORT_TX_ENABLE;
		data |= PORT_RX_ENABLE;
		data |= PORT_LEARN_DISABLE;
		if (port < SW_PORT_NUM &&
		    STP_STATE_DISABLED == port_cfg->stp_state)
			member = g_sw.HOST_MASK | (1 << port);
		break;
	case STP_STATE_LEARNING:
		data &= ~PORT_TX_ENABLE;
		data |= PORT_RX_ENABLE;
		data &= ~PORT_LEARN_DISABLE;
		break;
	case STP_STATE_FORWARDING:
		data |= (PORT_TX_ENABLE | PORT_RX_ENABLE);
		data &= ~PORT_LEARN_DISABLE;
		break;
	case STP_STATE_BLOCKED:
		// Need to setup MAC table with override to keep receiving BPDU messages.  See Ksz_sw_setup_stp routine.
		data &= ~(PORT_TX_ENABLE | PORT_RX_ENABLE); //disable
		data |= PORT_LEARN_DISABLE;
		if (port < SW_PORT_NUM && STP_STATE_DISABLED == port_cfg->stp_state)
			member = g_sw.HOST_MASK | (1 << port);
		break;
	case STP_STATE_SIMPLE:
		data |= (PORT_TX_ENABLE | PORT_RX_ENABLE);
		data |= PORT_LEARN_DISABLE;
		if (port < SW_PORT_NUM)
			/* Set port-base vlan membership with host port. */
			member = g_sw.HOST_MASK | (1 << port);
		break;
	}
	Ksz_port_w8( port, P_STP_CTRL, data);
	port_cfg->stp_state = state;
	if (data & PORT_RX_ENABLE)
		g_sw.rx_ports |= (1 << port);
	else
		g_sw.rx_ports &= ~(1 << port);
	if (data & PORT_TX_ENABLE)
		g_sw.tx_ports |= (1 << port);
	else
		g_sw.tx_ports &= ~(1 << port);

	/* Port membership may share register with STP state. */
	if (member >= 0) Ksz_sw_cfg_port_base_vlan( port, (u8) member);
}

#define STP_ENTRY			    0
#define BROADCAST_ENTRY			1
#define BRIDGE_ADDR_ENTRY		2
#define IPV6_ADDR_ENTRY			3

/**
 * sw_clr_sta_mac_table - clear MAC table
 * @sw:		The switch instance.
 *
 * This routine clears the MAC table.
 */
void Ksz_sw_clr_sta_mac_table()//struct ksz_sw *sw)
{
	struct ksz_mac_table *entry;
	int i;

	for (i = 0; i < STATIC_MAC_TABLE_ENTRIES; i++) {
		entry = &g_sw.info.mac_table[i];
		Ksz_sw_w_sta_mac_table( i,
			entry->mac_addr, entry->ports,
			0, 0,
			entry->use_fid, entry->fid);
	}
}

/**
 * Ksz_sw_setup_stp - setup switch spanning tree support
 * @sw:		The switch instance.
 *
 * This routine initializes the spanning tree support of the switch.
 */
void Ksz_sw_setup_stp()//struct ksz_sw *sw)
{
	struct ksz_mac_table *entry;
	struct ksz_alu_table *alu;
	//struct ksz_sw_info *pinfo = g_sw.info;

	//My mac
	entry = &g_sw.info.mac_table[STP_ENTRY];
	entry->mac_addr[0] = 0x01;
	entry->mac_addr[1] = 0x80;
	entry->mac_addr[2] = 0xC2;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x00;

	entry->ports = g_sw.HOST_MASK;
	entry->override = 1;
	entry->valid = 1;

	alu = &g_sw.info.alu_table[STP_ENTRY];
	alu->forward = FWD_STP_DEV | FWD_HOST | FWD_HOST_OVERRIDE;
	alu->owner = g_sw.PORT_MASK;
	alu->valid = 1;

	Ksz_sw_w_sta_mac_table( STP_ENTRY,
		entry->mac_addr, entry->ports,
		entry->override, entry->valid,
		entry->use_fid, entry->fid);

	//Broadcast Addr.
	entry = &g_sw.info.mac_table[BROADCAST_ENTRY];
	memset(entry->mac_addr, 0xFF, ETH_ALEN);
	entry->ports = g_sw.HOST_MASK;
	entry->override = 0;
	entry->valid = 1;
	alu = &g_sw.info.alu_table[BROADCAST_ENTRY];
	alu->forward = FWD_MAIN_DEV | FWD_STP_DEV | FWD_HOST;
	alu->owner = g_sw.PORT_MASK;
	alu->valid = 1;

	entry = &g_sw.info.mac_table[BRIDGE_ADDR_ENTRY];
	memcpy(entry->mac_addr, g_sw.info.mac_addr, ETH_ALEN);
	entry->ports = g_sw.HOST_MASK;
	entry->override = 0;
	entry->valid = 1;
	alu = &g_sw.info.alu_table[BRIDGE_ADDR_ENTRY];
	alu->forward = FWD_MAIN_DEV | FWD_HOST;
	alu->owner = g_sw.PORT_MASK;
	alu->valid = 1;

	//IPV6 Addr
	entry = &g_sw.info.mac_table[IPV6_ADDR_ENTRY];
	memcpy(entry->mac_addr, g_sw.info.mac_addr, ETH_ALEN);
	entry->mac_addr[0] = 0x33;
	entry->mac_addr[1] = 0x33;
	entry->mac_addr[2] = 0xFF;
	entry->ports = g_sw.HOST_MASK;
	entry->override = 0;
	entry->valid = 1;
	alu = &g_sw.info.alu_table[IPV6_ADDR_ENTRY];
	alu->forward = FWD_MAIN_DEV | FWD_STP_DEV | FWD_HOST;
	alu->owner = g_sw.PORT_MASK;
	alu->valid = 1;
}

#ifdef CONFIG_1588_PTP
void Ksz_sw_setup_ptp()
{
	struct ksz_mac_table *entry;
	struct ksz_alu_table *alu;
	int i;
	u8 forward;
	//struct ksz_sw_info *pinfo = g_sw.info;

	i = g_sw.info.multi_sys;
	forward = FWD_MAIN_DEV;
	forward |= FWD_VLAN_DEV;

	//Register 01005E-000181
	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x01;
	entry->mac_addr[1] = 0x00;
	entry->mac_addr[2] = 0x5E;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x01;
	entry->mac_addr[5] = 0x81;
	entry->ports = g_sw.PORT_MASK;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward;
	alu->owner = 1;
	alu->valid = 1;
	//Register 01005E-000006B
	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x01;
	entry->mac_addr[1] = 0x00;
	entry->mac_addr[2] = 0x5E;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x6B;
	entry->ports = g_sw.PORT_MASK;
	entry->override = 1;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward | FWD_HOST_OVERRIDE;
	alu->owner = 1;
	alu->valid = 1;
	//Register 333300-000181
	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x33;
	entry->mac_addr[1] = 0x33;
	entry->mac_addr[2] = 0x00;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x01;
	entry->mac_addr[5] = 0x81;
	entry->ports = g_sw.PORT_MASK;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward;
	alu->owner = 1;
	alu->valid = 1;
	//Register 333300-00006B
	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x33;
	entry->mac_addr[1] = 0x33;
	entry->mac_addr[2] = 0x00;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x6B;
	entry->ports = g_sw.PORT_MASK;
	entry->override = 1;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward | FWD_HOST_OVERRIDE;
	alu->owner = 1;
	alu->valid = 1;
	//Register 011b19-000000
	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x01;
	entry->mac_addr[1] = 0x1B;
	entry->mac_addr[2] = 0x19;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x00;
	entry->ports = g_sw.PORT_MASK;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward;
	alu->owner = 1;
	alu->valid = 1;
	//Register 0180C2-00000E
	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x01;
	entry->mac_addr[1] = 0x80;
	entry->mac_addr[2] = 0xC2;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x0E;
	entry->ports = g_sw.HOST_MASK;
	entry->override = 1;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward | FWD_HOST_OVERRIDE;
	alu->owner = 1;
	alu->valid = 1;

	g_sw.info.multi_sys = i;
}
#endif

void Ksz_sw_setup_multicastAddrOnMemory()
{
	struct ksz_mac_table *entry;
	struct ksz_alu_table *alu;
	int i;
	u8 forward;
	//struct ksz_sw_info *info = g_sw.info;

	i = MULTI_MAC_TABLE_ENTRIES;
	forward = FWD_STP_DEV;
	forward |= FWD_MAIN_DEV;

	// Used for V2 IGMP messages.
	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x01;
	entry->mac_addr[1] = 0x00;
	entry->mac_addr[2] = 0x5E;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x01;
	entry->ports = g_sw.HOST_MASK;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward;
	alu->owner = g_sw.PORT_MASK;
	alu->valid = 1;

	//IPv6
	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x33;
	entry->mac_addr[1] = 0x33;
	entry->mac_addr[2] = 0x00;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x01;
	entry->ports = g_sw.HOST_MASK;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward;
	alu->owner = g_sw.PORT_MASK;
	alu->valid = 1;

	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x01;
	entry->mac_addr[1] = 0x00;
	entry->mac_addr[2] = 0x5E;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x02;
	entry->ports = g_sw.HOST_MASK;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward;
	alu->owner = g_sw.PORT_MASK;
	alu->valid = 1;

	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x33;
	entry->mac_addr[1] = 0x33;
	entry->mac_addr[2] = 0x00;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x02;
	entry->ports = g_sw.HOST_MASK;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward;
	alu->owner = g_sw.PORT_MASK;
	alu->valid = 1;

	// Used for V3 IGMP messages.
	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x01;
	entry->mac_addr[1] = 0x00;
	entry->mac_addr[2] = 0x5E;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x16;
	entry->ports = g_sw.HOST_MASK;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward;
	alu->owner = g_sw.PORT_MASK;
	alu->valid = 1;

	//224.0.0.22 (IGMPv3)
	entry = &g_sw.info.mac_table[--i];
	entry->mac_addr[0] = 0x33;
	entry->mac_addr[1] = 0x33;
	entry->mac_addr[2] = 0x00;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x16;
	entry->ports = g_sw.HOST_MASK;
	alu = &g_sw.info.alu_table[i];
	alu->forward = forward;
	alu->owner = g_sw.PORT_MASK;
	alu->valid = 1;

	g_sw.info.multi_sys = i;
}

void Ksz_bridge_change()
{
	int port;
	u8 member;
	//struct ksz_sw_info *info = g_sw.info;

	for (port = 0; port < SW_PORT_NUM; port++) {
		if (STP_STATE_FORWARDING == g_sw.info.port_cfg[port].stp_state)
			member = g_sw.HOST_MASK | g_sw.info.member;
		else if (STP_STATE_DISABLED == g_sw.info.port_cfg[port].stp_state)
			member = 0;
		else
			member = g_sw.HOST_MASK | (1 << port);
		if (member != g_sw.info.port_cfg[port].member)
			sw_cfg_port_base_vlan( port, member);
	}
}  /* bridge_change */

/**
 * sw_pass_addr - allow certain packets to the host port
 * @sw:		The switch instance.
 *
 * This routine allows certain packets to reach the host port.
 */
void Ksz_sw_pass_addr(struct ksz_sw *sw)
{
	struct ksz_mac_table *entry;
	int i;
	//struct ksz_sw_info *info = g_sw.info;

	for (i = BROADCAST_ENTRY; i <= IPV6_ADDR_ENTRY; i++) {
		entry = &g_sw.info.mac_table[i];
		switch (i) {
		case BROADCAST_ENTRY:
			memset(entry->mac_addr, 0xFF, ETH_ALEN);
			break;
		case BRIDGE_ADDR_ENTRY:
			memcpy(entry->mac_addr, g_sw.info.br_addr, ETH_ALEN);
			break;
		case IPV6_ADDR_ENTRY:
			memcpy(entry->mac_addr, g_sw.info.br_addr, ETH_ALEN);
			entry->mac_addr[0] = 0x33;
			entry->mac_addr[1] = 0x33;
			entry->mac_addr[2] = 0xFF;
			break;
		}
		entry->ports = g_sw.HOST_MASK;
		entry->override = 0;
		entry->valid = 1;
		sw_w_sta_mac_table( i,
			entry->mac_addr, entry->ports,
			entry->override, entry->valid,
			entry->use_fid, entry->fid);
	}
}  /* sw_pass_addr */

void Ksz_sw_pass_multi(struct ksz_sw *sw)
{
	struct ksz_mac_table *entry;
	int i;

	for (i = STATIC_MAC_TABLE_ENTRIES; i < MULTI_MAC_TABLE_ENTRIES; i++) {
		entry = &g_sw.info.mac_table[i];
		if (entry->ports)
			entry->valid = 1;
	}
}  /* sw_pass_multi */

void Ksz_monitor_ports()//struct ksz_sw *sw)
{
	/*
	int port;
	struct net_device *bridge_dev = NULL;
	struct ksz_sw_info *info = g_sw.info;
	u8 member = info.member;
	u8 stp = info.rstp; //????
	u8 prev_stp = info.rstp; //????
	u8 stp_down = 0;
	u8 state;
	u8 forwarding[SW_PORT_NUM];

	memset(forwarding, 0, SW_PORT_NUM);
	////g_sw.ops->acquire(sw);
	for (port = 0; port < SW_PORT_NUM; port++) {
		struct net_device *dev = g_sw.netdev[port + g_sw.dev_offset];

		//state = g_sw.net_ops->get_port_state(dev, &bridge_dev);
		if (state != STP_STATE_SIMPLE) {
			stp |= (1 << port);
			if (STP_STATE_DISABLED == state)
				stp_down |= (1 << port);
		} else {
			stp &= ~(1 << port);
			//state = g_sw.net_ops->get_state(dev);
		}
		if (stp != info.rstp) {
			info.rstp = stp;

			// Device just removed from bridge.
			if (!(stp & (1 << port))) {
				if (netif_running(dev))
					state = STP_STATE_SIMPLE;
			}
		}
		//g_sw.net_ops->set_state(dev, state);

		if (info.port_cfg[port].stp_state != state) {
			if (STP_STATE_FORWARDING ==
					info.port_cfg[port].stp_state)
				member &= ~(1 << port);
			if (STP_STATE_FORWARDING == state)
				member |= (1 << port);

			//Try to set forwarding after the other states.
			if (STP_STATE_FORWARDING == state)
				forwarding[port] = true;
			else
				port_set_stp_state( port, state);
			if (STP_STATE_LEARNING == state ||
			    STP_STATE_BLOCKED == state)
				sw_flush_dyn_mac_table( port);
		}
	}
	for (port = 0; port < SW_PORT_NUM; port++) {
		if (forwarding[port])
			port_set_stp_state( port, STP_STATE_FORWARDING);
	}
	//g_sw.ops->release(sw);
	if (prev_stp != info.rstp && !info.rstp)
		memset(info.br_addr, 0, ETH_ALEN);
	if (stp_down != info.stp_down || prev_stp != info.rstp) {
		struct ksz_mac_table *entry = &g_sw.info.mac_table[0];

		if (stp_down == info.rstp) {

			// Turn off STP only when it is already setup.
			if (prev_stp == info.rstp) {
				sw_w_sta_mac_table( 0,
					entry->mac_addr, entry->ports,
					0, 0,
					entry->use_fid, entry->fid);

				// No ports in forwarding state.
				////g_sw.ops->acquire(sw);
				port_set_stp_state( SW_PORT_NUM,
					STP_STATE_SIMPLE);
				///g_sw.ops->release(sw);
				sw_block_addr(sw);
				sw_block_multi(sw);
			}
		} else if (info.stp_down == info.rstp ||
				(!prev_stp && info.rstp))
			sw_w_sta_mac_table( 0,
				entry->mac_addr, entry->ports,
				entry->override, entry->valid,
				entry->use_fid, entry->fid);

		// Update disabled ports when STP is settled down.
		if (prev_stp == info.rstp)
			info.stp_down = stp_down;
	}

	if (member != info.member) {
		int cnt = 0;

		for (port = 0; port < SW_PORT_NUM; port++)
			if (member & (1 << port))
				cnt++;
		info.fwd_ports = cnt;

		//Have first member.
		if (!info.member) {

			// Force to program bridge address.
			info.br_addr[0] = 0xFF;
		}
		info.member = member;
		////g_sw.ops->acquire(sw);
		bridge_change(sw);
		////g_sw.ops->release(sw);
	}

	// At least one port in forwarding state.
	if (info.member && bridge_dev && memcmp(bridge_dev->dev_addr,info.br_addr, ETH_ALEN)) {
		memcpy(info.br_addr, bridge_dev->dev_addr, ETH_ALEN);
		sw_pass_addr(sw);
		sw_pass_multi(sw);
	} */
}

void Ksz8794_Loop(){
	int i;
	char str[10];
	long dval = 0;
	u8 HOST_MAC_ADDR[6] = {0x00,0x01,0x2,0x3,0x4,0x5};

	Ksz_ConfigSPI_CheckWeHave(USE_SPI2,0x84,0x63); //KSZ8463;
	Ksz_setup_dsa_switch_init();
	Ksz_sw_init();
	Ksz_sw_setup_dev(&g_dev,
			"KSZ", //IFNAME
//			&g_main_port,
			4, //main port id
			3, //port_cnt
			3);//int mib_port_cnt);

	Ksz_sw_open_dev(HOST_MAC_ADDR);//Ksz_sw_init_mib();//Ksz_sw_start(HOST_MAC_ADDR); //setup, enable, set_addr,
	//Ksz_sw_open_port(); //Ksz_open_port(&bp->port, &bp->state);
	//Ksz_phy_start(bp->phy_dev);

	sysfs_sw_read(PROC_SW_INFO, NULL);
	sysfs_sw_read(PROC_SET_SW_DUPLEX,&g_sw.port_info[1]);

	i=0;

	int portIndex;
	unsigned char reg;

while(1){
	for (portIndex = 0; portIndex < 3; portIndex++){
		//check link status
		reg = Ksz_getReg(0x1e + (portIndex<<4), 1); //0x1e(PORT1), 0x2e(PORT2) Port n Status 2 Registers
		if (reg & (1 << 5)) //bit 5 Link Good.
		{
			// link is now good
			printf("Port %d> Link Good\r\n", portIndex+1);
		}
		else if (!(reg & (1 << 5)))
		{
			// link is now down
			printf("Port %d> Link Down\r\n", portIndex+1);
		}
	}
	delayms(1000);
}

		while(1){
			Ksz_getReg(i, 1);//KSZ8794_REGS[i]);
			delayms(1000);
			i++;
			if(i==0xff)	i=0;
		};

}



#ifdef CONFIG_KSZ_STP
/**
 * sw_block_addr - block certain packets from the host port
 * This routine blocks certain packets from reaching to the host port.
 */
void Ksz_sw_block_addr()
{
	struct ksz_mac_table *entry;
	int i;

	for (i = BROADCAST_ENTRY; i <= IPV6_ADDR_ENTRY; i++) {
		entry = &g_sw.info.mac_table[i];
		entry->valid = 0;
		Ksz_sw_w_sta_mac_table( i,
			entry->mac_addr, entry->ports,
			0, entry->valid,
			entry->use_fid, entry->fid);
	}
}

void Ksz_sw_block_multi()
{
	struct ksz_mac_table *entry;
	int i;

	for (i = STATIC_MAC_TABLE_ENTRIES; i < MULTI_MAC_TABLE_ENTRIES; i++) {
		entry = &g_sw.info.mac_table[i];
		if (entry->ports)
			entry->valid = 0;
	}
}
//==== General ====
#ifdef KSZ_DLR
#include "ksz_dlr.c"
#endif

/*
 * Link detection routines
 */

inline void Ksz_dPrintf_AllLinkStatus(int change) //Ksz_dbp_link
{
	struct ksz_port_info *pinfo;
	int i;
	int pid;

	for (i = 0, pid = g_sw.first_port; i < g_sw.port_cnt; i++, pid++) {
		pinfo = &g_sw.port_info[pid];
		if (MEDIA_CONNECTED == pinfo->media_state) {
			if (change & (1 << i)) {
				printf( "port %d: %d, %d\n", i + g_sw.first_port,
						pinfo->tx_rate / TX_RATE_UNIT,	pinfo->duplex);
			}
		} else {
			if (change & (1 << i))
				printf( "link %d-%d disconnected\n", g_sw.id, i + g_sw.first_port);
		}
	}
}

/**
 * sw_get_addr - get the switch MAC address.
 * @sw:		The switch instance.
 * @mac_addr:	Buffer to store the MAC address.
 *
 * This function retrieves the MAC address of the switch.
 */
inline void Ksz_sw_get_addr( u8 *mac_addr)
{
	int i;
#if (SW_SIZE == (2))
	u16 *addr = (u16 *) mac_addr;

	for (i = 0; i < 6; i += 2) {
		*addr = g_sw.reg->r16( REG_SW_MAC_ADDR_0 + i);
		*addr = ntohs(*addr);
		addr++;
	}
#else

	for (i = 0; i < 6; i++)
		mac_addr[i] = SW_R8( REG_SW_MAC_ADDR_0 + i);
#endif
	memcpy(g_sw.info.mac_addr, mac_addr, 6);
}

/**
 * sw_set_addr - configure switch MAC address
 * @sw:		The switch instance.
 * @mac_addr:	The MAC address.
 *
 * This function configures the MAC address of the switch.
 */
void Ksz_sw_set_addr( u8 *mac_addr)
{
	int i;
#if (SW_SIZE == (2))
	u16 *addr = (u16 *) mac_addr;

	for (i = 0; i < 6; i += 2) {
		g_sw.reg->w16( REG_SW_MAC_ADDR_0 + i, htons(*addr));
		addr++;
	}
#else

	for (i = 0; i < 6; i++)
		SW_W8( REG_SW_MAC_ADDR_0 + i, mac_addr[i]);
#endif
	memcpy(g_sw.info.mac_addr, mac_addr, 6);
}

#ifdef SW_PORT_PHY_ADDR_MASK
/**
 * Ksz_sw_init_phy_addr - initialize switch PHY address
 * @sw:		The switch instance.
 *
 * This function initializes the PHY address of the switch.
 */
void Ksz_sw_init_phy_addr(struct ksz_sw *sw)
{
	u8 addr;

	addr = SW_R8( REG_SW_CTRL_13);
	addr >>= SW_PORT_PHY_ADDR_SHIFT;
	addr &= SW_PORT_PHY_ADDR_MASK;
	g_sw.info.phy_addr = addr;
}

/**
 * sw_set_phy_addr - configure switch PHY address
 * @sw:		The switch instance.
 * @addr:	The PHY address.
 *
 * This function configures the PHY address of the switch.
 */
void sw_set_phy_addr( u8 addr)
{
	g_sw.info.phy_addr = addr;
	addr &= SW_PORT_PHY_ADDR_MASK;
	addr <<= SW_PORT_PHY_ADDR_SHIFT;
	SW_W8( REG_SW_CTRL_13, addr);
}
#endif

/**
 * sw_set_global_ctrl - set switch global control
 * @sw:		The switch instance.
 *
 * This routine sets the global control of the switch function.
 */
void Ksz_sw_set_global_ctrl_8794()//struct ksz_sw *sw)
{
	SW_D data;

	// was Enable switch MII flow control.
	//We disable the MII flow control.(Reg@06)
	data = SW_R8(REG_SW_CTRL_4);//S_REPLACE_VID_CTRL);
	data &= ~SW_FLOW_CTRL;//false //data |= SW_FLOW_CTRL;--set
	SW_W8(REG_SW_CTRL_4,data);// (S_REPLACE_VID_CTRL, data);

	//Reg@02
	//Enable the normal aging.
	data = SW_R8(REG_SW_CTRL_0);// S_LINK_AGING_CTRL);
	data |= SW_LINK_AUTO_AGING;
	SW_W8(REG_SW_CTRL_0,data);// S_LINK_AGING_CTRL, data);

	//Reg@3
	data = SW_R8( REG_SW_CTRL_1);
		// Enable aggressive back off algorithm in half duplex mode.
		data |= SW_AGGR_BACKOFF;
		// Enable automatic fast aging when link changed detected.
		data |= SW_AGING_ENABLE;

		//if (g_sw.overrides & FAST_AGING)
		data |= SW_FAST_AGING; //800usec
		//else
		//data &= ~SW_FAST_AGING;
	SW_W8(REG_SW_CTRL_1, data);

	//Reg@4
	data = SW_R8( REG_SW_CTRL_2);
		if (g_sw.dev_count > 1)
			data |= UNICAST_VLAN_BOUNDARY; 	// Make sure unicast VLAN boundary is set as default. --- ????
		data |= NO_EXC_COLLISION_DROP; 		// Enable no excessive collision drop.
	SW_W8( REG_SW_CTRL_2, data);

#ifdef REG_ANA_CTRL_3
	/* Enable LinkMD function. */
	sw_w16( REG_ANA_CTRL_3, 0x8008);
#endif
}
/*
//(1) R/W Tables
/**
 * sw_r_table - read 4 bytes of data from switch table
 * @sw:		The switch instance.
 * @table:	The table selector.
 * @addr:	The address of the table entry.
 * @data:	Buffer to store the read data.
 *
 * This routine reads 4 bytes of data from the table of the switch.
 * Hardware is locked to minimize corruption of read data.
 */
void Ksz_sw_r_table( int table, u16 addr, u32 *data)
{
	u16 ctrl_addr;

	ctrl_addr = IND_ACC_TABLE(table | TABLE_READ) | addr;

	//mutex_lock(g_sw.reglock);
	w16( REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY( REG_IND_CTRL_0);
	//*data = g_sw.reg->r32( REG_IND_DATA_LO);
	//mutex_unlock(g_sw.reglock);
}

/**
 * sw_w_table - write 4 bytes of data to the switch table
 * @sw:		The switch instance.
 * @table:	The table selector.
 * @addr:	The address of the table entry.
 * @data:	Data to be written.
 *
 * This routine writes 4 bytes of data to the table of the switch.
 * Hardware is locked to minimize corruption of written data.
 */
void Ksz_sw_w_table( int table, u16 addr, u32 data)
{
	u16 ctrl_addr;

	ctrl_addr = IND_ACC_TABLE(table) | addr;

	//mutex_lock(g_sw.reglock);
	w32( REG_IND_DATA_LO, data);
	w16( REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY( REG_IND_CTRL_0);
	//mutex_unlock(g_sw.reglock);
}

/**
 * sw_r_table_64 - read 8 bytes of data from the switch table
 * @sw:		The switch instance.
 * @table:	The table selector.
 * @addr:	The address of the table entry.
 * @data_hi:	Buffer to store the high part of read data (bit63 ~ bit32).
 * @data_lo:	Buffer to store the low part of read data (bit31 ~ bit0).
 *
 * This routine reads 8 bytes of data from the table of the switch.
 * Hardware is locked to minimize corruption of read data.
 */
void Ksz_sw_r_table_64( int table, u16 addr, u32 *data_hi,
	u32 *data_lo)
{
	u16 ctrl_addr;

	ctrl_addr = IND_ACC_TABLE(table | TABLE_READ) | addr;

	//mutex_lock(g_sw.reglock);
	w16( REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY( REG_IND_CTRL_0);
	*data_hi = r32( REG_IND_DATA_HI);
	*data_lo = r32( REG_IND_DATA_LO);
	//mutex_unlock(g_sw.reglock);
}

/**
 * sw_w_table_64 - write 8 bytes of data to the switch table
 * @sw:		The switch instance.
 * @table:	The table selector.
 * @addr:	The address of the table entry.
 * @data_hi:	The high part of data to be written (bit63 ~ bit32).
 * @data_lo:	The low part of data to be written (bit31 ~ bit0).
 *
 * This routine writes 8 bytes of data to the table of the switch.
 * Hardware is locked to minimize corruption of written data.
 */
void Ksz_sw_w_table_64( int table, u16 addr, u32 data_hi, u32 data_lo)
{
	u16 ctrl_addr;

	ctrl_addr = IND_ACC_TABLE(table) | addr;

	Ksz_setReg32(REG_IND_DATA_HI, data_hi); //w32( REG_IND_DATA_HI, data_hi);
	Ksz_setReg32(REG_IND_DATA_LO, data_lo);//w32( REG_IND_DATA_LO, data_lo);
	Ksz_setReg16(REG_IND_CTRL_0,  ctrl_addr); //w16( REG_IND_CTRL_0, ctrl_addr);

	HW_DELAY( REG_IND_CTRL_0);
}

inline int Ksz_valid_dyn_entry( u8 *data)
{
	int timeout = 100;

	do {
		*data = SW_R8( REG_IND_DATA_CHECK);
		timeout--;
	} while ((*data & DYNAMIC_MAC_TABLE_NOT_READY) && timeout);

	/* Entry is not ready for accessing. */
	if (*data & DYNAMIC_MAC_TABLE_NOT_READY)
		return 1;

	/* Entry is ready for accessing. */
	else {
		/* There is no valid entry in the table. */
		if (*data & DYNAMIC_MAC_TABLE_MAC_EMPTY)
			return 2;
	}
	return 0;
}

/**
 * sw_r_dyn_mac_table - read from dynamic MAC table
 * @sw:		The switch instance.
 * @addr:	The address of the table entry.
 * @mac_addr:	Buffer to store the MAC address.
 * @fid:	Buffer to store the FID.
 * @src_port:	Buffer to store the source port number.
 * @timestamp:	Buffer to store the timestamp.
 * @entries:	Buffer to store the number of entries.  If this is zero, the
 *		table is empty and so this function should not be called again
 *		until later.
 *
 * This function reads an entry of the dynamic MAC table of the switch.
 * Hardware is locked to minimize corruption of read data.
 *
 * Return 0 if the entry is successfully read; otherwise -1.
 */
int Ksz_sw_r_dyn_mac_table(u16 addr, u8 *mac_addr,
	u8 *fid, u8 *src_port, u8 *timestamp, u16 *entries)
{
	u32 data_hi;
	u32 data_lo;
	u16 ctrl_addr;
	int rc;
	u8 data;

	ctrl_addr = IND_ACC_TABLE(TABLE_DYNAMIC_MAC | TABLE_READ) | addr;

	//mutex_lock(g_sw.reglock);
	w16( REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY( REG_IND_CTRL_0);

	rc = Ksz_valid_dyn_entry( &data);
	if (1 == rc) {
		if (0 == addr)
			*entries = 0;
	} else if (2 == rc)
		*entries = 0;
	/* At least one valid entry in the table. */
	else {
		data_hi = r32( REG_IND_DATA_HI);

		/* Check out how many valid entry in the table. */
		*entries = (u16)(((((u16)
			data & DYNAMIC_MAC_TABLE_ENTRIES_H) <<	DYNAMIC_MAC_ENTRIES_H_SHIFT) |
			(((data_hi & DYNAMIC_MAC_TABLE_ENTRIES) >>	DYNAMIC_MAC_ENTRIES_SHIFT))) + 1);

		*fid = (u8)((data_hi & DYNAMIC_MAC_TABLE_FID) >>	DYNAMIC_MAC_FID_SHIFT);
		*src_port = (u8)((data_hi & DYNAMIC_MAC_TABLE_SRC_PORT) >>	DYNAMIC_MAC_SRC_PORT_SHIFT);
		*timestamp = (u8)((
			data_hi & DYNAMIC_MAC_TABLE_TIMESTAMP) >>	DYNAMIC_MAC_TIMESTAMP_SHIFT);

		data_lo = r32( REG_IND_DATA_LO);

		mac_addr[5] = (u8) data_lo;
		mac_addr[4] = (u8)(data_lo >> 8);
		mac_addr[3] = (u8)(data_lo >> 16);
		mac_addr[2] = (u8)(data_lo >> 24);

		mac_addr[1] = (u8) data_hi;
		mac_addr[0] = (u8)(data_hi >> 8);
		rc = 0;
	}
	//mutex_unlock(g_sw.reglock);

	return rc;
}

void Ksz_sw_d_dyn_mac_table()
{
	u16 entries = 0;
	u16 i;
	u8 mac_addr[ETH_ALEN];
	u8 ports = 0;
	u8 timestamp = 0;
	u8 fid = 0;
	//int locked = mutex_is_locked(&g_sw.lock);

	//if (locked)
	//	mutex_unlock(g_sw.reglock);
	memset(mac_addr, 0, ETH_ALEN);
	i = 0;
	do {
		if (!sw_r_dyn_mac_table( i, mac_addr, &fid, &ports,
				&timestamp, &entries)) {
			printf(
				"%02X:%02X:%02X:%02X:%02X:%02X %x %x %x %03x\n",
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5],
				fid, ports, timestamp, entries);
		}
		i++;
	} while (i < entries);
	//if (locked)
		//mutex_lock(g_sw.reglock);
}

/**
 * sw_r_sta_mac_table - read from MAC table
 * @sw:		The switch instance.
 * @addr:	The address of the table entry.
 * @mac_addr:	Buffer to store the MAC address.
 * @ports:	Buffer to store the port members.
 * @override:	Buffer to store the override flag.
 * @use_fid:	Buffer to store the use FID flag which indicates the FID is
 *		valid.
 * @fid:	Buffer to store the FID.
 *
 * This function reads an entry of the MAC table of the switch.  It
 * calls sw_r_table_64() to get the data.
 *
 * Return 0 if the entry is valid; otherwise -1.
 */
int Ksz_sw_r_sta_mac_table( u16 addr, u8 *mac_addr,
	u8 *ports, int *override, int *use_fid, u8 *fid)
{
	u32 data_hi;
	u32 data_lo;
	//int locked = mutex_is_locked(&g_sw.lock);

	//if (locked)		mutex_unlock(g_sw.reglock);
	Ksz_sw_r_table_64( TABLE_STATIC_MAC, addr, &data_hi, &data_lo);
	//if (locked)		mutex_lock(g_sw.reglock);
	if (data_hi & (STATIC_MAC_TABLE_VALID | STATIC_MAC_TABLE_OVERRIDE)) {
		mac_addr[5] = (u8) data_lo;
		mac_addr[4] = (u8)(data_lo >> 8);
		mac_addr[3] = (u8)(data_lo >> 16);
		mac_addr[2] = (u8)(data_lo >> 24);
		mac_addr[1] = (u8) data_hi;
		mac_addr[0] = (u8)(data_hi >> 8);
		*ports = (u8)((data_hi & STATIC_MAC_TABLE_FWD_PORTS) >>		STATIC_MAC_FWD_PORTS_SHIFT);
		*override = (data_hi & STATIC_MAC_TABLE_OVERRIDE) ? 1 : 0;
		*use_fid = (data_hi & STATIC_MAC_TABLE_USE_FID) ? 1 : 0;
		*fid = (u8)((data_hi & STATIC_MAC_TABLE_FID) >>			STATIC_MAC_FID_SHIFT);
		return 0;
	}
	return -1;
}

/**
 * sw_w_sta_mac_table - write to the MAC table
 * @sw:		The switch instance.
 * @addr:	The address of the table entry.
 * @mac_addr:	The MAC address.
 * @ports:	The port members.
 * @override:	The flag to override the port receive/transmit settings.
 * @valid:	The flag to indicate entry is valid.
 * @use_fid:	The flag to indicate the FID is valid.
 * @fid:	The FID value.
 *
 * This routine writes an entry of the MAC table of the switch.  It
 * calls sw_w_table_64() to write the data.
 */
void Ksz_sw_w_sta_mac_table( u16 addr, u8 *mac_addr,
	u8 ports, int override, int valid, int use_fid, u8 fid)
{
	u32 data_hi;
	u32 data_lo;

	data_lo = ((u32) mac_addr[2] << 24) |
		((u32) mac_addr[3] << 16) |
		((u32) mac_addr[4] << 8) | mac_addr[5];
	data_hi = ((u32) mac_addr[0] << 8) | mac_addr[1];
	data_hi |= (u32) ports << STATIC_MAC_FWD_PORTS_SHIFT;

	if (override)
		data_hi |= STATIC_MAC_TABLE_OVERRIDE;
	if (use_fid) {
		data_hi |= STATIC_MAC_TABLE_USE_FID;
		data_hi |= (u32) fid << STATIC_MAC_FID_SHIFT;
	}
	if (valid)
		data_hi |= STATIC_MAC_TABLE_VALID;
	else
		data_hi &= ~STATIC_MAC_TABLE_OVERRIDE;

	Ksz_sw_w_table_64( TABLE_STATIC_MAC, addr, data_hi, data_lo);
}

void Ksz_sw_d_sta_mac_table(struct ksz_sw *sw)
{
	u16 i;
	u8 mac_addr[ETH_ALEN];
	u8 ports;
	int override;
	int use_fid;
	u8 fid;

	i = 0;
	do {
		if (!Ksz_sw_r_sta_mac_table( i, mac_addr, &ports, &override,
				&use_fid, &fid)) {
			printf(
				"%d: %02X:%02X:%02X:%02X:%02X:%02X "
				"%x %u %u:%x\n",
				i, mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5],
				ports, override, use_fid, fid);
		}
		i++;
	} while (i < STATIC_MAC_TABLE_ENTRIES);
}

void Ksz_sw_d_mac_table(struct ksz_sw *sw)
{
	struct ksz_mac_table *entry;
	struct ksz_alu_table *alu;
	int i;
	int first_static = true;

	i = STATIC_MAC_TABLE_ENTRIES;
	do {
		entry = &g_sw.info.mac_table[i];
		if (entry->valid) {
			if (first_static) {
				first_static= false;
				printf( "\n");
			}
			alu = &g_sw.info.alu_table[i];
			printf(
				"%2x: %02X:%02X:%02X:%02X:%02X:%02X "
				"%x %u %u:%x  %02x:%x\n",
				i, entry->mac_addr[0], entry->mac_addr[1],
				entry->mac_addr[2], entry->mac_addr[3],
				entry->mac_addr[4], entry->mac_addr[5],
				entry->ports, entry->override, entry->use_fid,
				entry->fid,
				alu->forward, alu->owner);
		}
		i++;
		if (SW_MAC_TABLE_ENTRIES == i)
			first_static = true;
	} while (i < MULTI_MAC_TABLE_ENTRIES);
}
//=== MAC filter
/* -------------------------------------------------------------------------- */

inline void Ksz_port_cfg_dis_learn(int p, int set)
{
	Ksz_port_w8Cfg( p,P_STP_CTRL, PORT_LEARN_DISABLE, set);
}

inline void Ksz_port_cfg_src_filter_0( int p, int set)
{
	//Ksz_port_w8Cfg( p,P_SA_MAC_CTRL, PORT_SA_MAC1, set); //REG_PORT_CTRL1
}

inline void Ksz_port_cfg_src_filter_1( int p, int set)
{
	//Ksz_port_w8Cfg(p,	P_SA_MAC_CTRL, PORT_SA_MAC2, set);
}

inline int Ksz_port_chk_src_filter_0( int p)
{
	//return Ksz_port_chk(p,		P_SA_MAC_CTRL, PORT_SA_MAC1);
}

inline int Ksz_port_chk_src_filter_1( int p)
{
	//return Ksz_port_chk( p,		P_SA_MAC_CTRL, PORT_SA_MAC2);
}

/**
 * port_get_addr - get the port MAC address.
 * @sw:		The switch instance.
 * @port:	The port index.
 * @mac_addr:	Buffer to store the MAC address.
 *
 * This function retrieves the MAC address of the port.
 */
inline void Ksz_port_get_addr( int port, u8 *mac_addr)
{
	/*
	int i;
	SW_D reg;
#if (SW_SIZE == (2))
	u16 *addr = (u16 *) mac_addr;
#endif

	if (0 == port)
		reg = REG_PORT_0_MAC_ADDR_0;
	else
		reg = REG_PORT_1_MAC_ADDR_0;
#if (SW_SIZE == (2))
	for (i = 0; i < 6; i += 2) {
		*addr = r16( reg - i);
		*addr = ntohs(*addr);
		addr++;
	}
#else
	for (i = 0; i < 6; i++)
		mac_addr[i] = SW_R8( reg + i);
#endif
	memcpy(g_sw.port_info[port].mac_addr, mac_addr, 6);
	*/
}

/**
 * port_set_addr - configure port MAC address
 * @sw:		The switch instance.
 * @port:	The port index.
 * @mac_addr:	The MAC address.
 *
 * This function configures the MAC address of the port.
 */
void Ksz_port_set_addr( int port, u8 *mac_addr)
{
	/*
	int i;
	SW_D reg;
#if (SW_SIZE == (2))
	u16 *addr = (u16 *) mac_addr;
#endif

	if (0 == port)
		reg = REG_PORT_0_MAC_ADDR_0;
	else
		reg = REG_PORT_1_MAC_ADDR_0;
#if (SW_SIZE == (2))
	for (i = 0; i < 6; i += 2) {
		Ksz_setReg16( reg - i, htons(*addr));
		addr++;
	}
#else
	for (i = 0; i < 6; i++)
		Ksz_setReg8(reg + i, mac_addr[i]);
#endif
	memcpy(g_sw.port_info[port].mac_addr, mac_addr, 6);
	*/
}

inline void Ksz_sw_setup_src_filter( u8 *mac_addr)
{
	int port;

	for (port = 0; port < SW_PORT_NUM; port++) {
		Ksz_port_set_addr( port, mac_addr);
		Ksz_port_cfg_src_filter_0( port, 1);
		Ksz_port_cfg_src_filter_1(port, 1);
	}
}

void Ksz_sw_cfg_src_filter( int set)
{
	int port;

	for (port = 0; port < SW_PORT_NUM; port++) {
		Ksz_port_cfg_src_filter_0( port, set);
		Ksz_port_cfg_src_filter_1( port, set);
	}
}  /* sw_cfg_src_filter */

/* -------------------------------------------------------------------------- */



// sw_enable - enable the switch
void Ksz_sw_enable()
{
	int port;
	int p = 0;

	if (g_sw.port_cnt < g_sw.mib_port_cnt)
		p = 1;
	for (port = p; port < SW_PORT_NUM; port++) {
		if (g_sw.dev_count > 1)
			Ksz_port_set_stp_state(port, STP_STATE_DISABLED);
		else
			Ksz_port_set_stp_state(port, STP_STATE_FORWARDING);
	}
	if (g_sw.dev_count > 1)
		Ksz_port_set_stp_state(SW_PORT_NUM, STP_STATE_SIMPLE);
	else
		Ksz_port_set_stp_state(SW_PORT_NUM, STP_STATE_FORWARDING);

	// There may be some entries in the dynamic MAC table before the the learning is turned off.
	// Once the entries are in the table the switch may keep updating them even learning is off.
	if (g_sw.dev_count > 1)
		Ksz_sw_flush_dyn_mac_table(TOTAL_PORT_NUM);
	Ksz_setReg8(REG_CHIP_ID1, SW_START);
}
// Ksz_sw_init - initialize the switch: This routine initializes the hardware switch engine for default operation.
void Ksz_sw_init()//struct ksz_sw *sw)
{
	memset(g_sw.tx_pad, 0, 60);
	g_sw.tx_start = 0;

//#ifdef SW_PORT_PHY_ADDR_MASK
//	Ksz_sw_init_phy_addr();
//#endif

	//Ksz_sw_init_broad_storm();
	//Ksz_sw_init_prio(sw);
	//Ksz_sw_init_prio_rate();
	//Ksz_sw_init_vlan(sw);
	//Ksz_sw_init_acl();

	//if (!Ksz_sw_chk( REG_SW_CTRL_1, SW_TX_FLOW_CTRL | SW_RX_FLOW_CTRL))
	//	g_sw.overrides |= PAUSE_FLOW_CTRL;
}

//This routine setup the hardware switch engine for default operation.
void Ksz_sw_setup()
{
	int port;
	//int port_cnt, mib_port_cnt, dev_cnt;

	g_sw.HOST_PORT = 4; //3??
	g_sw.HOST_MASK = 32; //??
	g_sw.PORT_MASK  = ((1 << MAX_NPORTS) - 1);
	g_sw.features |= STP_SUPPORT;
	g_sw.features |= FAST_AGING;
	g_sw.features |= DSA_SUPPORT;

	//The followings will be updated
	g_sw.dev_count = 1; //indicates how many network devices to be created
	g_sw.port_cnt  = 1; //port_cnt variable indicates how many ports in the switch
	g_sw.mib_port_cnt = 1; //how many ports with MIB counters information(Normally it is the same as port_cnt.)
	//---sw_setup_special

    Ksz_sw_setup_special(&g_sw.port_cnt, &g_sw.mib_port_cnt, &g_sw.dev_count);

	Ksz_sw_set_global_ctrl_8794(); //disable MII flow control, enable fast aging.

	//disable flow control per port.
	for (port = 0; port < SW_PORT_NUM; port++) {
		SW_D data;

		//port_cfg_back_pressure( port, 1);
		// Switch actually cannot do auto-negotiation with old 10Mbit hub.
		Ksz_port_r8( port, P_FORCE_CTRL, &data);
		Ksz_port_cfg_force_flow_ctrl( port, 0); //disable
		data &= ~PORT_FORCE_FULL_DUPLEX;
		Ksz_port_w8( port, P_FORCE_CTRL, data);
	}

	//Ksz_sw_setup_broad_storm(sw);
	//Ksz_sw_setup_prio(sw);
	//Ksz_sw_setup_mirror();

	g_sw.info.multi_net = SW_MAC_TABLE_ENTRIES;
	if (g_sw.features & STP_SUPPORT) {
		Ksz_sw_setup_stp();
#ifdef CONFIG_KSZ_STP
		Ksz_sw_setup_multicastAddrOnMemory();
#ifdef CONFIG_1588_PTP
		Ksz_sw_setup_ptp();
#endif
#endif
	}
#ifdef KSZ_DLR
	Ksz_sw_setup_dlr();
#endif

}

static ssize_t sysfs_sw_read(int proc_num, u8 pid)//struct ksz_port *port)//, ssize_t len, char *buf)
{
	int i;
	int j;
	u16 map;
	struct ksz_sw_info *psw_info = &g_sw.info;
	ssize_t len=0;
	char buf[80];
	struct ksz_port_info *pport_info;

	pport_info = &g_sw.port_info[pid];

	switch (proc_num) {
	case PROC_SW_INFO:
		len = display_sw_info(g_sw.mib_port_cnt, buf, len);
		break;
	case PROC_SET_BROADCAST_STORM:
		printf("BroadcastStorm=%u%%\r\n", g_sw.info.broad_per);
		break;
	case PROC_SET_SW_DUPLEX:
		if (!pid)	break;
		printf("Duplex=(%u)",pport_info->duplex);
		if (MEDIA_CONNECTED == g_sw.port_info[pid].media_state){//port->linked->state) {
			if (1 == pport_info->duplex) //if (1 == port->linked->duplex)
				printf("half-duplex\r\n");
			else if (2 == pport_info->duplex)//else if (2 == port->linked->duplex)
				printf("full-duplex\r\n");
		} else
			printf("unlinked\r\n");
		break;
	case PROC_SET_SW_SPEED:
		if (!pid)	break;
		printf("Speed=(%u)", pport_info->speed);
		if (MEDIA_CONNECTED == pport_info->media_state)
			printf("%u\r\n",pport_info->tx_rate / TX_RATE_UNIT);
		else
			printf("unlinked\r\n");
		break;
	case PROC_SET_SW_FORCE:
		if (!pid)	break;
		printf("ForceLink=(%u)\r\n", pport_info->force_link);
		break;
	case PROC_SET_SW_FLOW_CTRL:
		if (!pid)	break;
		printf("FlowCtrl=(%u);", pport_info->flow_ctrl);
		switch (pport_info->flow_ctrl) {
		case PHY_FLOW_CTRL:
			printf("flow control\r\n");
			break;
		case PHY_TX_ONLY:
			printf("tx only\r\n");
			break;
		case PHY_RX_ONLY:
			printf("rx only\r\n");
			break;
		default:
			printf("no flow control\r\n");
			break;
		}
		break;
	case PROC_SET_SW_MIB:
		if (!pid)	break;
		//len += display_sw_mib_counters(port->first_port, port->mib_port_cnt, buf + len);
		break;

	case PROC_SET_DIFFSERV:
		for (i = 0; i < DIFFSERV_ENTRIES / KS_PRIO_IN_REG; i++) {
			len += sprintf(buf + len, "%2u=",i * KS_PRIO_IN_REG);
			map = g_sw.info.diffserv[i];
			for (j = 0; j < KS_PRIO_IN_REG; j++) {
				len += sprintf(buf + len, "%u ",map & KS_PRIO_M);
				map >>= KS_PRIO_S;
			}
			len += sprintf(buf + len, "\t"SW_SIZE_STR"\n",	psw_info->diffserv[i]);
		}
		break;
	case PROC_SET_802_1P:
		for (i = 0; i < PRIO_802_1P_ENTRIES / KS_PRIO_IN_REG; i++) {
			len += sprintf(buf + len, "%u=",i * KS_PRIO_IN_REG);
			map = psw_info->p_802_1p[i];
			for (j = 0; j < KS_PRIO_IN_REG; j++) {
				len += sprintf(buf + len, "%u ", map & KS_PRIO_M);
				map >>= KS_PRIO_S;
			}
			len += sprintf(buf + len, "\t"SW_SIZE_STR"\n",	psw_info->p_802_1p[i]);
		}
		break;
#ifdef SW_PORT_PHY_ADDR_MASK
	case PROC_SET_PHY_ADDR:
		len += sprintf(buf + len, "%u\n",
				pinfo->phy_addr);
		break;
#endif
	case PROC_SET_SW_VID:
		len += sprintf(buf + len, "0x%04x\n", g_sw.vid);
		break;
	case PROC_GET_PORTS:
		len += sprintf(buf + len, "%u\n", g_sw.mib_port_cnt - 1);
		break;
	case PROC_GET_DEV_START:
	{
		int start = 0;

		if (g_sw.dev_offset)
			start = 100;
		len += sprintf(buf + len, "%u\n", start);
		break;
	}
	case PROC_GET_VLAN_START:
	{
		int start = 0;

		if (g_sw.features & VLAN_PORT)
			start = VLAN_PORT_START;
		len += sprintf(buf + len, "%u\n", start);
		break;
	}
	case PROC_GET_STP:
		len += sprintf(buf + len, "%u\n",
			!!(g_sw.features & STP_SUPPORT));
		break;
	case PROC_SET_SW_FEATURES:
		len += sprintf(buf + len, "%08x:\n", g_sw.features);
		len += sprintf(buf + len, "\t%08x = STP support\n",
			STP_SUPPORT);
		len += sprintf(buf + len, "\t%08x = VLAN port forwarding\n",
			VLAN_PORT);
		len += sprintf(buf + len, "\t%08x = VLAN port remove tag\n",
			VLAN_PORT_REMOVE_TAG);
		len += sprintf(buf + len, "\t%08x = VLAN port tag tailing\n",
			VLAN_PORT_TAGGING);
		len += sprintf(buf + len, "\t%08x = different MAC addresses\n",
			DIFF_MAC_ADDR);
#ifdef CONFIG_1588_PTP
		len += sprintf(buf + len, "\t%08x = 1588 PTP\n",
			PTP_HW);
#endif
		break;
	case PROC_SET_SW_OVERRIDES:
		len += sprintf(buf + len, "%08x:\n", g_sw.overrides);
		len += sprintf(buf + len, "\t%08x = flow control\n",
			PAUSE_FLOW_CTRL);
		len += sprintf(buf + len, "\t%08x = fast aging\n",
			FAST_AGING);
		len += sprintf(buf + len, "\t%08x = tag is removed\n",
			TAG_REMOVE);
		len += sprintf(buf + len, "\t%08x = tail tagging\n",
			TAIL_TAGGING);
		break;
	}
	printf("%s",buf);
	return len;
}

static ssize_t sysfs_sw_read_hw(int proc_num, ssize_t len, char *buf)
{
	u8 data[8];

	switch (proc_num) {
	case PROC_SET_AGING:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_1,
				SW_AGING_ENABLE));
		break;
	case PROC_SET_FAST_AGING:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_1,
				SW_FAST_AGING));
		break;
	case PROC_SET_LINK_AGING:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_LINK_AGING_CTRL,
				SW_LINK_AUTO_AGING));
		break;
	case PROC_SET_MULTICAST_STORM:
		len += sprintf(buf + len, "%u\n",
			!Ksz_sw_chk(REG_SW_CTRL_2,
				MULTICAST_STORM_DISABLE));
		break;
	case PROC_ENABLE_VLAN:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_MIRROR_CTRL,
				SW_VLAN_ENABLE));
		break;
	case PROC_SET_REPLACE_NULL_VID:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_REPLACE_VID_CTRL,
				SW_REPLACE_VID));
		break;
	case PROC_SET_MAC_ADDR:
		Ksz_sw_get_addr(data);
		len += sprintf(buf + len, "%02X:%02X:%02X:%02X:%02X:%02X\n",
			data[0], data[1], data[2], data[3], data[4], data[5]);
		break;
	case PROC_SET_MIRROR_MODE:
		len += sprintf(buf + len, "%u\n",
				Ksz_sw_chk_mirror_rx_tx());
		break;
	case PROC_SET_IGMP_SNOOP:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_MIRROR_CTRL,
				SW_IGMP_SNOOP));
		break;
#ifdef SW_IPV6_MLD_SNOOP
	case PROC_SET_IPV6_MLD_SNOOP:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_MIRROR_CTRL,
				SW_IPV6_MLD_SNOOP));
		break;
	case PROC_SET_IPV6_MLD_OPTION:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_MIRROR_CTRL,
				SW_IPV6_MLD_OPTION));
		break;
#endif
	case PROC_SET_TAIL_TAG:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_TAIL_TAG_CTRL,
				SW_TAIL_TAG_ENABLE));
		break;
	case PROC_SET_AGGR_BACKOFF:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_1,
				SW_AGGR_BACKOFF));
		break;
	case PROC_SET_NO_EXC_DROP:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_2,
				NO_EXC_COLLISION_DROP));
		break;
#ifdef SW_BUF_RESERVE
	case PROC_SET_BUFFER_RESERVE:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_2,
				SW_BUF_RESERVE));
		break;
#endif
	case PROC_SET_VLAN_BOUNDARY:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_2,
				UNICAST_VLAN_BOUNDARY));
		break;
	case PROC_SET_HUGE_PACKET:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_2,
				SW_HUGE_PACKET));
		break;
	case PROC_SET_LEGAL_PACKET:
		//len += sprintf(buf + len, "%u\n",Ksz_sw_chk(REG_SW_CTRL_2,SW_LEGAL_PACKET));
		break;
	case PROC_SET_LENGTH_CHECK:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_1,
				SW_CHECK_LENGTH));
		break;
	case PROC_SET_BACK_PRESSURE_MODE:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_2,
				SW_BACK_PRESSURE));
		break;
	case PROC_SET_SW_FLOW_CTRL:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_REPLACE_VID_CTRL,
				SW_FLOW_CTRL));
		break;
	case PROC_SET_SW_HALF_DUPLEX:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_REPLACE_VID_CTRL,
				SW_HALF_DUPLEX));
		break;
#ifdef SW_10_MBIT
	case PROC_SET_SW_10_MBIT:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_REPLACE_VID_CTRL,
				SW_10_MBIT));
		break;
#endif
	case PROC_SET_RX_FLOW_CTRL:
		//len += sprintf(buf + len, "%u\n", Ksz_sw_chk(REG_SW_CTRL_1, SW_RX_FLOW_CTRL));
		break;
	case PROC_SET_TX_FLOW_CTRL:
		//len += sprintf(buf + len, "%u\n",	Ksz_sw_chk(REG_SW_CTRL_1,	SW_TX_FLOW_CTRL));
		break;
	case PROC_SET_FAIR_FLOW_CTRL:
		len += sprintf(buf + len, "%u\n",Ksz_sw_chk(REG_SW_CTRL_2,	FAIR_FLOW_CTRL));
		break;
	case PROC_SET_FORWARD_UNKNOWN_DEST:
		len += sprintf(buf + len, "%u\n",
			sw_chk_unk_dest());
		break;
	case PROC_SET_INS_TAG_0_1:
		len += sprintf(buf + len, "%u\n", 	Ksz_sw_chk(REG_SW_CTRL_9, SW_INS_TAG_1_PORT_2));
		break;
	case PROC_SET_INS_TAG_0_2:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_9,	SW_INS_TAG_1_PORT_3));
		break;
	case PROC_SET_INS_TAG_1_0:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_9,	SW_INS_TAG_2_PORT_1));
		break;
	case PROC_SET_INS_TAG_1_2:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_9,
				SW_INS_TAG_2_PORT_3));
		break;
	case PROC_SET_INS_TAG_2_0:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_9,
				SW_INS_TAG_3_PORT_1));
		break;
	case PROC_SET_INS_TAG_2_1:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_9,SW_INS_TAG_3_PORT_2));
		break;
	case PROC_SET_PASS_ALL:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(REG_SW_CTRL_1,
				SW_PASS_ALL));
		break;
	case PROC_SET_PASS_PAUSE:
		len += sprintf(buf + len, "%u\n",
			Ksz_sw_chk(S_LINK_AGING_CTRL,
				SW_PASS_PAUSE));
		break;
	case PROC_DYNAMIC:
		sw_d_dyn_mac_table();
		break;
	case PROC_STATIC:
		sw_d_sta_mac_table();
		sw_d_mac_table();
		break;
	case PROC_VLAN:
		sw_d_vlan_table();
		break;
	}
	return len;
}
ssize_t display_sw_info(int cnt, char *buf, ssize_t len)
{
	printf("\r\n");
	printf("duplex:\t\t 0=AUTO; 1=HDX; 2=FDX\r\n");
	printf("speed:\t\t 0=AUTO; =10; =100\r\n");
	printf("force:\t\t 1=forced linkspeed\r\n");
	printf("flow_ctrl:\t 0=disable\r\n");
	printf("mib:\t\t show MIB\r\n");
	len = 0;

	if (TOTAL_PORT_NUM != cnt)	return len;

	printf("dynamic_table:\t display the switch's dynamic MAC table\r\n");
	printf("static_table:\t display the switch's MAC table\r\n");
	printf("vlan_table:\t display the switch's VLAN table\r\n");

	printf("aging:\t\t disable/enable aging\r\n");
	printf("fast_aging:\t disable/enable fast aging\r\n");
	printf("link_aging:\t disable/enable link change auto aging\r\n");

	printf( "bcast_per:\t");
	printf("set broadcast storm percentage\r\n");
	printf( "mcast_storm:\t");
	printf(	"disable multicast storm protection\r\n");
	printf( "diffserv_map:\t");
	printf(	"set DiffServ value.  Use \"decimal=hexadecimal\" format\r\n");
	printf( "p_802_1p_map:\t");
	printf(	"set 802.1p value.  Use \"decimal=hexadecimal\" format\r\n");

	printf( "vlan:\t\t");
	printf(	"disable/enable 802.1Q VLAN\r\n");
	printf( "null_vid:\t");
	printf(	"set to 1 to replace null vid\r\n");
	printf( "macaddr:\t");
	printf(	"set switch MAC address\r\n");
	printf( "mirror_mode:\t");
	printf(	"set to 1 to use mirror rx AND tx mode\r\n");
	printf( "tail_tag:\t");
	printf(	"disable/enable tail tagging\r\n");

	printf( "igmp_snoop:\t");
	printf(	"disable/enable IGMP snooping\r\n");
	printf( "ipv6_mld_snoop:\t");
	printf(	"disable/enable IPv6 MLD snooping\r\n");
	printf( "ipv6_mld_option:");
	printf(	"disable/enable IPv6 MLD option snooping\r\n");

	printf( "aggr_backoff:\t");
	printf(	"disable/enable aggressive backoff in half-duplex mode\r\n");
	printf( "no_exc_drop:\t");
	printf(	"disable/enable no excessive collision drop\r\n");
	printf( "buf_reserve:\t");
	printf(	"disable/enable buffer reserve\r\n");


	printf( "huge_packet:\t");
	printf(	"disable/enable huge packet support\r\n");
	printf( "legal_packet:\t");
	printf(	"disable/enable legal packet\r\n");
	printf( "length_check:\t");
	printf(	"disable/enable legal packet length check\r\n");

	printf( "back_pressure:\t");
	printf(	"set back pressure mode\r\n");
	printf( "sw_flow_ctrl:\t");
	printf(	"disable/enable switch host port flow control\r\n");
	printf( "sw_half_duplex:\t");
	printf(	"disable/enable switch host port half-duplex mode\r\n");
#ifdef SW_10_MBIT
	printf( "sw_10_mbit:\t");
	printf(	"disable/enable switch host port 10Mbit mode\r\n");
#endif
	printf( "rx_flow_ctrl:\t");
	printf(	"disable/enable receive flow control\r\n");
	printf( "tx_flow_ctrl:\t");
	printf(	"disable/enable transmit flow control\r\n");
	printf( "fair_flow_ctrl:\t");
	printf(	"disable/enable fair flow control mode\r\n");

	printf( "vlan_bound:\t");
	printf(	"disable/enable unicast VLAN boundary\r\n");
	printf( "fw_unk_dest:\t");
	printf(	"disable/enable unknown destination address forwarding\r\n");

	printf( "ins_tag_0_1:\t");
	printf(	"set to 1 to insert tag from port 1 to 2\r\n");
	printf( "ins_tag_0_2:\t");
	printf(	"set to 1 to insert tag from port 1 to 3\r\n");
	printf( "ins_tag_1_0:\t");
	printf(	"set to 1 to insert tag from port 2 to 1\r\n");
	printf( "ins_tag_1_2:\t");
	printf(	"set to 1 to insert tag from port 2 to 3\r\n");
	printf( "ins_tag_2_0:\t");
	printf(	"set to 1 to insert tag from port 3 to 1\r\n");
	printf( "ins_tag_2_1:\t");
	printf(	"set to 1 to insert tag from port 3 to 2\r\n");

	printf( "pass_all:\t");
	printf(	"set to 1 to pass all frames for debugging\r\n");
	printf( "pass_pause:\t");
	printf(	"set to 1 to pass PAUSE frames for debugging\r\n");


	printf( "switch port settings:\r\n");
	printf( "duplex:\t\t");
	printf(	"display the port's duplex setting\r\n");
	printf( "speed:\t\t");
	printf(	"display the port's link speed\r\n");
	printf( "linkmd:\t\t");
	printf(	"write to start LinkMD test.  read for result\r\n");
	printf( "mib:\t\t");
	printf(	"display the port's MIB table\r\n");


	printf( "vid:\t\t");
	printf(	"set default VID value\r\n");
	printf( "member:\t\t");
	printf(	"set VLAN membership\r\n");

	printf( "bcast_storm:\t");
	printf(	"disable/enable broadcast storm protection\r\n");
	printf( "diffserv:\t");
	printf(	"disable/enable DiffServ priority\r\n");
	printf( "p_802_1p:\t");
	printf(	"disable/enable 802.1p priority\r\n");

	printf( "\r\n[port_based]\t");
	printf(	"disable/enable port-based priority\r\n");

	printf( "prio_queue:\t");
	printf(	"disable/enable priority queue\r\n");
	printf( "tx_p0_ctrl:\t");
	printf(	"set priority queue 0 control\r\n");
	printf( "tx_p1_ctrl:\t");
	printf(	"set priority queue 1 control\r\n");
	printf( "tx_p2_ctrl:\t");
	printf(	"set priority queue 2 control\r\n");
	printf( "tx_p3_ctrl:\t");
	printf(	"set priority queue 3 control\r\n");
	printf( "tx_p0_ratio:\t");
	printf(	"set priority queue 0 ratio\r\n");
	printf( "tx_p1_ratio:\t");
	printf(	"set priority queue 1 ratio\r\n");
	printf( "tx_p2_ratio:\t");
	printf(	"set priority queue 2 ratio\r\n");
	printf( "tx_p3_ratio:\t");
	printf(	"set priority queue 3 ratio\r\n");
	printf( "prio_rate:\t");
	printf(	"disable/enable priority queue rate limiting\r\n");
	printf( "rx_limit:\t");
	printf(	"set rx rate limiting mode\r\n");
	printf( "cnt_ifg:\t");
	printf(	"set to 1 to count IPG\r\n");
	printf( "cnt_pre:\t");
	printf(	"set to 1 to count preamble\r\n");
	printf( "rx_p0_rate:\t");
	printf(	"set rx priority queue 0 rate in 64Kbps unit\r\n");
	printf( "rx_p1_rate:\t");
	printf(	"set rx priority queue 1 rate in 64Kbps unit\r\n");
	printf( "rx_p2_rate:\t");
	printf(	"set rx priority queue 2 rate in 64Kbps unit\r\n");
	printf( "rx_p3_rate:\t");
	printf(	"set rx priority queue 3 rate in 64Kbps unit\r\n");
	printf( "tx_p0_rate:\t");
	printf(	"set tx priority queue 0 rate in 64Kbps unit\r\n");
	printf( "tx_p1_rate:\t");
	printf(	"set tx priority queue 1 rate in 64Kbps unit\r\n");
	printf( "tx_p2_rate:\t");
	printf(	"set tx priority queue 2 rate in 64Kbps unit\r\n");
	printf( "tx_p3_rate:\t");
	printf(	"set tx priority queue 3 rate in 64Kbps unit\r\n");

	printf( "rx:\t\t");
	printf(	"disable/enable rx\r\n");
	printf( "tx:\t\t");
	printf(	"disable/enable tx\r\n");
	printf( "learn:\t\t");
	printf(	"disable/enable learning\r\n");

	printf( "\r\n[mirror_port]\t");
	printf(	"disable/enable mirror port\r\n");
	printf( "mirror_rx:\t");
	printf(	"disable/enable mirror receive\r\n");
	printf( "mirror_tx:\t");
	printf(	"disable/enable mirror transmit\r\n");

	printf( "non_vid:\t");
	printf(	"set to 1 to discard non-VID packets\r\n");
	printf( "ingress:\t");
	printf(	"disable/enable ingress VLAN filtering\r\n");
	printf( "ins_tag:\t");
	printf(	"disable/enable insert VLAN tag feature\r\n");
	printf( "rmv_tag:\t");
	printf(	"disable/enable remove VLAN tag feature\r\n");
	printf( "drop_tagged:\t");
	printf(	"disable/enable drop tagged packet feature\r\n");
	printf( "replace_prio:\t");
	printf(	"disable/enable replace 802.1p priority feature\r\n");
	printf( "back_pressure:\t");
	printf(	"disable/enable back pressure in half-duplex mode\r\n");
	printf( "force_flow_ctrl:");
	printf(	"set to 1 to force flow control\r\n");
	printf( "fw_unk_dest:\t");
	printf(	"set to 1 to forward unknown destination address packets\r\n");
	printf( "fw_inv_vid:\t");
	printf(	"set to 1 to forward invalid VID packets\r\n");
	printf( "macaddr:\t");
	printf(	"set port MAC address\r\n");
	printf( "src_filter_0:\t");
	printf(	"disable/enable source filtering port 1 MAC address\r\n");
	printf( "src_filter_1:\t");
	printf(	"disable/enable source filtering port 2 MAC address\r\n");

	printf( "\r\n[MAC table]\r\n");
	printf( "addr:\t\t");
	printf(	"set MAC address\r\n");
	printf( "ports:\t\t");
	printf(	"set destination ports\r\n");
	printf( "override:\t");
	printf(	"set override bit\r\n");
	printf( "use_fid:\t");
	printf(	"set use FID bit\r\n");
	printf( "fid:\t\t");
	printf(	"set FID\r\n");
	printf( "valid:\t\t");
	printf(	"set valid bit and write to table\r\n");
	printf( "\r\n[VLAN table]\r\n");
	printf( "vid:\t\t");
	printf("set VID\r\n");
	printf( "fid:\t\t");
	printf("set FID\r\n");
	printf( "member:\t\t");
	printf(	"set membership\r\n");
	printf( "valid:\t\t");
	printf("set valid bit and write to table\r\n");

	len = 0;

	return len;
}
#ifdef CONFIG_KSZ_STP
//This procedure is used to remember which multicast addresses are accepted to the host controller
//so that the switch can filter those addresses as promiscuous mode is used. It is called within the
//network device set receive mode function.
void Ksz_sw_set_multi( struct net_device *dev,	struct ksz_port *priv)
{
	struct ksz_mac_table *entry;
	struct ksz_alu_table *alu;
	struct netdev_hw_addr *ha;
	int i;
	int found;
	int owner;
	int port = 0;
	struct ksz_sw_info *pinfo = &g_sw.info;

	/*
	// This device is associated with a switch port.
	if (1 == priv->port_cnt) port = priv->first_port + 1;
	owner = 1 << port;

	// Remove old multicast entries.
	for (i = SW_MAC_TABLE_ENTRIES; i < pinfo->multi_net; i++) {
		entry = &info.mac_table[i];
		alu = &pinfo->alu_table[i];
		if (alu->valid && (alu->owner & owner)) {
			// Remove device ownership.
			alu->owner &= ~owner;
			if (!port)
				alu->forward &= ~FWD_MAIN_DEV;
			else if (alu->owner <= 1)
				alu->forward &= ~FWD_STP_DEV;
			if (!alu->owner) {
				alu->valid = 0;
				entry->ports = 0;
				entry->valid = 0;
			}
		}
	}

	netdev_for_each_mc_addr(ha, dev)
	{
		if (!(*ha->addr & 1))
			continue;
		if (pinfo->multi_net == pinfo->multi_sys)
			break;
		found = 0;
		for (i = 0; i < MULTI_MAC_TABLE_ENTRIES; i++) {
			entry = &pinfo->mac_table[i];
			alu = &pinfo->alu_table[i];
			if (alu->valid &&
			    !memcmp(entry->mac_addr, ha->addr, ETH_ALEN)) {
				found = i + 1;
				break;
			}
			if (!alu->valid && !found &&
			    i >= SW_MAC_TABLE_ENTRIES &&
			    i < info.multi_net)
				found = i + 1;
		}
		if (!found) {
			pinfo->multi_net++;
			found = pinfo->multi_net;
		}
		found--;
		if (found >= SW_MAC_TABLE_ENTRIES &&
		    found < pinfo->multi_net) {
			entry = &pinfo->mac_table[found];
			alu = &pinfo->alu_table[found];
			if (port)
				alu->forward |= FWD_STP_DEV;
			else
				alu->forward |= FWD_MAIN_DEV;
			alu->owner |= owner;
			alu->valid = 1;
			memcpy(entry->mac_addr, ha->addr, ETH_ALEN);
			entry->ports = g_sw.PORT_MASK;
			entry->valid = 1;
		}
	}
	*/
}

void Ksz_sw_add_frame( u32 crc, unsigned long now,
	unsigned long expired, int num, int port, int max,
	struct ksz_frame_table *table, int *cnt)
{
	struct ksz_frame_table *entry;
	int i;

	/*
	// Table full.
	if (max == *cnt) {
		for (i = 0; i < max; i++) {
			entry = &table[i];
			if (entry->expired &&
					time_after(now, entry->expired)) {
				entry->expired = 0;
				--(*cnt);
			}
		}
	}
	for (i = 0; i < max; i++) {
		entry = &table[i];
		if (!entry->expired) {
			entry->crc = crc;
			entry->cnt = num;
			entry->port = port;
			if (0 == expired)
				expired = 1;
			entry->expired = expired;
			++(*cnt);
			break;
		}
	}
	*/
}

int Ksz_sw_del_frame( u32 crc, unsigned long now,
	int port, int max, struct ksz_frame_table *table, int *cnt)
{
	struct ksz_frame_table *entry;
	int i;
	int num = 0;

	/*
	for (i = 0; i < max; i++) {
		entry = &table[i];
		if (!entry->expired)
			continue;
		if (crc == entry->crc && port != entry->port) {
			if (time_after(now, entry->expired)) {
				entry->expired = 0;
				--(*cnt);
				break;
			}
			--entry->cnt;

			// No need to retain the entry.
			if (!entry->cnt) {
				entry->expired = 0;
				--(*cnt);
			}
			return i + 1;
		}
		++num;
		if (num == *cnt)
			break;
	}
	*/
	return 0;
}

void Ksz_sw_add_rx( u32 crc, unsigned long now,	unsigned long expired, int num, int port)
{
	//struct ksz_rx_table *pinfo = &g_sw.info.rx_table;

	//Ksz_sw_add_frame( crc, now, expired, num, port,	RX_TABLE_ENTRIES, info.table, &pinfo->cnt);
}

int Ksz_sw_del_rx( u32 crc, unsigned long now, int port)
{
	//struct ksz_rx_table *pinfo = &g_sw.info.rx_table;

	//return Ksz_sw_del_frame( crc, now, port, RX_TABLE_ENTRIES, info.table,&pinfo->cnt);
}  /* sw_del_rx */

void Ksz_sw_add_tx( u32 crc, unsigned long now,
	unsigned long expired, int num, int port)
{
	struct ksz_tx_table *pinfo = &g_sw.info.tx_table;

	Ksz_sw_add_frame( crc, now, expired, num, port,	TX_TABLE_ENTRIES, pinfo->table, &pinfo->cnt);
}  /* sw_add_tx */

int Ksz_sw_del_tx( u32 crc, unsigned long now, int port)
{
	struct ksz_tx_table *pinfo = &g_sw.info.tx_table;

	return Ksz_sw_del_frame( crc, now, port, TX_TABLE_ENTRIES, pinfo->table,	&pinfo->cnt);
}  /* sw_del_tx */

int Ksz_sw_blocked_rx( u8 *data)
{
	int i;
/*
	for (i = 0; i < g_sw.info.blocked_rx_cnt; i++)
		if (!memcmp(data, g_sw.info.blocked_rx[i], ETH_ALEN))
			return true;
	if (BLOCKED_RX_ENTRIES == i)
		g_sw.info.blocked_rx_cnt = 0;
	memcpy(g_sw.info.blocked_rx[g_sw.info.blocked_rx_cnt++], data,
		ETH_ALEN);
*/
	return false;
}

int Ksz_sw_block_rx( u8 *data, int len, int port)
{
	struct ksz_mac_table *entry;
	struct ksz_alu_table *alu;
	u32 crc;
	int i;
	int forward = 0;

	/*
	for (i = 0; i < MULTI_MAC_TABLE_ENTRIES; i++) {


		// * Special case of checking the frame is forwarded to the host.
		// * All entries before STATIC_MAC_TABLE_ENTRIES should have FWD_HOST.
		if (!len && STATIC_MAC_TABLE_ENTRIES == i)
			break;

		entry = &g_sw.info.mac_table[i];
		if (!entry->valid ||
		    memcmp(data, entry->mac_addr, ETH_ALEN))
			continue;

		// Block if received port is closed.
		if (len && !entry->override && !(g_sw.rx_ports & (1 << port)))
			break;

		alu = &g_sw.info.alu_table[i];
		forward = alu->forward;

		// Allow to reach host as the frame is not forwarded.
		if (alu->forward & FWD_HOST)
			break;

		// Remember the frame when forwarding to STP device.
		if ((alu->forward & FWD_STP_DEV) && g_sw.info.fwd_ports > 1) {
			unsigned long now;

			// Port is zero-based.
			port++;
			crc = ether_crc(len, data);
			//now = jiffies;
			//Ksz_sw_add_rx( crc, now, now + 1000 / HZ,	g_sw.info.fwd_ports - 1, port);
		}
		break;
	}

	// Check port state in case it is changed after processing arrived BPDU.
	//if (forward && len && !i) schedule_delayed_work(g_sw.stp_monitor, 1);
	*/
	return forward;
}

int Ksz_sw_block_tx( u8 *data, int len, int port)
{
	struct ksz_mac_table *entry;
	struct ksz_alu_table *alu;
	int i;
	u32 crc = 0;
	unsigned long now = 0;
	int block = false;
	int forward = 0;
/*
	for (i = 0; i < MULTI_MAC_TABLE_ENTRIES; i++) {
		entry = &g_sw.info.mac_table[i];
		if (!entry->valid ||
		    memcmp(data, entry->mac_addr, ETH_ALEN))
			continue;

		alu = &g_sw.info.alu_table[i];
		forward = alu->forward;

		// No need to block.
		if (alu->forward & FWD_HOST)
			break;

		// Check frame is not forwarded by software.
		if (port && (alu->forward & FWD_STP_DEV) &&
		    g_sw.info.fwd_ports > 1) {
			crc = ether_crc(len, data);
			now = jiffies;
			if (sw_del_rx( crc, now, port)) {
				if ((1 << (port - 1)) & g_sw.info.member) {
					block = true;
					forward = 0;
				}
			}
		}
		break;
	}

	// Check duplicate frames sent by main and STP devices.
	if ((forward & (FWD_MAIN_DEV | FWD_STP_DEV)) ==
	    (FWD_MAIN_DEV | FWD_STP_DEV)) {

		// Re-use CRC if already calculated.
		if (!now) {
			crc = ether_crc(len, data);
			now = jiffies;
		}
		if (Ksz_sw_del_tx( crc, now, !!port))
			block = true;
		else
			Ksz_sw_add_tx( crc, now, now + 100 / HZ, 1, !!port);
	}
	return block;
	*/
}
//This function determines whether to accept the received frame or not when STP is used. If
//accepted the forward rule will be retrieved from the stored MAC table. With exceptions for
//special MAC addresses stored in the hardware MAC table, all other addresses will be
//remembered so that the frame can be blocked when the STP daemon forwards it to the other
//ports.

int sw_stp_rx( struct net_device *dev,
	struct sk_buff *skb, int port, int *forward)
{
/*
	if ((g_sw.features & STP_SUPPORT) && br_port_exists(dev)) {
		*forward = Ksz_sw_block_rx( skb->data, skb->len, port);
		if (!*forward && g_sw.dev_offset && dev != g_sw.netdev[0]) {
			dev = g_sw.netdev[0];
			if ((dev->flags & IFF_PROMISC) ||
			    ((dev->flags & IFF_ALLMULTI) &&
			    (skb->data[0] & 1)))
				*forward = FWD_MAIN_DEV;
		}
		return true;
	}
*/
	return false;
}
#endif

struct net_device *Ksz_sw_rx_dev( u8 *data, u32 *len,
	int *tag, int *port)
{
	struct net_device *dev;

	// Get received port number.
	if (g_sw.overrides & TAIL_TAGGING) {
		(*len)--;
		*tag = data[*len]; //4 == CRC? //*tag = data[*len - 4]; //4 == CRC?

		// In case tagging is not working right.
		if (*tag >= SW_PORT_NUM) *tag = 0;

		// Save receiving port.
		*port = *tag;
	}
	dev = g_sw.netdev[*tag + g_sw.dev_offset];
#ifdef KSZ_DLR
	do {
		struct vlan_ethhdr *vlan = (struct vlan_ethhdr *) data;

		if (vlan->h_vlan_proto == htons(ETH_P_8021Q)) {
			if (vlan->h_vlan_encapsulated_proto ==
			    htons(DLR_TAG_TYPE))
				return dev;
		}
	} while (0);
#endif
	if (g_sw.dev_count > 1) {
		u8 stp;

		/*
		stp = g_sw.info.rstp & g_sw.info.stp_down;
		if (stp & (1 << *tag))
			return NULL;
		if (!netif_running(dev))
			return NULL;
			*/
	}
	if (g_sw.features & VLAN_PORT_TAGGING) {
		(*tag)++;
		if (!(g_sw.vlan_id & (1 << *tag)))
			*tag = 0;
	}
	return dev;
}

int pkt_matched(unsigned char *data, struct net_device *dev, void *ptr,
	int (*match_multi)(void *ptr, u8 *addr), u8 h_promiscuous)
{
	int drop = false;
	u8 bcast_addr[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

	if (data[0] & 0x01) {
		if (memcmp(data, bcast_addr, ETH_ALEN))
			drop = match_multi(ptr, data);
	} else if (h_promiscuous && memcmp(data, dev->dev_addr, ETH_ALEN))
		drop = true;
	if (drop)
		return 0;
	//return skb->len;
}
/*
int sw_match_pkt( struct net_device **dev,
	void **priv, int (*get_promiscuous)(void *ptr),
	int (*match_multi)(void *ptr, u8 *data), struct sk_buff *skb,
	u8 h_promiscuous)
{
	int s_promiscuous;

	if (g_sw.dev_count <= 1)
		return true;
	s_promiscuous = get_promiscuous(*priv);
	if (!s_promiscuous && !pkt_matched(skb, *dev, *priv, match_multi,
			h_promiscuous)) {
		int matched = false;

		// There is a parent network device.
		if (g_sw.dev_offset) {
			matched = true;
			*dev = g_sw.netdev[0];
			*priv = netdev_priv(*dev);
			s_promiscuous = get_promiscuous(*priv);
			if (!s_promiscuous && !pkt_matched(skb, *dev, *priv,
					match_multi, h_promiscuous))
				matched = false;
		}
		return matched;
	}
	return true;
}

struct net_device *sw_parent_rx(
	struct net_device *dev, struct sk_buff *skb, int forward,
	struct net_device **parent_dev, struct sk_buff **parent_skb)
{
	if (g_sw.dev_offset && dev != g_sw.netdev[0]) {
		*parent_dev = g_sw.netdev[0];
		if (!forward)
			forward = FWD_MAIN_DEV | FWD_STP_DEV;
		if ((forward & (FWD_MAIN_DEV | FWD_STP_DEV)) ==
		    (FWD_MAIN_DEV | FWD_STP_DEV))
			*parent_skb = skb_copy(skb, GFP_ATOMIC);
		else if (!(forward & FWD_STP_DEV))
			dev = *parent_dev;
	}
	return dev;
}

int sw_port_vlan_rx( struct net_device *dev,
	struct net_device *parent_dev, struct sk_buff *skb, int forward,
	int tag, void *ptr, void (*rx_tstamp)(void *ptr, struct sk_buff *skb))
{
	struct sk_buff *vlan_skb;
	struct net_device *vlan_dev = dev;

	// Add VLAN tag manually.
	if (!(forward & FWD_VLAN_DEV))
		return false;

	if (!tag || !(g_sw.features & VLAN_PORT))
		return false;
	tag += VLAN_PORT_START;
	vlan_skb = skb_copy(skb, GFP_ATOMIC);
	if (!vlan_skb)
		return false;
	skb_reset_mac_header(vlan_skb);
	__vlan_hwaccel_put_tag(vlan_skb, htons(ETH_P_8021Q), tag);
#ifdef CONFIG_1588_PTP
	do {
		struct Ksz_ptp_info *ptp = ptr;

		if (rx_tstamp && (ptp->rx_en & 1))
			rx_tstamp(ptp, vlan_skb);
	} while (0);
#endif
	if (parent_dev && dev != parent_dev) {
		vlan_dev = parent_dev;
		vlan_skb->dev = vlan_dev;
	}
	vlan_skb->protocol = eth_type_trans(vlan_skb, vlan_dev);
	netif_rx(vlan_skb);
	return true;
}   */
/*
int sw_drop_icmp(struct sk_buff *skb, int extra_skb)
{
	int drop = 0;

	if (skb && extra_skb &&	skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph = (struct iphdr *) skb->data;

		drop = (iph->protocol == IPPROTO_ICMP);
	}
	return drop;
}  */

int Ksz_sw_drv_rx( struct sk_buff *skb, int port)
{
	int ret = 1;

#ifdef KSZ_DLR
	ret = dlr_rcv(&g_sw.info.dlr, skb, port);
#endif
	return ret;
}  /* sw_drv_rx */
/*
int sw_get_tx_len( struct sk_buff *skb)
{
	int len = skb->len;

	if (!(g_sw.overrides & TAIL_TAGGING))
		return len;
	if (len < 60)
		len = 60;
	len += 1;
	return len;
} */


//This procedure is used to add the tail tag for sending the frame to specific ports..
//When using DSA this procedure should be defined:
void Ksz_api_add_tail_tag( struct sk_buff *skb, int ports)
{
	u8 *trailer;
	int len = 1;

	trailer = skb_put(skb, len);
	trailer[0] = (u8) ports;
}
// This function is used to find out how many extra bytes are used by the tail tag.
// Normally 1 byte is used, but for PTP message 4 additional bytes are used to store the receive timestamp.
//(NOTE) KSZ9477 has 2 byte tail tag.
// The receive port is also returned.
// When using DSA this function should be defined:


int Ksz_api_get_tail_tag(u8 *trailer, int *port)
{
	int len = 1;

	*port = *trailer;
	return len;
}

int Ksz_add_frag(void *from, char *to, int offset, int len, int odd,
	struct sk_buff *skb)
{
	memcpy(to + offset, from, len);
	return 0;
}
/*
struct sk_buff *sw_check_skb( struct sk_buff *skb,
	struct ksz_port *priv, void *ptr,
	int (*update_msg)(u8 *data, u32 port, u32 overrides))
{
	int len;
	int port;
	struct sk_buff *org_skb;
	int update_dst = (g_sw.overrides & TAIL_TAGGING);

#ifdef CONFIG_1588_PTP
	struct Ksz_ptp_info *ptp = ptr;

	if (ptp->overrides & (PTP_PORT_FORWARD | PTP_PORT_TX_FORWARD))
		update_dst |= 2;
#endif
	if (!update_dst)
		return skb;

#ifdef CONFIG_NET_DSA_TAG_TAIL
	if (skb->protocol == htons(ETH_P_TRAILER))
		return skb;
#endif
#ifdef KSZ_DLR
	if (skb->protocol == htons(DLR_TAG_TYPE))
		return skb;
#endif

	org_skb = skb;
	port = 0;

	// This device is associated with a switch port.
	if (1 == priv->port_cnt)
		port = priv->first_port + 1;

	do {
		u16 prio;
		u16 vid;

		if (!(g_sw.features & VLAN_PORT) || port ||
				vlan_get_tag(skb, &vid))
			break;
		prio = vid & VLAN_PRIO_MASK;
		vid &= VLAN_VID_MASK;
		if (vid < VLAN_PORT_START)
			break;
		vid -= VLAN_PORT_START;
		if (!vid || vid > SW_PORT_NUM)
			break;
		port = vid;

		if (g_sw.vid || prio) {
			struct vlan_ethhdr *vlan =
				(struct vlan_ethhdr *) skb->data;
			u16 vlan_tci = ntohs(vlan->h_vlan_TCI);

			vlan_tci &= ~VLAN_VID_MASK;
			vlan_tci |= g_sw.vid;
			vlan->h_vlan_TCI = htons(vlan_tci);

		// Need to remove VLAN tag manually.
		} else if (!(g_sw.overrides & TAG_REMOVE)) {
			u8 *data;

			len = VLAN_ETH_HLEN - 2;
			data = &skb->data[len];
			memmove(data - VLAN_HLEN, data, skb->len - len);
			skb->len -= VLAN_HLEN;
		}
	} while (0);
#ifdef CONFIG_1588_PTP
	if (ptp) {
		int blocked;
		u32 dst = port;
		u32 overrides = ptp->overrides;

		if (!dst && ptp->version < 1)
			dst = 3;
		if (ptp->features & PTP_PDELAY_HACK) {
			dst |= (u32) g_sw.tx_ports << 16;
			overrides |= PTP_UPDATE_DST_PORT;
		}
		blocked = update_msg(skb->data, dst, overrides);
		if (blocked) {
			dev_kfree_skb_irq(skb);
			return NULL;
		}
	}
#endif
	if (!(g_sw.overrides & TAIL_TAGGING))
		return skb;

	//Socket buffer has no fragments.
	if (!skb_shinfo(skb)->nr_frags) {
#ifdef NET_SKBUFF_DATA_USES_OFFSET
		len = skb_end_pointer(skb) - skb->data;
#else
		len = skb->end - skb->data;
#endif
		if (skb->len + 1 > len || len < 60 + 1) {
			len = (skb->len + 7) & ~3;
			if (len < 64)
				len = 64;
			skb = dev_alloc_skb(len);
			if (!skb)
				return NULL;
			memcpy(skb->data, org_skb->data, org_skb->len);
			skb->len = org_skb->len;
			copy_old_skb(org_skb, skb);
		}
		if (skb->len < 60) {
			memset(&skb->data[skb->len], 0, 60 - skb->len);
			skb->len = 60;
		}
		len = skb->len;
		skb->data[len] = 0;
		if (port)
			skb->data[len] = 1 << (port - 1);
#ifdef CONFIG_KSZ_STP
		else if (g_sw.features & STP_SUPPORT) {
			int forward = Ksz_sw_block_rx( skb->data, 0, 0);

			// Need destination port if lookup is set to forward to host.

			if (forward & FWD_HOST) {
				port = g_sw.tx_ports & ~g_sw.HOST_MASK;
				skb->data[len] = (u8) port;
			}
		}
#endif
		skb_put(skb, 1);
	} else {
		struct sock *sk;

		sk = skb->sk;
		if (!sk) {
			struct sock dummy;

			sk = &dummy;
			sk->sk_allocation = GFP_KERNEL;
			//atomic_set(&sk->sk_wmem_alloc, 1);
		}

		// Clear last tag.
		g_sw.tx_pad[g_sw.tx_start] = 0;
		g_sw.tx_start = 0;
		len = 1;
		if (skb->len < 60) {
			g_sw.tx_start = 60 - skb->len;
			len += g_sw.tx_start;
		}
		g_sw.tx_pad[g_sw.tx_start] = 0;
		if (port)
			g_sw.tx_pad[g_sw.tx_start] = 1 << (port - 1);
		skb_append_datato_frags(sk, skb, add_frag, g_sw.tx_pad, len);
	}
	return skb;
}

struct sk_buff *sw_check_tx( struct net_device *dev,
	struct sk_buff *skb, struct ksz_port *priv)
{
	void *ptr = NULL;
	int (*update_msg)(u8 *data, u32 port, u32 overrides) = NULL;

#ifdef CONFIG_1588_PTP
	if (g_sw.features & PTP_HW) {
		struct Ksz_ptp_info *ptp = &g_sw.ptp_hw;

		ptr = ptp;
		update_msg = ptp->ops->update_msg;
	}
#endif

#ifdef CONFIG_KSZ_STP
	if (g_sw.features & STP_SUPPORT) {
		int port = 0;

		// This device is associated with a switch port.
		if (1 == priv->port_cnt)
			port = priv->first_port + 1;
		if ((br_port_exists(dev) || !port) &&
		    sw_block_tx( skb->data, skb->len, port)) {
			dev_kfree_skb_irq(skb);
			return NULL;
		}
	}
#endif
	return sw_check_skb( skb, priv, ptr, update_msg);
}  */

#ifdef CONFIG_NET_DSA_TAG_TAIL
struct sk_buff *tail_xmit(struct sk_buff *skb, struct net_device *dev)
{
	/*
	struct sk_buff *nskb;
	int padlen;
	int addlen = 8;

	if (skb->protocol == htons(ETH_P_TRAILER))
		return skb;

	// We have to make sure that the trailer ends up as the very last 4 bytes of the packet.  This means that we have to pad the packet to the minimum ethernet frame size, if necessary,
	// before adding the trailer.

	padlen = 0;
	if (skb->len < 60)
		padlen = 60 - skb->len;

	nskb = alloc_skb(NET_IP_ALIGN + skb->len + padlen + addlen, GFP_ATOMIC);
	if (nskb == NULL) {
		dev_kfree_skb_irq(skb);
		return NULL;
	}
	skb_reserve(nskb, NET_IP_ALIGN);

	skb_reset_mac_header(nskb);
	skb_set_network_header(nskb, skb_network_header(skb) - skb->head);
	skb_set_transport_header(nskb, skb_transport_header(skb) - skb->head);
	skb_copy_and_csum_dev(skb, skb_put(nskb, skb->len));
	nskb->dev = skb->dev;
	dev_kfree_skb_irq(skb);

	if (padlen) {
		u8 *pad = skb_put(nskb, padlen);
		memset(pad, 0, padlen);
	}

	g_sw.net_ops->add_tail_tag( nskb, 0);

	nskb->protocol = htons(ETH_P_TRAILER);

	return nskb;
*/
}
#endif

struct sk_buff *sw_final_skb( struct sk_buff *skb,
	struct net_device *dev, struct ksz_port *port)
{
	/*
#ifdef CONFIG_NET_DSA_TAG_TAIL
	skb = tail_xmit(skb, dev, sw);
	if (!skb)
		return NULL;
#endif

	//skb = g_sw.net_ops->check_tx( dev, skb, port);
	if (!skb)
		return NULL;

#ifdef CONFIG_1588_PTP
	if (g_sw.features & PTP_HW) {
		struct Ksz_ptp_info *ptp = &g_sw.ptp_hw;

		if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)
			ptp->ops->get_tx_tstamp(ptp, skb);
	}
#endif
	return skb;
	*/
}
void Ksz_sw_ena_intr(){

	Ksz_setReg8(REG_INT_ENABLE, g_sw.intr_mask);
	Ksz_setReg8(REG_ACL_INT_ENABLE, INT_PORT_ALL);
}
void Ksz_sw_handle_intr() //YOON
{
	u8 status;
	status = Ksz_getReg8(REG_INT_STATUS);
	if(status & g_sw.intr_mask){
		//...
	}
}

void Ksz_sw_start( u8 *hostMacAddr)
{
	int need_tail_tag = true;

	Ksz_sw_setup();//sw);
	Ksz_sw_enable(); //make port forwarding and start switch.
	Ksz_sw_set_addr(hostMacAddr);

	if (1 == g_sw.dev_count)
		Ksz_sw_setup_src_filter(hostMacAddr);
#ifdef KSZ_DLR
	need_tail_tag = true;
#endif
	if (g_sw.dev_count > 1)
		need_tail_tag = true;

	if (g_sw.features & VLAN_PORT) {
		if (g_sw.features & VLAN_PORT_REMOVE_TAG) {
			int i;
			for (i = 0; i < SW_PORT_NUM; i++)
				Ksz_port_cfg_rmv_tag( i, true);
			g_sw.overrides |= TAG_REMOVE;
		}
		if (g_sw.features & VLAN_PORT_TAGGING)
			need_tail_tag = true;
	}

	if (g_sw.features & DSA_SUPPORT) {
		int p;
		need_tail_tag = true;
		for (p = 0; p < g_sw.mib_port_cnt - 1; p++) {
			Ksz_sw_cfg_each_port( p, false);
			Ksz_port_set_stp_state(p,STP_STATE_SIMPLE);
		}
		Ksz_sw_cfg_each_port( p, true);
		Ksz_port_set_stp_state( p, STP_STATE_SIMPLE);
	}

	if (need_tail_tag) {
		Ksz_sw_cfg_tail_tag( true); //Enable Tail tag function.
		g_sw.overrides |= TAIL_TAGGING;
	}

	g_sw.intr_mask = INT_PORT_1 | INT_PORT_2 | INT_PORT_3 | INT_PME;
	Ksz_sw_ena_intr();
	//g_sw.ops->release(sw);
#ifdef CONFIG_1588_PTP
	if (g_sw.features & PTP_HW) {
		struct Ksz_ptp_info *ptp = &g_sw.ptp_hw;

		ptp_start(ptp, true);
	}
#endif
}

int Ksz_sw_stop( int complete)
{
	int reset = false;

#ifdef CONFIG_1588_PTP
	if (g_sw.features & PTP_HW) {
		struct Ksz_ptp_info *ptp = &g_sw.ptp_hw;

		reset = ptp->ops->stop(ptp);
	}
#endif
	//g_sw.ops->acquire(sw);
	if (!reset)
		Ksz_sw_reset();
	reset = true;
	Ksz_sw_init();

	/* Clean out MAC table when the switch shutdown. */
	if ((g_sw.features & STP_SUPPORT) && complete)
		Ksz_sw_clr_sta_mac_table();
	//g_sw.ops->release(sw);
	return reset;
}  /* sw_stop */

void Ksz_sw_init_mib()//struct ksz_sw *sw)
{
	int i;
/*
	for (i = 0; i < g_sw.mib_port_cnt; i++) {
		g_sw.port_mib[i].mib_start = 0;
		if (next_jiffies < jiffies)
			next_jiffies = jiffies + HZ * 2;
		else
			next_jiffies += MIB_READ_INTERVAL;
		g_sw.counter[i].time = next_jiffies;
		g_sw.port_state[i].media_state = MEDIA_DISCONNECTED;
		port_init_cnt( i);
	}
	g_sw.port_state[g_sw.HOST_PORT].state = MEDIA_CONNECTED;
*/
}

void Ksz_sw_open_dev(u8 *macaddr)
{
	Ksz_sw_init_mib();
	Ksz_sw_start(macaddr);
	//g_sw.main_dev = dev;
}


/**
 * port_get_link_speed - get current link status
 * @port:	The port instance.
 *
 * This routine reads PHY registers to determine the current link status of the
 * switch ports.
 */
static int Ksz_All_port_get_link_speed_and_state()//struct ksz_port *port)
{
	struct ksz_port_info *port_info;
	struct ksz_port_info *linked = NULL;
	//struct ksz_port_state *state;
	//struct ksz_sw *sw = port->sw;
	SW_D data;
	SW_D status;
	SW_D link_status;
	SW_D local_status;
	SW_D remote_link_status;
	int i;
	int pid;
	int change = 0;

	// * Only check port which has interrupts triggered. If no interrupt poll all the ports with PHY.
	if (!g_sw.phy_intr)	g_sw.phy_intr = g_sw.PORT_MASK;

	for (i = 0, pid = g_sw.first_port; i < g_sw.port_cnt; i++, pid++) {
		if (!(g_sw.phy_intr & (1 << pid)))
			continue;

		port_info = &g_sw.port_info[pid];
		//state = &g_sw.port_info[pid].media_state;
		Ksz_port_r8(pid, P_LOCAL_CTRL, &local_status);
		Ksz_port_r8(pid, P_REMOTE_STATUS, &remote_link_status);
		Ksz_port_r8(pid, P_SPEED_STATUS, &status);
		data = remote_link_status;

		// The partner capability register is updated but the  auto-negotiation is not completed yet.
		link_status = data & (PORT_AUTO_NEG_COMPLETE | PORT_STAT_LINK_GOOD);

		if (data & PORT_STAT_LINK_GOOD) {
			// Remember the first linked port.
			if (!linked)linked = port_info;
		}

		// No change to status.
		if (local_status == port_info->advertised && link_status == port_info->link_status)
			continue;

		//printf("advertised: %02X-%02X; partner: %02X-%02X\n", local, info->advertised, remote, info->partner);
		if(data & PORT_STAT_LINK_GOOD ){
			if (Ksz_port_chk_force_link_and_state(pid, remote_link_status, status))	{
				if (linked == port_info)	linked = NULL;
				continue;
			}
			port_info->tx_rate = 10 * TX_RATE_UNIT;
			if (status & PORT_STAT_SPEED_100MBIT) port_info->tx_rate = 100 * TX_RATE_UNIT;

			port_info->duplex = 1;
			if (status & PORT_STAT_FULL_DUPLEX)  port_info->duplex = 2;

//#ifdef DEBUG
			printf("flow_ctrl: "SW_SIZE_STR"\n", status & (PORT_RX_FLOW_CTRL | PORT_TX_FLOW_CTRL));
//#endif
			if (MEDIA_CONNECTED != port_info->media_state) {
				//info->flow_ctrl = sw_determine_flow_ctrl(port, local, remote);
				//if (status & PORT_RX_FLOW_CTRL)		info->flow_ctrl |= 0x10;
				//if (status & PORT_TX_FLOW_CTRL)		info->flow_ctrl |= 0x20;
				//if (sw->info)		port_cfg_back_pressure(sw, p,	(1 == info->duplex));
				//change |= 1 << i;
			}
			port_info->media_state = MEDIA_CONNECTED;
		} else {
			if (MEDIA_DISCONNECTED != port_info->media_state) {
				change |= 1 << i;
				// Indicate the link just goes down.
				port_info->link_down = 1;
			}
			port_info->media_state = MEDIA_DISCONNECTED;
		}
		//state->media_state = port_info->media_state;
		port_info->report = true;
		port_info->advertised = local_status;
		port_info->partner = remote_link_status;
		//port_info->link = link;
	}
	g_sw.phy_intr = 0;

	//if (linked && MEDIA_DISCONNECTED == port->linked->state)
	//	port->linked = linked;

#ifdef DEBUG
	if (change)
		Ksz_dPrintf_AllLinkStatus(change); //Ksz_dPrintf_AllLinkStatus(port, change);
#endif
	if (change){// && port->first_port < SW_PORT_NUM){
		//schedule_work(&port->link_update);
	}

	return change;

}

/**
 * port_set_link_speed - set port speed
 * @port:	The port instance.
 *
 * This routine sets the link speed of the switch ports.
 */
void Ksz_All_port_set_link_speed_and_state()//struct ksz_port *port)
{
	SW_D data;
	SW_D cfg;
	SW_D status;
	int i;
	int p;

	for (i = 0, p = g_sw.first_port; i < g_sw.port_cnt; i++, p++) {
		if (g_sw.port_info[p].fiber)
			continue;

		Ksz_port_r8(p, REG_PORT_CTRL_9, &data);//P_PHY_CTRL, &data); //0x1C
		Ksz_port_r8(p, REG_PORT_STATUS_2,&status);// P_LINK_STATUS, &status); //0x1E

		cfg = 0;
		if (status & PORT_STATUS_LINK_GOOD) //Link Good.
			cfg = data;

		data &= ~PORT_AUTO_NEG_DISABLE;//Enable.	data |= PORT_AUTO_NEG_ENABLE;
		//data = port_advertised_flow_ctrl(port, data);

		data |= PORT_AUTO_NEG_100BTX_FD | PORT_AUTO_NEG_100BTX | PORT_AUTO_NEG_10BT_FD | PORT_AUTO_NEG_10BT;

		// Check if manual configuration is specified by the user.
		if (g_sw.port_info[p].speed || g_sw.port_info[p].duplex) {
			if (10 == g_sw.port_info[p].speed)
				data &= ~(PORT_AUTO_NEG_100BTX_FD |	PORT_AUTO_NEG_100BTX);
			else if (100 == g_sw.port_info[p].speed)
				data &= ~(PORT_AUTO_NEG_10BT_FD | 	PORT_AUTO_NEG_10BT);
			if (1 == g_sw.port_info[p].duplex)
				data &= ~(PORT_AUTO_NEG_100BTX_FD | PORT_AUTO_NEG_10BT_FD);
			else if (2 == g_sw.port_info[p].duplex)
				data &= ~(PORT_AUTO_NEG_100BTX | PORT_AUTO_NEG_10BT);
		}

		if (data != cfg) {
#if (SW_SIZE == (1))
			Ksz_port_w8(p, P_PHY_CTRL, data);
			Ksz_port_r8(p, P_NEG_RESTART_CTRL, &data);
#endif
			data |= PORT_AUTO_NEG_RESTART;
			Ksz_port_w8(p, P_NEG_RESTART_CTRL, data);

			// Link is going down.
			g_sw.port_info[p].media_state = MEDIA_DISCONNECTED;
		}
	}

}
/**
 * port_force_link_speed - force port speed
 * @port:	The port instance.
 *
 * This routine forces the link speed of the switch ports.
 */
void Ksz_All_port_force_link_speed_and_state()//struct ksz_port *port)
{
	SW_D data;
	int i;
	int pid;

	for (i = 0, pid = g_sw.first_port; i < g_sw.port_cnt; i++, pid++) {
		Ksz_port_r8(pid, P_PHY_CTRL, &data);
		data |= PORT_AUTO_NEG_DISABLE;//Disable.	//data &= ~PORT_AUTO_NEG_ENABLE;
		if (10 == g_sw.port_info[pid].speed)
			data &= ~PORT_FORCE_100_MBIT;
		else if (100 == g_sw.port_info[pid].speed)
			data |= PORT_FORCE_100_MBIT;
		if (1 == g_sw.port_info[pid].duplex)
			data &= ~PORT_FORCE_FULL_DUPLEX;
		else if (2 == g_sw.port_info[pid].duplex)
			data |= PORT_FORCE_FULL_DUPLEX;
		Ksz_port_w8(pid, P_PHY_CTRL, data);
	}

}
/*
void Ksz_sw_open_port(u8 *state)
{
	int i;
	int p;

	for (i = 0, p = g_sw.first_port; i < g_sw.port_cnt; i++, p++) {
		// Initialize to invalid value so that link detection is done.
		g_sw.port_info[p].partner = 0xFF;
		g_sw.port_info[p].media_state = MEDIA_UNKNOWN;
	}

	// Need to open the port in multiple device interfaces mode.
	if (g_sw.dev_count > 1 && (!g_sw.dev_offset || dev != g_sw.netdev[0])) {
		*state = STP_STATE_SIMPLE;
		Ksz_port_set_stp_state( port->first_port, *state);
		//Ksz_port_set_addr( port->first_port, dev->dev_addr);
		//Ksz_port_cfg_src_filter_0( port->first_port, 1);
		//Ksz_port_cfg_src_filter_1( port->first_port, 1);
	}

	//g_sw.phy_intr = PORT_MASK;
	if (g_sw.force_link)
		Ksz_All_port_force_link_speed_and_state();
	else
		Ksz_All_port_set_link_speed_and_state();
	Ksz_All_port_set_link_speed_and_state();
	g_sw.phy_intr = 0;

	//g_sw.ops->release(sw);
}

void Ksz_sw_close_port()// struct net_device *dev,struct ksz_port *port)
{
	// Need to shut the port manually in multiple device interfaces mode.
	Ksz_port_set_stp_state( &g_sw.port_info[g_sw.first_port], STP_STATE_DISABLED);

#ifdef CONFIG_KSZ_STP
		// Port is closed.  Need to change bridge setting.
		if (g_sw.features & STP_SUPPORT){
			int pi;

			pi = 1 << port->first_port;
			if (g_sw.info.member & pi) {
				g_sw.info.member &= ~pi;

				// No ports in forwarding state.
				if (!g_sw.info.member) {
					Ksz_port_set_stp_state( SW_PORT_NUM, STP_STATE_SIMPLE);
					Ksz_sw_block_addr();
					Ksz_sw_block_multi();
				}
				Ksz_bridge_change();
			}
		}
#endif
}
*/

void Ksz_sw_open(struct ksz_sw *sw)
{
#ifdef KSZ_DLR
	prep_dlr(&g_sw.info.dlr, g_sw.main_dev, g_sw.main_dev->dev_addr);
#endif
#ifdef CONFIG_1588_PTP
	if (g_sw.features & PTP_HW) {
		int i;
		struct Ksz_ptp_info *ptp = &g_sw.ptp_hw;

		for (i = 0; i < MAX_PTP_PORT; i++)
			ptp->linked[i] = (g_sw.port_info[i].state ==
				MEDIA_CONNECTED);
	}
#endif
	/* Timer may already be started by the SPI device. */
	if (!g_sw.monitor_timer_info.max)
		Ksz_start_timer(&g_sw.monitor_timer_info, g_sw.monitor_timer_info.period);
}  /* sw_open */

void sw_close(struct ksz_sw *sw)
{
	ksz_stop_timer(&g_sw.monitor_timer_info);
	//cancel_delayed_work_sync(g_sw.link_read);
	//cancel_delayed_work_sync(g_sw.stp_monitor);
}  /* sw_close */

//This function updates the switch when the host network device MAC address is changed.
//In multiple devices mode each port may have its own MAC address.
//In that case the host network device controller needs to be put in promiscuous mode for it to receive different unicast packets.
//The returned hardware promiscuous count will tell the host controller whether to turn on
//promiscuous mode or not.
u8 Ksz_sw_set_mac_addr( struct net_device *dev, u8 promiscuous, int port)
{
	//g_sw.ops->acquire(sw);
	if (g_sw.dev_count > 1 && (!g_sw.dev_offset || dev != g_sw.netdev[0])) {
		Ksz_port_set_addr( port, dev->dev_addr);
		if (g_sw.features & DIFF_MAC_ADDR) {
			g_sw.features &= ~DIFF_MAC_ADDR;
			--promiscuous;
		}
		for (port = 0; port < SW_PORT_NUM; port++)
			if (memcmp(g_sw.port_info[port].mac_addr,
					dev->dev_addr, ETH_ALEN)) {
				g_sw.features |= DIFF_MAC_ADDR;
				++promiscuous;
				break;
			}
	} else {
		int i;

		Ksz_sw_setup_src_filter( dev->dev_addr);

		/* Make MAC address the same in all the ports. */
		if (g_sw.dev_count > 1) {
			for (i = 0; i < SW_PORT_NUM; i++)
				memcpy(g_sw.netdev[i + 1]->dev_addr,
					dev->dev_addr, ETH_ALEN);
			if (g_sw.features & DIFF_MAC_ADDR) {
				g_sw.features &= ~DIFF_MAC_ADDR;
				--promiscuous;
			}
		}
		//if (dev == g_sw.netdev[0])
		//	Ksz_sw_set_addr( dev->dev_addr);
	}
	//g_sw.ops->release(sw);
#ifdef CONFIG_1588_PTP
	if (g_sw.features & PTP_HW) {
		struct Ksz_ptp_info *ptp = &g_sw.ptp_hw;

		ptp->ops->set_identity(ptp, dev->dev_addr);
	}
#endif
#ifdef KSZ_DLR
	dlr_change_addr(&g_sw.info.dlr, dev->dev_addr);
#endif
	return promiscuous;
}  /* sw_set_mac_addr */

/*
 * This enables multiple network device mode for the switch, which contains at
 * least two physical ports.  Some users like to take control of the ports for
 * running Spanning Tree Protocol.  The driver will create an additional eth?
 * device for each port depending on the mode.
 *
 * Some limitations are the network devices cannot have different MTU and
 * multicast hash tables.
 */
int multi_dev;

/*
 * As most users select multiple network device mode to use Spanning Tree
 * Protocol, this enables a feature in which most unicast and multicast packets
 * are forwarded inside the switch and not passed to the host.  Only packets
 * that need the host's attention are passed to it.  This prevents the host
 * wasting CPU time to examine each and every incoming packets and do the
 * forwarding itself.
 *
 * As the hack requires the private bridge header, the driver cannot compile
 * with just the kernel headers.
 *
 * Enabling STP support also turns on multiple network device mode.
 */
int stp;

/*
 * This enables fast aging in the switch.  Not sure what situation requires
 * that.  However, fast aging is used to flush the dynamic MAC table when STP
 * support is enabled.
 */
int fast_aging;

void Ksz_sw_setup_special( int *port_cnt, int *mib_port_cnt, int *dev_cnt)
{

	g_sw.dev_offset = 0;
	g_sw.phy_offset = 0;
	g_sw.dev_count = 1;
	g_sw.multi_dev = 1;

	if (g_sw.stp) {
		g_sw.fast_aging = 1;
		g_sw.multi_dev = 1;
#ifdef CONFIG_1588_PTP
		g_sw.multi_dev = 5;
#endif
		g_sw.features |= STP_SUPPORT;
	}
	if (g_sw.fast_aging)
		g_sw.overrides |= FAST_AGING;
/*
	// Multiple network device interfaces are required.
	if (1 == g_sw.multi_dev) {
		g_sw.dev_count = SW_PORT_NUM;
		//g_sw.phy_offset = 1;
	} else if (2 == g_sw.multi_dev)
		g_sw.features |= VLAN_PORT | VLAN_PORT_TAGGING;
	else if (3 == g_sw.multi_dev) {
		g_sw.dev_count = SW_PORT_NUM;
		//g_sw.dev_offset = 1;
	} else if (4 == g_sw.multi_dev)
		g_sw.features |= VLAN_PORT;
	else if (5 == g_sw.multi_dev) {
		g_sw.dev_count = SW_PORT_NUM;
		//g_sw.dev_offset = 1;
		g_sw.features |= VLAN_PORT | VLAN_PORT_TAGGING;
	}
*/
	// Single network device has multiple ports.
	if (1 == g_sw.dev_count) {
		*port_cnt = SW_PORT_NUM;
		*mib_port_cnt = SW_PORT_NUM;
	}
	*dev_cnt = g_sw.dev_count;
	if (3 == g_sw.multi_dev || 5 == g_sw.multi_dev)
		(*dev_cnt)++;
#ifdef CONFIG_1588_PTP
	if (g_sw.features & VLAN_PORT) {
		struct Ksz_ptp_info *ptp = &g_sw.ptp_hw;

		ptp->overrides |= PTP_PORT_FORWARD;
	}
#endif
}

void Ksz_sw_setup_dev( struct net_device *dev, char *dev_name,
		//struct ksz_port *port,
		int host_port_id, int port_cnt,	int mib_port_cnt)
{
	int cnt;
	int p;
	int pi;

	p = host_port_id;
	//if (p)	p -= g_sw.dev_offset;
	//if (g_sw.dev_offset) {
		// First device associated with switch has been created.
		if (host_port_id)
			snprintf(dev->name, IFNAMSIZ, "%s.10%%d", dev_name);
		else {
			g_sw.port_cnt = SW_PORT_NUM;
			g_sw.mib_port_cnt = SW_PORT_NUM;
		}
	//}
	g_sw.port_cnt = port_cnt;
	g_sw.mib_port_cnt = mib_port_cnt;
	g_sw.first_port = 0;//p;
	g_sw.flow_ctrl = PHY_FLOW_CTRL;
	//port->sw = sw;

	//port->linked = &g_sw.port_info[g_sw.first_port];

	for (cnt = 0, pi = p; cnt < port_cnt; cnt++, pi++) {
		g_sw.port_info[pi].phy_id = pi; //was .port_id
		g_sw.port_info[pi].media_state = MEDIA_DISCONNECTED;
		g_sw.netdev[pi + g_sw.dev_offset] = dev;
	}

	//INIT_WORK(&port->link_update, link_update_work);
	//if (g_sw.features & VLAN_PORT)	dev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
}

//Microchip tail tagging switch DSA setup code
void Ksz_setup_dsa_switch_init()
{
	int i;

	if (g_sw.port_cnt == g_sw.mib_port_cnt)
		i = g_sw.mib_port_cnt - 1;
	else
		i = g_sw.port_cnt;
	if (i > 6)
		i = 6;
	g_sw.dsa_port_cnt = i;
	//strcpy(ksz_switch_chip_data.port_names[i], "cpu");
	//while (i < 6)
	//	ksz_switch_chip_data.port_names[++i] = NULL;
}


//void Ksz8794_AN_EnDisablePerPort(unsigned char portId, unsigned char OnOff){ //PortId=1,2,(3)
	//The register bit value is the INVERT of the strap value at the pin.
//	unsigned regval;
//	regval = Ksz8794_getReg(0x1C);
//	Ksz8794_setReg(0x1C, regval & 0x7f);
//}
struct net_device g_dev;
//struct ksz_port g_main_port;

//=======================================================================================================
extern struct STP_BRIDGE* gp_bridge;

int stmKszRstpPtpLoop(char *str)
{
	int seq;
	u32 ret,x;
	TimeInternal currentTod;
	static struct STP_BRIDGE bridge;

	long dval = 0;
	u8 HOST_MAC_ADDR[6] = {0x00,0x01,0x2,0x3,0x4,0x5};

	//Install Systck Interrupt for 1msec local tick
	Init_SysTick(1000);//1msec

	//MCU's Ethernet Config.
	ETH_PTP_ForSwitchBoard_Config(str);

#if (PTP_PROTOCOL ==IEEE8021AS)
	lwIP_Init_8021as(1);//macAddr);
#else
	LwIP_Init(); // we call stm32_low_level_init() twice. Why?
#endif

	Ksz_ConfigSPI_CheckWeHave(USE_SPI2,0x84,0x63); //KSZ8463

	Ksz_setup_dsa_switch_init();
	Ksz_sw_init();
	Ksz_sw_setup_dev(&g_dev,
			"KSZ", //IFNAME
//			&g_main_port,
			4, //main port id
			3, //port_cnt
			3);//int mib_port_cnt);

	Ksz_sw_open_dev(HOST_MAC_ADDR);//Ksz_sw_init_mib();//Ksz_sw_start(HOST_MAC_ADDR); //setup, enable, set_addr,
	//Ksz_sw_open_port(); //Ksz_open_port(&bp->port, &bp->state);
	//Ksz_phy_start(bp->phy_dev);

	sysfs_sw_read(PROC_SW_INFO, NULL);
	sysfs_sw_read(PROC_SET_SW_DUPLEX,&g_sw.port_info[1]);

	//PTP Init
	stmPtpInit(&currentTod);

	//RSTP Init
	gp_bridge = stmRstpInit_KSZ8794 ();//&bridge);
	yAssert(gp_bridge != NULL);

//#if (PTP_PROTOCOL ==IEEE8021AS)
//	ieee8021as_init_and_loop();//call ieee8021as_thread()
//#elif (PTP_PROTOCOL ==IEEE1588V2)

	printf("Start RSTP and PTPd...\r\n");

	static unsigned int oneSecondTimerTickCount;
	oneSecondTimerTickCount = millis(); //Timer_GetTimeMilliseconds();
	static unsigned int portCheckTimerTickCount;
	portCheckTimerTickCount =  millis(); //Timer_GetTimeMilliseconds();

	while(1){
		//unsigned int timestamp;

		//timestamp = Timer_GetTimeMilliseconds();
		stmRstpInnerLoop(gp_bridge, &oneSecondTimerTickCount, &portCheckTimerTickCount,millis());//timestamp);

		stmPtpInLooop();
	}

//#endif
}


/**
 * struct ksz_port - Virtual port data structure
 * @first_port:		Index of first port this port supports.
 * @mib_port_cnt:	Number of ports with MIB counters.
 * @port_cnt:		Number of ports this port supports.
 * @flow_ctrl:		Flow control setting.  PHY_NO_FLOW_CTRL for no flow
 *					control, and PHY_FLOW_CTRL for flow control.
 *					PHY_TX_ONLY and PHY_RX_ONLY are not supported for 100 Mbit PHY.
 * @duplex:			Duplex mode setting.  1 for half duplex, 2 for full
 *					duplex, and 0 for auto, which normally results in fullduplex.
 * @speed:			Speed setting.  10 for 10 Mbit, 100 for 100 Mbit, and
 *					0 for auto, which normally results in 100 Mbit.
 * @force_link:		Force link setting.  0 for auto-negotiation, and 1 for force.
 * @linked:			Pointer to port information linked to this port.
 * @sw:				Pointer to virtual switch structure.
 */
/*
struct ksz_port {
	int first_port;
	int mib_port_cnt;
	int port_cnt;

	u8 flow_ctrl;
	u8 duplex;
	u8 speed;
	u8 force_link;

	//struct ksz_port_info *linked;

	//struct ksz_sw *sw;
	//struct work_struct link_update;
};
*/
//========micrel PTP =====================================
/*
 * PTP 1588 clock using Microchip PTP switch
 */
#ifdef CONFIG_1588_PTP

#define N_ALARM		0
#define N_EXT_TS	2

struct Ksz_ptp_info {
	//struct ptp_clock *clock;
	//struct ptp_clock_info caps;
	//struct Ksz_ptp_info *ptp;

	u32 clock_events;
	//u64 alarm_interval;
	//u64 alarm_value;
};

void ptp_event_pps(struct Ksz_ptp_info *info)
{
	//struct ptp_clock_event event;

	//event.type = PTP_CLOCK_PPS;
	//ptp_clock_event(info.clock, &event);
}

#if 0
void ptp_event_alarm(struct Ksz_ptp_info *info)
{
	struct ptp_clock_event event;

	event.type = PTP_CLOCK_ALARM;
	event.index = 0;
	event.timestamp = info.alarm_value;
	ptp_clock_event(info.clock, &event);
}
#endif

void ptp_event_trigger(struct Ksz_ptp_info *info, int index,
	u32 sec, u32 nsec)
{
	struct ptp_clock_event event;

	event.type = PTP_CLOCK_EXTTS;
	event.index = index;
	event.timestamp = sec;
	event.timestamp *= NSEC_PER_SEC;
	event.timestamp += nsec;
	ptp_clock_event(info.clock, &event);
}

// * PTP clock operations

int ptp_adjfreq(struct ptp_clock_info *clock, s32 ppb)
{
	struct ptp_clk_options clk_opt;
	int output;
	struct Ksz_ptp_info *info =	container_of(clock, struct Ksz_ptp_info, caps);
	int err = 0;

	output = 1;
	clk_opt.sec = clk_opt.nsec = 0;
	clk_opt.drift = -ppb;
	clk_opt.interval = NANOSEC_IN_SEC;
	err = proc_ptp_hw_access(info.ptp,	DEV_CMD_PUT, DEV_PTP_CLK, output,&clk_opt, sizeof(clk_opt), NULL, &output,true);
	return err;
}

int ptp_adjtime(struct ptp_clock_info *clock, s64 delta)
{
	struct ptp_clk_options clk_opt;
	int output;
	struct Ksz_ptp_info *info = container_of(clock, struct Ksz_ptp_info, caps);
	int neg_adj = 0;
	int err = 0;

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}
	clk_opt.sec = div_u64_rem(delta, NSEC_PER_SEC, &clk_opt.nsec);
	if (!neg_adj)
		output = 2;
	else
		output = 1;
	clk_opt.interval = 0;
	err = proc_ptp_hw_access(info.ptp,	DEV_CMD_PUT, DEV_PTP_CLK, output,&clk_opt, sizeof(clk_opt), NULL, &output,true);
	return err;
}

int ksz_ptp_gettime(struct ptp_clock_info *clock, struct timespec *ts)
{
	struct ptp_clk_options clk_opt;
	int output;
	//struct Ksz_ptp_info *info = container_of(clock, struct Ksz_ptp_info, caps);
	int err = 0;

	if (!ts)
		return -info.ptp->drift;
	err = proc_ptp_hw_access(info.ptp,	DEV_CMD_GET, DEV_PTP_CLK, 0,&clk_opt, sizeof(clk_opt), NULL, &output,true);
	if (err)return err;
	ts->tv_sec = clk_opt.sec;
	ts->tv_nsec = clk_opt.nsec;
	return 0;
}

int ksz_ptp_settime(struct ptp_clock_info *clock, const struct timespec *ts)
{
	struct ptp_clk_options clk_opt;
	int output;
	struct Ksz_ptp_info *info =	container_of(clock, struct Ksz_ptp_info, caps);
	int err = 0;

	output = 0;
	clk_opt.sec = ts->tv_sec;
	clk_opt.nsec = ts->tv_nsec;
	err = proc_ptp_hw_access(info.ptp,DEV_CMD_PUT, DEV_PTP_CLK, output,&clk_opt, sizeof(clk_opt), NULL, &output,true);
	return err;
}

int ksz_ptp_enable(struct ptp_clock_info *clock,	struct ptp_clock_request *rq, int on)
{
	struct Ksz_ptp_info *info =	container_of(clock, struct Ksz_ptp_info, caps);
	struct Ksz_ptp_info *ptp = info.ptp;
	u32 bit;

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		if (rq->extts.index >= 2)
			return -EINVAL;
		bit = 1 << rq->extts.index;
		if (on)
			ptp->clock_events |= bit;
		else
			ptp->clock_events &= ~bit;
		return 0;

	case PTP_CLK_REQ_PPS:
		bit = 1 << 31;
		if (on)
			ptp->clock_events |= bit;
		else
			ptp->clock_events &= ~bit;
		return 0;

	default:
		break;
	}

	return -EOPNOTSUPP;
}

struct ptp_clock_info ptp_caps = {
	.owner		= THIS_MODULE,
	// Only 16 characters.
	.name		= "Microchip clock",
	.max_adj	= MAX_DRIFT_CORR,
	.n_alarm	= N_ALARM,
	.n_ext_ts	= N_EXT_TS,
	.n_per_out	= 0,
	.pps		= 1,
	.adjfreq	= ptp_adjfreq,
	.adjtime	= ptp_adjtime,
	.gettime	= ptp_gettime,
	.settime	= ptp_settime,
	.enable		= ptp_enable,
};

int micrel_ptp_get_ts_info(struct Ksz_ptp_info *ptp,
	struct ethtool_ts_info *info)
{
	struct Ksz_ptp_info *clock_info = ptp->clock_info;

	info.so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;
	if (clock_info.clock)
		info.phc_index = ptp_clock_index(clock_info.clock);
	else
		info.phc_index = -1;
	info.tx_types = (1 << HWTSTAMP_TX_OFF) | (1 << HWTSTAMP_TX_ON);
	info.rx_filters = (1 << HWTSTAMP_FILTER_NONE) |
		(1 << HWTSTAMP_FILTER_ALL);
	return 0;
}

int micrel_ptp_probe(struct Ksz_ptp_info *ptp)
{
	struct Ksz_ptp_info *info;
#if 0
	struct timespec now;
#endif
	int err = -ENOMEM;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		goto no_memory;

	err = -ENODEV;

	info.caps = ptp_caps;

#if 0
	getnstimeofday(&now);
	ptp_settime(&info.caps, &now);
#endif

	info.clock = ptp_clock_register(&info.caps, ptp->parent);
	if (IS_ERR(info.clock)) {
		err = PTR_ERR(info.clock);
		goto no_clock;
	}
	info.ptp = ptp;
	ptp->clock_info = info;

	return 0;

no_clock:
	kfree(info);
no_memory:
	return err;
}

int micrel_ptp_remove(struct Ksz_ptp_info *ptp)
{
	//struct Ksz_ptp_info *info = ptp->clock_info;

	//ptp_clock_unregister(info.clock);
	//kfree(info);
	//ptp->clock_info = NULL;

	return 0;
}
*/
//=== ksz-ptp.c
/**
 * Microchip PTP common code
 *
 * Copyright (c) 2015-2016 Microchip Technology Inc.
 *	Tristram Ha <Tristram.Ha@microchip.com>
 *
 * Copyright (c) 2009-2015 Micrel, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define CURRENT_UTC_OFFSET  36 /* 1 Jul 2015 */

#if 1
#define ENABLE_10_MHZ_CLK
#endif

#define FMT_NSEC_SIZE			12

char *format_nsec(char *str, u32 nsec)
{
	u32 nsec0;
	u32 nsec1;
	u32 nsec2;
	char str0[4];

	nsec0 = nsec % 1000;
	nsec1 = (nsec / 1000) % 1000;
	nsec2 = (nsec / 1000000) % 1000;
	sprintf(str0, "%03u", nsec0);
	if (nsec2)
		sprintf(str, "%3u.%03u.%s", nsec2, nsec1, str0);
	else if (nsec1)
		sprintf(str, "    %3u.%s", nsec1, str0);
	else
		sprintf(str, "        %3u", nsec0);
	return str;
}  /* format_nsec */

//#ifdef PTP_PROCESS
/*
struct pseudo_iphdr {
	__u8 ttl;
	__u8 protocol;
	__be16 tot_len;
	__be32 saddr;
	__be32 daddr;
};

u32 timestamp_val(u32 timestamp, u8 *sec)
{
	*sec = timestamp >> 30;
	timestamp <<= 2;
	timestamp >>= 2;
	return timestamp;
}

void calc_diff(struct ksz_ptp_time *prev, struct ksz_ptp_time *cur, struct ksz_ptp_time *result)
{
	struct ksz_ptp_time diff;
	int prev_nsec = prev->nsec;
	int cur_nsec = cur->nsec;

	if (prev->sec < 0)
		prev_nsec = -prev_nsec;
	if (cur->sec < 0)
		cur_nsec = -cur_nsec;
	diff.sec = cur->sec - prev->sec;
	diff.nsec = cur_nsec - prev_nsec;
	if (diff.nsec >= NANOSEC_IN_SEC) {
		diff.nsec -= NANOSEC_IN_SEC;
		diff.sec++;
	} else if (diff.nsec <= -NANOSEC_IN_SEC) {
		diff.nsec += NANOSEC_IN_SEC;
		diff.sec--;
	}
	if (diff.sec > 0 && diff.nsec < 0) {
		diff.nsec += NANOSEC_IN_SEC;
		diff.sec--;
	} else if (diff.sec < 0 && diff.nsec > 0) {
		diff.nsec -= NANOSEC_IN_SEC;
		diff.sec++;
	}
	if (diff.nsec < 0 && diff.sec < 0)
		diff.nsec = -diff.nsec;
	result->sec = diff.sec;
	result->nsec = diff.nsec;
}

void calc_udiff(struct ptp_utime *prev, struct ptp_utime *cur,
	struct ksz_ptp_time *result)
{
	struct ksz_ptp_time t1;
	struct ksz_ptp_time t2;

	if (prev->sec > (1UL << 31) || cur->sec > (1UL << 31)) {
		s64 t3;
		s64 t4;
		s64 diff;
		s32 rem;

		t3 = (u64) prev->sec * NANOSEC_IN_SEC + prev->nsec;
		t4 = (u64) cur->sec * NANOSEC_IN_SEC + cur->nsec;
		diff = t4 - t3;
		t3 = div_s64_rem(diff, NSEC_PER_SEC, &rem);
		result->sec = (s32) t3;
		result->nsec = rem;
		return;
	}
	t1.sec = prev->sec;
	t1.nsec = prev->nsec;
	t2.sec = cur->sec;
	t2.nsec = cur->nsec;
	calc_diff(&t1, &t2, result);
}


void calc_diff64(struct ptp_ltime *prev, struct ptp_ltime *cur,
	struct ptp_ltime *result)
{
	struct ptp_ltime diff;
	s64 prev_nsec = prev->nsec;
	s64 cur_nsec = cur->nsec;
	s64 scaled_nsec = (s64) NANOSEC_IN_SEC << SCALED_NANOSEC_S;

	if (prev->sec < 0)
		prev_nsec = -prev_nsec;
	if (cur->sec < 0)
		cur_nsec = -cur_nsec;
	diff.sec = cur->sec - prev->sec;
	diff.nsec = cur_nsec - prev_nsec;
	if (diff.nsec >= scaled_nsec) {
		diff.nsec -= scaled_nsec;
		diff.sec++;
	} else if (diff.nsec <= -scaled_nsec) {
		diff.nsec += scaled_nsec;
		diff.sec--;
	}
	if (diff.sec > 0 && diff.nsec < 0) {
		diff.nsec += scaled_nsec;
		diff.sec--;
	} else if (diff.sec < 0 && diff.nsec > 0) {
		diff.nsec -= scaled_nsec;
		diff.sec++;
	}
	if (diff.nsec < 0 && diff.sec < 0)
		diff.nsec = -diff.nsec;
	result->sec = diff.sec;
	result->nsec = diff.nsec;
}
#endif

void add_nsec(struct ptp_utime *t, u32 nsec)
{
	t->nsec += nsec;
	if (t->nsec >= NANOSEC_IN_SEC) {
		t->nsec -= NANOSEC_IN_SEC;
		t->sec++;
	}
}  /* add_nsec */

void sub_nsec(struct ptp_utime *t, u32 nsec)
{
	if (t->nsec < nsec) {
		t->nsec += NANOSEC_IN_SEC;
		t->sec--;
	}
	t->nsec -= nsec;
}  /* sub_nsec */

void update_ts(struct ptp_ts *ts, u32 cur_sec)
{
	int sec;
	u8 sec_chk;

	ts->t.nsec = timestamp_val(ts->timestamp, &sec_chk);
	if (ts->timestamp)
		sec = (cur_sec - sec_chk) & 3;
	else
		sec = 0;
	if (sec >= 3)
		sec -= 4;
	ts->t.sec = cur_sec - sec;
}  /* update_ts */

#define INIT_NSEC			40
#define MIN_CYCLE_NSEC			8
#define MIN_GAP_NSEC			120
#define PULSE_NSEC			8

int check_cascade(struct Ksz_ptp_info *ptp, int first, int total,
	u16 *repeat, u32 sec, u32 nsec)
{
	struct ptp_output *cur;
	struct ptp_output *next;
	struct ptp_output *prev;
	int diff;
	int i;
	int tso;
	int min_cnt;
	int cnt;

	tso = first;
	cur = &ptp->outputs[tso];
	next = &ptp->outputs[first + total];
	next->start = cur->start;
	add_nsec(&next->start, cur->iterate);
	for (i = 0; i < total; i++, tso++) {
		cur = &ptp->outputs[tso];
		cur->stop = cur->start;
		add_nsec(&cur->stop, cur->len);
		next = &ptp->outputs[tso + 1];
		calc_udiff(&cur->stop, &next->start, &cur->gap);
		if ((cur->gap.sec < 0 || (!cur->gap.sec && cur->gap.nsec < 0))
				&& (i < total - 1 || 1 != *repeat)) {
			dbg_msg("gap too small: %d=%d\n", i, cur->gap.nsec);
			return 1;
		}
	}
	if (1 == *repeat)
		goto check_cascade_done;

	min_cnt = *repeat;
	tso = first + 1;
	for (i = 1; i < total; i++, tso++) {
		cur = &ptp->outputs[tso];
		prev = &ptp->outputs[tso - 1];
		if (cur->iterate < prev->iterate) {
			diff = prev->iterate - cur->iterate;
			cnt = prev->gap.nsec / diff + 1;
		} else if (cur->iterate > prev->iterate) {
			diff = cur->iterate - prev->iterate;
			cnt = cur->gap.nsec / diff + 1;
		} else
			cnt = *repeat;
		if (min_cnt > cnt)
			min_cnt = cnt;
	}
	if (*repeat > min_cnt)
		*repeat = min_cnt;
	prev = &ptp->outputs[first + tso];
	for (cnt = 0; cnt < *repeat; cnt++) {
		tso = first;
		for (i = 0; i < total; i++, tso++) {
			cur = &ptp->outputs[tso];
			next = &ptp->outputs[tso + 1];
			dbg_msg("%d: %d:%9d %d %d:%9d %d: %d:%9d\n",
				i, cur->start.sec, cur->start.nsec, cur->len,
				cur->gap.sec, cur->gap.nsec, cur->iterate,
				cur->stop.sec, cur->stop.nsec);
			if (cur->stop.sec > next->start.sec ||
					(cur->stop.sec == next->start.sec &&
					cur->stop.nsec > next->stop.nsec))
				dbg_msg("> %d %d:%9d %d:%9d\n", i,
					cur->stop.sec, cur->stop.nsec,
					next->start.sec, next->start.nsec);
			add_nsec(&cur->start, cur->iterate);
			cur->stop = cur->start;
			add_nsec(&cur->stop, cur->len);
			if (!i)
				prev->start = cur->start;
		}
		dbg_msg("%d:%9d\n", prev->start.sec, prev->start.nsec);
	}

check_cascade_done:
	tso = first;
	cur = &ptp->outputs[tso];
	if (cur->trig.sec >= sec)
		return 0;

	for (i = 0; i < total; i++, tso++) {
		cur = &ptp->outputs[tso];
		cur->trig.sec += sec;
		add_nsec(&cur->trig, nsec);
	}
	return 0;
}

#define MAX_DRIFT_CORR			6250000
#define LOW_DRIFT_CORR			2499981
#define MAX_U32_SHIFT			32
#define MAX_DIVIDER_SHIFT		31

u32 drift_in_sec(u32 abs_offset, u64 interval64)
{
	u64 drift64;

	drift64 = abs_offset;
	drift64 *= NANOSEC_IN_SEC;
	drift64 = div64_u64(drift64, interval64);
	return (u32) drift64;
}

u32 clk_adjust_val(int diff, u32 interval)
{
	u32 adjust;
	u32 rem;
	u64 adjust64;

	if (0 == diff)
		return 0;
	if (diff < 0)
		adjust = -diff;
	else
		adjust = diff;

	/* 2^32 * adjust * 1000000000 / interval / 25000000 */
	if (interval != NANOSEC_IN_SEC)
		adjust = drift_in_sec(adjust, interval);

	if (adjust >= MAX_DRIFT_CORR)
		adjust = 0x3fffffff;
	else {
		adjust64 = 1LL << 32;
		adjust64 *= adjust;
		adjust64 = div_u64_rem(adjust64, 25000000, &rem);
		adjust = (u32) adjust64;
		if (adjust >= 0x40000000)
			adjust = 0x3fffffff;
	}
	if (diff < 0)
		adjust |= PTP_RATE_DIR << 16;
	return adjust;
}  /* clk_adjust_val */

void ptp_tso_off(struct Ksz_ptp_info *ptp, u8 tso, u16 tso_bit)
{
	ptp->reg->tx_off(ptp, tso);
	ptp->tso_intr &= ~tso_bit;
	ptp->tso_used &= ~tso_bit;
	ptp->tso_dev[tso] = NULL;
}  /* ptp_tso_off */

inline void ptp_tx_reset(struct Ksz_ptp_info *ptp, u16 tso_bit)
{
	ptp->reg->write(ptp, ADDR_16, TRIG_RESET, tso_bit);
}  /* ptp_tx_reset */

inline void ptp_gpo_reset(struct Ksz_ptp_info *ptp, int gpo, u16 tso_bit)
{
	ptp_tx_reset(ptp, tso_bit);
	ptp->cascade_gpo[gpo].tso &= ~tso_bit;
}  /* ptp_gpo_reset */

/* -------------------------------------------------------------------------- */
/*
void ptp_acquire(struct Ksz_ptp_info *ptp)
{
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);

	mutex_lock(g_sw.hwlock);
#ifdef PTP_SPI
	mutex_lock(g_sw.reglock);
#endif
}

void ptp_release(struct Ksz_ptp_info *ptp)
{
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);

#ifdef PTP_SPI
	mutex_unlock(g_sw.reglock);
#endif
	mutex_unlock(g_sw.hwlock);
}
*/


void get_ptp_time(struct Ksz_ptp_info *ptp, struct ptp_utime *t)
{
	u16 data;
	u8 subnsec;
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);

	data = g_sw.cached.ptp_clk_ctrl;
	data |= PTP_READ_RTC;
	ksz_ptp_write(ptp, ADDR_16, PTP_CLK_CTRL, data);
	t->sec = ksz_ptp_read(ptp, ADDR_32, PTP_RTC_SEC_L);
	t->nsec = ksz_ptp_read(ptp, ADDR_32, PTP_RTC_NANOSEC_L);
	subnsec = ksz_ptp_read(ptp, ADDR_8, PTP_RTC_SUB_NANOSEC);
	add_nsec(t, subnsec * 8);
}  /* get_ptp_time */

void set_ptp_time(struct Ksz_ptp_info *ptp, struct ptp_utime *t)
{
	u16 data;
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);

	data = g_sw.cached.ptp_clk_ctrl;
	ksz_ptp_write(ptp, ADDR_8, PTP_RTC_SUB_NANOSEC, 0);
	ksz_ptp_write(ptp, ADDR_32, PTP_RTC_SEC_L, t->sec);
	ksz_ptp_write(ptp, ADDR_32, PTP_RTC_NANOSEC_L, t->nsec);
	data |= PTP_LOAD_TIME;
	ksz_ptp_write(ptp, ADDR_16, PTP_CLK_CTRL, data);
}  /* set_ptp_time */

void adjust_ptp_time(struct Ksz_ptp_info *ptp, int add, u32 sec, u32 nsec,
	int adj_hack)
{
	u16 ctrl;
	u16 adj = 0;
	u32 val = nsec;
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);

	ctrl = g_sw.cached.ptp_clk_ctrl;
	if (add)
		ctrl |= PTP_STEP_DIR;
	else
		ctrl &= ~PTP_STEP_DIR;
	g_sw.cached.ptp_clk_ctrl = ctrl;
	if (adj_hack) {
		adj = ctrl;
		ctrl &= ~PTP_CLK_ADJ_ENABLE;
	}
	ctrl |= PTP_STEP_TIME;
	ksz_ptp_write(ptp, ADDR_32, PTP_RTC_SEC_L, sec);
	do {
		if (nsec > NANOSEC_IN_SEC - 1)
			nsec = NANOSEC_IN_SEC - 1;
		ksz_ptp_write(ptp, ADDR_32, PTP_RTC_NANOSEC_L, nsec);
		ksz_ptp_write(ptp, ADDR_16, PTP_CLK_CTRL, ctrl);
		val -= nsec;
		nsec = val;
	} while (val);
	if (adj_hack && (adj & PTP_CLK_ADJ_ENABLE))
		ksz_ptp_write(ptp, ADDR_16, PTP_CLK_CTRL, adj);
}  /* adjust_ptp_time */

void adjust_sync_time(struct Ksz_ptp_info *ptp, int diff, u32 interval,
	u32 duration)
{
	u32 adjust;

	adjust = clk_adjust_val(diff, interval);
	adjust |= PTP_TMP_RATE_ENABLE << 16;
	ksz_ptp_write(ptp, ADDR_32, PTP_RATE_DURATION_L, duration);
	ksz_ptp_write(ptp, ADDR_32, PTP_SUBNANOSEC_RATE_L, adjust);
}  /* adjust_sync_time */

inline void ptp_rx_reset(struct Ksz_ptp_info *ptp, u16 tsi_bit)
{
	ksz_ptp_write(ptp, ADDR_16, TS_RESET, tsi_bit);
}  /* ptp_rx_reset */

void ptp_rx_off(struct Ksz_ptp_info *ptp, u8 tsi)
{
	u16 ctrl;
	u16 tsi_bit = (1 << tsi);

	/* Disable previous timestamp interrupt. */
	if (ptp->ts_intr & tsi_bit) {
		ptp->ts_intr &= ~tsi_bit;
		ksz_ptp_write(ptp, ADDR_16, TS_INT_ENABLE, ptp->ts_intr);
	}

	/* Disable previous timestamp detection. */
	ctrl = ksz_ptp_read(ptp, ADDR_16, TS_ENABLE);
	if (ctrl & tsi_bit) {
		ctrl &= ~tsi_bit;
		ksz_ptp_write(ptp, ADDR_16, TS_ENABLE, ctrl);
	}

	/*
	 * Need to turn off cascade mode if it is used previously; otherwise,
	 * event counter keeps increasing.
	 */
	if (ptp->cascade_rx & tsi_bit) {
		ptp_rx_reset(ptp, tsi_bit);
		ptp->cascade_rx &= ~tsi_bit;
	}
}  /* ptp_rx_off */

inline void ptp_rx_intr(struct Ksz_ptp_info *ptp, u16 tsi_bit)
{
	ptp->ts_intr |= tsi_bit;
	ksz_ptp_write(ptp, ADDR_16, TS_INT_ENABLE, ptp->ts_intr);
}  /* ptp_rx_intr */

inline void ptp_rx_on(struct Ksz_ptp_info *ptp, u16 tsi_bit)
{
	u16 ctrl;

	ctrl = ksz_ptp_read(ptp, ADDR_16, TS_ENABLE);
	ctrl |= tsi_bit;
	ksz_ptp_write(ptp, ADDR_16, TS_ENABLE, ctrl);
}  /* ptp_rx_on */

void ptp_rx_restart(struct Ksz_ptp_info *ptp, u16 tsi_bit)
{
	u16 ctrl;

	ctrl = ksz_ptp_read(ptp, ADDR_16, TS_ENABLE);
	ctrl &= ~tsi_bit;
	ksz_ptp_write(ptp, ADDR_16, TS_ENABLE, ctrl);
	ctrl |= tsi_bit;
	ksz_ptp_write(ptp, ADDR_16, TS_ENABLE, ctrl);
}  /* ptp_rx_restart */

void ptp_rx_event(struct Ksz_ptp_info *ptp, u8 tsi, u8 gpi, u8 event,
	int intr)
{
	u32 reg;
	u16 ctrl;
	u16 tsi_bit = (1 << tsi);

	/* Config pattern. */
	reg = TSn_CONF(tsi);
	ctrl = event | ((gpi & 0xf) << 8);
	ksz_ptp_write(ptp, ADDR_16, reg, ctrl);

	/* Enable timestamp interrupt. */
	if (intr)
		ptp_rx_intr(ptp, tsi_bit);

	/* Enable timestamp detection. */
	ptp_rx_on(ptp, tsi_bit);
}  /* ptp_rx_event */

void ptp_rx_cascade_event(struct Ksz_ptp_info *ptp, u8 first, u8 total,
	u8 gpi, u8 event, int intr)
{
	int last;
	int tsi;
	u32 reg;
	u16 ctrl;
	u16 tail;
	int i;
	int prev;

	last = (first + total - 1) % MAX_TIMESTAMP_UNIT;
	tsi = last;
	tail = TS_CASCADE_TAIL;
	for (i = 1; i < total; i++) {
		reg = TSn_CONF(tsi);
		prev = tsi - 1;
		if (prev < 0)
			prev = MAX_TIMESTAMP_UNIT - 1;
		ctrl = event | ((gpi & 0xf) << 8);
		ctrl |= TS_CASCADE_EN | ((prev & 0xf) << 1);
		ctrl |= tail;
		ptp->cascade_rx |= (1 << tsi);
		ksz_ptp_write(ptp, ADDR_16, reg, ctrl);

		/* Enable timestamp interrupt. */
		if (intr)
			ptp->ts_intr |= (1 << tsi);
		--tsi;
		if (tsi < 0)
			tsi = MAX_TIMESTAMP_UNIT - 1;
		tail = 0;
	}
	reg = TSn_CONF(first);
	ctrl = event | ((gpi & 0xf) << 8);
	ctrl |= TS_CASCADE_EN | ((last & 0xf) << 1);
	ptp->cascade_rx |= (1 << first);
	ksz_ptp_write(ptp, ADDR_16, reg, ctrl);

	/* Enable timestamp interrupt. */
	if (intr)
		ptp_rx_intr(ptp, (1 << first));

	/* Enable timestamp detection. */
	ptp_rx_on(ptp, (1 << first));
}  /* ptp_rx_cascade_event */

void ksz_ptp_read_event(struct Ksz_ptp_info *ptp, u8 tsi)
{
	u32 reg;
	u16 ctrl;
	u16 tsi_bit = (1 << tsi);

	u32 reg_ns;
	u32 reg_s;
	u32 reg_sub;
	struct ptp_utime t;
	u16 sub;
	int max_ts;
	int num;
	int i;
	int edge;
	struct ptp_event *event = &ptp->events[tsi];
	int last = event->num;

	reg = TSn_EVENT_STATUS(tsi);
	ctrl = ksz_ptp_read(ptp, ADDR_16, reg);
	num = (ctrl & TS_NO_EVENT_DET_MASK) >> 1;
	max_ts = (num <= event->max) ? num : event->max;
	i = event->num;

	reg_ns = TSn_0_EVENT_NANOSEC_L(tsi) + 0x10 * i;
	reg_s = TSn_0_EVENT_SEC_L(tsi) + 0x10 * i;
	reg_sub = TSn_0_EVENT_SUB_NANOSEC(tsi) + 0x10 * i;
	for (; i < max_ts; i++) {
		t.nsec = ksz_ptp_read(ptp, ADDR_32, reg_ns);
		t.sec = ksz_ptp_read(ptp, ADDR_32, reg_s);
		sub = ksz_ptp_read(ptp, ADDR_16, reg_sub);
		edge = ((t.nsec >> 30) & 1);
		t.nsec <<= 2;
		t.nsec >>= 2;
		add_nsec(&t, sub * 8);
#if 1
/*
 * THa  2011/10/06
 * Unit sometimes detects rising edge when it is configured to detect falling
 * edge only.  This happens in the case of hooking up the output pin to an
 * input pin and using two units running opposite cycle in cascade mode.  The
 * 8 ns switch pulse before the cycle is too short to detect properly,
 * resulting in missing edges.
 * When detecting events directly from the output pin, the minimum pulse time
 * is 24 ns for proper detection without missing any edge.
 */
		if (event->event < 2 && edge != event->event)
			edge = event->event;
#endif
		event->edge |= edge << i;
		event->t[i] = t;
		reg_ns += 0x10;
		reg_s += 0x10;
		reg_sub += 0x10;
	}
	event->num = max_ts;

	/* Indicate there is new event. */
	if (event->num > last)
		ptp->ts_status |= tsi_bit;
}  /* ksz_ptp_read_event */

void ptp_tx_off(struct Ksz_ptp_info *ptp, u8 tso)
{
	u16 ctrl;
	u16 tso_bit = (1 << tso);

	/* Disable previous trigger out if not already completed. */
	ctrl = ksz_ptp_read(ptp, ADDR_16, TRIG_EN);
	if (ctrl & tso_bit) {
		ctrl &= ~tso_bit;
		ksz_ptp_write(ptp, ADDR_16, TRIG_EN, ctrl);
	}

	/*
	 * Using cascade mode previously need to reset the trigger output so
	 * that an errorneous output will not be generated during next
	 * cascade mode setup.
	 */
	if (ptp->cascade_tx & tso_bit) {
		ptp_gpo_reset(ptp, ptp->outputs[tso].gpo, tso_bit);
		ptp->cascade_tx &= ~tso_bit;
	} else {
		u32 reg = TRIGn_CONF_1(tso);

		ctrl = ksz_ptp_read(ptp, ADDR_16, reg);
		if (ctrl & TRIG_CASCADE_EN) {
			ctrl &= ~TRIG_CASCADE_EN;
			ctrl &= ~TRIG_CASCADE_TAIL;
			ctrl |= TRIG_CASCADE_UPS_MASK;
			ksz_ptp_write(ptp, ADDR_16, reg, ctrl);
		}
	}
}  /* ptp_tx_off */

void ptp_tx_on(struct Ksz_ptp_info *ptp, u8 tso)
{
	u16 ctrl;
	u16 tso_bit = (1 << tso);

	ctrl = ksz_ptp_read(ptp, ADDR_16, TRIG_EN);
	ctrl |= tso_bit;
	ksz_ptp_write(ptp, ADDR_16, TRIG_EN, ctrl);
}  /* ptp_tx_on */

void ptp_tx_trigger_time(struct Ksz_ptp_info *ptp, u8 tso, u32 sec, u32 nsec)
{
	u32 reg;

	reg = TRIGn_TARGET_SEC_L(tso);
	ksz_ptp_write(ptp, ADDR_32, reg, sec);
	reg = TRIGn_TARGET_NANOSEC_L(tso);
	ksz_ptp_write(ptp, ADDR_32, reg, nsec);
}  /* ptp_tx_trigger_time */

void ptp_tx_event(struct Ksz_ptp_info *ptp, u8 tso, u8 gpo, u8 event,
	u32 pulse, u32 cycle, u16 cnt, u32 sec, u32 nsec, u32 iterate,
	int intr, int now, int opt)
{
	u32 reg;
	u16 ctrl;
	u16 tso_bit = (1 << tso);
	struct ptp_output *cur = &ptp->outputs[tso];

	/* Hardware immediately keeps level high on new GPIO if not reset. */
	if (cur->level && gpo != cur->gpo)
		ptp_gpo_reset(ptp, cur->gpo, tso_bit);

	/* Config pattern. */
	reg = TRIGn_CONF_1(tso);
	ctrl = ((event & 0x7) << 4);
	ctrl |= (gpo & 0xf);
	if (intr)
		ctrl |= TRIG_NOTIFY;
	if (now)
		ctrl |= TRIG_NOW;
	if (opt)
		ctrl |= TRIG_CLK_OPT;
	ctrl |= TRIG_CASCADE_UPS_MASK;
	ksz_ptp_write(ptp, ADDR_16, reg, ctrl);

	/* Config pulse width. */
	if (TRIG_REG_OUTPUT == event) {
		reg = TRIGn_BIT_PATTERN(tso);
		ksz_ptp_write(ptp, ADDR_16, reg, (u16) pulse);
		cur->level = 0;
		if (cnt) {
			reg = cnt - 1;
			reg %= 16;
			while (reg) {
				pulse >>= 1;
				reg--;
			}
			if (pulse & 1)
				cur->level = 1;
		}
		pulse = 0;
	} else if (event >= TRIG_NEG_PULSE) {
		if (0 == pulse)
			pulse = 1;
		else if (tso != 11 && pulse > 0xffff)
			pulse = 0xffff;
		reg = TRIGn_PULSE_WIDTH(tso);
		ksz_ptp_write(ptp, ADDR_16, reg, (u16) pulse);
		if (11 == tso) {
			if (pulse > 0xffffff)
				pulse = 0xffffff;
			ctrl = ksz_ptp_read(ptp, ADDR_16, TRIG_PPS_WS);
			ctrl &= ~TRIG_PPS_WS_MASK;
			ctrl |= ((pulse >> 16) & TRIG_PPS_WS_MASK);
			ksz_ptp_write(ptp, ADDR_16, TRIG_PPS_WS, ctrl);
		}
	}

	/* Config cycle width. */
	if (event >= TRIG_NEG_PERIOD) {
		int min_cycle = pulse * PULSE_NSEC + MIN_CYCLE_NSEC;

		if (cycle < min_cycle)
			cycle = min_cycle;
		reg = TRIGn_CYCLE_WIDTH_L(tso);
		ksz_ptp_write(ptp, ADDR_32, reg, cycle);

		/* Config trigger count. */
		reg = TRIGn_PER_OCCUR(tso);
		ksz_ptp_write(ptp, ADDR_16, reg, cnt);
	}

	cur->len = 0;
	if (event >= TRIG_NEG_PERIOD) {
		if (cnt)
			cur->len += cycle * cnt;
		else
			cur->len += 0xF0000000;
	} else if (event >= TRIG_NEG_PULSE)
		cur->len += pulse * PULSE_NSEC;
	else
		cur->len += MIN_CYCLE_NSEC;

	cur->start.sec = sec;
	cur->start.nsec = nsec;
	cur->iterate = iterate;
	cur->trig = cur->start;
	cur->stop = cur->start;
	add_nsec(&cur->stop, cur->len);
	cur->gpo = gpo;

	switch (event) {
	case TRIG_POS_EDGE:
	case TRIG_NEG_PULSE:
	case TRIG_NEG_PERIOD:
		cur->level = 1;
		break;
	case TRIG_REG_OUTPUT:
		break;
	default:
		cur->level = 0;
		break;
	}

	if (ptp->cascade)
		return;

	/*
	 * Need to reset after completion.  Otherwise, this output pattern
	 * does not behave consistently in cascade mode.
	 */
	if (TRIG_NEG_EDGE == event)
		ptp->cascade_tx |= tso_bit;

	ptp->cascade_gpo[gpo].total = 0;
	if (cur->level)
		ptp->cascade_gpo[gpo].tso |= tso_bit;
	else
		ptp->cascade_gpo[gpo].tso &= ~tso_bit;

	/* Config trigger time. */
	ptp_tx_trigger_time(ptp, tso, sec, nsec);

	/* Enable trigger. */
	ptp_tx_on(ptp, tso);
}  /* ptp_tx_event */

void ptp_pps_event(struct Ksz_ptp_info *ptp, u8 gpo, u32 sec)
{
	u32 reg;
	u16 ctrl;
	u32 nsec;
	u32 pulse = (20000000 / 8);	/* 20 ms */
	u32 cycle = 1000000000;
	u16 cnt = 0;
	u8 tso = ptp->pps_tso;
	u8 event = TRIG_POS_PERIOD;

	ptp_tx_off(ptp, tso);

	/* Config pattern. */
	reg = TRIGn_CONF_1(tso);
	ctrl = ((event & 0x7) << 4);
	ctrl |= (gpo & 0xf);
	ctrl |= TRIG_NOTIFY;
	ctrl |= TRIG_NOW;
	ctrl |= TRIG_CASCADE_UPS_MASK;
	ksz_ptp_write(ptp, ADDR_16, reg, ctrl);

	/* Config pulse width. */
	reg = TRIGn_PULSE_WIDTH(tso);
	if (11 != tso && pulse > 0xffff)
		pulse = 0xffff;
	ksz_ptp_write(ptp, ADDR_16, reg, (u16) pulse);
	if (11 == tso) {
		if (pulse > 0xffffff)
			pulse = 0xffffff;
		ctrl = ksz_ptp_read(ptp, ADDR_16, TRIG_PPS_WS);
		ctrl &= ~TRIG_PPS_WS_MASK;
		ctrl |= ((pulse >> 16) & TRIG_PPS_WS_MASK);
		ksz_ptp_write(ptp, ADDR_16, TRIG_PPS_WS, ctrl);
	}

	/* Config cycle width. */
	reg = TRIGn_CYCLE_WIDTH_L(tso);
	ksz_ptp_write(ptp, ADDR_32, reg, cycle);

	/* Config trigger count. */
	reg = TRIGn_PER_OCCUR(tso);
	ksz_ptp_write(ptp, ADDR_16, reg, cnt);

	/* Config trigger time. */
	if (ptp->pps_offset >= 0)
		nsec = ptp->pps_offset;
	else {
		nsec = NANOSEC_IN_SEC + ptp->pps_offset;
		sec--;
	}
	ptp_tx_trigger_time(ptp, tso, sec, nsec);

	/* Enable trigger. */
	ptp_tx_on(ptp, tso);
}  /* ptp_pps_event */

void cfg_10MHz(struct Ksz_ptp_info *ptp, u8 tso, u8 gpo, u32 sec, u32 nsec)
{
	u32 reg;
	u16 ctrl;
	u32 pulse = 6;
	u32 cycle = 200;
	u16 cnt = 0;
	u8 event = TRIG_POS_PERIOD;

	/* Config pattern. */
	reg = TRIGn_CONF_1(tso);
	ctrl = ((event & 0x7) << 4);
	ctrl |= (gpo & 0xf);
	ctrl |= TRIG_NOTIFY;
	ctrl |= TRIG_CASCADE_UPS_MASK;
	if (1 == tso)
		ctrl |= TRIG_CLK_OPT;
	ksz_ptp_write(ptp, ADDR_16, reg, ctrl);

	/* Config pulse width. */
	reg = TRIGn_PULSE_WIDTH(tso);
	if (11 != tso && pulse > 0xffff)
		pulse = 0xffff;
	ksz_ptp_write(ptp, ADDR_16, reg, (u16) pulse);
	if (11 == tso) {
		if (pulse > 0xffffff)
			pulse = 0xffffff;
		ctrl = ksz_ptp_read(ptp, ADDR_16, TRIG_PPS_WS);
		ctrl &= ~TRIG_PPS_WS_MASK;
		ctrl |= ((pulse >> 16) & TRIG_PPS_WS_MASK);
		ksz_ptp_write(ptp, ADDR_16, TRIG_PPS_WS, ctrl);
	}

	/* Config cycle width. */
	reg = TRIGn_CYCLE_WIDTH_L(tso);
	ksz_ptp_write(ptp, ADDR_32, reg, cycle);

	/* Config trigger count. */
	reg = TRIGn_PER_OCCUR(tso);
	ksz_ptp_write(ptp, ADDR_16, reg, cnt);

	ptp_tx_trigger_time(ptp, tso, sec, nsec);
}  /* cfg_10MHz */

void ptp_10MHz(struct Ksz_ptp_info *ptp, u8 tso, u8 gpo, u32 sec)
{
	int i;
	u32 nsec;

	/* Config trigger time. */
	if (ptp->pps_offset >= 0)
		nsec = ptp->pps_offset;
	else {
		nsec = NANOSEC_IN_SEC + ptp->pps_offset;
		sec--;
	}
	for (i = 0; i < 2; i++) {
		ptp_tx_off(ptp, tso);

		cfg_10MHz(ptp, tso, gpo, sec, nsec);

		/* Enable trigger. */
		ptp_tx_on(ptp, tso);

		tso = 1;
		nsec += 12 * 8;
	}
}  /* ptp_10MHz */

void ptp_tx_cascade_on(struct Ksz_ptp_info *ptp, u8 tso, u8 first, u8 last,
	u16 repeat)
{
	u32 reg;
	u16 ctrl;
	int repeat_reg = 0;

	reg = TRIGn_CONF_1(tso);
	ctrl = ksz_ptp_read(ptp, ADDR_16, reg);
	ctrl |= TRIG_CASCADE_EN;
	ctrl &= ~TRIG_CASCADE_UPS_MASK;
	if (tso == first)
		ctrl |= ((last & 0xf) << 10);
	else
		ctrl |= (((tso - 1) & 0xf) << 10);
	if (repeat && tso == last) {
		ctrl |= TRIG_CASCADE_TAIL;
		if (((ctrl >> 4) & 0xf) != TRIG_REG_OUTPUT)
			repeat_reg = TRIGn_BIT_PATTERN(tso);
		else
			repeat_reg = TRIGn_PULSE_WIDTH(tso);
	}
	ksz_ptp_write(ptp, ADDR_16, reg, ctrl);
	if (repeat_reg)
		ksz_ptp_write(ptp, ADDR_16, repeat_reg, repeat - 1);
}  /* ptp_tx_cascade_on */

void ptp_tx_cascade_cycle(struct Ksz_ptp_info *ptp, u8 tso, u32 nsec)
{
	u32 reg;

	reg = TRIGn_ITERATE_TIME_L(tso);
	ksz_ptp_write(ptp, ADDR_32, reg, nsec);
}  /* ptp_tx_cascade_cycle */

int ptp_tx_cascade(struct Ksz_ptp_info *ptp, u8 first, u8 total,
	u16 repeat, u32 sec, u32 nsec, int intr)
{
	int i;
	u8 tso;
	u8 last;
	struct ptp_output *cur;

	last = first + total - 1;
	if (last >= MAX_TRIG_UNIT)
		return 1;
	if (check_cascade(ptp, first, total, &repeat, sec, nsec)) {
		dbg_msg("cascade repeat timing is not right\n");
		return 1;
	}
	tso = first;
	for (i = 0; i < total; i++, tso++) {
		cur = &ptp->outputs[tso];
		ptp_tx_trigger_time(ptp, tso, cur->trig.sec,
			cur->trig.nsec);
		ptp_tx_cascade_cycle(ptp, tso, cur->iterate);
		ptp_tx_cascade_on(ptp, tso, first, last, repeat);
		ptp->cascade_tx |= (1 << tso);
	}

	/* Do not reset last unit to keep level high. */
	if (ptp->outputs[last].level) {
		ptp->cascade_tx &= ~(1 << last);
		ptp->cascade_gpo[ptp->outputs[last].gpo].tso |= (1 << last);
	} else
		ptp->cascade_gpo[ptp->outputs[last].gpo].tso &= ~(1 << last);
	ptp_tx_on(ptp, first);
	return 0;
}  /* ptp_tx_cascade */

/* -------------------------------------------------------------------------- */

void set_ptp_domain(struct Ksz_ptp_info *ptp, u8 domain)
{
	u16 ctrl;

	ctrl = ptp->reg->read(ptp, ADDR_16, PTP_DOMAIN_VERSION) &
		~PTP_DOMAIN_MASK;
	ctrl |= domain;
	ptp->reg->write(ptp, ADDR_16, PTP_DOMAIN_VERSION, ctrl);
}  /* set_ptp_domain */

void set_ptp_mode(struct Ksz_ptp_info *ptp, u16 mode)
{
	u16 val;
	u16 sav;

	val = ptp->reg->read(ptp, ADDR_16, PTP_MSG_CONF1);
	sav = val;
	val &= ~(PTP_1STEP | PTP_TC_P2P | PTP_MASTER);
	val |= mode;
	if (val != sav)
		ptp->reg->write(ptp, ADDR_16, PTP_MSG_CONF1, val);
}  /* set_ptp_mode */

void synchronize_clk(struct Ksz_ptp_info *ptp)
{
	u32 sec;
	int inc;

	if (ptp->adjust_offset < 0 || ptp->adjust_sec < 0) {
		ptp->adjust_offset = -ptp->adjust_offset;
		ptp->adjust_sec = -ptp->adjust_sec;
		inc = false;
	} else
		inc = true;
	sec = (u32) ptp->adjust_sec;
	ptp->reg->adjust_time(ptp, inc, sec, ptp->adjust_offset,
		ptp->features & PTP_ADJ_HACK);
	ptp->offset_changed = ptp->adjust_offset;
	ptp->adjust_offset = 0;
	ptp->adjust_sec = 0;
}  /* synchronize_clk */

void set_ptp_adjust(struct Ksz_ptp_info *ptp, u32 adjust)
{
	ptp->reg->write(ptp, ADDR_32, PTP_SUBNANOSEC_RATE_L, adjust);
}  /* set_ptp_adjust */

inline void unsyntonize_clk(struct Ksz_ptp_info *ptp)
{
	u16 ctrl;
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);

	ctrl = g_sw.cached.ptp_clk_ctrl;
	ctrl &= ~PTP_CLK_ADJ_ENABLE;
	g_sw.cached.ptp_clk_ctrl = ctrl;
	ptp->reg->write(ptp, ADDR_16, PTP_CLK_CTRL, ctrl);
}  /* unsyntonize_clk */

void syntonize_clk(struct Ksz_ptp_info *ptp)
{
	u16 ctrl;
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);

	ctrl = g_sw.cached.ptp_clk_ctrl;
	ctrl |= PTP_CLK_ADJ_ENABLE;
	g_sw.cached.ptp_clk_ctrl = ctrl;
	ptp->reg->write(ptp, ADDR_16, PTP_CLK_CTRL, ctrl);
}  /* syntonize_clk */

u16 get_ptp_delay(struct Ksz_ptp_info *ptp, int port, u32 reg)
{
	reg += PTP_PORT_INTERVAL(port);
	return ptp->reg->read(ptp, ADDR_16, reg);
}  /* get_ptp_delay */

void set_ptp_delay(struct Ksz_ptp_info *ptp, int port, u32 reg, u16 nsec)
{
	reg += PTP_PORT_INTERVAL(port);
	ptp->reg->write(ptp, ADDR_16, reg, nsec);
}  /* set_ptp_delay */

u16 get_ptp_ingress(struct Ksz_ptp_info *ptp, int port)
{
	return get_ptp_delay(ptp, port, PTP_PORT1_RX_MAC2PHY_DELAY);
}

u16 get_ptp_egress(struct Ksz_ptp_info *ptp, int port)
{
	return get_ptp_delay(ptp, port, PTP_PORT1_TX_MAC2PHY_DELAY);
}

short get_ptp_asym(struct Ksz_ptp_info *ptp, int port)
{
	short val;

	val = get_ptp_delay(ptp, port, PTP_PORT1_ASYM_DELAY);
	if (val & 0x8000)
		val = -(val & ~0x8000);
	return val;
}

u16 get_ptp_link(struct Ksz_ptp_info *ptp, int port)
{
	return get_ptp_delay(ptp, port, PTP_PORT1_LINK_DELAY);
}

void set_ptp_ingress(struct Ksz_ptp_info *ptp, int port, u16 nsec)
{
	set_ptp_delay(ptp, port, PTP_PORT1_RX_MAC2PHY_DELAY, nsec);
}

void set_ptp_egress(struct Ksz_ptp_info *ptp, int port, u16 nsec)
{
	set_ptp_delay(ptp, port, PTP_PORT1_TX_MAC2PHY_DELAY, nsec);
}

void set_ptp_asym(struct Ksz_ptp_info *ptp, int port, short nsec)
{
	if (nsec < 0)
		nsec = -nsec | 0x8000;
	set_ptp_delay(ptp, port, PTP_PORT1_ASYM_DELAY, nsec);
}

void set_ptp_link(struct Ksz_ptp_info *ptp, int port, u16 nsec)
{
	set_ptp_delay(ptp, port, PTP_PORT1_LINK_DELAY, nsec);
}

inline void dbp_tx_ts(char *name, u8 port, u32 timestamp)
{
	u8 overflow;
	char ts[FMT_NSEC_SIZE];

	timestamp = timestamp_val(timestamp, &overflow);
	format_nsec(ts, timestamp);
	dbg_msg("%s p:%d c:%u %08x:%s\n", name, port, overflow, timestamp, ts);
}  /* dbp_tx_ts */

void ptp_setup_udp_msg(struct ptp_dev_info *info, u8 *data, int len,
	void (*func)(u8 *data, void *param), void *param)
{
	u8 buf[MAX_TSM_UDP_LEN];
	int in_intr = in_interrupt();

	if (len > MAX_TSM_UDP_LEN)
		len = MAX_TSM_UDP_LEN;
	if (!in_intr)
		mutex_lock(&info.lock);
	memcpy(buf, data, len);
	if (func)
		func(buf, param);
	len += 2;
	if (info.read_len + len <= info.read_max) {
		u16 *udp_len = (u16 *) &info.read_buf[info.read_len];

		*udp_len = len;
		udp_len++;
		memcpy(udp_len, buf, len - 2);
		info.read_len += len;
	}
	if (!in_intr)
		mutex_unlock(&info.lock);
	wake_up_interruptible(&info.wait_udp);
}  /* ptp_setup_udp_msg */

void ptp_tsm_resp(u8 *data, void *param)
{
	struct tsm_db *db = (struct tsm_db *) data;
	struct ptp_ts *ts = param;
	u32 timestamp;
	u8 sec_chk;

	db->cmd |= TSM_CMD_RESP;
	db->cur_sec = htonl(ts->t.sec);
	db->cur_nsec = htonl(ts->t.nsec);
	timestamp = timestamp_val(ts->timestamp, &sec_chk);
	db->timestamp = htonl(timestamp);
	db->cur_nsec = db->timestamp;
}  /* ptp_tsm_resp */

void ptp_tsm_get_time_resp(u8 *data, void *param)
{
	struct tsm_get_time *get = (struct tsm_get_time *) data;
	struct ptp_utime *t = param;

	get->cmd |= TSM_CMD_GET_TIME_RESP;
	get->sec = htonl(t->sec);
	get->nsec = htonl(t->nsec);
}  /* ptp_tsm_get_time_resp */

void add_tx_delay(struct ptp_ts *ts, int delay, u32 cur_sec)
{
	update_ts(ts, cur_sec);

	/*
	 * Save timestamp without transmit latency for PTP stack that adjusts
	 * transmit latency itself.
	 */
	ts->r = ts->t;
	add_nsec(&ts->t, delay);
	ts->timestamp = ts->t.nsec;
}  /* add_tx_delay */

void save_tx_ts(struct Ksz_ptp_info *ptp, struct ptp_tx_ts *tx,
	struct ptp_hw_ts *htx, int delay, int port)
{
	unsigned long diff = 0;

	add_tx_delay(&htx->ts, delay, ptp->cur_time.sec);
	if (ptp->overrides & PTP_CHECK_PATH_DELAY) {
		if (ptp->last_rx_ts.t.sec) {
			struct ksz_ptp_time diff;

			calc_udiff(&htx->ts.t, &ptp->last_rx_ts.t, &diff);
			dbg_msg("pd: %d\n", diff.nsec);
		} else
			ptp->last_tx_ts = htx->ts;
	}
	if (!htx->sim_2step) {
		struct tsm_db *db = (struct tsm_db *) tx->data.buf;
		u8 msg = tx->data.buf[0] & 3;

		tx->ts = htx->ts;
		tx->resp_time = jiffies;
		if (tx->req_time)
			diff = tx->resp_time - tx->req_time;
		if (diff < 4 * ptp->delay_ticks) {
			if (tx->missed) {
				if (diff > 2 * ptp->delay_ticks)
					dbg_msg("  caught: %d, %lu; %x=%04x\n",
						port, diff, msg,
						ntohs(db->seqid));
				if (tx->dev) {
					ptp_setup_udp_msg(tx->dev,
						tx->data.buf, tx->data.len,
						ptp_tsm_resp, &tx->ts);
					tx->dev = NULL;
				}

				/* Invalidate the timestamp. */
				tx->ts.timestamp = 0;
				tx->req_time = 0;
			}
		} else {
			dbg_msg("  new: %d, %lu; %x=%04x\n", port, diff,
				msg, ntohs(db->seqid));
		}
		tx->missed = false;
		if (tx->skb) {
			int len;
			u64 ns;
			struct skb_shared_hwtstamps shhwtstamps;

			if (ptp->tx_en & (1 << 8))
				ns = (u64) tx->ts.t.sec * NANOSEC_IN_SEC +
					tx->ts.t.nsec;
			else
				ns = (u64) tx->ts.r.sec * NANOSEC_IN_SEC +
					tx->ts.r.nsec;
			memset(&shhwtstamps, 0, sizeof(shhwtstamps));
			shhwtstamps.hwtstamp = ns_to_ktime(ns);

			/* Indicate which port message is sent out. */
			tx->msg->hdr.reserved2 = (1 << port);
			len = (unsigned char *) tx->msg - tx->skb->data;
			__skb_pull(tx->skb, len);
			skb_tstamp_tx(tx->skb, &shhwtstamps);

			/* buffer not released yet. */
			if (skb_shinfo(tx->skb)->tx_flags & SKBTX_HW_TSTAMP)
				skb_shinfo(tx->skb)->tx_flags &=
					~SKBTX_IN_PROGRESS;
			else
				dev_kfree_skb_irq(tx->skb);
			tx->skb = NULL;
		}
	}
	htx->sending = false;
}  /* save_tx_ts */

int get_tx_time(struct Ksz_ptp_info *ptp, u16 status)
{
	u32 reg = 0;
	int port;
	int delay;
	u32 xts;
	u32 *pts;
	struct ptp_tx_ts *tx = NULL;
	struct ptp_hw_ts *htx = NULL;

	while (status) {
		/* Do port 1 first. */
		if (status & (TS_PORT1_INT_XDELAY | TS_PORT1_INT_SYNC))
			port = 0;
		else if (status & (TS_PORT2_INT_XDELAY | TS_PORT2_INT_SYNC))
			port = 1;
		else
			break;
		xts = 0;
		pts = NULL;
		delay = ptp->tx_latency[port];
		if (status & TS_PORT1_INT_XDELAY) {
			reg = PTP_PORT1_XDELAY_TIMESTAMP_L;
			pts = &ptp->xdelay_ts[port];
			tx = &ptp->tx_dreq[port];
			htx = &ptp->hw_dreq[port];
			status &= ~TS_PORT1_INT_XDELAY;
		} else if (status & TS_PORT1_INT_SYNC) {
			reg = PTP_PORT1_SYNC_TIMESTAMP_L;
			tx = &ptp->tx_sync[port];
			htx = &ptp->hw_sync[port];
			status &= ~TS_PORT1_INT_SYNC;
		} else if (status & TS_PORT2_INT_XDELAY) {
			reg = PTP_PORT1_XDELAY_TIMESTAMP_L;
			pts = &ptp->xdelay_ts[port];
			tx = &ptp->tx_dreq[port];
			htx = &ptp->hw_dreq[port];
			status &= ~TS_PORT2_INT_XDELAY;
		} else if (status & TS_PORT2_INT_SYNC) {
			reg = PTP_PORT1_SYNC_TIMESTAMP_L;
			tx = &ptp->tx_sync[port];
			htx = &ptp->hw_sync[port];
			status &= ~TS_PORT2_INT_SYNC;
		}

		/* PDELAY_REQ and PDELAY_RESP share same interrupt. */
		if (pts) {
			reg += PTP_PORT_INTERVAL(port);
			xts = ptp->reg->read(ptp, ADDR_32, reg);

			if (xts != *pts) {
				*pts = xts;
				htx->ts.timestamp = xts;
				save_tx_ts(ptp, tx, htx, delay, port);
			}

			reg = PTP_PORT1_PDRESP_TIMESTAMP_L;
			pts = &ptp->pdresp_ts[port];
			tx = &ptp->tx_resp[port];
			htx = &ptp->hw_resp[port];

			reg += PTP_PORT_INTERVAL(port);
			xts = ptp->reg->read(ptp, ADDR_32, reg);
			if (xts != *pts) {
				delay = ptp->tx_latency[port];
				*pts = xts;
				htx->ts.timestamp = xts;
				save_tx_ts(ptp, tx, htx, delay, port);
			}
		} else {
			reg += PTP_PORT_INTERVAL(port);
			htx->ts.timestamp = ptp->reg->read(ptp, ADDR_32, reg);
			save_tx_ts(ptp, tx, htx, delay, port);
		}
	}
	if (!htx)
		return false;

	return true;
}  /* get_tx_time */

void ptp_update_sec(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct Ksz_ptp_info *ptp =
		container_of(dwork, struct Ksz_ptp_info, update_sec);

	if (ptp->update_sec_jiffies) {
		ptp->cur_time.sec++;
		schedule_delayed_work(&ptp->update_sec, 1000 * HZ / 1000);
	}
}  /* ptp_update_sec */

void generate_tx_event(struct Ksz_ptp_info *ptp, int gpo)
{
	struct ptp_utime t;

	ptp->first_sec = 0;
	ptp->intr_sec = 0;
	ptp->update_sec_jiffies = jiffies;
	ptp->reg->get_time(ptp, &t);
	t.sec += 1;
	if (t.nsec >= (NANOSEC_IN_SEC - ptp->delay_ticks * 50000000))
		t.sec += 1;
	ptp->reg->pps_event(ptp, gpo, t.sec);
#ifdef ENABLE_10_MHZ_CLK
	ptp->reg->ptp_10MHz(ptp, ptp->mhz_tso, ptp->mhz_gpo, t.sec);
#endif
	schedule_delayed_work(&ptp->update_sec, (1000000 - t.nsec / 1000) * HZ
		/ 1000000);
}  /* generate_tx_event */

void ptp_check_pps(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct Ksz_ptp_info *ptp =
		container_of(dwork, struct Ksz_ptp_info, check_pps);

	if (ptp->update_sec_jiffies) {
		ptp->ops->acquire(ptp);
		generate_tx_event(ptp, ptp->pps_gpo);
		ptp->ops->release(ptp);
	}
}  /* ptp_check_pps */

void prepare_gps(struct Ksz_ptp_info *ptp)
{
	ptp->ops->acquire(ptp);
	ptp->tsi_used |= (1 << ptp->gps_tsi);
	ptp->events[ptp->gps_tsi].event = 1;
	ptp->events[ptp->gps_tsi].timeout = 0;
	ptp->reg->rx_event(ptp, ptp->gps_tsi, ptp->gps_gpi, TS_DETECT_RISE,
		true);
	ptp->ops->release(ptp);
}  /* prepare_gps */

void prepare_pps(struct Ksz_ptp_info *ptp)
{
	ptp->ops->acquire(ptp);
	ptp->tso_used |= (1 << ptp->pps_tso);
#ifdef ENABLE_10_MHZ_CLK
	ptp->tso_used |= (1 << ptp->mhz_tso);
	ptp->tso_used |= (1 << 1);
#endif
	generate_tx_event(ptp, ptp->pps_gpo);
	ptp->tsi_used |= (1 << ptp->pps_tsi);
	ptp->events[ptp->pps_tsi].event = 1;
	ptp->reg->rx_event(ptp, ptp->pps_tsi, ptp->pps_gpo, TS_DETECT_RISE,
		true);
	ptp->ops->release(ptp);
}  /* prepare_pps */

/* -------------------------------------------------------------------------- */

void ptp_check(struct Ksz_ptp_info *ptp)
{
	struct ptp_utime cur;
	struct ptp_utime now;
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);

	ptp->features |= PTP_ADJ_HACK;
	ptp->ops->acquire(ptp);
	g_sw.cached.ptp_clk_ctrl = ptp->reg->read(ptp, ADDR_16, PTP_CLK_CTRL);
	ptp->reg->get_time(ptp, &cur);
	ptp->reg->adjust_time(ptp, true, 10, 0, true);
	ptp->reg->get_time(ptp, &now);
	if (now.sec - cur.sec >= 10) {
		ptp->features &= ~PTP_ADJ_HACK;
		ptp->features |= PTP_ADJ_SEC;
		ptp->features |= PTP_PDELAY_HACK;
		ptp->reg->adjust_time(ptp, false, 10, 0, true);
		ptp->version = 1;
	}
/*
 * THa  2013/01/08
 * The Rev. D chip has a problem of decrementing nanosecond that is bigger than
 * the current nanosecond when continual clock adjustment is enabled.  The
 * workaround is to use the PTP_ADJ_HACK code although the actual problem
 * avoided is now different.
 */
	if (!(ptp->features & PTP_ADJ_HACK)) {
		u16 data;

		data = g_sw.cached.ptp_clk_ctrl;
		g_sw.cached.ptp_clk_ctrl |= PTP_CLK_ADJ_ENABLE;
		ptp->reg->write(ptp, ADDR_16, PTP_CLK_CTRL,
			g_sw.cached.ptp_clk_ctrl);
		if (cur.sec < 1)
			cur.sec = 1;
		cur.nsec = 0;
		ptp->reg->set_time(ptp, &cur);
		ptp->reg->adjust_time(ptp, false, 0, 800000000, false);
		ptp->reg->get_time(ptp, &now);
		dbg_msg("%x:%u %x:%u\n", cur.sec, cur.nsec, now.sec, now.nsec);
		if (abs(now.sec - cur.sec) > 2) {
			ptp->reg->get_time(ptp, &now);
			dbg_msg("! %x:%u\n", now.sec, now.nsec);
			ptp->features |= PTP_ADJ_HACK;
			ptp->reg->write(ptp, ADDR_16, PTP_CLK_CTRL, data);

			ptp->reg->write(ptp, ADDR_16, PTP_CLK_CTRL,
				data | PTP_CLK_ADJ_ENABLE);
			ptp->reg->set_time(ptp, &cur);
			ptp->reg->adjust_time(ptp, false, 0, 800000000, true);
			ptp->reg->get_time(ptp, &now);
			dbg_msg("ok %x:%u\n", now.sec, now.nsec);
		}
		g_sw.cached.ptp_clk_ctrl = data;
		ptp->reg->write(ptp, ADDR_16, PTP_CLK_CTRL, data);
	}
	ptp->ops->release(ptp);
}  /* ptp_check */

void ptp_start(struct Ksz_ptp_info *ptp, int init)
{
	u16 ctrl;
	struct timespec ts;
	struct ptp_utime t;

	if (!ptp->features) {
		ptp_check(ptp);
		if (ptp->test_access_time)
			ptp->test_access_time(ptp);
	}
	ptp_acquire(ptp);
	ctrl = ksz_ptp_read(ptp, ADDR_16, PTP_MSG_CONF1);
	if (ctrl == ptp->mode) {
		ptp->cfg = ksz_ptp_read(ptp, ADDR_16, PTP_MSG_CONF2);
		ptp->domain = ksz_ptp_read(ptp, ADDR_16, PTP_DOMAIN_VERSION) &
			PTP_DOMAIN_MASK;
		if (!init) {
			ptp_release(ptp);
			return;
		}
	} else if (!init)
		ptp->mode = ctrl;
	if (ptp->mode != ptp->def_mode) {
		dbg_msg("mode changed: %04x %04x; %04x %04x\n",
			ptp->mode, ptp->def_mode, ptp->cfg, ptp->def_cfg);
		ptp->mode = ptp->def_mode;
		ptp->cfg = ptp->def_cfg;
		ptp->ptp_synt = false;
	}
	dbg_msg("ptp_start: %04x %04x\n",
		ptp->mode, ptp->cfg);
	ksz_ptp_write(ptp, ADDR_16, PTP_MSG_CONF1, ptp->mode);
	ksz_ptp_write(ptp, ADDR_16, PTP_MSG_CONF2, ptp->cfg);
	ksz_ptp_write(ptp, ADDR_16, TRIG_INT_ENABLE, ptp->trig_intr);
	ksz_ptp_write(ptp, ADDR_16, TS_INT_ENABLE, ptp->ts_intr);

	/* PTP stack is still running while device is reset. */
	if (ptp->drift_set) {
		ptp->drift = ptp->drift_set;
		ptp->adjust = clk_adjust_val(ptp->drift, NANOSEC_IN_SEC);
		set_ptp_adjust(ptp, ptp->adjust);
		syntonize_clk(ptp);
		ptp->ptp_synt = true;
	}

	ptp_release(ptp);

	ts = ktime_to_timespec(ktime_get_real());
	t.sec = ts.tv_sec;
	t.nsec = ts.tv_nsec;

	/* Adjust for leap seconds. */
	t.sec += ptp->utc_offset;
	ptp_acquire(ptp);
	set_ptp_time(ptp, &t);
	ptp->cur_time = t;
	ptp_release(ptp);

	prepare_pps(ptp);
	ptp->started = true;
}  /* ptp_start */

/* -------------------------------------------------------------------------- */

int ptp_poll_event(struct Ksz_ptp_info *ptp, u8 tsi)
{
	int max_ts;
	int num;
	u16 status;
	u16 tsi_bit = (1 << tsi);
	u32 reg = TSn_EVENT_STATUS(tsi);
	struct ptp_event *event = &ptp->events[tsi];

	status = ptp->reg->read(ptp, ADDR_16, reg);
	num = (status & TS_NO_EVENT_DET_MASK) >> 1;
	max_ts = (num <= event->max) ? num : event->max;
	if (max_ts > event->num) {
		ptp->ops->acquire(ptp);
		status = ptp->reg->read(ptp, ADDR_16, TS_INT_STATUS);
		if (status & tsi_bit)
			ptp->reg->write(ptp, ADDR_16, TS_INT_STATUS, tsi_bit);
		ptp->reg->read_event(ptp, tsi);
		ptp->ts_status = 0;
		ptp->ops->release(ptp);
		return true;
	}
	return false;
}  /* ptp_poll_event */

void convert_scaled_nsec(s64 scaled_nsec, int s, s64 *sec, int *nsec)
{
	int sign;
	u64 quot;
	u32 rem;

	/* Convert to positive number first. */
	if (scaled_nsec < 0) {
		sign = -1;
		scaled_nsec = -scaled_nsec;
	} else
		sign = 1;
	scaled_nsec >>= s;
	quot = div_u64_rem(scaled_nsec, NSEC_PER_SEC, &rem);
	*sec = quot;
	*nsec = (int) rem;

	/* Positive number means clock is faster. */
	if (1 == sign) {
		*sec = -*sec;
		*nsec = -*nsec;
	}
}  /* convert_scaled_nsec */

void adj_cur_time(struct Ksz_ptp_info *ptp)
{
	if (ptp->adjust_offset || ptp->adjust_sec) {
		synchronize_clk(ptp);
		if (ptp->sec_changed)
			generate_tx_event(ptp, ptp->pps_gpo);
		else {
			ptp->update_sec_jiffies = jiffies;
			schedule_delayed_work(&ptp->check_pps,
				1200 * HZ / 1000);
		}
	}
	if (ptp->sec_changed) {
		struct timespec ts;
		struct ptp_utime cur;

		ptp->reg->get_time(ptp, &ptp->cur_time);
		ts = ktime_to_timespec(ktime_get_real());
		cur.sec = ts.tv_sec;
		cur.nsec = ts.tv_nsec;
		calc_udiff(&ptp->cur_time, &cur, &ptp->time_diff);
		ptp->sec_changed = 0;
	}
}  /* adj_cur_time */

void set_before_adj(struct Ksz_ptp_info *ptp, struct ptp_utime *cur)
{
	ptp->adjust_offset += cur->nsec;
	ptp->adjust_offset += ptp->set_delay;
	ptp->adjust_offset += ptp->get_delay;
	cur->nsec = 0;
	if (ptp->adjust_offset > NANOSEC_IN_SEC) {
		ptp->adjust_offset -= NANOSEC_IN_SEC;
		cur->sec++;
	}
	ptp->reg->set_time(ptp, cur);
}   /* set_before_adj */

void set_cur_time(struct Ksz_ptp_info *ptp, struct ptp_ts *ts)
{
	struct ptp_utime cur;
	s64 diff_sec;
	int diff_nsec;

	ptp->adjust_offset = ts->t.nsec - ts->timestamp;
	ptp->reg->get_time(ptp, &cur);
	diff_nsec = ts->t.nsec - ts->timestamp;
	diff_sec = (s64) ts->t.sec - cur.sec;
	if (ptp->features & PTP_ADJ_SEC) {
		if (diff_sec) {
			s64 nsec;

			nsec = diff_sec;
			nsec *= NANOSEC_IN_SEC;
			nsec += diff_nsec;
			convert_scaled_nsec(-nsec, 0, &ptp->adjust_sec,
				&ptp->adjust_offset);
		} else {
			ptp->adjust_offset = diff_nsec;
			ptp->adjust_sec = 0;
		}
		ptp->sec_changed = ptp->adjust_sec;
	} else {
		if (abs(diff_sec) <= 1) {
			diff_nsec += diff_sec * NANOSEC_IN_SEC;
			if (abs(diff_nsec) < NANOSEC_IN_SEC) {
				ptp->adjust_offset = diff_nsec;
				diff_sec = 0;
			}
		}
		if (diff_sec) {
			cur.sec = ts->t.sec;
			set_before_adj(ptp, &cur);
			ptp->sec_changed = diff_sec;
		}
		ptp->adjust_sec = 0;
	}
	adj_cur_time(ptp);
}  /* set_cur_time */

void adj_clock(struct work_struct *work)
{
	struct Ksz_ptp_info *ptp = container_of(work, struct Ksz_ptp_info, adj_clk);
	struct ptp_utime cur;

	ptp->ops->acquire(ptp);

	ptp->sec_changed = ptp->adjust_sec;
	if (!(ptp->features & PTP_ADJ_SEC)) {

		/* Need to adjust second. */
		if (abs(ptp->adjust_sec) > 1) {
			ptp->reg->get_time(ptp, &cur);
			cur.sec += ptp->adjust_sec;
			set_before_adj(ptp, &cur);
		} else
			ptp->adjust_offset += ptp->adjust_sec * NANOSEC_IN_SEC;
		ptp->adjust_sec = 0;
	}
	adj_cur_time(ptp);
	ptp->ops->release(ptp);
}  /* adj_clock */

void execute(struct Ksz_ptp_info *ptp, struct work_struct *work)
{
#ifdef PTP_SPI
	queue_work(ptp->access, work);
#else
	work->func(work);
#endif
}  /* execute */

int ptp_stop(struct Ksz_ptp_info *ptp)
{
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);

#ifdef PTP_SPI
	flush_work(&ptp->adj_clk);
	flush_workqueue(ptp->access);
#endif
	cancel_delayed_work_sync(&ptp->check_pps);
	cancel_delayed_work_sync(&ptp->update_sec);
	ptp->update_sec_jiffies = 0;
	ptp->ops->acquire(ptp);
	ptp->reg->write(ptp, ADDR_16, REG_RESET_CTRL,
		PTP_SOFTWARE_RESET);
	udelay(1);
	ptp->reg->write(ptp, ADDR_16, REG_RESET_CTRL, 0);
	g_sw.cached.ptp_clk_ctrl = ptp->reg->read(ptp, ADDR_16, PTP_CLK_CTRL);
	ptp->ptp_synt = false;
	ptp->started = false;
	ptp->first_drift = ptp->drift = 0;
	ptp->reg->write(ptp, ADDR_16, TRIG_INT_ENABLE, 0);
	ptp->reg->write(ptp, ADDR_16, TS_INT_ENABLE, 0);
	ptp->ops->release(ptp);
	return false;
}  /* ptp_stop */

void init_tx_ts(struct ptp_tx_ts *ts)
{
	ts->ts.timestamp = 0;
	ts->req_time = 0;
	ts->resp_time = 0;
	ts->missed = false;
}  /* init_tx_ts */

struct ptp_dev_info *find_minor_dev(struct ptp_dev_info *info)
{
	struct Ksz_ptp_info *ptp = info.ptp;
	struct ptp_dev_info *dev;
	struct ptp_dev_info *prev;

	dev = ptp->dev[info.minor ^ 1];
	prev = ptp->dev[info.minor];
	while (prev != info && prev && dev) {
		prev = prev->next;
		dev = dev->next;
	}
	if (prev != info)
		dev = NULL;
	return dev;
}  /* find_minor_dev */

void ptp_init_state(struct Ksz_ptp_info *ptp)
{
	int port;
	u32 reg;
	struct ptp_utime t;

	mutex_lock(&ptp->lock);
	ptp->udp_head = ptp->udp_tail = 0;
	for (reg = 0; reg < MAX_TSM_UDP_CNT; reg++)
		ptp->udp[reg].len = 0;
	mutex_unlock(&ptp->lock);

	if (!ptp->started)
		return;
	ptp->reg->start(ptp, false);

	/* Stop automatic drift adjustment if PTP operation is started. */
	if (!ptp->first_drift)
		ptp->first_drift = 1;

	ptp->ops->acquire(ptp);
	for (port = 0; port < MAX_PTP_PORT; port++) {
		ptp->hw_sync[port].ts.timestamp = 0;
		ptp->hw_sync[port].sending = false;
		ptp->hw_dreq[port].ts.timestamp = 0;
		ptp->hw_dreq[port].sending = false;
		ptp->hw_resp[port].ts.timestamp = 0;
		ptp->hw_resp[port].sending = false;
		init_tx_ts(&ptp->tx_sync[port]);
		init_tx_ts(&ptp->tx_dreq[port]);
		init_tx_ts(&ptp->tx_resp[port]);
		reg = PTP_PORT1_XDELAY_TIMESTAMP_L + PTP_PORT_INTERVAL(port);
		ptp->xdelay_ts[port] = ptp->reg->read(ptp, ADDR_32, reg);
		reg = PTP_PORT1_PDRESP_TIMESTAMP_L + PTP_PORT_INTERVAL(port);
		ptp->pdresp_ts[port] = ptp->reg->read(ptp, ADDR_32, reg);
		ptp->rx_latency[port] = get_ptp_ingress(ptp, port);
		ptp->tx_latency[port] = get_ptp_egress(ptp, port);
		ptp->asym_delay[port] = get_ptp_asym(ptp, port);
		ptp->peer_delay[port] = get_ptp_link(ptp, port);
		set_ptp_link(ptp, port, 0);
		dbg_msg("%d = %d %d %d; %d\n", port,
			ptp->rx_latency[port],
			ptp->tx_latency[port],
			ptp->asym_delay[port],
			ptp->peer_delay[port]);
	}

	ptp->adjust_offset = 0;
	ptp->offset_changed = 0;
	memset(&ptp->last_rx_ts, 0, sizeof(struct ptp_ts));
	memset(&ptp->last_tx_ts, 0, sizeof(struct ptp_ts));

	if (!ptp->ptp_synt) {
		syntonize_clk(ptp);
		ptp->ptp_synt = true;
	}
	ptp->reg->get_time(ptp, &t);
	ptp->cur_time = t;
	ptp->ops->release(ptp);
}  /* ptp_init_state */

void ptp_exit_state(struct Ksz_ptp_info *ptp)
{
	if (ptp->mode & PTP_MASTER) {
		u16 data;

		ptp->ops->acquire(ptp);
		data = ptp->reg->read(ptp, ADDR_16, PTP_MSG_CONF1);
		data &= ~PTP_MASTER;
		ptp->reg->write(ptp, ADDR_16, PTP_MSG_CONF1, data);
		ptp->ops->release(ptp);
		ptp->mode &= ~PTP_MASTER;
		ptp->def_mode &= ~PTP_MASTER;
	}
	ptp->adjust_offset = 0;
	ptp->offset_changed = 0;

	/* Indicate drift is not being set by PTP stack. */
	ptp->drift_set = 0;
}  /* ptp_exit_state */

struct ptp_msg *check_ptp_msg(u8 *data, u16 **udp_check_ptr)
{
	struct ethhdr *eth = (struct ethhdr *) data;
	struct vlan_ethhdr *vlan = (struct vlan_ethhdr *) data;
	struct iphdr *iph = NULL;
	struct ipv6hdr *ip6h = NULL;
	struct udphdr *udp;
	int ipv6;
	struct ptp_msg *msg;

	if (eth->h_proto == htons(0x88F7)) {
		msg = (struct ptp_msg *)(eth + 1);
		goto check_ptp_version;
	}

	if (eth->h_proto == htons(ETH_P_8021Q)) {
		if (vlan->h_vlan_encapsulated_proto == htons(ETH_P_8021Q)) {
			unsigned char *ptr = (unsigned char *) vlan;

			ptr += VLAN_HLEN;
			vlan = (struct vlan_ethhdr *) ptr;
		}
		if (vlan->h_vlan_encapsulated_proto == htons(0x88F7)) {
			msg = (struct ptp_msg *)(vlan + 1);
			goto check_ptp_version;
		}
		ipv6 = vlan->h_vlan_encapsulated_proto == htons(ETH_P_IPV6);
		if (vlan->h_vlan_encapsulated_proto != htons(ETH_P_IP) &&
				!ipv6)
			return NULL;
		ip6h = (struct ipv6hdr *)(vlan + 1);
		iph = (struct iphdr *)(vlan + 1);
	} else {
		ipv6 = eth->h_proto == htons(ETH_P_IPV6);
		if (eth->h_proto != htons(ETH_P_IP) && !ipv6)
			return NULL;
		ip6h = (struct ipv6hdr *)(eth + 1);
		iph = (struct iphdr *)(eth + 1);
	}

	if (ipv6) {
		if (ip6h->nexthdr != IPPROTO_UDP)
			return NULL;

		udp = (struct udphdr *)(ip6h + 1);
		if (udp_check_ptr)
			*udp_check_ptr = &udp->check;
	} else {
		if (iph->protocol != IPPROTO_UDP)
			return NULL;
		if (ntohs(iph->frag_off) & IP_OFFSET)
			return NULL;

		udp = (struct udphdr *)(iph + 1);
		if (udp_check_ptr)
			*udp_check_ptr = &udp->check;
	}

	if (udp->dest != htons(319) && udp->dest != htons(320))
		return NULL;

	msg = (struct ptp_msg *)(udp + 1);

check_ptp_version:
	if (msg->hdr.versionPTP >= 2)
		return msg;
	return NULL;
}  /* check_ptp_msg */

struct ptp_msg *check_ptp_event(u8 *data)
{
	struct ptp_msg *msg;

	msg = check_ptp_msg(data, NULL);
	if (!msg)
		return NULL;
	switch (msg->hdr.messageType) {
	case SYNC_MSG:
		break;
	case DELAY_REQ_MSG:
		break;
	case PDELAY_REQ_MSG:
		break;
	case PDELAY_RESP_MSG:
		break;
	default:
		msg = NULL;
		break;
	}
	return msg;
}

int update_ptp_msg(u8 *data, u32 port, u32 overrides)
{
	struct ethhdr *eth = (struct ethhdr *) data;
	struct vlan_ethhdr *vlan = (struct vlan_ethhdr *) data;
	struct iphdr *iph;
	struct ipv6hdr *ip6h;
	struct ptp_msg *msg;
	u32 dst;
	u8 src;
	struct udphdr *udp = NULL;
	int ipv6 = 0;
	int udp_check = 0;

	do {
		if (eth->h_proto == htons(0x88F7)) {
			msg = (struct ptp_msg *)(eth + 1);
			break;
		}

		if (eth->h_proto == htons(ETH_P_8021Q)) {
			if (vlan->h_vlan_encapsulated_proto == htons(0x88F7)) {
				msg = (struct ptp_msg *)(vlan + 1);
				break;
			}
			ipv6 = vlan->h_vlan_encapsulated_proto ==
				htons(ETH_P_IPV6);
			if (vlan->h_vlan_encapsulated_proto != htons(ETH_P_IP)
					&& !ipv6)
				return false;
			ip6h = (struct ipv6hdr *)(vlan + 1);
			iph = (struct iphdr *)(vlan + 1);
		} else {
			ipv6 = eth->h_proto == htons(ETH_P_IPV6);
			if (eth->h_proto != htons(ETH_P_IP) && !ipv6)
				return false;
			ip6h = (struct ipv6hdr *)(eth + 1);
			iph = (struct iphdr *)(eth + 1);
		}

		if (ipv6) {
			if (ip6h->nexthdr != IPPROTO_UDP)
				return false;

			udp = (struct udphdr *)(ip6h + 1);
		} else {
			if (iph->protocol != IPPROTO_UDP)
				return false;

			udp = (struct udphdr *)(iph + 1);
		}

		if (udp->dest != htons(319) && udp->dest != htons(320))
			return false;
		msg = (struct ptp_msg *)(udp + 1);
	} while (0);
	if (msg->hdr.versionPTP < 2)
		return false;
	if ((overrides & PTP_VERIFY_TIMESTAMP) &&
			PDELAY_RESP_MSG == msg->hdr.messageType &&
			msg->hdr.flagField.flag.twoStepFlag) {
		struct ptp_utime rx;
		u32 timestamp;
		int i;
		u16 *data = (u16 *) &msg->hdr.reserved3;

		rx.nsec = ntohl(msg->data.pdelay_resp.requestReceiptTimestamp.
			nsec);
		rx.sec = ntohl(msg->data.pdelay_resp.requestReceiptTimestamp.
			sec.lo);
		timestamp = (rx.sec << 30) | rx.nsec;
		for (i = 0; i < 2; i++)
			udp_check += ntohs(data[i]);
		msg->hdr.reserved3 = htonl(timestamp);
		for (i = 0; i < 2; i++)
			udp_check -= ntohs(data[i]);
	}
	dst = port >> 16;
	port &= 0xffff;
	src = msg->hdr.reserved2;

	/* No destination port specified, use whatever port assigned. */
	if (!src)
		src = port;

	/* No change if all ports is specified, or all ports is assigned. */
	else if (!(overrides & PTP_KEEP_DST_PORT) &&
			src != 3 && port != 0 && port != 3)
		src = port;
	port = src;
	if (overrides & PTP_UPDATE_DST_PORT) {
		switch (msg->hdr.messageType) {
		case PDELAY_REQ_MSG:
		case PDELAY_RESP_MSG:
		case PDELAY_RESP_FOLLOW_UP_MSG:

			/* Ports are always open for Pdelay messages. */
			dst = 3;

			/* Force to all ports to send message. */
			if (!src)
				src = 3;
			port = src & dst;
			break;
		default:
			port = src & dst;

			/*
			 * Zero port still can send message through an open
			 * port, which is not what the application wants.
			 * Need to have the network driver not to send the
			 * packet to workaround the problem.
			 */
			if (!port && src && src != 3)
				return true;
			break;
		}
	}
	if (msg->hdr.reserved2 != port) {
		u8 data = msg->hdr.reserved2;

		udp_check += data;
		msg->hdr.reserved2 = (u8) port;
		udp_check -= port;
	}
	if ((overrides & PTP_ZERO_RESERVED_FIELD) && msg->hdr.reserved3 &&
			(PDELAY_RESP_MSG != msg->hdr.messageType ||
			msg->hdr.flagField.flag.twoStepFlag)) {
		int i;
		u16 *data = (u16 *) &msg->hdr.reserved3;

		for (i = 0; i < 2; i++)
			udp_check += ntohs(data[i]);
		msg->hdr.reserved3 = 0;
	}
	if (udp_check) {
		u16 check;

		/* Zero checksum in IPv4. */
		if (udp && !ipv6 && !udp->check)
			udp = NULL;
		if (udp) {
			check = ntohs(udp->check);
			udp_check += check;
			udp_check = (udp_check >> 16) + (udp_check & 0xffff);
			udp_check += (udp_check >> 16);
			check = (u16) udp_check;
			if (!check)
				check = -1;
			udp->check = htons(check);
		}
	}
	return false;
}  /* update_ptp_msg */

void get_rx_tstamp(void *ptr, struct sk_buff *skb)
{
	struct Ksz_ptp_info *ptp = ptr;
	struct ptp_msg *msg;
	u8 port;
	int delay;
	struct ptp_ts ts;
	u64 ns;
	struct skb_shared_hwtstamps *shhwtstamps = skb_hwtstamps(skb);

	if (!shhwtstamps)
		return;
	if (ptp->rx_msg_parsed)
		msg = ptp->rx_msg;
	else {
		msg = check_ptp_event(skb->data);
		ptp->rx_msg_parsed = true;
		ptp->rx_msg = msg;
	}
	if (!msg)
		return;

	ts.timestamp = ntohl(msg->hdr.reserved3);
	update_ts(&ts, ptp->cur_time.sec);
	if (ptp->rx_en & (1 << 8)) {
		port = msg->hdr.reserved2;
		if (port)
			port--;
		delay = ptp->rx_latency[port];
		sub_nsec(&ts.t, delay);
	}

	ns = (u64) ts.t.sec * NANOSEC_IN_SEC + ts.t.nsec;
	memset(shhwtstamps, 0, sizeof(*shhwtstamps));
	shhwtstamps->hwtstamp = ns_to_ktime(ns);
}  /* get_rx_tstamp */

void get_tx_tstamp(struct Ksz_ptp_info *ptp, struct sk_buff *skb)
{
	int cnt;
	int p;
	struct ptp_msg *msg;
	u8 port;
	struct ptp_tx_ts *tx;
	struct sk_buff *orig_skb = skb;

	msg = check_ptp_event(skb->data);
	if (!msg)
		return;
	port = msg->hdr.reserved2;
	if (!port)
		port = (1 << MAX_PTP_PORT) - 1;
	cnt = 0;
	if (SYNC_MSG == msg->hdr.messageType)
		tx = ptp->tx_sync;
	else if (PDELAY_RESP_MSG == msg->hdr.messageType)
		tx = ptp->tx_resp;
	else
		tx = ptp->tx_dreq;
	for (p = 0; p < MAX_PTP_PORT; p++, tx++) {
		if (!(port & (1 << p)))
			continue;
		if (tx->skb) {
			if (skb_shinfo(tx->skb)->tx_flags & SKBTX_HW_TSTAMP)
				skb_shinfo(tx->skb)->tx_flags &=
					~SKBTX_IN_PROGRESS;
			else {
				dev_kfree_skb_irq(tx->skb);
				tx->skb = NULL;
			}
		}

		/* Need to create socket buffer for more than 1 port. */
		if (cnt++) {
			skb = skb_copy(orig_skb, GFP_ATOMIC);
			if (!skb)
				break;
			skb->sk = orig_skb->sk;
			msg = check_ptp_event(skb->data);
		}
		tx->skb = skb;
		tx->msg = msg;
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
	}
}  /* get_tx_tstamp */

int ptp_hwtstamp_ioctl(struct Ksz_ptp_info *ptp, struct ifreq *ifr)
{
	struct hwtstamp_config config;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		ptp->tx_en &= ~1;
		break;
	case HWTSTAMP_TX_ONESTEP_SYNC:
	case HWTSTAMP_TX_ON:
		ptp->tx_en |= 1;
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		if ((ptp->rx_en & 1) && (ptp->rx_en & (1 << 8))) {
			ptp->tx_en &= ~(1 << 8);
			ptp->rx_en &= ~(1 << 8);
		}
		ptp->rx_en &= ~1;
		break;
	case HWTSTAMP_FILTER_ALL:
#if 0
		ptp->rx_en |= 1;
		break;
#endif
	default:
		if (!(ptp->rx_en & 1) && (ptp->tx_en & 1)) {
			ptp->tx_en |= (1 << 8);
			ptp->rx_en |= (1 << 8);
		}
		ptp->rx_en |= 1;
		break;
	}

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}

int ptp_drop_pkt(struct Ksz_ptp_info *ptp, struct sk_buff *skb, u32 vlan_id,
	int *tag, int *ptp_tag)
{
	ptp->rx_msg_parsed = false;
	ptp->rx_msg = NULL;
	do {
		u16 vid;
		u16 *protocol;

		if (!(ptp->vid))
			break;
		if (vlan_get_tag(skb, &vid))
			break;
		vid &= VLAN_VID_MASK;
		protocol = (u16 *) &skb->data[VLAN_ETH_HLEN - 2];

		if (!vid)
			break;
		if (*protocol == ntohs(0x88F7) && vid != ptp->vid)
			return true;
	} while (0);
	if ((ptp->overrides & PTP_PORT_FORWARD) || (ptp->rx_en & 1)) {
		struct ptp_msg *msg = check_ptp_msg(skb->data, NULL);

		ptp->rx_msg_parsed = true;
		ptp->rx_msg = msg;
		if (msg) {
			/* Indicate this is a PTP message. */
			*ptp_tag = msg->hdr.reserved2;

			if (ptp->overrides & PTP_CHECK_PATH_DELAY) {
				struct ptp_ts ts;

				ts.timestamp = ntohl(msg->hdr.reserved3);
				update_ts(&ts, ptp->cur_time.sec);
				if (ptp->last_tx_ts.t.sec) {
					struct ksz_ptp_time diff;

					calc_udiff(&ptp->last_tx_ts.t, &ts.t,
						&diff);
					dbg_msg("pd: %d\n", diff.nsec);
				} else
					ptp->last_rx_ts = ts;
			}

			/*
			 * Tag is already updated to reflect VLAN forwarding if
			 * tail tagging is used.  This is a case of it not
			 * being used.
			 */
			if (!*tag) {
				*tag = *ptp_tag;
				if (!(vlan_id & (1 << *tag)))
					*tag = 0;
			}
			if (ptp->rx_en & 1)
				ptp->ops->get_rx_tstamp(ptp, skb);
		}
	}
	return false;
}  /* ptp_drop_pkt */

void proc_ptp_get_cfg(struct Ksz_ptp_info *ptp, u8 *data)
{
	struct ptp_cfg_options *cmd = (struct ptp_cfg_options *) data;

	ptp->ops->acquire(ptp);
	ptp->mode = ptp->reg->read(ptp, ADDR_16, PTP_MSG_CONF1);
	ptp->cfg = ptp->reg->read(ptp, ADDR_16, PTP_MSG_CONF2);
	ptp->domain = ptp->reg->read(ptp, ADDR_16, PTP_DOMAIN_VERSION) &
		PTP_DOMAIN_MASK;
	ptp->ops->release(ptp);
	if (ptp->mode != ptp->def_mode && ptp->started) {
		dbg_msg("mode mismatched: %04x %04x; %04x %04x\n",
			ptp->mode, ptp->def_mode, ptp->cfg, ptp->def_cfg);
		ptp->mode = ptp->def_mode;
		ptp->cfg = ptp->def_cfg;
		ptp->reg->start(ptp, false);
	}
	cmd->two_step = (ptp->mode & PTP_1STEP) ? 0 : 1;
	cmd->master = (ptp->mode & PTP_MASTER) ? 1 : 0;
	cmd->p2p = (ptp->mode & PTP_TC_P2P) ? 1 : 0;
	cmd->as = (ptp->mode & PTP_FORWARD_TO_PORT3) ? 1 : 0;
	cmd->unicast = (ptp->cfg & PTP_UNICAST_ENABLE) ? 1 : 0;
	cmd->alternate = (ptp->cfg & PTP_ALTERNATE_MASTER) ? 1 : 0;
	cmd->domain_check = (ptp->cfg & PTP_DOMAIN_CHECK) ? 1 : 0;
	cmd->udp_csum = (ptp->cfg & PTP_UDP_CHECKSUM) ? 1 : 0;
	cmd->delay_assoc = (ptp->cfg & PTP_DELAY_CHECK) ? 1 : 0;
	cmd->pdelay_assoc = (ptp->cfg & PTP_PDELAY_CHECK) ? 1 : 0;
	cmd->sync_assoc = (ptp->cfg & PTP_SYNC_CHECK) ? 1 : 0;
	cmd->drop_sync = (ptp->cfg & PTP_DROP_SYNC_DELAY_REQ) ? 1 : 0;
	cmd->priority = (ptp->cfg & PTP_ALL_HIGH_PRIORITY) ? 1 : 0;
	cmd->reserved = ptp->started;
	cmd->domain = ptp->domain;
	cmd->access_delay = ptp->get_delay;
}  /* proc_ptp_get_cfg */

int proc_ptp_set_cfg(struct Ksz_ptp_info *ptp, u8 *data)
{
	struct ptp_cfg_options *cmd = (struct ptp_cfg_options *) data;
	u16 cfg;
	u16 mode;
	u8 domain;

	mode = ptp->mode;
	cfg = ptp->cfg;
	domain = ptp->domain;
	if (cmd->domain_set) {
		domain = cmd->domain;
	} else {
		if (cmd->two_step_set) {
			if (cmd->two_step)
				ptp->mode &= ~PTP_1STEP;
			else
				ptp->mode |= PTP_1STEP;
		}
		if (cmd->master_set) {
			if (cmd->master)
				ptp->mode |= PTP_MASTER;
			else
				ptp->mode &= ~PTP_MASTER;
		}
		if (cmd->p2p_set) {
			if (cmd->p2p)
				ptp->mode |= PTP_TC_P2P;
			else
				ptp->mode &= ~PTP_TC_P2P;
		}
		if (cmd->as_set) {
			if (cmd->as)
				ptp->mode |= PTP_FORWARD_TO_PORT3;
			else
				ptp->mode &= ~PTP_FORWARD_TO_PORT3;
		}
		if (cmd->unicast_set) {
			if (cmd->unicast)
				ptp->cfg |= PTP_UNICAST_ENABLE;
			else
				ptp->cfg &= ~PTP_UNICAST_ENABLE;
		}
		if (cmd->alternate_set) {
			if (cmd->alternate)
				ptp->cfg |= PTP_ALTERNATE_MASTER;
			else
				ptp->cfg &= ~PTP_ALTERNATE_MASTER;
		}
		if (cmd->domain_check_set) {
			if (cmd->domain_check)
				ptp->cfg |= PTP_DOMAIN_CHECK;
			else
				ptp->cfg &= ~PTP_DOMAIN_CHECK;
		}
		if (cmd->udp_csum_set) {
			if (cmd->udp_csum)
				ptp->cfg |= PTP_UDP_CHECKSUM;
			else
				ptp->cfg &= ~PTP_UDP_CHECKSUM;
		}
		if (cmd->delay_assoc_set) {
			if (cmd->delay_assoc)
				ptp->cfg |= PTP_DELAY_CHECK;
			else
				ptp->cfg &= ~PTP_DELAY_CHECK;
		}
		if (cmd->pdelay_assoc_set) {
			if (cmd->pdelay_assoc)
				ptp->cfg |= PTP_PDELAY_CHECK;
			else
				ptp->cfg &= ~PTP_PDELAY_CHECK;
		}
		if (cmd->sync_assoc_set) {
			if (cmd->sync_assoc)
				ptp->cfg |= PTP_SYNC_CHECK;
			else
				ptp->cfg &= ~PTP_SYNC_CHECK;
		}
		if (cmd->drop_sync_set) {
			if (cmd->drop_sync)
				ptp->cfg |= PTP_DROP_SYNC_DELAY_REQ;
			else
				ptp->cfg &= ~PTP_DROP_SYNC_DELAY_REQ;
		}
		if (cmd->priority_set) {
			if (cmd->priority)
				ptp->cfg |= PTP_ALL_HIGH_PRIORITY;
			else
				ptp->cfg &= ~PTP_ALL_HIGH_PRIORITY;
		}
	}
	ptp->ops->acquire(ptp);
	if (mode != ptp->mode) {
		u16 ts_intr = ptp->ts_intr;

		if (ptp->mode & PTP_1STEP)
			ptp->ts_intr &= ~(TS_PORT2_INT_SYNC |
				TS_PORT1_INT_SYNC);
		else
			ptp->ts_intr |= (TS_PORT2_INT_SYNC |
				TS_PORT1_INT_SYNC);
		dbg_msg("mode: %x %x\n", mode, ptp->mode);
		mode = ptp->mode;
		if (ptp->overrides & PTP_VERIFY_TIMESTAMP)
			mode |= PTP_1STEP;
		ptp->reg->write(ptp, ADDR_16, PTP_MSG_CONF1, mode);
		ptp->def_mode = ptp->mode;
		if (ts_intr != ptp->ts_intr)
			ptp->reg->write(ptp, ADDR_16, TS_INT_ENABLE,
				ptp->ts_intr);
	}
	if (cfg != ptp->cfg) {
		dbg_msg("cfg: %x %x\n", cfg, ptp->cfg);
		ptp->reg->write(ptp, ADDR_16, PTP_MSG_CONF2, ptp->cfg);
	}
	if (domain != ptp->domain) {
		ptp->domain = domain;
		set_ptp_domain(ptp, ptp->domain);
	}
	ptp->ops->release(ptp);
	return 0;
}  /* proc_ptp_set_cfg */

void cancel_rx_unit(struct Ksz_ptp_info *ptp, int tsi)
{
	int first;
	int last;
	int tsi_bit;
	struct ptp_event *events;

	ptp->ops->acquire(ptp);
	first = tsi;
	events = &ptp->events[tsi];
	if (events->last) {
		first = events->first;
		last = events->last;
	} else
		last = first + 1;
	tsi = first;
	ptp->events[tsi].timeout = 0;
	do {
		if (tsi >= MAX_TIMESTAMP_UNIT)
			tsi = 0;
		events = &ptp->events[tsi];
		events->first = 0;
		events->last = 0;
		tsi_bit = 1 << tsi;
		if (ptp->tsi_used & tsi_bit) {
			if (events->num < events->max) {
				ptp->reg->read_event(ptp, tsi);
				ptp->ts_status = 0;
			}
			ptp->reg->rx_off(ptp, tsi);
			ptp->tsi_intr &= ~tsi_bit;
			ptp->tsi_used &= ~tsi_bit;
			ptp->tsi_dev[tsi] = NULL;
		}
		++tsi;
	} while (tsi != last);
	ptp->ops->release(ptp);
}  /* cancel_rx_unit */

int check_expired_rx_unit(struct Ksz_ptp_info *ptp, int tsi)
{
	int first;
	int last;
	u32 expired;
	struct ptp_event *events;
	struct ksz_ptp_time diff;
	struct ptp_utime t;

	events = &ptp->events[tsi];
	first = tsi;
	if (events->last) {
		first = events->first;
		last = events->last;
	} else
		last = first + 1;
	events = &ptp->events[first];
	if (events->num && events->timeout) {
		ptp->ops->acquire(ptp);
		ptp->reg->get_time(ptp, &t);
		ptp->ops->release(ptp);
		calc_udiff(events->t, &t, &diff);
		if (diff.sec >= 0 && diff.nsec >= 0) {
			expired = diff.sec * 1000 + diff.nsec / 1000000;
			expired = expired * HZ / 1000;
			if (expired > events->timeout) {
				cancel_rx_unit(ptp, first);
				return 1;
			}
		}
	}
	return 0;
}  /* check_expired_rx_unit */

int proc_dev_rx_event(struct ptp_dev_info *info, u8 *data)
{
	struct Ksz_ptp_info *ptp = info.ptp;
	struct ptp_tsi_options *cmd = (struct ptp_tsi_options *) data;
	u8 event;
	int first;
	int i;
	int intr;
	int tsi;
	int avail;
	int total;
	int last;
	int tsi_bit;
	struct ptp_event *events;

	tsi = cmd->tsi;
	total = cmd->total;
	if (!total)
		total = 1;
	first = tsi;

	/* Cancel operation. */
	if ((cmd->flags & PTP_CMD_CANCEL_OPER)) {
		if (tsi >= MAX_TIMESTAMP_UNIT)
			return DEV_IOC_UNIT_UNAVAILABLE;
		cancel_rx_unit(ptp, tsi);
		goto proc_ptp_rx_cascade_event_done;
	}
	if (tsi >= MAX_TIMESTAMP_UNIT) {
		first = 0;
		do {
			for (tsi = first; tsi < MAX_TIMESTAMP_UNIT; tsi++)
				if (!(ptp->tsi_used & (1 << tsi)) &&
						!ptp->events[tsi].last)
					break;
			if (tsi >= MAX_TIMESTAMP_UNIT)
				return DEV_IOC_UNIT_UNAVAILABLE;
			first = tsi;
			avail = 1;
			for (i = 1; i < total; i++)
				if (!(ptp->tsi_used & (1 << tsi)) &&
						!ptp->events[tsi].last) {
					++avail;
					++tsi;
					if (tsi >= MAX_TIMESTAMP_UNIT)
						tsi = 0;
				} else {
					++first;
					break;
				}
		} while (avail < total);
	} else {
		for (i = 0; i < total; i++) {
			if (ptp->tsi_used & (1 << tsi) ||
					ptp->events[tsi].last)
				if (!check_expired_rx_unit(ptp, tsi))
					return DEV_IOC_UNIT_USED;
			++tsi;
			if (tsi >= MAX_TIMESTAMP_UNIT)
				tsi = 0;
		}
	}
	if (cmd->gpi >= MAX_GPIO)
		return -EINVAL;
	if (0 == cmd->event)
		event = TS_DETECT_FALL;
	else if (1 == cmd->event)
		event = TS_DETECT_RISE;
	else {
		event = TS_DETECT_RISE | TS_DETECT_FALL;
		cmd->event = 2;
	}
	tsi = first;
	last = first + total;
	if (last > MAX_TIMESTAMP_UNIT)
		last -= MAX_TIMESTAMP_UNIT;
	intr = cmd->flags & PTP_CMD_INTR_OPER;
	ptp->ops->acquire(ptp);
	for (i = 0; i < total; i++) {
		tsi_bit = 1 << tsi;
		ptp->tsi_used |= tsi_bit;
		if (intr & !(cmd->flags & PTP_CMD_SILENT_OPER)) {
			ptp->tsi_intr |= tsi_bit;
			ptp->tsi_dev[tsi] = info;
		}
		events = &ptp->events[tsi];
		events->num = 0;
		events->event = cmd->event;
		events->edge = 0;
		events->expired = 0;
		if (total > 1) {
			events->first = first;
			events->last = last;
		}
		++tsi;
		if (tsi >= MAX_TIMESTAMP_UNIT)
			tsi = 0;
	}
	tsi = first;
	ptp->events[tsi].timeout = cmd->timeout * HZ / 1000;

	/* Zero timeout means repeatable. */
	if (!ptp->events[tsi].timeout && cmd->timeout)
		ptp->events[tsi].timeout = 1;
	if (total > 1)
		ptp->reg->rx_cascade_event(ptp, tsi, total, cmd->gpi, event,
			intr);
	else
		ptp->reg->rx_event(ptp, tsi, cmd->gpi, event, intr);
	ptp->ops->release(ptp);

proc_ptp_rx_cascade_event_done:
	*data = tsi;
	return 0;
}  /* proc_dev_rx_event */

int find_avail_tx_unit(struct Ksz_ptp_info *ptp, int total, int *unit)
{
	int avail;
	int first;
	int i;
	int tso;

	first = 0;
	do {
		for (tso = first; tso < MAX_TRIG_UNIT; tso++)
			if (!(ptp->tso_used & (1 << tso)))
				break;
		if (tso >= MAX_TRIG_UNIT)
			return DEV_IOC_UNIT_UNAVAILABLE;
		first = tso++;
		avail = 1;
		for (i = 1; i < total; i++) {
			if (tso >= MAX_TRIG_UNIT)
				return DEV_IOC_UNIT_UNAVAILABLE;
			if (!(ptp->tso_used & (1 << tso))) {
				++avail;
				++tso;
			} else {
				++first;
				break;
			}
		}
	} while (avail < total);
	*unit = first;
	return 0;
}  /* find_avail_tx_unit */

int proc_dev_tx_event(struct ptp_dev_info *info, u8 *data)
{
	struct Ksz_ptp_info *ptp = info.ptp;
	struct ptp_tso_options *cmd = (struct ptp_tso_options *) data;
	int gpo;
	int intr;
	int tso;
	int tso_bit;
	struct ptp_utime t;
	u16 active;
	u16 status;
	int err = 0;

	gpo = cmd->gpo;
	if (gpo >= MAX_GPIO)
		return -EINVAL;
	if (cmd->event > TRIG_REG_OUTPUT)
		return -EINVAL;
	tso = cmd->tso;
	tso_bit = 1 << tso;

	/* Cancel operation. */
	if ((cmd->flags & PTP_CMD_CANCEL_OPER)) {
		if (tso >= MAX_TRIG_UNIT)
			return DEV_IOC_UNIT_UNAVAILABLE;
		ptp->ops->acquire(ptp);

		/* Reset the tso. */
		ptp->cascade_tx |= tso_bit;
		ptp_tso_off(ptp, tso, tso_bit);
		ptp->ops->release(ptp);
		goto proc_dev_tx_event_done;
	}
	if (ptp->cascade && (tso < ptp->cascade_gpo[gpo].first ||
			tso >= ptp->cascade_gpo[gpo].first +
			ptp->cascade_gpo[gpo].total))
		return DEV_IOC_UNIT_UNAVAILABLE;

	/* Find available unit for use. */
	if (tso >= MAX_TRIG_UNIT) {
		int rc = find_avail_tx_unit(ptp, 1, &tso);

		if (rc)
			return rc;
	} else if (!ptp->cascade && (ptp->tso_used & tso_bit)) {
		ptp->ops->acquire(ptp);

		/* See whether previous operation is completed. */
		active = ptp->reg->read(ptp, ADDR_16, TRIG_ACTIVE);
		if (active & tso_bit) {
			status = ptp->reg->read(ptp, ADDR_16, TRIG_ERROR);
			if (!(status & tso_bit)) {
				ptp->ops->release(ptp);
				return DEV_IOC_UNIT_USED;
			}
			dbg_msg("trig err: %d\n", tso);
		}
		if (!(active & tso_bit)) {
			status = ptp->reg->read(ptp, ADDR_16, TRIG_DONE);
			if (!(status & tso_bit)) {
				/* Reset the unit. */
				ptp->cascade_tx |= tso_bit;
				dbg_msg(" !? trig done: %d\n", tso);
			}
		}
		ptp_tso_off(ptp, tso, tso_bit);
		ptp->ops->release(ptp);
	}
	ptp->ops->acquire(ptp);
	if (!ptp->cascade && (cmd->flags & PTP_CMD_REL_TIME) &&
			cmd->sec < 100) {
		ptp->reg->get_time(ptp, &t);
		if (0 == cmd->sec) {
			cmd->nsec += t.nsec;
			cmd->nsec += 500;
			cmd->nsec /= 1000;
			cmd->nsec *= 1000;
			if (cmd->nsec >= NANOSEC_IN_SEC) {
				cmd->nsec -= NANOSEC_IN_SEC;
				cmd->sec++;
			}
		}
		cmd->sec += t.sec;
	}
	intr = cmd->flags & PTP_CMD_INTR_OPER;
	if (intr & !(cmd->flags & PTP_CMD_SILENT_OPER)) {
		ptp->tso_intr |= tso_bit;
		ptp->tso_dev[tso] = info;
	}
	ptp->tso_used |= tso_bit;
	ptp->reg->tx_event(ptp, tso, cmd->gpo, cmd->event, cmd->pulse,
		cmd->cycle, cmd->cnt, cmd->sec, cmd->nsec, cmd->iterate, intr,
		!(cmd->flags & PTP_CMD_ON_TIME),
		(cmd->flags & PTP_CMD_CLK_OPT));
	if (cmd->flags & PTP_CMD_ON_TIME) {
		status = ptp->reg->read(ptp, ADDR_16, TRIG_ERROR);
		if (status & tso_bit)
			err = DEV_IOC_UNIT_ERROR;
	}
	ptp->ops->release(ptp);

proc_dev_tx_event_done:
	*data = tso;
	return err;
}  /* proc_dev_tx_event */

int proc_ptp_tx_cascade_init(struct Ksz_ptp_info *ptp, u8 *data)
{
	struct ptp_tso_options *cmd = (struct ptp_tso_options *) data;
	int first;
	int gpo;
	int i;
	int tso;
	int total;
	u16 status;

	tso = cmd->tso;
	gpo = cmd->gpo;
	total = cmd->total;
	if (!total)
		return -EINVAL;
	if (gpo >= MAX_GPIO)
		return -EINVAL;
	first = tso;

	/* Cancel operation. */
	if ((cmd->flags & PTP_CMD_CANCEL_OPER)) {
		if (tso >= MAX_TRIG_UNIT)
			return DEV_IOC_UNIT_UNAVAILABLE;
		if (first != ptp->cascade_gpo[gpo].first ||
				total != ptp->cascade_gpo[gpo].total) {
			first = ptp->cascade_gpo[gpo].first;
			total = ptp->cascade_gpo[gpo].total;
		}

		/* Reset the last unit in case it is used to raise the level. */
		first = first + total - 1;
		if (ptp->outputs[first].level) {
			ptp->cascade_tx |= (1 << first);
			ptp->tso_used |= (1 << first);
		}
		ptp->ops->acquire(ptp);
		for (i = 0; i < total; i++, tso++) {
			if (ptp->tso_used & (1 << tso))
				ptp_tso_off(ptp, tso, (1 << tso));
		}
		tso = total;
		ptp->cascade = false;
		ptp->ops->release(ptp);
		goto proc_ptp_tx_cascade_init_done;
	}

	if (ptp->cascade)
		return DEV_IOC_UNIT_UNAVAILABLE;

	/* Find available units for use. */
	if (tso >= MAX_TRIG_UNIT) {
		int rc = find_avail_tx_unit(ptp, total, &first);

		if (rc)
			return rc;
	} else {
		for (i = 0; i < total; i++) {
			if (tso >= MAX_TRIG_UNIT)
				return DEV_IOC_UNIT_UNAVAILABLE;
			if (ptp->tso_used & (1 << tso))
				return DEV_IOC_UNIT_USED;
			++tso;
		}
	}

	if ((cmd->flags & PTP_CMD_CASCADE_RESET_OPER))
		goto proc_ptp_tx_cascade_init_set;

	/* Last operation was not in cascade mode. */
	if (!ptp->cascade_gpo[gpo].total)
		goto proc_ptp_tx_cascade_init_set;

	/* previous last unit. */
	i = ptp->cascade_gpo[gpo].first + ptp->cascade_gpo[gpo].total - 1;

	/* current last unit. */
	tso = first + total - 1;

	/* Last operation not ended high. */
	if (tso == i || !ptp->outputs[i].level)
		goto proc_ptp_tx_cascade_init_set;

	ptp->ops->acquire(ptp);
	status = ptp->reg->read(ptp, ADDR_16, GPIO_MONITOR);

	/* Current level is high. */
	if (status & (1 << gpo)) {

		/* Set unit to hold the level high. */
		ptp->reg->tx_event(ptp, tso, gpo, TRIG_POS_EDGE, 0, 0, 1, 0, 1,
			0, PTP_CMD_INTR_OPER, 1, 0);

		/* Release the signal from the previous last unit. */
		ptp_gpo_reset(ptp, ptp->outputs[i].gpo, (1 << i));
	}
	ptp->ops->release(ptp);

proc_ptp_tx_cascade_init_set:
	ptp->cascade = true;
	ptp->cascade_gpo[gpo].first = first;
	ptp->cascade_gpo[gpo].total = total;
	tso = first;

proc_ptp_tx_cascade_init_done:
	*data = tso;
	return 0;
}  /* proc_ptp_tx_cascade_init */

int proc_ptp_tx_cascade(struct Ksz_ptp_info *ptp, u8 *data)
{
	struct ptp_tso_options *cmd = (struct ptp_tso_options *) data;
	int first;
	int gpo;
	int tso;
	int total;
	struct ptp_utime t;

	gpo = cmd->gpo;
	if (gpo >= MAX_GPIO)
		return -EINVAL;
	tso = cmd->tso;
	total = cmd->total;
	first = tso;
	if (!ptp->cascade || tso != ptp->cascade_gpo[gpo].first ||
			total != ptp->cascade_gpo[gpo].total)
		return DEV_IOC_UNIT_UNAVAILABLE;

	/* Cancel operation. */
	if ((cmd->flags & PTP_CMD_CANCEL_OPER)) {
		proc_ptp_tx_cascade_init(ptp, data);
		goto proc_ptp_tx_cascade_done;
	}
	ptp->ops->acquire(ptp);
	if ((cmd->flags & PTP_CMD_REL_TIME) && cmd->sec < 100) {
		ptp->reg->get_time(ptp, &t);
		cmd->sec += t.sec;
	}
	total = ptp->reg->tx_cascade(ptp, tso, total, cmd->cnt, cmd->sec,
		cmd->nsec, cmd->flags & PTP_CMD_INTR_OPER);
	if (!total)
		ptp->cascade = false;
	ptp->ops->release(ptp);

proc_ptp_tx_cascade_done:
	*data = tso;
	return 0;
}  /* proc_ptp_tx_cascade */

void proc_tsm_get_gps(struct Ksz_ptp_info *ptp, u8 *data)
{
	struct tsm_get_gps *get = (struct tsm_get_gps *) data;

	if (!ptp->gps_dev)
		return;

	get->cmd |= TSM_CMD_GET_TIME_RESP;
	get->seqid = htons(ptp->gps_seqid);
	get->sec = htonl(ptp->gps_time.sec);
	get->nsec = htonl(ptp->gps_time.nsec);
	ptp_setup_udp_msg(ptp->gps_dev, data, sizeof(struct tsm_get_gps),
		NULL, NULL);
	ptp->gps_dev = NULL;
}  /* proc_tsm_get_gps */

int proc_dev_get_event(struct ptp_dev_info *info, u8 *data)
{
	struct Ksz_ptp_info *ptp = info.ptp;
	int len;
	struct ptp_tsi_info *in = (struct ptp_tsi_info *) data;
	u8 buf[sizeof(struct ptp_utime) * MAX_TIMESTAMP_EVENT_UNIT +
		sizeof(struct ptp_tsi_info)];
	struct ptp_tsi_info *out = (struct ptp_tsi_info *) buf;

	if (in->unit >= MAX_TIMESTAMP_UNIT)
		return -EINVAL;
	if (!ptp->events[in->unit].num)
		return DEV_IOC_UNIT_UNAVAILABLE;
	out->cmd = in->cmd | PTP_CMD_RESP;
	out->unit = in->unit;
	out->event = ptp->events[in->unit].event;
	out->num = ptp->events[in->unit].num;
	out->edge = ptp->events[in->unit].edge;
	len = sizeof(struct ptp_utime) * out->num;
	memcpy(out->t, ptp->events[in->unit].t, len);
	len += sizeof(struct ptp_tsi_info);
	ptp_setup_udp_msg(info, buf, len, NULL, NULL);
	return 0;
}  /* proc_dev_get_event */

int proc_ptp_get_event(struct Ksz_ptp_info *ptp, u8 *data)
{
	struct ptp_tsi_info *in = (struct ptp_tsi_info *) data;
	int ret = -1;

	if (ptp->tsi_dev[in->unit])
		ret = proc_dev_get_event(ptp->tsi_dev[in->unit], data);
	return ret;
}  /* proc_ptp_get_event */

int proc_ptp_get_trig(struct Ksz_ptp_info *ptp, u8 *data, u16 done,
	u16 error)
{
	int len;
	struct ptp_tsi_info *in = (struct ptp_tsi_info *) data;
	u8 buf[sizeof(struct ptp_utime) + sizeof(struct ptp_tsi_info)];
	struct ptp_tsi_info *out = (struct ptp_tsi_info *) buf;
	struct ptp_output *cur;
	int tso = in->unit;
	int tso_bit = (1 << tso);

	out->cmd = in->cmd | PTP_CMD_RESP;
	out->unit = in->unit;
	cur = &ptp->outputs[tso];
	if (error & tso_bit)
		out->event = 1;
	else if (!(done & tso_bit))
		out->event = 2;
	else
		out->event = 0;
	out->num = 1;
	len = sizeof(struct ptp_utime) * out->num;
	memcpy(out->t, &cur->trig, len);
	len += sizeof(struct ptp_tsi_info);
	if (ptp->tso_dev[tso]) {
		ptp_setup_udp_msg(ptp->tso_dev[tso], buf, len, NULL, NULL);
		return 0;
	}
	return -1;
}  /* proc_ptp_get_trig */

int proc_dev_poll_event(struct ptp_dev_info *info, u8 *data)
{
	struct Ksz_ptp_info *ptp = info.ptp;
	struct ptp_tsi_info *in = (struct ptp_tsi_info *) data;

	if (in->unit >= MAX_TIMESTAMP_UNIT)
		return -EINVAL;
	if (!ptp_poll_event(ptp, in->unit))
		return DEV_IOC_UNIT_UNAVAILABLE;
	return proc_dev_get_event(info, data);
}  /* proc_dev_poll_event */

int proc_ptp_get_output(struct Ksz_ptp_info *ptp, u8 *data)
{
	int *output = (int *) data;
	struct ptp_tso_options *in = (struct ptp_tso_options *) data;

	if (in->gpo >= MAX_GPIO)
		return -EINVAL;
	*output = ptp->cascade_gpo[in->gpo].tso;
	return 0;
}  /* proc_ptp_get_output */

void proc_ptp_get_clk(struct Ksz_ptp_info *ptp, u8 *data)
{
	struct ptp_utime t;
	struct ptp_clk_options *cmd = (struct ptp_clk_options *) data;

	ptp->ops->acquire(ptp);
	ptp->reg->get_time(ptp, &t);
	ptp->ops->release(ptp);
	cmd->sec = t.sec;
	cmd->nsec = t.nsec;
}  /* proc_ptp_get_clk */

int proc_ptp_set_clk(struct Ksz_ptp_info *ptp, u8 *data)
{
	struct ptp_utime t;
	struct ptp_clk_options *cmd = (struct ptp_clk_options *) data;
	struct timespec ts;
	struct ptp_utime sys_time;

	t.sec = cmd->sec;
	t.nsec = cmd->nsec;
	ptp->ops->acquire(ptp);
	ts = ktime_to_timespec(ktime_get_real());
	sys_time.sec = ts.tv_sec;
	sys_time.nsec = ts.tv_nsec;
	ptp->reg->set_time(ptp, &t);
	ptp->cur_time = t;
	calc_udiff(&ptp->cur_time, &sys_time, &ptp->time_diff);
	generate_tx_event(ptp, ptp->pps_gpo);
	ptp->ops->release(ptp);
	dbg_msg(" set clk: %x:%09u\n", cmd->sec, cmd->nsec);
	return 0;
}  /* proc_ptp_set_clk */

int proc_ptp_adj_clk(struct Ksz_ptp_info *ptp, u8 *data, int adjust)
{
	struct ptp_clk_options *cmd = (struct ptp_clk_options *) data;

	adjust--;
	if (cmd->sec > 1) {
		ptp->adjust_sec = cmd->sec;
		ptp->adjust_offset = cmd->nsec;
		if (!adjust) {
			ptp->adjust_sec = -ptp->adjust_sec;
			ptp->adjust_offset = -ptp->adjust_offset;
		}
		cmd->sec = cmd->nsec = 0;
		adj_clock(&ptp->adj_clk);
	}
	ptp->ops->acquire(ptp);
	if (cmd->nsec || cmd->sec) {
		ptp->sec_changed = cmd->sec;
		if (!(ptp->features & PTP_ADJ_SEC)) {
			cmd->nsec += cmd->sec * NANOSEC_IN_SEC;
			cmd->sec = 0;
		}
		ptp->reg->adjust_time(ptp, adjust, cmd->sec, cmd->nsec,
			ptp->features & PTP_ADJ_HACK);
		ptp->offset_changed = cmd->nsec;
		if (!adjust)
			ptp->offset_changed = -cmd->nsec;
		if (ptp->sec_changed)
			generate_tx_event(ptp, ptp->pps_gpo);
		else {
			ptp->update_sec_jiffies = jiffies;
			schedule_delayed_work(&ptp->check_pps,
				1200 * HZ / 1000);
		}
		if (ptp->sec_changed) {
			if (adjust)
				ptp->cur_time.sec += cmd->sec;
			else
				ptp->cur_time.sec -= cmd->sec;
			ptp->sec_changed = 0;
		}
		if (adjust)
			add_nsec(&ptp->cur_time, cmd->nsec);
		else
			sub_nsec(&ptp->cur_time, cmd->nsec);
		dbg_msg(" adj clk: %d %u:%09u\n", adjust, cmd->sec, cmd->nsec);
	}
	if (cmd->interval) {
		ptp->drift = cmd->drift;
		if (ptp->drift > MAX_DRIFT_CORR)
			ptp->drift = MAX_DRIFT_CORR;
		else if (ptp->drift < -MAX_DRIFT_CORR)
			ptp->drift = -MAX_DRIFT_CORR;
		ptp->drift_set = ptp->drift;
		ptp->adjust = clk_adjust_val(ptp->drift, cmd->interval);
		set_ptp_adjust(ptp, ptp->adjust);
		if (!ptp->ptp_synt) {
			syntonize_clk(ptp);
			ptp->ptp_synt = true;
		}
		if (!ptp->first_drift)
			ptp->first_drift = ptp->drift_set;
		dbg_msg(" adj drift: %d\n", cmd->drift);
	}
	ptp->ops->release(ptp);
	return 0;
}  /* proc_ptp_adj_clk */

int proc_ptp_get_delay(struct Ksz_ptp_info *ptp, int port, u8 *data)
{
	struct ptp_delay_values *delay = (struct ptp_delay_values *) data;

	if (port >= MAX_PTP_PORT)
		return DEV_IOC_INVALID_CMD;
	ptp->ops->acquire(ptp);
	delay->rx_latency = get_ptp_ingress(ptp, port);
	delay->tx_latency = get_ptp_egress(ptp, port);
	delay->asym_delay = get_ptp_asym(ptp, port);
	ptp->ops->release(ptp);
	return 0;
}  /* proc_ptp_get_delay */

int proc_ptp_set_delay(struct Ksz_ptp_info *ptp, int port, u8 *data)
{
	struct ptp_delay_values *delay = (struct ptp_delay_values *) data;

	if (port >= MAX_PTP_PORT)
		return DEV_IOC_INVALID_CMD;
	ptp->ops->acquire(ptp);
	set_ptp_ingress(ptp, port, delay->rx_latency);
	set_ptp_egress(ptp, port, delay->tx_latency);
	set_ptp_asym(ptp, port, delay->asym_delay);
	ptp->rx_latency[port] = delay->rx_latency;
	ptp->tx_latency[port] = delay->tx_latency;
	ptp->asym_delay[port] = delay->asym_delay;
	ptp->ops->release(ptp);
	dbg_msg("set delay: %d = %d %d %d\n", port,
		ptp->rx_latency[port],
		ptp->tx_latency[port],
		ptp->asym_delay[port]);
	return 0;
}  /* proc_ptp_set_delay */

int proc_ptp_get_peer_delay(struct Ksz_ptp_info *ptp, int port, u8 *data)
{
	struct ptp_delay_values *delay = (struct ptp_delay_values *) data;

	if (port >= MAX_PTP_PORT)
		return DEV_IOC_INVALID_CMD;
	ptp->ops->acquire(ptp);
	delay->rx_latency = 0;
	delay->tx_latency = 0;
	delay->asym_delay = 0;
	delay->reserved = get_ptp_link(ptp, port);
	ptp->ops->release(ptp);
	return 0;
}  /* proc_ptp_get_peer_delay */

int proc_ptp_set_peer_delay(struct Ksz_ptp_info *ptp, int port, u8 *data)
{
	struct ptp_delay_values *delay = (struct ptp_delay_values *) data;

	if (port >= MAX_PTP_PORT)
		return DEV_IOC_INVALID_CMD;
	ptp->ops->acquire(ptp);
	set_ptp_link(ptp, port, delay->reserved);
	ptp->peer_delay[port] = delay->reserved;
	ptp->ops->release(ptp);
	dbg_msg("set delay: %d = %d\n", port,
		ptp->peer_delay[port]);
	return 0;
}  /* proc_ptp_set_peer_delay */

void ptp_tx_done(struct Ksz_ptp_info *ptp, int tso)
{
	int first;
	int last;
	int prev;
	u32 reg;
	u16 data;

	reg = TRIGn_CONF_1(tso);
	data = ptp->reg->read(ptp, ADDR_16, reg);
	if (data & TRIG_CASCADE_EN) {
		last = tso;
		do {
			--tso;
			reg = TRIGn_CONF_1(tso);
			data = ptp->reg->read(ptp, ADDR_16, reg);
			prev = (data & TRIG_CASCADE_UPS_MASK) >> 10;
			if (prev == last)
				break;
		} while (tso > 0);
		first = tso;
		for (tso = last; tso > first; tso--)
			ptp_tso_off(ptp, tso, (1 << tso));
	}
	ptp_tso_off(ptp, tso, (1 << tso));
}  /* ptp_tx_done */

struct ptp_tx_ts *proc_get_ts(struct Ksz_ptp_info *ptp, u8 port, u8 msg,
	u16 seqid, u8 *mac, struct ptp_dev_info *info, int len)
{
	struct ptp_tx_ts *tx;
	int from_stack = false;
	u8 *data = NULL;

	if (info)
		data = info.write_buf;
	if (SYNC_MSG == msg)
		tx = &ptp->tx_sync[port];
	else if (PDELAY_RESP_MSG == msg)
		tx = &ptp->tx_resp[port];
	else
		tx = &ptp->tx_dreq[port];
	if (seqid || mac[0] || mac[1])
		from_stack = true;
	if (data && tx->req_time && ptp->linked[port])
		dbg_msg("  last %x=%04x: %d, %lu\n", msg, seqid, port,
			jiffies - tx->req_time);
	tx->missed = false;
	tx->req_time = jiffies;
	if (tx->ts.timestamp && from_stack) {
		unsigned long diff = tx->req_time - tx->resp_time;

		/* The timestamp is not valid. */
		if (diff >= 4 * ptp->delay_ticks) {
			dbg_msg("  invalid: %x=%04x: %d, %lu\n",
				msg, seqid, port, diff);
			tx->ts.timestamp = 0;
		}
#ifdef PTP_SPI
		else if (diff > 2 * ptp->delay_ticks)
			dbg_msg("  ready? %x=%04x: %d, %lu\n",
				msg, seqid, port, diff);
#endif
	}
	if (!tx->ts.timestamp && ptp->linked[port] && data) {
		int rc = wait_event_interruptible_timeout(ptp->wait_ts,
			0 != tx->ts.timestamp, ptp->delay_ticks);

		if (rc < 0)
			return NULL;
	}
	if (!tx->ts.timestamp) {
		if (from_stack && data) {
			tx->missed = true;
			memcpy(tx->data.buf, data, len);
			tx->data.len = len;
			tx->dev = info;
		}
#ifndef PTP_SPI
		if (ptp->linked[port])
			dbg_msg("  missed %x=%04x: %d, %lu\n",
				msg, seqid, port, jiffies - tx->req_time);
#endif
		tx = NULL;
	}
	return tx;
}  /* proc_get_ts */

int proc_ptp_get_timestamp(struct Ksz_ptp_info *ptp, u8 *data,
	struct ptp_dev_info *info)
{
	struct ptp_ts_options *opt = (struct ptp_ts_options *) data;

	if (opt->timestamp) {
		struct ptp_ts ts;

		ts.timestamp = opt->timestamp;
		update_ts(&ts, ptp->cur_time.sec);
		opt->sec = ts.t.sec;
		opt->nsec = ts.t.nsec;
	} else {
		struct ptp_tx_ts *tx;
		struct tsm_db *db;

		if (opt->port >= MAX_PTP_PORT)
			return DEV_IOC_INVALID_CMD;

		/* Save timestamp information for later reporting. */
		if (info) {
			db = (struct tsm_db *) info.write_buf;
			db->cmd = opt->msg;
			db->cmd |= TSM_CMD_DB_GET;
			db->index = opt->port << 1;
			db->seqid = htons(opt->seqid);
			db->mac[0] = opt->mac[0];
			db->mac[1] = opt->mac[1];
		}
		tx = proc_get_ts(ptp, opt->port, opt->msg,
			opt->seqid, opt->mac, info, sizeof(struct tsm_db));
		if (!tx)
			return -EAGAIN;
		opt->sec = tx->ts.r.sec;
		opt->nsec = tx->ts.r.nsec;
		tx->ts.timestamp = 0;
		tx->req_time = 0;
	}
	return 0;
}  /* proc_ptp_get_timestamp */

int parse_tsm_msg(struct ptp_dev_info *info, int len)
{
	struct Ksz_ptp_info *ptp = info.ptp;
	u8 *data = info.write_buf;
	u8 cmd = data[0] & 0xf0;
	u8 msg = data[0] & 0x03;
	int result = 0;

	switch (cmd) {
	case TSM_CMD_DB_GET_TIME:
	{
		struct tsm_get_time *get = (struct tsm_get_time *) data;
		struct ptp_ts ts;

		ts.timestamp = ntohl(get->nsec);
		if (ts.timestamp) {
			update_ts(&ts, ptp->cur_time.sec);
		} else {
			ptp->ops->acquire(ptp);
			ptp->reg->get_time(ptp, &ts.t);
			ptp->ops->release(ptp);
		}
		ptp_setup_udp_msg(info, data, len, ptp_tsm_get_time_resp,
			&ts.t);
		break;
	}
	case TSM_CMD_DB_GET:
	{
		struct tsm_db *db = (struct tsm_db *) data;

		if (db->index <= 3) {
			struct ptp_tx_ts *tx;
			int port = db->index >> 1;

			tx = proc_get_ts(ptp, port, msg, ntohs(db->seqid),
				db->mac, info, len);
			if (tx) {
				ptp_setup_udp_msg(info, data, len,
					ptp_tsm_resp, &tx->ts);
				tx->ts.timestamp = 0;
				tx->req_time = 0;
			}
		}
		break;
	}
	case TSM_CMD_GET_GPS_TS:
	{
		/* First time to get GPS timestamp. */
		if (MAX_TIMESTAMP_UNIT == ptp->gps_tsi) {
			ptp->gps_tsi = DEFAULT_GPS_TSI;
			if (ptp->tsi_used & (1 << ptp->gps_tsi))
				ptp->reg->rx_off(ptp, ptp->gps_tsi);
			prepare_gps(ptp);
			ptp->gps_seqid = 0;
		}
		ptp->gps_req_time = jiffies;
		ptp->gps_dev = info;
		if (ptp->gps_resp_time) {
			unsigned long diff = ptp->gps_req_time -
				ptp->gps_resp_time;

			/* The timestamp is not valid. */
			if (diff >= 2 * ptp->delay_ticks) {
				dbg_msg("  invalid gps: %lu\n", diff);
				ptp->gps_time.sec = 0;
			}
		}
		if (ptp->gps_time.sec) {
			proc_tsm_get_gps(ptp, data);
			ptp->gps_time.sec = 0;
			ptp->gps_req_time = 0;
		} else
			dbg_msg("  missed gps\n");
		break;
	}
	case TSM_CMD_CNF_SET:
	{
		struct tsm_cfg *cfg = (struct tsm_cfg *) data;
		u32 ingress = htonl(cfg->ingress_delay);

		ptp->ops->acquire(ptp);
		if (0xFF == cfg->port) {
			u16 mode;

			if (cfg->enable & 0x04)
				ptp->mode |= PTP_TC_P2P;
			else
				ptp->mode &= ~PTP_TC_P2P;
			mode = ptp->mode;
			if (ptp->overrides & PTP_VERIFY_TIMESTAMP)
				mode |= PTP_1STEP;
			set_ptp_mode(ptp, mode);
			ptp->def_mode = ptp->mode;
		} else {
			u8 port = cfg->port - 1;

			if ((cfg->enable & 0x10) && port < MAX_PTP_PORT &&
					ptp->peer_delay[port] != ingress) {
				ptp->peer_delay[port] = ingress;
				set_ptp_link(ptp, port, ingress);
			}
		}
		ptp->ops->release(ptp);
		break;
	}
	case TSM_CMD_CLOCK_SET:
	{
		struct tsm_clock_set *clk = (struct tsm_clock_set *) data;
		struct ptp_ts ts;

		ts.t.sec = ntohl(clk->sec);
		ts.t.nsec = ntohl(clk->nsec);
		ts.timestamp = ntohl(clk->timestamp);
		ptp->ops->acquire(ptp);
		set_cur_time(ptp, &ts);
		ptp->ops->release(ptp);
		ptp->state = 2;
		break;
	}
	case TSM_CMD_CLOCK_CORRECT:
	{
		struct tsm_clock_correct *clk = (struct tsm_clock_correct *)
			data;
		u32 drift;
		u32 nsec;
		int ptp_offset;

		drift = ntohl(clk->drift);
		nsec = ntohl(clk->nsec);
		ptp_offset = ntohl(clk->offset);
		if (2 == (clk->add >> 4))
			break;

		ptp->ops->acquire(ptp);
		if (nsec) {
			ptp->reg->adjust_time(ptp, !ptp_offset, 0, nsec,
				ptp->features & PTP_ADJ_HACK);
			ptp->offset_changed = nsec;
			if (ptp_offset)
				ptp->offset_changed = -nsec;
			ptp->update_sec_jiffies = jiffies;
			schedule_delayed_work(&ptp->check_pps,
				1200 * HZ / 1000);
		}
		if (clk->add & 1)
			ptp->drift = drift;
		else
			ptp->drift = -drift;
		if (ptp->drift > MAX_DRIFT_CORR)
			ptp->drift = MAX_DRIFT_CORR;
		else if (ptp->drift < -MAX_DRIFT_CORR)
			ptp->drift = -MAX_DRIFT_CORR;
		ptp->drift_set = ptp->drift;
		ptp->adjust = clk_adjust_val(ptp->drift,
			NANOSEC_IN_SEC);
		set_ptp_adjust(ptp, ptp->adjust);
		if (!ptp->first_drift)
			ptp->first_drift = ptp->drift_set;
		ptp->ops->release(ptp);
		break;
	}
	default:
		dbg_msg("tsm cmd: %02X, %d\n", cmd, len);
	}
	return result;
}  /* parse_tsm_msg */

struct Ksz_ptp_info *ptp_priv;

struct ptp_dev_info *alloc_dev_info(unsigned int minor)
{
	struct ptp_dev_info *info;

	info = kzalloc(sizeof(struct ptp_dev_info), GFP_KERNEL);
	if (info) {
		info.ptp = ptp_priv;
		sema_init(&info.sem, 1);
		mutex_init(&info.lock);
		init_waitqueue_head(&info.wait_udp);
		info.write_len = 1000;
		info.write_buf = kzalloc(info.write_len, GFP_KERNEL);
		info.read_max = 60000;
		info.read_buf = kzalloc(info.read_max, GFP_KERNEL);

		info.minor = minor;
		info.next = ptp_priv->dev[minor];
		ptp_priv->dev[minor] = info;
	}
	return info;
}  /* alloc_dev_info */

void free_dev_info(struct ptp_dev_info *info)
{
	if (info) {
		struct Ksz_ptp_info *ptp = info.ptp;
		unsigned int minor = info.minor;
		struct ptp_dev_info *prev = ptp->dev[minor];

		if (prev == info) {
			ptp->dev[minor] = info.next;
		} else {
			while (prev && prev->next != info)
				prev = prev->next;
			if (prev)
				prev->next = info.next;
		}
		kfree(info.read_buf);
		kfree(info.write_buf);
		kfree(info);
	}
}  /* free_dev_info */

int ptp_dev_open(struct inode *inode, struct file *filp)
{
	struct ptp_dev_info *info = (struct ptp_dev_info *)
		filp->private_data;
	unsigned int minor = MINOR(inode->i_rdev);

	if (minor > 1)
		return -ENODEV;
	if (!info) {
		info = alloc_dev_info(minor);
		if (info)
			filp->private_data = info;
		else
			return -ENOMEM;
	}
	return 0;
}  /* ptp_dev_open */

int ptp_dev_release(struct inode *inode, struct file *filp)
{
	struct ptp_dev_info *info = (struct ptp_dev_info *)
		filp->private_data;

	free_dev_info(info);
	filp->private_data = NULL;
	return 0;
}  /* ptp_dev_release */

int execute_wait(struct ptp_work *work)
{
	int rc = 0;

	execute(work->ptp, &work->work);
#ifdef PTP_SPI
	wait_for_completion(&work->done);
#endif
	return rc;
}  /* execute_wait */

void proc_ptp_work(struct work_struct *work)
{
	struct ptp_work *parent =
		container_of(work, struct ptp_work, work);
	struct Ksz_ptp_info *ptp = parent->ptp;
	struct ptp_dev_info *info = parent->dev_info;
	u8 *data = parent->param.data;
	int port;
	u32 reg;
	int result = DEV_IOC_OK;

	parent->output = parent->option;
	switch (parent->cmd) {
	case DEV_CMD_INFO:
		switch (parent->subcmd) {
		case DEV_INFO_INIT:
			ptp_init_state(ptp);
			parent->output = ptp->drift;
			break;
		case DEV_INFO_EXIT:
			ptp_exit_state(ptp);
			break;
		case DEV_INFO_RESET:
			reg = parent->option;
			ptp->ops->acquire(ptp);
			ptp->reg->write(ptp, ADDR_16, REG_RESET_CTRL,
				1 << reg);
			udelay(1);
			ptp->reg->write(ptp, ADDR_16, REG_RESET_CTRL, 0);
			ptp->ops->release(ptp);
			break;
		default:
			result = DEV_IOC_INVALID_CMD;
			break;
		}
		break;
	case DEV_CMD_PUT:
		switch (parent->subcmd) {
		case DEV_PTP_CFG:
			result = proc_ptp_set_cfg(ptp, data);
			break;
		case DEV_PTP_TEVT:
			result = proc_dev_rx_event(info, data);
			parent->output = *data;
			break;
		case DEV_PTP_TOUT:
			result = proc_dev_tx_event(info, data);
			parent->output = *data;
			break;
		case DEV_PTP_CASCADE:
			if (parent->option)
				result = proc_ptp_tx_cascade(ptp, data);
			else
				result = proc_ptp_tx_cascade_init(ptp, data);
			parent->output = *data;
			break;
		case DEV_PTP_CLK:
			if (parent->option)
				result = proc_ptp_adj_clk(ptp, data,
					parent->option);
			else
				result = proc_ptp_set_clk(ptp, data);
			break;
		case DEV_PTP_DELAY:
			port = parent->option;
			result = proc_ptp_set_delay(ptp, port, data);
			break;
		case DEV_PTP_REG:
			reg = parent->option;
			ptp->ops->acquire(ptp);
			ptp->reg->write(ptp, ADDR_16, reg & 0xffff,
				reg >> 16);
			ptp->ops->release(ptp);
			break;
		case DEV_PTP_PEER_DELAY:
			port = parent->option;
			result = proc_ptp_set_peer_delay(ptp, port, data);
			break;
		default:
			result = DEV_IOC_INVALID_CMD;
			break;
		}
		break;
	case DEV_CMD_GET:
		switch (parent->subcmd) {
		case DEV_PTP_CFG:
			proc_ptp_get_cfg(ptp, data);
			break;
		case DEV_PTP_TEVT:
			if (parent->option)
				result = proc_dev_poll_event(info, data);

			/* Not actually used. */
			else
				result = proc_dev_get_event(info, data);
			break;
		case DEV_PTP_TOUT:
			break;
		case DEV_PTP_CLK:
			proc_ptp_get_clk(ptp, data);
			break;
		case DEV_PTP_DELAY:
			port = parent->option;
			result = proc_ptp_get_delay(ptp, port, data);
			break;
		case DEV_PTP_REG:
			reg = parent->option;
			ptp->ops->acquire(ptp);
			parent->output = ptp->reg->read(ptp, ADDR_16, reg);
			ptp->ops->release(ptp);
			break;
		case DEV_PTP_PEER_DELAY:
			port = parent->option;
			result = proc_ptp_get_peer_delay(ptp, port, data);
			break;
		}
		break;
	default:
		result = DEV_IOC_INVALID_CMD;
		break;
	}
	parent->result = result;
	parent->used = false;
	complete(&parent->done);
}  /* proc_ptp_work */

int proc_ptp_hw_access(struct Ksz_ptp_info *ptp, int cmd, int subcmd,
	int option, void *data, size_t len, struct ptp_dev_info *info,
	int *output, int wait)
{
	struct ptp_access *access;
	struct ptp_work *work;
	int ret = 0;

	access = &ptp->hw_access;
	work = &access->works[access->index];
	if (work->used) {
		pr_alert("work full\n");
		return -EFAULT;
	}
	work->cmd = cmd;
	work->subcmd = subcmd;
	work->option = option;
	memcpy(work->param.data, data, len);
	work->dev_info = info;
	work->used = true;
	access->index++;
	access->index &= PTP_WORK_LAST;
#ifdef PTP_SPI
	init_completion(&work->done);
#endif
	if (!wait) {
		execute(ptp, &work->work);
		goto hw_access_end;
	}
	ret = execute_wait(work);

	/* Cannot continue if ERESTARTSYS. */
	if (ret < 0)
		return ret;

	ret = work->result;
	if (DEV_IOC_OK == ret && DEV_CMD_GET == work->cmd)
		memcpy(data, work->param.data, len);
	*output = work->output;

hw_access_end:
	return ret;
}  /* proc_ptp_hw_access */

void exit_ptp_work(struct Ksz_ptp_info *ptp)
{
	struct ptp_access *access;
	struct ptp_work *work;
	int i;

	access = &ptp->hw_access;
	for (i = 0; i < PTP_WORK_NUM; i++) {
		work = &access->works[i];
		flush_work(&work->work);
	}
}  /* exit_ptp_work */

void init_ptp_work(struct Ksz_ptp_info *ptp)
{
	struct ptp_access *access;
	struct ptp_work *work;
	int i;

	access = &ptp->hw_access;
	for (i = 0; i < PTP_WORK_NUM; i++) {
		work = &access->works[i];
		work->ptp = ptp;
		INIT_WORK(&work->work, proc_ptp_work);
		init_completion(&work->done);
	}
}  /* init_ptp_work */

#ifdef CONFIG_PTP_1588_CLOCK
#include "micrel_ptp.c"
#endif

u32 _get_clk_cnt(void)
{
	return 0;
}

#ifdef PTP_SPI
#define ACCESS_VAL			1000
#else
#define ACCESS_VAL			100
#endif

void test_get_time(struct Ksz_ptp_info *ptp, struct ptp_utime *cur,
	struct ptp_utime *now, u32 *cur_cnt, u32 *now_cnt)
{
	ptp->reg->get_time(ptp, cur);
	*cur_cnt = ptp->get_clk_cnt();
	ptp->reg->get_time(ptp, now);
	*now_cnt = ptp->get_clk_cnt();
}

void test_set_time(struct Ksz_ptp_info *ptp, struct ptp_utime *cur,
	struct ptp_utime *now, u32 *cur_cnt, u32 *now_cnt)
{
	*cur_cnt = ptp->get_clk_cnt();
	ptp->reg->set_time(ptp, cur);
	*now_cnt = ptp->get_clk_cnt();
	ptp->reg->get_time(ptp, now);
}

u32 test_avg_time(struct Ksz_ptp_info *ptp,
	void (*test_time)(struct Ksz_ptp_info *ptp, struct ptp_utime *cur,
	struct ptp_utime *now, u32 *cur_cnt, u32 *now_cnt))
{
	struct ptp_utime cur;
	struct ptp_utime now;
	struct ksz_ptp_time diff;
	int i;
	int clk_delay[6];
	u32 cur_cnt;
	u32 now_cnt;
	u32 hw_delay[6];
	u64 clk;
	u32 rem;

	cur.sec = 5;
	cur.nsec = 0x12345678;
	ptp->ops->acquire(ptp);
	for (i = 0; i < 5; i++) {
		test_time(ptp, &cur, &now, &cur_cnt, &now_cnt);
		calc_udiff(&cur, &now, &diff);
		clk_delay[i] = (diff.nsec + (ACCESS_VAL / 2)) / ACCESS_VAL *
			ACCESS_VAL;
		hw_delay[i] = now_cnt - cur_cnt;
	}
	ptp->ops->release(ptp);
	clk_delay[5] = 20000000;
	hw_delay[5] = 50000000;
	for (i = 0; i < 5; i++) {
		clk = hw_delay[i];
		clk *= 1000000;
		if (ptp->clk_divider)
			clk = div_u64_rem(clk, ptp->clk_divider, &rem);
		dbg_msg(" %u %u=%llu\n", clk_delay[i], hw_delay[i], clk);
		if (clk_delay[i] < clk_delay[5])
			clk_delay[5] = clk_delay[i];
		if (hw_delay[i] < hw_delay[5])
			hw_delay[5] = hw_delay[i];
	}
	clk = hw_delay[5];
	clk *= 1000000;
	if (ptp->clk_divider)
		clk = div_u64_rem(clk, ptp->clk_divider, &rem);
	dbg_msg("%u %llu\n", clk_delay[5], clk);
	return clk_delay[5];
}

void _test_access_time(struct Ksz_ptp_info *ptp)
{
	ptp->get_delay = test_avg_time(ptp, test_get_time);
	ptp->set_delay = test_avg_time(ptp, test_set_time);
	if (ptp->get_delay < 10000)
		ptp->delay_ticks = 10 * HZ / 1000;
	else if (ptp->get_delay < 12000000)
		ptp->delay_ticks = 20 * HZ / 1000;
	else
		ptp->delay_ticks = 30 * HZ / 1000;
	dbg_msg("delay_ticks: %lu\n", ptp->delay_ticks);
}  /* test_access_time */

void set_ptp_drift(struct Ksz_ptp_info *ptp, int drift)
{
	drift /= 100;
	drift *= 100;
	drift = -drift;
	ptp->first_drift = ptp->drift = drift;
	ptp->first_sec = 0;
	ptp->adjust = clk_adjust_val(drift, NANOSEC_IN_SEC);
	set_ptp_adjust(ptp, ptp->adjust);
	syntonize_clk(ptp);
	ptp->ptp_synt = true;
	dbg_msg("drift: %d\n", drift);
}  /* set_ptp_drift */

void check_sys_time(struct Ksz_ptp_info *ptp, unsigned long cur_jiffies,
	union ktime cur_ktime)
{
	int diff;
	int interval;
	u32 cur_clk_cnt;

	cur_clk_cnt = ptp->get_clk_cnt();
	if (!ptp->first_drift) {
		interval = 8;
		diff = ptp->cur_time.sec - ptp->intr_sec;

		/*
		 * The second interval is not accurate after first setting up
		 * the clock until later.
		 */
		if (diff < 6)
			ptp->first_sec = 0;
	} else
		interval = 10;

	if (!ptp->first_sec) {
		ptp->last_clk_cnt = cur_clk_cnt;
		ptp->total_clk_cnt = 0;
		ptp->last_jiffies = cur_jiffies;
		ptp->total_jiffies = 0;
		ptp->first_ktime = cur_ktime;
		ptp->first_sec = ptp->cur_time.sec;
		return;
	}

	diff = ptp->cur_time.sec - ptp->first_sec;

	if (diff >= 1 && !(diff % interval)) {
		u32 rem;
		u64 clk;
		u64 clk_cnt;
		s64 drift_clk;
		s64 drift_jiffies;
		s64 drift_ktime;
		u32 passed_sec;
		u64 passed_usec;
		u64 passed_nsec;
		u32 cnt;

		cnt = cur_clk_cnt - ptp->last_clk_cnt;
		ptp->total_clk_cnt += cnt;
		ptp->last_clk_cnt = cur_clk_cnt;

		passed_sec = ptp->cur_time.sec - ptp->first_sec;
		passed_usec = passed_sec;
		passed_usec *= 1000000;
		passed_nsec = passed_usec;
		passed_nsec *= 1000;

		cnt = cur_jiffies - ptp->last_jiffies;
		ptp->total_jiffies += cnt;
		ptp->last_jiffies = cur_jiffies;

		clk = ptp->total_jiffies * (1000000 / HZ);
		drift_jiffies = clk - passed_usec;
		drift_jiffies *= 1000;
		drift_jiffies = div_s64_rem(drift_jiffies, passed_sec, &rem);

		cur_ktime.tv64 -= ptp->first_ktime.tv64;
		drift_ktime = cur_ktime.tv64 - passed_nsec;
		drift_ktime = div_s64_rem(drift_ktime, passed_sec, &rem);

		if (!ptp->clk_divider) {
			if (!ptp->first_drift)
				set_ptp_drift(ptp, (int) drift_ktime);
			else
				printf( "%lld %lld\n",
					drift_jiffies, drift_ktime);
			return;
		}

		clk_cnt = div_u64_rem(ptp->total_clk_cnt, passed_sec, &rem);

		clk = ptp->total_clk_cnt * 1000000;
		clk = div_u64_rem(clk, ptp->clk_divider, &rem);
		drift_clk = clk;
		if (drift_clk < 0)
			ptp->overrides &= ~PTP_CHECK_SYS_TIME;
		drift_clk -= passed_nsec;
		drift_clk = div_s64_rem(drift_clk, passed_sec, &rem);

		if (!ptp->first_drift)
			set_ptp_drift(ptp, (int) drift_clk);
		else
			printf( "%10llu %lld %lld %lld\n",
				clk_cnt, drift_clk, drift_jiffies, drift_ktime);
	}
}  /* check_sys_time */

void proc_ptp_intr(struct Ksz_ptp_info *ptp)
{
	struct ptp_event *event;
	u16 done;
	u16 error;
	u16 status;
	u16 tsi_bit;
	u8 data[24];
	int i;
	int tsi;
	int last;
	union ktime cur_ktime;
	struct timespec ts;

	cur_ktime = ktime_get_real();
	ts = ktime_to_timespec(cur_ktime);

proc_chk_trig_intr:
	status = ptp->reg->read(ptp, ADDR_16, TRIG_INT_STATUS);
	if (!status)
		goto proc_chk_ts_intr;

	ptp->reg->write(ptp, ADDR_16, TRIG_INT_STATUS, status);
	done = ptp->reg->read(ptp, ADDR_16, TRIG_DONE);
	error = ptp->reg->read(ptp, ADDR_16, TRIG_ERROR);
	for (i = 0; i < MAX_TRIG_UNIT; i++) {
		if (status & (1 << i)) {
			if (ptp->tso_intr & (1 << i)) {
				data[0] = PTP_CMD_GET_OUTPUT;
				data[1] = i;
				proc_ptp_get_trig(ptp, data, done, error);
			}
			ptp_tx_done(ptp, i);
		}
	}

proc_chk_ts_intr:
	status = ptp->reg->read(ptp, ADDR_16, TS_INT_STATUS);
	if (!status)
		goto proc_ptp_intr_done;

	ptp->reg->write(ptp, ADDR_16, TS_INT_STATUS, status);
	for (i = 0; i < MAX_TIMESTAMP_UNIT; i++) {
		tsi_bit = 1 << i;
		if (!(status & tsi_bit))
			continue;
		ptp->reg->read_event(ptp, i);
		event = &ptp->events[i];
		if (event->timeout && (event->num < event->max ||
				event->last)) {
			unsigned long expired;

			expired = jiffies + event->timeout;
			event->expired = expired;
			if (event->last) {
				tsi = i + 1;
				do {
					if (tsi >= MAX_TIMESTAMP_UNIT)
						tsi = 0;
					ptp->events[tsi].expired = expired;
					++tsi;
				} while (tsi != event->last);
			}
		} else if (event->last && i != event->first) {
			tsi = i - 1;
			if (tsi < 0)
				tsi = MAX_TIMESTAMP_UNIT - 1;
			if (ptp->tsi_used & (1 << tsi))
				ptp->events[tsi].expired = jiffies;
		}
		if (i == ptp->pps_tsi) {
			struct ptp_utime sys_time;

			ptp->cur_time.sec = event->t[0].sec;
			ptp->cur_time.nsec = event->t[0].nsec;
			ptp->update_sec_jiffies = 0;
			sys_time.sec = ts.tv_sec;
			sys_time.nsec = ts.tv_nsec;
			calc_udiff(&ptp->cur_time, &sys_time, &ptp->time_diff);
			if (!ptp->intr_sec)
				ptp->intr_sec = ptp->cur_time.sec;
			if ((ptp->overrides & PTP_CHECK_SYS_TIME) ||
					!ptp->first_drift)
				check_sys_time(ptp, jiffies, cur_ktime);
#ifdef CONFIG_PTP_1588_CLOCK
			if (ptp->clock_events & (1 << 0))
				ptp_event_trigger(ptp->clock_info, 0,
					ptp->cur_time.sec, ptp->cur_time.nsec);
			if (ptp->clock_events & (1 << 31))
				ptp_event_pps(ptp->clock_info);
#endif
		} else if (i == ptp->gps_tsi) {
			ptp->gps_time.sec = event->t[0].sec;
			ptp->gps_time.nsec = event->t[0].nsec;
			++ptp->gps_seqid;
			ptp->gps_resp_time = jiffies;
		}
	}
	for (i = 0; i < MAX_TIMESTAMP_UNIT; i++) {
		int stop;

		stop = false;
		tsi_bit = 1 << i;
		event = &ptp->events[i];
		if (ptp->tsi_used & tsi_bit) {

			/* At least one event. */
			if (event->num || event->expired) {
				if (event->num >= event->max)
					stop = true;
				else if (event->expired &&
						jiffies >= event->expired) {
					ptp->reg->read_event(ptp, i);
					stop = true;
				}
			}
		}
		if ((ptp->ts_status & ptp->ts_intr) & tsi_bit) {
			if (ptp->tsi_intr & tsi_bit) {
				data[0] = PTP_CMD_GET_EVENT;
				data[1] = i;
				proc_ptp_get_event(ptp, data);
			}
			if (i == ptp->gps_tsi && ptp->gps_req_time) {
				unsigned long diff = jiffies -
					ptp->gps_req_time;

				if (diff < 2 * ptp->delay_ticks) {
					data[0] = TSM_CMD_GET_GPS_TS;
					proc_tsm_get_gps(ptp, data);
					ptp->gps_time.sec = 0;
				}
				ptp->gps_req_time = 0;
			}

			/* Not used in cascade mode. */
			if (!event->timeout && !event->last) {
				event->num = 0;
				ptp->reg->rx_restart(ptp, tsi_bit);
				stop = false;
			}
		}
		if (stop) {
			ptp->reg->rx_off(ptp, i);
			ptp->tsi_intr &= ~tsi_bit;
			ptp->tsi_used &= ~tsi_bit;
			ptp->tsi_dev[i] = NULL;
			tsi = i;
			ptp->events[i].timeout = 0;
			if (i + 1 == event->last) {
				tsi = event->first;
				last = event->last;
				do {
					if (tsi >= MAX_TIMESTAMP_UNIT)
						tsi = 0;
					ptp->events[tsi].first = 0;
					ptp->events[tsi].last = 0;
					++tsi;
				} while (tsi != last);
			}
		}
	}
	ptp->ts_status = 0;
	if (!(status & 0xF000))
		goto proc_chk_trig_intr;

	if (get_tx_time(ptp, status & 0xF000))
		wake_up_interruptible(&ptp->wait_ts);
	goto proc_chk_trig_intr;

proc_ptp_intr_done:
	return;
}  /* proc_ptp_intr */

int ptp_get_ts_info(struct Ksz_ptp_info *ptp, struct net_device *dev,
	struct ethtool_ts_info *info)
{
	int ptp_clock = false;
	struct ksz_sw *sw = container_of(ptp, struct ksz_sw, ptp_hw);
	int ret = -1;

#ifdef CONFIG_PTP_1588_CLOCK
	ptp_clock = true;
#endif
	if (!(g_sw.features & PTP_HW) || !ptp_clock)
		return ethtool_op_get_ts_info(dev, info);
#ifdef CONFIG_PTP_1588_CLOCK
	ret = micrel_ptp_get_ts_info(ptp, info);
#endif
	return ret;
}  /* ptp_get_ts_info */

#define PTP_ENABLE_TXTS		SIOCDEVPRIVATE
#define PTP_DISABLE_TXTS	(SIOCDEVPRIVATE + 1)
#define PTP_ENABLE_RXTS		(SIOCDEVPRIVATE + 2)
#define PTP_DISABLE_RXTS	(SIOCDEVPRIVATE + 3)
#define PTP_GET_TX_TIMESTAMP	(SIOCDEVPRIVATE + 4)
#define PTP_GET_RX_TIMESTAMP	(SIOCDEVPRIVATE + 5)
#define PTP_SET_TIME		(SIOCDEVPRIVATE + 6)
#define PTP_GET_TIME		(SIOCDEVPRIVATE + 7)
#define PTP_SET_FIPER_ALARM	(SIOCDEVPRIVATE + 8)
#define PTP_SET_ADJ		(SIOCDEVPRIVATE + 9)
#define PTP_GET_ADJ		(SIOCDEVPRIVATE + 10)
#define PTP_CLEANUP_TS		(SIOCDEVPRIVATE + 11)
#define PTP_ADJ_TIME		(SIOCDEVPRIVATE + 12)

struct ixxat_ptp_time {
	/* just 48 bit used */
	u64 sec;
	u32 nsec;
};

struct ixxat_ptp_ident {
	u8 vers;
	u8 mType;
	u16 netwProt;
	u16 seqId;
	struct ptp_port_identity portId;
} __packed;

/* needed for timestamp data over ioctl */
struct ixxat_ptp_data {
	struct ixxat_ptp_ident ident;
	struct ixxat_ptp_time ts;
};

int ixxat_ptp_ioctl(struct Ksz_ptp_info *ptp, unsigned int cmd,
	struct ifreq *ifr)
{
	struct ixxat_ptp_time ptp_time;
	struct ixxat_ptp_data ptp_data;
	struct ptp_clk_options clk_opt;
	int output;
	s64 scaled_nsec;
	struct ptp_ts ts;
	struct ptp_tx_ts *tx;
	int drift;
	int err = 0;
	int port;

	switch (cmd) {
	case PTP_ENABLE_TXTS:
		ptp->tx_en |= 2;
		break;
	case PTP_DISABLE_TXTS:
		ptp->tx_en &= ~2;
		break;
	case PTP_ENABLE_RXTS:
		ptp->rx_en |= 2;
		break;
	case PTP_DISABLE_RXTS:
		ptp->rx_en &= ~2;
		break;
	case PTP_GET_TX_TIMESTAMP:
		if (copy_from_user(&ptp_data, ifr->ifr_data, sizeof(ptp_data)))
			return -EFAULT;

		err = -EINVAL;
		port = htons(ptp_data.ident.portId.port);
		if (port < 1 || port > MAX_PTP_PORT)
			break;
		port--;
		tx = proc_get_ts(ptp, port, ptp_data.ident.mType,
			ptp_data.ident.seqId,
			ptp_data.ident.portId.clockIdentity.addr,
			NULL, 0);
		if (!tx)
			break;
		ptp_data.ts.sec = tx->ts.r.sec;
		ptp_data.ts.nsec = tx->ts.r.nsec;
		tx->ts.timestamp = 0;
		tx->req_time = 0;
		err = copy_to_user(ifr->ifr_data, &ptp_data, sizeof(ptp_data));
		break;
	case PTP_GET_RX_TIMESTAMP:
		if (copy_from_user(&ptp_data, ifr->ifr_data, sizeof(ptp_data)))
			return -EFAULT;

		ts.timestamp = ptp_data.ts.nsec;
		if (ts.timestamp)
			update_ts(&ts, ptp->cur_time.sec);
		else {
			ptp->ops->acquire(ptp);
			ptp->reg->get_time(ptp, &ts.t);
			ptp->ops->release(ptp);
		}
		ptp_data.ts.sec = ts.t.sec;
		ptp_data.ts.nsec = ts.t.nsec;
		err = copy_to_user(ifr->ifr_data, &ptp_data, sizeof(ptp_data));
		break;
	case PTP_GET_TIME:
	{
		struct timespec ts;
		struct ksz_ptp_time cur_time;
		struct ksz_ptp_time sys_time;

		ts = ktime_to_timespec(ktime_get_real());
		sys_time.sec = ts.tv_sec;
		sys_time.nsec = ts.tv_nsec;
		calc_diff(&ptp->time_diff, &sys_time, &cur_time);
		ptp_time.sec = cur_time.sec;
		ptp_time.nsec = cur_time.nsec;
		err = proc_ptp_hw_access(ptp,
			DEV_CMD_GET, DEV_PTP_CLK, 0,
			&clk_opt, sizeof(clk_opt), NULL, &output,
			true);
		if (err)
			break;
		ptp_time.sec = clk_opt.sec;
		ptp_time.nsec = clk_opt.nsec;
		err = copy_to_user(ifr->ifr_data, &ptp_time, sizeof(ptp_time));
		break;
	}
	case PTP_SET_TIME:
		if (copy_from_user(&ptp_time, ifr->ifr_data, sizeof(ptp_time)))
			return -EFAULT;
		output = 0;
		clk_opt.sec = (u32) ptp_time.sec;
		clk_opt.nsec = ptp_time.nsec;
		err = proc_ptp_hw_access(ptp,
			DEV_CMD_PUT, DEV_PTP_CLK, output,
			&clk_opt, sizeof(clk_opt), NULL, &output,
			true);
		break;
	case PTP_ADJ_TIME:
		if (copy_from_user(&scaled_nsec, ifr->ifr_data, sizeof(s64)))
			return -EFAULT;
		convert_scaled_nsec(scaled_nsec, SCALED_NANOSEC_S,
			&ptp->adjust_sec, &ptp->adjust_offset);
		if (ptp->adjust_offset < 0 || ptp->adjust_sec < 0) {
			output = 1;
			ptp->adjust_sec = -ptp->adjust_sec;
			ptp->adjust_offset = -ptp->adjust_offset;
		} else
			output = 2;
		clk_opt.sec = (u32) ptp->adjust_sec;
		clk_opt.nsec = ptp->adjust_offset;
		clk_opt.interval = 0;
		ptp->adjust_sec = 0;
		ptp->adjust_offset = 0;
		err = proc_ptp_hw_access(ptp,
			DEV_CMD_PUT, DEV_PTP_CLK, output,
			&clk_opt, sizeof(clk_opt), NULL, &output,
			true);
		break;
	case PTP_SET_ADJ:
		if (copy_from_user(&drift, ifr->ifr_data, sizeof(drift)))
			return -EFAULT;
		output = 1;
		clk_opt.sec = clk_opt.nsec = 0;
		clk_opt.drift = drift;
		clk_opt.interval = NANOSEC_IN_SEC;
		err = proc_ptp_hw_access(ptp,
			DEV_CMD_PUT, DEV_PTP_CLK, output,
			&clk_opt, sizeof(clk_opt), NULL, &output,
			true);
		break;
	case PTP_GET_ADJ:
		drift = ptp->drift;
		err = copy_to_user(ifr->ifr_data, &drift, sizeof(drift));
		break;
	case PTP_CLEANUP_TS:
		break;
	case PTP_SET_FIPER_ALARM:
		break;
	default:
		err = -EOPNOTSUPP;
	}
	return err;
}

int ptp_dev_req(struct Ksz_ptp_info *ptp, int start, char *arg,
	struct ptp_dev_info *info)
{
	struct ksz_request *req = (struct ksz_request *) arg;
	int len;
	int maincmd;
	int port;
	int req_size;
	int subcmd;
	int output;
	u8 data[PARAM_DATA_SIZE];
	struct ptp_dev_info *dev;
	int err = 0;
	int result = 0;

	/* Assume success. */
	result = DEV_IOC_OK;

	/* Check request size. */
	__get_user(req_size, &req->size);
	if (chk_ioctl_size(req_size, SIZEOF_ksz_request, 0, &req_size,
			&result, NULL, NULL))
		goto dev_ioctl_resp;

	err = 0;
	__get_user(maincmd, &req->cmd);
	__get_user(subcmd, &req->subcmd);
	__get_user(output, &req->output);
	len = req_size - SIZEOF_ksz_request;
	switch (maincmd) {
	case DEV_CMD_INFO:
		switch (subcmd) {
		case DEV_INFO_INIT:
			req_size = SIZEOF_ksz_request + 4;
			if (len >= 4) {
				data[0] = 'M';
				data[1] = 'i';
				data[2] = 'c';
				data[3] = 'r';
				data[4] = ptp->version;
				data[5] = ptp->ports;
				if (!access_ok(VERIFY_WRITE, req->param.data,
						6) ||
						copy_to_user(req->param.data,
						data, 6)) {
					err = -EFAULT;
					goto dev_ioctl_done;
				}
				result = proc_ptp_hw_access(ptp,
					maincmd, subcmd, 0,
					data, 6, info, &output,
					true);
				__put_user(ptp->drift, &req->output);
			} else
				result = DEV_IOC_INVALID_LEN;
			break;
		case DEV_INFO_EXIT:
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, 0, info, &output,
				true);

		/* fall through */
		case DEV_INFO_QUIT:
			if (!info)
				break;
			data[0] = 0xF0;
			dev = find_minor_dev(info);
			if (dev)
				ptp_setup_udp_msg(dev, data, 4, NULL, NULL);
			ptp_setup_udp_msg(info, data, 4, NULL, NULL);
			break;
		case DEV_INFO_RESET:
			if (output < 3) {
				result = proc_ptp_hw_access(ptp,
					maincmd, subcmd, output,
					data, 0, info, &output,
					false);
			} else
				result = -EINVAL;
			break;
		default:
			result = DEV_IOC_INVALID_CMD;
			break;
		}
		break;
	case DEV_CMD_PUT:
		switch (subcmd) {
		case DEV_PTP_CFG:
			if (chk_ioctl_size(len, sizeof(struct ptp_cfg_options),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, len, info, &output,
				false);
			break;
		case DEV_PTP_TEVT:
			if (chk_ioctl_size(len, sizeof(struct ptp_tsi_options),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			if (!info) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, len, info, &output,
				true);
			__put_user(output, &req->output);
			break;
		case DEV_PTP_TOUT:
			if (chk_ioctl_size(len, sizeof(struct ptp_tso_options),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			if (!info) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, len, info, &output,
				true);
			__put_user(output, &req->output);
			break;
		case DEV_PTP_CASCADE:
			if (chk_ioctl_size(len, sizeof(struct ptp_tso_options),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				true);
			__put_user(output, &req->output);
			break;
		case DEV_PTP_CLK:
			if (chk_ioctl_size(len, sizeof(struct ptp_clk_options),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				true);
			break;
		case DEV_PTP_DELAY:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_delay_values),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			__get_user(port, &req->output);
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				false);
			break;
		case DEV_PTP_REG:
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				false);
			break;
		case DEV_PTP_IDENTITY:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_clock_identity),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			memcpy(&ptp->clockIdentity, data,
				sizeof(struct ptp_clock_identity));
			break;
		case DEV_PTP_PEER_DELAY:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_delay_values),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			__get_user(port, &req->output);
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				false);
			break;
		case DEV_PTP_UTC_OFFSET:
			ptp->utc_offset = output;
			break;
		default:
			result = DEV_IOC_INVALID_CMD;
			break;
		}
		break;
	case DEV_CMD_GET:
		switch (subcmd) {
		case DEV_PTP_CFG:
			if (chk_ioctl_size(len, sizeof(struct ptp_cfg_options),
					SIZEOF_ksz_request,
					&req_size, &result, NULL, NULL))
				goto dev_ioctl_resp;
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, len, info, &output,
				true);
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_cfg_options)) ||
					copy_to_user(req->param.data, data,
					sizeof(struct ptp_cfg_options))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		case DEV_PTP_TEVT:
			if (chk_ioctl_size(len, sizeof(struct ptp_tsi_info),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			if (!info) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			if (output)
				result = proc_ptp_hw_access(ptp,
					maincmd, subcmd, output,
					data, len, info, &output,
					false);
			else
				result = proc_dev_get_event(info, data);
			break;
		case DEV_PTP_TOUT:
			if (chk_ioctl_size(len, sizeof(struct ptp_tso_options),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			result = proc_ptp_get_output(ptp, data);
			output = *((int *) data);
			__put_user(output, &req->output);
			break;
		case DEV_PTP_CLK:
			if (chk_ioctl_size(len, sizeof(struct ptp_clk_options),
					SIZEOF_ksz_request,
					&req_size, &result, NULL, NULL))
				goto dev_ioctl_resp;
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, len, info, &output,
				true);
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_clk_options)) ||
					copy_to_user(req->param.data, data,
					sizeof(struct ptp_clk_options))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		case DEV_PTP_DELAY:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_delay_values),
					SIZEOF_ksz_request,
					&req_size, &result, NULL, NULL))
				goto dev_ioctl_resp;
			__get_user(port, &req->output);
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				true);
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_delay_values)) ||
					copy_to_user(req->param.data, data,
					sizeof(struct ptp_delay_values))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		case DEV_PTP_REG:
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				true);
			__put_user(output, &req->output);
			break;
		case DEV_PTP_IDENTITY:
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_clock_identity)) ||
					copy_to_user(req->param.data,
					&ptp->clockIdentity,
					sizeof(struct ptp_clock_identity))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		case DEV_PTP_PEER_DELAY:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_delay_values),
					SIZEOF_ksz_request,
					&req_size, &result, NULL, NULL))
				goto dev_ioctl_resp;
			__get_user(port, &req->output);
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				true);
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_delay_values)) ||
					copy_to_user(req->param.data, data,
					sizeof(struct ptp_delay_values))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		case DEV_PTP_UTC_OFFSET:
			__put_user(ptp->utc_offset, &req->output);
			break;
		case DEV_PTP_TIMESTAMP:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_ts_options),
					SIZEOF_ksz_request,
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			result = proc_ptp_get_timestamp(ptp, data, info);
			if (result)
				goto dev_ioctl_resp;
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_ts_options)) ||
					copy_to_user(req->param.data, data,
					sizeof(struct ptp_ts_options))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		}
		break;
	default:
		result = DEV_IOC_INVALID_CMD;
		break;
	}

dev_ioctl_resp:
	__put_user(req_size, &req->size);
	__put_user(result, &req->result);

	/* Return ERESTARTSYS so that the system call is called again. */
	if (result < 0)
		err = result;

dev_ioctl_done:
	return err;
}  /* ptp_dev_req */

#ifdef HAVE_UNLOCKED_IOCTL
long ptp_dev_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
#else
int ptp_dev_ioctl(struct inode *inode, struct file *filp,
	unsigned int cmd, unsigned long arg)
#endif
{
	struct ptp_dev_info *info = (struct ptp_dev_info *)
		filp->private_data;
	struct Ksz_ptp_info *ptp = info.ptp;
	int err = 0;

	if (_IOC_TYPE(cmd) != DEV_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > DEV_IOC_MAX)
		return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void *) arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void *) arg, _IOC_SIZE(cmd));
	if (err) {
		printk(KERN_ALERT "err fault\n");
		return -EFAULT;
	}
	if (down_interruptible(&info.sem))
		return -ERESTARTSYS;

	err = ptp_dev_req(ptp, 0, (char *) arg, info);
	up(&info.sem);
	return err;
}  /* ptp_dev_ioctl */

ssize_t ptp_dev_read(struct file *filp, char *buf, size_t count,
	loff_t *offp)
{
	struct ptp_dev_info *info = (struct ptp_dev_info *)
		filp->private_data;
	ssize_t result = 0;
	int rc;

	if (!info.read_len) {
		*offp = 0;
		rc = wait_event_interruptible(info.wait_udp,
			0 != info.read_len);

		/* Cannot continue if ERESTARTSYS. */
		if (rc < 0)
			return 0;
	}

	if (down_interruptible(&info.sem))
		return -ERESTARTSYS;

	mutex_lock(&info.lock);
	if (*offp >= info.read_len) {
		info.read_len = 0;
		count = 0;
		*offp = 0;
		goto dev_read_done;
	}

	if (*offp + count > info.read_len) {
		count = info.read_len - *offp;
		info.read_len = 0;
	}

	if (copy_to_user(buf, &info.read_buf[*offp], count)) {
		result = -EFAULT;
		goto dev_read_done;
	}
	if (info.read_len)
		*offp += count;
	else
		*offp = 0;
	result = count;

dev_read_done:
	mutex_unlock(&info.lock);
	up(&info.sem);
	return result;
}  /* ptp_dev_read */

ssize_t ptp_dev_write(struct file *filp, const char *buf, size_t count,
	loff_t *offp)
{
	struct ptp_dev_info *info = (struct ptp_dev_info *)
		filp->private_data;
	ssize_t result = 0;
	size_t size;
	int rc;
	u8 cmd;

	if (!count)
		return result;

	if (down_interruptible(&info.sem))
		return -ERESTARTSYS;

	if (*offp >= info.write_len) {
		result = -ENOSPC;
		goto dev_write_done;
	}
	if (*offp + count > info.write_len)
		count = info.write_len - *offp;
	if (copy_from_user(info.write_buf, buf, count)) {
		result = -EFAULT;
		goto dev_write_done;
	}
	cmd = info.write_buf[0] & 0xf0;
	switch (cmd) {
	case TSM_CMD_GET_GPS_TS:
		size = sizeof(struct tsm_get_gps);
		break;
	case TSM_CMD_DB_GET_TIME:
		size = sizeof(struct tsm_get_time);
		break;
	case TSM_CMD_DB_GET:
		size = sizeof(struct tsm_db);
		break;
	case TSM_CMD_CNF_SET:
		size = sizeof(struct tsm_cfg);
		break;
	case TSM_CMD_CLOCK_SET:
		size = sizeof(struct tsm_clock_set);
		break;
	case TSM_CMD_CLOCK_CORRECT:
		size = sizeof(struct tsm_clock_correct);
		break;
	default:
		dbg_msg("tsm: %x\n", info.write_buf[0]);
		result = count;
		goto dev_write_done;
	}
	if (count < size) {
		result = -EFAULT;
		goto dev_write_done;
	}
	result = size;
	rc = parse_tsm_msg(info, count);
	if (rc)
		result = rc;

dev_write_done:
	up(&info.sem);
	return result;
}  /* ptp_dev_write */

const struct file_operations ptp_dev_fops = {
	.read		= ptp_dev_read,
	.write		= ptp_dev_write,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl	= ptp_dev_ioctl,
#else
	.ioctl		= ptp_dev_ioctl,
#endif
	.open		= ptp_dev_open,
	.release	= ptp_dev_release,
};

struct class *ptp_class;

int init_ptp_device(int dev_major, char *dev_name, char *minor_name)
{
	int result;

	result = register_chrdev(dev_major, dev_name, &ptp_dev_fops);
	if (result < 0) {
		printk(KERN_WARNING "%s: can't get major %d\n", dev_name,
			dev_major);
		return result;
	}
	if (0 == dev_major)
		dev_major = result;
	ptp_class = class_create(THIS_MODULE, dev_name);
	if (IS_ERR(ptp_class)) {
		unregister_chrdev(dev_major, dev_name);
		return -ENODEV;
	}
	device_create(ptp_class, NULL, MKDEV(dev_major, 0), NULL, dev_name);
	device_create(ptp_class, NULL, MKDEV(dev_major, 1), NULL, minor_name);
	return dev_major;
}  /* init_ptp_device */

void exit_ptp_device(int dev_major, char *dev_name)
{
	device_destroy(ptp_class, MKDEV(dev_major, 1));
	device_destroy(ptp_class, MKDEV(dev_major, 0));
	class_destroy(ptp_class);
	unregister_chrdev(dev_major, dev_name);
}  /* exit_ptp_device */

void ptp_set_identity(struct Ksz_ptp_info *ptp, u8 *addr)
{
	memcpy(&ptp->clockIdentity.addr[0], &addr[0], 3);
	ptp->clockIdentity.addr[3] = 0xFF;
	ptp->clockIdentity.addr[4] = 0xFE;
	memcpy(&ptp->clockIdentity.addr[5], &addr[3], 3);
}  /* ptp_set_identity */

void ptp_init(struct Ksz_ptp_info *ptp, u8 *mac_addr)
{
	int i;

	ptp->utc_offset = CURRENT_UTC_OFFSET;
	ptp->get_delay = 100000;
	ptp->set_delay = 100000;
	ptp->delay_ticks = 1;
#ifdef PTP_SPI
	ptp->delay_ticks = 2;
	ptp->access = create_singlethread_workqueue("ptp_access");
#endif
	init_ptp_work(ptp);
	mutex_init(&ptp->lock);
	init_waitqueue_head(&ptp->wait_ts);
	init_waitqueue_head(&ptp->wait_intr);
	INIT_WORK(&ptp->adj_clk, adj_clock);
	INIT_DELAYED_WORK(&ptp->check_pps, ptp_check_pps);
	INIT_DELAYED_WORK(&ptp->update_sec, ptp_update_sec);
	ptp_set_identity(ptp, mac_addr);

	ptp->ports = MAX_PTP_PORT;
	if (!ptp->get_clk_cnt)
		ptp->get_clk_cnt = _get_clk_cnt;
	if (!ptp->test_access_time)
		ptp->test_access_time = _test_access_time;

	ptp->mode = PTP_ENABLE |
		PTP_IPV4_UDP_ENABLE |
		PTP_1STEP;
	ptp->mode |= PTP_IPV6_UDP_ENABLE;
	ptp->mode |= PTP_ETH_ENABLE;
	ptp->cfg = 0;
	ptp->cfg |= PTP_DOMAIN_CHECK;
	ptp->cfg |= PTP_PDELAY_CHECK | PTP_DELAY_CHECK;
	ptp->cfg |= PTP_UNICAST_ENABLE;
	if (ptp->version >= 1) {
		ptp->cfg |= PTP_UDP_CHECKSUM;
		ptp->cfg |= PTP_SYNC_CHECK;
	}
	ptp->def_mode = ptp->mode;
	ptp->def_cfg = ptp->cfg;
	ptp->trig_intr = 0xfff;
	ptp->ts_intr = (TS_PORT2_INT_XDELAY | TS_PORT1_INT_XDELAY);

	ptp->gps_tsi = MAX_TIMESTAMP_UNIT;
	ptp->gps_gpi = DEFAULT_GPS_GPI;
	ptp->pps_gpo = DEFAULT_PPS_GPO;
	ptp->pps_tsi = DEFAULT_PPS_TSI;
	ptp->pps_tso = DEFAULT_PPS_TSO;
	ptp->mhz_gpo = DEFAULT_MHZ_GPO;
	ptp->mhz_tso = DEFAULT_MHZ_TSO;

	for (i = 0; i < MAX_TIMESTAMP_UNIT - 1; i++)
		ptp->events[i].max = 2;
	ptp->events[i].max = MAX_TIMESTAMP_EVENT_UNIT;

	ptp_priv = ptp;
	sprintf(ptp->dev_name[0], "ptp_dev");
	sprintf(ptp->dev_name[1], "ptp_event");
	ptp->dev_major = init_ptp_device(0, ptp->dev_name[0],
		ptp->dev_name[1]);

#ifdef CONFIG_PTP_1588_CLOCK
	micrel_ptp_probe(ptp);
#endif
}  /* ptp_init */

void ptp_exit(struct Ksz_ptp_info *ptp)
{
	exit_ptp_work(ptp);
#ifdef PTP_SPI
	if (ptp->access) {
		destroy_workqueue(ptp->access);
		ptp->access = NULL;
	}
#endif
	if (ptp->dev_major >= 0)
		exit_ptp_device(ptp->dev_major, ptp->dev_name[0]);

#ifdef CONFIG_PTP_1588_CLOCK
	micrel_ptp_remove(ptp);
#endif
}  /* ptp_exit */

enum {
	PROC_SET_PTP_FEATURES,
	PROC_SET_PTP_OVERRIDES,
	PROC_SET_PTP_VID,
};

ssize_t sysfs_ksz_ptp_read(struct Ksz_ptp_info *ptp, int proc_num, ssize_t len,
	char *buf)
{
	switch (proc_num) {
	case PROC_SET_PTP_FEATURES:
		len += sprintf(buf + len, "%08x:\n", ptp->features);
		len += sprintf(buf + len, "\t%08x = adjust hack\n",
			PTP_ADJ_HACK);
		len += sprintf(buf + len, "\t%08x = adjust sec\n",
			PTP_ADJ_SEC);
		len += sprintf(buf + len, "\t%08x = pdelay hack\n",
			PTP_PDELAY_HACK);
		break;
	case PROC_SET_PTP_OVERRIDES:
		len += sprintf(buf + len, "%08x:\n", ptp->overrides);
		len += sprintf(buf + len, "\t%08x = PTP port forwarding\n",
			PTP_PORT_FORWARD);
		len += sprintf(buf + len, "\t%08x = PTP port TX forwarding\n",
			PTP_PORT_TX_FORWARD);
		len += sprintf(buf + len, "\t%08x = PTP verify timestamp\n",
			PTP_VERIFY_TIMESTAMP);
		len += sprintf(buf + len, "\t%08x = PTP zero reserved field\n",
			PTP_ZERO_RESERVED_FIELD);
		len += sprintf(buf + len, "\t%08x = PTP check system time\n",
			PTP_CHECK_SYS_TIME);
		break;
	case PROC_SET_PTP_VID:
		len += sprintf(buf + len, "0x%04x\n", ptp->vid);
		break;
	}
	return len;
}

void sysfs_ksz_ptp_write(struct Ksz_ptp_info *ptp, int proc_num, int num,
	const char *buf)
{
	int changes;

	switch (proc_num) {
	case PROC_SET_PTP_FEATURES:
		if ('0' != buf[0] || 'x' != buf[1])
			sscanf(buf, "%x", &num);
		changes = ptp->features ^ num;
		ptp->features = num;
#ifdef PTP_PROCESS
		if ((changes & (PTP_SYNT | PTP_SIM_2_STEP))) {
#ifdef PTP_2_STEP
			if (num & PTP_SIM_2_STEP) {
				ptp->sim_2_step = true;
				ptp->mode &= ~PTP_1STEP;
			} else {
				ptp->sim_2_step = false;
				ptp->mode |= PTP_1STEP;
			}
#endif
			if (num & (PTP_SYNT | PTP_SIM_2_STEP)) {
				ptp_init_state(ptp);
				if (num & PTP_SYNT) {
					ptp->sim = 1;
					ptp->I = 0;
					ptp->KP = 50;
					ptp->KI = 5;
				}
			} else {
				ptp_exit_state(ptp);
				dbg_msg("exit ptp\n");
			}
		}
#endif
		break;
	case PROC_SET_PTP_OVERRIDES:
		if ('0' != buf[0] || 'x' != buf[1])
			sscanf(buf, "%x", &num);
		changes = ptp->overrides ^ num;
		if ((changes & PTP_CHECK_SYS_TIME) &&
				(ptp->overrides & PTP_CHECK_SYS_TIME))
			ptp->first_sec = 0;
		ptp->overrides = num;
		break;
	case PROC_SET_PTP_VID:
		ptp->vid = num;
		break;
	}
}

struct ptp_reg_ops ptp_reg_ops = {
	.read			= ksz_ptp_read,
	.write			= ksz_ptp_write,

	.get_time		= get_ptp_time,
	.set_time		= set_ptp_time,
	.adjust_time		= adjust_ptp_time,
	.adjust_sync_time	= adjust_sync_time,

	.rx_off			= ptp_rx_off,
	.rx_restart		= ptp_rx_restart,
	.rx_event		= ptp_rx_event,
	.rx_cascade_event	= ptp_rx_cascade_event,
	.read_event		= ksz_ptp_read_event,

	.tx_off			= ptp_tx_off,
	.tx_event		= ptp_tx_event,
	.pps_event		= ptp_pps_event,
	.ptp_10MHz		= ptp_10MHz,
	.tx_cascade		= ptp_tx_cascade,

	.start			= ptp_start,
};

struct ptp_ops ptp_ops = {
	.acquire		= ptp_acquire,
	.release		= ptp_release,

	.init			= ptp_init,
	.exit			= ptp_exit,

	.stop			= ptp_stop,
	.set_identity		= ptp_set_identity,

	.check_msg		= check_ptp_msg,
	.update_msg		= update_ptp_msg,
	.get_rx_tstamp		= get_rx_tstamp,
	.get_tx_tstamp		= get_tx_tstamp,
	.hwtstamp_ioctl		= ptp_hwtstamp_ioctl,
	.ixxat_ioctl		= ixxat_ptp_ioctl,
	.dev_req		= ptp_dev_req,
	.proc_intr		= proc_ptp_intr,
	.get_ts_info		= ptp_get_ts_info,

	.sysfs_read		= sysfs_ksz_ptp_read,
	.sysfs_write		= sysfs_ksz_ptp_write,

	.drop_pkt		= ptp_drop_pkt,
};

#endif

#endif//PTP
#endif //KSZ8794

#endif
