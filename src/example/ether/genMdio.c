/* MDIO with bitbang for STM103
 * CHONGHO YOON
 * 2015.07.30
 *                     MDIO   +-------------+
 *                 +--------- |STM32F103MCU |
 *                            +-------------+
 * [GPIOs]
 * MDIO : PC6
 * MDC  : PD1
 * --EXTRAS----
 * (OLED's I2C)
 * I2CSCL : PB2
 * I2CSDA : PB3
 * nIRQ(PHY) : PD0
 */
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "yInc.h"
#include "example/ether/include/ymdio.h"
#include "cmdline.h"

char *mdio_phystring[] ={
		"GENERALPHY(0)",
		"IP101G(1)",
		"RTL8201(2)"
		"KSZ9021RN(3)",
		"BROADCOM(20)",
		"RTL9K(21)",
		"LAN9355SWITCH_BR_BR(30)",
		"LAN9355SWITCH_RTL_BR(31)",
		"LAN9355SWITCH_RTL_RTL(32)",
		"RTL9047SWITCH(33)"
};
tCmdLineEntry *gp_sCmdTable;

int GeneralPhy_Cmd_help(int argc, char *argv[]);
int GeneralPhy_Cmd_read(int argc, char *argv[]);
int GeneralPhy_Cmd_readAll(int argc, char *argv[]);
int GeneralPhy_Cmd_write(int argc, char *argv[]);
int GeneralPhy_Cmd_showstatus(int argc, char *argv[]);
int GeneralPhy_Cmd_Softreset(int argc, char *argv[]);
int GeneralPhy_Cmd_SetPhyAddrAndInitConfig(int argc, char *argv[]);
int GeneralPhy_Cmd_SoftReset(int argc, char *argv[]);
int GeneralPhy_Cmd_SelectPhy(int argc, char *argv[]);

//BroadR
extern int stmBroadR_Cmd_setms(int argc, char *argv[]);
extern int stmBroadR_Cmd_setNormalReverse(int argc, char *argv[]);
//RTL9K
extern int RTL9Kmc_Cmd_setms(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_read(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_write(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_readAll(int argc, char *argv[]);
//PHY SPECIFIC
extern tCmdLineEntry g_sCmdTable_GeneralPhy[];
extern tCmdLineEntry g_sCmdTable_IP101G[];
extern tCmdLineEntry g_sCmdTable_RTL8201[];
extern tCmdLineEntry g_sCmdTable_RTL9K[];
extern tCmdLineEntry g_sCmdTable_BROADCOM[];
extern tCmdLineEntry g_sCmdTable[];

extern char g_cCmdBuf[CMD_BUF_SIZE];
extern void yConfirmLEDBlink(void);

extern u16 mdio_read_clause45(u32 PhyAddr, u32 mmdeviceAddr, u32 PhyReg, u8 f_preamblesuppressed);
extern u16 mdio_read_clause45overClause22(u32 PhyAddr, u32 mmdeviceAddr, u32 PhyReg, u8 f_preamblesuppressed);
extern u16 mdio_write_clause45(u32 PhyAddr, u32 mmdeviceID, u32 PhyReg, u8 f_preamblesuppressed, u32 val);
extern u16 mdio_write_clause45overClause22(u32 PhyAddr, u32 mmdeviceAddr, u32 PhyReg, u8 f_preamblesuppressed, u32 val);

extern int dtMdioCmd_help(int argc, char *argv[]);
extern int dtMdioCmd_read(int argc, char *argv[]);
extern int dtMdioCmd_readSpan(int argc, char *argv[]);
extern int dtMdioCmd_readSQI(int argc, char *argv[]); //BroadR specific for reading SQI
extern int dtMdioCmd_write(int argc, char *argv[]);
extern int dtMdioCmd_softreset(int argc, char *argv[]);
extern void OzOLED_init();
extern void OzOLED_sendCommand(unsigned char command);
extern void OzOLED_sendData(unsigned char Data);
extern void OzOLED_printString(const char *String, u8 X, u8 Y, u8 numChar);
//
int stmBr_RTL_Switch9355_Cmd_help(int argc, char *argv[]);


//================================================================================
struct _PhyMdioModule PhyMdioModule;

struct _PhyREG GeneralPhyBasicReg[]={
		 //--- STANDARD REG.
		{0,0,"BMCR",""},//"basic mode control reg"},
		{1,0,"BMSR",""},//"basic mode status reg"},
		{2,0,"PHYID1",""},//"Vendor OUI reg"},
		{3,0,"PHYID2",""},//"Vendor OUI+ModelNum+RevNum reg"},
		{4,0,"ANAR",""},//"Auto-Negotiation Advertising reg"},
		{5,0,"ANLPAR",""},//"Auto-Negotiation LP Ability reg"},
		{6,0,"ANEXP",""},//"Auto-Negotiation Expansion reg"},
		{7,0,"ANNXP",""},//"Auto-Negotiation NP Transmit reg"},
		{8,0,"ANLPNPA",""},//"Auto-Negotiation NP Receive reg"},
		{9,0,"MSCR",""},//"Master/Slave Control Reg."},
		{10,0,"MSSR",""},//"Master/Slave Status Reg."},
		{11,0,"PSECR",""},//"PSE Control Reg"},
		{12,0,"PSESR",""},//"PSE Status Reg"},
		{13,0,"MMDACR",""},//"MMD Access Control Reg."},
		{14,0,"MMDAADR",""},//"MMD Access Address/Data Reg."},
		{15,0,"EXTSTS",""}//"Extended Status Reg."}
};

struct _PhyREG GeneralPhyBasicReg45[]={
		 //--- STANDARD REG.
		{0,0,"PMA/PMD/CTRL1",""},//"PMA/PMD Control Reg(1.0)"},
		{1,0,"PMA/PMD/STS1",""},//"PMA/PMD Status1 Reg(1.1)"},
		{2,0,"PMA/PMD/ID1",""},//"PMA/PMD Device Identifier1"},
		{3,0,"PMA/PMD/ID2",""},//"PMA/PMD Device Identifier2"},
		{4,0,"PMA/PMD/SPD",""},//"PMA/PMD Speed Ability"},
		{5,0,"PMA/PMD/DevPkg1",""},//"PMA/PMD Devices In Package1"},
		{6,0,"PMA/PMD/DevPkg2",""},//"PMA/PMD Devices In Package2"},
		{7,0,"PMA/PMD/CTRL2",""},//"PMA/PMD Control Reg(1.7)"},
		{8,0,"PMA/PMD/STS2",""},//"PMA/PMD Status 2 Reg(1.8)"},
		{9,0,"PMA/PMD/TXDIS",""},//"PMA/PMD Transmit Disable Reg(1.9)"},
		{10,0,"PMA/PMD/SD",""},//"PMA/PMD Receive Signal Detect(1.10)"},
		{11,0,"PMA/PMD/EAR",""},//"PMA/PMD Extended Ability(1.11)"},
		{12,0,"10GEPONAR",""},//"10G-EPON PMA/PMD Ability(1.12)"},
		{13,0,"MMDACR",""},//"MMD Access Control Reg."},
		{14,0,"MMDAADR",""},//"MMD Access Address/Data Reg."},
		{15,0,"EXTSTS",""}//"Extended Status Reg."},
};

//========== SPECIFIC PHYS ==========================
#if (PHYIS == PHYIS_KSZ9021RN)

struct _PhyREG KSZ9021SpecificReg[]={
		//--- KSZ9021 Specific REG.
		{16,0,"RSVD","(Rsvd :0x10,0)"},
		{17,0,"REMLB","(RemoteLoopback,LED mode :0x11,0)"},
		{18,0,"LNKMD","(LinkMD-CableDiag :0x12,0)"},
		{19,0,"DPPS","(Digital PMA/PCS Status :0x13,0)"},
		{20,0,"RSVD","(Rsvd :0x14,0)"},
		{21,0,"RXERC","(Rx Error Conter :0x15,0)"},
		{22,0,"RSVD","(Rsvd :0x16,0)"},
		{23,0,"RSVD","(Rsvd :0x17,0)"},
		{24,0,"RSVD","(Rsvd :0x18,0)"},
		{25,0,"RSVD","(Rsvd :0x19,0)"},
		{26,0,"RSVD","(Rsvd :0x1a,0)"},
		{27,0,"INTCS","(Interrupt Control/Status :0x1b,0)"},
		{28,0,"RSVD","(Rsvd :0x1c,0)"},
		{29,0,"RSVD","(Rsvd :0x1d,0)"},
		{30,0,"RSVD","(Rsvd :0x1e,0)"},
		{31,0,"PCR","(Phy Control Reg:0x1f,0)"},
		//extended registers
		{0x100,1,"CMC","(Common Control Reg:0x100,e)"},
		{0x101,1,"STRPS","(Strap Status Reg:0x101,e)"},
		{0x102,1,"OPMSOV","(Operation Mode Strap Override Reg:0x102,e)"},
		{0x103,1,"OPMSST","(Operation Mode Strap Status Reg:0x103,e)"},
		{0x104,1,"RGMIICCSKEW","(RGMII Clock and Control Pad Skew Reg:0x104,e)"},
		{0x105,1,"RGMIIRXSKEW","(RGMII RX Data Pad Skew Reg:0x105,e)"},
		{0x106,1,"RGMIITXSKEW","(RGMII TX Data Pad Skew Reg:0x106,e)"},
		{0x107,1,"ATR","(Analog Test Reg:0x107,e)"},
};
#endif

#if (PHYIS == PHYIS_RTL9047SWITCH)
struct RTL9KSPECIALREG RTL9047AASpecialReg[]={
		 //--- STANDARD REG.
		{0xdc0c,"OPCR1","(OPFSM control reg1:dc0c)"},
		{0xdd00,"OPCR2","(OPFSM control reg2:dd00)"},
		{0xdd02,"OPCR3","(OPFSM control reg3:dd02)"},
		{0xdd08,"OPINSR1","(OPFSM interrupt status reg1:dd08)"},
		{0xdd0c,"OPINER1","(OPFSM interrupt enable reg1:dd0c)"},
		{0xdd0e,"OPINMR1","(OPFSM interrupt mask reg1:dd0e)"},

		{0xdd10,"OPINSR2","(OPFSM interrupt status reg2:dd10)"},
		{0xdd14,"OPINER2","(OPFSM interrupt enable reg2:dd14)"},
		{0xdd16,"OPINMR2","(OPFSM interrupt mask reg2:dd16)"},

		{0xdd18,"OPINSR3","(OPFSM interrupt status reg3:dd18)"},
		{0xdd1c,"OPINER3","(OPFSM interrupt enable reg3:dd1c)"},
		{0xdd1e,"OPINMR3","(OPFSM interrupt mask reg3:dd1e)"},

		{0xdd20,"OPCR4","(OPFSM control reg4:dd20)"},
		{0,0x00,0x00}
};
/*========================================================
* [RTL9047AA Switch]
*                 |   MDIO   +------------+
*                 +--------- |LM3S811 MCU |
*                            |            |
*                            +------------+
*  [CONTROL INTERFACE]
*  PHYAddr RTL9047AA : 0x18
*  MDIO/MDC Controlled by LM3S811 MCU.
*
*  [READ] (ex) read register of 0x123456 (3 bytes)
*  <0x18><0><0x0000> : Set Data Register LSB 0
*  <0x18><2><0x0000> : Set Data Register MSB 0
*  <0x18><4><0x3456> : Set Register LSB
*  <0x18><6><0x8012> : Set Flags(to Read(80)) and Register LSB(12)
*  <WAIT>
*  <0x18><0><READ 2 LSB BYTES> : Get Data Register LSB 2 bytes
*  <0x18><2><READ 2 MSB BYTES> : Get Data Register MSB 2 bytes
*
*  [WRITE] (ex) write data of 0x12345678(4 bytes) in register of 0x123456: (3 bytes)
*  <0x18><0><0x0000> : Set Data Register LSB 0x5678
*  <0x18><2><0x0000> : Set Data Register MSB 0x1234
*  <0x18><4><0x3456> : Set Register LSB
*  <0x18><6><0x8112> : Set Flags(to Write(=81)) and Register LSB(12)
*  <WAIT>
*  */
/*
struct _PhyREG RTL9047AAReg[]={
		 //--- INDIRECT REG. for embedeed 8051 CPU core
		{0x000000, //unsigned long regaddr; //actually 3 bytes with LSB 2bits will be ignored.
				0, //NULL
				"ICR8051",//RegName
				"(INDIRECT_Control_Reg:0x000000)"}, //Detail
		{0x000004,0,"IDR0_8051","(INDIRECT_Data_Reg0:0x000004)"},
		{0x000008,0,"IDR1_8051","(INDIRECT_Data_Reg1:0x000008)"},
		{0x00000C,0,"IDR2_8051","(INDIRECT_Data_Reg2:0x00000C)"},
		//...
		{0x00002C,0,"IOSR","(Indirect_Occupy_Status_Reg:0x00002C)"},
		{0x000030,0,"ICR_CPU","(Indirect_Control_Reg:0x000030)"},
		//...
		{0x020000,0,"RGCR0","(Reset_Global_Control_Reg0:0x020000)"},
		{0x020004,0,"RGCR1","(Reset_Global_Control_Reg1:0x020004)"},
		//
		{0x040000,0,"P0PCR","(Port0_Property_Configure_Reg:0x040000)"},
		{0x040004,0,"P1PCR","(Port1_Property_Configure_Reg:0x040004)"},
		{0x040008,0,"P2PCR","(Port2_Property_Configure_Reg:0x040008)"},
		{0x04000c,0,"P3PCR","(Port3_Property_Configure_Reg:0x04000c)"},
		{0x040010,0,"P4PCR","(Port4_Property_Configure_Reg:0x040010)"},
		{0x040014,0,"P5PCR","(Port5_Property_Configure_Reg:0x040014)"},
		{0x040018,0,"P6PCR","(Port6_Property_Configure_Reg:0x040018)"},
		{0x04001c,0,"P7PCR","(Port7_Property_Configure_Reg:0x04001c)"},
		//
		{0x040020,0,"P0LSR","(Port0_LinkStatus_Reg:0x040020)"},
		{0x040024,0,"P1LSR","(Port1_LinkStatus_Reg:0x040024)"},
		{0x040028,0,"P2LSR","(Port2_LinkStatus_Reg:0x040028)"},
		{0x04002c,0,"P3LSR","(Port3_LinkStatus_Reg:0x04002c)"},
		//...
		{0x040040,0,"SMICR","(SMI_Control_Reg:0x040040)"},
		{0x040044,0,"PRACR0","(PHY_REG_ACCESS_Control_Reg0:0x040044)"},
		{0x040048,0,"PRACR1","(PHY_REG_ACCESS_Control_Reg1:0x040048)"},

		//Ch.3 VLAN

		//Ch.4 L2Switch

		//Ch. 5 ACL

		//Ch. 6 QoS

		//Ch. 7 Network Monitoring -- Mirroring

		//Ch. 8 MAC Control

		//Ch. 9 Counters

		//Ch. 10 Interrups/Gpio

		//Ch. 11 LEDs

		//Ch. 12 AVB
		{0x700000,0,"EGC","(EAV_Global_Control_Reg:0x700000)"},
		//Ch. 13 OP-FSM

		//Ch. 14 PHY : Refers 14.1 to access PHY registers.

		 //--- STANDARD REG.
		{0,0,"BMCR","(basic mode control reg:0,0)"},
		{1,0,"BMSR","(basic mode status reg:1,0)"},
		{2,0,"PHYID1:2,0",""},
		{3,0,"PHYID2:3,0",""},
		{9,0,"PHYCR","(phy control reg:9,0)"},
		{10,0,"PHYSR","(phy status reg:a,0)"},
		//--- RTL9K GENERAL REG.
		{0x10,0xa42,"PHYSFR","(phy status sub-flag reg,:10,a42)"},
		{0x11,0xa42,"RTCTCR","(rtct control reg:11,a42)"},
		{0x12,0xa42,"GINER","(general interrupt enable reg:12,a42)"},
		{0x14,0xa42,"GINMR","(general interrupt mask reg:14,a42)"},
		{0x15,0xa42,"SLPCR","(sleep control reg:15,a42)"},
		{0x18,0xa43,"PHYCR1","(phy specifici control reg1:18,a43)"},//0xa42 ???? -- Errata Found.
		{0x1B,0xa43,"PHYSRADR","(phy sram address reg:1b,a43)"},
		{0x1C,0xa43,"PHYSRDAT","(phy sram data reg:1c,a43)"},//Errta Found
		{0x1D,0xa43,"GINSR","(general interrupt status reg:1d,a43)"},
		{0x1F,0xa43,"PAGSR","(page select reg:1f,a43)"},
		{0x15,0xa47,"GPSFR","(general purpose sub-flag reg:15,a47)"},
		{0x14,0xa5a,"SLPCAP","(sleep capability reg:14,a5a)"},
		{0x10,0xa88,"CLENR","(cable length reg:10,a88)"},
		{0x11,0xd01,"SSCCR","(ssc control reg:11,d01)"},
		{0x10,0xd04,"LEDCR","(led control reg:10,d04)"},
		{0x15,0xd42,"LED_PTP","(led/ptp-gpio select reg:15,d42)"}, //22
		//PTP Registers (8.3)
		{0x10,0xe40,"PTP_CTL","(ptp control reg:10,e40)"} //23
		//TBD...

};
*/
#endif

// For MDIO Commands ======================================================
tCmdLineEntry g_sCmdTable_GENERALPHY[] =
{
#if (PHYIS == PHYIS_LAN9355SWITCH_RTL_BR)
    { "h",      stmBr_RTL_Switch9355_Cmd_help,   "  : Display list of commands" },
    { "?",      stmBr_RTL_Switch9355_Cmd_help,   "  : alias for help" },
#else
    { "h",      GeneralPhy_Cmd_help,   "  : Show commands" },
    { "?",      GeneralPhy_Cmd_help,   "  : alias for help" },
#endif
#if (PHYIS == PHYIS_RTL9K)
    { "ms",     RTL9Kmc_Cmd_setms,   " : set as master/slave(RTL9K) <1=master/0=slave>" },
    { "r",      RTL9Kmc_Cmd_read,   "  : read <phy addr> <reg addr> <page>" },
    { "ra",     RTL9Kmc_Cmd_readAll,   " : readall <phy addr>" },
#elif (PHYIS == PHYIS_BROADCOM)
    { "ms",     stmBroadR_Cmd_setms,   " : set as master/slave(BR89810) <1=master/0=slave>" },
    { "nr",     stmBroadR_Cmd_setNormalReverse,   " : set MII mode (BR89810) <1=normal/0=reverse>" },
    { "r",      GeneralPhy_Cmd_read,   "  : read <reg addr>" },
#elif (PHYIS == PHYIS_LAN9355SWITCH_RTL_BR)
    { "ms",     RTL9Kmc_Cmd_setms,   " : set as master/slave(RTL9K) <1=master/0=slave>" },
    { "r",      RTL9Kmc_Cmd_read,   "  : read <phy addr> <reg addr> <page>" },
    { "ra",     RTL9Kmc_Cmd_readAll,   " : readall <phy addr>" },
#else
    { "r",      GeneralPhy_Cmd_read,   "  : read [page/mmd] <reg addr>" },
    { "ra",     GeneralPhy_Cmd_readAll,   " : readAllbasic" },
#endif

#if (PHYIS == PHYIS_RTL9K)
    { "w",      RTL9Kmc_Cmd_write,   " : write <phy addr> <reg addr> <page> <value>" },
#else
    { "w",      GeneralPhy_Cmd_write,   "  : write [page/mmd] <reg addr> <value>" },
#endif
    //{ "ss",     IP101G_Cmd_showstatus,   " : showstatus <phy addr>" },
    { "sr",     GeneralPhy_Cmd_SoftReset,   " : softreset " },
    { "sa",     GeneralPhy_Cmd_SetPhyAddrAndInitConfig,   " : setPHYAD <phy addr> [45][4522]" }, //45=clause45, 4522=Annex22D
    { "sp",     GeneralPhy_Cmd_SelectPhy,   " : selectPhy <phylist>" },
    { 0, 0, 0 }
};

#if (PHYIS == PHYIS_IP101G) //paging
extern int IP101G_Cmd_read(int argc, char *argv[]);
extern int IP101G_Cmd_readAll(int argc, char *argv[]);
extern int IP101G_Cmd_write(int argc, char *argv[]);
tCmdLineEntry g_sCmdTable_IP101G[] =
{
	{ "h",      GeneralPhy_Cmd_help,   "    : Display list of commands" },
	{ "?",      GeneralPhy_Cmd_help,   "    : alias for help" },
    { "r",      IP101G_Cmd_read,   "  : read <phy addr> <reg addr> [page]" },
    { "ra",     IP101G_Cmd_readAll,   " : readAll <phy addr>" },
    { "w",      IP101G_Cmd_write,   " : write <phy addr> <reg addr> [page] <value>" },
    { "sr",     GeneralPhy_Cmd_SoftReset,   " : softreset <phy addr>" },
    { "sa",     GeneralPhy_Cmd_SetPhyAddrAndInitConfig,   " : setPhyAddress <phy addr> [45][4522]" }, //45=clause45, 4522=Annex22D
    { 0, 0, 0 }
};
#elif (PHYIS == PHYIS_RTL8201) //with paging
//extern struct _PhyREG RTL8211SpecificReg[];
extern struct _PhyREG RTL8201SpecificReg[];
extern int RTL8201_Cmd_read(int argc, char *argv[]);
extern int RTL8201_Cmd_readAll(int argc, char *argv[]);
extern int RTL8201_Cmd_write(int argc, char *argv[]);
extern int RTL8201_Cmd_help(int argc, char *argv[]);
extern int RTL8201_Cmd_SetPhyAddrAndInitConfig(int argc, char *argv[]);
extern int RTL8201_Cmd_softreset(int argc, char *argv[]);


tCmdLineEntry g_sCmdTable_RTL8201[] =
{
	{ "h",      GeneralPhy_Cmd_help,   " : Display list of commands" },
	{ "?",      GeneralPhy_Cmd_help,   " : alias for help" },
    { "r",      RTL8201_Cmd_read,   "  : read  <reg addr> [page]" },
    { "ra",     RTL8201_Cmd_readAll,   " : readAll" },
    { "w",      RTL8201_Cmd_write,   "   : write <reg addr> [page] <value>" },
    { "sr",     GeneralPhy_Cmd_SoftReset,   " : softreset " },
    { "sp",     GeneralPhy_Cmd_SetPhyAddrAndInitConfig,   " : setPhyAddress <phy addr>" },
    { 0, 0, 0 }
};
#endif

#if (PHYIS == PHYIS_RTL9K)
extern int RTL9Kmc_Cmd_read(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_readAll(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_write(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_setms(int argc, char *argv[]);
extern int RTL9Kmc_Cmd_showstatus(int argc, char *argv[]);

tCmdLineEntry g_sCmdTable_RTL9K[] =
{
	{ "h",      GeneralPhy_Cmd_help,   "    : Display list of commands" },
	{ "?",      GeneralPhy_Cmd_help,   "    : alias for help" },
    { "r",      RTL9Kmc_Cmd_read,   "  : read <phy addr> <reg addr> <page>" },
    { "ra",     RTL9Kmc_Cmd_readAll,   " : readAll <phy addr>" },
    { "w",      RTL9Kmc_Cmd_write,   " : write <phy addr> <reg addr> <page> <value>" },
    { "ms",     RTL9Kmc_Cmd_setms,   " : set as master/slave(RTL9K) <1=master/0=slave>" },
    { "ss",     RTL9Kmc_Cmd_showstatus,   " : showstatus <phy addr>" },
    { "sr",     GeneralPhy_Cmd_SoftReset,   " : softreset <phy addr>" },
    { 0, 0, 0 }
};
#endif

#if (PHYIS == PHYIS_RTL9047SWITCH)
int RTL9047AASwitch_Cmd_read(int argc, char *argv[]);
int RTL9047AASwitch_Cmd_readAll(int argc, char *argv[]);
int RTL9047AASwitch_Cmd_write(int argc, char *argv[]);
int RTL9047AASwitch_Cmd_SetPhyAddrAndInitConfig(int argc, char *argv[]);
int RTL9047AASwitch_Cmd_softreset(int argc, char *argv[]);

tCmdLineEntry g_sCmdTable_RTL9047SWITCH[] =
{
	{ "h",      GeneralPhy_Cmd_help,   "    : Display list of commands" },
	{ "?",      GeneralPhy_Cmd_help,   "    : alias for help" },
    { "r",      RTL9047AASwitch_Cmd_read,   "  : read <reg addr> " },
    { "ra",     RTL9047AASwitch_Cmd_readAll,   " : readAll " },
    { "w",      RTL9047AASwitch_Cmd_write,   " : write  <reg addr> <value>" },
    { "sr",     RTL9047AASwitch_Cmd_softreset,   " : softreset " },
    { "sa",     RTL9047AASwitch_Cmd_SetPhyAddrAndInitConfig,   " : setPhyAddress <phy addr>" },
    { "sp",     GeneralPhy_Cmd_SelectPhy,   " : selectPhy <phylist>" },
    { 0, 0, 0 }
};
#endif

//==========================================================================
//*****************************************************************************
//PB13 : MDC (was SCK)
//PB15 : MDIO (was MOSI)
void stmMdioInitPins(void)
{
	/* Set MDIO and MDC pin as output */
    // Set MDC Low @ Init
	GPIO_InitTypeDef GPIO_InitS;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitS.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitS.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitS.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitS);

	GPIO_ResetBits(GPIOB, GPIO_Pin_13); //MDC LOW
}

u32 stmMdio_GetBasicRegister(u8 MMDid,u16 reg){
	u32 retval;

	if(PhyMdioModule.clause45 == 0x45){
		if (MMDid == 0) {printf("Err\r\n");	return 0;}
		//retval = mdio_read_clause45(PhyMdioModule.phyaddr, MMDid, reg, PhyMdioModule.f_preamblesuppressed);
	}else if(PhyMdioModule.clause45 == 0x4522){
		if (MMDid == 0) {printf("Err\r\n");	return 0;}
		//retval = mdio_read_clause45overClause22(PhyMdioModule.phyaddr, MMDid, reg, PhyMdioModule.f_preamblesuppressed);
	}else{
		if(PhyMdioModule.platform == PHYIS_RTL9047SWITCH){
			retval = stmMdioRead(PhyMdioModule.phyaddr, reg); //read reg
		}else{
			retval = stmMdioRead(PhyMdioModule.phyaddr, reg); //read reg
		}
	}
	return retval;
}

u32 yGeneralPhy_SetBasicRegister(u8 page_MMDid,u16 reg, u16 value){
	u32 retval;
	if(PhyMdioModule.clause45 == 0x45){
		//retval = mdio_write_clause45(PhyMdioModule.phyaddr, page_MMDid, reg, PhyMdioModule.f_preamblesuppressed, value);
	}else if(PhyMdioModule.clause45 == 0x4522){
		//retval = mdio_write_clause45overClause22(PhyMdioModule.phyaddr, page_MMDid, reg, PhyMdioModule.f_preamblesuppressed,value);
	}else
		mdio_write(PhyMdioModule.phyaddr, reg, value); //Write reg
	return retval;
}



//=== KSZ9021 Extended Registers ==================================================================
u16 yKSZ9021_GetExtendedReg(u16 reg){
	u16 retval;

	mdio_write(PhyMdioModule.phyaddr, 0x0b, reg);
	retval = stmMdioRead(PhyMdioModule.phyaddr, 0x0d);
}
u16 yKSZ9021_SetExtendedReg(u16 reg, u16 value){

	mdio_write(PhyMdioModule.phyaddr, 0x0b, reg | 0x8000);
	mdio_write(PhyMdioModule.phyaddr, 0x0c, value);
	return value;
}

void yKSZ9021_Config(){

	printf("SpecialConfigForKSZ9021(0x104(260), 0xf000) for RXC Skew \r\n");
	//yKSZ9021_SetExtendedReg(0x104, 0xf000); //set RXC Skew
	//yKSZ9021_SetExtendedReg(0x104, 0xf0f0); //set RXC and TXC Skew
	//yKSZ9021_SetExtendedReg(0x104, 0x00f0);   //set TXC Skew


	delayms(500);
}

//=========== RTL Extended Registers=========================================
/*
extern void yRTL8211_Config(u8 phyad, char *phyname);

u16 yRTL_GetExtendedReg( u8 reg, u16 page){
	u16 retval;
	mdio_write(PhyMdioModule.phyaddr, 31, page); //Write Reg 31. Data = 0x0xyz page
	retval = stmMdioRead(PhyMdioModule.phyaddr, reg); //read reg
	mdio_write(PhyMdioModule.phyaddr, 31, 0x0000); //Write Reg 31. Data = 0x0000 to switch back to Standard Reg.
	return retval;
}

u32 yRTL_SetExtendedReg(u8 reg, u16 page, u16 value){
	u16 retval;
	mdio_write(PhyMdioModule.phyaddr, 31, page); //Write Reg 31. Data = 0x0xyz page
	mdio_write(PhyMdioModule.phyaddr, reg, value); //Write reg
	mdio_write(PhyMdioModule.phyaddr, 31, 0x0000); //Write Reg 31. Data = 0x0000 to switch back to Standard Reg.
	return retval;
}
*/
/*
//====================RTL9047AASwitch===================
//See 7.9.5
//{sa <phy addr>" },
int RTL9047AASwitch_Cmd_SetPhyAddrAndInitConfig(int argc, char *argv[]){
	u32 reg;
	u32 val1, val2;
	u16 page;
	u16 temp16;

	PhyMdioModule.f_preamblesuppressed = 0;

	if(argc==2){
		PhyMdioModule.clause45 =	0;
		PhyMdioModule.phyaddr =	(u8)strtol(argv[1], NULL, 16);
		PhyMdioModule.phyaddr_set_done = 1;
		//PhyMdioModule.PhyRegs.pBasicReg = RTL9047AAReg;//GeneralPhyBasicReg;
		PhyMdioModule.phyname = "RTL9047 Switch";
		PhyMdioModule.phyid32 = (val1 << 16) | val2;
		printf("Set %s Phy Addr(0x%02x)/PHYID1/PHYID2(=0x%04x/%04x)\r\n", PhyMdioModule.phyname, PhyMdioModule.phyaddr, val1, val2);
		yConfirmLEDBlink();
		return 0;
	}

	//errors
	yConfirmLEDBlink();
	printErr();

	return -1;

}
u32 RTL9047AASwitch_GetNormalRegister(u32 reg){
	u32 ret32;
	u32 retH32, retL32;
	u32 regH16, regL16;
	u16 regData16;
	u32 timeout;

	//reg = reg >> 2;
	regL16 = reg & 0x0000ffff;
	regH16 = reg >> 16;
	regH16 = regH16 & 0x00ff; //regH16 = regH16 & 0xffff;

	mdio_write(0x18, 0, 0x0000);//set 0x0000 on Reg0
	mdio_write(0x18, 2, 0x0000);//set 0x0000 on Reg2
	mdio_write(0x18, 4, regL16);//set L16 of given reg on Reg4
	mdio_write(0x18, 6, (regH16 & 0x00ff) | 0x8000 ); //Read command | MSB

	//somedelay(1);//delayms(1);
	//The following 2 lines are needless?
	//retL32 = (u32)stmMdioRead(0x18, 4); //??
	//retH32 = (u32)stmMdioRead(0x18, 6); //??

//	delayms(10);
	timeout = 0;
	while(1){
			regData16 = stmMdioRead(0x18, 6);
			if((regData16 & 0x8000) == 0)
				break;
			else
				timeout++;

			if(timeout>0x500)
			{
				printf("Read Timeout Error\r\n");
				return 0xffffffff;
			}
	}
	retL32 = 0x0000;
	retH32 = 0x0000;
	retL32 = (u32) stmMdioRead(0x18, 0); //read LSB
	retH32 = (u32) stmMdioRead(0x18, 2); //read MSB
	ret32 = retL32 | (retH32<<16);

	return ret32;
}

u32 RTL9047AASwitch_SetNormalRegister(u32 reg, u32 value){
	u32 retval;
	u32 timeout;
	u16 regData;

	//Value
	mdio_write(0x18,0, (u16)(value & 0x0000ffff));//LSB
	mdio_write(0x18,2, (u16)((value & 0xffff0000)>>16));//MSB
	//RegAddr with accessEn and WriteOperation.
	mdio_write(0x18,4, (u16)(reg & 0x0000ffff));//LSB
	mdio_write(0x18,6, (u16)((reg & 0x00ff0000)>>16 | 0x8100)); //MSB | Bit15=AccessEn, Bit8=WriteOperation.

	timeout = 0;
	//wait for MDC_Reg_en bit(bit15) on REGAD6 to 0.
	while(1){
			regData = (u16) stmMdioRead(0x18, 6);
			//printf("regData=%04x",regData);
			if((regData & 0x8000) == 0)
				break;
			else
				timeout++;

			if(timeout>0x500)
			{
				printf("Error\r\n");
				return 0xffffffff;
			}
	}

	return 1;
}

//See 7.9.6 - For OP-FSM Registers
u32 RTL9047AASwitch_GetSpecialRegister(u16 regaddr){
	u32 retval;
	mdio_write(0x01, 27, regaddr); //Write Reg 27. Data = 0xwxyz regaddr
	retval = stmMdioRead(0x01, 28); //read reg
	return retval;
}

u32 RTL9047AASwitch_SetSpecialRegister(u16 regaddr, u16 value){
	u32 retval;
	mdio_write(0x01, 27, regaddr); //Write Reg 27. Data = 0xwxyz regaddr
	mdio_write(0x01, 28, value); //Write reg
	return retval;
}


void RTL9047AASwitch_Show_All_Registers(){
	u8 i;
	u8 reg;
	u16 page;
	u32 retval;
	char str[16];
	/*
	printf("==RTL9047Switch==\r\n");
	for(i=0;i<23;i++){
		reg =  RTL9047AAReg[i].regaddr;
		//page = RTL9047AAReg[i].page;
		retval = RTL9047AASwitch_GetNormalRegister(reg);//,page);
		printf("%s%s=%04x\r\n",RTL9047AAReg[i].regName,RTL9047AAReg[i].detail, retval & 0xffff );
		sprintf(str,"%s=%04x\r\n",RTL9047AAReg[i].regName, retval & 0xffff );
		OzOLED_printString(str,0,4,15); //No Master/Slave Status
		delayms(500);
	}
	*/
	i=0;
/*	while(RTL9047AASpecialReg[i].regName != 0x00){
		retval = yRTL9047AASwitch_GetSpecialRegister(RTL9047AASpecialReg[i].regaddr);
		printf("%s%s=%04x\r\n",RTL9047AASpecialReg[i].regName,RTL9047AASpecialReg[i].detail, retval & 0xffff );
		i++;
	}

}

//{ "r",      RTL9047AASwitch_Cmd_read,   "  : read <phy addr> <reg addr> <page>" },
int RTL9047AASwitch_Cmd_read(int argc, char *argv[]){

 	 u32 phyaddr = 0x18;
	u32 reg;
	u32 val;
	u16 page;
	int strlen;
	char str[20];

	if(argc==2){
		reg	=	(u32)strtol(argv[1], NULL, 16);
		val	=	RTL9047AASwitch_GetNormalRegister(reg);
		strlen = sprintf(str,"R>%06x=0x%08x", reg, val);
		printf("%s\r\n", str);
		//OzOLED_printString(str, 0, 7, 15);//strlen);
		return 0;
	}else if(argc==3){
		reg	=	    (u32)strtol(argv[1], NULL, 16);
		val = stmMdioRead(phyaddr, reg);//IP101A
		printf("0x%04x\r\n", val);
	}else{
		printErr();
		return -1;
	}

}
//{ "ra",     RTL9047AASwitch_Cmd_readAll,   " : read all <phy addr>" },
int RTL9047AASwitch_Cmd_readAll(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	int strlen;
	char str[20];
	//read <phy addr> <reg addr>
	if(argc==2){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);

		RTL9047AASwitch_Show_All_Registers();
		return 0;

	}else{
		printErr();
		return -1;
	}

}
//{ "w",      RTL9047AASwitch_Cmd_write,   " : write <phy addr> <reg addr> <page> <value>" },
int RTL9047AASwitch_Cmd_write(int argc, char *argv[]){
	/*
	u32 phyaddr = 0x18;
	u32 reg;
	u32 val;
	u16 page;
	if(argc==3){
		reg	=	    (u32)strtol(argv[1], NULL, 16);
		val	=	    (u32)strtol(argv[2], NULL, 16);
		if(RTL9047AASwitch_SetNormalRegister(reg, val) != 0xffffffff)
			printf("Writing on Phy(0x18) Reg 0x%04x <-- 0x%08x\r\n", reg, val);

		return 0;
	}else{
		printErr();
		return -1;
	}

}
int RTL9047AASwitch_Cmd_softreset(int argc, char *argv[]){
	return 0;
}
*/
//===================General=====================================================================
void stmMdio_Show_All_BasicRegisters(){
	u8 i;
	u16 page_extended;
	u32 retval;
	char str[16];
	struct _PhyREG *pBasicReg;

	printf("==%s==\r\n",PhyMdioModule.phyname);
	//Basic Registers
	pBasicReg = PhyMdioModule.PhyRegs.pBasicReg;
	for(i=0;i<16;i++){
		if(PhyMdioModule.clause45 >= 0x45){ //clause45 or clause45over22
			retval = stmMdio_GetBasicRegister(DEV_PMAPMD, pBasicReg->regaddr);
			printf("[Phy %02x] 0x%02x.%04x(MMD.REG) = 0x%04x (%s %s)\r\n",PhyMdioModule.phyaddr,DEV_PMAPMD,pBasicReg->regaddr, retval & 0xffff, pBasicReg->regName,pBasicReg->detail);
		}else{
			retval = stmMdio_GetBasicRegister(DEV_NULL,pBasicReg->regaddr);
			printf("[Phy %02x] 0x%04x(REG) = 0x%04x (%s %s)\r\n",PhyMdioModule.phyaddr, pBasicReg->regaddr, retval & 0xffff, pBasicReg->regName,pBasicReg->detail);
		}

		if(PhyMdioModule.use_oled){
			sprintf(str,"%s=%04x\r\n",pBasicReg->regName, retval & 0xffff );
			OzOLED_printString(str,0,4,15); //No Master/Slave Status
		}

		yConfirmLEDBlink();
		delayms(400);
		pBasicReg++;
	}
}
//====================================================== cmdline processing =============================
int GeneralPhy_Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;
/*
    //printf("---------------------------------------------\r\n");
    printf("\r\n>>> EtherPhyManager by EtherCrafts(r) <<<\r\n");
    //printf("---------------------------------------------\r\n");
    printf("*Support IEEE802.3 Clause22/45/AnnexD.\r\n");
    printf("*Set PHY Address First with command sa, otherwise the default phy_addr is 1.\r\n");
    printf("*In addition, add 45 for accessing with IEEE802.3 Clause45.\r\n");
    printf("*or, add 4522 for accessing with IEEE802.3 Annex 22D.\r\n");
    //printf("------------------\r\n");
    printf("Available PHYs\r\n");
    printf("1)BR; 2)RTL9K; 3)SW_BR_BR; 4)SW_RTL_BR 5) SW_RTL_RTL 6)RTL9047; 7)General\r\n");
    //printf("------------------\r\n");
    printf("Available commands\r\n");
    //printf("------------------\r\n");
*/
    gp_sCmdTable = &g_sCmdTable_GENERALPHY[0];
    pEntry = gp_sCmdTable; //&g_sCmdTable[0];

    while(pEntry->pcCmd){
        // Print the command name and the brief description.
        printf("%s%s\r\n", pEntry->pcCmd, pEntry->pcHelp);
        // Advance to the next entry in the table.
        pEntry++;
    }
    yConfirmLEDBlink();
    printf("First, you should select a PHY with <sp> \r\n");
    return(0);    // Return success.
}

extern  void yRTL9Kmc_RegConfig(void);

//{sa <phy addr> [45]" },or {sa <phy addr> [4522]" },
int GeneralPhy_Cmd_SetPhyAddrAndInitConfig(int argc, char *argv[]){
	u32 reg;
	u32 val1, val2;
	u16 page;
	u16 temp16;

	PhyMdioModule.f_preamblesuppressed = 0;

	if(argc==2){
		PhyMdioModule.clause45 =	0;
		PhyMdioModule.phyaddr =	(u8)strtol(argv[1], NULL, 16);
		PhyMdioModule.phyaddr_set_done = 1;
		PhyMdioModule.PhyRegs.pBasicReg = GeneralPhyBasicReg;

#if(PLATFORM == RTL9K)
		yRTL9Kmc_RegConfig();
		yConfirmLEDBlink();
		return 0;
#elif(PLATFORM == LAN9355SWITCH_RTL_BR)
		yConfirmLEDBlink();
		return 0;
#endif

		//Confirm
		val1 = stmMdio_GetBasicRegister(0,2);
		val2 = stmMdio_GetBasicRegister(0,3);

		if(val1==0x2000){
			PhyMdioModule.phyname = "DP83848";
			PhyMdioModule.phyid32 = 0x2000;
		}else if(val1==0x001c){
/*			if(val2==0xC916){
				PhyMdioModule.phyname = "RTL8211F";
				PhyMdioModule.phyid32 = 0x001cc916;
				PhyMdioModule.PhyRegs.pSpecificReg = RTL8211SpecificReg;
				yRTL8211_Config(PhyMdioModule.phyaddr, PhyMdioModule.phyname);
				//Tx Skew(at transmitter) = 0. Rx Skew (at receiver)= 1.8nsec.
			}else if(val2==0xC912){
				PhyMdioModule.phyname = "RTL8211CL";
				PhyMdioModule.phyid32 = 0x001cc912;
				PhyMdioModule.PhyRegs.pSpecificReg = RTL8211SpecificReg;
				yRTL8211_Config(PhyMdioModule.phyaddr, PhyMdioModule.phyname);
			}else if(val2==0xC816){
				PhyMdioModule.phyname = "RTL8201F/FL/FN";
				PhyMdioModule.phyid32 = 0x001cc816;
				PhyMdioModule.PhyRegs.pSpecificReg = RTL8201SpecificReg;
				yRTL8201_Config(PhyMdioModule.phyaddr,PhyMdioModule.phyname);
			}else{
				PhyMdioModule.phyname = "RTL8201";
				PhyMdioModule.phyid32 = 0x001c;
				PhyMdioModule.PhyRegs.pSpecificReg = RTL8201SpecificReg;
				yRTL8201_Config(PhyMdioModule.phyaddr,PhyMdioModule.phyname);
			}
*/
		}else if(val1==0x0022){
			if(val2==0x1611){
				PhyMdioModule.phyname = "KSZ9021 GbE PHY";
				PhyMdioModule.phyid32 = 0x00221611;
				yKSZ9021_Config();
			}else if((val2 & 0xfff0)==0x1550){
				PhyMdioModule.phyname = "KSZ8051 PHY";
				PhyMdioModule.phyid32 = (val1 << 16) | val2;
			}
		}else if(val1==0x0141){
			if((val2 & 0xfff0)==0x0cc0){
				PhyMdioModule.phyname = "Marvell 88E1111 GbE PHY";
				PhyMdioModule.phyid32 = (val1 << 16) | val2;
				//yKSZ9021_Config();
			}
		}else if(val1==0x0007){
			if((val2 & 0xfff0)==0xc0f0){
				PhyMdioModule.phyname = "LAN8720 RMII PHY";
				PhyMdioModule.phyid32 = (val1 << 16) | val2;
			}
		}else
			PhyMdioModule.phyname = "";

		printf("Set %s Phy Addr(0x%02x)/PHYID1/PHYID2(=0x%04x/%04x)\r\n", PhyMdioModule.phyname, PhyMdioModule.phyaddr, val1, val2);

		yConfirmLEDBlink();
		return 0;
	}else if(argc==3){ //support clause45 and clause45over22.
		temp16 = strtol(argv[2], NULL, 16); //==0x45 or 0x4522?
		if(temp16 == 0x45){	//Clause45
			PhyMdioModule.phyaddr_set_done = 1;
			PhyMdioModule.phyaddr =	(u8)strtol(argv[1], NULL, 16);
			PhyMdioModule.clause45 =	temp16;
			PhyMdioModule.f_preamblesuppressed = 1;
			PhyMdioModule.PhyRegs.pBasicReg = GeneralPhyBasicReg45;

			val1 = stmMdio_GetBasicRegister(DEV_PMAPMD,2);
			val2 = stmMdio_GetBasicRegister(DEV_PMAPMD,3);

			if(val1==0x0141){
				if((val2 & 0xfff0)==0x0d80){
					PhyMdioModule.phyname = "Marvell 88Q2112 1GPHY";
					PhyMdioModule.phyid32 = (val1 << 16) | val2;
					//y88Q2112_Config();
				}
			}else
				PhyMdioModule.phyname = "";

			printf("Set %s Phy Addr(0x%02x)/PHYID1/PHYID2(=0x%04x/%04x)\r\n", PhyMdioModule.phyname, PhyMdioModule.phyaddr, val1, val2);

			yConfirmLEDBlink();
			return 0;
		}else if(temp16 == 0x4522){	//Clause45over22
			PhyMdioModule.phyaddr_set_done = 1;
			PhyMdioModule.phyaddr =	(u8)strtol(argv[1], NULL, 16);
			PhyMdioModule.clause45 =	temp16;
			PhyMdioModule.f_preamblesuppressed = 1;
			PhyMdioModule.PhyRegs.pBasicReg = GeneralPhyBasicReg45;

			val1 = stmMdio_GetBasicRegister(DEV_PMAPMD,2);
			val2 = stmMdio_GetBasicRegister(DEV_PMAPMD,3);

			if(val1==0x0141){
				if((val2 & 0xfff0)==0x0d80){
					PhyMdioModule.phyname = "Marvell 88Q2112 1GPHY";
					PhyMdioModule.phyid32 = (val1 << 16) | val2;
					//y88Q2112_Config();
				}
			}else
				PhyMdioModule.phyname = "";

			printf("Set %s Phy Addr(0x%02x)/PHYID1/PHYID2(=0x%04x/%04x)\r\n", PhyMdioModule.phyname, PhyMdioModule.phyaddr, val1, val2);

			yConfirmLEDBlink();
			return 0;
		}
	}
	//errors
	yConfirmLEDBlink();
	printErr();
	return -1;

}
//{ "r",     GeneralPhy_Cmd_read,   "  : read <reg addr> <page>" },
int GeneralPhy_Cmd_read(int argc, char *argv[]){
	u8 phyaddr;
	u16 reg;
	u16 val;
	u16 page_mmdid;
	int strlen;
	char str[20];
	if(!PhyMdioModule.phyaddr_set_done){
		printErr();
		return -1;
	}
	if(argc==3){ //"r <page/mmd> <reg addr> "
		page_mmdid=	    (u16)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[2], NULL, 16);
		if(page_mmdid==0){
			val	=	stmMdio_GetBasicRegister(NULL,reg);
			printf("[Phy%d] 0x%04x(Reg) = 0x%04x\r\n", PhyMdioModule.phyaddr, reg, val);
		}else{
			if(PhyMdioModule.clause45 == 0x45){
				val	=	stmMdio_GetBasicRegister(page_mmdid,reg);
				printf("[Phy%d] 0x%02x.%04x (MMD.REG) = 0x%04x\r\n",  PhyMdioModule.phyaddr,page_mmdid,reg, val);
			}else if(PhyMdioModule.clause45 == 0x4522){
				val	=	stmMdio_GetBasicRegister(page_mmdid,reg);
				printf("[Phy%d] (0x%02x.%04x (MMD.REG) = 0x%04x\r\n",  PhyMdioModule.phyaddr,page_mmdid,reg, val);
			}else{
				//val	=	yRTL_GetExtendedReg(reg,page_mmdid);
				printf("[Phy%d] Reg 0x%02x (Page 0x%03x) = 0x%04x\r\n", PhyMdioModule.phyaddr,reg, page_mmdid, val);
			}
		}
		//strlen = sprintf(str,"R>%02x(%02x)=0x%04x", PhyMdioModule.phyaddr, reg, val);
		//OzOLED_printString(str, 0, 7, 15);//strlen);
	}else if(argc==2){
		reg	=	    (u32)strtol(argv[1], NULL, 16);
		if(PhyMdioModule.phyid32 == 0x0022){ //ksz9021
			if(reg <0x100){
				val	=	stmMdio_GetBasicRegister(DEV_NULL,reg);
			}else{
				val = yKSZ9021_GetExtendedReg(reg);
			}
			printf("[Phy%d] 0x%04x(REG) = 0x%04x\r\n",  PhyMdioModule.phyaddr,reg, val);
		}else{ //Other Phys for basic registers(or page 0 or mmdid==1)
			if(PhyMdioModule.clause45 == 0x45){
				val	=	stmMdio_GetBasicRegister(DEV_PMAPMD,reg);
				printf("[Phy%d] 0x1.%04x(MMD.REG) = 0x%04x\r\n",  PhyMdioModule.phyaddr,reg, val);
			}else if(PhyMdioModule.clause45 == 0x4522){
				val	=	stmMdio_GetBasicRegister(page_mmdid,reg);
				printf("[Phy%d] 0x1.%04x(MMD.REG) = 0x%04x\r\n",  PhyMdioModule.phyaddr,reg, val);
			}else{
				val	=	stmMdio_GetBasicRegister(DEV_NULL,reg);
				//val	=	yRTL_GetExtendedReg(reg,page_mmdid);
				printf("[Phy%d] 0x0.%03x(Page.Reg) = 0x%04x\r\n", PhyMdioModule.phyaddr,reg, val);
			}
		}

	}else{
		printErr();
		return -1;
	}
	yConfirmLEDBlink();
	return 0;
}
//{ "ra",     RTL8201_Cmd_readAll,   " : read all <phy addr>" },
int GeneralPhy_Cmd_readAll(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	int strlen;
	char str[20];
	if(!PhyMdioModule.phyaddr_set_done){
		printErr();
		return -1;
	}
	//ra
	if(argc==1){
		stmMdio_Show_All_BasicRegisters();
		yConfirmLEDBlink();
		return 0;
	}else{
		printErr();
		return -1;
	}
}


//{ "w",      RTL8201_Cmd_write,   " : w  [page/mmd] <reg addr> <value>" },
int GeneralPhy_Cmd_write(int argc, char *argv[]){
	u32 reg;
	u32 val;
	u16 page_mmd;
	if(!PhyMdioModule.phyaddr_set_done){
		printErr();
		return -1;
	}
	if(argc==4){ //page/mmd included -- RTL8201, RTL8211
		page_mmd = (u16)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[2], NULL, 16);
		val	=	    (u32)strtol(argv[3], NULL, 16);

		if(PhyMdioModule.clause45){
			yGeneralPhy_SetBasicRegister(page_mmd,reg, val);
			printf("[Phy%d] Writing 0x%02x.0x%04x[MMD.REG]<-- 0x%04x\r\n",  PhyMdioModule.phyaddr, page_mmd, reg,val);
		}else{
			//yRTL_SetExtendedReg(reg, page_mmd, val);
			printf("[Phy%d] Writing 0x%02x.0x%04x[Page.Reg]<-- 0x%04x\r\n",  PhyMdioModule.phyaddr, page_mmd, reg,val);
		}
		yConfirmLEDBlink();
		return 0;
	}else if(argc==3){ //no page/mmd.  basic or extended(KSZ9021)
		reg	=   (u32)strtol(argv[1], NULL, 16);
		val	=   (u32)strtol(argv[2], NULL, 16);

		if(reg<0x100){
			yGeneralPhy_SetBasicRegister(DEV_NULL, reg, val);
		}else{
			yKSZ9021_SetExtendedReg(reg, val);
		}
		printf("[Phy%d] Writing Reg 0x%02x <-- 0x%04x\r\n",  PhyMdioModule.phyaddr, reg, val);
		yConfirmLEDBlink();
		return 0;
	}else{
		printErr();
		return -1;
	}
}

//{ "ss",     RTL8201_Cmd_showstatus,   " : showstatus " },
int GeneralPhy_Cmd_showstatus(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	if(!PhyMdioModule.phyaddr_set_done){
		printErr();
		return -1;
	}
	if(argc!=1){
		printErr();
		return -1;
	}

	//phyaddr	=	(u32)strtol(argv[1], NULL, 16);
//	if(phyaddr == 1){
		//RTL8201_showStatusRTL9K(0);

	//}else{
		printf("Not yet..\r\n");
	//}

	yConfirmLEDBlink();
	return 0;
}
int GeneralPhy_Cmd_SoftReset(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	if(!PhyMdioModule.phyaddr_set_done){
		printErr();
		return -1;
	}
	if(argc==1){
		if(PhyMdioModule.clause45){
			yGeneralPhy_SetBasicRegister(DEV_PMAPMD, 0x00, 0x8000);//mdio_write(g_phyaddr, 0x00, 0x8000);
		}else {
			yGeneralPhy_SetBasicRegister(DEV_NULL, 0x00, 0x8000);//mdio_write(g_phyaddr, 0x00, 0x8000);
		}
		printf("[Phy%d] Resetting..\r\n",PhyMdioModule.phyaddr);
		yConfirmLEDBlink();
		return 0;
	}
	printErr();
	return -1;
}

extern stmBroadR_SetupNormalMiiMode(u8 phyid);

int GeneralPhy_Cmd_SelectPhy(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	if(argc==2){
		PhyMdioModule.platform = (u8)strtol(argv[1], NULL, 16);
		printf("Selected PHY is %s \r\n", mdio_phystring[PhyMdioModule.platform]);
		if(PhyMdioModule.platform == PHYIS_BROADCOM){
		   	//gp_sCmdTable = &g_sCmdTable_BROADCOM[0];
		   	gp_sCmdTable = &g_sCmdTable_GENERALPHY[0];
		}
		else if(PhyMdioModule.platform == RTL9K)
			gp_sCmdTable = &g_sCmdTable_GENERALPHY[0];//gp_sCmdTable = &g_sCmdTable_RTL9K[0];
/*		else if(PhyMdioModule.platform == IP101G)
			 gp_sCmdTable = &g_sCmdTable_GENERALPHY[0];//&g_sCmdTable_IP101G[0];
		else if(PhyMdioModule.platform == RTL8201)
			 gp_sCmdTable = &g_sCmdTable_RTL8201[0];
		else if(PhyMdioModule.platform == KSZ9021RN)
			 gp_sCmdTable = &g_sCmdTable_GENERALPHY[0];
		else if(PhyMdioModule.platform == RTL9047SWITCH)
	    	gp_sCmdTable = &g_sCmdTable_RTL9047SWITCH[0];
*/		else //GENERALPHY
			 gp_sCmdTable = &g_sCmdTable_GENERALPHY[0];

		//yConfirmLEDBlink();
		return 0;
	}
	printErr();
	return -1;
}
void GeneralPhy_Periodic_Handler(u8 first){

	//yRTL8201_ModeSelectorHandler(first);
#if (USE_DISPLAY == USE_DISPLAY_OLED)
	//RTL8201_showStatusRTL9K(1);
#else
	//RTL8201_showStatus(0);
#endif

}

//Read basic register@2
unsigned short stmMdioGetPhyID()
{
	u16 phyId16, revId16;
	phyId16 = stmMdio_GetBasicRegister(0,2);
	revId16 = stmMdio_GetBasicRegister(0,3);

	switch(phyId16){
	case 0x2000:
		PhyMdioModule.phyname = "DP83848";
		PhyMdioModule.PhyRegs.pSpecificReg = NULL;
		break;
	case 0x001c:
		PhyMdioModule.phyname = "RTL8201";
		//PhyMdioModule.PhyRegs.pSpecificReg = RTL8201SpecificReg;
		//RTL8201_Config();
		break;
	case 0x0022:
		PhyMdioModule.phyname = "KSZ9021";
		//PhyMdioModule.PhyRegs.pSpecificReg = KSZ9021SpecificReg;
		//KSZ9021_Config();
	default:
		PhyMdioModule.phyname = "UNKNOWNPHY";
		PhyMdioModule.PhyRegs.pSpecificReg = NULL;
		break;
	}
};

//========= main loop ================================
void stmMdioGeneralPhy_Loop(){
	int i=0;
	char str[10];
	u32 nX, pX, nY, pY;

	delayms(500);

	stmMdioInitPins();

	stmMdioGetPhyID();

	PhyMdioModule.use_oled = 1;
	PhyMdioModule.phyaddr_set_done = 0;
	if(PhyMdioModule.use_oled){
		OzOLED_init();
		delayms(100);
		OzOLED_sendCommand(0x8d);
		delayms(10);
		OzOLED_sendCommand(0x14);
		OzOLED_printString("EtherPhyManager",0,0,15); //Print the String at top line.
		OzOLED_printString("EtherCrafts(R)",0,7,15); //Print the String at bottom line.
	}
	printf("EtherPhyManager\r\n");
	printf("EtherCrafts(R)");

	GeneralPhy_Cmd_help(0,0);

    printf("CLI>");
	while(1){
		    dtCmd_handler(); //Need a special handling for register 0x1c....
		    GeneralPhy_Periodic_Handler(0);
	}
}

/*void yRTL8201_ModeSelectorHandler(u8 first){
	long val1,val2;
	val1 = (GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_2) >> 2) & 0x00000001;//Read Forced1_AN0

	if(first || (RTL9K_PhyStatus.fHost_or_AN !=  val1)){
		RTL9K_PhyStatus.fHost_or_AN = val1;//Update
		if(RTL9K_PhyStatus.fHost_or_AN){
			val2 = (GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) >> 3) & 0x00000001;//Get Master/Slave
			RTL9K_PhyStatus.fMaster_or_Slave = val2;
			if(val2){
				//printf("Writing on Phy(0x19) Reg 0x%02x <-- 0x%04x\r\n",  phyaddr, reg, val);
				mdio_write(0x19, 0, 0x0208);//Reg0 set as Master
				printf("*Forced Master\r\n");
				OzOLED_printString("*Forced Master ",0,1,15);
			}else{
				mdio_write(0x19, 0, 0x0200);//Reg0 set as Slave
				printf("*Forced Slave\r\n");
				OzOLED_printString("*Forced Slave ",0,1,15);
			}

		}else{ //AN
			mdio_write(0x19, 0, 0x1000);//0x1200
			printf(">AutoNeg.\r\n");
			OzOLED_printString("*AutoNego. Mode.  ",0,1,15);
		}

	}else if(RTL9K_PhyStatus.fHost_or_AN > 0){ //NO Changed. But if it is still forced.
			val2 = (GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) >> 3) & 0x00000001; ////Get Master/Slave
			if(RTL9K_PhyStatus.fMaster_or_Slave !=  val2){
				RTL9K_PhyStatus.fMaster_or_Slave = val2;
				if(val2){
					mdio_write(0x19, 0, 0x0208); //Reg0 set Master
					printf(">Forced Master\r\n");
					OzOLED_printString("*Forced Master ",0,1,15);
				}else{
					mdio_write(0x19, 0, 0x0200); //Reg0 set Slave
					printf(">Forced Slave\r\n");
					OzOLED_printString("*Forced Slave ",0,1,15);
				}
			}
	}
	delayms(10);
}
 */

/*
void yRTL8201_StatusHandler(u8 extended){
	long val32;
	//struct _RTL9K_PhyStatus rdConfig;
	unsigned short linkStatus;

	if(extended){
		RTL9K_PhyStatus.regf_extstatus = (u16)stmMdioRead(0x19, 0x0f); //Extended LRE Status
		if((RTL9K_PhyStatus.reg1_status & 0x0300) == 0x0300) OzOLED_printString("+Rcv(L/R)=OK/OK",0,2,15);
		else if((RTL9K_PhyStatus.reg1_status & 0x0300) == 0x0200) OzOLED_printString("-Rcv(L/R)=OK/No",0,2,15);
		else if((RTL9K_PhyStatus.reg1_status & 0x0300) == 0x0100) OzOLED_printString("-Rcv(L/R)=No/OK",0,2,15);
		else                                               OzOLED_printString("-Rcv(L/R)=No/No",0,2,15);

		RTL9K_PhyStatus.reg11_phyextstatus = (u16)stmMdioRead(0x19, 0x11); //PHY Extended Status
		if((RTL9K_PhyStatus.reg11_phyextstatus & 0x2000)==0x2000) OzOLED_printString("+MDI Crossed   ",0,3,15);
		if((RTL9K_PhyStatus.reg11_phyextstatus & 0x0800)==0x0800) OzOLED_printString("+RemoteRcv:OK  ",0,4,15);
		else                                               OzOLED_printString("-RemoteRcv:FAIL",0,4,15);
		if((RTL9K_PhyStatus.reg11_phyextstatus & 0x0400)==0x0400) OzOLED_printString("+LocalRcv:OK   ",0,5,15);
		else                                               OzOLED_printString("-LocalRcv:FAIL ",0,5,15);
		delayms(2000);

		if((RTL9K_PhyStatus.reg11_phyextstatus & 0x0200)==0x0200) OzOLED_printString("+DSCR:Locked  ",0,2,15);
		else                                               OzOLED_printString("-DSCR:Unlocked",0,2,15);
		if((RTL9K_PhyStatus.reg11_phyextstatus & 0x0100)==0x0100) OzOLED_printString("+Link:Passed   ",0,3,15);
		else                                               OzOLED_printString("-Link:NotPassed",0,3,15);

		delayms(2000);

		RTL9K_PhyStatus.reg1a_intstatus = (u16)stmMdioRead(0x19, 0x1a); //Interrupt Status
		if((RTL9K_PhyStatus.reg1a_intstatus & 0x4000)==0x4000) OzOLED_printString("+PairSwapped  ",0,2,15);
		if((RTL9K_PhyStatus.reg1a_intstatus & 0x0020)==0x0020) OzOLED_printString("+RemRcvStsChanged",0,3,15);
		if((RTL9K_PhyStatus.reg1a_intstatus & 0x0010)==0x0020) OzOLED_printString("+LocRcvStsChanged",0,4,15);
		if((RTL9K_PhyStatus.reg1a_intstatus & 0x0004)==0x0004) OzOLED_printString("+LinkSpdChanged",0,5,15);
		//if((RTL9K_PhyStatus.reg1a_intstatus & 0x0002)==0x0002) OzOLED_printString("+LinkStsChanged",0,6,15);
		delayms(2000);
	}
	//Read LRE Status Register
	linkStatus = (u16)stmMdioRead(0x19, 1); //LRE Status
	if(RTL9K_PhyStatus.reg1_status != linkStatus){
		RTL9K_PhyStatus.reg1_status = linkStatus;//Update it.
		if(RTL9K_PhyStatus.reg1_status & 0x0004){
			OzOLED_printString("+Link Up   ",0,3,12);
			printf("+Link Up    \r\n");
			RTL9K_PhyStatus.LinkUp = 1;
		}else{
			OzOLED_printString("-Link Down  ",0,3,12);
			printf("-Link Down  \r\n");
			RTL9K_PhyStatus.LinkUp = 0;
		}
	}
	//Read LED Status Register
	mdio_write(0x19, 0x1c, 0x2000);//1c_shadow_01000 select
	delayms(10);
	RTL9K_PhyStatus.reg1c08_ledstatus = (u16)stmMdioRead(0x19, 0x1c);
	//printf("RTL9K_PhyStatus.reg1c08_ledstatus=%04x\r\n",RTL9K_PhyStatus.reg1c08_ledstatus);
	if((RTL9K_PhyStatus.reg1c08_ledstatus & 0x0080)==0x0080) OzOLED_printString("-HDX           ",0,5,15);
	else  											  OzOLED_printString("+FDX           ",0,5,15);

	if((RTL9K_PhyStatus.reg1c08_ledstatus & 0x0018)==0x0018) OzOLED_printString("-NoLink        ",0,6,15);
	else if((RTL9K_PhyStatus.reg1c08_ledstatus & 0x0018)==0x0010) OzOLED_printString("+10Mbps      ",0,6,15);
	else if((RTL9K_PhyStatus.reg1c08_ledstatus & 0x0018)==0x0008) OzOLED_printString("+100Mbps     ",0,6,15);
	else  											  OzOLED_printString("-NA            ",0,6,15);

	if((RTL9K_PhyStatus.reg1c08_ledstatus & 0x0001)==0x0001) OzOLED_printString("-PoorQuality   ",0,7,15);
	else  											  OzOLED_printString("+GoodQuality   ",0,7,15);


	if(RTL9K_PhyStatus.LinkUp){
		if(RTL9K_PhyStatus.fForced_or_AN == 1){ //ForcedMode
			if(RTL9K_PhyStatus.fMaster_or_Slave == 1)               OzOLED_printString("+M/S:Master        ",0,4,15);
			else  											 OzOLED_printString("-M/S:Slave         ",0,4,15);
		}else{ //AN Mode
			if((RTL9K_PhyStatus.reg1c08_ledstatus & 0x0100)==0x0100) OzOLED_printString("+M/S:Master        ",0,4,15);
			else  											  OzOLED_printString("-M/S:Slave         ",0,4,15);
		}
	}else{ //Link Down
              OzOLED_printString("-M/S: NA    ",0,4,15); //No Master/Slave Status
	}

	return;
}
*/


