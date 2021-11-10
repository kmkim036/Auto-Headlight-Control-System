/* IP101G_MDIO.c
 * IP101G can support
 *      * MII/RMII
 *      * RMII Back2Back, CRS_DV Toggle, CRS_DV Not Toggle. -- 16.13:12 and 16.2 (Page16 Reg16)
 * [GPIOs]
//-- LM3S811
//MDIO-PC6
//MDC-PD1
 * SMI: PD6
 */


/*
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"
#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"
#include "ylib/yInc.h"
#include "ylib/ymdio.h"
#include "yLib/yPortMap.h"
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
//#include "stm32f10x_flash.h"
#include "yInc.h"
#include "ymdio.h"
#include "cmdline.h"

extern void yMdioConfirmLEDBlink(void);
//PC5(pin 9(SPI_CS2) of USB dongle which will ties with th pin22 on target PHY board.
extern void init_MDIO_LEDs(void); //LEDonDongle and PC5(pin 9(SPI_CS2) of USB dongle which will ties with th pin22 on target PHY board.
extern void init_MDIO(void);

//PHY_ACCESS_METHOD == SMIIF (should set this in yInc.h)
struct _PhyREG IP101GReg[]={
		 //--- STANDARD REG.
		{0,0,"CR","(control reg[0])"},
		{1,0,"SR","(status reg[1])"},
		{2,0,"PHYID1","[2]"},
		{3,0,"PHYID2","[3]"},
		{4,0,"ANAR","(AN Advertisement reg[4])"},
		{5,0,"ANPAR","(AN link partner ability reg[5])"},
		{6,0,"ANER","(AN expansion reg[6])"},
		{7,0,"ANNPTR","(AN Next Page Transmit Reg[7])"},
		{8,0,"ANLPNPR","(AN Link Partner Next Page Reg[8])"},
		//Specific Registers
		{16,16,"UPHYSCR","(UTP PHY Specific Control Reg[16,16])"}, //RMII Version Setting
		{17,16,"ISR","(Interrupt Status Reg[17,16])"},
		{18,16,"PISCSR","(PHY Interrupt Control/Status Reg[18,16])"},
		{19,16,"DIOSCR","(Digital IO Specific Control Reg[19,16])"},
		{30,16,"PHYMCSSR","PHY MDI/MDIX Control/Specific Status Reg[30,16)"},
		{17,1,"PHYSCR","(PHY Specific Control Reg[17,1])"},
		{13,0,"MMDASCR","(MMD Access Control Reg[13,0])"},
		{14,0,"MMDAADR","(MMD Access Address Data Reg[14,0])"},
		//Other MMD Data Registers, and etc...
		//RX Counter Register
		{17,1,"RXCCR","(RX Counter Control Reg[17,1])"}, //duplicated with {17,8}?
		{18,1,"RXCRCECR","(RX CRC Error Counter Reg[18,1])"},
		{18,2,"RXPCR","(RX Packet Counter Reg[18,2])"},
		{17,8,"RXCCR","(RX Counter Control Reg[17,8])"},//duplicated with {17,1}?
		{18,11,"PHYICSR","(PHY Interrupt Control/Status Reg[18,11])"},
		{17,18,"RXCICSR","(RX Counter Interrupt Control/Status Reg[17,18])"},
		//LED Mode Control Reg
		{16,3,"LEDCR","(LED Control Reg[16,3])"},
		//WOL Reg
		{16,4,"WOL+CR","(WOL + Control Reg[16,4])"},
		{16,5,"WOL+MAR","(WOL + MAC Addr Reg[16,5])"},
		{17,17,"WOL+SR","(WOL + Status Reg[17,17])"},
		//UTP PHY Specific
		{22,1,"LDOCR","(LDO Control Reg[22,1])"},
		{23,1,"PHYSCR","(PHY Specific Control Reg[23,1])"},
		//Digital IO Pin Control Regs
		//TBD
		//
		{0,0,""}
		//TBD...
};


struct _PhyStatus IP101G_PhyStatus;

int IP101G_Cmd_read(int argc, char *argv[]);
int IP101G_Cmd_readAll(int argc, char *argv[]);
int IP101G_Cmd_write(int argc, char *argv[]);
//*****************************************************************************
extern char g_cCmdBuf[CMD_BUF_SIZE];
extern tCmdLineEntry g_sCmdTable[];

//-- LM3S811
//MDIO-PC6
//MDC-PD1

//See 7.9.5
u32 yIP101G_GetNormalRegister(u32 phyaddr, u8 reg, u16 page){
	u32 retval;
	if(page == 0)//normal registers
		retval = mdio_read(phyaddr, reg); //read reg
	else{
		mdio_write(phyaddr, 20, page & 0x1f); //(1)Write Reg 20 with 5 bits page selection.
		retval = mdio_read(phyaddr, reg); //(2) read reg.
		mdio_write(phyaddr, 20, 0x0000); //Write Reg 20 with page 0 to switch back to Standard Reg.
	}
	return retval;
}

u32 yIP101G_SetNormalRegister(u32 phyaddr,u8 reg, u16 page, u16 value){
	u32 retval;
	if(page == 0)//normal registers
		mdio_write(phyaddr, reg, value);
	else{
		mdio_write(phyaddr, 20, page & 0x1f); //(1)Write Reg 20 with 5 bits page selection.
		mdio_write(phyaddr, reg, value); //(2) Write value to the register.
		mdio_write(phyaddr, 20, 0x0000); //Write Reg 20 with page 0 to switch back to Standard Reg.
	}
	return 0;
}

/* for MMD registers -- TBD
u32 yIP101G_GetSpecialRegister(u32 phyaddr,u16 regaddr){
	u32 retval;
	mdio_write(phyaddr, 27, regaddr); //Write Reg 27. Data = 0xwxyz regaddr
	retval = mdio_read(phyaddr, 28); //read reg
	return retval;
}

u32 yIP101G_SetSpecialRegister(u32 phyaddr,u16 regaddr, u16 value){
	u32 retval;
	mdio_write(phyaddr, 27, regaddr); //Write Reg 27. Data = 0xwxyz regaddr
	mdio_write(phyaddr, 28, value); //Write reg
	return retval;
}
*/
void IP101G_Show_All_Registers(u32 phyaddr){
	u8 i;
	u8 reg;
	u16 page;
	u32 retval;
	char str[16];
	UARTprintf("==IP101G==\r\n");
	for(i=0;i<23;i++){
		reg =  IP101GReg[i].regaddr;
		page = IP101GReg[i].page;
		retval = yIP101G_GetNormalRegister(phyaddr,reg,page);
		UARTprintf("%s%s=%04x\r\n",IP101GReg[i].regName,IP101GReg[i].detail, retval & 0xffff );
		yMdioConfirmLEDBlink();
		delayms(400);
	}
}

//{ "r",      IP101G_Cmd_read,   "  : read <phy addr> <reg addr> <page>" },
int IP101G_Cmd_read(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	int strlen;
	char str[20];

	if(argc==4){ //with page
		phyaddr	=	(u32)strtol(argv[1], NULL, 10);
		reg	=	    (u32)strtol(argv[2], NULL, 10);
		page=	    (u16)strtol(argv[3], NULL, 10);
		val	=	yIP101G_GetNormalRegister(phyaddr, reg, page);
		UARTprintf("R> (Phy%02u)[Reg%02u : Page%02u] => 0x%04x", phyaddr, reg,page, val);
		yMdioConfirmLEDBlink();
		return 0;
	}else if(argc==3){ //for page 0.
		phyaddr	=	(u32)strtol(argv[1], NULL, 10);
		reg	=	    (u32)strtol(argv[2], NULL, 10); //We use 10 for decimal
		page=	    0;
		val	=	yIP101G_GetNormalRegister(phyaddr,reg, page);
		UARTprintf("R> (Phy%02u)[Reg%02u : Page%02u] => 0x%04x", phyaddr, reg,page, val);
		yMdioConfirmLEDBlink();
		return 0;
	}else{
		printErr();
		return -1;
	}
}
//{ "ra",     IP101G_Cmd_readAll,   " : read all <phy addr>" },
int IP101G_Cmd_readAll(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	int strlen;
	char str[20];

	if(argc==2){
		phyaddr	=	(u32)strtol(argv[1], NULL, 10);
		IP101G_Show_All_Registers(phyaddr); //IP101
		yMdioConfirmLEDBlink();
		return 0;
	}else{
		printErr();
		return -1;
	}
}
//{ "w",      IP101G_Cmd_write,   " : write <phy addr> <reg addr> <page> <value>" },
int IP101G_Cmd_write(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	if(argc==5){
		phyaddr	=	(u32)strtol(argv[1], NULL, 10);
		reg	=	    (u32)strtol(argv[2], NULL, 10);
		page = (u16)strtol(argv[3], NULL, 10);
		val	=	    (u32)strtol(argv[4], NULL, 16);
		yIP101G_SetNormalRegister(phyaddr,reg, page, val);
		UARTprintf("Writing on Phy(%02u) Reg %02u (page=%02u)<-- 0x%04x\r\n",  phyaddr, reg,page, val);
		yMdioConfirmLEDBlink();
		return 0;
	}else if(argc==4){ //for page 0.
			phyaddr	=	(u32)strtol(argv[1], NULL, 10);
			reg	=	    (u32)strtol(argv[2], NULL, 10);
			val	=	    (u32)strtol(argv[3], NULL, 16);
			page = 0;
			yIP101G_SetNormalRegister(phyaddr,reg, page, val);
			UARTprintf("Writing on Phy(%02u) Reg %02u (page=%02u)<-- 0x%04x\r\n",  phyaddr, reg,page, val);
			yMdioConfirmLEDBlink();
			return 0;

	}else{
		printErr();
		return -1;
	}
}

//========= main loop ===
void yIP101G_Loop(){
	int i=0;
	char str[10];
	u32 nX, pX, nY, pY;

	GuiInit("\r\n\r\nmyMDIO with Bitbang Style.\r\n\r\n");
/*
	//ETC LED CONFIG : USB MODULE FOR MDIO/MDC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, ~GPIO_PIN_6);//OFF
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5);//OFF
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, ~GPIO_PIN_4);//OFF
#if(PHY_ACCESS_METHOD == SMIIF)
		//Config GPIO PD6 (LM3S811)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);//ON
#elif(PHY_ACCESS_METHOD == I2CIF)//I2C
		//Config GPIO PD5 (LM3S811)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_PIN_5);//ON
#else//SPI
		//Config GPIO PD4 (LM3S811)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4);//ON
#endif
*/
	init_MDIO_LEDs();
	init_MDIO();
	IP101G_Cmd_help(0,0);

    UARTprintf("CLI>");
	while(1){
		    dtCmd_handler(); //Need a special handling for register 0x1c....
	}
}



