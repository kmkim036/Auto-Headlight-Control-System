/*
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "stm32f4x7_eth_bsp.h"
#include "misc.h"
#include "cmdline.h"
#include "yInc.h"
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#if((PROCESSOR == STM32F103C8) || (PROCESSOR == STM32F107VCT6))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#endif
#include "cmdline.h"


#define TJANXP_PHY_ADDRESS 0
/* PHYADDR TJA1100 = 0x00, 0x05, (0x04)
 *  - This can support
 *      * AutoNegotiated Mode
 *      * Forced Master/Slave Selection
 *      * Report Current Operation Status on OLED
 * [GPIOs]
 * MDIO/MDC : Ethernet on STM
 * BUZZER : PC4
 * ULED0(PHY): PD7
 * --EXTRAS----
 * (OLED's I2C)
 * I2CSCL : PB2
 * I2CSDA : PB3
 * nIRQ(PHY) : PD0
 */

#define USE_OLED 0

#define SMIIF 1
#define I2CIF 2

struct TJANXPREG{
	u8 regaddr;
	u16 page;
	char *regName;
	char *detail;
};
struct TJANXPREG TJANXPReg[]={
		 //--- STANDARD REG.
		{0,0,"BMCR","(basic mode control reg:0,0)"},
		{1,0,"BMSR","(basic mode status reg:1,0)"},
		{2,0,"PHYID1:2,0",""},
		{3,0,"PHYID2:3,0",""},
		{9,0,"PHYCR","(phy control reg:9,0)"},
		{10,0,"PHYSR","(phy status reg:a,0)"},
		//--- TJANXP GENERAL REG.
		{0x10,0,"PHYSFR","(phy status sub-flag reg,:10,a42)"},
		{0x11,0,"RTCTCR","(rtct control reg:11,a42)"},
		{0x12,0,"GINER","(general interrupt enable reg:12,a42)"},
		{0x14,0,"GINMR","(general interrupt mask reg:14,a42)"},
		{0x15,0,"SLPCR","(sleep control reg:15,a42)"},
		{0x18,0,"PHYCR1","(phy specifici control reg1:18,a43)"},//0xa42 ???? -- Errata Found.
		{0x1B,0,"PHYSRADR","(phy sram address reg:1b,a43)"},
		{0x1C,0,"PHYSRDAT","(phy sram data reg:1c,a43)"},//Errta Found
		{0x1D,0,"GINSR","(general interrupt status reg:1d,a43)"},
		{0x1F,0,"PAGSR","(page select reg:1f,a43)"},
		{0x15,0,"GPSFR","(general purpose sub-flag reg:15,a47)"},
		{0x14,0,"SLPCAP","(sleep capability reg:14,a5a)"},
		{0x10,0,"CLENR","(cable length reg:10,a88)"},
		{0x11,0,"SSCCR","(ssc control reg:11,d01)"},
		{0x10,0,"LEDCR","(led control reg:10,d04)"},
		{0x15,0,"LED_PTP","(led/ptp-gpio select reg:15,d42)"}, //22
		//TBD...
};

struct _PhyStatus{
	//u8 fHost_or_AN; //1-0
	u8 LinkStatus;       //0x01
	u8 fMaster_or_Slave; //0x09
	u16 localRcv_status; //0x0A
	u16 remoteRcv_status; //0x0A
	u16 PHYsub_flag; //0x10
	//u16 regf_extstatus; //[9]=local rx ok;[8]=retmoe rx ok
	//u16 reg11_phyextstatus; //[13]=mdix; [11]=retmoe rx ok; [10]=local rx ok;[9]=descambled locked ;[8]=LinkPassed
	//u16 reg1a_intstatus;//[14]=pair swap;[5]=remote rx status changed;[4]=local rx status changed;[2]=link speed changed;[1]=link status changed;[0]=crc error
	//u16 reg1c08_ledstatus; //[8]=1master;[7]=0:FDX; [6]=1=No INT; [4:3]=01=100Mbps,[0]=1=Poor Quality
};
struct _PhyStatus stmTJANXP_PhyStatus;

extern void stmOzOLED_init();
extern void stmOzOLED_SendCmd(unsigned char command);
extern void stmOzOLED_sendData(unsigned char Data);
extern void stmOzOLED_printString(const char *String, u8 X, u8 Y, u8 numChar);

int stmTJANXP_Cmd_help(int argc, char *argv[]);
int stmTJANXP_Cmd_read(int argc, char *argv[]);
int stmTJANXP_Cmd_readAll(int argc, char *argv[]);
int stmTJANXP_Cmd_write(int argc, char *argv[]);
int stmTJANXP_Cmd_softreset(int argc, char *argv[]);

extern char g_cCmdBuf[CMD_BUF_SIZE];
extern tCmdLineEntry g_sCmdTable[];
#if ((PROCESSOR == STM32F103C8) || (PROCESSOR == STM32F107VCT6))
#else
void stmTJANXP_GpioConf(){

	GPIO_InitTypeDef GPIO_InitStruct;

	//PE5 : ULED0
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //PE5
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;//PE5
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOE, &GPIO_InitStruct);//
	//PB4 : Buzzer
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //PB4
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;//PB4
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	printf("Init TJANXP GPIO Done.\r\n");
}

//See 7.9.5
unsigned short stmTJANXP_GetNormalRegister(u16 reg16, u16 page){
	uint16_t ret16 = 0;
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, page); //mdio_write(0x01, 31, page); //Write Reg 31. Data = 0x0xyz page
	ret16 = ETH_ReadPHYRegister(TJANXP_PHY_ADDRESS, reg16);//	retval = mdio_read(0x01, reg); //read reg
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0000); //mdio_write(0x01, 31, 0x0000); //Write Reg 31. Data = 0x0000 to switch back to Standard Reg.
	return ret16;
}

unsigned short stmTJANXP_SetNormalRegister(u16 reg, u16 page, u16 value){
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, page); //mdio_write(0x01, 31, page); //Write Reg 31. Data = 0x0xyz page
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, reg, value); //mdio_write(0x01, reg, value); //Write reg
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0000); //mdio_write(0x01, 31, 0x0000); //Write Reg 31. Data = 0x0000 to switch back to Standard Reg.
	return 0;
}

//See 7.9.6 - For OP-FSM Registers
unsigned short stmTJANXP_GetSpecialRegister(u16 regaddr){
	uint16_t ret16 = 0;
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 27, regaddr); //mdio_write(0x01, 27, regaddr); //Write Reg 27. Data = 0xwxyz regaddr
	ret16 = ETH_ReadPHYRegister(TJANXP_PHY_ADDRESS, 28);//retval = mdio_read(0x01, 28); //read reg
	return ret16;
}

u32 stmTJANXP_SetSpecialRegister(u16 regaddr, u16 value){
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 27, regaddr); //mdio_write(0x01, 27, regaddr); //Write Reg 27. Data = 0xwxyz regaddr
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 28, value); //mdio_write(0x01, 28, value); //Write reg
	return 0;
}


//Following the TJANXP Errata List
void stmTJANXP_FixErrat1_Registers(void){
	u8 i;
	unsigned short reg;
	u16 page;
	unsigned short retval;
	char str[16];
	printf("==TJANXP Init Registers(for fix Errata 1)==\r\n");

	//1-10
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a5a); //mdio_write(0x01, 31, 0x0a5a);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 16, 0x1c00); //mdio_write(0x01, 16, 0x1c00);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0d4a); //mdio_write(0x01, 31, 0x0d4a);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 18, 0x1211); //mdio_write(0x01, 18, 0x1211);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 19, 0x1211); //mdio_write(0x01, 19, 0x1211);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 20, 0x1211); //mdio_write(0x01, 20, 0x1211);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 21, 0x1111); //mdio_write(0x01, 21, 0x1111);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0d42); //mdio_write(0x01, 31, 0x0d42);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 18, 0x1fc2); //mdio_write(0x01, 18, 0x1fc2);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 20, 0x4900); //mdio_write(0x01, 20, 0x4900);

	//11-20
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0bcc);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 19, 0x0000);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0bc0);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 16, 0x0030);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0c40);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 21, 0x2000);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0dc1);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 16, 0xfc8d);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0dc0);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 18, 0x0270);

	//21-30
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a43);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 25, 0x0800);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0bc5);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 17, 0x0cc8);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a43);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 27, 0x8010);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 28, 0xffc7);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a81);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 17, 0x1044);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a86);
	//31-40
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 17, 0x7f23);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 21, 0xff00);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a87);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 22, 0xbf7f);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 23, 0xbf7f);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a80);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 19, 0xbf7f);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 20, 0xbf7f);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a87);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 19, 0x7f00);

	//41-50
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a5a);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 21, 0x0000);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a80);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 17, 0x1fff);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 18, 0x1fff);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a43);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 27, 0x8011);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 28, 0xceb7);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0bc4);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 23, 0x0100);

	//51-60
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0d0b);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 16, 0x9400);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 17, 0x2000);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 19, 0xdd05);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 20, 0xc004);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0d01);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 17, 0x0014);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0bc4);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 22, 0xa040);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0d09);

	//61-68
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 16, 0x9400);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 17, 0x2000);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 19, 0xdd05);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 20, 0xc004);

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0d01); //	mdio_write(0x01, 31, 0x0d01);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 17, 0x0010); //mdio_write(0x01, 17, 0x0010); //mdio_write(0x01, 17, 0x0015); //YOON: WE DO NOT USE Spread Spectrum RMII Clocking Capability.

	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 31, 0x0a40); //mdio_write(0x01, 31, 0x0a40);
	ETH_WritePHYRegister(TJANXP_PHY_ADDRESS, 0, 0x9200); //mdio_write(0x01, 0, 0x9200);

}

void stmTJANXP_Show_All_Registers(){
	u8 i;
	u8 reg;
	u16 page;
	u32 retval;
	char str[16];
	printf("==TJANXP==\r\n");
	for(i=0;i<23;i++){
		reg =  TJANXPReg[i].regaddr;
		page = TJANXPReg[i].page;
		retval = stmTJANXP_GetNormalRegister(reg,page);
		printf("%s%s=%04x\r\n",TJANXPReg[i].regName,TJANXPReg[i].detail, retval & 0xffff );
#if (USE_OLED == 1)
		sprintf(str,"%s=%04x\r\n",TJANXPReg[i].regName, retval & 0xffff );
		stmOzOLED_printString(str,0,4,15); //No Master/Slave Status
#endif
		//stmConfirmLEDBlink();
		delayms(400);
	}
	i=0;
	while(TJANXPSpecialReg[i].regName != 0x00){
		retval = stmTJANXP_GetSpecialRegister(TJANXPSpecialReg[i].regaddr);
		printf("%s%s=%04x\r\n",TJANXPSpecialReg[i].regName,TJANXPSpecialReg[i].detail, retval & 0xffff );
		stmConfirmLEDBlink();
		delayms(400);
		i++;
	}
}

void stmTJANXP_PTP_En(void){
	u32 val;
	printf("PTP_Enable: ");
	val = 0x0001;//TJANXP_PTP_CTL.PTP_Enable(16.1)
	stmTJANXP_SetNormalRegister(0x10,0xe40,val);
	//Issue SoftReset to be affected.
	stmTJANXP_SetNormalRegister(0x00,0x0000,0x8000);//Soft Reset(0.15)
	delayms(500);
	//check
	val = stmTJANXP_GetNormalRegister(0x10,0xe40);//TJANXP_.PTP_Enable(0.15)
	printf("Set as %04x\r\n", val);
}
void stmTJANXP_SetLedModeAsLinkStatus(void){
	u32 val;
	printf("Set LED Mode as Link Status \r\n");
	//stmTJANXP_SetNormalRegister(0x15,0xd42,0x0001); //Pin34 acts as LED pin. -- Default
	stmTJANXP_SetNormalRegister(0x10,0xd04,0xc627); //0xe627 //enable LED, Link Status,
	//Issue SoftReset to be affected.
	stmTJANXP_SetNormalRegister(0x00,0x0000,0x8000);//Soft Reset(0.15)
	delayms(500);

}

void stmTJANXP_SupportErrata2(void){
	u32 val;
	stmTJANXP_SetNormalRegister(0x14,0xbcd,0x0317);
	delayms(500);
}

//====================================================== cmdline processing =============================
int stmTJANXP_Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    printf("\r\nEtherPhyManager by EtherCrafts(r)\r\n");
    printf("Available commands\r\n");
    printf("------------------\r\n");
    printf("PHYAddr: TJANXP=1; IP101ALF=2\r\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_sCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The
    // end of the table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        // Print the command name and the brief description.
        printf("%s%s\r\n", pEntry->pcCmd, pEntry->pcHelp);
        // Advance to the next entry in the table.
        pEntry++;
    }
    stmConfirmLEDBlink();
    return(0);    // Return success.
}
//{ "r",      stmTJANXP_Cmd_read,   "  : read <phy addr> <reg addr> <page>" },


int stmTJANXP_Cmd_read(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	int strlen;
	char str[20];
	//read <phy addr> <reg addr> <page>
	if(argc==4){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[2], NULL, 16);
		page=	    (u16)strtol(argv[3], NULL, 16);
		val	=	stmTJANXP_GetNormalRegister(reg, page);
		printf("0x%04x\r\n", val);
		strlen = sprintf(str,"R>%02x(%02x)=0x%04x", phyaddr, reg, val);
		//stmOzOLED_printString(str, 0, 7, 15);//strlen);
		stmConfirmLEDBlink();
		return 0;
	}else{
		printf("Err\r\n");//printErr();
		return -1;
	}
}
//{ "ra",     stmTJANXP_Cmd_readAll,   " : read all <phy addr>" },
int stmTJANXP_Cmd_readAll(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	int strlen;
	char str[20];
	//read <phy addr> <reg addr>
	if(argc==2){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		if(phyaddr == 1){
			TJANXP_Show_All_Registers_TJANXP(); //TJANXP
			stmConfirmLEDBlink();
			return 0;
		}else{
			printf("Err\r\n");//printErr();
			return -1;
		}
	}else{
		printf("Err\r\n");//printErr();
		return -1;
	}
}
//{ "w",      stmTJANXP_Cmd_write,   " : write <phy addr> <reg addr> <page> <value>" },
int stmTJANXP_Cmd_write(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	if(argc==5){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[2], NULL, 16);
		page = (u16)strtol(argv[3], NULL, 16);
		val	=	    (u32)strtol(argv[4], NULL, 16);
		stmTJANXP_SetNormalRegister(reg, page, val);
		printf("Writing on Phy(0x%02x) Reg 0x%02x (page=0x%04x)<-- 0x%04x\r\n",  phyaddr, reg,page, val);
		stmConfirmLEDBlink();
		return 0;
	}else{
		printf("Err\r\n");//printErr();
		return -1;
	}
}
//Master/Slave Setting
int stmTJANXP_Cmd_setms(int argc, char *argv[]){
	u32 ms;
	u32 reg;
	u32 val;
	u16 page;
	if(argc==2){
		ms	=	(u32)strtol(argv[1], NULL, 16);
		if(ms){
			stmTJANXP_SetNormalRegister(0x9,0x0000,0x0800);//master/slave select(9.11)
			printf("Set as Master.\r\n");
		}else{
			stmTJANXP_SetNormalRegister(0x9,0x0000,0x0000);//master/slave select(9.11)
			printf("Set as Slave.\r\n");
		}
		//issue softreset to affect.
		stmTJANXP_SetNormalRegister(0x00,0x0000,0x8000);

		delayms(500);
		stmConfirmLEDBlink();
		return 0;
	}
	printf("Err\r\n");//printErr();
	return -1;
}
void stmTJANXP_showStatusTJANXP(u8 oled){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;

	val	=	stmTJANXP_GetNormalRegister(1,0);
	stmTJANXP_PhyStatus.LinkStatus = (val & 0x04) ? 1 : 0;//1.2
	if(oled){
		if(stmTJANXP_PhyStatus.LinkStatus)  stmOzOLED_printString("+Linked.        ",0,2,15);
		else  							stmOzOLED_printString("-Not Linked     ",0,2,15);
	}else {
		if(stmTJANXP_PhyStatus.LinkStatus) printf("1.2: Linked.\r\n"); else printf("1.2: Not Linked.\r\n");
	}

	val	=	stmTJANXP_GetNormalRegister(0x0A,0);
	stmTJANXP_PhyStatus.fMaster_or_Slave = (val & 0x4000) ? 1 : 0;//10.14
	if(oled){
		if(stmTJANXP_PhyStatus.fMaster_or_Slave)  stmOzOLED_printString("+Master.         ",0,3,15);
		else  							stmOzOLED_printString("-Slave         ",0,3,15);
	}else{
		if(stmTJANXP_PhyStatus.fMaster_or_Slave) printf("10.14: Master.\r\n"); else printf("10.14: Slave.\r\n");
	}

	stmTJANXP_PhyStatus.localRcv_status = (val & 0x2000) ? 1 : 0;//10.13
	if(oled){
		if(stmTJANXP_PhyStatus.localRcv_status)  stmOzOLED_printString("+LocRcv OK.     ",0,4,15);
		else  							stmOzOLED_printString("-LocRcv NOT OK.    ",0,4,15);
	}else{
		if(stmTJANXP_PhyStatus.localRcv_status) printf("10.13: LocalRcv OK.\r\n"); else printf("10.13: LocalRcv NOT OK.\r\n");
	}

	stmTJANXP_PhyStatus.remoteRcv_status = (val & 0x1000) ? 1 : 0;//10.12
	if(oled){
		if(stmTJANXP_PhyStatus.remoteRcv_status)  stmOzOLED_printString("+RemRcv OK.   ",0,5,15);
		else  							stmOzOLED_printString("-RemRcv NOT OK.   ",0,5,15);
	}else{
		if(stmTJANXP_PhyStatus.remoteRcv_status) printf("10.12: RemoteRcv OK.\r\n"); else printf("10.12: RemoteRcv NOT OK.\r\n");
	}

}
//{ "ss",     stmTJANXP_Cmd_showstatus,   " : showstatus <phy addr>" },
int stmTJANXP_Cmd_showstatus(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;

	if(argc!=2){
		printf("Err\r\n");//printErr();
		return -1;
	}
	phyaddr	=	(u32)strtol(argv[1], NULL, 16);
	if(phyaddr == 1){
		TJANXP_showStatusTJANXP(0);

	}else{
		printf("Not yet..\r\n");
	}
	stmConfirmLEDBlink();
}
int stmTJANXP_Cmd_softreset(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	if(argc==2){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		if(phyaddr == 1){
			stmTJANXP_SetNormalRegister(0x00,0x0000,0x8000);//Soft Reset(0.15)
			printf("TJANXP Resetting..\r\n");
			delayms(500);
			stmConfirmLEDBlink();
			return 0;
		}
	}
	printf("Err\r\n");//printErr();
	return -1;
}

void stmTJANXP_Periodic_Handler(){

	//stmTJANXP_ModeSelectorHandler(first);
#if (USE_OLED == 1)
	stmTJANXP_showStatusTJANXP(1);
#else
	stmTJANXP_showStatusTJANXP(0);
#endif

}
extern void ETH_X_BSP_Config(unsigned char PhyAddr);
extern void stm32_Simple_low_level_init(void);//in yEther.c
extern 	void stmConfirmLEDBlink(void);
unsigned int g_sendflag=0;

void stmTJANXP_TestLoop(){
	int i=0;
	char str[10];
	u32 nX, pX, nY, pY;
	int seq=0;
	unsigned char PhyAddr;
	unsigned short ret;

	delayms(100);
	printf("TJANXP\r\n");
	printf("EtherCrafts(R)");



	User_LED_GPIO_setup();

	PhyAddr = 1;

	//Config Eth
	ETH_X_BSP_Config(PhyAddr);//ETH_TJANXP_BSP_Config();
	stm32_Simple_low_level_init();

	//stmTJANXP_GpioConf();// ULED and Buzzer

	//Error Fix Sequence for TJANXP Specific.
	//stmTJANXP_FixErrat1_Registers(); //we exclude this function.
	//stmTJANXP_SupportErrata2(); //prepare for next PHYRSTB input.
	//Additional Setup for TJANXP

	//TJANXP_PTP_En();
	stmTJANXP_SetLedModeAsLinkStatus();

	stmTJANXP_Show_All_Registers();

	seq=0;

	//Every Received Frames will be served at ETH_IRQHandler() //See stm32f4x7_eth_bsp.c and startup_stm32f4xx.c.
	while (1)  {
		ySimpleSend(seq);
		stmConfirmLEDBlink();
		seq++;
		delayms(200);
	}
/*
	//TJANXP_Cmd_help(0,0);
    //printf("CLI>");
	while(1){
		//dtMdioCmd_handler();
	    //TJANXP_Periodic_Handler();

		//for tx and rx test
		//if(g_sendflag){
			ySimpleSend(seq);
			//g_sendflag = 0;
			seq++;
			delayms(2000);
		//}
		//seq++;
		//delayms(100);

		    //stmTJANXP_ModeSetHandler(0);
		    //dtMdio_readSQI(0x19,0);
		    //if(g_br_1c == 0)
		    //	stmTJANXP_StatusHandler(0);
		    //delayms(100);
	}
*/
}
//========= main loop ===
void Loop(){
	int i=0;
	char str[10];
	u32 nX, pX, nY, pY;
	int seq=0;

	delayms(100);
#if (USE_OLED == 1)
	//OLED
	stmOzOLED_init();
	delayms(100);
	stmOzOLED_SendCmd(0x8d);
	delayms(10);
	stmOzOLED_SendCmd(0x14);
	stmOzOLED_printString("Media Converter",0,0,15); //Print the String at top line.
	stmOzOLED_printString("EtherCrafts(R)",0,7,15); //Print the String at bottom line.
#else
	printf("TJANXP\r\n");
	printf("EtherCrafts(R)");
#endif

	//stmTJANXP_GpioConf();// ULED and Buzzer

	TJANXP_Cmd_help(0,0);

	//test MCU with LED blinking 3 times.
	for(i=3;i>=0;i--){
		stmConfirmLEDBlink();
#if (USE_OLED == 1)
		usprintf(str,"%d",i);
		stmOzOLED_printBigNumber(str,10,2,1);
#endif
	}
	GPIO_SetBits(GPIOE, GPIO_Pin_5); //and then, light up ULED0
#if (USE_OLED == 1)
	stmOzOLED_printString("              ",0,2,15); //erase line
	stmOzOLED_printString("              ",0,3,15); //erase line
#endif

	//Error Fix Sequence for TJANXP Specific.
	TJANXP_FixErrat1_Registers_TJANXP(); //we should include this function.
	TJANXP_SupportErrata2(); //prepare for next PHYRSTB input.
	//Additional Setup for TJANXP
	//TJANXP_PTP_En();
	TJANXP_SetLedModeAsLinkStatus();
	TJANXP_Show_All_Registers_TJANXP();

	//Config Eth
	ETH_TJANXP_BSP_Config();
	stm32_Simple_low_level_init();


	TJANXP_Cmd_help(0,0);
    printf("CLI>");
	while(1){
		dtMdioCmd_handler();//dtCmd_handler(); //Need a special handling for register 0x1c....
	    //TJANXP_Periodic_Handler();

		//for tx and rx test
		//if(g_sendflag){
			ySimpleSend(seq);
			//g_sendflag = 0;
			seq++;
			delayms(2000);
		//}
		//seq++;
		//delayms(100);

		    //stmTJANXP_ModeSetHandler(0);
		    //dtMdio_readSQI(0x19,0);
		    //if(g_br_1c == 0)
		    //	stmTJANXP_StatusHandler(0);
		    //delayms(100);
	}
}
//=============== TBD ====
/*
void stmTJANXP_SetupNormalMiiMode(){
	mdio_write(0x18, 0x18, 0xf067); //GMII/MII Mode Setup (use Shadow 7)
	mdio_write(0x18, 0x1c, 0xac01); //3.3V MII Pad Setup
	mdio_write(0x18, 0x17, 0x0f0e); //select expansion register of 0x0e
	mdio_write(0x18, 0x15, 0x0800); //MII-Lite Mode Setup
}

void stmTJANXP_ModeSelectorHandler(u8 first){
	long val1,val2;
	val1 = (GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_2) >> 2) & 0x00000001;//Read Forced1_AN0

	if(first || (stmTJANXP_PhyStatus.fForced_or_AN !=  val1)){
		stmTJANXP_PhyStatus.fForced_or_AN = val1;//Update
		if(stmTJANXP_PhyStatus.fForced_or_AN){
			val2 = (GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) >> 3) & 0x00000001;//Get Master/Slave
			stmTJANXP_PhyStatus.fMaster_or_Slave = val2;
			if(val2){
				//printf("Writing on Phy(0x19) Reg 0x%02x <-- 0x%04x\r\n",  phyaddr, reg, val);
				mdio_write(0x19, 0, 0x0208);//Reg0 set as Master
				printf("*Forced Master\r\n");
				stmOzOLED_printString("*Forced Master ",0,1,15);
			}else{
				mdio_write(0x19, 0, 0x0200);//Reg0 set as Slave
				printf("*Forced Slave\r\n");
				stmOzOLED_printString("*Forced Slave ",0,1,15);
			}

		}else{ //AN
			mdio_write(0x19, 0, 0x1000);//0x1200
			printf(">AutoNeg.\r\n");
			stmOzOLED_printString("*AutoNego. Mode.  ",0,1,15);
		}

	}else if(stmTJANXP_PhyStatus.fForced_or_AN > 0){ //NO Changed. But if it is still forced.
			val2 = (GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) >> 3) & 0x00000001; ////Get Master/Slave
			if(stmTJANXP_PhyStatus.fMaster_or_Slave !=  val2){
				stmTJANXP_PhyStatus.fMaster_or_Slave = val2;
				if(val2){
					mdio_write(0x19, 0, 0x0208); //Reg0 set Master
					printf(">Forced Master\r\n");
					stmOzOLED_printString("*Forced Master ",0,1,15);
				}else{
					mdio_write(0x19, 0, 0x0200); //Reg0 set Slave
					printf(">Forced Slave\r\n");
					stmOzOLED_printString("*Forced Slave ",0,1,15);
				}
			}
	}
	delayms(10);
}
*/


void stmTJANXP_StatusHandler(u8 extended){
	long val32;
	//struct _TJANXP_PhyStatus rdConfig;
	unsigned short linkStatus;
/*
	if(extended){
		stmTJANXP_PhyStatus.regf_extstatus = (u16)mdio_read(0x19, 0x0f); //Extended LRE Status
		if((stmTJANXP_PhyStatus.reg1_status & 0x0300) == 0x0300) stmOzOLED_printString("+Rcv(L/R)=OK/OK",0,2,15);
		else if((stmTJANXP_PhyStatus.reg1_status & 0x0300) == 0x0200) stmOzOLED_printString("-Rcv(L/R)=OK/No",0,2,15);
		else if((stmTJANXP_PhyStatus.reg1_status & 0x0300) == 0x0100) stmOzOLED_printString("-Rcv(L/R)=No/OK",0,2,15);
		else                                               stmOzOLED_printString("-Rcv(L/R)=No/No",0,2,15);

		stmTJANXP_PhyStatus.reg11_phyextstatus = (u16)mdio_read(0x19, 0x11); //PHY Extended Status
		if((stmTJANXP_PhyStatus.reg11_phyextstatus & 0x2000)==0x2000) stmOzOLED_printString("+MDI Crossed   ",0,3,15);
		if((stmTJANXP_PhyStatus.reg11_phyextstatus & 0x0800)==0x0800) stmOzOLED_printString("+RemoteRcv:OK  ",0,4,15);
		else                                               stmOzOLED_printString("-RemoteRcv:FAIL",0,4,15);
		if((stmTJANXP_PhyStatus.reg11_phyextstatus & 0x0400)==0x0400) stmOzOLED_printString("+LocalRcv:OK   ",0,5,15);
		else                                               stmOzOLED_printString("-LocalRcv:FAIL ",0,5,15);
		delayms(2000);

		if((stmTJANXP_PhyStatus.reg11_phyextstatus & 0x0200)==0x0200) stmOzOLED_printString("+DSCR:Locked  ",0,2,15);
		else                                               stmOzOLED_printString("-DSCR:Unlocked",0,2,15);
		if((stmTJANXP_PhyStatus.reg11_phyextstatus & 0x0100)==0x0100) stmOzOLED_printString("+Link:Passed   ",0,3,15);
		else                                               stmOzOLED_printString("-Link:NotPassed",0,3,15);

		delayms(2000);

		stmTJANXP_PhyStatus.reg1a_intstatus = (u16)mdio_read(0x19, 0x1a); //Interrupt Status
		if((stmTJANXP_PhyStatus.reg1a_intstatus & 0x4000)==0x4000) stmOzOLED_printString("+PairSwapped  ",0,2,15);
		if((stmTJANXP_PhyStatus.reg1a_intstatus & 0x0020)==0x0020) stmOzOLED_printString("+RemRcvStsChanged",0,3,15);
		if((stmTJANXP_PhyStatus.reg1a_intstatus & 0x0010)==0x0020) stmOzOLED_printString("+LocRcvStsChanged",0,4,15);
		if((stmTJANXP_PhyStatus.reg1a_intstatus & 0x0004)==0x0004) stmOzOLED_printString("+LinkSpdChanged",0,5,15);
		//if((stmTJANXP_PhyStatus.reg1a_intstatus & 0x0002)==0x0002) stmOzOLED_printString("+LinkStsChanged",0,6,15);
		delayms(2000);
	}
	//Read LRE Status Register
	linkStatus = (u16)mdio_read(0x19, 1); //LRE Status
	if(stmTJANXP_PhyStatus.reg1_status != linkStatus){
		stmTJANXP_PhyStatus.reg1_status = linkStatus;//Update it.
		if(stmTJANXP_PhyStatus.reg1_status & 0x0004){
			stmOzOLED_printString("+Link Up   ",0,3,12);
			printf("+Link Up    \r\n");
			stmTJANXP_PhyStatus.LinkUp = 1;
		}else{
			stmOzOLED_printString("-Link Down  ",0,3,12);
			printf("-Link Down  \r\n");
			stmTJANXP_PhyStatus.LinkUp = 0;
		}
	}
	//Read LED Status Register
	mdio_write(0x19, 0x1c, 0x2000);//1c_shadow_01000 select
	delayms(10);
	stmTJANXP_PhyStatus.reg1c08_ledstatus = (u16)mdio_read(0x19, 0x1c);
	//printf("stmTJANXP_PhyStatus.reg1c08_ledstatus=%04x\r\n",stmTJANXP_PhyStatus.reg1c08_ledstatus);
	if((stmTJANXP_PhyStatus.reg1c08_ledstatus & 0x0080)==0x0080) stmOzOLED_printString("-HDX           ",0,5,15);
	else  											  stmOzOLED_printString("+FDX           ",0,5,15);

	if((stmTJANXP_PhyStatus.reg1c08_ledstatus & 0x0018)==0x0018) stmOzOLED_printString("-NoLink        ",0,6,15);
	else if((stmTJANXP_PhyStatus.reg1c08_ledstatus & 0x0018)==0x0010) stmOzOLED_printString("+10Mbps      ",0,6,15);
	else if((stmTJANXP_PhyStatus.reg1c08_ledstatus & 0x0018)==0x0008) stmOzOLED_printString("+100Mbps     ",0,6,15);
	else  											  stmOzOLED_printString("-NA            ",0,6,15);

	if((stmTJANXP_PhyStatus.reg1c08_ledstatus & 0x0001)==0x0001) stmOzOLED_printString("-PoorQuality   ",0,7,15);
	else  											  stmOzOLED_printString("+GoodQuality   ",0,7,15);


	if(stmTJANXP_PhyStatus.LinkUp){
		if(stmTJANXP_PhyStatus.fForced_or_AN == 1){ //ForcedMode
			if(stmTJANXP_PhyStatus.fMaster_or_Slave == 1)               stmOzOLED_printString("+M/S:Master        ",0,4,15);
			else  											 stmOzOLED_printString("-M/S:Slave         ",0,4,15);
		}else{ //AN Mode
			if((stmTJANXP_PhyStatus.reg1c08_ledstatus & 0x0100)==0x0100) stmOzOLED_printString("+M/S:Master        ",0,4,15);
			else  											  stmOzOLED_printString("-M/S:Slave         ",0,4,15);
		}
	}else{ //Link Down
              stmOzOLED_printString("-M/S: NA    ",0,4,15); //No Master/Slave Status
	}
*/
	return;
}
#endif //PROCESSOR

