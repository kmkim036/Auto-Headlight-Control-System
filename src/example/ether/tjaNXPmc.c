/* TJA1100MC
 * This is the setup/management program for automotive Ethernet media converter module.
 *
 * CHONGHO YOON
 * 2017.07.30
 *
 * HW :
 *   IP101A RXD -->-+
 *   (0x02) TXD<-+  |
 *   (MII  )     |  |
 *               |  |
 *  TJA1100 RXD--+ -|
 *    (0x5) TXD<----+
 *   (RevMII)
 *                 |   MDIO   +------------+
 *                 +--------- |STM32F103MCU|
 *                            |            |
 *                            +------------+
 *  [CONTROL INTERFACE]
 *  MDIO for TJA1100 = 0x05; IP101A=0x02
 *  MDIO/MDC Controlled by LM3S811 MCU.
 *
 *
 *  - This can support
 *      * AutoNegotiated Mode
 *      * Forced Master/Slave Selection
 *      * Report Current Operation Status on OLED
 * [GPIOs]
 * MDIO : PC6
 * MDC  : PD1
 * ETH_ULED0: PD6
 * BUZZER : PC4
 * ULED0(PHY): PD7
 * --EXTRAS----
 * (OLED's I2C)
 * I2CSCL : PB2
 * I2CSDA : PB3
 * nIRQ(PHY) : PD0
 */
/*
  stmEncEthernet.c
 * 	  SPI : 1Mbps, 8 bit mode
 *    Connections to ENC28J60 Ethernet/SPI:
// SPI MODE = 0
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0CLK - PA2
//! - SSI0Fss - PA3 nCS0
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5

 *  Configures the SPI3 Peripheral.
 *  CLK -  PC10
 *  MISO - PC11
 *  MOSI - PC12
 *  nCS0 - PA15
*/

/* ENC28J60 is 3.3V. SPI Mode=0, 8 bit
 * Memory consists of 3-parts : Control Regs(00~1F), EtherBuffer(0000~1FFF), PHY regs(0~1F).
 */
#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6)|| (PROCESSOR == PROCESSOR_STM32F103RCT6)  || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"

#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
#else
#endif

#include "ether/include/ymdio.h"
#include "cmdline.h"

#define USE_TJA1100 1
#define USE_TJA1101 2
#define USE_TJA1102 3
#define USE_TJA USE_TJA1101 //  USE_TJA1100 //
//XX : User Button
//PA3(U2RX) > AN(ON) | FORCED(OFF) -- Input
//PA2(U2TX) > Master(ON) | Slave(OFF) -- Input
//XX : ETH_ULED0
//XX : Buzzer -- Output
#define AN1_FORCE0_PIN 		GPIO_Pin_3 //PA3
#define MASTER1_SLAVE0_PIN  GPIO_Pin_2 //PA2
//MDIO-PB15 @ MOSI
//MDC -PB13 @ SCK
#if (USE_TJA == USE_TJA1100)
#define TJAnxpPhyAddr 0x05
#elif (USE_TJA == USE_TJA1101)
#define TJAnxpPhyAddr 0x04 //6 //TJA1101
#endif

struct _PhyREG TJAnxpReg[]={
		 //--- STANDARD REG.
		{0,0,"BCR","(basic control reg(0))"},
		{1,0,"BSR","(basic status reg(1))"},
		{2,0,"PHYID1:2,0",""},
		{3,0,"PHYID2:3,0",""},
		//--- TJAnxp GENERAL REG.
		{15,0,"ESR","(extended status reg(15))"},
		{17,0,"ECR","(extended control reg(17))"},
		{18,0,"CFR1","(Config Reg 1(18))"}, //Bit15=M1/S0; Bit14=A(1)/M(0)
		{19,0,"CFR2","(Config Reg 2(19))"},
		{20,0,"SECR","(Symbole Error Counter Reg(20))"},
		{21,0,"ISR","(Interrupt Source Reg(21))"},
		{22,0,"IER","(Interrupt Enable Reg(22))"},
		{23,0,"CSR","(Commun. Status Reg(23))"},
		{24,0,"GSR","(general status reg(24))"},
		{25,0,"ESR","(External Status Reg(25))"},
		{26,0,"LFCR","(Link Fail Counter Reg(26))"},
#if (USE_TJA == USE_TJA1101)
		{27,0,"CCCR","(Common Config Reg(27))"},
#endif

};

struct _PhyStatus g_TJAnxp_PhyStatus;
//*****************************************************************************
extern char g_cCmdBuf[CMD_BUF_SIZE];
extern tCmdLineEntry g_sCmdTable[];

u8 stmTjaNxpMC_ModeSelectorHandler(struct _PhyStatus *pTJAnxp_PhyStatus, u8 first);

void stmTjaNxpMC_GpioConf(){
	u8 val8=0;

	GPIO_InitTypeDef GPIO_InitStruct;

	printf("stmTjaNxpMC_GpioConf..\r\n");
	//PA3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = AN1_FORCE0_PIN; //PA3
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	//PA2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = MASTER1_SLAVE0_PIN;//PA2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	delayms(10);

	g_TJAnxp_PhyStatus.fAN_or_Forced = GPIO_ReadInputDataBit(GPIOA,AN1_FORCE0_PIN); //PA3
	g_TJAnxp_PhyStatus.fMaster_or_Slave = GPIO_ReadInputDataBit(GPIOA,MASTER1_SLAVE0_PIN); //PA2

	if(g_TJAnxp_PhyStatus.fAN_or_Forced)
		printf("Configured as Autonomous\r\n");
	else
		printf("Configured as Managed\r\n");

	stmTjaNxpMC_ModeSelectorHandler(&g_TJAnxp_PhyStatus ,1);
	printf("stmTjaNxpMC_GpioConf Done.\r\n");
}
/*
void mTJAnxpMC_ConfirmLEDBlink(void){

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, ~GPIO_PIN_6);//OFF
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, ~GPIO_PIN_7);//OFF
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);//Beep ON
	delayms(100);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);//ON
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);//ON
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);//Beep OFF
}
*/
//See 7.9.5
u32 stmTjaNxpMC_GetNormalRegister(u8 reg){
	u32 retval;

	retval = stmMdioRead(TJAnxpPhyAddr, reg);//mdio_read(TJAnxpPhyAddr, reg); //read reg

	return retval;
}

u32 stmTjaNxpMC_SetNormalRegister(u8 reg, u16 value){
	u32 retval;

	stmMdioWrite(TJAnxpPhyAddr, reg, value);//mdio_write(TJAnxpPhyAddr, reg, value); //Write reg

	return retval;
}

void TJAnxpMC_Show_All_Registers_TJAnxp(){
	u8 i;
	u8 reg;
	u16 page;
	u32 retval;
	char str[16];
	printf("==TJAnxp Regs==\r\n");
	for(i=0;i<16;i++){
		reg =  TJAnxpReg[i].regaddr;
		retval = stmTjaNxpMC_GetNormalRegister(reg);
		printf("%s%s=%04x\r\n",TJAnxpReg[i].regName,TJAnxpReg[i].detail, retval & 0xffff );
#if (USE_OLED == 1)
		sprintf(str,"%s=%04x\r\n",TJAnxpReg[i].regName, retval & 0xffff );
		OzOLED_printString(str,0,4,15); //No Master/Slave Status
#endif
		//mTJAnxpMC_ConfirmLEDBlink();
		delayms(100);
	}
}

void TJAnxpMC_Show_All_Registers_IP101(){
	u8 i;
	u8 reg;
	u16 page;
	u32 val;
	char str[16];
	u32 phyaddr = 0x02;
	printf("==IP101A==\r\n");
	for(i=0;i<32;i++){
		val = stmTjaNxpMC_GetNormalRegister(i);//mdio_read(phyaddr, i);//IP101A
		printf("[Reg %u]=0x%04x\r\n",i, val);
		//mTJAnxpMC_ConfirmLEDBlink();
		delayms(400);
	}
}

void TJAnxpMC_EnConfig(void){
	u32 val;
	printf("Config Enable\r\n");
	val = stmTjaNxpMC_GetNormalRegister(0x11); //Extended Config Reg
	stmTjaNxpMC_SetNormalRegister(0x11,val | 0x0004);//CONFIG_EN=Bit2

	delayms(100);
}

#if (USE_TJA == USE_TJA1100)
void TJAnxpMC_SetLedModeAsLinkStatus(void){
	u32 val;
	printf("Set LED Mode as CRS...\r\n");

	val = stmTjaNxpMC_GetNormalRegister(0x12); //Config Reg
	stmTjaNxpMC_SetNormalRegister(0x12,val | 0x0038);//0xd839); //acts as LED pin (Bit5:4=11(CRS); Bit3=1(LED Enable/Wake NO)
	delayms(500);
}
#endif

#if (USE_TJA == USE_TJA1101)
void TJAnxpMC_SetClockOut(void){
	u32 val;

	val = stmTjaNxpMC_GetNormalRegister(27); //Common Config Reg
	printf("Set CLK_OUT(curReg 27= 0x%04x\r\n",val);
	stmTjaNxpMC_SetNormalRegister(27,val | 0x1000); //enable clkOut.

	delayms(100);
	val = stmTjaNxpMC_GetNormalRegister(27); //Config Reg
	printf("Reg27 = 0x%04x\r\n",val);

	delayms(500);
}
#endif

u8 TJAnxpMC_GetMsModeFromReg(void){
	u32 val;
	u16 m1_s0;

	printf("TJA>Get M/S Mode\r\n");
	val = stmTjaNxpMC_GetNormalRegister(0x012);
	m1_s0 = val & 0x8000;
	if(m1_s0){
		printf("TJA>Master Mode\r\n");
		return 1;
	}
	else {
		printf("TJA>Slave Mode\r\n");
		return 0;
	}
}

/*
void TJAnxpMC_LocalWakeUpEn(void){
	u32 val;
	printf("Set LED Mode as Link Status \r\n");
	//stmTjaNxpMC_SetNormalRegister(0x15,0xd42,0x0001); //Pin34 acts as LED pin. -- Default
	stmTjaNxpMC_SetNormalRegister(0x10,0xd04,0xc627); //0xe627 //enable LED, Link Status,
	//Issue SoftReset to be affected.
	//stmTjaNxpMC_SetNormalRegister(0x00,0x0000,0x8000);//Soft Reset(0.15)
	delayms(500);

}
*/


u8 TJAnxpMC_getLinkStatus(struct _PhyStatus *pTJAnxp_PhyStatus){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	u8  tmpval;
	u8  bChanged = 0;

	val	=	stmTjaNxpMC_GetNormalRegister(1);
	tmpval = (val & 0x04) ? 1 : 0;//1.2
	if(tmpval != pTJAnxp_PhyStatus->LinkStatus){ //changed
		bChanged = 1;
		pTJAnxp_PhyStatus->LinkStatus = tmpval;
	}
	pTJAnxp_PhyStatus->localRcv_status = (val & 0x2000) ? 1 : 0;//10.13
	pTJAnxp_PhyStatus->remoteRcv_status = (val & 0x1000) ? 1 : 0;//10.12

	return bChanged;
}

void TJAnxpMC_show_LinkAndMs_Status(struct _PhyStatus *pTJAnxp_PhyStatus, u8 oled, u8 xoffset)
{
#if (DISPLAY_ON ==DISPLAY_ON_OLED_I2C)
		if(pTJAnxp_PhyStatus->LinkStatus)  OzOLED_printString("+Linked.        ",0+xoffset,2,15);
		else  							OzOLED_printString("-Not Linked     ",0+xoffset,2,15);
		if(pTJAnxp_PhyStatus->fMaster_or_Slave)  OzOLED_printString("+Master.         ",0+xoffset,3,15);
		else  							OzOLED_printString("-Slave         ",0+xoffset,3,15);

#else
		if(pTJAnxp_PhyStatus->LinkStatus) printf("1.2: Linked.\r\n");
		else printf("1.2: Not Linked.\r\n");
		if(pTJAnxp_PhyStatus->fMaster_or_Slave) printf("18.15: Master.\r\n");
		else printf("18.15: Slave.\r\n");
#endif
}

void TJAnxpMC_showStatus(struct _PhyStatus *pTJAnxp_PhyStatus, u8 oled, u8 xoffset){

	TJAnxpMC_show_LinkAndMs_Status(pTJAnxp_PhyStatus, oled, xoffset);
#if (DISPLAY_ON ==DISPLAY_ON_OLED_I2C)
		if(pTJAnxp_PhyStatus->localRcv_status)  OzOLED_printString("+LocRcv OK.     ",0+xoffset,4,15);
		else  							OzOLED_printString("-LocRcv NOT OK.    ",0+xoffset,4,15);
		if(pTJAnxp_PhyStatus->remoteRcv_status)  OzOLED_printString("+RemRcv OK.   ",0+xoffset,5,15);
		else  							OzOLED_printString("-RemRcv NOT OK.   ",0+xoffset,5,15);
#else
		if(pTJAnxp_PhyStatus->localRcv_status) printf("10.13: LocalRcv OK.\r\n"); else printf("10.13: LocalRcv NOT OK.\r\n");
		if(pTJAnxp_PhyStatus->remoteRcv_status) printf("10.12: RemoteRcv OK.\r\n"); else printf("10.12: RemoteRcv NOT OK.\r\n");
#endif
}


void TJAnxpMC_showStatusIP101(u8 oled){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	phyaddr = 2;

	TJAnxpMC_Show_All_Registers_IP101();//IP101G_Show_All_Registers(phyaddr); //IP101

}
//====================================================== cmdline processing =============================
/*
//{ "ss",     TJAnxpMC_Cmd_showstatus,   " : showstatus <phy addr>" },
int TJAnxpMC_Cmd_showstatus(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;

	if(argc!=2){
		printErr();
		return -1;
	}
	phyaddr	=	(u32)strtol(argv[1], NULL, 16);
	if(phyaddr == 1){
		TJAnxpMC_getLinkStatusTJAnxp(&g_TJAnxp_PhyStatus);
		TJAnxpMC_showStatusTJAnxp(&g_TJAnxp_PhyStatus, 0, 0);

	}else{
		TJAnxpMC_Show_All_Registers_IP101(0);//TJAnxpMC_showStatusIP101(0);
	}
	mTJAnxpMC_ConfirmLEDBlink();
	return 0;
}
int TJAnxpMC_Cmd_softreset(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	if(argc==2){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		if(phyaddr == 1){
			stmTjaNxpMC_SetNormalRegister(0x00,0x0000,0x8000);//Soft Reset(0.15)
			printf("TJAnxp Resetting..\r\n");
			delayms(500);
			mTJAnxpMC_ConfirmLEDBlink();
			return 0;
		}else if(phyaddr==2){ //IP101A
			mdio_write(phyaddr, 0x00, 0x8000);
			printf("IP101A Resetting..\r\n");
			mTJAnxpMC_ConfirmLEDBlink();
			return 0;
		}
	}
	printErr();
	return -1;
}


int TJAnxpMC_Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;
    //
    // Print some header text.
    //
    printf("\r\nEtherPhyManager by EtherCrafts(r)\r\n");
    printf("Available commands\r\n");
    printf("------------------\r\n");
    printf("PHYAddr: TJAnxp=1; IP101ALF=2\r\n");
    pEntry = &g_sCmdTable_TJAnxp[0];//&g_sCmdTable[0];
    while(pEntry->pcCmd)
    {
        printf("%s%s\r\n", pEntry->pcCmd, pEntry->pcHelp);
        pEntry++;
    }
    mTJAnxpMC_ConfirmLEDBlink();
    return(0);    // Return success.
}

//{ "r",      TJAnxpMC_Cmd_read,   "  : read <phy addr> <reg addr> <page>" },
int TJAnxpMC_Cmd_read(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	int strlen;
	char str[20];

	if(argc==4){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[2], NULL, 16);
		page=	    (u16)strtol(argv[3], NULL, 16);
		val	=	stmTjaNxpMC_GetNormalRegister(reg, page);
		printf("0x%04x\r\n", val);
		strlen = sprintf(str,"R>%02x(%02x)=0x%04x", phyaddr, reg, val);
		//OzOLED_printString(str, 0, 7, 15);//strlen);
		mTJAnxpMC_ConfirmLEDBlink();
		return 0;
	}else if(argc==3){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[2], NULL, 16);
		val = mdio_read(phyaddr, reg);//IP101A
		printf("0x%04x\r\n", val);
		mTJAnxpMC_ConfirmLEDBlink();
	}else{
		printErr();
		return -1;
	}
}
//{ "ra",     TJAnxpMC_Cmd_readAll,   " : read all <phy addr>" },
int TJAnxpMC_Cmd_readAll(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	int strlen;
	char str[20];
	//read <phy addr> <reg addr>
	if(argc==2){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		if(phyaddr == 1)
			TJAnxpMC_Show_All_Registers_TJAnxp(); //TJAnxp
		else {
			TJAnxpMC_Show_All_Registers_IP101(); //IP101
		}
		mTJAnxpMC_ConfirmLEDBlink();
		return 0;

	}else{
		printErr();
		return -1;
	}
}
//{ "w",      TJAnxpMC_Cmd_write,   " : write <phy addr> <reg addr> <page> <value>" },
int TJAnxpMC_Cmd_write(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	u16 page;
	if(argc==5){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[2], NULL, 16);
		page = (u16)strtol(argv[3], NULL, 16);
		val	=	    (u32)strtol(argv[4], NULL, 16);
		stmTjaNxpMC_SetNormalRegister(reg, page, val);
		printf("Writing on Phy(0x%02x) Reg 0x%02x (page=0x%04x)<-- 0x%04x\r\n",  phyaddr, reg,page, val);
		mTJAnxpMC_ConfirmLEDBlink();
		return 0;
	}else if(argc==4){ //IP101A
			phyaddr	=	(u32)strtol(argv[1], NULL, 16);
			reg	=	    (u32)strtol(argv[2], NULL, 16);
			val	=	    (u32)strtol(argv[3], NULL, 16);
			mdio_write(phyaddr, reg, val);
			printf("Writing on Phy(0x%02x) Reg 0x%02x <-- 0x%04x\r\n",  phyaddr, reg, val);
			mTJAnxpMC_ConfirmLEDBlink();
			return 0;

	}else{
		printErr();
		return -1;
	}
}

int TJAnxpMC_Cmd_setms(int argc, char *argv[]){
	u32 ms;
	u32 reg;
	u32 val;
	u16 page;
	if(argc==2){
		ms	=	(u32)strtol(argv[1], NULL, 16);
		if(ms){
			stmTjaNxpMC_SetNormalRegister(0x9,0x0000,0x0800);//master/slave select(9.11)
			printf("Set as Master.\r\n");
		}else{
			stmTjaNxpMC_SetNormalRegister(0x9,0x0000,0x0000);//master/slave select(9.11)
			printf("Set as Slave.\r\n");
		}
		//issue softreset to affect.
		stmTjaNxpMC_SetNormalRegister(0x00,0x0000,0x8000);

		delayms(500);
		mTJAnxpMC_ConfirmLEDBlink();
		return 0;
	}
	printErr();
	return -1;
}
*/

//#define AN1_FORCE0_PIN 		GPIO_Pin_3 //PA3
//#define MASTER1_SLAVE0_PIN  GPIO_Pin_2 //PA2

u8 stmTjaNxpMC_ModeSelectorHandler(struct _PhyStatus *pTJAnxp_PhyStatus, u8 first){
	long val1,val2;
	unsigned short vals;
	unsigned char bChanged = 0;
	//printf("stmTjaNxpMC_ModeSelectorHandler..\r\n");
	if(first){
		val1 = GPIO_ReadInputDataBit(GPIOA, AN1_FORCE0_PIN);
		pTJAnxp_PhyStatus->fAN_or_Forced = val1;
		vals = stmTjaNxpMC_GetNormalRegister(0x12); //0x12 = ConfigReg1.
		if(val1){ //AN
			stmTjaNxpMC_SetNormalRegister(0x12,vals | 0x4000);//Bit14 = 1
			printf("Set as AN.\r\n");
			//OzOLED_printString("*AN ",0,1,15);
		}else{
			stmTjaNxpMC_SetNormalRegister(0x12,vals & 0xbfff);//BIt14 = 0
			printf("Set as Managed.\r\n");
			//OzOLED_printString("*Managed ",0,1,15);
		}
		//
		val2 = GPIO_ReadInputDataBit(GPIOA, MASTER1_SLAVE0_PIN);
		pTJAnxp_PhyStatus->fMaster_or_Slave = val2;
		vals = stmTjaNxpMC_GetNormalRegister(0x12);
		if(val2){ //master
			stmTjaNxpMC_SetNormalRegister(0x12,vals | 0x8000);//master/slave select = 1
			printf("Set as Master.\r\n");
			//OzOLED_printString("*Master ",0,1,15);
		}else{
			stmTjaNxpMC_SetNormalRegister(0x12,vals & 0x7fff);//master/slave select = 0
			printf("Set as Slave.\r\n");
			//OzOLED_printString("*Slave ",0,1,15);
		}
		 bChanged = 1;
	}else{
		val1 = GPIO_ReadInputDataBit(GPIOA, AN1_FORCE0_PIN);
		//if Changed...
		if(pTJAnxp_PhyStatus->fAN_or_Forced !=  val1){
			pTJAnxp_PhyStatus->fAN_or_Forced = val1;
			vals = stmTjaNxpMC_GetNormalRegister(0x12); //0x12 = ConfigReg1.
			if(val1){ //AN
				stmTjaNxpMC_SetNormalRegister(0x12,vals | 0x4000);//Bit14 = 1
				printf("Set as AN.\r\n");
				//OzOLED_printString("*AN ",0,1,15);
			}else{
				stmTjaNxpMC_SetNormalRegister(0x12,vals & 0xbfff);//BIt14 = 0
				printf("Set as Managed.\r\n");
				//OzOLED_printString("*Managed ",0,1,15);
			}
		}
		//

		val2 = GPIO_ReadInputDataBit(GPIOA, MASTER1_SLAVE0_PIN);
		if(pTJAnxp_PhyStatus->fMaster_or_Slave !=  val2){
			pTJAnxp_PhyStatus->fMaster_or_Slave = val2;
			vals = stmTjaNxpMC_GetNormalRegister(0x12);
			if(val2){
				stmTjaNxpMC_SetNormalRegister(0x12,vals | 0x8000);//master/slave select = 1
				printf("Set as Master.\r\n");
				//OzOLED_printString("*Master ",0,1,15);
			}else{
				stmTjaNxpMC_SetNormalRegister(0x12,vals & 0x7fff);//master/slave select = 0
				printf("Set as Slave.\r\n");
				//OzOLED_printString("*Slave ",0,1,15);
			}
			bChanged = 1;
		}
	}
	return bChanged;
}

void TJAnxpMC_Periodic_Handler(u8 first){
	u8 bChanged;

	bChanged = stmTjaNxpMC_ModeSelectorHandler(&g_TJAnxp_PhyStatus ,first);
	if(bChanged) TJAnxpMC_showStatus(&g_TJAnxp_PhyStatus, 0, 0);

	bChanged = TJAnxpMC_getLinkStatus(&g_TJAnxp_PhyStatus);
#if (USE_OLED == 1)
	TJAnxpMC_showStatus(&g_TJAnxp_PhyStatus, 1, 0);
#else
	if(bChanged) TJAnxpMC_showStatus(&g_TJAnxp_PhyStatus, 0, 0);
#endif

}

extern int g_mdioSomeDelay ;

//========= main loop ===
void stmTjaNxpMC_Loop(){
	int i=0;
	char str[10];
	u32 nX, pX, nY, pY;

	g_mdioSomeDelay = 500;

	delayms(100);
#if (USE_OLED == 1)
	//OLED
	OzOLED_init();
	delayms(100);
	OzOLED_sendCommand(0x8d);
	delayms(10);
	OzOLED_sendCommand(0x14);
	OzOLED_printString("Media Converter",0,0,15); //Print the String at top line.
	OzOLED_printString("EtherCrafts(R)",0,7,15); //Print the String at bottom line.
#else
	printf("Media Converter\r\n");
	printf("EtherCrafts(R)\r\n");
#endif

	stmTjaNxpMC_GpioConf();

/*
	//ETC LED CONFIG
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, ~GPIO_PIN_6);//OFF
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5);//OFF
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, ~GPIO_PIN_4);//OFF
	if(PHY_ACCESS_METHOD == SMIIF){
		//Config GPIO PD6 (LM3S811)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);//ON
	}else if(PHY_ACCESS_METHOD == I2CIF){//I2C
		//Config GPIO PD5 (LM3S811)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_PIN_5);//ON
	}else{//SPI
		//Config GPIO PD4 (LM3S811)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4);//ON
	}
*/
	//Joystick
	//A = -X; B=+Y; C=-Y; D=+X
	//-X (A) = PB1
	//+X (D) = PB6
	//+Y (B) = PB4
	//-Y (C) = PB5
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_6 |GPIO_PIN_4 | GPIO_PIN_5);

	//Read Joystick Test
/*	printf("TEST: Press Joystick\r\n");
	for(i=4;i>0;i--){
	nX = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1); //-X
	pX = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6); //+X
	nY = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_5); //-Y
	pY = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4); //+Y

	if(nX & GPIO_PIN_1) printf("-X");
	if(pX & GPIO_PIN_6) printf("+X");
	if(nY & GPIO_PIN_5) printf("-Y");
	if(pY & GPIO_PIN_4) printf("+Y");
	delayms(1000);
	}
*/
	//init_MDIO();
	stmMdioInitPins(); //with bitbang

	stmMdioGetPhyID();

	//TJAnxpMC_Cmd_help(0,0);
	//printf("CMD# "); //UARTSend((u8 *)"CMD# ", 5);

	//test MCU with LED blinking 3 times.
	for(i=3;i>=0;i--){
		//printf("A");
		//dtUserLed0Control(1);
		//beep(50);
#if (USE_OLED == 1)
		usprintf(str,"%d",i);
		OzOLED_printBigNumber(str,10,2,1);
#endif
		delayms(200);
		//dtUserLed0Control(0);
		//delayms(200);
	}
	//dtUserLed0Control(1); //and then, light up ULED0
#if (USE_OLED == 1)
	OzOLED_printString("              ",0,2,15); //erase line
	OzOLED_printString("              ",0,3,15); //erase line
#endif
	//printf("\r\nWait...\r\n");


	//TJAnxpMC_PTP_En();
	//TJAnxpMC_Show_All_Registers_TJAnxp();
	//TJAnxpMC_Show_All_Registers_IP101();

	//TJAnxpMC_Cmd_help(0,0);
    //printf("CLI>");

	TJAnxpMC_getLinkStatus(&g_TJAnxp_PhyStatus);


	TJAnxpMC_Show_All_Registers_TJAnxp();

	TJAnxpMC_EnConfig();

#if (USE_TJA == USE_TJA1100)
	TJAnxpMC_SetLedModeAsLinkStatus();
#elif (USE_TJA == USE_TJA1101)
	 TJAnxpMC_SetClockOut(); //set CLK_OUT.
#endif

	printf("Done.\r\n"); //UARTSend((u8 *)"CMD# ", 5);

	while(1){
		    //dtCmd_handler(); //Need a special handling for register 0x1c....
		    TJAnxpMC_Periodic_Handler(0);
	}
}



/*void stmTjaNxpMC_ModeSelectorHandler(u8 first){
	long val1,val2;
	val1 = (GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_2) >> 2) & 0x00000001;//Read Forced1_AN0

	if(first || (TJAnxp_PhyStatus.fAN_or_Forced !=  val1)){
		TJAnxp_PhyStatus.fAN_or_Forced = val1;//Update
		if(TJAnxp_PhyStatus.fAN_or_Forced){
			val2 = (GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) >> 3) & 0x00000001;//Get Master/Slave
			TJAnxp_PhyStatus.fMaster_or_Slave = val2;
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

	}else if(TJAnxp_PhyStatus.fAN_or_Forced > 0){ //NO Changed. But if it is still forced.
			val2 = (GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) >> 3) & 0x00000001; ////Get Master/Slave
			if(TJAnxp_PhyStatus.fMaster_or_Slave !=  val2){
				TJAnxp_PhyStatus.fMaster_or_Slave = val2;
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
void stmTjaNxpMC_StatusHandler(u8 extended){
	long val32;
	//struct _TJAnxp_PhyStatus rdConfig;
	unsigned short linkStatus;

	if(extended){
		TJAnxp_PhyStatus.regf_extstatus = (u16)mdio_read(0x19, 0x0f); //Extended LRE Status
		if((TJAnxp_PhyStatus.reg1_status & 0x0300) == 0x0300) OzOLED_printString("+Rcv(L/R)=OK/OK",0,2,15);
		else if((TJAnxp_PhyStatus.reg1_status & 0x0300) == 0x0200) OzOLED_printString("-Rcv(L/R)=OK/No",0,2,15);
		else if((TJAnxp_PhyStatus.reg1_status & 0x0300) == 0x0100) OzOLED_printString("-Rcv(L/R)=No/OK",0,2,15);
		else                                               OzOLED_printString("-Rcv(L/R)=No/No",0,2,15);

		TJAnxp_PhyStatus.reg11_phyextstatus = (u16)mdio_read(0x19, 0x11); //PHY Extended Status
		if((TJAnxp_PhyStatus.reg11_phyextstatus & 0x2000)==0x2000) OzOLED_printString("+MDI Crossed   ",0,3,15);
		if((TJAnxp_PhyStatus.reg11_phyextstatus & 0x0800)==0x0800) OzOLED_printString("+RemoteRcv:OK  ",0,4,15);
		else                                               OzOLED_printString("-RemoteRcv:FAIL",0,4,15);
		if((TJAnxp_PhyStatus.reg11_phyextstatus & 0x0400)==0x0400) OzOLED_printString("+LocalRcv:OK   ",0,5,15);
		else                                               OzOLED_printString("-LocalRcv:FAIL ",0,5,15);
		delayms(2000);

		if((TJAnxp_PhyStatus.reg11_phyextstatus & 0x0200)==0x0200) OzOLED_printString("+DSCR:Locked  ",0,2,15);
		else                                               OzOLED_printString("-DSCR:Unlocked",0,2,15);
		if((TJAnxp_PhyStatus.reg11_phyextstatus & 0x0100)==0x0100) OzOLED_printString("+Link:Passed   ",0,3,15);
		else                                               OzOLED_printString("-Link:NotPassed",0,3,15);

		delayms(2000);

		TJAnxp_PhyStatus.reg1a_intstatus = (u16)mdio_read(0x19, 0x1a); //Interrupt Status
		if((TJAnxp_PhyStatus.reg1a_intstatus & 0x4000)==0x4000) OzOLED_printString("+PairSwapped  ",0,2,15);
		if((TJAnxp_PhyStatus.reg1a_intstatus & 0x0020)==0x0020) OzOLED_printString("+RemRcvStsChanged",0,3,15);
		if((TJAnxp_PhyStatus.reg1a_intstatus & 0x0010)==0x0020) OzOLED_printString("+LocRcvStsChanged",0,4,15);
		if((TJAnxp_PhyStatus.reg1a_intstatus & 0x0004)==0x0004) OzOLED_printString("+LinkSpdChanged",0,5,15);
		//if((TJAnxp_PhyStatus.reg1a_intstatus & 0x0002)==0x0002) OzOLED_printString("+LinkStsChanged",0,6,15);
		delayms(2000);
	}
	//Read LRE Status Register
	linkStatus = (u16)mdio_read(0x19, 1); //LRE Status
	if(TJAnxp_PhyStatus.reg1_status != linkStatus){
		TJAnxp_PhyStatus.reg1_status = linkStatus;//Update it.
		if(TJAnxp_PhyStatus.reg1_status & 0x0004){
			OzOLED_printString("+Link Up   ",0,3,12);
			printf("+Link Up    \r\n");
			TJAnxp_PhyStatus.LinkUp = 1;
		}else{
			OzOLED_printString("-Link Down  ",0,3,12);
			printf("-Link Down  \r\n");
			TJAnxp_PhyStatus.LinkUp = 0;
		}
	}
	//Read LED Status Register
	mdio_write(0x19, 0x1c, 0x2000);//1c_shadow_01000 select
	delayms(10);
	TJAnxp_PhyStatus.reg1c08_ledstatus = (u16)mdio_read(0x19, 0x1c);
	//printf("TJAnxp_PhyStatus.reg1c08_ledstatus=%04x\r\n",TJAnxp_PhyStatus.reg1c08_ledstatus);
	if((TJAnxp_PhyStatus.reg1c08_ledstatus & 0x0080)==0x0080) OzOLED_printString("-HDX           ",0,5,15);
	else  											  OzOLED_printString("+FDX           ",0,5,15);

	if((TJAnxp_PhyStatus.reg1c08_ledstatus & 0x0018)==0x0018) OzOLED_printString("-NoLink        ",0,6,15);
	else if((TJAnxp_PhyStatus.reg1c08_ledstatus & 0x0018)==0x0010) OzOLED_printString("+10Mbps      ",0,6,15);
	else if((TJAnxp_PhyStatus.reg1c08_ledstatus & 0x0018)==0x0008) OzOLED_printString("+100Mbps     ",0,6,15);
	else  											  OzOLED_printString("-NA            ",0,6,15);

	if((TJAnxp_PhyStatus.reg1c08_ledstatus & 0x0001)==0x0001) OzOLED_printString("-PoorQuality   ",0,7,15);
	else  											  OzOLED_printString("+GoodQuality   ",0,7,15);


	if(TJAnxp_PhyStatus.LinkUp){
		if(TJAnxp_PhyStatus.fForced_or_AN == 1){ //ForcedMode
			if(TJAnxp_PhyStatus.fMaster_or_Slave == 1)               OzOLED_printString("+M/S:Master        ",0,4,15);
			else  											 OzOLED_printString("-M/S:Slave         ",0,4,15);
		}else{ //AN Mode
			if((TJAnxp_PhyStatus.reg1c08_ledstatus & 0x0100)==0x0100) OzOLED_printString("+M/S:Master        ",0,4,15);
			else  											  OzOLED_printString("-M/S:Slave         ",0,4,15);
		}
	}else{ //Link Down
              OzOLED_printString("-M/S: NA    ",0,4,15); //No Master/Slave Status
	}

	return;
}
*/
/*void stmTjaNxpMC_SetupNormalMiiMode(){
	mdio_write(0x18, 0x18, 0xf067); //GMII/MII Mode Setup (use Shadow 7)
	mdio_write(0x18, 0x1c, 0xac01); //3.3V MII Pad Setup
	mdio_write(0x18, 0x17, 0x0f0e); //select expansion register of 0x0e
	mdio_write(0x18, 0x15, 0x0800); //MII-Lite Mode Setup
}*/
