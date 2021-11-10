//Not Bitbang, but STM's MDIO/MDC based
#include <stdio.h>
 #include <stdint.h>
#include <string.h>
#include "cmdline.h"
#include "yLib/eth/include/ethopts.h"
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
#elif (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
//#include "stm32f4xx_dcmi.h"
//#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "misc.h"
#include "yLib/eth/include/stm32f4x7_eth.h"
#include "yLib/eth/include/stm32f4x7_eth_bsp.h"
#include "yLib/eth/include/ethernetif.h" //#include "lwip/include/ethernetif.h"
#else
//printf("Error. Not support here.\r\n");
#endif
#include "ymdio.h"
extern unsigned char g_macaddrLsb;

#if (CMDLINE_USE_FOR == CMDLINE_MDIO407_107) || (CMDLINE_USE_FOR == CMDLINE_SJA1105)

extern __IO uint32_t  gEthInitStatus;
extern __IO uint8_t EthLinkStatus;

#define	PARAM_MAXNUM	10
#define	PARAM_MAXLEN	20
#define BASE      16.0
#define NULL 0

char	line[CMD_BUF_SIZE];
unsigned char  line_char_cnt	=	0;
volatile unsigned char	inputed	=	0;

int g_br_1c = 0; //broadR specific flag -- mutex of reg 1C.

struct _stmMdioPhy{
	//u8 platform;
	//u8 use_oled;
	u8 phyaddr_set_done;
	//struct _PhyStatus PhyStatus;
	//struct GenealPhyREGS GeneralPhyRegs;
	u8 phyaddr;
	//char *phyname;
	//u32 phyid32;
	//u16  clause45;
	u8 f_preamblesuppressed;
} stmMdioPhy;

//register 2.
struct PhyName PhyNames[20]={
		{0x2000,"TI"},
		{0x004d,"Atheros"},
		{0x001c,"Realtek"},
		{0x0243,"ICplus"},
		{0x0022,"Micrel"}
};

extern void OzOLED_printString(const char *String, unsigned char X, unsigned char Y, unsigned char numChar);
static void stmOutputBitStreamMdio(unsigned val, unsigned n);
static void stmTurnaround_MDIO(void);
static unsigned int stmInputStreamMDIO(void);
void UARTIntHandler(void);
//static int parsing_line(s8 line[], s8 param[][PARAM_MAXLEN], unsigned int len);
//static int proc_cmd(s8 param[][PARAM_MAXLEN], int param_num);
//static void print_menu(void);
int stmMdio407_107Cmd_help(int argc, char *argv[]);
int stmMdio407_107Cmd_read(int argc, char *argv[]);
int stmMdio407_107Cmd_readSpan(int argc, char *argv[]);
int stmMdio407_107Cmd_readSQI(int argc, char *argv[]); //BroadR specific for reading SQI
int stmMdio407_107Cmd_write(int argc, char *argv[]);
//*****************************************************************************
extern char g_cCmdBuf[CMD_BUF_SIZE];
//extern struct _usart1rxbuf g_usart1rxbuf; //STM32 Specific
extern struct _usartXrxbuf g_usartXrxbuf; //STM32 Specific

// This is the table that holds the command names, implementing functions, and brief description.
extern tCmdLineEntry g_sCmdTable[];

//call after ETH_X_BSP_Config
extern void ETH_GPIO_Config(void);
extern void ETH_MACDMA_Config(unsigned short PHYAddress);

char *mdioFindPhyMaker(unsigned short prodId){
	int i;
	for(i=0;i<20;i++){
		if(PhyNames[i].prodId == prodId)
			return PhyNames[i].namestr;
	}
	return "NotFound";
}

//Oncifg GPIO only for MDIO/MDC pins
//107 : MDIO = PA2
//    : MDC  = PC1
void stm407_107Mdio_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO clocks for MDIO/MDC
#if(PROCESSOR == PROCESSOR_STM32F407VGT6)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC,  ENABLE);
	//Enable SysCfg Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#elif(PROCESSOR == PROCESSOR_STM32F107VCT)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC,  ENABLE);
#endif


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

#if 0
#if (MII_RMII == MII_MODE)
  //Select MII mode : Setting mode is allowed under reset or before enabling MAC clocks.
  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_MII); //0 = MII/ 1= RMII

#elif (MII_RMII == RMII_MODE)  /* Mode RMII with STM324xG-EVAL */
//#error using RMII_MODE !!!!!
  	  /* Output PLL clock divided by 2 (50MHz) on MCO pin (PA8) to clock the PHY */
  	  //RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_2);
  	  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);
  	printf("***RMII***\r\n");
#endif

  delayms(200);
#endif
  //GPIO_SetBits(GPIOC, GPIO_Pin_8); //nRESET_PHY TO HIGH
  //delayms(10);

/* Ethernet pins configuration (YOON)************************************************/
   /* 407
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
#if(PROCESSOR == PROCESSOR_STM32F407VGT6)
  /* Enable GPIOs clocks for Ethernet */
  // Configure PA2(MDIO)
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //?? NO GPIO_OType_AF
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH); //MDIO

  //Continue... Configure PC1(MDC)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH); //MDC
#elif(PROCESSOR == PROCESSOR_STM32F107VCT)
  //Configure MII_MDIO (PA2) - AF Output Push Pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //Configure MII_MDC (PC1) - AF Output Push Pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

  delayms(200);

}

void stm407_107Mdio_MAC_Config(unsigned short PHYAddress)// ETH_MACDMA_Config(unsigned short PHYAddress)
{
	ETH_InitTypeDef ETH_InitStructure;
	//Start MAC Clocking
#if(PROCESSOR == PROCESSOR_STM32F407VGT6)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC, ENABLE);//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx | RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);
#elif(PROCESSOR == PROCESSOR_STM32F107VCT)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx | RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);
#endif
	delayms(1); //should be ...

	/* Reset ETHERNET on AHB Bus */
	ETH_DeInit(); //Deinitializes the ETHERNET peripheral registers to their default reset values.

	/* Software reset -- Not Working*/ //YOON-----?????
#if 0
	ETH_SoftwareReset(); //resets all MAC subsystem internal registers and logic
	while (ETH_GetSoftwareResetStatus() == SET); //Never return 0. YOON....
#endif
	//Adjust MDC Clock Range depending on HCLK -- YOON -- Will do it in ETH_Init()
	//ETH->MACMIIAR = ETH_MACMIIAR_CR_Div102; //for 168MHz

	ETH_StructInit(&ETH_InitStructure);
#if 0
	ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;//ETH_AutoNegotiation_Disable;//ETH_AutoNegotiation_Enable;// ETH_AutoNegotiation_Disable;
	ETH_InitStructure.ETH_Speed = ETH_Speed_100M;
	ETH_InitStructure.ETH_InterFrameGap = ETH_InterFrameGap_96Bit; //Added
	ETH_InitStructure.ETH_ReceiveOwn = ETH_ReceiveOwn_Enable; //Added
	ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
	ETH_InitStructure.ETH_CarrierSense = ETH_CarrierSense_Disable; //YOON : VERY CRITICAL
	ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;//ETH_LoopbackMode_Enable;// Need RXCLK for Loopback.
	ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
	ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable; //ETH_AutomaticPadCRCStrip_Enable;
	ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Enable; //Unicast and Multicast Frames. Promiscuous Mode?
	ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
	ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Enable;//YOON : VERY CRITICAL FOR MULTICAST
	ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_None;//ETH_MulticastFramesFilter_Perfect;
	ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
	ETH_InitStructure.ETH_VLANTagComparison = ETH_VLANTagComparison_16Bit;
	ETH_InitStructure.ETH_VLANTagIdentifier = 0x00;//NO.    0x02; //For AVB


  /*------------------------   DMA   -----------------------------------*/

  /* When we use the Checksum offload feature, we need to enable the Store and Forward mode:
  the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum,
  if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
  ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;
  ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
  ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;
  ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;
  ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;
  ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;
  ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
  ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;
  ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_8Beat;//ETH_RxDMABurstLength_32Beat;//ETH_RxDMABurstLength_8Beat;//ETH_RxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_TxDMABurstLength = ETH_RxDMABurstLength_8Beat;//ETH_TxDMABurstLength_32Beat;//ETH_RxDMABurstLength_8Beat;//ETH_TxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_1_1;//ETH_DMAArbitration_RoundRobin_RxTx_1_1;//ETH_DMAArbitration_RoundRobin_RxTx_2_1;
  ETH_InitStructure.ETH_DescriptorSkipLength = 0x0;// Added
#endif
  /* Configure Ethernet */
  gEthInitStatus = ETH_Init(&ETH_InitStructure, PHYAddress);//IP101A_PHY_ADDRESS);//BROADR_PHY_ADDRESS);//IP101A_PHY_ADDRESS);//was KSZ8051_PHY_ADDRESS);

  //Disable all MAC Interrupts for MDIO
  ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T, DISABLE); //NIS=Normal interrupt summary; IT_R=Receive interrupt; T=Tx Interrupt(YOON add)

  printf("ETH_MACDMA_Config Pass\r\n");

}
void stmMdio407_107Config(unsigned char PhyAddr, char *str)
{
	int i;
	unsigned short retv;
	unsigned int result;

	stm407_107Mdio_GPIO_Config();//ETH_GPIO_Config(); 	//Setup GPIO and MII Mode before MAC Clocking.

	// Config NVIC for Ethernet
	//ETH_NVIC_Config();
	//printf("ETH_NVIC_Config...ok \r\n");
	//delayms(10);

	//Configure the Ethernet MAC/DMA
	stm407_107Mdio_MAC_Config(PhyAddr);//ETH_MACDMA_Config(PhyAddr);////After GPIO Config, we start MAC Clocking
	delayms(10);

	  if (gEthInitStatus == 0) {
	//    while(1){
	    	printf("EthInitStatus=0 --> Error\r\n"); delayms(100);
	//    }
	  }else
		  printf("EthInit OK\r\n");
#if(PROCESSOR == PROCESSOR_STM32F407VGT6)
	  //==== Get PHY Product ID
	  // Read PHY ID register2 and 3: Should be 0x0180 and 0xdc48
	  retv = ETH_ReadPHYRegister(PhyAddr, 0x02, &result);
	  if(result == 1)
		  printf("PHYID Reg2 = 0x%04x(%s) and ",retv,mdioFindPhyMaker(retv));
	  else
		  printf("PHYID Reg2 = NOT AVAILABLE\r\n");
	  retv = ETH_ReadPHYRegister(PhyAddr, 0x03, &result);
	  if(result == 1)
		  printf("PHYID Reg3 = 0x%04x \r\n",retv);
	  else
		  printf("PHYID Reg3 = NOT AVAILABLE\r\n");
	  // Read PHY status register: Get Ethernet link status
	  retv = ETH_ReadPHYRegister(PhyAddr, 0x01, &result);
	  printf("Read Reg01 = 0x%04\r\n", retv);
	  if(retv & 0x04)  { //LRE Status Register(01:2) == 1-> Link UP
		  EthLinkStatus |= ETH_LINK_FLAG;  //EthStatus |= ETH_LINK_FLAG;
	  }
	  if(EthLinkStatus & ETH_LINK_FLAG)
		  printf("Link Up\r\n");
	  else
		  printf("Link Down\r\n");
#elif(PROCESSOR == PROCESSOR_STM32F107VCT)
	  //==== Get PHY Product ID
	  // Read PHY ID register2 and 3: Should be 0x0180 and 0xdc48
	  result = ETH_ReadPHYRegister(PhyAddr, 0x02);
	  printf("PHYID Reg2 = 0x%04x(%s) and ",result ,mdioFindPhyMaker(result));

	  result = ETH_ReadPHYRegister(PhyAddr, 0x03);
	  printf("PHYID Reg3 = 0x%04x \r\n",result);

	  // Read PHY status register: Get Ethernet link status
	  result = ETH_ReadPHYRegister(PhyAddr, 0x01);
	  printf("Read Reg01 = 0x%04\r\n", result);
	  if(result & 0x04)  { //LRE Status Register(01:2) == 1-> Link UP
		  EthLinkStatus |= ETH_LINK_FLAG;  //EthStatus |= ETH_LINK_FLAG;
	  }
	  if(EthLinkStatus & ETH_LINK_FLAG)
		  printf("Link Up\r\n");
	  else
		  printf("Link Down\r\n");
#endif
}

unsigned int stmMdio407Read(unsigned int PhyAddr, unsigned int PhyReg){
	unsigned short retv;
	unsigned int result;
#if(PROCESSOR == PROCESSOR_STM32F407VGT6)
	retv = ETH_ReadPHYRegister(PhyAddr, PhyReg, &result);
	if(result == 0){
		printf("NOT AVAILABLE\r\n");
		return 0xFFFFFFFF;
	}else
		return (unsigned int)retv;
#elif(PROCESSOR == PROCESSOR_STM32F107VCT)
	result = ETH_ReadPHYRegister(PhyAddr, PhyReg);
	if(result == 0){
		printf("NOT AVAILABLE\r\n");
		return 0xFFFFFFFF;
	}else
		return (unsigned int)result;
#endif
}

void stmMdio407Write(unsigned int PhyAddr, unsigned int PhyReg, unsigned int val){
	unsigned int retv;
	retv = ETH_WritePHYRegister((unsigned short)PhyAddr, (unsigned short)PhyReg,(unsigned short)val);
	if(retv = ETH_ERROR){
		printf("Write Error\r\n");
	}
}
int stmMdio407_107Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    printf("\r\nAvailable commands\r\n");
    printf("------------------\r\n");

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
    g_br_1c = 0; //broadR specific flag -- RESET mutex of reg 1C.
    return(0);    // Return success.
}

int stmMdio407_107Cmd_read(int argc, char *argv[]){
	//unsigned int phyaddr;
	unsigned int reg;
	unsigned int val;
	int strlen;
	char str[40];
	//read <reg addr>
	if(argc==2){
		reg	=	(u32)strtol(argv[1], NULL, 16);
		val	=	stmMdio407Read(stmMdioPhy.phyaddr, reg);//ReadReg(reg1);
		if(val == 0xFFFFFFFF){
			return val;
		}
		strlen = sprintf(str,"R(PHYADDR=%02u)>Reg0x%02x=0x%04x", stmMdioPhy.phyaddr, reg, val);
		printf("%s\r\n",str);
		//OzOLED_printString(str, 0, 7, 15);//strlen);
		//g_br_1c = 0;//broadR specific flag -- RESET mutex of reg 1C.
		return 0;
	}else{
		printf("Err\r\n");//printErr();
		return -1;
	}
}
void stmMdio407Show_All_BasicRegisters(){
	u8 i;
	u16 page_extended;
	u32 retval;
	char str[16];
	//struct GenealPhyREG *pBasicReg;

	//printf("==%s==\r\n",GeneralPhyBoard.phyname);
	//Basic Registers
	//pBasicReg = GeneralPhyBoard.GeneralPhyRegs.pBasicReg;
	for(i=0;i<32;i++){
		retval = stmMdio407Read(stmMdioPhy.phyaddr, i);
		printf("[Phy %02u] Reg0x%02x = 0x%04x \r\n",stmMdioPhy.phyaddr, i, retval & 0xffff);
		delayms(400);
	}
}
int stmMdio407_107Cmd_readAll(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	int strlen;
	char str[20];

	//ra
	if(argc==1){
		printf("stmMdio407_107Cmd_readAll\r\n");
		stmMdio407Show_All_BasicRegisters();
		//yConfirmLEDBlink();
		return 0;
	}else{
		printf("ReadAll Err\r\n");
		return -1;
	}
}

int stmMdio407_107Cmd_SetPhyAddrAndInitConfig(int argc, char *argv[]){
	u32 reg;
	u32 val1, val2;
	//u16 page;
	//u16 temp16;

	//GeneralPhyBoard.f_preamblesuppressed = 0;
	if(argc==2){
		//g_phyaddr =	(u8)strtol(argv[1], NULL, 16);
		stmMdioPhy.phyaddr_set_done = 1;
		stmMdioPhy.phyaddr 			= (u8)strtol(argv[1], NULL, 16);
		stmMdio407_107Config(stmMdioPhy.phyaddr,NULL);
		//val1 = yGeneralPhy_GetBasicRegister(DEV_PMAPMD,2);
		//val2 = yGeneralPhy_GetBasicRegister(DEV_PMAPMD,3);
		//printf("Set %s Phy Addr(0x%02x)/PHYID1/PHYID2(=0x%04x/%04x)\r\n", GeneralPhyBoard.phyname, GeneralPhyBoard.phyaddr, val1, val2);
		return 0;
	}
	//errors
	printf("Set Err\r\n");
	return -1;

}

int stmMdio407_107Cmd_SetPhyClkOutput(int argc, char *argv[]){
	unsigned char phyclkout;
	GPIO_InitTypeDef GPIO_InitStructure;

	if(argc==2){
		phyclkout =	(u8)strtol(argv[1], NULL, 16);
		if(phyclkout){
			// Configure MCO1 pin(PA8) in alternate function
			MCO1_Config_25MHz();
			printf("Enabled 25MHz Clock Output to PHY\r\n");
		}else{
#if(PROCESSOR == PROCESSOR_STM32F407VGT6)
			// Configure MCO1 pin(PA8) in normal input function
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
#elif(PROCESSOR == PROCESSOR_STM32F107VCT)
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
			  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
#endif
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			printf("Disabled 25MHz Clock Output to PHY\r\n");
		}
		return 0;
	}
	//errors
	printf("Set Err\r\n");
	return -1;

}
/*
int stmMdioBitbang_readSQI(u32 phyaddr, int first){
	u32 Q;
	int strlen;
	char str[20];
	int D;
	static int M;

		//setup reg access for reg 18.
		g_br_1c = 1;//broadR specific flag -- SET mutex of reg 1C.
		//if(first){
			stmMdioWrite(phyaddr, 0x18, 0xf807); //0x18=Shadow Access Register; 1111(yyy=111=ShadowValueReg To Be Read):1000(PacketCountMode(1=RX):0000:0111(ShadowValue=111=MiscControlReg)
			stmMdioWrite(phyaddr, 0x17, 0x0002); //0x17=Expansion Register; 0002 = Expansion Register Selected. (No Document?)
		//}

		Q	=	stmMdioRead(phyaddr, 0x15);  //No Document..
		D  = 10.0*(log10(((double)Q)/32768.0));
		M   =  -20.0 - D;//???
		//M   =  34.0 + D;//???
		printf("SQI Q=0x%04x. D=%d, M=%d\r\n", Q,D, M);
		g_br_1c = 0;//broadR specific flag -- RESET mutex of reg 1C.

		return M;
}
int stmMdioBitbangCmd_readSQI(int argc, char *argv[]){
	u32 phyaddr;
	//rs <phy addr>
	if(argc==2){
		phyaddr	= (u32)strtol(argv[1], NULL, 16);
		stmMdioBitbang_readSQI(phyaddr,1);
		return 0;
	}else{
		printf("Err\r\n");//printErr();
		return -1;
	}
}
*/
//write <phy addr> <reg addr> <value>
int stmMdio407_107Cmd_write(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	if(argc==3){
		//phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[1], NULL, 16);
		val	=	    (u32)strtol(argv[2], NULL, 16);
		printf("Writing on Phy(0x%02x): Reg 0x%02x <-- 0x%04x\r\n",  stmMdioPhy.phyaddr, reg, val);
		//g_br_1c = (reg == 0x1c) ? 1: 0;  //broadR specific flag -- SET mutex of reg 1C. (Cleared by Read or Help)
		stmMdio407Write(stmMdioPhy.phyaddr, reg, val);
		return 0;
	}else{
		printf("Err\r\n");//printErr();
		return -1;
	}
}
#if 0
void stmMdio407_107Cmd_handler()//
{
    	int nStatus;
    	unsigned char showCmdPrompt = 0;
    	int len;

        // Get a line of text from the user.
    	len = UsartXGetStrNonBlocking();// 	len = Usart1GetStrNonBlocking();
		if(len !=-1){
			//printf("str=%s\r\n",g_usart1rxbuf.buf);
			memset(g_cCmdBuf,0x00,CMD_BUF_SIZE);
			memcpy(g_cCmdBuf,g_usartXrxbuf.buf,len);//memcpy(g_cCmdBuf,g_usart1rxbuf.buf,len);
		}
    	else
    		return;


        // Pass the line from the user to the command processor.
        // It will be parsed and valid commands executed.
        nStatus = CmdLineProcess(g_cCmdBuf);

        //
        // Handle the case of bad command.
        //
        if(nStatus == CMDLINE_BAD_CMD)
        {
            printf("Bad command!\r\n");
        }

        //
        // Handle the case of too many arguments.
        //
        else if(nStatus == CMDLINE_TOO_MANY_ARGS)
        {
            printf("Too many arguments for command processor!\r\n");
        }

        //
        // Otherwise the command was executed.  Print the error
        // code if one was returned.
        //
        else if(nStatus != 0)
        {
            //printf("Command returned error code %s\r\n",  StringFromFresult((FRESULT)nStatus));
        }
        // Print a prompt to the console.  Show the CWD.
        //
    	printf("\r\nCLI> ");
}
#endif


void stmMdio407_107Cmd_handler()//
{
    	int nStatus;
    	unsigned char showCmdPrompt = 0;
    	int len=0;
#if 0
        // Get a line of text from the user.
    	if(g_usartXrxbuf.pos >= 1){
    		  if(g_usartXrxbuf.buf[g_usartXrxbuf.pos-1]== 0x0d){//'\r'){// && (g_usartXrxbuf.buf[g_usartXrxbuf.pos-1]== '\n')){
    			  g_usartXrxbuf.buf[g_usartXrxbuf.pos-1]= '\0';//end of string//g_usartXrxbuf.buf[g_usartXrxbuf.pos-2]= '\0';//end of string
    			  len = g_usartXrxbuf.pos;//g_usartXrxbuf.pos - 2;
    			  //printf("lineLen=%d", len);
    			  g_usartXrxbuf.pos = 0;//reset for next.
    			  memset(g_cCmdBuf,0x00,80);//CMD_BUF_SIZE);
    			  memcpy(g_cCmdBuf,g_usartXrxbuf.buf,len);
    		  }
    	}
		if(len == 0){
    		return;
		}
#else
	       // Get a line of text from the user.
	    	len = UsartXGetStrNonBlocking();// 	len = Usart1GetStrNonBlocking();
			if(len !=-1){
				//printf("str=%s\r\n",g_usart1rxbuf.buf);
				memset(g_cCmdBuf,0x00,CMD_BUF_SIZE);
				memcpy(g_cCmdBuf,g_usartXrxbuf.buf,len);//memcpy(g_cCmdBuf,g_usart1rxbuf.buf,len);
			}
	    	else
	    		return;
#endif
        // Pass the line from the user to the command processor.
        // It will be parsed and valid commands executed.
        nStatus = CmdLineProcess(g_cCmdBuf);

        //
        // Handle the case of bad command.
        //
        if(nStatus == CMDLINE_BAD_CMD)
        {
            printf("Bad command!\r\n");
        }

        //
        // Handle the case of too many arguments.
        //
        else if(nStatus == CMDLINE_TOO_MANY_ARGS)
        {
            printf("Too many arguments for command processor!\r\n");
        }

        //
        // Otherwise the command was executed.  Print the error
        // code if one was returned.
        //
        else if(nStatus != 0)
        {
            //printf("Command returned error code %s\r\n",  StringFromFresult((FRESULT)nStatus));
        }
        // Print a prompt to the console.  Show the CWD.
        //
    	printf("\r\nCLI> ");
}


int stmMdio407_107Loop(void)
{
	//u32 	val;
	char	params[PARAM_MAXNUM][PARAM_MAXLEN];
	int		result;
	u8		len;

	printf("\r\n\r\nMDIO with MDIO/MDC Module.\r\n\r\n");

	stmMdio407_107Cmd_help(0,0);
	printf("CMD# ");

	while(1)
		stmMdio407_107Cmd_handler();
}
#endif
