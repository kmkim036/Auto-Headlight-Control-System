
#include <string.h>
#include "yInc.h"
#if(PROCESSOR == STM32F407VGT6)
//-- STM32F407VGT
//MDIO-PA2
//MDC-PC1
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "cmdline.h"
#include <stdint.h>
#include "yInc.h"

#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT6)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#endif

//[REF] Good Reference can be found in AN10859 "LPC1700 Ethernet MII Management (MDIO) via software".
#define	PARAM_MAXNUM	10
#define	PARAM_MAXLEN	20
#define BASE      16.0
#define NULL 0

char	line[CMD_BUF_SIZE];
unsigned char  line_char_cnt	=	0;
volatile unsigned char	inputed	=	0;

int g_br_1c = 0; //broadR specific flag -- mutex of reg 1C.

struct _stmMdioPhy{
	u8 phyaddr_set_done;
	u8 phyaddr;
	u8 f_preamblesuppressed;
};
static struct _stmMdioPhy stmMdioPhy;
int g_mdioSomeDelay = 0;
extern void OzOLED_printString(const char *String, unsigned char X, unsigned char Y, unsigned char numChar);
static void stmOutputBitStreamMdio(unsigned val, unsigned n);
static void stmTurnaround_MDIO(void);
static unsigned int stmInputStreamMDIO(void);
void UARTIntHandler(void);

unsigned int stmMdioRead(unsigned int PhyAddr, unsigned int PhyReg);

//static int parsing_line(s8 line[], s8 param[][PARAM_MAXLEN], unsigned int len);
//static int proc_cmd(s8 param[][PARAM_MAXLEN], int param_num);
//static void print_menu(void);
int stmMdioBitbangCmd_help(int argc, char *argv[]);
int stmMdioBitbangCmd_read(int argc, char *argv[]);
int stmMdioBitbangCmd_readSpan(int argc, char *argv[]);
int stmMdioBitbangCmd_readSQI(int argc, char *argv[]); //BroadR specific for reading SQI
int stmMdioBitbangCmd_write(int argc, char *argv[]);

//*****************************************************************************
extern char g_cCmdBuf[CMD_BUF_SIZE];
extern struct _usartXrxbuf g_usartXrxbuf; //STM32 Specific
// This is the table that holds the command names, implementing functions, and brief description.
extern tCmdLineEntry g_sCmdTable[];

//We have found that the clocking speed is 400Kbps without somelag();
//     |----0---|---1----|-- TA --|
//        +--+     +--+     +--+
// MDC |--+  +--|--+  +--|--+  +--|

// ----+        +--------+INPUT---+
// MDIO+--------+        +-- HI-Z-+

#define MDIO_PIN_IS_OUTPUT_MODE 1
#define MDIO_PIN_IS_INPUT_MODE  0

//================== Processor Specific =============================================
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT6)|| (PROCESSOR == PROCESSOR_GD32F130FX))

unsigned char g_mdio_pinmode = 0;
#if 0
//MDIO-PB4
//MDC-PB3
//=======================================================================
//[VERY IMPORTANT] PB3's default function is JTDO. (and PB4's default function is JNTRST.)
//Thus we shoul remap to GPIO as the follows.
//=======================================================================
void stmConfigGpioBitbangMDIO(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//MDIO-PB4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB4 Remap !< JTAG-DP Disabled and SW-DP Enabled
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //PB4
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;//PB4
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;//GPIO_Mode_Out_PP; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	//MDC-PB3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3 Remap !< JTAG-DP Disabled and SW-DP Enabled
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //PB3
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;//PB3
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);//
	//
	GPIO_SetBits(GPIOB, GPIO_Pin_3); // PB3-MDC-Low

	g_mdio_pinmode = MDIO_PIN_IS_OUTPUT_MODE;

	printf("Init MDIO Done.\r\n");
}
#else
//MDIO-PB15 @ MOSI
//MDC -PB13 @ SCK
#define MDIO_PIN GPIO_Pin_15
#define MDC_PIN  GPIO_Pin_13

void stmConfigGpioBitbangMDIO(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//MDIO-PB4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = MDIO_PIN;//was MOSI
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	//MDC-PB3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = MDC_PIN;//was SCK
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);//
	//
	GPIO_SetBits(GPIOB, MDC_PIN); // PB13-MDC-Low

	g_mdio_pinmode = MDIO_PIN_IS_OUTPUT_MODE;

	printf("Init bitBang MDIO Done.\r\n");
}
#endif

static void stmOutputBitStreamMdio(unsigned value, unsigned n)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	volatile val;
	val = value;

	if(g_mdio_pinmode !=  MDIO_PIN_IS_OUTPUT_MODE)
	{
		//MDIO -- Set Output
		GPIO_InitStruct.GPIO_Pin = MDIO_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStruct);//
		g_mdio_pinmode =  MDIO_PIN_IS_OUTPUT_MODE;
	}

	for(val <<= (32-n); n; val <<= 1, n--)
	{	// Output MDIO first
		if (val & 0x80000000)
			GPIO_SetBits(GPIOB, MDIO_PIN);
		else
			GPIO_ResetBits(GPIOB, MDIO_PIN);
		//after some delay, issue rising MDC pulse
		somedelay(g_mdioSomeDelay);
		GPIO_SetBits(GPIOB, MDC_PIN);	// Set High MDC -- Peer will latch on this rising edge.
		//after some delay, issue falling MDC pulse
		somedelay(g_mdioSomeDelay);
		GPIO_ResetBits(GPIOB, MDC_PIN);   // Set Low MDC
	}
	somedelay(g_mdioSomeDelay>>4); //at last stay low level.
}

static void stmTurnaround_MDIO(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if(g_mdio_pinmode =  MDIO_PIN_IS_OUTPUT_MODE){
		//MDIO- Set input to Hi-Z
		GPIO_InitStruct.GPIO_Pin = MDIO_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;//GPIO_Mode_IN; 		// we want the pins to be an output
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
		GPIO_Init(GPIOB, &GPIO_InitStruct);//
		somedelay(g_mdioSomeDelay - 20);
		g_mdio_pinmode =  MDIO_PIN_IS_INPUT_MODE;
	}
	// Output clock (MDC signal) : For this clock, the slave will send '0'. We disregard it.
	GPIO_SetBits(GPIOB, MDC_PIN);	// Set High MDC -- Peer will latch on this rising edge.
	somedelay(g_mdioSomeDelay);
	GPIO_ResetBits(GPIOB, MDC_PIN);   // Set Low MDC
	GPIO_ReadInputDataBit(GPIOB, MDIO_PIN); //Dummy.
	somedelay(g_mdioSomeDelay);
}

static unsigned int stmInputStreamMDIO(void)
{
	unsigned int	i;
	volatile val = 0;
	GPIO_InitTypeDef GPIO_InitStruct;

	if(g_mdio_pinmode =  MDIO_PIN_IS_OUTPUT_MODE){
		//MDIO: Set input to Hi-Z

		GPIO_InitStruct.GPIO_Pin = MDIO_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;//GPIO_Mode_IN; 		// we want the pins to be an output
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
		GPIO_Init(GPIOB, &GPIO_InitStruct);//
		g_mdio_pinmode =  MDIO_PIN_IS_INPUT_MODE;
	}
	for (i = 0; i < 16; i++)
	{
		val	<<=	1;
		// First, Output clock (MDC signal)
		GPIO_SetBits(GPIOB, MDC_PIN);	// Set High MDC -- Peer will latch on this rising edge.
		somedelay(g_mdioSomeDelay);
		GPIO_ResetBits(GPIOB, MDC_PIN);   // Set Low MDC
		//Get Data
		if(GPIO_ReadInputDataBit(GPIOB, MDIO_PIN))
			val	|=	1;
		else
			val	|=	0;
		somedelay(g_mdioSomeDelay);
	}
	//At the last, the mdio pin is set input mode.
	return	val;
}

unsigned int stmMdioRead(unsigned int PhyAddr, unsigned int PhyReg)
{
	static unsigned int val;
	GPIO_InitTypeDef GPIO_InitStruct;
	if(g_mdio_pinmode ==  MDIO_PIN_IS_INPUT_MODE){
		 // Set MDIO as output
		 //MDIO-- Set Output
		 GPIO_InitStruct.GPIO_Pin = MDIO_PIN;
		 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD; 		// we want the pins to be an output
		 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
		 GPIO_Init(GPIOB, &GPIO_InitStruct);//
		 GPIO_SetBits(GPIOB, MDIO_PIN); // PB4 Set high
		 g_mdio_pinmode =  MDIO_PIN_IS_OUTPUT_MODE;
	}

	// 32 consecutive ones on MDIO to establish sync
	stmOutputBitStreamMdio(0xffffffff, 32); //if preamble suppression is enabled, we should skip this.

	// start cod (01), read command (10)
	stmOutputBitStreamMdio(0x06, 4);

	// write PHY address
	stmOutputBitStreamMdio(PhyAddr, 5);

	// write the PHY register to read
	stmOutputBitStreamMdio(PhyReg, 5);

	// turnaround MDIO is tristated
	stmTurnaround_MDIO();

	// read the data value
	val	=	stmInputStreamMDIO();

	// turnaround MDIO makes MDIO pin as tristated. But in this phase, it will issue a needless clock pulse.
	//stmTurnaround_MDIO(); //

	return val;
}

void stmMdioWrite(unsigned int PhyAddr, unsigned int PhyReg, unsigned int val)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(g_mdio_pinmode !=  MDIO_PIN_IS_OUTPUT_MODE){
		// Set MDIO and MDC pin as output
		//MDIO-- Set Output
		GPIO_InitStruct.GPIO_Pin = MDIO_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
		GPIO_Init(GPIOB, &GPIO_InitStruct);//
		g_mdio_pinmode =  MDIO_PIN_IS_OUTPUT_MODE;
	}
	// 32 consecutive ones on MDIO to establish sync
	stmOutputBitStreamMdio(0xffffffff, 32);

	// start code (01), write command (01)
	stmOutputBitStreamMdio(0x05, 4);

	// write PHY address
	stmOutputBitStreamMdio(PhyAddr, 5);

	// write the PHY register to write
	stmOutputBitStreamMdio(PhyReg, 5);

	// turnaround MDIO (1,0)
	stmOutputBitStreamMdio(0x02, 2);

	// write the data value
	stmOutputBitStreamMdio(val, 16);

	// turnaround MDIO makes MDIO pin as tristated.
	//But in this phase, it will issue a needless clock pulse.
	//stmTurnaround_MDIO();

}
//Check for PB3/PB4
void stmMdioBitbangPortCheck(void){
	while(1){
		GPIO_SetBits(GPIOB, MDIO_PIN);
		GPIO_SetBits(GPIOB, MDC_PIN); // MDC-Low
		delayms(100);
		GPIO_ResetBits(GPIOB, MDIO_PIN);
		GPIO_ResetBits(GPIOB, MDC_PIN); //MDC-Low
		delayms(100);
	}
}

#else
//-- STM32F407VGT
//MDIO-PA2
//MDC-PC1
void stmConfigGpioBitbangMDIO(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//MDIO-PA2
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //PA2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;//PA2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);//
	//MDC-PC1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //PC1
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;//PC1
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);//
	//
	GPIO_SetBits(GPIOC, GPIO_Pin_1); // PC1-MDC-Low

	printf("Init MDIO Done.\r\n");

}

static void stmOutputBitStreamMdio(unsigned val, unsigned n)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//MDIO-PA2 -- Set Output
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //PA2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;//PA2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	for(val <<= (32-n); n; val <<= 1, n--)
	{	// Output MDIO first
		if (val & 0x80000000)
			GPIO_SetBits(GPIOA, GPIO_Pin_2); // PA2 Set high
		else
			GPIO_ResetBits(GPIOA, GPIO_Pin_2); // PA2 Set high		// Set low
		//Then issue MDC pulse
		//somelag(1);
		GPIO_SetBits(GPIOC, GPIO_Pin_1);	// Set High MDC -- Peer will latch on this rising edge.
		//somelag(1);
		GPIO_ResetBits(GPIOC, GPIO_Pin_1);   // Set Low MDC
	}
}

static void stmTurnaround_MDIO(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	//MDIO-PA2 - // Set input to Hi-Z
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //PA2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;//PA2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	// Output clock (MDC signal) : In this clock, the slave will send '0'. We disregard it.
	GPIO_SetBits(GPIOC, GPIO_Pin_1);	// Set High MDC -- Peer will latch on this rising edge.
	//somelag(1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_1);   // Set Low MDC
	//somelag(1);
}

static unsigned int stmInputStreamMDIO(void)
{
	unsigned int	i;
	volatile val = 0;
	GPIO_InitTypeDef GPIO_InitStruct;

	//MDIO-PA2 - // Set input to Hi-Z
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //PA2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;//PA2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	for (i = 0; i < 16; i++)
	{
		val	<<=	1;
		// First, Output clock (MDC signal)
		GPIO_SetBits(GPIOC, GPIO_Pin_1);	// Set High MDC -- Peer will latch on this rising edge.
		//somelag(1);
		GPIO_ResetBits(GPIOC, GPIO_Pin_1);   // Set Low MDC
		//somelag(1);

		//Get Data
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2))
			val	|=	1;
		//somelag(1); //Need?
	}
	return	val;
}

unsigned int stmMdioRead(unsigned int PhyAddr, unsigned int PhyReg)
{
	static unsigned int val;
	GPIO_InitTypeDef GPIO_InitStruct;
	// Set MDIO and MDC pin as output
	//MDIO-PA2 -- Set Output
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //PA2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;//PA2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	//MDC-PC1-- Set Output
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //PC1
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;//PC1
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

	// 32 consecutive ones on MDIO to establish sync
	stmOutputBitStreamMdio(0xffffffff, 32);

	// start cod (01), read command (10)
	stmOutputBitStreamMdio(0x06, 4);

	// write PHY address
	stmOutputBitStreamMdio(PhyAddr, 5);

	// write the PHY register to write
	stmOutputBitStreamMdio(PhyReg, 5);

	// turnaround MDIO is tristated
	stmTurnaround_MDIO();

	// read the data value
	val	=	stmInputStreamMDIO();

	// turnaround MDIO makes MDIO pin as tristated. But in this phase, it will issue a needless clock pulse.
	stmTurnaround_MDIO(); //

	return val;
}

void stmMdioWrite(unsigned int PhyAddr, unsigned int PhyReg, unsigned int val)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	// Set MDIO and MDC pin as output
	//MDIO-PA2 -- Set Output
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //PA2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;//PA2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	//MDC-PC1-- Set Output
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //PC1
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;//PC1
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

	// 32 consecutive ones on MDIO to establish sync
	stmOutputBitStreamMdio(0xffffffff, 32);

	// start cod (01), write command (01)
	stmOutputBitStreamMdio(0x05, 4);

	// write PHY address
	stmOutputBitStreamMdio(PhyAddr, 5);

	// write the PHY register to write

	stmOutputBitStreamMdio(PhyReg, 5);

	// turnaround MDIO (1,0)
	stmOutputBitStreamMdio(0x02, 2);

	// write the data value
	stmOutputBitStreamMdio(val, 16);

	// turnaround MDIO makes MDIO pin as tristated. But in this phase, it will issue a needless clock pulse.
	stmTurnaround_MDIO();

}


#endif //PROCESSOR

//============== Commons =====================================
int stmMdioBitbangCmd_read(int argc, char *argv[]){
	unsigned int reg;
	unsigned int val;
	int strlen;
	char str[40];
	//read <reg addr>
	if(argc==2){
		reg	=	(u32)strtol(argv[1], NULL, 16);
		val	=	stmMdioRead(stmMdioPhy.phyaddr, reg);//ReadReg(reg1);
		//printf("0x%04x\r\n", val);
		strlen = sprintf(str,"R>[PHYADDR:0x%02x] Reg=0x%02x -> Val=0x%04x", stmMdioPhy.phyaddr, reg, val);
		printf("%s\r\n",str);
		//OzOLED_printString(str, 0, 7, 15);//strlen);
		//g_br_1c = 0;//broadR specific flag -- RESET mutex of reg 1C.
		return 0;
	}else{
		printf("Err\r\n");//printErr();
		return -1;
	}
}

void stmMdioBitbangShow_All_BasicRegisters(){
	u8 i;
	u16 page_extended;
	u32 retval;
	char str[40];
	//struct GenealPhyREG *pBasicReg;

	//printf("==%s==\r\n",GeneralPhyBoard.phyname);
	//Basic Registers
	//pBasicReg = GeneralPhyBoard.GeneralPhyRegs.pBasicReg;
	for(i=0;i<32;i++){
		retval = stmMdioRead(stmMdioPhy.phyaddr, i);
		printf("[Phy %02u] %02u(REG) = 0x%04x \r\n",stmMdioPhy.phyaddr, i, retval & 0xffff);
		delayms(400);
	}
}

int stmMdioBitbangCmd_readAll(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	int strlen;
	char str[40];

	//ra
	if(argc==1){
		stmMdioBitbangShow_All_BasicRegisters();
		//yConfirmLEDBlink();
		return 0;
	}else{
		//printErr();
		return -1;
	}
}
int stmMdioBitbangCmd_help(int argc, char *argv[])
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

void stmMdioBitbangCmd_handler()//
{
    	int nStatus;
    	unsigned char showCmdPrompt = 0;
    	int len;

        // Get a line of text from the user.
    	len = UsartXGetStrNonBlocking();
		if(len !=-1){
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

int stmMdioBitbangCmd_SetPhyAddrAndInitConfig(int argc, char *argv[]){
	u32 reg;
	u32 val1, val2;
	//u16 page;
	//u16 temp16;

	//GeneralPhyBoard.f_preamblesuppressed = 0;
	if(argc==2){
		//g_phyaddr =	(u8)strtol(argv[1], NULL, 16);
		stmMdioPhy.phyaddr_set_done = 1;
		stmMdioPhy.phyaddr =	(u8)strtol(argv[1], NULL, 16);
		//val1 = yGeneralPhy_GetBasicRegister(DEV_PMAPMD,2);
		//val2 = yGeneralPhy_GetBasicRegister(DEV_PMAPMD,3);
		//printf("Set %s Phy Addr(0x%02x)/PHYID1/PHYID2(=0x%04x/%04x)\r\n", GeneralPhyBoard.phyname, GeneralPhyBoard.phyaddr, val1, val2);
		return 0;
	}
	return -1;

}
//write <phy addr> <reg addr> <value>
int stmMdioBitbangCmd_write(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	if(argc==3){
		//phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[1], NULL, 16);
		val	=	    (u32)strtol(argv[2], NULL, 16);
		printf("Writing on Phy(0x%02x) Reg 0x%02x <-- 0x%04x\r\n",  stmMdioPhy.phyaddr, reg, val);
		//g_br_1c = (reg == 0x1c) ? 1: 0;  //broadR specific flag -- SET mutex of reg 1C. (Cleared by Read or Help)
		stmMdioWrite(stmMdioPhy.phyaddr, reg, val);
		return 0;
	}else{
		printf("Err\r\n");//printErr();
		return -1;
	}
}
int stmMdioBitbangLoop(void)
{
	//u32 	val;
	char	params[PARAM_MAXNUM][PARAM_MAXLEN];
	int		result;
	u8		len;

	printf("\r\n\r\nmyMDIO with Bitbang Style.\r\n\r\n");
	g_mdioSomeDelay = 500;

	stmConfigGpioBitbangMDIO();
	//stmMdioBitbangPortCheck();
	stmMdioBitbangCmd_help(0,0);
	stmMdioPhy.phyaddr =	0;//(u8)strtol(argv[1], NULL, 16);
	printf("CMD# ");

	delayms(200);
	stmOutputBitStreamMdio(0xffffffff, 32);

	while(1){
		stmMdioBitbangCmd_handler();
		//stmMdioBitbangShow_All_BasicRegisters(); delayms(2000);
	}
}
