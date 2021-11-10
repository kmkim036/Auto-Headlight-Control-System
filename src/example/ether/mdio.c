#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include <string.h>
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"
#include "utils/cmdline.h"
#include "yLib/yInc.h"
#include "yLib/yPortMap.h"
//[REF] Good Reference can be found in AN10859 "LPC1700 Ethernet MII Management (MDIO) via software".

//-- LM3S811
//MDIO-PC6
//MDC-PD1

//-- LM3S8962 V6
//  MDIO = PG0
// MDC  = PG1
//#define	MDIO_PIN	GPIO_PIN_0  //PG0
//#define MDC_PIN		GPIO_PIN_1  //PG1

//We use 3 LEDs
#define USE_THREE_LED 1
//SMI = PD6
//I2C = PD5
//SPI = PD4

//In addition, PC5 for ULED0 on Target IP101G Board(PIN22).

//We SHOULD define PHY_ACCESS_METHOD as SMIIF (yInc.h)

#define	PARAM_MAXNUM	10
#define	PARAM_MAXLEN	20
#define   BASE      16.0
#define NULL 0

s8	line[MAXLINE];
u8  line_char_cnt	=	0;
volatile u8	inputed	=	0;

int g_br_1c = 0; //broadR specific flag -- mutex of reg 1C.

extern void OzOLED_printString(const char *String, u8 X, u8 Y, u8 numChar);
static void output_MDIO(u32 val, u32 n);
unsigned char turnaround_MDIO(void);
static unsigned short input_MDIO(void);
void UARTIntHandler(void);
//static int parsing_line(s8 line[], s8 param[][PARAM_MAXLEN], unsigned int len);
//static int proc_cmd(s8 param[][PARAM_MAXLEN], int param_num);
//static void print_menu(void);
int dtMdioCmd_help(int argc, char *argv[]);
int dtMdioCmd_read(int argc, char *argv[]);
int dtMdioCmd_readSpan(int argc, char *argv[]);
int dtMdioCmd_readSQI(int argc, char *argv[]); //BroadR specific for reading SQI
int dtMdioCmd_write(int argc, char *argv[]);
//*****************************************************************************
extern char g_cCmdBuf[CMD_BUF_SIZE];
//*****************************************************************************
//
// This is the table that holds the command names, implementing functions,
// and brief description.
//
//*****************************************************************************
extern tCmdLineEntry g_sCmdTable[];

//-- LM3S8962 V6
//  MDIO = PG0
// MDC  = PG1
//#define	MDIO_PIN	GPIO_PIN_0  //PG0
//#define MDC_PIN		GPIO_PIN_1  //PG1
//-- LM3S811
//MDIO-PC6
//MDC-PD1
//#define dtMdio_SYSCTL_PERIPH_GPIOGROUP SYSCTL_PERIPH_GPIOC
//#define	dtMdio_GPIO_PORTGROUP_BASE GPIO_PORTC_BASE
//#define dtMdio_GPIO_PIN GPIO_PIN_6
//MDC-PD1
//#define dtMdc_SYSCTL_PERIPH_GPIOGROUP SYSCTL_PERIPH_GPIOD
//#define	dtMdc_GPIO_PORTGROUP_BASE GPIO_PORTD_BASE
//#define dtMdc_GPIO_PIN GPIO_PIN_1

void yMdioConfirmLEDBlink(void){
#if (PHY_ACCESS_METHOD == SMIIF)
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, ~GPIO_PIN_6);//OFF
	delayms(80);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);//ON
#elif (PHY_ACCESS_METHOD == I2CIF)
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5);//OFF
	delayms(80);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_PIN_5);//ON
#else //(PHY_ACCESS_METHOD == SPIIF)
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, ~GPIO_PIN_4);//OFF
	delayms(80);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4);//ON
#endif

	//Blink LED on Targe PHY Board.
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);//OFF
	delayms(80);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);//ON
}
//PC5(pin 9(SPI_CS2) of USB dongle which will ties with th pin22 on target PHY board.
void dtMdioEnULEDonPHY(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
}

void init_MDIO_LEDs(void){

	//ETC LED CONFIG : USB MODULE FOR MDIO/MDC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, ~GPIO_PIN_6);//OFF
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5);//OFF
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, ~GPIO_PIN_4);//OFF
#if (PHY_ACCESS_METHOD == SMIIF)
		//Config GPIO PD6 (LM3S811)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);//ON
#elif (PHY_ACCESS_METHOD == I2CIF)//I2C
		//Config GPIO PD5 (LM3S811)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_PIN_5);//ON
#else//SPI
		//Config GPIO PD4 (LM3S811)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4);//ON
#endif

		dtMdioEnULEDonPHY();
}

//MDIO-PC6
//MDC-PD1
void init_MDIO(void)
{
	SysCtlPeripheralEnable(dtMdio_SYSCTL_PERIPH_GPIOGROUP); //MDIO
	SysCtlPeripheralEnable(dtMdc_SYSCTL_PERIPH_GPIOGROUP); //MDC

	/* Set MDIO and MDC pin as output */
	GPIOPinTypeGPIOOutputOD(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN);//GPIOPinTypeGPIOOutput(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN);
	GPIOPinTypeGPIOOutput(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN);

	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, ~dtMdc_GPIO_PIN);		// Set MDC Low
	UARTprintf("Init MDIO Done.\r\n");
}

void somelag(int x){
	//int y=x;
	//y++;
}
//     |----0---|---1----|-- TA --|
//        +--+     +--+     +--+
// MDC |--+  +--|--+  +--|--+  +--|

// ----+        +--------+INPUT---+
// MDIO+--------+        +-- HI-Z-+

//We have found that the clocking speed is 400Kbps without somelag();
static void output_MDIO(u32 val, u32 n)
{
	for(val <<= (32-n); n; val <<= 1, n--)
	{
		// Output MDIO first
		if (val & 0x80000000)
			GPIOPinWrite(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN, dtMdio_GPIO_PIN);		// Set high
		else
			GPIOPinWrite(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN, ~dtMdio_GPIO_PIN);		// Set low
		//Then issue MDC pulse
		somedelay(4);
		GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, dtMdc_GPIO_PIN);			// Set High MDC -- Peer will latch on this rising edge.
		GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, ~dtMdc_GPIO_PIN);			// Set Low MDC
		somedelay(1);
	}
}

void preamble_MDIO(u8 f_preamblesuppressed){

	GPIOPinTypeGPIOOutput(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN);
	GPIOPinTypeGPIOOutputOD(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN);
	GPIOPinWrite(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN, dtMdio_GPIO_PIN);

	if(f_preamblesuppressed){
		output_MDIO(0xffffffff, 8);//turnaround_MDIO(); //issue an idle period with a clock pulse.
	}else{
		output_MDIO(0xffffffff, 32);//32 consecutive ones on MDIO to establish sync
	}
}

//issue the idle bit.
unsigned char turnaround_MDIO(void)
{
	unsigned short val = 0;

	GPIOPinTypeGPIOInput(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN);				// Set input to Hi-Z
	somedelay(20);
	// Output clock (MDC signal) : In this clock, the slave will send '0'. We disregard it.
	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, dtMdc_GPIO_PIN);			// Set high MDC
	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, ~dtMdc_GPIO_PIN);			// Set low MDC
	somedelay(20);
	val = GPIOPinRead(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN) & dtMdio_GPIO_PIN;
	if(val !=0){
		//UARTprintf("ERR: TA\r\n");
		return 0;
	}else
		return 1;
	//this H-L transition takes 750nsec.
}
static void idle_MDIO(void)
{
	GPIOPinTypeGPIOOutputOD(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN);				// Set input to Hi-Z
	GPIOPinWrite(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN, dtMdio_GPIO_PIN);
	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, dtMdc_GPIO_PIN);			// Set High MDC -- We add it.
	somedelay(2);
	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, ~dtMdc_GPIO_PIN);			// Set low MDC
	//GPIOPinWrite(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN, dtMdio_GPIO_PIN);
	//MDIO=HI-Z
	//GPIOPinTypeGPIOInput(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN);				// Set input to Hi-Z
	//Output MDC=0
/*	somedelay(1);
	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, dtMdc_GPIO_PIN);			// Set High MDC
	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, ~dtMdc_GPIO_PIN);			// Set low MDC
	somedelay(1);
	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, dtMdc_GPIO_PIN);			// Set High MDC
	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, ~dtMdc_GPIO_PIN);			// Set low MDC
	somedelay(1);
	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, dtMdc_GPIO_PIN);			// Set High MDC
	GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, ~dtMdc_GPIO_PIN);			// Set low MDC
*/
	//this H-L transition takes 750nsec.
}

//Must set input ahead.
static u16 input_MDIO(void)
{
	u32	i;
	unsigned short val = 0;

	//GPIOPinTypeGPIOInput(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN);

	for (i = 0; i < 16; i++)
	{
		val	<<=	1;
		// First, Output clock (MDC signal)
		GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, dtMdc_GPIO_PIN);			// Set high MDC -- The peer will issue its data on the MDIO pin.
		somedelay(1); //Valid from MDC Rising Edge= 300nsec.
		GPIOPinWrite(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN, ~dtMdc_GPIO_PIN);			// Set low MDC
		//Get Data
		if (GPIOPinRead(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN) & dtMdio_GPIO_PIN)
			val	|=	1;
		else
			val |= 0;

	}
	return	val;
}
void mdio_write(u32 PhyAddr, u32 PhyReg, u32 val)
{
	/* 32 consecutive ones on MDIO to establish sync */
	preamble_MDIO(0); //output_MDIO(0xffffffff, 32);

	/* start cod (01), write command (01) */
	output_MDIO(0x05, 4);

	/* write PHY address */
	output_MDIO(PhyAddr, 5);

	/* write the PHY register to write */
	output_MDIO(PhyReg, 5);

	/* turnaround MDIO (1,0) */
	output_MDIO(0x02, 2);

	/* write the data value */
	output_MDIO(val, 16);

	/* turnaround MDIO makes MDIO pin as tristated. But in this phase, it will issue a needless clock pulse. */
	//GPIOPinTypeGPIOOutputOD(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN);
	//GPIOPinTypeGPIOInput(dtMdio_GPIO_PORTGROUP_BASE, dtMdio_GPIO_PIN);				// Set input to Hi-Z
	//GPIOPinTypeGPIOOutput(dtMdc_GPIO_PORTGROUP_BASE, dtMdc_GPIO_PIN);
	idle_MDIO(); //	turnaround_MDIO();
}
u16 mdio_read(u32 PhyAddr, u32 PhyReg)
{
	static u16 val;

	preamble_MDIO(0);//output_MDIO(0xffffffff, 32);//Preamble. 32 consecutive ones on MDIO to establish sync
	output_MDIO(0x06, 4);// start code (01), read command (10)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(PhyReg, 5);// write the PHY register to write
	if(!turnaround_MDIO()){
		input_MDIO();	// turnaround to make MDIO tristate.
		UARTprintf("Broken TA\r\n");
		return 0;
	}else{
		val	=	input_MDIO();// read the data value
		idle_MDIO();
		return val;
	}
}

//Following IEEE802.3 Annex22D
// the Reg13(MMD Access Control Register) =
//    FuncField[15:14(00=Addr,01=Data(NoPostInc),10=DataR/W(postInc),11=DataWrite(postInc)], Rsvd[13:5]; DevAD[4:0])
// Reg14(MMD Access Address Data Register)
//    FOr Func=00, MMD DEVAD's Address Register
//        Others,  MMD DEVAD's Data Register.
u16 mdio_read_clause45overClause22(u32 PhyAddr, u32 mmdeviceAddr, u32 PhyReg, u8 f_preamblesuppressed)
{
	static u16 val;

	//(1) Set Address. For Reg13, write Func and Address to specify the MMD and Register with the Clause22 procedure on Reg 13.
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x05, 4);// start code (01), Opcode=Write (10)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(13, 5); //Reg 13
	output_MDIO(0x02, 2); //TA on Write Operation
	output_MDIO(0x0000 | mmdeviceAddr,16); // write FN code(00) with the MMDID.
	idle_MDIO();

	//(2) 2nd, Write PhyReg on Reg 14.
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x05, 4);// start code (01), Opcode=Write (10)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(14, 5); //Reg 14
	output_MDIO(0x02, 2); //TA on Write Operation
	output_MDIO(PhyReg,16); // write the Address of desired Reg. to the MMD's addr register.
	idle_MDIO();

	//(3) 3rd, Write MMD on Reg 13 for reading
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x05, 4);// start code (01), Opcode=Write (01)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(13, 5); //Reg 13
	output_MDIO(0x02, 2); //TA on Write Operation
	output_MDIO(0x4000 | mmdeviceAddr,16); // opcode=Read_NoPostInc(01)...
	idle_MDIO();

	//(4) 4th. Read Reg 14.
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x06, 4);// start code (01), read command (10)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(14, 5); //Reg 14
	if(!turnaround_MDIO()){
		input_MDIO();	// turnaround to make MDIO tristate.
		UARTprintf("Broken TA\r\n");
		return 0;
	}else{
		val	= input_MDIO();// read the data value
		idle_MDIO();
		return val;
	}
}


u16 mdio_read_clause45(u32 PhyAddr, u32 mmdeviceID, u32 PhyReg, u8 f_preamblesuppressed)
{
	static u16 val;

	//UARTprintf("%d:%d:%d\r\n", PhyAddr, mmdeviceID, PhyReg);
	//[1] Set Address
	//(1) First write Address to specify the MMD and Register
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x00, 4);// start code (00), Opcode=Address (00)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(mmdeviceID, 5);
	output_MDIO(0x2, 2); //turnaround on Write Operation
	output_MDIO(PhyReg,16); // write the PhyReg.
	idle_MDIO();

	//[2]Read
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x03, 4);// start code (00), read command (11)/read_with_increment command (10)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(mmdeviceID, 5);
	//somedelay(10);
	if(!turnaround_MDIO()){
		input_MDIO();	// turnaround to make MDIO tristate.
		idle_MDIO();
		UARTprintf("Broken TA\r\n");
		return 0;
	}else{
		val	=	input_MDIO();// read the data value
		idle_MDIO();
		return val;
	}
}


u16 mdio_write_clause45(u32 PhyAddr, u32 mmdeviceID, u32 PhyReg, u8 f_preamblesuppressed, u32 val)
{
	//[1] Set Address
	//(1) First Write Address to specify the MMD and Register
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x00, 4);// start code (00), Opcode=Address (00)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(mmdeviceID, 5);
	output_MDIO(0x02, 2); //turnaround on Write Operation
	output_MDIO(PhyReg,16); // write the PhyReg.
	idle_MDIO();

	//[2]Write
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x01, 4);// start code (00), write command (01)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(mmdeviceID, 5);
	output_MDIO(0x02, 2); //turnarround for write
	output_MDIO(val, 16);
	idle_MDIO();

	return val;
}

//Following IEEE802.3 Annex22D
// the Reg13(MMD Access Control Register) =
//    FuncField[15:14(00=Addr,01=Data(NoPostInc),10=DataR/W(postInc),11=DataWrite(postInc)], Rsvd[13:5]; DevAD[4:0])
// Reg14(MMD Access Address Data Register)
//    FOr Func=00, MMD DEVAD's Address Register
//        Others,  MMD DEVAD's Data Register.
u16 mdio_write_clause45overClause22(u32 PhyAddr, u32 mmdeviceAddr, u32 PhyReg, u8 f_preamblesuppressed, u32 val)
{
	//(1) For Reg13, write Func and Address to specify the MMD and Register
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x05, 4);// start code (01), Opcode=Write (10)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(13, 5); //Reg 13
	output_MDIO(0x02, 2); //turnaround on Write Operation
	output_MDIO(0x0000 | mmdeviceAddr,16); // write FN code(00) with the MMDID.
	//idle_MDIO();

	//(2) 2nd, Write Register Address to specify the MMD and Register
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x05, 4);// start code (01), Opcode=Write (10)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(14, 5); //Reg 14
	output_MDIO(0x02, 2); //turnaround on Write Operation
	output_MDIO(PhyReg,16); // write the Address of desired Reg. to the MMD's addr register.
	//idle_MDIO();

	//[2]Write
	//(3) 3rd, Write PHY Register Address to specify the MMD and Register
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x05, 4);// start code (01), Opcode=Write (01)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(13, 5); //Reg 13
	output_MDIO(0x02, 2); //turnaround on Write Operation
	output_MDIO(0x8000 | mmdeviceAddr,16); // opcode=Write_NoPostInc(10)...
	//idle_MDIO();
	//(4) 4th. Write
	preamble_MDIO(f_preamblesuppressed);
	output_MDIO(0x05, 4);// start code (01), write command (01)
	output_MDIO(PhyAddr, 5);// write PHY address
	output_MDIO(14, 5); //Reg 14
	output_MDIO(0x02, 2); //turnarround for write
	output_MDIO(val, 16);
	idle_MDIO();

	return 1;
}

void GPIOPortB_IRQHandler(void)
{

}
/*
int mdio_probe45(u32 PhyAddr, u32 PhyReg, u8 f_preamblesuppressed){
	u32 mmdeviceAddr;
	//PHY must have a least one of PMA/PMD, WIS, PCS, PHY XS, DTE XS. Give up if none is present.
	for(mmd=1; mmd<5; mmd++){
		//Is this mmd present?
		stat2 = mdio_read_clause45(PhyAddr,mmd, PhyReg, f_preamblesuppressed);
	}
}
*/
int dtMdioCmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    UARTprintf("\r\nEtherPhyManager by EtherCrafts(r)\r\n");
    UARTprintf("\r\nAvailable commands\r\n");
    UARTprintf("------------------\r\n");

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
        UARTprintf("%s%s\r\n", pEntry->pcCmd, pEntry->pcHelp);
        // Advance to the next entry in the table.
        pEntry++;
    }

    g_br_1c = 0; //broadR specific flag -- RESET mutex of reg 1C.
    yMdioConfirmLEDBlink();
    return(0);    // Return success.
}

int dtMdioCmd_read(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	int strlen;
	char str[20];
	//read <phy addr> <reg addr>
	if(argc==3){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[2], NULL, 16);
		val	=	mdio_read(phyaddr, reg);//ReadReg(reg1);
		UARTprintf("R>[Reg%02x:Page%04x]=>0x%04x", phyaddr, reg, val);
		strlen = sprintf(str,"R>%02x(%02x)=0x%04x", phyaddr, reg, val);
		//OzOLED_printString(str, 0, 7, 15);//strlen);

		g_br_1c = 0;//broadR specific flag -- RESET mutex of reg 1C.
		yMdioConfirmLEDBlink();
		return 0;
	}else{
		printErr();
		return -1;
	}
}

//write <phy addr> <reg addr> <value>
int dtMdioCmd_write(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	if(argc==4){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		reg	=	    (u32)strtol(argv[2], NULL, 16);
		val	=	    (u32)strtol(argv[3], NULL, 16);
		UARTprintf("Writing on Phy(0x%02x) Reg 0x%02x <-- 0x%04x\r\n",  phyaddr, reg, val);
		g_br_1c = (reg == 0x1c) ? 1: 0;  //broadR specific flag -- SET mutex of reg 1C. (Cleared by Read or Help)
		mdio_write(phyaddr, reg, val);
		yMdioConfirmLEDBlink();
		return 0;
	}else{
		printErr();
		return -1;
	}
}

//softreset <phy addr>
int dtMdioCmd_softreset(int argc, char *argv[]){
	u32 phyaddr;
	u32 reg;
	u32 val;
	if(argc==2){
		phyaddr	=	(u32)strtol(argv[1], NULL, 16);
		mdio_write(phyaddr, 0x00, 0x8000);
		UARTprintf("PHY Soft Resetting..\r\n");
		yMdioConfirmLEDBlink();
		return 0;
	}
	printErr();
	return -1;
}


void dtMdioCmd_handler()//
{
    	int nStatus;
    	unsigned char showCmdPrompt = 0;

        // Get a line of text from the user.
        //

    	if(UARTPeek('\r') != -1){ //BUFFERED MODE ONLY
    		UARTgets(g_cCmdBuf, sizeof(g_cCmdBuf));
     	}
    	else
    		return;

        //
        // Pass the line from the user to the command processor.
        // It will be parsed and valid commands executed.
        //
        nStatus = CmdLineProcess(g_cCmdBuf);

        //
        // Handle the case of bad command.
        //
        if(nStatus == CMDLINE_BAD_CMD)
        {
            UARTprintf("Bad command!\r\n");
        }

        //
        // Handle the case of too many arguments.
        //
        else if(nStatus == CMDLINE_TOO_MANY_ARGS)
        {
            UARTprintf("Too many arguments for command processor!\r\n");
        }

        //
        // Otherwise the command was executed.  Print the error
        // code if one was returned.
        //
        else if(nStatus != 0)
        {
            UARTprintf("Command returned error code %s\r\n",
                        StringFromFresult((FRESULT)nStatus));
        }
        // Print a prompt to the console.  Show the CWD.
        //
    	UARTprintf("\r\nCLI> ");
}

int dtMdioLoop(void)
{
	//u32 	val;
	s8		params[PARAM_MAXNUM][PARAM_MAXLEN];
	s32		result;
	u8		len;

	GuiInit("\r\n\r\nmyMDIO with Bitbang Style.\r\n\r\n");

	init_MDIO_LEDs();
	init_MDIO();
	dtMdioCmd_help(0,0);

	UARTprintf("CMD# "); //UARTSend((u8 *)"CMD# ", 5);

	while(1)
	    dtCmd_handler();

/*	while(1)
    {
		if(inputed > 0)
		{
			len	=	line_char_cnt;
			line_char_cnt	=	0;
			inputed	=	0;

			result	=	parsing_line(line, params, len);
			result	=	MidoProc_cmd(params, result);

			UARTprintf("CMD# "); //		UARTSend((u8 *)"CMD# ", 5);
		}
    }
    */
}

int dtMdioSimpleLoop(u8 phyAddr)
{
	u32 	val;
	s8		params[PARAM_MAXNUM][PARAM_MAXLEN];
	s32		result;
	u8 		reg;
	u8		len;

	GuiInit("\r\n\r\nmyMDIO with Bitbang Style.\r\n\r\n");

	//init_MDIO_LEDs();
	init_MDIO();
	//dtMdioCmd_help(0,0);

	//UARTprintf("CMD# "); //UARTSend((u8 *)"CMD# ", 5);

	while(1){
		UARTprintf("For PHYADDR of %d\r\n", phyAddr);
		for(reg=0;reg<31;reg++){
			val	=	mdio_read(phyAddr, reg);//ReadReg(reg1);
			UARTprintf("R>[Reg%02x]=>0x%04x\r\n", reg, val);
			delayms(500);
		}
		UARTprintf("\r\n", reg, val);
		delayms(2000);
	}
}

/*
 * int dtMdioCmd_readSpan(int argc, char *argv[]){

}
int dtMdio_readSQI(u32 phyaddr, int first){
	u32 Q;
	int strlen;
	char str[20];
	int D;
	static int M;

		//setup reg access for reg 18.
		g_br_1c = 1;//broadR specific flag -- SET mutex of reg 1C.
		//if(first){
			mdio_write(phyaddr, 0x18, 0xf807); //0x18=Shadow Access Register; 1111(yyy=111=ShadowValueReg To Be Read):1000(PacketCountMode(1=RX):0000:0111(ShadowValue=111=MiscControlReg)
			mdio_write(phyaddr, 0x17, 0x0002); //0x17=Expansion Register; 0002 = Expansion Register Selected. (No Document?)
		//}

		Q	=	mdio_read(phyaddr, 0x15);  //No Document..
		D  = 10.0*(log10(((double)Q)/32768.0));
		M   =  -20.0 - D;//???
		//M   =  34.0 + D;//???
		UARTprintf("SQI Q=0x%04x. D=%d, M=%d\r\n", Q,D, M);
		g_br_1c = 0;//broadR specific flag -- RESET mutex of reg 1C.

		return M;
}
int dtMdioCmd_readSQI(int argc, char *argv[]){
	u32 phyaddr;
	//rs <phy addr>
	if(argc==2){
		phyaddr	= (u32)strtol(argv[1], NULL, 16);
		dtMdio_readSQI(phyaddr,1);
		return 0;
	}else{
		printErr();
		return -1;
	}
}
 */
/*
int MidoProc_cmd(s8 param[][PARAM_MAXLEN], int param_num)
{
	s32 result	=	0;
	u32	reg1, val;

	if (strncmp((s8 *)param[0], "menu", 4) == 0)
	{
		MdioPrint_menu();
	}
	else if (strncmp((s8 *)param[0], "?", 1) == 0)
	{
		print_menu();
	}
	else if ((strncmp(param[0], "read", 4) == 0) || (strncmp(param[0], "r", 1) == 0))
	{
		if(param_num==2){
			reg1	=	(u8)strtol(param[1], NULL, 16);
			sprintf(ostr, "Reading from Reg 0x%02x --> ", reg1);
			UARTSend((u8 *)ostr, strlen((s8 *)ostr));

			//val	=	ReadReg(reg1);
			sprintf(ostr, "0x%04x\r\n", val);
			UARTSend((u8 *)ostr, strlen((s8 *)ostr));
		}else
			printErr();
	}
	else if ((strncmp((s8 *)param[0], "write", 5) == 0) || (strncmp((s8 *)param[0], "w", 1) == 0))
	{
		if(param_num==3){
			reg1 =	(u8)strtol(param[1], NULL, 16);
			val	=	(u16)strtol(param[2], NULL, 16);
			sprintf(ostr, "Writing on Reg 0x%02x <-- 0x%04x\r\n", reg1, val);
			UARTSend((u8 *)ostr, strlen((s8 *)ostr));
			//WriteReg(reg1, val);
		}else
			printErr();
	}else if(param_num==1){

	}else{
		printErr();
	}
	return	result;
}
*/
