#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
//#include "driverlib/qei.h"
#include "utils/uartstdio.h"
#include "ylib/yInc.h"
#include "ylib/yPortMap.h"

extern void dtQEIConfigExp0();
extern unsigned long QEIGet0();
extern unsigned long QEIGet1();
extern void lmMCP23008_Init();
extern unsigned char lmMCP23008_ReadGpio();

void TDA_Init();
void TDA_VolHandler(unsigned char nextb, unsigned char prevb, unsigned short Vol);
void TDA_PwrHandler(void);


#define maxFreq 108000
#define minFreq 87500
unsigned char g_recvBurst[5];
unsigned char g_pwr;
char iastr[32];

unsigned char g_Tea_Write_Data[5];// = {0x29,0xc2,0x20,0x11,0x00};
unsigned char g_Tea_Read_Data[5];
unsigned int g_Tea_Pll,g_Tea_Frequency;

void Tea_GetFreq(void);
void Tea_Setup(void);

/* WRITE
 * D[0] = MUTE-SearchMode-PLL[13]~[8]
 * D[1] = PLL[7]~PLL[0]
 * D[2] = SearchUp/Dn[7]-SearchStopLevel[6:5]-HighLowSideInjection[4](0)-MonoStereo[3](0)-MuteRight[2](0)-MuteLeft[1](0)-SoftwarePort1
 * D[3]= SWP2[7]-STBY[6]-BandLimit[5](0)-XTAL[4]-SoftMute[3]-HighCutControl[2]-StereoNoiseCancel[1]-SearchIndicator[0]
 * D[4] = PLLRef[7] - DeEmphasis[6]- xxxxxx
 */
unsigned char Tea_WriteByteBurst(unsigned char sendBurst[],unsigned char len)
{
	UARTprintf("Tea_WriteByteBurst\r\n");
	I2CSendBurst(TEA_ADDR_W, sendBurst, len); //{START | ADDR | Instruction(0x01) |D1,D2,D3,D4 | STOP}
	somedelay(10);//add some delay..(ºÒÇÊ¿ä??)
	return len;
}
/* D[0] = ReadyFoundFlag[7]-BandLimitFlag[6]-PLL[5:0] after search or preset
 * D[1] = PLL[7:0]
 * D[2] = Stereo[7]-IF[6:0]
 * D[3] = LEV[7:4]-ChipId[3:1]-0
 * D[4] = 0000-0000 : Rsvd
 */
unsigned char Tea_ReadByteBurst(unsigned char recvBurst[],unsigned char len)
{
	I2CRecvBurst(TEA_ADDR_R, recvBurst, len); //{START | ADDR | Instruction(0x01) |D1,D2,D3,D4 | STOP}
	somedelay(10);
	return len;
}

unsigned char Tea_RadioWrite(void)
{
	Tea_WriteByteBurst(g_Tea_Write_Data,5);
	return 1;
}

unsigned char Tea_RadioRead(void)
{
	unsigned char tempH, tempL;
	g_Tea_Pll = 0;
	Tea_ReadByteBurst(g_Tea_Read_Data,5);
	UARTprintf("Tea_RadioRead=%x:%x:%x:%x:%x",g_Tea_Read_Data[0], g_Tea_Read_Data[1],g_Tea_Read_Data[2],g_Tea_Read_Data[3],g_Tea_Read_Data[4]);
	tempL=g_Tea_Read_Data[1];
	tempH=g_Tea_Read_Data[0];
	tempH &=0x3f;
	g_Tea_Pll = tempH*256 + tempL;

	Tea_GetFreq();

	return 1;
}

unsigned int Tea_GetFreq2Pll(unsigned long freq)
{
	unsigned char hlsi;
	unsigned int pll = 0;
	hlsi = 0;//g_Tea_Write_Data[2] & 0x10;

	if (hlsi)
		pll = (unsigned int)((float)(g_Tea_Frequency+225)*4)/(float)32.768; //K
	else
		pll = (unsigned int)((float)(g_Tea_Frequency-225)*4)/(float)32.768;

	return pll;
}
unsigned long Tea_GetPll2Freq(unsigned int pll)
{
	unsigned long freq;
	unsigned char hlsi;
	unsigned int npll = pll;

	hlsi = 0;//g_Tea_Write_Data[2] & 0x10;
	if (hlsi)
		freq = (unsigned long)((float)(npll)*(float)8.192 - 225); //KHz
	else
		freq = (unsigned int)((float)(npll)*(float)8.192 + 225);
	return (freq+50);
}

unsigned char Tea_GetLevel(void)
{
	return (g_Tea_Read_Data[3]>>4);
}
unsigned char Tea_GetStereoMono(void)
{
	return (g_Tea_Read_Data[2]&0x80);
}

void Tea_Setup(void){
	//I2c config
	if(!g_bI2CModuleConfigDone)
		I2CConfigModule();
	//Saa1064 4-LED Driver Init
	//Saa1064Init();
	//QEI encoder -- automatically added by CoIDE

	//dtQEIConfigExp0(); //PD0, PD1 : Tune Up/Down

	//dtQEIConfigExp1();
	somedelay(10000);

	g_Tea_Frequency = 93100;//87500; //93.1MHz KBS Classic/ 87.5MHz of starting freq
	UARTprintf("The default frequency is our famous classic channel of 93.1MHz\r\n");
	Tea_SetFreq(g_Tea_Frequency);

}
void Tea_SearchUp(unsigned long freq) //UP
{
	unsigned int pll = 0;
	pll= Tea_GetFreq2Pll(freq);
	g_Tea_Write_Data[0] = ((pll >> 8)&0x003f) | 0x40;
	g_Tea_Write_Data[1] = pll & 0x00ff;
	g_Tea_Write_Data[2] = 0xc0;//Up
	g_Tea_Write_Data[3] = 0x90;
	g_Tea_Write_Data[4] = 0x00;
	Tea_RadioWrite();

	UARTprintf("Tea_RadioWriteUp=%x:%x:%x:%x:%x\r\n",g_Tea_Write_Data[0], g_Tea_Write_Data[1],g_Tea_Write_Data[2],g_Tea_Write_Data[3],g_Tea_Write_Data[4]);
	somedelay(1000000);
	//Tea_ReadByteBurst(g_Tea_Read_Data,5);
	//somedelay(100000);
	//UARTprintf("Tea_RadioRead=%x:%x:%x:%x:%x\r\n",g_Tea_Read_Data[0], g_Tea_Read_Data[1],g_Tea_Read_Data[2],g_Tea_Read_Data[3],g_Tea_Read_Data[4]);

}

void Tea_SearchDown(unsigned long freq) //UP
{
	unsigned int pll = 0;
	pll= Tea_GetFreq2Pll(freq);
	g_Tea_Write_Data[0] = ((pll >> 8)&0x003f) | 0x40;
	g_Tea_Write_Data[1] = pll & 0x00ff;
	g_Tea_Write_Data[2] = 0x40;//Down
	g_Tea_Write_Data[3] = 0x90;
	g_Tea_Write_Data[4] = 0x00;
	Tea_RadioWrite();

	UARTprintf("Tea_RadioWriteDown=%x:%x:%x:%x:%x\r\n",g_Tea_Write_Data[0], g_Tea_Write_Data[1],g_Tea_Write_Data[2],g_Tea_Write_Data[3],g_Tea_Write_Data[4]);
	somedelay(1000000);

}
void Tea_SetFreq(unsigned long freq) //UP
{
	unsigned int pll = 0;

	pll= Tea_GetFreq2Pll(freq);

	g_Tea_Write_Data[0] = ((pll >> 8)&0x003f);
	g_Tea_Write_Data[1] = pll & 0x00ff;
	g_Tea_Write_Data[2] = 0x40;
	g_Tea_Write_Data[3] = 0x90;
	g_Tea_Write_Data[4] = 0x00;
	Tea_RadioWrite();

	UARTprintf("SetFreq(%u): Tea_RadioWrite=%x:%x:%x:%x:%x\r\n",freq, g_Tea_Write_Data[0], g_Tea_Write_Data[1],g_Tea_Write_Data[2],g_Tea_Write_Data[3],g_Tea_Write_Data[4]);
	somedelay(100000);

}
void Tea_SetMute(unsigned long freq) //UP
{
	unsigned int pll = 0;

	pll= Tea_GetFreq2Pll(freq);

	g_Tea_Write_Data[0] = ((pll >> 8)&0x003f) | 0x80;
	g_Tea_Write_Data[1] = pll & 0x00ff;
	g_Tea_Write_Data[2] = 0x40;
	g_Tea_Write_Data[3] = 0x90;
	g_Tea_Write_Data[4] = 0x00;
	Tea_RadioWrite();

	UARTprintf("Tea_RadioWrite=%x:%x:%x:%x:%x\r\n",g_Tea_Write_Data[0], g_Tea_Write_Data[1],g_Tea_Write_Data[2],g_Tea_Write_Data[3],g_Tea_Write_Data[4]);
	somedelay(100000);
}
//FM Tuner
//
void Tea_Search(unsigned char mode) //UP/Down
{
	//Tea_RadioRead();
rep:	if(mode){
		g_Tea_Frequency += 100;
		if(g_Tea_Frequency > maxFreq)
			g_Tea_Frequency = maxFreq;
		//Tea_SearchUp(g_Tea_Frequency);
	}else{
		g_Tea_Frequency -= 100;
		if(g_Tea_Frequency < minFreq)
			g_Tea_Frequency = maxFreq;
		//Tea_SearchDown(g_Tea_Frequency);
	}

	Tea_SetFreq(g_Tea_Frequency);
	//UARTprintf("Tea_RadioWrite=%x:%x:%x:%x:%x\r\n",g_Tea_Write_Data[0], g_Tea_Write_Data[1],g_Tea_Write_Data[2],g_Tea_Write_Data[3],g_Tea_Write_Data[4]);
	somedelay(100000);
	Tea_ReadByteBurst(g_Tea_Read_Data,5);
	UARTprintf("Tea_RadioRead=%x:%x:%x:%x:%x\r\n",g_Tea_Read_Data[0], g_Tea_Read_Data[1],g_Tea_Read_Data[2],g_Tea_Read_Data[3],g_Tea_Read_Data[4]);

	//if(Tea_GetLevel() < 5)
	//	goto rep;
}

void Tea_AutoSearch(unsigned char mode)
{
	Tea_RadioRead();
	Tea_GetPll();
	if(mode)
		g_Tea_Write_Data[2] = 0xa0;
	else
		g_Tea_Write_Data[2] = 0x20;
	g_Tea_Write_Data[0] = g_Tea_Pll/256;
	g_Tea_Write_Data[1] = g_Tea_Pll % 256;
	g_Tea_Write_Data[3] = 0x11;
	g_Tea_Write_Data[4] = 0x00;
	Tea_RadioWrite();
	Tea_RadioRead();
	while(!(g_Tea_Read_Data[0] & 0x80)){
		Tea_RadioRead();
		//lcd out freq
	}
}

//FM Tuner Handler
//

void Tea_Handler(unsigned char nextb, unsigned char prevb){
	unsigned char level, stereo;
	unsigned int FreqH,FreqL;
	char str[16];

	if(nextb)
		Tea_Search(1);
	else if(prevb)
		Tea_Search(0);
	level = Tea_GetLevel();
	stereo = Tea_GetStereoMono();
	FreqH = g_Tea_Frequency/1000;
	FreqL = g_Tea_Frequency%1000;
	FreqL = FreqL/100;
	if(stereo)
		//sprintf(iastr,"%dMHz:Lv=%d:St",g_Tea_Frequency,level);
		sprintf(iastr,"%d.%dMHz: Level=%d: Stereo",FreqH,FreqL,level);
	else
		sprintf(iastr,"%d.%dMHz: Level=%d: Mono   ",FreqH,FreqL,level);
	UARTprintf("%s\r\n",iastr);

	sprintf(str,"%d.%dMHz", FreqH,FreqL);
	OzOLED_printString(str,4,3,15);

	//    ST7735_Yoon_PrintStr(1,88,iastr);//    ST7735_Yoon_PrintStr(1,88,"IC: ST7735R");
//	sprintf(iastr,"%dMHz:Lv=%d:Mono",g_Tea_Frequency,level);
//	Hd44780WriteString(iastr);

	//Not working???
	//Hd44780GotoXY(14,1);
	//Hd44780WriteByteData(0x00);

}
/* TDA Amp...
void TDA_Init(void)
{
	int i=0;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //PF0 -- UD1/UD2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG); //PG1 -- OE, //PG0 -- BL ON/OFF OUTPUT
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //PC5/7 -- MODE SEL

	//UD, OE
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_0 | GPIO_PIN_1 );
	//GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, ~GPIO_PIN_0); //BL OFF
	GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, ~GPIO_PIN_0); //BL ON
	GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, ~GPIO_PIN_1); //OE disable

	//PWR -- PG0, MODESEL -- PC7/5
	//GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_0);//PG0 -- PWR ON/OFF INPUT
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7 | GPIO_PIN_5 ); //PC7/5 MODE SEL
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7 | GPIO_PIN_5, GPIO_PIN_7 | GPIO_PIN_5); //S1/S0 = 1/1 = Operate

	g_pwr = 0;
}
//TDA Amp Volume Control
//PG1 = Output Enable
//PF0 = Volume Up/Down
void TDA_Vol(unsigned char ud, unsigned short Vol) //UP/Down
{
	if(ud){ //up
		// Output high level
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1); //OE Enable
		somedelay(5000);
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, ~GPIO_PIN_1); //OE disable
	}else{
		// Output Low level
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1); //OE Enable
		somedelay(5000);
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, ~GPIO_PIN_1); //OE disable
	}
	//somedelay(1000);
	sprintf(iastr,"Vol=%02d",Vol);

	//Hd44780WriteString(iastr);
}
//TDA Amp Volume Control
//PG1 = Output Enable
//PF0 = Volume Up/Down
void TDA_VolHandler(unsigned char nextb, unsigned char prevb, unsigned short Vol){

	if(nextb){
		TDA_Vol(1,Vol);
	}else if(prevb)
		TDA_Vol(0,Vol);
}

void TDA_PwrHandler(void) ---- removed
{
	long pwr;
	pwr = GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_0); //PG0

	if(pwr == GPIO_PIN_0){ //ON
		// Output Mode == 1/1 == Active
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
		sprintf(iastr,"On");
		Hd44780WriteString(iastr);
	}else{
		// Output Mode == 1/0 == Standby
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);
		somedelay(5000);
		sprintf(iastr,"Standby");
		Hd44780WriteString(iastr);
	}
}


//Amp Power Mode Setting
// PF1 = IDX1 = PushPowerOnOff Input Pressed
// Mode [PC7(S1):PC5(S0)] = 11= Activate; 00=Standby;
void TDA_PwrHandler(void)
{
	long pwr;
	//pwr = GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_0); //PG0
	pwr = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1); //PF1
	if(pwr == GPIO_PIN_1){ //ON
		if(g_pwr==0){
			// Output Mode == 1/1 == Active
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7 | GPIO_PIN_5, GPIO_PIN_7 | GPIO_PIN_5);
			//GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
			sprintf(iastr,"On");
			//Hd44780WriteString(iastr);
			GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0 ); //BL ON
			g_pwr = 1;
		}else{
			// Output Mode == 0/0 == Standby
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7 , ~(GPIO_PIN_7 | GPIO_PIN_5));
			//GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);
			sprintf(iastr,"Standby");
			//Hd44780WriteString(iastr);
			GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, ~GPIO_PIN_0 );//BL OFF
			g_pwr = 0;

		}
		somedelay(3000000);
	}
}
*/


void tea5767GpioConf(){
	// Configure the GPIO for Tuning : PC4-PHA0, PC6-PHB0 : Tuning
	//dtQEIConfigExp0();
}

unsigned long Tea_FreqTbl[16] = {93100, 93900,93100, 93900,93100, 93900,93100, 93900,93100, 93900,93100, 93900,93100, 93900,93100, 93900};

void tea5767FmLoop(u8 uselineormic, u8 bypassEn){
	unsigned char pos0,pos1;
	unsigned short saved_Chs, Chs;
	unsigned char i2cBuf[10];
	unsigned char mcp23008val;
	unsigned long freqSel, volSel;
	unsigned long savedFreqSel, savedVolSel;//


	freqSel = 0;
	volSel = 8;

	lmMCP23008_Init();

	//long retEepromStatus;
	saved_Chs = 0;
	if(uselineormic == WM8731USELINEINPUT){
		GuiInit("Tea5767 FM Tuner with WM8731 Audio Codec (LINEINPUT/OUTPUT) Test");
	}else{
		GuiInit("Tea5767 Mic with WM8731 Audio Codec (MICINPUT/HPOUTPUT) Test");
	}
	UARTprintf("\r\n The FM tuner output is fed into the LineInput of WM8731 Codec, and\r\n");

	if(bypassEn)
		UARTprintf("the Codec's output will be out of the LineOutput because of Bypass Enabled.\r\n");
	else
		UARTprintf("the Codec's output will be on the I2S output. You need another slave I2C DAC.\r\n");

	UARTprintf("Use QEI for tuning, and Use Buttons for Volume Up/Down.\r\n");

	//codec Init
	Wm8731Init(uselineormic, bypassEn); //WM8731USEMICINPUT
	//Setup FM Tuner.
	Tea_Setup(); //

#if (USE_PROCESSOR == USE_LM3S8962)
#elif (USE_PROCESSOR == USE_LM3S811)
	UARTprintf("MAX98357> PB5/SHUTDOWN DISABLE\r\n");
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5); //UP-- Disable SHUTDOWN
/*
	//I231AP2111
	UARTprintf("IS31AP2111> Set 48KHz, DISABLE MUTE\r\n");
	i2cBuf[0] = 0x01; i2cBuf[1] = 0x04;
	I2CSendBurst(0x30, i2cBuf, 2); //R1 : 0->0x04
	i2cBuf[0] = 0x02;i2cBuf[1] = 0x00; //R2 : 0e -> 0x00 : Clear Mute
	I2CSendBurst(0x30, i2cBuf, 2); //R1 : 0->0x04
	i2cBuf[0] = 0x11;i2cBuf[1] = 0x30; //R0x11
	I2CSendBurst(0x30, i2cBuf, 2); //R0x11 : 0x32->0x30 : No Power Saving
	i2cBuf[0] = 0x0a;i2cBuf[1] = 0x80; //R0x11
	I2CSendBurst(0x30, i2cBuf, 2); //No Noise Gate

	I2CReceiveBurstWithRestartCondition(0x30, 0, i2cBuf, 8);
	UARTprintf("buffer[0..7]=%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\r\n",i2cBuf[0],i2cBuf[1],i2cBuf[2],i2cBuf[3],i2cBuf[4],i2cBuf[5],i2cBuf[6],i2cBuf[7]);
	I2CReceiveBurstWithRestartCondition(0x30, 8, i2cBuf, 8);
	UARTprintf("buffer[8..F]=%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\r\n",i2cBuf[0],i2cBuf[1],i2cBuf[2],i2cBuf[3],i2cBuf[4],i2cBuf[5],i2cBuf[6],i2cBuf[7]);
	I2CReceiveBurstWithRestartCondition(0x30, 0x10, i2cBuf, 8);
	UARTprintf("buffer[0x10..0x17]=%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\r\n",i2cBuf[0],i2cBuf[1],i2cBuf[2],i2cBuf[3],i2cBuf[4],i2cBuf[5],i2cBuf[6],i2cBuf[7]);
*/
#endif

	while(1){

		mcp23008val = lmMCP23008_ReadGpio();
		freqSel = (mcp23008val & 0xf0) >> 4;
		volSel  = (mcp23008val & 0x0f);
/*		if(savedFreqSel != freqSel){
			UARTprintf("frequency is %uKHz\r\n",Tea_FreqTbl[freqSel]);
			Tea_SetFreq(Tea_FreqTbl[freqSel]);
			savedFreqSel = freqSel;
		}
*/
		if(savedFreqSel < freqSel){
			Tea_Handler(1, 0);
			savedFreqSel = freqSel;

		}else if(savedFreqSel > freqSel){
			Tea_Handler(0,1);
			savedFreqSel = freqSel;
		}

		if(savedVolSel < volSel){
			Wm8731SendVolControlSeq(VOLUP,uselineormic);
			savedVolSel = volSel;
			UARTprintf("vol is %u\r\n",volSel);
		}else if(savedVolSel > volSel){
			Wm8731SendVolControlSeq(VOLDN,uselineormic);
			savedVolSel = volSel;
			UARTprintf("vol is %u\r\n",volSel);
		}
		continue;

		//WM8731Handler(uselineormic);

		//pos0 = QEIGet0();

		//pos1 = QEIGet1();
		//pos1 = 1;
		//Saa1064_Write4Digit(pos0,pos1); //Left/Right -- Display the value of Encoder on 4-LEDs
		//Tea_Handler(0, 0);//Tea_Handler(unsigned char nextb, unsigned char prevb)

		Chs = pos0;
		//pos0++;
		//pos1++;
		//Chs = Chs << 8;
		//Chs = Chs | pos1;

		continue;

		if(Chs != saved_Chs){
				UARTprintf("CHs=%x\r\n",Chs);
				//Hd44780WriteString(astr);

				UARTprintf("CHs=%04x ", Chs);
				if(Chs > saved_Chs)
					Tea_Handler(1, 0);//Tea_Handler(unsigned char nextb, unsigned char prevb)
				else
					Tea_Handler(0, 1);//Tea_Handler(unsigned char nextb, unsigned char prevb)
				//Tea_AutoSearch(0);
				UARTprintf("C=%d\r\n",g_Tea_Frequency);//
				//Hd44780WriteString(astr);
				//WriteReg(2, Chs); //Channel Set (pos0=PlayChannel(MSB), pos1=SendChannel(LSB))
		}
		saved_Chs = Chs;
		//yUARTprintf("Loop");
		//ssd_handler();
	}
}
