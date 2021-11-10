#include <string.h>
#include <stdarg.h>
#include "example/yInc.h"
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#else

#endif
#include "misc.h"

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;

struct _tea5767{
	unsigned char pos0,pos1;
	unsigned short saved_Chs, Chs;
	unsigned char i2cBuf[10];
	unsigned char mcp23008val;
	unsigned long freqSel, volSel;
	unsigned long savedFreqSel, savedVolSel;//
	unsigned int tea_Pll,Tea_Frequency;
} tea5767;

#define TEA_ADDR8 			0xc0 //8bitAddress

#define maxFreq 108000
#define minFreq 87500

#define TEA5767_UP (1)
#define TEA5767_DN (-1)

unsigned char g_recvBurst[5];
unsigned char g_pwr;
char iastr[32];

unsigned char g_Tea_Write_Data[5];// = {0x29,0xc2,0x20,0x11,0x00};
unsigned char g_Tea_Read_Data[5];

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
    if(stm_I2C_SendBurst(TEA_ADDR8, sendBurst, len) == 0)
    	return 0; //error
	//{START | ADDR | Instruction(0x01) |D1,D2,D3,D4 | STOP}
	somedelay(10);//add some delay..(ºÒÇÊ¿ä??)
	return len;
}
/* D[0] = ReadyFoundFlag[7]-BandLimitFlag[6]-PLL[5:0] after search or preset
 * D[1] = PLL[7:0]
 * D[2] = Stereo[7]-IF[6:0]
 * D[3] = LEV[7:4]-ChipId[3:1]-0
 * D[4] = 0000-0000 : Rsvd
 */
unsigned char Tea_ReadByteBurst(unsigned char recvBurst[],unsigned char rcvlen)
{
	//Different from the regular I2C transactions.
	//stm_I2C_ReceiveBurstWithRestartCondition(TEA_ADDR8, 1, recvBurst, rcvlen);
	if(!stm_I2C_ReceiveBurst(TEA_ADDR8, recvBurst, rcvlen))
		return 0;
	somedelay(10);
	return rcvlen;
}

unsigned char Tea_RadioWrite(void)
{
	Tea_WriteByteBurst(g_Tea_Write_Data,5);
	return 1;
}

unsigned char Tea_RadioRead(void)
{
	unsigned char tempH, tempL;
	tea5767.tea_Pll = 0;
	Tea_ReadByteBurst(g_Tea_Read_Data,5);
	printf("Tea_RadioRead=%x:%x:%x:%x:%x",g_Tea_Read_Data[0], g_Tea_Read_Data[1],g_Tea_Read_Data[2],g_Tea_Read_Data[3],g_Tea_Read_Data[4]);
	tempL=g_Tea_Read_Data[1];
	tempH=g_Tea_Read_Data[0];
	tempH &=0x3f;
	tea5767.tea_Pll = tempH*256 + tempL;

	//Tea_GetFreq();

	return 1;
}

unsigned int Tea_ConvFreq2Pll(unsigned long freq)
{
	unsigned char hlsi;
	unsigned int pll = 0;
	hlsi = 0;//g_Tea_Write_Data[2] & 0x10;

	if (hlsi)
		pll = (unsigned int)((float)(tea5767.Tea_Frequency+225)*4)/(float)32.768; //K
	else
		pll = (unsigned int)((float)(tea5767.Tea_Frequency-225)*4)/(float)32.768;

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


	tea5767.Tea_Frequency = 93100;//87500; //93.1MHz KBS Classic/ 87.5MHz of starting freq
	printf("The default frequency is our famous classic channel of 93.1MHz\r\n");
	Tea_SetFreq(tea5767.Tea_Frequency);

	tea5767.freqSel = 1;
	tea5767.volSel = 8;
	tea5767.saved_Chs = 1;

}

void Tea_SearchUp(unsigned long freq) //UP
{
	unsigned int pll = 0;

	pll= Tea_ConvFreq2Pll(freq);

	g_Tea_Write_Data[0] = ((pll >> 8)&0x003f) | 0x40;
	g_Tea_Write_Data[1] = pll & 0x00ff;
	g_Tea_Write_Data[2] = 0xc0;//Up //1(UP)-10(ADC MID)-0(STEREO ON)-0000 (NO MUTE) :
	g_Tea_Write_Data[3] = 0x92; //1001-00-1(STEREO NOISE CANCELLING)-0
	g_Tea_Write_Data[4] = 0x00;
	Tea_RadioWrite();

	printf("Tea_RadioWriteUp=%x:%x:%x:%x:%x\r\n",g_Tea_Write_Data[0], g_Tea_Write_Data[1],g_Tea_Write_Data[2],g_Tea_Write_Data[3],g_Tea_Write_Data[4]);
	somedelay(1000000);
	//Tea_ReadByteBurst(g_Tea_Read_Data,5);
	//somedelay(100000);
	//printf("Tea_RadioRead=%x:%x:%x:%x:%x\r\n",g_Tea_Read_Data[0], g_Tea_Read_Data[1],g_Tea_Read_Data[2],g_Tea_Read_Data[3],g_Tea_Read_Data[4]);

}

void Tea_SearchDown(unsigned long freq)
{
	unsigned int pll = 0;
	pll= Tea_ConvFreq2Pll(freq);
	g_Tea_Write_Data[0] = ((pll >> 8)&0x003f) | 0x40;
	g_Tea_Write_Data[1] = pll & 0x00ff;
	g_Tea_Write_Data[2] = 0x60;//Down//0(DN)-11(ADC HIGH)-0(STEREO ON)-0000 (NO MUTE) :
	g_Tea_Write_Data[3] = 0x92;//
	g_Tea_Write_Data[4] = 0x00;
	Tea_RadioWrite();

	printf("Tea_RadioWriteDown=%x:%x:%x:%x:%x\r\n",g_Tea_Write_Data[0], g_Tea_Write_Data[1],g_Tea_Write_Data[2],g_Tea_Write_Data[3],g_Tea_Write_Data[4]);
	somedelay(1000000);

}
void Tea_SetFreq(unsigned long freq) //UP
{
	unsigned int pll = 0;

	pll= Tea_ConvFreq2Pll(freq);

	g_Tea_Write_Data[0] = ((pll >> 8)&0x003f);
	g_Tea_Write_Data[1] = pll & 0x00ff;
	g_Tea_Write_Data[2] = 0x60;
	g_Tea_Write_Data[3] = 0x90;
	g_Tea_Write_Data[4] = 0x00;
	Tea_RadioWrite();

	printf("SetFreq(%u): Tea_RadioWrite=%x:%x:%x:%x:%x\r\n",freq, g_Tea_Write_Data[0], g_Tea_Write_Data[1],g_Tea_Write_Data[2],g_Tea_Write_Data[3],g_Tea_Write_Data[4]);
	somedelay(100000);

}
void Tea_SetMute(unsigned long freq) //UP
{
	unsigned int pll = 0;

	pll= Tea_ConvFreq2Pll(freq);

	g_Tea_Write_Data[0] = ((pll >> 8)&0x003f) | 0x80;
	g_Tea_Write_Data[1] = pll & 0x00ff;
	g_Tea_Write_Data[2] = 0x40;
	g_Tea_Write_Data[3] = 0x90;
	g_Tea_Write_Data[4] = 0x00;
	Tea_RadioWrite();

	printf("Tea_RadioWrite=%x:%x:%x:%x:%x\r\n",g_Tea_Write_Data[0], g_Tea_Write_Data[1],g_Tea_Write_Data[2],g_Tea_Write_Data[3],g_Tea_Write_Data[4]);
	somedelay(100000);
}
//FM Tuner
//
void Tea_Search(signed char upDn) //UP/Down
{
	//Tea_RadioRead();
rep: if(upDn == TEA5767_UP){
		tea5767.Tea_Frequency += 100;
		if(tea5767.Tea_Frequency > maxFreq)
			tea5767.Tea_Frequency = maxFreq;
		printf("Up @ %u\r\n", tea5767.Tea_Frequency);
	}else if(upDn == TEA5767_DN){
		tea5767.Tea_Frequency -= 100;
		if(tea5767.Tea_Frequency < minFreq)
			tea5767.Tea_Frequency = maxFreq;
		printf("Dn @ %u\r\n", tea5767.Tea_Frequency);
	}else {
		return ;
	}

	Tea_SetFreq(tea5767.Tea_Frequency);
	somedelay(100000);
	Tea_ReadByteBurst(g_Tea_Read_Data,5);
	printf("Tea_RadioRead=%x:%x:%x:%x:%x\r\n",g_Tea_Read_Data[0], g_Tea_Read_Data[1],g_Tea_Read_Data[2],g_Tea_Read_Data[3],g_Tea_Read_Data[4]);

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
	g_Tea_Write_Data[0] = tea5767.tea_Pll/256;
	g_Tea_Write_Data[1] = tea5767.tea_Pll % 256;
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

void Tea_Tuning(signed char up1Down0)
{
	unsigned char level, stereo;
	unsigned int FreqH,FreqL;
	char str[16];

	if(up1Down0)
		Tea_Search(up1Down0);
	else
		return;

	level = Tea_GetLevel(); 		//g_Tea_Read_Data[3]>>4
	stereo = Tea_GetStereoMono(); 	//g_Tea_Read_Data[2]&0x80
	FreqH = tea5767.Tea_Frequency/1000;
	FreqL = tea5767.Tea_Frequency%1000;
	FreqL = FreqL/100;


#if (USE_DISPLAY == USE_DISPLAY_OLED)
	if(stereo){
		sprintf(str,"%d.%dMHz: Level=%d: Stereo",FreqH,FreqL,level);
		SSD1306_OLED_printChar_8X8_Inverse('S', 15,0);
	}else{
		sprintf(str,"%d.%dMHz: Level=%d: Mono   ",FreqH,FreqL,level);
		SSD1306_OLED_printChar_8X8_Inverse('M', 15,0);
	}
#else
	if(stereo){
		printf("%d.%dMHz: Level=%d: Stereo", FreqH, FreqL, level);
	}else
	{
		printf("%d.%dMHz: Level=%d: Mono   ",FreqH, FreqL, level);
	}
#endif
	//Show Frequency




#if (USE_DISPLAY == USE_DISPLAY_OLED)
	sprintf(str,"%d.%d MHz", FreqH, FreqL);
	SSD1306_OLED_printNumber_16X32(str,3,0,5); 	//SSD1306_OLED_printBigNumber(str,4,0,4);	//SSD1306_OLED_printString(str,4,3,15);
    SSD1306_OLED_printString_8X8("MHz",13,3,3);
#endif


	//    ST7735_Yoon_PrintStr(1,88,iastr);//    ST7735_Yoon_PrintStr(1,88,"IC: ST7735R");
//	sprintf(iastr,"%dMHz:Lv=%d:Mono",tea5767.Tea_Frequency,level);
//	Hd44780WriteString(iastr);

	//Not working???
	//Hd44780GotoXY(14,1);
	//Hd44780WriteByteData(0x00);

}



void tea5767GpioConf(){
	// Configure the GPIO for Tuning : PC4-PHA0, PC6-PHB0 : Tuning
	//dtQEIConfigExp0();
	GPIO_InitTypeDef GPIO_InitStruct;

	//(1) Tunning  up/down
	//BUTTON1 : PB8
	//BUTTON2 : PB9

	printf("Tea5767> Tunning CONFIG PB8=UP PB9=DOWN \r\n");

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// PB8-UP; PB9=DN;
void wm8731_Volume_Handler(unsigned char uselineormic)
{
		long readVal;
		int i;
		unsigned char curVal;

		readVal = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8); //active low
		if(readVal == 0){
			printf("Up\r\n");
			Wm8731SendVolControlSeq(VOLUP,uselineormic);
		};

		readVal = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9); //active low
		if(readVal == 0){
			printf("Dn\r\n");
			Wm8731SendVolControlSeq(VOLDN,uselineormic);
		};

/*		curVal = readVal & 0xa0;

		if(curVal == 0xa0){ //1010-0000
			delayms(100);
			return;
		}

		//printf("curVal=0x%02x\r\n",curVal);
		curVal = readVal & 0x80;
		if(curVal == 0){ //VolUp //PC7 -- Active Low
			printf("Up\r\n");
			Wm8731SendVolControlSeq(VOLUP,uselineormic);
			dtUserLed0Control(1);
			somedelay(6000);//6.8msec for 6000
			dtUserLed0Control(0);
		}
		curVal = readVal & 0x20;
		if(curVal == 0){ //VolDN //PC5 -- Active Low
			printf("Down\r\n");
			Wm8731SendVolControlSeq(VOLDN,uselineormic);
			dtUserLed0Control(1);
			somedelay(6000);//6.8msec for 6000
			dtUserLed0Control(0);
		}
*/

		delayms(100);
}

unsigned long Tea_FreqTbl[16] = {93100, 93900,93100, 93900,93100, 93900,93100, 93900,93100, 93900,93100, 93900,93100, 93900,93100, 93900};
//================================================================================================
extern unsigned short stm_Qei_GetValue(void);

void stmTea5767FmLoop(void)
{
	unsigned maxValue = 127;
	char str[16];
	unsigned char level;

	//(1) QEI Setup
	stm_Qei_Config(maxValue);

	//(2) I2C config for TEA5767 Tuner
	if(!g_bI2CModuleConfigDone)
	{
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
		  gI2Cx = I2C2;
#else
		  gI2Cx = I2C1;
#endif
		printf("I2C Init...");
		stm_I2C_Init(gI2Cx,400000);//100Kbps
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	}

	//(3) Setup FM Tuner.
	Tea_Setup(); //

	//(4) Display Setup
#if (USE_DISPLAY == USE_DISPLAY_OLED)
	sprintf(str,"FM ");//sprintf(str,"Tea5767 FMTUNER");
	SSD1306_OLED_printString_8X8(str,0,0,3);
#elif(USE_DISPLAY == USE_DISPLAY_MAX7219)

#elif(USE_DISPLAY == USE_DISPLAY_SAA1064)
#else
#endif

	while(1){

		//mcp23008val = lmMCP23008_ReadGpio();
		//tea5767.freqSel = (mcp23008val & 0xf0) >> 4;
		//tea5767.volSel  = (mcp23008val & 0x0f);
/*		if(savedFreqSel != freqSel){
			printf("frequency is %uKHz\r\n",Tea_FreqTbl[freqSel]);
			Tea_SetFreq(Tea_FreqTbl[freqSel]);
			savedFreqSel = freqSel;
		}

		if(tea5767.savedFreqSel < tea5767.freqSel){
			Tea_Tuning(1, 0);
			tea5767.savedFreqSel = tea5767.freqSel;
		}else if(tea5767.savedFreqSel > tea5767.freqSel){
			Tea_Tuning(0,1);
			tea5767.savedFreqSel = tea5767.freqSel;
		}

		if(tea5767.savedVolSel < tea5767.volSel){
			tea5767.savedVolSel = tea5767.volSel;
			printf("vol is %u\r\n",tea5767.volSel);
		}else if(tea5767.savedVolSel > tea5767.volSel){
			tea5767.savedVolSel = tea5767.volSel;
			printf("vol is %u\r\n",tea5767.volSel);
		}
*/
		tea5767.Chs = stm_Qei_GetValue(); //pos0 = QEIGet0();

		//Saa1064_Write4Digit(pos0,pos1); //Left/Right -- Display the value of Encoder on 4-LEDs
		//Tea_Tuning(0, 0);//Tea_Tuning(unsigned char nextb, unsigned char prevb)
		//Chs = Chs << 8;
		//Chs = Chs | pos1;

		if(tea5767.Chs != tea5767.saved_Chs){
				//Hd44780WriteString(astr);
				printf("Qei=%d\r\n",tea5767.Chs);
				//printf("CHs=%04x ", tea5767.Chs);
				if(tea5767.Chs > tea5767.saved_Chs)
					Tea_Tuning(TEA5767_UP);//1, 0);//Tea_Tuning(unsigned char nextb, unsigned char prevb)
				else if(tea5767.Chs < tea5767.saved_Chs)
					Tea_Tuning(TEA5767_DN);//(0, 1);//Tea_Tuning(unsigned char nextb, unsigned char prevb)
				else
					return;

				tea5767.saved_Chs = tea5767.Chs;
				//Tea_AutoSearch(0);
				printf("C=%d\r\n",tea5767.Tea_Frequency);//
				//Hd44780WriteString(astr);
				//WriteReg(2, Chs); //Channel Set (pos0=PlayChannel(MSB), pos1=SendChannel(LSB))
		}
	}
}
//=============================================================================
#if (USE_WM8731CODEC)
void stmTea5767FmWithWm8731Loop(u8 uselineormic, u8 bypassEn)
{
	unsigned maxValue = 127;
	char str[16];

	stm_Qei_Config(maxValue);

	if(!g_bI2CModuleConfigDone)
	{
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
		  gI2Cx = I2C2;
#else
		  gI2Cx = I2C1;
#endif
		printf("I2C Init...");
		stm_I2C_Init(gI2Cx,400000);//400Kbps
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	}


	//Saa1064 4-LED Driver Init
	//Saa1064Init();
	//QEI encoder -- automatically added by CoIDE

	//dtQEIConfigExp0(); //PD0, PD1 : Tune Up/Down

	//dtQEIConfigExp1();


	tea5767.freqSel = 0;
	tea5767.volSel = 8;

	//stmMCP23008_Init();
	tea5767.saved_Chs = 0;

	//long retEepromStatus;

	if(uselineormic == AUDIO_USELINEINPUT){
		printf("Tea5767 FM Tuner with WM8731 Audio Codec (LINEINPUT/OUTPUT) Test");
	}else{
		printf("Tea5767 Mic with WM8731 Audio Codec (MICINPUT/HPOUTPUT) Test");
	}
	printf("\r\n The FM tuner output is fed into the LineInput of WM8731 Codec, and\r\n");

	if(bypassEn)
		printf("the Codec's output will be out of the LineOutput because of Bypass Enabled.\r\n");
	else
		printf("the Codec's output will be on the I2S output. You need another slave I2C DAC.\r\n");

	printf("Use QEI for tuning, and Use Buttons for Volume Up/Down.\r\n");

	//codec Init
	Wm8731Init(uselineormic, bypassEn); //WM8731USEMICINPUT

	//Setup FM Tuner.
	Tea_Setup(); //

	while(1){

		//mcp23008val = lmMCP23008_ReadGpio();
		//tea5767.freqSel = (mcp23008val & 0xf0) >> 4;
		//tea5767.volSel  = (mcp23008val & 0x0f);
/*		if(savedFreqSel != freqSel){
			printf("frequency is %uKHz\r\n",Tea_FreqTbl[freqSel]);
			Tea_SetFreq(Tea_FreqTbl[freqSel]);
			savedFreqSel = freqSel;
		}
*/
		if(tea5767.savedFreqSel < tea5767.freqSel){
			Tea_Tuning(1, 0);
			tea5767.savedFreqSel = tea5767.freqSel;

		}else if(tea5767.savedFreqSel > tea5767.freqSel){
			Tea_Tuning(0,1);
			tea5767.savedFreqSel = tea5767.freqSel;
		}
/*
		if(tea5767.savedVolSel < tea5767.volSel){
			Wm8731SendVolControlSeq(VOLUP,uselineormic);
			tea5767.savedVolSel = tea5767.volSel;
			sprintf(str,"vol is %u",tea5767.volSel);
		}else if(tea5767.savedVolSel > tea5767.volSel){
			Wm8731SendVolControlSeq(VOLDN,uselineormic);
			tea5767.savedVolSel = tea5767.volSel;
			sprintf(str, "vol is %u",tea5767.volSel);
		}
*/
		continue;

		//WM8731Handler(uselineormic);

		//pos0 = QEIGet0();

		//pos1 = QEIGet1();
		//pos1 = 1;
		//Saa1064_Write4Digit(pos0,pos1); //Left/Right -- Display the value of Encoder on 4-LEDs
		//Tea_Tuning(0, 0);//Tea_Tuning(unsigned char nextb, unsigned char prevb)

		tea5767.Chs = tea5767.pos0;
		//pos0++;
		//pos1++;
		//Chs = Chs << 8;
		//Chs = Chs | pos1;

		continue;

		if(tea5767.Chs != tea5767.saved_Chs){
				printf("CHs=%x\r\n",tea5767.Chs);
				//Hd44780WriteString(astr);

				printf("CHs=%04x ", tea5767.Chs);
				if(tea5767.Chs > tea5767.saved_Chs)
					Tea_Tuning(1, 0);//Tea_Tuning(unsigned char nextb, unsigned char prevb)
				else
					Tea_Tuning(0, 1);//Tea_Tuning(unsigned char nextb, unsigned char prevb)
				//Tea_AutoSearch(0);
				printf("C=%d\r\n",tea5767.Tea_Frequency);//
				//Hd44780WriteString(astr);
				//WriteReg(2, Chs); //Channel Set (pos0=PlayChannel(MSB), pos1=SendChannel(LSB))
		}
		tea5767.saved_Chs = tea5767.Chs;
		//yprintf("Loop");
		//ssd_handler();
	}
}
#endif


//=============== TDA AMP ===
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
