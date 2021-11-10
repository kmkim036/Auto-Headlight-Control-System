#include "yInc.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
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
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"

//i2c
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;

/// I2C Address
#define WM8731ADDR8 0x34 // 0001 1010 (7Bit address) + W# ;//#define WM8731ADDR 0x34 // 0011 0100



//BUTTON1 : PB8
//BUTTON2 : PB9
unsigned char g_vol_val;
unsigned char b_mute = 0;

//PC7(SW1) = VolUP
//PC5(SW0) = VolDown
//CODEC's ROLE = MASTER for I2S
//CODEC's ROLE = SLAVE for I2C; MCU's ROLE = MASTER for I2C

//Codec = master case
unsigned char initWM8731SeqData[]={
	0x1e,0x00, 		//reset
	0x01, 0x17, 	//Left LineMute=Disable 	//0x01,0x97, //Left LineMute=Enable (TEST)
	0x03,0x17,  	//Rightn LineMute=Disable 	//0x03,0x97,  //Rightn LineMute=Enable (TEST)
	0x05, 0x79,  	//LHPhone(LRHBOTH+!LZCEN+6dB)
	0x07, 0x79,   	//RHPhone(LRHBOTH+!LZCEN+6dB)

	//ANALOG AUDIO PATH CONTROL
	//0x08 | SideAttenuation(2bits=00) + SideTone(1bit) + DACSEL(1bit) | Bypass(1bit) + InSelect(1bit) + MuteMic(1bit)+ MicBoost(1bit)
	0x08, 0xFF,  //the 0xFF should be updated for correct configuration.
/*#if WM8731USELINEINPUT //-- Volume will control the Line Input Volume.
#if WM8731BYPASSENABLED
	0x08, 0x39,  //SideAtt(-6dB) + SideTone(1) + DACSEL(1) + BYPASS(1)+ INSEL(0=LineIn)+ MUTEMIC(0) + MICBOOST(1)
	//0x08, 0x29,  //SideAtt(-6dB) + SideTone(1) + DACSEL(0) + BYPASS(1)+ INSEL(0=LineIn)+ MUTEMIC(0) + MICBOOST(1)
#else
	0x08, 0x31,  //SideAtt(-6dB) + SideTone(1) + DACSEL(1) + BYPASS(0)+ INSEL(0=LineIn)+ MUTEMIC(0) +MICBOOST(1)
	//0x08, 0x11,  //SideAtt(-6dB) + SideTone(0) + DACSEL(1) + BYPASS(0)+ INSEL(0=LineIn)+ MUTEMIC(0) +MICBOOST(1)
#endif
#else // -- Volume will control the HP Output Volume.
#if WM8731BYPASSENABLED
	0x08, 0x3d,  //SideAtt(-6dB=00) + SideTone(1) + DACSEL(1) + BYPASS(1)+ INSEL(1=Mic)+ MUTEMIC(0) + MICBOOST(1) //Bypass
//	0x08, 0x2d,  //SideAtt(-6dB=00) + SideTone(1) + DACSEL(0) + BYPASS(1)+ INSEL(1=Mic)+ MUTEMIC(0) + MICBOOST(1) //Bypass
#else
	0x08, 0x35,  //SideAtt(-6dB=00) + SideTone(1) + DACSEL(1) + BYPASS(0)+ INSEL(1=Mic)+ MUTEMIC(0) + MICBOOST(1)
//	0x08, 0x15,  //SideAtt(-6dB)    + SideTone(0) + DACSEL(1) + BYPASS(0)+ INSEL(1=Mic)+ MUTEMIC(0) + MICBOOST(1)
#endif
#endif
*/
	//DIGITAL AUDIO PATH CONTROL : HPOR(When HPF disabled, Store DC offset) + DACMU(DAC Soft Mute Control) + Deemphasis(11=48KHz, 00=Disable) + ADCHPD(ADC HPF Enabled)
//	0x0a, 0x00, //HPOR(0)+DACMU(0)+DEEMP(00)(DISABLE)+ADCHPD(0)(enable HPF)
	0x0a, 0x07, //HPOR(0)+DACMU(0)+DEEMP(11=48KHz)+ADCHPD(1)(enable HPF)
	//PowerDownControl : All disable power down
	0x0c, 0x00,

	//I2S DigitalAudioIFformat(BCLK Inversion, Master/Slave, LRswapping, LRP=1(Right Channel Data when DACLRC=High) + InputWordLength + Format
//	0x0e, 0x5a, // (!BCLKINV+MS(1 Master)+LRSWAP(=0)+LRP(1)+InputWordLen(10=24bits)+Format(10=I2S)
	0x0e, 0x52, // (!BCLKINV+MS(1 Master)+LRSWAP(=0)+LRP(1)+InputWordLen(00=16bits)+Format(10=I2S)

	//Sampling Control : 48KHz
	0x10,0x00, //(CLKODIV2(0)+CLKDIV2(0)+SR(0000=48KHz)+BOSR(0=256fs)+ModeSel(USB/NormalMode=0)
	//Activate.
	0x12,0x01 //Active(=1) : We must activate.
};
/* If you need to set the wm8731 as a slave device, you will use the below.
//Codec = slave case
unsigned char initCodecSlaveSeqData[]={
	0x1e,0x00, //reset
	0x01, 0x17, //Left LineMute=Disable
	0x03,0x17,  //Rightn LineMute=Disable
	0x05, 0x7f,  //LHPhone(LRHBOTH+!LZCEN+6dB)
	0x07, 0x7f,   //RHPhone(LRHBOTH+!LZCEN+6dB)
	0x08, 0x35,  //AnalogPathCon (SideAtt(-6dB) + SideTone(1) + DACSEL(1) + BYPASS(0)+ INSEL(1)+!MUTEMIC+MICBOOST(1)
	0x0a, 0x06, //DigitalPathCon (!HPOR+!DACMU+DEEMP(48KHz)+!ADCHPD(enable HPF)
	0x0c, 0x00, //PowerDownControl : All disable power down
	0x0e, 0x1a, //DigitalAudioIFformat (!BCLKINV+MS(0 Slave)+LRSWAP(=0)+LRP(1)+InputWordLen(10=24bits)+Format(10=I2S)
	0x10,0x00, //Sampling Control (CLKODIV2(0)+CLKDIV2(0)+SR(0000=48KHz)+BOSR(0=256fs)+ModeSel(USB/NormalMode=0)
	0x12,0x01 //Active(=1)
};
*/

unsigned char Wm8731SendActivateSeq()
{
	volatile unsigned char ucLoop;
	unsigned char *ptr;

	ptr = &initWM8731SeqData[0];
	for(ucLoop = 0; ucLoop < 11; ucLoop++){
		stm_I2C_SendBurst(WM8731ADDR8, ptr, 2); //{START | ADDR | DATA[0]=REG | DATA[1]=VALUE | STOP}
		ptr+=2;
		somedelay(10);//add some delay..(ºÒÇÊ¿ä??)
	}
	return 1;
}

void Wm8731SendVolControlSeq(unsigned char volCtrl,u8 uselineormic)
{
	unsigned char data[2];

  	switch(volCtrl){
  	case VOLUP:
  		printf("VOLUP ");
  		if(uselineormic == AUDIO_USEMICINPUT){ //7bit
  			if(g_vol_val != MAX_HPOUT_VOL) g_vol_val += 1;
  			else g_vol_val = MAX_HPOUT_VOL;
  	  	    data[0]= 0x04 | 0x01; ////LEFTHPOUT | BOTH

  		}else{ //LINE 5bit
  			if(g_vol_val != MAX_LINEIN_VOL) g_vol_val += 1;
  			else g_vol_val = MAX_LINEIN_VOL;
  	  	    data[0]= 0x00 | 0x01; /////LINEIN_VOL | BOTH
  		}
   	    data[1] = g_vol_val;
   	    stm_I2C_SendBurst(WM8731ADDR8,&data[0], 2);
  		somedelay(500000);
  	    break;
  	case VOLDN:
 		printf("VOLDN ");
		if(g_vol_val != 0x00) g_vol_val -= 1;
		else g_vol_val = 0x00;
  		if(uselineormic == AUDIO_USEMICINPUT){ //7bit
  			data[0]= 0x04 | 0x01; ////LEFTHPOUT | BOTH
  		}else
  			data[0]= 0x00 | 0x01; ////LINEIN_VOL | BOTH
  	    data[1] = g_vol_val;
  	    stm_I2C_SendBurst(WM8731ADDR8, &data[0], 2);
  		somedelay(500000);
  	    break;
  	case AUD_MUTE:

  		break;
  	default:
  		break;
  	}
}

//Ver 2 PA2-VOUP; PA3=VOLDN; PA4=MUTE
void Wm8731VolConfig(u8 uselineormic)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	//(1) Volume  up/down
	//BUTTON1 : PB8
	//BUTTON2 : PB9

	printf("wm8731> VOL CONFIG PB8=UP PB9=DOWN \r\n");

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	//GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7); //UP
	//GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5); //DOWN
	//GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4); // Mute
	somedelay(100);

	if(uselineormic == AUDIO_USELINEINPUT)
		g_vol_val=0x10;
	else
		g_vol_val=0x50;
}

void Wm8731Init(u8 uselineormic, u8 bypassEn){//unsigned char volsel){
	int i;


	Wm8731VolConfig(uselineormic);

	//SET ANALOG AUDIO PATH CONTROL
	//0x08 | SideAttenuation(2bits=00) + SideTone(1bit) + DACSEL(1bit) | Bypass(1bit) + InSelect(1bit) + MuteMic(1bit)+ MicBoost(1bit)
	if(uselineormic == AUDIO_USELINEINPUT){//-- Volume will control the Line Input Volume.
		if(bypassEn)
			initWM8731SeqData[0xb] = 0x39; //SideAtt(-6dB) + SideTone(1) + DACSEL(1) + BYPASS(1)+ INSEL(0=LineIn)+ MUTEMIC(0) + MICBOOST(1)
		else
			//initWM8731SeqData[0xb] = 0x11;  //SideAtt(-6dB) + SideTone(0) + DACSEL(1) + BYPASS(0)+ INSEL(0=LineIn)+ MUTEMIC(0) +MICBOOST(1)
			initWM8731SeqData[0xb] = 0x31;  //SideAtt(-6dB) + SideTone(1) + DACSEL(1) + BYPASS(0)+ INSEL(0=LineIn)+ MUTEMIC(0) +MICBOOST(1)
			//initWM8731SeqData[0xb] = 0x01;  //SideAtt(-6dB) + SideTone(0) + DACSEL(0) + BYPASS(0)+ INSEL(0=LineIn)+ MUTEMIC(0) +MICBOOST(1)
	}else{// -- Volume will control the HP Output Volume.
		if(bypassEn)
			initWM8731SeqData[0xb] = 0x3d;  //SideAtt(-6dB=00) + SideTone(1) + DACSEL(1) + BYPASS(1)+ INSEL(1=Mic)+ MUTEMIC(0) + MICBOOST(1) //Bypass
		else
			initWM8731SeqData[0xb] = 0x35;  //SideAtt(-6dB=00) + SideTone(1) + DACSEL(1) + BYPASS(0)+ INSEL(1=Mic)+ MUTEMIC(0) + MICBOOST(1)
	}

	Wm8731SendActivateSeq(); //set codecs as Master and activate
	somedelay(6000);//6.8msec for 6000

}

// PB8-VOUP; PB9=VOLDN; -----PA4=MUTE
void WM8731_Volume_Handler(u8 uselineormic)
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


void stmWm8731CodecLoop(u8 uselineormic, u8 bypassEn){

	printf("\r\nWm8731 Audio Codec Test with I2C\r\n");
	//I2c config
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

	Wm8731Init(uselineormic, bypassEn);

	while(1)
		WM8731_Volume_Handler(uselineormic);
}
