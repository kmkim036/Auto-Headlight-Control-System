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
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"

//?? Why 500KHz wave is imposed with the selected tone?

//i2c
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;

extern void stmMCP23S17_InitForSn76489(unsigned char whichSPI);
extern void stmMCP23S17_WriteGpio(unsigned char AB, unsigned char gpio) ;

// MCP4725 DAC I2C Address
#define MCP4725ADDR8 0x34

#define CH1 1
#define CH2 2
#define CH3 3
#define CH4 4


unsigned char swapbits(unsigned char num){
	unsigned char tmp = 0;
	unsigned char i=0;
	for(i=0;i<7;i++){
		tmp |= (num & 1);
		num >>= 1;
		tmp <<= 1;
	}
	if(num & 1)	tmp |= 1; //last bit
	return tmp;
}
//======================================
void Sn76489_WriteByte(unsigned char byte){

	unsigned char tmp;
	tmp = swapbits(byte);

	printf("swapbits 0x%02x --> 0x%02x\r\n", byte, tmp);
	stmMCP23S17_WriteGpio('A', tmp);//byte);//D[7:0]
	somedelay(1000);
	stmMCP23S17_WriteGpio('B', 0b00000000); //nCE=0, nWE=0, if RDY==1
	somedelay(8000);
	stmMCP23S17_WriteGpio('B', 0b00000110); //nCE=1, nWE=1, if RDY==1
	somedelay(5000);//delayms(1);
}
void Sn76489_WriteShort(unsigned short word){
	unsigned char byte;
	//msb
	byte = word >> 8;
	Sn76489_WriteByte(byte);
	//lsb
	Sn76489_WriteByte(word & 0xff);
}
//+-+-+-+-+-----+ +-+-+--------+
//|1|CH |0|F3~F0| |0|X|F9..F4  |
//+-+-+-+-+-----+ +-+-+--------+
//======================================
void Sn76489SetToneFreqWithNfreq(
		unsigned char channel,
		unsigned short nfreq)
{
	unsigned short word;
	unsigned char f3f0, f9f4;

	f3f0 = nfreq & 0x000f;
	f9f4 = (nfreq >> 4);
	nfreq = f3f0;
	nfreq <<= 8;
	nfreq |= f9f4;

	if(channel == CH1){
		word = 0x8000 | 0x0000 | nfreq;
		Sn76489_WriteShort(word);
	}
	else if(channel == CH2){
		word = 0x8000 | 0x2000 | nfreq;
		Sn76489_WriteShort(word);
	}
	else if(channel == CH3){
		word = 0x8000 | 0x4000 | nfreq;
		Sn76489_WriteShort(word);
	}
	else{
		//Not happend
	}
}
//Attenuation
//A[3:0] = 0 -> 0dB ?
//A[3:0] = 1 -> 2dB
//A[3:0] = 2 -> 4dB
//A[3:0] = 3 -> 8dB
//A[3:0] = 4 -> 16dB
//A[3:0] = F -> OFF (No OUTPUT?)
//+-+-+-+-+-----+
//|1|CH |1|A3~A0|
//+-+-+-+-+-----+
void Sn76489SetToneAttenuation(
		unsigned char channel,
		unsigned char atten)
{
	unsigned char byte;
	if(channel == CH1){
		byte = 0x80 | 0x10 | atten;
	}else if(channel == CH2){
		byte = 0x80 | 0x30 | atten;
	}else if(channel == CH3){
		byte = 0x80 | 0x50 | atten;
	}else if(channel == CH4){ //Noise channel
		byte = 0x80 | 0x70 | atten;
	}

	Sn76489_WriteByte(byte);
}
//+-+-+-+-+-+-+----+
//|1 1 1 0|x|F|NF10|
//+-+-+-+-+-+-+----+
void Sn76489UpdateNoiseSource(
		unsigned char fb, //noise feedback control (0=Periodic Noise; 1=White Noise)
		unsigned char nf) //Noise Freq control (00=N/512; 1=N/1024; 2=N/2048; 3=CH3 Output)
{
	unsigned char byte=0x00;

	byte = 0xe0 | (fb<<2) | nf;
	Sn76489_WriteByte(byte);
}
unsigned short Sn76489GetFreqDivideN(unsigned short freq){
	unsigned short nfreq;
    //freq = 3.579545MHz/32/NFREQ;
    nfreq = 3579545/32/freq;
	if(nfreq>1023){
		printf("Too low freq...\r\n");
		nfreq = 1023; //10-bits max.
	}
	printf("freq=%u-->nfreq=%u\r\n",freq, nfreq);
	return nfreq;
}

void stmSn76489Config(void){
	unsigned short freq, nfreq;
	unsigned char channel, atten;

	printf("\r\nSn76489 ToneGen with MCP23S17 Config \r\n");

	stmMCP23S17_InitForSn76489(USE_SPI2);

    atten = 0x0; //0xf=MUTE
    Sn76489SetToneAttenuation(CH1, atten);
    Sn76489SetToneAttenuation(CH2, atten);
    Sn76489SetToneAttenuation(CH3, 0xf); //Mute CH3
    Sn76489SetToneAttenuation(CH4, 0xf); //Mute noise
    //Sn76489UpdateNoiseSource(0, 0);
}

void Sn76489SetToneFreq(unsigned char channel, unsigned short freq){
	unsigned short nfreq;
	nfreq = Sn76489GetFreqDivideN(freq);
	Sn76489SetToneFreqWithNfreq(channel, nfreq);
}
//===========================================
void stmSn76489Loop(void)
{
	unsigned short freq1, freq2, nfreq;

	printf("\r\nSn76489 ToneGen with MCP23S17 Test \r\n");

	stmSn76489Config();

    //Sn76489SetToneFreq(CH3, nfreq);
	freq1 = 100;
	while(1){
		//Tone 1
		Sn76489SetToneFreq(CH1, freq1);
		//Tone 2
		freq2 = freq1 + 100;
		Sn76489SetToneFreq(CH2, freq2);
		//Tone 3
	    //Sn76489SetToneFreq(CH3, nfreq);
		freq1 +=100;
		if(freq1 > 1023)
			freq1 = 100;
		delayms(3000);
	}

}
