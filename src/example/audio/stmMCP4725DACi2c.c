/* MCP4725/MCP4706 DAC with I2C
*/
#if 0
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
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
#define I2C_400KHZ						1

#define USE_MCP4725 12
#define USE_MCP4706 8
#define USE_MCP_DAC USE_MCP4706

//Address Option A2:A1=11
#if (USE_MCP_DAC == USE_MCP4725)
#define MCP47XX_ADDRESS				0xCC
#else
#define MCP47XX_ADDRESS				0xC2 //A1 (01), Marking DCXX
#endif

// registers
#define MCP47XX_WC_FASTMODE 	0x00
#define MCP47XX_WC_DATA     	0x40
#define MCP47XX_WC_DATA_EEPROM  0x60

#define MCP47XX_PD_NORMAL 		0x0
#define MCP47XX_PD_OTHERS       0x1

#define MCP47XX_GENERAL_RESET 	0x06
#define MCP47XX_GENERAL_WKUP 	0x09

unsigned char MCP47XXsendbuf[10];
unsigned char MCP47XXrecvbuf[5];
#if (USE_MCP_DAC == USE_MCP4725)
//Read 5 bytes of
unsigned char stmMCP4725_ReadRegAndEEPROM(void) {

	//if(!stm_I2C_SendBurst(MCP4725_ADDRESS, NULL, 0))
	//	return 0;

	if(!stm_I2C_ReceiveBurst(MCP47XX_ADDRESS, &MCP47XXrecvbuf[0], 5))
		return 0;
	else
		return 1;
}
#else //MCP4706
//Read 5 bytes of
unsigned char stmMCP4706_ReadRegAndEEPROM(void) {
	if(!stm_I2C_ReceiveBurst(MCP47XX_ADDRESS, &MCP47XXrecvbuf[0], 4))
		return 0;
	else
		return 1;
}
#endif

#if (USE_MCP_DAC == USE_MCP4725)
void stmMCP4725_WriteFastDacMode(unsigned char powerDown, unsigned short dacData12) {
	MCP47XXsendbuf[0]= MCP47XX_WC_FASTMODE | (powerDown << 4) | (dacData12 >> 8);
	MCP47XXsendbuf[1]= dacData12 & 0x00ff;
	stm_I2C_SendBurst(MCP47XX_ADDRESS, &MCP47XXsendbuf[0], 2);
}
#else
void stmMCP4706_WriteFastDacMode(unsigned char powerDown, unsigned char dacData8) {
	MCP47XXsendbuf[0]= MCP47XX_WC_FASTMODE | (powerDown << 4) | 0x00;
	MCP47XXsendbuf[1]= dacData8;
	stm_I2C_SendBurst(MCP47XX_ADDRESS, &MCP47XXsendbuf[0], 2);
}
#endif
#if (USE_MCP_DAC == USE_MCP4725)
void stmMCP4725_WriteNormalDacMode(unsigned char powerDown, unsigned short dacData12) {
	printf("TBD\r\n");
	MCP47XXsendbuf[0]= MCP47XX_WC_DATA | (powerDown << 1) ;
	MCP47XXsendbuf[1]= (dacData12 >> 4);
	MCP47XXsendbuf[2]= (dacData12 & 0x000f)<<4;
	stm_I2C_SendBurst(MCP47XX_ADDRESS, &MCP47XXsendbuf[0], 3);


#else
void stmMCP4706_WriteNormalDacMode(unsigned char powerDown, unsigned char vref, unsigned char gain, unsigned char dacData8)
{
	//printf("TBD\r\n");
	MCP47XXsendbuf[0]= MCP47XX_WC_DATA | (vref<<3) | (powerDown << 1) | gain ;
	MCP47XXsendbuf[1]= (dacData8 );
	//MCP47XXsendbuf[2]= (dacData8 ); //0x00; //???
	stm_I2C_SendBurst(MCP47XX_ADDRESS, &MCP47XXsendbuf[0], 2); //3

}
#endif
#if (USE_MCP_DAC == USE_MCP4725)
void stmMCP47XX_WriteNormalDacAndEepromMode(unsigned char powerDown, unsigned short dacData12) {
	printf("TBD\r\n");
	MCP47XXsendbuf[0]= MCP47XX_WC_DATA_EEPROM | (powerDown << 1) ;
	MCP47XXsendbuf[1]= (dacData12 >> 4);
	MCP47XXsendbuf[2]= (dacData12 & 0x000f)<<4;
	stm_I2C_SendBurst(MCP47XX_ADDRESS, &MCP47XXsendbuf[0], 3);
}
#else
void stmMCP4706_WriteNormalDacAndEepromMode(unsigned char powerDown, unsigned char vref, unsigned char gain, unsigned char dacData8) {
	printf("TBD\r\n");
	MCP47XXsendbuf[0]= MCP47XX_WC_DATA | (vref<<3) | (powerDown << 1) | gain ;
	MCP47XXsendbuf[1]= (dacData8 );
	MCP47XXsendbuf[2]= (dacData8 );
	stm_I2C_SendBurst(MCP47XX_ADDRESS, &MCP47XXsendbuf[0], 3);
}
#endif

#if (USE_MCP_DAC == USE_MCP4725)
#else
void stmMCP4706_WriteEepromOnly(unsigned char powerDown, unsigned char vref, unsigned char gain, unsigned char dacData8) {
	printf("TBD\r\n");
	MCP47XXsendbuf[0]= 0x80 | (vref<<3) | (powerDown << 1) | gain ;
	stm_I2C_SendBurst(MCP47XX_ADDRESS, &MCP47XXsendbuf[0], 1);
}
#endif


#define PI 3.14156
#define SAMPLING_RATE 8000
#define FREQ_GIVEN 100

//#define SAMPLES 256
//Num samples for a full period = 8000/f
#define SAMPLES (SAMPLING_RATE/FREQ_GIVEN)
//1KHz => 8
//100Hz => 80
//10Hz => 800
//1Hz => 8000


#if (USE_MCP_DAC == USE_MCP4706)
unsigned char g_sineWaveTable[SAMPLES];
//8bit
void stmMCP4706_prepareSineWave(float freq){
	int i;
	double x;
	unsigned short value;
	float step;

#if 1
	step = freq * ((2.0*3.14159))/SAMPLING_RATE;
	for(i=0;i<SAMPLES;i++){
		g_sineWaveTable[i] = sin(i*step)*128 + 128;
		printf("sin(%u)=%s \r\n", i, float2str(sin(i*step)));
		//printf("step1=%s\r\n", float2str(step));
	}

#else
	for (i=0; i<SAMPLES;i++){
		//360 degree / 256 = 1.40625
		//-1, +1
		//+1 ==> 0~2
		//*128 --> 0..255
		x = (sinf((2*PI/SAMPLES)*i)+1)*128;
		value = (unsigned char)rint(x);
		g_sineWaveTable[i] = value < 256 ? value : 255;
		printf("sineTable[%u]=%u\r\n",i,g_sineWaveTable[i]);
	}
#endif
}
void stmMCP4706_Loop() {
	int i;
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

	//MCP47XXsendbuf[0]= 0x09; //wakeup
	//stm_I2C_SendBurst(0x00, MCP47XXsendbuf, 1);
	//printf("Wakeup\r\n");
	stmMCP4706_prepareSineWave(FREQ_GIVEN);

	if(!stmMCP4706_ReadRegAndEEPROM())
		printf("Fail to read MCP4706\r\n");

	/* +---------------------------+--------------------+--------------------+-------
	 * |RDY-POR-0-VERF1:0-PD[1:0]-G|
	 */
	printf("STS[7:6]=%u\r\n", (MCP47XXrecvbuf[0] & 0xC0) >> 6);
	printf("VREF[4:3]=%u\r\n", (MCP47XXrecvbuf[0] & 0x18) >> 3);
	printf("PD[2:1]=%u\r\n", (MCP47XXrecvbuf[0] & 0x06) >> 1);
	//printf("D[11:4]=%02x\r\n", MCP47XXrecvbuf[1]);
	//printf("D[3:0]=%02x\r\n", MCP47XXrecvbuf[2]>>4);

	i =0;
	MCP47XXsendbuf[0]= MCP47XX_WC_FASTMODE | 0x00; //preset to boost speed.
	while(1){
		stmMCP4706_WriteFastDacMode(0x00, g_sineWaveTable[i]);
		//stmMCP4706_WriteNormalDacMode(0x00, 3, 0, g_sineWaveTable[i]);//Not working...???
		//stmMCP4706_WriteNormalDacAndEepromMode(0x00, 0, 0, g_sineWaveTable[i]);


		delayms(100);//delayus(350);//125); //1KHz sine wave
		i++;
		if(i == SAMPLES)
			i = 0;
	}
}
#else
unsigned short g_sineWaveTable[SAMPLES];
void stmMCP4725_sineWaveGen(){
	int i;
	double x;
	unsigned short value;
	for (i=0; i<SAMPLES;i++){
		//360 degree / 256 = 1.40625
		//-1, +1
		//+1 ==> 0~2
		//*2048 --> 0..4096
		x = (sinf((2*PI/SAMPLES)*i)+1)*2048;
		value = (unsigned short)rint(x);
		g_sineWaveTable[i] = value < 4096 ? value : 4095;
		printf("sineTable[%u]=%u\r\n",i,g_sineWaveTable[i]);
	}
}

void stmMCP4725_Loop() {
	//unsigned short dacVal=2000;
	int i;
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

	//MCP47XXsendbuf[0]= 0x09; //wakeup
	//stm_I2C_SendBurst(0x00, MCP47XXsendbuf, 1);
	//printf("Wakeup\r\n");
	stmMCP4725_sineWaveGen();

	if(!stmMCP47XX_ReadRegAndEEPROM())
		printf("Fail to read MCP4725\r\n");

	printf("PD[1:0]=%u\r\n", (MCP47XXrecvbuf[0] & 0x06) >> 1);
	printf("D[11:4]=%02x\r\n", MCP47XXrecvbuf[1]);
	printf("D[3:0]=%02x\r\n", MCP47XXrecvbuf[2]>>4);

	i =0;
	while(1){
		stmMCP47XX_WriteFastDacMode(0x00, g_sineWaveTable[i]);
		delayms(1); //1KHz sine wave
		i++;
		if(i >= SAMPLES)
			i = 0;
	}
}
#endif

#endif
