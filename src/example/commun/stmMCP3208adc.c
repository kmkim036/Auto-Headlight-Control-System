/* MCP3208/3204 12 bit ADC with SPI
 * - Single end
 * - Control Bits = 1nnn : nnn = Channel
 * - Mode0 : latch on rising edge
 * - SPI with 8-bit segment communications (Should be in Fullduplex mode. We need return value while writing.)
 * - master : 0000-011n nnxx-xxxx xxxx-xxxx
 * - slave  : zzzz-zzzz zzz0-bit11.......bit0
 *
  ADCControl.c - A library for controling Jog with a MCP3208 ADC
 * 	  SPI : 1Mbps, 8 bit mode
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

//We test with SPI2
/*
#include <string.h>
#include <stdarg.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
//#include "stm32f4xx_spi.h"
#include "yInc.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
#include <time.h>
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

#include "ySpiDrv.h"

#define nCS0_3208_1      nCS0_H//{GPIO_SetBits(GPIOD, GPIO_Pin_10);}
#define nCS0_3208_0      nCS0_L //{GPIO_ResetBits(GPIOD, GPIO_Pin_10);}

//#define nCS0_3208_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
//#define nCS0_3208_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern unsigned char stmSpi2RdByte();
extern unsigned short stmSpi2WrByte(unsigned char inbyte);
//extern unsigned char stmSpi3RdByte();
//extern void stmSpi3WrByte(unsigned char inbyte);
void stmMCP320XInit(void){
	//use MODE 0
#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_GD32F130FX)
	//1Mbps, 8 bit mode
	stmSPI2_Config(1, nCS0, stmSPIMODE0, stmSPIDATA8);   //use PD10 for nCS0
	//stmSPI2_Config(2, nCS1,stmSPIMODE0,stmSPIDATA8);//
#else
	stmSPI3_Config(2, nCS1,stmSPIMODE0,stmSPIDATA8);//
#endif
	nCS0_3208_1;//nCs=1
	delayms(100);
	printf("MCP3208> SPI ADC TEST INIT\r\n");
};
//use MODE 0
unsigned short stmMCP3201_readADC() {
    unsigned long retVal;
    unsigned short readAnalogValue, readAnalogValueFirst;

    nCS0_3208_0;//nCs=0
    retVal = stmSpi2RdByte(); // retVal = stmSpi3RdByte();
    readAnalogValueFirst = (retVal & 0x01f) << 7; //get first valid B11~B7
    //read remaining 8 bit value
    retVal = stmSpi2RdByte(); // retVal = stmSpi3RdByte(); //last 7 bits
	nCS0_3208_1;//nCs=1
    readAnalogValue = readAnalogValueFirst |  (retVal>> 1);

    return readAnalogValue;
}

// read SPI data from MCP3208 chip, 8 possible adc's (0 thru 7)
//use MODE 0
unsigned short stmMCP3208_readADC(int adcnum) {
    unsigned long retVal;
    unsigned short readAnalogValue, readAnalogValueFirst;
    unsigned long dummy=0;
	int i, adcout;
	unsigned char commandout;
	unsigned short retInWrite;

	if ((adcnum > 7) || (adcnum < 0)) return -1; // Wrong adc address return -1

	//-write and read - 1(single-ended) - nnn(adcNum) -
	//(1)prepare start(1), single-ended bit(1), and msb of adcnum : 11n nn
    commandout = 0x06 | ((adcnum & 0x04) >> 2);

    nCS0_3208_0;//nCs=0
	//-write : 0000-01(start) | 1(single-ended) - n
    stmSpi2WrByte(commandout); //stmSpi3WrByte(commandout);

    //Write nn00-0000 and Read simultaneously
    commandout = (adcnum & 0x3) << 6;
    retInWrite = stmSpi2WrByte(commandout); //stmSpi3WrByte(commandout);
    readAnalogValueFirst = (retInWrite & 0x1f) << 8; //get first valid 5 bits
    //printf("ADC%u: retInWrite = %02x : readAnalogValueFirst=%04x\r\n", adcnum, retInWrite, readAnalogValueFirst);

    //read remaining 6 bit value
    retVal = stmSpi2RdByte(); // retVal = stmSpi3RdByte(); //last 8 bits
	nCS0_3208_1;//nCs=1
    readAnalogValue = readAnalogValueFirst |  retVal;

    return readAnalogValue;
}

#define ADCNUM 2 //1 //4 //8

void stmMCP3201Loop(){
	 unsigned char i;
	 static unsigned int rdval;

	 	stmMCP320XInit();

		while(1){
			rdval = stmMCP3201_readADC();
			printf("ADC%u>ReadValue=0x%04x(%u)\r\n",i,rdval,rdval);
			printf("\r\n");
			//stmMax7219_show4digits(rdval);
			delayms(100);
		}
}

void stmMCP3208AdcLoop(){
	 unsigned char i;
	 static unsigned int rdval;

	 	stmMCP320XInit();

		while(1){
			//rdval = stmMCP3208_readADC(2);
			for(i=0;i<ADCNUM;i++){
				rdval = stmMCP3208_readADC(i);
				printf("ADC%u>ReadValue=0x%04x(%u)\r\n",i,rdval,rdval);
			}
			printf("\r\n");
			delayms(100);
		}
}


