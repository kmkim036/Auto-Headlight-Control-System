#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6)|| (PROCESSOR == PROCESSOR_STM32F103RCT6)  || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#endif
#include "ySpiDrv.h"
//Read Two Bytes
//nCs1 of TC77 is Pin15 (nCS1) = PD6
//use nCS0 of PB12 : 103
//use nCS0 of PA15 : 107
#define nCS_TC77_H nCS0_H
#define nCS_TC77_L nCS0_L

#if (PROCESSOR == PROCESSOR_STM32F401RET6)
extern void stmSpi1_Config(unsigned char nCS);
extern unsigned char stmSpi1RdByte();
extern void stmSpi1WrByte(unsigned char inbyte);
#elif (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
extern unsigned char stmSpi2RdByte();
extern void stmSpi2WrByte(unsigned char inbyte);//Read Two Bytes
extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
void stmTC77_ReadTemperatureLoop(){
    unsigned short retVal,retVal_MSB,retVal_LSB;

    printf("TC77 Temperature Test with SPI2 Mode 3, 1Mbps, 16bit data.");

    //Use nCS0(PB12), Mode 3, 8bit Mode,
    stmSPI2_Config(
    		1, //1Mbps
    		0, //use nCS0 on PB12
    		3, //Mode 3
    		8); //data 8 bit (or 16)

    nCS_TC77_H;//nCs0=1

    while(1){
    	nCS_TC77_L;//nCs0=0

    	//Read Two Bytes
    	retVal_MSB = stmSpi2RdByte(); 		//MSB
    	retVal = 	 stmSpi2RdByte(); 	//LSB
    	retVal_LSB = retVal;
    	retVal = (retVal_MSB << 8) + retVal_LSB; 	//MSB + LSB

    	nCS_TC77_H;//nCs0=1
    	printf("msb=%d, lsb=%d\r\n", retVal_MSB,retVal_LSB); //TBD
    	printf("Temp=%d(Need converstion with floating point value)\r\n", retVal); //TBD
    	delayms(2000);
    }
    return 1; //retVal;
}
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
#define M66 1
extern unsigned char stmSpi3RdByte();
extern void stmSpi3WrByte(unsigned char inbyte);//Read Two Bytes
extern void stmSPI3_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
void stmTC77_ReadTemperatureLoop(){
    unsigned short retVal,retVal_MSB;

    printf("TC77 Temperature Test with SPI3 Mode 3, 1Mbps, 16bit data.");
#if M66
    //Use nCS0(PB12), Mode 3, 8bit Mode, -- STM32F103
    //Use nCS0(PA15), Mode 3, 8bit Mode, -- STM32F107
    stmSPI3_Config(
    		1, //1Mbps
    		0, //use nCS0 on PA15
    		3, //Mode 3
    		8); //data 8 bit (or 16)

    nCS_TC77_H;//nCs0=1

    while(1){
    	nCS_TC77_L;//nCs0=0

    	//Read Two Bytes
    	retVal_MSB = stmSpi3RdByte(); 		//MSB
    	retVal = 	 stmSpi3RdByte(); 		//LSB
    	retVal = retVal_MSB << 8 + retVal; 	//MSB + LSB

    	nCS_TC77_H;//nCs0=1

    	printf("Temp=%d(Need converstion with floating point value)\r\n", retVal); //TBD
    	delayms(2000);
    }
    return 1; //retVal;
#else //M43 = The same as the STM32F407
    //Use nCS0(PB12), Mode 3, 8bit Mode, -- STM32F103
    //Use nCS0(PA15), Mode 3, 8bit Mode, -- STM32F107
    stmSPI2_Config(
    		1, //1Mbps
    		0, //use nCS0 on PA15
    		3, //Mode 3
    		8); //data 8 bit (or 16)

    nCS_TC77_H;//nCs0=1

    while(1){
    	nCS_TC77_L;//nCs0=0

    	//Read Two Bytes
    	retVal_MSB = stmSpi2RdByte(); 		//MSB
    	retVal = 	 stmSpi2RdByte(); 		//LSB
    	retVal = retVal_MSB << 8 + retVal; 	//MSB + LSB

    	nCS_TC77_H;//nCs0=1

    	printf("Temp=%d(Need converstion with floating point value)\r\n", retVal); //TBD
    	delayms(1000);
    }
    return 1; //retVal;
#endif
}
#endif


