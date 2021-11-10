/* MCP23S17 Eight GPIO with I2C
*/
/*
  LedControl.cpp - A library for controling Leds with a MAX7219/MAX7221
 *    Copyright (c) 2007 Eberhard Fahle
 *
 *    Modified for STM32F4xx by Arami
 * 	  SPI : 1Mbps, 8 bit mode
 *    Connections to MAX7219:
// SPI MODE = 0
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0CLK - PA2
//! - SSI0Fss - PA3 nCS0
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5

 *  Configures the SPI3 Peripheral.
 *  CLK - PC10
 *  MISO - PC11 -- NOT USED
 *  MOSI - PC12
 *  nCS1 - PD2
*/

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
#include "ySpiDrv.h"

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_GD32F130FX)
	#if (MCU_MODULE_VER >= 35)
	//use nCS0 of PB12
	#define nCS0_S17_1      nCS0_H //{GPIO_SetBits(GPIOB, GPIO_Pin_12);}
	#define nCS0_S17_0      nCS0_L //{GPIO_ResetBits(GPIOB, GPIO_Pin_12);}
	#else
	#define nCS0_S17_1      {GPIO_SetBits(GPIOB, GPIO_Pin_12);}
	#define nCS0_S17_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_12);}
	#endif
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
//use nCS0 of PA15
#define nCS0_S17_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
#define nCS0_S17_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
#endif

extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern void stmSpiWrByte(unsigned char inbyte);
extern void stmSPI_Config(unsigned char whichSPI, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern unsigned char stmSpiRdByte();

// registers(BANK 0)
#define MCP23S17_IODIRA 0x00
#define MCP23S17_IODIRB (MCP23S17_IODIRA+1)

#define MCP23S17_IPOLA 0x02
#define MCP23S17_IPOLB (MCP23S17_IPOLA+1)

#define MCP23S17_GPINTENA 0x04
#define MCP23S17_GPINTENB (MCP23S17_GPINTENA+1)

#define MCP23S17_DEFVALA 0x06
#define MCP23S17_DEFVALB (MCP23S17_DEFVALA+1)

#define MCP23S17_INTCONA 0x08
#define MCP23S17_INTCONB (MCP23S17_INTCONA+1)

#define MCP23S17_IOCONA 0x0A
#define MCP23S17_IOCONB (MCP23S17_IOCONA+1)

#define MCP23S17_GPPUA 0x0C
#define MCP23S17_GPPUB (MCP23S17_GPPUA+1)

#define MCP23S17_INTFA 0x0E
#define MCP23S17_INTFB (MCP23S17_INTFA+1)

#define MCP23S17_INTCAPA 0x10
#define MCP23S17_INTCAPB (MCP23S17_INTCAPA+1)

#define MCP23S17_GPIOA 0x12
#define MCP23S17_GPIOB (MCP23S17_GPIOA+1)

#define MCP23S17_OLATA 0x14
#define MCP23S17_OLATB (MCP23S17_OLATA+1)

#define MCP23S17_DIR_INPUT 	1
#define MCP23S17_DIR_OUTPUT 0
#define MCP23S17_PU_INPUT 	1

unsigned char MCP23S17sendbuf[3];
//We assume A[2:0] = 000
void stmMCP23S17_RegWrite(int reg, volatile unsigned char data) {
	nCS0_S17_0;
   	stmSpiWrByte(0x40);
   	stmSpiWrByte(reg);
   	stmSpiWrByte(data);
    nCS0_S17_1;
}
unsigned char stmMCP23S17_Read(int reg) {
	unsigned char readVal;

	nCS0_S17_0;
    stmSpiWrByte(0x41);
    stmSpiWrByte(reg);
    readVal = stmSpiRdByte();
    nCS0_S17_1;
    return readVal;
}
//read ReadyPin
//if 1 = ready true;
//0 == not ready
//PB0 = iRDY
unsigned char stmMCP23S17_isReady() {
	unsigned char retVal;
	nCS0_S17_0;
	retVal = stmMCP23S17_Read(MCP23S17_GPIOB);
	nCS0_S17_1;
	if(retVal & 0x01)
		return 1;
	else
		return 0;
}
//==================================================
unsigned char stmMCP23S17_ReadGpio(unsigned char AB) {
	unsigned char rdval;
	// read the current GPIO input
	if(AB=='A')
		rdval = stmMCP23S17_Read(MCP23S17_GPIOA);
	else
		rdval = stmMCP23S17_Read(MCP23S17_GPIOB);
	return rdval;
}

void stmMCP23S17_WriteGpio(unsigned char AB, unsigned char gpio) {
	if(AB=='A')
		stmMCP23S17_RegWrite(MCP23S17_GPIOA, gpio);
	else
		stmMCP23S17_RegWrite(MCP23S17_GPIOB, gpio);
}

void stmMCP23S17_SetPinMode(unsigned char AB, unsigned char p, unsigned char direction) {
  unsigned char iodir;
  unsigned char rdbuf;
  // only 8 bits!
  if (p > 7)  return;

  if(AB=='A')
	  iodir = stmMCP23S17_Read(MCP23S17_IODIRA);
  else
	  iodir = stmMCP23S17_Read(MCP23S17_IODIRB);
  // set the pin and direction
  if (direction == MCP23S17_DIR_INPUT) {
    iodir |= 1 << p;
  } else {
    iodir &= ~(1 << p);
  }
  // write the new IODIR
  if(AB=='A')
	  stmMCP23S17_RegWrite(MCP23S17_IODIRA,iodir);
  else
	  stmMCP23S17_RegWrite(MCP23S17_IODIRB,iodir);
}

void stmMCP23S17_RegWritePins(unsigned char AB, unsigned char p, unsigned char d) {
  unsigned char gpio;

  // only 8 bits!
  if (p > 7) return;

  // read the current GPIO output latches
  gpio = stmMCP23S17_ReadGpio(AB);

  // set the pin and direction
  if (d == 1) {
    gpio |= 1 << p;
  } else {
    gpio &= ~(1 << p);
  }

  // write the new GPIO
  stmMCP23S17_WriteGpio(AB, gpio);
}

void stmMCP23S17_pullUpPins(unsigned char AB, unsigned char p, unsigned char d) {
  unsigned char gppu;

  // only 8 bits!
  if (p > 7)    return;
  if(AB=='A')
	  gppu = stmMCP23S17_Read(MCP23S17_GPPUA);
  else
	  gppu = stmMCP23S17_Read(MCP23S17_GPPUB);
  // set the pin and direction
  if (d == 1) {
    gppu |= 1 << p;
  } else {
    gppu &= ~(1 << p);
  }
  // write the new GPIO
  if(AB=='A')
	  stmMCP23S17_RegWrite(MCP23S17_GPPUA, gppu);
  else
	  stmMCP23S17_RegWrite(MCP23S17_GPPUB, gppu);
}

void stmMCP23S17_InitForDefault(unsigned char whichSPI) {

	stmSPI_Config(whichSPI,
			0, //ncs0
			0, //MODE 0
			8);//8bit mode

	//PA[7:0] = D[7:0] All outputs
	stmMCP23S17_RegWrite(MCP23S17_IODIRB,0x00); //all output(0). Default is INPUT(0xFF)
	stmMCP23S17_RegWrite(MCP23S17_IODIRA,0x00); //all outputs(0). Default is INPUT(0xFF)
	//stmMCP23S17_RegWrite(MCP23S17_IODIRB,0b11111111); //all input(1)
	//stmMCP23S17_RegWrite(MCP23S17_IODIRB,0x00); //all output(0). Default is INPUT(0xFF)
	//Polarity -- Default..
	stmMCP23S17_RegWrite(MCP23S17_IPOLA,0x00);  //Input Polarity (0= as it; 1=reversed)
	stmMCP23S17_RegWrite(MCP23S17_IPOLB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_GPINTENA,0x00); //interrupt on change)
	stmMCP23S17_RegWrite(MCP23S17_GPINTENB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_DEFVALA,0x00);
	stmMCP23S17_RegWrite(MCP23S17_DEFVALB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_INTCONA,0x00);
	stmMCP23S17_RegWrite(MCP23S17_INTCONB,0x00);

	//stmMCP23S17_RegWrite(MCP23S17_IOCONA,0x00);
	//stmMCP23S17_RegWrite(MCP23S17_IOCONB,0x00);

	//stmMCP23S17_RegWrite(MCP23S17_GPPUA,0xFF); //Pull-up
	//stmMCP23S17_RegWrite(MCP23S17_GPPUB,0xFF);

	//stmMCP23S17_RegWrite(MCP23S17_INTFA,0x00); //Read Only
	//stmMCP23S17_RegWrite(MCP23S17_INTFB,0x00);

	//stmMCP23S17_RegWrite(MCP23S17_INTCAPA,0x00); //Read Only
	//stmMCP23S17_RegWrite(MCP23S17_INTCAPB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_GPIOA,0x00);
	stmMCP23S17_RegWrite(MCP23S17_GPIOB,0xFF);

	//stmMCP23S17_RegWrite(MCP23S17_OLATA,0x00);
	//stmMCP23S17_RegWrite(MCP23S17_OLATB,0x00);


  	printf("MCP23S17> INIT Default\r\n");
}




void stmMCP23S17_Loop() {
	unsigned char rdVal8;

	stmMCP23S17_InitForDefault(USE_SPI2);

	while(1){


		//stmMCP23S17_WriteGpio('A', 0xaa);
		//stmMCP23S17_WriteGpio('A', 0x55);
		stmMCP23S17_WriteGpio('B', 0xaa); //nCE=0, nWE=0, RDY(z)
		stmMCP23S17_WriteGpio('B', 0x54); //nCE=0, nWE=0, RDY(z)
		delayms(100);

		//rdVal8 = stmMCP23S17_ReadGpio('A');
		//printf("MCP23S17>ReadValueA=0x%02x(%u)\r\n",rdVal8);
		//rdVal8 = stmMCP23S17_ReadGpio('B');
		//printf("MCP23S17>ReadValueB=0x%02x(%u)\r\n",rdVal8);

		//delayms(1000);

	}
}
//========================================================

void stmMCP23S17_InitForSn76489(unsigned char whichSPI){
	int i;

	stmSPI_Config(whichSPI,
			0, //ncs0
			0, //MODE 0
			8);//8bit mode

	//PA[7:0] = D[7:0] All outputs
	stmMCP23S17_RegWrite(MCP23S17_IODIRA,0x00); //all outputs(0) -- to SN76489 (D7..D0)
	//PB0 = iRDY
	//PB1 = oWRn
	//PB2 = oCEn
	stmMCP23S17_RegWrite(MCP23S17_IODIRB,0x01); //all outputs except for PB0(iRDY)

	//Polarity -- Default..
	stmMCP23S17_RegWrite(MCP23S17_IPOLA,0x00); //Input Polarity (0= as it; 1=reversed)
	stmMCP23S17_RegWrite(MCP23S17_IPOLB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_GPINTENA,0x00); //interrupt on change)
	stmMCP23S17_RegWrite(MCP23S17_GPINTENB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_DEFVALA,0x00);
	stmMCP23S17_RegWrite(MCP23S17_DEFVALB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_INTCONA,0x00);
	stmMCP23S17_RegWrite(MCP23S17_INTCONB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_IOCONA,0x00);
	stmMCP23S17_RegWrite(MCP23S17_IOCONB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_GPPUA,0xFF); //Pull-up
	stmMCP23S17_RegWrite(MCP23S17_GPPUB,0xFF);

		//stmMCP23S17_RegWrite(MCP23S17_INTFA,0x00); //Read Only
		//stmMCP23S17_RegWrite(MCP23S17_INTFB,0x00);

		//stmMCP23S17_RegWrite(MCP23S17_INTCAPA,0x00); //Read Only
		//stmMCP23S17_RegWrite(MCP23S17_INTCAPB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_GPIOA,0x00);
	stmMCP23S17_RegWrite(MCP23S17_GPIOB,0x00);

	stmMCP23S17_RegWrite(MCP23S17_OLATA,0x00);
	stmMCP23S17_RegWrite(MCP23S17_OLATB,0x00);
  	printf("MCP23S17> INITforSn76489\r\n");
}
