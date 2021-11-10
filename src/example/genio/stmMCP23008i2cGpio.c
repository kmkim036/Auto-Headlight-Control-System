/* MCP23008 Eight GPIO with I2C
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

#define MCP23008_ADDRESS				0x40 	//7bit address(0x20) --> 0x40 (8 bit addr)
#define I2C_400KHZ						1		// 0 to use default 100Khz, 1 for 400Khz

// registers
#define MCP23008_IODIR 0x00
#define MCP23008_IPOL 0x01
#define MCP23008_GPINTEN 0x02
#define MCP23008_DEFVAL 0x03
#define MCP23008_INTCON 0x04
#define MCP23008_IOCON 0x05
#define MCP23008_GPPU 0x06
#define MCP23008_INTF 0x07
#define MCP23008_INTCAP 0x08
#define MCP23008_GPIO 0x09
#define MCP23008_OLAT 0x0A

#define MCP23008_DIR_INPUT 	1
#define MCP23008_DIR_OUTPUT 0
#define MCP23008_PU_INPUT 	1


unsigned char mcp23008sendbuf[40];

unsigned char stmMCP23008_ReadReg(unsigned char reg) {
	unsigned char rdbuf;
	// read the current GPIO input
	stm_I2C_ReceiveBurstWithRestartCondition(MCP23008_ADDRESS,MCP23008_GPIO, &rdbuf, 1);
	return rdbuf;
}

void stmMCP23008_WriteReg(unsigned char reg, unsigned char gpio) {

	mcp23008sendbuf[0]= MCP23008_GPIO;
	mcp23008sendbuf[1]= gpio;// all inputs
	stm_I2C_SendBurst(MCP23008_ADDRESS, mcp23008sendbuf, 2);// write8(MCP23008_GPIO, gpio);
}

//==================================================
unsigned char stmMCP23008_ReadGpio(void) {
	unsigned char rdval;
	// read the current GPIO input
	rdval = stmMCP23008_ReadReg(MCP23008_GPIO);
	return rdval;
}

void stmMCP23008_WriteGpio(unsigned char gpio) {
	stmMCP23008_WriteReg(MCP23008_GPIO, gpio);
}

void stmMCP23008_SetPinMode(unsigned char p, unsigned char direction) {
  unsigned char iodir;
  unsigned char rdbuf;

  // only 8 bits!
  if (p > 7)
    return;
  iodir = stmMCP23008_ReadReg(MCP23008_IODIR);
  // set the pin and direction
  if (direction == MCP23008_DIR_INPUT) {
    iodir |= 1 << p;
  } else {
    iodir &= ~(1 << p);
  }
  // write the new IODIR
  stmMCP23008_WriteReg(MCP23008_IODIR,iodir);
}

void stmMCP23008_WritePins(unsigned char p, unsigned char d) {
  unsigned char gpio;

  // only 8 bits!
  if (p > 7)
    return;

  // read the current GPIO output latches
  gpio = stmMCP23008_ReadGPIO();

  // set the pin and direction
  if (d == 1) {
    gpio |= 1 << p;
  } else {
    gpio &= ~(1 << p);
  }

  // write the new GPIO
  stmMCP23008_WriteGPIO(gpio);
}

void stmMCP23008_pullUpPins(unsigned char p, unsigned char d) {
  unsigned char gppu;

  // only 8 bits!
  if (p > 7)
    return;

  gppu = stmMCP23008_ReadReg(MCP23008_GPPU);
  // set the pin and direction
  if (d == 1) {
    gppu |= 1 << p;
  } else {
    gppu &= ~(1 << p);
  }
  // write the new GPIO
  stmMCP23008_WriteReg(MCP23008_GPPU, gppu);
}

void stmMCP23008_Init(void) {

  // set defaults! -- Auto Increment...
/*	mcp23008sendbuf[0]= MCP23008_IODIR; //the first register of 0
	mcp23008sendbuf[1]= 0xFF;// its value - all inputs
	mcp23008sendbuf[2]= 0x00;// the next register of 1
	mcp23008sendbuf[3]= 0x00;//2
	mcp23008sendbuf[4]= 0x00;//3
	mcp23008sendbuf[5]= 0x00;//4
	mcp23008sendbuf[6]= 0x00;//5
	mcp23008sendbuf[7]= 0x00;//6
	mcp23008sendbuf[8]= 0x00;//7
	mcp23008sendbuf[9]= 0x00;//8
	mcp23008sendbuf[10]= 0x00;//9
	//mcp23008sendbuf[11]= 0x00;//a
  	stm_I2C_SendBurst(MCP23008_ADDRESS, mcp23008sendbuf,11 );
*/
	stmMCP23008_WriteReg(MCP23008_IODIR,0xFF); //all inputs
	stmMCP23008_WriteReg(MCP23008_IPOL,0x00);
	stmMCP23008_WriteReg(MCP23008_GPINTEN,0x00);
	stmMCP23008_WriteReg(MCP23008_DEFVAL,0x00);
	stmMCP23008_WriteReg(MCP23008_INTCON,0x00);
	stmMCP23008_WriteReg(MCP23008_GPPU,0x00);
	stmMCP23008_WriteReg(MCP23008_INTF,0x00);
	stmMCP23008_WriteReg(MCP23008_INTCAP,0x00);
	stmMCP23008_WriteReg(MCP23008_GPIO,0x00);
	//MCP23008_OLAT 0x0A

  	//stmMCP23008_SetPinMode(0, MCP23008_DIR_INPUT);
  	//stmMCP23008_pullUpPins(0, HIGH);  // turn on a 100K pullup internally

  	printf("MCP23008> INIT\r\n");
}

void stmMCP23008_Loop() {
	unsigned char rdval, savedVal, tmpVal8;

	stmMCP23008_Init();

	while(1){
		tmpVal8 = stmMCP23008_ReadGpio();
		rdval = tmpVal8;
		rdval = (rdval << 4) | (rdval >> 4); //nibble swap.
		if(rdval != savedVal){
			printf("MCP23008>ReadValue=0x%02x(%u)\r\n",rdval,rdval);
			savedVal = rdval;
		}
		//delayms(1000);
	}
}

