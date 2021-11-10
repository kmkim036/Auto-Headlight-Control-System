/* FXL6408 Eight GPIO with I2C
*/
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "yInc.h"
#if (PROCESSOR == STM32F030F4P6) || (PROCESSOR == STM32F030C8T6)
#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_exti.h"
//#include "misc.h"
#elif (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif ((PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "core_cm3.h"

#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6))
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
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, unsigned int I2Cspeed);
extern I2C_TypeDef *gI2Cx;

#define I2C_400KHZ						1		// 0 to use default 100Khz, 1 for 400Khz

// Addr
//#define FXL_ADDR 0x43 // 100 0011 //7bit address
//#define FXL_ADDR8 0x86 // 1000 0110 //8bit address

#define FXL_ADDR 0x44 // 100 0011 //7bit address
#define FXL_ADDR8 0x88 // 1000 0110 //8bit address

// registers
#define FXL_REG_ID        0x01	//product ID +
#define FXL_REG_IODIR    0x03
#define FXL_REG_OUTSTATE 0x05
#define FXL_REG_HIGHZ    0x07
#define FXL_REG_IN_PULLEN   0x0D
#define FXL_REG_INSTATE  0x0F

#define FXL_ID_SW_RST   0x1		//reset
#define FXL_ID_MFG_MASK 0xE0
#define FXL_ID_MFG      0xA0

#define FXL_DIR_INPUT 	0
#define FXL_DIR_OUTPUT 	1

static uint8_t out_state = 0x0;
static uint8_t dir_state = 0x0;
static uint8_t pull_state = 0x0;

unsigned char fxl_sendbuf[40];

unsigned char stmFXL_ReadReg(unsigned char reg) {
	unsigned char rdbuf;
	// read the current GPIO input
	stm_I2C_ReceiveBurstWithRestartCondition(FXL_ADDR8, reg, &rdbuf, 1);
	return rdbuf;
}

void stmFXL_WriteReg(unsigned char reg, unsigned char gpio) {

	fxl_sendbuf[0]= reg;
	fxl_sendbuf[1]= gpio;// all inputs
	stm_I2C_SendBurst(FXL_ADDR8, fxl_sendbuf, 2);
}

//==================================================
unsigned char stmFXL_ReadGpio(void) {
	unsigned char rdval;
	// read the current GPIO input
	rdval = stmFXL_ReadReg(FXL_REG_INSTATE);
	return rdval;
}

void stmFXL_WriteGpio(unsigned char gpio) {
	stmFXL_WriteReg(FXL_REG_OUTSTATE, gpio);
}

void stmFXL_SetPinMode(
		unsigned char p, //per pin (0..7)
		unsigned char direction) //0=input/1=output
{
  unsigned char iodir;
  unsigned char rdbuf;

  // only 8 bits!
  if (p > 7)
    return;
  iodir = stmFXL_ReadReg(FXL_REG_IODIR);
  // set the pin and direction
  if (direction == FXL_DIR_OUTPUT) {
    iodir |= 1 << p;
  } else {
    iodir &= ~(1 << p);
  }
  // write the new IODIR
  stmFXL_WriteReg(FXL_REG_IODIR,iodir);
}

void stmFXL_WritePins(
		unsigned char p,
		unsigned char d)
{
  unsigned char gpio;

  // only 8 bits!
  if (p > 7)
    return;

  // read the current GPIO output latches
  gpio = stmFXL_ReadGpio();

  // set the pin and direction
  if (d == 1) {
    gpio |= 1 << p;
  } else {
    gpio &= ~(1 << p);
  }

  // write the new GPIO
  stmFXL_WriteGpio(gpio);
}
/*
void stmFXL8_pullUpPins(unsigned char p, unsigned char d) {
  unsigned char gppu;

  // only 8 bits!
  if (p > 7)
    return;

  gppu = stmFXL_ReadReg(MCP23008_GPPU);
  // set the pin and direction
  if (d == 1) {
    gppu |= 1 << p;
  } else {
    gppu &= ~(1 << p);
  }
  // write the new GPIO
  stmFXL_WriteReg(FXL_REG_GPPU, gppu);
}
*/
unsigned char stmFXL_Init(void) {
	unsigned char prodId;

	//get id
	prodId = stmFXL_ReadReg(FXL_REG_ID);
	if((prodId & FXL_ID_MFG_MASK) != FXL_ID_MFG){
		printf("NoFXL\r\n");
		return 0;
	}else
		printf("Found FXL\r\n");

	stmFXL_WriteReg(FXL_REG_ID, prodId | FXL_ID_SW_RST); // reset to synch with local state

	//stmFXL_WriteReg(FXL_REG_HIGHZ, 0x00); //output from Output State reg, not High-Z

	stmFXL_WriteReg(FXL_REG_IODIR,0x00); //all inputs

  	printf("FXL> INIT\r\n");
  	return 1;
}

#define ROTARY_SWAP 1

void stmFXL_Loop() {
	unsigned char rdval, savedVal, tmpVal8;

	stmFXL_Init();

	while(1){
		tmpVal8 = stmFXL_ReadGpio();
		rdval = tmpVal8;
		if (ROTARY_SWAP)
			rdval = (rdval << 4) | (rdval >> 4); //nibble swap.

		if(rdval != savedVal){
			printf("FXL>ReadValue=0x%02x(%u)\r\n",rdval,rdval);
			savedVal = rdval;
		}
		delayms(100);
	}
}

