#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#else
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
#include "lwipopts.h"
#endif
#include <math.h>

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern char *float2str(float x);

extern I2C_TypeDef *gI2Cx;
extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c

extern void somedelay(long ulLoop);

#define L3G4200ADDR 0xD0 //0x68 = 7-bit address of HMC5883 compass -->d0

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define L3G4200_WHO_AM_I         0x0F
#define L3G4200_TEMP         0x26

static float adc_data_float_value = 0.0f;

void writeL3G4200(u8 reg, u8 val) {
  stmI2cSendbuf[0] = reg;             // send register address
  stmI2cSendbuf[1] = val;                 // send value to write
  stm_I2C_SendBurst(L3G4200ADDR, stmI2cSendbuf, 2);
}

// Reads num u8s starting from address register on device in to _buff array
unsigned char readL3G4200(u8 reg) {
	  //stmI2cSendbuf[0] = reg;             // send register address to read from
	  //stm_I2C_SendBurst(L3G4200ADDR, stmI2cSendbuf, 1);
	  //Wire.requestFrom(DEVICE, num);    // request 1 byte from device
	  stm_I2C_ReceiveBurstWithRestartCondition(L3G4200ADDR, reg, stmI2cRecvbuf, 1);     //Read a byte
	  return stmI2cRecvbuf[0];
}

void stmL3G4200_GRYO_Get_XYZ(short xyz[])
{
	//2's complement values

	u8 xMSB = readL3G4200(0x29); //OUT_X_H
	u8 xLSB = readL3G4200(0x28); //OUT_X_L
    xyz[0] = ((xMSB << 8) | xLSB);

    u8 yMSB = readL3G4200(0x2B);
    u8 yLSB = readL3G4200(0x2A);
    xyz[1] = ((yMSB << 8) | yLSB);

    u8 zMSB = readL3G4200(0x2D);
    u8 zLSB = readL3G4200(0x2C);
    xyz[2] = ((zMSB << 8) | zLSB);
  }

 int L3G4200Setup(int scale){   //From  Jim Lindblom of Sparkfun's code
	 // Enable x, y, z and turn off power down:
	 writeL3G4200(CTRL_REG1, 0b00001111);

      // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
	 writeL3G4200(CTRL_REG2, 0b00000000);

    // Configure CTRL_REG3 to generate data ready interrupt on INT2
    // No interrupts used on INT1, if you'd like to configure INT1
    // or INT2 otherwise, consult the datasheet:
	 writeL3G4200(CTRL_REG3, 0b00001000);

    // CTRL_REG4 controls the full-scale range, among other things:

    if(scale == 250){
    	writeL3G4200(CTRL_REG4, 0b00000000);
    }else if(scale == 500){
    	writeL3G4200(CTRL_REG4, 0b00010000);
    }else{
    	writeL3G4200(CTRL_REG4, 0b00110000);
    }

    // CTRL_REG5 controls high-pass filtering of outputs, use it
    // if you'd like:
    writeL3G4200(CTRL_REG5, 0b00000000);
   }


void stmL3G4200_GYRO_Init()
{
	unsigned char ret;
	ret = readL3G4200(L3G4200_WHO_AM_I);
	printf("WHO_AM_I (reg 0x0f) = 0x%02x (Should be 0xd3)\r\n",ret);
	L3G4200Setup(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
    delayms(1500); //wait for the sensor to be ready
}

void L3G4200Loop() {
	unsigned char Temp;
	short xyz[3];

	printf("L3G4200 Driver Test with I2C");

	stmL3G4200_GYRO_Init();

	while(1){
		Temp = readL3G4200(L3G4200_TEMP);
		printf("[Temperature] = %d Degree   ",Temp);
		stmL3G4200_GRYO_Get_XYZ(xyz);  // This will update x, y, and z with new values
		printf("[x,y,z] = (%d,%d,%d)\r\n",xyz[0],xyz[1],xyz[2]);
		delayms(500); //Just here to slow down the serial to make it more readable
	}
}


