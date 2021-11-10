

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
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "yInc.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
#include "lwipopts.h"
#endif

#define si7021ADDR8 	0x80 //8-bit addr// 0x40 == 7bit Addr
#define I2C_400KHZ	1	// 0 to use default 100Khz, 1 for 400Khz

extern void delayms(unsigned long ms);
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
extern char *float2str(float x);
//
#define SI7021_MEASRH_HOLD_CMD           0xE5
#define SI7021_MEASRH_NOHOLD_CMD         0xF5
#define SI7021_MEASTEMP_HOLD_CMD         0xE3
#define SI7021_MEASTEMP_NOHOLD_CMD       0xF3
#define SI7021_READPREVTEMP_CMD          0xE0
#define SI7021_RESET_CMD                 0xFE
#define SI7021_WRITERHT_REG_CMD          0xE6
#define SI7021_READRHT_REG_CMD           0xE7
#define SI7021_WRITEHEATER_REG_CMD       0x51
#define SI7021_READHEATER_REG_CMD        0x11
#define SI7021_ID1_CMD                   0xFA0F
#define SI7021_ID2_CMD                   0xFCC9
#define SI7021_FIRMVERS_CMD              0x84B8

//protos
float si7021_readTemperature(void);
void si7021_reset(void);
void si7021_readSerialNumber(void);
//unsigned long si7021_readHumidity(void);//
float si7021_readHumidity(void);
unsigned char si7021_readRegister8(unsigned char reg);
unsigned short si7021_readRegister16(unsigned char reg);
void si7021_writeRegister8(unsigned char reg, unsigned char value);

struct si7021 {
  unsigned long sernum_a, sernum_b;
} si7021;

/*******************************************************************/
// Writes 8-bits to the specified destination register
void si7021_writeRegister8(unsigned char reg, unsigned char value) {
	unsigned char regbuf[10];
   	int ret=0;
   	regbuf[0] = reg;
   	regbuf[1] = value;
    ret = stm_I2C_SendBurst(si7021ADDR8, regbuf, 2);
}

  /**************************************************************************/
  /*  Reads 8-bits from the specified register  */
  /**************************************************************************/
unsigned char si7021_readRegister8(unsigned char reg) {
    unsigned char value;
   	unsigned char regbuf[10];
   	int ret=0;

	stm_I2C_ReceiveBurstWithRestartCondition(si7021ADDR8, reg, &regbuf[0], 1);
    value = regbuf[0];
    return value;
  }

unsigned short si7021_readRegister16(unsigned char reg) {
  unsigned short value;
  unsigned char regbuf[10];
  stm_I2C_ReceiveBurstWithRestartCondition(si7021ADDR8, reg, &regbuf[0], 2);
  value = (short)((unsigned short)(regbuf[1]*256) + regbuf[0]);
  return value;
}

void si7021_reset(void) {
	unsigned char regbuf[10];
	regbuf[0] = SI7021_RESET_CMD;
	stm_I2C_SendBurst(si7021ADDR8, regbuf, 1);//	si7021_writeRegister8(SI7021_RESET_CMD);
	delayms(50);
}

void si7021_readSerialNumber(void) {
	unsigned char rcvbuf[10];
	rcvbuf[0] = (unsigned char)(SI7021_ID1_CMD&0xFF);
/*

	stm_I2C_ReceiveBurstWithRestartCondition(si7021ADDR8, SI7021_ID1_CMD>>8, &rcvbuf[0], 8);

  Wire.beginTransmission(si7021._i2caddr);
  Wire.write((unsigned char)(SI7021_ID1_CMD>>8));
  Wire.write((unsigned char)(SI7021_ID1_CMD&0xFF));
  Wire.endTransmission();

  Wire.requestFrom(si7021._i2caddr, 8);
  si7021.sernum_a = Wire.read();
  Wire.read();
  si7021.sernum_a <<= 8;
 si7021.sernum_a |= Wire.read();
  Wire.read();
  si7021.sernum_a <<= 8;
 si7021. sernum_a |= Wire.read();
  Wire.read();
 si7021.sernum_a <<= 8;
  si7021.sernum_a |= Wire.read();
  Wire.read();

  Wire.beginTransmission(si7021._i2caddr);
  Wire.write((unsigned char)(SI7021_ID2_CMD>>8));
  Wire.write((unsigned char)(SI7021_ID2_CMD&0xFF));
  Wire.endTransmission();

  Wire.requestFromsi7021.(_i2caddr, 8);
  si7021.sernum_b = Wire.read();
  Wire.read();
  si7021.sernum_b <<= 8;
  si7021.sernum_b |= Wire.read();
  Wire.read();
  si7021.sernum_b <<= 8;
  si7021.sernum_b |= Wire.read();
  Wire.read();
  si7021.sernum_b <<= 8;
  si7021.sernum_b |= Wire.read();
  Wire.read();
*/
}


unsigned char si7021_init(void) {
    si7021.sernum_a =   si7021.sernum_b = 0;
	//i2c
    si7021_reset();
    if (si7021_readRegister8(SI7021_READRHT_REG_CMD) != 0x3A)
    	return 0;
    else {
    	printf("SI7021_READRHT_REG_CMD = 3A\r\n");
	}
    si7021_readSerialNumber();
    printf("0x%02x, 0x%02x\r\n",si7021.sernum_a,si7021.sernum_b);
    return 1;
}

float si7021_readHumidity(void) {
//unsigned long si7021_readHumidity(void) {
	unsigned short hum ;
	float humidity;
	unsigned char cksum;
	unsigned char regbuf[10];
	unsigned char reg;

	reg = SI7021_MEASRH_NOHOLD_CMD;
	if(!stm_I2C_SendBurst(si7021ADDR8, &reg, 1)){
		printf("i2c Err");
		return 0;
	}

	delayms(25);//delay until the measurement will be completed.

	if(!stm_I2C_ReceiveBurst(si7021ADDR8, regbuf, 3)){
		printf("i2c Err");
			return 0;
	}
	hum = (unsigned short)((unsigned short)(regbuf[0]*256) + regbuf[1]);
	cksum = regbuf[2];

	//printf("hum=%d, cksum=%02x ", hum, cksum);
	humidity = hum;
	humidity *= 125;
	humidity /= 65536;
	humidity -= 6;
	return humidity;
}

float si7021_readTemperature(void) {
	unsigned short temp;
	unsigned char cksum;
	float temperature;
	unsigned char regbuf[10];

	unsigned char reg;

	reg = SI7021_MEASTEMP_NOHOLD_CMD;
	if(!stm_I2C_SendBurst(si7021ADDR8, &reg, 1)){
		printf("i2c Err");
		return 0;
	}

	delayms(25);

	if(!stm_I2C_ReceiveBurst(si7021ADDR8, regbuf, 3)){
		printf("i2c Err");
			return 0;
	}
	temp = (unsigned short)((unsigned short)(regbuf[0]*256) + regbuf[1]);
	cksum = regbuf[2];


	temperature = temp;
	temperature *= 175.72;
	temperature /= 65536;
	temperature -= 46.85;

	return temperature;
}
void si7021_loop() {
	float humidity,temperature;

	printf("si7021 Humidity/Temp Sensor Test with I2C (8bit I2C addr = 0x%02x).\r\n",si7021ADDR8);

	if(!g_bI2CModuleConfigDone){
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
		gI2Cx = I2C2;
#else
		gI2Cx = I2C1;
#endif

		printf("I2C Init with 100Kbps...");
		stm_I2C_Init(gI2Cx,100000);//100Kbps
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	}

	si7021_init();

	while(1){
		humidity = si7021_readHumidity();
		printf("Humidity:    %s ",float2str(humidity));
		temperature = si7021_readTemperature();
		printf("\tTemperature:  %s\r\n",float2str(temperature));
		delayms(1000);
	}
}

