

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

#define PM2008ADDR8 	0x50 //8-bit addr// 0x28 == 7bit Addr
#define I2C_400KHZ	1	// 0 to use default 100Khz, 1 for 400Khz

extern void delayms(unsigned long ms);
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
extern char *float2str(float x);
//
#define pm2008_MEASRH_HOLD_CMD           0xE5
#define pm2008_MEASRH_NOHOLD_CMD         0xF5
#define pm2008_MEASTEMP_HOLD_CMD         0xE3
#define pm2008_MEASTEMP_NOHOLD_CMD       0xF3
#define pm2008_READPREVTEMP_CMD          0xE0
#define pm2008_RESET_CMD                 0xFE
#define pm2008_WRITERHT_REG_CMD          0xE6
#define pm2008_READRHT_REG_CMD           0xE7
#define pm2008_WRITEHEATER_REG_CMD       0x51
#define pm2008_READHEATER_REG_CMD        0x11
#define pm2008_ID1_CMD                   0xFA0F
#define pm2008_ID2_CMD                   0xFCC9
#define pm2008_FIRMVERS_CMD              0x84B8

//protos
unsigned char pm2008_readRegister8(unsigned char reg);
unsigned short pm2008_readRegister16(unsigned char reg);
void pm2008_writeRegister8(unsigned char reg, unsigned char value);

struct _pmReadData{
	unsigned char frmhdr, frmlen, sensorStatus;
	unsigned short measuringMode;
	unsigned short calib,pm1_0,pm2_5,pm10;
	unsigned short pm1_0tsi,pm2_5tsi,pm10tsi;
	unsigned short num[6];
	unsigned char cksum;
};

/*******************************************************************/
// Writes 8-bits to the specified destination register
void pm2008_writeRegister8(unsigned char reg, unsigned char value) {
	unsigned char regbuf[10];
   	int ret=0;
   	regbuf[0] = reg;
   	regbuf[1] = value;
    ret = stm_I2C_SendBurst(PM2008ADDR8, regbuf, 2);
}

  /**************************************************************************/
  /*  Reads 8-bits from the specified register  */
  /**************************************************************************/
unsigned char pm2008_readRegister8(unsigned char reg) {
    unsigned char value;
   	unsigned char regbuf[10];
   	int ret=0;

	stm_I2C_ReceiveBurstWithRestartCondition(PM2008ADDR8, reg, &regbuf[0], 1);
    value = regbuf[0];
    return value;
  }

unsigned short pm2008_readRegister16(unsigned char reg) {
  unsigned short value;
  unsigned char regbuf[10];
  stm_I2C_ReceiveBurstWithRestartCondition(PM2008ADDR8, reg, &regbuf[0], 2);
  value = (short)((unsigned short)(regbuf[1]*256) + regbuf[0]);
  return value;
}

unsigned char pm2008_init(void) {
	unsigned char regbuf[7] = {
			0x16,//frame header
			0x07,//length
			0x03,//continuous measurement (default)
			0xff, 0xff, //measurement time duration + calibCoeff.
			0x00, //rsvd
			0x12}; //cksum
	int i;

	//cksum
	//regbuf[6] = regbuf[0];
	//for(i=1;i<6;i++){
	//	regbuf[6] ^= regbuf[i];
	//}
	//regbuf[6] = 0x12; //cksum

	stm_I2C_SendBurst(PM2008ADDR8, regbuf, 7);

	delayms(50);
    return 1;
}


unsigned char pm2008_readDust(struct _pmReadData *pmData) {
	unsigned char *rxbuf;
	int i;

	if(!stm_I2C_ReceiveBurst(PM2008ADDR8, (unsigned char *)pmData, 32)){
		printf("i2c Err");
		return 0;
	}
	if(pmData->frmhdr != 0x16){
		printf("err Rx\r\n");
		return 0;
	}
	printf("frameLen = %u\t", pmData->frmlen);

	if(pmData->sensorStatus == 2)
		printf("sensorStatus = Testing(%u)", pmData->sensorStatus);
	else if(pmData->sensorStatus == 7)
		printf("sensorStatus = Alarm(%u)", pmData->sensorStatus);
	else if(pmData->sensorStatus == 0x80)
		printf("sensorStatus = Stable(%u)", pmData->sensorStatus);
	else
		printf("sensorStatus = %u", pmData->sensorStatus);

	rxbuf = (unsigned char *)pmData;
	//measuring mode
	pmData->measuringMode = rxbuf[3]*0x100 + rxbuf[4];
	pmData->calib = rxbuf[5]*0x100 + rxbuf[6];
	pmData->pm1_0 = rxbuf[7]*0x100 + rxbuf[8];
	pmData->pm2_5 = rxbuf[9]*0x100 + rxbuf[10];
	pmData->pm10 = rxbuf[11]*0x100 + rxbuf[12];

	pmData->pm1_0tsi = rxbuf[13]*0x100 + rxbuf[14];
	pmData->pm2_5tsi = rxbuf[15]*0x100 + rxbuf[16];
	pmData->pm10tsi = rxbuf[17]*0x100 + rxbuf[18];

	//...

	//the last 32th byte is the checksum (regbuf[0]^....regbuf[30])

	printf("\tmeasuringMode = %u", pmData->measuringMode);
	printf("\tcalib = %u\r\n", pmData->calib);
	printf("pm1.0 = %u", pmData->pm1_0);
	printf("\tpm2.5 = %u", pmData->pm2_5);
	printf("\tpm10 = %u\r\n", pmData->pm10);

	printf("pm1.0tsi = %u", pmData->pm1_0tsi);
	printf("\tpm2.5tsi = %u", pmData->pm2_5tsi);
	printf("\tpm10tsi = %u\r\n", pmData->pm10tsi);
	//data
	//printf("p[]=");
	for(i=0;i<32;i++){
		if(i == 16)
			printf("\r\n");
		printf("%02x ", i, rxbuf[i]);
	}

	return 1;
}
void pm2008i2c_loop() {
	struct _pmReadData pmData;
	unsigned char success = 0;

	printf("pm2008 Dust Sensor Test with I2C (8bit I2C addr = 0x%02x).\r\n",PM2008ADDR8);

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

	pm2008_init();

	while(1){
		success = pm2008_readDust(&pmData);
		if(success)
			printf("Dust : pm2.5 = %u, pm10 = %u\r\n",pmData.pm2_5,pmData.pm10);
		else
			printf("No PM2008M\r\n");
		printf("====================\r\n");
		delayms(1000);
	}
}

