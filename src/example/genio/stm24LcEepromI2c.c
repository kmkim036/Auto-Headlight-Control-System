/* MCP23008 Eight GPIO with I2C
*/

#include "yLib/include/yInc.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#if (PROCESSOR == STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif(PROCESSOR == STM32F103CCT6)
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"

#define EEPROM_24LC_32   32
#define EEPROM_24LC_64   64
#define EEPROM_24LC_128  128
#define EEPROM_24LC_256  256
#define EEPROM_24LC_512  512
#define USE_EEPROM 		 EEPROM_24LC_32
#if(USE_EEPROM == EEPROM_24LC_32)
	#define MEMSIZE 4096
	#define BUFSIZE 32	//bytes in sequence -- 64?
#elif(USE_EEPROM == EEPROM_24LC_64)
	#define MEMSIZE 8192
	#define BUFSIZE 32
#elif(USE_EEPROM == EEPROM_24LC_128)
	#define MEMSIZE 16384
	#define BUFSIZE 64
#elif(USE_EEPROM == EEPROM_24LC_256)
	#define MEMSIZE 32768
	#define BUFSIZE 64
#elif(USE_EEPROM == EEPROM_24LC_512)
	#define MEMSIZE 65536
	#define BUFSIZE 128
#endif

#define EEPROM24LC_ADDR7				0x50 	//7bit address
#define EEPROM24LC_ADDR8				0xA0 	//8bit address(0x50) --> 0xA0 (8 bit addr)
#define I2C_400KHZ				1		// 0 to use default 100Khz, 1 for 400Khz

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c

void stm24LC_WriteMem(unsigned short addr, unsigned char *pd, unsigned char datalen)
{
	if( datalen > 8){
		printf("Limit 8 in sequence\r\n");
		datalen  = 8;
	}
	//mem addr
    stmI2cSendbuf[0] = (unsigned char)((addr & 0xff00) >> 8); //high
    stmI2cSendbuf[1] = (unsigned char)(addr & 0xff);		  //low
    memcpy(&stmI2cSendbuf[2], pd, datalen);
    return stm_I2C_SendBurst(EEPROM24LC_ADDR8, stmI2cSendbuf, datalen+2);
}

unsigned char stm24LC_ReadMem(unsigned short addr, unsigned char *pReadData, unsigned char datalen)
{
    unsigned char value, retlen=0;
	//mem addr
    stmI2cSendbuf[0] = (unsigned char)((addr & 0xff00) >> 8); //high
    stmI2cSendbuf[1] = (unsigned char)(addr & 0xff);		  //low

    //send mem addr of 2 bytes
	if(!stm_I2C_SendBurst(EEPROM24LC_ADDR8, stmI2cSendbuf, 2))
		return 0;
	//receive data
	retlen = stm_I2C_ReceiveBurst(EEPROM24LC_ADDR8, pReadData, datalen);
	if(retlen==0){
		printf("i2c Rd error\r\n");
		return 0;
	}
    return retlen;
}

void stm24LC_Loop() {
	unsigned char rdval;
	int i;
	unsigned char testData[] = {0x12,0x34, 0x56, 0x78};
	unsigned char rdData[4];
	while(1){
		printf("stm24LC>Write=0x12 at 0x00\r\n");
		stm24LC_WriteMem(0x01, testData, 4);
		delayms(10);
		stm24LC_ReadMem(0x01, rdData, 4);
		printf("stm24LC>ReadValue=");
		for(i=0;i<4;i++)
			printf("0x%02x ",rdData[i]);
		printf("\r\n");
		delayms(1000);
	}
}

