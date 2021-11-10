#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#else
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "lwip/include/stm32f4x7_eth.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "lwipopts.h"
#include "lwip/include/mem.h"
#endif

#define PCF8574ADDR8 	0x40 //8-bit addr// 0x20/21 == 7bit Addr
#define I2C_400KHZ	1 // 0 to use default 100Khz, 1 for 400Khz

extern void delayms(uint32_t ms);
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;

#define PCF8574_OK          0x00
#define PCF8574_PIN_ERROR   0x81
#define PCF8574_I2C_ERROR   0x82

uint8_t stmPcf8574_read8(unsigned char *buf)
{
	if(!stm_I2C_ReceiveBurst(PCF8574ADDR8, buf, 1))
		return PCF8574_I2C_ERROR;
	else
        return PCF8574_OK;
}
void stmPcf8574_write8(const uint8_t value)
{
	unsigned char buf;
	buf = value;
	if(!stm_I2C_SendBurst(PCF8574ADDR8, &buf, 1))
		return PCF8574_I2C_ERROR;
	else
		return PCF8574_OK;
}


extern unsigned char g_bI2CModuleConfigDone;

void stmPcf8574I2cLoop (void)
{
	u8 i;

	printf("PCF8574 IO Expander with I2C (8bit I2C addr = 0x%02x).\r\n",PCF8574ADDR8);

	if(!g_bI2CModuleConfigDone)
	{
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
		  gI2Cx = I2C2;
#else
		  gI2Cx = I2C1;
#endif
		printf("I2C Init...");
		stm_I2C_Init(gI2Cx,100000);//100Kbps Only
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	}

	i=0;
    while(1) {
    	stmPcf8574_write8(i);
        printf("Write= 0x%02x\r\n",i);
        i++;
        delayms(500);                           //Delay 500ms
    }
}
