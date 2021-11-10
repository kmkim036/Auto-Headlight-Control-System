

#include "yInc.h"
#if ((PROCESSOR == STM32F103C8) || (PROCESSOR == STM32F107VCT6))
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

#define LM75ADDR8 	0x90 //8-bit addr// 0x48 == 7bit Addr
#define I2C_400KHZ	1	// 0 to use default 100Khz, 1 for 400Khz

extern void delayms(uint32_t ms);
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
extern char *float2str(float x);

void stmLm75TempSensorI2cLoop (void)
{
    int byte1;
    int byte2;
    int var;
    //float currTemp;
    unsigned short currTemp;
	u8 i;
	u8 rxbuf[20];
	u8 txbuf[2];

	printf("LM75 Temperature Driver Test with I2C (8bit I2C addr = 0x%02x).\r\n",LM75ADDR8);

	if(!g_bI2CModuleConfigDone){
#if (PROCESSOR == STM32F407VZT6)
		gI2Cx = I2C2;
#else
		gI2Cx = I2C1;
#endif

		printf("I2C Init with 100Kbps...");
		stm_I2C_Init(gI2Cx,100000);//100Kbps
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	}

	while(1) {
    	stm_I2C_ReceiveBurstWithRestartCondition(LM75ADDR8, 0x00, &rxbuf[0], 2); //0=Select temperature pointer register in sensor for Read Only
        byte1 = rxbuf[0];
        byte2 = rxbuf[1];

        var = (byte2 |= (byte1 << 8));          //Data-proccessing step 1 (Transform byte 1 and 2 to one integer)
        //currTemp = (0.125 * (var >> 5));      //Data-proccessing step 2 + multyplying with 0.125 to get an actual temperature
        currTemp = (var >> 5)/8;
        //currTemp = (var >> 5);

        printf("Temp= %d Degree/C\r\n",currTemp); 	//Need some processing to show the floating value.

        delayms(500);                           //Delay 500ms
    }
}
