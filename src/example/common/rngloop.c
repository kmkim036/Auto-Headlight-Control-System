/*
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

extern void TM_RNG_Init(void);
extern void TM_RNG_DeInit(void);
extern uint32_t TM_RNG_Get(void);
extern uint32_t rngConfigAndGetRand(void);

void rngLoop(void){
	TM_RNG_Init();

	while(1){
		printf("%d\r\n",TM_RNG_Get());
		delayms(1000);
	}

	TM_RNG_DeInit();
}

uint32_t rngConfigAndGetRand(void){
	TM_RNG_Init();
	return TM_RNG_Get();
}
