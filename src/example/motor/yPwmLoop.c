/*
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
#include "yInc.h"
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"

#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

static uint32_t speed = 10;

//PE9 = PWM -- TBC
void PWM_init(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_DeInit(TIM1);
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#else
	// Start Timer 1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	//Enable port E clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// Configure PE9 as output for PWM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Connect PE9 to Timer 1 channel 1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);

	TIM_TimeBaseStructure.TIM_Period = 716;
	TIM_TimeBaseStructure.TIM_Prescaler = 3906;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = speed;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
#endif
	/* TIM1 enable counter */
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

#define PWM_PERIOD 	101 //was 3

//PA3 PWMGen 1.629MHz
void stmPWM_Config(unsigned char duty){
	GPIO_InitTypeDef GPIO_InitStructure;//
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

#if (PROCESSOR == PROCESSOR_STM32F103RCT6)
    //(2) PA3 - TIM2-CH4
    //(2a) GPIO Init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);//
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
#endif

    //(2b) TIM init
   	//Freq = 36MHz/(Precaler+1)/(period-1)
   	//We need 1.629MHz/
   	TIM_DeInit(TIM2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

   	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
   	//1.6MHz --> (14,0,2,1) -- Duty=40% ?
   	//1.2MHz --> (14,0,3,2) -- Duty=50% (36/(14+1)/(3-1) = 1.2MHz)
   	//1.285MHz --> (13,0,3,2) -- Duty=50%
   	//1.385MHz --> (12,0,3,2) -- Duty=50%
   	//1.636MHz --> (10,0,3,2) -- Duty=50%
   	TIM_TimeBaseStructure.TIM_Prescaler 	= 9;//was 10; // ---> 36/(10+1)/(3-1) = 1.636MHz
   	TIM_TimeBaseStructure.TIM_ClockDivision = 100; //was 0;
   	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
   	TIM_TimeBaseStructure.TIM_Period 		= PWM_PERIOD;//(Auto-Reload Register at the next update event)
   	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

   	//channel 4;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    if(duty == 25)
    	TIM_OCInitStructure.TIM_Pulse  = 1; //25% = Pulse/(PERIOD+1)
    else if(duty == 50)
    	TIM_OCInitStructure.TIM_Pulse  = 2; //50% = Pulse/(PERIOD+1)
    else if(duty == 75)
    	TIM_OCInitStructure.TIM_Pulse  = 98; //75% = Pulse/(PERIOD+1)
    else if(duty == 100)
    	TIM_OCInitStructure.TIM_Pulse  = 4; //100% = Pulse/(PERIOD+1)
    else
    	TIM_OCInitStructure.TIM_Pulse  = 4;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);//Channel4

    //TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
    //TIM_ARRPreloadConfig(TIM2,ENABLE);

   	TIM_Cmd(TIM2, ENABLE);

   	//TIM_CtrlPWMOutputs(TIM2, ENABLE);
}
