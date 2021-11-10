//-- MCP8063 BLDC

//STM32F103RCT
	//[MOT0]
	// PC8 : PWM
	// PCX : FGout

#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"

#include "math.h"
#include "init.h"

#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) ||(PROCESSOR == PROCESSOR_STM32F107VCT))

#define VMOT 		12 //12V VMOT
#define PWM_PERIOD 	2100
//PC8 : TIM8-CH3
void TIM8_ini(void)
{
	TIM_TimeBaseInitTypeDef timer_init;
	TIM_OCInitTypeDef tim_oc_init3;

	TIM_DeInit(TIM8);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Period = PWM_PERIOD - 1;// ---> 20kHz
	timer_init.TIM_Prescaler = 2-1; // --->36MHz// 42 MHz
	timer_init.TIM_ClockDivision = 0;
	timer_init.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &timer_init);

	//channel 3;

	TIM_OCStructInit(&tim_oc_init3);
	tim_oc_init3.TIM_Pulse = 500; //speed
	tim_oc_init3.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init3.TIM_OCPolarity = TIM_OCPolarity_High;
	tim_oc_init3.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM8, &tim_oc_init3);//

	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM8, ENABLE);

	TIM_Cmd(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
}

void MCP8063_BLDC_PWM_PINx_init(void) // INx, IN - PWMpins
{
	GPIO_InitTypeDef init_AF;//

	//(1) PC8 - TIM8
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF;//
	init_AF.GPIO_Speed = GPIO_Speed_10MHz;
	init_AF.GPIO_Pin = GPIO_Pin_8;//
    GPIO_Init(GPIOC, &init_AF);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
}
//======= main
//-- STM32F103RCT6

void stmMcp8063BldcLoop(void)
{
	MCP8063_BLDC_PWM_PINx_init();
	 TIM8_ini(); // PWM timer
	 while(1)
	 {
		 	TIM3->CCR3 = (unsigned short)(1*PWM_PERIOD/VMOT)  ;
 	 }
}

#endif
