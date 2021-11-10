/*
  QEI
	Quadrature Encoder Example.
	Using Timer peripheral, we can read incremental encoder.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "yInc.h"
#include <time.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "core_cm3.h"    //for TIM3
#include "core_cmFunc.h" //for TIM3

#include "misc.h"

//PhA=PB4(TIM3_CH1); PhB=PB5(TIM3_CH2); Index=PXX : M83.. Need PartialRemap
//PhA=PA0(TIM2_CH1); PhB=PA1(TIM2_CH2); Index=PXX : M82
//PhA=PD12(TIM4_CH1); PhB=PD13(TIM4_CH2); Index=PB3 : RCT6...Need PartialRemap

// (NOTE) The TIM Interrupt will issue on Overflow, not for each pulse... Thus it is useless...

#define USE_QEI_INDEX 0

struct qei_table g_qei_table;

#if USE_QEI_INDEX
#if (USE_EXTI3 == USE_EXTI3_QEI_IDX)
//PB3 for Index Input (active Low) -- Need PinRemap
void EXTI3_IRQHandler(void) //Index
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        TIM_SetCounter(TIM3, 0);            // reset encoder counter
        printf("SW!\r\n");

        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}
#endif

//PB3 with remapping
void stm_Qei_Index_Config(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

     //PB3: index line interrupt configuration
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;//
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //GPIO_Speed_50MHz;
     GPIO_Init(GPIOB, &GPIO_InitStructure);
     //Pin remapping for PB3( and PB4) to use GPIO.
 	 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3/4 Remap !< JTAG-DP Disabled and SW-DP Enabled

     NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);

     EXTI_InitStructure.EXTI_Line = EXTI_Line3;
     EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //EXTI_Trigger_Falling;
     EXTI_InitStructure.EXTI_LineCmd = ENABLE;
     EXTI_Init(&EXTI_InitStructure);

     GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3); //Connect EXTI Line3 to PB3 pin
}
#endif

//=====================================

void stm_Qei_Config(uint32_t maxValue){
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    printf("\r\nQuadrature Encoder Test(X1)\r\n");

    g_qei_table.direction =0;
    g_qei_table.oldVal = 0;
    g_qei_table.val = 0;

#if USE_QEI_INDEX
    //Index pin config (optional)
    ////PB3 with remapping
    stm_Qei_Index_Config();
#endif

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_GD32F130FX)

    //a) GPIO Setup for PB4/5
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5; //TIM3_CH1 and TIM3_CH2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //Pin Remap with GPIO_PartialRemap_TIM3      : TIM3 Partial Alternate Function mapping
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);  //Remap for TIM3 : for PB4* and PB5

    //b) Timer3 setup
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
#if 0 //X4
    //config TIM3 to operate in Encoder Mode with Combined Channels (TT12) [encoder mode3 : X4]
    TIM_EncoderInterfaceConfig(
    		TIM3,
    		TIM_EncoderMode_TI12, 	//Counter counts on both TI1FP1 and TI2FP2 edges (TT12)
    		TIM_ICPolarity_Falling, //TIM_ICPolarity_Rising,  //specifies the IC1 Polarity
    		TIM_ICPolarity_Rising);//TIM_ICPolarity_Falling);//Rising); //specifies the IC2 Polarity
#else //X1
    TIM_EncoderInterfaceConfig(
    		TIM3,
    		TIM_EncoderMode_TI1, 	//Counter counts on TI1FP1 edge only
    		TIM_ICPolarity_Rising,  //specifies the IC1 Polarity
    		TIM_ICPolarity_Rising); //specifies the IC2 Polarity (dummy for this single edge config)
#endif

#if 0
    //Set Filter
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter = 15;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1; //Capture performed each time an edge detected
    //TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM3,&TIM_ICInitStruct);

    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;

    TIM_ICInit(TIM3,&TIM_ICInitStruct);
#endif
    TIM_SetAutoreload (TIM3,maxValue);// Sets the TIMx Autoreload Register(ARR) value

    __disable_irq();
    TIM_SetCounter(TIM3,maxValue >> 1); //Set initial value (max divide by 2)
    __enable_irq();

    TIM_Cmd(TIM3, ENABLE);//Turn on the timer/counter

#elif ((PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))

    printf("\r\nQuadrature Encoder Test(X2 or X4)\r\n");

    //Configure Times as QEI Counter.
    //TIM4, PD12/13
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPD;//GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //for PD12/PD13

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Rising); 		//OK
    TIM_SetAutoreload (TIM4,maxValue);// 0xffff);
    TIM_Cmd(TIM4, ENABLE);//Turn on the timer/counter

    __disable_irq();
    TIM_SetCounter(TIM4,maxValue>>1);//0);
    __enable_irq();
#else

#endif
}

unsigned short stm_Qei_GetValue(void)
{
	unsigned short retvals;
	retvals = TIM_GetCounter(TIM3);
	return retvals;
}
int stm_Qei_GetDirection(void)
{
	//TIM_CR1_DIR == 0(UP) 1==(DOWN)
	return (TIM3->CR1 & TIM_CR1_DIR) ? -1: 1;
}
void stmQeiLoop(void)
{
	unsigned maxValue = 100;//1023; //100
	char str[80];

    stm_Qei_Config(maxValue);

    while(1)
    {
    	g_qei_table.val = stm_Qei_GetValue();
    	if(g_qei_table.oldVal != g_qei_table.val){
    		g_qei_table.oldVal = g_qei_table.val;
    		g_qei_table.direction = stm_Qei_GetDirection();//(TIM3->CR1 & TIM_CR1_DIR) ? 1: -1;
    		printf("c=%d;d=%d\r\n", g_qei_table.val, g_qei_table.direction);
    		//printf("%s",str);
    		//sprintf(str,"%d", g_qei_table.val);
    		//stmOzOLED_printString(str,1,2,15);
    		//stmOzOLED_printBigNumber(str,1,2,4);
    		//SSD1306_OLED_printNumber_11X16(str,0,0,8);
    	}
    }
}



