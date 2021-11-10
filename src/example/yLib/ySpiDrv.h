#ifndef __YSPIDRV_H
#define __YSPIDRV_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "yInc.h"
#define nCS0 0
#define nCS1 1
#define nCS2 2

//========= THE VERY FIRST CONFIG for Various Chip Select Pins =================================
#if (PROCESSOR == PROCESSOR_STM32F103C8T6)|| (PROCESSOR == PROCESSOR_STM32F103RCT6)
	 //103-KONG/LEAN
	#define nCS0_H      {GPIO_SetBits(GPIOB, GPIO_Pin_12);}
	#define nCS0_L      {GPIO_ResetBits(GPIOB, GPIO_Pin_12);}
	#define nCS1_H      {GPIO_SetBits(GPIOB, GPIO_Pin_5);}//{GPIO_SetBits(GPIOB, GPIO_Pin_11);}
	#define nCS1_L      {GPIO_ResetBits(GPIOB, GPIO_Pin_5);}//{GPIO_ResetBits(GPIOB, GPIO_Pin_11);}
	#define nCS2_H      {GPIO_SetBits(GPIOA, GPIO_Pin_7);}
	#define nCS2_L      {GPIO_ResetBits(GPIOA, GPIO_Pin_7);}
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
	#define BOARD_REV 66
	#if (BOARD_REV == 66) //PA15
	#define nCS0_H      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
	#define nCS0_L      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
	#else //M43
 //use nCS0 of PD10
	#define nCS0_H      {GPIO_SetBits(GPIOD, GPIO_Pin_10);}
	#define nCS0_L      {GPIO_ResetBits(GPIOD, GPIO_Pin_10);}
	#endif

 	 #define nCS1_H      {GPIO_SetBits(GPIOD, GPIO_Pin_15);}
	#define nCS1_L      {GPIO_ResetBits(GPIOD, GPIO_Pin_15);}
	#define nCS2_H      {GPIO_SetBits(GPIOE, GPIO_Pin_5);}
	#define nCS2_L      {GPIO_ResetBits(GPIOE, GPIO_Pin_5);}
#elif (PROCESSOR == PROCESSOR_GD32F130FX)
	#define nCS0_H      {GPIO_SetBits(GPIOA, GPIO_Pin_4);}
	#define nCS0_L      {GPIO_ResetBits(GPIOA, GPIO_Pin_4);}
	#define nCS1_H      {GPIO_SetBits(GPIOA, GPIO_Pin_3);}
	#define nCS1_L      {GPIO_ResetBits(GPIOA, GPIO_Pin_3);}
	#define nCS2_H      {GPIO_SetBits(GPIOA, GPIO_Pin_2);}
	#define nCS2_L      {GPIO_ResetBits(GPIOA, GPIO_Pin_1);}

#elif (PROCESSOR == PROCESSOR_STM32F407VGT6)
//use nCS0 of PD10
#define nCS0_H      {GPIO_SetBits(GPIOD, GPIO_Pin_10);}
#define nCS0_L      {GPIO_ResetBits(GPIOD, GPIO_Pin_10);}

//use nCS1 of PD15
#define nCS1_H      {GPIO_SetBits(GPIOD, GPIO_Pin_15);}
#define nCS1_L      {GPIO_ResetBits(GPIOD, GPIO_Pin_15);}

//use nCS2 of PE5
#define nCS2_H      {GPIO_SetBits(GPIOE, GPIO_Pin_5);}
#define nCS2_L      {GPIO_ResetBits(GPIOE, GPIO_Pin_5);}

#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
//use nCS0 of PD10
#define nCS0_H      {GPIO_SetBits(GPIOD, GPIO_Pin_10);}
#define nCS0_L      {GPIO_ResetBits(GPIOD, GPIO_Pin_10);}

//use nCS1 of PD15
#define nCS1_H      {GPIO_SetBits(GPIOD, GPIO_Pin_15);}
#define nCS1_L      {GPIO_ResetBits(GPIOD, GPIO_Pin_15);}
#endif

#endif
