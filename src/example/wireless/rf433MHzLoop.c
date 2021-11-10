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
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"

#define LED0 GPIOB, GPIO_Pin_15
#define LED2 GPIOD, GPIO_Pin_13
#define LED3 GPIOD, GPIO_Pin_14
#define LED4 GPIOD, GPIO_Pin_12
#define button GPIOA, GPIO_Pin_0
int a, b, c, d ;

// RF433MHz : ASK
// TX : 433.92MHz, 3~12V
// RX : 5V

//TXD - PC8
void stmRF433tx_GPIO_setup(void){
	GPIO_InitTypeDef GPIO_InitStruct;
#if ((PROCESSOR == STM32F103C8) || (PROCESSOR == STM32F107VCT6))
#else
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //STM407VGT6
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); //ZGT6
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;// | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12; // we want to configure all LED GPIO pins
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14; //STM401
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;// ZGT6
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;// | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12; // we want to configure all LED GPIO pins
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	//GPIO_Init(GPIOB, &GPIO_InitStruct);//STM401

	GPIO_Init(GPIOC, &GPIO_InitStruct);
	//GPIO_Init(GPIOG, &GPIO_InitStruct);// ZGT6
#endif
	//GPIO_ResetBits(GPIOB, GPIO_Pin_14); //STM401
	GPIO_ResetBits(GPIOC, GPIO_Pin_6);
	//GPIO_ResetBits(GPIOG, GPIO_Pin_7);// ZGT6
}
//Interrupt enabled GPInput.(with EXTI)
//It is used for PE0/CAM_D2. (for KSZ and LAN9355 Switch Module)
void stmRF433rxGP3_PE0_setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;
#if ((PROCESSOR == STM32F103C8) || (PROCESSOR == STM32F107VCT6))
#else
	  /* Enable GPIOE clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Configure PE0 pin as input floating */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	  GPIO_Init(GPIOE, &GPIO_InitStructure);

	  /* Connect EXTI Line0 to PE0 pin */

	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
#endif
	  /* Configure EXTI Line0 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}




extern void stmUser_LED_GPIO_setup(void);
extern void stmBlinkOn();
extern void stmBlinkOff();
//Max 10KHz, 20Kbps...
void stmRF433txLoop(void)
{

	stmUser_LED_GPIO_setup();

	stmRF433tx_GPIO_setup();
	//button_setup();
	//GPIO_ResetBits(GPIOB, GPIO_Pin_4);//GPIO_Pin_12 | GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 );

	//PE15 - KONG
	while(1){
			//printf("ON  ");
			stmBlinkOn();
			//GPIO_SetBits(GPIOB, GPIO_Pin_15); // LED1 ON
			GPIO_SetBits(GPIOC, GPIO_Pin_6); 	// STM401. LED1 ON
			//GPIO_SetBits(GPIOE, GPIO_Pin_5); // LED1 ON
			//GPIO_SetBits(GPIOG, GPIO_Pin_7); // LED1 ON -- STM32F407ZGT6
			//delayms(1);//
			somedelay(800);
			stmBlinkOff();
			GPIO_ResetBits(GPIOC, GPIO_Pin_6);// LED off -- STM401
			//GPIO_ResetBits(GPIOB, GPIO_Pin_15);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//GPIO_ResetBits(GPIOE, GPIO_Pin_5);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//GPIO_ResetBits(GPIOG, GPIO_Pin_7);// LED1 OFF -- STM32F407ZGT6/LED off
			//printf("OFF  ");
			//GPIO_ResetBits(GPIOB, GPIO_Pin_4);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//delayms(1);//somedelay(1000000);
			somedelay(800);
	}



}
