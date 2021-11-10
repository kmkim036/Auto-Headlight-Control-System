#include "yInc.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT6) || (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"

u8 bIrq = 0; //Button IRQ
//+--------+-----------+-----------+-----------+---------+---------+-------------------------+----------+-----------+--------------+
//|        |401-M34    |401-M35    | 407-M35   | 407-M36 |103      |103LEAN                  | 103      | 103 BLDC  |103RCT/LCD    |103RCT/BGC
//+--------+-----------+-----------+-----------+---------+---------+-------------------------+----------+-----------+--------------+
//                                                       |M35,M37  | M78                     | M78      | M78       | M79          |M79
//                                                       |M39,~M70 |                         | Add32768 |           |
//+--------+-----------+-----------+-----------+---------+---------+-------------------------+----------+-----------+--------------+
//| ULED   | PB14      |PC4        |PE15       | <==     | PC14    |PC13                     | PC13     | PC13/PC14 |PC13          |PC5
//+--------+-----------+-----------+-----------+---------+---------+-------------------------+----------+-----------+--------------+
//| BUTTON |           |PC5(H)     |PD11(index)| PD11(L) | PC15    |PA15                     | PA15     | NC        |PA15          |PC3
//+--------+-----------+-----------+-----------+---------+---------+-------------------------+----------+-----------+------------+
//| BEEP   |           |PB13       |PD14       | <==     | PC13    |PC13(Shared with ULED)   | <-       |PB5        |PC12          |PC13
//+--------+-----------+-----------+-----------+---------+---------+-------------------------+----------+-----------+-------------+
//| QEI    |           |PB0,1,12   |PD12,13,11 | PD12,13 |                                                          |NC            |NC
//+--------+-----------+-----------+-----------+---------+---------+-------------------------+----------+-----------+--------------+
//|DGBUART |           |           |           |         |         |UART1(PA10/9)                                   |UART1(PA10/9) |UART3(PC11/10)
//+--------+-----------+-----------+-----------+---------+---------+-------------------------+----------+-----------+--------------+
//|I2C     |           |           |           |         |         |I2C1(PB7/6)                                     |I2C1 (PB7/6)  | I2C1 (PB7/6)
//+--------+-----------+-----------+-----------+---------+---------+-------------------------+----------+--------
//|                                                      |M70~main pinout changed. comply to M35.
//+--------+-----------+-----------+-----------+---------+---------+-------------------------+
//*H= Active High/L=Active Low
//TBD

//TEST
void stm103_portTest(){
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	//==ULED (PC14 or PC13)=====================================================================
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	while(1){
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	delayms(100);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	delayms(100);
	}
/*

	//PB8/9
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	//PB3
	//=======================================================================
	//VERY IMPORTANT] PB3's default function is JTDO. (and PB4's default function is JNTRST.)
	//Thus we shoul remap to GPIO as the follows.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	//=======================================================================
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3 Remap //< JTAG-DP Disabled and SW-DP Enabled

	while(1){
		GPIO_SetBits(GPIOB, GPIO_Pin_4);
		GPIO_SetBits(GPIOB, GPIO_Pin_3);
	delayms(100);
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);
	GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	delayms(100);
	}
*/
}
//+--------+-----------+-----------+-----------+---------+---------+-------
//|        |401-M34    |401-M35    | 407-M35   | 407-M36 |103(M35) |103LEAN
//+--------+-----------+-----------+-----------+---------+---------+--------
//| ULED   | PB14      |PC4        |PE15       | <==     | PC14    |PC13
//+--------+-----------+-----------+-----------+---------+---------+--------
void stmUser_LED_GPIO_setup(void){
	GPIO_InitTypeDef GPIO_InitStruct;
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)  || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))
	#if (PROCESSOR == PROCESSOR_STM32F103C8T6)
		#if 0// (MCU_MODULE_VER == 35) || (MCU_MODULE_VER >= 79)
		//PC14 - STM32F103-KONG
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
		GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_SetBits(GPIOC, GPIO_Pin_14); 		delayms(300); //GPIO_SetBits(GPIOC, GPIO_Pin_14); 		delayms(300);
		GPIO_ResetBits(GPIOC, GPIO_Pin_14);		delayms(300); //GPIO_ResetBits(GPIOC, GPIO_Pin_14);		delayms(300);
		GPIO_SetBits(GPIOC, GPIO_Pin_14);                     //GPIO_SetBits(GPIOC, GPIO_Pin_14);

		#else //(MCU_MODULE_VER == 78) or M84 //PC13 - STM32F103-LEAN
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
		GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delayms(100);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delayms(100);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		#endif
	#elif (PROCESSOR == PROCESSOR_STM32F103RCT6) //PC13

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
		GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delayms(300);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delayms(300);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
	//PE15 - KONG-STM32F107VCT
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_SetBits(GPIOE, GPIO_Pin_15);
	delayms(300);
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	delayms(300);
	GPIO_SetBits(GPIOE, GPIO_Pin_15);

	#elif (PROCESSOR == PROCESSOR_GD32F130FX) //PB1 - ULED
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_SetBits(GPIOB, GPIO_Pin_1);
	delayms(300);
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	delayms(300);
	GPIO_SetBits(GPIOB, GPIO_Pin_1);

	#endif
#elif (PROCESSOR == PROCESSOR_STM32F407VGT6)
	//PE15 - KONG-STM32F407
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //STM407VGT6
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); //ZGT6

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;// ZGT6
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
	delayms(100);
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	delayms(100);
	GPIO_SetBits(GPIOE, GPIO_Pin_15);

	//GPIO_ResetBits(GPIOG, GPIO_Pin_7);// ZGT6
#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //STM401-M34
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14; //STM401-M34
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //STM401-M35
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;//STM401-M35

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	//GPIO_Init(GPIOB, &GPIO_InitStruct);//STM401-M34
	//GPIO_ResetBits(GPIOB, GPIO_Pin_14); //STM401-M34
	GPIO_Init(GPIOC, &GPIO_InitStruct);//STM401-M35
	GPIO_SetBits(GPIOC, GPIO_Pin_4); //STM401-M35
	delayms(100);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4); //STM401-M35
	delayms(100);
	GPIO_SetBits(GPIOC, GPIO_Pin_4); //STM401-M35

#endif
}
//+--------+-----------+-----------+-----------+---------+---------+--------+
//|        |401-M34    |401-M35    | 407-M35   | 407-M36 |103KONG  |103LEAN |103RCT6
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| BUTTON |           |PC5(H)     |PD11(index)| PD11(L) |PC15     |PA15    |PA15
//+--------+-----------+-----------+-----------+---------+---------+--------+
#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
//STM32F103 (Active High)
void stmButton_setup(void)
{
	  GPIO_InitTypeDef   GPIO_InitStruct;
	#if (MCU_MODULE_VER == 35)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;		  // we want to configure PD11
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD; 	  // we want it to be an input
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	  GPIO_Init(GPIOC, &GPIO_InitStruct);
	#else
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;		  // we want to configure PD11
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD; 	  // we want it to be an input
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	  GPIO_Init(GPIOA, &GPIO_InitStruct);
	#endif
}
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
//PD11 - KONG-STM32F407-M36 (Active Low)
static void stmButtonPD11_setup(void)
{
	  GPIO_InitTypeDef   GPIO_InitStruct;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;		  // we want to configure PD11
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD; 	  // we want it to be an input
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	  GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void stmButtonPD11_IRQ_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;
/*
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //???
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11);

	  // Configure EXTI Line11
	  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);


	  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
*/
}

/*
void EXTI15_10_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line11) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line11);		// Clear the EXTI line 8 pending bit
		// Toggle LED0
		GPIO_ToggleBits(GPIOC, GPIO_Pin_4); // ULED ON (PC4)
		//irq = 1;
	}
}
*/
//PB15 - KONG-STM32F407-M35 (Active High)
static void stmButtonPB15_setup(void)
{
	  GPIO_InitTypeDef   GPIO_InitStruct;
/*
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;		  // we want to configure PD11
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 	  // we want it to be an input
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
	  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;   // this enables the pulldown resistor --> we want to detect a high level
	  GPIO_Init(GPIOB, &GPIO_InitStruct);
*/
}

void stmButtonPB15_IRQ_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;
/*
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //???
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  // Connect EXTI Line11 to PB15 pin
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource15);

	  // Configure EXTI Line11
	  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//_Falling; //EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);


	  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
*/
}
void stmUser_Button_GPIO_setup(void){
	stmButtonPD11_setup();
	//stmButtonPD11_IRQ_Setup();
}
//PD11
unsigned char  stmUserButton_Read(){
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11)){
		return 1;
	}
	else
		return 0;
}
/*
//PB15Very
 *
void EXTI15_10_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line15) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line15);		// Clear the EXTI line 15 pending bit
		// Toggle LED0
		GPIO_ToggleBits(GPIOC, GPIO_Pin_4); // ULED ON (PC4)
		//g_Ap3216_Module.irq = 1;
	}
}
*/
#endif
/*
//Interrupt enabled GPInput.(with EXTI)
//It is used for PE0/CAM_D2. (for KSZ and LAN9355 Switch Module)
void GP3_PE0_setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;


	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  // Configure PE0 pin as input floating
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	  GPIO_Init(GPIOE, &GPIO_InitStructure);

	  // Connect EXTI Line0 to PE0 pin

	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);

	  // Configure EXTI Line0
	  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  // Enable and set EXTI Line0 Interrupt to the lowest priority
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
*/

#if (PROCESSOR == PROCESSOR_STM32F103C8T6)
//PC13 - STM103C8T6
void stmBeepPC13_setup(void){
//...
  GPIO_InitTypeDef   GPIO_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_13;//GPIO_Pin_All ;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
}
void stmBeep_setup(void){
	stmBeepPC13_setup();
}
void stmBeepPC13(u32 delay){
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	delayms(delay);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void stmBeep(u32 delay){
	stmBeepPC13(delay);
}

#elif  (PROCESSOR == PROCESSOR_STM32F103RCT6)
//PC12 - STM103RCT6
void stmBeep_setup(void){
//...
  GPIO_InitTypeDef   GPIO_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void stmBeep(u32 delay){
	GPIO_SetBits(GPIOC, GPIO_Pin_12);
	delayms(delay);
	GPIO_ResetBits(GPIOC, GPIO_Pin_12);
}
#else
void stmBeepPD14_setup(void)
{
	  GPIO_InitTypeDef   GPIO_InitStruct;

	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;		  // we want to configure PD11
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_Mode_OUT; 	  // we want it to be an input
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	  //GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
	  //GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;   // this enables the pulldown resistor --> we want to detect a high level
	  GPIO_Init(GPIOD, &GPIO_InitStruct);
}
void stmBeepPC4(u32 delay){
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	delayms(delay);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);
}
void stmBeepPD14(u32 delay){
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
	delayms(delay);
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}
#endif

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
void stmConfirmLEDBlink(void){

	GPIO_ResetBits(GPIOC, GPIO_Pin_14);// LED OFF
	delayms(100);
	GPIO_SetBits(GPIOC, GPIO_Pin_14); // LED ON
	delayms(100);
}
void stmUserLED_ON(){
	GPIO_SetBits(GPIOC, GPIO_Pin_14); 	// LED1 ON
}
void stmUserLED_OFF(){
	GPIO_ResetBits(GPIOC, GPIO_Pin_14); 	// LED1 OFF
}
void stmLedToggle(void){
	GPIO_ToggleBits(GPIOC, GPIO_Pin_14); //ULED -103
}
#elif(PROCESSOR == PROCESSOR_STM32F401RET6)
//PC14
void stmConfirmLEDBlink(void){

	GPIO_ResetBits(GPIOC, GPIO_Pin_14);// LED OFF
	delayms(100);
	GPIO_SetBits(GPIOC, GPIO_Pin_14); // LED ON
	delayms(100);
}
void stmUserLED_ON(){
	GPIO_SetBits(GPIOC, GPIO_Pin_14); 	// STM401. LED1 ON
}
void stmUserLED_OFF(){
	GPIO_ResetBits(GPIOC, GPIO_Pin_14); 	// STM401. LED1 ON
}
#else
//PE15
void stmConfirmLEDBlink(void){

	GPIO_ResetBits(GPIOE, GPIO_Pin_15);// LED OFF
	//GPIO_SetBits(GPIOB, GPIO_Pin_4); // BUZZER ON
	delayms(200);

	GPIO_SetBits(GPIOE, GPIO_Pin_15); // LED ON
	//GPIO_ResetBits(GPIOB, GPIO_Pin_4);// BUZZER OFF
	delayms(200);
}
void stmUserLED_ON(){
	GPIO_SetBits(GPIOE, GPIO_Pin_15); 	// STM401. LED1 ON
}
void stmUserLED_OFF(){
	GPIO_ResetBits(GPIOE, GPIO_Pin_15); 	// STM401. LED1 ON
}
#endif

void stmBeepLoop(void){
	stmBeepPD14_setup();
	while(1){
		stmBeepPD14(100);
		delayms(100);
	}
}
void stmBlinkyVeryFirstLoop(void)
{
	int i;

	stmUser_LED_GPIO_setup();

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
	//PC14 - KONG-STM32F103 ULED

	while(1){

			GPIO_SetBits(GPIOC, GPIO_Pin_13); 	//LED ON --STM103   //or//GPIO_WriteBit(LED_PORT,GPIO_Pin_14,Bit_SET);
			printf("ON ");
			delayms(2000);
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);// LED off -- STM103 //or//GPIO_WriteBit(LED_PORT,GPIO_Pin_14,Bit_RESET);
			printf("OFF ");
			delayms(2000);

			//Button
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)){
				for(i=0;i<5;i++){
					GPIO_SetBits(GPIOC, GPIO_Pin_14); 	// LED1 ON
					delayms(500);
					GPIO_ResetBits(GPIOC, GPIO_Pin_14);// LED off
					delayms(500);
				}
			}
	}
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
	//PE15 - KONG-STM32F107 ULED

	while(1){
			GPIO_SetBits(GPIOE, GPIO_Pin_15); 	//LED ON --STM103   //or//GPIO_WriteBit(LED_PORT,GPIO_Pin_14,Bit_SET);
			printf("ON ");
			delayms(1000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_15);// LED off -- STM103 //or//GPIO_WriteBit(LED_PORT,GPIO_Pin_14,Bit_RESET);
			printf("OFF ");
			delayms(1000);
/*
			//Button
			if(GPIO_ReadInputDataBit(GPIO,GPIO_Pin_15)){
				for(i=0;i<5;i++){
					GPIO_SetBits(GPIOC, GPIO_Pin_14); 	// LED1 ON
					delayms(500);
					GPIO_ResetBits(GPIOC, GPIO_Pin_14);// LED off
					delayms(500);
				}
			}
*/
	}
#elif (PROCESSOR == PROCESSOR_GD32F130FX)
	//PE15 - KONG-STM32F107 ULED

	while(1){
			GPIO_SetBits(GPIOB, GPIO_Pin_1);
			//printf("ON ");
			delayms(1000);
			GPIO_ResetBits(GPIOB, GPIO_Pin_1);
			//printf("OFF ");
			delayms(1000);
/*
			//Button
			if(GPIO_ReadInputDataBit(GPIO,GPIO_Pin_15)){
				for(i=0;i<5;i++){
					GPIO_SetBits(GPIOC, GPIO_Pin_14); 	// LED1 ON
					delayms(500);
					GPIO_ResetBits(GPIOC, GPIO_Pin_14);// LED off
					delayms(500);
				}
			}
*/
	}
#else

	stmBeepLoop();
#endif

}

void stmBlinkyAndBeepLoop(void)
{
	int i;

	stmUser_LED_GPIO_setup();//PC14 (or PC13) - KONG-STM32F103 ULED

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
	stmBeepPC13_setup();
#else
	stmBeepPC14_setup();
#endif
	stmButton_setup();	//stmButtonPB15_IRQ_Setup();

	while(1){
		#if(MCU_MODULE_VER == 78)
			GPIO_SetBits(GPIOC, GPIO_Pin_13); 	//LED ON --STM103   M78 PC13
			delayms(500);
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);// LED off -- STM103 M78 PC13
			delayms(500);
		#else
			GPIO_SetBits(GPIOC, GPIO_Pin_14); 	//LED ON --STM103   //or//GPIO_WriteBit(LED_PORT,GPIO_Pin_14,Bit_SET);
			delayms(500);
			GPIO_ResetBits(GPIOC, GPIO_Pin_14);// LED off -- STM103 //or//GPIO_WriteBit(LED_PORT,GPIO_Pin_14,Bit_RESET);
			delayms(500);

		#endif
			//Button
/*			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)){
				for(i=0;i<5;i++){
					GPIO_SetBits(GPIOC, GPIO_Pin_14); 	// LED1 ON
					delayms(200);
					GPIO_ResetBits(GPIOC, GPIO_Pin_14);// LED off

					stmBeepPC13(100);//Beep
					delayms(100);
				}
			}
*/
	}
}

void stmRelayControlLoop(void)
{
	int i;
	GPIO_InitTypeDef   GPIO_InitStruct;

	stmUser_LED_GPIO_setup(); //PC14

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
	//Relay ON/OFF - PB0(Pin 5)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	while(1){
			printf("ON  ");
			GPIO_SetBits(GPIOB, GPIO_Pin_0);
			delayms(1000);
			GPIO_ResetBits(GPIOB, GPIO_Pin_0);
			printf("OFF  ");
			delayms(1000);
	}

#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
	//PB15 - KONG-STM32F407 -M35
	stmButtonPB15_setup();
	stmBeepPD14_setup();
	//stmButtonPB15_IRQ_Setup();

	while(1){

		for(i=0;i<10;i++){
			printf("ON  ");
			//GPIO_SetBits(GPIOB, GPIO_Pin_15); // LED1 ON
			GPIO_SetBits(GPIOE, GPIO_Pin_15); 	// STM401. LED1 ON
			//GPIO_SetBits(GPIOE, GPIO_Pin_5); // LED1 ON
			//GPIO_SetBits(GPIOG, GPIO_Pin_7); // LED1 ON -- STM32F407ZGT6
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_15);// LED off -- STM401
			//GPIO_ResetBits(GPIOB, GPIO_Pin_15);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//GPIO_ResetBits(GPIOE, GPIO_Pin_5);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//GPIO_ResetBits(GPIOG, GPIO_Pin_7);// LED1 OFF -- STM32F407ZGT6/LED off
			printf("OFF  ");
			//GPIO_ResetBits(GPIOB, GPIO_Pin_4);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			delayms(1000);//somedelay(1000000);
		}
		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11)){
		//if(bIrq){
			GPIO_SetBits(GPIOE, GPIO_Pin_15); 	// STM401. LED1 ON
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_15);// LED off -- STM401
			bIrq = 0;
		}
	}
#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
	//PB14 - M34
	//PC4  - M35
	while(1){
			printf("ON  ");
			GPIO_SetBits(GPIOC, GPIO_Pin_4); // LED1 ON
			//GPIO_SetBits(GPIOB, GPIO_Pin_14); 	// STM401. LED1 ON
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOC, GPIO_Pin_4);// LED off -- STM401
			//GPIO_ResetBits(GPIOB, GPIO_Pin_14);// LED off -- STM401
			printf("OFF  ");
			delayms(1000);//somedelay(1000000);
	}
#endif
}

void stmDualSolenoidControlLoop(void)
{
	int i;
	GPIO_InitTypeDef   GPIO_InitStruct;

	stmUser_LED_GPIO_setup(); //PC14

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
#if (MCU_MODULE_VER  ==	35) || (MCU_MODULE_VER  ==	70)
	//Solenoid ON/OFF - PB3(Pin 6)/PB4(Pin 7)

	//========================== PB3 and PB4 Specific =========================
	//[VERY IMPORTANT] PB3's default function is JTDO and PB4's default function is JNTRST.
	//Thus we should remap it to general GPIO pin as the follows if needed.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3/PB4 Remap to GPIO
    //==========================================================================

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	while(1){
			printf("ON  ");
			GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4);
			delayms(1000);
			GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4);
			printf("OFF  ");
			delayms(1000);
	}
#else //M37, M..
	//Solenoid ON/OFF - PB1(Pin 6)/PA4(Pin 7)

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	while(1){
			printf("ON  ");
			GPIO_SetBits(GPIOB, GPIO_Pin_1);
			GPIO_SetBits(GPIOA, GPIO_Pin_4);
			delayms(1000);
			GPIO_ResetBits(GPIOB, GPIO_Pin_1);
			GPIO_ResetBits(GPIOA, GPIO_Pin_4);
			printf("OFF  ");
			delayms(1000);
	}

#endif
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
	//PB15 - KONG-STM32F407 -M35
	stmButtonPB15_setup();
	stmBeepPD14_setup();
	//stmButtonPB15_IRQ_Setup();

	while(1){

		for(i=0;i<10;i++){
			printf("ON  ");
			//GPIO_SetBits(GPIOB, GPIO_Pin_15); // LED1 ON
			GPIO_SetBits(GPIOE, GPIO_Pin_15); 	// STM401. LED1 ON
			//GPIO_SetBits(GPIOE, GPIO_Pin_5); // LED1 ON
			//GPIO_SetBits(GPIOG, GPIO_Pin_7); // LED1 ON -- STM32F407ZGT6
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_15);// LED off -- STM401
			//GPIO_ResetBits(GPIOB, GPIO_Pin_15);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//GPIO_ResetBits(GPIOE, GPIO_Pin_5);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//GPIO_ResetBits(GPIOG, GPIO_Pin_7);// LED1 OFF -- STM32F407ZGT6/LED off
			printf("OFF  ");
			//GPIO_ResetBits(GPIOB, GPIO_Pin_4);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			delayms(1000);//somedelay(1000000);
		}
		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11)){
		//if(bIrq){
			GPIO_SetBits(GPIOE, GPIO_Pin_15); 	// STM401. LED1 ON
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_15);// LED off -- STM401
			bIrq = 0;
		}
	}
#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
	//PB14 - M34
	//PC4  - M35
	while(1){
			printf("ON  ");
			GPIO_SetBits(GPIOC, GPIO_Pin_4); // LED1 ON
			//GPIO_SetBits(GPIOB, GPIO_Pin_14); 	// STM401. LED1 ON
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOC, GPIO_Pin_4);// LED off -- STM401
			//GPIO_ResetBits(GPIOB, GPIO_Pin_14);// LED off -- STM401
			printf("OFF  ");
			delayms(1000);//somedelay(1000000);
	}
#endif
}
void stmBlinkyAndPrintfLoop(void)
{
	int i;

	stmUser_LED_GPIO_setup();

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
	stmBeepPC13_setup();
	//PC14 - KONG-STM32F103 ULED
	//stmButton_setup();	//stmButtonPB15_IRQ_Setup();

	while(1){
			printf("ON  ");
			//GPIO_SetBits(GPIOC, GPIO_Pin_13);
			GPIO_SetBits(GPIOC, GPIO_Pin_14); 	//LED ON --STM103   //or//GPIO_WriteBit(LED_PORT,GPIO_Pin_14,Bit_SET);
			delayms(1000);
			//GPIO_ResetBits(GPIOC, GPIO_Pin_13);//
			GPIO_ResetBits(GPIOC, GPIO_Pin_14);// LED off -- STM103 //or//GPIO_WriteBit(LED_PORT,GPIO_Pin_14,Bit_RESET);
			printf("OFF  ");
			delayms(1000);
/*		//Button
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)){
			printf("fast blinking...\r\n");
			for(i=0;i<5;i++){
				GPIO_SetBits(GPIOC, GPIO_Pin_14); 	// LED1 ON
				delayms(200);
				GPIO_ResetBits(GPIOC, GPIO_Pin_14);// LED off
				delayms(200);
				stmBeepPC13(100);//Beep
			}
		}
*/
	}
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
	//PB15 - KONG-STM32F407 -M35
	stmButtonPB15_setup();
	stmBeepPD14_setup();
	//stmButtonPB15_IRQ_Setup();

	while(1){

		for(i=0;i<10;i++){
			printf("ON  ");
			//GPIO_SetBits(GPIOB, GPIO_Pin_15); // LED1 ON
			GPIO_SetBits(GPIOE, GPIO_Pin_15); 	// STM401. LED1 ON
			//GPIO_SetBits(GPIOE, GPIO_Pin_5); // LED1 ON
			//GPIO_SetBits(GPIOG, GPIO_Pin_7); // LED1 ON -- STM32F407ZGT6
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_15);// LED off -- STM401
			//GPIO_ResetBits(GPIOB, GPIO_Pin_15);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//GPIO_ResetBits(GPIOE, GPIO_Pin_5);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//GPIO_ResetBits(GPIOG, GPIO_Pin_7);// LED1 OFF -- STM32F407ZGT6/LED off
			printf("OFF  ");
			//GPIO_ResetBits(GPIOB, GPIO_Pin_4);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			delayms(1000);//somedelay(1000000);
		}
		if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11)){
		//if(bIrq){
			GPIO_SetBits(GPIOE, GPIO_Pin_15); 	// STM401. LED1 ON
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_15);// LED off -- STM401
			bIrq = 0;
		}
	}
#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
	//PB14 - M34
	//PC4  - M35
	while(1){
			printf("ON  ");
			GPIO_SetBits(GPIOC, GPIO_Pin_4); // LED1 ON
			//GPIO_SetBits(GPIOB, GPIO_Pin_14); 	// STM401. LED1 ON
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOC, GPIO_Pin_4);// LED off -- STM401
			//GPIO_ResetBits(GPIOB, GPIO_Pin_14);// LED off -- STM401
			printf("OFF  ");
			delayms(1000);//somedelay(1000000);
	}

#elif (PROCESSOR == PROCESSOR_GD32F130FX)
	//PB1
	while(1){
			//printf("ON  ");
			GPIO_SetBits(GPIOB, GPIO_Pin_1);
			delayms(1000);
			GPIO_ResetBits(GPIOB, GPIO_Pin_1);
			//printf("OFF  ");
			delayms(1000);
	}
#endif
}

