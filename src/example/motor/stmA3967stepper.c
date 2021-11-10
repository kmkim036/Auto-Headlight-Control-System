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
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#include "yInc.h"
#if 0
#if (MOTOR_FOR == MOTOR_FOR_A3967)
/* Allegro's A3967SLBT can directly drive a Bipolar DC Stepper Motor (5V)// or 12V Version)
 * Logic = 3.3V
 * Driver = 5V~12V (H1 selectable)

   5V/12V 4 Pins Bipolar Stepper Motor
 * Org : Pin 2 : 1A
 * Yel : Pin 3 : 1B
 * Pnk : Pin 4 : 2A
 * Blu : Pin 5 : 2B
 *
 * GPIO[KONG] =====
 * STEP PA15
 * DIR  PC10
 * nEnable PC11
 * nRESET
 */

//+-----------------+-----------+-----------+-----------+---------+---------+
//|                 |401-M34    |401-M35    | 407-M35   | 407-M36 |103      |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| ULED            | PB14      |PC4        |PE15       | <==     |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| BUTTON          |           |PC5(H)     |PD11(index)| PD11(L) |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| BEEP            |           |PB13       |PD14       | <==     |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| QEI             |           |PB0,1,12   |PD12,13,11 | PD12,13 |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| STEP            |           |           |           | PA15    |PB1
//+-----------------+-----------+-----------+-----------+---------+---------+
//| DIR             |           |           |           | PC10    |PA5
//+-----------------+-----------+-----------+-----------+---------+---------+
//| nEN             |           |           |           | PC11    |PA4
//+-----------------+-----------+-----------+-----------+---------+---------+
//*H= Active High/L=Active Low
//*F= FallingEdge

//MS2 | MS1
#define FULLSTEP 0
#define HALFSTEP 1
#define QUARTERSTEP 2
#define OCTASTEP 3

#define FORWARD 1
#define BACKWARD 0
//#define MOTORSPEED 4000   // You can change speed... 4000 is the fastest.. 40000 can show step and LED
#define STEPSPERREV 210//640  	//# of steps per revolution (2 steps for 18 degree. --> 40 steps for 360 degree.)
 							// If we use octastep, the STEPSPERREV would be 8*80=640


#if (PROCESSOR == PROCESSOR_STM32F103C8T6)  || (PROCESSOR == PROCESSOR_STM32F103RCT6)

#define stmA3967_STEP_1        {GPIO_SetBits(GPIOB, GPIO_Pin_1);} //STEP
#define stmA3967_STEP_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_1);}

#define stmA3967_DIR_CW        {GPIO_SetBits(GPIOA, GPIO_Pin_5);} //DIR=1
#define stmA3967_DIR_CCW      {GPIO_ResetBits(GPIOA, GPIO_Pin_5);}

#define stmA3967_STOP        {GPIO_SetBits(GPIOA, GPIO_Pin_4);}
#define stmA3967_GO          {GPIO_ResetBits(GPIOA, GPIO_Pin_4);} //~EN
#else
#define stmA3967_STOP        {GPIO_SetBits(GPIOC, GPIO_Pin_11);}
#define stmA3967_GO          {GPIO_ResetBits(GPIOC, GPIO_Pin_11);} //~EN

#define stmA3967_DIR_CW        {GPIO_SetBits(GPIOC, GPIO_Pin_10);} //DIR=1
#define stmA3967_DIR_CCW      {GPIO_ResetBits(GPIOC, GPIO_Pin_10);}

#define stmA3967_STEP_1        {GPIO_SetBits(GPIOA, GPIO_Pin_15);} //STEP
#define stmA3967_STEP_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
#endif

void stmA3967Conf(){

	GPIO_InitTypeDef GPIO_InitStruct;
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
	//| STEP            |           |           |           | PA15    |PB1
	//+-----------------+-----------+-----------+-----------+---------+---------+
	//| DIR             |           |           |           | PC10    |PA5
	//+-----------------+-----------+-----------+-----------+---------+---------+
	//| nEN             |           |           |           | PC11    |PA4

	//PA5 and PA4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	//PB1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

#else
	//PC11 and PC10
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//PC11 and PC10
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

	//PA15
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //PA15
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);//
#endif
	printf("Init A3967 Stepper.\r\n");
	//dtUserLedCon0(1);
	delayms(1000);
	//dtUserLedCon0(0);
}



void stmA3967GoSteps (u8 dir, u32 nStep){
	u32 i;
	u16 highdur=5;
	u16 lowdur=2;

	//Set Dir
	if(dir){
		printf("Clockwise...\r\n");
		stmA3967_DIR_CW
	}else{
		printf("CountClockwise...\r\n");
		stmA3967_DIR_CCW
	}
	delayms(10);

	stmA3967_GO //nEN =0

	//for loop steps
	for(i=0;i<nStep;i++){
		//make step pulse
		stmA3967_STEP_0
		delayms(lowdur);//delayms(lowdur);
		stmA3967_STEP_1
		somedelay(500);//delayms(highdur);
	}
	stmA3967_STEP_0
	stmA3967_STOP //nEN =1
}

void stmA3967GoStepsForward(u32 nStep){
	stmA3967GoSteps(FORWARD,nStep);
}
void stmA3967GoStepsBackward(u32 nStep){
	stmA3967GoSteps(BACKWARD,nStep);
}

void stmA3967BipolarMotorLoop(){
	u32 count=0;
	u32 rev = 0;
	u32 steps;

	printf("Bipolar Motor Driver A3967 Test.\r\n");

	stmA3967Conf();

	rev = 1;
	steps = STEPSPERREV * rev;

	while(1){
		stmA3967GoStepsForward(steps);//stmA3967GoSteps(FORWARD,steps);
		delayms(200);

		stmA3967GoStepsBackward(steps);//stmA3967GoSteps(BACKWARD,steps);
		delayms(200);
	}
}
#endif
#endif
