
/* TB6612FNG H-Bridge Motor Driver Example code
Uses 2 motors to show examples of the functions in the library.  This causes
a robot to do a little 'jig'.  Each movement has an equal and opposite movement
so assuming your motors are balanced the bot should end up at the same place it
started.

GPIO_Remap : www.minokasago.org/STM32wiki/index.php?GPIO_PinRemapConfig
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#else
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
#include "yInc.h"
#endif
static uint32_t speed = 100;

//GPIO PINs
//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39   |
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN2(Pin32)     |           |           |           |  PB11         |PB5
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN1(Pin33)     |           |           |           |  PB1          |PA15
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN2(Pin34)     |           |           |           |  PA5          |PB3
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN1(Pin35)     |           |           |           |  PA4          |PB4
//+-----------------+-----------+-----------+-----------+---------------+---------+

/* 28BYJ-48 DC Stepper Motor (5V or 12V Version)
 * It is driving by ULN2003A Darlington Current Driver.
 *
   5V/12V 5 Pins Stepper Motor
 * Red : Pin 1 : Common (+5V/+12V)
 * Org : Pin 2 : 1A                  | Pin 21 PG1
 * Yel : Pin 3 : 1B                  | Pin 26 PF1
 * Pnk : Pin 4 : 2A                  | Pin 27 PE3
 * Blu : Pin 5 : 2B                  | Pin 28 PE2
 */

#define MOTORSPEED 8000   // You can change speed... 8000 may be the fastest.. 40000 can show step and LED
#define COUNTPERREV 512  //# of steps per revolution

u8 pulses[8] = {0x8, 0xc, 0x4, 0x6, 0x2, 0x3, 0x1, 0x9};
/*        0     1    2    3    4   5    6    7    0'   1'
 * 1A   +----+----+                        +----+~~~~+~~~~+
 *                |                        |              $
 *                +----+----+----+----+----+              +~~~~+
 *
 * 1B        +----+----+----+                        +~~~~+~~~~+
 *           |              |                        |
 *      +----+              +----+----+----+----+~~~~+
 *
 * 2A                  +----+----+----+
 *                     |              |
 *      +----+----+----+              +----+----+~~~~+~~~~+~~~~+
 *
 * 2B                            +----+----+----+
 *                               |              |
 *      +----+----+----+----+----+              +~~~~+~~~~+~~~~+
 */
void stmUnipolarMotorConf(){
	GPIO_InitTypeDef GPIO_InitStruct;

#if(PROCESSOR == PROCESSOR_STM32F103C8T6)
#if (MCU_MODULE_VER	== 35)
	//PB11/1 = 1A/1B
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	//PA5/4 = 2A/2B
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;// | GPIO_Pin_8 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOA, &GPIO_InitStruct);//
#else
	printf("TBD...\r\n");
#endif
	//PC13 --EN -- Shared with Buzzer
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

	//PC14 --EN -- LED
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

#else
#endif
}

void stmUnipolarMotorOutPulse4 (u8 i){
	u8 pulse4;
	pulse4 = pulses[i] & 0x0f;

	if(pulse4 & 0x08) 	GPIO_SetBits(GPIOB, GPIO_Pin_11); //1A
	else 				GPIO_ResetBits(GPIOB, GPIO_Pin_11); //~1A

	if(pulse4 & 0x04) 	GPIO_SetBits(GPIOB, GPIO_Pin_1); //1B
	else 				GPIO_ResetBits(GPIOB, GPIO_Pin_1); //~1B

	if(pulse4 & 0x02) 	GPIO_SetBits(GPIOA, GPIO_Pin_5); //2A
	else 				GPIO_ResetBits(GPIOA, GPIO_Pin_5); //~2A

	if(pulse4 & 0x01) 	GPIO_SetBits(GPIOA, GPIO_Pin_4); //2B
	else 				GPIO_ResetBits(GPIOA, GPIO_Pin_4); //~2B
}

void stmUnipolarMotorClockwise(){
	char i;
	for(i=8; i >0; i--){
		stmUnipolarMotorOutPulse4(i-1);
		somedelay(MOTORSPEED);
	}
}

void stmUnipolarMotorCountClockwise(){
	u8 i;
	for(i=0; i < 8; i++){
		stmUnipolarMotorOutPulse4(i);
		somedelay(MOTORSPEED);
	}
}

void stmUnipolarMotorLoop(){
	u32 count=0;

	printf("stmUnipolar Motor Driver Test.\r\n");
	stmUnipolarMotorConf();

	while(1){
		if (count < COUNTPERREV){
			stmUnipolarMotorClockwise();
		}
		else if(count >= COUNTPERREV*2){
			count = 0;
			printf("Clockwise...\r\n");
			stmUserLED_ON();//GPIO_SetBits(GPIOC, GPIO_Pin_14); 	//ON
			stmBeepPC13(100);
		}else{
			if(count == COUNTPERREV){
				printf("CountClockwise...\r\n");
				stmUserLED_OFF();//GPIO_ResetBits(GPIOC, GPIO_Pin_14);//OFF
				stmBeepPC13(100);
			}
			stmUnipolarMotorCountClockwise();
		}
		count++;
		printf("Count=%d\r",count);

	}
}
