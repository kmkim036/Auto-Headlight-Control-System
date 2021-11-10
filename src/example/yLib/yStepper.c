/* Stepper library
 * Drives a unipolar, bipolar, or five phase stepper motor.
 *
 * By making use of the fact that at any time two of the four motor coils are
 * the inverse of the other two, the number of control connections can be
 * reduced from 4 to 2 for the unipolar and bipolar motors.
 *
 * A slightly modified circuit around a Darlington transistor array or an
 * L293 H-bridge connects to only 2 microcontroler pins, inverts the signals
 * received, and delivers the 4 (2 plus 2 inverted ones) output signals
 * required for driving a stepper motor. Similarly the Arduino motor shields
 * 2 direction pins may be used.
 *
 * The sequence of control signals for 5 phase, 5 control wires is as follows:
 *
 * Step C0 C1 C2 C3 C4
 *    1  0  1  1  0  1
 *    2  0  1  0  0  1
 *    3  0  1  0  1  1
 *    4  0  1  0  1  0
 *    5  1  1  0  1  0
 *    6  1  0  0  1  0
 *    7  1  0  1  1  0
 *    8  1  0  1  0  0
 *    9  1  0  1  0  1
 *   10  0  0  1  0  1
 *
 * The sequence of control signals for 4 control wires is as follows:
 *
 * Step C0 C1 C2 C3
 *    1  1  0  1  0
 *    2  0  1  1  0
 *    3  0  1  0  1
 *    4  1  0  0  1
 *
 * The sequence of controls signals for 2 control wires is as follows
 * (columns C1 and C2 from above):
 *
 * Step C0 C1
 *    1  0  1
 *    2  1  1
 *    3  1  0
 *    4  0  0
 *
 * The circuits can be found at
 *
 * http://www.arduino.cc/en/Reference/Stepper
 */

#include <string.h>
#include <stdarg.h>
#include "yInc.h"

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#else
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "yInc.h"
#endif

extern unsigned long micros();
extern char *float2str(float x);

//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39   |
//+-----------------+-----------+-----------+-----------+---------------+------------
//| A1(Pin32)     |           |           |           |  PB11         |PB5
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| A2(Pin33)     |           |           |           |  PB1          |PA15
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| B1(Pin34)     |           |           |           |  PA5          |PB3(remap)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| B2(Pin35)     |           |           |           |  PA4          |PB4(remap)
//+-----------------+-----------+-----------+-----------+---------------+---------+

#if (MCU_MODULE_VER	== 35)
#define A1_1      {GPIO_SetBits(GPIOB, GPIO_Pin_11);}
#define A1_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_11);}
#define A2_1      {GPIO_SetBits(GPIOB, GPIO_Pin_1);}
#define A2_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_1);}

#define B1_1      {GPIO_SetBits(GPIOA, GPIO_Pin_5);}
#define B1_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_5);}
#define B2_1      {GPIO_SetBits(GPIOA, GPIO_Pin_4);}
#define B2_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_4);}

#else
#define A1_1      {GPIO_SetBits(GPIOB, GPIO_Pin_5);}
#define A1_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_5);}
#define A2_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
#define A2_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}

#define B1_1      {GPIO_SetBits(GPIOB, GPIO_Pin_3);}
#define B1_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_3);}
#define B2_1      {GPIO_SetBits(GPIOB, GPIO_Pin_4);}
#define B2_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_4);}
#endif

 struct _stepper{
	 int stepsPerRevolution;
	 int step_number;    //current steps
	 int direction;      // motor direction
	 int last_step_time; // time stamp in us of the last step taken
	 int Total_number_of_steps; // total number of steps to be required.
	 int step_delay;

	  // pin_count is used by the stepMotor() method:
	  int pin_count;
} g_stepper;

/*
 * two/four/five-wire constructor.
 * Sets which wires should control the motor.
 */
extern void stmUsingPB3_4_AsGPIO();

void stmStepper_GPIO_setup(void){
	GPIO_InitTypeDef GPIO_InitStruct;

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
#if	(MCU_MODULE_VER == 35)
	//PB11, PB1, PA5, PA4
	//+-----------------+-----------+-----------+-----------+---------------+------------------
	//|                 |           |           |           | STM103M35     |STM32F103-M37/M39   |
	//+-----------------+-----------+-----------+-----------+---------------+------------
	//| BIN2(Pin32)     |           |           |           |  PB11         |PB5
	//+-----------------+-----------+-----------+-----------+---------------+---------+
	//| BIN1(Pin33)     |           |           |           |  PB1          |PA15
	//+-----------------+-----------+-----------+-----------+---------------+---------+
	//| AIN2(Pin34)     |           |           |           |  PA5          |PB3(remap)
	//+-----------------+-----------+-----------+-----------+---------------+---------+
	//| AIN1(Pin35)     |           |           |           |  PA4          |PB4(remap)
	//+-----------------+-----------+-----------+-----------+---------------+---------+
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOB, GPIO_Pin_11 | GPIO_Pin_1);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_4);

#else
	//PB5, PA15, PB3, PB4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);

	stmUsingPB3_4_AsGPIO(); //remap
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_3 | GPIO_Pin_4);

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
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	//GPIO_ResetBits(GPIOG, GPIO_Pin_7);// ZGT6
#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
	//PB5, PA4, PB3, PB4
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //STM401-M35
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_3 | GPIO_Pin_4;//STM401-M35
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);//STM401-M35
	GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_3 | GPIO_Pin_4); //STM401-M35

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //STM401-M35
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;//STM401-M35
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);//STM401-M35
	GPIO_ResetBits(GPIOA, GPIO_Pin_4); //STM401-M35
#endif
}

stmStepperInit(int stepsPerRevolution, int pin_count)
{
  stmStepper_GPIO_setup();  //MCU Pins for the motor control connection:

  g_stepper.direction = 0;      // motor direction
  g_stepper.last_step_time = 0; // time stamp in us of the last step taken
  g_stepper.stepsPerRevolution = stepsPerRevolution; // total number of steps for this motor
  g_stepper.pin_count = pin_count;
}
// Sets the speed in RPM
void Stepper_SetSpeed(long whatSpeed)
{
  g_stepper.step_delay = (int)(60L * 1000L / g_stepper.stepsPerRevolution / whatSpeed);//60L * 1000L * 1000L / g_stepper.stepsPerRevolution / whatSpeed;
}

// Moves the motor steps_to_move steps.  If the number is negative, the motor moves in the reverse direction.
void stepper_RunForGivenSteps(int steps_to_move)
{
	unsigned long nowInTick;
  int steps_left = abs(steps_to_move);  // how many steps to take

  // determine direction based on whether steps_to_mode is + or -:
  if (steps_to_move > 0) { g_stepper.direction = 1; }
  if (steps_to_move < 0) { g_stepper.direction = 0; }

  // decrement the number of steps, moving one step each time:
  while (steps_left > 0) {

	  nowInTick = micros();

	  // move only if the appropriate delay has passed:
	  if (nowInTick - g_stepper.last_step_time >= g_stepper.step_delay){
      // get the timeStamp of when you stepped:
      g_stepper.last_step_time = nowInTick;
      // increment or decrement the step number, depending on direction:
      if (g_stepper.direction == 1) {
        g_stepper.step_number++;
        if (g_stepper.step_number == g_stepper.Total_number_of_steps)
        	g_stepper.step_number = 0;
      }
      else {
        if (g_stepper.step_number == 0)
        	g_stepper.step_number = g_stepper.Total_number_of_steps;
        g_stepper.step_number--;
      }
      steps_left--;       // decrement the steps left:
      stepper_GenStepPulse(g_stepper.step_number % g_stepper.stepsPerRevolution);
    }
  }
}

// Moves the motor forward or backwards.

void stepper_GenStepPulse(int thisStep)
{
	if (g_stepper.stepsPerRevolution == 4) {
    switch (thisStep) {
      case 0:  // 1010
        A1_1;        A2_0;        B1_1;        B2_0;
      break;
      case 1:  // 0110
    	  A1_0;   	  A2_1;    	  B1_1;    	  B2_0;
      break;
      case 2:  //0101
    	  A1_0;    	  A2_1;    	  B1_0;    	  B2_1;
      break;
      case 3:  //1001
    	  A1_1;    	  A2_0;    	  B1_0;    	  B2_1;
      break;
    }
  }else if (g_stepper.stepsPerRevolution == 6) {
    switch (thisStep) {
      case 0:  //1001
    	  A1_1;   A2_0;  B1_0;  B2_0;
      break;
      case 1:  //0001
    	  A1_0;	  A2_0;    	  B1_0;    	  B2_1;
      break;
      case 2:  //0111
    	  A1_0;    	  A2_1;    	  B1_1;    	  B2_1;
      break;
      case 3:  //0110
    	  A1_0;    	  A2_1;    	  B1_1;    	  B2_0;
      break;
      case 4:  //1110
    	  A1_1;    	  A2_1;    	  B1_1;    	  B2_0;
      break;
      case 5:  //1000
    	  A1_1;    	  A2_0;    	  B1_0;    	  B2_0;
      break;

    }
  }

}

void stmStepper_Loop(void)
{
	static int nextPos = 0;
	char c;
	//Steps for a signle rotation = 6
	//Pointer movement of 1 degree = Need 3 steps.
	//Pointer can move max 315 degree -> total 315 x 3 steps.
	//Pointer can move max 315 degree -> total 315/2 rotations.
	int WantSteps;
	int WantRevolutions = 220/2; //315/2; // MR1108 :
	int WantRPM = 720*6; //60x

	printf("Stepper Test-bitbang \r\n");

	g_stepper.stepsPerRevolution = 6;//60 degree of shaft // MR1108
	g_stepper.Total_number_of_steps = WantRevolutions * g_stepper.stepsPerRevolution;

	Init_SysTick(1000); //1msec tick

	stmStepperInit(g_stepper.stepsPerRevolution, 4);
	Stepper_SetSpeed(WantRPM);

	while(1){
		//(1) Correct to do a single Full Rotation.
		printf("Run Clockwise\r\n");
		stepper_RunForGivenSteps(g_stepper.Total_number_of_steps);
		delayms(500);
		printf("Run CountClockwise\r\n");
		stepper_RunForGivenSteps(-g_stepper.Total_number_of_steps);
		delayms(500);
		//(2) position to given speed.
		//...
		//
	}
}


