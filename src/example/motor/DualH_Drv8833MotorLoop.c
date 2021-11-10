/* Drv8833 Dual H-Bridge Motor Driver Example code
Uses 2 motors to show examples of the functions in the library.  This causes
a robot to do a little 'jig'.  Each movement has an equal and opposite movement
so assuming your motors are balanced the bot should end up at the same place it
started.
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"

//Only support M37..for TIMER pins limit

#if (MCU_MODULE_VER	== 35)

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
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

#define TIM_PRESCALE (0)
#define PWM_PERIOD 	 7200 //10KHz

/*[INPUT]                   [OUTPUT]     [MODE]
 * AIN1  AIN2   EN(nSLEEP)  AOUT1 AOUT2
 * PA4   PA5    PC13
 * PWM   0      H           CW             Forward, fast decay* -- We use
 * PWM   1      H           CCW            Reverse, slow decay* -- We use
 * 0     PWM    H           CCW            Reverse, fast decay*
 * 1     PWM    H           CW             Forward, slow decay*
 * 1     1      H           Brake
  */
//GPIO PINs -- Should use M35 verion
//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |           |           |           | STM103M35*     |STM32F103-M37/M39   |
//+-----------------+-----------+-----------+-----------+---------------+------------
//| BIN2(Pin32)     | DIR       |           |           | PB11(TIM2_CH4)| PB5(TIM3_CH2)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN1(Pin33)     | PWM       |           |           | PB1(TIM3_CH4) |PA15(TIM2_CH1)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN2(Pin34)     | DIR       |           |           | PA5(NO TIM)   |PB3(TIM2_CH2) -- Need remap
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN1(Pin35)     | PWM       |           |           | PA4(NO_TIM)   |PB4(TIM3_CH1) -- Need remap
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| EN  (Pin22)     |           |           |           |  PC13
//+-----------------+-----------+-----------+-----------+---------------+---------+
//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39*   |
//+-----------------+-----------+-----------+-----------+---------------+------------
//| BIN2(Pin32)     | PWM       |           |           |  PB11         |PB5(TIM3_CH2)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN1(Pin33)     | Dir       |           |           |  PB1          |PA15(NO TIM)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN2(Pin34)     | PWM       |           |           |  PA5          |PB3(TIM2_CH2)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN1(Pin35)     | Dir       |           |           |  PA4          |PB4(TIM3_CH1)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| EN  (Pin22)     |           |           |           |  PC13
//+-----------------+-----------+-----------+-----------+---------------+---------+
//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |           |           |           | STM103M35*     |STM32F103-M37/M39   |
//+-----------------+-----------+-----------+-----------+---------------+------------
//| BIN2(Pin32)     | DIR       |           |           | PB11(TIM2_CH4)| PB5(TIM3_CH2)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN1(Pin33)     | PWM       |           |           | PB1(TIM3_CH4) |PA15(TIM2_CH1)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN2(Pin34)     | DIR       |           |           | PA5(NO TIM)   |PB3(TIM2_CH2) -- Need remap
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN1(Pin35)     | PWM       |           |           | PA4(NO_TIM)   |PB4(TIM3_CH1) -- Need remap
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| EN  (Pin22)     |           |           |           |  PC13
//+-----------------+-----------+-----------+-----------+---------------+---------+

//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |           |           |           | STM103M35.M78*|STM32F103-M37/M39   |
//+-----------------+-----------+-----------+-----------+---------------+------------
//| BIN2(Pin32)     | DIR       |           |           | PB11(TIM2_CH4)| PB5(TIM3_CH2)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN1(Pin33)     | PWM       |           | Need remap| PB1(TIM3_CH4) |PA15(TIM2_CH1)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN2(Pin34)     | DIR       |           |           | PA5(NO TIM)   |PB3(TIM2_CH2)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN1(Pin35)     | PWM       |           |           | PA4(NO_TIM)   |PB4(TIM3_CH1)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| EN  (Pin22)     |           |           |           |  PC13
//+-----------------+-----------+-----------+-----------+---------------+---------+
#define DEFAULTSPEED 255

#define DIR_CW 		 1
#define DIR_CCW 	 0

stmDrv8833_Config(int duty, int offset);
// Drive in direction given by sign, at speed given by magnitude of the parameter.
void stmDrv8833_driveEachMotor(char Motor, int speed, int duration);

//currently not implemented
//void stop();           // Stop motors, but allow them to coast to a halt.
//void coast();          // Stop motors, but allow them to coast to a halt.

//Stops motor by setting both input pins high
void stmDrv8833_brake();

//set the chip to standby mode.  The drive function takes it out of standby
//(forward, back, left, and right all call drive)
void stmDrv8833_standby();

void stmDrv8833_fwdEachMotor(char Motor, int speed);
void stmDrv8833_revEachMotor(char Motor, int speed);
void stmDrv8833_forwardBothMotor(int speed, int duration);
void stmDrv8833_backBothMotor(int speed, int duration);

//Left and right take 2 motors, and it is important the order they are sent.
//The left motor should be on the left side of the bot.  These functions
//also take a speed value
void stmDrv8833_left(int speed, int duration);
void stmDrv8833_right(int speed, int duration);

//This function takes 2 motors and and brakes them
void stmDrv8833_brake();

//variables for the 2 inputs, PWM input, Offset value, and the Standby pin
int In1, In2, PWM, Offset,Standby;
int stmDrv8833_Offset;

#define MOTORA 'A'
#define MOTORB 'B'

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)

#if (MCU_MODULE_VER	== 35)
#define AIN2_8833_CW0      {GPIO_ResetBits(GPIOA, GPIO_Pin_5);}
#define AIN2_8833_CCW1      {GPIO_SetBits(GPIOA, GPIO_Pin_5);} //PA5

#define BIN2_8833_CW0      {GPIO_ResetBits(GPIOB, GPIO_Pin_11);}
#define BIN2_8833_CCW1      {GPIO_SetBits(GPIOB, GPIO_Pin_11);} //PB11

#define EN_8833_ON      {GPIO_SetBits(GPIOC, GPIO_Pin_13);}    //PC13
#define EN_8833_OFF      {GPIO_ResetBits(GPIOC, GPIO_Pin_13);} //DISABLE

#else
//#define AIN1_8833_1      {GPIO_SetBits(GPIOB, GPIO_Pin_4);}
//#define AIN2_8833_CCW1      {GPIO_ResetBits(GPIOB, GPIO_Pin_4);}
#define AIN2_8833_CCW1      {GPIO_SetBits(GPIOB, GPIO_Pin_3);} //
#define AIN2_8833_CW0      {GPIO_ResetBits(GPIOB, GPIO_Pin_3);}

//#define BIN1_8833_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
//#define BIN2_8833_CCW1      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
#define BIN2_8833_CCW1      {GPIO_SetBits(GPIOB, GPIO_Pin_5);}
#define BIN2_8833_CW0      {GPIO_ResetBits(GPIOB, GPIO_Pin_5);}

#define EN_8833_1      {GPIO_SetBits(GPIOC, GPIO_Pin_13);} //EN
#define SLEEP_8833_0      {GPIO_ResetBits(GPIOC, GPIO_Pin_13);} //EN
#endif

#if 0
#define AIN1_8833_1      {GPIO_SetBits(GPIOA, GPIO_Pin_4);}
#define AIN2_8833_CCW1      {GPIO_ResetBits(GPIOA, GPIO_Pin_4);}
#define AIN2_8833_1      {GPIO_SetBits(GPIOA, GPIO_Pin_5);}
#define AIN2_8833_CW0      {GPIO_ResetBits(GPIOA, GPIO_Pin_5);}

#define BIN1_8833_1      {GPIO_SetBits(GPIOB, GPIO_Pin_1);}
#define BIN2_8833_CCW1      {GPIO_ResetBits(GPIOB, GPIO_Pin_1);}
#define BIN2_8833_1      {GPIO_SetBits(GPIOB, GPIO_Pin_11);}
#define BIN2_8833_CW0      {GPIO_ResetBits(GPIOB, GPIO_Pin_11);}

#define EN_8833_1      {GPIO_SetBits(GPIOC, GPIO_Pin_13);} //EN
#define SLEEP_8833_0      {GPIO_ResetBits(GPIOC, GPIO_Pin_13);} //EN
#endif

#else
// GPIO Assignment (KONG-STM32F407)
// [A]
// PWMA : Pin6 :  PC7 -- Adjust speed : TIM3_CH2
// AIN1 : Pin29 : PC11 -- control direction
// AIN2 : Pin30 : PC10 -- control direction
// [B]
// PWMB : Pin7 :  PE6 -- Adjust speed : TIM9_CH2
// BIN1 : Pin31 : PA15  -- control direction
// BIN2 : Pin32 : PC12 -- control direction

// STBY : PIN22 : PC6(KongV4)//PE14(KongV0)


//#define PWMA_8833_1      {GPIO_SetBits(GPIOE, GPIO_Pin_0);} PWM3
//#define PWMA_8833_0      {GPIO_ResetBits(GPIOE, GPIO_Pin_0);}
#define AIN1_8833_1      {GPIO_SetBits(GPIOC, GPIO_Pin_11);}
#define AIN2_8833_CCW1      {GPIO_ResetBits(GPIOC, GPIO_Pin_11);}
#define AIN2_8833_1      {GPIO_SetBits(GPIOC, GPIO_Pin_10);}
#define AIN2_8833_CW0      {GPIO_ResetBits(GPIOC, GPIO_Pin_10);}

//#define PWMB_8833_1      {GPIO_SetBits(GPIOB, GPIO_Pin_14);} PWM..TIM12_CH1
//#define PWMB_8833_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_14);}
#define BIN1_8833_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
#define BIN2_8833_CCW1      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
#define BIN2_8833_1      {GPIO_SetBits(GPIOC, GPIO_Pin_12);}
#define BIN2_8833_CW0      {GPIO_ResetBits(GPIOC, GPIO_Pin_12);}

//PC6 - KONGv4
//#define EN_8833_1      {GPIO_SetBits(GPIOC, GPIO_Pin_6);}
//#define SLEEP_8833_0      {GPIO_ResetBits(GPIOC, GPIO_Pin_6);}

//PE14 - KONGv0
#define EN_8833_1      {GPIO_SetBits(GPIOE, GPIO_Pin_14);}
#define SLEEP_8833_0      {GPIO_ResetBits(GPIOE, GPIO_Pin_14);}
#endif

void stmDrv8833_PWM_Config(int duty) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
#if (MCU_MODULE_VER == 35)

	//First for Motor B
#if 1
	//For PWM -- using TIM2, PB11
	//For DIR -- using PB1

	//DIR pin config (PB1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_1);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOB, GPIO_Pin_1); //Clockwise

	//PWM pin config (PB11)
	//(0) ======= STM103 Specific Port Remapping =======================
	// GPIO_PartialRemap1_TIM2 : PA14 and PB3*
	// GPIO_PartialRemap2_TIM2 : PB10 and PB11*
	// GPIO_FullRemap_TIM2 : PA15,PB3,PB10 and PB11
	// GPIO_PartialRemap_TIM3 : PB4* and PB5
	// GPIO_FullRemap_TIM3 : PC6,7,8,9
	// GPIO_Remap_SWJ_JTAGDisable: PB3/PB4 Remap to GPIO

    //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3->GPIO, PB4->GPIO. JTAG-DP Disable
    //GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); //Remap for TIM2 : for PB3* and PA14
    //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);  //Remap for TIM3 : for PB4* and PB5

	//[Critical] For using PB11 as TIM2_CH4, we should remap as the following.
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
    //(1)
	//(a)Configure PB11 as outputs for PWM
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_11);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//(b) TIM2 clock enable for Speed control (PB11-TIM2_CH4)
	TIM_DeInit(TIM2); //for PWM-B
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (PWM_PERIOD - 1);// ---> 10kHz
	//TIM_TimeBaseStructure.TIM_Prescaler = 2-1; // --->36MHz
	//TIM_TimeBaseStructure.TIM_Prescaler = 4-1; // ---> 72/4 MHz
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_PRESCALE; //0
	//TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	//TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// (d) PWMA Mode configuration: Channel 4
	//Duty = (TIM_Pulse/TIM_Period)*100 %
	//Thus, for given Duty, Pulse = Duty * Period / 100;
	//Timer Clock = 72MHz,

	// -------------->CCR      APR
	// |               +--------+
	// |               |        |
	// +---------------+        +-...
	// |<-------- Period ------>|

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = duty * (PWM_PERIOD)/100;
	//TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure); //OCx == x is for channel id.

	//TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	//TIM_ARRPreloadConfig(TIM3, ENABLE);

	//(e) Enable and output
	TIM_Cmd(TIM2, ENABLE);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);


#else
	//For PWM -- using TIM3, PB1
	//For DIR -- using PB11

	//DIR pin config (PB11)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_11);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOB, GPIO_Pin_11); //Clockwise

	//Using PB1 with TIM3_CH4
	//GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE); //No need for PB1.
    //(1)
	//(a)Configure PB11 as outputs for PWM
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_1);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//(b) TIM3 clock enable for Speed control (PB1-TIM3_CH4)
	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (PWM_PERIOD - 1);// ---> 10kHz
	//TIM_TimeBaseStructure.TIM_Prescaler = 2-1; // --->36MHz
	//TIM_TimeBaseStructure.TIM_Prescaler = 4-1; // ---> 72/4 MHz
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_PRESCALE; //0
	//TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	//TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// (d) PWMA Mode configuration: Channel 4
	//Duty = (TIM_Pulse/TIM_Period)*100 %
	//Thus, for given Duty, Pulse = Duty * Period / 100;
	//Timer Clock = 72MHz,

	// -------------->CCR      APR
	// |               +--------+
	// |               |        |
	// +---------------+        +-...
	// |<-------- Period ------>|

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = duty * (PWM_PERIOD)/100;
	//TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure); //OCx == x is for channel id.

	//(e) Enable and output
	TIM_Cmd(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
#endif

	//(2) ------------------  Motor A


#elif (MCU_MODULE_VER < 70)

#endif
#endif
}

void stmDrv8833_fwdEachMotor(char Motor, int speed)
{
	if(Motor == 'A'){
		//AIN1_8833_1;
		AIN2_8833_CW0 ;

	}else{
		//BIN1_8833_1;
		BIN2_8833_CW0 ;
	}
}

void stmDrv8833_revEachMotor(char Motor, int speed)
{
	if(Motor == 'A'){
		AIN2_8833_CCW1 ;
	}else{
		BIN2_8833_CCW1 ;
	}
}

//========================================================
void stmDrv8833_driveEachMotor(char Motor, int speed, int duration)
{
	EN_8833_ON;
	speed = speed * stmDrv8833_Offset;
	if (speed>=0) stmDrv8833_fwdEachMotor(Motor, speed);
	else stmDrv8833_revEachMotor(Motor, -speed);
	delayms(duration);
	EN_8833_OFF;
}
//========================================================
void stmDrv8833_forwardBothMotor(int speed, int duration)
{
	printf("Drv8833>forwardBothMotor\r\n");
	//stmDrv8833_driveEachMotor('A', speed, duration);
	stmDrv8833_driveEachMotor('B', speed, duration);
}

void stmDrv8833_backBothMotor(int speed, int duration)
{
	int temp = abs(speed);
	printf("Drv8833>backBothMotor\r\n");
	//stmDrv8833_driveEachMotor('A', -temp,duration);
	stmDrv8833_driveEachMotor('B', -temp, duration);
}

void stmDrv8833_left(int speed, int duration)
{
	int temp = abs(speed)/2;
	printf("Drv8833>LeftMotor\r\n");
	//stmDrv8833_driveEachMotor('A', -temp, duration); //left motor
	stmDrv8833_driveEachMotor('B', temp, duration); //right motor

}

void stmDrv8833_right(int speed, int duration)
{
	int temp = abs(speed)/2;
	printf("Drv8833>RightMotor\r\n");
	//stmDrv8833_driveEachMotor('A', temp, duration); //left motor
	stmDrv8833_driveEachMotor('B', -temp, duration); //right motor
}

void stmDrv8833_brake()
{
		//AIN1_8833_1;
		AIN2_8833_CCW1;
		//analogWrite(PWM,0);

		//BIN1_8833_1;
		BIN2_8833_CCW1;
		//analogWrite(PWM,0);
}
void stmDrv8833_standby()
{
   EN_8833_OFF;
}
//========================== module config ==========================================
stmDrv8833_Config(int duty, int offset)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	stmDrv8833_PWM_Config(duty);

	//DIR and EN pin config
#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)

	//PC13 --EN -- [Note] It is shared with Buzzer on the MCU module.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //PC13
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

	stmDrv8833_standby(); //EN OFF

#endif

	printf("Init Drv8833.\r\n");
	stmDrv8833_Offset = offset;
}
//=============== main loop ==============
void stmDrv8833Loop(){

	int i;
	stmDrv8833_Offset = 0;

	printf("DRV8833 TEST\r\n");

	stmDrv8833_Config(
			90, //duty = 90%
			0);//offset -> TBD

	printf("Config Done\r\n");

	EN_8833_ON; //enable

	while(1){
		//stmDrv8833_forwardBothMotor(1000, 1000);
#if 1
		GPIO_ResetBits(GPIOB, GPIO_Pin_1); //BIN2_8833_CW0 ;
#else
		GPIO_ResetBits(GPIOB, GPIO_Pin_11); //BIN2_8833_CW0 ;
#endif
		delayms(500);
		//stmDrv8833_standby();
#if 1
		GPIO_SetBits(GPIOB, GPIO_Pin_1); //BIN2_8833_CW0 ;//stmDrv8833_backBothMotor(1000,1000);
#else
		GPIO_SetBits(GPIOB, GPIO_Pin_11); //BIN2_8833_CW0 ;//stmDrv8833_backBothMotor(1000,1000);
#endif
		delayms(500);
		//stmDrv8833_standby();
		delayms(500);
	}
}

#endif
