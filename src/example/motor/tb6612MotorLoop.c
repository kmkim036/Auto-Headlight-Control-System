
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
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))
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
/*[INPUT]                   [OUTPUT]    [MODE]
 * IN1  IN2  PWM  STBY       OUT1  OUT2
 * H    H    H/L  H          L     L     Short Brake
 * L    H    H    H          L     H     CCW
 *           L    H          L     L     Short Brake
 * H    L    H    H          H     L     CW
 *           L    H          L     L     Short Brake
 * L    L    H    H          OFF   OFF   STOP
*  H/L  H/L  H/L  L          OFF   OFF   STANDBY
 */
//GPIO PINs
//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39   |
//+-----------------+-----------+-----------+-----------+---------------+------------
//| PWMA(Pin6)      |           |           |need remap |  PB3/TIM2_CH2 |PB1/TIM3_CH4 |
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| PWMB(Pin7)      |           |           |need remap |  PB4/TIM3_CH1 |PA4/NA
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN2(Pin32)     |           |           |           |  PB11         |PB5
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN1(Pin33)     |           |           |           |  PB1          |PA15
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN2(Pin34)     |           |           |           |  PA5          | PB3
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN1(Pin35)     |           |           |           |  PA4          |PB4
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| EN  (Pin22)     |           |           |           |  PC13
//+-----------------+-----------+-----------+-----------+---------------+---------+
//used in some functions so you don't have to send a speed
#define DEFAULTSPEED 255

    stmTb6612_Config(int offset);

    // Drive in direction given by sign, at speed given by magnitude of the parameter.
    void stmTb6612_driveEachMotor(char Motor, int speed, int duration);

    //currently not implemented
    //void stop();           // Stop motors, but allow them to coast to a halt.
    //void coast();          // Stop motors, but allow them to coast to a halt.

    //Stops motor by setting both input pins high
    void stmTb6612_brake();

    //set the chip to standby mode.  The drive function takes it out of standby
    //(forward, back, left, and right all call drive)
    void stmTb6612_standby();

    //variables for the 2 inputs, PWM input, Offset value, and the Standby pin
    int In1, In2, PWM, Offset,Standby;

    //private functions that spin the motor CC and CCW
    void stmTb6612_fwdEachMotor(char Motor, int speed);
    void stmTb6612_revEachMotor(char Motor, int speed);

//Takes 2 motors and goes forward, if it does not go forward adjust offset
//values until it does.  These will also take a negative number and go backwards
//There is also an optional speed input, if speed is not used, the function will
//use the DEFAULTSPEED constant.
void stmTb6612_forwardBothMotor(int speed, int duration);

//Similar to forward, will take 2 motors and go backwards.  This will take either
//a positive or negative number and will go backwards either way.  Once again the
//speed input is optional and will use DEFAULTSPEED if it is not defined.
void stmTb6612_backBothMotor(int speed, int duration);

//Left and right take 2 motors, and it is important the order they are sent.
//The left motor should be on the left side of the bot.  These functions
//also take a speed value
void stmTb6612_left(int speed, int duration);
void stmTb6612_right(int speed, int duration);

//This function takes 2 motors and and brakes them
void stmTb6612_brake();


#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)

//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39   |
//+-----------------+-----------+-----------+-----------+---------------+------------
//| PWMA(Pin6)      |           |           |need remap |  PB3/TIM2_CH2 |PB1/TIM3_CH4 |
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| PWMB(Pin7)      |           |           |need remap |  PB4/TIM3_CH1 |PA4/NA
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN2(Pin32)     |           |           |           |  PB11         |PB5
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN1(Pin33)     |           |           |           |  PB1          |PA15
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN2(Pin34)     |           |           |           |  PA5          | PB3
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN1(Pin35)     |           |           |           |  PA4          |PB4
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| EN  (Pin22)     |           |           |           |  PC13
//+-----------------+-----------+-----------+-----------+---------------+---------+
#if (MCU_MODULE_VER	== 35)
#define AIN1_6612_1      {GPIO_SetBits(GPIOA, GPIO_Pin_4);}
#define AIN1_6612_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_4);}
#define AIN2_6612_1      {GPIO_SetBits(GPIOA, GPIO_Pin_5);}
#define AIN2_6612_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_5);}

#define BIN1_6612_1      {GPIO_SetBits(GPIOB, GPIO_Pin_1);}
#define BIN1_6612_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_1);}
#define BIN2_6612_1      {GPIO_SetBits(GPIOB, GPIO_Pin_11);}
#define BIN2_6612_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_11);}

#define STBY_6612_1      {GPIO_SetBits(GPIOC, GPIO_Pin_13);} //EN
#define STBY_6612_0      {GPIO_ResetBits(GPIOC, GPIO_Pin_13);} //EN
#else
#define AIN1_6612_1      {GPIO_SetBits(GPIOB, GPIO_Pin_4);}
#define AIN1_6612_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_4);}
#define AIN2_6612_1      {GPIO_SetBits(GPIOB, GPIO_Pin_3);}
#define AIN2_6612_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_3);}

#define BIN1_6612_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
#define BIN1_6612_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
#define BIN2_6612_1      {GPIO_SetBits(GPIOB, GPIO_Pin_5);}
#define BIN2_6612_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_5);}

#define STBY_6612_1      {GPIO_SetBits(GPIOC, GPIO_Pin_13);} //EN
#define STBY_6612_0      {GPIO_ResetBits(GPIOC, GPIO_Pin_13);} //EN
#endif

#if 0
#define AIN1_6612_1      {GPIO_SetBits(GPIOA, GPIO_Pin_4);}
#define AIN1_6612_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_4);}
#define AIN2_6612_1      {GPIO_SetBits(GPIOA, GPIO_Pin_5);}
#define AIN2_6612_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_5);}

#define BIN1_6612_1      {GPIO_SetBits(GPIOB, GPIO_Pin_1);}
#define BIN1_6612_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_1);}
#define BIN2_6612_1      {GPIO_SetBits(GPIOB, GPIO_Pin_11);}
#define BIN2_6612_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_11);}

#define STBY_6612_1      {GPIO_SetBits(GPIOC, GPIO_Pin_13);} //EN
#define STBY_6612_0      {GPIO_ResetBits(GPIOC, GPIO_Pin_13);} //EN
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


//#define PWMA_6612_1      {GPIO_SetBits(GPIOE, GPIO_Pin_0);} PWM3
//#define PWMA_6612_0      {GPIO_ResetBits(GPIOE, GPIO_Pin_0);}
#define AIN1_6612_1      {GPIO_SetBits(GPIOC, GPIO_Pin_11);}
#define AIN1_6612_0      {GPIO_ResetBits(GPIOC, GPIO_Pin_11);}
#define AIN2_6612_1      {GPIO_SetBits(GPIOC, GPIO_Pin_10);}
#define AIN2_6612_0      {GPIO_ResetBits(GPIOC, GPIO_Pin_10);}

//#define PWMB_6612_1      {GPIO_SetBits(GPIOB, GPIO_Pin_14);} PWM..TIM12_CH1
//#define PWMB_6612_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_14);}
#define BIN1_6612_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
#define BIN1_6612_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
#define BIN2_6612_1      {GPIO_SetBits(GPIOC, GPIO_Pin_12);}
#define BIN2_6612_0      {GPIO_ResetBits(GPIOC, GPIO_Pin_12);}

//PC6 - KONGv4
//#define STBY_6612_1      {GPIO_SetBits(GPIOC, GPIO_Pin_6);}
//#define STBY_6612_0      {GPIO_ResetBits(GPIOC, GPIO_Pin_6);}

//PE14 - KONGv0
#define STBY_6612_1      {GPIO_SetBits(GPIOE, GPIO_Pin_14);}
#define STBY_6612_0      {GPIO_ResetBits(GPIOE, GPIO_Pin_14);}
#endif
int stmTb6612_Offset;

#define MOTORA 'A'
#define MOTORB 'B'
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39 | STM32F103-M70
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| PWMA(Pin6)      |           |           |*need remap| PB3*/TIM2_CH2 |PB1/TIM3_CH4      | PB1/TIM3_CH4
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| PWMB(Pin7)      |           |           |*need remap| PB4*/TIM3_CH1 |PA4/NA            | PB11/TIM2_CH4
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| BIN2(Pin32)     |           |           |           |  PB11         |PB5               | <-
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| BIN1(Pin33)     |           |           |           |  PB1          |PA15              | <-
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| AIN2(Pin34)     |           |           |           |  PA5          |PB3      		   | <-
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| AIN1(Pin35)     |           |           |           |  PA4          |PB4			   | <-
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| EN  (Pin22)     |           |           |           |  PC13         |<-				   | <-
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------


void stmTb6612_PWM_Config(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_GD32F130FX)
#if (MCU_MODULE_VER == 35)

	//(a) ======= STM103 Specific Port Remapping =======================
	//the default function of STM32F103 Module Pin6 (PB3) is JTDO.
	//For assigning it to GPIO PB3 or TIM2-CH2, we should remap it.
	//After setting PINs, we should remap for PB3 and PB4 pins
	//PB3, PB4 ==> GPIO ==> TIM
	//(Need Remap for PB3 and PB4)
	// GPIO_PartialRemap1_TIM2 : PA14 and PB3*
	// GPIO_PartialRemap2_TIM2 : PB10 and PB11
	// GPIO_FullRemap_TIM2 : PA15,PB3,PB10 and PB11
	// GPIO_PartialRemap_TIM3 : PB4* and PB5
	// GPIO_FullRemap_TIM3 : PC6,7,8,9
	// GPIO_Remap_SWJ_JTAGDisable: PB3/PB4 Remap to GPIO

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3->GPIO, PB4->GPIO. JTAG-DP Disable
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); //Remap for TIM2 : for PB3* and PA14
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);  //Remap for TIM3 : for PB4* and PB5

    //(b)
	TIM_DeInit(TIM2); //for PWM-A
	TIM_DeInit(TIM3); //for PWM-B

	//(c) TIM2 and 3 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//(d)Configure PB3 and PB4 as outputs for PWM
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_Out_PP | GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

    //(a) PWM-A TIM2-CH2
    	TIM_TimeBaseStructure.TIM_Period = 716;//
    	TIM_TimeBaseStructure.TIM_Prescaler = 0;//0;//
    	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    	/* PWMA Mode configuration: Channel 2 */
    	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    	TIM_OCInitStructure.TIM_Pulse = 500;//speed;
    	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    	TIM_OC2Init(TIM2, &TIM_OCInitStructure); //OCx == x is for channel id.

    	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    	TIM_ARRPreloadConfig(TIM2, ENABLE);

    	// TIM2 enable counter
    	TIM_Cmd(TIM2, ENABLE);
    	TIM_CtrlPWMOutputs(TIM2, ENABLE);

    	//(b) PWM-B TIM3-CH1
    	TIM_TimeBaseStructure.TIM_Period = 716;
    	TIM_TimeBaseStructure.TIM_Prescaler = 0;//
    	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    	/* PWMB Mode configuration: Channel 1 */
    	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    	TIM_OCInitStructure.TIM_Pulse = 500;//speed;
    	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    	TIM_OC1Init(TIM3, &TIM_OCInitStructure); //OCx == x is for channel id.

    	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    	TIM_ARRPreloadConfig(TIM3, ENABLE);
    	// TIM2 enable counter
    	TIM_Cmd(TIM3, ENABLE);
    	TIM_CtrlPWMOutputs(TIM3, ENABLE);

#elif (MCU_MODULE_VER < 70)
    //Pin6 :PB1 and Pin7: PA4 --> Only TIM3 available

	TIM_DeInit(TIM3); //for PWM

	//(a) TIM3 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//(b)Configure PB1 as outputs for PWM
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP | GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//(c) The pin 7(PA4) should be tied with Pin 6 if we need.
	//(a) PWM-A
		TIM_TimeBaseStructure.TIM_Period = 999;//716;
		TIM_TimeBaseStructure.TIM_Prescaler = 0;//3906;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

		/* PWMA Mode configuration: Channel 2 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 1000;//speed;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC2Init(TIM2, &TIM_OCInitStructure); //OCx == x is for channel id.

		TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM2, ENABLE);
		// TIM2 enable counter
		TIM_Cmd(TIM2, ENABLE);
		TIM_CtrlPWMOutputs(TIM2, ENABLE);

		//(b) PWM-B TIM3-CH1
		TIM_TimeBaseStructure.TIM_Period = 999;//716;
		TIM_TimeBaseStructure.TIM_Prescaler = 0;//3906;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		/* PWMB Mode configuration: Channel  */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 1000;//speed;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure); //OCx == x is for channel id.

		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM3, ENABLE);
		// TIM2 enable counter
		TIM_Cmd(TIM3, ENABLE);
		TIM_CtrlPWMOutputs(TIM3, ENABLE);

#else //M70---------------------------------------------------------------------------
	//Pin6-> PB1, Pin7->PB11
	//STM32F103 has just one PWM module, thus we should tie PWMA and PWMB pins or use only one PWMA
	//PWMA = PB1 ( PWMB(PA4 pin) ties with PWMA (i.e., ties pins 6 and 7)
	TIM_DeInit(TIM3); //for PWM-A-CH4
	TIM_DeInit(TIM2); //for PWM-B-CH4

	//(a) TIM2 and 3 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//(b)Configure PB1 and PB11 as outputs for PWM
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP | GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//(a) PWM-A(TIM3-CH4)
	TIM_TimeBaseStructure.TIM_Period = 999;//716;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//3906;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWMA Mode configuration: Channel 4 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;//speed;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure); //OCx == x is for channel id.

	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);

	// TIM2 enable counter
	TIM_Cmd(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

	//(b) PWM-B TIM2-CH4
	TIM_TimeBaseStructure.TIM_Period = 999;//716;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//3906;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* PWMB Mode configuration: Channel  */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;//speed;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure); //OCx == x is for channel id.

	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	// TIM2 enable counter
	TIM_Cmd(TIM2, ENABLE);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
#endif

#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
	//| PWMA            |           |           |           |         |PB3/TIM2_CH2 |
	//Need Remap for PB3 and PB4
	// GPIO_PartialRemap1_TIM2 : PA14 and PB3
	// GPIO_PartialRemap2_TIM2 : PB10 and PB11
	// GPIO_FullRemap_TIM2 : PA15,PB3,PB10 and PB11
	// GPIO_PartialRemap_TIM3 : PB4 and PB5
	// GPIO_FullRemap_TIM3 : PC6,7,8,9

	//(1) == COMMON == PortRemap
	//TIM_DeInit(TIM2);
	//TIM_DeInit(TIM3);
	// TIM2 and 3 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//GPIO clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	// Configure PB3/4 as outputs for PWM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//========================= STM103 Specific =================================================
	//After setting PINs, we should remap for PB3 and PB4 pins
	//PB3, PB4 ==> GPIO ==> TIM
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3->GPIO, PB4->GPIO. JTAG-DP Disabled and SW-DP Enabled
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); //GPIO_PartialRemap1_TIM2 : PA15 and PB3 --> TIM2_CH1_ETR and TIM2_CH2
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //PB4 and PB5 ==> TIM_CH1 and TIM_CH2
    //============================================================================================

	//(2) Connect PB3 to Timer 2
	TIM_TimeBaseStructure.TIM_Period = 716;
	TIM_TimeBaseStructure.TIM_Prescaler = 3906;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* PWMA Mode configuration: Channe2 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = speed;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure); //OCx == x is for channel id.
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	// TIM2 enable counter
	TIM_Cmd(TIM2, ENABLE);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);

	//(3) ========= B (PB4)===============
	//| PWMB            |           |           |           |         |PB4/TIM3_CH1
	//TIM_DeInit(TIM3);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//Enable port B clock
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// Configure PB4 as output for PWM
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(GPIOB, &GPIO_InitStructure);

    //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable | GPIO_Remap_SWJ_NoJTRST, ENABLE); //PB3 Remap /*!< JTAG-DP Disabled and SW-DP Enabled */
    //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //PB4 and PB5 ==> TIM_CH1 and TIM_CH2

    // Connect PB4 to Timer 3/channel1
	TIM_TimeBaseStructure.TIM_Period = 716;
	TIM_TimeBaseStructure.TIM_Prescaler = 3906;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	//PWM Config
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = speed;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	// TIM3 enable counter
	TIM_Cmd(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

#else
	// PWMA : Pin6 :  PC7 -- Adjust speed : TIM3_CH2
	// PWMB : Pin7 :  PE6 -- Adjust speed : TIM9_CH2
	//Enable port C clock
	TIM_DeInit(TIM3);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// Configure PC7 as output for PWM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);

	// Start Timer 3 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Connect PC7 to Timer 3 channel 2
	TIM_TimeBaseStructure.TIM_Period = 716;
	TIM_TimeBaseStructure.TIM_Prescaler = 3906;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//Sohn.
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channe2 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = speed;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure); //OCx == x is for channel id.
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

	//========= B (PE6)===============
	TIM_DeInit(TIM9);
	// Start Timer 9 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	//Enable port E clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// Configure PE6 as output for PWM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Connect PE6 to Timer 9 channel 2
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
	TIM_TimeBaseStructure.TIM_Period = 716;
	TIM_TimeBaseStructure.TIM_Prescaler = 3906;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	// PWM1 Mode configuration: Channe2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = speed;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM9, ENABLE);

	// TIM9 enable counter
	TIM_Cmd(TIM9, ENABLE);
	TIM_CtrlPWMOutputs(TIM9, ENABLE);
#endif
}



stmTb6612_Config(int offset)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	stmTb6612_PWM_Config();

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_GD32F130FX)
	//+-----------------+-----------+-----------+-----------+---------------+------------------
	//|                 |           |           |           | STM103M35     |STM32F103-M37/M39   |
	//+-----------------+-----------+-----------+-----------+---------------+------------
	//| PWMA(Pin6)      |           |           |need remap |  PB3/TIM2_CH2 |PB1/TIM3_CH4 |
	//+-----------------+-----------+-----------+-----------+---------------+---------+
	//| PWMB(Pin7)      |           |           |need remap |  PB4/TIM3_CH1 |PA4/NA
	//+-----------------+-----------+-----------+-----------+---------------+---------+
	//| BIN2(Pin32)     |           |           |           |  PB11         |PB5
	//+-----------------+-----------+-----------+-----------+---------------+---------+
	//| BIN1(Pin33)     |           |           |           |  PB1          |PA15
	//+-----------------+-----------+-----------+-----------+---------------+---------+
	//| AIN2(Pin34)     |           |           |           |  PA5          | PB3
	//+-----------------+-----------+-----------+-----------+---------------+---------+
	//| AIN1(Pin35)     |           |           |           |  PA4          |PB4
	//+-----------------+-----------+-----------+-----------+---------------+---------+
	//| EN  (Pin22)     |           |           |           |  PC13
	//+-----------------+-----------+-----------+-----------+---------------+---------+
#if (MCU_MODULE_VER == 35)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	//PA4/5
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	//PC13 --EN -- Shared with Buzzer
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

	stmTb6612_standby();
#else

#endif

#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
	//PB11/1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	//PA5/4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_8 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	//PC13 --EN -- Shared with Buzzer
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

	stmTb6612_standby();

#else
	// AIN1 : Pin29 : PC11 -- control direction
	// AIN2 : Pin30 : PC10 -- control direction
	// BIN1 : Pin31 : PA15  -- control direction
	// BIN2 : Pin32 : PC12 -- control direction
	// STBY : PIN22 : PC6
	//PC11
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //PC11
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);//
	//PC10
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //PC10
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
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

	//PC12
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //PC12
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

/*	//PC6 -- KONGv4
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //PC6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);//
*/
	//PE14 -- KONGv0
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //PE14
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOE, &GPIO_InitStruct);//
#endif

	printf("Init TB6612.\r\n");

	stmTb6612_Offset = offset;
}

void stmTb6612_fwdEachMotor(char Motor, int speed)
{
	if(Motor == 'A'){
		AIN1_6612_1;
		AIN2_6612_0 ;
		//analogWrite(PWM, speed);
	}else{
		BIN1_6612_1;
		BIN2_6612_0 ;
		//analogWrite(PWM, speed);
	}
}

void stmTb6612_revEachMotor(char Motor, int speed)
{
	if(Motor == 'A'){
		AIN1_6612_0 ;
		AIN2_6612_1;
		//analogWrite(PWM, speed);
	}else{
		BIN1_6612_0 ;
		BIN2_6612_1;
		//analogWrite(PWM, speed);
	}
}

//========================================================
void stmTb6612_driveEachMotor(char Motor, int speed, int duration)
{
	STBY_6612_1;
	speed = speed * stmTb6612_Offset;
	if (speed>=0) stmTb6612_fwdEachMotor(Motor, speed);
	else stmTb6612_revEachMotor(Motor, -speed);
	delayms(duration);
	STBY_6612_0;
}
//========================================================
void stmTb6612_forwardBothMotor(int speed, int duration)
{
	printf("TB6612>forwardBothMotor\r\n");
	stmTb6612_driveEachMotor('A', speed, duration);
	stmTb6612_driveEachMotor('B', speed, duration);
}

void stmTb6612_backBothMotor(int speed, int duration)
{
	int temp = abs(speed);
	printf("TB6612>backBothMotor\r\n");
	stmTb6612_driveEachMotor('A', -temp,duration);
	stmTb6612_driveEachMotor('B', -temp, duration);
}

void stmTb6612_left(int speed, int duration)
{
	int temp = abs(speed)/2;
	printf("TB6612>LeftMotor\r\n");
	stmTb6612_driveEachMotor('A', -temp, duration); //left motor
	stmTb6612_driveEachMotor('B', temp, duration); //right motor

}

void stmTb6612_right(int speed, int duration)
{
	int temp = abs(speed)/2;
	printf("TB6612>RightMotor\r\n");
	stmTb6612_driveEachMotor('A', temp, duration); //left motor
	stmTb6612_driveEachMotor('B', -temp, duration); //right motor

}

void stmTb6612_brake()
{
		AIN1_6612_1;
		AIN2_6612_1;
		//analogWrite(PWM,0);

		BIN1_6612_1;
		BIN2_6612_1;
		//analogWrite(PWM,0);

}
void stmTb6612_standby()
{
   STBY_6612_0;
}
//=============== main loop ==============
void stmTb6612Loop(){

	int i;
	stmTb6612_Offset = 10;

	stmTb6612_Config(10);

	while(1){
		stmTb6612_forwardBothMotor(1000, 1000);
		delayms(500);
		stmTb6612_standby();
		stmTb6612_backBothMotor(1000,1000);
		delayms(500);
		stmTb6612_standby();
	}
}
