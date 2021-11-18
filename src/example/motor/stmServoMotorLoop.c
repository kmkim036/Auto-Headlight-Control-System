//-- DRV11873 --- GServo Motor Driver
// 50Hz PWM
// 0 : 1msec pulse
// 90 : 1.5msec pulse
// 180: 2msec pulse
// Need 5V power to servo motor

	// PA1 - EN1
	// PA2 - EN2 (nReset)
	// PA3 - EN3 (EN2)
	// PA4 - nSLEEP
	// PA5 - nRESET (EN3)

	//-- STM32F103C8T6
	//[MOT0]
	// IN1(TIM3_CH4) - PB1
	// IN2(TIM3_CH3) - PB0
	// IN3(TIM3_CH2) - PA7 	//IN3(PWM3) - PD14
	//[MOT1]
	// IN1(TIM3_CH1) - PA6	//PD12
	// IN2(TIM2_CH4) - PA3
	// IN3(TIM2_CH3) - PA2 //PD14
	//[MOT2]
	// IN1(TIM4_CH4) - PB9
	// IN2(TIM2_CH2) - PA1
	// IN3(TIM4_CH3) - PB8

#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"

#if (MOTOR_FOR == MOTOR_FOR_SERVO)

#include "math.h"
#include "init.h"
//#include "arm_math.h"

#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

/*
   MOTOR-0                    TIM3           N
    +---+                      +----+      +---+               +---+
 *  | U +----------------|PB1|-|CH4 | PWM  +   +...+       +...+
 *  +---+                      |----+              +---+---+
    +---+                      +----+        F +---+---+
 *  | V +----------------|PB0|-+CH3 | PWM  +...+       +...+
 *  +---+                      |----+                      +---+---+
 *  +---+                      +----+                  +---+---+
 *  | W +----------------|PA7|-|CH2 | PWM    S     +...+       +...+
 *  +---+                      +----+      +---+---+               +
 */

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#if 0
//PE9 = PWM -- TBC
void PWM_init(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_DeInit(TIM1);
	/* TIM1 enable counter */
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
#endif


#define High		0x01
#define Low			0x00

#define PI 			3.1415926535897932384

//main struct
struct _MotServoCntr{
	unsigned short angle;
	uint32_t speed;// = 10;
};

struct _MotServoCntr MotServoCntr;

void rotate_forv(void);
void rotate_back(void);
void MOT_ServoInitPosition(void);
void MOT_ServoControl(unsigned short angle);

#define PWM_PRESCALER 	71 //---> 72MHz/(71+1) => 1MHz
#define PWM_PERIOD 		20000 //1MHz/20000 => 50Hz (20msec)

// PWM out pin config
//-- STM32F103C8T6
//[MOT0]
// IN1(TIM3_CH4) - PB1
// IN2(TIM3_CH3) - PB0
// IN3(TIM3_CH2) - PA7
//[MOT1]
// IN1(TIM3_CH1) - PA6
// IN2(TIM2_CH4) - PA3
// IN3(TIM2_CH3) - PA2
//[MOT2]
// IN1(TIM4_CH4) - PB9
// IN2(TIM2_CH2) - PA1
// IN3(TIM4_CH3) - PB8
void PWM_PINx_init(void) // INx, IN - PWMpins
{
	GPIO_InitTypeDef init_AF;//

	//(1) PA6-TIM3 //PA7, PA6 - TIM3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF;//
	init_AF.GPIO_Speed = GPIO_Speed_10MHz;
	init_AF.GPIO_Pin = GPIO_Pin_6;//	init_AF.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;//
    GPIO_Init(GPIOA, &init_AF);
    //GPIO_PinRemapConfig(GPIO_Remap_TIM3, ENABLE); //GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);//
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
#if 0
    //(2) PA3, PA2, PA1 - TIM2
	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF_PP;
	init_AF.GPIO_Speed = GPIO_Speed_10MHz;
	init_AF.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_1;//
	GPIO_Init(GPIOA, &init_AF);//
    //GPIO_PinRemapConfig(GPIO_Remap_TIM2, ENABLE); //    //GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);//
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//(3) PB1,PB0-TIM3 //--> PD14--------------------------
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF;//
	init_AF.GPIO_Speed = GPIO_Speed_10MHz;
	init_AF.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;//
	GPIO_Init(GPIOB, &init_AF);
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Need???
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//(4) PB9,PB8-TIM4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF;//
	init_AF.GPIO_Speed = GPIO_Speed_10MHz;
	init_AF.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;//
	GPIO_Init(GPIOB, &init_AF);
	//GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
#endif
}
 //PA0
void user_button_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef User_But_ini;//
	User_But_ini.GPIO_Mode = GPIO_Mode_IPD;//GPIO_Mode_IN;//
	User_But_ini.GPIO_Pin = GPIO_Pin_0;//
	User_But_ini.GPIO_Speed = GPIO_Speed_2MHz;
 	//User_But_ini.GPIO_OType = GPIO_OType_PP;
	//User_But_ini.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &User_But_ini);//

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);
	NVIC_EnableIRQ(EXTI0_IRQn);
}
void led15_ini(void)
{
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef LED_pin;
	GPIO_StructInit(&LED_pin);
	LED_pin.GPIO_Pin = GPIO_Pin_15;
	LED_pin.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT;
	GPIO_Init(GPIOD, &LED_pin);
}
//============ TIM =================================================================================
//We will use 3 TIM for geneating PWM pulses to drive 3 BLDC Motors
//and one TIM for measuring the elpased time.
#if 0
void TIM2_ini(void)
{
	TIM_DeInit(TIM2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef timer_init;
	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Period 		= PWM_PERIOD;//(Auto-Reload Register at the next update event) ---> 20kHz
	timer_init.TIM_Prescaler 	= TIM_PRESCALE; // ---> 1MHz (1usec)
	TIM_TimeBaseInit(TIM2, &timer_init);
#if 0
	//channel 1;
	TIM_OCInitTypeDef tim_oc_init1;
	TIM_OCStructInit(&tim_oc_init1);
	tim_oc_init1.TIM_Pulse = 0;
	tim_oc_init1.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init1.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM3, &tim_oc_init1);//
#endif
	//channel 2;
	TIM_OCInitTypeDef tim_oc_init2;
	TIM_OCStructInit(&tim_oc_init2);
	tim_oc_init2.TIM_Pulse = 0;
	tim_oc_init2.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init2.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM2, &tim_oc_init2);//

	//channel 3;
	TIM_OCInitTypeDef tim_oc_init3;
	TIM_OCStructInit(&tim_oc_init3);
	tim_oc_init3.TIM_Pulse = 0;
	tim_oc_init3.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init3.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM2, &tim_oc_init3);//

	//channel 4;
	TIM_OCInitTypeDef tim_oc_init4;
	TIM_OCStructInit(&tim_oc_init4);
	tim_oc_init4.TIM_Pulse = 0;
	tim_oc_init4.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init4.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM2, &tim_oc_init4);//

	TIM_Cmd(TIM2, ENABLE);

	TIM_CtrlPWMOutputs(TIM2, ENABLE);
}
#endif
//[MOT0]
// IN1(TIM3_CH4) - PB1
// IN2(TIM3_CH3) - PB0
// IN3(TIM3_CH2) - PA7
//[MOT1]
// IN1(TIM3_CH1) - PA6 -- We use only this one.

void TIM3_ini(void)
{
	TIM_TimeBaseInitTypeDef timer_init;

	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//Set 50Hz PWM
	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Prescaler = PWM_PRESCALER; // ---> 72MHz/(71+1) => 1MHz
	timer_init.TIM_Period = PWM_PERIOD;    //1MHz/20000 => 50Hz (20msec)
	TIM_TimeBaseInit(TIM3, &timer_init);

	//channel 1; -- PA6
	TIM_OCInitTypeDef tim_oc_init1;
	TIM_OCStructInit(&tim_oc_init1);
	tim_oc_init1.TIM_Pulse = 0;
	tim_oc_init1.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init1.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM3, &tim_oc_init1);//
#if 0
	//channel 2: M0-W
	TIM_OCInitTypeDef tim_oc_init2;
	TIM_OCStructInit(&tim_oc_init2);
	tim_oc_init2.TIM_Pulse = 0;
	tim_oc_init2.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init2.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM3, &tim_oc_init2);//

	//channel 3: M0-V
	TIM_OCInitTypeDef tim_oc_init3;
	TIM_OCStructInit(&tim_oc_init3);
	tim_oc_init3.TIM_Pulse = 0;
	tim_oc_init3.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init3.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM3, &tim_oc_init3);//

	//channel 4: M0-U
	TIM_OCInitTypeDef tim_oc_init4;
	TIM_OCStructInit(&tim_oc_init4);
	tim_oc_init4.TIM_Pulse = 0;
	tim_oc_init4.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init4.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM3, &tim_oc_init4);//
#endif
	TIM_Cmd(TIM3, ENABLE);

	//TIM_CtrlPWMOutputs(TIM3, ENABLE); //TIMx: where x can be 1, 8, 15, 16 or 17 to select the TIMx peripheral.
}
#if 0
void TIM4_ini(void)
{
	TIM_DeInit(TIM4);

	//TIM4 Clock Enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//Config
	TIM_TimeBaseInitTypeDef timer_init;
	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Period = PWM_PERIOD;// ---> 20kHz
	timer_init.TIM_Prescaler = TIM_PRESCALE; // ---> 36 MHz (Half of 72 Mhz)
	TIM_TimeBaseInit(TIM4, &timer_init);
#if 0
	//channel 1;
	TIM_OCInitTypeDef tim_oc_init1;
	TIM_OCStructInit(&tim_oc_init1);
	tim_oc_init1.TIM_Pulse = 0;
	tim_oc_init1.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init1.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM4, &tim_oc_init1);//

	//channel 2;
	TIM_OCInitTypeDef tim_oc_init2;
	TIM_OCStructInit(&tim_oc_init2);
	tim_oc_init2.TIM_Pulse = 0;
	tim_oc_init2.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init2.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM4, &tim_oc_init2);//
#endif
	//channel 3;
	TIM_OCInitTypeDef tim_oc_init3;
	TIM_OCStructInit(&tim_oc_init3);
	tim_oc_init3.TIM_Pulse = 0;
	tim_oc_init3.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init3.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM4, &tim_oc_init3);//

	//channel 4;
	TIM_OCInitTypeDef tim_oc_init4;
	TIM_OCStructInit(&tim_oc_init4);
	tim_oc_init4.TIM_Pulse = 0;
	tim_oc_init4.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init4.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM4, &tim_oc_init4);//

	TIM_Cmd(TIM4, ENABLE);
	//TIM_CtrlPWMOutputs(TIM4, ENABLE);
}

//== TIM1 is used as Tick Counter to measure the elapsed time.
void TIM1_ini(void) /// was TIM5(No in c8T6) Time measuring timer (internal usage)
{
	TIM_DeInit(TIM1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	//base settings
	TIM_TimeBaseInitTypeDef TIM1_Base;
	TIM1_Base.TIM_Period = 0xFFFF;
  	TIM1_Base.TIM_Prescaler = 72-1;//84-1; // 1MHz
	TIM1_Base.TIM_ClockDivision = TIM_CKD_DIV1 ; //
	TIM1_Base.TIM_CounterMode = TIM_CounterMode_Up;
	TIM1_Base.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseInit(TIM1, &TIM1_Base);
	TIM_Cmd(TIM1, ENABLE);

	printf("TIM1 prescaler=%u\r\n",TIM_GetPrescaler(TIM1));
}
#endif

void MOT_ServoControl(unsigned short angle)
{
	unsigned short ccr;
	MotServoCntr.angle = angle;
	// -------------->CCR      APR
	// |               +--------+
	// |               |        |
	// +---------------+        +-...
	// |<-------- Period ------>|
	// ccr = 19000 - 1000*(angle)/180
	//e.g. 0 degree = 1msec --> ccr = 19000 - 0 = 19000 --> 1msec
	//e.g. 90 degree = 1.5msec --> ccr = 19000 - 500 = 18500 --> 1.5msec
	//e.g. 180 degree = 2msec --> ccr = 19000 - 1000 = 18000 --> 2msec
	// i.e., CCR for Duty...

	ccr = (unsigned short)((PWM_PERIOD-1000) - 1000*(angle)/180);

	//MOT0
	//TIM3->CCR4 = PWM_PERIOD - (unsigned short)(MotServoCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	//TIM3->CCR3 = PWM_PERIOD - (unsigned short)(MotServoCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	//TIM3->CCR2 = PWM_PERIOD - (unsigned short)(MotServoCntr.Vinv3*PWM_PERIOD/VMOT)  ;

	//TIM3->CCR4 = (unsigned short)(MotServoCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	//TIM3->CCR3 = (unsigned short)(MotServoCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	//TIM3->CCR2 = (unsigned short)(MotServoCntr.Vinv3*PWM_PERIOD/VMOT)  ;

	//MOT1 - TIM3-CH1


	TIM3->CCR1 = PWM_PERIOD - ccr ;
  	//TIM2->CCR4 = (unsigned short)(MotServoCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	//TIM2->CCR3 = (unsigned short)(MotServoCntr.Vinv3*PWM_PERIOD/VMOT)  ;
  	//MOT2
	//TIM4->CCR4 = (unsigned short)(MotServoCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	//TIM2->CCR2 = (unsigned short)(MotServoCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	//TIM4->CCR3 = (unsigned short)(MotServoCntr.Vinv3*PWM_PERIOD/VMOT)  ;

}


//==================================================================================================================
//CCR1 - A, CCR2 - B, CCR3 - C
void MOT_ServoInitPosition(void) // establishing zero position, d-axis directed to A winding, theta = 90
{
	MOT_ServoControl(90);
}

int MOT_ServoLoop(void)
{
	 PWM_PINx_init();

	 //TIM1_ini(); // Delay timer//TIM5_ini(); // Delay timer
	 //TIM2_ini(); // PWM timer
	 TIM3_ini(); // PWM timer
	 //TIM4_ini(); // PWM timer

	 MOT_ServoInitPosition();

	 while(1)
	 {
  		printf("angle=%u\r\n",MotServoCntr.angle);
  		MOT_ServoControl(MotServoCntr.angle);

		delayms(100);

		if(MotServoCntr.angle < 180){
			MotServoCntr.angle += 10;
		}else {
			MotServoCntr.angle = 0;
		}
	}
}

void Servo_control(unsigned short angle)
{
	MOT_ServoControl(angle);
}

void Servo_Setup()
{
	PWM_PINx_init();
	TIM3_ini();
}


#endif


#endif
