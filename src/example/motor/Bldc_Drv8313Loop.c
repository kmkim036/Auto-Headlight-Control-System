//-- DRV8313 --- G
//github :
//https://github.com/ViktorAnchutin/BLDC_CONTROL/tree/a7294cdadcd82061b336d9b1973c64421cd7e260
//3 techniques have been implemented:
//1) Position control with sinusoidal commutation -sinus_control_V2
//2) Position control with simplified FOC (no current sensing) - FOC
//3) Position control with special sinusoidal commutation(vectol angle limitation) - combined_control_V3

	// PA1 - EN1
	// PA2 - EN2 (nReset)
	// PA3 - EN3 (EN2)
	// PA4 - nSLEEP
	// PA5 - nRESET (EN3)

	//-- STM32F103C8T6 with MPU6050
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

#if (MOTOR_FOR == MOTOR_FOR_DRV8313)
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

/*[3 poles BLDC Principle] CounterClockwise case.
 *
 *             [U]
 *               S
 *              /
 *            /                              S   S  S    N   N N
 *          N                                /   |   \   /   |  \
 *      [V]           [W]                   N    N    N S    S   S
 *
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

 * i.e., each phase pulse has a sequence of  {1  1  0 -1 -1  0} with 120 degree offset.
 *       On either N or S pole is faced U/V/W, its phase pulse is floated(0).
 *
 *
 * [24 poles BLDC] - It can run more smoothly.
 * Actually, we consider only 12 poles. The other 12 poles are reversed.
 * Thus we generate 3 sinusoidal current wave with 120 degree apart each other.
 * This waves can be generated using PWM with variable duty.
 *
 *   MOTOR-0                    TIM3           N
    +---+                      +----+       /-\         /-\       +---+
 *  | U +----------------|PB1|-|CH4 | PWM  +   +...+   +   +...+
 *  +---+                      |----+               \-/     \-
    +---+                      +----+        F +---+---+
 *  | V +----------------|PB0|-+CH3 | PWM  +...+       +...+
 *  +---+                      |----+                      +---+---+
 *  +---+                      +----+                  +---+---+
 *  | W +----------------|PA7|-|CH2 | PWM    S     +...+       +...+
 *  +---+                      +----+      +---+---+               +
 *
 * These sinusoidal waves can be generated using PWM with variable duty.
 *  +-----+ +----+ +---+ +--+ +-+
 *  |     | |    | |   | |  | | |
 * -+     +-+    +-+   +-+  +-+ +-+ +-+  +-+   +-+     +- .......
 *                                | | |  | |   | |     |
 *                                +-+ +--+ +---+ +-----+
 *
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
#define average		100
#define POLE_PAIRS 	11 //2206: 22 means the number of poles

#define VMOT 		12 //12V VMOT
#define VMOT_HALF    6

#define PI 			3.1415926535897932384
#define CALIBRATION 0
//#define TIM_PRESCALE (36-1)  //--> 1MHz, 1usec
//#define PWM_PERIOD 	 (2100-1) //--> 2.1 msec
#define TIM_PRESCALE (360-1)  //--> 10KHz, 10msec
#define PWM_PERIOD 	 (21-1) //--> 2.1 msec

//timer_init.TIM_Prescaler = 8-1; // ---> 72/8 MHz //42MHz
//timer_init.TIM_Prescaler = 4-1; // ---> 72/4 MHz //42MHz

//For averaging
#define window_ADC 	200 //2000
#define window_Roll	100//10000
#define window      100
#define FILTER_BUF 	250//2500
typedef struct{
	unsigned char filled,
				  error,
				  init;
	float buffer[FILTER_BUF],
					sum,
					output;
	unsigned short counter1, counter2;
	float input;
} moving_average_type;

//main struct
struct _GimbalCntr{
	unsigned short deltaTime;
	unsigned char mode;
	float angle, angle_error, angle_test, angle1, angle_error_mem_in1, angle_error_mem_in2, angle_error_mem_in3, angle_error_mem_out1, angle_error_mem_out2, angle_error_mem_out3;
	float des_val,des_val1;
	uint32_t t1, t2, t3, t4,dt22, p, timex, t1_IMU, t2_IMU, dt_IMU;
	float voltage, current;
	float sin_x, cos_x, tv_g, t_g, t_d;

#if 0
	unsigned char ADC_started;
	// moving average filter variables
	unsigned char filled_ADC;
	uint16_t i_ADC; // counter for filling in
	float data_ADC;
	float arr_ADC[window_ADC],  angle_average, a_i_ADC,  des_val_raw; // variables for first order moving average
	uint16_t k_ADC; // counter
	float ADC_average;
	unsigned char flag_ADC;
	uint32_t sum_ADC;

	//moving average Roll
	unsigned char filled_Roll;
	uint32_t i_Roll; // counter for filling in
	float data_Roll;
	uint32_t arr_Roll[window_Roll],  Roll_average, a_i_Roll; // variables for first order moving average
	uint32_t k_Roll; // counter
	unsigned char flag_Roll;
	uint32_t sum_Roll;
	uint16_t USART_count;
	unsigned char IMU_data_ready;
	uint16_t IMU_Recieve_Buf[11];
	unsigned char IMU_count;
	uint16_t USART_test_rec;
	unsigned char IMU_data;
	float Pitch, Roll, Yaw;
	float Roll_raw, roll_sine, roll_cos, Roll_raw_test, roll_sine_test, roll_cos_test, Roll_test, Roll_cor;
	moving_average_type ADC_filter;
#endif
	unsigned char first_ini;
	int change;

	float Vq;
	float Vd;
	float Va, Vb, Vc, Va_1, Vb_1, Vc_1;
	float thetaRad, theta_elec_degrees;
	float Vinv1,Vinv2, Vinv3;
	float angle_init , error_angle_last, sine_init, cos_init;

	float error_in_proc, er_mem, angle_mem,integral;

	unsigned char started;
	float step, step2;

	float thetta_vector;
	uint32_t speed;// = 10;

	//extern float Speed;
	//extern float alpha;
};

struct _GimbalCntr GimbalCntr;

void Use_HalfBridge(int num, int Transistor, int power); // num - 1,2,3;  Switch - High or Low; power - duty cycle
void dead_time(uint32_t time);
void Change_winding(void);
void rotate_forv(void);
void rotate_back(void);
void sine_PWM_ini(void);
//float average_angle(uint16_t raw_angle);
void FOC_InitPosition(void);
void FOC(float angle, float error_angle, float K_p, float K_d, float K_I, uint32_t dt);
void sinus_control(float des_val_);
void sinus_control_V2(float error_angle, float V, float K, float step_max);
void combined_control_V3(float angle, float error_angle, float V, float K, float step_max);

extern float cosf(float xRad); //math.h

// PWM out pin config
//-- STM32F103C8T6 with MPU6050
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

	//(1) PA7(M0/W), PA6(M1/U) - TIM3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF;//
	init_AF.GPIO_Speed = GPIO_Speed_10MHz;
	init_AF.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;//
    GPIO_Init(GPIOA, &init_AF);
    //GPIO_PinRemapConfig(GPIO_Remap_TIM3, ENABLE); //GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);//
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    //(2) PA3(M1/V), PA2(M1/W), PA1(M2/V) - TIM2
	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF_PP;
	init_AF.GPIO_Speed = GPIO_Speed_10MHz;
	init_AF.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_1;//
	GPIO_Init(GPIOA, &init_AF);//
    //GPIO_PinRemapConfig(GPIO_Remap_TIM2, ENABLE); //    //GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);//
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//(3) PB1(M0/U),PB0(M0/V)-TIM3 //--> PD14--------------------------
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF;//
	init_AF.GPIO_Speed = GPIO_Speed_10MHz;
	init_AF.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;//
	GPIO_Init(GPIOB, &init_AF);
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Need???
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//(4) PB9(M2/U),PB8(M2/W)-TIM4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF;//
	init_AF.GPIO_Speed = GPIO_Speed_10MHz;
	init_AF.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;//
	GPIO_Init(GPIOB, &init_AF);
	//GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}
/*
//Not Used - Always Enabled
void ENx_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef ENx_init;
	GPIO_StructInit(&ENx_init);
	ENx_init.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5  ;
	ENx_init.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_Mode_OUT;
	GPIO_Init(GPIOA,&ENx_init);
}
*/
void ADC_initt(void) // PB1
{
	/*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitTypeDef GPIO_AI;
	GPIO_StructInit(&GPIO_AI);
	GPIO_AI.GPIO_Pin = GPIO_Pin_1;
	GPIO_AI.GPIO_Mode = GPIO_Mode_AIN;//GPIO_Mode_AN;
	//GPIO_AI.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_AI.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_AI);

	ADC_CommonInitTypeDef ADC_init;
	ADC_InitTypeDef ADC_InitStructure;

	ADC_StructInit(&ADC_InitStructure);
	ADC_CommonStructInit(&ADC_init);
	ADC_CommonInit (&ADC_init);
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_480Cycles);
	ADC_SoftwareStartConv(ADC1);
	*/

}

//Not Used
#if 0
void USART_2_init(void) //PD6,PD5
{
	/*
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_InitTypeDef USART2_ini;//
	USART2_ini.GPIO_Mode = GPIO_Mode_AF;//
	USART2_ini.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;//
	USART2_ini.GPIO_Speed = GPIO_Speed_2MHz;
	USART2_ini.GPIO_OType = GPIO_OType_PP;
	USART2_ini.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &USART2_ini);//

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);//
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);//

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE) ;

	USART_InitTypeDef USART2_user;
	USART2_user.USART_BaudRate= 115200;
	USART2_user.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART2_user.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART2_user.USART_Parity = USART_Parity_No;
	USART2_user.USART_StopBits = USART_StopBits_1;
	USART2_user.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART2_user);
	VIC_EnableIRQ(USART2_IRQn);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	*/
 }
#endif

//Not used
#if 0
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
#endif

void led14_ini(void) //PC14
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef LED_pin;
	GPIO_StructInit(&LED_pin);
	LED_pin.GPIO_Pin = GPIO_Pin_14;
	LED_pin.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &LED_pin);
}
/*
 //Delay
uint32_t time1;
void myDelay_microsec(uint32_t delay)
{
	time1 = TIM5->CNT;
	while( ((TIM5->CNT) - time1) < delay) {}
}

void myDelay_ms(uint32_t delay)
{
	time1 = TIM5->CNT;
	while( ((TIM5->CNT) - time1) < 1000*delay) {}
}

void USATRT2_SendStr(char* str_p)
{
	uint16_t i = 0;
	while(str_p[i]!=0)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET){}
		USART_SendData(USART2,str_p[i]);
		i++;
	}
}
*/
/* -------- NOT USED -------------
void dead_time(uint32_t time)
{
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	GPIO_ResetBits(GPIOA, GPIO_Pin_1|GPIO_Pin_2 | GPIO_Pin_5);
	Delay(time);
}
void Use_HalfBridge(int num, int Transistor, int power) // num - 1,2,3;  Switch - High or Low; power - duty cycle from 0 to 142(or any other which was chosen)
{
	if(num ==1)
	{
		if(Transistor == High)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_5);		//IN1 = 1
			TIM4->CCR1 = power;
		}
		if(Transistor == Low)
		{
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);		//IN1 = 0
			TIM4->CCR1 = 1000;
		}
	}

	else if(num ==2)
	{
		if(Transistor == High)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_1);		//IN2 = 1
			TIM4->CCR2 = power;
		}
		if(Transistor == Low)
		{
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);		//IN2 = 0
			TIM4->CCR2 = 1000;
		}
	}

	else if(num ==3)
	{
		if(Transistor == High)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_2);		//IN3 = 1
			TIM4->CCR3 = power;
		}
		if(Transistor == Low)
		{
			GPIO_ResetBits(GPIOA, GPIO_Pin_2);		//IN13= 0
			TIM4->CCR3 = 1000;
		}
	}

}

//Not Used
void Change_winding(void)
{
	dead_time(1000000);
	if (change == 0)
	{
	Use_HalfBridge(2, High, 71); // num - 1,2,3;  Transistor - High or Low , power from 0 to 142
	Use_HalfBridge(1, Low, 142); // num - 1,2,3;  Transistor - High or Low , power from 0 to 142
	change = 1;
	}
	else
	{
	Use_HalfBridge(1, High, 71); // num - 1,2,3;  Transistor - High or Low , power from 0 to 142
	Use_HalfBridge(3, Low, 142); // num - 1,2,3;  Transistor - High or Low , power from 0 to 142
	change = 0;
	}

}
void try_to_turn(void)
{
	Use_HalfBridge(1, High, 71);
	Use_HalfBridge(2, Low, 142);

}
---------------------------------*/
#if 0
void Set_nRes_nSleep(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_4|GPIO_Pin_5);
}
void Set_ENx(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3); // EN1,2,3 to 1 enable all half-bridges
}
#endif
//============ TIM =================================================================================
//We will use 3 TIM for geneating PWM pulses to drive 3 BLDC Motors
//and one TIM for measuring the elpased time.

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
//[MOT0]
// IN1(TIM3_CH4) - PB1
// IN2(TIM3_CH3) - PB0
// IN3(TIM3_CH2) - PA7
//[MOT1]
// IN1(TIM3_CH1) - PA6

void TIM3_ini(void)
{
	TIM_TimeBaseInitTypeDef timer_init;

	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Period = PWM_PERIOD;// ---> 20kHz
	//timer_init.TIM_Prescaler = 2-1; // --->36MHz// 42 MHz
	//timer_init.TIM_Prescaler = 4-1; // ---> 72/4 MHz //42MHz
	timer_init.TIM_Prescaler = TIM_PRESCALE; // ---> 72/8 MHz //42MHz
	TIM_TimeBaseInit(TIM3, &timer_init);

	//channel 1;
	TIM_OCInitTypeDef tim_oc_init1;
	TIM_OCStructInit(&tim_oc_init1);
	tim_oc_init1.TIM_Pulse = 0;
	tim_oc_init1.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init1.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM3, &tim_oc_init1);//

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

	TIM_Cmd(TIM3, ENABLE);

	//TIM_CtrlPWMOutputs(TIM3, ENABLE); //TIMx: where x can be 1, 8, 15, 16 or 17 to select the TIMx peripheral.
}

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

//==================================================================================================================
//CCR1 - A, CCR2 - B, CCR3 - C
void FOC_InitPosition(void) // establishing zero position, d-axis directed to A winding, theta = 90
{
	/*
	if(CALIBRATION)
	{
	Vq=3;

	Va_1 = cosf(0);//cos(theta         );
	Vb_1 = cosf(0 - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 ); //2*Pi/3
	Vc_1 = cosf(0 + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);

	Va = Va_1 * Vq; // projection calculation of Vq into A phase
	Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	Vc = Vc_1 * Vq; // projection calculation of Vq into C phase

	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + VMOT/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = VMOT/2 (with sine PWM)
	Vinv3 = Vc + 6;

	TIM4->CCR1 = (uint32_t)(Vinv1*PWM_PERIOD/VMOT)  ;
  TIM4->CCR2 = (uint32_t)(Vinv2*PWM_PERIOD/VMOT)  ;
  TIM4->CCR3 = (uint32_t)(Vinv3*PWM_PERIOD/VMOT)  ;

	myDelay_ms(1000);

	angle_init = CQ_average_angle();

	}
	else

	{

	// init angle was calculated once. Now it is used like starting point for electrical angles and engine does not need position initialization

	angle_init = 238.066;

	} */

	GimbalCntr.angle_init = 238.066;
}



//field-oriented control (FOC), is a variable-frequency drive (VFD) control method in which the stator
//currents of a three-phase AC electric motor are identified as two orthogonal components that can
//be visualized with a vector. -- Vector Control
void FOC(float angle, float error_angle, float K_p, float K_d, float K_I, uint32_t deltaTime)
{
	GimbalCntr.theta_elec_degrees = ((angle - GimbalCntr.angle_init)*POLE_PAIRS + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90
	GimbalCntr.thetaRad = GimbalCntr.theta_elec_degrees*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians
	GimbalCntr.Vd = 0;
	if(error_angle > 180)
	{
		error_angle = 360 - error_angle;
		error_angle = - error_angle;
	}
	if(error_angle < -180)
	{
		error_angle = 360 + error_angle;
		//error_angle = - error_angle;
	}
	GimbalCntr.error_in_proc = error_angle;
	if ((error_angle > 100)||(error_angle< -100))
	{
		GimbalCntr.er_mem = error_angle;
		GimbalCntr.angle_mem = angle;
	}
	GimbalCntr.integral = deltaTime*0.000001*GimbalCntr.error_angle_last+GimbalCntr.integral;
	GimbalCntr.Vq = K_p*error_angle ;//+ ((error_angle - error_angle_last)/(dt*0.000001))*K_d + integral*K_I; //Speed; //
	GimbalCntr.error_angle_last = error_angle;
	if(GimbalCntr.Vq < -VMOT_HALF) GimbalCntr.Vq = -VMOT_HALF; // 6V = VMOT/2 , voltage limitation
	if(GimbalCntr.Vq > VMOT_HALF) GimbalCntr.Vq = VMOT_HALF;

	//3-different phase signals for U,V,W
	GimbalCntr.Va_1 = cosf(GimbalCntr.thetaRad);
	GimbalCntr.Vb_1 = cosf(GimbalCntr.thetaRad - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 );	// - 2*Pi/3 : -120 degree
	GimbalCntr.Vc_1 = cosf(GimbalCntr.thetaRad + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);	// + 2*Pi/3 : +120 degree

	GimbalCntr.Va = GimbalCntr.Va_1 * GimbalCntr.Vq; // projection calculation of Vq into A phase
	GimbalCntr.Vb = GimbalCntr.Vb_1 * GimbalCntr.Vq; // projection calculation of Vq into B phase
	GimbalCntr.Vc = GimbalCntr.Vc_1 * GimbalCntr.Vq; // projection calculation of Vq into C phase

	GimbalCntr.Vinv1 = GimbalCntr.Va + VMOT_HALF; // Obtaining value for inverter, +6 because Vinv relates with V_phase as Vinv = Vphase + VMOT/2 in order to avoid negative values for invertor voltage.
	GimbalCntr.Vinv2 = GimbalCntr.Vb + VMOT_HALF; // should also be taken into account that Vphase(max) = VMOT/2 (with sine PWM)
	GimbalCntr.Vinv3 = GimbalCntr.Vc + VMOT_HALF;

	// -------------->CCR      APR
	// |               +--------+
	// |               |        |
	// +---------------+        +-...
	// |<-------- Period ------>|
	//
	// i.e., CCR for Duty... Ã­ËœÂ¹Ã¬â€¹Å“ ÃªÂ·Â¸ Ã«Â°ËœÃ«Å’â‚¬? Ã­ï¿½Â° ÃªÂ°â€™Ã¬ï¿½Â¸ ÃªÂ²Â½Ã¬Å¡Â°, CCRÃªÂ°â€™Ã¬ï¿½Â´ Ã¬Å¾â€˜Ã¬â€¢â€žÃ¬â€¢Â¼ Ã«ï¿½ËœÃ¬Â§â‚¬ Ã¬â€¢Å Ã«Å â€�ÃªÂ°â‚¬?
	//MOT0
	//TIM3->CCR4 = PWM_PERIOD - (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	//TIM3->CCR3 = PWM_PERIOD - (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	//TIM3->CCR2 = PWM_PERIOD - (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
  	TIM3->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM3->CCR3 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM3->CCR2 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
  	//MOT1
	TIM3->CCR1 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR4 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
  	//MOT2
	TIM4->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR2 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM4->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;

}
void sinus_control(float des_val_)
{
	GimbalCntr.theta_elec_degrees = ((des_val_)*POLE_PAIRS + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90
	GimbalCntr.thetaRad = GimbalCntr.theta_elec_degrees*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians
	GimbalCntr.Vq = VMOT_HALF;

	GimbalCntr.Va_1 = cosf(GimbalCntr.thetaRad);//cos(theta);
	GimbalCntr.Vb_1 = cosf(GimbalCntr.thetaRad - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	GimbalCntr.Vc_1 = cosf(GimbalCntr.thetaRad + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);

	GimbalCntr.Va = GimbalCntr.Va_1 * GimbalCntr.Vq; // projection calculation of Vq into A phase
	GimbalCntr.Vb = GimbalCntr.Vb_1 * GimbalCntr.Vq; // projection calculation of Vq into B phase
	GimbalCntr.Vc = GimbalCntr.Vc_1 * GimbalCntr.Vq; // projection calculation of Vq into C phase


	GimbalCntr.Vinv1 = GimbalCntr.Va + VMOT_HALF; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + VMOT/2 in order to avoid negative values for invertor voltage.
	GimbalCntr.Vinv2 = GimbalCntr.Vb + VMOT_HALF; // should also be taken into account that Vphase(max) = VMOT/2 (with sine PWM)
	GimbalCntr.Vinv3 = GimbalCntr.Vc + VMOT_HALF;
	printf("V1=%s\r\n",float2str(GimbalCntr.Vinv1));
	printf("V2=%s\r\n",float2str(GimbalCntr.Vinv2));
	printf("V3=%s\r\n",float2str(GimbalCntr.Vinv3));
	// Vinx_max = 12V, PWM = Vinv*PWM_PERIOD/Vinv_max
	//MOT0
	TIM3->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM3->CCR3 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM3->CCR2 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
  	//MOT1
	TIM3->CCR1 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR4 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
  	//MOT2
	TIM4->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR2 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM4->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;

}


void sinus_control_V2(float error_angle, float V, float K, float step_max)
{
	if(error_angle > 180)
	{
		error_angle = 360 - error_angle;
		error_angle = - error_angle;
	}
	if(error_angle < -180)
	{
		error_angle = 360 + error_angle;
		//error_angle = - error_angle;
	}

	GimbalCntr.error_in_proc = error_angle;
	if ((error_angle > 100)||(error_angle< -100))
	{
		GimbalCntr.er_mem = error_angle;
		GimbalCntr.angle_mem = GimbalCntr.angle;
	}
	if(!GimbalCntr.started)
	{
		GimbalCntr.thetaRad = 0;
		GimbalCntr.Va_1 = cosf(GimbalCntr.thetaRad);//cos(theta         );
		GimbalCntr.Vb_1 = cosf(GimbalCntr.thetaRad - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
		GimbalCntr.Vc_1 = cosf(GimbalCntr.thetaRad + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
		GimbalCntr.Va = GimbalCntr.Va_1 * V; // projection calculation of Vq into A phase
		GimbalCntr.Vb = GimbalCntr.Vb_1 * V; // projection calculation of Vq into B phase
		GimbalCntr.Vc = GimbalCntr.Vc_1 * V; // projection calculation of Vq into C phase

		GimbalCntr.Vinv1 = GimbalCntr.Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + VMOT/2 in order to avoid negative values for invertor voltage.
		GimbalCntr.Vinv2 = GimbalCntr.Vb + 6; // should also be taken into account that Vphase(max) = VMOT/2 (with sine PWM)
		GimbalCntr.Vinv3 = GimbalCntr.Vc + 6;

		printf("V1,V2,V3=%u,%u,%u\r\n", GimbalCntr.Vinv1,GimbalCntr.Vinv2,GimbalCntr.Vinv3);
		// Vinx_max = 12V, PWM = Vinv*PWM_PERIOD/Vinv_max
		//MOT0
		TIM3->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
	  	TIM3->CCR3 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
	  	TIM3->CCR2 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
/*	  	//MOT1
		TIM3->CCR1 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
	  	TIM2->CCR4 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
	  	TIM2->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
	  	//MOT2
		TIM4->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
	  	TIM2->CCR2 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
	  	TIM4->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
*/
		GimbalCntr.started =1;
	}

	GimbalCntr.step = error_angle*K;
	if(GimbalCntr.step>step_max) { GimbalCntr.step=step_max;}
	if(GimbalCntr.step< -step_max) {GimbalCntr.step=-step_max; }
	GimbalCntr.thetaRad = GimbalCntr.thetaRad + GimbalCntr.step;

	GimbalCntr.Va_1 = cosf(GimbalCntr.thetaRad);//cos(theta         );
	GimbalCntr.Vb_1 = cosf(GimbalCntr.thetaRad - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	GimbalCntr.Vc_1 = cosf(GimbalCntr.thetaRad + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);

	GimbalCntr.Va = GimbalCntr.Va_1 * V; // projection calculation of Vq into A phase
	GimbalCntr.Vb = GimbalCntr.Vb_1 * V; // projection calculation of Vq into B phase
	GimbalCntr.Vc = GimbalCntr.Vc_1 * V; // projection calculation of Vq into C phase

	GimbalCntr.Vinv1 = GimbalCntr.Va + VMOT_HALF; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + VMOT/2 in order to avoid negative values for invertor voltage.
	GimbalCntr.Vinv2 = GimbalCntr.Vb + VMOT_HALF; // should also be taken into account that Vphase(max) = VMOT/2 (with sine PWM)
	GimbalCntr.Vinv3 = GimbalCntr.Vc + VMOT_HALF;

	// Vinx_max = 12V, PWM = Vinv*PWM_PERIOD/Vinv_max
	//MOT0
	TIM3->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM3->CCR3 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM3->CCR2 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
 /* 	//MOT1
	TIM3->CCR1 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR4 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
  	//MOT2
	TIM4->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR2 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM4->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
*/
}

void combined_control_V3(float angle, float error_angle, float V, float K, float step_max)
{
	if(error_angle > 180)
	{
		error_angle = 360 - error_angle;
		error_angle = - error_angle;
	}
	if(error_angle < -180)
	{
		error_angle = 360 + error_angle;
		//error_angle = - error_angle;
	}


	GimbalCntr.error_in_proc = error_angle;
	if ((error_angle > 100)||(error_angle< -100))
	{
		GimbalCntr.er_mem = error_angle;
		GimbalCntr.angle_mem = angle;
	}

	GimbalCntr.theta_elec_degrees = ((angle - GimbalCntr.angle_init)*POLE_PAIRS + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90
	GimbalCntr.thetaRad = GimbalCntr.theta_elec_degrees*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians
	 if(!GimbalCntr.started)
	{
		 GimbalCntr.thetta_vector = GimbalCntr.thetaRad - PI/2;// aim voltage vector to d axis
		 GimbalCntr.Va_1 = cosf(GimbalCntr.thetta_vector);//cos(theta         );
		 GimbalCntr.Vb_1 = cosf(GimbalCntr.thetta_vector - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 ); //2*Pi/3
		 GimbalCntr.Vc_1 = cosf(GimbalCntr.thetta_vector + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
		 GimbalCntr.Va = GimbalCntr.Va_1 * V; // projection calculation of Vq into A phase
		 GimbalCntr.Vb = GimbalCntr.Vb_1 * V; // projection calculation of Vq into B phase
		 GimbalCntr.Vc = GimbalCntr.Vc_1 * V; // projection calculation of Vq into C phase

		 GimbalCntr.Vinv1 = GimbalCntr.Va + VMOT_HALF; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + VMOT/2 in order to avoid negative values for invertor voltage.
		 GimbalCntr.Vinv2 = GimbalCntr.Vb + VMOT_HALF; // should also be taken into account that Vphase(max) = VMOT/2 (with sine PWM)
		 GimbalCntr.Vinv3 = GimbalCntr.Vc + VMOT_HALF;
		// Vinx_max = 12V, PWM = Vinv*PWM_PERIOD/Vinv_max
		//MOT0
		TIM3->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
	  	TIM3->CCR3 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
	  	TIM3->CCR2 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
	  	//MOT1
		TIM3->CCR1 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
	  	TIM2->CCR4 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
	  	TIM2->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
	  	//MOT2
		TIM4->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
	  	TIM2->CCR2 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
	  	TIM4->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;

	  	GimbalCntr.started =1;
	}
	 GimbalCntr.step = error_angle*K;
//	step = error_angle*0.02;
	//step2 = step;
	//theta_elec_degrees = ((err)*11 + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90
	if(GimbalCntr.step > step_max) { GimbalCntr.step=step_max;}
	if(GimbalCntr.step < -step_max) {GimbalCntr.step=-step_max; }

	GimbalCntr.thetta_vector = GimbalCntr.thetta_vector + GimbalCntr.step;
	/*
	// tranlating to unit circle for visual determination of angle of loosing synchronisation

	//sin_x = arm_sin_f32(thetta_vector);
	//cos_x = cosf(thetta_vector);
	tv_g = thetta_vector*57.295779513082320876798154814105 ;//atan2(sin_x, cos_x)*57.295779513082320876798154814105 ;

	//sin_x = arm_sin_f32(theta);
	//cos_x = cosf(theta);
	t_g = theta*57.295779513082320876798154814105 ;//atan2(sin_x, cos_x)*57.295779513082320876798154814105 ;

	t_d = tv_g - t_g;

	*/
	if((GimbalCntr.thetta_vector - GimbalCntr.thetaRad) < -PI )
	{
		GimbalCntr.thetta_vector = GimbalCntr.thetaRad - PI ;//- 2*PI;
	}
	if((GimbalCntr.thetta_vector - GimbalCntr.thetaRad)> 0 )
	{
		GimbalCntr.thetta_vector = GimbalCntr.thetaRad;
	}
	//if(thetta_vector < - theta) thetta_vector = - theta;
	GimbalCntr.Va_1 = cosf(GimbalCntr.thetta_vector);//cos(theta         );
	GimbalCntr.Vb_1 = cosf(GimbalCntr.thetta_vector - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 );
	GimbalCntr.Vc_1 = cosf(GimbalCntr.thetta_vector + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);

	GimbalCntr.Va = GimbalCntr.Va_1 * V; // projection calculation of Vq into A phase
	GimbalCntr.Vb = GimbalCntr.Vb_1 * V; // projection calculation of Vq into B phase
	GimbalCntr.Vc = GimbalCntr.Vc_1 * V; // projection calculation of Vq into C phase

	GimbalCntr.Vinv1 = GimbalCntr.Va + VMOT_HALF; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + VMOT/2 in order to avoid negative values for invertor voltage.
	GimbalCntr.Vinv2 = GimbalCntr.Vb + VMOT_HALF; // should also be taken into account that Vphase(max) = VMOT/2 (with sine PWM)
	GimbalCntr.Vinv3 = GimbalCntr.Vc + VMOT_HALF;

	// Vinx_max = 12V, PWM = Vinv*PWM_PERIOD/Vinv_max
	//MOT0
	TIM3->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM3->CCR3 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM3->CCR2 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
  	//MOT1
	TIM3->CCR1 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR4 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;
  	//MOT2
	TIM4->CCR4 = (unsigned short)(GimbalCntr.Vinv1*PWM_PERIOD/VMOT)  ;
  	TIM2->CCR2 = (unsigned short)(GimbalCntr.Vinv2*PWM_PERIOD/VMOT)  ;
  	TIM4->CCR3 = (unsigned short)(GimbalCntr.Vinv3*PWM_PERIOD/VMOT)  ;

}

//======= main
//-- STM32F103C8T6
//[MOT0]
// IN1(TIM3_CH4) - PB1
// IN2(TIM3_CH3) - PB0
// IN3(TIM3_CH2) - PA7 	//IN3(PWM3) - PD14
int MOT_BLDC_Mcp8063Loop(void)
{
	 PWM_PINx_init();

	 TIM1_ini(); // Delay timer//TIM5_ini(); // Delay timer

	 TIM2_ini(); // PWM timer
	 TIM3_ini(); // PWM timer
	 TIM4_ini(); // PWM timer

	 FOC_InitPosition();

	 GimbalCntr.des_val = 36; ///////////////
	 GimbalCntr.angle_error = 100;
	 GimbalCntr.angle = 180;

	 GimbalCntr.mode = 1;
	 GimbalCntr.deltaTime = 1000;
	 while(1)
	 {
		 //current time
		 GimbalCntr.t1 = TIM1->CNT;

		 //Get actual angle from the external sensor
		//angle = CQ_average_angle(); //get from angle sensor AS5048A
		//GimbalCntr.des_val = GimbalCntr.ADC_average*360/4095;
		 GimbalCntr.angle += 1;

		 //Get angle error
 		//angle_error = des_val - angle;
		 GimbalCntr.angle_error = -1;//--;GimbalCntr.angle_error -= 1;//--;

		printf("angle=%s ", float2str(GimbalCntr.angle));
 		printf("angle_err=%s, deltaTime=%u\r\n", float2str(GimbalCntr.angle_error), GimbalCntr.deltaTime);

		FOC(GimbalCntr.angle, GimbalCntr.angle_error, 1.1, 0, 0.01,GimbalCntr.deltaTime)	;
		//sinus_control_V2(GimbalCntr.angle_error, VMOT_HALF, 0.8,1.0);//fast0.001,0.05);//slow--//
		delayms(500);

		//for repeating

		//GimbalCntr.angle -= 1;//45;

		GimbalCntr.t2 = TIM1->CNT;								//current time
		GimbalCntr.deltaTime = GimbalCntr.t2 - GimbalCntr.t1; 	//elapsed time

		if(GimbalCntr.angle_error <= -179){
			GimbalCntr.angle_error = 179;
			GimbalCntr.angle = 0.0;
		}

	}
}
int MOT_BLDC_BgcLoop(void)
{
	printf("DRV8313 BLDC Driver\r\n");

	 PWM_PINx_init();

	 TIM1_ini(); // Delay timer//TIM5_ini(); // Delay timer

	 TIM2_ini(); // PWM timer
	 TIM3_ini(); // PWM timer
	 TIM4_ini(); // PWM timer

	 FOC_InitPosition();

	 GimbalCntr.des_val = 36; ///////////////
	 GimbalCntr.angle_error = 100;
	 GimbalCntr.angle = 180;

	 GimbalCntr.mode = 1;
	 GimbalCntr.deltaTime = 1000;
	 while(1)
	 {
		 //current time
		 GimbalCntr.t1 = TIM1->CNT;

		 //Get actual angle from the external sensor
		//angle = CQ_average_angle(); //get from angle sensor AS5048A
		//GimbalCntr.des_val = GimbalCntr.ADC_average*360/4095;
		 GimbalCntr.angle += 1;

		 //Get angle error
 		//angle_error = des_val - angle;
		GimbalCntr.angle_error = -100;//-= 1;//--;

		printf("angle=%s ", float2str(GimbalCntr.angle));
 		printf("angle_err=%s, deltaTime=%u\r\n", float2str(GimbalCntr.angle_error), GimbalCntr.deltaTime);

 		//set a new pwm duty for making sinusoidal waves
		if(GimbalCntr.mode==0) FOC(GimbalCntr.angle, GimbalCntr.angle_error, 1.1, 0, 0.01,GimbalCntr.deltaTime)	;
		else if(GimbalCntr.mode==1) sinus_control_V2(GimbalCntr.angle_error, VMOT_HALF, 0.001,0.05);//slow--//0.8,1.0);//fast
		else if(GimbalCntr.mode==2) combined_control_V3(GimbalCntr.angle, GimbalCntr.angle_error, VMOT_HALF, 0.001, 0.05);

		//delayms(500);

		//for repeating

		//GimbalCntr.angle -= 1;//45;

		GimbalCntr.t2 = TIM1->CNT;								//current time
		GimbalCntr.deltaTime = GimbalCntr.t2 - GimbalCntr.t1; 	//elapsed time

		if(GimbalCntr.angle_error <= -179){
			GimbalCntr.angle_error = 179;
			GimbalCntr.angle = 0.0;
		}

	}
}

 int MOT_BLDC_Drv8313Loop(void)
{

	//USART_2_init();
	//SPI3_ini();
	//user_button_init();

	//TM_RNG_Init();


	 PWM_PINx_init();

	 TIM1_ini(); // Delay timer//TIM5_ini(); // Delay timer

	 TIM2_ini(); // PWM timer
	 TIM3_ini(); // PWM timer
	 TIM4_ini(); // PWM timer

	 //ENx_init();
	 //ADC_initt();
	 //Set_nRes_nSleep();
	 //Set_ENx();
	 FOC_InitPosition();

	 GimbalCntr.des_val = 36.0; ///////////////
	 GimbalCntr.angle_error = 2.0;
	 GimbalCntr.angle = 180.0;

	 GimbalCntr.mode = 1; //YOON
	 GimbalCntr.deltaTime = 1000;
	 while(1)
	 {
		 GimbalCntr.t1 = TIM1->CNT;//current time
		//angle = CQ_average_angle(); //get from angle sensor AS5048A

		//GimbalCntr.des_val = GimbalCntr.ADC_average*360/4095;
  		//angle_error = des_val - angle;
		//GimbalCntr.angle_error += 45;//--;
		//GimbalCntr.angle_error = 2;  //TM_RNG_Get();

  		printf("angle=%s ", float2str(GimbalCntr.angle));
  		printf("angle_err=%s, t1=%u, deltaTime=%u\r\n", float2str(GimbalCntr.angle_error), GimbalCntr.t1 ,GimbalCntr.deltaTime);

		//adjust PWM
		if(GimbalCntr.mode==0) FOC(GimbalCntr.angle, GimbalCntr.angle_error, 1.1, 0, 0.01,GimbalCntr.deltaTime)	;
 		//if(GimbalCntr.mode==1) sinus_control_V2(GimbalCntr.angle_error, VMOT_HALF, 0.001, 0.05); //Not working?
		if(GimbalCntr.mode==1) sinus_control(GimbalCntr.angle); //working
		if(GimbalCntr.mode==2) combined_control_V3(GimbalCntr.angle, GimbalCntr.angle_error, VMOT_HALF, 0.001, 0.05);

		GimbalCntr.t2 = TIM1->CNT;//TIM5->CNT; //current time
		GimbalCntr.deltaTime = GimbalCntr.t2 - GimbalCntr.t1; //delay time

		//if(GimbalCntr.angle_error <= -179)			GimbalCntr.angle_error = 179;
		if(GimbalCntr.angle < 180.0){
			GimbalCntr.angle_error = 9.0;
			GimbalCntr.angle += 10.0;
		}else if(GimbalCntr.angle >= 180.0){
			GimbalCntr.angle_error = -9.0;
			GimbalCntr.angle += 10.0;
		}
		if(GimbalCntr.angle >= 360.0){
			GimbalCntr.angle_error = 9.0;
			GimbalCntr.angle = 0.0;
		}

		delayms(1);//500);

	}
}
/*=========================================================
 void EXTI0_IRQHandler(void)
 {
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line0);
		NVIC_DisableIRQ(EXTI0_IRQn);
		mode++;
		if(mode>2) mode=0;
		NVIC_EnableIRQ(EXTI0_IRQn);
	}
}

void ADC_IRQHandler()
{

		if(ADC_GetITStatus(ADC1,ADC_IT_EOC) !=RESET)
		{
			ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
			ADC_average =  moving_average(&ADC_filter, (float)ADC_GetConversionValue(ADC1), 2000);
			ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_480Cycles);
			ADC_SoftwareStartConv(ADC1);
		}

}

void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_TXE)==SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_TXE);
	}

	if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	//	USART_test_rec = (unsigned char)USART_ReceiveData(USART2);

		USART_test_rec = USART_ReceiveData(USART2);

		if(IMU_data==0)
		{
			if(USART_test_rec==0x55)
			{
				IMU_data=1;
			}
		}

		else if(IMU_data==1)
		{
			if(USART_test_rec==0x53)
			{
				IMU_data = 2;
			}
			else
			{
				IMU_data=0;
			}
		}

		else if (IMU_data==2)
		{
			if(IMU_count<6)
			{
			IMU_Recieve_Buf[IMU_count]=USART_test_rec;
			IMU_count++;
			}
			else
			{
				IMU_data_ready=1;
				IMU_count=0;
				IMU_data=0;
			}
		}
	}
}
*/
/*
//==== angle sensor
	////Mean of circular quantities
	float sine_sum, cos_sum, arr_CQ[window], sine_arr[window], cos_arr[window], sine_av, cos_av, X_i_CQ;
	float a_i_CQ, sine_i, cos_i, raw_value;
	unsigned char filled_CQ;
	uint16_t k_CQ;


	float CQ_average_angle(void)
	{
		if(!filled_CQ)
		{
			sine_sum = 0;
			cos_sum = 0;
			for (int i=0; i < window; i++ )
			 {
				arr_CQ[i] = get_angle()*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians;
				sine_arr[i] = arm_sin_f32(arr_CQ[i]);
				cos_arr[i] = cosf(arr_CQ[i]);
				sine_sum = sine_sum + sine_arr[i];
				cos_sum = cos_sum + cos_arr[i];
			 }

			 sine_av = sine_sum/window;
			 cos_av = cos_sum/window;
			 X_i_CQ  = atan2(sine_av, cos_av)*57.295779513082320876798154814105 ; // out of the filter

			 filled_CQ = 1;
			 k_CQ=0;
			 return X_i_CQ;

		}

		// 2) start filtering

		else
		{
			raw_value = get_angle();
			a_i_CQ = raw_value*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians;
			sine_i = arm_sin_f32(a_i_CQ);
			cos_i = cosf(a_i_CQ);
			sine_sum = sine_sum - sine_arr[k_CQ] + sine_i;
			cos_sum = cos_sum - cos_arr[k_CQ] + cos_i;
			sine_av = sine_sum/window;
			cos_av = cos_sum/window;
			X_i_CQ  = atan2(sine_av, cos_av)*57.295779513082320876798154814105 ;

			sine_arr[k_CQ] = sine_i;
			cos_arr[k_CQ] = cos_i;
			 // substitute thrown out value with new value for cycling
			k_CQ++;
			if(k_CQ >= window) k_CQ=0; // array loop
			return X_i_CQ;

		}

	}
*/
#endif

#endif
