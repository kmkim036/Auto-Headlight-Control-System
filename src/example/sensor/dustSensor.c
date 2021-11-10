// Filename: DH11TempSensor.c
//-- It uses a specific 1-wire commun. See its datasheet. Too slow sensor.
// - Sensor Data pin is open drain(needs pullup resistor of 4.7K)
// - No under Zero degree...
// - Not support decimal point for Humidity and Temperature.

//-- We use PA8
//Not support float in STM32F103.

//GPIO PINs
//+-----------------+-----------+-----------+-----------+---------+---------+
//|                 |103C8T6
//+-----------------+-----------+-----------+-----------+---------+---------+
//| oTriggerPulse   | PC13
//+-----------------+-----------+-----------+-----------+---------+---------+
//| ADC input       | PA1 (ADC12_IN1) -- Two 12bit ADC
//+-----------------+-----------+-----------+-----------+---------+---------+
//*H= Active High/L=Active Low
//*F= FallingEdge

#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_adc.h"
#include "misc.h"
//#include "stm32f10x_flash.h"
#include "yInc.h"
#include "core_cm3.h"    //for irq
#include "core_cmFunc.h" //for irq

extern void delayus(uint32_t ms);

struct GP2Y {
  GPIO_InitTypeDef myGPIO;
  ADC_InitTypeDef myADC;

} gp2y;

//PA8 for pulsing
void GP2Y_LedPulse_Config(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //oLED-pinA8
    GPIO_StructInit(&gp2y.myGPIO);
    gp2y.myGPIO.GPIO_Pin = GPIO_Pin_8;
    gp2y.myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
    gp2y.myGPIO.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gp2y.myGPIO);
}

void GP2Y_Adc_Config()
{
  // PA6를 analog input 모드로 설정한다.
	gp2y.myGPIO.GPIO_Pin = GPIO_Pin_1; //set to PA1
	gp2y.myGPIO.GPIO_Mode = GPIO_Mode_AIN; //set as analog input
	GPIO_Init(GPIOA, &gp2y.myGPIO); //set to A1

	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //clock for ADC (max 14MHz, 72/6=12MHz)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //enable ADC clock

	//configure ADC parameters
	gp2y.myADC.ADC_Mode = ADC_Mode_Independent;
	gp2y.myADC.ADC_ScanConvMode = DISABLE;
	gp2y.myADC.ADC_ContinuousConvMode = ENABLE;
	gp2y.myADC.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	gp2y.myADC.ADC_DataAlign = ADC_DataAlign_Right;
	gp2y.myADC.ADC_NbrOfChannel  = 1;
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5); //PA1 as Input
	ADC_Init(ADC1, &gp2y.myADC);

  //enable
  ADC_Cmd(ADC1, ENABLE);

  //Calibrate ADC *optional?
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));

  //enable ADC to work
  ADC_Cmd(ADC1, ENABLE);
}
//              |----10msec --------|---10msec---------------|------
//
//----+         +0.32ms-+           +-0.32ms--+              +--
//    |         |       |           |         |              |
//    +---------+       +----9.68ms-+         +---9.68ms-----+
//                .-----.
  //             /       \
// -------------/      ^  \--------
//              |0.28ms|--- sampling
//get Analog Value at pin
unsigned short GP2Y_GetDust(void)
{
	unsigned short adcval_s;
	//LED ON
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
	delayus(280); //0.28msec
	//Sampling
	adcval_s = ADC_GetConversionValue(ADC1);
	delayus(40); //0.04msec
	//LED OFF
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	delayus(9680); //9.68msec
	return adcval_s;
}


void GP2Y_ShowDust(unsigned short dust){
	int i,j, invRange;
	char strbuf[100];

		j = dust/5;
		if(j>100) 	j= 100;
		for(i=0;i<j;i++){
			PutChar(&strbuf[i],'.');
		}
		PutChar(&strbuf[i],0x00);
		printf("%s %u\r\n",strbuf,dust);
}

extern char *float2str(float x);

int GP2Y_Loop(void)
{
	int dust;
	float dust_f;
	GP2Y_LedPulse_Config(); //configure pins
	GP2Y_Adc_Config(); //configure ADC

    while(1)
    {
    	dust = GP2Y_GetDust();
        dust_f = (float)(dust) *3.3/4095.0; //get analog value and convert to volts, 12bit ADC
        //printf("DustValue=%u(%sVolt)\r\n", dust, float2str(dust_f));
        GP2Y_ShowDust(dust);
        delayms(20);
     }
}

