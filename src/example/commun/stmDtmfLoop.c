/*
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
*/
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
//#include "stm32f10x_flash.h"


	//HT9200B -- Parallel Mode
	/* D0: P1,  Output, PA11
	 * D1: P2,  Output, PA12
	 * D2: P3,  Output, PD10
	 * D3: P4,  Output, PB5
	 * nCE:P5,  Output, PB6
	 */

void stmDtmfGen_GPIO_setup(void){
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //STM407VGT6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //STM407VGT6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //STM407VGT6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_6);
}

void stmDtmfRcv_GPIO_setup(void){
	GPIO_InitTypeDef GPIO_InitStruct;
/*	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //STM407VGT6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1);
*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //STM407VGT6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_IN; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);
/*
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //STM407VGT6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_6);
*/
}
void stmDtmfGenLoop()
{
	int i,j;

	//ENCODER : HT9200B
	//HT9200B -- Parallel Mode
	stmDtmfGen_GPIO_setup();
	//Rcv
	stmDtmfRcv_GPIO_setup();
	// Loop forever.

	while(1)
	{

		//PD0
		GPIO_ResetBits(GPIOD, GPIO_Pin_0 );// D0 = 0
		GPIO_ResetBits(GPIOB, GPIO_Pin_6); // PB6 => ENABLE
		delayms(200);
		GPIO_SetBits(GPIOB, GPIO_Pin_6); // PB6 => DISABLE
		delayms(200);
		//New Data
		GPIO_SetBits(GPIOD, GPIO_Pin_0); // D0 = 1
		GPIO_ResetBits(GPIOB, GPIO_Pin_6); // PB6 => ENABLE
		delayms(200);
		GPIO_SetBits(GPIOB, GPIO_Pin_6); // PB6 => DISABLE
		delayms(200);
	}
}
/*
void DtmfGenHT9200bLoop()
{
	int i,j;


	//HT9200B -- Parallel Mode
	 D3: P10, Output, PD1
	 * D2: P9, Output, PD0
	 * D1:PP8, Output, PD5
	 * D0: P7, Output, PD4
	 * nCE: P16, Output, PE0
	 * CLK : P34 Output, PC7  //Serial Mode Only

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//Serial Mode Only

	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_4);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, ~GPIO_PIN_1); //3
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, ~GPIO_PIN_0); //3
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5); //3

	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , GPIO_PIN_0 ); //0
	// Loop forever.
	while(1)
	{
		// Delay some time
		delayms(500);
		// Output high level
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_4 , GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_4); //3
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, ~GPIO_PIN_4); //D0=0
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , ~GPIO_PIN_0 ); //nCE=0
		delayms(500);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4); //D0=1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , GPIO_PIN_0 ); //nCE=1
		delayms(500);

		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4); //3
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , ~GPIO_PIN_0 ); //0
		delayms(500);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , GPIO_PIN_0 ); //1

		delayms(500);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); //D3=1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , ~GPIO_PIN_0 ); //nCE=0
		delayms(500);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, ~GPIO_PIN_1); //D3=1
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , GPIO_PIN_0 ); //nCE=1
	}
}
*/
/*
//for HT9170D
//#define isDV (GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_6) == 0x40) //V5
#define isDV (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6) == 0x40) //V3
void DtmfDecLoop()
{
	long val;
	long valtemp;

	/* D3: P32, Output, PD1
	 * D2: P33, Output, PD0
	 * D1: P34, Output, PD5
	 * D0: P35, Output, PD4
	 * nCE: P5, Output, PE0
	*/

	//input from HT9170D DTMF decoder (V3)
    /* oDV: P31, input, PB6
	   D3: P30, input, PB5
	 * D2: P29, input, PB4
	 * D1: P18, input, PB3
	 * D0: P17, input, PB2
	 * */
	//input from HT9170D DTMF decoder (V5)
    /* oDV: P15, input, PD6
	   D3: P22, input, PB0 (Need ULED1 off on Main board)
	 * D2: P21, input, PG1
	 * D1: P20, input, PB1
	 * D0: P19, input, PE1


	//V3

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
	delayms(100);
	// Loop forever.
	while(1)
	{
		if(isDV){
			valtemp = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_5); //D3
			val = valtemp >> 2;
			valtemp = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_4); //D2
			valtemp = valtemp >> 2;
			val = val | valtemp;
			valtemp = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3); //D1
			valtemp = valtemp >> 2;
			val = val | valtemp;
			valtemp = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2); //D0
			valtemp = valtemp >> 2;
			val = val | valtemp;
			UARTprintf("DTMFdec> Val=%02x\r\n",val);
			delayms(100);
		}
	}


}
*/
