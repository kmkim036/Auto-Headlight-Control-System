#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"

void DtmfGenLoop()
{
	int i,j;

	//HT9200B -- Parallel Mode
	/* D3: P10, Output, PD1
	 * D2: P9, Output, PD0
	 * D1:PP8, Output, PD5
	 * D0: P7, Output, PD4
	 * nCE: P16, Output, PE0
	 *
	 * */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_4);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, ~GPIO_PIN_1); //3
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, ~GPIO_PIN_0); //3
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5); //3

	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , GPIO_PIN_0 ); //0
	// Loop forever.
	while(1)
	{
		// Delay some time
		for(j=0; j<800000; j++) {}
		// Output high level
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_4 , GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_4); //3
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, ~GPIO_PIN_4); //3
		for(j=0; j<1000; j++) {}
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , ~GPIO_PIN_0 ); //0
		for(j=0; j<200000; j++) {}//upto 40msec
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , GPIO_PIN_0 ); //1

		for(j=0; j<800000; j++) {}
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4); //3
		for(j=0; j<1000; j++) {}
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , ~GPIO_PIN_0 ); //0
		for(j=0; j<200000; j++) {} //upto 40msec
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0  , GPIO_PIN_0 ); //1
	}
}
void DtmfGenHT9200bLoop()
{
	int i,j;

	//HT9200B -- Parallel Mode
	/* D3: P10, Output, PD1
	 * D2: P9, Output, PD0
	 * D1:PP8, Output, PD5
	 * D0: P7, Output, PD4
	 * nCE: P16, Output, PE0
	 * CLK : P34 Output, PC7  //Serial Mode Only
	 * */
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
//for HT9170D
//#define isDV (GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_6) == 0x40) //V5
#define isDV (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6) == 0x40) //V3
void DtmfDecLoop()
{
	long val;
	long valtemp;

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
	 */

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

/*	//V5
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_1);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);

	delayms(100);
	// Loop forever.
	while(1)
	{
		if(isDV){
			valtemp = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0); //D3
			val = valtemp << 3;
			valtemp = GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_1); //D2
			valtemp = valtemp << 1;
			val = val | valtemp;
			valtemp = GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_1); //D1
			val = val | valtemp;
			valtemp = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1); //D0
			val = val | (valtemp>>1);
			UARTprintf("DTMFdec> Val=%02x\r\n",val);
			delayms(100);
		}
	}
	*/
}
