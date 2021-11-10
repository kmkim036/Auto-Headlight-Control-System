#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "ylib/yInc.h"

void uPD22100Init(void);

/* XBAR_DATA    : Pin8 : PD5 (0=DISCONNECT; 1=CONNECT)
 * XBAR_STRBn   : Pin9 : PD0 (Active High Logic)-- Caution.
 * XBAR_A       : Pin10: PD1 (LSB)
 * XBAR_B       : Pin26: PF1
 * XBAR_C       : Pin27: PE3
 * XBAR_D       : Pin28: PE2 (MSB)
 */
#define XBAR_DATA_DISC (GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5))
#define XBAR_DATA_CONN (GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_PIN_5))
#define XBAR_STRB_ON   (GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0))
#define XBAR_STRB_OFF  (GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, ~GPIO_PIN_0))

#define XBAR_A0   		(GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, ~GPIO_PIN_1))
#define XBAR_A1         (GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1))
#define XBAR_B0   		(GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIO_PIN_1))
#define XBAR_B1         (GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1))
#define XBAR_C0   		(GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, ~GPIO_PIN_3))
#define XBAR_C1         (GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3))
#define XBAR_D0   		(GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, ~GPIO_PIN_2))
#define XBAR_D1         (GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2))

void uPD22100Init(void)
{
	UARTprintf("Space Switch Control with uPD22100.\n");

	//Set GPIOs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_1 | GPIO_PIN_0);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

	//POWER ON RESET
	//XBAR_DATA_DISC;delayms(100);XBAR_DATA_CONN;
}
unsigned char uPD22100_Control(unsigned char X, unsigned char Y, u8 con){
	unsigned long retVal;

	if((Y==0) && (X==0)){//Y0:X0 ==> 0000
		XBAR_D0;	XBAR_C0; 	XBAR_B0;  XBAR_A0;
	}else if((Y==0) && (X==1)){//Y0:X1 ==> 0001
		XBAR_D0;	XBAR_C0; 	XBAR_B0;  XBAR_A1;
	}else if((Y==0) && (X==2)){//Y0:X1 ==> 0010
		XBAR_D0;	XBAR_C0; 	XBAR_B1;  XBAR_A0;
	}else if((Y==0) && (X==3)){//Y0:X1 ==> 0011
		XBAR_D0;	XBAR_C0; 	XBAR_B1;  XBAR_A1;
	}else if((Y==1) && (X==0)){//Y1:X0 ==> 0100
		XBAR_D0;	XBAR_C1; 	XBAR_B0;  XBAR_A0;
	}else if((Y==1) && (X==1)){//Y1:X1 ==> 0101
		XBAR_D0;	XBAR_C1; 	XBAR_B0;  XBAR_A1;
	}else if((Y==1) && (X==2)){//Y1:X2 ==> 0110
		XBAR_D0;	XBAR_C1; 	XBAR_B1;  XBAR_A0;
	}else if((Y==1) && (X==3)){//Y1:X3 ==> 0111
		XBAR_D0;	XBAR_C1; 	XBAR_B1;  XBAR_A1;
	}else{ //TBD

	}
	if(con)
		XBAR_DATA_CONN;
	else
		XBAR_DATA_DISC;

	XBAR_STRB_ON;
	delayms(1);
	XBAR_STRB_OFF;

	if(con)
		UARTprintf("Connect X=%u <--> Y=%u.\n");
	else
		UARTprintf("Disconnect X=%u <--> Y=%u.\n");

	return 0;
}


int uPD22100Loop(){

	GuiInit("uPD22100 4x4 Crossbar Switch Test");

	uPD22100Init();
	while(1){

		uPD22100_Control(0,1,1); //CONNECT
		delayms(2000);
		uPD22100_Control(0,1,0); //DISCONNECT
		delayms(2000);

		uPD22100_Control(2,3,1); //CONNECT
		delayms(2000);
		uPD22100_Control(2,3,0); //DISCONNECT
		delayms(2000);
	}
}



