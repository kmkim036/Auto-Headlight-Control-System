#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
//#include "driverlib/qei.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "ylib/yInc.h"

//Util
//extern void UARTStdioIntHandler(void);//extern void UART0_IRQHandler(void);
//extern void Cmd_Init();

//SPI
//LEDëŠ” ì—†ì§€ë§Œ..ì�¼ì •ì‹œê°„ í‚¤ê±°ë‚˜ ë�”.
void dtIrdaGpioConf(void)
{
	// LED : PA7
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7); //LED
	//Init
	somedelay(100);

}
void dtIrdaLoop(void)
{
	unsigned char retc;

	InitUart0();
    IntMasterEnable();
	//Hello

	GuiInit("IrDA Test with Uart1.");
	InitUart1(9600);
	UARTprintf("IrDA> after InitUart1(9600bps).\r\n");
	//UI Config
	dtGpsGpioConf();
	dtUserLedCon(1);
	somedelay(100000);
	dtUserLedCon(0);
	//Cmd_Init();
	while(1) //main loop
    {
		UART1Send("A",1); //send 'A' via Uart 1
		dtUserLedCon(1);
		UARTprintf("IrDA>Send 'A'\r\n");
		somedelay(1000000);
		retc = UART1RcvToUART0(); //receive data and transfer to uart 0
		dtUserLedCon(0);
		UARTprintf("IrDA>Rcv..0x%02x\r\n",retc);
		somedelay(1000000);
	}
}


