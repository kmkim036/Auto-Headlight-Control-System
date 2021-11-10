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
/*Use RDM6300 with UART1 (9600,N,8,1)
 * Output Format = 0x02(SOF)-10 ASCII Charcters - Cksum - 0x03(EOF)
 * +------------------------------------------------------------------+
 * | ANT ANT                                               GND 5V LED |
 * |                                                                  |
 * |                                              5V GND NC NC(RXD)TXD| TXD means Tx from RDM6300 to UART1 RX of MCU.
 * +------------------------------------------------------------------+
 * UART0 = (115200,N,8,1)
 */
void dtRdm6300GpioConf(void)
{
	// LED : PA7
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7); //LED
	//Init
	somedelay(100);

}
void dtRdm6300Loop(void)
{
	unsigned char retc;

	//InitUart0();
    //IntMasterEnable();
	//Hello

	GuiInit("RDM6300 125KHz RFID Reader Test with Uart1.");
	InitUart1(9600);
	UARTprintf("RDM6300>InitUart1(9600bps).\r\n");
	//UI Config
	dtRdm6300GpioConf();
	dtUserLedCon(1);
	somedelay(100000);
	dtUserLedCon(0);
	//Cmd_Init();
	UARTprintf("Rdm6300>Rcv:");
	while(1) //main loop
    {
		dtUserLedCon(1);
		retc = UART1RcvToUART0(); //receive data and transfer to uart 0
		if(retc)
			UARTprintf("%02x ",retc);
		//hw1secDelay(2);
		//dtUserLedCon(0);
	}
}


