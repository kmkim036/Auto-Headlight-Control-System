#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"
#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"
#include "ylib/yInc.h"

//#define NUM_SSI_DATA 1

//*****************************************************************************
// YOON : The tp3155 TSA follows a kind of SPI Mode 1 for assigning slots.
// Configure SSI0 in master (SPI) mode 1.
// (*) You may design it with a special control bus using bitbang, instead of SPI.
//*****************************************************************************
unsigned char TP3155_WriteControl(unsigned char I, unsigned char R);
void spiTP3155Init(void);

#define nCS0_0 (GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3))
#define nCS0_1 (GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3))

unsigned char TP3155_WriteControl(unsigned char I, unsigned char R){
	unsigned char ch1,ch0;
	unsigned long dc;
	unsigned long retVal;
	//for Initiator
	ch1 = (I & 0x02)>>1; ch0= I & 0x01;
	if(ch1)
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
	else
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
	if(ch0)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
	else
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, ~GPIO_PIN_6);

	dc = 0x40 | I;
	nCS0_0;
	dtSpiWrByte(dc);
	nCS0_1;

	dc = 0x80 | R;
	nCS0_0;
	dtSpiWrByte(dc);
	nCS0_1;
	//for Responder
	ch1 = (R & 0x02)>>1; ch0= R & 0x01;
	if(ch1)
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
	else
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
	if(ch0)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
	else
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, ~GPIO_PIN_6);

	dc = 0x40 | R;
	nCS0_0;
	SSIDataPut(SSI0_BASE, dc);
	SSIDataGet(SSI0_BASE, &retVal); //dummy
	nCS0_1;
	while(SSIBusy(SSI0_BASE));
	dc = 0x80 | I;
	nCS0_0;
	SSIDataPut(SSI0_BASE, dc);
	SSIDataGet(SSI0_BASE, &retVal); //dummy
	nCS0_1;
	while(SSIBusy(SSI0_BASE));

	return 0;
}

void Tp3155Init(void)
{
	UARTprintf("TSI Telepone Switch Control with SPI Mode 1 ->\n");
	UARTprintf("Data: 8-bit\n\n");
	/*
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	// Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	//GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5);
	//GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3); //PA3 -- FSS
	//GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
	//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3); //Set HIGH ////PA3
	SSIDisable(SSI0_BASE);
	//SSIClockSourceSet(SSI0_BASE,SSI_CLOCK_SYSTEM); //dummy?
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(),SSI_FRF_MOTO_MODE_1,// SSI_FRF_MOTO_MODE_3,//SSI_FRF_MOTO_MODE_0,
                   SSI_MODE_MASTER, 1000000, 8);// 16);//8);//was 1000000

	SSIIntDisable(SSI0_BASE,SSI_TXFF | SSI_RXFF | SSI_RXOR | SSI_RXTO);
	SSIDMADisable(SSI0_BASE, SSI_DMA_RX | SSI_DMA_TX);
	SSIEnable(SSI0_BASE);
	*/

    dtSpiMasterInit(1,1000000,8, SYSCTL_PERIPH_GPIOA, GPIO_PORTA_BASE, GPIO_PIN_3); //use default nCS of PA3

	//Set CH1,CH1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6); //PD6 -- CH0
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0); //PE0 -- CH1
	//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); //Set HIGH ////PD6
	//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0); //Set HIGH ////PE0

	//Set Switch1,2,3,4 on TelBoard
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1); //PE1 -- SW1
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_1); //PB1 -- SW2
	GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_1); //PG1 -- SW3
	GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_0); //PG0 -- SW4
	//Set MainBoard Switch1,2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7 | GPIO_PIN_5); //PC7 -- SW3(2) on M/B Confirm; PC5--SW2(1) Set on M/B
}
/*
 * unsigned char TP3155_ReadByte(){
    unsigned short retVal;

  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);//nCsSpiEeprom = 0;
  //retVal = SpiRdByte();
  SSIDataPut(SSI0_BASE, 0xffff);
  SSIDataGet(SSI0_BASE, &retVal);
  while(SSIBusy(SSI0_BASE));
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);//nCsSpiEeprom = 1;

  return retVal;
}
*/
int Tp3155Loop(){

	GuiInit("Tp3155 TSA Switch Test");

	Tp3155Init();
	while(1){
		//TP3155_WriteControl(0,0);
		TP3155_WriteControl(0,1);
		delayms(2000);
		//TP3155_WriteControl(1,0);
		TP3155_WriteControl(0,2);
		delayms(2000);
		TP3155_WriteControl(0,3);
		delayms(2000);
	}
}



