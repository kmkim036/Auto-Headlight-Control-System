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
#include "yInc.h"

//use nCS2 of PA3
#define nCS3008_1      {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);}
#define nCS3008_0      {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);}

/*
class MCP3008
{
  public:
    MCP3008(int clockpin, int mosipin, int misopin, int cspin);
    int readADC(int adcnum);
  private:
      int _clockpin, _mosipin, _misopin, _cspin;
};
*/
void MCP3008Init(void){
	//use MODE 0
	  dtSpiMasterInit(SSI_FRF_MOTO_MODE_0,10000000,8, SYSCTL_PERIPH_GPIOA, GPIO_PORTA_BASE, GPIO_PIN_3); //use PA3 of nCS0
	  nCS3008_1;//nCs=1
	  delayms(10);
	  UARTprintf("MCP3008> SPI ADC TEST INIT\r\n");
};

// read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
//use MODE 0
unsigned short MCP3008_readADC(int adcnum) {
    unsigned long retVal;
    unsigned short readAnalogValue;
    unsigned long dummy=0;
	int i, adcout;
	int commandout = adcnum;
	if ((adcnum > 7) || (adcnum < 0)) return -1; // Wrong adc address return -1

    commandout = 0x08 | adcnum; //  prepare single-ended bit(1) : 1nnn
    commandout <<= 4;
	nCS3008_0;//nCs=0

	// Send Start Bit(1)
	dtSpiWrByte(0x01);
	//-write and read - 1(single-ended) - nnn(adcNum) -
    SSIDataPut(SSI0_BASE, commandout);
    SSIDataGet(SSI0_BASE, &retVal);
    while(SSIBusy(SSI0_BASE));
    readAnalogValue = retVal & 0x07;
    readAnalogValue << 8;
    //read remaining 8 bit value
    retVal = dtSpiRdByte();
    readAnalogValue = readAnalogValue |  retVal;

    return readAnalogValue;
}

void MCP3008Loop(){
	 static uint8_t i = 0;
	 static unsigned int rdval;

	 	 MCP3008Init();

		while(1){
			for(i=0;i<8;i++){
				rdval = MCP3008_readADC(i);
				UARTprintf("rdvalw=0x%04x\r\n",rdval);
			}
			delayms(20);
		}
}


