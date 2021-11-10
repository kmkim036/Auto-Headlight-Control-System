#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
//#include "driverlib/qei.h"
#include "ylib/yInc.h"

/* 4Bit Mode CLCD with PC1601LRS-ASO-B with HD44780
 * Notes:		(1)	The LCD pinout is as follows:
**
**					01 - Ground (Vss)
**					02 - 5V Power (Vcc)
**					03 - Contrast (Vo)
**					04 - Register Select (RS)
**					05 - Read/Write_L (R/W_L)
**					06 - Enable (E)
**					07 - Data 0 (DB0)
**					08 - Data 1 (DB1)
**					09 - Data 2 (DB2)
**					10 - Data 3 (DB3)
**					11 - Data 4 (DB4)
**					12 - Data 5 (DB5)
**					13 - Data 6 (DB6)
**					14 - Data 7 (DB7)
**					15 - Backlight 5V Power (use 10 ohm resistor)
**					16 - Ground (Vss)
**
**				(2)	Port pinouts are considered to be the following:
**
**					P7 - No connect (NC)
**					PB0 : P6 - Enable (E) : PB0
**					PD5 : P5 - Read/Write_L (R/W_L)  --- TO BE GND Permanently
**					PD4 : P4 - Register Select (RS)
**					PB6 : P3 - Data 7 (DB7)
**					PB5 : P2 - Data 6 (DB6)
**					PB4 : P1 - Data 5 (DB5)
**					PB1 : P0 - Data 4 (DB4)
 */
void Hd44780Hello(void){
	UARTprintf("Hd4470 Hello\r\n");
	Hd44780WriteString("Hd4470 Hello");
}

//High Nibble First...
void Hd44780WriteNibble(unsigned char nib){
	//WR = 0; //Always Low PD5
	unsigned char PortVal=0x00;
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5);

	//EN=1; PB0
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0);
	//Prepare Nibble
	nib = nib & 0x0f;
	PortVal = (nib & 0x08) << 3; //MSB --> PB6 //DB7
	PortVal = PortVal | ((nib & 0x04) << 3); //PB5//DB6
	PortVal = PortVal | ((nib & 0x02) << 3); //PB4//DB5
	PortVal = PortVal | ((nib & 0x01) << 1); //PB1//DB4
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_1, PortVal);
	somedelay(1);
	//EN=0; //Data to be latched at EN's falling edge
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
	//WR = 1;
	somedelay(1);
}
//Nibble
void Hd44780WriteCtrl(unsigned char ctl){
	//RS = 0; //Low=Instruction -- PD4
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, ~GPIO_PIN_4);
	Hd44780WriteNibble(ctl);
}

void Hd44780WriteByteData(unsigned char data){

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4);//RS = 1; //High=Data

	Hd44780WriteNibble(data >> 4); //MSB
	Hd44780WriteNibble(data);  //LSB

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, ~GPIO_PIN_4);	//RS=0;
}
void Hd44780ConfigModule(){

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//Make all output
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5) ;
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6) ;

	//WR = 0; //Always Low PD5
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5);
	//EN=1; PB0
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0);
}

unsigned char Hd44780WriteString(char *str){
	int i;
	for(i=0; i<16; i++){
		if(!str[i]) break;
		Hd44780WriteByteData(str[i]);
	}
}
void Hd44780Init(){

	Hd44780ConfigModule();

	//Wait  after powered up for 15msec
	somedelay(15000); //6.8msec for 6000
	//Init LCD to 4-bit mode
	//write 3 3-times
	Hd44780WriteCtrl(3);
	somedelay(5000);	//Wait 5msec for the instruction to complete
	Hd44780WriteCtrl(3);
	somedelay(160); //wait 160usec
	Hd44780WriteCtrl(3);
	somedelay(160);
	//Set 4-bit mode
	Hd44780WriteCtrl(2);
	somedelay(50);

	//Function Set
	Hd44780WriteCtrl(2);
	somedelay(50);
	Hd44780WriteCtrl(8);
	somedelay(50);

	//Display OFF
	Hd44780WriteCtrl(0);
	somedelay(50);
	Hd44780WriteCtrl(8);
	somedelay(50);
	//Display ON
	Hd44780WriteCtrl(0);
	somedelay(50);
	Hd44780WriteCtrl(0xF);
	somedelay(50);

	//EntryMode
	Hd44780WriteCtrl(0);
	somedelay(50);
	Hd44780WriteCtrl(6);
	somedelay(50);
	//Clear Screen
	Hd44780WriteCtrl(0);
	somedelay(50);
	Hd44780WriteCtrl(1);
	somedelay(50);
	//Cursor Home
	Hd44780WriteCtrl(0);
	somedelay(50);
	Hd44780WriteCtrl(2);
	somedelay(50);
	//Turn On and Blink
	Hd44780WriteCtrl(0);
	somedelay(50);
	Hd44780WriteCtrl(0xf);
	somedelay(50);

}

void Hd44780Loop(){
	Hd44780Init();
	Hd44780Hello();
	while(1){}
}

