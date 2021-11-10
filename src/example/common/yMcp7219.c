/*
  LedControl.cpp - A library for controling Leds with a MAX7219/MAX7221
 *    Copyright (c) 2007 Eberhard Fahle
 *
 *    Modified for STM32F4xx by Arami
 * 	  SPI : 1Mbps, 8 bit mode
 *    Connections to MAX7219:
// SPI MODE = 0
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0CLK - PA2
//! - SSI0Fss - PA3 nCS0
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5

 *  Configures the SPI3 Peripheral.
 *  CLK - PC10
 *  MISO - PC11 -- NOT USED
 *  MOSI - PC12
 *  nCS1 - PD2
*/
/*
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "yInc.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
*/

#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#include "yInc.h"
#include "ySpiDrv.h"
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//|                 |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   | 103
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| ULED            | PB14      |PC4        |PE15       | <==     | PG7     |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| BUTTON          |           |PC5(H)     |PD11(index)| PD11(L) |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| BEEP            |           |PB13       |PD14       | <==     |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| QEI             |           |PB0,1,12   |PD12,13,11 | PD12,13 |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| nCS0            |           |           |           | PD10    |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| nCS1            |           |           |           | PD15    |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| nCS2            |           |           |           | PE5     |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//*H= Active High/L=Active Low
//*F= FallingEdge


//======== The chip select assignment =========
// Use nCS0 for 7219
//#define nCS7219_H nCS0_H
//#define nCS7219_L nCS0_L

//Use nCS1 for 7219
#define nCS7219_H nCS1_H
#define nCS7219_L nCS1_L

extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern void stmSPI3_Config(unsigned char nCS);
extern void stmSpiWrByte(unsigned char inbyte);
extern unsigned char stmSpiRdByte();

// Segments to be switched on for characters and digits on 7-Segment Displays
 const static unsigned char charTable [] = {
	    0B01111110,0B00110000,0B01101101,0B01111001,0B00110011,0B01011011,0B01011111,0B01110000,
	    0B01111111,0B01111011,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,
	    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
	    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
	    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
	    0B00000000,0B00000000,0B00000000,0B00000000,0B10000000,0B00000001,0B10000000,0B00000000,
	    0B01111110,0B00110000,0B01101101,0B01111001,0B00110011,0B01011011,0B01011111,0B01110000,
	    0B01111111,0B01111011,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
	    0B00000000,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,0B00000000,
	    0B00110111,0B00000000,0B00000000,0B00000000,0B00001110,0B00000000,0B00000000,0B00000000,
	    0B01100111,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
	    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00001000,
	    0B00000000,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,0B00000000,
	    0B00110111,0B00000000,0B00000000,0B00000000,0B00001110,0B00000000,0B00010101,0B00011101,
	    0B01100111,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
	    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000

};
//the opcodes for the MAX7221 and MAX7219
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

struct MAX7219_LedControl {
        /* The array for shifting the data to the devices */
        unsigned char spidata[16];
        /* We keep track of the led-status for all 8 devices in this array */
        unsigned char status[64];
        /* The maximum number of devices we use */
        int maxDevices;
};
struct MAX7219_LedControl g_stmMax7219;
        /*
         * Set the shutdown (power saving) mode for the device
         * addr	The address of the display to control
         * status	If true the device goes into power-down mode. Set to false for normal operation.
         */
        void stmMax7219_shutdown(int addr, bool status);

        /*
         * Set the number of digits (or rows) to be displayed.
         * See datasheet for sideeffects of the scanlimit on the brightness
         * of the display.
         * Params :
         * addr	address of the display to control
         * limit	number of digits to be displayed (1..8)
         */
        void stmMax7219_setScanLimit(int addr, int limit);

        /*
         * Set the brightness of the display.
         * Params:
         * addr		the address of the display to control
         * intensity	the brightness of the display. (0..15)
         */
        void stmMax7219_setIntensity(int addr, int intensity);

        /*
         * Switch all Leds on the display off.
         * Params:
         * addr	address of the display to control
         */
        void stmMax7219_clearDisplay(int addr);

        /*
         * Set the status of a single Led.
         * Params :
         * addr	address of the display
         * row	the row of the Led (0..7)
         * col	the column of the Led (0..7)
         * state	If true the led is switched on,
         *		if false it is switched off
         */
        void stmMax7219_setLed(int addr, int row, int col, boolean state);

        /*
         * Set all 8 Led's in a row to a new state
         * Params:
         * addr	address of the display
         * row	row which is to be set (0..7)
         * value	each bit set to 1 will light up the
         *		corresponding Led.
         */
        void stmMax7219_setRow(int addr, int row, unsigned char value);

        /*
         * Set all 8 Led's in a column to a new state
         * Params:
         * addr	address of the display
         * col	column which is to be set (0..7)
         * value	each bit set to 1 will light up the
         *		corresponding Led.
         */
        void stmMax7219_setColumn(int addr, int col, unsigned char value);

        /*
         * Display a hexadecimal digit on a 7-Segment Display
         * Params:
         * addr	address of the display
         * digit	the position of the digit on the display (0..7)
         * value	the value to be displayed. (0x00..0x0F)
         * dp	sets the decimal point.
         */
        void stmMax7219_setDigit(int addr, int digit, unsigned char value, boolean dp);

        /*
         * Display a character on a 7-Segment display.
         * There are only a few characters that make sense here :
         *	'0','1','2','3','4','5','6','7','8','9','0',
         *  'A','b','c','d','E','F','H','L','P',
         *  '.','-','_',' '
         * Params:
         * addr	address of the display
         * digit	the position of the character on the display (0..7)
         * value	the character to be displayed.
         * dp	sets the decimal point.
         */
        void stmMax7219_setChar(int addr, int digit, char value, boolean dp);

        /*
         *  Configures the SPI3 Peripheral.
         *  CLK - PC10
         *  MISO - PC11
         *  MOSI - PC12
          */


void stmMax7219_spiTransfer(int addr, volatile unsigned char opcode, volatile unsigned char data) {
            //Create an array with the data to shift out
            int offset=addr*2;
            int maxbytes=g_stmMax7219.maxDevices*2;
            int i;

            for(i=0;i<maxbytes;i++)  g_stmMax7219.spidata[i]= (unsigned char)0;
            g_stmMax7219.spidata[offset+1]=opcode;
            g_stmMax7219.spidata[offset]=data;
            nCS7219_L;
            //Now shift out the data
            for(i=maxbytes;i>0;i--){
            	stmSpiWrByte(g_stmMax7219.spidata[i-1]);//shiftOut(SPI_MOSI,SPI_CLK,MSBFIRST,g_stmMax7219.spidata[i-1]); //shiftOut(dataPin, clockPin, bitOrder, value)
            }
            nCS7219_H;
}

int stmMax7219_getDeviceCount() {
    return g_stmMax7219.maxDevices;
}
//If true the device goes into power-down mode. Set to false for normal operation.
void stmMax7219_shutdown(int addr, bool b) {
    if(addr<0 || addr>=g_stmMax7219.maxDevices)
        return;
    if(b)
    	 stmMax7219_spiTransfer(addr, OP_SHUTDOWN,0); //--SPI.transfer(slaveSelectPin, val, transferMode)
    else
    	stmMax7219_spiTransfer(addr, OP_SHUTDOWN,1);
}

void stmMax7219_setScanLimit(int addr, int limit) {
    if(addr<0 || addr>=g_stmMax7219.maxDevices)
        return;
    if(limit>=0 && limit<8)
    	stmMax7219_spiTransfer(addr, OP_SCANLIMIT,limit);
}

void stmMax7219_setIntensity(int addr, int intensity) {
    if(addr<0 || addr>=g_stmMax7219.maxDevices)
        return;
    if(intensity>=0 && intensity<16)
    	stmMax7219_spiTransfer(addr, OP_INTENSITY,intensity);
}

void stmMax7219_clearDisplay(int addr) {
    int offset;
    int i;
    if(addr<0 || addr>=g_stmMax7219.maxDevices)
        return;
    offset=addr*8;
    for(i=0;i<8;i++) {
        g_stmMax7219.status[offset+i]=0;
        stmMax7219_spiTransfer(addr, i+1,g_stmMax7219.status[offset+i]);
    }
}

void stmMax7219_setLed(int addr, int row, int column, boolean state) {
    int offset;
    unsigned char val=0x00;

    if(addr<0 || addr>=g_stmMax7219.maxDevices)
        return;
    if(row<0 || row>7 || column<0 || column>7)
        return;
    offset=addr*8;
    val=0B10000000 >> column;
    if(state)
    	g_stmMax7219.status[offset+row]=g_stmMax7219.status[offset+row]|val;
    else {
        val=~val;
        g_stmMax7219.status[offset+row]=g_stmMax7219.status[offset+row]&val;
    }
    stmMax7219_spiTransfer(addr, row+1,g_stmMax7219.status[offset+row]);
}

void stmMax7219_setRow(int addr, int row, unsigned char value) {
    int offset;
    if(addr<0 || addr>=g_stmMax7219.maxDevices)
        return;
    if(row<0 || row>7)
        return;
    offset=addr*8;
    g_stmMax7219.status[offset+row]=value;
    stmMax7219_spiTransfer(addr, row+1,g_stmMax7219.status[offset+row]);
}

void stmMax7219_setColumn(int addr, int col, unsigned char value) {
    unsigned char val;
    int row;

    if(addr<0 || addr>=g_stmMax7219.maxDevices)
        return;
    if(col<0 || col>7)
        return;
    for(row=0;row<8;row++) {
        val=value >> (7-row);
        val=val & 0x01;
        stmMax7219_setLed(addr,row,col,val);
    }
}

void stmMax7219_setDigit(int addr, int digitpos, unsigned char value, boolean dp) {
    int offset;
    unsigned char v;

    if(addr<0 || addr>=g_stmMax7219.maxDevices)
        return;
    if(digitpos<0 || digitpos>7 || value>15)
        return;
    offset=addr*8;
    v = charTable[value];
    if(dp) //show decimal point
        v|=0B10000000;
    g_stmMax7219.status[offset+digitpos]=v;
    stmMax7219_spiTransfer(addr, digitpos+1,v);
}

void stmMax7219_setChar(int addr, int digit, char value, boolean dp) {
    int offset;
    unsigned char index,v;

    if(addr<0 || addr>=g_stmMax7219.maxDevices)
        return;
    if(digit<0 || digit>7)
        return;
    offset=addr*8;
    index=(unsigned char)value;
    if(index >127) {
        //no defined beyond index 127, so we use the space char
        index=32;
    }
    v=charTable[index];//pgm_read_byte_near(charTable + index);
    if(dp)
        v|=0B10000000;
    g_stmMax7219.status[offset+digit]=v;
    stmMax7219_spiTransfer(addr, digit+1,v);
}



/* we always wait a bit between updates of the display */
unsigned long delaytime=500;

stmMax7219_SpiConfig(u8 SpiId, unsigned short spiMode, unsigned char data8or16, int nCS) {
	int i;

	 if(SpiId == 2)
		stmSPI2_Config(2, nCS,spiMode,data8or16);
	 else if(SpiId == 3){
#if(PROCESSOR == STM32F103C8)
		 printf("ERROR> NO SPI3 in this processor\r\n");
		 return;
#else
		stmSPI3_Config(nCS);
#endif
	 }
}

void stmMax7219_config(int numLedModules) {

 	int i;
    if(numLedModules<=0 || numLedModules>8 ) numLedModules=8;
    g_stmMax7219.maxDevices=numLedModules;

    for(i=0;i<64;i++) g_stmMax7219.status[i]=0x00;
    for(i=0;i<g_stmMax7219.maxDevices;i++) {
    	stmMax7219_spiTransfer(i,OP_DISPLAYTEST,0);
        //scanlimit is set to max on startup
        stmMax7219_setScanLimit(i,7);
        //decode is done in source
        stmMax7219_spiTransfer(i,OP_DECODEMODE,0);
        stmMax7219_clearDisplay(i);
        //we go into shutdown-mode on startup
        stmMax7219_shutdown(i,true);
    }

    // The MAX72XX is in power-saving mode on startup, we have to do a wakeup call
    stmMax7219_shutdown(0,false);
    // Set the brightness to a medium values
    stmMax7219_setIntensity(0,8);
    // and clear the display
    stmMax7219_clearDisplay(0);

    stmMax7219_shutdown(1,false);
    // Set the brightness to a medium values
    stmMax7219_setIntensity(1,8);
    // and clear the display
    stmMax7219_clearDisplay(1);

	printf("MAX7219> LED-SPI CONFIG DONE\r\n");

}



/*
 This method will display the characters for the
 word "Arduino" one after the other on digit 0.
 */
void stmMax7219_writeArduinoOn7Segment() {
  stmMax7219_setChar(0,0,'a',false);
  delayms(delaytime);
  stmMax7219_setRow(0,0,0x05);
  delayms(delaytime);
  stmMax7219_setChar(0,0,'d',false);
  delayms(delaytime);
  stmMax7219_setRow(0,0,0x1c);
  delayms(delaytime);
  stmMax7219_setRow(0,0,0B00010000);
  delayms(delaytime);
  stmMax7219_setRow(0,0,0x15);
  delayms(delaytime);
  stmMax7219_setRow(0,0,0x1D);
  delayms(delaytime);
  stmMax7219_clearDisplay(0);
  delayms(delaytime);
}
//display "iEEE1588"
void stmMax7219_writePTPHELLOon7Segments() {
  stmMax7219_setRow(0,7,0B00010000); //'i'
  delayms(delaytime);
  stmMax7219_setChar(0,6,'E',false);
  delayms(delaytime);
  stmMax7219_setChar(0,5,'E',false);
  delayms(delaytime);
  stmMax7219_setChar(0,4,'E',false);
  delayms(delaytime);
  stmMax7219_setChar(0,3,'1',false);
  delayms(delaytime);
  stmMax7219_setChar(0,2,'5',false);
  delayms(delaytime);
  stmMax7219_setChar(0,1,'8',false);
  delayms(delaytime);
  stmMax7219_setChar(0,0,'8',true);
  delayms(delaytime);

  stmMax7219_setChar(1,6,'d',false);
  delayms(delaytime);
  stmMax7219_setChar(1,5,'E',false);
  delayms(delaytime);
  stmMax7219_setRow(1,4,0B11111011);//'m'
  delayms(delaytime);
  stmMax7219_setChar(1,3,'0',true);
  delayms(delaytime);

  delayms(1000);

  stmMax7219_clearDisplay(0);
  stmMax7219_clearDisplay(1);
  delayms(delaytime);
}

/* -- YOON....
void stmMax7219_writeHelloAramiOn7Segment() {
  stmMax7219_setChar(0,0,'h',false);
  delayms(delaytime);
  stmMax7219_setRow(0,0,0x05);
  delayms(delaytime);
  stmMax7219_setChar(0,0,'d',false);
  delayms(delaytime);
  stmMax7219_setRow(0,0,0x1c);
  delayms(delaytime);
  stmMax7219_setRow(0,0,0B00010000);
  delayms(delaytime);
  stmMax7219_setRow(0,0,0x15);
  delayms(delaytime);
  stmMax7219_setRow(0,0,0x1D);
  delayms(delaytime);
  stmMax7219_clearDisplay(0);
  delayms(delaytime);
}
*/


/*
  This method will scroll all the hexa-decimal
 numbers and letters on the display. You will need at least
 four 7-Segment digits. otherwise it won't really look that good.
 */
void stmMax7219_scrollDigits() {
	int i;
  for(i=0;i<13;i++) {
	    stmMax7219_setDigit(0,7,i,false);
	    stmMax7219_setDigit(0,6,i+1,false);
	    stmMax7219_setDigit(0,5,i+2,false);
	    stmMax7219_setDigit(0,4,i+3,false);
	    stmMax7219_setDigit(0,3,i+4,false);
	    stmMax7219_setDigit(0,2,i+5,false);
	    stmMax7219_setDigit(0,1,i+6,false);
	    stmMax7219_setDigit(0,0,i+7,false);
	    if(g_stmMax7219.maxDevices == 2){
		    stmMax7219_setDigit(1,7,i,false);
		    stmMax7219_setDigit(1,6,i+1,false);
		    stmMax7219_setDigit(1,5,i+2,false);
		    stmMax7219_setDigit(1,4,i+3,false);
		    stmMax7219_setDigit(1,3,i+4,false);
		    stmMax7219_setDigit(1,2,i+5,false);
		    stmMax7219_setDigit(1,1,i+6,false);
		    stmMax7219_setDigit(1,0,i+7,false);
	    }

    delayms(delaytime);
  }
  stmMax7219_clearDisplay(0);
  delayms(delaytime);
}
//YOON
//Use KYX-3662AS Common Cathode LED (6 digits)
void stmMax7219_showClock(unsigned h, unsigned m, unsigned s) {
	int hh,hl;
	int mh,ml;
	int sh,sl;

	stmMax7219_clearDisplay(0);
	hl = h%10;
	hh = h/10;
	stmMax7219_setDigit(0,0,hh,false);
	stmMax7219_setDigit(0,1,hl,true);
	ml = m%10;
	mh = m/10;

    stmMax7219_setDigit(0,2,mh,false);
	stmMax7219_setDigit(0,3,ml,true);
	sl = s%10;
	sh = s/10;

    stmMax7219_setDigit(0,4,sh,false);
    stmMax7219_setDigit(0,5,sl,true);
}

void stmMax7219_show4digits(unsigned d) {
	unsigned int i,j,k,l;
	unsigned char str[5];
	sprintf(str,"%u",d);

	/*i = d/1000;
	j = d/100;
	k = d/10;
	l = d%10;*/
	i = str[0]-'0';
	j = str[1]-'0';
	k = str[2]-'0';
	l = str[3]-'0';

	stmMax7219_clearDisplay(0);
	stmMax7219_setDigit(0,0,i,false);
	stmMax7219_setDigit(0,1,j,false);
    stmMax7219_setDigit(0,2,k,false);
	stmMax7219_setDigit(0,3,l,true);
}


void stmMax7219_ClockLoop() {
	int sec=0;
	int m = 0;
	int h=0;
	int nCS=1;
	int SpiId = 2;
	int numLEDmodule = 1;
	//GuiInit("LED-SPI Test");
	delayms(1000);

	stmSPI2_Config(2, nCS1,stmSPIMODE3,stmSPIDATA8);//2=2Mbps

	stmMax7219_config(1);

	while(1){
		//stmMax7219_writeArduinoOn7Segment();
		//stmMax7219_scrollDigits();
		stmMax7219_showClock(h, m, sec) ;
		delayms(100);
		if(sec==59){
			sec = 0;
			m++;
			if(m==60){
				m=0;
				h++;
				if(h==24){
					h=0;
				}
			}
		}else
			sec++;

	}
}


void stmMax7219_4Digit_Loop(int nCS) {

	int i=0;
	int SpiId = 2;
	int numLEDmodule = 1;

	printf("4Digit LED-MAX7219 SPI Test.");
	delayms(1000);

	//stmMax7219_Init(SpiId, stmSPIMODE3, stmSPIDATA8, numLEDmodule, nCS);
	stmMax7219_SpiConfig(SpiId, stmSPIMODE0, stmSPIDATA8,  nCS);
	stmMax7219_config(numLEDmodule);

	while(1){
		//stmMax7219_scrollDigits();
		stmMax7219_show4digits(i);
		delayms(100);
		i++;
	}
}

