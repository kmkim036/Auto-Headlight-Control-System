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

#include "yInc.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"

//+-----------------+-----------+-----------+-----------+---------+---------+
//|                 |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| ULED            | PB14      |PC4        |PE15       | <==     | PG7
//+-----------------+-----------+-----------+-----------+---------+---------+
//| BUTTON          |           |PC5(H)     |PD11(index)| PD11(L) |
//+-----------------+-----------+-----------+-----------+---------+
//| BEEP            |           |PB13       |PD14       | <==     |
//+-----------------+-----------+-----------+-----------+---------+
//| QEI             |           |PB0,1,12   |PD12,13,11 | PD12,13 |
//+-----------------+-----------+-----------+-----------+---------+
//| nCS0            |           |           |           | PD10    |
//+-----------------+-----------+-----------+-----------+---------+
//| nCS1            |           |           |           | PD15    |
//+-----------------+-----------+-----------+-----------+---------+
//| nCS2            |           |           |           | PE5     |
//+-----------------+-----------+-----------+-----------+---------+
//*H= Active High/L=Active Low
//*F= FallingEdge
//========= THE VERY FIRST CONFIG for Various Chip Select Pins =================================
//use nCS0 of PA3
//#define nCS7219_1      {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);}
//#define nCS7219_0      {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);}
//use nCS0 of PA4
//#define nCS7219_1      {GPIO_SetBits(GPIOD, GPIO_Pin_2);}
//#define nCS7219_0      {GPIO_ResetBits(GPIOD, GPIO_Pin_2);}

//use nCS0 of PA15
//#define nCS0_7219_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
//#define nCS0_7219_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}

#define MAX7219_FOR_7SEG 		1
#define MAX7219_FOR_DOT_MATRIX 	2
#define MAX7219_FOR MAX7219_FOR_7SEG //MAX7219_FOR_DOT_MATRIX

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_GD32F130FX)
	#if (MCU_MODULE_VER >= 35)
	//use nCS0 of PB12
	#define nCS0_7219_1      {GPIO_SetBits(GPIOB, GPIO_Pin_12);}
	#define nCS0_7219_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_12);}
	#else
	#define nCS0_7219_1      {GPIO_SetBits(GPIOB, GPIO_Pin_12);}
	#define nCS0_7219_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_12);}
	#endif
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
//use nCS0 of PA15
#define nCS0_7219_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
#define nCS0_7219_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
#endif

//use nCS1 of PD15
#define nCS1_7219_1      {GPIO_SetBits(GPIOD, GPIO_Pin_15);}
#define nCS1_7219_0      {GPIO_ResetBits(GPIOD, GPIO_Pin_15);}

//use nCS1 of PD15
#define nCS2_7219_1      {GPIO_SetBits(GPIOE, GPIO_Pin_5);}
#define nCS2_7219_0      {GPIO_ResetBits(GPIOE, GPIO_Pin_5);}
/*
#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
//use nCS0 of PD10
#define nCS0_7219_1      {GPIO_SetBits(GPIOD, GPIO_Pin_10);}
#define nCS0_7219_0      {GPIO_ResetBits(GPIOD, GPIO_Pin_10);}

//use nCS1 of PD15
#define nCS1_7219_1      {GPIO_SetBits(GPIOD, GPIO_Pin_15);}
#define nCS1_7219_0      {GPIO_ResetBits(GPIOD, GPIO_Pin_15);}
#endif
*/
//======== The main chip select signal : Use nCS0 for 7219 =========
//#define nCS7219_1 nCS0_7219_1
//#define nCS7219_0 nCS0_7219_0

//Use nCS1 for 7219
//#define nCS7219_1 nCS1_7219_1
//#define nCS7219_0 nCS1_7219_0

#define nCS7219_1 nCS0_7219_1
#define nCS7219_0 nCS0_7219_0


extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern void stmSPI3_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern void stmSpiWrByte(unsigned char inbyte);
extern unsigned char stmSpiRdByte();


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
        void stmMax7219_setLed(int addr, int row, int col, unsigned char state);

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
        void stmMax7219_setDigit(int addr, int digit, unsigned char value, unsigned char dp);

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
        void stmMax7219_setChar(int addr, int digit, char value, unsigned char dp);

        /*
         *  Configures the SPI3 Peripheral.
         *  CLK - PC10
         *  MISO - PC11
         *  MOSI - PC12
          */
#if (MAX7219_FOR == MAX7219_FOR_7SEG)
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
#elif (MAX7219_FOR == MAX7219_FOR_DOT_MATRIX)
         const static unsigned char font5_7 [] = {      //Numeric Font Matrix (Arranged as 7x font data + 1x kerning data)
             0B00000000,	//Space (Char 0x20)
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             3,//cambias el tama?o del espacio entre letras

             0B01000000,	//!
             0B01000000,
             0B01000000,
             0B01000000,
             0B01000000,
             0B00000000,
             0B01000000,
             2,

             0B10100000,	//"
             0B10100000,
             0B10100000,
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             4,

             0B01010000,	//#
             0B01010000,
             0B11111000,
             0B01010000,
             0B11111000,
             0B01010000,
             0B01010000,
             6,

             0B00100000,	//$
             0B01111000,
             0B10100000,
             0B01110000,
             0B00101000,
             0B11110000,
             0B00100000,
             6,

             0B11000000,	//%
             0B11001000,
             0B00010000,
             0B00100000,
             0B01000000,
             0B10011000,
             0B00011000,
             6,

             0B01100000,	//&
             0B10010000,
             0B10100000,
             0B01000000,
             0B10101000,
             0B10010000,
             0B01101000,
             6,

             0B11000000,	//'
             0B01000000,
             0B10000000,
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             3,

             0B00100000,	//(
             0B01000000,
             0B10000000,
             0B10000000,
             0B10000000,
             0B01000000,
             0B00100000,
             4,

             0B10000000,	//)
             0B01000000,
             0B00100000,
             0B00100000,
             0B00100000,
             0B01000000,
             0B10000000,
             4,

             0B00000000,	//*
             0B00100000,
             0B10101000,
             0B01110000,
             0B10101000,
             0B00100000,
             0B00000000,
             6,

             0B00000000,	//+
             0B00100000,
             0B00100000,
             0B11111000,
             0B00100000,
             0B00100000,
             0B00000000,
             6,

             0B00000000,	//,
             0B00000000,
             0B00000000,
             0B00000000,
             0B11000000,
             0B01000000,
             0B10000000,
             3,

             0B00000000,	//-
             0B00000000,
             0B11111000,
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             6,

             0B00000000,	//.
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             0B11000000,
             0B11000000,
             3,

             0B00000000,	///
             0B00001000,
             0B00010000,
             0B00100000,
             0B01000000,
             0B10000000,
             0B00000000,
             6,

             0B01110000,	//0
             0B10001000,
             0B10011000,
             0B10101000,
             0B11001000,
             0B10001000,
             0B01110000,
             6,

             0B01000000,	//1
             0B11000000,
             0B01000000,
             0B01000000,
             0B01000000,
             0B01000000,
             0B11100000,
             4,

             0B01110000,	//2
             0B10001000,
             0B00001000,
             0B00010000,
             0B00100000,
             0B01000000,
             0B11111000,
             6,

             0B11111000,	//3
             0B00010000,
             0B00100000,
             0B00010000,
             0B00001000,
             0B10001000,
             0B01110000,
             6,

             0B00010000,	//4
             0B00110000,
             0B01010000,
             0B10010000,
             0B11111000,
             0B00010000,
             0B00010000,
             6,

             0B11111000,	//5
             0B10000000,
             0B11110000,
             0B00001000,

             0B00001000,
             0B10001000,
             0B01110000,
             6,

             0B00110000,	//6
             0B01000000,
             0B10000000,
             0B11110000,
             0B10001000,
             0B10001000,
             0B01110000,
             6,

             0B11111000,	//7
             0B10001000,
             0B00001000,
             0B00010000,
             0B00100000,
             0B00100000,
             0B00100000,
             6,

             0B01110000,	//8
             0B10001000,
             0B10001000,
             0B01110000,
             0B10001000,
             0B10001000,
             0B01110000,
             6,

             0B01110000,	//9
             0B10001000,
             0B10001000,
             0B01111000,
             0B00001000,
             0B00010000,
             0B01100000,
             6,

             0B00000000,	//:
             0B11000000,
             0B11000000,
             0B00000000,
             0B11000000,
             0B11000000,
             0B00000000,
             3,

             0B00000000,	//;
             0B11000000,
             0B11000000,
             0B00000000,
             0B11000000,
             0B01000000,
             0B10000000,
             3,

             0B00010000,	//<
             0B00100000,
             0B01000000,
             0B10000000,
             0B01000000,
             0B00100000,
             0B00010000,
             5,

             0B00000000,	//=
             0B00000000,
             0B11111000,
             0B00000000,
             0B11111000,
             0B00000000,
             0B00000000,
             6,

             0B10000000,	//>
             0B01000000,
             0B00100000,
             0B00010000,
             0B00100000,
             0B01000000,
             0B10000000,
             5,

             0B01110000,	//?
             0B10001000,
             0B00001000,
             0B00010000,
             0B00100000,
             0B00000000,
             0B00100000,
             6,

             0B01110000,	//@
             0B10001000,
             0B00001000,
             0B01101000,
             0B10101000,
             0B10101000,
             0B01110000,
             6,

             0B01110000,	//A
             0B10001000,
             0B10001000,
             0B10001000,
             0B11111000,
             0B10001000,
             0B10001000,
             6,

             0B11110000,	//0B
             0B10001000,
             0B10001000,
             0B11110000,
             0B10001000,
             0B10001000,
             0B11110000,
             6,

             0B01110000,	//C
             0B10001000,
             0B10000000,
             0B10000000,
             0B10000000,
             0B10001000,
             0B01110000,
             6,

             0B11100000,	//D
             0B10010000,
             0B10001000,
             0B10001000,
             0B10001000,
             0B10010000,
             0B11100000,
             6,

             0B11111000,	//E
             0B10000000,
             0B10000000,
             0B11110000,
             0B10000000,
             0B10000000,
             0B11111000,
             6,

             0B11111000,	//F
             0B10000000,
             0B10000000,
             0B11110000,
             0B10000000,
             0B10000000,
             0B10000000,
             6,

             0B01110000,	//G
             0B10001000,
             0B10000000,
             0B10111000,
             0B10001000,
             0B10001000,
             0B01111000,
             6,

             0B10001000,	//H
             0B10001000,
             0B10001000,
             0B11111000,
             0B10001000,
             0B10001000,
             0B10001000,
             6,

             0B11100000,	//I
             0B01000000,
             0B01000000,
             0B01000000,
             0B01000000,
             0B01000000,
             0B11100000,
             4,

             0B00111000,	//J
             0B00010000,
             0B00010000,
             0B00010000,
             0B00010000,
             0B10010000,
             0B01100000,
             6,

             0B10001000,	//K
             0B10010000,
             0B10100000,
             0B11000000,
             0B10100000,
             0B10010000,
             0B10001000,
             6,

             0B10000000,	//L
             0B10000000,
             0B10000000,
             0B10000000,
             0B10000000,
             0B10000000,
             0B11111000,
             6,

             0B10001000,	//M
             0B11011000,
             0B10101000,
             0B10101000,
             0B10001000,
             0B10001000,
             0B10001000,
             6,

             0B10001000,	//N
             0B10001000,
             0B11001000,
             0B10101000,
             0B10011000,
             0B10001000,
             0B10001000,
             6,

             0B01110000,	//O
             0B10001000,
             0B10001000,
             0B10001000,
             0B10001000,
             0B10001000,
             0B01110000,
             6,

             0B11110000,	//P
             0B10001000,
             0B10001000,
             0B11110000,
             0B10000000,
             0B10000000,
             0B10000000,
             6,

             0B01110000,	//Q
             0B10001000,
             0B10001000,
             0B10001000,
             0B10101000,
             0B10010000,
             0B01101000,
             6,

             0B11110000,	//R
             0B10001000,
             0B10001000,
             0B11110000,
             0B10100000,
             0B10010000,
             0B10001000,
             6,

             0B01111000,	//S
             0B10000000,
             0B10000000,
             0B01110000,
             0B00001000,
             0B00001000,
             0B11110000,
             6,

             0B11111000,	//T
             0B00100000,
             0B00100000,
             0B00100000,
             0B00100000,
             0B00100000,
             0B00100000,
             6,

             0B10001000,	//U
             0B10001000,
             0B10001000,
             0B10001000,
             0B10001000,
             0B10001000,

             0B01110000,
             6,

             0B10001000,	//V
             0B10001000,
             0B10001000,
             0B10001000,
             0B10001000,
             0B01010000,
             0B00100000,
             6,

             0B10001000,	//W
             0B10001000,
             0B10001000,
             0B10101000,
             0B10101000,
             0B10101000,
             0B01010000,
             6,

             0B10001000,	//X
             0B10001000,
             0B01010000,
             0B00100000,
             0B01010000,
             0B10001000,
             0B10001000,
             6,

             0B10001000,	//Y
             0B10001000,
             0B10001000,
             0B01010000,
             0B00100000,
             0B00100000,
             0B00100000,
             6,

             0B11111000,	//Z
             0B00001000,
             0B00010000,
             0B00100000,
             0B01000000,
             0B10000000,
             0B11111000,
             6,

             0B11100000,	//[
             0B10000000,
             0B10000000,
             0B10000000,
             0B10000000,
             0B10000000,
             0B11100000,
             4,

             0B00000000,	//(0Backward Slash)
             0B10000000,
             0B01000000,
             0B00100000,
             0B00010000,
             0B00001000,
             0B00000000,
             6,

             0B11100000,	//]
             0B00100000,
             0B00100000,
             0B00100000,
             0B00100000,
             0B00100000,
             0B11100000,
             4,

             0B00100000,	//^
             0B01010000,
             0B10001000,
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             6,

             0B00000000,	//_
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             0B11111000,
             6,

             0B10000000,	//`
             0B01000000,
             0B00100000,
             0B00000000,
             0B00000000,
             0B00000000,
             0B00000000,
             4,

             0B00000000,	//a
             0B00000000,
             0B01110000,
             0B00001000,
             0B01111000,
             0B10001000,
             0B01111000,
             6,

             0B10000000,	//0B
             0B10000000,
             0B10110000,
             0B11001000,
             0B10001000,
             0B10001000,
             0B11110000,
             6,

             0B00000000,	//c
             0B00000000,
             0B01110000,
             0B10001000,
             0B10000000,
             0B10001000,
             0B01110000,
             6,

             0B00001000,	//d
             0B00001000,
             0B01101000,
             0B10011000,
             0B10001000,
             0B10001000,
             0B01111000,
             6,

             0B00000000,	//e
             0B00000000,
             0B01110000,
             0B10001000,
             0B11111000,
             0B10000000,
             0B01110000,
             6,

             0B00110000,	//f
             0B01001000,
             0B01000000,
             0B11100000,
             0B01000000,
             0B01000000,
             0B01000000,
             6,

             0B00000000,	//g
             0B01111000,
             0B10001000,
             0B10001000,
             0B01111000,
             0B00001000,
             0B01110000,
             6,

             0B10000000,	//h
             0B10000000,
             0B10110000,
             0B11001000,
             0B10001000,
             0B10001000,
             0B10001000,
             6,

             0B01000000,	//i
             0B00000000,
             0B11000000,
             0B01000000,
             0B01000000,
             0B01000000,
             0B11100000,
             4,

             0B00010000,	//j
             0B00000000,
             0B00110000,
             0B00010000,
             0B00010000,
             0B10010000,
             0B01100000,
             5,

             0B10000000,	//k
             0B10000000,
             0B10010000,
             0B10100000,
             0B11000000,
             0B10100000,
             0B10010000,
             5,

             0B11000000,	//l
             0B01000000,
             0B01000000,
             0B01000000,
             0B01000000,
             0B01000000,
             0B11100000,
             4,

             0B00000000,	//m
             0B00000000,
             0B11010000,
             0B10101000,
             0B10101000,
             0B10001000,
             0B10001000,
             6,

             0B00000000,	//n
             0B00000000,
             0B10110000,
             0B11001000,
             0B10001000,
             0B10001000,
             0B10001000,
             6,

             0B00000000,	//o
             0B00000000,
             0B01110000,
             0B10001000,
             0B10001000,
             0B10001000,
             0B01110000,
             6,

             0B00000000,	//p
             0B00000000,
             0B11110000,
             0B10001000,
             0B11110000,
             0B10000000,
             0B10000000,
             6,

             0B00000000,	//q
             0B00000000,
             0B01101000,
             0B10011000,
             0B01111000,
             0B00001000,
             0B00001000,
             6,

             0B00000000,	//r
             0B00000000,
             0B10110000,
             0B11001000,
             0B10000000,
             0B10000000,
             0B10000000,
             6,

             0B00000000,	//s
             0B00000000,
             0B01110000,
             0B10000000,
             0B01110000,
             0B00001000,
             0B11110000,
             6,

             0B01000000,	//t
             0B01000000,
             0B11100000,
             0B01000000,
             0B01000000,
             0B01001000,
             0B00110000,
             6,

             0B00000000,	//u
             0B00000000,
             0B10001000,
             0B10001000,
             0B10001000,
             0B10011000,
             0B01101000,
             6,

             0B00000000,	//v
             0B00000000,
             0B10001000,
             0B10001000,
             0B10001000,
             0B01010000,
             0B00100000,
             6,

             0B00000000,	//w
             0B00000000,
             0B10001000,
             0B10101000,
             0B10101000,
             0B10101000,
             0B01010000,
             6,

             0B00000000,	//x
             0B00000000,
             0B10001000,
             0B01010000,
             0B00100000,
             0B01010000,
             0B10001000,
             6,

             0B00000000,	//y
             0B00000000,
             0B10001000,
             0B10001000,
             0B01111000,
             0B00001000,
             0B01110000,
             6,

             0B00000000,	//z
             0B00000000,
             0B11111000,
             0B00010000,
             0B00100000,
             0B01000000,
             0B11111000,
             6,

             0B00100000,	//{
             0B01000000,
             0B01000000,
             0B10000000,
             0B01000000,
             0B01000000,
             0B00100000,
             4,

             0B10000000,	//|
             0B10000000,
             0B10000000,
             0B10000000,
             0B10000000,
             0B10000000,
             0B10000000,
             2,

             0B10000000,	//}
             0B01000000,
             0B01000000,
             0B00100000,
             0B01000000,
             0B01000000,
             0B10000000,
             4,

             0B00000000,	//~
             0B00000000,
             0B00000000,
             0B01101000,
             0B10010000,
             0B00000000,
             0B00000000,
             6,

             0B01100000,	// (Char 0x7F)
             0B10010000,
             0B10010000,
             0B01100000,
             0B00000000,
             0B00000000,
             0B00000000,
             5,

             0B00000000,	// smiley
             0B01100000,
             0B01100110,
             0B00000000,
             0B10000001,
             0B01100110,
             0B00011000,
             5
         };
#endif

void stmMax7219_spiTransfer(int addr, volatile unsigned char opcode, volatile unsigned char data) {
            //Create an array with the data to shift out
            int offset=addr*2;
            int maxbytes=g_stmMax7219.maxDevices*2;
            int i;

            for(i=0;i<maxbytes;i++)  g_stmMax7219.spidata[i]= (unsigned char)0;
            g_stmMax7219.spidata[offset+1]=opcode;
            g_stmMax7219.spidata[offset]=data;
            nCS7219_0;
            //Now shift out the data
            for(i=maxbytes;i>0;i--){
            	stmSpiWrByte(g_stmMax7219.spidata[i-1]);//shiftOut(SPI_MOSI,SPI_CLK,MSBFIRST,g_stmMax7219.spidata[i-1]); //shiftOut(dataPin, clockPin, bitOrder, value)
            }
            nCS7219_1;
}


int stmMax7219_getDeviceCount() {
    return g_stmMax7219.maxDevices;
}
//If true the device goes into power-down mode. Set to false for normal operation.
void stmMax7219_shutdown(int addr, bool b) {
    if(addr<0 || addr>=g_stmMax7219.maxDevices)
        return;
    //nCS7219_0;
    if(b)
    	 stmMax7219_spiTransfer(addr, OP_SHUTDOWN,0); //--SPI.transfer(slaveSelectPin, val, transferMode)
    else
    	stmMax7219_spiTransfer(addr, OP_SHUTDOWN,1);
    //nCS7219_0;
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


#if (MAX7219_FOR == MAX7219_FOR_7SEG)
void stmMax7219_setLed(int addr, int row, int column, unsigned char state) {
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


void stmMax7219_setDigit(int addr, int digitpos, unsigned char value, unsigned char dp) {
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

void stmMax7219_setChar(int addr, int digit, char value, unsigned char dp) {
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

#else

const int numDevices = 4;      // number of MAX7219s used in this case 2
const long scrollDelay = 140;   // adjust scrolling speed
unsigned long bufferLong [14] = {0};
//LedControl lc=LedControl(12,11,10,numDevices);//DATA | CLK | CS/LOAD | number of matrices
const unsigned char scrollText[] ={" WELCOME ANRLAB !!! 02-300-0124  "};


// Rotate the buffer
void rotateBufferLong(){
	int a;
	unsigned char b;
    for (a=0;a<7;a++){                      // Loop 7 times for a 5x7 font
        unsigned long x = bufferLong [a*2];     // Get low buffer entry
        b = bitRead(x,31);                 // Copy high order bit that gets lost in rotation
        x = x<<1;                               // Rotate left one bit
        bufferLong [a*2] = x;                   // Store new low buffer
        x = bufferLong [a*2+1];                 // Get high buffer entry
        x = x<<1;                               // Rotate left one bit
        bitWrite(x,0,b);                        // Store saved bit
        bufferLong [a*2+1] = x;                 // Store new high buffer
    }
}
// Load character into scroll buffer
void loadBufferLong(int ascii){
	int a;
	unsigned char count,x;
    if (ascii >= 0x20 && ascii <=0x7f){
        for (a=0;a<7;a++){                      // Loop 7 times for a 5x7 font
            unsigned long c = font5_7[((ascii - 0x20) * 8) + a];     // Index into character table to get row data
            unsigned long x = bufferLong [a*2];     // Load current scroll buffer
            x = x | c;                              // OR the new character onto end of current
            bufferLong [a*2] = x;                   // Store in buffer
        }
        count = font5_7[((ascii - 0x20) * 8) + 7];     // Index into character table for kerning data
        for (x=0; x<count;x++){
            rotateBufferLong();
            printBufferLong();
            delayms(scrollDelay);
        }
    }
}
void scrollFont() {
	int counter;
    for (counter=0x20;counter<0x80;counter++){
        loadBufferLong(counter);
        delayms(500);
    }
}

// Scroll Message
void scrollMessage(const unsigned char * messageString) {
    int counter = 0;
    int myChar=0;
    do {
        // read back a char
        myChar =  messageString[counter];
        if (myChar != 0){
            loadBufferLong(myChar);
        }
        counter++;
    }
    while (myChar != 0);
}

// Display Buffer on LED matrix
void printBufferLong(){
	int a;
	unsigned char y;
  for (a=0;a<7;a++){                    // Loop 7 times for a 5x7 font
    unsigned long x = bufferLong [a*2+1];   // Get high buffer entry
    y = x;                             // Mask off first character
    stmMax7219_setRow(3,a,y);                       // Send row to relevent MAX7219 chip
    x = bufferLong [a*2];                   // Get low buffer entry
    y = (x>>24);                            // Mask off second character
    stmMax7219_setRow(2,a,y);                       // Send row to relevent MAX7219 chip
    y = (x>>16);                            // Mask off third character
    stmMax7219_setRow(1,a,y);                       // Send row to relevent MAX7219 chip
    y = (x>>8);                             // Mask off forth character
    stmMax7219_setRow(0,a,y);                       // Send row to relevent MAX7219 chip
  }
}
#endif

/* we always wait a bit between updates of the display */
unsigned long delaytime=500;

stmMax7219_Init(u8 SpiId, unsigned short spiMode, unsigned char data8or16,int numLedModules, int nCS) {
	int i;

	if(SpiId == 3)
		stmSPI3_Config(1, nCS, spiMode, data8or16);//stmSPI3_Config(nCS);
	else if(SpiId == 2)
		stmSPI2_Config(1,nCS,spiMode,data8or16);

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
	printf("MAX7219> LED-SPI CONFIG DONE\r\n");
}

void stmMax7219_config(unsigned char numLedModules) {

	int i;
	for(i=0;i<numLedModules;i++){
		// The MAX72XX is in power-saving mode on startup, we have to do a wakeup call
		stmMax7219_shutdown(i,false);
		// Set the brightness to a medium values
		stmMax7219_setIntensity(i,8);
		// and clear the display
		stmMax7219_clearDisplay(i);
	}
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

void stmMax7219_showClock4(unsigned h, unsigned m) {
	int hh,hl;
	int mh,ml;

	stmMax7219_clearDisplay(0);

	hl = h%10;
	hh = h/10;
	stmMax7219_setDigit(0,0,hh,false);
	stmMax7219_setDigit(0,1,hl,true);

	ml = m%10;
	mh = m/10;
    stmMax7219_setDigit(0,2,mh,false);
	stmMax7219_setDigit(0,3,ml,true);

}

void stmMax7219_show4digits(unsigned d) {
	unsigned char i,j,k,l;
	d = d & 0xFFFF;

	i = (d & 0xF000) >> 12;
	j = (d & 0x0F00) >> 8;
	k = (d & 0x00F0) >> 4;
	l = (d & 0x000F);

	stmMax7219_clearDisplay(0);
	stmMax7219_setDigit(0,0,i,false);
	stmMax7219_setDigit(0,1,j,true);
    stmMax7219_setDigit(0,2,k,false);
	stmMax7219_setDigit(0,3,l,true);
}

void stmMax7219_ClockLoop() {
	int sec=0;
	int m = 0;
	int h=0;
	int nCS=0;

	int numLEDmodule = 1;
	printf("LED-SPI Clock Test\r\n");
	delayms(1000);
#if(PROCESSOR == PROCESSOR_STM32F103C8T6)
	stmMax7219_Init(USE_SPI2, stmSPIMODE0, stmSPIDATA8, numLEDmodule, nCS);
#elif(PROCESSOR == PROCESSOR_STM32F107VCT)
	stmMax7219_Init(USE_SPI3, stmSPIMODE0, stmSPIDATA8, numLEDmodule, nCS);
#endif

	stmMax7219_config(numLEDmodule);
	while(1){
		//stmMax7219_writeArduinoOn7Segment();
		//stmMax7219_scrollDigits();
		stmMax7219_showClock(h, m, sec) ;
		//stmMax7219_showClock4(h, m) ;
		delayms(10);
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





//================ for 16 digits clock (PTP) ================================
void max7219_16digit_Config(){
#ifdef USE_MAX7219
	int nCS=1;
	max7219_Init(2, 2,nCS);
	max7219_config();
	max7219_writePTPHELLOon7Segments();
#endif

}

void PTP_max7219_write16digits(
		unsigned char hh,
		unsigned char mm,
		unsigned char ss,
		unsigned long  ns
		)
{
	char str[10];
	unsigned char digit1,digit2,digit3;
	//hh.mm.ss.-n==nn.nnn.nnn.
	//hh.
	sprintf(str,"%02d",hh); digit1 = str[0] - '0';digit2 = str[1] - '0';
	max7219_setDigit(0,7,digit1,false); max7219_setDigit(0,6,digit2,true); delayms(1);
	//mm.
	sprintf(str,"%02d",mm); digit1 = str[0] - '0';digit2 = str[1] - '0';
	max7219_setDigit(0,5,digit1,false); max7219_setDigit(0,4,digit2,true); delayms(1);
	//ss.
	sprintf(str,"%02d",ss); digit1 = str[0] - '0';digit2 = str[1] - '0';
	max7219_setDigit(0,3,digit1,false); max7219_setDigit(0,2,digit2,true); delayms(1);
	//space
	//ns
	sprintf(str,"%09d",ns); digit1 = str[0] - '0';digit2 = str[1] - '0';digit3 = str[2] - '0';
	max7219_setDigit(0,0,digit1,false); max7219_setDigit(1,7,digit2,false); max7219_setDigit(1,6,digit3,true); delayms(1);

	digit1 = str[3] - '0';digit2 = str[4] - '0';digit3 = str[5] - '0';
	max7219_setDigit(1,5,digit1,false); max7219_setDigit(1,4,digit2,false); max7219_setDigit(1,3,digit3,true); delayms(1);

	digit1 = str[6] - '0';digit2 = str[7] - '0';digit3 = str[8] - '0';
	max7219_setDigit(1,2,digit1,false); max7219_setDigit(1,1,digit2,false); max7219_setDigit(1,0,digit3,true); delayms(1);
	delayms(10);
}
//=====================================================
void stmMax7219_4Digit_Loop() {

	int i=0;
	int nCS=0;
	int SpiId = 2;
	int numLEDmodule = 1;

	printf("4Digit LED-MAX7219 SPI Test.");
	delayms(1000);

	stmMax7219_Init(USE_SPIX, stmSPIMODE3, stmSPIDATA8, numLEDmodule, nCS);//2);
	stmMax7219_config(numLEDmodule);

	i = 0;
	while(1){
		//stmMax7219_scrollDigits();
		stmMax7219_show4digits(i);
		delayms(10);
		i++;
	}
}

//PA0
void dingDong_Config(){

	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void dingDong_Do(){

	GPIO_SetBits(GPIOA, GPIO_Pin_0);
	delayms(9000);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);

}
//=============== dotMatrix
void stmMax7219_DotMatrix_Loop() {

	int i=0;
	int nCS=0;
	int numLEDmodule = 4;

	printf("Quad DotMatrixLED-MAX7219 SPI Test.");
	delayms(1000);

	stmMax7219_Init(USE_SPIX, stmSPIMODE3, stmSPIDATA8, numLEDmodule, nCS);//2);
	stmMax7219_config(numLEDmodule);

	dingDong_Config(); //PA0

	while(1){
		//stmMax7219_scrollDigits();
		//scrollMessage(scrollText);
		dingDong_Do();
		delayms(1000);
	}
}
