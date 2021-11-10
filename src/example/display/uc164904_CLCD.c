/*
  LCD - A library for controling CLCD(UC164904/KS0074 Samsung) with SPI
 * linux... auxdisplay/charlcd.c and panel.c
 * 	  SPI : 1Mbps, 8 bit mode
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
#include "ySpiDrv.h"

// HD44780 commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00 //We used
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00   //We used
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00  //We used

// flags for backlight control
#define LCD_BACKLIGHT 0x00//08//00
#define LCD_NOBACKLIGHT 0x08//00//0x08//80

#define PCF9547_ENBIT 0x04//10  // Enable bit
#define PCF9547_RWBIT 0x02//20  // Read/Write bit
#define PCF9547_RSBIT 0x01//40  // Register select bit

//        GND    5V    VCi    CS    SI   SCK   SO   A   K
//         1                                            9
//	      +---------------------------------------------+
//+-------+---------+-----------+-----------+-----------+---------+
//| nCS0            |           |           |           | PB12 PD10    |
//+-----------------+-----------+-----------+-----------+---------+
//| nCS1            |           |           |           | PB5  PD15    |
//+-----------------+-----------+-----------+-----------+---------+
//| nCS2            |           |           |           | PA3  PE5     |
//+-----------------+-----------+-----------+-----------+---------+
//*H= Active High/L=Active Low
//*F= FallingEdge

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_GD32F130FX)
//use nCS0 of PD10
#define nCS0_CLCD_1      nCS0_H//{GPIO_SetBits(GPIOB, GPIO_Pin_12);}
#define nCS0_CLCD_0      nCS0_L//{GPIO_ResetBits(GPIOB, GPIO_Pin_12);}
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
//use nCS0 of PA15
#define nCS0_CLCD_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}
#define nCS0_CLCD_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
#endif

//use nCS1 of PB5//PD15
#define nCS1_CLCD_1      {GPIO_SetBits(GPIOB, GPIO_Pin_5);}
#define nCS1_CLCD_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_5);}

//use nCS2 of PA3//PD15
#define nCS2_CLCD_1      {GPIO_SetBits(GPIOA, GPIO_Pin_3);}
#define nCS2_CLCD_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_3);}

#define nCS_CLCD_1 nCS0_CLCD_1
#define nCS_CLCD_0 nCS0_CLCD_0

extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern void stmSPI3_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern void stmSpiWrByte(unsigned char inbyte);
extern unsigned char stmSpiRdByte();



struct clcdConfig{
  unsigned char _displayfunction;
  unsigned char _displaycontrol;
  unsigned char _displaymode;
  unsigned char _numlines;
  unsigned char _cols;
  unsigned char _rows;
  unsigned char _backlightval;
} g_clcdConfig;

// Starting Byte = (1111 1-R/S-R(1)/W(0)-0); R/S=1 == Data; R/S=0 == Instruction.
// And then if data is 0x12 (0001 0010) --> LSB First : 0100 1000 (D0..D7) ->To be sent as 0100 0000 + 1000 0000
void stmCLCD_command(unsigned char control)
{
	unsigned char lb, hb;
	lb =  (control << 7) & 0x80;
	lb |= (control << 5) & 0x40;
	lb |= (control << 3) & 0x20;
	lb |= (control << 1) & 0x10;

	hb =  (control << 3) & 0x80;
	hb |= (control << 1) & 0x40;
	hb |= (control >> 1) & 0x20;
	hb |= (control >> 3) & 0x10;

	nCS_CLCD_0;
	stmSpiWrByte(0xf8 | 0x01); 	//1111 1001 -- Starting Byte (Why it is different from the manual?)
	stmSpiWrByte(lb);			//LB (LSB FIRST)
	stmSpiWrByte(hb);			//HB
	nCS_CLCD_1;
	delayms(2);
}
void stmCLCD_SendCharData(char data){

	unsigned char lb, hb;
	//SWAP MSB FIRST INTO LSB FIRST
	//(1) Least Nibble -> Swap it, and append 0000
	lb = (data << 7) & 0x80;
	lb |= (data << 5) & 0x40;
	lb |= (data << 3) & 0x20;
	lb |= (data << 1) & 0x10;
	//(2)  Most Nibble -> Swap it, and append 0000
	hb = (data << 3) & 0x80;
	hb |= (data << 1) & 0x40;
	hb |= (data >> 1) & 0x20;
	hb |= (data >> 3) & 0x10;

	nCS_CLCD_0;
	stmSpiWrByte(0xf8 | 0x02); //1111 1010
	stmSpiWrByte(lb); //LSB First
	stmSpiWrByte(hb); //MSB Next
	nCS_CLCD_1;
}


// Turn the (optional) backlight off/on
void stmCLCD_setBacklight(unsigned char new_val){
	if(new_val){
		g_clcdConfig._backlightval=LCD_BACKLIGHT;
		//pcf9547expanderWrite(0);
	}else{
		g_clcdConfig._backlightval=LCD_NOBACKLIGHT;
		//pcf9547expanderWrite(0);
	}
}

void stmCLCD_clear(){
	stmCLCD_command(LCD_CLEARDISPLAY);// clear display and set cursor position to zero
	delayms(2);  // this command takes a long time!
}

void stmCLCD_home(){
	stmCLCD_command(LCD_RETURNHOME);  // set cursor position to zero
	delayms(2);  	// this command takes a long time!
}

void stmCLCD_setCursorPos(unsigned char row, unsigned char col){
	//int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	int row_offsets[] = { 0x00, 0x20, 0x40, 0x60 };
	//if ( row > g_clcdConfig._numlines ) {
	//	row = g_clcdConfig._numlines-1;    // we count rows starting w/0
	//}
	stmCLCD_command(LCD_SETDDRAMADDR | (row_offsets[row] +col));
	delayms(1);
}

// Turn the display on/off (quickly)
void stmCLCD_noDisplay() {
	g_clcdConfig._displaycontrol &= ~LCD_DISPLAYON;
	stmCLCD_command(LCD_DISPLAYCONTROL | g_clcdConfig._displaycontrol);
}
void stmCLCD_display() {
	g_clcdConfig._displaycontrol |= LCD_DISPLAYON;
	stmCLCD_command(LCD_DISPLAYCONTROL | g_clcdConfig._displaycontrol);
}

// Turns the underline cursor on/off
void stmCLCD_noCursor() {
	g_clcdConfig._displaycontrol &= ~LCD_CURSORON;
	stmCLCD_command(LCD_DISPLAYCONTROL | g_clcdConfig._displaycontrol);
	//delayms(1);
}
void stmCLCD_cursor() {
	g_clcdConfig._displaycontrol |= LCD_CURSORON;
	stmCLCD_command(LCD_DISPLAYCONTROL |g_clcdConfig. _displaycontrol);
}

// Turn on and off the blinking cursor
void stmCLCD_noBlink() {
	g_clcdConfig._displaycontrol &= ~LCD_BLINKON;
	stmCLCD_command(LCD_DISPLAYCONTROL | g_clcdConfig._displaycontrol);
}
void stmCLCD_blink() {
	g_clcdConfig._displaycontrol |= LCD_BLINKON;
	stmCLCD_command(LCD_DISPLAYCONTROL | g_clcdConfig._displaycontrol);
}

// These commands scroll the display without changing the RAM
void stmCLCD_scrollDisplayLeft(void) {
	stmCLCD_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void stmCLCD_scrollDisplayRight(void) {
	stmCLCD_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void stmCLCD_leftToRight(void) {
	g_clcdConfig._displaymode |= LCD_ENTRYLEFT;
	stmCLCD_command(LCD_ENTRYMODESET | g_clcdConfig._displaymode);
}

// This is for text that flows Right to Left
void stmCLCD_rightToLeft(void) {
	g_clcdConfig._displaymode &= ~LCD_ENTRYLEFT;
	stmCLCD_command(LCD_ENTRYMODESET | g_clcdConfig._displaymode);
}

// This will 'right justify' text from the cursor
void stmCLCD_autoscroll(void) {
	g_clcdConfig._displaymode |= LCD_ENTRYSHIFTINCREMENT;
	stmCLCD_command(LCD_ENTRYMODESET | g_clcdConfig._displaymode);
}

// This will 'left justify' text from the cursor
void stmCLCD_noAutoscroll(void) {
	g_clcdConfig._displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	stmCLCD_command(LCD_ENTRYMODESET | g_clcdConfig._displaymode);
}

void stmCLCD_WriteString(char *str){
	int i;
	for(i=0; i<16; i++){
		if(!str[i]) break;
		stmCLCD_SendCharData(str[i]);
		delayms(5);
	}
}

// When the display powers up, it is configured as follows:
// 1. Display clear
// 2. Function set:
//    DL = 0; 4-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we can't assume that its in that state when a sketch starts (and the
// Lcd constructor is called).

stmCLCD_Init(u8 SpiId, unsigned short spiMode, unsigned char data8or16,int nCS, unsigned char lcd_cols,unsigned char lcd_rows)
{
	unsigned char cols, lines, dotsize;

	if(SpiId == 3)
		stmSPI3_Config(1, nCS, spiMode, data8or16);
	else if(SpiId == 2)
		stmSPI2_Config(1, nCS, spiMode, data8or16);

	g_clcdConfig._cols = lcd_cols;
	g_clcdConfig._rows = lcd_rows;
	g_clcdConfig._backlightval = LCD_NOBACKLIGHT;
	g_clcdConfig._displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	lines = lcd_rows;

	dotsize = LCD_5x8DOTS;
	if (lines > 1)
		g_clcdConfig._displayfunction |= LCD_1LINE; //4LINE?

	g_clcdConfig._numlines = lines;

	// for some 1 line displays you can select a 10 pixel high font
	if ((dotsize != 0) && (lines == 1))
		g_clcdConfig._displayfunction |= LCD_5x8DOTS;//LCD_5x10DOTS; ???

	delayms(50);// Wait for 50ms before sending commands.

	// Now we pull both RS and R/W low to begin commands
	//stmCLCD_command(g_clcdConfig._backlightval); // reset expander and turn backlight off (Bit 8 =1)
	//delayms(1);

	stmCLCD_command(0x30);//Function Set (001-1(Data Length=8 bit); RE=0DH=0 (dot scroll enable); REV=0(Normal)
	delayms(1);
	//stmCLCD_command(0x0d);//Display OnOff Control(0000-1-D(1=Disp On)-C(1=cursor on)-B(1=cursor blink ON))
	stmCLCD_command(0x0c);//Display OnOff Control(0000-1-D(1=Disp On)-C(0=cursor off)-B(0=cursor blink off))
	delayms(1);
	stmCLCD_command(0x01);//Clear Display
	delayms(1);
	stmCLCD_command(0x06);//Entry mode Set (0000-01-10: Increment, DisplayShiftDisable ) under RE=0;
	delayms(1);
	//??
	stmCLCD_command(0x3c);//Function Set(0011-1100) -- RE=1
	delayms(1);
	stmCLCD_command(0x09);//Extended Function Set (0000-1-001) 5-bit, 4-line. (FW=0(5dot);B/W(0);NW=1:4-line)
	delayms(1);
	stmCLCD_command(0x38);//Function Set -- RE=0
	delayms(1);
	stmCLCD_command(0x0e);//Display OnOff (0000-1-D-C-B)
	delayms(1);

	stmCLCD_command(0x3c); //Function Set(0011-1100) --RE=1 (0011-1-1(RE=1)-00)
	delayms(1);
	stmCLCD_command(0x07);// Entry Mode Set with Segment Bidirection Function BID (Seg80->Seg1)
	delayms(1);
	stmCLCD_command(0x38); //Function Set(0011-1100) --RE=0 (0011-1-0(RE=0)-00)
	delayms(1);

	stmCLCD_command(0xa0); //Set DDRAM Address...
	delayms(1);
	printf("CLCD> CLCD-SPI CONFIG DONE\r\n");

/*	//===Config sequence for 4 bit mode: See HD44780 datasheet
	//a) we first start in 8bit mode 3 times, finally set 4 bit mode
	stmHd44780_Pcf8574_Wr4bits(0x03<<4);
	delayms(5); // wait min 4.1ms
	//b) second try
	stmHd44780_Pcf8574_Wr4bits(0x03<<4);
	delayms(5); // wait min 4.1ms
	//c) third again
	stmHd44780_Pcf8574_Wr4bits(0x03<<4);
	delayms(1);
	//d) finally, set to 4-bit interface
	stmHd44780_Pcf8574_Wr4bits(0x02<<4);
*/
/*
	//===set # lines, font size, etc.
	stmCLCD_command(LCD_FUNCTIONSET | g_clcdConfig._displayfunction);

	// turn ON with no cursor or blinking in default
	g_clcdConfig._displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	stmCLCD_display();

	stmCLCD_clear();

	// Set default text direction
	g_clcdConfig._displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

	// Set the entry mode
	stmCLCD_command(LCD_ENTRYMODESET | g_clcdConfig._displaymode);
*/

	stmCLCD_home(); //set cursor position to zero
}
/*
void stmCLCD_Init(u8 SpiId, unsigned short spiMode, unsigned char data8or16,int nCS)
{
	if(SpiId == 3)
		stmSPI3_Config(1, nCS, spiMode, data8or16);
	else if(SpiId == 2)
		stmSPI2_Config(1, nCS, spiMode, data8or16);

	stmCLCD_command(0x30);//Function Set (001-1(Data Length=8 bit)-
	delayms(1);
	stmCLCD_command(0x0d);//Display OnOff Control(0000-11-C(curson on)-B(cursor blink off))
	delayms(1);
	stmCLCD_command(0x01);//Clear Display
	delayms(1);
	stmCLCD_command(0x06);//Entry mode Set (0000-01-Inc/Dec-S)
	delayms(1);
	//??
	stmCLCD_command(0x3c);//Function Set(0011-1100) -- RE=1
	delayms(1);
	stmCLCD_command(0x09);//Extended Function Set (0000-1-0-0-1) 5-bit, 4-line.
	delayms(1);
	stmCLCD_command(0x38);//RE=0
	delayms(1);
	stmCLCD_command(0x0e);//Display OnOff (0000-1-D-C-B)
	delayms(1);

	stmCLCD_command(0x3c); //RE=1 (0011-1-1(RE=1)-00)
	delayms(1);
	stmCLCD_command(0x07);// Entry Mode Set with Segment Bidirection Function BID (Seg80->Seg1)
	delayms(1);
	stmCLCD_command(0x38); //RE=0 (0011-1-0(RE=0)-00)
	delayms(1);

	stmCLCD_command(0xa0); //Set DDRAM Address...
	delayms(1);
	printf("CLCD> CLCD-SPI CONFIG DONE\r\n");
}
*/
void stmCLCD_Samsung_Loop() {
	int m = 0;
	int h=0;
	int nCS=0;
	char str[16];
	int i = 0;

	char ch = '0';

	printf("CLCD-SPI Test\r\n");
	delayms(1000);
#if(PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_GD32F130FX)
	stmCLCD_Init(USE_SPI2, stmSPIMODE0, stmSPIDATA8, nCS, 16, 4);//col. row

#elif(PROCESSOR == PROCESSOR_STM32F107VCT)
	stmCLCD_Init(USE_SPI3, stmSPIMODE0, stmSPIDATA8,  nCS);
#endif
	//stmCLCD_leftToRight();
	//stmCLCD_autoscroll();
	//stmCLCD_noCursor();
	stmCLCD_clear();
	stmCLCD_home();//stmCLCD_setCursorPos(0,0);
	stmCLCD_setCursorPos(0,3);
	stmCLCD_WriteString("uc164904 CLCDT");

    while(1){
    	//stmCLCD_clear();
    	stmCLCD_setCursorPos(1,0);
    	if(i%2)
    		stmCLCD_WriteString("Hello  ");
    	else
    		stmCLCD_WriteString("AnNyong");
    	stmCLCD_setCursorPos(3,6);//stmCLCD_setCursorPos(8,1);
    	sprintf(str,"19:20:%02d",i);
    	stmCLCD_WriteString(str);
    	i++;
    	if(i==60) i=0;
    	delayms(1000);
    }

//TBD
    // (1) No Cursor
    // (2) Own LOGO design.
    // (3) Need row clear



	while(1){
		//stmCLCD_home();
		stmCLCD_SendCharData(ch);
		delayms(500);

		//stmCLCD_setCursorPos(1,1);

		if(ch=='z'){
			ch = '0';
		}else if(ch=='Z'){
			ch = 'a';
		}else if(ch=='9'){
			ch = 'A';
		}
		else
			ch++;
	}
}


