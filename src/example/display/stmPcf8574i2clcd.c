// PCF8574 for HD44780 CLCD over I2C.
// 8 lines of PCF8574 are used as
//  - 4 : for LCD data lines 4 to 7.(4-bit data operation)
//  - 4 : for Enable, register-select(RS), Read/Write and backlight control.
// Here we use HD44780 with 4-bit operation.

//2 versions of PCF8574(0x27) and PCF8574A(0x4E).
//The only difference between the two is the I2C base address.
//(A0, A1 and A3 un-linked, so pulled high).
/*
Jp3	Jp2	Jp1
A2	A1	A0	Dec	Hex
L	L	L	32	0x20
L	L	H	33	0x21
L	H	L	34	0x22
L	H	H	35	0x23
H	L	L	36	0x24
H	L	H	37	0x25
H	H	L	38	0x26
H	H	H	39	0x27

For PCF8574A the addressing is:

Jp3	Jp2	Jp1
A2	A1	A0	Dec	Hex
L	L	L	56	0x38
L	L	H	57	0x39
L	H	L	64	0x40
L	H	H	74	0x4A
H	L	L	75	0x4B
H	L	H	76	0x4C
H	H	L	77	0x4D
H	H	H	78	0x4E
*/

/* LCD pinout= 16:K 15:A 14:D7 13:D6 12:D5 11:D4  10:D3(NC) 9:D2(NC)  8:D1(NC)  7:D0(NC)  6:E  5:RW  4:RS  3: VO   2:VDD  1:GND
 * PCF8574 pinout = 1:A0 2:A1 3:A2 == All open = 111 (7 Bit Address = 0x27)
 *                  4: P0 -- RS
 *                  5: P1 -- RW
 *                  6: P2 -- E
 * 					7: P3 -- Backlight control (Base )
 * 					8: GND
 * 					9: P4 -- D4
 * 					10:P5 -- D5
 * 					11:P6 -- D6
 * 					12:P7 -- D7
 * 					13:nOUT
 * 					14:SCL
 * 					15:SDA
 * 					16:Vcc(3V3)
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
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"
extern void delayms(uint32_t ms);
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;

#define PCF8574ADDR8 	0x40 //8-bit addr// 0x20/21 == 7bit Addr
#define PCF8574ADDR 0x27 //No Connection

#define I2C_400KHZ	1 // 0 to use default 100Khz, 1 for 400Khz

#define PCF8574_OK          0x00
#define PCF8574_PIN_ERROR   0x81
#define PCF8574_I2C_ERROR   0x82

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

struct pcf8574config{
  unsigned char _Addr;
  unsigned char _displayfunction;
  unsigned char _displaycontrol;
  unsigned char _displaymode;
  unsigned char _numlines;
  unsigned char _cols;
  unsigned char _rows;
  unsigned char _backlightval;
} g_pcf8574config;

u8 pcf8574sendbuf[80];

void stmHd44780_Pcf8574Wr(unsigned char _data){
	pcf8574sendbuf[0]=_data |  g_pcf8574config._backlightval;
	stm_I2C_SendBurst(PCF8574ADDR8, pcf8574sendbuf, 1);
}
void stmHd44780_Pcf8574_Enable(unsigned char _data){
	stmHd44780_Pcf8574Wr(_data | PCF9547_ENBIT); // En high
	delayms(1); // enable pulse must be >450ns

	stmHd44780_Pcf8574Wr(_data & ~PCF9547_ENBIT); // En low
	delayms(1); // commands need > 37us to settle
}

void stmHd44780_Pcf8574_Wr4bits(unsigned char value) {
	stmHd44780_Pcf8574Wr(value);
	stmHd44780_Pcf8574_Enable(value);
}
// write either command or data
void stmHd44780_Pcf8574_send(unsigned char value, unsigned char mode) {
	unsigned char highnib=value & 0xf0;
	unsigned char lownib=(value <<4) & 0xF0;
	stmHd44780_Pcf8574_Wr4bits((highnib)|mode);
	stmHd44780_Pcf8574_Wr4bits((lownib)|mode);
}

inline void stmHd44780_Pcf8574_command(unsigned char value) {
	stmHd44780_Pcf8574_send(value, 0);
}

inline size_t stmHd44780_Pcf8574_write(unsigned char value) {
	stmHd44780_Pcf8574_send(value, PCF9547_RSBIT);
 return 0;
}
#if 0
// Allows us to fill the first 8 CGRAM locations
// with custom characters
// Custom Character
// We can generate max 8 characters in private. It will be stored in CGRAM.
unsigned char SparkChar[8]  = { 0b00000,0b10101,0b01010,0b00100,0b11111,0b00100,0b01010,0b10101 };
void stmHd44780_Pcf8574_load_custom_character(unsigned char char_num, unsigned char *rows){
 	 createChar(char_num, rows);
}

void stmHd44780_Pcf8574_loadCustom8Chars(unsigned char location) {
	int i;
	location &= 0x7; // we only have 8 locations 0-7
	stmHd44780_Pcf8574_command(LCD_SETCGRAMADDR | (location << 3));
	for (i=0; i<8; i++) {
		stmHd44780_Pcf8574_write(SparkChar[i]);
	}
}
#endif

// Turn the (optional) backlight off/on
void stmHd44780_Pcf8574_setBacklight(unsigned char new_val){
	if(new_val){
		g_pcf8574config._backlightval=LCD_BACKLIGHT;
		pcf9547expanderWrite(0);
	}else{
		g_pcf8574config._backlightval=LCD_NOBACKLIGHT;
		pcf9547expanderWrite(0);
	}
}

void stmHd44780_Pcf8574_clear(){
	stmHd44780_Pcf8574_command(LCD_CLEARDISPLAY);// clear display and set cursor position to zero
	delayms(2);  // this command takes a long time!
}

void stmHd44780_Pcf8574_home(){
	stmHd44780_Pcf8574_command(LCD_RETURNHOME);  // set cursor position to zero
	delayms(2);  	// this command takes a long time!
}

void stmHd44780_Pcf8574_setCursor(unsigned char col, unsigned char row){
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row > g_pcf8574config._numlines ) {
		row = g_pcf8574config._numlines-1;    // we count rows starting w/0
	}
	stmHd44780_Pcf8574_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void stmHd44780_Pcf8574_noDisplay() {
	g_pcf8574config._displaycontrol &= ~LCD_DISPLAYON;
	stmHd44780_Pcf8574_command(LCD_DISPLAYCONTROL | g_pcf8574config._displaycontrol);
}
void stmHd44780_Pcf8574_display() {
	g_pcf8574config._displaycontrol |= LCD_DISPLAYON;
	stmHd44780_Pcf8574_command(LCD_DISPLAYCONTROL | g_pcf8574config._displaycontrol);
}

// Turns the underline cursor on/off
void stmHd44780_Pcf8574_noCursor() {
 g_pcf8574config._displaycontrol &= ~LCD_CURSORON;
 stmHd44780_Pcf8574_command(LCD_DISPLAYCONTROL | g_pcf8574config._displaycontrol);
}
void stmHd44780_Pcf8574_cursor() {
	g_pcf8574config._displaycontrol |= LCD_CURSORON;
	stmHd44780_Pcf8574_command(LCD_DISPLAYCONTROL |g_pcf8574config. _displaycontrol);
}

// Turn on and off the blinking cursor
void stmHd44780_Pcf8574_noBlink() {
	g_pcf8574config._displaycontrol &= ~LCD_BLINKON;
	stmHd44780_Pcf8574_command(LCD_DISPLAYCONTROL | g_pcf8574config._displaycontrol);
}
void stmHd44780_Pcf8574_blink() {
	g_pcf8574config._displaycontrol |= LCD_BLINKON;
	stmHd44780_Pcf8574_command(LCD_DISPLAYCONTROL | g_pcf8574config._displaycontrol);
}

// These commands scroll the display without changing the RAM
void stmHd44780_Pcf8574_scrollDisplayLeft(void) {
	stmHd44780_Pcf8574_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void stmHd44780_Pcf8574_scrollDisplayRight(void) {
	stmHd44780_Pcf8574_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void stmHd44780_Pcf8574_leftToRight(void) {
	g_pcf8574config._displaymode |= LCD_ENTRYLEFT;
	stmHd44780_Pcf8574_command(LCD_ENTRYMODESET | g_pcf8574config._displaymode);
}

// This is for text that flows Right to Left
void stmHd44780_Pcf8574_rightToLeft(void) {
	g_pcf8574config._displaymode &= ~LCD_ENTRYLEFT;
	stmHd44780_Pcf8574_command(LCD_ENTRYMODESET | g_pcf8574config._displaymode);
}

// This will 'right justify' text from the cursor
void stmHd44780_Pcf8574_autoscroll(void) {
	g_pcf8574config._displaymode |= LCD_ENTRYSHIFTINCREMENT;
	stmHd44780_Pcf8574_command(LCD_ENTRYMODESET | g_pcf8574config._displaymode);
}

// This will 'left justify' text from the cursor
void stmHd44780_Pcf8574_noAutoscroll(void) {
	g_pcf8574config._displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	stmHd44780_Pcf8574_command(LCD_ENTRYMODESET | g_pcf8574config._displaymode);
}

void stmHd44780_Pcf8574_WriteString(char *str){
	int i;
	for(i=0; i<16; i++){
		if(!str[i]) break;
		stmHd44780_Pcf8574_write(str[i]);
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

stmHd44780_Pcf8574_init(unsigned char lcd_Addr,unsigned char lcd_cols,unsigned char lcd_rows){
	unsigned char cols, lines, dotsize;

	g_pcf8574config._Addr = lcd_Addr; //I2C addr
	g_pcf8574config._cols = lcd_cols;
	g_pcf8574config._rows = lcd_rows;
	g_pcf8574config._backlightval = LCD_NOBACKLIGHT;
	g_pcf8574config._displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	lines = lcd_rows;

	dotsize = LCD_5x8DOTS;
	if (lines > 1)
		g_pcf8574config._displayfunction |= LCD_2LINE;

	g_pcf8574config._numlines = lines;

	// for some 1 line displays you can select a 10 pixel high font
	if ((dotsize != 0) && (lines == 1))
		g_pcf8574config._displayfunction |= LCD_5x10DOTS;

	delayms(50);// Wait for 50ms before sending commands.

	// Now we pull both RS and R/W low to begin commands
	stmHd44780_Pcf8574Wr(g_pcf8574config._backlightval); // reset expander and turn backlight off (Bit 8 =1)
	delayms(1);

	//===Config sequence for 4 bit mode: See HD44780 datasheet
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

	//===set # lines, font size, etc.
	stmHd44780_Pcf8574_command(LCD_FUNCTIONSET | g_pcf8574config._displayfunction);

	// turn ON with no cursor or blinking in default
	g_pcf8574config._displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	stmHd44780_Pcf8574_display();

	stmHd44780_Pcf8574_clear();

	// Set default text direction
	g_pcf8574config._displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

	// Set the entry mode
	stmHd44780_Pcf8574_command(LCD_ENTRYMODESET | g_pcf8574config._displaymode);

	stmHd44780_Pcf8574_home(); //set cursor position to zero
}

void stmHd44780_Pcf8574_Clcd_Loop(){
	char str[16];
	int i;

	printf("PCF8574 IO Expander with I2C (8bit I2C addr = 0x%02x).\r\n",PCF8574ADDR8);

	if(!g_bI2CModuleConfigDone)
	{
#if (PROCESSOR == STM32F407VZT6)
		  gI2Cx = I2C2;
#else
		  gI2Cx = I2C1;
#endif
		printf("I2C Init...");
		stm_I2C_Init(gI2Cx,100000);//100Kbps Only
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	}

    stmHd44780_Pcf8574_init(0x27, 16, 2); //i2c addr, col. row
    //stmHd44780_Pcf8574_loadCustom8Chars(0);

    i=0;
    while(1){
    	stmHd44780_Pcf8574_setCursor(1,0);
    	stmHd44780_Pcf8574_WriteString("Hello I2C LCD");
    	stmHd44780_Pcf8574_setCursor(8,1);
    	sprintf(str,"%d",i);
    	stmHd44780_Pcf8574_WriteString(str);
    	i++;
    	if(i==100) i=0;
    	delayms(300);
    }

    //clear();
    //setCursor(4,0);
    //write((unsigned char)7);
    //setCursor(6,0 );
    //print("Core");
    //setCursor(12,0);
    //write((unsigned char)7);
    //setCursor(2,1);
    //print("GOOD LUCK");
}
