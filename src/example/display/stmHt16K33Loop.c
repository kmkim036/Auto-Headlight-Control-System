/*I2C LED Matrix
 * These displays use I2C to communicate, 2 pins are required to interface.
 * There are multiple selectable I2C addresses.
 * 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73.
 * For backpacks  with 3 Address Select pins: 0x70 thru 0x77
 */
 /***************************************************
  This is a library for our I2C LED Backpacks
  These displays use I2C to communicate, 2 pins are required to
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73.
 ****************************************************/
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
#include "lwipopts.h"
*/
#include "yInc.h"
#include <stdio.h>
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


extern void delayms(uint32_t ms);
extern int stm_I2C_SendBurst(unsigned char slv_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;

//#define HT16K33ADDR 0xE0 //(0x70 <<1)  //0x70 = 7bit address
#define HT16K33ADDR 0xEE //(0x77 <<1)  //0x70 = 7bit address
//#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz

#define LED_ON 1
#define LED_OFF 0
#define LED_RED 1
#define LED_YELLOW 2
#define LED_GREEN 3

#define HT16K33_DISPSETUP_CMD 0x80 //Display Setup Register
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3
#define HT16K33_CMD_BRIGHTNESS 0x0E

#define SEVENSEG_DIGITS 5

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0

static const unsigned char stmHt16K33numbertable[] = {
        0x3F, /* 0 */
        0x06, /* 1 */
        0x5B, /* 2 */
        0x4F, /* 3 */
        0x66, /* 4 */
        0x6D, /* 5 */
        0x7D, /* 6 */
        0x07, /* 7 */
        0x7F, /* 8 */
        0x6F, /* 9 */
        0x77, /* a */
        0x7C, /* b */
        0x39, /* C */
        0x5E, /* d */
        0x79, /* E */
        0x71, /* F */
};

unsigned short stmHt16K33displaybuffer[8];
unsigned char ht16Ksendbuf[32];

void stmHt16K33setBrightness(uint8_t b) { //Dimming Data
	if (b > 15) b = 15;
	ht16Ksendbuf[0]= 0xE0 | b;
  	stm_I2C_SendBurst(HT16K33ADDR, &ht16Ksendbuf[0], 1);
}

void stmHt16K33blinkRate(uint8_t b) {

	if (b > 3) b = 0; // turn off if not sure
	ht16Ksendbuf[0]= (HT16K33_DISPSETUP_CMD | HT16K33_BLINK_DISPLAYON | (b << 1));
  	stm_I2C_SendBurst(HT16K33ADDR, ht16Ksendbuf, 1);
}

void stmHt16K33init()
{
	unsigned char z;
	printf("ht16k33 Init..\r\n");
	delayms(30);
	//I2CScan();

	ht16Ksendbuf[0]= 0x21; //Set On for System Setup Register.

	stm_I2C_SendBurst(HT16K33ADDR, &ht16Ksendbuf[0], 1); // turn on oscillator

	delayms(20);

/*	ht16Ksendbuf[0]= 0x60; //Set On for INT flag Adress Pointer
  	stm_I2C_SendBurst(HT16K33ADDR, ht16Ksendbuf, 1);
    delayms(1);
	ht16Ksendbuf[0]= 0xA0; //Set On for ROW/INT
  	stm_I2C_SendBurst(HT16K33ADDR, ht16Ksendbuf, 1);
    delayms(100);
*/
	stmHt16K33blinkRate(HT16K33_BLINK_OFF);
  	stmHt16K33setBrightness(5); // (0..15). 15=max brightness
    delayms(10);
}
void stmHt16K33writeDisplay(void) {
	u8 i=0;
	u8 j;
	ht16Ksendbuf[i]= 0x00; // start at address $00

	i++;
	for (j=0; j<8; j++) {
	  	  ht16Ksendbuf[i++]= (stmHt16K33displaybuffer[j] & 0xFF);
	  	  ht16Ksendbuf[i++]= (stmHt16K33displaybuffer[j] >> 8);
	}
 	stm_I2C_SendBurst(HT16K33ADDR, ht16Ksendbuf, 17);

}

void stmHt16K33clear(void) {
	u8 i;
  for (i=0; i<8; i++) {
	  stmHt16K33displaybuffer[i] = 0;
  }
}
/*
void stmHt16K33drawPixel(short x, short y, u16 color) {
  if ((y < 0) || (y >= 8)) return;
  if ((x < 0) || (x >= 8)) return;

 // check rotation, move pixel around if necessary
  switch (getRotation()) {
  case 1:
    swap(x, y);
    x = 8 - x - 1;
    break;
  case 2:
    x = 8 - x - 1;
    y = 8 - y - 1;
    break;
  case 3:
    swap(x, y);
    y = 8 - y - 1;
    break;
  }

  // wrap around the x
  x += 7;
  x %= 8;


  if (color) {
	  stmHt16K33displaybuffer[y] |= 1 << x;
  } else {
	  stmHt16K33displaybuffer[y] &= ~(1 << x);
  }
}


void stmHt16K33BicolorMatrixdrawPixel(short x, short y, u16 color) {
  if ((y < 0) || (y >= 8)) return;
  if ((x < 0) || (x >= 8)) return;

  switch (getRotation()) {
  case 1:
    swap(x, y);
    x = 8 - x - 1;
    break;
  case 2:
    x = 8 - x - 1;
    y = 8 - y - 1;
    break;
  case 3:
    swap(x, y);
    y = 8 - y - 1;
    break;
  }

  if (color == LED_GREEN) {
	  stmHt16K33displaybuffer[y] |= 1 << x;
  } else if (color == LED_RED) {
	  stmHt16K33displaybuffer[y] |= 1 << (x+8);
  } else if (color == LED_YELLOW) {
	  stmHt16K33displaybuffer[y] |= (1 << (x+8)) | (1 << x);
  } else if (color == LED_OFF) {
	  stmHt16K33displaybuffer[y] &= ~(1 << x) & ~(1 << (x+8));
  }
}
*/
//TBD
void stmHt16K33Loop(){

	uint8_t counter = 0;
	u8 i;
	//I2C Config.
	if(!g_bI2CModuleConfigDone){
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
		  gI2Cx = I2C2;
#else
		  gI2Cx = I2C1;
#endif
		stm_I2C_Init(gI2Cx,400000);//400Kbps
	}
    printf("HT16K33 LED Driver Test with I2C\r\n");

    stmHt16K33init(); //ON

    while(1){
    	// paint one LED per row. The HT16K33 internal memory looks like a 8x16 bit matrix (8 rows, 16 columns)
    	// paint one LED per column. The HT16K33 internal memory looks like a 16x8 bit matrix (16 rows, 8 columns)
    	for (i=0; i<8; i++) {    // draw a diagonal row of pixels
    		stmHt16K33displaybuffer[i] = _BV((counter+i) % 16) | _BV((counter+i+8) % 16)  ;
    	}  // write the changes we just made to the display
    	stmHt16K33writeDisplay();
    	delayms(100);
    	counter++;
    	if (counter >= 16) counter = 0;
    }

 }
