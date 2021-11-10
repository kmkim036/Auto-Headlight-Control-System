/***************************************************
  This is a library for the 8-bit parallel display with ILI9163C controller.
  Use 8-bit parallel to communicate.
  [For STM32F103RCT6 for MTF0177SN-13 128x160 LCD with Parallel 8 bits]
  *RST is optional
  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
  - 1.8" ICB (128x160)

  - Just for STM32F103RCT6
  - For image, use Image2LCD program.
 ****************************************************/
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
#include "yLcdInc.h"

#if (LCDCONT_IS == LCDCONT_IS_IL9163_PARA)
/*[For STM32F103RCT6 for MTF0177SN-13 128x160 LCD with Parallel 8 bits]
 *[Graphic Controller = IL9163C]
 * LCD_CS : PC6 : FSMC_NE1
 * LCD_RS : PC7
 * LCD_WR : PC1
 * LCD_RD : PC2
 * LCD_D[7:0] : PB15:8
*/

//=============== PARALELL 8 BIT WIDE  ===========================
/* LCD_CS : PC6 : (Active Low)
 * LCD_RS : PC7 (1=DAT ; 0=CMD)
 * LCD_WR : PC1 (Active Low Strobe)
 * LCD_RD : PC2 (Active Low Strobe)
 * LCD_D[7:0] : PD15:8
 */
#define IL9163_CS_HIGH {GPIO_SetBits(GPIOC, GPIO_Pin_6);}
#define IL9163_CS_LOW  {GPIO_ResetBits(GPIOC, GPIO_Pin_6);}

#define IL9163_RS_DAT_HIGH {GPIO_SetBits(GPIOC, GPIO_Pin_7);}
#define IL9163_RS_CMD_LOW  {GPIO_ResetBits(GPIOC, GPIO_Pin_7);}

#define IL9163_WR_HIGH {GPIO_SetBits(GPIOC, GPIO_Pin_1);}
#define IL9163_WR_LOW  {GPIO_ResetBits(GPIOC, GPIO_Pin_1);}
#define IL9163_WR_STROBE {IL9163_WR_LOW ; IL9163_WR_HIGH;}

#define IL9163_RD_HIGH {GPIO_SetBits(GPIOC, GPIO_Pin_2);}
#define IL9163_RD_LOW  {GPIO_ResetBits(GPIOC, GPIO_Pin_2);}

//#define IL9163_RESET_HIGH {GPIO_SetBits(GPIOC, GPIO_Pin_0);}
//#define IL9163_RESET_LOW  {GPIO_ResetBits(GPIOC, GPIO_Pin_0);}

//================ IL9163 ============================
//LCD Dimension 128*160 Pixels 1.8"
#define IL9163_TFTWIDTH  128
#define IL9163_TFTHEIGHT 160

//Commands
#define IL9163_NOP     0x00
#define IL9163_SWRESET 0x01 //Soft Reset

#define IL9163_SLPIN   0x10 //Sleep IN
#define IL9163_SLPOUT  0x11 //Sleep OFF
#define IL9163_PTLON   0x12 //Partial Mode ON
#define IL9163_NORON   0x13

#define IL9163_INVOFF  0x20
#define IL9163_INVON   0x21 //Display Inversion ON

#define IL9163_GAMMASET 0x26
#define IL9163_DISPOFF 0x28
#define IL9163_DISPON  0x29
#define IL9163_CASET   0x2A //Column Address Set
#define IL9163_RASET   0x2B //Page Address Set
#define IL9163_RAMWR   0x2C //Memory Write
#define IL9163_RAMRD   0x2E //Memory Read
#define IL9163_COLORSPACE   0x2D //Color Space 4K/65K/256K

#define IL9163_PTLAR   0x30 //Partial Area
#define IL9163_VSCLLDEF   0x33 //Vertical Scroll Definition
#define IL9163_TEFXLOFF   0x34 //Tearig Effect Line OFF
#define IL9163_TEFXLON   0x35 //Tearig Effect Line ON
#define IL9163_MADCTL  0x36 //Memory Access Control
#define IL9163_VSSTADRS  0x37 //Vertical Scrolling Start Address
#define IL9163_COLMOD  0x3A  //Interface Pixel Format

#define IL9163_FRMCTR1 0xB1 //Frame Rate Control in Normal/Full colors
#define IL9163_FRMCTR2 0xB2 //Frame Rate Control in Idle/Full colors
#define IL9163_FRMCTR3 0xB3 //Frame Rate Control in Partial/Full colors
#define IL9163_INVCTR  0xB4 //Display Inversion Control
#define IL9163_RGBBLK  0xB5 //RGB Interface Blanking Porch Setting
#define IL9163_DFUNCTR 0xB6 //Display Function Set 5
#define IL9163_SDRVDIR 0xB7 //Source Driver Direction Control
#define IL9163_GDRVDIR 0xB8 //Gate Driver Direction Control

#define IL9163_PWCTR1  0xC0 //Power Control1
#define IL9163_PWCTR2  0xC1
#define IL9163_PWCTR3  0xC2
#define IL9163_PWCTR4  0xC3
#define IL9163_PWCTR5  0xC4
#define IL9163_VCOMCTR1  0xC5 //VCOM Control 1
#define IL9163_VCOMCTR2  0xC6 //VCOM Control 2
#define IL9163_VCOMOFFS  0xC7 //VCOM Offset Control

//#define IL9163_RDID1   0xDA
//#define IL9163_RDID2   0xDB
//#define IL9163_RDID3   0xDC
//#define IL9163_RDID4   0xDD

//#define IL9163_PWCTR6  0xFC

#define IL9163_GMCTRP1 0xE0 //Positive Gamma Correction Setting
#define IL9163_GMCTRN1 0xE1 //Negative Gamma Correction Setting

#define IL9163_GMRSEL  0xF2 //Gamma_R_Sel

// Color definitions (16bit RGB)===============
#define	IL9163_BLACK   0x0000
#define	IL9163_BLUE    0x001F
#define	IL9163_RED     0xF800
#define	IL9163_GREEN   0x07E0
#define IL9163_CYAN    0x07FF
#define IL9163_MAGENTA 0xF81F
#define IL9163_YELLOW  0xFFE0
#define IL9163_WHITE   0xFFFF

//===============

extern struct _LCD_gfx LCD_gfx;
extern struct _LCD_clock LCD_clk;
extern const unsigned char face8[];

//     255 = 500 ms delay

//Init config commands lists for ILI9163
//===========   Init Sequence for ILI9163 =========
static unsigned char
  ILI9163_InitCmds[] = {    // Initialization commands for ILI9163 screens
    17,                     // 17 commands in list:

    IL9163_SWRESET, DELAY,  	//  1: Software reset, no args, w/delay
      100,                  	//     100 ms delay

    IL9163_SLPOUT , 0,   		//  2: Exit sleep mode, no args, w/delay

    IL9163_COLMOD , 1+DELAY,  	//  3: Set color mode, 1 arg + delay:
      0x05,                   	//     16-bit color
      10,                     	//     10 ms delay

    IL9163_GAMMASET , 1,  		//  4: Set Gamma Curve, 1 arg:
        0x04,

    IL9163_GMRSEL, 1,  			//  5: Gamma Adj Enable, 1 arg :
          0x01,                 //     Enable

    IL9163_GMCTRP1, 15, 		//6 Set Positive Gamma Correction Setting(0xE0)
          0x3F, 0x25, 0x1C,0x1E, 0x20,0x12,0x2A,0x90,
          0x24,0x11,0x00,0x00,0x00,0x00,0x00,

    IL9163_GMCTRN1, 15, 		//7 Set Negative Gamma Correction Setting(0xE1)
          0x20,0x20,0x20,0x20,0x05,0x00,0x15,0xA7,
          0x3D,0x18,0x25,0x2A,0x2B,0x2B,0x3A,

    IL9163_FRMCTR1, 2,  		//  8: Frame rate control, 2 args : (normal / full color)
      0x08,//06,                //     8 lines front porch
      0x08,//03,                //     8 lines back porch

    IL9163_INVCTR , 1 ,  		//  9: Display inversion control, 1 arg:
      0x07,//0,                 //     Line inversion

    IL9163_PWCTR1 , 2,  		//  10: Power control1, 2 args + delay:
      0x0A,//02,                //		4.3V     GVDD = 4.7V
      0x02,//70,                //     	1.0uA

    IL9163_PWCTR2 , 1,  		//  11: Power control, 1 arg, no delay:
      0x02,//0x05,              //     VGH = 14.7V, VGL = -7.35V

    IL9163_VCOMCTR1 , 2,  		// 12: Power control, 2 args:
      0x50,//3C,                //     VCOMH = 4V
      0x5B,//0x38,              //     VCOML = -1.1V

    IL9163_VCOMOFFS , 1,  		//  13: VCOM Offset
        0x40,                   //

    //128x160
    IL9163_CASET  , 4+DELAY ,  	// 14: Column addr set, 4 args, delay:
      0x00, 0x00,             	//     XSTART = 0
      0x00, 0x7F,             	//     XEND = 127
      250,						// 250msec

    //set page?
    IL9163_RASET  , 4,  		// 15: Row addr set, 4 args, no delay:
      0x00, 0x00,             	//     YSTART = 0
      0x00, 0x9F,             	//     YEND = 160

    IL9163_MADCTL , 1,  		// 16: Memory access ctrl (directions), 1 arg:(0x36)
        0xC8,                   //     Row addr/col addr, bottom to top refresh


    //IL9163_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
    //  10,                     //     10 ms delay
    IL9163_DISPON ,  0,  		// 17: Main screen turn on, no args, w/delay

};


//============= command write with RS=0 over Port D (8bit wide) =========
//LCD_D[7:0] : PB15:8
void IL9163_WrByte8(unsigned b){
	 b = b << 8;
     GPIOB->BSRR = ((~b) << 16) | (b); //or GPIOB->ODR = b;
     //printf("b=%02x = 0x%08X\r\n",b, ((~b) << 16) | (b));
     IL9163_WR_STROBE; //L->H
}

//============= command write with RS=0 over Port D (8bit wide) =========
void IL9163_writeCommand(unsigned char c) {

	IL9163_RS_CMD_LOW; 	//RS=0; --CMD
	IL9163_CS_LOW;  //CS=0
	IL9163_WrByte8(c);
	IL9163_CS_HIGH; //CS=1
}

//============= data write with RS=1 over over Port D (8bit wide) =========
void IL9163_writeData(unsigned char c) {
	IL9163_RS_DAT_HIGH; 	//RS=1; -- DATA
	IL9163_CS_LOW; 	//CS=0
	IL9163_WrByte8(c);
	IL9163_CS_HIGH; 	//CS
}

// Init Sequence
void IL9163_InitSeq(unsigned char *addr) {

  unsigned char  numCommands, numArgs, savedNumCmds;
  long ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow

  savedNumCmds = numCommands;

  while(numCommands--) {                 // For each command...
	//Cmd
	IL9163_writeCommand(pgm_read_byte(addr++)); //   Read command, and issue the command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    //Data
    while(numArgs--) {                   //   For each argument...
    	IL9163_writeData(pgm_read_byte(addr++));  //Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delayms(ms);
    }
  }
}

/* LCD_CS : PC6 : FSMC_NE1
 * LCD_RS : PC7
 * LCD_WR : PC1
 * LCD_RD : PC2
 * LCD_D[7:0] : PD15:8
 */
void IL9163_gpioSetup()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	LCD_gfx._width  = IL9163_TFTWIDTH;
	LCD_gfx._height = IL9163_TFTHEIGHT;
	LCD_gfx.colstart  = LCD_gfx.rowstart = 0; // May be overridden in init func

  	//CS: PC6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_SetBits(GPIOC, GPIO_Pin_6);

	//RS: PC7
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_SetBits(GPIOC, GPIO_Pin_7);

	//WR : PC1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_SetBits(GPIOC, GPIO_Pin_1);

	//RD : PC2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_SetBits(GPIOC, GPIO_Pin_2);

	//Data : LCD_D[7:0] : PB15:8
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 |GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9 | GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);


	IL9163_CS_HIGH; //IDLE
	IL9163_WR_HIGH; //IDLE
	IL9163_RD_HIGH; //IDLE


  // RESET PIN will be needed?
  // toggle RST low to reset;
  	  //IL9163_RESET_HIGH;//RST=H
  	  //somedelay(500<<11);
  	  //IL9163_RESET_LOW;//RST=L
  	  //somedelay(500<<11);
  	  //IL9163_RESET_HIGH;	//RST=H
  	  //somedelay(500<<11);
}

//================= graphic functions ==============================
inline unsigned short IL9163_swapcolor(unsigned short x) {
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}

void IL9163_setAddrWindow(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1) {

	IL9163_writeCommand(IL9163_CASET); // Column addr set
	IL9163_writeData(0x00);
	IL9163_writeData(x0+LCD_gfx.colstart);     // XSTART
	IL9163_writeData(0x00);
	IL9163_writeData(x1+LCD_gfx.colstart);     // XEND

	IL9163_writeCommand(IL9163_RASET); // Row addr set
	IL9163_writeData(0x00);
	IL9163_writeData(y0+LCD_gfx.rowstart);     // YSTART
	IL9163_writeData(0x00);
	IL9163_writeData(y1+LCD_gfx.rowstart);     // YEND

	IL9163_writeCommand(IL9163_RAMWR); // write to RAM
}

void IL9163_pushColor(unsigned short color) {

	IL9163_RS_DAT_HIGH; 	//RS=1;
	IL9163_CS_LOW; 	//~CS

	color = IL9163_swapcolor(color);
	IL9163_WrByte8(color >> 8);	IL9163_WrByte8(color);

	IL9163_CS_HIGH; //CS
}

void IL9163_drawPixel(short x, short y, unsigned short color) {

  if((x < 0) ||(x >= LCD_gfx._width) || (y < 0) || (y >= LCD_gfx._height)){
	  printf("draw Error\r\n");
	  return;
  }

  IL9163_setAddrWindow(x,y,x+1,y+1);

  IL9163_RS_DAT_HIGH; 	//RS=1; //==data
  IL9163_CS_LOW; 	//~CS

  color = IL9163_swapcolor(color);
  //send 16 bit RGB data (MSB first)
  IL9163_WrByte8(color >> 8);  IL9163_WrByte8(color);

  IL9163_CS_HIGH;	//CS
}

void IL9163_drawFastVLine(short x, short y, short h, unsigned short color) {

  // Rudimentary clipping
  if((x >= LCD_gfx._width) || (y >= LCD_gfx._height)) return;
  if((y+h-1) >= LCD_gfx._height) h = LCD_gfx._height-y;
  IL9163_setAddrWindow(x, y, x, y+h-1);

  //if (LCD_gfx.tabcolor == INITR_BLACKTAB)
     color = IL9163_swapcolor(color);

  unsigned char hi = color >> 8, lo = color;
  IL9163_RS_DAT_HIGH;	//RS=1;
  IL9163_CS_LOW;	//~CS

  while (h--) {
	  IL9163_WrByte8(hi);
	  IL9163_WrByte8(lo);
  }
  IL9163_CS_HIGH;	//CS
}

void IL9163_drawFastHLine(short x, short y, short w, unsigned short color) {

  // Rudimentary clipping
  if((x >= LCD_gfx._width) || (y >= LCD_gfx._height)) return;
  if((x+w-1) >= LCD_gfx._width)  w = LCD_gfx._width-x;
  IL9163_setAddrWindow(x, y, x+w-1, y);

  //if (LCD_gfx.tabcolor == INITR_BLACKTAB)
	  color = IL9163_swapcolor(color);

  unsigned char hi = color >> 8, lo = color;
  IL9163_RS_DAT_HIGH;	//RS=1;
  IL9163_CS_LOW;	//~CS

  while (w--) {
	  IL9163_WrByte8(hi);
	  IL9163_WrByte8(lo);
  }
  IL9163_CS_HIGH;	//CS=1
}




// fill a rectangle
void IL9163_fillRect(short x, short y, short w, short h,  unsigned short color)
{
  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= LCD_gfx._width) || (y >= LCD_gfx._height)) return;
  if((x + w - 1) >= LCD_gfx._width)  w = LCD_gfx._width  - x;
  if((y + h - 1) >= LCD_gfx._height) h = LCD_gfx._height - y;

 // if (LCD_gfx.tabcolor == INITR_BLACKTAB)
	  color = IL9163_swapcolor(color);

  IL9163_setAddrWindow(x, y, x+w-1, y+h-1);

  unsigned char hi = color >> 8, lo = color;
  IL9163_RS_DAT_HIGH;	//RS=1;
  IL9163_CS_LOW;	//~CS

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
    	IL9163_WrByte8(hi);
    	IL9163_WrByte8(lo);
    }
  }
  IL9163_CS_HIGH;	//CS
}

void IL9163_fillScreen(unsigned short color) {
	IL9163_fillRect(0, 0,  LCD_gfx._width, LCD_gfx._height, color);
}

/*
// fill a rectangle
void IL9163_fillRect(unsigned char x0, unsigned char y0, unsigned char w, unsigned char h,unsigned short color) {

	IL9163_setAddrWindow(x0, y0, x0+w-1, y0+h-1);

	IL9163_RS_DAT_HIGH;	//RS=1;
	IL9163_CS_LOW;	//~CS
	for(x0=0;x0<2;x0++){
		for(y0=0;y0<h;y0++)
			IL9163_WrByte8(color >> 8);	  IL9163_WrByte8(color);
	}
	IL9163_CS_HIGH;	//CS

}
*/
// draw a circle outline
void IL9163_drawRect(unsigned char x0, unsigned char y0, unsigned char w, unsigned char h, unsigned short color)
{
    IL9163_drawFastHLine(x0,y0, w, color);
    IL9163_drawFastHLine(x0,y0+h-1, w, color);
    IL9163_drawFastVLine(x0,y0, h, color);
    IL9163_drawFastVLine(x0+w-1,y0, h, color);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x08
#define MADCTL_MH  0x04

void IL9163_setRotation(unsigned char m) {
	unsigned char rotation; //YOON

  IL9163_writeCommand(IL9163_MADCTL);

  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
	   IL9163_writeData(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
	   LCD_gfx._width  = IL9163_TFTWIDTH;
	   LCD_gfx._height = IL9163_TFTHEIGHT;
    break;
   case 1:
	   IL9163_writeData(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
	   LCD_gfx._width  = IL9163_TFTHEIGHT;
	   LCD_gfx._height = IL9163_TFTWIDTH;
    break;
   case 2:
	   IL9163_writeData(MADCTL_RGB);
	   LCD_gfx._width  = IL9163_TFTWIDTH;
	   LCD_gfx._height = IL9163_TFTHEIGHT;
    break;
   case 3:
	   IL9163_writeData(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
	   LCD_gfx._width  = IL9163_TFTHEIGHT;
	   LCD_gfx._height = IL9163_TFTWIDTH;
    break;
  }
}

void IL9163_invertDisplay(unsigned char i){//boolean i) {
	IL9163_writeCommand(i ? IL9163_INVON : IL9163_INVOFF);
}

// draw a character
void IL9163_GFX_drawChar(
		short x,
		short y,
		unsigned char c,
		unsigned short color,
		unsigned short bg,
		unsigned char size)
{
	char i,j;
    unsigned char line;
    if((x >= LCD_gfx._width) || // Clip right
     (y >= LCD_gfx._height)  || // Clip bottom
     ((x + 5 * size - 1) < 0)  || // Clip left
     ((y + 8 * size - 1) < 0)){   // Clip top

	  printf("IL9163_GFX_drawChar():Invalid \r\n");
	  return;
    }
    printf("IL9163_GFX_drawChar(%c)@(%u,%u)\r\n",c, x,y);
    for (i=0; i<6; i++ ) {
    	if (i == 5)
    		line = 0x0;
    	else
    		line = pgm_read_byte(font+(c*5)+i);
    	for (j = 0; j<8; j++) {
    		if (line & 0x1) {
    			if (size == 1) // default size
    				IL9163_drawPixel(x+i, y+j, color);
    			else {  // big size
    				IL9163_fillRect(x+(i*size), y+(j*size), size, size, color);
    			}
    		} else if (bg != color) {
    			if (size == 1) // default size
    				IL9163_drawPixel(x+i, y+j, bg);
    			else {  // big size
    				IL9163_fillRect(x+i*size, y+j*size, size, size, bg);
    			}
    		}
    		line >>= 1;
    	}
    }
}

void IL9163_GFX_write(unsigned char c) {

  if (c == '\n') {
	  LCD_gfx.cursor_y += LCD_gfx.textsize*8;
	  LCD_gfx.cursor_x = 0;
  } else if (c == '\r') {
    // skip em
  } else {
	  IL9163_GFX_drawChar(LCD_gfx.cursor_x, LCD_gfx.cursor_y, c, LCD_gfx.textcolor, LCD_gfx.textbgcolor, LCD_gfx.textsize);
	  LCD_gfx.cursor_x += LCD_gfx.textsize*6;
      if (LCD_gfx.wrap && (LCD_gfx.cursor_x > (LCD_gfx._width - LCD_gfx.textsize*6))) {
    	  LCD_gfx.cursor_y += LCD_gfx.textsize*8;
    	  LCD_gfx.cursor_x = 0;
      }
  }
}
void IL9163_GFX_setCursor(short x, short y) {
	LCD_gfx.cursor_x = x;
	LCD_gfx.cursor_y = y;
    //printf("(x,y)=(%u,%u)\r\n",LCD_gfx.cursor_x, LCD_gfx.cursor_y);
}

void IL9163_GFX_setTextSize(unsigned char s) {
	LCD_gfx.textsize = (s > 0) ? s : 1;
}

void IL9163_GFX_setTextColor(unsigned short c, unsigned short b) {
	LCD_gfx.textcolor = c;
	LCD_gfx.textbgcolor = IL9163_swapcolor(b); //BLACKTAB
}

void IL9163_GFX_setTextWrap(unsigned char w) {
	LCD_gfx.wrap = w;
}

void IL9163_GFX_drawBitmap(short x, short y, const unsigned char *bitmap, short w, short h)
{
	unsigned short color16;
    short i, j,k;

    k=0;
    color16 = 0;

    for(j=0; j<h; j++) {
      for(i=0; i<w; i++ ) {
    	color16 = bitmap[k++];
    	color16 = color16 << 8;
    	color16 = color16 + bitmap[k++];
    	IL9163_drawPixel(x+i, y+j, color16);
    }
  }
}
//YOON
void IL9163_drawBitmapWithSetWindow(short x, short y, short w, short h,  unsigned char *bitmap, unsigned char evenOdd)
{
  unsigned char hi, lo;
	unsigned short color16;
	short k;

    k=0;
    color16 = 0;

  if((x >= LCD_gfx._width) || (y >= LCD_gfx._height)) return;

  if((x + w - 1) >= LCD_gfx._width)  w = LCD_gfx._width  - x;

  if((y + h - 1) >= LCD_gfx._height) h = LCD_gfx._height - y;

  IL9163_setAddrWindow(x, y, x+w-1, y+h-1);

  IL9163_RS_DAT_HIGH;	//RS=1;
  IL9163_CS_LOW;	//~CS

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
    	color16 = bitmap[k++];
    	color16 = color16 << 8;
    	color16 = color16 + bitmap[k++];

    	//Only for test
    	if(evenOdd)
    		color16 = ~color16;//color16 = IL9163_swapcolor(color16);

  	    hi = color16 >> 8;
  	    lo = color16;
    	IL9163_WrByte8(hi);
    	IL9163_WrByte8(lo);
    }
  }
  IL9163_CS_HIGH;	//CS
}
//=== added =========
void IL9163_drawFastLine(unsigned char x, unsigned char y, unsigned char length, unsigned short color, unsigned char rotflag)
{
	if(rotflag)
		IL9163_setAddrWindow(x,y,x,y+length);
	else
		IL9163_setAddrWindow(x,y,x+length,y+1);

	//if (LCD_gfx.tabcolor == INITR_BLACKTAB)
	//	color = IL9163_swapcolor(color);

	unsigned char hi = color >> 8, lo = color;
	IL9163_RS_DAT_HIGH;	//RS=1;
	IL9163_CS_LOW;	//~CS

	while (length--) {
	  IL9163_WrByte8(hi);
	  IL9163_WrByte8(lo);
	}
	IL9163_CS_HIGH;	//CS
}
void IL9163_drawVerticalLine(unsigned char x0, unsigned char y0, unsigned char length, unsigned short color) {
	if(x0 > LCD_gfx._width) return;
	if(y0+length >= LCD_gfx._height) length = LCD_gfx._height - y0 - 1;
	IL9163_drawFastLine(x0, y0, length, color, 1);
	//void IL9163_drawFastVLine(short x, short y, short h, unsigned short color)
}

// fill a circle with multiple vertical lines
void IL9163_fillCircle(unsigned char x0, unsigned char y0, unsigned char r, unsigned short color) {
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  IL9163_drawVerticalLine(x0, y0-r, 2*r+1, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    IL9163_drawVerticalLine(x0+x, y0-y, 2*y+1, color);
    IL9163_drawVerticalLine(x0-x, y0-y, 2*y+1, color);
    IL9163_drawVerticalLine(x0+y, y0-x, 2*x+1, color);
    IL9163_drawVerticalLine(x0-y, y0-x, 2*x+1, color);
  }
}

// draw a circle outline
void IL9163_drawCircle(unsigned char x0, unsigned char y0, unsigned char r,
			unsigned short color) {
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  IL9163_drawPixel(x0, y0+r, color);
  IL9163_drawPixel(x0, y0-r, color);
  IL9163_drawPixel(x0+r, y0, color);
  IL9163_drawPixel(x0-r, y0, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    IL9163_drawPixel(x0 + x, y0 + y, color);
    IL9163_drawPixel(x0 - x, y0 + y, color);
    IL9163_drawPixel(x0 + x, y0 - y, color);
    IL9163_drawPixel(x0 - x, y0 - y, color);

    IL9163_drawPixel(x0 + y, y0 + x, color);
    IL9163_drawPixel(x0 - y, y0 + x, color);
    IL9163_drawPixel(x0 + y, y0 - x, color);
    IL9163_drawPixel(x0 - y, y0 - x, color);

  }
}

void IL9163_DispSingleColor(unsigned short int color){
	unsigned char i,j;
	IL9163_writeCommand(0x2c);
	for(i=0;i<160; i++){
		for(j=0;j<128; j++){
			IL9163_writeData(color>>8);
			IL9163_writeData(color&0xff);
		}
	}
}

void IL9163_displayHello()
{
	printf("Hello\r\n");
//    IL9163_displayString(1,100,"Hello YoonLAB");
//    IL9163_displayString(1,76,"Dots:128*160");
    //IL9163_displayString(1,88,"IC: IL9163R");
    //IL9163_displayString(1,112,"IL9163 HELLO 2013");
	IL9163_displayString(0,0,"IL9163 for STM");
   // IL9163_GFX_drawBitmap(0, 0, gImage_lcd, 160,80);//,  IL9163_RED);///
}

void IL9163_displayBitmap(){
	int i=0;
	while(1){//for(i=0;i<51;i++){ //
		//IL9163_GFX_drawBitmap(0+i*10, 8+i*10, gImage_lcd, 160,80);//128,  IL9163_RED);///
		IL9163_drawBitmapWithSetWindow(0, 8, 160,80, gImage_lcd, i%2);//160,80, gImage_lcd);
		//delayms(30);
		i++;
	}
}

void IL9163_displayValue(unsigned char x0,unsigned char y0, unsigned char val)
{
	char valstr[100];
	sprintf(valstr,"%d",val);
	printf("%s\r\n",valstr);
	IL9163_displayString(x0,y0,valstr);
}
void  IL9163_displayString(unsigned char x0,unsigned char y0,unsigned char *s)
{
	int i,j,k,x,y,xx;
	unsigned char qm;
	long int ulOffset;
	char  ywbuf[32],temp[2];
	IL9163_GFX_setTextColor	(
			IL9163_WHITE, //text color
			IL9163_BLACK); //IL9163_BLUE); //background color// 0XC552);//IL9163_RED);

	IL9163_GFX_setTextSize(1);//2);//1);

	for(i = 0; i<strlen((char*)s);i++)	{
		if(((unsigned char)(*(s+i))) >= 161)
		{
			temp[0] = *(s+i);
			temp[1] = '\0';
			return;
		}else{
			qm = *(s+i);
	  		xx=x0+i*6;
			IL9163_GFX_setCursor(xx,y0);
	  		IL9163_GFX_write(qm);
		}
	}
}
// Initialization for IL9163C screens with 8-bit Parallel Interface
void IL9163_initConfig_8bitParallel()
{

  IL9163_gpioSetup();

  IL9163_InitSeq(ILI9163_InitCmds);

  //Rotate 270 for horizontal view.
  IL9163_setRotation(3);

  IL9163_drawPixel(60, 60, IL9163_RED);
  IL9163_fillScreen(IL9163_BLACK);//0XC552);//IL9163_RED);
  //delayms(500);

  //printf("width=%u,height=%u\r\n",LCD_gfx._width, LCD_gfx._height);
  //IL9163_fillScreen(IL9163_RED);
  IL9163_fillScreen(IL9163_BLACK);
  //delayms(100);

  IL9163_DispSingleColor(0x001F);

  LCD_gfx.cursor_y = 0;
  LCD_gfx.cursor_x = 0;

  IL9163_GFX_setTextColor(0XC552, IL9163_WHITE);//IL9163_RED,IL9163_BLACK);
  IL9163_GFX_setCursor(0, 10);
  IL9163_GFX_setTextSize(1);
  //IL9163_GFX_write('a');

  IL9163_GFX_setCursor(0, 20);
  IL9163_GFX_setTextSize(1);
  //IL9163_GFX_write('b');


	IL9163_fillScreen(IL9163_BLACK);//IL9163_fillScreen(IL9163_RED);
}

void IL9163_ShowDemo()
{
	unsigned char i;
    IL9163_displayHello();
	//somedelay(1000000);
	IL9163_drawFastVLine(120,50,50,0x07E0);
	//somedelay(1000000);
	IL9163_drawFastHLine(1,50,150,IL9163_GREEN);
	//somedelay(1000000);
	IL9163_fillRect(100,1,20,20,IL9163_BLUE);
	//somedelay(1000000);

	IL9163_fillScreen(IL9163_BLUE);
	//IL9163_fillScreen(IL9163_GREEN);
	//IL9163_fillScreen(IL9163_RED);
	//IL9163_fillScreen(IL9163_BLACK);

	IL9163_displayBitmap();

	for(i=0;i<5;i++){
		//IL9163_drawCircle(40+(i*10),100,6,IL9163_GREEN);//IL9163_drawCircle(40+(i*5),80+(i*5),10,IL9163_GREEN);
		delayms(100);
		IL9163_fillCircle(40+(i*15),100,4,IL9163_YELLOW); //IL9163_fillCircle(40+(i*5),80+(i*5),8,IL9163_RED);
	}

	IL9163_drawRect(50, 120, 40, 20, IL9163_YELLOW);
	IL9163_fillRect(51, 121, 38, 18, IL9163_CYAN);
}


void stmIL9163_8Bit_LcdLoop(){
	unsigned char i;
    printf("stmIL9163 : 8Bit LCD Test");


	IL9163_initConfig_8bitParallel();
	IL9163_displayString(1,144,"GPWS");
	IL9163_displayString(1,152,"Proximity : ");

	IL9163_ShowDemo();

	while(1){
		for(i=0;i<256;i++){
			IL9163_displayString(1,152,"Proximity : ");
			IL9163_displayValue(80, 152, i);
			delayms(1000);
		}
	}
}
#endif

