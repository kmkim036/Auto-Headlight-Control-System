/***************************************************
  This is a library for the HY-1.8" SPI display with ST7735S controller.
  Use SPI to communicate.
  *RST is optional
  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution

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

#if (LCDCONT_IS == LCDCONT_IS_ST7735)

#if (PROCESSOR == PROCESSOR_STM32F103RCT6)
//=============== SPI ===========================
//LCD Dimension 128*160 Pixels 2.8"
//Dots:128*160;
//IC: ST7735R;

#define WE_SUPPORT_XPT2046_TOUCH
#undef WE_SUPPORT_XPT2046_TOUCH

//struct _LCD_gfx LCD_gfx;
extern struct _LCD_gfx LCD_gfx;
extern struct _LCD_clock LCD_gauge;

//================= graphic functions ==============================
void ST7735_setWindows(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1) {

	LCD_Spi_WriteCommand(ST7735_CASET); // Column addr set
	LCD_Spi_WriteData(0x00);
	LCD_Spi_WriteData(x0+LCD_gfx.colstart);     // XSTART
	LCD_Spi_WriteData(0x00);
	LCD_Spi_WriteData(x1+LCD_gfx.colstart);     // XEND

	LCD_Spi_WriteCommand(ST7735_RASET); // Row addr set
	LCD_Spi_WriteData(0x00);
	LCD_Spi_WriteData(y0+LCD_gfx.rowstart);     // YSTART
	LCD_Spi_WriteData(0x00);
	LCD_Spi_WriteData(y1+LCD_gfx.rowstart);     // YEND

	LCD_Spi_WriteCommand(ST7735_RAMWR); // write to RAM
}

void ST7735_initSeq(unsigned char nCS)
{
	LCD_gfx._width  = LCD_TFTWIDTH;
	LCD_gfx._height = LCD_TFTHEIGHT;
	LCD_gfx.colstart  = LCD_gfx.rowstart = 0; 	// May be overridden in init func
	LCD_gfx.tabcolor = INITR_BLACKTAB; 			//ST7735 specific

	//ST7735_commandList(Rcmd1);
	// Init for 7735R, part 1 (red or green tab) - Total 15 commands
	LCD_Spi_WriteCommand(ST7735_SWRESET); delayms(150);	//  1: Software reset/150 ms delay
	LCD_Spi_WriteCommand(ST7735_SLPOUT ); delayms(500); //  2: Out of sleep mode/500 ms delay
	LCD_Spi_WriteCommand(ST7735_FRMCTR1); LCD_Spi_WriteData(0x01); LCD_Spi_WriteData(0x2C);LCD_Spi_WriteData(0x2D); //  3: Frame rate ctrl - normal mode   Rate = fosc/(1x2+40) * (LINE+2C+2D)
	LCD_Spi_WriteCommand(ST7735_FRMCTR2); LCD_Spi_WriteData(0x01); LCD_Spi_WriteData(0x2C);LCD_Spi_WriteData(0x2D); //  4: Frame rate control - idle mode,  Rate = fosc/(1x2+40) * (LINE+2C+2D)
	LCD_Spi_WriteCommand(ST7735_FRMCTR3);//  5: Frame rate ctrl - partial mode
		LCD_Spi_WriteData(0x01); LCD_Spi_WriteData(0x2C);LCD_Spi_WriteData(0x2D); //     Dot inversion mode
		LCD_Spi_WriteData(0x01); LCD_Spi_WriteData(0x2C);LCD_Spi_WriteData(0x2D); //     Line inversion mode
	LCD_Spi_WriteCommand(ST7735_INVCTR); LCD_Spi_WriteData( 0x07);                //     No inversion
	LCD_Spi_WriteCommand(ST7735_PWCTR1); //  7: Power control, 3 args, no delay:
		LCD_Spi_WriteData(0xA2); LCD_Spi_WriteData(0x02); //     -4.6V
		LCD_Spi_WriteData(0x84);  //     AUTO mode
	LCD_Spi_WriteCommand(ST7735_PWCTR2); //  8: Power control, 1 arg, no delay:
		LCD_Spi_WriteData(0xC5); //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
	LCD_Spi_WriteCommand(ST7735_PWCTR3);   //  9: Power control, 2 args, no delay:
		LCD_Spi_WriteData(0x0A);           //     Opamp current small
		LCD_Spi_WriteData(0x00);           //     Boost frequency
	LCD_Spi_WriteCommand(ST7735_PWCTR4);   // 10: Power control, 2 args, no delay:
		LCD_Spi_WriteData(0x8A);                   //     BCLK/2, Opamp current small & Medium low
		LCD_Spi_WriteData(0x2A);
	LCD_Spi_WriteCommand(ST7735_PWCTR5);	// , 2      ,  // 11: Power control, 2 args, no delay:
	 	 LCD_Spi_WriteData(0x8A);LCD_Spi_WriteData(0xEE);
 	LCD_Spi_WriteCommand(ST7735_VMCTR1); LCD_Spi_WriteData(0x0E);// 12: Power control, 1 arg, no delay:
 	LCD_Spi_WriteCommand(ST7735_INVOFF);	// , 0      ,  // 13: Don't invert display, no args, no delay
 	LCD_Spi_WriteCommand(ST77XX_MADCTL);  //  , 1      ,  // 14: Memory access control (directions), 1 arg:
 		LCD_Spi_WriteData(0xC8);       //     row addr/col addr, bottom to top refresh
	LCD_Spi_WriteCommand(ST7735_COLMOD);// , 1      ,  // 15: set color mode, 1 arg, no delay:
		LCD_Spi_WriteData(0x55);                //16-bit color//was (0x05);
#if 0
	//ST7735_initR(INITR_BLACKTAB); //7735S - BlackTab;(INITR_GREENTAB); //7735S - GreenTab//(INITR_REDTAB); //7735S - RedTab
	//ST7735_initB(); //7735S - Red/GreenTab
#else
	//== initR with black tab : Init for 7735R, part 2 (red tab only) : Rcmd2red[]
	LCD_Spi_WriteCommand(ST7735_CASET); //Column addr set, 4 args, no delay:
		LCD_Spi_WriteData(0x00);  LCD_Spi_WriteData(0x00); //XSTART = 0
		LCD_Spi_WriteData(0x00);  LCD_Spi_WriteData(0x7F); //XEND = 127
	LCD_Spi_WriteCommand(ST7735_RASET); //  2: Row addr set, 4 args, no delay:
		LCD_Spi_WriteData(0x00);  LCD_Spi_WriteData(0x00); //YSTART = 0
		LCD_Spi_WriteData(0x00);  LCD_Spi_WriteData(0x9F); //YEND = 159

	//== initR with black tab : Init for 7735R, part 3 (red or green tab) : Rcmd3red[]
	LCD_Spi_WriteCommand(ST7735_GMCTRP1); // 16      , //  1: Magical unicorn dust, 16 args, no delay:
		LCD_Spi_WriteData(0x02);  LCD_Spi_WriteData(0x1c); LCD_Spi_WriteData(0x07);  LCD_Spi_WriteData(0x12);
		LCD_Spi_WriteData(0x37);  LCD_Spi_WriteData(0x32); LCD_Spi_WriteData(0x29);  LCD_Spi_WriteData(0x2d);
		LCD_Spi_WriteData(0x29);  LCD_Spi_WriteData(0x25); LCD_Spi_WriteData(0x2B);  LCD_Spi_WriteData(0x39);
		LCD_Spi_WriteData(0x00);  LCD_Spi_WriteData(0x01); LCD_Spi_WriteData(0x03);  LCD_Spi_WriteData(0x10);
	LCD_Spi_WriteCommand(ST7735_GMCTRN1); // 16      , //  1: Sparkles and rainbows, 16 args, no delay:
		LCD_Spi_WriteData(0x03);  LCD_Spi_WriteData(0x1d); LCD_Spi_WriteData(0x07);  LCD_Spi_WriteData(0x06);
		LCD_Spi_WriteData(0x2E);  LCD_Spi_WriteData(0x2C); LCD_Spi_WriteData(0x29);  LCD_Spi_WriteData(0x2D);
		LCD_Spi_WriteData(0x2E);  LCD_Spi_WriteData(0x2E); LCD_Spi_WriteData(0x37);  LCD_Spi_WriteData(0x3F);
		LCD_Spi_WriteData(0x00);  LCD_Spi_WriteData(0x00); LCD_Spi_WriteData(0x02);  LCD_Spi_WriteData(0x10);
	LCD_Spi_WriteCommand(ST7735_NORON); delayms(10);	//  ,    DELAY, //  3: Normal display on, no args, w/delay
	LCD_Spi_WriteCommand(ST7735_DISPON); delayms(100);	//  4: Main screen turn on, no args w/delay
#endif

	  //Should be for BlackTab
	  //if(options == INITR_BLACKTAB){
	  LCD_Spi_WriteCommand(ST77XX_MADCTL);	  LCD_Spi_WriteData(0xC0);
	  //}

	  //Rotate 270 for horizontal view.
	  LCD_ST77XX_setRotation(3);

	  //LCD_drawPixel(60, 60, ST7735_RED);
	  //LCD_fillScreen(COLOR_BLACK);//0XC552);//ST7735_RED);
	  //delayms(5000);

	  LCD_fillScreen(COLOR_BLACK);	  //ST7735_DispSingleColor(0x001F);

	  //colstart = 2;//YOON
	  //rowstart = 1;//YOON

	  GFX_setTextColor(ST7735_BLACK, COLOR_WHITE);//foreground/background
	  GFX_setCursor(0, 10);
	  GFX_setTextSize(1);
	  LCD_writeCharAtCursor('a');

	  delayms(1000);

	  GFX_setCursor(0, 20);
	  LCD_writeCharAtCursor('b');
	  delayms(1000);

	  LCD_fillScreen(COLOR_BLACK);
	  delayms(1000);

	  LCD_drawFastVLine(120,50,50,COLOR_RED);
	  delayms(1000);

	  LCD_drawFastHLine(1,50,150,COLOR_GREEN);
	  delayms(1000);

	  LCD_fillRect(100,1,20,20,COLOR_BLUE);
	  delayms(1000);

	  LCD_displayHello("HELLO");
	  printf("Init Done\r\n");
}

extern const unsigned char gImage_lcd[];

void ST7735_ShowDemo()
{
	unsigned char i;

    LCD_displayHello("ST7735 for STM");

    LCD_fillScreen(COLOR_BLACK);

    //LCD_drawBitmap(gImage_lcd);
    LCD_drawImageOnSpecifiedAreaWindows(0, 0, 159,80, gImage_lcd);
	//LCD_Gauge_Config_ImgWithPalette(VUmeter128128, LCD_TFTWIDTH/2, LCD_TFTHEIGHT*65/100);//aviGauge;//clockface8;);


    delayms(1000);
	for(i=0;i<5;i++){
		//LCD_drawCircle(40+(i*10),100,6,COLOR_GREEN);//LCD_drawCircle(40+(i*5),80+(i*5),10,COLOR_GREEN);
		delayms(400);
		LCD_fillCircle(40+(i*15),100,4,COLOR_YELLOW); //LCD_fillCircle(40+(i*5),80+(i*5),8,ST7735_RED);
	}

	LCD_drawRect(50, 120, 40, 20, COLOR_YELLOW);
	LCD_fillRect(51, 121, 38, 18, COLOR_CYAN);
}

extern void stmPWM_Config(unsigned char duty);
extern unsigned char XPT2046_getPosByIrq(struct _TouchScreen_posxy *posxy);
//===============================================================================================
void stmSpiSt7735HY18LcdLoop(){
	unsigned char i;
	struct _TouchScreen_posxy posxy;
	char str[80];
	unsigned char nCS;

    printf("stmST7735(HY1.8) SPI LCD Test");
    printf("We have several problems\r\n");
    printf("LCD can support 10Mbps SPI, but\r\n");
    printf("XPT2046 Touch Controller can only support 1Mbps SPI.\r\n");
    printf("Because we use a single SPI, we should do with 1Mbps SPI.\r\n");

	//stmPWM_Config(75); //BL 70% - PA3

	nCS = 0;

	LCD_spi_GpioSetup();

	ST7735_initSeq(nCS);

	ST7735_ShowDemo();

	LCD_displayString(50,144,"GPWS");
	LCD_displayString(50,152,"Proximity : ");

	//GFX_setCursor(8, 10);
#ifdef	WE_SUPPORT_XPT2046_TOUCH
	XPT2046_touch_init(0);
#endif

	while(1){
#ifdef	WE_SUPPORT_XPT2046_TOUCH
		if(XPT2046_getPosByIrq(&posxy)){
		//if(XPT2046_getPosByPolling(&posxy)){ //XPT2046_read_once();
			printf("x,y =(%u,%u)\r\n", posxy.x, posxy.y);
			snprintf(str,16,"X,Y=(%u,%u)    ", posxy.x, posxy.y);
			LCD_displayString(30,56,str);

			sprintf(str,"Proximity =%u", posxy.x);
			LCD_displayString(30,80,str);
		}
#else
		sprintf(str,"Proximity =%u", 1);
		LCD_displayString(30,80,str);
#endif
		//delayms(10);
	}
}
#endif

/*
 *
 * void ST7735_pushColor(unsigned short color) {

	LCD_RS_HIGH; 	//RS=1;
	LCD_CS0_LOW; 	//~CS

	if (LCD_gfx.tabcolor == INITR_BLACKTAB)
		color = gfx_swapcolor(color);
	stmSpiWrByte(color >> 8);	stmSpiWrByte(color);

	LCD_CS0_HIGH; 	//CS
}
void LCD_drawPixel(short x, short y, unsigned short color) {

  if((x < 0) ||(x >= LCD_gfx._width) || (y < 0) || (y >= LCD_gfx._height)){
	  printf("draw Error\r\n");
	  return;
  }

  ST7735_setAddrWindow(x,y,x+1,y+1);

  LCD_RS_HIGH; 	//RS=1;
  LCD_CS0_LOW; 	//~CS

  if (LCD_gfx.tabcolor == INITR_BLACKTAB)
	  color = gfx_swapcolor(color);

  stmSpiWrByte(color >> 8);  stmSpiWrByte(color);

  LCD_CS0_HIGH;	//CS
}

 *
void LCD_drawFastVLine(short x, short y, short h, unsigned short color) {

  // Rudimentary clipping
  if((x >= LCD_gfx._width) || (y >= LCD_gfx._height)) return;
  if((y+h-1) >= LCD_gfx._height) h = LCD_gfx._height-y;
  ST7735_setAddrWindow(x, y, x, y+h-1);

  if (LCD_gfx.tabcolor == INITR_BLACKTAB)   color = gfx_swapcolor(color);

  unsigned char hi = color >> 8, lo = color;
  LCD_RS_HIGH;	//RS=1;
  LCD_CS0_LOW;	//~CS

  while (h--) {
	  stmSpiWrByte(hi);
	  stmSpiWrByte(lo);
  }
  LCD_CS0_HIGH;	//CS
}

void LCD_drawFastHLine(short x, short y, short w, unsigned short color) {

  // Rudimentary clipping
  if((x >= LCD_gfx._width) || (y >= LCD_gfx._height)) return;
  if((x+w-1) >= LCD_gfx._width)  w = LCD_gfx._width-x;
  ST7735_setAddrWindow(x, y, x+w-1, y);

  if (LCD_gfx.tabcolor == INITR_BLACKTAB)
	  color = gfx_swapcolor(color);

  unsigned char hi = color >> 8, lo = color;
  LCD_RS_HIGH;	//RS=1;
  LCD_CS0_LOW;	//~CS

  while (w--) {
	  stmSpiWrByte(hi);
	  stmSpiWrByte(lo);
  }
  LCD_CS0_HIGH;	//CS=1
}






 * inline unsigned short gfx_swapcolor(unsigned short x) {
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}
 *void LCD_fillScreen(unsigned short color) {
	LCD_fillRect(0, 0,  LCD_gfx._width, LCD_gfx._height, color);
}
 * // fill a rectangle
void LCD_fillRect(short x, short y, short w, short h,  unsigned short color)
{
  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= LCD_gfx._width) || (y >= LCD_gfx._height)) return;
  if((x + w - 1) >= LCD_gfx._width)  w = LCD_gfx._width  - x;
  if((y + h - 1) >= LCD_gfx._height) h = LCD_gfx._height - y;

  if (LCD_gfx.tabcolor == INITR_BLACKTAB)
	  color = gfx_swapcolor(color);

  ST7735_setAddrWindow(x, y, x+w-1, y+h-1);

  unsigned char hi = color >> 8, lo = color;
  LCD_RS_HIGH;	//RS=1;
  LCD_CS0_LOW;	//~CS

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
    	stmSpiWrByte(hi);
    	stmSpiWrByte(lo);
    }
  }
  LCD_CS0_HIGH;	//CS
}

// fill a rectangle
void LCD_fillRect(unsigned char x0, unsigned char y0, unsigned char w, unsigned char h,unsigned short color) {

	ST7735_setAddrWindow(x0, y0, x0+w-1, y0+h-1);

	LCD_RS_HIGH;	//RS=1;
	LCD_CS0_LOW;	//~CS
	for(x0=0;x0<2;x0++){
		for(y0=0;y0<h;y0++)
			stmSpiWrByte(color >> 8);	  stmSpiWrByte(color);
	}
	LCD_CS0_HIGH;	//CS

}

// draw a circle outline
void LCD_drawRect(unsigned char x0, unsigned char y0, unsigned char w, unsigned char h, unsigned short color)
{
    LCD_drawFastHLine(x0,y0, w, color);
    LCD_drawFastHLine(x0,y0+h-1, w, color);
    LCD_drawFastVLine(x0,y0, h, color);
    LCD_drawFastVLine(x0+w-1,y0, h, color);
}
// fill a circle with multiple vertical lines
void LCD_fillCircle(unsigned char x0, unsigned char y0, unsigned char r, unsigned short color) {
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  LCD_drawVerticalLine(x0, y0-r, 2*r+1, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    LCD_drawVerticalLine(x0+x, y0-y, 2*y+1, color);
    LCD_drawVerticalLine(x0-x, y0-y, 2*y+1, color);
    LCD_drawVerticalLine(x0+y, y0-x, 2*x+1, color);
    LCD_drawVerticalLine(x0-y, y0-x, 2*x+1, color);
  }
}

void LCD_invertDisplay(unsigned char i){//boolean i) {
	LCD_Spi_WriteCommand(i ? ST7735_INVON : ST7735_INVOFF);
}
*/

/*
// draw a character
void LCD_drawChar(
		short x,
		short y,
		unsigned char c,
		unsigned short color,
		unsigned short bg,
		unsigned char size)
{
	char i,j;
    unsigned char line;
    if((x >= LCD_gfx._width)            || // Clip right
     (y >= LCD_gfx._height)           || // Clip bottom
     ((x + 5 * size - 1) < 0) || // Clip left
     ((y + 8 * size - 1) < 0)){   // Clip top

	  //printf("LCD_drawChar():Invalid \r\n");
	  return;
    }
    //printf("LCD_drawChar(%c)@(%u,%u)\r\n",c, x,y);
    for (i=0; i<6; i++ ) {
    	if (i == 5)
    		line = 0x0;
    	else
    		line = pgm_read_byte(font+(c*5)+i);
    	for (j = 0; j<8; j++) {
    		if (line & 0x1) {
    			if (size == 1) // default size
    				LCD_drawPixel(x+i, y+j, color);
    			else {  // big size
    				LCD_fillRect(x+(i*size), y+(j*size), size, size, color);
    			}
    		} else if (bg != color) {
    			if (size == 1) // default size
    				LCD_drawPixel(x+i, y+j, bg);
    			else {  // big size
    				LCD_fillRect(x+i*size, y+j*size, size, size, bg);
    			}
    		}
    		line >>= 1;
    	}
    }
}

void LCD_writeCharAtCursor(unsigned char c) {

  if (c == '\n') {
	  LCD_gfx.cursor_y += LCD_gfx.textsize*8;
	  LCD_gfx.cursor_x = 0;
  } else if (c == '\r') {
    // skip em
  } else {
	  LCD_drawChar(LCD_gfx.cursor_x, LCD_gfx.cursor_y, c, LCD_gfx.textcolor, LCD_gfx.textbgcolor, LCD_gfx.textsize);
	  LCD_gfx.cursor_x += LCD_gfx.textsize*6;
      if (LCD_gfx.wrap && (LCD_gfx.cursor_x > (LCD_gfx._width - LCD_gfx.textsize*6))) {
    	  LCD_gfx.cursor_y += LCD_gfx.textsize*8;
    	  LCD_gfx.cursor_x = 0;
      }
  }
}

void GFX_setTextColor(unsigned short c, unsigned short b) {
	LCD_gfx.textcolor = c;
	LCD_gfx.textbgcolor = gfx_swapcolor(b); //BLACKTAB
}

//=== added =========
void LCD_drawFastLine(unsigned char x, unsigned char y, unsigned char length, unsigned short color, unsigned char rotflag)
{
	if(rotflag)
		ST7735_setAddrWindow(x,y,x,y+length);
	else
		ST7735_setAddrWindow(x,y,x+length,y+1);

	//if (LCD_gfx.tabcolor == INITR_BLACKTAB)
	//	color = gfx_swapcolor(color);

	unsigned char hi = color >> 8, lo = color;
	LCD_RS_HIGH;	//RS=1;
	LCD_CS0_LOW;	//~CS

	while (length--) {
	  stmSpiWrByte(hi);
	  stmSpiWrByte(lo);
	}
	LCD_CS0_HIGH;	//CS
}




void LCD_drawVerticalLine(unsigned char x0, unsigned char y0, unsigned char length, unsigned short color) {
	if(x0 > LCD_gfx._width) return;
	if(y0+length >= LCD_gfx._height) length = LCD_gfx._height - y0 - 1;
	LCD_drawFastLine(x0, y0, length, color, 1);
	//void LCD_drawFastVLine(short x, short y, short h, unsigned short color)
}



// draw a circle outline
void LCD_drawCircle(unsigned char x0, unsigned char y0, unsigned char r,
			unsigned short color) {
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  LCD_drawPixel(x0, y0+r, color);
  LCD_drawPixel(x0, y0-r, color);
  LCD_drawPixel(x0+r, y0, color);
  LCD_drawPixel(x0-r, y0, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    LCD_drawPixel(x0 + x, y0 + y, color);
    LCD_drawPixel(x0 - x, y0 + y, color);
    LCD_drawPixel(x0 + x, y0 - y, color);
    LCD_drawPixel(x0 - x, y0 - y, color);

    LCD_drawPixel(x0 + y, y0 + x, color);
    LCD_drawPixel(x0 - y, y0 + x, color);
    LCD_drawPixel(x0 + y, y0 - x, color);
    LCD_drawPixel(x0 - y, y0 - x, color);

  }
}
*/




/*
 void ST7735_GFX_drawBitmap(short x, short y, const unsigned char *bitmap, short w, short h)
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
    	LCD_drawPixel(x+i, y+j, color16);
    }
  }
}
void LCD_drawBitmap(){
	int i=0;

	return;
	//for(i=0;i<4;i++){
		ST7735_GFX_drawBitmap(0+i*10, 8+i*10, gImage_lcd, 160,80);//128,  ST7735_RED);///
	//}
}

void LCD_displayValue(unsigned char x0,unsigned char y0, unsigned char val)
{
	char valstr[100];
	sprintf(valstr,"%d",val);
	printf("%s\r\n",valstr);
	LCD_displayString(x0,y0,valstr);
}

void  LCD_displayString(unsigned char x0,unsigned char y0,unsigned char *s)
{
	int i,j,k,x,y,xx;
	unsigned char qm;
	long int ulOffset;
	char  ywbuf[32],temp[2];
	GFX_setTextColor	(
			COLOR_WHITE, //text color
			COLOR_BLACK); //COLOR_BLUE); //background color// 0XC552);//ST7735_RED);

	GFX_setTextSize(1);//2);//1);

	for(i = 0; i<strlen((char*)s);i++)	{
		if(((unsigned char)(*(s+i))) >= 161)
		{
			temp[0] = *(s+i);
			temp[1] = '\0';
			return;
		}else{
			qm = *(s+i);
	  		xx=x0+i*6;
			GFX_setCursor(xx,y0);
	  		LCD_writeCharAtCursor(qm);
		}
	}
}

void LCD_ST77XX_setRotation(unsigned char m) {
	unsigned char rotation; //YOON

  LCD_Spi_WriteCommand(ST77XX_MADCTL);

  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
	   LCD_Spi_WriteData(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
	   LCD_gfx._width  = LCD_TFTWIDTH;
	   LCD_gfx._height = LCD_TFTHEIGHT;
    break;
   case 1:
	   LCD_Spi_WriteData(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
	   LCD_gfx._width  = LCD_TFTHEIGHT;
	   LCD_gfx._height = LCD_TFTWIDTH;
    break;
   case 2:
	   LCD_Spi_WriteData(MADCTL_RGB);
	   LCD_gfx._width  = LCD_TFTWIDTH;
	   LCD_gfx._height = LCD_TFTHEIGHT;
    break;
   case 3:
	   LCD_Spi_WriteData(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
	   LCD_gfx._width  = LCD_TFTHEIGHT;
	   LCD_gfx._height = LCD_TFTWIDTH;
    break;
  }
}
*/
#if 0
// Initialization for ST7735B screens
void ST7735_initB(void) {

	LCD_gfx.tabcolor = 0;//INITR_BLACKTAB; //ST7735S.

	ST7735_commonInit(Bcmd, nCS);

	LCD_fillScreen(COLOR_BLACK);
	//LCD_drawPixel(60, 60, ST7735_RED);

	delayms(100);
	ST7735_DispSingleColor(0x001F);

	LCD_gfx.colstart = 2;//YOON
	LCD_gfx.rowstart = 1;//YOON

	LCD_gfx.cursor_y = 0;
	LCD_gfx.cursor_x = 0;
	GFX_setTextColor(ST7735_RED,COLOR_WHITE);
	GFX_setCursor(0, 10);
	GFX_setTextSize(1);
	LCD_writeCharAtCursor('a');
	LCD_fillScreen(ST7735_RED);
	//LCD_drawFastVLine(20,50,50,0x07E0);
	delayms(100);
}

// Initialization for ST7735R screens (green or red tabs)
void ST7735_initR(unsigned char options) {
	//tabcolor = options;

  if(options == INITR_GREENTAB) {
	  ST7735_commandList(Rcmd2green);
	  LCD_gfx.colstart = 2;
	  LCD_gfx.rowstart = 1;
  } else {
	 ST7735_commandList(Rcmd2red);// colstart, rowstart left at default '0' values
  }
  ST7735_commandList(Rcmd3);

  //Should be for BlackTab
  if(options == INITR_BLACKTAB){
	  LCD_Spi_WriteCommand(ST7735_MADCTL);
	  LCD_Spi_WriteData(0xC0);
  }

  LCD_gfx.tabcolor = options;

  //Rotate 270 for horizontal view.
  LCD_ST77XX_setRotation(3);

  //LCD_drawPixel(60, 60, ST7735_RED);
  //LCD_fillScreen(COLOR_BLACK);//0XC552);//ST7735_RED);
  //delayms(5000);

  //printf("width=%u,height=%u\r\n",LCD_gfx._width, LCD_gfx._height);
  //LCD_fillScreen(ST7735_RED);
  LCD_fillScreen(COLOR_BLACK);
  delayms(3000);

  ST7735_DispSingleColor(0x001F);

  //colstart = 2;//YOON
  //rowstart = 1;//YOON

  LCD_gfx.cursor_y = 0;
  LCD_gfx.cursor_x = 0;

	GFX_setTextColor(0XC552, COLOR_WHITE);//ST7735_RED,COLOR_BLACK);
	GFX_setCursor(0, 10);
	GFX_setTextSize(1);
	LCD_writeCharAtCursor('a');

	GFX_setCursor(0, 20);
	GFX_setTextSize(1);
	LCD_writeCharAtCursor('b');

	  //LCD_fillScreen(ST7735_RED);
	  LCD_fillScreen(COLOR_BLACK);
	somedelay(1000000);
	LCD_drawFastVLine(120,50,50,0x07E0);
	somedelay(1000000);
	LCD_drawFastHLine(1,50,150,COLOR_GREEN);
	somedelay(1000000);
	LCD_fillRect(100,1,20,20,COLOR_BLUE);
	somedelay(1000000);
	LCD_displayHello("HELLO");
}
#endif

//Init config commands lists
#if 0
//===========   Bcmd for all 7735R =========
static unsigned char
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay ..120msec
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay ..120msec
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,   //0xC8??                //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     YSTART = 1
      0x00, 0x81,             //     YEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay
  //=============Rcmd1 for 7735R(RED/GREENTAB)=========
  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color
  //=============Rcmd2 for 7735R(GREENTAB ONLY)=========
  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  //=============Rcmd3 for 7735R(RED/GREENTAB)=========
Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100
};                  //     100 ms delay

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void ST7735_commandList(unsigned char *addr) {

  unsigned char  numCommands, numArgs, savedNumCmds;
  long ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  //printf("CommandList Number = %d\r\n",numCommands);
  savedNumCmds = numCommands;
  //numCommands = 1;
  while(numCommands--) {                 // For each command...
	//printf("Cmd = 0x%x; ",*addr);
	LCD_Spi_WriteCommand(pgm_read_byte(addr++)); //   Read command, and issue the command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
    	//printf("Arg = 0x%x\r\n",*addr);
    	LCD_Spi_WriteData(pgm_read_byte(addr++));  //Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delayms(ms);//somedelay(ms*8000);// delay(ms); <<12=ms
    }
  }
}

//Fill a color on entire screen.
void ST7735_DispSingleColor(unsigned short int color565){
	unsigned char i,j;
	LCD_Spi_WriteCommand(0x2c);
	for(i=0;i<LCD_gfx.width; i++){ //160
		for(j=0;j<LCD_gfx.height; j++){ //128
			LCD_Spi_WriteData(color565 >> 8);
			LCD_Spi_WriteData(color565 & 0xff);
		}
	}
}
#endif
#endif
