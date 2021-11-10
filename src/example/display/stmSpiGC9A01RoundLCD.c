/***************************************************
  This is a library for the GC9A01 240*240 Round LCD with SPI display
  Use SPI to communicate.
  *RST is optional
  - ST7739 is suited for 240x320.
  - But we have 240x240
  - SPI Mode3
  - Run on STM32F103C8T6 and STM32F103RCT6
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
#if (LCDCONT_IS == LCDCONT_IS_GC9A01)

extern unsigned char g_bI2CModuleConfigDone;
extern I2C_TypeDef *gI2Cx;
//================ GC9A01 ============================

//LCD Dimension 240*240 Pixels 1.28" ROUND

extern struct _LCD_gfx LCD_gfx;
extern struct _LCD_clock LCD_gauge;

extern const unsigned char gImage_lcd[];

//============= command write with RS=0 over SPI =========
/*
void LCD_Spi_WriteData_preRS_H(unsigned char c) {
	//LCD_RS_HIGH; 	//RS=1; -- DATA
	LCD_CS0_LOW; 	//CS=0
	stmSpiWrByte(c);
	LCD_CS0_HIGH; 	//CS
}
*/
/********************************************************************************
function:	Sets the start position and size of the display area
parameter:
		Xstart 	:   X direction Start coordinates(equal or less than Xend)
		Ystart  :   Y direction Start coordinates
		Xend    :   X direction end coordinates
		Yend    :   Y direction end coordinates
********************************************************************************/
//different from ST7789
//specific for GC9A01

//for a given pixel (xstart == xend; ystart == Yend)
//org src = LCD_SetCursor()
void GC9A01_setWindows(unsigned short Xstart, unsigned short Ystart, unsigned short Xend, unsigned short Yend)
{
    //set the X coordinates
    LCD_Spi_WriteCommand(0x2A); //Column addr set
    LCD_Spi_WriteData(0x00); LCD_Spi_WriteData(Xstart & 0xff); // XSTART COLUMN (MSB/LSB)
	LCD_Spi_WriteData(0x00); LCD_Spi_WriteData(Xend  & 0xff); // XEND COLUMN (MSB/LSB)

    //set the Y coordinates
    LCD_Spi_WriteCommand(0x2B); //Row addr set
    LCD_Spi_WriteData(0x00);	LCD_Spi_WriteData(Ystart & 0xff); //YSTART
	LCD_Spi_WriteData(0x00);    LCD_Spi_WriteData(Yend & 0xff); //// YEND

    LCD_Spi_WriteCommand(0X2C); //write to RAM
}

/********************************************************************************
function:	Set the resolution and scanning method of the screen
parameter:  We can mirror, flip,...
		Scan_dir:   Scan direction
********************************************************************************/
static void GC9A01_SetOrientation(unsigned char Scan_dir) //SetRotation, SetAttributes
{
    //MAP : D7 D6 D5 D4 - D3  D2 D1 D0
 	//para: MY MX MV ML - BGR MH  X  X
	// MY : Row Address Order
	// MX  : Col Address Order
	// MV  : R/C Exchange
	// ML  : Vertical Refresh Order
	// BGR
	// MH  : Horizontal Refresh Order
    unsigned char memoryAccessRegVal;
    //Get the screen scan direction
	LCD_gfx.SCAN_DIR = Scan_dir;
	LCD_gfx.colstart  = LCD_gfx.rowstart = 0; // May be overridden in init func

    //Get GRAM and LCD width and height
    if(Scan_dir == LCD_SCANDIR_HORIZONTAL) {
    	LCD_gfx._height	= LCD_TFTHEIGHT; //240
    	LCD_gfx._width    = LCD_TFTWIDTH;   //240
        memoryAccessRegVal = 0XC8; //1100-1000 MYMXMV=110 ML=0 | BGR=1
    } else { //Default = LCD_SCANDIR_VERTICAL
    	LCD_gfx._height	= LCD_TFTWIDTH;
    	LCD_gfx._width    = LCD_TFTHEIGHT;
        memoryAccessRegVal = 0X68; //0110-1000  MYMXMV=011|ML=0 | BGR=1
    }

    // Set the read / write scan direction of the frame memory
    LCD_Spi_WriteCommand(GC9A01_MADCTL);
    LCD_Spi_WriteData(memoryAccessRegVal);
}

//==== HW CONFIG ============================================================
//Init config commands
void GC9A01_InitSeq(void) {

	unsigned char  numCommands, numArgs, savedNumCmds;
  	long ms;

  	LCD_Spi_WriteCommand(0xEF); //Inner Register Enable 2

  	LCD_Spi_WriteCommand(0xEB);  LCD_Spi_WriteData(0x14);

  	LCD_Spi_WriteCommand(0xFE);//Inner Register Enable 1

  	LCD_Spi_WriteCommand(0xEF);//Inner Register Enable 2

  	LCD_Spi_WriteCommand(0xEB);  LCD_Spi_WriteData(0x14);

  	LCD_Spi_WriteCommand(0x84);  LCD_Spi_WriteData(0x40);

  	LCD_Spi_WriteCommand(0x85); 	LCD_Spi_WriteData(0xFF);

  	LCD_Spi_WriteCommand(0x86); 	LCD_Spi_WriteData(0xFF);

  	LCD_Spi_WriteCommand(0x87); 	LCD_Spi_WriteData(0xFF);

  	LCD_Spi_WriteCommand(0x88); 	LCD_Spi_WriteData(0x0A);

  	LCD_Spi_WriteCommand(0x89); 	LCD_Spi_WriteData(0x21);

  	LCD_Spi_WriteCommand(0x8A); 	LCD_Spi_WriteData(0x00);

  	LCD_Spi_WriteCommand(0x8B); 	LCD_Spi_WriteData(0x80);
  	LCD_Spi_WriteCommand(0x8C); 	LCD_Spi_WriteData(0x01);
  	LCD_Spi_WriteCommand(0x8D); 	LCD_Spi_WriteData(0x01);
  	LCD_Spi_WriteCommand(0x8E); 	LCD_Spi_WriteData(0xFF);
  	LCD_Spi_WriteCommand(0x8F); 	LCD_Spi_WriteData(0xFF);

  	LCD_Spi_WriteCommand(0xB6); 	LCD_Spi_WriteData(0x00); 	LCD_Spi_WriteData(0x20);
  	LCD_Spi_WriteCommand(0x36); 	LCD_Spi_WriteData(0x08);   //Memory Access Control : Set as LCD_SCANDIR_VERTICAL screen
  	LCD_Spi_WriteCommand(0x3A); 	LCD_Spi_WriteData(0x05);   //Pixel Format Set

  	LCD_Spi_WriteCommand(0x90);
  	LCD_Spi_WriteData(0x08);
  	LCD_Spi_WriteData(0x08);
  	LCD_Spi_WriteData(0x08);
  	LCD_Spi_WriteData(0x08);

  	LCD_Spi_WriteCommand(0xBD);
  	LCD_Spi_WriteData(0x06);

  	LCD_Spi_WriteCommand(0xBC);
  	LCD_Spi_WriteData(0x00);

  	LCD_Spi_WriteCommand(0xFF);
  	LCD_Spi_WriteData(0x60);
  	LCD_Spi_WriteData(0x01);
  	LCD_Spi_WriteData(0x04);

  	LCD_Spi_WriteCommand(0xC3);  //Vreg1a Voltage Control
  	LCD_Spi_WriteData(0x13);
  	LCD_Spi_WriteCommand(0xC4);  //Vreg1b Voltage Control
  	LCD_Spi_WriteData(0x13);

  	LCD_Spi_WriteCommand(0xC9); //Vreg2a Voltage Control
  	LCD_Spi_WriteData(0x22);

  	LCD_Spi_WriteCommand(0xBE);
  	LCD_Spi_WriteData(0x11);

  	LCD_Spi_WriteCommand(0xE1);
  	LCD_Spi_WriteData(0x10);
  	LCD_Spi_WriteData(0x0E);

  	LCD_Spi_WriteCommand(0xDF);
  	LCD_Spi_WriteData(0x21);
  	LCD_Spi_WriteData(0x0c);
  	LCD_Spi_WriteData(0x02);

  	LCD_Spi_WriteCommand(0xF0);//SET_GAMMA1
  	LCD_Spi_WriteData(0x45);
  	LCD_Spi_WriteData(0x09);
  	LCD_Spi_WriteData(0x08);
  	LCD_Spi_WriteData(0x08);
  	LCD_Spi_WriteData(0x26);
   	LCD_Spi_WriteData(0x2A);

   	LCD_Spi_WriteCommand(0xF1);//SET_GAMMA2
   	LCD_Spi_WriteData(0x43);
   	LCD_Spi_WriteData(0x70);
   	LCD_Spi_WriteData(0x72);
   	LCD_Spi_WriteData(0x36);
   	LCD_Spi_WriteData(0x37);
   	LCD_Spi_WriteData(0x6F);


   	LCD_Spi_WriteCommand(0xF2);//SET_GAMMA3
   	LCD_Spi_WriteData(0x45);
   	LCD_Spi_WriteData(0x09);
   	LCD_Spi_WriteData(0x08);
   	LCD_Spi_WriteData(0x08);
   	LCD_Spi_WriteData(0x26);
   	LCD_Spi_WriteData(0x2A);

   	LCD_Spi_WriteCommand(0xF3);//SET_GAMMA4
   	LCD_Spi_WriteData(0x43);
   	LCD_Spi_WriteData(0x70);
   	LCD_Spi_WriteData(0x72);
   	LCD_Spi_WriteData(0x36);
   	LCD_Spi_WriteData(0x37);
   	LCD_Spi_WriteData(0x6F);

  	LCD_Spi_WriteCommand(0xED);
  	LCD_Spi_WriteData(0x1B);
  	LCD_Spi_WriteData(0x0B);

  	LCD_Spi_WriteCommand(0xAE);
  	LCD_Spi_WriteData(0x77);

  	LCD_Spi_WriteCommand(0xCD);
  	LCD_Spi_WriteData(0x63);


  	LCD_Spi_WriteCommand(0x70);
  	LCD_Spi_WriteData(0x07);
  	LCD_Spi_WriteData(0x07);
  	LCD_Spi_WriteData(0x04);
  	LCD_Spi_WriteData(0x0E);
  	LCD_Spi_WriteData(0x0F);
  	LCD_Spi_WriteData(0x09);
  	LCD_Spi_WriteData(0x07);
  	LCD_Spi_WriteData(0x08);
  	LCD_Spi_WriteData(0x03);

  	LCD_Spi_WriteCommand(0xE8);
  	LCD_Spi_WriteData(0x34);

  	LCD_Spi_WriteCommand(0x62);
  	LCD_Spi_WriteData(0x18);
  	LCD_Spi_WriteData(0x0D);
  	LCD_Spi_WriteData(0x71);
  	LCD_Spi_WriteData(0xED);
  	LCD_Spi_WriteData(0x70);
  	LCD_Spi_WriteData(0x70);
  	LCD_Spi_WriteData(0x18);
  	LCD_Spi_WriteData(0x0F);
  	LCD_Spi_WriteData(0x71);
  	LCD_Spi_WriteData(0xEF);
  	LCD_Spi_WriteData(0x70);
  	LCD_Spi_WriteData(0x70);

  	LCD_Spi_WriteCommand(0x63);
  	LCD_Spi_WriteData(0x18);
  	LCD_Spi_WriteData(0x11);
  	LCD_Spi_WriteData(0x71);
  	LCD_Spi_WriteData(0xF1);
  	LCD_Spi_WriteData(0x70);
  	LCD_Spi_WriteData(0x70);
  	LCD_Spi_WriteData(0x18);
  	LCD_Spi_WriteData(0x13);
  	LCD_Spi_WriteData(0x71);
  	LCD_Spi_WriteData(0xF3);
  	LCD_Spi_WriteData(0x70);
  	LCD_Spi_WriteData(0x70);

  	LCD_Spi_WriteCommand(0x64);
  	LCD_Spi_WriteData(0x28);
  	LCD_Spi_WriteData(0x29);
  	LCD_Spi_WriteData(0xF1);
  	LCD_Spi_WriteData(0x01);
  	LCD_Spi_WriteData(0xF1);
  	LCD_Spi_WriteData(0x00);
  	LCD_Spi_WriteData(0x07);

  	LCD_Spi_WriteCommand(0x66);
  	LCD_Spi_WriteData(0x3C);
  	LCD_Spi_WriteData(0x00);
  	LCD_Spi_WriteData(0xCD);
  	LCD_Spi_WriteData(0x67);
  	LCD_Spi_WriteData(0x45);
  	LCD_Spi_WriteData(0x45);
  	LCD_Spi_WriteData(0x10);
  	LCD_Spi_WriteData(0x00);
  	LCD_Spi_WriteData(0x00);
  	LCD_Spi_WriteData(0x00);

  	LCD_Spi_WriteCommand(0x67);
  	LCD_Spi_WriteData(0x00);
  	LCD_Spi_WriteData(0x3C);
  	LCD_Spi_WriteData(0x00);
  	LCD_Spi_WriteData(0x00);
  	LCD_Spi_WriteData(0x00);
  	LCD_Spi_WriteData(0x01);
  	LCD_Spi_WriteData(0x54);
  	LCD_Spi_WriteData(0x10);
  	LCD_Spi_WriteData(0x32);
  	LCD_Spi_WriteData(0x98);

  	LCD_Spi_WriteCommand(0x74);
  	LCD_Spi_WriteData(0x10);
  	LCD_Spi_WriteData(0x85);
  	LCD_Spi_WriteData(0x80);
  	LCD_Spi_WriteData(0x00);
  	LCD_Spi_WriteData(0x00);
  	LCD_Spi_WriteData(0x4E);
  	LCD_Spi_WriteData(0x00);

    LCD_Spi_WriteCommand(0x98); 	LCD_Spi_WriteData(0x3e);  	LCD_Spi_WriteData(0x07); //

  	LCD_Spi_WriteCommand(0x35); //Tearing Effect ON
  	LCD_Spi_WriteCommand(0x21); //DISPLAY_INVERSION_ON

  	LCD_Spi_WriteCommand(0x11); //SLEEP_OUT
  	delayms(120);
  	LCD_Spi_WriteCommand(0x29); //DISPLAY_ON
  	delayms(20);
}
//========== apps ===

void GC9A01_ShowDemo()
{
	unsigned char i;

	GFX_setTextColor	(
			COLOR_WHITE, //text color
			COLOR_BLACK); //background color
	GFX_setTextSize(1);//2);//1);
	GFX_setTextWrap(0);
	GFX_setCursor(LCD_TFTWIDTH/2, LCD_TFTHEIGHT/2);
	LCD_writeCharAtCursor('a');
	delayms(1000);

	LCD_writeCharAtCursor('b');
	delayms(500);

	LCD_fillScreen(COLOR_BLACK);//LCD_fillScreen(COLOR_RED);
	delayms(500);

	LCD_drawFastHLine(LCD_TFTWIDTH/2,LCD_TFTHEIGHT/2,100,COLOR_GREEN);
	delayms(500);

	LCD_drawFastVLine(LCD_TFTWIDTH/2,LCD_TFTHEIGHT/2,100,0x07E0);
	delayms(500);

	LCD_fillRect(LCD_TFTWIDTH/2,LCD_TFTHEIGHT/2,20,20,COLOR_BLUE);
	delayms(500);

	for(i=0;i<5;i++){
		LCD_drawCircle(LCD_TFTWIDTH/2+(i*10),LCD_TFTHEIGHT/2,6,COLOR_GREEN);//LCD_drawCircle(40+(i*5),80+(i*5),10,COLOR_GREEN);
		delayms(400);
		LCD_fillCircle(LCD_TFTWIDTH/2+(i*10),LCD_TFTHEIGHT/2,4,COLOR_YELLOW); //LCD_fillCircle(40+(i*5),80+(i*5),8,COLOR_RED);
	}

	LCD_drawRect(50, 120, 40, 20, COLOR_YELLOW);
	delayms(500);
	LCD_fillRect(51, 121, 38, 18, COLOR_CYAN);
	delayms(500);

    LCD_fillScreen(COLOR_BLACK); //Clear Screen with Black
    LCD_fillScreen(COLOR_WHITE);
    LCD_fillScreen(COLOR_BLACK);
    //Draw a pixel at center (120,120)
    for(i=0;i<10;i++){
    	LCD_drawPixel(LCD_TFTWIDTH/2 + i,LCD_TFTHEIGHT/2, COLOR_RED);
    	delayms(50);
    }

    LCD_fillScreen(COLOR_BLACK);

	//LCD_displayBitmap(gImage_lcd);
	//delayms(1000);
}

//================================================================
/*
// Initialization for GC9A01 screens)
void GC9A01_Config(unsigned char whichSPI, unsigned char orientation) {

	//Set the resolution and scanning method of the screen
	//GC9A01_SetOrientation(orientation); //LCD_SCANDIR_VERTICAL
	//Set the resolution and scanning method of the screen
	GC9A01_SetOrientation(orientation); //LCD_SCANDIR_VERTICAL
}
*/
void stmSpiGC9A01RoundLcdLoop(){
	unsigned char i;
	char buf[100];
	int retVal;
	int error;
	short acc[3]; short gyro[3]; float f_degree;


	  if(!g_bI2CModuleConfigDone){
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
		  gI2Cx = I2C2;
#else
		  gI2Cx = I2C1;
#endif
		printf("I2C Init...");
		stm_I2C_Init(gI2Cx,400000);//400Kbps
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	  }
	    stmMPU6050_6DOF_Init();


    printf("stmGC9A01(Round1.3) 240x240 SPI LCD Test");

	LCD_spi_GpioSetup();//whichSPI);

	GC9A01_InitSeq();//initialize a set of registers

	GC9A01_SetOrientation(LCD_SCANDIR_HORIZONTAL);//LCD_SCANDIR_VERTICAL);//LCD_SCANDIR_VERTICAL = Default

    LCD_fillScreen(COLOR_BLACK);
    GFX_setCursor(0,0);

    LCD_displayHello("GC9A01 DEMO");

    GC9A01_ShowDemo();



#if 0
	//LCD_Gauge_Config_ImgWithPalette(VUmeter20, LCD_TFTWIDTH/2, LCD_TFTHEIGHT*65/100);
    GFX_setCursor(0,0);
	GFX_setTextColor(COLOR_WHITE, //text color
					COLOR_BLACK); //background color
	GFX_setTextSize(1);
	LCD_displayString(LCD_TFTWIDTH/3,LCD_TFTHEIGHT/2+50,"GPWS");
	LCD_displayString(LCD_TFTWIDTH/4,LCD_TFTHEIGHT/2+70,"Proximity : ");

	while(1){
		for(i=0;i<256;i++){
			LCD_displayValue(LCD_TFTWIDTH/4+50,LCD_TFTHEIGHT/2+70, i);
			delayms(1000);
		}
	}
#else
	LCD_Gauge_Config_ImgWithPalette(VUmeter20, LCD_IMGWIDTH/2, LCD_IMGHEIGHT*65/100);

	LCD_gauge.ms = millis();
	while(1){
		stmMPU6050_6DOF_GetData(acc, gyro, &f_degree);

		printf("[accel] x,y,z=%d,\t%d,\t%d ", acc[0], acc[1],acc[2]);
		printf("\t[Gyro] x,y,z =%d \t%d \t%d ", gyro[0],gyro[1],gyro[2]);
		printf("\t[Temp]=%s Degree\r\n",float2str(f_degree));


#if 0
		if( LCDapp_gaugeGetValue(&retVal) == ERR_SUCCESS)			LCD_gaugeUpdate(retVal);
#else
		LCD_gaugeUpdate(acc[1]/1000);

		//show digital time at LCD corner
		snprintf(buf,20,"ACC:(%03d,%03d,%03d)",acc[0], acc[1],acc[2]);
		GFX_setTextColor(COLOR_WHITE,COLOR_BLACK);
		LCD_displayString(0,200,buf);
		snprintf(buf,20,"GYR:(%03d,%03d,%03d)",gyro[0],gyro[1],gyro[2]);
		LCD_displayString(0,220,buf);
		delayms(100);
#endif

	}
#endif
}

#endif

/*
void GC9A01_setRotation(unsigned char m) {
	unsigned char rotation; //YOON

  LCD_Spi_WriteCommand(GC9A01_MADCTL);

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

//------------------------ Graphic Functions ---------------------------------------

/*
//function: Draw a point
//== LCD_drawPixel
void GC9A01_DrawPaint(unsigned short x, unsigned short y, unsigned short Color)
{
	LCD_setWindows(x,y,x,y);
	LCD_Spi_WriteData16(Color); //GC9A01_SendData_16Bit(Color);
}


void LCD_drawFastVLine(short x, short y, short h, unsigned short color) {

  // Rudimentary clipping
  if((x >= LCD_gfx._width) || (y >= LCD_gfx._height)) return;
  if((y+h-1) >= LCD_gfx._height) h = LCD_gfx._height-y;
  LCD_setWindows(x, y, x, y+h-1);

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
  LCD_setWindows(x, y, x+w-1, y);

  unsigned char hi = color >> 8, lo = color;
  LCD_RS_HIGH;	//RS=1;
  LCD_CS0_LOW;	//~CS

  while (w--) {
	  stmSpiWrByte(hi);
	  stmSpiWrByte(lo);
  }
  LCD_CS0_HIGH;	//CS=1
}

// fill a rectangle
void LCD_fillRect(short x, short y, short w, short h,  unsigned short color)
{
  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= LCD_gfx._width) || (y >= LCD_gfx._height)) return;
  if((x + w - 1) >= LCD_gfx._width)  w = LCD_gfx._width  - x;
  if((y + h - 1) >= LCD_gfx._height) h = LCD_gfx._height - y;

  LCD_setWindows(x, y, x+w-1, y+h-1);

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
*/
/*
// fill a rectangle
void LCD_fillRect(unsigned char x0, unsigned char y0, unsigned char w, unsigned char h,unsigned short color) {

	LCD_setWindows(x0, y0, x0+w-1, y0+h-1);

	LCD_RS_HIGH;	//RS=1;
	LCD_CS0_LOW;	//~CS
	for(x0=0;x0<2;x0++){
		for(y0=0;y0<h;y0++)
			stmSpiWrByte(color >> 8);	  stmSpiWrByte(color);
	}
	LCD_CS0_HIGH;	//CS

}



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

	  printf("LCD_drawChar():Invalid \r\n");
	  return;
    }
    printf("LCD_drawChar(%c)@(%u,%u)\r\n",c, x,y);
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
//Clear screen
void LCD_fillScreen(unsigned short Color)
{
    unsigned short i,j;

    LCD_setWindows(0, 0, LCD_TFTWIDTH-1, LCD_TFTHEIGHT-1);

    LCD_RS_HIGH; //Data
	for(i = 0; i < LCD_TFTWIDTH; i++){
		for(j = 0; j < LCD_TFTHEIGHT; j++){
			LCD_Spi_WriteData(Color>>8);
			LCD_Spi_WriteData(Color);
		}
	 }
}

void  LCD_displayString(unsigned char x0,unsigned char y0,unsigned char *s)
{
	int i,j,k,x,y,xx;
	unsigned char qm;
	long int ulOffset;
	char  ywbuf[32],temp[2];

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

//or...Full Size (240*240)
void GC9A01_Display(unsigned char *Image)
{
    unsigned short i,j;
    LCD_setWindows(0, 0, LCD_TFTWIDTH-1, LCD_TFTHEIGHT-1);
    LCD_RS_HIGH;
    for(i = 0; i < LCD_TFTWIDTH; i++){
      for(j = 0; j < LCD_TFTHEIGHT; j++){
    	  LCD_Spi_WriteData((*Image+i*LCD_TFTWIDTH+j)>>8);//DEV_SPI_WRITE((*Image+i*LCD_TFTWIDTH+j)>>8);
    	  LCD_Spi_WriteData(*(Image+i*LCD_TFTWIDTH+j));//DEV_SPI_WRITE(*(Image+i*LCD_TFTWIDTH+j));
      }
    }
}
*/
