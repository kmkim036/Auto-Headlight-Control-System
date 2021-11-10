/***************************************************
  This is a library for the IPS240x240-1.3" SPI display with ST7789 controller.
  Use SPI to communicate.
  *RST is optional
  - Just for STM32F103RCT6
  - For image, use Image2LCD program.
  - ST7739 is suited for 240x320.
  - But we have 240x240
  - SPI Mode3

  - REF : githyb.com/Floyd-Fish/ST7789-STM32/blob/master/ST7789
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



extern unsigned char g_bI2CModuleConfigDone;
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;

int g_vals = 0;
extern struct _LCD_gfx LCD_gfx;
extern struct _LCD_clock LCD_gauge;
//return 0 if changed.
char LCDapp_gaugeGetValue(int *retVal){
	  if((millis() - LCD_gauge.ms) < 50)
		  return -1;

	  LCD_gauge.ms = millis();

	  g_vals++;
	  if(g_vals > 20){
		  g_vals = -20;
	  }
	  LCD_gauge.ss=g_vals;
	  *retVal =  g_vals;

	  return ERR_SUCCESS;
}

void LCD_gaugeUpdate(int val)
{


  if(val <0){
	  LCD_gauge.sDeg = val*3 + 360; //3 degree per sec
  }else
	  LCD_gauge.sDeg = val*3; //3 degree per sec

  //(a) draw the original background image in the specified Window Region without color arguments
  LCD_drawGaugeHandS(LCD_gauge.sDegOld, LCD_gauge.style,LCD_gauge.sHandWidth,LCD_gauge.sHandLength, 0,0); //draw the old saved handle first
  //(b) draw Line in the specified Window Region with color
  LCD_drawGaugeHandS(LCD_gauge.sDeg,    LCD_gauge.style,LCD_gauge.sHandWidth,LCD_gauge.sHandLength,LCD_gauge.sHandColor1,LCD_gauge.sHandColor2);

  LCD_gauge.sDegOld = LCD_gauge.sDeg;

  //draw needle center circle
  LCD_fillCircle(LCD_gauge.cx, LCD_gauge.cy, 8, gfx_RgbTo565(0,0,0));//250,250,0));//40,40,40));  //lcd.fillCircle(cx,cy, cirSize, cirColor);
}

//==============================================================
#if (LCDCONT_IS == LCDCONT_IS_ST7789) && (LCDAPP_IS == LCDAPP_IS_GAUGE)
void stmSpiST7789_240X240_Gauge_LcdLoop()
{
	int retVal;
	char buf[80];
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

    LCD_spi_GpioSetup();
    ST7789_InitSeq();
    LCD_Gauge_Config_ImgWithPalette(VUmeter20, LCD_TFTWIDTH/2, LCD_TFTHEIGHT*65/100);//aviGauge;//clockface8;);
	LCD_gauge.ms = millis();

	while(1){

		stmMPU6050_6DOF_GetData(acc, gyro, &f_degree);

		printf("[accel] x,y,z=%d,\t%d,\t%d ", acc[0], acc[1],acc[2]);
		printf("\t[Gyro] x,y,z =%d \t%d \t%d ", gyro[0],gyro[1],gyro[2]);
		printf("\t[Temp]=%s Degree\r\n",float2str(f_degree));

		//if( LCDapp_gaugeGetValue(&retVal) == ERR_SUCCESS)


		LCD_gaugeUpdate(acc[1]/1000);

		//show digital time at LCD corner
		snprintf(buf,20,"ACC:(%03d,%03d,%03d)",acc[0], acc[1],acc[2]);
		GFX_setTextColor(COLOR_WHITE,COLOR_BLACK);
		LCD_displayString(0,200,buf);
		snprintf(buf,20,"GYR:(%03d,%03d,%03d)",gyro[0],gyro[1],gyro[2]);
		LCD_displayString(0,220,buf);


		delayms(100);
	}
}

/* -------------------- BUtton Setup -----------------
#define BUTTON PB9
int stateOld = HIGH;
long btDebounce    = 30;
long btDoubleClick = 600;
long btLongClick   = 700;
long btLongerClick = 2000;
long btTime = 0, btTime2 = 0;
int clickCnt = 1;

// 0=idle, 1,2,3=click, -1,-2=longclick
int checkButton()
{
  int state = digitalRead(BUTTON);
  if( state == LOW && stateOld == HIGH ) { btTime = millis(); stateOld = state; return 0; } // button just pressed
  if( state == HIGH && stateOld == LOW ) { // button just released
    stateOld = state;
    if( millis()-btTime >= btDebounce && millis()-btTime < btLongClick ) {
      if( millis()-btTime2<btDoubleClick ) clickCnt++; else clickCnt=1;
      btTime2 = millis();
      return clickCnt;
    }
  }
  if( state == LOW && millis()-btTime >= btLongerClick ) { stateOld = state; return -2; }
  if( state == LOW && millis()-btTime >= btLongClick ) { stateOld = state; return -1; }
  return 0;
}
*/
#endif



//=== added =========





//========== handle image ======================

/*
void ST7789_displayBitmap(){
	int i=0;
	//for(i=0;i<4;i++){
		ST7789_drawImageRectArea565(0+i*10, 8+i*10, 160,80, gImage_lcd);//128,  ST7789_RED);///
	//}
}
*/


//====== for clock/gauges ====
/*
// ------------------------------------------------
// hour and minute hands
void ST7789_drawClockHand(int deg, int style, int w, int l, int color1, int color2)
{
  int i,num = 4;
  LCD_gauge.cirColor = COLOR_YELLOW;
  LCD_gauge.cirSize = 1;
  switch(style) {
    default:
    case 0:  // tris 013, 230, 024, rect with triangle
    	LCD_gauge.px[0]=-w, LCD_gauge.py[0]= l-5;
    	LCD_gauge.px[1]=-w, LCD_gauge.py[1]=-10;
    	LCD_gauge.px[2]= w, LCD_gauge.py[2]= l-5;
    	LCD_gauge.px[3]= w, LCD_gauge.py[3]=-10;
    	LCD_gauge.px[4]= 0, LCD_gauge.py[4]= l-5+7;
      num = 5;
      break;
    case 1:  // tris 013,023, peak style
    	LCD_gauge.px[0]= 0, LCD_gauge.py[0]= l;
    	LCD_gauge.px[1]=-w-1, LCD_gauge.py[1]= 0;
    	LCD_gauge.px[2]= w+1, LCD_gauge.py[2]= 0;
    	LCD_gauge.px[3]= 0, LCD_gauge.py[3]=-15;
      break;
    case 2:  // tris 013, 230, rect
    	LCD_gauge.px[0]=-w, LCD_gauge.py[0]= l;
    	LCD_gauge.px[1]=-w, LCD_gauge.py[1]=-10;
    	LCD_gauge.px[2]= w, LCD_gauge.py[2]= l;
    	LCD_gauge.px[3]= w, LCD_gauge.py[3]=-12;
      break;
    case 3:  // tris 013, 230, 024, rect with peak
    	LCD_gauge.px[0]=-w-1, LCD_gauge.py[0]= l-15;
    	LCD_gauge.px[1]=-w+1, LCD_gauge.py[1]=-5;
    	LCD_gauge.px[2]= w+1, LCD_gauge.py[2]= l-15;
    	LCD_gauge.px[3]= w-1, LCD_gauge.py[3]=-5;
    	LCD_gauge.px[4]= 0, LCD_gauge.py[4]= l-15+17;
      num = 5;
      LCD_gauge.cirColor = COLOR_RED;
      LCD_gauge.cirSize = 3;
      break;
  }
  int x[5],y[5];
  int cc = fastCos(deg+180);
  int ss = fastSin(deg+180);
  for(i=0;i<num;i++) {
    x[i] = LCD_gauge.px[i]*cc - LCD_gauge.py[i]*ss;
    y[i] = LCD_gauge.px[i]*ss + LCD_gauge.py[i]*cc;
    x[i] = LCD_gauge.cx + (x[i]+(x[i]>0 ? MAXSIN/2:-MAXSIN/2))/MAXSIN;
    y[i] = LCD_gauge.cy + (y[i]+(y[i]>0 ? MAXSIN/2:-MAXSIN/2))/MAXSIN;
  }

  ST7789_drawTriangleArea(
		  x[0],y[0],
		  x[1],y[1],
		  x[3],y[3],
		  color2);
  ST7789_drawTriangleArea(
		  x[2],y[2],
		  x[3],y[3],
		  x[0],y[0],
		  color1);
}

//second hand
void ST7789_drawClockHandS(int deg, int style, int w, int l,
		int color1, int color2) //0,0 if the orginal background image.
{
  int i,num = 8;
  int x[8],y[8];
  int cc = fastCos(deg+180);
  int ss = fastSin(deg+180);

  LCD_gauge.cirColor = COLOR_YELLOW;
  LCD_gauge.cirSize = 3;
  LCD_gauge.px[0]=-w+3, LCD_gauge.py[0]= l;
  LCD_gauge.px[1]=-w+3, LCD_gauge.py[1]=-20;
  LCD_gauge.px[2]= w-3, LCD_gauge.py[2]= l;
  LCD_gauge.px[3]= w-3, LCD_gauge.py[3]=-20;

  //we modify for small tail
  LCD_gauge.px[4]=-w+1, LCD_gauge.py[4]=-20;//-40;
  LCD_gauge.px[5]=-w+1, LCD_gauge.py[5]=-5;//-15;
  LCD_gauge.px[6]= w-1, LCD_gauge.py[6]=-20;//-40;
  LCD_gauge.px[7]= w-1, LCD_gauge.py[7]=-5;//-15;

  for(i=0;i<num;i++) {
    x[i] = LCD_gauge.px[i]*cc - LCD_gauge.py[i]*ss;
    y[i] = LCD_gauge.px[i]*ss + LCD_gauge.py[i]*cc;
    x[i] = LCD_gauge.cx + (x[i]+(x[i]>0?MAXSIN/2:-MAXSIN/2))/MAXSIN;
    y[i] = LCD_gauge.cy + (y[i]+(y[i]>0?MAXSIN/2:-MAXSIN/2))/MAXSIN;
  }
  ST7789_drawTriangleArea(
		  x[0],y[0],
		  x[1],y[1],
		  x[3],y[3],
		  color1);
  ST7789_drawTriangleArea(
		  x[2],y[2],
		  x[3],y[3],
		  x[0],y[0],
		  color1);
// tail...
  ST7789_drawTriangleArea(
		  x[4],y[4],
		  x[5],y[5],
		  x[7],y[7],
		  color1);
  ST7789_drawTriangleArea(
		  x[6],y[6],
		  x[7],y[7],
		  x[4],y[4],
		  color1);
}

void ST7789_nextHandStyle()
{
  if(millis()-LCD_gauge.styleTime<15000) return;
  LCD_gauge.styleTime = millis();
  ST7789_drawClockHand(LCD_gauge.hDegOld,LCD_gauge.style,LCD_gauge.hHandWidth,LCD_gauge.hHandLength, 0, 0);
  ST7789_drawClockHand(LCD_gauge.mDegOld,LCD_gauge.style,LCD_gauge.mHandWidth,LCD_gauge.mHandLength, 0, 0);
  ST7789_drawClockHandS(LCD_gauge.sDegOld,LCD_gauge.style,LCD_gauge.sHandWidth,LCD_gauge.sHandLength, 0,0);
  if(++LCD_gauge.style>3) LCD_gauge.style=0;
  LCD_gauge.start = 1;
}

//======== Applications =============================================


void ST7789_displayHello()
{
	unsigned char *img;
	printf("Hello\r\n");
	LCD_displayString(0,0,"ST7789 for STM");

	img = (unsigned char*)LCD_gauge.clockface
			               +16*2+6;


    ST7789_drawImageRectArea565(0, 0, 240,180, img);//,  ST7789_RED);///
}

void ST7789_ShowDemo()
{
	unsigned char i;

    ST7789_displayHello();

	for(i=0;i<5;i++){
		//ST7789_drawCircle(40+(i*10),100,6,COLOR_GREEN);//ST7789_drawCircle(40+(i*5),80+(i*5),10,ST7789_GREEN);
		delayms(400);
		ST7789_fillCircle(40+(i*15),100,4,COLOR_YELLOW); //ST7789_fillCircle(40+(i*5),80+(i*5),8,ST7789_RED);
	}

	ST7789_drawRect(50, 120, 40, 20, COLOR_YELLOW);
	ST7789_fillRect(51, 121, 38, 18, COLOR_CYAN);
}

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void ST7789_InitSeq() {

  unsigned char  numCommands, numArgs, savedNumCmds;
  long ms;

	LCD_gfx.colstart  = LCD_gfx.rowstart = 0; // May be overridden in init func

    LCD_gfx.rowstart = (240 - LCD_TFTHEIGHT);
    LCD_gfx.rowstart2 = 0;
    LCD_gfx.colstart = LCD_gfx.colstart2 = (240 - LCD_TFTWIDTH);

	LCD_Spi_WriteCommand(ST7789_SWRESET); delayms(120); //  1: Software reset, no args, w/delay

	LCD_Spi_WriteCommand(ST7789_COLMOD); LCD_Spi_WriteData(0x55); delayms(10); //  3: Set color mode, 16-bit color

	//Porch control
	LCD_Spi_WriteCommand(0xB2); LCD_Spi_WriteData(0x0C); LCD_Spi_WriteData(0x0C);LCD_Spi_WriteData(0x00);LCD_Spi_WriteData(0x33);LCD_Spi_WriteData(0x33);

	//Display Rotation (2)
	ST7789_setRotation(LCD_ROTATION); //MADCTL

	//Internal LCD Voltage Generator Setting
	LCD_Spi_WriteCommand(0xB7);	LCD_Spi_WriteData(0x35);//Gate Control
	LCD_Spi_WriteCommand(0xBB);	LCD_Spi_WriteData(0x19);//VCOM Setting 0.725V
	LCD_Spi_WriteCommand(0xC0);	LCD_Spi_WriteData(0x2C);//LCMCTRL
	LCD_Spi_WriteCommand(0xC2);	LCD_Spi_WriteData(0x01);//VDV/VRH
	LCD_Spi_WriteCommand(0xC3);	LCD_Spi_WriteData(0x12);//VRH set +-4.45V
	LCD_Spi_WriteCommand(0xC4);	LCD_Spi_WriteData(0x21);//VDV Set default
	LCD_Spi_WriteCommand(0xC6);	LCD_Spi_WriteData(0x0F);//FrameRate Control in Normal 60Hz
	LCD_Spi_WriteCommand(0xD0);	LCD_Spi_WriteData(0xA4); LCD_Spi_WriteData(0xA1);//Power Control Default value

	delayms(10);

	//Gamma
	LCD_Spi_WriteCommand(0xE0);  LCD_Spi_WriteData(0xD0);LCD_Spi_WriteData(0x04);LCD_Spi_WriteData(0x0D);LCD_Spi_WriteData(0x11);
								LCD_Spi_WriteData(0x13);LCD_Spi_WriteData(0x2C);LCD_Spi_WriteData(0x3F);LCD_Spi_WriteData(0x44);
								LCD_Spi_WriteData(0x51);LCD_Spi_WriteData(0x2F);LCD_Spi_WriteData(0x1F);LCD_Spi_WriteData(0x1F);
								LCD_Spi_WriteData(0x20);LCD_Spi_WriteData(0x23);
	LCD_Spi_WriteCommand(0xE1);  LCD_Spi_WriteData(0xD0);LCD_Spi_WriteData(0x04);LCD_Spi_WriteData(0x0C);LCD_Spi_WriteData(0x11);
								LCD_Spi_WriteData(0x13);LCD_Spi_WriteData(0x2C);LCD_Spi_WriteData(0x3F);LCD_Spi_WriteData(0x44);
								LCD_Spi_WriteData(0x51);LCD_Spi_WriteData(0x2F);LCD_Spi_WriteData(0x1F);LCD_Spi_WriteData(0x1F);
								LCD_Spi_WriteData(0x20);LCD_Spi_WriteData(0x23);

    //Inversion ON
	LCD_Spi_WriteCommand(ST7789_INVON);

	LCD_Spi_WriteCommand(ST7789_SLPOUT); //sleep out
    LCD_Spi_WriteCommand(ST7789_NORON); delayms(50); //8: Normal display on, no args, w/delay
    LCD_Spi_WriteCommand(ST7789_DISPON);delayms(50); // 9: Main screen turn on, no args, delay

    ST7789_fillScreen(COLOR_BLACK);
}

*/
#if 0
void stmSpiST7789_240X240_Gauge_LcdLoop()
{
	int retVal;

    LCD_spi_GpioSetup();

    ST7789_InitSeq();

    //ST7789_ShowDemo();
	LCD_Gauge_Config_ImgWithPalette(VUmeter20, LCD_TFTWIDTH/2, LCD_TFTHEIGHT*65/100);//aviGauge;//clockface8;);

	LCD_gauge.ms = millis();

	while(1){
		if( LCDapp_gaugeGetValue(&retVal) == ERR_SUCCESS)
			LCD_gaugeUpdate(retVal);
	}
}
#endif
