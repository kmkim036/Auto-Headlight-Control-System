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

extern struct _LCD_gfx LCD_gfx;
extern struct _LCD_clock LCD_clk;
extern const unsigned char face8[];

void LCD_Clock_Config_Img(unsigned char *pImg, int cx, int cy)
//Clock Image Config
{
	char curTimeStr[]="12:00:00";
	LCD_clk.pFaceImg = 	pImg;
	LCD_clk.palette = (unsigned short*)LCD_clk.pFaceImg+3;
	LCD_clk.style = 1;
	LCD_clk.start = 1;
	LCD_clk.setMode = 0;
	LCD_clk.cirColor = COLOR_YELLOW;
	LCD_clk.cirSize = 3;

	LCD_clk.cx = cx;	LCD_clk.cy = cy;  // clock center

	LCD_clk.hHandLength = 25*2; LCD_clk.hHandWidth = 3*2;
	LCD_clk.mHandLength = 36*2; LCD_clk.mHandWidth = 3*2;
	LCD_clk.sHandLength = 44*2; LCD_clk.sHandWidth = 2*2;
	LCD_clk.hh = txt2num(curTimeStr+0); //txt2num(__TIME__+0); //__TIME__ == system clock
	LCD_clk.mm = txt2num(curTimeStr+3);
	LCD_clk.ss = txt2num(curTimeStr+6);
	LCD_clk.palette = (unsigned short*)LCD_clk.pFaceImg+3;

	LCD_clk.selHandColor1 = gfx_RgbTo565(250,250,0); //YELLO
	LCD_clk.selHandColor2 = gfx_RgbTo565(180,180,0); //dark YELLO
	LCD_clk.mHandColor1 = gfx_RgbTo565(220,220,220); //grey
	LCD_clk.mHandColor2 = gfx_RgbTo565(150,150,150); //dark grey
	LCD_clk.hHandColor1 = gfx_RgbTo565(220,220,220); //grey
	LCD_clk.hHandColor2 = gfx_RgbTo565(150,150,150); //dark grey
	//unsigned short hHandColor = gfx_RgbTo565(40,40,40);
	//unsigned short mHandColor = gfx_RgbTo565(80,80,80);
	//unsigned short hHandColor = gfx_RgbTo565(110,110,110);
	//unsigned short mHandColor = gfx_RgbTo565(180,180,180);
	//unsigned short sHandColor = RED;
	LCD_clk.sHandColor1 = gfx_RgbTo565(250,80,80); //Dark Pink
	LCD_clk.sHandColor2 = gfx_RgbTo565(200,0,0);   //Red
    LCD_drawFaceWithPalette(LCD_clk.pFaceImg);
}

void LCD_Clock_Update()
{
	char buf[80];

  if((millis() - LCD_clk.ms) < 1000 && !LCD_clk.start)
	  return;
#if 1
  LCD_clk.ms = millis();
  g_elpased_sec = LCD_clk.ms/1000;

  if(LCD_clk.setMode==0) {

	LCD_clk.curTime.second =  g_elpased_sec % 60;
	if(LCD_clk.curTime.second == 0){
		LCD_clk.curTime.minute++;
	}
	if(LCD_clk.curTime.minute == 60){
		LCD_clk.curTime.minute = 0;
		LCD_clk.curTime.hour++;
		if(LCD_clk.curTime.hour==12){
			LCD_clk.curTime.hour=0;
		}
	}

	LCD_clk.hh=LCD_clk.curTime.hour;
	LCD_clk.mm=LCD_clk.curTime.minute;
	LCD_clk.ss=LCD_clk.curTime.second;
  }
#endif

  LCD_clk.sDeg = LCD_clk.ss*6; //6 degree per sec

  if(LCD_clk.ss==0 || LCD_clk.start) { //at 12 clock position.
	  LCD_clk.start = 0;
	  LCD_clk.mDeg = LCD_clk.mm*6+LCD_clk.sDeg/60; //6 degree per min + sec portion
	  LCD_clk.hDeg = LCD_clk.hh*30+LCD_clk.mDeg/12; //30 degree per hour + min portion

	  LCD_drawClockHand(LCD_clk.hDegOld, LCD_clk.style, LCD_clk.hHandWidth, LCD_clk.hHandLength, 0,0);
	  LCD_drawClockHand(LCD_clk.mDegOld, LCD_clk.style, LCD_clk.mHandWidth, LCD_clk.mHandLength, 0,0);

	  LCD_clk.mDegOld = LCD_clk.mDeg; //update
	  LCD_clk.hDegOld = LCD_clk.hDeg; //update
  }

  LCD_drawClockHandS(LCD_clk.sDegOld,LCD_clk.style,LCD_clk.sHandWidth,LCD_clk.sHandLength, 0,0);

  if(LCD_clk.setMode==1) //@time setting
    LCD_drawClockHand(LCD_clk.hDeg,LCD_clk.style,LCD_clk.hHandWidth,LCD_clk.hHandLength,LCD_clk.selHandColor1,LCD_clk.selHandColor2);
  else //@normal
    LCD_drawClockHand(LCD_clk.hDeg,LCD_clk.style,LCD_clk.hHandWidth,LCD_clk.hHandLength,LCD_clk.hHandColor1,LCD_clk.hHandColor2);

  if(LCD_clk.setMode==2)
    LCD_drawClockHand(LCD_clk.mDeg,LCD_clk.style,LCD_clk.mHandWidth,LCD_clk.mHandLength,LCD_clk.selHandColor1,LCD_clk.selHandColor2);
  else
    LCD_drawClockHand(LCD_clk.mDeg,LCD_clk.style,LCD_clk.mHandWidth,LCD_clk.mHandLength,LCD_clk.mHandColor1,LCD_clk.mHandColor2);

  if(LCD_clk.setMode==0)
    LCD_drawClockHandS(LCD_clk.sDeg,LCD_clk.style,LCD_clk.sHandWidth,LCD_clk.sHandLength,LCD_clk.sHandColor1,LCD_clk.sHandColor2);

  LCD_clk.sDegOld = LCD_clk.sDeg;

  LCD_fillCircle(LCD_clk.cx, LCD_clk.cy, 4, gfx_RgbTo565(40,40,40));  //lcd.fillCircle(cx,cy, cirSize, cirColor);

  snprintf(buf,20,"%02d:%02d:%02d",LCD_clk.hh,LCD_clk.mm,LCD_clk.ss);
  GFX_setTextColor(COLOR_WHITE,COLOR_BLACK);
  LCD_displayString(0,0,buf);

}
#if (LCDCONT_IS == LCDCONT_IS_ST7789) || (LCDCONT_IS == LCDCONT_IS_ST7735)
void LCD_ST77XX_setRotation(unsigned char m) {
	unsigned char rotation; //YOON

	rotation = m % 4; // can't be higher than 3

  LCD_Spi_WriteCommand(ST77XX_MADCTL);
  switch (rotation) {
   case 0: //180
	   LCD_Spi_WriteData(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
	   LCD_gfx._width  = LCD_TFTWIDTH;
	   LCD_gfx._height = LCD_TFTHEIGHT;
    break;
   case 1: //90
	   LCD_Spi_WriteData(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
	   LCD_gfx._width  = LCD_TFTHEIGHT;
	   LCD_gfx._height = LCD_TFTWIDTH;
    break;
   case 2: //normal
	   LCD_Spi_WriteData(MADCTL_RGB);
	   LCD_gfx._width  = LCD_TFTWIDTH;
	   LCD_gfx._height = LCD_TFTHEIGHT;
    break;
   case 3: //270
	   LCD_Spi_WriteData(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
	   LCD_gfx._width  = LCD_TFTHEIGHT;
	   LCD_gfx._height = LCD_TFTWIDTH;
    break;
  }
}
#endif

#if (LCDCONT_IS == LCDCONT_IS_ST7789)



void ST7789_setWindows(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1) {
	unsigned short x_start, y_start, x_end, y_end;

	x_start = x0 + X_SHIFT, x_end = x1 + X_SHIFT;
	y_start = y0 + Y_SHIFT, y_end = y1 + Y_SHIFT;

	LCD_Spi_WriteCommand(ST7789_CASET); // Column addr set
	LCD_Spi_WriteData(x_start >> 8);
	LCD_Spi_WriteData(x_start & 0xFF);     // XSTART
	LCD_Spi_WriteData(x_end >> 8);
	LCD_Spi_WriteData(x_end & 0xFF);     // XEND

	LCD_Spi_WriteCommand(ST7789_RASET);  // Row addr set
	LCD_Spi_WriteData(y_start >> 8);     //0x00);
	LCD_Spi_WriteData(y_start & 0xff);   // YSTART
	LCD_Spi_WriteData(y_end >> 8);       //0x00);
	LCD_Spi_WriteData(y_end & 0xff);     // YEND

	LCD_Spi_WriteCommand(ST7789_RAMWR); // write to RAM
}

//paint a pixel with given color @ current cursor position
void ST7789_pushColor(unsigned short color) {

	LCD_RS_HIGH; 	//RS=1;
	LCD_CS0_LOW; 	//~CS

	color = gfx_swapcolor(color);

	stmSpiWrByte(color >> 8);	stmSpiWrByte(color);

	LCD_CS0_HIGH; 	//CS
}

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
*/



//??? =========?????????????
void ST7789_DispSingleColor(unsigned short int color){
	unsigned char i,j;
	LCD_Spi_WriteCommand(0x2c);
	for(i=0;i<160; i++){
		for(j=0;j<128; j++){
			LCD_Spi_WriteData(color>>8);
			LCD_Spi_WriteData(color&0xff);
		}
	}
}

//=== from clock





//=========== handle character and string with the given font on LCD ==================




//============================================
//Init config commands lists
void ST7789_InitSeq() {

  unsigned char  numCommands, numArgs, savedNumCmds;
  long ms;

	LCD_gfx._width  = LCD_TFTWIDTH;
	LCD_gfx._height = LCD_TFTHEIGHT;

	LCD_gfx.colstart  = LCD_gfx.rowstart = 0; // May be overridden in init func

    LCD_gfx.rowstart = (320 - LCD_gfx._height);
    LCD_gfx.rowstart2 = 0;
    LCD_gfx.colstart = LCD_gfx.colstart2 = (240 - LCD_gfx._width);

	LCD_Spi_WriteCommand(ST7789_SWRESET); delayms(120); //  1: Software reset, no args, w/delay

	LCD_Spi_WriteCommand(ST7789_COLMOD); LCD_Spi_WriteData(0x55); delayms(10); //  3: Set color mode, 16-bit color

	//Porch control
	LCD_Spi_WriteCommand(0xB2); LCD_Spi_WriteData(0x0C); LCD_Spi_WriteData(0x0C);LCD_Spi_WriteData(0x00);LCD_Spi_WriteData(0x33);LCD_Spi_WriteData(0x33);

	//Display Rotation (2)
	LCD_ST77XX_setRotation(LCD_ROTATION); //MADCTL

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

	//Gamma?
	LCD_Spi_WriteCommand(0xE0);  LCD_Spi_WriteData(0xD0);LCD_Spi_WriteData(0x04);LCD_Spi_WriteData(0x0D);LCD_Spi_WriteData(0x11);
								LCD_Spi_WriteData(0x13);LCD_Spi_WriteData(0x2C);LCD_Spi_WriteData(0x3F);LCD_Spi_WriteData(0x44);
								LCD_Spi_WriteData(0x51);LCD_Spi_WriteData(0x2F);LCD_Spi_WriteData(0x1F);LCD_Spi_WriteData(0x1F);
								LCD_Spi_WriteData(0x20);LCD_Spi_WriteData(0x23);
	LCD_Spi_WriteCommand(0xE1);  LCD_Spi_WriteData(0xD0);LCD_Spi_WriteData(0x04);LCD_Spi_WriteData(0x0C);LCD_Spi_WriteData(0x11);
								LCD_Spi_WriteData(0x13);LCD_Spi_WriteData(0x2C);LCD_Spi_WriteData(0x3F);LCD_Spi_WriteData(0x44);
								LCD_Spi_WriteData(0x51);LCD_Spi_WriteData(0x2F);LCD_Spi_WriteData(0x1F);LCD_Spi_WriteData(0x1F);
								LCD_Spi_WriteData(0x20);LCD_Spi_WriteData(0x23);

	LCD_Spi_WriteCommand(ST7789_INVON);
	LCD_Spi_WriteCommand(ST7789_SLPOUT); //sleep out
    LCD_Spi_WriteCommand(ST7789_NORON); delayms(50); //8: Normal display on, no args, w/delay
    LCD_Spi_WriteCommand(ST7789_DISPON);delayms(50); // 9: Main screen turn on, no args, delay

    LCD_fillScreen(COLOR_BLACK);//ST7789_WHITE);
}
//====================== application ===============================
void ST7789_displayHello()
{
	printf("Hello\r\n");
//    LCD_displayString(1,100,"Hello YoonLAB");
//    LCD_displayString(1,76,"Dots:128*160");
    //LCD_displayString(1,88,"IC: ST7789R");
    //LCD_displayString(1,112,"ST7789 HELLO 2013");
	LCD_displayString(0,0,"ST7789 for STM");
    //LCD_drawImageRectArea565(20, 20, 160,80, gImage_lcd);//,  ST7789_RED);///
}

/*
void ST7789_displayBitmap(){
	int i=0;
	//for(i=0;i<4;i++){
		LCD_drawImageRectArea565(0+i*10, 8+i*10, 160,80, gImage_lcd);//128,  ST7789_RED);///
	//}
}
*/




void ST7789_ShowDemo()
{
	unsigned char i;

    ST7789_displayHello();

	for(i=0;i<5;i++){
		//LCD_drawCircle(40+(i*10),100,6,COLOR_GREEN);//LCD_drawCircle(40+(i*5),80+(i*5),10,ST7789_GREEN);
		delayms(400);
		LCD_fillCircle(40+(i*15),100,4,COLOR_YELLOW); //LCD_fillCircle(40+(i*5),80+(i*5),8,ST7789_RED);
	}

	LCD_drawRect(50, 120, 40, 20, COLOR_YELLOW);
	LCD_fillRect(51, 121, 38, 18, COLOR_CYAN);
}
#if 0
void stmSpiST7789_240X240_LcdLoop(){
	unsigned char i;
    printf("stmST7789(240X240) SPI LCD Test");

#if 1//(PROCESSOR == PROCESSOR_STM32F103RCT6)
    ST7789_GpioSpiSetup();

    ST7789_InitSeq();

	LCD_displayString(1,144,"GPWS");
	LCD_displayString(1,152,"Proximity : ");
#endif
	//GFX_setCursor(8, 10);
	ST7789_ShowDemo();
	while(1){
		for(i=0;i<256;i++){
			LCD_displayString(1,152,"Proximity : ");
			LCD_displayValue(80, 152, i);
			delayms(1000);
		}
	}
}
#endif
//================================ clock ====================================



/*
void LCD_drawClockHandS(int deg, int style, int w, int l, int color1=0, int color2=0)
{
  int i,num = 4;
  cirColor = YELLOW;
  cirSize = 1;
  switch(style) {
    default:
    case 0:  // tris 013, 230, 024, rect with triangle
      px[0]=-w+1, py[0]= l-5;
      px[1]=-w+1, py[1]=-10;
      px[2]= w-1, py[2]= l-5;
      px[3]= w-1, py[3]=-10;
      px[4]=   0, py[4]= l-5+7;
      num = 5;
      break;
    case 1:  // tris 013,023, peak style
      px[0]= 0, py[0]= l;
      px[1]=-w, py[1]= 0;
      px[2]= w, py[2]= 0;
      px[3]= 0, py[3]=-15;
      break;
    case 2:  // tris 013, 230, rect
      px[0]=-w+1, py[0]= l;
      px[1]=-w+1, py[1]=-10;
      px[2]= w-1, py[2]= l;
      px[3]= w-1, py[3]=-12;
      break;
    case 3:  // tris 013, 230, rect with peak, sec thin long
      px[0]=-w+1, py[0]= l;
      px[1]=-w+1, py[1]=-15;
      px[2]= w-1, py[2]= l;
      px[3]= w-1, py[3]=-15;
      cirColor = RED;
      cirSize = 3;
      break;
  }
  int x[5],y[5];
  int cc = fastCos(deg+180);
  int ss = fastSin(deg+180);
  for(i=0;i<num;i++) {
    x[i] = px[i]*cc - py[i]*ss;
    y[i] = px[i]*ss + py[i]*cc;
    x[i] = cx + (x[i]+(x[i]>0?MAXSIN/2:-MAXSIN/2))/MAXSIN;
    y[i] = cy + (y[i]+(y[i]>0?MAXSIN/2:-MAXSIN/2))/MAXSIN;
  }
  //imgTriangle(x[0],y[0], x[1],y[1], x[3],y[3], col);
  //imgTriangle(x[2],y[2], x[3],y[3], x[0],y[0], col);
  //if(num==5) imgTriangle(x[0],y[0], x[2],y[2], x[4],y[4], col);
  imgTriangle(x[0],y[0], x[1],y[1], x[3],y[3], color2);
  imgTriangle(x[2],y[2], x[3],y[3], x[0],y[0], color1);
}
*/






//==============


// --------------------------------------------------------------------------


/*
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
// --------------------------------------------------------------------------
extern const char *delim;
void setBuildTime(tm_t *mt)
{
  // Timestamp format: "Mar 3 2019 12:34:56"
	snprintf(LCD_clk.bld,40,"%s %s\n", "May 3 2021", "12:34:56");//snprintf(bld,40,"%s %s\n", __DATE__, __TIME__);
  char *token = strtok(LCD_clk.bld, delim);
  while(token) {
    int m = str2month((const char*)token);
    if(m>0) {
      mt->month = m;
      token = strtok(NULL, delim);  mt->day = atoi(token);
      token = strtok(NULL, delim);  mt->year = atoi(token) - 1970;
      token = strtok(NULL, delim);  mt->hour = atoi(token);
      token = strtok(NULL, delim);  mt->minute = atoi(token);
      token = strtok(NULL, delim);  mt->second = atoi(token);
    }
    token = strtok(NULL, delim);
  }

  snprintf(LCD_clk.bld,40,"Build: %02d-%02d-%02d %02d:%02d:%02d\n",mt->year+1970,mt->month,mt->day,mt->hour,mt->minute,mt->second);
  printf("%s\r\n",LCD_clk.bld);//Serial.println(bld);
  //rtclock_setTime(rtclock_makeTime(mt)+10);
}

/*
void stmSpiST7789_240X240_Clock_setup()
{
  //Serial.begin(9600);
  //pinMode(BUTTON, INPUT_PULLUP);
  //pinMode(PC13, OUTPUT);
  //lcd.init(LCD_TFTWIDTH, LCD_TFTHEIGHT);

  //rtclock.breakTime(rtclock.now(), LCD_clk.curTime);
  //if(LCD_clk.curTime.year+1970 < 2021) setBuildTime(&LCD_clk.curTime);  //  <2021 - invalid year
  imgRect(0,0,LCD_TFTWIDTH,LCD_TFTHEIGHT);
  //LCD_clk.ms = millis();
}
*/

//===============================================================================================================
void stmSpiST7789_240X240_Clock_LcdLoop()
{

	int st;// = checkButton();
	char *curTimeStr = "17:20:19";

	LCD_spi_GpioSetup();//

    ST7789_InitSeq();

    LCD_Clock_Config_Img(face8, LCD_TFTWIDTH/2,LCD_TFTHEIGHT/2);
	LCD_drawImgRectAreaWithPalette(0,0,LCD_IMGWIDTH,LCD_IMGHEIGHT);
	LCD_clk.ms = millis();

    //rtclock.breakTime(rtclock.now(), curTime);
    //int st = checkButton();
    //if(st<0 && setMode==0)
	LCD_clk.setMode=1;
  if(LCD_clk.setMode>0) {
    if(st>0) { if(++LCD_clk.setMode>2) LCD_clk.setMode=0; }
    //if(setMode==1 && st<0) { if(++hh>23) hh=0; start=1; delay(600); }
    //if(setMode==2 && st<0) { if(++mm>59) mm=0; start=1; delay(200); }

    LCD_clk.curTime.hour= 1;//hh;
    LCD_clk.curTime.minute=30; //mm;

    //rtclock_setTime(rtclock_makeTime(curTime));

  } else {
    //if(st>0) nextFace();
  }

  LCD_clk.start = 1;
  LCD_clk.setMode=0;

  while(1){

	  LCD_Clock_Update();
  }
}


#endif

