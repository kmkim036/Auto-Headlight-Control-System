#ifndef __LCD_H
#define __LCD_H

//=====================================
#define LCDCONT_IS_ST7735 1
#define LCDCONT_IS_ST7789 2
#define LCDCONT_IS_GC9A01 3
#define LCDCONT_IS_IL9163_PARA 4
#define LCDCONT_IS LCDCONT_IS_GC9A01 //LCDCONT_IS_ST7789//LCDCONT_IS_ST7735//LCDCONT_ST7789
//===========================================
#define LCDAPP_IS_CLOCK 1
#define LCDAPP_IS_GAUGE 2
#define LCDAPP_IS_DASH  3
#define LCDAPP_IS LCDAPP_IS_GAUGE//LCDAPP_IS_CLOCK//LCDAPP_IS_DASH//LCDAPP_IS_GAUGE//

extern const unsigned char VUmeter20[];

//===========================================
#if (LCDCONT_IS == LCDCONT_IS_ST7735)
	#define LCD_TFTWIDTH  160
	#define LCD_TFTHEIGHT 128

	#define LCD_IMGWIDTH  128
	#define LCD_IMGHEIGHT 128

#else
	#define LCD_TFTWIDTH  240
	#define LCD_TFTHEIGHT 240

	#define LCD_IMGWIDTH  240
	#if (LCDAPP_IS == LCDAPP_IS_GAUGE)
		#define LCD_IMGHEIGHT 240//180
	#else
		#define LCD_IMGHEIGHT 240
	#endif
#endif
//===============
//#define LCD_ROTATION 2 //(normally for 240x240 IPS LCD)
//#define LCD_ROTATION 0 //(180)
#define LCD_ROTATION 1 //(90)
#if (LCD_ROTATION == 0)
	#define X_SHIFT 0
	#define Y_SHIFT 80
#elif (LCD_ROTATION == 1)
	#define X_SHIFT 80
	#define Y_SHIFT 0
#elif (LCD_ROTATION == 2)
	#define X_SHIFT 0
	#define Y_SHIFT 0
#elif (LCD_ROTATION == 3)
	#define X_SHIFT 0
	#define Y_SHIFT 0
#endif

#define LCD_SCANDIR_HORIZONTAL 0
#define LCD_SCANDIR_VERTICAL   1

// Color definitions ===============
#define	COLOR_BLACK   0x0000
#define	COLOR_BLUE    0x001F
#define	COLOR_RED     0xF800
#define	COLOR_GREEN   0x07E0
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_YELLOW  0xFFE0
#define COLOR_WHITE   0xFFFF

struct _LCD_gfx{
	unsigned char SCAN_DIR;
	unsigned char  tabcolor;
	volatile unsigned char *dataport, *clkport, *csport, *rsport;
	unsigned char _cs, _rs, _rst, _sid, _sclk;
    unsigned char datapinmask, clkpinmask, cspinmask, rspinmask;
    unsigned char colstart, rowstart; // some displays need this changed
    unsigned char colstart2, ///< Offset from the right
                  rowstart2;  ///< Offset from the bottom
    short _width, _height;
    short  cursor_x, cursor_y;
    unsigned short textcolor, textbgcolor;
    unsigned char  textsize;
    unsigned char  rotation;
    unsigned char  wrap; // If set, 'wrap' text at right edge of display

    unsigned short width;
    unsigned short height;

};

#define MAXSIN 255
#define DELAY 0x80

extern void GFX_SetPos(short x, short y) ;
extern void GFX_setTextSize(unsigned char s);
extern void GFX_setTextColor(unsigned short c, unsigned short b) ;
extern void GFX_setTextWrap(unsigned char w) ;
extern void LCD_pushColor(unsigned short color) ;

//===========================================
extern void spiMasterInit(void);
extern void stmSpiWrByte(unsigned char inbyte); //send cmd or data byte
extern unsigned char SpiRdByte();
//extern void SerialFlash_WriteEnable();// sends write enable command
//extern void somedelay(long ulLoop);

#define pgm_read_byte(x) (*(x))

#if (PROCESSOR == PROCESSOR_STM32F103RCT6)
//=============== SPI ===========================
	#if 1 //KONG M84
	//nCS0   - PB12

	#define LCD_CS0_HIGH {GPIO_SetBits(GPIOB, GPIO_Pin_12);}
	#define LCD_CS0_LOW  {GPIO_ResetBits(GPIOB, GPIO_Pin_12);}

	//b) RS/A0/DataCmd - PA8 : D/C=1(Data);0(Command)
	//PA0
	#define LCD_RS_HIGH {GPIO_SetBits(GPIOA, GPIO_Pin_0);}
	#define LCD_RS_LOW  {GPIO_ResetBits(GPIOA, GPIO_Pin_0);}

	//c) RST
	//#define LCD_RESET_HIGH {GPIO_SetBits(GPIOC, GPIO_Pin_0);}
	//#define LCD_RESET_LOW  {GPIO_ResetBits(GPIOC, GPIO_Pin_0);}
	#else //LCD Module
	//nCS0   - PC14
	//nRESET - PC0
	//a) CS
	//#define LCD_CS0_HIGH {GPIO_SetBits(GPIOC, GPIO_Pin_14);}
	//#define LCD_CS0_LOW  {GPIO_ResetBits(GPIOC, GPIO_Pin_14);}
	#define LCD_CS0_HIGH {GPIO_SetBits(GPIOC, GPIO_Pin_6);}
	#define LCD_CS0_LOW  {GPIO_ResetBits(GPIOC, GPIO_Pin_6);}

	//b) RS/A0/DataCmd - PA8 : D/C=1(Data);0(Command)
	//#define LCD_RS_HIGH {GPIO_SetBits(GPIOA, GPIO_Pin_8);}
	//#define LCD_RS_LOW  {GPIO_ResetBits(GPIOA, GPIO_Pin_8);}

	//PC7
	#define LCD_RS_HIGH {GPIO_SetBits(GPIOC, GPIO_Pin_7);}
	#define LCD_RS_LOW  {GPIO_ResetBits(GPIOC, GPIO_Pin_7);}

	#define LCD_RESET_HIGH {GPIO_SetBits(GPIOC, GPIO_Pin_0);}
	#define LCD_RESET_LOW  {GPIO_ResetBits(GPIOC, GPIO_Pin_0);}
	#endif
#elif (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_GD32F130FX)
//=============== SPI2 for LCD ===========================
//nCS0   - PB12
#define LCD_CS0_HIGH {GPIO_SetBits(GPIOB, GPIO_Pin_12);}
#define LCD_CS0_LOW  {GPIO_ResetBits(GPIOB, GPIO_Pin_12);}

//RS/A0/DataCmd - PA0 : D/C=1(Data);0(Command)
#define LCD_RS_HIGH {GPIO_SetBits(GPIOA, GPIO_Pin_0);}
#define LCD_RS_LOW  {GPIO_ResetBits(GPIOA, GPIO_Pin_0);}
//RS/A0/DataCmd - PA8 : D/C=1(Data);0(Command)
//#define LCD_RS_HIGH {GPIO_SetBits(GPIOA, GPIO_Pin_8);}
//#define LCD_RS_LOW  {GPIO_ResetBits(GPIOA, GPIO_Pin_8);}

#define LCD_RESET_HIGH {GPIO_SetBits(GPIOA, GPIO_Pin_5);}
#define LCD_RESET_LOW  {GPIO_ResetBits(GPIOA, GPIO_Pin_5);}
#endif

extern void LCD_Spi_WriteData16(unsigned short c);
//=== app

extern unsigned g_elpased_sec;
#define ERR_SUCCESS 0

//app
struct _LCD_clock{
	tm_t curTime; //in yInc.h
	unsigned char *pFaceImg;// = clockface8;
	int faceIndex;
	unsigned char *pImg; //real image bitmap after several offset
	int style;// = 1;//??

	unsigned char start;// = 1;
	int setMode;// = 0;
	int sDeg,mDeg,hDeg;
	int sDegOld,mDegOld,hDegOld;
	unsigned long styleTime, ms;

	unsigned short cirColor;// = COLOR_YELLOW;
	int cirSize;// = 3;

    int cx,cy;// = 240/2, cy = 240/2; //clock center

    int hHandLength, hHandWidth; //hHandLength = 25*2, hHandWidth = 3*2;
    int mHandLength, mHandWidth; //int mHandLength = 36*2, mHandWidth = 3*2;
    int sHandLength, sHandWidth; //int sHandLength = 44*2, sHandWidth = 2*2;

    unsigned char hh; //unsigned char hh = txt2num(__TIME__+0);
    unsigned char mm; //unsigned char mm = txt2num(__TIME__+3);
    //unsigned char ss; //unsigned char ss = txt2num(__TIME__+6);

    char ss;

    int px[8],py[8];

    unsigned short *palette;//=(unsigned short*)clockface+3;
    unsigned short line[LCD_TFTWIDTH+2];


    unsigned short selHandColor1;// = gfx_RgbTo565(250,250,0);
    unsigned short selHandColor2;// = gfx_RgbTo565(180,180,0);
    unsigned short mHandColor1;// = gfx_RgbTo565(220,220,220);
    unsigned short mHandColor2;// = gfx_RgbTo565(150,150,150);
    unsigned short hHandColor1;// = gfx_RgbTo565(220,220,220);
    unsigned short hHandColor2;// = gfx_RgbTo565(150,150,150);
    //unsigned short hHandColor = gfx_RgbTo565(40,40,40);
    //unsigned short mHandColor = gfx_RgbTo565(80,80,80);
    //unsigned short hHandColor = gfx_RgbTo565(110,110,110);
    //unsigned short mHandColor = gfx_RgbTo565(180,180,180);
    //unsigned short sHandColor = RED;
    unsigned short sHandColor1;// = gfx_RgbTo565(250,80,80);
    unsigned short sHandColor2;// = gfx_RgbTo565(200,0,0);

    char bld[40];
};
extern const unsigned char  font[];
extern const unsigned char sinTab[];
//===
#if (LCDCONT_IS == LCDCONT_IS_ST7735)
//================ ST7735 ============================
// some flags for initR()
#define INITR_GREENTAB 0x0
#define INITR_REDTAB   0x1
#define INITR_BLACKTAB 0x2
//====================================================
//Commands
#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST77XX_MADCTL  0x36
//Memory Data Access Control Register@0x36
//MAP : D7 D6 D5 D4 D3  D2 D1 D0
	//para: MY MX MV ML RGG MH -  -
	#define MADCTL_MY  0x80 //Page   Address Order (0=Top to Bottom; 1=opposite)
	#define MADCTL_MX  0x40 //Column Address Order (0=Left to Right; 1=opposite)
	#define MADCTL_MV  0x20 //Page/Col Order (0=Normal; 1=Reverse)
	#define MADCTL_ML  0x10 //Line Address Order(0=LCD Refresh Top to Bottom)
	#define MADCTL_RGB 0x00 //was 0x08 //RGB/BGR Order(0=RGB); 8=BGR
	#define MADCTL_MH  0x04

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions ===============
#define	ST7735_BLACK   0x0000
#define	ST7735_BLUE    0x001F
#define	ST7735_RED     0xF800
#define	ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF

#elif (LCDCONT_IS == LCDCONT_IS_ST7789)
//================ ST7789 ============================
//for 240x240 LCD Dimension 240*240 Pixels 1.3"
//====================================================
//Commands
#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST77XX_MADCTL  0x36
	//Memory Data Access Control Register@0x36
    //MAP : D7 D6 D5 D4 D3  D2 D1 D0
 	//para: MY MX MV ML RGG MH -  -
	#define MADCTL_MY  0x80 //Page   Address Order (0=Top to Bottom; 1=opposite)
	#define MADCTL_MX  0x40 //Column Address Order (0=Left to Right; 1=opposite)
	#define MADCTL_MV  0x20 //Page/Col Order (0=Normal; 1=Reverse)
	#define MADCTL_ML  0x10 //Line Address Order(0=LCD Refresh Top to Bottom)
	#define MADCTL_RGB 0x00 //0x08 //RGB/BGR Order(0=RGB)
	#define MADCTL_MH  0x04

#define ST7789_FRMCTR1 0xB1
#define ST7789_FRMCTR2 0xB2
#define ST7789_FRMCTR3 0xB3
#define ST7789_INVCTR  0xB4
#define ST7789_DISSET5 0xB6

#define ST7789_PWCTR1  0xC0
#define ST7789_PWCTR2  0xC1
#define ST7789_PWCTR3  0xC2
#define ST7789_PWCTR4  0xC3
#define ST7789_PWCTR5  0xC4
#define ST7789_VMCTR1  0xC5

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

#define ST7789_PWCTR6  0xFC

#define ST7789_GMCTRP1 0xE0
#define ST7789_GMCTRN1 0xE1



void ST7789_setAddressWindow(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1);
void ST7789_pushColor(unsigned short color);
void ST7789_fillScreen(unsigned short color);
void ST7789_drawPixel(short x, short y, unsigned short color);
void ST7789_drawFastVLine(short x, short y, short h, unsigned short color);
void ST7789_drawFastHLine(short x, short y, short w, unsigned short color);
void ST7789_fillRect(short x, short y, short w, short h, unsigned short color);
void ST7789_setRotation(unsigned char r);
void ST7789_invertDisplay(unsigned char i); //was boolean
unsigned short gfx_RgbTo565(unsigned char r, unsigned char g, unsigned char b);
void ST7789_spiwritecommand(unsigned char c);
void ST7789_spiwritedata(unsigned char d);
void ST7789_spicommandList(unsigned char *addr);
void ST7789_spicommonInit(unsigned char *cmdList);
#elif ((LCDCONT_IS == LCDCONT_IS_GC9A01))
//====================================================
//Commands
#define GC9A01_NOP     0x00
#define GC9A01_SWRESET 0x01
#define GC9A01_RDDID   0x04 //Read Display Identification Info. (xx-00-9a-01)
#define GC9A01_RDDST   0x09 //Read Display Status

#define GC9A01_SLPIN   0x10 //Enter Sleep Mode
#define GC9A01_SLPOUT  0x11
#define GC9A01_PTLON   0x12 //Partial Mode ON
#define GC9A01_NORON   0x13 //Normal Display Mode ON

#define GC9A01_INVOFF  0x20
#define GC9A01_INVON   0x21
#define GC9A01_DISPOFF 0x28
#define GC9A01_DISPON  0x29
#define GC9A01_CASET   0x2A //Column Address Set
#define GC9A01_RASET   0x2B //Row(Page) Address Set
#define GC9A01_RAMWR   0x2C //Memory Write
#define GC9A01_RAMRD   0x2E //Memory Read

#define GC9A01_PTLAR   0x30 //Partial Area
#define GC9A01_COLMOD  0x3A
#define GC9A01_MADCTL  0x36 //Memory Access Control (Important)
	//Memory Data Access Control Register@0x36
    //MAP : D7 D6 D5 D4 D3  D2 D1 D0
 	//para: MY MX MV ML RGG MH -  -
	#define MADCTL_MY  0x80 //Page   Address Order (0=Top to Bottom; 1=opposite)
	#define MADCTL_MX  0x40 //Column Address Order (0=Left to Right; 1=opposite)
	#define MADCTL_MV  0x20 //Page/Col Order (0=Normal; 1=Reverse)
	#define MADCTL_ML  0x10 //Line Address Order(0=LCD Refresh Top to Bottom)
	#define MADCTL_RGB 0x08 //0x00 //RGB/BGR Order(0=RGB) (1=BGR)
	#define MADCTL_MH  0x04 //Horizonal Refresh Order(Direction)

#define GC9A01_WRITE_CTRL_DISPLAY 0x53 //BL Control...

#define GC9A01_VCORE_VOLTAGE_CONTROL 0xA7

#define GC9A01_FRMCTR1 0xB1
#define GC9A01_FRMCTR2 0xB2
#define GC9A01_FRMCTR3 0xB3
#define GC9A01_INVCTR  0xB4
#define GC9A01_BLANKING_PORCH_CONTROL  	0xB5
#define GC9A01_DISPLAY_FUNCTION_CONTROL 0xB6

#define GC9A01_PWCTR1  0xC0
#define GC9A01_POCTR2  0xC1 //POWER CRITERION CONTROL
#define GC9A01_PWCTR3  0xC2
#define GC9A01_PWCTR4  0xC3
#define GC9A01_VREG1A_VOLTAGE_CONTROL  0xC3
#define GC9A01_PWCTR5  0xC4
#define GC9A01_VREG1B_VOLTAGE_CONTROL  0xC4
#define GC9A01_VMCTR1  0xC5


#define GC9A01_RDID1   0xDA //Read LCD Module/Driver Version
#define GC9A01_RDID2   0xDB //
#define GC9A01_RDID3   0xDC
#define GC9A01_RDID4   0xDD


#define GC9A01_INTERFACE_CONTROL  		0xF6
#define GC9A01_PWCTR6  					0xFC

#define GC9A01_GMCTRP1 0xE0
#define GC9A01_GMCTRN1 0xE1
#define GC9A01_FRAME_RATE 				0xE8
#define GC9A01_SPI2_DATA_CONTROL 		0xE9
#define GC9A01_CHARGE_PUMP_FREQ_CONTROL 		0xEC
#define GC9A01_INNER_REG_EN_2 			0xEF

#define GC9A01_SET_GAMMA_1 				0xF0
#define GC9A01_SET_GAMMA_2 				0xF1
#define GC9A01_SET_GAMMA_3 				0xF2
#define GC9A01_SET_GAMMA_4 				0xF3

#define GC9A01_INNER_REG_EN_1 			0xFE

void GC9A01_initB(void);   // for GC9A01B displays
void GC9A01_initR(unsigned char options); // for GC9A01R
//void GC9A01_SetWindows(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1);
void GC9A01_SetWindows(unsigned short Xstart, unsigned short Ystart, unsigned short Xend, unsigned short Yend);
void GC9A01_pushColor(unsigned short color);
void GFX_fillScreen(unsigned short color);
void GC9A01_drawPixel(short x, short y, unsigned short color);
void GC9A01_drawFastVLine(short x, short y, short h, unsigned short color);
void GC9A01_drawFastHLine(short x, short y, short w, unsigned short color);
void GC9A01_fillRect(short x, short y, short w, short h, unsigned short color);
void GC9A01_setRotation(unsigned char r);
void GC9A01_invertDisplay(unsigned char i); //was boolean
unsigned short GC9A01_Color565(unsigned char r, unsigned char g, unsigned char b);
void GC9A01_spiwritecommand(unsigned char c);
void GC9A01_spiwritedata(unsigned char d);
void GC9A01_spicommandList(unsigned char *addr);
void GC9A01_spicommonInit(unsigned char *cmdList);
#else
#endif //
//==================
#endif
