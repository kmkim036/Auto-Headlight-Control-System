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
#if (PROCESSOR == PROCESSOR_STM32F103RCT6)

#include "fonts.h"
/* Header files with pictures as integer arrays in RGB565 format */
#include "img/img00.h"
//#include "img/img01.h"
#include "img/img02.h"
//#include "img/img03.h"
//#include "img/img04.h"
//#include "img/img05.h"
//#include "img/img06.h"

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t CCR1_Val = 300;
uint16_t CCR2_Val = 100;

/*
 * How to connect LCD with STM32F4Discovery:
 *
 * RS    ->  PD11
 * WR    ->  PD5
 * RD    ->  PD4
 * CS    ->  PD7
 * RESET ->  PC13
 *
 * DB0   ->  PD14
 * DB1   ->  PD15
 * DB2   ->  PD0
 * DB3   ->  PD1
 * DB4   ->  PE7
 * DB5   ->  PE8
 * DB6   ->  PE9
 * DB7   ->  PE10
 * DB8   ->  PE11
 * DB9   ->  PE12
 * DB10  ->  PE13
 * DB11  ->  PE14
 * DB12  ->  PE15
 * DB13  ->  PD8
 * DB14  ->  PD9
 * DB15  ->  PD10
 *
 * BL_CNT -> PD12
 *
 * TP_IRQ -> PB12
 * TP_SO -> PB14
 * TP_SI -> PB15
 * TP_SCK -> PB13
 * TP_CS -> PC6
 *
 *
 * Display coordinates:
 *  [x,y]
 *     [0,319]                                 [0,0]
 *       -----------------------------------------
 *       |                                       |
 *       |                                       |
 *       |                                       |
 *       |                                       |
 *       |             TOUCH DISPLAY             |
 *       |                                       |
 *       |                                       |
 *       |                                       |
 *       |                                       |
 *       -----------------------------------------
 *   [239,319]                               [239,0]
 */





/* Type Definitions **********************************************************/

typedef struct
{
  __IO uint16_t Register;  /* LCD Index Write            Address offset 0x00 */
  __IO uint16_t Data;      /* LCD Data Write             Address offset 0x02 */
}LCD_TypeDef;

/* Definitions ***************************************************************/

/*
 * FSMC Memory Bank 1: 0x60000000 to 6FFFFFFF
 * NAND bank selections:
 *   SECTION NAME      HADDR[17:16]  ADDRESS RANGE
 *   Address section   1X            0x020000-0x03FFFF
 *   Command section   01            0x010000-0x01FFFF
 *   Data section      00            0x000000-0x0FFFF
 */

#define LCD_BASE            ((uint32_t) (0x60000000 | 0x0001FFFE))
#define LCD                 ((LCD_TypeDef *) LCD_BASE)

#define GDDRAM_PREPARE      0x0022  /* Graphic Display Data RAM Register. */

#define LCD_WHITE           0xFFFF
#define LCD_BLACK           0x0000
#define LCD_GREEN           0x07E0
#define LCD_RED             0xF800
#define LCD_BLUE            0x001F
#define LCD_GREY            0xF7DE
#define LCD_ORANGE          0xFA20
#define LCD_YELLOW          0xFFE0



/* Function Prototypes *******************************************************/

void Clear_Screen(uint16_t color);
void Set_Cursor(uint16_t x, uint16_t y);
void Draw_Pixel(uint16_t x, uint16_t y, uint16_t color);
void Draw_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void Draw_Rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void Draw_Full_Rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void Draw_Circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);
void Draw_Full_Circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);
void Draw_Char(uint16_t x, uint16_t y, const uint16_t *c, uint16_t color);
void Display_Char(uint16_t x, uint16_t y, uint8_t c, uint16_t color);
void Display_String(uint16_t x, uint16_t y, char *ptr, uint16_t color);
void Set_Font(sFONT *fonts);
void Draw_Image(uint16_t x, uint16_t y, uint16_t x_res, uint16_t y_res,const uint16_t *ptr_image);
void Init_LCD(void);
void Write_Command(uint16_t reg, uint16_t data);
void Write_GDDRAM_Prepare(void);
void Write_Data(uint16_t data);
void Init_GPIO(void);
void Init_FSMC(void);
void Init_SysTick(void);
void Delay_ms(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
int abs(int);

#endif /* LCD_STM32F4_H_ */

/* Private Variables *********************************************************/

static sFONT *Current_Font;
static __IO uint32_t TimingDelay;

/* Functions *****************************************************************/

/*
 * Clear the screen.
 */

void Clear_Screen(uint16_t color)
{
  __IO uint32_t i = 0;

  Set_Cursor(0, 0);

  i = 0x12C00;
  Write_GDDRAM_Prepare();
  while(i--)
  {
    Write_Data(color);
  }
}

/*
 * Set Cursor to Position [x, y].
 */

void Set_Cursor(uint16_t x, uint16_t y)
{
  Write_Command(0x004E, x);
  Write_Command(0x004F, y);
}

/*
 * Draw a Single Pixel on Position [x, y].
 */

void Draw_Pixel(uint16_t x, uint16_t y, uint16_t color)
{
  Set_Cursor(x, y);
  Write_GDDRAM_Prepare();
  Write_Data(color);
}

/*
 * Draw a line in the requested color.
 */

void Draw_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  uint8_t yLonger = 0;
  int incrementVal, endVal;
  int shortLen = y2-y1;
  int longLen = x2-x1;
	int decInc;
	int j = 0, i = 0;

  if(abs(shortLen) > abs(longLen)) {
    int swap = shortLen;
    shortLen = longLen;
    longLen = swap;
    yLonger = 1;
  }

  endVal = longLen;

  if(longLen < 0) {
    incrementVal =- 1;
    longLen =- longLen;
    endVal--;
  } else {
    incrementVal = 1;
    endVal++;
  }

  if(longLen == 0)
    decInc = 0;
  else
    decInc = (shortLen << 16) / longLen;

  if(yLonger) {
    for(i = 0;i != endVal;i += incrementVal) {
      Draw_Pixel(x1 + (j >> 16),y1 + i,color);
      j += decInc;
    }
  } else {
    for(i = 0;i != endVal;i += incrementVal) {
      Draw_Pixel(x1 + i,y1 + (j >> 16),color);
      j += decInc;
    }
  }
}

/*
 * Draw a rectangle in the requested color.
 * x1, y1 - the position of one corner
 * x2, y2 - the position of the other corner
 * color - color of the rectangle.
 */

void Draw_Rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  Draw_Line(x1, y1, x2, y1, color);
  Draw_Line(x2, y1, x2, y2, color);
  Draw_Line(x2, y2, x1, y2, color);
  Draw_Line(x1, y2, x1, y1, color);
}

/*
 * Draw a full rectangle.
 * x1, y1 - the position of one corner
 * x2, y2 - the position of the other corner
 * color - color of the rectangle.
 */

void Draw_Full_Rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  while(x1 < x2)
  {
    Draw_Line(x1, y1, x1, y2, color);
    x1++;
  }
}

/*
 * Draw a circle.
 * x, y - center of circle.
 * r - radius.
 * color - color of the circle.
 */

void Draw_Circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
  int32_t  D;       /* Decision Variable */
  uint32_t  CurX;   /* Current X Value */
  uint32_t  CurY;   /* Current Y Value */

  D = 3 - (r << 1);
  CurX = 0;
  CurY = r;

  while (CurX <= CurY)
  {
    if (((x+CurX) < 240) && ((y+CurY) < 320))
      Draw_Pixel(x+CurX, y+CurY, color);
    if (((x+CurX) < 240) && ((y-CurY) >= 0))
      Draw_Pixel(x+CurX, y-CurY, color);
    if (((x-CurX) >= 0) && ((y+CurY) < 320))
      Draw_Pixel(x-CurX, y+CurY, color);
    if (((x-CurX) >= 0) && ((y-CurY) >= 0))
      Draw_Pixel(x-CurX, y-CurY, color);
    if (((x+CurY) < 240) && ((y+CurX) < 320))
      Draw_Pixel(x+CurY, y+CurX, color);
    if (((x+CurY) < 240) && ((y-CurX) >= 0))
      Draw_Pixel(x+CurY, y-CurX, color);
    if (((x-CurY) >= 0) && ((y+CurX) < 320))
      Draw_Pixel(x-CurY, y+CurX, color);
    if (((x-CurY) >= 0) && ((y-CurX) >= 0))
      Draw_Pixel(x-CurY, y-CurX, color);

    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}

/*
 * Draw a full circle.
 * x, y - center of circle.
 * r - radius.
 * color - color of the circle.
 */

void Draw_Full_Circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
  int32_t  D;       /* Decision Variable */
  uint32_t  CurX;   /* Current X Value */
  uint32_t  CurY;   /* Current Y Value */

  D = 3 - (r << 1);
  CurX = 0;
  CurY = r;

  while (CurX <= CurY)
  {
    if(CurY > 0)
    {
      Draw_Line(x-CurX, y+CurY, x-CurX, y-CurY, color);
      Draw_Line(x+CurX, y+CurY, x+CurX, y-CurY, color);
    }

    if(CurX > 0)
    {
      Draw_Line(x-CurY, y+CurX, x-CurY, y-CurX, color);
      Draw_Line(x+CurY, y+CurX, x+CurY, y-CurX, color);
    }

    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}

/*
 * Draw a character.
 * x, y - position
 * color - character's color
 * *c - pointer to character data
 */

void Draw_Char(uint16_t x, uint16_t y, const uint16_t *c, uint16_t color)
{
  uint32_t index = 0, i = 0;

  for(index = 0; index < Current_Font->Height; index++)
  {
    for(i = 0; i < Current_Font->Width; i++)
    {
      if( ((((c[index] & ((0x80 << ((Current_Font->Width / 12 ) * 8 ) ) >> i)) == 0x00) && (Current_Font->Width <= 12)) ||
          (((c[index] & (0x1 << i)) == 0x00)&&(Current_Font->Width > 12 )))  == 0x00)
      {
        Draw_Pixel(x, y-1-i, color);
      }
    }
    x++;
  }
}

/*
 * Display a character.
 * x, y - position
 * c - character in ASCII
 * color - character's color
 */

void Display_Char(uint16_t x, uint16_t y, uint8_t c, uint16_t color)
{
  c -= 32;
  Draw_Char(x, y, &Current_Font->table[c * Current_Font->Height], color);
}

/*
 * Display a string.
 * x - line. Next line have to respect the high of used font.
 * y - vertical position
 * *ptr - pointer to string.
 */

void Display_String(uint16_t x, uint16_t y, char *ptr,uint16_t color)
{
  uint16_t refcolumn = y;

  /* Send the string character by character on LCD */
  while ((*ptr != 0) & (((refcolumn + 1) & 0xFFFF) >= Current_Font->Width))
  {
    /* Display one character on LCD */
    Display_Char(x, refcolumn, *ptr, color);
    /* Decrement the column position by 16 */
    refcolumn -= Current_Font->Width;
    /* Point on the next character */
    ptr++;
  }
}

/*
 * Sets the text font.
 */

void Set_Font(sFONT *fonts)
{
  Current_Font = fonts;
}

/*
 * Draw an image in format GRB565.
 * x, y - position, where to start displaying.
 * x_res, y_res - resolution in pixels.
 * *ptr_image - pointer to image array.
 */

void Draw_Image(uint16_t x, uint16_t y, uint16_t x_res, uint16_t y_res,const uint16_t *ptr_image)
{
  uint16_t i = 0, j = 0;

  for(i = 0; i < x_res; i++)
  {
    Set_Cursor((x+i), y);
    Write_GDDRAM_Prepare();

    for(j = 0; j < y_res; j++)
    {
      Write_Data(*(ptr_image++));
    }
  }
}

/*
 * Reset and Initialize Display.
 */

void Init_LCD(void)
{
  /* Reset */

  GPIO_ResetBits(GPIOC, GPIO_Pin_13);
  delayms(30);
  GPIO_SetBits(GPIOC, GPIO_Pin_13);
  delayms(10);

  /* Display ON Sequence (data sheet page 72) */

  Write_Command(0x0007, 0x0021);
  Write_Command(0x0000, 0x0001);
  Write_Command(0x0007, 0x0023);
  Write_Command(0x0010, 0x0000);  /* Exit Sleep Mode */
  delayms(30);
  Write_Command(0x0007, 0x0033);

  /*
   * Entry Mode R11h = 6018h
   *
   * DFM1 = 1, DFM0 = 1 => 65k Color Mode
   * ID0 = 1, AM = 1    => the way of automatic incrementing
   *                       of address counter in RAM
   */
  Write_Command(0x0011, 0x6018);
  Write_Command(0x0002, 0x0600);  /* AC Settings */

  /* Initialize other Registers */

  /*
   * Device Output Control R01h = 2B3Fh
   *
   * REV = 1            => character and graphics are reversed
   * BGR = 1            => BGR color is assigned from S0
   * TB  = 1            => sets gate output sequence (see datasheet page 29)
   * MUX[8, 5:0]        => specify number of lines for the LCD driver
   */
  Write_Command(0x0001, 0x2B3F);
}

/*
 * Write to LCD RAM.
 */

void Write_Command(uint16_t reg, uint16_t data)
{
  LCD->Register = reg;
  LCD->Data = data;
}

/*
 * Prepares writing to GDDRAM.
 * Next coming data are directly displayed.
 */

void Write_GDDRAM_Prepare(void)
{
  LCD->Register = GDDRAM_PREPARE;
}

/*
 * Writes data to last selected register.
 * Used with function Write_GDDRAM_Prepare().
 */

void Write_Data(uint16_t data)
{
  LCD->Data = data;
}

/*
 * Initialize GPIO ports D and E for FSMC use.
 * Also initialize PC6 for RESET.
 * RS will be on A16.
 */

void Init_GPIO(void)
{
  GPIO_InitTypeDef init={0};

  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);

  /* RESET */
  //init.GPIO_Pin=GPIO_Pin_13;
  //init.GPIO_Mode=GPIO_Mode_OUT;
  //init.GPIO_Speed=GPIO_Speed_50MHz;
  //GPIO_Init(GPIOC,&init);

  /* PORTD */
  init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
      GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
      GPIO_Pin_11 | GPIO_Pin_14 | GPIO_Pin_15;
  init.GPIO_Mode = GPIO_Mode_AF;
  init.GPIO_Speed = GPIO_Speed_2MHz;
  init.GPIO_OType = GPIO_OType_PP;
  init.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &init);


  /* PORTD */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);     // D2
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);     // D3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);     // NOE -> RD
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);     // NWE -> WR
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);     // NE1 -> CS
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);     // D13
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);     // D14
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);    // D15
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);    // A16 -> RS
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);    // D0
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);    // D1


    /* PORTE */
    init.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
        GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
        GPIO_Pin_15;
    init.GPIO_Mode = GPIO_Mode_AF;
    init.GPIO_Speed = GPIO_Speed_2MHz;
    init.GPIO_OType = GPIO_OType_PP;
    init.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &init);

    /* PORTE */
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);     // D4
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);     // D5
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);     // D6
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);    // D7
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);    // D8
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);    // D9
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);    // D10
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);    // D11
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);    // D12

}

/*
 * Initialize NOR/SRAM Bank 1.
 */

void Init_FSMC(void)
{

  FSMC_NORSRAMTimingInitTypeDef timing={0};
  FSMC_NORSRAMInitTypeDef init={0};

  RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);

  timing.FSMC_AddressSetupTime=0x00;
  timing.FSMC_DataSetupTime=0x0A;
  timing.FSMC_CLKDivision=0x0;
  timing.FSMC_AccessMode=FSMC_AccessMode_A;

  /*
   * Data/Address MUX = Disable
   * Memory Type = SRAM
   * Data Width = 16bit
   * Write Operation = Enable
   * Extended Mode = Disable
   * Asynchronous Wait = Disable
   */

  init.FSMC_Bank=FSMC_Bank1_NORSRAM1;
  init.FSMC_DataAddressMux=FSMC_DataAddressMux_Disable;
  init.FSMC_MemoryType=FSMC_MemoryType_SRAM;
  init.FSMC_MemoryDataWidth=FSMC_MemoryDataWidth_16b;
  init.FSMC_BurstAccessMode=FSMC_BurstAccessMode_Disable;
  init.FSMC_WaitSignalPolarity=FSMC_WaitSignalPolarity_Low;
  init.FSMC_WrapMode=FSMC_WrapMode_Disable;
  init.FSMC_WaitSignalActive=FSMC_WaitSignalActive_BeforeWaitState;
  init.FSMC_WriteOperation=FSMC_WriteOperation_Enable;
  init.FSMC_WaitSignal=FSMC_WaitSignal_Disable;
  init.FSMC_ExtendedMode=FSMC_ExtendedMode_Disable;
  init.FSMC_WriteBurst=FSMC_WriteBurst_Disable;
  init.FSMC_ReadWriteTimingStruct=&timing;
  init.FSMC_WriteTimingStruct=&timing;
  init.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable;

  FSMC_NORSRAMInit(&init);
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1,ENABLE);
}

/*
 * Initialize SysTick to 1 ms.


void Init_SysTick(void)
{
  RCC_ClocksTypeDef RCC_Clocks;

  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
}
*/
/*
 * Inserts a delay time.


void delayms(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0)
  {
  }
}
*/
/*
 * Decrements the TimingDelay variable.
 * Called from interrupt.
 */

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}


int stm_ssd1289_lcdLoop (void)
{
   //Init_SysTick();
  Init_GPIO();
  Init_FSMC();
  Init_LCD();
  //pwm_init();
  touch_init();

  /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_stm32f4xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_stm32f4xx.c file
       */

  /* Demo */
  //TIM4->CCR1 = CCR1_Val;  //TIM4->CCR2 = CCR2_Val;

  Clear_Screen(0x0000);
	Demo_MMIA();
  while(1)
  {
	  delayms(100);
	  Convert_Pos();

    //Draw_Image(0, 319, 240, 320, img03);
    //delayms(3000);

    //Draw_Image(0, 319, 240, 320, img04);
    //delayms(3000);

    //Draw_Image(0, 319, 240, 320, img05);
    //delayms(3000);

    //Draw_Image(0, 319, 240, 320, img06);
    //delayms(3000);
  }

  //return 0;
}

/*
 * Demonstration project designed for MMIA.
 */

void Demo_MMIA(void)
{
  uint16_t Number=0;
  //int CharCount;
  char StrNumber[10];

  Clear_Screen(0x0000);

  // delayms(3000);

  Set_Font(&Font16x24);
  Display_String(14, 295, "EtherCrafts", LCD_WHITE);
  uint16tostr(StrNumber, Number, 10);
  Display_String(43, 295, StrNumber, LCD_WHITE);

  //CharCount = sprintf(StrNumber,"%d", Number);
  //Display_String(43, 295, StrNumber, LCD_WHITE);

  Display_String(72, 287, "KISADO", LCD_WHITE);
  Set_Font(&Font12x12);
  Display_String(97, 285, "STM32F407", LCD_WHITE);

  Draw_Image(120, 195, 70, 70, img00);

  Set_Font(&Font8x8);
  Display_String(220, 259, "Compiled by YOON", LCD_WHITE);
  Display_String(230, 259, "SSD1289 + XPT2046", LCD_WHITE);
  delayms(5000);
  Number = 70;
  Set_Font(&Font16x24);
  while (Number != 0)
  {
	  Draw_Full_Rect(43, 295 ,61 ,25 , LCD_BLACK);
	  uint16tostr(StrNumber, Number, 10);
	  Display_String(43, 295, StrNumber, LCD_WHITE);
	  //CharCount = sprintf(StrNumber,"%d", Number);
	  //Display_String(43, 295, StrNumber, LCD_WHITE);
	    //TIM_OCInitStructure.TIM_Pulse = Number;
	    //TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	    //TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	  delayms(20);
	  Number--;
  }

  GPIO_ToggleBits(GPIOD, GPIO_Pin_13);

//	Clear_Screen(0x0000);
//	Draw_Image(0, 319, 240, 320, img02);
//	delayms(3000);

  Clear_Screen(0x0000);
  Set_Font(&Font16x24);
  Display_String(107, 255, "Random Lines", LCD_WHITE);

  delayms(2000);
  Clear_Screen(0x0000);
  Random_Lines();
  delayms(500);

  Clear_Screen(0x0000);
  Set_Font(&Font16x24);
  Display_String(107, 295, "Random Rectangles", LCD_WHITE);

  delayms(2000);
  Clear_Screen(0x0000);
  Random_Rect();
  delayms(500);

  Clear_Screen(0x0000);
  Set_Font(&Font16x24);
  Display_String(107, 271, "Random Circles", LCD_WHITE);

  delayms(2000);
  Clear_Screen(0x0000);
  Random_Circle();
  delayms(500);

/*  Clear_Screen(0x0000);
  Set_Font(&Font16x24);
  Display_String(107, 199, "Images", LCD_WHITE);

  delayms(2000);
  Draw_Image(0, 319, 240, 320, img02);
*/
}


/*
 * Draw random lines.
 */

void Random_Lines(void)
{
  uint16_t x1,y1,x2,y2;
  uint32_t i;
  uint16_t cr;

  for(i=0;i<100;i++)
  {
    x1=rand() % 240;  /*TODO: in Eclipse yields rand() error (no reference to _sbrk) */
    y1=rand() % 320;
    x2=rand() % 240;
    y2=rand() % 320;

    cr=rand();

    Draw_Line(x1, y1 ,x2 ,y2 , cr << 3);
    delayms(100);
  }
}

/*
 * Draw random rectangles.
 */

void Random_Rect(void)
{
  uint16_t x1,y1,x2,y2,z;
  uint32_t i;
  uint16_t cr;

  for(i=0;i<25;i++)
  {
    x1=rand() % 240;  /*TODO: in Eclipse yields rand() error (no reference to _sbrk) */
    y1=rand() % 320;
    x2=rand() % 240;
    y2=rand() % 320;

    cr=rand();

    z=rand() % 10;

    if (z >= 5) Draw_Rect(x1, y1 ,x2 ,y2 , cr << 3);
    else Draw_Full_Rect(x1, y1 ,x2 ,y2 , cr << 3);
    delayms(100);
  }
}

/*
 * Draw random circles.
 */

void Random_Circle(void)
{
  uint16_t x, y, r, z;
  uint32_t i;
  uint16_t cr;

  for(i=0;i<25;i++)
  {
    x=rand() % 140;  /*TODO: in Eclipse yields rand() error (no reference to _sbrk) */
    y=rand() % 220;
    r=(rand() % 50) + 1;

    cr=rand() << 3;

    z=rand() % 10;

    if (z >= 5) Draw_Circle(x+50, y+50, r, cr);
    else Draw_Full_Circle(x+50, y+50, r, cr);
    delayms(100);
  }
}

#endif
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
