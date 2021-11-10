/*
 * LCD_STM32F4.c
 *
 *  Created on: 11.4.2012
 *      Author: martin
 */

/* Includes ******************************************************************/

#include "LCD_STM32F4.h"

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
