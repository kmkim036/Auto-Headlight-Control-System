#ifndef __FONTS_H
#define __FONTS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

typedef struct _tFont
{    
  const uint16_t *table;
  uint16_t Width;
  uint16_t Height;
  
} sFONT;

extern sFONT Font16x24;
extern sFONT Font12x12;
extern sFONT Font8x12;
extern sFONT Font8x8;

#define LINE(x) ((x) * (((sFONT *)LCD_GetFont())->Height))

#endif /* __FONTS_H */
