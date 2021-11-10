/*
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "yInc.h"
*/
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
#define LED0 GPIOB, GPIO_Pin_15
#define LED2 GPIOD, GPIO_Pin_13
#define LED3 GPIOD, GPIO_Pin_14
#define LED4 GPIOD, GPIO_Pin_12
#define button GPIOA, GPIO_Pin_0

/* light weight WS2812 lib V2.1 - Arduino support
*
* Controls WS2811/WS2812/WS2812B RGB-LEDs
* Author: Matthias Riegler
*
* Mar 07 2014: Added Arduino and C++ Library
*
* September 6, 2014      Added option to switch between most popular color orders
*                          (RGB, GRB, and BRG) --  Windell H. Oskay
*
* January 24, 2015       Added option to make color orders static again
*                        Moved cRGB to a header so it is easier to replace / expand
*                              (added SetHSV based on code from kasperkamperman.com)
*                                              -- Freezy
*
* License: GNU GPL v2 (see License.txt)
*/

#ifndef F_CPU
#define  F_CPU 16000000UL
#endif

// If you want to use the setColorOrder functions, enable this line
#define RGB_ORDER_ON_RUNTIME

#ifdef RGB_ORDER_ON_RUNTIME
 #define OFFSET_R(r) r+offsetRed
 #define OFFSET_G(g) g+offsetGreen
 #define OFFSET_B(b) b+offsetBlue
#else
// CHANGE YOUR STATIC RGB ORDER HERE
 #define OFFSET_R(r) r+1
 #define OFFSET_G(g) g
 #define OFFSET_B(b) b+2
#endif

struct _ws2812{
	uint16_t count_led;
	uint8_t *pixels;

#ifdef RGB_ORDER_ON_RUNTIME
	uint8_t offsetRed;
	uint8_t offsetGreen;
	uint8_t offsetBlue;
#endif
	uint8_t pinMask;
	 const volatile uint8_t *ws2812_port;
	 volatile uint8_t *ws2812_port_reg;

} g_ws2812;


/*
Control a RGB led with Hue, Saturation and Brightness (HSB / HSV )
Hue is change by an analog input.
Brightness is changed by a fading function.
Saturation stays constant at 255
getRGB() function based on <http://www.codeproject.com/miscctrl/CPicker.asp>
dim_curve idea by Jims
created 05-01-2010 by kasperkamperman.com
*/

/*
dim_curve 'lookup table' to compensate for the nonlinearity of human vision.
Used in the getRGB function on saturation and brightness to make 'dimming' look more natural.
Exponential function used to create values below :
x from 0 - 255 : y = round(pow( 2.0, x+64/40.0) - 1)
*/

// uncomment this line if you use HSV is many projects
// #define USE_HSV
/*
#ifdef USE_HSV
const byte dim_curve[] = {
 0, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6,
 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8,
 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11,
 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15,
 15, 15, 16, 16, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20,
 20, 20, 21, 21, 22, 22, 22, 23, 23, 24, 24, 25, 25, 25, 26, 26,
 27, 27, 28, 28, 29, 29, 30, 30, 31, 32, 32, 33, 33, 34, 35, 35,
 36, 36, 37, 38, 38, 39, 40, 40, 41, 42, 43, 43, 44, 45, 46, 47,
 48, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62,
 63, 64, 65, 66, 68, 69, 70, 71, 73, 74, 75, 76, 78, 79, 81, 82,
 83, 85, 86, 88, 90, 91, 93, 94, 96, 98, 99, 101, 103, 105, 107, 109,
 110, 112, 114, 116, 118, 121, 123, 125, 127, 129, 132, 134, 136, 139, 141, 144,
 146, 149, 151, 154, 157, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 190,
 193, 196, 200, 203, 207, 211, 214, 218, 222, 226, 230, 234, 238, 242, 248, 255,
};
#endif

struct _cRGB {
 uint8_t r;
 uint8_t r;
 uint8_t b;
} g_cRGB;

#ifdef USE_HSV
 void SetHSV(int hue, byte sat, byte val) {
  // convert hue, saturation and brightness ( HSB/HSV ) to RGB
  //The dim_curve is used only on brightness/value and on saturation (inverted).
  //This looks the most natural.


  val = dim_curve[val];
  sat = 255 - dim_curve[255 - sat];

  int base;

  if (sat == 0) {// Acromatic color (gray). Hue doesn't mind.
   r = val;
   g = val;
   b = val;
  }
  else  {
   base = ((255 - sat) * val) >> 8;

   switch (hue / 60) {
   case 0:
    r = val;
    g = (((val - base)*hue) / 60) + base;
    b = base;
    break;

   case 1:
    r = (((val - base)*(60 - (hue % 60))) / 60) + base;
    g = val;
    b = base;
    break;

   case 2:
    r = base;
    g = val;
    b = (((val - base)*(hue % 60)) / 60) + base;
    break;

   case 3:
    r = base;
    g = (((val - base)*(60 - (hue % 60))) / 60) + base;
    b = val;
    break;

   case 4:
    r = (((val - base)*(hue % 60)) / 60) + base;
    g = base;
    b = val;
    break;

   case 5:
    r = val;
    g = base;
    b = (((val - base)*(60 - (hue % 60))) / 60) + base;
    break;
   }
  }
 }
#endif
/*
* light weight WS2812 lib V2.1 - Arduino support
*
* Controls WS2811/WS2812/WS2812B RGB-LEDs
* Author: Matthias Riegler
*
* Mar 07 2014: Added Arduino and C++ Library
*
* September 6, 2014: Added option to switch between most popular color orders
*      (RGB, GRB, and BRG) --  Windell H. Oskay
*
* License: GNU GPL v2 (see License.txt)
*/
/*

stmWs2812_Config(uint16_t num_leds) {
	g_ws2812.count_led = num_leds;

	g_ws2812.pixels = (uint8_t*)malloc(count_led*3);
 #ifdef RGB_ORDER_ON_RUNTIME
 g_ws2812.offsetGreen = 0;
 g_ws2812.offsetRed = 1;
 g_ws2812.offsetBlue = 2;
 #endif
}

struct cRGB  *stmWs2812_get_crgb_at(uint16_t index) {

 struct cRGB px_value;

 if(index < g_ws2812.count_led) {

  uint16_t tmp;
  tmp = index * 3;

  px_value.r = g_ws2812.pixels[OFFSET_R(tmp)];
  px_value.g = g_ws2812.pixels[OFFSET_G(tmp)];
  px_value.b = g_ws2812.pixels[OFFSET_B(tmp)];
 }

 return &px_value;
}

uint8_t stmWs2812set_crgb_at(uint16_t index, struct cRGB px_value) {

 if(index < g_ws2812.count_led) {

  uint16_t tmp;
  tmp = index * 3;

  g_ws2812.pixels[OFFSET_R(tmp)] = px_value.r;
  g_ws2812.pixels[OFFSET_G(tmp)] = px_value.g;
  g_ws2812.pixels[OFFSET_B(tmp)] = px_value.b;
  return 0;
 }
 return 1;
}

uint8_t stmWs2812set_subpixel_at(uint16_t index, uint8_t offset, uint8_t px_value) {
 if (index < g_ws2812.count_led) {
  uint16_t tmp;
  tmp = index * 3;

  g_ws2812.pixels[tmp + offset] = px_value;
  return 0;
 }
 return 1;
}

void stmWs2812sync() {
 *ws2812_port_reg |= g_ws2812.pinMask; // Enable DDR
 ws2812sendarray_mask(g_ws2812.pixels,3*g_ws2812.count_led,g_ws2812.pinMask,(uint8_t*) g_ws2812.ws2812_port,(uint8_t*) g_ws2812.ws2812_port_reg );
}

#ifdef RGB_ORDER_ON_RUNTIME
void stmWs2812setColorOrderGRB() {// Default color order
	g_ws2812.offsetGreen = 0;
	g_ws2812.offsetRed = 1;
	g_ws2812.offsetBlue = 2;
}

void stmWs2812setColorOrderRGB() {
	g_ws2812.offsetRed = 0;
	g_ws2812.offsetGreen = 1;
	g_ws2812.offsetBlue = 2;
}

void stmWs2812setColorOrderBRG() {
	g_ws2812.offsetBlue = 0;
	g_ws2812.offsetRed = 1;
	g_ws2812.offsetGreen = 2;
}
#endif

*/
/*
 * light_ws2812 example
 *
 * Created: 07.03.2014 12:49:30
 *  Author: Matthias Riegler
 */


//WS2812 LED(1); // 1 LED



//DATA(PIN22) - PC13  : STM401-M35
void stmWs2812_GPIO_setup(void){
	GPIO_InitTypeDef GPIO_InitStruct;
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
	//PE15 - KONG-STM32F407
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //STM407VGT6
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); //ZGT6

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;// ZGT6
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	//GPIO_ResetBits(GPIOG, GPIO_Pin_7);// ZGT6
#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //STM401-M34
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14; //STM401-M34
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //STM401-M35
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;//STM401-M35

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
#elif (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
	//PC13 with Buzzer
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //PC13

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);

#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
	//PC6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //PC6

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOC, GPIO_Pin_6);
#endif
}
// '0'   +--0.5usec--+                             +-
//       |           |                             |
//     --+           +----------2.0usec ------- ---+
// '1'   +-----1.2usec--------------+              +-
//       |                          |              |
//     --+                          +---1.3usec  --+
// 'RES'-+                                                   +--
//       |                                                   |
//       +------------------>50usec--------------------------+
void stmWs2812_Zero(void){
#if	(PROCESSOR == PROCESSOR_STM32F103C8T6)
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	//somedelay(1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
#else
	GPIO_SetBits(GPIOC, GPIO_Pin_6);
	//somedelay(1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_6);
#endif
	somedelay(11);
}

void stmWs2812_One(void){
#if	(PROCESSOR == PROCESSOR_STM32F103C8T6)
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	somedelay(5);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
#else
	GPIO_SetBits(GPIOC, GPIO_Pin_6);
	somedelay(5);
	GPIO_ResetBits(GPIOC, GPIO_Pin_6);
#endif
	somedelay(5);
}
void stmWs2812_RES(void){
#if	(PROCESSOR == PROCESSOR_STM32F103C8T6)
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
#else
	GPIO_ResetBits(GPIOC, GPIO_Pin_6);
#endif
	somedelay(500);
}
// MSB (NOT RGB sequence, but GRB sequence is correct)
// +---------------------cycle n-------------------------+                  +------------cycle N
// |R-G-B 24bit(LED0)|R-G-B 24bit(LED1)|R-G-B 24bit(LED2)|     RES          |
// +---------------------cycle n-------------------------+------------------+

unsigned char g_rgb[3];

void stmWs2812SendData(unsigned short numLED, unsigned char rgb[])
{
	int i,j,k;
	unsigned char pixel, pixelbit;
	for(i=0;i<numLED;i++){
		for(j=0;j<3;j++){
			pixel = rgb[j];
			for(k=0;k<8;k++){ //bit
				pixelbit = (pixel & 0x80);
				if(pixelbit)
					stmWs2812_One();
				else
					stmWs2812_Zero();
				pixel << 1;
			}
		}
	}
}

void stmWs2812LEDloop()
{
	int i;
	stmWs2812_GPIO_setup();

	g_rgb[0] = 255; g_rgb[1] = 0; g_rgb[2] = 0; // not RGB Value -> RED, GRB --> GREEN

	while(1){
		g_rgb[0] = 255; g_rgb[1] = 0; g_rgb[2] = 0; // GRB Value -> GREEN
		stmWs2812SendData(1, &g_rgb[0]); // Sends the value to the LED
		stmWs2812_RES();
		delayms(500); // Wait 200 ms

		g_rgb[0] = 0; g_rgb[1] = 255; g_rgb[2] = 0; // GRB Value -> GREEN RED
		stmWs2812SendData(2, &g_rgb[0]); // Sends the value to the LED
		stmWs2812_RES();
		delayms(500); // Wait 200 ms

		g_rgb[0] = 0; g_rgb[1] = 0; g_rgb[2] = 255; // RGB Value -> BLUE
		stmWs2812SendData(3, &g_rgb[0]); // Sends the value to the LED
		stmWs2812_RES();
		delayms(500); // Wait 200 ms

		g_rgb[0] = 255; g_rgb[1] = 255; g_rgb[2] = 255; // RGB Value -> WHITE
		stmWs2812SendData(4, &g_rgb[0]); // Sends the value to the LED
		stmWs2812_RES();
		delayms(500); // Wait 200 ms

//		g_rgb[0] = 0; g_rgb[1] = 0; g_rgb[2] = 0; // RGB Value -> OFF
//		stmWs2812SendData(4, &g_rgb[0]); // Sends the value to the LED
//		stmWs2812_RES();
//		delayms(500); // Wait 200 ms

		g_rgb[0] = 0; g_rgb[1] = 0; g_rgb[2] = 255; // RGB Value -> BLUE
		stmWs2812SendData(5, &g_rgb[0]); // Sends the value to the LED
		stmWs2812_RES();
		delayms(500); // Wait 200 ms

		g_rgb[0] = 0; g_rgb[1] = 0; g_rgb[2] = 255; // RGB Value -> BLUE
		stmWs2812SendData(6, &g_rgb[0]); // Sends the value to the LED
		stmWs2812_RES();
		delayms(500); // Wait 200 ms

//		for(i=7;i<24; i++){
		for(i=7;i<60; i++){
			if(i%2){
				g_rgb[0] = 255; g_rgb[1] = 255; g_rgb[2] = 0; // RGB Value -> BLUE
			}else{
				g_rgb[0] = 255; g_rgb[1] = 0; g_rgb[2] = 0; // RGB Value -> BLUE
			}
			stmWs2812SendData(i, &g_rgb[0]); // Sends the value to the LED
			stmWs2812_RES();
			delayms(500); // Wait 200 ms
		}
		g_rgb[0] = 0; g_rgb[1] = 0; g_rgb[2] = 0; // RGB Value -> OFF
		stmWs2812SendData(i, &g_rgb[0]); // Sends the value to the LED
		stmWs2812_RES();
		delayms(500); // Wait 200 ms


	}
}
/*
int blinkyLoop(void)
{
	stmUser_LED_GPIO_setup();
	//button_setup();

#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
	//PE15 - KONG-STM32F407
	while(1){
			printf("ON  ");
			//GPIO_SetBits(GPIOB, GPIO_Pin_15); // LED1 ON
			GPIO_SetBits(GPIOE, GPIO_Pin_15); 	// STM401. LED1 ON
			//GPIO_SetBits(GPIOE, GPIO_Pin_5); // LED1 ON
			//GPIO_SetBits(GPIOG, GPIO_Pin_7); // LED1 ON -- STM32F407ZGT6
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOE, GPIO_Pin_15);// LED off -- STM401
			//GPIO_ResetBits(GPIOB, GPIO_Pin_15);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//GPIO_ResetBits(GPIOE, GPIO_Pin_5);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			//GPIO_ResetBits(GPIOG, GPIO_Pin_7);// LED1 OFF -- STM32F407ZGT6/LED off
			printf("OFF  ");
			//GPIO_ResetBits(GPIOB, GPIO_Pin_4);// GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15 ); //LED off
			delayms(1000);//somedelay(1000000);
	}
#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
	//PB14 - M34
	//PC4  - M35
	while(1){
			printf("ON  ");
			GPIO_SetBits(GPIOC, GPIO_Pin_4); // LED1 ON
			//GPIO_SetBits(GPIOB, GPIO_Pin_14); 	// STM401. LED1 ON
			delayms(1000);//somedelay(1000000);
			GPIO_ResetBits(GPIOC, GPIO_Pin_4);// LED off -- STM401
			//GPIO_ResetBits(GPIOB, GPIO_Pin_14);// LED off -- STM401
			printf("OFF  ");
			delayms(1000);//somedelay(1000000);
	}
#endif
}
*/
