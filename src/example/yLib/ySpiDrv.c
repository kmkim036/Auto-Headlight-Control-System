/**
  *****************************************************************************
  * @title   ySpiDrv.c
  *******************************************************************************
  STM32F103 : Use SPI2
  STM32F107 : Use SPI3
  */


/**
  ******************************************************************************
  * @file    SPI/SPI_Tansfer/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "yInc.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#endif
#include "misc.h"
#include "cmdline.h"

	  //MODE0 == SCK: SPI_CPOL_0; SDA(SPI_CPHA_1Edge): Valid on SCK Rising(Leading) Edge.
	  //         +--+  +--+  +--+
	  // --------+  +--+  +--+  +-- CLK  (CPOL = 0) //Low in Idle
	  //
	  //      +--^--+     +--^--+   DATA (CPHASE = 0) //Data Latch at the Rising(Leading) SCK Edge
	  //------+     +--^--+

	  //MODE1 == SCK: SPI_CPOL_0; SDA(SPI_CPHA_2Edge): Valid on SCK Falling(Trailing) Edge.
	  //      +--+  +--+  +--+
	  // -----+  +--+  +--+  +--    CLK  (CPOL = 0) //Low in Idle
	  //
	  //      +--v--+     +--v--   DATA  (CPHASE = 1)//Data Latch at the Falling(Trailing) SCK Edge
	  //------+     +--v--+

	  //MODE2 == SCK: SPI_CPOL_1; SDA(SPI_CPHA_1Edge): Valid on SCK Falling Edge.
	  // --------+  +--+  +--+
	  //         +--+  +--+  +--   CLK  (CPOL = 1) //High in Idle
	  //
	  //      +--v--+     +--v--   DATA  (CPHASE = 1) //Data Latch at the Falling(Leading) SCK Edge
	  //------+     +--v--+

	  //MODE3 == SPI_CPOL_1;SDA(SPI_CPHA_2Edge): Valid on SCK Rising Edge.
	  // --------+  +--+  +--+
	  //         +--+  +--+  +-- CLK (CPOL = 1) //Data Latch at the Falling(Trailing) SCK Edge
	  //
	  //         +--^--+     +--- Data (CPHASE = 0)
	  //---------+     +--^--+

//SCK Clock Speed
// APB1 has clock of 1/4 sysclk
// APB2 has clock of 1/2 sysclk

static unsigned char g_Spi1ConfigDoneFlag = 0;
static unsigned char g_Spi2ConfigDoneFlag = 0;
static unsigned char g_Spi3ConfigDoneFlag = 0;
unsigned short Recevie[2];
SPI_InitTypeDef  SPI_InitStructure;

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
/* STM32F103RCT6 uses SPI1
*  CLK -  PA5
*  MISO - PA6
*  MOSI - PA7
*  nCS0 - PC14
*  nCS1 - PC15
*  nCS2 - PC5
*/
//Default SCK = 2.625MHz(= 42MHz/16), it is the lowest rate. You may increase it. Always Master Mode
void stmSPI1_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16)
{
	GPIO_InitTypeDef   GPIO_InitStruct;

	//[NOTE] SPI1 @ APB2
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	//First, we assign ChipSelect Pin.
	  if(nCS==0){
		  //PC14 -nCS0
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 ;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOC, &GPIO_InitStruct);
		  GPIO_SetBits(GPIOC, GPIO_Pin_14);
	  }else if(nCS==1){
		  //--- PC15 -nCS1
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 ;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOC, &GPIO_InitStruct);
		  GPIO_SetBits(GPIOC, GPIO_Pin_15);
	  }else if(nCS==2){
		  //--- PC5 -nCS2
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 ;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOC, &GPIO_InitStruct);
		  GPIO_SetBits(GPIOC, GPIO_Pin_5);
	  }else{
	  }
      //  CLK -  PA5
	  //  MISO - PA6
	  //  MOSI - PA7
	  // Next enable SPI1 Block @ APB2
	if(g_Spi1ConfigDoneFlag == 0)
	{
	  /* Enable SPI1 clocks */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //APB2...Not APB1
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 |GPIO_Pin_7;
  	  GPIO_Init(GPIOA, &GPIO_InitStruct);

  	  SPI_I2S_DeInit(SPI1);

  	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	  if(data8or16 == 8)
  		  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	  else
  		SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;

	  //MODE0 == SCK: SPI_CPOL_0; SDA(SPI_CPHA_1Edge): Valid on SCK Rising(Leading) Edge.
	  //         +--+  +--+  +--+
	  // --------+  +--+  +--+  +-- CLK  (CPOL = 0) //Low in Idle
	  //
	  //      +--^--+     +--^--+   DATA (CPHASE = 0) //Data Latch at the Rising(Leading) SCK Edge
	  //------+     +--^--+

	  //MODE1 == SCK: SPI_CPOL_0; SDA(SPI_CPHA_2Edge): Valid on SCK Falling(Trailing) Edge.
	  //      +--+  +--+  +--+
	  // -----+  +--+  +--+  +--    CLK  (CPOL = 0) //Low in Idle
	  //
	  //      +--v--+     +--v--   DATA  (CPHASE = 1)//Data Latch at the Falling(Trailing) SCK Edge
	  //------+     +--v--+

	  //MODE2 == SCK: SPI_CPOL_1; SDA(SPI_CPHA_1Edge): Valid on SCK Falling Edge.
	  // --------+  +--+  +--+
	  //         +--+  +--+  +--   CLK  (CPOL = 1) //High in Idle
	  //
	  //      +--v--+     +--v--   DATA  (CPHASE = 1) //Data Latch at the Falling(Leading) SCK Edge
	  //------+     +--v--+

	  //MODE3 == SPI_CPOL_1;SDA(SPI_CPHA_2Edge): Valid on SCK Rising Edge.
	  // --------+  +--+  +--+
	  //         +--+  +--+  +-- CLK (CPOL = 1) //Data Latch at the Falling(Trailing) SCK Edge
	  //
	  //         +--^--+     +--- Data (CPHASE = 0)
	  //---------+     +--^--+

	  if(spiMode==0){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //Rising SCK Edge
	  }else if(spiMode==1){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //Falling SCK Edge
	  }else if(spiMode==2){
  		  //MODE 2 -- AD9833
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //Falling SCK Edge
  	  }else if(spiMode==3){
  		  //MODE 3
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //Rising SCK Edge
  	  };

  	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;// | SPI_NSSInternalSoft_Set;;//SPI_NSS_Hard; //PA15 - Hard is not working
  	  if(sckMbps == 10)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //10.05MHz = 42MHz/4
  	  else if(sckMbps == 4)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//5MHz = 42MHz/8. You may increase SCK rate.
  	  else if(sckMbps == 2)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//2.625MHz = 42MHz/16. You may increase SCK rate.
  	  else if(sckMbps == 1)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//1.3MHz = 42MHz/32. You may increase SCK rate.
  	  else
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//1.3MHz = 42MHz/32. You may increase SCK rate.

  	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	  SPI_InitStructure.SPI_CRCPolynomial = 7;
  	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;


  	  SPI_Init(SPI1, &SPI_InitStructure);

  	  SPI_Cmd(SPI1, ENABLE);// Enable the SPI peripheral
	  g_Spi1ConfigDoneFlag = 1;

	  printf("SPI1 Config Done\r\n");
	}
}

/* STM32F103C8T6 uses SPI2
*  CLK -  PB13
*  MISO - PB14
*  MOSI - PB15
*  nCS0 - PB12
*  nCS1 - PB5
*  nCS2 - PA7
 */
//Default SCK = 2.625MHz(= 42MHz/16), it is the lowest rate. You may increase it. Always Master Mode
void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16)
{
	GPIO_InitTypeDef   GPIO_InitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	//First, we assign ChipSelect Pin.
	  if(nCS==0){
		  //PB12 -nCS0
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 ;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOB, &GPIO_InitStruct);
		  GPIO_SetBits(GPIOB, GPIO_Pin_12); //GPIOB->BSRRL |= GPIO_Pin_12; // set  high
	  }else if(nCS==1){
		  //--- PB5 -nCS1
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 ;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOB, &GPIO_InitStruct);
		  GPIO_SetBits(GPIOB, GPIO_Pin_5); //GPIOB->BSRRL |= GPIO_Pin_5; // set  high
	  }else if(nCS==2){
		  //--- PA7 -nCS2
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 ;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOA, &GPIO_InitStruct);
		  GPIO_SetBits(GPIOA, GPIO_Pin_7); //GPIOA->BSRRL |= GPIO_Pin_7; // set  high
	  }
	  //  CLK -  PB13
	  //  MISO - PB14
	  //  MOSI - PB15
	  // Next enable SPI2 Block
	if(g_Spi2ConfigDoneFlag == 0){
	  /* Enable SPI2 @APB1(36MHz) clocks */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
  	  GPIO_Init(GPIOB, &GPIO_InitStruct);

  	  SPI_I2S_DeInit(SPI2);

  	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	  if(data8or16 == 8)
  		  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	  else
  		SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;

	  if(spiMode==0){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //Rising SCK Edge
	  }else if(spiMode==1){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //Falling SCK Edge
	  }else if(spiMode==2){
  		  //MODE 2 -- AD9833
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //Falling SCK Edge
  	  }else if(spiMode==3){
  		  //MODE 3
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //Rising SCK Edge
  	  };

  	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;// | SPI_NSSInternalSoft_Set;;//SPI_NSS_Hard; //PA15 - Hard is not working

  	  if(sckMbps == 10)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //9MHz = 36MHz/4
  	  else if(sckMbps == 5)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//5.3MHz = 36MHz/8.
  	  else if(sckMbps == 2)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//2.625MHz = 36MHz/16.
  	  else if(sckMbps == 1)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//1.1MHz = 36MHz/32.
  	  else if(sckMbps == 100) //actually 100Kbps
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//140.625Kbps = 36MHz/256.
  	  else
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//1.3MHz = 36MHz/32. You may increase SCK rate.

  	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	  SPI_InitStructure.SPI_CRCPolynomial = 7;
  	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

  	  SPI_Init(SPI2, &SPI_InitStructure);

  	  SPI_Cmd(SPI2, ENABLE);// Enable the SPI peripheral
	  g_Spi2ConfigDoneFlag = 1;
	}
}
void stmSPI3_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16)
{
	printf("Not YET\r\n");
}
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
void stmSPI1_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16)
{
	printf("Not Support\r\n");
}
void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16)
{
	printf("Not Support\r\n");
}
/** @brief  Configures the SPI3 Peripheral.
 *  CLK  - PC10
 *  MISO - PC11
 *  MOSI - PC12
 *  nCS0 - PA15
 *  nCS1 - PD15
 *  nCS2 - PE5
  */
void stmSPI3_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16)
{
	GPIO_InitTypeDef   GPIO_InitStruct;

	//First, we assign ChipSelect Pin.
	if(nCS==0){ //PA15 -nCS0
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
		  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PA15 Remap /*!< JTAG-DP Disabled and SW-DP Enabled */
		  //[VERY IMPORTANT] PA15's default function is JTDI.
		  //Thus we should remap to GPIO as the above.
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 ;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOA, &GPIO_InitStruct);

		  GPIO_SetBits(GPIOA, GPIO_Pin_15); //GPIOA->BSRRL |= GPIO_Pin_15; // set  high
	  }else if(nCS==1){ //--- PD15 -nCS1
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 ;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOD, &GPIO_InitStruct);
		  GPIO_SetBits(GPIOD, GPIO_Pin_5); //GPIOD->BSRRL |= GPIO_Pin_5; // set  high
	  }else if(nCS==2){ //--- PE5 -nCS2
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 ;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOE, &GPIO_InitStruct);
		  GPIO_SetBits(GPIOE, GPIO_Pin_5); //GPIOE->BSRRL |= GPIO_Pin_5; // set  high
	  }

	// Next enable SPI3 Block
	// CLK  - PC10
	// MISO - PC11
	// MOSI - PC12
	if(g_Spi3ConfigDoneFlag == 0){
	    // Enable GPIOC with Alternate Function for SPI.
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE); //In addition,

		//
		GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF_PP;
		GPIO_InitStruct.GPIO_Speed 	= GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Pin 	= GPIO_Pin_10 | GPIO_Pin_11 |GPIO_Pin_12; //PC10,11,12
		GPIO_Init(GPIOC, &GPIO_InitStruct);

		//Enable SPI3 clocks
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

		//Config SPI3 - FDX, {8,16}, SPImode, Speed
		SPI_I2S_DeInit(SPI3);

		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;// | SPI_NSSInternalSoft_Set;//SPI_NSS_Hard; //PA15
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

		if(data8or16 == 8)
			SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		else
			SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;

		if(spiMode==0){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 	//POL=0; Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 	//PHA=0; Rising SCK Edge
		}else if(spiMode==1){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 	//POL=0;Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; 	//PHA=1;Falling SCK Edge
		}else if(spiMode==2){
  		  //MODE 2 -- AD9833
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; 	//High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; 	//Falling SCK Edge
		}else if(spiMode==3){
  		  //MODE 3
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; 	//High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 	//Rising SCK Edge
		};
		//SCK rate
		if(sckMbps == 10)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //10.05MHz = 42MHz/4
		else if(sckMbps == 2)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//2.625MHz = 42MHz/16. You may increase SCK rate.
		else if(sckMbps == 1)
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//1.3MHz = 42MHz/32. You may increase SCK rate.
		else
  		  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//1.3MHz = 42MHz/32. You may increase SCK rate.

		SPI_Init(SPI3, &SPI_InitStructure);

		SPI_Cmd(SPI3, ENABLE);// Enable the SPI peripheral

		g_Spi3ConfigDoneFlag = 1;
	}else
		printf("Already SPI3 configured\r\n");

}
#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
/** @brief  Configures the SPI1 Peripheral.
 * for STM32F401RET-M35
 *  CLK -  PA5
 *  MISO - PA6
 *  MOSI - PA7
 *  nCS0 - PA15
 *  nCS1 - PB7
 *  nCS2 - PB6
  */

//Default SCK = 2.625MHz(= 42MHz/16), it is the lowest rate. You may increase it.
void stmSPI1_Config(unsigned sckMbps,unsigned char nCS, unsigned short spiMode, unsigned char data8or16)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//First, we assign ChipSelect Pin.
	  if(nCS==0){
		  //PA15 -nCS0
		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//GPIO_Mode_AF;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		  GPIO_Init(GPIOA, &GPIO_InitStructure);
		  GPIOA->BSRRL |= GPIO_Pin_15; // set  high
	  }else if(nCS==1){
		  //--- PB7 -nCS1
		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//GPIO_Mode_AF;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		  GPIO_Init(GPIOB, &GPIO_InitStructure);
		  GPIOB->BSRRL |= GPIO_Pin_7; // set PB7 high
	  }else if(nCS==2){
		  //PB6 -nCS2
		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//GPIO_Mode_AF;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		  GPIO_Init(GPIOB, &GPIO_InitStructure);
		  GPIOB->BSRRL |= GPIO_Pin_6; // set high
	  }
	  // CLK -  PA5
	  // MISO - PA6
	  // MOSI - PA7
	  // Next enable SPI1 Block
	if(g_Spi1ConfigDoneFlag == 0){
	  /* Enable GPIO clocks */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 |GPIO_Pin_7;
  	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); //CLK - PA5
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); //MISO - PA6
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); //MOSI - PA7
	  g_Spi1ConfigDoneFlag = 1;

	  /* Enable the SPI1 clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //NOTE SPI1 is handle on APB2

  	  SPI_I2S_DeInit(SPI1);

  	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	  if(data8or16 == 8)
  		  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	  else
  		SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;


	  if(spiMode==0){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //Rising SCK Edge
	  }else if(spiMode==1){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //Falling SCK Edge
	  }else if(spiMode==2){
  		  //MODE 2 -- AD9833
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //Falling SCK Edge
  	  }else if(spiMode==3){
  		  //MODE 3
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //Rising SCK Edge
  	  };

  	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;// | SPI_NSSInternalSoft_Set;;//SPI_NSS_Hard; //PA15 - Hard is not working
  	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //10.05MHz = 42MHz/4
  	  //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//2.625MHz = 42MHz/16. You may increase SCK rate.
  	  //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//1.3MHz = 42MHz/32. You may increase SCK rate.
  	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	  SPI_InitStructure.SPI_CRCPolynomial = 7;
  	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

  	  SPI_Init(SPI1, &SPI_InitStructure);
  	  // Enable the SPI peripheral
  	  SPI_Cmd(SPI1, ENABLE);
	}
}
//============= SPI2 for KONGv2 ====================================
/** @brief  Configures the SPI2 Peripheral.
 *  CLK - PB10
 *  MISO - PB14
 *  MOSI - PB15
 *  nCS0 - PD10
 *  nCS1 - PD15
 *  nCS2 - PE5
  */

//Default SCK = 2.625MHz(= 42MHz/16), it is the lowest rate. You may increase it.
void stmSPI2_Config(unsigned sckMbps,unsigned char nCS, unsigned short spiMode, unsigned char data8or16)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//First, we assign ChipSelect Pin.
	  if(nCS==0){
		  //PD10 -nCS0
		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//GPIO_Mode_AF;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		  GPIO_Init(GPIOD, &GPIO_InitStructure);
		  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI3); //nCS - PA15 -- NOT WORKING
		  GPIOD->BSRRL |= GPIO_Pin_10; // set PD10 (CS0) high
	  }else if(nCS==1){
		  //--- PD15 -nCS1
		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//GPIO_Mode_AF;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		  GPIO_Init(GPIOD, &GPIO_InitStructure);
		  GPIOD->BSRRL |= GPIO_Pin_15; // set PD15 (CS1) high
	  }else if(nCS==2){
		  //PED5 -nCS2
		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//GPIO_Mode_AF;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		  GPIO_Init(GPIOE, &GPIO_InitStructure);
		  GPIOE->BSRRL |= GPIO_Pin_5; // set PE5 (CS2) high
	  }
	  // *  CLK - PB10
	  // *  MISO - PB14
	  // *  MOSI - PB15
	  //Next enable SPI2 Block
	if(g_Spi2ConfigDoneFlag == 0){
	  /* Enable GPIO clocks */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//DOWN;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_14 |GPIO_Pin_15;
  	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2); //CLK - PB10
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2); //MISO - PB14
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2); //MOSI - PB15
	  g_Spi2ConfigDoneFlag = 1;

	  /* Enable the SPI2 clock */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  	  SPI_I2S_DeInit(SPI2);

  	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	  if(data8or16 == 8)
  		  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	  else
  		SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;

	  if(spiMode==0){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //Rising SCK Edge
	  }else if(spiMode==1){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //Falling SCK Edge
	  }else if(spiMode==2){
  		  //MODE 2 -- AD9833
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //Falling SCK Edge
  	  }else if(spiMode==3){
  		  //MODE 3
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //Rising SCK Edge
  	  };

  	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;// | SPI_NSSInternalSoft_Set;;//SPI_NSS_Hard; //PA15 - Hard is not working
  	  //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//2.625MHz = 42MHz/16. You may increase SCK rate.
  	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	  SPI_InitStructure.SPI_CRCPolynomial = 7;
  	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

  	  SPI_Init(SPI2, &SPI_InitStructure);
  	  // Enable the SPI peripheral
  	  SPI_Cmd(SPI2, ENABLE);
	}
}



#endif //PROCESSOR

//Need nCS
//was void
unsigned short stmSpi1WrByte(unsigned char inbyte){ //send cmd or data byte
	unsigned short rxs;

     while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
     SPI_I2S_SendData(SPI1, inbyte);
     while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
     rxs = SPI_I2S_ReceiveData(SPI1); //dummy read

     return rxs; //dummy in general, but valid for MCP3208
}

//Need nCS
unsigned char stmSpi1RdByte(){
	unsigned char retVal;
	unsigned short dummy=0x0000;

	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
    SPI_I2S_SendData(SPI1, dummy);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
    retVal = SPI_I2S_ReceiveData(SPI1);
    return(retVal);
}

unsigned short stmSpi1WrShort(unsigned short inData){ //send cmd or data byte
	unsigned short rxs;

     while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
     SPI_I2S_SendData(SPI1, inData);
     while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
     rxs = SPI_I2S_ReceiveData(SPI1); //dummy read

     return rxs; //dummy in general, but valid for MCP3208
}
//============== SPI2 =============================
//Need nCS
unsigned short stmSpi2WrByte(unsigned char inbyte){ //send cmd or data byte
	unsigned short rxs;

     while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
     SPI_I2S_SendData(SPI2, inbyte);
     while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
     rxs = SPI_I2S_ReceiveData(SPI2); //dummy read

     return rxs; //dummy in general, but valid for MCP3208
}
//Need nCS
unsigned char stmSpi2RdByte(){
	unsigned char retVal;
	unsigned short dummy=0x0000;

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
    SPI_I2S_SendData(SPI2, dummy);
    while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
    retVal = SPI_I2S_ReceiveData(SPI2);
    return(retVal);
}

unsigned short stmSpi2WrShort(unsigned short inData){ //send cmd or data byte
	unsigned short rxs;

     while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
     SPI_I2S_SendData(SPI2, inData);
     while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
     rxs = SPI_I2S_ReceiveData(SPI2); //dummy read

     return rxs; //dummy in general, but valid for MCP3208
}
//============== SPI3 =============================
//Need nCS
unsigned short stmSpi3WrByte(unsigned char inbyte){
	unsigned short rxs;
     while(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
     SPI_I2S_SendData(SPI3, inbyte);
     while(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE));
     rxs = SPI_I2S_ReceiveData(SPI3); //dummy read
     return rxs; //dummy in general, but valid for MCP3208
}
//Need nCS
unsigned char stmSpi3RdByte(){
	unsigned char retVal;
	unsigned short dummy=0x0000;

	while(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
    SPI_I2S_SendData(SPI3, dummy);
    while(!SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE));
    retVal = SPI_I2S_ReceiveData(SPI3);
    return(retVal);
}

//=============== spi2 or spi3 ==============
unsigned short stmSpiWrByte(unsigned char inbyte){
	unsigned short rxs;
	if(g_Spi1ConfigDoneFlag){
		rxs = stmSpi1WrByte(inbyte);
	}else if(g_Spi2ConfigDoneFlag)
		rxs = stmSpi2WrByte(inbyte);
	else if(g_Spi3ConfigDoneFlag)
		rxs = stmSpi3WrByte(inbyte);
	else {
		printf("Err:stmSpiWrByte\r\n");
	}
	return rxs;
}
unsigned char stmSpiRdByte(){
	unsigned char retVal;
	if(g_Spi1ConfigDoneFlag)
		retVal = stmSpi1RdByte();
	else if(g_Spi2ConfigDoneFlag)
		retVal = stmSpi2RdByte();
	else if(g_Spi3ConfigDoneFlag)
		retVal = stmSpi3RdByte();
	else
		printf("Err:stmSpiRdByte\r\n");
	return retVal;
}
void stmSPI_Config(unsigned char whichSPI, unsigned char nCS, unsigned short spiMode, unsigned char data8or16)
{

	if(whichSPI == USE_SPI1){
		stmSPI1_Config(2, nCS, spiMode, data8or16);
		g_Spi1ConfigDoneFlag = 1;
	}else if(whichSPI == USE_SPI2){
		stmSPI2_Config(2, nCS, spiMode, data8or16); //2Mbps.//Default SCK = 2.625MHz(= 42MHz/16), it is the lowest rate. You may increase it.
		g_Spi2ConfigDoneFlag = 1;
	}else if(whichSPI == USE_SPI3){
		stmSPI3_Config(2,nCS, spiMode, data8or16);
		g_Spi3ConfigDoneFlag = 1;
	}
}

/*

//Need nCS
//was void
unsigned short stmSpi1WrByte(unsigned char inbyte){ //send cmd or data byte
	unsigned short dummy;

     while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
     SPI_I2S_SendData(SPI1, inbyte);
     while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
     dummy = SPI_I2S_ReceiveData(SPI1); //dummy read

     return dummy; //dummy in general, but valid for MCP3208
}
//Need nCS
unsigned char stmSpi1RdByte(){
	unsigned char retVal;
	unsigned short dummy=0x0000;

	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
    SPI_I2S_SendData(SPI1, dummy);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
    retVal = SPI_I2S_ReceiveData(SPI1);
    return(retVal);
}

unsigned short stmSpi1WrShort(unsigned short inData){ //send cmd or data byte
	unsigned short dummy;

     while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); //Transmit buffer empty flag.
     SPI_I2S_SendData(SPI1, inData);
     while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
     dummy = SPI_I2S_ReceiveData(SPI1); //dummy read

     return dummy; //dummy in general, but valid for MCP3208
}
*/
