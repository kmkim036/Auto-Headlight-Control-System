/**
  ******************************************************************************
  * @file    USART/Printf/main.c
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

//#include "stm32f4xx_usart.h"
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */

/*
void PrintChar(char c)
{
	uint8_t ch;
  ch = c;
  // Place your implementation of fputc here
  // e.g. write a character to the USART
  USART_SendData(USART3, (uint8_t) ch);

  // Loop until the end of transmission
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
  {}

}
*/
