#include "yInc.h"
#include <string.h>
#include <stdarg.h>

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)|| (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX)) //This has 3 usarts. We here use Two.
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"

//PA9  -- U1TX
//PA10 -- U1RX

//PA2  -- U2TX
//PA3 -- U2RX

//PD8 -- U3TX
//PD9 -- U3RX

void stmUsart1_Init(unsigned int baud) {
	/* USART1 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable USART1 and GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

     /* Enable the USARTx Interrupt */
     //NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
     //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     //NVIC_Init(&NVIC_InitStructure);

    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART1 configuration
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);/* USART configuration */
	USART_Cmd(USART1, ENABLE);/* Enable USART */
}

//PA2 -- U2TX
//PA3 -- U2RX
void stmUsart2_Init(unsigned int baud) {
	/* USART2 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable USART2 and GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
     /* Enable the USARTx Interrupt */
     //NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
     //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     //NVIC_Init(&NVIC_InitStructure);

    /* Configure USART2 Tx (PA.02) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Configure USART2 Rx (PA.3) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

    //USART2 configuration
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);/* USART configuration */
	USART_Cmd(USART2, ENABLE);/* Enable USART */
}


#if (PROCESSOR == PROCESSOR_STM32F103C8T6)
//PB11/PB10
void stmUsart3_Init(unsigned int baud) {

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable USART3 and GPIOD clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

     /* Enable the USARTx Interrupt */
     //NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
     //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     //NVIC_Init(&NVIC_InitStructure);

    // Configure USART3 Tx (PB10) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure USART3 Rx (PB11) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //[NOTE] NO REMAP FOR USART3
    //GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);
    //GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

	/* USART3 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);/* USART configuration */
	USART_Cmd(USART3, ENABLE);/* Enable USART */
}
#elif  (PROCESSOR == PROCESSOR_STM32F103RCT6)
//PA10(RX)/PA9(TX) -- USART1 - debug console
//PC11(RX)/PC10(TX) --USART3 - need remap
void stmUsart3_Init(unsigned int baud) {

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable USART3 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
     /* Enable the USARTx Interrupt */
     //NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
     //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     //NVIC_Init(&NVIC_InitStructure);

    // Configure UART3 Tx (PC10) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Configure UART3 Rx (PC11) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //[NOTE] NO REMAP FOR USART4
    //GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE); //remap to use PC10/11 for USART3

	/* USART3 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	//USART_Init(UART4, &USART_InitStructure);/* USART configuration *///
	USART_Init(USART3, &USART_InitStructure);/* USART configuration */
	//USART_Cmd(UART4, ENABLE);/* Enable USART *///
	USART_Cmd(USART3, ENABLE);/* Enable USART */
}
#else
//PD9/8
void stmUsart3_Init(unsigned int baud) {
	/* USART3 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable USART3 and GPIOD clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

     /* Enable the USARTx Interrupt */
     //NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
     //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     //NVIC_Init(&NVIC_InitStructure);

    /* Configure USART3 Tx (PD8) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    /* Configure USART3 Rx (PD9) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);

    //USART1 configuration
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);/* USART configuration */
	USART_Cmd(USART3, ENABLE);/* Enable USART */
}
#endif

#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
//PA9  -- U1TX
//PA10 -- U1RX
//PA2  -- U2TX
//PA3 -- U2RX
//PA11 -- U6TX
//PA12 -- U6RX
//#ifdef DEBUG_UART1
void stmUsart1_Init(unsigned int baud) {
	/* USART1 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIOAclock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Configure USART1 Tx(PA9) and USART1 Rx(PA10) as alternate functions  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);	/* Connect PXx to USART1_Tx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);/* Connect PXx to USARTx_Rx*/

	//Config USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);/* Enable USART1 clock */
	//USART_OverSampling8Cmd(USART1, ENABLE);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);/* USART configuration */
	USART_Cmd(USART1, ENABLE);/* Enable USART */
}

//M35
//PA2 -- U2TX
//PA3 -- U2RX
void stmUsart2_Init(unsigned int baud) {
	/* USART2 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIOAclock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Configure USART2 Tx(PA2) and USART2 Rx(PA3) as alternate functions  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

	//Config USART2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);/* Enable USART2 clock in APB1 not APB2*/
	//USART_OverSampling8Cmd(USART2, ENABLE);

	USART_InitStructure.USART_BaudRate = baud;// 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);/* USART configuration */
	USART_Cmd(USART2, ENABLE);/* Enable USART */
}

void stmUsart6_Init(unsigned int baud) {
	//uart3=========
	//PA11 = U6TXD
	//PA12 = U6RXD
	/* USART6 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIOAclock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Configure USART6 Tx(PA11) and USART6 Rx(PA12) as alternate functions  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //STM401-M35
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;// GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_USART6);	/* Connect PXx to USART6_Tx*/
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_USART6);/* Connect PXx to USART6_Rx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART6);	/* Connect PXx to USART6_Tx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART6);/* Connect PXx to USART6_Rx*/

	//Config USART6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);/* Enable USART6 clock -- NOTE: USART6 is on the APB2 */

	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART6, &USART_InitStructure);/* USART configuration */
	USART_Cmd(USART6, ENABLE);/* Enable USART */
}
#elif (PROCESSOR == PROCESSOR_STM32F407VGT6)
//PA9 -- U1TX
//PA10 -- U1RX
//#ifdef DEBUG_UART1
void stmUsart1_Init(unsigned int baud) {
	/* USART1 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIOAclock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Configure USART1 Tx(PA9) and USART1 Rx(PA10) as alternate functions  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);	/* Connect PXx to USART1_Tx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);/* Connect PXx to USARTx_Rx*/

	//Config USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);/* Enable USART1 clock */
	//USART_OverSampling8Cmd(USART1, ENABLE);

	USART_InitStructure.USART_BaudRate = baud;//115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);/* USART configuration */
	USART_Cmd(USART1, ENABLE);/* Enable USART */
}
//PD5 -- U2TX
//PD6 -- U2RX
//#ifdef DEBUG_UART1
void stmUsart2_Init(unsigned int baud) {
	/* USART1 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIOAclock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Configure USART2 Tx(PD5) and USART2 Rx(PD6) as alternate functions  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	//Config USART2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);/* Enable USART2 clock in APB1 not APB2*/
	//USART_OverSampling8Cmd(USART2, ENABLE);

	USART_InitStructure.USART_BaudRate = baud;// 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);/* USART configuration */
	USART_Cmd(USART2, ENABLE);/* Enable USART */
}
//#else
/* USART3 configured as follow:
 - BaudRate = 115200 baud
 - Word Length = 8 Bits
 - One Stop Bit
 - No parity
 - Hardware flow control disabled (RTS and CTS signals)
 - Receive and transmit enabled
 */

void stmUsart3_Init(unsigned int baud) { //for KONG
	//uart3=========
	//PD9 = U3RXD
	//PD8 = U3TXD
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// Enable GPIOD clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// Configure USART3 Tx(PD8) and USART3 Rx(PD9) as alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);	// Connect PXx to USART3_Tx
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);// Connect PXx to USAR3Tx_Rx

	//Config USART3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);// Enable USART3 clock
	//USART_OverSampling8Cmd(USART1, ENABLE);

	USART_InitStructure.USART_BaudRate = baud; //115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);// USART configuration
	USART_Cmd(USART3, ENABLE);// Enable USART
}
/*
void stmUsart3_Init(void) {
	//uart3=========
	//PC10 = U3TXD
	//PC11 = U3RXD
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	//Enable GPIOAclock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// Configure USART3 Tx(PC10) and USART3 Rx(PC11) as alternate functions
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);	// Connect PXx to USART3_Tx
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);// Connect PXx to USAR3Tx_Rx

	//Config USART3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);// Enable USART3 clock
	//USART_OverSampling8Cmd(USART1, ENABLE);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);// USART configuration
	USART_Cmd(USART3, ENABLE);// Enable USART
}
*/
//#endif
#elif (PROCESSOR == PROCESSOR_STM32F407VZT6)
//PB11 -- U3RX
//PB10 -- U3TX
void stmUsart3_Init(void) { //for Debug
	/* USART3 configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIOAclock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Configure USART3 Tx(PB10) and USART1 Rx(PB11) as alternate functions  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);/* Connect PXx to USARTx_Rx*/

	//Config USART3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);/* Enable USART3 clock */

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);/* USART configuration */
	USART_Cmd(USART3, ENABLE);/* Enable USART */
}
#endif
void usart_SendWord(uint16_t word) {
	uint8_t b1 = word & 0xFF;
	uint8_t b2 = word >> 8;
	PrintCharBsp(b1);
	PrintCharBsp(b2);
}

void stmUsartSendByte(unsigned usartId, unsigned char c) {
	uint8_t ch;
	ch = c;
	//Place your implementation of fputc here e.g. write a character to the USART
	if (usartId == 1){
		USART_SendData(USART1, (uint8_t) ch);
		// Loop until the end of transmission
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}
	}
	else if (usartId  == 2){
		USART_SendData(USART2, (uint8_t) ch);
		// Loop until the end of transmission
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {}//while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {} //ADDED YOON
	}else if (usartId == 3){
		USART_SendData(USART3, (uint8_t) ch);
		// Loop until the end of transmission
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) {}
	}else if (usartId == 6){
#if(PROCESSOR == PROCESSOR_STM32F407VGT6)
		USART_SendData(USART6, (uint8_t) ch);
		// Loop until the end of transmission
		while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET) {}
#endif
	}
}
//===================
/*
#define MAX_RXBUFLEN 256 // this is the maximum string length of our string in characters
volatile char g_RXBUF[MAX_RXBUFLEN]; // this will hold the recieved string

//void Delay(__IO uint32_t nCount) {  while(nCount--) {  }}
//PD5=U2TX
//PD6=U2RX
void init_USART2(uint32_t baudrate){


	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)


	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; // Pins 5 (TX) and 6 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOD, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers


	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); //
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);


	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting



	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART2, ENABLE);
}
*/

/* This function is used to transmit a string of characters via the USART specified in USARTx.
 *
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */
/*
void USART2_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}

u2TestLoop()
{

  init_USART1(115200);//9600);
  USART_puts(USART1, "Init complete! Hello World!\r\n"); // just send a message to indicate that it works

    while (1){
    }
}
// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART2_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) ){

		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART2->DR; // the character from the USART1 data register is saved in t

		// check if the received character is not the LF character (used to determine end of string) or the if the maximum string length has been been reached
		if( (t != '\n') && (cnt < MAX_STRLEN) ){
			g_RXBUF[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
			USART_puts(USART2, g_RXBUF);
		}
	}
}
*/

