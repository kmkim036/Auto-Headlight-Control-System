
//usart shim for GPS or others

#include <string.h>
#include <stdarg.h>
#include "yInc.h"

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)|| (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX)) //This has 3 usarts. We here use Two.
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#endif

/* in yInc.h
#define stmUART_RBUF_SIZE 256
struct usartbuf_st{
	unsigned int in; //next in index
	unsigned int out;//next out index
	char prevchar;
	char term; //0x0d/0x0a detected
	char buf[stmUART_RBUF_SIZE];
};
*/
struct usartbuf_st usart1_rbuf={0,0,0,0,};
struct usartbuf_st usart2_rbuf={0,0,0,0,};
struct usartbuf_st usart3_rbuf={0,0,0,0,};
#define USART1_RBUFLEN ((unsigned short)(usart1_rbuf.in - usart1_rbuf.out))
#define USART2_RBUFLEN ((unsigned short)(usart2_rbuf.in - usart2_rbuf.out))
#define USART3_RBUFLEN ((unsigned short)(usart3_rbuf.in - usart3_rbuf.out))

void stmUSART1_buf_Init(void){
	usart1_rbuf.in = 0;
	usart1_rbuf.out = 0;
	usart1_rbuf.prevchar = 0x00;
	usart1_rbuf.term = 0x00;
}
void stmUSART2_buf_Init(void){
	usart2_rbuf.in = 0;
	usart2_rbuf.out = 0;
	usart2_rbuf.prevchar = 0x00;
	usart2_rbuf.term = 0x00;
}
void stmUSART3_buf_Init(void){
	usart3_rbuf.in = 0;
	usart3_rbuf.out = 0;
	usart3_rbuf.prevchar = 0x00;
	usart3_rbuf.term = 0x00;
}

short stmUSART1_GetChar(void){
	struct usartbuf_st *bufp = &usart1_rbuf;
	if(USART1_RBUFLEN == 0)
		return -1;
	return (bufp->buf[(bufp->out++) & (stmUART_RBUF_SIZE-1)]);
}
short stmUSART2_GetChar(void){
	struct usartbuf_st *bufp = &usart2_rbuf;
	if(USART2_RBUFLEN == 0)
		return -1;
	return (bufp->buf[(bufp->out++) & (stmUART_RBUF_SIZE-1)]);
}
short stmUSART3_GetChar(void){
	char retc;
	struct usartbuf_st *bufp = &usart3_rbuf;
	if(USART3_RBUFLEN == 0)
		return -1;
	else
		retc = bufp->buf[(bufp->out++) & (stmUART_RBUF_SIZE-1)];
	return (bufp->buf[(bufp->out++) & (stmUART_RBUF_SIZE-1)]);
}


#define MINMEA_MAX_LENGTH 80
int hex2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}


/**
 * Convert a fixed-point value to a floating-point value.
 * Returns NaN for "unknown" values.
 */
static inline float tofloat(struct minmea_float *f)
{
    if (f->scale == 0)
        return NAN;
    return (float) f->value / (float) f->scale;
}


/* USART1 configured as follow:
	        - BaudRate = given
	        - Word Length = 8 Bits
	        - One Stop Bit
	        - No parity
	        - Hardware flow control disabled (RTS and CTS signals)
	        - Receive and transmit enabled
*/
void stmUSART1ShimConf(uint32_t USART_BaudRate)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;


	  //U1TX -PA9; U1RX-PA10

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))
	  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
	  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);/* Enable GPIO clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Configure USART1 Rx (PA.10) as input floating */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  //Configure USART1
	  //USART1,6=>APB2, Others=>APB1
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	  /* Enable UART clock */
	  USART_InitStructure.USART_BaudRate = USART_BaudRate;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	  USART_Init(USART1, &USART_InitStructure);

	  // Enable USART
	  USART_Cmd(USART1, ENABLE);

	  //Enable its Rx Interrupt mode
	  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
#else
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);/* Enable GPIO clock */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;//GPIO_Speed_50MHz;//Pin Toggle Speed Limit
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);	  /* Connect PXx to USARTx_Rx*/

	  //Configure USART1
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	  /* Enable UART clock */
	  USART_InitStructure.USART_BaudRate = USART_BaudRate;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART1, &USART_InitStructure);

	  // Enable USART
	  USART_Cmd(USART1, ENABLE);

	  //Enable its Rx Interrupt mode
	  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
#endif

}
/*
//Receive Handler
void USART1_IRQHandler(void){
	struct usartbuf_st *bufp;
	char curchar;
	//static unsigned char cnt = 0;//
	// check if the USART1 receive interrupt flag was set
	if(USART_GetITStatus(USART1, USART_IT_RXNE)){ //| USART_IT_ORE_RX
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//clear IRQ //| USART_IT_ORE_RX

		//TEST
		//curchar = USART3->DR; //USART_ReceiveData(USART3) & 0x1ff;
		//printf("%c",curchar);
		//return; //TEST

		bufp = &usart1_rbuf;

		if((bufp->term == 0) && (usart1_rbuf.in < stmUART_RBUF_SIZE)){

			curchar = USART1->DR; //USART_ReceiveData(USART3) & 0x1ff;
			//usart1_rbuf.buf[usart1_rbuf.in] = curchar;//
			bufp->buf[bufp->in & (stmUART_RBUF_SIZE-1)] = curchar;
			if((usart1_rbuf.prevchar == 0x0d) && (curchar == 0x0a)){
				usart1_rbuf.term = 1;
			}else{
				usart1_rbuf.prevchar = curchar;
			}
			usart1_rbuf.in++;
		}else{ //otherwise reset the character counter
			//buffer full...
			stmUSART1_buf_Init();
		}
	}
}
*/
//USART2 ==========================================

void stmUSART2ShimConf(uint32_t USART_BaudRate)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;

	  //USART1,6=>APB2, Others=>APB1
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
	  //USART-A
	  //U2TX -PA2; U2RX-PA3 -- USART2 === APB1
	  /* Configure USART2 Tx (PA2) as alternate function push-pull */

	  // Enable GPIOA clock
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //for usart function
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Configure USART2 Rx (PA.3) as input floating */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  //Need pinRemap for using USART2 on PA2/3 (STM32F103X Specific)
	  //GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); //Removed by YOON (If using pinremap, not working. Why?)

	  //Configure USART2
	  USART_InitStructure.USART_BaudRate = USART_BaudRate;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART2, &USART_InitStructure);

	  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  //Enable its Rx Interrupt mode
	  USART_ITConfig(USART2,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  // Enable USART2
	  USART_Cmd(USART2, ENABLE);
#else
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);/* Enable GPIO clock */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_AF;
	  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;//GPIO_Speed_50MHz;//Pin Toggle Speed Limit
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);	  /* Connect PXx to USARTx_Rx*/

	  //Configure USART2
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	  /* Enable UART clock */
	  USART_InitStructure.USART_BaudRate = USART_BaudRate;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART2, &USART_InitStructure);

	  // Enable USART
	  USART_Cmd(USART2, ENABLE);

	  //Enable its Rx Interrupt mode
	  USART_ITConfig(USART2,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
#endif

}
#if (USART2_FOR == USART2_FOR_GPS)//USART2_FOR_NONE)
//Receive Handler
void USART2_IRQHandler(void){
	struct usartbuf_st *bufp;
	char curchar;
	//static unsigned char cnt = 0;//
	// check if the USART2 receive interrupt flag was set
	if(USART_GetITStatus(USART2, USART_IT_RXNE)){ //| USART_IT_ORE_RX
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//clear IRQ //| USART_IT_ORE_RX
		bufp = &usart2_rbuf;
		if((bufp->term == 0) && (usart2_rbuf.in < stmUART_RBUF_SIZE)){
			curchar = USART2->DR;
			bufp->buf[bufp->in & (stmUART_RBUF_SIZE-1)] = curchar;
			if((usart2_rbuf.prevchar == 0x0d) && (curchar == 0x0a)){
				usart2_rbuf.term = 1;
				//printf("T(%u)\r\n",bufp->in);
			}else{
				usart2_rbuf.prevchar = curchar;
			}
			//printf("%02x ",curchar);
			usart2_rbuf.in++;
		}else{ //otherwise reset the character counter
			//buffer full...
			stmUSART2_buf_Init();
		}
	}
}
#endif
//=======================================
/* USART3 configured as follow:
	        - BaudRate = Given
	        - Word Length = 8 Bits
	        - One Stop Bit
	        - No parity
	        - Hardware flow control disabled (RTS and CTS signals)
	        - Receive and transmit enabled
*/

void stmUSART3ShimConf(uint32_t USART_BaudRate)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
	  //PB11/PB10
	  // PB10(TXD)
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //for usart function
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  // Configure USART3 Rx (PB.11) as input floating
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  //Remap
	  //GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); //Added

	  //Configure USART3

	  USART_InitStructure.USART_BaudRate = USART_BaudRate;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART3, &USART_InitStructure);

	  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  //Enable its Rx Interrupt mode
	  USART_ITConfig(USART3,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  // Enable USART
	  USART_Cmd(USART3, ENABLE);

#else
	  //USART1,6=>APB2, Others=>APB1
	  //PD9,8 -- USART3
	  //Pin6: U3TX - PD8; Pin5: U3RX - PD9

	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);// Enable GPIO clock
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//Pin Toggle Speed Limit
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);	  /* Connect PXx to USARTx_Rx*/

	  //Configure USART3
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	  /* Enable UART clock */
	  USART_InitStructure.USART_BaudRate = USART_BaudRate;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART3, &USART_InitStructure);

	  // Enable USART
	  USART_Cmd(USART3, ENABLE);

	  //Enable its Rx Interrupt mode
	  USART_ITConfig(USART3,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
#endif
}

//Receiver IRQ Handler
void USART3_IRQHandler(void){
	struct usartbuf_st *bufp;
	char curchar;
	//static unsigned char cnt = 0;//
	// check if the USART3 receive interrupt flag was set
	if(USART_GetITStatus(USART3, USART_IT_RXNE)){ //| USART_IT_ORE_RX
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//clear IRQ //| USART_IT_ORE_RX

		//TEST
		//curchar = USART3->DR; //USART_ReceiveData(USART3) & 0x1ff;
		//printf("%c",curchar);
		//return; //TEST

		bufp = &usart3_rbuf;

		if((bufp->term == 0) && (usart3_rbuf.in < stmUART_RBUF_SIZE)){

			curchar = USART3->DR; //USART_ReceiveData(USART3) & 0x1ff;
			//usart3_rbuf.buf[usart3_rbuf.in] = curchar;//
			bufp->buf[bufp->in & (stmUART_RBUF_SIZE-1)] = curchar;
			if((usart3_rbuf.prevchar == 0x0d) && (curchar == 0x0a)){
				usart3_rbuf.term = 1;
			}else if(usart3_rbuf.prevchar == 0x0d){
				usart3_rbuf.term = 1;
			}else{
				usart3_rbuf.prevchar = curchar;
			}
			usart3_rbuf.in++;
		}else{ //otherwise reset the character counter
			//buffer full...
			stmUSART3_buf_Init();
		}
	}
}

void stmUsartTestLoop(){
	int i=0;
	while(1){
		PrintCharBsp('A');
		printf("%d\r\n",i);
		i++;
	}
}
void stmUartLoop(unsigned char uartId, unsigned baud)
{
	int i;
	short len, rxchar;
	char nmeastr[256];
	char rxchar2;


	printf(">Uart Test with Usart%u<\r\n", uartId);
	if(uartId==3)
		stmUSART3ShimConf(baud); //Interrupt Mode. Rx Only
	else if(uartId==2)
		stmUSART2ShimConf(baud); //Interrupt Mode. Rx Only
	else
		stmUSART1ShimConf(baud); //Interrupt Mode. Rx Only

	printf("USART%uConf>%ubps Done.\r\n",uartId, baud);

	//delayms(1000);
/*
	for(i=0;i<10;i++){
	  USART_SendData(USART3, (uint8_t)(0x41+i));
	  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) {}
	}
*/
	while(1) //main loop
    {
		memset(nmeastr,0x00,256);
		if(uartId==3)
			rxchar = stmUSART3_GetChar();
		else
			rxchar = stmUSART1_GetChar();
		if(rxchar >=0){
			printf("%c",(char)rxchar);
		}
	}
}


