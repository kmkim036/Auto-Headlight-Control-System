/*
#include <string.h>
#include <stdarg.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "yInc.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
#include <time.h>
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

extern struct usartbuf_st usart1_rbuf;
extern struct usartbuf_st usart3_rbuf;

/* For STM32F4
 * Using USART3
 * Pin5: U3RX - PD9
 * Pin6: U3TX - PD8
 * Pin34: PPSgps(IN)   - PB14
 * (Pin3: PPSeth(OUT) - PB5)
 *
 *   */

/* The TCM3105 FSK Modem can support upto 1200bps.
 * However, the STM32F407's USART1 is running with 84MHz of PCLK1.
 * This high clock can not support the low 1200bps USART.
 * Option 1) Forced PCLK1 to 48MHz.
 * Option 2) Using USART3 which runs with PCLK2 of 48MHz.
 *
 */

/*
#define RBUF_SIZE 256
struct usartbuf_st{
	unsigned int in; //next in index
	unsigned int out;//next out index
	char prevchar;
	char term; //0x0d/0x0a detected
	char buf[RBUF_SIZE];

};
static struct usartbuf_st usart1_rbuf={0,0,0,0,};
static struct usartbuf_st usart3_rbuf={0,0,0,0,};
#define USART1_RBUFLEN ((unsigned short)(usart1_rbuf.in - usart1_rbuf.out))
#define USART3_RBUFLEN ((unsigned short)(usart3_rbuf.in - usart3_rbuf.out))

void USART1_buf_Init(void){
	usart1_rbuf.in = 0;
	usart1_rbuf.out = 0;
	usart1_rbuf.prevchar = 0x00;
	usart1_rbuf.term = 0x00;
}

void USART3_buf_Init(void){
	usart3_rbuf.in = 0;
	usart3_rbuf.out = 0;
	usart3_rbuf.prevchar = 0x00;
	usart3_rbuf.term = 0x00;
}
*/
/*For GeneralPurpose
void USART3_IRQHandler(void){
	struct usartbuf_st *bufp;
	char curchar;

	if(USART_GetITStatus(USART3, USART_IT_RXNE)!=RESET){
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//clear IRQ

		bufp = &usart3_rbuf;

		if(((bufp->in - bufp->out) & ~(RBUF_SIZE-1)) == 0){
			curchar = USART_ReceiveData(USART3) & 0x1ff;
			bufp->buf[bufp->in & (RBUF_SIZE-1)] = curchar;
			bufp->in++;
		}else{
			//buffer full...
		}
	}
}

short USART1_GetChar(void){
	struct usartbuf_st *bufp = &usart1_rbuf;
	if(USART1_RBUFLEN == 0)
		return -1;
	return (bufp->buf[(bufp->out++) & (RBUF_SIZE-1)]);
}

short USART3_GetChar(void){
	struct usartbuf_st *bufp = &usart3_rbuf;
	if(USART3_RBUFLEN == 0)
		return -1;
	return (bufp->buf[(bufp->out++) & (RBUF_SIZE-1)]);
}
*/



void tcm3105FskModemLoop(unsigned char uartId)
{
	int i;
	short len, rxchar;
	char nmeastr[256];

	printf(">FSK Modem Test with Usart%u<\r\n", uartId);
	if(uartId==3)
		stmUSART3Conf(1200);//9600);
	else
		stmUSART1Conf(1200);

	printf("TCM3105>ModemConf(1200bps).\r\n");

	delayms(1000);
/*
	for(i=0;i<10;i++){
	  USART_SendData(USART3, (uint8_t)(0x41+i));
	  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) {}
	}
*/
	//1Mbps, 8 bit mode
	//stmSPI3_Config(nCS);//(SSI_FRF_MOTO_MODE_0,1000000,8, SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_6); //use PD6 of nCS1
    //nCS7219_1;//nCs=1
	//max7219_Init(1);//2);
	//max7219_config();

	while(1) //main loop
    {
		for(i=0;i<10;i++){
		  USART_SendData(USART1, (uint8_t)(0xFF));//(0x41+i));
		  USART_SendData(USART1, (uint8_t)(0x00));//(0x41+i));
		  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}
		}
		printf("send\r\n");
		delayms(1000);

/*
		rxchar = USART3_GetChar();
		if(rxchar >=0){
			printf("%c",(char)rxchar);
*/
	}
}


