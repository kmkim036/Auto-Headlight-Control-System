//#include "stm32f4xx.h"
//#include "stm32f4xx_syscfg.h"
//#include "stm32f4xx_exti.h"
//#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "stm32f10x_usart.h"
//#include "stm32f10x_flash.h"
#include "yInc.h"

extern void stmConfirmLEDBlink(void);
extern stmUser_LED_GPIO_setup(void);
extern void stmLedToggle(void);
extern uint8_t g_irq;
//+--------+-----------+-----------+-----------+---------+---------+
//|        |401-M34    |401-M35    | 407-M35   | 407-M36 |103      |
//+--------+-----------+-----------+-----------+---------+---------+
//| ULED   | PB14      |PC4        |PE15       | <==     | PC14
//+--------+-----------+-----------+-----------+---------+---------+
//| BUTTON |           |PC5(H)     |PD11(index)| PD11(L) |
//+--------+-----------+-----------+-----------+---------+---------
//| BEEP   |           |PB13       |PD14       | <==     | PC13
//+--------+-----------+-----------+-----------+---------+---------
//| QEI    |           |PB0,1,12   |PD12,13,11 | PD12,13 |
//+--------+-----------+-----------+-----------+---------+-----------
//| SND    |           |PB0,1,12   |PD12,13,11 | PD12,13 | PA8
//+--------+-----------+-----------+-----------+---------+-----------


//*H= Active High/L=Active Low


void stm433MHz_PC13_IRQ_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  /* Enable GPIOC clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE); //We should add AFIO for EXTI
	  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	  /* Enable and set EXTI Line8 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  /* Configure PC13 pin as input floating */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING; //
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
	  /* Connect EXTI Line13 to Pc13 pin */
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

	  /* Configure EXTI Line13 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  printf("433MHz> PC13 IRQ Setup Done.\r\n");

}
unsigned char rxbuf[200];
volatile unsigned char writepos = 0;
unsigned char readpos = 0;
/*
void EXTI15_10_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line13) != RESET) {

		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)){
			rxbuf[writepos] = 1;
		}else{
			rxbuf[writepos] = 0;
		}
		writepos++;
		if(writepos ==100)
			writepos = 0;

		EXTI_ClearITPendingBit(EXTI_Line13);		// Clear the EXTI line 8 pending bit
	}
}
*/
/*
void EXTI15_10_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
		// Toggle LED0
		GPIO_ToggleBits(GPIOC, GPIO_Pin_14); //ULED -103 (PC14)
		g_irq = 1;
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
}
*/

void stm433MHz_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
	//rx
	stm433MHz_PC13_IRQ_Setup();

	//send
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;		  // we want to configure PD11
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 	  // we want it to be an input
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	  GPIO_Init(GPIOA, &GPIO_InitStruct);

	  printf("433MHz> Init Done.\r\n");
#else
#endif
}
void stm433MHz_SendBit(unsigned char b1){
	if(b1) GPIO_SetBits(GPIOA, GPIO_Pin_8);
	else    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	delayms(10);
}

void stm433MHz_SendByte(unsigned char b8){
	int i;
	unsigned char sig;
	//MSB first
	for(i=0;i<8;i++){
		sig = (b8 & 0x80) ? 1 : 0;
		if(sig) GPIO_SetBits(GPIOA, GPIO_Pin_8);
		else    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
		delayms(10);
		b8 = b8 << 1;
		printf("%u",sig);
	}
	printf("\r\n");
}
void stm433MHz_SendBytes(unsigned char b8[], unsigned char length){
	int i;
	for(i=0;i<length;i++){
		printf("%c %02x ",(char)b8[i], b8[i]);
		stm433MHz_SendByte(b8[i]);
	}
}

unsigned char stm433MHz_Readbyte(){
	int i=0;
	unsigned char retb = 0;
	unsigned char bit;
	for(i=0;i<8;i++){
		bit  = rxbuf[readpos];
		if(bit) retb |= 1;
		else	retb |= 0;
		printf("%u",bit);
		retb = retb << 1;
		readpos++;
		if(readpos == 100) readpos = 0;

	}
	printf(" = 0x%02x ", retb);
	return retb;
}
unsigned char stm433MHz_findStartDelimiter(){

}
unsigned char stm433MHz_Readbuf(unsigned char ret_data[]){
	unsigned char num, numbytes;
	unsigned char rxbyte,j;
	if(writepos > readpos)
		num = writepos - readpos;
	else if(writepos < readpos)
		num = (writepos + 100) - readpos;
	else
		num = 0;
	if(num < 8)
		return 0;

	stm433MHz_findStartDelimiter();

	numbytes = num / 8;
	printf("rd=%u, wr=%u, num=%d, numbytes = %d\r\n",readpos, writepos,num, numbytes);
	for(j=0;j<numbytes;j++){
		rxbyte = stm433MHz_Readbyte();
		ret_data[j] = rxbyte;
		stmLedToggle();
	}
	return numbytes;
}
void stm433MHz_Show(unsigned char *str, unsigned char len){
	int i;
	printf("RxFrame = ");
	for(i=0;i<len;i++){
		printf("%02x ", str[i]);
	}
	printf("\r\n");

}
int stm433MHzLoop(void)
{
	int i;
	unsigned char num, rxlen;
	unsigned char sendstr[10];
	unsigned char rxData[100];

	printf("433MHz Loop\r\n");
	stm433MHz_Init();

	while(1){

		//Send
		sendstr[0] = '$'; //sendstr = "Hello";
		sendstr[1] ='A';
		sendstr[2] ='@';
		stm433MHz_SendBytes(sendstr, 3);

		//We need some line coding...
		rxlen = stm433MHz_Readbuf(&rxData[0]);
		if(rxlen){
			stm433MHz_Show(&rxData[0],rxlen);
		}
	}
}
//===================================
extern void stmUSART2_Conf(uint32_t USART_BaudRate);
void stm433MHzWithUsartA_SendBytes(unsigned char sendbuf[], unsigned char leng){
	int i;
	for(i=0;i<leng;i++){
		USART_SendData(USART2, sendbuf[i]);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {}
	}
	printf("send done\r\n");
}
int stm433MHzWithUsartALoop(void)
{
	int i;
	unsigned char num, rxlen;
	unsigned char sendstr[10];
	unsigned char rxData[100];

	short len, rxchar;
	char nmeastr[256];
	char rxchar2;


	printf("433MHz Loop with USART-A\r\n");

	stmUSART2_Conf(300);

	while(1){

		//Send
		sendstr[0] = '$'; //sendstr = "Hello";
		sendstr[1] ='H';
		sendstr[2] ='E';
		sendstr[3] ='L';
		sendstr[4] ='L';
		sendstr[5] ='O';
		sendstr[6] =0x0d;
		sendstr[7] =0x0a;
		stm433MHzWithUsartA_SendBytes(sendstr, 8);

		while(1){
			rxchar = stmUSART2_GetChar();
			if(rxchar >= 0){
				printf("%c ",rxchar);
			}else
				break;
		}
	}

}
