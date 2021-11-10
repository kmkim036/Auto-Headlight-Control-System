//******************************************************************************
// stmDmx512.c - Example program for controlling stmDmx peripherals
//******************************************************************************
/*
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include <stdio.h>
#include <stdint.h>
#include "misc.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "lwip/include/stm32f4x7_eth.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "lwipopts.h"
#include "lwip/include/mem.h"
#include "yInc.h"
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

//extern unsigned char Saa1064_Write4Digit(unsigned char digitL,unsigned char digitR);
//extern void yPWMConfigExp();
//extern void yPWMSetVal(unsigned char duty);
// LM3s8962
// U1TX = PD3
// U1RX = PD2
// RS485Con = PA5        //PA3 (1=TXE; 0=RXE)
unsigned char *gp_stmDmxmsg;
unsigned char stmDmxmsg[512]={
		0x01,
		0x02,
		0x03,
		0x04,
		0x05,
		0xFF
};
// The UART1 interrupt handler for Slave
unsigned char gc_detect_break =0;
unsigned char gc_detect_fe =0;
unsigned char gc_startcode_rcvd =0;
unsigned short gs_index = 0;
unsigned char g_val=0;
unsigned short g_stmDmxAddr = 0;
void stmDmxMasterConfig(unsigned char usartId);
void stmDmxSlaveInit();
void stmDmxSendMessage(unsigned char usartId, unsigned char *p_stmDmxmsg);
extern void stmUser_LED_GPIO_setup(void);
extern void stmUsartSendByte(unsigned usartId, unsigned char c);

void dtstmDmxMasterGpioConf(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	printf("stmDmx512> GPIO Conf. PE15=ULED, PC8=DE \r\n");
	// LED : PE15
	stmUser_LED_GPIO_setup();

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX) ) //PB10 --DE
	//PB10
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //STM103
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);

#else
	//DE : PC8
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //STM407VGT6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_SetBits(GPIOC, GPIO_Pin_8);
#endif
	//Init
	somedelay(100);
}


void stmDmxMasterConfig(unsigned char usartId){
	gp_stmDmxmsg = &stmDmxmsg[0];
	dtstmDmxMasterGpioConf();// DE
	if(usartId == 1)
		stmUsart1_Init(250000); //250Kbps//U1 with 250Kbps
	else if(usartId == 2)
		stmUsart2_Init(250000); //250Kbps//U1 with 250Kbps
	else if(usartId == 3)
		stmUsart3_Init(250000); //250Kbps//U3 with 250Kbps
	GPIO_ResetBits(GPIOC, GPIO_Pin_8); //Driver Enable for TX
}

void stmDmxSendBreakMAB(unsigned char usartId){

	//make Break Signal
	USART_InitTypeDef USART_InitStructure;

	if(usartId == 1){
		//Config USART1
		USART_Cmd(USART1, DISABLE);// Disable USART

		USART_InitStructure.USART_BaudRate = 90000;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_2;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART1, &USART_InitStructure);// USART configuration
		USART_Cmd(USART1, ENABLE);// Enable USART

		stmUsartSendByte(1, 0x00); //Send Break in 100usec(meet for min 88usec)

	}else 	if(usartId == 3){
	//Config USART3
	USART_Cmd(USART3, DISABLE);// Disable USART

	USART_InitStructure.USART_BaudRate = 90000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);// USART configuration
	USART_Cmd(USART3, ENABLE);// Enable USART

	stmUsartSendByte(3, 0x00); //Send Break in 100usec(meet for min 88usec)
	}
	//Set Mark
	somedelay(100); //Emulate Mark -- min 8usec --> The Previous 2 Stop bits + This delay = about 38usec

}

void stmDmxSendMessage(unsigned char usartId, unsigned char *p_stmDmxmsg){
	int i=0;

	printf("stmDmx512>SendMessage\n");

	GPIO_SetBits(GPIOC, GPIO_Pin_8); //DE=1

	stmDmxSendBreakMAB(usartId);
	if( usartId == 1)
		stmUsart1_Init(250000);
	else if( usartId == 3)
		stmUsart3_Init(250000);


	//START code
	//250Kbps
	stmUsartSendByte(usartId, 0x00); //Send START CODE with 0x00
	//data
	for(i=0; i<64; i++){ //for(i=0; i<512; i++){
		stmUsartSendByte(usartId, p_stmDmxmsg[i]);
	}
	GPIO_ResetBits(GPIOC, GPIO_Pin_8); //DE=1

}

void stmDmx512MasterLoop(void)
{
	//Debug USART is 2 in 401 or 2 in 407

	printf("stmDmx512 Master Test with UART1 and RS485 (without interrupt).\r\n");

    stmDmxMasterConfig(1); //USART and DE pins

    while(1){
    	stmDmxSendMessage(1, gp_stmDmxmsg);
    	delayms(1000);
    }
}
/* ===========================================
void dtstmDmxSlaveGpioConf(void)
{
	// LED : PA7
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7); //LED

	// DE : PF0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0); //DE/PF0

	somedelay(100);

}

void stmDmxGetAddrA(){
	g_stmDmxAddr = 0x02;
	printf("stmDmx512Addr = %u\r\n",g_stmDmxAddr);
    Saa1064_Write4Digit(g_stmDmxAddr>>8,g_stmDmxAddr&0xff); //Optional
}
//Get stmDmx address of a special Slave board with DIP switch
void stmDmxGetAddrB(){
	int i;
	unsigned char stmDmxAddrDIP[10];
	unsigned short tmpAddr;
	for(i=0; i<9;i++){
		//CLK
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
		somedelay(1000);
		stmDmxAddrDIP[i]= (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4))>>4;
		tmpAddr = stmDmxAddrDIP[i]<<(8-i);
		g_stmDmxAddr = g_stmDmxAddr + tmpAddr;
		printf("[%d]=%d",i, stmDmxAddrDIP[i]);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
		somedelay(1000);
	}
	printf("stmDmx512Addr = %u\r\n",g_stmDmxAddr);
	Saa1064_Write4Digit(g_stmDmxAddr>>8,g_stmDmxAddr&0xff); //Optional
}

void stmDmxSlaveInit(){ //Rx Only (default)
	//LM3S8962
	// U1TX = PD3
	// U1RX = PD2
	// RS485Con = PA5 // RS485Con = PA3 (1=TXE; 0=RXE)

	unsigned char i=0;
	gp_stmDmxmsg = &stmDmxmsg[0];

	dtstmDmxSlaveGpioConf();

	printf("stmDmx512> Slave Init with Interrput of Uart1.");
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5); // DE
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, ~GPIO_PIN_5); //Receiver Enable for RX

    //OpStatus LED PA6
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //ADDED YOON -- I2C0ÃªÂ´â‚¬Ã«Â¹Â© Ã­â€¢â‚¬Ã¬ï¿½Â´ GPIOBÃ¬Å¡Â©Ã¬Å“Â¼Ã«Â¹ÂºÃ«ï¿½â€ž Ã­â€¢Â Ã«â€¹Â¹Ã«ï¿½ËœÃ¬â€“Â´ Ã¬Å¾Ë†Ã¬Å“Â¼Ã«Â¯â‚¬Ã«Â¹Âº.. (Ã¬â€šÂ¬Ã¬â€¹Â¤ Ã«Â¶Ë†Ã­â€¢â€žÃ¬Å¡â€�?)
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6); //PA6
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
    somedelay(1000000);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, ~GPIO_PIN_6);

    //Get DIP address
    //RESET4917 : PA2
    //4017CLK : PA3
    //stmDmxADDR : PA4

    //RESET(Active HIGH)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2); //PA2
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
    somedelay(1000000);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
    //CLK
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3); //PA3 //clk
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4); //PA4 //addr
    stmDmxGetAddrA();// or stmDmxGetAddrB() (for a special stmDmx slave board)

        //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, ~GPIO_PIN_6);
}

void UART1stmDmx512_IntHandler(void)
{
    unsigned long ulIntStatus;
    unsigned char val;

    IntDisable(INT_UART1); //in hw_ints.h (=22)
    // Get the interrrupt status.
    ulIntStatus = UARTIntStatus(UART1_BASE, true);
    // Clear the asserted interrupts.
    UARTIntClear(UART1_BASE, ulIntStatus);
    UARTRxErrorClear(UART1_BASE); //If using the overrun, framing error, parity error or break interrupts, this function must be called after clearing the interrupt

    if((ulIntStatus & UART_INT_BE) == UART_INT_BE){
    	printf("stmDmx512 SLAVE> BE\n");
    	if(UARTCharsAvail(UART1_BASE))
    	{// Read the this character from the UART.
    		val = UARTCharGetNonBlocking(UART1_BASE);
    	}
    	gc_detect_break = 1;
    	gs_index = 0;
    }else  if((ulIntStatus & UART_INT_FE) == UART_INT_FE){
    	printf("stmDmx512 SLAVE> FE\n");
    	if(UARTCharsAvail(UART1_BASE))
    	{ // Read the this character from the UART.
    		val = UARTCharGetNonBlocking(UART1_BASE);
    	}
    	gc_detect_fe = 1;
    	gs_index = 0;
    }else if((ulIntStatus & UART_INT_OE) == UART_INT_OE){
    	printf("stmDmx512 SLAVE> Get OE Int\n");
    }else if((ulIntStatus & UART_INT_PE) == UART_INT_PE){
    	printf("stmDmx512 SLAVE> Get PE Int\n");
    }else if((ulIntStatus & UART_INT_TX) == UART_INT_TX){
    	printf("stmDmx512 SLAVE> Get TX Int\n");
    }
    else if(ulIntStatus & (UART_INT_RX | UART_INT_RT))
    {
    	while(UARTCharsAvail(UART1_BASE))
    	{
    		// Read the next character from the UART.
    		val = UARTCharGetNonBlocking(UART1_BASE);
    		g_val = val;
    		//printf("val=%x ",val);
    		//if(val == -1) break;
			//if(gc_startcode_rcvd==0){
    		if(gc_detect_fe && gc_detect_break){
				if(gs_index == 0){
    				gc_startcode_rcvd=1;
    				gs_index = 0;
    				gc_detect_fe = 0;
    				gc_detect_break = 0;
    				gp_stmDmxmsg[gs_index++]= val;
    				printf("STCode=%x\r\n",val);
				}
				continue;
			}else{ //already got start code
    			if(gs_index==512){
    				gc_startcode_rcvd=0;
    				//gs_index = 0;
    				printf("Get Char but Over 512\r\n");
    			}
    			else{
    				//if(gs_index<10)
    				//   printf("[%u]=%u\r\n",gs_index, val);
    				if(gs_index < 8)
    				    printf("[%d]=%u\r\n",gs_index,val);
    				gp_stmDmxmsg[gs_index++]= val;
    				if(gs_index==511){ //The Max Last One.
    					 printf("LAST[%u]=%u\r\n",gs_index, val);
        				gc_startcode_rcvd=0;
        				//gs_index = 0;
    				}
    			}
    		}
    	}
    }else
		printf("stmDmx512 SLAVE> Unknown Int %u at %u\r\n", ulIntStatus,gs_index);

 	IntEnable(INT_UART1);
}

void stmDmx512SlaveLoop(void)
{
    // Set the device pinout appropriately for this board.
	printf("\r\nstmDmx512SlaveMain\r\n");
	stmDmx512Uart1PortInit();
    stmDmxSlaveInit();

    // Configure the UART for 250,000, 8-N-2 operation.
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 250000,
	                             (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_TWO |
	                              UART_CONFIG_WLEN_8));
    // Enable the UART1 interrupt.
	IntEnable(INT_UART1);
	UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT | UART_INT_BE | UART_INT_FE);

	yPWMConfigExp();//Optional...
    // Loop forever with UART1stmDmx512_IntHandler()
    while(1)
    {
	}

}

*/
