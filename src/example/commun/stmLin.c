// UART1 ÃƒÂ¬Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¬Ã…Â¡Ã‚Â©
#if 1

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
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT6)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"
#include "yLin.h"
#include "yMotor.h"
//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |  STM103M35|103-M37/M39|M70        |401
//+-----------------+-----------+-----------+-----------+---------------+------------
//| USART           | USART2    |    <--    |           |USART1
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| TXD(Pin12)      | PA2       |    <--    |
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| RXD(Pin13)      | PA3       |    <--    |           |
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| LINEN(Pin3)     | PB10      |    <--    |           |PC13
//+-----------------+-----------+-----------+-----------+---------------+---------+
/*
  USART LIN Master transmitter communication is possible through the following procedure:
     1. Program the Baud rate, Word length = 8bits, Stop bits = 1bit, Parity,
        Mode transmitter or Mode receiver and hardware flow control values using
        the USART_Init() function.
     2. Enable the USART using the USART_Cmd() function.
     3. Enable the LIN mode using the USART_LINCmd() function.
     4. Send the break character using USART_SendBreak() function.

  USART LIN Master receiver communication is possible through the following procedure:
     1. Program the Baud rate, Word length = 8bits, Stop bits = 1bit, Parity,
        Mode transmitter or Mode receiver and hardware flow control values using
        the USART_Init() function.
     2. Enable the USART using the USART_Cmd() function.
     3. Configures the break detection length using the USART_LINBreakDetectLengthConfig()
        function.
     4. Enable the LIN mode using the USART_LINCmd() function.


@note In LIN mode, the following bits must be kept cleared:
        - CLKEN in the USART_CR2 register,
        - STOP[1:0], SCEN, HDSEL and IREN in the USART_CR3 register.
*/

//Using MC33662 LIN Transceiver


// We employ 5 different LIN msgs.
// MsgID_0_2A 0b00000000 = Write for controlling Motor direction and speed
// MsgID_0_2B 0b01010000 = for reading 2 bytes
// MsgID_0_4  0b10100000 = for sending 4 bytes
// MsgID_0_8  0b11110000 = for reading 8 bytes
// MsgID_3c_8 0x3c       = Broadcast with 8 bytes data.

// Master Schedule
// MsgID_0_2A -> MsgID_0_2A -> MsgID_0_2A -> MsgID_0_2A -> MsgID_0_4 -> MsgID_0_8
//

 //#define LIN_ROLE LIN_ROLE_MASTER //LIN_ROLE_SLAVE //SLAVE

struct _yLIN_module g_LinMod;

unsigned char stmLinCksum(unsigned char lindatabyte, unsigned char stmLinCksum){
	unsigned short sum;
	sum = stmLinCksum + lindatabyte;
	sum = (sum & 0xff) + ((sum & 0xff00)>>8);
	sum = (sum & 0xff) + ((sum & 0xff00)>>8);
	return (unsigned char)(sum & 0xff);
}

unsigned char stmGetLinCksum(unsigned char lindatabyte[]){
	unsigned short cksum = 0;
	int i;
	for(i=0;i<8;i++){
		cksum += lindatabyte[i];
		if(cksum >= 256)
			cksum -= 255;
	}
	return (0xff &(~cksum));
	//return (unsigned char)(sum & 0xff);
}
void stmLinSendHeaderByMaster (struct _yLIN_module *linMod, unsigned char linMsgId){ //By Master

	USART_SendBreak(LIN_USART);//Send Break Signal
	while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}
	//delayms(1); //Needed?
	//UARTBreakCtl(UART1_BASE,false); //Stop Break Signal -- NOT NEED

	//Send Sync Byte (0x55)
	linMod->LinRxBuf.sync_rcvd = 0;
	USART_SendData(LIN_USART, 0x55);
	while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}

	//Send Msg id
	USART_SendData(LIN_USART, linMsgId);
	while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}
}

void stmLinSendData(unsigned char *p_lindata, unsigned char leng){ //By Master and Slave
	int i=0;
	unsigned char cksum = 0;

	for(i=0;i<leng;i++){
		stmLinCksum(p_lindata[i], cksum); //calc during response data transmission.
		while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}
		USART_SendData(LIN_USART, p_lindata[i]);
	}
	//After sending the whole data, and finally append Checksum
	while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}
	USART_SendData(LIN_USART, (uint8_t) cksum);


}

//By Master for reading or writing
void stmLinSendMsgByMaster(
		struct _yLIN_module *linMod,
		struct LinMsgInfo *plinMsgInfo,
		unsigned char msgindex,
		u8 *txD)
{
	plinMsgInfo->msgIndex = msgindex;
	stmLinSendHeaderByMaster(linMod, plinMsgInfo->msgid); //send header and control data by this master
	if((txD != NULL) && (!plinMsgInfo->needResp)) //for writing by master or responding by slave.
		stmLinSendData(txD, plinMsgInfo->dleng);

	if(linMod->role == LIN_ROLE_MASTER)
		printf("M>Sent(id=0x%02x)\r\n", plinMsgInfo->msgid);
	else
		printf("S>Sent(id=0x%02x)\r\n", plinMsgInfo->msgid);


}

u8 stmLinWaitForReponse(struct _yLIN_module *linMod){
	static u32 timeout;
	//printf("\r\nLinWaitForReponse()..\r\n");
	timeout = 0;
	while((!linMod->LinRxBuf.rcvd) && (timeout < LINMAXTIMEOUT)){
		delayms(1);
		timeout++;
	}

	if  (timeout < LINMAXTIMEOUT){
		printf("M>Got a response\r\n");
		timeout = 0;
		return 1;
	}else{
		//printf("\r\n===============No response\r\n");
		timeout = 0;
		return 0;
	}
}

void stmLinMaster_MsgAppHandler(struct _yLIN_module *linMod){
	u8 i;
	printf("M>RxDone. Need more detailed parsing...\r\n");
	for(i=0;i<linMod->LinRxBuf.msgInfo.dleng;i++){
			printf("[%d]=0x%02x ", i, linMod->LinRxBuf.data[i]);
	}
	printf("\r\n");
	linMod->LinRxBuf.rcvd = 0;
}

void stmLin_ConfigMsgInfoPool( struct _yLIN_module *linMod)
{
	//0
	linMod->LinMsgInfoPool[0].msgIndex = 0;
	linMod->LinMsgInfoPool[0].msgid = MsgID_0_2A;
	linMod->LinMsgInfoPool[0].needResp = 0;
	linMod->LinMsgInfoPool[0].dleng = 2;
	//1
	linMod->LinMsgInfoPool[1].msgIndex = 1;
	linMod->LinMsgInfoPool[1].msgid = MsgID_0_2B;
	linMod->LinMsgInfoPool[1].needResp = 1;
	linMod->LinMsgInfoPool[1].dleng = 2;
	//2
	linMod->LinMsgInfoPool[2].msgIndex = 2;
	linMod->LinMsgInfoPool[2].msgid = MsgID_0_4;
	linMod->LinMsgInfoPool[2].needResp = 0;
	linMod->LinMsgInfoPool[2].dleng = 4;
	//3
	linMod->LinMsgInfoPool[0].msgIndex = 3;
	linMod->LinMsgInfoPool[0].msgid = MsgID_0_8;
	linMod->LinMsgInfoPool[0].needResp = 1;
	linMod->LinMsgInfoPool[0].dleng = 8;
	//4
	linMod->LinMsgInfoPool[0].msgIndex = 4;
	linMod->LinMsgInfoPool[0].msgid = MsgID_3c_8;
	linMod->LinMsgInfoPool[0].needResp = 0;
	linMod->LinMsgInfoPool[0].dleng = 8;
}

//MASTER TASK
int stmLIN_MasterTask (struct _yLIN_module *linMod){
	unsigned char msgid, controldata;
	u8 txD[8];
	u8 i;
	u32 cnt;
	struct LinMsgInfo *plinMsgInfo;

	//A cycle consists of
	// | CW - 1000ms - CCW - 1000ms - UP - 1000ms - DN - 1000ms - Read(2) -- 1000ms  -- Read(8) - 1000ms - Sleep -- 5000ms - Wakeup |
	while(1){
		//Schedule timeslot 0 : rotate motor CW
		txD[0]=MOTOR_CMD_ROTATE_CW;
		txD[1]=0x01; //any data..
		plinMsgInfo = &linMod->LinMsgInfoPool[LinMsg0]; //no response, datalen =2
		stmLinSendMsgByMaster(linMod, plinMsgInfo, LinMsg0,txD);
		delayms(1000);

		//Schedule timeslot 1 : rotate motor CCW
		txD[0]=MOTOR_CMD_ROTATE_CCW;
		txD[1]=0x02; //any data..
		plinMsgInfo = &linMod->LinMsgInfoPool[LinMsg0];//no response, datalen =2
		stmLinSendMsgByMaster (linMod, plinMsgInfo,LinMsg0, txD); //send header and control data by this master
		delayms(1000);

		//Schedule timeslot 2 ; speedup
		txD[0]=MotorSpeedUP;
		txD[1]=0x03;
		plinMsgInfo = &linMod->LinMsgInfoPool[LinMsg0];//no response, datalen =2
		stmLinSendMsgByMaster (linMod, plinMsgInfo,LinMsg0, txD); //send header and control data by this master
		delayms(1000);

		//Schedule timeslot 3 : slow down
		txD[0]=MotorSpeedDOWN;
		txD[1]=0x04;
		plinMsgInfo = &linMod->LinMsgInfoPool[LinMsg0];//no response, datalen =2
		stmLinSendMsgByMaster(linMod, plinMsgInfo,LinMsg0,txD); //send header and control data by this master
		delayms(1000);

		//Schedule timeslot 4 -- Request and Handle Response
		plinMsgInfo = &linMod->LinMsgInfoPool[LinMsg1];
		stmLinSendMsgByMaster (linMod, plinMsgInfo,LinMsg1, 0); //need response, datalen =2
		if(!stmLinWaitForReponse(linMod)){//Wait for Response from the slave.
			printf("NoResp\r\n");
		}else{
			stmLinMaster_MsgAppHandler(linMod);
		}
		delayms(1000);

		//Schedule timeslot 5 -- Requst and Handle Response
		plinMsgInfo = &linMod->LinMsgInfoPool[LinMsg3];     //need response, datalen =8
		stmLinSendMsgByMaster(linMod, plinMsgInfo,LinMsg3,0); //
		if(!stmLinWaitForReponse(linMod)){//Wait for Response from the slave.
			printf("NoResp\r\n");
		}else{
			stmLinMaster_MsgAppHandler(linMod);
		}
		delayms(1000);

		//goto sleep

		stmLinGotoSleep(linMod);
		delayms(5000);
		//goto Wakeup
		stmLinGotoWakeup();
		delayms(1);

	}
}

void stmLinGotoSleep(struct _yLIN_module *linMod){ //Make EN Off. You will see the transition of INH Output.
#if  ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))

#if (LIN_ROLE == LIN_ROLE_MASTER)
	u8 txD[8];
	struct LinMsgInfo *plinMsgInfo;
	//-- prepare Sleep Command msg and send it.
	txD[0]=0x00;
	txD[1]=0xFF;txD[2]=0xFF;txD[3]=0xFF;txD[4]=0xFF;txD[5]=0xFF;txD[6]=0xFF;txD[7]=0xFF;
	plinMsgInfo = &linMod->LinMsgInfoPool[LinMsg4];//no response, datalen =8
	stmLinSendMsgByMaster(linMod, plinMsgInfo,LinMsg4,txD); //send header and control data by this master
	//after then, this master enters to sleep by disabling LIN Transceiver. At this moment, the LIN transceiver will issue INH=0.
	GPIO_ResetBits(GPIOB, GPIO_Pin_10); //EN=0
	printf("Goto Sleep with EN=0\r\n");
#else //Slave... On receiving the sleep msg, the slave controller(MCU) issue EN=0 to make INH = 0. It will power off the MCU.
	printf("Goto Sleep with EN=0. It will issue INH=0. then this slave will be powered OFF by itself.\r\n");
	delayms(10);
	GPIO_ResetBits(GPIOB, GPIO_Pin_10); //EN=0
#endif

#else
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
#endif
}

void stmLinGotoWakeup(){ //Make EN On. You will see the transition of INH Output.

#if  ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
	GPIO_SetBits(GPIOB, GPIO_Pin_10); //EN=1
#else
	GPIO_SetBits(GPIOC, GPIO_Pin_8); //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_PIN_5); //EN=1:
#endif

	//Send Wakeup Signal : Zeroing for 250usec ~ 5msec... (A break signal will zeroing for 550usec)..
	USART_SendBreak(LIN_USART);//Send Break Signal
	while (USART_GetFlagStatus(LIN_USART, USART_FLAG_TC) == RESET) {}
    delayms(100); //wait for awake the slaves.
	printf("Wakeup with EN=1\r\n");
}

//========== for SLAVE ==============================

char stmLinFindMsgIndexFromMsgInfoPool(u8 msgid,struct LinMsgInfo *linMsgInfoPool){
	char i;
	struct LinMsgInfo *linMsgInfo;
	linMsgInfo = linMsgInfoPool;
	for(i=0;i<MAX_MSG_NUM;i++){
		if(linMsgInfo->msgid == msgid){
			//printf("Id=%d\r\n",i);
			return i;
		}else
			linMsgInfo++;
	}
	return -1;
}
//Not used...
char stmLinGetDataLengFromMsgId(struct _yLIN_module *linMod, u8 msgid, u8 msgIndex){
	char retv;

	retv = (msgid & 0x30) >> 4;

	if(retv <= 1) 		retv = 2;
	else if(retv == 2) 	retv = 4;
	else  				retv = 8;

	if(linMod->LinMsgInfoPool[msgIndex].dleng == retv )
		return retv;
	else
		return -1;
}

//need some more...
void stmLinSlaveResponse(struct LinMsgInfo *plinMsgInfo){
	unsigned char linRespData[8]={0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8};
	if(plinMsgInfo->needResp){
		switch(plinMsgInfo->msgid){
		case MsgID_0_2B:
			stmLinSendData(linRespData, plinMsgInfo->dleng);
			break;
		case MsgID_0_8:
			stmLinSendData(linRespData, plinMsgInfo->dleng);
			break;
		default:
			printf("S>RX\r\n");
			break;
		}//switch
		printf("S>Rsp(Id=%x):",plinMsgInfo->msgid);
	}
}
//Parsing Data Field. (If this field is readbacked data, it will be disregarded.)
void stmLinParser(struct _yLIN_module *linMod,struct LinMsgInfo *plinMsgInfo){
	u8 i;

	if(linMod->role == LIN_ROLE_MASTER){
		if(plinMsgInfo->needResp){
			printf("M>Got a response from the Slave = ");
			if(plinMsgInfo->dleng != linMod->LinRxBuf.msgInfo.dleng){ //discard
				linMod->LinRxBuf.rcvd = 0;
			}else{
				for(i=0; i<plinMsgInfo->dleng;i++)	printf("%02x ", linMod->LinRxBuf.data[i]);
				printf("\r\n");
				linMod->LinRxBuf.rcvd = 1;
			}
		}else{//Readbacked data-- Discard
			linMod->LinRxBuf.rcvd = 0;
		}
	}else{ //as a slave -- need to response by sending data.
		if(plinMsgInfo->needResp){
			linMod->LinRxBuf.rcvd = 0; //Readbacked data -- discard
		}else{
			if(plinMsgInfo->dleng != linMod->LinRxBuf.msgInfo.dleng){ //discard
				linMod->LinRxBuf.rcvd = 0;
				printf("S>Wrong Length Discard\r\n");
			}else{
				linMod->LinRxBuf.rcvd = 1;
				printf("S>Got a request from the Master = ");
				for(i=0; i<plinMsgInfo->dleng;i++)	printf("%02x ", linMod->LinRxBuf.data[i]);
				printf("\r\n");
			}
		}
	}
}
//==========================================================================================================================

//Config USART2 (PA2,3) for LIN (19.2Kbps)
void stmUSART2forLinConf(unsigned char Role)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;

	  printf("LIN Config with USART2 and IRQ (PA2/PA3)\r\n");
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT6)|| (PROCESSOR == PROCESSOR_GD32F130FX))
	  //Config USART2 @ PA2(TXD)/PA3(RXD)

	  //a) Enable GPIOA clock with AFIO
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	  //(b) Remap : DO NOT USE. (May be its default AFIO is USART2)
	  //GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); //PA2/3 Remap to USART2 : NO USE IT...

	  //(c) Configure USART2 Tx Pin (PA2) as alternate function push-pull
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStruct);

	  //(d) Configure USART2 Rx (PA3) as input floating
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOA, &GPIO_InitStruct);

	  //(e) Supply clocking to USART2
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//USART(1,6)=>APB2, Others=>APB1

	  //(f) Disalbe USART2 if it is running.
	  USART_Cmd(USART2, DISABLE);// Disable USART

	    //(g) USART2 configuration
		USART_InitStructure.USART_BaudRate = 19200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART2, &USART_InitStructure);/* USART configuration */

		//(h) LIN BREAK Config
		USART_LINBreakDetectLengthConfig(USART2,USART_LINBreakDetectLength_11b);

		//(i) Enable LIN Function
		USART_LINCmd(USART2,ENABLE);
		//USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp)
		//void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState)
		//For Receiver

		//(j) Interrupt Config for RX and Break Detection
		if (Role == LIN_ROLE_MASTER){
			USART_ITConfig(USART2, USART_IT_RXNE | USART_IT_LBD, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
			//USART_ITConfig(USART2,USART_IT_LBD, ENABLE); 	//Link Break Detect
		}else{
			USART_ITConfig(USART2,USART_IT_RXNE, ENABLE); //Set USART Interrupt Mask with Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
			USART_ITConfig(USART2,USART_IT_LBD, ENABLE);
		}

		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure); //Initializes the NVIC peripheral

	  	  //(k) Finally Start USART2.
		USART_Cmd(USART2, ENABLE);// Enable USART2
#else
	  //STM407 and STM401 - USART1
	  //USART1,6=>APB2, Others=>APB1
	  //U1TX -PA9; U1RX-PA10
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);/* Enable GPIO clock */
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//Pin Toggle Speed Limit
	  GPIO_Init(GPIOA, &GPIO_InitStruct);

	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);	  /* Connect PXx to USARTx_Rx*/

	  //Configure USART1 for LIN : Word length = 8bits, Stop bits = 1bit, Parity,
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	  /* Enable UART clock */
	  USART_InitStructure.USART_BaudRate = LIN_BAUDRATE; //20Kbps
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART1, &USART_InitStructure);

	  // Enable USART
	  USART_Cmd(USART1, ENABLE);

	  //For Receiver
	  USART_LINBreakDetectLengthConfig(USART1,USART_LINBreakDetectLength_11b);
	  //Enable LIN
	  USART_LINCmd(USART1,ENABLE);//USART_Cmd(USART1, ENABLE);

	  //Enable its Rx Interrupt mode
#if (IN_ROLE == LIN_ROLE_MASTER)
	  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  USART_ITConfig(USART1,USART_IT_LBD, ENABLE); //Link Break Detect
#else
	  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE); //Enable Receive Data register not empty interrupt //| USART_IT_ORE_RX
	  USART_ITConfig(USART1,USART_IT_LBD, DISABLE); //
#endif

	  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

#endif //PROCESSOR
}

//Config USART for LIN and a GPIO(PB10) for LIN_Enable pin
void stmLinNodeConfig(struct _yLIN_module *linMod){

	GPIO_InitTypeDef GPIO_InitStruct;
	unsigned char en1_disable0 = 1;

	//(a) Config USART2 (PA2,3) for LIN
	stmUSART2forLinConf(linMod->role); //19.2Kbps UART2

	//(b) Config PB10 for LIN Enable output pin
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
	//PB10 = LIN_EN : STM103
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);/* Enable GPIO clock */
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//Pin Toggle Speed Limit
	  GPIO_Init(GPIOB, &GPIO_InitStruct);
	  if(en1_disable0 == 1){
		  GPIO_SetBits(GPIOB, GPIO_Pin_10); //issue ENABLE
		  printf("Make LIN enable (PB10=1)\r\n");
	  }else
		  GPIO_ResetBits(GPIOB, GPIO_Pin_10); //issue DISABLE

#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
	//USART_LINCmd(USART1,ENABLE);

	//PB10 = LIN_EN : STM401RET
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);/* Enable GPIO clock */
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//Pin Toggle Speed Limit
	  GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_SetBits(GPIOB, GPIO_Pin_10);
#else
	/*
		//PC8 = LIN_EN : STM407
		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);// Enable GPIO clock
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//Pin Toggle Speed Limit
		  GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_SetBits(GPIOC, GPIO_Pin_8);
	*/
#endif

}
#if (USART2_FOR == USART2_FOR_LIN)
//master or slave
void USART2_IRQHandler(void) //void USART2_IntHandlerForLIN(void) //
{
    unsigned long ulIntStatus;
    unsigned char val;
    u8 dlen;
#if  ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
    // (LIN_USART == USART2)
	// check if the USART2 receive interrupt flag was set
	if( USART_GetITStatus(LIN_USART, USART_IT_LBD) ) //LineBreakDetect.
	{
		USART_ClearITPendingBit(USART2,USART_IT_LBD);//clear this IRQ
   		//val = USART2->DR; 	// the character from the USART2 data register is saved in t
    	g_LinMod.LinRxBuf.break_rcvd =1;
    	g_LinMod.LinRxBuf.sync_rcvd = 0;
    	g_LinMod.LinRxBuf.msgInfo.dleng = 0;
	}

	if( USART_GetITStatus(LIN_USART, USART_IT_RXNE) ){
		USART_ClearITPendingBit(LIN_USART,USART_IT_RXNE);//clear IRQ
		val = USART2->DR; 	// the character from the USART2 data register is saved in t

   		if(g_LinMod.LinRxBuf.break_rcvd){ //expect Sync...
   			g_LinMod.LinRxBuf.break_rcvd=0;
   			if(val == 0x55 ){ //Sync
   				g_LinMod.LinRxBuf.sync_rcvd=1;
   				printf("[55]");//printf("Sync Rcvd\n");
   			}else{
   				printf("Get Char. But no Sync Code\n");
   				g_LinMod.LinRxBuf.sync_rcvd=0; //disregards this msg.
   			}
   		}else if(g_LinMod.LinRxBuf.sync_rcvd){//already got sync code. Expect Msg ID...
   			g_LinMod.LinRxBuf.sync_rcvd=0;
   			g_LinMod.LinRxBuf.msgid_rcvd=1;
			g_LinMod.LinRxBuf.msgInfo.msgIndex = stmLinFindMsgIndexFromMsgInfoPool(val,&g_LinMod.LinMsgInfoPool[0]);
			g_LinMod.LinRxBuf.msgInfo.dleng = 0;
	  		g_LinMod.LinRxBuf.msgInfo.msgid = val;
	  		printf("(%02x)", val);//
	      	if(g_LinMod.role == LIN_ROLE_MASTER){ //Readbacked --> will be Discarded
	      		//printf("Get Msg ID(0x%02x):ID[3:0]=0x%02x,ID[5:4]=0x%02x, P[1:0]=0x%02x\r\n",val, getMsgId(val),getMsgId54(val), getParity(val)); //Useless. Thus you should drop it.
	      	}else{
	      		if(g_LinMod.LinMsgInfoPool[g_LinMod.LinRxBuf.msgInfo.msgIndex].needResp){
	      			g_LinMod.LinRxBuf.needRespBySlave = 1;
	      		}else
	      			g_LinMod.LinRxBuf.needRespBySlave = 0;
	      	}
	    }
	    else if (g_LinMod.LinRxBuf.msgid_rcvd) { //Previous Event was MsgId_received.
	    	printf(" %02x", val);//
	    	if(g_LinMod.LinRxBuf.msgInfo.dleng < g_LinMod.LinMsgInfoPool[g_LinMod.LinRxBuf.msgInfo.msgIndex].dleng){
	    		g_LinMod.LinRxBuf.data[g_LinMod.LinRxBuf.msgInfo.dleng]=val;
	    		g_LinMod.LinRxBuf.msgInfo.dleng++;
	    		if(g_LinMod.LinRxBuf.msgInfo.dleng == g_LinMod.LinMsgInfoPool[g_LinMod.LinRxBuf.msgInfo.msgIndex].dleng){
	    			printf("$\r\n");
					stmLinParser(&g_LinMod, &g_LinMod.LinMsgInfoPool[g_LinMod.LinRxBuf.msgInfo.msgIndex]);
	    			g_LinMod.LinRxBuf.sync_rcvd=0;
	    			g_LinMod.LinRxBuf.msgid_rcvd=0;
	    			g_LinMod.LinRxBuf.break_rcvd = 0;
	    		}
	    	}else{
	    		printf("Error : Too long data...Reset\r\n");
	    		g_LinMod.LinRxBuf.msgInfo.dleng = 0; //used as rx byte count
				g_LinMod.LinRxBuf.sync_rcvd=0;
				g_LinMod.LinRxBuf.msgid_rcvd=0;
				g_LinMod.LinRxBuf.break_rcvd = 0;
	    	}
	    }
    }
#else
#endif
}
#endif
//======================================================================================================================
//Pseudo Motor Controller
void stmLinMotorControl(unsigned char controldata)
{
	if(controldata == MOTOR_CMD_ROTATE_CW){
		printf("Do Motor Control : Forward\r\n");
	}else if(controldata == MOTOR_CMD_ROTATE_CCW){
		printf("Do Motor Control : Backward\r\n");
	}else if(controldata == MotorSpeedUP)
		printf("Do Motor Control : SpeedUp\r\n");
	else if(controldata == MotorSpeedDOWN)
		printf("Do Motor Control : SpeedDown\r\n");
	else{
		printf("Do Motor Control : ???\r\n");
	}
}

unsigned char LIN_Slave_ProcessRxMsg(struct _yLIN_module *linMod, struct StepperDriver *pStepper)
{

	unsigned char msgid;
	u8 i;
	/* By polling
	while(1){

		while(USART_GetFlagStatus(USART2, USART_FLAG_LBD) == RESET);
		printf("B..\r\n");
		while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);

		printf("R=%02x\r\n", USART_ReceiveData(USART2));
    }
	 */

	//by Interrupt
		if(linMod->LinRxBuf.needRespBySlave){
			stmLinSlaveResponse(&linMod->LinMsgInfoPool[linMod->LinRxBuf.msgInfo.msgIndex]);
			linMod->LinRxBuf.needRespBySlave = 0;
		}
		else if(linMod->LinRxBuf.rcvd) //set by ISR
		{
			printf("S>Rx(%x):", linMod->LinRxBuf.msgInfo.msgid);
			for(i=0; i<linMod->LinRxBuf.msgInfo.dleng;i++)	printf("%02x ", linMod->LinRxBuf.data[i]);
			printf("\r\n");

			switch(linMod->LinRxBuf.msgInfo.msgid){
			case MsgID_0_2A:
				if(linMod->LinRxBuf.data[0]== MOTOR_CMD_ROTATE_CW){
					stmLinMotorControl(MOTOR_CMD_ROTATE_CW);
				}else if(linMod->LinRxBuf.data[0]== MOTOR_CMD_ROTATE_CCW){
					stmLinMotorControl(MOTOR_CMD_ROTATE_CCW);
				}
				else if(linMod->LinRxBuf.data[0]== MotorSpeedUP)
				{
					stmLinMotorControl(MotorSpeedUP);
				}
				else if(linMod->LinRxBuf.data[0]== MotorSpeedDOWN)
				{
					stmLinMotorControl(MotorSpeedDOWN);
				}
				else{
					printf("Invalid\r\n");
					return 0;
				}
			case MsgID_3c_8:
				if(linMod->LinRxBuf.data[0]== 0x00) //Goto Sleep
				{
					stmLinGotoSleep(linMod);
				}
				break;
			default:
				break;
			}
			linMod->LinRxBuf.rcvd = 0;
		}
		return 0;
}

unsigned char LIN_Slave_ProcessRxMsgWithStepper(struct _yLIN_module *linMod, struct StepperDriver *pStepper)
{
	unsigned char msgid;
	u8 i;

	//by Interrupt
		if(linMod->LinRxBuf.needRespBySlave){
			stmLinSlaveResponse(&linMod->LinMsgInfoPool[linMod->LinRxBuf.msgInfo.msgIndex]);
			linMod->LinRxBuf.needRespBySlave = 0;
		}
		else if(linMod->LinRxBuf.rcvd) //set by ISR
		{
			printf("S>Rx(%x):", linMod->LinRxBuf.msgInfo.msgid);
			for(i=0; i<linMod->LinRxBuf.msgInfo.dleng;i++)	printf("%02x ", linMod->LinRxBuf.data[i]);
			printf("\r\n");

			switch(linMod->LinRxBuf.msgInfo.msgid){
			case MsgID_0_2A:
				if(linMod->LinRxBuf.data[0]== MOTOR_CMD_ROTATE_CW){
					Stepper_Rotate_Steps(pStepper, linMod->LinRxBuf.data[1]*50);
				}else if(linMod->LinRxBuf.data[0]== MOTOR_CMD_ROTATE_CCW){
					Stepper_Rotate_Steps(pStepper, -linMod->LinRxBuf.data[1]*50);
				}
				else if(linMod->LinRxBuf.data[0]== MotorSpeedUP)
				{
					stmLinMotorControl(MotorSpeedUP);
				}
				else if(linMod->LinRxBuf.data[0]== MotorSpeedDOWN)
				{
					stmLinMotorControl(MotorSpeedDOWN);
				}
				else{
					printf("Invalid\r\n");
					return 0;
				}
			case MsgID_3c_8:
				if(linMod->LinRxBuf.data[0]== 0x00) //Goto Sleep
				{
					stmLinGotoSleep(linMod);
				}
				break;
			default:
				break;
			}
			linMod->LinRxBuf.rcvd = 0;
		}
		return 0;
}
int LIN_SlaveTask(struct _yLIN_module *linMod){

	unsigned char msgid;
	u8 i;

	linMod->LinRxBuf.rcvd = 0;
	/* By polling
	while(1){

		while(USART_GetFlagStatus(USART2, USART_FLAG_LBD) == RESET);
		printf("B..\r\n");
		while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);

		printf("R=%02x\r\n", USART_ReceiveData(USART2));
    }
	 */

	//by Interrupt
	while(1){
		LIN_Slave_ProcessRxMsg(linMod, NULL);
	}
}

//==========================================


int stmLinLoop(void)
{
#if (LIN_ROLE == LIN_ROLE_MASTER)
	printf("LIN Master Test.\r\n");
	g_LinMod.role = LIN_ROLE_MASTER;
	stmLinNodeConfig(&g_LinMod); //Master mode Init
	stmLin_ConfigMsgInfoPool(&g_LinMod);
	stmLIN_MasterTask (&g_LinMod);
#else
	printf("LIN Slave Test.\r\n");
	g_LinMod.role = LIN_ROLE_SLAVE;
	g_LinMod.LinRxBuf.rcvd = 0;
	stmLinNodeConfig(&g_LinMod); //Slave mode Init
	stmLin_ConfigMsgInfoPool(&g_LinMod);
	LIN_SlaveTask (&g_LinMod);
#endif

}

#endif
