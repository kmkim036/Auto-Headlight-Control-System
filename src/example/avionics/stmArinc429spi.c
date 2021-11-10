#include <string.h>
#include <stdarg.h>
#include "yInc.h"

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


#if ((PROCESSOR == PROCESSOR_STM32F103C8T6)|| (PROCESSOR == PROCESSOR_STM32F103RCT6)  || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"

#include "stm32f10x_tim.h"
#include "stm32f10x_i2c.h"
#include "misc.h"
#endif
#include "ySpiDrv.h"
//Write/Read 4 Bytes in sequence

//use CS0 of PB12 : 103
//use CS0 of PA15 : 107
#define CS_ARINC429_H nCS0_H
#define CS_ARINC429_L nCS0_L

#if (PROCESSOR == PROCESSOR_STM32F401RET6)
extern void stmSpi1_Config(unsigned char nCS);
extern unsigned char stmSpi1RdByte();
extern void stmSpi1WrByte(unsigned char inbyte);
#elif (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
extern unsigned char stmSpi2RdByte();
extern unsigned short stmSpi2WrByte(unsigned char inbyte);
extern unsigned short stmSpi2WrShort(unsigned short inData);
extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);

//PA1 ClockGen 1.629MHz
void stmArinc429_ClkGen(){
	GPIO_InitTypeDef init_AF;//
	TIM_TimeBaseInitTypeDef timer_init;
    TIM_OCInitTypeDef tim_oc_init2;

    //(2) PA1 - TIM2-CH2
    //(2a) GPIO Init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);

	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF_PP;
	init_AF.GPIO_Speed = GPIO_Speed_10MHz;
	init_AF.GPIO_Pin = GPIO_Pin_1;//
	GPIO_Init(GPIOA, &init_AF);//
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    //(2b) TIM init
   	//Freq = 36MHz/(Precaler+1)/(period-1)
   	//We need 1.629MHz/
   	TIM_DeInit(TIM2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

   	TIM_TimeBaseStructInit(&timer_init);
   	//1.6MHz --> (14,0,2,1) -- Duty=40% ?
   	//1.2MHz --> (14,0,3,2) -- Duty=50% (36/(14+1)/(3-1) = 1.2MHz)
   	//1.285MHz --> (13,0,3,2) -- Duty=50%
   	//1.385MHz --> (12,0,3,2) -- Duty=50%
   	//1.636MHz --> (10,0,3,2) -- Duty=50%
   	timer_init.TIM_Prescaler 	= 10; // ---> 36/(10+1)/(3-1) = 1.636MHz
   	timer_init.TIM_ClockDivision = 0;
   	timer_init.TIM_CounterMode = TIM_CounterMode_Up;
   	timer_init.TIM_Period 		= 3;//(Auto-Reload Register at the next update event)
   	TIM_TimeBaseInit(TIM2, &timer_init);

   	//channel 2;
    TIM_OCStructInit(&tim_oc_init2);
    tim_oc_init2.TIM_OCMode = TIM_OCMode_PWM1;
    tim_oc_init2.TIM_Pulse = 2; //50% = Pulse/(PERIOD+1)
    tim_oc_init2.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC2Init(TIM2, &tim_oc_init2);//

    //TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
    //TIM_ARRPreloadConfig(TIM2,ENABLE);

   	TIM_Cmd(TIM2, ENABLE);

   	//TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

void stmArinc429_TransmitterLoop()
{
    unsigned char i=0;

    printf("Arinc429 Test with SPI2 Mode 1, 100Kbps, 8bit data.");

    stmArinc429_ClkGen(); //Generate SCK of 36MHz/256

    stmSPI2_Config(
    		100, //100Kbps
    		0, //use nCS0 on PB12
    		0, //Mode 0
    		16); //data 8 bit (or 16)

	CS_ARINC429_H;

	stmSpi2WrShort(0x0000);//Init FiFo

    while(1){

    	i++;
    	i = i%2;

    	//Write 4Bytes(32bits) of an ARINC429 Msg.
    	CS_ARINC429_L;
    	stmSpi2WrShort(0xA0B0+i);
    	stmSpi2WrShort(0xC0D0+i);
    	CS_ARINC429_H;

    	//Push 16bit FIFO (74HC40105)
    	stmSpi2WrShort(0x0000); //Although nCS=H, it may generate SCK to pushout.

    	delayms(1000);
    }
    return 1; //retVal;
}
//==================== ARINC429 Receive ==========================================
struct _arinc429_RX{
	unsigned char 	bData;
	unsigned short 	sData; 		 //short data
	unsigned long 	arinc429msg; //32bit arinc msg
	unsigned long 	rxCnt16; 	 //spi's 16 bit transaction count
	unsigned long 	rxMsgCnt;	 //arinc429 msg count
};

struct _arinc429_RX arinc429_RX;

void stmSPI2_As_Slave_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	SPI_InitTypeDef  SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* ncs0 = always enabled
	if(nCS==0){
		  //PB12 -nCS0
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 ;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOB, &GPIO_InitStruct);
		  GPIO_SetBits(GPIOB, GPIO_Pin_12); //GPIOB->BSRRL |= GPIO_Pin_12; // set  high
	}else{
		//TBD
	}
     */
	// Enable SPI2 clocks -PB13(SCK)/14(MISO)/15(MOSI)
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
	  GPIO_Init(GPIOB, &GPIO_InitStruct);

	  SPI_I2S_DeInit(SPI2);
	  SPI_StructInit(SPI2);

	  //We config SPI2 as a slave mode.
  	  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;//SPI_Mode_Master;
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//Lowest speed of 36MHz/256
  	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	  SPI_InitStructure.SPI_NSS  = SPI_NSS_Hard;//SPI_NSS_Soft;//
  	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	  SPI_InitStructure.SPI_CRCPolynomial = 7;
  	  SPI_InitStructure.SPI_DataSize = (data8or16 == 8) ?	SPI_DataSize_8b : SPI_DataSize_16b;

	  if(spiMode==0){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //Rising SCK Edge
	  }else if(spiMode==1){
		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //Low in idle.
		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //Falling SCK Edge
	  }else if(spiMode==2){
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //Falling SCK Edge
  	  }else if(spiMode==3){
  		  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //High in idle.
  		  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //Rising SCK Edge
  	  };

	  SPI_Init(SPI2, &SPI_InitStructure);

	  //enable SPI interrupt
  	  //SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE); //enable tx empty interrupt
  	  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE); //enable rx not empty interrupt

  	  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

  	  //SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF); //Should be included here to generate RXNE event
  	  //SPI_SSOutputCmd(SPI2,ENABLE);

  	  SPI_Cmd(SPI2, ENABLE);// Enable the SPI peripheral
}

//Interrupt handler on Receive SPI each 2 bytes.
//We concatenate twp 16-bit spi transactions to reassemble a complete arinc429 message.
void SPI2_IRQHandler(void)
{
	if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE)){

		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));//needless?

	    arinc429_RX.sData = SPI_I2S_ReceiveData(SPI2); //same  rxb = SPI2->DR;
	    if(arinc429_RX.rxCnt16 == 1){ //we can reassemble a complete 429 msg.
	    	arinc429_RX.arinc429msg = (arinc429_RX.arinc429msg << 16) | arinc429_RX.sData;
   			arinc429_RX.rxCnt16 = 0;
   			arinc429_RX.rxMsgCnt++;
	    }else{ //we now receive the first fragment of an arinc 429 msg.
  			arinc429_RX.arinc429msg = arinc429_RX.sData;
  			arinc429_RX.rxCnt16 = 1;
	    }
		printf("R(%u)=0x%04x\r\n",arinc429_RX.rxCnt16,arinc429_RX.sData);

		SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE); //clear interrupt
	}
	/*
	//on tx empty....
	if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE)){
		SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_TXE);
		if(arinc429_RX.bData == 0xfe){
			SPI_SendData8(SPI2, arinc429_RX.prodId); //reply prodId
		}else if(arinc429_RX.bData == 2){
			SPI_SendData8(SPI1, arinc429_RX.value[arinc429_RX.index]); //SPI1->DR = ready_data;
			arinc429_RX.index++;
			if(arinc429_RX.index >= SPI_TRANSACTION_SIZE){
				arinc429_RX.index = 0;
			}
		}else {

			//SPI2->DR = 0xFF; //Send dummy.
			//arinc429_RX.index = 0;
//		}
	}*/
}

//SPI Slave Mode
//get data at falling edge (Mode1)
void stmArinc429_ReceiverLoop()
{
    unsigned long prev;

    printf("Arinc429 Receiver with SPI2 Mode 1, 100Kbps, 8bit data.");

    stmSPI2_As_Slave_Config(
    		100, //142Kbps = 36MHz/256
    		0, //use nCS0 on PB12
    		1, //Mode 1
    		16);//8); //data 8 bit (or 16)

    CS_ARINC429_L;

    memset(&arinc429_RX, 0x00, sizeof(struct _arinc429_RX)); //zeroing.

    while(1){
    	if(prev != arinc429_RX.rxMsgCnt){
    		printf("Rcv ARING429 Msg(%u)=0x%08x\r\n", arinc429_RX.rxMsgCnt,arinc429_RX.arinc429msg);
    		prev = arinc429_RX.rxMsgCnt;
    	}
    }
    return 1;
}

#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
#define M66 1
extern unsigned char stmSpi3RdByte();
extern void stmSpi3WrByte(unsigned char inbyte);//Read Two Bytes
extern void stmSPI3_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
void stmArinc429_ReadTemperatureLoop(){
    unsigned short retVal,retVal_MSB;

    printf("TC77 Temperature Test with SPI3 Mode 3, 1Mbps, 16bit data.");
#if M66
    //Use nCS0(PB12), Mode 3, 8bit Mode, -- STM32F103
    //Use nCS0(PA15), Mode 3, 8bit Mode, -- STM32F107
    stmSPI3_Config(
    		1, //1Mbps
    		0, //use nCS0 on PA15
    		3, //Mode 3
    		8); //data 8 bit (or 16)

    CS_ARINC429_H;//nCs0=1

    while(1){
    	CS_ARINC429_L;//nCs0=0

    	//Read Two Bytes
    	retVal_MSB = stmSpi3RdByte(); 		//MSB
    	retVal = 	 stmSpi3RdByte(); 		//LSB
    	retVal = retVal_MSB << 8 + retVal; 	//MSB + LSB

    	CS_ARINC429_H;//nCs0=1

    	printf("Temp=%d(Need converstion with floating point value)\r\n", retVal); //TBD
    	delayms(2000);
    }
    return 1; //retVal;
#else //M43 = The same as the STM32F407
    //Use nCS0(PB12), Mode 3, 8bit Mode, -- STM32F103
    //Use nCS0(PA15), Mode 3, 8bit Mode, -- STM32F107
    stmSPI2_Config(
    		1, //1Mbps
    		0, //use nCS0 on PA15
    		3, //Mode 3
    		8); //data 8 bit (or 16)

    CS_ARINC429_H;//nCs0=1

    while(1){
    	CS_ARINC429_H;//nCs0=0

    	//Read Two Bytes
    	retVal_MSB = stmSpi2RdByte(); 		//MSB
    	retVal = 	 stmSpi2RdByte(); 		//LSB
    	retVal = retVal_MSB << 8 + retVal; 	//MSB + LSB

    	CS_ARINC429_L;//nCs0=1

    	printf("Temp=%d(Need converstion with floating point value)\r\n", retVal); //TBD
    	delayms(1000);
    }
    return 1; //retVal;
#endif
}
#endif


