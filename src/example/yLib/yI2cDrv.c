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
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "core_cm3.h"    //for irq
#include "core_cmFunc.h" //for irq



	#if (USE_GD32F103C8T6) ////For GD32F103
	#undef   I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED
	#define  I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED       ((uint32_t)0x00060002)//((uint32_t)0x00060082)

	#undef   I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED
	#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED       ((uint32_t)0x00070002)//((uint32_t)0x00060082)
	//#define  I2C_EVENT_MASTER_TRANS ((uint32_t)0x00070002)
	#endif


#endif
#include "misc.h"




extern void I2C_Send8bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
//================== WE SUPPORT ONLY 8 bit I2C Address ========================================

// REF : https://raw.githubusercontent.com/geoffreymbrown/STM32-Template/master/Library/i2c.c

/**
 *  Names of events used in stdperipheral library
 *
 *      I2C_EVENT_MASTER_MODE_SELECT                          : EV5
 *      I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED            : EV6
 *      I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED               : EV6
 *      I2C_EVENT_MASTER_BYTE_RECEIVED                        : EV7
 *      I2C_EVENT_MASTER_BYTE_TRANSMITTING                    : EV8
 *      I2C_EVENT_MASTER_BYTE_TRANSMITTED                     : EV8_2
 *
 **/
#define Timed(x) Timeout = 0x17FF; while (x) { if (Timeout-- == 0){ goto errReturn;}}
//#define Timed(x) Timeout = 0x07FF; while (x) { if (Timeout-- == 0){ goto errReturn;}}
//#define Timed(x) Timeout = 0x07FF; while (x) { if (Timeout-- == 0){ }}

void stm_I2C_Init(I2C_TypeDef * I2Cx, unsigned int I2Cspeed);
int stm_I2C_StartAndAddr(unsigned char address8bit, unsigned char direction);
int stm_I2C_SendBurst(unsigned char slave_addr8, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern I2C_TypeDef *gI2Cx;
unsigned char g_bI2CModuleConfigDone = 0;

unsigned char stmI2cSendbuf[16]; //i2c
unsigned char stmI2cRecvbuf[16]; //i2c

#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
//I2C2 : PF0(SDA), PF1(SCL),
void stm_I2C_Init(I2C_TypeDef * I2Cx, unsigned int I2Cspeed){//, u8 SlaveAddr8) {

    GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	gI2Cx = I2Cx;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //OpenDrain
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //No internal PU/PD
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* Enable I2C clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_I2C2); //SCL First
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_I2C2);
	I2C_InitStructure.I2C_ClockSpeed = I2Cspeed;//100000; //100Kbps
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;//SlaveAddr8;//0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C2, &I2C_InitStructure);

	/* Enable I2C */
	I2C_Cmd(I2C2, DISABLE);
	I2C_Cmd(I2C2, ENABLE);
}
#else
//I2C1 :
void stm_I2C_Init(I2C_TypeDef * I2Cx,unsigned int I2Cspeed){//, unsigned char SlaveAddr8) {//, u8 SlaveAddr8) {
    GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	gI2Cx = I2Cx;
#if (((PROCESSOR == PROCESSOR_STM32F103C8T6)) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX) )
	//PB7(SDA) PB6(SCL)

	/* I2C Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* GPIO Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, DISABLE); //for PB6/7 as default

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_25MHz;//GPIO_Speed_50MHz;//GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_DeInit(I2Cx);
	I2C_Cmd(I2Cx,DISABLE);

	I2C_InitStructure.I2C_ClockSpeed = I2Cspeed;//100K or 400Kbps
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;// (SlaveAddr8 >> 1);//0;//SlaveAddr7;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx, &I2C_InitStructure);
	I2C_Cmd(I2Cx, ENABLE);	/* Enable I2C */
	//I2C_AcknowledgeConfig(I2Cx,ENABLE);//Added
#else
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);/* Enable GPIO clock */

	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //OpenDrain
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;//was GPIO_PuPd_NOPULL; //No internal PU/PD
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_25MHz;//GPIO_Speed_50MHz;//GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);/* Enable I2C clock */
	I2C_DeInit(I2Cx);
	I2C_Cmd(I2Cx,DISABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); //SCL First
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); //SDA

	I2C_InitStructure.I2C_ClockSpeed = I2Cspeed;//100K or 400Kbps
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;//SlaveAddr8;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx, &I2C_InitStructure);
	I2C_Cmd(I2Cx, ENABLE);	/* Enable I2C */
	//I2C_AcknowledgeConfig(I2Cx,ENABLE);//Added
#endif
}
#endif


/**
  * @brief  Transmits the address byte to select the slave device.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Address: specifies the slave address which will be transmitted
  * @param  I2C_Direction: specifies whether the I2C device will be a Transmitter
  *         or a Receiver.
  *          This parameter can be one of the following values
  *            @arg I2C_Direction_Transmitter: Transmitter mode
  *            @arg I2C_Direction_Receiver: Receiver mode
  * @retval None.
  */
//was I2C_Send7bitAddress. YOON
void I2C_Send8bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_DIRECTION(I2C_Direction));
  /* Test on the direction to set/reset the read/write bit */
  if (I2C_Direction != I2C_Direction_Transmitter)
  {
    /* Set the address bit0 for read */
    Address |= I2C_OAR1_ADD0;
  }
  else
  {
    /* Reset the address bit0 for write */
    Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
  }
  /* Send the address */
  I2Cx->DR = Address;

#if (USE_GD32F103C8T6) //for GD32F103 ONLY (add delay after sending)
  int i = 0x5fff;
  while(i--);
#endif

}


//Used for both Write and Read Operations
int stm_I2C_StartAndAddr(unsigned char slave_addr8, unsigned char direction) { //slave_addr = 8bit addr
	uint32_t Timeout = 0;
	unsigned char status=0;

    // Enable Error IT (used in all modes: DMA, Polling and Interrupts
    //    I2Cx->CR2 |= I2C_IT_ERR;

	Timed (I2C_GetFlagStatus(gI2Cx, I2C_FLAG_BUSY));

	I2C_GenerateSTART(gI2Cx, ENABLE);

	status=1;


	Timed(!I2C_CheckEvent(gI2Cx, I2C_EVENT_MASTER_MODE_SELECT)); //EV5. After sending the START condition
	                                                             //the master has to wait for this event.
	                                                             //It means that the Start condition has been correctly
	                                                             //released on the I2C bus.
	                                                             //(the bus is free, no other devices is communicating).
	//After checking on EV5 (start condition correctly released on the bus), the master sends the address of the slave(s) with which it will communicate (I2C_Send7bitAddress() function,
	// it also determines the direction of the communication: Master transmitter or Receiver).

	I2C_Send8bitAddress(gI2Cx,  slave_addr8, direction); //EV5

	//====== was commented
	//status=2;
	//Timed(!I2C_CheckEvent(gI2Cx, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED));

	//Then the master has to wait that a slave acknowledges.
	//Timed(!I2C_CheckEvent(gI2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)); //EV8

	//If an acknowledge is sent on the bus, one of the following events will be set:
	//(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED or I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
	//==============
	status=3;

	if (direction == I2C_Direction_Transmitter) {
		#if (USE_GD32F103C8T6)
		Timed(I2C_GetFlagStatus(gI2Cx, I2C_FLAG_ADDR));//On rcving ACK from Slave. -- FOR GD32F103
		#else
		Timed(!I2C_CheckEvent(gI2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); //On rcving ACK from Slave. -- FOR STM32F103
		#endif
	} else if (direction == I2C_Direction_Receiver) {
		Timed(!I2C_CheckEvent(gI2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}

    return 1;
 errReturn:
 	//delayms(1); //YOON
 	 printf("I2C error(status=%u)\r\n",status);
    return 0;//1; //was 0
}

int stm_I2C_SendBurst(unsigned char slave_addr8, unsigned char *burst, unsigned char datalen)
{
	uint32_t Timeout = 0;
	int ret = 0;
	int i;
	unsigned char ev=0;

	ret = stm_I2C_StartAndAddr(slave_addr8, I2C_Direction_Transmitter);

	if(ret ==0){
		printf("I2C:ErrorAtStartAndAddr(NO ACK From the SLAVE.)\r\n");
		return 0;
	}

	if(datalen == 0) //add @2020.7.28
		return 1;

	ev = 0;
	I2C_SendData(gI2Cx, burst[0]); //Write first byte EV8_1
	Timed(!I2C_CheckEvent(gI2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)); //EV8

	ev = 1;
	for(i=1;i<datalen;i++){
		Timed(!I2C_GetFlagStatus(gI2Cx, I2C_FLAG_BTF));// wait on BTF(Byte transfer finished flag)
		I2C_SendData(gI2Cx, burst[i]);
	}
	ev = 2;
	Timed(!I2C_GetFlagStatus(gI2Cx, I2C_FLAG_BTF));// wait on the last BTF

	//I2C_AcknowledgeConfig(gI2Cx,DISABLE); //Disable Last ACK for master.
	ev = 3;
	I2C_GenerateSTOP(gI2Cx, ENABLE);
	Timed(I2C_GetFlagStatus(gI2Cx, I2C_FLAG_STOPF));
    return 1;
 errReturn:
	 printf("I2C error(ev=%u)\r\n",ev);
    return 0;
}

/*
 *  Read process is documented in RM008
 *
 *   There are three cases  -- read  1 byte  AN2824 Figure 2
 *                             read  2 bytes AN2824 Figure 2
 *                             read >2 bytes AN2824 Figure 1
 */
unsigned char stm_I2C_ReceiveBurst(unsigned char SlaveAddress8, unsigned char *buf, unsigned char nbyte)
//Status I2C_Read(I2C_TypeDef* I2Cx, unsigned char *buf,uint32_t nbyte, unsigned char SlaveAddress)
{
   uint32_t Timeout = 0;

   //I2Cx->CR2 |= I2C_IT_ERR;  interrupts for errors

   if (!nbyte){
	   printf("I2C_ReceiveBurst> nByte==0?\r\n");
	   return 1;
   }

   // Wait for idle I2C interface
   Timed(I2C_GetFlagStatus(gI2Cx, I2C_FLAG_BUSY));

   // Enable Acknowledgement, clear POS flag
   I2C_AcknowledgeConfig(gI2Cx, ENABLE);
   I2C_NACKPositionConfig(gI2Cx, I2C_NACKPosition_Current);

   // Intiate Start Sequence (wait for EV5
   I2C_GenerateSTART(gI2Cx, ENABLE);

   Timed(!I2C_CheckEvent(gI2Cx, I2C_EVENT_MASTER_MODE_SELECT));

   //printf("I2C_ReceiveBurst> Pass1\r\n");
   // Send Address with READ
   I2C_Send8bitAddress(gI2Cx, SlaveAddress8, I2C_Direction_Receiver); //I2C_Send7bitAddress(I2Cx, SlaveAddress, I2C_Direction_Receiver);
   Timed(!I2C_GetFlagStatus(gI2Cx, I2C_FLAG_ADDR));
   //printf("I2C_ReceiveBurst> Pass2\r\n");
   if (nbyte == 1) {
	   // Clear Ack bit
	   I2C_AcknowledgeConfig(gI2Cx, DISABLE);
	   // EV6_1 -- must be atomic -- Clear ADDR, generate STOP
	   __disable_irq();
	   (void) gI2Cx->SR2;
	   I2C_GenerateSTOP(gI2Cx,ENABLE);
	   __enable_irq();

	   // Receive data   EV7
	   Timed(!I2C_GetFlagStatus(gI2Cx, I2C_FLAG_RXNE));


	   *buf++ = I2C_ReceiveData(gI2Cx);
    }else if (nbyte == 2) {
    	// Set POS flag
    	I2C_NACKPositionConfig(gI2Cx, I2C_NACKPosition_Next);
    	// EV6_1 -- must be atomic and in this order
    	__disable_irq();
    	(void) gI2Cx->SR2;                           // Clear ADDR flag
    	I2C_AcknowledgeConfig(gI2Cx, DISABLE);       // Clear Ack bit
    	__enable_irq();
    	// EV7_3  -- Wait for BTF, program stop, read data twice
    	Timed(!I2C_GetFlagStatus(gI2Cx, I2C_FLAG_BTF));
    	__disable_irq();
    	I2C_GenerateSTOP(gI2Cx,ENABLE);
    	*buf++ = gI2Cx->DR;
    	__enable_irq();
    	*buf++ = gI2Cx->DR;
    } else{
    	(void) gI2Cx->SR2;                           // Clear ADDR flag
    	while (nbyte-- != 3){
    		// EV7 -- cannot guarantee 1 transfer completion time, wait for BTF instead of RXNE
    		Timed(!I2C_GetFlagStatus(gI2Cx, I2C_FLAG_BTF));
    		*buf++ = I2C_ReceiveData(gI2Cx);
    	}
    	Timed(!I2C_GetFlagStatus(gI2Cx, I2C_FLAG_BTF));
    	//   printf("I2C_ReceiveBurst> Pass3\r\n");
    	// EV7_2 -- Figure 1 has an error, doesn't read N-2 !
    	I2C_AcknowledgeConfig(gI2Cx, DISABLE);           // clear ack bit
    	__disable_irq();
    	*buf++ = I2C_ReceiveData(gI2Cx);             // receive byte N-2
    	I2C_GenerateSTOP(gI2Cx,ENABLE);                  // program stop
    	__enable_irq();
    	*buf++ = I2C_ReceiveData(gI2Cx);             // receive byte N-1
    	// wait for byte N
    	Timed(!I2C_CheckEvent(gI2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
    	//   printf("I2C_ReceiveBurst> Pass4\r\n");
    	*buf++ = I2C_ReceiveData(gI2Cx);
    	nbyte = 0;
    }
   // Wait for stop
   Timed(I2C_GetFlagStatus(gI2Cx, I2C_FLAG_STOPF));
   //printf("I2C_ReceiveBurst> Pass5\r\n");
   return 1;

errReturn:

  // Any cleanup here
  return 0;

}


//SlaveAddr(W)| Reg | Repeated Start | SetSlaveAddr(R) | GetData ...|
//but we use SlaveAddr(W)| Reg | Stop | Start | SetSlaveAddr(R) | GetData ...|
unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress8, unsigned char reg, unsigned char *buf, unsigned char nbyte){
	if(!stm_I2C_SendBurst(SlaveAddress8, &reg, 1))
		return 0;
	if(!stm_I2C_ReceiveBurst(SlaveAddress8, buf, nbyte))
		return 0;
	else
		return 1;
}

void stmI2cModuleConfig(I2C_TypeDef* I2Cx, unsigned int I2Cspeed){
	printf("I2C Init (speed=%u bps)...", I2Cspeed);
	stm_I2C_Init(I2Cx,I2Cspeed);
	g_bI2CModuleConfigDone = 1;
	printf("Done.\r\n");
}
