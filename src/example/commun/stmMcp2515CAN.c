/*
  MCP2515.cpp - CAN library
  Written by Frank Kienast in November, 2010
  Modified by hooovahh in September, 2012 to fix bugs in extended addressing
	and allow single variable for both regular and extended addresses.

  Connections to MCP2515:

//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0CLK - PA2
//! - SSI0Fss - PD6 with select Header -- TBD
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5
 *
 *
 *
//+--------+-----------+-----------+-----------+---------+---------+--------+
//|        |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   | 103
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| ULED   | PB14      |PC4        |PE15       | <==     | PG7     |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| BUTTON |           |PC5(H)     |PD11(index)| PD11(L) |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| BEEP   |           |PB13       |PD14       | <==     |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| QEI    |           |PB0,1,12   |PD12,13,11 | PD12,13 |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| MOSI   |           |           |           |         |         |PB15    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| MISO   |           |           |           |         |         |PB14    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| SCLK   |           |           |           |         |         |PB13    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| nCS0   |           |           |           |         |         |PB12    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| IRQ    |           |           |           |         |         |PB4     |
//+--------+-----------+-----------+-----------+---------+---------+--------+

//H= Active High/L=Active Low
*/
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
//#include "stm32f4xx_spi.h"
#include "yInc.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
#include <time.h>
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

#include "ySpiDrv.h"

//======================================================================
//use nCS0 of PB12 - 103; use nCS0 of PD10-407
#define nCS_2515_H nCS0_H
#define nCS_2515_L nCS0_L
//======================================================================
typedef struct
{
  boolean isExtendedAdrs;
  unsigned long adrsValue;
  boolean rtr;
  u8 dataLength;
  u8 data[8];
}  CANMSG;

//Data rate selection constants
#define CAN_BAUD_10K 1
#define CAN_BAUD_50K 2
#define CAN_BAUD_100K 3
#define CAN_BAUD_125K 4
#define CAN_BAUD_250K 5
#define CAN_BAUD_500K 6

#define SLAVESELECT 10

//MCP2515 Registers
#define RXF0SIDH 0x00
#define RXF0SIDL 0x01
#define RXF0EID8 0x02
#define RXF0EID0 0x03
#define RXF1SIDH 0x04
#define RXF1SIDL 0x05
#define RXF1EID8 0x06
#define RXF1EID0 0x07
#define RXF2SIDH 0x08
#define RXF2SIDL 0x09
#define RXF2EID8 0x0A
#define RXF2EID0 0x0B
#define BFPCTRL 0x0C
#define TXRTSCTRL 0x0D
#define CANSTAT 0x0E
#define CANCTRL 0x0F
#define RXF3SIDH 0x10
#define RXF3SIDL 0x11
#define RXF3EID8 0x12
#define RXF3EID0 0x13
#define RXF4SIDH 0x14
#define RXF4SIDL 0x15
#define RXF4EID8 0x16
#define RXF4EID0 0x17
#define RXF5SIDH 0x18
#define RXF5SIDL 0x19
#define RXF5EID8 0x1A
#define RXF5EID0 0x1B
#define TEC 0x1C
#define REC 0x1D
#define RXM0SIDH 0x20
#define RXM0SIDL 0x21
#define RXM0EID8 0x22
#define RXM0EID0 0x23
#define RXM1SIDH 0x24
#define RXM1SIDL 0x25
#define RXM1EID8 0x26
#define RXM1EID0 0x27
#define CNF3 0x28
#define CNF2 0x29
#define CNF1 0x2A
#define CANINTE 0x2B

	#define MERRE 7
	#define WAKIE 6
	#define ERRIE 5
	#define TX2IE 4
	#define TX1IE 3
	#define TX0IE 2
	#define RX1IE 1
	#define RX0IE 0
#define CANINTF 0x2C
	#define MERRF 7
	#define WAKIF 6
	#define ERRIF 5
	#define TX2IF 4
	#define TX1IF 3
	#define TX0IF 2
	#define RX1IF 1
	#define RX0IF 0
#define EFLG 0x2D
#define TXB0CTRL 0x30
	#define TXREQ 3
#define TXB0SIDH 0x31
#define TXB0SIDL 0x32
	#define EXIDE 3
#define TXB0EID8 0x33
#define TXB0EID0 0x34
#define TXB0DLC 0x35
  #define TXRTR 7
#define TXB0D0 0x36

#define RXB0CTRL 0x60
	#define RXM1 6
	#define RXM0 5
	#define RXRTR 3
	// Bits 2:0 FILHIT2:0
#define RXB0SIDH 0x61
#define RXB0SIDL 0x62
#define RXB0EID8 0x63
#define RXB0EID0 0x64
#define RXB0DLC 0x65
#define RXB0D0 0x66

//MCP2515 Command Bytes
#define RESET 0xC0
#define READ 0x03
#define READ_RX_BUFFER 0x90
#define WRITE 0x02
#define LOAD_TX_BUFFER 0x40
#define RTS 0x80
#define READ_STATUS 0xA0
#define RX_STATUS 0xB0
#define BIT_MODIFY 0x05


extern 	  g_Spi2ConfigDoneFlag;
extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern unsigned char stmSpi2RdByte();
extern unsigned short stmSpi2WrByte(unsigned char inbyte);

extern uint8_t g_irq;
extern void stmLedToggle(void);
extern void Init_SysTick(int resolutionInUsec);
extern void stmUser_LED_GPIO_setup(void);
extern unsigned long millis();

boolean stmMCP2515_setCANBaud(int baudCAN);
u8 stmMCP2515_readReg(u8 regno);

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))	//PB4
void stmMCP2515CanSpi_IRQpin_Setup(void){

	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  /* Enable GPIOB clock */
 	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	  /* Enable and set EXTI Line4 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  /* Configure PB4 pin as input floating */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

	  /* Configure EXTI Line4 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
}
/* -- TO BE RESTORED
void EXTI4_IRQHandler()
{
    //PA8
	if(EXTI_GetITStatus(EXTI_Line4) != RESET) {
		// Toggle LED0
		stmLedToggle(); //ULED -103
		g_irq = 1;
		EXTI_ClearFlag(EXTI_Lin4);//==EXTI_ClearITPendingBit(EXTI_Line4);		// Clear the EXTI line 8 pending bit (PA8)
	}
}
*/
#else
#endif

void stmMCP2515CanSpiConfig(void){
	//use MODE 0
	//1Mbps, 8 bit mode
	//8Mbps, 16 bit mode
	stmSPI2_Config(2, 0,0,8);   //stmSPI3_Config(0); //use PA15 for nCS0
	nCS_2515_H;//nCs=1
	delayms(10);
	printf("MCP2515> SPI CAN TEST INIT\r\n");
};

void stmMCP2515_writeReg(u8 regno, u8 val)
{
  nCS_2515_L;;
  stmSpi2WrByte(WRITE);//SPI_transfer(WRITE);
  stmSpi2WrByte(regno);//SPI_transfer(regno);
  stmSpi2WrByte(val);//SPI_transfer(val);
  nCS_2515_H;;
}

void stmMCP2515_writeRegBit(u8 regno, u8 bitno, u8 val)
{
  nCS_2515_L;;

  stmSpi2WrByte(BIT_MODIFY);// SPI_transfer(BIT_MODIFY);
  stmSpi2WrByte(regno);// SPI_transfer(regno);
  stmSpi2WrByte(1 << bitno);//  SPI_transfer(1 << bitno);
  if(val != 0)
	  stmSpi2WrByte(0xff);//SPI_transfer(0xff);
  else
	  stmSpi2WrByte(0x00);//SPI_transfer(0x00);
  nCS_2515_H;;
}

u8 stmMCP2515_readReg(u8 regno)
{
  u8 val;

  nCS_2515_L;
  stmSpi2WrByte(READ);//  SPI_transfer(READ);
  stmSpi2WrByte(regno);//   SPI_transfer(regno);
  val = stmSpi2RdByte(); //  val = SPI_transfer(0);
  nCS_2515_H;

  return val;
}

u8 stmMCP2515_getCANTxErrCnt()
{
  return(stmMCP2515_readReg(TEC));
}

u8 MCP2515_getCANRxErrCnt()
{
  return(stmMCP2515_readReg(REC));
}

boolean stmMCP2515_initCAN(int baudCAN)
{
  u8 mode;
  //install 1msec tick
  //dtInstallMillisClock();

  stmMCP2515CanSpiConfig(); //SPI_begin();

  //reset first
  nCS_2515_L;//nCS_2515_L;;
  stmSpi2WrByte(RESET); // SPI_transfer(RESET); //Reset cmd
  nCS_2515_H; //nCS_2515_H;;

  delayms(10); //Should be delayed for resetting.

  //Read mode and make sure it is config
  mode = stmMCP2515_readReg(CANSTAT) >> 5;
  if(mode != 0b100){
	  printf("MCP2515> Mode=%u(should be 4) : InvalidMode\r\n",mode);
    return false;
  }else
	  printf("MCP2515> Mode=%u(= 4. OK...)\r\n",mode);

  return(stmMCP2515_setCANBaud(baudCAN));

}

boolean stmMCP2515_setCANBaud(int baudCAN)
{
  u8 brp;

  printf("MCP2515> set CAN Baud.\r\n");
  //BRP<5:0> = 00h, so divisor (0+1)*2 for 125ns per quantum at 16MHz for 500K
  //SJW<1:0> = 00h, Sync jump width = 1
  switch(baudCAN)
  {
    case CAN_BAUD_500K: brp = 0;
    	printf("set CAN baud = 500Kbps.\r\n");
    	break;
    case CAN_BAUD_250K: brp = 1;
    	printf("set CAN baud = 250Kbps.\r\n");
    	break;
    case CAN_BAUD_125K: brp = 3;
    	printf("set CAN baud = 125Kbps.\r\n");
    	break;
    case CAN_BAUD_100K: brp = 4;
		printf("set CAN baud = 100Kbps.\r\n");
		break;
    default: return false;
  }
  nCS_2515_L;;
  stmSpi2WrByte(WRITE);//SPI_transfer(WRITE);
  stmSpi2WrByte(CNF1); //SPI_transfer(CNF1);
  stmSpi2WrByte(brp & 0b00111111); //SPI_transfer(brp & 0b00111111);
  nCS_2515_H;;

  //PRSEG<2:0> = 0x01, 2 time quantum for prop
  //PHSEG<2:0> = 0x06, 7 time constants to PS1 sample
  //SAM = 0, just 1 sampling
  //BTLMODE = 1, PS2 determined by CNF3
  nCS_2515_L;;
  stmSpi2WrByte(WRITE);//SPI_transfer(WRITE);
  stmSpi2WrByte(CNF2); //SPI_transfer(CNF2);
  stmSpi2WrByte(0b10110001); // SPI_transfer(0b10110001);
  nCS_2515_H;

  //PHSEG2<2:0> = 5 for 6 time constants after sample
  nCS_2515_L;;
  stmSpi2WrByte(WRITE);//SPI_transfer(WRITE);
  stmSpi2WrByte(CNF3); //SPI_transfer(CNF3);
  stmSpi2WrByte(0x05);//SPI_transfer(0x05);
  nCS_2515_H;

  //SyncSeg + PropSeg + PS1 + PS2 = 1 + 2 + 7 + 6 = 16
  delayms(10);
  return true;
}

boolean stmMCP2515_setCANNormalMode(boolean singleShot)
{
  //REQOP2<2:0> = 000 for normal mode
  //ABAT = 0, do not abort pending transmission
  //OSM = 0, not one shot
  //CLKEN = 1, disable output clock
  //CLKPRE = 0b11, clk/8

  u8 settings;
  u8 mode;

  printf("MCP2515> set NormalMode.\r\n");

  settings = 0b00000111 | (singleShot << 3);

  stmMCP2515_writeReg(CANCTRL,settings);
  //Read mode and make sure it is normal
  mode = stmMCP2515_readReg(CANSTAT) >> 5;
  if(mode != 0)
    return false;

  return true;

}

boolean stmMCP2515_setCANReceiveonlyMode()
{
  //REQOP2<2:0> = 011 for receive-only mode
  //ABAT = 0, do not abort pending transmission
  //OSM = 0, not one shot
  //CLKEN = 1, disable output clock
  //CLKPRE = 0b11, clk/8

  u8 mode;

  stmMCP2515_writeReg(CANCTRL,0b01100111);
  //Read mode and make sure it is receive-only
  mode = stmMCP2515_readReg(CANSTAT) >> 5;
  if(mode != 3)
    return false;

  return true;

}

boolean stmMCP2515_receiveCANMessage(CANMSG *msg, unsigned long timeout)
{
    unsigned long startTime, endTime;
    unsigned short standardID = 0;
	boolean gotMessage;
    u8 val;
    int i;

    startTime = millis();//dtMillis();//
    endTime = startTime + timeout;
    gotMessage = false;
    while(millis() < endTime) //while(dtMillis() < endTime) //
    {
      val = stmMCP2515_readReg(CANINTF);
      //If we have a message available, read it
      if(bitRead(val,RX0IF) == 1)
      {
        gotMessage = true;
        break;
      }
    }

    if(gotMessage)
    {
      val = stmMCP2515_readReg(RXB0CTRL);
      msg->rtr = ((bitRead(val,3) == 1) ? true : false);

      //Address received from

      val = stmMCP2515_readReg(RXB0SIDH);
      standardID |= (val << 3);
      val = stmMCP2515_readReg(RXB0SIDL);
      standardID |= (val >> 5);

	  msg->adrsValue = (long) standardID;
      msg->isExtendedAdrs = ((bitRead(val,EXIDE) == 1) ? true : false);
      if(msg->isExtendedAdrs)
      {
        msg->adrsValue = ((msg->adrsValue << 2) | (val & 0b11));
        val = stmMCP2515_readReg(RXB0EID8);
		msg->adrsValue = (msg->adrsValue << 8) | val;
        val = stmMCP2515_readReg(RXB0EID0);
        msg->adrsValue = (msg->adrsValue << 8) | val;
      }
      msg->adrsValue = 0b11111111111111111111111111111 & msg->adrsValue; // mask out extra bits
      //Read data bytes
      val = stmMCP2515_readReg(RXB0DLC);
      msg->dataLength = (val & 0xf);
      nCS_2515_L;;
      stmSpi2WrByte(READ);//SPI_transfer(READ);
      stmSpi2WrByte(RXB0D0); //SPI_transfer(RXB0D0);
      for(i = 0; i < msg->dataLength; i++)
        msg->data[i] = stmSpi2RdByte(); //(SPI_transfer(0);
      nCS_2515_H;;

      //And clear read interrupt
      stmMCP2515_writeRegBit(CANINTF,RX0IF,0);
    }

    return gotMessage;
}

boolean stmMCP2515_transmitCANMessage(CANMSG *msg, unsigned long timeout)
{
  unsigned long startTime, endTime;
  boolean sentMessage;
  unsigned short val;
  int i;
  unsigned short standardID = 0;

  standardID = (short)msg->adrsValue;
  startTime = millis();
  endTime = startTime + timeout;
  sentMessage = false;
  if(!msg->isExtendedAdrs)
  {
	//Write standard ID registers
	val = standardID >> 3;
	stmMCP2515_writeReg(TXB0SIDH,val);
	val = standardID << 5;
	stmMCP2515_writeReg(TXB0SIDL,val);
  }
  else
  {
	//Write extended ID registers, which use the standard ID registers
	val = msg->adrsValue >> 21;
	stmMCP2515_writeReg(TXB0SIDH,val);
	val = msg->adrsValue >> 16;
	val = val & 0b00000011;
	val = val | (msg->adrsValue >> 13 & 0b11100000);
	val |= 1 << EXIDE;
	stmMCP2515_writeReg(TXB0SIDL,val);
	val = msg->adrsValue >> 8;
	stmMCP2515_writeReg(TXB0EID8,val);
    val = msg->adrsValue;
    stmMCP2515_writeReg(TXB0EID0,val);
  }

  val = msg->dataLength & 0x0f;
  if(msg->rtr)
    bitWrite(val,TXRTR,1);
  stmMCP2515_writeReg(TXB0DLC,val);

  //Message bytes
  nCS_2515_L;
  stmSpi2WrByte(WRITE);//SPI_transfer(WRITE);
  stmSpi2WrByte(TXB0D0); // SPI_transfer(TXB0D0);
  for(i = 0; i < msg->dataLength; i++)
	  stmSpi2WrByte(msg->data[i]);//SPI_transfer(msg->data[i]);
  nCS_2515_H;;

  //Transmit the message
  stmMCP2515_writeRegBit(TXB0CTRL,TXREQ,1);

  sentMessage = false;
  while(millis() < endTime)
  {
    val = stmMCP2515_readReg(CANINTF);
    if(bitRead(val,TX0IF) == 1)
    {
      sentMessage = true;
      break;
    }
  }

  //Abort the send if failed
  stmMCP2515_writeRegBit(TXB0CTRL,TXREQ,0);

  //And clear write interrupt
  stmMCP2515_writeRegBit(CANINTF,TX0IF,0);

  return sentMessage;

}

long stmMCP2515_queryOBD(u8 code)
{
  CANMSG msg;
  long val;
  boolean rxSuccess;
  int noMatch;

  printf("MCP2515> queryOBD(Code=0x%02x)\r\n",code);
  msg.adrsValue = 0x7df;
  msg.isExtendedAdrs = false;
  msg.rtr = false;
  msg.dataLength = 8;
  msg.data[0] = 0x02;
  msg.data[1] = 0x01;
  msg.data[2] = code;
  msg.data[3] = 0;
  msg.data[4] = 0;
  msg.data[5] = 0;
  msg.data[6] = 0;
  msg.data[7] = 0;

  if(!stmMCP2515_transmitCANMessage(&msg,1000))
    return 0;

  rxSuccess = stmMCP2515_receiveCANMessage(&msg,1000);
  if (rxSuccess)
  {
    //Check if the PIDs match (in case other messages are also on bus)
	noMatch = 0;
    while(msg.data[2] != code)
	{
        rxSuccess = stmMCP2515_receiveCANMessage(&msg,1000);
        noMatch++;
        if (!rxSuccess || noMatch >= 5)
		{
        	printf("Not successful Rx\r\n");
            return 0;
        }
    }
  }
  else{
  	printf("Rx Fail\r\n");
    return 0;
  }

  if(msg.data[0] == 3)
    val = msg.data[3];
  else
    val = 256 * msg.data[3] + msg.data[4];

  return val;
}

void stmMCP2515Loop()
{
	unsigned char retv = 0;
	stmUser_LED_GPIO_setup();
	stmMCP2515CanSpi_IRQpin_Setup();

	Init_SysTick(1000); //for internal timer.

	retv = stmMCP2515_initCAN(CAN_BAUD_250K);
	if(retv)
		printf("MCP2515>Init Done.\r\n");
	else{
		printf("MCP2515>Init Failed.\r\n");
	}

	stmMCP2515_setCANNormalMode(0);



	while(1){ 	// Add TBD
		printf("TxErrCnt = %u\r\n",stmMCP2515_getCANTxErrCnt());
		printf("RxErrCnt = %u\r\n",MCP2515_getCANRxErrCnt());
		stmMCP2515_queryOBD(1);
	}
}

