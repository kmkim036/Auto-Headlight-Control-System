/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0.2
* Date               : 07/11/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
#if 1

#include "cmdline.h"
#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "yInc.h"
#include <stdarg.h>
#include <string.h>

#include "yCan.h"

//For test, using CAN_Mode_LoopBack; Otherwise, use CAN_Mode_Normal;

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
//| CAN    |           |           |           |         |         |PB8/9(RX/TX)|
//+--------+-----------+-----------+-----------+---------+---------+--------+

/*
 *   *          ===================================================================
  *                                   How to use this driver
  *          ===================================================================

  *          1.  Enable the CAN controller interface clock using
  *                  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); for CAN1
  *              and RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE); for CAN2
  *  @note   In case you are using CAN2 only, you have to enable the CAN1 clock.
  *
  *          2. CAN pins configuration
  *               - Enable the clock for the CAN GPIOs using the following function:
  *                   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);
  *               - Connect the involved CAN pins to AF9 using the following function
  *                   GPIO_PinAFConfig(GPIOx, GPIO_PinSourcex, GPIO_AF_CANx);
  *                - Configure these CAN pins in alternate function mode by calling
  *                  the function  GPIO_Init();
  *
  *          3.  Initialise and configure the CAN using CAN_Init() and
  *               CAN_FilterInit() functions.
  *
  *          4.  Transmit the desired CAN frame using CAN_Transmit() function. The Mailbox is used to transmit.
  *
  *          5.  Check the transmission of a CAN frame using CAN_TransmitStatus()
  *              function.
  *
  *          6.  Cancel the transmission of a CAN frame using CAN_CancelTransmit()
  *              function.
  *
  *          7.  Receive a CAN frame using CAN_Recieve() function.
  *
  *          8.  Release the receive FIFOs using CAN_FIFORelease() function. : FIFO is the RXFIFO
  *
  *          9. Return the number of pending received frames using
  *              CAN_MessagePending() function.
  *
  *          10. To control CAN events you can use one of the following two methods:
  *               - Check on CAN flags using the CAN_GetFlagStatus() function.
  *               - Use CAN interrupts through the function CAN_ITConfig() at
  *                 initialization phase and CAN_GetITStatus() function into
  *                 interrupt routines to check if the event has occurred or not.
  *             After checking on a flag you should clear it using CAN_ClearFlag()
  *             function. And after checking on an interrupt event you should
  *             clear it using CAN_ClearITPendingBit() function.
 */
/* Local includes ------------------------------------------------------------*/

struct _yCAN_module yCAN_module;

//We provide 4 Message Data.
unsigned char g_ucMsg1_4[4] = { 0x00, 0xaa, 0xaa, 0xaa }; //the data of MsgObject1; msgid=0x0555
unsigned char g_ucMsgAutoResponseData2_8[8] = { 0x55, 0x55, 0xFF, 0xFF, 0x00, 0x00, 0x55, 0x55 }; //the data of MsgObject2; msgid=0x0666. automatic response value. You may use this value for using the value of temperature, etc.
unsigned char g_ucMsg3_6[6] = { 0x31, 0x31, 0x31, 0x31, 0x31, 0x31 }; //the data of MsgObject3; msgid=0x071c
unsigned char g_ucMsg4_8[8] = { 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32 }; //the data of MsgObject3; msgid=0x071d

/* Private variables ---------------------------------------------------------*/

//ErrorStatus HSEStartUpStatus;

/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
eResultStatus CAN_Polling(void);
eResultStatus CAN_Interrupt(void);

#define YCAN_BPS 100000 //100Kbps
#define YCAN_NUM_TQ 18 //time quata per bit

//Config CAN1 ports(PB8-CANRX and PB9-CANTX)
void stmCAN_Gpio_Irq_Can_Config(unsigned int bitRate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    RCC_ClocksTypeDef RCC_Clocks;
    unsigned int brp;

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_GD32F130FX) //PB8/9-CANRX/TX

    //(a) CAN port clock enable(APB2)
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    //(b) Configure CAN1 pins
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //(c) PIN Remap STM32F103's PB8/9 to use for CAN ****
    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

    //(d) CAN1 Peripheral clock enable(APB1=36MHz)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

#if (USE_CAN_MODE == USE_CAN_MODE_INTERRUPT)
    //(e) Enable internal CAN1 RX0 interrupt IRQ channel //(e) Enable internal CAN1 RX1 interrupt IRQ channel -- NOT WORKING
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn; //Working... //but CAN1_RX1_IRQn was Not Working -- YOON
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

    printf("CAN1(%u bps) : PB8=RX, PB9=TX, With RX IRQ\r\n", bitRate);

#elif (PROCESSOR == PROCESSOR_STM32F107VCT6) //PB8/9-CANRX/TX

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

    // Configure CAN1 pins
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //GPIO_Mode_Out_PP;//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE); //NEW ADDED

    // CAN1 Periph clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); /* CAN Periph clock enable APB1=48MHz*/

    // Enable internal CAN1 RX1 interrupt IRQ channel
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn; //USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

#else
    // CAN1 Periph clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); /* CAN Periph clock enable APB1=48MHz*/

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); /* GPIO for CAN Periph clock enable -- We need ?*/
    //Connect
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    /* Configure CAN1 pins: RX : PD0; TX :PD1*/ //We should Pulled Up.
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //RX
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //add
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //GPIO_AF_CAN1;//GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    // Enable CAN RX0 interrupt IRQ channel
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

#endif

    //(f) Init CAN Peripheral
    // use www.bittiming.can-wiki.info
    // PHB1 Clock = 36MHz

    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure); //Fills each CAN_InitStruct member with its default value.
    //modify some arguments
    CAN_InitStructure.CAN_TTCM = DISABLE; //time triggered communication mode
    CAN_InitStructure.CAN_ABOM = DISABLE; //automatic bus-off management (bus-off on errors)
    CAN_InitStructure.CAN_AWUM = DISABLE; //automatic wake-up mode
    CAN_InitStructure.CAN_NART = DISABLE; //non-automatic retransmission mode
    CAN_InitStructure.CAN_RFLM = DISABLE; //Receive FIFO Locked mode
    CAN_InitStructure.CAN_TXFP = DISABLE; //transmit FIFO priority

    //Set Normal Mode. On Loopback Mode, Tx enabled, RX is internal loopbacked. Ignore ACKs
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //CAN_Mode_LoopBack;//CAN_Mode_Normal; //In Loopback Mode, signals can be seen on pins.

    //(g) Setup precise bit clocks for CAN
    RCC_GetClocksFreq(&RCC_Clocks);
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT6) || (PROCESSOR == PROCESSOR_GD32F130FX))
    //CAN-BitTimingRegister.

    printf("PCLK1 = %uHz\r\n", RCC_Clocks.PCLK1_Frequency); //May be 72Mhz/2 = 36MHz (APB1 CLK)

    //we should here set the Prescaler. Prescaler = CAN_Clock(36MHz)/(bitRate*numTQ) = 36MHz/(bitRate*numTQ)
    switch (bitRate) {
    case 100000: //100Kbps
        //Prescaler = CAN_Clock(36MHz)/(bitRate*numTQ) = 36MHz/(100Kbps*18)
        // A TQ is 1/(PCLK1_Frequency/Prescaler) = 1/(36MHz/20) = 1/(1.8MHz) = 555nsec.
        // A BitTime is 555nsec*18TQ = 10usec. It yields 100Kbps.
        //We set the sampling point at 72% of a BitTime. --> SJW=1, TSEG1=12, TSEG2=5 --> BT = 18TQ, Sampled at 72%
        CAN_InitStructure.CAN_Prescaler = 20; //for 100Kbps..... Num tq = 18
        // A BT(18TQ) = SYNC(1TQ) + BS1(12TQ) + BS2(5TQ)
        CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; //was 4tq ...maximum number of time quanta of synchronisation_jump_width
        CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq; //CAN_BS1_11tq; bit_segment_1
        CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq; //CAN_BS2_1tq;//bit_segment_2
        break;
    case 125000: //125Kbps

        // 125Kbps : Prescaler=24; No. of TQ=16; Seg1(Prop_seg+Phase_seg1)=11; Seg2=4; SamplePoint=75% OR
        // 125Kbps : Prescaler=16; No. of TQ=24; Seg1(Prop_seg+Phase_seg1)=17; Seg2=6; SamplePoint=75%
        //numTQ = 16
        //Prescaler = CAN_Clock(36MHz)/(bitRate*numTQ) = 36MHz/(125Kbps*16) = 18
        // A TQ is 1/(PCLK1_Frequency/Prescaler) = 1/(36MHz/18) = 1/(2.25MHz) = 500nsec.
        // A BitTime is 500nsec*16TQ = 8 usec. It yields 125Kbps.
        //We set the sampling point at 75% of a BitTime. --> SJW=1, TSEG1=11, TSEG2=4 --> BT = 16TQ, Sampled at 75%
        CAN_InitStructure.CAN_Prescaler = 18; //for 125Kbps..... Num tq = 16
        // A BT(16TQ) = SYNC(1TQ) + BS1(11TQ) + BS2(4TQ)
        CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; //maximum number of time quanta of synchronisation_jump_width
        CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq; //bit_segment_1
        CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq; //bit_segment_2
        break;
    case 250000: //250Kbps
        //numTQ = 16
        //Prescaler = CAN_Clock(36MHz)/(bitRate*numTQ) = 36MHz/(250Kbps*16) = 9
        // A TQ is 1/(PCLK1_Frequency/Prescaler) = 1/(36MHz/9) = 1/(4MHz) = 250nsec.
        // A BitTime is 250nsec*16TQ = 4 usec. It yields 250Kbps.
        //We set the sampling point at 75% of a BitTime. --> SJW=1, TSEG1=11, TSEG2=4 --> BT = 16TQ, Sampled at 75%
        CAN_InitStructure.CAN_Prescaler = 9; //for 2505Kbps..... Num tq = 16
        // A BT(16TQ) = SYNC(1TQ) + BS1(11TQ) + BS2(4TQ)
        CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; //maximum number of time quanta of synchronisation_jump_width
        CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq; //bit_segment_1
        CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq; //bit_segment_2
        break;
    }
#if 1

#else
    //numTQ=8, //SJW=1, TSEG1=5, TSEG2=2 --> BT = 8TQ, Prescaler=> 45
    CAN_InitStructure.CAN_Prescaler = 45; //100Kbps..... Num tq = 8 -> 1 + 6 + 1
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; //maximum number of time quanta of synchronisation_jump_width
    CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq; //CAN_BS1_11tq; bit_segment_1
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq; //CAN_BS2_1tq;//bit_segment_2
#endif
    //CAN_InitStructure.CAN_Prescaler= 32;//125Kbps //	RCC_Clocks.PCLK1_Frequency/(15*500000); //=5.6 --> was 6 : PCLK1=42MHz
    //CAN_InitStructure.CAN_Prescaler= 16;//250Kbps; --- BS1=3tq; BS2=5tq
    //CAN_InitStructure.CAN_Prescaler= 4;//1Mbps
#else
    CAN_InitStructure.CAN_Prescaler = 24; //125Kbps..... was 6;//	RCC_Clocks.PCLK1_Frequency/(15*500000); //=5.6 --> was 6 : PCLK1=42MHz
#endif

    CAN_Init(CAN1, &CAN_InitStructure); //Apply this one.

    printf("stmCAN> Config Done.\r\n");
}

//Setup Message Filter
void stmCAN_AcceptAllFilterWithFifo0(void)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    CAN_FilterInitStructure.CAN_FilterNumber = 0; //0..13 for CAN1; 14..27 for CAN2
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //Mask mode filtering
        //=CAN_FilterMode_IdList //List mode filtering
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    //or CAN_FilterScale_16bit :
#if 1
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000; //MSB 0 == accept all
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000; //LSB
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
#else
    CAN_FilterInitStructure.CAN_FilterIdHigh = (CAN_MSG_ID0555 & 0xffff0000) >> 16; //accept only 0x0555
    CAN_FilterInitStructure.CAN_FilterIdLow = CAN_MSG_ID0555 & 0x0000ffff;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xfffff; //all 20 bits must match
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xfffff; //all 20 bits must match
#endif

    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0; //0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    printf("stmCAN> AcceptAllFilterWithFifo0.\r\n");
}

int stmCAN_SetAdvancedFilter(void)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    // (1: MsgObject #1) Initialize a message object to receive CAN messages with ID 0x0555 only.
    // The expected ID must be set along with the mask to indicate that all bits in the ID must match.
    CAN_FilterInitStructure.CAN_FilterNumber = 0; //0..13 for CAN1; 14..27 for CAN2
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //We use Mask mode filtering
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    //constrained range if the msg id may be passed the filter.
    CAN_FilterInitStructure.CAN_FilterIdHigh = (CAN_MSG_ID0555 & 0xffff0000) >> 16; //accept only 0x0555
    CAN_FilterInitStructure.CAN_FilterIdLow = CAN_MSG_ID0555 & 0x0000ffff;
    //set mask
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xfffff; //all 20 bits must match
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xfffff; //all 20 bits must match
    //assign rx fifo for handling this message of 0x555.
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0; //0;
    //activate this filter now.
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    // (2: MsgObject #2) For the ID of CAN_MSG_ID0666, and load into message object 2 which will be
    // used for automatic response any CAN messages with this ID.
    CAN_FilterInitStructure.CAN_FilterNumber = 1; //0..13 for CAN1; 14..27 for CAN2
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //identifier mask based filtering
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (CAN_MSG_ID0666 & 0xffff0000) >> 16; //accept only 0x0666
    CAN_FilterInitStructure.CAN_FilterIdLow = CAN_MSG_ID0666 & 0x0000ffff;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xfffff; //all 20 bits must match
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xfffff; //all 20 bits must match
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0; //0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    // (3: MsgObject #3) For the ID of CAN_MSG_ID071C, and load into message object 3 which will be
    // used for receiving any CAN messages with this ID.  Since only the CAN
    // ID field changes, we don't need to reload all the other fields.

    CAN_FilterInitStructure.CAN_FilterNumber = 2; //0..13 for CAN1; 14..27 for CAN2
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //identifier mask based filtering
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (CAN_MSG_ID071C & 0xffff0000) >> 16; //accept only 071C
    CAN_FilterInitStructure.CAN_FilterIdLow = CAN_MSG_ID071C & 0x0000ffff;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xfffff; //all 20 bits must match
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xfffff; //all 20 bits must match
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0; //0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    //(4) For the ID of CAN_MSG_ID071D, the slave will filter it.
    //-- NO FILTER
    printf("stmCAN> AdvFilterWithFifo0.\r\n");
    printf("stmCAN> Filter 0 = accept only 0x0555\r\n");
    printf("stmCAN> Filter 1 = accept only 0x0666\r\n");
    printf("stmCAN> Filter 2 = accept only 0x017C\r\n");
    printf("stmCAN> Filter 3 = disregard 0x017D\r\n");
}

//------ Send CAN message with Standard Format (MsgID len = 11bits)
// StdID = 0x11; DLC=2; Data= 0xca,0xfe
eResultStatus stmCAN_Send_StdPredefinedMsg(void)
{
    volatile CanTxMsg TxMessage;
    u32 i = 0;
    u8 UsedTransmitMailboxNum = 0;
    u8 ecode;

    // transmit
    yCAN_module.canMsgObject1.StdId = CAN_MSG_ID0555; //0666;//0x05555;//0x11; //msg id.
    yCAN_module.canMsgObject1.RTR = CAN_RTR_DATA; //NO remote_transmission_request (CAN_RTR_REMOTE if we want response.)
    yCAN_module.canMsgObject1.IDE = CAN_ID_STD; //type of identifier
    yCAN_module.canMsgObject1.DLC = 4; //Data Length of 4
    memcpy(yCAN_module.canMsgObject1.Data, g_ucMsg1_4, yCAN_module.canMsgObject1.DLC); //Move 4 byte user data into CAN FIFO

    //Start to transmit it. The return value is the mailbox number used to transmit.
    UsedTransmitMailboxNum = CAN_Transmit(CAN1, &yCAN_module.canMsgObject1);
    printf("Tx:UsedMboxNum=%u\r\n", UsedTransmitMailboxNum);
    //Initiates and transmits a CAN frame message. ret = The number of the mailbox that is used for transmission
    while ((CAN_TransmitStatus(CAN1, UsedTransmitMailboxNum) != CANTXOK) && (i < 0xFFF)) {
        i++;
    } //Checks the transmission status of a CAN Frame.
    if (i >= 0xFFF) {
        ecode = CAN_GetLastErrorCode(CAN1);
        if (ecode == CAN_ErrorCode_BitRecessiveErr) //0x40
            printf("CAN> Tx Fail(errCode=0x40: CAN_ErrorCode_BitRecessiveErr)\r\n");
        else if (ecode == CAN_ErrorCode_BitDominantErr) //0x50
            printf("CAN> Tx Fail(errCode=0x50: BitDominantErr)\r\n");
        else if (ecode == CAN_ErrorCode_ACKErr) //0x30
            printf("CAN> Tx Fail(errCode=0x30: CAN_ErrorCode_ACKErr)\r\n");
        else if (ecode == CAN_ErrorCode_CRCErr) //0x30
            printf("CAN> Tx Fail(errCode=0x60: CAN_ErrorCode_CRCErr)\r\n");
        /*
#define CAN_ErrorCode_NoErr           ((uint8_t)0x00)
#define	CAN_ErrorCode_StuffErr        ((uint8_t)0x10)
#define	CAN_ErrorCode_FormErr         ((uint8_t)0x20)
#define	CAN_ErrorCode_ACKErr          ((uint8_t)0x30)
#define	CAN_ErrorCode_BitRecessiveErr ((uint8_t)0x40)
#define	CAN_ErrorCode_BitDominantErr  ((uint8_t)0x50)
#define	CAN_ErrorCode_CRCErr          ((uint8_t)0x60)
#define	CAN_ErrorCode_SoftwareSetErr  ((uint8_t)0x70)
	  */
        else
            printf("CAN> Tx Fail(errCode=0x%02x)\r\n", ecode);
    } else
        printf("CAN> TxDone.\r\n");

    //Finish tx.
    CAN_CancelTransmit(CAN1, UsedTransmitMailboxNum);
}

eResultStatus stmCAN_Send_StdMsg(unsigned msgId32, unsigned char Rtr, unsigned char dLen, unsigned char* data)
{
    volatile CanTxMsg TxMessage;
    u32 i = 0;
    u8 UsedTransmitMailboxNum = 0;
    u8 ecode;

    // transmit
    yCAN_module.canMsgObject1.StdId = msgId32;
    yCAN_module.canMsgObject1.RTR = Rtr; //NO remote_transmission_request (CAN_RTR_REMOTE if we want response.)
    yCAN_module.canMsgObject1.IDE = CAN_ID_STD; //type of identifier
    yCAN_module.canMsgObject1.DLC = dLen; //Data Length of 4
    memcpy(yCAN_module.canMsgObject1.Data, data, dLen); //Move 4 byte user data into CAN FIFO

    // Start to transmit it. The return value is the mailbox number used to transmit.
    UsedTransmitMailboxNum = CAN_Transmit(CAN1, &yCAN_module.canMsgObject1);
    // Initiates and transmits a CAN frame message.
    // ret = The number of the mailbox that is used for transmission

    while ((CAN_TransmitStatus(CAN1, UsedTransmitMailboxNum) != CANTXOK) && (i < 0xFFF)) {
        i++;
    } // Checks the transmission status of a CAN Frame.

    if (i >= 0xFFF) {
        ecode = CAN_GetLastErrorCode(CAN1);
        if (ecode == CAN_ErrorCode_BitRecessiveErr) //0x40
            printf("CAN> Tx Fail(errCode=0x40: CAN_ErrorCode_BitRecessiveErr)\r\n");
        else if (ecode == CAN_ErrorCode_BitDominantErr) //0x50
            printf("CAN> Tx Fail(errCode=0x50: BitDominantErr)\r\n");
        else if (ecode == CAN_ErrorCode_ACKErr) //0x30
            printf("CAN> Tx Fail(errCode=0x30: CAN_ErrorCode_ACKErr)\r\n");
        else if (ecode == CAN_ErrorCode_CRCErr) //0x30
            printf("CAN> Tx Fail(errCode=0x60: CAN_ErrorCode_CRCErr)\r\n");
        else
            printf("CAN> Tx Fail(errCode=0x%02x)\r\n", ecode);
        /*
#define CAN_ErrorCode_NoErr           ((uint8_t)0x00)
#define	CAN_ErrorCode_StuffErr        ((uint8_t)0x10)
#define	CAN_ErrorCode_FormErr         ((uint8_t)0x20)
#define	CAN_ErrorCode_ACKErr          ((uint8_t)0x30)
#define	CAN_ErrorCode_BitRecessiveErr ((uint8_t)0x40)
#define	CAN_ErrorCode_BitDominantErr  ((uint8_t)0x50)
#define	CAN_ErrorCode_CRCErr          ((uint8_t)0x60)
#define	CAN_ErrorCode_SoftwareSetErr  ((uint8_t)0x70)
	  */
    } else
        printf("CAN> TxDone.\r\n");

    //Finish tx.
    CAN_CancelTransmit(CAN1, UsedTransmitMailboxNum);
}

//Send CAN Msg with Extended Format (msgID leng = 29bits)
//Extended ID = 0x1234; DLC=2; Data= 0xde,0xca
eResultStatus stmCAN_Send_Ext(void)
{
    CanTxMsg TxMessage;
    u32 i = 0;
    printf("CAN> Send_Ext.\r\n");
    // transmit 1 message
    TxMessage.StdId = 0x00;
    TxMessage.ExtId = 0x1234;
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = 0xDE;
    TxMessage.Data[1] = 0xCA;
    CAN_Transmit(CAN1, &TxMessage);
}

void stmCanShowRxMsg(void)
{
    int i = 0;

    if (yCAN_module.RxMessage.IDE != CAN_ID_STD)
        printf("RX>ExtentedMsgID=0x%04x(len=%u) ", yCAN_module.RxMessage.ExtId, yCAN_module.RxMessage.DLC);
    else
        printf("RX>StandardMsgID=0x%04x(len=%u) ", yCAN_module.RxMessage.StdId, yCAN_module.RxMessage.DLC);

    if (yCAN_module.RxMessage.DLC) {
        printf("Data=");
        for (i = 0; i < yCAN_module.RxMessage.DLC; i++)
            printf("%02x ", yCAN_module.RxMessage.Data[i]);
    }

    if (yCAN_module.RxMessage.RTR)
        printf(" : RETMOTE FREAME \r\n");
    else
        printf(" : NORMAL FREAME \r\n");

    //CAN_ClearFlag(CAN_FLAG_FF0);

    //stmLedToggle(); //stmUserLED_ON(); //indicates rx. (PC14)
    printf("CAN> Rx Succ(%u)\r\n", yCAN_module.RxCnt);
    yCAN_module.RxCnt++;
}

#if (USE_CAN_MODE == USE_CAN_MODE_POLLING)
/*******************************************************************************
* Description    : receive by polling
* Return         : PASSED if the reception is well done, FAILED in other case
*******************************************************************************/
unsigned char stmCAN_Receive_by_Polling(void)
{
    //CanRxMsg RxMessage;
    u32 i = 0;
    unsigned char ecode = 0;
    unsigned char numRxMsg = 0;
    unsigned char err_rxcnt = 0;
    FlagStatus bitstatus = RESET;

    i = 0;

    while (i != 0xFF) {
        numRxMsg = CAN_MessagePending(CAN1, CAN_FIFO0); //Returns the number of pending received messages.
        if (numRxMsg > 0)
            break;
        else
            i++;
    }

    //bitstatus = CAN_GetFlagStatus();

    //No rx frames .. return 0
    if (i >= 0xFF) {
        ecode = CAN_GetLastErrorCode(CAN1);
        if (ecode != CAN_ErrorCode_NoErr) {
            err_rxcnt = CAN_GetReceiveErrorCounter(CAN1);
            printf("errCode=0x%02x(err_rxcnt=%u)\r\n", ecode, err_rxcnt);
            CAN_FIFORelease(CAN1, CAN_FIFO0);
            return YCAN_RX_SOME_ERROR;
        } else {
            //No error, but Timeout...
            if (numRxMsg > 0) { //Clear Rx FIFO
                CAN_FIFORelease(CAN1, CAN_FIFO0);
                printf("Clear Rx FIFO\r\n");
            }
            return YCAN_RX_NO_FRAMES;
        }
    }

    //handle rx frame
    printf("CAN> Rx %u Msgs\r\n", numRxMsg);

    //fill zero
    yCAN_module.RxMessage.StdId = 0x00;
    yCAN_module.RxMessage.IDE = CAN_ID_STD;
    yCAN_module.RxMessage.DLC = 0;

    //Move a RxMessage from RxFIFO
    CAN_Receive(CAN1, CAN_FIFO0, &(yCAN_module.RxMessage));
    /*
  if (RxMessage.StdId == CAN_MSG_ID0555){
	  if (RxMessage.IDE != CAN_ID_STD)  {
		  return 0;
	  }
	  //if (RxMessage.DLC != 4)  {
		//  return 0;
	  //}
	  printf("CAN> Rx(%x)\r\n",RxMessage.StdId);
	  return 2;
  }else{

  }
*/
    if (yCAN_module.RxMessage.IDE != CAN_ID_STD) {
        printf("RX>ExtentedMsgID=0x%04x(len=%u) ", yCAN_module.RxMessage.ExtId, yCAN_module.RxMessage.DLC);
    } else {
        printf("RX>StandardMsgID=0x%04x(len=%u) ", yCAN_module.RxMessage.StdId, yCAN_module.RxMessage.DLC);
    }

    if (yCAN_module.RxMessage.DLC) {
        printf("Data=");
        for (i = 0; i < yCAN_module.RxMessage.DLC; i++)
            printf("%02x ", yCAN_module.RxMessage.Data[i]);
    }

    if (yCAN_module.RxMessage.RTR)
        printf(" : RETMOTE FREAME \r\n");
    else
        printf(" : NORMAL FREAME \r\n");

    CAN_FIFORelease(CAN1, CAN_FIFO0); //Clear Rx FIFO ? All clear?

    return YCAN_RX_WE_HAVE_FRAMES;
}
#elif (USE_CAN_MODE == USE_CAN_MODE_INTERRUPT)
void stmCanInterruptConfig(void)
{
    yCAN_module.RxStatus = YCAN_RX_NO_FRAMES;
    // Enable CAN FIFO0 message pending interrupt.
    CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_BOF, ENABLE); //Enables the specified CANx interrupts.
        //a) CAN_IT_FMP0= FIFO 0 message pending Interrupt on Rx msg
        //b) CAN_IT_BOF = BUS OFF error Interrupt
}

// This function handles CAN1 RX0 interrupts
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT6) || (PROCESSOR == PROCESSOR_GD32F130FX))
void USB_LP_CAN1_RX0_IRQHandler(void) //but...void CAN1_RX1_IRQHandler(void) -- Not working
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0)) //FIFO 0 Message Pending Flag = 1, if we receive CAN msg.
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0); //clear interrupt promptly.

        //fill Zero
        yCAN_module.RxMessage.StdId = 0x00;
        yCAN_module.RxMessage.ExtId = 0x00;
        yCAN_module.RxMessage.IDE = 0;
        yCAN_module.RxMessage.DLC = 0;
        yCAN_module.RxMessage.FMI = 0;
        yCAN_module.RxMessage.Data[0] = 0x00;
        yCAN_module.RxMessage.Data[1] = 0x00;

        //move rx msg from FIFO to here.
        CAN_Receive(CAN1, CAN_FIFO0, &yCAN_module.RxMessage); //after then, automatically release the FIFO

        printf("RX IRQ\r\n"); //Why multiple interrupts?

        //Check it.
        //if((yCAN_module.RxMessage.ExtId==0x1234) && (yCAN_module.RxMessage.IDE==CAN_ID_EXT)
        //	&& (yCAN_module.RxMessage.DLC==2) && ((yCAN_module.RxMessage.Data[1]|yCAN_module.RxMessage.Data[0]<<8)==0xDECA))
        {
            if (yCAN_module.RxStatus == YCAN_RX_NO_FRAMES)
                yCAN_module.RxStatus = YCAN_RX_WE_HAVE_FRAMES;
            else
                return;
        }
        //else  {
        //	yCAN_module.RxStatus = YCAN_RX_NO_FRAMES;
        //}
    } else if (CAN_GetITStatus(CAN1, CAN_IT_BOF)) //Bus-off Interrupt--Something wrong...
    {
        yCAN_module.RxStatus = YCAN_RX_SOME_ERROR;
        printf("BusOff\r\n");
    } else if (CAN_GetITStatus(CAN1, CAN_IT_FF0)) { //Full
        CAN_FIFORelease(CAN1, CAN_FIFO0); //Clear Rx FIFO ? All clear?
        printf("FIFO Release\r\n");
    } else {
        yCAN_module.RxStatus = YCAN_RX_NO_FRAMES;
        printf("Unknown IRQ\r\n");
    }
}
#else //STM407...
void CAN1_RX0_IRQHandler(void)
{

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0)) { //FIFO 0 Message Pending Flag
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);

        yCAN_module.RxMessage.StdId = 0x00;
        yCAN_module.RxMessage.ExtId = 0x00;
        yCAN_module.RxMessage.IDE = 0;
        yCAN_module.RxMessage.DLC = 0;
        yCAN_module.RxMessage.FMI = 0;
        yCAN_module.RxMessage.Data[0] = 0x00;
        yCAN_module.RxMessage.Data[1] = 0x00;

        CAN_Receive(CAN1, CAN_FIFO0, &yCAN_module.RxMessage); //Move the msg in the FIFO to RxMessage.

        //Check it.
        if ((yCAN_module.RxMessage.ExtId == 0x1234) && (yCAN_module.RxMessage.IDE == CAN_ID_EXT)
            && (yCAN_module.RxMessage.DLC == 2) && ((yCAN_module.RxMessage.Data[1] | yCAN_module.RxMessage.Data[0] << 8) == 0xDECA)) {
            yCAN_module.RxStatus = 2;
        } else {
            yCAN_module.RxStatus = 0;
        }
    } else
        yCAN_module.RxStatus = 0;
}
#endif
#endif

void stmCAN_InternalModuleInit(void)
{
    yCAN_module.RxCnt = 0;
    yCAN_module.RxStatus = YCAN_RX_NO_FRAMES;
}

void stmCAN_SendTestLoop(void)
{
    // CAN transmit at 100Kb/s
    while (1) {
        stmCAN_Send_StdPredefinedMsg();
        //stmCAN_Send_Ext();
        delayms(500);
    }
}

void stmCAN_ReceiveTestLoop(void)
{

#if (USE_CAN_MODE == USE_CAN_MODE_POLLING)
    while (1) {
        //(1) Polling method -- OK

        //CAN Receive Test
        yCAN_module.RxStatus = stmCAN_Receive_by_Polling();

        if (yCAN_module.RxStatus == YCAN_RX_SOME_ERROR) //No Rx Frame or Errors
        { // Turn off user led.
            //stmUserLED_OFF();
            stmLedToggle();
            ecode = 0;
            ecode = CAN_GetLastErrorCode(CAN1);

            if (ecode != CAN_ErrorCode_NoErr) {
                err_rxcnt = CAN_GetReceiveErrorCounter(CAN1);

                if (ecode == CAN_ErrorCode_StuffErr) {
                    printf("CAN> Rx Failed(err cnt = %u, err_code = 0x%02x(StuffErr)\r\n", err_rxcnt, ecode);
                } else
                    printf("CAN> Rx Failed(err cnt = %u, err_code = 0x%02x\r\n", err_rxcnt, ecode);
            } else {
                //No rx frame
            }

        } else if (yCAN_module.RxStatus == YCAN_RX_NO_FRAMES)
            continue;
        else if (yCAN_module.RxStatus == YCAN_RX_WE_HAVE_FRAMES) {
            printf("CAN> Rx Succ(%d)\r\n", yCAN_module.RxCnt);
            yCAN_module.RxCnt++;
            stmLedToggle();
            delayx(50000);
        }
        //delayms(10);
    }
#elif (USE_CAN_MODE == USE_CAN_MODE_INTERRUPT)
    printf("CAN_IT Config\r\n");
    // Enable CAN FIFO0 message pending interrupt.
    CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_BOF, ENABLE); //Enables the specified CANx interrupts.
        //a) CAN_IT_FMP0= FIFO 0 message pending Interrupt on Rx msg
        //b) CAN_IT_BOF = BUS OFF error Interrupt

    while (1) {
        if (yCAN_module.RxStatus == YCAN_RX_WE_HAVE_FRAMES) //if we got the rx msg.
        {
            stmCanShowRxMsg();
            yCAN_module.RxStatus = YCAN_RX_NO_FRAMES; //reset for next rx.
        } else if (yCAN_module.RxStatus == YCAN_RX_SOME_ERROR) {
            printf("CAN> Rx Bus Off...\r\n");
        }

        //send if we need
        //stmCAN_Send_Ext();
        delayms(1);
    }

    //disable interrupt handling
    CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
#endif
}

void stmCan_Config(unsigned int bitRate)
{
    stmUser_LED_GPIO_setup(); //PC14 for User LED (ON for receiving)

    stmCAN_Gpio_Irq_Can_Config(bitRate);

    stmCAN_AcceptAllFilterWithFifo0();
    //stmCAN_SetAdvancedFilter();

    stmCAN_InternalModuleInit();

#if (USE_CAN_MODE == USE_CAN_MODE_INTERRUPT)
    stmCanInterruptConfig();
#endif
}

int stmCanLoop(void)
{
    stmCan_Config(100000); //100Kbps.

#if 0
		//send test
		stmCAN_SendTestLoop();
#else
        //receive test
    stmCAN_ReceiveTestLoop();
#endif
}
#endif

void CAN_SEND_CMD(char led, char servo)
{
    unsigned char sndMsg[2];
    sndMsg[0] = led;
    sndMsg[1] = servo;
    stmCAN_Send_StdMsg(CAN_MSG_ID0555, CAN_RTR_DATA, 2, sndMsg);
}

int CAN_RECEIVE_CMD()
{
    int ret = 0;

    //Enables the specified CANx interrupts.
    CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_BOF, ENABLE);
    //a) CAN_IT_FMP0= FIFO 0 message pending Interrupt on Rx msg
    //b) CAN_IT_BOF = BUS OFF error Interrupt

    while (1) {
        if (yCAN_module.RxStatus == YCAN_RX_WE_HAVE_FRAMES) //if we got the rx msg.
        {
            int i;
            char buf[2];
            if (yCAN_module.RxMessage.DLC) {
                for (i = 0; i < yCAN_module.RxMessage.DLC; i++) {
                    buf[i] = yCAN_module.RxMessage.Data[i];
                }
            }
            ret = buf[0] << 1 | buf[1];
            yCAN_module.RxStatus = YCAN_RX_NO_FRAMES; //reset for next rx.
            break;
        } else if (yCAN_module.RxStatus == YCAN_RX_SOME_ERROR) {
            printf("CAN> Rx Bus Off...\r\n");
        }
        delayms(1);
    }

    //disable interrupt handling
    CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
    return ret;
}
