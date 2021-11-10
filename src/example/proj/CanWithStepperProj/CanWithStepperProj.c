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
/* Includes ------------------------------------------------------------------*/
/*
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
*/
#include "yInc.h"
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#include "math.h"


#include "yMotor.h"
#include "yCan.h"


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

//We provide 4 Message Data.
unsigned char g_ucCanMsg1_4[4] = {0x00, 0xaa, 0xaa, 0xaa}; //the data of MsgObject1; msgid=0x0555
unsigned char g_ucCanMsgAutoResponseData2_8[8]={0x55,0x55,0xFF,0xFF,0x00,0x00,0x55,0x55}; //the data of MsgObject2; msgid=0x0666. automatic response value. You may use this value for using the value of temperature, etc.
unsigned char g_ucCanMsg3_6[6] = {0x31, 0x31, 0x31, 0x31, 0x31, 0x31 }; //the data of MsgObject3; msgid=0x071c
unsigned char g_ucCanMsg4_8[8] = {0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32};//the data of MsgObject3; msgid=0x071d

//Config CAN1 ports(PB8-CANRX and PB9-CANTX)
extern void stmCan_Config(unsigned int bitRate);
extern int stmCAN_SetAdvancedFilter(void);
extern void stmCAN_SetFilter(void);
extern eResultStatus stmCAN_Send_StdMsg(unsigned msgId32, unsigned char Rtr, unsigned char dLen, unsigned char *data);
extern eResultStatus stmCAN_Send_Ext(void);
extern unsigned char stmCAN_Receive_by_Polling(void);
extern struct _yCAN_module yCAN_module;
extern void stmCanInterruptConfig(void);

extern void Stepper_GpioConf();
extern void Stepper_Init_and_Config(struct StepperDriver *pStepper, int   rpm,	short microstepMode,short max_microstep);

#define CANPROJ_ROLE_MASTER_CONTROLLER 	1
#define CANPROJ_ROLE_SLAVE_STEPPER 		0
#define CANPROJ_ROLE CANPROJ_ROLE_MASTER_CONTROLLER//CANPROJ_ROLE_SLAVE_STEPPE //

#if (CANPROJ_ROLE == CANPROJ_ROLE_MASTER_CONTROLLER)
void stmCanWithStepperPrj_SendCommand(unsigned char cmd, unsigned char steps)
{
	unsigned char sndMsg[8];
	switch(cmd){
	case MOTOR_CMD_ROTATE_CW:
		sndMsg[0] = 1;
		sndMsg[1] = steps;
		printf("send CAN msg : CW, steps(%u)\r\n",steps);
		break;
	case MOTOR_CMD_ROTATE_CCW:
		sndMsg[0] = 2;
		sndMsg[1] = steps;
		printf("send CAN msg : CCW, steps(%u)\r\n",steps);
		break;
	}
	stmCAN_Send_StdMsg(CAN_MSG_ID0555, CAN_RTR_DATA, 4, sndMsg);
}
#else
void stmCanWithStepperPrj_HandleRxMsg(struct StepperDriver *pStepper, CanRxMsg *canRxMsg)
{
	stmCanShowRxMsg();

	if(canRxMsg->IDE == CAN_ID_STD){
		if(canRxMsg->StdId == CAN_MSG_ID0555){
			switch(canRxMsg->Data[0]){
				case MOTOR_CMD_ROTATE_CW:
					Stepper_Rotate_Steps(pStepper, canRxMsg->Data[1]*50);//STEPS_PER_REV);
					break;
				case MOTOR_CMD_ROTATE_CCW:
					Stepper_Rotate_Steps(pStepper, -canRxMsg->Data[1]*50);//STEPS_PER_REV);					break;

			 		 break;
			 	default:
			 		 break;
			}

		}
	}
	yCAN_module.RxStatus = YCAN_RX_NO_FRAMES; //reset for next rx.
}

#endif

extern struct qei_table g_qei_table;

int stmCanWithStepperPrj(void)
{
	unsigned char ecode, err_rxcnt;
	 struct StepperDriver stepper, *pStepper;
	 long long_journey_steps;
	 int offset = 0;

	//a) CAN config
	stmCan_Config(100000);//100Kbps.

#if (CANPROJ_ROLE == CANPROJ_ROLE_MASTER_CONTROLLER)
	 stm_Qei_Config(99);// maxValue);
	 g_qei_table.oldVal = 100/2;
	 g_qei_table.val = 100/2;

	     while(1)
	     {
	     	if(g_qei_table.oldVal != g_qei_table.val){

	     		g_qei_table.direction = stm_Qei_GetDirection();
	     		if(g_qei_table.direction > 0)
	     		{
	     			offset = abs(g_qei_table.val - g_qei_table.oldVal);
	     			stmCanWithStepperPrj_SendCommand(MOTOR_CMD_ROTATE_CW, (unsigned char)offset);
	     		}else
	     		{
	     			offset = abs(g_qei_table.oldVal - g_qei_table.val);
	     			stmCanWithStepperPrj_SendCommand(MOTOR_CMD_ROTATE_CCW, offset);
	     		}
	     		printf("%d(dir=%s,offset=%d)\r\n", g_qei_table.val, (g_qei_table.direction > 0) ? "CW" : "CCW", offset );
	     		g_qei_table.oldVal = g_qei_table.val;
	     	}
	     }
#else
	 //a) Stepper config
	Stepper_GpioConf();//DIR, STEP, SLEEP, EN, (MS1, MS2, MS3)

	pStepper = &stepper;
	Stepper_Init_and_Config(
			pStepper,
			RPM, 	// Set target motor RPM.
			1, 		//microstepMode == FULLSTEP
			32);    //MaxmicrostepMode

	//Test for working...
	//(a-1)Moving... One complete revolution is 360°
	Stepper_rotate_Degree(pStepper,360);     // forward revolution
	delayms(100);
	Stepper_Rotate_Steps(pStepper, STEPS_PER_REV);    // forward revolution
	delayms(100);
	Stepper_Rotate_Steps(pStepper, -STEPS_PER_REV); //reverse
	delayms(100);

	//main loop
	while(1){
 	    if (yCAN_module.RxStatus == YCAN_RX_WE_HAVE_FRAMES ) //if we got the rx msg.
	 	{
	 		stmCanWithStepperPrj_HandleRxMsg(pStepper, &(yCAN_module.RxMessage));
	 	}
	 	else if( yCAN_module.RxStatus == YCAN_RX_SOME_ERROR){
	 		  printf("CAN> Rx Bus Off...\r\n");
	 	}
 		//  delayms(1);
	}
#endif
	//disable interrupt handling
	CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);
}

#endif
