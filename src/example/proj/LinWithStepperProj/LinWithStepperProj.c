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
#include "yLin.h"


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
extern struct qei_table g_qei_table;

extern void Stepper_GpioConf();
extern void Stepper_Init_and_Config(struct StepperDriver *pStepper, int   rpm,	short microstepMode,short max_microstep);

#if (LIN_ROLE == LIN_ROLE_MASTER )
void stmLinWithStepperPrj_SendCommand(struct _yLIN_module *linMod, unsigned char cmd, unsigned char steps)
{
	struct LinMsgInfo *plinMsgInfo;
	unsigned char sndMsg[8];
	sndMsg[0] = cmd;
	sndMsg[1] = steps;

	switch(cmd){
	case MOTOR_CMD_ROTATE_CW:
		plinMsgInfo = &linMod->LinMsgInfoPool[LinMsg0]; //no response, datalen =2
		printf("send msg : CW, steps(%u)\r\n",steps);
		break;
	case MOTOR_CMD_ROTATE_CCW:
		plinMsgInfo = &linMod->LinMsgInfoPool[LinMsg0]; //no response, datalen =2
		printf("send CAN msg : CCW, steps(%u)\r\n",steps);
		break;
	}
	stmLinSendMsgByMaster(linMod, plinMsgInfo, LinMsg0, sndMsg);
}
#else

#endif

extern unsigned char LIN_Slave_ProcessRxMsgWithStepper(struct _yLIN_module *linMod, struct StepperDriver *pStepper);

int stmLinWithStepperPrj(void)
{
	unsigned char ecode, err_rxcnt;
	 struct StepperDriver stepper, *pStepper;
	 long long_journey_steps;
	 int offset = 0;

#if (LIN_ROLE == LIN_ROLE_MASTER )
	 stm_Qei_Config(99);// maxValue);
	 g_qei_table.oldVal = 100/2;
	 g_qei_table.val = 100/2;

     printf("LIN Master Test.\r\n");
	 g_LinMod.role = LIN_ROLE_MASTER;
	 stmLinNodeConfig(&g_LinMod); //Master mode Init
	 stmLin_ConfigMsgInfoPool(&g_LinMod);

	     while(1)
	     {
	     	if(g_qei_table.oldVal != g_qei_table.val){

	     		g_qei_table.direction = stm_Qei_GetDirection();
	     		if(g_qei_table.direction > 0)
	     		{
	     			offset = abs(g_qei_table.val - g_qei_table.oldVal);
	     			stmLinWithStepperPrj_SendCommand(&g_LinMod, MOTOR_CMD_ROTATE_CW, (unsigned char)offset);
	     		}else
	     		{
	     			offset = abs(g_qei_table.oldVal - g_qei_table.val);
	     			stmLinWithStepperPrj_SendCommand(&g_LinMod, MOTOR_CMD_ROTATE_CCW, offset);
	     		}
	     		printf("%d(dir=%s,offset=%d)\r\n", g_qei_table.val, (g_qei_table.direction > 0) ? "CW" : "CCW", offset );
	     		g_qei_table.oldVal = g_qei_table.val;
	     	}
	     }
#else
	printf("LIN Slave Test.\r\n");

	//a) Config Stepper
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

	//(b) Config LIN Slave
	g_LinMod.role = LIN_ROLE_SLAVE;
	g_LinMod.LinRxBuf.rcvd = 0;
	stmLinNodeConfig(&g_LinMod); //Slave mode Init
	stmLin_ConfigMsgInfoPool(&g_LinMod);

	//(c) main loop
	while(1){

		//LIN_Slave_ProcessRxMsg(&g_LinMod, pStepper);
		LIN_Slave_ProcessRxMsgWithStepper(&g_LinMod, pStepper);
 		//  delayms(1);
	}
#endif
	//disable interrupt handling
	//CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);
}

#endif
