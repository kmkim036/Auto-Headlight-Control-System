
/* Includes ------------------------------------------------------------------*/

#include "yLib/eth/include/ethopts.h"
#include "yInc.h"
#if (PROCESSOR == PROCESSOR_STM32F107VCT)
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f10x.h"
//#include "core_cm3.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"//Reset and Clock Control
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f107_eth.h"
#include "misc.h"
#include "ySpiDrv.h"
#include "err.h"

#if !USE_LWIP_PTP
extern err_t ethernetif_low_level_output(volatile u8 *p,int framelength);
extern void yEther_SimpleSend(int i);
extern void yEther_SimpleShortSend(int i);

extern uint32_t g_tickEvent;

int ETH_SimpleIP101Loop(void)
{
	int seq;
	u32 ret,x;
	int i;
	u32 result;

	printf("ySimpleIp101Loop\r\n");
	Init_SysTick(1000);//1msec
    /***************************************************************************
    NOTE: When using Systick to manage the delay in Ethernet driver, the Systick
         must be configured before Ethernet initialization and, the interrupt
         priority should be the highest one.
     *****************************************************************************/

	//==== configure ethernet (GPIOs, clocks, MAC, DMA)
	ETH_X_BSP_Config(IP101A_PHY_ADDRESS, "IP101A");//ETH_IP101A_BSP_Config();//see stm32f4x7_eth_bsp.c //ETH_SimpleOrSwitchPHY_BSP_Config();
	//ETH_RTL9K_BSP_Config();
	//ETH_PTP_ForSwitchBoard_Config("Simple");
	ethernetif_low_level_init();
/*
	  //Walk through PHY Registers for IP101A
		for(i=0;i<=16;i++){
			ret = ETH_ReadPHYRegister(IP101A_PHY_ADDRESS, i);//ret = ETH_ReadPHYRegister(BROADR_PHY_ADDRESS, i);
			printf("PHYReg(%d)=0x%04x\r\n",i,ret);
			delayms(100);
		}
*/
	seq=0;

	//Every Received Frames will be served at ETH_IRQHandler() in stm32f4x7_eth_bsp.c and startup_stm32f4xx.c.
	while (1)  {
		//for send test
		yEther_SimpleSend(seq);
		seq++;
		printf("TX\r\n");

/*
		while (ETH_CheckFrameReceived())   {// process received ethernet packet
					printf("=====main: ETH_CheckFrameReceived(%d)\r\n",seq);
					seq++;
					//LwIP_Pkt_Handle(); //move FIFO into pbuf, and handles it.
					ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
					ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
				}
*/
		delayms(1000);

		if(g_tickEvent){
			g_tickEvent = 0;
			printf("tick\r\n");
		}
	}

}
#endif
#endif

