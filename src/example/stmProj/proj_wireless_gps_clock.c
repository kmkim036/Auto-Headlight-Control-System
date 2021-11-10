/*
  LedControl.cpp - A library for controling Leds with a MAX7219/MAX7221
 *    Copyright (c) 2007 Eberhard Fahle
 *
 *    Modified for STM32F4xx by Arami
 * 	  SPI : 1Mbps, 8 bit mode
 *    Connections to MAX7219:
// SPI MODE = 0
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0CLK - PA2
//! - SSI0Fss - PA3 nCS0
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5

 *  Configures the SPI3 Peripheral.
 *  CLK - PC10
 *  MISO - PC11 -- NOT USED
 *  MOSI - PC12
 *  nCS1 - PD2
*/
/*
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "yInc.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
*/

#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#include "yInc.h"
#include "ySpiDrv.h"
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//|                 |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   | 103
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| ULED            | PB14      |PC4        |PE15       | <==     | PG7     |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| BUTTON          |           |PC5(H)     |PD11(index)| PD11(L) |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| BEEP            |           |PB13       |PD14       | <==     |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| QEI             |           |PB0,1,12   |PD12,13,11 | PD12,13 |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| nCS0            |           |           |           | PD10    |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| nCS1            |           |           |           | PD15    |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//| nCS2            |           |           |           | PE5     |         |
//+-----------------+-----------+-----------+-----------+---------+---------+--------+
//*H= Active High/L=Active Low
//*F= FallingEdge

extern unsigned char g_bI2CModuleConfigDone;
extern struct _ppsHoldOver g_ppsHoldOver;
extern volatile uint32_t g_tickCnt;
extern char stmGpsDisplayTimeOfDay(unsigned char uartId, struct minmea_sentence_rmc *rmc);
extern unsigned char RTC_DS3231_isrunning(void);

//PB3 - GPSpps
#if (USE_EXTI3 	== USE_EXTI3_GPSPPS)
void EXTI3_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line3);

		if(g_ppsHoldOver.lost_GPS_PPS){ //was lost, now received.
			g_ppsHoldOver.lost_GPS_PPS = 0;
		}
		g_ppsHoldOver.GpsPpsTick = g_tickCnt;// set current tick
	}
}
#endif

#if (USE_EXTI4 == USE_EXTI4_GPSPPS)
//PB4 - RTCpps
void EXTI4_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line4);
		g_ppsHoldOver.showTimeByRTCperSec = 1;
		// Toggle LED0
		GPIO_ToggleBits(GPIOC, GPIO_Pin_13); // ULED ON (PC13)
	}
}
#endif

#if (USE_EXTI9_5 ==	USE_EXTI9_5_RTC )
//PA8-RTCpps
void EXTI9_5_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line8);		// Clear the EXTI line 8 pending bit

		if(g_ppsHoldOver.lost_GPS_PPS){ //was GPS lost, still lost. Thus we inform advance 1 sec by RTC.
			g_ppsHoldOver.showTimeByRTCperSec = 1;
		}

		//g_ppsHoldOver.RtcPpsTick = g_tickCnt + 100;//reload for next pps


		// Toggle LED0
		GPIO_ToggleBits(GPIOC, GPIO_Pin_13); // ULED ON (PC13)
		//g_irq_ap3216 = 1;
	}
}
#endif

void proj_wireless_gps_clock_gpio_irq_config(void)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
    EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	//a) ULED-pc13 setup
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
	  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_13;//GPIO_Pin_All ;
	  GPIO_Init(GPIOC, &GPIO_InitStruct);

	  GPIO_SetBits(GPIOC, GPIO_Pin_13);
	  delayms(1000);//somedelay(1000000);
	  GPIO_ResetBits(GPIOC, GPIO_Pin_13);//

	//b) PPS pins config(PB4-GPSpps and PA8-RTCpps)
	//PB3-GPSpps //was PB4-GPSpps
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3; //GPIO_Pin_4;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStruct);

	  //INT for PB3//PB4
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);// GPIO_PinSource4);// Connect EXTI Line4 to PB4 pin
	  EXTI_InitStructure.EXTI_Line = EXTI_Line3; //= EXTI_Line4;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; //= EXTI4_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  //c) PB4//was PA8-RTCpps
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4; //GPIO_Pin_8;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStruct);//GPIO_Init(GPIOA, &GPIO_InitStruct);

	  //INT for PB4//PA8
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4); //GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
	  EXTI_InitStructure.EXTI_Line = EXTI_Line4; //EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn; //NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}

void proj_wireless_gps_clock_pong(void)
{
	char str[16];
	char ret;
	struct minmea_sentence_rmc rmc;
	char currTimeStr[16];
	char retPingStatus;
	unsigned int unixTime, nbUnixTime;

	tTod currTod;

	//(a) Check GPS
	if(!g_ppsHoldOver.firstToDSetDoneByGps)	{
		printf("Please connect GPS\r\n");
		SSD1306_OLED_printString_8X8("NoGPS",10,0,15);
	}

	//(b) Check Lost GPS...
	if(g_tickCnt > (g_ppsHoldOver.GpsPpsTick + 110)){ //No GPS PPS in 1100msec from the last PPS.
		g_ppsHoldOver.lost_GPS_PPS = 1;
		SSD1306_OLED_printString_8X8("NoGPS",10,0,15);
	}else{
		g_ppsHoldOver.lost_GPS_PPS = 0;
		SSD1306_OLED_printString_8X8("  GPS",10,0,15);
	}

	//set RTC using RMC msg.
	if(!g_ppsHoldOver.lost_GPS_PPS)
	{
		if(stmGpsGetTimeOfDay(2, &rmc) == ERR_OK)
		{
			g_ppsHoldOver.DateTime.yOff = rmc.date.year;
			g_ppsHoldOver.DateTime.m = rmc.date.month;
			g_ppsHoldOver.DateTime.date = rmc.date.day;
			g_ppsHoldOver.DateTime.hh = rmc.time.hours;
			g_ppsHoldOver.DateTime.mm = rmc.time.minutes;
			g_ppsHoldOver.DateTime.ss = rmc.time.seconds;

			//is it the fresh new born gps time?
			if(!g_ppsHoldOver.firstToDSetDoneByGps)
			{
				g_ppsHoldOver.firstToDSetDoneByGps = 1;
				RTC_DS3231_Adjust(&g_ppsHoldOver.DateTime); //set current on rtc. (Too frequent adjust the RTC, thus we will update a single update per day)
				printf("FIRST RMCTIME=%u.%u.%u %u:%u:%u\r\n", g_ppsHoldOver.DateTime.yOff + 2000,g_ppsHoldOver.DateTime.m,g_ppsHoldOver.DateTime.date,g_ppsHoldOver.DateTime.hh,g_ppsHoldOver.DateTime.mm, g_ppsHoldOver.DateTime.ss);
			}else{
				//RTC_DS3231_Adjust(&g_ppsHoldOver.DateTime); //set per day.... TBD
			}

			//stmGpsDisplayTimeOfDay(2, &rmc);
			//SSD1306_OLED_printString_8X8("by GPS",1,5,15);
			printf("RMCTIME=%u.%u.%u %u:%u:%u\r\n", rmc.date.year + 2000,rmc.date.month,rmc.date.day,rmc.time.hours,rmc.time.minutes, rmc.time.seconds);
		}
	}

	//prepare current time
    if(g_ppsHoldOver.showTimeByRTCperSec){ //show Time per 1sec by RTC
		g_ppsHoldOver.showTimeByRTCperSec = 0; //for next round (set by RTC PPS IRQ)
		RTC_DS3231_now(&g_ppsHoldOver.DateTime);//Get RTC time

		printf("NowTime = ");
		printf("%u/%u/%u",g_ppsHoldOver.DateTime.yOff + 2000, g_ppsHoldOver.DateTime.m, g_ppsHoldOver.DateTime.date);
		printf("(%s) ", getStr_DaysOfTheWeek(&g_ppsHoldOver.DateTime));//daysOfTheWeek[DateTime_dayOfTheWeek(&DateTime)]);
		printf(" %u:%u:%u\r\n",g_ppsHoldOver.DateTime.hh,g_ppsHoldOver.DateTime.mm,g_ppsHoldOver.DateTime.ss);
		//=== unix time since 1970.1.1 midnight
		unixTime = DateTime_unixtime(&g_ppsHoldOver.DateTime);
		printf("UnixTime= %us\r\n", unixTime );

		SSD1306_OLED_showDate(g_ppsHoldOver.DateTime.yOff, g_ppsHoldOver.DateTime.m, g_ppsHoldOver.DateTime.date, 0, 1);
		SSD1306_OLED_showClock(g_ppsHoldOver.DateTime.hh,g_ppsHoldOver.DateTime.mm, g_ppsHoldOver.DateTime.ss, 0, 2);
    }

	//pong if ping request present.
#if 0
	sprintf(currTimeStr, "%02u:%02u:%02u",g_ppsHoldOver.DateTime.hh,g_ppsHoldOver.DateTime.mm,g_ppsHoldOver.DateTime.ss);
	stm_RF24_6Star_Pong(curTimeStr, 0);
#else
	unixTime = DateTime_unixtime(&g_ppsHoldOver.DateTime);
	nbUnixTime = htonl(unixTime);
	stm_RF24_6Star_Pong(NULL, &nbUnixTime);
#endif
}

void proj_wireless_gps_clock_ping()
{
	char retPingStatus;
	tTod currTod;

	//send ping
#if 0
	char currTimeStr[16];

	retPingStatus = stm_RF24_6Star_Ping("Time?", currTimeStr, NULL);
#else
	unsigned int unixTime, nbUnixTime;
	retPingStatus = stm_RF24_6Star_Ping("Time?", NULL, &nbUnixTime);
#endif

	//on receiving the pong
	if(retPingStatus == ERR_OK){
#if 0
	 	printf("Time=%s\r\n", curTimeStr);
#else
	 	unixTime = ntohl(nbUnixTime);
	    ulocaltime(unixTime, &currTod);

	    g_ppsHoldOver.DateTime.yOff = currTod.usYear;
	    g_ppsHoldOver.DateTime.m = currTod.ucMon;
	    g_ppsHoldOver.DateTime.date = currTod.ucMday;
	    g_ppsHoldOver.DateTime.hh = currTod.ucHour;
	    g_ppsHoldOver.DateTime.mm = currTod.ucMin;
	    g_ppsHoldOver.DateTime.ss = currTod.ucSec;

	    //set time in RTC
	    RTC_DS3231_Adjust(&g_ppsHoldOver.DateTime); //set current on rtc. (Too frequent adjust the RTC, thus we will update a single update per day)
#endif
	 }else{
	 	printf("Err on Ping.. Show time in RTC\r\n");

	 }
	 //show time in RTC
	 RTC_DS3231_now(&g_ppsHoldOver.DateTime);//Get RTC time

	printf("NowTime = ");
	printf("%u/%u/%u",g_ppsHoldOver.DateTime.yOff + 2000, g_ppsHoldOver.DateTime.m, g_ppsHoldOver.DateTime.date);
	printf("(%s) ", getStr_DaysOfTheWeek(&g_ppsHoldOver.DateTime));//daysOfTheWeek[DateTime_dayOfTheWeek(&DateTime)]);
	printf(" %u:%u:%u\r\n",g_ppsHoldOver.DateTime.hh,g_ppsHoldOver.DateTime.mm,g_ppsHoldOver.DateTime.ss);
		//=== unix time since 1970.1.1 midnight
	unixTime = DateTime_unixtime(&g_ppsHoldOver.DateTime);
	printf("UnixTime= %us\r\n", unixTime );

	SSD1306_OLED_showDate(g_ppsHoldOver.DateTime.yOff, g_ppsHoldOver.DateTime.m, g_ppsHoldOver.DateTime.date, 0, 1);
	SSD1306_OLED_showClock(g_ppsHoldOver.DateTime.hh,g_ppsHoldOver.DateTime.mm, g_ppsHoldOver.DateTime.ss, 0, 2);

	delayms(1000);
}

//if (RTC_DS3231_isrunning())
//   g_ppsHoldOver.firstToDSetDoneByGps = 1;

//return 1 if already configured.
unsigned char proj_wireless_gps_clock_rtc_config(void)
{
	unsigned char is_running = 0;
	//
	g_ppsHoldOver.GpsPpsTick = 0;
//	g_ppsHoldOver.RtcPpsTick = 0;
	g_ppsHoldOver.lost_GPS_PPS = 0;
	g_ppsHoldOver.showTimeByRTCperSec = 0;

    //DS3231
    if (RTC_DS3231_isrunning()){ //==1 --> Osc  Running.
	    printf("DS3231 is running\r\n");
	    return 1;
    }

    //on stop
	printf("DS3231 Config with I2C\r\n");
	g_ppsHoldOver.firstToDSetDoneByGps = 0;
	g_ppsHoldOver.DateTime.yOff = 21;
	g_ppsHoldOver.DateTime.m = 1;
	g_ppsHoldOver.DateTime.date = 1;
	g_ppsHoldOver.DateTime.day = 1;
	g_ppsHoldOver.DateTime.hh = 0;
	g_ppsHoldOver.DateTime.mm = 0;
	g_ppsHoldOver.DateTime.ss = 0;

	is_running = DS3231_setup(&g_ppsHoldOver.DateTime);
	return is_running;
}

void proj_wireless_gps_clock(){
	unsigned char role; //role (ping = 0; pong = 1)
	unsigned char is_running = 0;

	if(!g_bI2CModuleConfigDone){
		printf("Not I2C installed\r\n");
		return;
	}

	proj_wireless_gps_clock_gpio_irq_config();
	is_running = proj_wireless_gps_clock_rtc_config();
	role = stm_RF24_6StarConfig(); //role (ping = 0; pong = 1)

   //GPS
    if(role){ //pong
    	printf("GPS Config with USART2\r\n"); //PA2 and PA3
    	stmGpsConfig(2);//unsigned char uartId);
    }

	while(1){

	    if(role) //PONG...
	    {
	    	proj_wireless_gps_clock_pong();

	    }else{ //PING per 1sec
	    	proj_wireless_gps_clock_ping();

	    }
	}
}

