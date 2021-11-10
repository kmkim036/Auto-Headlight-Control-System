// We found PENIRQ genetates when we depress. --?? debounce?
// XPT2046 = Ads7846
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6)|| (PROCESSOR == PROCESSOR_STM32F103RCT6)  || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "core_cm3.h" //for NVIC_DisableIRQ()
//#include "stm32f10x_bkp.h"
#include "misc.h"
#endif
#include "ySpiDrv.h"

typedef struct
{
	u16 X0;
	u16 Y0;
	u16 X;
	u16 Y;
	u8  Key_Sta;
	u8  Key_LSta;
        u8  noise;
        u16 time;
	float xfac;
	float yfac;
	short xoff;
	short yoff;
}Pen_Holder;
#if 0 //@yInc.h
struct _TouchScreen_posxy{
	u16 x;
	u16 y;
};
#endif

#define MAX_ADC_VAL 2047 //12bit ADC

//CONTROL FIELD
//PD[1:0]= 00=IRQ_ON; 01=Ref_OFF|ADC_ON|IRQ_OFF; 11=Ref_ON|ADC_ON|IRQ_OFF
#define XPT2046_CMD_READX 0x90
#define XPT2046_CMD_READY 0xD0
#define XPT2046_CMD_READZ 0xB0
#define XPT2046_CMD_READX_CONT 0x91 //IRQ_OFF
#define XPT2046_CMD_READY_CONT 0xD1 //IRQ_OFF
#define XPT2046_CMD_READZ_CONT 0xB1 //IRQ_OFF

#define Key_Down 0x01
#define Key_Up   0x00

//========
#if (PROCESSOR == PROCESSOR_STM32F401RET6)
//USE SPI1
extern void stmSpi1_Config(unsigned char nCS);
extern unsigned char stmSpi1RdByte();
extern void stmSpi1WrByte(unsigned char inbyte);

#elif (PROCESSOR == PROCESSOR_STM32F103RCT6)
//use SPI1
//use nCS  of A2 : 103RCT6
#define nCS_XPT_H {GPIO_SetBits(GPIOA, GPIO_Pin_2);}
#define nCS_XPT_L {GPIO_ResetBits(GPIOA, GPIO_Pin_2);}

extern unsigned char stmSpi1RdByte();
extern void stmSpi1WrByte(unsigned char inbyte);//Read Two Bytes
extern void stmSPI1_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);

inline unsigned char XPT2046_spiRDBYTE(void){
	return stmSpi1RdByte();
}
inline void XPT2046_spiWRBYTE(unsigned char val){
	stmSpi1WrByte(val);
}

#elif (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_GD32F130FX)
//use SPI2
//use nCS0 of PB12 : 103
#define nCS_XPT_H {GPIO_SetBits(GPIOB, GPIO_Pin_12);}
#define nCS_XPT_L {GPIO_ResetBits(GPIOB, GPIO_Pin_12);}

extern unsigned char stmSpi2RdByte();
extern void stmSpi2WrByte(unsigned char inbyte);//Read Two Bytes
extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);

inline unsigned char XPT2046_spiRDBYTE(void){
	return stmSpi2RdByte();
}
inline void XPT2046_spiWRBYTE(unsigned char val){
	stmSpi2WrByte(val);
}
#endif


#define XPT2046_PENIRQ_USE_YES 		1
#define XPT2046_PENIRQ_USE_NO 		0
#define XPT2046_PENIRQ_USE 			XPT2046_PENIRQ_USE_NO//XPT2046_PENIRQ_USE_YES //
//==========

Pen_Holder Pen_Point;
struct _TouchScreen_posxy g_TouchScreen_posxy;
unsigned char flag=0;
u16 LastX=0,LastY=0;
unsigned char g_bTouch = 0;

void XPT2046_irq_enable(void){
	#if (PROCESSOR == PROCESSOR_STM32F103RCT6)
		EXTI->IMR |= (EXTI_Line14); //Interrupt Mask Register
		//EXTI->EMR |= EXTI_Line14;
		//EXTI->FTSR |= EXTI_Line14; //Falling Trigger
		//EXTI->RTSR &= ~EXTI_Line14; //Rising Trigger
	#elif (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_GD32F130FX)
		EXTI->IMR |= (EXTI_Line13); //Interrupt Mask Register
		//EXTI->EMR |= EXTI_Line14;
		//EXTI->FTSR |= EXTI_Line14; //Falling Trigger
		//EXTI->RTSR &= ~EXTI_Line14; //Rising Trigger
	#endif
}
void XPT2046_irq_disable(void){
	#if (PROCESSOR == PROCESSOR_STM32F103RCT6)
		EXTI->IMR &= ~(EXTI_Line14);
		//EXTI->EMR &= ~EXTI_Line14;
		//EXTI->RTSR &= ~EXTI_Line14;
		//EXTI->FTSR &= ~EXTI_Line14;
	#elif (PROCESSOR == PROCESSOR_STM32F103C8T6)|| (PROCESSOR == PROCESSOR_GD32F130FX)
		EXTI->IMR &= ~(EXTI_Line13);
		//EXTI->EMR &= ~EXTI_Line13;
		//EXTI->RTSR &= ~EXTI_Line13;
		//EXTI->FTSR &= ~EXTI_Line13;
	#endif
}

//================================= TOUCH ====================================================
// 1-A2,A2,A0:Mode(0=12bitADC)-DifferentialReferenceMode(=0)-PowerDownMode1(=0)-PowerDownMode0(=0)
// REQ 1-A2A1A0:0-0-00                                                   PD[1:0]= 00=IRQ_ON; 01=Ref_OFF|ADC_ON|IRQ_OFF; 11=Ref_ON|ADC_ON|IRQ_OFF
//       000 = Temp0
//       001 = Y*     --> 1001:0000 (0x90)
//       010 =Vbat
//       011 = Z1     --> pressure 1011:0000 (0xB0)
//       100 = Z2
//       101 = X*    --> 1101:0000 (0xD0)
//       110 = AUXIN
//       111 = Temp1
// [RESP]
//       MSB12bit + 0000

//PEN_IRQ will generate on each WriteByte()
u16 XPT2046_Read_Single(unsigned char cmd)
{
	u16 val=0;

	nCS_XPT_L;
	XPT2046_spiWRBYTE(cmd);//stmSpi1WrByte(cmd);
	//somedelay(1000);
	val = XPT2046_spiRDBYTE();	val <<= 8;//val=stmSpi1RdByte();	val <<= 8;
	val += XPT2046_spiRDBYTE();//val+=stmSpi1RdByte();
	nCS_XPT_H;
	return val;
}

u16 XPT2046_getPosByPolling(struct _TouchScreen_posxy *posxy)
{
	unsigned short pressure;
	unsigned char done = 0;
	unsigned x_sum = 0;
	unsigned y_sum = 0;
	unsigned cnt = 0;
	unsigned x,y,z;

	while(!done){
		//delayms(1);
		//discard first 20 readings
		if(cnt < 20){//50){
			XPT2046_Read_Single(XPT2046_CMD_READX);
			XPT2046_Read_Single(XPT2046_CMD_READY);
			//XPT2046_Read_Single(XPT2046_CMD_READZ);
			x_sum = y_sum = 0;
		}else{
			x = XPT2046_Read_Single(XPT2046_CMD_READX) >> 4;
			x_sum += x;
			y = XPT2046_Read_Single(XPT2046_CMD_READY) >> 4;
			y_sum += y;
			//z = XPT2046_Read_Single(XPT2046_CMD_READZ);
			//printf("x,y,z=%u,%u,%u \r\n", x, y,z);

			if(cnt >= 50){//250){
				x_sum = x_sum/(cnt - 20);//50);
				y_sum = y_sum/(cnt - 20);//50);

				//XPT2046_Convert_to_orienation(x_sum, y_sum, posxy->x, posxy->y);
				done = 1;

			}
		}
		cnt++;
	}
	if(done){
		if((x_sum == 0) || (y_sum == 0))
			return 0;
		//printf("x_sum,y_sum=%u,%u\r\n", x_sum, y_sum);
		posxy->x = x_sum;
		posxy->y = y_sum;
		return 1;
	}else
		return 0;

}

unsigned char XPT2046_getPosByIrq(struct _TouchScreen_posxy *posxy)
{
	unsigned char retb = 0;
	if(g_bTouch){
		XPT2046_irq_disable();
		if(XPT2046_getPosByPolling(posxy)){
			printf("x,y =(%u,%u)\r\n", posxy->x, posxy->y);
			retb = 1;
		}
		g_bTouch = 0;
		XPT2046_irq_enable();
	}
	return retb;
}

#if 0
void XPT2046_update(void)
{
	unsigned short x,y;

	XPT2046_irq_disable();

	//x = XPT2046_Read_Single(XPT2046_CMD_READX_CONT); //91 = IRQ_OFF
	//y = XPT2046_Read_Single(XPT2046_CMD_READY_CONT); //D1 = IRQ_OFF

	x = XPT2046_Read_Single(XPT2046_CMD_READX) >> 4; //90
	y = XPT2046_Read_Single(XPT2046_CMD_READY) >> 4; //D0

	//if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)){ //If not a real touch
	//	XPT2046_irq_enable();
	//	printf("No Real Touch\r\n");
	//	return;
	//}
/*
	if((x==0) || (y==0) || (x==0x7FF8) || (y==0x7FF8))
	{
		XPT2046_irq_enable();
		return;
	}
*/
	//XPT2046_Convert_to_orienation(x, y, &m_tp.x, &m_tp.y);
	//m_tp.x += m_ts.offset_x;
	//m_tp.y += m_ts.offset_y;
	XPT2046_irq_enable();

	printf("XPT2046_update(X,Y)=%u,%u\r\n",x,y);
}
void XPT2046_get_point_adc(struct _TouchScreen_posxy *posxy){
	//posxy->x = m_tp.x;
	//posxy->y = m_tp.y;
}

u16 XPT2046_read_once(void)
{
	unsigned short pressure;
	nCS_XPT_L;

	Pen_Point.X=XPT2046_TPReadX();
	Pen_Point.Y=XPT2046_TPReadY();
	pressure = XPT2046_TPReadZ();

	nCS_XPT_H;
	//if(pressedZ > 100){
		printf("read_once(X,Y, Z)=%u,%u, %u\r\n",Pen_Point.X,Pen_Point.Y, pressure);
		return 1;
	//}else
	//	return 0;

	return 1;
}	 


u8 XPT2046_ReadCoord(void)
{
	u8 t,t1,count=0;
	u16 databuffer[2][10];
	u16 temp=0;	 
			  
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0)
		{
		XPT2046_read_once();
		while(count<10)
			{
				{	if(XPT2046_read_once())
					{
					databuffer[0][count]=Pen_Point.X;
					databuffer[1][count]=Pen_Point.Y;
					count++;
					}
				}
			}

	if(count==10)
		{
			do
				{
				t1=0;
				for(t=0;t<count-1;t++)
					{
					if(databuffer[0][t]>databuffer[0][t+1])
						{
						temp=databuffer[0][t+1];
						databuffer[0][t+1]=databuffer[0][t];
						databuffer[0][t]=temp;
						t1=1;
						}
					}
				}while(t1);
					do
					{
						t1=0;
						for(t=0;t<count-1;t++)
							{
							if(databuffer[1][t]>databuffer[1][t+1])
								{
								temp=databuffer[1][t+1];
								databuffer[1][t+1]=databuffer[1][t];
								databuffer[1][t]=temp;
								t1=1;
								}
							}
					}while(t1);
		 	 		  
					Pen_Point.X=((databuffer[0][3]+databuffer[0][4]+databuffer[0][5])/3);
					Pen_Point.Y=2047-((databuffer[1][3]+databuffer[1][4]+databuffer[1][5])/3);
					flag=1;
					return 1;
		}
		flag=0;
	}
	return 0;
}

/*
void Convert_Pos(void)
{
	char NMB[10];

	Read_Ads7846();
	Pen_Point.X0=(int)((Pen_Point.X-103)/7.7);
	Pen_Point.Y0=(int)((Pen_Point.Y-104)/5.56);
	if(Pen_Point.X0>240)
		{
		Pen_Point.X0=240;
		}
	if(Pen_Point.Y0>320)
		{
		Pen_Point.Y0=320;
		}

	Draw_Full_Rect(43, 295 ,61 ,205 , LCD_BLACK);
	Draw_Full_Rect(63, 295 ,81 ,205 , LCD_BLACK);
	Set_Font(&Font16x24);
	uint16tostr(NMB, Pen_Point.X0, 10);
	Display_String(43, 295, NMB, LCD_WHITE);
	uint16tostr(NMB, Pen_Point.Y0, 10);
	Display_String(63, 295, NMB, LCD_WHITE);
	//Draw_Pixel((240-Pen_Point.X0), Pen_Point.Y0, LCD_WHITE);
	Draw_Line((240-Pen_Point.X0),Pen_Point.Y0,(240-LastX),LastY,LCD_WHITE);
	LastX = Pen_Point.X0;
	LastY = Pen_Point.Y0;
	TIM4->CCR1 = (320-Pen_Point.Y0) * 1.5;

}
*/
//called by TP_IRQ on PC13
unsigned char XTP2046_Read(struct _TouchScreen_posxy *posxy) //replaced with XPT2046...
{
	u8 t,t1,count=0;
	u16 databuffer[2][10];
	u16 temp=0;

	//PC13 (TP_IRQ line is active low)
	//if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)==0)
	{
		if(XPT2046_read_once()){
			//TEST
			posxy->x = Pen_Point.X0;
			posxy->y = Pen_Point.Y0;
			return 1;
		}else
			return 0;


		while(count<10){ //read x and y, 10 times
			if(XPT2046_read_once()){
				databuffer[0][count]=Pen_Point.X;
				databuffer[1][count]=Pen_Point.Y;
				count++;
			}
		}

		if(count==10){
			//for x
			do{
				t1=0;
				for(t=0;t<count-1;t++){
					if(databuffer[0][t] > databuffer[0][t+1])	{
						temp=databuffer[0][t+1];
						databuffer[0][t+1]=databuffer[0][t];
						databuffer[0][t]=temp;
						t1=1;
					}
				}
			}while(t1);
			//for y
			do{
				t1=0;
				for(t=0;t<count-1;t++){
					if(databuffer[1][t] > databuffer[1][t+1]){
						temp=databuffer[1][t+1];
						databuffer[1][t+1]=databuffer[1][t];
						databuffer[1][t]=temp;
						t1=1;
					}
				}
			}while(t1);

			Pen_Point.X=((databuffer[0][3]+databuffer[0][4]+databuffer[0][5])/3);
			Pen_Point.Y=2047-((databuffer[1][3]+databuffer[1][4]+databuffer[1][5])/3);
			printf("x,y=(%d,%d)\r\n",Pen_Point.X,Pen_Point.Y);
			if((Pen_Point.X == 2047) && (Pen_Point.Y == 2047)){
				return 0;
			}
			else{
				//flag=1;
				Pen_Point.X0=(int)((Pen_Point.X-103)/5.56); //Pen_Point.X0=(int)((Pen_Point.X-103)/7.7);
				Pen_Point.Y0=(int)((Pen_Point.Y-104)/7.7); //Pen_Point.Y0=(int)((Pen_Point.Y-104)/5.56);
				if(Pen_Point.X0>320) Pen_Point.X0=320; //if(Pen_Point.X0>240) Pen_Point.X0=240;
				if(Pen_Point.Y0>240) Pen_Point.Y0=240; //if(Pen_Point.Y0>320) Pen_Point.Y0=320;

				posxy->x = Pen_Point.X0;
				posxy->y = Pen_Point.Y0;
				return 1;
			}
		}
		//else flag=0;
	}
	return 0;
}

u8 XPT2046_Convert_Pos(struct _TouchScreen_posxy *posxy)
{
	//if(!g_bTouch) return 0;

	if(XTP2046_Read(posxy) == 0){
		//g_bTouch = 0;
		printf("X,Y=%u,%u\r\n", posxy->x, posxy->y);
		return 0;
	}

	TIM4->CCR1 = (240 - posxy->y) * 1.5; //TIM4->CCR1 = (240-Pen_Point.Y0) * 1.5; //TIM4->CCR1 = (320-Pen_Point.Y0) * 1.5; //???
	return 1;
}

void lcd_FSMC_Action(struct _TouchScreen_posxy *posxy){
	char NMB[10];
	u16 x,y,tmpx,tmpy,dx,dy,tmpdx,tmpdy;
/*
	y = 238;
	x = 5;
	tmpy = 240 - y; //10
	tmpx = 320 - x; //310
	dx = 55;
	dy = 10; //actually -20

	Draw_Full_Rect(tmpy, tmpx ,tmpy+dy ,tmpx - dx , RGB_COL_BLACK); //delete screen
	//Draw_Full_Rect(43, 295 ,61 ,205 , RGB_COL_BLACK); //delete screen

	lcdSetTextFont(&Font8x8);//&Font16x24);
	uint16tostr(NMB, posxy->x, 10);
	lcdPrintString(tmpy, tmpx, NMB, RGB_COL_YELLOW); //lcdPrintString(43, 295, NMB, RGB_COL_WHITE);
	uint16tostr(NMB, posxy->y, 10);
	lcdPrintString(tmpy, tmpx-30, NMB, RGB_COL_YELLOW); //lcdPrintString(43, 225, NMB, RGB_COL_WHITE);//uint16tostr(NMB, Pen_Point.Y0, 10);	lcdPrintString(63, 295, NMB, RGB_COL_WHITE);
	//lcdDrawPixel((240-Pen_Point.X0), Pen_Point.Y0, RGB_COL_WHITE);
	//lcdDrawLine((240-Pen_Point.X0),Pen_Point.Y0,(240-LastX),LastY,RGB_COL_WHITE);
	//lcdDrawPixel((240-Pen_Point.X0), Pen_Point.Y0, RGB_COL_WHITE);
	lcdDrawLine(240-LastY,(320-LastX), 240-posxy->y, (320-posxy->x),RGB_COL_WHITE);

	if((LastX == posxy->x) && (LastY == posxy->y)){

	}else{
		LastX = posxy->x;
		LastY = posxy->y;
	}

	if((posxy->x > 300) && (posxy->y < 20))
		  lcdFillRGB(0x0000);
*/
}



#endif

#if(XPT2046_PENIRQ_USE == XPT2046_PENIRQ_USE_YES)
//PC14 = TP_IRQ
void EXTI15_10_IRQHandler(void)
{
	#if (PROCESSOR == PROCESSOR_STM32F103RCT6)
	if(EXTI_GetITStatus(EXTI_Line14) != RESET) {
		if(g_bTouch == 0){
			g_bTouch = 1;
			printf("IRQ ");
			//GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
		}
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
	#elif (PROCESSOR == PROCESSOR_STM32F103C8T6)
	if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
		if(g_bTouch == 0){
			g_bTouch = 1;
			printf("IRQ ");
			//GPIO_ToggleBits(GPIOC, GPIO_Pin_14); //LED Blinking
		}
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
	#endif
}
#endif

void XPT2046_touch_init(unsigned char bConfigNewSpi)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;

  if (bConfigNewSpi){
#if (PROCESSOR == PROCESSOR_STM32F103C8T6)|| (PROCESSOR == PROCESSOR_GD32F130FX)
	  //SPI2. Use nCS0(PB12=NCS0, Mode 3, 8bit Mode,
	  stmSPI2_Config(
  		1, //1Mbps
  		0, //use nCS0 (PB12)
  		3, //Mode 3
  		8); //data 8 bit (or 16)
	  printf("XPT2046 TouchSensor with SPI2 Mode 3, 1Mbps, 8bit data.");
#else
	  //SPI1. Use nCS0(PA2=NCS, Mode 3, 8bit Mode,
	  stmSPI1_Config(
  		1, //1Mbps
  		0xFF, //nCS will be set after this SPI1 config.
  		3, //Mode 3
  		8); //data 8 bit (or 16)
	  printf("XPT2046 TouchSensor with SPI1 Mode 3, 1Mbps, 8bit data.");
#endif
  }

  //TP_CS : PA2
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_2); //nCS_XPT_H;

#if (XPT2046_PENIRQ_USE == XPT2046_PENIRQ_USE_YES)
	#if (PROCESSOR == PROCESSOR_STM32F103RCT6)
  	  //T_PEN_IRQ @ PC14
  	  // Enable GPIOC clock
  	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
      //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
      //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  	  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  	  //Use PC14,PC15 as a normal IO
  	  //PWR_BackupAccessCmd(ENABLE); //we need to handle LSE Off
  	  //RCC_LSEConfig(RCC_LSE_OFF);
  	  //BKP_TamperPinCmd(DISABLE);
  	  //Configure PC14 pin as input floating
  	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
  	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING; //
  	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //max 2MHz
  	  GPIO_Init(GPIOC, &GPIO_InitStructure);

  	  //PWR_BackupAccessCmd(DISABLE);

  	  // Connect EXTI Line14 to PC14 pin
  	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //Need for using EXTI
  	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);

  	  //Configure EXTI Line14
  	  //XPT2046_irq_enable();
  	  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	  EXTI_Init(&EXTI_InitStructure);

	#elif (PROCESSOR == PROCESSOR_STM32F103C8T6)
	  //T_PEN_IRQ @ PC13

	  // Enable GPIOC clock
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
	  //Configure PC13 pin as input floating
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING; //
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  // Connect EXTI Line13 to PC13 pin
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //Need for using EXTI
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

	  //Configure EXTI Line13
	  //XPT2046_irq_enable();
	  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
	#endif

	// Enable and set EXTI Interrupt to the lowest priority
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#endif
	XPT2046_Read_Single(XPT2046_CMD_READX); //do dummy read to generate IRQ
}

void stmXPT2046Touch_Loop()
{
    unsigned short retVal,retVal_MSB,retVal_LSB;
    struct _TouchScreen_posxy posxy;

    XPT2046_touch_init(1);

    while(1){
		#if(XPT2046_PENIRQ_USE == XPT2046_PENIRQ_USE_NO)
    	if(XPT2046_getPosByPolling(&posxy))
    		printf("x,y =(%u,%u)\r\n", posxy.x, posxy.y);
		#else
    	  	XPT2046_getPosByIrq(&posxy);
		#endif

#if 0
    	XPT2046_Read_Single(XPT2046_CMD_READX);
    	if(g_bTouch){ //set by IRQ Handler
    		g_bTouch = 0;
    		//XPT2046_update();

    		//XPT2046_get_point_adc(&p);
    		//XPT2046_get_point_pixel(&p);

    	}

    	//XPT2046_ReadCoord();
    	//NVIC_DisableIRQ(EXTI15_10_IRQn);
    	//XPT2046_read_once();
    	//NVIC_EnableIRQ(EXTI15_10_IRQn);
    	printf("Loop");
    	delayms(500);
#endif
    }
}
