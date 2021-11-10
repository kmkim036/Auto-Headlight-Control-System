/* The screen is divided  into “8 pages” (I know it’s not divided into rows, but pages), and 128 columns.
For example, display a dot on the top left corner, we need to send a hex number 0x01 (B0000 0001) to the data register

  stmAs5600 - 0.96' I2C 128x64 OLED Driver Library
  Driver = SSD1306
  I2C 8Bit Address = 0x78.
  2014 Copyright (c) OscarLiang.net  All right reserved.

  Author: Oscar Liang
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

#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
#include "lwipopts.h"
*/
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
//#include "stm32f10x_flash.h"

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern void delayms(uint32_t ms);
extern I2C_TypeDef *gI2Cx;

#define AS5600_ADDRESS					0x6C //0x36 = 7bit address(0011 0110) --> 0110 1100 0x6C (8 bit addr)
#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz

//AS5600 Registers
#define AS5600_REG_ZMCO 0x00
#define AS5600_REG_ZPOS_MSB 0x01 //START POSITION
#define AS5600_REG_ZPOS_LSB 0x02
#define AS5600_REG_MPOS_MSB 0x03 //STOP POSITION
#define AS5600_REG_MPOS_LSB 0x04
#define AS5600_REG_MANG_MSB 0x05 //MAXIMUM ANGLE
#define AS5600_REG_MANG_LSB 0x06
#define AS5600_REG_CONF_MSB 0x07
#define AS5600_REG_CONF_LSB 0x08
#define AS5600_REG_RAW_ANGLE_MSB 0x0c //MSB
#define AS5600_REG_RAW_ANGLE_LSB 0x0d //LSB
#define AS5600_REG_ANGLE_MSB 0x0e
#define AS5600_REG_ANGLE_LSB 0x0f
#define AS5600_REG_STATUS 0x0b
#define AS5600_REG_AGC 0x1a
#define AS5600_REG_MAGNITUDE_MSB 0x1b
#define AS5600_REG_MAGNITUDE_LSB 0x1c
#define AS5600_REG_BURN 0xff

struct stmAs5600 {
    unsigned short _rawStartAngle;
    unsigned short _zPosition;
    unsigned short _rawEndAngle;
    unsigned short _mPosition;
    unsigned short _maxAngle;
} g_As5600;  // stmAs5600 object

unsigned short stmAs5600getRawAngle();
unsigned short stmAs5600readTwoBytes(int regAddr_msb, int regAddr_lsb);

/*******************************************************
/* In: new maximum angle to set OR none
/* Out: value of max angle register
/* Sets a value in maximum angle register.
/* If no value is provided, method will read position of
/* magnet.  Setting this register zeros out max position
/* register.
/*******************************************************/
unsigned short stmAs5600setMaxAngle(unsigned short newMaxAngle)
{
  unsigned short retVal;
  if(newMaxAngle == -1)
  {
	  g_As5600._maxAngle = stmAs5600getRawAngle();
  }
  else
	  g_As5600._maxAngle = newMaxAngle;

  stmAs5600writeOneByte(AS5600_REG_MANG_MSB, g_As5600._maxAngle >> 8);
  delayms(2);
  stmAs5600writeOneByte(AS5600_REG_MANG_LSB, (g_As5600._maxAngle)&0x00ff);
  delayms(2);

  retVal = stmAs5600readTwoBytes(AS5600_REG_MANG_MSB, AS5600_REG_MANG_LSB);
  return retVal;
}

/*******************************************************
/* Gets value of maximum angle register.
/*******************************************************/
unsigned short stmAs5600getMaxAngle()
{
  return stmAs5600readTwoBytes(AS5600_REG_MANG_MSB, AS5600_REG_MANG_LSB);
}

/*******************************************************
/* In: new start angle position
/* Out: value of start position register
/* Sets a value in start position register.If no value is provided, method will read position of magnet.
/*******************************************************/
unsigned short stmAs5600setStartPosition(unsigned short startAngle)
{
  if(startAngle == -1)
  {
	  g_As5600._rawStartAngle = stmAs5600getRawAngle();
  }
  else
	  g_As5600._rawStartAngle = startAngle;

  stmAs5600writeOneByte(AS5600_REG_ZPOS_MSB, g_As5600._rawStartAngle >> 8);
  delayms(2);
  stmAs5600writeOneByte(AS5600_REG_ZPOS_LSB, g_As5600._rawStartAngle & 0x00ff);
  delayms(2);
  g_As5600._zPosition = stmAs5600readTwoBytes(AS5600_REG_ZPOS_MSB, AS5600_REG_ZPOS_LSB);

  return(g_As5600._zPosition);
}

/*******************************************************
/* Gets value of start position register.
/*******************************************************/
unsigned short stmAs5600getStartPosition()
{
  return stmAs5600readTwoBytes(AS5600_REG_ZPOS_MSB, AS5600_REG_ZPOS_LSB);
}

/*******************************************************
/* In: new end angle position
/* Out: value of end position register
/* Sets a value in end position register. If no value is provided, method will read position of magnet.
/*******************************************************/
unsigned short stmAs5600setEndPosition(unsigned short endAngle)
{
  if(endAngle == -1)
    g_As5600._rawEndAngle = stmAs5600getRawAngle();
  else
	  g_As5600._rawEndAngle = endAngle;

  stmAs5600writeOneByte(AS5600_REG_MPOS_MSB, g_As5600._rawEndAngle >> 8);
  delayms(2);
  stmAs5600writeOneByte(AS5600_REG_MPOS_LSB, g_As5600._rawEndAngle & 0x00ff);
  delayms(2);
  g_As5600._mPosition = stmAs5600readTwoBytes(AS5600_REG_MPOS_MSB, AS5600_REG_MPOS_LSB);

  return(g_As5600._mPosition);
}

/*******************************************************
/* Gets value of end position register.
/*******************************************************/
unsigned short stmAs5600getEndPosition()
{
  unsigned short retVal = stmAs5600readTwoBytes(AS5600_REG_MPOS_MSB, AS5600_REG_MPOS_LSB);
  return retVal;
}

/*******************************************************
/* Out: value of raw angle register
/* Gets raw value of magnet position.
/* start, end, and max angle settings do not apply
/*******************************************************/
unsigned short stmAs5600getRawAngle(){
	unsigned short rawAngle;
	rawAngle = stmAs5600readTwoBytes(AS5600_REG_RAW_ANGLE_MSB, AS5600_REG_RAW_ANGLE_LSB);
	//printf("rawAngle=%x\r\n",rawAngle);

	return (rawAngle & 0x0fff); //12 bits
}

/*******************************************************
/* Out: value of scaled angle register
/* Gets scaled value of magnet position.
/* start, end, or max angle settings are used to determine value
/*******************************************************/
unsigned short stmAs5600getScaledAngle()
{
  return stmAs5600readTwoBytes(AS5600_REG_ANGLE_MSB, AS5600_REG_ANGLE_LSB);
}

/*******************************************************
/* Out: 1 if magnet is detected, 0 if not
/* Reads status register and examines the MH bit
/*******************************************************/
int stmAs5600detectMagnet()
{
  int magStatus;
  int retVal = 0;
  /*0 0 MD ML MH 0 0 0*/
  /* MD high = AGC minimum overflow, Magnet to strong */
  /* ML high = AGC Maximum overflow, magnet to weak*/
  /* MH high = magnet detected*/
  magStatus = stmAs5600readOneByte(AS5600_REG_STATUS);

  if(magStatus & 0x20)
    retVal = 1;

  return retVal;
}

/*******************************************************
/* Out: 0 if no magnet is detected
/*      1 if magnet is to weak
/*      2 if magnet is just right
/*      3 if magnet is to strong
/* Reads status register andexamins the MH,ML,MD bits
/*******************************************************/
int stmAs5600getMagnetStrength()
{
  int magStatus;
  int retVal = 0;
  /*0 0 MD ML MH 0 0 0*/
  /* MD high = AGC minimum overflow, Magnet to strong */
  /* ML high = AGC Maximum overflow, magnet to weak*/
  /* MH high = magnet detected*/
  magStatus = stmAs5600readOneByte(AS5600_REG_STATUS);
  if(detectMagnet() ==1)
  {
      retVal = 2; /*just right */
      if(magStatus & 0x10)
        retVal = 1; /*to weak */
      else if(magStatus & 0x08)
        retVal = 3; /*to strong */
  }

  return retVal;
}

/*******************************************************
Gets value of AGC register.
******************************************************/
int stmAs5600getAgc()
{
  return stmAs5600readOneByte(AS5600_REG_AGC);
}

/*******************************************************
 Description: gets value of magnitude register.
*******************************************************/
unsigned short stmAs5600getMagnitude()
{
  return stmAs5600readTwoBytes(AS5600_REG_MAGNITUDE_MSB, AS5600_REG_MAGNITUDE_LSB);
}

/*******************************************************
/* Method: getBurnCount
/* In: none
/* Out: value of zmco register
/* Description: determines how many times chip has been
/* permanently written to.
/*******************************************************/
int stmAs5600getBurnCount()
{
  return stmAs5600readOneByte(AS5600_REG_ZMCO);
}

/*******************************************************
/* Method: stmAs5600burnAngle
/* In: none
/* Out: 1 success
/*     -1 no magnet
/*     -2 burn limit exceeded
/*     -3 start and end positions not set (useless burn)
/* Description: burns start and end positions to chip.
/* THIS CAN ONLY BE DONE 3 TIMES
/*******************************************************/
int stmAs5600stmAs5600burnAngle()
{
  int retVal = 1;
  g_As5600._zPosition = getStartPosition();
  g_As5600._mPosition = getEndPosition();
  g_As5600._maxAngle  = getMaxAngle();

  if(detectMagnet() == 1)
  {
    if(getBurnCount() < 3)
    {
      if((g_As5600._zPosition == 0)&&(g_As5600._mPosition ==0))
        retVal = -3;
      else
    	  stmAs5600writeOneByte(AS5600_REG_BURN, 0x80);
    }
    else
      retVal = -2;
  }
  else
    retVal = -1;

  return retVal;
}

/*******************************************************
/* Method: burnMaxAngleAndConfig
/* In: none
/* Out: 1 success
/*     -1 burn limit exceeded
/*     -2 max angle is to small, must be at or above 18 degrees
/* Description: burns max angle and config data to chip.
/* THIS CAN ONLY BE DONE 1 TIME
/*******************************************************/
int stmAs5600burnMaxAngleAndConfig()
{
  int retVal = 1;
  g_As5600._maxAngle  = getMaxAngle();

  if(stmAs5600getBurnCount() ==0)
  {
    if(g_As5600._maxAngle*0.087 < 18)
      retVal = -2;
    else
    	stmAs5600writeOneByte(AS5600_REG_BURN, 0x40);
  }
  else
    retVal = -1;

  return retVal;
}


/*******************************************************
/* Method: readOneByte
/* In: register to read
/* Out: data read from i2c
/* Description: reads one byte register from i2c
/*******************************************************/
int stmAs5600readOneByte(int regAddr)
{
  unsigned char retVal;

  stm_I2C_ReceiveBurstWithRestartCondition(AS5600_ADDRESS, regAddr, &retVal, 1);

  return retVal;
}

/*******************************************************
/* Method: readOneByte
/* In: two registers to read
/* Out: data read from i2c as a unsigned short
/* Description: reads two bytes register from i2c
/*******************************************************/
unsigned short stmAs5600readTwoBytes(int regAddr_msb, int regAddr_lsb)
{
  unsigned short retVal;
  unsigned short lsb,msb;
  unsigned char twobytes[2];
  //stm_I2C_ReceiveBurstWithRestartCondition(AS5600_ADDRESS, regAddr_msb, &twobytes, 2);
  //msb = twobytes[0];
  //lsb = twobytes[1];
  //msb = twobytes[0] * 256;
  //retVal = msb + twobytes[1];

  // Read High Byte
  stm_I2C_ReceiveBurstWithRestartCondition(AS5600_ADDRESS, regAddr_msb, &twobytes[0], 1);
  // Read Low Byte @ +1 address
  stm_I2C_ReceiveBurstWithRestartCondition(AS5600_ADDRESS, regAddr_lsb, &twobytes[1], 1);

  msb = (unsigned short)twobytes[0] * 256;
  retVal = msb + twobytes[1];

  //printf("msb = 0x%02x; lsb = 0x%02x; retVal=%04x\r\n",twobytes[0],twobytes[1], retVal);
  return retVal;
}

/*******************************************************
/* Method: writeOneByte
/* In: address and data to write
/* Out: none
/* Description: writes one byte to a i2c register
/*******************************************************/
void stmAs5600writeOneByte(int regAddr, int val)
{
	unsigned char send_buf[2];
	send_buf[0] = regAddr;
	send_buf[1] = val;
	stm_I2C_SendBurst(AS5600_ADDRESS, &send_buf[0], 2);
}

void stmAs5600printMenu()
{
  printf("AS5600 Menu.\r\n");

  printf("1 - Set start position\t|  "); printf(" 6 - get MPOS\r\n");
  printf("2 - Set end position\t|  ");   printf(" 7 - get raw angle\r\n");
  printf("3 - Set max angle range\t|  ");  printf(" 8 - get scaled angle\r\n");
  printf("4 - Get max angle range\t|  ");  printf(" 9 - detect magnet\r\n");
  printf("5 - Get ZPOS \t\t|  ");     printf("10 - get magnet strength\r\n");
  //printf();
  printf("Number of burns remaining: "); printf("%d\r\n",3 - stmAs5600getBurnCount());
  printf("96 - Burn Angle\r\n");
  printf("98 - Burn Settings (one time)\r\n");

  printf("TBD\r\n");
}

/*******************************************************
/* Out: human readable degrees as float
/*******************************************************/
//float stmAs5600convertRawAngleToDegrees(unsigned short newAngle)
unsigned short stmAs5600convertRawAngleToDegrees(unsigned short newAngle)
{
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  //float retVal = newAngle * 0.087;
	unsigned int angle;
	unsigned short retVal;
	angle = (unsigned int) newAngle;
	retVal = (unsigned short)((angle * 360)/4096);
	return retVal;
}

/*******************************************************
/* In: angle data from AMS_5600::getScaledAngle
/* Out: human readable degrees as float
/* Description: takes the scaled angle and calculates
/*******************************************************/
float stmAs5600convertScaledAngleToDegrees(unsigned short newAngle)
{
  unsigned short startPos = stmAs5600getStartPosition();
  unsigned short endPos = stmAs5600getEndPosition();
  unsigned short maxAngle = stmAs5600getMaxAngle();

  float multipler = 0;

  /* max angle and end position are mutually exclusive*/
  if(maxAngle >0)
  {
    if(startPos == 0)
      multipler = (maxAngle*0.0878)/4096;
    else  /*startPos is set to something*/
      multipler = ((maxAngle*0.0878)-(startPos * 0.0878))/4096;
  }
  else
  {
    if((startPos == 0) && (endPos == 0))
      multipler = 0.0878;
    else if ((startPos > 0 ) && (endPos == 0))
      multipler = ((360 * 0.0878) - (startPos * 0.0878)) / 4096;
    else if ((startPos == 0 ) && (endPos > 0))
      multipler = (endPos*0.0878) / 4096;
    else if ((startPos > 0 ) && (endPos > 0))
      multipler = ((endPos*0.0878)-(startPos * 0.0878))/ 4096;
  }
  return (newAngle * multipler);
}

/*******************************************************
Burn angle data to AMS5600
*******************************************************/
char *stmAs5600burnAngle()
{
  int burnResult = stmAs5600stmAs5600burnAngle();
  char *returnStr = "Brun angle error: ";
/*
  switch (burnResult)
  {
    case 1:
      returnStr = "Brun angle success";
      break;
    case -1:
      returnStr += "no magnet detected";
      break;
    case -2:
      returnStr += "no more burns left";
      break;
    case -3:
      returnStr += "no positions set";
      break;
    default:
      returnStr += "unknown";
      break;
  }
*/
  return returnStr;
}

/*******************************************************
Burn max angle and config data to AMS5600
******************************************************/
char *burnMaxAngleAndConfig()
{
  int burnResult = stmAs5600burnMaxAngleAndConfig();
  char *retStr = "Brun max angle and config error: ";
/*
  switch(burnResult)
  {
    case 1:
      retStr = "Brun max angle and config success";
      break;
    case -1:
      retStr += "chip has been burned once already";
      break;
    case -2:
      retStr += "max angle less than 18 degrees";
      break;
    default:
      retStr += "unknown";
      break;
  }
  */
  return retStr;
}


void stmAs5600getAllReg(void){
	//Config
	int i;
	unsigned int val;

	val = stmAs5600readOneByte(0);
	printf("Reg00=0x%02x\r\n",val);

	val = stmAs5600readTwoBytes(1,2);
	printf("RegW0102=0x%04x\r\n",val);

	val = stmAs5600readTwoBytes(3,4);
	printf("RegW0304=0x%04x\r\n",val);

	val = stmAs5600readTwoBytes(5,6);
	printf("RegW0506=0x%04x\r\n",val);

	val = stmAs5600readTwoBytes(7,8);
	printf("RegW0708=0x%04x\r\n",val);

	val = stmAs5600readTwoBytes(0x0c,0x0d);
	printf("RegW0c0d=0x%04x\r\n",val);

	val = stmAs5600readTwoBytes(0x0e,0x0f);
	printf("RegW0E0F=0x%04x\r\n",val);

	val = stmAs5600readOneByte(0x0B);
	printf("Reg0x%02x=0x%02x\r\n",0x0B, val);

	val = stmAs5600readOneByte(0x1A);
	printf("Reg0x%02x=0x%02x\r\n",0x1A, val);

	val = stmAs5600readTwoBytes(0x1b,0x1c);
	printf("RegW1b1c=0x%04x\r\n",val);

	val = stmAs5600readOneByte(0xFF);
	printf("Reg0xFF=0x%02x\r\n", val);
}
//float stmAs5600_GetDegree(){
short stmAs5600_GetDegree(){
	float f_degree;
	short degree;
	unsigned short rawAngle;
	if(stmAs5600detectMagnet() == 0){
		printf("No Magnet...\r\n");
		delayms(100);
		return -1;
	}
	rawAngle = stmAs5600getRawAngle();
	//f_degree = stmAs5600convertRawAngleToDegrees(rawAngle);
	degree = stmAs5600convertRawAngleToDegrees(rawAngle);
	//printf("rawAngle=%d, degree=%d\r\n",rawAngle, degree);
	return degree; //return f_degree;
}
extern unsigned char g_bI2CModuleConfigDone;
void stmAs5600_Loop(){
	int i=0;
	char str[10];
	int degree;
	int rawAngle;

	if(!g_bI2CModuleConfigDone){
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
			gI2Cx = I2C2;
#else
		gI2Cx = I2C1;
#endif
		printf("I2C Init...");
		stm_I2C_Init(gI2Cx,400000);//400Kbps
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	}

	printf("stmAS5600 Loop\r\n");

	//stmAs5600printMenu();

	stmAs5600getAllReg();

	while(1){
		degree = (int)stmAs5600_GetDegree();
		printf("Degree = %d\r\n", degree);
		delayms(100);
		//i++; if(i==50){ stmAs5600getAllReg();printf("===\r\n" );	i=0;}
  };
}

//====
/*******************************************************
/* Function: loop
/* In: none
/* Out: none
/* Description: main program loop
/*******************************************************/
void stmAs5600_loop()
{
 /*
  if (Serial.available() > 0)
  {
    char incomingByteBuffer[2];
    char incomingByte;

    incomingByteBuffer[0] = NULL;
    incomingByteBuffer[1] = NULL;

    Serial.readBytes(incomingByteBuffer,2);

    if ((incomingByteBuffer[0] >= 48) && (incomingByteBuffer[0] < 60))
    {
      incomingByte = incomingByteBuffer[0] - 48;
    }

    if ((incomingByteBuffer[1] >= 48) && (incomingByteBuffer[1] < 60))
    {
      incomingByte *=10;
      incomingByte += incomingByteBuffer[1] - 48;
    }


    switch (incomingByte)
    {
      case 1:
      {
        if(ams5600.detectMagnet()==1)
          lastResponse = ("Start angle set to = "+String(stmAs5600convertRawAngleToDegrees(stmAs5600setStartPosition()), DEC));  //Print Raw Angle Value
        else
          lastResponse = noMagnetStr;
      }
      break;

      case 2:
      {
        if(ams5600.detectMagnet()==1)
          lastResponse = ("End angle set to = "+String(stmAs5600convertRawAngleToDegrees(stmAs5600setEndPosition()), DEC));
        else
          lastResponse = noMagnetStr;
      }
      break;

      case 3:
      {
        if(ams5600.detectMagnet()==1)
          lastResponse = ("Max angle range set to = "+String(stmAs5600convertRawAngleToDegrees(stmAs5600setMaxAngle()), DEC));
        else
          lastResponse = noMagnetStr;
      }
      break;

      case 4:
      {
        lastResponse = ("Max angle range= "+String(stmAs5600convertRawAngleToDegrees(stmAs5600getMaxAngle()), DEC));
      }
      break;

      case 5:
      {
        lastResponse = ("Start angle = "+String(stmAs5600convertRawAngleToDegrees(stmAs5600getStartPosition()), DEC));
      }
      break;

      case 6:
      {
        lastResponse = "End angle = " + String(stmAs5600convertRawAngleToDegrees(stmAs5600getEndPosition()),DEC);
      }
      break;

      case 7:
      {
        lastResponse = "Raw angle = "+ String(stmAs5600convertRawAngleToDegrees(stmAs5600getRawAngle()),DEC);
      }
      break;

      case 8:
      {
        lastResponse = "Scaled angle = "+String(stmAs5600convertScaledAngleToDegrees(stmAs5600getScaledAngle()),DEC);
      }
      break;

      case 9:
      {
        if(ams5600.detectMagnet()==1)
          lastResponse = "Magnet detected";
        else
          lastResponse = noMagnetStr;
      }
      break;

      case 10:
      {
        lastResponse = "Magnet strength ";
        if(ams5600.detectMagnet()==1)
        {
          int magStrength = stmAs5600getMagnetStrength();

          if(magStrength == 1)
            lastResponse += "is weak";
          else if(magStrength == 2)
            lastResponse += "is acceptable";
          else if (magStrength == 3)
            lastResponse += "is to strong";
        }
        else
          lastResponse = noMagnetStr;
      }
      break;

      case 96:
      {
        printf ("BURN ANGLE (Y/N)?");

        char answer;

        Serial.readBytes(&answer,1);

        if((answer == 'y') || (answer == 'Y'))
        {
          lastResponse = stmAs5600burnAngle();
        }
        else
          lastResponse = "Brun canceled";
      }
      break;

      case 98:
      {
        printf ("BURN MAX ANGLE AND CONFIG (Y/N)?");

        char answer = Serial.read();

        if((answer == 'y') || (answer == 'Y'))
        {
          lastResponse = burnMaxAngleAndConfig();
        }
        else
          lastResponse = "Brun canceled";
      }
      break;

      default:
      {
          lastResponse = "Invalid Entry";
      }
      break;
    }
    //end of menu processing
    stmAs5600printMenu();
  }
  */
}
