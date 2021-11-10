/*
   Compass Module 3-Axis QMC5883
   Read the Compass Module 3-Axis QMC5883 and prints them over the serial connection to the computer.

   The circuit:
    * SDA (Data)
    * SCL (Clock)
    * +V of accelerometer to +3.3V
    * GND of accelerometer to ground

 WARNING: THE QMC5883 IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for QMC5883:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/QMC5883_3-Axis_Digital_Compass_IC.pdf

 [Important]
 To add math library for using atan2(), you must open Configuration of IDE, and  add just a single character of 'm' at "Linked Libraries".
 Place the magnetor in openspace. Keep out a large iron plate.
*/

#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include <stdio.h>
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
#else
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
#include "lwipopts.h"
#endif
#include <math.h> //for M_PI, atan....

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern char *float2str(float x);

extern I2C_TypeDef *gI2Cx;
extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c

extern void somedelay(long ulLoop);

//#define M_PI 3.14156
#define TWO_PI (2.0 * M_PI)
#define _180_DIV_PI         57.295779
#define LOCAL_DECLINE ((float)-0.14661) //in Radian, Gimpo. W8.4 degree.
//=========================================================================================
#define QMC5883ADDR 0x1A		// 0x0D = 7-bit address of QMC5883 compass :
//registers
#define  QMC5883_XYZ         0
#define  QMC5883_STATUS      6
#define  QMC5883_TEMP        7
#define  QMC5883_CONFIG_A    9
#define  QMC5883_CONFIG_B      0xA
#define  QMC5883_SET_RESET_PERIOD 0xB
#define QMC5883_DEVID    		0x0D

#define QMC5883_MODE_CONTINUOUS    1
#define QMC5883_MODE_STANDBY       0

#define QMC5883_ODR_10Hz    0
#define QMC5883_ODR_50Hz    4
#define QMC5883_ODR_100Hz   8
#define QMC5883_ODR_200Hz   0xC

#define QMC5883_RNG_2G    0
#define QMC5883_RNG_8G    0x10

#define QMC5883_OSR_512    0
#define QMC5883_OSR_256    0x40
#define QMC5883_OSR_128    0x80
#define QMC5883_OSR_64     0xc0

//CONFIG_B
#define QMC5883_ROL_PNT    0x40
#define QMC5883_INT_DIS     0x01

void QMC5883_ReadRawAxis();
void MAG_GetScaledFromRawAxisValue();//struct MAG_raw *raw, struct MAG_scaled *scaled);

int SetMeasurementMode(unsigned char mode);
int QMC5883_SetScale(u8 val);


struct MAG_raw{
	short XAxis;
	short YAxis;
	short ZAxis;
};

struct _MAG{
	struct MAG_raw raw;
	float f_magnitude;
	//
	float f_localDecline;
	float f_headingRad;
	float f_headingDegrees;
	float f_bearingDegrees;
};

struct _MAG mag;

void QMC5883_SoftReset(void)
{
    stmI2cSendbuf[0] = QMC5883_CONFIG_B; //@0A
	stmI2cSendbuf[1] = 0x80;
    stm_I2C_SendBurst(QMC5883ADDR, stmI2cSendbuf, 2);

	delayms(2000);
}

void QMC5883_ReadRawAxis(){
  unsigned char buf[8];

  stm_I2C_ReceiveBurstWithRestartCondition(QMC5883ADDR, QMC5883_STATUS, buf, 1);
  printf("Status@6 : DOR=%u; OVL=%u; DRDY=%u\r\n", (buf[0] & 0x04 ? 1:0),(buf[0] & 0x02 ? 1:0), (buf[0] & 0x01 ? 1:0));
  if(buf[0] & 0x01){
    stm_I2C_ReceiveBurstWithRestartCondition(QMC5883ADDR, QMC5883_XYZ, buf, 6);
    // NOTE : data is LSB first !!  order is x,y,z!!
    mag.raw.XAxis = buf[0] + buf[1]*256;
    mag.raw.YAxis = buf[2] + buf[3]*256;
    mag.raw.ZAxis = buf[4] + buf[5]*256;

    printf("Raw    [x,y,z]= %d,%d,%d\r\n", mag.raw.XAxis, mag.raw.YAxis, mag.raw.ZAxis);


    //get magnitude
    mag.f_magnitude = sqrtf(mag.raw.XAxis*mag.raw.XAxis +mag.raw.YAxis*mag.raw.YAxis +mag.raw.ZAxis+mag.raw.ZAxis);
    printf("Mag = %s\r\n",float2str(mag.f_magnitude));
  }else{
	  printf("Not Ready\r\n");
  }

}

void QMC5883_SetMode(int mode, int odr, int rng, int osr ) {

	stmI2cSendbuf[0] = QMC5883_SET_RESET_PERIOD; //@0B
	stmI2cSendbuf[1] = 0x01;
	stm_I2C_SendBurst(QMC5883ADDR, stmI2cSendbuf, 2);

    stmI2cSendbuf[0] = QMC5883_CONFIG_A; //@09
	stmI2cSendbuf[1] = mode | odr | rng | osr;
    stm_I2C_SendBurst(QMC5883ADDR, stmI2cSendbuf, 2);

    stmI2cSendbuf[0] = QMC5883_CONFIG_B; //@0A
   	stmI2cSendbuf[1] = QMC5883_ROL_PNT  | QMC5883_INT_DIS;
    stm_I2C_SendBurst(QMC5883ADDR, stmI2cSendbuf, 2);
}

void QMC5883_view_reg(){
	unsigned char reg = 0x00;
	static unsigned char buffer[2];
	//check
	for(reg=0;reg<=0xD;reg++){
		stm_I2C_ReceiveBurstWithRestartCondition(QMC5883ADDR, reg, buffer, 1);
		printf("Reg[%d] = 0x%02x\r\n",reg,buffer[0]);
		delayms(10);
	}
	printf("Done.\r\n");
}

char QMC5883_getProductID()
{
	unsigned char id[3];
	stm_I2C_ReceiveBurstWithRestartCondition(QMC5883ADDR, QMC5883_DEVID, id, 1);

	if((id[0] == 0xFF)){
		printf("Found QMC5883 (ID=0xFF) FOUND\r\n");
		return 1;
	}
	else{
		printf("Err: No QMC5883 .. GetID=0x%02x\r\n",id[0]);

		QMC5883_view_reg();

		return 0;
	}
}

void stmQMC5883_MAG_Init()
{

	int error;

	if(!QMC5883_getProductID()){
		printf("ERR\r\n");
		delayms(10000);
		return;
	}

	 QMC5883_SetMode(QMC5883_MODE_CONTINUOUS, QMC5883_ODR_200Hz, QMC5883_RNG_2G, QMC5883_OSR_512 );
	 QMC5883_view_reg();
}

char stmQMC5883_MAG_GetHeading()
{
	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	//For using atan2, you should include <math.h> and library with m.
	mag.f_headingRad =  atan2f(mag.raw.YAxis,mag.raw.XAxis); //atan(y/x)
	//printf("Heading=%s(radian)\r\n",float2str(*f_headingRad));

	// Once you have your heading, we add 'Declination Angle'
	mag.f_headingRad += mag.f_localDecline;

	// Correct for when signs are reversed.
	if(mag.f_headingRad < 0.0)  mag.f_headingRad += TWO_PI;

	// Check for wrap due to addition of declination.
	if(mag.f_headingRad > TWO_PI)  mag.f_headingRad -= TWO_PI;

	// Convert radians to degrees for readability.
	mag.f_headingDegrees = mag.f_headingRad * 180.0/M_PI;

	return 1;
}
/*
char MAG_GetBearing()
{
	mag.f_bearingDegrees = mag.f_headingDegrees + 270;
	if(mag.f_bearingDegrees >=360)
		mag.f_bearingDegrees -= 360;
	return 1;
}
*/
void stmQMC5883Loop()
{
  printf("QMC5883 Driver Test with I2C\r\n");

  //init
  mag.f_localDecline = LOCAL_DECLINE;

  stmQMC5883_MAG_Init();

  while(1){

	  //Get x,y,z raw data from MAG sensor
	  QMC5883_ReadRawAxis();

	  //get heading
	  stmQMC5883_MAG_GetHeading();
	  //get bearing
	  //MAG_GetBearing();

	  //Show
	  //printf("Magnetic Heading(Radian):%s\r\n",float2str(mag.f_headingRad)); //float
	  printf("Heading  (Degree)=%s\r\n",float2str(mag.f_headingDegrees));//float
	  //printf("Ypointing(Degree)=%s\r\n",float2str(mag.f_bearingDegrees));
	  printf("===============\r\n");
	  // Normally we would delay the application by 66ms to allow the loop
	  // to run at 15Hz (default bandwidth for the QMC5883).
	  // However since we have a long serial out (104ms at 9600) we will let
	  // it run at its natural speed.
	  // delay(66);
	  delayms(500);
  }
}

