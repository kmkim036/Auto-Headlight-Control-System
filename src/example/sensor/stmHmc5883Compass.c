/*
   Compass Module 3-Axis HMC5883L
   Read the Compass Module 3-Axis HMC5883L and prints them over the serial connection to the computer.

   The circuit:
    * SDA (Data)
    * SCL (Clock)
    * +V of accelerometer to +3.3V
    * GND of accelerometer to ground

 WARNING: THE HMC5883L IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for HMC5883L:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf

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

//=========================================================================================
#define HMC5883LADDR 0x3C		// 0x1E = 7-bit address of HMC5883 compass : -->0x3c(READ)
//registers
#define  HMC5883L_CONFIG_A      0
#define  HMC5883L_CONFIG_B      1
#define  HMC5883L_MODE          2
#define  HMC5883L_DATA          3
#define HMC5883L_DEVID    		0x0A //[3]//0x0F
//setup measurement mode
#define HMC5883L_MEAS_CONTINUOUS        0
#define HMC5883L_MEAS_SINGLE_SHOT       1
#define HMC5883L_MEAS_IDLE              3

//Orientation
#define COMPASS_NORTH 0x00
#define COMPASS_SOUTH 0x01
#define COMPASS_WEST  0x02
#define COMPASS_EAST  0x03
#define COMPASS_UP    0x04
#define COMPASS_DOWN  0x05

#define COMPASS_HORIZONTAL_X_NORTH ((COMPASS_NORTH << 6>) | (COMPASS_WEST << 3) | COMPASS_UP) << 5

#define ErrorCode_1 "Entered scale was not valid, valid gauss values are: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1"
#define ErrorCode_1_Num 1

//#define M_PI 3.14156
#define TWO_PI (2.0 * M_PI)
#define _180_DIV_PI         57.295779
//setup scale
#define HMC5883L_SCALE088 1 //0.88
#define HMC5883L_SCALE13 2 //1.3
#define HMC5883L_SCALE19 3 //1.9
#define HMC5883L_SCALE25 4 //2.5
#define HMC5883L_SCALE40 5 //4.0
#define HMC5883L_SCALE47 6 //4.7
#define HMC5883L_SCALE56 7 //5.6
#define HMC5883L_SCALE81 8 //8.1
#define HMC5883L_SCALE HMC5883L_SCALE40 //HMC5883L_SCALE13

// At 75Hz continuous measurement rate, the delay between samples = 1/75 = 13mS
#define  HMC5883L_MEAS_DELAY_MS     15
#define  HMC5883L_MEAS_NORMAL       0

#define MAX_MAG_SAMPLES 10

#define HMC5883L_CALIBRATED 1 //enable this if this magn is calibrated

//calibration values
//#if HMC5883L_CALIBRATED == 1
#define HMC5883L_OFFSETX -99.7
#define HMC5883L_OFFSETY -154.0
#define HMC5883L_OFFSETZ -22.7
#define HMC5883L_GAINX1 0.952017
#define HMC5883L_GAINX2 0.00195895
#define HMC5883L_GAINX3 0.0139661
#define HMC5883L_GAINY1 0.00195895
#define HMC5883L_GAINY2 0.882824
#define HMC5883L_GAINY3 0.00760243
#define HMC5883L_GAINZ1 0.0139661
#define HMC5883L_GAINZ2 0.00760243
#define HMC5883L_GAINZ3 0.995365
//#endif



int gGaussScale[8] = {
    1370,   //  0.9
    1090,   //  1.3
    820,    //  1.9
    660,    //  2.5
    440,    //  4.0
    390,    //  4.7
    330,    //  5.6
    230     //  8.1
    };

void HMC5883L_ReadRawAxis();
void MAG_GetScaledFromRawAxisValue();//struct MAG_raw *raw, struct MAG_scaled *scaled);

int SetMeasurementMode(unsigned char mode);
int HMC5883L_SetScale(u8 val);
char* GetErrorText(int errorCode);

#define LOCAL_DECLINE ((float)-0.14661) //in Radian, Gimpo. W8.4 degree.


struct MAG_raw{
	short XAxis;
	short YAxis;
	short ZAxis;
};
struct MAG_scaled{
	float XAxis;
	float YAxis;
	float ZAxis;
};
typedef struct MAG_CALIB_DATA_ {
        short xMax;
        short xMin;
        short yMax;
        short yMin;
        short zMax;
        short zMin;
        short xOff;
        short yOff;
        short zOff;
} MAG_CALIB_DATA;

struct _MAG{
	float m_Scale;
	MAG_CALIB_DATA calib;
	struct MAG_scaled scaled;
	struct MAG_raw raw;
	float f_magnitude;
	//
	float f_localDecline;
	float f_headingRad;
	float f_headingDegrees;
	float f_bearingDegrees;
};

struct _MAG mag;

void MAG_CalibOnInit(void)
{
	int xc,yc,zc;
	unsigned char xZero = 0;
	unsigned char yZero = 0;
	unsigned char zZero = 0;

	mag.calib.xMin = 0; mag.calib.yMin = 0; 	mag.calib.zMin = 0;
	mag.calib.xMax = 0; mag.calib.yMax = 0; 	mag.calib.zMax = 0;

	mag.f_localDecline = LOCAL_DECLINE;

	printf("Please rotate for calibrate\r\n");
	printf("Until (xc,yc,zc)=(>3,>3,>3)\r\n");
	delayms(1000);
	//do 3 times at least.
	xc=0; yc=0; zc=0;
	while(xc<3 || yc<3 || zc <3)
	{
		HMC5883L_ReadScaledAxis();

		if((fabs(mag.scaled.XAxis) > 0x600)
			|| (fabs(mag.scaled.YAxis) > 0x600)
			|| (fabs(mag.scaled.ZAxis) > 0x600))
		{
			continue;
		}

		//get min and max for all axises
		mag.calib.xMin = Y_MIN(mag.scaled.XAxis , mag.calib.xMin);
		mag.calib.yMin = Y_MIN(mag.scaled.YAxis , mag.calib.yMin);
		mag.calib.zMin = Y_MIN(mag.scaled.ZAxis , mag.calib.zMin);

		mag.calib.xMax = Y_MAX(mag.scaled.XAxis , mag.calib.xMax);
		mag.calib.yMax = Y_MAX(mag.scaled.YAxis , mag.calib.yMax);
		mag.calib.zMax = Y_MAX(mag.scaled.ZAxis , mag.calib.zMax);

		if(xZero){
			if(fabs(mag.scaled.XAxis) > 50){
				xZero = 0;	xc++;
			}
		}else{
			if(fabs(mag.scaled.XAxis) < 40)
				xZero = 1;
		}

		if(yZero){
			if(fabs(mag.scaled.YAxis) > 50){
				yZero = 0;	yc++;
			}
		}else{
			if(fabs(mag.scaled.YAxis) < 40)
				yZero = 1;
		}

		if(zZero){
			if(fabs(mag.scaled.ZAxis) > 50){
				zZero = 0;	zc++;
			}
		}else{
			if(fabs(mag.scaled.ZAxis) < 40)
				zZero = 1;
		}

		printf("xc,yc,zc=(%u,%u,%u)\r\n",xc,yc,zc);
		printf("Do more rotatation until (xc,yc,zc)=(^3,^3,^3)\r\n");
		delayms(100);
	}

	//mag.calib.xRange = mag.calib.xMax -  mag.calib.xMin;
	//mag.calib.yRange = mag.calib.yMax -  mag.calib.yMin;
	//mag.calib.zRange = mag.calib.zMax -  mag.calib.zMin;

	mag.calib.xOff = (mag.calib.xMax +  mag.calib.xMin)/2;
	mag.calib.yOff = (mag.calib.yMax +  mag.calib.yMin)/2;
	mag.calib.zOff = (mag.calib.zMax +  mag.calib.zMin)/2;

	printf("calibrated Done\r\n");

	printf("max(x,y,z)=%d,%d,%d\r\n", mag.calib.xMax,mag.calib.yMax,mag.calib.zMax);
	printf("min(x,y,z)=%d,%d,%d\r\n", mag.calib.xMin,mag.calib.yMin,mag.calib.zMin);
	printf("off(x,y,z)=%d,%d,%d\r\n", mag.calib.xOff,mag.calib.yOff,mag.calib.zOff);

	delayms(2000);
}

void HMC5883L_ReadRawAxis(){
  unsigned char buffer[6];

  //Overflow -> -4096
  //Range = -2048(0xf800) ~ 2047(0x07ff)
  stm_I2C_ReceiveBurstWithRestartCondition(HMC5883LADDR, HMC5883L_DATA, buffer, 6);
  // NOTE : data is MSB first !!  order is x,z,y!!
  mag.raw.XAxis = buffer[0]*256 + buffer[1];
  mag.raw.ZAxis = buffer[2]*256 + buffer[3];
  mag.raw.YAxis = buffer[4]*256 + buffer[5];
  printf("Raw    [x,y,z]= %d,%d,%d\r\n", mag.raw.XAxis, mag.raw.YAxis, mag.raw.ZAxis);
}

void HMC5883L_ReadScaledAxis()
{
	HMC5883L_ReadRawAxis();

	mag.scaled.XAxis = (float)((short)mag.raw.XAxis) * mag.m_Scale;
	mag.scaled.ZAxis = (float)((short)mag.raw.ZAxis) * mag.m_Scale;
	mag.scaled.YAxis = (float)((short)mag.raw.YAxis) * mag.m_Scale;
}

void MAG_GetScaledFromRawAxisValue()
{
	mag.scaled.XAxis = (float)((short)mag.raw.XAxis) * mag.m_Scale;
	mag.scaled.ZAxis = (float)((short)mag.raw.ZAxis) * mag.m_Scale;
	mag.scaled.YAxis = (float)((short)mag.raw.YAxis) * mag.m_Scale;

	printf("SCALED [x,y,z]= %d,%d,%d\r\n", (int)(mag.scaled.XAxis),
			(int)(mag.scaled.YAxis),
			(int)(mag.scaled.ZAxis));
}

int HMC5883L_SetScale(u8 val)
{
	unsigned char regValue = 0x00;
	static unsigned char buffer[2];

	switch(val){
	case HMC5883L_SCALE088:  //0.88
		regValue = 0x00;
		mag.m_Scale = 0.73;
		break;
	case HMC5883L_SCALE13: //1.3
		regValue = 0x01;
		mag.m_Scale = 0.92;
		printf("HMC5883L_SetScale to 1.3\r\n");
		break;
	case HMC5883L_SCALE19: //1.9
		regValue = 0x02;
		mag.m_Scale = 1.22;
		break;
	case HMC5883L_SCALE25: // 4 //2.5
		regValue = 0x03;
		mag.m_Scale = 1.52;
		break;
	/*
		#define HMC5883L_SCALE40 5 //4.0
		#define HMC5883L_SCALE47 6 //4.7
		#define HMC5883L_SCALE56 7 //5.6
		#define HMC5883L_SCALE81 8 //8.1
    */
	case HMC5883L_SCALE40:
		regValue = 0x04;
		mag.m_Scale = 2.27;
		break;
/*
	else if(gauss == 4.7)
	{
		regValue = 0x05;
		m_Scale = 2.56;
	}
	else if(gauss == 5.6)
	{
		regValue = 0x06;
		m_Scale = 3.03;
	}
	else if(gauss == 8.1)
	{
		regValue = 0x07;
		m_Scale = 4.35;
	}
	*/
	default:
		return ErrorCode_1_Num;
	}
	// Setting is in the top 3 bits of the register.
	regValue = regValue << 5;

	//Write(ConfigurationRegisterB, regValue);
	stmI2cSendbuf[0] = HMC5883L_CONFIG_B;
	stmI2cSendbuf[1] = regValue;
	printf("[Write] ConfigurationRegisterB(=%02x) <- %02x\r\n",stmI2cSendbuf[0],stmI2cSendbuf[1]);
    stm_I2C_SendBurst(HMC5883LADDR, stmI2cSendbuf, 2);
	delayms(100);

	//check
	stm_I2C_ReceiveBurstWithRestartCondition(HMC5883LADDR, HMC5883L_CONFIG_B, buffer, 1);
	printf("[READ] ConfigurationRegisterB = %02x\r\n",buffer[0]);
	//printf("return 0");
	return 0;
}

int HMC5883L_SetOperatingMode(unsigned char mode)
{
	//Write(ModeRegister, mode);
    stmI2cSendbuf[0] = HMC5883L_MODE;
	stmI2cSendbuf[1] = mode;
    stm_I2C_SendBurst(HMC5883LADDR, stmI2cSendbuf, 2);
	delayms(100);
    return 0;
}
void hmc5883l_SetMeasurementMode(int mode) {
    static unsigned char b;

    stm_I2C_ReceiveBurstWithRestartCondition(HMC5883LADDR, HMC5883L_CONFIG_A, &b, 1);

    b &= 0x7c; //"01111100" clear CRA7, keep existing averaging and datarate settings, and clear measuremode bits
    b |= ((u8)mode); //b = (u08)mode;

    stmI2cSendbuf[0] = HMC5883L_CONFIG_A;
	stmI2cSendbuf[1] = mode;
    stm_I2C_SendBurst(HMC5883LADDR, stmI2cSendbuf, 2);
}

void hmc5883l_view_reg(){
	unsigned char reg = 0x00;
	static unsigned char buffer[2];
	//check
	for(reg=0;reg<=12;reg++){
		stm_I2C_ReceiveBurstWithRestartCondition(HMC5883LADDR, reg, buffer, 1);
		printf("Reg[%d] = 0x%02x\r\n",reg,buffer[0]);
		delayms(10);
	}
	printf("Done.\r\n");
}
char* GetErrorText(int errorCode)
{
	if(ErrorCode_1_Num == 1)
		return ErrorCode_1;

	return "Error not defined.";
}

char hmc5883l_getProductID()
{
	unsigned char id[3];
	stm_I2C_ReceiveBurstWithRestartCondition(HMC5883LADDR, HMC5883L_DEVID, id, 3);

	if((id[0] == 0x48) && (id[1] == 0x34) && (id[2]==0x33)){
		printf("Found HMC5883L (ID=0x%02x:%02x:%02x(=%c%c%c))\r\n",id[0],id[1],id[2],id[0],id[1],id[2]);
		return 1;
	}
	else{
		printf("Err: No HMC5883L .. GetID=0x%02x:%02x:%02x(!= 0x48:34:33)\r\n",id[0],id[1],id[2]);

		hmc5883l_view_reg();

		return 0;
	}
}

void stmHMC5883L_MAG_Init()
{

	int error;

	if(!hmc5883l_getProductID()){
		printf("ERR\r\n");
		delayms(10000);
		return;
	}

	// 0-NUMSAMPLES[1:0]-DataOutRate[2:0]-MODE[1:0]
	// 0- 1,2,4,8       -100=15Hz;       -0=NORMAL;
	stmI2cSendbuf[0] = HMC5883L_CONFIG_A; //reg 0
	stmI2cSendbuf[1] = 0x70; //"0-11-100-00" 8 samples average,15Hz data rate, normal measurement flow
	//stmI2cSendbuf[1] = 0x78; //"0-11-110-00" 8 samples average,75Hz data rate, normal measurement flow
    //stmI2cSendbuf[1] = 0x10; //
    stm_I2C_SendBurst(HMC5883LADDR, stmI2cSendbuf, 2);

    //CONFIG_B (1)
	printf("Setting scale to +/- 1.3 Ga\r\n");
	//011-0:0000 = 2.5G Higher (lower Gain, coarse resolution) cause overflow
	//001-0:0000 = 1.3G (default)
	error = HMC5883L_SetScale(HMC5883L_SCALE13);//HMC5883L_SCALE25);//HMC5883L_SCALE19);//HMC5883L_SCALE40);// // Set the scale of the compass.(1.3)
	if(error != 0) // If there is an error, print it out.
	    printf("%s\r\n",GetErrorText(error));

	//reg2
	//0000:00-00 : Continuouse Mode
	//0000:00-01 : Single Short Mode
	printf("Setting Operating mode to continuous.\r\n");
	error = HMC5883L_SetOperatingMode(HMC5883L_MEAS_CONTINUOUS); // Set the measurement mode to Continuous
    //printf("Setting Operating mode to single shot.\r\n");
	//error = HMC5883L_SetOperatingMode(HMC5883L_MEAS_SINGLE_SHOT);
	if(error != 0) // If there is an error, print it out.
	  printf("Error in SetOperatingMode\r\n");

	//Do calibrate....
	MAG_CalibOnInit();
}
char MAG_ApplyCalibOffset()
{
	//apply offsets from the calibration.
	mag.scaled.XAxis -= mag.calib.xOff;
	mag.scaled.YAxis -= mag.calib.yOff;
	mag.scaled.ZAxis -= mag.calib.zOff;

	//get magnitude
	mag.f_magnitude = sqrtf(mag.scaled.XAxis*mag.scaled.XAxis +mag.scaled.YAxis*mag.scaled.YAxis +mag.scaled.ZAxis+mag.scaled.ZAxis);
	printf("Mag = %s\r\n",float2str(mag.f_magnitude));
	printf("Fixed [x,y,z]= %d,%d,%d\r\n", (int)(mag.scaled.XAxis),(int)(mag.scaled.YAxis), (int)(mag.scaled.ZAxis));
}

char MAG_GetHeading()
{
	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	//For using atan2, you should include <math.h> and library with m.
	mag.f_headingRad =  atan2f(mag.scaled.YAxis,mag.scaled.XAxis); //atan(y/x)
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
//Get bearing from heading ????
//bearing  0 = Y pointing to North
//bearing 90 = Y pointing to East
char MAG_GetBearing()
{
	mag.f_bearingDegrees = 450 - mag.f_headingDegrees;
	if(mag.f_bearingDegrees >=360)
		mag.f_bearingDegrees -= 360;
	return 1;
}
*/
char MAG_GetBearing()
{
	mag.f_bearingDegrees = mag.f_headingDegrees + 270;
	if(mag.f_bearingDegrees >=360)
		mag.f_bearingDegrees -= 360;
	return 1;
}

void stmHMC5883LLoop()
{
  printf("HMC5883L Driver Test with I2C\r\n");

  //init
  stmHMC5883L_MAG_Init();

  while(1){

	  //Get x,y,z raw data from MAG sensor
	  HMC5883L_ReadRawAxis();

	  //apply scale factor
	  MAG_GetScaledFromRawAxisValue();
	  //apply offset
	  MAG_ApplyCalibOffset();

	  //get heading
	  MAG_GetHeading();
	  //get bearing
	  MAG_GetBearing();

	  //Show
	  //printf("Magnetic Heading(Radian):%s\r\n",float2str(mag.f_headingRad)); //float
	  printf("Heading  (Degree)=%s\r\n",float2str(mag.f_headingDegrees));//float
	  printf("Ypointing(Degree)=%s\r\n",float2str(mag.f_bearingDegrees));
	  printf("===============\r\n");
	  // Normally we would delay the application by 66ms to allow the loop
	  // to run at 15Hz (default bandwidth for the HMC5883L).
	  // However since we have a long serial out (104ms at 9600) we will let
	  // it run at its natural speed.
	  // delay(66);
	  delayms(500);
  }
}

