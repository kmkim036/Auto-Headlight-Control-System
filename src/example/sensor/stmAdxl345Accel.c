/* Bare bones ADXL345 i2c example for Arduino 1.0                       *
* by Jens C Brynildsen <http://www.flashgamer.com>                     *
* Demonstrates use of ADXL345 (using the Sparkfun ADXL345 breakout)    *
* with i2c communication. Datasheet:                                   *
* http://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf *
***********************************************************************/

/* INT2
 * INT1
 * I2C Addr = 0x53 (7bits) --> 0xA6(WRITE)
 *
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
#include <math.h>

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);

extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c

extern char *float2str(float x);

extern I2C_TypeDef *gI2Cx;

extern void somedelay(long ulLoop);

#define ADXL345ADDR 0xA6 //0x53 - 7 bit addr



char POWER_CTL = 0x2D;        //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;        //X-Axis Data 0
char DATAX1 = 0x33;        //X-Axis Data 1
char DATAY0 = 0x34;        //Y-Axis Data 0
char DATAY1 = 0x35;        //Y-Axis Data 1
char DATAZ0 = 0x36;        //Z-Axis Data 0
char DATAZ1 = 0x37;        //Z-Axis Data 1

void ADXL345_Write(u8 reg, u8 val) {
  stmI2cSendbuf[0] = reg;             // send register address
  stmI2cSendbuf[1] = val;                 // send value to write
  stm_I2C_SendBurst(ADXL345ADDR, stmI2cSendbuf, 2);
}

// Reads num u8s starting from address register on device in to adxl_buf array
void ADXL345_Read(u8 reg, int num, u8 adxl_buf[]) {
	  //stmI2cSendbuf[0] = reg;             // send register address to read from
	  //stm_I2C_SendBurst(ADXL345ADDR, stmI2cSendbuf, 1);
	  // request 6 bytes from device
	  stm_I2C_ReceiveBurstWithRestartCondition(ADXL345ADDR, reg, stmI2cRecvbuf, num);     //Read a byte
	  memcpy(adxl_buf, stmI2cRecvbuf, num);
}

void stmADXL345_ACCEL_Init()
{
  printf("ADXL345 Init.\r\n");

  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  ADXL345_Write(DATA_FORMAT, 0x01); //right justified with sign extension(0).  10 bit mode(0),
  printf("check\n");
  somedelay(100);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  ADXL345_Write(POWER_CTL, 0x08);
  printf("check\n");
}

void stmADXL345_ACCEL_ReadXYZ(short *acc)
{
	u8 adxlBuf[6];
	u8 howManyu8sToRead = 6;
	ADXL345_Read(DATAX0, howManyu8sToRead, adxlBuf); //read the acceleration data from the ADXL345

	//Two's complement values.
	// each axis reading comes in 10 bit resolution, ie 2 u8s.  Least Significant u8 first!!
	// thus we are converting both u8s in to one int
	acc[0] = ((adxlBuf[1] & 0x03) << 8) | adxlBuf[0]; //  x = (((int)adxl_buf[1]) << 8) | adxl_buf[0];
	acc[1] = ((adxlBuf[3] & 0x03) << 8) | adxlBuf[2]; //(((int)adxl_buf[3]) << 8) | adxl_buf[2];
	acc[2] = ((adxlBuf[5] & 0x03) << 8) | adxlBuf[4]; //(((int)adxl_buf[5]) << 8) | adxl_buf[4];

}

void stmADXL345Loop()
{
	short acc[3];
    printf("ADXL345 Driver Test with I2C");
	stmADXL345_ACCEL_Init();
	while(1){
		stmADXL345_ACCEL_ReadXYZ(acc); // read the x/y/z tilt
	    printf("(x,y,z)= (%d,%d,%d)\r\n",acc[0],acc[1],acc[2]);
//	    printf("pitch1: %d\r\n", (int) (10 * (atan(acc[0] / sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180.0 / 3.14)));
//	    printf("pitch2: %d\r\n", (int) (10 * (atan(acc[1] / sqrt(acc[0] * acc[0] + acc[2] * acc[2])) * 180.0 / 3.14)));
	    //	    printf("pitch: %d\r\n", (int) ())
	    delayms(1000); // read every 0.1 seconds (10Hz)
	}
}

int ADXL345get()
{
	short acc[3];
	stmADXL345_ACCEL_ReadXYZ(acc);
	int pitch = (int) (10 * 180 * atan(acc[0]/sqrt(acc[1]*acc[1] + acc[2]*acc[2])) / (3.141592));
	return pitch;
}
