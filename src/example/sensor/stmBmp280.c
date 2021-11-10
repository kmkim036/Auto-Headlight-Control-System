/***************************************************************************
  This is a library for the BMP280 pressure sensor

  These sensors use I2C to communicate, 2 pins are required to interface.

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
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

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern char *float2str(float x);

extern I2C_TypeDef *gI2Cx;

/*=========================================================================
    I2C ADDRESS/BITS/SETTINGS
    -----------------------------------------------------------------------*/
#define BMP280_7BIT_I2C_ADDRESS       (0x76) //(0x77) //111 0110
#define BMP280_8BIT_I2C_ADDRESS       (0xec) //(0xee) //1110 1100 (0xec)
#define BMP280_CHIPID                 (0x58)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
    } bmp280_calib_data;

   // bool  begin(uint8_t addr = BMP280_ADDRESS, uint8_t chipid = BMP280_CHIPID);
    float stmBmp280_BARO_readTemperature(void);
    float stmBmp280_BARO_readPressure(void);
    float stmBmp280_BARO_readAltitude();
    void stmBmp280_readCoefficients(void);


    void      stmBmp280_write8(unsigned char reg, unsigned char value);
    uint8_t   stmBmp280_read8(unsigned char reg);
    uint16_t  stmBmp280_read16(unsigned char reg);
    uint32_t  stmBmp280_read24(unsigned char reg);
    int16_t   stmBmp280_readS16(unsigned char reg);
    uint16_t  stmBmp280_read16_LE(unsigned char reg); // little endian
    int16_t   stmBmp280_readS16_LE(unsigned char reg); // little endian


    int32_t t_fine;
    bmp280_calib_data _bmp280_calib;
    float seaLevelhPa = 1013.25;



/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C.
*/
/**************************************************************************/
void stmBmp280_write8(unsigned char reg, unsigned char value)
{
	unsigned char regbuf[10];
   	int ret=0;
   	regbuf[0] = reg;
   	regbuf[1] = value;
    ret = stm_I2C_SendBurst(BMP280_8BIT_I2C_ADDRESS, regbuf, 2);
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C/SPI
*/
/**************************************************************************/
uint8_t stmBmp280_read8(unsigned char reg)
{
 	uint8_t value;
 	unsigned char regbuf[10];
 	int ret=0;

	stm_I2C_ReceiveBurstWithRestartCondition(BMP280_8BIT_I2C_ADDRESS, reg, &regbuf[0], 1);
	value = regbuf[0];
	return value;

}

uint16_t stmBmp280_read16(unsigned char reg)
{
	uint16_t s_value;
	unsigned char regbuf[10];
	int ret=0;

	stm_I2C_ReceiveBurstWithRestartCondition(BMP280_8BIT_I2C_ADDRESS, reg, &regbuf[0], 2);

	s_value = regbuf[0]; s_value <<= 8;
	s_value |= regbuf[1];

	return s_value;

}

uint16_t stmBmp280_read16_LE(unsigned char reg) {

 	unsigned short s_value;
 	unsigned char regbuf[10];
 	int ret=0;

	stm_I2C_ReceiveBurstWithRestartCondition(BMP280_8BIT_I2C_ADDRESS, reg, &regbuf[0], 2);
	s_value = regbuf[1]; s_value <<= 8;
	s_value |= regbuf[0];
	return s_value;
}

// Reads a signed 16 bit value over I2C
int16_t stmBmp280_readS16(unsigned char reg)
{
  return (int16_t)stmBmp280_read16(reg);

}

int16_t stmBmp280_readS16_LE(unsigned char reg)
{
  return (int16_t)stmBmp280_read16_LE(reg);

}

 /**************************************************************************/
/*!
    @brief  Reads a 24 bit value over I2C
*/
/**************************************************************************/
uint32_t stmBmp280_read24(unsigned char reg)
{
	uint32_t value;
	unsigned char regbuf[10];
	int ret=0;

	stm_I2C_ReceiveBurstWithRestartCondition(BMP280_8BIT_I2C_ADDRESS, reg, &regbuf[0], 3);
	value = regbuf[0]; value <<= 8;
	value |= regbuf[1];value <<= 8;
	value |= regbuf[2];

	return value;
}

//Reads the factory-set coefficients
void stmBmp280_readCoefficients(void)
{
	printf("BMP280> Read Factory Coefficients.\r\n");
    _bmp280_calib.dig_T1 = stmBmp280_read16_LE(BMP280_REGISTER_DIG_T1);
    _bmp280_calib.dig_T2 = stmBmp280_readS16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib.dig_T3 = stmBmp280_readS16_LE(BMP280_REGISTER_DIG_T3);

    _bmp280_calib.dig_P1 = stmBmp280_read16_LE(BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = stmBmp280_readS16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = stmBmp280_readS16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = stmBmp280_readS16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = stmBmp280_readS16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = stmBmp280_readS16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = stmBmp280_readS16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = stmBmp280_readS16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = stmBmp280_readS16_LE(BMP280_REGISTER_DIG_P9);

   	printf("Calib.T1 = %u\r\n",_bmp280_calib.dig_T1);
   	printf("Calib.T2 = %d\r\n",_bmp280_calib.dig_T2);
   	printf("Calib.T3 = %d\r\n",_bmp280_calib.dig_T3);
   	printf("Calib.P1 = %u\r\n",_bmp280_calib.dig_P1);
   	printf("Calib.P2 = %d\r\n",_bmp280_calib.dig_P2);
   	printf("Calib.P3 = %d\r\n",_bmp280_calib.dig_P3);
}

//To get t_fine value.
float stmBmp280_BARO_readTemperature(void)
{
  int32_t var1, var2;
  float fT;
  int32_t adc_T;
  char *str;

  adc_T = stmBmp280_read24(BMP280_REGISTER_TEMPDATA);
  //printf("adc_t=0x%06x\r\n",adc_T);
  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
    ((int32_t)_bmp280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
      ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
    ((int32_t)_bmp280_calib.dig_T3)) >> 14;

  t_fine = var1 + var2;
  //printf("t_fine=%u\r\n",t_fine);
  fT  = (float)(t_fine * 5.0 + 128.0);

  fT  = fT/256.0; //fT >> 8;
  return fT/100.0;
}

/**************************************************************************/
float stmBmp280_BARO_readPressure(void)
{
  int64_t var1, var2, p;
  float hPa;

  // Must be done first to get the t_fine variable set up
  stmBmp280_BARO_readTemperature();

  int32_t adc_P = stmBmp280_read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
  hPa = ((float)p)/256.0/100.0; //in hPa

  return hPa;
}

float stmBmp280_BARO_readAltitude()
{
  float f_altitude;

  float pressure_hPa = stmBmp280_BARO_readPressure();///100.0; // in hPa, //was in Si units for Pascal
  //pressure /= 100.0;

  f_altitude = 44330.0 * (1.0 - pow(pressure_hPa / seaLevelhPa, 0.1903));

  return f_altitude;
}

extern unsigned char g_bI2CModuleConfigDone;

void stmBmp280_BARO_Init()
{

	unsigned char chipid;
	if(!g_bI2CModuleConfigDone)
	{
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

	printf("BMP280 test\r\n");

	chipid = stmBmp280_read8(BMP280_REGISTER_CHIPID);

	if (stmBmp280_read8(BMP280_REGISTER_CHIPID) != BMP280_CHIPID){
		printf("Could not find a valid BMP280 sensor, check wiring!(? 0x%02x)\r\n",chipid);
	    return false;
	}else{
		printf("Found a valid BMP280 sensor @ 0x%02x(7-bitAddr).\r\n",BMP280_7BIT_I2C_ADDRESS);
	}

	stmBmp280_readCoefficients();
	stmBmp280_write8(BMP280_REGISTER_CONTROL, 0x3F);
	return true;
}

void stmBmp280Loop()
{

	stmBmp280_BARO_Init();

	while(1){
    printf("Temperature \t= ");
    printf("%s",float2str(stmBmp280_BARO_readTemperature()));
    printf(" *C\r\n");

    printf("Pressure \t= ");
    printf("%s",float2str(stmBmp280_BARO_readPressure()));
    printf(" hPa\r\n");

    printf("Altitude \t= ");
    printf("%s",float2str(stmBmp280_BARO_readAltitude())); // this should be adjusted to your local forcase
    printf(" Meter\r\n");

    printf("=====\r\n");
    delayms(200);
	}
}

