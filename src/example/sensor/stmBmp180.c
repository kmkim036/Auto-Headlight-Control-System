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

extern void somedelay(long ulLoop);
//*** [NOTE] We need reset to begin.

//TempData = 16bit
//Pressure Data = 16~19 bit
//addr [7bit] = 0x77
// Sequence
// ---- addr (Write)-- | - Take meas reg  -|| ---- Temp Reg -- |||
// I2C Address (0xEE) | RegisterAddr(0xF4) ||  TempReg(0x2E)  |||
// ---- addr (Read)-- | - read meas reg U- | -- Result (MSB)----| -- Result(LSB) --|
// I2C Address (0xEF) | RegisterAddr(0xF6)   |-- Value           (0x5C) | Value(0x96) -----|


#define BMP180_I2CADDR 0xEE 		//0x77 = 7 bit address

/* Register definitions */
#define BMP180_TAKE_MEAS_REG   0xf4 //Ctrl_MEAS
#define BMP180_READ_MEAS_REG_U   0xf6 //OUT_MSB
#define BMP180_READ_MEAS_REG_L   0xf7 //OUT_LSB
#define BMP180_READ_MEAS_REG_XL  0xf8 //OUT_XLSB
//etc : E0=SoftReset; D0=ID; Calib21 downto calib0 = BF to AA
#define BMP180_WRITE_SOFTRESET              0xE0
#define BMP180_READ_CHIPID                         0xD0

// Control Register values for differenti internal oversampling setting (oss)
#define BMP180_MEAS_TEMP     0x2e // write reg 0xF4, wait 4.5msec
#define BMP180_MEAS_PRESS_OVERSAMP_0   0x34  // write reg 0xF4, wait 4.5msec
#define BMP180_MEAS_PRESS_OVERSAMP_1     0x74 //7.5ms wait for measurement
/* 13.5ms wait for measurement */
#define BMP180_MEAS_PRESS_OVERSAMP_2   0xb4
/* 25.5ms wait for measurement */
#define BMP180_MEAS_PRESS_OVERSAMP_3   0xf4

//Read A/D Conversion Result or EEPROM Data
//Before the first calculation of temperature and pressure, the master reads out the PROM data.
//EEPROM registers each is a two byte value so there is an upper byte and a lower byte
#define BMP180_EEPROM_AC1_U  0xaa
#define BMP180_EEPROM_AC1_L  0xab
#define BMP180_EEPROM_AC2_U  0xac
#define BMP180_EEPROM_AC2_L  0xad
#define BMP180_EEPROM_AC3_U  0xae
#define BMP180_EEPROM_AC3_L  0xaf
#define BMP180_EEPROM_AC4_U  0xb0
#define BMP180_EEPROM_AC4_L  0xb1
#define BMP180_EEPROM_AC5_U  0xb2
#define BMP180_EEPROM_AC5_L  0xb3
#define BMP180_EEPROM_AC6_U  0xb4
#define BMP180_EEPROM_AC6_L  0xb5
#define BMP180_EEPROM_B1_U  0xb6
#define BMP180_EEPROM_B1_L  0xb7
#define BMP180_EEPROM_B2_U  0xb8
#define BMP180_EEPROM_B2_L  0xb9
#define BMP180_EEPROM_MB_U  0xba
#define BMP180_EEPROM_MB_L  0xbb
#define BMP180_EEPROM_MC_U  0xbc
#define BMP180_EEPROM_MC_L  0xbd
#define BMP180_EEPROM_MD_U  0xbe
#define BMP180_EEPROM_MD_L  0xbf

#define I2C_TRIES   5
#define AUTO_INCREMENT   0x80

#define DELAY_LOWBOUND   (50 * NSEC_PER_MSEC)
#define DELAY_UPBOUND   (500 * NSEC_PER_MSEC)
#define DELAY_DEFAULT   (200 * NSEC_PER_MSEC)

#define PRESSURE_MAX   125000
#define PRESSURE_MIN   95000
#define PRESSURE_FUZZ   5
#define PRESSURE_FLAT   5

#define FACTORY_TEST
#ifdef FACTORY_TEST
#define TEMP_MAX   3000
#define TEMP_MIN   -3000
#define SEA_LEVEL_MAX   999999
#define SEA_LEVEL_MIN   -1
#endif

struct bmp180_eeprom_data {
  s16 AC1, AC2, AC3;
  u16 AC4, AC5, AC6;
  s16 B1, B2;
  s16 MB, MC, MD;
};

struct bmp180_data {
//  struct work_struct work_pressure;
  u8 oversampling_rate;
  struct bmp180_eeprom_data bmp180_eeprom_vals;
};
//void bmp180Read(u8 reg, u8 *buf, int len);
void bmp180ReadEeprom(struct bmp180_data *barom);
u16 be16_to_cpu(u8 x[]) {
	u16 y;
	y = x[0]*256 + x[1];
  return y;
 }
u16 be32_to_cpu(u16 x){
	return x;
}

extern unsigned char I2CReceiveBurstWithRestartCondition(unsigned char slaveAddr, unsigned char reg, unsigned char *burst, unsigned char len);
//To Read, send I2C addr + Register Address, and then Restart and send I2C addr + Read 2 bytes data
void bmp180_i2c_read(u8 reg, u8 *buf,u8 len) //always 2 Byte Read
{
	stm_I2C_ReceiveBurstWithRestartCondition(BMP180_I2CADDR, reg,buf, len);
}

int bmp180_i2c_write( u8 cmd, u8 data)
{
  unsigned char i2cdata[2];

  i2cdata[0] = cmd;
  i2cdata[1] = data;
  stm_I2C_SendBurst(BMP180_I2CADDR,i2cdata,2); //send addr, command, data
  return 1;
 }

//Write 0x2E into Reg 0xF4, wait 4.5ms
int bmp180_get_raw_temperature(struct bmp180_data *barom,  u16 *raw_temperature)
 {
  u8 buf[2];

 //Issue Command
  bmp180_i2c_write( BMP180_TAKE_MEAS_REG,  BMP180_MEAS_TEMP); //F4(Control Measure)+OSS+SCO+ControlRegValue (0x2E=Temp)
  somedelay(45000);//wait 4.5msec

  //Read reg 0xf6(HIGH) and 0xf7(LOW)
 //To Read, send I2C addr + Register Address, and then Restart and send I2C addr + Read 2 bytes data
  bmp180_i2c_read(BMP180_READ_MEAS_REG_U, buf,2)  ;
  *raw_temperature = be16_to_cpu(buf);
  printf("bmp180>  uncompensated temperature:  %d  \r\n",*raw_temperature);
  return 1;
 }
//pressure == 16~19bits
int bmp180_get_raw_pressure(struct bmp180_data *barom,u32 *raw_pressure)
 {
  u8 buf[3];

  bmp180_i2c_write( BMP180_TAKE_MEAS_REG,  BMP180_MEAS_PRESS_OVERSAMP_0 |   (barom->oversampling_rate << 6));
  somedelay(50000); // msleep(2+(3 << barom->oversampling_rate));

  bmp180_i2c_read(BMP180_READ_MEAS_REG_U, buf,3);

  *raw_pressure = buf[0]*65536 + buf[1]*256 + buf[0];
  *raw_pressure >>= (8 - barom->oversampling_rate);
  printf("bmp180>  uncompensated pressure:  %d  \r\n", *raw_pressure);

  return 1;
 }

void bmp180_get_pressure_data(struct bmp180_data *barom)
 {
  u16 raw_temperature;
  u32 raw_pressure;
  long x1, x2, x3, b3, b5, b6;
  unsigned long b4, b7;
  long p;
  int pressure;

  bmp180_get_raw_temperature(barom, &raw_temperature);
  bmp180_get_raw_pressure(barom, &raw_pressure);

  x1 = ((raw_temperature - barom->bmp180_eeprom_vals.AC6) *
        barom->bmp180_eeprom_vals.AC5) >> 15;
  x2 = (barom->bmp180_eeprom_vals.MC << 11) /
      (x1 + barom->bmp180_eeprom_vals.MD);
  b5 = x1 + x2;

  printf("Compensated Temp = %d\n", ((b5 + 8)>>4)/10);

  b6 = (b5 - 4000);
  x1 = (barom->bmp180_eeprom_vals.B2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (barom->bmp180_eeprom_vals.AC2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)barom->bmp180_eeprom_vals.AC1) * 4 +
   x3) << barom->oversampling_rate) + 2) >> 2;
  x1 = (barom->bmp180_eeprom_vals.AC3 * b6) >> 13;
  x2 = (barom->bmp180_eeprom_vals.B1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (barom->bmp180_eeprom_vals.AC4 *
        (unsigned long)(x3 + 32768)) >> 15;
  b7 = ((unsigned long)raw_pressure - b3) *
   (50000 >> barom->oversampling_rate);
  if (b7 < 0x80000000)
   p = (b7 * 2) / b4;
  else
   p = (b7 / b4) * 2;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = p + ((x1 + x2 + 3791) >> 4);
  printf("bmp180> calibrated pressure: %dPa(%dhPa)  \r\n", pressure,pressure/100);

  return;
 }

void bmp180ReadEeprom(struct bmp180_data *barom)
{
	int i;
	unsigned char buf[22];

	bmp180_i2c_read(BMP180_EEPROM_AC1_U, buf,2);
	barom->bmp180_eeprom_vals.AC1 = be16_to_cpu(buf);
	bmp180_i2c_read(BMP180_EEPROM_AC2_U, buf+2,2);
	barom->bmp180_eeprom_vals.AC2 = be16_to_cpu(buf+2);
	bmp180_i2c_read(BMP180_EEPROM_AC3_U, buf+4,2);
    barom->bmp180_eeprom_vals.AC3 = be16_to_cpu(buf+4);
    bmp180_i2c_read(BMP180_EEPROM_AC4_U, buf+6,2);
    barom->bmp180_eeprom_vals.AC4 = be16_to_cpu(buf+6);
    bmp180_i2c_read(BMP180_EEPROM_AC5_U, buf+8,2);
    barom->bmp180_eeprom_vals.AC5 = be16_to_cpu(buf+8);
    bmp180_i2c_read(BMP180_EEPROM_AC6_U, buf+10,2);
    barom->bmp180_eeprom_vals.AC6 = be16_to_cpu(buf+10);
    bmp180_i2c_read(BMP180_EEPROM_B1_U, buf+12,2);
    barom->bmp180_eeprom_vals.B1 = be16_to_cpu(buf+12);
    bmp180_i2c_read(BMP180_EEPROM_B2_U, buf+14,2);
    barom->bmp180_eeprom_vals.B2 = be16_to_cpu(buf+14);
    bmp180_i2c_read(BMP180_EEPROM_MB_U, buf+16,2);
    barom->bmp180_eeprom_vals.MB = be16_to_cpu(buf+16);
    bmp180_i2c_read(BMP180_EEPROM_MC_U, buf+18,2);
    barom->bmp180_eeprom_vals.MC = be16_to_cpu(buf+18);
    bmp180_i2c_read(BMP180_EEPROM_MD_U, buf+20,2);
    barom->bmp180_eeprom_vals.MD = be16_to_cpu(buf+20);

	for(i=0;i<22;i++){
			printf("bmp180> EEPROM[%d]=0x%02x(%d)\r\n",i,buf[i]);
	}
 }
extern unsigned char g_bI2CModuleConfigDone;
int bmp180_init(struct bmp180_data *barom)
 {
	u16 buf[11];
	memset(buf, 0x00, 22);
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

  bmp180ReadEeprom(barom);

  barom->oversampling_rate = 0;

  return 1;//err;
 }

void bmp180Loop() {
  struct bmp180_data dbarom;
  struct bmp180_data *barom;

  //To work, you should reset promptly.
  printf("BMP180 Digital Barometer Test\r\n");
  barom = &dbarom;
  bmp180_init(barom);
  somedelay(10000);

  while(1) {
   bmp180_get_pressure_data(barom);
   //dtUserLedCon(1);
   somedelay(100000);
   //dtUserLedCon(0);
   somedelay(100000);
  }
}
