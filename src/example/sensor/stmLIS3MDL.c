/* LIS3MDL : ultra-low-power high-performance 3-axis "nano" magnetometer.
*  LIS3MDL has 2 interrupts. Most interrupts are available only on the Int1 pin.
*  Interrupt pins are ACTIVE-HIGH and PP by default.
*/

#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#include "yInc.h"
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
//#include "yInc.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
#include "lwipopts.h"
#endif
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
#define I2C_400KHZ				  1	// 0 to use default 100Khz, 1 for 400Khz

#define LIS3MDL_8BIT_I2C_ADDRESS  0x38 //(0x1c<<1)    // if SDO/SA0 is 1, it becomes 0x1e

extern void delayms(uint32_t ms);
extern char *float2str(float x);

#define LIS3MDL_DEVICE_ID         0b00111101 //0x3D
#define LIS3MDL_I2C_AUTOINCREMENT 0x80

/* Defines -------------------------------------------------------------------*/
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_4G   0.14  /**< Sensitivity value for 4 gauss full scale [LSB/gauss] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_8G   0.29  /**< Sensitivity value for 8 gauss full scale [LSB/gauss] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_12G  0.43  /**< Sensitivity value for 12 gauss full scale [LSB/gauss] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_16G  0.58  /**< Sensitivity value for 16 gauss full scale [LSB/gauss] */

/* Typedefs ------------------------------------------------------------------*/
#define LIS3MDL_ERROR  0
#define LIS3MDL_OK     1
//} char;

typedef union{
	short int i16bit[3];
	unsigned char u8bit[6];
} Type3Axis16bit_U;

typedef union{
	short int i16bit;
	unsigned char u8bit[2];
} Type1Axis16bit_U;

typedef union{
	int i32bit;
	unsigned char u8bit[4];
} Type1Axis32bit_U;

typedef enum {
  MEMS_SUCCESS				=		0x01,
  MEMS_ERROR				=		0x00
} mems_status_t;

//====== REG ==========
/************** Who am I  *******************/
#define LIS3MDL_MAG_WHO_AM_I_REG  	0X0F
#define LIS3MDL_MAG_WHO_AM_I_BIT_MASK  	0xFF
#define LIS3MDL_MAG_WHO_AM_I_BIT_POSITION  	0
#define LIS3MDL_MAG_WHO_AM_I         0x3D
//========== 0x20 ==============
#define LIS3MDL_MAG_CTRL_REG1  	0X20
typedef enum {
  	LIS3MDL_MAG_DO_0_625Hz 		 =0x00,
  	LIS3MDL_MAG_DO_1_25Hz 		 =0x04,
  	LIS3MDL_MAG_DO_2_5Hz 		 =0x08,
  	LIS3MDL_MAG_DO_5Hz 		 =0x0C,
  	LIS3MDL_MAG_DO_10Hz 		 =0x10,
  	LIS3MDL_MAG_DO_20Hz 		 =0x14,
  	LIS3MDL_MAG_DO_40Hz 		 =0x18,
  	LIS3MDL_MAG_DO_80Hz 		 =0x1C,
} LIS3MDL_MAG_DO_t;
#define  	LIS3MDL_MAG_DO_MASK  	0x1C

typedef enum {
  	LIS3MDL_MAG_ST_DISABLE 		 =0x00,
  	LIS3MDL_MAG_ST_ENABLE 		 =0x01,
} LIS3MDL_MAG_ST_t;
#define  	LIS3MDL_MAG_ST_MASK  	0x01

typedef enum {
  	LIS3MDL_MAG_OM_LOW_POWER 		 =0x00,
  	LIS3MDL_MAG_OM_MEDIUM 		 =0x20,
  	LIS3MDL_MAG_OM_HIGH 		 =0x40,
  	LIS3MDL_MAG_OM_ULTRA_HIGH 		 =0x60,
} LIS3MDL_MAG_OM_t;
#define  	LIS3MDL_MAG_OM_MASK  	0x60

typedef enum {
  	LIS3MDL_MAG_TEMP_EN_DISABLE 		 =0x00,
  	LIS3MDL_MAG_TEMP_EN_ENABLE 		 =0x80,
} LIS3MDL_MAG_TEMP_EN_t;
#define  	LIS3MDL_MAG_TEMP_EN_MASK  	0x80

//========== 0x21 ========================
#define LIS3MDL_MAG_CTRL_REG2  	0X21
typedef enum {
  	LIS3MDL_MAG_FS_4Ga 		 =0x00,
  	LIS3MDL_MAG_FS_8Ga 		 =0x20,
  	LIS3MDL_MAG_FS_12Ga 		 =0x40,
  	LIS3MDL_MAG_FS_16Ga 		 =0x60,
} LIS3MDL_MAG_FS_t;
#define  	LIS3MDL_MAG_FS_MASK  	0x60
//Bit Group Name: SOFT_RST
typedef enum {
  	LIS3MDL_MAG_SOFT_RST_NO 		 =0x00,
  	LIS3MDL_MAG_SOFT_RST_YES 		 =0x04,
} LIS3MDL_MAG_SOFT_RST_t;
#define  	LIS3MDL_MAG_SOFT_RST_MASK  	0x04
//* Bit Group Name: REBOOT
typedef enum {
  	LIS3MDL_MAG_REBOOT_NO 		 =0x00,
  	LIS3MDL_MAG_REBOOT_YES 		 =0x08,
} LIS3MDL_MAG_REBOOT_t;
#define  	LIS3MDL_MAG_REBOOT_MASK  	0x08

//========== 0x22 ==========================
#define LIS3MDL_MAG_CTRL_REG3  	0X22
typedef enum {
  	LIS3MDL_MAG_MD_CONTINUOUS 		 =0x00,
  	LIS3MDL_MAG_MD_SINGLE 		 =0x01,
  	LIS3MDL_MAG_MD_POWER_DOWN 		 =0x02,
  	LIS3MDL_MAG_MD_POWER_DOWN_AUTO 		 =0x03,
} LIS3MDL_MAG_MD_t;
#define  	LIS3MDL_MAG_MD_MASK  	0x03
// Bit Group Name: SIM (Serial Interface Mode for SPI)
typedef enum {
  	LIS3MDL_MAG_SIM_4_WIRE 		 =0x00,
  	LIS3MDL_MAG_SIM_3_WIRE 		 =0x04,
} LIS3MDL_MAG_SIM_t;
#define  	LIS3MDL_MAG_SIM_MASK  	0x04

//========= 0x23 ====================
#define LIS3MDL_MAG_CTRL_REG4  	0X23
#define LIS3MDL_MAG_CTRL_REG5  	0X24 // Bit Group Name: BDU
typedef enum {
  	LIS3MDL_MAG_BDU_DISABLE 		 =0x00,
  	LIS3MDL_MAG_BDU_ENABLE 		 =0x40,
} LIS3MDL_MAG_BDU_t;
#define  	LIS3MDL_MAG_BDU_MASK  	0x40

#define LIS3MDL_MAG_STATUS_REG  	0X27
#define LIS3MDL_MAG_OUTX_L  	0X28
#define LIS3MDL_MAG_OUTX_H  	0X29
#define LIS3MDL_MAG_OUTY_L  	0X2A
#define LIS3MDL_MAG_OUTY_H  	0X2B
#define LIS3MDL_MAG_OUTZ_L  	0X2C
#define LIS3MDL_MAG_OUTZ_H  	0X2D
#define LIS3MDL_MAG_TEMP_OUT_L  0X2E
#define LIS3MDL_MAG_TEMP_OUT_H  0X2F
#define LIS3MDL_MAG_INT_CFG  	0X30
#define LIS3MDL_MAG_INT_SRC  	0X31
#define LIS3MDL_MAG_INT_THS_L  	0X32
#define LIS3MDL_MAG_INT_THS_H  	0X33

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: LP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_LP_DISABLE 		 =0x00,
  	LIS3MDL_MAG_LP_ENABLE 		 =0x20,
} LIS3MDL_MAG_LP_t;

#define  	LIS3MDL_MAG_LP_MASK  	0x20
mems_status_t  LIS3MDL_MAG_W_FastLowPowerXYZ(LIS3MDL_MAG_LP_t newValue);
mems_status_t LIS3MDL_MAG_R_FastLowPowerXYZ(LIS3MDL_MAG_LP_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_BLE_INVERT 		 =0x00,
  	LIS3MDL_MAG_BLE_DEFAULT 		 =0x02,
} LIS3MDL_MAG_BLE_t;

#define  	LIS3MDL_MAG_BLE_MASK  	0x02
mems_status_t  LIS3MDL_MAG_W_LittleBigEndianInversion(LIS3MDL_MAG_BLE_t newValue);
mems_status_t LIS3MDL_MAG_R_LittleBigEndianInversion(LIS3MDL_MAG_BLE_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: OMZ
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_OMZ_LOW_POWER 		 =0x00,
  	LIS3MDL_MAG_OMZ_MEDIUM 		 =0x04,
  	LIS3MDL_MAG_OMZ_HIGH 		 =0x08,
  	LIS3MDL_MAG_OMZ_ULTRA_HIGH 		 =0x0C,
} LIS3MDL_MAG_OMZ_t;

#define  	LIS3MDL_MAG_OMZ_MASK  	0x0C
mems_status_t  LIS3MDL_MAG_W_OperatingModeZ(LIS3MDL_MAG_OMZ_t newValue);
mems_status_t LIS3MDL_MAG_R_OperatingModeZ(LIS3MDL_MAG_OMZ_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: XDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_XDA_NOT_AVAILABLE 		 =0x00,
  	LIS3MDL_MAG_XDA_AVAILABLE 		 =0x01,
} LIS3MDL_MAG_XDA_t;

#define  	LIS3MDL_MAG_XDA_MASK  	0x01
mems_status_t LIS3MDL_MAG_R_NewXData(LIS3MDL_MAG_XDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: YDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_YDA_NOT_AVAILABLE 		 =0x00,
  	LIS3MDL_MAG_YDA_AVAILABLE 		 =0x02,
} LIS3MDL_MAG_YDA_t;

#define  	LIS3MDL_MAG_YDA_MASK  	0x02
mems_status_t LIS3MDL_MAG_R_NewYData(LIS3MDL_MAG_YDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_ZDA_NOT_AVAILABLE 		 =0x00,
  	LIS3MDL_MAG_ZDA_AVAILABLE 		 =0x04,
} LIS3MDL_MAG_ZDA_t;

#define  	LIS3MDL_MAG_ZDA_MASK  	0x04
mems_status_t LIS3MDL_MAG_R_NewZData(LIS3MDL_MAG_ZDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZYXDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_ZYXDA_NOT_AVAILABLE 		 =0x00,
  	LIS3MDL_MAG_ZYXDA_AVAILABLE 		 =0x08,
} LIS3MDL_MAG_ZYXDA_t;

#define  	LIS3MDL_MAG_ZYXDA_MASK  	0x08
mems_status_t LIS3MDL_MAG_R_NewXYZData(LIS3MDL_MAG_ZYXDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: XOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_XOR_NOT_OVERRUN 		 =0x00,
  	LIS3MDL_MAG_XOR_OVERRUN 		 =0x10,
} LIS3MDL_MAG_XOR_t;

#define  	LIS3MDL_MAG_XOR_MASK  	0x10
mems_status_t LIS3MDL_MAG_R_DataXOverrun(LIS3MDL_MAG_XOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: YOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_YOR_NOT_OVERRUN 		 =0x00,
  	LIS3MDL_MAG_YOR_OVERRUN 		 =0x20,
} LIS3MDL_MAG_YOR_t;

#define  	LIS3MDL_MAG_YOR_MASK  	0x20
mems_status_t LIS3MDL_MAG_R_DataYOverrun(LIS3MDL_MAG_YOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_ZOR_NOT_OVERRUN 		 =0x00,
  	LIS3MDL_MAG_ZOR_OVERRUN 		 =0x40,
} LIS3MDL_MAG_ZOR_t;

#define  	LIS3MDL_MAG_ZOR_MASK  	0x40
mems_status_t LIS3MDL_MAG_R_DataZOverrun(LIS3MDL_MAG_ZOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZYXOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_ZYXOR_NOT_OVERRUN 		 =0x00,
  	LIS3MDL_MAG_ZYXOR_OVERRUN 		 =0x80,
} LIS3MDL_MAG_ZYXOR_t;

#define  	LIS3MDL_MAG_ZYXOR_MASK  	0x80
mems_status_t LIS3MDL_MAG_R_DataXYZOverrun(LIS3MDL_MAG_ZYXOR_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: IEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_IEN_DISABLE 		 =0x00,
  	LIS3MDL_MAG_IEN_ENABLE 		 =0x01,
} LIS3MDL_MAG_IEN_t;

#define  	LIS3MDL_MAG_IEN_MASK  	0x01
mems_status_t  LIS3MDL_MAG_W_InterruptEnable(LIS3MDL_MAG_IEN_t newValue);
mems_status_t LIS3MDL_MAG_R_InterruptEnable(LIS3MDL_MAG_IEN_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: LIR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_LIR_LATCHED 		 =0x00,
  	LIS3MDL_MAG_LIR_NOT_LATCHED 		 =0x02,
} LIS3MDL_MAG_LIR_t;

#define  	LIS3MDL_MAG_LIR_MASK  	0x02
mems_status_t  LIS3MDL_MAG_W_LatchInterruptRq(LIS3MDL_MAG_LIR_t newValue);
mems_status_t LIS3MDL_MAG_R_LatchInterruptRq(LIS3MDL_MAG_LIR_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: IEA
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_IEA_LOW 		 =0x00,
  	LIS3MDL_MAG_IEA_HIGH 		 =0x04,
} LIS3MDL_MAG_IEA_t;

#define  	LIS3MDL_MAG_IEA_MASK  	0x04
mems_status_t  LIS3MDL_MAG_W_InterruptActive(LIS3MDL_MAG_IEA_t newValue);
mems_status_t LIS3MDL_MAG_R_InterruptActive(LIS3MDL_MAG_IEA_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: ZIEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_ZIEN_DISABLE 		 =0x00,
  	LIS3MDL_MAG_ZIEN_ENABLE 		 =0x20,
} LIS3MDL_MAG_ZIEN_t;

#define  	LIS3MDL_MAG_ZIEN_MASK  	0x20
mems_status_t  LIS3MDL_MAG_W_InterruptOnZ(LIS3MDL_MAG_ZIEN_t newValue);
mems_status_t LIS3MDL_MAG_R_InterruptOnZ(LIS3MDL_MAG_ZIEN_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: YIEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_YIEN_DISABLE 		 =0x00,
  	LIS3MDL_MAG_YIEN_ENABLE 		 =0x40,
} LIS3MDL_MAG_YIEN_t;

#define  	LIS3MDL_MAG_YIEN_MASK  	0x40
mems_status_t  LIS3MDL_MAG_W_InterruptOnY(LIS3MDL_MAG_YIEN_t newValue);
mems_status_t LIS3MDL_MAG_R_InterruptOnY(LIS3MDL_MAG_YIEN_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: XIEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_XIEN_DISABLE 		 =0x00,
  	LIS3MDL_MAG_XIEN_ENABLE 		 =0x80,
} LIS3MDL_MAG_XIEN_t;

#define  	LIS3MDL_MAG_XIEN_MASK  	0x80
mems_status_t  LIS3MDL_MAG_W_InterruptOnX(LIS3MDL_MAG_XIEN_t newValue);
mems_status_t LIS3MDL_MAG_R_InterruptOnX(LIS3MDL_MAG_XIEN_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: INT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_INT_DOWN 		 =0x00,
  	LIS3MDL_MAG_INT_UP 		 =0x01,
} LIS3MDL_MAG_INT_t;

#define  	LIS3MDL_MAG_INT_MASK  	0x01
mems_status_t  LIS3MDL_MAG_W_InterruptFlag(LIS3MDL_MAG_INT_t newValue);
mems_status_t LIS3MDL_MAG_R_InterruptFlag(LIS3MDL_MAG_INT_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: MROI
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_MROI_IN_RANGE 		 =0x00,
  	LIS3MDL_MAG_MROI_OVERFLOW 		 =0x02,
} LIS3MDL_MAG_MROI_t;

#define  	LIS3MDL_MAG_MROI_MASK  	0x02
mems_status_t  LIS3MDL_MAG_W_MagneticFieldOverflow(LIS3MDL_MAG_MROI_t newValue);
mems_status_t LIS3MDL_MAG_R_MagneticFieldOverflow(LIS3MDL_MAG_MROI_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: NTH_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_NTH_Z_DOWN 		 =0x00,
  	LIS3MDL_MAG_NTH_Z_UP 		 =0x04,
} LIS3MDL_MAG_NTH_Z_t;

#define  	LIS3MDL_MAG_NTH_Z_MASK  	0x04
mems_status_t  LIS3MDL_MAG_W_NegativeThresholdFlagZ(LIS3MDL_MAG_NTH_Z_t newValue);
mems_status_t LIS3MDL_MAG_R_NegativeThresholdFlagZ(LIS3MDL_MAG_NTH_Z_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: NTH_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_NTH_Y_DOWN 		 =0x00,
  	LIS3MDL_MAG_NTH_Y_UP 		 =0x08,
} LIS3MDL_MAG_NTH_Y_t;

#define  	LIS3MDL_MAG_NTH_Y_MASK  	0x08
mems_status_t  LIS3MDL_MAG_W_NegativeThresholdFlagY(LIS3MDL_MAG_NTH_Y_t newValue);
mems_status_t LIS3MDL_MAG_R_NegativeThresholdFlagY(LIS3MDL_MAG_NTH_Y_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: NTH_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_NTH_X_DOWN 		 =0x00,
  	LIS3MDL_MAG_NTH_X_UP 		 =0x10,
} LIS3MDL_MAG_NTH_X_t;

#define  	LIS3MDL_MAG_NTH_X_MASK  	0x10
mems_status_t  LIS3MDL_MAG_W_NegativeThresholdFlagX(LIS3MDL_MAG_NTH_X_t newValue);
mems_status_t LIS3MDL_MAG_R_NegativeThresholdFlagX(LIS3MDL_MAG_NTH_X_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: PTH_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_PTH_Z_DOWN 		 =0x00,
  	LIS3MDL_MAG_PTH_Z_UP 		 =0x20,
} LIS3MDL_MAG_PTH_Z_t;

#define  	LIS3MDL_MAG_PTH_Z_MASK  	0x20
mems_status_t  LIS3MDL_MAG_W_PositiveThresholdFlagZ(LIS3MDL_MAG_PTH_Z_t newValue);
mems_status_t LIS3MDL_MAG_R_PositiveThresholdFlagZ(LIS3MDL_MAG_PTH_Z_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: PTH_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_PTH_Y_DOWN 		 =0x00,
  	LIS3MDL_MAG_PTH_Y_UP 		 =0x40,
} LIS3MDL_MAG_PTH_Y_t;

#define  	LIS3MDL_MAG_PTH_Y_MASK  	0x40
mems_status_t  LIS3MDL_MAG_W_PositiveThresholdFlagY(LIS3MDL_MAG_PTH_Y_t newValue);
mems_status_t LIS3MDL_MAG_R_PositiveThresholdFlagY(LIS3MDL_MAG_PTH_Y_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: PTH_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS3MDL_MAG_PTH_X_DOWN 		 =0x00,
  	LIS3MDL_MAG_PTH_X_UP 		 =0x80,
} LIS3MDL_MAG_PTH_X_t;

#define  	LIS3MDL_MAG_PTH_X_MASK  	0x80


/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
void LIS3MDL_MAG_SwapHighLowByte(unsigned char *bufferToSwap, unsigned char numberOfByte, unsigned char dimension);

mems_status_t LIS3MDL_MAG_WriteRegN( unsigned char Reg, unsigned char *Bufp, unsigned short len );
mems_status_t LIS3MDL_MAG_ReadRegN( unsigned char Reg, unsigned char *Bufp, unsigned short len );

mems_status_t LIS3MDL_MAG_R_WHO_AM_I_(unsigned char *value);
mems_status_t  LIS3MDL_MAG_W_SystemOperatingMode(LIS3MDL_MAG_MD_t newValue);
mems_status_t LIS3MDL_MAG_R_SystemOperatingMode(LIS3MDL_MAG_MD_t *value);
mems_status_t  LIS3MDL_MAG_W_BlockDataUpdate(LIS3MDL_MAG_BDU_t newValue);
mems_status_t LIS3MDL_MAG_R_BlockDataUpdate(LIS3MDL_MAG_BDU_t *value);
mems_status_t  LIS3MDL_MAG_W_FullScale(LIS3MDL_MAG_FS_t newValue);
mems_status_t LIS3MDL_MAG_R_FullScale(LIS3MDL_MAG_FS_t *value);
mems_status_t  LIS3MDL_MAG_W_OutputDataRate(LIS3MDL_MAG_DO_t newValue);
mems_status_t LIS3MDL_MAG_R_OutputDataRate(LIS3MDL_MAG_DO_t *value);
mems_status_t  LIS3MDL_MAG_W_SelfTest(LIS3MDL_MAG_ST_t newValue);
mems_status_t LIS3MDL_MAG_R_SelfTest(LIS3MDL_MAG_ST_t *value);
mems_status_t  LIS3MDL_MAG_W_OperatingModeXY(LIS3MDL_MAG_OM_t newValue);
mems_status_t LIS3MDL_MAG_R_OperatingModeXY(LIS3MDL_MAG_OM_t *value);
mems_status_t  LIS3MDL_MAG_W_TemperatureSensor_EnDis(LIS3MDL_MAG_TEMP_EN_t newValue);
mems_status_t LIS3MDL_MAG_R_TemperatureSensorEnDis(LIS3MDL_MAG_TEMP_EN_t *value);
mems_status_t  LIS3MDL_MAG_W_SoftRST(LIS3MDL_MAG_SOFT_RST_t newValue);
mems_status_t LIS3MDL_MAG_R_SoftRST(LIS3MDL_MAG_SOFT_RST_t *value);
mems_status_t  LIS3MDL_MAG_W_Reboot(LIS3MDL_MAG_REBOOT_t newValue);
mems_status_t LIS3MDL_MAG_R_Reboot(LIS3MDL_MAG_REBOOT_t *value);
mems_status_t  LIS3MDL_MAG_W_SerialInterfaceMode(LIS3MDL_MAG_SIM_t newValue);
mems_status_t LIS3MDL_MAG_R_SerialInterfaceMode(LIS3MDL_MAG_SIM_t *value);
mems_status_t  LIS3MDL_MAG_W_PositiveThresholdFlagX(LIS3MDL_MAG_PTH_X_t newValue);
mems_status_t LIS3MDL_MAG_R_PositiveThresholdFlagX(LIS3MDL_MAG_PTH_X_t *value);
mems_status_t LIS3MDL_MAG_Get_Temperature(unsigned char *buff);
mems_status_t LIS3MDL_MAG_Set_MagneticThreshold(unsigned char *buff);
mems_status_t LIS3MDL_MAG_Get_MagneticThreshold(unsigned char *buff);
mems_status_t LIS3MDL_MAG_Get_Magnetic(unsigned char *buff);

unsigned char LIS3MDL_MAG_WriteRegN( unsigned char WriteAddr, unsigned char *pBuffer, unsigned short nBytesToWrite );
unsigned char LIS3MDL_IO_Read( unsigned char ReadAddr, unsigned char *pBuffer, unsigned short nBytesToRead );

typedef struct {
	  char name[10];
  unsigned short scale;
  short min[3];
  short max[3];
} LIS3MDL_t;



LIS3MDL_t g_LIS3MDL;

const static unsigned short LIS3MDLGAUSS_TO_SCALE[] = { 4, 8, 12, 16 };

// **************** Writes 8-bits to the specified destination register **************
void LIS3MDL_writeRegister8(unsigned char reg, unsigned char value) {
	unsigned char regbuf[10];
   	int ret=0;
   	regbuf[0] = reg;
   	regbuf[1] = value;
    ret = stm_I2C_SendBurst(LIS3MDL_8BIT_I2C_ADDRESS, regbuf, 2);
}


mems_status_t LIS3MDL_MAG_WriteRegN(unsigned char reg, unsigned char *pBuffer, unsigned short nBytesToWrite )
{
	unsigned char regbuf[32];
   	int ret=0;
   	regbuf[0] = reg;
   	memcpy(&regbuf[1], pBuffer,nBytesToWrite);
    ret = stm_I2C_SendBurst(LIS3MDL_8BIT_I2C_ADDRESS, regbuf, nBytesToWrite + 1);
    if (ret == 0)     return MEMS_ERROR;
    else     return MEMS_SUCCESS;

}

  /**************************************************************************/
  /*  Reads 8-bits from the specified register  */
  /**************************************************************************/
  unsigned char LIS3MDL_readRegister8(unsigned char reg) {
    unsigned char value;
   	unsigned char regbuf[10];
   	int ret=0;

	stm_I2C_ReceiveBurstWithRestartCondition(LIS3MDL_8BIT_I2C_ADDRESS, reg, &regbuf[0], 1);
    value = regbuf[0];
    return value;
  }

  unsigned short LIS3MDL_readRegister16(unsigned char regL) {
    unsigned short vals;
   	unsigned char regbuf[10];
   	int ret=0;

   	//L-H
	stm_I2C_ReceiveBurstWithRestartCondition(LIS3MDL_8BIT_I2C_ADDRESS, regL, &regbuf[0], 2);
	vals = regbuf[1];
    vals = (vals << 8) + regbuf[0];
    printf("regbuf[1]=%02x; [0] = %02x; vals= %04x\r\n",regbuf[1], regbuf[0], vals);
    return vals;
  }

mems_status_t  LIS3MDL_MAG_ReadRegN(unsigned char reg, unsigned char *retVal, unsigned short len){
  int ret=0;

  if(len > 1)
	  reg |= LIS3MDL_I2C_AUTOINCREMENT;//0x80;// multi bytes

  ret = stm_I2C_ReceiveBurstWithRestartCondition(LIS3MDL_8BIT_I2C_ADDRESS, reg, retVal, len);
  if(ret == 0)
	  return MEMS_ERROR;
  else
	  return MEMS_SUCCESS;
}

/**
 * @brief  Read Magnetometer Sensitivity
 * @param  pfData the pointer where the magnetometer sensitivity is stored
 * @retval LIS3MDL_OK in case of success, an error code otherwise
 */
char LIS3MDL_GetSensitivity(float *pfData)
{
  LIS3MDL_MAG_FS_t fullScale;

  /* Read actual full scale selection from sensor. */
  if ( LIS3MDL_MAG_R_FullScale(  &fullScale ) == MEMS_ERROR )
  {
    return LIS3MDL_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case LIS3MDL_MAG_FS_4Ga:
      *pfData = ( float )LIS3MDL_MAG_SENSITIVITY_FOR_FS_4G;
      break;
    case LIS3MDL_MAG_FS_8Ga:
      *pfData = ( float )LIS3MDL_MAG_SENSITIVITY_FOR_FS_8G;
      break;
    case LIS3MDL_MAG_FS_12Ga:
      *pfData = ( float )LIS3MDL_MAG_SENSITIVITY_FOR_FS_12G;
      break;
    case LIS3MDL_MAG_FS_16Ga:
      *pfData = ( float )LIS3MDL_MAG_SENSITIVITY_FOR_FS_16G;
      break;
    default:
      *pfData = -1.0f;
      break;
  }

  return LIS3MDL_OK;
}

/**
 * @brief  Read raw data from LIS3MDL Magnetometer
 * @param  sRaw the pointer where the magnetomer raw data are stored
 * @retval LIS3MDL_OK in case of success, an error code otherwise
 */
char stmLIS3MDL_MAG_GetAxesRaw(short *sRaw)
{
  unsigned char ucValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from LIS3MDL_MAG_OUTX_L to LIS3MDL_MAG_OUTZ_H. */
  if ( LIS3MDL_MAG_Get_Magnetic(  ( unsigned char* )ucValue ) == MEMS_ERROR )
  {
    return LIS3MDL_ERROR;
  }

  /* Format the data. */
  sRaw[0] = ( ( ( ( short )ucValue[1] ) << 8 ) + ( short )ucValue[0] );
  sRaw[1] = ( ( ( ( short )ucValue[3] ) << 8 ) + ( short )ucValue[2] );
  sRaw[2] = ( ( ( ( short )ucValue[5] ) << 8 ) + ( short )ucValue[4] );

  printf("raw(x,y,z)=(%d, %d, %d)\r\n",sRaw[0],sRaw[1],sRaw[2] );
  return LIS3MDL_OK;
}
/**
 * @brief  Read data from LIS3MDL Magnetometer
 * @param  pData the pointer where the magnetometer data are stored
 * @retval LIS3MDL_OK in case of success, an error code otherwise
 */
char stmLIS3MDL_MAG_GetAxes(float *fData)
{
  short sDataRaw[3];
  float f_raw[3];
  float sensitivity = 0.0;

  /* Read raw data from LIS3MDL output register. */
  if ( stmLIS3MDL_MAG_GetAxesRaw( sDataRaw ) == LIS3MDL_ERROR )
  {
    return LIS3MDL_ERROR;
  }

  /* Get LIS3MDL actual sensitivity. */
  if ( LIS3MDL_GetSensitivity( &sensitivity ) == LIS3MDL_ERROR )
  {
    return LIS3MDL_ERROR;
  }
  printf("Sensitivity = %s\r\n", float2str(sensitivity));

  /* Calculate the data. */
  fData[0] = ( float )( sDataRaw[0] * sensitivity );
  fData[1] = ( float )( sDataRaw[1] * sensitivity );
  fData[2] = ( float )( sDataRaw[2] * sensitivity );

  printf("(x,y,z)=(%s,", float2str(fData[0]));
  printf("%s,", float2str(fData[1]));
  printf("%s)\r\n", float2str(fData[2]));

  return LIS3MDL_OK;
}

/**
 * @brief  Set ODR
 * @param  odr the output data rate to be set
 * @retval LIS3MDL_OK in case of success, an error code otherwise
 */
char LIS3MDL_SetODR(float odr)
{
  LIS3MDL_MAG_DO_t new_odr;

  new_odr = ( odr <=  0.625f ) ? LIS3MDL_MAG_DO_0_625Hz
          : ( odr <=  1.250f ) ? LIS3MDL_MAG_DO_1_25Hz
          : ( odr <=  2.500f ) ? LIS3MDL_MAG_DO_2_5Hz
          : ( odr <=  5.000f ) ? LIS3MDL_MAG_DO_5Hz
          : ( odr <= 10.000f ) ? LIS3MDL_MAG_DO_10Hz
          : ( odr <= 20.000f ) ? LIS3MDL_MAG_DO_20Hz
          : ( odr <= 40.000f ) ? LIS3MDL_MAG_DO_40Hz
          :                      LIS3MDL_MAG_DO_80Hz;

  if ( LIS3MDL_MAG_W_OutputDataRate(  new_odr ) == MEMS_ERROR )
  {
    return LIS3MDL_ERROR;
  }

  return LIS3MDL_OK;
}


/**
 * @brief  Set full scale
 * @param  fullScale the full scale to be set
 * @retval LIS3MDL_OK in case of success, an error code otherwise
 */
char LIS3MDL_SetFS(float fullScale)
{
  LIS3MDL_MAG_FS_t new_fs;

  new_fs = ( fullScale <=  4.0f ) ? LIS3MDL_MAG_FS_4Ga
         : ( fullScale <=  8.0f ) ? LIS3MDL_MAG_FS_8Ga
         : ( fullScale <= 12.0f ) ? LIS3MDL_MAG_FS_12Ga
         :                          LIS3MDL_MAG_FS_16Ga;

  if ( LIS3MDL_MAG_W_FullScale(  new_fs ) == MEMS_ERROR )
  {
    return LIS3MDL_ERROR;
  }

  return LIS3MDL_OK;
}



mems_status_t LIS3MDL_MAG_R_WHO_AM_I_(unsigned char *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_WHO_AM_I_REG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_WHO_AM_I_BIT_MASK; //coerce
  *value = *value >> LIS3MDL_MAG_WHO_AM_I_BIT_POSITION; //mask

  return MEMS_SUCCESS;
}

mems_status_t  LIS3MDL_MAG_W_SystemOperatingMode(LIS3MDL_MAG_MD_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_MD_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_SystemOperatingMode
* Description    : Read MD
* Input          : Pointer to LIS3MDL_MAG_MD_t
* Output         : Status of MD see LIS3MDL_MAG_MD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_SystemOperatingMode(LIS3MDL_MAG_MD_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG3, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_MD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_BlockDataUpdate
* Description    : Write BDU
* Input          : LIS3MDL_MAG_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_BlockDataUpdate(LIS3MDL_MAG_BDU_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_BDU_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_BlockDataUpdate
* Description    : Read BDU
* Input          : Pointer to LIS3MDL_MAG_BDU_t
* Output         : Status of BDU see LIS3MDL_MAG_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_BlockDataUpdate(LIS3MDL_MAG_BDU_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG5, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_FullScale
* Description    : Write FS
* Input          : LIS3MDL_MAG_FS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_FullScale(LIS3MDL_MAG_FS_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_FS_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_FullScale
* Description    : Read FS
* Input          : Pointer to LIS3MDL_MAG_FS_t
* Output         : Status of FS see LIS3MDL_MAG_FS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_FullScale(LIS3MDL_MAG_FS_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG2, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_FS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_OutputDataRate
* Description    : Write DO
* Input          : LIS3MDL_MAG_DO_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_OutputDataRate(LIS3MDL_MAG_DO_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_DO_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_OutputDataRate
* Description    : Read DO
* Input          : Pointer to LIS3MDL_MAG_DO_t
* Output         : Status of DO see LIS3MDL_MAG_DO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_OutputDataRate(LIS3MDL_MAG_DO_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG1, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_DO_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : mems_status_t LIS3MDL_MAG_Get_Magnetic(unsigned char *buff)
* Description    : Read Magnetic output register
* Input          : pointer to [unsigned char]
* Output         : Magnetic buffer unsigned char
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_Get_Magnetic(unsigned char *buff)
{
  unsigned char i, j, k;
  unsigned char numberOfByteForDimension;

  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ )
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{
		if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_OUTX_L+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;
	}
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_SelfTest
* Description    : Write ST
* Input          : LIS3MDL_MAG_ST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_SelfTest(LIS3MDL_MAG_ST_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_ST_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_SelfTest
* Description    : Read ST
* Input          : Pointer to LIS3MDL_MAG_ST_t
* Output         : Status of ST see LIS3MDL_MAG_ST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_SelfTest(LIS3MDL_MAG_ST_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG1, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_OperatingModeXY
* Description    : Write OM
* Input          : LIS3MDL_MAG_OM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_OperatingModeXY(LIS3MDL_MAG_OM_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_OM_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_OperatingModeXY
* Description    : Read OM
* Input          : Pointer to LIS3MDL_MAG_OM_t
* Output         : Status of OM see LIS3MDL_MAG_OM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_OperatingModeXY(LIS3MDL_MAG_OM_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG1, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_OM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_TemperatureSensor_EnDis
* Description    : Write TEMP_EN
* Input          : LIS3MDL_MAG_TEMP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_TemperatureSensor_EnDis(LIS3MDL_MAG_TEMP_EN_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_TEMP_EN_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_TemperatureSensorEnDis
* Description    : Read TEMP_EN
* Input          : Pointer to LIS3MDL_MAG_TEMP_EN_t
* Output         : Status of TEMP_EN see LIS3MDL_MAG_TEMP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_TemperatureSensorEnDis(LIS3MDL_MAG_TEMP_EN_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG1, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_TEMP_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_SoftRST
* Description    : Write SOFT_RST
* Input          : LIS3MDL_MAG_SOFT_RST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_SoftRST(LIS3MDL_MAG_SOFT_RST_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_SOFT_RST_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_SoftRST
* Description    : Read SOFT_RST
* Input          : Pointer to LIS3MDL_MAG_SOFT_RST_t
* Output         : Status of SOFT_RST see LIS3MDL_MAG_SOFT_RST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_SoftRST(LIS3MDL_MAG_SOFT_RST_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG2, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_SOFT_RST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_Reboot
* Description    : Write REBOOT
* Input          : LIS3MDL_MAG_REBOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_Reboot(LIS3MDL_MAG_REBOOT_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_REBOOT_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_Reboot
* Description    : Read REBOOT
* Input          : Pointer to LIS3MDL_MAG_REBOOT_t
* Output         : Status of REBOOT see LIS3MDL_MAG_REBOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_Reboot(LIS3MDL_MAG_REBOOT_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG2, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_REBOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_SerialInterfaceMode
* Description    : Write SIM
* Input          : LIS3MDL_MAG_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_SerialInterfaceMode(LIS3MDL_MAG_SIM_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_SIM_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_SerialInterfaceMode
* Description    : Read SIM
* Input          : Pointer to LIS3MDL_MAG_SIM_t
* Output         : Status of SIM see LIS3MDL_MAG_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_SerialInterfaceMode(LIS3MDL_MAG_SIM_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG3, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_SIM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_FastLowPowerXYZ
* Description    : Write LP
* Input          : LIS3MDL_MAG_LP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_FastLowPowerXYZ(LIS3MDL_MAG_LP_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_LP_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_FastLowPowerXYZ
* Description    : Read LP
* Input          : Pointer to LIS3MDL_MAG_LP_t
* Output         : Status of LP see LIS3MDL_MAG_LP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_FastLowPowerXYZ(LIS3MDL_MAG_LP_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG3, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_LP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_LittleBigEndianInversion
* Description    : Write BLE
* Input          : LIS3MDL_MAG_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_LittleBigEndianInversion(LIS3MDL_MAG_BLE_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_BLE_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_LittleBigEndianInversion
* Description    : Read BLE
* Input          : Pointer to LIS3MDL_MAG_BLE_t
* Output         : Status of BLE see LIS3MDL_MAG_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_LittleBigEndianInversion(LIS3MDL_MAG_BLE_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG4, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_OperatingModeZ
* Description    : Write OMZ
* Input          : LIS3MDL_MAG_OMZ_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_OperatingModeZ(LIS3MDL_MAG_OMZ_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_OMZ_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_OperatingModeZ
* Description    : Read OMZ
* Input          : Pointer to LIS3MDL_MAG_OMZ_t
* Output         : Status of OMZ see LIS3MDL_MAG_OMZ_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_OperatingModeZ(LIS3MDL_MAG_OMZ_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_CTRL_REG4, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_OMZ_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NewXData
* Description    : Read XDA
* Input          : Pointer to LIS3MDL_MAG_XDA_t
* Output         : Status of XDA see LIS3MDL_MAG_XDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_NewXData(LIS3MDL_MAG_XDA_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_STATUS_REG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_XDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NewYData
* Description    : Read YDA
* Input          : Pointer to LIS3MDL_MAG_YDA_t
* Output         : Status of YDA see LIS3MDL_MAG_YDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_NewYData(LIS3MDL_MAG_YDA_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_STATUS_REG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_YDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NewZData
* Description    : Read ZDA
* Input          : Pointer to LIS3MDL_MAG_ZDA_t
* Output         : Status of ZDA see LIS3MDL_MAG_ZDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_NewZData(LIS3MDL_MAG_ZDA_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_STATUS_REG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ZDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NewXYZData
* Description    : Read ZYXDA
* Input          : Pointer to LIS3MDL_MAG_ZYXDA_t
* Output         : Status of ZYXDA see LIS3MDL_MAG_ZYXDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_NewXYZData(LIS3MDL_MAG_ZYXDA_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_STATUS_REG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ZYXDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_DataXOverrun
* Description    : Read XOR
* Input          : Pointer to LIS3MDL_MAG_XOR_t
* Output         : Status of XOR see LIS3MDL_MAG_XOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_DataXOverrun(LIS3MDL_MAG_XOR_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_STATUS_REG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_XOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_DataYOverrun
* Description    : Read YOR
* Input          : Pointer to LIS3MDL_MAG_YOR_t
* Output         : Status of YOR see LIS3MDL_MAG_YOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_DataYOverrun(LIS3MDL_MAG_YOR_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_STATUS_REG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_YOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_DataZOverrun
* Description    : Read ZOR
* Input          : Pointer to LIS3MDL_MAG_ZOR_t
* Output         : Status of ZOR see LIS3MDL_MAG_ZOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_DataZOverrun(LIS3MDL_MAG_ZOR_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_STATUS_REG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ZOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_DataXYZOverrun
* Description    : Read ZYXOR
* Input          : Pointer to LIS3MDL_MAG_ZYXOR_t
* Output         : Status of ZYXOR see LIS3MDL_MAG_ZYXOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_DataXYZOverrun(LIS3MDL_MAG_ZYXOR_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_STATUS_REG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ZYXOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptEnable
* Description    : Write IEN
* Input          : LIS3MDL_MAG_IEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_InterruptEnable(LIS3MDL_MAG_IEN_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_IEN_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptEnable
* Description    : Read IEN
* Input          : Pointer to LIS3MDL_MAG_IEN_t
* Output         : Status of IEN see LIS3MDL_MAG_IEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_InterruptEnable(LIS3MDL_MAG_IEN_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_IEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_LatchInterruptRq
* Description    : Write LIR
* Input          : LIS3MDL_MAG_LIR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_LatchInterruptRq(LIS3MDL_MAG_LIR_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_LIR_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_LatchInterruptRq
* Description    : Read LIR
* Input          : Pointer to LIS3MDL_MAG_LIR_t
* Output         : Status of LIR see LIS3MDL_MAG_LIR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_LatchInterruptRq(LIS3MDL_MAG_LIR_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_LIR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptActive
* Description    : Write IEA
* Input          : LIS3MDL_MAG_IEA_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_InterruptActive(LIS3MDL_MAG_IEA_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_IEA_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptActive
* Description    : Read IEA
* Input          : Pointer to LIS3MDL_MAG_IEA_t
* Output         : Status of IEA see LIS3MDL_MAG_IEA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_InterruptActive(LIS3MDL_MAG_IEA_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_IEA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptOnZ
* Description    : Write ZIEN
* Input          : LIS3MDL_MAG_ZIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_InterruptOnZ(LIS3MDL_MAG_ZIEN_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_ZIEN_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptOnZ
* Description    : Read ZIEN
* Input          : Pointer to LIS3MDL_MAG_ZIEN_t
* Output         : Status of ZIEN see LIS3MDL_MAG_ZIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_InterruptOnZ(LIS3MDL_MAG_ZIEN_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_ZIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptOnY
* Description    : Write YIEN
* Input          : LIS3MDL_MAG_YIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_InterruptOnY(LIS3MDL_MAG_YIEN_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_YIEN_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptOnY
* Description    : Read YIEN
* Input          : Pointer to LIS3MDL_MAG_YIEN_t
* Output         : Status of YIEN see LIS3MDL_MAG_YIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_InterruptOnY(LIS3MDL_MAG_YIEN_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_YIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptOnX
* Description    : Write XIEN
* Input          : LIS3MDL_MAG_XIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_InterruptOnX(LIS3MDL_MAG_XIEN_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_XIEN_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptOnX
* Description    : Read XIEN
* Input          : Pointer to LIS3MDL_MAG_XIEN_t
* Output         : Status of XIEN see LIS3MDL_MAG_XIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_InterruptOnX(LIS3MDL_MAG_XIEN_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_CFG, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_XIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_InterruptFlag
* Description    : Write INT
* Input          : LIS3MDL_MAG_INT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_InterruptFlag(LIS3MDL_MAG_INT_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_INT_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_InterruptFlag
* Description    : Read INT
* Input          : Pointer to LIS3MDL_MAG_INT_t
* Output         : Status of INT see LIS3MDL_MAG_INT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_InterruptFlag(LIS3MDL_MAG_INT_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_INT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_MagneticFieldOverflow
* Description    : Write MROI
* Input          : LIS3MDL_MAG_MROI_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_MagneticFieldOverflow(LIS3MDL_MAG_MROI_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_MROI_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_MagneticFieldOverflow
* Description    : Read MROI
* Input          : Pointer to LIS3MDL_MAG_MROI_t
* Output         : Status of MROI see LIS3MDL_MAG_MROI_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_MagneticFieldOverflow(LIS3MDL_MAG_MROI_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_MROI_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_NegativeThresholdFlagZ
* Description    : Write NTH_Z
* Input          : LIS3MDL_MAG_NTH_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_NegativeThresholdFlagZ(LIS3MDL_MAG_NTH_Z_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_NTH_Z_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NegativeThresholdFlagZ
* Description    : Read NTH_Z
* Input          : Pointer to LIS3MDL_MAG_NTH_Z_t
* Output         : Status of NTH_Z see LIS3MDL_MAG_NTH_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_NegativeThresholdFlagZ(LIS3MDL_MAG_NTH_Z_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_NTH_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_NegativeThresholdFlagY
* Description    : Write NTH_Y
* Input          : LIS3MDL_MAG_NTH_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_NegativeThresholdFlagY(LIS3MDL_MAG_NTH_Y_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_NTH_Y_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NegativeThresholdFlagY
* Description    : Read NTH_Y
* Input          : Pointer to LIS3MDL_MAG_NTH_Y_t
* Output         : Status of NTH_Y see LIS3MDL_MAG_NTH_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_NegativeThresholdFlagY(LIS3MDL_MAG_NTH_Y_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_NTH_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_NegativeThresholdFlagX
* Description    : Write NTH_X
* Input          : LIS3MDL_MAG_NTH_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_NegativeThresholdFlagX(LIS3MDL_MAG_NTH_X_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_NTH_X_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_NegativeThresholdFlagX
* Description    : Read NTH_X
* Input          : Pointer to LIS3MDL_MAG_NTH_X_t
* Output         : Status of NTH_X see LIS3MDL_MAG_NTH_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_NegativeThresholdFlagX(LIS3MDL_MAG_NTH_X_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_NTH_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_PositiveThresholdFlagZ
* Description    : Write PTH_Z
* Input          : LIS3MDL_MAG_PTH_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_PositiveThresholdFlagZ(LIS3MDL_MAG_PTH_Z_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_PTH_Z_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_PositiveThresholdFlagZ
* Description    : Read PTH_Z
* Input          : Pointer to LIS3MDL_MAG_PTH_Z_t
* Output         : Status of PTH_Z see LIS3MDL_MAG_PTH_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_PositiveThresholdFlagZ(LIS3MDL_MAG_PTH_Z_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_PTH_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_PositiveThresholdFlagY
* Description    : Write PTH_Y
* Input          : LIS3MDL_MAG_PTH_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_PositiveThresholdFlagY(LIS3MDL_MAG_PTH_Y_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_PTH_Y_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_PositiveThresholdFlagY
* Description    : Read PTH_Y
* Input          : Pointer to LIS3MDL_MAG_PTH_Y_t
* Output         : Status of PTH_Y see LIS3MDL_MAG_PTH_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_PositiveThresholdFlagY(LIS3MDL_MAG_PTH_Y_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_PTH_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_W_PositiveThresholdFlagX
* Description    : Write PTH_X
* Input          : LIS3MDL_MAG_PTH_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t  LIS3MDL_MAG_W_PositiveThresholdFlagX(LIS3MDL_MAG_PTH_X_t newValue)
{
  unsigned char value;

  if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS3MDL_MAG_PTH_X_MASK;
  value |= newValue;

  if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3MDL_MAG_R_PositiveThresholdFlagX
* Description    : Read PTH_X
* Input          : Pointer to LIS3MDL_MAG_PTH_X_t
* Output         : Status of PTH_X see LIS3MDL_MAG_PTH_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_R_PositiveThresholdFlagX(LIS3MDL_MAG_PTH_X_t *value)
{
 if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_SRC, (unsigned char *)value, 1) )
    return MEMS_ERROR;

  *value &= LIS3MDL_MAG_PTH_X_MASK; //mask

  return MEMS_SUCCESS;
}

//Read Temperature output register
mems_status_t LIS3MDL_MAG_Get_Temperature(unsigned char *buff)
{
  if(!LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_TEMP_OUT_L, buff, 2))
	  return MEMS_ERROR;

//  buff[0] = LIS3MDL_readRegister8(LIS3MDL_MAG_TEMP_OUT_L);
//  buff[1] = LIS3MDL_readRegister8(LIS3MDL_MAG_TEMP_OUT_H);

  return MEMS_SUCCESS;
}

short LIS3MDL_MAG_readTemperature() {
	short vals;
	unsigned char buf[2];

	LIS3MDL_MAG_Get_Temperature(buf);//(LIS3MDL_MAG_TEMP_OUT_L and LIS3MDL_MAG_TEMP_OUT_H);

	//printf("temper: buf[1..0]=%d %d\r\n", buf[1], buf[0]);
	vals = buf[1];
	vals = (short)(vals << 8) + buf[0];
	return vals;
}

/*******************************************************************************
* Function Name  : mems_status_t LIS3MDL_MAG_Set_MagneticThreshold(unsigned char *buff)
* Description    : Set MagneticThreshold data row
* Input          : pointer to [unsigned char]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_Set_MagneticThreshold(unsigned char *buff)
{
  unsigned char  i;

  for (i=0; i<2;i++ )
  {
	if( !LIS3MDL_MAG_WriteRegN(LIS3MDL_MAG_INT_THS_L+i,  &buff[i], 1) )
		return MEMS_ERROR;
  }
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : mems_status_t LIS3MDL_MAG_Get_MagneticThreshold(unsigned char *buff)
* Description    : Read MagneticThreshold output register
* Input          : pointer to [unsigned char]
* Output         : MagneticThreshold buffer unsigned char
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t LIS3MDL_MAG_Get_MagneticThreshold(unsigned char *buff)
{
  unsigned char i, j, k;
  unsigned char numberOfByteForDimension;

  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ )
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{
		if( !LIS3MDL_MAG_ReadRegN(LIS3MDL_MAG_INT_THS_L+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;
	}
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name		: SwapHighLowByte
* Description		: Swap High/low byte in multiple byte values
*                     It works with minimum 2 byte for every dimension.
*                     Example x,y,z with 2 byte for every dimension
*
* Input				: bufferToSwap -> buffer to swap
*                     numberOfByte -> the buffer length in byte
*                     dimension -> number of dimension
*
* Output			: bufferToSwap -> buffer swapped
* Return			: None
*******************************************************************************/
void LIS3MDL_MAG_SwapHighLowByte(unsigned char *bufferToSwap, unsigned char numberOfByte, unsigned char dimension)
{
  unsigned char numberOfByteForDimension, i, j;
  unsigned char tempValue[10];

  numberOfByteForDimension=numberOfByte/dimension;

  for (i=0; i<dimension;i++ )
  {
	for (j=0; j<numberOfByteForDimension;j++ )
		tempValue[j]=bufferToSwap[j+i*numberOfByteForDimension];
	for (j=0; j<numberOfByteForDimension;j++ )
		*(bufferToSwap+i*(numberOfByteForDimension)+j)=*(tempValue+(numberOfByteForDimension-1)-j);
  }
}
char LIS3MDL_Enable(void)
{
  // Operating mode selection - continuous
  if ( LIS3MDL_MAG_W_SystemOperatingMode( LIS3MDL_MAG_MD_CONTINUOUS ) == MEMS_ERROR )
  {
    return LIS3MDL_ERROR;
  }

  return LIS3MDL_OK;
}

/**
 * @brief  Disable LIS3MDL magnetometer
 * @retval LIS3MDL_OK in case of success, an error code otherwise
 */
char LIS3MDLSensor_Disable(void)
{
  /* Operating mode selection - power down */
  if ( LIS3MDL_MAG_W_SystemOperatingMode( LIS3MDL_MAG_MD_POWER_DOWN ) == MEMS_ERROR )
  {
    return LIS3MDL_ERROR;
  }

  return LIS3MDL_OK;
}

/*
//PB0_INT2pin
//PB3_INT1pin
#if(PROCESSOR == PROCESSOR_STM32F103C8T6)
void stmLIS3MDL_INT_1_2_pin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  //=======================================================================
	  //[VERY IMPORTANT] PB3's default function is JTDO. (and PB4's default function is JNTRST.)
	  //Thus we shoul remap to GPIO as the follows.
 	  //=======================================================================
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3 Remap !< JTAG-DP Disabled and SW-DP Enabled
	  //PB3 - INT1
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3); //Different from M4

	  //Enable and set EXTI Line3 Interrupt to the lowest priority
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  // Configure EXTI Line3
	  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

 	  //PB0-INT2 ==================================
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0); //Different from M4

	  // Enable and set EXTI Line0 Interrupt to the lowest priority
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;//Same Prio
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  // Configure EXTI Line0  //for INT2
	  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
}
#endif
*/

//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39 | STM32F103-M70
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| INT(Pin7)       |           |           | *remap    | PB4*          | PA4    		   | PB11
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| RDY(Pin8)       |           |           |           | PB5           |PB11              | PA4
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------

//PB4_INTpin (PIN 7)
//PB5_DRDYpin (PIN8)
#if(PROCESSOR == PROCESSOR_STM32F103C8T6)
void stmLIS3MDL_INT_DRDY_pin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;
#if (MCU_MODULE_VER == 35)
 	  //PB4-INT ==================================
	  //=======================================================================
	  //[VERY IMPORTANT] PB4's default function is JNTRST.
	  //Thus we shoul remap to GPIO as the follows.
 	  //=======================================================================
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3 Remap !< JTAG-DP Disabled and SW-DP Enabled
 	  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

	  // Enable and set EXTI Line4 Interrupt to the lowest priority
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;//Same Prio
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  // Configure EXTI Line4  //for INT4
	  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  //PB5 - DRDY
   	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  	  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
  	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	  GPIO_Init(GPIOB, &GPIO_InitStructure);
  	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

  	  //Enable and set EXTI Line5 Interrupt to the lowest priority
  	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;
  	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	  NVIC_Init(&NVIC_InitStructure);

  	  // Configure EXTI Line5
  	  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
  	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	  EXTI_Init(&EXTI_InitStructure);
#else
#endif
}
#elif(PROCESSOR == PROCESSOR_STM32F103RCT6)
//PC4
void stmLIS3MDL_INT_DRDY_pin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

 	  //PC4-INT and DRDY ==================================
 	  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
/*
	  // Enable and set EXTI Line4 Interrupt to the lowest priority
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;//Same Prio
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  // Configure EXTI Line4  //for INT4
	  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
*/
}
#endif
volatile int g_INT_LIS3MDL, g_RDRY_LIS3MDL;
#if (USE_EXTI4 == USE_EXTI4_LIS3MDL)
//for PB4_INTpin
void EXTI4_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET) {
		EXTI_ClearFlag(EXTI_Line4);//== EXTI_ClearITPendingBit(EXTI_Line4);		// Clear the EXTI line 0 pending bit
		g_INT_LIS3MDL = 1;
	}
}
#endif
#if(USE_EXTI9_5 ==	USE_EXTI9_5_LIS3MDL_RDY)
//for PB5_DRDY
void EXTI9_5_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
		EXTI_ClearFlag(EXTI_Line5);
		g_RDRY_LIS3MDL = 1;
		//EXTI_ClearITPendingBit(EXTI_Line5);		// Clear the EXTI line 8 pending bit (PA8)
	}
}
#endif




void stmLIS3MDL_ConfigIo(){
	g_INT_LIS3MDL = 0;
	g_RDRY_LIS3MDL = 0;

	if(!g_bI2CModuleConfigDone)
	{
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
		  gI2Cx = I2C2;
#elif(PROCESSOR == PROCESSOR_STM32F103C8T6 || PROCESSOR == PROCESSOR_STM32F103RCT6)
		  gI2Cx = I2C1;
#endif
		printf("I2C Init...");
		stm_I2C_Init(gI2Cx,400000);//400Kbps
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	}

	//LED
	stmUser_LED_GPIO_setup(); //Accessory LED

#if(PROCESSOR == PROCESSOR_STM32F103C8T6 || PROCESSOR == PROCESSOR_STM32F103RCT6)
	//INT = PB4 (remap)
	//RDY = PB5
	stmLIS3MDL_INT_DRDY_pin_Setup();
#else
#endif
	delayms(100);
}

/* (1) General Init Sequence
 * Write CTRL1
 * Write CTRL2
 * Write CTRL3
 * Write CTRL4
 * Write CTRL5
 * Write CTRL6
 * Write REFERENCE
 * Write INTx_THS
 * Write INTx_DUR
 * Write INTx_CFG
 * Write CTRL5
 *
 * (2) Reading Acc
 * a) Using Status Reg
 * 		1. Read Status Reg.
 * 		2. If STATUS_REG(3) = 0, goto 1
 * 		3. else if STATUS_REG(7) = 1, some data have been overwritten
 * 		4. Read OUTX_L, _H, OUTY_L,_H, ...Z
 * 		5. Do processing
 * 		6. goto 1
 * b) Using DataReady Signal (INTERRUPT)
 * 		ZYXDA can be drive to INT1 pin when I1_ZYXDA bit of CTRL3(0x22) is set to '1'.
 *
 * (3) Interrupt generation method
 *  The interrupts behave as free-fall, wake-up, 6D and 4D orientation detection, and click detection with INT1 and INT2 pins.
 *
 */

bool stmLIS3MDL_MAG_ConfIo_Init()
{
	unsigned char i;
	unsigned char deviceid;
	unsigned char reg2;
	LIS3MDL_MAG_TEMP_EN_t tempEnDis;

	stmLIS3MDL_ConfigIo();

	deviceid = LIS3MDL_readRegister8(LIS3MDL_MAG_WHO_AM_I_REG); //
	if (deviceid != 0x3D) { //No LIS3MDL detected ... return false (LIS3MDL_DEVICE_ID = 0X3d)
		printf("ERR> NO LIS3MDL DEVICE of 0x3D.\r\n");
   		return false;
	}
	printf("Found an LIS3MDL Sensor with DeviceID of 0x3D @reg0f\r\n");

	//Get Initial Register Contents
	for (i=0; i<0x3F; i++) {
		printf("Reg(0x%02x)=0x%02x\r\n",i, LIS3MDL_readRegister8(i));
	}

#if 0
	//Operating Mode Selection
	//Power Down
	if ( LIS3MDL_MAG_W_SystemOperatingMode( LIS3MDL_MAG_MD_POWER_DOWN ) == MEMS_ERROR )
	{
		printf("Err in Power Down State\r\n");
	    return MEMS_ERROR;
	}
	//Enable Block Data Update
	if ( LIS3MDL_MAG_W_BlockDataUpdate( LIS3MDL_MAG_BDU_ENABLE ) == MEMS_ERROR )
	{
		printf("Err in BlockDataUpdate\r\n");
	   return MEMS_ERROR;
	}

	//
	LIS3MDL_SetODR(80.0f); //set output data rate

	LIS3MDL_SetFS(4.0f); //set full scale

	//X and Y axes Operating mode selection
	if ( LIS3MDL_MAG_W_OperatingModeXY(LIS3MDL_MAG_OM_HIGH ) == MEMS_ERROR )
	{
		printf("Err in OperatingModeXY\r\n");
	    return MEMS_ERROR;
	}
#else
	//LIS3MDL_writeRegister8(LIS3MDL_MAG_CTRL_REG1, 0x70); //Ultra high performance mode for X and Y
	//LIS3MDL_writeRegister8(LIS3MDL_MAG_CTRL_REG4, 0x0C); //Ultra high performance mode for Z
	LIS3MDL_writeRegister8(LIS3MDL_MAG_CTRL_REG1, 0x00); //Low power mode for X and Y
	LIS3MDL_writeRegister8(LIS3MDL_MAG_CTRL_REG4, 0x00); //Low power mode for Z

	LIS3MDL_SetFS(4.0f); //set full scale//LIS3MDL_writeRegister8(LIS3MDL_MAG_CTRL_REG2, 0x00); //Full scale
	LIS3MDL_writeRegister8(LIS3MDL_MAG_CTRL_REG3, 0x00); //Cont conversion -- 	LIS3MDL_Enable();

#endif

	//Enable Temp
   	if ( LIS3MDL_MAG_W_TemperatureSensor_EnDis(LIS3MDL_MAG_TEMP_EN_ENABLE ) == MEMS_ERROR )
	{
   		printf("Err in TemperatureSensor_EnDis\r\n");
	    return MEMS_ERROR;
	}

	LIS3MDL_MAG_R_TemperatureSensorEnDis(&tempEnDis);
	if(tempEnDis = LIS3MDL_MAG_TEMP_EN_ENABLE ){
		printf("TempSensor Enabled\r\n");
	}else
		printf("TempSensor Disabled\r\n");

	return MEMS_SUCCESS;
}
void stmLIS3MDL_Loop(){
	int i;
	char str[10];
	float f_temp;
    int fifo_level=0;
    float f_xyz[3];

	printf("LIS3MDL MagnetoMeter driver test loop.\r\n");

	if(stmLIS3MDL_MAG_ConfIo_Init() == MEMS_ERROR){
		printf("ERRRRR\r\n");
		delayms(10000);
	}
	delayms(100);

	i=0;
	while(1){

        //if(g_INT_LIS3MDL)   	printf("INT_LIS3MDL\r\n");
        if(g_RDRY_LIS3MDL)      printf("DRDY_LIS3MDL\r\n");
//        if((g_INT_LIS3MDL) || (g_RDRY_LIS3MDL)  )//Int1
       	if(g_RDRY_LIS3MDL)//RDY
       	{
    		//stmLedToggle();

            //First get axis -- Do not read in sequence including temperature.
    		stmLIS3MDL_MAG_GetAxes(f_xyz);

    		printf("Mag[mGauss]:(x,y,z)=(");
    		printf("%s,",float2str(f_xyz[0]));
    		printf("%s,",float2str(f_xyz[1]));
    		printf("%s),",float2str(f_xyz[2]));

        	//Next Get temperature
    		f_temp = (float)LIS3MDL_MAG_readTemperature(); //Range = -40~+85 : 8 bit
    		//temperature = temperature/256.0 + 25.0;
    		f_temp = f_temp/256.0 + 25.0;
    		printf("T=%s.\r\n ",float2str(f_temp));
    		//Why it is always around 25 degree?

    		//delayms(300);
    	    //stmLedToggle();
        	//		printf("%02d:",fifo_level);
        			//LIS3MDL_read_XYZ();
        			//printf("X: %6d Y: %6d Z: %6d Temp: %6d, INT1: %08x\n",
        			//       acc_data.acc_x, acc_data.acc_y, acc_data.acc_z, f_temp, int1);
        			//--fifo_level;
        	//	}
        	//}
    		g_INT_LIS3MDL = 0;
    		g_RDRY_LIS3MDL = 0;
       		//int1 = LIS3MDL_readRegister8(LIS3MDL_REG_INT1SRC); //Clear INT1
        	//LIS3MDL_show4D(int1);
        	//printf("   INT1SRC = 0x%02x\r\n",int1);
        	//LIS3MDL_show4D(int1);
        }
        //delayms(100);

	}
};

