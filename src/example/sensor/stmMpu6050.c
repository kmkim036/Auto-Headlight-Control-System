//Support Both MPU6050 and MPU9050 with same I2C Address

#include <string.h>
/*
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
#include "lwipopts.h"
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

#define MPU6050_8BIT_I2C_ADDRESS 0xD0// (0x68 << 1) //110-1000 -> 1101-0000 (0xD0)
//#define MPU6050_8BIT_I2C_ADDRESS (0x69 << 1)
#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz

// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

//Registers
#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22

#define MPU6050_RA_FIFO_EN          0x23

#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
//Sensor Data (14 bytes)
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
//
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74

#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0
#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1
#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

union accel_t_gyro_union
{
  struct
  {
    unsigned char x_accel_h;
    unsigned char x_accel_l;
    unsigned char y_accel_h;
    unsigned char y_accel_l;
    unsigned char z_accel_h;
    unsigned char z_accel_l;
    unsigned char t_h;
   unsigned char t_l;
   unsigned char x_gyro_h;
    unsigned char x_gyro_l;
    unsigned char y_gyro_h;
    unsigned char y_gyro_l;
    unsigned char z_gyro_h;
    unsigned char z_gyro_l;
 } reg;
  struct
  {
    short x_accel;
    short y_accel;
   short z_accel;
    short temperature;
    short x_gyro;
    short y_gyro;
    short z_gyro;
  } value;
};


u8 _buff[6];
u8 stm_i2ctxbuf[256];
u8 stm_i2crxbuf[256];

extern unsigned char g_bI2CModuleConfigDone;
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
//extern int I2CScan(u8 addr);

extern char *float2str(float x);
extern void delayms(uint32_t ms);

#define SWAP(x,y) swap = x; x = y; y = swap

//======================================
// MPU6050_write
// This is a common function to write multiple bytes to an I2C device.
// If only a single register is written, use the function MPU_6050_write_reg().
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
// If only a single register is written, a pointer to the data has to be used, and the size is a single byte:
int stmMPU6050_write(int reg, const unsigned char *pData, int size)
{
  int i, error;

  stm_i2ctxbuf[0] = reg;// write the reg address
  for(i=0;i<size;i++) // write data bytes
	  stm_i2ctxbuf[i+1] = *(pData+i);
  stm_I2C_SendBurst(MPU6050_8BIT_I2C_ADDRESS, stm_i2ctxbuf, size+1);
  return (0);         // return : no error
}

// MPU6050_write_reg: An extra function to write a byte on the specified register.
// It is just a wrapper around the MPU_6050_write() function, and it is only a convenient function to make it easier to write a single register.
int stmMPU6050_6DOF_writeSingleByte(int reg, unsigned char data)
{
	int error;
	error = stmMPU6050_write(reg, &data, 1);
	return (error);
}

// --------------------------------------------------------
// MPU6050_read

// This is a common function to read multiple bytes from an I2C device.
// It uses the boolean parameter for Wire.endTransMission() to be able to hold or release the I2C-bus.
// Only this function is used to read. There is no function for a single byte.

char stmMPU6050_6DOF_readRegs(int startReg, unsigned char *buffer, int size)
{
  stm_I2C_ReceiveBurstWithRestartCondition(MPU6050_8BIT_I2C_ADDRESS,startReg,buffer,size);
  return (0);  // return : no error
}

//=========================================================================
// XA_OFFS_* registers
short stmMPU6050_getXAccelOffset() {
	u8 buffer[2];
	stmMPU6050_6DOF_readRegs(MPU6050_RA_XA_OFFS_H, buffer,2);
	return (((short)buffer[0]) << 8) | buffer[1];
}
void stmMPU6050_setXAccelOffset(short sval) {
	u8 buffer[2];
	buffer[0] = sval <<8;
	buffer[1] = sval & 0xff;
	stmMPU6050_write(MPU6050_RA_XA_OFFS_H, buffer,2);
}
// YA_OFFS_* register
short stmMPU6050_getYAccelOffset() {
	u8 buffer[2];
	stmMPU6050_6DOF_readRegs(MPU6050_RA_YA_OFFS_H, buffer,2);
	return (((short)buffer[0]) << 8) | buffer[1];
}
void stmMPU605_setYAccelOffset(short sval) {
	u8 buffer[2];
	buffer[0] = sval <<8;
	buffer[1] = sval & 0xff;
	stmMPU6050_write(MPU6050_RA_YA_OFFS_H, buffer,2);
}
// ZA_OFFS_* register
short stmMPU6050_getZAccelOffset() {
	u8 buffer[2];
	stmMPU6050_6DOF_readRegs(MPU6050_RA_ZA_OFFS_H, buffer,2);
	return (((short)buffer[0]) << 8) | buffer[1];
}
void stmMPU6050_setZAccelOffset(short sval) {
	u8 buffer[2];
	buffer[0] = sval <<8;
	buffer[1] = sval & 0xff;
	stmMPU6050_write(MPU6050_RA_ZA_OFFS_H, buffer,2);
}

// XG_OFFS_USR* registers
short stmMPU6050_getXGyroOffset() {
	u8 buffer[2];
	stmMPU6050_6DOF_readRegs(MPU6050_RA_XG_OFFS_USRH, buffer,2);
	return (((short)buffer[0]) << 8) | buffer[1];
}
void stmMPU6050_setXGyroOffset(short sval) {
	u8 buffer[2];
	buffer[0] = sval <<8;
	buffer[1] = sval & 0xff;
	stmMPU6050_write(MPU6050_RA_XG_OFFS_USRH, buffer,2);
}

// YG_OFFS_USR* register
short stmMPU6050_getYGyroOffset() {
	u8 buffer[2];
	stmMPU6050_6DOF_readRegs(MPU6050_RA_YG_OFFS_USRH, buffer,2);
	return (((short)buffer[0]) << 8) | buffer[1];
}
void stmMPU6050_setYGyroOffset(short sval) {
	u8 buffer[2];
	buffer[0] = sval <<8;
	buffer[1] = sval & 0xff;
	stmMPU6050_write(MPU6050_RA_YG_OFFS_USRH, buffer,2);
}
// ZG_OFFS_USR* register
short stmMPU6050_getZGyroOffset() {
	u8 buffer[2];
	stmMPU6050_6DOF_readRegs(MPU6050_RA_ZG_OFFS_USRH, buffer,2);
	return (((short)buffer[0]) << 8) | buffer[1];
}
void stmMPU6050_setZGyroOffset(short sval) {
	u8 buffer[2];
	buffer[0] = sval <<8;
	buffer[1] = sval & 0xff;
	stmMPU6050_write(MPU6050_RA_ZG_OFFS_USRH, buffer,2);
}


void stmMPU6050_6DOF_Init()
{
  char error;
  unsigned char c;

  printf("MPU6050 Init.\r\n");

  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  error = stmMPU6050_6DOF_readRegs (MPU6050_RA_WHO_AM_I, &c, 1);
  printf("WHO_AM_I : %02x (Should be 0x68), error=%d\r\n",c,error);
  // the 'sleep' bit  should read a '1'. That bit has to be cleared, since the sensor is in sleep mode at power-up.
  error = stmMPU6050_6DOF_readRegs (MPU6050_RA_PWR_MGMT_1, &c, 1);
  printf("PWR_MGMT_1 : %02x (error=%d)\r\n",c,error);
  // Clear the 'sleep' bit to start the sensor.
  stmMPU6050_6DOF_writeSingleByte (MPU6050_RA_PWR_MGMT_1, 0);

  delayms(100);
  /*
  // use the code below to change accel/gyro offset values
  printf("Updating internal sensor offsets...\r\n");
  // -76	-2359	1688	0	0	0
  printf("%d ", MPU6050_getXAccelOffset());// -76
  printf("%d ", MPU6050_getYAccelOffset());// -2359
  printf("%d ", MPU6050_getZAccelOffset());// 1688
  printf("%d ", MPU6050_getXGyroOffset()); //0
  printf("%d ", MPU6050_getYGyroOffset()); //0
  printf("%d\r\n", MPU6050_getZGyroOffset()); //0

  MPU6050_setXGyroOffset(220);
  MPU6050_setYGyroOffset(76);
  MPU6050_setZGyroOffset(-85);

  printf("%d ", MPU6050_getXAccelOffset());// -76
  printf("%d ", MPU6050_getYAccelOffset());// -2359
  printf("%d ", MPU6050_getZAccelOffset());// 1688
  printf("%d ", MPU6050_getXGyroOffset()); //0
  printf("%d ", MPU6050_getYGyroOffset()); //0
  printf("%d\r\n", MPU6050_getZGyroOffset()); //0
*/
}

void stmMPU6050_6DOF_GetData(short acc[], short gyro[], float *degree)
{
	  float f_T;
	  unsigned char swap;
	  union accel_t_gyro_union accel_t_gyro;
	  char err;
	  // Read the raw values: Read 14 bytes at once,  containing acceleration, temperature and gyro.
	  // With the default settings of the MPU-6050, there is no filter enabled, and the values are very unstable.
	  err = stmMPU6050_6DOF_readRegs (MPU6050_RA_ACCEL_XOUT_H, (unsigned char *) &accel_t_gyro, sizeof(accel_t_gyro));

	  // Swap all high and low bytes.
	  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
	  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
	  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
	  SWAP (accel_t_gyro.reg.t_h,       accel_t_gyro.reg.t_l); //temperature
	  SWAP (accel_t_gyro.reg.x_gyro_h,  accel_t_gyro.reg.x_gyro_l);
	  SWAP (accel_t_gyro.reg.y_gyro_h,  accel_t_gyro.reg.y_gyro_l);
	  SWAP (accel_t_gyro.reg.z_gyro_h,  accel_t_gyro.reg.z_gyro_l);


	  // The temperature sensor is -40 to +85 degrees Celsius. It is a signed integer.
	  //340 per degrees Celsius. -512 at 35 degrees. At 0 degrees: -512 - (340 * 35) = -12412
	  //dT = (int)( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
	  f_T = (float)(accel_t_gyro.value.temperature + 12412)/340.0;
	  //printf("[Temp]=%s Degree\r\n",float2str(f_T));

	  //return values
	  memcpy(acc, &accel_t_gyro.value.x_accel,6);
	  memcpy(gyro, &accel_t_gyro.value.x_gyro,6);
	  *degree = f_T;
}

void stmMPU6050Loop()
{
	  int error;
	  short acc[3]; short gyro[3]; float f_degree;

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

	printf("MPU6050ADDR Driver Test with I2C");

    stmMPU6050_6DOF_Init();


	while(1){
		stmMPU6050_6DOF_GetData(acc, gyro, &f_degree);

		printf("[accel] x,y,z=%d,\t%d,\t%d ", acc[0], acc[1],acc[2]);
		printf("\t[Gyro] x,y,z =%d \t%d \t%d ", gyro[0],gyro[1],gyro[2]);
		printf("\t[Temp]=%s Degree\r\n",float2str(f_degree));

		delayms(100);
	}
}


