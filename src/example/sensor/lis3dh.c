/* LIS3DH : ultra-low-power high-performance 3-axis "nano" gravity accelerometer.
*  LIS3DH has 2 interrupts. Most interrupts are available only on the Int1 pin.
*  Interrupt pins are ACTIVE-HIGH and PP by default.
*/

#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))
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


#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz
#define LIS3DH_8BIT_I2C_ADDRESS  (0x18<<1)    // if SDO/SA0 is 3V, its 0x19
#define LIS3DH_I2C_AUTOINCREMENT 0x80

#define LIS3DH_REG_STATUS_AUX_07H       0x07
#define LIS3DH_REG_OUTADC1_L     0x08
#define LIS3DH_REG_OUTADC1_H     0x09
#define LIS3DH_REG_OUTADC2_L     0x0A
#define LIS3DH_REG_OUTADC2_H     0x0B
#define LIS3DH_REG_OUTADC3_L     0x0C
#define LIS3DH_REG_OUTADC3_H     0x0D
#define LIS3DH_REG_INTCOUNT      0x0E
#define LIS3DH_REG_WHOAMI        0x0F
#define LIS3DH_REG_TEMPCFG       0x1F
#define LIS3DH_REG_CTRL1         0x20
#define LIS3DH_REG_CTRL2         0x21
#define LIS3DH_REG_CTRL3         0x22
#define LIS3DH_REG_CTRL4         0x23
#define LIS3DH_REG_CTRL5         0x24
#define LIS3DH_REG_CTRL6         0x25
#define LIS3DH_REG_REFERENCE     0x26
#define LIS3DH_REG_STATUS_27H    0x27
#define LIS3DH_REG_OUT_X_L       0x28
#define LIS3DH_REG_OUT_X_H       0x29
#define LIS3DH_REG_OUT_Y_L       0x2A
#define LIS3DH_REG_OUT_Y_H       0x2B
#define LIS3DH_REG_OUT_Z_L       0x2C
#define LIS3DH_REG_OUT_Z_H       0x2D
#define LIS3DH_REG_FIFOCTRL      0x2E
#define LIS3DH_REG_FIFOSRC       0x2F
#define LIS3DH_REG_INT1CFG       0x30
#define LIS3DH_REG_INT1SRC       0x31
#define LIS3DH_REG_INT1THS       0x32
#define LIS3DH_REG_INT1DUR       0x33

#define LIS3DH_REG_INT2CFG       0x34

#define LIS3DH_REG_CLICKCFG      0x38
#define LIS3DH_REG_CLICKSRC      0x39
#define LIS3DH_REG_CLICKTHS      0x3A
#define LIS3DH_REG_TIMELIMIT     0x3B
#define LIS3DH_REG_TIMELATENCY   0x3C
#define LIS3DH_REG_TIMEWINDOW    0x3D
#define LIS3DH_REG_ACTTHS        0x3E
#define LIS3DH_REG_ACTDUR        0x3F

#define WATERMARK_LEVEL 16

typedef enum
{
  LIS3DH_RANGE_16_G         = 0b11,   // +/- 16g
  LIS3DH_RANGE_8_G           = 0b10,   // +/- 8g
  LIS3DH_RANGE_4_G           = 0b01,   // +/- 4g
  LIS3DH_RANGE_2_G           = 0b00    // +/- 2g (default value)
} lis3dh_range_t;

volatile lis3dh_range_t lis3dh_range;

typedef enum
{
  LIS3DH_AXIS_X         = 0x0,
  LIS3DH_AXIS_Y         = 0x1,
  LIS3DH_AXIS_Z         = 0x2,
} lis3dh_axis_t;


/* Used with register 0x2A (LIS3DH_REG_CTRL_REG1) to set bandwidth */
typedef enum
{
  LIS3DH_DATARATE_400_HZ     = 0b0111, //  400Hz
  LIS3DH_DATARATE_200_HZ     = 0b0110, //  200Hz
  LIS3DH_DATARATE_100_HZ     = 0b0101, //  100Hz
  LIS3DH_DATARATE_50_HZ      = 0b0100, //   50Hz
  LIS3DH_DATARATE_25_HZ      = 0b0011, //   25Hz
  LIS3DH_DATARATE_10_HZ      = 0b0010, // 10 Hz
  LIS3DH_DATARATE_1_HZ       = 0b0001, // 1 Hz
  LIS3DH_DATARATE_POWERDOWN  = 0,
  LIS3DH_DATARATE_LOWPOWER_1K6HZ  = 0b1000,
  LIS3DH_DATARATE_LOWPOWER_5KHZ  =  0b1001,

} lis3dh_dataRate_t;

/*
 * Bit offsets within the individual registers
 * source: LIS3DH datasheet
 */

/**
 * @name    TEMP_CFG_REG bitfield macros
 * @{
 */
/**
 * @brief   ADC enable
 *
 * Default value: 0
 *
 * 0: ADC disabled; 1: ADC enabled
 */
#define LIS3DH_TEMP_CFG_REG_ADC_PD_MASK          (1 << 7)
/**
 * @brief   Temperature sensor (T) enable.
 *
 * Default value: 0
 *
 * 0: T disabled; 1: T enabled
 */
#define LIS3DH_TEMP_CFG_REG_TEMP_EN_MASK         (1 << 6)
/** @} */ /* TEMP_CFG_REG bitfield macros */

/**
 * @name    CTRL_REG1 bitfield macros
 * @{
 */
/**
 * @brief    ODR global shift
 */
#define LIS3DH_CTRL_REG1_ODR_SHIFT               (4)
/**
 * @brief    ODR fourth bit mask
 */
#define LIS3DH_CTRL_REG1_ODR3_MASK               (1 << (LIS3DH_CTRL_REG1_ODR_SHIFT + 3))
/**
 * @brief    ODR third bit mask
 */
#define LIS3DH_CTRL_REG1_ODR2_MASK               (1 << (LIS3DH_CTRL_REG1_ODR_SHIFT + 2))
/**
 * @brief    ODR second bit mask
 */
#define LIS3DH_CTRL_REG1_ODR1_MASK               (1 << (LIS3DH_CTRL_REG1_ODR_SHIFT + 1))
/**
 * @brief   ODR first bit mask
 */
#define LIS3DH_CTRL_REG1_ODR0_MASK               (1 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   Output data rate (ODR) selection bitfield
 *
 * Default value: 0000
 *
 * 0000: Power down; Others: Refer to data sheet
 *
 * @see LIS3DH data sheet Table 25, “Data rate configuration”
 */
#define LIS3DH_CTRL_REG1_ODR_MASK                (LIS3DH_CTRL_REG1_ODR3_MASK | \
                                                  LIS3DH_CTRL_REG1_ODR2_MASK | \
                                                  LIS3DH_CTRL_REG1_ODR1_MASK | \
                                                  LIS3DH_CTRL_REG1_ODR0_MASK)
/**
 * @brief   Low power mode enable.
 *
 * Default value: 0
 *
 *  0. normal mode
 *  1. low power mode
 */
#define LIS3DH_CTRL_REG1_LPEN_MASK               (1 << 3)
/**
 * @brief   Z enable bit offset
 */
#define LIS3DH_CTRL_REG1_ZEN_SHIFT               (2)
/**
 * @brief   Z axis enable.
 *
 * Default value: 1
 *
 *  0. Z axis disabled
 *  1. Z axis enabled
 */
#define LIS3DH_CTRL_REG1_ZEN_MASK                (1 << LIS3DH_CTRL_REG1_ZEN_SHIFT)
/**
 * @brief   Y enable bit offset
 */
#define LIS3DH_CTRL_REG1_YEN_SHIFT               (1)
/**
 * @brief   Y axis enable.
 *
 * Default value: 1
 *
 *  0. Y axis disabled
 *  1. Y axis enabled
 */
#define LIS3DH_CTRL_REG1_YEN_MASK                (1 << LIS3DH_CTRL_REG1_YEN_SHIFT)
/**
 * @brief   X enable bit offset
 */
#define LIS3DH_CTRL_REG1_XEN_SHIFT               (0)
/**
 * @brief   X axis enable.
 *
 * Default value: 1
 *
 *  0. X axis disabled
 *  1. X axis enabled
 */
#define LIS3DH_CTRL_REG1_XEN_MASK                (1 << LIS3DH_CTRL_REG1_XEN_SHIFT)
/**
 * @brief   XYZ enable bitfield offset
 */
#define LIS3DH_CTRL_REG1_XYZEN_SHIFT             (0)
/**
 * @brief   X, Y, Z enable bitfield mask
 */
#define LIS3DH_CTRL_REG1_XYZEN_MASK              (LIS3DH_CTRL_REG1_XEN_MASK | \
                                                  LIS3DH_CTRL_REG1_YEN_MASK | LIS3DH_CTRL_REG1_ZEN_MASK)

/**
 * @brief    enable X axis (Use when calling lis3dh_set_axes())
 */
#define LIS3DH_AXES_X                            (LIS3DH_CTRL_REG1_XEN_MASK)
/**
 * @brief   enable Y axis (Use when calling lis3dh_set_axes())
 */
#define LIS3DH_AXES_Y                            (LIS3DH_CTRL_REG1_YEN_MASK)
/**
 * @brief   enable Z axis (Use when calling lis3dh_set_axes())
 */
#define LIS3DH_AXES_Z                            (LIS3DH_CTRL_REG1_ZEN_MASK)
/** @} */  /* CTRL_REG1 bitfield macros */

/**
 * @brief   Convenience macro for enabling all axes.
 */
#define LIS3DH_AXES_XYZ (LIS3DH_CTRL_REG1_XYZEN_MASK)

/**
 * @name    CTRL_REG2 bitfield macros
 * @{
 */
/**
 * @brief   High pass filter mode selection second bit
 *
 * Default value: 0
 *
 * @see Refer to Table 29, "High pass filter mode configuration"
 */
#define LIS3DH_CTRL_REG2_HPM1_MASK               (1 << 7)
/**
 * @brief   High pass filter mode selection first bit
 *
 * Default value: 0
 *
 * @see Refer to Table 29, "High pass filter mode configuration"
 */
#define LIS3DH_CTRL_REG2_HPM0_MASK               (1 << 6)
/**
 * @brief   High pass filter cut off frequency selection second bit
 */
#define LIS3DH_CTRL_REG2_HPCF2_MASK              (1 << 5)
/**
 * @brief   High pass filter cut off frequency selection second bit
 */
#define LIS3DH_CTRL_REG2_HPCF1_MASK              (1 << 4)
/**
 * @brief   Filtered data selection
 *
 * Default value: 0
 *
 *  0. internal filter bypassed
 *  1. data from internal filter sent to output register and FIFO
 */
#define LIS3DH_CTRL_REG2_FDS_MASK                (1 << 3)
/**
 * @brief   High pass filter enabled for CLICK function.
 *
 *  0. filter bypassed
 *  1. filter enabled
 */
#define LIS3DH_CTRL_REG2_HPCLICK_MASK            (1 << 2)
/**
 * @brief   High pass filter enabled for AOI function on interrupt 2, second bit
 *
 *  0. filter bypassed
 *  1. filter enabled
 */
#define LIS3DH_CTRL_REG2_HPIS2_MASK              (1 << 1)
/**
 * @brief   High pass filter enabled for AOI function on interrupt 2, first bit
 *
 *  0. filter bypassed
 *  1. filter enabled
 */
#define LIS3DH_CTRL_REG2_HPIS1_MASK              (1 << 0)
/** @} */ /* CTRL_REG2 bitfield macros */

/**
 * @name    CTRL_REG3 bitfield macros
 * @{
 */
/**
 * @brief   CLICK interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_CLICK_MASK           (1 << 7)
/**
 * @brief   AOI1 interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_AOI1_MASK            (1 << 6)
/**
 * @brief   AOI2 interrupt on INT1.
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_AOI2_MASK            (1 << 5)
/**
 * @brief   DRDY1 interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_DRDY1_MASK           (1 << 4)
/**
 * @brief   DRDY2 interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_DRDY2_MASK           (1 << 3)
/**
 * @brief   FIFO Watermark interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_WTM_MASK             (1 << 2)
/**
 * @brief   FIFO Overrun interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_OVERRUN_MASK         (1 << 1)
/** @} */ /* CTRL_REG3 bitfield macros */

/**
 * @name CTRL_REG4 bitfield macros
 * @{
 */
/**
 * @brief   Block data update (BDU) bit mask
 *
 * Default value of BDU: 0
 *
 *  0. continuous update
 *  1. output registers not updated until MSB and LSB reading
 */
#define LIS3DH_CTRL_REG4_BDU_MASK                (1 << 7)
/**
 * @brief   Block data update (BDU) enable
 */
#define LIS3DH_CTRL_REG4_BDU_ENABLE              (LIS3DH_CTRL_REG4_BDU_MASK)
/**
 * @brief    Block data update (BDU) disable
 */
#define LIS3DH_CTRL_REG4_BDU_DISABLE             (0)
/**
 * @brief   Big/little endian bit mask
 *
 * Default value of BLE: 0.
 *
 *  0. Data LSB @ lower address
 *  1. Data MSB @ lower address
 */
#define LIS3DH_CTRL_REG4_BLE_MASK                (1 << 6)
/**
 * @brief   Big/little endian little endian mode
 */
#define LIS3DH_CTRL_REG4_BLE_LITTLE_ENDIAN       (0)
/**
 * @brief   Big/little endian big endian mode
 */
#define LIS3DH_CTRL_REG4_BLE_BIG_ENDIAN          (LIS3DH_CTRL_REG4_BLE_MASK)
/**
 * @brief   Full scale selection mask second bit
 */
#define LIS3DH_CTRL_REG4_FS1_MASK                (1 << 5)
/**
 * @brief   Full scale selection mask first bit
 */
#define LIS3DH_CTRL_REG4_FS0_MASK                (1 << 4)
/**
 * @brief   Full scale selection mask
 */
#define LIS3DH_CTRL_REG4_FS_MASK                 (LIS3DH_CTRL_REG4_FS1_MASK | \
                                                  LIS3DH_CTRL_REG4_FS0_MASK)
/**
 * @brief   Scale register value: +/- 2G
 */
#define LIS3DH_CTRL_REG4_SCALE_2G                (0)
/**
 * @brief   Scale register value: +/- 4G
 */
#define LIS3DH_CTRL_REG4_SCALE_4G                (LIS3DH_CTRL_REG4_FS0_MASK)
/**
 * @brief   Scale register value: +/- 8G
 */
#define LIS3DH_CTRL_REG4_SCALE_8G                (LIS3DH_CTRL_REG4_FS1_MASK)
/**
 * @brief   Scale: +/- 16G
 */
#define LIS3DH_CTRL_REG4_SCALE_16G               (LIS3DH_CTRL_REG4_FS1_MASK | LIS3DH_CTRL_REG4_FS0_MASK)
/**
 * @brief   High resolution output mode
 *
 * Default value: 0
 *
 *  0. High resolution disable
 *  1. High resolution enable
 */
#define LIS3DH_CTRL_REG4_HR_MASK                 (1 << 3)
/**
 * @brief   Self test enable second bit mask
 *
 * Default value of self test: 00
 *
 *  - 00: Self test disabled
 *  - Other: See Table 34
 *
 * @see Table 34
 */
#define LIS3DH_CTRL_REG4_ST1_MASK                (1 << 2)
/**
 * @brief   Self test enable first bit mask
 */
#define LIS3DH_CTRL_REG4_ST0_MASK                (1 << 1)
/**
 * @brief   SPI serial interface mode selection
 *
 * Default value: 0
 *
 *  0. 4-wire interface
 *  1. 3-wire interface
 */
#define LIS3DH_CTRL_REG4_SIM_MASK                (1 << 0)
/**
 * @brief   Reboot memory content
 *
 * Default value: 0
 *
 *  0. normal mode
 *  1. reboot memory content
 */
#define LIS3DH_CTRL_REG5_REBOOT_MASK             (1 << 7)
/**
 * @brief   FIFO enable
 *
 * Default value: 0
 *
 *  0. FIFO disable
 *  1. FIFO enable
 */
#define LIS3DH_CTRL_REG5_FIFO_EN_MASK            (1 << 6)
/**
 * @brief   Latch interrupt request on INT1
 *
 * Latch interrupt request on INT1_SRC register, with INT1_SRC register
 * cleared by reading INT1_SRC itself.
 *
 * Default value: 0
 *
 *  0. interrupt request not latched
 *  1. interrupt request latched
 */
#define LIS3DH_CTRL_REG5_LIR_I1_MASK             (1 << 3)
/**
 * @brief   4D enable
 *
 * 4D detection is enabled on INT1 when 6D bit on INT1_CFG is set to 1.
 */
#define LIS3DH_CTRL_REG5_D4D_I1_MASK             (1 << 2)
/** @} */ /* CTRL_REG4 bitfield macros */

/**
 * @name    STATUS_REG bitfield macros
 * @{
 */
/**
 * @brief   X, Y or Z axis data overrun
 *
 * Default value: 0
 *
 *  0. no overrun has occurred
 *  1. a new set of data has overwritten the previous ones
 */
#define LIS3DH_STATUS_REG_ZYXOR_MASK             (1 << 7)
/**
 * @brief   Z axis data overrun
 *
 * Default value: 0
 *
 *  0. no overrun has occurred
 *  1. a new data for the Z-axis has overwritten the previous one
 */
#define LIS3DH_STATUS_REG_ZOR_MASK               (1 << 6)
/**
 * @brief   Y axis data overrun
 *
 * Default value: 0
 *
 *  0. no overrun has occurred
 *  1. a new data for the Y-axis has overwritten the previous one
 */
#define LIS3DH_STATUS_REG_YOR_MASK               (1 << 5)
/**
 * @brief   X axis data overrun
 *
 * Default value: 0
 *
 *  0. no overrun has occurred
 *  1. a new data for the X-axis has overwritten the previous one
 */
#define LIS3DH_STATUS_REG_XOR_MASK               (1 << 4)
/**
 * @brief   X, Y or Z axis new data available
 *
 * Default value: 0
 *
 *  0. a new set of data is not yet available
 *  1. a new set of data is available
 */
#define LIS3DH_STATUS_REG_ZYXDA_MASK             (1 << 3)
/**
 * @brief   Z axis new data available
 *
 * Default value: 0
 *
 *  0. a new data for the Z-axis is not yet available
 *  1. a new data for the Z-axis is available
 */
#define LIS3DH_STATUS_REG_ZDA_MASK               (1 << 2)
/**
 * @brief   Y axis new data available
 *
 * Default value: 0
 *
 *  0. a new data for the Y-axis is not yet available
 *  1. a new data for the Y-axis is available
 */
#define LIS3DH_STATUS_REG_YDA_MASK               (1 << 1)
/**
 * @brief   X axis new data available
 *
 * Default value: 0
 *
 *  0. a new data for the X-axis is not yet available
 *  1. a new data for the X-axis is available
 */
#define LIS3DH_STATUS_REG_XDA_MASK               (1 << 0)
/** @} */ /* STATUS_REG bitfield macros */

/**
 * @name    FIFO_CTRL_REG bitfield macros
 * @{
 */
#define LIS3DH_FIFO_CTRL_REG_FM_SHIFT            (6)
#define LIS3DH_FIFO_CTRL_REG_FM1_MASK            (1 << 7)
#define LIS3DH_FIFO_CTRL_REG_FM0_MASK            (1 << 6)
#define LIS3DH_FIFO_CTRL_REG_FM_MASK             (LIS3DH_FIFO_CTRL_REG_FM1_MASK | \
                                                  LIS3DH_FIFO_CTRL_REG_FM0_MASK)
#define LIS3DH_FIFO_CTRL_REG_TR_MASK             (1 << 5)
#define LIS3DH_FIFO_CTRL_REG_FTH4_MASK           (1 << 4)
#define LIS3DH_FIFO_CTRL_REG_FTH3_MASK           (1 << 3)
#define LIS3DH_FIFO_CTRL_REG_FTH2_MASK           (1 << 2)
#define LIS3DH_FIFO_CTRL_REG_FTH1_MASK           (1 << 1)
#define LIS3DH_FIFO_CTRL_REG_FTH0_MASK           (1 << 0)
#define LIS3DH_FIFO_CTRL_REG_FTH_SHIFT           (0)
#define LIS3DH_FIFO_CTRL_REG_FTH_MASK            (LIS3DH_FIFO_CTRL_REG_FTH0_MASK | \
                                                  LIS3DH_FIFO_CTRL_REG_FTH1_MASK | \
                                                  LIS3DH_FIFO_CTRL_REG_FTH2_MASK | \
                                                  LIS3DH_FIFO_CTRL_REG_FTH3_MASK | \
                                                  LIS3DH_FIFO_CTRL_REG_FTH4_MASK)
/** @} */ /* FIFO_CTRL_REG bitfield macros */

/**
 * @name    FIFO_SRC_REG bitfield macros
 * @{
 */
#define LIS3DH_FIFO_SRC_REG_WTM_MASK             (1 << 7)
#define LIS3DH_FIFO_SRC_REG_OVRN_FIFO_MASK       (1 << 6)
#define LIS3DH_FIFO_SRC_REG_EMPTY_MASK           (1 << 5)
#define LIS3DH_FIFO_SRC_REG_FSS4_MASK            (1 << 4)
#define LIS3DH_FIFO_SRC_REG_FSS3_MASK            (1 << 3)
#define LIS3DH_FIFO_SRC_REG_FSS2_MASK            (1 << 2)
#define LIS3DH_FIFO_SRC_REG_FSS1_MASK            (1 << 1)
#define LIS3DH_FIFO_SRC_REG_FSS0_MASK            (1 << 0)
#define LIS3DH_FIFO_SRC_REG_FSS_SHIFT            (0)
#define LIS3DH_FIFO_SRC_REG_FSS_MASK             (LIS3DH_FIFO_SRC_REG_FSS0_MASK | \
                                                  LIS3DH_FIFO_SRC_REG_FSS1_MASK | \
                                                  LIS3DH_FIFO_SRC_REG_FSS2_MASK | \
                                                  LIS3DH_FIFO_SRC_REG_FSS3_MASK | \
                                                  LIS3DH_FIFO_SRC_REG_FSS4_MASK)
/** @} */ /* FIFO_CTRL_REG bitfield macros */

/**
 * @name    Register address bitfield macros
 * @{
 */
/**
 * @brief   Write to register
 */
#define LIS3DH_SPI_WRITE_MASK                    (0 << 7)
/**
 * @brief   The READ bit must be set when reading
 */
#define LIS3DH_SPI_READ_MASK                     (1 << 7)
/**
 * @brief   Multi byte transfers must assert this bit when writing the address.
 */
#define LIS3DH_SPI_MULTI_MASK                    (1 << 6)
/**
 * @brief   Opposite of LIS3DH_SPI_MULTI_MASK.
 */
#define LIS3DH_SPI_SINGLE_MASK                   (0 << 6)
/**
 * @brief   Mask of the address bits in the address byte during transfers.
 */
#define LIS3DH_SPI_ADDRESS_MASK                  (0x3F)
/** @} */ /* Register address bitfield macros */

/**
 * @brief   Length of scalar measurement data in bytes.
 */
#define LIS3DH_ADC_DATA_SIZE                     (2U)

/**
 * @name    FIFO modes.
 *
 * Used when calling lis3dh_set_fifo()
 * @{
 */
/**
 * @brief   FIFO mode: Bypass
 */
#define LIS3DH_FIFO_MODE_BYPASS                  (0x00 << LIS3DH_FIFO_CTRL_REG_FM_SHIFT)
/**
 * @brief   FIFO mode: FIFO
 */
#define LIS3DH_FIFO_MODE_FIFO                    (0x01 << LIS3DH_FIFO_CTRL_REG_FM_SHIFT)
/**
 * @brief   FIFO mode: Stream
 */
#define LIS3DH_FIFO_MODE_STREAM                  (0x02 << LIS3DH_FIFO_CTRL_REG_FM_SHIFT)
/**
 * @brief   FIFO mode: Stream to FIFO
 */
#define LIS3DH_FIFO_MODE_STREAM_TO_FIFO          (0x03 << LIS3DH_FIFO_CTRL_REG_FM_SHIFT)
/** @} */

/**
 * @name    Output Data Rates (ODR) macros
 *
 * Use these when calling lis3dh_set_odr(odr).
 * @{
 */
/**
 * @brief    Powerdown mode
 */
#define LIS3DH_ODR_POWERDOWN                     (0x00 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   1Hz mode
 */
#define LIS3DH_ODR_1Hz                           (0x01 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   10Hz mode
 */
#define LIS3DH_ODR_10Hz                          (0x02 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   25Hz mode
 */
#define LIS3DH_ODR_25Hz                          (0x03 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   50Hz mode
 */
#define LIS3DH_ODR_50Hz                          (0x04 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   100Hz mode
 */
#define LIS3DH_ODR_100Hz                         (0x05 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   200Hz mode
 */
#define LIS3DH_ODR_200Hz                         (0x06 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   400Hz mode
 */
#define LIS3DH_ODR_400Hz                         (0x07 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   Low power 1600Hz mode
 */
#define LIS3DH_ODR_LP1600Hz                      (0x08 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   Normal mode 1250 Hz
 * @note    Normal mode 1250 Hz and Low power mode 5000 Hz share the same setting
 */
#define LIS3DH_ODR_NP1250Hz                      (0x09 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   Low power mode 5000 Hz
 * @note    Normal mode 1250 Hz and Low power mode 5000 Hz share the same setting
 */
#define LIS3DH_ODR_LP5000HZ                      (0x09 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/** @} */

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);

extern I2C_TypeDef *gI2Cx;

extern void delayms(uint32_t ms);
extern char *float2str(float x);

typedef struct{
  char name[10];
  unsigned short scale;
  unsigned short range;
  unsigned char version;
  unsigned char sensor_id;
  unsigned char type;
  unsigned char min_delay;
  unsigned char max_value;
  unsigned char min_value;
  unsigned char resolution;
}sensor_t;

int16_t x, y, z;
float x_g, y_g, z_g;
int32_t _sensorID;

sensor_t g_lis3dh;

lis3dh_range_t LIS3DH_getRange(void);

// Writes 8-bits to the specified destination register
void LIS3DH_writeRegister8(uint8_t reg, uint8_t value) {
	unsigned char regbuf[10];
   	int ret=0;
   	regbuf[0] = reg;
   	regbuf[1] = value;
    ret = stm_I2C_SendBurst(LIS3DH_8BIT_I2C_ADDRESS, regbuf, 2);
}

  /**************************************************************************/
  /*  Reads 8-bits from the specified register  */
  /**************************************************************************/
  uint8_t LIS3DH_readRegister8(uint8_t reg) {
    uint8_t value;
   	unsigned char regbuf[10];
   	int ret=0;

	stm_I2C_ReceiveBurstWithRestartCondition(LIS3DH_8BIT_I2C_ADDRESS, reg, &regbuf[0], 1);
    value = regbuf[0];
    return value;
  }
  /**
   * @brief Write (both set and clear) bits of an 8-bit register on the LIS3DH.
   *
   * @param[in]  addr         Register address on the LIS3DH.
   * @param[in]  mask         Bitmask for the bits to modify.
   * @param[in]  values       The values to write to the masked bits.
   *
   * @return                  0 on success
   * @return                  -1 on error
   */
static inline int LIS3DH_write_bits(const unsigned char reg, const unsigned char mask, const unsigned char values)
  {
      unsigned char tmp;

      if ((tmp = LIS3DH_readRegister8(reg)) < 0) {
          /* Communication error */
          return -1;
      }

      tmp &= ~mask;
      tmp |= (values & mask);

      LIS3DH_writeRegister8(reg, tmp);

      return 0;
  }

// read x y z at once
void LIS3DH_read_XYZ(void) {
	//unsigned char range;
	float divider;
	unsigned int mx,my,mz;
	unsigned char regbuf[10];

	stm_I2C_ReceiveBurstWithRestartCondition(LIS3DH_8BIT_I2C_ADDRESS, LIS3DH_REG_OUT_X_L | LIS3DH_I2C_AUTOINCREMENT, &regbuf[0], 6); //0x80 for autoincrement

	x=regbuf[0]; mx = regbuf[1]; x = mx <<8 + x;
	y=regbuf[2]; my = regbuf[3]; y = my <<8 + y;
	z=regbuf[4]; mz = regbuf[5]; z = mz <<8 + z;

	//g_lis3dh.range = LIS3DH_getRange();

	/* Scale to milli-G */
	if (g_lis3dh.range == LIS3DH_RANGE_16_G) divider = 1365.0; // different sensitivity at 16g
  	if (g_lis3dh.range == LIS3DH_RANGE_8_G) divider = 4096.0;
  	if (g_lis3dh.range == LIS3DH_RANGE_4_G) divider = 8190.0;
  	if (g_lis3dh.range == LIS3DH_RANGE_2_G) divider = 16380.0;

  	x_g = (float)x /divider;// x_g = (float)x / divider;
  	y_g = (float)y /divider;//y_g = (float)y / divider;
  	z_g = (float)z /divider; //z_g = (float)z / divider;
  	if((x==0) && (y==0) && (z==0)){
  		printf("\r\n");
  	}
  	else{
  		printf("{x,y,z}={%05d, %05d, %05d}",x,y,z);
  		printf("\tx_g,y_g,z_g ={%s,",float2str(x_g));
  		printf("%s,",float2str(y_g));
  		printf("%s}\r\n",float2str(z_g));
  	}
}

/**************************************************************************/
/*!
    @brief  Read the auxilary ADC
*/
/**************************************************************************/

int16_t LIS3DH_readADC(uint8_t adc) {
	  uint16_t value;
	  uint8_t reg;
	  unsigned char regbuf[10];

  if ((adc < 1) || (adc > 3)) return 0;

  adc--;

  //read all ADC registers(1,2,3) because of BDU enabled.
  reg = LIS3DH_REG_OUTADC1_L + adc*2;

  stm_I2C_ReceiveBurstWithRestartCondition(LIS3DH_8BIT_I2C_ADDRESS, reg | LIS3DH_I2C_AUTOINCREMENT, &regbuf[0], 6);// 2); //0x80 for autoincrement
  if(adc==0){
	  value = regbuf[0]; value = ((uint16_t)regbuf[1])<<8 + value;
  }
  else if(adc==1){
	  value = regbuf[2]; value = ((uint16_t)regbuf[3])<<8 + value;
  }else{
	  //Choose Temperature
	  value = ((unsigned short)regbuf[5])*256 + regbuf[4];
  }
  return value;
}
//Temperature with ADC channel 3.
//[See Table 5.]
//-40 ~ +85 degree
// 8 bit resolution
//
int16_t LIS3DH_readTempFromADC3() {
	int16_t value;
	uint8_t reg;
	unsigned char regbuf[10];

	stm_I2C_ReceiveBurstWithRestartCondition(LIS3DH_8BIT_I2C_ADDRESS, LIS3DH_REG_OUTADC3_L | LIS3DH_I2C_AUTOINCREMENT, &regbuf[0], 2);//0x80 for autoincrement
	value = (int16_t)((unsigned short)((regbuf[1] & 0x0F)*256) + regbuf[0]);
	//printf("ADC regbuf[L]=%02x [H]=%02x, val=%04x(%d)\r\n",regbuf[0],regbuf[1],value,value);
	return value;
}
/**************************************************************************/
/*!
    @brief  Set INT to output for single or double click
*/
/**************************************************************************/

void LIS3DH_setClick(uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow) {
  if (!c) {
    //disable int
    uint8_t r = LIS3DH_readRegister8(LIS3DH_REG_CTRL3);
    r &= ~(0x80); // turn off I1_CLICK
    LIS3DH_writeRegister8(LIS3DH_REG_CTRL3, r);
    LIS3DH_writeRegister8(LIS3DH_REG_CLICKCFG, 0);
    return;
  }
  // else...

  LIS3DH_writeRegister8(LIS3DH_REG_CTRL3, 0x80); // turn on int1 click
  LIS3DH_writeRegister8(LIS3DH_REG_CTRL5, 0x08); // latch interrupt on int1

  if (c == 1)
	  LIS3DH_writeRegister8(LIS3DH_REG_CLICKCFG, 0x15); // turn on all axes & singletap
  if (c == 2)
	  LIS3DH_writeRegister8(LIS3DH_REG_CLICKCFG, 0x2A); // turn on all axes & doubletap


  LIS3DH_writeRegister8(LIS3DH_REG_CLICKTHS, clickthresh); // arbitrary
  LIS3DH_writeRegister8(LIS3DH_REG_TIMELIMIT, timelimit); // arbitrary
  LIS3DH_writeRegister8(LIS3DH_REG_TIMELATENCY, timelatency); // arbitrary
  LIS3DH_writeRegister8(LIS3DH_REG_TIMEWINDOW, timewindow); // arbitrary
}

uint8_t LIS3DH_getClick(void) {
  return LIS3DH_readRegister8(LIS3DH_REG_CLICKSRC);
}


/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
    Default range is +/- 2G.
*/
/**************************************************************************/
void LIS3DH_setRange(lis3dh_range_t range)
{
  uint8_t r = LIS3DH_readRegister8(LIS3DH_REG_CTRL4);
  r &= ~(0x30);
  r |= range << 4;
  LIS3DH_writeRegister8(LIS3DH_REG_CTRL4, r);
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
lis3dh_range_t LIS3DH_getRange(void)
{
	lis3dh_range_t range;
	range = (lis3dh_range_t)((LIS3DH_readRegister8(LIS3DH_REG_CTRL4) >> 4) & 0x03);
	printf("LIS3DH> getRange = %u\r\n", range);
	return range;
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
void LIS3DH_setDataRate(lis3dh_dataRate_t dataRate)
{
  uint8_t ctl1 = LIS3DH_readRegister8(LIS3DH_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (dataRate << 4);
  LIS3DH_writeRegister8(LIS3DH_REG_CTRL1, ctl1);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
lis3dh_dataRate_t LIS3DH_getDataRate(void)
{
  return (lis3dh_dataRate_t)((LIS3DH_readRegister8(LIS3DH_REG_CTRL1) >> 4)& 0x0F);
}

int LIS3DH_setScale(const uint8_t scale)
{
    uint8_t scale_reg;
    /* Sensor full range is -32768 -- +32767 (measurements are left adjusted) */
    /*  => Scale factor is scale/32768 */
    switch (scale)
    {
        case 2:
            g_lis3dh.scale = 2000;
            scale_reg = LIS3DH_CTRL_REG4_SCALE_2G;
            break;
        case 4:
        	g_lis3dh.scale = 4000;
            scale_reg = LIS3DH_CTRL_REG4_SCALE_4G;
            break;
        case 8:
        	g_lis3dh.scale = 8000;
            scale_reg = LIS3DH_CTRL_REG4_SCALE_8G;
            break;
        case 16:
        	g_lis3dh.scale = 16000;
            scale_reg = LIS3DH_CTRL_REG4_SCALE_16G;
            break;
        default:
            return -1;
    }
    return LIS3DH_write_bits( LIS3DH_REG_CTRL4,
                             LIS3DH_CTRL_REG4_FS_MASK, scale_reg);
}
/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
/*
bool LIS3DH_getEvent(sensors_event_t *event) {
  // Clear the event
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;

  LIS3DH_read();

  event->acceleration.x = x_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = y_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = z_g * SENSORS_GRAVITY_STANDARD;

  return true;
}
*/
int LIS3DH_setFifo(const uint8_t mode, const uint8_t watermark)
{
    int status;
    uint8_t reg;
    reg = (watermark << LIS3DH_FIFO_CTRL_REG_FTH_SHIFT)
            & LIS3DH_FIFO_CTRL_REG_FTH_MASK;
    reg |= mode;
    LIS3DH_writeRegister8(LIS3DH_REG_FIFOCTRL, reg);
    if (mode != 0x00) {
        status = LIS3DH_write_bits( LIS3DH_REG_CTRL5,
            LIS3DH_CTRL_REG5_FIFO_EN_MASK, LIS3DH_CTRL_REG5_FIFO_EN_MASK);
    } else {
        status = LIS3DH_write_bits(LIS3DH_REG_CTRL5,
            LIS3DH_CTRL_REG5_FIFO_EN_MASK, 0);
    }
    return status;
}

int LIS3DH_setOutputDataRate(const uint8_t odr)
{
    return LIS3DH_write_bits(LIS3DH_REG_CTRL1, LIS3DH_CTRL_REG1_ODR_MASK, odr);
}

int LIS3DH_getFifoLevel()
{
    unsigned char reg;
    int level;

    reg = LIS3DH_readRegister8(LIS3DH_REG_FIFOSRC) ;
    level = (reg & LIS3DH_FIFO_SRC_REG_FSS_MASK) >> LIS3DH_FIFO_SRC_REG_FSS_SHIFT;
    return level;
}

int LIS3DH_setInt1(const uint8_t mode)
{
	LIS3DH_writeRegister8(LIS3DH_REG_CTRL3, mode);
    return 0;
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
bool LIS3DH_init() {
	unsigned char i;
	uint8_t deviceid;



	/* Check connection */
	deviceid = LIS3DH_readRegister8(LIS3DH_REG_WHOAMI);
	if (deviceid != 0x33) { //No LIS3DH detected ... return false
		printf("ERR> NO LIS3DH DEVICE.\r\n");
   		return false;
	}
	printf("Found an LIS3DH Sensor.\r\n");

	//Get Initial Register Contents
	//for (i=0; i<0x3F; i++) {
	//	printf("$%x=%x\r\n",i, LIS3DH_readRegister8(i));
	//}
    /* Clear all settings */
	LIS3DH_writeRegister8(LIS3DH_REG_CTRL1, LIS3DH_CTRL_REG1_XYZEN_MASK);    /* Clear all settings */

	LIS3DH_writeRegister8(LIS3DH_REG_CTRL3, 0x00);	/* Disable INT1 interrupt sources */
	LIS3DH_writeRegister8(LIS3DH_REG_CTRL5, 0);/* Disable FIFO */

	//Step-by-step Init
	//(1) enable all axes, in 400Hz
	//LIS3DH_writeRegister8(LIS3DH_REG_CTRL1, 0x07 | 0x70); //-> Bit3(LPen=0)|Zen|Yen|Xen -> ADC Resolution becomes 10bits.
															//LIS3DH_DATARATE_400_HZ; //ODR3~0: = 0111 (bit7~4)
	//LIS3DH_write_bits(LIS3DH_REG_CTRL1, LIS3DH_CTRL_REG1_XYZEN_MASK, LIS3DH_AXES_XYZ);
	LIS3DH_writeRegister8(LIS3DH_REG_CTRL1, 0x3F); //Low power with ODR=25Hz

	//(2)
	LIS3DH_writeRegister8(LIS3DH_REG_CTRL2, 0);/* Disable HP filter */
	//(3)
	LIS3DH_writeRegister8(LIS3DH_REG_CTRL3, LIS3DH_CTRL_REG3_I1_AOI1_MASK); //AOI1 Interrupt generation is routed to INT1 pin.
	//(4) High res & BDU enabled
	LIS3DH_writeRegister8(LIS3DH_REG_CTRL4, 0x80); //Block Data Update(bit7) | Low Resolution)
	//LIS3DH_writeRegister8(LIS3DH_REG_CTRL4, 0x88); //Block Data Update(bit7) | High Resolution) /* Set block data update and little endian, set Normal mode (LP=0, HighRate=1) */
    //(5)
	LIS3DH_writeRegister8(LIS3DH_REG_CTRL5, 0x08 | 0x40); // latch interrupt on int1 with D4D_INT1 bit enabled.
	                                                      // If AOI1 interrput, INT1 goes to 1 and stay high. Reading INT1_SRC(0x31) register will clear it.
	//(6)
	LIS3DH_writeRegister8(LIS3DH_REG_CTRL6, 0x00); 	// Disable INT2. default int is active high.
	//(7)
	//REFERENCE
	//(8)
    //LIS3DH_writeRegister8(LIS3DH_REG_INT1THS, 0x20); // Threshold = 32LSBs * 15.625mg/LSB = 500mg.
                                                     //This is the same as 30 degree of tilt(=asin(0.5)) cone zone around the vertical gravity vector.
    LIS3DH_writeRegister8(LIS3DH_REG_INT1THS, 0x30); // Threshold = 48LSBs * 15.625mg/LSB = 750mg.
    												 //This is the same as 48.6 degree of tilt(=asin(0.75)) cone zone around the vertical gravity vector.

    //LIS3DH_writeRegister8(LIS3DH_REG_INT1THS, 0x10); // Threshold = 16LSBs * 15.625mg/LSB = 250mg. => asin(0.25) = 14.48 degree. (? 90-14.48?)
    //LIS3DH_writeRegister8(LIS3DH_REG_INT1THS, 0x60); // Threshold = 60LSBs * 15.625mg/LSB = 937mg. => asin(0.937) = 69.6 degree. (? 90-69.6 = 20.4?)-- NOT WORKING...
    //(9)
    LIS3DH_writeRegister8(LIS3DH_REG_INT1DUR, 0x0A); // Duration = 10 LSBs *(1/25Hz) = 0.4s
    //(10)
    LIS3DH_writeRegister8(LIS3DH_REG_INT1CFG, 0x4F); // 6D Movement detection with Z axis disabled and YUPE, YDONE, XUPE, XDOWNE bits enabled.

	/* Set Output data rate*/
	//LIS3DH_setOutputDataRate(LIS3DH_ODR_200Hz); //the same register of LIS3DH_REG_CTRL1.
	//LIS3DH_setDataRate(LIS3DH_DATARATE_400_HZ); //ODR3~0: = 0111 (bit7~4)
	//LIS3DH_setDataRate(LIS3DH_DATARATE_100_HZ); //ODR3~0: = 0101 (bit7~4)

	/* Configure scale */
	//LIS3DH_setScale(g_lis3dh.scale);


    //puts("Set INT1 watermark function... ");
    //LIS3DH_setInt1(LIS3DH_CTRL_REG3_I1_WTM_MASK);


	// DRDY on INT1. ZYXDA= X,Y,Z new data available (Chosen)
	//               321DA = 1,2,3-axis new data available
	//LIS3DH_writeRegister8(LIS3DH_REG_CTRL3, 0x10); //CLICK(7) | IA1(6) | IA2(5) | ZYXDA(4) | 321DA(3) | WTM(2) | OVR(1) | NA(0)
	//LIS3DH_writeRegister8(LIS3DH_REG_CTRL3, 0x14 ); // | ZYXDA(4) |  WTM(2)
	//LIS3DH_writeRegister8(LIS3DH_REG_CTRL3, 0x10 | 0x80); //CLICK(7) | ZYXDA(4)

    //LIS3DH_writeRegister8(LIS3DH_REG_CLICKCFG, 0x15); // turn on all axes & singletap
    //LIS3DH_writeRegister8(LIS3DH_REG_CLICKCFG, 0x2A); // turn on all axes & doubletap
    //LIS3DH_writeRegister8(LIS3DH_REG_CLICKTHS,80); // clickthresh
	//LIS3DH_writeRegister8(LIS3DH_REG_TIMELIMIT,10);// timelimit); // arbitrary
	//LIS3DH_writeRegister8(LIS3DH_REG_TIMELATENCY, 20);//timelatency); // arbitrary
	//LIS3DH_writeRegister8(LIS3DH_REG_TIMEWINDOW, 255);//timewindow); // arbitrary

	//puts("Enable streaming FIFO mode... ");
    //LIS3DH_setFifo(LIS3DH_FIFO_MODE_STREAM, WATERMARK_LEVEL);

	// Turn on orientation config
	//LIS3DH_writeRegister8(LIS3DH_REG_PL_CFG, 0x40);



	//g_lis3dh.scale = 4;
	//LIS3DH_setRange(LIS3DH_RANGE_2_G);
	//g_lis3dh.range = LIS3DH_getRange();


	// enable adcs
	//LIS3DH_writeRegister8(LIS3DH_REG_TEMPCFG, 0x80);
	// enable temperature (adc3 to temp sensor)
	//LIS3DH_writeRegister8(LIS3DH_REG_TEMPCFG, 0x40);

    //enable adcs and temp
	LIS3DH_writeRegister8(LIS3DH_REG_TEMPCFG, 0xc0); //ADC_EN=1(bit7) | TEMP_EN (bit6) = 1.

	//Show Registers
	for (i=0; i<0x3F; i++) {
		printf("$0x%02x \t=0x%02x\r\n",i,LIS3DH_readRegister8(i));
	}
	printf("LIS3DH Init Done.\r\n");
	return true;
}

//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39 | STM32F103-M70
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| INT2_DH(Pin5)   |           |           |           | PB0           |PB0      		   | PB0
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//| INT1_DH(Pin6)   |           |           |*need remap| PB3*/GPIO     |PB1/TIM3_CH4      | PB1/GPIO
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------

/*
#if(PROCESSOR == PROCESSOR_STM32F103C8T6)
void stmLIS3DH_INT_1_2_pin_Setup(void){
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


//PB0_INT2pin (PIN 5)
//PB3_INT1pin (PIN 6):M35  ------------------------------------------------or PA6_INT1pin (NEW M38) (Pin4) --
#if(PROCESSOR == PROCESSOR_STM32F103C8T6)
void stmLIS3DH_INT_1_2_pin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;
#if (MCU_MODULE_VER == 35)
 	  //PB0-INT2 ==================================
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
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

	  /*
	   	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	  	  //PA6 - INT1
	  	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	  	  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
	  	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	  	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);

	  	  //Enable and set EXTI Line6 Interrupt to the lowest priority
	  	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	  	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;
	  	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  	  NVIC_Init(&NVIC_InitStructure);

	  	  // Configure EXTI Line6
	  	  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	  	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	  	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  	  EXTI_Init(&EXTI_InitStructure);
	  */
	  //PB3
	  //[VERY IMPORTANT] PB3's default function is JTDO. (and PB4's default function is JNTRST.)
	  	//Thus we should remap to GPIO as the follows.
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //PB3 Remap /*!< JTAG-DP Disabled and SW-DP Enabled */
   	  //-- [NOT USE] RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  	  //PB3 - INT1
  	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  	  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
  	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	  GPIO_Init(GPIOB, &GPIO_InitStructure);
  	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);

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
#else

#endif
}
#elif(PROCESSOR == PROCESSOR_STM32F103RCT6)
//PC15
void stmLIS3DH_INT_1_2_pin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

 	  //PC15 ==================================
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
/*
	  // Enable and set EXTI Line0 Interrupt to the lowest priority
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource15);
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;//Same Prio
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  // Configure EXTI Line0  //for INT2
	  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
*/
}
#endif
volatile int g_INT2_LIS3DH, g_INT1_LIS3DH;
#if (USE_EXTI0 == USE_EXTI0_LIS3DH)
//for PB0_INT2pin
void EXTI0_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		EXTI_ClearFlag(EXTI_Line0);//== EXTI_ClearITPendingBit(EXTI_Line0);		// Clear the EXTI line 0 pending bit
		g_INT2_LIS3DH = 1;
	}
}
#endif
#if (USE_EXTI3 == USE_EXTI3_LIS3DH)
//for PB3_INT1
void EXTI3_IRQHandler() //PB3
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET) {
		EXTI_ClearFlag(EXTI_Line3);//==EXTI_ClearITPendingBit(EXTI_Line3);		// Clear the EXTI line 3 pending bit
		g_INT1_LIS3DH = 1;
	}
}
#endif
/*
//for PA6_INT1//was PB3 -- Shared with APDS9960
void EXTI9_5_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
		EXTI_ClearFlag(EXTI_Line6);//==EXTI_ClearITPendingBit(EXTI_Line6);		// Clear the EXTI line 6 pending bit
		g_INT1_LIS3DH = 1;
	}
}
*/
//NO Z axis.
//We can show x,y tilts.
void LIS3DH_show4D(u8  int1_src){
	if(int1_src == 0x48) printf("YH");
	else if(int1_src == 0x44) printf("YL");
	else if(int1_src == 0x42) printf("XH");
	else if(int1_src == 0x41) printf("XL");
}

extern unsigned char g_bI2CModuleConfigDone;

void LIS3DH_BaseConfig(){
	g_INT2_LIS3DH = 0;
	g_INT1_LIS3DH = 0;

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

	//LED
	stmUser_LED_GPIO_setup(); //Accessory LED
	stmConfirmLEDBlink();

#if(PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
	stmLIS3DH_INT_1_2_pin_Setup();
#else
#endif
	delayms(100);
}

u8 stmLIS3DH_getInt1src(){
	u8	int1_src;
	int1_src = LIS3DH_readRegister8(LIS3DH_REG_INT1SRC); //Clear INT1
	return int1_src;
}

void stmLIS3DH_Loop(){
	int i;
	char str[10];
	float temperature;
	u8 int1;
    int fifo_level=0;

    LIS3DH_BaseConfig();

	printf("LIS3DH Gravity Accelerometer driver test loop.\r\n");

	LIS3DH_init();
	delayms(100);

	i=0;
	while(1){
        fifo_level=0;
//        if(g_INT1_LIS3DH == 1)   	printf("INT1_LIS3DH\r\n");
//        if(g_INT2_LIS3DH)       	printf("INT2_LIS3DH\r\n");
//        if(g_INT1_LIS3DH == 1){//Int1
//    		stmLedToggle();
        	//fifo_level = LIS3DH_getFifoLevel();
        	//printf("int1_count = %d\n", int1_count);

        	//if(fifo_level > 0){
        	//	printf("==================================\r\n");
        	//	printf("Reading %d measurements\r\n", fifo_level);
        	//	while (fifo_level > 0) {
        	//		printf("%02d:",fifo_level);
        			LIS3DH_read_XYZ();
        			//printf("X: %6d Y: %6d Z: %6d Temp: %6d, INT1: %08x\n",
        			//       acc_data.acc_x, acc_data.acc_y, acc_data.acc_z, temperature, int1);
        			//--fifo_level;
        	//	}
        		temperature = (float)LIS3DH_readTempFromADC3()/126.0; //Range = -40~+85 : 8 bit
        		printf("Temp = %s Degree.\r\n ",float2str(temperature));
        	//}
    		g_INT1_LIS3DH = 0;
    		g_INT2_LIS3DH = 0;
       		int1 = LIS3DH_readRegister8(LIS3DH_REG_INT1SRC); //Clear INT1
        	LIS3DH_show4D(int1);
        	printf("   INT1SRC = 0x%02x\r\n",int1);
        	LIS3DH_show4D(int1);
//        }

        	delayms(100);
	}
};
