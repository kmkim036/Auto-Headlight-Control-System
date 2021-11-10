/*
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
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
#include "ethopts.h"
#include "yInc.h"
#include <stdint.h>

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

//+-----------------+-----------+-----------+-----------+---------+---------+-----
//|                 |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   |103
//+-----------------+-----------+-----------+-----------+---------+---------+-----
//| ULED            | PB14      |PC4        |PE15       | <==     | PG7
//+-----------------+-----------+-----------+-----------+---------+---------+
//| BUTTON          |           |PC5(H)     |PD11(index)| PD11(L) |
//+-----------------+-----------+-----------+-----------+---------+---------+------
//| AP3216/APSD9960 |           |PA8        | PB8       | PE14(F) |         | PA8
//+-----------------+-----------+-----------+-----------+---------+---------+------
//| VL53L0x         |           |           |           | PC6 (F) |         | PC13
//+-----------------+-----------+-----------+-----------+---------+---------+------
//| BEEP            |           |PB13       |PD14       | <==     |
//+-----------------+-----------+-----------+-----------+---------+
//| QEI             |           |PB0,1,12   |PD12,13,11 | PD12,13 |
//+-----------------+-----------+-----------+-----------+---------+
//*H= Active High/L=Active Low
//*F= FallingEdge

/* APDS-9960 I2C address */
#define APDS9960_I2C_ADDR7       0x39 //0011 1001
#define APDS9960_I2C_ADDR8       0x72 //0111 0010
extern I2C_TypeDef *gI2Cx;
extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c

extern void delayms(uint32_t ms);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern int stm_I2C_StartAndAddr(uint8_t address8bit, uint8_t direction);
extern int stm_I2C_SendBurst(unsigned char slave_addr8, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);


/* Raw I2C Commands */
//bool wireWriteByte(uint8_t val);
//bool wireWriteDataByte(uint8_t reg, uint8_t val);
//bool wireWriteDataBlock(uint8_t reg, uint8_t *val, unsigned int len);
//bool wireReadRegDataByte(uint8_t reg, uint8_t &val);
//int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len);

/*
 * @file    SparkFun_APDS-9960.h
 * @brief   Library for the SparkFun APDS-9960 breakout board
 * @author  Shawn Hymel (SparkFun Electronics)
 *
 * @copyright This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 *
 * This library interfaces the Avago APDS-9960 to Arduino over I2C. The library
 * relies on the Arduino Wire (I2C) library. to use the library, instantiate an
 * APDS9960 object, call init(), and call the appropriate functions.
 */

#define APDS9960_NO_INTERRUPT 0


/* Gesture parameters */
#define GESTURE_THRESHOLD_OUT   10
#define GESTURE_SENSITIVITY_1   50
#define GESTURE_SENSITIVITY_2   20

/* Error code for returned values */
#define ERROR                   0xFF

/* Acceptable device IDs */
#define APDS9960_ID_REG92_1           0xAB
#define APDS9960_ID_REG92_2           0x9C

/* Misc parameters */
#define FIFO_PAUSE_TIME         30      // Wait period (ms) between FIFO reads

/* APDS-9960 register addresses */
#define APDS9960_ENABLE_REG80         0x80
#define APDS9960_ATIME          0x81
#define APDS9960_WTIME          0x83
#define APDS9960_AILTL          0x84
#define APDS9960_AILTH          0x85
#define APDS9960_AIHTL          0x86
#define APDS9960_AIHTH          0x87
#define APDS9960_PILT           0x89
#define APDS9960_PIHT           0x8B
#define APDS9960_PERS           0x8C
#define APDS9960_CONFIG1_REG8D        0x8D
#define APDS9960_PPULSE         0x8E
#define APDS9960_CONTROL_REG8F        0x8F
#define APDS9960_CONFIG2_REG90        0x90
#define APDS9960_ID_REG92             0x92
#define APDS9960_STATUS_REG93         0x93
#define APDS9960_CDATAL         0x94
#define APDS9960_CDATAH         0x95
#define APDS9960_RDATAL         0x96
#define APDS9960_RDATAH         0x97
#define APDS9960_GDATAL         0x98
#define APDS9960_GDATAH         0x99
#define APDS9960_BDATAL         0x9A
#define APDS9960_BDATAH         0x9B
#define APDS9960_PROXIMITY_DATA_REG9C          0x9C
#define APDS9960_PROXIMITY_OFFSET_UP_RIGHT_REG9D     0x9D
#define APDS9960_PROXIMITY_OFFSET_DOWN_LEFT_REG9E     0x9E
#define APDS9960_CONFIG3_FOR_PROXIMITY_REG9F        0x9F
#define APDS9960_PROXIMITY_ENTER_THRESHOLD_REGA0         0xA0
#define APDS9960_GEXTH          0xA1
#define APDS9960_GCONF1         0xA2
#define APDS9960_GCONF2         0xA3
#define APDS9960_GOFFSET_U      0xA4
#define APDS9960_GOFFSET_D      0xA5
#define APDS9960_GOFFSET_L      0xA7
#define APDS9960_GOFFSET_R      0xA9
#define APDS9960_GPULSE         0xA6
#define APDS9960_GCONF3         0xAA
#define APDS9960_GCONF4         0xAB
#define APDS9960_GFLVL          0xAE
#define APDS9960_GSTATUS        0xAF
#define APDS9960_IFORCE         0xE4
#define APDS9960_PICLEAR        0xE5
#define APDS9960_CICLEAR        0xE6
#define APDS9960_AICLEAR        0xE7
#define APDS9960_GFIFO_U        0xFC
#define APDS9960_GFIFO_D        0xFD
#define APDS9960_GFIFO_L        0xFE
#define APDS9960_GFIFO_R        0xFF

/* Bit fields */
#define APDS9960_PON            0b00000001
#define APDS9960_AEN            0b00000010
#define APDS9960_PEN            0b00000100
#define APDS9960_WEN            0b00001000
#define APSD9960_AIEN           0b00010000
#define APDS9960_PIEN           0b00100000
#define APDS9960_GEN            0b01000000
#define APDS9960_GVALID         0b00000001

/* On/Off definitions */
#define OFF                     0
#define ON                      1

/* Acceptable parameters for setMode */
#define POWER                   0
#define AMBIENT_LIGHT           1
#define PROXIMITY               2
#define WAIT                    3
#define AMBIENT_LIGHT_INT       4
#define PROXIMITY_INT           5
#define GESTURE                 6
#define ALL                     7

/* LED Drive values */
#define LED_DRIVE_100MA         0
#define LED_DRIVE_50MA          1
#define LED_DRIVE_25MA          2
#define LED_DRIVE_12_5MA        3

/* Proximity Gain (PGAIN) values */
#define PGAIN_1X                0
#define PGAIN_2X                1
#define PGAIN_4X                2
#define PGAIN_8X                3

/* ALS Gain (AGAIN) values */
#define AGAIN_1X                0
#define AGAIN_4X                1
#define AGAIN_16X               2
#define AGAIN_64X               3

/* Gesture Gain (GGAIN) values */
#define GGAIN_1X                0
#define GGAIN_2X                1
#define GGAIN_4X                2
#define GGAIN_8X                3

/* LED Boost values */
#define LED_BOOST_100           0
#define LED_BOOST_150           1
#define LED_BOOST_200           2
#define LED_BOOST_300           3

/* Gesture wait time values */
#define GWTIME_0MS              0
#define GWTIME_2_8MS            1
#define GWTIME_5_6MS            2
#define GWTIME_8_4MS            3
#define GWTIME_14_0MS           4
#define GWTIME_22_4MS           5
#define GWTIME_30_8MS           6
#define GWTIME_39_2MS           7

/* Default values */
#define DEFAULT_ATIME           219     // 103ms
#define DEFAULT_WTIME           246     // 27ms
#define DEFAULT_PROX_PPULSE     0x87    // 16us, 8 pulses
#define DEFAULT_GESTURE_PPULSE  0x89    // 16us, 10 pulses
#define DEFAULT_POFFSET_UR      0       // 0 offset
#define DEFAULT_POFFSET_DL      0       // 0 offset
#define DEFAULT_CONFIG1         0x60    // No 12x wait (WTIME) factor
#define DEFAULT_LDRIVE          LED_DRIVE_100MA
#define DEFAULT_PGAIN           PGAIN_4X
#define DEFAULT_AGAIN           AGAIN_4X
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2         0x01    // No saturation interrupts or LED boost
#define DEFAULT_CONFIG3         0       // Enable all photodiodes, no SAI
#define DEFAULT_GPENTH          40      // Threshold for entering gesture mode
#define DEFAULT_GEXTH           30      // Threshold for exiting gesture mode
#define DEFAULT_GCONF1          0x40    // 4 gesture events for int., 1 for exit
#define DEFAULT_GGAIN           GGAIN_4X
#define DEFAULT_GLDRIVE         LED_DRIVE_100MA
#define DEFAULT_GWTIME          GWTIME_2_8MS
#define DEFAULT_GOFFSET         0       // No offset scaling for gesture mode
#define DEFAULT_GPULSE          0xC9    // 32us, 10 pulses
#define DEFAULT_GCONF3          0       // All photodiodes active during gesture
#define DEFAULT_GIEN            0       // Disable gesture interrupts

/* Direction definitions */
enum eAPDSDIR{
  DIR_NONE,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_UP,
  DIR_DOWN,
  DIR_NEAR,
  DIR_FAR,
  DIR_ALL
};

/* State definitions */
enum eAPDSSTATE{
  NA_STATE,
  NEAR_STATE,
  FAR_STATE,
  ALL_STATE
};

/* Container for gesture data */
typedef struct gesture_data_type {
    uint8_t u_data[32];
    uint8_t d_data[32];
    uint8_t l_data[32];
    uint8_t r_data[32];
    uint8_t index;
    uint8_t total_gestures;
    uint8_t in_threshold;
    uint8_t out_threshold;
} gesture_data_type;

volatile uint8_t g_irq=0;

struct _APDSmodule {
    gesture_data_type gesture_data_;
    int gesture_ud_delta_;
    int gesture_lr_delta_;
    int gesture_ud_count_;
    int gesture_lr_count_;
    int gesture_near_count_;
    int gesture_far_count_;
    int gesture_state_;
    int gesture_motion_;

    //sensor values
    uint8_t proximity_data;
    //uint8_t irq; //volatile
} APDSmodule;

//bool APDS9960_setProximityIntEnable(uint8_t enable);
//bool APDS9960_setAmbientLightIntEnable(uint8_t enable);
//bool APDS9960_setAmbientLightGain(uint8_t drive);

    bool APDS9960_enableLightSensor(bool interrupts);// = false);
    bool APDS9960_disableLightSensor();
    bool APDS9960_enableProximitySensor(bool interrupts);// = false);
    bool APDS9960_disableProximitySensor();
    bool APDS9960_enableGestureSensor(bool interrupts);// = true);
    bool APDS9960_disableGestureSensor();

    /* LED drive strength control */
    uint8_t APDS9960_getLEDDrive();
    bool APDS9960_setLEDDrive(uint8_t drive);
    uint8_t APDS9960_getGestureLEDDrive();
    bool APDS9960_setGestureLEDDrive(uint8_t drive);

    /* Gain control */
    uint8_t APDS9960_getAmbientLightGain();
    bool APDS9960_setAmbientLightGain(uint8_t gain);
    uint8_t APDS9960_getProximityGain();
    bool APDS9960_setProximityGain(uint8_t gain);
    uint8_t APDS9960_getGestureGain();
    bool APDS9960_setGestureGain(uint8_t gain);

    // Get and set light interrupt thresholds
    bool APDS9960_getLightIntLowThreshold(unsigned short *threshold);
    bool APDS9960_setLightIntLowThreshold(unsigned short threshold);
    bool APDS9960_getLightIntHighThreshold(unsigned short *threshold);
    bool APDS9960_setLightIntHighThreshold(unsigned short threshold);

    // Get and set proximity interrupt thresholds
    bool APDS9960_getProximityIntLowThreshold(uint8_t *threshold);
    bool APDS9960_setProximityIntLowThreshold(uint8_t threshold);
    bool APDS9960_getProximityIntHighThreshold(uint8_t *threshold);
    bool APDS9960_setProximityIntHighThreshold(uint8_t threshold);

    // Get and set interrupt enables
    uint8_t APDS9960_getAmbientLightIntEnable();
    bool APDS9960_setAmbientLightIntEnable(uint8_t enable);
    uint8_t APDS9960_getProximityIntEnable();
    bool APDS9960_setProximityIntEnable(uint8_t enable);
    uint8_t APDS9960_getGestureIntEnable();
    bool APDS9960_setGestureIntEnable(uint8_t enable);

    /* Clear interrupts */
    bool APDS9960_clearAmbientLightInt();
    bool APDS9960_clearProximityInt();

    /* Ambient light methods */
    bool APDS9960_readAmbientLight(unsigned short *retVal);
    bool APDS9960_readRedLight(unsigned short *retVal);
    bool APDS9960_readGreenLight(unsigned short *retVal);
    bool APDS9960_readBlueLight(unsigned short *retVal);

    /* Proximity methods */
    bool APDS9960_readProximity(uint8_t *retVal);

    /* Gesture methods */
    bool APDS9960_isGestureAvailable();
    int APDS9960_readGesture();

    /* Gesture processing */
    void APDS9960_resetGestureParameters();
    bool APDS9960_processGestureData();
    bool APDS9960_decodeGesture();

    /* Proximity Interrupt Threshold */
    uint8_t APDS9960_getProxIntLowThresh();
    bool APDS9960_setProxIntLowThresh(uint8_t threshold);
    uint8_t APDS9960_getProxIntHighThresh();
    bool APDS9960_setProxIntHighThresh(uint8_t threshold);

    /* LED Boost Control */
    uint8_t APDS9960_getLEDBoost();
    bool APDS9960_setLEDBoost(uint8_t boost);

    /* Proximity photodiode select */
    uint8_t APDS9960_getProxGainCompEnable();
    bool APDS9960_setProxGainCompEnable(uint8_t enable);
    uint8_t APDS9960_getProxPhotoMask();
    bool APDS9960_setProxPhotoMask(uint8_t mask);

    /* Gesture threshold control */
    uint8_t APDS9960_getGestureEnterThresh();
    bool APDS9960_setGestureEnterThresh(uint8_t threshold);
    uint8_t APDS9960_getGestureExitThresh();
    bool APDS9960_setGestureExitThresh(uint8_t threshold);

    /* Gesture LED, gain, and time control */
    uint8_t APDS9960_getGestureWaitTime();
    bool APDS9960_setGestureWaitTime(uint8_t time);

    /* Gesture mode */
    uint8_t APDS9960_getGestureMode();
    bool APDS9960_setGestureMode(uint8_t mode);

    //Power
    bool APDS9960_enablePower();


    extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
    extern int stm_I2C_StartAndAddr(uint8_t address8bit, uint8_t direction);
    extern int stm_I2C_SendBurst(unsigned char slave_addr8, unsigned char *burst, unsigned char datalen);
    extern unsigned char I2C_yReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
    extern unsigned char g_bI2CModuleConfigDone;
    extern I2C_TypeDef *gI2Cx;
    /* Raw I2C Commands */
//bool wireWriteDataByte(uint8_t reg, uint8_t reg, uint8_t val);
//bool wireWriteDataBlock(uint8_t reg, uint8_t *val, unsigned int len);
//bool wireReadRegDataByte(uint8_t reg, uint8_t &val);
//int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len);

bool wireWriteDataByte(uint8_t reg,uint8_t val)
{
	stmI2cSendbuf[0] = reg;
	stmI2cSendbuf[1] = val;
	stm_I2C_SendBurst(APDS9960_I2C_ADDR8 ,stmI2cSendbuf, 2); //2= including reg
	return 1;
}
bool wireWriteDataBlock(uint8_t reg, uint8_t *val, unsigned int len){
	stmI2cSendbuf[0] = reg;
	memcpy(&stmI2cSendbuf[1],val,len);
	stm_I2C_SendBurst(APDS9960_I2C_ADDR8 ,val, len+1);
	return 1;
}
bool wireReadRegDataByte(uint8_t reg, uint8_t *retVal){
	stm_I2C_ReceiveBurstWithRestartCondition(APDS9960_I2C_ADDR8, reg, retVal, 1);
	return 1;
}
int wireReadDataBlock(uint8_t reg, uint8_t *retVal, unsigned int len){
	stm_I2C_ReceiveBurstWithRestartCondition(APDS9960_I2C_ADDR8, reg, retVal, len);
	return len;
}
/*
bool APDS9960_wireWriteByte(uint8_t val)
{
    Wire.beginTransmission(APDS9960_I2C_ADDR8);
    Wire.write(val);
    if( Wire.endTransmission() != 0 ) {
        return false;
    }

    return true;
}
*/

/**
 * @brief Reads a block (array) of bytes from the I2C device and register
 *
 * @param[in] reg the register to read from
 * @param[out] val pointer to the beginning of the data
 * @param[in] len number of bytes to read
 * @return Number of bytes read. -1 on read error.
 */

// * Public methods for controlling the APDS-9960
//Reads and returns the contents of the ENABLE register
//@return Contents of the ENABLE register. 0xFF if error.

uint8_t APDS9960_getMode()
{
    uint8_t enable_value;

    // Read current ENABLE register
    if( !wireReadRegDataByte(APDS9960_ENABLE_REG80, &enable_value) ) {
        return ERROR;
    }
    return enable_value;
}

/**
 * @brief Enables or disables a feature in the APDS-9960
 *
 * @param[in] mode which feature to enable
 * @param[in] enable ON (1) or OFF (0)
 * @return True if operation success. False otherwise.
 */
bool APDS9960_setMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    /* Read current ENABLE register */
    reg_val = APDS9960_getMode();
    if( reg_val == ERROR ) {
        return false;
    }

    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if( mode >= 0 && mode <= 6 ) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }

    /* Write value back to ENABLE register */
    if( !wireWriteDataByte(APDS9960_ENABLE_REG80, reg_val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Starts the light (R/G/B/Ambient) sensor on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware interrupt on high or low light
 * @return True if sensor enabled correctly. False on error.
 */
bool APDS9960_enableLightSensor(bool interrupts)
{

    /* Set default gain, interrupts, enable power, and enable sensor */
    if( !APDS9960_setAmbientLightGain(DEFAULT_AGAIN) ) {
        return false;
    }
    if( interrupts ) {
        if( !APDS9960_setAmbientLightIntEnable(1) ) {
            return false;
        }
    } else {
        if( !APDS9960_setAmbientLightIntEnable(0) ) {
            return false;
        }
    }
    if( !APDS9960_enablePower() ){
        return false;
    }
    if( !APDS9960_setMode(AMBIENT_LIGHT, 1) ) {
        return false;
    }

    return true;

}

/**
 * @brief Ends the light sensor on the APDS-9960
 *
 * @return True if sensor disabled correctly. False on error.
 */
bool APDS9960_disableLightSensor()
{
    if( !APDS9960_setAmbientLightIntEnable(0) ) {
        return false;
    }
    if( !APDS9960_setMode(AMBIENT_LIGHT, 0) ) {
        return false;
    }

    return true;
}

/**
 * @brief Starts the proximity sensor on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware external interrupt on proximity
 * @return True if sensor enabled correctly. False on error.
 */
bool APDS9960_enableProximitySensor(bool interrupts)
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    if( !APDS9960_setProximityGain(DEFAULT_PGAIN) ) {
        return false;
    }
    if( !APDS9960_setLEDDrive(DEFAULT_LDRIVE) ) {
        return false;
    }
    if( interrupts ) {
        if( !APDS9960_setProximityIntEnable(1) ) {
            return false;
        }
    } else {
        if( !APDS9960_setProximityIntEnable(0) ) {
            return false;
        }
    }
    if( !APDS9960_enablePower() ){
        return false;
    }
    if( !APDS9960_setMode(PROXIMITY, 1) ) {
        return false;
    }

    return true;
}

/**
 * @brief Ends the proximity sensor on the APDS-9960
 *
 * @return True if sensor disabled correctly. False on error.
 */
bool APDS9960_disableProximitySensor()
{
 if( !APDS9960_setProximityIntEnable(0) ) {
 return false;
}
 if( !APDS9960_setMode(PROXIMITY, 0) ) {
 return false;
}

 return true;
}

/**
 * @brief Starts the gesture recognition engine on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware external interrupt on gesture
 * @return True if engine enabled correctly. False on error.
 */
bool APDS9960_enableGestureSensor(bool interrupts)
{

    /* Enable gesture mode
       Set ENABLE to 0 (power off)
       Set WTIME to 0xFF
       Set AUX to LED_BOOST_300
       Enable PON, WEN, PEN, GEN in ENABLE
    */
    APDS9960_resetGestureParameters();
    if( !wireWriteDataByte(APDS9960_WTIME, 0xFF) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE) ) {
        return false;
    }
    if( !APDS9960_setLEDBoost(LED_BOOST_300) ) {
        return false;
    }
    if( interrupts ) {
        if( !APDS9960_setGestureIntEnable(1) ) {
            return false;
        }
    } else {
        if( !APDS9960_setGestureIntEnable(0) ) {
            return false;
        }
    }
    if( !APDS9960_setGestureMode(1) ) {
        return false;
    }
    if( !APDS9960_enablePower() ){
        return false;
    }
    if( !APDS9960_setMode(WAIT, 1) ) {
        return false;
    }
    if( !APDS9960_setMode(PROXIMITY, 1) ) {
        return false;
    }
    if( !APDS9960_setMode(GESTURE, 1) ) {
        return false;
    }

    return true;
}

/**
 * @brief Ends the gesture recognition engine on the APDS-9960
 *
 * @return True if engine disabled correctly. False on error.
 */
bool APDS9960_disableGestureSensor()
{
    APDS9960_resetGestureParameters();
    if( !APDS9960_setGestureIntEnable(0) ) {
        return false;
    }
    if( !APDS9960_setGestureMode(0) ) {
        return false;
    }
    if( !APDS9960_setMode(GESTURE, 0) ) {
        return false;
    }

    return true;
}

/**
 * @brief Determines if there is a gesture available for reading
 *
 * @return True if gesture available. False otherwise.
 */
bool APDS9960_isGestureAvailable()
{
    uint8_t val;

    /* Read value from GSTATUS register */
    if( !wireReadRegDataByte(APDS9960_GSTATUS, &val) ) {
        return ERROR;
    }

    /* Shift and mask out GVALID bit */
    val &= APDS9960_GVALID;

    /* Return true/false based on GVALID bit */
    if( val == 1) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Processes a gesture event and returns best guessed gesture
 *
 * @return Number corresponding to gesture. -1 on error.
 */
int APDS9960_readGesture()
{
    uint8_t fifo_level = 0;
    uint8_t bytes_read = 0;
    uint8_t fifo_data[128];
    uint8_t gstatus;
    int motion;
    int i;

    /* Make sure that power and gesture is on and data is valid */
    if( !APDS9960_isGestureAvailable() || !(APDS9960_getMode() & 0b01000001) ) {
        return DIR_NONE;
    }

    /* Keep looping as long as gesture data is valid */
    while(1) {

        /* Wait some time to collect next batch of FIFO data */
        delayms(FIFO_PAUSE_TIME);

        /* Get the contents of the STATUS register. Is data still valid? */
        if( !wireReadRegDataByte(APDS9960_GSTATUS, &gstatus) ) {
            return ERROR;
        }

        /* If we have valid data, read in FIFO */
        if( (gstatus & APDS9960_GVALID) == APDS9960_GVALID ) {

            /* Read the current FIFO level */
            if( !wireReadRegDataByte(APDS9960_GFLVL, &fifo_level) ) {
                return ERROR;
            }

#if DEBUG
            printf("FIFO Level = %u\r\n",fifo_level);
#endif

            /* If there's stuff in the FIFO, read it into our data block */
            if( fifo_level > 0) {
                bytes_read = wireReadDataBlock(  APDS9960_GFIFO_U,
                                                (uint8_t*)fifo_data,
                                                (fifo_level * 4) );
                if( bytes_read == -1 ) return ERROR;

#if DEBUG
                printf("FIFO Dump: ");
                for ( i = 0; i < bytes_read; i++ )                  printf("%02x ",fifo_data[i]);
                printf("\r\n");
#endif

                /* If at least 1 set of data, sort the data into U/D/L/R */
                if( bytes_read >= 4 ) {
                    for( i = 0; i < bytes_read; i += 4 ) {
                        APDSmodule.gesture_data_.u_data[APDSmodule.gesture_data_.index] =   fifo_data[i + 0];
                        APDSmodule.gesture_data_.d_data[APDSmodule.gesture_data_.index] =  fifo_data[i + 1];
                        APDSmodule.gesture_data_.l_data[APDSmodule.gesture_data_.index] =  fifo_data[i + 2];
                        APDSmodule.gesture_data_.r_data[APDSmodule.gesture_data_.index] =   fifo_data[i + 3];
                        APDSmodule.gesture_data_.index++;
                        APDSmodule.gesture_data_.total_gestures++;
                    }

#if DEBUG
                    printf("Up Data: ");
                    for ( i = 0; i < APDSmodule.gesture_data_.total_gestures; i++ ) {
                    	printf("%02x ", APDSmodule.gesture_data_.u_data[i]);
                    }
                    printf("\r\n");
#endif

                    // Filter and process gesture data. Decode near/far state
                    if( APDS9960_processGestureData() ) {
                        if( APDS9960_decodeGesture() ) {
                            //***TODO: U-Turn Gestures
#if DEBUG
                            //Serial.println(APDSmodule.gesture_motion_);
#endif
                        }
                    }

                    // Reset data
                    APDSmodule.gesture_data_.index = 0;
                    APDSmodule.gesture_data_.total_gestures = 0;
                }
            }
        } else {
            /* Determine best guessed gesture and clean up */
            delayms(FIFO_PAUSE_TIME);
            APDS9960_decodeGesture();
            motion = APDSmodule.gesture_motion_;
#if DEBUG
            Serial.print("END: ");
            Serial.println(APDSmodule.gesture_motion_);
#endif
            APDS9960_resetGestureParameters();
            printf("Motion = %u\r\n",motion);
            return motion;
        }
    }
}

/**
 * Turn the APDS-9960 on
 *
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_enablePower()
{
    if( !APDS9960_setMode(POWER, 1) ) {
        return false;
    }
    printf("Enable Power\r\n");
    return true;
}

/**
 * Turn the APDS-9960 off
 *
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_disablePower()
{
    if( !APDS9960_setMode(POWER, 0) ) {
        return false;
    }

    return true;
}

/*******************************************************************************
 * Ambient light and color sensor controls
 ******************************************************************************/

/**
 * @brief Reads the ambient (clear) light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_readAmbientLight(unsigned short *val)
{
    uint8_t val_byte;
    *val = 0;

    /* Read value from clear channel, low byte register */
    if( !wireReadRegDataByte(APDS9960_CDATAL, &val_byte) ) {
        return false;
    }
    *val = val_byte;

    /* Read value from clear channel, high byte register */
    if( !wireReadRegDataByte(APDS9960_CDATAH, &val_byte) ) {
        return false;
    }
    *val = *val + ((unsigned short)val_byte << 8);

    return true;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_readRedLight(unsigned short *val)
{
    uint8_t val_byte;
    *val = 0;

    /* Read value from clear channel, low byte register */
    if( !wireReadRegDataByte(APDS9960_RDATAL, &val_byte) ) {
        return false;
    }
    *val = val_byte;

    /* Read value from clear channel, high byte register */
    if( !wireReadRegDataByte(APDS9960_RDATAH, &val_byte) ) {
        return false;
    }
    *val = *val + ((unsigned short)val_byte << 8);

    return true;
}

/**
 * @brief Reads the green light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_readGreenLight(unsigned short *val)
{
    uint8_t val_byte;
    *val = 0;

    /* Read value from clear channel, low byte register */
    if( !wireReadRegDataByte(APDS9960_GDATAL, &val_byte) ) {
        return false;
    }
    *val = val_byte;

    /* Read value from clear channel, high byte register */
    if( !wireReadRegDataByte(APDS9960_GDATAH, &val_byte) ) {
        return false;
    }
    *val = *val + ((unsigned short)val_byte << 8);

    return true;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_readBlueLight(unsigned short *val)
{
    uint8_t val_byte;
    *val = 0;

    /* Read value from clear channel, low byte register */
    if( !wireReadRegDataByte(APDS9960_BDATAL, &val_byte) ) {
        return false;
    }
    *val = val_byte;

    /* Read value from clear channel, high byte register */
    if( !wireReadRegDataByte(APDS9960_BDATAH, &val_byte) ) {
        return false;
    }
    *val = *val + ((unsigned short)val_byte << 8);

    return true;
}

/*******************************************************************************
 * Proximity sensor controls
 ******************************************************************************/

/**
 * @brief Reads the proximity level as an 8-bit value
 *
 * @param[out] val value of the proximity sensor.
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_readProximity(uint8_t *val)
{
    *val = 0;

    /* Read value from proximity data register */
    if( !wireReadRegDataByte(APDS9960_PROXIMITY_DATA_REG9C, val) ) {
        return false;
    }

    return true;
}

/*******************************************************************************
 * High-level gesture controls
 ******************************************************************************/

/**
 * @brief Resets all the parameters in the gesture data member
 */
void APDS9960_resetGestureParameters()
{
    APDSmodule.gesture_data_.index = 0;
    APDSmodule.gesture_data_.total_gestures = 0;

    APDSmodule.gesture_ud_delta_ = 0;
    APDSmodule.gesture_lr_delta_ = 0;

    APDSmodule.gesture_ud_count_ = 0;
    APDSmodule.gesture_lr_count_ = 0;

    APDSmodule.gesture_near_count_ = 0;
    APDSmodule.gesture_far_count_ = 0;

    APDSmodule.gesture_state_ = 0;
    APDSmodule.gesture_motion_ = DIR_NONE;
}

/**
 * @brief Processes the raw gesture data to determine swipe direction
 *
 * @return True if near or far state seen. False otherwise.
 */
bool APDS9960_processGestureData()
{
    uint8_t u_first = 0;
    uint8_t d_first = 0;
    uint8_t l_first = 0;
    uint8_t r_first = 0;
    uint8_t u_last = 0;
    uint8_t d_last = 0;
    uint8_t l_last = 0;
    uint8_t r_last = 0;
    int ud_ratio_first;
    int lr_ratio_first;
    int ud_ratio_last;
    int lr_ratio_last;
    int ud_delta;
    int lr_delta;
    int i;

    /* If we have less than 4 total gestures, that's not enough */
    if( APDSmodule.gesture_data_.total_gestures <= 4 ) {
        return false;
    }

    /* Check to make sure our data isn't out of bounds */
    if( (APDSmodule.gesture_data_.total_gestures <= 32) && (APDSmodule.gesture_data_.total_gestures > 0) ) {

        /* Find the first value in U/D/L/R above the threshold */
        for( i = 0; i < APDSmodule.gesture_data_.total_gestures; i++ ) {
            if( (APDSmodule.gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (APDSmodule.gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (APDSmodule.gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (APDSmodule.gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {

                u_first = APDSmodule.gesture_data_.u_data[i];
                d_first = APDSmodule.gesture_data_.d_data[i];
                l_first = APDSmodule.gesture_data_.l_data[i];
                r_first = APDSmodule.gesture_data_.r_data[i];
                break;
            }
        }

        /* If one of the _first values is 0, then there is no good data */
        if( (u_first == 0) || (d_first == 0) ||
            (l_first == 0) || (r_first == 0) ) {

            return false;
        }
        /* Find the last value in U/D/L/R above the threshold */
        for( i = APDSmodule.gesture_data_.total_gestures - 1; i >= 0; i-- ) {
#if DEBUG
            Serial.print(F("Finding last: "));
            Serial.print(F("U:"));
            Serial.print(APDSmodule.gesture_data_.u_data[i]);
            Serial.print(F(" D:"));
            Serial.print(APDSmodule.gesture_data_.d_data[i]);
            Serial.print(F(" L:"));
            Serial.print(APDSmodule.gesture_data_.l_data[i]);
            Serial.print(F(" R:"));
            Serial.println(APDSmodule.gesture_data_.r_data[i]);
#endif
            if( (APDSmodule.gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (APDSmodule.gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (APDSmodule.gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (APDSmodule.gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {

                u_last = APDSmodule.gesture_data_.u_data[i];
                d_last = APDSmodule.gesture_data_.d_data[i];
                l_last = APDSmodule.gesture_data_.l_data[i];
                r_last = APDSmodule.gesture_data_.r_data[i];
                break;
            }
        }
    }

    /* Calculate the first vs. last ratio of up/down and left/right */
    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);

#if DEBUG
    Serial.print(F("Last Values: "));
    Serial.print(F("U:"));
    Serial.print(u_last);
    Serial.print(F(" D:"));
    Serial.print(d_last);
    Serial.print(F(" L:"));
    Serial.print(l_last);
    Serial.print(F(" R:"));
    Serial.println(r_last);

    Serial.print(F("Ratios: "));
    Serial.print(F("UD Fi: "));
    Serial.print(ud_ratio_first);
    Serial.print(F(" UD La: "));
    Serial.print(ud_ratio_last);
    Serial.print(F(" LR Fi: "));
    Serial.print(lr_ratio_first);
    Serial.print(F(" LR La: "));
    Serial.println(lr_ratio_last);
#endif

    /* Determine the difference between the first and last ratios */
    ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;

#if DEBUG
    Serial.print("Deltas: ");
    Serial.print("UD: ");
    Serial.print(ud_delta);
    Serial.print(" LR: ");
    Serial.println(lr_delta);
#endif

    /* Accumulate the UD and LR delta values */
    APDSmodule.gesture_ud_delta_ += ud_delta;
    APDSmodule.gesture_lr_delta_ += lr_delta;

#if DEBUG
    Serial.print("Accumulations: ");
    Serial.print("UD: ");
    Serial.print(APDSmodule.gesture_ud_delta_);
    Serial.print(" LR: ");
    Serial.println(APDSmodule.gesture_lr_delta_);
#endif

    /* Determine U/D gesture */
    if( APDSmodule.gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
        APDSmodule.gesture_ud_count_ = 1;
    } else if( APDSmodule.gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        APDSmodule.gesture_ud_count_ = -1;
    } else {
        APDSmodule.gesture_ud_count_ = 0;
    }

    /* Determine L/R gesture */
    if( APDSmodule.gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
        APDSmodule.gesture_lr_count_ = 1;
    } else if( APDSmodule.gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        APDSmodule.gesture_lr_count_ = -1;
    } else {
        APDSmodule.gesture_lr_count_ = 0;
    }

    /* Determine Near/Far gesture */
    if( (APDSmodule.gesture_ud_count_ == 0) && (APDSmodule.gesture_lr_count_ == 0) ) {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) &&
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {

            if( (ud_delta == 0) && (lr_delta == 0) ) {
                APDSmodule.gesture_near_count_++;
            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
                APDSmodule.gesture_far_count_++;
            }

            if( (APDSmodule.gesture_near_count_ >= 10) && (APDSmodule.gesture_far_count_ >= 2) ) {
                if( (ud_delta == 0) && (lr_delta == 0) ) {
                    APDSmodule.gesture_state_ = NEAR_STATE;
                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
                    APDSmodule.gesture_state_ = FAR_STATE;
                }
                return true;
            }
        }
    } else {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) &&
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {

            if( (ud_delta == 0) && (lr_delta == 0) ) {
                APDSmodule.gesture_near_count_++;
            }

            if( APDSmodule.gesture_near_count_ >= 10 ) {
                APDSmodule.gesture_ud_count_ = 0;
                APDSmodule.gesture_lr_count_ = 0;
                APDSmodule.gesture_ud_delta_ = 0;
                APDSmodule.gesture_lr_delta_ = 0;
            }
        }
    }

#if DEBUG
    Serial.print("UD_CT: ");
    Serial.print(APDSmodule.gesture_ud_count_);
    Serial.print(" LR_CT: ");
    Serial.print(APDSmodule.gesture_lr_count_);
    Serial.print(" NEAR_CT: ");
    Serial.print(APDSmodule.gesture_near_count_);
    Serial.print(" FAR_CT: ");
    Serial.println(APDSmodule.gesture_far_count_);
    Serial.println("----------");
#endif

    return false;
}

/**
 * @brief Determines swipe direction or near/far state
 *
 * @return True if near/far event. False otherwise.
 */
bool APDS9960_decodeGesture()
{
    /* Return if near or far event is detected */
    if( APDSmodule.gesture_state_ == NEAR_STATE ) {
        APDSmodule.gesture_motion_ = DIR_NEAR;
        return true;
    } else if ( APDSmodule.gesture_state_ == FAR_STATE ) {
        APDSmodule.gesture_motion_ = DIR_FAR;
        return true;
    }

    /* Determine swipe direction */
    if( (APDSmodule.gesture_ud_count_ == -1) && (APDSmodule.gesture_lr_count_ == 0) ) {
        APDSmodule.gesture_motion_ = DIR_UP;
    } else if( (APDSmodule.gesture_ud_count_ == 1) && (APDSmodule.gesture_lr_count_ == 0) ) {
        APDSmodule.gesture_motion_ = DIR_DOWN;
    } else if( (APDSmodule.gesture_ud_count_ == 0) && (APDSmodule.gesture_lr_count_ == 1) ) {
        APDSmodule.gesture_motion_ = DIR_RIGHT;
    } else if( (APDSmodule.gesture_ud_count_ == 0) && (APDSmodule.gesture_lr_count_ == -1) ) {
        APDSmodule.gesture_motion_ = DIR_LEFT;
    } else if( (APDSmodule.gesture_ud_count_ == -1) && (APDSmodule.gesture_lr_count_ == 1) ) {
        if( abs(APDSmodule.gesture_ud_delta_) > abs(APDSmodule.gesture_lr_delta_) ) {
            APDSmodule.gesture_motion_ = DIR_UP;
        } else {
            APDSmodule.gesture_motion_ = DIR_RIGHT;
        }
    } else if( (APDSmodule.gesture_ud_count_ == 1) && (APDSmodule.gesture_lr_count_ == -1) ) {
        if( abs(APDSmodule.gesture_ud_delta_) > abs(APDSmodule.gesture_lr_delta_) ) {
            APDSmodule.gesture_motion_ = DIR_DOWN;
        } else {
            APDSmodule.gesture_motion_ = DIR_LEFT;
        }
    } else if( (APDSmodule.gesture_ud_count_ == -1) && (APDSmodule.gesture_lr_count_ == -1) ) {
        if( abs(APDSmodule.gesture_ud_delta_) > abs(APDSmodule.gesture_lr_delta_) ) {
            APDSmodule.gesture_motion_ = DIR_UP;
        } else {
            APDSmodule.gesture_motion_ = DIR_LEFT;
        }
    } else if( (APDSmodule.gesture_ud_count_ == 1) && (APDSmodule.gesture_lr_count_ == 1) ) {
        if( abs(APDSmodule.gesture_ud_delta_) > abs(APDSmodule.gesture_lr_delta_) ) {
            APDSmodule.gesture_motion_ = DIR_DOWN;
        } else {
            APDSmodule.gesture_motion_ = DIR_RIGHT;
        }
    } else {
        return false;
    }

    return true;
}

/*******************************************************************************
 * Getters and setters for register values
 ******************************************************************************/

/**
 * @brief Returns the lower threshold for proximity detection
 *
 * @return lower threshold
 */
uint8_t APDS9960_getProxIntLowThresh()
{
    uint8_t val;

    /* Read value from PILT register */
    if( !wireReadRegDataByte(APDS9960_PILT, &val) ) {
        val = 0;
    }

    return val;
}

/**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] threshold the lower proximity threshold
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setProxIntLowThresh(uint8_t threshold)
{
    if( !wireWriteDataByte(APDS9960_PILT, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Returns the high threshold for proximity detection
 *
 * @return high threshold
 */
uint8_t APDS9960_getProxIntHighThresh()
{
    uint8_t val;

    /* Read value from PIHT register */
    if( !wireReadRegDataByte(APDS9960_PIHT, &val) ) {
        val = 0;
    }

    return val;
}

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setProxIntHighThresh(uint8_t threshold)
{
    if( !wireWriteDataByte(APDS9960_PIHT, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Returns LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the value of the LED drive strength. 0xFF on failure.
 */
uint8_t APDS9960_getLEDDrive()
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadRegDataByte(APDS9960_CONTROL_REG8F, &val) ) {
        return ERROR;
    }

    /* Shift and mask out LED drive bits */
    val = (val >> 6) & 0b00000011;

    return val;
}

/**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value (0-3) for the LED drive strength
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setLEDDrive(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadRegDataByte(APDS9960_CONTROL_REG8F, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 6;
    val &= 0b00111111;
    val |= drive;

    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9960_CONTROL_REG8F, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Returns receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the value of the proximity gain. 0xFF on failure.
 */
uint8_t APDS9960_getProximityGain()
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadRegDataByte(APDS9960_CONTROL_REG8F, &val) ) {
        return ERROR;
    }

    /* Shift and mask out PDRIVE bits */
    val = (val >> 2) & 0b00000011;

    return val;
}

/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setProximityGain(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadRegDataByte(APDS9960_CONTROL_REG8F, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 2;
    val &= 0b11110011;
    val |= drive;

    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9960_CONTROL_REG8F, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Returns receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @return the value of the ALS gain. 0xFF on failure.
 */
uint8_t APDS9960_getAmbientLightGain()
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadRegDataByte(APDS9960_CONTROL_REG8F, &val) ) {
        return ERROR;
    }

    /* Shift and mask out ADRIVE bits */
    val &= 0b00000011;

    return val;
}

/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setAmbientLightGain(uint8_t drive)
{
    uint8_t val;

    /* Read value from CONTROL register */
    if( !wireReadRegDataByte(APDS9960_CONTROL_REG8F, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    val &= 0b11111100;
    val |= drive;

    /* Write register value back into CONTROL register */
    if( !wireWriteDataByte(APDS9960_CONTROL_REG8F, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Get the current LED boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @return The LED boost value. 0xFF on failure.
 */
uint8_t APDS9960_getLEDBoost()
{
    uint8_t val;

    /* Read value from CONFIG2 register */
    if( !wireReadRegDataByte(APDS9960_CONFIG2_REG90, &val) ) {
        return ERROR;
    }

    /* Shift and mask out LED_BOOST bits */
    val = (val >> 4) & 0b00000011;

    return val;
}

/**
 * @brief Sets the LED current boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setLEDBoost(uint8_t boost)
{
    uint8_t val;

    /* Read value from CONFIG2 register */
    if( !wireReadRegDataByte(APDS9960_CONFIG2_REG90, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    boost &= 0b00000011;
    boost = boost << 4;
    val &= 0b11001111;
    val |= boost;

    /* Write register value back into CONFIG2 register */
    if( !wireWriteDataByte(APDS9960_CONFIG2_REG90, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets proximity gain compensation enable
 *
 * @return 1 if compensation is enabled. 0 if not. 0xFF on error.
 */
uint8_t APDS9960_getProxGainCompEnable()
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if( !wireReadRegDataByte(APDS9960_CONFIG3_FOR_PROXIMITY_REG9F, &val) ) {
        return ERROR;
    }

    /* Shift and mask out PCMP bits */
    val = (val >> 5) & 0b00000001;

    return val;
}

/**
 * @brief Sets the proximity gain compensation enable
 *
 * @param[in] enable 1 to enable compensation. 0 to disable compensation.
 * @return True if operation successful. False otherwise.
 */
 bool APDS9960_setProxGainCompEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if( !wireReadRegDataByte(APDS9960_CONFIG3_FOR_PROXIMITY_REG9F, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 5;
    val &= 0b11011111;
    val |= enable;

    /* Write register value back into CONFIG3 register */
    if( !wireWriteDataByte(APDS9960_CONFIG3_FOR_PROXIMITY_REG9F, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the current mask for enabled/disabled proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @return Current proximity mask for photodiodes. 0xFF on error.
 */
uint8_t APDS9960_getProxPhotoMask()
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if( !wireReadRegDataByte(APDS9960_CONFIG3_FOR_PROXIMITY_REG9F, &val) ) {
        return ERROR;
    }

    /* Mask out photodiode enable mask bits */
    val &= 0b00001111;

    return val;
}

/**
 * @brief Sets the mask for enabling/disabling proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[in] mask 4-bit mask value
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setProxPhotoMask(uint8_t mask)
{
    uint8_t val;

    /* Read value from CONFIG3 register */
    if( !wireReadRegDataByte(APDS9960_CONFIG3_FOR_PROXIMITY_REG9F, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    mask &= 0b00001111;
    val &= 0b11110000;
    val |= mask;

    /* Write register value back into CONFIG3 register */
    if( !wireWriteDataByte(APDS9960_CONFIG3_FOR_PROXIMITY_REG9F, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the entry proximity threshold for gesture sensing
 *
 * @return Current entry proximity threshold.
 */
uint8_t APDS9960_getGestureEnterThresh()
{
    uint8_t val;

    /* Read value from GPENTH register */
    if( !wireReadRegDataByte(APDS9960_PROXIMITY_ENTER_THRESHOLD_REGA0, &val) ) {
        val = 0;
    }

    return val;
}

/**
 * @brief Sets the entry proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to start gesture mode
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setGestureEnterThresh(uint8_t threshold)
{
    if( !wireWriteDataByte(APDS9960_PROXIMITY_ENTER_THRESHOLD_REGA0, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the exit proximity threshold for gesture sensing
 *
 * @return Current exit proximity threshold.
 */
uint8_t APDS9960_getGestureExitThresh()
{
    uint8_t val;

    /* Read value from GEXTH register */
    if( !wireReadRegDataByte(APDS9960_GEXTH, &val) ) {
        val = 0;
    }

    return val;
}

/**
 * @brief Sets the exit proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to end gesture mode
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setGestureExitThresh(uint8_t threshold)
{
    if( !wireWriteDataByte(APDS9960_GEXTH, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the current photodiode gain. 0xFF on error.
 */
uint8_t APDS9960_getGestureGain()
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( !wireReadRegDataByte(APDS9960_GCONF2, &val) ) {
        return ERROR;
    }

    /* Shift and mask out GGAIN bits */
    val = (val >> 5) & 0b00000011;

    return val;
}

/**
 * @brief Sets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] gain the value for the photodiode gain
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setGestureGain(uint8_t gain)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( !wireReadRegDataByte(APDS9960_GCONF2, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    gain &= 0b00000011;
    gain = gain << 5;
    val &= 0b10011111;
    val |= gain;

    /* Write register value back into GCONF2 register */
    if( !wireWriteDataByte(APDS9960_GCONF2, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the drive current of the LED during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the LED drive current value. 0xFF on error.
 */
uint8_t APDS9960_getGestureLEDDrive()
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( !wireReadRegDataByte(APDS9960_GCONF2, &val) ) {
        return ERROR;
    }

    /* Shift and mask out GLDRIVE bits */
    val = (val >> 3) & 0b00000011;

    return val;
}

/**
 * @brief Sets the LED drive current during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value for the LED drive current
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setGestureLEDDrive(uint8_t drive)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( !wireReadRegDataByte(APDS9960_GCONF2, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 3;
    val &= 0b11100111;
    val |= drive;

    /* Write register value back into GCONF2 register */
    if( !wireWriteDataByte(APDS9960_GCONF2, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @return the current wait time between gestures. 0xFF on error.
 */
uint8_t APDS9960_getGestureWaitTime()
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( !wireReadRegDataByte(APDS9960_GCONF2, &val) ) {
        return ERROR;
    }

    /* Mask out GWTIME bits */
    val &= 0b00000111;

    return val;
}

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] the value for the wait time
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setGestureWaitTime(uint8_t time)
{
    uint8_t val;

    /* Read value from GCONF2 register */
    if( !wireReadRegDataByte(APDS9960_GCONF2, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    time &= 0b00000111;
    val &= 0b11111000;
    val |= time;

    /* Write register value back into GCONF2 register */
    if( !wireWriteDataByte(APDS9960_GCONF2, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the low threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_getLightIntLowThreshold(unsigned short *threshold)
{
    uint8_t val_byte;
    *threshold = 0;

    /* Read value from ambient light low threshold, low byte register */
    if( !wireReadRegDataByte(APDS9960_AILTL, &val_byte) ) {
        return false;
    }
    *threshold = val_byte;

    /* Read value from ambient light low threshold, high byte register */
    if( !wireReadRegDataByte(APDS9960_AILTH, &val_byte) ) {
        return false;
    }
    *threshold = *threshold + ((unsigned short)val_byte << 8);

    return true;
}

/**
 * @brief Sets the low threshold for ambient light interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setLightIntLowThreshold(unsigned short threshold)
{
    uint8_t val_low;
    uint8_t val_high;

    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;

    /* Write low byte */
    if( !wireWriteDataByte(APDS9960_AILTL, val_low) ) {
        return false;
    }

    /* Write high byte */
    if( !wireWriteDataByte(APDS9960_AILTH, val_high) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the high threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_getLightIntHighThreshold(unsigned short *threshold)
{
    uint8_t val_byte;
    *threshold = 0;

    /* Read value from ambient light high threshold, low byte register */
    if( !wireReadRegDataByte(APDS9960_AIHTL, &val_byte) ) {
        return false;
    }
    *threshold = val_byte;

    /* Read value from ambient light high threshold, high byte register */
    if( !wireReadRegDataByte(APDS9960_AIHTH, &val_byte) ) {
        return false;
    }
    *threshold = *threshold + ((unsigned short)val_byte << 8);

    return true;
}

/**
 * @brief Sets the high threshold for ambient light interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setLightIntHighThreshold(unsigned short threshold)
{
    uint8_t val_low;
    uint8_t val_high;

    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;

    /* Write low byte */
    if( !wireWriteDataByte(APDS9960_AIHTL, val_low) ) {
        return false;
    }

    /* Write high byte */
    if( !wireWriteDataByte(APDS9960_AIHTH, val_high) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the low threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_getProximityIntLowThreshold(uint8_t *threshold)
{
    *threshold = 0;

    /* Read value from proximity low threshold register */
    if( !wireReadRegDataByte(APDS9960_PILT, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Sets the low threshold for proximity interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setProximityIntLowThreshold(uint8_t threshold)
{

    /* Write threshold value to register */
    if( !wireWriteDataByte(APDS9960_PILT, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets the high threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_getProximityIntHighThreshold(uint8_t *threshold)
{
    *threshold = 0;

    /* Read value from proximity low threshold register */
    if( !wireReadRegDataByte(APDS9960_PIHT, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Sets the high threshold for proximity interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setProximityIntHighThreshold(uint8_t threshold)
{

    /* Write threshold value to register */
    if( !wireWriteDataByte(APDS9960_PIHT, threshold) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets if ambient light interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_getAmbientLightIntEnable()
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( !wireReadRegDataByte(APDS9960_ENABLE_REG80, &val) ) {
        return ERROR;
    }

    /* Shift and mask out AIEN bit */
    val = (val >> 4) & 0b00000001;

    return val;
}

/**
 * @brief Turns ambient light interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setAmbientLightIntEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( !wireReadRegDataByte(APDS9960_ENABLE_REG80, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 4;
    val &= 0b11101111;
    val |= enable;

    /* Write register value back into ENABLE register */
    if( !wireWriteDataByte(APDS9960_ENABLE_REG80, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets if proximity interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_getProximityIntEnable()
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( !wireReadRegDataByte(APDS9960_ENABLE_REG80, &val) ) {
        return ERROR;
    }

    /* Shift and mask out PIEN bit */
    val = (val >> 5) & 0b00000001;

    return val;
}

/**
 * @brief Turns proximity interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setProximityIntEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from ENABLE register */
    if( !wireReadRegDataByte(APDS9960_ENABLE_REG80, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 5;
    val &= 0b11011111;
    val |= enable;

    /* Write register value back into ENABLE register */
    if( !wireWriteDataByte(APDS9960_ENABLE_REG80, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Gets if gesture interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_getGestureIntEnable()
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if( !wireReadRegDataByte(APDS9960_GCONF4, &val) ) {
        return ERROR;
    }

    /* Shift and mask out GIEN bit */
    val = (val >> 1) & 0b00000001;

    return val;
}

/**
 * @brief Turns gesture-related interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setGestureIntEnable(uint8_t enable)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if( !wireReadRegDataByte(APDS9960_GCONF4, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    enable &= 0b00000001;
    enable = enable << 1;
    val &= 0b11111101;
    val |= enable;

    /* Write register value back into GCONF4 register */
    if( !wireWriteDataByte(APDS9960_GCONF4, val) ) {
        return false;
    }

    return true;
}

/**
 * @brief Clears the ambient light interrupt
 *
 * @return True if operation completed successfully. False otherwise.
 */
bool APDS9960_clearAmbientLightInt()
{
    uint8_t throwaway;
    if( !wireReadRegDataByte(APDS9960_AICLEAR, &throwaway) ) {
        return false;
    }

    return true;
}

/**
 * @brief Clears the proximity interrupt
 *
 * @return True if operation completed successfully. False otherwise.
 */
bool APDS9960_clearProximityInt()
{
    uint8_t throwaway;
    if( !wireReadRegDataByte(APDS9960_PICLEAR, &throwaway) ) {
        return false;
    }

    return true;
}

/**
 * @brief Tells if the gesture state machine is currently running
 *
 * @return 1 if gesture state machine is running, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_getGestureMode()
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if( !wireReadRegDataByte(APDS9960_GCONF4, &val) ) {
        return ERROR;
    }

    /* Mask out GMODE bit */
    val &= 0b00000001;

    return val;
}

/**
 * @brief Tells the state machine to either enter or exit gesture state machine
 *
 * @param[in] mode 1 to enter gesture state machine, 0 to exit.
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_setGestureMode(uint8_t mode)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if( !wireReadRegDataByte(APDS9960_GCONF4, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    mode &= 0b00000001;
    val &= 0b11111110;
    val |= mode;

    /* Write register value back into GCONF4 register */
    if( !wireWriteDataByte(APDS9960_GCONF4, val) ) {
        return false;
    }

    return true;
}



//===
/****************************************************************
ProximityTest.ino
APDS-9960 RGB and Gesture Sensor
Shawn Hymel @ SparkFun Electronics
October 28, 2014
https://github.com/sparkfun/APDS-9960_RGB_and_Gesture_Sensor

Tests the proximity sensing abilities of the APDS-9960.
Configures the APDS-9960 over I2C and polls for the distance to
the object nearest the sensor.

Hardware Connections:

IMPORTANT: The APDS-9960 can only accept 3.3V!

 Arduino Pin  APDS-9960 Board  Function

 3.3V         VCC              Power
 GND          GND              Ground
 A4           SDA              I2C Data
 A5           SCL              I2C Clock

Resources:
Include Wire.h and SparkFun_APDS-9960.h

Development environment specifics:
Written in Arduino 1.0.5
Tested with SparkFun Arduino Pro Mini 3.3V

This code is beerware; if you see me (or any other SparkFun
employee) at the local, and you've found our code helpful, please
buy us a round!

Distributed as-is; no warranty is given.
****************************************************************/




void stmAPDS9960_ShowAllReg(){
	int i;
	u8 regValue;
	for(i=0;i<=0xFF;i++){
		if(wireReadRegDataByte(i, &regValue)){
			printf("Reg[0x%02x]=0x%02x\r\n",i, regValue);
			delayms(10);
		}else{
			printf("I2C NO RESPONSE FROM SLAVE.\r\n");
		}
	}
}
//Check Product ID
int stmAPDS9960_CheckProduct(){
	u8 rxbuf[20];
	u8 mid,pid, rid;

	stm_I2C_ReceiveBurstWithRestartCondition(APDS9960_I2C_ADDR8, 0x03, &mid, 1);
	stm_I2C_ReceiveBurstWithRestartCondition(APDS9960_I2C_ADDR8, 0x04, &pid, 1);
	stm_I2C_ReceiveBurstWithRestartCondition(APDS9960_I2C_ADDR8, 0x05, &rid, 1);

	if ( (mid == 0x01 && pid == 0x02 && rid == 0x00) || (mid == 0x02 && pid == 0x02 && rid == 0x01)){
		printf("Ap3216 RevID [%d], ==> v2.0 AP3212C/Ap3216C detected\n", rid);
		return 1;
	}
	else{
		printf("MakeID[%d] ProductID[%d] RevID[%d] .... can't detect ... bad reversion!!!\n", mid, pid, rid);
		return 0;
	}
}
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
void stmAPDS9960_PA8_IRQpin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  /* Enable GPIOA clock */
 	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	  /* Enable and set EXTI Line8 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  /* Configure PA8 pin as input floating */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	  /* Connect EXTI Line8 to PA8 pin */
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
	  //SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);

	  /* Configure EXTI Line8 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);


}
#elif(PROCESSOR == PROCESSOR_STM32F401RET6)
//PB8 : 407VG
//PA8 : 401RET
void stmAPDS9960_PA8_IRQpin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  /* Enable GPIOA clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Configure PA8 pin as input floating */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Connect EXTI Line8 to PA8 pin */
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);

	  /* Configure EXTI Line8 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Line8 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
#else
//PE14 : 407VG-M36
//PC6  : 407VG-M34
//PC4  : 401RET
void stmAPDS9960_PC6_IRQpin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  /* Enable GPIOC clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Configure PC6 pin as input floating */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//GPIO_OType_PP; //???
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP; //GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  // Connect EXTI Line6 to PC6 pin
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);

	  // Configure EXTI Line6
	  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling; //EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Line8 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}

//M36
void stmAPDS9960_AP3216_PE14_IRQpin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);/* Enable GPIOE clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);/* Enable SYSCFG clock */

	  /* Configure PE14 pin as input floating */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//GPIO_OType_PP; //???
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	  GPIO_Init(GPIOE, &GPIO_InitStructure);

	  /* Connect EXTI Line14 to PE14 pin */
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource14);

	  /* Configure EXTI Line14 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Line14 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
void stmAPDS9960_PB8_IRQpin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);/* Enable GPIOB clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);/* Enable SYSCFG clock */

	  /* Configure PA8 pin as input floating */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//GPIO_OType_PP; //???
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Connect EXTI Line8 to PA8 pin */
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);

	  /* Configure EXTI Line8 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Line8 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
#endif
//PA8 : 103
//PB8 : 407VG-M36
//PC6 : 407VG-M34
//PC4 : 401RET

#if (USE_EXTI9_5 ==	USE_EXTI9_5_APDS9960)

void EXTI9_5_IRQHandler()
{
#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) //PA8
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
		// Toggle LED0
		GPIO_ToggleBits(GPIOC, GPIO_Pin_14); //ULED -103 (PC14)
		g_irq = 1;
		EXTI_ClearITPendingBit(EXTI_Line8);		// Clear the EXTI line 8 pending bit (PA8)
	}
#else
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
		// Toggle LED0
		//GPIO_ToggleBits(GPIOC, GPIO_Pin_4);
		GPIO_ToggleBits(GPIOE, GPIO_Pin_15); //ULED -407-M35
		g_irq = 1;
		EXTI_ClearITPendingBit(EXTI_Line6);		// Clear the EXTI line 6 pending bit (PC6)
	}
#endif
}

#endif

void APDS9960_AppSetup(){
/*
	//============= APPLICATIONS =================================
	//(1) App - Lux and Colors
	if ( APDS9960_enableLightSensor(APDS9960_NO_INTERRUPT))  { //==false
	  printf("Light sensor is now running\r\n");
	} else {
	  printf("Something went wrong during sensor init!\r\n");
	}
	delayms(500);

	stmAPDS9960_ShowAllReg();

	 while(1){
		 // Read lux value
		 if ( !APDS9960_readAmbientLight(&ambientLight_data) ) 	 printf("Error reading ambientLight_data\r\n");
		 if ( !APDS9960_readRedLight(&RedLight_data) ) 	 	printf("Error reading RedLight_data\r\n");
		 if ( !APDS9960_readGreenLight(&GreenLight_data) ) 	 	printf("Error reading GreenLight_data\r\n");
		 if ( !APDS9960_readGreenLight(&BlueLight_data) ) 	 	printf("Error reading BlueLight_data\r\n");
		 printf("Light(Ambient(%u), Red(%u), Green(%u), Blue(%u)\r\n", ambientLight_data,RedLight_data,GreenLight_data,BlueLight_data);

		 delayms(250);
	}

	//(2) App - Proximity
	// Adjust the Proximity sensor gain
	if ( !APDS9960_setProximityGain(PGAIN_2X) ) {
	  printf("Something went wrong trying to set PGAIN");
	}

	// Start running the APDS-9960 proximity sensor (no interrupts)
	if ( APDS9960_enableProximitySensor(APDS9960_NO_INTERRUPT))  { //==false
	  printf("Proximity sensor is now running");
	} else {
	  printf("Something went wrong during sensor init!");
	}

	 while(1){
		 // Read the proximity value
		 if ( !APDS9960_readProximity(&(APDSmodule.proximity_data)) ) {
			 printf("Error reading proximity value");
		 } else {
			 printf("Proximity: %u\r\n", APDSmodule.proximity_data);
		 }
		 delayms(250);
	}

*/
	//(3) App - Gesture Sensor
	 //stmAPDS9960_PB8_IRQpin_Setup(); //407-M36
	if ( APDS9960_enableGestureSensor(1))  { //with Interrupt
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
			stmAPDS9960_PA8_IRQpin_Setup(); //103
#else
			//stmAPDS9960_PC6_IRQpin_Setup();//407-M34
			//stmAPDS9960_AP3216_PE14_IRQpin_Setup();//407-M36
#endif
			printf("Gesture sensor is now running");
	} else {
		printf("Something went wrong during sensor init!");
	}
}
bool APDS9960_init()
{
    uint8_t id;

#if ((PROCESSOR == PROCESSOR_STM32F103C8T6)  || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
	stmAPDS9960_PA8_IRQpin_Setup(); //103
#else
	//stmAPDS9960_PC6_IRQpin_Setup();//407-M34
	//stmAPDS9960_AP3216_PE14_IRQpin_Setup();//407-M34
#endif

	//Check Product ID
	//while(!stmAp3216CheckProduct()){};

    APDSmodule.gesture_ud_delta_ = 0;
    APDSmodule.gesture_lr_delta_ = 0;

    APDSmodule.gesture_ud_count_ = 0;
    APDSmodule.gesture_lr_count_ = 0;

    APDSmodule.gesture_near_count_ = 0;
    APDSmodule.gesture_far_count_ = 0;

    APDSmodule.gesture_state_ = 0;
    APDSmodule.gesture_motion_ = DIR_NONE;


    /* Read ID register and check against known values for APDS-9960 */
    if( !wireReadRegDataByte(APDS9960_ID_REG92, &id) ) {
        return false;
    }
    if( !(id == APDS9960_ID_REG92_1 || id == APDS9960_ID_REG92_2) ) {
    	printf("APDS-9960> NOT DEV FOUND..(Product ID = 0xAB)\r\n");
        return false;
    }else
    	printf("APDS-9960> FOUND..(Product ID = 0xAB)\r\n");

    /* Set ENABLE register to 0 (disable all features) */
    if( !APDS9960_setMode(ALL, OFF) ) {
        return false;
    }

    /* Set default values for ambient light and proximity registers */
    if( !wireWriteDataByte(APDS9960_ATIME, DEFAULT_ATIME) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_WTIME, DEFAULT_WTIME) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_PROXIMITY_OFFSET_UP_RIGHT_REG9D, DEFAULT_POFFSET_UR) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_PROXIMITY_OFFSET_DOWN_LEFT_REG9E, DEFAULT_POFFSET_DL) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_CONFIG1_REG8D, DEFAULT_CONFIG1) ) {
        return false;
    }
    if( !APDS9960_setLEDDrive(DEFAULT_LDRIVE) ) {
        return false;
    }
    if( !APDS9960_setProximityGain(DEFAULT_PGAIN) ) {
        return false;
    }
    if( !APDS9960_setAmbientLightGain(DEFAULT_AGAIN) ) {
        return false;
    }
    if( !APDS9960_setProxIntLowThresh(DEFAULT_PILT) ) {
        return false;
    }
    if( !APDS9960_setProxIntHighThresh(DEFAULT_PIHT) ) {
        return false;
    }
    if( !APDS9960_setLightIntLowThreshold(DEFAULT_AILT) ) {
        return false;
    }
    if( !APDS9960_setLightIntHighThreshold(DEFAULT_AIHT) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_PERS, DEFAULT_PERS) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_CONFIG2_REG90, DEFAULT_CONFIG2) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_CONFIG3_FOR_PROXIMITY_REG9F, DEFAULT_CONFIG3) ) {
        return false;
    }

    /* Set default values for gesture sense registers */
    if( !APDS9960_setGestureEnterThresh(DEFAULT_GPENTH) ) {
        return false;
    }
    if( !APDS9960_setGestureExitThresh(DEFAULT_GEXTH) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GCONF1, DEFAULT_GCONF1) ) {
        return false;
    }
    if( !APDS9960_setGestureGain(DEFAULT_GGAIN) ) {
        return false;
    }
    if( !APDS9960_setGestureLEDDrive(DEFAULT_GLDRIVE) ) {
        return false;
    }
    if( !APDS9960_setGestureWaitTime(DEFAULT_GWTIME) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GPULSE, DEFAULT_GPULSE) ) {
        return false;
    }
    if( !wireWriteDataByte(APDS9960_GCONF3, DEFAULT_GCONF3) ) {
        return false;
    }
    if( !APDS9960_setGestureIntEnable(DEFAULT_GIEN) ) {
        return false;
    }

#if 0
    /* Gesture config register dump */
    uint8_t reg;
    uint8_t val;

    for(reg = 0x80; reg <= 0xAF; reg++) {
        if( (reg != 0x82) && \\
            (reg != 0x8A) && \\
            (reg != 0x91) && \\
            (reg != 0xA8) && \\
            (reg != 0xAC) && \\
            (reg != 0xAD) )
        {
            wireReadRegDataByte(reg, &val);
            Serial.print(reg, HEX);
            Serial.print(": 0x");
            Serial.println(val, HEX);
        }
    }

    for(reg = 0xE4; reg <= 0xE7; reg++) {
        wireReadRegDataByte(reg, &val);
        Serial.print(reg, HEX);
        Serial.print(": 0x");
        Serial.println(val, HEX);
    }
#endif

	stmAPDS9960_ShowAllReg();

    return true;
}
void stmAPDS9960Loop (void)
{
    int byteh;
    int bytel;
    int var;
    u8 retVal;
    //float output;
    unsigned short output;
	u8 i;
	u8 rxbuf[20];
	u8 txbuf[2];
	u8 ir_ps_InValid;
	u8 ints;
	unsigned short ird, msb, lsb;
	u8 irvalid;
	u8 mid,pid, rid;
	unsigned short ambientLight_data,RedLight_data,GreenLight_data,BlueLight_data;
    GPIO_InitTypeDef   GPIO_InitStruct;

	printf("------------------------------------");
	printf("APDS-9960 - Proximity/Light/Gesture Sensor");
	printf("------------------------------------");

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

	printf("stmApds9960 Light/Gesture Sensor Test with I2C1\r\n");

	//stmUser_LED_GPIO_setup(); //Accessory LED

	APDS9960_init();// read all the registers once to fill the cache.
	APDS9960_AppSetup();

	 while(1){
		 if( g_irq){//		 if( APDSmodule.irq){
			 if ( APDS9960_isGestureAvailable() ) {//if ( !APDS9960_isGestureAvailable() ) {
				 switch(APDS9960_readGesture()){
				 case DIR_UP:
					 printf("Up\r\n"); break;
				 case DIR_DOWN:
					 printf("Down\r\n");break;
				 case DIR_LEFT:
					 printf("Left\r\n"); break;
				 case DIR_RIGHT:
					 printf("Right\r\n");break;
				 case DIR_NEAR:
					 printf("Near\r\n"); break;
				 case DIR_FAR:
					 printf("Far\r\n");break;
				 default:
					 printf("None\r\n");break;
				 }
			 }
			 g_irq = 0;// APDSmodule.irq = 0;
		 }
	}

}
