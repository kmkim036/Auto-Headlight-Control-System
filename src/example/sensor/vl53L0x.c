#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT6)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#else
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "lwip/include/stm32f4x7_eth.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "lwipopts.h"
#include "lwip/include/mem.h"
#endif
// Most of the functionality of this library is based on the VL53L0X API provided by ST (STSW-IMG005),
// and some of the explanatory comments are quoted or paraphrased from the API source code, API user manual (UM2039),
// and the VL53L0X datasheet.

//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39 | STM32F103-M70|103RCT6
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------+
//| nIRQ(Pin22)     |           |           |           | PC13          |<--               | <--          |<--
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------+
//| nSHUT(Pin7)*    |           |           |           | PB4           |PA4               | PB11         |PC4
//+-----------------+-----------+-----------+-----------+---------------+------------------+--------------
// * Not USED

#define VL53L0X_I2CADDR_8BIT 0x52 //(0x29 <<1) //   0b0101001

extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;

extern void delayms(uint32_t ms);
extern void Init_SysTick(int resolutionInUsec);
extern unsigned long millis();

void stmVL53L0X_writeReg(uint8_t reg, uint8_t value);
void stmVL53L0X_writeReg16Bit(uint8_t reg, uint16_t value);
void stmVL53L0X_writeReg32Bit(uint8_t reg, uint32_t value);
uint8_t stmVL53L0X_readReg(uint8_t reg);
uint16_t stmVL53L0X_readReg16Bit(uint8_t reg);
uint32_t stmVL53L0X_readReg32Bit(uint8_t reg);

struct _VL53L0X{

    uint8_t last_status; // status of last I2C transmission
    // TCC: Target CentreCheck
    // MSRC: Minimum Signal Rate Check
    // DSS: Dynamic Spad Selection
    uint8_t i2cAddress8;
    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;
    uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
    uint32_t measurement_timing_budget_us;
 } VL53L0Xmodule;

#define VL53L0X_SIGMA_ESTIMATE_MAX_VALUE 65535 //eq. to 655.35mm

  // register addresses from API vl53l0x_device.h (ordered as listed there)
enum regAddr {
      SYSRANGE_START                              = 0x00,


      SYSTEM_THRESH_HIGH                          = 0x0C,
      SYSTEM_THRESH_LOW                           = 0x0E,

      SYSTEM_SEQUENCE_CONFIG                      = 0x01,
      SYSTEM_RANGE_CONFIG                         = 0x09,
      SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

      SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

      GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

      SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

      //Result Registers
      RESULT_INTERRUPT_STATUS                     = 0x13,
      RESULT_RANGE_STATUS                         = 0x14,

      //RESULT_CORE_PAGE                            = 0x01, //added
      RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
      RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
      RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
      RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
      RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

      //Algorithm Registers
      ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

      I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

      //Check Limit Registers
      MSRC_CONFIG_CONTROL                         = 0x60,

      PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
      PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
      PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
      PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

      FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
      FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
      FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
      FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

      PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
      PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

      //Pre Range Registers
      PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

      SYSTEM_HISTOGRAM_BIN                        = 0x81,
      HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
      HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

      FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
      CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

      MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

      SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
      IDENTIFICATION_MODEL_ID                     = 0xC0, //WHO_AM_I (SHOULD BE 0xEE)
      IDENTIFICATION_REVISION_ID                  = 0xC2, //==10

      OSC_CALIBRATE_VAL                           = 0xF8,

      GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

      GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
      DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E, //0x14E
      DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F, //0x14F
      POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

      VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89, //I2C

      ALGO_PHASECAL_LIM                           = 0x30,//ALGO_PHASECAL_LIM
      ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,//ALGO_PHASECAL_PHYSICAL_CONFIG_TIMEOUT
    };

//REGISTER BITS
//SYSRANGE_START Reg(0x00)
	#define SYSRANAGE_MODE_MASK		0x0F //#define VL53L0X_REG_SYSRANGE_MODE_MASK          0x0F
	//Bit 0
		#define VL53L0X_SYSRANGE_REG_MODE_BIT0_START_STOP    0x01 // bit 0 in SYSRANGE_START write 1 toggle state in continuous mode and arm next shot in single shot mode.
	//Bit 1
		#define VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT    0x00 // bit 1 write 0 in SYSRANGE_START set single shot mode
 	//Bit 1
		#define VL53L0X_SYSRANGE_REG_MODE_BIT1_BACKTOBACK    0x02 // bit 1 write 1 in #VL53L0X_REG_SYSRANGE_START set back-to-back  operation mode
 	//Bit 2
		#define VL53L0X_SYSRANGE_REG_MODE_BIT2_TIMED         0x04 // bit 2 write 1 in #VL53L0X_REG_SYSRANGE_START set timed operation  mode
	//Bit 3
 	 	#define VL53L0X_REG_SYSRANGE_MODE_HISTOGRAM     0x08 // bit 3 write 1 in #VL53L0X_REG_SYSRANGE_START set histogram operation mode

//For SYSTEM_INTERRUPT_CONFIG_GPIO reg(0x000A)
	#define VL53L0X_INTERRUPT_GPIO_DISABLED_BIT0  0x00
	#define VL53L0X_INTERRUPT_GPIO_LEVEL_LOW_BIT1 0x01
	#define VL53L0X_INTERRUPT_GPIO_LEVEL_HIGH_BIT2  0x02
	#define VL53L0X_INTERRUPT_GPIO_OUT_OF_WINDOW_BIT3 0x03
	#define VL53L0X_INTERRUPT_GPIO_NEW_SAMPLE_READY_BIT4  0x04

//For POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,
#define POWERMODE_STANDBY 0x00 //set power mode to standby level 1
#define POWERMODE_STANDBY 0x01 //set power mode to idle level 1
//===============
#define SPEED_OF_LIGHT 2997 //speed of light per 1e-10 sec


//=========
typedef enum  { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;

struct SequenceStepEnables
{
  boolean tcc, msrc, dss, pre_range, final_range;
};

struct SequenceStepTimeouts
{
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
};

typedef enum {APP_DEFAULT, APP_LONGRANGE_DEFAULT,APP_DEFAULTRANGE_HIGH_SPEED,APP_DEFAULTRANGE_HIGH_ACCURACY,APP_LONGRANGE_HIGH_SPEED,APP_LONGRANGE_HIGH_ACCURACY} e_AppOptions;

void stmVL53L0X_writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
void stmVL53L0X_readMulti(uint8_t reg, uint8_t * dst, uint8_t count);

bool stmVL53L0X_setSignalRateLimit(float limit_Mcps);
float stmVL53L0X_getSignalRateLimit(void);

bool stmVL53L0X_setMeasurementTimingBudget(uint32_t budget_us);
uint32_t stmVL53L0X_getMeasurementTimingBudget(void);

bool stmVL53L0X_setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
uint8_t stmVL53L0X_getVcselPulsePeriod(vcselPeriodType type);


//void startContinuous(uint32_t period_ms = 0);
//void stopContinuous(void);
uint16_t stmVL53L0X_readRangeContinuousMillimeters(void);
uint16_t stmVL53L0X_readRangeSingleMillimeters(void);

inline void stmVL53L0X_setTimeout(uint16_t timeout) {
	VL53L0Xmodule.io_timeout = timeout;
}
inline uint16_t stmVL53L0X_getTimeout(void) { return VL53L0Xmodule.io_timeout; }
//bool timeoutOccurred(void);
bool stmVL53L0X_GetSpadInfo(uint8_t * count, bool * type_is_aperture);

void stmVL53L0X_getSequenceStepEnables(struct SequenceStepEnables * enables);
void stmVL53L0X_getSequenceStepTimeouts(struct SequenceStepEnables * enables, struct SequenceStepTimeouts * timeouts);

bool stmVL53L0X_performSingleRefCalibration(uint8_t vhv_init_byte);

uint16_t stmVL53L0X_decodeTimeout(uint16_t reg_val);
static uint16_t stmVL53L0X_encodeTimeout(uint16_t timeout_mclks);
static uint32_t stmVL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t stmVL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

 //void setAddress(uint8_t new_addr);
 //inline uint8_t getAddress(void) { return VL53L0Xmodule.i2cAddress8; }
 //bool init(bool io_2v8 = true);

// Record the current time to check an upcoming timeout against
#define stmVL53StartTimeout() (VL53L0Xmodule.timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define stmVL53CheckTimeoutExpired() (VL53L0Xmodule.io_timeout > 0 && ((uint16_t)millis() - VL53L0Xmodule.timeout_start_ms) > VL53L0Xmodule.io_timeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define stmVL53DecodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define stmVL53EncodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define stmVL53CalcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

//======================== i2c ==================


// Write an 8-bit register
stmVL53L0X_writeReg(u8 reg, u8 val){
 	stmI2cSendbuf[0] = reg;
 	stmI2cSendbuf[1] = val;
 	stm_I2C_SendBurst(VL53L0X_I2CADDR_8BIT,&stmI2cSendbuf[0], 2);
 }


// Write a 16-bit register
void stmVL53L0X_writeReg16Bit(uint8_t reg, uint16_t value)
{
 	stmI2cSendbuf[0] = reg;
 	stmI2cSendbuf[1] = (value >> 8) & 0xff;// value high byte
 	stmI2cSendbuf[2] = (value  & 0xff);    // value low byte
 	stm_I2C_SendBurst(VL53L0X_I2CADDR_8BIT,&stmI2cSendbuf[0], 3);
}

// Write a 32-bit register
void stmVL53L0X_writeReg32Bit(uint8_t reg, uint32_t value)
{
 	stmI2cSendbuf[0] = reg;
 	stmI2cSendbuf[1] = (value >> 24) & 0xff;// value high byte
 	stmI2cSendbuf[2] = (value >> 16) & 0xff;
 	stmI2cSendbuf[3] = (value >> 8) & 0xff;// value high byte
 	stmI2cSendbuf[4] = (value  & 0xff);    // value low byte
 	stm_I2C_SendBurst(VL53L0X_I2CADDR_8BIT,&stmI2cSendbuf[0], 5);
}
// Write an arbitrary number of bytes from the given array to the sensor, starting at the given register
void stmVL53L0X_writeMulti(uint8_t reg, uint8_t const * src, uint8_t count)
{
	stmI2cSendbuf[0] = reg;
	memcpy(&stmI2cSendbuf[1], src, count);
 	stm_I2C_SendBurst(VL53L0X_I2CADDR_8BIT,&stmI2cSendbuf[0], count+1);
}

// Read an 8-bit register
u8 stmVL53L0X_readReg(u8 reg){
	static unsigned char retVal;
	stm_I2C_ReceiveBurstWithRestartCondition(VL53L0X_I2CADDR_8BIT, reg, &retVal, 1);
	return retVal;
}

// Read a 16-bit register
uint16_t stmVL53L0X_readReg16Bit(uint8_t reg)
{
	unsigned short retVal;

	stm_I2C_ReceiveBurstWithRestartCondition(VL53L0X_I2CADDR_8BIT, reg, &stmI2cRecvbuf[0], 2);
	retVal  = (uint16_t)stmI2cRecvbuf[0];
	retVal <<= 8; // value high byte
	retVal |=  stmI2cRecvbuf[1];      // value low byte

  return retVal;
}

// Read a 32-bit register
uint32_t stmVL53L0X_readReg32Bit(uint8_t reg)
{
  uint32_t retVal;

  stm_I2C_ReceiveBurstWithRestartCondition(VL53L0X_I2CADDR_8BIT, reg, &stmI2cRecvbuf[0], 4);
  retVal = stmI2cRecvbuf[0]; retVal <<=8;
  retVal |= stmI2cRecvbuf[1]; retVal <<=8;
  retVal |= stmI2cRecvbuf[2]; retVal <<=8;
  retVal |= stmI2cRecvbuf[3];
  /*
	retVal  = ((uint32_t)stmI2cRecvbuf[0]) << 24; // value high byte
	retVal  = ((uint32_t)stmI2cRecvbuf[1]) << 16; // value high byte
	retVal  = ((uint16_t)stmI2cRecvbuf[2]) << 8; // value high byte
	retVal |=  stmI2cRecvbuf[3];      // value low byte
	*/

	return retVal;
}

// Read an arbitrary number of bytes from the sensor, starting at the given register, into the given array
void stmVL53L0X_readMulti(uint8_t reg, uint8_t * dst, uint8_t count)
{
	stm_I2C_ReceiveBurstWithRestartCondition(VL53L0X_I2CADDR_8BIT, reg, dst, count);
}
//========================
void stmVL53L0X_setAddress7(uint8_t new_i2cAddr7)
{
  stmVL53L0X_writeReg(I2C_SLAVE_DEVICE_ADDRESS, new_i2cAddr7 & 0x7F);
  VL53L0Xmodule.i2cAddress8 = (new_i2cAddr7 << 1);
}

void stmVL53L0X_getBasicInfo(void){
	uint8_t retVal;
	 uint32_t val32;
	  // Get some basic information about the sensor
	   retVal = stmVL53L0X_readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD);
	   printf("PRE_RANGE_CONFIG_VCSEL_PERIOD=%u(",retVal);
	   printf("decodedValue = %u)\r\n", stmVL53DecodeVcselPeriod(retVal));

	   retVal = stmVL53L0X_readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD);
	   printf("FINAL_RANGE_CONFIG_VCSEL_PERIOD=%u(",retVal);
	   printf("decodedValue = %u)\r\n", stmVL53DecodeVcselPeriod(retVal));

	   val32 = stmVL53L0X_readReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD);
	   printf("System Inter-Measurement Period = %u ms.\r\n", val32);
}




// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool stmVL53L0X_setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  stmVL53L0X_writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

// Get the return signal rate limit check value in MCPS
float stmVL53L0X_getSignalRateLimit(void)
{
  return (float)stmVL53L0X_readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool stmVL53L0X_setMeasurementTimingBudget(uint32_t budget_us)
{
	struct SequenceStepEnables enables;
	struct SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  stmVL53L0X_getSequenceStepEnables(&enables);
  stmVL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
    		stmVL53L0X_timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    stmVL53L0X_writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    		stmVL53L0X_encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    VL53L0Xmodule.measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds based on VL53L0X_get_measurement_timing_budget_micro_seconds() in us.
uint32_t stmVL53L0X_getMeasurementTimingBudget(void)
{
	struct SequenceStepEnables enables;
	struct SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  stmVL53L0X_getSequenceStepEnables(&enables);
  stmVL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  VL53L0Xmodule.measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool stmVL53L0X_setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = stmVL53EncodeVcselPeriod(period_pclks);

  struct SequenceStepEnables enables;
  struct SequenceStepTimeouts timeouts;

  stmVL53L0X_getSequenceStepEnables(&enables);
  stmVL53L0X_getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        stmVL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        stmVL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        stmVL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        stmVL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    stmVL53L0X_writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    stmVL53L0X_writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
    		stmVL53L0X_timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    stmVL53L0X_writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    		stmVL53L0X_encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
    		stmVL53L0X_timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    stmVL53L0X_writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        stmVL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        stmVL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        stmVL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        stmVL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        stmVL53L0X_writeReg(0xFF, 0x01);
        stmVL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x30);
        stmVL53L0X_writeReg(0xFF, 0x00);
        break;

      case 10:
        stmVL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        stmVL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        stmVL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        stmVL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        stmVL53L0X_writeReg(0xFF, 0x01);
        stmVL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x20);
        stmVL53L0X_writeReg(0xFF, 0x00);
        break;

      case 12:
        stmVL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        stmVL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        stmVL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        stmVL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        stmVL53L0X_writeReg(0xFF, 0x01);
        stmVL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x20);
        stmVL53L0X_writeReg(0xFF, 0x00);
        break;

      case 14:
        stmVL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        stmVL53L0X_writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        stmVL53L0X_writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        stmVL53L0X_writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        stmVL53L0X_writeReg(0xFF, 0x01);
        stmVL53L0X_writeReg(ALGO_PHASECAL_LIM, 0x20);
        stmVL53L0X_writeReg(0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    stmVL53L0X_writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
    		stmVL53L0X_timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    stmVL53L0X_writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    		stmVL53L0X_encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  stmVL53L0X_setMeasurementTimingBudget(VL53L0Xmodule.measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = stmVL53L0X_readReg(SYSTEM_SEQUENCE_CONFIG);
  stmVL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  stmVL53L0X_performSingleRefCalibration(0x0);
  stmVL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t stmVL53L0X_getVcselPulsePeriod(vcselPeriodType type)
{
	uint8_t period;
  if (type == VcselPeriodPreRange)
  {
	  period = stmVL53L0X_readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD);
    return stmVL53DecodeVcselPeriod(period);
  }
  else if (type == VcselPeriodFinalRange)
  {
	  period = stmVL53L0X_readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD);
    return stmVL53DecodeVcselPeriod(period);
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void stmVL53L0X_startContinuous(uint32_t period_ms)
{
	printf("VL53L0X> StartContinuous...\r\n");
  stmVL53L0X_writeReg(0x80, 0x01);
  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x00, 0x00);
  stmVL53L0X_writeReg(0x91, VL53L0Xmodule.stop_variable);
  stmVL53L0X_writeReg(0x00, 0x01);
  stmVL53L0X_writeReg(0xFF, 0x00);
  stmVL53L0X_writeReg(0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
    uint16_t osc_calibrate_val = stmVL53L0X_readReg16Bit(OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    stmVL53L0X_writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    stmVL53L0X_writeReg(SYSRANGE_START, VL53L0X_SYSRANGE_REG_MODE_BIT2_TIMED); // VL53L0X_SYSRANGE_REG_MODE_BIT2_TIMED = 0x04
  }
  else
  {
    // continuous back-to-back mode
    stmVL53L0X_writeReg(SYSRANGE_START, VL53L0X_SYSRANGE_REG_MODE_BIT1_BACKTOBACK); // VL53L0X_SYSRANGE_REG_MODE_BIT1_BACKTOBACK = 0x02
  }

  //stmVL53L0X_writeReg(SYSRANGE_START,VL53L0X_SYSRANGE_REG_MODE_BIT0_START_STOP); //ADDED
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void stmVL53L0X_stopContinuous(void)
{
  stmVL53L0X_writeReg(SYSRANGE_START, 0x01); // Set Bit0 to toggle for stop

  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x00, 0x00);
  stmVL53L0X_writeReg(0x91, 0x00);
  stmVL53L0X_writeReg(0x00, 0x01);
  stmVL53L0X_writeReg(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active.
//(readRangeSingleMillimeters() also calls this function after starting a single-shot range measurement.
uint16_t stmVL53L0X_readRangeContinuousMillimeters(void)
{
	u8 retVal;
	u8 rangeData[14];
	uint16_t range;

	stmVL53StartTimeout();

	while ((stmVL53L0X_readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0){ //0x40 returned ??? TBD
		if (stmVL53CheckTimeoutExpired())
		{
			VL53L0Xmodule.did_timeout = true;
			printf("VL50L0X> ReadRangeContinuousMillimeters -> Timeout.\r\n");
			return 65535;
		}
	}

	// assumptions: Linearity Corrective Gain is 1000 (default);
	// fractional ranging is not enabled

	range = stmVL53L0X_readReg16Bit(RESULT_RANGE_STATUS + 10);
	//printf("Range=%u\r\n",range);
	/*stmVL53L0X_readMulti(RESULT_RANGE_STATUS, &rangeData[0], 14);
	u8 devErr = (rangeData[0] & 0x78) >> 3;
	if(devErr == 0){
		printf("Data OK!\r\n");
		printf("Range=%u\r\n",(unsigned short)((unsigned short)rangeData[10]<<8) | rangeData[11]);
	}
	else{
		printf("devErrStatus=0x%02x\r\n",devErr);
		if(devErr == 0x01) printf("devErrStatus=0x01 (VCSEL Continuity Test Failure.)\r\n");
		if(devErr == 0x02) printf("devErrStatus=0x02 (VCSEL Watchdog Test Failure.)\r\n");
		if(devErr == 0x03) printf("devErrStatus=0x03 (No VHV Value Found.)\r\n");
		if(devErr == 0x04) printf("devErrStatus=0x04 (MSRC NO Target.)\r\n");

	}
	return (unsigned short)((unsigned short)rangeData[10]<<8) | rangeData[11];
	*/
	return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t stmVL53L0X_readRangeSingleMillimeters(void)
{
	unsigned char expired;

  stmVL53L0X_writeReg(0x80, 0x01);
  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x00, 0x00);
  stmVL53L0X_writeReg(0x91, VL53L0Xmodule.stop_variable);
  stmVL53L0X_writeReg(0x00, 0x01);
  stmVL53L0X_writeReg(0xFF, 0x00);
  stmVL53L0X_writeReg(0x80, 0x00);

  stmVL53L0X_writeReg(SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  stmVL53StartTimeout();
  //printf("StartTimer @ %d\r\n",VL53L0Xmodule.timeout_start_ms);

  while (stmVL53L0X_readReg(SYSRANGE_START) & 0x01) {
	 expired =  (VL53L0Xmodule.io_timeout > 0 && ((uint16_t)millis() - VL53L0Xmodule.timeout_start_ms) > VL53L0Xmodule.io_timeout);
     if(expired){ //if (stmVL53CheckTimeoutExpired())  {
    	VL53L0Xmodule.did_timeout = true;
    	printf("timeout @%d\r\n", (uint16_t)millis());
    	return 65535;
    }
  }
  return stmVL53L0X_readRangeContinuousMillimeters();
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool stmVL53L0X_timeoutOccurred()
{
  bool tmp = VL53L0Xmodule.did_timeout;
  VL53L0Xmodule.did_timeout = false;
  return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool stmVL53L0X_GetSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  stmVL53L0X_writeReg(0x80, 0x01);
  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x00, 0x00);

  stmVL53L0X_writeReg(0xFF, 0x06);
  stmVL53L0X_writeReg(0x83, stmVL53L0X_readReg(0x83) | 0x04);
  stmVL53L0X_writeReg(0xFF, 0x07);
  stmVL53L0X_writeReg(0x81, 0x01);

  stmVL53L0X_writeReg(0x80, 0x01);

  stmVL53L0X_writeReg(0x94, 0x6b);
  stmVL53L0X_writeReg(0x83, 0x00);
  stmVL53StartTimeout();
  while (stmVL53L0X_readReg(0x83) == 0x00)
  {
    if (stmVL53CheckTimeoutExpired()) { return false; }
  }
  stmVL53L0X_writeReg(0x83, 0x01);
  tmp = stmVL53L0X_readReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  stmVL53L0X_writeReg(0x81, 0x00);
  stmVL53L0X_writeReg(0xFF, 0x06);
  stmVL53L0X_writeReg(0x83, stmVL53L0X_readReg(0x83)  & ~0x04);
  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x00, 0x01);

  stmVL53L0X_writeReg(0xFF, 0x00);
  stmVL53L0X_writeReg(0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void stmVL53L0X_getSequenceStepEnables(struct SequenceStepEnables * enables)
{
  uint8_t sequence_config = stmVL53L0X_readReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void stmVL53L0X_getSequenceStepTimeouts(struct SequenceStepEnables * enables, struct SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = stmVL53L0X_getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = stmVL53L0X_readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
		  stmVL53L0X_timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
		  stmVL53L0X_decodeTimeout(stmVL53L0X_readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
		  stmVL53L0X_timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = stmVL53L0X_getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
		  stmVL53L0X_decodeTimeout(stmVL53L0X_readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
		  stmVL53L0X_timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t stmVL53L0X_decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t stmVL53L0X_encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t stmVL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = stmVL53CalcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t stmVL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = stmVL53CalcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// based on VL53L0X_perform_single_ref_calibration()
bool stmVL53L0X_performSingleRefCalibration(uint8_t vhv_init_byte)
{
	unsigned char res8 = 0;

  stmVL53L0X_writeReg(SYSRANGE_START, VL53L0X_SYSRANGE_REG_MODE_BIT0_START_STOP | vhv_init_byte); // VL53L0X_SYSRANGE_REG_MODE_BIT0_START_STOP = 0x01 Toggle. //START? or STOP?

  stmVL53StartTimeout();
  while ((res8 & 0x07) == 0)
  {
	  res8 = stmVL53L0X_readReg(RESULT_INTERRUPT_STATUS);
	  printf("stmVL53L0X_performSingleRefCalibration> RESULT_INTERRUPT_STATUS = 0x%02x\r\n",res8);
	  if (stmVL53CheckTimeoutExpired()) {
		  return false;
	  }
  }

  stmVL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  stmVL53L0X_writeReg(SYSRANGE_START, 0x00); //VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT == 0x00

  return true;
}
void stmLib_ShowRange(unsigned short range, bool normal1_inverted0){
	int i,j, invRange;
	char strbuf[100];
	if(normal1_inverted0){
		j = range/20;
		if(j>100) 	j= 100;
		for(i=0;i<j;i++){
			PutChar(&strbuf[i],'.');
		}
		PutChar(&strbuf[i],0x00);
		printf("%s %u\r\n",strbuf,range);
	}else{
		if(range > 1023) range = 1024; //Sanity Check
		invRange = 1023 - range;
		j = invRange/20;
		if(j>100) 	j= 100;
		if(j == 0){
			PutChar(&strbuf[0],'.');
			PutChar(&strbuf[1],0x00);
			printf("%s %u\r\n",strbuf,range);
		}else{
			for(i=0;i<j;i++){
				PutChar(&strbuf[i],'.');
			}
			PutChar(&strbuf[i],0x00);
			printf("%s %u\r\n",strbuf,range);
		}
	}
}

//PC13 = nIRQ
#if(PROCESSOR == PROCESSOR_STM32F103C8T6)
void stmVL53L0X_PC13_IRQpin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  /* Enable GPIOC clock */
 	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	  /* Enable and set EXTI Line8 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  /* Configure PC13 pin as input floating */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
	  /* Connect EXTI Line13 to PC13 pin */
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

	  /* Configure EXTI Line13 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
}
#else
#endif

extern uint8_t g_irq;
//PC13
/*
void EXTI15_10_IRQHandler()
{
	//struct Ap3216_Module *data = data_;
	if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
		// Toggle LED0
		GPIO_ToggleBits(GPIOC, GPIO_Pin_14); //ULED -103 (PC14)
		g_irq = 1;
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
}
*/
// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.

bool stmVL53L0X_init(bool io_2v8, e_AppOptions AppOptions)
{
	  uint8_t spad_count;
	  uint32_t val32;
	  bool spad_type_is_aperture;
	  uint8_t i,retVal,retVal2;
	  uint8_t ref_spad_map[6];

	  VL53L0Xmodule.i2cAddress8 = VL53L0X_I2CADDR_8BIT;

	  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	  if (io_2v8) {
		  retVal = stmVL53L0X_readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
		  stmVL53L0X_writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, retVal | 0x01); // set bit0
		  printf("Set I2C IO Voltage of 2.8V.\r\n");
	  }

	  //Install internal timer.
	  Init_SysTick(1000); //install 1msec tick for the local timer.
	  VL53L0Xmodule.did_timeout = false;
	  stmVL53L0X_setTimeout(500); //500msec 	  //VL53L0Xmodule.io_timeout = 0; // no timeout

	  //Check Product ID and Revision ID
	  retVal = stmVL53L0X_readReg(IDENTIFICATION_MODEL_ID);
	  if(retVal == 0xEE){
		  printf("IDENTIFICATION_MODEL_ID = 0xEE(Found VL53L0X)\r\n");
	  	  retVal = stmVL53L0X_readReg(IDENTIFICATION_REVISION_ID);
	  	  printf("IDENTIFICATION_REVISION_ID = 0x%02x\r\n", retVal);
	  }else
		  printf("IDENTIFICATION_MODEL_ID = 0x%02x(Incorrect device(Should be 0xEE).)\r\n", retVal);

	  //Issue SoftReset
	  stmVL53L0X_writeReg(SOFT_RESET_GO2_SOFT_RESET_N, 0x01); //Reset Device
	  delayms(100);

	  //Check 2.8V IO
	  retVal = stmVL53L0X_readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
	  stmVL53L0X_writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, retVal | 0x01); // set bit0

	  if((retVal & ~0xfe) == 0x00) printf("I2C IO Voltage = 1.8V.\r\n");
	  if((retVal & ~0xfe) == 0x01) printf("I2C IO Voltage = 2.8V.\r\n");

	  //Read registers for debugging.
	  /*
	  for(i=0;i<0xF8;i++){
		  retVal = stmVL53L0X_readReg(i);
		  if(i==0x8a)
			  printf("Reg[0x8a]=%02x[Should be 0x29]\r\n",retVal);
		  else
			  printf("Reg[%02x]=%02x\r\n",i,retVal);
	  }
	  */

	  printf("Set I2C standard mode.\r\n");
	  stmVL53L0X_writeReg(0x88, 0x00);

	  stmVL53L0X_writeReg(0x80, 0x01);
	  stmVL53L0X_writeReg(0xFF, 0x01);
	  stmVL53L0X_writeReg(0x00, 0x00);
	  VL53L0Xmodule.stop_variable = stmVL53L0X_readReg(0x91);
	  stmVL53L0X_writeReg(0x00, 0x01);
	  stmVL53L0X_writeReg(0xFF, 0x00);
	  stmVL53L0X_writeReg(0x80, 0x00);


	  // Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	  retVal = stmVL53L0X_readReg(MSRC_CONFIG_CONTROL);
	  stmVL53L0X_writeReg(MSRC_CONFIG_CONTROL, retVal | 0x12);

	  // Set final range signal rate limit to 0.25 MCPS (million counts per second)
	  stmVL53L0X_setSignalRateLimit(0.25);//2)
	  stmVL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);


	  printf("VL53L0X> Data Init Done.\r\n");

	  //=====StaticInit == begin
	  //Get spad info
	  if (!stmVL53L0X_GetSpadInfo(&spad_count, &spad_type_is_aperture)) {
		  printf("Fail to get SpadInfo.\r\n");
		  return false;
	  }

	  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	  // the API, but the same data seems to be more easily readable from
	  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there

	  stmVL53L0X_readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
	  printf("VL53L0X> Set_reference_spads..\r\n");
	  stmVL53L0X_writeReg(0xFF, 0x01);
	  stmVL53L0X_writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	  stmVL53L0X_writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	  stmVL53L0X_writeReg(0xFF, 0x00);
	  stmVL53L0X_writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	  uint8_t spads_enabled = 0;

	  for (i = 0; i < 48; i++){
		  if (i < first_spad_to_enable || spads_enabled == spad_count) {
			  // This bit is lower than the first one that should be enabled, or
			  // (reference_spad_count) bits have already been enabled, so zero this bit
			  ref_spad_map[i / 8] &= ~(1 << (i % 8));
		  } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)  {
			  spads_enabled++;
		  }
	  }

	  stmVL53L0X_writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
	  /*stmVL53L0X_writeReg(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map[0]);
	  stmVL53L0X_writeReg(GLOBAL_CONFIG_SPAD_ENABLES_REF_1, ref_spad_map[1]);
	  stmVL53L0X_writeReg(GLOBAL_CONFIG_SPAD_ENABLES_REF_2, ref_spad_map[2]);
	  stmVL53L0X_writeReg(GLOBAL_CONFIG_SPAD_ENABLES_REF_3, ref_spad_map[3]);
	  stmVL53L0X_writeReg(GLOBAL_CONFIG_SPAD_ENABLES_REF_4, ref_spad_map[4]);
	  stmVL53L0X_writeReg(GLOBAL_CONFIG_SPAD_ENABLES_REF_5, ref_spad_map[5]);
	  */

	  // -- VL53L0X_set_reference_spads() end

	  // -- VL53L0X_load_tuning_settings() begin
	  // DefaultTuningSettings from vl53l0x_tuning.h

  printf("Load Tuning Set\r\n");
  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x00, 0x00);

  stmVL53L0X_writeReg(0xFF, 0x00);
  stmVL53L0X_writeReg(0x09, 0x00);
  stmVL53L0X_writeReg(0x10, 0x00);
  stmVL53L0X_writeReg(0x11, 0x00);

  stmVL53L0X_writeReg(0x24, 0x01);
  stmVL53L0X_writeReg(0x25, 0xFF);
  stmVL53L0X_writeReg(0x75, 0x00);

  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x4E, 0x2C);
  stmVL53L0X_writeReg(0x48, 0x00);
  stmVL53L0X_writeReg(0x30, 0x20);

  stmVL53L0X_writeReg(0xFF, 0x00);
  stmVL53L0X_writeReg(0x30, 0x09);
  stmVL53L0X_writeReg(0x54, 0x00);
  stmVL53L0X_writeReg(0x31, 0x04);
  stmVL53L0X_writeReg(0x32, 0x03);
  stmVL53L0X_writeReg(0x40, 0x83);
  stmVL53L0X_writeReg(0x46, 0x25);
  stmVL53L0X_writeReg(0x60, 0x00);
  stmVL53L0X_writeReg(0x27, 0x00);
  stmVL53L0X_writeReg(0x50, 0x06);
  stmVL53L0X_writeReg(0x51, 0x00);
  stmVL53L0X_writeReg(0x52, 0x96);
  stmVL53L0X_writeReg(0x56, 0x08);
  stmVL53L0X_writeReg(0x57, 0x30);
  stmVL53L0X_writeReg(0x61, 0x00);
  stmVL53L0X_writeReg(0x62, 0x00);
  stmVL53L0X_writeReg(0x64, 0x00);
  stmVL53L0X_writeReg(0x65, 0x00);
  stmVL53L0X_writeReg(0x66, 0xA0);

  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x22, 0x32);
  stmVL53L0X_writeReg(0x47, 0x14);
  stmVL53L0X_writeReg(0x49, 0xFF);
  stmVL53L0X_writeReg(0x4A, 0x00);

  stmVL53L0X_writeReg(0xFF, 0x00);
  stmVL53L0X_writeReg(0x7A, 0x0A);
  stmVL53L0X_writeReg(0x7B, 0x00);
  stmVL53L0X_writeReg(0x78, 0x21);

  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x23, 0x34);
  stmVL53L0X_writeReg(0x42, 0x00);
  stmVL53L0X_writeReg(0x44, 0xFF);
  stmVL53L0X_writeReg(0x45, 0x26);
  stmVL53L0X_writeReg(0x46, 0x05);
  stmVL53L0X_writeReg(0x40, 0x40);
  stmVL53L0X_writeReg(0x0E, 0x06);
  stmVL53L0X_writeReg(0x20, 0x1A);
  stmVL53L0X_writeReg(0x43, 0x40);

  stmVL53L0X_writeReg(0xFF, 0x00);
  stmVL53L0X_writeReg(0x34, 0x03);
  stmVL53L0X_writeReg(0x35, 0x44);

  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x31, 0x04);
  stmVL53L0X_writeReg(0x4B, 0x09);
  stmVL53L0X_writeReg(0x4C, 0x05);
  stmVL53L0X_writeReg(0x4D, 0x04);

  stmVL53L0X_writeReg(0xFF, 0x00);
  stmVL53L0X_writeReg(0x44, 0x00);
  stmVL53L0X_writeReg(0x45, 0x20);
  stmVL53L0X_writeReg(0x47, 0x08);
  stmVL53L0X_writeReg(0x48, 0x28);
  stmVL53L0X_writeReg(0x67, 0x00);
  stmVL53L0X_writeReg(0x70, 0x04);
  stmVL53L0X_writeReg(0x71, 0x01);
  stmVL53L0X_writeReg(0x72, 0xFE);
  stmVL53L0X_writeReg(0x76, 0x00);
  stmVL53L0X_writeReg(0x77, 0x00);

  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x0D, 0x01);

  stmVL53L0X_writeReg(0xFF, 0x00);
  stmVL53L0X_writeReg(0x80, 0x01);
  stmVL53L0X_writeReg(0x01, 0xF8);

  stmVL53L0X_writeReg(0xFF, 0x01);
  stmVL53L0X_writeReg(0x8E, 0x01);
  stmVL53L0X_writeReg(0x00, 0x01);
  stmVL53L0X_writeReg(0xFF, 0x00);
  stmVL53L0X_writeReg(0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin
  printf("VL53L0X> SetGpioConfig for Interrupt with ActiveLow.\r\n");
  //stmVL53L0X_writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, VL53L0X_INTERRUPT_GPIO_LEVEL_LOW_BIT1 | VL53L0X_INTERRUPT_GPIO_NEW_SAMPLE_READY_BIT4);// enable data ready interrupt
  //stmVL53L0X_writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, (1<< VL53L0X_INTERRUPT_GPIO_LEVEL_LOW_BIT1) | (1<<VL53L0X_INTERRUPT_GPIO_NEW_SAMPLE_READY_BIT4));// enable data ready interrupt
  //retVal = stmVL53L0X_readReg(GPIO_HV_MUX_ACTIVE_HIGH); //// Get GPIO1 interrupt config
  stmVL53L0X_writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);//enable
  retVal = stmVL53L0X_readReg(GPIO_HV_MUX_ACTIVE_HIGH); //// Get GPIO1 interrupt config
  stmVL53L0X_writeReg(GPIO_HV_MUX_ACTIVE_HIGH, retVal & ~0x10); //active low. 4th bit = 0
  //stmVL53L0X_writeReg(GPIO_HV_MUX_ACTIVE_HIGH, 0x10); // active low//stmVL53L0X_writeReg(GPIO_HV_MUX_ACTIVE_HIGH, retVal | 0x10); // active low
  stmVL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01); //Clear Interrupt
  // -- VL53L0X_SetGpioConfig() end

  stmVL53L0X_getBasicInfo();

   VL53L0Xmodule.measurement_timing_budget_us = stmVL53L0X_getMeasurementTimingBudget();
   printf("VL53L0X> Get MeasurementTimingBudget = %dusec.\r\n", VL53L0Xmodule.measurement_timing_budget_us);

   // "Disable MSRC and TCC by default"
   // MSRC = Minimum Signal Rate Check
   // TCC = Target CentreCheck

   // -- VL53L0X_SetSequenceStepEnable() begin
   printf("VL53L0X> SetSequenceStepEnable.\r\n");
   stmVL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
   // -- VL53L0X_SetSequenceStepEnable() end

   // "Recalculate timing budget"
   printf("VL53L0X> Recalculate timing budget.\r\n");
   stmVL53L0X_setMeasurementTimingBudget(VL53L0Xmodule.measurement_timing_budget_us);
  // VL53L0X_StaticInit() end


  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())
  // -- VL53L0X_perform_vhv_calibration
   printf("VL50L0X> Perform VHV Calibration.\r\n");
  stmVL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!stmVL53L0X_performSingleRefCalibration(0x40)) {
	  printf("VL50L0X> ERROR: in Perform Single Reference Calibration.\r\n");
	  return false;
  }

  // -- VL53L0X_perform_phase_calibration
  printf("VL50L0X> Perform Phase Calibration.\r\n");
  stmVL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!stmVL53L0X_performSingleRefCalibration(0x00)) {
	  printf("VL50L0X> ERROR: in Perform Single Reference Calibration.\r\n");
	  return false;
  }

  // "restore the previous Sequence Config"
  printf("VL50L0X> Restore the previous Sequence Config.\r\n");
  stmVL53L0X_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end
  //===================== APP SELECTION =================================
  switch(AppOptions){
  case APP_DEFAULT:
	  break;
  case APP_LONGRANGE_DEFAULT: //Set for Long Range
	  stmVL53L0X_setSignalRateLimit(0.1);//-- lower the return signal rate limit (default is 0.25 MCPS)
	  stmVL53L0X_setVcselPulsePeriod(VcselPeriodPreRange,18);	  //increase laser pulse periods (defaults are 14 and 10 PCLKS)
	  stmVL53L0X_setVcselPulsePeriod(VcselPeriodFinalRange,14);
	  break;
  case APP_DEFAULTRANGE_HIGH_SPEED:	  //(2) HIGH SPEED case ============
	  stmVL53L0X_setMeasurementTimingBudget(20000); //20msec //-- reduce timing budget to 20msec(default is 33msec)
	  break;
  case APP_DEFAULTRANGE_HIGH_ACCURACY:	  //(3) HIGH ACCURACY case =========
	  stmVL53L0X_setMeasurementTimingBudget(200000);//200msec //Increase timing budget to 200msec
	  break;
  case APP_LONGRANGE_HIGH_SPEED:	  //(2) HIGH SPEED case ============
	  stmVL53L0X_setSignalRateLimit(0.1);//-- lower the return signal rate limit (default is 0.25 MCPS)
	  stmVL53L0X_setVcselPulsePeriod(VcselPeriodPreRange,18);	  //increase laser pulse periods (defaults are 14 and 10 PCLKS)
	  stmVL53L0X_setVcselPulsePeriod(VcselPeriodFinalRange,14);

	  stmVL53L0X_setMeasurementTimingBudget(20000); //20msec //-- reduce timing budget to 20msec(default is 33msec)
	  break;
  case APP_LONGRANGE_HIGH_ACCURACY:	  //(3) HIGH ACCURACY case =========
	  stmVL53L0X_setSignalRateLimit(0.1);//-- lower the return signal rate limit (default is 0.25 MCPS)
	  stmVL53L0X_setVcselPulsePeriod(VcselPeriodPreRange,18);	  //increase laser pulse periods (defaults are 14 and 10 PCLKS)
	  stmVL53L0X_setVcselPulsePeriod(VcselPeriodFinalRange,14);

	  stmVL53L0X_setMeasurementTimingBudget(200000);//200msec //Increase timing budget to 200msec
	  break;
  }
  stmVL53L0X_writeReg(SYSRANGE_START, VL53L0X_SYSRANGE_REG_MODE_BIT1_BACKTOBACK); // VL53L0X_SYSRANGE_REG_MODE_BIT1_BACKTOBACK = 0x02
  printf("VL53L0X> Init Done.\r\n");
  return true;
}

/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.
The range readings are in units of mm. */


extern unsigned char g_bI2CModuleConfigDone;

void stmVL53L0X_ConfigAndInit();
short stmVL53L0X_readRange();

void stmVL53L0X_ConfigAndInit()
{
	  e_AppOptions eAppOptions;

	  printf("VL53L0X LIDAR Proximity Sensor Driver Test with I2C(7-bit Addr=0x29)\r\n");

	  if(!g_bI2CModuleConfigDone){
#if (PROCESSOR == STM32F407VZT6)
		  gI2Cx = I2C2;
#else
		  gI2Cx = I2C1;
#endif
		  printf("I2C Init with 400Kbps...");
		  stm_I2C_Init(gI2Cx,400000);//400Kbps
		  g_bI2CModuleConfigDone = 1;
		  printf("Done.\r\n");
	  }

	  //We temporay do not use interrupt.
	  //stmVL53L0X_PC13_IRQpin_Setup();

	  eAppOptions = APP_LONGRANGE_HIGH_ACCURACY;
	  stmVL53L0X_init(1,eAppOptions);//  sensor.init();
	  stmVL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01); //Clear Interrupt

	  // Start continuous back-to-back mode (take readings as fast as possible).
	  // To use continuous timed mode instead,
	  // provide a desired inter-measurement period in ms (e.g. sensor.startContinuous(100)).
	  stmVL53L0X_stopContinuous();
	  stmVL53L0X_setTimeout(500); //500msec - Not need for the continuous back-to-back mode
	  stmVL53L0X_startContinuous(0);// 0 == back-to-back ... uint32_t period_ms)
}

short stmVL53L0X_readRange()
{
	short range;

	//range = stmVL53L0X_readRangeContinuousMillimeters();//Not Working???
	range = stmVL53L0X_readRangeSingleMillimeters();

	if (stmVL53L0X_timeoutOccurred()) {
		printf(" TIMEOUT");
		return -1; //err
	}
	else{
		stmLib_ShowRange(range,1); //1 = Normal
		return range;
	}

}
void stmVL53L0X_loop()
{
	  int error;
	  int i=0;
	  float dT; //double dT;
	  uint8_t swap;
	  //union accel_t_gyro_union accel_t_gyro;
	  char str[100];
	  unsigned short range;
	  e_AppOptions eAppOptions;

	  printf("VL53L0X LIDAR Proximity Sensor Driver Test with I2C(7-bit Addr=0x29)\r\n");

	  if(!g_bI2CModuleConfigDone){
#if (PROCESSOR == STM32F407VZT6)
  		  gI2Cx = I2C2;
#else
		  gI2Cx = I2C1;
#endif
		  printf("I2C Init with 400Kbps...");
		  stm_I2C_Init(gI2Cx,400000);//400Kbps
		  g_bI2CModuleConfigDone = 1;
		  printf("Done.\r\n");
	  }

	  stmVL53L0X_ConfigAndInit(); //continuous back-to-back mode and No IRQ.

	  while(1){
		//if(g_irq){
			stmVL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01); //Clear Interrupt

			stmVL53L0X_readRange();

			if(g_irq){
				g_irq = 0;
				stmVL53L0X_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01); //Clear Interrupt
			}
			delayms(100);
		//}
	  }
}
