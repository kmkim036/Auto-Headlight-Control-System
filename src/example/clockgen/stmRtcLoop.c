
//ISL1208 Real-Time Clock

#include "yInc.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
//#include "stm32f10x_systick.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rtc.h"
#endif
#include "misc.h"

extern volatile uint32_t g_tickCnt;

//#include "lwipopts.h"
//========== differs from DS1307 ========================
#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurst(unsigned char SlaveAddress, unsigned char *buf, unsigned char nbyte);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);

extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c


/** Constants */
#define SECONDS_PER_DAY       86400L  ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000 946684800  ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

/** Constants */
#define SECONDS_PER_DAY       86400L  ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000 946684800  ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

#define RTC_ALARM_RESETVALUE_REGISTER    (uint16_t)0xFFFF
#define RTC_ALARM_RESETVALUE             0xFFFFFFFFU

unsigned char DateTime_dayOfTheWeek(struct _DateTime *dt);

extern char daysOfTheWeek [7][14];
extern const unsigned char daysInMonth [];

/**************************************************************************/
/*!
    @brief  Given a date, return number of days since 2000/01/01, valid for 2001..2099
    @param y Year
    @param m Month
    @param d Day
    @return Number of days
*/
/**************************************************************************/
extern unsigned short date2days(unsigned short y, unsigned char m, unsigned char d) ;

/**************************************************************************/
/*!
    @brief  Given a number of days, hours, minutes, and seconds, return the total seconds
    @param days Days
    @param h Hours
    @param m Minutes
    @param s Seconds
    @return Number of seconds total
*/
/**************************************************************************/
extern long time2long(unsigned short days, unsigned char h, unsigned char m, unsigned char s) ;

/**************************************************************************/
/*!
    @brief  DateTime constructor from unixtime
    @param t Initial time in seconds since Jan 1, 1970 (Unix time)
*/
/**************************************************************************/
extern DateTime_Time (struct _DateTime *dt, unsigned int t);
/**************************************************************************/
/*!
    @brief  DateTime constructor from Y-M-D H:M:S
    @param year Year, 2 or 4 digits (year 2000 or higher)
    @param month Month 1-12
    @param day Day 1-31
    @param hour 0-23
    @param min 0-59
    @param sec 0-59
*/
/**************************************************************************/
extern DateTime_DateTime (struct _DateTime *dt, unsigned short year, unsigned char month, unsigned char day, unsigned char hour, unsigned char min, unsigned char sec) ;
/**************************************************************************/
/*!
    @brief  Convert a string containing two digits to unsigned char, e.g. "09" returns 9
    @param p Pointer to a string containing two digits
*/
/**************************************************************************/
extern  unsigned char conv2d(const char* p) ;

/**************************************************************************/
/*!
    @brief  A convenient constructor for using "the compiler's time":
            DateTime now (__DATE__, __TIME__);
            NOTE: using F() would further reduce the RAM footprint, see below.
    @param date Date string, e.g. "Dec 26 2009"
    @param time Time string, e.g. "12:34:56"
*/
/**************************************************************************/
extern DateTime_DateTimeConfig (struct _DateTime *dt, const char* date, const char* time) ;

/**************************************************************************/
/*!
    @brief  Return DateTime in based on user defined format.
    @param buffer: array of char for holding the format description and the formatted DateTime.
                   Before calling this method, the buffer should be initialized by the user with
                   a format string, e.g. "YYYY-MM-DD hh:mm:ss". The method will overwrite
                   the buffer with the formatted date and/or time.
    @return a pointer to the provided buffer. This is returned for convenience,
            in order to enable idioms such as Serial.println(now.toString(buffer));
*/
/**************************************************************************/

extern char* DateTime_toString(struct _DateTime *dt, char* buffer);

/**************************************************************************/
/*!
    @brief  Return the day of the week for this object, from 0-6.
    @return Day of week 0-6 starting with Sunday, e.g. Sunday = 0, Saturday = 6
*/
/**************************************************************************/
extern unsigned char DateTime_dayOfTheWeek(struct _DateTime *dt) ;

/**************************************************************************/
/*!
    @brief  Return unix time, seconds since Jan 1, 1970.
    @return Number of seconds since Jan 1, 1970
*/
/**************************************************************************/
extern unsigned int DateTime_unixtime(struct _DateTime *dt);
/**************************************************************************/
/*!
    @brief  Convert the DateTime to seconds
    @return The object as seconds since 2000-01-01
*/
/**************************************************************************/
extern long DateTime_secondstime(struct _DateTime *dt) ;

/**************************************************************************/
/*!
    @brief  Convert a binary coded decimal value to binary. RTC stores time/date values as BCD.
    @param val BCD value
    @return Binary value
*/
/**************************************************************************/
extern unsigned char bcd2bin (unsigned char val);

/**************************************************************************/
/*!
    @brief  Convert a binary value to BCD format for the RTC registers
    @param val Binary value
    @return BCD value
*/
/**************************************************************************/
extern unsigned char bin2bcd (unsigned char val) ;
uint32_t           RTC_ReadTimeCounter(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef  RTC_WriteTimeCounter(RTC_HandleTypeDef *hrtc, uint32_t TimeCounter);
uint32_t           RTC_ReadAlarmCounter(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef  RTC_WriteAlarmCounter(RTC_HandleTypeDef *hrtc, uint32_t AlarmCounter);
HAL_StatusTypeDef  RTC_EnterInitMode(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef  RTC_ExitInitMode(RTC_HandleTypeDef *hrtc);
unsigned char      RTC_ByteToBcd2(unsigned char Value);
unsigned char      RTC_Bcd2ToByte(unsigned char Value);
unsigned char      RTC_IsLeapYear(uint16_t nYear);
void               RTC_DateUpdate(RTC_HandleTypeDef *hrtc, uint32_t DayElapsed);
unsigned char      RTC_WeekDayNum(uint32_t nYear, unsigned char nMonth, unsigned char nDay);
void 			   RTC_DateUpdate(RTC_HandleTypeDef *hrtc, uint32_t DayElapsed);
unsigned char 		RTC_IsLeapYear(uint16_t nYear);
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);


/*
 *  ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
   [..] This section provides functions allowing to initialize and configure the
         RTC Prescaler (Asynchronous), disable RTC registers Write protection,
         enter and exit the RTC initialization mode,
         RTC registers synchronization check and reference clock detection enable.
         (#) The RTC Prescaler should be programmed to generate the RTC 1Hz time base.
         (#) All RTC registers are Write protected. Writing to the RTC registers
             is enabled by setting the CNF bit in the RTC_CRL register.
         (#) To read the calendar after wakeup from low power modes (Standby or Stop)
             the software must first wait for the RSF bit (Register Synchronized Flag)
             in the RTC_CRL register to be set by hardware.
             The HAL_RTC_WaitForSynchro() function implements the above software
             sequence (RSF clear and RSF check).
 */
unsigned char stmRTC_Init(RTC_HandleTypeDef *hrtc){

  uint32_t prescaler = 0U;
  if (hrtc == NULL)
    return HAL_ERROR;

  if (hrtc->State == HAL_RTC_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    //hrtc->Lock = HAL_UNLOCKED;

    /* Initialize RTC MSP */
    //HAL_RTC_MspInit(hrtc);
  }

	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_PWR |  //enable the PWR peripheral
							RCC_APB1Periph_BKP, ENABLE); //enable RTC Backup
	PWR_BackupAccessCmd(ENABLE);

	if((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN){
		RCC_BackupResetCmd(ENABLE);
		RCC_BackupResetCmd(DISABLE);


		RCC_LSEConfig(RCC_LSE_ON); //Turn on 32768
		while((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY);
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); //Select LSE as RTC Clock Source

		//Set prescaler
		RTC_SetPrescaler(0x7FFF); //1Hz : Div128, Div256

		//en RTC
		RCC_RTCCLKCmd(ENABLE);

		RTC_WaitForSynchro();


	    /* Initialize date to 1st of January 2000 */
	    hrtc->DateToUpdate.Year = 0x00U;
	    hrtc->DateToUpdate.Month = RTC_MONTH_JANUARY;
	    hrtc->DateToUpdate.Date = 0x01U;

	    hrtc->State = HAL_RTC_STATE_READY;


		return 1;
	}
	else return 0;

}
/**
  * @brief  Enters the RTC Initialization mode.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
HAL_StatusTypeDef RTC_EnterInitMode(RTC_HandleTypeDef *hrtc)
{
  uint32_t tickstart = 0U;

  tickstart = g_tickCnt;
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while ((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if ((g_tickCnt - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);


  return HAL_OK;
}

/**
  * @brief  Exit the RTC Initialization mode.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
HAL_StatusTypeDef RTC_ExitInitMode(RTC_HandleTypeDef *hrtc)
{
  uint32_t tickstart = 0U;

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

  tickstart = g_tickCnt;//SysTick_GetCounter();
  /* Wait till RTC is in INIT state and if Time out is reached exit */
  while ((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
    if ((g_tickCnt - tickstart) >  RTC_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

/**
  * @brief  Read the time counter available in RTC_ALR registers.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval Time counter
  */
uint32_t RTC_ReadAlarmCounter(RTC_HandleTypeDef *hrtc)
{
  unsigned short high1 = 0U, low = 0U;

  high1 = READ_REG(hrtc->Instance->ALRH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(hrtc->Instance->ALRL & RTC_CNTL_RTC_CNT);

  return (((uint32_t) high1 << 16U) | low);
}

/**
  * @brief  Write the time counter in RTC_ALR registers.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  AlarmCounter: Counter to write in RTC_ALR registers
  * @retval HAL status
  */
HAL_StatusTypeDef RTC_WriteAlarmCounter(RTC_HandleTypeDef *hrtc, uint32_t AlarmCounter)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Set Initialization mode */
  if (RTC_EnterInitMode(hrtc) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  else
  {
    /* Set RTC COUNTER MSB word */
    WRITE_REG(hrtc->Instance->ALRH, (AlarmCounter >> 16U));
    /* Set RTC COUNTER LSB word */
    WRITE_REG(hrtc->Instance->ALRL, (AlarmCounter & RTC_ALRL_RTC_ALR));

    /* Wait for synchro */
    if (RTC_ExitInitMode(hrtc) != HAL_OK)
    {
      status = HAL_ERROR;
    }
  }

  return status;
}


/**
  * @brief  Read the time counter available in RTC_CNT registers.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval Time counter
  */
uint32_t RTC_ReadTimeCounter(RTC_HandleTypeDef *hrtc)
{
  uint16_t high1 = 0U, high2 = 0U, low = 0U;
  uint32_t timecounter = 0U;

  high1 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT);
  high2 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);

  if (high1 != high2)
  {
    /* In this case the counter roll over during reading of CNTL and CNTH registers,
       read again CNTL register then return the counter value */
    timecounter = (((uint32_t) high2 << 16U) | READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT));
  }
  else
  {
    /* No counter roll over during reading of CNTL and CNTH registers, counter
       value is equal to first value of CNTL and CNTH */
    timecounter = (((uint32_t) high1 << 16U) | low);
  }

  return timecounter;
}

/**
  * @brief  Write the time counter in RTC_CNT registers.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  TimeCounter: Counter to write in RTC_CNT registers
  * @retval HAL status
  */
HAL_StatusTypeDef RTC_WriteTimeCounter(RTC_HandleTypeDef *hrtc, uint32_t TimeCounter)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Set Initialization mode */
  if (RTC_EnterInitMode(hrtc) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  else
  {
    /* Set RTC COUNTER MSB word */
    WRITE_REG(hrtc->Instance->CNTH, (TimeCounter >> 16U));
    /* Set RTC COUNTER LSB word */
    WRITE_REG(hrtc->Instance->CNTL, (TimeCounter & RTC_CNTL_RTC_CNT));

    /* Wait for synchro */
    if (RTC_ExitInitMode(hrtc) != HAL_OK)
    {
      status = HAL_ERROR;
    }
  }

  return status;
}

/**
  * @brief  Sets RTC current time.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTime: Pointer to Time structure
  * @param  Format: Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN: Binary data format
  *            @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
  uint32_t counter_time = 0U, counter_alarm = 0U;

  /* Check input parameters */
  if ((hrtc == NULL) || (sTime == NULL))
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RTC_FORMAT(Format));

  /* Process Locked */
  // __HAL_LOCK(hrtc);

  hrtc->State = HAL_RTC_STATE_BUSY;

  if (Format == RTC_FORMAT_BIN)
  {
    counter_time = (uint32_t)(((uint32_t)sTime->Hours * 3600U) + \
                              ((uint32_t)sTime->Minutes * 60U) + \
                              ((uint32_t)sTime->Seconds));
  }
  else
  {
    counter_time = (((uint32_t)(RTC_Bcd2ToByte(sTime->Hours)) * 3600U) + \
                    ((uint32_t)(RTC_Bcd2ToByte(sTime->Minutes)) * 60U) + \
                    ((uint32_t)(RTC_Bcd2ToByte(sTime->Seconds))));
  }

  /* Write time counter in RTC registers */
  if (RTC_WriteTimeCounter(hrtc, counter_time) != HAL_OK)
  {
    /* Set RTC state */
    hrtc->State = HAL_RTC_STATE_ERROR;

    /* Process Unlocked */
    //__HAL_UNLOCK(hrtc);

    return HAL_ERROR;
  }
  else
  {
    /* Clear Second and overflow flags */
    CLEAR_BIT(hrtc->Instance->CRL, (RTC_FLAG_SEC | RTC_FLAG_OW));

    /* Read current Alarm counter in RTC registers */
    counter_alarm = RTC_ReadAlarmCounter(hrtc);

    /* Set again alarm to match with new time if enabled */
    if (counter_alarm != RTC_ALARM_RESETVALUE)
    {
      if (counter_alarm < counter_time)
      {
        /* Add 1 day to alarm counter*/
        counter_alarm += (uint32_t)(24U * 3600U);

        /* Write new Alarm counter in RTC registers */
        if (RTC_WriteAlarmCounter(hrtc, counter_alarm) != HAL_OK)
        {
          /* Set RTC state */
          hrtc->State = HAL_RTC_STATE_ERROR;

          /* Process Unlocked */
          //__HAL_UNLOCK(hrtc);

          return HAL_ERROR;
        }
      }
    }

    hrtc->State = HAL_RTC_STATE_READY;

    //__HAL_UNLOCK(hrtc);

    return HAL_OK;
  }
}

/**
  * @brief  Gets RTC current time.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTime: Pointer to Time structure
  * @param  Format: Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN: Binary data format
  *            @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
  uint32_t counter_time = 0U, counter_alarm = 0U, days_elapsed = 0U, hours = 0U;

  /* Check input parameters */
  if ((hrtc == NULL) || (sTime == NULL))
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RTC_FORMAT(Format));

  /* Check if counter overflow occurred */
  if (__HAL_RTC_OVERFLOW_GET_FLAG(hrtc, RTC_FLAG_OW))
  {
    return HAL_ERROR;
  }

  /* Read the time counter*/
  counter_time = RTC_ReadTimeCounter(hrtc);

  /* Fill the structure fields with the read parameters */
  hours = counter_time / 3600U;
  sTime->Minutes  = (unsigned char)((counter_time % 3600U) / 60U);
  sTime->Seconds  = (unsigned char)((counter_time % 3600U) % 60U);

  if (hours >= 24U)
  {
    /* Get number of days elapsed from last calculation */
    days_elapsed = (hours / 24U);

    /* Set Hours in RTC_TimeTypeDef structure*/
    sTime->Hours = (hours % 24U);

    /* Read Alarm counter in RTC registers */
    counter_alarm = RTC_ReadAlarmCounter(hrtc);

    /* Calculate remaining time to reach alarm (only if set and not yet expired)*/
    if ((counter_alarm != RTC_ALARM_RESETVALUE) && (counter_alarm > counter_time))
    {
      counter_alarm -= counter_time;
    }
    else
    {
      /* In case of counter_alarm < counter_time */
      /* Alarm expiration already occurred but alarm not deactivated */
      counter_alarm = RTC_ALARM_RESETVALUE;
    }

    /* Set updated time in decreasing counter by number of days elapsed */
    counter_time -= (days_elapsed * 24U * 3600U);

    /* Write time counter in RTC registers */
    if (RTC_WriteTimeCounter(hrtc, counter_time) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Set updated alarm to be set */
    if (counter_alarm != RTC_ALARM_RESETVALUE)
    {
      counter_alarm += counter_time;

      /* Write time counter in RTC registers */
      if (RTC_WriteAlarmCounter(hrtc, counter_alarm) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }
    else
    {
      /* Alarm already occurred. Set it to reset values to avoid unexpected expiration */
      if (RTC_WriteAlarmCounter(hrtc, counter_alarm) != HAL_OK)
      {
        return HAL_ERROR;
      }
    }

    /* Update date */
    RTC_DateUpdate(hrtc, days_elapsed);
  }
  else
  {
    sTime->Hours = hours;
  }

  /* Check the input parameters format */
  if (Format != RTC_FORMAT_BIN)
  {
    /* Convert the time structure parameters to BCD format */
    sTime->Hours    = (unsigned char)RTC_ByteToBcd2(sTime->Hours);
    sTime->Minutes  = (unsigned char)RTC_ByteToBcd2(sTime->Minutes);
    sTime->Seconds  = (unsigned char)RTC_ByteToBcd2(sTime->Seconds);
  }

  return HAL_OK;
}


/**
  * @brief  Sets RTC current date.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sDate: Pointer to date structure
  * @param  Format: specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN: Binary data format
  *            @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format)
{
  uint32_t counter_time = 0U, counter_alarm = 0U, hours = 0U;

  /* Check input parameters */
  if ((hrtc == NULL) || (sDate == NULL))
  {
    return HAL_ERROR;
  }

  /* Process Locked */
  //__HAL_LOCK(hrtc);

  hrtc->State = HAL_RTC_STATE_BUSY;

  if (Format == RTC_FORMAT_BIN)
  {
    // Change the current date
    hrtc->DateToUpdate.Year  = sDate->Year;
    hrtc->DateToUpdate.Month = sDate->Month;
    hrtc->DateToUpdate.Date  = sDate->Date;
  }
  else
  {
    // Change the current date
    hrtc->DateToUpdate.Year  = RTC_Bcd2ToByte(sDate->Year);
    hrtc->DateToUpdate.Month = RTC_Bcd2ToByte(sDate->Month);
    hrtc->DateToUpdate.Date  = RTC_Bcd2ToByte(sDate->Date);
  }

  /* WeekDay set by user can be ignored because automatically calculated */
  hrtc->DateToUpdate.WeekDay = RTC_WeekDayNum(hrtc->DateToUpdate.Year, hrtc->DateToUpdate.Month, hrtc->DateToUpdate.Date);
  sDate->WeekDay = hrtc->DateToUpdate.WeekDay;

  /* Reset time to be aligned on the same day */
  /* Read the time counter*/
  counter_time = RTC_ReadTimeCounter(hrtc);

  /* Fill the structure fields with the read parameters */
  hours = counter_time / 3600U;
  if (hours > 24U)
  {
    /* Set updated time in decreasing counter by number of days elapsed */
    counter_time -= ((hours / 24U) * 24U * 3600U);
    /* Write time counter in RTC registers */
    if (RTC_WriteTimeCounter(hrtc, counter_time) != HAL_OK)
    {
      /* Set RTC state */
      hrtc->State = HAL_RTC_STATE_ERROR;

      /* Process Unlocked */
      //__HAL_UNLOCK(hrtc);

      return HAL_ERROR;
    }

    /* Read current Alarm counter in RTC registers */
    counter_alarm = RTC_ReadAlarmCounter(hrtc);

    /* Set again alarm to match with new time if enabled */
    if (counter_alarm != RTC_ALARM_RESETVALUE)
    {
      if (counter_alarm < counter_time)
      {
        /* Add 1 day to alarm counter*/
        counter_alarm += (uint32_t)(24U * 3600U);

        /* Write new Alarm counter in RTC registers */
        if (RTC_WriteAlarmCounter(hrtc, counter_alarm) != HAL_OK)
        {
          /* Set RTC state */
          hrtc->State = HAL_RTC_STATE_ERROR;

          /* Process Unlocked */
          //__HAL_UNLOCK(hrtc);

          return HAL_ERROR;
        }
      }
    }
  }

  hrtc->State = HAL_RTC_STATE_READY ;

  /* Process Unlocked */
  //__HAL_UNLOCK(hrtc);

  return HAL_OK;
}

/**
  * @brief  Gets RTC current date.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sDate: Pointer to Date structure
  * @param  Format: Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN:  Binary data format
  *            @arg RTC_FORMAT_BCD:  BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format)
{
  RTC_TimeTypeDef stime = {0U};

  /* Check input parameters */
  if ((hrtc == NULL) || (sDate == NULL))
  {
    return HAL_ERROR;
  }

  /* Call HAL_RTC_GetTime function to update date if counter higher than 24 hours */
  if (HAL_RTC_GetTime(hrtc, &stime, RTC_FORMAT_BIN) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Fill the structure fields with the read parameters */
  sDate->WeekDay  = hrtc->DateToUpdate.WeekDay;
  sDate->Year     = hrtc->DateToUpdate.Year;
  sDate->Month    = hrtc->DateToUpdate.Month;
  sDate->Date     = hrtc->DateToUpdate.Date;

  /* Check the input parameters format */
  if (Format != RTC_FORMAT_BIN)
  {
    /* Convert the date structure parameters to BCD format */
    sDate->Year   = (unsigned char)RTC_ByteToBcd2(sDate->Year);
    sDate->Month  = (unsigned char)RTC_ByteToBcd2(sDate->Month);
    sDate->Date   = (unsigned char)RTC_ByteToBcd2(sDate->Date);
  }
  return HAL_OK;
}

/**
  * @brief  Updates date when time is 23:59:59.
  * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  DayElapsed: Number of days elapsed from last date update
  * @retval None
  */
void RTC_DateUpdate(RTC_HandleTypeDef *hrtc, uint32_t DayElapsed)
{
  uint32_t year = 0U, month = 0U, day = 0U;
  uint32_t loop = 0U;

  /* Get the current year*/
  year = hrtc->DateToUpdate.Year;

  /* Get the current month and day */
  month = hrtc->DateToUpdate.Month;
  day = hrtc->DateToUpdate.Date;

  for (loop = 0U; loop < DayElapsed; loop++)
  {
    if ((month == 1U) || (month == 3U) || (month == 5U) || (month == 7U) || \
        (month == 8U) || (month == 10U) || (month == 12U))
    {
      if (day < 31U)
      {
        day++;
      }
      /* Date structure member: day = 31 */
      else
      {
        if (month != 12U)
        {
          month++;
          day = 1U;
        }
        /* Date structure member: day = 31 & month =12 */
        else
        {
          month = 1U;
          day = 1U;
          year++;
        }
      }
    }
    else if ((month == 4U) || (month == 6U) || (month == 9U) || (month == 11U))
    {
      if (day < 30U)
      {
        day++;
      }
      /* Date structure member: day = 30 */
      else
      {
        month++;
        day = 1U;
      }
    }
    else if (month == 2U)
    {
      if (day < 28U)
      {
        day++;
      }
      else if (day == 28U)
      {
        /* Leap year */
        if (RTC_IsLeapYear(year))
        {
          day++;
        }
        else
        {
          month++;
          day = 1U;
        }
      }
      else if (day == 29U)
      {
        month++;
        day = 1U;
      }
    }
  }

  /* Update year */
  hrtc->DateToUpdate.Year = year;

  /* Update day and month */
  hrtc->DateToUpdate.Month = month;
  hrtc->DateToUpdate.Date = day;

  /* Update day of the week */
  hrtc->DateToUpdate.WeekDay = RTC_WeekDayNum(year, month, day);
}

/**
  * @brief  Check whether the passed year is Leap or not.
  * @param  nYear  year to check
  * @retval 1: leap year
  *         0: not leap year
  */
unsigned char RTC_IsLeapYear(uint16_t nYear)
{
  if ((nYear % 4U) != 0U)
  {
    return 0U;
  }

  if ((nYear % 100U) != 0U)
  {
    return 1U;
  }

  if ((nYear % 400U) == 0U)
  {
    return 1U;
  }
  else
  {
    return 0U;
  }
}

/**
  * @brief  Determines the week number, the day number and the week day number.
  * @param  nYear   year to check
  * @param  nMonth  Month to check
  * @param  nDay    Day to check
  * @note   Day is calculated with hypothesis that year > 2000
  * @retval Value which can take one of the following parameters:
  *         @arg RTC_WEEKDAY_MONDAY
  *         @arg RTC_WEEKDAY_TUESDAY
  *         @arg RTC_WEEKDAY_WEDNESDAY
  *         @arg RTC_WEEKDAY_THURSDAY
  *         @arg RTC_WEEKDAY_FRIDAY
  *         @arg RTC_WEEKDAY_SATURDAY
  *         @arg RTC_WEEKDAY_SUNDAY
  */
unsigned char RTC_WeekDayNum(uint32_t nYear, unsigned char nMonth, unsigned char nDay)
{
  uint32_t year = 0U, weekday = 0U;

  year = 2000U + nYear;

  if (nMonth < 3U)
  {
    /*D = { [(23 x month)/9] + day + 4 + year + [(year-1)/4] - [(year-1)/100] + [(year-1)/400] } mod 7*/
    weekday = (((23U * nMonth) / 9U) + nDay + 4U + year + ((year - 1U) / 4U) - ((year - 1U) / 100U) + ((year - 1U) / 400U)) % 7U;
  }
  else
  {
    /*D = { [(23 x month)/9] + day + 4 + year + [year/4] - [year/100] + [year/400] - 2 } mod 7*/
    weekday = (((23U * nMonth) / 9U) + nDay + 4U + year + (year / 4U) - (year / 100U) + (year / 400U) - 2U) % 7U;
  }

  return (unsigned char)weekday;
}

/**
  * @brief  Converts a 2 digit decimal to BCD format.
  * @param  Value: Byte to be converted
  * @retval Converted byte
  */
unsigned char RTC_ByteToBcd2(unsigned char Value)
{
  uint32_t bcdhigh = 0U;

  while (Value >= 10U)
  {
    bcdhigh++;
    Value -= 10U;
  }

  return ((unsigned char)(bcdhigh << 4U) | Value);
}

/**
  * @brief  Converts from 2 digit BCD to Binary.
  * @param  Value: BCD value to be converted
  * @retval Converted word
  */
unsigned char RTC_Bcd2ToByte(unsigned char Value)
{
  uint32_t tmp = 0U;
  tmp = ((unsigned char)(Value & (unsigned char)0xF0) >> (unsigned char)0x4) * 10U;
  return (tmp + (Value & (unsigned char)0x0F));
}
/**************************************************************************/
/*!
    @brief  Set the date and time in the DS1307
    @param dt DateTime object containing the desired date/time
*/
/*
void RTC_adjust(struct _DateTime *dt) {

	  //ISL1208_setDateTime_DateTime("Feb 05 20", "17:36:38");//ISL1208_setDateTime_DateTime("Feb 05 2020", "17:36:38");
	  //make CH bit 0
		stmI2cSendbuf[0] = ISL1208_REG_TIME;
		stmI2cSendbuf[1] = bin2bcd(dt->ss);
		stmI2cSendbuf[2] = bin2bcd(dt->mm);
		stmI2cSendbuf[3] = bin2bcd(dt->hh);
		stmI2cSendbuf[4] = bin2bcd(0);
		stmI2cSendbuf[5] = bin2bcd(dt->date);
		stmI2cSendbuf[6] = bin2bcd(dt->m);
		stmI2cSendbuf[7] = bin2bcd(dt->yOff);

	  ISL1208_writeRegister8(0x00, 0x00);
	  stm_I2C_SendBurst(ISL1208_ADDR8, stmI2cSendbuf, 8); //write reg.

}
*/
/**************************************************************************/
/*!
    @brief  Get the current date and time from the DS1307
    @return DateTime object containing the current date and time
*/
/**************************************************************************/
//struct _DateTime* RTC_ISL1208_now(struct _DateTime *dt) {
void RTC_now(struct _DateTime *dt) { //ISL1208_getDateTime()

    unsigned char retlen;

    //retlen = stm_I2C_ReceiveBurstWithRestartCondition(ISL1208_ADDR8, ISL1208_REG_TIME, stmI2cRecvbuf, 7); //Reg0~6
	if(retlen==0){
		printf("i2c Rd error\r\n");
		return 0;
	}

    dt->ss = bcd2bin(stmI2cRecvbuf[0] & 0x7F);
    dt->mm = bcd2bin(stmI2cRecvbuf[1]);
    dt->hh = bcd2bin(stmI2cRecvbuf[2]);
    //skip 3 (Day: Mon, ...)
    dt->date = bcd2bin(stmI2cRecvbuf[4]);
    dt->m = bcd2bin(stmI2cRecvbuf[5]);
    dt->yOff = bcd2bin(stmI2cRecvbuf[6]);
}

//=============== stm32f1xx_hal_rtc.c==============
/*==============================================================================
                  ##### How to use this driver #####
  ==================================================================
  [..]
    (+) Enable the RTC domain access (see description in the section above).
    (+) Configure the RTC Prescaler (Asynchronous prescaler to generate RTC 1Hz time base)
        using the HAL_RTC_Init() function.

  *** Time and Date configuration ***
  ===================================
  [..]
    (+) To configure the RTC Calendar (Time and Date) use the HAL_RTC_SetTime()
        and HAL_RTC_SetDate() functions.
    (+) To read the RTC Calendar, use the HAL_RTC_GetTime() and HAL_RTC_GetDate() functions.

  *** Alarm configuration ***
  ===========================
  [..]
    (+) To configure the RTC Alarm use the HAL_RTC_SetAlarm() function.
        You can also configure the RTC Alarm with interrupt mode using the HAL_RTC_SetAlarm_IT() function.
    (+) To read the RTC Alarm, use the HAL_RTC_GetAlarm() function.

  *** Tamper configuration ***
  ============================
  [..]
    (+) Enable the RTC Tamper and configure the Tamper Level using the
        HAL_RTCEx_SetTamper() function. You can configure RTC Tamper with interrupt
        mode using HAL_RTCEx_SetTamper_IT() function.
    (+) The TAMPER1 alternate function can be mapped to PC13

  *** Backup Data Registers configuration ***
  ===========================================
  [..]
    (+) To write to the RTC Backup Data registers, use the HAL_RTCEx_BKUPWrite()
        function.
    (+) To read the RTC Backup Data registers, use the HAL_RTCEx_BKUPRead()
        function.

                  ##### WARNING: Drivers Restrictions  #####
  ==================================================================
  [..] RTC version used on STM32F1 families is version V1. All the features supported by V2
       (other families) will be not supported on F1.
  [..] As on V2, main RTC features are managed by HW. But on F1, date feature is completely
       managed by SW.
  [..] Then, there are some restrictions compared to other families:
    (+) Only format 24 hours supported in HAL (format 12 hours not supported)
    (+) Date is saved in SRAM. Then, when MCU is in STOP or STANDBY mode, date will be lost.
        User should implement a way to save date before entering in low power mode (an
        example is provided with firmware package based on backup registers)
    (+) Date is automatically updated each time a HAL_RTC_GetTime or HAL_RTC_GetDate is called.
    (+) Alarm detection is limited to 1 day. It will expire only 1 time (no alarm repetition, need
        to program a new alarm)

              ##### Backup Domain Operating Condition #####
  ==============================================================================
  [..] The real-time clock (RTC) and the RTC backup registers can be powered
       from the VBAT voltage when the main VDD supply is powered off.
       To retain the content of the RTC backup registers and supply the RTC
       when VDD is turned off, VBAT pin can be connected to an optional
       standby voltage supplied by a battery or by another source.

  [..] To allow the RTC operating even when the main digital supply (VDD) is turned
       off, the VBAT pin powers the following blocks:
    (#) The RTC
    (#) The LSE oscillator
    (#) The backup SRAM when the low power backup regulator is enabled
    (#) PC13 to PC15 I/Os, plus PI8 I/O (when available)

  [..] When the backup domain is supplied by VDD (analog switch connected to VDD),
       the following pins are available:
    (+) PC13 can be used as a Tamper pin

  [..] When the backup domain is supplied by VBAT (analog switch connected to VBAT
       because VDD is not present), the following pins are available:
    (+) PC13 can be used as the Tamper pin

                   ##### Backup Domain Reset #####
  ==================================================================
  [..] The backup domain reset sets all RTC registers and the RCC_BDCR register
       to their reset values.
  [..] A backup domain reset is generated when one of the following events occurs:
    (#) Software reset, triggered by setting the BDRST bit in the
        RCC Backup domain control register (RCC_BDCR).
    (#) VDD or VBAT power on, if both supplies have previously been powered off.
    (#) Tamper detection event resets all data backup registers.

                   ##### Backup Domain Access #####
  ==================================================================
  [..] After reset, the backup domain (RTC registers, RTC backup data
       registers and backup SRAM) is protected against possible unwanted write
       accesses.
  [..] To enable access to the RTC Domain and RTC registers, proceed as follows:
    (+) Call the function HAL_RCCEx_PeriphCLKConfig in using RCC_PERIPHCLK_RTC for
        PeriphClockSelection and select RTCClockSelection (LSE, LSI or HSE)
    (+) Enable the BKP clock in using __HAL_RCC_BKP_CLK_ENABLE()

                  ##### RTC and low power modes #####
  ==================================================================
  [..] The MCU can be woken up from a low power mode by an RTC alternate
       function.
  [..] The RTC alternate functions are the RTC alarms (Alarm A),
       and RTC tamper event detection.
       These RTC alternate functions can wake up the system from the Stop and
       Standby low power modes.
  [..] The system can also wake up from low power modes without depending
       on an external interrupt (Auto-wakeup mode), by using the RTC alarm.

  *** Callback registration ***
  =============================================
  [..]
  The compilation define  USE_HAL_RTC_REGISTER_CALLBACKS when set to 1
  allows the user to configure dynamically the driver callbacks.
  Use Function @ref HAL_RTC_RegisterCallback() to register an interrupt callback.

  [..]
  Function @ref HAL_RTC_RegisterCallback() allows to register following callbacks:
    (+) AlarmAEventCallback          : RTC Alarm A Event callback.
    (+) Tamper1EventCallback         : RTC Tamper 1 Event callback.
    (+) MspInitCallback              : RTC MspInit callback.
    (+) MspDeInitCallback            : RTC MspDeInit callback.
  [..]
  This function takes as parameters the HAL peripheral handle, the Callback ID
  and a pointer to the user callback function.

  [..]
  Use function @ref HAL_RTC_UnRegisterCallback() to reset a callback to the default
  weak function.
  @ref HAL_RTC_UnRegisterCallback() takes as parameters the HAL peripheral handle,
  and the Callback ID.
  This function allows to reset following callbacks:
    (+) AlarmAEventCallback          : RTC Alarm A Event callback.
    (+) Tamper1EventCallback         : RTC Tamper 1 Event callback.
    (+) MspInitCallback              : RTC MspInit callback.
    (+) MspDeInitCallback            : RTC MspDeInit callback.
  [..]
  By default, after the @ref HAL_RTC_Init() and when the state is HAL_RTC_STATE_RESET,
  all callbacks are set to the corresponding weak functions :
  example @ref AlarmAEventCallback().
  Exception done for MspInit and MspDeInit callbacks that are reset to the legacy weak function
  in the @ref HAL_RTC_Init()/@ref HAL_RTC_DeInit() only when these callbacks are null
  (not registered beforehand).
  If not, MspInit or MspDeInit are not null, @ref HAL_RTC_Init()/@ref HAL_RTC_DeInit()
  keep and use the user MspInit/MspDeInit callbacks (registered beforehand)
  [..]
  Callbacks can be registered/unregistered in HAL_RTC_STATE_READY state only.
  Exception done MspInit/MspDeInit that can be registered/unregistered
  in HAL_RTC_STATE_READY or HAL_RTC_STATE_RESET state,
  thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
  In that case first register the MspInit/MspDeInit user callbacks
  using @ref HAL_RTC_RegisterCallback() before calling @ref HAL_RTC_DeInit()
  or @ref HAL_RTC_Init() function.
  [..]
  When The compilation define USE_HAL_RTC_REGISTER_CALLBACKS is set to 0 or
  not defined, the callback registration feature is not available and all callbacks
  are set to the corresponding weak functions.
   @endverbatim
  ******************************************************************************
  *******************************************************************************/
/**
  * @brief  Writes a data in a specified RTC Backup data register.
  * @param  hrtc: pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  BackupRegister: RTC Backup data Register number.
  *          This parameter can be: RTC_BKP_DRx where x can be from 1 to 10 (or 42) to
  *                                 specify the register (depending devices).
  * @param  Data: Data to be written in the specified RTC Backup data register.
  * @retval None
  */
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data)
{
  uint32_t tmp = 0U;

  /* Prevent unused argument(s) compilation warning */
  //UNUSED(hrtc);

  /* Check the parameters */
  //assert_param(IS_RTC_BKP(BackupRegister));

  tmp = (uint32_t)BKP_BASE;
  tmp += (BackupRegister * 4U);

  *(__IO uint32_t *) tmp = (Data & BKP_DR1_D);
}

void RTC_CalendarConfig(RTC_HandleTypeDef *hRtc){
	RTC_DateTypeDef sdatestruct;
	RTC_TimeTypeDef stimestruct;
	sdatestruct.Year = 0x20; //2020
	sdatestruct.Month = RTC_MONTH_MARCH;
	sdatestruct.Date = 25;
	sdatestruct.WeekDay = RTC_WEEKDAY_FRIDAY;

	if(HAL_RTC_SetDate(hRtc, &sdatestruct, RTC_FORMAT_BCD) != 0){ //err

	}
	//set time
	stimestruct.Hours = 2;
	stimestruct.Minutes = 1;
	stimestruct.Seconds = 0;
	if(HAL_RTC_SetTime(hRtc, &stimestruct, RTC_FORMAT_BCD) != 0){ //err

	}
	HAL_RTCEx_BKUPWrite(hRtc,RTC_BKP_DR1, 0x32F2);


}
//================================================================================
void stmRTCLoop()
{
	unsigned cnt;
	//LED
	GPIO_InitTypeDef 	GPIO_InitStruct;
	struct _DateTime  	DateTime;
	RTC_TimeTypeDef 	sTime;
	RTC_DateTypeDef 	sDate;
	RTC_HandleTypeDef 	rtcHandle;

	printf("RTC Loop\r\n");

	//LED PC13
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	delayms(300);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	delayms(300);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);

	//OLED
	if(!g_bI2CModuleConfigDone){
#if (PROCESSOR == STM32F407VGT6)
		gI2Cx = I2C1;
#elif (PROCESSOR == STM32F407VZT6)
		gI2Cx = I2C2;
#endif
		stmI2cModuleConfig(gI2Cx,400000);//400Kbps
	}
	printf("Done.\r\n");

	//Config RTC
	rtcHandle.Instance = RTC;
	rtcHandle.Init.AsynchPrediv =  RTC_AUTO_1_SECOND;
	stmRTC_Init(&rtcHandle);

	//read backup reg 1. if so, no need to config
	//set time
	DateTime.yOff = 20;
	DateTime.m = 2;
	DateTime.date = 6;
	DateTime.day = 3;
	DateTime.hh = 9;
	DateTime.mm = 43;
	DateTime.ss = 20;
	RTC_CalendarConfig(&rtcHandle);
	//ISL1208_setup(&DateTime);
	//RTC_SetCounter(1000);

	//enable RTC Alarm IRQ
	//RTC_ITConfig(RTC_IT_ALR,ENABLE); //
	//RTC_WaitForLastTask();

  // For leading zero look to ISL1208_dateformat example
	while(1){

		//Get Date
		if (HAL_RTC_GetDate(&rtcHandle, &sDate, RTC_FORMAT_BIN) != HAL_OK)
		{
		    //return HAL_ERROR;
		}
		DateTime.yOff = sDate.Year;
		DateTime.m = sDate.Month;
		DateTime.date = sDate.Date;

		//Get Time
		if (HAL_RTC_GetTime(&rtcHandle, &sTime, RTC_FORMAT_BIN) != HAL_OK)
		{
		    //return HAL_ERROR;
		}
		DateTime.hh = sTime.Hours;
		DateTime.mm = sTime.Minutes;
		DateTime.ss = sTime.Seconds;

		//Show
		printf("NowTime = ");
		printf("%u-",DateTime.yOff + 2000);
		printf("%u-",DateTime.m);
		printf("%u ",DateTime.date);
		printf("(%s) ", daysOfTheWeek[DateTime_dayOfTheWeek(&DateTime)]);

		printf("%u:",DateTime.hh);
		printf("%u:",DateTime.mm);
		printf("%u\r\n",DateTime.ss);
		//=== unix time since 1970.1.1 midnight
		printf("UnixTime= %us\r\n", DateTime_unixtime(&DateTime) );

		cnt = RTC_GetCounter();
		printf("rtcCnt= %u\r\n", cnt);

		//Alarm in 5sec ============
		//RTC_SetAlarm(RTC_GetCounter()+5);
		//RTC_WaitForLastTask();

		//Enter STANDBY mode
		//PWR_EnterSTANDBYMode(); // after alarm --> run main loop again...

		delayms(1000);
	}
}


