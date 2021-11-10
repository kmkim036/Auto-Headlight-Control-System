/**
  ******************************************************************************
  * @file    stm32f10x_rtc.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the RTC firmware 
  *          library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_RTC_H
#define __STM32F10x_RTC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/** @defgroup RTC_interrupts_define 
  * @{
  */

#define RTC_IT_OW            ((uint16_t)0x0004)  /*!< Overflow interrupt */
#define RTC_IT_ALR           ((uint16_t)0x0002)  /*!< Alarm interrupt */
#define RTC_IT_SEC           ((uint16_t)0x0001)  /*!< Second interrupt */
#define IS_RTC_IT(IT) ((((IT) & (uint16_t)0xFFF8) == 0x00) && ((IT) != 0x00))
#define IS_RTC_GET_IT(IT) (((IT) == RTC_IT_OW) || ((IT) == RTC_IT_ALR) || \
                           ((IT) == RTC_IT_SEC))
/**
  * @}
  */ 

/** @defgroup RTC_interrupts_flags 
  * @{
  */

#define RTC_FLAG_RTOFF       ((uint16_t)0x0020)  /*!< RTC Operation OFF flag */
#define RTC_FLAG_RSF         ((uint16_t)0x0008)  /*!< Registers Synchronized flag */
#define RTC_FLAG_OW          ((uint16_t)0x0004)  /*!< Overflow flag */
#define RTC_FLAG_ALR         ((uint16_t)0x0002)  /*!< Alarm flag */
#define RTC_FLAG_SEC         ((uint16_t)0x0001)  /*!< Second flag */
#define IS_RTC_CLEAR_FLAG(FLAG) ((((FLAG) & (uint16_t)0xFFF0) == 0x00) && ((FLAG) != 0x00))
#define IS_RTC_GET_FLAG(FLAG) (((FLAG) == RTC_FLAG_RTOFF) || ((FLAG) == RTC_FLAG_RSF) || \
                               ((FLAG) == RTC_FLAG_OW) || ((FLAG) == RTC_FLAG_ALR) || \
                               ((FLAG) == RTC_FLAG_SEC))
#define IS_RTC_PRESCALER(PRESCALER) ((PRESCALER) <= 0xFFFFF)

void RTC_ITConfig(uint16_t RTC_IT, FunctionalState NewState);
void RTC_EnterConfigMode(void);
void RTC_ExitConfigMode(void);
uint32_t  RTC_GetCounter(void);
void RTC_SetCounter(uint32_t CounterValue);
void RTC_SetPrescaler(uint32_t PrescalerValue);
void RTC_SetAlarm(uint32_t AlarmValue);
uint32_t  RTC_GetDivider(void);
void RTC_WaitForLastTask(void);
void RTC_WaitForSynchro(void);
FlagStatus RTC_GetFlagStatus(uint16_t RTC_FLAG);
void RTC_ClearFlag(uint16_t RTC_FLAG);
ITStatus RTC_GetITStatus(uint16_t RTC_IT);
void RTC_ClearITPendingBit(uint16_t RTC_IT);


//========add by YOON. from stm32f1xx_hal_rtc.h =======================================================

//
#define IS_RTC_ASYNCH_PREDIV(PREDIV)  (((PREDIV) <= 0xFFFFFU) || ((PREDIV) == RTC_AUTO_1_SECOND))
#define IS_RTC_HOUR24(HOUR)           ((HOUR) <= 23U)
#define IS_RTC_MINUTES(MINUTES)       ((MINUTES) <= 59U)
#define IS_RTC_SECONDS(SECONDS)       ((SECONDS) <= 59U)
#define IS_RTC_FORMAT(FORMAT)         (((FORMAT) == RTC_FORMAT_BIN) || ((FORMAT) == RTC_FORMAT_BCD))
#define IS_RTC_YEAR(YEAR)             ((YEAR) <= 99U)
#define IS_RTC_MONTH(MONTH)           (((MONTH) >= 1U) && ((MONTH) <= 12U))
#define IS_RTC_DATE(DATE)             (((DATE) >= 1U) && ((DATE) <= 31U))
#define IS_RTC_ALARM(ALARM)           ((ALARM) == RTC_ALARM_A)
#define IS_RTC_CALIB_OUTPUT(__OUTPUT__) (((__OUTPUT__) == RTC_OUTPUTSOURCE_NONE) || \
                                        ((__OUTPUT__) == RTC_OUTPUTSOURCE_CALIBCLOCK) || \
                                        ((__OUTPUT__) == RTC_OUTPUTSOURCE_ALARM) || \
                                        ((__OUTPUT__) == RTC_OUTPUTSOURCE_SECOND))

//RTC_Timeout_Value Default Timeout Value
#define RTC_TIMEOUT_VALUE           1000U
#define RTC_EXTI_LINE_ALARM_EVENT   ((uint32_t)EXTI_IMR_MR17)  /*!< External interrupt line 17 Connected to the RTC Alarm event */
/* Exported types ------------------------------------------------------------*/
// @brief  RTC Time structure definition
typedef struct
{
  uint8_t Hours;            /*!< Specifies the RTC Time Hour.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 23 */

  uint8_t Minutes;          /*!< Specifies the RTC Time Minutes.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t Seconds;          /*!< Specifies the RTC Time Seconds.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

} RTC_TimeTypeDef;

//RTC Alarm structure definition

typedef struct
{
  RTC_TimeTypeDef AlarmTime;     /*!< Specifies the RTC Alarm Time members */
  uint32_t Alarm;                /*!< Specifies the alarm ID (only 1 alarm ID for STM32F1).
                                      This parameter can be a value of @ref RTC_Alarms_Definitions */
} RTC_AlarmTypeDef;

//HAL State structures definition
typedef enum
{
  HAL_RTC_STATE_RESET             = 0x00U,  /*!< RTC not yet initialized or disabled */
  HAL_RTC_STATE_READY             = 0x01U,  /*!< RTC initialized and ready for use   */
  HAL_RTC_STATE_BUSY              = 0x02U,  /*!< RTC process is ongoing              */
  HAL_RTC_STATE_TIMEOUT           = 0x03U,  /*!< RTC timeout state                   */
  HAL_RTC_STATE_ERROR             = 0x04U   /*!< RTC error state                     */

} HAL_RTCStateTypeDef;

//RTC Configuration Structure definition
typedef struct
{
  uint32_t AsynchPrediv;    /*!< Specifies the RTC Asynchronous Predivider value.
                                 This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFFFFF  or RTC_AUTO_1_SECOND
                                 If RTC_AUTO_1_SECOND is selected, AsynchPrediv will be set automatically to get 1sec timebase */

  uint32_t OutPut;          /*!< Specifies which signal will be routed to the RTC Tamper pin.
                                 This parameter can be a value of @ref RTC_output_source_to_output_on_the_Tamper_pin */

} RTC_InitTypeDef;

//RTC Date structure definition

typedef struct
{
  uint8_t WeekDay;  /*!< Specifies the RTC Date WeekDay (not necessary for HAL_RTC_SetDate).
                         This parameter can be a value of @ref RTC_WeekDay_Definitions */

  uint8_t Month;    /*!< Specifies the RTC Date Month (in BCD format).
                         This parameter can be a value of @ref RTC_Month_Date_Definitions */

  uint8_t Date;     /*!< Specifies the RTC Date.
                         This parameter must be a number between Min_Data = 1 and Max_Data = 31 */

  uint8_t Year;     /*!< Specifies the RTC Date Year.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 99 */

} RTC_DateTypeDef;

/**
  * @brief  Time Handle Structure definition
  */
#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
typedef struct __RTC_HandleTypeDef
#else
typedef struct
#endif /* (USE_HAL_RTC_REGISTER_CALLBACKS) */
{
  RTC_TypeDef                 *Instance;  /*!< Register base address    */

  RTC_InitTypeDef             Init;       /*!< RTC required parameters  */

  RTC_DateTypeDef             DateToUpdate;       /*!< Current date set by user and updated automatically  */

  //HAL_LockTypeDef             Lock;       /*!< RTC locking object       */

  __IO HAL_RTCStateTypeDef    State;      /*!< Time communication state */

#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
  void (* AlarmAEventCallback)(struct __RTC_HandleTypeDef *hrtc);           /*!< RTC Alarm A Event callback         */

  void (* Tamper1EventCallback)(struct __RTC_HandleTypeDef *hrtc);          /*!< RTC Tamper 1 Event callback        */

  void (* MspInitCallback)(struct __RTC_HandleTypeDef *hrtc);               /*!< RTC Msp Init callback              */

  void (* MspDeInitCallback)(struct __RTC_HandleTypeDef *hrtc);             /*!< RTC Msp DeInit callback            */

#endif /* (USE_HAL_RTC_REGISTER_CALLBACKS) */

} RTC_HandleTypeDef;

#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL RTC Callback ID enumeration definition
  */
typedef enum
{
  HAL_RTC_ALARM_A_EVENT_CB_ID           = 0x00u,    /*!< RTC Alarm A Event Callback ID       */
  HAL_RTC_TAMPER1_EVENT_CB_ID           = 0x04u,    /*!< RTC Tamper 1 Callback ID            */
  HAL_RTC_MSPINIT_CB_ID                 = 0x0Eu,    /*!< RTC Msp Init callback ID            */
  HAL_RTC_MSPDEINIT_CB_ID               = 0x0Fu     /*!< RTC Msp DeInit callback ID          */
} HAL_RTC_CallbackIDTypeDef;

/**
  * @brief  HAL RTC Callback pointer definition
  */
typedef  void (*pRTC_CallbackTypeDef)(RTC_HandleTypeDef *hrtc);  /*!< pointer to an RTC callback function */
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

// RTC_Automatic_Prediv_1_Second Automatic calculation of prediv for 1sec timebase
#define RTC_AUTO_1_SECOND                      0xFFFFFFFFU

/** @defgroup RTC_Input_parameter_format_definitions Input Parameter Format
  * @{
  */
#define RTC_FORMAT_BIN                         0x000000000U
#define RTC_FORMAT_BCD                         0x000000001U

/** @defgroup RTC_Month_Date_Definitions Month Definitions
  * @{
  */

/* Coded in BCD format */
#define RTC_MONTH_JANUARY              ((uint8_t)0x01)
#define RTC_MONTH_FEBRUARY             ((uint8_t)0x02)
#define RTC_MONTH_MARCH                ((uint8_t)0x03)
#define RTC_MONTH_APRIL                ((uint8_t)0x04)
#define RTC_MONTH_MAY                  ((uint8_t)0x05)
#define RTC_MONTH_JUNE                 ((uint8_t)0x06)
#define RTC_MONTH_JULY                 ((uint8_t)0x07)
#define RTC_MONTH_AUGUST               ((uint8_t)0x08)
#define RTC_MONTH_SEPTEMBER            ((uint8_t)0x09)
#define RTC_MONTH_OCTOBER              ((uint8_t)0x10)
#define RTC_MONTH_NOVEMBER             ((uint8_t)0x11)
#define RTC_MONTH_DECEMBER             ((uint8_t)0x12)

/** @defgroup RTC_WeekDay_Definitions WeekDay Definitions
  * @{
  */
#define RTC_WEEKDAY_MONDAY             ((uint8_t)0x01)
#define RTC_WEEKDAY_TUESDAY            ((uint8_t)0x02)
#define RTC_WEEKDAY_WEDNESDAY          ((uint8_t)0x03)
#define RTC_WEEKDAY_THURSDAY           ((uint8_t)0x04)
#define RTC_WEEKDAY_FRIDAY             ((uint8_t)0x05)
#define RTC_WEEKDAY_SATURDAY           ((uint8_t)0x06)
#define RTC_WEEKDAY_SUNDAY             ((uint8_t)0x00)

/** @defgroup RTC_Alarms_Definitions Alarms Definitions
  * @{
  */
#define RTC_ALARM_A                        0U                                 /*!< Specify alarm ID (mainly for legacy purposes) */


/** @defgroup RTC_output_source_to_output_on_the_Tamper_pin Output source to output on the Tamper pin
  * @{
  */

#define RTC_OUTPUTSOURCE_NONE               0x00000000U                       /*!< No output on the TAMPER pin  */
#define RTC_OUTPUTSOURCE_CALIBCLOCK         BKP_RTCCR_CCO                     /*!< RTC clock with a frequency divided by 64 on the TAMPER pin  */
#define RTC_OUTPUTSOURCE_ALARM              BKP_RTCCR_ASOE                    /*!< Alarm pulse signal on the TAMPER pin  */
#define RTC_OUTPUTSOURCE_SECOND             (BKP_RTCCR_ASOS | BKP_RTCCR_ASOE) /*!< Second pulse signal on the TAMPER pin  */

/** @defgroup RTC_Interrupts_Definitions Interrupts Definitions
  * @{
  */
#define RTC_IT_OW            RTC_CRH_OWIE       /*!< Overflow interrupt */
#define RTC_IT_ALRA          RTC_CRH_ALRIE      /*!< Alarm interrupt */
#define RTC_IT_SEC           RTC_CRH_SECIE      /*!< Second interrupt */
#define RTC_IT_TAMP1         BKP_CSR_TPIE       /*!< TAMPER Pin interrupt enable */

/** @defgroup RTC_Flags_Definitions Flags Definitions
  * @{
  */
#define RTC_FLAG_RTOFF       RTC_CRL_RTOFF      /*!< RTC Operation OFF flag */
#define RTC_FLAG_RSF         RTC_CRL_RSF        /*!< Registers Synchronized flag */
#define RTC_FLAG_OW          RTC_CRL_OWF        /*!< Overflow flag */
#define RTC_FLAG_ALRAF       RTC_CRL_ALRF       /*!< Alarm flag */
#define RTC_FLAG_SEC         RTC_CRL_SECF       /*!< Second flag */
#define RTC_FLAG_TAMP1F      BKP_CSR_TEF        /*!< Tamper Interrupt Flag */

/** @brief  Reset RTC handle state
  * @param  __HANDLE__: RTC handle.
  * @retval None
  */
#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
#define __HAL_RTC_RESET_HANDLE_STATE(__HANDLE__) do{\
                                                      (__HANDLE__)->State = HAL_RTC_STATE_RESET;\
                                                      (__HANDLE__)->MspInitCallback = NULL;\
                                                      (__HANDLE__)->MspDeInitCallback = NULL;\
                                                     }while(0u)
#else
#define __HAL_RTC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_RTC_STATE_RESET)
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */

/**
  * @brief  Disable the write protection for RTC registers.
  * @param  __HANDLE__: specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_WRITEPROTECTION_DISABLE(__HANDLE__)         SET_BIT((__HANDLE__)->Instance->CRL, RTC_CRL_CNF)

/**
  * @brief  Enable the write protection for RTC registers.
  * @param  __HANDLE__: specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_WRITEPROTECTION_ENABLE(__HANDLE__)          CLEAR_BIT((__HANDLE__)->Instance->CRL, RTC_CRL_CNF)

/**
  * @brief  Enable the RTC Alarm interrupt.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Alarm interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg RTC_IT_ALRA: Alarm A interrupt
  * @retval None
  */
#define __HAL_RTC_ALARM_ENABLE_IT(__HANDLE__, __INTERRUPT__)  SET_BIT((__HANDLE__)->Instance->CRH, (__INTERRUPT__))

/**
  * @brief  Disable the RTC Alarm interrupt.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Alarm interrupt sources to be enabled or disabled.
  *         This parameter can be any combination of the following values:
  *            @arg RTC_IT_ALRA: Alarm A interrupt
  * @retval None
  */
#define __HAL_RTC_ALARM_DISABLE_IT(__HANDLE__, __INTERRUPT__) CLEAR_BIT((__HANDLE__)->Instance->CRH, (__INTERRUPT__))

/**
  * @brief  Check whether the specified RTC Alarm interrupt has been enabled or not.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Alarm interrupt sources to be checked
  *         This parameter can be:
  *            @arg RTC_IT_ALRA: Alarm A interrupt
  * @retval None
  */
#define __HAL_RTC_ALARM_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)     ((((((__HANDLE__)->Instance->CRH)& ((__INTERRUPT__)))) != RESET)? SET : RESET)

/**
  * @brief  Get the selected RTC Alarm's flag status.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __FLAG__: specifies the RTC Alarm Flag sources to be enabled or disabled.
  *          This parameter can be:
  *            @arg RTC_FLAG_ALRAF
  * @retval None
  */
#define __HAL_RTC_ALARM_GET_FLAG(__HANDLE__, __FLAG__)        (((((__HANDLE__)->Instance->CRL) & (__FLAG__)) != RESET)? SET : RESET)

/**
  * @brief  Check whether the specified RTC Alarm interrupt has occurred or not.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Alarm interrupt sources to check.
  *         This parameter can be:
  *            @arg RTC_IT_ALRA: Alarm A interrupt
  * @retval None
  */
#define __HAL_RTC_ALARM_GET_IT(__HANDLE__, __INTERRUPT__)        (((((__HANDLE__)->Instance->CRL) & (__INTERRUPT__)) != RESET)? SET : RESET)

/**
  * @brief  Clear the RTC Alarm's pending flags.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __FLAG__: specifies the RTC Alarm Flag sources to be enabled or disabled.
  *         This parameter can be:
  *            @arg RTC_FLAG_ALRAF
  * @retval None
  */
#define __HAL_RTC_ALARM_CLEAR_FLAG(__HANDLE__, __FLAG__)      ((__HANDLE__)->Instance->CRL) = ~(__FLAG__)

/**
  * @brief Enable interrupt on ALARM Exti Line 17.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_ENABLE_IT()                  SET_BIT(EXTI->IMR, RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief Disable interrupt on ALARM Exti Line 17.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_DISABLE_IT()                 CLEAR_BIT(EXTI->IMR, RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief Enable event on ALARM Exti Line 17.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_ENABLE_EVENT()               SET_BIT(EXTI->EMR, RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief Disable event on ALARM Exti Line 17.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_DISABLE_EVENT()              CLEAR_BIT(EXTI->EMR, RTC_EXTI_LINE_ALARM_EVENT)


/**
  * @brief  ALARM EXTI line configuration: set falling edge trigger.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_ENABLE_FALLING_EDGE()        SET_BIT(EXTI->FTSR, RTC_EXTI_LINE_ALARM_EVENT)


/**
  * @brief Disable the ALARM Extended Interrupt Falling Trigger.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_DISABLE_FALLING_EDGE()       CLEAR_BIT(EXTI->FTSR, RTC_EXTI_LINE_ALARM_EVENT)


/**
  * @brief  ALARM EXTI line configuration: set rising edge trigger.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE()         SET_BIT(EXTI->RTSR, RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief Disable the ALARM Extended Interrupt Rising Trigger.
  * This parameter can be:
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_DISABLE_RISING_EDGE()        CLEAR_BIT(EXTI->RTSR, RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief  ALARM EXTI line configuration: set rising & falling edge trigger.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_ENABLE_RISING_FALLING_EDGE()      \
do{                                                            \
    __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE();                 \
    __HAL_RTC_ALARM_EXTI_ENABLE_FALLING_EDGE();                \
  } while(0U)

/**
  * @brief Disable the ALARM Extended Interrupt Rising & Falling Trigger.
  * This parameter can be:
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_DISABLE_RISING_FALLING_EDGE()      \
do{                                                             \
    __HAL_RTC_ALARM_EXTI_DISABLE_RISING_EDGE();                 \
    __HAL_RTC_ALARM_EXTI_DISABLE_FALLING_EDGE();                \
  } while(0U)

/**
  * @brief Check whether the specified ALARM EXTI interrupt flag is set or not.
  * @retval EXTI ALARM Line Status.
  */
#define __HAL_RTC_ALARM_EXTI_GET_FLAG()                   (EXTI->PR & (RTC_EXTI_LINE_ALARM_EVENT))

/**
  * @brief Clear the ALARM EXTI flag.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_CLEAR_FLAG()                 (EXTI->PR = (RTC_EXTI_LINE_ALARM_EVENT))

/**
  * @brief Generate a Software interrupt on selected EXTI line.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_GENERATE_SWIT()              SET_BIT(EXTI->SWIER, RTC_EXTI_LINE_ALARM_EVENT)
/**
  * @}
  */

/* Include RTC HAL Extension module */
//#include "stm32f1xx_hal_rtc_ex.h"
// stm32f1xx_hal_rtc_ex.h

//#include "stm32f1xx_hal_def.h"

typedef enum{
	HAL_OK = 0x00U,
	HAL_ERROR = 0x01U,
	HAL_BUSY = 0x02U,
	HAL_TIMEOUT= 0x03U
} HAL_StatusTypeDef;

#define HAL_RTCEx_TamperTimeStampIRQHandler HAL_RTCEx_TamperIRQHandler
#define IS_RTC_TAMPER(__TAMPER__) ((__TAMPER__) == RTC_TAMPER_1)
#define IS_RTC_TAMPER_TRIGGER(__TRIGGER__)  (((__TRIGGER__) == RTC_TAMPERTRIGGER_LOWLEVEL) || \
                                             ((__TRIGGER__) == RTC_TAMPERTRIGGER_HIGHLEVEL))

#define RTC_BKP_NUMBER 10 //YOON

#if RTC_BKP_NUMBER > 10U
#define IS_RTC_BKP(BKP)                   (((BKP) <= (uint32_t)RTC_BKP_DR10) || (((BKP) >= (uint32_t)RTC_BKP_DR11) && ((BKP) <= (uint32_t)RTC_BKP_DR42)))
#else
#define IS_RTC_BKP(BKP)                   ((BKP) <= (uint32_t)RTC_BKP_NUMBER)
#endif
#define IS_RTC_SMOOTH_CALIB_MINUS(__VALUE__) ((__VALUE__) <= 0x0000007FU)
typedef struct
{
  uint32_t Tamper;                      /*!< Specifies the Tamper Pin.
                                             This parameter can be a value of @ref  RTCEx_Tamper_Pins_Definitions */
  uint32_t Trigger;                     /*!< Specifies the Tamper Trigger.
                                             This parameter can be a value of @ref  RTCEx_Tamper_Trigger_Definitions */
} RTC_TamperTypeDef;

#define RTC_TAMPER_1                        BKP_CR_TPE            /*!< Select tamper to be enabled (mainly for legacy purposes) */
/** @defgroup RTCEx_Tamper_Trigger_Definitions Tamper Trigger Definitions
  * @{
  */
#define RTC_TAMPERTRIGGER_LOWLEVEL          BKP_CR_TPAL           /*!< A high level on the TAMPER pin resets all data backup registers (if TPE bit is set) */
#define RTC_TAMPERTRIGGER_HIGHLEVEL         0x00000000U           /*!< A low level on the TAMPER pin resets all data backup registers (if TPE bit is set) */

/** @defgroup RTCEx_Backup_Registers_Definitions Backup Registers Definitions
  * @{
  */
#if RTC_BKP_NUMBER > 0U
#define RTC_BKP_DR1                         0x00000001U
#define RTC_BKP_DR2                         0x00000002U
#define RTC_BKP_DR3                         0x00000003U
#define RTC_BKP_DR4                         0x00000004U
#define RTC_BKP_DR5                         0x00000005U
#define RTC_BKP_DR6                         0x00000006U
#define RTC_BKP_DR7                         0x00000007U
#define RTC_BKP_DR8                         0x00000008U
#define RTC_BKP_DR9                         0x00000009U
#define RTC_BKP_DR10                        0x0000000AU
#endif /* RTC_BKP_NUMBER > 0 */

#if RTC_BKP_NUMBER > 10U
#define RTC_BKP_DR11                        0x00000010U
#define RTC_BKP_DR12                        0x00000011U
#define RTC_BKP_DR13                        0x00000012U
#define RTC_BKP_DR14                        0x00000013U
#define RTC_BKP_DR15                        0x00000014U
#define RTC_BKP_DR16                        0x00000015U
#define RTC_BKP_DR17                        0x00000016U
#define RTC_BKP_DR18                        0x00000017U
#define RTC_BKP_DR19                        0x00000018U
#define RTC_BKP_DR20                        0x00000019U
#define RTC_BKP_DR21                        0x0000001AU
#define RTC_BKP_DR22                        0x0000001BU
#define RTC_BKP_DR23                        0x0000001CU
#define RTC_BKP_DR24                        0x0000001DU
#define RTC_BKP_DR25                        0x0000001EU
#define RTC_BKP_DR26                        0x0000001FU
#define RTC_BKP_DR27                        0x00000020U
#define RTC_BKP_DR28                        0x00000021U
#define RTC_BKP_DR29                        0x00000022U
#define RTC_BKP_DR30                        0x00000023U
#define RTC_BKP_DR31                        0x00000024U
#define RTC_BKP_DR32                        0x00000025U
#define RTC_BKP_DR33                        0x00000026U
#define RTC_BKP_DR34                        0x00000027U
#define RTC_BKP_DR35                        0x00000028U
#define RTC_BKP_DR36                        0x00000029U
#define RTC_BKP_DR37                        0x0000002AU
#define RTC_BKP_DR38                        0x0000002BU
#define RTC_BKP_DR39                        0x0000002CU
#define RTC_BKP_DR40                        0x0000002DU
#define RTC_BKP_DR41                        0x0000002EU
#define RTC_BKP_DR42                        0x0000002FU
#endif /* RTC_BKP_NUMBER > 10 */
/**
  * @brief  Enable the RTC Tamper interrupt.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Tamper interrupt sources to be enabled
  *          This parameter can be any combination of the following values:
  *            @arg RTC_IT_TAMP1: Tamper A interrupt
  * @retval None
  */
#define __HAL_RTC_TAMPER_ENABLE_IT(__HANDLE__, __INTERRUPT__) SET_BIT(BKP->CSR, (__INTERRUPT__))

/**
  * @brief  Disable the RTC Tamper interrupt.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Tamper interrupt sources to be disabled.
  *         This parameter can be any combination of the following values:
  *            @arg RTC_IT_TAMP1: Tamper A interrupt
  * @retval None
  */
#define __HAL_RTC_TAMPER_DISABLE_IT(__HANDLE__, __INTERRUPT__)  CLEAR_BIT(BKP->CSR, (__INTERRUPT__))

/**
  * @brief  Check whether the specified RTC Tamper interrupt has been enabled or not.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Tamper interrupt sources to be checked.
  *         This parameter can be:
  *            @arg  RTC_IT_TAMP1
  * @retval None
  */
#define __HAL_RTC_TAMPER_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)    ((((BKP->CSR) & ((__INTERRUPT__))) != RESET)? SET : RESET)

/**
  * @brief  Get the selected RTC Tamper's flag status.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __FLAG__: specifies the RTC Tamper Flag sources to be enabled or disabled.
  *         This parameter can be:
  *            @arg RTC_FLAG_TAMP1F
  * @retval None
  */
#define __HAL_RTC_TAMPER_GET_FLAG(__HANDLE__, __FLAG__)       ((((BKP->CSR) & (__FLAG__)) != RESET)? SET : RESET)

/**
  * @brief  Get the selected RTC Tamper's flag status.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Tamper interrupt sources to be checked.
  *         This parameter can be:
  *            @arg  RTC_IT_TAMP1
  * @retval None
  */
#define __HAL_RTC_TAMPER_GET_IT(__HANDLE__, __INTERRUPT__)       ((((BKP->CSR) & (BKP_CSR_TEF)) != RESET)? SET : RESET)

/**
  * @brief  Clear the RTC Tamper's pending flags.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __FLAG__: specifies the RTC Tamper Flag sources to be enabled or disabled.
  *         This parameter can be:
  *            @arg RTC_FLAG_TAMP1F
  * @retval None
  */
#define __HAL_RTC_TAMPER_CLEAR_FLAG(__HANDLE__, __FLAG__)     SET_BIT(BKP->CSR, BKP_CSR_CTE | BKP_CSR_CTI)

/**
  * @brief  Enable the RTC Second interrupt.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Second interrupt sources to be enabled
  *          This parameter can be any combination of the following values:
  *            @arg RTC_IT_SEC: Second A interrupt
  * @retval None
  */
#define __HAL_RTC_SECOND_ENABLE_IT(__HANDLE__, __INTERRUPT__)  SET_BIT((__HANDLE__)->Instance->CRH, (__INTERRUPT__))

/**
  * @brief  Disable the RTC Second interrupt.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Second interrupt sources to be disabled.
  *         This parameter can be any combination of the following values:
  *            @arg RTC_IT_SEC: Second A interrupt
  * @retval None
  */
#define __HAL_RTC_SECOND_DISABLE_IT(__HANDLE__, __INTERRUPT__) CLEAR_BIT((__HANDLE__)->Instance->CRH, (__INTERRUPT__))

/**
  * @brief  Check whether the specified RTC Second interrupt has occurred or not.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Second interrupt sources to be enabled or disabled.
  *         This parameter can be:
  *            @arg RTC_IT_SEC: Second A interrupt
  * @retval None
  */
#define __HAL_RTC_SECOND_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)      ((((((__HANDLE__)->Instance->CRH)& ((__INTERRUPT__)))) != RESET)? SET : RESET)

/**
  * @brief  Get the selected RTC Second's flag status.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __FLAG__: specifies the RTC Second Flag sources to be enabled or disabled.
  *          This parameter can be:
  *            @arg RTC_FLAG_SEC
  * @retval None
  */
#define __HAL_RTC_SECOND_GET_FLAG(__HANDLE__, __FLAG__)        (((((__HANDLE__)->Instance->CRL) & (__FLAG__)) != RESET)? SET : RESET)

/**
  * @brief  Clear the RTC Second's pending flags.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __FLAG__: specifies the RTC Second Flag sources to be enabled or disabled.
  *         This parameter can be:
  *            @arg RTC_FLAG_SEC
  * @retval None
  */
#define __HAL_RTC_SECOND_CLEAR_FLAG(__HANDLE__, __FLAG__)      ((__HANDLE__)->Instance->CRL) = ~(__FLAG__)

/**
  * @brief  Enable the RTC Overflow interrupt.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Overflow interrupt sources to be enabled
  *          This parameter can be any combination of the following values:
  *            @arg RTC_IT_OW: Overflow A interrupt
  * @retval None
  */
#define __HAL_RTC_OVERFLOW_ENABLE_IT(__HANDLE__, __INTERRUPT__)  SET_BIT((__HANDLE__)->Instance->CRH, (__INTERRUPT__))

/**
  * @brief  Disable the RTC Overflow interrupt.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Overflow interrupt sources to be disabled.
  *         This parameter can be any combination of the following values:
  *            @arg RTC_IT_OW: Overflow A interrupt
  * @retval None
  */
#define __HAL_RTC_OVERFLOW_DISABLE_IT(__HANDLE__, __INTERRUPT__) CLEAR_BIT((__HANDLE__)->Instance->CRH, (__INTERRUPT__))

/**
  * @brief  Check whether the specified RTC Overflow interrupt has occurred or not.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __INTERRUPT__: specifies the RTC Overflow interrupt sources to be enabled or disabled.
  *         This parameter can be:
  *            @arg RTC_IT_OW: Overflow A interrupt
  * @retval None
  */
#define __HAL_RTC_OVERFLOW_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)    ((((((__HANDLE__)->Instance->CRH)& ((__INTERRUPT__))) ) != RESET)? SET : RESET)

/**
  * @brief  Get the selected RTC Overflow's flag status.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __FLAG__: specifies the RTC Overflow Flag sources to be enabled or disabled.
  *          This parameter can be:
  *            @arg RTC_FLAG_OW
  * @retval None
  */
#define __HAL_RTC_OVERFLOW_GET_FLAG(__HANDLE__, __FLAG__)        (((((__HANDLE__)->Instance->CRL) & (__FLAG__)) != RESET)? SET : RESET)

/**
  * @brief  Clear the RTC Overflow's pending flags.
  * @param  __HANDLE__: specifies the RTC handle.
  * @param  __FLAG__: specifies the RTC Overflow Flag sources to be enabled or disabled.
  *         This parameter can be:
  *            @arg RTC_FLAG_OW
  * @retval None
  */
#define __HAL_RTC_OVERFLOW_CLEAR_FLAG(__HANDLE__, __FLAG__)      ((__HANDLE__)->Instance->CRL) = ~(__FLAG__)

/* RTC Tamper functions *****************************************/
/** @addtogroup RTCEx_Exported_Functions_Group1
  * @{
  */
HAL_StatusTypeDef HAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper);
HAL_StatusTypeDef HAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTamper(RTC_HandleTypeDef *hrtc, uint32_t Tamper);
void              HAL_RTCEx_TamperIRQHandler(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);

/* RTC Second functions *****************************************/
/** @addtogroup RTCEx_Exported_Functions_Group2
  * @{
  */
HAL_StatusTypeDef HAL_RTCEx_SetSecond_IT(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateSecond(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_RTCIRQHandler(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_RTCEventErrorCallback(RTC_HandleTypeDef *hrtc);

/* Extension Control functions ************************************************/
/** @addtogroup RTCEx_Exported_Functions_Group3
  * @{
  */
void              HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t          HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);

HAL_StatusTypeDef HAL_RTCEx_SetSmoothCalib(RTC_HandleTypeDef *hrtc, uint32_t SmoothCalibPeriod, uint32_t SmoothCalibPlusPulses, uint32_t SmouthCalibMinusPulsesValue);

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#if 1
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_HAL_RTC_REGISTER_CALLBACKS == 1)
HAL_StatusTypeDef HAL_RTC_RegisterCallback(RTC_HandleTypeDef *hrtc, HAL_RTC_CallbackIDTypeDef CallbackID, pRTC_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_RTC_UnRegisterCallback(RTC_HandleTypeDef *hrtc, HAL_RTC_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_RTC_REGISTER_CALLBACKS */
/**
  * @}
  */

/* RTC Time and Date functions ************************************************/
/** @addtogroup RTC_Exported_Functions_Group2
  * @{
  */
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
/**
  * @}
  */

/* RTC Alarm functions ********************************************************/
/** @addtogroup RTC_Exported_Functions_Group3
  * @{
  */
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc, uint32_t Alarm);
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format);
void              HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
void              HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);

HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef   HAL_RTC_WaitForSynchro(RTC_HandleTypeDef *hrtc);

#endif


#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_RTC_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
