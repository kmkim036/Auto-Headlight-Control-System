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
extern char *float2str(float x);

extern I2C_TypeDef *gI2Cx;
extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c
extern unsigned char g_bI2CModuleConfigDone;

extern void somedelay(long ulLoop);

#define L3GD20HADDR 0xD4 //0x6A = 7-bit address of L3GD20H Gyro -->d4

//========= L3GD20H Gyro can support both I2C and SPI.
//            We here support only I2C
//[NOTE] for Multiple byte read/write, we should add msb for register address.
//       (it differs from SPI case)
//pin7 : DEN_20H
//pin8 : INT_SENSOR

//#define GYRO_INT_GPIO_PORT                      GPIOE                       /* GPIOE */
//#define GYRO_INT_GPIO_CLK_ENABLE()              __GPIOE_CLK_ENABLE()
//#define GYRO_INT_GPIO_CLK_DISABLE()             __GPIOE_CLK_DISABLE()
//#define GYRO_INT1_PIN                           GPIO_PIN_0                  /* PE.00 */
//#define GYRO_INT1_EXTI_IRQn                     EXTI0_IRQn
//#define GYRO_INT2_PIN                           GPIO_PIN_1                  /* PE.01 */
//#define GYRO_INT2_EXTI_IRQn                     EXTI1_IRQn


/*************************** START REGISTER MAPPING  **************************/
#define L3GD20H_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define L3GD20H_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20H_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20H_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20H_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20H_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define L3GD20H_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define L3GD20H_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define L3GD20H_STATUS_REG_ADDR        0x27  /* Status register */
#define L3GD20H_OUT_X_L_ADDR           0x28  /* Output Register X */
#define L3GD20H_OUT_X_H_ADDR           0x29  /* Output Register X */
#define L3GD20H_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define L3GD20H_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define L3GD20H_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define L3GD20H_OUT_Z_H_ADDR           0x2D  /* Output Register Z */
#define L3GD20H_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define L3GD20H_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

#define L3GD20H_INT1_CFG_ADDR          0x30  /* Interrupt 1 configuration Register */
#define L3GD20H_INT1_SRC_ADDR          0x31  /* Interrupt 1 source Register */
#define L3GD20H_INT1_TSH_XH_ADDR       0x32  /* Interrupt 1 Threshold X register */
#define L3GD20H_INT1_TSH_XL_ADDR       0x33  /* Interrupt 1 Threshold X register */
#define L3GD20H_INT1_TSH_YH_ADDR       0x34  /* Interrupt 1 Threshold Y register */
#define L3GD20H_INT1_TSH_YL_ADDR       0x35  /* Interrupt 1 Threshold Y register */
#define L3GD20H_INT1_TSH_ZH_ADDR       0x36  /* Interrupt 1 Threshold Z register */
#define L3GD20H_INT1_TSH_ZL_ADDR       0x37  /* Interrupt 1 Threshold Z register */
#define L3GD20H_INT1_DURATION_ADDR     0x38  /* Interrupt 1 DURATION register */

#define I_AM_L3GD20                 ((unsigned char)0xD4)
//#define I_AM_L3GD20H_TR              ((unsigned char)0xD5)
#define I_AM_L3GD20H                ((unsigned char)0xD7) //YOON

// @defgroup Power_Mode_selection
#define L3GD20H_MODE_POWERDOWN       ((unsigned char)0x00)
#define L3GD20H_MODE_ACTIVE          ((unsigned char)0x08)
// @defgroup OutPut_DataRate_Selection
#define L3GD20H_OUTPUT_DATARATE_1    ((unsigned char)0x00)
#define L3GD20H_OUTPUT_DATARATE_2    ((unsigned char)0x40)
#define L3GD20H_OUTPUT_DATARATE_3    ((unsigned char)0x80)
#define L3GD20H_OUTPUT_DATARATE_4    ((unsigned char)0xC0)
/** @defgroup Axes_Selection
  * @{
  */
#define L3GD20H_X_ENABLE            ((unsigned char)0x02)
#define L3GD20H_Y_ENABLE            ((unsigned char)0x01)
#define L3GD20H_Z_ENABLE            ((unsigned char)0x04)
#define L3GD20H_AXES_ENABLE         ((unsigned char)0x07)
#define L3GD20H_AXES_DISABLE        ((unsigned char)0x00)

/** @defgroup Bandwidth_Selection
  * @{
  */
#define L3GD20H_BANDWIDTH_1         ((unsigned char)0x00)
#define L3GD20H_BANDWIDTH_2         ((unsigned char)0x10)
#define L3GD20H_BANDWIDTH_3         ((unsigned char)0x20)
#define L3GD20H_BANDWIDTH_4         ((unsigned char)0x30)

/** @defgroup Full_Scale_Selection
  * @{
  */
#define L3GD20H_FULLSCALE_250       ((unsigned char)0x00)
#define L3GD20H_FULLSCALE_500       ((unsigned char)0x10)
#define L3GD20H_FULLSCALE_2000      ((unsigned char)0x20)
#define L3GD20H_FULLSCALE_SELECTION ((unsigned char)0x30)
/** @defgroup Full_Scale_Sensitivity
  * @{
  */
#define L3GD20H_SENSITIVITY_250DPS  ((float)8.75f)         /*!< gyroscope sensitivity with 250 dps full scale [DPS/LSB]  */
#define L3GD20H_SENSITIVITY_500DPS  ((float)17.50f)        /*!< gyroscope sensitivity with 500 dps full scale [DPS/LSB]  */
#define L3GD20H_SENSITIVITY_2000DPS ((float)70.00f)        /*!< gyroscope sensitivity with 2000 dps full scale [DPS/LSB] */
/** @defgroup Block_Data_Update
  * @{
  */
#define L3GD20H_BlockDataUpdate_Continous   ((unsigned char)0x00)
#define L3GD20H_BlockDataUpdate_Single      ((unsigned char)0x80)

/** @defgroup Endian_Data_selection
  * @{
  */
#define L3GD20H_BLE_LSB                     ((unsigned char)0x00)
#define L3GD20H_BLE_MSB	                   ((unsigned char)0x40)
/** @defgroup High_Pass_Filter_status
  * @{
  */
#define L3GD20H_HIGHPASSFILTER_DISABLE      ((unsigned char)0x00)
#define L3GD20H_HIGHPASSFILTER_ENABLE	   ((unsigned char)0x10)
/** @defgroup INT1_INT2_selection
  * @{
  */
#define L3GD20H_INT1                        ((unsigned char)0x00)
#define L3GD20H_INT2                        ((unsigned char)0x01)
/** @defgroup INT1_Interrupt_status
  * @{
  */
#define L3GD20H_INT1INTERRUPT_DISABLE       ((unsigned char)0x00)
#define L3GD20H_INT1INTERRUPT_ENABLE        ((unsigned char)0x80)
/** @defgroup INT2_Interrupt_status
  * @{
  */
#define L3GD20H_INT2INTERRUPT_DISABLE       ((unsigned char)0x00)
#define L3GD20H_INT2INTERRUPT_ENABLE        ((unsigned char)0x08)
/** @defgroup INT1_Interrupt_ActiveEdge
  * @{
  */
#define L3GD20H_INT1INTERRUPT_LOW_EDGE      ((unsigned char)0x20)
#define L3GD20H_INT1INTERRUPT_HIGH_EDGE     ((unsigned char)0x00)
/** @defgroup Boot_Mode_selection
  * @{
  */
#define L3GD20H_BOOT_NORMALMODE             ((unsigned char)0x00)
#define L3GD20H_BOOT_REBOOTMEMORY           ((unsigned char)0x80)

//High_Pass_Filter_Mode
#define L3GD20H_HPM_NORMAL_MODE_RES         ((unsigned char)0x00)
#define L3GD20H_HPM_REF_SIGNAL              ((unsigned char)0x10)
#define L3GD20H_HPM_NORMAL_MODE             ((unsigned char)0x20)
#define L3GD20H_HPM_AUTORESET_INT           ((unsigned char)0x30)

//High_Pass_CUT OFF_Frequency
#define L3GD20H_HPFCF_0              0x00
#define L3GD20H_HPFCF_1              0x01
#define L3GD20H_HPFCF_2              0x02
#define L3GD20H_HPFCF_3              0x03
#define L3GD20H_HPFCF_4              0x04
#define L3GD20H_HPFCF_5              0x05
#define L3GD20H_HPFCF_6              0x06
#define L3GD20H_HPFCF_7              0x07
#define L3GD20H_HPFCF_8              0x08
#define L3GD20H_HPFCF_9              0x09

unsigned char L3GD20H_ReadID(void);
void    L3GD20H_GYRO_RebootCmd(void);

void    L3GD20H_INT1InterruptConfig(unsigned short Int1Config);
void    L3GD20H_EnableIT(unsigned char IntSel);
void    L3GD20H_DisableIT(unsigned char IntSel);

/* High Pass Filter Configuration Functions */
void    L3GD20H_FilterConfig(unsigned char FilterStruct);
void    L3GD20H_FilterCmd(unsigned char HighPassFilterState);
void    L3GD20H_ReadXYZAngRate(float *pfData);
unsigned char L3GD20H_GetDataStatus(void);

/* Gyroscope IO functions */
char stmL3GD20H_GYRO_ConfIo_Init(void);
//void    L3GD20H_GYRO_DeInit(void);
void    L3GD20H_GYRO_Write(unsigned char WriteAddr, unsigned char *pBuffer, unsigned short NumByteToWrite);
void    L3GD20H_GYRO_Read(unsigned char ReadAddr, unsigned char *pBuffer, unsigned short NumByteToRead);

//==================================
short x,y,z;
static float adc_data_float_value = 0.0f;

typedef enum
{
  GYRO_OK = 0,
  GYRO_ERROR = 1,
  GYRO_TIMEOUT = 2
}GYRO_StatusTypeDef;


/*################################ GYROSCOPE #################################*/
/* Multiple byte read/write command */
#define MULTIPLEBYTE_I2C_CMD                        ((unsigned char)0x80) //I2C


#if 0
typedef struct
{
  void       (*Init)(unsigned short);
  unsigned char    (*ReadID)(void);
  void       (*Reset)(void);
  void       (*ConfigIT)(unsigned short);
  void       (*EnableIT)(unsigned char);
  void       (*DisableIT)(unsigned char);
  unsigned char    (*ITStatus)(unsigned short, unsigned short);
  void       (*ClearIT)(unsigned short, unsigned short);
  void       (*FilterConfig)(unsigned char);
  void       (*FilterCmd)(unsigned char);
  void       (*GetXYZ)(float *);
}GYRO_DrvTypeDef;

typedef struct
{
  unsigned char Power_Mode;                         /* Power-down/Sleep/Normal Mode */
  unsigned char Output_DataRate;                    /* OUT data rate */
  unsigned char Axes_Enable;                        /* Axes enable */
  unsigned char Band_Width;                         /* Bandwidth selection */
  unsigned char BlockData_Update;                   /* Block Data Update */
  unsigned char Endianness;                         /* Endian Data selection */
  unsigned char Full_Scale;                         /* Full Scale selection */
}GYRO_InitTypeDef;

/* GYRO High Pass Filter struct */
typedef struct
{
  unsigned char HighPassFilter_Mode_Selection;      /* Internal filter mode */
  unsigned char HighPassFilter_CutOff_Frequency;    /* High pass filter cut-off frequency */
}GYRO_FilterConfigTypeDef;
#endif
/*GYRO Interrupt struct */
typedef struct
{
  unsigned char Latch_Request;                      /* Latch interrupt request into CLICK_SRC register */
  unsigned char Interrupt_Axes;                     /* X, Y, Z Axes Interrupts */
  unsigned char Interrupt_ActiveEdge;               /* Interrupt Active edge */
}GYRO_InterruptConfigTypeDef;

/**
  * @brief  Reads a block of data from the GYRO.
  * @param  pBuffer: pointer to the buffer that receives the data read from the GYRO.
  * @param  ReadAddr: GYRO's internal address to read from.
  * @param  NumByteToRead: Number of bytes to read from the GYRO.
  * @retval None
  */
void L3GD20H_GYRO_Read(unsigned char reg, unsigned char* pBuffer, unsigned short NumByteToRead)
{
  if(NumByteToRead > 0x01)
	  reg |= MULTIPLEBYTE_I2C_CMD;//0x80
  stm_I2C_ReceiveBurstWithRestartCondition(L3GD20HADDR, reg, pBuffer, NumByteToRead);
  return NumByteToRead;
}

/**
  * @brief  Writes one byte to the GYRO.
  * @param  pBuffer: pointer to the buffer  containing the data to be written to the GYRO.
  * @param  WriteAddr : GYRO's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void L3GD20H_GYRO_Write(unsigned char reg, unsigned char* pBuf, unsigned short NumByteToWrite)
{
  /* Configure the MS bit:
     - When 0, the address will remain unchanged in multiple read/write commands.
     - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    reg |= MULTIPLEBYTE_I2C_CMD;//0x80
  }
  stmI2cSendbuf[0] = reg;             // send register address :the Address of the indexed register
  memcpy(&stmI2cSendbuf[1], pBuf, NumByteToWrite);                 // send value to write

  stm_I2C_SendBurst(L3GD20HADDR, stmI2cSendbuf, NumByteToWrite+1);
}

/**
  * @brief  Read ID address of L3GD20
  * @param  None
  * @retval ID name
  */
unsigned char L3GD20H_GYRO_ReadID(void)
{
  unsigned char tmp;
  L3GD20H_GYRO_Read(L3GD20H_WHO_AM_I_ADDR, &tmp,  1);
  return (unsigned char)tmp;
}

/**
  * @brief  Reboot memory content of L3GD20
  * @param  None
  * @retval None
  */
void L3GD20H_GYRO_RebootCmd(void)
{
  unsigned char tmpreg;

  /* Read CTRL_REG5 register */
  L3GD20H_GYRO_Read(L3GD20H_CTRL_REG5_ADDR,&tmpreg,  1);

  /* Enable or Disable the reboot memory */
  tmpreg |= L3GD20H_BOOT_REBOOTMEMORY;

  /* Write value to MEMS CTRL_REG5 register */
  L3GD20H_GYRO_Write(L3GD20H_CTRL_REG5_ADDR,&tmpreg,  1);
}

/** ============ Interrupt ===========================
  * @brief  Configures INT1 interrupt.
  * @param  pIntConfig: pointer to a L3GD20H_InterruptConfig_TypeDef
  *         structure that contains the configuration setting for the L3GD20 Interrupt.
  * @retval None
  */

void L3GD20H_INT1InterruptConfig(unsigned short Int1Config)
{
  unsigned char ctrl_cfr = 0x00, ctrl3 = 0x00;

  /* Read INT1_CFG register */
  L3GD20H_GYRO_Read(L3GD20H_INT1_CFG_ADDR,&ctrl_cfr,  1);

  /* Read CTRL_REG3 register */
  L3GD20H_GYRO_Read(L3GD20H_CTRL_REG3_ADDR, &ctrl3, 1);

  ctrl_cfr &= 0x80;
  ctrl_cfr |= ((unsigned char) Int1Config >> 8);

  ctrl3 &= 0xDF;
  ctrl3 |= ((unsigned char) Int1Config);

  /* Write value to MEMS INT1_CFG register */
  L3GD20H_GYRO_Write(L3GD20H_INT1_CFG_ADDR,&ctrl_cfr,  1);

  /* Write value to MEMS CTRL_REG3 register */
  L3GD20H_GYRO_Write(L3GD20H_CTRL_REG3_ADDR,&ctrl3,  1);
}

/**
  * @brief  Enable INT1 or INT2 interrupt
  * @param  IntSel: choice of INT1 or INT2
  *      This parameter can be:
  *        @arg L3GD20H_INT1
  *        @arg L3GD20H_INT2
  * @retval None
  */
void L3GD20H_GYRO_EnableIT(unsigned char IntSel)
{
  unsigned char tmpreg;

  /* Read CTRL_REG3 register */
  L3GD20H_GYRO_Read(L3GD20H_CTRL_REG3_ADDR,&tmpreg,  1);

  if(IntSel == L3GD20H_INT1)
  {
    tmpreg &= 0x7F;
    tmpreg |= L3GD20H_INT1INTERRUPT_ENABLE;
  }
  else if(IntSel == L3GD20H_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20H_INT2INTERRUPT_ENABLE;
  }

  /* Write value to MEMS CTRL_REG3 register */
  L3GD20H_GYRO_Write(L3GD20H_CTRL_REG3_ADDR, &tmpreg, 1);
}

/**
  * @brief  Disable  INT1 or INT2 interrupt
  * @param  IntSel: choice of INT1 or INT2
  *      This parameter can be:
  *        @arg L3GD20H_INT1
  *        @arg L3GD20H_INT2
  * @retval None
  */
void L3GD20H_GYRO_DisableIT(unsigned char IntSel)
{
  unsigned char tmpreg;

  /* Read CTRL_REG3 register */
  L3GD20H_GYRO_Read(&tmpreg, L3GD20H_CTRL_REG3_ADDR, 1);

  if(IntSel == L3GD20H_INT1)
  {
    tmpreg &= 0x7F;
    tmpreg |= L3GD20H_INT1INTERRUPT_DISABLE;
  }
  else if(IntSel == L3GD20H_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20H_INT2INTERRUPT_DISABLE;
  }

  /* Write value to MEMS CTRL_REG3 register */
  L3GD20H_GYRO_Write(&tmpreg, L3GD20H_CTRL_REG3_ADDR, 1);
}

/**====== filter config
  * @brief  Set High Pass Filter Modality
  * @param  FilterStruct: contains the configuration setting for the L3GD20.
  * @retval None
  */
void L3GD20H_GYRO_FilterConfig(unsigned char FilterStruct)
{
  unsigned char tmpreg;

  /* Read CTRL_REG2 register */
  L3GD20H_GYRO_Read(L3GD20H_CTRL_REG2_ADDR,&tmpreg,  1);

  tmpreg &= 0xC0;

  // Configure MEMS: mode and cutoff frequency
  tmpreg |= FilterStruct;

  // Write value to MEMS CTRL_REG2 register
  L3GD20H_GYRO_Write(L3GD20H_CTRL_REG2_ADDR,&tmpreg,  1);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be:
  *         @arg: L3GD20H_HIGHPASSFILTER_DISABLE
  *         @arg: L3GD20H_HIGHPASSFILTER_ENABLE
  * @retval None
  */
void L3GD20H_GYRO_FilterCmd(unsigned char HighPassFilterState)
{
  unsigned char tmpreg;

  /* Read CTRL_REG5 register */
  L3GD20H_GYRO_Read(L3GD20H_CTRL_REG5_ADDR, &tmpreg,  1);

  tmpreg &= 0xEF;
  tmpreg |= HighPassFilterState;

  /* Write value to MEMS CTRL_REG5 register */
  L3GD20H_GYRO_Write(L3GD20H_CTRL_REG5_ADDR, &tmpreg,  1);
}

/**
  * @brief  Set L3GD20 Initialization.
  * @param  L3GD20H_InitStruct: pointer to a L3GD20H_InitTypeDef structure
  *         that contains the configuration setting for the L3GD20.
  * @retval None
  */
char stmL3GD20H_GYRO_ConfIo_Init(void)
{
	unsigned char id;
  unsigned char ret = GYRO_ERROR;
  unsigned short ctrl = 0x0000;
  unsigned char tmp,i;
  //GYRO_InitTypeDef         L3GD20H_InitStructure;
  //GYRO_FilterConfigTypeDef L3GD20H_FilterStructure = {0,0};

  L3GD20H_GYRO_I2C_GPIO_Config();

  id = L3GD20H_GYRO_ReadID();

  if((id == I_AM_L3GD20) || (id == I_AM_L3GD20H))//_TR))
  {
	  printf("We have L3GD20H @ 0xD4\r\n");

	// Configure MEMS: data rate, power mode, full scale and axes
	//Write value to MEMS CTRL_REG1 register
    //ctrl = (unsigned short) (L3GD20H_MODE_ACTIVE 	//Power_Mode
	  ctrl = (L3GD20H_MODE_ACTIVE 	//Power_Mode
    		| L3GD20H_OUTPUT_DATARATE_1 			//Output_DataRate
    		| L3GD20H_AXES_ENABLE  					//Axes_Enable
    		| L3GD20H_BANDWIDTH_4); 				//Band_Width
    L3GD20H_GYRO_Write(L3GD20H_CTRL_REG1_ADDR,&ctrl,  1);

    //Write value to MEMS CTRL_REG4 register
#if 0
    ctrl |= (unsigned short) ((L3GD20H_BlockDataUpdate_Continous //BlockData_Update (BDU)
    		| L3GD20H_BLE_LSB  						//Endianness
    		| L3GD20H_FULLSCALE_500) << 8); 		//Full_Scale
#else
    ctrl = L3GD20H_BlockDataUpdate_Continous //BlockData_Update (BDU)
    		| L3GD20H_BLE_LSB  						//Endianness
    		| L3GD20H_FULLSCALE_500; 		//Full_Scale
#endif
    L3GD20H_GYRO_Write(L3GD20H_CTRL_REG4_ADDR, &ctrl, 1);

    //set Filter
    ctrl = (unsigned char) ((L3GD20H_HPM_NORMAL_MODE_RES | L3GD20H_HPFCF_0));    // Configure the Gyroscope main parameters
    L3GD20H_GYRO_FilterConfig(ctrl);
    L3GD20H_GYRO_FilterCmd(L3GD20H_HIGHPASSFILTER_ENABLE);


//  	  ctrl = 0x0f;
//	  L3GD20H_GYRO_Write(L3GD20H_CTRL_REG1_ADDR,&ctrl,  1);
	  for(i=0x20;i<0x39;i++){
		  L3GD20H_GYRO_Read(i, &tmp,  1);
		  printf("reg[0x%02x]=0x%02x\r\n",i, tmp);
	  }

    ret = GYRO_OK;
  }
  else {
	  printf("We have no L3GD20H Gyro 0xD4(read val=0x%02x\r\n",id);
  }
  return ret;
}

/**
  * @brief  Reboot memory content of Gyroscope.
  * @param  None
  * @retval None
  */
void L3GD20H_GYRO_Reset(void)
{
    //GyroscopeDrv->Reset();
}
// ============== Get Data ================================
//  * @brief  Get status for L3GD20 data
unsigned char L3GD20H_GYRO_GetDataStatus(void)
{
  unsigned char status;

  /* Read STATUS_REG register */
  L3GD20H_GYRO_Read( L3GD20H_STATUS_REG_ADDR, &status, 1);

  return status;
}

/**
* @brief  Calculate the L3GD20 angular data.
* @param  pfData: Data out pointer
* @retval None
*/
void stmL3GD20H_GYRO_ReadXYZAngRate(float *pfData)
{
  unsigned char tmpbuffer[6] ={0};
  short RawData[3] = {0};
  unsigned char reg4val = 0;
  float sensitivity = 0;
  int i =0;

  L3GD20H_GYRO_Read(L3GD20H_CTRL_REG4_ADDR,&reg4val,1);
  printf("reg4=0x%02x\r\n", reg4val);
/*
  L3GD20H_GYRO_Read(L3GD20H_OUT_X_L_ADDR,&tmpbuffer[0],1);
  L3GD20H_GYRO_Read(L3GD20H_OUT_X_H_ADDR,&tmpbuffer[1],1);
  L3GD20H_GYRO_Read(L3GD20H_OUT_Y_L_ADDR,&tmpbuffer[2],1);
  L3GD20H_GYRO_Read(L3GD20H_OUT_Y_H_ADDR,&tmpbuffer[3],1);
  L3GD20H_GYRO_Read(L3GD20H_OUT_Z_L_ADDR,&tmpbuffer[4],1);
  L3GD20H_GYRO_Read(L3GD20H_OUT_Z_H_ADDR,&tmpbuffer[5],1);

  //for (i=0;i<6;i++)	  printf("%02x ", tmpbuffer[i]);
  //printf("\r\n");
*/
  L3GD20H_GYRO_Read(L3GD20H_OUT_X_L_ADDR,tmpbuffer,6);//Read {X_Low, X_High, Y_Low, Y_High, Z_Low, Z_High}

  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(reg4val & L3GD20H_BLE_MSB))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(short)(((unsigned short)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(short)(((unsigned short)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }

  /* Switch the sensitivity value set in the CRTL4 */
  switch(reg4val & L3GD20H_FULLSCALE_SELECTION)
  {
  case L3GD20H_FULLSCALE_250:
    sensitivity=L3GD20H_SENSITIVITY_250DPS;
    break;

  case L3GD20H_FULLSCALE_500:
    sensitivity=L3GD20H_SENSITIVITY_500DPS;
    break;

  case L3GD20H_FULLSCALE_2000:
    sensitivity=L3GD20H_SENSITIVITY_2000DPS;
    break;
  default:
   sensitivity = 0.00875;
   break;
  }

  printf("sensitivity=%s\r\n", float2str(sensitivity));

  /* Divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)(RawData[i] * sensitivity);
  }
  printf("rawData[0,1,2]=(%d, %d, %d)\r\n", RawData[0],RawData[1],RawData[2]);
}
//===================== INIT ================================
//spi case not....
void L3GD20H_GYRO_I2C_GPIO_Config(void)
{
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

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
	//stmUser_LED_GPIO_setup(); //Accessory LED
	//stmConfirmLEDBlink();

	//RCT6
	//pin7(PC4) : DEN_20H -- OUT
	//pin8(PC15) : INT_SENSOR

	//PC4-oDEN to the GYRO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_4); //Gyroscope Data Enable to the GYRO.

	//PC15- Interrupt... TBD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void L3GD20H_GYRO_Loop(){

	float fData[3]; //floating array
	char fx[20],fy[20],fz[20];

	printf("L3GD20H Gyro Test with I2C\r\n");

	stmL3GD20H_GYRO_ConfIo_Init();
	//L3GD20H_INT1InterruptConfig();
	//L3GD20H_GYRO_EnableIT();

	//L3GD20H_GYRO_RebootCmd();

	while(1){
		stmL3GD20H_GYRO_ReadXYZAngRate(fData); //Get XYZ angular acceleration.
		strcpy(fx, float2str(fData[0]));
		strcpy(fy, float2str(fData[1]));
		strcpy(fz, float2str(fData[2]));

		printf("[x,y,z] = (%s,%s,%s)\r\n",fx,fy,fz);
		delayms(300);
	}
}


