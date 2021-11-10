// Use the I2C bus for Si5351a
/*
#include <string.h>
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
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#include "yInc.h"
#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurst(unsigned char SlaveAddress, unsigned char *buf, unsigned char nbyte);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);

extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c

//============================================================================================================
//[NOTE] There are 2 different 7-bit addresses of Si5351A. (Very Curious. It makes us very angry.)
//#define SI5351aADDR7 0x60              //0110 0000 (60) -- Datasheet.  (Marking on the chip i == BLC2 702)
#define SI5351aADDR7 0x6F                //0110 1111 (6F) -- We have this. (Marking on the chip i == A8GD 507)

//#define SI5351aADDR8 0xc0 //Si5351a/c
#define SI5351aADDR8 0xDE 	//Si5351a/c -- We use it. However, the 0xc0 may be correct on different chips.

//#define SI5351aADDR7 0x62//Si5351a-b04486-GT for XMOS preprogrammed.

//============================================================================================================
#define ERROR_NONE 0
#define ERROR_INVALIDPARAMETER 1
#define ERROR_DEVICENOTINITIALISED 2

/* See http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf for registers 26..41 */
enum
{
  SI5351_REGISTER_0_DEVICE_STATUS                       = 0,
  SI5351_REGISTER_1_INTERRUPT_STATUS_STICKY             = 1,
  SI5351_REGISTER_2_INTERRUPT_STATUS_MASK               = 2,
  SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL               = 3,
  SI5351_REGISTER_9_OEB_PIN_ENABLE_CONTROL              = 9,
  SI5351_REGISTER_15_PLL_INPUT_SOURCE                   = 15,
  SI5351_REGISTER_16_CLK0_CONTROL                       = 16,
  SI5351_REGISTER_17_CLK1_CONTROL                       = 17,
  SI5351_REGISTER_18_CLK2_CONTROL                       = 18,
  SI5351_REGISTER_19_CLK3_CONTROL                       = 19,
  SI5351_REGISTER_20_CLK4_CONTROL                       = 20,
  SI5351_REGISTER_21_CLK5_CONTROL                       = 21,
  SI5351_REGISTER_22_CLK6_CONTROL                       = 22,
  SI5351_REGISTER_23_CLK7_CONTROL                       = 23,
  SI5351_REGISTER_24_CLK3_0_DISABLE_STATE               = 24,
  SI5351_REGISTER_25_CLK7_4_DISABLE_STATE               = 25,
  SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1           = 42,
  SI5351_REGISTER_43_MULTISYNTH0_PARAMETERS_2           = 43,
  SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3           = 44,
  SI5351_REGISTER_45_MULTISYNTH0_PARAMETERS_4           = 45,
  SI5351_REGISTER_46_MULTISYNTH0_PARAMETERS_5           = 46,
  SI5351_REGISTER_47_MULTISYNTH0_PARAMETERS_6           = 47,
  SI5351_REGISTER_48_MULTISYNTH0_PARAMETERS_7           = 48,
  SI5351_REGISTER_49_MULTISYNTH0_PARAMETERS_8           = 49,
  SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1           = 50,
  SI5351_REGISTER_51_MULTISYNTH1_PARAMETERS_2           = 51,
  SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3           = 52,
  SI5351_REGISTER_53_MULTISYNTH1_PARAMETERS_4           = 53,
  SI5351_REGISTER_54_MULTISYNTH1_PARAMETERS_5           = 54,
  SI5351_REGISTER_55_MULTISYNTH1_PARAMETERS_6           = 55,
  SI5351_REGISTER_56_MULTISYNTH1_PARAMETERS_7           = 56,
  SI5351_REGISTER_57_MULTISYNTH1_PARAMETERS_8           = 57,
  SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1           = 58,
  SI5351_REGISTER_59_MULTISYNTH2_PARAMETERS_2           = 59,
  SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3           = 60,
  SI5351_REGISTER_61_MULTISYNTH2_PARAMETERS_4           = 61,
  SI5351_REGISTER_62_MULTISYNTH2_PARAMETERS_5           = 62,
  SI5351_REGISTER_63_MULTISYNTH2_PARAMETERS_6           = 63,
  SI5351_REGISTER_64_MULTISYNTH2_PARAMETERS_7           = 64,
  SI5351_REGISTER_65_MULTISYNTH2_PARAMETERS_8           = 65,
  SI5351_REGISTER_66_MULTISYNTH3_PARAMETERS_1           = 66,
  SI5351_REGISTER_67_MULTISYNTH3_PARAMETERS_2           = 67,
  SI5351_REGISTER_68_MULTISYNTH3_PARAMETERS_3           = 68,
  SI5351_REGISTER_69_MULTISYNTH3_PARAMETERS_4           = 69,
  SI5351_REGISTER_70_MULTISYNTH3_PARAMETERS_5           = 70,
  SI5351_REGISTER_71_MULTISYNTH3_PARAMETERS_6           = 71,
  SI5351_REGISTER_72_MULTISYNTH3_PARAMETERS_7           = 72,
  SI5351_REGISTER_73_MULTISYNTH3_PARAMETERS_8           = 73,
  SI5351_REGISTER_74_MULTISYNTH4_PARAMETERS_1           = 74,
  SI5351_REGISTER_75_MULTISYNTH4_PARAMETERS_2           = 75,
  SI5351_REGISTER_76_MULTISYNTH4_PARAMETERS_3           = 76,
  SI5351_REGISTER_77_MULTISYNTH4_PARAMETERS_4           = 77,
  SI5351_REGISTER_78_MULTISYNTH4_PARAMETERS_5           = 78,
  SI5351_REGISTER_79_MULTISYNTH4_PARAMETERS_6           = 79,
  SI5351_REGISTER_80_MULTISYNTH4_PARAMETERS_7           = 80,
  SI5351_REGISTER_81_MULTISYNTH4_PARAMETERS_8           = 81,
  SI5351_REGISTER_82_MULTISYNTH5_PARAMETERS_1           = 82,
  SI5351_REGISTER_83_MULTISYNTH5_PARAMETERS_2           = 83,
  SI5351_REGISTER_84_MULTISYNTH5_PARAMETERS_3           = 84,
  SI5351_REGISTER_85_MULTISYNTH5_PARAMETERS_4           = 85,
  SI5351_REGISTER_86_MULTISYNTH5_PARAMETERS_5           = 86,
  SI5351_REGISTER_87_MULTISYNTH5_PARAMETERS_6           = 87,
  SI5351_REGISTER_88_MULTISYNTH5_PARAMETERS_7           = 88,
  SI5351_REGISTER_89_MULTISYNTH5_PARAMETERS_8           = 89,
  SI5351_REGISTER_90_MULTISYNTH6_PARAMETERS             = 90,
  SI5351_REGISTER_91_MULTISYNTH7_PARAMETERS             = 91,
  SI5351_REGISTER_092_CLOCK_6_7_OUTPUT_DIVIDER          = 92,
  SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET         = 165,
  SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET         = 166,
  SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET         = 167,
  SI5351_REGISTER_168_CLK3_INITIAL_PHASE_OFFSET         = 168,
  SI5351_REGISTER_169_CLK4_INITIAL_PHASE_OFFSET         = 169,
  SI5351_REGISTER_170_CLK5_INITIAL_PHASE_OFFSET         = 170,
  SI5351_REGISTER_177_PLL_RESET                         = 177,
  SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE	= 183
};

typedef enum
{
  SI5351_PLL_A = 0,
  SI5351_PLL_B,
} si5351PLL_t;

typedef enum
{
  SI5351_CRYSTAL_LOAD_6PF  = (1<<6),
  SI5351_CRYSTAL_LOAD_8PF  = (2<<6),
  SI5351_CRYSTAL_LOAD_10PF = (3<<6)
} si5351CrystalLoad_t;

typedef enum
{
  SI5351_CRYSTAL_FREQ_25MHZ = (25000000),
  SI5351_CRYSTAL_FREQ_27MHZ = (27000000)
} si5351CrystalFreq_t;

typedef enum
{
  SI5351_MULTISYNTH_DIV_4  = 4,
  SI5351_MULTISYNTH_DIV_6  = 6,
  SI5351_MULTISYNTH_DIV_8  = 8
} si5351MultisynthDiv_t;

typedef enum
{
  SI5351_R_DIV_1   = 0,
  SI5351_R_DIV_2   = 1,
  SI5351_R_DIV_4   = 2,
  SI5351_R_DIV_8   = 3,
  SI5351_R_DIV_16  = 4,
  SI5351_R_DIV_32  = 5,
  SI5351_R_DIV_64  = 6,
  SI5351_R_DIV_128 = 7,
} si5351RDiv_t;

typedef struct
{
  bool                initialised;
  si5351CrystalFreq_t crystalFreq;
  si5351CrystalLoad_t crystalLoad;
  uint32_t            crystalPPM;
  bool                plla_configured;
  uint32_t            plla_freq;
  bool                pllb_configured;
  uint32_t            pllb_freq;
} si5351Config_t;


#define SI_PLL_RESET		177

/* Test setup from SI5351 ClockBuilder
 * -----------------------------------
 * XTAL:      25     MHz
 * Channel 0: 120.00 MHz
 * Channel 1: 12.00  MHz
 * Channel 2: 13.56  MHz
 */
static const uint8_t m_si5351_regs_15to92_149to170[100][2] =
{
  {  15, 0x00 },    /* Input source = crystal for PLLA and PLLB */
  {  16, 0x4F },    /* CLK0 Control: 8mA drive, Multisynth 0 as CLK0 source, Clock not inverted, Source = PLLA, Multisynth 0 in integer mode, clock powered up */
  {  17, 0x4F },    /* CLK1 Control: 8mA drive, Multisynth 1 as CLK1 source, Clock not inverted, Source = PLLA, Multisynth 1 in integer mode, clock powered up */
  {  18, 0x6F },    /* CLK2 Control: 8mA drive, Multisynth 2 as CLK2 source, Clock not inverted, Source = PLLB, Multisynth 2 in integer mode, clock powered up */
  {  19, 0x80 },    /* CLK3 Control: Not used ... clock powered down */
  {  20, 0x80 },    /* CLK4 Control: Not used ... clock powered down */
  {  21, 0x80 },    /* CLK5 Control: Not used ... clock powered down */
  {  22, 0x80 },    /* CLK6 Control: Not used ... clock powered down */
  {  23, 0x80 },    /* CLK7 Control: Not used ... clock powered down */
  {  24, 0x00 },    /* Clock disable state 0..3 (low when disabled) */
  {  25, 0x00 },    /* Clock disable state 4..7 (low when disabled) */
  /* PLL_A Setup */
  {  26, 0x00 },
  {  27, 0x05 },
  {  28, 0x00 },
  {  29, 0x0C },
  {  30, 0x66 },
  {  31, 0x00 },
  {  32, 0x00 },
  {  33, 0x02 },
  /* PLL_B Setup */
  {  34, 0x02 },
  {  35, 0x71 },
  {  36, 0x00 },
  {  37, 0x0C },
  {  38, 0x1A },
  {  39, 0x00 },
  {  40, 0x00 },
  {  41, 0x86 },
  /* Multisynth Setup */
  {  42, 0x00 },
  {  43, 0x01 },
  {  44, 0x00 },
  {  45, 0x01 },
  {  46, 0x00 },
  {  47, 0x00 },
  {  48, 0x00 },
  {  49, 0x00 },
  {  50, 0x00 },
  {  51, 0x01 },
  {  52, 0x00 },
  {  53, 0x1C },
  {  54, 0x00 },
  {  55, 0x00 },
  {  56, 0x00 },
  {  57, 0x00 },
  {  58, 0x00 },
  {  59, 0x01 },
  {  60, 0x00 },
  {  61, 0x18 },
  {  62, 0x00 },
  {  63, 0x00 },
  {  64, 0x00 },
  {  65, 0x00 },
  {  66, 0x00 },
  {  67, 0x00 },
  {  68, 0x00 },
  {  69, 0x00 },
  {  70, 0x00 },
  {  71, 0x00 },
  {  72, 0x00 },
  {  73, 0x00 },
  {  74, 0x00 },
  {  75, 0x00 },
  {  76, 0x00 },
  {  77, 0x00 },
  {  78, 0x00 },
  {  79, 0x00 },
  {  80, 0x00 },
  {  81, 0x00 },
  {  82, 0x00 },
  {  83, 0x00 },
  {  84, 0x00 },
  {  85, 0x00 },
  {  86, 0x00 },
  {  87, 0x00 },
  {  88, 0x00 },
  {  89, 0x00 },
  {  90, 0x00 },
  {  91, 0x00 },
  {  92, 0x00 },
  /* Misc Config Register */
  { 149, 0x00 },
  { 150, 0x00 },
  { 151, 0x00 },
  { 152, 0x00 },
  { 153, 0x00 },
  { 154, 0x00 },
  { 155, 0x00 },
  { 156, 0x00 },
  { 157, 0x00 },
  { 158, 0x00 },
  { 159, 0x00 },
  { 160, 0x00 },
  { 161, 0x00 },
  { 162, 0x00 },
  { 163, 0x00 },
  { 164, 0x00 },
  { 165, 0x00 },
  { 166, 0x00 },
  { 167, 0x00 },
  { 168, 0x00 },
  { 169, 0x00 },
  { 170, 0x00 }
};


si5351Config_t m_si5351Config;

//proto
char stm_Si5351_setupMultisynth(unsigned char     output,
                                       si5351PLL_t pllSource,
                                       uint32_t    div,
                                       uint32_t    num,
                                       uint32_t    denom);

//==================
//write to a single register
char stm_i2c_si5351a_write_single_reg(unsigned int regaddr, unsigned char data ) {
    int ret;

    stmI2cSendbuf[0] = regaddr;
    stmI2cSendbuf[1] = data; //a byte
    ret = stm_I2C_SendBurst(SI5351aADDR8, &stmI2cSendbuf[0], 2);
    return ret;
}
u8 stm_i2c_si5351a_read_single_reg(unsigned int regaddr, unsigned char *rdata ) {
	u8  retlen = 0;
	stmI2cSendbuf[0] = regaddr;
	stm_I2C_SendBurst(SI5351aADDR8, &stmI2cSendbuf[0], 1); //write reg.

	retlen = stm_I2C_ReceiveBurst(SI5351aADDR8, &stmI2cRecvbuf[0], 1);     //Read a single byte
	if(retlen==0){
		printf("i2c error\r\n");
		return 0;
	}
    *rdata  = stmI2cRecvbuf[0];
    return retlen;
}
//write one or more contiguous registers. data[0] should be the first value.
void i2c_si5351a_write_reg(unsigned int regaddr, unsigned char data[], unsigned char dataleng ) {
    int i;

    stmI2cSendbuf[0] = regaddr;
    //fill data
    for(i=0;i<dataleng;i++)
    	stmI2cSendbuf[1+i] = data[i]; //a byte
    stm_I2C_SendBurst(SI5351aADDR8, &stmI2cSendbuf[0], dataleng+1);
}
//====================================
void stm_Si5351_Init(void)
{
  m_si5351Config.initialised     = false;
  m_si5351Config.crystalFreq     = SI5351_CRYSTAL_FREQ_25MHZ;
  m_si5351Config.crystalLoad     = SI5351_CRYSTAL_LOAD_10PF;
  m_si5351Config.crystalPPM      = 30;
  m_si5351Config.plla_configured = false;
  m_si5351Config.plla_freq       = 0;
  m_si5351Config.pllb_configured = false;
  m_si5351Config.pllb_freq       = 0;

  /* Disable all outputs setting CLKx_DIS high */
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF);

  /* Power down all output drivers */
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_16_CLK0_CONTROL, 0x80);
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_17_CLK1_CONTROL, 0x80);
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_18_CLK2_CONTROL, 0x80);
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_19_CLK3_CONTROL, 0x80);
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_20_CLK4_CONTROL, 0x80);
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_21_CLK5_CONTROL, 0x80);
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_22_CLK6_CONTROL, 0x80);
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_23_CLK7_CONTROL, 0x80);

  /* Set the load capacitance for the XTAL */
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE, m_si5351Config.crystalLoad);

  /* Set interrupt masks as required (see Register 2 description in AN619).
     By default, ClockBuilder Desktop sets this register to 0x18.
     Note that the least significant nibble must remain 0x8, but the most
     significant nibble may be modified to suit your needs. */

  /* Reset the PLL config fields just in case we call init again */
  m_si5351Config.plla_configured = false;
  m_si5351Config.plla_freq = 0;
  m_si5351Config.pllb_configured = false;
  m_si5351Config.pllb_freq = 0;

  /* All done! */
  m_si5351Config.initialised = true;

  return ERROR_NONE;
}

/*!
    @brief  Configures the Si5351 with config settings generated in
            ClockBuilder. You can use this function to make sure that
            your HW is properly configure and that there are no problems
            with the board itself.

            Running this function should provide the following output:
            * Channel 0: 120.00 MHz
            * Channel 1: 12.00  MHz
            * Channel 2: 13.56  MHz

    @note   This will overwrite all of the config registers!
*/

char stm_Si5351_setClockBuilderData(void)
{
  uint16_t i = 0;

  /* Make sure we've called init first */
  //ASSERT(m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED);

  /* Disable all outputs setting CLKx_DIS high */
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF);

  /* Writes configuration data to device using the register map contents
	 generated by ClockBuilder Desktop (registers 15-92 + 149-170) */
  for (i=0; i<sizeof(m_si5351_regs_15to92_149to170)/2; i++)
  {
    stm_i2c_si5351a_write_single_reg( m_si5351_regs_15to92_149to170[i][0],
                          m_si5351_regs_15to92_149to170[i][1] );
  }

  /* Apply soft reset */
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_177_PLL_RESET, 0xAC);

  /* Enabled desired outputs (see Register 3) */
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0x00);

  return ERROR_NONE;
}


//===================
/*!
    @brief  Sets the multiplier for the specified PLL

    @param  pll   The PLL to configure, which must be one of the following:
                  - SI5351_PLL_A
                  - SI5351_PLL_B
    @param  mult  The PLL integer multiplier (must be between 15 and 90)
    @param  num   The 20-bit numerator for fractional output (0..1,048,575).
                  Set this to '0' for integer output.
    @param  denom The 20-bit denominator for fractional output (1..1,048,575).
                  Set this to '1' or higher to avoid divider by zero errors.

    @section PLL Configuration

    fVCO is the PLL output, and must be between 600..900MHz, where:

        fVCO = fXTAL * (a+(b/c))

    fXTAL = the crystal input frequency
    a     = an integer between 15 and 90
    b     = the fractional numerator (0..1,048,575)
    c     = the fractional denominator (1..1,048,575)

    NOTE: Try to use integers whenever possible to avoid clock jitter
    (only use the a part, setting b to '0' and c to '1').

    See: http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
*/
//========
char stm_Si5351_setupPLL(si5351PLL_t pll,
                                uint8_t     mult,
                                uint32_t    num,
                                uint32_t    denom)
{
  uint32_t P1;       /* PLL config register P1 */
  uint32_t P2;	     /* PLL config register P2 */
  uint32_t P3;	     /* PLL config register P3 */

  /* Basic validation */
  //ASSERT( m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED );
  //ASSERT( (mult > 14) && (mult < 91), ERROR_INVALIDPARAMETER ); /* mult = 15..90 */
  //ASSERT( denom > 0,                  ERROR_INVALIDPARAMETER ); /* Avoid divide by zero */
  //ASSERT( num <= 0xFFFFF,             ERROR_INVALIDPARAMETER ); /* 20-bit limit */
  //ASSERT( denom <= 0xFFFFF,           ERROR_INVALIDPARAMETER ); /* 20-bit limit */

  /* Feedback Multisynth Divider Equation
   *
   * where: a = mult, b = num and c = denom
   *
   * P1 register is an 18-bit value using following formula:
   *
   * 	P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
   *
   * P2 register is a 20-bit value using the following formula:
   *
   * 	P2[19:0] = 128 * num - denom * floor(128*(num/denom))
   *
   * P3 register is a 20-bit value using the following formula:
   *
   * 	P3[19:0] = denom
   */

  /* Set the main PLL config registers */
  if (num == 0)
  {
    /* Integer mode */
    P1 = 128 * mult - 512;
    P2 = num;
    P3 = denom;
  }
  else
  {
    /* Fractional mode */
    P1 = (uint32_t)(128 * mult + floor(128 * ((float)num/(float)denom)) - 512);
    P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P3 = denom;
  }

  /* Get the appropriate starting point for the PLL registers */
  uint8_t baseaddr = (pll == SI5351_PLL_A ? 26 : 34);

  /* The datasheet is a nightmare of typos and inconsistencies here! */
  stm_i2c_si5351a_write_single_reg( baseaddr,   (P3 & 0x0000FF00) >> 8);
  stm_i2c_si5351a_write_single_reg( baseaddr+1, (P3 & 0x000000FF));
  stm_i2c_si5351a_write_single_reg( baseaddr+2, (P1 & 0x00030000) >> 16);
  stm_i2c_si5351a_write_single_reg( baseaddr+3, (P1 & 0x0000FF00) >> 8);
  stm_i2c_si5351a_write_single_reg( baseaddr+4, (P1 & 0x000000FF));
  stm_i2c_si5351a_write_single_reg( baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) );
  stm_i2c_si5351a_write_single_reg( baseaddr+6, (P2 & 0x0000FF00) >> 8);
  stm_i2c_si5351a_write_single_reg( baseaddr+7, (P2 & 0x000000FF));

  /* Reset both PLLs */
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_177_PLL_RESET, (1<<7) | (1<<5) );

  /* Store the frequency settings for use with the Multisynth helper */
  if (pll == SI5351_PLL_A)
  {
    float fvco = m_si5351Config.crystalFreq * (mult + ( (float)num / (float)denom ));
    m_si5351Config.plla_configured = true;
    m_si5351Config.plla_freq = (uint32_t)floor(fvco);
  }
  else
  {
    float fvco = m_si5351Config.crystalFreq * (mult + ( (float)num / (float)denom ));
    m_si5351Config.pllb_configured = true;
    m_si5351Config.pllb_freq = (uint32_t)floor(fvco);
  }

  return ERROR_NONE;
}

//=======================
/*!
  @brief  Sets the multiplier for the specified PLL using integer values

  @param  pll   The PLL to configure, which must be one of the following:
                - SI5351_PLL_A
                - SI5351_PLL_B
  @param  mult  The PLL integer multiplier (must be between 15 and 90)
*/
//=============================
char stm_Si5351_setupPLLInt(si5351PLL_t pll, uint8_t mult)
{
  return stm_Si5351_setupPLL(pll, mult, 0, 1);
}
//===========
/*!
    @brief  Configures the Multisynth divider using integer output.

    @param  output    The output channel to use (0..2)
    @param  pllSource	The PLL input source to use, which must be one of:
                      - SI5351_PLL_A
                      - SI5351_PLL_B
    @param  div       The integer divider for the Multisynth output,
                      which must be one of the following values:
                      - SI5351_MULTISYNTH_DIV_4
                      - SI5351_MULTISYNTH_DIV_6
                      - SI5351_MULTISYNTH_DIV_8
*/
//===========
char stm_Si5351_setupMultisynthInt(uint8_t               output,
                                          si5351PLL_t           pllSource,
                                          si5351MultisynthDiv_t div)
{
  return stm_Si5351_setupMultisynth(output, pllSource, div, 0, 1);
}


char stm_Si5351_setupRdiv(unsigned char  output, si5351RDiv_t div) {
  //ASSERT( output < 3,  ERROR_INVALIDPARAMETER);  /* Channel range */

  unsigned char Rreg, regval;

  if (output == 0) Rreg = SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3;
  if (output == 1) Rreg = SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3;
  if (output == 2) Rreg = SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3;

  stm_i2c_si5351a_read_single_reg(Rreg, &regval);// read8(Rreg, &regval);

  regval &= 0x0F;
  uint8_t divider = div;
  divider &= 0x07;
  divider <<= 4;
  regval |= divider;
  return stm_i2c_si5351a_write_single_reg(Rreg, regval);
}

//===========
/*!
    @brief  Configures the Multisynth divider, which determines the
            output clock frequency based on the specified PLL input.

    @param  output    The output channel to use (0..2)
    @param  pllSource	The PLL input source to use, which must be one of:
                      - SI5351_PLL_A
                      - SI5351_PLL_B
    @param  div       The integer divider for the Multisynth output.
                      If pure integer values are used, this value must
                      be one of:
                      - SI5351_MULTISYNTH_DIV_4
                      - SI5351_MULTISYNTH_DIV_6
                      - SI5351_MULTISYNTH_DIV_8
                      If fractional output is used, this value must be
                      between 8 and 900.
    @param  num       The 20-bit numerator for fractional output
                      (0..1,048,575). Set this to '0' for integer output.
    @param  denom     The 20-bit denominator for fractional output
                      (1..1,048,575). Set this to '1' or higher to
                      avoid divide by zero errors.

    @section Output Clock Configuration

    The multisynth dividers are applied to the specified PLL output,
    and are used to reduce the PLL output to a valid range (500kHz
    to 160MHz). The relationship can be seen in this formula, where
    fVCO is the PLL output frequency and MSx is the multisynth
    divider:

        fOUT = fVCO / MSx

    Valid multisynth dividers are 4, 6, or 8 when using integers,
    or any fractional values between 8 + 1/1,048,575 and 900 + 0/1

    The following formula is used for the fractional mode divider:

        a + b / c

    a = The integer value, which must be 4, 6 or 8 in integer mode (MSx_INT=1)
        or 8..900 in fractional mode (MSx_INT=0).
    b = The fractional numerator (0..1,048,575)
    c = The fractional denominator (1..1,048,575)

    @note   Try to use integers whenever possible to avoid clock jitter

    @note   For output frequencies > 150MHz, you must set the divider
            to 4 and adjust to PLL to generate the frequency (for example
            a PLL of 640 to generate a 160MHz output clock). This is not
            yet supported in the driver, which limits frequencies to
            500kHz .. 150MHz.

    @note   For frequencies below 500kHz (down to 8kHz) Rx_DIV must be
            used, but this isn't currently implemented in the driver.
*/
//===========
char stm_Si5351_setupMultisynth(unsigned char     output,
                                       si5351PLL_t pllSource,
                                       uint32_t    div,
                                       uint32_t    num,
                                       uint32_t    denom)
{
  uint32_t P1;       /* Multisynth config register P1 */
  uint32_t P2;	     /* Multisynth config register P2 */
  uint32_t P3;	     /* Multisynth config register P3 */

  /* Basic validation */
  //ASSERT( m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED);
  //ASSERT( output < 3,                 ERROR_INVALIDPARAMETER);  /* Channel range */
  //ASSERT( div > 3,                    ERROR_INVALIDPARAMETER);  /* Divider integer value */
  //ASSERT( div < 901,                  ERROR_INVALIDPARAMETER);  /* Divider integer value */
  //ASSERT( denom > 0,                  ERROR_INVALIDPARAMETER ); /* Avoid divide by zero */
  //ASSERT( num <= 0xFFFFF,             ERROR_INVALIDPARAMETER ); /* 20-bit limit */
  //ASSERT( denom <= 0xFFFFF,           ERROR_INVALIDPARAMETER ); /* 20-bit limit */

  /* Make sure the requested PLL has been initialised */
  //if (pllSource == SI5351_PLL_A) {
  //  ASSERT(m_si5351Config.plla_configured = true, ERROR_INVALIDPARAMETER);
  //} else {
  //  ASSERT(m_si5351Config.pllb_configured = true, ERROR_INVALIDPARAMETER);
  // }

  /* Output Multisynth Divider Equations
   *
   * where: a = div, b = num and c = denom
   *
   * P1 register is an 18-bit value using following formula:
   *
   * 	P1[17:0] = 128 * a + floor(128*(b/c)) - 512
   *
   * P2 register is a 20-bit value using the following formula:
   *
   * 	P2[19:0] = 128 * b - c * floor(128*(b/c))
   *
   * P3 register is a 20-bit value using the following formula:
   *
   * 	P3[19:0] = c
   */

  /* Set the main PLL config registers */
  if (num == 0)
  {
    /* Integer mode */
    P1 = 128 * div - 512;
    P2 = num;
    P3 = denom;
  }
  else
  {
    /* Fractional mode */
    P1 = (uint32_t)(128 * div + floor(128 * ((float)num/(float)denom)) - 512);
    P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P3 = denom;
  }

  /* Get the appropriate starting point for the PLL registers */
  uint8_t baseaddr = 0;
  switch (output)
  {
    case 0:
      baseaddr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
      break;
    case 1:
      baseaddr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
      break;
    case 2:
      baseaddr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
      break;
  }

  /* Set the MSx config registers */
  stm_i2c_si5351a_write_single_reg( baseaddr,   (P3 & 0x0000FF00) >> 8);
  stm_i2c_si5351a_write_single_reg( baseaddr+1, (P3 & 0x000000FF));
  stm_i2c_si5351a_write_single_reg( baseaddr+2, (P1 & 0x00030000) >> 16);	/* ToDo: Add DIVBY4 (>150MHz) and R0 support (<500kHz) later */
  stm_i2c_si5351a_write_single_reg( baseaddr+3, (P1 & 0x0000FF00) >> 8);
  stm_i2c_si5351a_write_single_reg( baseaddr+4, (P1 & 0x000000FF));
  stm_i2c_si5351a_write_single_reg( baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) );
  stm_i2c_si5351a_write_single_reg( baseaddr+6, (P2 & 0x0000FF00) >> 8);
  stm_i2c_si5351a_write_single_reg( baseaddr+7, (P2 & 0x000000FF));

  /* Configure the clk control and enable the output */
  uint8_t clkControlReg = 0x0F;                             /* 8mA drive strength, MS0 as CLK0 source, Clock not inverted, powered up */
  if (pllSource == SI5351_PLL_B) clkControlReg |= (1 << 5); /* Uses PLLB */
  if (num == 0) clkControlReg |= (1 << 6);                  /* Integer mode */
  switch (output)
  {
    case 0:
      stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_16_CLK0_CONTROL, clkControlReg);
      break;
    case 1:
      stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_17_CLK1_CONTROL, clkControlReg);
      break;
    case 2:
      stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_18_CLK2_CONTROL, clkControlReg);
      break;
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Enables or disables all clock outputs
*/
/**************************************************************************/
char stm_Si5351_enableOutputs(unsigned char enabled)
{
  /* Make sure we've called init first */
  //ASSERT(m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED);

  /* Enabled desired outputs (see Register 3) */
  if(enabled)
	  printf("Si5351a> Enable Outputs\r\n");
  else
	  printf("Si5351a> Disable Outputs\r\n");
  stm_i2c_si5351a_write_single_reg(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, enabled ? 0x00: 0xFF);

  return ERROR_NONE;
}


//============= main loop =========
void stmSi5351aLoop (void)
{
	u8 i,  port;
	int regaddr=0; //first address
	int len;
	unsigned long rdata, port_bank_sel;

	printf("Si5351 Loop\r\n");
	printf("Si5351a> I2C Init (8-bit Address = 0x%02x) ...", SI5351aADDR8);
	if(!g_bI2CModuleConfigDone){
#if (PROCESSOR == STM32F407VGT6)
		gI2Cx = I2C1;
#elif (PROCESSOR == STM32F407VZT6)
		gI2Cx = I2C2;
#endif
		stmI2cModuleConfig(gI2Cx,400000);//400Kbps
	}
	printf("Done.\r\n");

	delayms(100);
	printf("Si5351a> Init\r\n");
	stm_Si5351_Init();
	//Disable Clocks
	stm_Si5351_enableOutputs(0);

	//Integer Only Mode --> Most accurate output
	//Setup PLLA to integer only mode @900MHz(must be 600..900MHz)
	//Set Multisynth 0 to 112.5MHz using integer only mode (div by 4/6/8)
	//25MHz*36=960MHz, then 900MHz/8 => 112.5MHz
	//printf("set plla to 960MHz\r\n");
	//stm_Si5351_setupPLLInt(SI5351_PLL_A,36); //multi must be between 15 and 90
	//printf("set Output 0 to 112.5MHz\r\n");
	//stm_Si5351_setupMultisynthInt(0,SI5351_PLL_A, SI5351_MULTISYNTH_DIV_8);
/*
	//25MHz*15= 375MHz
	printf("Si5351a> Set PLL-A (Integer Only Mode) to 375MHz(minimum)\r\n");
	stm_Si5351_setupPLLInt(SI5351_PLL_A,15); //multi must be between 15 and 90
	printf("Si5351a> Set Output 0 to 46.875MHz (= 375MHz/8)\r\n");
	stm_Si5351_setupMultisynthInt(0,SI5351_PLL_A, SI5351_MULTISYNTH_DIV_8);

	//Fractional Mode : More flexible but occurs clock jitter
	//Setup PLLB to fractional mode @616.6666MHz(XTAL*24 +2/3)
	printf("Si5351a> Set PLL-B to fractional mode @616.6666MHz\r\n");
	stm_Si5351_setupPLL(SI5351_PLL_B,24,2,3);
	//Setup Multisynth 1 to 13.5531MHz(PLLB/45.5)
	printf("Si5351a> Set Output 1 to 13.553115MHz ( = 616.6666MHz/45.5)\r\n");
	stm_Si5351_setupMultisynth(1,SI5351_PLL_B, 45,1,2);

	//Multisynth 2 is not yet used and will not be enabled, but can be
	//Use PLLB @616.6666MHz, the divide by 900 -->685.185KHz
	printf("Si5351a> Using the same PLLB @616.6666MHz, we divide by 900 -->685.185 KHz.\r\n");
	stm_Si5351_setupMultisynth(2,SI5351_PLL_B, 900,0,1);

	//then divide by 64 for 10.706KHz
	printf("Si5351a> Set Output 2 to 10.706KHz (= 685.185KHz/64).\r\n");
	stm_Si5351_setupRdiv(2,SI5351_R_DIV_64);
*/

	//25MHz*15= 375MHz
	printf("Si5351a> Set PLL-A (Integer Only Mode) to 400MHz\r\n");
	stm_Si5351_setupPLLInt(SI5351_PLL_A,16); //multi must be between 15 and 90
	printf("Si5351a> Set Output 0 to 25.0MHz (= 400MHz/4/4)\r\n");
	stm_Si5351_setupMultisynthInt(0,SI5351_PLL_A, SI5351_MULTISYNTH_DIV_4); //400/4
	stm_Si5351_setupRdiv(0,SI5351_R_DIV_2); //100/4

	printf("Si5351a> Set Output 1 to 25MHz \r\n");
	stm_Si5351_setupMultisynthInt(1,SI5351_PLL_A, SI5351_MULTISYNTH_DIV_4); //400/4
	stm_Si5351_setupRdiv(1,SI5351_R_DIV_2); //100/4

	printf("Si5351a> Set Output 2 to 25MHz \r\n");
	stm_Si5351_setupMultisynthInt(2,SI5351_PLL_A, SI5351_MULTISYNTH_DIV_4); //400/4
	stm_Si5351_setupRdiv(2,SI5351_R_DIV_2); //100/4

	//Enable Clocks
	stm_Si5351_enableOutputs(1);
	while(1){

	}
/*
	i2c_si5351a_disable_all_outputs(); //write reg3 = 0xff

	//reg 187: Fanout Enable
	//Tun off
	i2c_si5351a_write_reg(187,0xC0, 1);

	//i2c_si5351a_disable_oeb_pin_control();
	i2c_si5351a_power_down_all_clocks(); //wirte reg16~23 = 0x80

	//set interrupt mask. Reg2.

	//Set Load capacitance: Reg183[7:6]
	i2c_si5351a_set_crystal_configuration();

	//i2c_si5351a_enable_xo_and_ms_fanout();

	//reg 15.
	i2c_si5351a_configure_pll_sources_for_xtal();

	i2c_si5351a_configure_pll1_multisynth();

	i2c_si5351a_configure_multisynth(4,1536,0,1,0);//50MHz
	i2c_si5351a_configure_multisynth(5,1536,0,1,0);//50MHz
	i2c_si5351a_configure_multisynths_6_and_7();

	i2c_si5351a_configure_clock_control();


	//Apply PLLA and B soft reset
	i2c_si5351a_write_reg(117, 0xAC, 1);

	//Enable outputs : Reg.3
	i2c_si5351a_enable_clock_outputs();


	delayms(1000);
    //Now Read it
    printf("[Read Register Content]\r\n");

    while(1){
    	for(i=0;i<100;i++){
    		stm_i2c_si5351a_read_single_reg(i,&rdata);
    		printf("0x%08x : 0x%08x \r\n",i, rdata);
    		delayms(1000);
    	}
    }
*/

}

//======================= FOR REFERENCE ONLY ==================================

/*

void i2c_si5351a_disable_all_outputs (void){
	unsigned char data[] = {0xff};
	i2c_si5351a_write_reg(3,data, sizeof(data));
}
void i2c_si5351a_disable_oeb_pin_control (void){
	unsigned char data[] = {0xff};
	i2c_si5351a_write_reg(9,data, sizeof(data));
}
//Write Reg16~23 = 0x80
void i2c_si5351a_power_down_all_clocks (void){
	//unsigned char data[] = {0x80,0x80,0x80,0x80,0x80,0xc0,0xc0};
	unsigned char data[] = {0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80};
	i2c_si5351a_write_reg(16,data, sizeof(data));
}
//reg 183: Crystal internal load capacitance
//read as 0xe4 on power up.
//Set to???
void i2c_si5351a_set_crystal_configuration (void){
	unsigned char data[] = {0b10100100};
	i2c_si5351a_write_reg(183,data, sizeof(data));
}

//reg 187: Fanout Enable
//Tun on X) and MultiSynth Fanout only

void i2c_si5351a_enable_xo_and_ms_fanout(void){
	unsigned char data[] = {0x50};
	i2c_si5351a_write_reg(187,data, sizeof(data));
}

//Reg15: PLL Input Source
// CLKIN_DIV=0 (divide by 1)
// PLLB_SRC=0 (XTAL_input)
// PLLA_SRC=0 (XTAL_input)
void i2c_si5351a_configure_pll_sources_for_xtal (void){
	unsigned char data[] = {0x00};
	i2c_si5351a_write_reg(15,data, sizeof(data));
}

//MultiSynth NA(PLL1)
//Multiply clock source by 32
//a=32, b=0; c=1
//p1=0xe00, p2=0; p3=1
void i2c_si5351a_configure_pll1_multisynth (void){
	unsigned char data[] = {0x00,0x01,0x00,0x0e,0x00,0x00,0x00,0x00};
	i2c_si5351a_write_reg(26,data, sizeof(data));
}
*/

void i2c_si5351a_configure_multisynth(const unsigned int ms_number,
		const unsigned int p1, const unsigned int p2, const unsigned int p3,
    	const unsigned int r_div)
{
	/*
	 * TODO: Check for p3 > 0? 0 has no meaning in fractional mode?
	 * And it makes for more jitter in integer mode.
	 */
	/*
	 * r is the r divider value encoded:
	 *   0 means divide by 1
	 *   1 means divide by 2
	 *   2 means divide by 4
	 *   ...
	 *   7 means divide by 128
	 */
	//const uint_fast8_t register_number = 42 + (ms_number * 8);
	const unsigned int register_number = 42 + (ms_number * 8);
	uint8_t data[] = {
			//register_number,
			(p3 >> 8) & 0xFF,
			(p3 >> 0) & 0xFF,
			(r_div << 4) | (0 << 2) | ((p1 >> 16) & 0x3),
			(p1 >> 8) & 0xFF,
			(p1 >> 0) & 0xFF,
			(((p3 >> 16) & 0xF) << 4) | (((p2 >> 16) & 0xF) << 0),
			(p2 >> 8) & 0xFF,
			(p2 >> 0) & 0xFF };
	i2c_si5351a_write_reg(register_number,data, sizeof(data));
}

void i2c_si5351a_configure_multisynths_6_and_7(void) {
	// ms6_p1 = 6, ms7_pi1 = 6, r6_div = /1, r7_div = /1
	uint8_t ms6_7_data[] = { //90,
		0b00000110, 0b00000110,
		0b00000000
	};
	i2c_si5351a_write_reg(90, ms6_7_data, sizeof(ms6_7_data));
}
/*
 * Registers 16 through 23: CLKx Control
 * CLK0:
 *   CLK0_PDN=1 (powered down)
 *   MS0_INT=1 (integer mode)
 * CLK1:
 *   CLK1_PDN=1 (powered down)
 *   MS1_INT=1 (integer mode)
 * CLK2:
 *   CLK2_PDN=1 (powered down)
 *   MS2_INT=1 (integer mode)
 * CLK3:
 *   CLK3_PDN=1 (powered down)
 *   MS3_INT=1 (integer mode)
 * CLK4:
 *   CLK4_PDN=0 (powered up)
 *   MS4_INT=1 (integer mode)
 *   MS4_SRC=0 (PLLA as source for MultiSynth 4)
 *   CLK4_INV=1 (inverted)
 *   CLK4_SRC=11 (MS4 as input source)
 *   CLK4_IDRV=11 (8mA)
 * CLK5:
 *   CLK5_PDN=0 (powered up)
 *   MS5_INT=1 (integer mode)
 *   MS5_SRC=0 (PLLA as source for MultiSynth 5)
 *   CLK5_INV=0 (not inverted)
 *   CLK5_SRC=10 (MS4 as input source)
 *   CLK5_IDRV=11 (8mA)
 * CLK6: (not connected)
 *   CLK6_PDN=0 (powered up)
 *   FBA_INT=1 (FBA MultiSynth integer mode)
 *   MS6_SRC=0 (PLLA as source for MultiSynth 6)
 *   CLK6_INV=1 (inverted)
 *   CLK6_SRC=10 (MS4 as input source)
 *   CLK6_IDRV=11 (8mA)
 * CLK7: (not connected)
 *   CLK7_PDN=0 (powered up)
 *   FBB_INT=1 (FBB MultiSynth integer mode)
 *   MS7_SRC=0 (PLLA as source for MultiSynth 7)
 *   CLK7_INV=0 (not inverted)
 *   CLK7_SRC=10 (MS4 as input source)
 *   CLK7_IDRV=11 (8mA)
 */
void i2c_si5351a_configure_clock_control()
{
	uint8_t data[] = {
			//16,
		0x80,
		0x80,
		0x80,
		0x80,
		0x5f,
		0x4b,
		0x5b,
		0x4b
	};
	i2c_si5351a_write_reg(16, data, sizeof(data)); //// si5351c_write(data, sizeof(data));
}

/* Enable CLK outputs 4, 5, 6, 7 only. */
void i2c_si5351a_enable_clock_outputs()
{
	uint8_t data[] = { 0x0F };
	i2c_si5351a_write_reg(3, data, sizeof(data));
}

