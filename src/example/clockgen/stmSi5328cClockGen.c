// Use the I2C bus for Si5328c
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
#include "si5351.h"
#include "Si5328.h"
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#include "si5328.h"
//#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurst(unsigned char SlaveAddress, unsigned char *buf, unsigned char nbyte);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);

extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c
/*
@brief  Configures the Si5351 with config settings generated in
         ClockBuilder. You can use this function to make sure that
         your HW is properly configure and that there are no problems
         with the board itself.
*/
/* Test setup from SI5328 ClockBuilder
 * -----------------------------------
 * TCXO:      40     MHz
 * CKIN1    : 25 MHz (from PHY) : Low Prio
 * CKIN2    : 25 MHz (from Local 25MHz OSC) : High Prio
 * f3 = CKINn/N3n = 25MHz/N31 = 25MHz/40 = 625KHz,
 *    =           = 40MHz/N32 = 40MHz/64 = 625KHz
 * fosc =
 */

//--- Free run mode
// No ckin2.
// TCXO 40MHz becomes CKIN2 input internally.(FreeRun = route XA/XB to CKIN2.)
/*
static const uint8_t m_Si5328_regs[][2] =
{
  {  0, 0x60 },    //0x60...x-FreeRUN-CKOUT_ALWAYS_ON-x-x-x-BYPASS_REG-x @ FreeRun = route XA/XB to CKIN2. (We enabled) --> x-1-1-x xx-0-x
//  {  1, 0x01 },    //0x04..xxxx-CK_PRIO2[1:0]-CK_PRIO1[1:0]@ 01 = CKIN2 is Lower Prio. 00 = CKIN1 is Higher Prio.-->0x04//00= CKIN1 is 2nd Prio. 01 = CKIN2 is 1st Prio. ==>0x01
  {  1, 0x04 },    //0x04..xxxx-CK_PRIO2[1:0]-CK_PRIO1[1:0]@ 01-00
  {  2, 0x4F },    //was 0x4F//BWSEL : BWSEL_REG[3:0]-xxxx @ f3dB Bandwidth for PLL. (Our = 0x90 or 0x9F)
  {  3, 0x6F },    //was 0x6f//CKSEL DHOLD SW_ICAL : CKSEL[1:0]-DHOLD-SQ_ICAL-xxxx @ 00-0-1 (0ur = 0x10). CKSEL=01 (CKIN_2 Selected). Squelch During Internal calibration. (1=Output clocks disabled during ICAL.)
  {  4, 0x92 },    //was 0x80//AUTOSEL[1:0]-x-HIST_DEL[4:0] @ 10-x-10010 (0x92) : 10=Automatic Revertive. HIST_DEL = 0x12. Delay to be used in generating the history information.
  {  5, 0x80 },    //ICMOS[1:0]-xx xxxx : Driver Current Setting : 10 = 24mA/15mA
  {  6, 0x12 },    //was 0x80//x-SLEEP-SFOUT2[2:0]-SFOUT1[2:0] : 0 = NoSleep;  SFOUT2 = 010; SFOUT1 = 010 -->CMOS Output : 0001 0010//SFOUT2 = 101; SFOUT1 = 101
  {  7, 0x01 },    //was 0x02//xxxxx-FORCEFSEL[2:0] @ Which input clock is used as the ref freq for freq offset (FOS) alarms. (Our 2 = CKIN2)
  {  8, 0x00 },    //was 80//HLOG2[1:0]-HLOG1[1:0]-xxxx @ 00= Normal Operation. ???
  {  9, 0xC0 },    //was 00//HIST_AVG[4:0]-xxx @ Amount of averaging time(0x18). 1100 0xxx
  {  10, 0x00 },   //xxx-DSBL2-DSBL1-XX @ Disable clk 2/1. --> Enable
  {  11, 0x00 },   //xxxx-xx-PD_CK2-PD_CK1 @ 0-0 = enable
  {  17, 0x00 },   //FLAT_VALID[14:0] ==???
  {  18, 0x00 },   //FLAT_VALID[14:0] ==???
  {  19, 0x21 },   //FOC_EN-FOS_THR[1:0]-VALTIME[1:0]-LOCKT[2:0] @ FrequencyOffsetEnable(0). FrequencyOffsetDeclaredThreshold(01). VALTIME =01(100msec) LOCT=001(53msec) ==> 0-01-0 0-001 (0x21)
  {  20, 0x0e },   //was 0x05//xxxx-CK2_BAD_PIN-CK1_BAD_PIN-LOL_PIN-INT_PIN @ C2B status reflect to output pin(1).xxxx-1-1-LOL_INT(1)-INT(0) ==> 0x0e
  {  21, 0x03 },   //was 0x03//xxxx-xx-CK1_ACTV_PIN-CKSEL_PIN @ 0000-00-1-1
  {  22, 0x00 },   //was 0x0c//Output pin polarity : xxxx-CK_ACTV_POL-CK_BAD_POL-LOL_POL-INT_POL @ (Out all active lows) =0)
  {  23, 0x07 },   //was 0x0c//LOS mask: xxxx-x-LOS2_MSK-LOS1_MS-LOSX_MSK :(Our = 111)
  {  24, 0x07 },    //was 0x00//FOS mask: xxxx-x-FOS2_MSK-FOS1_MSK-LOL_MSK(Our = 111)

  //PLL Dividers
  // Output Divider NC1 = N1_HS * N1_LS(NC1_LS?) = 7*28 =  196. --> Output Clk = 4900/196 =25MHz
  // Output Divider NC2 = N1_HS * N2_LS(NC2_LS?) = 7*28 =  196. --> Output Clk = 4900/196 =25MHz
  // Feedback Divider N2 = N2_HS * N2_LS = 10 * 784 = 7840.
  // N31 = 40
  // N32 = 64
  // Fosc = 4.9GHz
  // f3=625KHz
  // example : 5600MHz/NC1 = 5600/(4*54) = 25MHz.
  // example : 4900MHz/NC1 = 4900/(7*28) = 25MHz.

  //N1 = 7 x 28 = 196
  {  25, 0x60 },    //N1_HS : High speed divider: N1_HS[2:0]x-xxxx (Our = N1_HS = 7-->3 ) --> 011x-xxxx
  {  31, 0x00 },    //NC1_LS Low speed divider, which drives CKOUT1. Must be 0 or odd. (Our NC1_LS = 28(-->27(0x1B)): xxxx-NC1_LS[19:16]
  {  32, 0x00 },    //NC1_LS[15:8] = 0x00
  {  33, 0x1B },    //was 0x35 ??????? //NC1_LS[7:0] = 0x1B (27)

  //Not applicable for ours. We not use CKOUT2.
  {  34, 0x00 },    //NC2_LS Low speed divier, which drives CKOUT2. Must be 0 or odd. (Our NC2_LS = 28 --> 27(0x1B)): xxxx-NC2_LS[19:16]
  {  35, 0x00 },	//NC2_LS[15:8] = 0x00
  {  36, 0x1B },	//NC2_LS[7:0] = 0x1B

  //Feedback N2 = 7840
  {  40, 0x60 },    //N2_HS[2:0]-x-N2_LS[19:16] : N2_HS =10 --> 0x6(3bits)
  {  41, 0x03 },    //N2_LS[15:8] = 0x03 ; N2_LS = 784 -->783 (20bits =0x30F) @ 110-x-0000
  {  42, 0x0F },    //N2_LS[7:0] = 0x0F

  //N31 = 40
  {  43, 0x00 },	//N31 : xxxx-x-N31[18:16] @ N31 = 40 --> 39(0x27)
  {  44, 0x00 },    //N31[15:8]
  {  45, 0x27 },    //N31[7:0]

  //N32 = 64
  {  46, 0x00 },    //N32 : xxxx-x-N32[18:16] @ N32 = 64 --> 63(0x3F)
  {  47, 0x00 }, 	//N32[15:8]
  {  48, 0x3F }, 	//N32[7:0]

  //  {  55, 0x00 },	//ClockInRate: xx-CLKIN2Rate[2:0]-CLKIN1Rate[2:0] @ 000 = 10~27MHz --> xx-001-001
  {  55, 0x10 },	//ClockInRate: xx-CLKIN2Rate[2:0]-CLKIN1Rate[2:0] @ 000 = 10~27MHz --> xx-001-000

  {  128, 0x00 },   //xxxx-xx-CLK2_ACTV_REG-CLK1_ACTV_REG : Read Only
  {  129, 0x00 },   //xxxx-x-LOS2_INT-LOS1_INT-LOSX_INT : Read Only
  {  130, 0x00 },   //x-DIGHOLDVALID-xxx-FOS2_INT-FOS1_INT-LOL_INT : Read Only
  {  131, 0x07 },	//xxxx-xLOS2_FLG-LOS1_FLG-LOSX_FLG : R/W
  {  132, 0x02 },   //xxxx-FOS2_FLG-FOS1_FLG-LOL_FLG-x : R/W (001x)
  //{  133, 0x00 },
  {  134, 0x00 },   //PARTNUM_RO[11:4] : Read Only (PARTNUM_RO[11:0] = may be 0x1C)
  {  135, 0x00 },   //PARTNUM_RO[3:0]-REVID_RO[3:0] : Read Only (REVID = 0x2 == Rev C)

  {  136, 0x40 },    //RST_REG-ICAL-xx-xxxx (ICAL= start Internal Calibration Sequence)
  {  137, 0x01 },    //xxxx-xxx-FastLock
  {  138, 0x03 },    //xxxx-xx-LOS2_EN-LOS1_EN
  {  139, 0x33 },	//xx-LOS2_EN-LOS1_EN-xx-FOS2_EN-FOS1_EN
  { 142, 0x00 },	//Indefpend-EntSkew1[7:0] = 0x00
  { 143, 0x00 }		//Indefpend-EntSkew2[7:0] = 0x00

};
*/
/* Test setup from SI5328 ClockBuilder
 * -----------------------------------
 * TCXO:      40     MHz
 * CKIN1    : 25 MHz (from PHY) : Low Prio
 * CKIN2    : 25 MHz (from Local 25MHz OSC) : High Prio
 * N31=13
 * N32=13
 * f3 = CKINn/N3n = 25MHz/N31 = 25MHz/13 = 1.923077MHz,
 *    =           = 25MHz/N32 = 25MHz/13 = 1.923077MHz
 * fosc = 5.4GHz
 * Output Divider NC1 = N1_HS * N1_LS = 4 * 54
  Feedback Divider N2 = N2_HS * N2_LS = 9 * 312
  example : 5600MHz/NC1 = 5600/(4*54) = 25MHz.
 */
/*
//Success on special module. (Single CK1 Input, Two Outputs)
//R0: Not Free-Run Mode
//Manual Select.
//Not Free-Run Mode
static const uint8_t m_Si5328_regs[][2] =
{
  {  0, 0b001011101 },   //x-FreeRUN-CKOUT_ALWAYS_ON-x-x-x-BYPASS_REG-x @ FreeRun = route XA/XB to CKIN2. (We disabled),
//  {  1, 0b11110001 },    //xxxx-CK_PRIO2[1:0]-CK_PRIO1[1:0]@ PRIO2=00= CKIN1 is Lower. PRIO1=01 = CKIN2 is Higher Prio.
  {  1, 0b11110100 },    //xxxx-CK_PRIO2[1:0]-CK_PRIO1[1:0]@ 01 = CKIN2 is 2nd Prio. 00 = CKIN1 is 1st Prio.
  {  2, 0xAF },    		 //BWSEL_REG[3:0]-xxxx @ f3dB Bandwidth for PLL. (Our = 10...0xA0 of 0xAF)
//  {  3, 0b00001111 },    //CKSEL[1:0]-DHOLD-SQ_ICAL-xxxx @ 00-0-1 (0ur = 0x5F). CKSEL=(00=CKIN_1 Selected, 01=CLK2 Selected). Squelch During Internal calibration. (1=Output clocks disabled during ICAL.)
  {  3, 0b00001111 },    //CKSEL[1:0]-DHOLD-SQ_ICAL-xxxx @ 00-0-0. CKSEL=(00=CKIN_1 Selected). DHOLD = 0, No Squelch During Internal calibration. (1=Output clocks disabled during ICAL.)
//  {  4, 0b00110010 },    //was 0x80//AUTOSEL[1:0]-x-HIST_DEL[4:0] @ 10-x-10010 (0x92) : 10=Automatic Revertive||00=Manual HIST_DEL(26msec)=0x12. Delay to be used in generating the history information.
  {  4, 0b00110010 },    //AUTOSEL[1:0]-x-HIST_DEL[4:0] @ 00-x-10010 : 00=Manual. HIST_DEL(26msec)=0x12. Delay to be used in generating the history information.
  {  5, 0b10111111 },    //ICMOS:ICMOS[1:0]-xx xxxx : Driver Current Setting : 10 = 24mA/15mA
  {  6, 0b10010010 },    //was 0x80//x-SLEEP-SFOUT2[2:0]-SFOUT1[2:0] @ 0 = NoSleep;  SFOUT2 = 010; SFOUT1 = 010 -->CMOS Output//SFOUT2 = 101; SFOUT1 = 101
  {  7, 0b11111010 },    //was 80//xxxxx-FORCEFSEL[2:0] @ Which input clock is used as the ref freq for freq offset (FOS) alarms. (Our 2 = CKIN2)
  {  8, 0x00 },    		//was 80//HLOG2[1:0]-HLOG1[1:0]-xxxx @ 00= Normal Operation. ???
  {  9, 0xC0 },    		//was 00//HIST_AVG[4:0]-xxx @ Amount of averaging time(=6711msec,0x18). 1100 0xxx
  {  10, 0b1110011 },   //xxx-DSBL2-DSBL1-XX @ Disable clk 2/1. --> Enable
  {  11, 0xF0 },   //xxxx-xx-PD_CK2-PD_CK1 @ 0-0 = enable
  {  19, 0x00 },   //FOC_EN-FOS_THR[1:0]-VALTIME[1:0]-LOCKT[2:0] @ FrequencyOffsetEnable(0). FrequencyOffsetDeclaredThreshold(01).
  {  20, 0xfe },   //was 0x05//xxxx-CK2_BAD_PIN-CK1_BAD_PIN-LOL_PIN-INT_PIN @ C2B status reflect to output pin(1).-LOS1(1)-LOL_INT(1)-INT(0)
  //{  21, 0xF1 },   //xxxx-xx-CK1_ACTV_PIN-CKSEL_PIN (both roles: CS_CA roles) Not Acitivity Output Indicator, but CLKSEL input pin (in manual selection)
  {  21, 0xF2 },   //xxxx-xx-CK1_ACTV_PIN-CKSEL_PIN (both roles: CS_CA roles) either 0x02(Acitivity Output Indicator). Not CLKSEL input pin.
  {  22, 0xF0 },   //was 0x0c//Output pin polarity : xxxx-CK_ACTV_POL-CK_BAD_POL-LOL_POL-INT_POL @ (Out all active lows) =0
  {  23, 0xF7 },   //was 0x0c//LOS mask: xxxx-x-LOS2_MSK-LOS1_MS-LOSX_MSK :(Our = 111)
  {  24, 0xF7},    //was 0x00//FOS mask: xxxx-x-FOS2_MSK-FOS1_MSK-LOL_MSK(Our = 111)

  //PLL Dividers
  // Output Divider NC1 = N1_HS * N1_LS = 4*54
  // Feedback Divider N2 = N2_HS * N2_LS = 9*312
  // Fosc = 5.4GHz
  // example : 5600MHz/NC1 = 5600/(4*54) = 25MHz.

  {  25, 0x00 },    //N1_HS High speed divider: N1_HS[2:0]x-xxxx (Our = N1_HS = 4-->0 ) --> 000x-xxxx
  {  31, 0x00 },    //NC1_LS Low speed divider, which drives CKOUT1. Must be 0 or odd. (Our NC1_LS = 54(-->53(0x35)): xxxx-NC1_LS[19:16]
  {  32, 0x00 },    //NC1_LS[15:8] = 0x00
  {  33, 0x35 },    //was 0x35 ??????? //NC1_LS[7:0] = 0x35 (53)

  //Not applicable for ours. We not use CKOUT2.
  {  34, 0x00 },  //NC2_LS Low speed divier, which drives CKOUT2. Must be 0 or odd. (Our NC2_LS = 312 --> 311(0x137): xxxx-NC2_LS[19:16]
  {  35, 0x00 },	//NC2_LS[15:8] = 0x01
  {  36, 0x35 },	//NC2_LS[7:0] = 0x35

  {  40, 0b10100000 },    //N2_HS[2:0]-x-N2_LS[19:16] : N2_HS =9 --> 0x5(3bits) ; N2_LS = 312 -->311 (20bits =0x137) @ 101-x-0000
  {  41, 0x01 },    //N2_LS[15:8] = 0x01
  {  42, 0x37 },    //N2_LS[7:0] = 0x37

  {  43, 0x00 },	//N31 : xxxx-x-N31[18:16] @ N31 = 13 --> 12(0xc)
  {  44, 0x00 },    //N31[15:8]
  {  45, 0x0c },    //N31[7:0]

  {  46, 0x00 },    //N32 : xxxx-x-N32[18:16] @ N32 = 13 --> 12(0xc)
  {  47, 0x00 }, 	//N32[15:8]
  {  48, 0x0c }, 	//N32[7:0]

  {  55, 0x00 },	//ClockInRate: xx-CLKIN2Rate[2:0]-CLKIN1Rate[2:0] @ 000 = 10~27MHz

  {  128, 0xF0 },   //xxxx-xx-CLK2_ACTV_REG-CLK1_ACTV_REG : Read Only
  {  129, 0xF0 },   //xxxx-x-LOS2_INT-LOS1_INT-LOSX_INT : Read Only
  {  130, 0x00 },   //x-DIGHOLDVALID-xxx-FOS2_INT-FOS1_INT-LOL_INT : Read Only
  {  131, 0xF7 },	//xxxx-xLOS2_FLG-LOS1_FLG-LOSX_FLG : R/W
  {  132, 0xF2 },   //xxxx-FOS2_FLG-FOS1_FLG-LOL_FLG-x : R/W (001x)
  //{  133, 0x00 },
  {  134, 0x00 },   //PARTNUM_RO[11:4] : Read Only (PARTNUM_RO[11:0] = may be 0x1C)
  {  135, 0x00 },   //PARTNUM_RO[3:0]-REVID_RO[3:0] : Read Only (REVID = 0x2 == Rev C)

  {  136, 0x40 },    //RST_REG-ICAL-xx-xxxx (ICAL= start Internal Calibration Sequence)
  {  137, 0xF1 },    //xxxx-xxx-FastLock
  {  138, 0xF3 },    //xxxx-xx-LOS2_EN-LOS1_EN
  {  139, 0x33 },	//xx-LOS2_EN-LOS1_EN-xx-FOS2_EN-FOS1_EN
  { 142, 0x00 },	//Indefpend-EntSkew1[7:0] = 0x00
  { 143, 0x00 }		//Indefpend-EntSkew2[7:0] = 0x00

};
*/
/* Free run mode Test on on special module. (Single CK1 Input, Two Outputs)
 * TCXO 40MHz becomes CKIN2 input internally.(FreeRun = route XA/XB to CKIN2.)
 * -----------------------------------
 * TCXO:      40     MHz
 * CKIN1    : 25 MHz (from PHY) : High Prio (24.7449~28.929MHz)
 * CKIN2    : 40 MHz (from TCXO) : Low Prio (39.592~46.286MHz)
 * f3 = CKINn/N3n = 25MHz/N31 = 25MHz/40 = 625KHz,
 *    =           = 40MHz/N32 = 40MHz/64 = 625KHz
 * fosc = 4.9GHz
 * N1_HS =7; NC1_LS and NC2_LS = 28
 * N2_HS = 10; N2_LS = 784
 * N31 = 40; N32 = 64
 *
 * FreeRun = 1
 * BW = 9~3
 */

static const uint8_t m_Si5328_regs[][2] =
{
  {  0, 0b01101101 },   //x-FreeRUN-CKOUT_ALWAYS_ON-x-x-x-BYPASS_REG-x @ FreeRun = route XA/XB to CKIN2. (We enabled) --> x-1-1-x xx-0-x
  {  1, 0b11110100 },   //xxxx-CK_PRIO2[1:0]-CK_PRIO1[1:0]@ 01 = CKIN2 is 2nd Prio. 00 = CKIN1 is 1st Prio.
  //{  2, 0x9F },    		//BWSEL_REG[3:0]-xxxx @ f3dB Bandwidth for PLL. (Our = 0x90 or 0x9F)
  {  2, 0x9F },    		//BWSEL_REG[3:0]-xxxx @ f3dB Bandwidth for PLL. (Our = 0x90 or 0x9F)
  //  {  3, 0x6F },    	//CKSEL[1:0]-DHOLD-SQ_ICAL-xxxx @ 00-0-1 (0ur = 0x10). CKSEL=01 (CKIN_2 Selected). Squelch During Internal calibration. (1=Output clocks disabled during ICAL.)
  {  3, 0b00001111 },   //CKSEL[1:0]-DHOLD-SQ_ICAL-xxxx (Only Valid for Manual Selection) @  CKSEL=(00=CKIN_1 Selected). DHOLD = 0, No Squelch During Internal calibration. (1=Output clocks disabled during ICAL.)
  //{  4, 0b00110010 }, //AUTOSEL[1:0]-x-HIST_DEL[4:0] @ 00=Manual. HIST_DEL(26msec)=0x12. Delay to be used in generating the history information.
  {  4, 0b10110010 },   //AUTOSEL[1:0]-x-HIST_DEL[4:0] @ 10=Automatic Revertive. HIST_DEL = 0x12. Delay to be used in generating the history information.
  {  5, 0x80 },    		//ICMOS[1:0]-xx xxxx : Driver Current Setting : 10 = 24mA/15mA
  {  6, 0x12 },    		//was 0x80//x-SLEEP-SFOUT2[2:0]-SFOUT1[2:0] : 0 = NoSleep;  SFOUT2 = 010; SFOUT1 = 010 -->CMOS Output : 0001 0010//SFOUT2 = 101; SFOUT1 = 101
  {  7, 0b11111010 },   //xxxxx-FORCEFSEL[2:0] @ Which input clock is used as the ref freq for freq offset (FOS) alarms. (2 = CKIN2)
  {  8, 0x00 },    		//HLOG2[1:0]-HLOG1[1:0]-xxxx @ 00= Normal Operation. ???
  {  9, 0xC0 },    		//HIST_AVG[4:0]-xxx @ Amount of averaging time(0x18). 1100 0xxx
  {  10, 0x00 },   		//xxx-DSBL2-DSBL1-XX @ Disable clk 2/1. --> Enable
  {  11, 0x00 },   		//xxxx-xx-PD_CK2-PD_CK1 @ 0-0 = enable
  {  17, 0x00 },   		//FLAT_VALID[14:0] ==???
  {  18, 0x00 },   		//FLAT_VALID[14:0] ==???
  {  19, 0b00101001 },  //FOC_EN-FOS_THR[1:0]-VALTIME[1:0]-LOCKT[2:0] @ FrequencyOffsetEnable(0). FrequencyOffsetDeclaredThreshold(01). VALTIME =01(100msec) LOCT=001(53msec) ==> 0-01-0 0-001 (0x21)
  {  20, 0xfe },   		//xxxx-CK2_BAD_PIN-CK1_BAD_PIN-LOL_PIN-INT_PIN @ C2B status reflect to output pin(1).-LOS1(1)-LOL_INT(1)-INT(0)
  //{  21, 0xF1 },   //xxxx-xx-CK1_ACTV_PIN-CKSEL_PIN (both roles: CS_CA roles) Not Acitivity Output Indicator, but CLKSEL input pin (in manual selection)
  {  21, 0xF2 },   //xxxx-xx-CK1_ACTV_PIN-CKSEL_PIN (both roles: CS_CA roles) either 0x02(Acitivity Output Indicator). Not CLKSEL input pin.
  {  22, 0xF0 },   //was 0x0c//Output pin polarity : xxxx-CK_ACTV_POL-CK_BAD_POL-LOL_POL-INT_POL @ (Out all active lows) =0
  {  23, 0xF7 },   //was 0x0c//LOS mask: xxxx-x-LOS2_MSK-LOS1_MS-LOSX_MSK :(Our = 111)
  {  24, 0xF7},    //was 0x00//FOS mask: xxxx-x-FOS2_MSK-FOS1_MSK-LOL_MSK(Our = 111)

  //PLL Dividers
  // Output Divider NC1 = N1_HS * N1_LS(NC1_LS?) = 7*28 =  196. --> Output Clk = 4900/196 =25MHz
  // Output Divider NC2 = N1_HS * N2_LS(NC2_LS?) = 7*28 =  196. --> Output Clk = 4900/196 =25MHz
  // Feedback Divider N2 = N2_HS * N2_LS = 10 * 784 = 7840.
  // N31 = 40
  // N32 = 64
  // Fosc = 4.9GHz
  // f3=625KHz
  // example : 4900MHz/NC1 = 4900/(7*28) = 25MHz.

  //N1 = 7 x 28 = 196
  {  25, 0b01111111 },    //N1_HS : High speed divider: N1_HS[2:0]x-xxxx (Our = N1_HS = 7-->3 ) --> 011x-xxxx
  {  31, 0x00 },    //NC1_LS Low speed divider, which drives CKOUT1. Must be 0 or odd. (Our NC1_LS = 28(-->27(0x1B)): xxxx-NC1_LS[19:16]
  {  32, 0x00 },    //NC1_LS[15:8] = 0x00
  {  33, 0x1B },    //was 0x35 ??????? //NC1_LS[7:0] = 0x1B (27)

  //Not applicable for ours. We not use CKOUT2.
  {  34, 0x00 },    //NC2_LS Low speed divier, which drives CKOUT2. Must be 0 or odd. (Our NC2_LS = 28 --> 27(0x1B)): xxxx-NC2_LS[19:16]
  {  35, 0x00 },	//NC2_LS[15:8] = 0x00
  {  36, 0x1B },	//NC2_LS[7:0] = 0x1B

  //Feedback N2 = 7840
  {  40, 0b11010000 },    //N2_HS[2:0]-x-N2_LS[19:16] : N2_HS =10 --> 0x6(3bits)
  {  41, 0x03 },    //N2_LS[15:8] = 0x03 ; N2_LS = 784 -->783 (20bits =0x30F) @ 110-x-0000
  {  42, 0x0F },    //N2_LS[7:0] = 0x0F

  //N31 = 40
  {  43, 0x00 },	//N31 : xxxx-x-N31[18:16] @ N31 = 40 --> 39(0x27)
  {  44, 0x00 },    //N31[15:8]
  {  45, 0x27 },    //N31[7:0]

  //N32 = 64
  {  46, 0x00 },    //N32 : xxxx-x-N32[18:16] @ N32 = 64 --> 63(0x3F)
  {  47, 0x00 }, 	//N32[15:8]
  {  48, 0x3F }, 	//N32[7:0]

 // {  55, 0x00 },	//ClockInRate: xx-CLKIN2Rate[2:0]-CLKIN1Rate[2:0] @ 000 = 10~27MHz, 010 = 25~54MHz
 {  55, 0x08 },		//ClockInRate: xx-CLKIN2Rate[2:0]-CLKIN1Rate[2:0] @ xx-001-000

  {  128, 0x00 },   //xxxx-xx-CLK2_ACTV_REG-CLK1_ACTV_REG : Read Only
  {  129, 0x00 },   //xxxx-x-LOS2_INT-LOS1_INT-LOSX_INT : Read Only
  {  130, 0x00 },   //x-DIGHOLDVALID-xxx-FOS2_INT-FOS1_INT-LOL_INT : Read Only
  {  131, 0x07 },	//xxxx-xLOS2_FLG-LOS1_FLG-LOSX_FLG : R/W
  {  132, 0x02 },   //xxxx-FOS2_FLG-FOS1_FLG-LOL_FLG-x : R/W (001x)
  //{  133, 0x00 },
  {  134, 0x00 },   //PARTNUM_RO[11:4] : Read Only (PARTNUM_RO[11:0] = may be 0x1C)
  {  135, 0x00 },   //PARTNUM_RO[3:0]-REVID_RO[3:0] : Read Only (REVID = 0x2 == Rev C)

  {  136, 0x40 },    //was 0x40 RST_REG-ICAL-xx-xxxx (ICAL= start Internal Calibration Sequence)
  {  137, 0x00 },    //xxxx-xxx-FastLock
  {  138, 0x03 },    //xxxx-xx-LOS2_EN-LOS1_EN
  {  139, 0x33 },	//xx-LOS2_EN-LOS1_EN-xx-FOS2_EN-FOS1_EN
  { 142, 0x00 },	//Indefpend-EntSkew1[7:0] = 0x00
  { 143, 0x00 }		//Indefpend-EntSkew2[7:0] = 0x00

};
Si5328Config_t m_Si5328Config;
//==================
//write to a single register
char stm_i2c_Si5328c_write_single_reg(unsigned int regaddr, unsigned char data ) {
    int rdata = data;

    stmI2cSendbuf[0] = regaddr;
    stmI2cSendbuf[1] = data; //a byte
    return stm_I2C_SendBurst(Si5328cADDR8, stmI2cSendbuf, 2);
}
u8 stm_i2c_Si5328c_read_single_reg(unsigned int regaddr, unsigned char *rdata ) {
	u8  retlen = 0;
	stmI2cSendbuf[0] = regaddr;
	stm_I2C_SendBurst(Si5328cADDR8, stmI2cSendbuf, 1); //write reg.

	retlen = stm_I2C_ReceiveBurst(Si5328cADDR8, stmI2cRecvbuf, 1);     //Read a single byte
	if(retlen==0){
		printf("i2c error\r\n");
		return 0;
	}
    *rdata  = stmI2cRecvbuf[0];
    return retlen;
}
//write one or more contiguous registers. data[0] should be the first value.
void i2c_Si5328c_write_reg(unsigned int regaddr, unsigned char data[], unsigned char dataleng ) {
    int i;

    stmI2cSendbuf[0] = regaddr;
    //fill data
    for(i=0;i<dataleng;i++)
    	stmI2cSendbuf[1+i] = data[i]; //a byte
    stm_I2C_SendBurst(Si5328cADDR8, stmI2cSendbuf, dataleng+1);
}

void stm_Si5328_WhoAmI(void){
	unsigned char regVal;
	unsigned short productId;
	unsigned char revId;

	stm_i2c_Si5328c_read_single_reg(134, &regVal); //PARTNUM_RO[11:4] : Read Only (PARTNUM_RO[11:0] = may be 0x01C)
	productId = regVal;
	productId = productId << 4;
	stm_i2c_Si5328c_read_single_reg(135, &revId);
	productId = productId | ((revId & 0xf0) >> 4);
	revId = revId & 0x0f;
	if((productId == 0x01c) && (revId == 0x02))
		printf("Product ID = Si5328(0x%03x) and RevID=C(0x%02x)\r\n",productId,revId); //PARTNUM_RO[3:0]-REVID_RO[3:0] : Read Only (REVID = 0x2 == Rev C)
	else
		printf("Product ID = 0x%03x and RevID=0x%02x\r\n",productId,revId); //PARTNUM_RO[3:0]-REVID_RO[3:0] : Read Only (REVID = 0x2 == Rev C)
}

void stm_Si5328_ShowReg(void){
	unsigned char regVal;
	unsigned char retSts;
	int i;

	for (i=0; i<sizeof(m_Si5328_regs)/2; i++) {
		retSts = stm_i2c_Si5328c_read_single_reg(m_Si5328_regs[i][0], &regVal);
		if(retSts == 0)
			printf("Error in i2c\r\n");
		else {
			printf("Reg[%u]=0x%02x\r\n",m_Si5328_regs[i][0],regVal);
		}
	}
}
void stm_Si5328_ReadConfig(void){
	printf("======================\r\n");
	printf("Config for Si5328C\r\n");
	printf("f3=625KHz \tfosc = 4.9GHz\r\n");
	printf("Input Divider      : N31 = 40 \tN32=64\r\n");
	printf("CLK1 Output Divider: N1_HS=7 \tNC1_LS=28 \tNC1=7*28= 196.\r\n");
	printf("CLK2 Output Divider: N1_HS=7 \tNC2_LS=28 \tNC2=7*28= 196.\r\n");
	printf("Feedback Divider   : N2_HS=10 \tN2_LS=784 \tN2=10*784= 7840.\r\n");
	printf("fout1 = fin1 x N2/(N1xN31) 25MHz * 7840/(196x40) = 25MHz.\r\n");
	printf("fout2 = fin2 x N2/(N1xN32) 40MHz * 7840/(196x64) = 25MHz.\r\n");
	printf("======================\r\n");
}
void stm_Si5328_ReadStatus(void){
	unsigned char regVal;
	unsigned char retSts;
	printf("======================\r\n");
	retSts = stm_i2c_Si5328c_read_single_reg(128, &regVal);if(retSts == 0)	printf("Error in i2c\r\n");
	printf("Reg[%u] xxxx-xx-CLK2_ACTV_REG-CLK1_ACTV_REG \t= 0x%02x\r\n",128,regVal);
	retSts = stm_i2c_Si5328c_read_single_reg(129, &regVal);if(retSts == 0)	printf("Error in i2c\r\n");
	printf("Reg[%u] xxxx-x-LOS2_INT-LOS1_INT-LOSX_INT \t= 0x%02x\r\n",129,regVal);
	retSts = stm_i2c_Si5328c_read_single_reg(130, &regVal);if(retSts == 0)	printf("Error in i2c\r\n");
	printf("Reg[%u] x-DIGHOLDVALID-xxx-FOS2_INT-FOS1_INT-LOL_INT \t= 0x%02x\r\n",130,regVal);
	retSts = stm_i2c_Si5328c_read_single_reg(131, &regVal);if(retSts == 0)	printf("Error in i2c\r\n");
	printf("Reg[%u] xxxx-xLOS2_FLG-LOS1_FLG-LOSX_FLG \t= 0x%02x\r\n",131,regVal);
	retSts = stm_i2c_Si5328c_read_single_reg(132, &regVal);if(retSts == 0)	printf("Error in i2c\r\n");
	printf("Reg[%u] xxxx-FOS2_FLG-FOS1_FLG-LOL_FLG-x \t= 0x%02x\r\n",132,regVal);
}
//====================================
void stm_Si5328_Init(void)
{
	stm_Si5328_WhoAmI();


  m_Si5328Config.initialised     = false;
  m_Si5328Config.crystalFreq     = Si5328_CRYSTAL_FREQ_25MHZ;
  m_Si5328Config.crystalLoad     = Si5328_CRYSTAL_LOAD_10PF;
  m_Si5328Config.crystalPPM      = 30;
  //m_Si5328Config.plla_configured = false;
  //m_Si5328Config.plla_freq       = 0;
  //m_Si5328Config.pllb_configured = false;
  //m_Si5328Config.pllb_freq       = 0;



  /* Disable all outputs setting CLKx_DIS high */
  //stm_i2c_Si5328c_write_single_reg(Si5328_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF);

  /* Power down all output drivers */
  //stm_i2c_Si5328c_write_single_reg(0, 0x62);
  //stm_i2c_Si5328c_write_single_reg(1, 0x00 | 0x04); //CLK1 - Prio1
  //stm_i2c_Si5328c_write_single_reg(Si5328_REGISTER_18_CLK2_CONTROL, 0x80);

  /* Set the load capacitance for the XTAL */
  //stm_i2c_Si5328c_write_single_reg(Si5328_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE, m_Si5328Config.crystalLoad);

  /* Set interrupt masks as required (see Register 2 description in AN619).
     By default, ClockBuilder Desktop sets this register to 0x18.
     Note that the least significant nibble must remain 0x8, but the most
     significant nibble may be modified to suit your needs. */

  /* Reset the PLL config fields just in case we call init again */
  //m_Si5328Config.plla_configured = false;
  //m_Si5328Config.plla_freq = 0;
  //m_Si5328Config.pllb_configured = false;
  //m_Si5328Config.pllb_freq = 0;

  /* All done! */
  m_Si5328Config.initialised = true;

  return ERROR_NONE;
}
//===========
/*!
    @brief  Configures the Multisynth divider, which determines the
            output clock frequency based on the specified PLL input.

    @param  output    The output channel to use (0..2)
    @param  pllSource	The PLL input source to use, which must be one of:
                      - Si5328_PLL_A
                      - Si5328_PLL_B
    @param  div       The integer divider for the Multisynth output.
                      If pure integer values are used, this value must
                      be one of:
                      - Si5328_MULTISYNTH_DIV_4
                      - Si5328_MULTISYNTH_DIV_6
                      - Si5328_MULTISYNTH_DIV_8
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
/*
char stm_Si5328_setupMultisynth(unsigned char     output,
                                       Si5328PLL_t pllSource,
                                       uint32_t    div,
                                       uint32_t    num,
                                       uint32_t    denom)
{
  uint32_t P1;       //* Multisynth config register P1
  uint32_t P2;	     // Multisynth config register P2
  uint32_t P3;	     // Multisynth config register P3

  // Set the main PLL config registers
  if (num == 0)
  {
    // Integer mode
    P1 = 128 * div - 512;
    P2 = num;
    P3 = denom;
  }
  else
  {
    // Fractional mode
    P1 = (uint32_t)(128 * div + floor(128 * ((float)num/(float)denom)) - 512);
    P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P3 = denom;
  }

  // Get the appropriate starting point for the PLL registers
  uint8_t baseaddr = 0;
  switch (output)
  {
    case 0:
      baseaddr = Si5328_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
      break;
    case 1:
      baseaddr = Si5328_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
      break;
    case 2:
      baseaddr = Si5328_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
      break;
  }

  // Set the MSx config registers
  stm_i2c_Si5328c_write_single_reg( baseaddr,   (P3 & 0x0000FF00) >> 8);
  stm_i2c_Si5328c_write_single_reg( baseaddr+1, (P3 & 0x000000FF));
  stm_i2c_Si5328c_write_single_reg( baseaddr+2, (P1 & 0x00030000) >> 16);	// ToDo: Add DIVBY4 (>150MHz) and R0 support (<500kHz) later
  stm_i2c_Si5328c_write_single_reg( baseaddr+3, (P1 & 0x0000FF00) >> 8);
  stm_i2c_Si5328c_write_single_reg( baseaddr+4, (P1 & 0x000000FF));
  stm_i2c_Si5328c_write_single_reg( baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) );
  stm_i2c_Si5328c_write_single_reg( baseaddr+6, (P2 & 0x0000FF00) >> 8);
  stm_i2c_Si5328c_write_single_reg( baseaddr+7, (P2 & 0x000000FF));

  // Configure the clk control and enable the output
  uint8_t clkControlReg = 0x0F;                             // 8mA drive strength, MS0 as CLK0 source, Clock not inverted, powered up
  if (pllSource == Si5328_PLL_B) clkControlReg |= (1 << 5); // Uses PLLB
  if (num == 0) clkControlReg |= (1 << 6);                  // Integer mode
  switch (output)
  {
    case 0:
      stm_i2c_Si5328c_write_single_reg(Si5328_REGISTER_16_CLK0_CONTROL, clkControlReg);
      break;
    case 1:
      stm_i2c_Si5328c_write_single_reg(Si5328_REGISTER_17_CLK1_CONTROL, clkControlReg);
      break;
    case 2:
      stm_i2c_Si5328c_write_single_reg(Si5328_REGISTER_18_CLK2_CONTROL, clkControlReg);
      break;
  }

  return ERROR_NONE;
}
*/

//    @brief  Enables or disables all clock outputs
char stm_Si5328_enableOutputs(void)
{
  // Enabled desired outputs (see Register 0)
	stm_i2c_Si5328c_write_single_reg(Si5328_REGISTER_0_FREERUNEN_CLKOUTON_BYPASS, 0x20);//x-FreeRUN-CKOUT_ALWAYS_ON-x-x-x-BYPASS_REG-x @ FreeRun = route XA/XB to CKIN2. (We disabled),


  return ERROR_NONE;
}

/*!
    @brief  Configures the Si5328 with config settings generated in
            ClockBuilder. You can use this function to make sure that
            your HW is properly configure and that there are no problems
            with the board itself.

    @note   This will overwrite all of the config registers!
*/

char stm_Si5328_setClockBuilderData(void)
{
  uint16_t i = 0;

  /* Make sure we've called init first */
  //if(m_Si5328Config.initialised)
	//  return ERROR_DEVICENOTINITIALISED;

  // Disable all outputs setting CLKx_DIS high
  //stm_i2c_Si5328c_write_single_reg(Si5328_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF);

  // Writes configuration data to device using the register map contents generated by ClockBuilder Desktop (registers)
  for (i=0; i<sizeof(m_Si5328_regs)/2; i++) {
    stm_i2c_Si5328c_write_single_reg( m_Si5328_regs[i][0],
                          m_Si5328_regs[i][1] );
  }


  // Apply soft reset
  //stm_i2c_Si5328c_write_single_reg(Si5328_REGISTER_177_PLL_RESET, 0xAC);



  return ERROR_NONE;
}




//============= main loop =========
void stm_Si5328cLoop (void)
{
	u8 i,  port;
	int regaddr=0; //first address
	int len;
	unsigned long rdata, port_bank_sel;

	printf("Si5351 Loop\r\n");

/*
#if (PROCESSOR == STM32F407VGT6)
	gI2Cx = I2C1;
#elif (PROCESSOR == STM32F407VZT6)
	gI2Cx = I2C2;
#endif

	printf("I2C Init...");
	if(!g_bI2CModuleConfigDone)
		stm_I2C_Init(gI2Cx,400000);//400Kbps - for OLED
	printf("Done.\r\n");

	delayms(100);
	printf("Si5328c I2C Driver Config");
*/

	stm_Si5328_Init();
	//Disable Clocks
	//stm_Si5328_enableOutputs(0);

	stm_Si5328_setClockBuilderData();

	//stm_Si5328_enableOutputs();

/*-- To be modified
	//Integer Only Mode --> Most accurate output
	//Setup PLLA to integer only mode @900MHz(must be 600..900MHz)
	//Set Multisynth 0 to 112.5MHz using integer only mode (div by 4/6/8)
	//25MHz*36=960MHz, then 900MHz/8 => 112.5MHz
	//printf("set plla to 960MHz\r\n");
	//stm_Si5328_setupPLLInt(Si5328_PLL_A,36); //multi must be between 15 and 90
	//printf("set Output 0 to 112.5MHz\r\n");
	//stm_Si5328_setupMultisynthInt(0,Si5328_PLL_A, Si5328_MULTISYNTH_DIV_8);

	//25MHz*15= 375MHz
	printf("set plla to 375MHz(minimum)\r\n");
	stm_Si5328_setupPLLInt(Si5328_PLL_A,15); //multi must be between 15 and 90
	printf("set Output 0 to 46.875MHz\r\n");
	stm_Si5328_setupMultisynthInt(0,Si5328_PLL_A, Si5328_MULTISYNTH_DIV_8);

	//Fractional Mode : More flexible but occurs clock jitter
	//Setup PLLB to fractional mode @616.6666MHz(XTAL*24 +2/3)
	//Setup Multisynth 1 to 13.5531MHz(PLLB/45.5)
	stm_Si5328_setupPLL(Si5328_PLL_B,24,2,3);
	printf("set Output 1 to 13.553115MHz\r\n");
	stm_Si5328_setupMultisynth(1,Si5328_PLL_B, 45,1,2);

	//Multisynth 2 is not yet used and will not be enabled, but can be
	//Use PLLB @616.6666MHz, the divide by 900 -->685.185KHz
	//then divide by 64 for 10.706KHz
	printf("set Output 2 to 10.706KHz\r\n");
	stm_Si5328_setupMultisynth(2,Si5328_PLL_B, 900,0,1);
	stm_Si5328_setupRdiv(2,Si5328_R_DIV_64); //-- NOT WORKING????
	//Enable Clocks

 */
	//stm_Si5328_ShowReg();
	stm_Si5328_ReadConfig();
	//stm_Si5328_enableOutputs(1);
	while(1){

		stm_Si5328_ReadStatus();
		delayms(2000);
	}
}

