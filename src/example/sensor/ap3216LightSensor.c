// Filename: ap3216LightSensor.c
// Ap3216C is combined proximity, ambient light sensor and IRLED.
/* This may also support AP3212B with combined proximity and ambient light sensor.
 * Contact: YC Hou <yc.hou@liteonsemi.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

//stmAp3216ADDR8 = 0x3C
//GPIO PINs
//+-----------------+-----------+-----------+-----------+---------+---------+
//|                 |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| ULED            | PB14      |PC4        |PE15       | <==     | PG7
//+-----------------+-----------+-----------+-----------+---------+---------+
//| BUTTON          |           |PC5(H)     |PD11(index)| PD11(L) |
//+-----------------+-----------+-----------+-----------+---------+
//| AP3216/APSD9960 |           |PA8        | PB8       | PE14(F) |
//+-----------------+-----------+-----------+-----------+---------+
//| VL53L0x         |           |           |           | PC6 (F) |
//+-----------------+-----------+-----------+-----------+---------+
//| BEEP            |           |PB13       |PD14       | <==     |
//+-----------------+-----------+-----------+-----------+---------+
//| QEI             |           |PB0,1,12   |PD12,13,11 | PD12,13 |
//+-----------------+-----------+-----------+-----------+---------+
//*H= Active High/L=Active Low
//*F= FallingEdge
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
#include "stm32f4xx_rcc.h"
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
//#include "stm32f10x_flash.h"
#include "yInc.h"
extern void delayms(uint32_t ms);

#define stmAp3216ADDR8 0x3C //8-bit addr(0011 1100)// 0x1E == 7bit Addr 0001 1110
extern I2C_TypeDef *gI2Cx;
unsigned char stmAp3216Sendbuf[2]; //i2c

uint8_t g_irq_ap3216 = 0;

extern unsigned char g_bI2CModuleConfigDone;
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern int stm_I2C_StartAndAddr(uint8_t address8bit, uint8_t direction);
extern int stm_I2C_SendBurst(unsigned char slave_addr8, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stmLib_ShowRange(unsigned short range, bool normal1_inverted0);

#define Ap3216_NUM_CACHABLE_REGS	26

#define Ap3216_SysConfMode_Reg00H 0x00
#define Ap3216_MODE_SHIFT	(0)
#define Ap3216_MODE_MASK	0x07

#define Ap3216_IntStatus_Reg01H 0x01
#define Ap3216_INT_COMMAND	0x01
#define Ap3216_INT_SHIFT	(0)
#define Ap3216_INT_MASK		0x03
#define Ap3216_INT_PMASK		0x02
#define Ap3216_INT_AMASK		0x01

#define Ap3216_IRdataLow_Reg0AH 0x0A
#define Ap3216_IRdataHigh_Reg0BH 0x0B

#define Ap3216_ALSdataLow_Reg0CH 0x0C
#define Ap3216_ALSdataHigh_Reg0DH 0x0D

#define Ap3216_PSdataLow_Reg0EH 0x0E
#define Ap3216_PSdataHigh_Reg0FH 0x0F
#define	Ap3216_PSdataLow_Reg0EH_MASK	0x0f
#define	Ap3216_PSdataHigh_Reg0FH_MASK	0x3f

#define Ap3216_OBJ_COMMAND	0x0f
#define Ap3216_OBJ_MASK		0x80
#define Ap3216_OBJ_SHIFT	(7)

#define Ap3216_ALSCONF_REG10H 0x10
#define Ap3216_ALSCONF_RANGE_MASK		0x30
#define Ap3216_ALSCONF_RANGE_SHIFT	(4)

#define Ap3216_ALSLowThresholdLsb_REG1AH 0x1A
#define Ap3216_ALSLowThresholdMsb_REG1BH 0x1B
#define Ap3216_ALSHighThresholdLsb_REG1CH 0x1C
#define Ap3216_ALSHighThresholdMsb_REG1DH 0x1D

#define Ap3216_ALS_LowThLsb1AH			0x1a
#define Ap3216_ALS_LowThLsb1AH_SHIFT	(0)
#define Ap3216_ALS_LowThLsb1AH_MASK	0xff

#define Ap3216_ALS_LowThMsb1BH			0x1b
#define Ap3216_ALS_LowThMsb1BH_SHIFT	(0)
#define Ap3216_ALS_LowThMsb1BH_MASK	0xff

#define Ap3216_ALS_HighThLsb1CH			0x1c
#define Ap3216_ALS_HighThLsb1CH_SHIFT	(0)
#define Ap3216_ALS_HighThLsb1CH_MASK	0xff

#define Ap3216_ALS_HighThMsb1DH			0x1d
#define Ap3216_ALS_HighThMsb1DH_SHIFT	(0)
#define Ap3216_ALS_HighThMsb1DH_MASK	0xff

#define Ap3216_PSConf_Reg20H 0x20

#define Ap3216_PSLedControl_Reg21H 0x21
#define Ap3216_PSMeanTime_Reg23H 0x23
#define Ap3216_PSLedWaiting_Reg24H 0x24

#define Ap3216_PSCalibrationFocalLsb_Reg28H 0x28 //xxxx xxxN
#define Ap3216_PSCalibrationFocalMsb_Reg29H 0x29 //PS_Cal[8:1]

//+-----0x2a(LSB) ---+----x2b(MSB)-----+
//| x x x x x x L L  | M M M M M M M M |
//+------------------+-----------------+
#define Ap3216_PSLowThresholdLsb_Reg2AH			0x2a
#define Ap3216_PSLowThresholdLsb_Reg2AH_SHIFT	(0)
#define Ap3216_PSLowThresholdLsb_Reg2AH_MASK	0x03

#define Ap3216_PSLowThresholdMsb_Reg2BH			0x2b
#define Ap3216_PSLowThresholdMsb_Reg2BH_SHIFT	(0)
#define Ap3216_PSLowThresholdMsb_Reg2BH_MASK	0xff
//+-----0x2c(LSB) ---+----x2d(MSB)-----+
//| x x x x x x L L  | M M M M M M M M |
//+------------------+-----------------+
#define Ap3216_PSHighThresholdLsb_Reg2CH		0x2c
#define Ap3216_PSHighThresholdLsb_Reg2CH_SHIFT	(0)
#define Ap3216_PSHighThresholdLsb_Reg2CH_MASK	0x03

#define Ap3216_PSHighThresholdMsb_Reg2DH		0x2d
#define Ap3216_PSHighThresholdMsb_Reg2DH_SHIFT	(0)
#define Ap3216_PSHighThresholdMsb_Reg2DH_MASK	0xff

struct Ap3216_Module {
	u8 reg_cache[Ap3216_NUM_CACHABLE_REGS];
	u8 power_state_before_suspend;
	int irq;
	int cali;
};

struct Ap3216_Module g_Ap3216_Module;

// Ap3216C Register Address Storage
static u8 g_Ap3216_regArrayCache[Ap3216_NUM_CACHABLE_REGS] =
	{0x00,0x01,0x02,0x0a,0x0b,
	 0x0c,0x0d,0x0e,0x0f,
	 0x10,//9: ALS Config Reg: 0x10. Default(0.36Lux/count)=0x00, 0.089Lux = 0x10; 0.0056Lux = 0x30
	 0x19,//10:ALS Calibration (Def=0x40)
	 0x1a,0x1b, //11..12:ALS Low Threshold LSB/MSB
	 0x1c,0x1d, //13..14:ALS High Threshold  LSB/MSB
	 0x20,//15: PS Config Reg : x20. Default = 0x05; IntegrationTime 2T=0x15; 16T=0xF5 16T/Gainx8 = 0xFC
	 0x21,//16: PS LED Config Reg : 0x21. Default(1 LED Pulse) = 0x13; 3 Led Pulses = 0x33
	 0x22,//17:	PS INT Mode :
	 0x23,//18: PS Mean Time
	 0x24,//19: PS LED Waiting
	 0x28,0x29,  //20..21 PS Calibration LSB/MSB
	 0x2a,0x2b,  //22..23 PS LowThrehold_LSB/MSB
	 0x2c,0x2d}; //24..25 PS HighThrehold_LSB/MSB

// Ap3216C
static int g_Ap3216_rangeArray[4] = {23360,5840,1460,265};
static u16 g_Ap3216_ALSthresholdArray[8] = {28,444,625,888,1778,3555,7222,0xffff};

//Macro.
#define GetIndexFromRegAddr(regAddr,idx)	{				\
		int i;												\
		for(i = 0; i < Ap3216_NUM_CACHABLE_REGS; i++)						\
		{													\
			if (regAddr == g_Ap3216_regArrayCache[i])					\
			{												\
				idx = i;									\
				break;										\
			}												\
		}													\
}

//Check Product ID
int stmAp3216CheckProduct(){
	u8 rxbuf[20];
	u8 mid=0;
	u8 pid=0;
	u8 rid=0;

	//GET PID
	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, 0x03, &mid, 1);
	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, 0x04, &pid, 1);
	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, 0x05, &rid, 1);

	if ( (mid == 0x01 && pid == 0x02 && rid == 0x00) || (mid == 0x02 && pid == 0x02 && rid == 0x01)){
		printf("Ap3216 RevID [%d], ==> v2.0 AP3212C/Ap3216C detected\n", rid);
		return 1;
	}
	else{
		printf("MakeID[%d] ProductID[%d] RevID[%d] .... can't detect ... bad reversion!!!\n", mid, pid, rid);
		return 0;
	}
}

static int __Ap3216_Read_Reg_fromLocalCache(u32 reg, u8 mask, u8 shift)
{
	u8 idx = 0xff;

	GetIndexFromRegAddr(reg,idx)
	return (g_Ap3216_Module.reg_cache[idx] & mask) >> shift;
}

void __Ap3216_Write_Reg_ByI2cAndLocalCache(u32 reg, u8 mask, u8 shift, u8 val)
{
	u8 tmp;
	u8 idx = 0xff;

	GetIndexFromRegAddr(reg,idx)
	if (idx >= Ap3216_NUM_CACHABLE_REGS) return;// -1;

	tmp = g_Ap3216_Module.reg_cache[idx];
	tmp &= ~mask;
	tmp |= val << shift;

	stmAp3216Sendbuf[0] = reg;
	stmAp3216Sendbuf[1] = tmp;
	stm_I2C_SendBurst(stmAp3216ADDR8,&stmAp3216Sendbuf[0], 2);//i2c_smbus_write_byte_data(reg, tmp);
	g_Ap3216_Module.reg_cache[idx] = tmp;
}
// Get Range Config Register 0x10
static int Ap3216_Get_RangeFromLocalCache()
{
	u8 idx = __Ap3216_Read_Reg_fromLocalCache(Ap3216_ALSCONF_REG10H, Ap3216_ALSCONF_RANGE_MASK, Ap3216_ALSCONF_RANGE_SHIFT);
	return g_Ap3216_rangeArray[idx];
}

void Ap3216_Set_Range(int range)
{
	__Ap3216_Write_Reg_ByI2cAndLocalCache(Ap3216_ALSCONF_REG10H,	Ap3216_ALSCONF_RANGE_MASK, Ap3216_ALSCONF_RANGE_SHIFT, range);
}

//  mode
static int Ap3216_GetModeFromLocalCache(){
	int ret;

	ret = __Ap3216_Read_Reg_fromLocalCache(Ap3216_SysConfMode_Reg00H,	Ap3216_MODE_MASK, Ap3216_MODE_SHIFT);
	return ret;
}

void Ap3216_SetMode(int mode){
	__Ap3216_Write_Reg_ByI2cAndLocalCache(Ap3216_SysConfMode_Reg00H,	Ap3216_MODE_MASK, Ap3216_MODE_SHIFT, mode);

}
// ALS low threshold
static int Ap3216_get_AlsLowThresholdFromLocalCache(){
	int lsb, msb;
	lsb = __Ap3216_Read_Reg_fromLocalCache(Ap3216_ALS_LowThLsb1AH,Ap3216_ALS_LowThLsb1AH_MASK, Ap3216_ALS_LowThLsb1AH_SHIFT);
	msb = __Ap3216_Read_Reg_fromLocalCache(Ap3216_ALS_LowThMsb1BH,Ap3216_ALS_LowThMsb1BH_MASK, Ap3216_ALS_LowThMsb1BH_SHIFT);
	return ((msb << 8) | lsb);
}

void Ap3216_Set_AlsLowThreshold(int val)
{
	int lsb, msb;

	msb = val >> 8;
	lsb = val & Ap3216_ALS_LowThLsb1AH_MASK;

	__Ap3216_Write_Reg_ByI2cAndLocalCache(Ap3216_ALS_LowThLsb1AH,	Ap3216_ALS_LowThLsb1AH_MASK, Ap3216_ALS_LowThLsb1AH_SHIFT, lsb);
	__Ap3216_Write_Reg_ByI2cAndLocalCache(Ap3216_ALS_LowThMsb1BH,	Ap3216_ALS_LowThMsb1BH_MASK, Ap3216_ALS_LowThMsb1BH_SHIFT, msb);


}

// ALS high threshold
static int Ap3216_Get_AlsHighThresholdFromLocalCache()
{
	int lsb, msb;
	lsb = __Ap3216_Read_Reg_fromLocalCache(Ap3216_ALS_HighThLsb1CH,		Ap3216_ALS_HighThLsb1CH_MASK, Ap3216_ALS_HighThLsb1CH_SHIFT);
	msb = __Ap3216_Read_Reg_fromLocalCache(Ap3216_ALS_HighThMsb1DH,		Ap3216_ALS_HighThMsb1DH_MASK, Ap3216_ALS_HighThMsb1DH_SHIFT);
	return ((msb << 8) | lsb);
}

void Ap3216_Set_AlsHighThreshold(int val)
{
	int lsb, msb;

	msb = val >> 8;
	lsb = val & Ap3216_ALS_HighThLsb1CH_MASK;

	__Ap3216_Write_Reg_ByI2cAndLocalCache(Ap3216_ALS_HighThLsb1CH,	Ap3216_ALS_HighThLsb1CH_MASK, Ap3216_ALS_HighThLsb1CH_SHIFT, lsb);
	__Ap3216_Write_Reg_ByI2cAndLocalCache(Ap3216_ALS_HighThMsb1DH,	Ap3216_ALS_HighThMsb1DH_MASK, Ap3216_ALS_HighThMsb1DH_SHIFT, msb);

}

// PX low threshold (10-bits)
// Ap3216_PSLowThresholdLsb_Reg2AH [1:0]
// Ap3216_PSLowThresholdMsb_Reg2BH [7:0]
//+-----0x2a(LSB) ---+----x2b(MSB)-----+
//| x x x x x x L L  | M M M M M M M M |
//+------------------+-----------------+

void Ap3216_Set_PsLowThreshold_apiByI2cAndUpdateLocalCache(int val){
	int lsb, msb, err;

	msb = val >> 2;
	lsb = val & Ap3216_PSLowThresholdLsb_Reg2AH_MASK; //0x03

	__Ap3216_Write_Reg_ByI2cAndLocalCache(Ap3216_PSLowThresholdLsb_Reg2AH,	Ap3216_PSLowThresholdLsb_Reg2AH_MASK, Ap3216_PSLowThresholdLsb_Reg2AH_SHIFT, lsb);
	__Ap3216_Write_Reg_ByI2cAndLocalCache(Ap3216_PSLowThresholdMsb_Reg2BH, Ap3216_PSLowThresholdMsb_Reg2BH_MASK, Ap3216_PSLowThresholdMsb_Reg2BH_SHIFT, msb);
}
//PS low threshold = Reg_0x2B * 4 + Reg_0x2A
static int Ap3216_Get_PsLowThresholdFromLocalCache()
{
	int lsb, msb;
	lsb = __Ap3216_Read_Reg_fromLocalCache(Ap3216_PSLowThresholdLsb_Reg2AH,Ap3216_PSLowThresholdLsb_Reg2AH_MASK, Ap3216_PSLowThresholdLsb_Reg2AH_SHIFT);
	msb = __Ap3216_Read_Reg_fromLocalCache(Ap3216_PSLowThresholdMsb_Reg2BH,Ap3216_PSLowThresholdMsb_Reg2BH_MASK, Ap3216_PSLowThresholdMsb_Reg2BH_SHIFT);
	return ((msb << 2) | lsb);
}
//+-----0x2c(LSB) ---+----x2d(MSB)-----+
//| x x x x x x L L  | M M M M M M M M |
//+------------------+-----------------+

// PX high threshold
void Ap3216_Set_PsHighThresholdByI2cAndUpdateLocalCache(int val)
{
	int lsb, msb;

	msb = val >> 2;
	lsb = val & Ap3216_PSHighThresholdLsb_Reg2CH_MASK; //0x03

	__Ap3216_Write_Reg_ByI2cAndLocalCache(Ap3216_PSHighThresholdLsb_Reg2CH,	Ap3216_PSHighThresholdLsb_Reg2CH_MASK, Ap3216_PSHighThresholdLsb_Reg2CH_SHIFT, lsb);
	__Ap3216_Write_Reg_ByI2cAndLocalCache(Ap3216_PSHighThresholdMsb_Reg2DH,	Ap3216_PSHighThresholdMsb_Reg2DH_MASK, Ap3216_PSHighThresholdMsb_Reg2DH_SHIFT, msb);

}
// PS high threshold = Reg_0x2D * 4 + Reg_0x2C
static int Ap3216_Get_PsHighThresholdFromLocalCache(){
	int lsb, msb;
	lsb = __Ap3216_Read_Reg_fromLocalCache(Ap3216_PSHighThresholdLsb_Reg2CH,	Ap3216_PSHighThresholdLsb_Reg2CH_MASK, Ap3216_PSHighThresholdLsb_Reg2CH_SHIFT);
	msb = __Ap3216_Read_Reg_fromLocalCache(Ap3216_PSHighThresholdMsb_Reg2DH,	Ap3216_PSHighThresholdMsb_Reg2DH_MASK, Ap3216_PSHighThresholdMsb_Reg2DH_SHIFT);
	return ((msb << 2) | lsb);
}

static int Ap3216_Get_AlsValueInAdcByI2cAndReturnThresholdStepIndex(unsigned int *retAlsValue, unsigned int *retCalibratedAlsInLux){
	unsigned int msb,lsb, val;
	unsigned char buf;
	unsigned int tmp,range;
	unsigned char retThresholdIndex=0;

	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8,Ap3216_ALSdataLow_Reg0CH, &buf,1); //i2c_smbus_read_byte_data(Ap3216_ALSdataLow_Reg0CH);
	lsb = buf;
	//if (lsb < 0) {	return lsb;	}
	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8,Ap3216_ALSdataHigh_Reg0DH, &buf,1); //i2c_smbus_read_byte_data(Ap3216_ALSdataHigh_Reg0DH);
	msb = buf;

	range = Ap3216_Get_RangeFromLocalCache();
	tmp = (((msb << 8) | lsb) * range) >> 16;
	tmp = tmp * g_Ap3216_Module.cali / 100;
	*retCalibratedAlsInLux = tmp;

	val = msb << 8 | lsb;
	//printf("ALS val=%d(msb=%x;lsb=%x \r\n",tmp,msb,lsb);
	*retAlsValue = val;
	for(retThresholdIndex = 0; retThresholdIndex < 7 && val > g_Ap3216_ALSthresholdArray[retThresholdIndex];retThresholdIndex++)
		;
	return retThresholdIndex;
}
//1= close to ; 0= away from and cross the thresold
static int Ap3216_Get_ObjectFromPsRegByI2C()
{
	int val;

	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8,Ap3216_OBJ_COMMAND, &val,1); //i2c_smbus_read_byte_data(Ap3216_OBJ_COMMAND);
	val &= Ap3216_OBJ_MASK; //0x80

	return val >> Ap3216_OBJ_SHIFT;
}

static int Ap3216_Get_Ps_Value_ByI2c(){
	int lsb, msb;
	unsigned char b;

	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_PSdataLow_Reg0EH, &b,1); //i2c_smbus_read_byte_data(Ap3216_PSdataLow_Reg0EH);
	lsb = b;

	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_PSdataHigh_Reg0FH, &b,1);//i2c_smbus_read_byte_data(Ap3216_PSdataHigh_Reg0FH);
	msb = b;
	//printf("PS>LSB=%02x MSB=%02x \r\n",msb,lsb);
	if(lsb & 0x40)	printf("OVF: \r\n");
	else printf("\r\n");
	return (u32)(((msb & Ap3216_PSdataHigh_Reg0FH_MASK) << 4) | (lsb & Ap3216_PSdataLow_Reg0EH_MASK));
}


void Ap3216_ALSsensor_Enable()
{
	int ret = 0,mode;

	mode = Ap3216_GetModeFromLocalCache();
	if((mode & 0x01) == 0){
		mode |= 0x01;
		Ap3216_SetMode(mode);
	}

	delayms(200);//msleep(200);
	stmAp3216Change_ALS_threshold();
}

void Ap3216_ALSsensor_Disable()
{
	int ret = 0,mode;

	mode = Ap3216_GetModeFromLocalCache();
	if(mode & 0x01){
		mode &= ~0x01;
		if(mode == 0x04)
			mode = 0;
		Ap3216_SetMode(mode);
	}
}


void Ap3216_PSsensor_Enable()
{
	int ret = 0,mode;

	mode = Ap3216_GetModeFromLocalCache();
	if((mode & 0x02) == 0){
		mode |= 0x02;
		Ap3216_SetMode(mode);
	}
	delayms(200);

}

void Ap3216_PSsensor_Disable()
{
	int ret = 0,mode;

	mode = Ap3216_GetModeFromLocalCache();
	if(mode & 0x02){
		mode &= ~0x02;
		if(mode == 0x04)
			mode = 0x00;
		Ap3216_SetMode(mode);
	}
	return ret;
}
void stmAp3216Change_ALS_threshold()
{
	int stepIndex;
	unsigned int retValue, retCalibratedValue;

	stepIndex = Ap3216_Get_AlsValueInAdcByI2cAndReturnThresholdStepIndex(&retValue, &retCalibratedValue);
	//printf("ALS lux index: %u\r\n", index);
	if(stepIndex > 0){
		Ap3216_Set_AlsLowThreshold(g_Ap3216_ALSthresholdArray[stepIndex-1]);
		Ap3216_Set_AlsHighThreshold(g_Ap3216_ALSthresholdArray[stepIndex]);
	}
	else{
		Ap3216_Set_AlsLowThreshold(0);
		Ap3216_Set_AlsHighThreshold(g_Ap3216_ALSthresholdArray[stepIndex]);
	}
}

void Ap3216_Show_Range_api()
{
	printf("%i\n", Ap3216_Get_RangeFromLocalCache());
}

void Ap3216_Store_Range(int val)
{
	int ret;

	if ((val < 0) || (val > 3))
		return -1;

	Ap3216_Set_Range(val);
}


// mode
void Ap3216_Show_Mode()
{
	printf("%d\n", Ap3216_GetModeFromLocalCache());
}

void Ap3216_Store_Mode(char val)
{
	int ret;

	if ((val < 0) || (val > 7)){
		printf("Err> Mode should be 0..7\r\n");
		return; //error
	}

	Ap3216_SetMode(val);

}

// lux
int Ap3216_Show_Lux_api()
{
	int stepIndex;
	unsigned int retValue, retCalibratedValue;

	if (Ap3216_GetModeFromLocalCache() == 0x00){// No LUX data if power down
		printf("Err> Please power up first!\r\n");
		return;
	}

	stepIndex = Ap3216_Get_AlsValueInAdcByI2cAndReturnThresholdStepIndex(&retValue, &retCalibratedValue);
	// printf("ALS> Calibrated Lux = %d.\r\n", retCalibratedValue);
	return retCalibratedValue;
}


// Px data
eResultStatus Ap3216_Show_PsValue_api()
{
	int range;
	// No Px data if power down
	if (Ap3216_GetModeFromLocalCache() == 0x00){
		printf("Err> Please power up first!\r\n");
		return FAILED;
	}
	range = Ap3216_Get_Ps_Value_ByI2c();
	//printf("PS>Value=%d\r\n", Ap3216_Get_Ps_Value_ByI2c());
	stmLib_ShowRange(range, 0); //0 = inverted
	return PASSED;
}

// proximity object detect
void Ap3216_Show_Ps_Object_api()
{
	printf("ShowObj>%d\r\n", Ap3216_Get_ObjectFromPsRegByI2C());
}
// ALS low threshold
void Ap3216_show_ALS_LOW_thres_api()
{
	printf("ShowALS_LOWthreshold>%d\r\n", Ap3216_get_AlsLowThresholdFromLocalCache());
}

void Ap3216_store_ALS_LOW_thres_api(int val)
{
	Ap3216_Set_AlsLowThreshold(val);
}

// ALS high threshold
void Ap3216_show_ALS_HIGH_thres_api()
{
	printf("ShowALS_HIGHthreshold>%d\r\n", Ap3216_Get_AlsHighThresholdFromLocalCache());
}

void Ap3216_store_ALS_HIGH_thres_api(int val)
{
	Ap3216_Set_AlsHighThreshold(val);
}

// Px low threshold
void Ap3216_Show_PsLowThreshold_api()
{
	printf("PS>LowThreshold = %d\r\n", Ap3216_Get_PsLowThresholdFromLocalCache());
}

void Ap3216_Set_PsLowThreshold_api(unsigned int val)
{
	Ap3216_Set_PsLowThreshold_apiByI2cAndUpdateLocalCache(val);
}

// Px high threshold
void Ap3216_Show_PsHighThreshold_api()
{
	printf("PS>HighThreshold = %u\r\n", Ap3216_Get_PsHighThresholdFromLocalCache());
}

static int Ap3216_Set_PsHighThreshold_api(unsigned int val)
{
	int ret;

	Ap3216_Set_PsHighThresholdByI2cAndUpdateLocalCache( val);

	return 1;
}
// calibration
static int Ap3216_Show_Calibration_State()
{
	printf("%d\n", g_Ap3216_Module.cali);
}
//???
static int Ap3216_store_calibration_state(int stdls) //standard light source
{
	int  lux_StepIndex; //actually it is the index.
	char tmp[10];
	unsigned int retValue, retCalibratedValue;

	// No LUX data if not operational
	if (Ap3216_GetModeFromLocalCache() == 0x00)	{
		printf("Please power up first!");
		return -1;
	}

	g_Ap3216_Module.cali = 100;

	if (stdls < 0){
		printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\	Set calibration factor to 100.\n", stdls);
		return -1;
	}

	lux_StepIndex = Ap3216_Get_AlsValueInAdcByI2cAndReturnThresholdStepIndex(&retValue, &retCalibratedValue);
	g_Ap3216_Module.cali = stdls * 100 / lux_StepIndex; //???

	return 1;
}

#if (USE_EXTI9_5 ==	USE_EXTI9_5_AP3216)
//PA8
void EXTI9_5_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line8);		// Clear the EXTI line 8 pending bit
		// Toggle LED0
		GPIO_ToggleBits(GPIOC, GPIO_Pin_4); // ULED ON (PC4)
		g_irq_ap3216 = 1;
	}
}
#endif

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F401RET6)
//Config IRQ Pin (PA8) : 103C8 or 401-RET
void stmAp3216_PA8_IRQpin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;
#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
	  // Enable GPIOA clock
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	  // Configure PA8 pin as input floating
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  // Connect EXTI Line8 to PA8 pin
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);

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
#else
	  /* Enable GPIOA clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  // Configure PA8 pin as input floating
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //???
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //GPIO_PuPd_NOPULL;
	  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  // Connect EXTI Line8 to PA8 pin
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
#endif
}
#else
//+-----------------+-----------+-----------+-----------+---------+---------+---------
//|                 |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   | 103
//+-----------------+-----------+-----------+-----------+---------+---------+--------
//| AP3216/APSD9960 |           |PA8        | PB8       | PE14(F) |         | PA8

void stmAp3216_PC6_IRQpin_Setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))

#else
	  /* Enable GPIOC clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	  /* Configure PC6 pin as input floating */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //???
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  // Connect EXTI Line6 to PC6 pin
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);

	  // Configure EXTI Line6
	  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Line8 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
#endif
}
//M36
//extern void stmAPDS9960_AP3216_PE14_IRQpin_Setup(void);
//extern void EXTI15_10_IRQHandler();//in apds9960
//extern void EXTI9_5_IRQHandler(); //in apds9960
//extern uint8_t g_irq;//in apds9960
#endif

static int Ap3216_Get_InterruptStatusByI2c(){
	int val;

	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_INT_COMMAND, &val,1);//i2c_smbus_read_byte_data(Ap3216_INT_COMMAND);
	val &= Ap3216_INT_MASK;

	return val >> Ap3216_INT_SHIFT;
}

u8 stmAp3216_CheckIrqThenServeIt()
{
	u8 rxbuf[20];
	u8 int_stat;
	int Pval;
	u8 retVal = 0;

	if(g_irq_ap3216 == 0)
		return 0;

	g_irq_ap3216 = 0;

	int_stat = Ap3216_Get_InterruptStatusByI2c();//get IRQ status

	// ALS int
	if (int_stat & Ap3216_INT_AMASK) //0x01
	{
		stmAp3216Change_ALS_threshold();//Ap3216_Change_ALS_Threshold();
		//Clear ALS IRQ Flag by reading ALS Data Register
		//stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_ALSdataHigh_Reg0DH, &rxbuf[0], 1);

		retVal = 1;
	}

	// PX int
	if (int_stat & Ap3216_INT_PMASK) //0x02
	{
		Pval = Ap3216_Get_ObjectFromPsRegByI2C();
		//Clear PS IRQ Flag by reading PS Data Register
		//stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_PSdataHigh_Reg0FH, &rxbuf[0], 1);

		printf("PS>%s\r\n", Pval ? "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< obj near":">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>obj far");
		retVal = 2;
	}
	return retVal;
}

int stmAp3216_Init(unsigned char enALS, unsigned char enPS)
{
	int i,v;

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
	//Check Product ID
	while(!stmAp3216CheckProduct()){};
	//Read All Registers for checking.
	stmAp3216ShowAllReg();

#if(PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F401RET6)
	//IRQ : PA8
	stmAp3216_PA8_IRQpin_Setup(); //103 or stm401
#else
	//stmAPDS9960_PC6_IRQpin_Setup();//407-M34
	//IRQ : PE14 - M36
	stmAPDS9960_AP3216_PE14_IRQpin_Setup();//stmAp3216_PC6_IRQpin_Setup(); //stm407-M35
#endif

	/// read all the registers once to fill the cache.  if one of the reads fails, we consider the init failed
	for (i = 0; i < Ap3216_NUM_CACHABLE_REGS; i++) {
		stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, g_Ap3216_regArrayCache[i], &v,1); //i2c_smbus_read_byte_data(reg_array[i]);
		if (v < 0)
			return -1;

		g_Ap3216_Module.reg_cache[i] = v;
	}

	g_Ap3216_Module.cali = 100;
	// set defaults
	Ap3216_Set_Range(0);
	Ap3216_SetMode(0);

	if(enALS)
		Ap3216_ALSsensor_Enable();
	if(enPS)
		Ap3216_PSsensor_Enable();

	//Ap3216_Set_AlsLowThreshold(200);
	//Ap3216_Set_AlsHighThreshold(1000);

	stmAp3216_PsConfig(	10, //Low Threshold
						100);//High Threshold


	return 0;
}

void stmAp3216ShowAllReg(){
	int i;
	u8 rxbuf[20];
	for(i=0;i<=0x2D;i++){
		if(stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, i, &rxbuf[0], 1)){
			printf("Reg[0x%02x]=0x%02x\r\n",i, rxbuf[0]);
			delayms(10);
		}else{
			printf("I2C NO RESPONSE FROM SLAVE.\r\n");
		}
	}
}
void stmAp3216_PsConfig(int psLowThreshold, int psHighThreshold){
	//Reg20 : PS Config Reg.
	stmAp3216Sendbuf[0] = Ap3216_PSConf_Reg20H; //Reg20
	stmAp3216Sendbuf[1] = 0xF5; //0x36; //0xF5;//0x05; //PS/IR Integration Time =F=x16
	stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);

	//Reg21 : PS LED Control(Default=0x13)
	stmAp3216Sendbuf[0] = Ap3216_PSLedControl_Reg21H; //Reg21
	//stmAp3216Sendbuf[1] = 0x33; //3= 3 Pulses. 3=Max Driver Ratio = 100%
	stmAp3216Sendbuf[1] = 0x31;//0x31; //3= 3 Pulses. 3=Max Driver Ratio = 100%
	stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);

	//Reg22 - default = 0x01
	stmAp3216Sendbuf[0] = 0x22; //PS_INT_Mode; //Reg22
	stmAp3216Sendbuf[1] = 0x01; //Hysteresis Type(Mode2) //0x00; =Zone Type
	stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);

	//Reg23 PS MeanTime
	stmAp3216Sendbuf[0] = Ap3216_PSMeanTime_Reg23H;
	stmAp3216Sendbuf[1] = 0x00;//0x3; // 00=12.5msec; 11=50msec
	stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);

	//Reg24 PS LED WaitingTime
	stmAp3216Sendbuf[0] = Ap3216_PSLedWaiting_Reg24H; //PS LED Waiting Time Reg
	stmAp3216Sendbuf[1] = 0x00;//0x01; // 1 MeanTime; 0=Default
	stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);
/*
	//Focal
	stmAp3216Sendbuf[0] = Ap3216_PSCalibrationFocalMsb_Reg29H; //
	stmAp3216Sendbuf[1] = 0xAA;//0x0F;
	stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);

*/
	//Set Thresholds -- psHighThreshold< 1024
	Ap3216_Set_PsLowThreshold_api(psLowThreshold);
	Ap3216_Set_PsHighThreshold_api(psHighThreshold);
	//Show them.
	Ap3216_Show_PsLowThreshold_api();
	Ap3216_Show_PsHighThreshold_api();
}

//extern bool g_I2C_InitDone;
extern unsigned char g_bI2CModuleConfigDone;


void stmAp3216Loop (void)
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

	printf("stmAp3216 Light Sensor Test with I2C1\r\n");

	//LED
	//stmUser_LED_GPIO_setup(); //Accessory LED

	//stmConfirmLEDBlink();

	delayms(100);

	stmAp3216_Init(1,1);//enable ALS, and PS

	/* LED CUrrent = 60mA
	 * * INTEGRATION TIME=300usec
	 * THUP{MSB}=0x8F
	 * THUP_LSB = 0xFF
	 * THLOMSB=0x70
	 * THLOLSB=0x99
	 * FILTER : OFF

	 * PS INTERVAL WAIT = 50ms
	 * ENABLE
	 *
	 */

	//Clear IRQ
	//stmAp3216CheckAndClrIRQ();
	//Reg10 - default
	//stmAp3216Sendbuf[0] =  Ap3216_ALSCONF_REG10H; //Reg10
	//stmAp3216Sendbuf[1] = 0x00; //default : ALS DynamicRange=20661 Lux. ALS Persist = 0000(ALS Int after 1 conversion time)
	//stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);

	printf("...\r\n");


    while(1){

		Ap3216_Show_Lux_api();
		Ap3216_Show_PsValue_api();
		delayms(10);
/*
		 //IRQ Based
    	retVal = stmAp3216_CheckIrqThenServeIt();
    	if(retVal > 0){
    		Ap3216_Show_Lux_api();
    		Ap3216_Show_PsValue_api();
    		printf("%u\r\n",retVal);

        	printf("PS>HIGHTH=%d\r\n",Ap3216_Get_PsHighThresholdFromLocalCache());
        	printf("PS>LOWTH=%d\r\n",Ap3216_Get_PsLowThresholdFromLocalCache());
    	}


    	//Ap3216_show_ALS_LOW_thres_api();
    	//Ap3216_show_ALS_HIGH_thres_api();
*/


/*
    	ints = stmAp3216CheckAndClrIRQ();//IRQ Status Read
    	if(ints != 0){

    		//get ALS
    		stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_ALSdataLow_Reg0CH, &rxbuf[0], 1);
    		stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_ALSdataHigh_Reg0DH, &rxbuf[1], 1);
    		lsb = rxbuf[0];
    		msb = (unsigned short)rxbuf[1]*256;
    		printf("ALS(12/13)=%d (0x%02x,0x%02x)\r\n",msb + lsb, rxbuf[0], rxbuf[1]);
    		delayms(10);

    		//get IR
    		stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_IRdataLow_Reg0AH, &rxbuf[0], 1); //L
    		stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_IRdataHigh_Reg0BH, &rxbuf[1], 1); //H
    		msb = rxbuf[1];
    		lsb = rxbuf[0];
    		ird = msb*4 + (lsb & 0x03);
    		ir_ps_InValid = (lsb & 0x80) >> 7;

    		//if(ir_ps_InValid){
    		//	printf("IR/PS : Invalid\r\n");
    		//}else{
    			//get ps
    			stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_PSdataLow_Reg0EH, &rxbuf[0], 1); //LOW+HIGH
    			stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_PSdataLow_Reg0EH, &rxbuf[1], 1); //LOW+HIGH
    			msb = rxbuf[1];
    			lsb = rxbuf[0];
    			printf("IR(10/11)=%d (IV=%d)\r\n", ird, ir_ps_InValid );
    			printf("PS(14/15)=%d/OVF=%d){[14L]0x%02x,[15H]0x%02x}\r\n",((msb & 0x3f) * 16) + (lsb & 0x0F) , (lsb & 0x40)>>6, lsb, msb);
    			delayms(300);
    		//}
    		 *

    		printf("================\r\n");
    	}
    	*/
        delayms(10);
    }
}


/*
void stmAp3216AlsStart(){
	stmAp3216Sendbuf[0] = Ap3216_SysConfMode_Reg00H; //Reg0
	stmAp3216Sendbuf[1] = 0x01; // ALS and PS+IR active all. 0x01; //0x03 = ALS and PS+IR active all.
	stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);
}
void stmAp3216PsStart(){
	stmAp3216Sendbuf[0] = Ap3216_SysConfMode_Reg00H; //Reg0
	stmAp3216Sendbuf[1] = 0x02;
	stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);
}
void stmAp3216AlsAndPsStart(){
	stmAp3216Sendbuf[0] = Ap3216_SysConfMode_Reg00H; //Reg0
	stmAp3216Sendbuf[1] = 0x03; // ALS and PS+IR active all. 0x01; //0x03 = ALS and PS+IR active all.
	stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);
}
void stmAp3216AlsStop(){
	stmAp3216Sendbuf[0] = Ap3216_SysConfMode_Reg00H;
	stmAp3216Sendbuf[1] = 0x00;
	stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);
}

int stmAp3216CheckAndClrIRQ(){
	u8 rxbuf[20];
	u8 ints;
	u8 ir_ps_InValid;
	//get IRQ status
	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_IntStatus_Reg01H, &ints, 1); //L
	if(ints)
		printf("IrqStatusReg(0x01)=0x%02x\r\n", ints);
	else {
		printf(".");
	}

	//Clear ALS IRQ Flag by reading ALS Data Register
	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_ALSdataHigh_Reg0DH, &rxbuf[0], 1);
	//Clear PS IRQ Flag by reading PS Data Register
	stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_PSdataHigh_Reg0FH, &rxbuf[0], 1);
	//
	return ints;
}
*/
/*
static int stmAp3216Get_intstat(){
	u8 int_stat;

	int_stat = stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_IntStatus_Reg01H, &int_stat, 1); //Lstm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, //i2c_smbus_read_byte_data(Ap3216_INT_COMMAND);
	int_stat &= Ap3216_INT_MASK; //0x03

	return int_stat;// >> Ap3216C_INT_SHIFT; //0
}
*/
/*
static void stmAp3216irq(int irq, void *data_)
{
	struct Ap3216_Module *data = data_;
	u8 int_stat;
	int Pval;

	int_stat = stmAp3216Get_intstat();//stmAp3216CheckAndClrIRQ();//IRQ Status Read
	// ALS int
	if (int_stat & Ap3216C_INT_AMASK) //0x01
	{
		stmAp3216Change_ALS_threshold();
	}

	// PX int
	if (int_stat & Ap3216C_INT_PMASK) //0x02
	{
		Pval = Ap3216_Get_ObjectFromPsRegByI2C();
		printf("%s\n", Pval ? "obj near":"obj far");
		//input_report_abs(data->psensor_input_dev, ABS_DISTANCE, Pval);
		//input_sync(data->psensor_input_dev);
	}

    return IRQ_HANDLED;
}
*/

void AP3216setup(void)
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

	printf("stmAp3216 Light Sensor Test with I2C1\r\n");


	delayms(100);

	stmAp3216_Init(1,1);//enable ALS, and PS


	printf("...\r\n");
}

int AP3216get(void)
{
	return Ap3216_Show_Lux_api();
}
