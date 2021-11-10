#include <string.h>
#include <stdarg.h>
#include "example/yInc.h"
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
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;

//KT0803L's I2C addresses
#define KT0803L_ADDR8  (0x7c) //(0x3e)

/*
 * KT0803L driver 
 */
//KT0803L's Register addresses & maps
#define KT0803L_REG_ADDR_00		(0x00)
#define KT0803L_REG_ADDR_01		(0x01)
#define KT0803L_REG_ADDR_02		(0x02)
#define KT0803L_REG_ADDR_04		(0x04)
#define KT0803L_REG_ADDR_0B		(0x0b)
#define KT0803L_REG_ADDR_0C		(0x0c)
#define KT0803L_REG_ADDR_0E		(0x0e)
#define KT0803L_REG_ADDR_0F		(0x0f)
#define KT0803L_REG_ADDR_10		(0x10)
#define KT0803L_REG_ADDR_12		(0x12)
#define KT0803L_REG_ADDR_13		(0x13)
#define KT0803L_REG_ADDR_14		(0x14)
#define KT0803L_REG_ADDR_15		(0x15)
#define KT0803L_REG_ADDR_16		(0x16)
#define KT0803L_REG_ADDR_17		(0x17)
#define KT0803L_REG_ADDR_1E		(0x1e)
#define KT0803L_REG_ADDR_26		(0x26)
#define KT0803L_REG_ADDR_27		(0x27)

typedef union {
	struct {
		unsigned char CHSEL_8_1		: 8;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_00;

typedef union {
	struct {
		unsigned char CHSEL_11_9	: 3;
		unsigned char PGA_2_0		: 3;
		unsigned char RFGAIN_1_0	: 2;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_01;

typedef union {
	struct {
		unsigned char PHTCNST		: 1;
		unsigned char unused0		: 1;
		unsigned char PLTADJ		: 1;
		unsigned char MUTE			: 1;
		unsigned char unused1		: 2;
		unsigned char RFGAIN_3		: 1;
		unsigned char CHSEL_0		: 1;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_02;

typedef union {
	struct {
		unsigned char BASS_1_0		: 1;
		unsigned char unused0		: 3;
		unsigned char PGA_LSB_1_0	: 2;
		unsigned char MONO			: 1;
		unsigned char ALC_EN		: 1;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_04;

typedef union {
	struct {
		unsigned char unused0		: 2;
		unsigned char AUTO_PADN		: 1;
		unsigned char unused1		: 2;
		unsigned char PDPA			: 1;
		unsigned char unused2		: 1;
		unsigned char STANDBY		: 1;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_0B;

typedef union {
	struct {
		unsigned char ALC_ATTACH_TIME_3_0	: 4;
		unsigned char ALC_DECAY_TIME_3_0	: 4;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_0C;

typedef union {
	struct {
		unsigned char unused0		: 1;
		unsigned char PA_BIAS		: 1;
		unsigned char unused1		: 6;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_0E;

typedef union {
	struct {
		unsigned char unused0		: 2;
		unsigned char SLNCID		: 1;
		unsigned char unused1		: 1;
		unsigned char PW_OK			: 1;
		unsigned char unused2		: 3;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_0F;

typedef union {
	struct {
		unsigned char PGAMOD		: 1;
		unsigned char unused0		: 7;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_10;

typedef union {
	struct {
		unsigned char SW_MOD		: 1;
		unsigned char SLNCTHH_2_0	: 3;
		unsigned char SLNCTHL_2_0	: 3;
		unsigned char SLNCDIS		: 1;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_12;

typedef union {
	struct {
		unsigned char unused0		: 2;
		unsigned char PA_CTRL		: 1;
		unsigned char unused1		: 4;
		unsigned char RFGAIN_2		: 1;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_13;

typedef union {
	struct {
		unsigned char SLNCTIME_3		: 1;
		unsigned char unused0			: 1;
		unsigned char SLNCCNTHIGH_2_0	: 3;
		unsigned char SLNCTIME_2_0		: 3;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_14;

typedef union {
	struct {
		unsigned char unused0			: 5;
		unsigned char ALCCMPGAIN_2_0	: 3;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_15;

typedef union {
	struct {
		unsigned char SLNCCNTLOW_2_0	: 3;
		unsigned char unused0			: 5;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_16;

typedef union {
	struct {
		unsigned char unused0		: 3;
		unsigned char XTAL_SEL		: 1;
		unsigned char unused1		: 1;
		unsigned char AU_ENHANCE	: 1;
		unsigned char FDEV			: 1;
		unsigned char unused2		: 1;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_17;

typedef union {
	struct {
		unsigned char REF_CLK_3_0	: 4;
		unsigned char unused0		: 1;
		unsigned char XTALD			: 1;
		unsigned char DCLK			: 1;
		unsigned char unused1		: 1;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_1E;

typedef union {
	struct {
		unsigned char unused0		: 1;
		unsigned char ALCHIGHTH_2_0	: 3;
		unsigned char unused1		: 1;
		unsigned char ALCHOLD_2_0	: 3;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_26;

typedef union {
	struct {
		unsigned char ALCLOWTH_3_0	: 4;
		unsigned char unused0		: 4;
	} f;
	unsigned char b;
} __attribute__((__packed__)) KT0803L_REG_27;

//register values
enum {
	KT0803L_CLK_FREQ_32_768KHZ=0,
	KT0803L_CLK_FREQ_6_5MHZ,
	KT0803L_CLK_FREQ_7_6MHZ,
	KT0803L_CLK_FREQ_12MHZ,
	KT0803L_CLK_FREQ_13MHZ,
	KT0803L_CLK_FREQ_15_2MHZ,
	KT0803L_CLK_FREQ_19_2MHZ,
	KT0803L_CLK_FREQ_24MHZ,
	KT0803L_CLK_FREQ_26MHZ
};

enum {
	KT0803L_SD_TIME_50MS=0,
	KT0803L_SD_TIME_100MS,
	KT0803L_SD_TIME_200MS,
	KT0803L_SD_TIME_400MS,
	KT0803L_SD_TIME_1S,
	KT0803L_SD_TIME_2S,
	KT0803L_SD_TIME_4S,
	KT0803L_SD_TIME_8S,
	KT0803L_SD_TIME_16S,
	KT0803L_SD_TIME_24S,
	KT0803L_SD_TIME_32S,
	KT0803L_SD_TIME_40S,
	KT0803L_SD_TIME_48S,
	KT0803L_SD_TIME_56S,
	KT0803L_SD_TIME_60S,
	KT0803L_SD_TIME_64S,
};

enum {
	KT0803L_SD_LTH_0_25MV=0,
	KT0803L_SD_LTH_0_5MV,
	KT0803L_SD_LTH_1MV,
	KT0803L_SD_LTH_2MV,
	KT0803L_SD_LTH_4MV,
	KT0803L_SD_LTH_8MV,
	KT0803L_SD_LTH_16MV,
	KT0803L_SD_LTH_32MV
};

enum {
	KT0803L_SD_HTH_0_5MV=0,
	KT0803L_SD_HTH_1MV,
	KT0803L_SD_HTH_2MV,
	KT0803L_SD_HTH_4MV,
	KT0803L_SD_HTH_8MV,
	KT0803L_SD_HTH_16MV,
	KT0803L_SD_HTH_32MV,
	KT0803L_SD_HTH_64MV
};

enum {
	KT0803L_SD_LCNT_1=0,
	KT0803L_SD_LCNT_2,
	KT0803L_SD_LCNT_4,
	KT0803L_SD_LCNT_8,
	KT0803L_SD_LCNT_16,
	KT0803L_SD_LCNT_32,
	KT0803L_SD_LCNT_64,
	KT0803L_SD_LCNT_128
};

enum {
	KT0803L_SD_HCNT_15=0,
	KT0803L_SD_HCNT_31,
	KT0803L_SD_HCNT_63,
	KT0803L_SD_HCNT_127,
	KT0803L_SD_HCNT_255,
	KT0803L_SD_HCNT_511,
	KT0803L_SD_HCNT_1023,
	KT0803L_SD_HCNT_2047
};

enum {
	KT0803L_BB_DISABLE=0,
	KT0803L_BB_LVL_5DB,
	KT0803L_BB_LVL_11DB,
	KT0803L_BB_LVL_17DB
};

enum {
	KT0803L_PGA_GAIN_12DB=0x1f,
	KT0803L_PGA_GAIN_11DB=0x1e,
	KT0803L_PGA_GAIN_10DB=0x1d,
	KT0803L_PGA_GAIN_9DB=0x1c,
	KT0803L_PGA_GAIN_8DB=0x1b,
	KT0803L_PGA_GAIN_7DB=0x1a,
	KT0803L_PGA_GAIN_6DB=0x19,
	KT0803L_PGA_GAIN_5DB=0x18,
	KT0803L_PGA_GAIN_4DB=0x17,
	KT0803L_PGA_GAIN_3DB=0x16,
	KT0803L_PGA_GAIN_2DB=0x15,
	KT0803L_PGA_GAIN_1DB=0x14,
	KT0803L_PGA_GAIN_0DB=0x00,
	KT0803L_PGA_GAIN_M1DB=0x01,
	KT0803L_PGA_GAIN_M2DB=0x02,
	KT0803L_PGA_GAIN_M3DB=0x03,
	KT0803L_PGA_GAIN_M4DB=0x04,
	KT0803L_PGA_GAIN_M5DB=0x05,
	KT0803L_PGA_GAIN_M6DB=0x06,
	KT0803L_PGA_GAIN_M7DB=0x07,
	KT0803L_PGA_GAIN_M8DB=0x08,
	KT0803L_PGA_GAIN_M9DB=0x09,
	KT0803L_PGA_GAIN_M10DB=0x0a,
	KT0803L_PGA_GAIN_M11DB=0x0b,
	KT0803L_PGA_GAIN_M12DB=0x0c,
	KT0803L_PGA_GAIN_M13DB=0x0d,
	KT0803L_PGA_GAIN_M14DB=0x0e,
	KT0803L_PGA_GAIN_M15DB=0x0f
};

enum {
	KT0803L_RF_GAIN_95_5DBUV=0,
	KT0803L_RF_GAIN_96_5DBUV,
	KT0803L_RF_GAIN_97_5DBUV,
	KT0803L_RF_GAIN_98_2DBUV,
	KT0803L_RF_GAIN_98_9DBUV,
	KT0803L_RF_GAIN_100DBUV,
	KT0803L_RF_GAIN_101_5DBUV,
	KT0803L_RF_GAIN_102_8DBUV,
	KT0803L_RF_GAIN_105_1DBUV,
	KT0803L_RF_GAIN_105_6DBUV,
	KT0803L_RF_GAIN_106_2DBUV,
	KT0803L_RF_GAIN_106_5DBUV,
	KT0803L_RF_GAIN_107DBUV,
	KT0803L_RF_GAIN_107_4DBUV,
	KT0803L_RF_GAIN_107_7DBUV,
	KT0803L_RF_GAIN_108DBUV
};

enum {
	KT0803L_ALC_ATK_TIME_25US=0,
	KT0803L_ALC_ATK_TIME_50US,
	KT0803L_ALC_ATK_TIME_75US,
	KT0803L_ALC_ATK_TIME_100US,
	KT0803L_ALC_ATK_TIME_125US,
	KT0803L_ALC_ATK_TIME_150US,
	KT0803L_ALC_ATK_TIME_175US,
	KT0803L_ALC_ATK_TIME_200US,
	KT0803L_ALC_ATK_TIME_50MS,
	KT0803L_ALC_ATK_TIME_100MS,
	KT0803L_ALC_ATK_TIME_150MS,
	KT0803L_ALC_ATK_TIME_200MS,
	KT0803L_ALC_ATK_TIME_250MS,
	KT0803L_ALC_ATK_TIME_300MS,
	KT0803L_ALC_ATK_TIME_350MS,
	KT0803L_ALC_ATK_TIME_400MS
};

enum {
	KT0803L_ALC_DCY_TIME_25US=0,
	KT0803L_ALC_DCY_TIME_50US,
	KT0803L_ALC_DCY_TIME_75US,
	KT0803L_ALC_DCY_TIME_100US,
	KT0803L_ALC_DCY_TIME_125US,
	KT0803L_ALC_DCY_TIME_150US,
	KT0803L_ALC_DCY_TIME_175US,
	KT0803L_ALC_DCY_TIME_200US,
	KT0803L_ALC_DCY_TIME_50MS,
	KT0803L_ALC_DCY_TIME_100MS,
	KT0803L_ALC_DCY_TIME_150MS,
	KT0803L_ALC_DCY_TIME_200MS,
	KT0803L_ALC_DCY_TIME_250MS,
	KT0803L_ALC_DCY_TIME_300MS,
	KT0803L_ALC_DCY_TIME_350MS,
	KT0803L_ALC_DCY_TIME_400MS
};

enum {
	KT0803L_ALC_CMP_GAIN_6DB=0x04,
	KT0803L_ALC_CMP_GAIN_3DB=0x05,
	KT0803L_ALC_CMP_GAIN_0DB=0x06,
	KT0803L_ALC_CMP_GAIN_M3DB=0x07,
	KT0803L_ALC_CMP_GAIN_M6DB=0x00,
	KT0803L_ALC_CMP_GAIN_M9DB=0x01,
	KT0803L_ALC_CMP_GAIN_M12DB=0x02,
	KT0803L_ALC_CMP_GAIN_M15DB=0x03
};

enum {
	KT0803L_ALC_HOLD_TIME_50MS=0,
	KT0803L_ALC_HOLD_TIME_100MS,
	KT0803L_ALC_HOLD_TIME_150MS,
	KT0803L_ALC_HOLD_TIME_200MS,
	KT0803L_ALC_HOLD_TIME_1S,
	KT0803L_ALC_HOLD_TIME_5S,
	KT0803L_ALC_HOLD_TIME_10S,
	KT0803L_ALC_HOLD_TIME_15S
};

enum {
	KT0803L_ALC_LTH_0_25=0,
	KT0803L_ALC_LTH_0_2,
	KT0803L_ALC_LTH_0_15,
	KT0803L_ALC_LTH_0_1,
	KT0803L_ALC_LTH_0_05,
	KT0803L_ALC_LTH_0_03,
	KT0803L_ALC_LTH_0_02,
	KT0803L_ALC_LTH_0_01,
	KT0803L_ALC_LTH_0_005,
	KT0803L_ALC_LTH_0_001,
	KT0803L_ALC_LTH_0_0005,
	KT0803L_ALC_LTH_0_0001
};

enum {
	KT0803L_ALC_HTH_0_6=0,
	KT0803L_ALC_HTH_0_5,
	KT0803L_ALC_HTH_0_4,
	KT0803L_ALC_HTH_0_3,
	KT0803L_ALC_HTH_0_2,
	KT0803L_ALC_HTH_0_1,
	KT0803L_ALC_HTH_0_05,
	KT0803L_ALC_HTH_0_01
};

typedef struct {
	unsigned char useExtInductor;
	struct {
		unsigned char isUpToSW;
		unsigned char isXTAL;
		unsigned char freq;
	} clkSetting;
	unsigned char isPLTAmpHigh;
	unsigned char isPHTCNST50us;
	unsigned char isFDEV112_5KHZ;
	unsigned char isCHSELPAOff;
} KT0803L_SETTING;

typedef struct {
	unsigned char window;
	unsigned char lowThreshold;
	unsigned char highThreshold;
	unsigned char lowCount;
	unsigned char highCount;
} KT0803L_SD_SETTING;

typedef struct {
	unsigned char attackTime;
	unsigned char decayTime;
	unsigned char ALCCMPGain;
	unsigned char holdTime;
	unsigned char lowThreshold;
	unsigned char highThreshold;
} KT0803L_ALC_SETTING;

//protos //////////////////////////////////////////////////////////////////

void KT0803L_dump(void);
int KT0803L_setup(const KT0803L_SETTING *pSetting);

//int KT0803L_setFreq(unsigned short freq);
int KT0803L_setFreq(float f_freq);
int KT0803L_getFreq(unsigned short *pFreq);

//Output related
int KT0803L_setRFGain(unsigned char gain, unsigned char isPABias);
int KT0803L_getRFGain(unsigned char *pGain, unsigned char *pIsPABias);
int KT0803L_setAutoPADown(unsigned char flag);
int KT0803L_isAutoPADown(unsigned char *pFlag);
int KT0803L_PADown(unsigned char flag);
int KT0803L_isPADown(unsigned char *pFlag);
int KT0803L_mute(unsigned char flag);
int KT0803L_isMute(unsigned char *pFlag);
int KT0803L_setMono(unsigned char flag);
int KT0803L_isMono(unsigned char *pFlag);
int KT0803L_setBBLevel(unsigned char level);
int KT0803L_getBBLevel(unsigned char *pLevel);
int KT0803L_enableAUENHANCE(unsigned char flag);
int KT0803L_isAUENHANCEEnable(unsigned char *pFlag);

//Input related
int KT0803L_setALCSetting(const KT0803L_ALC_SETTING *pSetting);
int KT0803L_getALCSetting(KT0803L_ALC_SETTING *pSetting);
int KT0803L_enableALC(unsigned char flag);
int KT0803L_isALCEnable(unsigned char *pFlag);
int KT0803L_setPGAGain(unsigned char gain, unsigned char is1dBStep);
int KT0803L_getPGAGain(unsigned char *pGain, unsigned char *is1dBStep);
int KT0803L_setSDSetting(const KT0803L_SD_SETTING *pSetting);
int KT0803L_getSDSetting(KT0803L_SD_SETTING *pSetting);
int KT0803L_disableSD(unsigned char flag);
int KT0803L_isSDDisable(unsigned char *pFlag);

//Power related
int KT0803L_standby(unsigned char flag);
int KT0803L_isStandby(unsigned char *pFlag);

//Status related
int KT0803L_getStatus(unsigned char *pIsPwrOk, unsigned char *pIsSilence);

//===================== i2c =================================
//read single byte
int kt0803_readReg(unsigned char regAddr, unsigned char *pData) {

  stm_I2C_ReceiveBurstWithRestartCondition(KT0803L_ADDR8, regAddr, pData, 1);

  return(0); //success
}

int kt0803_writeReg(unsigned char regAddr, unsigned char data){
	unsigned char sendbuf[2];
	sendbuf[0] = regAddr;
	sendbuf[1] = data;
	  stm_I2C_SendBurst(KT0803L_ADDR8, sendbuf, 2);
}
//==========================================================
void KT0803L_dump(void) {
	unsigned char regs[]={
		KT0803L_REG_ADDR_00, 
		KT0803L_REG_ADDR_01, 
		KT0803L_REG_ADDR_02, 	
		KT0803L_REG_ADDR_04, 	
		KT0803L_REG_ADDR_0B, 	
		KT0803L_REG_ADDR_0C, 	
		KT0803L_REG_ADDR_0E, 	
		KT0803L_REG_ADDR_0F, 	
		KT0803L_REG_ADDR_10, 	
		KT0803L_REG_ADDR_12, 	
		KT0803L_REG_ADDR_13, 	
		KT0803L_REG_ADDR_14, 	
		KT0803L_REG_ADDR_15, 	
		KT0803L_REG_ADDR_16, 	
		KT0803L_REG_ADDR_17, 	
		KT0803L_REG_ADDR_1E, 	
		KT0803L_REG_ADDR_26, 	
		KT0803L_REG_ADDR_27
	};
	int i=0;
	
	for(i=0;i<sizeof(regs)/sizeof(regs[0]);i++) {
		unsigned char reg=0;

		kt0803_readReg(regs[i], &reg);
		printf("%02X: %02X\r\n", regs[i], reg);
	}
}

#if 0
//Frequency related
int KT0803L_setFreq(unsigned short freq) {
	KT0803L_REG_00 r00;
	KT0803L_REG_01 r01;
	KT0803L_REG_02 r02;
	
	freq*=2;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_00, &r00.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_01, &r01.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_02, &r02.b)<0)
		goto failed;
	
	r00.f.CHSEL_8_1=(unsigned char)((freq&0x01fe)>>1);
	r01.f.CHSEL_11_9=(unsigned char)((freq&0x0e00)>>9);
	r02.f.CHSEL_0=(unsigned char)(freq&0x0001);
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_00, r00.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_01, r01.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_02, r02.b)<0)
		goto failed;

	
	return ERR_OK;

failed:
	printf("Failed\r\n");
	return ERR_FAIL;
}
#else
//Frequency related
int KT0803L_setFreq(float f_freq) {
	KT0803L_REG_00 r00;
	KT0803L_REG_01 r01;
	KT0803L_REG_02 r02;
	unsigned int freq;

	freq = (unsigned int) (20.0 * f_freq);

	printf("freq=%u, f_freq=%s\r\n",freq, float2str(f_freq));

	if(kt0803_readReg(KT0803L_REG_ADDR_00, &r00.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_01, &r01.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_02, &r02.b)<0)
		goto failed;

	r00.f.CHSEL_8_1=(unsigned char)((freq&0x01fe)>>1);
	r01.f.CHSEL_11_9=(unsigned char)((freq&0x0e00)>>9);
	r02.f.CHSEL_0=(unsigned char)(freq&0x0001);

	if(kt0803_writeReg(KT0803L_REG_ADDR_00, r00.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_01, r01.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_02, r02.b)<0)
		goto failed;


	return ERR_OK;

failed:
	printf("Failed\r\n");
	return ERR_FAIL;
}
#endif
int KT0803L_getFreq(unsigned short *pFreq) {
	KT0803L_REG_00 r00;
	KT0803L_REG_01 r01;
	KT0803L_REG_02 r02;
	
	if(!pFreq)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_00, &r00.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_01, &r01.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_02, &r02.b)<0)
		goto failed;

	*pFreq=r02.f.CHSEL_0;
	*pFreq|=((unsigned short)r00.f.CHSEL_8_1)<<1;
	*pFreq|=((unsigned short)r01.f.CHSEL_11_9)<<9;
	*pFreq/=2;

	return ERR_OK;

failed:
	return ERR_FAIL;
}

//Output related
int KT0803L_setRFGain(unsigned char gain, unsigned char isPABias) {
	KT0803L_REG_01 r01;
	KT0803L_REG_02 r02;
	KT0803L_REG_13 r13;
	KT0803L_REG_0E r0E;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_01, &r01.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_02, &r02.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_13, &r13.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_0E, &r0E.b)<0)
		goto failed;

	r01.f.RFGAIN_1_0=gain&0x03;
	r02.f.RFGAIN_3=(gain&0x08)>>3;
	r13.f.RFGAIN_2=(gain&0x04)>>2;
	r0E.f.PA_BIAS=isPABias?1:0;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_01, r01.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_02, r02.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_13, r13.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_0E, r0E.b)<0)
		goto failed;

	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_getRFGain(unsigned char *pGain, unsigned char *pIsPABias) {
	KT0803L_REG_01 r01;
	KT0803L_REG_02 r02;
	KT0803L_REG_13 r13;
	KT0803L_REG_0E r0E;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_01, &r01.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_02, &r02.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_13, &r13.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_0E, &r0E.b)<0)
		goto failed;

	*pGain=r01.f.RFGAIN_1_0;
	*pGain|=r02.f.RFGAIN_3<<3;
	*pGain|=r13.f.RFGAIN_2<<2;
	*pIsPABias=r0E.f.PA_BIAS?TRUE:FALSE;

	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_setAutoPADown(unsigned char flag) {
	KT0803L_REG_0B r0B;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_0B, &r0B.b)<0)
		goto failed;
	
	r0B.f.AUTO_PADN=flag?1:0;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_0B, r0B.b)<0)
		goto failed;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_isAutoPADown(unsigned char *pFlag) {
	KT0803L_REG_0B r0B;
	
	if(!pFlag)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_0B, &r0B.b)<0)
		goto failed;
	
	*pFlag=r0B.f.AUTO_PADN?TRUE:FALSE;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_PADown(unsigned char flag) {
	KT0803L_REG_0B r0B;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_0B, &r0B.b)<0)
		goto failed;
	
	r0B.f.PDPA=flag?1:0;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_0B, r0B.b)<0)
		goto failed;
	
	return ERR_OK;

failed:
	printf("Failed\r\n");
	return ERR_FAIL;
}

int KT0803L_isPADown(unsigned char *pFlag) {
	KT0803L_REG_0B r0B;
	
	if(!pFlag)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_0B, &r0B.b)<0)
		goto failed;
	
	*pFlag=r0B.f.PDPA?TRUE:FALSE;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_mute(unsigned char flag) {
	KT0803L_REG_02 r02;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_02, &r02.b)<0)
		goto failed;
	
	r02.f.MUTE=flag?1:0;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_02, r02.b)<0)
		goto failed;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_isMute(unsigned char *pFlag) {
	KT0803L_REG_02 r02;
	
	if(!pFlag)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_02, &r02.b)<0)
		goto failed;
	
	*pFlag=r02.f.MUTE?TRUE:FALSE;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_setMono(unsigned char flag) {
	KT0803L_REG_04 r04;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_04, &r04.b)<0)
		goto failed;
	
	r04.f.MONO=flag?1:0;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_04, r04.b)<0)
		goto failed;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_isMono(unsigned char *pFlag) {
	KT0803L_REG_04 r04;
	
	if(!pFlag)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_04, &r04.b)<0)
		goto failed;
	
	*pFlag=r04.f.MONO?TRUE:FALSE;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_setBBLevel(unsigned char level) {
	KT0803L_REG_04 r04;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_04, &r04.b)<0)
		goto failed;
	
	r04.f.BASS_1_0=level;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_04, r04.b)<0)
		goto failed;

	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_getBBLevel(unsigned char *pLevel) {
	KT0803L_REG_04 r04;
	
	if(!pLevel)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_04, &r04.b)<0)
		goto failed;
	
	*pLevel=r04.f.BASS_1_0;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_enableAUENHANCE(unsigned char flag) {
	KT0803L_REG_17 r17;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_17, &r17.b)<0)
		goto failed;

	r17.f.AU_ENHANCE=flag?1:0;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_17, r17.b)<0)
		goto failed;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_isAUENHANCEEnable(unsigned char *pFlag) {
	KT0803L_REG_17 r17;
	
	if(!pFlag)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_17, &r17.b)<0)
		goto failed;

	*pFlag=r17.f.AU_ENHANCE?TRUE:FALSE;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

//Input related
int KT0803L_setALCSetting(const KT0803L_ALC_SETTING *pSetting) {
	KT0803L_REG_0C r0C;
	KT0803L_REG_15 r15;
	KT0803L_REG_26 r26;
	KT0803L_REG_27 r27;

	if(!pSetting)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_0C, &r0C.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_15, &r15.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_26, &r26.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_27, &r27.b)<0)
		goto failed;

	r0C.f.ALC_ATTACH_TIME_3_0=pSetting->attackTime;
	r0C.f.ALC_DECAY_TIME_3_0=pSetting->decayTime;
	r15.f.ALCCMPGAIN_2_0=pSetting->ALCCMPGain;
	r26.f.ALCHOLD_2_0=pSetting->holdTime;
	r27.f.ALCLOWTH_3_0=pSetting->lowThreshold;
	r26.f.ALCHIGHTH_2_0=pSetting->highThreshold;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_0C, r0C.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_15, r15.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_26, r26.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_27, r27.b)<0)
		goto failed;

	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_getALCSetting(KT0803L_ALC_SETTING *pSetting) {
	KT0803L_REG_0C r0C;
	KT0803L_REG_15 r15;
	KT0803L_REG_26 r26;
	KT0803L_REG_27 r27;
	
	if(!pSetting)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_0C, &r0C.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_15, &r15.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_26, &r26.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_27, &r27.b)<0)
		goto failed;

	pSetting->attackTime=r0C.f.ALC_ATTACH_TIME_3_0;
	pSetting->decayTime=r0C.f.ALC_DECAY_TIME_3_0;
	pSetting->ALCCMPGain=r15.f.ALCCMPGAIN_2_0;
	pSetting->holdTime=r26.f.ALCHOLD_2_0;
	pSetting->lowThreshold=r27.f.ALCLOWTH_3_0;
	pSetting->highThreshold=r26.f.ALCHIGHTH_2_0;

	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_enableALC(unsigned char flag) {
	KT0803L_REG_04 r04;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_04, &r04.b)<0)
		goto failed;
	
	r04.f.ALC_EN=flag?1:0;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_04, r04.b)<0)
		goto failed;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_isALCEnable(unsigned char *pFlag) {
	KT0803L_REG_04 r04;
	
	if(!pFlag)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_04, &r04.b)<0)
		goto failed;
	
	*pFlag=r04.f.ALC_EN?TRUE:FALSE;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_setPGAGain(unsigned char gain, unsigned char is1dBStep) {
	KT0803L_REG_01 r01;
	KT0803L_REG_04 r04;
	KT0803L_REG_10 r10;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_01, &r01.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_04, &r04.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_10, &r10.b)<0)
		goto failed;
	
	r01.f.PGA_2_0=(gain&0x1c)>>2;
	r04.f.PGA_LSB_1_0=gain&0x3;
	r10.f.PGAMOD=is1dBStep?1:0;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_01, r01.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_04, r04.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_10, r10.b)<0)
		goto failed;

	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_getPGAGain(unsigned char *pGain, unsigned char *pIs1dBStep) {
	KT0803L_REG_01 r01;
	KT0803L_REG_04 r04;
	KT0803L_REG_10 r10;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_01, &r01.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_04, &r04.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_10, &r10.b)<0)
		goto failed;

	*pGain=r04.f.PGA_LSB_1_0;
	*pGain|=r01.f.PGA_2_0<<2;
	*pIs1dBStep=r10.f.PGAMOD?TRUE:FALSE;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_setSDSetting(const KT0803L_SD_SETTING *pSetting) {
	KT0803L_REG_12 r12;
	KT0803L_REG_14 r14;
	KT0803L_REG_16 r16;
	
	if(!pSetting)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_12, &r12.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_14, &r14.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_16, &r16.b)<0)
		goto failed;

	r14.f.SLNCTIME_2_0=pSetting->window&0x07;
	r14.f.SLNCTIME_3=(pSetting->window&0x08)>>3;
	r12.f.SLNCTHL_2_0=pSetting->lowThreshold;
	r12.f.SLNCTHH_2_0=pSetting->highThreshold;
	r16.f.SLNCCNTLOW_2_0=pSetting->lowCount;
	r14.f.SLNCCNTHIGH_2_0=pSetting->highCount;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_12, r12.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_14, r14.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_16, r16.b)<0)
		goto failed;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_getSDSetting(KT0803L_SD_SETTING *pSetting) {
	KT0803L_REG_12 r12;
	KT0803L_REG_14 r14;
	KT0803L_REG_16 r16;
	
	if(!pSetting)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_12, &r12.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_14, &r14.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_16, &r16.b)<0)
		goto failed;

	memset(pSetting, 0, sizeof(KT0803L_SD_SETTING));
	pSetting->window=r14.f.SLNCTIME_2_0;
	pSetting->window|=r14.f.SLNCTIME_3<<3;
	pSetting->lowThreshold=r12.f.SLNCTHL_2_0;
	pSetting->highThreshold=r12.f.SLNCTHH_2_0;
	pSetting->lowCount=r16.f.SLNCCNTLOW_2_0;
	pSetting->highCount=r14.f.SLNCCNTHIGH_2_0;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_disableSD(unsigned char flag) {
	KT0803L_REG_12 r12;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_12, &r12.b)<0)
		goto failed;

	r12.f.SLNCDIS=flag?1:0;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_12, r12.b)<0)
		goto failed;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_isSDDisable(unsigned char *pFlag) {
	KT0803L_REG_12 r12;
	
	if(!pFlag)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_12, &r12.b)<0)
		goto failed;

	*pFlag=r12.f.SLNCDIS?TRUE:FALSE;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

//Power related
int KT0803L_standby(unsigned char flag) {
	KT0803L_REG_0B r0B;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_0B, &r0B.b)<0)
		goto failed;

	r0B.f.STANDBY=flag?1:0;
	
	if(kt0803_writeReg(KT0803L_REG_ADDR_0B, r0B.b)<0)
		goto failed;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_isStandby(unsigned char *pFlag) {
	KT0803L_REG_0B r0B;
	
	if(!pFlag)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_0B, &r0B.b)<0)
		goto failed;

	*pFlag=r0B.f.STANDBY?TRUE:FALSE;
	
	return ERR_OK;

failed:
	return ERR_FAIL;
}

//Status related
int KT0803L_getStatus(unsigned char *pIsPwrOk, unsigned char *pIsSilence) {
	KT0803L_REG_0F r0F;
	
	if(!pIsPwrOk || !pIsSilence)
		goto failed;
	
	if(kt0803_readReg(KT0803L_REG_ADDR_0F, &r0F.b)<0)
		goto failed;

	*pIsPwrOk=r0F.f.PW_OK?TRUE:FALSE;
	*pIsSilence=r0F.f.SLNCID?TRUE:FALSE;

	return ERR_OK;

failed:
	return ERR_FAIL;
}

int KT0803L_setup(const KT0803L_SETTING *pSetting)
{
	KT0803L_REG_0F r0F;
	KT0803L_REG_13 r13;
	KT0803L_REG_1E r1E;
	KT0803L_REG_17 r17;
	KT0803L_REG_02 r02;
	KT0803L_REG_12 r12;
	unsigned int now=0, prev=0;

	if(!pSetting)
		goto failed;

	now = prev = millis();
	while(1) {
		if(kt0803_readReg(KT0803L_REG_ADDR_0F, &r0F.b)<0)
			goto failed;

		if(r0F.f.PW_OK)
			break;

		printf("KT0803L is not ready yet.\r\n");

		delayms(500);
		now=millis();
		if(now - prev >= 1500) //over 1.5sec
			goto failed;
	}
	printf("r0F.f.PW_OK=0x%X.\r\n", r0F.f.PW_OK);

	if(kt0803_readReg(KT0803L_REG_ADDR_13, &r13.b)<0)
		goto failed;

	//PA structure selection
	r13.f.PA_CTRL=pSetting->useExtInductor?1:0;
	printf("r13.f.PA_CTRL=0x%X.\r\n", r13.f.PA_CTRL);

	if(kt0803_writeReg(KT0803L_REG_ADDR_13, r13.b)<0)
		goto failed;

	//Read registers
	if(kt0803_readReg(KT0803L_REG_ADDR_1E, &r1E.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_17, &r17.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_02, &r02.b)<0)
		goto failed;
	if(kt0803_readReg(KT0803L_REG_ADDR_12, &r12.b)<0)
		goto failed;

	//Clock selection
	if(!pSetting->clkSetting.isUpToSW) {
		printf("Clock setting is up to software.\r\n");
		r1E.f.DCLK=1;

		r1E.f.REF_CLK_3_0=pSetting->clkSetting.freq;
		printf("r1E.f.REF_CLK_3_0=0x%X.\r\n", r1E.f.REF_CLK_3_0);
		if(pSetting->clkSetting.freq==KT0803L_CLK_FREQ_32_768KHZ) {
			printf("Clock frequency = 32.768KHz.\r\n");
			r17.f.XTAL_SEL=0;
		} else {
			printf("Clock frequency != 32.768KHz.\r\n");
			r17.f.XTAL_SEL=1;
		}

		if(!pSetting->clkSetting.isXTAL) {
			printf("Use reference clock.\r\n");
			r1E.f.XTALD=1;
		} else {
			printf("Use XTAL as clock.\r\n");
			r1E.f.XTALD=0;
		}
	} else {
		printf("Clock setting is fixed by SW1/SW2 pin(1,0)=32.768KHz, Powered ON.\r\n");
		r1E.f.DCLK=0;
	}

	//Pilot tone amplitude adjustment
	r02.f.PLTADJ=pSetting->isPLTAmpHigh?1:0;
	printf("r02.f.PLTADJ(Pilot tone amplitude adjustment)=0x%X.\r\n", r02.f.PLTADJ);

	//Pre-emphasis time constant set
	r02.f.PHTCNST=pSetting->isPHTCNST50us?1:0;
	printf("r02.f.PHTCNST(Pre-emphasis time constant)=0x%X.\r\n", r02.f.PHTCNST);

	//Frequency deviation detection
	r17.f.FDEV=pSetting->isFDEV112_5KHZ?1:0;
	printf("r17.f.FDEV(Frequency deviation detection)=0x%X.\r\n", r17.f.FDEV);

	//Channel switching behavior selection
	r12.f.SW_MOD=pSetting->isCHSELPAOff?1:0;
	printf("r12.f.SW_MOD(Channel switching behavior)=0x%X.\r\n", r12.f.SW_MOD);
	
	//Write registers
	if(kt0803_writeReg(KT0803L_REG_ADDR_1E, r1E.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_17, r17.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_02, r02.b)<0)
		goto failed;
	if(kt0803_writeReg(KT0803L_REG_ADDR_12, r12.b)<0)
		goto failed;

	return ERR_OK;

failed:
	printf("Failed\r\n");
	return ERR_FAIL;
}
//==== mainloop =====================
void KT0803L_FMstation_loop() {

	KT0803L_SETTING ktFMstation;

	//I2c config
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

	printf("kt0803L FM Transmitter @ 0x7C\r\n");

	KT0803L_setup(&ktFMstation);

	KT0803L_setFreq(87.5);
	KT0803L_mute(0);
	//KT0803L_setPGAGain(KT0803L_PGA_GAIN_8DB,0);//unsigned char gain, unsigned char is1dBStep)

	while(1)
	{

	}
}
