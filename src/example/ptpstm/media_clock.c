#include <stdio.h>
#include <string.h>
#include "ethopts.h"
#include "yInc.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "misc.h"
//#include "lwipopts.h"
#include "lwip/include/lwipopts.h"
//#include "ptpstm/include/datatypes.h"


#if (PROJ_FOR == PROJ_FOR_PTP)
//========================= YOON ==========================================================================
extern void PTP_SyncLed(unsigned char on_off);
extern unsigned char g_locked;
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);

// Ratio Register = (12.288MHz / 1024Hz) << 12 = 12000 * 4096 =  49152000 = 0x02ee0000
#define MCLKFREQ 12288000
#define REFCLK   1024
#define CS2300_I2CADDR7 0x4e //7 bit addr
#define CS2300_I2CADDR8 0x9c //8 bit addr
#define CS2300_PLL_AUX_OUT_SRC0 0x03 //PLL Lock Status Indicator
struct _cs2300configData{
	unsigned char addr;
	unsigned char data;
};

/* High Multi(20.12 Format) : example: 12.288MHz/60Hz = 204,800. Scaled = 0x3200 0000.
 * High Resol(12.20 Format) : example: 12.288MHz/10MHz = 1.2288. --> <<20. --> Scaled = 128490. (=0x0013 a92a)
 */

struct _cs2300configData cs2300configData[9] ={
		{0x02, 0x00}, // Output Enable
		{0x09, 0x00}, //00  LSB of Ratio Register
		{0x08, 0x00},  //00
		{0x07, 0xee}, //ee
		{0x06, 0x02},// 02 MSB of Ratio Register
		{0x17, 0x00}, //FuncConfigReg2(RRR-ClkOutUnlock(0:ClkOut=low if PLL is unlocked)-LFRatioCfg(0=HighMultiplier;1=HighResol)-RRR
		{0x16, 0x10},//Function Config Reg. 1 (ClockSkipEn-AuxLockCfg-R-EnDevCfg3-R-R-R-R => 0(lock disabled if no CLK_IN)-0(PushPull:acive High. 0 for locked condition)-1(enableDevice Cfg Reg3)-RRRR
		{0x05, 0x01}, //Global Cfg (R-R-R-R-Freeze-R-R-Enable Device Cfg Reg.2) => 0x01
		{0x03, 0x07}  //Device Cfg 1.(RModSel[2:0]-R-R-AuxOutSrc[1:0]-EnDeviceCfg1) ; 000(=x1)-0-0-11(PLL Lock Status Indicator)-1(EnDeviceCfg1)
		//,0x1e 		//Function Config Reg.3 (Clock Input BW config. R-BW[2:0]-RRRR)(1~128Hz)
};
/*
//High Resol(12.20 Format) : example: 12.288MHz/10MHz = 1.2288. --> <<20. --> Scaled = 128490. (=0x0013 a92a)
struct _cs2300configData cs2300configData[9] ={
		{0x02, 0x00}, // Output Enable
		{0x09, 0x2a}, //00  LSB of Ratio Register
		{0x08, 0xa9},  //00
		{0x07, 0x13}, //ee
		{0x06, 0x00},// 02 MSB of Ratio Register
		{0x17, 0x08}, //FuncConfigReg2(RRR-ClkOutUnlock(0:ClkOut=low if PLL is unlocked)-LFRatioCfg(0=HighMultiplier;1=HighResol)-RRR
		{0x16, 0x10},//Function Config Reg. 1 (ClockSkipEn-AuxLockCfg-R-EnDevCfg3-R-R-R-R => 0(lock disabled if no CLK_IN)-0(PushPull:acive High. 0 for locked condition)-1(enableDevice Cfg Reg3)-RRRR
		{0x05, 0x01}, //Global Cfg (R-R-R-R-Freeze-R-R-Enable Device Cfg Reg.2) => 0x01
		{0x03, 0x07}  //Device Cfg 1.(RModSel[2:0]-R-R-AuxOutSrc[1:0]-EnDeviceCfg1) ; 000(=x1)-0-0-11(PLL Lock Status Indicator)-1(EnDeviceCfg1)
		//,0x1e 		//Function Config Reg.3 (Clock Input BW config. R-BW[2:0]-RRRR)(1~128Hz)
};
*/
unsigned char cs2300_sendbuf[2];
unsigned char cs2300_recvbuf[2];

void cs2300Config(){

	int i;
	unsigned char rev;


	stm_I2C_ReceiveBurstWithRestartCondition(CS2300_I2CADDR8, 0x01, &cs2300_recvbuf[0], 1); //Device ID and Revision Register at 0x01
	rev = cs2300_recvbuf[0] & 0x07;
	if(rev = 0x6)
		printf("\r\n cs2300 ID=%x,Rev=C1(6)\r\n", (cs2300_recvbuf[0] & 0xf8) >> 3, rev);
	else if(rev = 0x4)
		printf("\r\n cs2300 ID=%x,Rev=B2/B3(4)\r\n", (cs2300_recvbuf[0] & 0xf8) >> 3, rev);
	else
		printf("\r\n cs2300 ID=%x,Rev=0x ??????????\r\n", (cs2300_recvbuf[0] & 0xf8) >> 3, cs2300_recvbuf[0] & 0x07);


	//uint32_t mult; //= 0x2ee00000
	//set user defined ratio
	//mult  = (MCLKFREQ/REFCLK) << 12;
	//for(i=8;i>=0; i--){
	for(i=8;i>=0; i--){
		cs2300_sendbuf[0] = cs2300configData[i].addr;
		cs2300_sendbuf[1] = cs2300configData[i].data;
		printf("cs2300([%d]reg=%02x<-val(%02x)\r\n", i,cs2300_sendbuf[0],cs2300_sendbuf[1]);
		//stm_I2C_SendBurst(CS2300_I2CADDR8,(unsigned char *)(&cs2300configData[i]),2);//&cs2300_sendbuf[0], 2);
		stm_I2C_SendBurst(CS2300_I2CADDR8,&cs2300_sendbuf[0], 2);
		delayms(10);
	}
	printf("cs2300 PLL config Done(12.288MHz from 1024Hz).\r\n");

	delayms(1000);
	//check by reading
	for(i=8;i>=0; i--){
		stm_I2C_ReceiveBurstWithRestartCondition(CS2300_I2CADDR8, cs2300configData[i].addr, &cs2300_recvbuf[0], 1); //Device ID and Revision Register at 0x01
		rev = cs2300_recvbuf[0];
		printf("\r\n[%d] cs2300 Reg0x%02x = 0x%02x\r\n",i, cs2300configData[i].addr, rev);
	}
}
#endif
