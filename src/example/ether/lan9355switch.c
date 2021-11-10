/*[REF] It is a similar to KSZ8463 Switch.
  LAN9355 Switch
  I2C 8Bit Address = 0x0A.
  Author: YOON
  * Refer to LAN9303 Linux Driver in github
  */
#include "yLib/include/yInc.h"
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
//#include "lwip/include/lwipopts.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "yLib/inet/ipconf.h"
#include "yLib/eth/include/ethopts.h"

#if	(SWITCH_ID == SWITCH_LAN9355)
#include "ethers/include/LAN9355.h"

//I2C
#define OLED_ADDRESS					0x78 //0x3C = 7bit address --> 0x78 (8 bit addr)
#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz
#define LAN9355_ADDRESS8				0x14 //0x0A(000 1010) 7bit address --> 0x14(8 bit addr)

//#include "datatypes.h"

//For LAN9355, We disable PPS Function of STM32F407. The PIN PB5 will be used for PPS output with GPIO OUTPUT which is driven by LAN9355's 1588 Event Trigger.

// READ/WRITE SEQUENCE
// [ADDR][R/W] - [RegAddr(A9~A2)] - [DATA][DATA][DATA][DATA]
//
//NOTE
//i) To check I2C Slave functions,we must poll BYTE_TEST
//ii) To check I2C Slave full configuration,we must poll READY bit of HW_CFG reg.
//(iii) READ: [ADDR][W(0)] - [RegAddr(A9~A2)] -[ADDR][R(1)] - [DATA][DATA][DATA][DATA]
//(iv) WRITE: [ADDR][W(0)] - [RegAddr(A9~A2)] -[DATA][DATA][DATA][DATA]
//==============================
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
//SlaveAddr(W)| Reg | SetSlaveAddr(R) | BurstReceiveStart for Repeated Start | GetData |

unsigned long LAN9355_get_INT_STS(void);

extern uint32_t g_tickEvent;
extern unsigned long millis();

//volatile u8 regbuf[80];
u8 regbuf[80];
void Lan9355_init();
void Lan9355_Config();




struct _Lan9355;

struct Lan9355_phy_ops {
	/* PHY 1 and 2 access*/
	int	(*phy_read)(int port, int regnum);
	int	(*phy_write)(int port,int regnum, u16 val);
};

#define LAN9355_NUM_ALR_RECORDS 512
struct Lan9355_alr_cache_entry {
	u8  mac_addr[6];//ETH_ALEN];
	u8  port_map;         /* Bitmap of ports. Zero if unused entry */
	u8  stp_override;     /* non zero if set LAN9303_ALR_DAT1_AGE_OVERRID */
};

struct _Lan9355 {
	//struct device *dev;
	//struct regmap *regmap;
	//struct regmap_irq_chip_data *irq_data;
	//struct gpio_desc *reset_gpio;

	unsigned short io_timeout;
    bool did_timeout;
    unsigned short timeout_start_ms;

	u32 reset_duration; /* in [ms] */
	int phy_addr_base;
	//struct dsa_switch *ds;
	//const struct lan9355_phy_ops *ops;
	bool is_bridged; /* true if port 1 and 2 are bridged */

	/* remember LAN9303_SWE_PORT_STATE while not bridged */
	u32 swe_port_state;

	// LAN9303 do not offer reading specific ALR entry. Cache all static entries in a flat table
	struct Lan9355_alr_cache_entry alr_cache[LAN9355_NUM_ALR_RECORDS];
};

struct _Lan9355 g_Lan9355;

struct LAN9355_1588_REG LAN9355_1588_REGS[]={
		{i1588_CMD_CTL,0xf,"1588_CMD_CTL(0x100)"},
		{i1588_GENERAL_CONFIG,0xf,"1588_GENERAL_CONFIG(0x104)"},
		{i1588_INT_STS,0xf,"1588_INT_STS(0x108)"},
		{i1588_INT_EN,0xf,"1588_INT_EN(0x10C)"},
		{i1588_CLOCK_SEC,0xf,"1588_CLOCK_SEC(0x110)"},
		{i1588_CLOCK_NS,0xf,"1588_CLOCK_NS(0x114)"},
		{i1588_CLOCK_SUBNS,0xf,"1588_CLOCK_SUBNS(0x118)"},
		{i1588_CLOCK_RATE_ADJ,0xf,"1588_CLOCK_RATE_ADJ(0x11C)"},
		{i1588_CLOCK_STEP_ADJ,0xf,"1588_CLOCK_STEP_ADJ(0x128)"},
		{i1588_LATENCY_X,0,"1588_LATENCY_X(0x158)"},//BANK0
		{i1588_ASYM_PEERDLY_X,0,"1588_ASYM_PEERDLY_X(0x15C)"},				//BANK0
		{i1588_CAP_INFO_X,0,"1588_CAP_INFO_X(0x160)"},						//BANK0
		{i1588_RX_PARSE_CONFIG_X,1,"1588_RX_PARSE_CONFIG_X(0x158)"}, 		//BANK1
		{i1588_RX_TIMESTAMP_CONFIG_X,1,"1588_RX_TIMESTAMP_CONFIG_X(0x15C)"},	//BANK1
		{i1588_RX_TS_INSERT_CONFIG_X,1, "1588_RX_TS_INSERT_CONFIG_X(0x160)"}	//BANK1
};


void EthIRQ_PA3_setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  /* Enable GPIOE clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Configure PA3 pin as input floating */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Connect EXTI Line3 to PA3 pin */

	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);

	  /* Configure EXTI Line3 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}

void EXTI3_IRQHandler(){
	unsigned long sts;

	 if(EXTI_GetITStatus(EXTI_Line3) != RESET){
		 EXTI_ClearITPendingBit(EXTI_Line3); //Clear this IRQ.
		 sts = LAN9355_get_INT_STS();

		 printf("\r\n mainIRQ=%x\r\n",sts);
		 //LAN9355_1588_getINT_STS();
	 }
}

//1588_BANK_PORT_GPIO_SEL (0x154)
void Lan9355_setBankAndPort(unsigned short reg,unsigned long port){
	char regbuf[10];
	unsigned char bank;
	unsigned long dval_port_bank = 0;
	int ret=0;

	bank = GetRegBank(reg);
	if(bank != 0xf){ //set bank and port
		dval_port_bank = (port<<4) | bank; //(PORT_SEL[5:4] and BANK_SEL[2:0]
		regbuf[0]= GetRegAddr(i1588_BANK_PORT_GPIO_SEL);//reg);
		regbuf[1]= (dval_port_bank >>24) & 0xff;
		regbuf[2]= (dval_port_bank >>16) & 0xff;
		regbuf[3]= (dval_port_bank >>8) & 0xff;
		regbuf[4]= dval_port_bank & 0xff;

		ret = stm_I2C_SendBurst(LAN9355_ADDRESS8, regbuf, 5);
	}
	return;
}

//See 5.0 Register MAP -- General Register Map
//Direct Addressable Registers: System Control and Status Registers
//InDirect Addressable Registers: PHY Registers, Switch Fabric Control and Status Registers.
//  - PHY Registers : Use PMI_ACCESS(0x0A8) and PMI_DATA(0x0A4) -- See.14.3 and Table 14-5.(page 389)
//  - with Fabric Control and Status Registers:
//Detailed Register ID : See Table 5.1
unsigned long Lan9355_getReg(unsigned short reg,unsigned long port){
	unsigned char regbuf[10];
	char str[10];
	unsigned long dval = 0;
	unsigned char bank;

	Lan9355_setBankAndPort(reg,port);

	stm_I2C_ReceiveBurstWithRestartCondition(LAN9355_ADDRESS8, GetRegAddr(reg), &regbuf[0], 4);
	dval = regbuf[0]*0x1000000 + regbuf[1]*0x10000 + regbuf[2]*0x100 + regbuf[3];
	//sprintf(str,"%03x>%08x",reg,dval);
	//stmOzOLED_printString(str,0,3,12);
	//printf("%03x(%u:%u)>0x%08x\r\n",reg,port,bank,dval);
	return dval;
}
void Lan9355_setReg(unsigned short reg,unsigned long port, unsigned long dval){
	char regbuf[10];
	int ret=0;
	unsigned char bank;
	unsigned long dval_port_bank = 0;

	Lan9355_setBankAndPort(reg,port);

	regbuf[0]= GetRegAddr(reg);
	regbuf[1]= dval >>24;
	regbuf[2]= (dval >>16)& 0xff;
	regbuf[3]= (dval >>8)& 0xff;
	regbuf[4]= dval & 0xff;

	ret = stm_I2C_SendBurst(LAN9355_ADDRESS8, regbuf, 5);
	return;

}
//============
/*
static int lan9355_read(struct regmap *regmap, unsigned int offset, u32 *reg)
{
	int ret, i;

	for (i = 0; i < 5; i++) {
		ret = regmap_read(regmap, offset, reg);
		if (!ret)
			return 0;
		if (ret != -EAGAIN)
			break;
		msleep(500);
	}

	return -EIO;
}
*/
static int Lan9355_read_wait(int reg, u32 mask)
{
	int i;

	for (i = 0; i < 25; i++) {
		u32 val;
		int ret;

		val = Lan9355_getReg(reg, PORT_NONE);
		if (ret) {
			//dev_err(chip->dev, "%s failed to read offset %d: %d\n",	__func__, offset, ret);
			return ret;
		}
		if (!(val & mask))
			return 0;
		delayms(2);//usleep_range(1000, 2000);
	}

	return -1;
}
static int Lan9355_switch_wait_for_completion(void)
{
	return Lan9355_read_wait(LAN9355_SWITCH_CSR_CMD, LAN9355_SWITCH_CSR_CMD_BUSY);
}
static unsigned long Lan9355_getRegIndirect(u16 regnum)
{
	u32 reg, val;
	int ret;

	reg = regnum;
	reg |= LAN9355_SWITCH_CSR_CMD_LANES;
	reg |= LAN9355_SWITCH_CSR_CMD_RW;
	reg |= LAN9355_SWITCH_CSR_CMD_BUSY;

	//Lan9355_switch_wait_for_completion();
	Lan9355_setReg(LAN9355_SWITCH_CSR_CMD,PORT_NONE, reg);
	//Lan9355_read_wait(LAN9355_SWITCH_CSR_CMD, LAN9355_SWITCH_CSR_CMD_BUSY);
	delayms(10);
	val = Lan9355_getReg(LAN9355_SWITCH_CSR_DATA,  PORT_NONE);
	delayms(10);
	return val;
}

void Lan9355_setRegIndirect(u16 regnum, u32 val)
{
/*	u32 reg, val;
	int ret;

	reg = regnum;
	reg |= LAN9355_SWITCH_CSR_CMD_LANES;
	reg |= LAN9355_SWITCH_CSR_CMD_RW;
	reg |= LAN9355_SWITCH_CSR_CMD_BUSY;

	//Lan9355_switch_wait_for_completion();
	Lan9355_setReg(LAN9355_SWITCH_CSR_CMD,PORT_NONE, reg);
	//Lan9355_read_wait(LAN9355_SWITCH_CSR_CMD, LAN9355_SWITCH_CSR_CMD_BUSY);
	delayms(10);
	val = Lan9355_getReg(LAN9355_SWITCH_CSR_DATA,  PORT_NONE);
	delayms(10);
	return val;
*/
}


//=========================


//================================================================

void LAN9355_GetChipIdRev(void){
	unsigned long dval = 0;

	dval = Lan9355_getReg(LAN9355_ID_REV, PORT_NONE);
	printf("\r\nLAN9355 ChipID and REV=(%04x : %04x)\r\n",dval >>16, dval & 0xffff);
}

void LAN9355_WaitForDeviceReady(void){
	unsigned long dval = 0;
	while(1){
		dval = Lan9355_getReg(LAN9355_PMT_CTRL, PORT_NONE); //see pp.67-69
		if(dval & 0x00000001){
			printf("\r\nLAN9355 becomes Ready\r\n",dval >>16, dval & 0xffff);
			break;
		}
	}
}
//See 8.3 -- The final master IRQ enable with Active Low and Push-Pull
void LAN9355_IRQ_Config(void){
	unsigned long dval = 0;

	dval = 0x20000000; //1588_EVNT_EN
	Lan9355_setReg(LAN9355_INT_EN, PORT_NONE, dval); //set READ | 1588_ENABLE

	dval = 0x00000101; //IRQ_EN | Push-Pull
	Lan9355_setReg(LAN9355_IRQ_CFG, PORT_NONE, dval); //set READ | 1588_ENABLE
}

unsigned long LAN9355_get_INT_STS(void){
	long dval = 0;

	dval = Lan9355_getReg(LAN9355_INT_STS, PORT_NONE); //get global IRQ status.
	printf("9355_INT_STS=%08x\r\n",dval);
	return dval;
}
//============= GPIO ===================
unsigned long LAN9355_get_GPIO_CFG(void){
	long dval = 0;

	dval = Lan9355_getReg(LAN9355_GPIO_CFG, PORT_NONE);
	printf("LAN9355_GPIO_CFG=%08x\r\n",dval);
	return dval;
}
void LAN9355_set_GPIO_CFG(unsigned int dval){
	Lan9355_setReg(LAN9355_GPIO_CFG, PORT_NONE, dval);
}

void LAN9355_set_GPIO_INT_STS_EN(unsigned int dval){
	Lan9355_setReg(LAN9355_GPIO_INT_STS_EN, PORT_NONE, dval);
}
unsigned long LAN9355_get_GPIO_INT_STS_EN(void){
	long dval = 0;

	dval = Lan9355_getReg(LAN9355_GPIO_INT_STS_EN, PORT_NONE);
	printf("LAN9355_GPIO_INT_STS_EN=%08x\r\n",dval);
	return dval;
}
void LAN9355_set_GPIO_DATA_DIR(unsigned int dval){
	Lan9355_setReg(LAN9355_GPIO_DATA_DIR, PORT_NONE, dval);
}
unsigned long LAN9355_get_GPIO_DATA_DIR(void){
	long dval = 0;

	dval = Lan9355_getReg(LAN9355_GPIO_DATA_DIR, PORT_NONE);
	printf("LAN9355_GPIO_DATA_DIR=%08x\r\n",dval);
	return dval;
}


//=============================IEEE1588==========================================
void LAN9355_1588_INT_EN(void){
	long dval = 0;

	dval = 0x00007700; //1588_TX_TS_EN[2:0] and 1588_RX_TS_EN[2:0]
	Lan9355_setReg(i1588_INT_EN, PORT_NONE, dval); //set READ | 1588_ENABLE
}

void LAN9355_1588_getINT_STS(void){ //and Clear it
	unsigned long dval = 0;

	dval = Lan9355_getReg(i1588_INT_STS, PORT_NONE); //get 1588_TX_TS_EN[2:0] and 1588_RX_TS_EN[2:0]
	printf("1588_INT_STS=%x\r\n",dval);
	Lan9355_setReg(i1588_INT_STS, PORT_NONE, dval); //clear with WC
}
/*
void LAN9355_1588_setINT_STS(void){ //for Clear Interrupt with writing 1
	unsigned long dval = 0;

	dval = Lan9355_getReg(i1588_INT_STS, PORT_NONE); //get 1588_TX_TS_EN[2:0] and 1588_RX_TS_EN[2:0]
	Lan9355_setReg(i1588_INT_STS, PORT_NONE, dval); //clear with WC
}
*/


//MsgIDs
// enum PTP1588MsgID = {Sync=0, Delay_Req, PDelay_Req, PDelay_Resp=3, FollowUp=8,Delay_Resp,PDelay_Resp_FollowUp, Announce=11, PTPmgmt=12};
//1588 related registers
//Each port is selected by the bankselect register.
//0x158 : 1588 PORT x Latency Register : Port0: 0x0014 0014 (TX latency=20nsc, Rx latency=20nsec. Fixd for 100Mbps MII) -- Latency between Medium and PTP timestamp point.
//                                       Port1/2: 0x005f 011d (TX latency=95nsc, Rx latency=285nsec.Fixd for 100Mbps MII) -- Latency between Medium and PTP timestamp point.

void LAN9355_Enable_1588(void){
	long dval = 0;

	//clock read enable
	dval = 0x0000000c ; //1100 (1588_CLOCK_READ | 1588_ENABLE)
	Lan9355_setReg(i1588_CMD_CTL, PORT_NONE, dval); //set READ | 1588_ENABLE
}
//set 30 bit rate adj register.
//min(43):1ns/42.94967296sec; max(0x3fffffff):1ns/40nsec.
void LAN9355_ClockRateAdj(unsigned char slower_faster, unsigned int clockrateadjValue){
	unsigned int dval = 0;

	if(slower_faster == 1)
		dval = 0x80000000; //Faster
	else
		dval = 0x00000000; //Slower

	dval |= clockrateadjValue;

	Lan9355_setReg(i1588_CLOCK_RATE_ADJ, PORT_NONE, dval);
}

void LAN9355_ClockRateAdj_10nsecPerSec(unsigned char slower_faster){
	unsigned int dval = 0;
	 unsigned int clockrateadjValue=429;//~430

	if(slower_faster == 1)
		dval = 0x80000000; //Faster
	else
		dval = 0x00000000; //Slower

	dval |= clockrateadjValue;

	Lan9355_setReg(i1588_CLOCK_RATE_ADJ, PORT_NONE, dval);

	//for check
	dval = Lan9355_getReg(i1588_CLOCK_RATE_ADJ, PORT_NONE); //Check
	printf("ClockRateAdjReg=%08x\r\n", dval);
}

//(15.8.11) -- Only lower 4 bits value will be affected.
void LAN9355_ClockStepSec(unsigned char sub0_add1, unsigned int clockStepSec){
	unsigned int dval = 0;

	if(sub0_add1 == 1)
		dval = 0x80000000; //Add
	else
		dval = 0x00000000; //Sub

	Lan9355_setReg(i1588_CLOCK_STEP_ADJ, PORT_NONE, dval | clockStepSec);
	dval = 0x00000020; //---10 0000b (1588_CLOCK_STEP_SECONDS)
	Lan9355_setReg(i1588_CMD_CTL, PORT_NONE,dval);//enable step sec.
}

void Lan9355_Enable_Rx_CorrectionField(){
	int i;
	char str[10];
	long dval = 0;
	unsigned long port=0;
	unsigned short reg;
	unsigned long bank=0;
	unsigned long sec, ns, subns;

	//BANK1
	//correction field enable per port
	for(port=0;port<3;port++){
		//dval = 0x00000f0f; //{11,10,9,8,3,2,1,0}
		//dval = 0x0000040c; //{a,3,2}
		dval = 0x0000ffff;  //SYNC(0), DELAY_REQ(1), PDELAY)REQ(2), PDELAY_RESP(3)
		//Lan9355_setReg(i1588_RX_CF_MOD_X, port, dval); //set specified msg for inserting correction field.
		Lan9355_setReg(i1588_RX_TIMESTAMP_CONFIG_X, port, dval); //SET All Msg
	}
}
void Lan9355_Enable_Tx_CorrectionField(){
	int i;
	char str[10];
	long dval = 0;
	unsigned long port=0;
	unsigned short reg;
	unsigned long bank=0;
	unsigned long sec, ns, subns;

	//BANK2
	//correction field enable per port
	for(port=0;port<3;port++){
		//dval = 0x00000f0f; //{11,10,9,8,3,2,1,0}
		dval = 0x0000040c; //{a,3,2}
		dval = 0x00000000;  //SYNC(0), DELAY_REQ(1), PDELAY)REQ(2), PDELAY_RESP(3)
		Lan9355_setReg(i1588_TX_CF_MOD_X, port, dval); //set specified msg for inserting correction field.
		//Lan9355_setReg(REG_1588_TX_TIMESTAMP_CONFIG_X, port, dval); //SET All Msg
	}
}
void Lan9355_Set_Clock_and_Target_Clock(unsigned int sec, unsigned int ns){
	long dval = 0;

	//(1)Prepare 1588_CLOCK_SEC,1588_CLOCK_NS, 1588_CLOCK_SUBNS
	Lan9355_setReg(i1588_CLOCK_SEC,PORT_NONE, sec);
	Lan9355_setReg(i1588_CLOCK_NS, PORT_NONE,ns);
	Lan9355_setReg(i1588_CLOCK_SUBNS, PORT_NONE, 0x00);

	//(2)Write the value of 1588_CLOCK_SEC,1588_CLOCK_NS, 1588_CLOCK_SUBNS registers into 1588 clock
	dval = 0x00000010; //---10000b (1588_CLOCK_LOAD)
	Lan9355_setReg(i1588_CMD_CTL, PORT_NONE,dval);

	//(3) set Target Clock after 2 nsec to generate PPS.
	Lan9355_SetTargetClock(sec+2,ns); //we set the PPS pulse after 2 sec.
}

TimeInternal Lan9355_Get_Clock(){
	unsigned long dval;
	TimeInternal currentTime;
	unsigned long subns;

		dval = 0x00000008; //1000 (1588_CLOCK_READ)
		Lan9355_setReg(i1588_CMD_CTL, PORT_NONE, dval); //set READ. The current time is saved in 1588_CLOCK_SEC,1588_CLOCK_NS, 1588_CLOCK_SUBNS
		currentTime.seconds = Lan9355_getReg(i1588_CLOCK_SEC,PORT_NONE);
		currentTime.nanoseconds = Lan9355_getReg(i1588_CLOCK_NS, PORT_NONE);
		subns = Lan9355_getReg(i1588_CLOCK_SUBNS, PORT_NONE);
		printf("%us: %uns: %usubns\r\n", currentTime.seconds,currentTime.nanoseconds, subns);
		return currentTime;
}

TimeInternal Lan9355_Get_RxPDReqIngressTime(){
	unsigned long dval;
	TimeInternal currentTime;
	unsigned long subns;
	unsigned long port;

	//It is automatically updated whenever PDelay_Req msg is received, if AutoUpdate has been set.
	for(port=0;port<3;port++){
		currentTime.seconds = Lan9355_getReg(i1588_RX_PDREQ_SEC_X,port);
		currentTime.nanoseconds = Lan9355_getReg(i1588_RX_PDREQ_NS_X, port);
		printf("RxPDelayReqMsgIngressTime(Port%d)=",port);
		printf("%us:%uns\r\n", currentTime.seconds,currentTime.nanoseconds);
	}
}
TimeInternal Lan9355_Get_RxPDelayReq_Ingress_CorrectionField(){
	unsigned long dval;
	TimeInternal cf;
	unsigned long subns;
	unsigned long port;

	//It is automatically updated whenever PDelay_Req msg is received, if AutoUpdate has been set.
	for(port=0;port<3;port++){
		cf.seconds = Lan9355_getReg(i1588_RX_PDREQ_CF_HI_X,port);
		cf.nanoseconds = Lan9355_getReg(i1588_RX_PDREQ_CF_LOW_X, port);
		printf("RxPDelayReqMsgIngressCorrectionField(Port%d)=",port);
		printf("%us:%uns\r\n", cf.seconds,cf.nanoseconds);
	}
}

TimeInternal Lan9355_Get_Asym_PeerDelay(){
	unsigned long dlyAsymns;
	unsigned long rxPeerdlyns;
	unsigned long dval;
	unsigned long port;

	//It is automatically updated whenever PDelay_Req msg is received, if AutoUpdate has been set.
	for(port=0;port<3;port++){
		dval = Lan9355_getReg(i1588_ASYM_PEERDLY_X,port);
		dlyAsymns = (dval & 0xffff0000) >>16;
		rxPeerdlyns = (dval & 0x0000ffff);
		printf("DelayAsymmetry(Port%d)=%dns::",port, dlyAsymns);
		printf("RxPeerDelay(Port%d)=%uns\r\n",port, rxPeerdlyns);
	}
}

unsigned long Lan9355_Get_RxMsgHeader(){
	unsigned long seqId;
	unsigned long msgType;
	unsigned long dval;
	unsigned long port;

	for(port=0;port<3;port++){

		dval = Lan9355_getReg(i1588_CAP_INFO_X,port);
		if(dval & 0x07){
			printf("1588_CAP_INFO_X(RX_TS_CNT=%u) -- ",dval & 0x07 );
			dval = Lan9355_getReg(i1588_RX_MSG_HEADER_X,port);
			msgType = (dval & 0x000f0000) >>16;
			seqId = (dval & 0x0000ffff);
			printf("RxMsg(Port%d):Type=%u : Seq=%u\r\n",port, msgType, seqId);
		}
	}
}

void Lan9355_SetAutoConfig(){
	int i;
	char str[10];
	long dval = 0;
	unsigned long port=0;
	unsigned short reg;
	unsigned long bank=0;
	unsigned long sec, ns, subns;

	//Automatically updated whenever PDelay_Req msg is received.
	for(port=0;port<3;port++){
		Lan9355_setReg(i1588_RX_PDREQ_NS_X,port, 0x80000000); //set Auto Update bit.
	}
}

void Lan9355_GetRxParseConfig(){
	unsigned long port=0;
	unsigned long dval = 0;
	for(port=0;port<3;port++){
		dval = Lan9355_getReg(i1588_RX_PARSE_CONFIG_X,port);
		printf("Get1588_RX_PARSE_CONFIG_X(%d)=%08x\r\n",port,dval);
		delayms(10);
	}
}

void Lan9355_SetAndGetRxParseConfig(unsigned long dval){
	unsigned long port=0;
	for(port=0;port<3;port++){
		Lan9355_setReg(i1588_RX_PARSE_CONFIG_X,port,dval);
		delayms(10);
	}
	Lan9355_GetRxParseConfig();
}


void Lan9355_SetRxTimeStampConfig(unsigned long dval){
	unsigned long port=0;

	//Automatically updated whenever PDelay_Req msg is received.
	for(port=0;port<3;port++){
		Lan9355_setReg(i1588_RX_TIMESTAMP_CONFIG_X,port, dval); //set SYNC,DelayReq,PDelayReq,PDelayResp
	}
}
void Lan9355_SetTxTimeStampConfig(unsigned long dval){
	unsigned long port=0;

	//Automatically updated whenever PDelay_Req msg is received.
	for(port=0;port<3;port++){
		Lan9355_setReg(i1588_TX_TIMESTAMP_CONFIG_X,port, dval); //set SYNC,DelayReq,PDelayReq,PDelayResp
	}
}

//==== SET/GET TargetClock for PPS signal generation via GP7 output.
void LAN9355_Enable_1588_ClockTargetRead(void){
	long dval = 0;

	//clock read enable
	dval = 0x0000000c | 0x2000; //1100 (1588_CLOCK_READ | 1588_ENABLE | ClockTargetRead(bit13)
	Lan9355_setReg(i1588_CMD_CTL, PORT_NONE, dval); //set READ | 1588_ENABLE
}
//port 1, channel A only we support, AutoIncrement.
void Lan9355_SetTargetClock(unsigned int sec, unsigned int ns){
	unsigned long port=1;
	Lan9355_setReg(i1588_CLOCK_TARGET_SEC_X_CHA,port, sec);
	Lan9355_setReg(i1588_CLOCK_TARGET_NS_X_CHA,port, ns);
	Lan9355_setReg(i1588_CLOCK_TARGET_RELOAD_SEC_X_CHA,port, 1); //Reload/Add(*) Value is set with the auto increment value of 1 sec.
	Lan9355_setReg(i1588_CLOCK_TARGET_RELOAD_NS_X_CHA,port, 0);  //No Add value for nanosec.
}

void Lan9355_GetTargetClock(){ //port 1, channel A only we support, AutoIncrement.
	unsigned long port=1;
	//TimeInternal curTime;
	TimeInternal targetTime;

	//To read TargetRegister, we Set Clock_Target_Read bit.
	LAN9355_Enable_1588_ClockTargetRead();//Set Clock_Target_Read bit //It is the same as LAN9355_Enable_1588();

	targetTime.seconds = Lan9355_getReg(i1588_CLOCK_TARGET_SEC_X_CHA,port);
	targetTime.nanoseconds = Lan9355_getReg(i1588_CLOCK_TARGET_NS_X_CHA,port);
	printf("TagetTime=%us:%uns\r\n", targetTime.seconds,targetTime.nanoseconds);
	//curTime = Lan9355_Get_Clock();
	//printf("CurTime  =%us:%uns\r\n", curTime.seconds,curTime.nanoseconds);
}

//============= Ingress Time Insertion into Packets
//
void Lan9355_SetInsertTimeIntoPacket(unsigned long port){
	unsigned long dval;
	dval = 0x040000 | 0x000080 | 0x008000;//RX_PTP_INSERT_DREQ_DRESP_EN(bit17), RX_PTP_INSERT_TS_EN(bit7),RX_PTP_INSERT_TS_SEC_EN(bit15)
	Lan9355_setReg(i1588_RX_TS_INSERT_CONFIG_X, port, dval);
}
//=== GPIO
// For generating PPS signal through GP7, we set GP7 as 1588 Clock Event Output.
void Lan9355_SetGpio7asPPSoutput(void){
	int i;
	LAN9355_set_GPIO_CFG(0x00000000 | 0x00800000 | 0x00008000 | 0x80); //set 1588 channel A as the output | active high event clock output (GP7)| 1588 GPIO Output Enable | PushPull
	LAN9355_set_GPIO_DATA_DIR(0x00800000); //GP7 as output.

	//TEST output via GP7 with 1-0-1-0....1-0.
	for(i=0;i<10;i++){
		LAN9355_set_GPIO_DATA_DIR(0x00800080); //OUTPUT GP7, SET 1
		delayms(10);
		LAN9355_set_GPIO_DATA_DIR(0x00800000);//OUTPUT GP7, SET 0
	}

	// After then, the 100nsec pulse will be output when the clock target sec (Channel A) is equal to current time. (See 15.4)
	// Also the target sec register will be added by ONE sec automatically. (RELOAD_ADD = 0)

}


//================== Internal PHY ==========================
unsigned long LAN9355_get_phy_Reg(unsigned short phyaddr, unsigned short reg){
	long dval = 0;
	//PHYADDR[15:11] | MIIRegIndex[10:6]
	dval = phyaddr << 11;
	dval |= reg << 6;
	Lan9355_setReg(LAN9355_PMI_ACCESS, PORT_NONE, dval);
	//printf("Set 9355_PHY_REG(%x)=%08x\r\n",reg, dval);

	dval = Lan9355_getReg(LAN9355_PMI_DATA, PORT_NONE);
	printf("PHY_REG(%x)=%08x\r\n",reg, dval);
	return dval;
}

void LAN9355_ShowAll_Internal_Phy_Reg(){
	unsigned short i;
	//We pulled up pin24 (PED5/PHYADDR = 1)
	//PHY1 is the virtual PHY
	//PHY2
	printf(">InternalPHY2\r\n");
	for(i=0;i<32;i++){
		LAN9355_get_phy_Reg(2, i);
	}
	//PHY3
	printf(">InternalPHY3\r\n");
	for(i=0;i<32;i++){
		LAN9355_get_phy_Reg(3,i);
	}

}
//================== Virtual PHY ==========================
unsigned long LAN9355_get_Vphy_Reg(unsigned short reg){
	long dval = 0;

	dval = Lan9355_getReg(reg, PORT_NONE);
	printf("9355_VPHY_REG(%x)=%08x\r\n",reg, dval);
	return dval;
}

void LAN9355_ShowAll_Vphy_Reg(){
	//VPHY0
	LAN9355_get_Vphy_Reg(LAN9355_VPHY_BASIC_CTRL_0);
	LAN9355_get_Vphy_Reg(LAN9355_VPHY_BASIC_STATUS_0);
	LAN9355_get_Vphy_Reg(LAN9355_VPHY_SPECIAL_CONTROL_STATUS_0);
	//VPHY1
	LAN9355_get_Vphy_Reg(LAN9355_VPHY_BASIC_CTRL_1);
	LAN9355_get_Vphy_Reg(LAN9355_VPHY_BASIC_STATUS_1);
	LAN9355_get_Vphy_Reg(LAN9355_VPHY_SPECIAL_CONTROL_STATUS_1);
}


void Lan9355_ShowAllReg(){
	int i;
	unsigned long port=0;
	unsigned long dval = 0;

	for(i=0;i<SIZEOFREG;i++){
		for(port=0;port<3;port++){
			dval = Lan9355_getReg(LAN9355_1588_REGS[i].regAddr,port);
			printf("%s_X(%d)=%08x\r\n",LAN9355_1588_REGS[i].regName, port,dval);
			delayms(10);
		}
	}
	LAN9355_ShowAll_Vphy_Reg();
	delayms(100);
	LAN9355_ShowAll_Internal_Phy_Reg();
	delayms(200);
}
//=====================
void PB5_AS_GPIO_setup(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);// ZGT6
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);// ZGT6
}
void Lan9355_PE3_UsrButtonInput_setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  /* Enable GPIOE clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	  /* Enable SYSCFG clock */
	  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Configure PE3 pin as input floating */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	  GPIO_Init(GPIOE, &GPIO_InitStructure);

	  /*
	  // Connect EXTI Line3 to PE3 pin
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  // Enable and set EXTI Line0 Interrupt to the lowest priority
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	  */
}
//PE3 = UsrButton for STM32 MCU

void Lan9355_Config(){
	int i;
	char str[10];
	long dval = 0;
	unsigned long port=0;
	unsigned short reg;
	unsigned long bank=0;
	unsigned long sec, ns, subns;

	stmOzOLED_printString("Config",0,1,6);

	//Lan9355_PE3_UsrButtonInput_setup();

	//Lan9355_SetGpio7asPPSoutput(); //1588 clock event with target clock will be triggered via PIN28 of I/F, which is tied with PIN28 of MCU I/F (PA15).

	//Lan9355_GetRxParseConfig();
	//Lan9355_SetAndGetRxParseConfig(0x0000711f);

	//correction field enable per port
	//Lan9355_Enable_Rx_CorrectionField();
	//Lan9355_Enable_Tx_CorrectionField();

	//Lan9355_SetTxTimeStampConfig(0x0000000F);
	//Lan9355_SetRxTimeStampConfig(0x0000000F);

	//Automatically updated whenever PDelay_Req msg is received.
	//Lan9355_SetAutoConfig();

	//LAN9355_1588_INT_EN();//
	//LAN9355_IRQ_Config();

	//Enable the timestamp insertion on received packet in reserved fields.(v2)
	//Lan9355_SetInsertTimeIntoPacket(0);
	//Lan9355_SetInsertTimeIntoPacket(1);
	//Lan9355_SetInsertTimeIntoPacket(2);

	//Lan9355_ShowAllReg();

	//enable 1588 function
	//LAN9355_Enable_1588();

	//GP7_PA15_setup(); //Interrupt enabled GPInput.(with EXTI)
	stmOzOLED_printString("Config:Done",0,1,11);

}
//========== PA15 = Gp7 === 1588 clock event input from LAN9355.
//Interrupt enabled GPInput.(with EXTI)
void GP7_PA15_setup(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  /* Enable GPIOE clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Configure PE0 pin as input floating */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Connect EXTI Line15 to PA15 pin */

	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);

	  /* Configure EXTI Line0 */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
/*
//Useful for GPS -- Defined in yEther.c
void EXTI15_10_IRQHandler(void){
	 if(EXTI_GetITStatus(EXTI_Line15) != RESET){ //PB15 from GPS's PPS
		 EXTI_ClearITPendingBit(EXTI_Line15); //Clear this IRQ.
		//g_tickCnt++;
		g_tickEvent = 1;
	 }
}
*/
void Lan9355_qei_control(){
	unsigned short qeival = 50;
	qeival = stm_Qei_GetValue();
				if(qeival < 50){
					LAN9355_ClockRateAdj_10nsecPerSec(CLOCK_SLOWER);
					printf("Slow...\r\n");
				}else if(qeival > 50){
					LAN9355_ClockRateAdj_10nsecPerSec(CLOCK_FASTER);
					printf("Faster...\r\n");
				}

}
//Set next target clock and generate PPS pulse on PB5.
void Lan9355_GenPPS(void){
	TimeInternal currentTime;
	currentTime = Lan9355_Get_Clock();
	//LAN9355_ClockStepSec(1, 0x0F); //HomeWork
	//currentTime = Lan9355_Get_Clock();
	Lan9355_SetTargetClock(currentTime.seconds + 1, 0); //we should set the PPS pulse after 1 sec.

	GPIO_SetBits(GPIOB, GPIO_Pin_5); // PULSE ON
	delayms(100);
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);// PULSE OFF
}

// Record the current time to check an upcoming timeout against
#define stm_Lan9355_StartTimeout() (g_Lan9355.timeout_start_ms = millis())
// Check if timeout is enabled (set to nonzero value) and has expired
//#define stm_Lan9355_CheckTimeoutExpired() (g_Lan9355.io_timeout > 0 && ((uint16_t)millis() - g_Lan9355.timeout_start_ms) > g_Lan9355.io_timeout)
inline bool stm_Lan9355_CheckTimeoutExpired() {
			if(g_Lan9355.io_timeout > 0){
				if(((unsigned short)millis() - g_Lan9355.timeout_start_ms) > g_Lan9355.io_timeout){
					return 1;
				}
			}
			return 0;
}

inline void stm_Lan9355_setTimeout(uint16_t timeout) {
	g_Lan9355.io_timeout = timeout;
}
inline uint16_t stm_Lan9355_getTimeout(void) { return g_Lan9355.io_timeout; }
 // Did a timeout occur in one of the read functions since the last call to
 // timeoutOccurred()?
bool stm_Lan9355_timeoutOccurred()
{
   bool tmp = g_Lan9355.did_timeout;
   g_Lan9355.did_timeout = false;
   return tmp;
}
extern unsigned char g_ip_addr3;
void stm_Lan9355_ipConfig(){
	int i;
	char ipstr[16];

	Lan9355_PE3_UsrButtonInput_setup();

	g_Lan9355.did_timeout = 0;
	g_Lan9355.io_timeout =0;
	g_Lan9355.timeout_start_ms = 0;
	printf("ipConfig. Press Usr Button in 5sec.\r\n");
	stmOzOLED_printString("PRESS BUTTON",0,7,15);
	for(i=0;i<10;i++){
		stm_Lan9355_setTimeout(5000); //1sec
		stm_Lan9355_StartTimeout();
		while(1){
			if (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)== 0){ //active low
				g_ip_addr3++;
				printf("IPaddr=%u.%u.%u.%u\r\n",IP_ADDR0,IP_ADDR1,IP_ADDR2,g_ip_addr3);
				sprintf(ipstr,"%u.%u.%u.%u",IP_ADDR0,IP_ADDR1,IP_ADDR2,g_ip_addr3);
				stmOzOLED_printString(ipstr,0,1,15); //
				delayms(500);
				break;
			}
			if (stm_Lan9355_CheckTimeoutExpired()){
				g_Lan9355.did_timeout = true;
				break;
			}
		}
		if(g_Lan9355.did_timeout){
			break;
		}
	}
	printf("IPaddr=%u.%u.%u.%u\r\n",IP_ADDR0,IP_ADDR1,IP_ADDR2,g_ip_addr3);
	sprintf(ipstr,"%u.%u.%u.%u",IP_ADDR0,IP_ADDR1,IP_ADDR2,g_ip_addr3);
	stmOzOLED_printString(ipstr,0,1,15); //
	stmOzOLED_printString("EtherCraft",0,7,15); //
}
//STP===========================
/* Map ALR-port bits to port bitmap, and back */
static const int alrport_2_portmap[] = {1, 2, 4, 0, 3, 5, 6, 7 };
static const int portmap_2_alrport[] = {3, 0, 1, 4, 2, 5, 6, 7 };

/* forward special tagged packets from port 0 to port 1 *or* port 2 */
void Lan9355_setup_tagging_dsa(void)
{
	u32 val;

	printf("Lan9355_setup_tagging_dsa for Port 0 with special VLAN tagging\r\n");
	// enable defining the destination port via special VLAN tagging for port 0
	Lan9355_setReg(LAN9355_SWE_INGRESS_PORT_TYP_DIRECT, PORT_NONE, LAN9355_SWE_INGRESS_PORT_TYPE_VLAN); //PORT 0

	// tag incoming packets at port 1 and 2 on their way to port 0 to be able to discover their source port
	val = LAN9355_BM_EGRSS_PORT_TYPE_SPECIAL_TAG_PORT0;//PORT 0
	Lan9355_setReg(LAN9355_BM_EGRSS_PORT_TYPE_DIRECT,  PORT_NONE, val);//PORT 0

}

#define BR_STATE_DISABLED 0
#define BR_STATE_BLOCKING 1
#define BR_STATE_LISTENING 2
#define BR_STATE_LEARNING 3
#define BR_STATE_FORWARDING 4

static void Lan9355_port_stp_state_set(int port, u8 state)
{
	int portmask, portstate;
	//dev_dbg(chip->dev, "%s(port %d, state %d)\n",	__func__, port, state);
	switch (state) {
	case BR_STATE_DISABLED:
		portstate = LAN9355_SWE_PORT_STATE_DISABLED_PORT0;
		break;
	case BR_STATE_BLOCKING:
	case BR_STATE_LISTENING:
		portstate = LAN9355_SWE_PORT_STATE_BLOCKING_PORT0;
		break;
	case BR_STATE_LEARNING:
		portstate = LAN9355_SWE_PORT_STATE_LEARNING_PORT0;
		break;
	case BR_STATE_FORWARDING:
		portstate = LAN9355_SWE_PORT_STATE_FORWARDING_PORT0;
		break;
	default:
		portstate = LAN9355_SWE_PORT_STATE_DISABLED_PORT0;
		//dev_err(chip->dev, "unknown stp state: port %d, state %d\n",			port, state);
	}
	portmask = 0x3 << (port * 2);
	portstate <<= (port * 2);
	g_Lan9355.swe_port_state = (g_Lan9355.swe_port_state & ~portmask) | portstate;

	if (g_Lan9355.is_bridged)	Lan9355_setReg(LAN9355_SWE_PORT_STATE_DIRECT, PORT_NONE, g_Lan9355.swe_port_state);
	//else: touching SWE_PORT_STATE would break port separation
}
/*
static void Lan9355_port_fast_age(struct dsa_switch *ds, int port)
{
	struct del_port_learned_ctx del_ctx = {	.port = port,};
	//dev_dbg(chip->dev, "%s(%d)\n", __func__, port);
	lan9355_alr_loop(alr_loop_cb_del_port_learned, &del_ctx);
}
*/
/* We want a special working switch:
 * - do not forward packets between port 1 and 2
 * - forward everything from port 1 to port 0
 * - forward everything from port 2 to port 0
 */
//TBD --
static int lan9355_separate_ports(struct lan9303 *chip)
{
	int ret;

	//Lan9355_alr_del_port(chip, eth_stp_addr, 0);
	Lan9355_setReg(LAN9355_SWE_PORT_MIRROR, PORT_NONE,
				LAN9355_SWE_PORT_MIRROR_SNIFFER_PORT0 |
				LAN9355_SWE_PORT_MIRROR_MIRRORED_PORT1 |
				LAN9355_SWE_PORT_MIRROR_MIRRORED_PORT2 |
				LAN9355_SWE_PORT_MIRROR_ENABLE_RX_MIRRORING |
				LAN9355_SWE_PORT_MIRROR_SNIFF_ALL);

	// prevent port 1 and 2 from forwarding packets by their own
	Lan9355_setReg(LAN9355_SWE_PORT_STATE, PORT_NONE,
				LAN9355_SWE_PORT_STATE_FORWARDING_PORT0 |
				LAN9355_SWE_PORT_STATE_BLOCKING_PORT1 |
				LAN9355_SWE_PORT_STATE_BLOCKING_PORT2);

	return 0;
}

Lan9355_enable_processing_port( unsigned int port)
{
	printf("Lan9355_enable_processing_port for Port 0\r\n");
	if(port == 0){
		// enable RX and keep register reset default values else
		Lan9355_setReg(LAN9355_MAC_RX_CFG_0_DIRECT, port,
					LAN9355_MAC_RX_CFG_X_REJECT_MAC_TYPES |	LAN9355_MAC_RX_CFG_X_RX_ENABLE);
		// enable TX and keep register reset default values else
		Lan9355_setReg(LAN9355_MAC_TX_CFG_0_DIRECT, port,
				LAN9355_MAC_TX_CFG_X_TX_IFG_CONFIG_DEFAULT |
				LAN9355_MAC_TX_CFG_X_TX_PAD_ENABLE |
				LAN9355_MAC_TX_CFG_X_TX_ENABLE);
	}else if(port == 1){
		// enable RX and keep register reset default values else
		Lan9355_setReg(LAN9355_MAC_RX_CFG_1_DIRECT, port,
				LAN9355_MAC_RX_CFG_X_REJECT_MAC_TYPES |	LAN9355_MAC_RX_CFG_X_RX_ENABLE);
		// enable TX and keep register reset default values else
		Lan9355_setReg(LAN9355_MAC_TX_CFG_1_DIRECT, port,
				LAN9355_MAC_TX_CFG_X_TX_IFG_CONFIG_DEFAULT |
				LAN9355_MAC_TX_CFG_X_TX_PAD_ENABLE |
				LAN9355_MAC_TX_CFG_X_TX_ENABLE);
	}else if(port == 2){
		// enable RX and keep register reset default values else
		Lan9355_setReg(LAN9355_MAC_RX_CFG_2_DIRECT, port,
					LAN9355_MAC_RX_CFG_X_REJECT_MAC_TYPES |	LAN9355_MAC_RX_CFG_X_RX_ENABLE);
		// enable TX and keep register reset default values else
		Lan9355_setReg(LAN9355_MAC_TX_CFG_2_DIRECT, port,
			LAN9355_MAC_TX_CFG_X_TX_IFG_CONFIG_DEFAULT |
			LAN9355_MAC_TX_CFG_X_TX_PAD_ENABLE |
			LAN9355_MAC_TX_CFG_X_TX_ENABLE);
	}
}

Lan9355_write_switch_reg_mask(u16 regnum, u32 givenVal, u32 mask)
{
	int ret;
	u32 val;

	val = Lan9355_getReg(regnum, PORT_NONE);//ret = Lan9355_read_switch_reg(chip, regnum, &reg);

	val = (val & ~mask) | givenVal;

	Lan9355_setReg(regnum,  PORT_NONE, val);//return lan9355_write_switch_reg(chip, regnum, reg);
}

static int Lan9355_alr_make_entry_raw(u32 dat0, u32 dat1)
{
	Lan9355_setReg(LAN9355_SWE_ALR_WR_DAT_0_DIRECT, PORT_NONE, dat0);
	Lan9355_setReg(LAN9355_SWE_ALR_WR_DAT_1_DIRECT, PORT_NONE, dat1);
	Lan9355_setReg(LAN9355_SWE_ALR_CMD_DIRECT, PORT_NONE, LAN9355_ALR_CMD_MAKE_ENTRY);
	//lan9355_csr_reg_wait(LAN9355_SWE_ALR_CMD_STS, ALR_STS_MAKE_PEND);
	Lan9355_setReg(LAN9355_SWE_ALR_CMD_DIRECT, PORT_NONE, 0);
	return 0;
}
/* Set a static ALR entry. Delete entry if port_map is zero */
static void Lan9355_alr_set_entry(const u8 *mac,  u8 port_map, bool stp_override)
{
	u32 dat0, dat1, alr_port;

	//dev_dbg(chip->dev, "%s(%pM, %d)\n", __func__, mac, port_map);
	dat1 = LAN9355_ALR_DAT1_STATIC;
	if (port_map)	dat1 |= LAN9355_ALR_DAT1_VALID;
	// otherwise no ports: delete entry
	if (stp_override)	dat1 |= LAN9355_ALR_DAT1_AGE_OVERRID;

	alr_port = portmap_2_alrport[port_map & 7];
	dat1 &= ~LAN9355_ALR_DAT1_PORT_MASK;
	dat1 |= alr_port << LAN9355_ALR_DAT1_PORT_BITOFFS;

	dat0 = 0;
	dat0 |= (mac[0] << 0);
	dat0 |= (mac[1] << 8);
	dat0 |= (mac[2] << 16);
	dat0 |= (mac[3] << 24);

	dat1 |= (mac[4] << 0);
	dat1 |= (mac[5] << 8);

	Lan9355_alr_make_entry_raw(dat0, dat1);
}
// Add port to static ALR entry, create new static entry if needed
static int lan9355_alr_add_port(const u8 *mac, int port, bool stp_override)
{
	struct lan9355_alr_cache_entry *entr;
/*
	entr = lan9355_alr_cache_find_mac(chip, mac);
	if (!entr) { //New entry
		entr = lan9355_alr_cache_find_free(chip);
		if (!entr) {
			return -ENOSPC;
		}
		ether_addr_copy(entr->mac_addr, mac);
	}
	entr->port_map |= BIT(port);
	entr->stp_override = stp_override;
	lan9355_alr_set_entry(chip, mac, entr->port_map, stp_override);
*/
	return 0;
}

static void lan9355_bridge_ports(void)
{
	// ports bridged: remove mirroring
	Lan9355_setReg(LAN9355_SWE_PORT_MIRROR, PORT_NONE, LAN9355_SWE_PORT_MIRROR_DISABLED);//	lan9355_write_switch_reg(LAN9355_SWE_PORT_MIRROR, LAN9355_SWE_PORT_MIRROR_DISABLED);
	//Lan9355_setReg(LAN9355_SWE_PORT_STATE,PORT_NONE,  chip->swe_port_state);//	lan9355_write_switch_reg(LAN9355_SWE_PORT_STATE, chip->swe_port_state);
	//Lan9355_alr_add_port(eth_stp_addr, 0, true);
}



// =================== High Level ===========================
void Lan9355_init(){
	unsigned long ret = 0x00;
	char str[10];
	//stm_I2C_Init(I2C1, 400000);//400Kbps
	//stmOzOLED_init(">LAN9355<");

	//PA3 == INT from LAN9355 IRQ

	//Check BYTE ORDER TEST REGISTER (0x64)
	//See 18.1 Miscellaneous System Configuration and Status Registers
	// 0x050 (ID_REV)   = 0x9355 REV
	// 0x064 (BYTE_TEST)= 0x87654321
	// 0x074 (HW_CFG)   =
	while(ret != 0x87654321){
		stm_I2C_ReceiveBurstWithRestartCondition(LAN9355_ADDRESS8, GetRegAddr(LAN9355_BYTE_TEST), &regbuf[0], 4);
		ret = regbuf[0]*0x1000000 + regbuf[1]*0x10000 + regbuf[2]*0x100 + regbuf[3];
	}

	LAN9355_WaitForDeviceReady();

	LAN9355_GetChipIdRev();

	//EthIRQ_PA3_setup();

	delayms(2000);
	Lan9355_ShowAllReg();


	stmOzOLED_printString("Init:Pass+",0,1,10);
	return;

}
//============= main ==============
void Lan9355_Loop(){
	int i;
	char str[10];
	long dval = 0;
	unsigned long port=0;
	unsigned short reg;
	unsigned long bank=0;
	unsigned long sec, ns, subns;
	TimeInternal currentTime;

//	PB5_AS_GPIO_setup();
/*
	while(1){
		GPIO_SetBits(GPIOB, GPIO_Pin_5); // PULSE ON
		delayms(100);
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);// PULSE OFF
		delayms(900);
	}
*/
	stmOzOLED_printString("LAN9355",0,1,11);
	delayms(1000);

#ifdef USE_MCO1_FOR_PHY_CLOCK_SRC
   /* Output HSE clock (25MHz) on MCO pin (PA8) to clock the PHY */
	MCO1_Config_25MHz();
	printf("***Using MCO1 25MHz Output ***\r\n");

	delayms(500); //Should be here to settle down the clock.
#endif /* USE_MCO1_FOR_PHY_CLOCK_SRC */

	Lan9355_init();
	Lan9355_Config();

	LAN9355_ShowAll_Internal_Phy_Reg();
	delayms(1000);

	printf("Show Switch Fabric Control and Status Registers(Indirect)\r\n");
/*	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SW_DEV_ID));
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SW_RESET));
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SW_IMR));
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SW_IPR));
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_MAC_VER_ID_0));
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_MAC_RX_CFG_0));//
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_MAC_TX_CFG_0));

	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SWE_PORT_MIRROR));
*/

	//Set DSA for Port 0
	Lan9355_setup_tagging_dsa();
	//Lan9355_separate_ports();
	Lan9355_enable_processing_port(0);

	// Trap IGMP to port 0
	//Lan9355_write_switch_reg_mask(LAN9355_SWE_GLB_INGRESS_CFG,  LAN9355_SWE_GLB_INGR_IGMP_TRAP |   LAN9355_SWE_GLB_INGR_IGMP_PORT(0),  LAN9355_SWE_GLB_INGR_IGMP_PORT(1) |   LAN9355_SWE_GLB_INGR_IGMP_PORT(2));
	u32 mask, val;
	mask =  LAN9355_SWE_GLB_INGR_IGMP_PORT(1) | LAN9355_SWE_GLB_INGR_IGMP_PORT(2);
	val = Lan9355_getRegIndirect(LAN9355_SWE_GLB_INGRESS_CFG);
	val = (val & ~mask) | (LAN9355_SWE_GLB_INGR_IGMP_TRAP | LAN9355_SWE_GLB_INGR_IGMP_PORT(0));
	Lan9355_setReg(LAN9355_SWE_GLB_INGRESS_CFG_DIRECT, PORT_NONE,val);

	//Set Port Mirroring
	Lan9355_setReg(LAN9355_SWE_PORT_MIRROR_DIRECT,PORT_NONE,0x0000003b);//001-110-1-1: Sniff Port = 0; Mirrored Port = 1,2. Enable RX/TX Mirroring
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SWE_PORT_MIRROR));

	//
	Lan9355_Set_Clock_and_Target_Clock(12345678, 0x00000000);

	//LAN9355_ClockRateAdj(CLOCK_FASTER,0x2B100);
	//LAN9355_ClockRateAdj_10nsecPerSec(CLOCK_FASTER);

	/*
	//TEST FOR ClockStepSec====WORKING
	currentTime = Lan9355_Get_Clock();
	LAN9355_ClockStepSec(1, 0x0F);
	currentTime = Lan9355_Get_Clock();
	Lan9355_SetTargetClock(currentTime.seconds + 1, 0); //we should set the PPS pulse after 1 sec.
	//==========================
	 */

	Init_SysTick(1000);//1msec

    //stm_Qei_Config(100);

	while(1){
		if(g_tickEvent){
			g_tickEvent=0;
			currentTime = Lan9355_Get_Clock();
			printf("CurTime=(%us:%uns:%usubns)\r\n", currentTime.seconds,currentTime.nanoseconds, subns);
			//LAN9355_1588_INT_STS();

			//Lan9355_Get_RxPDReqIngressTime();
			//Lan9355_Get_RxPDelayReq_Ingress_CorrectionField();

			//Lan9355_Get_Asym_PeerDelay();
			//Lan9355_Get_RxMsgHeader();
			LAN9355_1588_getINT_STS();

			//Lan9355_GetTargetClock();
			 //displayCurrent();
			 //displayState(&ptpClock);
			//Lan9355_qei_control();

			Lan9355_GenPPS();


		}
	}
}

//============= main ==============
void Lan9355_Tap_Loop(){
	int i;
	char str[10];
	long dval = 0;
	unsigned long port=0;
	unsigned short reg;
	unsigned long bank=0;
	unsigned long sec, ns, subns;
	TimeInternal currentTime;

//	PB5_AS_GPIO_setup();
/*
	while(1){
		GPIO_SetBits(GPIOB, GPIO_Pin_5); // PULSE ON
		delayms(100);
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);// PULSE OFF
		delayms(900);
	}
*/
	stmOzOLED_printString("LAN9355TAP",0,1,11);
	delayms(1000);

#ifdef USE_MCO1_FOR_PHY_CLOCK_SRC
   /* Output HSE clock (25MHz) on MCO pin (PA8) to clock the PHY */
	MCO1_Config_25MHz();
	printf("***Using MCO1 25MHz Output ***\r\n");

	delayms(500); //Should be here to settle down the clock.
#endif /* USE_MCO1_FOR_PHY_CLOCK_SRC */

	Lan9355_init();
	Lan9355_Config();

	LAN9355_ShowAll_Internal_Phy_Reg();
	delayms(1000);

	printf("Show Switch Fabric Control and Status Registers(Indirect)\r\n");
/*	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SW_DEV_ID));
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SW_RESET));
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SW_IMR));
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SW_IPR));
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_MAC_VER_ID_0));
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_MAC_RX_CFG_0));//
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_MAC_TX_CFG_0));

	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SWE_PORT_MIRROR));
*/

	//Set DSA for Port 0
	Lan9355_setup_tagging_dsa();
	//Lan9355_separate_ports();
	Lan9355_enable_processing_port(0);

	// Trap IGMP to port 0
	//Lan9355_write_switch_reg_mask(LAN9355_SWE_GLB_INGRESS_CFG,  LAN9355_SWE_GLB_INGR_IGMP_TRAP |   LAN9355_SWE_GLB_INGR_IGMP_PORT(0),  LAN9355_SWE_GLB_INGR_IGMP_PORT(1) |   LAN9355_SWE_GLB_INGR_IGMP_PORT(2));
	u32 mask, val;
	mask =  LAN9355_SWE_GLB_INGR_IGMP_PORT(1) | LAN9355_SWE_GLB_INGR_IGMP_PORT(2);
	val = Lan9355_getRegIndirect(LAN9355_SWE_GLB_INGRESS_CFG);
	val = (val & ~mask) | (LAN9355_SWE_GLB_INGR_IGMP_TRAP | LAN9355_SWE_GLB_INGR_IGMP_PORT(0));
	Lan9355_setReg(LAN9355_SWE_GLB_INGRESS_CFG_DIRECT, PORT_NONE,val);

	//Set Port Mirroring
	Lan9355_setReg(LAN9355_SWE_PORT_MIRROR_DIRECT,PORT_NONE,0x0000003b);//001-110-1-1: Sniff Port = 0; Mirrored Port = 1,2. Enable RX/TX Mirroring
	printf("%08x\r\n",Lan9355_getRegIndirect(LAN9355_SWE_PORT_MIRROR));

	//
	Lan9355_Set_Clock_and_Target_Clock(12345678, 0x00000000);

	//LAN9355_ClockRateAdj(CLOCK_FASTER,0x2B100);
	//LAN9355_ClockRateAdj_10nsecPerSec(CLOCK_FASTER);

	/*
	//TEST FOR ClockStepSec====WORKING
	currentTime = Lan9355_Get_Clock();
	LAN9355_ClockStepSec(1, 0x0F);
	currentTime = Lan9355_Get_Clock();
	Lan9355_SetTargetClock(currentTime.seconds + 1, 0); //we should set the PPS pulse after 1 sec.
	//==========================
	 */

	Init_SysTick(1000);//1msec

    //stm_Qei_Config(100);

	while(1){
		if(g_tickEvent){
			g_tickEvent=0;
			currentTime = Lan9355_Get_Clock();
			printf("CurTime=(%us:%uns:%usubns)\r\n", currentTime.seconds,currentTime.nanoseconds, subns);
			//LAN9355_1588_INT_STS();

			//Lan9355_Get_RxPDReqIngressTime();
			//Lan9355_Get_RxPDelayReq_Ingress_CorrectionField();

			//Lan9355_Get_Asym_PeerDelay();
			//Lan9355_Get_RxMsgHeader();
			LAN9355_1588_getINT_STS();

			//Lan9355_GetTargetClock();
			 //displayCurrent();
			 //displayState(&ptpClock);
			//Lan9355_qei_control();

			Lan9355_GenPPS();


		}
	}
}
#endif

#if 0
static int lan9355_virt_phy_reg_read(struct lan9303 *chip, int regnum)
{
	int ret;
	u32 val;

	if (regnum > MII_EXPANSION)
		return -EINVAL;

	ret = lan9355_read(chip->regmap, LAN9355_VIRT_PHY_BASE + regnum, &val);
	if (ret)
		return ret;

	return val & 0xffff;
}

static int lan9355_virt_phy_reg_write(struct lan9303 *chip, int regnum, u16 val)
{
	if (regnum > MII_EXPANSION)
		return -EINVAL;

	return regmap_write(chip->regmap, LAN9355_VIRT_PHY_BASE + regnum, val);
}

static int lan9355_indirect_phy_wait_for_completion(struct lan9303 *chip)
{
	return lan9355_read_wait(chip, LAN9355_PMI_ACCESS,
				 LAN9355_PMI_ACCESS_MII_BUSY);
}

static int lan9355_indirect_phy_read(struct lan9303 *chip, int addr, int regnum)
{
	int ret;
	u32 val;

	val = LAN9355_PMI_ACCESS_PHY_ADDR(addr);
	val |= LAN9355_PMI_ACCESS_MIIRINDA(regnum);

	mutex_lock(&chip->indirect_mutex);

	ret = lan9355_indirect_phy_wait_for_completion(chip);
	if (ret)
		goto on_error;

	/* start the MII read cycle */
	ret = regmap_write(chip->regmap, LAN9355_PMI_ACCESS, val);
	if (ret)
		goto on_error;

	ret = lan9355_indirect_phy_wait_for_completion(chip);
	if (ret)
		goto on_error;

	/* read the result of this operation */
	ret = lan9355_read(chip->regmap, LAN9355_PMI_DATA, &val);
	if (ret)
		goto on_error;

	mutex_unlock(&chip->indirect_mutex);

	return val & 0xffff;

on_error:
	mutex_unlock(&chip->indirect_mutex);
	return ret;
}

static int lan9355_indirect_phy_write(struct lan9303 *chip, int addr,
				      int regnum, u16 val)
{
	int ret;
	u32 reg;

	reg = LAN9355_PMI_ACCESS_PHY_ADDR(addr);
	reg |= LAN9355_PMI_ACCESS_MIIRINDA(regnum);
	reg |= LAN9355_PMI_ACCESS_MII_WRITE;

	mutex_lock(&chip->indirect_mutex);

	ret = lan9355_indirect_phy_wait_for_completion(chip);
	if (ret)
		goto on_error;

	/* write the data first... */
	ret = regmap_write(chip->regmap, LAN9355_PMI_DATA, val);
	if (ret)
		goto on_error;

	/* ...then start the MII write cycle */
	ret = regmap_write(chip->regmap, LAN9355_PMI_ACCESS, reg);

on_error:
	mutex_unlock(&chip->indirect_mutex);
	return ret;
}

const struct lan9355_phy_ops lan9355_indirect_phy_ops = {
	.phy_read = lan9355_indirect_phy_read,
	.phy_write = lan9355_indirect_phy_write,
};
EXPORT_SYMBOL_GPL(lan9355_indirect_phy_ops);

static int lan9355_write_switch_reg(struct lan9303 *chip, u16 regnum, u32 val)
{
	u32 reg;
	int ret;

	reg = regnum;
	reg |= LAN9355_SWITCH_CSR_CMD_LANES;
	reg |= LAN9355_SWITCH_CSR_CMD_BUSY;

	mutex_lock(&chip->indirect_mutex);

	ret = lan9355_switch_wait_for_completion(chip);
	if (ret)
		goto on_error;

	ret = regmap_write(chip->regmap, LAN9355_SWITCH_CSR_DATA, val);
	if (ret) {
		dev_err(chip->dev, "Failed to write csr data reg: %d\n", ret);
		goto on_error;
	}

	/* trigger write */
	ret = regmap_write(chip->regmap, LAN9355_SWITCH_CSR_CMD, reg);
	if (ret)
		dev_err(chip->dev, "Failed to write csr command reg: %d\n",
			ret);

on_error:
	mutex_unlock(&chip->indirect_mutex);
	return ret;
}



static int lan9355_write_switch_port(struct lan9303 *chip, int port,
				     u16 regnum, u32 val)
{
	return lan9355_write_switch_reg(
		chip, LAN9355_SWITCH_PORT_REG(port, regnum), val);
}

static int lan9355_read_switch_port(struct lan9303 *chip, int port,
				    u16 regnum, u32 *val)
{
	return lan9355_read_switch_reg(
		chip, LAN9355_SWITCH_PORT_REG(port, regnum), val);
}

static int lan9355_detect_phy_setup(struct lan9303 *chip)
{
	int reg;

	/* Calculate chip->phy_addr_base:
	 * Depending on the 'phy_addr_sel_strap' setting, the three phys are
	 * using IDs 0-1-2 or IDs 1-2-3. We cannot read back the
	 * 'phy_addr_sel_strap' setting directly, so we need a test, which
	 * configuration is active:
	 * Special reg 18 of phy 3 reads as 0x0000, if 'phy_addr_sel_strap' is 0
	 * and the IDs are 0-1-2, else it contains something different from
	 * 0x0000, which means 'phy_addr_sel_strap' is 1 and the IDs are 1-2-3.
	 * 0xffff is returned on MDIO read with no response.
	 */
	reg = chip->ops->phy_read(chip, 3, MII_LAN911X_SPECIAL_MODES);
	if (reg < 0) {
		dev_err(chip->dev, "Failed to detect phy config: %d\n", reg);
		return reg;
	}

	chip->phy_addr_base = reg != 0 && reg != 0xffff;

	dev_dbg(chip->dev, "Phy setup '%s' detected\n",
		chip->phy_addr_base ? "1-2-3" : "0-1-2");

	return 0;
}

/* Return pointer to first free ALR cache entry, return NULL if none */
static struct lan9355_alr_cache_entry *
lan9355_alr_cache_find_free(struct lan9303 *chip)
{
	int i;
	struct lan9355_alr_cache_entry *entr = chip->alr_cache;

	for (i = 0; i < LAN9355_NUM_ALR_RECORDS; i++, entr++)
		if (entr->port_map == 0)
			return entr;

	return NULL;
}

/* Return pointer to ALR cache entry matching MAC address */
static struct lan9355_alr_cache_entry *
lan9355_alr_cache_find_mac(struct lan9303 *chip, const u8 *mac_addr)
{
	int i;
	struct lan9355_alr_cache_entry *entr = chip->alr_cache;

	BUILD_BUG_ON_MSG(sizeof(struct lan9355_alr_cache_entry) & 1,
			 "ether_addr_equal require u16 alignment");

	for (i = 0; i < LAN9355_NUM_ALR_RECORDS; i++, entr++)
		if (ether_addr_equal(entr->mac_addr, mac_addr))
			return entr;

	return NULL;
}

static int lan9355_csr_reg_wait(struct lan9303 *chip, int regno, u32 mask)
{
	int i;

	for (i = 0; i < 25; i++) {
		u32 reg;

		lan9355_read_switch_reg(chip, regno, &reg);
		if (!(reg & mask))
			return 0;
		usleep_range(1000, 2000);
	}

	return -ETIMEDOUT;
}



typedef void alr_loop_cb_t(struct lan9303 *chip, u32 dat0, u32 dat1,
			   int portmap, void *ctx);

static void lan9355_alr_loop(struct lan9303 *chip, alr_loop_cb_t *cb, void *ctx)
{
	int i;

	mutex_lock(&chip->alr_mutex);
	lan9355_write_switch_reg(chip, LAN9355_SWE_ALR_CMD,
				 LAN9355_ALR_CMD_GET_FIRST);
	lan9355_write_switch_reg(chip, LAN9355_SWE_ALR_CMD, 0);

	for (i = 1; i < LAN9355_NUM_ALR_RECORDS; i++) {
		u32 dat0, dat1;
		int alrport, portmap;

		lan9355_read_switch_reg(chip, LAN9355_SWE_ALR_RD_DAT_0, &dat0);
		lan9355_read_switch_reg(chip, LAN9355_SWE_ALR_RD_DAT_1, &dat1);
		if (dat1 & LAN9355_ALR_DAT1_END_OF_TABL)
			break;

		alrport = (dat1 & LAN9355_ALR_DAT1_PORT_MASK) >>
						LAN9355_ALR_DAT1_PORT_BITOFFS;
		portmap = alrport_2_portmap[alrport];

		cb(chip, dat0, dat1, portmap, ctx);

		lan9355_write_switch_reg(chip, LAN9355_SWE_ALR_CMD,
					 LAN9355_ALR_CMD_GET_NEXT);
		lan9355_write_switch_reg(chip, LAN9355_SWE_ALR_CMD, 0);
	}
	mutex_unlock(&chip->alr_mutex);
}

static void alr_reg_to_mac(u32 dat0, u32 dat1, u8 mac[6])
{
	mac[0] = (dat0 >>  0) & 0xff;
	mac[1] = (dat0 >>  8) & 0xff;
	mac[2] = (dat0 >> 16) & 0xff;
	mac[3] = (dat0 >> 24) & 0xff;
	mac[4] = (dat1 >>  0) & 0xff;
	mac[5] = (dat1 >>  8) & 0xff;
}

struct del_port_learned_ctx {
	int port;
};

/* Clear learned (non-static) entry on given port */
static void alr_loop_cb_del_port_learned(struct lan9303 *chip, u32 dat0,
					 u32 dat1, int portmap, void *ctx)
{
	struct del_port_learned_ctx *del_ctx = ctx;
	int port = del_ctx->port;

	if (((BIT(port) & portmap) == 0) || (dat1 & LAN9355_ALR_DAT1_STATIC))
		return;

	/* learned entries has only one port, we can just delete */
	dat1 &= ~LAN9355_ALR_DAT1_VALID; /* delete entry */
	lan9355_alr_make_entry_raw(chip, dat0, dat1);
}

struct port_fdb_dump_ctx {
	int port;
	void *data;
	dsa_fdb_dump_cb_t *cb;
};

static void alr_loop_cb_fdb_port_dump(struct lan9303 *chip, u32 dat0,
				      u32 dat1, int portmap, void *ctx)
{
	struct port_fdb_dump_ctx *dump_ctx = ctx;
	u8 mac[ETH_ALEN];
	bool is_static;

	if ((BIT(dump_ctx->port) & portmap) == 0)
		return;

	alr_reg_to_mac(dat0, dat1, mac);
	is_static = !!(dat1 & LAN9355_ALR_DAT1_STATIC);
	dump_ctx->cb(mac, 0, is_static, dump_ctx->data);
}


/* Delete static port from ALR entry, delete entry if last port */
static int lan9355_alr_del_port(struct lan9303 *chip, const u8 *mac, int port)
{
	struct lan9355_alr_cache_entry *entr;

	mutex_lock(&chip->alr_mutex);
	entr = lan9355_alr_cache_find_mac(chip, mac);
	if (!entr)
		goto out;  /* no static entry found */

	entr->port_map &= ~BIT(port);
	if (entr->port_map == 0) /* zero means its free again */
		eth_zero_addr(entr->mac_addr);
	lan9355_alr_set_entry(chip, mac, entr->port_map, entr->stp_override);

out:
	mutex_unlock(&chip->alr_mutex);
	return 0;
}

static int lan9355_disable_processing_port(struct lan9303 *chip,
					   unsigned int port)
{
	int ret;

	/* disable RX, but keep register reset default values else */
	ret = lan9355_write_switch_port(chip, port, LAN9355_MAC_RX_CFG_0,
					LAN9355_MAC_RX_CFG_X_REJECT_MAC_TYPES);
	if (ret)
		return ret;

	/* disable TX, but keep register reset default values else */
	return lan9355_write_switch_port(chip, port, LAN9355_MAC_TX_CFG_0,
				LAN9355_MAC_TX_CFG_X_TX_IFG_CONFIG_DEFAULT |
				LAN9355_MAC_TX_CFG_X_TX_PAD_ENABLE);
}

static int lan9355_enable_processing_port(struct lan9303 *chip,
					  unsigned int port)
{
	int ret;

	/* enable RX and keep register reset default values else */
	ret = lan9355_write_switch_port(chip, port, LAN9355_MAC_RX_CFG_0,
					LAN9355_MAC_RX_CFG_X_REJECT_MAC_TYPES |
					LAN9355_MAC_RX_CFG_X_RX_ENABLE);
	if (ret)
		return ret;

	/* enable TX and keep register reset default values else */
	return lan9355_write_switch_port(chip, port, LAN9355_MAC_TX_CFG_0,
				LAN9355_MAC_TX_CFG_X_TX_IFG_CONFIG_DEFAULT |
				LAN9355_MAC_TX_CFG_X_TX_PAD_ENABLE |
				LAN9355_MAC_TX_CFG_X_TX_ENABLE);
}





static void lan9355_handle_reset(struct lan9303 *chip)
{
	if (!chip->reset_gpio)
		return;

	if (chip->reset_duration != 0)
		msleep(chip->reset_duration);

	/* release (deassert) reset and activate the device */
	gpiod_set_value_cansleep(chip->reset_gpio, 0);
}

/* stop processing packets for all ports */
static int lan9355_disable_processing(struct lan9303 *chip)
{
	int p;

	for (p = 1; p < LAN9355_NUM_PORTS; p++) {
		int ret = lan9355_disable_processing_port(chip, p);

		if (ret)
			return ret;
	}

	return 0;
}

static int lan9355_check_device(struct lan9303 *chip)
{
	int ret;
	u32 reg;

	ret = lan9355_read(chip->regmap, LAN9355_CHIP_REV, &reg);
	if (ret) {
		dev_err(chip->dev, "failed to read chip revision register: %d\n",
			ret);
		if (!chip->reset_gpio) {
			dev_dbg(chip->dev,
				"hint: maybe failed due to missing reset GPIO\n");
		}
		return ret;
	}

	if ((reg >> 16) != LAN9355_CHIP_ID) {
		dev_err(chip->dev, "expecting LAN9303 chip, but found: %X\n",
			reg >> 16);
		return -ENODEV;
	}

	/* The default state of the LAN9303 device is to forward packets between
	 * all ports (if not configured differently by an external EEPROM).
	 * The initial state of a DSA device must be forwarding packets only
	 * between the external and the internal ports and no forwarding
	 * between the external ports. In preparation we stop packet handling
	 * at all for now until the LAN9303 device is re-programmed accordingly.
	 */
	ret = lan9355_disable_processing(chip);
	if (ret)
		dev_warn(chip->dev, "failed to disable switching %d\n", ret);

	dev_info(chip->dev, "Found LAN9303 rev. %u\n", reg & 0xffff);

	ret = lan9355_detect_phy_setup(chip);
	if (ret) {
		dev_err(chip->dev,
			"failed to discover phy bootstrap setup: %d\n", ret);
		return ret;
	}

	return 0;
}

/* ---------------------------- DSA -----------------------------------*/

static enum dsa_tag_protocol lan9355_get_tag_protocol(struct dsa_switch *ds,
						      int port)
{
	return DSA_TAG_PROTO_LAN9303;
}

static int lan9355_setup(struct dsa_switch *ds)
{
	struct lan9303 *chip = ds->priv;
	int ret;

	/* Make sure that port 0 is the cpu port */
	if (!dsa_is_cpu_port(ds, 0)) {
		dev_err(chip->dev, "port 0 is not the CPU port\n");
		return -EINVAL;
	}

	ret = lan9355_setup_tagging(chip);
	if (ret)
		dev_err(chip->dev, "failed to setup port tagging %d\n", ret);

	ret = lan9355_separate_ports(chip);
	if (ret)
		dev_err(chip->dev, "failed to separate ports %d\n", ret);

	ret = lan9355_enable_processing_port(chip, 0);
	if (ret)
		dev_err(chip->dev, "failed to re-enable switching %d\n", ret);

	/* Trap IGMP to port 0 */
	ret = lan9355_write_switch_reg_mask(chip, LAN9355_SWE_GLB_INGRESS_CFG,
					    LAN9355_SWE_GLB_INGR_IGMP_TRAP |
					    LAN9355_SWE_GLB_INGR_IGMP_PORT(0),
					    LAN9355_SWE_GLB_INGR_IGMP_PORT(1) |
					    LAN9355_SWE_GLB_INGR_IGMP_PORT(2));
	if (ret)
		dev_err(chip->dev, "failed to setup IGMP trap %d\n", ret);

	return 0;
}

struct lan9355_mib_desc {
	unsigned int offset; /* offset of first MAC */
	const char *name;
};

static int lan9355_phy_read(struct dsa_switch *ds, int phy, int regnum)
{
	struct lan9303 *chip = ds->priv;
	int phy_base = chip->phy_addr_base;

	if (phy == phy_base)
		return lan9355_virt_phy_reg_read(chip, regnum);
	if (phy > phy_base + 2)
		return -ENODEV;

	return chip->ops->phy_read(chip, phy, regnum);
}

static int lan9355_phy_write(struct dsa_switch *ds, int phy, int regnum,
			     u16 val)
{
	struct lan9303 *chip = ds->priv;
	int phy_base = chip->phy_addr_base;

	if (phy == phy_base)
		return lan9355_virt_phy_reg_write(chip, regnum, val);
	if (phy > phy_base + 2)
		return -ENODEV;

	return chip->ops->phy_write(chip, phy, regnum, val);
}

static void lan9355_adjust_link(struct dsa_switch *ds, int port,
				struct phy_device *phydev)
{
	struct lan9303 *chip = ds->priv;
	int ctl, res;

	if (!phy_is_pseudo_fixed_link(phydev))
		return;

	ctl = lan9355_phy_read(ds, port, MII_BMCR);

	ctl &= ~BMCR_ANENABLE;

	if (phydev->speed == SPEED_100)
		ctl |= BMCR_SPEED100;
	else if (phydev->speed == SPEED_10)
		ctl &= ~BMCR_SPEED100;
	else
		dev_err(ds->dev, "unsupported speed: %d\n", phydev->speed);

	if (phydev->duplex == DUPLEX_FULL)
		ctl |= BMCR_FULLDPLX;
	else
		ctl &= ~BMCR_FULLDPLX;

	res =  lan9355_phy_write(ds, port, MII_BMCR, ctl);

	if (port == chip->phy_addr_base) {
		/* Virtual Phy: Remove Turbo 200Mbit mode */
		lan9355_read(chip->regmap, LAN9355_VIRT_SPECIAL_CTRL, &ctl);

		ctl &= ~LAN9355_VIRT_SPECIAL_TURBO;
		res =  regmap_write(chip->regmap,
				    LAN9355_VIRT_SPECIAL_CTRL, ctl);
	}
}

static int lan9355_port_enable(struct dsa_switch *ds, int port,
			       struct phy_device *phy)
{
	struct lan9303 *chip = ds->priv;

	return lan9355_enable_processing_port(chip, port);
}

static void lan9355_port_disable(struct dsa_switch *ds, int port,
				 struct phy_device *phy)
{
	struct lan9303 *chip = ds->priv;

	lan9355_disable_processing_port(chip, port);
	lan9355_phy_write(ds, chip->phy_addr_base + port, MII_BMCR, BMCR_PDOWN);
}

static int lan9355_port_bridge_join(struct dsa_switch *ds, int port,
				    struct net_device *br)
{
	struct lan9303 *chip = ds->priv;

	dev_dbg(chip->dev, "%s(port %d)\n", __func__, port);
	if (dsa_to_port(ds, 1)->bridge_dev == dsa_to_port(ds, 2)->bridge_dev) {
		lan9355_bridge_ports(chip);
		chip->is_bridged = true;  /* unleash stp_state_set() */
	}

	return 0;
}

static void lan9355_port_bridge_leave(struct dsa_switch *ds, int port,
				      struct net_device *br)
{
	struct lan9303 *chip = ds->priv;

	dev_dbg(chip->dev, "%s(port %d)\n", __func__, port);
	if (chip->is_bridged) {
		lan9355_separate_ports(chip);
		chip->is_bridged = false;
	}
}

static void lan9355_port_stp_state_set(struct dsa_switch *ds, int port,
				       u8 state)
{
	int portmask, portstate;
	struct lan9303 *chip = ds->priv;

	dev_dbg(chip->dev, "%s(port %d, state %d)\n",
		__func__, port, state);

	switch (state) {
	case BR_STATE_DISABLED:
		portstate = LAN9355_SWE_PORT_STATE_DISABLED_PORT0;
		break;
	case BR_STATE_BLOCKING:
	case BR_STATE_LISTENING:
		portstate = LAN9355_SWE_PORT_STATE_BLOCKING_PORT0;
		break;
	case BR_STATE_LEARNING:
		portstate = LAN9355_SWE_PORT_STATE_LEARNING_PORT0;
		break;
	case BR_STATE_FORWARDING:
		portstate = LAN9355_SWE_PORT_STATE_FORWARDING_PORT0;
		break;
	default:
		portstate = LAN9355_SWE_PORT_STATE_DISABLED_PORT0;
		dev_err(chip->dev, "unknown stp state: port %d, state %d\n",
			port, state);
	}

	portmask = 0x3 << (port * 2);
	portstate <<= (port * 2);

	chip->swe_port_state = (chip->swe_port_state & ~portmask) | portstate;

	if (chip->is_bridged)
		lan9355_write_switch_reg(chip, LAN9355_SWE_PORT_STATE,
					 chip->swe_port_state);
	/* else: touching SWE_PORT_STATE would break port separation */
}

static void lan9355_port_fast_age(struct dsa_switch *ds, int port)
{
	struct lan9303 *chip = ds->priv;
	struct del_port_learned_ctx del_ctx = {
		.port = port,
	};

	dev_dbg(chip->dev, "%s(%d)\n", __func__, port);
	lan9355_alr_loop(chip, alr_loop_cb_del_port_learned, &del_ctx);
}

static int lan9355_port_fdb_add(struct dsa_switch *ds, int port,
				const unsigned char *addr, u16 vid)
{
	struct lan9303 *chip = ds->priv;

	dev_dbg(chip->dev, "%s(%d, %pM, %d)\n", __func__, port, addr, vid);
	if (vid)
		return -EOPNOTSUPP;

	return lan9355_alr_add_port(chip, addr, port, false);
}

static int lan9355_port_fdb_del(struct dsa_switch *ds, int port,
				const unsigned char *addr, u16 vid)

{
	struct lan9303 *chip = ds->priv;

	dev_dbg(chip->dev, "%s(%d, %pM, %d)\n", __func__, port, addr, vid);
	if (vid)
		return -EOPNOTSUPP;
	lan9355_alr_del_port(chip, addr, port);

	return 0;
}

static int lan9355_port_fdb_dump(struct dsa_switch *ds, int port,
				 dsa_fdb_dump_cb_t *cb, void *data)
{
	struct lan9303 *chip = ds->priv;
	struct port_fdb_dump_ctx dump_ctx = {
		.port = port,
		.data = data,
		.cb   = cb,
	};

	dev_dbg(chip->dev, "%s(%d)\n", __func__, port);
	lan9355_alr_loop(chip, alr_loop_cb_fdb_port_dump, &dump_ctx);

	return 0;
}

static int lan9355_port_mdb_prepare(struct dsa_switch *ds, int port,
				    const struct switchdev_obj_port_mdb *mdb)
{
	struct lan9303 *chip = ds->priv;

	dev_dbg(chip->dev, "%s(%d, %pM, %d)\n", __func__, port, mdb->addr,
		mdb->vid);
	if (mdb->vid)
		return -EOPNOTSUPP;
	if (lan9355_alr_cache_find_mac(chip, mdb->addr))
		return 0;
	if (!lan9355_alr_cache_find_free(chip))
		return -ENOSPC;

	return 0;
}

static void lan9355_port_mdb_add(struct dsa_switch *ds, int port,
				 const struct switchdev_obj_port_mdb *mdb)
{
	struct lan9303 *chip = ds->priv;

	dev_dbg(chip->dev, "%s(%d, %pM, %d)\n", __func__, port, mdb->addr,
		mdb->vid);
	lan9355_alr_add_port(chip, mdb->addr, port, false);
}

static int lan9355_port_mdb_del(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_mdb *mdb)
{
	struct lan9303 *chip = ds->priv;

	dev_dbg(chip->dev, "%s(%d, %pM, %d)\n", __func__, port, mdb->addr,
		mdb->vid);
	if (mdb->vid)
		return -EOPNOTSUPP;
	lan9355_alr_del_port(chip, mdb->addr, port);

	return 0;
}

static const struct dsa_switch_ops lan9355_switch_ops = {
	.get_tag_protocol = lan9355_get_tag_protocol,
	.setup = lan9355_setup,
	.get_strings = lan9355_get_strings,
	.phy_read = lan9355_phy_read,
	.phy_write = lan9355_phy_write,
	.adjust_link = lan9355_adjust_link,
	.get_ethtool_stats = lan9355_get_ethtool_stats,
	.get_sset_count = lan9355_get_sset_count,
	.port_enable = lan9355_port_enable,
	.port_disable = lan9355_port_disable,
	.port_bridge_join       = lan9355_port_bridge_join,
	.port_bridge_leave      = lan9355_port_bridge_leave,
	.port_stp_state_set     = lan9355_port_stp_state_set,
	.port_fast_age          = lan9355_port_fast_age,
	.port_fdb_add           = lan9355_port_fdb_add,
	.port_fdb_del           = lan9355_port_fdb_del,
	.port_fdb_dump          = lan9355_port_fdb_dump,
	.port_mdb_prepare       = lan9355_port_mdb_prepare,
	.port_mdb_add           = lan9355_port_mdb_add,
	.port_mdb_del           = lan9355_port_mdb_del,
};

static int lan9355_register_switch(struct lan9303 *chip)
{
	int base;

	chip->ds = dsa_switch_alloc(chip->dev, LAN9355_NUM_PORTS);
	if (!chip->ds)
		return -ENOMEM;

	chip->ds->priv = chip;
	chip->ds->ops = &lan9355_switch_ops;
	base = chip->phy_addr_base;
	chip->ds->phys_mii_mask = GENMASK(LAN9355_NUM_PORTS - 1 + base, base);

	return dsa_register_switch(chip->ds);
}

static int lan9355_probe_reset_gpio(struct lan9303 *chip,
				     struct device_node *np)
{
	chip->reset_gpio = devm_gpiod_get_optional(chip->dev, "reset",
						   GPIOD_OUT_LOW);
	if (IS_ERR(chip->reset_gpio))
		return PTR_ERR(chip->reset_gpio);

	if (!chip->reset_gpio) {
		dev_dbg(chip->dev, "No reset GPIO defined\n");
		return 0;
	}

	chip->reset_duration = 200;

	if (np) {
		of_property_read_u32(np, "reset-duration",
				     &chip->reset_duration);
	} else {
		dev_dbg(chip->dev, "reset duration defaults to 200 ms\n");
	}

	/* A sane reset duration should not be longer than 1s */
	if (chip->reset_duration > 1000)
		chip->reset_duration = 1000;

	return 0;
}

int lan9355_probe(struct lan9303 *chip, struct device_node *np)
{
	int ret;

	mutex_init(&chip->indirect_mutex);
	mutex_init(&chip->alr_mutex);

	ret = lan9355_probe_reset_gpio(chip, np);
	if (ret)
		return ret;

	lan9355_handle_reset(chip);

	ret = lan9355_check_device(chip);
	if (ret)
		return ret;

	ret = lan9355_register_switch(chip);
	if (ret) {
		dev_dbg(chip->dev, "Failed to register switch: %d\n", ret);
		return ret;
	}

	return 0;
}
#endif
