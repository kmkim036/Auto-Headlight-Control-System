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
/* [NOTE] Using basic LIN functions.
 *
	// LIN Master Controller Using UART1 of MCU Board
	PD3 : U1TX
	PD2 : U1RX
    PD5 : LIN EN
  The Slave of AMIS30621 with Bipolar DC Stepper Motor (12V Version)
 * It is directly drived by OnSemi's AMIS30621 chip
 * The AMIS30621 works with
 * Logic = 5V
 * Driver = 12V (H1 selectable)

   5V/12V 4 Pins Bipolar Stepper Motor
 * Org : Pin 2 : 1A
 * Yel : Pin 3 : 1B
 * Pnk : Pin 4 : 2A
 * Blu : Pin 5 : 2B
 *
*/

#define LINbaudrate 20000 //20Kbps
#define LINbaudrate 19200 //19.2Kbps

#define LIN_ROLE_MASTER 1
#define LIN_ROLE_SLAVE 0



#define MotorDirectionCW 0x01
#define MotorDirectionCCW 0x02
#define MotorSpeedUP 0x03
#define MotorSpeedDOWN 0x04

/*This function uses a master request frame to get two response frames from the AMIS Device.
 * The Data is stored in LIN_Data[0 to 7] and LIN_Data[8 to 15].
*/
#define LIN_MsgID_0_8_PREP_PUT_3C 0x3C //  (MSB)00(parity)-11-1100(LSB) : 8byte data (ID=0x3C)  -- Diagnostic/Config Frame (For Wakeup)
#define LIN_DIAG_CONFIG_RESP_MSG_7D  0x7D  //      01(parity)-11-1101      : 8byte data (ID=0x3D)  -- Diag/Config Resp Frame

#define AMIS_AppCMD_80 0x80
#define SleepREQ 0x00
#define MsgID_0 0x11 //for testing Header-->Response transaction (Button ON/OFFÃƒÂ¬Ã†â€™Ã¯Â¿Â½ÃƒÂ­Ã†â€™Ã…â€œ ÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂªÃ‚Â³Ã‚Â )
#define MsgID_1 0x12 //for controling Motor (Master: transmiting Header and Data)
#define MsgID_0_8_PREP_PUT 	0x3C  //  (MSB)00(parity)-11-1100(LSB) : 8byte data (ID=0x3C)  -- Diagnostic/Config Frame (For Wakeup)
#define MsgID_0_8_GET 		0x7D //       01(parity)-11-1101      : 8byte data (ID=0x3D)  -- Diag/Config Resp Frame

typedef enum {PrepPutMsg0=0, GetMsg1=1} AmsiLinMsgIndex;

//struct LinMsgInfo g_AmisLinMsgInfoPool[MAX_MSG_NUM]={
//		{0, MsgID_0_8_PREP_PUT,  0, 8}, //9=8+1(crc)
//		{1, MsgID_0_8_GET,       1, 8}
//};
//#define ArbitrationId 0x3c

//READING CMDs
#define AmisGetActualPos_00 0x00     //Prep # 7,8; Read # 5,6
#define AmisGetFullStatusCMD_81 0x01 | 0x80 //Prep # 7,8; Read # 6//0x81
#define AmisGetStatusCMD_03 0x03     // Read # 5 ; Read # 5
#define AmisGetOtpParamCMD  (0x02 | 0x80)    //Prep #8

//WRITING CMDs
#define AmisHardStopCMD_05 0x05         //Write #1
#define AmisResetPosition_06 0x06    //Write #1
#define RESETTODEFAULT 0x07   //Write #1
#define SETMOTORPARAM 0x09    //Write #4
#define SETPOSITION 0x0B      //Write #1,3,4
#define SOFTSLEEP       0x0F   //Write #1
#define DYNAMICIDASSIGNMENT 0x11 //Write #1
//#define PREPARINGFRAME 0x

#define ST_IDLE   	  0
#define ST_SEND_REQ   1
#define ST_WAIT_RESP 2
//#define ST_WAIT_RESP2 3

#define Rx_Error 8
#define AMIS30621MAXTIMEOUT 6000000 //may be about 3msec
typedef u8 BYTE;
//u8 LIN_Data[8];

//

extern void stmLinNodeInit(unsigned char Role); //stmLin.c

struct _stmLINmodule{
	u8 NodeAddressNAD7; //physical address
	u8 NodeAddressNAD8;  //physical address
	u8 replyCount;
	u8 LIN_State;
	u8 Motion;
	unsigned long timeOut;
	char detectBreak;
	char detectSync;
	unsigned char role;
	unsigned short bufPos;
	unsigned char detectMsgid;
	unsigned char rxMsgDone;
	unsigned char LinMsgByteArray[100];//TBD
	unsigned char AppRxBuf16[16];
};
volatile struct _stmLINmodule stmLINmodule;

extern struct usartbuf_st usart1_rbuf;//={0,0,0,0,};

//extern void LinSendDataByMaster(unsigned char msgid, unsigned char controlData);
//extern void doMotorControl(unsigned char controldata);
//extern void UART1_IntHandlerForLIN(void);
extern void stmLinSendHeader (unsigned char linMsgId);
extern unsigned char stmLinCksum(unsigned char lindatabyte, unsigned char lincksum);
extern void LinMasterInit(unsigned char Role);

void stmAMIS30621LinGotoSleep(){ //Make EN Off. You will see the transition of INH Output.
	GPIO_ResetBits(GPIOB, GPIO_Pin_10); //EN=0:
}

void stmAMIS30621LinGotoWakeup(){ //Make EN On. You will see the transition of INH Output.
	GPIO_SetBits(GPIOB, GPIO_Pin_10);//EN=1:
}
extern unsigned char stmGetLinCksum(unsigned char lindatabyte[]);

void  stmAmisAssemble_MsgWithMsgId_AND_8ByteData(u8 id, u8 B1, u8 B2, u8 B3, u8 B4, u8 B5, u8 B6, u8 B7, u8 B8) {
	stmLINmodule.LinMsgByteArray[0] = id; //Msg Header (0x3c or 0x7d)
	stmLINmodule.LinMsgByteArray[1] = B1; //AMIS_AppCMD_80;
	stmLINmodule.LinMsgByteArray[2] = B2;
	stmLINmodule.LinMsgByteArray[3] = B3;
	stmLINmodule.LinMsgByteArray[4] = B4;
	stmLINmodule.LinMsgByteArray[5] = B5;
	stmLINmodule.LinMsgByteArray[6] = B6;
	stmLINmodule.LinMsgByteArray[7] = B7;
	stmLINmodule.LinMsgByteArray[8] = B8;
	stmLINmodule.LinMsgByteArray[9] = stmGetLinCksum(&(stmLINmodule.LinMsgByteArray[1]));
	//stmLINmodule.LinMsgByteArray[9] = crc;
}
//As a master, we first send header(with Sync), and then 8 bytes of data, and finally crc.
void stmAmisSendLinMsg(u8 datalen){ //By Master
	int i=0;
	volatile unsigned char cksum = 0;

	stmLinSendHeader(stmLINmodule.LinMsgByteArray[0]); //Header with SYNC+ msg id(datalen=8)
	//somedelay(400); //Inter-frame space
	//1..8== AppCmd | Cmd | Ad | ....
	//9 == Cksum
	for(i=1;i<=(datalen);i++){ //datalen + 1
		//cksum = stmLinCksum(stmLINmodule.LinMsgByteArray[i], cksum); //calc during response data transmission.
		USART_SendData(USART1, stmLINmodule.LinMsgByteArray[i]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}
	}
	//After sending id and the whole data of 8 bytes, and finally append Checksum
	//printf("cksum = %02x\r\n",cksum);
	//cksum = 0x0d;
	USART_SendData(USART1, stmLINmodule.LinMsgByteArray[9]);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}
	printf("Master> Sent LinMsg with cksum = 0x%02x.\r\n",stmLINmodule.LinMsgByteArray[9]);
}


void stmAmisLinParser(u8 frmPos){
	u8 i;
	u8 crc;
	u8 pos;
	pos = frmPos*10;

	if(stmLINmodule.rxMsgDone){
		stmLINmodule.rxMsgDone = 0;
		printf("Parser>RxDone.\r\n");
		//need crc check.
		printf("Parser>MsgId =(0x%02x)\r\n", stmLINmodule.LinMsgByteArray[0+pos]);
		for(i=1;i<=8;i++){
			printf("Parser>data byte [%d]=(0x%02x)\r\n", i, stmLINmodule.LinMsgByteArray[i+pos]);
		}
		crc = stmLINmodule.LinMsgByteArray[i+pos];
		printf("Parser>CRC byte [%d]=(0x%02x)\r\n", i, crc);
		//TBD

	}else if(stmLINmodule.bufPos > 0){
		printf("Parser> data byte[0]=(0x%02x)\r\n", stmLINmodule.LinMsgByteArray[0+pos]);
	}
}

u8  stmAmisGetOTPparam()
{
	u8 ret;
	u8 txD[8];
	struct LinMsgInfo *plinMsgInfo;

	printf("<GetOTPparam>\r\n");
	//(1) Send a single preparing frame
	stmLINmodule.replyCount = 0;
	stmAmisAssemble_MsgWithMsgId_AND_8ByteData(LIN_MsgID_0_8_PREP_PUT_3C,AMIS_AppCMD_80,AmisGetOtpParamCMD,stmLINmodule.NodeAddressNAD8,0xFF,0xFF,0xFF,0xFF,0xFF);
	stmLINmodule.LIN_State = ST_SEND_REQ;
	stmAmisSendLinMsg(8);
	//delayms(2);
	//========== (2) Get the Response //Type #6(1)
	//type#6 msg
	stmLINmodule.replyCount = 0;
	stmLINmodule.rxMsgDone = 0;
	stmLINmodule.LIN_State = ST_SEND_REQ;
	stmLinSendHeader(LIN_DIAG_CONFIG_RESP_MSG_7D); // Actually, 0x3c. According to Parity Bits => this becomes 0x7D. Master wants data from slave
	stmLINmodule.LIN_State = ST_WAIT_RESP;
	stmLINmodule.timeOut = 0;
	while  ((!stmLINmodule.rxMsgDone) && (stmLINmodule.timeOut < AMIS30621MAXTIMEOUT))
		stmLINmodule.timeOut++;
	if  (stmLINmodule.timeOut < AMIS30621MAXTIMEOUT){
		//printf("...GetOTPparam2()\r\n");
		memcpy(&stmLINmodule.AppRxBuf16[0], &stmLINmodule.LinMsgByteArray[1], 8);
		stmAmisLinMsgAppHandler(AmisGetOtpParamCMD,8);// all information stored in LIN_Data[] now
		stmLINmodule.timeOut = 0; //got it.
		return 1;
	}else{
		printf("Err> No Response of GetOTPparam(%u)\r\n", stmLINmodule.rxMsgDone);
		return Rx_Error;
	}
}

//This function is used to set the motor parameters.
//According to the AMIS Datasheet some //data bytes contain more than one parameter.
//These are placed to the appropriate Bit positions.
//At the end all parameter are send via one LIN frame.


u8 stmAmisSetMotorParameter(u8 adress, u8 Irun, u8 Ihold, BYTE Vmax, BYTE Vmin, u16 SecPos11, u8 Shaft, u8 Acc, u8 AccShape, u8 StepMode) {
	u8 data4, data5, data6, data7, data8;
	if  ((stmLINmodule.Motion & 0x03) > 0) return 0;  // if motor is in motion break
	else {
		data4 = (Irun << 4) + Ihold;
		data5 = (Vmax << 4) + Vmin;
		data6 = (BYTE)((SecPos11 & 0x0700) >> 3) + (Shaft << 4)+ (Acc & 0x0F);
		data7 = (BYTE)(SecPos11 & 0x00FF);
		data8 = 0xE3 + (AccShape<<4) + (StepMode << 2);
		stmAmisAssemble_MsgWithMsgId_AND_8ByteData(
				LIN_MsgID_0_8_PREP_PUT_3C,
				AMIS_AppCMD_80,
				SETMOTORPARAM | 0x80,
				adress,
				data4,
				data5,
				data6,
				data7,
				data8);
		stmLINmodule.replyCount = 0;
		stmLINmodule.LIN_State = ST_SEND_REQ;
		stmAmisSendLinMsg(8);//Start_LIN_Message();
		//while  (LIN_State > 0);  // process complete
		return  1;
	}
}
// To leave from the shutdown mode, we must send this GetFullStatus.
// Need 1 preparing frame and 2 response frames with 0x3D.
u8  stmAmisGetFullStatus ()
{
	printf("GetFullStatus\r\n");
	//(1) Send Prepairing Frame(Type #8) (msgid = 0x3c)
	stmLINmodule.LIN_State = ST_SEND_REQ;

	stmAmisAssemble_MsgWithMsgId_AND_8ByteData(LIN_MsgID_0_8_PREP_PUT_3C, AMIS_AppCMD_80,AmisGetFullStatusCMD_81,stmLINmodule.NodeAddressNAD8,0xFF,0xFF,0xFF,0xFF,0xFF);//Type #8 or Type#7(Preparing)
	stmAmisSendLinMsg(8);//cksum should be 0x0d.
	//disregards the readbacked msg while my transmitting.
	stmLINmodule.rxMsgDone = 0;
	stmLINmodule.bufPos = 0;
	stmLINmodule.timeOut = 0;
	delayms(1); //somedelay(10000);

	//(2) Issue First ResponseRequest : Type #6(1)
	//printf("Send 1st Response Request\r\n");
	stmLINmodule.replyCount = 0;
	stmLINmodule.rxMsgDone = 0;
	stmLINmodule.LIN_State = ST_SEND_REQ;
	stmLinSendHeader(LIN_DIAG_CONFIG_RESP_MSG_7D); // Actually, 0x3c. According to Parity Bits => this becomes 0x7D. Master wants data from slave
	stmLINmodule.LIN_State = ST_WAIT_RESP;

	//The response will be handled in ISR
	stmLINmodule.timeOut = 0;
	while  ((!stmLINmodule.rxMsgDone) && (stmLINmodule.timeOut < AMIS30621MAXTIMEOUT))
		stmLINmodule.timeOut++;
	if  (stmLINmodule.timeOut < AMIS30621MAXTIMEOUT){
		//printf("Got Response1 (in time=%u)\r\n",stmLINmodule.timeOut);
		memcpy(&stmLINmodule.AppRxBuf16[0], &stmLINmodule.LinMsgByteArray[1], 8);
		stmLINmodule.timeOut = 0; //got it.
	}else{
		printf("Err> No Response1(%u)\r\n", stmLINmodule.rxMsgDone);
		return Rx_Error;
	}
	stmLINmodule.LIN_State = ST_IDLE;
	//stmAmisLinParser(0);
	stmLINmodule.rxMsgDone = 0;
	delayms(3);

	//(3)Issue 2nd ResponseRequest
	//Type #6(2)
	//printf("Send 2nd Response Request\r\n");
	stmLINmodule.replyCount = 1;
	stmLINmodule.timeOut = 0;
	stmLINmodule.rxMsgDone = 0;
	stmLINmodule.LIN_State = ST_SEND_REQ;
	stmLinSendHeader(LIN_DIAG_CONFIG_RESP_MSG_7D);   // Master wants data from slave
	stmLINmodule.LIN_State = ST_WAIT_RESP;
	//The response will be handled in ISR
	while  ((!stmLINmodule.rxMsgDone) && (stmLINmodule.timeOut < AMIS30621MAXTIMEOUT)) stmLINmodule.timeOut++; //may be isr change stmLINmodule.rxMsgDone...
	if  (stmLINmodule.timeOut < AMIS30621MAXTIMEOUT){
		//printf("Got the Response2 (in time=%u)\r\n",stmLINmodule.timeOut);
		memcpy(&stmLINmodule.AppRxBuf16[8], &stmLINmodule.LinMsgByteArray[1], 8);
		stmLINmodule.timeOut = 0; //got it.
	}else{
		printf("Err> No Response of 2(%u)\r\n", stmLINmodule.rxMsgDone);
		return Rx_Error;
	}

	stmAmisLinMsgAppHandler(AmisGetFullStatusCMD_81,16);

	stmLINmodule.LIN_State = ST_IDLE;
	//stmAmisLinParser(0);//1);// all information stored in LIN_Data[] now
	stmLINmodule.rxMsgDone = 0;

	return 1;
}
u8  stmAMIS30621IssueSleep () { //0x3c-0x00-0xff-0xff-...0xff Version1.2
	stmAmisAssemble_MsgWithMsgId_AND_8ByteData(LIN_MsgID_0_8_PREP_PUT_3C, SleepREQ,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF);//
	stmAmisSendLinMsg(8);
	stmLINmodule.rxMsgDone = 0;
	return 1;
}



//In the example four dynamic Identifiers are used:
// GetStatus, SetPosition (16-bit), General Purpose 2 Data bytes, and General Purpose 4 Data bytes.
// The following source code shows how these IDs are linked to the appropriate ROM pointers
void  stmAMIS30621Set_Dynamic_ID() {
	stmLINmodule.LIN_State = ST_SEND_REQ;
	stmAmisAssemble_MsgWithMsgId_AND_8ByteData(
			LIN_MsgID_0_8_PREP_PUT_3C, //MsgID
			AMIS_AppCMD_80,//1
			DYNAMICIDASSIGNMENT | 0x80,//2 0x80 | CMD[6:0] = 0x80 | 0x11 = 0x81
			0x00,//3 NodeAddressNAD8 = 0, (Bit 7 = 0 : broadcast), AD[6..0] = 0x00
			0x81,	//4 ID1[3:0] | ROMp1[3:0] = 1000 | 0001
			0x0e,	//5 ID2[1:0] | ROMp2[3:0] | ID1[5:4]=00|0011|10    	::::ID1(101000,0x28) <-> ROMp1(0001) = General Purpose 4 Data Bytes
			0x40,	//6 ROMp3[3:0] | ID2[5:2] = 0100 | 0000 			::::ID2(000000,0x00) <-> ROMp2(0011) = GetStatus
			0x10,	//7 ROMp4[1:0] | ID3[5:0] = 00 | 010000 			::::ID3(010000,0x10) <-> ROMp3(0100) = SetPosition(2byte data)
			0x60); 	//8 ID4[5:0]  | ROMp4[3:2] = 0000 					::::ID4(011000,0x18) <-> ROMp4(0000) = General Purpose 2 Data Bytes
/*
	stmLINmodule.LinMsgByteArray[1] = AMIS_AppCMD_80
	stmLINmodule.LinMsgByteArray[2] = 0x80;  //CMD = 0x80 | 0x11 DYNAMICIDASSIGNMENT;
	stmLINmodule.LinMsgByteArray[3] = 0x81;  //NodeAddressNAD8 = 0, non broad??(boradcast?)
	stmLINmodule.LinMsgByteArray[4] = 0x0E;  //ID1 = 101000 ROMp1 = 0001
	stmLINmodule.LinMsgByteArray[5] = 0x40;  //ID2 = 000000 ROMp2 = 0011
	stmLINmodule.LinMsgByteArray[6] = 0x10;  //ID3 = 010000 ROMp3 = 0100
	stmLINmodule.LinMsgByteArray[7] = 0x60;  //ID4 = 011000 ROMp4 = 0000
*/
	stmLINmodule.replyCount = 0;
	stmAmisSendLinMsg(8); //Start_LIN_Message();
	//TBD -- the followings
	while  ((stmLINmodule.LIN_State>0) && (stmLINmodule.timeOut < AMIS30621MAXTIMEOUT)) stmLINmodule.timeOut++;
	if  (stmLINmodule.timeOut < AMIS30621MAXTIMEOUT) stmLINmodule.timeOut = 0;
	else  return Rx_Error;
}

//This function is used to set the motor positions using type#4 msg.
u8 stmAmisSetPosition(u8 motorAddr1, u16 Pos1, BYTE motorAddr2, u16 Pos2)
{
	printf("SetPosition()\r\n");
/*
	stmLINmodule.replyCount = 0;
	stmAmisAssemble_MsgWithMsgId_AND_8ByteData(
			LIN_MsgID_0_8_PREP_PUT_3C, //id
			AMIS_AppCMD_80,//1
			DYNAMICIDASSIGNMENT, //2
			0x80,//3
			0x81,//4
			0x0e,//5
			0x40,//6
			0x10,//7
			0x60);//8
	stmLINmodule.LIN_State = ST_SEND_REQ;
	stmAmisSendLinMsg(8); //Start_LIN_Message();
*/
	stmLINmodule.replyCount = 0;
	stmAmisAssemble_MsgWithMsgId_AND_8ByteData(
			LIN_MsgID_0_8_PREP_PUT_3C,
			AMIS_AppCMD_80,
			SETPOSITION | 0x80,
			motorAddr1 | 0x80,
			Pos1 >> 8,
			Pos1 & 0xff,
			motorAddr2 | 0x80,
			Pos2 >> 8,
			Pos2 & 0xff);
	stmAmisSendLinMsg(8);
	return  1;
}
//You should install this isr in interrupt vector table.
// This ISR will be used for Header Detect and Response.
// +-----------------------------------------------+
// * | 0x7D (SLAVE RESPONSE) : ID                    | Header sent by this master to slave
// * +-----------------------------------------------+
// * | 1 |    NodeAddressNAD7[6:0]                   | 1 Data[1] from slave to master.
// * +-----------------------------------------------+
// * | ...                                           | 2 Data[2] from slave to master.
// * |....                                           + ~7
// * +-----------------------------------------------+
// * |                 0xFF              data[8]     | 8 data[8] for End Frame Delimiter?
// * +-----------------------------------------------+
// * |                 CRC                           | 9
// * +-----------------------------------------------+
//
void stmAmisLinMsgAppHandler(u8 relatedMsg, u8 datalen){
	u8 i;
	printf("AmisLinMsgAppHandler...\r\n");
	//printf("RXDAPP>");
	//for(i=0;i<datalen;i++)		printf("[%d]=0x%02x ", i, stmLINmodule.AppRxBuf16[i]);
	switch(relatedMsg){
	case AmisGetFullStatusCMD_81:
		printf("Resp of GetFullStatus:\r\n");
		printf("\tIrun=%02x  \tIhold=%02x\r\n",(stmLINmodule.AppRxBuf16[1] & 0xf0 )>>4, (stmLINmodule.AppRxBuf16[1] & 0x0f ));
		printf("\tVmax=%2x \tVmin =%02x\r\n",(stmLINmodule.AppRxBuf16[2] & 0xf0 )>>4, (stmLINmodule.AppRxBuf16[2] & 0x0f ));
		printf("\tAccShape=%02x \tStepMode =%02x \tShaft=%02x \tAcc[3:0]=%02x\r\n",(stmLINmodule.AppRxBuf16[3] & 0x80 )>>7, (stmLINmodule.AppRxBuf16[3] & 0x60)>>5, (stmLINmodule.AppRxBuf16[3] & 0x10)>>4,(stmLINmodule.AppRxBuf16[3] & 0x0f));
//		printf("VddReset=%d | SetpLoss =%d | EIDef=%d | UV2 | Acc[3:0]=%x\r\n",(stmLINmodule.AppRxBuf16[3] & 0x80 )>>7, (stmLINmodule.AppRxBuf16[3] & 0x60)>>5, (stmLINmodule.AppRxBuf16[3] & 0x10)>>4,(stmLINmodule.AppRxBuf16[3] & 0x0f));
		printf("\tActPos=%02x \tTargetPos=%02x \tSecPos=%02x\r\n",stmLINmodule.AppRxBuf16[9]*0x100+stmLINmodule.AppRxBuf16[10], stmLINmodule.AppRxBuf16[11]*0x100+stmLINmodule.AppRxBuf16[12],stmLINmodule.AppRxBuf16[13]+ (stmLINmodule.AppRxBuf16[14] & 0x07)*256);

		break;
	case AmisGetOtpParamCMD:
		printf("Resp of GetOTPparam:\r\n");
		printf("\tOSC[3:0]=%x \tIREF[3:0] =%x\r\n",(stmLINmodule.AppRxBuf16[0] & 0xF0 )>>4, (stmLINmodule.AppRxBuf16[0] & 0x0F)>>0);
		printf("\tEnableLIN=%x \tTSD[2:0] =%x, \tBG[3:0] =%x\r\n",(stmLINmodule.AppRxBuf16[1] & 0x80 )>>7, (stmLINmodule.AppRxBuf16[1] & 0x70)>>4, (stmLINmodule.AppRxBuf16[1] & 0x0F)>>0);
		printf("\tADM=%d \tHW[2:0] =0x%02x \tAD[3:0]=0x%x\r\n",(stmLINmodule.AppRxBuf16[2] & 0x80 )>>7, (stmLINmodule.AppRxBuf16[2] & 0x70)>>4, (stmLINmodule.AppRxBuf16[1] & 0x0F)>>0);
		printf("\tIrun[3:0]=%x \tIhold[3:0] =%x\r\n",(stmLINmodule.AppRxBuf16[3] & 0xF0 )>>4, (stmLINmodule.AppRxBuf16[3] & 0x0F)>>0);
		printf("\tVmax[3:0]=%x \tVmin[3:0] =%x\r\n",(stmLINmodule.AppRxBuf16[4] & 0xF0 )>>4, (stmLINmodule.AppRxBuf16[4] & 0x0F)>>0);
		printf("\tSecPos[10:8]=%x \tShaft =%x, \tAcc[3:0]=0x%0x\r\n",(stmLINmodule.AppRxBuf16[5] & 0xE0 )>>5, (stmLINmodule.AppRxBuf16[5] & 0x10 )>>4,(stmLINmodule.AppRxBuf16[5] & 0x0F)>>0);
		printf("\tSecPos[7:0]=%02x \r\n",(stmLINmodule.AppRxBuf16[6]));
		printf("\tStepMode[1:0]=%x, \tLOCKBT=%x, \t LOCKBG=%x \r\n",(stmLINmodule.AppRxBuf16[7]&0x0c)>>2, (stmLINmodule.AppRxBuf16[7]&0x02)>>1,(stmLINmodule.AppRxBuf16[7]&0x01));
		break;
	default:
		break;

	}
}
/*
void USART1_IRQHandler(void) //UART1_IntHandlerForAMIS30621LIN(void) //
{
	struct usartbuf_st *bufp;
	char curchar;
	//static unsigned char cnt = 0;//
	if(USART_GetITStatus(USART1,USART_IT_LBD)){ //line break detect
		USART_ClearITPendingBit(USART1,USART_IT_LBD);
		stmLINmodule.bufPos = 0;
		stmLINmodule.detectBreak=1;
		return;
	}

	// check if the USART1 receive interrupt flag was set
	if(USART_GetITStatus(USART1, USART_IT_RXNE)){ //| USART_IT_ORE_RX

		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//clear IRQ //| USART_IT_ORE_RX
		curchar = USART1->DR; //get char

		if(stmLINmodule.LIN_State != ST_WAIT_RESP){
			return;
		}

		if(stmLINmodule.detectBreak == 0){ //disregard it
   			stmLINmodule.bufPos = 0;
   			stmLINmodule.detectBreak=0;
   			return;
		}
		stmLINmodule.LinMsgByteArray[stmLINmodule.bufPos] = curchar;
		stmLINmodule.bufPos++;
		if(stmLINmodule.bufPos == 9){//TBD (| addr[1] | data[2] |...| data[8] = 0xff | crc[9]
			printf("Master> RxDone\r\n");
			stmLINmodule.rxMsgDone = 1;
			stmLINmodule.bufPos = 0;
		}
    }
}
*/

//Address Intially
// ADM = 0;
// AD[6:0] = HW[0..2] | PA[3:0] = xxx | 0000

//MASTER
int stmAMIS30621LIN_MasterTask (void){
	unsigned char msgid, controldata, i;
	unsigned short pos;

	//stmLINmodule.NodeAddressNAD8 = 0x80; //0K Unicast (HW0=0, HW1=0, HW2=0)
	stmLINmodule.NodeAddressNAD8 = 0x40;   //OK Unicast (HW0=1, HW1=0, HW2=0)
	//stmLINmodule.NodeAddressNAD8 = 0x00; //0K Broadcast

	stmLINmodule.rxMsgDone = 0;
	stmLINmodule.role = LIN_ROLE_MASTER;
	stmLinGotoWakeup();

	stmAmisGetFullStatus();
	stmAmisGetOTPparam();
	stmAMIS30621Set_Dynamic_ID();

	stmAmisSetMotorParameter(stmLINmodule.NodeAddressNAD8,
			0x07,//Irun
			0x07,//Ihold
			0x2, //maxVelocity
			0x2, //minVelocity
			0,	//SetPos1
			0,
			0,
			0,
			2);//1/8 step////0=HarfStep); //1 = StepMode = 1/4 step

	while(1){
		//Schedule timeslot 0
		//stmLinGotoWakeup();
		stmAmisGetFullStatus();
		printf("===========\r\n");
		//delayms(500);
		pos =0;
		for(i=0;i<10;i++){

					stmAmisSetPosition(stmLINmodule.NodeAddressNAD8,pos,stmLINmodule.NodeAddressNAD8 + 1,pos+2);
					pos = pos + 0x800;
					delayms(1000);
		}
		continue;
		//stmLinGotoSleep();

		//dtUserLedCon(1);
		delayms(500);
		stmAMIS30621Set_Dynamic_ID();
		//Schedule timeslot 1
		//hw100msecDelay(1);//somedelay(1000000);
		//dtUserLedCon(0);
		delayms(500);
		//Another Test ....
		//AMIS30621SetMotorParameter(,,,); //TBD
		//AMIS30621Set_Dynamic_ID(); //TBD

		//goto sleep
		//LinGotoSleep();
		//hw100msecDelay(1);//somedelay(10000000);
		//goto Wakeup
		//LinGotoWakeup();
		//hw100msecDelay(1);//somedelay(100000);

	}
}

//need time scheduler
/*
//MASTER
int stmLIN_MasterTask (void){
	unsigned char msgid, controldata;
	while(1){
		//Schedule timeslot 0
		stmLinSendHeader (MsgID_0); //expected response from slave will be handled in ISR
		if(stmLINmodule.bufPos > 0){
			printf("MASTER> data byte[0]=(0x%02)\r\n", stmLINmodule.LinMsgByteArray[0]);
			stmLINmodule.detectSync = 0;
			stmLINmodule.bufPos = 0;
		}
		delayms(1);
		//dtUserLedCon(1);
		//Schedule timeslot 1
		while(1)
		stmLinSendHeader (MsgID_1); //send header and control data by this master

		stmLinSendDataByMaster (MsgID_1, MotorDirectionCW);
		delayms(100);
		//dtUserLedCon(0);

		//goto sleep
		stmLinGotoSleep();
		delayms(1);//somedelay(10000000);
		//goto Wakeup
		stmLinGotoWakeup();
		delayms(1);//somedelay(100000);

	}
}
*/
void stmAMIS30621_TMC211_LinLoop(void)
{
	printf("LIN Master Test with the Slave of AMIS30621/TMC211 LIN Motor Driver.");
	stmLinNodeInit(LIN_ROLE_MASTER); //Master mode Init

	//AMIS30621LinMasterConfig(); //Master mode Init

	stmAMIS30621LIN_MasterTask ();

}

