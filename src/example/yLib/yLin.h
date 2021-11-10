#ifndef __YLIN_H
#define __YLIN_H

#ifdef __cplusplus
 extern "C" {
#endif

 //LIN
 #define LIN_BAUDRATE 19200 //19.2Kbps//#define LIN_BAUDRATE 20000 //20Kbps
  #if (USART2_FOR == USART2_FOR_LIN)
 	//extern void USART2_IntHandlerForLIN(void);//for LIN void USART2_IRQHandler(void);//for LIN
 	#define LIN_USART USART2
 #elif (USART1_FOR == USART1_FOR_LIN)
 	extern void USART1_IntHandlerForLIN(void);//for LIN
 	#define LIN_USART USART1
 #endif

 #define LIN_ROLE_MASTER 1
 #define LIN_ROLE_SLAVE 0


 #define LINMAXTIMEOUT 1000 //1sec

 //========== LIN MSG ===========================

 //For LIN msg parsing
 #define getMsgId(x) 	(x & 0x0f)
 #define getParity(x) 	((x & 0xc0)>>6)
 #define getMsgId54(x) 	((x & 0x30)>>4)

 struct LinMsgInfo{
 	u8 msgIndex;
 	u8 msgid;
 	u8 needResp;
 	char dleng;
 };

 struct LinTxBuf{
 	struct LinMsgInfo msgInfo;
 	u8 data[8]; //excluding crc
 };
 struct LinRxBuf{
 	struct LinMsgInfo msgInfo;
 	u8 data[8];//excluding crc
 	u8 sync_rcvd;
 	u8 msgid_rcvd;
   	u8 break_rcvd;
   	u8 fe_rcvd; //not used
   	u8 rcvd;
 	u8 needRespBySlave;
 };

 //============ Configurables =================================

#define LIN_ROLE LIN_ROLE_MASTER //LIN_ROLE_SLAVE //SLAVE

#define MAX_MSG_NUM (5)
 typedef enum {LinMsg0=0, LinMsg1=1, LinMsg2=2,LinMsg3=3, LinMsg4=4} LinMsgIndex;
 //LIN MSG FORMAT
 //(MSB)---- /P1-P0-ID5-ID4-ID3-ID2-ID1-ID0 ----(LSB); LSB is transmitted first.
 //P0 = ID4^ID2^ID1^ID0
 // ~P1=ID5^ID4^ID3^ID1
 //[ID5:ID4]=00 or 01 == 2bytes Length
 //         =10       == 4bytes Length
 //         =11       == 8bytes Length

 //#define MsgID_0 0x11 //0b00-01-0001 Read for requesting status --> expect Response transaction (e.g., Button ON/OFF status)
 					    //for reading : MsgId = 0001, leng=01 (msgLen=2), P0=0,P1=0
 //#define MsgID_1 0x12 //0b00-01-0010 Write for controling Motor (Master: transmiting Header and Data)
 					    //for writing : MsgId = 0010, leng=00 (msgLen=2), P0=0,P1=0

 //msgId with precalculated parity bits.
 #define MsgID_0_2A 0b00000000 //for sending 2 bytes : MsgId = 0000, leng=00 (==2bytes), P0=0,P1=0
 #define MsgID_0_2B 0b01010000 //for reading 2 bytes : Master=Header; Slave=Data[2]
 							   //MsgId=0000, leng=01 (=2bytes), P0=1 P1=0
 #define MsgID_0_4  0b10100000 //for sending 4 bytes : Master=Header+Data[4]
 							   //MsgId=0000, leng=10 (=4bytes), P0=0 P1=1
 #define MsgID_0_8  0b11110000 //for reading 8 bytes : Master=Header; Slave=Data[8]
 							   //MsgId=0000, leng=11 (=8bytes), P0=1 P1=1
 #define MsgID_3c_8 0x3c       //Broadcast with 8 bytes data.

 struct _yLIN_module{
	unsigned char role;
	struct LinTxBuf LinTxBuf;
	struct LinRxBuf LinRxBuf;
 	//msgPool
 	struct LinMsgInfo LinMsgInfoPool[MAX_MSG_NUM];
 };

 extern struct _yLIN_module g_LinMod;

#endif
