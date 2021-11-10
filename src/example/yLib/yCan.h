#ifndef __YCAN_H
#define __YCAN_H

#ifdef __cplusplus
 extern "C" {
#endif

 /* Private typedef -----------------------------------------------------------*/
 //typedef enum {FAILED = 0, PASSED = !FAILED} eResultStatus;
 //typedef enum {CAN_ROLE_SLAVE=0, CAN_ROLE_MASTER=1} CAN_ROLE;


 		  /*
 #define CAN_ErrorCode_NoErr           ((uint8_t)0x00)
 #define	CAN_ErrorCode_StuffErr        ((uint8_t)0x10)
 #define	CAN_ErrorCode_FormErr         ((uint8_t)0x20)
 #define	CAN_ErrorCode_ACKErr          ((uint8_t)0x30)
 #define	CAN_ErrorCode_BitRecessiveErr ((uint8_t)0x40)
 #define	CAN_ErrorCode_BitDominantErr  ((uint8_t)0x50)
 #define	CAN_ErrorCode_CRCErr          ((uint8_t)0x60)
 #define	CAN_ErrorCode_SoftwareSetErr  ((uint8_t)0x70)
 		  */

 #define USE_CAN_MODE_POLLING   	0
 #define USE_CAN_MODE_INTERRUPT 	1
 #define USE_CAN_MODE 			USE_CAN_MODE_INTERRUPT //USE_CAN_MODE_POLLING/

 #define YCAN_RX_SOME_ERROR     	2
 #define YCAN_RX_WE_HAVE_FRAMES 	1
 #define YCAN_RX_NO_FRAMES 		0

 #define ISTX 1
 #define ISRX 0

 #define CAN_MSG_ID0555 0x0555
 #define CAN_MSG_ID0666 0x0666
 #define CAN_MSG_ID071C 0x071C
 #define CAN_MSG_ID071D 0x071D

 #define CANTXMASK (0xffffffff)
 #define CANRXMASK (0xffffffff)

 struct _yCAN_module{
 	//u8 g_can_role;
 	unsigned long g_ulMsg1Count;
 	unsigned long g_ulMsg2Count;
 	unsigned long g_ulMsg3Count;
 	unsigned long g_bMsgObj3Sent;
 	unsigned long g_bErrFlag;
 	unsigned long g_bRxErrFlag;
 	unsigned long g_ulRxMsgCount;
 	unsigned char g_bRXFlags[33];
 	unsigned long g_bRXErrFlag;

 	//We use 3 MsgObjects.
 	CanTxMsg canMsgObject1;
 	CanTxMsg canMsgObject2;
 	CanTxMsg canMsgObject3;

 	//rx
 	unsigned char RxStatus;
 	CanRxMsg RxMessage;
 	unsigned int RxCnt;
 };
#endif
