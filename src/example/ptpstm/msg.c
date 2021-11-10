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

/* msg.c */
#if (PROJ_FOR == PROJ_FOR_PTP)

#include "ptpstm/include/ptpd.h" //#include "../ptpd.h"

unsigned short flip16b(unsigned char *buf)
{
	unsigned short a;
	a = buf[0];
	a = (a << 8) | buf[1];
	return a;
}

/* Unpack header message */
void msgUnpackHeader(const octet_t *buf, MsgHeader *header)
{
	int32_t msb;
	uint32_t lsb;

	header->transportSpecific = buf[0] >> 4;
	header->messageType = buf[0] & 0x0F;
	header->versionPTP = buf[1] & 0x0F; //force reserved bit to zero if not
	header->messageLength = flip16b(buf+2);//buf[2]*0x100 + buf[3];
	header->domainNumber = buf[4];
	header->flagField0 = buf[6];
	header->flagField1 = buf[7];
	memcpy(&msb, (buf + 8), 4);
	memcpy(&lsb, (buf + 12), 4);
	header->correctionField.msb = flip32(msb);
	header->correctionField.lsb = flip32(lsb);
	memcpy(header->sourcePortIdentity.clockIdentity, (buf + 20), CLOCK_IDENTITY_LENGTH);
	header->sourcePortIdentity.portNumber = flip16b(buf+28);//[28]*0x100+buf[29];
	header->sequenceId = flip16b(buf+30);//buf[30]*0x100+buf[31];//header->sequenceId = flip16(*(int16_t*)(buf + 30));
	header->controlField = buf[32];
	header->logMessageInterval = buf[33];
}

/* Pack header message */
void msgPackHeader(const PtpClock *ptpClock, octet_t *buf)
{
	buf[0] = ptpClock->portDS.transportSpecific << 4; //0x10; //(spec annex D)
	buf[1] = ptpClock->portDS.versionNumber;
	buf[4] = ptpClock->defaultDS.domainNumber;
	if (ptpClock->defaultDS.twoStepFlag)
	{
			buf[6] = FLAG0_TWO_STEP;
	}
	memset((buf + 8), 0, 8);
	memcpy((buf + 20), ptpClock->portDS.portIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
	buf[28] = (ptpClock->portDS.portIdentity.portNumber)>>8; //*(int16_t*)(buf + 28) = flip16(ptpClock->portDS.portIdentity.portNumber);
	buf[29] = (ptpClock->portDS.portIdentity.portNumber) & 0xff;
	buf[33] = 0x7F; //Default value (spec Table 24)
}

/* Pack Announce message */
void msgPackAnnounce(const PtpClock *ptpClock, octet_t *buf)
{
	/* Changes in header */
	buf[0] = ptpClock->portDS.transportSpecific << 4; //0x10;
	buf[0] = buf[0] | ANNOUNCE; //Table 19
	*(int16_t*)(buf + 2)  = flip16(ANNOUNCE_LENGTH);
	*(int16_t*)(buf + 30) = flip16(ptpClock->sentAnnounceSequenceId);
	*(uint8_t*)(buf + 32) = CTRL_OTHER; /* Table 23 - controlField */
	buf[33] = ptpClock->portDS.logAnnounceInterval;

	/* Announce message */
	memset((buf + 34), 0, 10); /* originTimestamp */
	*(int16_t*)(buf + 44) = flip16(ptpClock->timePropertiesDS.currentUtcOffset);
	buf[47] = ptpClock->parentDS.grandmasterPriority1;
	buf[48] = ptpClock->defaultDS.clockQuality.clockClass;
	buf[49] = ptpClock->defaultDS.clockQuality.clockAccuracy;
	*(int16_t*)(buf + 50) = flip16(ptpClock->defaultDS.clockQuality.offsetScaledLogVariance);
	buf[52] = ptpClock->parentDS.grandmasterPriority2;
	memcpy((buf + 53), ptpClock->parentDS.grandmasterIdentity, CLOCK_IDENTITY_LENGTH);
	buf[61] = (ptpClock->currentDS.stepsRemoved)>>8; //*(int16_t*)(buf + 61) = flip16(ptpClock->currentDS.stepsRemoved);
	buf[62] = (ptpClock->currentDS.stepsRemoved) & 0xFF;
	buf[63] = ptpClock->timePropertiesDS.timeSource;
}

/* Unpack Announce message */
void msgUnpackAnnounce(const octet_t *buf, MsgAnnounce *announce)
{
	announce->originTimestamp.secondsField.msb = flip16b(buf+34);//flip16(*(int16_t*)(buf + 34));
	announce->originTimestamp.secondsField.lsb = flip32(*(uint32_t*)(buf + 36));
	announce->originTimestamp.nanosecondsField = flip32(*(uint32_t*)(buf + 40));
	announce->currentUtcOffset = flip16b(buf+44);//buf[44]*0x100+buf[45]; //flip16(*(int16_t*)(buf + 44));
	announce->grandmasterPriority1 = buf[47];
	announce->grandmasterClockQuality.clockClass = buf[48];
	announce->grandmasterClockQuality.clockAccuracy = buf[49];
	announce->grandmasterClockQuality.offsetScaledLogVariance = flip16b(buf+50);//flip16(*(int16_t*)(buf  + 50));
	announce->grandmasterPriority2 = buf[52];
	memcpy(announce->grandmasterIdentity, (buf + 53), CLOCK_IDENTITY_LENGTH);
	announce->stepsRemoved = flip16b(buf+61);//buf[61]*0x100+buf[62]; //flip16(*(int16_t*)(buf + 61));
	announce->timeSource = buf[63];
}

/* Pack SYNC message */
void msgPackSync(const PtpClock *ptpClock, octet_t *buf, const Timestamp *originTimestamp)
{
	/* Changes in header */

	buf[0] = ptpClock->portDS.transportSpecific << 4; //0x10; //RAZ messageType
	buf[0] = buf[0] | SYNC; //Table 19
	*(int16_t*)(buf + 2)  = flip16(SYNC_LENGTH);
	*(int16_t*)(buf + 30) = flip16(ptpClock->sentSyncSequenceId);
	*(uint8_t*)(buf + 32) = CTRL_SYNC; //Table 23
	*(int8_t*)(buf + 33) = ptpClock->portDS.logSyncInterval;
	memset((buf + 8), 0, 8); /* correction field */

	/* Sync message */
	*(int16_t*)(buf + 34) = flip16(originTimestamp->secondsField.msb);
	*(uint32_t*)(buf + 36) = flip32(originTimestamp->secondsField.lsb);
	*(uint32_t*)(buf + 40) = flip32(originTimestamp->nanosecondsField);
}

/* Unpack Sync message */
void msgUnpackSync(const octet_t *buf, MsgSync *sync)
{
	sync->originTimestamp.secondsField.msb = flip16(*(int16_t*)(buf + 34));
	sync->originTimestamp.secondsField.lsb = flip32(*(uint32_t*)(buf + 36));
	sync->originTimestamp.nanosecondsField = flip32(*(uint32_t*)(buf + 40));
}

/* Pack delayReq message */
void msgPackDelayReq(const PtpClock *ptpClock, octet_t *buf, const Timestamp *originTimestamp)
{
	/* Changes in header */
	buf[0] = ptpClock->portDS.transportSpecific  << 4; //0x10; //RAZ messageType
	buf[0] = buf[0] | DELAY_REQ; //Table 19
	*(int16_t*)(buf + 2)  = flip16(DELAY_REQ_LENGTH);
	*(int16_t*)(buf + 30) = flip16(ptpClock->sentDelayReqSequenceId);
	*(uint8_t*)(buf + 32) = CTRL_DELAY_REQ; //Table 23
	*(int8_t*)(buf + 33) = 0x7F; //Table 24
	memset((buf + 8), 0, 8);

	/* delay_req message */
	*(int16_t*)(buf + 34) = flip16(originTimestamp->secondsField.msb);
	*(uint32_t*)(buf + 36) = flip32(originTimestamp->secondsField.lsb);
	*(uint32_t*)(buf + 40) = flip32(originTimestamp->nanosecondsField);
}

/* Unpack delayReq message */
void msgUnpackDelayReq(const octet_t *buf, MsgDelayReq *delayreq)
{
	delayreq->originTimestamp.secondsField.msb = flip16(*(int16_t*)(buf + 34));
	delayreq->originTimestamp.secondsField.lsb = flip32(*(uint32_t*)(buf + 36));
	delayreq->originTimestamp.nanosecondsField = flip32(*(uint32_t*)(buf + 40));
}

/* Pack Follow_up message */
void msgPackFollowUp(const PtpClock *ptpClock, octet_t*buf, const Timestamp *preciseOriginTimestamp)
{
	/* Changes in header */
	buf[0] = ptpClock->portDS.transportSpecific  << 4; //0x10; //RAZ messageType
	buf[0] = buf[0] | FOLLOW_UP; //Table 19
	*(int16_t*)(buf + 2)  = flip16(FOLLOW_UP_LENGTH);
	*(int16_t*)(buf + 30) = flip16(ptpClock->sentSyncSequenceId - 1);//sentSyncSequenceId has already been  incremented in issueSync
	*(uint8_t*)(buf + 32) = CTRL_FOLLOW_UP; //Table 23
	*(int8_t*)(buf + 33) = ptpClock->portDS.logSyncInterval;

	/* Follow_up message */
	*(int16_t*)(buf + 34) = flip16(preciseOriginTimestamp->secondsField.msb);
	*(uint32_t*)(buf + 36) = flip32(preciseOriginTimestamp->secondsField.lsb);
	*(uint32_t*)(buf + 40) = flip32(preciseOriginTimestamp->nanosecondsField);
}

/* Unpack Follow_up message */
void msgUnpackFollowUp(const octet_t *buf, MsgFollowUp *follow)
{
	follow->preciseOriginTimestamp.secondsField.msb = flip16(*(int16_t*)(buf  + 34));
	follow->preciseOriginTimestamp.secondsField.lsb = flip32(*(uint32_t*)(buf + 36));
	follow->preciseOriginTimestamp.nanosecondsField = flip32(*(uint32_t*)(buf + 40));
}

/* Pack delayResp message */
void msgPackDelayResp(const PtpClock *ptpClock, octet_t *buf, const MsgHeader *header, const Timestamp *receiveTimestamp)
{
	/* Changes in header */
	buf[0] = ptpClock->portDS.transportSpecific << 4; //0x10; //RAZ messageType
	buf[0] = buf[0] | DELAY_RESP; //Table 19
	*(int16_t*)(buf + 2)  = flip16(DELAY_RESP_LENGTH);
	/* *(uint8_t*)(buf+4) = header->domainNumber; */ /* TODO: Why? */

	memset((buf + 8), 0, 8); //correction field
	/* Copy correctionField of  delayReqMessage */
	*(int32_t*)(buf + 8) = flip32(header->correctionField.msb);
	*(int32_t*)(buf + 12) = flip32(header->correctionField.lsb);
	//clockID [8]
	//sourcePortID[2]
	//seq id[2]
	*(int16_t*)(buf + 30) = flip16(header->sequenceId);
	*(uint8_t*)(buf + 32) = CTRL_DELAY_RESP; //Table 23
	*(int8_t*)(buf + 33) = ptpClock->portDS.logMinDelayReqInterval; //Table 24

	*(int16_t*)(buf + 34) = flip16(receiveTimestamp->secondsField.msb);
	*(uint32_t*)(buf + 36) = flip32(receiveTimestamp->secondsField.lsb);
	*(uint32_t*)(buf + 40) = flip32(receiveTimestamp->nanosecondsField);
	memcpy((buf + 44), header->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
	*(int16_t*)(buf + 52) = flip16(header->sourcePortIdentity.portNumber);
}

/* Unpack delayResp message */
void msgUnpackDelayResp(const octet_t *buf, MsgDelayResp *resp)
{
	resp->receiveTimestamp.secondsField.msb = flip16(*(int16_t*)(buf  + 34));
	resp->receiveTimestamp.secondsField.lsb = flip32(*(uint32_t*)(buf + 36));
	resp->receiveTimestamp.nanosecondsField = flip32(*(uint32_t*)(buf + 40));
	memcpy(resp->requestingPortIdentity.clockIdentity, (buf + 44), CLOCK_IDENTITY_LENGTH);
	resp->requestingPortIdentity.portNumber = flip16(*(int16_t*)(buf  + 52));
}

/* Pack PdelayReq message */
void msgPackPDelayReq(const PtpClock *ptpClock, octet_t *buf, const Timestamp *originTimestamp)
{
	buf[0] = ptpClock->portDS.transportSpecific << 4; //0x10; //RAZ messageType
	buf[0] = buf[0] | PDELAY_REQ; //Table 19
	*(int16_t*)(buf + 2)  = flip16(PDELAY_REQ_LENGTH);
	*(int16_t*)(buf + 30) = flip16(ptpClock->sentPDelayReqSequenceId);
	*(uint8_t*)(buf + 32) = CTRL_OTHER; //Table 23
	*(int8_t*)(buf + 33) = 0x7F; //Table 24
	memset((buf + 8), 0, 8);

	/* Pdelay_req message */
	*(int16_t*)(buf + 34) = flip16(originTimestamp->secondsField.msb);
	*(uint32_t*)(buf + 36) = flip32(originTimestamp->secondsField.lsb);
	*(uint32_t*)(buf + 40) = flip32(originTimestamp->nanosecondsField);

	memset((buf + 44), 0, 10); // RAZ reserved octets
}

/* Unpack PdelayReq message */
void msgUnpackPDelayReq(const octet_t *buf, MsgPDelayReq *pdelayreq)
{
	pdelayreq->originTimestamp.secondsField.msb = flip16(*(int16_t*)(buf  + 34));
	pdelayreq->originTimestamp.secondsField.lsb = flip32(*(uint32_t*)(buf + 36));
	pdelayreq->originTimestamp.nanosecondsField = flip32(*(uint32_t*)(buf + 40));
}

/* Pack PdelayResp message */
void msgPackPDelayResp(const PtpClock *ptpClock, octet_t *buf, const MsgHeader *header, const Timestamp *requestReceiptTimestamp)
{
	/* Changes in header */
	buf[0] = ptpClock->portDS.transportSpecific << 4; //0x10; //RAZ messageType
	buf[0] = buf[0] | PDELAY_RESP; //Table 19
	*(int16_t*)(buf + 2)  = flip16(PDELAY_RESP_LENGTH);
	/* *(uint8_t*)(buf+4) = header->domainNumber; */ /* TODO: Why? */
	memset((buf + 8), 0, 8);
	*(int16_t*)(buf + 30) = flip16(header->sequenceId);
	*(uint8_t*)(buf + 32) = CTRL_OTHER; //Table 23
	*(int8_t*)(buf + 33) = 0x7F; //Table 24

	/* Pdelay_resp message */
	*(int16_t*)(buf + 34) = flip16(requestReceiptTimestamp->secondsField.msb);
	*(uint32_t*)(buf + 36) = flip32(requestReceiptTimestamp->secondsField.lsb);
	*(uint32_t*)(buf + 40) = flip32(requestReceiptTimestamp->nanosecondsField);
	memcpy((buf + 44), header->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
	*(int16_t*)(buf + 52) = flip16(header->sourcePortIdentity.portNumber);

}

/* Unpack PdelayResp message */
void msgUnpackPDelayResp(const octet_t *buf, MsgPDelayResp *presp)
{
	presp->requestReceiptTimestamp.secondsField.msb = flip16(*(int16_t*)(buf  + 34));
	presp->requestReceiptTimestamp.secondsField.lsb = flip32(*(uint32_t*)(buf + 36));
	presp->requestReceiptTimestamp.nanosecondsField = flip32(*(uint32_t*)(buf + 40));
	memcpy(presp->requestingPortIdentity.clockIdentity, (buf + 44), CLOCK_IDENTITY_LENGTH);
	presp->requestingPortIdentity.portNumber = flip16(*(int16_t*)(buf + 52));
}

/* Pack PdelayRespfollowup message */
void msgPackPDelayRespFollowUp(const PtpClock *ptpClock, octet_t *buf, const MsgHeader *header, const Timestamp *responseOriginTimestamp)
{
	/* Changes in header */
	buf[0] = ptpClock->portDS.transportSpecific << 4; //0x10; //RAZ messageType
	buf[0] = buf[0]  | PDELAY_RESP_FOLLOW_UP; //Table 19
	*(int16_t*)(buf + 2)  = flip16(PDELAY_RESP_FOLLOW_UP_LENGTH);
	*(int16_t*)(buf + 30) = flip16(header->sequenceId);
	*(uint8_t*)(buf + 32) = CTRL_OTHER; //Table 23
	*(int8_t*)(buf + 33) = 0x7F; //Table 24

	/* Copy correctionField of  PdelayReqMessage */
	*(int32_t*)(buf + 8) = flip32(header->correctionField.msb);
	*(int32_t*)(buf + 12) = flip32(header->correctionField.lsb);

	/* Pdelay_resp_follow_up message */
	*(int16_t*)(buf + 34) = flip16(responseOriginTimestamp->secondsField.msb);
	*(uint32_t*)(buf + 36) = flip32(responseOriginTimestamp->secondsField.lsb);
	*(uint32_t*)(buf + 40) = flip32(responseOriginTimestamp->nanosecondsField);
	memcpy((buf + 44), header->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
	*(int16_t*)(buf + 52) = flip16(header->sourcePortIdentity.portNumber);
}

/* Unpack PdelayResp message */
void msgUnpackPDelayRespFollowUp(const octet_t *buf, MsgPDelayRespFollowUp *prespfollow)
{
	prespfollow->responseOriginTimestamp.secondsField.msb = flip16(*(int16_t*)(buf  + 34));
	prespfollow->responseOriginTimestamp.secondsField.lsb = flip32(*(uint32_t*)(buf + 36));
	prespfollow->responseOriginTimestamp.nanosecondsField = flip32(*(uint32_t*)(buf + 40));
	memcpy(prespfollow->requestingPortIdentity.clockIdentity, (buf + 44), CLOCK_IDENTITY_LENGTH);
	prespfollow->requestingPortIdentity.portNumber = flip16(*(int16_t*)(buf + 52));
}

copyClockIdentity( ClockIdentity dest, ClockIdentity src)
{
		memcpy(dest, src, CLOCK_IDENTITY_LENGTH);
}

#endif
