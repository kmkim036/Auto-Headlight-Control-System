#include "ethopts.h"
#include "yInc.h"
#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"

#include "lwip/include/lwipopts.h"
//#include "ptpstm/include/datatypes.h"

/* bmc.c */
#if (PROJ_FOR == PROJ_FOR_PTP)
#include "ptpstm/include/ptpd.h"

/* Convert EUI48 format to EUI64 */
void EUI48toEUI64(const octet_t * eui48, octet_t * eui64)
{
	eui64[0] = eui48[0];
	eui64[1] = eui48[1];
	eui64[2] = eui48[2];
	eui64[3] = 0xff;
	eui64[4] = 0xfe;
	eui64[5] = eui48[3];
	eui64[6] = eui48[4];
	eui64[7] = eui48[5];
}

/* Init ptpClock with run time values (initialization constants are in constants.h) */
void initData(PtpClock *ptpClock)
{
	RunTimeOpts * rtOpts;

	DBG("initData\r\n");
	rtOpts = ptpClock->rtOpts;

	/* Default data set */
	ptpClock->defaultDS.twoStepFlag = DEFAULT_TWO_STEP_FLAG;

	/* Init clockIdentity with MAC address and 0xFF and 0xFE. see spec 7.5.2.2.2 */
	if ((CLOCK_IDENTITY_LENGTH == 8) && (PTP_UUID_LENGTH == 6))	{
			DBGVV("initData: EUI48toEUI64\r\n");
			EUI48toEUI64(ptpClock->portUuidField, ptpClock->defaultDS.clockIdentity);
	}
	else if (CLOCK_IDENTITY_LENGTH == PTP_UUID_LENGTH){
			memcpy(ptpClock->defaultDS.clockIdentity, ptpClock->portUuidField, CLOCK_IDENTITY_LENGTH);
	}else{
			ERROR("initData: UUID length is not valid");
	}
	ptpClock->foreignMasterDS.bestMaster = NULL;
	ptpClock->defaultDS.numberPorts = NUMBER_PORTS;
	ptpClock->disabled = rtOpts->portDisabled;

	/*
	if(rtOpts->ipMode == IPMODE_MULTICAST &&rtOpts->delayMechanism == E2E) {
			memcpy(&ptpClock->profileIdentity, &PROFILE_ID_DEFAULT_E2E,6);
		}

		if(rtOpts->ipMode == IPMODE_MULTICAST &&rtOpts->delayMechanism == P2P) {
			memcpy(&ptpClock->profileIdentity, &PROFILE_ID_DEFAULT_P2P,6);
		}

		if(rtOpts->dot1AS) {
			memcpy(&ptpClock->profileIdentity, &PROFILE_ID_802_1AS,6);
		}
	*/

	ptpClock->defaultDS.clockQuality.clockAccuracy = rtOpts->clockQuality.clockAccuracy;
	ptpClock->defaultDS.clockQuality.clockClass = rtOpts->clockQuality.clockClass;
	ptpClock->defaultDS.clockQuality.offsetScaledLogVariance = rtOpts->clockQuality.offsetScaledLogVariance;

	ptpClock->defaultDS.priority1 = rtOpts->priority1;
	ptpClock->defaultDS.priority2 = rtOpts->priority2;

	ptpClock->defaultDS.domainNumber = rtOpts->domainNumber;
	ptpClock->defaultDS.slaveOnly = rtOpts->slaveOnly;
	if(ptpClock->defaultDS.slaveOnly){
		rtOpts->clockQuality.clockClass = SLAVE_ONLY_CLOCK_CLASS;
		ptpClock->defaultDS.clockQuality.clockClass = SLAVE_ONLY_CLOCK_CLASS;
	}

	// Port configuration data set

	// PortIdentity Init (portNumber = 1 for an ordinary clock spec 7.5.2.3)
	memcpy(ptpClock->portDS.portIdentity.clockIdentity, ptpClock->defaultDS.clockIdentity, CLOCK_IDENTITY_LENGTH);
	ptpClock->portDS.portIdentity.portNumber = rtOpts->portNumber;//NUMBER_PORTS;

	//set initial rate of delayreqs until we receive the first announce message
	ptpClock->portDS.logMinDelayReqInterval = DEFAULT_DELAYREQ_INTERVAL;//rtOpts->initial_delayreq;//DEFAULT_DELAYREQ_INTERVAL;
	ptpClock->portDS.peerMeanPathDelay.seconds = ptpClock->portDS.peerMeanPathDelay.nanoseconds = 0;//clearTime(&ptpClock->portDS.peerMeanPathDelay);
	ptpClock->portDS.logAnnounceInterval = rtOpts->announceInterval;
	ptpClock->portDS.announceReceiptTimeout = rtOpts->announceReceiptTimeout;//DEFAULT_ANNOUNCE_RECEIPT_TIMEOUT;
	ptpClock->portDS.logSyncInterval = rtOpts->syncInterval;
	ptpClock->portDS.delayMechanism = rtOpts->delayMechanism;
	ptpClock->portDS.logMinPDelayReqInterval = rtOpts->logMinPdelayReqInterval;//DEFAULT_PDELAYREQ_INTERVAL;
	ptpClock->portDS.versionNumber = VERSION_PTP;
	if(rtOpts->dot1AS) {
	    ptpClock->portDS.transportSpecific = TSP_ETHERNET_AVB;
	} else {
	    ptpClock->portDS.transportSpecific = TSP_DEFAULT;
	}
	// Init other stuff
	ptpClock->foreignMasterDS.number_foreign_records = 0;
	ptpClock->foreignMasterDS.max_foreign_records = rtOpts->maxForeignRecords;

	//added
	ptpClock->inboundLatency = rtOpts->inboundLatency;
	ptpClock->outboundLatency = rtOpts->outboundLatency;

	ptpClock->servo.sDelay = rtOpts->servo.sDelay;
	ptpClock->servo.sOffset = rtOpts->servo.sOffset;
	ptpClock->servo.ai = rtOpts->servo.ai;
	ptpClock->servo.ap = rtOpts->servo.ap;
	ptpClock->servo.noAdjust = rtOpts->servo.noAdjust;
	ptpClock->servo.noResetClock = rtOpts->servo.noResetClock;

	ptpClock->stats = rtOpts->stats;
}

bool isSamePortIdentity(const PortIdentity * A, const PortIdentity * B)
{
	return (bool)(0 == memcmp(A->clockIdentity, B->clockIdentity, CLOCK_IDENTITY_LENGTH) && (A->portNumber == B->portNumber));
}
/*
void addForeign(PtpClock *ptpClock, const MsgHeader *header, const MsgAnnounce * announce)
{
	int i, j;
	bool found = FALSE;

	j = ptpClock->foreignMasterDS.foreign_record_best;

	// Check if Foreign master is already known
	for (i = 0; i < ptpClock->foreignMasterDS.number_foreign_records; i++){
		if (isSamePortIdentity(&header->sourcePortIdentity, &ptpClock->foreignMasterDS.records[j].foreignMasterPortIdentity)){
			// Foreign Master is already in ForeignMaster data set
			ptpClock->foreignMasterDS.records[j].foreignMasterAnnounceMessages++;
			found = TRUE;
			DBGV("addForeign: AnnounceMessage incremented \r\n");
			memcpy(&ptpClock->foreignMasterDS.records[j].header, header, sizeof(MsgHeader)); //ptpClock->foreignMasterDS.records[j].header = *header;
			memcpy(&ptpClock->foreignMasterDS.records[j].announce, announce, sizeof(MsgAnnounce)); //ptpClock->foreignMasterDS.records[j].announce = *announce;
			break;
		}

		j = (j + 1) % ptpClock->foreignMasterDS.number_foreign_records;
	}

	// New Foreign Master
	if (!found)
	{
		if (ptpClock->foreignMasterDS.number_foreign_records < ptpClock->foreignMasterDS.max_foreign_records){
			ptpClock->foreignMasterDS.number_foreign_records++;
		}

		j = ptpClock->foreignMasterDS.foreign_record_i;

		// Copy new foreign master data set from Announce message
		memcpy(ptpClock->foreignMasterDS.records[j].foreignMasterPortIdentity.clockIdentity, header->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
		ptpClock->foreignMasterDS.records[j].foreignMasterPortIdentity.portNumber = header->sourcePortIdentity.portNumber;
		ptpClock->foreignMasterDS.records[j].foreignMasterAnnounceMessages = 0;

		// Header and announce field of each Foreign Master are useful to run Best Master Clock Algorithm
		memcpy(&ptpClock->foreignMasterDS.records[j].header, header, sizeof(MsgHeader)); //ptpClock->foreignMasterDS.records[j].header = *header;
		memcpy(&ptpClock->foreignMasterDS.records[j].announce, announce, sizeof(MsgAnnounce));//ptpClock->foreignMasterDS.records[j].announce = *announce;
		DBGV("addForeign: New foreign Master added \r\n");

		ptpClock->foreignMasterDS.i = (ptpClock->foreignMasterDS.i + 1) % ptpClock->foreignMasterDS.capacity;
	}
}
*/
#define m2 m1

/* Local clock is becoming Master. Table 13 (9.3.5) of the spec.*/
void m1(PtpClock *ptpClock)
{
	DBGV("bmc: m1\r\n");

	/* Current data set update */
	ptpClock->currentDS.stepsRemoved = 0;
	ptpClock->currentDS.offsetFromMaster.seconds = ptpClock->currentDS.offsetFromMaster.nanoseconds = 0;
	ptpClock->currentDS.meanPathDelay.seconds = ptpClock->currentDS.meanPathDelay.nanoseconds = 0;

	/* Parent data set */
	memcpy(ptpClock->parentDS.parentPortIdentity.clockIdentity, ptpClock->defaultDS.clockIdentity, CLOCK_IDENTITY_LENGTH);
	ptpClock->parentDS.parentPortIdentity.portNumber = 0;
	memcpy(ptpClock->parentDS.grandmasterIdentity, ptpClock->defaultDS.clockIdentity, CLOCK_IDENTITY_LENGTH);
	ptpClock->parentDS.grandmasterClockQuality.clockAccuracy = ptpClock->defaultDS.clockQuality.clockAccuracy;
	ptpClock->parentDS.grandmasterClockQuality.clockClass = ptpClock->defaultDS.clockQuality.clockClass;
	ptpClock->parentDS.grandmasterClockQuality.offsetScaledLogVariance = ptpClock->defaultDS.clockQuality.offsetScaledLogVariance;
	ptpClock->parentDS.grandmasterPriority1 = ptpClock->defaultDS.priority1;
	ptpClock->parentDS.grandmasterPriority2 = ptpClock->defaultDS.priority2;

	/* Time Properties data set */
	ptpClock->timePropertiesDS.currentUtcOffset = ptpClock->rtOpts->currentUtcOffset;
	ptpClock->timePropertiesDS.currentUtcOffsetValid = DEFAULT_UTC_VALID;
	ptpClock->timePropertiesDS.leap59 = FALSE;
	ptpClock->timePropertiesDS.leap61 = FALSE;
	ptpClock->timePropertiesDS.timeTraceable = DEFAULT_TIME_TRACEABLE;
	ptpClock->timePropertiesDS.frequencyTraceable = DEFAULT_FREQUENCY_TRACEABLE;
	ptpClock->timePropertiesDS.ptpTimescale = (bool)(DEFAULT_TIMESCALE == PTP_TIMESCALE);
#if (ETHIF_ROLE == PTP_GRANDMASTERCLOCK)
#ifdef USE_GPS
	ptpClock->timePropertiesDS.timeSource = GPS;
#else
	ptpClock->timePropertiesDS.timeSource = DEFAULT_TIME_SOURCE;
#endif
#else
	ptpClock->timePropertiesDS.timeSource = DEFAULT_TIME_SOURCE;
#endif
}

void p1(PtpClock *ptpClock)
{
	// make sure we revert to ARB timescale in Passive mode
	if(ptpClock->portDS.portState == PTP_PASSIVE){
		ptpClock->timePropertiesDS.currentUtcOffsetValid = ptpClock->rtOpts->timeProperties.currentUtcOffsetValid;
		ptpClock->timePropertiesDS.currentUtcOffset = ptpClock->rtOpts->timeProperties.currentUtcOffset;
	}
}

/* Local clock is synchronized to Ebest Table 16 (9.3.5) of the spec */
void s1(PtpClock *ptpClock, const MsgHeader *header, const MsgAnnounce *announce)
{
	bool isFromCurrentParent;

	DBGV("bmc: s1\r\n");

	/* Current DS */
	ptpClock->currentDS.stepsRemoved = announce->stepsRemoved + 1;

	isFromCurrentParent = isSamePortIdentity(&ptpClock->parentDS.parentPortIdentity, &header->sourcePortIdentity);

	if (!isFromCurrentParent){
			setFlag(ptpClock->events, MASTER_CLOCK_CHANGED);
	}

	/* Parent DS */
	memcpy(ptpClock->parentDS.parentPortIdentity.clockIdentity, header->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
	ptpClock->parentDS.parentPortIdentity.portNumber = header->sourcePortIdentity.portNumber;
	memcpy(ptpClock->parentDS.grandmasterIdentity, announce->grandmasterIdentity, CLOCK_IDENTITY_LENGTH);
	ptpClock->parentDS.grandmasterClockQuality.clockAccuracy = announce->grandmasterClockQuality.clockAccuracy;
	ptpClock->parentDS.grandmasterClockQuality.clockClass = announce->grandmasterClockQuality.clockClass;
	ptpClock->parentDS.grandmasterClockQuality.offsetScaledLogVariance = announce->grandmasterClockQuality.offsetScaledLogVariance;
	ptpClock->parentDS.grandmasterPriority1 = announce->grandmasterPriority1;
	ptpClock->parentDS.grandmasterPriority2 = announce->grandmasterPriority2;

	/* Timeproperties DS */
	ptpClock->timePropertiesDS.currentUtcOffset = announce->currentUtcOffset;
	ptpClock->timePropertiesDS.currentUtcOffsetValid = getFlag(header->flagField1, FLAG1_UTC_OFFSET_VALID);//UTCV
	ptpClock->timePropertiesDS.leap59 = getFlag(header->flagField1, FLAG1_LEAP59);//LT59
	ptpClock->timePropertiesDS.leap61 = getFlag(header->flagField1, FLAG1_LEAP61); //LT61
	ptpClock->timePropertiesDS.timeTraceable = getFlag(header->flagField1, FLAG1_TIME_TRACEABLE); //TTRA
	ptpClock->timePropertiesDS.frequencyTraceable = getFlag(header->flagField1, FLAG1_FREQUENCY_TRACEABLE);//FTRA
	ptpClock->timePropertiesDS.ptpTimescale = getFlag(header->flagField1, FLAG1_PTP_TIMESCALE);//PPTP
	ptpClock->timePropertiesDS.timeSource = announce->timeSource;
}

/**
 * \brief Copy local data set into header and announce message. 9.3.4 table 12
 */
void copyD0(MsgHeader *header, MsgAnnounce *announce, PtpClock *ptpClock)
{
	announce->grandmasterPriority1 = ptpClock->defaultDS.priority1;
	memcpy(announce->grandmasterIdentity, ptpClock->defaultDS.clockIdentity, CLOCK_IDENTITY_LENGTH);
	announce->grandmasterClockQuality.clockClass = ptpClock->defaultDS.clockQuality.clockClass;
	announce->grandmasterClockQuality.clockAccuracy = ptpClock->defaultDS.clockQuality.clockAccuracy;
	announce->grandmasterClockQuality.offsetScaledLogVariance = ptpClock->defaultDS.clockQuality.offsetScaledLogVariance;
	announce->grandmasterPriority2 = ptpClock->defaultDS.priority2;
	announce->stepsRemoved = 0;
	memcpy(header->sourcePortIdentity.clockIdentity, ptpClock->defaultDS.clockIdentity, CLOCK_IDENTITY_LENGTH);
}


#define A_better_then_B 1
#define B_better_then_A -1
#define A_better_by_topology_then_B 1
#define B_better_by_topology_then_A -1
#define ERROR_1 0
#define ERROR_2 -0


#define COMPARE_AB_RETURN_BETTER(cond, msg)                             \
	if ((announceA->cond) > (announceB->cond)) {                           \
		DBGVV("bmcDataSetComparison: " msg ": B better then A\r\n");          \
		return B_better_then_A;                                             \
	}                                                                     \
	if ((announceB->cond) > (announceA->cond)) {                           \
		DBGVV("bmcDataSetComparison: " msg ": A better then B\r\n");          \
		return A_better_then_B;                                             \
	}                                                                     \

/* Data set comparison bewteen two foreign masters (9.3.4 fig 27) return similar to memcmp() */
/*
int8_t bmcDataSetComparison(MsgHeader *headerA, MsgAnnounce *announceA,
															MsgHeader *headerB, MsgAnnounce *announceB, PtpClock *ptpClock)
{
	int grandmasterIdentityComp;
	short comp = 0;

	DBGV("bmcDataSetComparison\r\n");
	// Identity comparison

	// GM identity of A == GM identity of B
	// TODO: zkontrolovat memcmp, co vraci za vysledky !
	grandmasterIdentityComp = memcmp(announceA->grandmasterIdentity, announceB->grandmasterIdentity, CLOCK_IDENTITY_LENGTH);

	if (0 != grandmasterIdentityComp)
	{
		// Algoritgm part 1 - Figure 27
		COMPARE_AB_RETURN_BETTER(grandmasterPriority1,"grandmaster.Priority1");
		COMPARE_AB_RETURN_BETTER(grandmasterClockQuality.clockClass,"grandmaster.clockClass");
		COMPARE_AB_RETURN_BETTER(grandmasterClockQuality.clockAccuracy,"grandmaster.clockAccuracy");
		COMPARE_AB_RETURN_BETTER(grandmasterClockQuality.offsetScaledLogVariance,"grandmaster.Variance");
		COMPARE_AB_RETURN_BETTER(grandmasterPriority2,"grandmaster.Priority2");

		if (grandmasterIdentityComp > 0)
		{
			DBGVV("bmcDataSetComparison: grandmaster.Identity: B better then A\r\n");
			return B_better_then_A;
		}
		else if (grandmasterIdentityComp < 0)
		{
			DBGVV("bmcDataSetComparison: grandmaster.Identity: A better then B\r\n");
			return A_better_then_B;
		}
	}

	// Algoritgm part 2 - Figure 28
	if ((announceA->stepsRemoved) > (announceB->stepsRemoved + 1))
	{
		DBGVV("bmcDataSetComparison: stepsRemoved: B better then A\r\n");
		return B_better_then_A;
	}

	if ((announceB->stepsRemoved) > (announceA->stepsRemoved + 1))
	{
		DBGVV("bmcDataSetComparison: stepsRemoved: A better then B\r\n");
		return A_better_then_B;
	}

	if ((announceA->stepsRemoved) > (announceB->stepsRemoved))
	{
		comp = memcmp(headerA->sourcePortIdentity.clockIdentity, ptpClock->portDS.portIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);

		if (comp > 0){
			// reciever < sender
			DBGVV("bmcDataSetComparison: PortIdentity: B better then A\r\n");
			return B_better_then_A;
		}
		else if (comp < 0){
			// reciever > sender
			DBGVV("bmcDataSetComparison: PortIdentity: B better by topology then A\r\n");
			return B_better_by_topology_then_A;
		}else{
			DBGVV("bmcDataSetComparison: ERROR 1\r\n");
			return ERROR_1;
		}
	}
	else if ((announceA->stepsRemoved) < (announceB->stepsRemoved))
	{
		comp = memcmp(headerB->sourcePortIdentity.clockIdentity, ptpClock->portDS.portIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
		if (comp > 0){	// reciever < sender
			DBGVV("bmcDataSetComparison: PortIdentity: A better then B\r\n");
			return A_better_then_B;
		}else if (comp < 0){
			// reciever > sender
			DBGVV("bmcDataSetComparison: PortIdentity: A better by topology then B\r\n");
			return A_better_by_topology_then_B;
		}else{
			DBGV("bmcDataSetComparison: ERROR 1\r\n");
			return ERROR_1;
		}
	}

	comp = memcmp(headerA->sourcePortIdentity.clockIdentity, headerB->sourcePortIdentity.clockIdentity, CLOCK_IDENTITY_LENGTH);
	if (comp > 0)
	{
		// A > B
		DBGVV("bmcDataSetComparison: sourcePortIdentity: B better by topology then A\r\n");
		return B_better_by_topology_then_A;
	}
	else if (comp < 0)
	{
		// B > A
		DBGVV("bmcDataSetComparison: sourcePortIdentity: A better by topology then B\r\n");
		return A_better_by_topology_then_B;
	}

	// compare port numbers of recievers of A and B - same as we have only one port
	DBGV("bmcDataSetComparison: ERROR 2\r\n");
	return ERROR_2;
}
*/
/*Data set comparison bewteen two foreign masters (9.3.4 fig 27)
 * return similar to memcmp() */

static unsigned char bmcDataSetComparison(const ForeignMasterRecord *a, const ForeignMasterRecord *b, const PtpClock *ptpClock)
{


	DBGV("Data set comparison \n");
	short comp = 0;


	/* disqualification comes above anything else */

	if(a->disqualified > b->disqualified) {
	    return -1;
	}

	if(a->disqualified < b->disqualified) {
	    return 1;
	}

	/*Identity comparison*/
	comp = memcmp(a->announce.grandmasterIdentity,b->announce.grandmasterIdentity,CLOCK_IDENTITY_LENGTH);

	if (comp!=0)
		goto dataset_comp_part_1;

	  /* Algorithm part2 Fig 28 */
	if (a->announce.stepsRemoved > b->announce.stepsRemoved+1)
		return 1;
	if (a->announce.stepsRemoved+1 < b->announce.stepsRemoved)
		return -1;

	/* A within 1 of B */

	if (a->announce.stepsRemoved > b->announce.stepsRemoved) {
		comp = memcmp(a->header.sourcePortIdentity.clockIdentity,ptpClock->parentDS.parentPortIdentity.clockIdentity,CLOCK_IDENTITY_LENGTH);
		if(comp < 0)
			return -1;
		if(comp > 0)
			return 1;
		DBG("Sender=Receiver : Error -1");
		return 0;
	}

	if (a->announce.stepsRemoved < b->announce.stepsRemoved) {
		comp = memcmp(b->header.sourcePortIdentity.clockIdentity,ptpClock->parentDS.parentPortIdentity.clockIdentity,CLOCK_IDENTITY_LENGTH);

		if(comp < 0)
			return -1;
		if(comp > 0)
			return 1;
		DBG("Sender=Receiver : Error -1");
		return 0;
	}
	/*  steps removed A = steps removed B */
	comp = memcmp(a->header.sourcePortIdentity.clockIdentity,b->header.sourcePortIdentity.clockIdentity,CLOCK_IDENTITY_LENGTH);

	if (comp<0) {
		return -1;
	}

	if (comp>0) {
		return 1;
	}

	/* identity A = identity B */

	if (a->header.sourcePortIdentity.portNumber < b->header.sourcePortIdentity.portNumber)
		return -1;
	if (a->header.sourcePortIdentity.portNumber > b->header.sourcePortIdentity.portNumber)
		return 1;

	DBG("Sender=Receiver : Error -2");
	return 0;

	  /* Algorithm part 1 Fig 27 */
dataset_comp_part_1:

	/* OPTIONAL domain comparison / any domain */

	if(ptpClock->rtOpts->anyDomain) {
	    // part 1: preferred domain wins
	    if(a->header.domainNumber == ptpClock->rtOpts->domainNumber && b->header.domainNumber != ptpClock->defaultDS.domainNumber)
		return -1;
	    if(a->header.domainNumber != ptpClock->rtOpts->domainNumber && b->header.domainNumber == ptpClock->defaultDS.domainNumber)
		return 1;

	    // part 2: lower domain wins
	    if(a->header.domainNumber < b->header.domainNumber)
		return -1;

	    if(a->header.domainNumber > b->header.domainNumber)
		return 1;
	}

	/* Compare localPreference - only used by slaves when using unicast negotiation */
	DBGV("bmcDataSetComparison a->localPreference: %d, b->localPreference: %d\n", a->localPreference, b->localPreference);
	if(a->localPreference < b->localPreference) {
	    return -1;
	}
	if(a->localPreference > b->localPreference) {
	    return 1;
	}

	/* Compare GM priority1 */
	if (a->announce.grandmasterPriority1 < b->announce.grandmasterPriority1)
		return -1;
	if (a->announce.grandmasterPriority1 > b->announce.grandmasterPriority1)
		return 1;

	/* non-standard BMC extension to prioritise GMs with UTC valid */
	/*
	if(ptpClock->rtOpts->preferUtcValid) {
		bool utcA = IS_SET(a->header.flagField1, UTCV);
		bool utcB = IS_SET(b->header.flagField1, UTCV);
		if(utcA > utcB)
			return -1;
		if(utcA < utcB)
			return 1;
	}
	*/
	/* Compare GM class */
	if (a->announce.grandmasterClockQuality.clockClass <
			b->announce.grandmasterClockQuality.clockClass)
		return -1;
	if (a->announce.grandmasterClockQuality.clockClass >
			b->announce.grandmasterClockQuality.clockClass)
		return 1;

	/* Compare GM accuracy */
	if (a->announce.grandmasterClockQuality.clockAccuracy <
			b->announce.grandmasterClockQuality.clockAccuracy)
		return -1;
	if (a->announce.grandmasterClockQuality.clockAccuracy >
			b->announce.grandmasterClockQuality.clockAccuracy)
		return 1;

	/* Compare GM offsetScaledLogVariance */
	if (a->announce.grandmasterClockQuality.offsetScaledLogVariance <
			b->announce.grandmasterClockQuality.offsetScaledLogVariance)
		return -1;
	if (a->announce.grandmasterClockQuality.offsetScaledLogVariance >
			b->announce.grandmasterClockQuality.offsetScaledLogVariance)
		return 1;

	/* Compare GM priority2 */
	if (a->announce.grandmasterPriority2 < b->announce.grandmasterPriority2)
		return -1;
	if (a->announce.grandmasterPriority2 > b->announce.grandmasterPriority2)
		return 1;

	/* Compare GM identity */
	if (comp < 0)
		return -1;
	else if (comp > 0)
		return 1;
	return 0;
}

/* State decision algorithm 9.3.3 Fig 26 */
/*
uint8_t bmcStateDecision( PtpClock *ptpClock, ForeignMasterRecord *foreign)//bmcStateDecision(MsgHeader *header, MsgAnnounce *announce, PtpClock *ptpClock)
{
	int comp;
	bool newBM;
	ForeignMasterRecord me;

	if ((!ptpClock->foreignMasterDS.count) && (ptpClock->portDS.portState == PTP_LISTENING))
	{
		return PTP_LISTENING;
	}

	copyD0(&ptpClock->msgTmpHeader, &ptpClock->msgTmp.announce, ptpClock);

	comp = bmcDataSetComparison(&ptpClock->msgTmpHeader, &ptpClock->msgTmp.announce, header, announce, ptpClock);

	DBGV("bmcStateDecision: %d\r\n", comp);

	if (ptpClock->defaultDS.clockQuality.clockClass < 128)
	{
		if (A_better_then_B == comp)
		{
			m1(ptpClock);  // M1
			return PTP_MASTER;
		}
		else
		{
			p1(ptpClock);
			return PTP_PASSIVE;
		}
	}
	else
	{
		if (A_better_then_B == comp)
		{
			m2(ptpClock); // M2
			return PTP_MASTER;
		}
		else
		{
			s1(ptpClock, header, announce);
			return PTP_SLAVE;
		}
	}
}
*/
//State decision algorithm 9.3.3 Fig 26
uint8_t bmcStateDecision( PtpClock *ptpClock, ForeignMasterRecord *foreign)
{
	int comp;
	bool newBM;
	ForeignMasterRecord me;

	memset(&me, 0, sizeof(me));
	me.localPreference = LOWEST_LOCALPREFERENCE;

	newBM = ((memcmp(foreign->header.sourcePortIdentity.clockIdentity,
			    ptpClock->parentDS.parentPortIdentity.clockIdentity,CLOCK_IDENTITY_LENGTH)) ||
		(foreign->header.sourcePortIdentity.portNumber != ptpClock->parentDS.parentPortIdentity.portNumber));

	if (ptpClock->defaultDS.slaveOnly) {
		/* TBD
		// master has changed: mark old grants for cancellation - refreshUnicastGrants will pick this up
		if(newBM && (ptpClock->parentGrants != NULL)) {
		    ptpClock->previousGrants = ptpClock->parentGrants;
		}
		s1(ptpClock, &foreign->header,&foreign->announce);
		if(ptpClock->rtOpts->unicastNegotiation) {
			ptpClock->parentGrants = findUnicastGrants(&ptpClock->parentDS.parentPortIdentity, 0,
						ptpClock->unicastGrants, &ptpClock->grantIndex, ptpClock->unicastDestinationCount,
					    FALSE);
		}
		if (newBM) {
			// New BM #1
			//SET_ALARM(ALRM_MASTER_CHANGE, TRUE);
			//displayPortIdentity(&foreign->header.sourcePortIdentity,"New best master selected:");
			ptpClock->counters.bestMasterChanges++;
			//if (ptpClock->portDS.portState == PTP_SLAVE) displayStatus(ptpClock, "State: ");
			if(ptpClock->rtOpts->calibrationDelay) {
					ptpClock->isCalibrated = FALSE;
					timerStart(&ptpClock->timers[CALIBRATION_DELAY_TIMER], ptpClock->rtOpts->calibrationDelay);
			}
		}
        if(rtOpts->unicastNegotiation && ptpClock->parentGrants != NULL) {
            ptpClock->portDS.logAnnounceInterval = ptpClock->parentGrants->grantData[ANNOUNCE_INDEXED].logInterval;
			me.localPreference = ptpClock->parentGrants->localPreference;
		}
		*/
		return PTP_SLAVE;
	}

	if ((!ptpClock->foreignMasterDS.number_foreign_records) && (ptpClock->portDS.portState == PTP_LISTENING))
		return PTP_LISTENING;

	copyD0(&me.header, &me.announce, ptpClock);

	//DBGV("local clockQuality.clockClass: %d \n", ptpClock->defaultDS.clockQuality.clockClass);

	comp = bmcDataSetComparison(&me, foreign, ptpClock);
	if (ptpClock->defaultDS.clockQuality.clockClass < 128) {
		if (comp < 0) {
			m1(ptpClock);
			return PTP_MASTER;
		} else if (comp > 0) {
			s1(ptpClock,&foreign->header, &foreign->announce);
			if (newBM) { //MASTER_CHANGE.....
			// New BM #2
				//SET_ALARM(ALRM_MASTER_CHANGE, TRUE);
				//displayPortIdentity(&foreign->header.sourcePortIdentity, "New best master selected:");
				printf("New best master selected.\r\n");
				//ptpClock->counters.bestMasterChanges++;
				//if(ptpClock->portDS.portState == PTP_PASSIVE)	displayStatus(ptpClock, "State: ");
			}
			return PTP_PASSIVE;
		} else {
			DBG("Error in bmcDataSetComparison..\n");
		}
	} else {
		if (comp < 0) {
			m1(ptpClock);
			return PTP_MASTER;
		} else if (comp > 0) {
			s1(ptpClock,&foreign->header, &foreign->announce);
			if (newBM) {
			/* New BM #3 */
				//SET_ALARM(ALRM_MASTER_CHANGE, TRUE);
				//displayPortIdentity(&foreign->header.sourcePortIdentity, "New best master selected:");
				printf("New best master selected:\r\n");
				//ptpClock->counters.bestMasterChanges++;
				//if(ptpClock->portDS.portState == PTP_SLAVE)	displayStatus(ptpClock, "State: ");
				if(ptpClock->rtOpts->calibrationDelay) {
					ptpClock->isCalibrated = FALSE;
					ptpd_timerStart(CALIBRATION_DELAY_TIMER, ptpClock->rtOpts->calibrationDelay,ptpClock->itimer);
				}
			}
			return PTP_SLAVE;
		} else {
			DBG("Error in bmcDataSetComparison..\n");
		}
	}

	//ptpClock->counters.protocolErrors++;
	/*  MB: Is this the return code below correct? */
	/*  Anyway, it's a valid return code. */

	return PTP_FAULTY;
}

uint8_t bmc(PtpClock *ptpClock)
{
	int16_t i, best;

	/* Starting from i = 1, not necessery to test record[i = 0] against record[best = 0] -> they are the same */
	for (i = 1, best = 0; i < ptpClock->foreignMasterDS.number_foreign_records; i++)
	{
		if ((bmcDataSetComparison(&ptpClock->foreignMasterDS.foreign[i], &ptpClock->foreignMasterDS.foreign[best], ptpClock)) < 0)
		{
			best = i;
		}
	}

	DBGV("bmc: best record %d\r\n", best);
	ptpClock->foreignMasterDS.foreign_record_best = best;
	ptpClock->foreignMasterDS.bestMaster =  &ptpClock->foreignMasterDS.foreign[best];

	//return bmcStateDecision(&ptpClock->foreignMasterDS.records[best].header, &ptpClock->foreignMasterDS.records[best].announce, ptpClock);
	return (bmcStateDecision(ptpClock,ptpClock->foreignMasterDS.bestMaster));
}
#endif
