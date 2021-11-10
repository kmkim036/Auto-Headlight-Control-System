//REF: https://github.com/ptpd/ptpd/blob/master/src/protocol.c

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "ethopts.h"
#include "yInc.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "misc.h"
//#include "lwip/include/lwipopts.h"

/* msg.c */
#if (PROJ_FOR == PROJ_FOR_PTP)
#include "ptpstm/include/ptpd.h"

static void handle_ptp_rcvmsg(PtpClock*);
static void handleAnnounce(PtpClock*, bool);
static void handleSync(PtpClock*, TimeInternal*, bool);
static void handleFollowUp(PtpClock*, bool);
static void handlePDelayReq(PtpClock*, TimeInternal*, bool);
static void handleDelayReq(PtpClock*, TimeInternal*, bool);
static void handlePDelayResp(PtpClock*, TimeInternal*, bool);
static void handleDelayResp(PtpClock*, bool);
static void handlePDelayRespFollowUp(PtpClock*, bool);
static void handleManagement(PtpClock*, bool);
static void handleSignaling(PtpClock*, bool);

static void issueDelayReqOnTimerExpired(PtpClock*);
static void issueAnnounce(PtpClock*);
static void issueSync(PtpClock*);
static void issueFollowup(PtpClock*, const TimeInternal*);
static void issueDelayReq(PtpClock*);
static void issueDelayResp(PtpClock*, const TimeInternal*, const MsgHeader*);
static void issuePDelayReq(PtpClock*);
static void issuePDelayResp(PtpClock*, TimeInternal*, const MsgHeader*);
static void issuePDelayRespFollowUp(PtpClock*, const TimeInternal*, const MsgHeader*);
//static void issueManagement(const MsgHeader*,MsgManagement*,PtpClock*);

void addForeign(PtpClock *ptpClock,unsigned char *buf,MsgHeader *header, unsigned char localPreference, uint32_t sourceAddr);
bool respectUtcOffset(PtpClock * ptpClock);
//========== REV HISTORY==============
//2016.06.24 : Corrects in doState()
bool ptp_doInit(PtpClock *ptpClock);
//#ifdef PTPD_DBG
static char *stateString(uint8_t state)
{
	switch (state)
	{
		case PTP_INITIALIZING: return (char *) "PTP_INITIALIZING";
		case PTP_FAULTY: return (char *) "PTP_FAULTY";
		case PTP_DISABLED: return (char *) "PTP_DISABLED";
		case PTP_LISTENING: return (char *) "PTP_LISTENING";
		case PTP_PRE_MASTER: return (char *) "PTP_PRE_MASTER";
		case PTP_MASTER: return (char *) "PTP_MASTER";
		case PTP_PASSIVE: return (char *) "PTP_PASSIVE";
		case PTP_UNCALIBRATED: return (char *) "PTP_UNCALIBRATED";
		case PTP_SLAVE: return (char *) "PTP_SLAVE";
		default: break;
	}
	return (char *) "UNKNOWN";
}
//#endif
/* state change wrapper to allow for event generation / control when changing state */
/*
void setPortState(PtpClock *ptpClock, uint8_t state)//Enumeration8 state)
{

    if(ptpClock == NULL) {
    	return;
    }
    if(ptpClock->portDS.portState != state) {
	ptpClock->portDS.lastPortState = ptpClock->portDS.portState;
	//DBG("State change from %s to %s\n", portState_getName(ptpClock->portDS.lastPortState), portState_getName(state));
    }
    // "expected state" checks
    ptpClock->portDS.portState = state;
    if(ptpClock->defaultDS.slaveOnly) {
	    //SET_ALARM(ALRM_PORT_STATE, state != PTP_SLAVE);
    } else if(ptpClock->defaultDS.clockQuality.clockClass < 128) {
	    //SET_ALARM(ALRM_PORT_STATE, state != PTP_MASTER && state != PTP_PASSIVE );
    }
}
*/

/* state change wrapper to allow for event generation / control when changing state */
void setPortState(PtpClock *ptpClock, unsigned char state)//Enumeration8 state)
{

    if(ptpClock == NULL) {
    	return;
    }
    if(ptpClock->portDS.portState != state) {
    	ptpClock->portDS.lastPortState = ptpClock->portDS.portState;
    	DBG("State change from %s to %s\n", stateString(ptpClock->portDS.lastPortState), stateString(state));
    }

    // "expected state" checks
    ptpClock->portDS.portState = state;

    if(ptpClock->defaultDS.slaveOnly) {
	    printf("State != PTP_SLAVE\r\n");//SET_ALARM(ALRM_PORT_STATE, state != PTP_SLAVE);
    } else if(ptpClock->defaultDS.clockQuality.clockClass < 128) {
    	printf("state != PTP_MASTER && state != PTP_PASSIVE\r\n");//SET_ALARM(ALRM_PORT_STATE, state != PTP_MASTER && state != PTP_PASSIVE );
    }
}

/* Perform actions required when leaving 'port_state' and entering the new 'state' */
void stm_ptpd_toState(PtpClock *ptpClock, uint8_t state)
{
	ptpClock->messageActivity = TRUE;

	printf("ToState: leaving from state %s to %s\r\n", stateString(ptpClock->portDS.portState),stateString(state));

	// leaving from the current state
	switch (ptpClock->portDS.portState)
	{
		case PTP_MASTER:

			initClock(ptpClock);
			ptpd_timerStop(SYNC_INTERVAL_TIMER,ptpClock->itimer);
			ptpd_timerStop(ANNOUNCE_INTERVAL_TIMER,ptpClock->itimer);
			ptpd_timerStop(PDELAYREQ_INTERVAL_TIMER,ptpClock->itimer);
			ptpd_timerStop(DELAY_RECEIPT_TIMER, ptpClock->itimer); //Added YOON
			break;

		//case PTP_UNCALIBRATED:
		case PTP_SLAVE:

			ptpd_timerStop(ANNOUNCE_RECEIPT_TIMER,ptpClock->itimer);
			ptpd_timerStop(SYNC_RECEIPT_TIMER, ptpClock->itimer); //Added YOON
			ptpd_timerStop(DELAY_RECEIPT_TIMER, ptpClock->itimer); //Added YOON
			switch (ptpClock->portDS.delayMechanism){
				case E2E:
					ptpd_timerStop(DELAYREQ_INTERVAL_TIMER,ptpClock->itimer);
					break;
				case P2P:
					ptpd_timerStop(PDELAYREQ_INTERVAL_TIMER,ptpClock->itimer);
					break;
				default:
					/* none */
					break;
			}
			initClock(ptpClock);
			break;

		case PTP_PASSIVE:

			ptpd_timerStop(PDELAYREQ_INTERVAL_TIMER,ptpClock->itimer);
			ptpd_timerStop(ANNOUNCE_RECEIPT_TIMER,ptpClock->itimer);
			ptpd_timerStop(DELAY_RECEIPT_TIMER, ptpClock->itimer); //Added YOON
			break;

		case PTP_LISTENING:

			initClock(ptpClock);
			ptpd_timerStop(ANNOUNCE_RECEIPT_TIMER,ptpClock->itimer);
			ptpd_timerStop(SYNC_RECEIPT_TIMER, ptpClock->itimer); //Added YOON
			ptpd_timerStop(DELAY_RECEIPT_TIMER, ptpClock->itimer); //Added YOON

			// we're leaving LISTENING - reset counter --  2016.09.30
            if(state != PTP_LISTENING) {
	                    ptpClock->listenCount = 0;
            }

			break;
		default:
			break;
	}

	DBG("entering state %s\r\n", stateString(state));

	// Entering new state
	switch (state)
	{
		case PTP_INITIALIZING:
			setPortState(ptpClock, PTP_INITIALIZING);
			break;

		case PTP_FAULTY:
			setPortState(ptpClock, PTP_FAULTY);//
			break;

		case PTP_DISABLED:
			//well, theoretically we're still in the previous state, so we're not in breach of standard
			ptpClock->foreignMasterDS.bestMaster = NULL;
			/* see? NOW we're in disabled state */
			setPortState(ptpClock, PTP_DISABLED);	//
			break;

		case PTP_LISTENING:

			// in Listening state, make sure we don't send anything.
			//Instead we just expect/wait for announces (started below)
			ptpd_timerStop(SYNC_INTERVAL_TIMER, ptpClock->itimer);
			ptpd_timerStop(ANNOUNCE_RECEIPT_TIMER,ptpClock->itimer);
			ptpd_timerStop(SYNC_RECEIPT_TIMER, ptpClock->itimer); //Added YOON
			ptpd_timerStop(DELAY_RECEIPT_TIMER, ptpClock->itimer); //Added YOON
			ptpd_timerStop(DELAYREQ_INTERVAL_TIMER,ptpClock->itimer);
			ptpd_timerStop(PDELAYREQ_INTERVAL_TIMER,ptpClock->itimer);

			// if we're ignoring announces (disable_bmca), go straight to master
			if(ptpClock->defaultDS.clockQuality.clockClass <= 127 && ptpClock->rtOpts->disableBMCA) {
				DBG("unicast master only and ignoreAnnounce: going into MASTER state\n");
				ptpClock->foreignMasterDS.foreign_record_i = 0;
				ptpClock->foreignMasterDS.number_foreign_records = 0;
				m1(ptpClock);
				stm_ptpd_toState( ptpClock, PTP_MASTER);
				break;
			}

			//  Count how many _unique_ timeouts happen to us.
			//  If we were already in Listen mode, then do not count this as a seperate reset, but stil do a new IGMP refresh
			if (ptpClock->portDS.portState != PTP_LISTENING) {
					ptpClock->resetCount++;
			} else {
                    ptpClock->listenCount++;
                    if( ptpClock->listenCount >= ptpClock->rtOpts->maxListen ) {
			             printf("Warning: Still in LISTENING after %d restarts - restarting transports\n", ptpClock->rtOpts->maxListen);
                         stm_ptpd_toState(ptpClock, PTP_FAULTY);
                         ptpClock->listenCount = 0;
                         break;
                    }
           }

			// Revert to the original DelayReq interval, and ignore the one for the last master
			ptpClock->portDS.logMinDelayReqInterval = DEFAULT_DELAYREQ_INTERVAL;//ptpClock->rtOpts->initial_delayreq;

			//Start Announce Receipt timer.
			ptpd_timerStart(ANNOUNCE_RECEIPT_TIMER, (ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)), ptpClock->itimer);

			//ptpClock->announceTimeouts = 0; //?? TBD YOON
			ptpClock->foreignMasterDS.bestMaster = NULL;

			if (ptpClock->portDS.portState != state) {
					setPortState(ptpClock, PTP_LISTENING);
			} else {
					setPortState(ptpClock, PTP_LISTENING);
			}
			break;

		case PTP_MASTER://5

			ptpClock->portDS.logMinDelayReqInterval = DEFAULT_DELAYREQ_INTERVAL; // it may be changed during slave state
			ptpd_timerStart(SYNC_INTERVAL_TIMER, pow2ms(ptpClock->portDS.logSyncInterval), ptpClock->itimer);
			ptpd_timerStart(ANNOUNCE_INTERVAL_TIMER, pow2ms(ptpClock->portDS.logAnnounceInterval), ptpClock->itimer);
			ptpd_timerStart(PDELAYREQ_INTERVAL_TIMER, getRand(pow2ms(ptpClock->portDS.logMinPDelayReqInterval) + 1), ptpClock->itimer);

			switch (ptpClock->portDS.delayMechanism){
				case E2E:
						/* none */
						break;
				case P2P:
					ptpd_timerStart(DELAY_RECEIPT_TIMER,
							max((ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)),
							MISSED_MESSAGES_MAX * (pow2ms(ptpClock->portDS.logMinPDelayReqInterval))), ptpClock->itimer);
						break;
				default:
						break;
			}
			setPortState(ptpClock, PTP_MASTER);
			ptpClock->foreignMasterDS.bestMaster = NULL;
			break;

		case PTP_PASSIVE:
			ptpd_timerStart(ANNOUNCE_RECEIPT_TIMER, (ptpClock->portDS.announceReceiptTimeout)*(pow2ms(ptpClock->portDS.logAnnounceInterval)), ptpClock->itimer);
			//ptpd_timerStart(PDELAYREQ_INTERVAL_TIMER, getRand(pow2ms(ptpClock->portDS.logMinPDelayReqInterval + 1)), ptpClock->itimer);

			if (ptpClock->portDS.delayMechanism == P2P){
				ptpd_timerStart(PDELAYREQ_INTERVAL_TIMER, getRand(pow2ms(ptpClock->portDS.logMinPDelayReqInterval + 1)), ptpClock->itimer);
				ptpd_timerStart(DELAY_RECEIPT_TIMER,
					max((ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)),
					MISSED_MESSAGES_MAX * (pow2ms(ptpClock->portDS.logMinPDelayReqInterval))), ptpClock->itimer);
			}
			setPortState(ptpClock, PTP_PASSIVE);
			p1(ptpClock); //TBD -- Only for PASSIVE STATE
			break;

		case PTP_UNCALIBRATED://7

			setPortState(ptpClock, PTP_UNCALIBRATED);

/*			ptpd_timerStart(ANNOUNCE_RECEIPT_TIMER, (ptpClock->portDS.announceReceiptTimeout)*(pow2ms(ptpClock->portDS.logAnnounceInterval)), ptpClock->itimer);
			switch (ptpClock->portDS.delayMechanism){
				case E2E:
					ptpd_timerStart(DELAYREQ_INTERVAL_TIMER, getRand(pow2ms(ptpClock->portDS.logMinDelayReqInterval + 1)), ptpClock->itimer);
						break;
				case P2P:
					ptpd_timerStart(PDELAYREQ_INTERVAL_TIMER, getRand(pow2ms(ptpClock->portDS.logMinPDelayReqInterval + 1)), ptpClock->itimer);
					ptpd_timerStart(DELAY_RECEIPT_TIMER,
												max((ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)),
												MISSED_MESSAGES_MAX * (pow2ms(ptpClock->portDS.logMinPDelayReqInterval))), ptpClock->itimer);

						break;
				default:
						break;
			}
			ptpClock->portDS.portState = PTP_UNCALIBRATED;
*/
			break;

		case PTP_SLAVE:

			initClock(ptpClock);

			// restore the observed drift value using the selected method,
			// reset on failure or when -F 0 (default) is used, don't inform user
			restoreDrift(ptpClock, TRUE);

			ptpClock->waitingForFollowUp = FALSE;
			ptpClock->waitingForDelayResp = FALSE; //YOON 2016.09.30

			ptpClock->timestamp_delayReqSend.seconds = ptpClock->timestamp_delayReqSend.nanoseconds =0;
			ptpClock->timestamp_delayReqRecieve.seconds = ptpClock->timestamp_delayReqRecieve.nanoseconds =0;

			//set rx timers for expecting announce and sync msgs from the master anywhere.
			ptpd_timerStart(ANNOUNCE_RECEIPT_TIMER, (ptpClock->portDS.announceReceiptTimeout)*(pow2ms(ptpClock->portDS.logAnnounceInterval)), ptpClock->itimer);
			ptpd_timerStart(SYNC_RECEIPT_TIMER,
					max((ptpClock->portDS.announceReceiptTimeout)*(pow2ms(ptpClock->portDS.logAnnounceInterval)),MISSED_MESSAGES_MAX * (pow2ms(ptpClock->portDS.logSyncInterval))),
					ptpClock->itimer); //Added YOON

			if(ptpClock->portDS.delayMechanism == E2E){
				ptpd_timerStart(DELAY_RECEIPT_TIMER,
						max((ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)),	MISSED_MESSAGES_MAX * (pow2ms(ptpClock->portDS.logMinDelayReqInterval))),
						ptpClock->itimer);
			}else if(ptpClock->portDS.delayMechanism == P2P){
				ptpd_timerStart(DELAY_RECEIPT_TIMER,
						max((ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)),	MISSED_MESSAGES_MAX * (pow2ms(ptpClock->portDS.logMinPDelayReqInterval))),
						ptpClock->itimer);

			}

			// Previously, this state transition would start the delayreq timer immediately.
			// However, if this was faster than the first received sync, then the servo would drop the delayResp
			// Now, we only start the timer after we receive the first sync (in handle_sync())

			ptpClock->WaitingForSync = TRUE; //ptpClock->syncWaiting = TRUE;
			ptpClock->waitingForDelayResp = TRUE;//delayRespWaiting = TRUE;
			//ptpClock->announceTimeouts = 0; //TBD
			setPortState(ptpClock, PTP_SLAVE);
			//ptpClock->followUpGap = 0; //TBD

			break;

/*
		case PTP_PRE_MASTER:

			// If you implement not ordinary clock, you can manage this code
			ptpd_timerStart(QUALIFICATION_TIMEOUT, pow2ms(DEFAULT_QUALIFICATION_TIMEOUT));
			ptpClock->portDS.portState = PTP_PRE_MASTER;
			break;
*/
		default:

			break;
	}
}


bool ptp_doInit(PtpClock *ptpClock)
{
	DBG("manufacturerIdentity: %s\r\n", MANUFACTURER_ID);

	/* initialize networking */
#if (PTP_PROTOCOL == IEEE1588V2)

	//netShutdown(&ptpClock->netPath);

	//if (!netInit(&ptpClock->netPath, ptpClock)){
	//	ERROR("pdp_doInit: failed to initialize network\r\n");
	//	return FALSE;
	//}else{	// initialize other stuff
		initData(ptpClock);

		ptpd_initTimer();
		initClock(ptpClock);
		//setupPIservo(&ptpClock->servo, rtOpts); //TBD
		// restore observed drift and inform user
		if(ptpClock->defaultDS.clockQuality.clockClass > 127)
				restoreDrift(ptpClock, FALSE); //call AdjFreq(0)

		m1(ptpClock);
		msgPackHeader(ptpClock, ptpClock->msgObuf);

		stm_ptpd_toState( ptpClock, PTP_LISTENING); //Added

		return TRUE;
	//}
#elif (PTP_PROTOCOL == IEEE8021AS)
	//netShutdown8021as(&ptpClock->netPath); //net.c

	//if (!netInit8021(&ptpClock->netPath, ptpClock)){
	//	ERROR("ptp_doInit: failed to initialize network\r\n");
	//	return FALSE;
	//}
	//else	{
		/* initialize other stuff */
		initData(ptpClock);
		ptpd_initTimer();
		initClock(ptpClock);
		// restore observed drift and inform user
		if(ptpClock->defaultDS.clockQuality.clockClass > 127)
				restoreDrift(ptpClock, FALSE); //call AdjFreq(0)
		m1(ptpClock);
		msgPackHeader(ptpClock, ptpClock->msgObuf);

		stm_ptpd_toState( ptpClock, PTP_LISTENING); //Added

		return TRUE;
	//}
#endif
}

// Handle actions and events for 'port_state'
// Also, it handles received msgs.
void stm_ptpd_doState(PtpClock *ptpClock)
{
	unsigned char recommendedState;
	ptpClock->messageActivity = FALSE;

	// Process record_update (BMC algorithm) before everything else
	switch (ptpClock->portDS.portState)
	{
		case PTP_LISTENING:
		case PTP_PASSIVE: //related with bmc
		case PTP_SLAVE:
		case PTP_MASTER:

			// State decision Event
			// If we received a valid Announce message and can use it (record_update),
			//or we received a SET management message that changed an attribute in ptpClock,
			// then run the BMC algorithm
			if (ptpClock->foreignMasterDS.record_update){//if (getFlag(ptpClock->events, STATE_DECISION_EVENT)){
				DBGV("event STATE_DECISION_EVENT\r\n");
				ptpClock->foreignMasterDS.record_update = FALSE;//clearFlag(ptpClock->events, STATE_DECISION_EVENT);
				recommendedState = bmc(ptpClock);
				DBGV("recommended state1 %s\r\n", stateString(recommendedState));
				if (recommendedState != ptpClock->portDS.portState){
					stm_ptpd_toState(ptpClock, recommendedState);
				}
			}
			break;

		default:
			break;
	}

	switch (ptpClock->portDS.portState)
	{
		case PTP_FAULTY:

			/* Imaginary troubleshooting */
			DBG("event FAULT_CLEARED for state PTP_FAULT\r\n");
			stm_ptpd_toState(ptpClock, PTP_INITIALIZING);
			return;

		case PTP_LISTENING:
		case PTP_UNCALIBRATED:
			// passive mode behaves like the SLAVE state,
			//in order to wait for the announce timeout of the current active master
		case PTP_SLAVE:
		case PTP_PASSIVE://related with bmc

			handle_ptp_rcvmsg(ptpClock);//handle rec msg if it has been arrived.

			// handle SLAVE timers:
			//   - No Announce message was received
			//   - Time to send new delayReq  (miss of delayResp is not monitored explicitly)

			if (ptpd_timerExpired(ANNOUNCE_RECEIPT_TIMER,ptpClock->itimer))
			{
				DBGV("event ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES for state %s\r\n", stateString(ptpClock->portDS.portState));

				if (!(ptpClock->defaultDS.slaveOnly || ptpClock->defaultDS.clockQuality.clockClass == 255))
				{
					ptpClock->foreignMasterDS.number_foreign_records = 0;
					ptpClock->foreignMasterDS.foreign_record_i = 0;
					ptpClock->foreignMasterDS.bestMaster = NULL;

					m1(ptpClock);
					//ptpClock->recommendedState = PTP_MASTER;
					//DBGV("recommending state3 %s\r\n", stateString(ptpClock->recommendedState));
					stm_ptpd_toState(ptpClock, PTP_MASTER);
				}
				else if (ptpClock->portDS.portState != PTP_LISTENING){
					if (!ptpClock->foreignMasterDS.bestMaster->disqualified) {
						ptpClock->foreignMasterDS.bestMaster->disqualified = TRUE;
						//WARNING("GM announce timeout, disqualified current best GM\n");
						//ptpClock->counters.announceTimeouts++;
					}
					stm_ptpd_toState(ptpClock, PTP_LISTENING);
				}
			}

			// Reset the slave if clock update timeout configured
			if ( ptpClock->portDS.portState == PTP_SLAVE && (ptpClock->rtOpts->clockUpdateTimeout > 0) &&
					ptpd_timerExpired(CLOCK_UPDATE_TIMER,ptpClock->itimer))
			{
				if(ptpClock->panicMode || ptpClock->rtOpts->servo.noAdjust) {
					ptpd_timerStart(CLOCK_UPDATE_TIMER, ptpClock->rtOpts->clockUpdateTimeout,ptpClock->itimer);
				} else {
				    printf("No offset updates in %d seconds - resetting slave\n", ptpClock->rtOpts->clockUpdateTimeout);
				    stm_ptpd_toState(ptpClock,PTP_LISTENING);
				}
			}

			if(ptpClock->portDS.portState==PTP_SLAVE && ptpClock->rtOpts->calibrationDelay && !ptpClock->isCalibrated) {
				if(ptpd_timerExpired(CALIBRATION_DELAY_TIMER, ptpClock->itimer)) {
					ptpClock->isCalibrated = TRUE;
					if(ptpClock->clockControl.granted) {
						printf("Offset computation now calibrated, enabled clock control\n");
					} else {
						printf("Offset computation now calibrated\n");
					}
				} else if(!ptpd_timerRunning(CALIBRATION_DELAY_TIMER, ptpClock->itimer)) {
					ptpd_timerStart(CALIBRATION_DELAY_TIMER, ptpClock->rtOpts->calibrationDelay, ptpClock->itimer);
				}
			}

			if(ptpClock->portDS.delayMechanism == P2P){
				if (ptpd_timerExpired(PDELAYREQ_INTERVAL_TIMER,ptpClock->itimer)){
					issuePDelayReq(ptpClock);
					//ADD YOON - NEED Check
					//ptpd_timerStart(PDELAYREQ_INTERVAL_TIMER,
					//		getRand(pow2ms(ptpClock->portDS.logMinPDelayReqInterval + 1)),
					//		ptpClock->itimer);
				}
			}else{ //E2E
				//ptpd_timerUpdate(ptpClock->itimer); //YOON Added
				if (ptpd_timerExpired(DELAYREQ_INTERVAL_TIMER,ptpClock->itimer)){
					issueDelayReq(ptpClock);
					//ADD YOON - NEED Check
					//ptpd_timerStart(DELAYREQ_INTERVAL_TIMER,
					//	getRand(pow2ms(ptpClock->portDS.logMinDelayReqInterval + 1)),
					//	ptpClock->itimer);
				}
			}

			break;

		case PTP_MASTER:

			// handle MASTER timers:
			//   - Time to send new Announce(s)
			//   - Time to send new PathDelay
			//   - Time to send new Sync(s) (last order - so that follow-up always follows sync
			//     in two-step mode: improves interoperability
			//   (DelayResp has no timer - as these are sent and retransmitted by the slaves)

			// if we have an offset from some source, we assume it's valid
			if(ptpClock->clockStatus.utcOffset != 0) {
				ptpClock->timePropertiesDS.currentUtcOffset = ptpClock->clockStatus.utcOffset;
				ptpClock->timePropertiesDS.currentUtcOffsetValid = TRUE;

			}

			if (ptpd_timerExpired(SYNC_INTERVAL_TIMER,ptpClock->itimer))
			{
					//DBGV("event SYNC_INTERVAL_TIMEOUT_EXPIRES in state PTP_MASTER\r\n");
					//printf("event SYNC_INTERVAL_TIMEOUT_EXPIRES in state PTP_MASTER\r\n");
					issueSync(ptpClock);
			}

			if (ptpd_timerExpired(ANNOUNCE_INTERVAL_TIMER,ptpClock->itimer))
			{
					//DBGV("event ANNOUNCE_INTERVAL_TIMEOUT_EXPIRES for state PTP_MASTER\r\n");
					issueAnnounce(ptpClock);
			}

			if(ptpClock->portDS.delayMechanism  == P2P){ //if(!rtOpts->delayMechanism == P2P){ //add 2016.6.24
				if (ptpd_timerExpired(PDELAYREQ_INTERVAL_TIMER,ptpClock->itimer))
					issuePDelayReq(ptpClock);
			}
			//issueDelayReqTimerExpired(ptpClock); //remove 2016.6.24

			// TODO: why is handle() below expiretimer, while in slave is the opposite
			handle_ptp_rcvmsg(ptpClock);

			if(ptpClock->defaultDS.slaveOnly || ptpClock->defaultDS.clockQuality.clockClass == 255)
				stm_ptpd_toState(ptpClock, PTP_LISTENING);

			break;

		case PTP_DISABLED:
			handle_ptp_rcvmsg(ptpClock);

			//TBD YOON
			//if(!ptpClock->disabled) {
			//    stm_ptpd_toState( ptpClock, PTP_LISTENING);
			//}

			break;

		/*	YOON 2016.09.30
		case PTP_INITIALIZING:

			if (ptp_doInit(ptpClock) == TRUE)
			{
				stm_ptpd_toState(ptpClock, PTP_LISTENING);
			}
			else
			{
				stm_ptpd_toState(ptpClock, PTP_FAULTY);
			}

			break;
	  */
		default:
			DBG("stm_ptpd_doState: do unrecognized state %d\n", ptpClock->portDS.portState);
			break;
	}
	if (ptpd_timerExpired(YOON_TIMER,ptpClock->itimer)){ //YOON
		//ptpd_timerStart(YOON_TIMER,2000,ptpClock->itimer); //Restarted automatically in timer_update()
		//displayStats(ptpClock); //moved to IRQ PPS

	}
}

extern void *netQPeek(BufQueue *queue);

static bool isFromCurrentParent(const PtpClock *ptpClock, const MsgHeader* header)
{
	return(!memcmp(
			ptpClock->parentDS.parentPortIdentity.clockIdentity,header->sourcePortIdentity.clockIdentity,CLOCK_IDENTITY_LENGTH)
			&& (ptpClock->parentDS.parentPortIdentity.portNumber == header->sourcePortIdentity.portNumber
		)
	);
}
// apply any corrections and manglings required to the timestamp
static void timestampCorrection(PtpClock *ptpClock, TimeInternal *timeStamp)
{

	TimeInternal fudge = {0,0};
	RunTimeOpts *rtOpts;
	rtOpts = ptpClock->rtOpts;
/*
	if(rtOpts->leapSecondHandling == LEAP_SMEAR && (ptpClock->leapSecondPending)) {
	DBG("Leap second smear: correction %.09f ns, seconds to midnight %f, leap smear period %d\n", ptpClock->leapSmearFudge,
		secondsToMidnight(), rtOpts->leapSecondSmearPeriod);
	    if(secondsToMidnight() <= rtOpts->leapSecondSmearPeriod) {
		ptpClock->leapSmearFudge = 1.0 - (secondsToMidnight() + 0.0) / (rtOpts->leapSecondSmearPeriod+0.0);
		if(ptpClock->timePropertiesDS.leap59) {
		    ptpClock->leapSmearFudge *= -1;
		}
		fudge = doubleToTimeInternal(ptpClock->leapSmearFudge);
	    }
	}
*/
	if(ptpClock->portDS.portState == PTP_SLAVE && ptpClock->leapSecondPending && !ptpClock->leapSecondInProgress) {
	    addTime(timeStamp, timeStamp, &fudge);
	}

}

// Check and handle received messages
//processMessage()
static void handle_ptp_rcvmsg(PtpClock *ptpClock)
{

		int ret;
		bool  isFromSelf;
		TimeInternal timeStamp = { 0, 0 };
		NetPath *pnetPath; //YOON
		struct pbuf *pEvent, *pGeneral; //YOON

		/* Get the next buffer from the APP queue. */
		pnetPath = &ptpClock->netPath;
		if (!ptpClock->messageActivity){
				ret = netSelect(&ptpClock->netPath, 0);
				if (ret < 0){ //never enters.
						ERROR("handle: failed to poll sockets\r\n");
						stm_ptpd_toState(ptpClock, PTP_FAULTY);
						return;
				}else if (!ret){ //if not received.
						//printf("handle: nothing\r\n");
						return;
				}
		}

		//=========== handle received packets ====================================

		//DBGVV("handle: something\r\n");

		/*
		//=== WE FOUND AN ERROR IN IN-SEQUENCE HANDLING ON NetQGet()
		// Handle first Receive an event.
		ptpClock->msgIbufLength = netRecvEvent(&ptpClock->netPath, ptpClock->msgIbuf, &timeStamp);
		// local time is not UTC, we can calculate UTC on demand, otherwise UTC time is not used
		// timeStamp.seconds += ptpClock->timePropertiesDS.currentUtcOffset;
		DBGV("handle: netRecvEvent returned %d\r\n", ptpClock->msgIbufLength);

		if (ptpClock->msgIbufLength < 0){
				ERROR("handle: failed to receive on the event socket\r\n");
				stm_ptpd_toState(ptpClock, PTP_FAULTY);
				return;
		}
		else if (!ptpClock->msgIbufLength){	// Next. handle Receive a general packet.
				ptpClock->msgIbufLength = netRecvGeneral(&ptpClock->netPath, ptpClock->msgIbuf, &timeStamp);
				DBGV("handle: netRecvGeneral returned %d\n", ptpClock->msgIbufLength);

				if (ptpClock->msgIbufLength < 0){
						ERROR("handle: failed to receive on the general socket\n");
						stm_ptpd_toState(ptpClock, PTP_FAULTY);
						return;
				}
				else if (!ptpClock->msgIbufLength)
						return;
		}
		*/
		pEvent = (struct pbuf*)netQPeek(&pnetPath->eventQ);
		pGeneral = (struct pbuf*)netQPeek(&pnetPath->generalQ);

		if((pEvent != NULL) && (pGeneral != NULL))//Both queues have packets.
		{
			// compare both sequence numbers
			if(pEvent->rxseq == pGeneral->rxseq){ //Normal situation. Handle event(SYN) first ===
				ptpClock->msgIbufLength = netRecvEvent(&ptpClock->netPath, ptpClock->msgIbuf, &timeStamp);
				if (ptpClock->msgIbufLength < 0){ //check error
					printf("handle: failed to receive on the event socket\r\n");
					stm_ptpd_toState(ptpClock, PTP_FAULTY);
					return;
				}
				else if (ptpClock->msgIbufLength == 0) //really happend?
					return;
			}else{ //Different in Abnormal situation.(e.g.)--SYN(1) ----[Followup(1)SYN(2)] ------ Followup(2)--
				//handle the general packet first.
				ptpClock->msgIbufLength = netRecvGeneral(&ptpClock->netPath, ptpClock->msgIbuf, &timeStamp);
				if (ptpClock->msgIbufLength < 0){ //check error
					printf("handle: failed to receive on the general socket\r\n");
					stm_ptpd_toState(ptpClock, PTP_FAULTY);
					return;
				}
				else if (ptpClock->msgIbufLength == 0)//really happend?
					return;
			}
/*
			if(pEvent->rxseq <= pGeneral->rxseq){ // compare receive sequence number: Handle first Receive an event. ===
				ptpClock->msgIbufLength = netRecvEvent(&ptpClock->netPath, ptpClock->msgIbuf, &timeStamp);
				if (ptpClock->msgIbufLength < 0){
					ERROR("handle: failed to receive on the event socket\r\n");
					stm_ptpd_toState(ptpClock, PTP_FAULTY);
					return;
				}else if (!ptpClock->msgIbufLength){	// Next. handle Receive a general packet.
					ptpClock->msgIbufLength = netRecvGeneral(&ptpClock->netPath, ptpClock->msgIbuf, &timeStamp);
					if (ptpClock->msgIbufLength < 0){
						ERROR("handle: failed to receive on the general socket\r\n");
						stm_ptpd_toState(ptpClock, PTP_FAULTY);
						return;
					}else if (!ptpClock->msgIbufLength)	return;
					//======
				}
			}else{	// Handle first Receive a General. ===
				ptpClock->msgIbufLength = netRecvGeneral(&ptpClock->netPath, ptpClock->msgIbuf, &timeStamp);
				if (ptpClock->msgIbufLength < 0){
					ERROR("handle: failed to receive on the General socket\r\n");
					stm_ptpd_toState(ptpClock, PTP_FAULTY);
					return;
				}else if (!ptpClock->msgIbufLength){	// Next. handle Receive a general packet.
					ptpClock->msgIbufLength = netRecvEvent(&ptpClock->netPath, ptpClock->msgIbuf, &timeStamp);
					if (ptpClock->msgIbufLength < 0){
						ERROR("handle: failed to receive on the Event socket\r\n");
						stm_ptpd_toState(ptpClock, PTP_FAULTY);
						return;
					}else if (!ptpClock->msgIbufLength)	return;
					//======
				}
			}
*/
		}else{
			// Check and Handle first received an event.
			ptpClock->msgIbufLength = netRecvEvent(&ptpClock->netPath, ptpClock->msgIbuf, &timeStamp);
			if (ptpClock->msgIbufLength < 0){
				printf("handle: failed to receive on the event socket\r\n");
				stm_ptpd_toState(ptpClock, PTP_FAULTY);
				return;
			}
			else if (ptpClock->msgIbufLength == 0)
			{	// Next. handle Receive a general packet.
				ptpClock->msgIbufLength = netRecvGeneral(&ptpClock->netPath, ptpClock->msgIbuf, &timeStamp);
				if (ptpClock->msgIbufLength < 0){
					printf("handle: failed to receive on the general socket\r\n");
					stm_ptpd_toState(ptpClock, PTP_FAULTY);
					return;
				}
				else if (ptpClock->msgIbufLength == 0){
					printf("handle: ptpClock->msgIbufLength == 0.\r\n");
					return;
				}
			}
		}

		//Check its validity.
		ptpClock->messageActivity = TRUE;

		if (ptpClock->msgIbufLength < HEADER_LENGTH){
				printf("handle: message shorter than header length\r\n");
				stm_ptpd_toState(ptpClock, PTP_FAULTY);
				return;
		}

		msgUnpackHeader(ptpClock->msgIbuf, &ptpClock->msgTmpHeader);
		//printf("handle: unpacked message type %d\r\n", ptpClock->msgTmpHeader.messageType);

		//Version Check
		if (ptpClock->msgTmpHeader.versionPTP != ptpClock->portDS.versionNumber){
				printf("handle: ignore version %d message\r\n", ptpClock->msgTmpHeader.versionPTP);
				return;
		}

		//Domain Check
		if (ptpClock->msgTmpHeader.domainNumber != ptpClock->defaultDS.domainNumber){
				printf("handle: ignore message from domainNumber %d\r\n", ptpClock->msgTmpHeader.domainNumber);
				return;
		}

	    timestampCorrection(ptpClock, &timeStamp);

		// Spec 9.5.2.2
		isFromSelf = isSamePortIdentity(&ptpClock->portDS.portIdentity,	&ptpClock->msgTmpHeader.sourcePortIdentity);
	    //isFromSelf = !cmpPortIdentity(&ptpClock->portDS.portIdentity, &ptpClock->msgTmpHeader.sourcePortIdentity);

		// Subtract the inbound latency adjustment
		// if it is not a loop back and the time stamp seems reasonable
		if (!isFromSelf && timeStamp.seconds > 0)
				subTime(&timeStamp, &timeStamp, &ptpClock->inboundLatency);

		switch (ptpClock->msgTmpHeader.messageType)
		{

		case ANNOUNCE:
				handleAnnounce(ptpClock, isFromSelf);
				break;

		case SYNC:
				handleSync(ptpClock, &timeStamp, isFromSelf);
				break;

		case FOLLOW_UP:
				handleFollowUp(ptpClock, isFromSelf);
				break;

		case DELAY_REQ:
				handleDelayReq(ptpClock, &timeStamp, isFromSelf);
				break;

		case PDELAY_REQ:
				handlePDelayReq(ptpClock, &timeStamp, isFromSelf);
				break;

		case DELAY_RESP:
				handleDelayResp(ptpClock, isFromSelf);
				break;

		case PDELAY_RESP:
				handlePDelayResp(ptpClock, &timeStamp, isFromSelf);
				break;

		case PDELAY_RESP_FOLLOW_UP:
				handlePDelayRespFollowUp(ptpClock, isFromSelf);
				break;

		case MANAGEMENT:
				handleManagement(ptpClock, isFromSelf);
				break;

		case SIGNALING:
				handleSignaling(ptpClock, isFromSelf);
				break;

		default:
				DBG("handle: unrecognized message %d\r\n", ptpClock->msgTmpHeader.messageType);
				break;
		}
}

/* spec 9.5.3 */
static void handleAnnounce(PtpClock *ptpClock, bool isFromSelf)
{
	bool  isFromCurrentParent = FALSE;
	unsigned char localPreference = LOWEST_LOCALPREFERENCE;

	DBGV("handleAnnounce: received in state %s\r\n", stateString(ptpClock->portDS.portState));

	if (ptpClock->msgIbufLength < ANNOUNCE_LENGTH){
			ERROR("handleAnnounce: short message\r\n");
			stm_ptpd_toState(ptpClock, PTP_FAULTY);
			return;
	}

	switch (ptpClock->portDS.portState)	{
		case PTP_INITIALIZING:
		case PTP_FAULTY:
		case PTP_DISABLED:

			printf("handleAnnounce: disregard(%s)\r\n",stateString(ptpClock->portDS.portState));
			break;

		case PTP_UNCALIBRATED:
		case PTP_SLAVE:

			if (isFromSelf){
					DBGV("handleAnnounce: ignore from self\r\n");
					return;
			}

			/* Valid announce message is received : BMC algorithm will be executed */
			ptpClock->foreignMasterDS.record_update = TRUE;//setFlag(ptpClock->events, STATE_DECISION_EVENT);

			isFromCurrentParent = isSamePortIdentity(
					&ptpClock->parentDS.parentPortIdentity,
					&ptpClock->msgTmpHeader.sourcePortIdentity);

			if (isFromCurrentParent){
				msgUnpackAnnounce(ptpClock->msgIbuf, &ptpClock->msgTmp.announce);

				// update datasets (file bmc.c)
				s1(ptpClock, &ptpClock->msgTmpHeader, &ptpClock->msgTmp.announce);

				// update current master in the fmr as well
				memcpy(&ptpClock->foreignMasterDS.bestMaster->header,
						&ptpClock->msgTmpHeader,sizeof(MsgHeader));
				memcpy(&ptpClock->foreignMasterDS.bestMaster->announce,
				       &ptpClock->msgTmp.announce,sizeof(MsgAnnounce));

				// Reset Timer handling Announce receipt timeout - in anyDomain, time out from current domain first
				if(ptpClock->msgTmpHeader.domainNumber == ptpClock->defaultDS.domainNumber) {
					ptpd_timerStart(ANNOUNCE_RECEIPT_TIMER, (ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)), ptpClock->itimer);
				}
			}
			else{
				printf("handleAnnounce: from another foreign master\r\n");
				// addForeign takes care  of AnnounceUnpacking
				//the actual decision to change masters is only done in stm_ptpd_doState() / record_update == TRUE / bmc()
				addForeign(ptpClock, ptpClock->msgIbuf, &ptpClock->msgTmpHeader, localPreference,ptpClock->netPath.lastSourceAddr);
			}

			break;

			// Passive case: previously, this was handled in the default, just like the master case.
			//This the announce would call addForeign(), but NOT reset the timer, so after 12s it would expire and we would come alive periodically
			// This code is now merged with the slave case to reset the timer, and call addForeign() if it's a third master
		case PTP_PASSIVE:
			if (isFromSelf)	return;
			//if(rtOpts->requireUtcValid && !IS_SET(header->flagField1, UTCV)) {
			//		ptpClock->counters.ignoredAnnounce++;
			//	return;
			//}

			// Valid announce message is received : BMC algorithm will be executed
			//ptpClock->counters.announceMessagesReceived++;
			ptpClock->foreignMasterDS.record_update = TRUE;
			isFromCurrentParent = isSamePortIdentity(
					&ptpClock->parentDS.parentPortIdentity,
					&ptpClock->msgTmpHeader.sourcePortIdentity);

			if (isFromCurrentParent) {
				msgUnpackAnnounce(ptpClock->msgIbuf, &ptpClock->msgTmp.announce);

				// TODO: not in spec: datasets should not be updated by another master.
				// this is the reason why we are PASSIVE and not SLAVE this should be p1(ptpClock, rtOpts);
				// update datasets (file bmc.c)
				s1(ptpClock, &ptpClock->msgTmpHeader, &ptpClock->msgTmp.announce);//s1(header,&ptpClock->msgTmp.announce,ptpClock, rtOpts);

				printf("___ Announce: received Announce from current Master, so reset the Announce timer\n\n");

				// Reset Timer handling Announce receipt timeout: different domain my get here when using anyDomain.
				ptpd_timerStart(ANNOUNCE_RECEIPT_TIMER, (ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)), ptpClock->itimer);
			} else {
						//addForeign takes care of AnnounceUnpacking
						// the actual decision to change masters is only done in  stm_ptpd_doState() / record_update == TRUE / bmc()
						// the original code always called: addforeign(new master) + timerstart(announce)

				printf("___ Announce: received Announce from another master, will add to the list, as it might be better\n\n");
				DBGV("this is to be decided immediatly by bmc())\n\n");
				addForeign(ptpClock, ptpClock->msgIbuf, &ptpClock->msgTmpHeader, localPreference,ptpClock->netPath.lastSourceAddr);
			}
			break;
		case PTP_MASTER:
		//case PTP_PRE_MASTER:
		case PTP_LISTENING:
		default :
			if (isFromSelf) {
				DBGV("HandleAnnounce : Ignore message from self \n");
				return;
			}
			if(ptpClock->rtOpts->requireUtcValid && !getFlag(ptpClock->msgTmpHeader.flagField1, FLAG1_UTC_OFFSET_VALID)) {//!IS_SET(ptpClock->msgTmpHeader.flagField1, UTCV)) {
			//	ptpClock->counters.ignoredAnnounce++;
				return;
			}
			DBGV("handleAnnounce: from another foreign master\r\n");
			msgUnpackAnnounce(ptpClock->msgIbuf, &ptpClock->msgTmp.announce);

			// Valid announce message is received : BMC algorithm will be executed
			//addForeign(ptpClock, &ptpClock->msgTmpHeader, &ptpClock->msgTmp.announce);
			addForeign(ptpClock, ptpClock->msgIbuf, &ptpClock->msgTmpHeader, localPreference,ptpClock->netPath.lastSourceAddr);
			ptpClock->foreignMasterDS.record_update = TRUE;    // run BMC() as soon as possible //setFlag(ptpClock->events, STATE_DECISION_EVENT);
			break;
	}
}

static void handleSync(PtpClock *ptpClock, TimeInternal *time, bool isFromSelf)
{
	TimeInternal originTimestamp;
	TimeInternal correctionField;
	bool  isFromCurrentParent = FALSE;
	TimeInternal tmpTime;//for debug

	//printf("handleSync: received in state %s\r\n", stateString(ptpClock->portDS.portState));

	if (ptpClock->msgIbufLength < SYNC_LENGTH){
		ERROR("handleSync: short message\r\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
		return;
	}

	switch (ptpClock->portDS.portState)	{
		case PTP_INITIALIZING:
		case PTP_FAULTY:
		case PTP_DISABLED:
		case PTP_PASSIVE:
		case PTP_LISTENING:
			//printf("handleSync: disregard(%s)\r\n",stateString(ptpClock->portDS.portState));
			break;

		case PTP_UNCALIBRATED:
		case PTP_SLAVE:

			if (isFromSelf)	{
				//DBGV("handleSync: ignore from self\r\n");
				break;
			}

			isFromCurrentParent = isSamePortIdentity(
					&ptpClock->parentDS.parentPortIdentity,
					&ptpClock->msgTmpHeader.sourcePortIdentity);

			if (!isFromCurrentParent){
				//DBGV("handleSync: ignore from another master\r\n");
				break;
			}

			ptpd_timerStart(SYNC_RECEIPT_TIMER, max((ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)),
						    MISSED_MESSAGES_MAX * (pow2ms(ptpClock->portDS.logSyncInterval))),
						    ptpClock->itimer
						);

			//========(IMPORTANT) We only start our own delayReq timer after receiving the first sync
			if (ptpClock->WaitingForSync) { //The First SYNC
				ptpClock->WaitingForSync = FALSE;
				printf("\r\n ===========Received first Sync from Master in Slave State.==========\r\n");
				if (ptpClock->portDS.delayMechanism == E2E)
					ptpd_timerStart(DELAYREQ_INTERVAL_TIMER, pow2ms(ptpClock->portDS.logMinDelayReqInterval), ptpClock->itimer);
				else if (ptpClock->portDS.delayMechanism == P2P)
					ptpd_timerStart(PDELAYREQ_INTERVAL_TIMER, pow2ms(ptpClock->portDS.logMinPDelayReqInterval), ptpClock->itimer);
			}
			//===========================================================================================
			ptpClock->rcvdSyncSequenceId = ptpClock->msgTmpHeader.sequenceId;
			//printf("rx SYNC seq=%x\r\n", ptpClock->rcvdSyncSequenceId);

			//set logSyncInterval from the Master.
			ptpClock->portDS.logSyncInterval = ptpClock->msgTmpHeader.logMessageInterval;

			// this will be 0x7F for unicast so if we have a grant, use the granted value
			//if(ptpClock->rtOpts->unicastNegotiation && ptpClock->ptpClock->parentGrants
			//		&& ptpClock->ptpClock->parentGrants->grantData[SYNC_INDEXED].granted)
			//{
			//		ptpClock->portDS.logSyncInterval =  ptpClock->parentGrants->grantData[SYNC_INDEXED].logInterval;
			//}


			//save its rcv time.
			ptpClock->timestamp_syncRecieve.seconds = time->seconds;
			ptpClock->timestamp_syncRecieve.nanoseconds = time->nanoseconds;

			integer64_to_internalTime(&ptpClock->msgTmpHeader.correctionField, &correctionField);//was scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionField, &correctionField);

			if (getFlag(ptpClock->msgTmpHeader.flagField0, FLAG0_TWO_STEP)){
				ptpClock->waitingForFollowUp = TRUE;
				ptpClock->defaultDS.twoStepFlag = TRUE;
				//getTimeFromReg(&tmpTime);printf("(d %d.%09d) ", tmpTime.seconds, tmpTime.nanoseconds); //for debug

				// Save correctionField of Sync message for future use
				//ptpClock->correctionField_sync = correctionField; //replaced with the below YOON 2016.10.1
				integer64_to_internalTime(&ptpClock->msgTmpHeader.correctionField,&correctionField);
				ptpClock->lastSyncCorrectionField.seconds =	correctionField.seconds;
				ptpClock->lastSyncCorrectionField.nanoseconds =	correctionField.nanoseconds;
				//ptpClock->rcvdSyncSequenceId =	ptpClock->msgTmpHeader.sequenceId;
			}
			else//Single Step
			{
				msgUnpackSync(ptpClock->msgIbuf, &ptpClock->msgTmp.sync);
				ptpClock->waitingForFollowUp = FALSE;
				/* Synchronize  local clock with the received Sync Frame's timestamp*/
				toInternalTime(&originTimestamp, &ptpClock->msgTmp.sync.originTimestamp);
				/* use correctionField of Sync message for future use */
				updateOffset(ptpClock, &ptpClock->timestamp_syncRecieve, &originTimestamp, &correctionField);
#if (ETHIF_ROLE == PTP_GRANDMASTERCLOCK)
				updateClockForGMC(ptpClock);
#else
				updateClock(ptpClock);
#endif
				//issueDelayReqTimerExpired(ptpClock);
				printf("SINGLESTEP\r\n");
			}

			break;

		case PTP_MASTER:

			if (!isFromSelf){
				//printf("handleSync: Rx SYNC from another master in MASTER state\r\n");
				break;
			}else{
				//printf("handleSync: Rx SYNC is ignored because it is from self in MASTER state\r\n");
				break;
			}

//      if waitingForLoopback && TWO_STEP_FLAG
//        {
//            /* Add  latency */
//            addTime(time, time, &rtOpts->outboundLatency);
//
//            issueFollowup(ptpClock, time);
//            break;
//        }
		default:

			//DBGV("handleSync: disregard\r\n");
			break;
	}
}

//for TWO-STEP
static void handleFollowUp(PtpClock *ptpClock, bool isFromSelf)
{
	TimeInternal preciseOriginTimestamp;
	TimeInternal correctionField = {0,0};
	bool  isFromCurrentParent = FALSE;
	TimeInternal tmpTime;//for debug

	DBGV("handleFollowup: received in state %s\r\n", stateString(ptpClock->portDS.portState));

	if (ptpClock->msgIbufLength < FOLLOW_UP_LENGTH)	{
		ERROR("handleFollowup: short message\r\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
		return;
	}

	if (isFromSelf)	{
		DBGV("handleFollowup: ignore from self\r\n");
		return;
	}

	switch (ptpClock->portDS.portState)	{
		case PTP_INITIALIZING:
		case PTP_FAULTY:
		case PTP_DISABLED:
		case PTP_LISTENING:

			DBGV("handleFollowup: disregard\r\n");
			break;

		case PTP_UNCALIBRATED:
		case PTP_SLAVE:

			isFromCurrentParent = isSamePortIdentity(
					&ptpClock->parentDS.parentPortIdentity,
					&ptpClock->msgTmpHeader.sourcePortIdentity);

			if (!ptpClock->waitingForFollowUp)	{
				DBGV("handleFollowup: not waiting a message\r\n");
				break;
			}

			if (!isFromCurrentParent){
				DBGV("handleFollowup: not from current parent\r\n");
				break;
			}
			//getTimeFromReg(&tmpTime);printf("(d %d.%09d) ", tmpTime.seconds, tmpTime.nanoseconds); //for debug

			//Check its paired Sync.
			//printf("Followup's Seq (%x), prevSyncSeqId(%x)\r\n",  ptpClock->msgTmpHeader.sequenceId,ptpClock->rcvdSyncSequenceId);
			if (ptpClock->rcvdSyncSequenceId !=  ptpClock->msgTmpHeader.sequenceId)	{
				//DBGV("Followup: SequenceID (%x) != Sync message(%x)\r\n",  ptpClock->msgTmpHeader.sequenceId,ptpClock->rcvdSyncSequenceId);
				printf("Followup: SequenceID (%x) != Sync message(%x)\r\n",  ptpClock->msgTmpHeader.sequenceId,ptpClock->rcvdSyncSequenceId);
				break;
			}
			//For valid Followup Msg.
			//get precise T1 time.
			msgUnpackFollowUp(ptpClock->msgIbuf, &ptpClock->msgTmp.follow);
			ptpClock->waitingForFollowUp = FALSE;

			// synchronize local clock
			toInternalTime(&preciseOriginTimestamp, &ptpClock->msgTmp.follow.preciseOriginTimestamp);
			//integer64_to_internalTime(&ptpClock->msgTmpHeader.correctionField, &correctionField);//was scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionField, &correctionField);//integer64_to_internalTime
			//addTime(&correctionField,&correctionField, &ptpClock->lastSyncCorrectionField); //was correctionField_sync

			//send_time(T1) = preciseOriginTimestamp (received inside followup)
			//recv_time(T2) = sync_receive_time (received as CMSG in handleEvent)
			updateOffset(ptpClock, &ptpClock->timestamp_syncRecieve, &preciseOriginTimestamp, &correctionField);

			checkOffset(ptpClock);

			if (ptpClock->clockControl.updateOK) {
#if (ETHIF_ROLE == PTP_GRANDMASTERCLOCK)
				updateClockForGMC(ptpClock); //Actually not used for GMC
#else
				updateClock(ptpClock);
#endif
			}
			break;

		case PTP_MASTER:

			DBGV("handleFollowup: from another master\r\n");
			break;

		case PTP_PASSIVE:

			DBGV("handleFollowup: disregard\r\n");
			//issueDelayReqTimerExpired(ptpClock);
			break;

		default:

			DBG("handleFollowup: unrecognized state\r\n");
			break;
	}
}


static void handleDelayReq(PtpClock *ptpClock, TimeInternal *time, bool isFromSelf)
{
	if (isFromSelf){
			DBGV("handleDelayReq: ignore from self\r\n");
			return;
	}

	switch (ptpClock->portDS.delayMechanism)
	{
		case E2E:

			DBGV("handleDelayReq: received in mode E2E in state %s\r\n", stateString(ptpClock->portDS.portState));
			if (ptpClock->msgIbufLength < DELAY_REQ_LENGTH)
			{
				ERROR("handleDelayReq: short message\r\n");
				stm_ptpd_toState(ptpClock, PTP_FAULTY);
				return;
			}

			switch (ptpClock->portDS.portState)
			{
				case PTP_INITIALIZING:
				case PTP_FAULTY:
				case PTP_DISABLED:
				case PTP_UNCALIBRATED:
				case PTP_LISTENING:
					DBGV("handleDelayReq: disregard\\rn");
					return;

				case PTP_SLAVE:
					DBGV("handleDelayReq: disregard\r\n");
					break;

				case PTP_MASTER:
					/* TODO: manage the value of ptpClock->logMinDelayReqInterval form logSyncInterval to logSyncInterval + 5 */
					issueDelayResp(ptpClock, time, &ptpClock->msgTmpHeader);
					break;

				default:
					DBG("handleDelayReq: unrecognized state\r\n");
					break;
			}

			break;

		case P2P: //802.1as Only support P2P. Thus this delayReq msg will be disregarded.

			ERROR("handleDelayReq: disregard in P2P mode\r\n");
			break;

		default:

			/* none */
			break;
	}
}



static void handleDelayResp(PtpClock *ptpClock, bool  isFromSelf)
{
	bool  isFromCurrentParent = FALSE;
	bool  isCurrentRequest = FALSE;
	TimeInternal requestReceiptTimestamp;
	TimeInternal correctionField;


	switch (ptpClock->portDS.delayMechanism)
	{
		case E2E:

			DBGV("handleDelayResp: received in mode E2E in state %s\r\n", stateString(ptpClock->portDS.portState));
			if (ptpClock->msgIbufLength < DELAY_RESP_LENGTH)
			{
				ERROR("handleDelayResp: short message\r\n");
				stm_ptpd_toState(ptpClock, PTP_FAULTY);
				return;
			}

			switch (ptpClock->portDS.portState)
			{
				case PTP_INITIALIZING:
				case PTP_FAULTY:
				case PTP_DISABLED:
				case PTP_LISTENING:
					printf("handleDelayResp(Status=%d): disregard\r\n", ptpClock->portDS.portState); //DBGV("handleDelayResp: disregard\r\n");
					return;

				case PTP_UNCALIBRATED:
				case PTP_SLAVE:

					msgUnpackDelayResp(ptpClock->msgIbuf, &ptpClock->msgTmp.resp);

					isFromCurrentParent = isSamePortIdentity(
							&ptpClock->parentDS.parentPortIdentity,
							&ptpClock->msgTmpHeader.sourcePortIdentity);

					isCurrentRequest = isSamePortIdentity(
							&ptpClock->portDS.portIdentity,
							&ptpClock->msgTmp.resp.requestingPortIdentity);

					if(isCurrentRequest && isFromCurrentParent){

						ptpd_timerStart(DELAY_RECEIPT_TIMER,
								max((ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)),MISSED_MESSAGES_MAX * (pow2ms(ptpClock->portDS.logMinDelayReqInterval))),
								ptpClock->itimer);
					}else{
						break;// disregard
					}

					//if (!ptpClock->waitingForDelayResp) {
					//	DBG("Ignored DelayResp sequence %d - wasn't waiting for one\n",	ptpClock->msgTmpHeader.sequenceId);
					//	break;
					//}

					//check seq.
					if ((ptpClock->sentDelayReqSequenceId  != (ptpClock->msgTmpHeader.sequenceId+1))){
							DBGV("handleDelayResp: doesn't match with the delayReq\r\n");
							break;
					}

					//handle this valid msg.
					// TODO: revisit 11.3
					ptpClock->waitingForDelayResp = FALSE;

					toInternalTime(&ptpClock->timestamp_delayReqRecieve, &ptpClock->msgTmp.resp.receiveTimestamp);
					integer64_to_internalTime(&ptpClock->msgTmpHeader.correctionField, &correctionField);//was scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield, &correctionField);

					//send_time = delay_req_send_time (received as CMSG in handleEvent)
					//recv_time = requestReceiptTimestamp (received inside delayResp)

					updateDelay(ptpClock, &ptpClock->timestamp_delayReqSend, &ptpClock->timestamp_delayReqRecieve, &correctionField);

					if (ptpClock->waitingForDelayResp) DBG("Received first Delay Response from Master\n");

					//Check for different DelayRequestInterval.
					if (ptpClock->portDS.logMinDelayReqInterval != ptpClock->msgTmpHeader.logMessageInterval) {
						//Update DelayReqInterval as noted by the Master.
						ptpClock->portDS.logMinDelayReqInterval = ptpClock->msgTmpHeader.logMessageInterval;

						// arm the timer again now that we have the correct delayreq interval
						ptpd_timerStart(DELAY_RECEIPT_TIMER,
								max(
										(ptpClock->portDS.announceReceiptTimeout) * (pow2ms(ptpClock->portDS.logAnnounceInterval)),
										MISSED_MESSAGES_MAX * (pow2ms(ptpClock->portDS.logMinDelayReqInterval))
								),
								ptpClock->itimer);

					}
			}
			break;

		case P2P:

			printf("handleDelayResp: disregard in P2P mode\r\n");
			break;

		default:

			break;
	}
}


static void handlePDelayReq(PtpClock *ptpClock, TimeInternal *time, bool  isFromSelf)
{
	switch (ptpClock->portDS.delayMechanism)
	{
		case E2E:
			ERROR("handlePDelayReq: disregards in E2E mode\r\n");
			break;

		case P2P: //802.1as only support this P2P

			DBGV("handlePDelayReq: received in mode P2P in state %s\r\n", stateString(ptpClock->portDS.portState));
			if (ptpClock->msgIbufLength < PDELAY_REQ_LENGTH)
			{
					ERROR("handlePDelayReq: short message\r\n");
					stm_ptpd_toState(ptpClock, PTP_FAULTY);
					return;
			}

			switch (ptpClock->portDS.portState)
			{
				case PTP_INITIALIZING:
				case PTP_FAULTY:
				case PTP_DISABLED:
				case PTP_UNCALIBRATED:
				case PTP_LISTENING:
					printf("handlePDelayReq(Status=%d): disregard\r\n", ptpClock->portDS.portState); //DBGV("handlePDelayReq: disregard\r\n");
					return;

				case PTP_PASSIVE:
				case PTP_SLAVE:
				case PTP_MASTER:

					if (isFromSelf) //SoftTimeStamping
					{
							DBGV("handlePDelayReq: ignore from self\r\n");
							break;
					}

					//ptpClock->PdelayReqHeader = ptpClock->msgTmpHeader;
					msgUnpackHeader(ptpClock->msgIbuf, &ptpClock->msgTmpHeader);//add 2016.6.24
					issuePDelayResp(ptpClock, time, &ptpClock->msgTmpHeader);

					//YOON- WHY seconds>0?
					//if ((time->seconds != 0) && getFlag(ptpClock->msgTmpHeader.flagField[0], FLAG0_TWO_STEP)) //TWO-STEP
					if (getFlag(ptpClock->msgTmpHeader.flagField0, FLAG0_TWO_STEP)) //TWO-STEP
					{
							issuePDelayRespFollowUp(ptpClock, time, &ptpClock->msgTmpHeader);
					}

					break;


				default:

					DBG("handlePDelayReq: unrecognized state\r\n");
					break;
			}
			break;

		default:

			break;
	}
}

static void handlePDelayResp(PtpClock *ptpClock, TimeInternal *time, bool isFromSelf)
{
	TimeInternal requestReceiptTimestamp;
	TimeInternal correctionField;
	bool  isCurrentRequest;

	switch (ptpClock->portDS.delayMechanism)
	{
		case E2E:
			printf("Err: handlePDelayResp: disregard in E2E mode\n");
			break;

		case P2P:

			DBGV("handlePDelayResp: received in mode P2P in state %s\r\n", stateString(ptpClock->portDS.portState));
			if (ptpClock->msgIbufLength < PDELAY_RESP_LENGTH)
			{
					ERROR("handlePDelayResp: short message\r\n");
					//stm_ptpd_toState(ptpClock, PTP_FAULTY);
					return;
			}

			switch (ptpClock->portDS.portState)
			{
				case PTP_INITIALIZING:
				case PTP_FAULTY:
				case PTP_DISABLED:
				case PTP_UNCALIBRATED:
				case PTP_LISTENING:

				DBGV("handlePDelayResp: disregard\r\n");
				return;

				case PTP_MASTER:
				case PTP_SLAVE:

					msgUnpackPDelayResp(ptpClock->msgIbuf, &ptpClock->msgTmp.presp);

					isCurrentRequest = isSamePortIdentity(
							&ptpClock->portDS.portIdentity,
							&ptpClock->msgTmp.presp.requestingPortIdentity
							);

					if ((ptpClock->sentPDelayReqSequenceId  == (ptpClock->msgTmpHeader.sequenceId +1)) && isCurrentRequest)	{
						if (getFlag(ptpClock->msgTmpHeader.flagField0, FLAG0_TWO_STEP)){//Two Step Clock
							ptpClock->waitingForPDelayRespFollowUp = TRUE;

							/* Store  t4 (Fig 35)*/
							ptpClock->pdelay_t4 = *time;//the time is the precise rx time of this frame.

							/* store  t2 (Fig 35)*/
							toInternalTime(&requestReceiptTimestamp, &ptpClock->msgTmp.presp.requestReceiptTimestamp);//The receive time of pDelayReq frame at the peer.
							ptpClock->pdelay_t2 = requestReceiptTimestamp;
							integer64_to_internalTime(&ptpClock->msgTmpHeader.correctionField, &correctionField);//was scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield, &correctionField);

							ptpClock->lastPdelayRespCorrectionField.seconds = correctionField.seconds;
							ptpClock->lastPdelayRespCorrectionField.nanoseconds = correctionField.nanoseconds;
							//was ptpClock->correctionField_pDelayResp = correctionField;
						}else{ //One step Clock
							ptpClock->waitingForPDelayRespFollowUp = FALSE;
							/* Store  t4 (Fig 35)*/
							ptpClock->pdelay_t4 = *time;

							integer64_to_internalTime(&ptpClock->msgTmpHeader.correctionField, &correctionField);//was scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield, &correctionField);
							updatePeerDelay(ptpClock, &correctionField, FALSE);
						}
					}
					else
					{
							DBGV("handlePDelayResp: PDelayResp doesn't match with the PDelayReq.\r\n");
					}
					ptpClock->recvPdelayRespSequenceId = ptpClock->msgTmpHeader.sequenceId;
					break;

				default:

						DBG("handlePDelayResp: unrecognized state\r\n");
						break;
			}
			break;

		default:

			break;
	}
}

static void handlePDelayRespFollowUp(PtpClock *ptpClock, bool isFromSelf)
{
	TimeInternal responseOriginTimestamp;
	TimeInternal correctionField;

	switch (ptpClock->portDS.delayMechanism)
	{
		case E2E:

			ERROR("handlePDelayRespFollowUp: disregard in E2E mode\r\n");
			break;

		case P2P:
			DBGV("handlePDelayRespFollowUp: received in mode P2P in state %s\r\n", stateString(ptpClock->portDS.portState));
			if (ptpClock->msgIbufLength < PDELAY_RESP_FOLLOW_UP_LENGTH)	{
				ERROR("handlePDelayRespFollowUp: short message\r\n");
				stm_ptpd_toState(ptpClock, PTP_FAULTY);
				return;
			}

			switch (ptpClock->portDS.portState)	{
				case PTP_INITIALIZING:
				case PTP_FAULTY:
				case PTP_DISABLED:
				case PTP_UNCALIBRATED:
					DBGV("handlePDelayRespFollowUp: disregard\r\n");
					return;

				case PTP_SLAVE:
				case PTP_MASTER:
					if (!ptpClock->waitingForPDelayRespFollowUp){
						DBG("handlePDelayRespFollowUp: not waiting a message\r\n");
						break;
					}

					if (((ptpClock->msgTmpHeader.sequenceId +1)== ptpClock->sentPDelayReqSequenceId) &&	(ptpClock->msgTmpHeader.sequenceId == ptpClock->recvPdelayRespSequenceId)) {

							msgUnpackPDelayRespFollowUp(ptpClock->msgIbuf, &ptpClock->msgTmp.prespfollow);
							toInternalTime(&responseOriginTimestamp, &ptpClock->msgTmp.prespfollow.responseOriginTimestamp);//the tx time of pDelayResp frame from the peer.
							ptpClock->pdelay_t3 = responseOriginTimestamp;
							integer64_to_internalTime(&ptpClock->msgTmpHeader.correctionField, &correctionField);//was scaledNanosecondsToInternalTime(&ptpClock->msgTmpHeader.correctionfield, &correctionField);
							addTime(&correctionField, &correctionField, &ptpClock->lastPdelayRespCorrectionField);
							updatePeerDelay(ptpClock, &correctionField, TRUE);
							ptpClock->waitingForPDelayRespFollowUp = FALSE;
							break;
					}

				default:

					DBGV("handlePDelayRespFollowUp: unrecognized state\r\n");
			}
			break;

		default:

			break;
	}
}

static void handleManagement(PtpClock *ptpClock, bool isFromSelf)
{
	/* ENABLE_PORT -> DESIGNATED_ENABLED -> stm_ptpd_toState(PTP_INITIALIZING) */
	/* DISABLE_PORT -> DESIGNATED_DISABLED -> stm_ptpd_toState(PTP_DISABLED) */
}

static void handleSignaling(PtpClock *ptpClock, bool  isFromSelf)
{
}
//Issue the DelayRequestTimer Expiration Event to send DelayRequest Frame.
static void issueDelayReqOnTimerExpired(PtpClock *ptpClock)
{
	switch (ptpClock->portDS.delayMechanism)
	{
		case E2E:

			if (ptpClock->portDS.portState != PTP_SLAVE) //For Master, do nothing.
			{
					break;
			}

			if (ptpd_timerExpired(DELAYREQ_INTERVAL_TIMER,ptpClock->itimer)) //For Slave,
			{
				ptpd_timerStart(DELAYREQ_INTERVAL_TIMER,
						getRand(pow2ms(ptpClock->portDS.logMinDelayReqInterval + 1)),
						ptpClock->itimer);
					DBGV("event DELAYREQ_INTERVAL_TIMEOUT_EXPIRES\r\n");
					issueDelayReq(ptpClock);
			}

			break;

		case P2P:

			if (ptpd_timerExpired(PDELAYREQ_INTERVAL_TIMER,ptpClock->itimer))
			{
				ptpd_timerStart(PDELAYREQ_INTERVAL_TIMER, getRand(pow2ms(ptpClock->portDS.logMinPDelayReqInterval + 1)), ptpClock->itimer);
					DBGV("event PDELAYREQ_INTERVAL_TIMEOUT_EXPIRES\r\n");
					issuePDelayReq(ptpClock);
			}
			break;

		default:
				break;
	}
}


/* Pack and send  on general multicast ip adress an Announce message */
static void issueAnnounce(PtpClock *ptpClock)
{
	msgPackAnnounce(ptpClock, ptpClock->msgObuf);
#if (PTP_PROTOCOL == IEEE1588V2)
	if (!netSendGeneral(&ptpClock->netPath, ptpClock->msgObuf, ANNOUNCE_LENGTH))
	{
		ERROR("issueAnnounce: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}
	else
	{
		DBGV("issueAnnounce\r\n");
		ptpClock->sentAnnounceSequenceId++;
	}
#elif (PTP_PROTOCOL == IEEE8021AS)
	if (!netSendGeneral8021as(&ptpClock->netPath, ptpClock->msgObuf, ANNOUNCE_LENGTH))
	{
		ERROR("issueAnnounce: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}
	else
	{
		DBGV("issueAnnounce\r\n");
		ptpClock->sentAnnounceSequenceId++;
	}
#endif
}

/* Pack and send  on event multicast ip adress a Sync message */
static void issueSync(PtpClock *ptpClock)
{
	Timestamp originTimestamp;
	TimeInternal internalTime;

	/* try to predict outgoing time stamp */
	getTimeFromReg(&internalTime);
	fromInternalTime(&internalTime, &originTimestamp);

	msgPackSync(ptpClock, ptpClock->msgObuf, &originTimestamp);

#if (PTP_PROTOCOL ==IEEE1588V2)
	if (!netSendEvent(&ptpClock->netPath, ptpClock->msgObuf, SYNC_LENGTH, &internalTime))
	{
		ERROR("issueSync: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}
	else
	{
		DBGV("issueSync\n");
		ptpClock->sentSyncSequenceId++;

		/* sync TX timestamp is valid */
		if ((internalTime.seconds != 0) && (ptpClock->defaultDS.twoStepFlag))
		{
			// waitingForLoopback = false;
			addTime(&internalTime, &internalTime, &ptpClock->outboundLatency);
			issueFollowup(ptpClock, &internalTime);
		}
		else
		{
			// waitingForLoopback = ptpClock->twoStepFlag;
		}
	}
#elif (PTP_PROTOCOL ==IEEE8021AS)

	if (!netSendEvent8021as(&ptpClock->netPath, ptpClock->msgObuf, SYNC_LENGTH, &internalTime))
	{
		ERROR("issueSync: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}
	else
	{
		DBGV("issueSync\n");
		ptpClock->sentSyncSequenceId++;

		/* sync TX timestamp is valid */
		if ((internalTime.seconds != 0) && (ptpClock->defaultDS.twoStepFlag))
		{
			// waitingForLoopback = false;
			addTime(&internalTime, &internalTime, &ptpClock->outboundLatency);
			issueFollowup(ptpClock, &internalTime);
		}
		else
		{
			// waitingForLoopback = ptpClock->twoStepFlag;
		}
	}
#endif
}

/* Pack and send on general multicast ip adress a FollowUp message */
static void issueFollowup(PtpClock *ptpClock, const TimeInternal *time)
{
	Timestamp preciseOriginTimestamp;

	fromInternalTime(time, &preciseOriginTimestamp);
	msgPackFollowUp(ptpClock, ptpClock->msgObuf, &preciseOriginTimestamp);
#if (PTP_PROTOCOL == IEEE1588V2)
	if (!netSendGeneral(&ptpClock->netPath, ptpClock->msgObuf, FOLLOW_UP_LENGTH))
	{
		ERROR("issueFollowup: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}
	else
	{
		DBGV("issueFollowup\r\n");
	}
#elif (PTP_PROTOCOL ==IEEE8021AS)
	if (!netSendGeneral8021as(&ptpClock->netPath, ptpClock->msgObuf, FOLLOW_UP_LENGTH))
	{
		ERROR("issueFollowup: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}
	else
	{
		DBGV("issueFollowup\r\n");
	}
#endif
}


/* Pack and send on event multicast ip address a DelayReq message */
static void issueDelayReq(PtpClock *ptpClock)
{
	Timestamp originTimestamp;
	TimeInternal internalTime;

	if(ptpClock->leapSecondInProgress) {
	    DBG("Leap second in progress - will not send DELAY_REQ\n");
	    return;
	}

	getTimeFromReg(&internalTime);
	if (respectUtcOffset(ptpClock) == TRUE) {
		internalTime.seconds += ptpClock->timePropertiesDS.currentUtcOffset;
	}
	fromInternalTime(&internalTime, &originTimestamp);

	msgPackDelayReq(ptpClock, ptpClock->msgObuf, &originTimestamp);
#if (PTP_PROTOCOL ==IEEE1588V2)
	if (!netSendEvent(&ptpClock->netPath, ptpClock->msgObuf, DELAY_REQ_LENGTH, &internalTime))
	{
		ERROR("issueDelayReq: can't sent\n");
		printf("issueDelayReq: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}
	else
	{
		DBGV("issueDelayReq\r\n");
		//printf(">issueDelayReq\r\n");
		ptpClock->sentDelayReqSequenceId++;

		/* Delay req TX timestamp is valid */
		if (internalTime.seconds != 0)
		{
			addTime(&internalTime, &internalTime, &ptpClock->outboundLatency);
			ptpClock->timestamp_delayReqSend = internalTime;
		}
	}
#elif (PTP_PROTOCOL ==IEEE8021AS)
	if (!netSendEvent8021as(&ptpClock->netPath, ptpClock->msgObuf, DELAY_REQ_LENGTH, &internalTime))
	{
		ERROR("issueDelayReq: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}
	else
	{
		DBGV("issueDelayReq\r\n");
		ptpClock->sentDelayReqSequenceId++;

		/* Delay req TX timestamp is valid */
		if (internalTime.seconds != 0)
		{
			addTime(&internalTime, &internalTime, &ptpClock->outboundLatency);
			ptpClock->timestamp_delayReqSend = internalTime;
		}
	}
#endif
	// From now on, we will only accept delayreq and delayresp of (sentDelayReqSequenceId - 1)
	// Explicitly re-arm timer for sending the next delayReq
	// 9.5.11.2: arm the timer with a uniform range from 0 to 2 x interval
	 // this is only ever used here, so removed the timerStart_random function
	//Restart delayReq timer
	ptpd_timerStart(DELAYREQ_INTERVAL_TIMER,
			getRand(pow2ms(ptpClock->portDS.logMinDelayReqInterval + 1)),
			ptpClock->itimer);
	//timerStart(&ptpClock->timers[DELAYREQ_INTERVAL_TIMER],  pow(2,ptpClock->portDS.logMinDelayReqInterval) * getRand() * 2.0);
}

/* Pack and send on event multicast ip adress a PDelayReq message */
static void issuePDelayReq(PtpClock *ptpClock)
{
	Timestamp originTimestamp;
	TimeInternal internalTime;

	getTimeFromReg(&internalTime);
	fromInternalTime(&internalTime, &originTimestamp);

	msgPackPDelayReq(ptpClock, ptpClock->msgObuf, &originTimestamp);

#if (PTP_PROTOCOL ==IEEE1588V2)
	if (!netSendPeerEvent(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_REQ_LENGTH, &internalTime))	{
		ERROR("issuePDelayReq: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}else{
		DBGV("issuePDelayReq\r\n");
		ptpClock->sentPDelayReqSequenceId++;

		/* Delay req TX timestamp is valid */
		if (internalTime.seconds != 0){ //?? For considering the very first time?
			addTime(&internalTime, &internalTime, &ptpClock->outboundLatency);
			ptpClock->pdelay_t1 = internalTime;
		}
	}
#elif (PTP_PROTOCOL ==IEEE8021AS)
	if (!netSendPeerEvent8021as(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_REQ_LENGTH, &internalTime)){
			ERROR("issuePDelayReq: can't sent\n");
			stm_ptpd_toState(ptpClock, PTP_FAULTY);
		}else{
			DBGV("issuePDelayReq\r\n");
			ptpClock->sentPDelayReqSequenceId++;

			/* Delay req TX timestamp is valid */
			if (internalTime.seconds != 0){
				addTime(&internalTime, &internalTime, &ptpClock->outboundLatency);
				ptpClock->pdelay_t1 = internalTime;
			}
		}
#endif
}

/* Pack and send on event multicast ip adress a PDelayResp message */
static void issuePDelayResp(PtpClock *ptpClock, TimeInternal *time, const MsgHeader * pDelayReqHeader)
{
	Timestamp requestReceiptTimestamp;

	fromInternalTime(time, &requestReceiptTimestamp);
	msgPackPDelayResp(ptpClock, ptpClock->msgObuf, pDelayReqHeader, &requestReceiptTimestamp);

#if (PTP_PROTOCOL ==IEEE1588V2)
	if (!netSendPeerEvent(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_RESP_LENGTH, time))	{
		ERROR("issuePDelayResp: can't sent\r\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}else{
		if (time->seconds != 0)	{
			/* Add  outboundLatency */
			addTime(time, time, &ptpClock->outboundLatency);
		}

		DBGV("issuePDelayResp\r\n");
	}
#elif (PTP_PROTOCOL ==IEEE8021AS)
	if (!netSendPeerEvent8021as(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_RESP_LENGTH, time)){
		ERROR("issuePDelayResp: can't sent\r\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}else{
		if (time->seconds != 0)	{
			/* Add  latency */
			addTime(time, time, &ptpClock->outboundLatency);
		}

		DBGV("issuePDelayResp\r\n");
	}
#endif
}


/* Pack and send on event multicast ip adress a DelayResp message */
static void issueDelayResp(PtpClock *ptpClock, const TimeInternal *time, const MsgHeader * delayReqHeader)
{
	Timestamp requestReceiptTimestamp;

	fromInternalTime(time, &requestReceiptTimestamp);
	msgPackDelayResp(ptpClock, ptpClock->msgObuf, delayReqHeader, &requestReceiptTimestamp);

#if (PTP_PROTOCOL ==IEEE1588V2)
	if (!netSendGeneral(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_RESP_LENGTH)){
		ERROR("issueDelayResp: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}else{
		DBGV("issueDelayResp\r\n");
	}
#elif (PTP_PROTOCOL ==IEEE8021AS)
	if (!netSendGeneral8021as(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_RESP_LENGTH))
	{
		ERROR("issueDelayResp: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}
	else
	{
		DBGV("issueDelayResp\r\n");
	}
#endif
}

static void issuePDelayRespFollowUp(PtpClock *ptpClock, const TimeInternal *time, const MsgHeader * pDelayReqHeader)
{
	Timestamp responseOriginTimestamp;
	fromInternalTime(time, &responseOriginTimestamp);

	msgPackPDelayRespFollowUp(ptpClock,ptpClock->msgObuf, pDelayReqHeader, &responseOriginTimestamp);

#if (PTP_PROTOCOL ==IEEE1588V2)
	if (!netSendPeerGeneral(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_RESP_FOLLOW_UP_LENGTH)){
		ERROR("issuePDelayRespFollowUp: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}else{
		DBGV("issuePDelayRespFollowUp\r\n");
	}
#elif (PTP_PROTOCOL ==IEEE8021AS)
	if (!netSendPeerGeneral8021as(&ptpClock->netPath, ptpClock->msgObuf, PDELAY_RESP_FOLLOW_UP_LENGTH))
	{
		ERROR("issuePDelayRespFollowUp: can't sent\n");
		stm_ptpd_toState(ptpClock, PTP_FAULTY);
	}
	else
	{
		DBGV("issuePDelayRespFollowUp\r\n");
	}
#endif

}

//=============
//call by handleAnnounce()
void addForeign(PtpClock *ptpClock,unsigned char *buf,MsgHeader *header, unsigned char localPreference, uint32_t sourceAddr)
{
	int i,j;
	bool found = FALSE;

	//DBGV("addForeign localPref: %d\n", localPreference);

	j = ptpClock->foreignMasterDS.foreign_record_best;

	/*Check if Foreign master is already known*/
	for (i=0;i<ptpClock->foreignMasterDS.number_foreign_records;i++) {
		if (!memcmp(header->sourcePortIdentity.clockIdentity,
			    ptpClock->foreignMasterDS.foreign[j].foreignMasterPortIdentity.clockIdentity,
			    CLOCK_IDENTITY_LENGTH) &&
		    (header->sourcePortIdentity.portNumber == ptpClock->foreignMasterDS.foreign[j].foreignMasterPortIdentity.portNumber))
		{
			/*Foreign Master is already in Foreignmaster data set*/
			ptpClock->foreignMasterDS.foreign[j].foreignMasterAnnounceMessages++;
			found = TRUE;
			DBGV("addForeign : AnnounceMessage incremented \n");
			msgUnpackHeader(buf,&ptpClock->foreignMasterDS.foreign[j].header);
			msgUnpackAnnounce(buf,&ptpClock->foreignMasterDS.foreign[j].announce);
			ptpClock->foreignMasterDS.foreign[j].disqualified = FALSE;
			ptpClock->foreignMasterDS.foreign[j].localPreference = localPreference;
			break;
		}

		j = (j+1)%ptpClock->foreignMasterDS.number_foreign_records;
	}

	/*New Foreign Master*/
	if (!found) {
		if (ptpClock->foreignMasterDS.number_foreign_records <
		    ptpClock->foreignMasterDS.max_foreign_records) {
			ptpClock->foreignMasterDS.number_foreign_records++;
		}

		/* Preserve best master record from overwriting (sf FR #22) - use next slot */
		if (ptpClock->foreignMasterDS.foreign_record_i == ptpClock->foreignMasterDS.foreign_record_best) {
			ptpClock->foreignMasterDS.foreign_record_i++;
			ptpClock->foreignMasterDS.foreign_record_i %= ptpClock->foreignMasterDS.number_foreign_records;
		}

		j = ptpClock->foreignMasterDS.foreign_record_i;

		/*Copy new foreign master data set from Announce message*/
		copyClockIdentity(ptpClock->foreignMasterDS.foreign[j].foreignMasterPortIdentity.clockIdentity,
		       header->sourcePortIdentity.clockIdentity);
		ptpClock->foreignMasterDS.foreign[j].foreignMasterPortIdentity.portNumber =
			header->sourcePortIdentity.portNumber;
		ptpClock->foreignMasterDS.foreign[j].foreignMasterAnnounceMessages = 0;
		ptpClock->foreignMasterDS.foreign[j].localPreference = localPreference;
		ptpClock->foreignMasterDS.foreign[j].sourceAddr = sourceAddr;
		ptpClock->foreignMasterDS.foreign[j].disqualified = FALSE;
		/*
		 * header and announce field of each Foreign Master are
		 * usefull to run Best Master Clock Algorithm
		 */
		msgUnpackHeader(buf,&ptpClock->foreignMasterDS.foreign[j].header);
		msgUnpackAnnounce(buf,&ptpClock->foreignMasterDS.foreign[j].announce);
		DBGV("New foreign Master added \n");

		ptpClock->foreignMasterDS.foreign_record_i =
			(ptpClock->foreignMasterDS.foreign_record_i+1) %
			ptpClock->foreignMasterDS.max_foreign_records;
	}
}

bool respectUtcOffset(PtpClock * ptpClock) {
	if (ptpClock->timePropertiesDS.currentUtcOffsetValid || ptpClock->rtOpts->alwaysRespectUtcOffset) {
		return TRUE;
	}
	return FALSE;
}

#endif //USE_LWIP_PTP
