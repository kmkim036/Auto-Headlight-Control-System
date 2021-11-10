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

#if (PROJ_FOR == PROJ_FOR_PTP)
#include "ptpstm/include/ptpd.h" //#include "../ptpd.h"

#define CLAMP(var,bound) {\
    if(var < -bound) {\
	var = -bound;\
    }\
    if(var > bound) {\
	var = bound;\
    }\
}

void initClock(PtpClock *ptpClock)
{
	printf("initClock\r\n");
	ptpClock->disabled = ptpClock->rtOpts->portDisabled;

	// ClearTime
	ptpClock->currentDS.offsetFromMaster.seconds = ptpClock->currentDS.offsetFromMaster.nanoseconds = 0;
	ptpClock->currentDS.meanPathDelay.seconds = ptpClock->currentDS.meanPathDelay.nanoseconds = 0;
	ptpClock->Tms_offset.seconds = ptpClock->Tms_offset.nanoseconds = 0;
	ptpClock->delaySM.seconds = ptpClock->delaySM.nanoseconds = 0;

	// filter for One way delay
	ptpClock->owd_filt.n = 0;
	ptpClock->owd_filt.s = ptpClock->servo.sDelay;

	// filter for Offset from master
	ptpClock->ofm_filt.n = 0;
	ptpClock->ofm_filt.s = ptpClock->servo.sOffset;

	ptpClock->offsetFirstUpdated   = FALSE;

/*	ptpClock->waitingForFollowUp = FALSE;
	ptpClock->waitingForPDelayRespFollowUp = FALSE;
	ptpClock->waitingForDelayResp = FALSE; //YOON 2016.09.30

	ptpClock->pdelay_t1.seconds = ptpClock->pdelay_t1.nanoseconds = 0;
	ptpClock->pdelay_t2.seconds = ptpClock->pdelay_t2.nanoseconds = 0;
	ptpClock->pdelay_t3.seconds = ptpClock->pdelay_t3.nanoseconds = 0;
	ptpClock->pdelay_t4.seconds = ptpClock->pdelay_t4.nanoseconds = 0;

	// Reset parent statistics
	ptpClock->parentDS.parentStats = FALSE;
	ptpClock->parentDS.observedParentClockPhaseChangeRate = 0;
	ptpClock->parentDS.observedParentOffsetScaledLogVariance = 0;
	// Scaled log variance
	if (DEFAULT_PARENTS_STATS){
		ptpClock->slv_filt.n = 0;
		ptpClock->slv_filt.s = 6;
		ptpClock->offsetHistory[0] = 0;
		ptpClock->offsetHistory[1] = 0;
	}
	// Level clock
	//do not reset frequency here - restoreDrift will do it if necessary -- TBD. ì�¼ë‹¨ ì‚´ë ¤ ë‘�ê¸°ë¡œ í•¨.
	if (!ptpClock->servo.noAdjust)	adjFreq(0);
*/
	ptpClock->maxDelayRejected = 0;

	//ptpClock->servo.observedDrift = 0;  /* clears clock servo accumulator (the I term) */

	netEmptyEventQ(&ptpClock->netPath);
}
//Get closest integer value less than or equal to the base 2 log of the input value.
//(ex) order(4) --> 2, order(5) --> 2;  order(7)-->2; order(8)-->3;
static int32_t filter_order(int32_t n)
{
	if (n < 0) {
		n = -n;
	}
	if (n == 0) {
		return 0;
	}
	return floorLog2(n);
}

// Exponential smoothing
static void filter(int32_t * nsec_current, Filter * filt){
	int32_t s, s2;
	//pritnf("filter\r\n");

	/*
			using floatingpoint math
			alpha = 1/2^s
			y[1] = x[0]
			y[n] = alpha * x[n-1] + (1-alpha) * y[n-1]

			or equivalent with integer math
			y[1] = x[0]
			y_sum[1] = y[1] * 2^s
			y_sum[n] = y_sum[n-1] + x[n-1] - y[n-1]
			y[n] = y_sum[n] / 2^s
	*/

	// Increment number of samples
	filt->n++;

	// If it is first time, we are running filter, initialize it
	if (filt->n == 1){
			filt->y_prev = *nsec_current;
			filt->y_sum = *nsec_current;
			filt->s_prev = 0;
	}

	s = filt->s;

	/* Speedup filter, if not 2^s > n */
	if ((1<<s) > filt->n){// Lower the filter order
		s = filter_order(filt->n);
	}else{	// Avoid overflowing of n
		filt->n = 1<<s;
	}

	/* Avoid overflowing of filter. 30 is because using signed 32bit integers */
	s2 = 30 - filter_order(max(filt->y_prev, *nsec_current));

	/* Use the lower filter order, higher will overflow */
	s = min(s, s2);

	/* If the order of the filter changed, change also y_sum value */
	if (filt->s_prev > s) {
		filt->y_sum >>= (filt->s_prev - s);
	} else if (filt->s_prev < s) {
		filt->y_sum <<= (s - filt->s_prev);
	}

	/* Compute the filter itself */
	filt->y_sum += *nsec_current - filt->y_prev;
	filt->y_prev = filt->y_sum >> s;

	/* Save previous order of the filter */
	filt->s_prev = s;

	printf("filter: %d -> %d (order s=%d)\r\n", *nsec_current, filt->y_prev, s);

	/* Actualize target value */
	*nsec_current = filt->y_prev;
}

/* 11.2 */
void updateOffset(PtpClock *ptpClock,
		const TimeInternal *syncEventIngressTimestamp,//recv_time
		const TimeInternal *preciseOriginTimestamp,//send_time
		const TimeInternal *correctionField)
{
	bool maxDelayHit = FALSE;
	bool checkThreshold = ptpClock->rtOpts->maxDelay; //YOON --?  TYPE MISMATCH?. maxDelay = Maximum number of nanoseconds of delay
	TimeInternal master_to_slave_delay;

	//printf("updateOffset:Current UTCOffset=%d\r\n",  ptpClock->timePropertiesDS.currentUtcOffset);

	ptpClock->clockControl.offsetOK = FALSE;
	//-------------
	// updates paused, leap second pending - do nothing
    if(ptpClock->leapSecondInProgress)	return;

	/*  <offsetFromMaster> = <syncEventIngressTimestamp> - <preciseOriginTimestamp>
		 - <meanPathDelay>  -  correctionField  of  Sync  message
		 -  correctionField  of  Follow_Up message. */

	// Compute offsetFromMaster (ofm)
    //master_to_slave_delay(=Tms) =

    //perform basic checks, using only local variables
	subTime(&master_to_slave_delay, syncEventIngressTimestamp, preciseOriginTimestamp);//master_to_slave_delay=recv_time-send_time;

	if(checkThreshold) { // If maxDelay is 0 then we skip.
		if (master_to_slave_delay.seconds && checkThreshold) {
			printf("updateOffset> Aborted. Master to slave delay > 1 sec.\r\n");
			return;
		}
		if (abs(master_to_slave_delay.nanoseconds) > ptpClock->rtOpts->maxDelay) {
			printf("updateOffset aborted, master to slave delay %d greater than administratively set maximum %d\r\n",
						master_to_slave_delay.nanoseconds,ptpClock->rtOpts->maxDelay);
			if(ptpClock->rtOpts->maxDelayMaxRejected) {
				maxDelayHit = TRUE;
				// if we blocked maxDelayMaxRejected samples, reset the slave to unblock the filter
				if(++ptpClock->maxDelayRejected > ptpClock->rtOpts->maxDelayMaxRejected) {
					printf("%d consecutive delay measurements above %d threshold - resetting slave\r\n",
							ptpClock->rtOpts->maxDelayMaxRejected, master_to_slave_delay.nanoseconds);
					stm_ptpd_toState(ptpClock,PTP_LISTENING);
				}
			}else{
				ptpClock->maxDelayRejected=0;
			}
			return;
		}
	}
	//The packet has passed basic checks, so we'll:
	//   - run the filters
	//  - update the global Tms_offset variable
	//  - calculate the new filtered OFM

	// raw value before filtering
	//subTime(&ptpClock->rawDelayMS, syncEventIngressTimestamp, preciseOriginTimestamp);//recv_time, send_time);
	//DBG("UpdateOffset: max delay hit: %d\r\n", maxDelayHit);

	// Used just for End to End mode.
	subTime(&ptpClock->Tms_offset, syncEventIngressTimestamp, preciseOriginTimestamp);

	// Take care of correctionField
	subTime(&ptpClock->Tms_offset, &ptpClock->Tms_offset, correctionField);
	if(ptpClock->Tms_offset.nanoseconds <0)
		printf(">Tms_offset=-%d.%09dnsec.\r\n", ptpClock->Tms_offset.seconds, abs(ptpClock->Tms_offset.nanoseconds));
	else
		printf(">Tms_offset=%d.%09dnsec.\r\n", ptpClock->Tms_offset.seconds, ptpClock->Tms_offset.nanoseconds);
	printf(">meanPathDelay=%d.%09dnsec.\r\n", ptpClock->currentDS.meanPathDelay.seconds, ptpClock->currentDS.meanPathDelay.nanoseconds);

	// update 'offsetFromMaster, ofm'
	switch (ptpClock->portDS.delayMechanism){
		case E2E:
			subTime(&ptpClock->currentDS.offsetFromMaster, &ptpClock->Tms_offset, &ptpClock->currentDS.meanPathDelay);	//subTime(&ptpClock->currentDS.offsetFromMaster, &ptpClock->currentDS.offsetFromMaster, &ptpClock->currentDS.meanPathDelay);
			break;
		case P2P:
			subTime(&ptpClock->currentDS.offsetFromMaster, &ptpClock->Tms_offset, &ptpClock->portDS.peerMeanPathDelay);//subTime(&ptpClock->currentDS.offsetFromMaster, &ptpClock->currentDS.offsetFromMaster, &ptpClock->portDS.peerMeanPathDelay);
			break;
		default:
			break;
	}

	//If the ofm is larger than +/- Seconds --> SYNC FAULT.
	if (ptpClock->currentDS.offsetFromMaster.seconds){ // >=1 sec. cannot filter with secs, clear filter
		ptpClock->offsetFirstUpdated = TRUE;
		ptpClock->clockControl.offsetOK = TRUE;
		//SET_ALARM(ALRM_OFM_SECONDS, TRUE);
		printf("updateOffset: Offset from master is %ds, above 1 second. Can not filter seconds.\r\n",ptpClock->currentDS.offsetFromMaster.seconds);
		return;
	}else {
		//SET_ALARM(ALRM_OFM_SECONDS, FALSE);
		//if(rtOpts->ofmAlarmThreshold) {
		//    if( abs(ptpClock->currentDS.offsetFromMaster.nanoseconds)			> rtOpts->ofmAlarmThreshold) {
		//	SET_ALARM(ALRM_OFM_THRESHOLD, TRUE);
		//    } else {
		//	SET_ALARM(ALRM_OFM_THRESHOLD, FALSE);
		//    }
		//}
	}

	// Filter offsetFromMaster -- TBD
	//filter(&ptpClock->currentDS.offsetFromMaster.nanoseconds, &ptpClock->ofm_filt);

	// Apply the offset shift
	subTime(&ptpClock->currentDS.offsetFromMaster, &ptpClock->currentDS.offsetFromMaster, &ptpClock->rtOpts->ofmShift);

	//DBGV("offset filter %d\n", ptpClock->ofm_filt->y);

	// Offset must have been computed at least one time before computing end to end delay
	ptpClock->offsetFirstUpdated = TRUE;
	ptpClock->clockControl.offsetOK = TRUE;

/*
	// Check results
	if (abs(ptpClock->currentDS.offsetFromMaster.nanoseconds) < DEFAULT_CALIBRATED_OFFSET_NS){
		if (ptpClock->portDS.portState == PTP_UNCALIBRATED)	{
				setFlag(ptpClock->events, MASTER_CLOCK_SELECTED);
		}
	}
	else if (abs(ptpClock->currentDS.offsetFromMaster.nanoseconds) > DEFAULT_UNCALIBRATED_OFFSET_NS){
		if (ptpClock->portDS.portState == PTP_SLAVE){
				setFlag(ptpClock->events, SYNCHRONIZATION_FAULT);
		}
	}
*/
	//else stay in the current state.
}

void stepClock(PtpClock * ptpClock)
{
	TimeInternal oldTime, newTime;

	if(ptpClock->rtOpts->servo.noAdjust){
		printf("Could not step clock - clock adjustment disabled\n");//WARNING("Could not step clock - clock adjustment disabled\n");
		return;
	}

	printf("stepClock: do set new time, initClock(), and goes to PTP_FAULTY.\r\n");
	//SET_ALARM(ALRM_CLOCK_STEP, TRUE);

	ptpClock->clockControl.stepRequired = FALSE;

	//No need to reset the frequency offset: if we're far off, it will quickly get back to a high value
	getTimeFromReg(&oldTime);//getTime(&oldTime);
	subTime(&newTime, &oldTime, &ptpClock->currentDS.offsetFromMaster);
	setTimeToReg(&newTime); //setTime(&newTime);

	ptpClock->clockStatus.majorChange = TRUE;

	initClock(ptpClock);

	if(ptpClock->defaultDS.clockQuality.clockClass > 127) restoreDrift(ptpClock, TRUE);

	ptpClock->servo.runningMaxOutput = FALSE;

	stm_ptpd_toState(ptpClock, PTP_FAULTY);		// make a full protocol reset

    if(ptpClock->rtOpts->calibrationDelay) {
          ptpClock->isCalibrated = FALSE;
    }
}

/* 11.3 */
//Used in E2E Mode
void updateDelay(PtpClock * ptpClock, const TimeInternal *delayEventEgressTimestamp,
								 const TimeInternal *recieveTimestamp, const TimeInternal *correctionField)
{
	bool checkThreshold = ptpClock->rtOpts->maxDelay;
	TimeInternal prev_meanPathDelay;
	bool maxDelayHit = FALSE;
	TimeInternal slave_to_master_delay;
	Integer16 s;

	// updates paused, leap second pending - do nothing
	if(ptpClock->leapSecondInProgress)	return;

	prev_meanPathDelay.seconds = ptpClock->currentDS.meanPathDelay.seconds;
	prev_meanPathDelay.nanoseconds = ptpClock->currentDS.meanPathDelay.nanoseconds;

	// Tms valid ?
	//if (0 == ptpClock->ofm_filt.n){
	//	DBGV("updateDelay: Tms_offset is not valid");
	//	return;
	//}

	//calc 'slave_to_master_delay'.(Master to Slave delay is already computed in updateOffset
	subTime(&slave_to_master_delay, //
			recieveTimestamp,   //delay_req_receive_time
			delayEventEgressTimestamp); //delay_req_send_time

	if (checkThreshold && ptpClock->offsetFirstUpdated) {
		if ((slave_to_master_delay.nanoseconds < 0) && (abs(slave_to_master_delay.nanoseconds) > ptpClock->rtOpts->maxDelay)) {
						printf("updateDelay aborted delay (sec: %d ns: %d) is negative\n", slave_to_master_delay.seconds, slave_to_master_delay.nanoseconds);
						printf("send (sec: %d ns: %d)\n", delayEventEgressTimestamp->seconds, delayEventEgressTimestamp->nanoseconds);
						printf("recv (sec: %d n	s: %d)\n", recieveTimestamp->seconds,recieveTimestamp->nanoseconds);
						goto finish;
		}
		if (slave_to_master_delay.seconds && checkThreshold) {
						printf("updateDelay aborted, slave to master delay %d.%d greater than 1 second\n", slave_to_master_delay.seconds, slave_to_master_delay.nanoseconds);
						goto finish;
		}
		if (slave_to_master_delay.nanoseconds > ptpClock->rtOpts->maxDelay) {
			//ptpClock->counters.maxDelayDrops++;
			printf("updateDelay aborted, slave to master delay %d greater than administratively set maximum %d\n", slave_to_master_delay.nanoseconds, ptpClock->rtOpts->maxDelay);
			if(ptpClock->rtOpts->maxDelayMaxRejected) {
				maxDelayHit = TRUE;
				//if we blocked maxDelayMaxRejected samples, reset the slave to unblock the filter
				if(++ptpClock->maxDelayRejected > ptpClock->rtOpts->maxDelayMaxRejected) {
					printf("WARNING:%d consecutive measurements above %d threshold - resetting slave\n", ptpClock->rtOpts->maxDelayMaxRejected, slave_to_master_delay.nanoseconds);
					stm_ptpd_toState(ptpClock, PTP_LISTENING);
				}
			}
			goto finish;
		} else {
			ptpClock->maxDelayRejected=0;
		}
	}
	//The packet has passed basic checks, so we'll:
	//   - update the global delaySM variable
	//	 - calculate a new filtered MPD
	if (ptpClock->offsetFirstUpdated) {
		subTime(&ptpClock->delaySM, 		//
				recieveTimestamp,   		//delay_req_receive_time
				delayEventEgressTimestamp); //delay_req_send_time

		//update MeanPathDelay
		addTime(&ptpClock->currentDS.meanPathDelay, &ptpClock->delaySM,	&ptpClock->Tms_offset);
		//Subtract correctionField
		subTime(&ptpClock->currentDS.meanPathDelay, &ptpClock->currentDS.meanPathDelay,	correctionField);
		//Compute one-way delay
		div2Time(&ptpClock->currentDS.meanPathDelay);

		if (ptpClock->currentDS.meanPathDelay.seconds) {
					printf ("update delay: cannot filter with large OFM, clearing filter\n");
					printf("Servo: Ignoring delayResp because of large OFM\n");

					//mpd_filt->s_exp = mpd_filt->nsec_prev = 0;
					// revert back to previous value
					ptpClock->currentDS.meanPathDelay.seconds = prev_meanPathDelay.seconds;
					ptpClock->currentDS.meanPathDelay.nanoseconds = prev_meanPathDelay.nanoseconds;
					return;//goto finish;
		}
		if(ptpClock->currentDS.meanPathDelay.nanoseconds < 0){
					printf("updateDelay>Negative OWD:%dns\n",ptpClock->currentDS.meanPathDelay.nanoseconds);
					// revert back to previous value
					ptpClock->currentDS.meanPathDelay.seconds = prev_meanPathDelay.seconds;
					ptpClock->currentDS.meanPathDelay.nanoseconds = prev_meanPathDelay.nanoseconds;
					return;//goto finish;
		}

		// Filter delay
		if (0 != ptpClock->currentDS.meanPathDelay.seconds){
			printf("updateDelay: cannot filter with seconds\r\n");
		}else{
			filter(&ptpClock->currentDS.meanPathDelay.nanoseconds, &ptpClock->owd_filt);
		}
	} else {
		printf("Ignoring delayResp because we didn't receive any sync yet\n");
		//ptpClock->counters.discardedMessages++;
	}
finish:
	printf("UpdateDelay: Max delay hit: %d\r\n", maxDelayHit);
}
//Used in P2P Mode
void updatePeerDelay(PtpClock *ptpClock, const TimeInternal *correctionField, bool  twoStep)
{
	TimeInternal Tab, Tba;
	DBGV("updatePeerDelay\r\n");

	if (twoStep){
		//Tab = t2-t1.
		//Tba = t4-t3
		//pDelay = Tab + Tba
		subTime(&Tab, &ptpClock->pdelay_t2 , &ptpClock->pdelay_t1);
		subTime(&Tba, &ptpClock->pdelay_t4, &ptpClock->pdelay_t3);
		addTime(&ptpClock->portDS.peerMeanPathDelay, &Tab, &Tba);
	}
	else{ // One step  clock
		subTime(&ptpClock->portDS.peerMeanPathDelay, &ptpClock->pdelay_t4, &ptpClock->pdelay_t1);
	}

	//apply correctionField.
	subTime(&ptpClock->portDS.peerMeanPathDelay, &ptpClock->portDS.peerMeanPathDelay, correctionField);
	div2Time(&ptpClock->portDS.peerMeanPathDelay);

	printf("peerMeanPathDelay=%d.%09ds\r\n",ptpClock->portDS.peerMeanPathDelay.seconds, ptpClock->portDS.peerMeanPathDelay.nanoseconds);
	/* Filter delay */
	if (ptpClock->portDS.peerMeanPathDelay.seconds != 0){ //too big delay over 1 sec.
		DBGV("updatePeerDelay: cannot filter with seconds\r\n");
		printf("updatePeerDelay: cannot filter with seconds\r\n");
		return;
	}else{
		filter(&ptpClock->portDS.peerMeanPathDelay.nanoseconds, &ptpClock->owd_filt);
	}
}

// check if it's OK to update the clock, deal with panic mode, call for clock step
void checkOffset(PtpClock *ptpClock)
{
	RunTimeOpts *rtOpts;
	rtOpts = ptpClock->rtOpts;

	// unless offset OK
	ptpClock->clockControl.updateOK = FALSE;

	// if false, updateOffset does not want us to continue
	if(!ptpClock->clockControl.offsetOK) {
		printf("checkOffset: !offsetOK\n");
		return;
	}

	if(rtOpts->servo.noAdjust) {
		printf("checkOffset: noAdjust\n");
		/* in case if noAdjust has changed */
		ptpClock->clockControl.available = FALSE;
		return;
	}

	// Leap second pending: do not update clock
    if(ptpClock->leapSecondInProgress) {
	    // let the watchdog know that we still want to hold the clock control
	    printf("checkOffset: leapSecondInProgress\n");
	    ptpClock->clockControl.activity = TRUE;
	    return;
	}

	// check if we are allowed to step the clock. If we have already started up once.
	if(!ptpClock->pastStartup && ( rtOpts->servo.stepForce || (rtOpts->servo.stepOnce && ptpClock->currentDS.offsetFromMaster.seconds)))
	{
	    if(rtOpts->servo.stepForce) printf("First clock update - will step the clock\n");
	    if(rtOpts->servo.stepOnce) printf("First clock update and offset >= 1 second - will step the clock\n");
	    ptpClock->clockControl.stepRequired = TRUE;
	    ptpClock->clockControl.updateOK = TRUE;
	    ptpClock->pastStartup = TRUE;
		printf("checkOffset: stepOnce again.\r\n");
	    return;
	}

	// check if offset within allowed limit
	if (rtOpts->maxOffset && ((abs(ptpClock->currentDS.offsetFromMaster.nanoseconds) > abs(rtOpts->maxOffset)) ||
		ptpClock->currentDS.offsetFromMaster.seconds))
	{
		printf("checkOffset:Offset %d.%09d greater than administratively set maximum %d\n. Will not update clock",
				ptpClock->currentDS.offsetFromMaster.seconds,
				ptpClock->currentDS.offsetFromMaster.nanoseconds, rtOpts->maxOffset);
		return;
	}

	// offset above 1 second
	if (ptpClock->currentDS.offsetFromMaster.seconds){
		if(!rtOpts->enablePanicMode) {
			if (!rtOpts->servo.noResetClock){
				printf("Offset above 1 second (%.09f s). Clock will step.\n", timeInternalToDouble(&ptpClock->currentDS.offsetFromMaster));
				printf("checkOffset:Offset above 1 second. Clock will step.\n");
			}
			ptpClock->clockControl.stepRequired = TRUE;
			ptpClock->clockControl.updateOK = TRUE;
			ptpClock->pastStartup = TRUE;
			printf("checkOffset: The very first stepRequired\r\n");
			return;
		}

		// still in panic mode, do nothing
		if(ptpClock->panicMode) {
			printf("checkOffset: still in panic mode\n");
			/* if we have not released clock control, keep the heartbeats going */
			if (ptpClock->clockControl.available) {
				ptpClock->clockControl.activity = TRUE;
			/* if for some reason we don't have clock control and not configured to release it, re-acquire it */
			} else if(!rtOpts->panicModeReleaseClock) {
				ptpClock->clockControl.available = TRUE;
				ptpClock->clockControl.activity = TRUE;
			/* and vice versa - in case the setting has changed */
			} else {
				ptpClock->clockControl.available = FALSE;
			}
			return;
		}

		if(ptpClock->panicOver) {
			if (rtOpts->servo.noResetClock)
				printf("Panic mode timeout - accepting current offset. Clock will be slewed at maximum rate.\n");
			else
				printf("Panic mode timeout - accepting current offset. Clock will step.\n");
			ptpClock->panicOver = FALSE;
			ptpd_timerStop(PANIC_MODE_TIMER,  ptpClock->itimer);
			ptpClock->clockControl.available = TRUE;
			ptpClock->clockControl.stepRequired = TRUE;
			ptpClock->clockControl.updateOK = TRUE;
			ptpClock->pastStartup = TRUE;
			ptpClock->isCalibrated = FALSE;
			return;
		}
		printf("ERROR: Offset above 1 second - entering panic mode. Clock updates paused.\n");
		//CRITICAL("Offset above 1 second (%.09f s)  - entering panic mode. Clock updates paused.\n",
		//	    timeInternalToDouble(&ptpClock->currentDS.offsetFromMaster));
		ptpClock->panicMode = TRUE;
		ptpClock->panicModeTimeLeft = 6 * rtOpts->panicModeDuration;
		ptpd_timerStart(PANIC_MODE_TIMER,1000,ptpClock->itimer);
		// do not release if not configured to do so
		if(rtOpts->panicModeReleaseClock) {
			ptpClock->clockControl.available = FALSE;
		}
		return;

	}

	/* offset below 1 second - exit panic mode if no threshold or if below threshold,
	 * but make sure we stayed in panic mode for at least one interval,
	 * so that we avoid flapping.
	 */
	if(rtOpts->enablePanicMode && ptpClock->panicMode &&
	    (ptpClock->panicModeTimeLeft != (2 * rtOpts->panicModeDuration)) ) {
		if (rtOpts->panicModeExitThreshold == 0) {
			ptpClock->panicMode = FALSE;
			ptpClock->panicOver = FALSE;
			ptpd_timerStop(PANIC_MODE_TIMER, ptpClock->itimer);
			printf("Offset below 1 second again: resuming clock control\n");
			/* we can control the clock again */
			ptpClock->clockControl.available = TRUE;
		} else if ( abs(ptpClock->currentDS.offsetFromMaster.nanoseconds) < rtOpts->panicModeExitThreshold ) {
			ptpClock->panicMode = FALSE;
			ptpClock->panicOver = FALSE;
			ptpd_timerStop(PANIC_MODE_TIMER, ptpClock->itimer);
			printf("Offset below %d ns threshold: resuming clock control\n",
				    ptpClock->currentDS.offsetFromMaster.nanoseconds);
			/* we can control the clock again */
			ptpClock->clockControl.available = TRUE;
		}

	}

	/* can this even happen if offset is < 1 sec? */
	if(rtOpts->enablePanicMode && ptpClock->panicOver) {
			ptpClock->panicMode = FALSE;
			ptpClock->panicOver = FALSE;
			ptpd_timerStop(PANIC_MODE_TIMER, ptpClock->itimer);
			printf("Panic mode timeout and offset below 1 second again: resuming clock control\n");
			/* we can control the clock again */
			ptpClock->clockControl.available = TRUE;
	}
	ptpClock->clockControl.updateOK = TRUE;

}

void updateClock(PtpClock *ptpClock)
{
	int32_t adj;
	TimeInternal timeTmp;
	int32_t offsetNorm;

	//printf("updateClock\r\n");

	if(ptpClock->rtOpts->servo.noAdjust) {
		ptpClock->clockControl.available = FALSE;
		printf("updateClock: noAdjust - skipped clock update\n");
		return;
	}
	if(!ptpClock->clockControl.updateOK) {
		printf("updateClock: !clockUpdateOK - skipped clock update\n");
			return;
	}

	if(ptpClock->clockControl.stepRequired) {
		if (!ptpClock->rtOpts->servo.noResetClock) { //if ResetClock is allowed, we step the clock.

			stepClock(ptpClock); //forced clock adjustment. The state goes to 'PTP_FAULTY'.

			ptpClock->clockControl.stepRequired = FALSE;
		} else {
			if(ptpClock->currentDS.offsetFromMaster.nanoseconds > 0)
				ptpClock->servo.observedDrift = ptpClock->rtOpts->servo.servoMaxPpb;
			else
				ptpClock->servo.observedDrift = -ptpClock->rtOpts->servo.servoMaxPpb;
			//warn_operator_slow_slewing(rtOpts, ptpClock);
			adjFreq(-ptpClock->servo.observedDrift); //adjFreq_wrapper(rtOpts, ptpClock, -ptpClock->servo.observedDrift);
			ptpClock->clockControl.stepRequired = FALSE;
		}
		return;
	}

	if (ptpClock->currentDS.offsetFromMaster.seconds != 0 || abs(ptpClock->currentDS.offsetFromMaster.nanoseconds) > MAX_ADJ_OFFSET_NS)	{
		// if secs, reset clock or set freq adjustment to max
		if (!ptpClock->servo.noAdjust) //if the noAdjust is set NO_ADJUST(==FALSE), it means Adjust is allowed.
		{
			if (!ptpClock->servo.noResetClock) //if we can reset the clock (YES in our case. if the noResetClock is set NO_RESET(==FALSE), it means Reset is allowed.
			{
				getTimeFromReg(&timeTmp); //Get the current TOD.
				subTime(&timeTmp, &timeTmp, &ptpClock->currentDS.offsetFromMaster); //exclude a fixed offset value.
				setTimeToReg(&timeTmp);   //Set the updated TOD.
				//initClock(ptpClock);      //Reset PTPd.
				//printf("updateClock:Call initClock\r\n");
				return; //Add YOON.
			}
			else //otherwise we set max freq adjust.
			{
				adj = ptpClock->currentDS.offsetFromMaster.nanoseconds > 0 ? ADJ_FREQ_MAX : -ADJ_FREQ_MAX;
				adjFreq(-adj);
			}
		}
	}else{ //For small deviations, we use the PI controller.
		// normalize offset to 1s sync interval -> response of the servo will be same for all sync interval values,
		//  but faster/slower (possible lost of precision/overflow but much more stable)
		offsetNorm = ptpClock->currentDS.offsetFromMaster.nanoseconds;
		if (ptpClock->portDS.logSyncInterval > 0)
			offsetNorm >>= ptpClock->portDS.logSyncInterval;
		else if (ptpClock->portDS.logSyncInterval < 0)
			offsetNorm <<= -ptpClock->portDS.logSyncInterval;

		// the accumulator for the I component
		ptpClock->servo.observedDrift += offsetNorm / ptpClock->servo.ai;

		// clamp the accumulator to ADJ_FREQ_MAX for sanity
		if (ptpClock->servo.observedDrift > ADJ_FREQ_MAX)
			ptpClock->servo.observedDrift = ADJ_FREQ_MAX;
		else if (ptpClock->servo.observedDrift < -ADJ_FREQ_MAX)
			ptpClock->servo.observedDrift = -ADJ_FREQ_MAX;

		// apply controller output as a clock tick rate adjustment
		if (!ptpClock->servo.noAdjust){
			adj = offsetNorm / ptpClock->servo.ap + ptpClock->servo.observedDrift;
			adjFreq(-adj);
		}

		if (DEFAULT_PARENTS_STATS){
			int a, scaledLogVariance;
			ptpClock->parentDS.parentStats = TRUE;
			ptpClock->parentDS.observedParentClockPhaseChangeRate = 1100 * ptpClock->servo.observedDrift;

			a = (ptpClock->offsetHistory[1] - 2 * ptpClock->offsetHistory[0] + ptpClock->currentDS.offsetFromMaster.nanoseconds);
			ptpClock->offsetHistory[1] = ptpClock->offsetHistory[0];
			ptpClock->offsetHistory[0] = ptpClock->currentDS.offsetFromMaster.nanoseconds;

			scaledLogVariance = filter_order(a * a) << 8;
			filter(&scaledLogVariance, &ptpClock->slv_filt);
			ptpClock->parentDS.observedParentOffsetScaledLogVariance = 17000 + scaledLogVariance;
			DBGV("updateClock: observed scalled log variance: 0x%x\r\n", ptpClock->parentDS.observedParentOffsetScaledLogVariance);
		}
	}

	switch (ptpClock->portDS.delayMechanism)
	{
		case E2E:
			DBG("updateClock: one-way delay averaged (E2E): %d sec %d nsec\r\n",
					ptpClock->currentDS.meanPathDelay.seconds, ptpClock->currentDS.meanPathDelay.nanoseconds);
			printf("one-way delay : %d s %d ns\r\n",
				ptpClock->currentDS.meanPathDelay.seconds, ptpClock->currentDS.meanPathDelay.nanoseconds);

			break;

		case P2P:
			DBG("updateClock: one-way delay averaged (P2P): %d sec %d nsec\r\n",
					ptpClock->portDS.peerMeanPathDelay.seconds, ptpClock->portDS.peerMeanPathDelay.nanoseconds);
			printf("one-way delay : %d s %d ns\r\n",
				ptpClock->portDS.peerMeanPathDelay.seconds, ptpClock->portDS.peerMeanPathDelay.nanoseconds);
			break;

		default:
			DBG("updateClock: one-way delay not computed\r\n");
	}

	DBG("updateClock: offset from master: %d sec %d nsec\r\n",
			ptpClock->currentDS.offsetFromMaster.seconds,
			ptpClock->currentDS.offsetFromMaster.nanoseconds);
	DBG("updateClock: observed drift: %d\r\n", ptpClock->servo.observedDrift);

	printf("ofm: %d s %d ns\r\n",
		ptpClock->currentDS.offsetFromMaster.seconds,
		ptpClock->currentDS.offsetFromMaster.nanoseconds);

	printf("==================\r\n");

}
//#if (CLOCKSOURCE == CLOCKSOURCE_GPS)
#if (ETHIF_ROLE == PTP_GRANDMASTERCLOCK)
//== add for GMC. Called at each GPS PPS IRQ.
void updateClockForGMC(PtpClock *ptpClock,TimeInternal *gpsTime)
{
	int32_t adj;
	TimeInternal timeMac;
	TimeInternal rdMacTime; //TBR
	int32_t offsetNorm;
	TimeInternal offsetFromGps;
	//int32_t offsetFromGpsNanosec;

	getTimeFromReg(&timeMac); //Get the current TOD from MAC.(call ETH_PTPTime_GetTime())
	subTime(&offsetFromGps, gpsTime, &timeMac); //offset = gpsTime - timeMac.
	if ((offsetFromGps.seconds != 0) || (abs(offsetFromGps.nanoseconds) > MAX_ADJ_OFFSET_NS)) //over 1sec. Forced Reset Clock
	{
		// if |secs|>0, reset clock
		//timeMac.seconds = gpsTime.seconds;
		//subTime(&timeMac, &timeMac, &offsetFromGps); //exclude a fixed offset value.
		setTimeToReg(gpsTime);//Set the updated TOD.
		//getTimeFromReg(&rdMacTime);//test
		//printf("resetting system clock to %dsec %d nsec\r\r\n", timeMac.seconds, timeMac.nanoseconds);
		initClock(ptpClock);      //Reset PTPd.
		printf("\r\nLargeOffset(%d:%d):Reset Clock\r\r\n\r\r\n",offsetFromGps.seconds, offsetFromGps.nanoseconds);
		//printf("-(%d:%d)\r\r\n",offsetFromGps.seconds, offsetFromGps.nanoseconds);
		//printf("(%d:%d)(%d:%d)\r\r\n",timeMac.seconds, timeMac.nanoseconds, rdMacTime.seconds, rdMacTime.nanoseconds);
	}
	else{
		/* the PI controller */
		/* normalize offset to 1s sync interval -> response of the servo will be same for all sync interval values,
		 * but faster/slower (possible lost of precision/overflow but much more stable) */
		offsetNorm = offsetFromGps.nanoseconds;

		// the accumulator for the I component
		ptpClock->servo.observedDrift += offsetNorm / ptpClock->servo.ai;//16

		/* clamp the accumulator to ADJ_FREQ_MAX for sanity */
		if (ptpClock->servo.observedDrift > ADJ_FREQ_MAX)
			ptpClock->servo.observedDrift = ADJ_FREQ_MAX;
		else if (ptpClock->servo.observedDrift < -ADJ_FREQ_MAX)
			ptpClock->servo.observedDrift = -ADJ_FREQ_MAX;

		/* apply controller output as a clock tick rate adjustment */
		adj = offsetNorm / ptpClock->servo.ap + ptpClock->servo.observedDrift;
		adjFreq(adj); //adjFreq(-adj);
		//printf("(offsetNorm=%d, adj=%d, ap=%d, drift=%d)\r\r\n",offsetNorm, adj, ptpClock->servo.ap, ptpClock->observedDrift);
	}
}
#endif

//== News=====
/*
//YOON 2016.09.30 -- TBD
void setupPIservo(PIservo* servo, const RunTimeOpts* rtOpts)
{

    servo->maxOutput = rtOpts->servo.servoMaxPpb;
    servo->kP = rtOpts->servo.servoKP;
    servo->kI = rtOpts->servo.servoKI;
    servo->dTmethod = rtOpts->servo.servoDtMethod;

#ifdef PTPD_STATISTICS
    servo->stabilityThreshold = rtOpts->servoStabilityThreshold;
    servo->stabilityPeriod = rtOpts->servoStabilityPeriod;
    servo->stabilityTimeout = (60 / rtOpts->statsUpdateInterval) * rtOpts->servoStabilityTimeout;
#endif
}
*/
void resetPIservo(PIservo* servo)
{
/* not needed: restoreDrift handles this */
/*   servo->observedDrift = 0; */
    servo->input = 0;
    servo->output = 0;
    servo->lastUpdate.seconds = 0;
    servo->lastUpdate.nanoseconds = 0;
}
/*
//sys.c
void getTimeMonotonic(TimeInternal * time)
{
#if defined(_POSIX_TIMERS) && (_POSIX_TIMERS > 0)

	struct timespec tp;
#ifndef CLOCK_MONOTINIC
	if (clock_gettime(CLOCK_REALTIME, &tp) < 0) {
#else
	if (clock_gettime(CLOCK_MONOTONIC, &tp) < 0) {
#endif // CLOCK_MONOTONIC
		PERROR("clock_gettime() failed, exiting.");
		exit(0);
	}
	time->seconds = tp.tv_sec;
	time->nanoseconds = tp.tv_nsec;
#else

	struct timeval tv;
	gettimeofday(&tv, 0);//TBRETORE
	time->seconds = tv.tv_sec;
	time->nanoseconds = tv.tv_usec * 1000;

#endif
}

double runPIservo(PIservo* servo, const Integer32 input)
{

        double dt;

        TimeInternal now, delta;

        switch (servo->dTmethod) {

        case DT_MEASURED:

                getTimeMonotonic(&now);
                if(servo->lastUpdate.seconds == 0 &&
                servo->lastUpdate.nanoseconds == 0) {
                        dt = servo->dT;
                } else {
                        subTime(&delta, &now, &servo->lastUpdate);
                        dt = delta.seconds + delta.nanoseconds / 1E9;
                }

                // Don't use dT longer then max update interval multiplier
                if(dt > (servo->maxdT * servo->dT))
                        dt = (servo->maxdT + 0.0) * servo->dT;

                break;

        case DT_CONSTANT:

                dt = servo->dT;

		break;

        case DT_NONE:
        default:
                dt = 1.0;
                break;
        }

        if(dt <= 0.0)
            dt = 1.0;

	servo->input = input;

	if (servo->kP < 0.000001)
		servo->kP = 0.000001;
	if (servo->kI < 0.000001)
		servo->kI = 0.000001;

	servo->observedDrift +=
		dt * ((input + 0.0 ) * servo->kI);

	if(servo->observedDrift >= servo->maxOutput) {
		servo->observedDrift = servo->maxOutput;
		servo->runningMaxOutput = TRUE;
#ifdef PTPD_STATISTICS
		servo->stableCount = 0;
		servo->updateCount = 0;
		servo->isStable = FALSE;
#endif
	}
	else if(servo->observedDrift <= -servo->maxOutput) {
		servo->observedDrift = -servo->maxOutput;
		servo->runningMaxOutput = TRUE;
#ifdef PTPD_STATISTICS
		servo->stableCount = 0;
		servo->updateCount = 0;
		servo->isStable = FALSE;
#endif
	} else {
		servo->runningMaxOutput = FALSE;
	}

	servo->output = (servo->kP * (input + 0.0) ) + servo->observedDrift;

	if(servo->dTmethod == DT_MEASURED)
		servo->lastUpdate = now;

	DBGV("Servo dt: %.09f, input (ofm): %d, output(adj): %.09f, accumulator (observed drift): %.09f\n", dt, input, servo->output, servo->observedDrift);

	return -servo->output;

}

#ifdef PTPD_STATISTICS
static void
checkServoStable(PtpClock *ptpClock, const RunTimeOpts *rtOpts)
{

        DBG("servo stablecount: %d\n",ptpClock->servo.stableCount);

	// if not calibrated, do nothing
	if( !rtOpts->calibrationDelay || ptpClock->isCalibrated ) {
		++ptpClock->servo.updateCount;
	} else {
		return;
	}

	// check if we're below the threshold or not
	if(ptpClock->servo.runningMaxOutput || !ptpClock->acceptedUpdates ||
	    (ptpClock->servo.driftStdDev > ptpClock->servo.stabilityThreshold)) {
	    ptpClock->servo.stableCount = 0;
	} else if (ptpClock->servo.driftStdDev <= ptpClock->servo.stabilityThreshold) {
	    ptpClock->servo.stableCount++;
	}

	// Servo considered stable - drift std dev below threshold for n measurements - saving drift
        if(ptpClock->servo.stableCount >= ptpClock->servo.stabilityPeriod) {

		if(!ptpClock->servo.isStable) {
                        NOTICE("Clock servo now within stability threshold of %.03f ppb\n",
				ptpClock->servo.stabilityThreshold);
		}

                saveDrift(ptpClock, rtOpts, ptpClock->servo.isStable);

		ptpClock->servo.isStable = TRUE;
		ptpClock->servo.stableCount = 0;
		ptpClock->servo.updateCount = 0;

	// servo not stable within max interval
        } else if(ptpClock->servo.updateCount >= ptpClock->servo.stabilityTimeout) {

		ptpClock->servo.stableCount = 0;
		ptpClock->servo.updateCount = 0;

		if(ptpClock->servo.isStable) {
                        WARNING("Clock servo outside stability threshold (%.03f ppb dev > %.03f ppb thr). Too many warnings may mean the threshold is too low.\n",
			    ptpClock->servo.driftStdDev,
			    ptpClock->servo.stabilityThreshold);
                        ptpClock->servo.isStable = FALSE;
		} else {
                        if(ptpClock->servo.runningMaxOutput) {
				WARNING("Clock servo outside stability threshold after %d seconds - running at maximum rate.\n",
					rtOpts->statsUpdateInterval * ptpClock->servo.stabilityTimeout);
                        } else {
                                WARNING("Clock servo outside stability threshold %d seconds after last check. Saving current observed drift.\n",
					rtOpts->statsUpdateInterval * ptpClock->servo.stabilityTimeout);
				saveDrift(ptpClock, rtOpts, FALSE);
                        }
		}
	}

}
#endif
*/
//in sys.c

void restoreDrift(PtpClock * ptpClock, bool quiet)
{

	return; //TBD


	RunTimeOpts * rtOpts = ptpClock->rtOpts;
	//FILE *driftFP;
	bool reset_offset = FALSE;
	double recovered_drift;

	DBGV("restoreDrift called\n");
/*
	if (ptpClock->drift_saved && rtOpts->drift_recovery_method > 0 ) {
		ptpClock->servo.observedDrift = ptpClock->last_saved_drift;
		if (!rtOpts->noAdjust && ptpClock->clockControl.granted) {
			adjFreq_wrapper(rtOpts, ptpClock, -ptpClock->last_saved_drift);
		}
		DBG("loaded cached drift\n");
		return;
	}

	switch (rtOpts->drift_recovery_method) {

		case DRIFT_FILE:

			if( (driftFP = fopen(rtOpts->driftFile,"r")) == NULL) {
			    if(errno!=ENOENT) {
				    PERROR("Could not open drift file: %s - using current kernel frequency offset. Ignore this error if ",
				    rtOpts->driftFile);
			    } else {
				    NOTICE("Drift file %s not found - will be initialised on write\n",rtOpts->driftFile);
			    }
			} else if (fscanf(driftFP, "%lf", &recovered_drift) != 1) {
				PERROR("Could not load saved offset from drift file - using current kernel frequency offset");
				fclose(driftFP);
			} else {

			if(recovered_drift == 0)
				recovered_drift = 0;

			fclose(driftFP);
			if(quiet)
				DBGV("Observed drift loaded from %s: "DRIFTFORMAT" ppb\n",
					rtOpts->driftFile,
					recovered_drift);
			else
				INFO("Observed drift loaded from %s: "DRIFTFORMAT" ppb\n",
					rtOpts->driftFile,
					recovered_drift);
				break;
			}

		case DRIFT_KERNEL:
#ifdef HAVE_SYS_TIMEX_H
			recovered_drift = -getAdjFreq();
#else
			recovered_drift = 0;
#endif
			if(recovered_drift == 0)
				recovered_drift = 0;

			if (quiet)
				DBGV("Observed_drift loaded from kernel: "DRIFTFORMAT" ppb\n",
					recovered_drift);
			else
				INFO("Observed_drift loaded from kernel: "DRIFTFORMAT" ppb\n",
					recovered_drift);

		break;


		default:

			reset_offset = TRUE;

	}

	if (reset_offset) {
		if (!rtOpts->noAdjust && ptpClock->clockControl.granted)
		  adjFreq_wrapper(rtOpts, ptpClock, 0);
		ptpClock->servo.observedDrift = 0;
		return;
	}

	ptpClock->servo.observedDrift = recovered_drift;

	ptpClock->drift_saved = TRUE;
	ptpClock->last_saved_drift = recovered_drift;

	if (!rtOpts->noAdjust)
		adjFreq_wrapper(rtOpts, ptpClock, -recovered_drift);
*/
}
#endif
