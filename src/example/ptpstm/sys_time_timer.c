#include <stdio.h>
#include <time.h>
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
#include "ptpstm/include/ptpd.h"

//*****************************************************************************
// This array contains the number of days in a year at the beginning of each
// month of the year, in a non-leap year.
//*****************************************************************************
static const short g_psDaysToMonth[12] ={0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

//*****************************************************************************
//! Converts from seconds to calendar date and time.
//! \param ulTime is the number of seconds.
//! \param psTime is a pointer to the time structure that is filled in with the
//! broken down date and time.
//! This function converts a number of seconds since midnight GMT on January 1,
//! 1970 (traditional Unix epoch) into the equivalent month, day, year, hours,
//! minutes, and seconds representation.
//! \return None.
//
//*****************************************************************************
void ulocaltime(unsigned long ulTime, tTod *psTime)
{
    unsigned long ulTemp, ulMonths;

    //
    // Extract the number of seconds, converting time to the number of minutes.
    //
    ulTemp = ulTime / 60;
    psTime->ucSec = ulTime - (ulTemp * 60);
    ulTime = ulTemp;

    //
    // Extract the number of minutes, converting time to the number of hours.
    //
    ulTemp = ulTime / 60;
    psTime->ucMin = ulTime - (ulTemp * 60);
    ulTime = ulTemp;

    //
    // Extract the number of hours, converting time to the number of days.
    //
    ulTemp = ulTime / 24;
    psTime->ucHour = ulTime - (ulTemp * 24);
    ulTime = ulTemp;

    //
    // Compute the day of the week.
    //
    psTime->ucWday = (ulTime + 4) % 7;

    //
    // Compute the number of leap years that have occurred since 1968, the
    // first leap year before 1970.  For the beginning of a leap year, cut the
    // month loop below at March so that the leap day is classified as February
    // 29 followed by March 1, instead of March 1 followed by another March 1.
    //
    ulTime += 366 + 365;
    ulTemp = ulTime / ((4 * 365) + 1);
    if((ulTime - (ulTemp * ((4 * 365) + 1))) > (31 + 28))
    {
        ulTemp++;
        ulMonths = 12;
    }
    else
    {
        ulMonths = 2;
    }

    //
    // Extract the year.
    //
    psTime->usYear = ((ulTime - ulTemp) / 365) + 1968;
    ulTime -= ((psTime->usYear - 1968) * 365) + ulTemp;

    //
    // Extract the month.
    //
    for(ulTemp = 0; ulTemp < ulMonths; ulTemp++){
        if(g_psDaysToMonth[ulTemp] > ulTime) {
            break;
        }
    }
    psTime->ucMon = ulTemp; //psTime->ucMon = ulTemp - 1;

    //
    // Extract the day of the month.
    //
    psTime->ucMday = ulTime - g_psDaysToMonth[ulTemp - 1] + 1;
}

// this file deals a real TOD clock.
void utc2tod(u32 utc, u32 nanosec) {
	u8 hr, min, sec;
	tTod tod;
	char str[80];
    // Convert the elapsed seconds (ulSeconds) into time structure.
    ulocaltime(utc, &tod);
	printf("[UTC=%d] ToD=%04d/%02d/%02d %02d:%02d:%02d.%09d \r\n",utc, tod.usYear, tod.ucMon, tod.ucMday, tod.ucHour, tod.ucMin, tod.ucSec, nanosec);
	//sprintf(str,"ToD=%04d/%02d/%02d %02d:%02d:%02d.%09d",tod.usYear, tod.ucMon, tod.ucMday, tod.ucHour, tod.ucMin, tod.ucSec, nanosec);
	//showTimeToLCD(str,10,50);
}

int tod2utc(unsigned year, unsigned month, unsigned day, unsigned h, unsigned m, unsigned sec) {
	tTod tod;
	char str[80];
	struct tm tm;
	int utc;

	tm.tm_year = year - 1900;
	tm.tm_mon = month - 1;
	tm.tm_mday = day;
	tm.tm_hour = h;
	tm.tm_min = m;
	tm.tm_sec = sec;
	tm.tm_isdst = -1;

	utc = mktime(&tm);
	return utc;
}

void displayState(const PtpClock *ptpClock){
	const char *s;
	int32_t Offset=0;
	int32_t pDelay=0;
	char str[80];
	switch (ptpClock->portDS.portState)
	{
		case PTP_INITIALIZING:  s = "+Init        ";  break;
		case PTP_FAULTY:        s = "+Faulty      ";   break;
		case PTP_LISTENING:     s = "+Listening   ";  break;
		case PTP_PASSIVE:       s = "+Passive     ";  break;
		case PTP_UNCALIBRATED:  s = "+UnCalibrated";  break;
		case PTP_SLAVE:         s = "+Slave       ";
			break;
		case PTP_PRE_MASTER:    s = "+PreMaster  ";  break;
		case PTP_MASTER:        s = "+Master      ";
			break;
		break;
		case PTP_DISABLED:      s = "+Disabled    ";  break;
		default:                s = "-?           ";     break;
	}

	stmOzOLED_printString(s,0,2,16);
/*
	if (ptpClock->currentDS.offsetFromMaster.seconds){
		Offset= ptpClock->currentDS.offsetFromMaster.seconds;
		if(Offset<0)
			sprintf(str,"Ofm:-%09usec",abs(Offset));
		else
			sprintf(str,"Ofm:+%09usec",Offset);
	}else{
		Offset= ptpClock->currentDS.offsetFromMaster.nanoseconds;
		if(Offset<0)
			sprintf(str,"Ofm:-%09uns",abs(Offset));
		else
			sprintf(str,"Ofm:+%09uns",Offset);
	}
	stmOzOLED_printString(str,0,3,16);
	printf("%s\r\n",str);
*/
}

extern void PTP_SyncLed(unsigned char on_off);
unsigned char g_locked=0;
void displayStats(const PtpClock *ptpClock)
{
	const char *s;
	unsigned char *uuid;
	char sign=' ';
	TimeInternal tmpTime;
	char str[80];
	tTod tod;
	unsigned short ms,us,ns;
	int32_t pDelay=0;
	int32_t Offset=0;
	int32_t observedDrift = 0;

	getTimeFromReg(&tmpTime);
    ulocaltime(tmpTime.seconds, &tod);

	switch (ptpClock->portDS.delayMechanism)
	{
		case E2E:
			pDelay = ptpClock->currentDS.meanPathDelay.nanoseconds;
			break;
		case P2P:
			pDelay = ptpClock->portDS.peerMeanPathDelay.nanoseconds;
			break;
	}

	if (ptpClock->currentDS.offsetFromMaster.seconds){
		Offset= ptpClock->currentDS.offsetFromMaster.seconds;
		g_locked = 0;
		if(Offset<0)
			sprintf(str,"Ofm:-%04ds ",abs(Offset));
		else
			sprintf(str,"Ofm:+%04ds ",Offset);
	}else{
		Offset= ptpClock->currentDS.offsetFromMaster.nanoseconds;
		if(abs(Offset) < DEFAULT_PTP_ALLOWABLE_JITTER_FOR_SYNCRONIZED_IN_NS){ //<500nsec
			g_locked = 1;
		}
		else{
			g_locked = 0;
		}
		if(Offset<0)
			sprintf(str,"Ofm:-%04dns ",abs(Offset));
		else
			sprintf(str,"Ofm:+%04dns ",Offset);
	}



	//Offset

	stmOzOLED_printString(str,0,3,16);
	/* Observed drift from master */
	if (ptpClock->servo.observedDrift > 0) sign = '+';
	if (ptpClock->servo.observedDrift < 0) sign = '-';

	observedDrift=ptpClock->servo.observedDrift;
	//showTimeToLCD(str,10,26);

    //utc2tod(tmpTime.seconds, tmpTime.nanoseconds);
	//sprintf(str,"%04d/%02d/%02d %02d:%02d:%02d.%09d",tod.usYear, tod.ucMon, tod.ucMday, tod.ucHour, tod.ucMin, tod.ucSec, nanosec);
	sprintf(str,"%04d/%02d/%02d",tod.usYear, tod.ucMon, tod.ucMday);
	stmOzOLED_printString(str,0,5,16);
	//printf("\r\n%s ",str);
	sprintf(str,"%02d:%02d:%02d",tod.ucHour, tod.ucMin, tod.ucSec);
	stmOzOLED_printString(str,0,6,16);
	//printf("%s ",str);
	ms = tmpTime.nanoseconds / 1000000;
	us = tmpTime.nanoseconds / 1000;	us = us % 1000;
	ns = tmpTime.nanoseconds % 1000;
	sprintf(str,"%03d.%03d.%03d",ms,us,ns);
	stmOzOLED_printString(str,0,7,16);
	//printf("%s\r\n",str);
#ifdef USE_MAX7219
	 //Show current TOD on 7 segments.
	 PTP_max7219_write16digits(
			tod.ucHour, tod.ucMin, tod.ucSec,
			tmpTime.nanoseconds
	 );
#endif

	//getTimeFromReg(&tmpTime);
	//utc2tod(tmpTime.seconds, tmpTime.nanoseconds);
	//sprintf(str,"ToD=%04d/%02d/%02d %02d:%02d:%02d.%09d",tod.usYear, tod.ucMon, tod.ucMday, tod.ucHour, tod.ucMin, tod.ucSec, nanosec);
	//showTimeToLCD(str,10,50);


	//uuid = (unsigned char*) ptpClock->parentDS.parentPortIdentity.clockIdentity;

	/* Master clock UUID
	printf("%02X%02X:%02X%02X:%02X%02X:%02X%02X\n",
					uuid[0], uuid[1],
					uuid[2], uuid[3],
					uuid[4], uuid[5],
					uuid[6], uuid[7]);
	 */
	// One way delay
	switch (ptpClock->portDS.delayMechanism)
	{
		case E2E:
			//printf("E2E pDelay: %d ns\r\n", pDelay);
			break;
		case P2P:
			//printf("P2P pDelay: %d ns\r\n", pDelay);
			break;
		default:
			printf("pDelay: unknown\r\n");
			/* none */
			break;
	}

	/* Offset from master */
	if (ptpClock->currentDS.offsetFromMaster.seconds){
		sprintf(str,"Offset=%d sec", Offset);
	}else{
		sprintf(str,"Offset=%09d nsec", Offset);
	}


	//showTimeToLCD(str,10,38);
	//printf("%s\r\n",str);
	/* Observed drift from master */
	sign = ' ';
	if (observedDrift > 0) sign = '+';
	if (observedDrift < 0) sign = '-';

	sprintf(str,"Drift=%c%d.%03d ppm", sign, abs(observedDrift / 1000), abs(observedDrift % 1000));
	//showTimeToLCD(str,10,26);
	//printf("%s\r\n",str);

	switch (ptpClock->portDS.portState)
	{
		case PTP_INITIALIZING:  s = "+Init        ";  break;
		case PTP_FAULTY:        s = "+Faulty      ";   break;
		case PTP_LISTENING:     s = "+Listening   ";  break;
		case PTP_PASSIVE:       s = "+Passive     ";  break;
		case PTP_UNCALIBRATED:  s = "+Uncalibrated";
			break;
		case PTP_SLAVE:         s = "+Slave       ";
			g_locked ? 	PTP_SyncLed(1):	PTP_SyncLed(0);
			break;
		case PTP_PRE_MASTER:    s = "+PreMaster  ";  break;
		case PTP_MASTER:        s = "+Master      ";
			break;
		break;
		case PTP_DISABLED:      s = "+Disabled    ";  break;
		default:                s = "-?           ";     break;
	}

	/* State of the PTP */
	//printf("port State: %s\r\n", s);
	stmOzOLED_printString(s,0,2,16);
}

//we can set the current TOD also.
void setTimeToReg(const TimeInternal *time)
{
	struct ptptime_t ts;

	ts.tv_sec = time->seconds;
	ts.tv_nsec = time->nanoseconds;
	ethernetif_PTPTime_SetTime(&ts);
	/*printf("resetting system clock to %dsec %d nsec\r\n", time->seconds, time->nanoseconds);*/

#if (ETHIF_ROLE == PTP_OC)
//	#if(SWITCH_ID == SWITCH_LAN9355)
//	Lan9355_Set_Clock_and_Target_Clock(time->seconds, time->nanoseconds);
//	#endif
#endif
}
extern TimeInternal Lan9355_Get_Clock();
// we can get the current TOD if we are slaves.
void getTimeFromReg(TimeInternal *time)
{
	static struct ptptime_t timestamp;
	TimeInternal rettime;
//#if ((ETHIF_ROLE == PTP_OC) && (SWITCH_ID == SWITCH_LAN9355))
//	rettime = Lan9355_Get_Clock();
//	time->seconds = rettime.seconds ;
//	time->nanoseconds = rettime.nanoseconds;
//#else
	ETH_PTPTime_GetTime(&timestamp);
	time->seconds = timestamp.tv_sec;
	time->nanoseconds = timestamp.tv_nsec;
//#endif

}

// adjust TOD with the given time offset.(positive value will move the needles backward.)
// Only used in Coarse Method
void updateTimeWithOffsetReg(const TimeInternal *time)
{
	struct ptptime_t timeoffset;

	DBGV("updateTime: %d sec %d nsec\r\n", time->seconds, time->nanoseconds);

	timeoffset.tv_sec = -time->seconds;
	timeoffset.tv_nsec = -time->nanoseconds;

	/* Coarse update method */
	ETH_PTPTime_UpdateOffset(&timeoffset);
	DBGV("updateTime: updated\r\n");
}

unsigned int getRand(uint32_t randMax)
{
	return rand() % randMax;
}

bool  adjFreq(int32_t adj)
{
	printf("adjFreq %d\r\n", adj);

	if (adj > ADJ_FREQ_MAX)
		adj = ADJ_FREQ_MAX;
	else if (adj < -ADJ_FREQ_MAX)
		adj = -ADJ_FREQ_MAX;

	//Used only for Fine update method
	ethernetif_PTPTime_AdjFreq(adj);

	return TRUE;
}
#endif
