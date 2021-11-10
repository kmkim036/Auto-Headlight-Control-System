#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"

#include "ethopts.h"
#include "yInc.h"
#if (PROJ_FOR == PROJ_FOR_PTP) || (PROJ_FOR == PROJ_FOR_RSTP_PTP)
//#include "lwip/include/lwipopts.h"
#include "ptpstm/include/constants.h"
//#include "ptpstm/include/datatypes.h"
#include "ptpstm/include/ptpd.h"
/* arith.c */

/*
//YOON 2016.10.1
void internalTime_to_integer64(TimeInternal internal, Integer64ML *bigint)
{
	int64_t scaledNanoseconds;

	scaledNanoseconds = internal.seconds;
	scaledNanoseconds *= 1000000000;
	scaledNanoseconds += internal.nanoseconds;
	scaledNanoseconds <<= 16;

	bigint->msb = (scaledNanoseconds >> 32) & 0x00000000ffffffff;
	bigint->lsb = scaledNanoseconds & 0x00000000ffffffff;
}
*/
//YOON 2016.10.1
//was scaledNanosecondsToInternalTime
void integer64_to_internalTime(Integer64LM *bigint, TimeInternal * internal)
{
	int sign;
	int64_t scaledNanoseconds;

	scaledNanoseconds = bigint->msb;
	scaledNanoseconds = scaledNanoseconds << 32;
	scaledNanoseconds = scaledNanoseconds | bigint->lsb;

	//determine sign of result big integer number

	if (scaledNanoseconds < 0) {
		scaledNanoseconds = -scaledNanoseconds;
		sign = -1;
	} else {
		sign = 1;
	}

	//fractional nanoseconds are excluded (see 5.3.2)
	scaledNanoseconds >>= 16;
	internal->seconds = sign * (scaledNanoseconds / 1000000000);
	internal->nanoseconds = sign * (scaledNanoseconds % 1000000000);
}
/* sameAs the integer64_to_internalTime
void scaledNanosecondsToInternalTime(const int64_t *scaledNanoseconds, TimeInternal *internal)
{
	int sign;
	int64_t nanoseconds = *scaledNanoseconds;

	// Determine sign of result big integer number
	if (nanoseconds < 0)
	{
		nanoseconds = -nanoseconds;
		sign = -1;
	}
	else
	{
		sign = 1;
	}

	// fractional nanoseconds are excluded (see 5.3.2)
	nanoseconds >>= 16;
	internal->seconds = sign * (nanoseconds / 1000000000);
	internal->nanoseconds = sign * (nanoseconds % 1000000000);
}
*/
void fromInternalTime(const TimeInternal *internal, Timestamp *external)
{
	/* fromInternalTime is only used to convert time given by the system to a timestamp.
	 * As a consequence, no negative value can normally be found in (internal)
	 * Note that offsets are also represented with TimeInternal structure, and can be negative,
	 * but offset are never convert into Timestamp so there is no problem here.*/
	if ((internal->seconds & ~INT_MAX) || (internal->nanoseconds & ~INT_MAX))
	{
		DBG("Negative value canno't be converted into timestamp \n");
		return;
	}
	else
	{
		external->secondsField.lsb = internal->seconds;
		external->nanosecondsField = internal->nanoseconds;
		external->secondsField.msb = 0;
	}
}

void toInternalTime(TimeInternal *internal, const Timestamp *external)
{
	// Program will not run after 2038...
	if (external->secondsField.lsb < INT_MAX){
		internal->seconds = external->secondsField.lsb;
		internal->nanoseconds = external->nanosecondsField;
	}else{
		DBG("Clock servo canno't be executed : seconds field is higher than signed integer (32bits)\n");
		return;
	}
}
//YOON 2016.10.1
void ts_to_InternalTime(const struct timespec *a,  TimeInternal * b)
{
	b->seconds = a->tv_sec;
	b->nanoseconds = a->tv_nsec;
}
/* compile error ?
void tv_to_InternalTime(const struct timeval *a,  TimeInternal *b)
{

	b->seconds = (int32_t)(a->tv_sec);
	b->nanoseconds = (int32_t)(a->tv_usec * 1000);
}
*/
void normalizeTime(TimeInternal *r)
{
	r->seconds += r->nanoseconds / 1000000000;
	r->nanoseconds -= r->nanoseconds / 1000000000 * 1000000000;

	if (r->seconds > 0 && r->nanoseconds < 0)
	{
		r->seconds -= 1;
		r->nanoseconds += 1000000000;
	}
	else if (r->seconds < 0 && r->nanoseconds > 0)
	{
		r->seconds += 1;
		r->nanoseconds -= 1000000000;
	}
}

void addTime(TimeInternal *r, const TimeInternal *x, const TimeInternal *y)
{
	r->seconds = x->seconds + y->seconds;
	r->nanoseconds = x->nanoseconds + y->nanoseconds;

	normalizeTime(r);
}

void subTime(TimeInternal *r, const TimeInternal *x, const TimeInternal *y)
{
	r->seconds = x->seconds - y->seconds;
	r->nanoseconds = x->nanoseconds - y->nanoseconds;
	normalizeTime(r);
}

void div2Time(TimeInternal *r)
{
	r->nanoseconds += r->seconds % 2 * 1000000000;
	r->seconds /= 2;
	r->nanoseconds /= 2;

	normalizeTime(r);
}

// clear an internal time value
void clearTime(TimeInternal *r)
{
	r->seconds     = 0;
	r->nanoseconds = 0;
}

//  sets a time value to a certain nanoseconds
void nano_to_Time(TimeInternal *x, int nano)
{
	x->seconds     = 0;
	x->nanoseconds = nano;
	normalizeTime(x);
}
// greater than operation
int gtTime(const TimeInternal *x, const TimeInternal *y)
{
	TimeInternal r;

	subTime(&r, x, y);
	return !isTimeInternalNegative(&r);
}

// remove sign from variable
void absTime(TimeInternal *r)
{
	// Make sure signs are the same
	normalizeTime(r);
	r->seconds       = abs(r->seconds);
	r->nanoseconds   = abs(r->nanoseconds);
}

// if 2 time values are close enough for X nanoseconds
int is_Time_close(const TimeInternal *x, const TimeInternal *y, int nanos)
{
	TimeInternal r1;
	TimeInternal r2;

	// first, subtract the 2 values. then call abs(),
	// then call gtTime for requested the number of nanoseconds
	subTime(&r1, x, y);
	absTime(&r1);

	nano_to_Time(&r2, nanos);

	return !gtTime(&r1, &r2);
}
int check_timestamp_is_fresh2(const TimeInternal * timeA, const TimeInternal * timeB)
{
	int ret;

	// maximum 1 millisecond offset
	ret = is_Time_close(timeA, timeB, 1000000);
	DBG2("check_timestamp_is_fresh: %d\n ", ret);
	return ret;
}


int check_timestamp_is_fresh(const TimeInternal * timeA)
{
	TimeInternal timeB;
	getTime(&timeB);

	return check_timestamp_is_fresh2(timeA, &timeB);
}


int isTimeInternalNegative(const TimeInternal * p)
{
	return (p->seconds < 0) || (p->nanoseconds < 0);
}

double timeInternalToDouble(const TimeInternal * p)
{

	double sign = (p->seconds < 0 || p->nanoseconds < 0 ) ? -1.0 : 1.0;
	return (sign * ( abs(p->seconds) + abs(p->nanoseconds) / 1E9 ));

}

TimeInternal doubleToTimeInternal(const double d)
{

	TimeInternal t = {0, 0};

	t.seconds = trunc(d);
	t.nanoseconds = (d - (t.seconds + 0.0)) * 1E9;

	return t;

}

int floorLog2(unsigned int n)
{
	int pos = 0;

	if (n == 0)
		return -1;

	if (n >= 1<<16) { n >>= 16; pos += 16; }
	if (n >= 1<< 8) { n >>=  8; pos +=  8; }
	if (n >= 1<< 4) { n >>=  4; pos +=  4; }
	if (n >= 1<< 2) { n >>=  2; pos +=  2; }
	if (n >= 1<< 1) {           pos +=  1; }
	return pos;
}
#endif
