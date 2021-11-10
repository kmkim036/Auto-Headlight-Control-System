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
static int g_iElapsedMilliSeconds = 0;


//These Timers are used for handling PTPd protocols such as periodic transmission of SYNC, ANNOUNCE,...
//IntervalTimer  itimer[TIMER_ARRAY_SIZE];
///==================== timer.c ==================
/*
typedef struct
{
	osTimerId    id;
	osTimerDef_t def;
#ifdef __CMSIS_RTOS
	int32_t      data[5];
#endif
} sys_timer_t;


// An array to hold the various system timer handles.
static sys_timer_t ptpdTimers[TIMER_ARRAY_SIZE];
static bool ptpdTimersExpired[TIMER_ARRAY_SIZE];

// see cmsis_os.h
typedef void (*os_ptimer) (void const *argument);
/// Timer Definition structure contains timer parameters.
/// \note CAN BE CHANGED: \b os_timer_def is implementation specific in every CMSIS-RTOS.
typedef const struct os_timer_def  {
  os_ptimer                 ptimer;    ///< start address of a timer function
} osTimerDef_t;
/// Timer type value for the timer definition
/// \note MUST REMAIN UNCHANGED: \b os_timer_type shall be consistent in every CMSIS-RTOS.
typedef enum  {
  osTimerOnce             =     0,       ///< one-shot timer
  osTimerPeriodic         =     1        ///< repeating timer
} os_timer_type;
*/

//=== sys_arch.c =====================================

/** Create a new timer
 * @param timer pointer to the timer to create
 * @param ptimer pointer to timer callback function
 * @param type timer type
 * @param argument generic argument type
 * @return a new mutex */
/*
err_t sys_timer_new(sys_timer_t *timer, os_ptimer ptimer, os_timer_type type, void *argument)
{
#ifdef __CMSIS_RTOS
	memset(timer->data, 0, sizeof(int32_t)*5);
	timer->def.timer = timer->data;
	timer->def.ptimer = ptimer;
#endif
	timer->id = osTimerCreate(&timer->def, type, argument);
	if (timer->id == NULL)
		return ERR_MEM;

	return ERR_OK;
}
*/
void ptpd_initTimer(void)
{
  DBG("ptpd_initTimer\n");

  g_iElapsedMilliSeconds = 0;
}

void ptpd_timerTick(int iTickMilliSeconds)
{
    g_iElapsedMilliSeconds += iTickMilliSeconds; //=1
    //DBGV("timerTick: iElapsedMs = %d\n",iElapsedMilliSeconds );
}
#if (PTP_PROTOCOL == IEEE1588V2)
//Valid each Second.
void ptpd_timerUpdate(IntervalTimer *itimer)
{
  int i, deltaSec; //was int
  IntervalTimer dlyreqtimer;

  deltaSec = g_iElapsedMilliSeconds / 1000;

  if(deltaSec <= 0) return; //YOON 2017.01.03 for 802.1as.

  g_iElapsedMilliSeconds %= 1000; //0; -- YOON Which is more valid?

  for(i = 0; i < TIMER_ARRAY_SIZE; ++i) { //Restart Timers on Expired.
    if(itimer[i].interval > 0 && (itimer[i].left -= (deltaSec*1000)) <= 0)  {
      itimer[i].left = itimer[i].interval;
      itimer[i].expire = 1;//TRUE;
      //printf("timerUpdate: timer %u expired\r\n", i);
    }
  }
  //printf("%d",itimer[DELAYREQ_INTERVAL_TIMER].left);
}
#else
void ptpd_timerUpdate(IntervalTimer *itimer)
{
  int i, delta125mSec;
  IntervalTimer dlyreqtimer;

  delta125mSec = g_iElapsedMilliSeconds / 125;

  if(delta125mSec <= 0) return;

  g_iElapsedMilliSeconds %= 125;

  for(i = 0; i < TIMER_ARRAY_SIZE; ++i) { //Restart Timers on Expired.
	  itimer[i].left -= (delta125mSec*125);
      if(itimer[i].interval > 0 && (itimer[i].left <= 0))  {
    	  itimer[i].left = itimer[i].interval;
    	  itimer[i].expire = 1;//TRUE;
      }
  }
}
#endif
/** Start or restart a timer
 * @param timer the timer to start
 * @param millisec the value of the timer */
/*void sys_timer_start(sys_timer_t *timer, uint32_t millisec) {
	if (osTimerStart(timer->id, millisec) != osOK)
		printf("sys_timer_start error\n");
}
*/

void ptpd_timerStart(unsigned short index, unsigned short  intervalms, IntervalTimer *itimer)
{
  if(index >= TIMER_ARRAY_SIZE)   return;

  itimer[index].expire = 0;//FALSE;
  itimer[index].left = intervalms;
  itimer[index].interval = intervalms;
}
/*
void sys_timer_start(unsigned short index, float interval, TimeInternal * itimer)
{
	if (index >= TIMER_ARRAY_SIZE) return;

	itimer[index].expire = FALSE;

	// US_TIMER_INTERVAL defines the minimum interval between sigalarms.
	// * timerStart has a float parameter for the interval, which is casted to integer.
	// * very small amounts are forced to expire ASAP by setting the interval to 1

	itimer[index].left = (int)((interval * 1E6) / US_TIMER_INTERVAL);
	if(itimer[index].left == 0){
		// the interval is too small, raise it to 1 to make sure it expires ASAP
		// Timer cancelation is done explicitelly with stopTimer()
		itimer[index].left = 1;

		static int operator_warned_interval_too_small = 0;
		if(!operator_warned_interval_too_small){
			operator_warned_interval_too_small = 1;

			// * using random uniform timers it is pratically guarantted that we hit the possible minimum timer.
			// * This is because of the current timer model based on periodic sigalarm, irrespective if the next
			// * event is close or far away in time.
			// *
			// * A solution would be to recode this whole module with a calendar queue, while keeping the same API:
			// * E.g.: http://www.isi.edu/nsnam/ns/doc/node35.html
			// *
			// * Having events that expire immediatly (ie, delayreq invocations using random timers) can lead to
			// * messages appearing in unexpected ordering, so the protocol implementation must check more conditions
			// * and not assume a certain ususal ordering

			//DBG("Timer would be issued immediatly. Please raise dep/timer.c:US_TIMER_INTERVAL to hold %.2fs\n",	interval);
		}
	}
	itimer[index].interval = itimer[index].left;
	DBG("timerStart: Set timer %d to %f.  New interval: %d; new left: %d\n", index, interval, itimer[index].left , itimer[index].interval);
}
*/

/** Stop a timer
 * @param timer the timer to stop */
/*void sys_timer_stop(sys_timer_t *timer) {
	if (osTimerStop(timer->id) != osOK)
		printf("sys_timer_stop error\n");
}
*/
void ptpd_timerStop(unsigned short index, IntervalTimer *itimer)
{
  DBG("timerStop()\n");
  if(index >= TIMER_ARRAY_SIZE) return;
  itimer[index].interval = 0;
}

bool ptpd_timerExpired(unsigned short index, IntervalTimer *itimer)
{
	//ptpd_timerUpdate(itimer);

	if(index >= TIMER_ARRAY_SIZE)
		return 0;//FALSE;

	if(!itimer[index].expire)
		return 0;//FALSE;
	else{
		itimer[index].expire = 0;//FALSE;
		DBG("timerExpired(%d)\r\n",index);
		return 1;//TRUE;
	}
}

bool ptpd_timerRunning(unsigned short index, IntervalTimer * itimer)
{
	//ptpd_timerUpdate(itimer);

	if (index >= TIMER_ARRAY_SIZE)
		return FALSE;

	if ((itimer[index].interval != 0) &&
	    (itimer[index].expire == FALSE)) {
		return TRUE;
	//DBG2("timerRunning:   Timer %d is running\n", index);
	}

	return FALSE;

}

/** Delete a timer
 * @param timer the timer to delete */
//void sys_timer_free(sys_timer_t *timer) {}

#endif

