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
//#include "lwip/include/lwipopts.h"
/* ptpd.c */
#if (PROJ_FOR == PROJ_FOR_PTP)
#include "ptpstm/include/ptpd.h"

//#define PTPD_THREAD_PRIO    (tskIDLE_PRIORITY + 2)

//unsigned long g_ulSystemTimeTicks = 0;

__IO uint32_t g_ptpLocalTime_in_msec = 0; /* this variable is used to create a time reference incremented by 10ms */
// The local time for the lwIP Library Abstraction layer, used to support the
// Host and lwIP periodic callback functions.
unsigned long g_ulLocalTimer = 0;
// Statically allocated run-time configuration data.
RunTimeOpts rtOpts;
PtpClock ptpClock;
ForeignMasterRecord ptpForeignRecords[DEFAULT_MAX_FOREIGN_RECORDS];

__IO uint32_t PTPTimer = 0;
extern bool ptp_doInit(PtpClock *ptpClock);

//*****************************************************************************
// The interrupt handler for the SysTick interrupt.
// [This is a main scheduler or event handler]
//*****************************************************************************
//Called each 1 msec (=1/SYSTICKHZ)
//*****************************************************************************
//
//! Handles periodic timer events for the lwIP TCP/IP stack.
//!
//! \param ulTimeMS is the incremental time for this periodic interrupt.
//!
//! This function will update the local timer by the value in \e ulTimeMS.
//! If the system is configured for use without an RTOS, an Ethernet interrupt
//! will be triggered to allow the lwIP periodic timers to be serviced in the
//! Ethernet interrupt.
//!
//! \return None.
//
//*****************************************************************************
//[NOTE: VERY IMPORTANT : This ft. is called at each tick by SysTickIntHandler_PTP().
// And, it fakes Ethernet Interrupts(The lwIPEthernetIntHandler() ISR will be called)
/*void lwIPTimer(unsigned long ulTimeMS)
{
    //
    // Increment the lwIP Ethernet timer.
    //
    g_ulLocalTimer += ulTimeMS;
    //UARTprintf("T=%d\r\n",g_ulLocalTimer);

    // Generate a Faked Ethernet interrupt.  This will perform the actual work
    // of checking the lwIP timers and taking the appropriate actions.  This is
    // needed since lwIP is not re-entrant, and this allows all lwIP calls to
    // be placed inside the Ethernet interrupt handler ensuring that all calls
    // into lwIP are coming from the same context, preventing any reentrancy
    // issues.  Putting all the lwIP calls in the Ethernet interrupt handler
    // avoids the use of mutexes to avoid re-entering lwIP.
    //
    //YOON
    HWREG(NVIC_SW_TRIG) |= INT_ETH - 16;
}
*/
extern int g_cnt;



#ifdef USE_TARGETTIME
extern bool g_TargetTimeReached;
void ptp_TagetTimeInit(){
	TimeInternal tmpTime;
	getTimeFromReg(&tmpTime);
	ETH_SetPTPTargetTime(tmpTime.seconds,tmpTime.nanoseconds);
	ETH_MACITConfig(ETH_MAC_IT_TST, ENABLE);
	ETH_EnablePTPTimeStampInterruptTrigger();
}
#endif

#ifdef USE_SYNCLED
//=== PE6.
void PE6_SYNCLED_init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE ,  ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOE, GPIO_Pin_6);
}
//PE6 SYNC LED :
void PTP_SyncLed(unsigned char on_off){
	if(on_off)
	    GPIO_SetBits(GPIOE, GPIO_Pin_6);
	else{
	    GPIO_ToggleBits(GPIOE, GPIO_Pin_6);
	}
}
/*
//=== PE5 is now used for CAM_D6. Thus the below is useless.
void PE5_ULED0_init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE ,  ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOE, GPIO_Pin_5);
    GPIO_SetBits(GPIOE, GPIO_Pin_5);
    delayms(100);
    GPIO_ResetBits(GPIOE, GPIO_Pin_5);
}
//PE5 PPS LED :
//TBD.....May use CAN TX LED on the front.
//PPS from MAC : PB5, Pin3. --> to PPS LED
void PTP_SyncLed(unsigned char on_off){
	if(on_off)
	    GPIO_SetBits(GPIOE, GPIO_Pin_5);
	else{
	    GPIO_ToggleBits(GPIOE, GPIO_Pin_5);
	}
}
*/
#endif

void displayCurrent(){
	static TimeInternal tmpTime;
	tTod tod;
	char str[16];
	unsigned short ms,us,ns;

	getTimeFromReg(&tmpTime); //Not Working?
    ulocaltime(tmpTime.seconds, &tod);

	//sprintf(str,"%04d/%02d/%02d %02d:%02d:%02d.%09d",tod.usYear, tod.ucMon, tod.ucMday, tod.ucHour, tod.ucMin, tod.ucSec, nanosec);
	sprintf(str,"%04d/%02d/%02d",tod.usYear, tod.ucMon, tod.ucMday);
	printf("%s\r\n",str);
	stmOzOLED_printString(str,0,5,16);
	sprintf(str,"%02d:%02d:%02d",tod.ucHour, tod.ucMin, tod.ucSec);
	printf("%s\r\n",str);
	stmOzOLED_printString(str,0,6,16);

	tmpTime.nanoseconds = tmpTime.nanoseconds;//g_cnt;
	ms = tmpTime.nanoseconds / 1000000;
	us = tmpTime.nanoseconds / 1000;	us = us % 1000;
	ns = tmpTime.nanoseconds % 1000;
	//sprintf(str,"%03d.%03d.%03d",ms,us,ns);
	//stmOzOLED_printString(str,0,7,16);
	//printf("%s.%d\r\n",str,tmpTime.nanoseconds);

	 //Show current TOD on 7 segments.
#ifdef USE_MAX7219
	 PTP_max7219_write16digits(
			tod.ucHour, tod.ucMin, tod.ucSec,
			tmpTime.nanoseconds
	 );
#endif
}

void SysTickIntHandler_PTP(void)//SysTick_IRQHandler(void)
{

    // Update internal time and set PPS output, if needed.
	g_ptpLocalTime_in_msec += SYSTICKMS; //SYSTEMTICK_PERIOD_MS;

	//g_ulSystemTime_in_nsec += SYSTICKNS; //1000,000,000 / SYSTICKHZ(=1000) ==> 1000,000.
    //max7219_writeTimeOn7Segment(g_ulSystemTimeSeconds);
	// Increments various PTPd Timers.
    ptpd_timerTick(SYSTICKMS); //PTPÃƒÆ’Ã‚Â¬Ãƒâ€šÃ‚Â ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾ÃƒÆ’Ã‚Â¬Ãƒâ€¦Ã‚Â¡Ãƒâ€šÃ‚Â© -- iElapsedMilliSeconds += 1;
    // Increment the run-time tick counter.
    //g_ulSystemTimeTicks++;

    //void displayStats(const PtpClock *ptpClock) displayCurrent(&ptpClock); //TEST ONLY

    //Indicate that a SysTick INT has detected.  -- This flag will be handled in Main Loop
//    HWREGBITW(&g_ulFlags, FLAG_SYSTICK) = 1;
	 // call the main lwIP timer handler.
    //lwIPTimer(SYSTICKMS); //ÃƒÆ’Ã‚Â¬ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬ï¿½Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã‚ÂªÃƒâ€šÃ‚Â¸Ãƒâ€šÃ‚Â°ÃƒÆ’Ã‚Â¬ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾Ãƒâ€¦Ã¢â‚¬Å“ g_ulLocalTimer += ulTimeMS(=1msec) ÃƒÆ’Ã‚Â¬ÃƒÂ¯Ã‚Â¿Ã‚Â½ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾ ÃƒÆ’Ã‚Â¬Ãƒâ€¹Ã¢â‚¬Â Ãƒâ€¹Ã…â€œÃƒÆ’Ã‚Â­ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â°ÃƒÆ’Ã‚Â­ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢Ãƒâ€¹Ã…â€œÃƒÆ’Ã‚ÂªÃƒâ€šÃ‚Â³Ãƒâ€šÃ‚Â  ÃƒÆ’Ã‚Â¬ÃƒÂ¯Ã‚Â¿Ã‚Â½Ãƒâ€šÃ‚Â´ÃƒÆ’Ã‚Â«ÃƒÂ¯Ã‚Â¿Ã‚Â½ÃƒÂ¢Ã¢â€šÂ¬Ã¯Â¿Â½ÃƒÆ’Ã‚Â«ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾Ãƒâ€šÃ‚Â· ÃƒÆ’Ã‚Â¬ÃƒÂ¯Ã‚Â¿Ã‚Â½Ãƒâ€šÃ‚Â¸ÃƒÆ’Ã‚Â­ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾Ãƒâ€šÃ‚Â°ÃƒÆ’Ã‚Â«Ãƒâ€¦Ã‚Â¸Ãƒâ€šÃ‚Â½ÃƒÆ’Ã‚Â­ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾Ãƒâ€šÃ‚Â°ÃƒÆ’Ã‚Â«ÃƒÂ¯Ã‚Â¿Ã‚Â½ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾ ÃƒÆ’Ã‚ÂªÃƒâ€šÃ‚Â°ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÆ’Ã‚Â¬Ãƒâ€šÃ‚Â§Ãƒâ€¦Ã¢â‚¬Å“ÃƒÆ’Ã‚Â«Ãƒâ€šÃ‚Â¡Ãƒâ€¦Ã¢â‚¬Å“ ÃƒÆ’Ã‚Â«Ãƒâ€šÃ‚Â°Ãƒâ€¦Ã¢â‚¬Å“ÃƒÆ’Ã‚Â¬Ãƒâ€ Ã¢â‚¬â„¢ÃƒÂ¯Ã‚Â¿Ã‚Â½ÃƒÆ’Ã‚Â¬ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹Ãƒâ€¦Ã¢â‚¬Å“ÃƒÆ’Ã‚Â¬Ãƒâ€šÃ‚Â¼Ãƒâ€¦Ã¢â‚¬Å“ lwIPHostTimerHandler()ÃƒÆ’Ã‚Â«Ãƒâ€šÃ‚Â¥Ãƒâ€šÃ‚Â¼ ÃƒÆ’Ã‚Â­Ãƒâ€¹Ã…â€œÃƒâ€šÃ‚Â¸ÃƒÆ’Ã‚Â¬Ãƒâ€šÃ‚Â¶Ãƒâ€¦Ã¢â‚¬Å“ÃƒÆ’Ã‚Â­ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢Ãƒâ€¹Ã…â€œÃƒÆ’Ã‚ÂªÃƒâ€šÃ‚Â³Ãƒâ€šÃ‚Â , ÃƒÆ’Ã‚ÂªÃƒâ€šÃ‚Â±Ãƒâ€šÃ‚Â°ÃƒÆ’Ã‚ÂªÃƒâ€šÃ‚Â¸Ãƒâ€šÃ‚Â°ÃƒÆ’Ã‚Â¬ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾Ãƒâ€¦Ã¢â‚¬Å“ ÃƒÆ’Ã‚Â¬Ãƒâ€¦Ã‚Â Ãƒâ€šÃ‚Â¤ÃƒÆ’Ã‚Â¬Ãƒâ€šÃ‚Â¼ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÆ’Ã‚Â«ÃƒÂ¢Ã¢â€šÂ¬Ã‹Å“Ãƒâ€¹Ã…â€œÃƒÆ’Ã‚Â«Ãƒâ€šÃ‚Â§ÃƒÂ¯Ã‚Â¿Ã‚Â½ÃƒÆ’Ã‚Â¬ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬ï¿½ÃƒÂ¯Ã‚Â¿Ã‚Â½ ÃƒÆ’Ã‚Â­ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢Ãƒâ€¦Ã¢â‚¬Å“ÃƒÆ’Ã‚Â¬Ãƒâ€¦Ã‚Â¡Ãƒâ€šÃ‚Â©ÃƒÆ’Ã‚Â­ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢Ãƒâ€¦Ã¢â‚¬Å“ÃƒÆ’Ã‚Â«ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¹Ãƒâ€šÃ‚Â¤. YOON

}
extern void ptpd_timerUpdate(IntervalTimer *itimer);

bool g_TargetTimeInit = 0;

// Notify the PTP thread of a pending operation.
void ptpd_alert(void)
{
	// Send a message to the alert queue to wake up the PTP thread.
	//sys_mbox_trypost(&ptp_alert_queue, NULL);
}
int16_t ptpdStartup(PtpClock * ptpClock, RunTimeOpts *rtOpts, ForeignMasterRecord* foreign)
{
	ptpClock->rtOpts = rtOpts;
	ptpClock->foreignMasterDS.foreign = foreign;
	ptpClock->pastStartup = FALSE; //YOON added
	rtOpts->disableBMCA = FALSE;

	/* 9.2.2 */
	if (rtOpts->slaveOnly)
		rtOpts->clockQuality.clockClass = DEFAULT_CLOCK_CLASS_SLAVE_ONLY;

	/* No negative or zero attenuation */
	if (rtOpts->servo.ap < 1) rtOpts->servo.ap = 1;
	if (rtOpts->servo.ai < 1) rtOpts->servo.ai = 1;

	printf("START UP... to PTP_INITIALIZING\n");

	stm_ptpd_toState(ptpClock, PTP_INITIALIZING);

	return 0;
}

void displayStatus(PtpClock *ptpClock, const char *prefixMessage)
{

	//static char sbuf[SCREEN_BUFSZ];
	//char strAddr[MAXHOSTNAMELEN];
	int len = 0;

	printf("%s(State=%d)\r\n", prefixMessage,ptpClock->portDS.portState );
/*
	memset(strAddr, 0, sizeof(strAddr));
	memset(sbuf, ' ', sizeof(sbuf));
	len += snprintf(sbuf + len, sizeof(sbuf) - len, "%s", prefixMessage);
	len += snprintf(sbuf + len, sizeof(sbuf) - len, "%s",
			portState_getName(ptpClock->portDS.portState));

	if (ptpClock->portDS.portState >= PTP_MASTER) {
		len += snprintf(sbuf + len, sizeof(sbuf) - len, ", Best master: ");
		len += snprint_PortIdentity(sbuf + len, sizeof(sbuf) - len,
			&ptpClock->parentDS.parentPortIdentity);
		if(ptpClock->foreignMasterDS.bestMaster && ptpClock->foreignMasterDS.bestMaster->sourceAddr) {
		    struct in_addr tmpAddr;
		    tmpAddr.s_addr = ptpClock->foreignMasterDS.bestMaster->sourceAddr;
		    inet_ntop(AF_INET, (struct sockaddr_in *)(&tmpAddr), strAddr, MAXHOSTNAMELEN);
		    len += snprintf(sbuf + len, sizeof(sbuf) - len, " (IPv4:%s)",strAddr);
		}
        }
	if(ptpClock->portDS.portState == PTP_MASTER)
    		len += snprintf(sbuf + len, sizeof(sbuf) - len, " (self)");
        len += snprintf(sbuf + len, sizeof(sbuf) - len, "\n");
        NOTICE("%s",sbuf);
*/
}

#if(PTP_PROTOCOL == IEEE1588V2)
void stm_1588_configRunTimeOptions(){

	rtOpts.delayMechanism = E2E;//DEFAULT_DELAY_MECHANISM; //E2E or P2P

	rtOpts.slaveOnly = 0;//False

	//=== SET DEFAULT CONFIGURATION ====
	//PROTOCOL
	rtOpts.announceInterval           	= DEFAULT_ANNOUNCE_INTERVAL; //1 --> 2sec
	rtOpts.syncInterval 				= DEFAULT_SYNC_INTERVAL; //0 --> 1sec
	rtOpts.logMinPdelayReqInterval 		= DEFAULT_PDELAYREQ_INTERVAL;//1 --> 2sec
	//rtOpts.initial_delayreq 			= DEFAULT_DELAYREQ_INTERVAL; //1 --> 2sec
	rtOpts.logMinDelayReqInterval 		= DEFAULT_DELAYREQ_INTERVAL; //1 --> 2sec
	//rtOpts.autoDelayReqInterval 		= TRUE;
	//rtOpts.masterRefreshInterval 		= 60;
	//rtOpts.syncSequenceChecking 		= FALSE;// by default we don't check Sync message sequence continuity
	rtOpts.announceReceiptTimeout  		= DEFAULT_ANNOUNCE_RECEIPT_TIMEOUT;//3 -> 8sec

	// maximum values for unicast negotiation
    rtOpts.logMaxPdelayReqInterval 		= 5;
	rtOpts.logMaxDelayReqInterval 		= 5;
	rtOpts.logMaxSyncInterval 			= 5;
	rtOpts.logMaxAnnounceInterval 		= 5;

	//Network -- UDP/MULTICAST
	rtOpts.portNumber 			= 1;// NUMBER_PORTS;
	rtOpts.portDisabled	 		= FALSE; //Enable
	rtOpts.transport 			= UDP_IPV4;
	rtOpts.ipMode 				= IPMODE_MULTICAST;
	rtOpts.dot1AS 				= FALSE;
	rtOpts.disableUdpChecksums 	= TRUE;
	rtOpts.inboundLatency.nanoseconds  = 700;//DEFAULT_INBOUND_LATENCY; //7700ns
	rtOpts.outboundLatency.nanoseconds = 700;//DEFAULT_OUTBOUND_LATENCY;//7700ns
	rtOpts.ofmShift.seconds 	= 0;
	rtOpts.ofmShift.nanoseconds = (10); //YOON.- IF YOU NEED SOME OFFSET AS A SLAVE, SET IT in NSEC.

	//Servo
	rtOpts.servo.noAdjust 		= FALSE; //rtOpts.servo.noAdjust = NO_ADJUST; //FALSE --> DO ADJUST.
	rtOpts.servo.noResetClock 	= FALSE;//DEFAULT_NO_RESET_CLOCK; ////FALSE --> CAN RESET.
	rtOpts.servo.sDelay 		= DEFAULT_DELAY_S; //6
	rtOpts.servo.sOffset 		= DEFAULT_OFFSET_S; //1
	rtOpts.servo.ap 			= DEFAULT_AP; //2
	rtOpts.servo.ai 			= DEFAULT_AI; //16
	rtOpts.servo.servoMaxPpb 	= ADJ_FREQ_MAX / 1000;// ADJ_FREQ_MAX by default
	rtOpts.servo.stepOnce	 	= FALSE;
	rtOpts.servo.stepForce	 	= FALSE;
	rtOpts.servo.noResetClock   = DEFAULT_NO_RESET_CLOCK; //"Do not step the clock - only slew"

	rtOpts.clockUpdateTimeout = 0;//If set to non-zero, timeout in seconds, after which the slave resets if no clock updates made.

	//BMC
	rtOpts.anyDomain 					= FALSE;
	rtOpts.clockQuality.clockAccuracy 	= DEFAULT_CLOCK_ACCURACY;
#if (ETHIF_ROLE == PTP_GRANDMASTERCLOCK)
	rtOpts.clockQuality.clockClass 		= DEFAULT_CLOCK_CLASS-1;
#else
	rtOpts.clockQuality.clockClass 		= DEFAULT_CLOCK_CLASS+1;
#endif
	rtOpts.clockQuality.offsetScaledLogVariance = DEFAULT_CLOCK_VARIANCE;//==5000 /* 7.6.3.3 */ //???? YOON //
	rtOpts.priority1 					= DEFAULT_PRIORITY1; //248
	rtOpts.priority2 					= DEFAULT_PRIORITY2; //248
	rtOpts.domainNumber 				= DEFAULT_DOMAIN_NUMBER; //==0
	rtOpts.maxForeignRecords = sizeof(ptpForeignRecords) / sizeof(ptpForeignRecords[0]);

	// timePropertiesDS
	rtOpts.timeProperties.currentUtcOffsetValid = DEFAULT_UTC_VALID;
	rtOpts.timeProperties.currentUtcOffset = DEFAULT_UTC_OFFSET;//rtOpts.currentUtcOffset = DEFAULT_UTC_OFFSET;
	rtOpts.timeProperties.timeSource 	= INTERNAL_OSCILLATOR;
	rtOpts.timeProperties.timeTraceable = FALSE;
	rtOpts.timeProperties.frequencyTraceable = FALSE;
	rtOpts.timeProperties.ptpTimescale 	= TRUE;

	// currentUtcOffsetValid compatibility flags
	rtOpts.alwaysRespectUtcOffset 		= TRUE;
	rtOpts.preferUtcValid 				= FALSE;
	rtOpts.requireUtcValid 				= FALSE;

	// panic mode options
	rtOpts.enablePanicMode 				= FALSE;
	rtOpts.panicModeDuration 			= 2;
	rtOpts.panicModeExitThreshold 		= 0;

	///
	rtOpts.maxDelay 					= 0; //Do accept master to slave delay (Tms_offset - from Sync message) or slave to master delay (delaySM - from Delay messages)
	//if greater than this value (nanoseconds).
	//0 = not used.", RANGECHECK_RANGE,0,NANOSECONDS_MAX);

	rtOpts.maxDelayMaxRejected 			= 0; //Maximum number of consecutive delay measurements exceeding maxDelay threshold. 0= Not used.
	//rtOpts.stats = PTP_TEXT_STATS;
}
extern void Lan9355_UsrButtonHandler();

//static void ieee1588ptpd_thread(TimeInternal *pcurrentTod)
void ieee1588ptpd_thread(TimeInternal *pcurrentTod)
{
	volatile uint32_t res;
	struct ptptime_t curTime;


	stmOzOLED_printString("Init Config..",0,2,10);

	stm_1588_configRunTimeOptions();

	// Initialize run time options. State becomes PTP_INITIALIZING.
	if (ptpdStartup(&ptpClock, &rtOpts, ptpForeignRecords) != 0){
		printf("PTPD: startup failed");
		return;
	}

	////bring up network interface and ip/udp stack. -- YOON 2016.10.12
	if (!netInit(&ptpClock.netPath, &ptpClock)){
		printf("ERROR: ieee1588ptpd_thread: failed to initialize network\r\n");
		return;
	}
#ifdef USE_DHCP
	// If DHCP, wait until the default interface has an IP address.
	while (!netif_default->ip_addr.addr)
	{
		// Sleep for 500 milliseconds.
		sys_msleep(500);
	}
#endif

	//Start Software Timers for PTP protocols.
	ptpd_timerStart(YOON_TIMER,1000,ptpClock.itimer);	//YOON

#ifdef USE_TARGETTIME

#endif

	// Loop forever.
	while(1){

#ifdef USE_TARGETTIME

		if(ptpClock.portDS.portState == PTP_SLAVE){
			if(g_TargetTimeInit == 0){
				ptp_TagetTimeInit();
				g_TargetTimeInit = 1;
			}else{
				if(g_TargetTimeReached){
					res = ETH->PTPTSSR;//For clearing, we must read Status (ETH_PTPTSSR) Register)
					printf("\r\nTick \r\n");
					g_TargetTimeReached = 0;
					ETH_PTPTime_GetTime(&curTime);
					ETH_SetPTPTargetTime(curTime.tv_sec + 2 ,0); //Next Interrupt.
				}
			}
		}
#endif
		// Process the current state.
		if(ptpClock.disabled && (ptpClock.portDS.portState != PTP_DISABLED)) {
			stm_ptpd_toState(&ptpClock, PTP_DISABLED);
		}

		if (ptpClock.portDS.portState == PTP_INITIALIZING) {
			if(!ptp_doInit(&ptpClock)) { //after some init and transition to PTP_LISTENING state.
				stmOzOLED_printString("PTPd initFail",0,2,15);
				printf("PTPd init failed\r\n");// - will retry in %d seconds\n", DEFAULT_FAILURE_WAITTIME);
			}
		} else {
			stm_ptpd_doState(&ptpClock);
		}

		//accessory procedures
		ptpd_timerUpdate(ptpClock.itimer); //YOON Added

		LwIP_Periodic_Handle(g_ptpLocalTime_in_msec);		 // handle periodic timers for LwIP

		//BroadR_Periodic_Handle(g_ptpLocalTime_in_msec); //check link status -- Only For BroadR-Reach
		if (ptpd_timerExpired(YOON_TIMER,ptpClock.itimer)){
			displayStats(&ptpClock);
			ptpd_timerStart(YOON_TIMER,1000,ptpClock.itimer);	//YOON
		}
	}
}


/*
void ptpd_init_and_loop(TimeInternal *pcurrentTod)
{

	// ============= Set Current Time of Day (2015.12.19. 2:20:35 GMT) ============================
	//pcurrentTod->seconds = 1450491635;
	//pcurrentTod->nanoseconds = 681000000;
	//Set GPS TIME to STM32F407 MAC's PTP Registers.
	//setTimeToReg(pcurrentTod); //call ETH_PTPTime_SetTime()
	// Create the PTP daemon thread.
	ieee1588ptpd_thread(pcurrentTod);//sys_thread_new("PTPD", ieee1588ptpd_thread, NULL, DEFAULT_THREAD_STACKSIZE * 2, osPriorityAboveNormal);
}
*/

//======= startup.c ==============
void ptpdShutdown(PtpClock *ptpClock)
{
	netShutdown(&ptpClock->netPath);
}
#endif //(PTP_PROTOCOL == IEEE1588V2)

void stm_ptp_mainloop()
{

#if (USE_DISPLAY == USE_DISPLAY_OLED)
	stmOzOLED_init(">IEEE1588/802.1AS<");
#endif
//	cs2300Config();

#ifdef USE_MAX7219
	stmMax7219_16digit_Config();//PTP_config_stmMax7219_16digit_display();
#endif

#if (USE_SYNCLED == 1) && (PROJ_FOR == PROJ_FOR_PTP)
	PE6_SYNCLED_init();
#endif

    /***************************************************************************
    NOTE: When using Systick to manage the delay in Ethernet driver, the Systick
         must be configured before Ethernet initialization and, the interrupt
         priority should be the highest one.
  *****************************************************************************/
    Init_SysTick(1000);//1msec for ptpd state machine
	//SysTick_ITConfig(ENABLE); //SysTick Interrupt Enable

#if (ETHIF_ROLE == PTP_GRANDMASTERCLOCK)

    yEther_PTP_1588_8021AS_Loop("GMC"); //IP addr= lwipmain.h; MAC addr=

#elif (ETHIF_ROLE == PTP_OC)
	#if(SWITCH_ID == SWITCH_ID_KSZ8794)
        stmOzOLED_printString("TC-KSZ8794",0,0,15); ////stmOzOLED_init("PTP KSZ8794"); //
		Ksz8794_init();
		stm_Lan9355_ipConfig(); ///
		yEther_PTP_1588_8021AS_Loop("TC-KSZ9355"); //IP addr= lwipmain.h; MAC addr=
	#elif(SWITCH_ID == SWITCH_ID_LAN9355)
		stmOzOLED_printString("TC-LAN9355",0,0,15); //stmOzOLED_init("PTP LAN9355"); //
		Lan9355_init();
		Lan9355_Config();
		stm_Lan9355_ipConfig();
		yEther_PTP_1588_8021AS_Loop("TC-LAN9355");
	#else
		yEther_PTP_1588_8021AS_Loop("OrdClock");
	#endif
//#elif (ETHIF_ROLE == SIMPLE_ETH)
//	ETH_SimpleIP101Loop();
#endif
}
//==== ADDED =====
#endif
