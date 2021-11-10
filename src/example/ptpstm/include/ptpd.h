//-- STM PTP

/**
 * \author van Kempen Alexandre
 * \mainpage PTPd v2 Documentation
 * \version 2.0.1
 * \date 17 nov 2010
 * \section implementation Implementation
 * PTPd is full implementation of IEEE 1588 - 2008 standard of ordinary clock.
*/



/**
*\file
* \brief Main functions used in ptpdv2
*
* This header file includes all others headers.
* It defines functions which are not dependant of the operating system.
 */

#ifndef PTPD_H_
#define PTPD_H_

//#define PTPD_DBGVV
//#define PTPD_DBGV
//#define PTPD_DBG
#define PTPD_ERR

#include "yInc.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stm32f10x.h>
#include <stm32f107_eth.h>
#include <limits.h>

#if USE_LWIP

#include "lwip/include/opt.h"
#include "lwip/include/api.h"
//#include "lwip/include/inet.h"
#include "lwip/include/mem.h"
#include "lwip/include/udp.h"
#include "lwip/include/igmp.h"
#include "lwip/include/raw.h"
#include "lwip/include/arch.h"
//#include "lwip/include/timers.h"
#include "ethernetif.h"

#include "constants.h"
//#include "lwipopts.h" //override the constant.h
//#include "dep/constants_dep.h"
//#include "dep/datatypes_dep.h"
#include "datatypes.h"

//#include "dep/ptpd_dep.h"
//#define PTPD_DBGVV 1


//=========== PTP Related =================================
//#define CLOCKSOURCE_GPS 1
//#define CLOCKSOURCE_LOC 2
//#define CLOCKSOURCE CLOCKSOURCE_LOC //CLOCKSOURCE_GPS//CLOCKSOURCE_LOC //CLOCKSOURCE_GPS

//#if (CLOCKSOURCE == CLOCKSOURCE_GPS)
//#define USE_GPS
//#else
//#undef USE_GPS
//#endif

//ROLEs
//#define PTP_GRANDMASTERCLOCK 1
//#define PTP_OC 	 			 2
//#define SIMPLE_ETH_PHY     	6 //FOR EDUCATIONAL PURPOSE ONLY

//#define PTP_ROLE 			SIMPLE_ETH_PHY//PTP_OC // SIMPLE_ETH_PHY////PTP_GRANDMASTERCLOCK //SIMPLE_ETH_IP101 //PTP_TC_9355 //PTP_GRANDMASTERCLOCK //SHOULD BE SET

//==================== (2) PTP =======================================
//#define LWIP_PTP		0 //1
//#define USE_DHCP       /* enable DHCP, if disabled static address is used */
//#define IEEE8021AS      0
//#define IEEE1588V2      1

//#define PTP_PROTOCOL  IEEE8021AS //IEEE1588V2

#define DEFAULT_DELAY_MECHANISM P2P//E2E//P2P = 802.1as -- predefined  in constants.h
//Also set in constants.h #define DEFAULT_DELAY_MECHANISM         E2E//P2P = 802.1as
//#endif

#define USE_TARGETTIME  1
#undef USE_TARGETTIME

#define YPTP_PPS_FREQ 10 //freq = 2**10 = 1024Hz. Default = 0(=1Hz)
//=================================================ptpd_dep.h ==========

/** \name Debug messages */
/**\{*/
#ifdef PTPD_DBGVV
#define PTPD_DBGV
#define PTPD_DBG
#define PTPD_ERR
#define DBGVV(...) printf("(V) " __VA_ARGS__)
#else
#define DBGVV(...)
#endif

#ifdef PTPD_DBGV
#define PTPD_DBG
#define PTPD_ERR
#define DBGV(...)  { TimeInternal tmpTime; getTimeFromReg(&tmpTime); printf("(d %d.%09d) ", tmpTime.seconds, tmpTime.nanoseconds); printf(__VA_ARGS__); }
#else
#define DBGV(...)
#endif

#ifdef PTPD_DBG
#define PTPD_ERR
#define DBG(...)  { TimeInternal tmpTime; getTimeFromReg(&tmpTime); printf("(D %d.%09d) ", tmpTime.seconds, tmpTime.nanoseconds); printf(__VA_ARGS__); }
#else
#define DBG(...)
#endif
/** \}*/

/** \name System messages */
/**\{*/
#ifdef PTPD_ERR
#define ERROR(...)  { TimeInternal tmpTime; getTimeFromReg(&tmpTime); printf("(E %d.%09d) ", tmpTime.seconds, tmpTime.nanoseconds); printf(__VA_ARGS__); }
/* #define ERROR(...)  { printf("(E) "); printf(__VA_ARGS__); } */
#else
#define ERROR(...)
#endif
/** \}*/

/** \name Endian corrections */
/**\{*/

#if defined(PTPD_MSBF)
#define shift8(x,y)   ( (x) << ((3-y)<<3) )
#define shift16(x,y)  ( (x) << ((1-y)<<4) )
#elif defined(PTPD_LSBF)
#define shift8(x,y)   ( (x) << ((y)<<3) )
#define shift16(x,y)  ( (x) << ((y)<<4) )
#endif

#define flip16(x) htons(x)
#define flip32(x) htonl(x)

/* i don't know any target platforms that do not have htons and htonl,
	 but here are generic funtions just in case */
/*
#if defined(PTPD_MSBF)
#define flip16(x) (x)
#define flip32(x) (x)
#elif defined(PTPD_LSBF)
static inline int16_t flip16(int16_t x)
{
	 return (((x) >> 8) & 0x00ff) | (((x) << 8) & 0xff00);
}

static inline int flip32(x)
{
	return (((x) >> 24) & 0x000000ff) | (((x) >> 8 ) & 0x0000ff00) |
				 (((x) << 8 ) & 0x00ff0000) | (((x) << 24) & 0xff000000);
}
#endif
*/

/** \}*/


/** \name Bit array manipulations */
/**\{*/
#define getFlag(flagField, mask) (bool)(((flagField)  & (mask)) == (mask))
#define setFlag(flagField, mask) (flagField) |= (mask)
#define clearFlag(flagField, mask) (flagField) &= ~(mask)
/* #define getFlag(x,y)  (bool)!!(  *(uint8_t*)((x)+((y)<8?1:0)) &   (1<<((y)<8?(y):(y)-8)) ) */
/* #define setFlag(x,y)    ( *(uint8_t*)((x)+((y)<8?1:0)) |=   1<<((y)<8?(y):(y)-8)  ) */
/* #define clearFlag(x,y)  ( *(uint8_t*)((x)+((y)<8?1:0)) &= ~(1<<((y)<8?(y):(y)-8)) ) */
/** \}*/

/** \name msg.c
 *-Pack and unpack PTP messages */
/**\{*/

void msgUnpackHeader(const octet_t*, MsgHeader*);
void msgUnpackAnnounce(const octet_t*, MsgAnnounce*);
void msgUnpackSync(const octet_t*, MsgSync*);
void msgUnpackFollowUp(const octet_t*, MsgFollowUp*);
void msgUnpackDelayReq(const octet_t*, MsgDelayReq*);
void msgUnpackDelayResp(const octet_t*, MsgDelayResp*);
void msgUnpackPDelayReq(const octet_t*, MsgPDelayReq*);
void msgUnpackPDelayResp(const octet_t*, MsgPDelayResp*);
void msgUnpackPDelayRespFollowUp(const octet_t*, MsgPDelayRespFollowUp*);
void msgUnpackManagement(const octet_t*, MsgManagement*);
void msgUnpackManagementPayload(const octet_t *buf, MsgManagement *manage);
void msgPackHeader(const PtpClock*, octet_t*);
void msgPackAnnounce(const PtpClock*, octet_t*);
void msgPackSync(const PtpClock*, octet_t*, const Timestamp*);
void msgPackFollowUp(const PtpClock*, octet_t*, const Timestamp*);
void msgPackDelayReq(const PtpClock*, octet_t*, const Timestamp*);
void msgPackDelayResp(const PtpClock*, octet_t*, const MsgHeader*, const Timestamp*);
void msgPackPDelayReq(const PtpClock*, octet_t*, const Timestamp*);
void msgPackPDelayResp(const PtpClock*,octet_t*, const MsgHeader*, const Timestamp*);
void msgPackPDelayRespFollowUp(const PtpClock*,octet_t*, const MsgHeader*, const Timestamp*);
int16_t msgPackManagement(const PtpClock*,  octet_t*, const MsgManagement*);
int16_t msgPackManagementResponse(const PtpClock*,  octet_t*, MsgHeader*, const MsgManagement*);
/** \}*/

/** \name net.c (Linux API dependent)
 * -Init network stuff, send and receive datas */
/**\{*/

bool  netInit(NetPath*, PtpClock*);
bool  netShutdown(NetPath*);
int netSelect(NetPath*, const TimeInternal*);
ssize_t netRecvEvent(NetPath*, octet_t*, TimeInternal*);
ssize_t netRecvGeneral(NetPath*, octet_t*, TimeInternal*);
ssize_t netSendEvent(NetPath*, const octet_t*, int16_t, TimeInternal*);
ssize_t netSendGeneral(NetPath*, const octet_t*, int16_t);
ssize_t netSendPeerGeneral(NetPath*, const octet_t*, int16_t);
ssize_t netSendPeerEvent(NetPath*, const octet_t*, int16_t, TimeInternal*);
void netEmptyEventQ(NetPath *netPath);
/** \}*/

/** \name servo.c
 * -Clock servo */
/**\{*/

void initClock(PtpClock*);
void updatePeerDelay(PtpClock*, const TimeInternal*, bool);
void updateDelay(PtpClock*, const TimeInternal*, const TimeInternal*, const TimeInternal*);
void updateOffset(PtpClock *, const TimeInternal*, const TimeInternal*, const TimeInternal*);
void updateClock(PtpClock*);
/** \}*/

/** \name startup.c (Linux API dependent)
 * -Handle with runtime options */
/**\{*/
int16_t ptpdStartup(PtpClock*, RunTimeOpts*, ForeignMasterRecord*);
void ptpdShutdown(PtpClock *);
/** \}*/

/** \name sys.c (Linux API dependent)
 * -Manage timing system API */
/**\{*/
void displayStats(const PtpClock *ptpClock);
bool nanoSleep(const TimeInternal*);
void getTimeFromReg(TimeInternal*);
void setTimeToReg(const TimeInternal*);
void updateTimeWithOffsetReg(const TimeInternal*); //Only used for coarse mode.
bool adjFreq(int32_t);
unsigned int getRand(uint32_t);
/** \}*/

/** \name timer.c (Linux API dependent)
 * -Handle with timers */
/**\{*/
void initTimer(void);
void timerStop(int32_t);
void timerStart(int32_t,  uint32_t);
bool timerExpired(int32_t);
/** \}*/


/* Test functions */


//===========
/** \name arith.c
 * -Timing management and arithmetic */
/**\{*/
/* arith.c */

/**
 * \brief Convert scaled nanoseconds into TimeInternal structure
 */
void scaledNanosecondsToInternalTime(const int64_t*, TimeInternal*);
/**
 * \brief Convert TimeInternal into Timestamp structure (defined by the spec)
 */
void fromInternalTime(const TimeInternal*, Timestamp*);

/**
 * \brief Convert Timestamp to TimeInternal structure (defined by the spec)
 */
void toInternalTime(TimeInternal*, const Timestamp*);

/**
 * \brief Add two TimeInternal structure and normalize
 */
void addTime(TimeInternal*, const TimeInternal*, const TimeInternal*);

/**
 * \brief Substract two TimeInternal structure and normalize
 */
void subTime(TimeInternal*, const TimeInternal*, const TimeInternal*);

/**
 * \brief Divide the TimeInternal by 2 and normalize
 */
void div2Time(TimeInternal*);

/**
 * \brief Returns the floor form of binary logarithm for a 32 bit integer.
 * -1 is returned if ''n'' is 0.
 */
int floorLog2(unsigned int);

/**
 * \brief return maximum of two numbers
 */
static __INLINE int max(int a, int b)
{
	return a > b ? a : b;
}

/**
 * \brief return minimum of two numbers
 */
static __INLINE int min(int a, int b)
{
	return a > b ? b : a;
}

/** \}*/

/** \name bmc.c
 * -Best Master Clock Algorithm functions */
/**\{*/
/* bmc.c */
/**
 * \brief Compare data set of foreign masters and local data set
 * \return The recommended state for the port
 */
uint8_t bmc(PtpClock*);

/**
 * \brief When recommended state is Master, copy local data into parent and grandmaster dataset
 */
void m1(PtpClock*);

/**
 * \brief When recommended state is Passive
 */
void p1(PtpClock*);

/**
 * \brief When recommended state is Slave, copy dataset of master into parent and grandmaster dataset
 */
void s1(PtpClock*, const MsgHeader*, const MsgAnnounce*);

/**
 * \brief Initialize datas
 */
void initData(PtpClock*);

/**
 * \brief Compare two port identities
 */
bool  isSamePortIdentity(const PortIdentity*, const PortIdentity*);

/**
 * \brief Add foreign record defined by announce message
 */
//void addForeign(PtpClock*, const MsgHeader*, const MsgAnnounce*);
void addForeign(PtpClock *ptpClock,unsigned char *buf,MsgHeader *header, unsigned char localPreference, uint32_t sourceAddr);


/** \}*/


/** \name protocol.c
 * -Execute the protocol engine */
/**\{*/
/**
 * \brief Protocol engine
 */
/* protocol.c */

/**
 * \brief Run PTP stack in current state
 */
void stm_ptpd_doState(PtpClock*);

/**
 * \brief Change state of PTP stack
 */
void stm_ptpd_toState(PtpClock*, uint8_t);
/** \}*/

// Send an alert to the PTP daemon thread.
void ptpd_alert(void);

// Initialize PTP daemon thread.
void ptpd_init(void);




#endif /* PTPD_H_*/

#endif
