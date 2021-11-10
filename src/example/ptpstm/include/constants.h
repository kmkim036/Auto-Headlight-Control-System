/**
 * This file is part of the STM32 PTP
 * 
 *
 **/
#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#ifndef TRUE
#define TRUE true
#endif

#ifndef FALSE
#define FALSE false
#endif

/**
*\file
* \brief Default values and constants used in ptpdv2
*
* This header file includes all default values used during initialization
* and enumeration defined in the spec
 */
//============= MAIN features, only change to reflect changes in implementation ===============
#define NUMBER_PORTS      1
#define VERSION_PTP       2
#define BOUNDARY_CLOCK    FALSE
#define SLAVE_ONLY        TRUE
#define NO_ADJUST         FALSE
//==============================================================================================
/* 5.3.4 ClockIdentity */
#define CLOCK_IDENTITY_LENGTH 8

#define MANUFACTURER_ID \
		"PTPd;2.0.1\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"

//============= IMPORTANT Implementation specific constants ========================================================
#define DEFAULT_PRIORITY1               248
#define DEFAULT_CLOCK_CLASS             248 //default
#define DEFAULT_CLOCK_CLASS_SLAVE_ONLY  255
#define DEFAULT_CLOCK_ACCURACY          0xFE
#define DEFAULT_CLOCK_VARIANCE          5000 /* In 802.1AS, it is offsetScaledLogVariance */
#define DEFAULT_PRIORITY2               248
//The lowest Priority is GM Identity

#define DEFAULT_TIME_SOURCE             INTERNAL_OSCILLATOR
#define DEFAULT_DELAY_MECHANISM         E2E //P2P//= 802.1as //E2E

#define DEFAULT_INBOUND_LATENCY         7700       /* in nsec */
#define DEFAULT_OUTBOUND_LATENCY        7700       /* in nsec */
#define DEFAULT_NO_RESET_CLOCK          FALSE
#define DEFAULT_DOMAIN_NUMBER           0
#define DEFAULT_AP                      2
#define DEFAULT_AI                      16
#define DEFAULT_DELAY_S                 6 /* exponencial smoothing - 2^s */
#define DEFAULT_OFFSET_S                1 /* exponencial smoothing - 2^s */
#define DEFAULT_ANNOUNCE_INTERVAL       1 /* 0 in 802.1AS */
#define DEFAULT_UTC_OFFSET              34
#define DEFAULT_UTC_VALID               FALSE
#define DEFAULT_PDELAYREQ_INTERVAL      1 /* -4 in 802.1AS */
#define DEFAULT_DELAYREQ_INTERVAL       3 /* from DEFAULT_SYNC_INTERVAL to DEFAULT_SYNC_INTERVAL + 5 */
#if (PTP_PROTOCOL == IEEE8021AS)
#define DEFAULT_SYNC_INTERVAL           (-3) //0 /* -3 in 802.1AS */
#else
#define DEFAULT_SYNC_INTERVAL           0
#endif
#define DEFAULT_SYNC_RECEIPT_TIMEOUT    3
#define DEFAULT_ANNOUNCE_RECEIPT_TIMEOUT 3 /* 3 by default */
#define DEFAULT_QUALIFICATION_TIMEOUT   -9 /* DEFAULT_ANNOUNCE_INTERVAL + N */ //If you implement not ordinary clock.
#define DEFAULT_FOREIGN_MASTER_TIME_WINDOW 4
#define DEFAULT_FOREIGN_MASTER_THRESHOLD 2

#define DEFAULT_MAX_FOREIGN_RECORDS     2//5
#define DEFAULT_PARENTS_STATS           FALSE
#define DEFAULT_TWO_STEP_FLAG           TRUE /* Transmitting only SYNC message or SYNC and FOLLOW UP */

#define DEFAULT_TIME_TRACEABLE          FALSE /* time derived from atomic clock? */
#define DEFAULT_FREQUENCY_TRACEABLE     FALSE /* frequency derived from frequency standard? */
#define DEFAULT_TIMESCALE               ARB_TIMESCALE /* PTP_TIMESCALE or ARB_TIMESCALE */

#define DEFAULT_CALIBRATED_OFFSET_NS    1000000//1000 /* offset from master < 1us -> calibrated */
#define DEFAULT_UNCALIBRATED_OFFSET_NS  1000000000//10000 /* offset from master > 10us -> uncalibrated */
#define MAX_ADJ_OFFSET_NS       		400000 /* max offset to try to adjust it < 400us */
#define ADJ_FREQ_MAX  					512000

#define LOWEST_LOCALPREFERENCE 255 //for bmc
#define SLAVE_ONLY_CLOCK_CLASS			255

/* UDP/IPv4 dependent */
#define SUBDOMAIN_ADDRESS_LENGTH  		4
#define PORT_ADDRESS_LENGTH       		2
#define PTP_UUID_LENGTH     			NETIF_MAX_HWADDR_LEN
#define CLOCK_IDENTITY_LENGTH   		8
#define FLAG_FIELD_LENGTH    			2
#define PACKET_SIZE  					300 /* ptpdv1 value kept because of use of TLV... */

#define PTP_EVENT_PORT   				319
#define PTP_GENERAL_PORT  				320

#define DEFAULT_PTP_DOMAIN_ADDRESS  	"224.0.1.129" //"255.255.255.255" //"224.0.1.129" //"255.255.255.255" //for test only "224.0.1.129"
#define PEER_PTP_DOMAIN_ADDRESS     	"224.0.0.107"

#define MM_STARTING_BOUNDARY_HOPS  		0x7fff

#define DEFAULT_PTP_ALLOWABLE_JITTER_FOR_SYNCRONIZED_IN_NS   500
//=========================================== PACKET FORMAT ==========================================================

/* name Packet length
 Minimal length values for each message.
 If TLV used length could be higher.*/

#define HEADER_LENGTH                 34
#define ANNOUNCE_LENGTH               64
#define SYNC_LENGTH                   44
#define FOLLOW_UP_LENGTH              44
#define PDELAY_REQ_LENGTH             54
#define DELAY_REQ_LENGTH              44
#define DELAY_RESP_LENGTH             54
#define PDELAY_RESP_LENGTH            54
#define PDELAY_RESP_FOLLOW_UP_LENGTH  54
#define MANAGEMENT_LENGTH             48

//Domain Number (Table 2 in the spec)
enum{
		DFLT_DOMAIN_NUMBER = 0,
		ALT1_DOMAIN_NUMBER,
		ALT2_DOMAIN_NUMBER,
		ALT3_DOMAIN_NUMBER
};

/**
 * \brief Network Protocol  (Table 3 in the spec)*/
enum
{
		UDP_IPV4 = 1,
		UDP_IPV6,
		IEEE_802_3,
		DeviceNet,
		ControlNet,
		PROFINET
};
/* IP transmission mode */
enum {
	IPMODE_MULTICAST = 0,
	IPMODE_UNICAST,
	IPMODE_HYBRID,
#if 0
	IPMODE_UNICAST_SIGNALING
#endif
};
/**
 * \brief Time Source (Table 7 in the spec)*/
enum
{
	ATOMIC_CLOCK = 0x10,
	GPS = 0x20,
	TERRESTRIAL_RADIO = 0x30,
	PTP = 0x40,
	NTP = 0x50,
	HAND_SET = 0x60,
	OTHER = 0x90,
	INTERNAL_OSCILLATOR = 0xA0
};


/**
 * \brief PTP State (Table 8 in the spec)*/
enum
{
	PTP_INITIALIZING = 0,
	PTP_FAULTY,
	PTP_DISABLED,
	PTP_LISTENING,
	PTP_PRE_MASTER, //Removed 2016.09.30 YOON
	PTP_MASTER,
	PTP_PASSIVE,
	PTP_UNCALIBRATED,
	PTP_SLAVE
};

/**
 * \brief Delay mechanism (Table 9 in the spec)
 */
enum
{
	E2E = 1,
	P2P = 2,
	DELAY_DISABLED = 0xFE
};

/**
 * \brief PTP timers
 */
enum
{
	PDELAYREQ_INTERVAL_TIMER = 0,/**<\brief Timer handling the PdelayReq Interval */
	DELAYREQ_INTERVAL_TIMER,/**<\brief Timer handling the delayReq Interva */
	SYNC_INTERVAL_TIMER,/**<\brief Timer handling Interval between master sends two Syncs messages */
	ANNOUNCE_RECEIPT_TIMER,/**<\brief Timer handling announce receipt timeout */
	ANNOUNCE_INTERVAL_TIMER, /**<\brief Timer handling interval before master sends two announce messages */
	  /* non-spec timers */
	SYNC_RECEIPT_TIMER,
	DELAY_RECEIPT_TIMER,
	//ADDED TO BE USED
	UNICAST_GRANT_TIMER, /* used to age out unicast grants (sent, received) */
	OPERATOR_MESSAGES_TIMER,  /* used to limit the operator messages */
	LEAP_SECOND_PAUSE_TIMER, /* timer used for pausing updates when leap second is imminent */
	STATUSFILE_UPDATE_TIMER, /* timer used for refreshing the status file */
	PANIC_MODE_TIMER,	   /* timer used for the duration of "panic mode" */
	PERIODIC_INFO_TIMER,	   /* timer used for dumping periodic status updates */
#ifdef PTPD_STATISTICS
	  STATISTICS_UPDATE_TIMER, /* online mean / std dev updare interval (non-moving statistics) */
#endif /* PTPD_STATISTICS */
	ALARM_UPDATE_TIMER,
	MASTER_NETREFRESH_TIMER,
	CALIBRATION_DELAY_TIMER,
	CLOCK_UPDATE_TIMER,
	TIMINGDOMAIN_UPDATE_TIMER,
	YOON_TIMER,
	TIMER_ARRAY_SIZE  /* this one is non-spec */
};


/**
 * \brief PTP Messages (Table 19)
 */
enum
{
	SYNC = 0x0,
	DELAY_REQ,
	PDELAY_REQ,
	PDELAY_RESP,
	FOLLOW_UP = 0x8,
	DELAY_RESP,
	PDELAY_RESP_FOLLOW_UP,
	ANNOUNCE,
	SIGNALING,
	MANAGEMENT,
};

/**
 * \brief message flags
 */
//flagField1 bit position values (Table 20 in the spec)
 enum
{
	FLAG1_LEAP61 = 0x01,
	FLAG1_LEAP59 = 0x02,
	FLAG1_UTC_OFFSET_VALID = 0x04,
	FLAG1_PTP_TIMESCALE = 0x08,
	FLAG1_TIME_TRACEABLE = 0x10,
	FLAG1_FREQUENCY_TRACEABLE = 0x20,
};
 /* the different representation of the above.
 enum {
 	LI61=0,
 	LI59,
 	UTCV,
 	PTPT, // this is referred to as PTP in the spec but already defined above
 	TTRA,
 	FTRA
 };
 */
/**
 * \brief PTP Messages control field (Table 23)
 */
enum
{
	CTRL_SYNC = 0x00,
	CTRL_DELAY_REQ,
	CTRL_FOLLOW_UP,
	CTRL_DELAY_RESP,
	CTRL_MANAGEMENT,
	CTRL_OTHER,
};

/**
 * \brief Output statistics
 */

enum
{
	PTP_NO_STATS = 0,
	PTP_TEXT_STATS,
	PTP_CSV_STATS /* not implemented */
};

/**
 * \brief message flags
 */

enum
{
	FLAG0_ALTERNATE_MASTER = 0x01,
	FLAG0_TWO_STEP = 0x02,
	FLAG0_UNICAST = 0x04,
	FLAG0_PTP_PROFILE_SPECIFIC_1 = 0x20,
	FLAG0_PTP_PROFILE_SPECIFIC_2 = 0x40,
	FLAG0_SECURITY = 0x80,
};

/* communication technology */
enum {
	PTP_ETHER, PTP_DEFAULT
};


/**
 * \brief ptp stack events
 */

enum
{
	POWERUP = 0x0001,
	INITIALIZE = 0x0002,
	DESIGNATED_ENABLED = 0x0004,
	DESIGNATED_DISABLED = 0x0008,
	FAULT_CLEARED = 0x0010,
	FAULT_DETECTED = 0x0020,
	STATE_DECISION_EVENT = 0x0040, //Valid announce message is received : BMC algorithm will be executed
	QUALIFICATION_TIMEOUT_EXPIRES = 0x0080,
	ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES = 0x0100,
	SYNCHRONIZATION_FAULT = 0x0200,
	MASTER_CLOCK_SELECTED = 0x0400,
	/* non spec */
	MASTER_CLOCK_CHANGED = 0x0800,
};

/**
 * \brief ptp time scale
 */

enum
{
	ARB_TIMESCALE,
	PTP_TIMESCALE
};

//================ constants_dep.h =============
/* platform dependent */

#define IF_NAMESIZE             2
#define INET_ADDRSTRLEN         16

// TransportSpecific values
enum {
		TSP_DEFAULT = 0x00,
		TSP_ETHERNET_AVB = 0x01
};

#ifndef stdout
#define stdout 1
#endif

#define IFACE_NAME_LENGTH         IF_NAMESIZE
#define NET_ADDRESS_LENGTH        INET_ADDRSTRLEN

#define IFCONF_LENGTH 10

#if BYTE_ORDER == LITTLE_ENDIAN
#define PTPD_LSBF
#elif BYTE_ORDER == BIG_ENDIAN
#define PTPD_MSBF
#endif

/* pow2ms(a) = round(pow(2,a)*1000) */
#define pow2ms(a) (((a)>0) ? (1000 << (a)) : (1000 >>(-(a))))

/* others */

#define SCREEN_BUFSZ  128
#define SCREEN_MAXSZ  80

//2016.09.30 YOON
#define MISSED_MESSAGES_MAX 20 // how long we wait to trigger a [sync/delay] receipt alarm

//YOON
//constants_dep.h
//log timestamp mode
enum {
	TIMESTAMP_DATETIME,
	TIMESTAMP_UNIX,
	TIMESTAMP_BOTH
};
// servo dT calculation mode
enum {
		DT_NONE,
		DT_CONSTANT,
		DT_MEASURED
};
// Leap second handling
enum {
	LEAP_ACCEPT,
	LEAP_IGNORE,
	LEAP_STEP,
	LEAP_SMEAR
};
#endif /* CONSTANTS_H_*/
