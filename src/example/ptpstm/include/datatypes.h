//-- STM PTP

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include "yInc.h"
#include "ptpstm/include/constants.h"
#include "lwipopts.h"
#include "opt.h"
//============= datatypes_dep.h
// Implementation specific datatypes

typedef unsigned char enum4bit_t;
typedef unsigned char uint4bit_t;// 4-bit  unsigned integer
typedef unsigned char nibble_t;// 4-bit data without numerical representation
typedef char octet_t;// 8-bit data without numerical representation

//typedef unsigned char boolean;
//typedef boolean booleanean;
//typedef boolean booleanean;

// 8-bit enumeration
typedef unsigned char enum8bit_t;

// 16-bit enumeration
typedef unsigned short enum16bit_t;

// 48-bit unsigned integer
typedef struct
{
	unsigned int lsb;
	unsigned short msb;
} uint48bit_t;

typedef struct
{
	unsigned int lsb;
	int msb;
} Integer64LM;



// Struct used  to average the offset from master and the one way delay
//
// Exponencial smoothing
//
// alpha = 1/2^s
// y[1] = x[0]
// y[n] = alpha * x[n-1] + (1-alpha) * y[n-1]
//
typedef struct
{
	int   y_prev;
	int   y_sum;
	short   s;		//filter order
	short   s_prev; 	//filter order
	int n;
} Filter;



// Define compiler specific symbols
#if defined   ( __CC_ARM   )
typedef long ssize_t;
#elif defined ( __ICCARM__ )
typedef long ssize_t;
#elif defined (  __GNUC__  )

#elif defined   (  __TASKING__  )
typedef long ssize_t;
#endif
/**
 *\file
 * \brief 5.3 Derived data type specifications
 *
 * This header file defines structures defined by the spec,
 * main program data structure, and all messages structures
 */


/**
 * \brief 5.3.2 The TimeInterval type represents time intervals
 * in scaled nanoseconds where scaledNanoseconds = time[ns] * 2^16
 */

typedef struct
{
		int64_t scaledNanoseconds;
} TimeInterval;

/**
 * \brief 5.3.3 The Timestamp type represents a positive time with respect to the epoch
 */

typedef struct
{
	uint48bit_t secondsField;
	unsigned int nanosecondsField;
} Timestamp;

/**
 * \brief 5.3.4 The ClockIdentity type identifies a clock
 */
typedef char ClockIdentity[CLOCK_IDENTITY_LENGTH];

/**
 * \brief 5.3.5 The PortIdentity identifies a PTP port.
 */

typedef struct
{
		ClockIdentity clockIdentity;
		short portNumber;
} PortIdentity;

/**
 * \brief 5.3.6 The PortAdress type represents the protocol address of a PTP port
 */

 typedef struct
{
		enum16bit_t networkProtocol;
		short adressLength;
		char* adressField;
} PortAdress;

/**
* \brief 5.3.7 The ClockQuality represents the quality of a clock
 */

typedef struct
{
		unsigned char clockClass;
		enum8bit_t clockAccuracy;
		short offsetScaledLogVariance;
} ClockQuality;

/**
 * \brief 5.3.8 The TLV type represents TLV extension fields
 */

typedef struct
{
		enum16bit_t tlvType;
		short lengthField;
		char* valueField;
} TLV;

/**
 * \brief 5.3.9 The PTPText data type is used to represent textual material in PTP messages
 * textField - UTF-8 encoding
 */

typedef struct
{
		unsigned char lengthField;
		char* textField;
} PTPText;

/**
* \brief 5.3.10 The FaultRecord type is used to construct fault logs
 */

typedef struct
{
		short faultRecordLength;
		Timestamp faultTime;
		enum8bit_t severityCode;
		PTPText faultName;
		PTPText faultValue;
		PTPText faultDescription;
} FaultRecord;


/**
 * \brief The common header for all PTP messages (Table 18 of the spec)
 */

typedef struct
{
		nibble_t transportSpecific;
		enum4bit_t messageType;
		nibble_t  versionPTP;
		short messageLength;
		unsigned char domainNumber;
		unsigned char reserved1;		//char flagField[2];
		unsigned char flagField0;
		unsigned char flagField1;
		Integer64LM correctionField;//was int64_t correctionField;
		unsigned int reserved2;
		PortIdentity sourcePortIdentity; //10
		unsigned short sequenceId;
		unsigned char controlField;
		char logMessageInterval;
} MsgHeader;


/**
 * \brief Announce message fields (Table 25 of the spec)
 */

typedef struct
{
		Timestamp originTimestamp;
		short currentUtcOffset;
		unsigned char grandmasterPriority1;
		ClockQuality grandmasterClockQuality;
		unsigned char grandmasterPriority2;
		ClockIdentity grandmasterIdentity;
		short stepsRemoved;
		enum8bit_t timeSource;
}MsgAnnounce;


/**
 * \brief Sync message fields (Table 26 of the spec)
 */

typedef struct
{
		Timestamp originTimestamp;
}MsgSync;

/**
 * \brief DelayReq message fields (Table 26 of the spec)
 */

typedef struct
{
		Timestamp originTimestamp;
}MsgDelayReq;

/**
 * \brief DelayResp message fields (Table 30 of the spec)
 */

typedef struct
{
		Timestamp receiveTimestamp;
		PortIdentity requestingPortIdentity;
}MsgDelayResp;

/**
 * \brief FollowUp message fields (Table 27 of the spec)
 */

typedef struct
{
		Timestamp preciseOriginTimestamp;
}MsgFollowUp;

/**
 * \brief PDelayReq message fields (Table 29 of the spec)
 */

typedef struct
{
		Timestamp originTimestamp;

}MsgPDelayReq;

/**
 * \brief PDelayResp message fields (Table 30 of the spec)
 */

typedef struct
{
		Timestamp requestReceiptTimestamp;
		PortIdentity requestingPortIdentity;
}MsgPDelayResp;

/**
 * \brief PDelayRespFollowUp message fields (Table 31 of the spec)
 */

typedef struct
{
		Timestamp responseOriginTimestamp;
		PortIdentity requestingPortIdentity;
}MsgPDelayRespFollowUp;

/**
* \brief Signaling message fields (Table 33 of the spec)
 */

typedef struct
{
		PortIdentity targetPortIdentity;
		char* tlv;
}MsgSignaling;

/**
* \brief Management message fields (Table 37 of the spec)
 */

typedef struct
{
		PortIdentity targetPortIdentity;
		unsigned char startingBoundaryHops;
		unsigned char boundaryHops;
		enum4bit_t actionField;
		char* tlv;
}MsgManagement;


/**
* \brief Time structure to handle Linux time information
 */

typedef struct
{
		int seconds;
		int nanoseconds;
} TimeInternal;

typedef struct {
	int  interval;
	int  left;
	boolean expire;
} IntervalTimer;

/*
typedef struct {
      Integer32 scaledNanoseconds;   //was Integer64 scaledNanoseconds;
} TimeInterval; //YOON-TI
*/
/**
* \brief ForeignMasterRecord is used to manage foreign masters
 */

typedef struct
{
		PortIdentity foreignMasterPortIdentity;
		short foreignMasterAnnounceMessages;

		/* This one is not in the spec */
		MsgAnnounce  announce; //all data set
		MsgHeader    header; //some datasets
		unsigned char    localPreference; /* local preference - only used by telecom profile */
		unsigned int   sourceAddr; /* source address */
		boolean	     disqualified; /* if true, this one always loses */

} ForeignMasterRecord;

/**
 * \struct DefaultDS
 * \brief spec 8.2.1 default data set
 * spec 7.6.2, spec 7.6.3 PTP device attributes
 */

typedef struct
{
		boolean  twoStepFlag;
		ClockIdentity clockIdentity; /**< spec 7.6.2.1 */
		short numberPorts;  /**< spec 7.6.2.7 */
		ClockQuality clockQuality; /**< spec 7.6.2.4, 7.6.3.4 and 7.6.3 */
		unsigned char priority1; /**< spec 7.6.2.2 */
		unsigned char priority2; /**< spec 7.6.2.3 */
		unsigned char domainNumber;
		boolean  slaveOnly;
} DefaultDS;


/**
 * \struct CurrentDS
 * \brief spec 8.2.2 current data set
 */

typedef struct
{
		short stepsRemoved;
		TimeInternal offsetFromMaster;
		TimeInternal meanPathDelay;
} CurrentDS;


/**
 * \struct ParentDS
 * \brief spec 8.2.3 parent data set
 */

typedef struct
{
		PortIdentity parentPortIdentity;
		/* 7.6.4 Parent clock statistics - parentDS */
		boolean  parentStats; /**< spec 7.6.4.2 */
		short observedParentOffsetScaledLogVariance; /**< spec 7.6.4.3 */
		int observedParentClockPhaseChangeRate; /**< spec 7.6.4.4 */

		ClockIdentity grandmasterIdentity;
		ClockQuality grandmasterClockQuality;
		unsigned char grandmasterPriority1;
		unsigned char grandmasterPriority2;
} ParentDS;

/**
 * \struct TimePropertiesDS
 * \brief spec 8.2.4 time properties data set
 */

 typedef struct
{
		short currentUtcOffset;//Underlying time source UTC offset announced in master state. It will be updated with Master's Announce Msg.
		boolean  currentUtcOffsetValid; //"Underlying time source UTC offset validity announced in master state."
		boolean  leap59;
		boolean  leap61;
		boolean  timeTraceable;//Underlying time source time traceability announced in master state.
		boolean  frequencyTraceable;//Underlying time source frequency traceability announced in master state.
		boolean  ptpTimescale; //TRUE=PTP; FALSE=ARB
		//"Time scale announced in master state (with ARB, UTC properties are ignored by slaves).
		// When clock class is set to 13 (application specific), this value is ignored and ARB is used.",
		enum8bit_t timeSource; //< spec 7.6.2.6. GPS, ATOMIC,  .. INTERNAL_OSCILLATOR
} TimePropertiesDS;


/**
 * \struct PortDS
 * \brief spec 8.2.5 port data set
 */

 typedef struct
{
		PortIdentity portIdentity;
		enum8bit_t portState;
		enum8bit_t lastPortState; //YOON 2016.09.30
		char logMinDelayReqInterval; /**< spec 7.7.2.4 */
		TimeInternal peerMeanPathDelay;
		char logAnnounceInterval; /**< spec 7.7.2.2 */
		unsigned char announceReceiptTimeout; /**< spec 7.7.3.1 */
		char logSyncInterval; /**< spec 7.7.2.3 */
		enum8bit_t delayMechanism;
		char logMinPDelayReqInterval; /**< spec 7.7.2.5 */
		nibble_t  versionNumber; //4 bit
		char transportSpecific;
} PortDS;


/**
 * \struct ForeignMasterDS
 * \brief Foreign master data set
 */

//YOON 2016.9.30
typedef struct
{
	// Foreign master data set
	ForeignMasterRecord *foreign;
	// Current best master (unless it's us)
	ForeignMasterRecord *bestMaster;

	// Other things we need for the protocol
	unsigned short number_foreign_records;//was count
	short  max_foreign_records; //was capacity
	short  foreign_record_i; //was i
	short  foreign_record_best;	//was best

	boolean  record_update;    // should we run bmc() after receiving an announce message?
} ForeignMasterDS;

/**
 * \struct Servo
 * \brief Clock servo filters and PI regulator values
 */

typedef struct
{
		boolean  noResetClock;
		boolean  noAdjust;
		short ap, ai;
		short sDelay;
		short sOffset;
		int servoMaxPpb;
		//==from PIservo
		int  observedDrift;
	    int maxOutput;
	    TimeInternal lastUpdate;
	    boolean runningMaxOutput;
		boolean stepForce; 	// force clock step on first sync after startup
		boolean stepOnce; 		// only step clock on first sync after startup

} Servo;

typedef struct{
    int maxOutput;
    int input;
    double output;
    double observedDrift;
    double kP, kI;
    TimeInternal lastUpdate;
    boolean runningMaxOutput;
    int dTmethod;
    double dT;
    int maxdT;
#ifdef PTPD_STATISTICS
    int updateCount;
    int stableCount;
    boolean statsUpdated;
    boolean statsCalculated;
    boolean isStable;
    double stabilityThreshold;
    int stabilityPeriod;
    int stabilityTimeout;
    double driftMean;
    double driftStdDev;
    double driftMedian;
    double driftMin;
    double driftMax;
    double driftMinFinal;
    double driftMaxFinal;
    DoublePermanentStdDev driftStats;
    DoublePermanentMedian driftMedianContainer;
#endif /* PTPD_STATISTICS */
} PIservo;

typedef struct {
	boolean activity; 		/* periodic check, updateClock sets this to let the watchdog know we're holding clock control */
	boolean	available; 	/* flags that we can control the clock */
	boolean granted; 	/* upstream watchdog will grant this when we're the best provider */
	boolean updateOK;		/* if not, updateClock() will not run */
	boolean stepRequired;		/* if set, we need to step the clock */
	boolean offsetOK;		/* if set, updateOffset accepted oFm */
} ClockControlInfo;

typedef struct {
	boolean inSync;
	boolean leapInsert;
	boolean leapDelete;
	boolean majorChange;
	boolean update;
	boolean override; /* this tells the client that this info takes priority
			   * over whatever the client itself would like to set
			   * prime example: leap second file
			   */
	int utcOffset;
	long clockOffset;
} ClockStatusInfo;


#if USE_LWIP

// Network  buffer queue
typedef struct{
	void      *pbuf[PBUF_QUEUE_SIZE];
	short   head;
	short   tail;
	unsigned short qtype; //YOON for debug (319ev/320gen)
	//sys_mutex_t mutex;
} BufQueue;

// Struct used  to store network datas
typedef struct
{
	int   multicastAddr;
	int   peerMulticastAddr;
	int   unicastAddr;

	struct in_addr interfaceAddr;

#if (PTP_PROTOCOL == IEEE1588V2)
	struct udp_pcb    *eventPcb;
	struct udp_pcb    *generalPcb;
#elif (PTP_PROTOCOL == IEEE8021AS)
	struct rawavb_pcb    *eventPcb;
	struct rawavb_pcb    *generalPcb;
#endif
	struct rawavb_pcb    *ieee8021Pcb;
	// source address of last received packet - used for unicast replies to Delay Requests
	int lastSourceAddr;
	// destination address of last received packet - used for unicast FollowUp for multiple slaves
	int lastDestAddr;

	BufQueue    eventQ;
	BufQueue    generalQ;

	BufQueue    ieee8021Q;

	//unsigned short qseq; //YOON added for buffer synchronization.
} NetPath;
/**
 * \struct RunTimeOpts
 * \brief Program options set at run-time
 */

typedef struct
{
	//Protocols
		char  announceInterval;
		char  syncInterval;
		char logMinDelayReqInterval;
		char logMinPdelayReqInterval;
		char masterRefreshInterval;


		// Max intervals for unicast negotiation
		char logMaxPdelayReqInterval;
		char logMaxDelayReqInterval;
		char logMaxSyncInterval;
		char logMaxAnnounceInterval;

		char announceReceiptTimeout;

		boolean syncSequenceChecking;//ptpd ensures that Sync message sequence numbers  are increasing (consecutive sync is not lower than last).
		 // This can prevent reordered sequences, but can also lock the slave up if, say, GM restarted and reset sequencing.

		int maxDelayMaxRejected;

		//Clocks

		ClockQuality clockQuality;
		TimePropertiesDS timeProperties;

		unsigned char  priority1;
		unsigned char  priority2;
		unsigned char  domainNumber;
		unsigned short portNumber;
		boolean anyDomain;// optional BMC extension: accept any domain, prefer configured domain, prefer lower domain


		//BMCA
		boolean   slaveOnly;
		boolean disableBMCA; // used to achieve master-only for unicast


		short  currentUtcOffset;

		enum8bit_t stats;

		short   maxForeignRecords;
		enum8bit_t  delayMechanism;


		Servo servo;

		int calibrationDelay;//(0..300). Delay between moving to slave state and enabling clock updates (seconds).
		// This allows one-way delay to stabilise before starting clock updates.
		// Activated when going into slave state and during slave's GM failover.

		int clockUpdateTimeout;//If set to non-zero, timeout in seconds, after which the slave resets if no clock updates made. (0..3600)

		// "panic mode" support

		boolean enablePanicMode;
		boolean panicModeReleaseClock;
		int panicModeDuration;
		unsigned int panicModeExitThreshold;

		int maxOffset; // Maximum number of nanoseconds of offset
		int maxDelay;  // Maximum number of nanoseconds of delay

		int maxListen; //YOON 2016.09.30
		//char initial_delayreq;//YOON 2016.09.30

		boolean alwaysRespectUtcOffset;
		boolean preferUtcValid;
		boolean requireUtcValid;

		//network
		boolean   portDisabled;
		unsigned char transport; // transport type
		unsigned char ipMode; // IP transmission mode
		boolean dot1AS; // 801.2AS support -> transportSpecific field
		boolean disableUdpChecksums; // disable UDP checksum validation where supported
		char   unicastAddress[NET_ADDRESS_LENGTH];
		char   ifaceName[IFACE_NAME_LENGTH];
		TimeInternal inboundLatency, outboundLatency;
		TimeInternal ofmShift; //Apply an arbitrary shift (nanoseconds) to offset from master when in slave state.
		//Value can be positive or negative - useful for correcting for of antenna latencies,
		//delay assymetry and IP stack latencies. This will not be visible in the offset from master value -
		//only in the resulting clock correction.

} RunTimeOpts;

/**
 * \struct PtpClock
 * \brief Main program data structure
 */
/* main program data structure */

typedef struct
{
	boolean disabled;

    unsigned char subdomainAddress[SUBDOMAIN_ADDRESS_LENGTH]; //YOON

	DefaultDS defaultDS; /**< default data set */
	CurrentDS currentDS; /**< current data set */
	ParentDS parentDS; /**< parent data set */
	TimePropertiesDS timePropertiesDS; /**< time properties data set */
	PortDS portDS; /**< port data set */
	ForeignMasterDS foreignMasterDS; /**< foreign master data set */

	// Leap second related flags
    boolean leapSecondInProgress;
    boolean leapSecondPending;

	MsgHeader msgTmpHeader; /**< buffer for incomming message header */

		union
		{
				MsgSync  sync;
				MsgFollowUp  follow;
				MsgDelayReq  req;
				MsgDelayResp resp;
				MsgPDelayReq  preq;
				MsgPDelayResp  presp;
				MsgPDelayRespFollowUp  prespfollow;
				MsgManagement  manage;
				MsgAnnounce  announce;
				MsgSignaling signaling;
		} msgTmp; /**< buffer for incomming message body */


		char msgObuf[PACKET_SIZE]; /**< buffer for outgoing message */
		char msgIbuf[PACKET_SIZE]; /** <buffer for incomming message */
		ssize_t msgIbufLength; /**< length of incomming message */

		TimeInternal Tms_offset;//Tms; /**< Time Master -> Slave */
		TimeInternal delaySM; //Tsm; /**< Time Slave -> Master */

		TimeInternal pdelay_t1; /**< peer delay time t1 */
		TimeInternal pdelay_t2; /**< peer delay time t2 */
		TimeInternal pdelay_t3; /**< peer delay time t3 */
		TimeInternal pdelay_t4; /**< peer delay time t4 */

		TimeInternal timestamp_syncRecieve; /**< timestamp of Sync message */
		TimeInternal timestamp_delayReqSend; /**< timestamp of delay request message */
		TimeInternal timestamp_delayReqRecieve; /**< timestamp of delay request message */

		TimeInternal  lastSyncCorrectionField;//TimeInternal correctionField_sync; /**< correction field of Sync and FollowUp messages */
		TimeInternal  lastPdelayRespCorrectionField;//TimeInternal correctionField_pDelayResp; /**< correction fieald of peedr delay response */

		/* MsgHeader  PdelayReqHeader; */ /**< last recieved peer delay reques header */

		short sentPDelayReqSequenceId;
		short sentDelayReqSequenceId;
		short sentSyncSequenceId;
		short sentAnnounceSequenceId;

		short recvPDelayReqSequenceId;
		short recvPdelayRespSequenceId; //added
		unsigned short rcvdSyncSequenceId;

		boolean   waitingForFollowUp; /**< true if sync message was recieved and 2step flag is set */
		boolean   waitingForPDelayRespFollowUp; /**< true if PDelayResp message was recieved and 2step flag is set */
		boolean   waitingForDelayResp; //delayRespWaiting //YOON 2016.09.30
		boolean   WaitingForSync; //syncWaiting;

		int maxDelayRejected;

		//announceTimeouts
		//followUpGap

		Filter  ofm_filt; /**< filter offset from master */
		Filter  owd_filt; /**< filter one way delay */
		Filter  slv_filt; /**< filter scaled log variance */
		short offsetHistory[2];


		boolean  messageActivity;

		IntervalTimer  itimer[TIMER_ARRAY_SIZE]; //YOON ADDED

		NetPath netPath; //YOON
		unsigned short  event_port_address; //YOON
		unsigned short general_port_address; //YOON

		enum8bit_t recommendedState;

		char portUuidField[PTP_UUID_LENGTH]; /**< Usefull to init network stuff */

		TimeInternal  inboundLatency, outboundLatency;

		Servo servo; //Duplciate on RunTimeOpts...

		int  events;

		enum8bit_t  stats;
		RunTimeOpts * rtOpts;

		int  listenCount; //2016.09.30
		int resetCount; //2016.09.30

		ClockControlInfo clockControl;
		ClockStatusInfo clockStatus;

		boolean startup_in_progress;
		boolean pastStartup;				/* we've set the clock already, at least once */
		boolean offsetFirstUpdated;
		boolean isCalibrated;

		boolean panicMode; // in panic mode - do not update clock or calculate offsets
		boolean panicOver; // panic mode is over, we can reset the clock
		int panicModeTimeLeft; // How many 30-second periods left in panic mode

} PtpClock;

PACK_STRUCT_BEGIN
struct ieee8021as_hdr {
  // transportspecific(4b)| messageType(4b)
  PACK_STRUCT_FIELD(u8_t _ts_msgType);
  // rsvd(4b) | ver(4b)
  PACK_STRUCT_FIELD(u8_t _rsvdVer);
  // msgLen(2)
  PACK_STRUCT_FIELD(u16_t _msgLen);
  //DomainNum(1)
  PACK_STRUCT_FIELD(u8_t _domainNum);
  //rsvd(1)
  PACK_STRUCT_FIELD(u8_t _Rsvd1);
  //flagField(2)
  PACK_STRUCT_FIELD(u16_t _flagField);
//#define IP_RF 0x8000U        /* reserved fragment flag */
//#define IP_DF 0x4000U        /* dont fragment flag */
//#define IP_MF 0x2000U        /* more fragments flag */
//#define IP_OFFMASK 0x1fffU   /* mask for fragmenting bits */
  //Correction Field(8)
  PACK_STRUCT_FIELD(long long _correctionField);
  //rsvd(4)
  PACK_STRUCT_FIELD(u32_t _Rsvd2);
  //sourcePortID(10)
  PACK_STRUCT_FIELD(u8_t _srcPortId[10]);
  //SeqId(2)
  PACK_STRUCT_FIELD(u16_t _seqId);
  //controlField(1)
  PACK_STRUCT_FIELD(u8_t _controlField);
  //logMsgInterval(1)
  PACK_STRUCT_FIELD(u8_t _logMsgInterval);
  //PACK_STRUCT_FIELD(ip_addr_p_t src);
  //PACK_STRUCT_FIELD(ip_addr_p_t dest);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

#endif//USE_LWIP

#endif /* DATATYPES_H_*/
