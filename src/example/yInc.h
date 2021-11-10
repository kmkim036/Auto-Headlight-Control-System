#ifndef __YINC_H
#define __YINC_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
#include "yLib/eth/include/ethopts.h"
//#define NULL (*(0))

//typedef unsigned char u8;
//typedef char Octet;
//typedef short int s16;
//typedef unsigned short int u16;
//typedef u8 bool;
typedef unsigned char boolean;
typedef unsigned char Boolean;
//typedef u16 uint16_t;
//typedef u8 uint8_t;
typedef signed char Integer8;
typedef signed short Integer16;
typedef signed int Integer32;
typedef unsigned char UInteger8;
typedef unsigned short UInteger16;
typedef unsigned int UInteger32;
//typedef long long Integer64;
typedef unsigned long long   UInteger64;

#define RESOLUTION_IN_USEC 10000	//For SysTick	//10000 = 10millsec;
//#define RESOLUTION_IN_USEC 1000	//For SysTick	//1000= 1millsec; 10000 = 10millsec;
//#define RESOLUTION_IN_USEC 100	//For SysTick	//100= 100usec; 10000 = 10millsec;

#define FALSE 0
#define TRUE  1

#define ERR_OK 		0
#define ERR_FAIL 	-1

extern uint32_t randNum32;

#define ETH_IF_MODE ETH_IF_MII_MODE //ETH_IF_RMII_MODE //ETH_IF_SPI_MODE

#define PROJ_FOR_RSTP 		1
#define PROJ_FOR_PTP  		2
#define PROJ_FOR_COSEM 		3
#define PROJ_FOR_CONTIKI 	4
#define PROJ_FOR_RSTP_PTP	5
#define PROJ_FOR_SJA1105	6
#define PROJ_FOR_EDU		10
#define PROJ_FOR           PROJ_FOR_EDU //PROJ_FOR_PTP //PROJ_FOR_EDU //PROJ_FOR_PTP //PROJ_FOR_EDU //PROJ_FOR_SJA1105 //PROJ_FOR_PTP//PROJ_FOR_RSTP

#if (PROJ_FOR == PROJ_FOR_CONTIKI) || (PROJ_FOR==PROJ_FOR_EDU) ||(PROJ_FOR==PROJ_FOR_SJA1105)
	#define USE_LWIP 	 		0
#else
	#define USE_LWIP 	 		1
#endif

#if (PROJ_FOR == PROJ_FOR_PTP) || (PROJ_FOR == PROJ_FOR_RSTP_PTP)
	#define USE_LWIP_PTP 		1 //for PTP
#endif

#if (PROJ_FOR == PROJ_FOR_RSTP) || (PROJ_FOR == PROJ_FOR_RSTP_PTP)
	#define SWITCH_FAMILY		SWITCH_FAMILY_ICPLUS//ETH_FAMILY_SMSC
	#define SWITCH_ID 			SWITCH_ID_IP175D//SWITCH_LAN9355//SWITCH_KSZ8567 //SWITCH_KSZ8794//  //SWITCH_KSZ8463////SWITCH_LAN9355 //SWITCH_KSZ8794 //SWITCH_KSZ8794 //SWITCH_NOSWITCH//SWITCH_LAN9355 //SWITCH_NOSWITCH//SWITCH_KSZ8794 //SWITCH_NOSWITCH// SWITCH_LAN9355
	#define SWITCH2MCU          SWITCH2MCU_VPHY
	#define PHYCHIP 			VPHY_SWITCH2MCU
#elif (PROJ_FOR == PROJ_FOR_SJA1105)
	#define SWITCH_FAMILY		SWITCH_FAMILY_NXP
	#define SWITCH_ID 			SWITCH_ID_SJA1105 //SWITCH_NOSWITCH
	#define SWITCH2MCU          SWITCH2MCU_VPHY
	#define PHYCHIP 			IP101 //TJA1100 //IP101//RTL9K //IP101//VPHY_SWITCH2MCU //IP101 //BCM89810
#else
	#define SWITCH_FAMILY		SWITCH_FAMILY NOSWITCH// SWITCH_FAMILY_ICPLUS//
	#define SWITCH_ID 			SWITCH_ID_NOSWITCH//SWITCH_ID_IP175D//
	#define PHYCHIP 			IP101//VPHY_SWITCH2MCU//RTL9K //IP101//VPHY_SWITCH2MCU //IP101 //BCM89810
#endif

#if (USE_LWIP_PTP == 1)

	#define USE_MCU_PTP   1          //Use STM32 MCU's PTP Registers.

	#define USE_SYNCLED   1 		//PE6

	#define PTP_PROTOCOL  IEEE1588V2//IEEE8021AS//IEEE8021AS //IEEE1588V2
	#define DEFAULT_DELAY_MECHANISM E2E// P2P//E2E//P2P = 802.1as -- predefined  in constants.h
	#define CLOCKSOURCE CLOCKSOURCE_LOC //CLOCKSOURCE_GPS//CLOCKSOURCE_LOC //CLOCKSOURCE_GPS

	#if (CLOCKSOURCE == CLOCKSOURCE_GPS)
		#define USE_GPS
	#else
		#undef USE_GPS
	#endif

	//ROLEs
	#define ETHIF_ROLE 	PTP_OC//SIMPLE_ETH //PTP_OC // SIMPLE_ETH////PTP_GRANDMASTERCLOCK //SIMPLE_ETH_IP101 //PTP_TC_9355 //PTP_GRANDMASTERCLOCK //SHOULD BE SET

	#if (ETHIF_ROLE == PTP_GRANDMASTERCLOCK)
		//#define IP_ADDR3   209
	#elif (ETHIF_ROLE == PTP_OC)
		#if(SWITCH_ID == SWITCH_ID_KSZ8794)
			//#define IP_ADDR3   94
		#elif(SWITCH_ID == SWITCH_ID_LAN9355)
			//#define IP_ADDR3   55
		#else
			//#define IP_ADDR3    MAC_ADDR5 //101
		#endif
	#else
		//#define IP_ADDR3   MAC_ADDR5 //101
	#endif
#else //RSTP or Simple Purpose
	#define ETHIF_ROLE 	SIMPLE_ETH
#endif

/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#if (ETHIF_ROLE == PTP_GRANDMASTERCLOCK)
	#define MAC_ADDR0   'B'
	#define MAC_ADDR1   'G'
	#define MAC_ADDR2   'R'
	#define MAC_ADDR3   'A'
	#define MAC_ADDR4   'M'
	//#define MAC_ADDR5   g_ip_addr3//'I'
#elif (ETHIF_ROLE == PTP_OC)
	#if(SWITCH_ID == SWITCH_ID_KSZ8794)
		#define MAC_ADDR0   'B'
		#define MAC_ADDR1   'K'
		#define MAC_ADDR2   'R'
		#define MAC_ADDR3   'A'
		#define MAC_ADDR4   'M'
		//#define MAC_ADDR5   'M'
	#elif(SWITCH_ID == SWITCH_ID_LAN9355)
		#define MAC_ADDR0   'B'
		#define MAC_ADDR1   'L'
		#define MAC_ADDR2   'R'
		#define MAC_ADDR3   'A'
		#define MAC_ADDR4   'M'
		//#define MAC_ADDR5   'M'
	#else
		#define MAC_ADDR0   'B'
		#define MAC_ADDR1   'O'
		#define MAC_ADDR2   'R'
		#define MAC_ADDR3   'A'
		#define MAC_ADDR4   'M'
		//#define MAC_ADDR5   ((unsigned char)randNum32)
	#endif
#else
		#define MAC_ADDR0   'B'
		#define MAC_ADDR1   'P'
		#define MAC_ADDR2   'R'
		#define MAC_ADDR3   'A'
		#define MAC_ADDR4   'M'
		//#define MAC_ADDR5   'I'
#endif

//Uncomment the define below to clock the PHY from external 25MHz crystal (only for MII mode)
//Otherwise the PHY clock will be sourced from PA8 MCO1
#define USE_MCO1_FOR_PHY_CLOCK_SRC 1

#if (SWITCH_ID == SWITCH_ID_KSZ8567)
#undef USE_MCO1_FOR_PHY_CLOCK_SRC
#endif

#if (MII_RMII == RMII_MODE)
#undef USE_MCO1_FOR_PHY_CLOCK_SRC
#endif
//=========== hw config ===============

//(0) General and Clocks
#define USE_MCO1_FOR_PHY_CLOCK_SRC
//#undef USE_MCO1_FOR_PHY_CLOCK_SRC

//(1) Processor
#define PROCESSOR_STM32F407VGT6 1
#define PROCESSOR_STM32F407VZT6 2
#define PROCESSOR_STM32F401RET6 3
#define PROCESSOR_STM32F030C8   4 //TBD
#define PROCESSOR_STM32F103C8T6 5
#define PROCESSOR_STM32F103RCT6 6
#define PROCESSOR_STM32F107VCT  7
#define PROCESSOR_GD32F130FX    8
#define PROCESSOR_GD32F103C8T6  9
#define PROCESSOR PROCESSOR_STM32F103C8T6 //PROCESSOR_STM32F103RCT6 //PROCESSOR_GD32F130FX //PROCESSOR_STM32F103C8T6 //PROCESSOR_STM32F103RCT6 //

//#define USE_GD32F103C8T6        1

#if (PROCESSOR == PROCESSOR_STM32F103C8T6)
#define MCU_MODULE_VER 			35 //orginal //39 //M35 //M84
//#define MCU_MODULE_VER 			37 //35 //orginal //39 //M35
//#define MCU_MODULE_VER 				78
#elif (PROCESSOR == PROCESSOR_STM32F103RCT6)
#define MCU_MODULE_VER 				79
#else
#endif
//(2) Debug Console
#define IS_USB   		1
#define IS_USART 		2
#define DEBUGG_CONSOLE 	IS_USART //IS_USB //IS_USART //IS_USB

#define USART1_FOR_NONE 0
#define USART1_FOR_GPS  1
#define USART1_FOR_LIN  2
#define USART1_FOR USART1_USART1_FOR_NONE //

#define USART2_FOR_NONE 0
#define USART2_FOR_GPS  1
#define USART2_FOR_LIN  2
#define USART2_FOR USART2_FOR_LIN//USART2_FOR_GPS//USART2_FOR_GPS //

#if (PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6)
//#define DEBUG_UART_INDEX  1//org
//#define DEBUG_UART_INDEX  3 //-- BLDC Module //2//3//2//1// 6
#define DEBUG_UART_INDEX  1 //PA9/10
#elif (PROCESSOR == PROCESSOR_GD32F130FX)
#define DEBUG_UART_INDEX  2 //PA2 and PA3 (in GD, it is called as USART1)
#else
#define DEBUG_UART_INDEX  3//2//3//2//1// 6
#endif

#if (DEBUG_UART_INDEX == 1)
	#define USARTX USART1
#elif (DEBUG_UART_INDEX == 2)
#define USARTX USART2
#elif (DEBUG_UART_INDEX == 3)
	#if (PROCESSOR == PROCESSOR_STM32F103C8T6)
		#define USARTX USART3
	#elif (PROCESSOR == PROCESSOR_STM32F103RCT6)
		#define USARTX USART3 //UART4
	#endif
#elif (DEBUG_UART_INDEX == 6)
	#define USARTX USART6
#endif


//========= MDIO/cmdline ================
#define MDIO_FOR    	MDIO_FOR_EDU//GENERALPHY
//#define MDIO_FOR   	MDIO_FOR_NXP //REALTEK9K
//#define MDIO_FOR 		MDIO_FOR_RTL9047SWITCH //RTL9K

#define CMDLINE_MDIO 		1 	//BitBang
#define CMDLINE_RSTP 		2
#define CMDLINE_MDIO407_107 3
#define CMDLINE_SJA1105 	4

#define CMDLINE_USE_FOR CMDLINE_MDIO //CMDLINE_MDIO407_107 //CMDLINE_SJA1105 ////CMDLINE_SJA1105 //CMDLINE_MDIO407 //CMDLINE_RSTP //

#define stmUART_RBUF_SIZE 256

struct usartbuf_st{
	unsigned int in; //next in index
	unsigned int out;//next out index
	char prevchar;
	char term; //0x0d/0x0a detected
	char buf[stmUART_RBUF_SIZE];
};

// for printf
struct _usartXrxbuf{
	char buf[stmUART_RBUF_SIZE];
	char pos;
};
struct _usart1rxbuf{
	char buf[stmUART_RBUF_SIZE];
	char pos;
};
struct _usart2rxbuf{
	char buf[stmUART_RBUF_SIZE];
	char pos;
};
struct _usart3rxbuf{
	char buf[stmUART_RBUF_SIZE];
	char pos;
};

//(1) Audio
//CODEC
#define VOLUP 1
#define VOLDN 2
#define AUD_MUTE  4

#define MAX_HPOUT_VOL  0x7F  //7bits
#define MAX_LINEIN_VOL 0x1F  //5bits

#define AUDIO_USELINEINPUT 	1
#define AUDIO_USEMICINPUT 	2
#define AUDIO_BYPASSENABLED 1
#define AUDIO_BYPASSDISABLED 0

//(3) DISPLAY
#define USE_DISPLAY_NONE        0
#define USE_DISPLAY_MAX7219 	1
#define USE_DISPLAY_SAA1064 	2
#define USE_DISPLAY_OLED    	3
#define USE_DSPLAY_HD44780_CLCD 4
#define USE_DISPLAY_LCD     	5
#define USE_DISPLAY 			USE_DISPLAY_LCD//USE_DISPLAY_OLED//USE_DISPLAY_OLED

//(4) I2C
#define 	I2C_Direction_Transmitter   ((uint8_t)0x00)
#define 	I2C_Direction_Receiver   	((uint8_t)0x01)
//#define 	IS_I2C_DIRECTION(DIRECTION)
#define 	I2C_SPEED 					100000//400000 //400Kbps

//(5) SPI
//which SPI Modules
#define USE_SPI1 1
#define USE_SPI2 2
#define USE_SPI3 3

#if (PROCESSOR == PROCESSOR_STM32F103C8T6)
#define USE_SPIX USE_SPI2 //-STM32F103C8T6
#elif (PROCESSOR == PROCESSOR_GD32F130FX)
#define USE_SPIX USE_SPI1
#elif (PROCESSOR == PROCESSOR_STM32F103RCT6)
#define USE_SPIX USE_SPI2
#else
#define USE_SPIX USE_SPI1
#endif

//SPI Modes
#define stmSPIMODE0 0
#define stmSPIMODE1 1
#define stmSPIMODE2 2
#define stmSPIMODE3 3

#define stmSPIDATA8 8
#define stmSPIDATA16 16

//(6) IRQ

#define USE_EXTI0_LIS3DH 		1
#define USE_EXTI0_QEI           2
#define USE_EXTI0 				USE_EXTI0_QEI


#define USE_EXTI3_LIS3DH 		1
#define USE_EXTI3_GPSPPS 		2
#define USE_EXTI3_QEI_IDX 		3
#define USE_EXTI3 				USE_EXTI3_QEI_IDX

#define USE_EXTI4_LIS3MDL 		1
#define USE_EXTI4_GPSPPS 		2
#define USE_EXTI4_QEI           3
#define USE_EXTI4 				USE_EXTI4_QEI //USE_EXTI4_GPSPPS //USE_EXTI4_LIS3MDL//0 //USE_EXTI4_GPSPPS //

#define USE_EXTI9_5_AP3216 		1
#define USE_EXTI9_5_APDS9960 	2
#define USE_EXTI9_5_RTC 		3
#define USE_EXTI9_5_LIS3MDL_RDY	4
#define USE_EXTI9_5 			USE_EXTI9_5_LIS3MDL_RDY //USE_EXTI9_5_APDS9960//USE_EXTI9_5_RTC // USE_EXTI9_5_AP3216//


//PC13
#define USE_EXTI15_10_ETHER 		1
#define USE_EXTI15_10_BLINK 		2
#define USE_EXTI15_10_AP3216 		3
#define USE_EXTI15_10_VL53L0X 		4
#define USE_EXTI15_10_433MHz 		5
#define USE_EXTI15_10_TCA8418_KEYPAD 		6
#define USE_EXTI15_10 USE_EXTI15_10_TCA8418_KEYPAD

//(7) Ticks
//Ticks
//Define how many clock ticks in one second.
//Note:  This should match the value of SYSTICKHZ in the main program.
#define CLOCK_CONF_SECOND       1000//1msec per tick...
#define SYSTICKHZ               CLOCK_CONF_SECOND //1000 -- WAS 100 (ORG)
#define SYSTICKMS               (1000 / SYSTICKHZ) //1
#define SYSTICKUS               (1000000 / SYSTICKHZ) //1000
#define SYSTICKNS               (1000000000 / SYSTICKHZ) //100000


//(8) Display on which?
#define DISPLAY_ON_NONE    		0
#define DISPLAY_ON_CONSOLE 		1
#define DISPLAY_ON_MAX7219_SPI 	2
#define DISPLAY_ON_OLED_I2C 	3
#define DISPLAY_ON 				DISPLAY_ON_CONSOLE //DISPLAY_ON_OLED_I2C

//(9) Which Motors
#define MOTOR_FOR_DRV11873 		1
#define MOTOR_FOR_DRV8313		2
#define MOTOR_FOR_DRV8833		3
#define MOTOR_FOR_A3967			4
#define MOTOR_FOR_A4988			5
#define MOTOR_FOR_DRV8825		6
#define MOTOR_FOR_VID29 		7
#define MOTOR_FOR_MCP8063		8
#define MOTOR_FOR_TB6612		9
#define MOTOR_FOR_SERVO			10
#define MOTOR_FOR 				MOTOR_FOR_SERVO //MOTOR_FOR_A3967 //MOTOR_FOR_DRV8825 //MOTOR_FOR_DRV8313 //MOTOR_FOR_DRV11873//MOTOR_FOR_SERVO //MOTOR_FOR_VID29

//==================== Useful Macros ==================
typedef enum {FAILED = 0, PASSED = !FAILED} eResultStatus;

#define NAN (0.0/0.0)

#define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)
//For Arduino
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define _BV(bit) (1 << (bit))

//=================== time struct ==============
typedef struct
{
    //! The number of years since 0 AD.
    unsigned short usYear;
    //! The month, where January is 0 and December is 11.
    unsigned char ucMon;
    //! The day of the month.
    unsigned char ucMday;
    //! The day of the week, where Sunday is 0 and Saturday is 6.
    unsigned char ucWday;
    //! The number of hours.
    unsigned char ucHour;
    //! The number of minutes.
    unsigned char ucMin;
    //! The number of seconds.
    unsigned char ucSec;
}tTod;

//    @brief  Simple general-purpose date/time class (no TZ / DST / leap second handling!).
//            See http://en.wikipedia.org/wiki/Leap_second
struct _DateTime {
  unsigned short yOff; //to be +2000
  unsigned char m;
  unsigned char date;
  unsigned char hh;
  unsigned char mm;
  unsigned char ss;
  //==
  unsigned short year;
  unsigned char  day; //Monday...
};

typedef struct
{
    unsigned char year;//since 1970
    //! The month, where January is 0 and December is 11.
    unsigned char month;
    //! The day of the month.
    unsigned char day;
    //! The day of the week, where Sunday is 0 and Saturday is 6.
    unsigned char weekday;
    unsigned char pm; //am=0; pm=1
    //! The number of hours.
    unsigned char hour;
    //! The number of minutes.
    unsigned char minute;
    //! The number of seconds.
    unsigned char second;
}tm_t; //arduino


//Constants
#define SECONDS_PER_DAY       86400L  ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000 946684800  ///< Unixtime for 2000-01-01 00:00:00, useful for initialization


#include <stdbool.h>
//typedef unsigned char u8;
//typedef char Octet;
//typedef short int s16;
//typedef unsigned short int u16;
//typedef u8 bool;
//typedef bool boolean;
//typedef bool Boolean;
//typedef u16 uint16_t;
//typedef u8 uint8_t;
//typedef signed char Integer8;
//typedef signed short Integer16;
//typedef signed int Integer32;
//typedef unsigned char UInteger8;
//typedef unsigned short UInteger16;
//typedef unsigned int UInteger32;
//typedef long long Integer64;
//typedef unsigned long long   UInteger64;



//gps ================= float handling ===============================

struct _ppsHoldOver{
	unsigned char firstToDSetDoneByGps;
	unsigned GpsPpsTick;
	//unsigned RtcPpsTick;
	unsigned lost_GPS_PPS;
	unsigned showTimeByRTCperSec;
	struct _DateTime DateTime;
};

struct minmea_float {
    int_least32_t value;
    int_least32_t scale;
};

struct minmea_date {
    int day;
    int month;
    int year;
};

struct minmea_time {
    int hours;
    int minutes;
    int seconds;
    int microseconds;
};

struct minmea_sentence_rmc {
    struct minmea_time time;
    bool valid;
    struct minmea_float latitude;
    struct minmea_float longitude;
    struct minmea_float speed;
    struct minmea_float course;
    struct minmea_date date;
    struct minmea_float variation;
};
struct minmea_sentence_gga {
    struct minmea_time time;
    struct minmea_float latitude;
    struct minmea_float longitude;
    int fix_quality;
    int satellites_tracked;
    struct minmea_float hdop;
    struct minmea_float altitude; char altitude_units;
    struct minmea_float height; char height_units;
    int dgps_age;
};
//=============================== externs =================================
extern void Display_String(unsigned short x, unsigned short y, char *ptr,unsigned short color); //LCD
extern void showTimeToLCD(char *str, unsigned short xpos, unsigned short ypos);
extern int hex2int(char c);
extern uint32_t randNum32;

//==== USART1 and 3
extern int hex2int(char c);


//==================
#define offsetof(TYPE, MEMBER) ((unsigned int *)&(((TYPE*)0)->MEMBER))

int ySeperateDotString(char *givenstr, unsigned char retArray[]);


//sock utils
#define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)
#define Y_MIN(x , y)  (((x) < (y)) ? (x) : (y))
#define Y_MAX(x , y)  (((x) > (y)) ? (x) : (y))


//Bit Opeatatons
#define m_bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define m_bitSet(value, bit) ((value) |=  (1UL << (bit)))
#define m_bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define m_bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define USE_SINE_TABLE 1

extern char *float2str(float x);

//#ifndef isprint
#define in_range(c, lo, up)  ((unsigned char)c >= lo && (unsigned char)c <= up)
#define isprint(c)           in_range(c, 0x20, 0x7f)
#define isdigit(c)           in_range(c, '0', '9')
#define isxdigit(c)          (isdigit(c) || in_range(c, 'a', 'f') || in_range(c, 'A', 'F'))
#define islower(c)           in_range(c, 'a', 'z')
#define isspace(c)           (c == ' ' || c == '\f' || c == '\n' || c == '\r' || c == '\t' || c == '\v')
//#endif
//============ LCD ==============
struct _TouchScreen_posxy{
	unsigned short x;
	unsigned short y;
};


#define swap(a, b) { short t = a; a = b; b = t; }

//=== QEI
struct qei_table{
	unsigned short maxval;
	unsigned short val;
	unsigned short oldVal;
	int direction; //+1,-1
	char phB_at_PhArising;
	char valid;
};
#endif
//==================
