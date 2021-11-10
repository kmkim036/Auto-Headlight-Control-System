#if 0
//ISL1208 Real-Time Clock

#include "yInc.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_STM32F103RCT6))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"
//#include "lwipopts.h"
//========== differs from DS1307 ========================
#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurst(unsigned char SlaveAddress, unsigned char *buf, unsigned char nbyte);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);

extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c

#define ISL1208_ADDR7              	(0x6F)
#define ISL1208_ADDR8              	(0xDE) //(0x6F<<1) 110-1111 -> 1101-1110
#define ISL1208_REG_TIME            (0x00)
#define ISL1208_STATUS           	(0x07)
#define ISL1208_INTSTATUS           (0x08)//#define ISL1208_REG_CONTROL         (0x07)
#define ISL1208_REG_NVRAM           (0x08)

/** DS1307 SQW pin mode settings */
//enum Ds1307SqwPinMode {
typedef enum  {
  ISL1208_OFF              = 0x00, // Low
  ISL1208_ON               = 0x80, // High
  ISL1208_SquareWave1HZ    = 0x0A, // 1Hz square wave//ISL1208_SquareWave1HZ    = 0x10, // 1Hz square wave
  ISL1208_SquareWave4kHz   = 0x11, // 4kHz square wave
  ISL1208_SquareWave8kHz   = 0x12, // 8kHz square wave
  ISL1208_SquareWave32kHz  = 0x01  // 32kHz square wave//ISL1208_SquareWave32kHz  = 0x13  // 32kHz square wave
} Ds1307SqwPinMode;

/** Constants */
#define SECONDS_PER_DAY       86400L  ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000 946684800  ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

#define ISL1208_ADDRESS        0x68  ///< I2C address for DS1307
#define ISL1208_CONTROL        0x07  ///< Control/Status registers (0x7~0xb)
#define ISL1208_ALARM          0x0C  ///< Alarm registers (0xc~0x11)
#define ISL1208_NVRAM          0x12  ///< Start of RAM registers - 16 bytes, 0x12 to 0x13

/** Constants */
#define SECONDS_PER_DAY       86400L  ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000 946684800  ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

/**************************************************************************/
/*!
    @brief  Simple general-purpose date/time class (no TZ / DST / leap second handling!).
            See http://en.wikipedia.org/wiki/Leap_second
*/
/**************************************************************************/
/*
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
*/
unsigned char DateTime_dayOfTheWeek(struct _DateTime *dt);



  boolean RTC_ISL1208_begin(void);
  static void RTC_ISL1208_adjust(struct _DateTime *dt);
  unsigned char RTC_ISL1208_isrunning(void);
  void RTC_ISL1208_now(struct _DateTime *dt);
  static Ds1307SqwPinMode RTC_ISL1208_readSqwPinMode();
  static void RTC_ISL1208_writeSqwPinMode(Ds1307SqwPinMode mode);
//  unsigned char RTC_ISL1208_readnvram(unsigned char address);
  void RTC_ISL1208_readnvram(unsigned char* buf, unsigned char size, unsigned char address);
//  void RTC_ISL1208_writenvram(unsigned char address, unsigned char data);
  void RTC_ISL1208_writenvram(unsigned char address, unsigned char* buf, unsigned char size);

char daysOfTheWeek [7][14] = {"Monday","Tuesday","Wednesday", "Thursday","Friday", "Saturday", "Sunday"};
/**
  Number of days in each month, from January to November. December is not
  needed. Omitting it avoids an incompatibility with Paul Stoffregen's Time
  library. C.f. https://github.com/adafruit/RTClib/issues/114
*/
const unsigned char daysInMonth []   = { 31,28,31,30,31,30,31,31,30,31,30 };
//============ i2c ==================
void ISL1208_writeRegister8(unsigned char reg, unsigned char value)
{
    stmI2cSendbuf[0] = reg;
    stmI2cSendbuf[1] = value; //a byte
    return stm_I2C_SendBurst(ISL1208_ADDR8, stmI2cSendbuf, 2);
}

unsigned char ISL1208_readRegister8(unsigned char reg, unsigned char *rdata)
{
    unsigned char value, retlen=0;
		// read the current GPIO input
    retlen = stm_I2C_ReceiveBurstWithRestartCondition(ISL1208_ADDR8, reg, stmI2cRecvbuf, 1);
	if(retlen==0){
		printf("i2c Rd8 error\r\n");
		return 0;
	}
    *rdata  = stmI2cRecvbuf[0];
    return retlen;
}


/**************************************************************************/
/*!
    @brief  Given a date, return number of days since 2000/01/01, valid for 2001..2099
    @param y Year
    @param m Month
    @param d Day
    @return Number of days
*/
/**************************************************************************/
static unsigned short date2days(unsigned short y, unsigned char m, unsigned char d) {
	unsigned char i;
    if (y >= 2000)
        y -= 2000;
    unsigned short days = d;
    for (i = 1; i < m; ++i)
        days += daysInMonth[i - 1];//pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

/**************************************************************************/
/*!
    @brief  Given a number of days, hours, minutes, and seconds, return the total seconds
    @param days Days
    @param h Hours
    @param m Minutes
    @param s Seconds
    @return Number of seconds total
*/
/**************************************************************************/
static long time2long(unsigned short days, unsigned char h, unsigned char m, unsigned char s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

/**************************************************************************/
/*!
    @brief  DateTime constructor from unixtime
    @param t Initial time in seconds since Jan 1, 1970 (Unix time)
*/
/**************************************************************************/
DateTime_Time (struct _DateTime *dt, unsigned int t) {
  t -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970

  dt->ss = t % 60;
  t /= 60;
  dt->mm = t % 60;
  t /= 60;
  dt->hh = t % 24;
  unsigned short days = t / 24;
  unsigned char leap;
  for (dt->yOff = 0; ; ++dt->yOff) {
	  leap = dt->yOff % 4 == 0;
	  if (days < 365 + leap)
		  break;
	  days -= 365 + leap;
  }
  for (dt->m = 1; dt->m < 12; ++dt->m) {
    unsigned char daysPerMonth = daysInMonth[dt->m - 1];//pgm_read_byte(daysInMonth + dt->m - 1);
    if (leap && dt->m == 2)
      ++daysPerMonth;
    if (days < daysPerMonth)
      break;
    days -= daysPerMonth;
  }
  dt->date = days + 1;
}

/**************************************************************************/
/*!
    @brief  DateTime constructor from Y-M-D H:M:S
    @param year Year, 2 or 4 digits (year 2000 or higher)
    @param month Month 1-12
    @param day Day 1-31
    @param hour 0-23
    @param min 0-59
    @param sec 0-59
*/
/**************************************************************************/
DateTime_DateTime (struct _DateTime *dt, unsigned short year, unsigned char month, unsigned char day, unsigned char hour, unsigned char min, unsigned char sec) {
    if (year >= 2000)
    	dt->year -= 2000;
    dt->yOff = year;
    dt->m = month;
    dt->date = day;
    dt->hh = hour;
    dt->mm = min;
    dt->ss = sec;
}
/**************************************************************************/
/*!
    @brief  Convert a string containing two digits to unsigned char, e.g. "09" returns 9
    @param p Pointer to a string containing two digits
*/
/**************************************************************************/
static unsigned char conv2d(const char* p) {
    unsigned char v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

/**************************************************************************/
/*!
    @brief  A convenient constructor for using "the compiler's time":
            DateTime now (__DATE__, __TIME__);
            NOTE: using F() would further reduce the RAM footprint, see below.
    @param date Date string, e.g. "Dec 26 2009"
    @param time Time string, e.g. "12:34:56"
*/
/**************************************************************************/
DateTime_DateTimeConfig (struct _DateTime *dt, const char* date, const char* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
	dt->yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (date[0]) {
        case 'J': dt->m = (date[1] == 'a') ? 1 : ((date[2] == 'n') ? 6 : 7); break;
        case 'F': dt->m = 2; break;
        case 'A': dt->m = date[2] == 'r' ? 4 : 8; break;
        case 'M': dt->m = date[2] == 'r' ? 3 : 5; break;
        case 'S': dt->m = 9; break;
        case 'O': dt->m = 10; break;
        case 'N': dt->m = 11; break;
        case 'D': dt->m = 12; break;
    }
    dt->date = conv2d(date + 4);
    dt->hh = conv2d(time);
    dt->mm = conv2d(time + 3);
    dt->ss = conv2d(time + 6);
}
#if 0
/**************************************************************************/
/*!
    @brief  A convenient constructor for using "the compiler's time":
            This version will save RAM by using   to store it by using the F macro.
            DateTime now (F(__DATE__), F(__TIME__));
    @param date Date string, e.g. "Dec 26 2009"
    @param time Time string, e.g. "12:34:56"
*/
/**************************************************************************/
DateTime_DateTime (const __FlashStringHelper* date, const __FlashStringHelper* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    char buff[11];
    memcpy_P(buff, date, 11);
    yOff = conv2d(buff + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (buff[0]) {
        case 'J': m = (buff[1] == 'a') ? 1 : ((buff[2] == 'n') ? 6 : 7); break;
        case 'F': m = 2; break;
        case 'A': m = buff[2] == 'r' ? 4 : 8; break;
        case 'M': m = buff[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(buff + 4);
    memcpy_P(buff, time, 8);
    hh = conv2d(buff);
    mm = conv2d(buff + 3);
    ss = conv2d(buff + 6);
}
#endif
/**************************************************************************/
/*!
    @brief  Return DateTime in based on user defined format.
    @param buffer: array of char for holding the format description and the formatted DateTime.
                   Before calling this method, the buffer should be initialized by the user with
                   a format string, e.g. "YYYY-MM-DD hh:mm:ss". The method will overwrite
                   the buffer with the formatted date and/or time.
    @return a pointer to the provided buffer. This is returned for convenience,
            in order to enable idioms such as Serial.println(now.toString(buffer));
*/
/**************************************************************************/

char* DateTime_toString(struct _DateTime *dt, char* buffer){
	int i;
		for(i=0;i<strlen(buffer)-1;i++){
		if(buffer[i] == 'h' && buffer[i+1] == 'h'){
			buffer[i] = '0'+dt->hh/10;
			buffer[i+1] = '0'+dt->hh%10;
		}
		if(buffer[i] == 'm' && buffer[i+1] == 'm'){
			buffer[i] = '0'+dt->mm/10;
			buffer[i+1] = '0'+dt->mm%10;
		}
		if(buffer[i] == 's' && buffer[i+1] == 's'){
			buffer[i] = '0'+dt->ss/10;
			buffer[i+1] = '0'+dt->ss%10;
		}
    if(buffer[i] == 'D' && buffer[i+1] =='D' && buffer[i+2] =='D'){
      static   const char day_names[] = "SunMonTueWedThuFriSat";
      const char *p = &day_names[3*DateTime_dayOfTheWeek(dt)];
      buffer[i] = p[0];//pgm_read_byte(p);
      buffer[i+1] = p[1];//pgm_read_byte(p+1);
      buffer[i+2] = p[2];//pgm_read_byte(p+2);
    }else
		if(buffer[i] == 'D' && buffer[i+1] == 'D'){
			buffer[i] = '0'+dt->date/10;
			buffer[i+1] = '0'+dt->date%10;
		}
    if(buffer[i] == 'M' && buffer[i+1] =='M' && buffer[i+2] =='M'){
      static   const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
      const char *p = &month_names[3*(dt->m-1)];
      buffer[i] = p[0];//pgm_read_byte(p);
      buffer[i+1] = p[1];//pgm_read_byte(p+1);
      buffer[i+2] = p[2];//pgm_read_byte(p+2);
    }else
		if(buffer[i] == 'M' && buffer[i+1] == 'M'){
			buffer[i] = '0'+dt->m/10;
			buffer[i+1] = '0'+dt->m%10;
		}
		if(buffer[i] == 'Y'&& buffer[i+1] == 'Y'&& buffer[i+2] == 'Y'&& buffer[i+3] == 'Y'){
			buffer[i] = '2';
			buffer[i+1] = '0';
			buffer[i+2] = '0'+(dt->yOff/10)%10;
			buffer[i+3] = '0'+dt->yOff%10;
		}else
		if(buffer[i] == 'Y'&& buffer[i+1] == 'Y'){
			buffer[i] = '0'+(dt->yOff/10)%10;
			buffer[i+1] = '0'+dt->yOff%10;
		}

	}
	return buffer;
}

/**************************************************************************/
/*!
    @brief  Return the day of the week for this object, from 0-6.
    @return Day of week 0-6 starting with Sunday, e.g. Sunday = 0, Saturday = 6
*/
/**************************************************************************/
unsigned char DateTime_dayOfTheWeek(struct _DateTime *dt) {
    unsigned short day = date2days(dt->yOff, dt->m, dt->date);
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

/**************************************************************************/
/*!
    @brief  Return unix time, seconds since Jan 1, 1970.
    @return Number of seconds since Jan 1, 1970
*/
/**************************************************************************/
unsigned int DateTime_unixtime(struct _DateTime *dt)
{
  unsigned int t;
  unsigned short days = date2days(dt->yOff, dt->m, dt->date);
  t = time2long(days, dt->hh, dt->mm, dt->ss);
  t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000

  return t;
}

/**************************************************************************/
/*!
    @brief  Convert the DateTime to seconds
    @return The object as seconds since 2000-01-01
*/
/**************************************************************************/
long DateTime_secondstime(struct _DateTime *dt)  {
  long t;
  unsigned short days = date2days(dt->yOff, dt->m, dt->date);
  t = time2long(days, dt->hh, dt->mm, dt->ss);
  return t;
}


/**************************************************************************/
/*!
    @brief  Convert a binary coded decimal value to binary. RTC stores time/date values as BCD.
    @param val BCD value
    @return Binary value
*/
/**************************************************************************/
static unsigned char bcd2bin (unsigned char val) { return val - 6 * (val >> 4); }

/**************************************************************************/
/*!
    @brief  Convert a binary value to BCD format for the RTC registers
    @param val Binary value
    @return BCD value
*/
/**************************************************************************/
static unsigned char bin2bcd (unsigned char val) { return val + 6 * (val / 10); }

/**************************************************************************/
/*!
    @brief  Startup for the DS1307
    @return Always true
*/
/**************************************************************************/
boolean RTC_ISL1208_begin(void) {
  //i2c config
  return true;
}

/**************************************************************************/
/*!
    @brief  Is the DS1307 running? Check the Clock Halt bit in register 0
    @return 1 if the RTC is running, 0 if not
*/
/**************************************************************************/

unsigned char RTC_ISL1208_isrunning(void) {
	unsigned char ss;
	int i;
/*
	while(1){
	for(i=7;i<0x13;i++){
		ISL1208_readRegister8(i,&ss);
		printf("val=%02x\r\n", ss);
		delayms(100);
	}

		delayms(500);
	}
*/
	ISL1208_readRegister8(ISL1208_STATUS,&ss);//ISL1208_REG_CONTROL,&ss); //get WRTC: 1= running; 0=stop

	return (ss & 0x10);
}

/**************************************************************************/
/*!
    @brief  Set the date and time in the DS1307
    @param dt DateTime object containing the desired date/time
*/
/**************************************************************************/
void RTC_ISL1208_adjust(struct _DateTime *dt) {

	  //ISL1208_setDateTime_DateTime("Feb 05 20", "17:36:38");//ISL1208_setDateTime_DateTime("Feb 05 2020", "17:36:38");
	  //make CH bit 0
		stmI2cSendbuf[0] = ISL1208_REG_TIME;
		stmI2cSendbuf[1] = bin2bcd(dt->ss);
		stmI2cSendbuf[2] = bin2bcd(dt->mm);
		stmI2cSendbuf[3] = bin2bcd(dt->hh);
		stmI2cSendbuf[4] = bin2bcd(0);
		stmI2cSendbuf[5] = bin2bcd(dt->date);
		stmI2cSendbuf[6] = bin2bcd(dt->m);
		stmI2cSendbuf[7] = bin2bcd(dt->yOff);

	  ISL1208_writeRegister8(0x00, 0x00);
	  stm_I2C_SendBurst(ISL1208_ADDR8, stmI2cSendbuf, 8); //write reg.

}

/**************************************************************************/
/*!
    @brief  Get the current date and time from the DS1307
    @return DateTime object containing the current date and time
*/
/**************************************************************************/
//struct _DateTime* RTC_ISL1208_now(struct _DateTime *dt) {
void RTC_ISL1208_now(struct _DateTime *dt) { //ISL1208_getDateTime()

    unsigned char retlen;

    retlen = stm_I2C_ReceiveBurstWithRestartCondition(ISL1208_ADDR8, ISL1208_REG_TIME, stmI2cRecvbuf, 7); //Reg0~6
	if(retlen==0){
		printf("i2c Rd error\r\n");
		return 0;
	}

    dt->ss = bcd2bin(stmI2cRecvbuf[0] & 0x7F);
    dt->mm = bcd2bin(stmI2cRecvbuf[1]);
    dt->hh = bcd2bin(stmI2cRecvbuf[2]);
    //skip 3 (Day: Mon, ...)
    dt->date = bcd2bin(stmI2cRecvbuf[4]);
    dt->m = bcd2bin(stmI2cRecvbuf[5]);
    dt->yOff = bcd2bin(stmI2cRecvbuf[6]);
}

/**************************************************************************/
/*!
    @brief  Read the current mode of the SQW pin
    @return Mode as Ds1307SqwPinMode enum
*/
/**************************************************************************/
Ds1307SqwPinMode RTC_ISL1208_readSqwPinMode() {
  unsigned char mode;

  ISL1208_readRegister8(ISL1208_INTSTATUS, &mode);//ISL1208_REG_CONTROL, &mode);
  //mode &= 0x93;
  return mode;//static_cast<Ds1307SqwPinMode>(mode);
}

/**************************************************************************/
/*!
    @brief  Change the SQW pin mode
    @param mode The mode to use
*/
/**************************************************************************/
void RTC_ISL1208_writeSqwPinMode(Ds1307SqwPinMode mode) {
	ISL1208_writeRegister8(ISL1208_INTSTATUS, mode);//ISL1208_REG_CONTROL, mode);
}

/**************************************************************************/
/*!
    @brief  Read data from the DS1307's NVRAM (from Reg8..)
    @param buf Pointer to a buffer to store the data - make sure it's large enough to hold size bytes
    @param size Number of bytes to read
    @param address Starting NVRAM address, from 0 to 55
*/
/**************************************************************************/
void RTC_ISL1208_readnvram(unsigned char* buf, unsigned char size, unsigned char address) {
  int addrByte = ISL1208_NVRAM + address;
  unsigned char retlen;
  unsigned char pos;
  retlen = stm_I2C_ReceiveBurstWithRestartCondition(ISL1208_ADDR8, addrByte, stmI2cRecvbuf, size);
	if(retlen==0){
		printf("i2c Rd error\r\n");
		return 0;
	}
  for (pos = 0; pos < size; ++pos) {
    buf[pos] = stmI2cRecvbuf[pos];
  }
}

/**************************************************************************/
/*!
    @brief  Write data to the DS1307 NVRAM
    @param address Starting NVRAM address, from 0 to 55
    @param buf Pointer to buffer containing the data to write
    @param size Number of bytes in buf to write to NVRAM
*/
/**************************************************************************/
void RTC_ISL1208_writenvram(unsigned char address, unsigned char* buf, unsigned char size) {
  int addrByte = ISL1208_NVRAM + address;
  int pos;
  for(pos=0;pos<size;pos++)
	  stmI2cSendbuf[pos] = buf;
  ISL1208_writeRegister8(0x00, addrByte);
  stm_I2C_SendBurst(ISL1208_ADDR8, stmI2cSendbuf, size);
}

/**************************************************************************/
/*!
    @brief  Shortcut to read one byte from NVRAM
    @param address NVRAM address, 0 to 55
    @return The byte read from NVRAM
*/
/**************************************************************************/
unsigned char RTC_ISL1208_readnvram_single(unsigned char address) {
  unsigned char data;
  RTC_ISL1208_readnvram(&data, 1, address);
  return data;
}

/**************************************************************************/
/*!
    @brief  Shortcut to write one byte to NVRAM
    @param address NVRAM address, 0 to 55
    @param data One byte to write
*/
/**************************************************************************/
void RTC_ISL1208_writenvram_single(unsigned char address, unsigned char data) {
	RTC_ISL1208_writenvram(address, &data, 1);
}

void ISL1208_setup(struct _DateTime *dt)
{
  // Initialize DS1307
  printf("Initialize ISL1208 RTC\r\n");

  // If date not set
  if (!RTC_ISL1208_isrunning()) //==1 --> Osc Not Running.... Clock Hold...
  {
	  printf("is1208 is fresh\r\n");

	  //write WRTC=1
	  ISL1208_writeRegister8(0x07, 0x10);//WRTC=1
	  printf("Start is1208\r\n");
	  //set time
	  RTC_ISL1208_adjust(dt);
	  //ISL1208_setDateTime_DateTime("Feb 05 20", "17:36:38");//ISL1208_setDateTime_DateTime("Feb 05 2020", "17:36:38");
	  //ISL1208_setDateTime_DateTime(__DATE__, __TIME__);
  }else{  //1 --> Osc is running.
	  printf("isl208 is running\r\n");
  }
}


//================================================================================
void ISL1208rtc_loop()
{
	struct _DateTime  DateTime;
	printf("isl1208 Loop\r\n");
	printf("isl1208> I2C Init (8-bit Address = 0x%02x) ...", ISL1208_ADDR8);
	if(!g_bI2CModuleConfigDone){
#if (PROCESSOR == STM32F407VGT6)
		gI2Cx = I2C1;
#elif (PROCESSOR == STM32F407VZT6)
		gI2Cx = I2C2;
#endif
		stmI2cModuleConfig(gI2Cx,400000);//400Kbps
	}
	printf("Done.\r\n");

	delayms(100);
	printf("isl1208> Init\r\n");

	//set time
	DateTime.yOff = 20;
	DateTime.m = 2;
	DateTime.date = 6;
	DateTime.day = 3;
	DateTime.hh = 9;
	DateTime.mm = 43;
	DateTime.ss = 20;
	ISL1208_setup(&DateTime);

	//SQ -// Enable output as rate to 1Hz
	printf("PPS out from SQ pin\r\n");
	RTC_ISL1208_writeSqwPinMode (ISL1208_SquareWave1HZ);//ISL1208_SquareWave32kHz);//ISL1208_setOutput(ISL1208_1HZ);
/*
	switch (ISL1208_getOutput())
	{
		case ISL1208_LOW:     printf("SQW = LOW"); break;
		case ISL1208_HIGH:    printf("SQW = HIGH"); break;
		case ISL1208_1HZ:     printf("SQW = 1Hz"); break;
		case ISL1208_4096HZ:  printf("SQW = 4096Hz"); break;
		case ISL1208_8192HZ:  printf("SQW = 8192Hz"); break;
		case ISL1208_32768HZ: printf("SQW = 32768Hz"); break;
		default: printf("SQW = Unknown"); break;
	}
*/

  // For leading zero look to ISL1208_dateformat example
	while(1){

		RTC_ISL1208_now(&DateTime);

		printf("NowTime = ");
		printf("%u-",DateTime.yOff + 2000);
		printf("%u-",DateTime.m);
		printf("%u ",DateTime.date);
		printf("(%s) ", daysOfTheWeek[DateTime_dayOfTheWeek(&DateTime)]);
		printf("%u:",DateTime.hh);
		printf("%u:",DateTime.mm);
		printf("%u\r\n",DateTime.ss);
		//=== unix time since 1970.1.1 midnight
		printf("UnixTime= %us\r\n", DateTime_unixtime(&DateTime) );


		delayms(1000);
	}
}


#endif
