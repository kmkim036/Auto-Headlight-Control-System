
//DS3231 for the DS3231 Real-Time Clock

/**************************************************************************/
/*!
  This is a fork of JeeLab's fantastic real time clock library for Arduino.
  For details on using this library with an RTC module like the DS3231, PCF8523, or DS3231,
  see the guide at: https://learn.adafruit.com/DS3231-real-time-clock-breakout-board-kit/overview
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  @section classes Available classes

  This library provides the following classes:

  - Classes for manipulating dates, times and durations:
    - DateTime represents a specific point in time; this is the data
      type used for setting and reading the supported RTCs
    - TimeSpan represents the length of a time interval
  - Interfacing specific RTC chips:
    - RTC_DS3231
    - RTC_DS3231
    - RTC_PCF8523
  - RTC emulated in software; do not expect much accuracy out of these:
    - RTC_Millis is based on `millis()`
    - RTC_Micros is based on `micros()`; its drift rate can be tuned by
      the user

  @section license License

  Original library by JeeLabs http://news.jeelabs.org/code/, released to the public domain.

  This version: MIT (see LICENSE)
*/
/**************************************************************************/
#if 1
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

#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurst(unsigned char SlaveAddress, unsigned char *buf, unsigned char nbyte);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);

extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;
extern unsigned char stmI2cSendbuf[16]; //i2c
extern unsigned char stmI2cRecvbuf[16]; //i2c

#define DS3231_ADDR7        		0x68  ///< I2C address for DS3231
#define DS3231_ADDR8              	(0xD0) //(0x68<<1)

//DS3231 REGISTERS
#define DS3231_REG_TIME             0x00 //~0x06
#define DS3231_REG_CONTROL        	0x0E  ///< Control register
#define DS3231_REG_STATUS      		0x0F  ///< Status register
#define DS3231_REG_TEMPERATURE		0x11  ///< Temperature register (high byte - low byte is at 0x12), 10-bit temperature value

/** DS3231 SQW pin mode settings */
typedef enum  {
	  DS3231_OFF            = 0x01, // Off ???
	  DS3231_SquareWave1Hz  = 0x00, // 1Hz square wave //RS2/1=00; INTCN=0
	  DS3231_SquareWave1kHz = 0x08, // 1kHz square wave//RS2/1=01; INTCN=0
	  DS3231_SquareWave4kHz = 0x10, // 4kHz square wave//RS2/1=10; INTCN=0
	  DS3231_SquareWave8kHz = 0x18  // 8kHz square wave//RS2/1=11; INTCN=0
} DS3231SqwPinMode;

unsigned char DateTime_dayOfTheWeek(struct _DateTime *dt);

boolean RTC_DS3231_begin(void);
void RTC_DS3231_Adjust(struct _DateTime *dt);
unsigned char RTC_DS3231_isrunning(void);
void RTC_DS3231_now(struct _DateTime *dt);
DS3231SqwPinMode RTC_DS3231_readSqwPinMode();
  //static void RTC_DS3231_writeSqwPinMode(DS3231SqwPinMode mode);
//  unsigned char RTC_DS3231_readnvram(unsigned char address);
  void RTC_DS3231_readnvram(unsigned char* buf, unsigned char size, unsigned char address);
//  void RTC_DS3231_writenvram(unsigned char address, unsigned char data);
  void RTC_DS3231_writenvram(unsigned char address, unsigned char* buf, unsigned char size);

char daysOfTheWeek [7][14] = {"Monday","Tuesday","Wednesday", "Thursday","Friday", "Saturday", "Sunday"};
/**
  Number of days in each month, from January to November. December is not
  needed. Omitting it avoids an incompatibility with Paul Stoffregen's Time
  library. C.f. https://github.com/adafruit/RTClib/issues/114
*/
const unsigned char daysInMonth []   = { 31,28,31,30,31,30,31,31,30,31,30 };

//============ i2c ==================
void DS3231_writeRegister8(unsigned char reg, unsigned char value)
{
    stmI2cSendbuf[0] = reg;
    stmI2cSendbuf[1] = value; //a byte
    return stm_I2C_SendBurst(DS3231_ADDR8, stmI2cSendbuf, 2);
}

unsigned char DS3231_readRegister8(unsigned char reg, unsigned char *rdata)
{
    unsigned char value, retlen=0;
		// read the current GPIO input
    retlen = stm_I2C_ReceiveBurstWithRestartCondition(DS3231_ADDR8, reg, stmI2cRecvbuf, 1);
	if(retlen==0){
		printf("i2c Rd8 error\r\n");
		return 0;
	}
    *rdata  = stmI2cRecvbuf[0];
    return retlen;
}
#if 0
/**************************************************************************/
/*!
    @brief  Read a byte from an I2C register
    @param addr I2C address
    @param reg Register address
    @return Register value
*/
/**************************************************************************/
static unsigned char read_i2c_register(unsigned char addr, unsigned char reg) {
  Wire.beginTransmission(addr);
  Wire._I2C_WRITE((byte)reg);
  Wire.endTransmission();

  Wire.requestFrom(addr, (byte)1);
  return Wire._I2C_READ();
}

/**************************************************************************/
/*!
    @brief  Write a byte to an I2C register
    @param addr I2C address
    @param reg Register address
    @param val Value to write
*/
/**************************************************************************/
static void write_i2c_register(unsigned char addr, unsigned char reg, unsigned char val) {
  Wire.beginTransmission(addr);
  Wire._I2C_WRITE((byte)reg);
  Wire._I2C_WRITE((byte)val);
  Wire.endTransmission();
}

#endif



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
DateTime_Time (struct _DateTime *dt, unsigned int t)
{
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
#if 0
/**************************************************************************/
/*!
    @brief  DateTime copy constructor using a member initializer list
    @param copy DateTime object to copy
*/
/**************************************************************************/
DateTime_InitDateTime (struct _DateTime *copy):
  yOff(copy->yOff),
  m(copy.m),
  d(copy.d),
  hh(copy.hh),
  mm(copy.mm),
  ss(copy.ss)
{}
#endif
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


char *getStr_DaysOfTheWeek(struct _DateTime *dt)
{
	return daysOfTheWeek[DateTime_dayOfTheWeek(dt)];
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
    @brief  Startup for the DS3231
    @return Always true
*/
/**************************************************************************/
boolean RTC_DS3231_begin(void) {
  //i2c config
  return true;
}

/**************************************************************************/
/*!
    @brief  Is the DS3231 running? Check the Clock Halt bit in register 0
    @return 1 if the RTC is running, 0 if not
*/
/**************************************************************************/

unsigned char RTC_DS3231_isrunning(void) {
	unsigned char ss,ss2;
	int i;

	//for(i=0;i<0x12;i++){	DS3231_readRegister8(i,&ss);	printf("val=%02x\r\n", ss);	delayms(100);	}

	DS3231_readRegister8(DS3231_REG_STATUS,&ss);

	if(ss & 0x80) printf("DS3231 is STOP\r\n"); //bit 7 = OscStopFlag (OSF).
	else printf("DS3231 is RUNNING\r\n");

	//DS3231_REG_CONTROL(0xE) = nEOSC,...
	//DS3231_REG_STATUS (0xF) = OSF,...
	DS3231_readRegister8(DS3231_REG_CONTROL,&ss2);//nEnableOSC (bit7 of controlReg 0E) = 1 when the DS3231 switches to VBAT. nEOSC bit = 0 in batterybacked mode.
	printf("DS3231 0xE reg = %02x\r\n",ss2);

	return !(ss>>7); //return OSF: Oscillator Stop Flag (1=First time power is applied; VCC/VBAT is too low.

}

#if 0
/**************************************************************************/
/*!
    @brief  Check the status register Oscillator Stop Flag to see if the DS3231 stopped due to power loss
    @return True if the bit is set (oscillator stopped) or false if it is running
*/
/**************************************************************************/
unsigned char RTC_DS3231_isLostPower(void)
{
	unsigned char ss;
	DS3231_readRegister8(DS3231_REG_STATUS,&ss);
	return(ss >> 7);
}
#endif
/**************************************************************************/
/*!
    @brief  Set the date and time in the DS3231
    @param dt DateTime object containing the desired date/time
*/
/**************************************************************************/
void RTC_DS3231_Adjust(struct _DateTime *dt)
{

	  //DS3231_setDateTime_DateTime("Feb 05 20", "17:36:38");//DS3231_setDateTime_DateTime("Feb 05 2020", "17:36:38");
	  //make CH bit 0
		stmI2cSendbuf[0] = DS3231_REG_TIME;
		stmI2cSendbuf[1] = bin2bcd(dt->ss);
		stmI2cSendbuf[2] = bin2bcd(dt->mm);
		stmI2cSendbuf[3] = bin2bcd(dt->hh);
		stmI2cSendbuf[4] = bin2bcd(0);
		stmI2cSendbuf[5] = bin2bcd(dt->date);
		stmI2cSendbuf[6] = bin2bcd(dt->m);
		stmI2cSendbuf[7] = bin2bcd(dt->yOff);

		//DS3231_writeRegister8(0x00, 0x00); //removed -- fixed by YOON.
		stm_I2C_SendBurst(DS3231_ADDR8, stmI2cSendbuf, 8); //write reg.

	//printf("DS3231_ADJ=%u.%u.%u %u:%u:%u\r\n",dt->yOff+2000, dt->m, dt->date, dt->hh, dt->mm, dt->ss);
}

/**************************************************************************/
/*!
    @brief  Get the current date and time from the DS3231
    @return DateTime object containing the current date and time
*/
/**************************************************************************/
//struct _DateTime* RTC_DS3231_now(struct _DateTime *dt) {
void RTC_DS3231_now(struct _DateTime *dt) { //DS3231_getDateTime()

    unsigned char retlen;

    retlen = stm_I2C_ReceiveBurstWithRestartCondition(DS3231_ADDR8, DS3231_REG_TIME, stmI2cRecvbuf, 7); //Reg0~6
	if(retlen==0){
		printf("i2c Rd error\r\n");
		return 0;
	}

    dt->ss = bcd2bin(stmI2cRecvbuf[0] & 0x7F);
    dt->mm = bcd2bin(stmI2cRecvbuf[1] & 0x7F);
    dt->hh = bcd2bin(stmI2cRecvbuf[2]);
    //skip 3 (Day: Mon, ...)
    dt->date = bcd2bin(stmI2cRecvbuf[4]);
    dt->m = bcd2bin(stmI2cRecvbuf[5]);
    dt->yOff = bcd2bin(stmI2cRecvbuf[6]);

	printf("DS3231_NOW=%u.%u.%u %u:%u:%u\r\n",dt->yOff+2000, dt->m, dt->date, dt->hh, dt->mm, dt->ss);
}

/**************************************************************************/
/*!
    @brief  Read the current mode of the SQW pin
    @return Mode as DS3231SqwPinMode enum
*/
/**************************************************************************/
DS3231SqwPinMode RTC_DS3231_readSqwPinMode() {
  unsigned char mode;

  DS3231_readRegister8(DS3231_REG_CONTROL, &mode);
  mode &= 0x18; //BIt[4:3] = RS2:RS1
  return mode;
}

/**************************************************************************/
/*!
    @brief  Change the SQW pin mode
    @param mode The mode to use
*/
/**************************************************************************/
void RTC_DS3231_writeSqwPinMode(DS3231SqwPinMode mode) {
	DS3231_writeRegister8(DS3231_REG_CONTROL, mode);
}
#if 0
/**************************************************************************/
/*!
    @brief  Read data from the DS3231's NVRAM (from Reg8..)
    @param buf Pointer to a buffer to store the data - make sure it's large enough to hold size bytes
    @param size Number of bytes to read
    @param address Starting NVRAM address, from 0 to 55
*/
/**************************************************************************/
void RTC_DS3231_readnvram(unsigned char* buf, unsigned char size, unsigned char address) {
  int addrByte = DS3231_NVRAM + address;
  unsigned char retlen;
  unsigned char pos;
  retlen = stm_I2C_ReceiveBurstWithRestartCondition(DS3231_ADDR8, addrByte, stmI2cRecvbuf, size);
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
    @brief  Write data to the DS3231 NVRAM
    @param address Starting NVRAM address, from 0 to 55
    @param buf Pointer to buffer containing the data to write
    @param size Number of bytes in buf to write to NVRAM
*/
/**************************************************************************/
void RTC_DS3231_writenvram(unsigned char address, unsigned char* buf, unsigned char size) {
  int addrByte = DS3231_NVRAM + address;
  int pos;
  for(pos=0;pos<size;pos++)
	  stmI2cSendbuf[pos] = buf;
  DS3231_writeRegister8(0x00, addrByte);
  stm_I2C_SendBurst(DS3231_ADDR8, stmI2cSendbuf, size);
}

/**************************************************************************/
/*!
    @brief  Shortcut to read one byte from NVRAM
    @param address NVRAM address, 0 to 55
    @return The byte read from NVRAM
*/
/**************************************************************************/
unsigned char RTC_DS3231_readnvram_single(unsigned char address) {
  unsigned char data;
  RTC_DS3231_readnvram(&data, 1, address);
  return data;
}

/**************************************************************************/
/*!
    @brief  Shortcut to write one byte to NVRAM
    @param address NVRAM address, 0 to 55
    @param data One byte to write
*/
/**************************************************************************/
void RTC_DS3231_writenvram_single(unsigned char address, unsigned char data) {
	RTC_DS3231_writenvram(address, &data, 1);
}
#endif

void DS3231_showTime(struct _DateTime *dt)
{
	char str[16];

	printf("NowTime = ");
	printf("%u-",dt->yOff + 2000);
	printf("%u-",dt->m);
	printf("%u ",dt->date);
	printf("(%s) ", daysOfTheWeek[DateTime_dayOfTheWeek(dt)]);
	printf("%u:",dt->hh);
	printf("%u:",dt->mm);
	printf("%u\r\n",dt->ss);
	//=== unix time since 1970.1.1 midnight
	printf("UnixTime= %us\r\n", DateTime_unixtime(dt) );
	sprintf(str,"%02u:%02u:%02u",dt->hh,dt->mm, dt->ss);
	SSD1306_OLED_printString_8X8(str,1,3,15);
}

//return 1 if already running.
unsigned char DS3231_setup(struct _DateTime *dt)
{
	// Initialize DS3231
	printf("Initialize DS3231\r\n");

	//Check Osc Stop Flag
	if (RTC_DS3231_isrunning()){ //==1 --> Osc  Running. show time and return.
		printf("DS3231 is already running. Return simply: ");
	    RTC_DS3231_now(dt);
	    DS3231_showTime(dt);
	    return 1; //
	}
	else{// If OSC stopped.
	  printf("DS3231 is fresh/stop\r\n");

	  //reset Osc Stop Flag -- ADDED YOON
	  DS3231_writeRegister8(DS3231_REG_STATUS,0x00);

	  //Set Time
	  RTC_DS3231_Adjust(dt);
	  //DS3231_setDateTime_DateTime("Feb 05 20", "17:36:38");//DS3231_setDateTime_DateTime("Feb 05 2020", "17:36:38");
	  //DS3231_setDateTime_DateTime(__DATE__, __TIME__);

	  //Enable SQ output as rate to 1Hz
	  printf("DS3231> Enable PPS out from SQ pin(1Hz).\r\n");
	  //nEOSC=0; BBSQW=0; CONV=0; RS2/1; INTCN=0; A2IE/A1IE = 00;
	  RTC_DS3231_writeSqwPinMode (DS3231_SquareWave1Hz);//DS3231_SquareWave32kHz);//DS3231_setOutput(DS3231_1HZ);
	  /*
		switch (DS3231_getOutput())
		{
		case DS3231_LOW:     printf("SQW = LOW"); break;
		case DS3231_HIGH:    printf("SQW = HIGH"); break;
		case DS3231_1HZ:     printf("SQW = 1Hz"); break;
		case DS3231_4096HZ:  printf("SQW = 4096Hz"); break;
		case DS3231_8192HZ:  printf("SQW = 8192Hz"); break;
		case DS3231_32768HZ: printf("SQW = 32768Hz"); break;
		default: printf("SQW = Unknown"); break;
		}
	   */
	  return 0; //fresh
  }
}

//================================================================================
void DS3231rtc_loop()
{
	struct _DateTime  DateTime;
	char str[16];
	unsigned char is_running = 0;

	printf("DS3231 Loop\r\n");
	printf("DS3231> I2C Init (8-bit Address = 0x%02x) ...", DS3231_ADDR8);
	if(!g_bI2CModuleConfigDone){
		printf("Not I2C installed\r\n");
		return;
	}
	//set time
	DateTime.yOff = 20;
	DateTime.m = 2;
	DateTime.date = 6;
	DateTime.day = 3;
	DateTime.hh = 9;
	DateTime.mm = 43;
	DateTime.ss = 20;
	is_running  = DS3231_setup(&DateTime);

  // For leading zero look to DS3231_dateformat example
	while(1){

		RTC_DS3231_now(&DateTime);
		DS3231_showTime(&DateTime);
		delayms(1000);
	}
}
#if 0

/**************************************************************************/
/*!
    @brief  Start using the PCF8523
    @return True
*/
/**************************************************************************/
////////////////////////////////////////////////////////////////////////////////
// RTC_PCF8563 implementation
boolean RTC_PCF8523::begin(void) {
  Wire.begin();
  return true;
}

/**************************************************************************/
/*!
    @brief  Check control register 3 to see if we've run adjust() yet (setting the date/time and battery switchover mode)
    @return True if the PCF8523 has been set up, false if not
*/
/**************************************************************************/
boolean RTC_PCF8523::initialized(void) {
  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire._I2C_WRITE((byte)PCF8523_CONTROL_3);
  Wire.endTransmission();

  Wire.requestFrom(PCF8523_ADDRESS, 1);
  unsigned char ss = Wire._I2C_READ();
  return ((ss & 0xE0) != 0xE0);
}

/**************************************************************************/
/*!
    @brief  Set the date and time, set battery switchover mode
    @param dt DateTime to set
*/
/**************************************************************************/
void RTC_PCF8523::adjust(const DateTime& dt) {
  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire._I2C_WRITE((byte)3); // start at location 3
  Wire._I2C_WRITE(bin2bcd(dt.second()));
  Wire._I2C_WRITE(bin2bcd(dt.minute()));
  Wire._I2C_WRITE(bin2bcd(dt.hour()));
  Wire._I2C_WRITE(bin2bcd(dt.day()));
  Wire._I2C_WRITE(bin2bcd(0)); // skip weekdays
  Wire._I2C_WRITE(bin2bcd(dt.month()));
  Wire._I2C_WRITE(bin2bcd(dt.year() - 2000));
  Wire.endTransmission();

  // set to battery switchover mode
  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire._I2C_WRITE((byte)PCF8523_CONTROL_3);
  Wire._I2C_WRITE((byte)0x00);
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Get the current date/time
    @return DateTime object containing the current date/time
*/
/**************************************************************************/
DateTime RTC_PCF8523::now() {
  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire._I2C_WRITE((byte)3);
  Wire.endTransmission();

  Wire.requestFrom(PCF8523_ADDRESS, 7);
  unsigned char ss = bcd2bin(Wire._I2C_READ() & 0x7F);
  unsigned char mm = bcd2bin(Wire._I2C_READ());
  unsigned char hh = bcd2bin(Wire._I2C_READ());
  unsigned char d = bcd2bin(Wire._I2C_READ());
  Wire._I2C_READ();  // skip 'weekdays'
  unsigned char m = bcd2bin(Wire._I2C_READ());
  unsigned short y = bcd2bin(Wire._I2C_READ()) + 2000;

  return DateTime (y, m, d, hh, mm, ss);
}

/**************************************************************************/
/*!
    @brief  Read the mode of the SQW pin on the PCF8523
    @return SQW pin mode as a Pcf8523SqwPinMode enum
*/
/**************************************************************************/
Pcf8523SqwPinMode RTC_PCF8523::readSqwPinMode() {
  int mode;

  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire._I2C_WRITE(PCF8523_CLKOUTCONTROL);
  Wire.endTransmission();

  Wire.requestFrom((unsigned char)PCF8523_ADDRESS, (unsigned char)1);
  mode = Wire._I2C_READ();

  mode >>= 3;
  mode &= 0x7;
  return static_cast<Pcf8523SqwPinMode>(mode);
}

/**************************************************************************/
/*!
    @brief  Set the SQW pin mode on the PCF8523
    @param mode The mode to set, see the Pcf8523SqwPinMode enum for options
*/
/**************************************************************************/
void RTC_PCF8523::writeSqwPinMode(Pcf8523SqwPinMode mode) {
  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire._I2C_WRITE(PCF8523_CLKOUTCONTROL);
  Wire._I2C_WRITE(mode << 3);
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Use an offset to calibrate the PCF8523. This can be used for:
            - Aging adjustment
            - Temperature compensation
            - Accuracy tuning
    @param mode The offset mode to use, once every two hours or once every minute. See the Pcf8523OffsetMode enum.
    @param offset Offset value from -64 to +63. See the datasheet for exact ppm values.
*/
/**************************************************************************/
void RTC_PCF8523::calibrate(Pcf8523OffsetMode mode, int8_t offset) {
  unsigned char reg = (unsigned char) offset & 0x7F;
  reg |= mode;

  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire._I2C_WRITE(PCF8523_OFFSET);
  Wire._I2C_WRITE(reg);
  Wire.endTransmission();
}



/**************************************************************************/
/*!
    @brief  Start I2C for the DS3231 and test succesful connection
    @return True if Wire can find DS3231 or false otherwise.
*/
/**************************************************************************/
boolean RTC_DS3231::begin(void) {
  Wire.begin();
  Wire.beginTransmission (DS3231_ADDRESS);
  if (Wire.endTransmission() == 0) return true;
  return false;
}

/**************************************************************************/
/*!
    @brief  Check the status register Oscillator Stop Flag to see if the DS3231 stopped due to power loss
    @return True if the bit is set (oscillator stopped) or false if it is running
*/
/**************************************************************************/
bool RTC_DS3231::lostPower(void) {
  return (read_i2c_register(DS3231_ADDRESS, DS3231_STATUSREG) >> 7);
}

/**************************************************************************/
/*!
    @brief  Set the date and flip the Oscillator Stop Flag
    @param dt DateTime object containing the date/time to set
*/
/**************************************************************************/
void RTC_DS3231::adjust(const DateTime& dt) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire._I2C_WRITE((byte)0); // start at location 0
  Wire._I2C_WRITE(bin2bcd(dt.second()));
  Wire._I2C_WRITE(bin2bcd(dt.minute()));
  Wire._I2C_WRITE(bin2bcd(dt.hour()));
  Wire._I2C_WRITE(bin2bcd(0));
  Wire._I2C_WRITE(bin2bcd(dt.day()));
  Wire._I2C_WRITE(bin2bcd(dt.month()));
  Wire._I2C_WRITE(bin2bcd(dt.year() - 2000));
  Wire.endTransmission();

  unsigned char statreg = read_i2c_register(DS3231_ADDRESS, DS3231_STATUSREG);
  statreg &= ~0x80; // flip OSF bit
  write_i2c_register(DS3231_ADDRESS, DS3231_STATUSREG, statreg);
}

/**************************************************************************/
/*!
    @brief  Get the current date/time
    @return DateTime object with the current date/time
*/
/**************************************************************************/
DateTime RTC_DS3231::now() {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire._I2C_WRITE((byte)0);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, 7);
  unsigned char ss = bcd2bin(Wire._I2C_READ() & 0x7F);
  unsigned char mm = bcd2bin(Wire._I2C_READ());
  unsigned char hh = bcd2bin(Wire._I2C_READ());
  Wire._I2C_READ();
  unsigned char d = bcd2bin(Wire._I2C_READ());
  unsigned char m = bcd2bin(Wire._I2C_READ());
  unsigned short y = bcd2bin(Wire._I2C_READ()) + 2000;

  return DateTime (y, m, d, hh, mm, ss);
}

/**************************************************************************/
/*!
    @brief  Read the SQW pin mode
    @return Pin mode, see Ds3231SqwPinMode enum
*/
/**************************************************************************/
Ds3231SqwPinMode RTC_DS3231::readSqwPinMode() {
  int mode;

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire._I2C_WRITE(DS3231_CONTROL);
  Wire.endTransmission();

  Wire.requestFrom((unsigned char)DS3231_ADDRESS, (unsigned char)1);
  mode = Wire._I2C_READ();

  mode &= 0x93;
  return static_cast<Ds3231SqwPinMode>(mode);
}

/**************************************************************************/
/*!
    @brief  Set the SQW pin mode
    @param mode Desired mode, see Ds3231SqwPinMode enum
*/
/**************************************************************************/
void RTC_DS3231::writeSqwPinMode(Ds3231SqwPinMode mode) {
  unsigned char ctrl;
  ctrl = read_i2c_register(DS3231_ADDRESS, DS3231_CONTROL);

  ctrl &= ~0x04; // turn off INTCON
  ctrl &= ~0x18; // set freq bits to 0

  if (mode == DS3231_OFF) {
    ctrl |= 0x04; // turn on INTCN
  } else {
    ctrl |= mode;
  }
  write_i2c_register(DS3231_ADDRESS, DS3231_CONTROL, ctrl);

  //Serial.println( read_i2c_register(DS3231_ADDRESS, DS3231_CONTROL), HEX);
}

/**************************************************************************/
/*!
    @brief  Get the current temperature from the DS3231's temperature sensor
    @return Current temperature (float)
*/
/**************************************************************************/
float RTC_DS3231::getTemperature()
{
  unsigned char msb, lsb;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire._I2C_WRITE(DS3231_TEMPERATUREREG);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, 2);
  msb = Wire._I2C_READ();
  lsb = Wire._I2C_READ();

//  Serial.print("msb=");
//  Serial.print(msb,HEX);
//  Serial.print(", lsb=");
//  Serial.println(lsb,HEX);

  return (float) msb + (lsb >> 6) * 0.25f;
}

#endif


#if 0
struct RTCDateTime
{
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
    unsigned char dayOfWeek;
    unsigned int unixtime;
};

typedef enum
{
    DS3231_LOW          = 0x00,
    DS3231_HIGH         = 0x80,
    DS3231_1HZ          = 0x10,
    DS3231_4096HZ       = 0x11,
    DS3231_8192HZ       = 0x12,
    DS3231_32768HZ      = 0x13
} DS3231_sqwOut_t;


struct RTCDateTime g_RTCDateTime, *gp_RTCDateTime;
/*
  DS3231: Real-Time Clock. Simple example
  Read more: www.jarzebski.pl/arduino/komponenty/zegar-czasu-rzeczywistego-rtc-DS3231.html
  GIT: https://github.com/jarzebski/Arduino-DS3231
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/

	void DS3231_setDateTime(unsigned short year, unsigned char month, unsigned char day, unsigned char hour, unsigned char minute, unsigned char second);
	void DS3231_setDateTime_Time(unsigned int t);
	void DS3231_setDateTime_DateTime(const char* date, const char* time);
	struct RTCDateTime *DS3231_getDateTime(void);
	unsigned char DS3231_isReady(void);

	unsigned char DS3231_readByte(unsigned char offset);
	unsigned char DS3231_writeByte(unsigned char offset, unsigned char data);

	void DS3231_readMemory(unsigned char offset, unsigned char * buff, unsigned char size);
	void DS3231_writeMemory(unsigned char offset, unsigned char * buff, unsigned char size);

	void DS3231_clearMemory(void);

	//DS3231_sqwOut_t DS3231_getOutput(void);
	void DS3231_setOutputSqe(DS3231_sqwOut_t mode);
	void DS3231_setOutput(bool mode);

	char* DS3231_dateFormat(const char* dateFormat, struct RTCDateTime dt);



	char *strDayOfWeek(unsigned char dayOfWeek);
	char *strMonth(unsigned char month);
	char *DS3231_strAmPm(unsigned char hour, bool uppercase);
	char *strDaySufix(unsigned char day);

	void DS3231_readPacket(unsigned char offset, unsigned char * buff, unsigned char size);
	void DS3231_writePacket(unsigned char offset, unsigned char * buff, unsigned char size);

	unsigned char hour12(unsigned char hour24);
	unsigned char bcd2dec(unsigned char bcd);
	unsigned char dec2bcd(unsigned char dec);

	long time2long(unsigned short days, unsigned char hours, unsigned char minutes, unsigned char seconds);
	unsigned short date2days(unsigned short year, unsigned char month, unsigned char day);
	unsigned char daysInMonth(unsigned short year, unsigned char month);
	unsigned short DS3231_dayInYear(unsigned short year, unsigned char month, unsigned char day);
	bool isLeapYear(unsigned short year);
	unsigned char dow(unsigned short y, unsigned char m, unsigned char d);

	unsigned int unixtime(void);
	unsigned char conv2d(const char* p);

	void DS3231_writeRegister8(unsigned char reg, unsigned char value);
	unsigned char DS3231_readRegister8(unsigned char reg, unsigned char *retval);

const unsigned char daysArray[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
const unsigned char dowArray[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };


unsigned char pgm_read_byte(unsigned x){

}

unsigned char bcd2dec(unsigned char bcd)
{
    return ((bcd / 16) * 10) + (bcd % 16);
}

unsigned char dec2bcd(unsigned char dec)
{
    return ((dec / 10) * 16) + (dec % 10);
}


char *strDayOfWeek(unsigned char dayOfWeek)
{
    switch (dayOfWeek) {
        case 1:
            return "Monday";
            break;
        case 2:
            return "Tuesday";
            break;
        case 3:
            return "Wednesday";
            break;
        case 4:
            return "Thursday";
            break;
        case 5:
            return "Friday";
            break;
        case 6:
            return "Saturday";
            break;
        case 7:
            return "Sunday";
            break;
        default:
            return "Unknown";
    }
}

char *strMonth(unsigned char month)
{
    switch (month) {
        case 1:
            return "January";
            break;
        case 2:
            return "February";
            break;
        case 3:
            return "March";
            break;
        case 4:
            return "April";
            break;
        case 5:
            return "May";
            break;
        case 6:
            return "June";
            break;
        case 7:
            return "July";
            break;
        case 8:
            return "August";
            break;
        case 9:
            return "September";
            break;
        case 10:
            return "October";
            break;
        case 11:
            return "November";
            break;
        case 12:
            return "December";
            break;
        default:
            return "Unknown";
    }
}

char *DS3231_strAmPm(unsigned char hour, bool uppercase)
{
    if (hour < 12)
    {
        if (uppercase)
        {
            return "AM";
        } else
        {
            return "am";
        }
    } else
    {
        if (uppercase)
        {
            return "PM";
        } else
        {
            return "pm";
        }
    }
}

char *strDaySufix(unsigned char day)
{
    if (day % 10 == 1)
    {
        return "st";
    } else
    if (day % 10 == 2)
    {
        return "nd";
    }
    if (day % 10 == 3)
    {
        return "rd";
    }

    return "th";
}

unsigned char hour12(unsigned char hour24)
{
    if (hour24 == 0)
    {
        return 12;
    }

    if (hour24 > 12)
    {
       return (hour24 - 12);
    }

    return hour24;
}

long time2long(unsigned short days, unsigned char hours, unsigned char minutes, unsigned char seconds)
{
    return ((days * 24L + hours) * 60 + minutes) * 60 + seconds;
}

unsigned short DS3231_dayInYear(unsigned short year, unsigned char month, unsigned char day)
{
    unsigned short fromDate;
    unsigned short toDate;

    fromDate = date2days(year, 1, 1);
    toDate = date2days(year, month, day);

    return (toDate - fromDate);
}

bool isLeapYear(unsigned short year)
{
    return (year % 4 == 0);
}

unsigned char daysInMonth(unsigned short year, unsigned char month)
{
    unsigned char days;

    days = pgm_read_byte(daysArray + month - 1);

    if ((month == 2) && isLeapYear(year))
    {
        ++days;
    }

    return days;
}

unsigned short date2days(unsigned short year, unsigned char month, unsigned char day)
{
	unsigned char i;
    year = year - 2000;

    unsigned short days16 = day;

    for (i = 1; i < month; ++i)
    {
        days16 += pgm_read_byte(daysArray + i - 1);
    }

    if ((month == 2) && isLeapYear(year))
    {
        ++days16;
    }

    return days16 + 365 * year + (year + 3) / 4 - 1;
}

unsigned int unixtime(void)
{
    unsigned int u;

    u = time2long(date2days(g_RTCDateTime.year, g_RTCDateTime.month, g_RTCDateTime.day), g_RTCDateTime.hour, g_RTCDateTime.minute, g_RTCDateTime.second);
    u += 946681200;

    return u;
}

unsigned char conv2d(const char* p)
{
    unsigned char v = 0;

    if ('0' <= *p && *p <= '9')
    {
        v = *p - '0';
    }

    return 10 * v + *++p - '0';
}

unsigned char dow(unsigned short y, unsigned char m, unsigned char d)
{
    unsigned char dow;

    y -= m < 3;
    dow = ((y + y/4 - y/100 + y/400 + pgm_read_byte(dowArray+(m-1)) + d) % 7);

    if (dow == 0)
    {
        return 7;
    }

    return dow;
}



void DS3231_setDateTime(unsigned short year, unsigned char month, unsigned char day, unsigned char hour, unsigned char minute, unsigned char second)
{

	stmI2cSendbuf[0] = DS3231_REG_TIME;
	stmI2cSendbuf[1] = dec2bcd(second);
	stmI2cSendbuf[2] = dec2bcd(minute);
	stmI2cSendbuf[3] = dec2bcd(hour);
	stmI2cSendbuf[4] = dec2bcd(dow(year, month, day));
	stmI2cSendbuf[5] = dec2bcd(day);
	stmI2cSendbuf[6] = dec2bcd(month);
	stmI2cSendbuf[7] = dec2bcd((year-2000));
	stmI2cSendbuf[8] = DS3231_REG_TIME;
	stm_I2C_SendBurst(DS3231_ADDR8, stmI2cSendbuf, 8); //write reg.

}

void DS3231_setDateTime_time(unsigned int t)
{
    t -= 946681200;

    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;

    second = t % 60;
    t /= 60;

    minute = t % 60;
    t /= 60;

    hour = t % 24;
    unsigned short days = t / 24;
    unsigned char leap;

    for (year = 0; ; ++year)
    {
        leap = year % 4 == 0;
        if (days < 365 + leap)
        {
            break;
        }
        days -= 365 + leap;
    }

    for (month = 1; ; ++month)
    {
        unsigned char daysPerMonth = pgm_read_byte(daysArray + month - 1);

        if (leap && month == 2)
        {
            ++daysPerMonth;
        }

        if (days < daysPerMonth)
        {
            break;
        }
        days -= daysPerMonth;
    }

    day = days + 1;

    DS3231_setDateTime(year+2000, month, day, hour, minute, second);
}

void DS3231_setDateTime_DateTime(const char* date, const char* time) //"Feb 05 2020" "xx:xx:xx"
{
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;

    year = conv2d(date + 7); //year = conv2d(date + 9);

    switch (date[0])
    {
        case 'J': month = (date[1] == 'a') ? 1 : (month = (date[2] == 'n') ? 6 : 7); break;
        case 'F': month = 2; break;
        case 'A': month = (date[2] == 'r') ? 4 : 8; break;
        case 'M': month = (date[2] == 'r') ? 3 : 5; break;
        case 'S': month = 9; break;
        case 'O': month = 10; break;
        case 'N': month = 11; break;
        case 'D': month = 12; break;
    }

    day = conv2d(date + 4);
    hour = conv2d(time);
    minute = conv2d(time + 3);
    second = conv2d(time + 6);

    printf("year-Month-day = %u-%u-%u\r\n", year, month, day);

    DS3231_setDateTime(year+2000, month, day, hour, minute, second);
}

char* DS3231_dateFormat(const char* dateFormat, struct RTCDateTime dt)
{
    char buffer[255];

    buffer[0] = 0;

    char helper[11];

    while (*dateFormat != '\0')
    {
        switch (dateFormat[0])
        {
            // Day decoder
            case 'd':
                sprintf(helper, "%02d", dt.day);
                strcat(buffer, (const char *)helper);
                break;
            case 'j':
                sprintf(helper, "%d", dt.day);
                strcat(buffer, (const char *)helper);
                break;
            case 'l':
                strcat(buffer, (const char *)strDayOfWeek(dt.dayOfWeek));
                break;
            case 'D':
                strncat(buffer, strDayOfWeek(dt.dayOfWeek), 3);
                break;
            case 'N':
                sprintf(helper, "%d", dt.dayOfWeek);
                strcat(buffer, (const char *)helper);
                break;
            case 'w':
                sprintf(helper, "%d", (dt.dayOfWeek + 7) % 7);
                strcat(buffer, (const char *)helper);
                break;
            case 'z':
                sprintf(helper, "%d", dayInYear(dt.year, dt.month, dt.day));
                strcat(buffer, (const char *)helper);
                break;
            case 'S':
                strcat(buffer, (const char *)strDaySufix(dt.day));
                break;

            // Month decoder
            case 'm':
                sprintf(helper, "%02d", dt.month);
                strcat(buffer, (const char *)helper);
                break;
            case 'n':
                sprintf(helper, "%d", dt.month);
                strcat(buffer, (const char *)helper);
                break;
            case 'F':
                strcat(buffer, (const char *)strMonth(dt.month));
                break;
            case 'M':
                strncat(buffer, (const char *)strMonth(dt.month), 3);
                break;
            case 't':
                sprintf(helper, "%d", daysInMonth(dt.year, dt.month));
                strcat(buffer, (const char *)helper);
                break;

            // Year decoder
            case 'Y':
                sprintf(helper, "%d", dt.year);
                strcat(buffer, (const char *)helper);
                break;
            case 'y': sprintf(helper, "%02d", dt.year-2000);
                strcat(buffer, (const char *)helper);
                break;
            case 'L':
                sprintf(helper, "%d", isLeapYear(dt.year));
                strcat(buffer, (const char *)helper);
                break;

            // Hour decoder
            case 'H':
                sprintf(helper, "%02d", dt.hour);
                strcat(buffer, (const char *)helper);
                break;
            case 'G':
                sprintf(helper, "%d", dt.hour);
                strcat(buffer, (const char *)helper);
                break;
            case 'h':
                sprintf(helper, "%02d", hour12(dt.hour));
                strcat(buffer, (const char *)helper);
                break;
            case 'g':
                sprintf(helper, "%d", hour12(dt.hour));
                strcat(buffer, (const char *)helper);
                break;
            case 'A':
                strcat(buffer, (const char *)strAmPm(dt.hour, true));
                break;
            case 'a':
                strcat(buffer, (const char *)strAmPm(dt.hour, false));
                break;

            // Minute decoder
            case 'i':
                sprintf(helper, "%02d", dt.minute);
                strcat(buffer, (const char *)helper);
                break;

            // Second decoder
            case 's':
                sprintf(helper, "%02d", dt.second);
                strcat(buffer, (const char *)helper);
                break;

            // Misc decoder
            case 'U':
                sprintf(helper, "%lu", dt.unixtime);
                strcat(buffer, (const char *)helper);
                break;

            default:
                strncat(buffer, dateFormat, 1);
                break;
        }
        dateFormat++;
    }

    return buffer;
}

struct RTCDateTime *DS3231_getDateTime(void)
{
    int values[7];
    int i;
    unsigned char value;
    unsigned char retlen;

    retlen = stm_I2C_ReceiveBurstWithRestartCondition(DS3231_ADDR8, DS3231_REG_TIME, stmI2cRecvbuf, 7); //Reg0~6
	if(retlen==0){
		printf("i2c Rd error\r\n");
		return 0;
	}

    for (i = 6; i >= 0; i--)
    {
    	if (i == 3)
    	{
    		values[i] = stmI2cRecvbuf[i];
    	} else
    	{
    		values[i] = bcd2dec(stmI2cRecvbuf[i]);
        }
    }

    g_RTCDateTime.year = values[0] + 2000;
    g_RTCDateTime.month = values[1];
    g_RTCDateTime.day = values[2];
    g_RTCDateTime.dayOfWeek = values[3];
    g_RTCDateTime.hour = values[4];
    g_RTCDateTime.minute = values[5];
    g_RTCDateTime.second = values[6];
    g_RTCDateTime.unixtime = unixtime();

    return &g_RTCDateTime;
}

unsigned char DS3231_isReady(void)
{
    unsigned char ss;
    int i;
    for(i=0;i<8;i++){
    	DS3231_readRegister8(i,&ss);
    	printf("val=%02x\r\n", ss);
    	delayms(100);
    }


	DS3231_readRegister8(DS3231_REG_TIME,&ss);

    return !(ss>>7);
}
/*
unsigned char DS3231_readByte(unsigned char offset)
{
    unsigned char buff[1];

    readPacket(offset, buff, 1);

    return (unsigned char)buff[0];
}

unsigned char DS3231_writeByte(unsigned char offset, unsigned char data)
{
    unsigned char buff[1];

    buff[0] = data;

    writePacket(offset, buff, 1);
}

void DS3231_readPacket(unsigned char offset, unsigned char * buff, unsigned char size)
{
    Wire.beginTransmission(DS3231_ADDR8);

    #if ARDUINO >= 100
        Wire.write(DS3231_REG_RAM + offset);
    #else
        Wire.send(DS3231_REG_RAM + offset);
    #endif

    Wire.endTransmission();

    Wire.requestFrom(DS3231_ADDR8, (int)size);

    while (!Wire.available()) {}

    for (int i = 0; i < size; i++)
    {
        #if ARDUINO >= 100
            buff[i] = Wire.read();
        #else
            buff[i] = Wire.receive();
        #endif
    }

    Wire.endTransmission();
}

void DS3231_writePacket(unsigned char offset, unsigned char * buff, unsigned char size)
{
    Wire.beginTransmission(DS3231_ADDR8);

    #if ARDUINO >= 100
        Wire.write(DS3231_REG_RAM + offset);
        Wire.write(buff, size);
    #else
        Wire.send(DS3231_REG_RAM + offset);
        Wire.send(buff, size);
    #endif

    Wire.endTransmission();
}

void DS3231_readMemory(unsigned char offset, unsigned char * buff, unsigned char size)
{
    if (size > 56)
    {
        size = 56;
    }

    if (size > 31)
    {
        readPacket(offset, buff, size);
        readPacket(offset + 31, buff + 31, size - 31);
    } else
    {
        readPacket(offset, buff, size);
    }
}

void DS3231_writeMemory(unsigned char offset, unsigned char * buff, unsigned char size)
{
    if (size > 56)
    {
        size = 56;
    }

    if (size > 31)
    {
        writePacket(offset, buff, 31);
        writePacket(offset+31, buff+31, size-31);
    } else
    {
        writePacket(offset, buff, size);
    }
}

void DS3231_clearMemory(void)
{
    for (unsigned char offset = 0; offset < 56; offset++)
    {
        writeByte(offset, 0);
    }
}
*/
void DS3231_setOutputSqe(DS3231_sqwOut_t mode)
{
	DS3231_writeRegister8(DS3231_REG_CONTROL, mode);
}

void DS3231_setOutput(bool high)
{
    if (high)
    {
    	DS3231_writeRegister8(DS3231_REG_CONTROL, DS3231_HIGH);
    } else
    {
    	DS3231_writeRegister8(DS3231_REG_CONTROL, DS3231_LOW);
    }
}

DS3231_sqwOut_t DS3231_getOutput(void)
{
    unsigned char value;
    value = DS3231_readRegister8(DS3231_REG_CONTROL, &value);
    return (DS3231_sqwOut_t)value;
}

//================================================================================
void DS3231rtc_loop()
{
	printf("DS3231 Loop\r\n");
	printf("DS3231> I2C Init (8-bit Address = 0x%02x) ...", DS3231_ADDR8);
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
	printf("DS3231> Init\r\n");

	DS3231_setup();

	gp_RTCDateTime = DS3231_getDateTime();

  // For leading zero look to DS3231_dateformat example
	while(1){

		RTC_DS3231_now();

		printf("Raw data: ");
		printf("%u-",gp_RTCDateTime->year);
		printf("%u-",gp_RTCDateTime->month);
		printf("%u ",gp_RTCDateTime->day);
		printf("%u:",gp_RTCDateTime->hour);
		printf("%u:",gp_RTCDateTime->minute);
		printf("%u\r\n",gp_RTCDateTime->second);

/*
		//SQ -// Enable output as rate to 1Hz
		DS3231_setOutput(DS3231_1HZ);

		switch (DS3231_getOutput())
		{
			case DS3231_LOW:     printf("SQW = LOW"); break;
			case DS3231_HIGH:    printf("SQW = HIGH"); break;
			case DS3231_1HZ:     printf("SQW = 1Hz"); break;
			case DS3231_4096HZ:  printf("SQW = 4096Hz"); break;
			case DS3231_8192HZ:  printf("SQW = 8192Hz"); break;
			case DS3231_32768HZ: printf("SQW = 32768Hz"); break;
			default: printf("SQW = Unknown"); break;
		}
*/
		delayms(1000);
}
}
#endif


#endif

#if 0
  DateTime operator+(const TimeSpan& span);
  DateTime operator-(const TimeSpan& span);
  TimeSpan operator-(const DateTime& right);
  bool operator<(const DateTime& right) const;
  /*!
      @brief  Test if one DateTime is greater (later) than another
      @param right DateTime object to compare
      @return True if the left object is greater than the right object, false otherwise
  */
  bool operator>(const DateTime& right) const  { return right < *this; }
  /*!
      @brief  Test if one DateTime is less (earlier) than or equal to another
      @param right DateTime object to compare
      @return True if the left object is less than or equal to the right object, false otherwise
  */
  bool operator<=(const DateTime& right) const { return !(*this > right); }
  /*!
      @brief  Test if one DateTime is greater (later) than or equal to another
      @param right DateTime object to compare
      @return True if the left object is greater than or equal to the right object, false otherwise
  */
  bool operator>=(const DateTime& right) const { return !(*this < right); }
  bool operator==(const DateTime& right) const;
  /*!
      @brief  Test if two DateTime objects not equal
      @param right DateTime object to compare
      @return True if the two objects are not equal, false if they are
  */
  bool operator!=(const DateTime& right) const { return !(*this == right); }


char* DateTime_toString(char* buffer);
 unsigned short DateTime_year(struct _DateTime *dt)       { return 2000 + dt->yOff; }
  unsigned char DateTime_month(struct _DateTime *dt)       { return dt->m; }
  unsigned char DateTime_day(struct _DateTime *dt)          { return dt->d; }
  unsigned char DateTime_hour(struct _DateTime *dt)        { return dt->hh; }
  unsigned char DateTime_minute(struct _DateTime *dt)      { return dt->mm; }
  unsigned char DateTime_second(struct _DateTime *dt)      { return dt->ss; }
  unsigned char DateTime_dayOfTheWeek(struct _DateTime *dt);
  // 32-bit times as seconds since 1/1/2000
long DateTime_secondstime(struct _DateTime *dt);

DateTime (unsigned int t = SECONDS_FROM_1970_TO_2000);
DateTime (unsigned short year, unsigned char month, unsigned char day,unsigned char hour = 0, unsigned char min = 0, unsigned char sec = 0);
DateTime (struct _DateTime *copy);
DateTime (const char* date, const char* time);
//DateTime (const __FlashStringHelper* date, const __FlashStringHelper* time);

/** 32-bit times as seconds since 1/1/1970 */
unsigned int DateTime_unixtime(struct _DateTime *dt);
	  // 32-bit times as seconds since 1/1/2000
long DateTime_secondstime();

/** ISO 8601 Timestamp function */
enum timestampOpt{
    TIMESTAMP_FULL, // YYYY-MM-DDTHH:MM:SS
    TIMESTAMP_TIME, // HH:MM:SS
    TIMESTAMP_DATE  // YYYY-MM-DD
};

String timestamp(timestampOpt opt = TIMESTAMP_FULL);

// Timespan which can represent changes in time with seconds accuracy.
struct _TimeSpan {
  int32_t _seconds;   ///< Actual TimeSpan value is stored as seconds
} TimeSpan;
#endif

#if 0
TimeSpan operator+(const TimeSpan& right);
TimeSpan operator-(const TimeSpan& right);
TimeSpan (int32_t seconds = 0);
TimeSpan (int16_t days, int8_t hours, int8_t minutes, int8_t seconds);
TimeSpan (const TimeSpan& copy);

/*!
    @brief  Number of days in the TimeSpan
            e.g. 4
    @return int16_t days
*/
int16_t TimeSpan_days(struct _TimeSpan *ts) { return ts->_seconds / 86400L; }
/*!
    @brief  Number of hours in the TimeSpan
            This is not the total hours, it includes the days
            e.g. 4 days, 3 hours - NOT 99 hours
    @return int8_t hours
*/
int8_t  TimeSpan_hours(struct _TimeSpan *ts) { return ts->_seconds / 3600 % 24; }
/*!
    @brief  Number of minutes in the TimeSpan
            This is not the total minutes, it includes days/hours
            e.g. 4 days, 3 hours, 27 minutes
    @return int8_t minutes
*/
int8_t  TimeSpan_minutes(struct _TimeSpan *ts) { return ts->_seconds / 60 % 60; }
/*!
    @brief  Number of seconds in the TimeSpan
            This is not the total seconds, it includes the days/hours/minutes
            e.g. 4 days, 3 hours, 27 minutes, 7 seconds
    @return int8_t seconds
*/
int8_t  TimeSpan_seconds(struct _TimeSpan *ts) { return ts->_seconds % 60; }
/*!
    @brief  Total number of seconds in the TimeSpan, e.g. 358027
    @return int32_t seconds
*/
int32_t TimeSpan_totalseconds(struct _TimeSpan *ts) { return ts->_seconds; }
#endif

#if 0
/**************************************************************************/
/*!
    @brief  RTC using the internal millis() clock, has to be initialized before use.
            NOTE: this is immune to millis() rollover events.
*/
/**************************************************************************/
struct _RTC_Millis{
  unsigned int lastUnix;   ///< Unix time from the previous call to now() - prevents rollover issues
  unsigned int lastMillis; ///< the millis() value corresponding to the last **full second** of Unix time
} RTC_Millis;

//    @param dt DateTime object with the date/time to set
static void RTC_Millis_begin(const DateTime *dt) { adjust(dt); }
static void RTC_Millis_adjust(const DateTime *dt);
static RTC_Millis_DateTime now();

/**************************************************************************/
/*!
    @brief  RTC using the internal micros() clock, has to be initialized before
            use. Unlike RTC_Millis, this can be tuned in order to compensate for
            the natural drift of the system clock. Note that now() has to be
            called more frequently than the micros() rollover period, which is
            approximately 71.6 minutes.
*/
/**************************************************************************/
  /*!
      @brief  Start the RTC
      @param dt DateTime object with the date/time to set
  */
struct _RTC_Micros{
  unsigned int microsPerSecond;  ///< Number of microseconds reported by micros() per "true" (calibrated) second
  unsigned int lastUnix;         ///< Unix time from the previous call to now() - prevents rollover issues
  unsigned int lastMicros;       ///< micros() value corresponding to the last full second of Unix time
} RTC_Micros;

static void RTC_Micros_begin(structDateTime* dt) { RTC_Micros_adjust(dt); }
static void RTC_Micros_adjust(struct _DateTime* dt);
static void RTC_Micros_adjustDrift(int ppm);
static RTC_Micros_DateTime now();
#endif

#if 0
/**************************************************************************/
/*!
    @brief  Add a TimeSpan to the DateTime object
    @param span TimeSpan object
    @return new DateTime object with span added to it
*/
/**************************************************************************/
DateTime DateTime::operator+(struct _TimeSpan *span) {
  return DateTime(unixtime()+span.totalseconds());
}

/**************************************************************************/
/*!
    @brief  Subtract a TimeSpan from the DateTime object
    @param span TimeSpan object
    @return new DateTime object with span subtracted from it
*/
/**************************************************************************/
DateTime DateTime::operator-(const TimeSpan& span) {
  return DateTime(unixtime()-span.totalseconds());
}

/**************************************************************************/
/*!
    @brief  Subtract one DateTime from another
    @param right The DateTime object to subtract from self (the left object)
    @return TimeSpan of the difference between DateTimes
*/
/**************************************************************************/
TimeSpan DateTime::operator-(const DateTime& right) {
  return TimeSpan(unixtime()-right.unixtime());
}

/**************************************************************************/
/*!
    @brief  Is one DateTime object less than (older) than the other?
    @param right Comparison DateTime object
    @return True if the left object is older than the right object
*/
/**************************************************************************/
bool DateTime::operator<(const DateTime& right) const {
  return unixtime() < right.unixtime();
}

/**************************************************************************/
/*!
    @brief  Is one DateTime object equal to the other?
    @param right Comparison DateTime object
    @return True if both DateTime objects are the same
*/
/**************************************************************************/
bool DateTime::operator==(const DateTime& right) const {
  return unixtime() == right.unixtime();
}

/**************************************************************************/
/*!
    @brief  ISO 8601 Timestamp
    @param opt Format of the timestamp
    @return Timestamp string, e.g. "2000-01-01T12:34:56"
*/
/**************************************************************************/
String DateTime_timestamp(enum timestampOpt opt){
  char buffer[20];

  //Generate timestamp according to opt
  switch(opt){
    case TIMESTAMP_TIME:
    //Only time
    sprintf(buffer, "%02d:%02d:%02d", hh, mm, ss);
    break;
    case TIMESTAMP_DATE:
    //Only date
    sprintf(buffer, "%d-%02d-%02d", 2000+yOff, m, d);
    break;
    default:
    //Full
    sprintf(buffer, "%d-%02d-%02dT%02d:%02d:%02d", 2000+yOff, m, d, hh, mm, ss);
  }
  return String(buffer);
}



/**************************************************************************/
/*!
    @brief  Create a new TimeSpan object in seconds
    @param seconds Number of seconds
*/
/**************************************************************************/
TimeSpan::TimeSpan (int32_t seconds):
  _seconds(seconds)
{}

/**************************************************************************/
/*!
    @brief  Create a new TimeSpan object using a number of days/hours/minutes/seconds
            e.g. Make a TimeSpan of 3 hours and 45 minutes: new TimeSpan(0, 3, 45, 0);
    @param days Number of days
    @param hours Number of hours
    @param minutes Number of minutes
    @param seconds Number of seconds
*/
/**************************************************************************/
TimeSpan::TimeSpan (int16_t days, int8_t hours, int8_t minutes, int8_t seconds):
  _seconds((int32_t)days*86400L + (int32_t)hours*3600 + (int32_t)minutes*60 + seconds)
{}

/**************************************************************************/
/*!
    @brief  Copy constructor, make a new TimeSpan using an existing one
    @param copy The TimeSpan to copy
*/
/**************************************************************************/
TimeSpan::TimeSpan (const TimeSpan& copy):
  _seconds(copy._seconds)
{}

/**************************************************************************/
/*!
    @brief  Add two TimeSpans
    @param right TimeSpan to add
    @return New TimeSpan object, sum of left and right
*/
/**************************************************************************/
TimeSpan TimeSpan::operator+(const TimeSpan& right) {
  return TimeSpan(_seconds+right._seconds);
}

/**************************************************************************/
/*!
    @brief  Subtract a TimeSpan
    @param right TimeSpan to subtract
    @return New TimeSpan object, right subtracted from left
*/
/**************************************************************************/
TimeSpan TimeSpan::operator-(const TimeSpan& right) {
  return TimeSpan(_seconds-right._seconds);
}

#endif
#if 0
/** Alignment between the milis() timescale and the Unix timescale. These
  two variables are updated on each call to now(), which prevents
  rollover issues. Note that lastMillis is **not** the millis() value
  of the last call to now(): it's the millis() value corresponding to
  the last **full second** of Unix time. */
unsigned int RTC_Millis_lastMillis;
unsigned int RTC_Millis_lastUnix;

/**************************************************************************/
/*!
    @brief  Set the current date/time of the RTC_Millis clock.
    @param dt DateTime object with the desired date and time
*/
/**************************************************************************/
void RTC_Millis_adjust(struct _DateTime *dt) {
	RTC_Millis.lastMillis = millis();
	RTC_Millis.lastUnix = dt->unixtime();
}

/**************************************************************************/
/*!
    @brief  Return a DateTime object containing the current date/time.
            Note that computing (millis() - lastMillis) is rollover-safe as long
            as this method is called at least once every 49.7 days.
    @return DateTime object containing current time
*/
/**************************************************************************/
struct _DateTime* RTC_Millis_now() {
  unsigned int elapsedSeconds = (millis() - lastMillis) / 1000;
  RTC_Millis.lastMillis += elapsedSeconds * 1000;
  RTC_Millis.lastUnix += elapsedSeconds;
  return lastUnix;
}



/** Number of microseconds reported by micros() per "true" (calibrated) second. */
unsigned int microsPerSecond = 1000000;

/** The timing logic is identical to RTC_Millis. */
unsigned int RTC_Micros_lastMicros;
unsigned int RTC_Micros_lastUnix;

/**************************************************************************/
/*!
    @brief  Set the current date/time of the RTC_Micros clock.
    @param dt DateTime object with the desired date and time
*/
/**************************************************************************/
void RTC_Micros_adjust(struct _RTC_Micros *pmic, struct _DateTime *dt) {
  pmic->lastMicros = micros();
  pmic->lastUnix = DateTime_unixtime(dt);
}

/**************************************************************************/
/*!
    @brief  Adjust the RTC_Micros clock to compensate for system clock drift
    @param ppm Adjustment to make
*/
/**************************************************************************/
// A positive adjustment makes the clock faster.
void RTC_Micros_adjustDrift(struct _RTC_Micros *pmic,int ppm) {
	pmic->microsPerSecond = 1000000 - ppm;
}

/**************************************************************************/
/*!
    @brief  Get the current date/time from the RTC_Micros clock.
    @return DateTime object containing the current date/time
*/
/**************************************************************************/
DateTime RTC_Micros_now(struct _RTC_Micros *pmic) {
  unsigned int elapsedSeconds = (micros() - RTC_Micros.lastMicros) / RTC_Micros.microsPerSecond;
  RTC_Micros.lastMicros += elapsedSeconds * RTC_Micros.microsPerSecond;
  RTC_Micros.lastUnix += elapsedSeconds;
  return RTC_Micros.lastUnix;
}
#endif
