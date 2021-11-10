#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>

#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"

extern struct usartbuf_st usart1_rbuf;
extern struct usartbuf_st usart2_rbuf;
extern struct usartbuf_st usart3_rbuf;

//Strange --> No ANT, but RMC msg is receiving without PPS. ????????????????

/* For STM32F1
 * Using USART2 (PIN12/11=U2RXD/U2TXD)
 */
/*
#define RBUF_SIZE 256
struct usartbuf_st{
	unsigned int in; //next in index
	unsigned int out;//next out index
	char prevchar;
	char term; //0x0d/0x0a detected
	char buf[RBUF_SIZE];

};
static struct usartbuf_st usart1_rbuf={0,0,0,0,};
static struct usartbuf_st usart3_rbuf={0,0,0,0,};
#define USART1_RBUFLEN ((unsigned short)(usart1_rbuf.in - usart1_rbuf.out))
#define USART3_RBUFLEN ((unsigned short)(usart3_rbuf.in - usart3_rbuf.out))

void USART1_buf_Init(void){
	usart1_rbuf.in = 0;
	usart1_rbuf.out = 0;
	usart1_rbuf.prevchar = 0x00;
	usart1_rbuf.term = 0x00;
}

void USART3_buf_Init(void){
	usart3_rbuf.in = 0;
	usart3_rbuf.out = 0;
	usart3_rbuf.prevchar = 0x00;
	usart3_rbuf.term = 0x00;
}
*/
/*For GeneralPurpose
void USART3_IRQHandler(void){
	struct usartbuf_st *bufp;
	char curchar;

	if(USART_GetITStatus(USART3, USART_IT_RXNE)!=RESET){
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//clear IRQ

		bufp = &usart3_rbuf;

		if(((bufp->in - bufp->out) & ~(RBUF_SIZE-1)) == 0){
			curchar = USART_ReceiveData(USART3) & 0x1ff;
			bufp->buf[bufp->in & (RBUF_SIZE-1)] = curchar;
			bufp->in++;
		}else{
			//buffer full...
		}
	}
}

short USART1_GetChar(void){
	struct usartbuf_st *bufp = &usart1_rbuf;
	if(USART1_RBUFLEN == 0)
		return -1;
	return (bufp->buf[(bufp->out++) & (RBUF_SIZE-1)]);
}

short USART3_GetChar(void){
	struct usartbuf_st *bufp = &usart3_rbuf;
	if(USART3_RBUFLEN == 0)
		return -1;
	return (bufp->buf[(bufp->out++) & (RBUF_SIZE-1)]);
}
*/

short stmUSART1_GetNMEAString(char *retstr){
	struct usartbuf_st *bufp = &usart1_rbuf;
	short len=0;
	if(bufp->term == 0)
		return -1;

	if(bufp->buf[0] != '$'){
		stmUSART1_buf_Init();
		return -1;
	}
	len = bufp->in;
	memcpy(retstr, bufp->buf, len);
	stmUSART1_buf_Init();
	return len;
}
short stmUSART2_GetNMEAString(char *retstr){
	struct usartbuf_st *bufp = &usart2_rbuf;
	short len=0;
	if(bufp->term == 0)
		return -1;

	if(bufp->buf[0] != '$'){
		stmUSART2_buf_Init();
		//printf("N$\r\n");
		return -1;
	}
	len = bufp->in;
	memcpy(retstr, bufp->buf, len);
	stmUSART2_buf_Init();
	//printf("%s(%u)\r\n",retstr,len);
	return len;
}
short stmUSART3_GetNMEAString(char *retstr){
	struct usartbuf_st *bufp = &usart3_rbuf;
	short len=0;
	if(bufp->term == 0)
		return -1;

	if(bufp->buf[0] != '$'){
		stmUSART3_buf_Init();
		return -1;
	}
	len = bufp->in;
	memcpy(retstr, bufp->buf, len);
	stmUSART3_buf_Init();
	return len;
}

static inline bool minmea_isfield(char c) {
    return isprint((unsigned char) c) && c != ',' && c != '*';
}

bool minmea_scan(const char *sentence, const char *format, ...)
{
	int f;
    bool result = false;
    bool optional = false;
    va_list ap;
    va_start(ap, format);

    const char *field = sentence;
#define next_field() \
    do { \
        /* Progress to the next field. */ \
        while (minmea_isfield(*sentence)) \
            sentence++; \
        /* Make sure there is a field there. */ \
        if (*sentence == ',') { \
            sentence++; \
            field = sentence; \
        } else { \
            field = NULL; \
        } \
    } while (0)

    while (*format) {
        char type = *format++;

        if (type == ';') {
            // All further fields are optional.
            optional = true;
            continue;
        }

        if (!field && !optional) {
            // Field requested but we ran out if input. Bail out.
            goto parse_error;
        }

        switch (type) {
            case 'c': { // Single character field (char).
                char value = '\0';

                if (field && minmea_isfield(*field))
                    value = *field;

                *va_arg(ap, char *) = value;
            } break;

            case 'd': { // Single character direction field (int).
                int value = 0;

                if (field && minmea_isfield(*field)) {
                    switch (*field) {
                        case 'N':
                        case 'E':
                            value = 1;
                            break;
                        case 'S':
                        case 'W':
                            value = -1;
                            break;
                        default:
                            goto parse_error;
                    }
                }

                *va_arg(ap, int *) = value;
            } break;

            case 'f': { // Fractional value with scale (struct minmea_float).
                int sign = 0;
                int_least32_t value = -1;
                int_least32_t scale = 0;

                if (field) {
                    while (minmea_isfield(*field)) {
                        if (*field == '+' && !sign && value == -1) {
                            sign = 1;
                        } else if (*field == '-' && !sign && value == -1) {
                            sign = -1;
                        } else if (isdigit((unsigned char) *field)) {
                            int digit = *field - '0';
                            if (value == -1)
                                value = 0;
                            if (value > (INT_LEAST32_MAX-digit) / 10) {
                                /* we ran out of bits, what do we do? */
                                if (scale) {
                                    /* truncate extra precision */
                                    break;
                                } else {
                                    /* integer overflow. bail out. */
                                    goto parse_error;
                                }
                            }
                            value = (10 * value) + digit;
                            if (scale)
                                scale *= 10;
                        } else if (*field == '.' && scale == 0) {
                            scale = 1;
                        } else if (*field == ' ') {
                            /* Allow spaces at the start of the field. Not NMEA
                             * conformant, but some modules do this. */
                            if (sign != 0 || value != -1 || scale != 0)
                                goto parse_error;
                        } else {
                            goto parse_error;
                        }
                        field++;
                    }
                }

                if ((sign || scale) && value == -1)
                    goto parse_error;

                if (value == -1) {
                    /* No digits were scanned. */
                    value = 0;
                    scale = 0;
                } else if (scale == 0) {
                    /* No decimal point. */
                    scale = 1;
                }
                if (sign)
                    value *= sign;

                *va_arg(ap, struct minmea_float *) = (struct minmea_float) {value, scale};
            } break;

            case 'i': { // Integer value, default 0 (int).
                int value = 0;

                if (field) {
                    char *endptr;
                    value = strtol(field, &endptr, 10);
                    if (minmea_isfield(*endptr))
                        goto parse_error;
                }

                *va_arg(ap, int *) = value;
            } break;

            case 's': { // String value (char *).
                char *buf = va_arg(ap, char *);

                if (field) {
                    while (minmea_isfield(*field))
                        *buf++ = *field++;
                }

                *buf = '\0';
            } break;

            case 't': { // NMEA talker+sentence identifier (char *).
                // This field is always mandatory.
                if (!field)
                    goto parse_error;

                if (field[0] != '$')
                    goto parse_error;
                for (f=0; f<5; f++)
                    if (!minmea_isfield(field[1+f]))
                        goto parse_error;

                char *buf = va_arg(ap, char *);
                memcpy(buf, field+1, 5);
                buf[5] = '\0';
            } break;

            case 'D': { // Date (int, int, int), -1 if empty.
                struct minmea_date *date = va_arg(ap, struct minmea_date *);

                int d = -1, m = -1, y = -1;

                if (field && minmea_isfield(*field)) {
                    // Always six digits.
                    for (f=0; f<6; f++)
                        if (!isdigit((unsigned char) field[f]))
                            goto parse_error;

                    char dArr[] = {field[0], field[1], '\0'};
                    char mArr[] = {field[2], field[3], '\0'};
                    char yArr[] = {field[4], field[5], '\0'};
                    d = strtol(dArr, NULL, 10);
                    m = strtol(mArr, NULL, 10);
                    y = strtol(yArr, NULL, 10);
                }

                date->day = d;
                date->month = m;
                date->year = y;
            } break;

            case 'T': { // Time (int, int, int, int), -1 if empty.
                struct minmea_time *time_ = va_arg(ap, struct minmea_time *);

                int h = -1, i = -1, s = -1, u = -1;

                if (field && minmea_isfield(*field)) {
                    // Minimum required: integer time.
                    for (f=0; f<6; f++)
                        if (!isdigit((unsigned char) field[f]))
                            goto parse_error;

                    char hArr[] = {field[0], field[1], '\0'};
                    char iArr[] = {field[2], field[3], '\0'};
                    char sArr[] = {field[4], field[5], '\0'};
                    h = strtol(hArr, NULL, 10);
                    i = strtol(iArr, NULL, 10);
                    s = strtol(sArr, NULL, 10);
                    field += 6;

                    // Extra: fractional time. Saved as microseconds.
                    if (*field++ == '.') {
                        int value = 0;
                        int scale = 1000000;
                        while (isdigit((unsigned char) *field) && scale > 1) {
                            value = (value * 10) + (*field++ - '0');
                            scale /= 10;
                        }
                        u = value * scale;
                    } else {
                        u = 0;
                    }
                }

                time_->hours = h;
                time_->minutes = i;
                time_->seconds = s;
                time_->microseconds = u;
            } break;

            case '_': { // Ignore the field.
            } break;

            default: { // Unknown.
                goto parse_error;
            } break;
        }

        next_field();
    }

    result = true;

parse_error:
    va_end(ap);
    return result;
}

//GET UTC TIME
bool minmea_parse_gga(struct minmea_sentence_gga *frame, const char *sentence)
{
    // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    char type[6];
    int latitude_direction;
    int longitude_direction;

    if (!minmea_scan(sentence, "tTfdfdiiffcfci_",
            type,
            &frame->time,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->fix_quality,
            &frame->satellites_tracked,
            &frame->hdop,
            &frame->altitude, &frame->altitude_units,
            &frame->height, &frame->height_units,
            &frame->dgps_age))
        return false;
    if (strcmp(type+2, "GGA"))
        return false;

    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;

    return true;
}
bool minmea_parse_rmc(struct minmea_sentence_rmc *frame, const char *sentence)
{
    // $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
    char type[6];
    char validity;
    int latitude_direction;
    int longitude_direction;
    int variation_direction;
    if (!minmea_scan(sentence, "tTcfdfdffDfd",
            type,
            &frame->time,
            &validity,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->speed,
            &frame->course,
            &frame->date,
            &frame->variation, &variation_direction))
        return false;
    if (strcmp(type+2, "RMC"))
        return false;

    frame->valid = (validity == 'A');
    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;
    frame->variation.value *= variation_direction;

    return true;
}

enum minmea_sentence_id {
    MINMEA_INVALID = -1,
    MINMEA_UNKNOWN = 0,
    MINMEA_SENTENCE_RMC, //1
    MINMEA_SENTENCE_GGA,
    MINMEA_SENTENCE_GSA,
    MINMEA_SENTENCE_GLL,
    MINMEA_SENTENCE_GST,
    MINMEA_SENTENCE_GSV,
    MINMEA_SENTENCE_VTG,
};
#define MINMEA_MAX_LENGTH 80
/*
static int hex2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}
*/
uint8_t minmea_checksum(const char *sentence)
{
    // Support senteces with or without the starting dollar sign.
    if (*sentence == '$')
        sentence++;

    uint8_t checksum = 0x00;

    // The optional checksum is an XOR of all bytes between "$" and "*".
    while (*sentence && *sentence != '*')
        checksum ^= *sentence++;

    return checksum;
}

bool minmea_check(const char *sentence, bool strict)
{
    uint8_t checksum = 0x00;

    // Sequence length is limited.
    if (strlen(sentence) > MINMEA_MAX_LENGTH + 3)
        return false;

    // A valid sentence starts with "$".
    if (*sentence++ != '$')
        return false;

    // The optional checksum is an XOR of all bytes between "$" and "*".
    while (*sentence && *sentence != '*' && isprint((unsigned char) *sentence))
        checksum ^= *sentence++;

    // If checksum is present...
    if (*sentence == '*') {
        // Extract checksum.
        sentence++;
        int upper = hex2int(*sentence++);
        if (upper == -1)
            return false;
        int lower = hex2int(*sentence++);
        if (lower == -1)
            return false;
        int expected = upper << 4 | lower;

        // Check for checksum mismatch.
        if (checksum != expected)
            return false;
    } else if (strict) {
        // Discard non-checksummed frames in strict mode.
        return false;
    }

    // The only stuff allowed at this point is a newline.
    if (*sentence && strcmp(sentence, "\n") && strcmp(sentence, "\r\n"))
        return false;

    return true;
}
enum minmea_sentence_id minmea_sentence_id(const char *sentence, bool strict)
{
    if (!minmea_check(sentence, strict))
        return MINMEA_INVALID;

    char type[6];
    if (!minmea_scan(sentence, "t", type))
        return MINMEA_INVALID;

    if (!strcmp(type+2, "RMC"))
        return MINMEA_SENTENCE_RMC;
    if (!strcmp(type+2, "GGA"))
        return MINMEA_SENTENCE_GGA;
    if (!strcmp(type+2, "GSA"))
        return MINMEA_SENTENCE_GSA;
    if (!strcmp(type+2, "GLL"))
        return MINMEA_SENTENCE_GLL;
    if (!strcmp(type+2, "GST"))
        return MINMEA_SENTENCE_GST;
    if (!strcmp(type+2, "GSV"))
        return MINMEA_SENTENCE_GSV;
    if (!strcmp(type+2, "VTG"))
        return MINMEA_SENTENCE_VTG;

    return MINMEA_UNKNOWN;
}

/**
 * Rescale a fixed-point value to a different scale. Rounds towards zero.
 */
static inline int_least32_t minmea_rescale(struct minmea_float *f, int_least32_t new_scale)
{
    if (f->scale == 0)
        return 0;
    if (f->scale == new_scale)
        return f->value;
    if (f->scale > new_scale)
        return (f->value + ((f->value > 0) - (f->value < 0)) * f->scale/new_scale/2) / (f->scale/new_scale);
    else
        return f->value * (new_scale/f->scale);
}
/*
#define NAN (0.0/0.0)

// * Convert a fixed-point value to a floating-point value.
// * Returns NaN for "unknown" values.

static inline float minmea_tofloat(struct minmea_float *f)
{
    if (f->scale == 0)
        return NAN;
    return (float) f->value / (float) f->scale;
}
*/
/**
 * Convert a raw coordinate to a floating point DD.DDD... value.
 * Returns NaN for "unknown" values.
 */
static inline float minmea_tocoord(struct minmea_float *f)
{
    if (f->scale == 0)
        return NAN;
    int_least32_t degrees = f->value / (f->scale * 100);
    int_least32_t minutes = f->value % (f->scale * 100);
    return (float) degrees + (float) minutes / (60 * f->scale);
}

int minmea_gettime(struct timespec *ts, const struct minmea_date *date, const struct minmea_time *time_)
{
    if (date->year == -1 || time_->hours == -1)
        return -1;

    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    tm.tm_year = 2000 + date->year - 1900;
    tm.tm_mon = date->month - 1;
    tm.tm_mday = date->day;
    tm.tm_hour = time_->hours;
    tm.tm_min = time_->minutes;
    tm.tm_sec = time_->seconds;
/*
    time_t timestamp = gmtime(&tm);//timegm(&tm); //See README.md if your system lacks timegm().
    if (timestamp != -1) {
        ts->tv_sec = timestamp;
        ts->tv_nsec = time_->microseconds * 1000;
        return 0;
    } else {
        return -1;
    }
*/
    return 0;
}
/*
short stmGpsGetTimeOfDay(unsigned char uartId, struct minmea_sentence_rmc *prmc){ //retrun value
	short len, rxchar;
	char nmeastr[256];
    //struct minmea_sentence_rmc frame;
    struct timespec ts;

	memset(nmeastr,0x00,256);
	if(uartId==3)
		len = stmUSART3_GetNMEAString(nmeastr);
	else if(uartId==2)
		len = stmUSART2_GetNMEAString(nmeastr);
	else
		len = stmUSART1_GetNMEAString(nmeastr);
	if(len){
	    if(minmea_sentence_id(nmeastr, false) ==MINMEA_SENTENCE_RMC){
	            if (minmea_parse_rmc(prmc, nmeastr)) {
	            		prmc->time.hours += 9;
	            		if(prmc->time.hours > 24) prmc->time.hours -= 24;
	            		return len;
	            }
	    }
	}
	return 0;
}
short stmGpsGetTimeOfDayAndShow(unsigned char uartId, struct minmea_sentence_rmc *prmc){ //retrun value
	short len, rxchar;
	char nmeastr[256];
    //struct minmea_sentence_rmc frame;
    struct timespec ts;

	memset(nmeastr,0x00,256);
	if(uartId==3)
		len = stmUSART3_GetNMEAString(nmeastr);
	else if(uartId==2)
		len = stmUSART2_GetNMEAString(nmeastr);
	else
		len = stmUSART1_GetNMEAString(nmeastr);

	if(len){
		printf("%s",nmeastr);
	    if(minmea_sentence_id(nmeastr, false) ==MINMEA_SENTENCE_RMC){
	            if (minmea_parse_rmc(prmc, nmeastr)) {
	            		prmc->time.hours += 9;
	            		if(prmc->time.hours > 24) prmc->time.hours -= 24;
	                	//printf("\r\n$GPRMC:ToD=20%d.%d.%d %d:%d:%d \r\n",prmc->date.year, prmc->date.month, prmc->date.day, prmc->time.hours, prmc->time.minutes, prmc->time.seconds);
	                	//printf("\r\n");
	            }else {
	                printf("$GPRMC sentence is not parsed\n");
	                return 0;
	            }
	    }
	    return len;
	}else
		return 0;
}
*/
#if (DISPLAY_ON == DISPLAY_ON_OLED_I2C)
extern void SSD1306_OLED_showClock(unsigned h, unsigned m, unsigned s, unsigned xpos, unsigned ypos);
extern void SSD1306_OLED_showDate(unsigned y, unsigned mo, unsigned d, unsigned xpos, unsigned ypos);
#endif
char stmGpsGetTimeOfDay(unsigned char uartId, struct minmea_sentence_rmc *rmc)
{
	short len, rxchar;
	char nmeastr[256];
    struct timespec ts;

	memset(nmeastr,0x00,sizeof(struct minmea_sentence_rmc));
	if(uartId==3)
		len = stmUSART3_GetNMEAString(nmeastr);
	else if(uartId==2)
		len = stmUSART2_GetNMEAString(nmeastr);
	else
		len = stmUSART1_GetNMEAString(nmeastr);

	if(len > 0){
		//printf(">%s",nmeastr);
	    switch (minmea_sentence_id(nmeastr, false)) {
	        case MINMEA_SENTENCE_RMC: {
	            if (minmea_parse_rmc(rmc, nmeastr)) {
/*	            	printf("$GPRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\r\n",
	            							frame.latitude.value, frame.latitude.scale,
	                                        frame.longitude.value, frame.longitude.scale,
	                                        frame.speed.value, frame.speed.scale);
	                printf("$RMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\r\n",
	                                        minmea_rescale(&frame.latitude, 1000),
	                                        minmea_rescale(&frame.longitude, 1000),
	                                        minmea_rescale(&frame.speed, 1000));
	                printf("$GPRMC floating point degree coordinates and speed: (%f,%f) %f\r\n",
	                                        minmea_tocoord(&frame.latitude),
	                                        minmea_tocoord(&frame.longitude),
	                                        minmea_tofloat(&frame.speed));
*/
	                //if(minmea_gettime(&ts,&frame.date,&frame.time)){
	            		rmc->time.hours += 9;
	            		if(rmc->time.hours > 24)
	            			rmc->time.hours -= 24;

	                	printf("\r\n$GPRMC:ToD=20%d.%d.%d %d:%d:%d \r\n",rmc->date.year, rmc->date.month, rmc->date.day, rmc->time.hours, rmc->time.minutes, rmc->time.seconds);
	                	//printf("\r\n");
	                	//printf("$GPRMC:Time=%d:%d:%d(%dus) \r\n", frame.time.hours,frame.time.minutes,frame.time.seconds, frame.time.microseconds);
	                //}
	            		return ERR_OK;; //OK
	            }else {
	                printf("$GPRMC sentence is not parsed\n");
	                return ERR_FAIL; //ERR
	            }
	        }
	        break;

/*	    case MINMEA_SENTENCE_GGA: {
	                    struct minmea_sentence_gga frame;
	                    if (minmea_parse_gga(&frame, nmeastr)) {
	                        //printf( "$GPGGA: fix quality: %d\n", frame.fix_quality);
	                        printf("$GPGGA:Time=%d:%d:%d(%dus) \r\n", frame.time.hours,frame.time.minutes,frame.time.seconds, frame.time.microseconds);
	                    }
	                    else {
	                        printf("$GPGGA sentence is not parsed\n");
	                    }
	                }
	                break;
*/	    }
	}
	return ERR_FAIL;
}
char stmGpsDisplayTimeOfDay(unsigned char uartId, struct minmea_sentence_rmc *rmc)
{
	short len, rxchar;
	char nmeastr[256];
    struct timespec ts;
    unsigned ypos = 2;

    if(rmc != NULL){
		#if (DISPLAY_ON == DISPLAY_ON_MAX7219_SPI)
       		stmMax7219_showClock(rmc->time.hours, rmc->time.minutes, rmc->time.seconds) ;
		#else if(DISPLAY_ON == DISPLAY_ON_OLED_I2C)
       		SSD1306_OLED_showDate(rmc->date.year, rmc->date.month, rmc->date.day, 0, 1);
       		SSD1306_OLED_showClock(rmc->time.hours, rmc->time.minutes, rmc->time.seconds,0, ypos);
			SSD1306_OLED_printString_8X8("GPSOK",10,0,5);
		#endif
	}else{
		#if (DISPLAY_ON == DISPLAY_ON_MAX7219_SPI)
			stmMax7219_showClock(0, 0, 0) ;
		#else if(DISPLAY_ON == DISPLAY_ON_OLED_I2C)
			SSD1306_OLED_showClock(0, 0, 0, 0,ypos);
			SSD1306_OLED_printString_8X8("NoGPS",8,3,5);
		#endif
	}
    return ERR_OK;
}

char stmGpsGetTimeOfDayAndDisplay(unsigned char uartId, struct minmea_sentence_rmc *rmc)
{
	short len, rxchar;
	char nmeastr[256];
    struct timespec ts;
    unsigned ypos = 3;

    if(stmGpsGetTimeOfDay(uartId, rmc) == ERR_OK)
    	stmGpsDisplayTimeOfDay(uartId, rmc);
    else {
    	stmGpsDisplayTimeOfDay(uartId, NULL);
	}
}

void stmGpsConfig(unsigned char uartId){
	if(uartId==3)
		stmUSART3ShimConf(9600); //Interrupt Mode. Rx Only
	else if(uartId==2)
		stmUSART2ShimConf(9600); //Interrupt Mode. Rx Only
	else
		stmUSART1ShimConf(9600); //Interrupt Mode. Rx Only

	printf("GPS>GpsConf(9600bps).\r\n");
}

void stmGpsLoop(unsigned char uartId)
{
	int i;
	short len, rxchar;
	char nmeastr[256];
	struct minmea_sentence_rmc rmc;

	printf(">GPS Test with Usart%u<\r\n", uartId);

	stmGpsConfig(uartId);

	delayms(1000);

	//1Mbps, 8 bit mode
	//stmSPI3_Config(nCS);//(SSI_FRF_MOTO_MODE_0,1000000,8, SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_6); //use PD6 of nCS1
    //nCS7219_1;//nCs=1
	//max7219_Init(1);//2);
	//max7219_config();

	len = 0;
	while(1) //main loop
    {
		memset(nmeastr,0x00,256);
		if(uartId==3)
			len = stmUSART3_GetNMEAString(nmeastr);
		else if(uartId==2)
			len = stmUSART2_GetNMEAString(nmeastr);
		else
			len = stmUSART1_GetNMEAString(nmeastr);

		if(len > 0){
			printf("%u>%s",len,nmeastr);
			stmGpsGetTimeOfDayAndDisplay(uartId, &rmc);
			SSD1306_OLED_printString_8X8("GPS RX...",1,3,15); //Print the String
		}
		delayms(100);


/*
		rxchar = USART3_GetChar();
		if(rxchar >=0){
			printf("%c",(char)rxchar);
*/
	}
}


