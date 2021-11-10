
/**************************************************************************//*****
 * @file     printf.c
 * @brief    Implementation of several stdio.h methods, such as printf(),
 *           sprintf() and so on. This reduces the memory footprint of the
 *           binary when using those methods, compared to the libc implementation.
 ********************************************************************************/
//#include "semihosting.h"

#include <ctype.h>
#include <stdarg.h>
#include <stddef.h>
#include <inttypes.h>
#include <string.h>
#include <limits.h>
#include <stdio.h>
#include "yInc.h"
#include "stm32f10x_usart.h"
//#include "lwip/include/lwipopts.h"

//For scanf
struct _usartXrxbuf g_usartXrxbuf;
struct _usart1rxbuf g_usart1rxbuf;
struct _usart2rxbuf g_usart2rxbuf;
struct _usart3rxbuf g_usart3rxbuf;
static char *SPRINTF_buffer;

/** Maximum string size allowed (in bytes). */
#define MAX_STRING_SIZE        stmUART_RBUF_SIZE //=256//80// 120
/**
 * @brief  Transmit a char, if you want to use printf(),
 *         you need implement this function
 *
 * @param  pStr	Storage string.
 * @param  c    Character to write.

void PrintCharBsp(char c)
{
	SH_SendChar(c);
}
*/
#if (DEBUG_CONSOLE == IS_USB)
extern unsigned short VCP_DataTx (unsigned char* Buf, unsigned long Len);
#endif
//USARTX
void PrintCharBsp(char c) {
	unsigned char ch;
	ch = c;
	//Place your implementation of fputc here e.g. write a character to the USART
#if (DEBUG_CONSOLE == IS_USB)
	VCP_DataTx(&ch,1); ////VCP
	return;
#else
	USART_SendData(USARTX, (unsigned short) ch);
	// Loop until the end of transmission
	while (USART_GetFlagStatus(USARTX, USART_FLAG_TC) == RESET) {}//while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}
#endif
}



//==================================Added to support LONGLONG ==========
#define LONGFLAG     0x00000001
#define LONGLONGFLAG 0x00000002
#define HALFFLAG     0x00000004
#define HALFHALFFLAG 0x00000008
#define SIZETFLAG    0x00000010
#define ALTFLAG      0x00000020
#define CAPSFLAG     0x00000040
#define SHOWSIGNFLAG 0x00000080
#define SIGNEDFLAG   0x00000100
#define LEFTFORMATFLAG 0x00000200
#define LEADZEROFLAG 0x00000400

static char *longlong_to_string(char *buf, unsigned long long n, int len, uint flag)
{
	int pos = len;
	int negative = 0;

	if((flag & SIGNEDFLAG) && (long long)n < 0) {
		negative = 1;
		n = -n;
	}

	buf[--pos] = 0;

	/* only do the math if the number is >= 10 */
	while(n >= 10) {
		int digit = n % 10;

		n /= 10;

		buf[--pos] = digit + '0';
	}
	buf[--pos] = n + '0';

	if(negative)
		buf[--pos] = '-';
	else if((flag & SHOWSIGNFLAG))
		buf[--pos] = '+';

	return &buf[pos];
}

static char *longlong_to_hexstring(char *buf, unsigned long long u, int len, uint flag)
{
	int pos = len;
	static const char hextable[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
	static const char hextable_caps[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
	const char *table;

	if((flag & CAPSFLAG))
		table = hextable_caps;
	else
		table = hextable;

	buf[--pos] = 0;
	do {
		unsigned int digit = u % 16;
		u /= 16;

		buf[--pos] = table[digit];
	} while(u != 0);

	return &buf[pos];
}
//=========================================

/** Required for proper compilation. */
//struct _reent r = {0, (FILE *) 0, (FILE *) 1, (FILE *) 0};
//struct _reent *_impure_ptr = &r;

/**
 * @brief  Writes a character inside the given string. Returns 1.
 *
 * @param  pStr	Storage string.
 * @param  c    Character to write.
*/
int PutChar(char *pStr, char c)
{
    *pStr = c;
    return 1;
}


/**
 * @brief  Writes a string inside the given string.
 *
 * @param  pStr     Storage string.
 * @param  pSource  Source string.
 * @return  The size of the written
 */
signed int PutString(char *pStr, const char *pSource)
{
    signed int num = 0;

    while (*pSource != 0) {

        *pStr++ = *pSource++;
        num++;
    }

    return num;
}


/**
 * @brief  Writes an unsigned int inside the given string, using the provided fill &
 *         width parameters.
 *
 * @param  pStr  Storage string.
 * @param  fill  Fill character.
 * @param  width  Minimum integer width.
 * @param  value  Integer value.
 */
signed int PutUnsignedInt(
    char *pStr,
    char fill,
    signed int width,
    unsigned int value)
{
    signed int num = 0;

    /* Take current digit into account when calculating width */
    width--;

    /* Recursively write upper digits */
    if ((value / 10) > 0) {

        num = PutUnsignedInt(pStr, fill, width, value / 10);
        pStr += num;
    }

    /* Write filler characters */
    else {

        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }
    }

    /* Write lower digit */
    num += PutChar(pStr, (value % 10) + '0');

    return num;
}


/**
 * @brief  Writes a signed int inside the given string, using the provided fill & width
 *         parameters.
 *
 * @param pStr   Storage string.
 * @param fill   Fill character.
 * @param width  Minimum integer width.
 * @param value  Signed integer value.
 */
signed int PutSignedInt(
    char *pStr,
    char fill,
    signed int width,
    signed int value)
{
    signed int num = 0;
    unsigned int absolute;

    /* Compute absolute value */
    if (value < 0) {

        absolute = -value;
    }
    else {

        absolute = value;
    }

    /* Take current digit into account when calculating width */
    width--;

    /* Recursively write upper digits */
    if ((absolute / 10) > 0) {

        if (value < 0) {

            num = PutSignedInt(pStr, fill, width, -(absolute / 10));
        }
        else {

            num = PutSignedInt(pStr, fill, width, absolute / 10);
        }
        pStr += num;
    }
    else {

        /* Reserve space for sign */
        if (value < 0) {

            width--;
        }

        /* Write filler characters */
        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }

        /* Write sign */
        if (value < 0) {

            num += PutChar(pStr, '-');
            pStr++;
        }
    }

    /* Write lower digit */
    num += PutChar(pStr, (absolute % 10) + '0');

    return num;
}


/**
 * @brief  Writes an hexadecimal value into a string, using the given fill, width &
 *         capital parameters.
 *
 * @param pStr   Storage string.
 * @param fill   Fill character.
 * @param width  Minimum integer width.
 * @param maj    Indicates if the letters must be printed in lower- or upper-case.
 * @param value  Hexadecimal value.
 *
 * @return  The number of char written
 */
signed int PutHexa(
    char *pStr,
    char fill,
    signed int width,
    unsigned char maj,
    unsigned int value)
{
    signed int num = 0;

    /* Decrement width */
    width--;

    /* Recursively output upper digits */
    if ((value >> 4) > 0) {

        num += PutHexa(pStr, fill, width, maj, value >> 4);
        pStr += num;
    }
    /* Write filler chars */
    else {

        while (width > 0) {

            PutChar(pStr, fill);
            pStr++;
            num++;
            width--;
        }
    }

    /* Write current digit */
    if ((value & 0xF) < 10) {

        PutChar(pStr, (value & 0xF) + '0');
    }
    else if (maj) {

        PutChar(pStr, (value & 0xF) - 10 + 'A');
    }
    else {

        PutChar(pStr, (value & 0xF) - 10 + 'a');
    }
    num++;

    return num;
}

signed int PutUnsignedLongLong(
    char *pStr,
    char fill,
    signed int width,
    unsigned long long llvalue)
{
	char num_buffer_str[32];
    signed int num = 0;
    const char *s;
    //flags = SIGNEDFLAG;
    s = longlong_to_string(num_buffer_str, llvalue, sizeof(num_buffer_str), SIGNEDFLAG);//flags);
    strcpy(pStr, s);
    num = strlen(s);

    return num;
}

/* Global Functions ----------------------------------------------------------- */


/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pStr    Destination string.
 * @param length  Length of Destination string.
 * @param pFormat Format string.
 * @param ap      Argument list.
 *
 * @return  The number of characters written.
*/
signed int vsnprintf(char *pStr, size_t length, const char *pFormat, va_list ap)
{
    char          fill;
    unsigned char width;
    signed int    num = 0;
    signed int    size = 0;

    // Clear the string
    if (pStr) {

        *pStr = 0;
    }

    // Phase string
    while (*pFormat != 0 && size < length) {

        // Normal character
        if (*pFormat != '%') {

            *pStr++ = *pFormat++;
            size++;
        }
        // Escaped '%'
        else if (*(pFormat+1) == '%') {

            *pStr++ = '%';
            pFormat += 2;
            size++;
        }
        // Token delimiter
        else {

            fill = ' ';
            width = 0;
            pFormat++;

            // Parse filler
            if (*pFormat == '0') {

                fill = '0';
                pFormat++;
            }

            // Parse width
            while ((*pFormat >= '0') && (*pFormat <= '9')) {

                width = (width*10) + *pFormat-'0';
                pFormat++;
            }

            // Check if there is enough space
            if (size + width > length) {

                width = length - size;
            }

            // Parse type
            switch (*pFormat) {
            case 'd':
            case 'i':
            	num = PutSignedInt(pStr, fill, width, va_arg(ap, signed int));
            	break;
            case 'u':
            	num = PutUnsignedInt(pStr, fill, width, va_arg(ap, unsigned int));
            	break;
            case 'x':
            	num = PutHexa(pStr, fill, width, 0, va_arg(ap, unsigned int));
            	break;
            case 'X':
            	num = PutHexa(pStr, fill, width, 1, va_arg(ap, unsigned int));
            	break;
            case 's':
            	num = PutString(pStr, va_arg(ap, char *));
            	break;
            case 'c':
            	num = PutChar(pStr, va_arg(ap, unsigned int));
            	break;
            case 'L'://added
            	num = PutUnsignedLongLong(pStr, fill, width, va_arg(ap, unsigned long long));
            	break;

            default:
                return EOF;
            }

            pFormat++;
            pStr += num;
            size += num;
        }
    }

    // NULL-terminated (final \0 is not counted)
    if (size < length) {

        *pStr = 0;
    }
    else {

        *(--pStr) = 0;
        size--;
    }

    return size;
}


/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pStr    Destination string.
 * @param length  Length of Destination string.
 * @param pFormat Format string.
 * @param ...     Other arguments
 *
 * @return  The number of characters written.
*/
signed int snprintf(char *pString, size_t length, const char *pFormat, ...)
{
    va_list    ap;
    signed int rc;

    va_start(ap, pFormat);
    rc = vsnprintf(pString, length, pFormat, ap);
    va_end(ap);

    return rc;
}


/**
 * @brief  Stores the result of a formatted string into another string. Format
 *         arguments are given in a va_list instance.
 *
 * @param pString  Destination string.
 * @param length   Length of Destination string.
 * @param pFormat  Format string.
 * @param ap       Argument list.
 *
 * @return  The number of characters written.
*/
signed int vsprintf(char *pString, const char *pFormat, va_list ap)
{
   return vsnprintf(pString, MAX_STRING_SIZE, pFormat, ap);
}

/*
 * @brief  Outputs a formatted string on the given stream. Format arguments are given
 *         in a va_list instance.
 *
 * @param pStream  Output stream.
 * @param pFormat  Format string
 * @param ap       Argument list.
*/

signed int stm_vfprintf(FILE *pStream, const char *pFormat, va_list ap)
{
    char pStr[MAX_STRING_SIZE];
    char pError[] = "stdio.c: increase MAX_STRING_SIZE\n\r";

    // Write formatted string in buffer
    if (vsprintf(pStr, pFormat, ap) >= MAX_STRING_SIZE) {

        fputs(pError, stderr);
        while (1); //Increase MAX_STRING_SIZE
    }

    // Display string
    return fputs(pStr, pStream);
}


/**
 * @brief  Outputs a formatted string on the DBGU stream. Format arguments are given
 *         in a va_list instance.
 *
 * @param pFormat  Format string.
 * @param ap  Argument list.
*/
signed int vprintf(const char *pFormat, va_list ap)
{
    return stm_vfprintf(stdout, pFormat, ap);
}


/**
 * @brief  Outputs a formatted string on the given stream, using a variable
 *         number of arguments.
 *
 * @param pStream  Output stream.
 * @param pFormat  Format string.
*/
signed int fprintf(FILE *pStream, const char *pFormat, ...)
{
    va_list ap;
    signed int result;

    // Forward call to stm_stm_vfprintf
    va_start(ap, pFormat);
    result = stm_vfprintf(pStream, pFormat, ap);
    va_end(ap);

    return result;
}
/*
signed int yprintf(const char *pFormat, ...)
{
    va_list ap;
    signed int result;

    // Forward call to vprintf
    va_start(ap, pFormat);
    result = vprintf(pFormat, ap);
    va_end(ap);

    return result;
}
*/
/**
 * @brief  Outputs a formatted string on the DBGU stream, using a variable number of
 *         arguments.
 *
 * @param  pFormat  Format string.
*/

signed int printf(const char *pFormat, ...)
{
    va_list ap;
    signed int result;

    // Forward call to vprintf
    va_start(ap, pFormat);
    result = vprintf(pFormat, ap);
    va_end(ap);

    return result;
}


/**
 * @brief  Writes a formatted string inside another string.
 *
 * @param pStr     torage string.
 * @param pFormat  Format string.
*/
signed int sprintf(char *pStr, const char *pFormat, ...)
{
    va_list ap;
    signed int result;

    // Forward call to vsprintf
    va_start(ap, pFormat);
    result = vsprintf(pStr, pFormat, ap);
    va_end(ap);

    return result;
}


/**
 * @brief  Outputs a string on stdout.
 *
 * @param pStr  String to output.
*/
signed int puts(const char *pStr)
{
    return fputs(pStr, stdout);
}


/**
 * @brief  Implementation of fputc using the DBGU as the standard output. Required
 *         for printf().
 *
 * @param c        Character to write.
 * @param pStream  Output stream.
 * @param The character written if successful, or -1 if the output stream is
 *        not stdout or stderr.
 */
signed int fputc(signed int c, FILE *pStream)
{
    if ((pStream == stdout) || (pStream == stderr)) {

    	PrintCharBsp(c);

        return c;
    }
    else {

        return EOF;
    }
}
signed int fputc3(signed int c, FILE *pStream)
{
    if ((pStream == stdout) || (pStream == stderr)) {

    	PrintCharBsp3(c);

        return c;
    }
    else {

        return EOF;
    }
}

/**
 * @brief  Implementation of fputs using the DBGU as the standard output. Required
 *         for printf().
 *
 * @param pStr     String to write.
 * @param pStream  Output stream.
 *
 * @return  Number of characters written if successful, or -1 if the output
 *          stream is not stdout or stderr.
 */
signed int fputs(const char *pStr, FILE *pStream)
{
    signed int num = 0;

    while (*pStr != 0) {

        if (fputc(*pStr, pStream) == -1) {

            return -1;
        }
        num++;
        pStr++;
    }

    return num;
}
signed int fputs3(const char *pStr, FILE *pStream)
{
    signed int num = 0;

    while (*pStr != 0) {

        if (fputc3(*pStr, pStream) == -1) {

            return -1;
        }
        num++;
        pStr++;
    }

    return num;
}
//=============== Scanf ======================================================
/*
// The UART interrupt handler.
void UART0_IRQHandler(void)
{
    unsigned long ulStatus;
    unsigned char iChar;
    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //

    while(UARTCharsAvail(UART0_BASE))
    {
    	iChar =  UARTCharGetNonBlocking(UART0_BASE);
    	gline [glineCharCnt++] = iChar;
    	UARTCharPutNonBlocking(UART0_BASE, iChar); //echoback
    	if(iChar == '\r'){
    		 	gline[glineCharCnt++] = '\n';
    		 	UARTCharPutNonBlocking(UART0_BASE,'\n');
    		 	glineInputDone = 1;
    	}
    }
}
*/

//Blocked function.
uint8_t UsartXGetChar(void){
	char ch = 0;
     while ( USART_GetFlagStatus(USARTX, USART_FLAG_RXNE) == RESET); //blocked until received.
     ch = (uint8_t)USART_ReceiveData(USARTX);
     PrintCharBsp(ch); //echoback
     return (uint8_t)ch;
}

uint8_t Usart1GetChar(void){
	char ch = 0;
     while ( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); //blocked until received.
     ch = (uint8_t)USART_ReceiveData(USART1);
     PrintCharBsp(ch); //echoback
     return (uint8_t)ch;
}
uint8_t Usart2GetChar(void){
	char ch = 0;
     while ( USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); //blocked until received.
     ch = (uint8_t)USART_ReceiveData(USART2);
     PrintCharBsp(ch); //echoback
     return (uint8_t)ch;
}
uint8_t Usart3GetChar(void){
	char ch = 0;
     while ( USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET); //blocked until received.
     ch = (uint8_t)USART_ReceiveData(USART3);
     PrintCharBsp(ch); //echoback
     return (uint8_t)ch;
}

//nonBlocking function. ====================================
short int UsartXGetCharNonBlocking(void){
	char ch = 0;
    if(USART_GetFlagStatus(USARTX, USART_FLAG_RXNE) != RESET){
    	ch = USART_ReceiveData(USARTX);
    	PrintCharBsp(ch); //echoback
    	return ch;
    }else {
		return -1;
	}
}

//==========================================================
short int Usart1GetCharNonBlocking(void){
	char ch = 0;
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET){
    	ch = USART_ReceiveData(USART1);
    	PrintCharBsp(ch); //echoback
    	return ch;
    }else {
		return -1;
	}
}
short int Usart2GetCharNonBlocking(void){
	char ch = 0;
    if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET){
    	ch = USART_ReceiveData(USART2);
    	PrintCharBsp(ch); //echoback
    	return ch;
    }else {
		return -1;
	}
}
short int Usart3GetCharNonBlocking(void){
	char ch = 0;
    if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET){
    	ch = USART_ReceiveData(USART3);
    	PrintCharBsp(ch); //echoback
    	return ch;
    }else {
		return -1;
	}
}

//===========================================
int UsartXGetStr(char *str){
	int count = 0;
	char ch = 0;
	char *ptr;

	ptr = str;

	while(count <= MAX_STRING_SIZE )//get string
	{
		count++;
		ch = UsartXGetChar();
		if(ch != '\n' && ch != '\r') *ptr++ = ch;
		else
			break;

	}
	*ptr = '\0';//end of string
	return count;

}
/*
int Usart1GetStr(char *str){
	int count = 0;
	char ch = 0;
	char *ptr;

	ptr = str;

	while(count <= MAX_STRING_SIZE )//get string
	{
		count++;
		ch = Usart1GetChar();
		if(ch != '\n' && ch != '\r') *ptr++ = ch;
		else
			break;

	}
	*ptr = '\0';//end of string
	return count;

}
int Usart2GetStr(char *str){
	int count = 0;
	char ch = 0;
	char *ptr;

	ptr = str;

	while(count <= MAX_STRING_SIZE )//get string
	{
		count++;
		ch = Usart2GetChar();
		if(ch != '\n' && ch != '\r') *ptr++ = ch;
		else
			break;

	}
	*ptr = '\0';//end of string
	return count;

}
int Usart3GetStr(char *str){
	int count = 0;
	char ch = 0;
	char *ptr;

	ptr = str;

	while(count <= MAX_STRING_SIZE )//get string
	{
		count++;
		ch = Usart3GetChar();
		if(ch != '\n' && ch != '\r') *ptr++ = ch;
		else
			break;

	}
	*ptr = '\0';//end of string
	return count;

}
*/


void UsartXGetStrNonBlockingInit(){
	g_usartXrxbuf.pos = 0;
}
int UsartXGetStrNonBlocking(){
	short int retch;
	char retlen;

	retch = UsartXGetCharNonBlocking();

	if(retch == -1) return;

	if(retch != '\n' && retch != '\r'){
		g_usartXrxbuf.buf[g_usartXrxbuf.pos] = retch;
		g_usartXrxbuf.pos++;
		return -1;
	}else{
		g_usartXrxbuf.buf[g_usartXrxbuf.pos]= '\0';//end of string
		retlen = g_usartXrxbuf.pos;
		g_usartXrxbuf.pos = 0;//reset for next.
		return retlen;
	}
}

//========================
void Usart1GetStrNonBlockingInit(){
	g_usart1rxbuf.pos = 0;
}
int Usart1GetStrNonBlocking(){
	short int retch;
	char retlen;

	retch = Usart1GetCharNonBlocking();

	if(retch == -1) return;

	if(retch != '\n' && retch != '\r'){
		g_usart1rxbuf.buf[g_usart1rxbuf.pos] = retch;
		g_usart1rxbuf.pos++;
		return -1;
	}else{
		g_usart1rxbuf.buf[g_usart1rxbuf.pos]= '\0';//end of string
		retlen = g_usart1rxbuf.pos;
		g_usart1rxbuf.pos = 0;//reset for next.
		return retlen;
	}
}
int Usart2GetStrNonBlocking(){
	short int retch;
	char retlen;

	retch = Usart2GetCharNonBlocking();

	if(retch == -1) return;

	if(retch != '\n' && retch != '\r'){
		g_usart2rxbuf.buf[g_usart1rxbuf.pos] = retch;
		g_usart2rxbuf.pos++;
		return -1;
	}else{
		g_usart2rxbuf.buf[g_usart1rxbuf.pos]= '\0';//end of string
		retlen = g_usart1rxbuf.pos;
		g_usart2rxbuf.pos = 0;//reset for next.
		return retlen;
	}
}
//
void Usart3GetStrNonBlockingInit(){
	g_usart3rxbuf.pos = 0;
}
int Usart3GetStrNonBlocking(){
	short int retch;
	char retlen;

	retch = Usart3GetCharNonBlocking();

	if(retch == -1) return;

	if(retch != '\n' && retch != '\r'){
		g_usart3rxbuf.buf[g_usart3rxbuf.pos] = retch;
		g_usart3rxbuf.pos++;
		return -1;
	}else{
		g_usart3rxbuf.buf[g_usart3rxbuf.pos]= '\0';//end of string
		retlen = g_usart3rxbuf.pos;
		g_usart3rxbuf.pos = 0;//reset for next.
		return retlen;
	}
}


//============================= TBD ============================================================
//This will be very slow due to the implementation of getc().
char *fgets(char *s, int n, FILE *f)
{
	int ch;
	char *p = s;

	while (n > 1) {
		ch = getc(f);
		if (ch == EOF) {
			*p = '\0';
			return NULL;
		}
		*p++ = ch;
		n--;
		if (ch == '\n')
			break;
	}
	if (n)
		*p = '\0';

	return s;
}

//vsscanf(), from which the rest of the scanf()family is built
#ifndef LONG_BIT
#define LONG_BIT (CHAR_BIT*sizeof(long))
#endif

enum flags {
	FL_SPLAT = 0x01,	/* Drop the value, do not assign */
	FL_INV   = 0x02,	/* Character-set with inverse */
	FL_WIDTH = 0x04,	/* Field width specified */
	FL_MINUS = 0x08,	/* Negative number */
};

enum ranks {
	rank_char     = -2,
	rank_short    = -1,
	rank_int      = 0,
	rank_long     = 1,
	rank_longlong = 2,
	rank_ptr      = INT_MAX	/* Special value used for pointers */
};

#define MIN_RANK	rank_char
#define MAX_RANK	rank_longlong

#define INTMAX_RANK	rank_longlong
#define SIZE_T_RANK	rank_long
#define PTRDIFF_T_RANK	rank_long

enum bail {
	bail_none = 0,		/* No error condition */
	bail_eof,		/* Hit EOF */
	bail_err		/* Conversion mismatch */
};

static inline const char *skipspace(const char *p)
{
	while (isspace((unsigned char)*p))
		p++;
	return p;
}

#undef set_bit
static inline void set_bit(unsigned long *bitmap, unsigned int bit)
{
	bitmap[bit / LONG_BIT] |= 1UL << (bit % LONG_BIT);
}

#undef test_bit
static inline int test_bit(unsigned long *bitmap, unsigned int bit)
{
	return (int)(bitmap[bit / LONG_BIT] >> (bit % LONG_BIT)) & 1;
}

int vsscanf(const char *buffer, const char *format, va_list ap)
{
	const char *p = format;
	char ch;
	unsigned char uc;
	const char *q = buffer;
	const char *qq;
	uintmax_t val = 0;
	int rank = rank_int;	/* Default rank */
	unsigned int width = UINT_MAX;
	int base;
	enum flags flags = 0;
	enum {
		st_normal,	/* Ground state */
		st_flags,	/* Special flags */
		st_width,	/* Field width */
		st_modifiers,	/* Length or conversion modifiers */
		st_match_init,	/* Initial state of %[ sequence */
		st_match,	/* Main state of %[ sequence */
		st_match_range,	/* After - in a %[ sequence */
	} state = st_normal;
	char *sarg = NULL;	/* %s %c or %[ string argument */
	enum bail bail = bail_none;
	int sign;
	int converted = 0;	/* Successful conversions */
	unsigned long matchmap[((1 << CHAR_BIT) + (LONG_BIT - 1)) / LONG_BIT];
	int matchinv = 0;	/* Is match map inverted? */
	unsigned char range_start = 0;
	(void)sign;

	while ((ch = *p++) && !bail) {
		switch (state) {
		case st_normal:
			if (ch == '%') {
				state = st_flags;
				flags = 0;
				rank = rank_int;
				width = UINT_MAX;
			} else if (isspace((unsigned char)ch)) {
				q = skipspace(q);
			} else {
				if (*q == ch)
					q++;
				else
					bail = bail_err; /* Match failure */
			}
			break;

		case st_flags:
			switch (ch) {
			case '*':
				flags |= FL_SPLAT;
				break;
			case '0'...'9':
				width = (ch - '0');
				state = st_width;
				flags |= FL_WIDTH;
				break;
			default:
				state = st_modifiers;
				p--;	/* Process this character again */
				break;
			}
			break;

		case st_width:
			if (ch >= '0' && ch <= '9') {
				width = width * 10 + (ch - '0');
			} else {
				state = st_modifiers;
				p--;	/* Process this character again */
			}
			break;

		case st_modifiers:
			switch (ch) {
				/* Length modifiers - nonterminal sequences */
			case 'h':
				rank--;	/* Shorter rank */
				break;
			case 'l':
				rank++;	/* Longer rank */
				break;
			case 'j':
				rank = INTMAX_RANK;
				break;
			case 'z':
				rank = SIZE_T_RANK;
				break;
			case 't':
				rank = PTRDIFF_T_RANK;
				break;
			case 'L':
			case 'q':
				rank = rank_longlong;	/* long double/long long */
				break;

			default:
				/* Output modifiers - terminal sequences */
				/* Next state will be normal */
				state = st_normal;

				/* Canonicalize rank */
				if (rank < MIN_RANK)
					rank = MIN_RANK;
				else if (rank > MAX_RANK)
					rank = MAX_RANK;

				switch (ch) {
				case 'P':	/* Upper case pointer */
				case 'p':	/* Pointer */
					rank = rank_ptr;
					base = 0;
					sign = 0;
					goto scan_int;

				case 'i':	/* Base-independent integer */
					base = 0;
					sign = 1;
					goto scan_int;

				case 'd':	/* Decimal integer */
					base = 10;
					sign = 1;
					goto scan_int;

				case 'o':	/* Octal integer */
					base = 8;
					sign = 0;
					goto scan_int;

				case 'u':	/* Unsigned decimal integer */
					base = 10;
					sign = 0;
					goto scan_int;

				case 'x':	/* Hexadecimal integer */
				case 'X':
					base = 16;
					sign = 0;
					goto scan_int;

				case 'n':	/* # of characters consumed */
					val = (q - buffer);
					goto set_integer;

				      scan_int:
					q = skipspace(q);
					if (!*q) {
						bail = bail_eof;
						break;
					}
					val =
					    strntoumax(q, (char **)&qq, base,
						       width);
					if (qq == q) {
						bail = bail_err;
						break;
					}
					q = qq;
					if (!(flags & FL_SPLAT))
						converted++;
					/* fall through */

				      set_integer:
					if (!(flags & FL_SPLAT)) {
						switch (rank) {
						case rank_char:
							*va_arg(ap,unsigned char *)= val;
							break;
						case rank_short:
							*va_arg(ap,unsigned short *) = val;
							break;
						case rank_int:
							*va_arg(ap,unsigned int *) = val;
							break;
						case rank_long:
							*va_arg(ap,unsigned long *)	= val;
							break;
						case rank_longlong:
							*va_arg(ap,unsigned long long *) = val;
							break;
						case rank_ptr:
							*va_arg(ap, void **) =(void *)(uintptr_t)val;
							break;
						}
					}
					break;

				case 'c':	/* Character */
					/* Default width == 1 */
					width = (flags & FL_WIDTH) ? width : 1;
					if (flags & FL_SPLAT) {
						while (width--) {
							if (!*q) {
								bail = bail_eof;
								break;
							}
						}
					} else {
						sarg = va_arg(ap, char *);
						while (width--) {
							if (!*q) {
								bail = bail_eof;
								break;
							}
							*sarg++ = *q++;
						}
						if (!bail)
							converted++;
					}
					break;

				case 's':	/* String */
					uc = 1;	/* Anything nonzero */
					if (flags & FL_SPLAT) {
						while (width-- && (uc = *q) &&
						       !isspace(uc)) {
							q++;
						}
					} else {
						char *sp;
						sp = sarg = va_arg(ap, char *);
						while (width-- && (uc = *q) &&
						       !isspace(uc)) {
							*sp++ = uc;
							q++;
						}
						if (sarg != sp) {
							/* Terminate output */
							*sp = '\0';
							converted++;
						}
					}
					if (!uc)
						bail = bail_eof;
					break;

				case '[':	/* Character range */
					sarg = (flags & FL_SPLAT) ? NULL
						: va_arg(ap, char *);
					state = st_match_init;
					matchinv = 0;
					memset(matchmap, 0, sizeof matchmap);
					break;

				case '%':	/* %% sequence */
					if (*q == '%')
						q++;
					else
						bail = bail_err;
					break;

				default:	/* Anything else */
					/* Unknown sequence */
					bail = bail_err;
					break;
				}
			}
			break;

		case st_match_init:	/* Initial state for %[ match */
			if (ch == '^' && !(flags & FL_INV)) {
				matchinv = 1;
			} else {
				set_bit(matchmap, (unsigned char)ch);
				state = st_match;
			}
			break;

		case st_match:	/* Main state for %[ match */
			if (ch == ']') {
				goto match_run;
			} else if (ch == '-') {
				range_start = (unsigned char)ch;
				state = st_match_range;
			} else {
				set_bit(matchmap, (unsigned char)ch);
			}
			break;

		case st_match_range:	/* %[ match after - */
			if (ch == ']') {
				/* - was last character */
				set_bit(matchmap, (unsigned char)'-');
				goto match_run;
			} else {
				int i;
				for (i = range_start; i < (unsigned char)ch;
				     i++)
					set_bit(matchmap, i);
				state = st_match;
			}
			break;

		      match_run:	/* Match expression finished */
			qq = q;
			uc = 1;	/* Anything nonzero */
			while (width && (uc = *q)
			       && test_bit(matchmap, uc)^matchinv) {
				if (sarg)
					*sarg++ = uc;
				q++;
			}
			if (q != qq && sarg) {
				*sarg = '\0';
				converted++;
			} else {
				bail = bail_err;
			}
			if (!uc)
				bail = bail_eof;
			break;
		}
	}

	if (bail == bail_eof && !converted)
		converted = -1;	/* Return EOF (-1) */

	return converted;
}
int sscanf(const char *str, const char *format, ...)
{
	va_list ap;
	int rv;

	va_start(ap, format);
	rv = vsscanf(str, format, ap);
	va_end(ap);

	return rv;
}

//*****************************************************************************
//
//! A simple UART based get string function, with some line processing.
//!
//! \param pcBuf points to a buffer for the incoming string from the UART.
//! \param ulLen is the length of the buffer for storage of the string,
//! including the trailing 0.
//!
//! This function will receive a string from the UART input and store the
//! characters in the buffer pointed to by \e pcBuf.  The characters will
//! continue to be stored until a termination character is received.  The
//! termination characters are CR, LF, or ESC.  A CRLF pair is treated as a
//! single termination character.  The termination characters are not stored in
//! the string.  The string will be terminated with a 0 and the function will
//! return.
//!
//! In both buffered and unbuffered modes, this function will block until
//! a termination character is received.  If non-blocking operation is required
//! in buffered mode, a call to UARTPeek() may be made to determine whether
//! a termination character already exists in the receive buffer prior to
//! calling UARTgets().
//!
//! Since the string will be null terminated, the user must ensure that the
//! buffer is sized to allow for the additional null character.
//!
//! \return Returns the count of characters that were stored, not including
//! the trailing 0.
//
//*****************************************************************************
/*int UARTgets(char *pcBuf, unsigned long ulLen)
{
//UART_BUFFERED
    unsigned long ulCount = 0;
    char cChar;

    //
    // Check the arguments.
    //
    ASSERT(pcBuf != 0);
    ASSERT(ulLen != 0);
    ASSERT(g_ulBase != 0);

    //
    // Adjust the length back by 1 to leave space for the trailing
    // null terminator.
    //
    ulLen--;

    //
    // Process characters until a newline is received.
    //
    while(1)
    {
        //
        // Read the next character from the receive buffer.
        //
        if(!RX_BUFFER_EMPTY)
        {
            cChar = g_pcUARTRxBuffer[g_ulUARTRxReadIndex];
            ADVANCE_RX_BUFFER_INDEX(g_ulUARTRxReadIndex);

            //
            // See if a newline or escape character was received.
            //
            if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
            {
                //
                // Stop processing the input and end the line.
                //
                break;
            }

            //
            // Process the received character as long as we are not at the end
            // of the buffer.  If the end of the buffer has been reached then
            // all additional characters are ignored until a newline is
            // received.
            //
            if(ulCount < ulLen)
            {
                //
                // Store the character in the caller supplied buffer.
                //
                pcBuf[ulCount] = cChar;

                //
                // Increment the count of characters received.
                //
                ulCount++;
            }
        }
    }

    //
    // Add a null termination to the string.
    //
    pcBuf[ulCount] = 0;

    //
    // Return the count of chars in the buffer, not counting the trailing 0.
    //
    return(ulCount);

}
*/
//*****************************************************************************
//
//! Read a single character from the UART, blocking if necessary.
//!
//! This function will receive a single character from the UART and store it at
//! the supplied address.
//!
//! In both buffered and unbuffered modes, this function will block until a
//! character is received.  If non-blocking operation is required in buffered
//! mode, a call to UARTRxAvail() may be made to determine whether any
//! characters are currently available for reading.
//!
//! \return Returns the character read.
//
//*****************************************************************************

/* unsigned char UARTgetc(void)
{

    unsigned char cChar;

    //
    // Wait for a character to be received.
    //
    while(RX_BUFFER_EMPTY)
    {
        //
        // Block waiting for a character to be received (if the buffer is
        // currently empty).
        //
    }

    //
    // Read a character from the buffer.
    //
    cChar = g_pcUARTRxBuffer[g_ulUARTRxReadIndex];
    ADVANCE_RX_BUFFER_INDEX(g_ulUARTRxReadIndex);

    //
    // Return the character to the caller.
    //
    return(cChar);

    //
    // Block until a character is received by the UART then return it to
    // the caller.
    //
    return(MAP_UARTCharGet(g_ulBase));

}
*/
/*
int _read(int file, char *ptr, int len) {
    int n;
    int num = 0;
    switch (file) {
    case STDIN_FILENO:
        for (n = 0; n < len; n++) {
            char c = Usart1GetChar();
            *ptr++ = c;
            num++;
        }
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return num;
}
*/
// Reduced version of scanf (%d, %x, %c, %n are supported)
// %d dec integer (E.g.: 12)
// %x hex integer (E.g.: 0xa0)
// %b bin integer (E.g.: b1010100010)
// %n hex, de or bin integer (e.g: 12, 0xa0, b1010100010)
// %c any character
//
int rsscanf(const char* str, const char* format, ...)
{
        va_list ap;
        int value, tmp;
        int count;
        int pos;
        char neg, fmt_code;
        const char* pf;
        va_start(ap, format);
        for (pf = format, count = 0; *format != 0 && *str != 0; format++, str++)
        {
                while (*format == ' ' && *format != 0)
                        format++;
                if (*format == 0)
                        break;
                while (*str == ' ' && *str != 0)
                        str++;
                if (*str == 0)
                        break;
                if (*format == '%')
                {
                        format++;
                        if (*format == 'n')
                        {
                if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X'))
                {
                    fmt_code = 'x';
                    str += 2;
                }
                else
                if (str[0] == 'b')
                {
                    fmt_code = 'b';
                    str++;
                }
                else
                    fmt_code = 'd';
                        }
                        else
                                fmt_code = *format;
                        switch (fmt_code)
                        {
                        case 'x':
                        case 'X':
                                for (value = 0, pos = 0; *str != 0; str++, pos++)
                                {
                                        if ('0' <= *str && *str <= '9')
                                                tmp = *str - '0';
                                        else
                                        if ('a' <= *str && *str <= 'f')
                                                tmp = *str - 'a' + 10;
                                        else
                                        if ('A' <= *str && *str <= 'F')
                                                tmp = *str - 'A' + 10;
                                        else
                                                break;
                                        value *= 16;
                                        value += tmp;
                                }
                                if (pos == 0)
                                        return count;
                                *(va_arg(ap, int*)) = value;
                                count++;
                                break;
            case 'b':
                                for (value = 0, pos = 0; *str != 0; str++, pos++)
                                {
                                        if (*str != '0' && *str != '1')
                        break;
                                        value *= 2;
                                        value += *str - '0';
                                }
                                if (pos == 0)
                                        return count;
                                *(va_arg(ap, int*)) = value;
                                count++;
                                break;
                        case 'd':
                                if (*str == '-')
                                {
                                        neg = 1;
                                        str++;
                                }
                                else
                                        neg = 0;
                                for (value = 0, pos = 0; *str != 0; str++, pos++)
                                {
                                        if ('0' <= *str && *str <= '9')
                                                value = value*10 + (int)(*str - '0');
                                        else
                                                break;
                                }
                                if (pos == 0)
                                        return count;
                                *(va_arg(ap, int*)) = neg ? -value : value;
                                count++;
                                break;
                        case 'c':
                                *(va_arg(ap, char*)) = *str;
                                count++;
                                break;
                        default:
                                return count;
                        }
                }
                else
                {
                        if (*format != *str)
                                break;
                }
        }
        va_end(ap);
        return count;
}

int hextoi(char ch)
{
  if(ch>='0' && ch<='9'){
      return ch - '0';
  } else if (ch>='a' && ch<='f'){
      return ch - 'a' + 10;
  } else {
      return 0;
  }
}
//Reads data from usart1  and stores them according to parameter format into the locations given by the additional arguments, as if scanf was used
// Reduced version of scanf (%d, %x, %c, %n are supported)
// %d dec integer (E.g.: 12)
// %x hex integer (E.g.: 0xa0)
// %b bin integer (E.g.: b1010100010)
// %n hex, de or bin integer (e.g: 12, 0xa0, b1010100010)
// %c any character
//buffer support 12 bytes
#define scanf_buff_size 12


int rscanf(const char* format, ...){
	va_list args;
	va_start( args, format );
	int count = 0;
	char ch = 0;
	char buffer[scanf_buff_size];

	SPRINTF_buffer = buffer;


	while(count <= scanf_buff_size )//get string
	{
		count++;
		ch = UsartXGetChar();
//#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
//		ch = Usart1GetChar();
//#elif (PROCESSOR == PROCESSOR_STM32F407VZT6)
//		ch = Usart3GetChar();
//#endif

		if(ch != '\n' && ch != '\r') *SPRINTF_buffer++ = ch;
		else
			break;
	}
	*SPRINTF_buffer = '\0';//end of string

	SPRINTF_buffer = buffer;
	count =  rsscanf(SPRINTF_buffer, format, args);
	va_end(args);
	return count;

}


/* --------------------------------- End Of File ------------------------------ */
