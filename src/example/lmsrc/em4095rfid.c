#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
//#include "driverlib/qei.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "ylib/yInc.h"

/* For 50MHz Clock, Tc= 20nsec. Max time = 1.3107msec.
 * 125Kbps RFID
 * RFID Data Rate =  1/64 of 125KHz = about 2Kbps (Demodulated data rate). Each bit takes 25000 clks for 50MHz Timer.
 * 16 bit input edge time mode : No prescaled. use free-running down counter (init val = 0xffff)
 * To change from rising to falling, or vice versa, you must call TimerDisable(), and reload 0xFFFF, and then TimerEnable()...
 */
//For RFID reader EM4095. Working with EM4100 series Tags
/* ULED0 : PA7
 * DEMOD : CCP1, PA6, Pin 4 Input (2Kbps Manchester)
 * MOD   : PD6, Pin 9 Output (May be 2Kbps Manchester) -- Not used.
 * SHD   : PD1, Pin 10 Output -- need for stopping unnecessary back-to-back reads.
 * RDY/CLK: CCP0, PD4, Pin 33 Input (125KHz from EM4095)
 * BUZZER : PC7 Pin 34 GPIO Output
 */

/* EM4100 Tag's Data Format (Non-standard) (64bits)

 1 1 1 1  1   1   1   1   1	- header (9 bit) -- for sync (will be encoded as Manchester)
	      x   x   x   x  P0 \       --- Ver or ID
	      x   x   x   x  P1  |      --- Ver or ID
	      x   x   x   x  P2  |      --- ID
	      x   x   x   x  P3  |      --- ID
	      x   x   x   x  P4   \ parity
	      x   x   x   x  P5   /
	      x   x   x   x  P6  |
	      x   x   x   x  P7  |
	      x   x   x   x  P8  |
	      x   x   x   x  P9 /
 parity	 PC0 PC1 PC2 PC3  0        :0=stop bit  -- Column Parity (Even)
 */

#define	RFID_MOD_ON		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); //MOD_port |= 1 << MOD_pin	// set high level on pin
#define RFID_MOD_OFF	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, ~GPIO_PIN_6);//MOD_port &= ~(1 << MOD_pin)	// sel low level on pin

//Manchester coded demodulated frame
/*     |   1   |   1   |   1   |   0   |   1   |
 *         +---+   +---+   +---+---+       +---+
 *  ***+---+   +---+   +---+       +---+---+
 *     ****|   1   |   1   |   1   |   0   |   1   |
 */
//For 50MHz MCU, a clock takes 20nsec.
#define MANCHESTER_BIT		25000	//2Kbps RFID (500usec) == 25000 clks. (1/64)
#define MANCHESTER_HALF_BIT	(MANCHESTER_BIT / 2)  //200 clks
#define MANCHESTER_MIDDLE	((MANCHESTER_BIT + MANCHESTER_HALF_BIT) / 2) // 300 clks
#define MANCHESTER_MAX		(MANCHESTER_MIDDLE + MANCHESTER_HALF_BIT)    // 300+200=500 clks
#define MANCHESTER_MIN		(MANCHESTER_MIDDLE - MANCHESTER_HALF_BIT)    // 300-100 =200 clks
#define IS_MANCHESTER		((sigHoldTime < MANCHESTER_MAX) &&( sigHoldTime > MANCHESTER_MIN)) // 200 < sigHoldTime < 500
#define IS_MANCHESTER_T		(sigHoldTime < MANCHESTER_MIDDLE)             // sigHoldTime < 300
#define IS_MANCHESTER_2T	(sigHoldTime > MANCHESTER_MIDDLE)             // sigHoldTime > 300
//#define _XTAL_FREQ 50000000	// oscillator frequency

char rfid_code[17];
char g_dataReady = 0;
char currentBit = 1;
char needNextManchesterT = 0;
unsigned char bitReceived = 7;
unsigned char byteReceived = 6;
unsigned char buffer[8];
unsigned char data[5];

#define SPKON           {GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);}
#define SPKOFF          {GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, ~GPIO_PIN_1);}

char Em4095checkdata(void);

void Em4095GpioConf(void)
{
	/* LED : PA7
	 * DEMOD : CCP1, PA6, Pin 4 Input
	 * MOD   : PD6, Pin 9 Output
	 * SHD   : PD1, Pin 10 Output
	 * RDY/CLK: CCP0, PD4, Pin 33 Input
	 * BUZZER : PC7 Pin 34 GPIO Output
	 * */
	// set MOD and SHDN pins as outputs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//SPK VER4
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);//LED PA7
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6); //DEMOD CCP1 PA6
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);//MOD PD6
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);//SHD PD1
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4); //RDY/CLK CCP0 PD4
	//GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7); //BUZZER PC7 -- VER5
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);//SPK PB1 (VER4)
	//Init
	UARTprintf("Init Done\r\n");
	somedelay(100);
}
/******************************************************************************************
 1. set port direction
 2. set timer to 125kHz or set external from CLK/RDY on EM4095
 3. set interrupt on capture (falling edge) with CCP on DEMOD pin
 CCP1(PA6) of LM3s8962 can support CCP with Edge Timing Mode, but only either rising or falling edge. Not Both.
 Thus, we switch edge mode at each interrupt.
*****************************************************************************************/
void Em4095Init(void) {



/*  TIP: You may use another way for decoding the DEMOD signal with GPIO Interrupt and a timer.
    // Set the Rx pin to generate an interrupt on the next falling edge.
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_BOTH_EDGES);
    GPIOPinIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
    IntPrioritySet(INT_GPIOA, 0x00);//priority 0 is the highest
    GPIOPinIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6);
    IntEnable(INT_GPIOA);
*/
	// initialization of capture module
	UARTprintf("Using 16-Bit Timer Interrupt for decoding the demodulated Manchester Code.\r\n");
    UARTprintf("   Timer = Timer0B\r\n");
    UARTprintf("   Mode =  Edge-Timer Mode(Interrupt on Next Rising/Falling Edge)\r\n");
    UARTprintf("   RFID Data Rate =  1/64 of 125KHz = about 2Kbps\r\n");
    UARTprintf("DeMOD signal is routed into CCP1(PA6 of LM3S8962)\r\n");
    UARTprintf("You may use the CLK of 125KHz from EM4905.\r\n");
    UARTprintf("In addition, you may use the MOD to write something to Tag.\r\n");
    UARTprintf("\r\nContact me your EM4102/4200/4205/4305 Tags.\r\n");

    // Configure the PA6 pin muxing for the Timer/CCP function.
    // Configi Timer 0B (CCP1)(DEMOD input pin).   //If you need CLK pin, add Timer 0A(CCP0).
    GPIOPinConfigure(GPIO_PA6_CCP1);
    GPIOPinTypeTimer(GPIO_PORTA_BASE, GPIO_PIN_6);
    // Configure the ccp settings for CCP pin.
    // Set up Timer0B for edge-timer mode, positive edge.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_16_BIT_PAIR | TIMER_CFG_B_CAP_TIME);
    TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE); //rising edge
    TimerLoadSet(TIMER0_BASE, TIMER_B, 0xffff); //load initial counter value of 0xffff(will be count down)
    // Configure the Timer0B interrupt
    //IntPrioritySet(INT_TIMER0B, 0x40);//priority 0 is the highest
    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_B);
    IntEnable(INT_TIMER0B);
    IntMasterEnable();

	Em4095off();				// turns RFID module off
}
/*****************************************************************************************
* Data reception (call on capture interrupt)
*****************************************************************************************/
/*
 * u32 prev_timer = 0xffff;
u16 isrcountUp = 0;
u16 isrcountDn = 0x7fff;

void Em4095GpioIsr(void){//called at each falling edge
	u8 edge;
//	u32 curr_timer=0;
//	u32 sigHoldTime=0;
//	IntMasterDisable(); //Mask all interrupts
	GPIOPinIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
	edge = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6);

	if(edge){ //on rising edge
//		TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE); //switch to falling edge
//	    TimerLoadSet(TIMER0_BASE, TIMER_B, 0xffff); //load initial counter value of 0xffff(will be count down)
//		TimerEnable(TIMER0_BASE, TIMER_B);
//		sigHoldTime = 0xffff - curr_timer;// +2;
		//UARTprintf("|-"); //
	}//else
		//UARTprintf("|_"); //
	//UARTprintf("GPIOint\r\n");
//	curr_timer = TimerValueGet(TIMER0_BASE, TIMER_B);
//	if(prev_timer >= curr_timer)
//		sigHoldTime = prev_timer - curr_timer;
//	else
//		sigHoldTime = (prev_timer + 0x10000) - curr_timer;
//	prev_timer = curr_timer;

}
*/
//[NOTE] CCP1(PA6) of LM3s8962 can support CCP with Edge Timing Mode, but only either rising or falling edge. Not Both.
// Thus, we switch edge mode at each interrupt.
volatile u8 edge;
volatile u16 sigHoldTime=0;
void Em4095Isr(void) { //called at each rising or falling edge
	u32 curr_timer;

	IntMasterDisable(); //Mask all interrupts
	TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);    //Timer0B Interrupt Clear.
	curr_timer = TimerValueGet(TIMER0_BASE, TIMER_B);//HWREG(TIMER0_BASE + TIMER_O_TBR);//
	TimerDisable(TIMER0_BASE, TIMER_B);
	//sigHoldTime = (saved > curr_timer) ? saved-curr_timer : curr_timer - saved + 0xffff;
	//saved = curr_timer;
	edge = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) >> 6; //get whether it is rising or falling edge.

	if(edge){ //on rising edge
		TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE); //switch to falling edge
	    TimerLoadSet(TIMER0_BASE, TIMER_B, 0xffff);                    //load initial counter value of 0xffff(will be count down)
		TimerEnable(TIMER0_BASE, TIMER_B);
		sigHoldTime = 0xffff - curr_timer;// +2;
		//UARTprintf("|-%d (%d)",curr_timer,sigHoldTime);
	}
	else //falling edge
	{
		TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE); //switch to rising edge
	    TimerLoadSet(TIMER0_BASE, TIMER_B, 0xffff);                    //load initial counter value of 0xffff(will be count down)
		TimerEnable(TIMER0_BASE, TIMER_B);
		sigHoldTime = 0xffff - curr_timer;// +2;
		//UARTprintf("|_%d (%d)",curr_timer,sigHoldTime);
	}
	//edge = ~edge;

	if (IS_MANCHESTER) {				    // test if it can be manchester symbol
		if (needNextManchesterT) {		    // if previous reception was half-bit
			if (IS_MANCHESTER_T) {	        // if it's second half-bit
				needNextManchesterT = 0;	// we don't expecting next half-bit
				Em4095buffer_add_bit();	    // add received bit to buffer
			}
			else {			                // if we expected half bit and it wasn't
				Em4095BufReset();	        // reset buffer
			}
		}
		else {								// if half-bit wasn't expected
			if (IS_MANCHESTER_T) {			// expect second half-bit on next capture
				needNextManchesterT = 1;
			}
			else {							// if it was full byte
				currentBit ^= 0x01;			// toggle bit that will be added
				Em4095buffer_add_bit();		// add received bit to buffer
			}
		}
		//UARTprintf("%d",currentBit);
	}
	else {									// if not manchester symbol
		//UARTprintf("Not ");
		Em4095BufReset();					// reset buffer
	}

    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);
    IntEnable(INT_TIMER0B);
    IntMasterEnable();

	return;
}

/*****************************************************************************************
* Add bit to buffer on correct reception
* Each bit will be stored in the buffer[8]. The first bit will be stored in the last bit of the last byte of the buffer.
* That is, buffer[0]                buffer[7]
*                   <--------------
*****************************************************************************************/
void Em4095buffer_add_bit(void) {
	//UARTprintf("%d ",currentBit);
	if (buffer[7] == 0xff) {			// if we received 8 bits of header (it is synchronized)
		buffer[byteReceived] |= currentBit << bitReceived;	// save received bit on current position
		if (bitReceived-- == 0) {		// change bit position
			bitReceived = 7;			// if we are at the and of byte jump on start
			if (byteReceived-- == 0) {	// and change byte position
				Em4095checkdata();		// if reception end, check received buffer and save data
				Em4095BufReset();		// prepare buffer and counters for next transfer
				//UARTprintf("Got a full frame\r\n");
				return;
			}
		}
	}
	else {								// if we are not synchronized, thus we will get 111111111(sync 9 bits)
		buffer[7] <<= 1;				// shift first byte
		buffer[7] |= currentBit;		// and add received bit
	}
}
/*****************************************************************************************
* Return even parity of data
*****************************************************************************************/
char parity(unsigned char dataIn) {
	char n;
	char par = 0;

	for (n = 8; n; n--) {
		par ^= dataIn & 0x01;	// makes exclussive-or with every bit
		dataIn >>= 1;
	}
	return par;
}
/*****************************************************************************************
* Reset buffer, positions and capture edge
*****************************************************************************************/
void Em4095BufReset(void) {
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;
	buffer[3] = 0;
	buffer[4] = 0;
	buffer[5] = 0;
	buffer[6] = 0;
	buffer[7] = 0;				// reset buffer

	currentBit = 1;				// next bit that will bi added is 1
	needNextManchesterT = 0;	// don't expect half-bit

	bitReceived = 7;			// reset bit position in buffer
	byteReceived = 6;			// reset byte position in buffer

}
/*****************************************************************************************
* Set RFID module to sleep mode
*****************************************************************************************/
void Em4095off(void) {
	g_dataReady = 0;				// if it's in sleep mode, data cant't be ready
	RFID_MOD_OFF;					// it's recomended to turn modulation off in sleep mode
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); //SHDN_port |= 1 << SHDN_pin;	// high level forces the circuit into sleep mode
}
/*****************************************************************************************
* Wake-up from sleep mode
*****************************************************************************************/
void Em4095on(void) {
	Em4095BufReset();				// reset buffer etc.

	RFID_MOD_OFF;	//if you need ON, change...				// turn modulation off
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, ~GPIO_PIN_1); //SHDN_port &= ~(1 << SHDN_pin);	// wake-up
	hw1msecDelay(35); //WAIT_35_MS;						// wait for antenna oscillation
}
/*****************************************************************************************
* Check received data in buffer
******************************************************************************************
 Returns 1 if it's correct format
 Returns 0 if there is something wrong

 Data format:
 1 1 1 1  1   1   1   1   1	- header (9 bits of 1) : buffer[7]
	      x   x   x   x  P0 \
	      x   x   x   x  P1  |
	      x   x   x   x  P2  |
	      x   x   x   x  P3  |
	      x   x   x   x  P4   \ parity.    (Data = 20bits, 10 nibbles)
	      x   x   x   x  P5   /
	      x   x   x   x  P6  |
	      x   x   x   x  P7  |
	      x   x   x   x  P8  |
	      x   x   x   x  P9 /
 parity	 PC0 PC1 PC2 PC3  0 - stop bit             : buffer[0]
*****************************************************************************************/

char Em4095checkdata(void) {
signed char n;

	if ((buffer[7] != 0xff) || ((buffer[6] & 0x80) != 0x80)) {	// check header bits(9 bits of '1')
		UARTprintf(" BAD HEADER\r\n");
		return 0;
	}
	//else UARTprintf(" GOOD HEADER\r\n");

	if ((buffer[0] & 0x01) != 0) {		// check stop bit
		//UARTprintf(" No good stop bit\r\n");
		return 0;
	}

	if (parity(buffer[6] & 0b01111000) != ((buffer[6] & 0x04) >> 2))	// check P0
		return 0;

	if (( parity(buffer[6] & 0b00000011) ^ parity(buffer[5] & 0b11000000)) != ((buffer[5] & 0x20) >> 5))	// check P1
		return 0;

	if (parity(buffer[5] & 0b00011110) != (buffer[5] & 0x01))	// check P2
		return 0;

	if (parity(buffer[4] & 0b11110000) != ((buffer[4] & 0x08) >> 3))	// check P3
		return 0;

	if ((parity(buffer[4] & 0b00000111) ^ parity(buffer[3] & 0b10000000)) != ((buffer[3] & 0x40) >> 6))	// check P4
		return 0;

	if (parity(buffer[3] & 0b00111100) != ((buffer[3] & 0x02) >> 1))	// check P5
		return 0;

	if ((parity(buffer[3] & 0b00000001) ^ parity(buffer[2] & 0b11100000)) != ((buffer[2] & 0x10) >> 4))	// check P6
		return 0;

	if (parity(buffer[2] & 0b00001111) != ((buffer[1] & 0x80) >> 7))	// check P7
		return 0;

	if (parity(buffer[1] & 0b01111000) != ((buffer[1] & 0x04) >> 2))	// check P8
		return 0;

	if ((parity(buffer[1] & 0b00000011) ^ parity(buffer[0] & 0b11000000)) != ((buffer[0] & 0x20) >> 5))	// check P9
		return 0;

	if (parity(buffer[6] & 0b01000010) ^
		parity(buffer[5] & 0b00010000) ^
		parity(buffer[4] & 0b10000100) ^
		parity(buffer[3] & 0b00100001) ^
		parity(buffer[2] & 0b00001000) ^
		parity(buffer[1] & 0b01000010) != ((buffer[0] & 0x10) >> 4))	// check PC0
		return 0;

	if (parity(buffer[6] & 0b00100001) ^
		parity(buffer[5] & 0b00001000) ^
		parity(buffer[4] & 0b01000010) ^
		parity(buffer[3] & 0b00010000) ^
		parity(buffer[2] & 0b10000100) ^
		parity(buffer[1] & 0b00100001) != ((buffer[0] & 0x08) >> 3))	// check PC1
		return 0;

	if ((parity(buffer[6] & 0b00010000) ^
		parity(buffer[5] & 0b10000100) ^
		parity(buffer[4] & 0b00100001) ^
		parity(buffer[3] & 0b00001000) ^
		parity(buffer[2] & 0b01000010) ^
		parity(buffer[1] & 0b00010000) ^
		parity(buffer[0] & 0b10000000)) != ((buffer[0] & 0x04) >> 2))	// check PC2
		return 0;

	if ((parity(buffer[6] & 0b00001000) ^
		parity(buffer[5] & 0b01000010) ^
		parity(buffer[4] & 0b00010000) ^
		parity(buffer[3] & 0b10000100) ^
		parity(buffer[2] & 0b00100001) ^
		parity(buffer[1] & 0b00001000) ^
		parity(buffer[0] & 0b01000000)) != ((buffer[0] & 0x02) >> 1))	// check PC3
		return 0;

	data[4] = ((buffer[6] & 0b01111000) << 1) |
			  ((buffer[6] & 0b00000011) << 2) |
			  ((buffer[5] & 0b11000000) >> 6);

	data[3] = ((buffer[5] & 0b00011110) << 3) |
			  ((buffer[4] & 0b11110000) >> 4);

	data[2] = ((buffer[4] & 0b00000111) << 5) |
			  ((buffer[3] & 0b10000000) >> 3) |
			  ((buffer[3] & 0b00111100) >> 2);

	data[1] = ((buffer[3] & 0b00000001) << 7) |
			  ((buffer[2] & 0b11100000) >> 1) |
			  (buffer[2] & 0b00001111);

	data[0] = ((buffer[1] & 0b01111000) << 1) |
			  ((buffer[1] & 0b00000011) << 2) |
			  ((buffer[0] & 0b11000000) >> 6);		// save data from buffer


	UARTprintf("RX>[buffer[7..0] 8bytes : HEX] ");
	for (n = 7; n >= 0; n--)
		UARTprintf("%02x ", buffer[n]);
	UARTprintf("\r\n");

	g_dataReady = 1;		// data is ready for reading
	//UARTprintf("g_ready\r\n");
	return 1;
}

/*****************************************************************************************
* Save code to array -- TBD
 Final string is compatible with Innovations ID-12 module
 Format:
 [0x12] [Code - HEX format] [Checksum] [0x0d] [0x0a] [0x03] [0x00]
   1B           10B             2B       1B     1B     1B     1B
*****************************************************************************************/
char Em4095read(char *code) {
	signed char n;
	char checksum = data[4] ^ data[3] ^ data[2] ^ data[1] ^ data[0];	// compute ID12's checksum
	if (g_dataReady) {
		*(code++) = 0x02;						// start of text

		for (n = 4; n >= 0; n--) {
			*(code++) = toAscii(data[n] >> 4);
			*(code++) = toAscii(data[n]);		// convert code to text
		}

		*(code++) = toAscii(checksum >> 4);
		*(code++) = toAscii(checksum);			// checksum

		*(code++) = 0x0d;						// carriage reurn
		*(code++) = 0x0a;						// new line
		*(code++) = 0x03;						// end of text
		*(code++) = 0x00;						// end of array

		g_dataReady = 0;						// data was read

		return 1;
	}
	return 0;
}


/*****************************************************************************************
* Main function
******************************************************************************************
 1. initialize monules
 2. reads code from RFID module in infinite loop
 3. after reading, send to uart
*****************************************************************************************/
void Em4095Loop(void) {
	signed char n;

	GuiInit("EM4095 125KHz RFID Reader Test");
	Em4095GpioConf();
	Em4095Init();

	//ei();						// enable interrupts
	Em4095on();						// turn on RFID module
	SPKON;
	hw100msecDelay(1);
	SPKOFF;
	IntMasterEnable();

	while(1) {
		if (Em4095read(rfid_code)) {	// read data
			Em4095off();// turn RFID module off (to avoid interrupting and getting program slow)

			dtUserLedCon(1);			// set LED to red
			dtUserLedCon(1);	 // set LED to green
			SPKON;

			UARTprintf("RX>[Tag Data[4..0] 5bytes : HEX] ");
			for (n = 4; n >= 0; n--)
				UARTprintf("%02x ", data[n]);
			UARTprintf("\r\n");
			UARTprintf("RX>[ID-12 Format: HEX] ");
			for(n=0;n<17;n++)
				UARTprintf("%02x ", rfid_code[n]);
			UARTprintf("\r\n");
			UARTprintf("RX>[RFID Data:10 nibbles : CHAR] ");
			for(n=1;n<11;n++)
							UARTprintf("%c ", rfid_code[n]);
			UARTprintf("\r\n");
			hw100msecDelay(1); // wait for 100 msec
			SPKOFF;
			hw1secDelay(2); // wait for 2sec
			dtUserLedCon(0);
		}
		Em4095on();			// turn on RFID again

	}

}


