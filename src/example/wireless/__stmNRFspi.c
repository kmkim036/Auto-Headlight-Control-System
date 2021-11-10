//[YOON]- VERY IMPORTANT IN SPI Transaction
// At the first byte of mosi data from MCU will make the stateReg value of the NRF on miso simultaneously.
// [MOSI] ---| WRITE/READ(REG,or NULL) | VALUE/orNULL    |--------
// [MISO] ---| STATUSREG               | APPROPRIATE BYTE|--------
/**
 * Driver for nRF24L01(+) 2.4GHz Wireless Transceiver
 *
 * ?This chip uses the SPI bus, plus two chip control pins.  Remember that pin 10 must still remain an output, or
 * the SPI hardware will go into 'slave' mode.

 */
/* Refer from Coliz <maniacbug@ymail.com>
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "yInc.h"
#if((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT6))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#endif

#include "ySpiDrv.h"

//103(M35)
//+--------+-----------+-----------+-----------+---------+---------+-------
//|        |401-M34    |401-M35    | 407-M35   | 407-M36 |103(M35) |103LEAN
//+--------+-----------+-----------+-----------+---------+---------+--------
//| ULED   | PB14      |PC4        |PE15       | <==     | PC14    |PC13
//+--------+-----------+-----------+-----------+---------+---------+--------
//| BUTTON |           |PC5(H)     |PD11(index)| PD11(L) | PC15    |PA15
//+--------+-----------+-----------+-----------+---------+---------+---------

//[PINOUT]
//     PB5  PB15 PB0
// 3V3 nCS  MOS  IRQ (IRQ -- NOT USED)
//  2  4    6    8
//+ 1  3    5    7
// GND CE   SCK  MISO
//     PB10 PB13 PB14

//use nCS0 of PB12 : 103 KONG
//use nCS1 of PB5  : 103 LEAN
#define nCS_NRF_H nCS1_H //nCS0_H
#define nCS_NRF_L nCS1_L //nCS0_L

#if (PROCESSOR == PROCESSOR_STM32F401RET6)
extern void stmSpi1_Config(unsigned char nCS);
extern unsigned char stmSpi1RdByte();
extern void stmSpi1WrByte(unsigned char inbyte);
#else
void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern unsigned char stmSpi2RdByte();
extern unsigned short stmSpi2WrByte(unsigned char inbyte);
#endif

#define _BV(x) (1<<(x))

typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;// Power Amplifier level.
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

struct RF24
{

  unsigned char ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
  unsigned char csn_pin; /**< SPI Chip select */
  unsigned char wide_band; /* 2Mbs data rate in use? */
  unsigned char p_variant; /* False for RF24L01 and true for RF24L01P */
  unsigned char payload_size; /**< Fixed size of payloads */
  unsigned char ack_payload_available; /**< Whether there is an ack payload waiting */
  unsigned char dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
  unsigned char ack_payload_length; /**< Dynamic size of pending ack payload. */
  unsigned long long pipe0_reading_address; /**< Last address set on pipe 0 for reading. *///uint64_t pipe0_reading_address; /**< Last address set on pipe 0 for reading. */
};

struct RF24 g_radio24;

typedef enum { role_master = 0, role_led_slave } role_e;
const char* role_friendly_name[] = { "invalid", "RemoteMaster", "LedSlave"};

// sets the role of this unit in hardware.  Connect to GND to be the 'led' board receiver
// Leave open to be the 'remote' transmitter
//const int role_pin = 0;//A4; //YOON

// Pins on the remote for buttons
//const unsigned char button_pins[] = { 2,3,4,5,6,7 };
//const unsigned char num_button_pins = sizeof(button_pins);

// Pins on the LED board for LED's
//const unsigned char led_pins[] = { 2,3,4,5,6,7 };
//const unsigned char num_led_pins = sizeof(led_pins);

// Single radio pipe address for the 2 nodes to communicate.
//const uint64_t pipe = 0xE8E8F0F0E1LL;
const unsigned long long pipe = 0xE7E7E7E7E7;
role_e role;// The role of the current running
unsigned char button_states[1];//num_button_pins];
unsigned char led_states[1];//num_led_pins];

extern void stmButton_setup(void); //PC15/PA15
extern void stmUser_LED_GPIO_setup(void);//PC14/PC13

void RF24_nCS(int mode);//SPI chip select (Active LOW)

// chip enable
//level = HIGH : actively begin transmission
//      = LOW  : put in standby
void RF24_ce(int level); //Active HIGH

  /**
   * Read a chunk of data in from a register
   *
   * @param reg Which register. Use constants from nRF24L01.h
   * @param buf Where to put the data
   * @param len How many bytes of data to transfer
   * @return Current value of status register
   */
  unsigned char RF24_read_register_withLen(unsigned char reg, unsigned char* buf, unsigned char len);

  /**
   * Read single byte from a register
   *
   * @param reg Which register. Use constants from nRF24L01.h
   * @return Current value of register @p reg
   */
  unsigned char RF24_read_register(unsigned char reg);

  /**
   * Write a chunk of data to a register
   *
   * @param reg Which register. Use constants from nRF24L01.h
   * @param buf Where to get the data
   * @param len How many bytes of data to transfer
   * @return Current value of status register
   */
  unsigned char RF24_write_register_withLen(unsigned char reg, unsigned char* buf, unsigned char len);

  /**
   * Write a single byte to a register
   *
   * @param reg Which register. Use constants from nRF24L01.h
   * @param value The new value to write
   * @return Current value of status register
   */
  unsigned char RF24_write_register(unsigned char reg, unsigned char value);

  /**
   * Write the transmit payload
   *
   * The size of data written is the fixed payload size, see getPayloadSize()
   *
   * @param buf Where to get the data
   * @param len Number of bytes to be sent
   * @return Current value of status register
   */
  unsigned char RF24_write_payload(unsigned char* buf, unsigned char len);

  /**
   * Read the receive payload
   *
   * The size of data read is the fixed payload size, see getPayloadSize()
   *
   * @param buf Where to put the data
   * @param len Maximum number of bytes to read
   * @return Current value of status register
   */
  unsigned char RF24_read_payload(void* buf, unsigned char len);

  /**
   * Empty the receive buffer
   *
   * @return Current value of status register
   */
  unsigned char RF24_flush_rx(void);

  /**
   * Empty the transmit buffer
   *
   * @return Current value of status register
   */
  unsigned char RF24_flush_tx(void);

  /**
   * Retrieve the current status of the chip
   *
   * @return Current value of status register
   */
  unsigned char RF24_get_status(void);

  /**
   * Decode and print the given status to stdout
   *
   * @param status Status value to print
   *
   * @warning Does nothing if stdout is not defined.  See fdevopen in stdio.h
   */
  void RF24_print_status(unsigned char status);

  /**
   * Decode and print the given 'observe_tx' value to stdout
   *
   * @param value The observe_tx value to print
   *
   * @warning Does nothing if stdout is not defined.  See fdevopen in stdio.h
   */
  void RF24_print_observe_tx(unsigned char value);

  /**
   * Print the name and value of an 8-bit register to stdout
   *
   * Optionally it can print some quantity of successive
   * registers on the same line.  This is useful for printing a group
   * of related registers on one line.
   *
   * @param name Name of the register
   * @param reg Which register. Use constants from nRF24L01.h
   * @param qty How many successive registers to print
   */
  void RF24_print_byte_register(const char* name, unsigned char reg, unsigned char qty);//unsigned char qty = 1);

  /**
   * Print the name and value of a 40-bit address register to stdout (5 bytes)
   *
   * Optionally it can print some quantity of successive
   * registers on the same line.  This is useful for printing a group
   * of related registers on one line.
   *
   * @param name Name of the register
   * @param reg Which register. Use constants from nRF24L01.h
   * @param qty How many successive registers to print
   */
  void RF24_print_address_register(const char* name, unsigned char reg,unsigned char qty );// unsigned char qty = 1);

  /**
   * Turn on or off the special features of the chip
   *
   * The chip has certain 'features' which are only available when the 'features'
   * are enabled.  See the datasheet for details.
   */
  void RF24_toggle_features(void);
  /**@}*/

  /**
   * Constructor
   *
   * Creates a new instance of this driver.  Before using, you create an instance
   * and send in the unique pins that this chip is connected to.
   *
   * @param _cepin The pin attached to Chip Enable on the RF module
   * @param _cspin The pin attached to Chip Select
   */
  RF24_Config(unsigned char _cepin, unsigned char _cspin);

  /**
   * Begin operation of the chip
   *
   * Call this in setup(), before calling any other methods.
   */
  void RF24_begin(void);

  /**
   * Start listening on the pipes opened for reading.
   *
   * Be sure to call openReadingPipe() first.  Do not call write() while
   * in this mode, without first calling stopListening().  Call
   * isAvailable() to check for incoming traffic, and read() to get it.
   */
  void RF24_startListening(void);

  /**
   * Stop listening for incoming messages
   *
   * Do this before calling write().
   */
  void RF24_stopListening(void);

  /**
   * Write to the open writing pipe
   * Be sure to call openWritingPipe() first to set the destination of where to write to.
   * This blocks until the message is successfully acknowledged by the receiver or the timeout/retransmit maxima are reached.
   * In the current configuration, the max delay here is 60ms.
   * The maximum size of data written is the fixed payload size, see getPayloadSize().
   * However, you can write less, and the remainder will just be filled with zeroes.
   *
   * @param buf Pointer to the data to be sent
   * @param len Number of bytes to be sent
   * @return True if the payload was delivered successfully false if not
   */
  unsigned char NRF24_WriteMsg(unsigned char* buf, unsigned char len );

  /**Test whether there are bytes available to be read
   * @return True if there is a payload available, false if none is
   */
  unsigned char RF24_rcvd_available(void);

  /**
   * Read the payload
   *
   * Return the last payload received
   *
   * The size of data read is the fixed payload size, see getPayloadSize()
   *
   * @note I specifically chose 'void*' as a data type to make it easier
   * for beginners to use.  No casting needed.
   *
   * @param buf Pointer to a buffer where the data should be written
   * @param len Maximum number of bytes to read into the buffer
   * @return True if the payload was delivered successfully false if not
   */
  unsigned char RF24_read( void* buf, unsigned char len );

  /**
   * Open a pipe for writing
   *
   * Only one pipe can be open at once, but you can change the pipe
   * you'll listen to.  Do not call this while actively listening.
   * Remember to stopListening() first.
   *
   * Addresses are 40-bit hex values, e.g.:
   *
   * @code
   *   openWritingPipe(0xF0F0F0F0F0);
   * @endcode
   *
   * @param address The 40-bit address of the pipe to open.  This can be
   * any value whatsoever, as long as you are the only one writing to it
   * and only one other radio is listening to it.  Coordinate these pipe
   * addresses amongst nodes on the network.
   */
  //void RF24_openWritingPipe(uint64_t address);
  void RF24_openWritingPipe(unsigned long long address);
  /**
   * Open a pipe for reading
   *
   * Up to 6 pipes can be open for reading at once.  Open all the
   * reading pipes, and then call startListening().
   *
   * @see openWritingPipe
   *
   * @warning Pipes 1-5 should share the first 32 bits.
   * Only the least significant byte should be unique, e.g.
   * @code
   *   openReadingPipe(1,0xF0F0F0F0AA);
   *   openReadingPipe(2,0xF0F0F0F066);
   * @endcode
   *
   * @warning Pipe 0 is also used by the writing pipe.  So if you open
   * pipe 0 for reading, and then startListening(), it will overwrite the
   * writing pipe.  Ergo, do an openWritingPipe() again before write().
   *
   * @todo Enforce the restriction that pipes 1-5 must share the top 32 bits
   *
   * @param number Which pipe# to open, 0-5.
   * @param address The 40-bit address of the pipe to open.
   */
  void RF24_openReadingPipe(unsigned char number, unsigned long long address);

  /**@}*/
  /**
   * @name Optional Configurators
   *
   *  Methods you can use to get or set the configuration of the chip.
   *  None are required.  Calling begin() sets up a reasonable set of
   *  defaults.
   */
  /**@{*/
  /**
   * Set the number and delay of retries upon failed submit
   *
   * @param delay How long to wait between each retry, in multiples of 250us,
   * max is 15.  0 means 250us, 15 means 4000us.
   * @param count How many retries before giving up, max 15
   */
  void sRF24_etRetries(unsigned char delay, unsigned char count);

  /**
   * Set RF communication channel
   *
   * @param channel Which RF channel to communicate on, 0-127
   */
  void RF24_setChannel(unsigned char channel);

  /**
   * Set Static Payload Size
   *
   * This implementation uses a pre-stablished fixed payload size for all
   * transmissions.  If this method is never called, the driver will always
   * transmit the maximum payload size (32 bytes), no matter how much
   * was sent to write().
   *
   * @todo Implement variable-sized payloads feature
   *
   * @param size The number of bytes in the payload
   */
  void RF24_setPayloadSize(unsigned char size);

  /**
   * Get Static Payload Size
   *
   * @see setPayloadSize()
   *
   * @return The number of bytes in the payload
   */
  unsigned char RF24_getPayloadSize(void);

  /**
   * Get Dynamic Payload Size
   *
   * For dynamic payloads, this pulls the size of the payload off
   * the chip
   *
   * @return Payload length of last-received dynamic payload
   */
  unsigned char RF24_getDynamicPayloadSize(void);

  /**
   * Enable custom payloads on the acknowledge packets
   *
   * Ack payloads are a handy way to return data back to senders without
   * manually changing the radio modes on both units.
   *
   * @see examples/pingpair_pl/pingpair_pl.pde
   */
  void RF24_enableAckPayload(void);

  /**
   * Enable dynamically-sized payloads
   *
   * This way you don't always have to send large packets just to send them
   * once in a while.  This enables dynamic payloads on ALL pipes.
   *
   * @see examples/pingpair_pl/pingpair_dyn.pde
   */
  void RF24_enableDynamicPayloads(void);

  /**
   * Determine whether the hardware is an nRF24L01+ or not.
   *
   * @return true if the hardware is nRF24L01+ (or compatible) and false
   * if its not.
   */
  unsigned char RF24_isPVariant(void) ;

  /**
   * Enable or disable auto-acknowlede packets
   *
   * This is enabled by default, so it's only needed if you want to turn
   * it off for some reason.
   *
   * @param enable Whether to enable (true) or disable (false) auto-acks
   */
  void RF24_setAutoAck(unsigned char enable);

  /**
   * Enable or disable auto-acknowlede packets on a per pipeline basis.
   *
   * AA is enabled by default, so it's only needed if you want to turn
   * it off/on for some reason on a per pipeline basis.
   *
   * @param pipe Which pipeline to modify
   * @param enable Whether to enable (true) or disable (false) auto-acks
   */
  void RF24_setAutoAck_withPipe( unsigned char pipe, unsigned char enable ) ;

  /**
   * Set Power Amplifier (PA) level to one of four levels.
   * Relative mnemonics have been used to allow for future PA level
   * changes. According to 6.5 of the nRF24L01+ specification sheet,
   * they translate to: RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm,
   * RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm.
   *
   * @param level Desired PA level.
   */
  void RF24_setPALevel( rf24_pa_dbm_e level ) ;

  /**
   * Fetches the current PA level.
   *
   * @return Returns a value from the rf24_pa_dbm_e enum describing
   * the current PA setting. Please remember, all values represented
   * by the enum mnemonics are negative dBm. See setPALevel for
   * return value descriptions.
   */
  rf24_pa_dbm_e RF24_getPALevel( void ) ;

  /**
   * Set the transmission data rate
   *
   * @warning setting RF24_250KBPS will fail for non-plus units
   *
   * @param speed RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
   * @return true if the change was successful
   */
  unsigned char RF24_setDataRate(rf24_datarate_e speed);

  /**
   * Fetches the transmission data rate
   *
   * @return Returns the hardware's currently configured datarate. The value
   * is one of 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS, as defined in the
   * rf24_datarate_e enum.
   */
  rf24_datarate_e RF24_getDataRate( void ) ;

  /**
   * Set the CRC length
   *
   * @param length RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
   */
  void RF24_setCRCLength(rf24_crclength_e length);

  /**
   * Get the CRC length
   *
   * @return RF24_DISABLED if disabled or RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
   */
  rf24_crclength_e RF24_getCRCLength(void);

  /**
   * Disable CRC validation
   *
   */
  void RF24_disableCRC( void ) ;

  /**@}*/
  /**
   * @name Advanced Operation
   *
   *  Methods you can use to drive the chip in more advanced ways
   */
  /**@{*/

  /**
   * Print a giant block of debugging information to stdout
   *
   * @warning Does nothing if stdout is not defined.  See fdevopen in stdio.h
   */
  void RF24_printDetails(void);

  /**
   * Enter low-power mode
   *
   * To return to normal power mode, either write() some data or
   * startListening, or powerUp().
   */
  void RF24_powerDown(void);

  /**
   * Leave low-power mode - making radio more responsive
   *
   * To return to low power mode, call powerDown().
   */
  void RF24_powerUp(void) ;

  /**
   * Test whether there are bytes available to be read
   *
   * Use this version to discover on which pipe the message
   * arrived.
   *
   * @param[out] pipe_num Which pipe has the payload available
   * @return True if there is a payload available, false if none is
   */
  unsigned char RF24_rcvd_available_withPipe(unsigned char* pipe_num);

  /**
   * Non-blocking write to the open writing pipe
   *
   * Just like write(), but it returns immediately. To find out what happened
   * to the send, catch the IRQ and then call whatHappened().
   *
   * @see write()
   * @see whatHappened()
   *
   * @param buf Pointer to the data to be sent
   * @param len Number of bytes to be sent
   * @return True if the payload was delivered successfully false if not
   */
  void RF24_startWrite(unsigned char* buf, unsigned char len );

  /**
   * Write an ack payload for the specified pipe
   *
   * The next time a message is received on @p pipe, the data in @p buf will
   * be sent back in the acknowledgement.
   *
   * @warning According to the data sheet, only three of these can be pending
   * at any time.  I have not tested this.
   *
   * @param pipe Which pipe# (typically 1-5) will get this response.
   * @param buf Pointer to data that is sent
   * @param len Length of the data to send, up to 32 bytes max.  Not affected
   * by the static payload set by setPayloadSize().
   */
  void RF24_writeAckPayload(unsigned char pipe, void* buf, unsigned char len);

  /**
   * Determine if an ack payload was received in the most recent call to
   * write().
   *
   * Call read() to retrieve the ack payload.
   *
   * @warning Calling this function clears the internal flag which indicates
   * a payload is available.  If it returns true, you must read the packet
   * out as the very next interaction with the radio, or the results are
   * undefined.
   *
   * @return True if an ack payload is available.
   */
  unsigned char RF24_isAckPayloadAvailable(void);

  /**
   * Call this when you get an interrupt to find out why
   *
   * Tells you what caused the interrupt, and clears the state of
   * interrupts.
   *
   * @param[out] tx_ok The send was successful (TX_DS)
   * @param[out] tx_fail The send failed, too many retries (MAX_RT)
   * @param[out] rx_ready There is a message waiting to be read (RX_DS)
   */
  void RF24_whatHappened(unsigned char *tx_ok,unsigned char *tx_fail,unsigned char *rx_ready);//void RF24_whatHappened(unsigned char& tx_ok,unsigned char& tx_fail,unsigned char& rx_ready);

  /**
   * Test whether there was a carrier on the line for the
   * previous listening period.
   *
   * Useful to check for interference on the current channel.
   *
   * @return true if was carrier, false if not
   */
  unsigned char RF24_testCarrier(void);

  /**
   * Test whether a signal (carrier or otherwise) greater than
   * or equal to -64dBm is present on the channel. Valid only
   * on nRF24L01P (+) hardware. On nRF24L01, use testCarrier().
   *
   * Useful to check for interference on the current channel and
   * channel hopping strategies.
   *
   * @return true if signal => -64dBm, false if not
   */
  unsigned char RF24_testRPD(void) ;


// Register Map
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
  //Status register
#define RX_DR       6 //rx data ready in FIFO. rx interrupt.
#define TX_DS       5 //tx data sent. tx interrupt
#define MAX_RT      4 //max number of retransmit. Must clear to next transmit (write 1 to clear)
#define RX_P_NO     1 //data pipe number to be received. (3bit : [3:1] = ... 111(empty rx FIFO)
#define TX_FULL     0 //tx fifo full

#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define RPD         0x09

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

/****************************************************************************/
//Chip Enable : PB10
void RF24_ce(int level) //PB10-STM32F103
{
	if(level)
		GPIO_SetBits(GPIOB, GPIO_Pin_10);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_10);
}

/****************************************************************************/
unsigned char RF24_read_register(unsigned char reg)
{
	unsigned char result;
	nCS_NRF_L;
	stmSpi2WrByte( R_REGISTER | ( REGISTER_MASK & reg ) );
	result = stmSpi2RdByte();
	nCS_NRF_H;
	return result;
}

unsigned char RF24_read_register_withLen(unsigned char reg, unsigned char* buf, unsigned char len)
{
  unsigned char status;

  nCS_NRF_L;
  status = stmSpi2WrByte(R_REGISTER | ( REGISTER_MASK & reg )); //status = SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );

  while ( len-- )
	  *buf++ = stmSpi2RdByte();

  nCS_NRF_H;

  return status;
}

/****************************************************************************/
void RF24_writeAckPayload(unsigned char pipe, void* buf, unsigned char len)
{
	const unsigned char* current = (const unsigned char*)(buf);//const unsigned char* current = reinterpret_cast<const unsigned char*>(buf);
	  const unsigned char max_payload_size = 32;
	  unsigned char data_len = Y_MIN(len,max_payload_size);

  nCS_NRF_L;
  stmSpi2WrByte( W_ACK_PAYLOAD | ( pipe & 0b111 ));

  while ( data_len-- )
	  stmSpi2WrByte(*current++);

  nCS_NRF_H;
}


/****************************************************************************/

unsigned char RF24_write_register_withLen(unsigned char reg, unsigned char* buf, unsigned char len)
{
  unsigned char status;

  nCS_NRF_L;
  status = stmSpi2WrByte(W_REGISTER | ( REGISTER_MASK & reg ));
  while ( len-- )
	  stmSpi2WrByte(*buf++);
  nCS_NRF_H;

  return status;
}

/****************************************************************************/

unsigned char RF24_write_register(unsigned char reg, unsigned char value)
{
  unsigned char status;

  printf("write_register(%02x,%02x)\r\n",reg,value);

  nCS_NRF_L;
  status = stmSpi2WrByte(W_REGISTER | ( REGISTER_MASK & reg ));
  stmSpi2WrByte(value);
  nCS_NRF_H;

  return status;
}

/****************************************************************************/

unsigned char RF24_write_payload(unsigned char *buf, unsigned char len)
{
  unsigned char status = 0;

  const unsigned char* current = buf;

  unsigned char data_len = Y_MIN(len,g_radio24.payload_size);
  unsigned char blank_len = g_radio24.dynamic_payloads_enabled ? 0 : g_radio24.payload_size - data_len;

  printf("[RF24_write_payload]Writing buf[0]=0x%02x(data=%u bytes; blanks=%u bytes)\r\n",current[0],data_len,blank_len);

  nCS_NRF_L;
  status = stmSpi2WrByte(W_TX_PAYLOAD);
  while ( data_len-- )
	  status = stmSpi2WrByte(*current++);
  while ( blank_len-- )
	  status = stmSpi2WrByte(0x00);
  nCS_NRF_H;

  //??? status ???
  return status;
}

/****************************************************************************/

unsigned char RF24_read_payload(void* buf, unsigned char len)
{
  unsigned char status;
  unsigned char* current = (unsigned char*)(buf);//unsigned char* current = reinterpret_cast<unsigned char*>(buf);
  unsigned char data_len = Y_MIN(len,g_radio24.payload_size);
  unsigned char blank_len = g_radio24.dynamic_payloads_enabled ? 0 : g_radio24.payload_size - data_len;

  printf("[Reading %u bytes %u blanks]\r\n",data_len,blank_len);

  nCS_NRF_L;
  status = stmSpi2WrByte(R_RX_PAYLOAD);
  while ( data_len-- )
    *current++ = stmSpi2RdByte();
  while ( blank_len-- )
	  stmSpi2RdByte();
  nCS_NRF_H;

  return status;
}

void RF24_toggle_features(void)
{
  nCS_NRF_L;
  stmSpi2WrByte(ACTIVATE);
  stmSpi2WrByte(0x73);//0x7e???? SPI.transfer( 0x73 );
  nCS_NRF_H;
}

/****************************************************************************/

unsigned char RF24_flush_rx(void)
{
  unsigned char status;

  nCS_NRF_L;
  status = stmSpi2WrByte(FLUSH_RX);
  nCS_NRF_H;

  return status;
}

/****************************************************************************/

unsigned char RF24_flush_tx(void)
{
  unsigned char status;

  nCS_NRF_L;
  status = stmSpi2WrByte(FLUSH_TX );
  nCS_NRF_H;

  return status;
}

/****************************************************************************/

unsigned char RF24_get_status(void)
{
  unsigned char status;

  nCS_NRF_L;
  status = (unsigned char) stmSpi2WrByte(NOP);
  nCS_NRF_H;

  return status;
}

/****************************************************************************/

void RF24_print_status(unsigned char status)
{
  printf("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x(PipeNum. if 7=Empty) TX_FULL=%x\r\n",
           status,
           (status & _BV(RX_DR))?1:0,
           (status & _BV(TX_DS))?1:0,
           (status & _BV(MAX_RT))?1:0,
           ((status >> RX_P_NO) & 0B111),
           (status & _BV(TX_FULL))?1:0
          );
}

/****************************************************************************/

void RF24_print_observe_tx(unsigned char value)
{
  printf("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n",
           value,
           (value >> PLOS_CNT) & 0B1111,
           (value >> ARC_CNT) & 0B1111
          );
}

/****************************************************************************/

void RF24_print_byte_register(const char* name, unsigned char reg, unsigned char qty)
{
  char extra_tab = strlen(name) < 8 ? '\t' : 0;
  printf("\t%s%c=",name,extra_tab);
  while (qty--)
    printf(" 0x%02x",RF24_read_register(reg++));
  printf("\r\n");
}

/****************************************************************************/

void RF24_print_address_register(const char* name, unsigned char reg, unsigned char qty)
{
  char extra_tab = strlen(name) < 8 ? '\t' : 0;
  printf("\t%s%c =",name,extra_tab);

  while (qty--)
  {
    unsigned char buffer[5];
    RF24_read_register_withLen(reg++, buffer, sizeof buffer);

    printf(" 0x");
    unsigned char* bufptr = buffer + sizeof(buffer);
    while( --bufptr >= buffer )
      printf("%02x",*bufptr);
  }

  printf("\r\n");
}

RF24_radio_Config_default(void)
{
	g_radio24.wide_band = true;
	g_radio24.p_variant = false;
	g_radio24.payload_size = 32;
	g_radio24.ack_payload_available = false;
	g_radio24.dynamic_payloads_enabled = false;
	g_radio24.pipe0_reading_address = 0;
}

/****************************************************************************/

void RF24_setChannel(unsigned char channel)
{
  // TODO: This method could take advantage of the 'wide_band' calculation
  // done in setChannel() to require certain channel spacing.

  const unsigned char max_channel = 127;
  RF24_write_register(RF_CH,Y_MIN(channel,max_channel));
}

/****************************************************************************/

void RF24_setPayloadSize(unsigned char size)
{
  const unsigned char max_payload_size = 32;
  g_radio24.payload_size = Y_MIN(size,max_payload_size);
}

/****************************************************************************/

unsigned char RF24_getPayloadSize(void)
{
  return g_radio24.payload_size;
}

/****************************************************************************/

static const char rf24_datarate_e_str_0[] = "1MBPS";
static const char rf24_datarate_e_str_1[] = "2MBPS";
static const char rf24_datarate_e_str_2[] = "250KBPS";
static const char * const rf24_datarate_e_str_P[]  = {
  rf24_datarate_e_str_0,
  rf24_datarate_e_str_1,
  rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[]  = "nRF24L01";
static const char rf24_model_e_str_1[]  = "nRF24L01+";
static const char * const rf24_model_e_str_P[]  = {
  rf24_model_e_str_0,
  rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] = "Disabled";
static const char rf24_crclength_e_str_1[] = "8 bits";
static const char rf24_crclength_e_str_2[] = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] = {
  rf24_crclength_e_str_0,
  rf24_crclength_e_str_1,
  rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] = "LA_MED";
static const char rf24_pa_dbm_e_str_3[] = "PA_HIGH";
static const char * const rf24_pa_dbm_e_str_P[] = {
  rf24_pa_dbm_e_str_0,
  rf24_pa_dbm_e_str_1,
  rf24_pa_dbm_e_str_2,
  rf24_pa_dbm_e_str_3,
};

void RF24_printDetails(void)
{

	RF24_print_status(RF24_get_status());

	RF24_print_address_register("RX_ADDR_P[0..1]",RX_ADDR_P0,2);//1); //??? YOON
	RF24_print_byte_register("RX_ADDR_P[2..5]",RX_ADDR_P2,4);//??? YOON
	RF24_print_address_register("TX_ADDR",TX_ADDR,1);//??? YOON

	RF24_print_byte_register("RX_PW_P[0..5]",RX_PW_P0,6);//??? YOON
	RF24_print_byte_register("EN_AA",EN_AA,1);
	RF24_print_byte_register("EN_RXADDR(Pipe#)",EN_RXADDR,1);
	RF24_print_byte_register("RF_CH",RF_CH,1);
	RF24_print_byte_register("RF_SETUP",RF_SETUP,1);
	RF24_print_byte_register("CONFIG",CONFIG,1);
	RF24_print_byte_register("DYNPD/FEATURE",DYNPD,2);

	printf("Data Rate\t = %s\r\n",rf24_datarate_e_str_P[RF24_getDataRate()]);
	printf("Model\t\t = %s\r\n",rf24_model_e_str_P[RF24_isPVariant()]);
	printf("CRC Length\t = %s\r\n",rf24_crclength_e_str_P[RF24_getCRCLength()]);
	printf("PA Power\t = %s\r\n",rf24_pa_dbm_e_str_P[RF24_getPALevel()]);

	delayms(1000);
}

/****************************************************************************/
void RF24_Spi_Gpio_Config(void)
{
	int i;
	GPIO_InitTypeDef GPIO_InitStruct;

	// Initialize pins
	//CE = PB10 -- Output -- It is not for nCS1.
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == STM32F107VCT6))
	//PB10
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //STM103
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);

#else
	//DE : PC8
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //STM407VGT6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_SetBits(GPIOC, GPIO_Pin_8);
#endif

	// Initialize SPI bus
    //Use nCS0(PB12), Mode 0, 8bit Mode,
	//Use nCS1(PB5), Mode 0, 8bit Mode,
    stmSPI2_Config(
    		2,//5, //5Mbps,...2Mbps
    		1,//use nCS1 on PB5       //0, --> use nCS0 on PB12
    		0, //Mode 0
    		8); //data 8 bit (or 16)

    nCS_NRF_H;

    RF24_ce(0); //Under CE=0 or 1, we can write/read registers
}

// Config nRF24L01 radio with default values.
void RF24_Radio_Config(void)
{
	int i;
    nCS_NRF_H;

    printf("RF24_Radio_Config()\r\n");

    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    delayms( 5 ) ;

  // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
  // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
  // sizes must never be used. See documentation for a more complete explanation.
    RF24_write_register(SETUP_RETR,(0B0100 << ARD) | (0B1111 << ARC));

  // Restore our default PA level
    RF24_setPALevel( RF24_PA_MAX ) ;

    // Determine if this is a p or non-p RF24 module and then
    // reset our data rate back to default value. This works
    // because a non-P variant won't allow the data rate to
    // be set to 250Kbps.
    if( RF24_setDataRate( RF24_250KBPS ) )
    {
    	g_radio24.p_variant = true ;
    }

  // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
    RF24_setDataRate( RF24_1MBPS ) ;

  // Initialize CRC and request 2-byte (16bit) CRC
    RF24_setCRCLength( RF24_CRC_16 ) ;

  // Disable dynamic payloads, to match dynamic_payloads_enabled setting
  RF24_write_register(DYNPD,0);

  // Reset current status by writing 1 for the releated bit.
  // Notice reset and flush is the last thing we do
  RF24_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  RF24_setChannel(76);

  // Flush buffers
  RF24_flush_rx();
  RF24_flush_tx();

  //check
  for(i=0;i<10;i++){
	  printf("ReadReg[%d] =0x%02x\r\n",i,RF24_read_register(i));
	  delayms(100);
  }
}

/****************************************************************************/

void RF24_startListening(void)
{
	unsigned char rdconf;

	printf("RF24_startListening\r\n");
	rdconf = RF24_read_register(CONFIG);
	RF24_write_register(CONFIG, rdconf | _BV(PWR_UP) | _BV(PRIM_RX));
	RF24_write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) ); //clear status bits.

  // Restore the pipe0 adddress, if exists
  if (g_radio24.pipe0_reading_address){
	  RF24_write_register_withLen(RX_ADDR_P0, (const unsigned char*)&g_radio24.pipe0_reading_address, 5);
  }

  // Flush buffers
  RF24_flush_rx();
  RF24_flush_tx();


  RF24_ce(1);// Go!

  // wait for the radio to come up (130us actually only needed)
  delayms(1);//delayMicroseconds(130);
}

/****************************************************************************/

void RF24_stopListening(void)
{
	RF24_ce(0);
	RF24_flush_tx();
	RF24_flush_rx();
}

/****************************************************************************/

void RF24_powerDown(void)
{
	RF24_write_register(CONFIG,RF24_read_register(CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/

void RF24_powerUp(void)
{
	RF24_write_register(CONFIG,RF24_read_register(CONFIG) | _BV(PWR_UP));
}

/******************************************************************/

unsigned char NRF24_WriteMsg( unsigned char* buf, unsigned char len )
{
  unsigned char result = false;
  unsigned char observe_tx = 0;
  unsigned char status = 0;
  uint32_t sent_at;
  unsigned char tx_ok, tx_fail;

  // Begin the write
  RF24_startWrite(buf,len);

  //by polling - check tx complete interrupt status in 500msec.
#if 1
  // ------------
  // At this point we could return from a non-blocking write, and then call
  // the rest after an interrupt

  // Instead, we are going to block here until we get TX_DS (transmission completed and ack'd)
  // or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
  // is flaky and we get neither.

  // IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
  // if I tighted up the retry logic.  (Default settings will be 1500us.
  // Monitor the send


  sent_at = millis();
  const uint32_t timeout = 500; //500ms to wait for timeout
  do {
    status = RF24_read_register_withLen(OBSERVE_TX,&observe_tx,1); //get observe_tx register for checking the lost or retransmissions.
    printf("NRF24_WriteMsg(): StatusReg = 0x%02x(observeTxReg = 0x%02x)\r\n",status, observe_tx);
    if(observe_tx != 0x00){ //in some errors.
    	printf("NRF24_WriteMsg(): observeTxReg=0x%02x,observeTxReg.lostCount=%u, observeTxReg.reTxCount=%u)\r\n",status, observe_tx,(observe_tx & 0xf0)>>4, (observe_tx & 0x0f));
    }
  }
  while( ! ( status & ( _BV(TX_DS) | _BV(MAX_RT) ) ) && ( millis() - sent_at < timeout ) );

  // The part above is what you could recreate with your own interrupt handler,
  // and then call this when you got an interrupt
  // ------------

  // Call this when you get an interrupt
  // The status tells us three things
  // * The send was successful (TX_DS)
  // * The send failed, too many retries (MAX_RT)
  // * There is an ack packet waiting (RX_DR)
  RF24_whatHappened(&tx_ok,&tx_fail,&g_radio24.ack_payload_available);
  printf("whatHappened: TxOK=0x%02x TxFail=0x%02x AckPayloadAvailable=0x%02x\r\n",
		  tx_ok,
		  tx_fail,
		  g_radio24.ack_payload_available);

  result = tx_ok;
  if(result)
	  printf("OK\r\n");
  else
	  printf("...Failed\r\n");
#endif

  // Handle the ack packet
  if ( g_radio24.ack_payload_available )
  {
	  g_radio24.ack_payload_length = RF24_getDynamicPayloadSize();
	  printf("[AckPacket]\r\n");
	  printf("len=%d\r\n",g_radio24.ack_payload_length);
  }

  //We are done.

  // Power down
  RF24_powerDown();

  // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
  RF24_flush_tx();

  return result;
}
/****************************************************************************/

void RF24_startWrite( unsigned char* buf, unsigned char len )
{
	unsigned char rdconf;

	printf("RF24_startWrite> length of %u\r\n",len);

	// Transmitter power-up
	rdconf = RF24_read_register(CONFIG);
	RF24_write_register(CONFIG,
			( rdconf | _BV(PWR_UP) )
			& ~_BV(PRIM_RX) //RX/TX Control : PRX=1; PTX=0
			);
	delayms(1);

	// Send the payload
	RF24_write_payload( buf, len );

	// Allons!
	RF24_ce(1);
	somedelay(10000);//delayms(1);//delayMicroseconds(15);
	RF24_ce(0);
}

/****************************************************************************/

unsigned char RF24_getDynamicPayloadSize(void)
{
  unsigned char result = 0;

  nCS_NRF_L;
  stmSpi2WrByte(R_RX_PL_WID);
  result = stmSpi2RdByte();
  nCS_NRF_H;

  return result;
}

/****************************************************************************/

unsigned char RF24_rcvd_available(void)
{
  return RF24_rcvd_available_withPipe(NULL);
}

/****************************************************************************/

unsigned char RF24_rcvd_available_withPipe(unsigned char* pipe_num)
{
  unsigned char status = RF24_get_status();

  // Too noisy, enable if you really want lots o data!!
  //if(status != 0x00){
	//  RF24_print_status(status);
  //}

  unsigned char result = ( status & _BV(RX_DR) );

  if (result)
  {
	  printf("StatusReg = 0x%02x\r\n", result);
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = ( status >> RX_P_NO ) & 0B111;


    // ??? Should this REALLY be cleared now?  Or wait until we actually READ the payload?
    RF24_write_register(STATUS,_BV(RX_DR) ); //clear status bit

    // Handle ack payload receipt
    if ( status & _BV(TX_DS) )
    {
    	RF24_write_register(STATUS,_BV(TX_DS));
    }
  }

  return result;
}

/****************************************************************************/

unsigned char RF24_read( void* buf, unsigned char len )
{
  // Fetch the payload
  RF24_read_payload( buf, len );

  // was this the last of the data available?
  return RF24_read_register(FIFO_STATUS) & _BV(RX_EMPTY);
}

/****************************************************************************/

void RF24_whatHappened(unsigned char *tx_ok,unsigned char  *tx_fail,unsigned char  *rx_ready)
{
  // Read the status & reset the status in one easy call
  // Or is that such a good idea?
  unsigned char status = RF24_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) ); //[NOTE] read and reset statusReg simultaneously

  // Report to the user what happened
  *tx_ok = status & _BV(TX_DS);
  *tx_fail = status & _BV(MAX_RT);
  *rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/

void RF24_openWritingPipe(unsigned long long ull_pipe)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

	RF24_write_register_withLen(RX_ADDR_P0, (unsigned char*)(&ull_pipe), 5);
	RF24_write_register_withLen(TX_ADDR, (unsigned char*)(&ull_pipe), 5);

	const unsigned char max_payload_size = 32;
	RF24_write_register(RX_PW_P0,Y_MIN(g_radio24.payload_size,max_payload_size));
}

/****************************************************************************/

static const unsigned char child_pipe[]  =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const unsigned char child_payload_size[] =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const unsigned char child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void RF24_openReadingPipe(unsigned char child, unsigned long long ull_address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0)
	  g_radio24.pipe0_reading_address = ull_address;

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 )
    	RF24_write_register_withLen(&child_pipe[child], (const unsigned char*)(&ull_address), 5);
    else
    	RF24_write_register_withLen(&child_pipe[child], (const unsigned char*)(&ull_address), 1);//write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const unsigned char*>(&address), 1);

    RF24_write_register(&child_payload_size[child],g_radio24.payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    RF24_write_register(EN_RXADDR, RF24_read_register(EN_RXADDR) | _BV(child_pipe_enable[child]));//RF24_write_register(EN_RXADDR, RF24_read_register(EN_RXADDR) | _BV(&child_pipe_enable[child]));
  }
}

/****************************************************************************/
void RF24_enableDynamicPayloads(void)
{
  // Enable dynamic payload throughout the system
	RF24_write_register(FEATURE,read_register(FEATURE) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! RF24_read_register(FEATURE) )
  {
    // So enable them and try again
    toggle_features();
    RF24_write_register(FEATURE,RF24_read_register(FEATURE) | _BV(EN_DPL) );
  }

  printf("FEATURE=%i\r\n",RF24_read_register(FEATURE));

  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  RF24_write_register(DYNPD,RF24_read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  g_radio24.dynamic_payloads_enabled = true;
}

/****************************************************************************/

void RF24_enableAckPayload(void)
{
  //
  // enable ack payload and dynamic payload features
  //

	RF24_write_register(FEATURE,RF24_read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! RF24_read_register(FEATURE) )
  {
    // So enable them and try again
	  RF24_toggle_features();
	  RF24_write_register(FEATURE,RF24_read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
  }

  printf("FEATURE=%i\r\n",RF24_read_register(FEATURE));

  //
  // Enable dynamic payload on pipes 0 & 1
  //

  RF24_write_register(DYNPD,RF24_read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}


unsigned char RF24_isAckPayloadAvailable(void)
{
  unsigned char result = g_radio24.ack_payload_available;
  g_radio24.ack_payload_available = false;
  return result;
}

unsigned char RF24_isPVariant(void)
{
  return g_radio24.p_variant ;
}

void RF24_setAutoAck(unsigned char enable)
{
  if ( enable )
    RF24_write_register(EN_AA, 0B111111);
  else
	  RF24_write_register(EN_AA, 0);
}

void RF24_setAutoAck_withPipe( unsigned char pipe, unsigned char enable )
{
  if ( pipe <= 6 )
  {
    unsigned char en_aa = RF24_read_register( EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    RF24_write_register( EN_AA, en_aa ) ;
  }
}

/****************************************************************************/

unsigned char RF24_testCarrier(void)
{
  return ( RF24_read_register(CD) & 1 );
}

/****************************************************************************/

unsigned char RF24_testRPD(void)
{
  return ( RF24_read_register(RPD) & 1 ) ;
}

/****************************************************************************/

void RF24_setPALevel(rf24_pa_dbm_e level)
{
  unsigned char setup = RF24_read_register(RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_MAX )
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  RF24_write_register( RF_SETUP, setup ) ;
}

/****************************************************************************/

rf24_pa_dbm_e RF24_getPALevel(void)
{
  rf24_pa_dbm_e result = RF24_PA_ERROR ;
  unsigned char power = RF24_read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
  {
    result = RF24_PA_MAX ;
  }
  else if ( power == _BV(RF_PWR_HIGH) )
  {
    result = RF24_PA_HIGH ;
  }
  else if ( power == _BV(RF_PWR_LOW) )
  {
    result = RF24_PA_LOW ;
  }
  else
  {
    result = RF24_PA_MIN ;
  }

  return result ;
}

/****************************************************************************/

unsigned char RF24_setDataRate(rf24_datarate_e speed)
{
  unsigned char result = false;
  unsigned char setup = RF24_read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  g_radio24.wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    g_radio24.wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
    	g_radio24.wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
    	g_radio24.wide_band = false ;
    }
  }
  RF24_write_register(RF_SETUP,setup);

  // Verify our result
  if ( RF24_read_register(RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
	  g_radio24.wide_band = false;
  }

  return result;
}

rf24_datarate_e RF24_getDataRate( void )
{
  rf24_datarate_e result ;
  unsigned char dr = RF24_read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}

void RF24_setCRCLength(rf24_crclength_e length)
{
  unsigned char config = RF24_read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above.
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  RF24_write_register( CONFIG, config ) ;
}

rf24_crclength_e RF24_getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  unsigned char config = RF24_read_register(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

  if ( config & _BV(EN_CRC ) )
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

void RF24_disableCRC( void )
{
  unsigned char disable = RF24_read_register(CONFIG) & ~_BV(EN_CRC) ;
  RF24_write_register( CONFIG, disable ) ;
}


void RF24_setRetries(unsigned char delay, unsigned char count)
{
	RF24_write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

//=============================APP Exampels =============================
/**
* @example nordic_fob.pde
*
* This is an example of how to use the RF24 class to receive signals from the
* Sparkfun Nordic FOB.  See http://www.sparkfun.com/products/8602 .
* Thanks to Kirk Mower for providing test hardware.
*/

/**
* @example led_remote.pde
*
* This is an example of how to use the RF24 class to control a remote
* bank of LED's using buttons on a remote control.
*
* Every time the buttons change on the remote, the entire state of
* buttons is send to the led board, which displays the state.
*/

/**
* @example pingpair.pde
*
* This is an example of how to use the RF24 class.  Write this sketch to two
* different nodes, connect the role_pin to ground on one.  The ping node sends
* the current time to the pong node, which responds by sending the value back.
* The ping node can then see how long the whole cycle took.
*/

/**
* @example pingpair_maple.pde
*
* This is an example of how to use the RF24 class on the Maple.  For a more
* detailed explanation, see my blog post:
* <a href="http://maniacbug.wordpress.com/2011/12/14/nrf24l01-running-on-maple-3/">nRF24L01+ Running on Maple</a>
*
* It will communicate well to an Arduino-based unit as well, so it's not for only Maple-to-Maple communication.
*
* Write this sketch to two different nodes,
* connect the role_pin to ground on one.  The ping node sends the current time to the pong node,
* which responds by sending the value back.  The ping node can then see how long the whole cycle
* took.
*/

/**
* @example starping.pde
*
* This sketch is a more complex example of using the RF24 library for Arduino.
* Deploy this on up to six nodes.  Set one as the 'pong receiver' by tying the
* role_pin low, and the others will be 'ping transmit' units.  The ping units
* unit will send out the value of millis() once a second.  The pong unit will
* respond back with a copy of the value.  Each ping unit can get that response
* back, and determine how long the whole cycle took.
*
* This example requires a bit more complexity to determine which unit is which.
* The pong receiver is identified by having its role_pin tied to ground.
* The ping senders are further differentiated by a byte in eeprom.
*/

/**
* @example pingpair_pl.pde
*
* This is an example of how to do two-way communication without changing
* transmit/receive modes.  Here, a payload is set to the transmitter within
* the Ack packet of each transmission.  Note that the payload is set BEFORE
* the sender's message arrives.
*/

/**
* @example pingpair_irq.pde
*
* This is an example of how to user interrupts to interact with the radio.
* It builds on the pingpair_pl example, and uses ack payloads.
*/

/**
* @example pingpair_sleepy.pde
*
* This is an example of how to use the RF24 class to create a battery-
* efficient system.  It is just like the pingpair.pde example, but the
* ping node powers down the radio and sleeps the MCU after every
* ping/pong cycle.
*/

/**
* @example scanner.pde
*
* Example to detect interference on the various channels available.
* This is a good diagnostic tool to check whether you're picking a
* good channel for your application.
*
* Inspired by cpixip.
* See http://arduino.cc/forum/index.php/topic,54795.0.html
*/
/**
 * Example LED Remote
 *
 * This is an example of how to use the RF24 driver to control a remote
 * bank of LED's using buttons on a remote control.
 *
 * ['remote' Module-Master Transmitter]: pressing buttons to on/off its peer led module.
 * ['led' Module-Slave Receiver]:  Update 'led_pins' to reflect the pins used.
 *   - 'role_pin': 0 = it's running as LED board.
 */



//User button config for Role Selection
void RF24_App_LED_And_RoleButton_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	//(1) LED
	stmUser_LED_GPIO_setup();//PC14/PC13- STM32F103KONG/LEAN (Active High)

	#if 1
	GPIO_SetBits(GPIOC, GPIO_Pin_14); 	//LED ON --STM103 KONG
	delayms(1000);
	GPIO_ResetBits(GPIOC, GPIO_Pin_14);// LED off -- STM103 KONG
	delayms(1000);
	#else
	GPIO_SetBits(GPIOC, GPIO_Pin_13); 	//LED ON --STM103 LEAN
	delayms(1000);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);// LED off -- STM103 LEAN
	delayms(1000);
	#endif

	//(2)Role Button : set up the role using user button Input (with Active Low)
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == STM32F107VCT6))
	#if 1
	//PC15
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //STM103-KONG
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	#else
	//PA15
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //STM103-LEAN
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	#endif
#else
#endif
	delayms(20); // Just to get a solid reading on the role pin

	//determine my role with reading user button.
#if 0
	if(role = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)){
		printf("Role is Led Slave Receiver\r\n");
	}else{
		printf("Role is Remote Controller Master Transmitter\r\n");
	}
#else
	if(role = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)){
		printf("BTN 1 : Role is Led Slave Receiver\r\n");
	}else{
		printf("BTN 0 : Role is Remote Controller Master Transmitter\r\n");
	}

#endif

	// Open pipes to other nodes for communication
	// We simply open a single pipes for these two nodes to communicate back and forth.
	// One listens on it, the other talks to it.

	if ( role == role_master )
	{
		printf("\r\nAs a master, openWritingPipe\r\n");
		RF24_openWritingPipe(pipe);
	}
	else
	{
		printf("\r\nAs a slave, openReadingPipe\r\n");
		RF24_openReadingPipe(1,pipe);
	}

	// Start listening if I am a slave
	if ( role == role_led_slave ){
		printf("\r\nAs a slave, I am listening\r\n");
		RF24_startListening();
	}

	// Dump the configuration of the rf unit for debugging
	RF24_printDetails();
}

//
// Loop
//
void stm_RF24_loop(void)
{
	static unsigned char b_rxdone;
	static unsigned char b_ok;
	static unsigned char b_toggle;
	static unsigned char st_button;

    printf("NRF24L01 Wireless Test with SPI Mode 0, 5Mbps, 8bit data.\r\n");

    RF24_Spi_Gpio_Config();
    RF24_radio_Config_default();
    RF24_Radio_Config();
    RF24_App_LED_And_RoleButton_Config();

    while(1){
    	// Remote role.  If the state of any button has changed, send the whole state of all buttons.
    	if ( role == role_master )
    	{
    		// Get the current state of buttons, and test if the current state is different from the last state we sent
    		b_toggle = false;
    		st_button = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15);//st_button = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15);//read from button
    		if ( st_button != button_states[0] ){
    			b_toggle = true;
    			button_states[0] = st_button;
    		}

    		// Send the state of the buttons to the LED board
    		if ( b_toggle )	{
    			printf("[Master]Now sending...\r\n");

    			b_ok = NRF24_WriteMsg( button_states, 1 );

    			if (b_ok)			printf("[Master]Tx OK\r\n");
    			else    			printf("[Master]TX failed\r\n");
    		}

    		// Try again in a short while
    		delayms(20);
    	}
    	// LED role.  Receive the state of all buttons, and reflect that in the LEDs
    	else if ( role == role_led_slave )
    	{
    		// if there is data ready
    		if (RF24_rcvd_available())
    		{
    			printf("[Slave] Rx Frame\n\r");
    			// Dump the payloads until we've gotten everything
    			b_rxdone = false;
    			while (!b_rxdone)
    			{
    				// Fetch the payload, and see if this was the last one.
    				b_rxdone = RF24_read( button_states, 1);//num_button_pins );

    				if ( button_states[0] )
    				{
    					led_states[0] ^= 1;// toggle

    					//LED OUT
    					if(led_states[0]){
    						GPIO_SetBits(GPIOC, GPIO_Pin_14);
    						printf("[Slave] LED ON\n\r");
    					}
    					else{
    						GPIO_ResetBits(GPIOC, GPIO_Pin_14);
    						printf("[Slave] LED OFF\n\r");
    					}
    				}
    			}
    		}
    	}
    }//while
}


