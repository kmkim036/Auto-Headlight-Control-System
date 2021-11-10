//== w5100s and w5500
/* --- REF: github/Wiznet/Ethernet
 * --- by taylor-an
* Copyright (c) 2010 by WIZnet <support@wiznet.co.kr>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

// WORKING BUT TO BE REFINED ! YOON

//+--------+-----------+-----------+-----------+---------+---------+--------+
//|        |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   | 103
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| MOSI   |           |           |           |         |         |PB15    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| MISO   |           |           |           |         |         |PB14    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| SCLK   |           |           |           |         |         |PB13    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| nCS0   |           |           |           |         |         |PB12    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| nCS2   |           |           |           |         |         |PA7     |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| IRQ0/1 |           |           |           |         |         |PB3/PB4 |
//+--------+-----------+-----------+-----------+---------+---------+--------+

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "ySpiDrv.h"

//============================================
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))
extern void stmSpi2_Config(unsigned char nCS);
extern unsigned char stmSpi2RdByte();
extern void stmSpi2WrByte(unsigned char inbyte);
#else
//...
#endif

//extern void stmLedToggle(void);
//extern void stmUser_LED_GPIO_setup(void);

//This program can support both W5100S and W5500.
//These two chips seem to be similar, but those have different register maps and SPI access methods.
//5100S
	//OffsetAddressMapping =0
	//max SOCK NUM   4
    //max TXBUF_SIZE  8kBytes @ 0x4000
	//max RXBUF_SIZE  8kBytes @ 0x6000
	//Using specific Registers : W5100_RMSR@0x001A(Receive memory size)
	//                         : W5100_TMSR@0x001B(Transmit memory size)
	//                         : Chip Version@0x0080  //Chip Version Register(W5100S only) -- value = 0x81
//5500
	//OffsetAddressMapping =1
	//max SOCK NUM   8
    //max TXBUF_SIZE  16kBytes
	//max RXBUF_SIZE  16kBytes

#define USE_W5100S 	1
#define USE_W5500  	2

#define USE_W5XXX 	USE_W5100S //USE_W5500 //

//== SPI == use nCS2 of PA7; use nCS0 of PB12 - 103;
#define nCS_W5XXX_H nCS2_H
#define nCS_W5XXX_L nCS2_L

//============================
struct _IPAddress{
	union {
		unsigned char bytes[4];
		unsigned long dword;
	};
};
typedef struct _IPAddress IPAddress;
typedef unsigned char SOCKET;

//W5100S and W5500 have different register maps.

#if (USE_W5XXX == USE_W5100S)

#define W5XXX_hasOffsetAddressMapping 0

#define MAX_SOCK_NUM 		1//..4   // Select the number of Sockets (1-4)
#define RXBUF_SIZE   		(8/MAX_SOCK_NUM) //2   // Select the Receiver buffer size (1 - 8.192 kBytes)
#define TXBUF_SIZE   		(8/MAX_SOCK_NUM)//2   // Select the Transmitter buffer size (1 - 8 kBytes)

#define W5100_SN_BASE 		(0x0400)
#define W5100_SN_SIZE 		(0x0100)
#define W5100_TXBUF_BASE 	(0x4000) //Internal Tx Buffer Address
#define W5100_RXBUF_BASE	(0x6000) //Internal Rx Buffer Address

//Basic Registers
#define W5100_RMSR       	0x001A    // Receive memory size (W5100 only) : Config RX buffer size per socket.
#define W5100_TMSR       	0x001B    // Transmit memory size (W5100 only): Config TX buffer size per socket. Sum of the buffers should be under 8KB.
#define W5XXX_PHYSR0       	0x003C //PHY Status Register 0

#define W5XXX_PHYAR        	0x003E //PHY Address
#define W5XXX_PHYRAR       	0x003F //PHY Register Address
#define W5XXX_PHYDIR       	0x0040 //PHY Data Input
#define W5XXX_PHYDOR       	0x0042 //PHY Data Output
#define W5XXX_PHYACR       	0x0042 //PHY Action
#define W5XXX_PHYCR0       	0x0046 //PHY Control0
#define W5XXX_PHYCR1       	0x0047 //PHY Control1
#define W5XXX_VERR     	   	0x0080  //Chip Version Register(W5100S only) -- value = 0x81

//Registers per socket
#define SN_MODE_REG 		0x0400
#define SN_CMD_REG 			0x0401
#define SN_INT_REG 			0x0402
#define SN_STATUS_REG 		0x0403 //SOCK_CLOSED,...SOCK_ESTAB,...
#define SN_PORT_REG 		0x0404 //Source Port
#define SN_DEST_HARDWARE_ADDR_REG 		0x0406 //Peer MAC Address
#define SN_DEST_IP_ADDR_REG 0x040C //Peer IP Address
#define SN_DEST_PORT_REG 	0x0410 //Peer Port
//...
#define SN_RXBUF_SIZE_REG 	0x041E  //Rx Memory Size (1K,2K,4K,8K).
#define SN_TXBUF_SIZE_REG 	0x041F  //Tx Memory Size (1K,2K,4K,8K).
#define SN_TX_FREE_BUF_SIZE_REG 	0x0420  //Tx Free Memory Size. Used to check before moving data into tx buffer memory in the chip.
//..
#define SN_TX_BUFFER_READ_POINTER_REG 	0x0422  //On send command, the chip begins transmit from this read pointer.
#define SN_TX_BUFFER_WRITE_POINTER_REG 	0x0424  //To send command, MCU moves data from this write pointer. After finishing a whole data of N bytes, we then add(update?) N to this register.

#define SN_RX_DATA_SIZE_REG 		0x0426  //On receving, it holds the received data size.
#define SN_RX_READ_POINTER_REG 		0x0428  //MCU begins read data from this pointer of RXbuf.
#define SN_RX_WRITE_POINTER_REG 	0x042A  //Chip store data in the RXBUF at this pointer.
//...

#define DUMMY 		 0x00			//provides compatibility of W5100S and W5500 for spi access.

static unsigned short W5XXX_SSIZE = 2048; // Max a single Tx buffer size
static unsigned short W5XXX_RSIZE = 2048; // Max a single Rx buffer size
static unsigned short W5XXX_SMASK = 0x07FF;// = (W5XXX_SSIZE - 1);


#elif (USE_W5XXX == USE_W5500)

/** W5500's yotal RAM buffer is 16 kBytes for Transmitter and 16 kBytes for receiver for 1 Socket.
 *  The Total W5500 RAM buffer is 32 kBytes (16 + 16).
 *  If you use more Sockets then the RAM buffer must be split.
 *  For example: if you use 2 Sockets then all W5500_socket must use upto 16 kBytes in total RAM.
 *  So, we have:
 *
 *  #define MAX_SOCK_NUM 2   // Select two Sockets.
 *  #define RXBUF_SIZE   8   // The Receiver buffer size will be 8 kBytes
 *  #define TXBUF_SIZE   8   // The Transmitter buffer size will be 8 kBytes
 *
 *  In total we use (2 Sockets)*(8 kBytes) for transmitter + (2 Sockets)*(8 kBytes) for receiver = 32 kBytes.
 *
 *  I would prefer to use only 1 Socket with 16 kBytes RAM buffer for transmitter and 16 kByte for receiver buffer.
 *
 *  #define MAX_SOCK_NUM 1   // Select only one Socket
 *  #define RXBUF_SIZE   16  // Select 16 kBytes Receiver RAM buffer
 *  #define TXBUF_SIZE   16  // Select 16 kBytes Transmitter RAM buffer
 */

#define W5XXX_hasOffsetAddressMapping 1

#define MAX_SOCK_NUM 1//..8   // Select the number of Sockets (1-8)
#define RXBUF_SIZE   16//2   // Select the Receiver buffer size (1 - 16 kBytes)
#define TXBUF_SIZE   16//2   // Select the Transmitter buffer size (1 - 16 kBytes)

//Registers
#define W5XXX_PHYSR0        0x2E  //PHY Status Register 0
#define W5XXX_VERR      0x39  //Chip Version Register(5500) (value =4)

static unsigned short W5XXX_SSIZE = 2048; // Max a single Tx buffer size
static unsigned short W5XXX_RSIZE = 2048; // Max a single Rx buffer size
static unsigned short W5XXX_SMASK = 0x07FF;//(W5XXX_SSIZE - 1); //

#endif

//-------- For the basic mode register(MR)@0x0000 : Socket Protocol -------------------------
#define SnMR_CLOSE   0x00
#define SnMR_TCP     0x01
#define SnMR_UDP     0x02
#define SnMR_IPRAW   0x03
#define SnMR_MACRAW  0x04
#define SnMR_PPPOE   0x05
#define SnMR_ND      0x20
#define SnMR_MULTI   0x80

//-------- TCP Socket Status -------------------------
typedef enum  {
  Sock_OPEN      = 0x01,
  Sock_LISTEN    = 0x02,
  Sock_CONNECT   = 0x04,
  Sock_DISCON    = 0x08,
  Sock_CLOSE     = 0x10,
  Sock_SEND      = 0x20,
  Sock_SEND_MAC  = 0x21,
  Sock_SEND_KEEP = 0x22,
  Sock_RECV      = 0x40
} SockCMD;
//-------- Per socket Interrupt Status
#define SnIR_SEND_OK  0x10
#define SnIR_TIMEOUT 0x08
#define SnIR_RECV    0x04
#define SnIR_DISCON  0x02
#define SnIR_CON     0x01
//-------- Per socket TCP Socket Status Register Value-------------------------
  static const unsigned char SnSR_CLOSED      = 0x00;
  static const unsigned char SnSR_INIT        = 0x13;
  static const unsigned char SnSR_LISTEN      = 0x14;
  static const unsigned char SnSR_SYNSENT     = 0x15;
  static const unsigned char SnSR_SYNRECV     = 0x16;
  static const unsigned char SnSR_ESTABLISHED = 0x17;
  static const unsigned char SnSR_FIN_WAIT    = 0x18;
  static const unsigned char SnSR_CLOSING     = 0x1A;
  static const unsigned char SnSR_TIME_WAIT   = 0x1B;
  static const unsigned char SnSR_CLOSE_WAIT  = 0x1C;
  static const unsigned char SnSR_LAST_ACK    = 0x1D;
  static const unsigned char SnSR_UDP         = 0x22;
  static const unsigned char SnSR_IPRAW       = 0x32;
  static const unsigned char SnSR_MACRAW      = 0x42;
  static const unsigned char SnSR_PPPOE       = 0x5F;

//-------- Protocols -----
  static const unsigned char IPPROTO_IP   = 0;
  static const unsigned char IPPROTO_ICMP = 1;
  static const unsigned char IPPROTO_IGMP = 2;
  static const unsigned char IPPROTO_GGP  = 3;
  static const unsigned char IPPROTO_TCP  = 6;
  static const unsigned char IPPROTO_PUP  = 12;
  static const unsigned char IPPROTO_UDP  = 17;
  static const unsigned char IPPROTO_IDP  = 22;
  static const unsigned char IPPROTO_ND   = 77;
  static const unsigned char IPPROTO_RAW  = 255;

  //Protos
  //void recv_data_processing(SOCKET s, unsigned char *data, unsigned short len, unsigned char peek);

  inline void setGatewayIp(unsigned char *_addr);
  inline void getGatewayIp(unsigned char *_addr);

  inline void setSubnetMask(unsigned char *_addr);
  inline void getSubnetMask(unsigned char *_addr);

  inline void setMACAddress(unsigned char * addr);
  inline void getMACAddress(unsigned char * addr);

  inline void setIPAddress(unsigned char * addr);
  inline void getIPAddress(unsigned char * addr);

  inline void setRetransmissionTime(unsigned short timeout);
  inline void setRetransmissionCount(unsigned char _retry);

  inline void setPHYCFGR(unsigned char _val);
  inline unsigned char getPHYCFGR();

  void execCmdSn(SOCKET s, SockCMD _cmd);

  unsigned short getTXFreeSize(SOCKET s);
  unsigned short W5XXX_getRXReceivedSize(SOCKET s);

  //============================================================
    int  W5XXX_ResetAndConfigBuf();
    void W5XXX_read_data(SOCKET s, volatile unsigned short  src, volatile unsigned char * dst, unsigned short len);
    void W5XXX_socket_send_data_processing(SOCKET s, const unsigned char *data, unsigned short len);
    void W5XXX_write_data_with_offset_for_send(SOCKET s, unsigned short data_offset, const unsigned char *data, unsigned short len);

unsigned char *W5XXX_getRaw_address(IPAddress ipaddr){
  	  return ipaddr.bytes; //ptr
}

//============== SPI CONFIG ========================
void stmW5XXX_SpiConfig(void){
	//2Mbps, 8 bit mode
	//use MODE 0
#if (PROCESSOR == STM32F401RET6)
	stmSPI1_Config(0,0,8); //use PA15 of nCS0
#else
	stmSPI2_Config(10, //5Mbps
			2, //use PA7 of nCS2
			0, //SPI MODE 0
			8); //8-bit transaction
#endif
	nCS_W5XXX_H;//nCs=1

	printf("W5XXX> SPI CONFIG DONE.\r\n");
}

#if (USE_W5XXX == USE_W5100S)
//+----------+-------+------+-----------------------------------------+
//|F0(WRITE)  |    REG ADDR  | DATA...                                 |
//+----------+-------+------+-----------------------------------------+
unsigned char W5XXX_spi_write_Single(unsigned short _addr,
		unsigned char _cb, //dummy
		unsigned char _data)
{
	nCS_W5XXX_L;
	stmSpi2WrByte(0xF0);  	// control byte for Write
	stmSpi2WrByte(_addr >> 8); //2 byte reg addr
	stmSpi2WrByte(_addr & 0xFF);
	stmSpi2WrByte(_data); 	//data phase
	nCS_W5XXX_H;
	return 1;
}

unsigned short W5XXX_spi_write_Multi(unsigned short _addr,
		unsigned char _cb, //dummy
		const unsigned char *_buf,
		unsigned short _len)
{
	unsigned short i;
	nCS_W5XXX_L;
	stmSpi2WrByte(0xF0);// control byte for Write
	stmSpi2WrByte(_addr >> 8);
	stmSpi2WrByte(_addr & 0xFF);

	for (i=0; i<_len; i++){
		stmSpi2WrByte(_buf[i]);
	}
	nCS_W5XXX_H;
    return _len;
}

//+----------+-------+------+-----------------------------------------+
//|0F(READ)  |    REG ADDR  | DATA...                                 |
//+----------+-------+------+-----------------------------------------+
unsigned char W5XXX_spi_read_Single(unsigned short _addr,
		unsigned char _cb) //dummy
{
	unsigned char _data;
	nCS_W5XXX_L;
	stmSpi2WrByte(0x0F);// control byte for Read
	stmSpi2WrByte(_addr >> 8);
	stmSpi2WrByte(_addr & 0xFF);

	_data = stmSpi2RdByte();
	nCS_W5XXX_H;
    return _data;
}

unsigned short W5XXX_spi_read_Multi(unsigned short _addr,
		unsigned char _cb, //dummy
		unsigned char *_buf, unsigned short _len)
{
	unsigned short i;
	nCS_W5XXX_L;
	stmSpi2WrByte(0x0F);// control byte for Read
	stmSpi2WrByte(_addr >> 8);
	stmSpi2WrByte(_addr & 0xFF);

	for (i=0; i<_len; i++){
        _buf[i] = stmSpi2RdByte();
	}
	nCS_W5XXX_H;
    return _len;
}

#elif (USE_W5XXX == USE_W5500)
//+-----+-----+-------+-----------------------------------------+
//|REG ADDR   |CTRL   | DATA...                                 |
//+-----------+-------+-----------------------------------------+
//CTRL (cb) = (0b0000-0101)= pageSelect[7:3]-R(0)/W(1)-length[1:0](01=1byte)

unsigned char W5XXX_spi_write_Single(unsigned short _addr, unsigned char _cb, unsigned char _data)
{
	nCS_W5XXX_L;
	stmSpi2WrByte(_addr >> 8); //2 byte reg addr
	stmSpi2WrByte(_addr & 0xFF);
	stmSpi2WrByte(_cb);  // control byte
	stmSpi2WrByte(_data); //data phase
	nCS_W5XXX_H;
	return 1;
}

unsigned short W5XXX_spi_write_Multi(unsigned short _addr, unsigned char _cb, const unsigned char *_buf, unsigned short _len)
{
	unsigned short i;
	nCS_W5XXX_L;
	stmSpi2WrByte(_addr >> 8);
	stmSpi2WrByte(_addr & 0xFF);
	stmSpi2WrByte(_cb);
	for (i=0; i<_len; i++){
		stmSpi2WrByte(_buf[i]);
	}
	nCS_W5XXX_H;
    return _len;
}

unsigned char W5XXX_spi_read_Single(unsigned short _addr, unsigned char _cb)
{
	nCS_W5XXX_L;
	stmSpi2WrByte(_addr >> 8);
	stmSpi2WrByte(_addr & 0xFF);
	stmSpi2WrByte(_cb);
	unsigned char _data = stmSpi2RdByte();
	nCS_W5XXX_H;
    return _data;
}

unsigned short W5XXX_spi_read_Multi(unsigned short _addr, unsigned char _cb, unsigned char *_buf, unsigned short _len)
{
	unsigned short i;
	nCS_W5XXX_L;
	stmSpi2WrByte(_addr >> 8);
	stmSpi2WrByte(_addr & 0xFF);
	stmSpi2WrByte(_cb);

  for (i=0; i<_len; i++){
        _buf[i] = stmSpi2RdByte();
  }
	nCS_W5XXX_H;
    return _len;
}

#endif

#if (USE_W5XXX == USE_W5100S)

//GatewayAddrReg@0x0001~4.
void W5XXX_getGatewayIp(unsigned char *_addr) {
	W5XXX_spi_read_Multi(0x0001, DUMMY, _addr, 4);
}
void W5XXX_setGatewayIp(unsigned char *_addr) {
	W5XXX_spi_write_Multi(0x0001, DUMMY, _addr, 4);
}
//SubnetMaskReg@0x5~8
void W5XXX_getSubnetMask(unsigned char *_addr) {
	W5XXX_spi_read_Multi(0x0005, DUMMY, _addr, 4);
}

void W5XXX_setSubnetMask(unsigned char *_addr) {
	W5XXX_spi_write_Multi(0x0005, DUMMY, _addr, 4);
}

//SourceHWAddr@0x9~E
void W5XXX_getMACAddress(unsigned char *_addr) {
    W5XXX_spi_read_Multi(0x0009, DUMMY, _addr, 6);
}

void W5XXX_setMACAddress(unsigned char *_addr){
	W5XXX_spi_write_Multi(0x0009, DUMMY, _addr, 6);
}

//SourceIPAddr@0x0F~0x12
void W5XXX_getIPAddress(unsigned char *_addr) {
	W5XXX_spi_read_Multi(0x000F, DUMMY, _addr, 4);
}

void W5XXX_setIPAddress(unsigned char *_addr) {
	W5XXX_spi_write_Multi(0x000F, DUMMY, _addr, 4);
}

//RTR0@0x17~18
void W5XXX_setRetransmissionTime(unsigned short _timeout) {
	W5XXX_spi_read_Multi(0x0017, DUMMY, _timeout, 2);  //W5XXX_writeRTR(_timeout);
}
// RCR@0x19 -- YOON
void W5XXX_setRetransmissionCount(unsigned char _retry) {
	W5XXX_spi_read_Multi(0x0019, DUMMY, _retry, 1);  //W5XXX_writeRCR(_retry);
}
/* --???
void W5XXX_setPHYCFGR(unsigned char _val) {
	W5XXX_writePHYCFGR(_val);
}

unsigned char W5XXX_getPHYCFGR() {
//  readPHYCFGR();
  return W5XXX_read(0x002E, 0x00);
}
*/


//================== per socket related =================================================
unsigned char W5XXX_readSn_Single(SOCKET s, unsigned short baseRegAddr) {
    unsigned short addr = baseRegAddr + (s)*0x100; // unsigned short addr = _baseRegAddr + (_s << 8);
    return W5XXX_spi_read_Single(addr, DUMMY);
}

unsigned char W5XXX_writeSn_Single(SOCKET s, unsigned short baseRegAddr, unsigned char data) {
	unsigned short addr = baseRegAddr + (s)*0x100; // unsigned short addr = baseRegAddr + (_s << 8);
    return W5XXX_spi_write_Single(addr, DUMMY, data);
}

unsigned short W5XXX_readSn_Multi(SOCKET s, unsigned short baseRegAddr, unsigned char *buf, unsigned short len) {
	unsigned short addr = baseRegAddr + (s)*0x100; // unsigned short addr = _baseRegAddr + (_s << 8);
    return W5XXX_spi_read_Multi(addr, DUMMY, buf, len );
}

unsigned short W5XXX_writeSn_Multi(SOCKET s, unsigned short baseRegAddr, unsigned char *buf, unsigned short len) {
	unsigned short addr = baseRegAddr + (s)*0x100; // unsigned short addr = _baseRegAddr + (_s << 8);
    return W5XXX_spi_write_Multi(addr, DUMMY, buf, len);
}
//=============================================================================
//Sn_MR (Socket Mode) $ 0x0401 +(0x100 x n)
static void W5XXX_writeSnMR(SOCKET _s, unsigned char _data) {
	W5XXX_writeSn_Single(_s, 0x0400,  _data);
}

//Sn_CR (Socket Command) $ 0x0401 +(0x100 x n)
static void W5XXX_writeSnCR(SOCKET _s, unsigned char _data) {
	W5XXX_writeSn_Single(_s, 0x0401,  _data);
}

static unsigned char W5XXX_readSnCR(SOCKET _s) {
    unsigned char res;
    res = W5XXX_readSn_Single(_s, 0x0401);
    return res;
}

//Sn_IR (Socket Interrupt) $ 0x0402 +(0x100 x n)
static void W5XXX_writeSnIR(SOCKET _s, unsigned char _data) {
	W5XXX_writeSn_Single(_s, 0x0402,  _data);
}
static unsigned char W5XXX_readSnIR(SOCKET _s) {
    unsigned char res;
    res = W5XXX_readSn_Single(_s, 0x0402);
    return res;
}

//Sn_SR (Socket Status) $ 0x0403 +(0x100 x n)
static void W5XXX_writeSnSR(SOCKET _s, unsigned char _data) {
	W5XXX_writeSn_Single(_s, 0x0403,  _data);
}
static unsigned char W5XXX_readSnSR(SOCKET _s) {
    unsigned char res;
    res = W5XXX_readSn_Single(_s, 0x0403);
    return res;
}

//Sn_Port0 and 1 (Socket Source Port) $ 0x0404~0x0405+(0x100 x n)
static void W5XXX_writeSnPORT(SOCKET _s, unsigned short _data) {
	W5XXX_writeSn_Single(_s, 0x0404,   _data >> 8);
	W5XXX_writeSn_Single(_s, 0x0404 + 1, _data & 0xFF);
}

static unsigned short W5XXX_readSnPORT(SOCKET _s) {
    unsigned short res;
    res = W5XXX_readSn_Single(_s, 0x0404);
    res = (res << 8) + W5XXX_readSn_Single(_s, 0x0404 + 1);
    return res;
}


//Sn_TX_FSR0 and 1 (TxFreeSize) $ 0x0420~0x0421+(0x100 x n)
static void W5XXX_writeSnTX_FSR(SOCKET _s, unsigned short _data) {
	W5XXX_writeSn_Single(_s, 0x0420,   _data >> 8);
	W5XXX_writeSn_Single(_s, 0x0420 + 1, _data & 0xFF);
}

static unsigned short W5XXX_readSnTX_FSR(SOCKET _s) {
    unsigned short res;
    res = W5XXX_readSn_Single(_s, 0x0420);
    res = (res << 8) + W5XXX_readSn_Single(_s, 0x0420 + 1);
    return res;
}

//Sn_TX_RD0 and 1 (TxReadPointer) $ 0x0422~0x0423+(0x100 x n)
static void W5XXX_writeSnTX_RD(SOCKET _s, unsigned short _data) {
	W5XXX_writeSn_Single(_s, 0x0422,   _data >> 8);
	W5XXX_writeSn_Single(_s, 0x0422 + 1, _data & 0xFF);
}

static unsigned short W5XXX_readSnTX_RD(SOCKET _s) {
    unsigned short res;
    res = W5XXX_readSn_Single(_s, 0x0422);
    res = (res << 8) + W5XXX_readSn_Single(_s, 0x0422 + 1);
    //printf("readSnTRX_RD() rd pointer = 0x%04x\r\n", res);
    return res;
}

//Sn_TX_WR0 and 1 (TxWritePointer) $ 0x0424~0x0425+(0x100 x n)
static void W5XXX_writeSnTX_WR(SOCKET _s, unsigned short _data) {
	W5XXX_writeSn_Single(_s, 0x0424,   _data >> 8);
	W5XXX_writeSn_Single(_s, 0x0424 + 1, _data & 0xFF);
}
//Socket n TX Write Pointer Register
// indicates the last stored data address in Tx Buffer Block.
// On TCP mode, this should be set as tx data size.?????
static unsigned short W5XXX_readSnTX_WR(SOCKET _s) {
    unsigned short res = 0x0;
    res = W5XXX_readSn_Single(_s, 0x0424);
    res = (res << 8) + W5XXX_readSn_Single(_s, 0x0424 + 1);
    return res;
}

//Sn_RX_RSR0 and 1 (RxReceiveSize) $ 0x0426~0x0427+(0x100 x n)
static void W5XXX_writeSnRX_RSR(SOCKET _s, unsigned short _data) {
	W5XXX_writeSn_Single(_s, 0x0426,   _data >> 8);
	W5XXX_writeSn_Single(_s, 0x0426 + 1, _data & 0xFF);
}

static unsigned short W5XXX_readSnRX_RSR(SOCKET _s) {
    unsigned short res = 0x0;
    res = W5XXX_readSn_Single(_s, 0x0426);
    res = (res << 8) + W5XXX_readSn_Single(_s, 0x0426 + 1);
    return res;
}

//Sn_RX_RD0 and 1 (RxReadPointer) $ 0x0428~0x0429+(0x100 x n)
static void W5XXX_writeSnRX_RD(SOCKET _s, unsigned short _data) {
	W5XXX_writeSn_Single(_s, 0x0428,   _data >> 8);
	W5XXX_writeSn_Single(_s, 0x0428 + 1, _data & 0xFF);
}
//rx read pointer register. The last data address read by host.
static unsigned short W5XXX_readSnRX_RD(SOCKET _s) {
    unsigned short res;
    res = W5XXX_readSn_Single(_s, 0x0428);
    res = (res << 8) + W5XXX_readSn_Single(_s, 0x0428 + 1);
    return res;
}

//Sn_RX_WR0 and 1 (RxWritePointer) $ 0x042A~0x042B+(0x100 x n)
static void W5XXX_writeSnRX_WR(SOCKET _s, unsigned short _data) {
	W5XXX_writeSn_Single(_s, 0x042A,   _data >> 8);
	W5XXX_writeSn_Single(_s, 0x042A + 1, _data & 0xFF);
}

static unsigned short W5XXX_readSnRX_WR(SOCKET _s) {
    unsigned short res;
    res = W5XXX_readSn_Single(_s, 0x042A);
    res = (res << 8) + W5XXX_readSn_Single(_s, 0x042A + 1);
    return res;
}

unsigned char getSn_TXMEM_SIZE(SOCKET s){
	unsigned char tmsr;
	tmsr = W5XXX_spi_read_Single(0x001B, DUMMY);
	return (tmsr & (0x03 << (2*s))) >> (2*s);
}
unsigned char getSn_RXMEM_SIZE(SOCKET s){
	unsigned char rmsr;
	rmsr = W5XXX_spi_read_Single(0x001A, DUMMY); //Rx Memory Size Register
	return (rmsr & (0x03 << (2*s))) >> (2*s);
}

//#define getSn_RXMEM_SIZE(s) (Read_RMSR & (0x03 << (2*s))) >> (2*s)
//get the max RX buffer size of socket s
#define getSn_RxMAX(s) ((unsigned short) (0x0001 << getSn_RXMEM_SIZE(s)) << 10)
#define getSn_RxMASK(s) (getSn_RxMAX(s)-1)

//TMSR=Transmit Memory Size Reg.
//#define getSn_TXMEM_SIZE(s) (getSn_TXMEM_SIZE(s) & (0x03 << (2*s))) >> (2*s)
#define getSn_TxMAX(s) ((unsigned short) (0x0001 << getSn_TXMEM_SIZE(s)) << 10)
#define getSn_TxMASK(s) (getSn_TxMAX(s)-1)

static unsigned short getSn_SBASE(unsigned char socknum) {
	return (W5100_TXBUF_BASE + socknum * W5XXX_SSIZE); //only for w5100s
}
static unsigned short getSn_RBASE(uint8_t socknum) {
    return (W5100_RXBUF_BASE + socknum * W5XXX_SSIZE);//only for w5100s
}


#elif (USE_W5XXX == USE_W5500)

  //================== macros =================================================
  //(cb for W5500) = (0b00000-0-01)= pageSelect[7:3]-R(0)/W(1)-length[1:0](01=1byte)
  unsigned char W5XXX_readSn_Single(SOCKET _s, unsigned short _addr) {
      unsigned char cntl_byte = (_s<<5)+0x08; // -1-0-00
      return W5XXX_spi_read_Single(_addr, cntl_byte);
  }

  unsigned char W5XXX_writeSn_Single(SOCKET _s, unsigned short _addr, unsigned char _data) {
      unsigned char cntl_byte = (_s<<5)+0x0C; // -1-1-00
      return W5XXX_spi_write_Single(_addr, cntl_byte, _data);
}

  unsigned short W5XXX_readSn_Multi(SOCKET _s, unsigned short _addr, unsigned char *_buf, unsigned short _len) {
      unsigned char cntl_byte = (_s<<5)+0x08;
      return W5XXX_spi_read_Multi(_addr, cntl_byte, _buf, _len );
  }

  unsigned short W5XXX_writeSn_Multi(SOCKET _s, unsigned short _addr, unsigned char *_buf, unsigned short _len) {
      unsigned char cntl_byte = (_s<<5)+0x0C;
      return W5XXX_spi_write_Multi(_addr, cntl_byte, _buf, _len);
  }

#define __GP_REGISTER8(name, address)             \
  static inline void W5XXX_write##name(unsigned char _data) { \
	W5XXX_spi_write_Single(address, 0x04, _data);                  \
  }                                               \
  static inline unsigned char read##name() {            \
    return W5XXX_spi_read_Single(address, 0x00);                   \
  }
#define __GP_REGISTER16(name, address)            \
  static void W5XXX_write##name(unsigned short _data) {       \
	W5XXX_spi_write_Single(address,  0x04, _data >> 8);            \
	W5XXX_spi_write_Single(address+1, 0x04, _data & 0xFF);         \
  }                                               \
  static unsigned short W5XXX_read##name() {                  \
    unsigned short res = W5XXX_spi_read_Single(address, 0x00);           \
    res = (res << 8) + W5XXX_spi_read_Single(address + 1, 0x00);   \
    return res;                                   \
  }
#define __GP_REGISTER_N(name, address, size)      \
  static unsigned short W5XXX_write##name(unsigned char *_buff) {   \
    return W5XXX_spi_write_Multi(address, 0x04, _buff, size);     \
  }                                               \
  static unsigned short W5XXX_read##name(unsigned char *_buff) {    \
    return W5XXX_spi_read_Multi(address, 0x00, _buff, size);      \
  }


  __GP_REGISTER8 (MR,     0x0000);    // Mode Register
  __GP_REGISTER_N(GAR,    0x0001, 4); // Gateway IP address
  __GP_REGISTER_N(SUBR,   0x0005, 4); // Subnet mask address
  __GP_REGISTER_N(SHAR,   0x0009, 6); // Source MAC address
  __GP_REGISTER_N(SIPR,   0x000F, 4); // Source IP address
  __GP_REGISTER8 (IR,     0x0015);    // Interrupt
  __GP_REGISTER8 (IMR,    0x0016);    // Interrupt Mask
  __GP_REGISTER16(RTR,    0x0019);    // RetxTimeout address. 100usec unit.
  __GP_REGISTER8 (RCR,    0x001B);    // Retry count. On reaching this count, Timeout interrupt will be issued.
  __GP_REGISTER_N(UIPR,   0x0028, 4); // Unreachable IP address in UDP mode
  __GP_REGISTER16(UPORT,  0x002C);    // Unreachable Port address in UDP mode
  __GP_REGISTER8 (PHYCFGR,     0x002E);    // PHY Configuration register, default value: 0b 1011 1xxx


#undef __GP_REGISTER8
#undef __GP_REGISTER16
#undef __GP_REGISTER_N

// W5500 Socket registers

#define __SOCKET_REGISTER8(name, address)                    \
  static inline void W5XXX_write##name(SOCKET _s, unsigned char _data) { \
	W5XXX_writeSn_Single(_s, address, _data);                             \
  }                                                          \
  static inline unsigned char W5XXX_read##name(SOCKET _s) {              \
    return W5XXX_readSn_Single(_s, address);                              \
  }
#if defined(REL_GR_KURUMI) || defined(REL_GR_KURUMI_PROTOTYPE)
#define __SOCKET_REGISTER16(name, address)                   \
  static void write##name(SOCKET _s, unsigned short _data) {       \
    writeSn(_s, address,   _data >> 8);                      \
    writeSn(_s, address+1, _data & 0xFF);                    \
  }                                                          \
  static unsigned short read##name(SOCKET _s) {                    \
    unsigned short res = readSn(_s, address);                      \
    unsigned short res2 = readSn(_s,address + 1);                  \
    res = res << 8;                                          \
    res2 = res2 & 0xFF;                                      \
    res = res | res2;                                        \
    return res;                                              \
  }
#else
#define __SOCKET_REGISTER16(name, address)                   \
  static void W5XXX_write##name(SOCKET _s, unsigned short _data) {       \
	W5XXX_writeSn_Single(_s, address,   _data >> 8);                      \
	W5XXX_writeSn_Single(_s, address+1, _data & 0xFF);                    \
  }                                                          \
  static unsigned short W5XXX_read##name(SOCKET _s) {                    \
    unsigned short res = W5XXX_readSn_Single(_s, address);                      \
    res = (res << 8) + W5XXX_readSn_Single(_s, address + 1);              \
    return res;                                              \
  }
#endif
#define __SOCKET_REGISTER_N(name, address, size)             \
  static unsigned short W5XXX_write##name(SOCKET _s, unsigned char *_buff) {   \
    return W5XXX_writeSn_Multi(_s, address, _buff, size);                \
  }                                                          \
  static unsigned short W5XXX_read##name(SOCKET _s, unsigned char *_buff) {    \
    return W5XXX_readSn_Multi(_s, address, _buff, size);                 \
  }

  __SOCKET_REGISTER8(SnMR,        0x0000)        // Mode
  __SOCKET_REGISTER8(SnCR,        0x0001)        // Command
  __SOCKET_REGISTER8(SnIR,        0x0002)        // Interrupt
  __SOCKET_REGISTER8(SnSR,        0x0003)        // Status
  __SOCKET_REGISTER16(SnPORT,     0x0004)        // Source Port
  __SOCKET_REGISTER_N(SnDHAR,     0x0006, 6)     // Destination Hardw Addr
  __SOCKET_REGISTER_N(SnDIPR,     0x000C, 4)     // Destination IP Addr
  __SOCKET_REGISTER16(SnDPORT,    0x0010)        // Destination Port
  __SOCKET_REGISTER16(SnMSSR,     0x0012)        // Max Segment Size
  __SOCKET_REGISTER8(SnPROTO,     0x0014)        // Protocol in IP RAW Mode
  __SOCKET_REGISTER8(SnTOS,       0x0015)        // IP TOS
  __SOCKET_REGISTER8(SnTTL,       0x0016)        // IP TTL
  __SOCKET_REGISTER16(SnTX_FSR,   0x0020)        // TX Free Size
  __SOCKET_REGISTER16(SnTX_RD,    0x0022)        // TX Read Pointer
  __SOCKET_REGISTER16(SnTX_WR,    0x0024)        // TX Write Pointer
  __SOCKET_REGISTER16(SnRX_RSR,   0x0026)        // RX Free Size
  __SOCKET_REGISTER16(SnRX_RD,    0x0028)        // RX Read Pointer
  __SOCKET_REGISTER16(SnRX_WR,    0x002A)        // RX Write Pointer (supported?)

#undef __SOCKET_REGISTER8
#undef __SOCKET_REGISTER16
#undef __SOCKET_REGISTER_N
//==================================================================================


void W5XXX_getGatewayIp(unsigned char *_addr) {
	  W5XXX_readGAR(_addr);
}

void W5XXX_setGatewayIp(unsigned char *_addr) {
	W5XXX_writeGAR(_addr);
}

void W5XXX_getSubnetMask(unsigned char *_addr) {
	W5XXX_readSUBR(_addr);
}

void W5XXX_setSubnetMask(unsigned char *_addr) {
	W5XXX_writeSUBR(_addr);
}

//void W5XXX_getMACAddress(unsigned char *_addr) {	W5XXX_readSHAR(_addr);}
void W5XXX_getMACAddress(unsigned char *_addr) {
    W5XXX_spi_read_Multi(0x0009, 0x00, _addr, 6);
}
//void W5XXX_setMACAddress(unsigned char *_addr) {	W5XXX_writeSHAR(_addr);}
void W5XXX_setMACAddress(unsigned char *_addr){
	return W5XXX_spi_write_Multi(0x0009, 0x04, _addr, 6);
}


void W5XXX_getIPAddress(unsigned char *_addr) {
	W5XXX_readSIPR(_addr);
}

void W5XXX_setIPAddress(unsigned char *_addr) {
	W5XXX_writeSIPR(_addr);
}

void W5XXX_setRetransmissionTime(unsigned short _timeout) {
	W5XXX_writeRTR(_timeout);
}

void W5XXX_setRetransmissionCount(unsigned char _retry) {
	W5XXX_writeRCR(_retry);
}

void W5XXX_setPHYCFGR(unsigned char _val) {
	W5XXX_writePHYCFGR(_val);
}

unsigned char W5XXX_getPHYCFGR() {
//  readPHYCFGR();
  return W5XXX_read(0x002E, 0x00);
}


static unsigned short getSn_SBASE(unsigned char socknum) {
    return socknum * W5XXX_SSIZE + 0x8000;
}
static unsigned short getSn_RBASE(unsigned char socknum) {
    return socknum * W5XXX_SSIZE + 0xC000;
}


#endif


static unsigned short g_local_port = 49152;

typedef struct {
	unsigned short RX_RSR; // Number of bytes received
	unsigned short RX_RD;  // Address to read
	unsigned short TX_FSR; // Free space ready for transmit
	unsigned short RX_inc; // how much have we advanced RX_RD
} socketstate_t;

//static socketstate_t state_sock[MAX_SOCK_NUM];


struct  _W5XXX_Client{
	  unsigned short _srcport;
	  unsigned char _sockfd;
};

struct _W5XXX_Server {
  unsigned short _port;
};

struct _W5XXXClass {
	  unsigned char myMac[6];
	  IPAddress myIP;
	  IPAddress mySubnetMask;
	  IPAddress myDefaultGW;
	  IPAddress dnsServerAddress;
	  IPAddress serverIP;

	  socketstate_t 			state_sock[MAX_SOCK_NUM]; //unsigned 	char 			state[MAX_SOCK_NUM]; 	//socket state
	  unsigned 	short 			server_port[MAX_SOCK_NUM];
	  struct  	_W5XXX_Client 	client[MAX_SOCK_NUM]; 	// friend class W5500_Client;
	  struct 	_W5XXX_Server 	server;	  				// friend class EthernetServer;
};

static struct _W5XXXClass W5XXX_Manager;

#if defined(WIZ550io_WITH_MACADDRESS)
  // Initialize function when use the ioShield series (included WIZ550io)
  // WIZ550io has a MAC address which is written after reset.
  // Default IP, Gateway and subnet address are also writen.
  // so, It needs some initial time. please refer WIZ550io Datasheet in details.
  //void begin(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);
#else



#endif

unsigned char W5XXX_socketStatus(SOCKET sockfd);

//======================================================

#if(USE_W5XXX == USE_W5100S)
int W5XXX_GetLinkStatus(void){
	unsigned char retb;

	retb = W5XXX_spi_read_Single(W5XXX_PHYSR0, DUMMY); // 1 byte read (PHY Status : PHYSR0@0x003C)
	if(retb & 0x01){
		printf("Link Up \r\n");
		return 1;
	}else{
		printf("Link Down \r\n");
		return 0;
	}
}

int W5XXX_ResetAndConfigBuf(void)
{
	int i;
	unsigned char retb;

	delayms(100);
    // Software reset the W5500 chip
	printf("Soft Reset\r\n");
    W5XXX_spi_write_Single(
    		0x0000, //MR reg @ 0x0000.
    		DUMMY, //control byte (DUMMY for W5100S)
    		0x80);//data
    delayms(1000);

#if MAX_SOCK_NUM <= 1
    W5XXX_SSIZE = 8192; //Max Tx buffer size
	W5XXX_spi_write_Single( W5100_TMSR, DUMMY, 0x03); //0000-00-11
	W5XXX_spi_write_Single( W5100_RMSR, DUMMY, 0x03);
#elif MAX_SOCK_NUM <= 2
    W5XXX_SSIZE = 4096;
	W5XXX_spi_write_Single( W5100_TMSR, DUMMY, 0x0A); //0000-10-10
	W5XXX_spi_write_Single( W5100_RMSR, DUMMY, 0x0A);
#else //MAX_SOCK_NUM <= 4 (default)
    W5XXX_SSIZE = 2048;
	W5XXX_spi_write_Single( W5100_TMSR, DUMMY, 0x55); //01-01-01-01
	W5XXX_spi_write_Single( W5100_RMSR, DUMMY, 0x55);
#endif
    W5XXX_SMASK = W5XXX_SSIZE - 1;


    //TEST ONYL : read config register for check
    //for(i=0; i<0xff; i++){
    //	retb = W5XXX_spi_read_Single(i, DUMMY); 					// first 1 byte read
    //	printf("Reg0x%04x = 0x%02x\r\n", i, retb);
    //	delayms(50);
    //}

    return W5XXX_GetLinkStatus();
}
#elif (USE_W5XXX == USE_W5500)

int W5XXX_GetLinkStatus(void){
	unsigned short retb;

	retb = W5XXX_spi_read_Single(W5XXX_PHYSR0, 0x01); // 1 byte read
	if(retb & 0x01){
		printf("Link Up \r\n");
		return 1;
	}else{
		printf("Link Down \r\n");
		return 0;
	}
}

int W5XXX_ResetAndConfigBuf(void)
{
	int i;
	unsigned short rets;

	delayms(1000);

    // Software reset the W5500 chip
    W5XXX_spi_write_Single(
    		0x00, //reg addr
    		0x05, //control byte (0b0000-0101)= pageSelect[7:3]-R(0)/W(1)-length[1:0](01=1byte)
    		0x80);//data
    delayms(100);


    //#ifdef ETHERNET_LARGE_BUFFERS
#if MAX_SOCK_NUM <= 1
     W5XXX_SSIZE = 16384; //Max Tx buffer size
#elif MAX_SOCK_NUM <= 2
     W5XXX_SSIZE = 8192;
#elif MAX_SOCK_NUM <= 4
     W5XXX_SSIZE = 4096;
#else
     W5XXX_SSIZE = 2048;
#endif
     W5XXX_SMASK = W5XXX_SSIZE - 1;

     //set rx and tx buffer size
    for (i=0; i<MAX_SOCK_NUM; i++) {
    	unsigned char cntl_byte = (0x0C + (i<<5));
    	W5XXX_spi_write_Single( 0x1E, cntl_byte, W5XXX_SSIZE >> 10);//RXBUF_SIZE); //0x1E - Sn_RXBUF_SIZE
    	W5XXX_spi_write_Single( 0x1F, cntl_byte, W5XXX_SSIZE >> 10);//TXBUF_SIZE); //0x1F - Sn_TXBUF_SIZE
    }
    for (; i<8; i++) {
    	unsigned char cntl_byte = (0x0C + (i<<5));
    	W5XXX_spi_write_Single( 0x1E, cntl_byte, 0); //fill 0 for remainings
    	W5XXX_spi_write_Single( 0x1F, cntl_byte, 0); //fill 0 for remainings
    }
/*
    //read config register for check
    for(i=0; i<0xff; i+=2){
    	rets = W5XXX_spi_read_Single(i, 0x01); 					// first 1 byte read
    	rets = rets << 8 + W5XXX_spi_read_Single(i+1, 0x01); 	// second 1 byte read
    	printf("Reg0x%02x = 0x%04x\r\n", i, rets);
    	delayms(50);
    }
*/
    return W5XXX_GetLinkStatus();
}
#endif

unsigned short W5XXX_getSnTXFreeSize(SOCKET s)
{
    unsigned short val=0, val1=0;
    do {
        val1 = W5XXX_readSnTX_FSR(s);
        if (val1 != 0)
            val = W5XXX_readSnTX_FSR(s);
    }
    while (val != val1);
    return val;
}

unsigned short W5XXX_getRXReceivedSize(SOCKET s)
{
    unsigned short val=0,val1=0;
    do {
        val1 = W5XXX_readSnRX_RSR(s);
        if (val1 != 0)
            val = W5XXX_readSnRX_RSR(s);
    }
    while (val != val1);
    return val;
}

//======================= socket ==================================================================
#if (USE_W5XXX == USE_W5100S)

void W5XXX_write_data_with_offset_for_send(SOCKET s,unsigned short data_offset, const unsigned char *data, unsigned short len)
{
    unsigned short ptr, size, dst_mask, dst_ptr;

    ptr = W5XXX_readSnTX_WR(s);
	ptr += data_offset;
	unsigned short offset = ptr & W5XXX_SMASK;
	unsigned short dstAddr = offset + getSn_SBASE(s);

	if (W5XXX_hasOffsetAddressMapping || offset + len <= W5XXX_SSIZE) {
		W5XXX_spi_write_Multi(dstAddr, DUMMY, data, len);
	} else {
		// Wrap around circular buffer
		unsigned short size = W5XXX_SSIZE - offset;
		W5XXX_spi_write_Multi(dstAddr, DUMMY, data, size);
		W5XXX_spi_write_Multi(getSn_SBASE(s), DUMMY, data + size, len - size);
	}
	ptr += len;
	W5XXX_writeSnTX_WR(s, ptr);

    /*

    dst_mask = (unsigned)ptr & getSn_TxMASK(s);
    dst_ptr = (getSn_TxBase(s) + dst_mask);

    if((dst_mask + len) > getSn_TxMAX(s)){
    	size = getSn_TxMAX(s) - dst_mask;
    	W5XXX_spi_write_Multi((unsigned short)dst_ptr , DUMMY, (unsigned char *)data, size);
    	data += size;
    	size = len - size;
    	dst_ptr = getSn_TxBase(s);
    	W5XXX_spi_read_Multi((unsigned short)dst_ptr , DUMMY, (unsigned char *)data, size);
    }
    else{
    	W5XXX_spi_write_Multi((unsigned short)dst_ptr , DUMMY, (unsigned char *)data, len);
    }
    ptr += len;

    W5XXX_writeSnTX_WR(s, ptr);
    */

}

void W5XXX_read_data(SOCKET s,
		volatile unsigned short src,
		volatile unsigned char *dst,
		unsigned short len)
{
	unsigned short size, src_mask, src_ptr;

	printf("read_data, len=%d, at:%d\n", len, src);
	src_mask = (unsigned short)src & W5XXX_SMASK;
	src_ptr = getSn_RBASE(s) + src_mask;

	if (W5XXX_hasOffsetAddressMapping || src_mask + len <= W5XXX_SSIZE) {
		W5XXX_spi_read_Multi(src_ptr , DUMMY, dst, len);
	}
	else{ //W5100S
		size = W5XXX_SSIZE - src_mask;
		W5XXX_spi_read_Multi(src_ptr , DUMMY, dst, size);
		dst += size;
		W5XXX_spi_read_Multi(src_ptr , DUMMY, dst, len - size);
	}
}



void W5XXX_execCmdSn(SOCKET s, SockCMD _cmd) {
    // Send command to W5500_socket
	W5XXX_writeSnCR(s, _cmd);
    // Wait for command to complete
    while (W5XXX_readSnCR(s))
    ;
}
#elif (USE_W5XXX == USE_W5500)
void W5XXX_write_data_with_offset_for_send(SOCKET s, unsigned short data_offset, const unsigned char *data, unsigned short len)
{

    unsigned short ptr = W5XXX_readSnTX_WR(s);
    unsigned char cntl_byte = (0x14+(s<<5));
    ptr += data_offset;
    W5XXX_spi_write_Multi(ptr, cntl_byte, data, len);
    ptr += len;
    W5XXX_writeSnTX_WR(s, ptr);

}
void W5XXX_read_data(SOCKET s, volatile unsigned short src, volatile unsigned char *dst, unsigned short len)
{
    unsigned char cntl_byte = (0x18+(s<<5));
    W5XXX_spi_read_Multi((unsigned short)src , cntl_byte, (unsigned char *)dst, len);
}
void W5XXX_write_data_for_send(SOCKET s, const unsigned char *data, unsigned short len)
{
  // This is same as having no offset in a call to W5XXX_write_data_with_offset_for_send
  W5XXX_write_data_with_offset_for_send(s, 0, data, len);

}

void W5XXX_recv_data_processing(SOCKET s,
		unsigned char *data, //to be moved into.
		unsigned short len,
		unsigned char peek)
{
    unsigned short ptr;
    ptr = W5XXX_readSnRX_RD(s);

    W5XXX_read_data(s,
    		ptr, //from
    		data,//to
    		len);
    if (!peek){
        ptr += len;
        W5XXX_writeSnRX_RD(s, ptr);
    }
}

void W5XXX_execCmdSn(SOCKET s, SockCMD _cmd) {
    // Send command to W5500_socket
	W5XXX_writeSnCR(s, _cmd);
    // Wait for command to complete
    while (W5XXX_readSnCR(s))
    ;
}
#endif

//============== W5XXX_socket ====================
//extern unsigned char W5XXX_socket_Begin(SOCKET s, unsigned char protocol, unsigned short port, unsigned char flag); // Opens a W5500_socket(TCP or UDP or IP_RAW mode)
unsigned char W5XXX_socket_Begin(unsigned char protocol, unsigned short port, unsigned char flag); // Opens a W5500_socket(TCP or UDP or IP_RAW mode)
extern void W5XXX_socket_close(SOCKET s); // Close W5500_socket
extern unsigned char W5XXX_connect(SOCKET s, unsigned char * addr, unsigned short port); // Establish TCP W5500_connection (Active W5500_connection)
extern void W5XXX_socket_disconnect(SOCKET s); // W5500_socket_disconnect the W5500_connection
extern unsigned char W5XXX_listen(SOCKET s);	// Establish TCP W5500_connection (Passive W5500_connection)
unsigned short W5XXX_socket_send(SOCKET s, unsigned char * buf, unsigned short len); // Send data (TCP)
extern int16_t W5XXX_recv(SOCKET s, unsigned char * buf, int16_t len);	// Receive data (TCP)
extern unsigned short peek(SOCKET s, unsigned char *buf);
unsigned short W5XXX_socket_sendto(SOCKET s, const unsigned char * buf, unsigned short len, unsigned char * addr, unsigned short port); // Send data (UDP/IP RAW)
extern unsigned short W5XXX_recvfrom(SOCKET s, unsigned char * buf, unsigned short len, unsigned char * addr, unsigned short *port); // Receive data (UDP/IP RAW)
extern void flush(SOCKET s); // Wait for transmission to complete
unsigned short igmpW5XXX_socket_send(SOCKET s, const unsigned char * buf, unsigned short len);
extern int W5XXX_startUDP(SOCKET s, unsigned char* addr, unsigned short port);
unsigned short W5XXX_bufferingDataForSend(SOCKET s, unsigned short offset, const unsigned char* buf, unsigned short len);
int W5XXX_socket_sendUDP(SOCKET s);


void W5XXX_socket_PortRand(unsigned short n)
{
	n &= 0x3FFF;
	g_local_port ^= n;
}

// @return 	sockfd
 unsigned char W5XXX_socket_Begin(unsigned char protocol, unsigned short port, unsigned char flag)
{
	unsigned char s=0;
	unsigned char status[MAX_SOCK_NUM], chip, maxindex=MAX_SOCK_NUM;
#if	(USE_W5XXX == USE_W5100S)
	chip = USE_W5100S;
#elif (USE_W5XXX == USE_W5500)
	chip = USE_W5500;
#endif

#if MAX_SOCK_NUM > 4
	if (chip == USE_W5100S) maxindex = 4; // W5100S chip never supports more than 4 sockets
#endif
	printf("W5XXX socket begin(): sock=%u, protocol=%d, port=%d\r\n", s, protocol, port);

	// look at all the sockets, use any that are closed (unused)
	for (s=0; s < MAX_SOCK_NUM; s++) {
		status[s] = W5XXX_readSnSR(s);
		if (status[s] == SnSR_CLOSED)
			goto makesocket;
	}

	// as a last resort, forcibly close any already closing
	for (s=0; s < MAX_SOCK_NUM; s++) {
		unsigned char stat = status[s];
		if (stat == SnSR_LAST_ACK) 	goto closemakesocket;
		if (stat == SnSR_TIME_WAIT) goto closemakesocket;
		if (stat == SnSR_FIN_WAIT) 	goto closemakesocket;
		if (stat == SnSR_CLOSING) 	goto closemakesocket;
	}
#if 0
	// next, use any that are effectively closed
	for (s=0; s < MAX_SOCK_NUM; s++) {
		uint8_t stat = status[s];
		// TODO: this also needs to check if no more data
		if (stat == SnSR_CLOSE_WAIT) goto closemakesocket;
	}
#endif
	//SPI.endTransaction();
	return MAX_SOCK_NUM; // Unfortunately, all sockets are in use


closemakesocket:
    printf("On creating socket, we first close and then make socket in sequence\r\n");
    W5XXX_execCmdSn(s, Sock_CLOSE); //W5XXX_socket_close(s);

makesocket:
	printf("W5XXX_socket_Begin(): makesocket with sockfd of %u (port = %u)\r\n",s, port);
	W5XXX_Manager.server_port[s] = 0; //why not port?
	delayms(1);//Needed?

	W5XXX_writeSnMR(s, protocol); //W5XXX_writeSnMR(s, protocol | flag);
	W5XXX_writeSnIR(s, 0xFF);
    if (port > 0) {
      W5XXX_writeSnPORT(s, port);
    }
    else {
		// if don't set the source port, set local_port number.
		if (++g_local_port < 49152) g_local_port = 49152;
		 W5XXX_writeSnPORT(s, g_local_port);
    }

    W5XXX_execCmdSn(s, Sock_OPEN);
	printf("W5XXX_socket_Begin(): Sock_OPEN with sockfd of %u (port = %u)\r\n",s, port);

	W5XXX_Manager.state_sock[s].RX_RSR = 0;
	W5XXX_Manager.state_sock[s].RX_RD  = W5XXX_readSnRX_RD(s); // always zero?
	W5XXX_Manager.state_sock[s].RX_inc = 0;
	W5XXX_Manager.state_sock[s].TX_FSR = 0;
	//printf("W5XXXsocket prot=%d, RX_RD=%d\n", W5XXX_readSnMR(s), W5XXX_Manager.state_sock[s].RX_RD);
	return s; //return sockfd.
}



/**
 * @brief	This function used for W5500_socket_disconnect the W5500_socket and parameter is "s" which represent the W5500_socket number
 * @return	1 for success else 0.
 */
void W5XXX_socket_disconnect(SOCKET s) //gracefully
{
	printf("disconnecting\r\n");
	W5XXX_execCmdSn(s, Sock_DISCON);
	printf("disconnected\r\n");
}


/**
 * @brief	This function close the W5500_socket and parameter is "s" which represent the W5500_socket number
 */
void W5XXX_socket_close(SOCKET s) //forcefully
{
  W5XXX_execCmdSn(s, Sock_CLOSE);
  W5XXX_writeSnIR(s, 0xFF);

  printf("close\r\n");
}
//========== Client
/**
 * @brief	This function established  the W5500_connection for the channel in Active (client) mode.
 * 		This function waits for the until the W5500_connection is established.
 *
 * @return	1 for success else 0.
 */
unsigned char W5500_socket_connect(SOCKET s, unsigned char * addr, unsigned short port)
{
  if
    (
  ((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF) && (addr[3] == 0xFF)) ||
    ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
    (port == 0x00)
    )
    return 0;

  // set destination IP
  W5XXX_writeSnDIPR(s, addr);
  W5XXX_writeSnDPORT(s, port);
  W5XXX_execCmdSn(s, Sock_CONNECT);

  return 1;
}

/*
int W5500_Client_connect_byIP(IPAddress ip, unsigned short port) {
	int i;
  if (W5XXX_Manager.client._sockfd != MAX_SOCK_NUM)
    return 0;

  for (i = 0; i < MAX_SOCK_NUM; i++) {
    unsigned char s = W5500_readSnSR(i);
    if (s == SnSR_CLOSED || s == SnSR_FIN_WAIT || s == SnSR_CLOSE_WAIT) {
    	W5XXX_Manager.client._sockfd = i;
      break;
    }
  }

  if (W5XXX_UDP._sockfd == MAX_SOCK_NUM)
    return 0;

  W5XXX_Manager.client._srcport++;
  if (W5XXX_Manager.client._srcport == 0) W5XXX_Manager.client._srcport = 1024;
  W5XXX_Manager.client._sockfd = W5XXX_socket_Begin(SnMR_TCP, W5XXX_Manager.client._srcport, 0);

  if (!W5XXX_connect(W5XXX_Manager.client._sockfd, rawIPAddress(ip), port)) {
	  W5XXX_Manager.client._sockfd = MAX_SOCK_NUM;
    return 0;
  }

  while (W5XXX_socketStatus() != SnSR_ESTABLISHED) {
    delay(1);
    if (W5XXX_socketStatus() == SnSR_CLOSED) {
    	W5XXX_Manager.client._sockfd = MAX_SOCK_NUM;
      return 0;
    }
  }

  return 1;
}

size_t W5500_Client_write(unsigned char b) {
  return write(&b, 1);
}

size_t W5XXX_Client_write(const unsigned char *buf, size_t size) {
  if (W5XXX_Manager.client._sockfd == MAX_SOCK_NUM) {
    setWriteError();
    return 0;
  }
  if (!W5XXX_socket_send(W5XXX_Manager.client._sockfd, buf, size)) {
    setWriteError();
    return 0;
  }
  return size;
}


int W5XXX_Client_read_byte() {
  unsigned char b;
  if ( W5XXX_socket_recv(W5XXX_Manager.client._sockfd, &b, 1) > 0 )
  {
    // recv worked
    return b;
  }
  else
  {
    // No data available
    return -1;
  }
}

int W5XXX_Client_read(unsigned char *buf, size_t size) {
  return W5XXX_socket_recv(W5XXX_Manager.client._sockfd,
		  buf,
		  size);
}

int W5500_Client_peek() {
  unsigned char b;
  // Unlike recv, peek doesn't check to see if there's any data available, so we must
  if (!W5XXX_socketRecvAvailable())
    return -1;
  b = W5XXX_socket_peek(W5XXX_Manager.client._sockfd);
  return b;
}

void W5XXX_Client_flush() {
  W5XXX_flush(W5XXX_Manager.client._sockfd);
}
*/
int W5XXX_socketRecvAvailable(SOCKET sockfd) {
  if (W5XXX_Manager.client[sockfd]._sockfd != MAX_SOCK_NUM)
    return W5XXX_getRXReceivedSize(sockfd);
  return 0;
}

unsigned char W5XXX_socketStatus(SOCKET sockfd) {
	unsigned char ClientStatus;
	if (W5XXX_Manager.client[sockfd]._sockfd == MAX_SOCK_NUM) return SnSR_CLOSED;

	ClientStatus =W5XXX_readSnSR(sockfd);

	return ClientStatus;
}

void W5XXX_Client_stop(SOCKET sockfd) {
	printf("W5XXX_Client_stop\r\n");

  if (W5XXX_Manager.client[sockfd]._sockfd == MAX_SOCK_NUM){
	  printf("W5XXX_Client_stop ???\r\n");
    return;
  }

  // attempt to close gracefully (W5XXX_socket_send a FIN to other side)
  W5XXX_socket_disconnect(sockfd);
  //unsigned long start = millis();

  // wait a second for the W5500_connection to close
  //while (W5XXX_socketStatus() != SnSR_CLOSED && millis() - start < 1000)
  //     delayms(1);
  delayms(1000);

  // if it hasn't closed, close it forcefully
  if (W5XXX_socketStatus(sockfd) != SnSR_CLOSED){
	  W5XXX_socket_close(sockfd);
  }

  W5XXX_Manager.server_port[sockfd] = 0; //W5XXX_Manager__server_port[W5XXX_Manager.client._sockfd] = 0;
  W5XXX_Manager.client[sockfd]._sockfd = MAX_SOCK_NUM;

  printf("Close Sock.\r\n");
}

unsigned char W5XXX_isClientConnected(SOCKET sockfd) {
  if (W5XXX_Manager.client[sockfd]._sockfd == MAX_SOCK_NUM) return 0;

  unsigned char st = W5XXX_socketStatus(sockfd);
  return !(st == SnSR_LISTEN || st == SnSR_CLOSED || st == SnSR_FIN_WAIT ||
    (st == SnSR_CLOSE_WAIT && ! W5XXX_socketRecvAvailable(sockfd)));
}


//=========== Server ==================
unsigned char W5XXX_socket_listen(SOCKET s) { //in passive mode
  if (W5XXX_readSnSR(s) != SnSR_INIT)
    return 0;

  printf("W5XXX_socket_listen(): with sockfd of %u\r\n",s);
  W5XXX_execCmdSn(s, Sock_LISTEN);
  return 1;
}
//-- NOT USED BUT WHY?
struct  _W5XXX_Client *W5XXX_Server_Accept()//SOCKET s)
{
	int listening = 0;
	unsigned char i;
	unsigned char sockfd = MAX_SOCK_NUM;
	unsigned char tcpState;

	for(i=0; i <MAX_SOCK_NUM;i++){
		if(W5XXX_Manager.server_port[i] == W5XXX_Manager.server._port){
			tcpState = W5XXX_socketStatus(i);

			if(sockfd == MAX_SOCK_NUM &&
				  (tcpState == SnSR_ESTABLISHED || tcpState == SnSR_CLOSE_WAIT)) {
					// Return the connected client even if no data received.
					// Some protocols like FTP expect the server to send the
					// first data.
					sockfd = i;
					W5XXX_Manager.server_port[i] = 0; // only return the client once
			}
			else if(tcpState == SnSR_LISTEN){
				listening = 1;
			}else if(tcpState == SnSR_CLOSED){
				W5XXX_Manager.server_port[i] = 0;
			}
		}
	}
	if(!listening){
		W5XXX_Server_begin(W5XXX_Manager.server_port[sockfd]);
	}

	W5XXX_Manager.client[sockfd]._sockfd = sockfd;
	return &W5XXX_Manager.client[sockfd];//EthernetClient(sockifd);
}

//int
struct  _W5XXX_Client *W5XXX_Server_Available_CheckForIncomingCllients()
{
	int listening = 0;
	int i, sockfd = MAX_SOCK_NUM;
	unsigned char tcpState;
	struct  _W5XXX_Client *client;

	//client = W5XXX_Server_Accept();

	for(i=0; i <MAX_SOCK_NUM;i++){
		if(W5XXX_Manager.server_port[i] == W5XXX_Manager.server._port){
			tcpState = W5XXX_socketStatus(i);
			if((tcpState ==	SnSR_ESTABLISHED) || (tcpState == SnSR_CLOSE_WAIT))
			{
				if(W5XXX_socketRecvAvailable(i) > 0) {
					sockfd = i;
				}else{
					if(tcpState == SnSR_CLOSE_WAIT){
						W5XXX_socket_disconnect(i); //state becomes LAST_ACK for a short duration.
					}
				}
			}else if(tcpState == SnSR_LISTEN){
				listening = 1;
			}else if(tcpState == SnSR_CLOSED){
				W5XXX_Manager.server_port[i] = 0;
			}

		}else{

		}
	}

	//YOON : WHY DO THIS...>>>
	if(listening == 0)
		W5XXX_Server_begin(W5XXX_Manager.server._port);

	if(sockfd < MAX_SOCK_NUM){
		W5XXX_Manager.client[sockfd]._sockfd = sockfd;
		return &W5XXX_Manager.client[sockfd];//EthernetClient(sockfd);
	}else
		return NULL;
}
//=======================

/**
 * @brief	This function used to W5XXX_socket_send the data in TCP mode
 * @return	1 for success else 0.
 */
unsigned short W5XXX_socket_send(SOCKET s, unsigned char * buf, unsigned short len)
{
  unsigned char status=0;
  unsigned short ret=0;
  unsigned short freesize=0;

  printf("W5XXX_socket_send():%s\r\n", buf);

  if (len > W5XXX_SSIZE)
    ret = W5XXX_SSIZE; // check size not to exceed MAX size.
  else
    ret = len;

  // if freebuf is available, start.
  do
  {
    freesize = W5XXX_getSnTXFreeSize(s);
    status = W5XXX_readSnSR(s);
    if ((status != SnSR_ESTABLISHED) && (status != SnSR_CLOSE_WAIT))
    {
      ret = 0;
      break;
    }
  } while (freesize < ret);

  // copy data
  //W5XXX_write_data_with_offset_for_send(s,  0,   (unsigned char *)buf,  ret);
  W5XXX_write_data_with_offset_for_send(s, 0, (unsigned char *)buf,  ret);
  W5XXX_execCmdSn(s, Sock_SEND);

  /* +2008.01 bj */
  while ( (W5XXX_readSnIR(s) & SnIR_SEND_OK) != SnIR_SEND_OK )
  {
    /* m2008.01 [bj] : reduce code */
    if ( W5XXX_readSnSR(s) == SnSR_CLOSED )
    {
    	//W5XXX_socket_close(s);
      return 0;
    }
    delayms(1);
  }
  /* +2008.01 bj */
  W5XXX_writeSnIR(s, SnIR_SEND_OK);
  return ret;
}


//@return	received data size for success else -1.
int W5XXX_socket_recv(SOCKET s,
		unsigned char *data,
		short len)
{
	int ret;
	unsigned short ptr, size, inc;
  // Check how much data is available
	ret = W5XXX_Manager.state_sock[s].RX_RSR; 	//W5XXX_getRXReceivedSize(s);

	if (ret < len) {
		unsigned short rsr = W5XXX_getRXReceivedSize(s); //getSnRX_RSR(s);
		ret = rsr - W5XXX_Manager.state_sock[s].RX_inc;
		W5XXX_Manager.state_sock[s].RX_RSR = ret;
		printf("Sock_RECV, RX_RSR=%d, RX_inc=%d\n", ret, W5XXX_Manager.state_sock[s].RX_inc);
	}

	if ( ret == 0 )  {    // No data available.
		unsigned char status = W5XXX_readSnSR(s);
		if ( status == SnSR_LISTEN || status == SnSR_CLOSED || status == SnSR_CLOSE_WAIT )
		{
			// The remote end has closed its side of the W5500_connection, so this is the eof state
			ret = 0;
		}
		else
		{
			// The W5500_connection is still up, but there's no data waiting to be read
			ret = -1;
		}
	}
	else{
		if (ret > len)	ret = len; // more data available than buffer length
#if 0
		//W5XXX_recv_data_processing(s, buf, ret, 0);
		W5XXX_execCmdSn(s, Sock_RECV);
#else
		ptr = W5XXX_Manager.state_sock[s].RX_RD;

		if (data)
			W5XXX_read_data(s,
				    		ptr, //from
				    		data,//to
				    		len);
		ptr += ret;
		W5XXX_Manager.state_sock[s].RX_RD = ptr;
		W5XXX_Manager.state_sock[s].RX_RSR -= ret;
		inc = W5XXX_Manager.state_sock[s].RX_inc + ret;
		if (inc >= 250 || W5XXX_Manager.state_sock[s].RX_RSR == 0) {
			W5XXX_Manager.state_sock[s].RX_inc = 0;
			W5XXX_writeSnRX_RD(s, ptr);
			W5XXX_execCmdSn(s, Sock_RECV);
			printf("Sock_RECV cmd, RX_RD=%d, RX_RSR=%d\n",	W5XXX_Manager.state_sock[s].RX_RD, W5XXX_Manager.state_sock[s].RX_RSR);
		} else {
			W5XXX_Manager.state_sock[s].RX_inc = inc;
		}
#endif
	}
  return ret;
}



// @brief	Returns the first byte in the receive queue (no checking)
unsigned char W5XXX_socket_peek(SOCKET s)
{
	unsigned char b = 0;
	unsigned short ptr = W5XXX_Manager.state_sock[s].RX_RD;
#if	(USE_W5XXX == USE_W5100S)
	W5XXX_spi_read_Multi((ptr & W5XXX_SMASK) + getSn_RBASE(s), DUMMY, &b, 1);	//W5100.read((ptr & W5100.SMASK) + W5100.RBASE(s), &b, 1);
#else
	//TBD...
#endif
	return b;

}


/**
 * @brief	This function is an application I/F function which is used to W5XXX_socket_send the data for other then TCP mode.
 * 		Unlike TCP transmission, The peer's destination address and the port is needed.
 *
 * @return	This function return W5XXX_socket_send data size for success else -1.
 */
unsigned short W5XXX_socket_sendto(SOCKET s, const unsigned char *buf, unsigned short len, unsigned char *addr, unsigned short port)
{
  unsigned short ret=0;

  if (len > W5XXX_SSIZE) ret = W5XXX_SSIZE; // check size not to exceed MAX size.
  else ret = len;

  if
    (
  ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
    ((port == 0x00)) ||(ret == 0)
    )
  {
    /* +2008.01 [bj] : added return value */
    ret = 0;
  }
  else
  {
    W5XXX_writeSnDIPR(s, addr);
    W5XXX_writeSnDPORT(s, port);

    // copy data
    W5XXX_write_data_with_offset_for_send(s, 0, (unsigned char *)buf, ret);
    W5XXX_execCmdSn(s, Sock_SEND);

    /* +2008.01 bj */
    while ( (W5XXX_readSnIR(s) & SnIR_SEND_OK) != SnIR_SEND_OK )
    {
      if (W5XXX_readSnIR(s) & SnIR_TIMEOUT)
      {
        /* +2008.01 [bj]: clear interrupt */
        W5XXX_writeSnIR(s, (SnIR_SEND_OK | SnIR_TIMEOUT)); /* clear SEND_OK & TIMEOUT */
        return 0;
      }
    }

    /* +2008.01 bj */
    W5XXX_writeSnIR(s, SnIR_SEND_OK);
  }
  return ret;
}


/**
 * @brief	This function is an application I/F function which is used to receive the data in other then
 * 	TCP mode. This function is used to receive UDP, IP_RAW and MAC_RAW mode, and handle the header as well.
 *
 * @return	This function return received data size for success else -1.
 */
unsigned short W5XXX_recvfrom(SOCKET s, unsigned char *buf, unsigned short len, unsigned char *addr, unsigned short *port)
{
  unsigned char head[8];
  unsigned short data_len=0;
  unsigned short ptr=0;

  if ( len > 0 )
  {
    ptr = W5XXX_readSnRX_RD(s);
    switch (W5XXX_readSnMR(s) & 0x07)
    {
    case SnMR_UDP :
      W5XXX_read_data(s, ptr, head, 0x08);
      ptr += 8;
      // read peer's IP address, port number.
      addr[0] = head[0];
      addr[1] = head[1];
      addr[2] = head[2];
      addr[3] = head[3];
      *port = head[4];
      *port = (*port << 8) + head[5];
      data_len = head[6];
      data_len = (data_len << 8) + head[7];

      W5XXX_read_data(s, ptr, buf, data_len); // data copy.
      ptr += data_len;

      W5XXX_writeSnRX_RD(s, ptr);
      break;

    case SnMR_IPRAW :
      W5XXX_read_data(s, ptr, head, 0x06);
      ptr += 6;

      addr[0] = head[0];
      addr[1] = head[1];
      addr[2] = head[2];
      addr[3] = head[3];
      data_len = head[4];
      data_len = (data_len << 8) + head[5];

      W5XXX_read_data(s, ptr, buf, data_len); // data copy.
      ptr += data_len;

      W5XXX_writeSnRX_RD(s, ptr);
      break;

    case SnMR_MACRAW:
      W5XXX_read_data(s, ptr, head, 2);
      ptr+=2;
      data_len = head[0];
      data_len = (data_len<<8) + head[1] - 2;

      W5XXX_read_data(s, ptr, buf, data_len);
      ptr += data_len;
      W5XXX_writeSnRX_RD(s, ptr);
      break;

    default :
      break;
    }
    W5XXX_execCmdSn(s, Sock_RECV);
  }
  return data_len;
}

/**
 * @brief	Wait for buffered transmission to complete.
 */
void W5XXX_flush(SOCKET s) {
  // TODO
}

unsigned short W5500_igmp_send(SOCKET s, const unsigned char * buf, unsigned short len)
{
  unsigned char status=0;
  unsigned short ret=0;

  if (len > W5XXX_SSIZE)
    ret = W5XXX_SSIZE; // check size not to exceed MAX size.
  else
    ret = len;

  if (ret == 0)
    return 0;

  W5XXX_socket_send_data_processing(s, (unsigned char *)buf, ret);
  W5XXX_execCmdSn(s, Sock_SEND);

  while ( (W5XXX_readSnIR(s) & SnIR_SEND_OK) != SnIR_SEND_OK )
  {
    status = W5500_readSnSR(s);
    if (W5XXX_readSnIR(s) & SnIR_TIMEOUT)
    {
      /* in case of igmp, if W5XXX_socket_send fails, then W5500_socket closed */
      /* if you want change, remove this code. */
    	W5XXX_socket_close(s);
      return 0;
    }
  }

  W5XXX_writeSnIR(s, SnIR_SEND_OK);
  return ret;
}

unsigned short W5XXX_socket_bufferDataForSend(SOCKET s, unsigned short offset, const unsigned char* buf, unsigned short len)
{
  unsigned short ret =0;
  unsigned short txfree;
  txfree = W5XXX_getSnTXFreeSize(s);
  if (len > txfree)  {
    ret = txfree; // check size not to exceed MAX size.
  }
  else
  {
    ret = len;
  }
  W5XXX_write_data_with_offset_for_send(s, offset, buf, ret);
  return ret;
}

int W5XXX_startUDP(SOCKET s, unsigned char* addr, unsigned short port)
{
  if
    (
     ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
     ((port == 0x00))
    )
  {
    return 0;
  }
  else
  {
    W5XXX_writeSnDIPR(s, addr);
    W5XXX_writeSnDPORT(s, port);
    return 1;
  }
}

int W5XXX_socket_sendUDP(SOCKET s)
{
  W5XXX_execCmdSn(s, Sock_SEND);

  /* +2008.01 bj */
  while ( (W5XXX_readSnIR(s) & SnIR_SEND_OK) != SnIR_SEND_OK )
  {
    if (W5XXX_readSnIR(s) & SnIR_TIMEOUT)
    {
      /* +2008.01 [bj]: clear interrupt */
      W5XXX_writeSnIR(s, (SnIR_SEND_OK|SnIR_TIMEOUT));
      return 0;
    }
  }

  /* +2008.01 bj */
  W5XXX_writeSnIR(s, SnIR_SEND_OK);

  /* Sent ok */
  return 1;
}
//==========

/*
 *  Udp.cpp: Library to W5XXX_socket_send/receive UDP packets.
 * NOTE: UDP is fast, but has some important limitations (thanks to Warren Gray for mentioning these)
 */

#define UDP_TX_PACKET_MAX_SIZE 24

struct _W5XXX_UDP{
  unsigned char _sockfd;  // W5500_socket ID for Wiz5100
  unsigned short _port; // local port to W5500_listen on
  IPAddress _remoteIP; // remote IP address for the incoming packet whilst it's being processed
  unsigned short _remotePort; // remote port for the incoming packet whilst it's being processed
  unsigned short _offset; // offset into the packet being sent
  unsigned short _remaining; // remaining bytes of incoming packet yet to be processed
};

struct _W5XXX_UDP W5XXX_UDP;

IPAddress localIP();
IPAddress subnetMask();
IPAddress gatewayIP();
IPAddress dnsServerIP();

Init_W5XXX_Client(unsigned char sock);

void W5XXX_Ethernet_begin()
{
	unsigned char retMacStr[6];
	IPAddress retIP;

	while(1){
		if(W5XXX_ResetAndConfigBuf()){
			break; //Link up
		}else{
			delayms(100);
			printf("Connect UTP cable\r\n");
		}
	}
	W5XXX_setMACAddress(W5XXX_Manager.myMac);


/*
  // Now try to get our config info from a DHCP server
  int ret = _dhcp.beginWithDHCP(mac_address);
  if(ret == 1)
  {
    // We've successfully found a DHCP server and got our configuration info, so set things
    // accordingly
    W5XXX_setIPAddress(_dhcp.getLocalIp().raw_address());
    W5XXX_setGatewayIp(_dhcp.getGatewayIp().raw_address());
    W5XXX_setSubnetMask(_dhcp.getSubnetMask().raw_address());
    _dnsServerAddress = _dhcp.getDnsServerIp();
  }
*/
	W5XXX_setIPAddress(&W5XXX_Manager.myIP);
	W5XXX_setGatewayIp(&W5XXX_Manager.myDefaultGW);
	W5XXX_setSubnetMask(&W5XXX_Manager.mySubnetMask);
	//_dnsServerAddress = dns_server;

	//For Check Only
	W5XXX_getMACAddress(retMacStr);
	printf("MAC addr = %02x%02x%02x:%02x%02x%02x\r\n",retMacStr[0],retMacStr[1],retMacStr[2],retMacStr[3],retMacStr[4],retMacStr[5]);
	W5XXX_getIPAddress(&retIP);
	printf("myIP = %u.%u.%u.%u\r\n",retIP.bytes[0],retIP.bytes[1],retIP.bytes[2],retIP.bytes[3]);
	W5XXX_getGatewayIp(&retIP);
	printf("myDefGW = %u.%u.%u.%u\r\n",retIP.bytes[0],retIP.bytes[1],retIP.bytes[2],retIP.bytes[3]);
	W5XXX_getSubnetMask(&retIP);
	printf("mySubnetMask = %u.%u.%u.%u\r\n",retIP.bytes[0],retIP.bytes[1],retIP.bytes[2],retIP.bytes[3]);

}

//================== server ================
void W5XXX_Server_begin(unsigned short listen_port){
	//int sock;
	unsigned char sockfd;

	printf("W5XXX_Server_begin with listen port of %u\r\n", listen_port);

	W5XXX_Manager.server._port = listen_port;

//	for(sock = 0; sock < MAX_SOCK_NUM; sock++){

//		if(W5XXX_socketStatus(sock) == SnSR_CLOSED)
//		{
			sockfd = W5XXX_socket_Begin(SnMR_TCP,listen_port,0);
			if(sockfd < MAX_SOCK_NUM){
				W5XXX_socket_listen(sockfd);
				W5XXX_Manager.server_port[sockfd] = listen_port;
				W5XXX_Manager.client[sockfd]._sockfd = sockfd;
			}else {
				W5XXX_socket_disconnect(sockfd);
			}
//			break;
//		}
//	}
}
int count = 0;
int W5XXX_App_ScanGET(char *rxbuf, int rxlen){
	int i;
	int len = strlen("GET / HTTP/1.1\r\n");

	//printf("Got a string:%s\r\n", rxbuf);
	for(i=0;i<(rxlen - len);i++){
		if(strncmp(rxbuf+i,"GET / HTTP/1.1\r\n",len) == 0)
			return 1;
	}
	return 0;
}

void W5XXX_Server_handleClients(struct  _W5XXX_Client *client){
	char c;
	int sock=0;
	char sendbuf[100];
	int rxlen;
	volatile char rxbuf[512];
	int txlen;

	if(client != NULL){
		printf("Svr> Got a client\r\n");
		while(W5XXX_isClientConnected(client->_sockfd)){
			if(W5XXX_socketRecvAvailable(client->_sockfd)){
				//rxlen = W5XXX_Client_read(rxbuf,1024);
				rxlen = W5XXX_socket_recv(client->_sockfd,
						  rxbuf,
						  512);

				if(rxlen == 0)
					continue;

				printf("Got %u lines from the client.\r\n", rxlen);

				if(W5XXX_App_ScanGET(rxbuf, rxlen)){ //On receiving GET Method

					printf("SVR: Responding..\r\n");
					//send resp
					sprintf(sendbuf,"HTTP/1.1 200 OK\r\n");
					txlen = strlen(sendbuf);
					W5XXX_socket_send(client->_sockfd,sendbuf,txlen);
					W5XXX_socket_send(client->_sockfd,"Content-Type: text/html\r\n",strlen("Content-Type: text/html\r\n"));
					W5XXX_socket_send(client->_sockfd,"\r\n", 2);
#if (USE_W5XXX == USE_W5100S)
					sprintf(sendbuf,">>> HELLO KONG WEB SVR WITH W5100S (%u) <<<\r\n", count++);
#elif (USE_W5XXX == USE_W5500)
					sprintf(sendbuf,">>> HELLO KONG WEB SVR WITH W5500 (%u) <<<\r\n", count++);
#endif
					W5XXX_socket_send(client->_sockfd,sendbuf,strlen(sendbuf));
					break;
				}
			}
			//else	continue;
		}
		delayms(100);// give the web browser time to receive the data
		W5XXX_Client_stop(client->_sockfd); //close this connection.
		printf("client disconnected");

		W5XXX_Server_begin(80); //do HTTP Server again for next client.
	}
}

//========= main =============================
void W5XXX_loop()
{
	struct  _W5XXX_Client *client;
	unsigned char rxbuf[256];
	unsigned char sendbuf[256];
	int rxlen;
	//================== CONFIG ==================================
	// Set a MAC address for your controller below.
	W5XXX_Manager.myMac[0] = 0xAA;		W5XXX_Manager.myMac[1] = 0xBB; W5XXX_Manager.myMac[2] = 0xCC;
	W5XXX_Manager.myMac[3] = 0xDD;		W5XXX_Manager.myMac[4] = 0xEE; W5XXX_Manager.myMac[5] = 0xFF;

	//Set IP
	W5XXX_Manager.myIP.bytes[0] = 192;
	W5XXX_Manager.myIP.bytes[1] = 168;
	W5XXX_Manager.myIP.bytes[2] = 10;
	W5XXX_Manager.myIP.bytes[3] = 3;

	W5XXX_Manager.serverIP.bytes[0] = 192;
	W5XXX_Manager.serverIP.bytes[1] = 168;
	W5XXX_Manager.serverIP.bytes[2] = 10;
	W5XXX_Manager.serverIP.bytes[3] = 2;  // numeric IP for Serasidis.gr (no DNS)

	W5XXX_Manager.mySubnetMask.bytes[0] = 255;
	W5XXX_Manager.mySubnetMask.bytes[1] = 255;
	W5XXX_Manager.mySubnetMask.bytes[2] = 255;
	W5XXX_Manager.mySubnetMask.bytes[3] = 0;

	W5XXX_Manager.myDefaultGW.bytes[0] = 192;
	W5XXX_Manager.myDefaultGW.bytes[1] = 168;
	W5XXX_Manager.myDefaultGW.bytes[2] = 10;
	W5XXX_Manager.myDefaultGW.bytes[3] = 1;

	//IPAddress myDNS = {192,168,10,2};

	//== SPI CONFIG ==
	stmW5XXX_SpiConfig(); //SPI_MODE0, 10Mbps

	//== BEGIN
	W5XXX_Ethernet_begin();//myMac, myIP, myDNS, myDefaultGW, mySubnetMask);
	W5XXX_Server_begin(80); //for HTTP Server.

	delayms(1);

	while(1){
		//YOON....TBD...
		//why two belows are different?
		//client = W5XXX_Server_Accept(); //not working but to be correct.
		client = W5XXX_Server_Available_CheckForIncomingCllients(); //accept and ...

		if(client != NULL)
			W5XXX_Server_handleClients(client);
		else
			continue;

/*
		client = W5XXX_Server_Available_CheckForIncomingCllients(); // listen for incoming clients

		if(client == NULL){
			continue;
		}else{

			while(W5XXX_isClientConnected(client->_sockfd)){ //while (client.connected()) {

				printf("new client\r\n");

				if(W5XXX_socketRecvAvailable(client->_sockfd)){ //if (client.available()) {
					rxlen = W5XXX_socket_recv(client->_sockfd,
							  rxbuf,
							  1024); //W5XXX_Client_read(rxbuf,1024); // char c = client.read();
					printf("rxlen= %u\r\n",rxlen);

					W5XXX_socket_send(client->_sockfd,"HTTP/1.1 200 OK\r\n",strlen("HTTP/1.1 200 OK\r\n"));
					W5XXX_socket_send(client->_sockfd,"Content-Type: text/html\r\n",strlen("Content-Type: text/html\r\n"));
					W5XXX_socket_send(client->_sockfd,"\r\n", 2);
					sprintf(sendbuf,">>> HELLO KONG WEB SVR WITH W5500 (%u) <<<\r\n", count++);
					W5XXX_socket_send(client->_sockfd,sendbuf,strlen(sendbuf));


				}
			}
			delayms(100);// give the web browser time to receive the data
			W5XXX_Client_stop(client->_sockfd); //close this connection.
			printf("client disconnected");
		}
*/
	}//while
}
/* Client ===================================================
void W5500_loop()
{
	//BitOrder(MSBFIRST);
	//SPI.setDataMode(SPI_MODE0);
	//SPI.setClockDivider(SPI_CLOCK_DIV4);
	stmW5XXX_SpiConfig();

	//unsigned char *mac, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet)
	W5XXX_Ethernet_begin(&myMac, myIP, myDNS, myDefaultGW, mySubnetMask);
  // if there are incoming bytes available from the server, read them and print them:

  if (W5XXX_socketRecvAvailable()) {
    char c = W5XXX_Client_read_byte();
    printf(c);
  }
  // if the server's W5500_socket_disconnected, stop the client:
  if (!W5XXX_isClientConnected()) {
    printf("W5500_socket_disconnecting.");
    W5500_Client_stop();

    // do nothing forevermore:
    while(true);
  }
}
*/
/* =================================================================================
 * udp

// Constructor
//void W5500_UDP(){
//	_sockfd(MAX_SOCK_NUM) {}
//}

// Start W5500_UDP W5500_socket, W5500_listening at local port PORT
unsigned char W5500_UDP_begin(unsigned short port) {
	int i;
	unsigned char s;

	W5500_UDP._sockfd = MAX_SOCK_NUM;

	if (W5500_UDP._sockfd != MAX_SOCK_NUM) return 0;

  for (i = 0; i < MAX_SOCK_NUM; i++) {
    s = W5500_readSnSR(i);
    if (s == SnSR_CLOSED || s == SnSR_FIN_WAIT) {
    	W5500_UDP._sockfd = i;
      break;
    }
  }

  if (W5500_UDP._sockfd == MAX_SOCK_NUM)
    return 0;

  W5500_UDP._port = port;
  W5500_UDP._remaining = 0;
  W5500_socket(W5500_UDP._sockfd, SnMR_UDP, W5500_UDP._port, 0);

  return 1;
}

// return number of bytes available in the current packet,
//   will return zero if parsePacket hasn't been called yet
int W5500_UDP_available() {
  return W5500_UDP._remaining;
}

// Release any resources being used by this W5500_UDP instance
void W5500_UDP_stop()
{
  if (W5500_UDP._sockfd == MAX_SOCK_NUM)
    return;

  W5XXX_socket_close(W5500_UDP._sockfd);

  W5XXX_Manager__server_port[W5500_UDP._sockfd] = 0;
  W5500_UDP._sockfd = MAX_SOCK_NUM;
}

//int W5500_UDP_beginPacket(const char *host, unsigned short port)
//{
  // Look up the host first
//  int ret = 0;
//  DNSClient dns;
//  IPAddress remote_addr;

//  dns.begin(Ethernet.dnsServerIP());
//  ret = dns.getHostByName(host, remote_addr);
//  if (ret == 1) {
//    return beginPacket(remote_addr, port);
//  } else {
//    return ret;
//  }
//}

int W5500_UDP_beginPacket(IPAddress ip, unsigned short port)
{
	W5500_UDP._offset = 0;
  return W5XXX_startUDP(W5500_UDP._sockfd, rawIPAddress(ip), port);
}

int W5500_UDP_endPacket()
{
  return W5XXX_socket_sendUDP(W5500_UDP._sockfd);
}

//size_t W5500_UDP_write(unsigned char byte){
//  return write(&byte, 1);
//}

size_t W5500_UDP_write(const unsigned char *buffer, size_t size)
{
  unsigned short bytes_written = W5XXX_socket_bufferDataForSend(W5500_UDP._sockfd, W5500_UDP._offset, buffer, size);
  W5500_UDP._offset += bytes_written;
  return bytes_written;
}

int W5500_UDP_parsePacket()
{
  // discard any remaining bytes in the last packet
	W5XXX_Client_flush();//flush();

  if (W5XXX_getRXReceivedSize(W5500_UDP._sockfd) > 0)
  {
    //HACK - hand-parse the UDP packet using TCP recv method
    unsigned char tmpBuf[8];
    int ret =0;
    //read 8 header bytes and get IP and port from it
    ret = W5500_recv(W5500_UDP._sockfd,tmpBuf,8);
    if (ret > 0)
    {
    	memcpy(&W5500_UDP._remoteIP,tmpBuf,4);
    	memcpy(&W5500_UDP._remotePort, tmpBuf[4],2);
    	W5500_UDP._remotePort = (W5500_UDP._remotePort << 8) + tmpBuf[5];
    	memcpy(&W5500_UDP._remaining,tmpBuf[6],2);
    	W5500_UDP._remaining = (W5500_UDP._remaining << 8) + tmpBuf[7];

      // When we get here, any remaining bytes are the data
    	ret = W5500_UDP._remaining;
    }
    return ret;
  }
  // There aren't any packets available
  return 0;
}

//int W5500_UDP_read()
//{
//  unsigned char byte;

//  if ((W5500_UDP._remaining > 0) && (W5500_recv(W5500_UDP._sockfd, &byte, 1) > 0))
//  {
    // We read things without any problems
//	  W5500_UDP._remaining--;
//    return byte;
//  }

  // If we get here, there's no data available
//  return -1;
//}

int W5500_UDP_read(unsigned char* buffer, size_t len)
{

  if (W5500_UDP._remaining > 0)
  {

    int got;

    if (W5500_UDP._remaining <= len)
    {
      // data should fit in the buffer
      got = W5500_recv(W5500_UDP._sockfd, buffer, W5500_UDP._remaining);
    }
    else
    {
      // too much data for the buffer,
      // grab as much as will fit
      got = W5500_recv(W5500_UDP._sockfd, buffer, len);
    }

    if (got > 0)
    {
    	W5500_UDP._remaining -= got;
      return got;
    }

  }

  // If we get here, there's no data available or recv failed
  return -1;

}

int W5500_UDP_peek()
{
  unsigned char b;
  // Unlike recv, peek doesn't check to see if there's any data available, so we must.
  // If the user hasn't called parsePacket yet then return nothing otherwise they
  // may get the UDP header
  if (!W5500_UDP._remaining)
    return -1;
  W5500_peek(W5500_UDP._sockfd, &b);
  return b;
}

void W5500_UDP_flush()
{
  // could this fail (loop endlessly) if _remaining > 0 and recv in read fails?
  // should only occur if recv fails after telling us the data is there, lets
  // hope the w5100 always behaves :)

  while (W5500_UDP._remaining)
  {
    read();
  }
}
//=======================================================================================
 */
#if defined(WIZ550io_WITH_MACADDRESS)
int W5500_Config(void)
{
  byte mac_address[6] ={0,};

  // Initialise the basic info
  W5XXX_ResetAndConfigBuf();
  W5XXX_setIPAddress(IPAddress(0,0,0,0).raw_address());
  W5500_getMACAddress(mac_address);

  // Now try to get our config info from a DHCP server
  int ret = _dhcp.beginWithDHCP(mac_address);
  if(ret == 1)
  {
    // We've successfully found a DHCP server and got our configuration info, so set things
    // accordingly
    W5XXX_setIPAddress(_dhcp.getLocalIp().raw_address());
    W5XXX_setGatewayIp(_dhcp.getGatewayIp().raw_address());
    W5XXX_setSubnetMask(_dhcp.getSubnetMask().raw_address());
    _dnsServerAddress = _dhcp.getDnsServerIp();
  }

  return ret;
}

void W5500_Config(IPAddress local_ip)
{
  // Assume the DNS server will be the machine on the same network as the local IP
  // but with last octet being '1'
  IPAddress dns_server = local_ip;
  dns_server[3] = 1;
  begin(local_ip, dns_server);
}

void W5500_Config(IPAddress local_ip, IPAddress dns_server)
{
  // Assume the gateway will be the machine on the same network as the local IP
  // but with last octet being '1'
  IPAddress gateway = local_ip;
  gateway[3] = 1;
  begin(local_ip, dns_server, gateway);
}

void W5500_Config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway)
{
  IPAddress subnet(255, 255, 255, 0);
  begin(local_ip, dns_server, gateway, subnet);
}

void W5500_Config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet)
{
	W5XXX_ResetAndConfigBuf();
  W5XXX_setIPAddress(local_ip.raw_address());
  W5XXX_setGatewayIp(gateway.raw_address());
  W5XXX_setSubnetMask(subnet.raw_address());
  _dnsServerAddress = dns_server;
}
#endif
/*
int W5XXX_Manager_maintain(){
  int rc = DHCP_CHECK_NONE;
  if(_dhcp != NULL){
    //we have a pointer to dhcp, use it
    rc = _dhcp.checkLease();
    switch ( rc ){
      case DHCP_CHECK_NONE:
        //nothing done
        break;
      case DHCP_CHECK_RENEW_OK:
      case DHCP_CHECK_REBIND_OK:
        //we might have got a new IP.
        W5XXX_setIPAddress(_dhcp.getLocalIp().raw_address());
        W5XXX_setGatewayIp(_dhcp.getGatewayIp().raw_address());
        W5XXX_setSubnetMask(_dhcp.getSubnetMask().raw_address());
        _dnsServerAddress = _dhcp.getDnsServerIp();
        break;
      default:
        //this is actually a error, it will retry though
        break;
    }
  }
  return rc;
}

IPAddress W5XXX_Manager_localIP()
{
  IPAddress ret;
  W5500_getIPAddress(ret.raw_address());
  return ret;
}

IPAddress W5500_getSubnetMask()
{
  IPAddress ret;
  W5500_getSubnetMask(ret.raw_address());
  return ret;
}

IPAddress W5XXX_Manager_gatewayIP()
{
  IPAddress ret;
  W5500_getGatewayIp(ret.raw_address());
  return ret;
}

IPAddress W5XXX_Manager_dnsServerIP()
{
  return _dnsServerAddress;
}

EthernetClass Ethernet;
*/


/*
int W5500_Client_W5500_connect_byName(const char* host, unsigned short port) {
  // Look up the host first
  int ret = 0;
  DNSClient dns;
  IPAddress remote_addr;

  dns.begin(Ethernet.dnsServerIP());
  ret = dns.getHostByName(host, remote_addr);
  if (ret == 1) {
    return W5500_connect(remote_addr, port);
  } else {
    return ret;
  }
}
*/
#if 0
void W5XXX_recv_data_processing(SOCKET s,
		unsigned char *data, //to be moved into.
		unsigned short len,
		unsigned char peek)
{
	unsigned short ptr, size, src_mask, src_ptr, inc;

#if 0
	ptr = state[s].RX_RD;
	if (data) W5XXX_read_data((unsigned short)_ptr , DUMMY, (unsigned char *)data, len); //read_data(s, ptr, data, ret);
	ptr += ret;
	state[s].RX_RD = ptr;
	state[s].RX_RSR -= ret;
	inc = state[s].RX_inc + ret;
	if (inc >= 250 || state[s].RX_RSR == 0) {
		state[s].RX_inc = 0;
		W5XXX_writeSnRX_RD(s, ptr);
		W5XXX_execCmdSn(s, Sock_RECV);
		printf("Sock_RECV cmd, RX_RD=%d, RX_RSR=%d\n",	state[s].RX_RD, state[s].RX_RSR);
	} else {
		state[s].RX_inc = inc;
	}
#elif 0
    ptr = W5XXX_readSnRX_RD(s); //get rx

    src_mask = (unsigned)ptr & getSn_RxMASK(s);
    src_ptr = (getSn_RxBase(s) + src_mask);
    if((src_mask + len) > getSn_RxMAX(s)){
    	size = getSn_RxMAX(s) - src_mask;
    	W5XXX_spi_read_Multi((unsigned short)src_ptr , DUMMY, (unsigned char *)data, size);
    	data += size;
    	size = len - size;
    	src_ptr = getSn_RxBase(s);
    	W5XXX_spi_read_Multi((unsigned short)src_ptr , DUMMY, (unsigned char *)data, size);
    }else{
    	W5XXX_spi_read_Multi((unsigned short)src_ptr , DUMMY, (unsigned char *)data, len);
    }
    ptr += len;
    W5XXX_writeSnRX_RD(s, ptr);
#endif
/*    W5XXX_read_data(s,
    		ptr, //from
    		data,//to
    		len);
    if (!peek){
        ptr += len;
        W5XXX_writeSnRX_RD(s, ptr);
    }
    */
}
#endif
// the next function allows us to use the client returned by
// EthernetServer::available() as the condition in an if-statement.
//W5500_Client::operator bool() {   return _sockfd != MAX_SOCK_NUM; }

//bool W5500_Client::operator==(const W5500_Client& rhs) {  return _sockfd == rhs._sockfd && _sockfd != MAX_SOCK_NUM && rhs._sockfd != MAX_SOCK_NUM; }
/*
  // if you get a W5500_connection, report back via serial:
  if (W5500_connect(server, 80)) {
    printf("W5500_connected");
    // Make an HTTP request:
    printf("GET /util/time.php HTTP/1.1");
    printf("Host: www.serasidis.gr");
    printf("Connection: close");
  }
  else {
    printf("W5500_connection failed");
  }
*/
