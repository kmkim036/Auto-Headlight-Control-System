//== w5500
/*
* Copyright (c) 2010 by WIZnet <support@wiznet.co.kr>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
/** Total RAM buffer is 16 kBytes for Transmitter and 16 kBytes for receiver for 1 Socket.
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
 *
 *  (c) 02 September 2015 By Vassilis Serasidis
 *
 */

//+--------+-----------+-----------+-----------+---------+---------+--------+
//|        |401-M34    |401-M35    | 407-M35   | 407-M36 |407VZT   | 103
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| ULED   | PB14      |PC4        |PE15       | <==     | PG7     |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| BUTTON |           |PC5(H)     |PD11(index)| PD11(L) |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| BEEP   |           |PB13       |PD14       | <==     |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| QEI    |           |PB0,1,12   |PD12,13,11 | PD12,13 |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| MOSI   |           |           |           |         |         |PB15    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| MISO   |           |           |           |         |         |PB14    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| SCLK   |           |           |           |         |         |PB13    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| nCS0   |           |           |           |         |         |PB12    |
//+--------+-----------+-----------+-----------+---------+---------+--------+
//| IRQ    |           |           |           |         |         |PB4     |
//+--------+-----------+-----------+-----------+---------+---------+--------+

//#include <SPI.h>
//#include <Ethernet_STM.h>
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
#include "cmdline.h"
#include "ySpiDrv.h"


//============================================
#if (PROCESSOR == STM32F401RET6)
extern void stmSpi1_Config(unsigned char nCS);
extern unsigned char stmSpi1RdByte();
extern void stmSpi1WrByte(unsigned char inbyte);
#else
extern void stmSpi2_Config(unsigned char nCS);
extern unsigned char stmSpi2RdByte();
extern void stmSpi2WrByte(unsigned char inbyte);
#endif

extern 	  g_Spi2ConfigDoneFlag;

extern unsigned char g_irq;
extern void stmLedToggle(void);
extern void Init_SysTick(int resolutionInUsec);
extern void stmUser_LED_GPIO_setup(void);
extern unsigned long millis();


//== SPI ==

//======================================================================
//use nCS0 of PB12 - 103; use nCS0 of PD10-407
#define nCS_W5500_H nCS0_H
#define nCS_W5500_L nCS0_L

//BitOrder(MSBFIRST);
//SPI.setDataMode(SPI_MODE0);
//SPI.setClockDivider(SPI_CLOCK_DIV4);

void stmW5500_SpiConfig(void){
	//2Mbps, 8 bit mode
	//use MODE 0
#if (PROCESSOR == STM32F401RET6)
	stmSPI1_Config(0,0,8); //use PA15 of nCS0
#else
	stmSPI2_Config(0,0,8); //use PD10 of nCS0
#endif
	nCS_W5500_H;//nCs=1

	printf("W5500> SPI CONFIG DONE.\r\n");
}

// W5500 Registers -- via SPI
//  static unsigned char  write(unsigned short _addr, unsigned char _cb, unsigned char _data);
//  static unsigned char  read(unsigned short _addr, unsigned char _cb );
//static unsigned short W5500_write(unsigned short _addr, unsigned char _cb, const unsigned char *buf, unsigned short len);
//static unsigned short W5500_read(unsigned short _addr, unsigned char _cb, unsigned char *buf, unsigned short len);

unsigned char W5500_spi_write_Single(unsigned short _addr, unsigned char _cb, unsigned char _data)
{
	nCS_W5500_L;
	stmSpi2WrByte(_addr >> 8); //2 byte reg addr
	stmSpi2WrByte(_addr & 0xFF);
	stmSpi2WrByte(_cb);  // control byte
	stmSpi2WrByte(_data); //data phase
	nCS_W5500_H;
	return 1;
}

unsigned short W5500_spi_write_Multi(unsigned short _addr, unsigned char _cb, const unsigned char *_buf, unsigned short _len)
{
	unsigned short i;
	nCS_W5500_L;
	stmSpi2WrByte(_addr >> 8);
	stmSpi2WrByte(_addr & 0xFF);
	stmSpi2WrByte(_cb);
	for (i=0; i<_len; i++){
		stmSpi2WrByte(_buf[i]);
	}
	nCS_W5500_H;
    return _len;
}

unsigned char W5500_spi_read_Single(unsigned short _addr, unsigned char _cb)
{
	nCS_W5500_L;
	stmSpi2WrByte(_addr >> 8);
	stmSpi2WrByte(_addr & 0xFF);
	stmSpi2WrByte(_cb);
	unsigned char _data = stmSpi2RdByte();
	nCS_W5500_H;
    return _data;
}

unsigned short W5500_spi_read_Multi(unsigned short _addr, unsigned char _cb, unsigned char *_buf, unsigned short _len)
{
	unsigned short i;
	nCS_W5500_L;
	stmSpi2WrByte(_addr >> 8);
	stmSpi2WrByte(_addr & 0xFF);
	stmSpi2WrByte(_cb);

  for (i=0; i<_len; i++){
        _buf[i] = stmSpi2RdByte();
  }
	nCS_W5500_H;
    return _len;
}
//============================
struct _IPAddress{
	union {
		unsigned char bytes[4];
		unsigned long dword;
	};
};
typedef struct _IPAddress IPAddress;
unsigned char *W5500_getRaw_address(IPAddress ipaddr){
	  return ipaddr.bytes; //ptr
}

typedef unsigned char SOCKET;
#define MAX_SOCK_NUM 1//8   // Select the number of Sockets (1-8)
#define RXBUF_SIZE   16//2   // Select the Receiver buffer size (1 - 16 kBytes)
#define TXBUF_SIZE   16//2   // Select the Transmitter buffer size (1 - 16 kBytes)



#define SnMR_CLOSE  0x00
#define SnMR_TCP     0x01
#define SnMR_UDP     0x02
#define SnMR_IPRAW   0x03
#define SnMR_MACRAW  0x04
#define SnMR_PPPOE   0x05
#define SnMR_ND      0x20
#define SnMR_MULTI   0x80


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


#define SnIR_SEND_OK  0x10
#define SnIR_TIMEOUT 0x08
#define SnIR_RECV    0x04
#define SnIR_DISCON  0x02
#define SnIR_CON     0x01



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

  static const unsigned char  RST = 7; // Reset BIT
  static const int SOCKETS = 8;
  static const unsigned short W5500_SSIZE = 2048; // Max Tx buffer size
  static const unsigned short W5500_RSIZE = 2048; // Max Rx buffer size



  //============================================================
  //Protos
  int  W5500_ResetAndConfigBuf();
  /**
   * @brief	This function is being used for copy the data form Receive buffer of the chip to application buffer.
   *
   * It calculate the actual physical address where one has to read
   * the data from Receive buffer. Here also take care of the condition while it exceed
   * the Rx memory uper-bound of W5500_socket.
   */
  void W5500_read_data(SOCKET s, volatile unsigned short  src, volatile unsigned char * dst, unsigned short len);

  /**
   * @brief	 This function is being called by W5500_socket_send() and W5500_socket_sendto() function also.
   *
   * This function read the Tx write pointer register and after copy the data in buffer update the Tx write pointer
   * register. User should read upper byte first and lower byte later to get proper value.
   */
  void W5500_socket_send_data_processing(SOCKET s, const unsigned char *data, unsigned short len);
  /**
   * @brief A copy of W5500_socket_send_data_processing that uses the provided ptr for the
   *        write offset.  Only needed for the "streaming" UDP API, where
   *        a single UDP packet is built up over a number of calls to
   *        W5500_socket_send_data_processing_ptr, because TX_WR doesn't seem to get updated
   *        correctly in those scenarios
   * @param ptr value to use in place of TX_WR.  If 0, then the value is read
   *        in from TX_WR
   * @return New value for ptr, to be used in the next call
   */
  // FIXME Update documentation
  void W5500_send_data_processing_offset(SOCKET s, unsigned short data_offset, const unsigned char *data, unsigned short len);

  /**
   * @brief	This function is being called by recv() also.
   *
   * This function read the Rx read pointer register
   * and after copy the data from receive buffer update the Rx write pointer register.
   * User should read upper byte first and lower byte later to get proper value.
   */
  void recv_data_processing(SOCKET s, unsigned char *data, unsigned short len, unsigned char peek);

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
  unsigned short getRXReceivedSize(SOCKET s);

  unsigned char W5500_readSn_Single(SOCKET _s, unsigned short _addr) {
      unsigned char cntl_byte = (_s<<5)+0x08;
      return W5500_spi_read_Single(_addr, cntl_byte);
  }

  unsigned char W5500_writeSn_Single(SOCKET _s, unsigned short _addr, unsigned char _data) {
      unsigned char cntl_byte = (_s<<5)+0x0C;
      return W5500_spi_write_Single(_addr, cntl_byte, _data);
}

  unsigned short W5500_readSn_Multi(SOCKET _s, unsigned short _addr, unsigned char *_buf, unsigned short _len) {
      unsigned char cntl_byte = (_s<<5)+0x08;
      return W5500_spi_read_Multi(_addr, cntl_byte, _buf, _len );
  }

  unsigned short W5500_writeSn_Multi(SOCKET _s, unsigned short _addr, unsigned char *_buf, unsigned short _len) {
      unsigned char cntl_byte = (_s<<5)+0x0C;
      return W5500_spi_write_Multi(_addr, cntl_byte, _buf, _len);
  }

//struct W5500Class {

#define __GP_REGISTER8(name, address)             \
  static inline void W5500_write##name(unsigned char _data) { \
	W5500_spi_write_Single(address, 0x04, _data);                  \
  }                                               \
  static inline unsigned char read##name() {            \
    return W5500_spi_read_Single(address, 0x00);                   \
  }
#define __GP_REGISTER16(name, address)            \
  static void W5500_write##name(unsigned short _data) {       \
	W5500_spi_write_Single(address,  0x04, _data >> 8);            \
	W5500_spi_write_Single(address+1, 0x04, _data & 0xFF);         \
  }                                               \
  static unsigned short W5500_read##name() {                  \
    unsigned short res = W5500_spi_read_Single(address, 0x00);           \
    res = (res << 8) + W5500_spi_read_Single(address + 1, 0x00);   \
    return res;                                   \
  }
#define __GP_REGISTER_N(name, address, size)      \
  static unsigned short W5500_write##name(unsigned char *_buff) {   \
    return W5500_spi_write_Multi(address, 0x04, _buff, size);     \
  }                                               \
  static unsigned short W5500_read##name(unsigned char *_buff) {    \
    return W5500_spi_read_Multi(address, 0x00, _buff, size);      \
  }


  __GP_REGISTER8 (MR,     0x0000);    // Mode Register
  __GP_REGISTER_N(GAR,    0x0001, 4); // Gateway IP address
  __GP_REGISTER_N(SUBR,   0x0005, 4); // Subnet mask address
  __GP_REGISTER_N(SHAR,   0x0009, 6); // Source MAC address
  __GP_REGISTER_N(SIPR,   0x000F, 4); // Source IP address
  __GP_REGISTER8 (IR,     0x0015);    // Interrupt
  __GP_REGISTER8 (IMR,    0x0016);    // Interrupt Mask
  __GP_REGISTER16(RTR,    0x0019);    // Timeout address
  __GP_REGISTER8 (RCR,    0x001B);    // Retry count
  __GP_REGISTER_N(UIPR,   0x0028, 4); // Unreachable IP address in UDP mode
  __GP_REGISTER16(UPORT,  0x002C);    // Unreachable Port address in UDP mode
  __GP_REGISTER8 (PHYCFGR,     0x002E);    // PHY Configuration register, default value: 0b 1011 1xxx


#undef __GP_REGISTER8
#undef __GP_REGISTER16
#undef __GP_REGISTER_N

  // W5500 Socket registers
  // ----------------------
  //static inline unsigned char W5500_readSn(SOCKET _s, unsigned short _addr);
  //static inline unsigned char W5500_writeSn(SOCKET _s, unsigned short _addr, unsigned char _data);
  //static inline unsigned short W5500_readSn(SOCKET _s, unsigned short _addr, unsigned char *_buf, unsigned short len);
  //static inline unsigned short W5500_writeSn(SOCKET _s, unsigned short _addr, unsigned char *_buf, unsigned short len);

  //static const unsigned short CH_BASE = 0x0000;
  //static const unsigned short CH_SIZE = 0x0000;

#define __SOCKET_REGISTER8(name, address)                    \
  static inline void W5500_write##name(SOCKET _s, unsigned char _data) { \
	W5500_writeSn_Single(_s, address, _data);                             \
  }                                                          \
  static inline unsigned char W5500_read##name(SOCKET _s) {              \
    return W5500_readSn_Single(_s, address);                              \
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
  static void W5500_write##name(SOCKET _s, unsigned short _data) {       \
	W5500_writeSn_Single(_s, address,   _data >> 8);                      \
	W5500_writeSn_Single(_s, address+1, _data & 0xFF);                    \
  }                                                          \
  static unsigned short W5500_read##name(SOCKET _s) {                    \
    unsigned short res = W5500_readSn_Single(_s, address);                      \
    res = (res << 8) + W5500_readSn_Single(_s, address + 1);              \
    return res;                                              \
  }
#endif
#define __SOCKET_REGISTER_N(name, address, size)             \
  static unsigned short W5500_write##name(SOCKET _s, unsigned char *_buff) {   \
    return W5500_writeSn_Multi(_s, address, _buff, size);                \
  }                                                          \
  static unsigned short W5500_read##name(SOCKET _s, unsigned char *_buff) {    \
    return W5500_readSn_Multi(_s, address, _buff, size);                 \
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



void W5500_getGatewayIp(unsigned char *_addr) {
	  W5500_readGAR(_addr);
}

void W5500_setGatewayIp(unsigned char *_addr) {
	W5500_writeGAR(_addr);
}

void W5500_getSubnetMask(unsigned char *_addr) {
	W5500_readSUBR(_addr);
}

void W5500_setSubnetMask(unsigned char *_addr) {
	W5500_writeSUBR(_addr);
}

void W5500_getMACAddress(unsigned char *_addr) {
	W5500_readSHAR(_addr);
}

void W5500_setMACAddress(unsigned char *_addr) {
	W5500_writeSHAR(_addr);
}

void W5500_getIPAddress(unsigned char *_addr) {
	W5500_readSIPR(_addr);
}

void W5500_setIPAddress(unsigned char *_addr) {
	W5500_writeSIPR(_addr);
}

void W5500_setRetransmissionTime(unsigned short _timeout) {
	W5500_writeRTR(_timeout);
}

void W5500_setRetransmissionCount(unsigned char _retry) {
	W5500_writeRCR(_retry);
}

void W5500_setPHYCFGR(unsigned char _val) {
	W5500_writePHYCFGR(_val);
}

unsigned char W5500_getPHYCFGR() {
//  readPHYCFGR();
  return W5500_read(0x002E, 0x00);
}

// Initialize the Ethernet shield to use the provided MAC address and gain the rest of the
// configuration through DHCP.
// Returns 0 if the DHCP configuration failed, and 1 if it succeeded
//void W5500_begin(unsigned char *mac_address, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);

struct  _W5500_Client{
	  unsigned short _srcport;
	  unsigned char _sockfd;
};

struct _W5500_Server {
  unsigned short _port;
};

struct _W5500Class {

	//================== CONFIG ==================================
	  // Enter a MAC address for your controller below.
	  // Newer Ethernet shields have a MAC address printed on a sticker on the shield
	  unsigned char myMac[6];
	  IPAddress myIP;
	  IPAddress mySubnetMask;
	  IPAddress myDefaultGW;
	  IPAddress dnsServerAddress;
	  IPAddress serverIP;

	  unsigned 	char 			state[MAX_SOCK_NUM];
	  unsigned 	short 			server_port[MAX_SOCK_NUM];
	  struct  	_W5500_Client 	client;//[MAX_SOCK_NUM];
	  struct 	_W5500_Server 	server;
	  // friend class W5500_Client;
	  // friend class EthernetServer;
};

struct _W5500Class W5500_Manager;

#if defined(WIZ550io_WITH_MACADDRESS)
  // Initialize function when use the ioShield series (included WIZ550io)
  // WIZ550io has a MAC address which is written after reset.
  // Default IP, Gateway and subnet address are also writen.
  // so, It needs some initial time. please refer WIZ550io Datasheet in details.
  //void begin(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);
#else



#endif

unsigned char W5500_Get_Client_status();

/*
 *   virtual int W5500_connect(IPAddress ip, unsigned short port);
  virtual int W5500_connect(const char *host, unsigned short port);
  virtual size_t write(unsigned char);
  virtual size_t write(const unsigned char *buf, size_t size);
  virtual int available();
  virtual int read();
  virtual int read(unsigned char *buf, size_t size);
  virtual int peek();
  virtual void flush();
  virtual void stop();
  virtual unsigned char W5500_connected();
  virtual operator bool();
  virtual bool operator==(const W5500_Client&);
  virtual bool operator!=(const W5500_Client& rhs) { return !this->operator==(rhs); };

  friend class EthernetServer;

  using Print::write;

private:
 *
 */
//======================================================
int W5500_GetLinkStatus(void){
	unsigned short retb;
	retb = W5500_spi_read_Single(0x2e, 0x01); // 1 byte read
	//retb = retb << 8 + W5500_spi_read_Single(0x2f, 0x01); // 1 byte read
	if(retb & 0x01){
		printf("Link Up \r\n");
		return 1;
	}else{
		printf("Link Down \r\n");
		return 0;
	}
}

int W5500_ResetAndConfigBuf(void)
{
	int i;
	unsigned short retb;
    delayms(1000);

    W5500_spi_write_Single(0x00, 0x05, 128); // Software reset the W5500 chip
    delayms(100);
    for (i=0; i<MAX_SOCK_NUM; i++) {
    	unsigned char cntl_byte = (0x0C + (i<<5));
    	W5500_spi_write_Single( 0x1E, cntl_byte, RXBUF_SIZE); //0x1E - Sn_RXBUF_SIZE
    	W5500_spi_write_Single( 0x1F, cntl_byte, TXBUF_SIZE); //0x1F - Sn_TXBUF_SIZE
    }
    //read config register
    //MR (mode reg = 0x00)
    for(i=0; i<0xff; i+=2){
    	retb = W5500_spi_read_Single(i, 0x01); // 1 byte read
    	retb = retb << 8 + W5500_spi_read_Single(i+1, 0x01); // 1 byte read
    	printf("Reg0x%02x = 0x%02x\r\n", i, retb);
    	delayms(50);
    }

    return W5500_GetLinkStatus();
}

unsigned short W5500_getTXFreeSize(SOCKET s)
{
    unsigned short val=0, val1=0;
    do {
        val1 = W5500_readSnTX_FSR(s);
        if (val1 != 0)
            val = W5500_readSnTX_FSR(s);
    }
    while (val != val1);
    return val;
}

unsigned short W5500_getRXReceivedSize(SOCKET s)
{
    unsigned short val=0,val1=0;
    do {
        val1 = W5500_readSnRX_RSR(s);
        if (val1 != 0)
            val = W5500_readSnRX_RSR(s);
    }
    while (val != val1);
    return val;
}

//======================= socket ==================================================================
void W5500_send_data_processing_offset(SOCKET s, unsigned short data_offset, const unsigned char *data, unsigned short len)
{

    unsigned short ptr = W5500_readSnTX_WR(s);
    unsigned char cntl_byte = (0x14+(s<<5));
    ptr += data_offset;
    W5500_spi_write_Multi(ptr, cntl_byte, data, len);
    ptr += len;
    W5500_writeSnTX_WR(s, ptr);

}

void W5500_send_data_processing(SOCKET s, const unsigned char *data, unsigned short len)
{
  // This is same as having no offset in a call to W5500_send_data_processing_offset
  W5500_send_data_processing_offset(s, 0, data, len);

}

void W5500_read_data(SOCKET s, volatile unsigned short src, volatile unsigned char *dst, unsigned short len)
{
    unsigned char cntl_byte = (0x18+(s<<5));
    W5500_spi_read_Multi((unsigned short)src , cntl_byte, (unsigned char *)dst, len);
}

void W5500_recv_data_processing(SOCKET s, unsigned char *data, unsigned short len, unsigned char peek)
{
    unsigned short ptr;
    ptr = W5500_readSnRX_RD(s);

    W5500_read_data(s, ptr, data, len);
    if (!peek){
        ptr += len;
        W5500_writeSnRX_RD(s, ptr);
    }
}

void W5500_execCmdSn(SOCKET s, SockCMD _cmd) {
    // Send command to W5500_socket
	W5500_writeSnCR(s, _cmd);
    // Wait for command to complete
    while (W5500_readSnCR(s))
    ;
}
//============== W5500_socket =
extern unsigned char W5500_socket(SOCKET s, unsigned char protocol, unsigned short port, unsigned char flag); // Opens a W5500_socket(TCP or UDP or IP_RAW mode)
extern void W5500_socket_close(SOCKET s); // Close W5500_socket
extern unsigned char W5500_connect(SOCKET s, unsigned char * addr, unsigned short port); // Establish TCP W5500_connection (Active W5500_connection)
extern void W5500_socket_disconnect(SOCKET s); // W5500_socket_disconnect the W5500_connection
extern unsigned char W5500_listen(SOCKET s);	// Establish TCP W5500_connection (Passive W5500_connection)
extern unsigned short W5500_socket_send(SOCKET s, const unsigned char * buf, unsigned short len); // Send data (TCP)
extern int16_t W5500_recv(SOCKET s, unsigned char * buf, int16_t len);	// Receive data (TCP)
extern unsigned short peek(SOCKET s, unsigned char *buf);
extern unsigned short W5500_socket_sendto(SOCKET s, const unsigned char * buf, unsigned short len, unsigned char * addr, unsigned short port); // Send data (UDP/IP RAW)
extern unsigned short W5500_recvfrom(SOCKET s, unsigned char * buf, unsigned short len, unsigned char * addr, unsigned short *port); // Receive data (UDP/IP RAW)
extern void flush(SOCKET s); // Wait for transmission to complete

extern unsigned short igmpW5500_socket_send(SOCKET s, const unsigned char * buf, unsigned short len);

// Functions to allow buffered UDP W5500_socket_send (i.e. where the UDP datagram is built up over a
// number of calls before being sent
/*
  @brief This function sets up a UDP datagram, the data for which will be provided by one
  or more calls to W5500_bufferingDataForSend and then finally sent with W5500_socket_sendUDP.
  @return 1 if the datagram was successfully set up, or 0 if there was an error
*/
extern int W5500_startUDP(SOCKET s, unsigned char* addr, unsigned short port);
/*
  @brief This function copies up to len bytes of data from buf into a UDP datagram to be
  sent later by W5500_socket_sendUDP.  Allows datagrams to be built up from a series of W5500_bufferingDataForSend calls.
  @return Number of bytes successfully buffered
*/
unsigned short W5500_bufferingDataForSend(SOCKET s, unsigned short offset, const unsigned char* buf, unsigned short len);
/*
  @brief Send a UDP datagram built up from a sequence of W5500_startUDP followed by one or more
  calls to W5500_bufferingDataForSend.
  @return 1 if the datagram was successfully sent, or 0 if there was an error
*/
int W5500_socket_sendUDP(SOCKET s);



static unsigned short local_port;

/**
 * @brief	This Socket function initialize the channel in perticular mode, and set the port and wait for W5500 done it.
 * @return 	1 for success else 0.
 */
unsigned char W5500_socket(SOCKET s, unsigned char protocol, unsigned short port, unsigned char flag)
{
  if ((protocol == SnMR_TCP) || (protocol == SnMR_UDP) || (protocol == SnMR_IPRAW) || (protocol == SnMR_MACRAW) || (protocol == SnMR_PPPOE))
  {
	  W5500_socket_close(s);
    W5500_writeSnMR(s, protocol | flag);
    if (port != 0) {
      W5500_writeSnPORT(s, port);
    }
    else {
      local_port++; // if don't set the source port, set local_port number.
      W5500_writeSnPORT(s, local_port);
    }

    W5500_execCmdSn(s, Sock_OPEN);

    return 1;
  }

  return 0;
}

/**
 * @brief	This function established  the W5500_connection for the channel in Active (client) mode.
 * 		This function waits for the untill the W5500_connection is established.
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
  W5500_writeSnDIPR(s, addr);
  W5500_writeSnDPORT(s, port);
  W5500_execCmdSn(s, Sock_CONNECT);

  return 1;
}

/**
 * @brief	This function used for W5500_socket_disconnect the W5500_socket and parameter is "s" which represent the W5500_socket number
 * @return	1 for success else 0.
 */
void W5500_socket_disconnect(SOCKET s) //gracefully
{
  W5500_execCmdSn(s, Sock_DISCON);
}


/**
 * @brief	This function close the W5500_socket and parameter is "s" which represent the W5500_socket number
 */
void W5500_socket_close(SOCKET s) //forcefully
{
  W5500_execCmdSn(s, Sock_CLOSE);
  W5500_writeSnIR(s, 0xFF);
}

//=========== Server ==================
unsigned char W5500_socket_listen(SOCKET s) { //in passive mode
  if (W5500_readSnSR(s) != SnSR_INIT)
    return 0;
  W5500_execCmdSn(s, Sock_LISTEN);
  return 1;
}

unsigned char W5500_socket_accept()//SOCKET s)
{
	int listening = 0;
	int sock;
	for(sock=0; sock <MAX_SOCK_NUM;sock++){
		if(W5500_Manager.server_port[sock] == W5500_Manager.server._port){
			if(W5500_Get_Client_status(sock) == SnSR_LISTEN){
				listening = 1;
			}else if(W5500_Get_Client_status(sock) == SnSR_CLOSE_WAIT && !W5500_Client_Is_Available()){
				W5500_Client_stop();
			}
		}
	}
	if(!listening){
		W5500_Server_begin(W5500_Manager.server._port);
	}
  return 1;
}

int W5500_Server_Check_ClientAvailable() {
	int listening = 0;
	int sock;

	W5500_socket_accept();
	for(sock=0; sock <MAX_SOCK_NUM;sock++){
		//W5500_Manager.client.sock
		if(W5500_Manager.server_port[sock] == W5500_Manager.server._port &&
				(W5500_Get_Client_status() == SnSR_ESTABLISHED || W5500_Get_Client_status() == SnSR_CLOSE_WAIT)){
			if(W5500_Client_Is_Available()) {
				///XXX don't always pick the lowest numbered socket
				return 1; //client
			}
		}
	}
	return 0;
}
//=======================

/**
 * @brief	This function used to W5500_socket_send the data in TCP mode
 * @return	1 for success else 0.
 */
unsigned short W5500_socket_send(SOCKET s, const unsigned char * buf, unsigned short len)
{
  unsigned char status=0;
  unsigned short ret=0;
  unsigned short freesize=0;

  if (len > W5500_SSIZE)
    ret = W5500_SSIZE; // check size not to exceed MAX size.
  else
    ret = len;

  // if freebuf is available, start.
  do
  {
    freesize = W5500_getTXFreeSize(s);
    status = W5500_readSnSR(s);
    if ((status != SnSR_ESTABLISHED) && (status != SnSR_CLOSE_WAIT))
    {
      ret = 0;
      break;
    }
  }
  while (freesize < ret);

  // copy data
  W5500_send_data_processing(s, (unsigned char *)buf, ret);
  W5500_execCmdSn(s, Sock_SEND);

  /* +2008.01 bj */
  while ( (W5500_readSnIR(s) & SnIR_SEND_OK) != SnIR_SEND_OK )
  {
    /* m2008.01 [bj] : reduce code */
    if ( W5500_readSnSR(s) == SnSR_CLOSED )
    {
    	W5500_socket_close(s);
      return 0;
    }
  }
  /* +2008.01 bj */
  W5500_writeSnIR(s, SnIR_SEND_OK);
  return ret;
}


/**
 * @brief	This function is an application I/F function which is used to receive the data in TCP mode.
 * 		It continues to wait for data as much as the application wants to receive.
 *
 * @return	received data size for success else -1.
 */
int16_t W5500_socket_recv(SOCKET s, unsigned char *buf, int16_t len)
{
  // Check how much data is available
  int16_t ret = W5500_getRXReceivedSize(s);
  if ( ret == 0 )
  {
    // No data available.
    unsigned char status = W5500_readSnSR(s);
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
  else if (ret > len)
  {
    ret = len;
  }

  if ( ret > 0 )
  {
    W5500_recv_data_processing(s, buf, ret, 0);
    W5500_execCmdSn(s, Sock_RECV);
  }
  return ret;
}


/**
 * @brief	Returns the first byte in the receive queue (no checking)
 *
 * @return
 */
unsigned short W5500_socket_peek(SOCKET s, unsigned char *buf)
{
  W5500_recv_data_processing(s, buf, 1, 1);

  return 1;
}


/**
 * @brief	This function is an application I/F function which is used to W5500_socket_send the data for other then TCP mode.
 * 		Unlike TCP transmission, The peer's destination address and the port is needed.
 *
 * @return	This function return W5500_socket_send data size for success else -1.
 */
unsigned short W5500_socket_sendto(SOCKET s, const unsigned char *buf, unsigned short len, unsigned char *addr, unsigned short port)
{
  unsigned short ret=0;

  if (len > W5500_SSIZE) ret = W5500_SSIZE; // check size not to exceed MAX size.
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
    W5500_writeSnDIPR(s, addr);
    W5500_writeSnDPORT(s, port);

    // copy data
    W5500_send_data_processing(s, (unsigned char *)buf, ret);
    W5500_execCmdSn(s, Sock_SEND);

    /* +2008.01 bj */
    while ( (W5500_readSnIR(s) & SnIR_SEND_OK) != SnIR_SEND_OK )
    {
      if (W5500_readSnIR(s) & SnIR_TIMEOUT)
      {
        /* +2008.01 [bj]: clear interrupt */
        W5500_writeSnIR(s, (SnIR_SEND_OK | SnIR_TIMEOUT)); /* clear SEND_OK & TIMEOUT */
        return 0;
      }
    }

    /* +2008.01 bj */
    W5500_writeSnIR(s, SnIR_SEND_OK);
  }
  return ret;
}


/**
 * @brief	This function is an application I/F function which is used to receive the data in other then
 * 	TCP mode. This function is used to receive UDP, IP_RAW and MAC_RAW mode, and handle the header as well.
 *
 * @return	This function return received data size for success else -1.
 */
unsigned short W5500_recvfrom(SOCKET s, unsigned char *buf, unsigned short len, unsigned char *addr, unsigned short *port)
{
  unsigned char head[8];
  unsigned short data_len=0;
  unsigned short ptr=0;

  if ( len > 0 )
  {
    ptr = W5500_readSnRX_RD(s);
    switch (W5500_readSnMR(s) & 0x07)
    {
    case SnMR_UDP :
      W5500_read_data(s, ptr, head, 0x08);
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

      W5500_read_data(s, ptr, buf, data_len); // data copy.
      ptr += data_len;

      W5500_writeSnRX_RD(s, ptr);
      break;

    case SnMR_IPRAW :
      W5500_read_data(s, ptr, head, 0x06);
      ptr += 6;

      addr[0] = head[0];
      addr[1] = head[1];
      addr[2] = head[2];
      addr[3] = head[3];
      data_len = head[4];
      data_len = (data_len << 8) + head[5];

      W5500_read_data(s, ptr, buf, data_len); // data copy.
      ptr += data_len;

      W5500_writeSnRX_RD(s, ptr);
      break;

    case SnMR_MACRAW:
      W5500_read_data(s, ptr, head, 2);
      ptr+=2;
      data_len = head[0];
      data_len = (data_len<<8) + head[1] - 2;

      W5500_read_data(s, ptr, buf, data_len);
      ptr += data_len;
      W5500_writeSnRX_RD(s, ptr);
      break;

    default :
      break;
    }
    W5500_execCmdSn(s, Sock_RECV);
  }
  return data_len;
}

/**
 * @brief	Wait for buffered transmission to complete.
 */
void W5500_flush(SOCKET s) {
  // TODO
}

unsigned short W5500_igmp_send(SOCKET s, const unsigned char * buf, unsigned short len)
{
  unsigned char status=0;
  unsigned short ret=0;

  if (len > W5500_SSIZE)
    ret = W5500_SSIZE; // check size not to exceed MAX size.
  else
    ret = len;

  if (ret == 0)
    return 0;

  W5500_socket_send_data_processing(s, (unsigned char *)buf, ret);
  W5500_execCmdSn(s, Sock_SEND);

  while ( (W5500_readSnIR(s) & SnIR_SEND_OK) != SnIR_SEND_OK )
  {
    status = W5500_readSnSR(s);
    if (W5500_readSnIR(s) & SnIR_TIMEOUT)
    {
      /* in case of igmp, if W5500_socket_send fails, then W5500_socket closed */
      /* if you want change, remove this code. */
    	W5500_socket_close(s);
      return 0;
    }
  }

  W5500_writeSnIR(s, SnIR_SEND_OK);
  return ret;
}

unsigned short W5500_bufferingDataForSend(SOCKET s, unsigned short offset, const unsigned char* buf, unsigned short len)
{
  unsigned short ret =0;
  if (len > W5500_getTXFreeSize(s))
  {
    ret = W5500_getTXFreeSize(s); // check size not to exceed MAX size.
  }
  else
  {
    ret = len;
  }
  W5500_send_data_processing_offset(s, offset, buf, ret);
  return ret;
}

int W5500_startUDP(SOCKET s, unsigned char* addr, unsigned short port)
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
    W5500_writeSnDIPR(s, addr);
    W5500_writeSnDPORT(s, port);
    return 1;
  }
}

int W5500_socket_sendUDP(SOCKET s)
{
  W5500_execCmdSn(s, Sock_SEND);

  /* +2008.01 bj */
  while ( (W5500_readSnIR(s) & SnIR_SEND_OK) != SnIR_SEND_OK )
  {
    if (W5500_readSnIR(s) & SnIR_TIMEOUT)
    {
      /* +2008.01 [bj]: clear interrupt */
      W5500_writeSnIR(s, (SnIR_SEND_OK|SnIR_TIMEOUT));
      return 0;
    }
  }

  /* +2008.01 bj */
  W5500_writeSnIR(s, SnIR_SEND_OK);

  /* Sent ok */
  return 1;
}
//==========

/*
 *  Udp.cpp: Library to W5500_socket_send/receive UDP packets.
 * NOTE: UDP is fast, but has some important limitations (thanks to Warren Gray for mentioning these)
 */

#define UDP_TX_PACKET_MAX_SIZE 24
/*
 *   W5500_UDP();  // Constructor
  virtual unsigned char begin(unsigned short);	// initialize, start W5500_listening on specified port. Returns 1 if successful, 0 if there are no W5500_sockets available to use
  virtual void stop();  // Finish with the UDP W5500_socket

  // Sending UDP packets

  // Start building up a packet to W5500_socket_send to the remote host specific in ip and port
  // Returns 1 if successful, 0 if there was a problem with the supplied IP address or port
  virtual int beginPacket(IPAddress ip, unsigned short port);
  // Start building up a packet to W5500_socket_send to the remote host specific in host and port
  // Returns 1 if successful, 0 if there was a problem resolving the hostname or port
  virtual int beginPacket(const char *host, unsigned short port);
  // Finish off this packet and W5500_socket_send it
  // Returns 1 if the packet was sent successfully, 0 if there was an error
  virtual int endPacket();
  // Write a single byte into the packet
  virtual size_t write(unsigned char);
  // Write size bytes from buffer into the packet
  virtual size_t write(const unsigned char *buffer, size_t size);

  using Print::write;

  // Start processing the next available incoming packet
  // Returns the size of the packet in bytes, or 0 if no packets are available
  virtual int parsePacket();
  // Number of bytes remaining in the current packet
  virtual int available();
  // Read a single byte from the current packet
  virtual int read();
  // Read up to len bytes from the current packet and place them into buffer
  // Returns the number of bytes read, or 0 if none are available
  virtual int read(unsigned char* buffer, size_t len);
  // Read up to len characters from the current packet and place them into buffer
  // Returns the number of characters read, or 0 if none are available
  virtual int read(char* buffer, size_t len) { return read((unsigned char*)buffer, len); };
  // Return the next byte from the current packet without moving on to the next byte
  virtual int peek();
  virtual void flush();	// Finish reading the current packet

  // Return the IP address of the host who sent the current incoming packet
  virtual IPAddress remoteIP() { return _remoteIP; };
  // Return the port of the host who sent the current incoming packet
  virtual unsigned short remotePort() { return _remotePort; };
 */
struct _W5500_UDP{
  unsigned char _sockfd;  // W5500_socket ID for Wiz5100
  unsigned short _port; // local port to W5500_listen on
  IPAddress _remoteIP; // remote IP address for the incoming packet whilst it's being processed
  unsigned short _remotePort; // remote port for the incoming packet whilst it's being processed
  unsigned short _offset; // offset into the packet being sent
  unsigned short _remaining; // remaining bytes of incoming packet yet to be processed
};

struct _W5500_UDP W5500_UDP;

IPAddress localIP();
IPAddress subnetMask();
IPAddress gatewayIP();
IPAddress dnsServerIP();

Init_W5500_Client(unsigned char sock);





//==
/*
  Web client

 This sketch W5500_connects to a website (http://www.google.com)
 using an Arduino Wiznet Ethernet shield.

 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13

 created 18 Dec 2009
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe, based on work by Adrian McEwen
 modified 12 Aug 2013
 by Soohwan Kim
 Modified 18 Aug 2015
 by Vassilis Serasidis

 =========================================================
 Ported to STM32F103 on 18 Aug 2015 by Vassilis Serasidis

 <---- Pinout ---->
 W5x00 <--> STM32F103
 SS    <-->  PA4 <-->  BOARD_SPI1_NSS_PIN
 SCK   <-->  PA5 <-->  BOARD_SPI1_SCK_PIN
 MISO  <-->  PA6 <-->  BOARD_SPI1_MISO_PIN
 MOSI  <-->  PA7 <-->  BOARD_SPI1_MOSI_PIN
 =========================================================


 */
//==eth_stm
/*
 modified 12 Aug 2013
 by Soohwan Kim (suhwan@wiznet.co.kr)
*/



//DhcpClass _dhcp;

#if defined(WIZ550io_WITH_MACADDRESS)
int W5500_Config(void)
{
  byte mac_address[6] ={0,};

  // Initialise the basic info
  W5500_ResetAndConfigBuf();
  W5500_setIPAddress(IPAddress(0,0,0,0).raw_address());
  W5500_getMACAddress(mac_address);

  // Now try to get our config info from a DHCP server
  int ret = _dhcp.beginWithDHCP(mac_address);
  if(ret == 1)
  {
    // We've successfully found a DHCP server and got our configuration info, so set things
    // accordingly
    W5500_setIPAddress(_dhcp.getLocalIp().raw_address());
    W5500_setGatewayIp(_dhcp.getGatewayIp().raw_address());
    W5500_setSubnetMask(_dhcp.getSubnetMask().raw_address());
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
	W5500_ResetAndConfigBuf();
  W5500_setIPAddress(local_ip.raw_address());
  W5500_setGatewayIp(gateway.raw_address());
  W5500_setSubnetMask(subnet.raw_address());
  _dnsServerAddress = dns_server;
}
#endif

void W5500_begin()//unsigned char *mac, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet)
{
	unsigned char retMacStr[6];
	IPAddress retIP;

	while(1){
		if(W5500_ResetAndConfigBuf()){
			break; //Link up
		}else{
			delayms(100);
			printf("Connect UTP cable\r\n");
		}
	}
	W5500_setMACAddress(W5500_Manager.myMac);


/*
  // Now try to get our config info from a DHCP server
  int ret = _dhcp.beginWithDHCP(mac_address);
  if(ret == 1)
  {
    // We've successfully found a DHCP server and got our configuration info, so set things
    // accordingly
    W5500_setIPAddress(_dhcp.getLocalIp().raw_address());
    W5500_setGatewayIp(_dhcp.getGatewayIp().raw_address());
    W5500_setSubnetMask(_dhcp.getSubnetMask().raw_address());
    _dnsServerAddress = _dhcp.getDnsServerIp();
  }
*/
	W5500_setIPAddress(&W5500_Manager.myIP);
	W5500_setGatewayIp(&W5500_Manager.myDefaultGW);
	W5500_setSubnetMask(&W5500_Manager.mySubnetMask);
	//_dnsServerAddress = dns_server;

	//For Check Only
	W5500_getMACAddress(retMacStr);
	printf("MAC addr = %02x%02x%02x:%02x%02x%02x\r\n",retMacStr[0],retMacStr[1],retMacStr[2],retMacStr[3],retMacStr[4],retMacStr[5]);
	W5500_getIPAddress(&retIP);
	printf("myIP = %u.%u.%u.%u\r\n",retIP.bytes[0],retIP.bytes[1],retIP.bytes[2],retIP.bytes[3]);
	W5500_getGatewayIp(&retIP);
	printf("myDefGW = %u.%u.%u.%u\r\n",retIP.bytes[0],retIP.bytes[1],retIP.bytes[2],retIP.bytes[3]);
	W5500_getSubnetMask(&retIP);
	printf("mySubnetMask = %u.%u.%u.%u\r\n",retIP.bytes[0],retIP.bytes[1],retIP.bytes[2],retIP.bytes[3]);

}
/*
int W5500_Manager_maintain(){
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
        W5500_setIPAddress(_dhcp.getLocalIp().raw_address());
        W5500_setGatewayIp(_dhcp.getGatewayIp().raw_address());
        W5500_setSubnetMask(_dhcp.getSubnetMask().raw_address());
        _dnsServerAddress = _dhcp.getDnsServerIp();
        break;
      default:
        //this is actually a error, it will retry though
        break;
    }
  }
  return rc;
}

IPAddress W5500_Manager_localIP()
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

IPAddress W5500_Manager_gatewayIP()
{
  IPAddress ret;
  W5500_getGatewayIp(ret.raw_address());
  return ret;
}

IPAddress W5500_Manager_dnsServerIP()
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
int W5500_Client_connect_byIP(IPAddress ip, unsigned short port) {
	int i;
  if (W5500_Manager.client._sockfd != MAX_SOCK_NUM)
    return 0;

  for (i = 0; i < MAX_SOCK_NUM; i++) {
    unsigned char s = W5500_readSnSR(i);
    if (s == SnSR_CLOSED || s == SnSR_FIN_WAIT || s == SnSR_CLOSE_WAIT) {
    	W5500_Manager.client._sockfd = i;
      break;
    }
  }

  if (W5500_UDP._sockfd == MAX_SOCK_NUM)
    return 0;

  W5500_Manager.client._srcport++;
  if (W5500_Manager.client._srcport == 0) W5500_Manager.client._srcport = 1024;
  W5500_socket(W5500_Manager.client._sockfd, SnMR_TCP, W5500_Manager.client._srcport, 0);

  if (!W5500_connect(W5500_Manager.client._sockfd, rawIPAddress(ip), port)) {
	  W5500_Manager.client._sockfd = MAX_SOCK_NUM;
    return 0;
  }

  while (W5500_Get_Client_status() != SnSR_ESTABLISHED) {
    delay(1);
    if (W5500_Get_Client_status() == SnSR_CLOSED) {
    	W5500_Manager.client._sockfd = MAX_SOCK_NUM;
      return 0;
    }
  }

  return 1;
}
/*
size_t W5500_Client_write(unsigned char b) {
  return write(&b, 1);
}
*/
size_t W5500_Client_write(const unsigned char *buf, size_t size) {
  if (W5500_Manager.client._sockfd == MAX_SOCK_NUM) {
    setWriteError();
    return 0;
  }
  if (!W5500_socket_send(W5500_Manager.client._sockfd, buf, size)) {
    setWriteError();
    return 0;
  }
  return size;
}

int W5500_Client_Is_Available() {
  if (W5500_Manager.client._sockfd != MAX_SOCK_NUM)
    return W5500_getRXReceivedSize(W5500_Manager.client._sockfd);
  return 0;
}

int W5500_Client_read_Single() {
  unsigned char b;
  if ( W5500_socket_recv(W5500_Manager.client._sockfd, &b, 1) > 0 )
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

int W5500_Client_read(unsigned char *buf, size_t size) {
  return W5500_socket_recv(W5500_Manager.client._sockfd, buf, size);
}

int W5500_Client_peek() {
  unsigned char b;
  // Unlike recv, peek doesn't check to see if there's any data available, so we must
  if (!W5500_Client_Is_Available())
    return -1;
  W5500_peek(W5500_Manager.client._sockfd, &b);
  return b;
}

void W5500_Client_flush() {
  W5500_flush(W5500_Manager.client._sockfd);
}

unsigned char W5500_Get_Client_status() {
	unsigned char ClientStatus;
	if (W5500_Manager.client._sockfd == MAX_SOCK_NUM) return SnSR_CLOSED;

	ClientStatus =W5500_readSnSR(W5500_Manager.client._sockfd);

	return ClientStatus;
}

void W5500_Client_stop() {
  if (W5500_Manager.client._sockfd == MAX_SOCK_NUM)
    return;

  // attempt to close the W5500_connection gracefully (W5500_socket_send a FIN to other side)
  W5500_socket_disconnect(W5500_Manager.client._sockfd);
  //unsigned long start = millis();

  // wait a second for the W5500_connection to close
  //while (W5500_Get_Client_status() != SnSR_CLOSED && millis() - start < 1000)
  //     delayms(1);
  delayms(100);

  // if it hasn't closed, close it forcefully
  if (W5500_Get_Client_status() != SnSR_CLOSED)
	  W5500_socket_close(W5500_Manager.client._sockfd);

  W5500_Manager.server_port[W5500_Manager.client._sockfd] = 0; //W5500_Manager__server_port[W5500_Manager.client._sockfd] = 0;
  W5500_Manager.client._sockfd = MAX_SOCK_NUM;

  printf("Close Sock.\r\n");
}

unsigned char W5500_Client_Is_Connected() {
  if (W5500_Manager.client._sockfd == MAX_SOCK_NUM) return 0;

  unsigned char s = W5500_Get_Client_status();
  return !(s == SnSR_LISTEN || s == SnSR_CLOSED || s == SnSR_FIN_WAIT ||
    (s == SnSR_CLOSE_WAIT && ! W5500_Client_Is_Available()));
}

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
//================== server ================
void W5500_Server_begin(unsigned short listen_port){
	int sock;
	W5500_Manager.server._port = listen_port;

	for(sock = 0; sock < MAX_SOCK_NUM; sock++){
		W5500_Manager.client._sockfd = sock;
		if(W5500_Get_Client_status(sock) == SnSR_CLOSED){
			W5500_socket(sock,SnMR_TCP,listen_port,0);
			W5500_socket_listen(sock);
			W5500_Manager.server_port[sock] = listen_port;
		}
	}
}
int count = 0;
int W5500_ScanGET(char *rxbuf, int rxlen){
	int i;
	int len = strlen("GET / HTTP/1.1\r\n");
	for(i=0;i<(rxlen - len);i++){
		if(strncmp(rxbuf+i,"GET / HTTP/1.1\r\n",len) == 0)
			return 1;
	}
	return 0;
}
void W5500_Server_listenForClients(){
	//struct _W5500_Client client;
	int client;
	char c;
	int sock=0;
	char sendbuf[100];
	int rxlen;
	char rxbuf[512];

	client = W5500_Server_Check_ClientAvailable();
	if(client){
		printf("Svr> Got a client\r\n");
		while(W5500_Client_Is_Connected()){
			if(W5500_Client_Is_Available()){
				rxlen = W5500_Client_read(rxbuf,512);
				if(rxlen == 0) continue;
				printf("Got %u lines from the client.\r\n");

				if(W5500_ScanGET(rxbuf, rxlen)){ //On receiving GET Method
					//send resp
					W5500_socket_send(sock,"HTTP/1/1 200 OK\r\n",strlen("HTTP/1/1 200 OK\r\n"));
					W5500_socket_send(sock,"Content-Type: text/html\r\n",strlen("Content-Type: text/html\r\n"));
					W5500_socket_send(sock,"\r\n", 2);
					sprintf(sendbuf,">>> HELLO KONG WEB SVR WITH W5500 (%u) <<<\r\n", count++);
					W5500_socket_send(sock,sendbuf,strlen(sendbuf));
					break;
				}
			}
		}
		delayms(1);
		W5500_Client_stop(); //close this connection.
	}
}
//========= main =============================
void W5500_loop()
{
	//================== CONFIG ==================================
	  // Enter a MAC address for your controller below.
	  // Newer Ethernet shields have a MAC address printed on a sticker on the shield

		W5500_Manager.myMac[0] = 0xAA;		W5500_Manager.myMac[1] = 0xBB; W5500_Manager.myMac[2] = 0xCC;
		W5500_Manager.myMac[3] = 0xDD;		W5500_Manager.myMac[4] = 0xEE; W5500_Manager.myMac[5] = 0xFF;

		W5500_Manager.myIP.bytes[0] = 192;
		W5500_Manager.myIP.bytes[1] = 168;
		W5500_Manager.myIP.bytes[2] = 10;
		W5500_Manager.myIP.bytes[3] = 3;

		W5500_Manager.serverIP.bytes[0] = 192;
		W5500_Manager.serverIP.bytes[1] = 168;
		W5500_Manager.serverIP.bytes[2] = 10;
		W5500_Manager.serverIP.bytes[3] = 2;  // numeric IP for Serasidis.gr (no DNS)

		W5500_Manager.mySubnetMask.bytes[0] = 255;
		W5500_Manager.mySubnetMask.bytes[1] = 255;
		W5500_Manager.mySubnetMask.bytes[2] = 255;
		W5500_Manager.mySubnetMask.bytes[3] = 0;

		W5500_Manager.myDefaultGW.bytes[0] = 192;
		W5500_Manager.myDefaultGW.bytes[1] = 168;
		W5500_Manager.myDefaultGW.bytes[2] = 10;
		W5500_Manager.myDefaultGW.bytes[3] = 1;

		//IPAddress myDNS = {192,168,10,2};

		//== SPI CONFIG ==
		stmW5500_SpiConfig(); //MSBFIRST, SPI_MODE0, SPI.setClockDivider(SPI_CLOCK_DIV4); ..

		//== BEGIN
		W5500_begin();//myMac, myIP, myDNS, myDefaultGW, mySubnetMask);
		W5500_Server_begin(80); //for HTTP Server.

		while(1){
			W5500_Server_listenForClients();
		}

}
/* Client ===================================================
void W5500_loop()
{
	//BitOrder(MSBFIRST);
	//SPI.setDataMode(SPI_MODE0);
	//SPI.setClockDivider(SPI_CLOCK_DIV4);
	stmW5500_SpiConfig();

	//unsigned char *mac, IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet)
	W5500_begin(&myMac, myIP, myDNS, myDefaultGW, mySubnetMask);
  // if there are incoming bytes available from the server, read them and print them:

  if (W5500_Client_Is_Available()) {
    char c = W5500_Client_read_Single();
    printf(c);
  }
  // if the server's W5500_socket_disconnected, stop the client:
  if (!W5500_Client_Is_Connected()) {
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

  W5500_socket_close(W5500_UDP._sockfd);

  W5500_Manager__server_port[W5500_UDP._sockfd] = 0;
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
  return W5500_startUDP(W5500_UDP._sockfd, rawIPAddress(ip), port);
}

int W5500_UDP_endPacket()
{
  return W5500_socket_sendUDP(W5500_UDP._sockfd);
}

//size_t W5500_UDP_write(unsigned char byte){
//  return write(&byte, 1);
//}

size_t W5500_UDP_write(const unsigned char *buffer, size_t size)
{
  unsigned short bytes_written = W5500_bufferingDataForSend(W5500_UDP._sockfd, W5500_UDP._offset, buffer, size);
  W5500_UDP._offset += bytes_written;
  return bytes_written;
}

int W5500_UDP_parsePacket()
{
  // discard any remaining bytes in the last packet
	W5500_Client_flush();//flush();

  if (W5500_getRXReceivedSize(W5500_UDP._sockfd) > 0)
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
