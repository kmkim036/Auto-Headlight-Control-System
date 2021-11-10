/*
  stmEncEthernet.c
 * 	  SPI : 1Mbps, 8 bit mode
 *    Connections to ENC28J60 Ethernet/SPI:
// SPI MODE = 0
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0CLK - PA2
//! - SSI0Fss - PA3 nCS0
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5

 *  Configures the SPI3 Peripheral.
 *  CLK -  PC10
 *  MISO - PC11
 *  MOSI - PC12
 *  nCS0 - PA15
*/

/* ENC28J60 is 3.3V. SPI Mode=0, 8 bit
 * Memory consists of 3-parts : Control Regs(00~1F), EtherBuffer(0000~1FFF), PHY regs(0~1F).
 */
#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6)|| (PROCESSOR == PROCESSOR_STM32F103RCT6)  || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "ySpiDrv.h"
extern void stmSpi2_Config(unsigned char nCS);
extern unsigned char stmSpi2RdByte();
extern void stmSpi2WrByte(unsigned char inbyte);
//use nCS0 of PB12 : 103
#define nCS28_H nCS0_H
#define nCS28_L nCS0_L

#elif (PROCESSOR == PROCESSOR_STM32F401RET6)
extern void stmSPI1_Config(unsigned sckMbps,unsigned char nCS, unsigned short spiMode, unsigned char data8or16);
extern unsigned char stmSpi1RdByte();
extern void stmSpi1WrByte(unsigned char inbyte);
#else

#endif

// Microchip ENC28J60 Ethernet Interface Driver

//ENC28J60 Memory Map
// 3 types of memory = {control registers, EthernetBuffer, PHY registers}
// ENC28J60 PHY Registers(00..1F)  -- Can be indirectly accessed through MII
// ENC28J60 Control Registers      -- Can be access by SPI

// Many registers in this chip
// They are grouped in 4 banks
// Each control register consists of a combination of {address, bank number, Ethernet/MAC/MII(PHY) indicator} bits.
// - Register address        (bits 0-4)
// - Bank number             (bits 5-6)
// - MAC/PHY(MII) indicator  (bit 7)
#define ADDR_MASK        0b00011111 //0x1F //(bits 0-4)
#define BANK_MASK        0b01100000 //0x60
#define SPRD_MASK        0b10000000 //0x80

// All-bank registers (0,1,2,3 banks)
#define EIE              0x1B //Ethernet Interrupt Enable (Reg12-2)
#define EIR              0x1C //Ethernet Interrupt Request (Reg12-3)
#define ESTAT            0x1D //Ethernet Status Reg(Reg12-1) : INT|LateCollision|RxBusy|TxAbortErr|ClockReady
#define ECON2            0x1E //Ethernet Control Reg2(Reg3-2) : AutoINC|PakcetDecrement|PowerSaveEn|VoltageRegulatorPowerSaveEn
#define ECON1            0x1F //Ethernet Control Reg1(Reg3-1) :TxReset|RxReset|DMAstatus|DMAcksumEn|TxRTS| RxEn | BankSelectBits

// Bank 0 registers      (RegAddr | bank | MACorMII)
#define ERDPT           (0x00|0x00)
#define EWRPT           (0x02|0x00)
#define ETXST           (0x04|0x00)
#define ETXND           (0x06|0x00)
#define ERXST           (0x08|0x00)
#define ERXND           (0x0A|0x00)
#define ERXRDPT         (0x0C|0x00)
// #define ERXWRPT         (0x0E|0x00)
#define EDMAST          (0x10|0x00)
#define EDMAND          (0x12|0x00)
// #define EDMADST         (0x14|0x00)
#define EDMACS          (0x16|0x00)
// Bank 1 registers
#define EHT0             (0x00|0x20)
#define EHT1             (0x01|0x20)
#define EHT2             (0x02|0x20)
#define EHT3             (0x03|0x20)
#define EHT4             (0x04|0x20)
#define EHT5             (0x05|0x20)
#define EHT6             (0x06|0x20)
#define EHT7             (0x07|0x20)
#define EPMM0            (0x08|0x20)
#define EPMM1            (0x09|0x20)
#define EPMM2            (0x0A|0x20)
#define EPMM3            (0x0B|0x20)
#define EPMM4            (0x0C|0x20)
#define EPMM5            (0x0D|0x20)
#define EPMM6            (0x0E|0x20)
#define EPMM7            (0x0F|0x20)
#define EPMCS           (0x10|0x20)
// #define EPMO            (0x14|0x20)
#define EWOLIE           (0x16|0x20)
#define EWOLIR           (0x17|0x20)
#define ERXFCON          (0x18|0x20)
#define EPKTCNT          (0x19|0x20)
// Bank 2 registers -- PHY(MII) Registers
#define MACON1           (0x00|0x40|0x80)
#define MACON2           (0x01|0x40|0x80)
#define MACON3           (0x02|0x40|0x80)
#define MACON4           (0x03|0x40|0x80)
#define MABBIPG          (0x04|0x40|0x80)
#define MAIPG           (0x06|0x40|0x80)
#define MACLCON1         (0x08|0x40|0x80)
#define MACLCON2         (0x09|0x40|0x80)
#define MAMXFL          (0x0A|0x40|0x80)
#define MAPHSUP          (0x0D|0x40|0x80)
#define MICON            (0x11|0x40|0x80)
#define MICMD            (0x12|0x40|0x80)
#define MIREGADR         (0x14|0x40|0x80)
#define MIWR            (0x16|0x40|0x80)
#define MIWRL            (0x16|0x40|0x80)
#define MIWRH            (0x17|0x40|0x80)
#define MIRD            (0x18|0x40|0x80)
#define MIRDL            (0x18|0x40|0x80)
#define MIRDH            (0x19|0x40|0x80)
// Bank 3 registers -- PHY(MII) Registers and Others
#define MAADR1           (0x00|0x60|0x80)
#define MAADR0           (0x01|0x60|0x80)
#define MAADR3           (0x02|0x60|0x80)
#define MAADR2           (0x03|0x60|0x80)
#define MAADR5           (0x04|0x60|0x80)
#define MAADR4           (0x05|0x60|0x80)
#define EBSTSD           (0x06|0x60)
#define EBSTCON          (0x07|0x60)
#define EBSTCS          (0x08|0x60)
#define MISTAT           (0x0A|0x60|0x80)
#define EREVID           (0x12|0x60)
#define ECOCON           (0x15|0x60)
#define EFLOCON          (0x17|0x60)
#define EPAUS           (0x18|0x60)

// ENC28J60 ERXFCON Register Bit Definitions
#define ERXFCON_UCEN     0x80
#define ERXFCON_ANDOR    0x40
#define ERXFCON_CRCEN    0x20
#define ERXFCON_PMEN     0x10
#define ERXFCON_MPEN     0x08
#define ERXFCON_HTEN     0x04
#define ERXFCON_MCEN     0x02
#define ERXFCON_BCEN     0x01
// ENC28J60 EIE Register Bit Definitions
#define EIE_INTIE        0x80
#define EIE_PKTIE        0x40
#define EIE_DMAIE        0x20
#define EIE_LINKIE       0x10
#define EIE_TXIE         0x08
#define EIE_WOLIE        0x04
#define EIE_TXERIE       0x02
#define EIE_RXERIE       0x01
// ENC28J60 EIR Register Bit Definitions
#define EIR_PKTIF        0x40
#define EIR_DMAIF        0x20
#define EIR_LINKIF       0x10
#define EIR_TXIF         0x08
#define EIR_WOLIF        0x04
#define EIR_TXERIF       0x02
#define EIR_RXERIF       0x01
// ENC28J60 ESTAT Register Bit Definitions
#define ESTAT_INT        0x80
#define ESTAT_LATECOL    0x10
#define ESTAT_RXBUSY     0x04
#define ESTAT_TXABRT     0x02
#define ESTAT_CLKRDY     0x01
// ENC28J60 ECON2 Register Bit Definitions
#define ECON2_AUTOINC    0x80
#define ECON2_PKTDEC     0x40
#define ECON2_PWRSV      0x20
#define ECON2_VRPS       0x08
// ENC28J60 ECON1 Register Bit Definitions
#define ECON1_TXRST      0x80
#define ECON1_RXRST      0x40
#define ECON1_DMAST      0x20
#define ECON1_CSUMEN     0x10
#define ECON1_TXRTS      0x08
#define ECON1_RXEN       0x04
#define ECON1_BSEL1      0x02
#define ECON1_BSEL0      0x01
// ENC28J60 MACON1 Register Bit Definitions
#define MACON1_LOOPBK    0x10
#define MACON1_TXPAUS    0x08
#define MACON1_RXPAUS    0x04
#define MACON1_PASSALL   0x02
#define MACON1_MARXEN    0x01
// ENC28J60 MACON2 Register Bit Definitions
#define MACON2_MARST     0x80
#define MACON2_RNDRST    0x40
#define MACON2_MARXRST   0x08
#define MACON2_RFUNRST   0x04
#define MACON2_MATXRST   0x02
#define MACON2_TFUNRST   0x01
// ENC28J60 MACON3 Register Bit Definitions
#define MACON3_PADCFG2   0x80
#define MACON3_PADCFG1   0x40
#define MACON3_PADCFG0   0x20
#define MACON3_TXCRCEN   0x10
#define MACON3_PHDRLEN   0x08
#define MACON3_HFRMLEN   0x04
#define MACON3_FRMLNEN   0x02
#define MACON3_FULDPX    0x01
// ENC28J60 MICMD Register Bit Definitions
#define MICMD_MIISCAN    0x02
#define MICMD_MIIRD      0x01
// ENC28J60 MISTAT Register Bit Definitions
#define MISTAT_NVALID    0x04
#define MISTAT_SCAN      0x02
#define MISTAT_BUSY      0x01

// ENC28J60 EBSTCON Register Bit Definitions
#define EBSTCON_PSV2     0x80
#define EBSTCON_PSV1     0x40
#define EBSTCON_PSV0     0x20
#define EBSTCON_PSEL     0x10
#define EBSTCON_TMSEL1   0x08
#define EBSTCON_TMSEL0   0x04
#define EBSTCON_TME      0x02
#define EBSTCON_BISTST    0x01

// PHY registers
#define PHCON1           0x00
#define PHSTAT1          0x01
#define PHID1           0x02
#define PHID2           0x03
#define PHCON2           0x10
#define PHSTAT2          0x11
#define PHIE             0x12
#define PHIR             0x13
#define PHLCON           0x14

// ENC28J60 PHY PHCON1 Register Bit Definitions
#define PHCON1_PRST      0x8000
#define PHCON1_PLOOPBK   0x4000
#define PHCON1_PPWRSV    0x0800
#define PHCON1_PDPXMD    0x0100
// ENC28J60 PHY PHSTAT1 Register Bit Definitions
#define PHSTAT1_PFDPX    0x1000
#define PHSTAT1_PHDPX    0x0800
#define PHSTAT1_LLSTAT   0x0004
#define PHSTAT1_JBSTAT   0x0002
// ENC28J60 PHY PHCON2 Register Bit Definitions
#define PHCON2_FRCLINK   0x4000
#define PHCON2_TXDIS     0x2000
#define PHCON2_JABBER    0x0400
#define PHCON2_HDLDIS    0x0100
// END28J60 PHY PHLCON(0x14) Register Bit Definitions


// ENC28J60 Packet Control u8 Bit Definitions
#define PKTCTRL_PHUGEEN  0x08
#define PKTCTRL_PPADEN   0x04
#define PKTCTRL_PCRCEN   0x02
#define PKTCTRL_POVERRIDE 0x01

// SPI operation codes (See 4.2 SPI Instruction Set)
#define ENC28J60_SPI_READ_CTRL_REG       0x00 //RCR
#define ENC28J60_SPI_READ_BUF_MEM        0x3A //RBM
#define ENC28J60_SPI_WRITE_CTRL_REG      0x40 //WCR
#define ENC28J60_SPI_WRITE_BUF_MEM       0x7A //WBM
#define ENC28J60_SPI_BIT_FIELD_SET       0x80 //BFS
#define ENC28J60_SPI_BIT_FIELD_CLR       0xA0 //BFC
#define ENC28J60_SPI_SOFT_RESET          0xFF //SC

// The RXSTART_INIT must be zero. See Rev. B4 Silicon Errata point 5.
// Buffer boundaries applied to internal 8K ram
// the entire available packet buffer space is allocated

#define RXSTART_INIT        0x0000  // start of RX buffer, room for 2 packets
#define RXSTOP_INIT         (0x1FFF-0x0600-1) //0x0BFF  // end of RX buffer

#define TXSTART_INIT        (0x1FFF-0x0600) //0x0C00  // start of TX buffer, room for 1 packet
#define TXSTOP_INIT         0x1FFF  // end of TX buffer

#define SCRATCH_START       0x1200  // start of scratch area
#define SCRATCH_LIMIT       0x2000  // past end of area, i.e. 3.5 Kb
#define SCRATCH_PAGE_SHIFT  6       // addressing is in pages of 64 bytes
#define SCRATCH_PAGE_SIZE   (1 << SCRATCH_PAGE_SHIFT)

// max frame length which the conroller will accept:
// (note: maximum ethernet frame length would be 1518)
#define MAX_FRAMELEN      1500

#define FULL_SPEED  1   // switch to full-speed SPI for bulk transfers

u32 gEnc28BufSize;
static u8 g8Enc28j60Bank=0;
static int gNextPacketPtr;
static u8 gEtherBuf[1514];
//proto
u8 stmEnc28ReadReg8 (u8 address);
extern void stmSPI2_Config(unsigned sckMbps, unsigned char nCS, unsigned short spiMode, unsigned char data8or16);

void stmEnc28J60EtherSpiConfig(void){
	//2Mbps, 8 bit mode
	//use MODE 0
#if (PROCESSOR == PROCESSOR_STM32F401RET6)
	stmSPI1_Config(2,0,0,8); //use PA15 of nCS0
#else
	stmSPI2_Config(10, 0,0,8); //use PB12 of nCS0
#endif
    nCS28_H;//nCs=1

	printf("ENC28>SPI CONFIG DONE.\r\n");
}

void stmEnc28writeOpAndData (u8 op, u8 address, u8 data) {
    nCS28_L;
#if (PROCESSOR == PROCESSOR_STM32F401RET6)
    stmSpi1WrByte(op | (address & ADDR_MASK)); //issue write command
    stmSpi1WrByte(data);
#else
    stmSpi2WrByte(op | (address & ADDR_MASK)); //issue write command
    stmSpi2WrByte(data);
#endif
    nCS28_H;
}

void stmEnc28SetBank (u8 address) {
    if ((address & BANK_MASK) != g8Enc28j60Bank) { //if not previous set.
        stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_CLR, ECON1, ECON1_BSEL1|ECON1_BSEL0); //Reg ECON1 Bank Selection.
        g8Enc28j60Bank = (address & BANK_MASK);
        stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, ECON1, g8Enc28j60Bank>>5);
    }
}

void stmEnc28WriteReg8 (u8 address, u8 data) {
    stmEnc28SetBank(address);
    stmEnc28writeOpAndData(ENC28J60_SPI_WRITE_CTRL_REG, address, data);
}

void stmEnc28WriteRegLH(u8 address, u16 data) {
    stmEnc28WriteReg8(address, data); //lsb first
    stmEnc28WriteReg8(address + 1, data >> 8); //msb last
}

void stmEnc28WritePhyLH (u8 address, u16 data) {
	stmEnc28WriteReg8(MIREGADR, address); //Set phy reg addr
    //stmEnc28WriteRegLH(MIWR, data); //write data.//stmEnc28WriteReg8(MIWRH, data >> 8);	//stmEnc28WriteReg8(MIWRL, data & 0xff);
	stmEnc28WriteReg8(MIWRL, data);
	stmEnc28WriteReg8(MIWRH, data>>8);
    while (stmEnc28ReadReg8(MISTAT) & MISTAT_BUSY){
    	//delayms(1);
    }
}

u8 stmEnc28ReadOp8 (u8 op, u8 address) {
	 u8 result;
    nCS28_L;
#if (PROCESSOR == PROCESSOR_STM32F401RET6)
    stmSpi1WrByte(op | (address & ADDR_MASK)); 	//issue read command
    result = stmSpi1RdByte(); 					//for ETH Registers
    if (address & 0x80) 						//for MAC and MII Registers, do actual Read.
    	result = stmSpi1RdByte();
#else
    stmSpi2WrByte(op | (address & ADDR_MASK)); 	//issue read command
    result = stmSpi2RdByte(); 					//for ETH Registers
    if (address & 0x80) 						//for MAC and MII Registers, do actual Read.
    	result = stmSpi2RdByte();
#endif
    nCS28_H;

    return result;
}

u8 stmEnc28ReadReg8 (u8 address) {
    stmEnc28SetBank(address); 					//set this new bank
    return stmEnc28ReadOp8(ENC28J60_SPI_READ_CTRL_REG, address);
}

u16 stmEnc28ReadRegLH(u8 address) {
        return stmEnc28ReadReg8(address) + (stmEnc28ReadReg8(address+1) << 8);
}


u8 stmEnc28ReadPhy8 (u8 address) { //was u16
	 stmEnc28WriteReg8(MIREGADR, address); //write addr of the phy reg to read from into the MIREGADR.
	 stmEnc28WriteReg8(MICMD, MICMD_MIIRD); //set MICMD.miird bit
	 delayms(1);
     while (stmEnc28ReadReg8(MISTAT) & MISTAT_BUSY); //wait 10.24usec.
     stmEnc28WriteReg8(MICMD, 0x00); //clear miird bit.

     return stmEnc28ReadReg8(MIRD+1); //MIRDH (MSB ONLY)
}

u16 stmEnc28ReadPhy16 (u8 address) { //was u16
	 stmEnc28WriteReg8(MIREGADR, address); //write addr of the phy reg to read from into the MIREGADR.
	 stmEnc28WriteReg8(MICMD, MICMD_MIIRD); //set MICMD.miird bit
	 delayms(1);
     while (stmEnc28ReadReg8(MISTAT) & MISTAT_BUSY); //wait 10.24usec.
     stmEnc28WriteReg8(MICMD, 0x00); //clear miird bit.
     return(stmEnc28ReadRegLH(MIRD)); //MIRDL and MIRDH
}


//=========================================
void stmEnc28ReadBuf(u16 len, u8* data) {
    nCS28_L;//enableChip();
#if (PROCESSOR == PROCESSOR_STM32F401RET6)
    stmSpi1WrByte(ENC28J60_SPI_READ_BUF_MEM);
    while (len) {
    	len--;
        *data = stmSpi1RdByte(); //read data
        data++;
    }
    *data = '\0';
#else
    stmSpi2WrByte(ENC28J60_SPI_READ_BUF_MEM);
    while (len) {
    	len--;
        *data = stmSpi2RdByte(); //read data
        data++;
    }
    *data = '\0';
#endif

    nCS28_H;
}

 void stmEnc28WriteBuf(u16 len, u8* data) {
    nCS28_L;
#if (PROCESSOR == PROCESSOR_STM32F401RET6)
    stmSpi1WrByte(ENC28J60_SPI_WRITE_BUF_MEM);
    while (len){
    	len--;
        stmSpi1WrByte(*data);
        data++;
    }
#else
    stmSpi2WrByte(ENC28J60_SPI_WRITE_BUF_MEM);
    while (len){
    	len--;
        stmSpi2WrByte(*data);
        data++;
    }
#endif

    nCS28_H;
}

//derived from 25MHz phy clk.
//001 = divide by 1 = 25MHz
//004 = divide by 4 = 6.25MHz (Default)
void stmENC28J60clkOut(u8 clk) {
	stmEnc28WriteReg8(ECOCON, clk & 0x7);
	printf("ENC28> ENABLE CLOCK OUT.\r\n");
}

boolean stmENC28J60isLinkUp() {
	int timeout=0;
	for(timeout=0; timeout < 100; timeout++){
		if(stmEnc28ReadPhy8(PHSTAT2) & 4){ //bit 10 (=bit 3 in upper reg)
			printf("ENC28> LINK UP.\r\n");
			return 1;
		}
		delayms(10);
	}
	printf("ERR> Link is still down\r\n");
    return 0;
}
/*
#define PHCON1           0x00
#define PHSTAT1          0x01
#define PHID1           0x02
#define PHID2           0x03
#define PHCON2           0x10
#define PHSTAT2          0x11
#define PHIE             0x12
#define PHIR             0x13
#define PHLCON           0x14
 */
void stmENC28J60ShowPhyRegs() {
	printf(">>>PHY REGISTERS<<\r\n");
	printf("PHCON1 : PHY CONTROL REG1. \t= %04x\r\n",stmEnc28ReadPhy16(PHCON1));
	printf("PHSTAT1: PHY STATUS  REG1. \t= %04x\r\n",stmEnc28ReadPhy16(PHSTAT1));
	printf("PHID1  : PHY ID      REG1. \t= %04x(SHOULD BE EQUAL TO 0x0083)\r\n",stmEnc28ReadPhy16(PHID1));
	printf("PHID2  : PHY ID      REG2. \t= %04x\r\n",stmEnc28ReadPhy16(PHID2));
	printf("PHCON2 : PHY CONTROL REG2. \t= %04x\r\n",stmEnc28ReadPhy16(PHCON2));
	printf("PHSTAT2: PHY STATUS  REG2. \t= %04x\r\n",stmEnc28ReadPhy16(PHSTAT2));
	printf("PHIE   : PHY INTR EN REG.  \t= %04x\r\n",stmEnc28ReadPhy16(PHIE));
	printf("PHIR   : PHY INTR RQ REG.  \t= %04x\r\n",stmEnc28ReadPhy16(PHIR));
	printf("PHLCON : PHY LED CON REG.  \t= %04x\r\n",stmEnc28ReadPhy16(PHLCON));
	printf("\r\n");

    return 0;
}

void stmENC28J60packetSend(u8 *txbuf, u16 len) {

	//Check no tx in progress
    while (stmEnc28ReadOp8(ENC28J60_SPI_READ_CTRL_REG, ECON1) & ECON1_TXRTS){
        if (stmEnc28ReadReg8(EIR) & EIR_TXERIF) {
            stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, ECON1, ECON1_TXRST);
            stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
        }
    }

    stmEnc28WriteRegLH(EWRPT, TXSTART_INIT); 		//set write pointer to start of tx buf area
    stmEnc28WriteRegLH(ETXND, TXSTART_INIT+len); 	//set TXND pointer to correspond to the packet size given
    stmEnc28writeOpAndData(ENC28J60_SPI_WRITE_BUF_MEM, 0, 0x00); //write per-packet control byte(0x00 == use macon3 settings)
    stmEnc28WriteBuf(len, txbuf); 					//copy
    stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, ECON1, ECON1_TXRTS);//send the contents of tx buf onto the network
}

void stmENC28J60IpHandler(u16 iplen){
	printf("IP Handler...\r\n");
}
void stmENC28J60ArpHandler(u16 arplen){
	printf("ARP Handler...\r\n");
}
void stmENC28J60EthParser(u16 frmlen){
	if(frmlen <60) //sanity check
		printf("Enc28> Too short Rx Frm(Len=%d)\r\n",frmlen);
	else{
		printf("Enc28> Rx Frm Len=%d, ",frmlen);

		if(gEtherBuf[12]== 0x08){ //check EtherType.
			if(gEtherBuf[13]== 0x00){
				printf("Ethetype=IP\r\n");
				stmENC28J60IpHandler(frmlen - 14);
			}
			else if(gEtherBuf[13]== 0x06) {
				printf("Ethetype=ARP\r\n");
				stmENC28J60ArpHandler(frmlen - 14);
			}
		}
	}
}

// Gets a packet from the network receive buffer, if one is available.
// The packet will by headed by an ethernet header.
//      maxlen  The maximum acceptable length of a retrieved packet.
//      packet  Pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
//unsigned int enc28j60PacketReceive(unsigned int maxlen, unsigned char* packet)
u16 stmENC28J60packetReceive(unsigned int maxlen, unsigned char* packet) {
    u16 len = 0;

    if (stmEnc28ReadReg8(EPKTCNT) > 0) { //Rcvd?

        stmEnc28WriteRegLH(ERDPT, gNextPacketPtr);

        //Parsing the received frame.
        struct {
            u16 nextPacket;
            u16 byteCount;
            u16 status;
        } header; //FIFO Header Information. Not the real ethernet header field.

        stmEnc28ReadBuf(sizeof header, (u8*) &header); //Move FIFO information field. Not the ethernet header.

        gNextPacketPtr  = header.nextPacket;
        len = header.byteCount - 4; //remove the CRC count
        if (len>gEnc28BufSize-1)
            len=gEnc28BufSize-1;
        if ((header.status & 0x80)==0)
            len = 0;
        else
        	stmEnc28ReadBuf(len, packet);//stmEnc28ReadBuf(len, gEtherBuf); //Move FIFO into the local gEtherBuf Memory.

        gEtherBuf[len] = 0; //Fill the last with 0x00.

        //Parsing the ethernet frame.
        stmENC28J60EthParser(len);

        //At the last, prepare the next frame.
        if (gNextPacketPtr - 1 > RXSTOP_INIT)
            stmEnc28WriteRegLH(ERXRDPT, RXSTOP_INIT);
        else
            stmEnc28WriteRegLH(ERXRDPT, gNextPacketPtr - 1);
        stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
    }
    return len;
}

void stmENC28J60copyout (u8 page, const u8* data) {
    u16 destPos = SCRATCH_START + (page << SCRATCH_PAGE_SHIFT);
    if (destPos < SCRATCH_START || destPos > SCRATCH_LIMIT - SCRATCH_PAGE_SIZE)
        return;
    stmEnc28WriteRegLH(EWRPT, destPos);
    stmEnc28WriteBuf(SCRATCH_PAGE_SIZE, data);
}

void stmENC28J60copyin (u8 page, u8* data) {
    u16 destPos = SCRATCH_START + (page << SCRATCH_PAGE_SHIFT);
    if (destPos < SCRATCH_START || destPos > SCRATCH_LIMIT - SCRATCH_PAGE_SIZE)
        return;
    stmEnc28WriteRegLH(ERDPT, destPos);
    stmEnc28ReadBuf(SCRATCH_PAGE_SIZE, data);
}

u8 stmENC28J60peekin (u8 page, u8 off) {
    u8 result = 0;
    u16 destPos = SCRATCH_START + (page << SCRATCH_PAGE_SHIFT) + off;
    if (SCRATCH_START <= destPos && destPos < SCRATCH_LIMIT) {
        stmEnc28WriteRegLH(ERDPT, destPos);
        stmEnc28ReadBuf(1, &result);
    }
    return result;
}

void stmENC28J60powerDown() { //Enter to Sleep Mode
    stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_CLR, ECON1, ECON1_RXEN);
    while(stmEnc28ReadReg8(ESTAT) & ESTAT_RXBUSY);
    while(stmEnc28ReadReg8(ECON1) & ECON1_TXRTS);
    stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, ECON2, ECON2_VRPS);
    stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, ECON2, ECON2_PWRSV);
}

void stmENC28J60powerUp() {
    stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_CLR, ECON2, ECON2_PWRSV);
    while(!stmEnc28ReadReg8(ESTAT) & ESTAT_CLKRDY);
    stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}
// Functions to enable/disable broadcast filter bits (Rx)
// With the bit set, broadcast packets are filtered.
void stmEnc28J60enableBroadcast () {
	stmEnc28WriteReg8(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN|ERXFCON_BCEN);
}

void stmENC28J60disableBroadcast () {
	stmEnc28WriteReg8(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
}

void stmENC28J60disableMulticast() { // disable multicast filter , enable multicast reception
	stmEnc28WriteReg8(ERXFCON, ERXFCON_CRCEN);
}
/*
#define RANDOM_FILL                0b0000
#define ADDRESS_FILL        0b0100
#define PATTERN_SHIFT        0b1000
#define RANDOM_RACE                0b1100

u8 ENC28J60doBIST ( u8 csPin) {
    u16 macResult;
    u16 bitsResult;

    nCS28_H;//disableChip();

    stmEnc28writeOpAndData(ENC28J60_SPI_SOFT_RESET, 0, ENC28J60_SPI_SOFT_RESET);
    somedelay(2); // errata B7/2

    while (!stmEnc28ReadOp8(ENC28J60_SPI_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY) ;
        // now we can start the memory test
        // clear some of the registers registers
    stmEnc28WriteReg8(ECON1, 0);
        stmEnc28WriteRegLH(EDMAST, 0);

        // Set up necessary pointers for the DMA to calculate over the entire memory
        stmEnc28WriteRegLH(EDMAND, 0x1FFFu);
        stmEnc28WriteRegLH(ERXND, 0x1FFFu);

        // Enable Test Mode and do an Address Fill
        stmEnc28SetBank(EBSTCON);
        stmEnc28WriteReg8(EBSTCON, EBSTCON_TME | EBSTCON_BISTST | ADDRESS_FILL);

        // wait for BISTST to be reset, only after that are we actually ready to
        // start the test
        // this was undocumented :(
        while (stmEnc28ReadOp8(ENC28J60_SPI_READ_CTRL_REG, EBSTCON) & EBSTCON_BISTST);
        stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_CLR, EBSTCON, EBSTCON_TME);

        // now start the actual reading an calculating the checksum until the end is
        // reached
        stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, ECON1, ECON1_DMAST | ECON1_CSUMEN);
        stmEnc28SetBank(EDMACS);
        while(stmEnc28ReadOp8(ENC28J60_SPI_READ_CTRL_REG, ECON1) & ECON1_DMAST);
        macResult = stmEnc28ReadRegLH(EDMACS);
        bitsResult = stmEnc28ReadRegLH(EBSTCS);
        // Compare the results
        // 0xF807 should always be generated in Address fill mode
        if ((macResult != bitsResult) || (bitsResult != 0xF807)) {
                return 0;
        }
        // reset test flag
        stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_CLR, EBSTCON, EBSTCON_TME);

        // Now start the BIST with random data test, and also keep on swapping the
        // DMA/BIST memory ports.
        stmEnc28WriteReg8(EBSTSD, 0b10101010 );//| millis()); ////////////////////////////////////???????????????????????
        stmEnc28WriteReg8(EBSTCON, EBSTCON_TME | EBSTCON_PSEL | EBSTCON_BISTST | RANDOM_FILL);

        // wait for BISTST to be reset, only after that are we actually ready to
        // start the test
        // this was undocumented :(
        while (stmEnc28ReadOp8(ENC28J60_SPI_READ_CTRL_REG, EBSTCON) & EBSTCON_BISTST);
        stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_CLR, EBSTCON, EBSTCON_TME);

        // now start the actual reading an calculating the checksum until the end is
        // reached
        stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, ECON1, ECON1_DMAST | ECON1_CSUMEN);
        stmEnc28SetBank(EDMACS);
        while(stmEnc28ReadOp8(ENC28J60_SPI_READ_CTRL_REG, ECON1) & ECON1_DMAST);

        macResult = stmEnc28ReadRegLH(EDMACS);
        bitsResult = stmEnc28ReadRegLH(EBSTCS);
        // The checksum should be equal
        return macResult == bitsResult;
}
*/
u8 stmENC28J60Init (u16 size, const u8* macaddr) {
	u8 rev;
    gEnc28BufSize = size;

	stmEnc28J60EtherSpiConfig();

    nCS28_H;

    //clkout
    stmENC28J60clkOut(1);//1=6MHz

    //SoftReset
    stmEnc28writeOpAndData(ENC28J60_SPI_SOFT_RESET, 0, ENC28J60_SPI_SOFT_RESET); //op | addr, data
    delayms(1); // errata B7/2
    //Wait until Clk Ready.
    //while (!stmEnc28ReadOp8(ENC28J60_SPI_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY) ; //Bank 0, 0x1D

    gNextPacketPtr = RXSTART_INIT;

    //Rx Start
    stmEnc28WriteRegLH(ERXST, RXSTART_INIT);//Rx Start (STL + STH)
    stmEnc28WriteRegLH(ERXRDPT, RXSTART_INIT);//Set Rx Point Address
    stmEnc28WriteRegLH(ERXND, RXSTOP_INIT);//Rx End
    //TX Start
    stmEnc28WriteRegLH(ETXST, TXSTART_INIT);//TX Start
    stmEnc28WriteRegLH(ETXND, TXSTOP_INIT);// Tx End

    stmEnc28J60enableBroadcast(); // change to add ERXFCON_BCEN recommended by epam

    //Do bank 1 stuff. Filter
    stmEnc28WriteReg8(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_PMEN);
    stmEnc28WriteRegLH(EPMM0, 0x303f);
    stmEnc28WriteRegLH(EPMCS, 0xf7f9);

    //Do bank 2 stuff.
    //enable MAC receive
    stmEnc28WriteReg8(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
    //Bring MAC out of reset
    stmEnc28WriteReg8(MACON2, 0x00);
    //Enable automatic padding and CRC, and FDX
    stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, MACON3,MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN | MACON3_FULDPX);
    stmEnc28WriteRegLH(MAIPG, 0x0C12); //IPG (non back-to-back) .. 0x12, then 0x0c
    stmEnc28WriteReg8(MABBIPG, 0x12);  //IPG (back-to-back)
    stmEnc28WriteRegLH(MAMXFL, MAX_FRAMELEN); //Max Frame Length

    //Do bank 3 stuff
    stmEnc28WriteReg8(MAADR5, macaddr[0]);
    stmEnc28WriteReg8(MAADR4, macaddr[1]);
    stmEnc28WriteReg8(MAADR3, macaddr[2]);
    stmEnc28WriteReg8(MAADR2, macaddr[3]);
    stmEnc28WriteReg8(MAADR1, macaddr[4]);
    stmEnc28WriteReg8(MAADR0, macaddr[5]);

    stmEnc28WritePhyLH(PHCON1, PHCON1_PDPXMD);
    stmEnc28WritePhyLH(PHCON2, PHCON2_HDLDIS); //No loopback
    stmEnc28WritePhyLH(PHLCON, 0x34D2); //YOON LED CONFIG - LEDB = LinkStatus and tx/rx activities

    stmEnc28SetBank(ECON1); //return to bank 0
    stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE); //enable interrupts
    stmEnc28writeOpAndData(ENC28J60_SPI_BIT_FIELD_SET, ECON1, ECON1_RXEN); //enable rx.

     rev = stmEnc28ReadReg8(EREVID);
    // Check for correct configuration by read silicon revision code.
    // microchip forgot to step the number on the silcon when they released the revision B7. 6 is now rev B7.
    // We still have to see what they do when they release B8. At the moment there is no B8 out yet
    printf("Enc28> Chip Revision = %d(6==rev.B7)\r\n",rev);
    if (rev > 5) ++rev;
    return rev;
}
void stmEnc28WaitForLinkUp(){
	while(1){
		if(!stmENC28J60isLinkUp()){
			printf("ERROR: LINK DOWN\r\n");
			continue;
		}else{
			printf("LinkUp\r\n");
			break;
		}
		delayms(100);
	}
}
void stmEnc28J60EtherSpiLoop(){
	u16 size,rcvlen;
	u8 myMACAddr[6],i;
	u32 seq;
	unsigned char gEtherBufRx[1514];

	printf("ENC28J60 10Mbps Ethernet with SPI TEST with SPI MODE_0,2Mbps, 8bit.\r\n");


	//myMacAddress
	myMACAddr[0] = 'A'; //0x41
	myMACAddr[1] = 'R';
	myMACAddr[2] = 'A';
	myMACAddr[3] = 'M';
	myMACAddr[4] = 'I';
	myMACAddr[5] = 'I';


	stmENC28J60Init(1514, myMACAddr);

	stmENC28J60ShowPhyRegs();

	//ENC28J60doBIST(0);
	stmEnc28WaitForLinkUp();

    //make a packet
    //DA -- Broadcast Addr
    for(i=0;i<6;i++)
    	gEtherBuf[i] = 0xFF; //Fill with 0xFF.
    //SA
    for(i=0;i<6;i++)
    	gEtherBuf[i+6] = myMACAddr[i];
	gEtherBuf[12] = 0x08;
	gEtherBuf[13] = 0x00;
	//IPHeader
	gEtherBuf[14] = 0x45;
    seq = 0;
	while(1){
		printf(">Send(%u)Etype=%04x\r\n",seq, gEtherBuf[12]*0x100+gEtherBuf[13]);
		for(i=0;i<16;i++)
			printf("%02x ", gEtherBuf[i]);
		printf("\r\n");
		stmENC28J60packetSend(gEtherBuf, 64);
		seq++;
		delayms(100);

		//handle received frames...
		while(stmENC28J60packetReceive(1514,gEtherBufRx));
	}
}
