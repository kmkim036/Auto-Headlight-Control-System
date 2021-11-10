
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"

typedef unsigned char u8;
typedef unsigned char uchar;
typedef unsigned short u16;
typedef unsigned long u32;
typedef u8 boolean;

//UART related
extern void UART0_IRQHandler(void);

/**************************************************************************
I2C Driver for NXP's PN532 NFC/13.56MHz RFID Transceiver
This is a library for the Adafruit PN532 NFC/RFID shields
v1.4
**************************************************************************/
/* Pinouts
I2C SCL : Pin17 : PB2
I2C SDA : Pin18 : PB3
IRQNFC  : Pin15 : PD6
BUZZER : Pin34 : PB1(v4. TBC )
LED : PA7
*/
/*
 * #define IRQ   (2)
#define RESET (3)  // Not connected by default on the NFC Shield
 */
////////////////////GPIO PIN Mapping /////////////////////////////////////////////////
#define IRQ532         ((GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_6) == GPIO_PIN_6) ? 1:0)
#define SPKON532           {GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);}
#define SPKOFF532           {GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, ~GPIO_PIN_1);}
#define LED_RED_ON532   GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
#define LED_RED_OFF532  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, ~GPIO_PIN_7);

#define  CR  {UARTprintf("\r\n");}
// Uncomment these lines to enable debug output for PN532(I2C) and/or MIFARE related code
#define PN532DEBUG
#define MIFAREDEBUG

#define PN532_PREAMBLE                      (0x00)
#define PN532_STARTCODE1                    (0x00)
#define PN532_STARTCODE2                    (0xFF)
#define PN532_POSTAMBLE                     (0x00)

#define PN532_HOSTTOPN532                   (0xD4)
#define PN532_PN532TOHOST                   (0xD5)

// PN532 Commands
#define PN532_COMMAND_DIAGNOSE              (0x00)
#define PN532_COMMAND_GETFIRMWAREVERSION    (0x02)
#define PN532_COMMAND_GETGENERALSTATUS      (0x04)
#define PN532_COMMAND_READREGISTER          (0x06)
#define PN532_COMMAND_WRITEREGISTER         (0x08)
#define PN532_COMMAND_PN532readGPIO         (0x0C)
#define PN532_COMMAND_PN532writeGPIO        (0x0E)
#define PN532_COMMAND_SETSERIALBAUDRATE     (0x10)
#define PN532_COMMAND_SETPARAMETERS         (0x12)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)
#define PN532_COMMAND_POWERDOWN             (0x16)
#define PN532_COMMAND_RFCONFIGURATION       (0x32)
#define PN532_COMMAND_RFREGULATIONTEST      (0x58)
#define PN532_COMMAND_INJUMPFORDEP          (0x56)
#define PN532_COMMAND_INJUMPFORPSL          (0x46)
#define PN532_COMMAND_ISO14443AinListPassiveTarget   (0x4A)
#define PN532_COMMAND_INATR                 (0x50)
#define PN532_COMMAND_INPSL                 (0x4E)
#define PN532_COMMAND_ISO14443AinDataExchange        (0x40)
#define PN532_COMMAND_INCOMMUNICATETHRU     (0x42)
#define PN532_COMMAND_INDESELECT            (0x44)
#define PN532_COMMAND_INRELEASE             (0x52)
#define PN532_COMMAND_INSELECT              (0x54)
#define PN532_COMMAND_INAUTOPOLL            (0x60)
#define PN532_COMMAND_TGINITASTARGET        (0x8C)
#define PN532_COMMAND_TGSETGENERALBYTES     (0x92)
#define PN532_COMMAND_TGGETDATA             (0x86)
#define PN532_COMMAND_TGSETDATA             (0x8E)
#define PN532_COMMAND_TGSETMETADATA         (0x94)
#define PN532_COMMAND_TGGETINITIATORCOMMAND (0x88)
#define PN532_COMMAND_TGRESPONSETOINITIATOR (0x90)
#define PN532_COMMAND_TGGETTARGETSTATUS     (0x8A)

#define PN532_RESPONSE_ISO14443AinDataExchange       (0x41)
#define PN532_RESPONSE_ISO14443AinListPassiveTarget  (0x4B)

#define PN532_WAKEUP                        (0x55)

#define PN532_SPI_STATREAD                  (0x02)
#define PN532_SPI_DATAWRITE                 (0x01)
#define PN532_SPI_DATAREAD                  (0x03)
#define PN532_SPI_READY                     (0x01)

#define PN532_I2C_ADDRESS                   (0x48 >> 1)
#define PN532_I2C_READBIT                   (0x01)
#define PN532_I2C_BUSY                      (0x00)
#define PN532_I2C_READY                     (0x01)
#define PN532_I2C_READYTIMEOUT              (20)

#define PN532_MIFARE_ISO14443A              (0x00)

// Mifare Commands
#define MIFARE_CMD_AUTH_A                   (0x60)
#define MIFARE_CMD_AUTH_B                   (0x61)
#define MIFARE_CMD_READ                     (0x30)
#define MIFARE_CMD_WRITE                    (0xA0)
#define MIFARE_CMD_TRANSFER                 (0xB0)
#define MIFARE_CMD_DECREMENT                (0xC0)
#define MIFARE_CMD_INCREMENT                (0xC1)
#define MIFARE_CMD_STORE                    (0xC2)

// Prefixes for NDEF Records (to identify record type)
#define NDEF_URIPREFIX_NONE                 (0x00)
#define NDEF_URIPREFIX_HTTP_WWWDOT          (0x01)
#define NDEF_URIPREFIX_HTTPS_WWWDOT         (0x02)
#define NDEF_URIPREFIX_HTTP                 (0x03)
#define NDEF_URIPREFIX_HTTPS                (0x04)
#define NDEF_URIPREFIX_TEL                  (0x05)
#define NDEF_URIPREFIX_MAILTO               (0x06)
#define NDEF_URIPREFIX_FTP_ANONAT           (0x07)
#define NDEF_URIPREFIX_FTP_FTPDOT           (0x08)
#define NDEF_URIPREFIX_FTPS                 (0x09)
#define NDEF_URIPREFIX_SFTP                 (0x0A)
#define NDEF_URIPREFIX_SMB                  (0x0B)
#define NDEF_URIPREFIX_NFS                  (0x0C)
#define NDEF_URIPREFIX_FTP                  (0x0D)
#define NDEF_URIPREFIX_DAV                  (0x0E)
#define NDEF_URIPREFIX_NEWS                 (0x0F)
#define NDEF_URIPREFIX_TELNET               (0x10)
#define NDEF_URIPREFIX_IMAP                 (0x11)
#define NDEF_URIPREFIX_RTSP                 (0x12)
#define NDEF_URIPREFIX_URN                  (0x13)
#define NDEF_URIPREFIX_POP                  (0x14)
#define NDEF_URIPREFIX_SIP                  (0x15)
#define NDEF_URIPREFIX_SIPS                 (0x16)
#define NDEF_URIPREFIX_TFTP                 (0x17)
#define NDEF_URIPREFIX_BTSPP                (0x18)
#define NDEF_URIPREFIX_BTL2CAP              (0x19)
#define NDEF_URIPREFIX_BTGOEP               (0x1A)
#define NDEF_URIPREFIX_TCPOBEX              (0x1B)
#define NDEF_URIPREFIX_IRDAOBEX             (0x1C)
#define NDEF_URIPREFIX_FILE                 (0x1D)
#define NDEF_URIPREFIX_URN_EPC_ID           (0x1E)
#define NDEF_URIPREFIX_URN_EPC_TAG          (0x1F)
#define NDEF_URIPREFIX_URN_EPC_PAT          (0x20)
#define NDEF_URIPREFIX_URN_EPC_RAW          (0x21)
#define NDEF_URIPREFIX_URN_EPC              (0x22)
#define NDEF_URIPREFIX_URN_NFC              (0x23)

#define PN532_GPIO_VALIDATIONBIT            (0x80)
#define PN532_GPIO_P30                      (0)
#define PN532_GPIO_P31                      (1)
#define PN532_GPIO_P32                      (2)
#define PN532_GPIO_P33                      (3)
#define PN532_GPIO_P34                      (4)
#define PN532_GPIO_P35                      (5)

//Prototypes
// Generic PN532 functions
  boolean 	PN532SAMConfig(void);
  u32 		PN532getFirmwareVersion(void);
  boolean 	PN532sendCommandCheckAck(u8 *cmd, u8 cmdlen, u16 timeout); //timeout default =1000
  boolean 	PN532writeGPIO(u8 pinstate);
  u8 		PN532readGPIO(void);
  boolean 	PN532setPassiveActivationRetries(u8 maxRetries);

  // ISO14443A functions
  boolean ISO14443AinListPassiveTarget();
  boolean ISO14443AreadPassiveTargetID(u8 cardbaudrate, u8 * uid, u8 * uidLength);
  boolean ISO14443AinDataExchange(u8 * send, u8 sendLength, u8 * response, u8 * responseLength);

  // Mifare Classic functions
  boolean mifareclassic_IsFirstBlock (u32 uiBlock);
  boolean mifareclassic_IsTrailerBlock (u32 uiBlock);
  u8 mifareclassic_AuthenticateBlock (u8 * uid, u8 uidLen, u32 blockNumber, u8 keyNumber, u8 * keyData);
  u8 mifareclassic_ReadDataBlock (u8 blockNumber, u8 * data);
  u8 mifareclassic_WriteDataBlock (u8 blockNumber, u8 * data);
  u8 mifareclassic_FormatNDEF (void);
  u8 mifareclassic_WriteNDEFURI (u8 sectorNumber, u8 uriIdentifier, const char * url);

  // Mifare Ultralight functions
  u8 mifareultralight_ReadPage (u8 page, u8 * buffer);

  // Help functions to display formatted text
  //static void PrintHex(const byte * data, const u32 numBytes);
  //static void PrintHexChar(const byte * pbtData, const u32 numBytes);

//u8 _irq, _reset;
u8 _uid[7];  // ISO14443A uid
u8 _uidLen;  // uid len
u8 _key[6];  // Mifare Classic key
u8 inListedTag; // Tg number of inlisted tag.

void    PN532sendCommand(u8* cmd, u8 cmdlen);
boolean PN532waitUntilReady(u16 timeout);
boolean PN532readackframe(void);
u8 		PN532wirereadstatus(void);
u8 pn532ackFrame[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
u8 pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

#define PN532_PACKBUFFSIZ 64
u8 pn532_packetbuffer[PN532_PACKBUFFSIZ];

// I2C Interface Functions

boolean    I2CreadStream(u8* buff, u8 n);
extern void I2CConfigModule();
extern void I2CBusDelay();
extern unsigned char I2CDelay();
extern unsigned char I2CSendBurstWoPCA(unsigned char slaveAddr, unsigned char *burst, unsigned char len);
extern unsigned char I2CRecvBurstWoPCA(unsigned char slaveAddr, unsigned char *burst, unsigned char len);

//Reads a single byte via I2C
static inline u8 I2CrecvByte(void) {
	return 1;//WIRE.read();
}
// Reads n bytes of data from the PN532 via I2C
//param  buff      Pointer to the buffer where data will be written
//@param  n         Number of bytes to be read
boolean I2CreadStream(u8* buff, u8 n) {
	u16 timer = 0;
	u8 i;
	u16 timeout = 1000;

	  // Wait for chip to say its ready!
	  while (PN532wirereadstatus() != PN532_I2C_READY) {
	    if (timeout != 0) {
	      timer+=10;
	      if (timer > timeout){
	    	  //UARTprintf("---I2CreadStream IRQ:Timeout\r\n");
	    	  return false;
	      }
	    }
	    somedelay(10);
	  }

	  #ifdef PN532DEBUG
	  UARTprintf("IRQ received\r\n");
	  #endif

	//somedelay(2); //delay(2);
	#ifdef PN532DEBUG
		UARTprintf("Reading: ");
	#endif
/*
	// Start read (n+1 to take into account leading 0x01(RDY) with I2C)
	I2CrequestFrom((u8)PN532_I2C_ADDRESS, (u8)(n+2));// WIRE.requestFrom((u8)PN532_I2C_ADDRESS, (u8)(n+2));
	// Discard the leading 0x01(RDY)
	I2CrecvByte();
	for (i=0; i<n; i++) {
		delay(1);
		buff[i] = I2CrecvByte();
		#ifdef PN532DEBUG
		//UARTprintf(" 0x"); UARTprintf(buff[i], HEX);
		sprintf(ostr," %02x", buff[i]);
		UARTSend((unsigned char *)ostr, strlen((char *)ostr));
		#endif
	}
  // Discard trailing 0x00 0x00
  // I2CrecvByte();
*/
	I2CRecvBurst(PN532_I2C_ADDRESS, buff, (n+2));

	#ifdef PN532DEBUG
	UARTprintf("RDY:%02x,Frame=",buff[0]);
	//discard the first byte of RDY
	for (i=0; i<n; i++){
		buff[i] = buff[i+1];
		UARTprintf("%02x ",buff[i]);
	}
	UARTprintf("\r\n");
		//sprintf(ostr,"\r\n");
		//UARTSend((unsigned char *)ostr, strlen((char *)ostr));

	#endif

	return true;
}
//////////////////////////////////////////////////////////////////////////////////////
//  Tries to read the PN532 ACK frame (not to be confused with the I2C ACK signal)
boolean PN532readackframe(void) {
	u8 ackbuff[6];
	I2CreadStream(ackbuff, 6);
	return (0 == strncmp((char *)ackbuff, (char *)pn532ackFrame, 6));
}
// Checks the IRQ pin to know if the PN532 is ready @returns 0 if the PN532 is busy, 1 if it is free
u8 PN532wirereadstatus(void) {
	//u8 x = IRQ532; //digitalRead(_irq);
	if (IRQ532) return PN532_I2C_BUSY; //No IRQ ...active low IRQ
	else return PN532_I2C_READY; //yes IRQ
}

//ConfGPIO
//irq       Location of the IRQ pin
//reset     Location of the RSTPD_N pin
void PN532GpioConf(void)
{
	//I2C SCL : Pin17 : PB2
	//I2C SDA : Pin18 : PB3
	//IRQNFC  : Pin15 : PD6
	//BUZZER : Pin34 : PB1(v4. TBC )
	//LED : PA7

	I2CConfigModule();

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6); //IRQNFC
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7); //LED
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1); //SPK
	//Init
	somedelay(100);

}
/*
void initNFC(){
I2CWIRE.begin();
// Reset the PN532
digitalWrite(_reset, HIGH);
digitalWrite(_reset, LOW);
delay(400);
digitalWrite(_reset, HIGH);
}
*/
//Checks the firmware version of the PN5xx chip.
u32 PN532getFirmwareVersion(void) {
  u32 response;
  u8 i;
  pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
  if (!PN532sendCommandCheckAck(pn532_packetbuffer, 1,1000)) return 0;
  // read data packet
  //UARTprintf("I2cReadStream()\r\n");
  I2CreadStream(pn532_packetbuffer, 12);
  // check some basic stuff
  UARTprintf("Rx Stream = ");
  for(i=0;i<12;i++)
	  UARTprintf("%02x ",pn532_packetbuffer[i]);
  UARTprintf("\r\n");

  if (0 != strncmp((char *)pn532_packetbuffer, (char *)pn532response_firmwarevers, 6)) {
    #ifdef PN532DEBUG
    UARTprintf("Firmware doesn't match!\r\n");
    #endif
    //return 0;//YOON
  }
  response = pn532_packetbuffer[7];   response <<= 8;
  response |= pn532_packetbuffer[8];   response <<= 8;
  response |= pn532_packetbuffer[9];   response <<= 8;
  response |= pn532_packetbuffer[10];
  return response;
}

// Sends a command and waits a specified period for the ACK. default timeout of one second
/*
    @param  cmd       Pointer to the command buffer
    @param  cmdlen    The size of the command in bytes
    @param  timeout   timeout before giving up
@returns  1 if everything is OK, 0 if timeout occured before an
              ACK was recieved
*/
boolean PN532sendCommandCheckAck(u8 *cmd, u8 cmdlen, u16 timeout) {
  u16 timer = 0;
// write the command
  UARTprintf("SendCommandCheckAck(cmd=%x;len=%d)\r\n",cmd[0],cmdlen);
  PN532sendCommand(cmd, cmdlen);
  /*
  // Wait for chip to say its ready!
  while (PN532wirereadstatus() != PN532_I2C_READY) {
    if (timeout != 0) {
      timer+=10;
      if (timer > timeout){
    	  UARTprintf("----------SendCommandCheckAck:Timeout\r\n");
    	  return false;
      }
    }
    somedelay(10);
  }

  #ifdef PN532DEBUG
  UARTprintf("IRQ received\r\n");
  #endif
  */
  // read acknowledgement
  if (!PN532readackframe()) {
    #ifdef PN532DEBUG
    UARTprintf("No ACK frame received!\r\n");
    #endif
    return false;
  }
  return true; // ack'd command
}

/*!     Writes an 8-bit value that sets the state of the PN532's GPIO pins
@warning This function is provided exclusively for board testing and
             is dangerous since it will throw an error if any pin other
             than the ones marked "Can be used as GPIO" are modified!  All
             pins that can not be used as GPIO should ALWAYS be left high
             (value = 1) or the system will become unstable and a HW reset
             will be required to recover the PN532.

             pinState[0]  = P30     Can be used as GPIO
             pinState[1]  = P31     Can be used as GPIO
             pinState[2]  = P32     *** RESERVED (Must be 1!) ***
             pinState[3]  = P33     Can be used as GPIO
             pinState[4]  = P34     *** RESERVED (Must be 1!) ***
             pinState[5]  = P35     Can be used as GPIO

    @returns 1 if everything executed properly, 0 for an error
*/
boolean PN532writeGPIO(u8 pinstate) {
  u8 errorbit,i;
  // Make sure pinstate does not try to toggle P32 or P34
  pinstate |= (1 << PN532_GPIO_P32) | (1 << PN532_GPIO_P34);

  // Fill command buffer
  pn532_packetbuffer[0] = PN532_COMMAND_PN532writeGPIO;
  pn532_packetbuffer[1] = PN532_GPIO_VALIDATIONBIT | pinstate;  // P3 Pins
  pn532_packetbuffer[2] = 0x00;    // P7 GPIO Pins (not used ... taken by I2C)

  #ifdef PN532DEBUG
    UARTprintf("Writing P3 GPIO: 0x%02x ",pn532_packetbuffer[1]);
  #endif

  // Send the PN532writeGPIO command (0x0E)
  if (! PN532sendCommandCheckAck(pn532_packetbuffer, 3,1000)) return 0x0;

  // Read response packet (00 00 FF PLEN PLENCHECKSUM D5 CMD+1(0x0F) DATACHECKSUM)
  I2CreadStream(pn532_packetbuffer, 8);

  #ifdef PN532DEBUG
    UARTprintf("Received: ");
    for(i=0;i<8;i++)    //PrintHex(pn532_packetbuffer, 8);
    	UARTprintf(" %02x",pn532_packetbuffer[i]);
    UARTprintf("");
  #endif

  return  (pn532_packetbuffer[6] == 0x0F);
}

/*!     Reads the state of the PN532's GPIO pins
@returns An 8-bit value containing the pin state where:
pinState[0]  = P30
             pinState[1]  = P31
             pinState[2]  = P32
             pinState[3]  = P33
             pinState[4]  = P34
             pinState[5]  = P35
*/
u8 PN532readGPIO(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_PN532readGPIO;
  // Send the PN532readGPIO command (0x0C)
  if (! PN532sendCommandCheckAck(pn532_packetbuffer, 1,1000))    return 0x0;
  // Read response packet (00 00 FF PLEN PLENCHECKSUM D5 CMD+1(0x0D) P3 P7 IO1 DATACHECKSUM)
  I2CreadStream(pn532_packetbuffer, 11);
  /* PN532readGPIO response should be in the following format:
    byte            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              P3 GPIO Pins
    b8              P7 GPIO Pins (not used ... taken by I2C)
    b9              Interface Mode Pins (not used ... bus select pins)
    b10             checksum */

  #ifdef PN532DEBUG
    UARTprintf("Received: ");
    //PrintHex(pn532_packetbuffer, 11);
    UARTprintf("");
    UARTprintf("P3 GPIO: 0x%02x",pn532_packetbuffer[7]);
    UARTprintf("P7 GPIO: 0x%02x",pn532_packetbuffer[8]);
    UARTprintf("IO GPIO: 0x%02x",pn532_packetbuffer[9]);
    // Note: You can use the IO GPIO value to detect the serial bus being used
    switch(pn532_packetbuffer[9])
    {
      case 0x00:    // Using UART
        UARTprintf("Using UART (IO = 0x00)");        break;
      case 0x01:    // Using I2C
        UARTprintf("Using I2C (IO = 0x01)");        break;
      case 0x02:    // Using I2C
        UARTprintf("Using I2C (IO = 0x02)");        break;
    }
  #endif

  return pn532_packetbuffer[6];
}

//Configures the SAM (Secure Access Module)
boolean PN532SAMConfig(void) {
	UARTprintf("SAMConfig(NormalMode wo SAM)..\r\n");
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x01; // normal mode;
  pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second (Valid only in Mode 2)
  pn532_packetbuffer[3] = 0x01; // use P70IRQ pin!

  if (! PN532sendCommandCheckAck(pn532_packetbuffer, 4, 1000)) return false;
  // read data packet
  I2CreadStream(pn532_packetbuffer, 8); //...0xD5 0x15
  return  (pn532_packetbuffer[6] == 0x15);
}

/*!
    Sets the MxRtyPassiveActivation byte of the RFConfiguration register
@param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout after mxRetries
@returns 1 if everything executed properly, 0 for an error
*/
boolean PN532setPassiveActivationRetries(u8 maxRetries) {
  pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
  pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
  pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
  pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
  pn532_packetbuffer[4] = maxRetries;

#ifdef MIFAREDEBUG
  UARTprintf("Setting MxRtyPassiveActivation to %d \r\n",maxRetries);
#endif

  if (! PN532sendCommandCheckAck(pn532_packetbuffer, 5, 1000)) return 0x0;  // no ACK
return 1;
}

/***** ISO14443A Commands ******/
/*!     Waits for an ISO14443A target to enter the field
@param  cardBaudRate  Baud rate of the card
    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 bytes)
    @param  uidLength     Pointer to the variable that will hold the length of the card's UID.
@returns 1 if everything executed properly, 0 for an error
*/
boolean ISO14443AreadPassiveTargetID(u8 cardbaudrate, u8 * uid, u8 * uidLength) {
	u8 i;

	UARTprintf("\r\nRequest PN532 for reading Iso14443A tags\r\n");

  pn532_packetbuffer[0] = PN532_COMMAND_ISO14443AinListPassiveTarget; //0x4A
  pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
  pn532_packetbuffer[2] = cardbaudrate; //0x00=106Kbps Type A(14443 type A)


  if (!PN532sendCommandCheckAck(pn532_packetbuffer, 3, 1000)) {
    #ifdef PN532DEBUG
        UARTprintf("No ACK from Chip...\r\n");
    #endif
    return 0x0;
  }
/*
  // Wait for a card to enter the field
  u8 status = PN532_I2C_BUSY;
  #ifdef PN532DEBUG
  UARTprintf("Waiting for IRQ (indicates card presence)\r\n");
  #endif

  //If not, then endless loop....YOON
  while (PN532wirereadstatus() != PN532_I2C_READY) somedelay(10);

  #ifdef PN532DEBUG
  UARTprintf("Found a card\r\n");
  #endif
  */
  UARTprintf("Waiting for an ISO14443A Card ...\r\n");
// read data packet (endless loop)
  while(!I2CreadStream(pn532_packetbuffer, 20)) somedelay(10);


  // check some basic stuff
  /* ISO14443A card response should be in the following format:
    byte            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              Tags Found
    b8              Tag Number (only one used in this example)
    b9..10          SENS_RES
    b11             SEL_RES
    b12             NFCID Length
    b13..NFCIDLen   NFCID                                      */

  if (pn532_packetbuffer[7] != 1)  return 0;
#ifdef MIFAREDEBUG
    UARTprintf("[8]Found %d Tags: ",pn532_packetbuffer[8]);//7]);
#endif

  //Type A: Tg | Sens_Res[2] | Sel_Res[1] | NFCIDLen[1] | NFCID1[ ] | ATS[ ]
  u16 sens_res = pn532_packetbuffer[9];
  sens_res <<= 8;
  sens_res |= pn532_packetbuffer[10];
#ifdef MIFAREDEBUG
    UARTprintf("[9..10]ATQA(SensRes): 0x%02x ",sens_res);  //UARTprintf(sens_res, HEX);
    UARTprintf("[11]SAK[SelRes]: 0x%02x ",pn532_packetbuffer[11]);  //UARTprintf(pn532_packetbuffer[11], HEX);
#endif

  /* Card appears to be Mifare Classic */
  *uidLength = pn532_packetbuffer[12]; //NFCidLen
  UARTprintf("[12]NFCidLen(UidLen): %d ",pn532_packetbuffer[12]);
#ifdef MIFAREDEBUG
    UARTprintf(" UID(HEX):");
#endif
  for (i=0; i < pn532_packetbuffer[12]; i++) {
    uid[i] = pn532_packetbuffer[13+i];
#ifdef MIFAREDEBUG
      UARTprintf(" %02x",uid[i]);
#endif
  }
#ifdef MIFAREDEBUG
    UARTprintf("\r\n");
#endif
  return 1;
}
/***** Mifare Classic Functions ******/
/*!
      Indicates whether the specified block number is the first block
      in the sector (block 0 relative to the current sector)
*/
boolean mifareclassic_IsFirstBlock (u32 uiBlock){
  // Test if we are in the small or big sectors
  if (uiBlock < 128)    return ((uiBlock) % 4 == 0);
  else    return ((uiBlock) % 16 == 0);
}

// Indicates whether the specified block number is the sector trailer
boolean mifareclassic_IsTrailerBlock (u32 uiBlock){
  // Test if we are in the small or big sectors
  if (uiBlock < 128)    return ((uiBlock + 1) % 4 == 0);
  else    return ((uiBlock + 1) % 16 == 0);
}

/*!     Tries to authenticate a block of memory on a MIFARE card using the
    ISO14443AinDataExchange command.  See section 7.3.8 of the PN532 User Manual
    for more information on sending MIFARE and other commands.

    @param  uid           Pointer to a byte array containing the card UID
    @param  uidLen        The length (in bytes) of the card's UID (Should be 4 for MIFARE Classic)
    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  keyNumber     Which key type to use during authentication
                          (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
    @param  keyData       Pointer to a byte array containing the 6 byte key value
@returns 1 if everything executed properly, 0 for an error
*/
u8 mifareclassic_AuthenticateBlock (u8 * uid, u8 uidLen, u32 blockNumber, u8 keyNumber, u8 * keyData){
  u8 len;
  u8 i;

  // Hang on to the key and uid data
  memcpy (_key, keyData, 6);
  memcpy (_uid, uid, uidLen);
  _uidLen = uidLen;

  #ifdef MIFAREDEBUG
  UARTprintf("Trying to authenticate card : UID=");
  for (i = 0; i < _uidLen; i++)
	  UARTprintf(" %02x",_uid[i]);  //PrintHex(_uid, _uidLen);

  UARTprintf("\r\nUsing authentication KEY=%c :", (keyNumber ? 'B' : 'A'));//;UARTprintf(": ");
  for(i=0;i<6;i++)
	  UARTprintf(" %02x",_key[i]);  //PrintHex(_key, 6);
  #endif
  UARTprintf("\r\n");
  // Prepare the authentication command //
  pn532_packetbuffer[0] = PN532_COMMAND_ISO14443AinDataExchange;   /* Data Exchange Header */
  pn532_packetbuffer[1] = 1;                              /* Max card numbers */
  pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
  pn532_packetbuffer[3] = blockNumber;                    /* Block Number (1K = 0..63, 4K = 0..255 */
  memcpy (pn532_packetbuffer+4, _key, 6);
  for (i = 0; i < _uidLen; i++)  pn532_packetbuffer[10+i] = _uid[i];  // 4 byte card ID
  if (! PN532sendCommandCheckAck(pn532_packetbuffer, 10+_uidLen, 1000))    return 0;

  // Read the response packet
  while (PN532wirereadstatus() != PN532_I2C_READY) somedelay(10); //YOON added
  I2CreadStream(pn532_packetbuffer, 12);

  // Check if the response is valid and we are authenticated???
  // for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
  // Mifare auth error is technically byte 7: 0x14 but anything other and 0x00 is not good
  if (pn532_packetbuffer[7] != 0x00)  {
    #ifdef PN532DEBUG
    UARTprintf("Authentification[failed]: ");
    for(i=0;i<12;i++)
    	UARTprintf(" %02x",pn532_packetbuffer[i] );    //PrintHexChar(pn532_packetbuffer, 12);
    #endif
    UARTprintf("\r\n");
    return 0;
  }
  else
	  UARTprintf("AuthenticateBlock[Success] \r\n ");
  return 1;
}

/* Tries to read an entire 16-byte data block at the specified block address.
    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          Pointer to the byte array that will hold the retrieved data (if any)
@returns 1 if everything executed properly, 0 for an error
*/
u8 mifareclassic_ReadDataBlock (u8 blockNumber, u8 * data){
	u8 i;

  #ifdef MIFAREDEBUG
  UARTprintf("Trying to read 16 bytes from block %d\r\n",blockNumber);
  #endif

  //  Prepare the command
  pn532_packetbuffer[0] = PN532_COMMAND_ISO14443AinDataExchange;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */

  // Send the command
  if (! PN532sendCommandCheckAck(pn532_packetbuffer, 4, 1000))  {
    #ifdef MIFAREDEBUG
    UARTprintf("Failed to receive ACK for read command\r\n");
    #endif
    return 0;
  }

  // Read the response packet
  I2CreadStream(pn532_packetbuffer, 26);

  // If byte 8 isn't 0x00 we probably have an error
  if (pn532_packetbuffer[7] != 0x00) {
    #ifdef MIFAREDEBUG
    UARTprintf("Unexpected response[HEX]:");
    for(i=0;i<26;i++)
    	UARTprintf(" %02x",pn532_packetbuffer[i]);     //PrintHexChar(pn532_packetbuffer, 26);
    UARTprintf("\r\n");
    #endif
    return 0;
  }

  // Copy the 16 data bytes to the output buffer
  // Block content starts at byte 9 of a valid response
  memcpy (data, pn532_packetbuffer+8, 16);

  // Display data for debug if requested
  #ifdef MIFAREDEBUG
    UARTprintf("Block[HEX]: ");
    for(i=0;i<16;i++)
    	UARTprintf(" %02x",data[i]);    //PrintHexChar(data, 16);
    UARTprintf("\r\n");
  #endif

  return 1;
}

/*!   Tries to write an entire 16-byte data block at the specified block address.
    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          The byte array that contains the data to write.
@returns 1 if everything executed properly, 0 for an error
*/
u8 mifareclassic_WriteDataBlock (u8 blockNumber, u8 * data){
  #ifdef MIFAREDEBUG
  UARTprintf("Trying to write 16 bytes to block ");//UARTprintf(blockNumber);
  #endif

  // Prepare the first command
  pn532_packetbuffer[0] = PN532_COMMAND_ISO14443AinDataExchange;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
  memcpy (pn532_packetbuffer+4, data, 16);          /* Data Payload */

  // Send the command
  if (! PN532sendCommandCheckAck(pn532_packetbuffer, 20, 1000))  {
    #ifdef MIFAREDEBUG
    UARTprintf("Failed to receive ACK for write command\r\n");
    #endif
    return 0;
  }
  delay(10);
  // Read the response packet
  I2CreadStream(pn532_packetbuffer, 26);
  return 1;
}

/*! Formats a Mifare Classic card to store NDEF Records
@returns 1 if everything executed properly, 0 for an error
*/
u8 mifareclassic_FormatNDEF (void){
  u8 sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
  u8 sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
  u8 sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // Note 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 must be used for key A
  // for the MAD sector in NDEF records (sector 0)

  // Write block 1 and 2 to the card
  if (!(mifareclassic_WriteDataBlock (1, sectorbuffer1)))
    return 0;
  if (!(mifareclassic_WriteDataBlock (2, sectorbuffer2)))
    return 0;
  // Write key A and access rights card
  if (!(mifareclassic_WriteDataBlock (3, sectorbuffer3)))
    return 0;

  // Seems that everything was OK (?!)
  return 1;
}

/**************************************************************************
Writes an NDEF URI Record to the specified sector (1..15)
Note that this function assumes that the Mifare Classic card is
    already formatted to work as an "NFC Forum Tag" and uses a MAD1
    file system.  You can use the NXP TagWriter app on Android to
    properly format cards for this.

    @param  sectorNumber  The sector that the URI record should be written
                          to (can be 1..15 for a 1K card)
    @param  uriIdentifier The uri identifier code (0 = none, 0x01 =
                          "http://www.", etc.)
    @param  url           The uri text to write (max 38 characters).

    @returns 1 if everything executed properly, 0 for an error
*/
u8 mifareclassic_WriteNDEFURI (u8 sectorNumber, u8 uriIdentifier, const char * url){
  // Figure out how long the string is
  u8 len = strlen(url);

  // Make sure we're within a 1K limit for the sector number
  if ((sectorNumber < 1) || (sectorNumber > 15))
    return 0;

  // Make sure the URI payload is between 1 and 38 chars
  if ((len < 1) || (len > 38))
    return 0;

  // Note 0xD3 0xF7 0xD3 0xF7 0xD3 0xF7 must be used for key A
  // in NDEF records

  // PN532Config the sector buffer (w/pre-formatted TLV wrapper and NDEF message)
  u8 sectorbuffer1[16] = {0x00, 0x00, 0x03, len+5, 0xD1, 0x01, len+1, 0x55, uriIdentifier, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  u8 sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  u8 sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  u8 sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (len <= 6)
  {
    // Unlikely we'll get a url this short, but why not ...
    memcpy (sectorbuffer1+9, url, len);
    sectorbuffer1[len+9] = 0xFE;
  }
  else if (len == 7)
  {
    // 0xFE needs to be wrapped around to next block
    memcpy (sectorbuffer1+9, url, len);
    sectorbuffer2[0] = 0xFE;
  }
  else if ((len > 7) || (len <= 22))
  {
    // Url fits in two blocks
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, len-7);
    sectorbuffer2[len-7] = 0xFE;
  }
  else if (len == 23)
  {
    // 0xFE needs to be wrapped around to final block
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, len-7);
    sectorbuffer3[0] = 0xFE;
  }
  else {
    // Url fits in three blocks
    memcpy (sectorbuffer1+9, url, 7);
    memcpy (sectorbuffer2, url+7, 16);
    memcpy (sectorbuffer3, url+23, len-24);
    sectorbuffer3[len-22] = 0xFE;
  }

  // Now write all three blocks back to the card
  if (!(mifareclassic_WriteDataBlock (sectorNumber*4, sectorbuffer1)))    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+1, sectorbuffer2)))    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+2, sectorbuffer3)))    return 0;
  if (!(mifareclassic_WriteDataBlock ((sectorNumber*4)+3, sectorbuffer4)))    return 0;

  // Seems that everything was OK (?!)
  return 1;
}

/***** Mifare Ultralight Functions ******
Tries to read an entire 4-byte page at the specified address.
    @param  page        The page number (0..63 in most cases)
    @param  buffer      Pointer to the byte array that will hold the
                        retrieved data (if any)
*/
u8 mifareultralight_ReadPage (u8 page, u8 * buffer){
	u8 i;
  if (page >= 64)  {
    #ifdef MIFAREDEBUG
    UARTprintf("Page value out of range");
    #endif
    return 0;
 }

  #ifdef MIFAREDEBUG
    UARTprintf("Reading page ");//UARTprintf(page);
  #endif

  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_ISO14443AinDataExchange;
  pn532_packetbuffer[1] = 1;                   /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;     /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = page;                /* Page Number (0..63 in most cases) */

  /* Send the command */
  if (! PN532sendCommandCheckAck(pn532_packetbuffer, 4, 1000))
  {
    #ifdef MIFAREDEBUG
    UARTprintf("Failed to receive ACK for write command\r\n");
    #endif
    return 0;
  }

  /* Read the response packet */
  I2CreadStream(pn532_packetbuffer, 26);
  #ifdef MIFAREDEBUG
    UARTprintf("Received[HEX]: ");
    for(i=0;i<26;i++) //PrintHexChar(pn532_packetbuffer, 26);
    	UARTprintf(" %02x", pn532_packetbuffer[i]);
    UARTprintf("\r\n");
  #endif

  /* If byte 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] == 0x00)  {
    /* Copy the 4 data bytes to the output buffer         */
    /* Block content starts at byte 9 of a valid response */
    /* Note that the command actually reads 16 byte or 4  */
    /* pages at a time ... we simply discard the last 12  */
    /* bytes                                              */
    memcpy (buffer, pn532_packetbuffer+8, 4);
  }
  else  {
    #ifdef MIFAREDEBUG
      UARTprintf("Unexpected response reading block[HEX]: ");
      for(i=0;i<26;i++) //PrintHexChar(pn532_packetbuffer, 26);
      	UARTprintf(" %02x", pn532_packetbuffer[i]);
      UARTprintf("\r\n");
    #endif
    return 0;
  }

  /* Display data for debug if requested */
  #ifdef MIFAREDEBUG
    UARTprintf("Page :");//UARTprintf(page);UARTprintf(":");
    for(i=0;i<4;i++)
    	UARTprintf(" %02x", buffer[i]);
    UARTprintf("\r\n");    //PrintHexChar(buffer, 4);
  #endif

  // Return OK signal
  return 1;
}

/*!  Writes a command to the PN532, automatically inserting the preamble and required frame details (checksum, len, etc.)
	@param  cmd       Pointer to the command buffer
    @param  cmdlen    Command length in bytes
*/
void PN532sendCommand(u8* cmd, u8 cmdlen) {
  u8 checksum,i;
  u8 burst[100];


#ifdef PN532DEBUG
  UARTprintf("SendingCommand(cmd=%x:Len=%d):",cmd[0],cmdlen);
#endif
  cmdlen++;//add one byte of TFI(0xd4 or d5)
  somedelay(2);     // or whatever the delay is for waking up the board
/*
  // I2C START
  I2CbeginTransmission(PN532_I2C_ADDRESS);//WIRE.xxx
  checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
  I2CsendByte(PN532_PREAMBLE);
  I2CsendByte(PN532_PREAMBLE);
  I2CsendByte(PN532_STARTCODE2);

  I2CsendByte(cmdlen);
  I2CsendByte(~cmdlen + 1);
  I2CsendByte(PN532_HOSTTOPN532);
  checksum += PN532_HOSTTOPN532;

#ifdef PN532DEBUG
  UARTprintf(" 0x"); //UARTprintf(PN532_PREAMBLE, HEX);
  UARTprintf(" 0x"); //UARTprintf(PN532_PREAMBLE, HEX);
  UARTprintf(" 0x"); //UARTprintf(PN532_STARTCODE2, HEX);
  UARTprintf(" 0x"); //UARTprintf(cmdlen, HEX);
  UARTprintf(" 0x"); //UARTprintf(~cmdlen + 1, HEX);
  UARTprintf(" 0x"); //UARTprintf(PN532_HOSTTOPN532, HEX);
#endif

  for (i=0; i<cmdlen-1; i++) {
    I2CsendByte(cmd[i]);
    checksum += cmd[i];
#ifdef PN532DEBUG
    UARTprintf(" 0x"); //UARTprintf(cmd[i], HEX);
#endif
  }

  I2CsendByte(~checksum);
  I2CsendByte(PN532_POSTAMBLE);

  // I2C STOP
  I2CendTransmission(); //  WIRE.endTransmission();

#ifdef PN532DEBUG
  UARTprintf(" 0x"); //UARTprintf(~checksum, HEX);
  UARTprintf(" 0x"); //UARTprintf(PN532_POSTAMBLE, HEX);
  UARTprintf();
#endif
*/
  checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
  burst[0]=PN532_PREAMBLE; //0x00
  burst[1]=PN532_PREAMBLE; //0x00
  burst[2]=PN532_STARTCODE2; //0xFF
  burst[3]=cmdlen; //LEN
  burst[4]=~cmdlen+1; //LCC PktLen Checksum
  burst[5]=PN532_HOSTTOPN532; //TFI Specific PN532 Packet Frame ID. (0xD4 to PN532)
  checksum += PN532_HOSTTOPN532;

  for (i=0; i<cmdlen-1; i++) {
    burst[i+6]= cmd[i];
    checksum += cmd[i];
  }
  burst[6+cmdlen-1]=~checksum;
  burst[6+cmdlen]=PN532_POSTAMBLE;

  for(i=0;i<(6+cmdlen+1);i++){
	  UARTprintf("%02x",burst[i]);
  }
  UARTprintf("\r\n");
  I2CSendBurst(PN532_I2C_ADDRESS, burst, 6+cmdlen+1);
}

/*! Waits until the PN532 is ready.
    @param  timeout   Timeout before giving up
*/
boolean PN532waitUntilReady(u16 timeout) {
  u16 timer = 0;
  while(PN532wirereadstatus() != PN532_I2C_READY) {
    if (timeout != 0) {
      timer += 10;
      if (timer > timeout) {
        return false;
      }
    }
    delay(10);
  }
  return true;
}

/*! Exchanges an APDU with the currently inlisted peer
    @param  send            Pointer to data to send
    @param  sendLength      Length of the data to send
    @param  response        Pointer to response data
    @param  responseLength  Pointer to the response data length
*/
boolean ISO14443AinDataExchange(u8 * send, u8 sendLength, u8 * response, u8 * responseLength) {
  if (sendLength > PN532_PACKBUFFSIZ -2) {
    #ifdef PN532DEBUG
      UARTprintf("APDU length too long for packet buffer");
    #endif
    return false;
  }
  u8 i;

  pn532_packetbuffer[0] = 0x40; // PN532_COMMAND_ISO14443AinDataExchange;
  pn532_packetbuffer[1] = inListedTag;
  for (i=0; i<sendLength; ++i) {
    pn532_packetbuffer[i+2] = send[i];
  }

  if (!PN532sendCommandCheckAck(pn532_packetbuffer,sendLength+2,1000)) {
    #ifdef PN532DEBUG
      UARTprintf("Could not send ADPU");
    #endif
    return false;
  }

  if (!PN532waitUntilReady(1000)) {
    #ifdef PN532DEBUG
      UARTprintf("Response never received for ADPU...");
    #endif
    return false;
  }

  I2CreadStream(pn532_packetbuffer,sizeof(pn532_packetbuffer));

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    u8 length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(u8)(~length+1)) {
      #ifdef PN532DEBUG
        UARTprintf("Length check invalid");
        //UARTprintf(length,HEX);
        //UARTprintf((~length)+1,HEX);
      #endif
      return false;
    }
    if(pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_ISO14443AinDataExchange) {
      if ((pn532_packetbuffer[7] & 0x3f)!=0) {
        #ifdef PN532DEBUG
          UARTprintf("Status code indicates an error");
        #endif
        return false;
      }
      length -= 3;
      if (length > *responseLength) length = *responseLength; // silent truncation...
      for (i=0; i<length; ++i)  response[i] = pn532_packetbuffer[8+i];
      *responseLength = length;

      return true;
    }
    else {
      UARTprintf("Don't know how to handle this command: ");
      //UARTprintf(pn532_packetbuffer[6],HEX);
      return false;
    }
  }
  else {
    UARTprintf("Preamble missing");    return false;
  }
}

/*!  'InLists' a passive target. PN532 acting as reader/initiator,
            peer acting as card/responder.
*/
boolean ISO14443AinListPassiveTarget() {
  pn532_packetbuffer[0] = PN532_COMMAND_ISO14443AinListPassiveTarget;
  pn532_packetbuffer[1] = 1;
  pn532_packetbuffer[2] = 0;

  #ifdef PN532DEBUG
    UARTprintf("About to inList passive target");
  #endif

  if (!PN532sendCommandCheckAck(pn532_packetbuffer,3,1000)) {
    #ifdef PN532DEBUG
      UARTprintf("Could not send inlist message");
    #endif
    return false;
  }

  if (!PN532waitUntilReady(30000)) return false;

  I2CreadStream(pn532_packetbuffer,sizeof(pn532_packetbuffer));

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    u8 length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(u8)(~length+1)) {
      #ifdef PN532DEBUG
        UARTprintf("Length check invalid");
        //UARTprintf(length,HEX);
        //UARTprintf((~length)+1,HEX);
      #endif
      return false;
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_ISO14443AinListPassiveTarget) {
      if (pn532_packetbuffer[7] != 1) {
        #ifdef PN532DEBUG
        UARTprintf("Unhandled number of targets inlisted");
        #endif
        UARTprintf("Number of tags inlisted:");
        //UARTprintf(pn532_packetbuffer[7]);
        return false;
      }

      inListedTag = pn532_packetbuffer[8];
      UARTprintf("Tag number: ");
      //UARTprintf(inListedTag);

      return true;
    } else {
      #ifdef PN532DEBUG
        UARTprintf("Unexpected response to inlist passive host");
      #endif
      return false;
    }
  }
  else {
    #ifdef PN532DEBUG
      UARTprintf("Preamble missing");
    #endif
    return false;
  }

  return true;
}

//ReadMifare Example ================================================================================================
/*! These chips use I2C to communicate.

This example will wait for any ISO14443A card or tag, and
depending on the size of the UID will attempt to read from it.
If the card has a 4-byte UID it is probably a Mifare
Classic card, and the following steps are taken:

- Authenticate block 4 (the first block of Sector 1) using
  the default KEYA of 0XFF 0XFF 0XFF 0XFF 0XFF 0XFF
- If authentication succeeds, we can then read any of the
  4 blocks in that sector (though only block 4 is read here)

    If the card has a 7-byte UID it is probably a Mifare
    Ultralight card, and the 4 byte pages can be read directly.
    Page 4 is read by default since this is the first 'general-
    purpose' page on the tags.

*/



void PN532Config(void) {
  //Serial.begin(115200);
  UARTprintf("Start Config NXP PN532 NFC.... ");
  //nfc.begin();

  u32 versiondata = PN532getFirmwareVersion();
  if (!versiondata) {
	  UARTprintf("Didn't find PN53x board\r\n");
	  LED_RED_ON532;
	  while(1){//halt
		  LED_RED_ON532;
	  	  somedelay(1000000);
		  LED_RED_OFF532;
	  }
  }
  LED_RED_ON532;
  // Got ok data, print it out!
  UARTprintf("Found chip PN5%02x ", (versiondata>>24) & 0xFF);
  UARTprintf("Firmware ver.%02x",(versiondata>>16) & 0xFF);
  UARTprintf(".%d ",(versiondata>>8) & 0xFF);//Revision
  UARTprintf("(support=0x%02x)\r\n",(versiondata) & 0xFF);//Support

  // configure board to read RFID tags
  if(PN532SAMConfig())
	  UARTprintf("SAMConfig Done\r\n");
  else
	  UARTprintf("[Fail]SAMConfig Done\r\n");

}

void PN532ReadTagLoopForISO14443A(void) {
  u8 success;
  u8 uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  u8 uidLength,i; // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  UARTprintf("\r\n");
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)

  success = ISO14443AreadPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    // Display some basic information about the card
    UARTprintf(">Found an ISO14443A card");
    UARTprintf("  UID Length: %d bytes\r\n",uidLength);
    UARTprintf("  UID Value:");    //nfc.PrintHex(uid, uidLength);
    for(i=0;i<uidLength;i++)
    	UARTprintf(" %02x",uid[i]);
    CR;

    if (uidLength == 4){// We probably have a Mifare Classic card ...
      UARTprintf("Seems to be a Mifare Classic card (4 byte UID)\r\n");

      // Now we need to try to authenticate it for read/write access
      // Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
      UARTprintf(">Trying to authenticate block 4 with default KEYA value(0xFF..0xFF)\r\n");
      u8 keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

      // Start with block 4 (the first block of sector 1) since sector 0
      // contains the manufacturer data and it's probably better just
      // to leave it alone unless you know what you're doing
      success = mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya); //(0 = using CMD_AUTH_A)

      UARTprintf("Sector 1 (Blocks 4..7) Authentication:");
      if (success){
        UARTprintf("[Success]\r\n");
        u8 data[16];

        // If you want to write something to block 4 to test with, uncomment
        // the following line and this text should be read back in a minute
        //memcpy(data, (const u8[]){ 'a', 'd', 'a', 'f', 'r', 'u', 'i', 't', '.', 'c', 'o', 'm', 0, 0, 0, 0 }, sizeof data);
        //success = mifareclassic_WriteDataBlock (4, data);

        // Try to read the contents of block 4
        success = mifareclassic_ReadDataBlock(4, data);
        UARTprintf(">Read Block 4:");
        if (success) {
          // Data seems to have been read ... spit it out
          UARTprintf("[Success] =");
          for(i=0;i<16;i++)         //nfc.PrintHexChar(data, 16);
        	  UARTprintf(" %02x", data[i]);
          UARTprintf("\r\n");
          // Wait a bit before reading the card again
    	  LED_RED_ON532;
    	  SPKON532;
   	  	  somedelay(1000000);
   		  LED_RED_OFF532;
   		  SPKOFF532;


        }
        else{
          UARTprintf("[Ooops] Unable to read the requested block.  Try another key?\r\n");
        }
      }
      else{
        UARTprintf("[Ooops] Authentication failed: Try another key?\r\n");
      }
    }

    if (uidLength == 7){
      // We probably have a Mifare Ultralight card ...
      UARTprintf("Seems to be a Mifare Ultralight tag (7 byte UID)\r\n");

      // Try to read the first general-purpose user page (#4)
      UARTprintf(">Reading page 4...");
      u8 data[32];
      success = mifareultralight_ReadPage (4, data);
      if (success)
      {
        // Data seems to have been read ... spit it out
    	UARTprintf("[Success]:");
        for(i=0;i<4;i++)
    		  UARTprintf(" %02x",data[i]);//nfc.PrintHexChar(data, 4);
        UARTprintf("\r\n");

        // Wait a bit before reading the card again
        somedelay(1000);
      }
      else {
        UARTprintf("[Ooops] Unable to read the requested page!?\r\n");
      }
    }
  }
}

void PN532mainLoop(){
	PN532GpioConf();
	PN532Config();
    GuiInit("PN532 NFC Test");
	while(1) PN532ReadTagLoopForISO14443A();
}
