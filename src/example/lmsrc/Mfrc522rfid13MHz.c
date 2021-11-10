#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "ylib/yInc.h"
//NXP's MFRC522 Driver (13.52MHz RFID - Mifare 1K (one))
//No interrupt Routine
// Description: Mifarel find cards, anti-collision - election card  to read and write interface
//
/* The SPI is coded with Bitbang Style...
  SCK:PA2
  nSS: PE0 :
 MISO:PA4
 MOSI:PA5
 IRQ : PA6(NA)
 nRST: TBD
 LED: PA7
 BUZZER : PB1(Pin34) --> Will be changed to P?
*/
//Bit-bang SPI
////////////////////GPIO PIN Mapping /////////////////////////////////////////////////
//Pin24: nSS (otuput by MCU)
#define NSS522_1      {GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);}
#define NSS522_0      {GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, ~GPIO_PIN_0);}
//pin29: SCK (otuput by MCU)
#define SCK522_1       {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);}
#define SCK522_0       {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, ~GPIO_PIN_2);}
//pin30: MOSI (otuput by MCU)
#define SI522_1        {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);}
#define SI522_0        {GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, ~GPIO_PIN_5);}
//pin 31: MISO (Input by MCU)
#define SO522         ((GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == GPIO_PIN_4) ? 1:0)
//pin 6: nRESET (Output by MCU)
//#define RST522_1       P1OUT |=  BIT3        //nRSTwithPowerDown(NRSTPD, Pin 6)
//#define RST522_0       P1OUT &=~ BIT3
#define SPKON           {GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);}
#define SPKOFF           {GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, ~GPIO_PIN_1);}
//LED
#define LED_RED_ON   GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
#define LED_RED_OFF  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, ~GPIO_PIN_7);

////////////////////////////////////////////////////////////////////////////////////////////////////
//MF522 Commands
#define PCD_IDLE               0x00               //No Action, Cancel Current Execution
#define PCD_AUTHENT            0x0E               //Authentication Key
#define PCD_RECEIVE            0x08               //Activate Rx Circuits
#define PCD_TRANSMIT           0x04               //Transmit data from FIFO buffer to antenna
#define PCD_TRANSCEIVE         0x0C               //Transmit data from FIFO buffer to antenna, and activate Receiver after transmitting
#define PCD_SOFTRESET         0x0F               //SoftReset
#define PCD_CALCCRC            0x03               //Activate CRC copressor or perform self test

//Mifare_One Frame Commands (Software Frame Format)
// Mifare_One card command word
#define PICC_REQIDL 0x26 // look for antenna region does not enter hibernation
#define PICC_REQALL 0x52 // look for the antenna all the cards in the region
#define PICC_ANTICOLL 0x93 // anti-collision
#define PICC_SElECTTAG 0x93 // election card
#define PICC_AUTHENT1A 0x60 // Verify A key
#define PICC_AUTHENT1B 0x61 // verify the B key
#define PICC_READ 0x30 // read block
#define PICC_WRITE 0xA0 // write block
#define PICC_DECREMENT 0xC0 // chargeback
#define PICC_INCREMENT 0xC1 // recharge
#define PICC_RESTORE 0xC2 //adjust the block data to buffer
#define PICC_TRANSFER 0xB0 //save the buffer data
#define PICC_HALT 0x50 // sleep

//MF522 FIFO Depth
#define DEF_FIFO_LENGTH       64                 //FIFO size=64byte
#define MAXRLEN 18

//MF522 Registers
// PAGE 0
#define     RFU00                 0x00
#define     CommandReg            0x01
#define     ComIEnReg             0x02
#define     DivlEnReg             0x03
#define     ComIrqReg             0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     RFU0F                 0x0F
// PAGE 1
#define     RFU10                 0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14 //3:Tx2CW(Output on Tx2 continuous wave with unmodulated 13.56MHz energy carrier); 1:Tx2REFn ; 0:Tx1REFn
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     RFU1A                 0x1A
#define     RFU1B                 0x1B
#define     MifareReg             0x1C
#define     RFU1D                 0x1D
#define     RFU1E                 0x1E
#define     SerialSpeedReg        0x1F
// PAGE 2
#define     RFU20                 0x20
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     RFU23                 0x23
#define     ModWidthReg           0x24
#define     RFU25                 0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsCfgReg            0x28
#define     ModGsCfgReg           0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
// PAGE 3
#define     RFU30                 0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     RFU3C                 0x3C
#define     RFU3D                 0x3D
#define     RFU3E                 0x3E
#define     RFU3F   0x3F

// And the MF522 communication error code returned
#define MI_OK                          0
#define MI_NOTAGERR                    1
#define MI_ERR                         2

//Prototypes
void Mfrc522Main(void);
//void DelayMs(unsigned int _MS);

char MFRC522_Init(void); //RC522 Reset
void PcdAntennaOn(void); //
void PcdAntennaOff(void);
char MFRC522_Request(unsigned char req_code,unsigned char *pTagType);
char PcdAnticoll(unsigned char *pSnr);
u8 MFRC522_SelectTag(unsigned char *pSnr);
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr);
char PcdRead(unsigned char addr,unsigned char *pData);
char PcdWrite(unsigned char addr,unsigned char *pData);
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue);
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr);
char PcdHalt(void);
char MFRC522_ToCard(unsigned char Command,
                 unsigned char *pInData,
                 unsigned char InLenByte,
                 unsigned char *pOutData,
                 unsigned int  *pOutLenBit);
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData);
void Write_MFRC522(unsigned char Address,unsigned char value);
unsigned char Read_MFRC522(unsigned char Address);
void SetBitMask(unsigned char reg,unsigned char mask);
void ClearBitMask(unsigned char reg,unsigned char mask);


// RC522 register bit
void SetBitMask(unsigned char reg,unsigned char mask)
{
    char tmp = 0x0;
    tmp = Read_MFRC522(reg); //maskSET
    Write_MFRC522(reg,tmp | mask);  // set bit mask
}
//clear RC522 register bit
void ClearBitMask(unsigned char reg,unsigned char mask)  {
    char tmp = 0x0;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & ~mask);  // clear bit mask
}

//Read RC522 Register Value
unsigned char Read_MFRC522(unsigned char Address){
     unsigned char i, ucAddr;
     unsigned char ucResult=0;

     //Data bit is latched on rising edge.
     SCK522_0; //SCK=0 //Idle...
     NSS522_0; //nSS=0
     ucAddr = ((Address<<1)&0x7E)|0x80; //For Read MSB=1 + Address (for 0x37--> 0xee)
	 //UARTprintf("ucAddr=%0x(for %0x)\r\n",ucAddr,Address);
	 //UARTSend((unsigned char *)ostr, strlen((char *)ostr));

     for(i=8;i>0;i--)  {
         if ((ucAddr&0x80)==0x80) {   SI522_1;   } //MOSI=1
         else         {   SI522_0;   } //MOSI=0
         SCK522_1;//SCK=1
         ucAddr <<= 1;
         SCK522_0;//SCK=0
     }

     somedelay(10);
     //read data
     for(i=8;i>0;i--)     {
         SCK522_1; //SCK=1
         ucResult <<= 1;
         SI522_0; //dummy output(0)
         ucResult |= SO522;
         //GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4);// SO522 ;//MISO
         SCK522_0; //SCK=0
     }
     SCK522_0; //SCK=0 //Idle...
     NSS522_1; //nSS=1
     //SCK522_1; //SCK=1
     somedelay(1000);
     return ucResult;
}

//Write a value on Register
void Write_MFRC522(unsigned char Address, unsigned char value)
{
    unsigned char i, ucAddr;


    SCK522_0; //SCK=0; -- Idle

    NSS522_0; //nSS=0
    ucAddr = ((Address<<1)&0x7E); //MSB=0; LSB=0

    for(i=8;i>0;i--)  {
        if ((ucAddr&0x80)==0x80)  {   SI522_1;   } //MOSI=1
        else        {   SI522_0;   } //MOSI=0
        SCK522_1; //SCK=1
        ucAddr <<= 1;
        SCK522_0; //SCK=0
    }
    somedelay(10);
    for(i=8;i>0;i--)  {
        if ((value&0x80)==0x80)  {   SI522_1;   } //MOSI=1
        else        {   SI522_0;   } //MOSI=0
        SCK522_1; //SCK=1
        value <<= 1;
        SCK522_0; //SCK=0
    }
    SCK522_0; //SCK=0 //Idle
    NSS522_1; //nSS=1
    //SCK522_1; //SCK=1

}

/* Description: look for the card, read the card type
 * Enter parameters: reqMode, - to find the card the way,
 * TagType - return type of card
 * 0x4400 = Mifare_UltraLight
 * 0x0400 = Mifare_One (S50)
 * 0x0200 = Mifare_One (S70)
 * 0x0800 = Mifare_Pro (X)
 * 0x4403 = Mifare_DESFire
 * Return values: successful return MI_OK
*/

char MFRC522_Request(unsigned char req_code,unsigned char *TagType){
   char status;
   unsigned int  backBits; //data bits to be received

   //ClearBitMask(Status2Reg,0x08);  //.3=MFCrypto1On(0=Clear)
   Write_MFRC522(BitFramingReg,0x07); //.2.1.0=TxLastBits[2:0] = the number of bits of the last byte that will be txed. (000=All bits of the last byte will be txed)
   //SetBitMask(TxControlReg,0x03);  // bit1=Tx2REFn and bit0=Tx1REFn (output signal Tx2 and Tx1 delivers 13.56MHz energy carrier modulated by tx data.)

   TagType[0] = req_code; //MIFARE Command
   status = MFRC522_ToCard(PCD_TRANSCEIVE,TagType,1,TagType,&backBits);

   if ((status != MI_OK) || (backBits != 0x10))   {   status = MI_ERR;   }
   return status;
}
/*
 * Function: MFRC522_Anticoll
 * Description: anti-collision detection, and read the card serial number of the selected card
 * Input parameters: serNum - return to the 4-byte card serial number, + the last byte is checksum byte
 * Return values: successful return MI_OK
 */
//Anti-Collision
char PcdAnticoll(unsigned char *pSnr)
{
    char status;
    unsigned char i,snr_check=0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ClearBitMask(Status2Reg,0x08);
    Write_MFRC522(BitFramingReg,0x00);
    ClearBitMask(CollReg,0x80);

    ucComMF522Buf[0] = PICC_ANTICOLL;//0x93(ISO14443-3 AntiCollision) : Cascade Level 1;
    ucComMF522Buf[1] = 0x20; //0x20=ANTICOLL(ISO14443-3)

    status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    //return from PICC : CT(0x88)|UID[3]
    if (status == MI_OK)    {
      for (i=0; i<4; i++)   {
             *(pSnr+i)  = ucComMF522Buf[i];
             snr_check ^= ucComMF522Buf[i];
      }
      if (snr_check != ucComMF522Buf[i]) //BCC1 check
      	    status = MI_ERR;
    }

    SetBitMask(CollReg,0x80);
    return status;
}

u8 MFRC522_SelectTag(unsigned char *pSnr)
{
    u8 size, status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_SElECTTAG; //0x93(ISO14443-3 AntiCollision)//==PICC_ANTICOLL;//Cascade Level 1;
    ucComMF522Buf[1] = 0x70; //0x70=SELECT(ISO14443-3)
    ucComMF522Buf[6] = 0;//BCC reset
    for (i=0; i<4; i++)    { //echo back to PICC
     ucComMF522Buf[i+2] = *(pSnr+i);
     ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
    ClearBitMask(Status2Reg,0x08);

    //0x93 0x70(SELECT) 0x88(CT) | SN0,1,2 | BCC | CRC0 | CRC1
    status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);

    if ((status == MI_OK) && (unLen == 0x18))    {
    	size = ucComMF522Buf[0];
    }
    else    {   size = 0;    }

    return size;
}

/*
u8 MFRC522_SelectTag (u8 * serNum)
{
    u8, i;
u8, the status;
u8, size;
    uint recvBits;
    u8, buffer [9];

/ / ClearBitMask (Status2Reg, 0x08); / / MFCrypto1On = 0

    buffer [0] = PICC_SElECTTAG;
    buffer [1] = 0x70;
    for (i = 0; i <5; i + +)
    {
    buffer [i +2] = * (serNum + i);
    }
CalulateCRC (buffer, 7, & buffer [7]); / /??
    status = MFRC522_ToCard (PCD_TRANSCEIVE, buffer, 9, buffer, & recvBits);

    if ((status == MI_OK) && (recvBits == 0x18))
    {
size = buffer [0];
}
    else
    {
size = 0;
}

    return size;
}
*/
/*
 * Function: MFRC522_Auth
 * Description: Verify that the card password
 * Input parameters: authMode - Password Authentication Mode
                 0x60 = verify A key
                 0x61 = verify the B key
             BlockAddr - block address
             Sectorkey - sectors password
             serNum - Card serial number, 4 bytes
 * Return values: successful return MI_OK
 */
//auth_mode[IN]:
//                 0x60 = A
//                 0x61 = B
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr) //Snr=serNum
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr; //BlockAddr
    for (i=0; i<6; i++)    {    ucComMF522Buf[i+2] = *(pKey+i);   } //SectorKey
    for (i=0; i<6; i++)    {    ucComMF522Buf[i+8] = *(pSnr+i);   }

    status = MFRC522_ToCard(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen); //unLen=recvBits
    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }

    return status;
}

//Receive MIFARE msg
char PcdRead(unsigned char addr,unsigned char *pData){
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_READ; //0x30 MIFARE READ COMMAND
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))    {
        for (i=0; i<16; i++)       {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else    {   status = MI_ERR;   }

    return status;
}

//MIFARE
/* Description: Write block data
 * Enter parameters: blockAddr, - block address; writeData to write 16 bytes of data - to block
 * Return values: successful return MI_OK
*/
char PcdWrite(unsigned char addr,unsigned char *pData){
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_WRITE; //0xA0 MIFARE WRITE COMMAND
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))    {   status = MI_ERR;   }

    if (status == MI_OK)    {
        for (i=0; i<16; i++)        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))        {   status = MI_ERR;   } //ulLen=bits
    }

    return status;
}

//dd_mode[IN]
//               0xC0 = ee
//               0xC1 =
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    if (status == MI_OK)  {
        for (i=0; i<16; i++)      {    ucComMF522Buf[i] = *(pValue+i);   }
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
        unLen = 0;
        status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)      {    status = MI_OK;    }
    }

    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

        status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))     {   status = MI_ERR;   }
    }
    return status;
}

char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))    {   status = MI_ERR;   }

    if (status == MI_OK)    {
        ucComMF522Buf[0] = 0;
        ucComMF522Buf[1] = 0;
        ucComMF522Buf[2] = 0;
        ucComMF522Buf[3] = 0;
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);

        status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)        {    status = MI_OK;    }
    }

    if (status != MI_OK)
    {    return MI_ERR;   }

    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))    {   status = MI_ERR;   }

    return status;
}

// Description: command card into hibernation
char PcdHalt(void)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return MI_OK;
}

//MF522 CRC16
/* Function: CalulateCRC
 * Function Description: MF522 calculate the CRC
 * Input parameters: pIndata - to be reading a CRC data, len - the length of the data, pOutData - calculated CRC results
 * Return value:
 */
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData){
    unsigned char i,n;
    ClearBitMask(DivIrqReg,0x04);
    Write_MFRC522(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)    {   Write_MFRC522(FIFODataReg, *(pIndata+i));   }
    Write_MFRC522(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do   {
        n = Read_MFRC522(DivIrqReg);
        i--;
    }  while ((i!=0) && !(n&0x04));
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegM);
}

//Reset RC522
char MFRC522_Init(){
	u8 ver;
    //RST522_1; //nRST=1
    //_NOP();
    //RST522_0; //nRST=0
    //_NOP();
    //RST522_1; //nRST=1
    //_NOP();
    Write_MFRC522(CommandReg,PCD_SOFTRESET); //Issue Reset Command
    //_NOP();
    somedelay(100);

// Timer: TPrescaler * TreloadVal/6.78MHz = 24ms
    Write_MFRC522 (TModeReg, 0x8D); // Tauto = 1; f(Timer) = 6.78MHz/TPreScaler
    Write_MFRC522 (TPrescalerReg, 0x3E); // TModeReg [3 .. 0] + TPrescalerReg
    Write_MFRC522 (TReloadRegL, 30);
    Write_MFRC522 (TReloadRegH, 0);

    Write_MFRC522 (TxAutoReg, 0x40); // 100% ASK
    Write_MFRC522 (ModeReg, 0x3D); // CRC initial value of 0x6363???

    // ClearBitMask (Status2Reg, 0x08); / / MFCrypto1On = 0
    // Write_MFRC522 (RxSelReg, 0x86); / / RxWait = RxSelReg [5 .. 0]
    // Write_MFRC522 (RFCfgReg, 0x7F); / / RxGain = 48dB

    PcdAntennaOn();//AntennaOn (); // open the antenna

//    Write_MFRC522(ModeReg,0x3D); //
//    Write_MFRC522(TReloadRegL,30);
//    Write_MFRC522(TReloadRegH,0);
//    Write_MFRC522(TModeReg,0x8D);
//    Write_MFRC522(TPrescalerReg,0x3E);

	 ver = Read_MFRC522(VersionReg);
	 UARTprintf("MFRC522> Ver=%0x(expected = 0x91 or 0x92)\r\n",ver);

    return MI_OK;
}

/*
 * Letter: MFRC522_ToCard
 * Description: RC522 and ISO14443 card communication
 * Input Parameters: command - MF522 command word,
 * SendData - RC522 sent to the card data
 * sendLen - Send data length
 * BackData card - received data is returned,
 * BackLen - Returns the length of the data bits
 * Return values: successful return MI_OK
 */
char MFRC522_ToCard(unsigned char Command,
                 unsigned char *pInData,
                 unsigned char InLenByte,
                 unsigned char *pOutData,
                 unsigned int  *pOutLenBit)
{
    char status = MI_ERR;
    unsigned char irqEn   = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    unsigned int i;
    switch (Command)
    {
       case PCD_AUTHENT:
          irqEn   = 0x12;
          waitFor = 0x10;
          break;
       case PCD_TRANSCEIVE:
          irqEn   = 0x77;
          waitFor = 0x30;
          break;
       default:
         break;
    }

    Write_MFRC522(ComIEnReg,irqEn|0x80); //allow the interrupt request
    ClearBitMask(ComIrqReg,0x80); //clear all int req bit
    //Write_MFRC522(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);//flush buffer 1, FIFO Init.
    Write_MFRC522(CommandReg,PCD_IDLE); //No Action to cancel the current

    //Write data into the buffer
    for (i=0; i<InLenByte; i++)    {   Write_MFRC522(FIFODataReg, pInData[i]);    } //FIFO MIFARE
    //Execute the command
    Write_MFRC522(CommandReg, Command); //
    if (Command == PCD_TRANSCEIVE)    {    SetBitMask(BitFramingReg,0x80);  } //Start transmission

    //Wait for data to be received
    i = 600;//425ms
    do {
         n = Read_MFRC522(ComIrqReg);
         i--;
    }while ((i!=0) && !(n&0x01) && !(n&waitFor));

    ClearBitMask(BitFramingReg,0x80); //StartSend=0(Stop)

    if (i!=0){
         if(!(Read_MFRC522(ErrorReg)&0x1B)){ //No Error (BufferOverflow, CollisionErr, CRCerr, ProtectErr)
             status = MI_OK;
             if (n & irqEn & 0x01)     {   status = MI_NOTAGERR;   }
             if (Command == PCD_TRANSCEIVE) {
                n = Read_MFRC522(FIFOLevelReg);
                lastBits = Read_MFRC522(ControlReg) & 0x07;
                if (lastBits)     {   *pOutLenBit = (n-1)*8 + lastBits;   }
                else               {   *pOutLenBit = n*8;   }

                if (n == 0)        {   n = 1;    }
                if (n > MAXRLEN)       {   n = MAXRLEN;   }
                //Read the rxed data in the FIFO
                for (i=0; i<n; i++)      {   pOutData[i] = Read_MFRC522(FIFODataReg);    }
                //UARTprintf("MI_OK\r\n");
                //UARTSend((unsigned char *)ostr, strlen((char *)ostr));
            }
         } else{
        	 status = MI_ERR;
        	 UARTprintf("MI_ERR\r\n");
         }
   }
   SetBitMask(ControlReg,0x80);           // stop timer now
   Write_MFRC522(CommandReg,PCD_IDLE);
   //UARTprintf("Status=0x%02x\r\n",status);
   //UARTSend((unsigned char *)ostr, strlen((char *)ostr));
   return status;
}

/////////////////////////////////////////////////////////////////////
void PcdAntennaOn(){ //Tx Energy wave ON
    unsigned char i;
    i = Read_MFRC522(TxControlReg);
    if (!(i & 0x03)){ //ON
    	UARTprintf("Antenna ON(TxControlReg = 0x%02x)\r\n",i);
        SetBitMask(TxControlReg, 0x03);
    }
}

void PcdAntennaOff(){ //Tx Energy wave OFF
	UARTprintf("Antenna OFF\r\n");
    ClearBitMask(TxControlReg, 0x03);
}

//#pragma memory = constseg(TAB)
unsigned char data1[16] = {0x12,0x34,0x56,0x78,0xED,0xCB,0xA9,0x87,0x12,0x34,0x56,0x78,0x01,0xFE,0x01,0xFE};
//M1
unsigned char data2[4]  = {0x12,0,0,0};
unsigned char DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//#pragma memory=default
unsigned char g_ucTempbuf[20];

//MCU GPIO
void Mfrc522GpioConf(void)
{
    //Stop watchdog timer
	//nSS: PE0 :
	// SCK:PA2
	// MOSI:PA5
	// MISO:PA4
	// IRQ : PA6(NA)
	// nRST: TBD
	// LED : PA7
	// SPK : PB1 (to be changed)

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);//nSS
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 );//SCK
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5 ); //MOSI
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4); //MISO
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6); //IRQ
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7); //LED
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1); //SPK
	//Init
	somedelay(100);

}

u8 mifare_tag_type(u8 *b){
	if(b[0] == 0x44 && b[1] == 0x00)
		UARTprintf(ostr,"TagType=MifareUltralight\r\n");
	else if(b[0] == 0x44 && b[1] == 0x03)
		UARTprintf(ostr,"TagType=Mifare_DESFire\r\n");
	else if(b[0] == 0x04 && b[1] == 0x00)
		UARTprintf(ostr,"TagType=MifareOne(S50)\r\n");
	else if(b[0] == 0x02 && b[1] == 0x00)
		UARTprintf(ostr,"TagType=MifareOne(S70)\r\n");
	else if(b[0] == 0x08 && b[1] == 0x00)
		UARTprintf(ostr,"TagType=MifarePro(x)\r\n");

	return 0;
}
// 4-byte card serial number, 5 byte checksum byte
u8 serNum [5];
u8 writeData[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100}; // initialize the 100 yuan
u8 moneyConsume = 18; // consumption of 18 yuan
u8 moneyAdd = 10; // recharge 10 yuan
// Sector A password, 16 sectors, each sector password 6Byte
u8 sectorKeyA [16][16] = {
		 {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
         {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
         //{0x19, 0x84, 0x07, 0x15, 0x76, 0x14},
         {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
         {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                            };
 u8 sectorNewKeyA [16][16] = {
		 {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
         {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xff, 0x07, 0x80, 0x69, 0x19, 0x84, 0x07, 0x15, 0x76, 0x14},
         // You can set another key, such as "0x19, 0x84, 0x07, 0x15, 0x76, 0x14"
                          // {0x19, 0x84, 0x07, 0x15, 0x76, 0x14, 0xff, 0x07, 0x80, 0x69, 0x19, 0x84, 0x07, 0x15, 0x76, 0x14},
                          // But when loop, please set the sectorKeyA, the same key, so that RFID module can read the card
         {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xff, 0x07, 0x80, 0x69, 0x19, 0x33, 0x07, 0x15, 0x34, 0x14},
                             };

 void Mfrc522Loop(void){
     unsigned char status, ver, size;
     u8 uid[5];

     Mfrc522GpioConf(); //MCU GPIO
     LED_RED_ON;
     somedelay(1000000);
     LED_RED_OFF;

     GuiInit("MFRC522 RFID Test");
     MFRC522_Init();
/*
     Write_MFRC522(CommandReg,PCD_SOFTRESET); //Issue SoftReset Command
     somedelay(100);

 // Timer: TPrescaler * TreloadVal/6.78MHz = 24ms
     Write_MFRC522 (TModeReg, 0x8D); // Tauto = 1; f (Timer) = 6.78MHz/TPreScaler
     Write_MFRC522 (TPrescalerReg, 0x3E); // TModeReg [3 .. 0] + TPrescalerReg
     Write_MFRC522 (TReloadRegL, 30);
     Write_MFRC522 (TReloadRegH, 0);

     Write_MFRC522 (TxAutoReg, 0x40); // 100% ASK
     Write_MFRC522 (ModeReg, 0x3D); // CRC initial value of 0x6363???

     ClearBitMask (Status2Reg, 0x08); // MFCrypto1On = 0
     Write_MFRC522 (RxSelReg, 0x86); // RxWait = RxSelReg [5 .. 0]
     Write_MFRC522 (RFCfgReg, 0x7F); // RxGain = 48dB

     PcdAntennaOn();//AntennaOn (); // open the antenna
*/
     somedelay(10000);
     //while(1){
    	 ver = Read_MFRC522(VersionReg);
    	 UARTprintf(ostr,"MFRC522> Ver=%0x(expected = 0x91(v1) or 0x92(v2))\r\n",ver);
    	 somedelay(1000);
   //}



 //    Write_MFRC522(ModeReg,0x3D); //
 //    Write_MFRC522(TReloadRegL,30);
 //    Write_MFRC522(TReloadRegH,0);
 //    Write_MFRC522(TModeReg,0x8D);
 //    Write_MFRC522(TPrescalerReg,0x3E);
     //PcdAntennaOff();
     //PcdAntennaOn();   //Energy Wave


     while (1){
    	 //Get TagType
    	 status = MFRC522_Request(PICC_REQIDL, g_ucTempbuf); //status = MFRC522_Request(PICC_REQALL, g_ucTempbuf);
         if (status != MI_OK)   {
             LED_RED_OFF;
             continue;
         }
         mifare_tag_type(g_ucTempbuf);
         UARTprintf("Find out a card=0x%02x:0x%02x\r\n",g_ucTempbuf[0],g_ucTempbuf[1]);
         //UARTSend((unsigned char *)ostr, strlen((char *)ostr));
         //UARTPrintf("Find out a card=%0x:%0x\\r\\n",g_ucTempbuf[0],g_ucTempbuf[1] );

         LED_RED_ON;
    	 SPKON;
    	 somedelay(100000);
    	 SPKOFF;
    	 LED_RED_OFF;

         status = PcdAnticoll(g_ucTempbuf);
         if (status != MI_OK)      {    continue;    }
         //memcpy(uid, g_ucTempbuf,5);
         //UARTPrintf("SerialNum[0]=%0x %0x : %0x\\r\\n",g_ucTempbuf[0],g_ucTempbuf[1] ,g_ucTempbuf[2]  ); //,,,
         UARTprintf("SerialNum[0]=%0x %0x %0x %0x %0x\r\n",g_ucTempbuf[0],g_ucTempbuf[1],g_ucTempbuf[2]  ,g_ucTempbuf[3],g_ucTempbuf[4]);
         //UARTSend((unsigned char *)ostr, strlen((char *)ostr));

         // Election card, return the card capacity
         size = MFRC522_SelectTag(g_ucTempbuf);//uid);
         if (size == 0)         {    continue;    }
         UARTprintf("MFRC522_SelectTag. Size=%d\r\n", size);
         //UARTSend((unsigned char *)ostr, strlen((char *)ostr));

         status = PcdAuthState(PICC_AUTHENT1A, 1, DefaultKey, g_ucTempbuf);
         if (status != MI_OK)         {    continue;    }
         UARTprintf("PcdAuthState\r\n");

         status = PcdWrite(1, data1);
         if (status != MI_OK)         {    continue;    }
         UARTprintf("PcdWrited\r\n");

         status = PcdValue(PICC_DECREMENT,1,data2);
         if (status != MI_OK)         {    continue;    }

         status = PcdBakValue(1, 2);
         if (status != MI_OK)         {    continue;    }

         status = PcdRead(2, g_ucTempbuf);
         if (status != MI_OK)         {    continue;    }
         PcdHalt();
    }
}
/*
 void Mfrc522Test(void){
      unsigned char status;

      Mfrc522GpioConf(); //MCU GPIO
      LED_RED_ON;
      somedelay(1000000);
      LED_RED_OFF;
      MFRC522_Init();
      //PcdAntennaOff();
      //PcdAntennaOn();   //Energy Wave
      while (1){
     	 somedelay(100000);
     	 status = MFRC522_Request(PICC_REQIDL, g_ucTempbuf); //        status = MFRC522_Request(PICC_REQALL, g_ucTempbuf);
          if (status != MI_OK)   {
              LED_RED_OFF;
              continue;
          }
          UARTprintf("Find out a card=%0x:%0x\r\n",g_ucTempbuf[0],g_ucTempbuf[1]);
          UARTSend((unsigned char *)ostr, strlen((char *)ostr));
          //UARTPrintf("Find out a card=%0x:%0x\\r\\n",g_ucTempbuf[0],g_ucTempbuf[1] );

          LED_RED_ON;

          status = PcdAnticoll(g_ucTempbuf);
          if (status != MI_OK)      {    continue;    }
          //UARTPrintf("SerialNum[0]=%0x %0x : %0x\\r\\n",g_ucTempbuf[0],g_ucTempbuf[1] ,g_ucTempbuf[2]  ); //,,,
            // Election card, return the card capacity
          status = MFRC522_SelectTag(g_ucTempbuf);
          if (status != MI_OK)         {    continue;    }

          status = PcdAuthState(PICC_AUTHENT1A, 1, DefaultKey, g_ucTempbuf);
          if (status != MI_OK)         {    continue;    }

          status = PcdWrite(1, data1);
          if (status != MI_OK)         {    continue;    }

          status = PcdValue(PICC_DECREMENT,1,data2);
          if (status != MI_OK)         {    continue;    }

          status = PcdBakValue(1, 2);
          if (status != MI_OK)         {    continue;    }

          status = PcdRead(2, g_ucTempbuf);
          if (status != MI_OK)         {    continue;    }
          PcdHalt();
     }
 }
*/
