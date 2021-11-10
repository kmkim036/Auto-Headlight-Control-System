#include "yLib/eth/include/ethopts.h"
#include "yInc.h"
//#if ((PROCESSOR == STM32F103C8) || (PROCESSOR == STM32F107VCT6))
#if (ETH_IF_MODE == ETH_IF_MII_MODE)
#include <string.h>
//#include "driverlib/debug.h"
//#include "inc/hw_nvic.h"
//#include "inc/hw_ints.h"
//#include "inc/hw_memmap.h"
//#include "inc/hw_types.h"
//#include "driverlib/ethernet.h"
//#include "driverlib/sysctl.h"
#include "stm_lib/inc/stm32f10x_gpio.h"
//#include "stm_lib/inc/stm32f10x_interrupt.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/systick.h"
//#include "inc/hw_ethernet.h"
//#include "utils/uartstdio.h"

#define ETHERNET_INT_PRIORITY 0x80

void yEtherSimpleLoop();

void InitEtherWOinterrupt(u8 myMACAddr[]) //Without Interrupt
{
	unsigned char pucMACAddress[6];

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH); //Enable PHY and MAC in MCU
	SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);
	IntPrioritySet(INT_ETH, ETHERNET_INT_PRIORITY);
	EthernetInitExpClk(ETH_BASE, SysCtlClockGet());

	// Configure the Ethernet controller for normal operation
	// Enable TX Duplex Mode ; Enable TX Padding, auto CRC
	EthernetConfigSet(ETH_BASE, (ETH_CFG_TX_DPLXEN | ETH_CFG_TX_PADEN | ETH_CFG_TX_CRCEN));
	// Set my MAC Address (01-23-45-67-89-AB)
	EthernetMACAddrSet(ETH_BASE, myMACAddr);
	// Enable the Ethernet controller
	EthernetEnable(ETH_BASE);

	//In addition,... GPIOFÃƒÂ¬Ã¢â‚¬â€�Ã¯Â¿Â½ ÃƒÂ¬Ã…Â¾Ã‹â€ ÃƒÂ«Ã…Â Ã¢â‚¬ï¿½ LED0, LED1 ÃƒÂ­Ã¢â‚¬Â¢Ã¢â€šÂ¬ÃƒÂ¬Ã¯Â¿Â½Ã¢â‚¬Å¾ ethernet moduleÃƒÂ¬Ã¢â‚¬â€�Ã¯Â¿Â½ÃƒÂ¬Ã¢â‚¬Å¾Ã…â€œ ÃƒÂ¬Ã‚Â§Ã¯Â¿Â½ÃƒÂ¬Ã‚Â Ã¢â‚¬Ëœ ÃƒÂ¬Ã‚Â Ã…â€œÃƒÂ¬Ã¢â‚¬â€œÃ‚Â´ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â´ÃƒÂ¬Ã¢â‚¬Å¾Ã…â€œ ethernet status LEDÃƒÂ«Ã‚Â¡Ã…â€œ ÃƒÂ¬Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¬Ã…Â¡Ã‚Â©ÃƒÂ­Ã¢â‚¬Â¢Ã‹Å“ÃƒÂ«Ã¯Â¿Â½Ã¢â‚¬Å¾ÃƒÂ«Ã‚Â¡Ã¯Â¿Â½ ÃƒÂ¬Ã¢â‚¬Å¾Ã‚Â¤ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢.(see Manual p.626)
    // Enable Port F for Ethernet LEDs.
    //  LED0        Bit 3   Output
    //  LED1        Bit 2   Output
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	EthernetPHYWrite(ETH_BASE, PHY_MR23, PHY_MR23_LED1_LINK | PHY_MR23_LED0_RXTX);
	somedelay(10000);
	UARTprintf("Init Ether Done.\r\n");
}

void yEtherSimpleLoop()
{
	u8 pucMyTxPacket[64],pucMyRxPacket[1518],i;
	u8 myMACAddr[6];
	long rxlen;

	u32 ulTemp;
	u32 ulMyTxPacketLength;

	GuiInit("SimpleEthernet Test");
	//myMacAddress
	myMACAddr[0] = 0x01;
	myMACAddr[1] = 0x23;
	myMACAddr[2] = 0x45;
	myMACAddr[3] = 0x67;
	myMACAddr[4] = 0x89;
	myMACAddr[5] = 0xAB;

	InitEtherWOinterrupt(myMACAddr); //Set myMAC addr
	//EthernetMACAddrGet(ETH_BASE, myMACAddr); //Get MAC Addr

	//Add Multicast enable option
    ulTemp = EthernetConfigGet(ETH_BASE);
    ulTemp |= ETH_CFG_RX_AMULEN; //Mulicast Enable
    EthernetConfigSet(ETH_BASE, ulTemp);

    //make a packet
    //DA
    for(i=0;i<6;i++)
    	pucMyTxPacket[i] = 0xFF;
    //SA
    for(i=0;i<6;i++)
    	pucMyTxPacket[i+6] = myMACAddr[i];
    //EtherType[2]
    pucMyTxPacket[12] = 0x08;
    pucMyTxPacket[12] = 0x06; //ARP
    //You must fill the remainings..

	// Send a packet.
	// (assume that the packet has been filled in appropriately.
    ulMyTxPacketLength = 64;
	EthernetPacketPut(ETH_BASE, pucMyTxPacket, ulMyTxPacketLength);

	while(1){
		// Wait for a packet to come in.
		rxlen = EthernetPacketGet(ETH_BASE, pucMyRxPacket, sizeof(pucMyRxPacket));
		UARTprintf("Got a Frame with Ethertype of 0x%02x02x (len=%dbytes)",pucMyRxPacket[12], pucMyRxPacket[13],rxlen);
	}
}

unsigned long dtEtherGetMii(unsigned char ucRegAddr)
{
	unsigned long ret;
	ret = EthernetPHYRead(ETH_BASE, ucRegAddr);
}
#endif
