#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__
#include "yInc.h"

//Media Converter for Automotive Ethernet
struct _mcConfig{
	u8 fForced_or_AN; //1-0
	u8 fMaster_or_Slave;
	u8 LinkUp;
	u16 reg1_status; //[2]=1 linkup
	u16 regf_extstatus; //[9]=local rx ok;[8]=retmoe rx ok
	u16 reg11_phyextstatus; //[13]=mdix; [11]=retmoe rx ok; [10]=local rx ok;[9]=descambled locked ;[8]=LinkPassed
	u16 reg1a_intstatus;//[14]=pair swap;[5]=remote rx status changed;[4]=local rx status changed;[2]=link speed changed;[1]=link status changed;[0]=crc error
	u16 reg1c08_ledstatus; //Reg1c with shadow 01000: [8]=1master;[7]=0:FDX; [6]=1=No INT; [4:3]=01=100Mbps,[0]=1=Poor Quality
} mcConfig;


#if (USE_LWIP_PTP)
//#include "yLib/include/err.h"
//#include "lwip/include/netif.h"
//#include "lwip/include/lwipopts.h"
//err_t ethernetif_init(struct netif *netif);
//err_t ethernetif_input(struct netif *netif);

//==================== ADD FOR PTP YOON

struct ptptime_t {
	  long tv_sec;
	  long tv_nsec;
};
void ethernetif_PTPStart(unsigned UpdateMethod, unsigned pps_freq);
void ethernetif_PTPTime_SetTime(struct ptptime_t * timestamp); //Set my current LocalToD.
void ethernetif_PTPTime_GetTime(struct ptptime_t * timestamp); //Get LocalToD is sec since 1900.
void ethernetif_PTPTime_UpdateOffset(struct ptptime_t * timeoffset);
void ethernetif_PTPTime_AdjFreq(int Adj);

/* Examples of subsecond increment and addend values using SysClk = 144 MHz

 Addend * Increment = 2^63 / SysClk
 ptp_tick = Increment * 10^9 / 2^31
 +-----------+-----------+------------+
 | ptp tick  | Increment | Addend     |
 +-----------+-----------+------------+
 |  119 ns   |   255     | 0x0EF8B863 |
 |  100 ns   |   215     | 0x11C1C8D5 |
 |   50 ns   |   107     | 0x23AE0D90 |
 |   20 ns   |    43     | 0x58C8EC2B |
 |   14 ns   |    30     | 0x7F421F4F |
 +-----------+-----------+------------+
*/

/* Examples of subsecond increment and addend values using SysClk = 168 MHz

 Addend * Increment = 2^63 / SysClk
 ptp_tick = Increment * 10^9 / 2^31
 +-----------+-----------+------------+
 | ptp tick  | Increment | Addend     |
 +-----------+-----------+------------+
 |  119 ns   |   255     | 0x0CD53055 |
 |  100 ns   |   215     | 0x0F386300 |
 |   50 ns   |   107     | 0x1E953032 |
 |   20 ns   |    43     | 0x4C19EF00 |
 |   14 ns   |    30     | 0x6D141AD6 |
 +-----------+-----------+------------+
*/

/* Examples of subsecond increment and addend values using 72MHz SysClock
 SysClk = 72MHz
 Addend * Increment = 2^63 / SysClk
 ptp_tick = Increment * 10^9 / 2^31
(See AN3411)
 +-----------+-----------+------------+
 | ptp tick  | Increment | Addend     |
 +-----------+-----------+------------+
 |  119 ns   |   255     | 0x1DF170C7 |
 |  100 ns   |   215     | 0x238391AA |
 |   50 ns   |   107     | 0x475C1B20 |
 |   20 ns   |    43     | 0xB191D856 |
 |   14 ns   |    30     | 0xFE843E9E |
 +-----------+-----------+------------+
*/
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
//20nsec case and 168MHz
#define ADJ_FREQ_BASE_ADDEND     0x4C19EF00//(CORRECT) 0x4C1CB1EA////was 0x58C8EC2B
#define ADJ_FREQ_BASE_INCREMENT   43
#elif(PROCESSOR == PROCESSOR_STM32F107VCT)
//20nsec case and 72MHz
#define ADJ_FREQ_BASE_ADDEND     0xB191D856
#define ADJ_FREQ_BASE_INCREMENT   43

#endif
#endif


#endif
