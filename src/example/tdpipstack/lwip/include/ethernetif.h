#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__


#include "lwip/include/err.h"
//#include "lwip/include/ethernetif.h"
#include "lwip/include/netif.h"
#include "lwip/include/lwipopts.h"
err_t ethernetif_init(struct netif *netif);
err_t ethernetif_input(struct netif *netif);

//==================== ADD FOR PTP YOON
//#if (LWIP_PTP == 1)

#define IFTYPE_IEEE8021AS_EVENT 1
#define IFTYPE_IEEE8021AS_GENERAL 2
#define IFTYPE_IEEE8021BPDU 3

#define ETHTYPE_8021AS 0x88f7
#define ETHTYPE_8021BPDU 0x0042 //TEMPO -- YOON

struct ptptime_t {
	  s32_t tv_sec;
	  s32_t tv_nsec;
};

void ethernetif_PTPTime_SetTime(struct ptptime_t * timestamp); //Set my current LocalToD.
void ethernetif_PTPTime_GetTime(struct ptptime_t * timestamp); //Get LocalToD is sec since 1900.
void ethernetif_PTPTime_UpdateOffset(struct ptptime_t * timeoffset);
void ethernetif_PTPTime_AdjFreq(int32_t Adj);

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

/* Examples of subsecond increment and addend values using 25MHz OSC
 SysClk = 25MHz
 Addend * Increment = 2^63 / SysClk
 ptp_tick = Increment * 10^9 / 2^31

 * Addend = 25MHz
 +-----------+-------------+------------------------+---------------------+
 | ptp tick  | Increment   | Addend(25Mhz)          |Addend(700Mhz)
 +-----------+-------------+------------------------+---------------------+
 |   40 ns   |   85.899(86)| 0xFFB34C02(4289940482) |0x0921D500(153212160.1)
 |   20 ns   |    43       | 0x                     |
 +-----------+-------------+------------------------+-----------------------
*/
//20nsec case and 168MHz
#define ADJ_FREQ_BASE_ADDEND     0x4C19EF00//(CORRECT) 0x4C1CB1EA////was 0x58C8EC2B
#define ADJ_FREQ_BASE_INCREMENT   43

//#endif


#endif
