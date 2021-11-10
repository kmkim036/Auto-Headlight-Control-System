//int yEtherLoop(void)

/*
 * Time stamping unitÃƒÂ¬Ã¯Â¿Â½Ã‹Å“ ÃƒÂ­Ã¯Â¿Â½Ã‚Â´ÃƒÂ«Ã…Â¸Ã‚Â­ÃƒÂ¬Ã¯Â¿Â½Ã‹Å“ 2ÃƒÂªÃ‚Â°Ã¢â€šÂ¬ÃƒÂ¬Ã‚Â§Ã¢â€šÂ¬ ÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ ÃƒÂ«Ã‚Â°Ã‚Â©ÃƒÂ«Ã‚Â²Ã¢â‚¬Â¢
 *  - FineÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢
 *  - Coarse ÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢
 *  ÃƒÂ¬Ã…Â¡Ã‚Â°ÃƒÂ«Ã‚Â¦Ã‚Â¬ÃƒÂ«Ã…Â Ã¢â‚¬ï¿½ Fine ÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ ÃƒÂ«Ã‚Â°Ã‚Â©ÃƒÂ«Ã‚Â²Ã¢â‚¬Â¢ ÃƒÂ¬Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¬Ã…Â¡Ã‚Â©. Coarse ÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ ÃƒÂ«Ã‚Â°Ã‚Â©ÃƒÂ«Ã‚Â²Ã¢â‚¬Â¢ÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¤ ÃƒÂ¬Ã‚Â¢Ã¢â€šÂ¬ÃƒÂ«Ã¯Â¿Â½Ã¢â‚¬ï¿½ ÃƒÂ«Ã¢â‚¬Â Ã¢â‚¬â„¢ÃƒÂ¬Ã¯Â¿Â½Ã¢â€šÂ¬ ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ÃƒÂ«Ã‚Â°Ã¢â€šÂ¬ÃƒÂ«Ã¯Â¿Â½Ã¢â‚¬Å¾ ÃƒÂ¬Ã‚Â Ã…â€œÃƒÂªÃ‚Â³Ã‚Âµ.
 *  ÃƒÂ¬Ã¢â‚¬Å¾Ã‚Â¤ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ÃƒÂ¬Ã¯Â¿Â½Ã¢â€šÂ¬ ETH_PTPStart() ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â¨ÃƒÂ¬Ã‹â€ Ã‹Å“ÃƒÂ¬Ã¢â‚¬â€�Ã¯Â¿Â½ÃƒÂ¬Ã¢â‚¬Å¾Ã…â€œ  ÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ ÃƒÂ«Ã‚Â°Ã‚Â©ÃƒÂ«Ã‚Â²Ã¢â‚¬Â¢ ÃƒÂ¬Ã¢â‚¬Å¾Ã‚Â ÃƒÂ­Ã†â€™Ã¯Â¿Â½.
 *
 *  Fine ÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ ÃƒÂ«Ã‚Â°Ã‚Â©ÃƒÂ«Ã‚Â²Ã¢â‚¬Â¢ÃƒÂ¬Ã¯Â¿Â½Ã¢â€šÂ¬ ethernetif_PTPTime_SetTime()ÃƒÂªÃ‚Â³Ã‚Â¼ ETH_PTPTimeAdjFreq() ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â¨ÃƒÂ¬Ã‹â€ Ã‹Å“ÃƒÂ«Ã‚Â¥Ã‚Â¼ ÃƒÂ¬Ã¯Â¿Â½Ã‚Â´ÃƒÂ¬Ã…Â¡Ã‚Â©ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â´ ÃƒÂ­Ã¯Â¿Â½Ã‚Â´ÃƒÂ«Ã…Â¸Ã‚Â­ ÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â¨.

 * ethernetif_PTPTime_SetTime() ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â¨ÃƒÂ¬Ã‹â€ Ã‹Å“:  ÃƒÂªÃ‚Â¸Ã‚Â°ÃƒÂ¬Ã‚Â¤Ã¢â€šÂ¬ ÃƒÂ¬Ã¢â‚¬Â¹Ã…â€œÃƒÂªÃ‚Â°Ã¢â‚¬Å¾ÃƒÂ¬Ã¯Â¿Â½Ã¢â‚¬Å¾ ÃƒÂ¬Ã¢â‚¬Å¾Ã‚Â¤ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â¨(ÃƒÂ­Ã‹Å“Ã¢â‚¬Å¾ÃƒÂ¬Ã…Â¾Ã‚Â¬ ÃƒÂ¬Ã¢â‚¬Â¹Ã…â€œÃƒÂªÃ‚Â°Ã¢â‚¬Å¾ÃƒÂ¬Ã¯Â¿Â½Ã¢â‚¬Å¾ ÃƒÂ¬Ã¯Â¿Â½Ã‚Â¸ÃƒÂ¬Ã…Â¾Ã¯Â¿Â½ÃƒÂªÃ‚Â°Ã¢â‚¬â„¢ÃƒÂ¬Ã…â€œÃ‚Â¼ÃƒÂ«Ã‚Â¡Ã…â€œ ÃƒÂ«Ã¢â‚¬Å¾Ã‚Â£ÃƒÂ¬Ã¢â‚¬â€œÃ‚Â´ÃƒÂ¬Ã¢â‚¬Â¢Ã‚Â¼ ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â¨)
 * ETH_PTPTimeAdjFreq() ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â¨ÃƒÂ¬Ã‹â€ Ã‹Å“ : addend register ÃƒÂªÃ‚Â°Ã¢â‚¬â„¢ÃƒÂ¬Ã¯Â¿Â½Ã¢â‚¬Å¾ ÃƒÂ¬Ã‚Â¡Ã‚Â°ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ ÃƒÂ­Ã¢â‚¬Â¢Ã‹Å“ÃƒÂ«Ã‚Â©Ã‚Â°
 *    ppb (parts per 109 - billion)ÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¨ÃƒÂ¬Ã…â€œÃ¢â‚¬Å¾ÃƒÂ«Ã‚Â¡Ã…â€œ ÃƒÂªÃ‚Â¸Ã‚Â°ÃƒÂ«Ã‚Â³Ã‚Â¸ ÃƒÂ­Ã¯Â¿Â½Ã‚Â´ÃƒÂ«Ã…Â¸Ã‚Â­ ÃƒÂ¬Ã‚Â£Ã‚Â¼ÃƒÂ­Ã…â€™Ã…â€™ÃƒÂ¬Ã‹â€ Ã‹Å“ÃƒÂ¬Ã¯Â¿Â½Ã‹Å“ ÃƒÂ¬Ã†â€™Ã¯Â¿Â½ÃƒÂ«Ã…â€™Ã¢â€šÂ¬ÃƒÂ¬Ã‚Â Ã¯Â¿Â½ÃƒÂ¬Ã¯Â¿Â½Ã‚Â¸ ÃƒÂ«Ã‚Â³Ã¢â€šÂ¬ÃƒÂ­Ã¢â€žÂ¢Ã¢â‚¬ï¿½ ÃƒÂªÃ‚Â°Ã¢â‚¬â„¢ÃƒÂ¬Ã¯Â¿Â½Ã¢â‚¬Å¾ ÃƒÂ¬Ã¯Â¿Â½Ã‚Â¸ÃƒÂ¬Ã…Â¾Ã¯Â¿Â½ÃƒÂªÃ‚Â°Ã¢â‚¬â„¢ÃƒÂ¬Ã…â€œÃ‚Â¼ÃƒÂ«Ã‚Â¡Ã…â€œ ÃƒÂ«Ã¢â‚¬Å¾Ã‚Â£ÃƒÂ¬Ã¢â‚¬â€œÃ‚Â´ÃƒÂ¬Ã¢â‚¬Â¢Ã‚Â¼ ÃƒÂ­Ã¢â‚¬Â¢Ã…â€œÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¤. ÃƒÂ¬Ã‹Å“Ã‹â€ ÃƒÂ«Ã‚Â¥Ã‚Â¼ ÃƒÂ«Ã¢â‚¬Å“Ã‚Â¤ÃƒÂ¬Ã¢â‚¬â€œÃ‚Â´ ÃƒÂ¬Ã¢â‚¬â€œÃ‚Â´ÃƒÂ­Ã¢â‚¬ï¿½Ã…â€™ÃƒÂ«Ã‚Â¦Ã‚Â¬ÃƒÂ¬Ã‚Â¼Ã¢â€šÂ¬ÃƒÂ¬Ã¯Â¿Â½Ã‚Â´ÃƒÂ¬Ã¢â‚¬Â¦Ã‹Å“ÃƒÂ¬Ã¢â‚¬â€�Ã¯Â¿Â½ÃƒÂ¬Ã¢â‚¬Å¾Ã…â€œ 5ppmÃƒÂ¬Ã¯Â¿Â½Ã‹Å“ ÃƒÂ¬Ã¢â‚¬Â¹Ã…â€œÃƒÂªÃ‚Â°Ã¢â‚¬Å¾ÃƒÂ¬Ã‚Â°Ã‚Â¨ÃƒÂªÃ‚Â°Ã¢â€šÂ¬ ÃƒÂ«Ã‚Â°Ã…â€œÃƒÂ¬Ã†â€™Ã¯Â¿Â½ ÃƒÂ­Ã¢â‚¬Â¢Ã‹Å“ÃƒÂ¬Ã‹Å“Ã¢â€šÂ¬ÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¤ÃƒÂ«Ã‚Â©Ã‚Â´ 5000ÃƒÂ¬Ã¯Â¿Â½Ã‚Â´ÃƒÂ«Ã…Â¾Ã¢â€šÂ¬ ÃƒÂªÃ‚Â°Ã¢â‚¬â„¢ÃƒÂ¬Ã¯Â¿Â½Ã¢â‚¬Å¾ ÃƒÂ¬Ã¢â‚¬Å¾Ã‚Â¤ÃƒÂ¬Ã‚Â Ã¢â‚¬Â¢ÃƒÂ­Ã¢â‚¬Â¢Ã‹Å“ÃƒÂ«Ã‚Â©Ã‚Â´ ÃƒÂ¬Ã‹Å“Ã‚Â¤ÃƒÂ«Ã‚Â¥Ã‹Å“ÃƒÂ«Ã‚Â¥Ã‚Â¼ ÃƒÂ«Ã‚Â³Ã‚Â´ÃƒÂ¬Ã†â€™Ã¯Â¿Â½ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â´ÃƒÂ¬Ã‚Â¤Ã¢â€šÂ¬ÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¤.
 *
 *

 *ÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¤ÃƒÂ¬Ã¯Â¿Â½Ã…â€™ÃƒÂ¬Ã¯Â¿Â½Ã¢â€šÂ¬ ÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¨ÃƒÂªÃ‚Â³Ã¢â‚¬Å¾ÃƒÂ«Ã…Â Ã¢â‚¬ï¿½ addend registerÃƒÂ¬Ã¯Â¿Â½Ã‹Å“ ÃƒÂ¬Ã¢â‚¬â€�Ã¢â‚¬Â¦ÃƒÂ«Ã¯Â¿Â½Ã‚Â°ÃƒÂ¬Ã¯Â¿Â½Ã‚Â´ÃƒÂ­Ã…Â Ã‚Â¸ ÃƒÂ¬Ã‹â€ Ã‹Å“ÃƒÂ­Ã¢â‚¬â€œÃ¢â‚¬Â°ÃƒÂ­Ã¢â‚¬Â¢Ã‹Å“ÃƒÂ«Ã…Â Ã¢â‚¬ï¿½ÃƒÂ«Ã¯Â¿Â½Ã‚Â° ÃƒÂ¬Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¬Ã…Â¡Ã‚Â©ÃƒÂ«Ã¯Â¿Â½Ã…â€œÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¤.
    - Addend regisver ÃƒÂªÃ‚Â°Ã¢â‚¬â„¢ÃƒÂ¬Ã¯Â¿Â½Ã¢â‚¬Å¾ ÃƒÂªÃ‚Â³Ã¢â‚¬Å¾ÃƒÂ¬Ã¢â‚¬Å¡Ã‚Â°ÃƒÂ­Ã¢â‚¬Â¢Ã…â€œÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¤.
    - Time stamp addend registerÃƒÂ«Ã‚Â¥Ã‚Â¼ ÃƒÂªÃ‚Â³Ã¢â‚¬Å¾ÃƒÂ¬Ã¢â‚¬Å¡Ã‚Â°ÃƒÂ­Ã¢â‚¬Â¢Ã…â€œ ÃƒÂªÃ‚Â°Ã¢â‚¬â„¢ÃƒÂ¬Ã…â€œÃ‚Â¼ÃƒÂ«Ã‚Â¡Ã…â€œ ÃƒÂ¬Ã¢â‚¬â€�Ã¢â‚¬Â¦ÃƒÂ«Ã¯Â¿Â½Ã‚Â°ÃƒÂ¬Ã¯Â¿Â½Ã‚Â´ÃƒÂ­Ã…Â Ã‚Â¸ ÃƒÂ­Ã¢â‚¬Â¢Ã…â€œÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¤.
    - Time stamp addend registerÃƒÂ«Ã‚Â¥Ã‚Â¼ ETH_PTPTSCRÃƒÂ¬Ã¯Â¿Â½Ã‹Å“ TSAZRUÃƒÂ«Ã‚Â¥Ã‚Â¼ ÃƒÂ¬Ã¯Â¿Â½Ã‚Â´ÃƒÂ¬Ã…Â¡Ã‚Â©ÃƒÂ­Ã¢â‚¬Â¢Ã‚Â´ÃƒÂ¬Ã¢â‚¬Å¾Ã…â€œ Enable ÃƒÂ­Ã¢â‚¬Â¢Ã…â€œÃƒÂ«Ã¢â‚¬Â¹Ã‚Â¤.
*/
/* Examples of subsecond increment and addend values using SysClk = 168 MHz

 Addend * Increment = 2^63 / SysClk
 ptp_tick = Increment * 10^9 / 2^31 == 20nsecÃƒÂ¬Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¬Ã…Â¡Ã‚Â©:
 +-----------+-----------+------------+
 | ptp tick  | Increment | Addend     |
 +-----------+-----------+------------+
 |  119 ns   |   255     | 0x0CD53055 |
 |  100 ns   |   215     | 0x0F386300 |
 |   50 ns   |   107     | 0x1E953032 |
 |   20 ns   |    43     | 0x4C19EF00 |
 |   14 ns   |    30     | 0x6D141AD6 |
 +-----------+-----------+------------+

//20nsec case and 168MHz -- see ethernetif.h
#define ADJ_FREQ_BASE_ADDEND      0x4C19EF00 //was 0x58C8EC2B    (If the wrong value is used, you will encounter invalid PPS, etc.) -- YOON
 */
//We should modify stm32f4x7_eth.c for BroadR PHYs
/*
 * LwIP has 3 application programming interface (API) sets:
	ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢ Raw API is the native API of LwIP. It enables the development of applications using event callbacks. This API provides the best performance and code size, but adds some
	  complexity for application development. -- WE HAVE CHOSEN.
	ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢ Netconn API is a high-level sequential API that requires the services of a real-time operating system (RTOS). The Netconn API enables multi-threaded operations.
	ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢ BSD Socket API: Berkeley-like Socket API (developed on top of the Netconn API)
 *
 * The Raw API is based on the native API of LwIP. It is used to develop callback-based applications.
   When initializing the application, the user needs to register callback functions to different core events (such as TCP_Sent, TCP_error,...).
   The callback functions will be called from the LwIP core layer when the corresponding event occurs.

(2)PBUFs
   LwIP defines 3 types of pbufs, depending on the allocation type:
	ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢ PBUF_POOL: pbuf allocation is performed from a pool of statically pre-allocated pbufs that have a predefined size. Depending on the data size that needs to be allocated, one
      or multiple chained pbufs are allocated.
	ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢ PBUF_RAM: pbuf is dynamically allocated in memory (one contiguous chunk of memory for the full pbuf)
	ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢ PBUF_ROM: there is no allocation for memory space for user payload, the pbuf payload pointer points to data in the ROM memory (it can be used only for sending constant data).

For packet reception, the suitable pbuf type is PBUF_POOL; it allows to rapidly allocate memory for the received packet from the pool of pbufs. Depending on the size of the
received packet, one or multiple chained pbufs are allocated.
The PBUF_RAM is not suitable for packet reception because dynamic allocation takes some delay. It may also lead to memory fragmentation.
For packet transmission, depending on the data to be transmitted, the user can choose the most suitable pbuf type.

(3) Interfacing LwIP to STM32F4x7 Ethernet network interface
	The port of LwIP stack to STM32F4x7 is located in folder ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œ/port/STM32F4x7ÃƒÂ¢Ã¢â€šÂ¬Ã¯Â¿Â½.
	This demonstration package provides 2 implementations:
	ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢ Implementation without RTOS (standalone) -- WE HAVE CHOSEN
	ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¢ Implementation with an RTOS using FreeRTOS (http://www.freertos.org/)

	For both implementations, the "ethernet_if.c" file is used to link the LwIP stack to the STM32F4x7 Ethernet network interface.

(4) In standalone mode, the model of operation is based on continuous software polling to check if a packet is received.
  When a packet is received, it is first copied from the Ethernet driver buffers into the LwIP buffers.
  In order to copy the packet as fast as possible, the LwIP buffers (pbufs) should be allocated from the pool of buffers (PBUF_POOL).
  When a packet has been copied, it is handed to the LwIP stack for processing. Depending on the received packet, the stack may or may not notify the application layer.
  LwIP communicates with the application layer using event callback functions. These functions should be assigned before starting the communication process.

 (5) Example

 int main(void){
...
 // configure Ethernet (GPIOs, clocks, MAC, DMA)
 //ETH_BSP_Config();//initialize the Ethernet peripheral (GPIOs, Clocks, MAC and DMA options).

 // Initilaize the LwIP stack
 //LwIP_Init();// initialize the LwIP stack internal structures and for starting stack operations.

 //  DO SOMETHING (ex) tcp echo server Init
 //tcp_echoserver_init();

 // Infinite loop
 while (1) {
 // check if any packet received
 if (ETH_CheckFrameReceived()) {
 //software polls for packet reception using Ethernet driver ETH_CheckFrameReceived function.
 // When a packet is received, it should be handled by the LwIP stack using function LwIP_Pkt_Handle.

 // process received Ethernet packet
 //LwIP_Pkt_Handle();
 //}
  // handle periodic timers for LwIP
  //handle certain LwIP internal periodic tasks (protocol timers, retransmission of TCP packets,...).
 //LwIP_Periodic_Handle(LocalTime);
 }
}

*** BUG ??
Why we call twice stm32_low_level_init(netif) in ethernetif_init(struct netif *netif)
 */
/* Includes ------------------------------------------------------------------*/

#include "yLib/eth/include/ethopts.h"
#include "yInc.h"
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"//Reset and Clock Control
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#else
...
#endif
#if (PROCESSOR == PROCESSOR_STM32F107VCT)
#include "stm32f107_eth.h"
#else
//	printf("Use SPI for Ethernet\r\n");
#endif
#include "misc.h"
#include "ySpiDrv.h"
#include "err.h"

#if (PROJ_FOR == PROJ_FOR_PTP)
#include "lwip/include/etharp.h"
#include "ethernetif.h"

#include "ptpstm/include/datatypes.h"
#include "lwipopts.h"
#include "lwip/include/netconf.h"
#include "lwip/include/mem.h"
//#include "lwip/include/httpd.h"
//#include "tapdev.h"
#elif (PROJ_FOR == PROJ_FOR_CONTIKI)
#include "contiki.h"
#include "contiki/os/net/ipv6/uipopt.h"
#include "dev/con-ethernet.h"
//#include "yEnc28j60driver.h" //#include "net/ethernet.h"
#include "contiki/os/net/packetbuf.h"
#include "contiki/os/net/netstack.h"
#include "contiki/os/net/ipv6/uip.h"
#include "contiki/os/sys/timer.h"
#include "contiki/os/sys/clock.h"
#define DEBUG DEBUG_PRINT //DEBUG_NONE
#include "contiki/os/net/ipv6/uip-debug.h"
#endif

extern unsigned char g_macaddrLsb;

// ptpd
//#include "ptpstm/ptpd.h"
/* On STM407 PPS output pulse
 * It is used to check sync between local slave clock and master clock.
 * Default Period is 1sec with 125msec pulse width.
 * When set to 1Hz, PPS pulse width is
 *   125ms with binary rollover (TSSSR=0 bit 9 in ETH_PTPTSCR) -- timestamp low register rolls over when 999 999 999 ns. or
 *   100ms with digital rollover (TSSSR=1 bit 9 in ETH_PTPTSCR) -- timestamp low register rolls over when 0x7fff ffff.
 * When set to 2Hz and higher,
 *   - Duty is 50% in binary rollover.
 *   - in Digital rollover, set Only PPS of 1Hz.
 * [THUS]
 * When set to 2Hz and higher,
 *  - We should set binary rollover(TSSSR=0 bit 9 in ETH_PTPTSCR).
 *  - PPSFREQ[3:0] in ETH_PTPPPSCR register can be used to set the frequency of PPS to 2**PPSFREQ Hz. (max freq is 2**15 = 32.768KHz)
 *  - In this case, duty is 50%.
 *  - When we use cs2300 PLL, we set PPSFREQ[3:0] = 0xA (1024Hz) or PPSFREQ[3:0] = 0xF (32.768KHz)
 */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* BRMC
 * This is the setup/management program for automotive Ethernet media converter module.
 *
 * CHONGHO YOON
 * 2015.07.30
 *
 * HW :
 *   IP101A RXD -->-+
 *   (0x01) TXD<-+  |
 *   (NORMAL)    |  |
 *               |  |
 *      BR1 RXD--+ -|
 *   (0x19) TXD<----+
 *  (REVERSE)
 *                 |   MDIO   +------------+
 *                 +--------- |LM3S811 MCU |
 *                            |            |
 *                            +------------+
 *  [CONTROL INTERFACE]
 *  MDIO for BR0 = 0x19; IP101A=0x01
 *  MDIO/MDC Controlled by LM3S811 MCU.
 *
 *
 *  - This can support
 *      * AutoNegotiated Mode
 *      * Forced Master/Slave Selection
 *      * Report Current Operation Status on OLED
 *
 */
/*DMA controls all transactions for tx path.
 * Eth frames read from the system memory into the FIFO(Depth=2KB) by DMA via AHB bus. Then the frame is popped out to MAC core.
 * 2 modes for popping to MAC core.
 *   a) Threshold mode
 *   b) Store and Forward mode.
 * After tx by MAC core, transmit status is backed to App. (1588 enabled, 64 bit time stamp is also returned.)
 *
 * [DMA]
 *  2 descript methods : Ring and Chained.
 *
 *
 *
 * [Interrupt]
 * Ethernet controller has 2 interrupt vectors:
 *      a) for normal Ethernet operation : generated by MAC and DMA.
 *      b) for wake up  event (for MAGIC, or wake up frame) --mapped on EXTI line19. (PMT)
 * [Loopback] Supported
 *
 * [MAC management counters]
 *
 * [Power Management]
 *
 * [PTP]
 * Reference Time in 32+32bit channels. : System Time.
 * See System Time Correction Method (2 correction method = coarse and fine.
 * Tx TS => TDES2(LSB) and TDES3(MSB)
 * Rx TS => RDES2 and RDES3
 */
//#define yPHYADDR 0x19 //0x18 is not working...
#if (PROJ_FOR == PROJ_FOR_PTP)
extern volatile uint32_t g_ptpLocalTime_in_msec;
extern void ieee8021as_thread();
uint32_t timingdelay;
#endif

#if (PROCESSOR == PROCESSOR_STM32F107VCT)
void ETH_IP101A_BSP_Config(void);

//DMA===
extern ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB];
extern ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB];
extern  uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE];
extern  uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE];

// Global pointers on Tx and Rx descriptor used to track transmit and receive descriptors
extern volatile ETH_DMADESCTypeDef  *gp_DMATxDescToSet;
extern volatile ETH_DMADESCTypeDef  *gp_DMARxDescToGet;

//struct _mcConfig rdConfig;
//extern struct netif g_netif;

extern void ETH_GPIO_Config(void);
extern void ETH_NVIC_Config(void);
extern void ETH_MACDMA_Config(void);
extern short USART3_GetChar(void);

void yEther_SimpleSend(int i){
	volatile u8 simppkt[1518];
	simppkt[0]=0xff;simppkt[1]=0xff;simppkt[2]=0xff;simppkt[3]=0xff;simppkt[4]=0xff;simppkt[5]=0xff;
	simppkt[6]='Y';simppkt[7]='O';simppkt[8]='O';simppkt[9]='N';simppkt[10]='C';simppkt[11]='H';
	simppkt[12]=0x08;simppkt[13]=0x06; //ARP 0x0806
	simppkt[14]=0xff;

	if(ethernetif_low_level_output(&simppkt[0],48)==ERR_OK){
		printf("T>%d\r\n",i);
	}else {
		printf("Send(%d:Fail) ",i);
	}
}

void yEther_SimpleShortSend(int i){
	volatile u8 simppkt[64];
	simppkt[0]=0xff;simppkt[1]=0xff;simppkt[2]=0xff;simppkt[3]=0xff;simppkt[4]=0xff;simppkt[5]=0xff;
	simppkt[6]='Y';simppkt[7]='O';simppkt[8]='O';simppkt[9]='N';simppkt[10]='C';simppkt[11]='H';
	simppkt[12]=0x08;simppkt[13]=0x06; //ARP 0x0806
	simppkt[14]=0xff;
	simppkt[63] = 0x01;

	if(ethernetif_low_level_output(&simppkt[0],64)==ERR_OK){
		printf("T>%d\r\n",i);
	}else {
		printf("Send(%d:Fail) ",i);
	}
}
#endif
#if (PROJ_FOR == PROJ_FOR_PTP) || (PROJ_FOR == PROJ_FOR_RSTP_PTP)
extern short stmGpsGetTimeOfDay(unsigned char uartId, struct minmea_sentence_rmc *prmc);
TimeInternal g_currentTod = {0,0};
TimeInternal g_currentTodFromRMCmsg = {0,0};
unsigned char g_getFirstGPStod = 0;

//==================================================================
/*
void Delay(uint32_t nCount) //nCount: number of 10ms periods to wait for.
{
  // Capture the current local time
  timingdelay = g_ptpLocalTime_in_msec + nCount;
  // wait until the desired delay finish
  while(timingdelay > g_ptpLocalTime_in_msec)
  {
  }
}
*/

extern void max7219_16digit_Config();
extern void PTP_max7219_write16digits(
		unsigned char hh,
		unsigned char mm,
		unsigned char ss,
		unsigned long  ns
		);

/*extern void OzOLED_init();
extern void OzOLED_SendCmd(unsigned char command);
extern void OzOLED_sendData(unsigned char Data);
extern void stmOzOLED_printString(const char *String, u8 X, u8 Y, u8 numChar);

//PD2(U1RX) > Forced(ON) | AN(OFF) -- Input
//PD3(U1TX) > Master(ON) | Slave(OFF) -- Input

 */

// For handling PPSInput(PB15, Pin2) : The PB15 has dual functions of GP0 and ULED0. Thus we select GP0 with S4(UP).
// The PB15(Pin2, GP0) should be connected with GPS's PPS signal or MAC's PPS on PB5(Pin3).
//Interrupt enabled GPInput.(with EXTI)
void ETH_PB15_PPSfromClockSourceInConf(void){
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  /* Enable GPIOB clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	  /* Configure PB15 pin as input floating */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

/*407... TBR
	  // Enable SYSCFG clock
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	  // Connect EXTI Line15 to PB15 pin
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource15);
	  // Configure EXTI Line15
	  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
*/
	  // Enable and set EXTI Line15 Interrupt to the lowest priority
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
extern void updateClockForGMC(PtpClock *ptpClock,TimeInternal *gpsTime);
extern PtpClock ptpClock;
extern void displayState(const PtpClock *ptpClock);
#endif

#if (CLOCKSOURCE == CLOCKSOURCE_GPS)
//============================================== PPSfromGPS ======================================================
void EXTI15_10_IRQHandler(){
	struct minmea_sentence_rmc rmc;
	TimeInternal currentTod;
	tTod tod;

	 if(EXTI_GetITStatus(EXTI_Line15) != RESET){ //PB15 from GPS's PPS
		 EXTI_ClearITPendingBit(EXTI_Line15); //Clear this IRQ.

		 if(g_getFirstGPStod == 1){ //if we got the ToD at the main's Init. Forced Set

			 //Update MAC's PTP Registers if needed.
			 //Set GPS TIME to STM32F407 MAC's PTP Registers.
			 g_currentTod.seconds = g_currentTodFromRMCmsg.seconds + 1;
			 g_currentTod.nanoseconds = 0; //+ need internal_latency?

			 setTimeToReg(&g_currentTod);   //Set the updated TOD.
			 //initClock(&ptpClock);      //Reset PTPd.
			 g_getFirstGPStod = 2;

			 //Show current TOD on 7 segments.
			 //ulocaltime(g_currentTod.seconds, &tod);

			 displayCurrent();//
			 //displayStats(&ptpClock);
		 }else if(g_getFirstGPStod == 2){
			 g_currentTod.seconds = g_currentTod.seconds + 1;//g_currentTodFromRMCmsg.seconds + 1;
			 g_currentTod.nanoseconds = 2000; //+ need internal_latency?
			 updateClockForGMC(&ptpClock, &g_currentTod);
			 //Show current TOD on 7 segments.
			 displayCurrent();
		 }else{
			 //We are not in sync state with GPS.
			 stmOzOLED_printString("NoGPS Signal..",0,3,16);
		 }
		 //======================= common ===============
#ifdef USE_SYNCLED
		 PTP_SyncLed(g_locked);
#endif

/*		 if(g_getFirstGPStod != 0){
			 //Show current TOD on 7 segments.
			 ulocaltime(g_currentTod.seconds, &tod);
#ifdef USE_MAX7219
			 PTP_max7219_write16digits(
				tod.ucHour, tod.ucMin, tod.ucSec,
				g_currentTod.nanoseconds
			 );
#endif
		 }
*/
		 displayState(&ptpClock);
		 //get from the UART buffer from GPS for next PPS. It encounters some delay.
		 if(stmGpsGetTimeOfDay(uartId, &rmc)){
			 //g_currentTodFromRMCmsg.seconds = tod2utc(rmc.date.year+2000, rmc.date.month, rmc.date.day, rmc.time.hours, rmc.time.minutes, rmc.time.seconds);
			 //printf("\r\n$GPRMC:ToD=20%d.%d.%d %d:%d:%d \r\n",rmc.date.year, rmc.date.month, rmc.date.day, rmc.time.hours, rmc.time.minutes, rmc.time.seconds);
			 //printf("\r\n");
		 }

	  }
}


void PTP_getFirstGPStod(TimeInternal *pcurrentTod){
	struct minmea_sentence_rmc rmc;

	while(1){ //Wait for GPS Synchronized.
		if(stmGpsGetTimeOfDay(uartId, &rmc)){
			printf("\r\n$GPRMC:ToD=20%d.%d.%d %d:%d:%d \r\n",rmc.date.year, rmc.date.month, rmc.date.day, rmc.time.hours, rmc.time.minutes, rmc.time.seconds);
			printf("\r\n");
			stmOzOLED_printString("Got NMEA..",0,3,16);
			break;
		}else{
			//stmOzOLED_printString("WaitForNMEA..",0,3,16);
			//delayms(100);
		}
	}
	//For year, we should add 2000
	pcurrentTod->seconds = tod2utc(rmc.date.year+2000, rmc.date.month, rmc.date.day, rmc.time.hours, rmc.time.minutes, rmc.time.seconds);
	pcurrentTod->nanoseconds = 0;
}
#else //NO GPS -- == PPSfromMAC ====
extern uint32_t g_tickEvent;
/* SHOULD BE RESTORED
void EXTI15_10_IRQHandler(){
	 if(EXTI_GetITStatus(EXTI_Line15) != RESET){ //PA15 from LAN9355 on 1588 Clock Event.
		 EXTI_ClearITPendingBit(EXTI_Line15); //Clear this IRQ.
			 g_tickEvent = 1;
			 //displayCurrent();
			 //displayState(&ptpClock);
	  }
}
*/
#endif //#if (CLOCKSOURCE == CLOCKSOURCE_GPS)
/*
//============================================== PPSfromMAC ======================================================
// For handling PPSfromGPS(PB15) : The PB15 has dual functions of GP0 and ULED0. Thus we select GP0 with S4(UP).
// The Pin2(GP0) should be connected with GPS's PPS signal.
//Display Current Status on each PPS Input Pulse from MCU's MAC.
//To do this, you should ties PB5(PPS)output(Pin3) with PE0(CAM_D2 pin)
//Interrupt enabled GPInput.(with EXTI)]

// FromPPS_MAC(PB5, Pin3) to CANRX/PA11(Pin9) (Ties Pin3 and Pin9). Use S2 to select CAN.
void ETH_PA11_PPSfromMACinConf(void)
{
	  GPIO_InitTypeDef   GPIO_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//as input floating //GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);


	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource11);
	  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);


	  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
*/




//============================================PHY SPECIFIC ================================
#if(PHYCHIP == BCM89810)
//============================== BROADR-REACH SPECIFIC =================================
//Forced1_AN0 = PC13
//Master1_Slave0=PC0
void BroadR_ModeSelectors_GPIO_setup(void){

	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_0;//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_0); //??? We need it?

	memset(&mcConfig, 0x00, sizeof(struct _mcConfig));
}

void BroadR_ModeSelectorHandler(u8 first){
	unsigned char val1,val2;

	val1 = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13); //Read Forced1_AN0

	if(first || (mcConfig.fForced_or_AN !=  val1)){
		mcConfig.fForced_or_AN = val1; //Update
		if(mcConfig.fForced_or_AN){
			val2 = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0); //Get Master/Slave
			mcConfig.fMaster_or_Slave = val2;
			if(val2){
				//printf("Writing on Phy(0x19) Reg 0x%02x <-- 0x%04x\r\n",  phyaddr, reg, val);
				ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0x00, 0x0208);//Reg0 set as Master
				//printf(">Forced Master\r\n");
				stmOzOLED_printString("*Forced MASTER ",0,1,15);
			}else{
				ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0x00, 0x0200);//mdio_write(0x19, 0, 0x0200);
				//printf(">Forced:Slave\r\n");
				stmOzOLED_printString("*Forced SLAVE  ",0,1,15);
			}

		}else{ //AN
			ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0x00, 0x1000);//mdio_write(0x19, 0, 0x1000);//0x1200
			//printf(">AutoNeg.\r\n");
			stmOzOLED_printString("*AutoNego Mode. ",0,1,15);
		}

	}else if(mcConfig.fForced_or_AN > 0){ //NO Changed. But if it is still forced.
			val2 = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0); //Get Master/Slave
			if(mcConfig.fMaster_or_Slave !=  val2){ //Changed?
				mcConfig.fMaster_or_Slave = val2;
				if(val2){
					ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0x00, 0x0208);//Reg0 set as Master//mdio_write(0x19, 0, 0x0208);
					//printf(">Forced Master\r\n");
					stmOzOLED_printString("*Forced MASTER ",0,1,15);
				}else{ //Slave
					ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0x00, 0x0200);//mdio_write(0x19, 0, 0x0200);
					//printf(">Forced Slave\r\n");
					stmOzOLED_printString("*Forced SLAVE  ",0,1,15);
				}
			}
	}
	delayms(10);
}

void BroadR_StatusHandler(void){

	u16 linkStatus = 0;
	u32 result;

	//stmOzOLED_printString("           ",0,2,15);
	//Read LRE Status Register
	linkStatus = (u16)ETH_ReadPHYRegister(BROADR_PHY_ADDRESS, 1); //LRE Status
	if(mcConfig.reg1_status != linkStatus){
		mcConfig.reg1_status = linkStatus;//Update it.
		if(mcConfig.reg1_status & 0x0004){
			stmOzOLED_printString("+Link Up   ",0,3,12);
			//printf("+Link Up    \r\n");
			mcConfig.LinkUp = 1;
		}else{
			stmOzOLED_printString("-Link Down  ",0,3,12);
			//printf("-Link Down  \r\n");
			mcConfig.LinkUp = 0;
		}
	}

	//Read LED Status Register
	if(ETH_ERROR == (u16)ETH_WritePHYRegister(BROADR_PHY_ADDRESS, 0x1c, 0x2000))//1c_shadow_01000 select
		return;
	delayms(10);
	linkStatus = (u16)ETH_ReadPHYRegister(BROADR_PHY_ADDRESS, 0x1c); //LRE StatusmcConfig.reg1c08_ledstatus = (u16)mdio_read(0x19, 0x1c);
	if(linkStatus == ETH_ERROR)
		return;

//	if(mcConfig.reg1c08_ledstatus != linkStatus){

		mcConfig.reg1c08_ledstatus = linkStatus;//Update it.

		//printf("LED status reg=0x%04x\n", linkStatus);

		if((mcConfig.reg1c08_ledstatus & 0x0080)==0x0080) stmOzOLED_printString("-HDX           ",0,5,15);
		else  											  stmOzOLED_printString("+FDX           ",0,5,15);

		if((mcConfig.reg1c08_ledstatus & 0x0018)==0x0018)	stmOzOLED_printString("-NoLink        ",0,6,15);
		else if((mcConfig.reg1c08_ledstatus & 0x0018)==0x0010) stmOzOLED_printString("+10Mbps      ",0,6,15);
		else if((mcConfig.reg1c08_ledstatus & 0x0018)==0x0008) stmOzOLED_printString("+100Mbps     ",0,6,15);
		else  											  stmOzOLED_printString("-NA            ",0,6,15);


		if((mcConfig.reg1c08_ledstatus & 0x0001)==0x0001) stmOzOLED_printString("-PoorQuality   ",0,7,15);
		else 											  stmOzOLED_printString("+GoodQuality   ",0,7,15);

		if(mcConfig.LinkUp){
			if(mcConfig.fForced_or_AN == 1){ //ForcedMode
				if(mcConfig.fMaster_or_Slave == 1)               stmOzOLED_printString("+M/S:Master        ",0,4,15);
				else  											  stmOzOLED_printString("-M/S:Slave         ",0,4,15);
			}else{ //AN Mode
				if((mcConfig.reg1c08_ledstatus & 0x0100)==0x0100) stmOzOLED_printString("+M/S:Master        ",0,4,15);
				else  											  stmOzOLED_printString("-M/S:Slave         ",0,4,15);
			}
		}else{ //Link Down
	              stmOzOLED_printString("-M/S: NA    ",0,4,15); //No Master/Slave Status
		}
//	}
}

BroadR_Periodic_Handle(volatile int localTime){
	BroadR_ModeSelectorHandler(0);
	BroadR_StatusHandler();
}

#endif
//============================================PHY SPECIFIC PTP =============================================================
#if (PROJ_FOR == PROJ_FOR_PTP) || (PROJ_FOR == PROJ_FOR_RSTP_PTP)

#if(PHYCHIP == BCM89810)
//============================== BROADR-REACH SPECIFIC =================================

int yEther_PTP_1588_8021AS_Loop(char *str)
{
	int seq;
	u32 ret,x;
	TimeInternal currentTod;
	/*!< At this stage the microcontroller clock setting is already configured to
       168 MHz, this is done through SystemInit() function which is called from
       startup file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
    */

	//LED_Init();
	printf("yEtherLoop with BroadR-Reach PHY\r\n");
	//Display_String(240-238, 320-220, "EtherCrafts", 0x07E0); //GREEN
	//Display_String(240-220, 320-10, "Init BroadR-Reach PHY...", 0xFFFF);
	//Init_SysTick(1000);//1msec
    /***************************************************************************
    NOTE: When using Systick to manage the delay in Ethernet driver, the Systick
         must be configured before Ethernet initialization and, the interrupt
         priority should be the highest one.
  *****************************************************************************/
	//SysTick_ITConfig(ENABLE); //SysTick Interrupt Enable
	//==== configure ethernet (GPIOs, clocks, MAC, DMA)
	ETH_BROADR_BSP_Config();
	//Display_String(240-210, 320-10, "Config BroadR-Reach PHY...", 0xFFFF); //WHITE
	//BroadR_ModeSelectorHandler(1);

	// Initilaize the LwIP stack
/*
#if (PTP_PROTOCOL == IEEE8021AS)
	lwIP_Init_8021as(1);//macAddr);
#else
	LwIP_Init(); // we call stm32_low_level_init() twice. Why?
	//Display_String(240-200, 320-10, "LwIP Init...", 0xFFFF);
#endif

	//Http webserver Init
	//lwip_httpd_init();
#if (PTP_PROTOCOL ==IEEE8021AS)
	ieee8021as_thread();
#elif (PTP_PROTOCOL == IEEE1588V2)
	//Display_String(240-190, 320-10, "Start PTPd...", 0xFFFF);
	currentTod.seconds = 1450491635;
	currentTod.nanoseconds = 681000000;
	ieee1588ptpd_thread(&currentTod);//ptpd_init_and_loop(&currentTod);// Initialize the PTP daemon.
#endif
*/
	seq=0;
	while (1)  {
		//__WFI(); //Wait for Interrupts including SysTick IRQ and PHY Link Status Changes. (including DMA? -- may be YES.)

		// Polling Mode: check if any packet received -- moved to ETH_IRQHandler in stm32f4x7_eth_bsp.c
/*		while (ETH_CheckFrameReceived())   {// process received ethernet packet
			printf("=====main: ETH_CheckFrameReceived(%d)\r\n",seq);
			seq++;
			LwIP_Pkt_Handle(); //move FIFO into pbuf, and handles it.
			//ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
			//ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
		}
*/
		// handle periodic timers for LwIP
		//LwIP_Periodic_Handle(g_ptpLocalTime_in_msec);
		//BroadR_Periodic_Handle(g_ptpLocalTime_in_msec); //check link status

		//for send test only
		ySimpleSend(seq);
		seq++;
	}

}
//==========================(2) DP83848 =========================
#elif (PHYCHIP == DP83848)
int yEther_PTP_1588_8021AS_Loop(char *str)
{
	int seq;
	u32 ret,x;
	TimeInternal currentTod;
	/*!< At this stage the microcontroller clock setting is already configured to
       168 MHz, this is done through SystemInit() function which is called from
       startup file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
    */

	//LED_Init();
	printf("ETH_PTP8021as on NS83848 PHY.\r\n");

	//Init_SysTick(1000);//1msec
    /***************************************************************************
    NOTE: When using Systick to manage the delay in Ethernet driver, the Systick
         must be configured before Ethernet initialization and, the interrupt
         priority should be the highest one.
  *****************************************************************************/
	//==== configure ethernet (GPIOs, clocks, MAC, DMA)
	ETH_BSP_Config();

#if (PTP_PROTOCOL == IEEE8021AS)
	lwIP_Init_8021as(1);//macAddr);
#else
	LwIP_Init(); // we call stm32_low_level_init() twice. Why?
#endif

	//Http webserver Init
	//lwip_httpd_init();
#if (PTP_PROTOCOL == IEEE8021AS)
	ieee8021as_thread();
#elif (PTP_PROTOCOL ==IEEE1588V2)
	currentTod.seconds = 1450491635;
	currentTod.nanoseconds = 681000000;
	ieee1588ptpd_thread(&currentTod);//ptpd_init_and_loop(&currentTod);// Initialize the PTP daemon.
#endif

	seq=0;
	while (1)  {
		//__WFI(); //Wait for Interrupts including SysTick IRQ and PHY Link Status Changes. (including DMA? -- may be YES.)

		// Polling Mode: check if any packet received -- moved to ETH_IRQHandler in stm32f4x7_eth_bsp.c
/*		while (ETH_CheckFrameReceived())   {// process received ethernet packet
			printf("=====main: ETH_CheckFrameReceived(%d)\r\n",seq);
			seq++;
			LwIP_Pkt_Handle(); //move FIFO into pbuf, and handles it.
			//ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
			//ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
		}
*/
		// handle periodic timers for LwIP
		LwIP_Periodic_Handle(g_ptpLocalTime_in_msec);
	}

}

#elif (PHYCHIP == IP101) //===(3) IP101A/G==============================================================================
extern void stmUSART3ShimConf(uint32_t USART_BaudRate);
extern void ETH_X_BSP_Config(unsigned char PhyAddr, char *str);//extern void ETH_IP101A_BSP_Config(void);

int yEther_PTP_Config(char *str)
{
	int seq;
	u32 ret,x;
	short rxchar;
	unsigned char len;

	printf("%s\r\n",str);

	//yTIM3_Config(); //each 100msec period. Call TIM3_IRQ --> Show Current TOD -- //Not Working? - For PTP, TIM2 may be used for PTP's Target Timer.
#if	(ETHIF_ROLE == PTP_GRANDMASTERCLOCK)
	stmUSART3ShimConf(9600); //for GPS
#endif
	//PB5 -- OUTPUT -- PPS from MCU -- H2 ON --> to PA11
	//PB15-- INPUT(pin2)  -- get PPS signal
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
	//ETH_PB15_PPSfromClockSourceInConf();//ETH_PG8_PPSfromGPSinConf();
#elif (PROCESSOR == PROCESSOR_STM32F107VCT)
	//ETH_PB15_PPSfromClockSourceInConf();//ETH_PB15_PPSfromGPSinConf();
	//GPIO_PinRemapConfig(GPIO_Remap_PTP_PPS, ENABLE);//YOON for  PB5 PPS
#endif
	//ETH_PA11_PPSfromMACinConf(); //CANRXLED (needs to tie PIN3 and PIN9)
	//==== configure ethernet (GPIOs, clocks, MAC, DMA)
	ETH_X_BSP_Config(2, "IP101A");
	//ETH_X_BSP_Config(9, "IP101G");//ETH_IP101A_BSP_Config();
	//Display_String(240-210, 320-10, "Config IP101G PHY...", 0xFFFF); //WHITE
	printf("Config IP101A/G PHY...\r\n");

	return 1;
}

int yEther_PTP_1588_8021AS_Loop(char *str)
{
	int seq;
	u32 ret,x;
	short rxchar;
	unsigned char len;

	//(1) Config Eth to support HW timestamping.
	yEther_PTP_Config(str);

	//(2) Config lwip stack with mem and memp init.
#if (PTP_PROTOCOL ==IEEE8021AS)
	lwIP_Init_8021as(g_macaddrLsb);
#else
	LwIP_Init(); // we call stm32_low_level_init() twice. Why?
	//Display_String(240-200, 320-10, "LwIP Init...", 0xFFFF);
#endif
	stmOzOLED_printString("-----------",0,4,16);

	//(3) Do loop
#if (PTP_PROTOCOL ==IEEE8021AS)
	ieee8021as_thread();
#else (PTP_PROTOCOL ==IEEE1588V2)
	#if	(ETHIF_ROLE == PTP_GRANDMASTERCLOCK)
		//Display_String(240-190, 320-10, "Start PTPd...", 0xFFFF);
		//Get the First Current ToD from GPS.
		g_getFirstGPStod = 0;
		PTP_getFirstGPStod(&g_currentTodFromRMCmsg);//g_currentTod);
		g_getFirstGPStod = 1;

		while(g_getFirstGPStod != 2){} //Wait for resetting this flag by IRQ.
		//set currentToD to STM32F407 PTP Registers.
	#else
		g_currentTod.seconds = 1450491635;
		g_currentTod.nanoseconds = 681000000;
    #endif
	ieee1588ptpd_thread(&g_currentTod);//ptpd_init_and_loop(&g_currentTod);// Initialize the PTP daemon.
#endif
}
#elif (PHYCHIP == VPHY_SWITCH2MCU)
extern void Ksz8794_init();
int ETH_PTP_ForSwitchBoard_Config(char *str)
{
	unsigned char PhyAddr = 0;

	printf("%s\r\n",str);

	//==== configure ethernet (GPIOs, clocks, MAC, DMA)
	ETH_SimpleOrSwitchPHY_BSP_Config(PhyAddr);//ETH_BROADR_BSP_Config();
	delayms(100);

	//Ties PB5(OUTPUT PPS from MCU) to PE0(INPUT)
	//GP3_PE0_setup(); //PE0 <-- MacPPS (PB5). It is different from Baseboard(PB15, pin2)
	//ETH_PA11_PPSfromMACinConf(); //CANRXLED (needs to tie PIN3 and PIN9)

}
//extern void GP3_PE0_setup(void); //see blinky.c
void EXTI0_IRQHandler(){
	 if(EXTI_GetITStatus(EXTI_Line0) != RESET){
		 EXTI_ClearITPendingBit(EXTI_Line0);
		 //displayStats(&ptpClock);		 //displayCurrent();
		 //displayCurrent();
		 //displayState(&ptpClock);

	  }
}
extern void ieee1588ptpd_thread(TimeInternal *pcurrentTod);

int yEther_PTP_1588_8021AS_Loop(char *str)
{
	int seq;
	u32 ret,x;
	TimeInternal currentTod;

	ETH_PTP_ForSwitchBoard_Config(str);

#if (PTP_PROTOCOL ==IEEE8021AS)
	lwIP_Init_8021as(g_macaddrLsb);//macAddr);
#else
	LwIP_Init(); // we call stm32_low_level_init() twice. Why?
	//Display_String(240-200, 320-10, "LwIP Init...", 0xFFFF);
#endif
	stmOzOLED_printString("----------",0,4,16);

#if (PTP_PROTOCOL ==IEEE8021AS)
	ieee8021as_thread();
#elif (PTP_PROTOCOL ==IEEE1588V2)
	//Display_String(240-190, 320-10, "Start PTPd...", 0xFFFF);
	ieee1588ptpd_thread(&currentTod);//ptpd_init_and_loop(&currentTod);// Initialize the PTP daemon.
#endif
}
#endif
#endif

/*
uint32_t Eth_Link_PHYITConfig(uint16_t PHYAddress)
{
  uint32_t tmpreg = 0;
  	u32 result;

  // Read PHY_IRQ_STAT_CTRL register
  tmpreg = ETH_ReadPHYRegister(PHYAddress, PHY_IRQ_STAT_CTRL);
  if (tmpreg & PHY_LINK_UP)
  {
    EthLinkStatus = 1;
    GPIOD->ODR |= GPIO_Pin_12;
  }

  // Enable output interrupt events to signal via the INT pin
  tmpreg |= (uint32_t) PHY_IRQ_LINK_DOWN | PHY_IRQ_LINK_UP;
  if(!(ETH_WritePHYRegister(PHYAddress, PHY_IRQ_STAT_CTRL, tmpreg)))
  {
    // Return ERROR in case of write timeout
    return ETH_ERROR;
  }

  //debug
  //tmpreg = 0;
  //tmpreg = ETH_ReadPHYRegister(PHYAddress, PHY_IRQ_STAT_CTRL);
  //printf("PHY_IRQ_STAT_CTRL 0x%04x\r\n",tmpreg);
  //printf("ret good\r\n");

  return ETH_SUCCESS;
}
*/
/**
  * @brief  Updates the system local time
  * @param  None
  * @retval None
  */
//void lwip_Time_Update(void){
//	g_ptpLocalTime_in_msec += SYSTEMTICK_PERIOD_MS;
//}

/* ================ REFERENCE ONLY ===================================
//Display Current Status per...
//See void yTIM3_Config(void) in yTIM.c
//We do not use it
void TIM3_IRQHandler(){
	tTod tod;
	TimeInternal timeMac;
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  		if(g_getFirstGPStod == 2){
    			 //Show current TOD on 7 segments.
  				getTimeFromReg(&timeMac);
    			ulocaltime(timeMac.seconds, &tod);
#ifdef USE_MAX7219
    			PTP_max7219_write16digits(
    					tod.ucHour, tod.ucMin, tod.ucSec,
    					timeMac.nanoseconds
    			);
#endif
  		}
    }
}

*/

/*
//=========== CONTIKI =========================================================================
//---------------------------------------------------------------------------
PROCESS(yEth_driver_process, "IP101 Ethernet driver");
//---------------------------------------------------------------------------

unsigned short yEth_low_level_input_with_ptr(u8 *prxbuf)
{
  u16_t len;
  int l =0;
  FrameTypeDef frame;
  u8 *pbuf;
  uint32_t i=0;
  __IO ETH_DMADESCTypeDef *DMARxNextDesc;
  char str[10];

  // get received frame
  frame = ETH_Get_Received_Frame_interrupt();
  //frame = ETH_Get_Received_Frame_byPollingMethod();

  len = frame.length;
  if(len == 0)
	  return 0;
  pbuf = (u8 *)frame.buffer;
  memcpy(prxbuf, pbuf, len);

  sprintf(str,"R>Type=%02x%02x(len=%d)(%dth)",pbuf[12], pbuf[13],len, rxnum);
  printf("%s\r\n", str);
  //stmOzOLED_printString(str,0,4,16);
  //rxnum++;

  // Release descriptors to DMA
  // Check if frame with multiple DMA buffer segments
  if (gp_DMA_RX_FRAME_infos->Seg_Count > 1)  {
    DMARxNextDesc = gp_DMA_RX_FRAME_infos->FS_Rx_Desc;
  }else {
    DMARxNextDesc = frame.descriptor;
  }

  // Set Own bit in Rx descriptors: gives the buffers back to DMA
  for (i=0; i<gp_DMA_RX_FRAME_infos->Seg_Count; i++) {
    DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
    DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
  }

  // Clear Segment_Count
  gp_DMA_RX_FRAME_infos->Seg_Count =0;

  // When Rx Buffer unavailable flag is set: clear it and resume reception
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET) {
    // Clear RBUS ETHERNET DMA flag
    ETH->DMASR = ETH_DMASR_RBUS;
    // Resume DMA reception
    ETH->DMARPDR = 0;
  }
  return len;
}

void yIp101_init(void);
unsigned char yIp101_output(void);
int yIp101_read(uint8_t *packet, uint16_t maxlen);

const struct con_ethernet_driver yIp101_driver =
{
	"IP101ALF-100-Ethernet Driver",
    yIp101_init,
    //y2420_transmit,
    yIp101_output,
    yIp101_read
    //y2420_receiving_packet,
};

void yIp101_init(void)
{
  printf("yIp101_init\r\n");

  //uip_setethaddr(module->ethernet_address);
  //stmENC28J60Init(600, uip_lladdr.addr); //enc28j60Init(uip_ethaddr.addr); //tapdev_init();
  //stmENC28J60ShowPhyRegs();
  //stmEnc28WaitForLinkUp();

  ETH_X_BSP_Config(2);//ETH_IP101A_BSP_Config();//see stm32f4x7_eth_bsp.c //ETH_SimpleOrSwitchPHY_BSP_Config();
  //ETH_RTL9K_BSP_Config();
  //ETH_PTP_ForSwitchBoard_Config("Simple");
  ethernetif_low_level_init();
  //process_start(&yEnc28j60_driver_process, NULL);
}

unsigned char yIp101_output(void)//static int yEnc28_output(uint8_t *packet, uint16_t len)
{
#ifndef NETSTACK_CONF_WITH_IPV6
    uip_arp_out(); //Prepend Ethernet header to an outbound IP packet and see if we need to send out an ARP request.
#endif
	if(uip_len == 0) {
		return 2;//UIP_FW_ZEROLEN;
	}
#if NETSTACK_CONF_WITH_IPV6
	memcpy(&uip_buf[6], uip_lladdr.addr,6);
	uip_buf[12] = 0x08;
	uip_buf[13] = 0x00; //TEMP YOON
#endif
	if(ethernetif_low_level_output(uip_buf,uip_len+14)==ERR_OK){
		printf("T>Send\r\n");
	}else {
		printf("Send(Fail) ");
	}
	//stmENC28J60packetSend(uip_buf, uip_len+14);
	return uip_len;
}

int yIp101_read(uint8_t *packet, uint16_t maxlen)
{
	int retlen;
	//retlen = stmENC28J60packetReceive(maxlen, packet);
	retlen = yEth_low_level_input_with_ptr(packet);
	return retlen;
}

#define ethBUF ((struct uip_eth_hdr *)&uip_buf[0])

PROCESS_THREAD(yEth_driver_process, ev, data)
{
  static struct etimer e;
  int i;
  struct timer periodic_timer, arp_timer;

  PROCESS_BEGIN();

  printf("Start yEth_driver_process\r\n");

  //timer_set(&periodic_timer, CLOCK_SECOND / 2);
  timer_set(&arp_timer, CLOCK_SECOND * 10); //10sec

  //Registers output function.
  tcpip_set_outputfunc(yEnc28_driver.send);//tcpip_set_outputfunc(yIp101_driver.send);

  //Polling mode. In future, we will support Ethernet interrupt with poll...
  while(1) {

    etimer_set(&e, CLOCK_SECOND/100);

	PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

	//uip_len = yIp101_driver.read(uip_buf, UIP_BUFSIZE + 2);
	uip_len = yEnc28_driver.read(uip_buf, UIP_BUFSIZE + 2);//stmENC28J60packetReceive(UIP_BUFSIZE + 2, uip_buf);
    if (uip_len > 0) {
    	printf("RX>etype=%04x(l=%d)\r\n",UIP_HTONS(ethBUF->type),uip_len);
        if (ethBUF->type == UIP_HTONS(0x0800)) {
#ifndef NETSTACK_CONF_WITH_IPV6
            uip_arp_ipin(); //-- do noting.
#endif
            uip_len -= sizeof(struct uip_eth_hdr); //remove ethernet header.
            tcpip_input(); //this input causes to send if needed while it does tcpip_process().

        } else if (ethBUF->type == UIP_HTONS(0x0806)) {
#ifndef NETSTACK_CONF_WITH_IPV6
            uip_arp_arpin(); // If this function invocation resulted in data that
                             //   should be sent out on the network, the global variable
                             //  uip_len is set to a value > 0.
#endif
            if (uip_len > 0) {
            	//stmENC28J60packetSend(uip_buf, uip_len);
            	yEnc28_driver.send(uip_buf, uip_len) ;//response ARP
            }
        }
    }

    // We should call the ARP timer function every 10 seconds for refresh ARP cache.
   // if (timer_expired(&arp_timer)) {
   //     timer_reset(&arp_timer);
   //     uip_arp_timer();
   // }
  }
  PROCESS_END();
}
*/

