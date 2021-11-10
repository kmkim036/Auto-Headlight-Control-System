#ifndef __KSZ_COMMON_H__
#define __KSZ_COMMON_H__

//ksz_common.h
#define OLED_ADDRESS					0x78 //0x3C = 7bit address --> 0x78 (8 bit addr)
//#define LAN9355_ADDRESS					0x14 //0x0A(000 1010) 7bit address --> 0x14(8 bit addr)
#define I2C_400KHZ						1	// 0 to use default 100Khz, 1 for 400Khz
extern void stmOzOLED_init(char *str);
extern void stmOzOLED_printString(const char *String, unsigned char X, unsigned char Y, unsigned char numChar);
extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
//SlaveAddr(W)| Reg | SetSlaveAddr(R) | BurstReceiveStart for Repeated Start | GetData |
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, unsigned long I2Cspeed);

#define ETH_ALEN   6
#define IFNAMSIZ 20

#define CONFIG_KSZ_STP
//#define CONFIG_1588_PTP
#define CONFIG_NET_DSA_TAG_TAIL


/* Used to indicate type of flow control support. */
enum {
 PHY_NO_FLOW_CTRL,
 PHY_FLOW_CTRL,
 PHY_TX_ONLY,
 PHY_RX_ONLY
};

// Used to indicate link connection state.
enum {
	MEDIA_CONNECTED,
	MEDIA_DISCONNECTED,
	MEDIA_UNKNOWN
};
#if 0
/**
 * struct ksz_timer_info - Timer information data structure
 * @timer:	Kernel timer.
 * @cnt:	Running timer counter.
 * @max:	Number of times to run timer; -1 for infinity.
 * @period:	Timer period in jiffies.
 */
struct ksz_timer_info {
	struct timer_list timer;
	int cnt;
	int max;
	int period;
};

/**
 * struct ksz_counter_info - OS dependent counter information data structure
 * @counter:	Wait queue to wakeup after counters are read.
 * @time:	Next time in jiffies to read counter.
 * @read:	Indication of counters read in full or not.
 */
struct ksz_counter_info {
	wait_queue_head_t counter;
	unsigned long time;
	int read;
};

#define DEV_NAME_SIZE			32

/**
 * struct ksz_dev_attr - Sysfs data structure
 * @dev_attr:	Device attribute.
 * @dev_name:	Attribute name.
 */
struct ksz_dev_attr {
	struct device_attribute dev_attr;
	char dev_name[DEV_NAME_SIZE];
};
#endif
// Required for common switch control in ksz_sw.c
//#define SW_D u16
//#define SW_R(addr) Ksz_getReg16(addr)
//#define SW_W(addr,val) Ksz_setReg16(addr,val)

#define SW_D				u8
//#define SW_R( addr)			(sw)->reg->r8( addr)
//#define SW_W( addr, val)		(sw)->reg->w8( addr, val)
#define SW_R8(addr)			Ksz_getReg(addr,1) //reg->r8(addr)
#define SW_W8(addr, val)	Ksz_setReg(addr,val, 1) //	reg->w8(addr, val)

#define SW_SIZE				(1)
#define SW_SIZE_STR			"%02x"
//#define Ksz_port_r8				Ksz_Ksz_port_r8 //#define Ksz_port_r8 Ksz_port_r816
//#define Ksz_port_w8				Ksz_Ksz_port_w8




#endif
