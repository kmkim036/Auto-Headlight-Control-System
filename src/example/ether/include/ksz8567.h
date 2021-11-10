//=========== ksz8567.h =========================
// from ksz9897.h
#ifndef _KSZ8567_H
#define _KSZ8567_H

//========= NUM PORTS ================
#define MICREL_MAX_PORTS 	5
#define MAX_NPORTS 			6 //7
//==============ksz8795_switch Tables =====================================


#define RESERVED_MCAST_TABLE_ENTRIES	0x30
#define ACTUAL_MCAST_TABLE_ENTRIES	8
#define LEARNED_MAC_TABLE_ENTRIES	1024
#define STATIC_MAC_TABLE_ENTRIES	32 //16
#define SW_MAC_TABLE_ENTRIES		32 //#define SWITCH_MAC_TABLE_ENTRIES	16
#define MULTI_MAC_TABLE_ENTRIES		56 //80
#define RX_TABLE_ENTRIES			128
#define TX_TABLE_ENTRIES			8
#define BLOCKED_RX_ENTRIES			8
#define ACL_TABLE_LEN				14
//=====================================
// Default values are used in ksz_sw_8567 if these are not defined.
#define PRIO_QUEUES					4
#define RX_PRIO_QUEUES				8
#define KS_PRIO_IN_REG   			2
#define SW_PORT_NUM					MAX_NPORTS //6..7
#define TOTAL_PORT_NUM				SW_PORT_NUM

#define KSZ8567_COUNTER_NUM			0x20
#define TOTAL_KSZ8567_COUNTER_NUM	(KSZ8567_COUNTER_NUM + 2 +2)
#define SWITCH_COUNTER_NUM			KSZ8567_COUNTER_NUM
#define TOTAL_SW_COUNTER_NUM		TOTAL_KSZ8567_COUNTER_NUM
//=====================================================================
#include "ether/include/ksz_common.h"
//==== SPI for KSZ8567 ====================================================
// SPI_MODE_3
// Max 48MHz

//-- For use SPI2 (New Kong Board)
#define nCS0_KSZ_1      {GPIO_SetBits(GPIOD, GPIO_Pin_10);}   //PA15=0 case : GPIOA->BSRRL |= GPIO_Pin_15;
#define nCS0_KSZ_0      {GPIO_ResetBits(GPIOD, GPIO_Pin_10);} //PA15=1 case : GPIOA->BSRRH |= GPIO_Pin_15;

//-- For use SPI3 (Old Board)
//#define nCS0_KSZ_1      {GPIO_SetBits(GPIOA, GPIO_Pin_15);}   //PA15=0 case : GPIOA->BSRRL |= GPIO_Pin_15;
//#define nCS0_KSZ_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);} //PA15=1 case : GPIOA->BSRRH |= GPIO_Pin_15;

/* Tail tagging
 * HOST->KSZ : 2 byte tail tagging : tag0 = prio, tag1=bit-wise port : 0x01 = Port1, 0x02=Port2,0x04=Port3,...0x10=Port5
 * KSZ->HOST : 1 byte tail tagging : zero-based value port : 0x00=Port1, 0x02=Port2, .. 0x06 = Port7
*/

#define KSZ_INGRESS_TAG_FROM_HOST_LEN2 	2 	//HOST(TX)->KSZ
#define KSZ_EGRESS_TAG_TO_HOST_LEN1  	1	//HOST(RX)<-KSZ

/* Register Address Mapping
 *
 * +----PORT--------+-----PAGE------+-------------------+
 * |PortSpace[15:12]|FuncSpace[11:8]| RegSpace[7:0]     |
 * +----PORT--------+-----PAGE------+-------------------+
 *     0 = Global   |
 *
 * [Byte Ordering] - BigEndian
 * 16-bit RegAddr = 0x1234        :   32-bit Reg
 *   |0x12 | 0x34|                |0x12 | 0x34| 0x56 | 0x78
 *   0x100   0x101                 0x100  0x101 0x102  0x103
 *
 * [READ/WRITE SEQUENCE]
 * READ : [CMD = 011] [ADDR=A23..A0] [TA:XXXXX][DATA[7..0]] , Actual Address = 16bitSpace[15:0]. Thus A23-A16 are Don't care.
 * WRITE: [CMD = 010] [ADDR=A23..A0] [TA:XXXXX][DATA[7..0]]
 */
// READ/WRITE SEQUENCE
// [ADDR][R/W] - [RegAddr(A9~A2)] - [DATA][DATA][DATA][DATA]
// SPI frame opcodes
#define SPI_ADDR_S			12
#define SPI_ADDR_M			((1 << SPI_ADDR_S) - 1)
#define SPI_TURNAROUND_S	5

#define KS_SPIOP_RD			3
#define KS_SPIOP_WR			2



//Per Port Register
#define PORT_CTRL_ADDR(port, addr)	((addr) | (((port) + 1) << 12))



/**
 * struct ksz_mac_table - Static MAC table data structure
 * @mac_addr:	MAC address to filter.
 * @vid:	VID value.
 * @fid:	FID value.
 * @ports:	Port membership.
 * @override:	Override setting.
 * @use_fid:	FID use setting.
 * @valid:	Valid setting indicating the entry is being used.
 */
struct ksz_mac_table {
	u8 addr[ETH_ALEN];
	u32 ports;
	u16 fid;
	u8 mstp;
	u8 prio;
	u8 src:1;
	u8 dst:1;
	u8 override:1;
	u8 use_fid:1;
	u8 valid:1;
	u8 ignore_use_fid:1;
	u8 dirty:1;
};
#define FWD_HOST_OVERRIDE		(1 << 0)
#define FWD_HOST			(1 << 1)
#define FWD_STP_DEV			(1 << 2)
#define FWD_MAIN_DEV			(1 << 3)
#define FWD_VLAN_DEV			(1 << 4)
#define FWD_MCAST			(1 << 5)
#define FWD_UCAST			(1 << 6)

struct ksz_alu_table {
	u16 index;
	u16 owner;
	u8 forward;
	u8 type;
	u8 valid:1;
};

#define VLAN_TABLE_ENTRIES		4096
#define VID_IN_DATA			32
#define FID_ENTRIES			128
#define FID_IN_DATA			32
#define NUM_OF_MSTI			8
#if 0
//-----------
struct sw_dev_info {
	void *sw;
	unsigned int minor;
	u8 *write_buf;
	u8 *read_buf;
	size_t read_max;
	size_t read_len;
	size_t write_len;
	struct semaphore sem;
	struct mutex lock;
	wait_queue_head_t wait_msg;
	struct sw_dev_info *next;
};
/**
 * struct ksz_vlan_table - VLAN table data structure
 * @vid:	VID value.
 * @fid:	FID value.
 * @ports:	Port membership.
 * @untag:	Untag membership.
 * @mstp:	MSTP number.
 * @prio:	Priority
 * @fo:		Forward option.
 * @valid:	Valid setting indicating the entry is being used.
 */
struct ksz_vlan_table {
	u16 vid;
	u16 fid;
	u32 ports;
	u32 untag;
	u8 mstp;
	u8 prio;
	u8 option:1;
	u8 valid:1;
	u8 dirty:1;
};
#define PRIO_802_1P_ENTRIES		8

#define DIFFSERV_ENTRIES		64

#define ACL_TABLE_ENTRIES		16

struct ksz_acl_table {
	u16 first_rule;
	u16 ruleset;
	u8 mac[ETH_ALEN];
	u16 eth_type;
	u8 protocol;
	u8 ip4_addr[4];
	u8 ip4_mask[4];
	u32 seqnum;
	u16 max_port;
	u16 min_port;
	u8 prio;
	u8 vlan_prio;
	u16 ports;
	u16 cnt;
	u8 tcp_flag_mask;
	u8 tcp_flag;
#if 0
	u8 ip6_addr[16];
	u8 ip6_mask[16];
#endif
	u32 mode:2;
	u32 enable:2;
	u32 src:1;
	u32 equal:1;
	u32 port_mode:2;
	u32 tcp_flag_enable:1;
	u32 msec:1;
	u32 intr_mode:1;
	u32 prio_mode:2;
	u32 vlan_prio_replace:1;
	u32 map_mode:2;
	u32 changed:1;
	u32 action_changed:1;
	u32 ruleset_changed:1;
	u32 action_selected:1;

	u8 data[ACL_TABLE_LEN];
};

/**
 * struct ksz_port_mib - Port MIB data structure
 * @cnt_ptr:	Current pointer to MIB counter index.
 * @mib_start:	The starting counter index.  Some ports do not start at 0.
 * @counter:	64-bit MIB counter value.
 * @dropped:	Temporary buffer to remember last read packet dropped values.
 * @read_cnt:	Used to signal when to read the MIB counter.
 * @read_max:	Used to indicate how often to read the MIB counter.
 *
 * MIB counters needs to be read periodically so that counters do not get
 * overflowed and give incorrect values.  A right balance is needed to
 * satisfy this condition and not waste too much CPU time.
 */
struct ksz_port_mib {
	u8 cnt_ptr;
	u8 mib_start;
	u8 interval;
	u8 reserved[1];

	u64 counter[TOTAL_SWITCH_COUNTER_NUM];
	struct {
		unsigned long last;
		u64 last_cnt;
		u32 peak;
	} rate[2];
	unsigned long last_drop;
	u64 drop;
	u64 first_drop;
};

enum {
	STP_STATE_DISABLED = 0,
	STP_STATE_LISTENING,
	STP_STATE_LEARNING,
	STP_STATE_FORWARDING,
	STP_STATE_BLOCKED,
	STP_STATE_SIMPLE
};

/**
 * struct ksz_port_cfg - Port configuration data structure
 * @vid:	VID value.
 * @member:	Port membership.
 * @port_prio:	Port priority.
 * @rate_ctrl:	Priority rate control.
 * @rx_rate:	Receive priority rate.
 * @tx_rate:	Transmit priority rate.
 * @rate_limit: Priority rate limit value.
 * @vid_member:	VLAN membership.
 * @index:	Net device pointer.
 * @stp_state:	Current Spanning Tree Protocol state.
 */
struct ksz_port_cfg {
	u16 vid;
	u16 member;
	u8 rate_ctrl[PRIO_QUEUES];
	u32 rx_packet[RX_PRIO_QUEUES];
	u32 rx_rate[RX_PRIO_QUEUES];
	u32 tx_packet[PRIO_QUEUES];
	u32 tx_rate[PRIO_QUEUES];
	u32 color_map[DIFFSERV_ENTRIES / 16];
	u32 tc_map[PRIO_802_1P_ENTRIES / 8];
	u8 p_index;
	u8 q_index;
	u8 port_prio;
	u8 rate_limit;
	int packet_based;
	u16 intr_mask;
	u16 vid_member;
	int index;
	int stp_state[NUM_OF_MSTI];

	struct ksz_acl_table acl_info[ACL_TABLE_ENTRIES];
	u16 acl_index;
	u16 acl_act_index;
	u16 acl_rule_index;

	u16 mmd_id;
	u16 mmd_reg;

	u32 enabled:1;
	u32 asCapable:1;

	u16 phy_ctrl;
	u16 phy_adv;
	u16 phy_adv_g;
	u8 phy_intr;

	u8 mstp;

	u32 setup_time;
};

/**
 * struct ksz_sw_info - KSZ9897 switch information data structure
 * @mac_table:	MAC table entries information.
 * @alu_table:	ALU table entries information.
 * @multi_net:	Network multicast addresses used.
 * @multi_sys:	System multicast addresses used.
 * @port_cfg:	Port configuration information.
 * @rstp:	RSTP information.
 * @iba:	IBA information.
 * @dlr:	DLR information.
 * @hsr:	HSR information.
 * @hsr_entry:	HSR table entry information.
 * @mac_entry:	MAC table entry information.
 * @vlan_entry:	VLAN table entry information.
 * @diffserv:	DiffServ priority settings.  Possible values from 6-bit of ToS
 *		(bit7 ~ bit2) field.
 * @p_802_1p:	802.1P priority settings.  Possible values from 3-bit of 802.1p
 *		Tag priority field.
 * @br_addr:	Bridge address.  Used for STP.
 * @mac_addr:	Switch MAC address.
 * @broad_per:	Broadcast storm percentage.
 * @member:	Current port membership.  Used for STP.
 * @phy_addr:	PHY address used by first port.
 */
struct ksz_sw_info {
	struct ksz_mac_table mac_table[MULTI_MAC_TABLE_ENTRIES];
	struct ksz_alu_table alu_table[MULTI_MAC_TABLE_ENTRIES];
	int forward;
	int multi_net;
	int multi_sys;
	struct ksz_port_cfg port_cfg[TOTAL_PORT_NUM];
#ifdef CONFIG_KSZ_STP
	struct ksz_stp_info rstp;
#endif
#ifdef CONFIG_KSZ_IBA
	struct ksz_iba_info iba;
#endif
#ifdef CONFIG_KSZ_DLR
	struct ksz_dlr_info dlr;
#endif
#ifdef CONFIG_KSZ_HSR
	struct ksz_hsr_info hsr;
	struct ksz_hsr_table hsr_entry;
#endif
	struct ksz_mac_table mac_entry;
	struct ksz_vlan_table vlan_entry;

	SW_D diffserv[DIFFSERV_ENTRIES / KS_PRIO_IN_REG];
	SW_D p_802_1p[PRIO_802_1P_ENTRIES / KS_PRIO_IN_REG];

	u8 br_addr[ETH_ALEN];
	u8 mac_addr[ETH_ALEN];

	u8 vid2fid[VLAN_TABLE_ENTRIES];
	u8 fid2mstid[FID_ENTRIES];
	u32 vid[VLAN_TABLE_ENTRIES / VID_IN_DATA];
	u32 fid[FID_ENTRIES / FID_IN_DATA];
	u16 fid_cnt;

	u8 broad_per;
	u8 member[NUM_OF_MSTI];
	u8 phy_addr;
	u8 fid_updated;
};

/**
 * struct ksz_port_state - Port state information data structure
 * @state:	Connection status of the port.
 * @link_down:	Indication the link has just gone down.
 *
 * It is pointless to read MIB counters when the port is disconnected.  The
 * @state provides the connection status so that MIB counters are read only
 * when the port is connected.  The @link_down indicates the port is just
 * disconnected so that all MIB counters are read one last time to update the
 * information.
 */
struct ksz_port_state {
	uint state;
	uint tx_rate;
	u8 link_down;
};

#define TX_RATE_UNIT			10000

/**
 * struct ksz_port_info - Port information data structure
 * @interface:	PHY interface.
 * @state:	Connection status of the port.
 * @tx_rate:	Transmit rate divided by 10000 to get Mbit.
 * @duplex:	Duplex mode.
 * @flow_ctrl:	Flow control.
 * @link:	Link status.  Used to determine link.
 * @advertised:	Advertised auto-negotiation setting.  Used to determine link.
 * @partner:	Auto-negotiation partner setting.  Used to determine link.
 * @status:	LinkMD status values.
 * @length:	LinkMD length values.
 * @sqi:	Signal Quality Indicator.
 * @mac_addr:	MAC address of the port.
 * @phy_id:	PHY id used by the port.
 */
struct ksz_port_info {
	phy_interface_t interface;
	uint state;
	uint tx_rate;
	u8 duplex;
	u8 flow_ctrl;
	u8 link_down;
	u16 link;
	u32 advertised;
	u32 partner;
	u32 status[5];
	u32 length[5];
	u16 sqi;
	u8 mac_addr[ETH_ALEN];
	u8 own_flow_ctrl;
	u8 own_duplex;
	u16 own_speed;
	u8 phy_id;
	u32 report:1;
	u32 phy:1;
};

struct ksz_sw;
struct ksz_port;

struct ksz_sw_tx_tag {
	u32 timestamp;
	u16 ports;
};

struct ksz_sw_cached_regs {
	u32 ptp_unit_index;
	u16 ptp_clk_ctrl;
	u16 xmii[2];
};

/* Switch features and bug fixes. */
#define STP_SUPPORT			(1 << 0)
#define VLAN_PORT			(1 << 1)
#define VLAN_PORT_REMOVE_TAG		(1 << 2)
#define VLAN_PORT_TAGGING		(1 << 3)
#define VLAN_PORT_START			200
#define SW_VLAN_DEV			(1 << 4)
#define MRP_SUPPORT			(1 << 5)

#define ACL_CORRUPT_BUG			(1 << 8)
#define NO_GLOBAL_RESET			(1 << 9)
#define PHY_INTR_BUG			(1 << 10)
#define IS_9893				(1 << 15)
#define SETUP_PHY			(1 << 16)
#define NEW_XMII			(1 << 17)
#define USE_FEWER_PORTS			(1 << 18)
#define IBA_SUPPORT			(1 << 20)
#define NEW_CAP				(1 << 21)
#define AVB_SUPPORT			(1 << 22)
#define REDUNDANCY_SUPPORT		(1 << 23)
#define DLR_HW				(1 << 24)
#define HSR_HW				(1 << 25)
#define HSR_REDBOX			(1 << 26)
#define DSA_SUPPORT			(1 << 28)
#define DIFF_MAC_ADDR			(1 << 29)
#define QW_HW				(1 << 30)
#define PTP_HW				(1 << 31)

/* Software overrides. */
#define PAUSE_FLOW_CTRL			(1 << 0)
#define FAST_AGING			(1 << 1)
#define MCAST_FILTER			(1 << 2)
#define HAVE_MORE_THAN_2_PORTS		(1 << 3)
#define DLR_FORWARD			(1 << 4)

#define BAD_SPI				(1 << 15)
#define IBA_TEST			(1 << 16)
#define ACL_INTR_MONITOR		(1 << 17)

#define TAIL_PRP_0			(1 << 24)
#define TAIL_PRP_1			(1 << 25)

#define USE_802_1X_AUTH			(1 << 27)
#define VLAN_SET			(1 << 28)
#define PTP_TAG				(1 << 29)
#define TAG_REMOVE			(1 << 30)
#define TAIL_TAGGING			(1 << 31)

/**
 * struct ksz_sw - Virtual switch data structure
 * @dev:		Pointer to hardware device.
 * @phydev:		Pointer to PHY device interface.
 * @interface:		The R/G/MII interface used.
 * @msg_enable:		The message flags controlling driver output.
 * @hwlock:		Pointer to hardware lock.
 * @reglock:		Pointer to register lock.
 * @acllock:		ACL table lock.
 * @alulock:		ALU table lock.
 * @vlanlock:		VLAN table lock.
 * @hsrlock:		HSR table lock.
 * @lock		Software lock to switch structure.
 * @locked:		locked status.
 * @info:		Pointer to switch information structure.
 * @port_info:		Port information.
 * @netdev:		Pointer to OS dependent network devices.
 * @phy:		Pointer to OS dependent PHY devices.
 * @dev_offset:		Indication of a switch associated network device.
 * @phy_offset:		Indication of a port associated PHY device.
 * @port_state:		Port state information.
 * @port_mib:		Port MIB information.
 * @mib_cnt:		Number of MIB counters this switch has.
 * @mib_port_cnt:	Number of ports with MIB counters.
 * @phy_port_cnt:	Number of ports with actual PHY.
 * @port_cnt:		Number of ports to support.
 * @monitor_timer_info:	Timer information for monitoring ports.
 * @counter:		Pointer to OS dependent MIB counter information.
 * @link_read:		Workqueue for link monitoring.
 * @ops:		Switch function access.
 * @reg:		Switch register access.
 * @net_ops:		Network related switch function access.
 * @HOST_PORT:		A predefined value indicating the host port.
 * @HOST_MASK:		A predefined value indicating the host port mask.
 * @PORT_MASK:		A predefined value indicating the port mask.
 * @TAIL_TAG_LOOKUP:	A predefined value indicating tx tail tag lookup.
 * @TAIL_TAG_OVERRIDE:	A predefined value indicating tx tail tag override.
 * @live_ports:		Bitmap of ports with link enabled.
 * @on_ports:		Bitmap of ports with 802.1X enabled.
 * @rx_ports:		Bitmap of ports with receive enabled.
 * @tx_ports:		Bitmap of ports with transmit enabled.
 * @dev_count:		Number of network devices this switch supports.
 * @id:			Hardware ID.  Used for display only.
 * @vlan_id		Used for the VLAN port forwarding feature.
 * @vid:		Used for the VLAN port forwarding feature.
 * @revision:		Hardware revision number.
 * @features:		Switch features to enable.
 * @overrides:		Switch features to override.
 * @multi_dev:		Used to specify multiple devices mode.
 * @stp:		Used to enable STP.
 * @fast_aging:		Used to enable fast aging.
 */
struct ksz_sw {
	void *dev;
	void *phydev;
	phy_interface_t interface;
	u32 msg_enable;
	wait_queue_head_t queue;
	struct mutex *hwlock;
	struct mutex *reglock;
	struct mutex acllock;
	struct mutex alulock;
	struct mutex vlanlock;
	struct mutex hsrlock;
	struct mutex lock;
	int intr_cnt;
	int intr_using;

	struct ksz_sw_info *info;
	struct ksz_port_info port_info[SWITCH_PORT_NUM];
	struct net_device *main_dev;
	struct net_device *netdev[TOTAL_PORT_NUM];
	struct phy_device phy_map[TOTAL_PORT_NUM + 1];
	struct phy_device *phy[TOTAL_PORT_NUM + 1];
	int dev_offset;
	int phy_offset;
	struct ksz_port_state port_state[TOTAL_PORT_NUM];
	struct ksz_port_mib port_mib[TOTAL_PORT_NUM];
	u8 mib_interval_start[4];
	unsigned long next_jiffies;
	int mib_cnt;
	int mib_port_cnt;
	int phy_port_cnt;
	int dsa_port_cnt;
	int port_cnt;
	int last_port;
	struct ksz_timer_info *monitor_timer_info;
	struct ksz_counter_info *counter;
	struct delayed_work *link_read;

	const struct ksz_sw_ops *ops;
	const struct ksz_sw_reg_ops *reg;
	const struct ksz_sw_reg_ops *cur;
	struct ksz_sw_net_ops *net_ops;
	struct delayed_work set_ops;
	struct delayed_work set_mrp;
	struct work_struct set_addr;

	int HOST_PORT;
	u16 HOST_MASK;
	u16 PORT_MASK;
	u16 TAIL_TAG_LOOKUP;
	u16 TAIL_TAG_OVERRIDE;
	u32 intr_mask;
	u32 port_intr_mask;
	u32 phy_intr;
	u16 live_ports;
	u16 on_ports;
	u16 open_ports;
	u16 rx_ports[NUM_OF_MSTI];
	u16 tx_ports[NUM_OF_MSTI];
	u8 tx_pad[60];
	int tx_start;
	struct ksz_sw_tx_tag tag;
	struct ksz_sw_cached_regs cached;

	int dev_major;
	u8 *msg_buf;
	struct sw_dev_info *dev_list[2];
	struct sw_dev_info *dev_info;
	uint notifications;
	char dev_name[20];

	int dev_count;
	int id;
	u32 vlan_id;
	u16 vid;
	u16 alu_index;
	u8 alu_type;
	u8 alu_dirty;
	u16 vlan_index;
	u16 hsr_index;
	u8 hsr_dirty;
	u8 vlan_dirty;
	u8 verbose;
	u8 running;

	int revision;
	uint features;
	uint overrides;

	struct napi_struct *napi;

	int multi_dev;
	int stp;
	int fast_aging;
	struct {
		u16 cnt;
		u16 mask;
		u16 port;
		u16 phy_id;
		u16 vlan;
		uint proto;
	} eth_maps[SWITCH_PORT_NUM];
	int eth_cnt;

#ifdef CONFIG_KSZ_MRP
	struct mrp_info mrp;
#endif

#ifdef CONFIG_1588_PTP
	/* PTP structure size can be variable. */
	struct ptp_info ptp_hw;
#endif
};

struct ksz_sw_sysfs {
	struct ksz_dev_attr *ksz_port_attrs[TOTAL_PORT_NUM];
	struct attribute **port_attrs[TOTAL_PORT_NUM];
};

/**
 * struct ksz_port - Virtual port data structure
 * @first_port:		Index of first port this port supports.
 * @mib_port_cnt:	Number of ports with MIB counters.
 * @port_cnt:		Number of ports this port supports.
 * @flow_ctrl:		Flow control setting.  PHY_NO_FLOW_CTRL for no flow
 *			control, and PHY_FLOW_CTRL for flow control.
 *			PHY_TX_ONLY and PHY_RX_ONLY are not supported for 100
 *			Mbit PHY.
 * @duplex:		Duplex mode setting.  1 for half duplex, 2 for full
 *			duplex, and 0 for auto, which normally results in full
 *			duplex.
 * @speed:		Speed setting.  10 for 10 Mbit, 100 for 100 Mbit, and
 *			0 for auto, which normally results in 100 Mbit.
 * @force_link:		Force link setting.  0 for auto-negotiation, and 1 for
 *			force.
 * @linked:		Pointer to port information linked to this port.
 * @sw:			Pointer to virtual switch structure.
 */
struct ksz_port {
	int first_port;
	int mib_port_cnt;
	int port_cnt;

	u8 flow_ctrl;
	u8 duplex;
	u16 speed;
	u8 force_link;

	struct ksz_port_info *linked;

	struct ksz_sw *sw;
	struct work_struct link_update;
};

struct lan_attributes {
	int info;
	int version;
	int duplex;
	int speed;
	int force;
	int flow_ctrl;
	int features;
	int overrides;
	int mib;
	int reg;
	int vid;
	int dynamic_table;
	int static_table;
	int vlan_table;
	int hsr_table;
	int aging;
	int fast_aging;
	int link_aging;
	int bcast_per;
	int mcast_storm;
	int tx_queue_based;
	int diffserv_map;
	int p_802_1p_map;
	int vlan;
	int null_vid;
	int drop_inv_vid;
	int macaddr;
	int mirror_mode;
	int igmp_snoop;
	int ipv6_mld_snoop;
	int ipv6_mld_option;
	int aggr_backoff;
	int no_exc_drop;
	int jumbo_packet;
	int legal_packet;
	int length_check;
	int back_pressure;
	int sw_flow_ctrl;
	int sw_half_duplex;
	int sw_10_mbit;
	int fair_flow_ctrl;
	int vlan_bound;
	int double_tag;
	int isp;
	int hsr;
	int hsr_redbox_id;
	int hsr_net_id;
	int mtu;
	int unk_ucast_fwd;
	int unk_ucast_ports;
	int unk_mcast_fwd;
	int unk_mcast_ports;
	int unk_vid_fwd;
	int unk_vid_ports;
	int pass_pause;
	int pme;
	int pme_polarity;

	int host_port;
	int ports;
	int dev_start;
	int vlan_start;
	int avb;
	int stp;
	int two_dev;
	int authen;

	int alu_fid;
	int alu_use_fid;
	int alu_override;
	int alu_valid;
	int alu_mstp;
	int alu_prio;
	int alu_src;
	int alu_dst;
	int alu_ports;
	int alu_addr;
	int alu_type;
	int alu_index;
	int alu_info;

	int vlan_valid;
	int vlan_ports;
	int vlan_untag;
	int vlan_fid;
	int vlan_mstp;
	int vlan_prio;
	int vlan_option;
	int vlan_index;
	int vlan_info;
	int vid2fid;
	int fid2mstid;

#ifdef CONFIG_KSZ_STP
	int stp_br_info;
	int stp_br_on;
	int stp_br_prio;
	int stp_br_fwd_delay;
	int stp_br_max_age;
	int stp_br_hello_time;
	int stp_br_tx_hold;
	int stp_version;
#ifdef CONFIG_KSZ_MSTP
	int stp_br_max_hops;
	int stp_msti;
	int stp_msti_vid;
	int stp_mstp_cfg;
	int stp_mstp_name;
#endif
#endif
	int no_color;
	int color_red;
	int color_yellow;
	int color_green;
};
struct sw_attributes {
	int mib;
	int vid;
	int member;
	int bcast_storm;
	int diffserv;
	int p_802_1p;
	int prio_vlan;
	int prio_mac;
	int prio_acl;
	int prio_highest;
	int prio_or;
	int port_prio;
	int non_vid;
	int ingress;
	int drop_non_vlan;
	int drop_tagged;
	int replace_vid;
	int replace_prio;
	int mac_802_1x;
	int src_addr_filter;
	int vlan_lookup_0;
	int mstp;
	int rx;
	int tx;
	int learn;
	int power;
	int prio_queue;
	int rx_prio_rate;
	int tx_prio_rate;
	int limit;
	int limit_port_based;
	int limit_packet_based;
	int limit_flow_ctrl;
	int limit_cnt_ifg;
	int limit_cnt_pre;
	int rx_p0_rate;
	int rx_p1_rate;
	int rx_p2_rate;
	int rx_p3_rate;
	int rx_p4_rate;
	int rx_p5_rate;
	int rx_p6_rate;
	int rx_p7_rate;
	int tx_q0_rate;
	int tx_q1_rate;
	int tx_q2_rate;
	int tx_q3_rate;
	int color_map;
	int tc_map;
	int mirror_port;
	int mirror_rx;
	int mirror_tx;
	int back_pressure;
	int force_flow_ctrl;
	int pass_all;
	int tail_tag;

	int cust_vid;
	int sr_1_vid;
	int sr_2_vid;
	int sr_1_type;
	int sr_2_type;

	int pme_ctrl;
	int pme_status;

	int authen_mode;
	int acl;
	int acl_first_rule;
	int acl_ruleset;
	int acl_mode;
	int acl_enable;
	int acl_src;
	int acl_equal;
	int acl_addr;
	int acl_type;
	int acl_cnt;
	int acl_msec;
	int acl_intr_mode;
	int acl_ip_addr;
	int acl_ip_mask;
	int acl_protocol;
	int acl_seqnum;
	int acl_port_mode;
	int acl_max_port;
	int acl_min_port;
	int acl_tcp_flag_enable;
	int acl_tcp_flag;
	int acl_tcp_flag_mask;
	int acl_prio_mode;
	int acl_prio;
	int acl_vlan_prio_replace;
	int acl_vlan_prio;
	int acl_map_mode;
	int acl_ports;
	int acl_index;
	int acl_act_index;
	int acl_act;
	int acl_rule_index;
	int acl_info;
	int acl_table;

	int p_index;
	int q_index;
	int police_type;
	int non_dscp_color;
	int police_drop_all;
	int police_port_based;
	int color_mark;
	int color_remap;
	int drop_srp;
	int color_aware;
	int police;

	int q_cir;
	int q_pir;
	int q_cbs;
	int q_pbs;

	int wred_max;
	int wred_min;
	int wred_multiplier;
	int wred_avg_size;
	int wred_q_max;
	int wred_q_min;
	int wred_q_multiplier;
	int wred_q_avg_size;
	int wred_random_drop;
	int wred_drop_gyr;
	int wred_drop_yr;
	int wred_drop_r;
	int wred_drop_all;
	int wred_q_pmon;

	int q_scheduling;
	int q_shaping;
#ifdef MTI_PREEMPT_ENABLE
	int preempt;
#endif
	int q_tx_ratio;
	int q_credit_hi;
	int q_credit_lo;
	int q_credit_incr;
	int srp;

	int qm_drop;
	int qm_burst;
	int qm_resv_space;
	int qm_hi;
	int qm_lo;
	int qm_tx_used;
	int qm_tx_avail;
	int qm_tx_calc;

	int mmd_id;
	int mmd_reg;
	int mmd_val;

	int rx_flow_ctrl;
	int tx_flow_ctrl;

	int duplex;
	int speed;

#ifdef CONFIG_KSZ_STP
	int stp_info;
	int stp_on;
	int stp_prio;
	int stp_admin_path_cost;
	int stp_path_cost;
	int stp_admin_edge;
	int stp_auto_edge;
	int stp_mcheck;
	int stp_admin_p2p;
	int stp_auto_isolate;
#endif
	int linkmd;
	int sqi;
	int mac_loopback;
	int phy_loopback;
	int remote_loopback;
};
#endif

//REF : KSZ9897
//============== KSZ8567 Registers =====================================
//#define KS_PORT_M				0x1F
#define KS_PRIO_M				0x7
#define KS_PRIO_S				4

//(1) Global Registers
//-Reg@0x00
#define REG_CHIP_ID0			0x00
//-Reg@0x01
#define REG_CHIP_ID1			0x01
#define SW_CHIP_ID_MSB			0x85 	//FAMILY_ID
//-Reg@0x02
#define REG_CHIP_ID2			0x02
#define SW_CHIP_ID_LSB			0x67	//CHIP_ID
//-Reg@0x03
#define REG_CHIP_ID3			0x03
#define SW_CHIP_REVID			0xF0
#define SW_RESET				0x01//#define SW_SOFT_RESET			0x01
#define SW_REVISION_M			0x0F
#define SW_REVISION_S			4

//-Reg@0x006 PME Pin Control Register
#define REG_SW_PME_CTRL			0x06
#define SW_PME_ENABLE			(1 << 1) //Pin Output Enable
#define SW_PME_POLARITY			(1 << 0) //Output Polarity
//
//#define REG_GLOBAL_OPTIONS		0x000F -- NO in 8567

//-Reg@0x010
#define REG_SW_GLOBAL_INT_STATUS32	0x10 //0x10~13 //REG_SW_INT_STATUS__4
//-Reg@0x014
#define REG_SW_GLOBAL_INT_MASK32	0x14 //0x14~17 (1=int enable) //REG_SW_INT_MASK__4
#define SW_LUE_INT				(1 << 31)
#define SW_TRIG_TS_INT			(1 << 30)
#define SWITCH_INT_MASK			(SW_TRIG_TS_INT) //(TRIG_TS_INT | APB_TIMEOUT_INT)
//-Reg@0x018/0x1C
#define REG_SW_PORT_INT_STATUS32 0x18
#define REG_SW_PORT_INT_MASK32 	0x1C
//#define REG_SW_PHY_INT_STATUS32 0x20
//#define REG_SW_PHY_INT_ENABLE32 0x24



//====================Global I/O Control Reg=============================
//--Global I/O Control Reg@0x100~0x01FF
//-Reg@0x100
#define REG_SW_GLOBAL_SERIAL_CTRL 0x100	//MIIM Preamble suppression, ...
#define SPI_AUTO_EDGE_DETECTION		(1 << 1)
#define SPI_CLOCK_OUT_RISING_EDGE	(1 << 0)

//-Reg@0x103
#define REG_SW_GLOBAL_OUTPUT_CLK_CTRL 0x103	//RecovedClockReady, SYNCLK_O Source Config,...
#define SW_ENABLE_REFCLKO		(1 << 1)
#define SW_REFCLKO_IS_125MHZ	(1 << 0)

//-Reg@104~107 (In-Band Management Control Reg)
#define REG_SW_IBA__4			0x0104
#define SW_IBA_ENABLE			(1 << 31)
#define SW_IBA_DA_MATCH			(1 << 30)
#define SW_IBA_INIT			(1 << 29)
#define SW_IBA_QID_M			0xF
#define SW_IBA_QID_S			22
#define SW_IBA_PORT_M			0x2F
#define SW_IBA_PORT_S			16
#define SW_IBA_FRAME_TPID_M		0xFFFF

//-Reg@10D (I/O Driver Strength)
#define REG_SW_IO_STRENGTH__1		0x010D
#define SW_DRIVE_STRENGTH_M		0x7
#define SW_DRIVE_STRENGTH_2MA		0
#define SW_DRIVE_STRENGTH_4MA		1
#define SW_DRIVE_STRENGTH_8MA		2
#define SW_DRIVE_STRENGTH_12MA		3
#define SW_DRIVE_STRENGTH_16MA		4
#define SW_DRIVE_STRENGTH_20MA		5
#define SW_DRIVE_STRENGTH_24MA		6
#define SW_DRIVE_STRENGTH_28MA		7
#define SW_HI_SPEED_DRIVE_STRENGTH_S	4
#define SW_LO_SPEED_DRIVE_STRENGTH_S	0
//-Reg@110~113 (In-Band Management Operation Status 1 Reg)
#define REG_SW_IBA_STATUS__4		0x0110
#define SW_IBA_REQ			(1 << 31)
#define SW_IBA_RESP			(1 << 30)
#define SW_IBA_DA_MISMATCH		(1 << 14)
#define SW_IBA_FMT_MISMATCH		(1 << 13)
#define SW_IBA_CODE_ERROR		(1 << 12)
#define SW_IBA_CMD_ERROR		(1 << 11)
#define SW_IBA_CMD_LOC_M		((1 << 6) - 1)

//-Reg@120~123 (LED Override)
//-Reg@124~127 (LED Output) //Active High/low
//-Reg@128~12B (LED2_0/LED2_1 Source)

//================== GLOBAL PHY CONTROL REG =======================
//-Reg@0x201
#define REG_SW_POWER_MANAGEMNT_CTRL 0x201	//PLL PD,...
#define SW_PLL_POWER_DOWN		(1 << 5)
#define SW_POWER_DOWN_MODE		0x3
#define SW_ENERGY_DETECTION		1
#define SW_SOFTWARE_POWER_DOWN	2
#define SW_POWER_SAVING			3

#define REG_SW_LED_CONFIG_STRAP32 	0x210

//================== GLOBAL SWITCH CONTROL REG(0x300) =======================
//-Reg@0x300
#define REG_SW_OPERTATION_CTRL 	0x300
#define SW_DOUBLE_TAG			(1 << 7)
#define SW_SOFT_HARDWARE_RESET	(1 << 1)
#define SW_START_SWITCH			(1 << 0)

//-Reg@302
#define REG_SW_MAC_ADDR_0		0x302	//MACA[47..40]
#define REG_SW_MAC_ADDR_1		0x303	//MACA[39..32]
#define REG_SW_MAC_ADDR_2		0x304	//MACA[31..24]
#define REG_SW_MAC_ADDR_3		0x305	//MACA[23..16]
#define REG_SW_MAC_ADDR_4		0x306	//MACA[15..8]
#define REG_SW_MAC_ADDR_5		0x307	//MACA[7..0]

//-Reg@308
#define REG_SW_MTU16				0x308	//308~309
//-Reg@30A
#define REG_SW_ISP_TPID16			0x30A
//-Reg@30E
#define REG_SW_AVB_STRATEGY16		0x30E
#define SW_SHAPING_CREDIT_ACCT		(1 << 1)
#define SW_POLICING_CREDIT_ACCT		(1 << 0)

//-Reg@310
#define REG_SW_LUE_CTRL_0		0x310
#define SW_VLAN_ENABLE			(1 << 7)
#define SW_DROP_INVALID_VID		(1 << 6)
#define SW_AGE_CNT_M			0x7
#define SW_AGE_CNT_S			3
#define SW_RESV_MCAST_ENABLE	(1 << 2)
#define SW_HASH_OPTION_M		0x03
#define SW_HASH_OPTION_CRC		1
#define SW_HASH_OPTION_XOR		2
#define SW_HASH_OPTION_DIRECT	3
//-Reg@311
#define REG_SW_LUE_CTRL_1		0x0311
#define UNICAST_LEARN_DISABLE	(1 << 7)
#define SW_SRC_ADDR_FILTER		(1 << 6)
#define SW_FLUSH_STP_TABLE		(1 << 5)
#define SW_FLUSH_MSTP_TABLE		(1 << 4)
#define SW_FWD_MCAST_SRC_ADDR	(1 << 3)
#define SW_AGING_ENABLE			(1 << 2)
#define SW_FAST_AGING			(1 << 1)
#define SW_LINK_AUTO_AGING		(1 << 0)
//-Reg@312
#define REG_SW_LUE_CTRL_2		0x0312
#define SW_TRAP_DOUBLE_TAG		(1 << 6)
#define SW_EGRESS_VLAN_FILTER_DYN	(1 << 5)
#define SW_EGRESS_VLAN_FILTER_STA	(1 << 4)
#define SW_FLUSH_OPTION_M		0x3
#define SW_FLUSH_OPTION_S		2
#define SW_FLUSH_OPTION_DYN_MAC		1
#define SW_FLUSH_OPTION_STA_MAC		2
#define SW_FLUSH_OPTION_BOTH		3
#define SW_PRIO_M			0x3
#define SW_PRIO_DA			0
#define SW_PRIO_SA			1
#define SW_PRIO_HIGHEST_DA_SA		2
#define SW_PRIO_LOWEST_DA_SA		3

//-Reg@313-AgePeriod
#define REG_SW_LUE_CTRL_3		0x0313

//-Reg@314-ALT Interrupt
#define REG_SW_LUE_INT_STATUS		0x0314
#define REG_SW_LUE_INT_ENABLE		0x0315
#define LEARN_FAIL_INT			(1 << 2)
#define ALMOST_FULL_INT			(1 << 1)
#define WRITE_FAIL_INT			(1 << 0)

//-Reg@316 : ALT Entry Index 0
#define REG_SW_LUE_INDEX16_0		0x0316
#define ENTRY_INDEX_M				0x0FFF

#define REG_SW_LUE_INDEX16_1		0x0318
#define FAIL_INDEX_M				0x03FF
#define REG_SW_LUE_INDEX16_2		0x031A

////-Reg@320 Unknown Unicast Control
#define REG_SW_LUE_UNK_UCAST_CTRL32	0x0320
#define SW_UNK_UCAST_ENABLE		(1 << 31)

////-Reg@324 Unknown Multicast Control
#define REG_SW_LUE_UNK_MCAST_CTRL32	0x0324
#define SW_UNK_MCAST_ENABLE		(1 << 31)

//Reg@328- Unknown VLAN ID Control
#define REG_SW_LUE_UNK_VID_CTRL32	0x0328
#define SW_UNK_VID_ENABLE		(1 << 31)

////Reg@330- Switch MAC Control 0
#define REG_SW_MAC_CTRL_0		0x0330
#define SW_NEW_BACKOFF			(1 << 7) //Alternate Back-off mode ?
#define SW_CHECK_LENGTH			(1 << 3)
#define SW_PAUSE_UNH_MODE		(1 << 1)
#define SW_AGGRESSIVE_BACKOFF	(1 << 0)

////Reg@331- Switch MAC Control 1
#define REG_SW_MAC_CTRL_1		0x0331
#define MULTICAST_STORM_DISABLE		(1 << 6)
#define SW_BACK_PRESSURE		(1 << 5)
#define FAIR_FLOW_CTRL			(1 << 4)
#define NO_EXC_COLLISION_DROP		(1 << 3)
#define SW_JUMBO_PACKET			(1 << 2)
#define SW_LEGAL_PACKET_DISABLE		(1 << 1)
#define SW_PASS_SHORT_FRAME		(1 << 0)

////Reg@332- Switch MAC Control 2
#define REG_SW_MAC_CTRL_2		0x0332
#define SW_REPLACE_VID			(1 << 3)
#define BROADCAST_STORM_RATE_HI		0x07

////Reg@333- Switch MAC Control 3
#define REG_SW_MAC_CTRL_3		0x0333
#define BROADCAST_STORM_RATE_LO		0xFF
#define BROADCAST_STORM_RATE		0x07FF

////Reg@334- Switch MAC Control 4
#define REG_SW_MAC_CTRL_4		0x0334
#define SW_PASS_PAUSE			(1 << 3)

////Reg@335- Switch MAC Control 5 - Ingress rate limit period, egress rate limit
#define REG_SW_MAC_CTRL_5		0x0335
#define SW_OUT_RATE_LIMIT_QUEUE_BASED	(1 << 3)

////Reg@336- Switch MAC Control 6 - MIB
#define REG_SW_MAC_CTRL_6		0x0336
#define SW_MIB_COUNTER_FLUSH		(1 << 7)
#define SW_MIB_COUNTER_FREEZE		(1 << 6)

////Reg@338- Prio mapping Regs
#define REG_SW_MAC_802_1P_MAP_0		0x0338
#define REG_SW_MAC_802_1P_MAP_1		0x0339
#define REG_SW_MAC_802_1P_MAP_2		0x033A
#define REG_SW_MAC_802_1P_MAP_3		0x033B
#define SW_802_1P_MAP_M			KS_PRIO_M
#define SW_802_1P_MAP_S			KS_PRIO_S

//===== To be check
#if KSZ9897
#define REG_SW_MAC_ISP_CTRL		0x033C
#endif
//Reg@33C- DIFFServ
#define REG_SW_MAC_TOS_CTRL		0x033E
#define SW_TOS_DSCP_REMARK		(1 << 1)
#define SW_TOS_DSCP_REMAP		(1 << 0)

#define REG_SW_MAC_TOS_PRIO_0		0x0340
#define REG_SW_MAC_TOS_PRIO_1		0x0341
#define REG_SW_MAC_TOS_PRIO_2		0x0342
#define REG_SW_MAC_TOS_PRIO_3		0x0343
#define REG_SW_MAC_TOS_PRIO_4		0x0344
#define REG_SW_MAC_TOS_PRIO_5		0x0345
#define REG_SW_MAC_TOS_PRIO_6		0x0346
#define REG_SW_MAC_TOS_PRIO_7		0x0347
#define REG_SW_MAC_TOS_PRIO_8		0x0348
#define REG_SW_MAC_TOS_PRIO_9		0x0349
#define REG_SW_MAC_TOS_PRIO_10		0x034A
#define REG_SW_MAC_TOS_PRIO_11		0x034B
#define REG_SW_MAC_TOS_PRIO_12		0x034C
#define REG_SW_MAC_TOS_PRIO_13		0x034D
#define REG_SW_MAC_TOS_PRIO_14		0x034E
#define REG_SW_MAC_TOS_PRIO_15		0x034F
#define REG_SW_MAC_TOS_PRIO_16		0x0350
#define REG_SW_MAC_TOS_PRIO_17		0x0351
#define REG_SW_MAC_TOS_PRIO_18		0x0352
#define REG_SW_MAC_TOS_PRIO_19		0x0353
#define REG_SW_MAC_TOS_PRIO_20		0x0354
#define REG_SW_MAC_TOS_PRIO_21		0x0355
#define REG_SW_MAC_TOS_PRIO_22		0x0356
#define REG_SW_MAC_TOS_PRIO_23		0x0357
#define REG_SW_MAC_TOS_PRIO_24		0x0358
#define REG_SW_MAC_TOS_PRIO_25		0x0359
#define REG_SW_MAC_TOS_PRIO_26		0x035A
#define REG_SW_MAC_TOS_PRIO_27		0x035B
#define REG_SW_MAC_TOS_PRIO_28		0x035C
#define REG_SW_MAC_TOS_PRIO_29		0x035D
#define REG_SW_MAC_TOS_PRIO_30		0x035E
#define REG_SW_MAC_TOS_PRIO_31		0x035F

//Global Port Mirroring and Snooping Control
#define REG_SW_MRI_CTRL_0		0x0370
#define SW_IGMP_SNOOP			(1 << 6)
#define SW_IPV6_MLD_OPTION		(1 << 3)
#define SW_IPV6_MLD_SNOOP		(1 << 2)
#define SW_MIRROR_RX_TX			(1 << 0)

#if KSZ9897
//#define REG_SW_CLASS_D_IP_CTRL__4	0x0374
//#define SW_CLASS_D_IP_ENABLE		(1 << 31)
#endif
//WRED Diffserv Color Mapping
#define REG_SW_MRI_CTRL_8		0x0378
#define SW_NO_COLOR_S			6
#define SW_RED_COLOR_S			4
#define SW_YELLOW_COLOR_S		2
#define SW_GREEN_COLOR_S		0
#define SW_COLOR_M				0x3

//PTP Event Message Prio
#define REG_SW_PTP_EV_MSG_PRIO	0x037C
#define REG_SW_PTP_NONEV_MSG_PRIO	0x037D

//Queue Management Control 0
#define REG_SW_QM_CTRL32_0		0x0390
#define PRIO_SCHEME_SELECT_M	KS_PRIO_M
#define PRIO_SCHEME_SELECT_S	6
#define PRIO_MAP_3_HI			0
#define PRIO_MAP_2_HI			2
#define PRIO_MAP_0_LO			3
#define UNICAST_VLAN_BOUNDARY	(1 << 1)

//#define REG_SW_EEE_QM_CTRL__2		0x03C0
//#define REG_SW_EEE_TXQ_WAIT_TIME__2	0x03C2
//============= 0x400 GLOBAL SWITCH LOOK UP ENGINE CONTROL ==============================
#define REG_SW_VLAN_ENTRY32_0	0x0400
#define VLAN_VALID				(1 << 31)
#define VLAN_FORWARD_OPTION		(1 << 27)
#define VLAN_PRIO_M				KS_PRIO_M
#define VLAN_PRIO_S				24
#define VLAN_MSTP_M				0x7
#define VLAN_MSTP_S				12
#define VLAN_FID_M				0x7F

#define REG_SW_VLAN_ENTRY32_UNTAG_1	0x0404
#define REG_SW_VLAN_ENTRY32_PORTS_2	0x0408
#define REG_SW_VLAN_ENTRY_INDEX16	0x040C
#define VLAN_INDEX_M			0x0FFF

#define REG_SW_VLAN_TABLE_ACCESS_CTRL		0x040E
#define VLAN_START				(1 << 7)
#define VLAN_ACTION				0x3
#define VLAN_WRITE				1
#define VLAN_READ				2
#define VLAN_CLEAR				3

#define REG_SW_ALU_INDEX_0		0x0410
#define ALU_FID_INDEX_S			16
#define ALU_MAC_ADDR_HI			0xFFFF

#define REG_SW_ALU_INDEX_1		0x0414
#define ALU_DIRECT_INDEX_M		((1 << 12) - 1)

#define REG_SW_ALU_CTRL32		0x0418
#define ALU_VALID_CNT_M			((1 << 14) - 1)
#define ALU_VALID_CNT_S			16
#define ALU_START				(1 << 7)
#define ALU_VALID				(1 << 6)
#define ALU_VALID_OR_STOP		(1 << 5)
#define ALU_DIRECT				(1 << 2)
#define ALU_ACTION				0x3
#define ALU_WRITE				1
#define ALU_READ				2
#define ALU_SEARCH				3

//Static Address and Reserved Multicast Table Control Register@0x41C~041F
#define REG_SW_ALU_STAT_CTRL32	0x041C

#define ALU_STAT_INDEX_M		((1 << 4) - 1)		//TABLE_INDEX [19:16]-Static Address Table
#define ALU_STAT_INDEX_S		16
#define ALU_RESV_MCAST_INDEX_M	((1 << 6) - 1)	//TABLE_INDEX [21:16]-Reserved Multicast Table
#define ALU_STAT_START			(1 << 7)	//START_FINISH	BIT8
#define ALU_RESV_MCAST_ADDR		(1 << 1)	//TABLESELECT 	BIT1
#define ALU_STAT_READ			(1 << 0) 	//ACTION 		BIT0

#define REG_SW_ALU_STATIC_ADDR_TABLE_ENTRY32_A		0x0420
#define ALU_V_STATIC_VALID		(1 << 31)
#define ALU_V_SRC_FILTER		(1 << 30)
#define ALU_V_DST_FILTER		(1 << 29)
#define ALU_V_PRIO_AGE_CNT_M	((1 << 3) - 1)
#define ALU_V_PRIO_AGE_CNT_S	26
#define ALU_V_MSTP_M			0x7

#define REG_SW_ALU_STATIC_ADDR_TABLE_ENTRY32_B		0x0424

#define ALU_V_OVERRIDE			(1 << 31)
#define ALU_V_USE_FID			(1 << 30)
#define ALU_V_PORT_MAP			((1 << 24) - 1)

#define REG_SW_ALU_STATIC_ADDR_TABLE_ENTRY32_C		0x0428

#define ALU_V_FID_M				((1 << 16) - 1)
#define ALU_V_FID_S				16
#define ALU_V_MAC_ADDR_HI		0xFFFF

#define REG_SW_ALU_STATIC_ADDR_TABLE_ENTRY32_D		0x042C
#if KSZ9897
//#define REG_HSR_ALU_INDEX_0		0x0440
//#define REG_HSR_ALU_INDEX_1		0x0444
//#define HSR_DST_MAC_INDEX_LO_S		16
//#define HSR_SRC_MAC_INDEX_HI		0xFFFF
//#define REG_HSR_ALU_INDEX_2		0x0448
//#define HSR_INDEX_MAX			(1 << 9)
//#define HSR_DIRECT_INDEX_M		(HSR_INDEX_MAX - 1)

#define REG_HSR_ALU_INDEX_3		0x044C

#define HSR_PATH_INDEX_M		((1 << 4) - 1)

#define REG_HSR_ALU_CTRL__4		0x0450

#define HSR_VALID_CNT_M			((1 << 14) - 1)
#define HSR_VALID_CNT_S			16
#define HSR_START			(1 << 7)
#define HSR_VALID			(1 << 6)
#define HSR_SEARCH_END			(1 << 5)
#define HSR_DIRECT			(1 << 2)
#define HSR_ACTION			0x3
#define HSR_WRITE			1
#define HSR_READ			2
#define HSR_SEARCH			3

#define REG_HSR_ALU_VAL_A		0x0454
#define HSR_V_STATIC_VALID		(1 << 31)
#define HSR_V_AGE_CNT_M			((1 << 3) - 1)
#define HSR_V_AGE_CNT_S			26
#define HSR_V_PATH_ID_M			((1 << 4) - 1)

#define REG_HSR_ALU_VAL_B		0x0458

#define REG_HSR_ALU_VAL_C		0x045C
#define HSR_V_DST_MAC_ADDR_LO_S		16
#define HSR_V_SRC_MAC_ADDR_HI		0xFFFF

#define REG_HSR_ALU_VAL_D		0x0460

#define REG_HSR_ALU_VAL_E		0x0464
#define HSR_V_START_SEQ_1_S		16
#define HSR_V_START_SEQ_2_S		0

#define REG_HSR_ALU_VAL_F		0x0468
#define HSR_V_EXP_SEQ_1_S		16
#define HSR_V_EXP_SEQ_2_S		0

#define REG_HSR_ALU_VAL_G		0x046C
#define HSR_V_SEQ_CNT_1_S		16
#define HSR_V_SEQ_CNT_2_S		0

#define HSR_V_SEQ_M			((1 << 16) - 1)
#endif

//=======0x500 : GLOBAL SWITCH PTP CONTROL REG ==========
#define REG_PTP_CLK_CTRL		0x0500

#define PTP_STEP_ADJ			(1 << 6)
#define PTP_STEP_DIR			(1 << 5)
#define PTP_READ_TIME			(1 << 4)
#define PTP_LOAD_TIME			(1 << 3)
#define PTP_CLK_ADJ_ENABLE		(1 << 2)
#define PTP_CLK_ENABLE			(1 << 1)
#define PTP_CLK_RESET			(1 << 0)

#define REG_PTP_RTC_SUB_NANOSEC__2	0x0502

#define PTP_RTC_SUB_NANOSEC_M		0x0007

#define REG_PTP_RTC_NANOSEC		0x0504
#define REG_PTP_RTC_NANOSEC_H		0x0504
#define REG_PTP_RTC_NANOSEC_L		0x0506

#define REG_PTP_RTC_SEC			0x0508
#define REG_PTP_RTC_SEC_H		0x0508
#define REG_PTP_RTC_SEC_L		0x050A

#define REG_PTP_SUBNANOSEC_RATE		0x050C
#define REG_PTP_SUBNANOSEC_RATE_H	0x050C

#define PTP_RATE_DIR			(1 << 31)
#define PTP_TMP_RATE_ENABLE		(1 << 30)

#define REG_PTP_SUBNANOSEC_RATE_L	0x050E

#define REG_PTP_RATE_DURATION		0x0510
#define REG_PTP_RATE_DURATION_H		0x0510
#define REG_PTP_RATE_DURATION_L		0x0512

#define REG_PTP_MSG_CONF1		0x0514

#define PTP_802_1AS			(1 << 7)
#define PTP_ENABLE			(1 << 6)
#define PTP_ETH_ENABLE			(1 << 5)
#define PTP_IPV4_UDP_ENABLE		(1 << 4)
#define PTP_IPV6_UDP_ENABLE		(1 << 3)
#define PTP_TC_P2P			(1 << 2)
#define PTP_MASTER			(1 << 1)
#define PTP_1STEP			(1 << 0)

#define REG_PTP_MSG_CONF2		0x0516

#define PTP_UNICAST_ENABLE		(1 << 12)
#define PTP_ALTERNATE_MASTER		(1 << 11)
#define PTP_ALL_HIGH_PRIO		(1 << 10)
#define PTP_SYNC_CHECK			(1 << 9)
#define PTP_DELAY_CHECK			(1 << 8)
#define PTP_PDELAY_CHECK		(1 << 7)
#define PTP_DROP_SYNC_DELAY_REQ		(1 << 5)
#define PTP_DOMAIN_CHECK		(1 << 4)
#define PTP_UDP_CHECKSUM		(1 << 2)

#define REG_PTP_DOMAIN_VERSION		0x0518
#define PTP_VERSION_M			0xFF00
#define PTP_DOMAIN_M			0x00FF

#define REG_PTP_UNIT_INDEX__4		0x0520

#define PTP_UNIT_M			0xF

/* 2013-09-10 */
#define PTP_GPIO_INDEX_S		16
#define PTP_TSI_INDEX_S			8
#define PTP_TOU_INDEX_S			0

#define REG_PTP_TRIG_STATUS__4		0x0524

#define TRIG_ERROR_S			16
#define TRIG_DONE_S			0

#define REG_PTP_INT_STATUS__4		0x0528

#define TRIG_INT_S			16
#define TS_INT_S			0

#define TRIG_UNIT_M			0x7
#define TS_UNIT_M			0x3

#define REG_PTP_CTRL_STAT__4		0x052C

#define GPIO_IN				(1 << 7)
#define GPIO_OUT			(1 << 6)
#define TS_INT_ENABLE			(1 << 5)
#define TRIG_ACTIVE			(1 << 4)
#define TRIG_ENABLE			(1 << 3)
#define TRIG_RESET			(1 << 2)
#define TS_ENABLE			(1 << 1)
#define TS_RESET			(1 << 0)

#define GPIO_CTRL_M			\
	(GPIO_IN | GPIO_OUT)

#define TRIG_CTRL_M			\
	(TRIG_ACTIVE | TRIG_ENABLE | TRIG_RESET)

#define TS_CTRL_M			\
	(TS_INT_ENABLE | TS_ENABLE | TS_RESET)

#define REG_TRIG_TARGET_NANOSEC		0x0530
#define REG_TRIG_TARGET_SEC		0x0534

#define REG_TRIG_CTRL__4		0x0538

#define TRIG_CASCADE_ENABLE		(1 << 31)
#define TRIG_CASCADE_TAIL		(1 << 30)
#define TRIG_CASCADE_UPS_M		0xF
#define TRIG_CASCADE_UPS_S		26
#define TRIG_NOW			(1 << 25)
#define TRIG_NOTIFY			(1 << 24)
#define TRIG_EDGE			(1 << 23)
#define TRIG_PATTERN_S			20
#define TRIG_PATTERN_M			0x7
#define TRIG_NEG_EDGE			0
#define TRIG_POS_EDGE			1
#define TRIG_NEG_PULSE			2
#define TRIG_POS_PULSE			3
#define TRIG_NEG_PERIOD			4
#define TRIG_POS_PERIOD			5
#define TRIG_REG_OUTPUT			6
#define TRIG_GPO_S			16
#define TRIG_GPO_M			0xF
#define TRIG_CASCADE_ITERATE_CNT_M	0xFFFF

#define REG_TRIG_CYCLE_WIDTH		0x053C

#define REG_TRIG_CYCLE_CNT		0x0540

#define TRIG_CYCLE_CNT_M		0xFFFF
#define TRIG_CYCLE_CNT_S		16
#define TRIG_BIT_PATTERN_M		0xFFFF

#define REG_TRIG_ITERATE_TIME		0x0544

#define REG_TRIG_PULSE_WIDTH__4		0x0548

#define TRIG_PULSE_WIDTH_M		0x00FFFFFF

#define REG_TS_CTRL_STAT__4		0x0550

#define TS_EVENT_DETECT_M		0xF
#define TS_EVENT_DETECT_S		17
#define TS_EVENT_OVERFLOW		(1 << 16)
#define TS_GPI_M			0xF
#define TS_GPI_S			8
#define TS_DETECT_RISE			(1 << 7)
#define TS_DETECT_FALL			(1 << 6)
#define TS_DETECT_S			6
#define TS_CASCADE_TAIL			(1 << 5)
#define TS_CASCADE_UPS_M		0xF
#define TS_CASCADE_UPS_S		1
#define TS_CASCADE_ENABLE		(1 << 0)

#define DETECT_RISE			(TS_DETECT_RISE >> TS_DETECT_S)
#define DETECT_FALL			(TS_DETECT_FALL >> TS_DETECT_S)

#define REG_TS_EVENT_0_NANOSEC		0x0554
#define REG_TS_EVENT_0_SEC		0x0558
#define REG_TS_EVENT_0_SUB_NANOSEC	0x055C

#define REG_TS_EVENT_1_NANOSEC		0x0560
#define REG_TS_EVENT_1_SEC		0x0564
#define REG_TS_EVENT_1_SUB_NANOSEC	0x0568

#define REG_TS_EVENT_2_NANOSEC		0x056C
#define REG_TS_EVENT_2_SEC		0x0570
#define REG_TS_EVENT_2_SUB_NANOSEC	0x0574

#define REG_TS_EVENT_3_NANOSEC		0x0578
#define REG_TS_EVENT_3_SEC		0x057C
#define REG_TS_EVENT_3_SUB_NANOSEC	0x0580

#define REG_TS_EVENT_4_NANOSEC		0x0584
#define REG_TS_EVENT_4_SEC		0x0588
#define REG_TS_EVENT_4_SUB_NANOSEC	0x058C

#define REG_TS_EVENT_5_NANOSEC		0x0590
#define REG_TS_EVENT_5_SEC		0x0594
#define REG_TS_EVENT_5_SUB_NANOSEC	0x0598

#define REG_TS_EVENT_6_NANOSEC		0x059C
#define REG_TS_EVENT_6_SEC		0x05A0
#define REG_TS_EVENT_6_SUB_NANOSEC	0x05A4

#define REG_TS_EVENT_7_NANOSEC		0x05A8
#define REG_TS_EVENT_7_SEC		0x05AC
#define REG_TS_EVENT_7_SUB_NANOSEC	0x05B0

#define TS_EVENT_EDGE_M			0x1
#define TS_EVENT_EDGE_S			30
#define TS_EVENT_NANOSEC_M		((1 << 30) - 1)

#define TS_EVENT_SUB_NANOSEC_M		0x7

#define TS_EVENT_SAMPLE			\
	(REG_TS_EVENT_1_NANOSEC - REG_TS_EVENT_0_NANOSEC)

#define REG_GLOBAL_RR_INDEX__1		0x0600

//======== DLR ==============================
#define REG_DLR_SRC_PORT__4		0x0604

#define DLR_SRC_PORT_UNICAST		(1 << 31)
#define DLR_SRC_PORT_M			0x3
#define DLR_SRC_PORT_BOTH		0
#define DLR_SRC_PORT_EACH		1

#define REG_DLR_IP_ADDR__4		0x0608

#define REG_DLR_CTRL__1			0x0610

#define DLR_RESET_SEQ_ID		(1 << 3)
#define DLR_BACKUP_AUTO_ON		(1 << 2)
#define DLR_BEACON_TX_ENABLE		(1 << 1)
#define DLR_ASSIST_ENABLE		(1 << 0)

#define REG_DLR_STATE__1		0x0611

#define DLR_NODE_STATE_M		0x3
#define DLR_NODE_STATE_S		1
#define DLR_NODE_STATE_IDLE		0
#define DLR_NODE_STATE_FAULT		1
#define DLR_NODE_STATE_NORMAL		2
#define DLR_RING_STATE_FAULT		0
#define DLR_RING_STATE_NORMAL		1

#define REG_DLR_PRECEDENCE__1		0x0612

#define REG_DLR_BEACON_INTERVAL__4	0x0614

#define REG_DLR_BEACON_TIMEOUT__4	0x0618

#define REG_DLR_TIMEOUT_WINDOW__4	0x061C

#define DLR_TIMEOUT_WINDOW_M		((1 << 22) - 1)

#define REG_DLR_VLAN_ID__2		0x0620

#define DLR_VLAN_ID_M			((1 << 12) - 1)

#define REG_DLR_DEST_ADDR_0		0x0622
#define REG_DLR_DEST_ADDR_1		0x0623
#define REG_DLR_DEST_ADDR_2		0x0624
#define REG_DLR_DEST_ADDR_3		0x0625
#define REG_DLR_DEST_ADDR_4		0x0626
#define REG_DLR_DEST_ADDR_5		0x0627

#define REG_DLR_PORT_MAP__4		0x0628

#define REG_DLR_CLASS__1		0x062C

#define DLR_FRAME_QID_M			0x3

#if KSZ9897
//=================HSR================================
#define REG_HSR_PORT_MAP__4		0x0640

#define REG_HSR_ALU_CTRL_0__1		0x0644

#define HSR_DUPLICATE_DISCARD		(1 << 7)
#define HSR_NODE_UNICAST		(1 << 6)
#define HSR_AGE_CNT_DEFAULT_M		0x7
#define HSR_AGE_CNT_DEFAULT_S		3
#define HSR_LEARN_MCAST_DISABLE		(1 << 2)
#define HSR_HASH_OPTION_M		0x3
#define HSR_HASH_DISABLE		0
#define HSR_HASH_UPPER_BITS		1
#define HSR_HASH_LOWER_BITS		2
#define HSR_HASH_XOR_BOTH_BITS		3

#define REG_HSR_ALU_CTRL_1__1		0x0645

#define HSR_LEARN_UCAST_DISABLE		(1 << 7)
#define HSR_FLUSH_TABLE			(1 << 5)
#define HSR_PROC_MCAST_SRC		(1 << 3)
#define HSR_AGING_ENABLE		(1 << 2)

#define REG_HSR_ALU_CTRL_2__2		0x0646

#define REG_HSR_ALU_AGE_PERIOD__4	0x0648

#define REG_HSR_ALU_INT_STATUS__1	0x064C
#define REG_HSR_ALU_INT_MASK__1		0x064D

#define HSR_WINDOW_OVERFLOW_INT		(1 << 3)
#define HSR_LEARN_FAIL_INT		(1 << 2)
#define HSR_ALMOST_FULL_INT		(1 << 1)
#define HSR_WRITE_FAIL_INT		(1 << 0)

#define REG_HSR_ALU_ENTRY_0__2		0x0650

#define HSR_ENTRY_INDEX_M		((1 << 10) - 1)
#define HSR_FAIL_INDEX_M		((1 << 8) - 1)

#define REG_HSR_ALU_ENTRY_1__2		0x0652

#define HSR_FAIL_LEARN_INDEX_M		((1 << 8) - 1)

#define REG_HSR_ALU_ENTRY_3__2		0x0654

#define HSR_CPU_ACCESS_ENTRY_INDEX_M	((1 << 8) - 1)
#endif
//================== PER PORT REGISTERS (0xN000) ==============================
// N= Port Number.
// N = 1~7 : for some registers
// N = 6~7 : for MAC port specific registers
// N = 0~5 : PHY specific registers
//======================  0 - Operation ====================================
#define REG_PORT_DEFAULT_VID		0x0000 //Port Default Tag0@N000; Port Default Tag1@N001;
//#define REG_PORT_CUSTOM_VID			0x0002
//#define REG_PORT_AVB_SR_1_VID		0x0004
//#define REG_PORT_AVB_SR_2_VID		0x0006
//#define REG_PORT_AVB_SR_1_TYPE		0x0008
//#define REG_PORT_AVB_SR_2_TYPE		0x000A

//Port PME_WoL Event Register@13(N=1~7)
#define REG_PORT_PME_STATUS		0x0013
//Port PME_WoL Enable Register@17(N=1~7)
#define REG_PORT_PME_CTRL		0x0017
#define PME_WOL_MAGICPKT		(1 << 2)
#define PME_WOL_LINKUP			(1 << 1)
#define PME_WOL_ENERGY			(1 << 0)
//Port Interrupt Register@1B(N=1~7)
#define REG_PORT_INT_STATUS		0x001B
#define REG_PORT_INT_MASK		0x001F
#define PORT_SGMII_INT			(1 << 3)
#define PORT_PTP_INT			(1 << 2)
#define PORT_PHY_INT			(1 << 1)
#define PORT_ACL_INT			(1 << 0)
#define PORT_INT_MASK			(PORT_SGMII_INT | PORT_PTP_INT | PORT_PHY_INT | PORT_ACL_INT)

//Port Operation Control Register@20(N=1~7)
#define REG_PORT_CTRL_0			0x0020
#define PORT_MAC_LOOPBACK		(1 << 7)
#define PORT_FORCE_TX_FLOW_CTRL	(1 << 4)
#define PORT_FORCE_RX_FLOW_CTRL	(1 << 3)
#define PORT_TAIL_TAG_ENABLE	(1 << 2) ////////////////// DSA ////////////////
#define PORT_QUEUE_SPLIT_ENABLE	0x3

#if KSZ9897
#define REG_PORT_CTRL_1			0x0021
#define PORT_SRP_ENABLE			0x3
#endif
#define REG_PORT_STATUS_0		0x0030
#define PORT_INTF_SPEED_M		0x3
#define PORT_INTF_SPEED_S		3
#define PORT_INTF_FULL_DUPLEX		(1 << 2)
#define PORT_TX_FLOW_CTRL		(1 << 1)
#define PORT_RX_FLOW_CTRL		(1 << 0)

#if KSZ9897
#define REG_PORT_STATUS_1		0x0034
#endif

// 1 - PHY (0xN100~0xN1FF)
#define REG_PORT_PHY_BASIC_CTRL	0x0100
#define PORT_PHY_RESET			(1 << 15)
#define PORT_PHY_LOOPBACK		(1 << 14)
#define PORT_SPEED_100MBIT		(1 << 13)
#define PORT_AUTO_NEG_ENABLE		(1 << 12)
#define PORT_POWER_DOWN			(1 << 11)
#define PORT_ISOLATE			(1 << 10)
#define PORT_AUTO_NEG_RESTART		(1 << 9)
#define PORT_FULL_DUPLEX		(1 << 8)
#define PORT_COLLISION_TEST		(1 << 7)
#if KSZ9897
#define PORT_SPEED_1000MBIT		(1 << 6)
#define PHY_HP_MDIX				(1 << 5)
#define PHY_REMOTE_FAULT_DISABLE	(1 << 2)
#define PHY_LED_DISABLE			(1 << 0)
#endif

#define REG_PORT_PHY_BASIC_STATUS	0x0102
#define PORT_100BT4_CAPABLE		(1 << 15)
#define PORT_100BTX_FD_CAPABLE		(1 << 14)
#define PORT_100BTX_CAPABLE		(1 << 13)
#define PORT_10BT_FD_CAPABLE		(1 << 12)
#define PORT_10BT_CAPABLE		(1 << 11)
#define PORT_EXTENDED_STATUS		(1 << 8)
#define PORT_MII_SUPPRESS_CAPABLE	(1 << 6)
#define PORT_AUTO_NEG_ACKNOWLEDGE	(1 << 5)
#define PORT_REMOTE_FAULT		(1 << 4)
#define PORT_AUTO_NEG_CAPABLE		(1 << 3)
#define PORT_LINK_STATUS		(1 << 2)
#define PORT_JABBER_DETECT		(1 << 1)
#define PORT_EXTENDED_CAPABILITY	(1 << 0)

#define REG_PORT_PHY_ID_HI		0x0104
#define REG_PORT_PHY_ID_LO		0x0106
#if KSZ9897
#define KSZ9897_ID_HI			0x0022
#define KSZ9897_ID_LO			0x1622
#endif
#define REG_PORT_PHY_AUTO_NEGOTIATION	0x0108
#define PORT_AUTO_NEG_NEXT_PAGE		(1 << 15)
#define PORT_AUTO_NEG_REMOTE_FAULT	(1 << 13)
#define PORT_AUTO_NEG_ASYM_PAUSE	(1 << 11)
#define PORT_AUTO_NEG_SYM_PAUSE		(1 << 10)
#define PORT_AUTO_NEG_100BT4		(1 << 9)
#define PORT_AUTO_NEG_100BTX_FD		(1 << 8)
#define PORT_AUTO_NEG_100BTX		(1 << 7)
#define PORT_AUTO_NEG_10BT_FD		(1 << 6)
#define PORT_AUTO_NEG_10BT			(1 << 5)
#define PORT_AUTO_NEG_SELECTOR		0x001F
#define PORT_AUTO_NEG_802_3			0x0001
#define PORT_AUTO_NEG_PAUSE			(PORT_AUTO_NEG_ASYM_PAUSE | PORT_AUTO_NEG_SYM_PAUSE)

#define REG_PORT_PHY_REMOTE_CAPABILITY	0x010A
#define PORT_REMOTE_NEXT_PAGE		(1 << 15)
#define PORT_REMOTE_ACKNOWLEDGE		(1 << 14)
#define PORT_REMOTE_REMOTE_FAULT	(1 << 13)
#define PORT_REMOTE_ASYM_PAUSE		(1 << 11)
#define PORT_REMOTE_SYM_PAUSE		(1 << 10)
#define PORT_REMOTE_100BTX_FD		(1 << 8)
#define PORT_REMOTE_100BTX			(1 << 7)
#define PORT_REMOTE_10BT_FD			(1 << 6)
#define PORT_REMOTE_10BT			(1 << 5)

//PHY Auto-Neg Expansion Status Reg
#define REG_PORT_PHY_AN_EXP_STATUS	0x010C
//PHY Auto-Neg-NP Reg
#define REG_PORT_PHY_AN_NP			0x010E
//PHY Auto-Neg-LP_NPA Reg
#define REG_PORT_PHY_AN_LP_NPA		0x0110

//
#define REG_PORT_PHY_1000_CTRL		0x0112
#define PORT_AUTO_NEG_MANUAL		(1 << 12)
#define PORT_AUTO_NEG_MASTER		(1 << 11)
#define PORT_AUTO_NEG_MASTER_PREFERRED	(1 << 10)
#define PORT_AUTO_NEG_1000BT_FD		(1 << 9)
#define PORT_AUTO_NEG_1000BT		(1 << 8)

#define REG_PORT_PHY_1000_STATUS	0x0114
#define PORT_MASTER_FAULT		(1 << 15)
#define PORT_LOCAL_MASTER		(1 << 14)
#define PORT_LOCAL_RX_OK		(1 << 13)
#define PORT_REMOTE_RX_OK		(1 << 12)
#define PORT_REMOTE_1000BT_FD		(1 << 11)
#define PORT_REMOTE_1000BT		(1 << 10)
#define PORT_REMOTE_IDLE_CNT_M		0x0F

#define PORT_PHY_1000_STATIC_STATUS	\
	(PORT_LOCAL_RX_OK |		\
	PORT_REMOTE_RX_OK |		\
	PORT_REMOTE_1000BT_FD |		\
	PORT_REMOTE_1000BT)

#define REG_PORT_PHY_MMD_SETUP		0x011A
#define PORT_MMD_OP_MODE_M			0x3
#define PORT_MMD_OP_MODE_S			14
#define PORT_MMD_OP_INDEX			0
#define PORT_MMD_OP_DATA_NO_INCR	1
#define PORT_MMD_OP_DATA_INCR_RW	2
#define PORT_MMD_OP_DATA_INCR_W		3
#define PORT_MMD_DEVICE_ID_M		0x1F

#define MMD_SETUP(mode, dev)		\
	(((u16) mode << PORT_MMD_OP_MODE_S) | dev)

//PHY MMD Data Register
#define REG_PORT_PHY_MMD_INDEX_DATA	0x011C
#define MMD_DEVICE_ID_DSP			1
#define MMD_DSP_SQI_CHAN_A			0xAC
#define MMD_DSP_SQI_CHAN_B			0xAD
#define MMD_DSP_SQI_CHAN_C			0xAE
#define MMD_DSP_SQI_CHAN_D			0xAF
#define DSP_SQI_ERR_DETECTED		(1 << 15)
#define DSP_SQI_AVG_ERR				0x7FFF
#define MMD_DEVICE_ID_COMMON		2
#define MMD_DEVICE_ID_EEE_ADV		7
#define MMD_EEE_ADV					0x3C
#define EEE_ADV_100MBIT				(1 << 1)
#define EEE_ADV_1GBIT				(1 << 2)
#define MMD_EEE_LP_ADV				0x3D
#define MMD_EEE_MSG_CODE			0x3F
#define MMD_DEVICE_ID_AFED			0x1C

#define REG_PORT_PHY_EXTENDED_STATUS	0x011E
#define PORT_100BTX_FD_ABLE		(1 << 15)
#define PORT_100BTX_ABLE		(1 << 14)
#define PORT_10BT_FD_ABLE		(1 << 13)
#define PORT_10BT_ABLE			(1 << 12)


// PHY registers greater than 15 needs to be written in 32-bit.
#define REG_PORT_PHY_REMOTE_LB_LED	0x0122
#define PORT_REMOTE_LOOPBACK		(1 << 8)
#define PORT_LED_SELECT			(3 << 6)
#define PORT_LED_CTRL			(3 << 4)
#define PORT_LED_CTRL_TEST		(1 << 3)
#define PORT_10BT_PREAMBLE		(1 << 2)
#define PORT_LINK_MD_10BT_ENABLE	(1 << 1)
#define PORT_LINK_MD_PASS		(1 << 0)

#define REG_PORT_PHY_LINK_MD		0x0124
#define PORT_START_CABLE_DIAG		(1 << 15)
#define PORT_TX_DISABLE			(1 << 14)
#define PORT_CABLE_DIAG_PAIR_M		0x3
#define PORT_CABLE_DIAG_PAIR_S		12
#define PORT_CABLE_DIAG_SELECT_M	0x3
#define PORT_CABLE_DIAG_SELECT_S	10
#define PORT_CABLE_DIAG_RESULT_M	0x3
#define PORT_CABLE_DIAG_RESULT_S	8
#define PORT_CABLE_STAT_NORMAL		0
#define PORT_CABLE_STAT_OPEN		1
#define PORT_CABLE_STAT_SHORT		2
#define PORT_CABLE_STAT_FAILED		3
#if 0
#define PORT_CABLE_10M_SHORT		(1 << 12)
#endif
#define PORT_CABLE_FAULT_COUNTER	0x00FF

#define REG_PORT_PHY_PMA_STATUS		0x0126
#define PORT_1000_LINK_GOOD		(1 << 1)
#define PORT_100_LINK_GOOD		(1 << 0)

#define REG_PORT_PHY_DIGITAL_STATUS	0x0128
#define PORT_LINK_DETECT		(1 << 14)
#define PORT_SIGNAL_DETECT		(1 << 13)
#define PORT_PHY_STAT_MDI		(1 << 12)
#define PORT_PHY_STAT_MASTER		(1 << 11)

#define REG_PORT_PHY_RXER_COUNTER	0x012A
#define REG_PORT_PHY_INT_ENABLE		0x0136
#define REG_PORT_PHY_INT_STATUS		0x0137
#define JABBER_INT			(1 << 7)
#define RX_ERR_INT			(1 << 6)
#define PAGE_RX_INT			(1 << 5)
#define PARALLEL_DETECT_FAULT_INT	(1 << 4)
#define LINK_PARTNER_ACK_INT		(1 << 3)
#define LINK_DOWN_INT			(1 << 2)
#define REMOTE_FAULT_INT		(1 << 1)
#define LINK_UP_INT			(1 << 0)

#define REG_PORT_PHY_DIGITAL_DEBUG_1	0x0138
#define PORT_REG_CLK_SPEED_25_MHZ	(1 << 14)
#define PORT_PHY_FORCE_MDI		(1 << 7)
#define PORT_PHY_AUTO_MDIX_DISABLE	(1 << 6)
/* Same as PORT_PHY_LOOPBACK */
#define PORT_PHY_PCS_LOOPBACK		(1 << 0)

#define REG_PORT_PHY_PHY_CTRL		0x013E
#define PORT_INT_PIN_HIGH		(1 << 14)
#define PORT_ENABLE_JABBER		(1 << 9)
#define PORT_STAT_SPEED_1000MBIT	(1 << 6)
#define PORT_STAT_SPEED_100MBIT		(1 << 5)
#define PORT_STAT_SPEED_10MBIT		(1 << 4)
#define PORT_STAT_FULL_DUPLEX		(1 << 3)
// Same as PORT_PHY_STAT_MASTER
#define PORT_STAT_MASTER		(1 << 2)
#define PORT_RESET			(1 << 1)
#define PORT_LINK_STATUS_FAIL		(1 << 0)

#if 0
//======= 200
#define REG_PORT_SGMII_ADDR__4		0x0200
#define PORT_SGMII_AUTO_INCR		(1 << 23)
#define PORT_SGMII_DEVICE_ID_M		0x1F
#define PORT_SGMII_DEVICE_ID_S		16
#define PORT_SGMII_ADDR_M		((1 << 21) - 1)

#define REG_PORT_SGMII_DATA__4		0x0204
#define PORT_SGMII_DATA_M		((1 << 16) - 1)

#define MMD_DEVICE_ID_PMA		0x01
#define MMD_DEVICE_ID_PCS		0x03
#define MMD_DEVICE_ID_PHY_XS		0x04
#define MMD_DEVICE_ID_DTE_XS		0x05
#define MMD_DEVICE_ID_AN		0x07
#define MMD_DEVICE_ID_VENDOR_CTRL	0x1E
#define MMD_DEVICE_ID_VENDOR_MII	0x1F

#define SR_MII				MMD_DEVICE_ID_VENDOR_MII

#define MMD_SR_MII_CTRL			0x0000

#define SR_MII_RESET			(1 << 15)
#define SR_MII_LOOPBACK			(1 << 14)
#define SR_MII_SPEED_100MBIT		(1 << 13)
#define SR_MII_AUTO_NEG_ENABLE		(1 << 12)
#define SR_MII_POWER_DOWN		(1 << 11)
#define SR_MII_AUTO_NEG_RESTART		(1 << 9)
#define SR_MII_FULL_DUPLEX		(1 << 8)
#define SR_MII_SPEED_1000MBIT		(1 << 6)

#define MMD_SR_MII_STATUS		0x0001
#define MMD_SR_MII_ID_1			0x0002
#define MMD_SR_MII_ID_2			0x0003
#define MMD_SR_MII_AUTO_NEGOTIATION	0x0004

#define SR_MII_AUTO_NEG_NEXT_PAGE	(1 << 15)
#define SR_MII_AUTO_NEG_REMOTE_FAULT_M	0x3
#define SR_MII_AUTO_NEG_REMOTE_FAULT_S	12
#define SR_MII_AUTO_NEG_NO_ERROR	0
#define SR_MII_AUTO_NEG_OFFLINE		1
#define SR_MII_AUTO_NEG_LINK_FAILURE	2
#define SR_MII_AUTO_NEG_ERROR		3
#define SR_MII_AUTO_NEG_PAUSE_M		0x3
#define SR_MII_AUTO_NEG_PAUSE_S		7
#define SR_MII_AUTO_NEG_NO_PAUSE	0
#define SR_MII_AUTO_NEG_ASYM_PAUSE_TX	1
#define SR_MII_AUTO_NEG_SYM_PAUSE	2
#define SR_MII_AUTO_NEG_ASYM_PAUSE_RX	3
#define SR_MII_AUTO_NEG_HALF_DUPLEX	(1 << 6)
#define SR_MII_AUTO_NEG_FULL_DUPLEX	(1 << 5)

#define MMD_SR_MII_REMOTE_CAPABILITY	0x0005
#define MMD_SR_MII_AUTO_NEG_EXP		0x0006
#define MMD_SR_MII_AUTO_NEG_EXT		0x000F

#define MMD_SR_MII_DIGITAL_CTRL_1	0x8000


#define MMD_SR_MII_AUTO_NEG_CTRL	0x8001

#define SR_MII_8_BIT			(1 << 8)
#define SR_MII_SGMII_LINK_UP		(1 << 4)
#define SR_MII_TX_CFG_PHY_MASTER	(1 << 3)
#define SR_MII_PCS_MODE_M		0x3
#define SR_MII_PCS_MODE_S		1
#define SR_MII_PCS_SGMII		2
#define SR_MII_AUTO_NEG_COMPLETE_INTR	(1 << 0)

#define MMD_SR_MII_AUTO_NEG_STATUS	0x8002

#define SR_MII_STAT_LINK_UP		(1 << 4)
#define SR_MII_STAT_M			0x3
#define SR_MII_STAT_S			2
#define SR_MII_STAT_10_MBPS		0
#define SR_MII_STAT_100_MBPS		1
#define SR_MII_STAT_1000_MBPS		2
#define SR_MII_STAT_FULL_DUPLEX		(1 << 1)

#define MMD_SR_MII_PHY_CTRL		0x80A0

#define SR_MII_PHY_LANE_SEL_M		0xF
#define SR_MII_PHY_LANE_SEL_S		8
#define SR_MII_PHY_WRITE		(1 << 1)
#define SR_MII_PHY_START_BUSY		(1 << 0)

#define MMD_SR_MII_PHY_ADDR		0x80A1

#define SR_MII_PHY_ADDR_M		((1 << 16) - 1)

#define MMD_SR_MII_PHY_DATA		0x80A2

#define SR_MII_PHY_DATA_M		((1 << 16) - 1)

#define SR_MII_PHY_JTAG_CHIP_ID_HI	0x000C
#define SR_MII_PHY_JTAG_CHIP_ID_LO	0x000D
#endif

//========== 3 - xMII ==========================
#define REG_PORT_XMII_CTRL_0		0x0300
//#define PORT_SGMII_SEL			(1 << 7)
#define PORT_MII_FULL_DUPLEX		(1 << 6)
#define PORT_MII_TX_FLOW_CTRL		(1 << 5)
#define PORT_MII_100MBIT		(1 << 4)
#define PORT_MII_RX_FLOW_CTRL		(1 << 3)
//#define PORT_GRXC_ENABLE		(1 << 0)

#define REG_PORT_XMII_CTRL_1		0x0301
//#define PORT_RMII_CLK_SEL		(1 << 7)
#define PORT_MII_1000MBIT_S1		(1 << 6) //S1
#define PORT_MII_NOT_1GBIT		(1 << 6) //S2
#define PORT_MII_SEL_EDGE		(1 << 5)
#define PORT_RGMII_ID_IG_ENABLE		(1 << 4)
#define PORT_RGMII_ID_EG_ENABLE		(1 << 3)
#define PORT_MII_MAC_MODE		(1 << 2)
#define PORT_MII_SEL_M			0x3
// S1
#define PORT_MII_SEL_S1			0x0
#define PORT_RMII_SEL_S1		0x1
#define PORT_GMII_SEL_S1		0x2
#define PORT_RGMII_SEL_S1		0x3
// S2
#define PORT_RGMII_SEL			0x0
#define PORT_RMII_SEL			0x1
#define PORT_GMII_SEL			0x2
#define PORT_MII_SEL			0x3

// 4 - MAC ========
#define REG_PORT_MAC_CTRL_0		0x0400
#define PORT_BROADCAST_STORM		(1 << 1)
#define PORT_JUMBO_FRAME		(1 << 0)

#define REG_PORT_MAC_CTRL_1		0x0401
#define PORT_BACK_PRESSURE		(1 << 3)
#define PORT_PASS_ALL			(1 << 0)

#define REG_PORT_MAC_CTRL_2		0x0402
#define PORT_100BT_EEE_DISABLE		(1 << 7)
#define PORT_1000BT_EEE_DISABLE		(1 << 6)

#define REG_PORT_MAC_IN_RATE_LIMIT	0x0403
#define PORT_IN_PORT_BASED_S		6
#define PORT_RATE_PACKET_BASED_S	5
#define PORT_IN_FLOW_CTRL_S		4
#define PORT_COUNT_IFG_S		1
#define PORT_COUNT_PREAMBLE_S		0
#define PORT_IN_PORT_BASED		(1 << 6)
#define PORT_IN_PACKET_BASED		(1 << 5)
#define PORT_IN_FLOW_CTRL		(1 << 4)
#define PORT_IN_LIMIT_MODE_M		0x3
#define PORT_IN_LIMIT_MODE_S		2
#define PORT_IN_ALL			0
#define PORT_IN_UNICAST			1
#define PORT_IN_MULTICAST		2
#define PORT_IN_BROADCAST		3
#define PORT_COUNT_IFG			(1 << 1)
#define PORT_COUNT_PREAMBLE		(1 << 0)

#define REG_PORT_IN_RATE_0		0x0410
#define REG_PORT_IN_RATE_1		0x0411
#define REG_PORT_IN_RATE_2		0x0412
#define REG_PORT_IN_RATE_3		0x0413
#define REG_PORT_IN_RATE_4		0x0414
#define REG_PORT_IN_RATE_5		0x0415
#define REG_PORT_IN_RATE_6		0x0416
#define REG_PORT_IN_RATE_7		0x0417

#define REG_PORT_OUT_RATE_0		0x0420
#define REG_PORT_OUT_RATE_1		0x0421
#define REG_PORT_OUT_RATE_2		0x0422
#define REG_PORT_OUT_RATE_3		0x0423

#define PORT_RATE_LIMIT_M		((1 << 7) - 1)

//====== 5 - MIB Counters
#define REG_PORT_MIB_CTRL_STAT32	0x0500
#define MIB_COUNTER_OVERFLOW		(1 << 31)
#define MIB_COUNTER_VALID		(1 << 30)
#define MIB_COUNTER_READ		(1 << 25)
#define MIB_COUNTER_FLUSH_FREEZE	(1 << 24)
#define MIB_COUNTER_INDEX_M		((1 << 8) - 1)
#define MIB_COUNTER_INDEX_S		16
#define MIB_COUNTER_DATA_HI_M		0xF

#define REG_PORT_MIB_DATA		0x0504

//========= 6 - ACL
#define REG_PORT_ACL_0			0x0600
#define ACL_FIRST_RULE_M		0xF

#define REG_PORT_ACL_1			0x0601
#define ACL_MODE_M				0x3
#define ACL_MODE_S				4
#define ACL_MODE_DISABLE		0
#define ACL_MODE_LAYER_2		1
#define ACL_MODE_LAYER_3		2
#define ACL_MODE_LAYER_4		3
#define ACL_ENABLE_M			0x3
#define ACL_ENABLE_S			2
#define ACL_ENABLE_2_COUNT		0
#define ACL_ENABLE_2_TYPE		1
#define ACL_ENABLE_2_MAC		2
#define ACL_ENABLE_2_BOTH		3
#define ACL_ENABLE_3_IP			1
#define ACL_ENABLE_3_SRC_DST_COMP	2
#define ACL_ENABLE_4_PROTOCOL		0
#define ACL_ENABLE_4_TCP_PORT_COMP	1
#define ACL_ENABLE_4_UDP_PORT_COMP	2
#define ACL_ENABLE_4_TCP_SEQN_COMP	3
#define ACL_SRC				(1 << 1)
#define ACL_EQUAL			(1 << 0)

#define REG_PORT_ACL_2			0x0602
#define REG_PORT_ACL_3			0x0603

#define ACL_MAX_PORT			0xFFFF

#define REG_PORT_ACL_4			0x0604
#define REG_PORT_ACL_5			0x0605

#define ACL_MIN_PORT			0xFFFF
#define ACL_IP_ADDR				0xFFFFFFFF
#define ACL_TCP_SEQNUM			0xFFFFFFFF

#define REG_PORT_ACL_6			0x0606

#define ACL_RESERVED			0xF8
#define ACL_PORT_MODE_M			0x3
#define ACL_PORT_MODE_S			1
#define ACL_PORT_MODE_DISABLE		0
#define ACL_PORT_MODE_EITHER		1
#define ACL_PORT_MODE_IN_RANGE		2
#define ACL_PORT_MODE_OUT_OF_RANGE	3

#define REG_PORT_ACL_7			0x0607
#define ACL_TCP_FLAG_ENABLE		(1 << 0)

#define REG_PORT_ACL_8			0x0608
#define ACL_TCP_FLAG_M			0xFF

#define REG_PORT_ACL_9			0x0609
#define ACL_TCP_FLAG			0xFF
#define ACL_ETH_TYPE			0xFFFF
#define ACL_IP_M				0xFFFFFFFF

#define REG_PORT_ACL_A			0x060A
#define ACL_PRIO_MODE_M			0x3
#define ACL_PRIO_MODE_S			6
#define ACL_PRIO_MODE_DISABLE	0
#define ACL_PRIO_MODE_HIGHER	1
#define ACL_PRIO_MODE_LOWER		2
#define ACL_PRIO_MODE_REPLACE	3
#define ACL_PRIO_M				KS_PRIO_M
#define ACL_PRIO_S				3
#define ACL_VLAN_PRIO_REPLACE	(1 << 2)
#define ACL_VLAN_PRIO_M			KS_PRIO_M
#define ACL_VLAN_PRIO_HI_M		0x3

#define REG_PORT_ACL_B			0x060B
#define ACL_VLAN_PRIO_LO_M		0x8
#define ACL_VLAN_PRIO_S			7
#define ACL_MAP_MODE_M			0x3
#define ACL_MAP_MODE_S			5
#define ACL_MAP_MODE_DISABLE		0
#define ACL_MAP_MODE_OR			1
#define ACL_MAP_MODE_AND		2
#define ACL_MAP_MODE_REPLACE	3

#define ACL_CNT_M				((1 << 11) - 1)
#define ACL_CNT_S				5

#define REG_PORT_ACL_C			0x060C

#define REG_PORT_ACL_D			0x060D
#define ACL_MSEC_UNIT			(1 << 6)
#define ACL_INTR_MODE			(1 << 5)
#define ACL_PORT_MAP			0x7F

#define REG_PORT_ACL_E			0x060E
#define REG_PORT_ACL_F			0x060F

#define REG_PORT_ACL_BYTE_EN_MSB	0x0610
#define REG_PORT_ACL_BYTE_EN_LSB	0x0611

#define ACL_ACTION_START		0xA
#define ACL_ACTION_LEN			4
#define ACL_INTR_CNT_START		0xD
#define ACL_RULESET_START		0xE
#define ACL_RULESET_LEN			2
#define ACL_TABLE_LEN			16

#define ACL_ACTION_ENABLE		0x003C
#define ACL_MATCH_ENABLE		0x7FC0
#define ACL_RULESET_ENABLE		0x8003
#define ACL_BYTE_ENABLE			0xFFFF

#define REG_PORT_ACL_CTRL_0		0x0612
#define PORT_ACL_WRITE_DONE		(1 << 6)
#define PORT_ACL_READ_DONE		(1 << 5)
#define PORT_ACL_WRITE			(1 << 4)
#define PORT_ACL_INDEX_M		0xF

#define REG_PORT_ACL_CTRL_1		0x0613

//===== 8 - Ingress Classification and Policing
#define REG_PORT_MRI_MIRROR_CTRL	0x0800

#define PORT_MIRROR_RX			(1 << 6)
#define PORT_MIRROR_TX			(1 << 5)
#define PORT_MIRROR_SNIFFER		(1 << 1)

#define REG_PORT_MRI_PRIO_CTRL		0x0801
#define PORT_HIGHEST_PRIO		(1 << 7)
#define PORT_OR_PRIO			(1 << 6)
#define PORT_MAC_PRIO_ENABLE		(1 << 4)
#define PORT_VLAN_PRIO_ENABLE		(1 << 3)
#define PORT_802_1P_PRIO_ENABLE		(1 << 2)
#define PORT_DIFFSERV_PRIO_ENABLE	(1 << 1)
#define PORT_ACL_PRIO_ENABLE		(1 << 0)

#define REG_PORT_MRI_MAC_CTRL		0x0802
#define PORT_USER_PRIO_CEILING		(1 << 7)
#define PORT_DROP_NON_VLAN		(1 << 4)
#define PORT_DROP_TAG			(1 << 3)
#define PORT_BASED_PRIO_M		KS_PRIO_M
#define PORT_BASED_PRIO_S		0

#define REG_PORT_MRI_AUTHEN_CTRL	0x0803
#define PORT_ACL_ENABLE			(1 << 2)
#define PORT_AUTHEN_MODE		0x3
#define PORT_AUTHEN_NORMAL		0
#define PORT_AUTHEN_BLOCK		1
#define PORT_AUTHEN_PASS		2
#define PORT_AUTHEN_TRAP		3

#define REG_PORT_MRI_INDEX__4		0x0804
#define MRI_INDEX_P_M			0x7
#define MRI_INDEX_P_S			16
#define MRI_INDEX_Q_M			0x3
#define MRI_INDEX_Q_S			0

#define REG_PORT_MRI_TC_MAP__4		0x0808
#define PORT_TC_MAP_M			0xf
#define PORT_TC_MAP_S			4

#define REG_PORT_MRI_POLICE_CTRL__4	0x080C
#define POLICE_DROP_ALL			(1 << 10)
#define POLICE_PACKET_TYPE_M		0x3
#define POLICE_PACKET_TYPE_S		8
#define POLICE_PACKET_DROPPED		0
#define POLICE_PACKET_GREEN		1
#define POLICE_PACKET_YELLOW		2
#define POLICE_PACKET_RED		3
#define PORT_BASED_POLICING		(1 << 7)
#define NON_DSCP_COLOR_M		0x3
#define NON_DSCP_COLOR_S		5
#define COLOR_MARK_ENABLE		(1 << 4)
#define COLOR_REMAP_ENABLE		(1 << 3)
#define POLICE_DROP_SRP			(1 << 2)
#define POLICE_COLOR_NOT_AWARE		(1 << 1)
#define POLICE_ENABLE			(1 << 0)

#define REG_PORT_POLICE_COLOR_0__4	0x0810
#define REG_PORT_POLICE_COLOR_1__4	0x0814
#define REG_PORT_POLICE_COLOR_2__4	0x0818
#define REG_PORT_POLICE_COLOR_3__4	0x081C

#define POLICE_COLOR_MAP_S		2
#define POLICE_COLOR_MAP_M		((1 << POLICE_COLOR_MAP_S) - 1)

#define REG_PORT_POLICE_RATE__4		0x0820
#define POLICE_CIR_S			16
#define POLICE_PIR_S			0

#define REG_PORT_POLICE_BURST_SIZE__4	0x0824
#define POLICE_BURST_SIZE_M		0x3FFF
#define POLICE_CBS_S			16
#define POLICE_PBS_S			0

#define REG_PORT_WRED_PM_CTRL_0__4	0x0830
#define WRED_PM_CTRL_M			((1 << 11) - 1)
#define WRED_PM_MAX_THRESHOLD_S		16
#define WRED_PM_MIN_THRESHOLD_S		0

#define REG_PORT_WRED_PM_CTRL_1__4	0x0834
#define WRED_PM_MULTIPLIER_S		16
#define WRED_PM_AVG_QUEUE_SIZE_S	0

#define REG_PORT_WRED_QUEUE_CTRL_0__4	0x0840
#define REG_PORT_WRED_QUEUE_CTRL_1__4	0x0844

#define REG_PORT_WRED_QUEUE_PMON__4	0x0848
#define WRED_RANDOM_DROP_ENABLE		(1 << 31)
#define WRED_PMON_FLUSH			(1 << 30)
#define WRED_DROP_GYR_DISABLE		(1 << 29)
#define WRED_DROP_YR_DISABLE		(1 << 28)
#define WRED_DROP_R_DISABLE		(1 << 27)
#define WRED_DROP_ALL			(1 << 26)
#define WRED_PMON_M			((1 << 24) - 1)

//========== 9 - Egress Shaping ==================

#define REG_PORT_MTI_QUEUE_INDEX32	0x0900

#define REG_PORT_MTI_QUEUE_CTRL32_0	0x0904
#define MTI_PVID_REPLACE		(1 << 0)

#define REG_PORT_MTI_QUEUE_CTRL_0	0x0914
#define MTI_SCHEDULE_MODE_M		0x3
#define MTI_SCHEDULE_MODE_S		6
#define MTI_SCHEDULE_STRICT_PRIO	0
#define MTI_SCHEDULE_WRR		2
#define MTI_SHAPING_M			0x3
#define MTI_SHAPING_S			4
#define MTI_SHAPING_OFF			0
#define MTI_SHAPING_SRP			1
#define MTI_SHAPING_TIME_AWARE		2
#if 0
#define MTI_PREEMPT_ENABLE		(1 << 3)
#endif

#define REG_PORT_MTI_QUEUE_CTRL_1		0x0915
#define MTI_TX_RATIO_M					((1 << 7) - 1)

#define REG_PORT_MTI_QUEUE_CTRL16_2		0x0916
#define REG_PORT_MTI_HI_WATER_MARK		0x0916
#define REG_PORT_MTI_QUEUE_CTRL16_3		0x0918
#define REG_PORT_MTI_LO_WATER_MARK		0x0918  //Def = 0x05F2
#define REG_PORT_MTI_QUEUE_CTRL16_4		0x091A
#define REG_PORT_MTI_CREDIT_INCREMENT16	0x091A //Def= 0x2000

#define REG_PORT_MTI_TAS_CTRL			0x0920	//Enable TAS Cut-through, RefTimeSelect.
#define REG_PORT_MTI_TAS_EVENT_INDEX	0x0923
#define REG_PORT_MTI_TAS_EVENT32		0x0924	//Scheduled Open, GB start, Scheduled Closed.

/* A - QueueManagement */

#define REG_PORT_QM_CTRL32				0x0A00 //Drop Mode
#define PORT_QM_DROP_PRIO_M				0x3

#define REG_PORT_VLAN_MEMBERSHIP32		0x0A04
#if KSZ9897
#define REG_PORT_QM_QUEUE_INDEX32		0x0A08
#define PORT_QM_QUEUE_INDEX_S		24
#define PORT_QM_BURST_SIZE_S		16
#define PORT_QM_MIN_RESV_SPACE_M	((1 << 11) - 1)
#define REG_PORT_QM_WATER_MARK__4	0x0A0C
#define PORT_QM_HI_WATER_MARK_S		16
#define PORT_QM_LO_WATER_MARK_S		0
#define PORT_QM_WATER_MARK_M		((1 << 11) - 1)
#define REG_PORT_QM_TX_CNT_0__4		0x0A10

#define PORT_QM_TX_CNT_USED_S		0
#define PORT_QM_TX_CNT_M		((1 << 11) - 1)

#define REG_PORT_QM_TX_CNT_1__4		0x0A14

#define PORT_QM_TX_CNT_CALCULATED_S	16
#define PORT_QM_TX_CNT_AVAIL_S		0
#endif
//========== B - LUE Address Look Up Control Reg
#define REG_PORT_LUE_CTRL		0x0B00
#define PORT_VLAN_LOOKUP_VID_0		(1 << 7)
#define PORT_INGRESS_FILTER		(1 << 6)
#define PORT_DISCARD_NON_VID		(1 << 5)
#define PORT_MAC_BASED_802_1X		(1 << 4)
#define PORT_SRC_ADDR_FILTER		(1 << 3) //Self-Address Filtering-Port Enable

#define REG_PORT_LUE_MSTP_INDEX		0x0B01

#define REG_PORT_LUE_MSTP_STATE		0x0B04
#define PORT_TX_ENABLE			(1 << 2)
#define PORT_RX_ENABLE			(1 << 1)
#define PORT_LEARN_DISABLE		(1 << 0)

//====== C - PTP

#define REG_PTP_PORT_RX_DELAY16		0x0C00	//Rx Latency in nsec.
#define REG_PTP_PORT_TX_DELAY16		0x0C02
#define REG_PTP_PORT_ASYM_DELAY16	0x0C04	//Asymmetry Correction

#define REG_PTP_PORT_XDELAY_TS		0x0C08
#define REG_PTP_PORT_XDELAY_TS_H	0x0C08
#define REG_PTP_PORT_XDELAY_TS_L	0x0C0A

#define REG_PTP_PORT_SYNC_TS		0x0C0C
#define REG_PTP_PORT_SYNC_TS_H		0x0C0C
#define REG_PTP_PORT_SYNC_TS_L		0x0C0E

#define REG_PTP_PORT_PDRESP_TS		0x0C10
#define REG_PTP_PORT_PDRESP_TS_H	0x0C10
#define REG_PTP_PORT_PDRESP_TS_L	0x0C12

#define REG_PTP_PORT_TX_INT_STATUS__2	0x0C14
#define REG_PTP_PORT_TX_INT_ENABLE__2	0x0C16

#define PTP_PORT_SYNC_INT		(1 << 15)
#define PTP_PORT_XDELAY_REQ_INT		(1 << 14)
#define PTP_PORT_PDELAY_RESP_INT	(1 << 13)

#define REG_PTP_PORT_LINK_DELAY__4	0x0C18

//=====================================================================
/* Required for common switch control in ksz_sw_8567.c */
#define SW_D				u8
#define SW_R(sw, addr)		(sw)->reg->r8(sw, addr)
#define SW_W(sw, addr, val)	(sw)->reg->w8(sw, addr, val)
#define SW_SIZE				(1)
#define SW_SIZE_STR			"%02x"
#define port_r				port_r8
#define port_w				port_w8

#define P_BCAST_STORM_CTRL		REG_PORT_MAC_CTRL_0
#define P_PRIO_CTRL				REG_PORT_MRI_PRIO_CTRL
#define P_MIRROR_CTRL			REG_PORT_MRI_MIRROR_CTRL
#define P_STP_CTRL				REG_PORT_LUE_MSTP_STATE
#define P_PHY_CTRL				REG_PORT_PHY_CTRL
#define P_NEG_RESTART_CTRL		REG_PORT_PHY_CTRL
#define P_LINK_STATUS			REG_PORT_PHY_STATUS
#define P_SPEED_STATUS			REG_PORT_PHY_PHY_CTRL
#define P_RATE_LIMIT_CTRL		REG_PORT_MAC_IN_RATE_LIMIT

#define PORT_FORCE_FLOW_CTRL   (PORT_FORCE_TX_FLOW_CTRL | PORT_FORCE_RX_FLOW_CTRL) //YOON

#define S_LINK_AGING_CTRL		REG_SW_LUE_CTRL_1
#define S_MIRROR_CTRL			REG_SW_MRI_CTRL_0
#define S_REPLACE_VID_CTRL		REG_SW_MAC_CTRL_2
#define S_802_1P_PRIO_CTRL		REG_SW_MAC_802_1P_MAP_0
#define S_TOS_PRIO_CTRL			REG_SW_MAC_TOS_PRIO_0
#define S_FLUSH_TABLE_CTRL		REG_SW_LUE_CTRL_1

#define REG_SWITCH_RESET		REG_RESET_CTRL

#define SW_FLUSH_DYN_MAC_TABLE	SW_FLUSH_MSTP_TABLE

#define MAX_TIMESTAMP_UNIT		2
#define MAX_TRIG_UNIT			3
#define MAX_TIMESTAMP_EVENT_UNIT	8
#define MAX_GPIO				4

#define PTP_TRIG_UNIT_M			((1 << MAX_TRIG_UNIT) - 1)
#define PTP_TS_UNIT_M			((1 << MAX_TIMESTAMP_UNIT) - 1)


//==========================================
extern unsigned long Ksz_getReg(unsigned long reg, unsigned char len);

// Required for common switch control in ksz_sw.c
//#define SW_D u16
//#define SW_R(addr) Ksz_getReg16(addr)
//#define SW_W(addr,val) Ksz_setReg16(addr,val)

#define SW_D					u8
//#define SW_R( addr)			(sw)->reg->r8( addr)
//#define SW_W( addr, val)		(sw)->reg->w8( addr, val)
#define SW_R8(addr)				Ksz_getReg(addr,1) //reg->r8(addr)
#define SW_W8(addr, val)		Ksz_setReg(addr,val, 1) //	reg->w8(addr, val)
#define SW_R16(addr)			Ksz_getReg(addr,2) //reg->r8(addr)
#define SW_W16(addr, val)		Ksz_setReg(addr,val, 2) //	reg->w8(addr, val)
#define SW_R32(addr)			Ksz_getReg(addr,4) //reg->r8(addr)
#define SW_W32(addr, val)		Ksz_setReg(addr,val, 4) //	reg->w8(addr, val)

#define SW_SIZE					(1)
#define SW_SIZE_STR				"%02x"
//#define Ksz_port_r8				Ksz_port_r8 //#define Ksz_port_r8 Ksz_port_r816
//#define Ksz_port_w8				Ksz_port_w8

/* Switch features and bug fixes. */
#define STP_SUPPORT			(1 << 0)
#define VLAN_PORT			(1 << 1)
#define VLAN_PORT_REMOVE_TAG		(1 << 2)
#define VLAN_PORT_TAGGING		(1 << 3)
#define VLAN_PORT_START			200
#define SW_VLAN_DEV			(1 << 4)

#define USE_FEWER_PORTS			(1 << 18)
#define DLR_HW				(1 << 24)
#define HSR_HW				(1 << 25)

#define DSA_SUPPORT			(1 << 28)
#define DIFF_MAC_ADDR			(1 << 30)

/* Software overrides. */
#define PAUSE_FLOW_CTRL			(1 << 0)
#define FAST_AGING			(1 << 1)

#define ACL_INTR_MONITOR		(1 << 17)

#define TAIL_PRP_0			(1 << 24)
#define TAIL_PRP_1			(1 << 25)

#define TAG_REMOVE			(1 << 30)
#define TAIL_TAGGING			(1 << 31)


enum phy_state {
	PHY_DOWN = 0,
	PHY_STARTING,
	PHY_READY,
	PHY_PENDING,
	PHY_UP,
	PHY_AN,
	PHY_RUNNING,
	PHY_NOLINK,
	PHY_FORCING,
	PHY_CHANGELINK,
	PHY_HALTED,
	PHY_RESUMING
};
typedef enum {
	PHY_INTERFACE_MODE_NA,
	PHY_INTERFACE_MODE_MII,
	PHY_INTERFACE_MODE_GMII,
	PHY_INTERFACE_MODE_SGMII,
	PHY_INTERFACE_MODE_TBI,
	PHY_INTERFACE_MODE_REVMII,
	PHY_INTERFACE_MODE_RMII,
	PHY_INTERFACE_MODE_RGMII,
	PHY_INTERFACE_MODE_RGMII_ID,
	PHY_INTERFACE_MODE_RGMII_RXID,
	PHY_INTERFACE_MODE_RGMII_TXID,
	PHY_INTERFACE_MODE_RTBI,
	PHY_INTERFACE_MODE_SMII,
	PHY_INTERFACE_MODE_XGMII,
	PHY_INTERFACE_MODE_MOCA,
	PHY_INTERFACE_MODE_QSGMII,
	PHY_INTERFACE_MODE_MAX,
} phy_interface_t;


struct ksz_sw_cached_regs {
	u16 ptp_clk_ctrl;
};

// Switch features and bug fixes.
#define STP_SUPPORT			(1 << 0)
#define VLAN_PORT			(1 << 1)
#define VLAN_PORT_REMOVE_TAG		(1 << 2)
#define VLAN_PORT_TAGGING		(1 << 3)
#define VLAN_PORT_START			200
#define SW_VLAN_DEV			(1 << 4)
#define MRP_SUPPORT			(1 << 5)
#define DLR_HW				(1 << 24)
#define HSR_HW				(1 << 25)
#define DSA_SUPPORT			(1 << 28)
#define DIFF_MAC_ADDR			(1 << 30)
#ifdef CONFIG_1588_PTP
#define PTP_HW				(1 << 31)
#endif

// Software overrides.
#define PAUSE_FLOW_CTRL			(1 << 0)
#define FAST_AGING			(1 << 1)
#define TAIL_PRP_0			(1 << 24)
#define TAIL_PRP_1			(1 << 25)
#define TAG_REMOVE			(1 << 30)
#define TAIL_TAGGING			(1 << 31)

#define STATIC_MAC_TABLE_ADDR		0x0000FFFF
#define STATIC_MAC_TABLE_FWD_PORTS	0x00070000
#define STATIC_MAC_TABLE_VALID		0x00080000
#define STATIC_MAC_TABLE_OVERRIDE	0x00100000
#define STATIC_MAC_TABLE_USE_FID	0x00200000
#define STATIC_MAC_TABLE_FID		0x03C00000

#define STATIC_MAC_FWD_PORTS_SHIFT	16
#define STATIC_MAC_FID_SHIFT		22

#define DYNAMIC_MAC_TABLE_ADDR		0x0000FFFF
#define DYNAMIC_MAC_TABLE_FID		0x000F0000
#define DYNAMIC_MAC_TABLE_SRC_PORT	0x00300000
#define DYNAMIC_MAC_TABLE_TIMESTAMP	0x00C00000
#define DYNAMIC_MAC_TABLE_ENTRIES	0xFF000000

#define DYNAMIC_MAC_TABLE_ENTRIES_H	0x03
#define DYNAMIC_MAC_TABLE_MAC_EMPTY	0x04
#define DYNAMIC_MAC_TABLE_RESERVED	0x78
#define DYNAMIC_MAC_TABLE_NOT_READY	0x80

#define DYNAMIC_MAC_FID_SHIFT		16
#define DYNAMIC_MAC_SRC_PORT_SHIFT	20
#define DYNAMIC_MAC_TIMESTAMP_SHIFT	22
#define DYNAMIC_MAC_ENTRIES_SHIFT	24
#define DYNAMIC_MAC_ENTRIES_H_SHIFT	8

//YOON
#define IND_ACC_TABLE(table)		((table) << 8)
//(4) ==== Advanced Control Registers (for static MAC addr table, VLAN table, dynamic addr table, PME, ACL tables, EEE and MIB registers ================
#define REG_IND_CTRL_0			0x6E //Advanced Control Register @ 0x6E (IND== Indirect Access Control)
//- Register 0x6E/6F are used for PME, EEE, and ACL, those are inditect data registers.
#define REG_IND_DATA_PME_EEE_ACL	0xA0

#define HW_DELAY(reg)		\
	do {					\
		u16 dummy;			\
		dummy = SW_R8(reg);	\
	} while (0)


enum {
		TABLE_STATIC_MAC = 0,
		TABLE_VLAN,
		TABLE_DYNAMIC_MAC,
		TABLE_MIB
	};

//dummy = SW_R16(sw, reg);	\

#endif //_KSZ8567_H
