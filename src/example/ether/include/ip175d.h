//=========== ip175d.h =========================
#ifndef IP175D_H
#define IP175D_H

#define STP_NUM_PORTS 6

//4+1 ports

#define PORT_MII		5 //MAC5

/* Valid for PHY 0-4 */

#define MII_BMCR 				0       //Basic mode control reg.
#define IP175D_REG_ST 			1		/* Status Register */
#define IP175D_REG_ST_ANEG		(1<<5)	/* Autonegotiation: 1=Complete, 0=In Progress */
#define IP175D_REG_ST_LINK		(1<<2)	/* Link Status: 1=Link Pass, 0=Link Fail */

#define IP175D_REG_SS			18		/* Special Status Register */
#define IP175D_REG_SS_SPD		(1<<11)	/* Speed: 1=100Mbps, 0=10Mbps */
#define IP175D_REG_SS_DPX		(1<<10) /* Duplex: 1=FDX, 0=HDX */

//========== Global Registers@PHYADDR of 20 ================
#define IP175D_GLOBAL_PHY_ADDR 	20
#define IP175D_PHY_SWCON		20		/* Switch Control Registers */
#define IP175D_REG_CHIPID		0		/* Chip Identification (def: 0x175D) */
#define IP175D_REG_SOFT_RST		2		/* Software Reset Register */

#define IP175D_PHY_VLCON		22		/* VLAN Group Control Registers */
#define IP175D_PHY_VLCON2		23
#define IP175D_REG_VLCLASS		0		/* VLAN Classification Register */
#define IP175D_REG_VLCLASS_P0	(1<<6)	/* Per-port classification */
#define IP175D_REG_VLCLASS_P1	(1<<7)	/*     (tag-based only)    */
#define IP175D_REG_VLCLASS_P2	(1<<8)	/* 0*: Use VID if tagged,   */
#define IP175D_REG_VLCLASS_P3	(1<<9)	/*    or PVID if untagged  */
#define IP175D_REG_VLCLASS_P4	(1<<10)	/*                         */
#define IP175D_REG_VLCLASS_P5	(1<<11)	/* 1: Use only PVID        */

#define IP175D_REG_VLCLASS_M0	(1<<0)	/* Per-port mode setting */
#define IP175D_REG_VLCLASS_M1	(1<<1)	/*                       */
#define IP175D_REG_VLCLASS_M2	(1<<2)  /* 0*: Port based VLAN    */
#define IP175D_REG_VLCLASS_M3	(1<<3)	/*                       */
#define IP175D_REG_VLCLASS_M4	(1<<4)  /* 1: Tag based VLAN     */
#define IP175D_REG_VLCLASS_M5	(1<<5)  /*                       */

#define IP175D_REG_VLINGR		1		/* VLAN Ingress Rule */
#define IP175D_REG_VLEGR		2		/* VLAN Egress Rule */
#define IP175D_REG_VLEGR_KEEP0	(1<<0)	/* Keep VLAN Tag Header */
#define IP175D_REG_VLEGR_KEEP1	(1<<1)	/* (per-port)			*/
#define IP175D_REG_VLEGR_KEEP2	(1<<2)	/* 0*: Disabled			*/
#define IP175D_REG_VLEGR_KEEP3	(1<<3)	/* 1: Keep VLAN tag header */
#define IP175D_REG_VLEGR_KEEP4	(1<<4)	/*    from frame.		*/
#define IP175D_REG_VLEGR_KEEP5	(1<<5)	/*						*/

/* VLAN Identifiers (bits [11:0])       *
 * Runs from 14 (VLAN_0) to 29 (VLAN_F) */
#define IP175D_REG_VID0			14
#define IP175D_REG_VID1			15
#define IP175D_REG_VID2			16
#define IP175D_REG_VID3			17
#define IP175D_REG_VID4			18

/* MII Force Mode */
#define IP175D_PHY_FORCE		20
#define IP175D_REG_FORCE		4
#define IP175D_MAC5_FORCE_100	(1<<15)
#define IP175D_MAC5_FORCE_FULL	(1<<13)
#define IP175D_MAC4_FORCE_100	(1<<14)
#define IP175D_MAC4_FORCE_FULL	(1<<12)

/* MII Status Report */
#define IP175D_PHY_MIISTAT		21
#define IP175D_REG_MIISTAT		0
#define IP175D_MII0_FULL		(1<<7)
#define IP175D_MII0_SPEED10		(1<<6)
#define IP175D_MII0_FLOW		(1<<5)

#endif