#ifndef __YMDIO_H__
#define __YMDIO_H__
//========================================
//Platform
#define PHYIS_GENERAL   0 //Support only the first 16 standard registers
#define PHYIS_IP101G    1
#define PHYIS_RTL8201   2
#define PHYIS_KSZ9021RN 3

#define PHYIS_BROADCOM  20
#define PHYIS_RTL9K     21

#define PHYIS_LAN9355SWITCH_BR_BR 30
#define PHYIS_LAN9355SWITCH_RTL_BR 31
#define PHYIS_LAN9355SWITCH_RTL_RTL 32
#define PHYIS_RTL9047SWITCH 33

//#define PLATFORM IP101G //EDU//RTL9K
//#define PLATFORM LAN9355SWITCH_RTL_BR //BROADCOM //RTL9K //RTL9047SWITCH //RTL9K
//#define PLATFORM RTL9K
//#define PLATFORM RTL8201
#define PHYIS   PHYIS_GENERAL

//=== PHY Access Method for MDIO ============
#define SMIIF 1   //Default
#define I2CIF 2
#define SPIIF 3

#define PHY_ACCESS_METHOD SMIIF



//============== General ==========================
struct _PhyStatus{
	u8 LinkStatus;       //0x01
	u8 fMaster_or_Slave; //0x09
	u16 localRcv_status; //0x0A
	u16 remoteRcv_status; //0x0A

	u8 fAN_or_Forced; //1-0//u8 fHost_or_AN; //1-0
	u16 PHYsub_flag; //0x10

	//u16 regf_extstatus; //[9]=local rx ok;[8]=retmoe rx ok
	//u16 reg11_phyextstatus; //[13]=mdix; [11]=retmoe rx ok; [10]=local rx ok;[9]=descambled locked ;[8]=LinkPassed
	//u16 reg1a_intstatus;//[14]=pair swap;[5]=remote rx status changed;[4]=local rx status changed;[2]=link speed changed;[1]=link status changed;[0]=crc error
	//u16 reg1c08_ledstatus; //[8]=1master;[7]=0:FDX; [6]=1=No INT; [4:3]=01=100Mbps,[0]=1=Poor Quality
};

struct _PhyREG{ //== IP101GREG,RTL8201REG,RTL9KREG
	unsigned short 	regaddr;
	unsigned short 	page; //0= basic ; 1=extended or page
	char 			*regName;
	char 			*detail;
};

struct _PhyREGS {
	struct _PhyREG *pBasicReg;
	struct _PhyREG *pSpecificReg;
	struct _PhyREG *pRTL9047Reg;
};

struct _PhyMdioModule{
	u8 phyaddr;
	char *phyname;
	u8 platform;
	u8 use_oled;
	u8 phyaddr_set_done;
	u32 phyid32;
	u16 clause45;
	u8 f_preamblesuppressed;

	struct _PhyREGS PhyRegs;
	struct _PhyStatus PhyStatus;
};

enum MMD{
	DEV_NULL=0,
	DEV_PMAPMD=1,
	DEV_WIS,
	DEV_PCS,
	DEV_PHY_XS,
	DEV_DTE_XS,
	DEV_TC, //6
	DEV_AN,//7
	DEV_SEP_PMA1,
	DEV_SEP_PMA2,
	DEV_SEP_PMA3,
	DEV_SEP_PMA4,
	DEV_29 = 29,
	DEV_VENDORSPECIFIC1,
	DEV_VENDORSPECIFIC2
};
//==========================================
//for Broadcom PHY
struct _CarPhyStatus{ //_BRmcConfig{
	u8 LinkStatus;
	u8 fForced_or_AN; //1-0
	u8 fMaster_or_Slave;

	u16 br_reg1_status; //[2]=1 linkup
	u16 br_regf_extstatus; //[9]=local rx ok;[8]=retmoe rx ok
	u16 br_reg11_phyextstatus; //[13]=mdix; [11]=retmoe rx ok; [10]=local rx ok;[9]=descambled locked ;[8]=LinkPassed
	u16 br_reg1a_intstatus;//[14]=pair swap;[5]=remote rx status changed;[4]=local rx status changed;[2]=link speed changed;[1]=link status changed;[0]=crc error
	u16 br_reg1c08_ledstatus; //[8]=1master;[7]=0:FDX; [6]=1=No INT; [4:3]=01=100Mbps,[0]=1=Poor Quality

	u8 rtl_fHost_or_AN; //1-0
	u16 rtl_localRcv_status; //0x0A
	u16 rtl_remoteRcv_status; //0x0A
	u16 rtl_PHYsub_flag; //0x10
};

//for RTL PHY ============================


struct RTL9KSPECIALREG{
	u16 regaddr;
	char *regName;
	char *detail;
};





#endif
