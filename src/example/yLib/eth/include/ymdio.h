#ifndef __YMDIO_H__
#define __YMDIO_H__
//========================================
#include "ethopts.h"
#include "yInc.h"
#if ((CMDLINE_USE_FOR == CMDLINE_MDIO) || (CMDLINE_USE_FOR == CMDLINE_MDIO407_107) || (CMDLINE_USE_FOR == CMDLINE_SJA1105))

/*//Platform
#define EDU 0
#define BROADCOM 1
#define RTL9K 2
//#define IP101G 4
//#define RTL8201 5
//#define KSZ9021RN 6
#define LAN9355SWITCH_BR_BR 3
#define LAN9355SWITCH_RTL_BR 4
#define LAN9355SWITCH_RTL_RTL 5
#define RTL9047SWITCH 6
#define GENERALPHY 7

//#define MDIO_FOR MDIO_FOR_IP101G //EDU//RTL9K
//#define MDIO_FOR MDIO_FOR_LAN9355SWITCH_RTL_BR //BROADCOM //RTL9K //RTL9047SWITCH //RTL9K
//#define MDIO_FOR MDIO_FOR_RTL9K
//#define MDIO_FOR MDIO_FOR_RTL8201
#define MDIO_FOR MDIO_FOR_GENERALPHY
//========================================
*/

#define PHY_ACCESS_METHOD SMIIF



//for Broadcom PHY
struct _CarPhyStatus{ //_BRmcConfig{
	unsigned char LinkStatus;
	unsigned char fForced_or_AN; //1-0
	unsigned char fMaster_or_Slave;

	unsigned short br_reg1_status; //[2]=1 linkup
	unsigned short br_regf_extstatus; //[9]=local rx ok;[8]=retmoe rx ok
	unsigned short br_reg11_phyextstatus; //[13]=mdix; [11]=retmoe rx ok; [10]=local rx ok;[9]=descambled locked ;[8]=LinkPassed
	unsigned short br_reg1a_intstatus;//[14]=pair swap;[5]=remote rx status changed;[4]=local rx status changed;[2]=link speed changed;[1]=link status changed;[0]=crc error
	unsigned short br_reg1c08_ledstatus; //[8]=1master;[7]=0:FDX; [6]=1=No INT; [4:3]=01=100Mbps,[0]=1=Poor Quality

	unsigned char rtl_fHost_or_AN; //1-0
	unsigned short rtl_localRcv_status; //0x0A
	unsigned short rtl_remoteRcv_status; //0x0A
	unsigned short rtl_PHYsub_flag; //0x10
};

//for RTL PHY
struct _PhyStatus{
	unsigned char LinkStatus;       //0x01
	unsigned char fHost_or_AN; //1-0
	unsigned char fMaster_or_Slave; //0x09

	unsigned short localRcv_status; //0x0A
	unsigned short remoteRcv_status; //0x0A
	unsigned short PHYsub_flag; //0x10
	//

	//unsigned short regf_extstatus; //[9]=local rx ok;[8]=retmoe rx ok
	//unsigned short reg11_phyextstatus; //[13]=mdix; [11]=retmoe rx ok; [10]=local rx ok;[9]=descambled locked ;[8]=LinkPassed
	//unsigned short reg1a_intstatus;//[14]=pair swap;[5]=remote rx status changed;[4]=local rx status changed;[2]=link speed changed;[1]=link status changed;[0]=crc error
	//unsigned short reg1c08_ledstatus; //[8]=1master;[7]=0:FDX; [6]=1=No INT; [4:3]=01=100Mbps,[0]=1=Poor Quality
};

struct GenealPhyREG{ //== IP101GREG,RTL8201REG,RTL9KREG
	unsigned short regaddr;
	unsigned short page; //0= basic ; 1=extended or page
	char *regName;
	char *detail;
};

struct RTL9KSPECIALREG{
	unsigned short regaddr;
	char *regName;
	char *detail;
};


struct GenealPhyREGS {
	struct GenealPhyREG *pBasicReg;
	struct GenealPhyREG *pSpecificReg;
	struct GenealPhyREG *pRTL9047Reg;
};


struct GeneralPhyBOARD{
	unsigned char platform;
	unsigned char use_oled;
	unsigned char phyaddr_set_done;
	struct _PhyStatus PhyStatus;
	struct GenealPhyREGS GeneralPhyRegs;
	unsigned char phyaddr;
	char *phyname;
	unsigned long phyid32;
	unsigned short  clause45;
	unsigned char f_preamblesuppressed;
};

struct PhyName{
	unsigned short prodId;
	char namestr[10];
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
#if (USE_DISPLAY == USE_DISPLAY_OLED)
extern void OzOLED_init();
extern void OzOLED_sendCommand(unsigned char command);
extern void OzOLED_sendData(unsigned char Data);
extern void OzOLED_printString(const char *String, unsigned char X, unsigned char Y, unsigned char numChar);
#endif //USE_OLED
#endif //(CMDLINE_USE_FOR == CMDLINE_MDIO)
#endif //__YMDIO_H__
