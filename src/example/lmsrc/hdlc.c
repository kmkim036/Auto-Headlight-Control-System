#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "yLib/yInc.h"

//Test on V5 Board

/* Operation Modes : Flags and CRC are automatically generated.
 * 	Auto         : TX=Data Only.           RX= Address recog. WindowSize=1
 * 	Non-AutoMode : Address recog. any WindowSize
 * 	TransparentMode1 : High Address recog. any WindowSize
 * 	TransparentMode0 : No Address recog. any WindowSize
 * 	ExtendedTransparentMode1 : Fully transparent
 * 	ExtendedTransparentMode0 : Fully transparent
 */
//HDLC Registers
#define XFIFOA  0x00 //~0x1F -- Transmit Fifo Reg.(write) // Receive Fifo Reg(Read)
#define XFIFOB  0x40 //~0x5F
//#define XBYTE ((volatile unsigned char *)0)
#define MASKA 0x20 //Wr
#define MASKB 0x60 //Wr
#define ISTA 0x20 //Rd Interrupt Status Register
#define ISTB 0x60 //Rd
#define CMDRA 0x21 //Wr
#define CMDRB 0x61 //Wr
#define STARA 0x21 //Rd Status Reg
#define STARB 0x61 //Rd Status Reg
#define MODEA 0x22 //R/W
#define MODEB 0x62 //R/W
#define TIMRA 0x23 //Timing R/W
#define TIMRB 0x63 //Timing R/W
#define XAD1A 0x24 //Wr Transmit Address Reg.1
#define XAD1B 0x64 //Wr Transmit Address Reg.1
#define EXIRA 0x24 //Rd Extended Interrupt Reg
#define EXIRB 0x64 //Rd Extended Interrupt Reg
#define XAD2A 0x25 //Wr Transmit Address Reg.2
#define XAD2B 0x65 //Wr Transmit Address Reg.2
#define RBCLA 0x25 //Rd Rx Byte Counter LOW
#define RBCLB 0x65 //Rd Rx Byte Counter LOW
#define RAH1A 0x26 //Wr
#define RAH2A 0x27 //Wr
#define RSTAA 0x27 //Rd Rx Status Reg
#define RSTAB 0x67 //Rd Rx Status Reg
#define RAL1A 0x28 //Rd/Wr Rx Addr Low 1
#define RAL1B 0x68 //Rd/Wr Rx Addr Low 1
#define RAL2A 0x29 //Wr Rx Addrr Low 2
#define RHCRA 0x29 //Rd Rx HDLC Control
#define XBCLA 0x2a //Wr Tx Byte Count Low
#define BGRA  0x2b //Wr Baud rate Gen
#define CCR2A 0x2c //R/W Channel Config Reg 2
#define XBCHA 0x2d //Wr Tx Byte Count High
#define RBCHA 0x2d //Rd Rx Byte Count High
#define RLCRA 0x2e //Wr Rx Frame Len Check
#define VSTRA 0x2e //Rd Carrier Detect Reg
#define CCR1A 0x2f //R/W Channel Config Reg 1 -- Clock Mode

//#define CEC  (STAR&0x04)>>2                     //CEC＝1，CMDR꼇옵畇；CEC＝0，CMDR옵畇
/*
#define RME  (ISTA&0x80)>>7                     //斤口쌈澗써監
#define RPF  (ISTA&0x40)>>6                     //32Bytes쌈肝넥찮
#define RSC  (ISTA&0x20)>>5                     //菱땡친駕苟뚤렘돨榴檄
#define XPR  (ISTA&0x10)>>4                     //랙箇넥攣왕
#define TIN  (ISTA&0x08)>>3                     //땍珂포櫓뙤
#define XMR  (EXIR&0x80)>>7                     //랙箇矜狼路릿
#define XDU  (EXIR&0x40)>>6                     //랙箇櫓槁櫓岺
#define PCE  (EXIR&0x20)>>5                     //葵累댄轎
#define RFO  (EXIR&0x10)>>4                     //쌈澗鑒앴轟랬닸흙RFIFO芚냥淚놔
#define CSC  (EXIR&0x08)>>3                     //CTS榴檄맣긴
#define RFS  (EXIR&0x04)>>2                     //쌈澗돕煉庫
*/
#define XFW  ((readRegHdlc(STARA)&0x40)  >> 6)              //TX FIFO Write Enable
#define CEC ((readRegHdlc(STARA) &0x04) >> 2) 				//CEC＝1，CMDR Executing；CEC＝0，CMDR Done.
//---------------------------------------------------------------------
unsigned char flag_txd;
unsigned char pre_txd[33];
u8 g_TxBuf[20][33];
u8 g_count;
u8 g_front,g_rear;

//DATA Array for Test Start====================================================
unsigned char data_array01[32]={0x02,0x93};
unsigned char data_array02[32]={0x02,0x00,0xff,0xee};
unsigned char data_array03[32]={0x03,0x00,0x66};
unsigned char data_array04[32]={0x04,0x00,0xff};
unsigned char data_array05[32]={
	         0x05,0x00,0xff,0xff,0xee,0xff,0xee,0xff,0xee,0xff,0xee,0xff,0xee,0xff,0xee
	    };

//-----------------------------------------------------------------------------
void InitQ();
unsigned char IsQEmpty();
unsigned char IsQFull();
void EnQueue(unsigned char txd_data[32],unsigned char num_enQ);    //흙뚠
void DeQ();        //놔뚠
//-----------------------------------------------------------------------------
//PT7A6525L --
/* AD7: PF0 -Pin7                         //AD7: PD4 -Pin7  v3
 * AD6: PD5 - Pin8
 * AD5: PD0 --Pin9
 * AD4: PD1 -- Pin10
 * AD3:	PA2 -- Pin11
 * AD2:	PA5 --Pin12
 * AD1:	PA4 -- Pin13
 * AD0:	PA3 --Pin14
 	 	 	 	 	 	 	 	 	 	  // nCS2153: PA6 -- NC
 * nCSHDLC: PG1  --Pin15                  //PD6(*) --V3
 * nRD:PE0 -- Pin16
 * nWR:PB1 -- Pin20                       //PB2 -- Pin20
 * ALE:PG1 --Pin21                        //PE1
 * nINTHDLC:PB0 -Pin22                    //nINT2:PF1 --Pin22
                                           //* nINT1:PE2
                                          // * nINTHDLC:PG1(*) PD6(*)을 PG1으로 수정해야 함. (20번 핀에서 21번 핀으로)
 * */

#define nCsHDLC0 (GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, ~GPIO_PIN_1)) //nCSHdlc=0);
#define nCsHDLC1 (GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1)) //nCSHdlc=1);
#define nRdHDLC0 (GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, ~GPIO_PIN_0)) //nRd=0)
#define nRdHDLC1 (GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0)) //nRd=1)
#define nWrHDLC0 (GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, ~GPIO_PIN_1)) //nWR=0)
#define nWrHDLC1 (GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1)) //nWR=1)
#define AleHDLC0 (GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, ~GPIO_PIN_1)) //ALE=0
#define AleHDLC1 (GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1)) //ALE=0

void writeByteHdlc(unsigned char x, unsigned char nWR){
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3);

	while(GPIO_DIR_MODE_OUT != GPIODirModeGet(GPIO_PORTA_BASE, 3)); //wait for output mode
	if(nWR)
		nWrHDLC1; //nWR=1
	else
		nWrHDLC0; //nWR=0
	somedelay(1);

	if(x & 0x01) 	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
	else 			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);

	if(x & 0x02) 	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
	else 			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, ~GPIO_PIN_4);

	if(x & 0x04) 	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
	else 			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, ~GPIO_PIN_5);

	if(x & 0x08) 	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
	else 			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, ~GPIO_PIN_2);

	if(x & 0x10) 	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
	else 			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, ~GPIO_PIN_1);

	if(x & 0x20) 	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
	else 			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, ~GPIO_PIN_0);

	if(x & 0x40) 	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_PIN_5);
	else 			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, ~GPIO_PIN_5);

	if(x & 0x80) 	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
	else 			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, ~GPIO_PIN_0);

	somedelay(1);
	nWrHDLC1; //nWR=1
	somedelay(1);
}
unsigned char readByteHdlc(void){
	unsigned char x=0;
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_5 );
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_2);
	while(GPIO_DIR_MODE_IN != GPIODirModeGet(GPIO_PORTA_BASE, 2));//GPIO_PIN_4)); //NOTE!

	nRdHDLC0;//nRD=0
	somedelay(1);

	if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3)) x=1;
	if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4)) x = x + 2;
	if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5)) x = x + 4;
	if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2)) x = x + 8;
	if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1)) x = x + 16;
	if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0)) x = x + 32;
	if(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_5)) x = x + 64;
	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)) x = x + 128;

	somedelay(1);
	nRdHDLC1; //nRD=1
	somedelay(1);
	return x;
}
void writeAddrHdlc(unsigned char addr){
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3);
	while(GPIO_DIR_MODE_OUT != GPIODirModeGet(GPIO_PORTA_BASE, 3));//GPIO_PIN_4));

	//nWR=1;nRD=1,nCS=0
	nRdHDLC1;//nRD=1
	nWrHDLC1; //nWR=1
	AleHDLC1; //ALE=1
	writeByteHdlc(addr,1);
	somedelay(1); //10nsec
	AleHDLC0; //ALE=0
	somedelay(1); //10nsec
}

void writeRegHdlc(unsigned char addr, unsigned char reg){
	nCsHDLC0; //nCSHdlc
	writeAddrHdlc(addr);
	writeByteHdlc(reg,0);
	nCsHDLC1; //nCSHdlc

}

unsigned char readRegHdlc(unsigned char addr){
	unsigned char x = 0;
	nCsHDLC0;
	writeAddrHdlc(addr);
	x = readByteHdlc();
	nCsHDLC1;
	return x;
}

//=========================================
void InitQ(){
	g_front = 0; g_rear=0; g_count=0;
}
unsigned char IsQEmpty(){
	if(g_count !=0) return 0;
	else return 1;
}
unsigned char IsQFull(){
	if(g_count ==20) return 1;
	else return 0;
}
void EnQueue(u8 txd[32], u8 numEnQ){
	u8 i=0;
	for(i=0;i<numEnQ;i++)	g_TxBuf[g_rear][i] = txd[i];
	g_TxBuf[g_rear][32] = numEnQ;

	if(g_rear >=19) {
			g_rear=0;
			g_count = 20 + g_rear - g_front;
	}else{
		g_rear++;
		g_count = g_rear - g_front;
	}
	UARTprintf("EnQueue> g_count=%d\r\n",g_count);
}

void DeQ(){
	unsigned char i_deQ=0;
	unsigned char num_deQ=0;
    num_deQ=g_TxBuf[g_front][32];
    for(i_deQ=0;i_deQ<num_deQ;i_deQ++)
    {
         pre_txd[i_deQ]=g_TxBuf[g_front][i_deQ];
    }
    pre_txd[32]=g_TxBuf[g_front][32];
		        //Plus------------------------------------------------
	        //for(i_deQ=0;i_deQ<33;i_deQ++)
	        //{
	        //    g_TxBuf[front][i_deQ]=0;
	        //}
        //----------------------------------------------------
	     //count--;
	if(g_front>=19)  {
	       g_front=0;
	       g_count=g_rear-g_front;
	 }     else  {
	                g_front++;
	                g_count=g_rear-g_front;
     }
	UARTprintf("DeQueue> g_count=%d\r\n",g_count);
}
////------------------------------------------------------------------
void setMode_Auto_Clk_Ifg_Nrzi(u8 ClockMode, u8 ifgFillEn, u8 nrziEn)
{
	u8 x=0x00;
	//Set AutoMode
	writeRegHdlc(MODEA,0x18);   //00-01_1000 (00-Auto, 10=TransparentMode; 0(addressMode=0=8bit); 1(TimerMode=1=InternalMode);RAC(ReceveActive=1),00,0(TestMode=0 or 1 (Loopback))
	writeRegHdlc(MODEB,0x18);   //00-01_1000

	//Channel Config Reg. (0x2f/6f)
	x=0x90;   //1xx1-immm
	x = x | ClockMode;
	x = x | (ifgFillEn << 3);
	if(nrziEn)
		x = x | 0x40;
	writeRegHdlc(CCR1A,x);//0x99);    //.7=PowerUp(1); .6,5=00(NRZ enc); .4=PushPullOutput(1); .3=IFG Fill(1=0x7e inserted); .2.1.0=ClockMode(1)

	writeRegHdlc(MASKA,0x27);  //0010_0111	--
	writeRegHdlc(MASKB,0xfb);   // 1111_1011
	writeRegHdlc(TIMRA,0x7f);   //InternalTimerMode의 경우, (Counter(3 bit) + Value(5 bits) = N2 Retry Counter +
	writeRegHdlc(XAD1A,0x00);   //TxAddress Byte 1 (Command)
	writeRegHdlc(XAD2A,0x03);   //TxAddress Byte 2 (Response)
	writeRegHdlc(RAH1A,0x02);   //RxAddress Byte 1 IMPORMANT!!!
	writeRegHdlc(RAH2A,0x00);   //RxAddress Byte 2 IMPORTANT!!!
	writeRegHdlc(RAL1A,0x01);   //RxAddress Byte low 1 (First Indivisual Address - Command Address)
	writeRegHdlc(RAL2A,0x01);   //RxAddress Byte low 1 (2nd Indivisual Address - Response Address)
	writeRegHdlc(CCR2A,0x08);   //Channel Conf Reg 2.3 (TxClk Output D3=1)
	writeRegHdlc(XBCHA,0x00);   //TxByteCountHigh(0x00)
    while(CEC);
    writeRegHdlc(CMDRA,0x41);      //0100-0001 : .6=Reset HDLC Rcv, .0=Transmit Reset
}

void setMode_Transparent_Clk_Ifg_Nrzi(u8 ClockMode, u8 ifgFillEn, u8 nrziEn)
{
	u8 x=0x00;
	//Set Transparent Mode
	writeRegHdlc(MODEA,0x98);   //10-01_1000 (10=TransparentMode; 0(addressMode=0=8bit); 1(TimerMode=1=InternalMode);RAC(ReceveActive=1),00,0(TestMode=1 (Loopback))
	writeRegHdlc(MODEB,0x98);   //10-01_1000

	//Channel Config Reg. (0x2f/6f)
	x=0x90;   //1xx1-immm
	x = x | ClockMode;
	x = x | (ifgFillEn << 3);
	if(nrziEn)
		x = x | 0x40;
	writeRegHdlc(CCR1A,x);//0x99);    //.7=PowerUp(1); .6,5=00(NRZ enc); .4=PushPullOutput(1); .3=IFG Fill(1=0x7e inserted); .2.1.0=ClockMode(1)
	    					//1001_1001
	writeRegHdlc(MASKA, 0x27);  //0010_0111
	writeRegHdlc(MASKB,0xfb);   // 1111_1011
	writeRegHdlc(TIMRA,0x7f);   //
	writeRegHdlc(XAD1A,0x00);
	writeRegHdlc(XAD2A,0x03);
	writeRegHdlc(RAH1A,0x02);          //IMPORMANT!!!
	writeRegHdlc(RAH2A,0x00);          //IMPORTANT!!!
	writeRegHdlc(RAL1A,0x01);
	writeRegHdlc(RAL2A,0x01);
	writeRegHdlc(CCR2A,0x08);   //Channel Conf Reg 2.3 (TxClk Output D3=1)
	writeRegHdlc(XBCHA,0x00);
    while(CEC);
    writeRegHdlc(CMDRA,0x41);      //0100-0001 : .6=Reset HDLC Rcv, .0=Transmit Reset
}

void transmit(void){
    unsigned char it=0;
    unsigned char xfw,cmdr;
    flag_txd=1;

	    if(g_count!=0)  {
	            DeQ();
	            while(CEC);

	            do {
	                xfw=XFW;
	            }while(xfw!=1);
	            UARTprintf("Tx..\r\n");
	            for(it=0;it<pre_txd[32];it++)
	            //for(it=0;it<10;it++)
	            	writeRegHdlc(XFIFOA+it, pre_txd[it]);
	            while(CEC==1);//
	            cmdr = readRegHdlc(CMDRA);
	            writeRegHdlc(CMDRA,cmdr | 0x0a);
	    }

	    delayms(1);
	    flag_txd=0;

}


void  PT7A6525LLoop(){
	int i,j, im;
	unsigned char id = 0x00;

	GuiInit("PT7A6525L HDLC Test");

	//PT7A6525L --
	/* AD7: PF0 -Pin7                         //AD7: PD4 -Pin7  v3
	 * AD6: PD5 - Pin8
	 * AD5: PD0 --Pin9
	 * AD4: PD1 -- Pin10
	 * AD3:	PA2 -- Pin11
	 * AD2:	PA5 --Pin12
	 * AD1:	PA4 -- Pin13
	 * AD0:	PA3 --Pin14
	 	 	 	 	 	 	 	 	 	 	  // nCS2153: PA6 -- NC
	 * nCSHDLC: PG1  --Pin15                  //PD6(*) --V3
	 * nRD:PE0 -- Pin16
	 * nWR:PB1 -- Pin20                       //PB2 -- Pin20
	 * ALE:PG1 --Pin21                        //PE1
	 * nINTHDLC:PB0 -Pin22                    // * nINTHDLC:PG1(*) PD6(*)을 PG1으로 수정해야 함. (20번 핀에서 21번 핀으로)
	 * */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

	//GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);//nCS2153
	GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_1);//nCSHDLC
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0); //nRD
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1 );//nWR
	GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_1); //ALE
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);   //nINTHDLC

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); //nCSHdlc = 1
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0); //nRD=1
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1); //nWR=1
	GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PIN_1); //ALE=0

	//Init
	somedelay(100);

	//setMode_Auto_Clk_Ifg_Nrzi(4,0,0);
	setMode_Transparent_Clk_Ifg_Nrzi(4,0,0); //(4,0,1);

	UARTprintf("Mode Set Done.\r\n");
	InitQ();

	for(i=0x00;i<0x33;i++){
		id = readRegHdlc(i);
		UARTprintf("PT7A6525L HDLC> Reg[0x%02x]=0x%02x\r\n",i,id);
		delayms(100);
	}

	//Make 1st Frame and Enqueue it
    pre_txd[0]=0x02; //0000-0010 1001-0011
    pre_txd[1]=0x93;
    EnQueue(pre_txd,2);
	//Make 2nd Frame and Enqueue it
    pre_txd[0]=0x02;
    pre_txd[1]=0x00;
    for(im=2;im<17;im++)
    	pre_txd[im]=0x99;
	EnQueue(pre_txd,17);
	//Make 3rd Frame and Enqueue it
	for(im=0;im<27;im++)
		pre_txd[im]=0xaa;
    EnQueue(pre_txd,27);

    while(flag_txd);
	while(1){
		UARTprintf("Begin Tx.\r\n");
		transmit();
		delayms(10000);
	}


}
/*
void InitPT7A6525L(void)
{
	writeReg(

124.
125.	    //箇흙셕鑒렘駕
126.	    XBYTE[0x3300]=0x36; //뒈囹11_0000_0000    鑒앴0011_0110
127.	    XBYTE[0x3000]=0x02; //뒈囹00_0000_0000    鑒앴0000_0010
128.	    XBYTE[0x3000]=0x00; //뒈囹00_0000_0000    鑒앴0000_0000
129.
130.	    XBYTE[0x3300]=0x76; //                                  鑒앴0111_0110
131.	    XBYTE[0x3100]=0x02; //뒈囹01_0000_0000    鑒앴0000_0010
132.	    XBYTE[0x3100]=0x00; //뒈囹01_0000_0000    鑒앴0000_0000
133.
134.	    XBYTE[0x3300]=0xB6; //                                  鑒앴1011_0110
135.	    XBYTE[0x3200]=0x02; //뒈囹10_0000_0000    鑒앴0000_0010
136.	    XBYTE[0x3200]=0x00; //뒈囹10_0000_0000    鑒앴0000_0000
137.
138.	    //폘땡롸틉셕鑒포
139.	    PORTD|=0x1c;                //0001_1100
140.
141.	    //혤句튬朞
142.	    PORTD|=0x20;                //0010_0000
143.	    return;
144.	}

main()	{
	  int im=0;
//	  resethdlc();
	  cshdlc();
	  automode();
	  InitQueue();
	  timer0_init();
	  int_open();

	    txd[0]=0x02;
229.	    txd[1]=0x93;
230.	    EnQueue(txd,2);
231.
232.	    txd[0]=0x02;
233.	    txd[1]=0x00;
234.	    for(im=2;im<17;im++)
235.	        txd[im]=0x99;
236.	    EnQueue(txd,17);
237.	    for(im=0;im<27;im++)
238.	        txd[im]=0xaa;
239.	    EnQueue(txd,27);
240.
241.	    delayms(1000);
242.	    EnQueue(txd,27);
243.
244.	    while(flag_txd);
245.	    transmit();
246.	    while(flag_txd);
247.	    transmit();
248.	    while(flag_txd);
249.	    transmit();
250.
251.	}


#pragma interrupt_handler timer0_txd_srv

void timer0_txd_srv(void)
13.	{
14.	    unsigned char num_tmr=0;
15.	    TCNT0=0xb5; //셕鑒令路陋
16.
17.	    transmit();
18.
19.	    if(count<(20-5))
20.	        {
21.	            EnQueue(data_array01,2);
22.	            EnQueue(data_array02,4);
23.	            EnQueue(data_array03,3);
24.	            EnQueue(data_array04,3);
25.	            EnQueue(data_array05,15);
26.	        }
27.
28.
29.	}
30.	//------------------------------------------------------------------
31.	void timer0_init(void)
32.	{
33.	    TIMSK=0x01;
34.	    TCCR0=0x00;
35.	    TCNT0=0xb5; //20ms돨셕鑒令
36.
37.	    TCCR0=0x07;
*/
