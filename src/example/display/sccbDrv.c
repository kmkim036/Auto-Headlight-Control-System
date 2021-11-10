/*
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
//#include "yInc.h"
*/
#include "yInc.h"

#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"

#if(PROCESSOR == STM32F103C8)
#else
//================== WE SUPPORT ONLY 8 bit I2C Address ========================================

// REF : https://raw.githubusercontent.com/geoffreymbrown/STM32-Template/master/Library/i2c.c

/**
 *  Names of events used in stdperipheral library
 *
 *      I2C_EVENT_MASTER_MODE_SELECT                          : EV5
 *      I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED            : EV6
 *      I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED               : EV6
 *      I2C_EVENT_MASTER_BYTE_RECEIVED                        : EV7
 *      I2C_EVENT_MASTER_BYTE_TRANSMITTING                    : EV8
 *      I2C_EVENT_MASTER_BYTE_TRANSMITTED                     : EV8_2
 *
 **/
#define Timed(x) Timeout = 0x00FF; while (x) { if (Timeout-- == 0){ goto errReturn;}}
#define Y_I2C_CLOCKFREQ 100000 //100Kbps

void sccb_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
int sccb_StartAndAddr(uint8_t address8bit, uint8_t direction);
int sccb_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
unsigned char sccb_ReceiveBurst(unsigned char SlaveAddress, unsigned char *buf, unsigned char nbyte);

extern I2C_TypeDef *gI2Cx;

#define OV7670_REG_NUM 40
#define OV7670_ADDR_7BIT 0x21
#define OV7670_ADDR_8BIT_WRITE 0x42
#define OV7670_ADDR_8BIT_READ 0x43 //0100-0011
//The 7 bit SCCB/I2C address is 0x21. 0x42=Write 8 bit address

const uint8_t OV7670_reg[OV7670_REG_NUM][2] = {
		{ 0x12, 0x80 },
		// Image format
		{ 0x12, 0x8 },		// 0x14 = QVGA size, RGB mode; 0x8 = QCIF, YUV, 0xc = QCIF (RGB)
		{ 0xc, 0x8 }, //
		{ 0x11, 0b1000000 }, //

		{ 0xb0, 0x84 },		//Color mode (Not documented??)

		// Hardware window
		{ 0x11, 0x01 },		//PCLK settings, 15fps
		{ 0x32, 0x80 },		//HREF
		{ 0x17, 0x17 },		//HSTART
		{ 0x18, 0x05 },		//HSTOP
		{ 0x03, 0x0a },		//VREF
		{ 0x19, 0x02 },		//VSTART
		{ 0x1a, 0x7a },		//VSTOP

		// Scalling numbers
		{ 0x70, 0x3a },		//X_SCALING
		{ 0x71, 0x35 },		//Y_SCALING
		{ 0x72, 0x11 },		//DCW_SCALING
		{ 0x73, 0xf0 },		//PCLK_DIV_SCALING
		{ 0xa2, 0x02 },		//PCLK_DELAY_SCALING

		// Matrix coefficients
		{ 0x4f, 0x80 }, //
		{ 0x50, 0x80 }, //
		{ 0x51, 0x00 }, //
		{ 0x52, 0x22 }, //
		{ 0x53, 0x5e }, //
		{ 0x54, 0x80 }, //
		{ 0x58, 0x9e },

		// Gamma curve values
		{ 0x7a, 0x20 }, //
		{ 0x7b, 0x10 }, //
		{ 0x7c, 0x1e }, //
		{ 0x7d, 0x35 }, //
		{ 0x7e, 0x5a }, //
		{ 0x7f, 0x69 }, //
		{ 0x80, 0x76 }, //
		{ 0x81, 0x80 }, //
		{ 0x82, 0x88 }, //
		{ 0x83, 0x8f }, //
		{ 0x84, 0x96 }, //
		{ 0x85, 0xa3 }, //
		{ 0x86, 0xaf }, //
		{ 0x87, 0xc4 }, //
		{ 0x88, 0xd7 }, //
		{ 0x89, 0xe8 },

		// AGC and AEC parameters
		{ 0xa5, 0x05 }, //
		{ 0xab, 0x07 }, //
		{ 0x24, 0x95 }, //
		{ 0x25, 0x33 }, //
		{ 0x26, 0xe3 }, //
		{ 0x9f, 0x78 }, //
		{ 0xa0, 0x68 }, //
		{ 0xa1, 0x03 }, //
		{ 0xa6, 0xd8 }, //
		{ 0xa7, 0xd8 }, //
		{ 0xa8, 0xf0 }, //
		{ 0xa9, 0x90 }, //
		{ 0xaa, 0x94 }, //
		{ 0x10, 0x00 },

		// AWB parameters
		{ 0x43, 0x0a }, //
		{ 0x44, 0xf0 }, //
		{ 0x45, 0x34 }, //
		{ 0x46, 0x58 }, //
		{ 0x47, 0x28 }, //
		{ 0x48, 0x3a }, //
		{ 0x59, 0x88 }, //
		{ 0x5a, 0x88 }, //
		{ 0x5b, 0x44 }, //
		{ 0x5c, 0x67 }, //
		{ 0x5d, 0x49 }, //
		{ 0x5e, 0x0e }, //
		{ 0x6c, 0x0a }, //
		{ 0x6d, 0x55 }, //
		{ 0x6e, 0x11 }, //
		{ 0x6f, 0x9f }, //
		{ 0x6a, 0x40 }, //
		{ 0x01, 0x40 }, //
		{ 0x02, 0x60 }, //
		{ 0x13, 0xe7 },

		// Additional parameters
		{ 0x34, 0x11 }, //
		{ 0x3f, 0x00 }, //
		{ 0x75, 0x05 }, //
		{ 0x76, 0xe1 }, //
		{ 0x4c, 0x00 }, //
		{ 0x77, 0x01 }, //
		{ 0xb8, 0x0a }, //
		{ 0x41, 0x18 }, //
		{ 0x3b, 0x12 }, //
		{ 0xa4, 0x88 }, //
		{ 0x96, 0x00 }, //
		{ 0x97, 0x30 }, //
		{ 0x98, 0x20 }, //
		{ 0x99, 0x30 }, //
		{ 0x9a, 0x84 }, //
		{ 0x9b, 0x29 }, //
		{ 0x9c, 0x03 }, //
		{ 0x9d, 0x4c }, //
		{ 0x9e, 0x3f }, //
		{ 0x78, 0x04 }, //
		{ 0x0e, 0x61 }, //
		{ 0x0f, 0x4b }, //
		{ 0x16, 0x02 }, //
		{ 0x1e, 0x00 }, //
		{ 0x21, 0x02 }, //
		{ 0x22, 0x91 }, //
		{ 0x29, 0x07 }, //
		{ 0x33, 0x0b }, //
		{ 0x35, 0x0b }, //
		{ 0x37, 0x1d }, //
		{ 0x38, 0x71 }, //
		{ 0x39, 0x2a }, //
		{ 0x3c, 0x78 }, //
		{ 0x4d, 0x40 }, //
		{ 0x4e, 0x20 }, //
		{ 0x69, 0x00 }, //
		{ 0x6b, 0x3a }, //
		{ 0x74, 0x10 }, //
		{ 0x8d, 0x4f }, //
		{ 0x8e, 0x00 }, //
		{ 0x8f, 0x00 }, //
		{ 0x90, 0x00 }, //
		{ 0x91, 0x00 }, //
		{ 0x96, 0x00 }, //
		{ 0x9a, 0x00 }, //
		{ 0xb1, 0x0c }, //
		{ 0xb2, 0x0e }, //
		{ 0xb3, 0x82 }, //
		{ 0x4b, 0x01 },
};


//-- STM32F407VGT
//I2C1 : PB8(SCL),9(SDA)
void sscb_bb_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //PB8/9
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 ; //SCL
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; //SDA --
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 		// we want the pins to be an input on init. It may be output on Write operation or input on Read Operation.
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; 	//OpenDrain
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	//
	GPIO_SetBits(GPIOB, GPIO_Pin_8); // PB8-SCL-HIGH
	GPIO_SetBits(GPIOB, GPIO_Pin_9); // PB9-SDA-HIGH

	printf("Init sscb-bb Done.\r\n");
}

//We have found that the clocking speed is 400Kbps without somelag();
//     |----0---|---1----|-- TA --|
//        +--+     +--+     +--+
// SCL |--+  +--|--+  +--|--+  +--|

// ----+        +--------+INPUT---+
// SDA +--------+        +-- HI-Z-+

static void sscb_bb_output_8bits(unsigned char val)
{
	unsigned char n=8;
	GPIO_InitTypeDef GPIO_InitStruct;
	//SDA-PB9 -- Set Output
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //PB9
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;//PB9
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; 	// this sets the pin type to push / pull (as opposed to open drain)
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	GPIO_ResetBits(GPIOB, GPIO_Pin_8);   // Set Low SCL
	somedelay(100);
	for(val <<= (8-n); n; val <<= 1, n--)
	{	// Output SDA first
		if (val & 0x80)
			GPIO_SetBits(GPIOB, GPIO_Pin_9); // PB9 Set high
		else
			GPIO_ResetBits(GPIOB, GPIO_Pin_9); // PB9 Set low
		//Then issue SCL pulse
		somedelay(50);
		GPIO_SetBits(GPIOB, GPIO_Pin_8);	// Set High SCL -- Peer will latch on this rising edge.
		somedelay(50);
		GPIO_ResetBits(GPIOB, GPIO_Pin_8);   // Set Low SCL
		somedelay(100);
	}
	//issue a clk for dont care bit
	GPIO_SetBits(GPIOB, GPIO_Pin_8);	// Set High SCL -- Peer will latch on this rising edge.
	somedelay(100);
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);   // Set Low SCL
	somedelay(100);
}
static void sscb_bb_stop(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	//SDA LOW
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //PB9
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;//PB9
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; 	// this sets the pin type to push / pull (as opposed to open drain)
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	GPIO_ResetBits(GPIOB, GPIO_Pin_9);   // Set Low SDA
	somedelay(50);

	// Output clock (SCL signal) : HIGH
	GPIO_SetBits(GPIOB, GPIO_Pin_8);

	// Set input to Hi-Z
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //PB9
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;//PB9
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 		// we want the pins to be an output
	//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;	// this sets the pin type to open drain
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);//
	somedelay(100);
}
static void sscb_bb_turnaround(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	// Set input to Hi-Z
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //PB9
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;//PB9
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 		// we want the pins to be an output
	//GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	//GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;	// this sets the pin type to push / pull (as opposed to open drain)
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

	// Output clock (SCL signal) : HIGH
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	//GPIO_ResetBits(GPIOB, GPIO_Pin_8);

}

static unsigned char sscb_bb_input_8bits(void)
{
	unsigned int	i;
	volatile val = 0;
	GPIO_InitTypeDef GPIO_InitStruct;

	//SDA INPUT - // Set input to Hi-Z
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //PB9
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;//PB9
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStruct);//


	GPIO_ResetBits(GPIOB, GPIO_Pin_8);   // Set Low SCL
	somedelay(100);
	for (i = 0; i < 8; i++)
	{
		val	<<=	1;

		// First, Output clock (SCL signal)

		GPIO_SetBits(GPIOB, GPIO_Pin_8);	// Set High SCL -- Peer will latch on this rising edge.
		somedelay(100);
		//Get Data
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9))
			val	|=	1;
		GPIO_ResetBits(GPIOB, GPIO_Pin_8);   // Set Low SCL
		somedelay(100);


	}
	return	(unsigned char)val;
}

unsigned char sscb_bb_readReg(unsigned char i2cAddr8, unsigned char Reg)
{
	static unsigned char val;

	// i2c address with ReadFlag(1)
	sscb_bb_output_8bits(OV7670_ADDR_8BIT_WRITE);//i2cAddr8);

	// reg address
	sscb_bb_output_8bits(Reg);

	sscb_bb_stop();


	// i2c address with ReadFlag(1)
	sscb_bb_output_8bits(OV7670_ADDR_8BIT_READ);

	// read the data value
	val	=	sscb_bb_input_8bits();

	//stop
	sscb_bb_stop();
	// turnaround makes SDA pin as tristated.
	//sscb_bb_turnaround(); //

	return val;
}

void sscb_bb_writeReg(unsigned char Reg, unsigned char val)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	// Set MDIO and MDC pin as output
	sscb_bb_output_8bits(OV7670_ADDR_8BIT_WRITE);

	// reg address
	sscb_bb_output_8bits(Reg);

	// write the data value
	sscb_bb_output_8bits(val);

	// turnaround MDIO makes MDIO pin as tristated. But in this phase, it will issue a needless clock pulse.
	//sscb_bb_turnaround();
	sscb_bb_stop();

}


//TEST
//you don't use reading from OV Sensor registers?
//The SCCB specification not contains definition of  "repeated start" condition, therefore a STOP condition must be generated
//after address phase of reading cycle before data read phase
//to avoid generation of "repeated START" condition.
void sccb_OV7670_loop(void) {
	uint8_t data, i = 0;
	unsigned char sendbuf[10];
	u8 err;
/*
	for (i = 0; i < OV7670_REG_NUM; i++) {
		data = OV7670_reg[i][1];
		err = SCCB_write_reg(OV7670_reg[i][0], &data);
		printf("Writing register: ");
		printfi(i);
		printf("\r\n");

		if (err == true) {
			printf("Failed to update register\r\n");
			break;
		}

		Delay(0xFFFF);
	}
*/
	//Generate 24MHz Clock from MCO2.
	MCO2_Config_24MHz();
/*
	//sccb_Init(I2C1,Y_I2C_CLOCKFREQ);
	gI2Cx = I2C1;
	yI2C_Init(gI2Cx,100000);//100Kbps

	while(1){
		for (i = 0; i < OV7670_REG_NUM; i++) {
			sendbuf[0]=OV7670_reg[i][0];//data mode
			sendbuf[1]=OV7670_reg[i][1];
			I2C_ySendBurst(OV7670_ADDR_8BIT_WRITE, &sendbuf[0], 2);//I2CSendBurst(OLED_ADDRESS, ozledsendbuf, 2);
		}
	}

	// Read camera registers
	while(1){
	for (i = 0; i < OV7670_REG_NUM; i++) {
		//data = OV7670_reg[i][1];
		err = sccb_read_reg(OV7670_reg[i][0], &data);
		if (err == 1) {
			printf("Failed to update register\r\n");
			//break;
		}

		printf("Reading register[0x%02x]=0x%02x\r\n",OV7670_reg[i][0], data);

		delayms(100);
	}
	}
*/
	sscb_bb_init();
	// Read camera registers
	while(1){
		for (i = 0; i < OV7670_REG_NUM; i++) {

			data = sscb_bb_readReg(OV7670_ADDR_8BIT_READ, OV7670_reg[i][0]);

			printf("Reading register[0x%02x]=0x%02x\r\n",OV7670_reg[i][0], data);

			delayms(100);
		}

		sscb_bb_writeReg(0x12, 0x0c);
		data = sscb_bb_readReg(OV7670_ADDR_8BIT_READ, 0x12);

		printf("Reading register[0x%02x]=0x%02x(should be 0x0c)\r\n",0x12, data);

		delayms(1000);
	}
}
/*
short sccb_Timeout(short wert){
	short ret_wert = wert;
	//Stop and Reset
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_SoftwareResetCmd(I2C1, ENABLE);
	I2C_SoftwareResetCmd(I2C1, DISABLE);

	//Deinit
	I2C_DeInit(I2C1);
	//INit
	I2C_Cmd(I2C1, DISABLE);
	I2C_Cmd(I2C1, ENABLE);

	return(ret_wert);
}

void sccb_Internal_Init_I2C(void) {

	I2C_InitTypeDef I2C_InitStructure;

	I2C_DeInit(I2C1);
	//I2C Init
	I2C_InitStructure.I2C_ClockSpeed = Y_I2C_CLOCKFREQ;//100000; //100Kbps
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;//I2C_Ack_Enable; //// disable acknowledge when reading
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	//I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
	I2C_Init(I2C1, &I2C_InitStructure);
	//Enable I2C (reset)
	I2C_Cmd(I2C1, DISABLE);
	I2C_Cmd(I2C1, ENABLE);
    //I2C reset
	//RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	//RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
}

#if (PROCESSOR == STM32F407VGT6)
//I2C1 : PB8(SCL),9(SDA)
void sccb_Init(I2C_TypeDef * I2Cx,u32 I2Cspeed) {
    GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	//gI2Cx = I2Cx;

	// Enable I2C clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	// enable the peripheral clock for the pins used by PB8 for I2C SCL and PB9 for I2C1_SDL
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//Assign alternate functions
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); //SCL First
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	//Set OpenDrain
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //OpenDrain
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //No internal PU/PD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//I2C Init
	sccb_Internal_Init_I2C();


}
#elif(PROCESSOR == STM32F407VZT6)
//I2C1 : PF0(SDA), PF1(SCL),
void sccb_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed) {

    GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	gI2Cx = I2Cx;

	// Enable GPIO clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //OpenDrain
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //No internal PU/PD
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	// Enable I2C clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_I2C1); //SCL First
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_I2C1);
	I2C_InitStructure.I2C_ClockSpeed = I2Cspeed;//100000; //100Kbps
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStructure);

	// Enable I2C
	I2C_Cmd(I2C1, DISABLE);
	I2C_Cmd(I2C1, ENABLE);
}
#endif

//Used for both Write and Read Operations
bool sccb_write_reg(uint8_t reg_addr, uint8_t* data) {
	uint32_t timeout = 0x7FFFFF;

	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		if ((timeout--) == 0) {
			printf("Busy Timeout\r\n");
			return 1;
		}
	}

	// Send start bit
	I2C_GenerateSTART(I2C1, ENABLE);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			printf("Start bit Timeout\r\n");
			return 1;
		}
	}

	// Send slave address (camera write address)
	I2C_Send8bitAddress(I2C1, OV7670_ADDR_8BIT_WRITE, I2C_Direction_Transmitter); //0x42

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			printf("Slave address timeout\r\n");
			return 1;
		}
	}

	// Send register address
	I2C_SendData(I2C1, reg_addr);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			printf("Register timeout\r\n");
			return 1;
		}
	}

	// Send new register value
	I2C_SendData(I2C1, *data);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			printf("Value timeout\r\n");
			return 1;
		}
	}

	// Send stop bit
	I2C_GenerateSTOP(I2C1, ENABLE);
	return 0;
}

bool sccb_read_reg(uint8_t reg_addr, uint8_t* data) {
	uint32_t timeout = 0x7FFF;

	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		if ((timeout--) == 0) {
			printf("Busy Timeout\r\n");
			return 1;
		}
	}

	// Send start bit
	I2C_GenerateSTART(I2C1, ENABLE);
	//Check EV5 and clear it
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			printf("Start bit Timeout\r\n");
			return 1;
		}
	}

	// Send slave address (camera write address)
	I2C_Send8bitAddress(I2C1, OV7670_ADDR_8BIT_WRITE, I2C_Direction_Transmitter); //0x42
	//Check EV6 and clear it
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			printf("Slave address timeout\r\n");
			return 1;
		}
	}

	// Send register address
	I2C_SendData(I2C1, reg_addr);
	//Check EV8 and clear it
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			printf("Register timeout\r\n");
			return 1;
		}
	}

	//Clear AF flag if arised
	I2C1->SR1 |= (unsigned short)0x0400;
	//Check EV7 and clear it
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if ((timeout--) == 0) {
			printf("Register timeout\r\n");
			return 1;
		}
	}

	//Gen ReStart Condition========938
	I2C_GenerateSTART(I2C1, ENABLE);
	//Check EV6 and clear it
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			printf("Slave address timeout\r\n");
			return 1;
		}
	}

	// Send slave address (camera write address) -950
	I2C_Send8bitAddress(I2C1, OV7670_ADDR_8BIT_READ, I2C_Direction_Receiver); //0x43
	//Check EV6 and clear it
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			printf("Slave address timeout\r\n");
			return 1;
		}
	}

	//Prepare NACK
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	//Check EV7 and clear it
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if ((timeout--) == 0) {
			printf("Slave address timeout\r\n");
			return 1;
		}
	}

	// Send stop bit
	I2C_GenerateSTOP(I2C1, ENABLE);

	*data = I2C_ReceiveData(I2C1);

	return 0;
}
*/
#endif //PROCESSOR
