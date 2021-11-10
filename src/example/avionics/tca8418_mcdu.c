// TCA8418 Keyscan IC Driver
// All for Keyscan (not for GPIO)
// I2C Address = 0x68(8-bit)
// Test on STM32F103RCT6
//	-SPI1
//	-PC13 for IRQ
#if 1

#include "yInc.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"

extern int stm_I2C_SendBurst(unsigned char slave_addr, unsigned char *burst, unsigned char datalen);
extern unsigned char stm_I2C_ReceiveBurstWithRestartCondition(unsigned char SlaveAddress, unsigned char reg, unsigned char *buf, unsigned char nbyte);
extern void stm_I2C_Init(I2C_TypeDef * I2Cx, u32 I2Cspeed);
extern I2C_TypeDef *gI2Cx;
extern unsigned char g_bI2CModuleConfigDone;

#define TCA8418_INTERRUPT_SUPPORT
//#undef TCA8418_INTERRUPT_SUPPORT

//I2C Address
#define TCA8418_KEYPAD_ADDR8 0x68//0x37 (7-bit)

#define GPIO 0x38
#define EDGE 0x39
#define LEVEL 0x40
#define INTERRUPT 0x41
#define NOINTERRUPT 0x42
#define FIFO 0x43
#define NOFIFO 0x44
#define DEBOUNCE 0x45
#define NODEBOUNCE 0x46

//YOON
#define  INPUT 0
#define  OUTPUT 1
#define  INPUT_PULLUP 2
#define  KEYPAD 0x37

/* TCA8418 hardware limits */
#define TCA8418_MAX_ROWS 8
#define TCA8418_MAX_COLS 10

/* TCA8418 register offsets */
#define REG_CFG 0x01
#define REG_INT_STAT 0x02
#define REG_KEY_LCK_EC 0x03
#define REG_KEY_EVENT_A 0x04
#define REG_KEY_EVENT_B 0x05
#define REG_KEY_EVENT_C 0x06
#define REG_KEY_EVENT_D 0x07
#define REG_KEY_EVENT_E 0x08
#define REG_KEY_EVENT_F 0x09
#define REG_KEY_EVENT_G 0x0A
#define REG_KEY_EVENT_H 0x0B
#define REG_KEY_EVENT_I 0x0C
#define REG_KEY_EVENT_J 0x0D
#define REG_KP_LCK_TIMER 0x0E
#define REG_UNLOCK1 0x0F
#define REG_UNLOCK2 0x10
#define REG_GPIO_INT_STAT1 0x11
#define REG_GPIO_INT_STAT2 0x12
#define REG_GPIO_INT_STAT3 0x13
#define REG_GPIO_DAT_STAT1 0x14
#define REG_GPIO_DAT_STAT2 0x15
#define REG_GPIO_DAT_STAT3 0x16
#define REG_GPIO_DAT_OUT1 0x17
#define REG_GPIO_DAT_OUT2 0x18
#define REG_GPIO_DAT_OUT3 0x19
#define REG_GPIO_INT_EN1 0x1A
#define REG_GPIO_INT_EN2 0x1B
#define REG_GPIO_INT_EN3 0x1C
#define REG_KP_GPIO1 0x1D
#define REG_KP_GPIO2 0x1E
#define REG_KP_GPIO3 0x1F
#define REG_GPI_EM1 0x20
#define REG_GPI_EM2 0x21
#define REG_GPI_EM3 0x22
#define REG_GPIO_DIR1 0x23
#define REG_GPIO_DIR2 0x24
#define REG_GPIO_DIR3 0x25
#define REG_GPIO_INT_LVL1 0x26
#define REG_GPIO_INT_LVL2 0x27
#define REG_GPIO_INT_LVL3 0x28
#define REG_DEBOUNCE_DIS1 0x29
#define REG_DEBOUNCE_DIS2 0x2A
#define REG_DEBOUNCE_DIS3 0x2B
#define REG_GPIO_PULL1 0x2C
#define REG_GPIO_PULL2 0x2D
#define REG_GPIO_PULL3 0x2E

/* TCA8418 bit definitions */
#define CFG_AI 0x80
#define CFG_GPI_E_CFG 0x40
#define CFG_OVR_FLOW_M 0x20
#define CFG_INT_CFG 0x10
#define CFG_OVR_FLOW_IEN 0x08
#define CFG_K_LCK_IEN 0x04
#define CFG_GPI_IEN 0x02
#define CFG_KE_IEN 0x01

#define INT_STAT_CAD_INT 0x10
#define INT_STAT_OVR_FLOW_INT 0x08
#define INT_STAT_K_LCK_INT 0x04
#define INT_STAT_GPI_INT 0x02
#define INT_STAT_K_INT 0x01

/* TCA8418 register masks */
#define KEY_LCK_EC_KEC 0x7
#define KEY_EVENT_CODE 0x7f
#define KEY_EVENT_VALUE 0x80

/* TCA8418 Rows and Columns */
#define ROW0 0x01
#define ROW1 0x02
#define ROW2 0x04
#define ROW3 0x08
#define ROW4 0x10
#define ROW5 0x20
#define ROW6 0x40
#define ROW7 0x80

#define COL0 0x0001
#define COL1 0x0002
#define COL2 0x0004
#define COL3 0x0008
#define COL4 0x0010
#define COL5 0x0020
#define COL6 0x0040
#define COL7 0x0080
#define COL8 0x0100
#define COL9 0x0200

struct _tca8418_keypad {
#ifdef TCA8418_INTERRUPT_SUPPORT
  volatile bool keyIRQ;
  volatile unsigned  _oldPIN;  /** Old value of _PIN variable */
  volatile unsigned char _isrIgnore;  /** ISR ignore flag */
 unsigned char _pcintPin;  /** PCINT pin used for "INT" pin handling */
 //unsigned char _intMode[24];  /** Interrupts modes of pins ( LOW, CHANGE, FALLING, RISING)  */
#endif
  unsigned  _PKG; // Pin Keypad or GPIO 0=GPIO, 1=Keypad
  unsigned  _PORT;
  unsigned  _PIN; // Pin State
  unsigned  _DDR; //Pin Direction INPUT or OUTPUT
  unsigned  _PUR; //Pull-Up Resistor Selection
 };

//============== prototypes =========================================
  void tca8418_setup(void);
  //void tca8418_begin(unsigned char rows, uint16_t cols, unsigned char config);
  char tca8418_readKeypad(unsigned char *key, unsigned char *row, unsigned char *col);
  bool tca8418_configureKeys( unsigned char rows, uint16_t cols, unsigned char config);
  void tca8418_toggle(struct _tca8418_keypad *pkeypad,unsigned  pin);
  void tca8418_blink(struct _tca8418_keypad *pkeypad, unsigned  pin, uint16_t count, unsigned  duration);

  void  tca8418_readGPIO();
  void  tca8418_updateGPIO();
  void  tca8418_dumpreg(struct _tca8418_keypad *pkeypad);

  unsigned char  tca8418_getKeyEvent(unsigned char event);
  unsigned char  tca8418_getKeyEventCount(struct _tca8418_keypad *pkeypad);
  unsigned   tca8418_getGPIOInterrupt(void);
  //bool  tca8418_isKeyDown(unsigned char key);
  //bool  tca8418_getKey(unsigned char *key);

#ifdef TCA8418_INTERRUPT_SUPPORT
  void tca8418_enableInterrupt(void);//unsigned char pin, void(*selfCheckFunction)(void));
  //void tca8418_disableInterrupt();
  //void tca8418_pinInterruptMode(unsigned  pin, unsigned char mode, unsigned char level, unsigned char fifo);
  //void tca8418_pinInterruptMode(unsigned  pin, unsigned char mode);
#endif
  unsigned char  tca8418_getInterruptStatus(struct _tca8418_keypad *pkeypad);
  void  tca8418_clearInterruptStatusWithFlags(struct _tca8418_keypad *pkeypad, unsigned char flags);
  void  tca8418_clearInterruptStatus(struct _tca8418_keypad *pkeypad);


struct _tca8418_keypad g_tca8418_keypad;

//===================== i2c write/read for TCA8418 ==========================
void tca8418_writeByte(unsigned char data, unsigned char reg) {
	unsigned char sendbuf[4];
	sendbuf[0] = reg;
	sendbuf[1] = data;

	stm_I2C_SendBurst(TCA8418_KEYPAD_ADDR8, sendbuf, 2);

  return;
}

void tca8418_write3Bytes(unsigned  data, unsigned char reg) {

	unsigned char sendbuf[4];
  union
  {
    unsigned char b[4];
    unsigned  w;
  } datau;
  
  datau.w = data;

  sendbuf[0] = reg;
  sendbuf[1] = datau.b[0];
  sendbuf[2] = datau.b[1];
  sendbuf[3] = datau.b[2];

  stm_I2C_SendBurst(TCA8418_KEYPAD_ADDR8, sendbuf, 4);
  
  return;
}

bool tca8418_readByte(unsigned char *data, unsigned char reg)
{
	stm_I2C_ReceiveBurstWithRestartCondition(TCA8418_KEYPAD_ADDR8, reg, data, 1);
	return 0; //success
}

bool tca8418_read3Bytes(unsigned  *data, unsigned char reg) {

  union
  {
    unsigned char b[4];
    unsigned  w;
  } datau;
  
  datau.w = *data;
  
  stm_I2C_ReceiveBurstWithRestartCondition(TCA8418_KEYPAD_ADDR8, reg, data, 3);

  
  datau.b[0] = data[0];
  datau.b[1] = data[1];
  datau.b[2] = data[2];
  
  *data = datau.w;
  return(0); //success
}

//show registers
void tca8418_dumpreg(struct _tca8418_keypad *pkeypad) {
  unsigned char data;
  int x;
  for(x=0x01;x<0x2F;x++) {
    tca8418_readByte(&data, x);
	printf("%02x:",x);
	printf("%02x ",data);
  }
  printf("\r\n");
}

//======================== for GPIO opeation (Not used here) ================================
void tca8418_readGPIO(struct _tca8418_keypad *pkeypad)
{
#ifdef TCA8418_INTERRUPT_SUPPORT
	/* Store old _PIN value */
	//_oldPIN = _PIN;
#endif

	tca8418_read3Bytes((unsigned  *)&pkeypad->_PORT, REG_GPIO_DAT_OUT1);  //Read Data OUT Registers
	tca8418_read3Bytes((unsigned  *)&pkeypad->_PIN, REG_GPIO_DAT_STAT1);	//Read Data STATUS Registers
	tca8418_read3Bytes((unsigned  *)&pkeypad->_DDR, REG_GPIO_DIR1);		//Read Data DIRECTION Registers
	tca8418_read3Bytes((unsigned  *)&pkeypad->_PKG, REG_KP_GPIO1);		//Read Keypad/GPIO SELECTION Registers
	tca8418_read3Bytes((unsigned  *)&pkeypad->_PUR, REG_GPIO_PULL1);		//Read KPull-Up RESISTOR Registers
}

void tca8418_updateGPIO(struct _tca8418_keypad *pkeypad) {

	tca8418_write3Bytes((unsigned )pkeypad->_PORT, REG_GPIO_DAT_OUT1);  	//Write Data OUT Registers
	tca8418_write3Bytes((unsigned )pkeypad->_DDR, REG_GPIO_DIR1);			//Write Data DIRECTION Registers
	tca8418_write3Bytes((unsigned )pkeypad->_PKG, REG_KP_GPIO1);			//Write Keypad/GPIO SELECTION Registers
	tca8418_write3Bytes((unsigned )pkeypad->_PUR, REG_GPIO_PULL1);			//Write Pull-Up RESISTOR Registers
}

unsigned  tca8418_getGPIOInterrupt(void) {
  unsigned  Ints;

  union {
    unsigned  val;
	unsigned char arr[4];
  } IntU;

  tca8418_readByte(&IntU.arr[2], REG_GPIO_INT_STAT3);
  tca8418_readByte(&IntU.arr[1], REG_GPIO_INT_STAT2);
  tca8418_readByte(&IntU.arr[0], REG_GPIO_INT_STAT1);

  Ints = IntU.val;
  return(Ints);
}


void tca8418_digitalWrite(struct _tca8418_keypad *pkeypad, unsigned  pin, unsigned char value) {

  
  if(value)
    m_bitSet(pkeypad->_PORT, pin);
  else
    m_bitClear(pkeypad->_PORT, pin);

  tca8418_updateGPIO(pkeypad);
}

unsigned char tca8418_digitalRead(struct _tca8418_keypad *pkeypad,unsigned  pin) {

	tca8418_readGPIO(pkeypad);
  
  return(pkeypad->_PIN & bit(pin)) ? 1 : 0;
}

void tca8418_write(struct _tca8418_keypad *pkeypad,unsigned  value) {

	pkeypad->_PORT = value;
  
	tca8418_updateGPIO(pkeypad);
}

unsigned  tca8418_read(struct _tca8418_keypad *pkeypad) {

	tca8418_readGPIO(pkeypad);
  
	return pkeypad->_PORT;
}

void tca8418_toggle(struct _tca8418_keypad *pkeypad,unsigned  pin) {

	pkeypad->_PORT ^= (bit(pin));
  
	tca8418_updateGPIO(pkeypad);
}

void tca8418_blink(struct _tca8418_keypad *pkeypad,unsigned  pin, uint16_t count, unsigned  duration) {

  duration /= count * 2;
  
  while(count--) {
	tca8418_toggle(pkeypad,pin);
	delayms(duration);
	tca8418_toggle(pkeypad,pin);
	delayms(duration);
  }
}
//for changing tac8418's pin mode. e.g., from gpio to keyscan or vice versa.
void tca8418_pinMode(struct _tca8418_keypad *pkeypad, unsigned  pin, unsigned char mode) {
  unsigned  pullUp, dbc;

  tca8418_readGPIO(pkeypad);

  switch(mode) {
    case INPUT:
	  m_bitClear(pkeypad->_PORT, pin);
	  m_bitClear(pkeypad->_DDR, pin);
	  m_bitClear(pkeypad->_PKG, pin);
	  m_bitSet(pkeypad->_PUR, pin);
	  break;
	case INPUT_PULLUP:
	  m_bitClear(pkeypad->_PORT, pin);
	  m_bitClear(pkeypad->_DDR, pin);
	  m_bitClear(pkeypad->_PKG, pin);
	  m_bitClear(pkeypad->_PUR, pin);
	  break;
    case OUTPUT:
	  m_bitClear(pkeypad->_PORT, pin);
	  m_bitSet(pkeypad->_DDR, pin);
	  m_bitClear(pkeypad->_PKG, pin);
	  m_bitSet(pkeypad->_PUR, pin);
	  break;
	case KEYPAD:
	  m_bitClear(pkeypad->_PORT, pin);
	  m_bitClear(pkeypad->_DDR, pin);
	  m_bitSet(pkeypad->_PKG, pin);
	  m_bitClear(pkeypad->_PUR, pin);
	  break;
	case DEBOUNCE:
	  tca8418_read3Bytes((unsigned  *)&dbc, REG_DEBOUNCE_DIS1);
	  m_bitClear(dbc, pin);
	  tca8418_write3Bytes((unsigned )dbc, REG_DEBOUNCE_DIS1);
	  break;
	case NODEBOUNCE:
	  tca8418_read3Bytes((unsigned  *)&dbc, REG_DEBOUNCE_DIS1);
	  m_bitSet(dbc, pin);
	  tca8418_write3Bytes((unsigned )dbc, REG_DEBOUNCE_DIS1);
	  break;
	default:
	  break;
  }
  tca8418_updateGPIO(pkeypad);
}
//===================== interrupt related ===================================================
#ifdef TCA8418_INTERRUPT_SUPPORT
//enable irq for PC15//enable irq for PC13
void tca8418_enableInterrupt(void)
{
		  GPIO_InitTypeDef   GPIO_InitStructure;
		  NVIC_InitTypeDef   NVIC_InitStructure;
		  EXTI_InitTypeDef   EXTI_InitStructure;

		  /* Enable GPIOC clock */
	 	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

		  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
		  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
		  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		  NVIC_Init(&NVIC_InitStructure);

		  /* Configure PC15 pin as input floating */ /* Configure PC13 pin as input floating */
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//GPIO_Pin_13;
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOC, &GPIO_InitStructure);
		  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource15);//		  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

		  EXTI_InitStructure.EXTI_Line = EXTI_Line15; //EXTI_Line13;
		  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		  EXTI_Init(&EXTI_InitStructure);
}

// irq handler for pc13 pin of the MCU.
#if (USE_EXTI15_10 == USE_EXTI15_10_TCA8418_KEYPAD)
//PC13
void EXTI15_10_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line15) != RESET) {//if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
		GPIO_ToggleBits(GPIOC, GPIO_Pin_14); //ULED -103 (PC14)
		g_tca8418_keypad.keyIRQ = 1;
		EXTI_ClearITPendingBit(EXTI_Line15); //EXTI_ClearITPendingBit(EXTI_Line13);
	}
}

#endif
#endif

#if 0
void tca8418_pinInterruptMode(unsigned  pin, unsigned char mode, unsigned char level, unsigned char fifo) {
  unsigned  intSetting, levelSetting, eventmodeSetting;

	
  tca8418_read3Bytes((unsigned  *)&intSetting, REG_GPIO_INT_EN1);
  tca8418_read3Bytes((unsigned  *)&levelSetting, REG_GPIO_INT_LVL1);
  tca8418_read3Bytes((unsigned  *)&eventmodeSetting, REG_GPI_EM1);

  switch(mode) {
    case INTERRUPT:
	  m_bitSet(intSetting, pin);
	  break;
	case NOINTERRUPT:
	  m_bitClear(intSetting, pin);
	  break;
	default:
		break;
  }
  
  switch(level) {
    case LOW:
	  m_bitClear(levelSetting, pin);
	  break;
	case HIGH:
	  m_bitSet(levelSetting, pin);
	  break;
	default:
	  break;
  }
  
  switch(fifo) {
    case FIFO:
	  m_bitSet(eventmodeSetting, pin);
	  break;
	case NOFIFO:
	  m_bitClear(eventmodeSetting, pin);
	  break;
	default:
	  break;
  }
  
  tca8418_write3Bytes((unsigned )intSetting, REG_GPIO_INT_EN1);
  tca8418_write3Bytes((unsigned )levelSetting, REG_GPIO_INT_LVL1);
  tca8418_write3Bytes((unsigned )eventmodeSetting, REG_GPI_EM1);
  
}

void tca8418_pinInterruptMode(unsigned  pin, unsigned char mode) {
  pinInterruptMode(pin, mode, 0, 0);
}

#endif
// read interrupt status register
unsigned char tca8418_getInterruptStatus(struct _tca8418_keypad *pkeypad) {
  unsigned char status;
  tca8418_readByte(&status, REG_INT_STAT);
  return(status & 0x0F);
}

// clear interrupt by writing 1 on interrupt status register
void tca8418_clearInterruptStatusWithFlags(struct _tca8418_keypad *pkeypad,unsigned char flags) {
  flags &= 0x0F;
  tca8418_writeByte(flags, REG_INT_STAT); //clear irq by writing bits of 1.
}

void tca8418_clearInterruptStatus(struct _tca8418_keypad *pkeypad) {
	tca8418_clearInterruptStatusWithFlags(pkeypad, 0x0F);
}
//=========================== get keyscan ============================
// get keycode when both pressed and depressed.
unsigned char tca8418_getKeyEvent(unsigned char event) {
  unsigned char keycode;

  if (event > 9)
    return 0x00;

  tca8418_readByte(&keycode, (REG_KEY_EVENT_A+event));

  printf("keyEv=%u\r\n", keycode);
  return(keycode);
}
/*
unsigned char tca8418_getKeyEvent(struct _tca8418_keypad *pkeypad) {
  return(tca8418_getKeyEvent(0));
}
*/

unsigned char tca8418_getKeyEventCount(struct _tca8418_keypad *pkeypad) {
  unsigned char count;

  tca8418_readByte(&count, REG_KEY_LCK_EC);
  return(count & 0x0F);
}


/* ======= accessory ================
bool tca8418_isKeyDown(unsigned char key) {
  if(key & 0x80)
    return true;
  else
    return false;
}

bool tca8418_getKey(unsigned char *key) {
	char updown;
	unsigned char row, col;

	updown = tca8418_readKeypad(key, &row, &col);
	*key = *key & 0x7F;

	return(tca8418_isKeyDown(updown));
}
*/

// key decoding
char tca8418_readKeypad(unsigned char *key, unsigned char *row, unsigned char *col)
{
	unsigned char keycode;
	//Check for Interrupt flag and process
	if(g_tca8418_keypad.keyIRQ) {

		keycode = tca8418_getKeyEvent(0); 				//Get first keycode from FIFO

		//See the key event table in the datasheet
		*row = (keycode & 0x7F) / 10;
		*col = (keycode & 0x7F) % 10;
		printf("%u=(%u:%u)(row,col) ",(keycode & 0x7F), *row, *col );

		g_tca8418_keypad.keyIRQ = false; 		//Reset Our Interrupt flag
		tca8418_clearInterruptStatus(&g_tca8418_keypad); 	//Reset TCA8418 Interrupt Status Register.

		*key = keycode;
		//print keycode masking the key down/key up bit (bit7)
		if(keycode & 0x80){
			//printf(" key down\r\n");
			return 1; //PRESSED
		}
		else{
			//printf(" key up\r\n");
			return 0; //UNPRESSED
		}
	}else
		return -1; //NONE
}

//====  Configure the TCA8418 for keypad operation ====
bool tca8418_configureKeysForAllKeypad(unsigned char rows, uint16_t cols, unsigned char config)
{
	unsigned char col_tmp;

	//Pins all default to Keypad.

	//rows
	tca8418_writeByte(rows, REG_KP_GPIO1); //each bit of 1 --> KP matrix; 0--> GPIO

	//columns of 10 bits
	col_tmp = (unsigned char)(0xff & cols);
	tca8418_writeByte(col_tmp, REG_KP_GPIO2);

	col_tmp = (unsigned char)(0x03 & (cols>>8));
	tca8418_writeByte(col_tmp, REG_KP_GPIO3);

	config |= CFG_AI; //auto increment
	tca8418_writeByte(config, REG_CFG);
}
bool tca8418_configureKeysForAllGpio(unsigned char rows, uint16_t cols, unsigned char config)
{
	unsigned char col_tmp;
  //Pins all default to GPIO. pinMode(x, KEYPAD); may be used for individual pins.
  tca8418_writeByte(rows, REG_KP_GPIO1);

  col_tmp = (unsigned char)(0xff & cols);
  tca8418_writeByte(col_tmp, REG_KP_GPIO2);

  col_tmp = (unsigned char)(0x03 & (cols>>8));
  tca8418_writeByte(col_tmp, REG_KP_GPIO3);
  
  config |= CFG_AI;
  tca8418_writeByte(config, REG_CFG);
}
//== useless =========
tca8418_init(struct _tca8418_keypad *pkeypad)
{
#ifdef TCA8418_INTERRUPT_SUPPORT
	pkeypad->keyIRQ = false;
	pkeypad->_oldPIN = 0;
	pkeypad->_isrIgnore = 0;
	pkeypad->_pcintPin = 0;
	//tca8418_intMode();
#endif
}
//============== setup ==================================
//Configure for a 10X8 matrix on COL0-COL9, ROW0-ROW7 for KPAD and enable interrupts
void tca8418_setup(void) {

	printf("TCA8418 KP Setup\r\n");

	tca8418_configureKeysForAllKeypad(
		  ROW0|ROW1|ROW2|ROW3|ROW4|ROW5|ROW6|ROW7, //Row
		  COL0|COL1|COL2|COL3|COL4|COL5|COL6|COL7|COL8|COL9, //Column
          CFG_KE_IEN //keypad event interrupt enable
          | CFG_OVR_FLOW_IEN
          //| CFG_INT_CFG
          | CFG_OVR_FLOW_M); //Config
	tca8418_enableInterrupt();

/*	tca8418_pinMode(pkeypad, 1, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 2, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 3, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 4, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 5, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 6, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 7, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 8, KEYPAD);//OUTPUT);

	tca8418_pinMode(pkeypad, 9, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 10, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 11, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 12, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 13, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 14, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 15, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 16, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 17, KEYPAD);//OUTPUT);
	tca8418_pinMode(pkeypad, 18, KEYPAD);//OUTPUT);

	//tca8418_digitalWrite(pkeypad, 15, true);
	//tca8418_digitalWrite(pkeypad, 16, false);
	//tca8418_digitalWrite(pkeypad, 17, false);
*/
	tca8418_dumpreg(&g_tca8418_keypad);
}
//==== mainloop =====================
void tca8418_loop() {

	unsigned char key, row, col;
	char none_up_down;

	//I2c config
	if(!g_bI2CModuleConfigDone)
	{
#if (PROCESSOR == PROCESSOR_STM32F407VZT6)
		  gI2Cx = I2C2;
#else
		  gI2Cx = I2C1;
#endif
		printf("I2C Init...");
		stm_I2C_Init(gI2Cx,400000);//400Kbps
		g_bI2CModuleConfigDone = 1;
		printf("Done.\r\n");
	}

	printf("tca8418 @ 0x68\r\n");

	tca8418_setup();

	while(1)
	{
		none_up_down = tca8418_readKeypad(&key, &row, &col); 				//Get first keycode from FIFO
		if(none_up_down == 1){
			printf("%02u=(%u:%u)(row,col) Key Pressed\r\n",key, row, col );
		}else if(none_up_down == 0){
			printf("%02u=(%u:%u)(row,col) Key UnPressed\r\n",key, row, col );
		}
		else{ //Not happend
			//printf("Nothing\r\n");
		}
	}
}
/*
//===================

void KeyISR(void) {  //Keypad Interrupt Service Routine
  keyIRQ = true;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  //Configure for a 4X4 matrix on COL0-COL4, ROW0-ROW4 and enable interrupts
  tca8418_begin(ROW0|ROW1|ROW2|ROW3, COL0|COL1|COL2|COL3, 
                 CFG_KE_IEN|CFG_OVR_FLOW_IEN|CFG_INT_CFG|CFG_OVR_FLOW_M); 
                 
                 
  tca8418_enableInterrupt(2, KeyISR);  //Arg1= Arduino Pin number INT is connected to. Arg2= Interrupt Routine

  tca8418_pinMode(15, OUTPUT);
  tca8418_pinMode(16, OUTPUT);
  tca8418_pinMode(17, OUTPUT);
  
  tca8418_digitalWrite(15, true);
  tca8418_digitalWrite(16, false);
  tca8418_digitalWrite(17, false);
  
}

void loop() {
  //Check for Interrupt flag and process
  if(keyIRQ) {
    unsigned char key;

    key=tca8418_readKeypad(); //Get first keycode from FIFO
    printf("Keyboard ISR...Key:");
    printf((key&0x7F), HEX); //print keycode masking the key down/key up bit (bit7)
    if(key & 0x80) {
      printf(" key down");
    } else {
      printf(" key up");
    } 
    keyIRQ=false; //Reset Our Interrupt flag
    tca8418_clearInterruptStatus(); //Reset TCA8418 Interrupt Status Register.
  }
  //Do other processing
  ;
}
*/

#endif
