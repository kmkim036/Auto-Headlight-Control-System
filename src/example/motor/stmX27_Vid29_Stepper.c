//VID29 SPEC
// Gear Ratio : 180:1
// StepSize in Full    Step Mode = 1   Degree
// StepSize in Partial Step Mode = 1/3 Degree (We use it.)
// Max Angle for Rotation        = 315 Degree
#if 1
//----------- SEE STM32F407 VERSION ---------------------
#include <string.h>
#include <stdarg.h>
#include "example/yInc.h"
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
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#endif
#include "misc.h"

extern unsigned long micros();
extern char *float2str(float x);
extern void stmStepper_GPIO_setup(void);

//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39   |
//+-----------------+-----------+-----------+-----------+---------------+------------
//| BIN2(Pin32)     |           |           |           |  PB11         |PB5
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN1(Pin33)     |           |           |           |  PB1          |PA15
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN2(Pin34)     |           |           |           |  PA5          |PB3(remap)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN1(Pin35)     |           |           |           |  PA4          |PB4(remap)
//+-----------------+-----------+-----------+-----------+---------------+---------+

//----------------------------------------------------------------------
// https://github.com/clearwater/SwitecX25
// This is an Arduino library for driving Switec X25 miniature stepper motors.
// It was written specifically for the Switec X25.168,
// and compatible steppers from other manufacturers including the VID29 and MCR1108
//
// Using Zeroing(), we set the position to mid-range and waits for serial input to indicate new motor positions.
//
// Open the serial monitor and try entering values between 0 ~ 944.
//
// Note that the maximum speed of the motor will be determined by how frequently you call update().
// If you put a big slow serial.println() call in the loop below, the motor will move very slowly!
//----------------------------------------------------------------------

struct SwitecX27Vid29
{
   unsigned char stateCount;		// = 6;
   unsigned char currentState;    	// 6 steps
   unsigned int currentStep;      	// step we are currently at
   unsigned int targetStep;       	// target we are moving to
   unsigned int maxSteps;           // total steps available
   unsigned long time0;           	// time when we entered this state
   unsigned int microDelay;       	// microsecs until next state
   unsigned short (*accelTable)[2]; // accel table can be modified.
   unsigned int maxVel;           	// fastest vel allowed
   unsigned int vel;              	// steps traveled under acceleration
   char dir;                      	// direction -1,0,1
   boolean stopped;               	// true if stopped
} g_X27_Vid29;

// During zeroing we will step the motor CCW with a fixed step period defined by RESET_STEP_MICROSEC
#define RESET_STEP_MICROSEC 800
#define MAXSTEPS (315*3) //Standard X25.168 range 315 degrees at 1/3 degree steps.


// experimentation suggests that 400uS is about the step limit
// with my hand-made needles made by cutting up aluminium from
// floppy disk sliders.  A lighter needle will go faster.
// This table defines the acceleration curve.
// 1st value is the speed step, 2nd value is delay in microseconds
// 1st value in each row must be > 1st value in subsequent row
// 1st value in last row should be == maxVel, must be <= maxVel
static unsigned short X27_Vid29_defaultAccelTable[][2] = {
  {   20, 3000}, //speed step, delayInMsec
  {   50, 1500},
  {  100, 1000},
  {  150,  800},
  {  300,  600}
};
#define DEFAULT_ACCEL_TABLE_SIZE (sizeof(X27_Vid29_defaultAccelTable)/sizeof(*X27_Vid29_defaultAccelTable))

//[YOON-VID29]
// State  3 2 1 0   Value
// ------+-------+-------
// 0      0 1 0 1   0x5
// 1      0 1 0 0   0x4
// 2      0 0 1 0   0x2
// 3      1 0 1 0   0xA
// 4      1 0 0 0   0x8
// 5      0 0 0 1   0x1
static unsigned char X27_Vid29_StateMap[] = {0x5, 0x4, 0x2, 0xA, 0x8, 0x1};
//[REF]Pin4 3 2 1
// ------+-------+-------
// 0      1 0 0 1   0x9
// 1      1 0 0 0   0x8
// 2      1 1 1 0   0xd
// 3      0 1 1 0   0x6
// 4      0 1 1 1   0x7
// 5      0 0 0 1   0x1
static unsigned char MR11_StateMap[] = {0x9, 0x8, 0xd, 0x6, 0x7, 0x1};
//[REF]Pin4 3 2 1 (inverted by ULN2803A)
// ------+-------+-------
// 0      0 1 1 0   0x6
// 1      0 1 1 1   0x7
// 2      0 0 0 1   0x1
// 3      1 0 0 1   0x9
// 4      1 0 0 0   0x8
// 5      1 1 1 0   0xd
//static unsigned char MR11_StateMap[] = {0x6, 0x7, 0x1, 0x9, 0x8, 0xd};

extern void stmPcf8574_write8(const uint8_t value);

void stmPcf8574_MR11_GenApartialStep()
{
	int i;
	unsigned char mask;
	mask = MR11_StateMap[g_X27_Vid29.currentState];
	stmPcf8574_write8(mask);
	delayms(10);
	printf("State=%d(0x%02x)\r\n",g_X27_Vid29.currentState,mask);
}
#if(MCU_MODULE_VER == 35)
void stmX27_Vid29_GenApartialStep()
{
	int i;
	unsigned char mask;
	mask = X27_Vid29_StateMap[g_X27_Vid29.currentState];
	//Pin 1 - PIN32
	if(mask & 0x01)
		GPIO_SetBits(GPIOB, GPIO_Pin_11);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_11);

	//Pin 2 - Pin33
	mask >>= 1;
	if(mask & 0x01)
		GPIO_SetBits(GPIOB, GPIO_Pin_1);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);

	//Pin 3 - Pin34
	mask >>= 1;
	if(mask & 0x01)
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);

	//Pin 4 - Pin35
	mask >>= 1;
	if(mask & 0x01)
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}
#else
//+-----------------+-----------+-----------+-----------+---------------+------------------
//|                 |           |           |           | STM103M35     |STM32F103-M37/M39   |
//+-----------------+-----------+-----------+-----------+---------------+------------
//| BIN2(Pin32)     |           |           |           |  PB11         |PB5
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| BIN1(Pin33)     |           |           |           |  PB1          |PA15
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN2(Pin34)     |           |           |           |  PA5          |PB3(remap)
//+-----------------+-----------+-----------+-----------+---------------+---------+
//| AIN1(Pin35)     |           |           |           |  PA4          |PB4(remap)
//+-----------------+-----------+-----------+-----------+---------------+---------+

void stmX27_Vid29_GenApartialStep()
{
	int i;
	unsigned char mask;
	mask = X27_Vid29_StateMap[g_X27_Vid29.currentState];
	//Pin 1
	if(mask & 0x01)
		GPIO_SetBits(GPIOB, GPIO_Pin_5);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);

	//Pin 2
	mask >>= 1;
	if(mask & 0x01)
		GPIO_SetBits(GPIOA, GPIO_Pin_15);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_15);

	//Pin 3
	mask >>= 1;
	if(mask & 0x01)
		GPIO_SetBits(GPIOB, GPIO_Pin_3);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_3);

	//Pin 4
	mask >>= 1;
	if(mask & 0x01)
		GPIO_SetBits(GPIOB, GPIO_Pin_4);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_4);
}
#endif
void stmX27_Vid29_stepUp()
{
  if (g_X27_Vid29.currentStep < g_X27_Vid29.maxSteps) {
	  g_X27_Vid29.currentStep++;
	  g_X27_Vid29.currentState = (g_X27_Vid29.currentState + 1) % g_X27_Vid29.stateCount;
	  //stmPcf8574_MR11_GenApartialStep();//
	  stmX27_Vid29_GenApartialStep();
  }
}

void stmX27_Vid29_stepDown()
{
  if (g_X27_Vid29.currentStep > 0) {
	  g_X27_Vid29.currentStep--;
	  g_X27_Vid29.currentState = (g_X27_Vid29.currentState + 5) % g_X27_Vid29.stateCount;
	  //stmPcf8574_MR11_GenApartialStep();//
	  stmX27_Vid29_GenApartialStep();
  }
}

void stmX27_Vid29_Zeroing()
{
	unsigned int i;
	g_X27_Vid29.currentStep = g_X27_Vid29.maxSteps - 1;
	for (i=0;i<g_X27_Vid29.maxSteps;i++) {
		stmX27_Vid29_stepDown();
		delayms(3);//was 800usec
	}
	g_X27_Vid29.currentStep = 0;
	g_X27_Vid29.targetStep = 0;
	g_X27_Vid29.vel = 0;
	g_X27_Vid29.dir = 0;
}

// 315 degrees of range = 315x3 steps = 945 steps
// declare motor1 with 945 steps on pins 4-7
//SwitecX25 motor1(315*3, 4,5,6,7);
// declare motor2 with 945 steps on pins 8-11
//SwitecX25 motor2(315*3, 8,9,10,11);

// This function determines the speed and accel
// characteristics of the motor.  Ultimately it
// steps the motor once (up or down) and computes
// the delay until the next step.  Because it gets
// called once per step per motor, the calcuations
// here need to be as light-weight as possible, so
// we are avoiding floating-point arithmetic.
//
// To model acceleration we maintain vel, which indirectly represents
// velocity as the number of motor steps traveled under acceleration
// since starting.  This value is used to look up the corresponding
// delay in accelTable.  So from a standing start, vel is incremented
// once each step until it reaches maxVel.  Under deceleration
// vel is decremented once each step until it reaches zero.

void stmX27_Vid29_advanceAstepUpOrDown()
{
  // detect stopped state
  if (g_X27_Vid29.currentStep==g_X27_Vid29.targetStep && g_X27_Vid29.vel==0) {
	  g_X27_Vid29.stopped = true;
	  g_X27_Vid29.dir = 0;
	  g_X27_Vid29.time0 = micros();
	  printf("stopped state\r\n");
    return;
  }

  // if stopped, determine direction
  if (g_X27_Vid29.vel==0) {
	  g_X27_Vid29.dir = g_X27_Vid29.currentStep<g_X27_Vid29.targetStep ? 1 : -1;
    // do not set to 0 or it could go negative in case 2 below
	  g_X27_Vid29.vel = 1;
  }

  if (g_X27_Vid29.dir>0) {
	  stmX27_Vid29_stepUp();
  } else {
	  stmX27_Vid29_stepDown();
  }

  // determine delta, number of steps in current direction to target.
  // may be negative if we are headed away from target
  int delta = g_X27_Vid29.dir>0 ? g_X27_Vid29.targetStep-g_X27_Vid29.currentStep : g_X27_Vid29.currentStep-g_X27_Vid29.targetStep;

  if (delta>0) {
    // case 1 : moving towards target (maybe under accel or decel)
    if (delta < g_X27_Vid29.vel) {
      // time to declerate
    	g_X27_Vid29.vel--;
    } else if (g_X27_Vid29.vel < g_X27_Vid29.maxVel) {
      // accelerating
    	g_X27_Vid29.vel++;
    } else {
      // at full speed - stay there
    }
  } else {
    // case 2 : at or moving away from target (slow down!)
	  g_X27_Vid29.vel--;
  }

  // vel now defines delay
  unsigned char i = 0;
  // this is why vel must not be greater than the last vel in the table.
  while (g_X27_Vid29.accelTable[i][0]<g_X27_Vid29.vel) {
    i++;
  }
  g_X27_Vid29.microDelay = g_X27_Vid29.accelTable[i][1];
  g_X27_Vid29.time0 = micros();
}

void stmX27_Vid29_setPosition(unsigned int pos)
{
  // pos is unsigned so don't need to check for <0
  if (pos >= g_X27_Vid29.maxSteps)
	  pos = g_X27_Vid29.maxSteps-1;
  g_X27_Vid29.targetStep = pos;
  if (g_X27_Vid29.stopped) {
    // reset the timer to avoid possible time overflow giving spurious deltas
	  g_X27_Vid29.stopped = false;
	  g_X27_Vid29.time0 = micros();
    g_X27_Vid29.microDelay = 0;
  }
}
void stmX27_Vid29_MoveTheNeedleAt(unsigned int pos){
	stmX27_Vid29_setPosition(pos);
	stmX27_Vid29_doRotating();
}
/*
void stmX27_Vid29_update()
{
	unsigned long delta;
	if (!g_X27_Vid29.stopped) {
	  delta = micros() - g_X27_Vid29.time0;
	  if (delta >= g_X27_Vid29.microDelay) {
		  stmX27_Vid29_advanceAstepUpOrDown();
    }
  }
}
*/
void stmX27_Vid29_doRotating()
{
	unsigned int i;
	unsigned int deltaSteps;

	if(g_X27_Vid29.currentStep == g_X27_Vid29.targetStep )
		return;
	else if(g_X27_Vid29.currentStep > g_X27_Vid29.targetStep ){//
		deltaSteps = g_X27_Vid29.currentStep - g_X27_Vid29.targetStep;
		for (i=0;i<deltaSteps;i++) {
			stmX27_Vid29_stepDown();
			delayms(3);// delayMicroseconds(RESET_STEP_MICROSEC); //800usec
		}
	}else{
		deltaSteps = g_X27_Vid29.targetStep - g_X27_Vid29.currentStep;
		for (i=0;i<deltaSteps;i++) {
			stmX27_Vid29_stepUp();
			delayms(3);// delayMicroseconds(RESET_STEP_MICROSEC); //800usec
		}
	}
}
/*
//This updateMethod is blocking, it will give you smoother movements, but your application will wait for it to finish
void stmX27_Vid29_updateBlocking()
{
	unsigned long delta;
	while (!g_X27_Vid29.stopped) {
		delta = micros() - g_X27_Vid29.time0;
		if (delta >= g_X27_Vid29.microDelay) {
			stmX27_Vid29_advanceAstepUpOrDown();
		}
	}
}
*/


// For motors connected to digital pins 4,5,6,7

void stmX27_Vid29_Init(unsigned int maxSteps)
{
	int i;

	//Init_SysTick(1000); //1msec tick

	stmStepper_GPIO_setup();

	g_X27_Vid29.currentState = 0;
	g_X27_Vid29.maxSteps = maxSteps;
	g_X27_Vid29.stateCount = 6;
	g_X27_Vid29.dir = 0;
	g_X27_Vid29.vel = 0;
	g_X27_Vid29.stopped = true;
	g_X27_Vid29.currentStep = 0;
	g_X27_Vid29.targetStep = 0;

	g_X27_Vid29.accelTable = X27_Vid29_defaultAccelTable;
	g_X27_Vid29.maxVel = X27_Vid29_defaultAccelTable[DEFAULT_ACCEL_TABLE_SIZE-1][0]; // last value in table.

	// moving the needle at the initial position.
	stmX27_Vid29_Zeroing();
}

void stmX27_Vid29_Loop(void)
{
	static int nextPos = 0;

	stmX27_Vid29_Init(MAXSTEPS);

	//at Zero...
	printf("Zeroing..\r\n");
	stmX27_Vid29_Zeroing();//move to the zero point.
	nextPos = 0;

	// moving the needle at the center of the range in this initial stage.
	printf("Move the needle at the center.\r\n");
	stmX27_Vid29_MoveTheNeedleAt(MAXSTEPS/2);

	printf("We can step at the position from 0 through %d.\r\n",MAXSTEPS-1);

	printf("Zeroing..\r\n");
	stmX27_Vid29_Zeroing();//move to the zero point.

	while(1){

		//c = Serial.read();
		nextPos += 10;
		if (nextPos > (MAXSTEPS-1)) {
			printf("Zeroing..\r\n");
			stmX27_Vid29_Zeroing();//move to the zero point.
			nextPos = 0;
		} else{
			printf("SetPos=%d\r\n",nextPos);
			stmX27_Vid29_MoveTheNeedleAt(nextPos);
		}
		//delayms(3);
	}
}
#endif
