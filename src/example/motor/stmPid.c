/*
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
#include "yInc.h"
*/
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
#include "yInc.h"


void stmPidConf(){

	GPIO_InitTypeDef GPIO_InitStruct;
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
	//PA5 and PA4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an OutputVaue
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	//PB1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an OutputVaue
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

#else
	//PC11 and PC10
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//PC11 and PC10
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an OutputVaue
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

	//PA15
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //PA15
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an OutputVaue
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);//
#endif
	printf("Init Pid.\r\n");

	delayms(1000);

}


/**********************************************************************************************
PID with STM32F103
 **********************************************************************************************/
  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0

  #define DIRECT  0
  #define REVERSE  1

  #define P_ON_M 0
  #define P_ON_E 1

struct Pid{
	double dispKp;				// * we'll hold on to the tuning parameters in user-entered
	double dispKi;				//   format for display purposes
	double dispKd;				//

	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    double myInValue;              //  InValue, OutputVaue, and TargetValue variables
    double myOutputVaue;             //   This creates a hard link between the variables and the
    double myTargetValue;           //   PID, freeing the user from having to constantly tell us
                                 //   what these values are.

	double OutputValSum, lastInValue;

	unsigned long SamplingPeriod_msec;
	double outMin, outMax;
	unsigned char inAuto, pOnE;
	unsigned long now,lastTime;
};

struct Pid g_Pid;

/* Compute() **********************************************************************
 *   this function should be called every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid OutputVaue needs to be computed.  returns true when the OutputVaue is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool stmPid_Compute()
{
	unsigned long elpased_msec;
    double inVal;      //sampling value
    double diff_Target;//ifference between tagret and this sample
    double diff_Samples; //difference between samples (for Kd)
    double outputVal;

   if(!g_Pid.inAuto) return 0;
   g_Pid.now = millis();

   elpased_msec = (g_Pid.now - g_Pid.lastTime);

   if(elpased_msec < g_Pid.SamplingPeriod_msec) return 0;

   inVal = g_Pid.myInValue; //new sampling value
   diff_Target = g_Pid.myTargetValue - inVal; //get the difference between tagret and this sample
   diff_Samples = (inVal - g_Pid.lastInValue); //get the difference between samples (for Kd)

   //for Ki
   g_Pid.OutputValSum+= (g_Pid.ki * diff_Target); //for Ki

   //Add Proportional on Measurement, if P_ON
   if(!g_Pid.pOnE)
	   g_Pid.OutputValSum-= g_Pid.kp * diff_Samples;

   if(g_Pid.OutputValSum > g_Pid.outMax) g_Pid.OutputValSum= g_Pid.outMax;
   else if(g_Pid.OutputValSum < g_Pid.outMin) g_Pid.OutputValSum= g_Pid.outMin;

   //(a) for Kp
   if(g_Pid.pOnE)
	   outputVal = g_Pid.kp * diff_Target;
   else outputVal = 0;

   //(b) Kd portion
   outputVal += g_Pid.OutputValSum - g_Pid.kd * diff_Samples;

   if(outputVal > g_Pid.outMax) outputVal = g_Pid.outMax;
   else if(outputVal < g_Pid.outMin) outputVal = g_Pid.outMin;
   g_Pid.myOutputVaue = outputVal;

   //Save some variables for next time
   g_Pid.lastInValue = inVal;
   g_Pid.lastTime = g_Pid.now;
   return 1;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * tunings can also be adjusted on the fly during normal operation
 ******************************************************************************/
void stmPid_SetParams(double kp, double ki, double kd, int pOn)
{
	double samplingPeriodInSec;

   if (kp<0 || ki<0 || kd<0) return;

   g_Pid.pOn = pOn;
   g_Pid.pOnE = (g_Pid.pOn == 1) ? 1 : 0;

   g_Pid.dispKp = kp; g_Pid.dispKi = ki; g_Pid.dispKd = kd;

   samplingPeriodInSec = ((double)g_Pid.SamplingPeriod_msec)/1000;
   g_Pid.kp = kp;
   g_Pid.ki = ki * samplingPeriodInSec;
   g_Pid.kd = kd / samplingPeriodInSec;

  if(g_Pid.controllerDirection ==REVERSE)
   {
	  g_Pid.kp = (0 - g_Pid.kp);
	  g_Pid.ki = (0 - g_Pid.ki);
	  g_Pid.kd = (0 - g_Pid.kd);
   }
}

/* SetSamplingPeriod_msec(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void stmPid_SetSamplingPeriod_msec(int newSamplingPeriod_msec)
{
   if (newSamplingPeriod_msec > 0)
   {
      double ratio  = (double)newSamplingPeriod_msec
                      / (double)g_Pid.SamplingPeriod_msec;
      g_Pid.ki *= ratio;
      g_Pid.kd /= ratio;
      g_Pid.SamplingPeriod_msec = (unsigned long)newSamplingPeriod_msec;
   }
}

/* SetOutputVaueLimits(...)****************************************************
 *  This function will be used far more often than SetInValueLimits.  while
 *  the InValue to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the OutputVaue will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void stmPid_SetOutputVaueLimits(double min, double max)
{
   if(min >= max) return;
   g_Pid.outMin = min;
   g_Pid.outMax = max;

   if(g_Pid.inAuto)
   {
	   if(g_Pid.myOutputVaue > g_Pid.outMax) g_Pid.myOutputVaue = g_Pid.outMax;
	   else if(g_Pid.myOutputVaue < g_Pid.outMin) g_Pid.myOutputVaue = g_Pid.outMin;

	   //???
	   if(g_Pid.OutputValSum > g_Pid.outMax) g_Pid.OutputValSum= g_Pid.outMax;
	   else if(g_Pid.OutputValSum < g_Pid.outMin) g_Pid.OutputValSum= g_Pid.outMin;
   }
}

void stmPid_Initialize()
{
	g_Pid.OutputValSum = g_Pid.myOutputVaue;
	g_Pid.lastInValue = g_Pid.myInValue;
   if(g_Pid.OutputValSum > g_Pid.outMax)
	   g_Pid.OutputValSum = g_Pid.outMax;
   else if(g_Pid.OutputValSum < g_Pid.outMin)
	   g_Pid.OutputValSum = g_Pid.outMin;
}

void stmPid_SetMode(int mode)
{
    unsigned char newAuto = (mode == AUTOMATIC) ? 1: 0;
    if(newAuto && !g_Pid.inAuto)
    {
        stmPid_Initialize(); //goto AUTO
    }
    g_Pid.inAuto = newAuto;
}

void stmPid_SetControllerDirection(int Direction)
{
   if(g_Pid.inAuto && (Direction !=g_Pid.controllerDirection))
   {
	   g_Pid.kp = (0 - g_Pid.kp);
	   g_Pid.ki = (0 - g_Pid.ki);
	   g_Pid.kd = (0 - g_Pid.kd);
   }
   g_Pid.controllerDirection = Direction;
}

double stmPid_GetKp(){ return  g_Pid.dispKp; }
double stmPid_GetKi(){ return  g_Pid.dispKi;}
double stmPid_GetKd(){ return  g_Pid.dispKd;}
int stmPid_GetMode(){ return  g_Pid.inAuto ? AUTOMATIC : MANUAL;}
int stmPid_GetDirection(){ return g_Pid.controllerDirection;}

stmPid_Config(
 double inValue,
 double targetValue,
 double outputVaue,
 double kp,
 double ki,
 double kd,
 int pOn,
 int samplingPeriod_msec,
 int direction)
{
    g_Pid.myOutputVaue = outputVaue;
    g_Pid.myInValue    = inValue;
    g_Pid.myTargetValue= targetValue;
    g_Pid.inAuto = 0;

    stmPid_SetOutputVaueLimits(0, 100);
    g_Pid.SamplingPeriod_msec = samplingPeriod_msec;

    stmPid_SetControllerDirection(direction);
    stmPid_SetParams(kp, ki, kd, pOn);

    g_Pid.lastTime = millis()-g_Pid.SamplingPeriod_msec;
}

void pidLoop(void)
{
	double targetVal=25,    // 25 degree
	       inVal = 25,   // InValue temp
	       outputVal;     //OutputVaue temp

	double consKp=1, consKi=0.05, consKd=0.25;
	double aggKp=4, aggKi=0.2, aggKd=1;
	double diff ;

	stmPid_Config(inVal, targetVal, outputVal,
			consKp, consKi, consKd,
			1, //pOn
			100, //100msec sampling period
			DIRECT);//Direction - Clockwise

	//g_Pid.myInValue = analogRead();


	//??? stmPid_SetSamplingPeriod_msec();//Why not used.

    stmPid_SetMode(AUTOMATIC);

	while(1){
		//g_Pid.myInValue = analogRead(PIN_InValue);

		double diff = abs(g_Pid.myTargetValue - g_Pid.myInValue);
		if (diff < 10)
		{
			stmPid_SetParams(consKp, consKi, consKd, 1);
		}
		else
		{
			//use aggressive tuning parameters
			stmPid_SetParams(aggKp, aggKi, aggKd, 1);
		}

		stmPid_Compute();

		//analogWrite(PIN_OutputVaue, g_Pid.myOutputVaue);
	}
}
#if 0
/********************************************************
 * PID Adaptive Tuning Example
 * One of the benefits of the PID library is that you can
 * change the tuning parameters at any time.  this can be
 * helpful if we want the controller to be agressive at some
 * times, and conservative at others.   in the example below
 * we set the controller to use Conservative Tuning Parameters
 * when we're near TargetValue and more agressive Tuning
 * Parameters when we're farther away.
 ********************************************************/


#define PIN_InValue 0
#define PIN_OutputVaue 3

//Define Variables we'll be connecting to
double TargetValue, InValue, OutputVaue;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&InValue, &OutputVaue, &TargetValue, consKp, consKi, consKd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  InValue = analogRead(PIN_InValue);
  TargetValue = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  InValue = analogRead(PIN_InValue);

  double gap = abs(TargetValue-InValue); //distance away from TargetValue
  if (gap < 10)
  {  //we're close to TargetValue, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from TargetValue, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
  analogWrite(PIN_OutputVaue, OutputVaue);
}


void stmA3967BipolarMotorLoop(){
	u32 count=0;
	u32 rev = 0;
	u32 steps;

	printf("Bipolar Motor Driver A3967 Test.\r\n");

	stmA3967Conf();

	rev = 1;
	steps = STEPSPERREV * rev;

	while(1){
		stmA3967GoStepsForward(steps);//stmA3967GoSteps(FORWARD,steps);
		delayms(200);

		stmA3967GoStepsBackward(steps);//stmA3967GoSteps(BACKWARD,steps);
		delayms(200);
	}
}
#endif
