/*
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
#include "yInc.h"
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

#include "yMotor.h"

//#include "math.h"

/* TI's DRV8825 and Allegro's A3967 and A4988 Bipolar DC Stepper Motor
 * Logic = 3.3V
 * Driver = 5V~12V (H1 selectable)

   5V/12V 4 Pins Bipolar Stepper Motor
 * Blk : 1A
 * GRN : 1B
 * BLU : 2A
 * RED : 2B
  */

//+-----------------+-----------+-----------+-----------+---------+---------+
//|                 |401-M34    |401-M35    | 407-M35   | 407-M36 |103 -M35 |
//+-----------------+-----------+-----------+-----------+---------+---------+
//| STEP            |           |           |           | PA15    |PB1*
//+-----------------+-----------+-----------+-----------+---------+---------+
//| DIR             |           |           |           | PC10    |PA5*
//+-----------------+-----------+-----------+-----------+---------+---------+
//| nEN             |           |           |           | PC11    |PA4*
//+-----------------+-----------+-----------+-----------+---------+---------+
//| nSLP            |           |           |           | PC11    |PA6*
//+-----------------+-----------+-----------+-----------+---------+---------+
//*H= Active High/L=Active Low
//*F= FallingEdge

//PIN ASSIGNMENTS
#if (PROCESSOR == PROCESSOR_STM32F103C8T6)  || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_GD32F130FX)

//STEP - PB1
#if 1 //for M81 Board
#define stmSTEPPER_STEP_1      {GPIO_SetBits(GPIOB, GPIO_Pin_1);} //STEP
#define stmSTEPPER_STEP_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_1);}
#else //for M78 Board
#define stmSTEPPER_STEP_1      {GPIO_SetBits(GPIOB, GPIO_Pin_11);} //STEP
#define stmSTEPPER_STEP_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_11);}
#endif
//DIR - PA5
#define stmSTEPPER_DIR_CW      {GPIO_SetBits(GPIOA, GPIO_Pin_5);} //DIR=1
#define stmSTEPPER_DIR_CCW     {GPIO_ResetBits(GPIOA, GPIO_Pin_5);}
//nENABLE - PA4
#define stmSTEPPER_DISABLE     {GPIO_SetBits(GPIOA, GPIO_Pin_4);}
#define stmSTEPPER_nENABLE      {GPIO_ResetBits(GPIOA, GPIO_Pin_4);} //~EN
//nSLEEP - PA6
#define stmSTEPPER_nSLEEP        {GPIO_ResetBits(GPIOA, GPIO_Pin_6);}
#define stmSTEPPER_WAKEUP      {GPIO_SetBits(GPIOA, GPIO_Pin_6);}

#else
#define stmSTEPPER_nDISABLE        {GPIO_SetBits(GPIOC, GPIO_Pin_11);}
#define stmSTEPPER_nENABLE          {GPIO_ResetBits(GPIOC, GPIO_Pin_11);} //~EN

#define stmSTEPPER_DIR_CW        {GPIO_SetBits(GPIOC, GPIO_Pin_10);} //DIR=1
#define stmSTEPPER_DIR_CCW      {GPIO_ResetBits(GPIOC, GPIO_Pin_10);}

#define stmSTEPPER_STEP_1        {GPIO_SetBits(GPIOA, GPIO_Pin_15);} //STEP
#define stmSTEPPER_STEP_0      {GPIO_ResetBits(GPIOA, GPIO_Pin_15);}
#endif


//#define stmSTEPPER_SLEEP_PIN 	6 // optional (just delete SLEEP from everywhere if not used)
static inline void delayMicros(unsigned long delay_us, unsigned long start_us)
{
	if (delay_us){
		if (!start_us){
			start_us = micros(); //NOW
        }
        //if (delay_us > MIN_YIELD_MICROS){
            //    yield();
        //}
        somedelay(10);
        // See https://www.gammon.com.au/millis
        while (micros() - start_us < delay_us);
	}
}

//======== config =================
// Set target motor RPM (1-200 is a reasonable range)
void Stepper_setRPM(struct StepperDriver *pStepper, int rpm){
    if (pStepper->rpm == 0){
    	Stepper_begin(pStepper, rpm, pStepper->microstepMode);
    }
    pStepper->rpm = rpm;
}

float Stepper_getCurrentRPM(struct StepperDriver *pStepper)
{
    return (60.0*1000000L / pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec / pStepper->microstepMode / STEPS_PER_REV);
}
/*
 *
short Stepper_getStepsPerRevolution(struct StepperDriver *pStepper){
    return pStepper->steps_per_rev;
}

// Set microstep, 1=full speed, 32=fine microstepping
short Stepper_setMicrostep(struct StepperDriver *pStepper, short microstepMode)
{
	short ms;
    for (ms=1; ms <= getMaxMicrostep(pStepper); ms<<=1){
        if (microstepMode == ms){
        	pStepper->microstepMode = microstepMode;
            break;
        }
    }
    return pStepper->microstepMode;
}
 short Stepper_getMicrostep(struct StepperDriver *pStepper){
    return pStepper->microstepMode;
}


float Stepper_getRPM(struct StepperDriver *pStepper){
    return pStepper->rpm;
};

struct Profile Stepper_getSpeedProfile(struct StepperDriver *pStepper){
    return pStepper->profile;
}

short Stepper_getAcceleration(struct StepperDriver *pStepper){
    return pStepper->profile.accel;
}
short Stepper_getDeceleration(struct StepperDriver *pStepper){
    return pStepper->profile.decel;
}

// Return calculated time to complete the given move
long Stepper_getTimeForMove(struct StepperDriver *pStepper, long steps);


// Get the number of completed steps so far. * This is always a positive number
long Stepper_getStepsPerRevolutionCompleted(struct StepperDriver *pStepper){
    return pStepper->step_count;
}
// Get the number of steps remaining to complete the move. This is always a positive number
 long Stepper_getStepsPerRevolutionRemaining(struct StepperDriver *pStepper)
 {
    return pStepper->steps_remaining;
}
// Get movement direction: forward +1, back -1
 int Stepper_getDirection(struct StepperDriver *pStepper){
    return (pStepper->dir_state == 1) ? 1 : -1;
}
*/
// Calculate steps needed to rotate requested angle, given in degrees
int Stepper_getStepsForRotation(struct StepperDriver *pStepper, int deg)
{
	int steps;
	steps = deg * STEPS_PER_REV * pStepper->microstepMode / 360;
	printf("Need %d Steps to rotate %d degrees\r\n",steps, deg);
    return steps;
}

 // Set speed profile - CONSTANT_SPEED, LINEAR_SPEED (accelerated). accel and decel are given in [full steps/s^2]
 void Stepper_setSpeedProfile(struct StepperDriver *pStepper,e_Mode mode, short accel, short decel)
 {
 	pStepper->profile.mode = mode;
 	pStepper->profile.accel = accel;//1000;
 	pStepper->profile.decel = decel;//1000;
 }
 e_State Stepper_getCurrentState(struct StepperDriver *pStepper)
 {
     e_State state;
     if (pStepper->steps_remaining <= 0){
         state = STOPPED;
     } else {
         if (pStepper->steps_remaining <= pStepper->steps_to_brake){
         	state = DECELERATING;
         } else if (pStepper->step_count <= pStepper->steps_to_cruise){
         	state = ACCELERATING;
         } else {
         	state = CRUISING;
         }
     }
     return state;
 }

//======================= MOVING ========================
// Set up a new move (calculate and save the parameters)
 void Stepper_startMove(struct StepperDriver *pStepper, long steps, long time){
    float speed_stepsPerSec;
    // set up new move
    pStepper->dir_state = (steps >= 0) ? 1 : 0;
    pStepper->last_action_end = 0;
    pStepper->steps_remaining = labs(steps);
    pStepper->step_count = 0;
    pStepper->rest = 0;

    printf("Stepper_startMove() steps=%d, time=%u\r\n", steps, time);

    switch (pStepper->profile.mode){
    case LINEAR_SPEED:
        // speed is in [steps/s]
    	speed_stepsPerSec = pStepper->rpm * STEPS_PER_REV / 60;
        if (time > 0){
            // Calculate a new speed to finish in the time requested
            float t = time / (1e+6);                  						// convert to seconds
            float d = pStepper->steps_remaining / pStepper->microstepMode;   	// convert to full steps
            float a2 = 1.0 / pStepper->profile.accel + 1.0 / pStepper->profile.decel;
            float sqrt_candidate = t*t - 2 * a2 * d;  //
            if (sqrt_candidate >= 0){
            	speed_stepsPerSec = Y_MIN(speed_stepsPerSec, (t - (float)sqrt(sqrt_candidate)) / a2);
            };
        }
        // how many microstepMode from 0 to target speed
        pStepper->steps_to_cruise = pStepper->microstepMode * (speed_stepsPerSec * speed_stepsPerSec / (2 * pStepper->profile.accel));
        // how many microstepMode are needed from cruise speed to a full stop
        pStepper->steps_to_brake = pStepper->steps_to_cruise * pStepper->profile.accel / pStepper->profile.decel;
        if (pStepper->steps_remaining < pStepper->steps_to_cruise + pStepper->steps_to_brake){
            // cannot reach max speed, will need to brake early
        	pStepper->steps_to_cruise = pStepper->steps_remaining * pStepper->profile.decel / (pStepper->profile.accel + pStepper->profile.decel);
        	pStepper->steps_to_brake = pStepper->steps_remaining - pStepper->steps_to_cruise;
        }
        // Initial pulse (c0) including error correction factor 0.676 [us]
        pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec = (1e+6)*0.676*sqrt(2.0f/pStepper->profile.accel/pStepper->microstepMode);
        // Save cruise timing since we will no longer have the calculated target speed later
        pStepper->cruise_SinlgeStepPeriod_for_given_rpm_in_microsec = 1e+6 / speed_stepsPerSec / pStepper->microstepMode;
        break;

    case CONSTANT_SPEED:
    default:
    	pStepper->steps_to_cruise = 0;
    	pStepper->steps_to_brake = 0;
    	pStepper->cruise_SinlgeStepPeriod_for_given_rpm_in_microsec = SinlgeStepPeriod_for_given_rpm_in_microsec(STEPS_PER_REV, pStepper->microstepMode, pStepper->rpm);
    	pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec = pStepper->cruise_SinlgeStepPeriod_for_given_rpm_in_microsec;

    	//if (time > pStepper->steps_remaining * pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec){
        //	pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec = (float)time / pStepper->steps_remaining;
        //}

    	printf("PeriodPerStep=%d\r\n",pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec);
    	break;
    }
}
// Alter a running move by adding/removing steps. This is a naive implementation and it only works well in CRUISING state
 void Stepper_alterMove(struct StepperDriver *pStepper, long steps){
    switch (Stepper_getCurrentState(pStepper)){
    case ACCELERATING: // this also works but will keep the original speed target
    case CRUISING:
        if (steps >= 0){
        	pStepper->steps_remaining += steps;
        } else {
        	pStepper->steps_remaining = max(pStepper->steps_to_brake, pStepper->steps_remaining+steps);
        };
        break;
    case DECELERATING:
        // would need to start accelerating again -- NOT IMPLEMENTED
        break;
    case STOPPED:
        startMove(pStepper, steps);
        break;
    }
}

// Brake early.
void Stepper_startBrake(struct StepperDriver *pStepper){
    switch (Stepper_getCurrentState(pStepper)){
    case CRUISING:  // this applies to both CONSTANT_SPEED and LINEAR_SPEED modes
    	pStepper->steps_remaining = pStepper->steps_to_brake;
        break;

    case ACCELERATING:
    	pStepper->steps_remaining = pStepper->step_count * pStepper->profile.accel / pStepper->profile.decel;
        break;

    default:
        break; // nothing to do if already stopped or braking
    }
}
// Stop movement immediately and return remaining steps.
 long Stepper_stop(struct StepperDriver *pStepper){
    long retval = pStepper->steps_remaining;
    pStepper->steps_remaining = 0;
    return retval;
}

// Return calculated time to complete the given move
 long Stepper_getTimeForMove(struct StepperDriver *pStepper, long steps){
    float t;
    long cruise_steps;
    float fullStepsPerSec; //speed_in_steps
    if (steps == 0){
        return 0;
    }
    switch (pStepper->profile.mode){
        case LINEAR_SPEED:
        	Stepper_startMove(pStepper, steps, 0);
            cruise_steps = pStepper->steps_remaining - pStepper->steps_to_cruise - pStepper->steps_to_brake;
            fullStepsPerSec = pStepper->rpm * STEPS_PER_REV / 60;   // full steps/s
            t = (cruise_steps / (pStepper->microstepMode * fullStepsPerSec)) +
                sqrt(2.0 * pStepper->steps_to_cruise / pStepper->profile.accel / pStepper->microstepMode) +
                sqrt(2.0 * pStepper->steps_to_brake / pStepper->profile.decel / pStepper->microstepMode);
            t *= (1e+6); // seconds -> micros
            break;
        case CONSTANT_SPEED:
        default:
            t = steps * SinlgeStepPeriod_for_given_rpm_in_microsec(STEPS_PER_REV, pStepper->microstepMode, pStepper->rpm);
    }
    return round(t);
}

// Move the motor an integer number of degrees (360 = full rotation)
 //This has poor precision for small amounts, since step is usually 1.8deg
 void Stepper_startRotate(struct StepperDriver *pStepper, long deg){
	 Stepper_startMove(pStepper, Stepper_getStepsForRotation(pStepper, deg), 0);
}
 /*
// Move the motor with sub-degree precision.
void Stepper_startRotate_double(struct StepperDriver *pStepper, double deg){
	Stepper_startMove(pStepper, calcStepsForRotation(pStepper, deg), 0);
}
*/
// calculate the interval until the next pulse
void Stepper_calcStepPulse(struct StepperDriver *pStepper){
    if (pStepper->steps_remaining <= 0){  // this should not happen, but avoids strange calculations
        return;
    }
    pStepper->steps_remaining--;
    pStepper->step_count++;

    if (pStepper->profile.mode == LINEAR_SPEED){
        switch (Stepper_getCurrentState(pStepper)){
        case ACCELERATING:
            if (pStepper->step_count < pStepper->steps_to_cruise){
            	pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec = pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec - (2*pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec+ pStepper->rest)/(4*pStepper->step_count+1);
            	pStepper->rest = (pStepper->step_count < pStepper->steps_to_cruise) ? (2*pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec+pStepper->rest) % (4*pStepper->step_count+1) : 0;
            } else {
                // The series approximates target, set the final value to what it should be instead
            	pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec = pStepper->cruise_SinlgeStepPeriod_for_given_rpm_in_microsec;
            }
            break;

        case DECELERATING:
        	pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec = pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec - (2*pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec+pStepper->rest)/(-4*pStepper->steps_remaining+1);
        	pStepper->rest = (2*pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec+pStepper->rest) % (-4*pStepper->steps_remaining+1);
            break;

        default:
            break; // no speed changes
        }
    }
}
//Yield to step control.  Toggle step and return time until next change is needed (micros)
long Stepper_nextAction(struct StepperDriver *pStepper){
	unsigned m;
	unsigned long l_pulse;
    if (pStepper->steps_remaining > 0)
    {
        delayMicros(pStepper->next_action_interval,//duration
        		pStepper->last_action_end); //from
        //if((pStepper->next_action_interval - (micros() - pStepper->last_action_end)) > 0){
        //	 delayus(pStepper->next_action_interval - (micros() - pStepper->last_action_end));
        //}

        //(a) DIR pin is sampled on rising STEP edge, so it is set first
    	if( pStepper->dir_state ){
    		stmSTEPPER_DIR_CW;        //digitalWrite(dir_pin, dir_state);
    	}else{
    		stmSTEPPER_DIR_CCW;
    	}
        //      +--
    	//(b) --+
    	stmSTEPPER_STEP_1;        //digitalWrite(step_pin, HIGH);
        m = micros(); //get Now.
        l_pulse = pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec; // save value because calcStepPulse() will overwrite it
        Stepper_calcStepPulse(pStepper);

        // We should pull HIGH for at least 1-2us (step_high_min)
        delayus(pStepper->step_high_min);//delayMicros(pStepper->step_high_min, 0);
        // ----+
        //     +----
        stmSTEPPER_STEP_0;//digitalWrite(step_pin, LOW);

        // account for calcStepPulse() execution time; sets ceiling for max rpm on slower MCUs
        pStepper->last_action_end = micros(); //now
        m = pStepper->last_action_end - m;
        pStepper->next_action_interval = (l_pulse > m) ? l_pulse - m : 1;
    } else {
        // end of move
    	pStepper->last_action_end = 0;
    	pStepper->next_action_interval = 0;
    }
    //printf("l_pulse=%u, m=%u, next_action_interval=%u\r\n", l_pulse, m, pStepper->next_action_interval);
    return pStepper->next_action_interval;
}

// Move the motor a given number of steps. positive to move forward, negative to reverse
 void Stepper_Rotate_Steps(struct StepperDriver *pStepper, long steps)
 {
	Stepper_startMove(pStepper, steps, 0);
    while (Stepper_nextAction(pStepper));
}
// Rotate the motor a given number of degrees (1-360)
void Stepper_rotate_Degree(struct StepperDriver *pStepper, int deg)
{
	long steps;
	steps = Stepper_getStepsForRotation(pStepper, deg);
	Stepper_Rotate_Steps(pStepper, steps);
}

//======================================
void Stepper_Init_and_Config(
		struct StepperDriver *pStepper,
		int   rpm,
		short microstepMode,
		short max_microstep)
{
	stmSTEPPER_WAKEUP;
	stmSTEPPER_DISABLE;
	printf("Stepper: Wakeup, and Disable\r\n");

	pStepper->microstepMode = microstepMode;
	pStepper->rpm = rpm;
	pStepper->step_high_min = 100;    // STEP minimum, HIGH pulse width (was 1us)
	pStepper->step_low_min = 100;     // STEP minimum, LOW pulse width (was 1us)
	pStepper->wakeup_time= 1000;    // wakeup time, nSLEEP inactive to STEP (1000us)
	    						    // also 200ns between ENBL/DIR/MSx changes and STEP HIGH

	pStepper->max_microstep = max_microstep;// = 16;// microstep range (1, 16, 32 etc)
 	pStepper->profile.mode = CONSTANT_SPEED;//LINEAR_SPEED;//
 	pStepper->profile.accel = 1000;
 	pStepper->profile.decel = 1000;

	pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec = SinlgeStepPeriod_for_given_rpm_in_microsec(STEPS_PER_REV, pStepper->microstepMode, pStepper->rpm);
	pStepper->cruise_SinlgeStepPeriod_for_given_rpm_in_microsec = pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec;

	pStepper->steps_to_cruise = 0;
	pStepper->steps_remaining = 0;
	pStepper->dir_state = 0;
	pStepper->steps_to_brake = 0;
	pStepper->rest = 0;
	pStepper->step_count = 0;
	pStepper->next_action_interval = 0;

	printf("RPM=%u, StepPeriod=%u usec\r\n", pStepper->rpm, pStepper->SinlgeStepPeriod_for_given_rpm_in_microsec);

    //modify additionally
	pStepper->step_high_min = 200;// pulse duration, STEP high, min value (1.9us)
	pStepper->step_low_min = 200;	// pulse duration, STEP low, min value (1.9us)
	pStepper->wakeup_time = 1700; //wakeup time, nSLEEP inactive to STEP (1000us) and 650ns between ENBL/DIR/MODEx changes and STEP HIGH

	// if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
    stmSTEPPER_nENABLE;
    delayus(2);

    printf("Stepper_Init_and_Config Done\r\n");
    delayms(100);

    return pStepper;
}

void Stepper_GpioConf()
{
	GPIO_InitTypeDef GPIO_InitStruct;
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT) || (PROCESSOR == PROCESSOR_GD32F130FX))
	//| STEP            |           |           |           | PA15    |PB1*
	//+-----------------+-----------+-----------+-----------+---------+---------+
	//| DIR             |           |           |           | PC10    |PA5*
	//+-----------------+-----------+-----------+-----------+---------+---------+
	//| nEN             |           |           |           | PC11    |PA4*
	//+-----------------+-----------+-----------+-----------+---------+---------+
	//| nSLP            |           |           |           | PC11    |PA6*
	//PA5(DIR) , PA4(nEN), PA6(nSLEEP)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOA, &GPIO_InitStruct);//

	//PB1 for STEP
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1; //for M81
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_Init(GPIOB, &GPIO_InitStruct);//

#else
	//PC11 and PC10
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//PC11 and PC10
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOC, &GPIO_InitStruct);//

	//PA15
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //PA15
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStruct);//
#endif
	printf("Config Stepper: PA4=nEN, PA5=DIR, PA6=nSLP, PB1=STEP.\r\n");
}
/*
//Basic connection: only DIR, STEP are connected.
//Microstepping controls should be hardwired.
//A4988_Init(short steps, short dir_pin, short step_pin);
//A4988_Init(short steps, short dir_pin, short step_pin, short enable_pin);
//A4988(short steps, short dir_pin, short step_pin);//, short ms1_pin, short ms2_pin, short ms3_pin);
//A4988(short steps, short dir_pin, short step_pin, short enable_pin);//, short ms1_pin, short ms2_pin, short ms3_pin);

//void A4988_begin(float rpm, short microstepMode);
//void A4988_begin(float rpm=60, short microstepMode=1);
//short A4988_setMicrostep(short microstepMode);

const unsigned char* A4988_getMicrostepTable(struct StepperDriver *pStepper){
    return A4988_MS_TABLE;
}

size_t A4988_getMicrostepTableSize(struct StepperDriver *pStepper){
    return sizeof(A4988_MS_TABLE);
}

short A4988_getMaxMicrostep(struct StepperDriver *pStepper){
    return pStepper->max_microstep;//A4988_MAX_MICROSTEP;
}

// Set microstepping mode (1:divisor).  Allowed ranges for A4988 are 1:1 to 1:16
 short A4988_setMicrostep(struct StepperDriver *pStepper, short microstepMode)
{
	const unsigned char* ms_table;
	size_t ms_table_size;
    pStepper->microstepMode = microstepMode;//Stepper_setMicrostep(pStepper,microstepMode);

    ms_table = A4988_MS_TABLE; //A4988_getMicrostepTable(pStepper);
    ms_table_size = sizeof(A4988_MS_TABLE);//A4988_getMicrostepTableSize(pStepper);

    unsigned short i = 0;
    while (i < ms_table_size){
        if (pStepper->microstepMode & (1<<i)){
            unsigned char mask = ms_table[i];
            //digitalWrite(ms3_pin, mask & 4);
            //digitalWrite(ms2_pin, mask & 2);
            //digitalWrite(ms1_pin, mask & 1);
            break;
        }
        i++;
    }
    return pStepper->microstepMode;
}
*/

#if (MOTOR_FOR == MOTOR_FOR_A3967)
//A3967
void stmA3967BipolarMotorLoop(){
	u32 count=0;
	u32 rev = 0;
	u32 steps;

	struct StepperDriver stepper, *pStepper;
	long long_journey_steps;

	printf("Bipolar Motor Driver A3967 Test.\r\n");

	 //install systick if not installed in the main.
	 //Init_SysTick(RESOLUTION_IN_USEC);// 100 = 100usec; //1000= 1msec Tick; //10000= 10msec Tick Init_SysTick(10000);// 10msec Tick

	pStepper = &stepper;

	//PIN ASSIGN
	//DIR, STEP, SLEEP, EN, (MS1, MS2, MS3)
	Stepper_GpioConf();
	printf("GPIO Config Done\r\n");

	stmSTEPPER_nDISABLE; //PA4 -> 0

    Stepper_Init_and_Config(pStepper,
			RPM, 	// Set target motor RPM.
			1, 		//microstepMode == FULLSTEP
			16);    //MaxmicrostepMode
	// if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);

    stmSTEPPER_nENABLE; //PA4 -> 1

    delayus(2);

    printf("Stepper_Init_and_Config Done\r\n");

    delayms(100);

    while(1){
    	//(a-1)Moving... One complete revolution is 360°
    	printf("Ratate 360\r\n");
    	Stepper_rotate_Degree(pStepper,360);     // forward revolution
    	printf("Reverse Ratate 360\r\n");
    	Stepper_rotate_Degree(pStepper,-360);    // reverse revolution

    	//(a-2) One complete revolution is also steps_per_rev steps in full step mode
    	Stepper_Rotate_Steps(pStepper, STEPS_PER_REV);    // forward revolution
    	Stepper_Rotate_Steps(pStepper, -STEPS_PER_REV);   // reverse revolution

    	//(a-3) 10 revolutions.
    	long_journey_steps = STEPS_PER_REV*5;
    	printf("For 5 rotations, It takes %u sec\r\n", Stepper_getTimeForMove(pStepper, long_journey_steps));
    	Stepper_Rotate_Steps(pStepper, STEPS_PER_REV*5);

    	//reverse
    	printf("For 5 rotations, It takes %u microsec\r\n", Stepper_getTimeForMove(pStepper, long_journey_steps));
    	Stepper_Rotate_Steps(pStepper, -STEPS_PER_REV*5);


    	delayms(1000);
    }

#if 0
    //(b) microstepping...
   Stepper_Init_and_Config(pStepper,
   			RPM, 			// Set target motor RPM.
   			8); 			//microstepMode == 1/8 STEP

    while(1{)

     //(b-1) In 1:8 microstepping mode, one revolution takes 8 times as many microstepMode
     Stepper_Rotate_Steps(pStepper,8 * STEPS_PER_REV);    // forward revolution
     Stepper_Rotate_Steps(pStepper,-8 * STEPS_PER_REV);   // reverse revolution

     // One complete revolution is still 360° regardless of microstepping mode
     //(b-2) rotate() is easier to use than move() when no need to land on precise microstep position
     Stepper_rotate_Degree(pStepper,360);
     Stepper_rotate_Degree(pStepper,-360);

     delayms(5000);
    }
#endif
}

//===A4988 ====================
/* Basic connection: only DIR, STEP are connected.
 * Microstepping controls should be hardwired.
 */
#elif (MOTOR_FOR == MOTOR_FOR_A4988)
// Indexer mode only.
void A4988_loop() {
	 //struct A4988 a4988;
	 struct StepperDriver stepper, *pStepper;
	 long long_journey_steps;

	 //install systick if not installed in the main.
	 //Init_SysTick(RESOLUTION_IN_USEC);// 100 = 100usec; //1000= 1msec Tick; //10000= 10msec Tick Init_SysTick(10000);// 10msec Tick

	 pStepper = &stepper;

	//PIN ASSIGN
	//DIR, STEP, SLEEP, EN, (MS1, MS2, MS3)
	Stepper_GpioConf();
	printf("GPIO Config Done\r\n");

	stmSTEPPER_nDISABLE;
    Stepper_Init_and_Config(pStepper,
			RPM, 	// Set target motor RPM.
			1, 		//microstepMode == FULLSTEP
			16);    //MaxmicrostepMode
	// if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
    stmSTEPPER_nENABLE;
    delayus(2);// delayMicros(2, 0);

    printf("Stepper_Init_and_Config Done\r\n");

    delayms(100);

    while(1){

    	printf("Ratate 360\r\n");
    	//(a-1)Moving... One complete revolution is 360°
    	Stepper_rotate_Degree(pStepper,360);     // forward revolution
    	printf("Reverse Ratate 360\r\n");
    	Stepper_rotate_Degree(pStepper,-360);    // reverse revolution

    	//(a-2) One complete revolution is also steps_per_rev steps in full step mode
    	Stepper_Rotate_Steps(pStepper, STEPS_PER_REV);    // forward revolution
    	Stepper_Rotate_Steps(pStepper, -STEPS_PER_REV);   // reverse revolution

    	//(a-3) 10 revolutions.
    	long_journey_steps = STEPS_PER_REV*10;
    	printf("For 10 rotations, It takes %u sec\r\n", Stepper_getTimeForMove(pStepper, long_journey_steps));
    	Stepper_Rotate_Steps(pStepper, STEPS_PER_REV*10);

    	//reverse
    	printf("For 10 rotations, It takes %u microsec\r\n", Stepper_getTimeForMove(pStepper, long_journey_steps));
    	Stepper_Rotate_Steps(pStepper, -STEPS_PER_REV*10);


    	delayms(1000);
    }
#if 0
    //(b) microstepping...
   Stepper_Init_and_Config(pStepper,
   			RPM, 			// Set target motor RPM.
   			8); 			//microstepMode == 1/8 STEP

    while(1{)

     //(b-1) In 1:8 microstepping mode, one revolution takes 8 times as many microstepMode
     Stepper_Rotate_Steps(pStepper,8 * STEPS_PER_REV);    // forward revolution
     Stepper_Rotate_Steps(pStepper,-8 * STEPS_PER_REV);   // reverse revolution

     // One complete revolution is still 360° regardless of microstepping mode
     //(b-2) rotate() is easier to use than move() when no need to land on precise microstep position
     Stepper_rotate_Degree(pStepper,360);
     Stepper_rotate_Degree(pStepper,-360);

     delayms(5000);
    }
#endif
 }
#elif (MOTOR_FOR == MOTOR_FOR_DRV8825)
void DRV8825Stepper_loop() {

	 struct StepperDriver stepper, *pStepper;
	 long long_journey_steps;

	 //install systick if not installed in the main.
	 //Init_SysTick(RESOLUTION_IN_USEC);// 100 = 100usec; //1000= 1msec Tick; //10000= 10msec Tick Init_SysTick(10000);// 10msec Tick

	Stepper_GpioConf();//DIR, STEP, SLEEP, EN, (MS1, MS2, MS3)

	pStepper = &stepper;

	Stepper_Init_and_Config(
			&stepper,
			RPM, 	// Set target motor RPM.
			1, 		//microstepMode == FULLSTEP
			32);    //MaxmicrostepMode

    while(1){

    	printf("Ratate 360\r\n");
    	//(a-1)Moving... One complete revolution is 360°
    	Stepper_rotate_Degree(pStepper,360);     // forward revolution

    	delayms(1000);

    	printf("Reverse Ratate 360\r\n");
    	Stepper_rotate_Degree(pStepper,-360);    // reverse revolution

    	delayms(1000);
    	//(a-2) One complete revolution is also STEPS_PER_REV steps in full step mode
    	Stepper_Rotate_Steps(pStepper, STEPS_PER_REV);    // forward revolution
    	delayms(1000);
    	Stepper_Rotate_Steps(pStepper, -STEPS_PER_REV);   // reverse revolution
    	delayms(1000);

    	//(a-3) 10 revolutions.
    	long_journey_steps = STEPS_PER_REV*10;
    	printf("For 10 rotations, It takes %u microsec\r\n", Stepper_getTimeForMove(pStepper, long_journey_steps));
    	Stepper_Rotate_Steps(pStepper, STEPS_PER_REV*10);
    	delayms(1000);
    	//reverse
    	printf("For 10 rotations, It takes %u microsec\r\n", Stepper_getTimeForMove(pStepper, long_journey_steps));
    	Stepper_Rotate_Steps(pStepper, -STEPS_PER_REV*10);
    	delayms(1000);
    }

#if 0
    //(b) microstepping...
    Stepper_Init_and_Config(pStepper,
    			RPM, 			// Set target motor RPM.
    			8); 			//microstepMode == 1/8 STEP

    while(1){
     //(b-1) In 1:8 microstepping mode, one revolution takes 8 times as many microstepMode
     Stepper_Rotate_Steps(pStepper,8 * STEPS_PER_REV);    // forward revolution
     Stepper_Rotate_Steps(pStepper,-8 * STEPS_PER_REV);   // reverse revolution

     // One complete revolution is still 360° regardless of microstepping mode
     //(b-2) rotate() is easier to use than move() when no need to land on precise microstep position
     Stepper_rotate_Degree(pStepper,360);
     Stepper_rotate_Degree(pStepper,-360);

     delayms(5000);
    }
#endif
 }

#endif

#if 0

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int step=0,durum=0,sayac=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		sayac++;
		if(sayac == step)
		{
			HAL_TIM_PWM_Stop_IT(&htim1,TIM_CHANNEL_1);	// PWM'i durdur.
			sayac=0;
			durum=0;
		}
	}
}

void Step(int adim ,int yon)
{
	step=adim;
	if(yon==0)
		HAL_GPIO_WritePin(GPIOA,DIR_Pin,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOA,DIR_Pin,GPIO_PIN_SET);
	durum=1;
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
	while(1)
	{
		if(durum==0)
			break;
		HAL_Delay(1);
	}

}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 217;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  HAL_TIM_MspPostInit(&htim1);

}

int main(void)
{

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();

  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 128); // Duty cycle %50 olarak ayarlandi.
	HAL_Delay(100);


  /* Infinite loop */
  while (1)
  {
		Step(200,0);
		HAL_Delay(1000);
		Step(400,1);
		HAL_Delay(1000);
  }

}

//===============
void stmA4988GoSteps (u8 dir, u32 nStep){
	u32 i;
	u16 highdur=4;
	u16 lowdur=4;

	//Set Dir
	if(dir){
		printf("Clockwise...\r\n");
		stmSTEPPER_DIR_CW
	}else{
		printf("CountClockwise...\r\n");
		stmSTEPPER_DIR_CCW
	}
	delayms(10);

	stmSTEPPER_nENABLE;
	delayus(2);//delayMicros(2, 0); //nEN =0

	//for loop steps
	for(i=0;i<nStep;i++){
		//make step pulse
		stmSTEPPER_STEP_0
		delayms(lowdur);
		stmSTEPPER_STEP_1
		delayms(highdur);
	}
	stmSTEPPER_STEP_0
	stmSTEPPER_nDISABLE //nEN =1
}

void stmA4988GoStepsForward(u32 nStep){
	stmA4988GoSteps(FORWARD,nStep);
}
void stmA4988GoStepsBackward(u32 nStep){
	stmA4988GoSteps(BACKWARD,nStep);
}

void stmA4988BipolarMotorLoop(){
	u32 count=0;
	u32 rev = 0;
	u32 steps;

	printf("Bipolar Motor Driver A3967 Test.\r\n");

	stmA4988Conf();

	rev = 1;
	steps = STEPSPERREV * rev;

	while(1){
		stmA4988GoStepsForward(steps);//stmA4988GoSteps(FORWARD,steps);
		delayms(200);

		stmA4988GoStepsBackward(steps);//stmA4988GoSteps(BACKWARD,steps);
		delayms(200);
	}
}


#endif
