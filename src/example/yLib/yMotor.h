#ifndef __YMOTOR_H
#define __YMOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif

//#include "yInc.h"


 //Motor Control data
#define MOTOR_CMD_ROTATE_CW  1
#define MOTOR_CMD_ROTATE_CCW 2
#define MotorSpeedUP 		0x03
#define MotorSpeedDOWN 		0x04

#define FORWARD 		1
#define BACKWARD 		0

//#define RPM (60*30) //30Hz
//#define RPM (60*20) //20Hz
//#define RPM (60*10) //10Hz
#define RPM (60*5) 	  //5Hz -- 300PRM (for NEMA17)
//#define RPM (60*1)  //1Hz
//#define RPM (60*50) //50Hz
// Steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
//# of steps per revolution (18 degree/step --> 20 steps for 360 degree.)
//Micro mini stepper
#define STEPS_PER_REV_NMB_MAT_PM15S 20 //20 Steps/Rev;    18 degrees/step; 12V
#define STEPS_PER_REV_SP42RD        48 //48 Steps/Rev, 7.5 degrees/step; 24V//SP42RD -- max 10Hz...
#define STEPS_PER_REV_J40G37        48 //48 Steps/Rev, 7.5 degrees/step; 24V
#define STEPS_PER_REV_NEMA17        200 //200 Steps/Rev, 1.8 degrees/step; 12V

//#define STEPS_PER_REV STEPS_PER_REV_SP42RD
//#define STEPS_PER_REV STEPS_PER_REV_NMB_MAT_PM15S //for DVD ROM
#define STEPS_PER_REV STEPS_PER_REV_NEMA17 //for NEMA17

// Microstepping resolution truth table
//#define MAX_MICROSTEP 16 //A4988
//#define MAX_MICROSTEP 32 //32=DRV8825

/* calculate the step period in microseconds for a given rpm value.
 * 60[s/min] * 1000000[us/s] / microstepMode / steps / rpm
 */
#define SinlgeStepPeriod_for_given_rpm_in_microsec(steps, microstepMode, rpm) (60.0*1000000L/steps/microstepMode/rpm)
//5000 usec for 20 steps/rev and 600 rpm, Mode =1.
//const unsigned char A4988_MS_TABLE[] = {0b000, 0b001, 0b010, 0b011, 0b111};

typedef enum {CONSTANT_SPEED, LINEAR_SPEED} e_Mode;
typedef enum {STOPPED, ACCELERATING, CRUISING, DECELERATING} e_State;

struct CurrentProfile {
        e_Mode mode;// = CONSTANT_SPEED;
        short accel;// = 1000;     // acceleration [steps/s^2]
        short decel;// = 1000;     // deceleration [steps/s^2]
};

struct StepperDriver
{
    long rest;
    unsigned long 	last_action_end;
    unsigned long 	next_action_interval;
    short 			microstepMode;// = 1;    // current microstep level (1,2,4,8,...), must be < getMaxMicrostep()
    short 			max_microstep;//MAX_MICROSTEP;// = 128;// microstep range (1, 16, 32 etc)
    short 			enable_active_state;// = HIGH;
    int 			step_high_min;// = 1;    // (STEP) pulse duration, STEP high, min value (us)
    int 			step_low_min;// = 1;    // (STEP) pulse duration, STEP low, min value (us)
    int 			wakeup_time;// = 0;    // WAKE wakeup time, nSLEEP inactive to STEP (us)

    int rpm;//was float

    //Movement state
    struct CurrentProfile profile;

    long step_count;        // current position
    long steps_remaining;   // to complete the current move (absolute value)
    long steps_to_cruise;   // steps to reach cruising (max) rpm
    long steps_to_brake;    // steps needed to come to a full stop
    long SinlgeStepPeriod_for_given_rpm_in_microsec;        // step pulse duration (microseconds)
    long cruise_SinlgeStepPeriod_for_given_rpm_in_microsec; // step pulse duration for constant speed section (max rpm)

    // DIR pin state
    short dir_state;
};
#endif
