#ifndef __SERVO_DRIVER_H__
#define __SERVO_PROTECT_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"

#define MAX_OUTPUT_PWM		255


typedef enum
{
    IDLE_MODE = 0,	
    SPEED_MODE,		   
    POS_MODE,
    PWM_MODE,
    ERROR_MODE,		    
    BRAKE_MODE,     
    DEBUG_MODE,
} MotionStatus;


void servodriver_init(void);
int16_t servodriver_get_nfault_value(void);
void servodriver_set_pwm(int16_t pwm);
void servodriver_run_idle(void);
void servodriver_run_abs_pos(long angle,float speed);
void servodriver_run_relative_pos(long angle,float speed);
void servodriver_run_speed(float speed);
void servodriver_run_pwm(int16_t pwm);
void servodriver_run_error(void);
void servodriver_run_debug(long angle,float speed);


extern MotionStatus g_eSysMotionStatus;  


#endif/* __SMARTSERVO_H__ */
