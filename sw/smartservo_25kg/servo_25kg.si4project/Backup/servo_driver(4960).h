#ifndef __SERVO_DRIVER_H__
#define __SERVO_DRIVER_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"

#define MP6515  1//10kg
#define MP6528	0//25kg

//GPIO define 
#if  MP6528
#define SMART_SERVO_ENA 			P3_4
#define SMART_SERVO_ENB 			P3_5
#define SMART_SERVO_PWMA    	P2_5
#define SMART_SERVO_PWMB    	P2_6
#define SMART_SERVO_SLEEP   	P3_6
#define SMART_SERVO_NFAULT  	P3_2
#elif MP6515
#define SMART_SERVO_ENBL      P2_2
#define SMART_SERVO_PHASE     P2_3
#define SMART_SERVO_SLEEP     P3_6
#define SMART_SERVO_NFAULT    P3_2
#endif

#define MAX_OUTPUT_PWM		1024
#define MAX_TAR_SPEED  (50)    // ---- 0.16sec/60   (28.57 rpm)    0.200s/60°


typedef enum
{
    IDLE_MODE = 0,	
    SPEED_MODE,		   
    POS_MODE,
    PWM_MODE,
    ERROR_MODE,		    
    BRAKE_MODE,     
    DEBUG_MODE, //6
} MOTOR_CTRL_STATUS;


void servodriver_init(void);
void servodriver_set_pwm(int16_t pwm);
void servodriver_run_idle(void);
void servodriver_run_abs_pos(long angle,float speed);
void servodriver_run_relative_pos(long angle,float speed);
void servodriver_run_speed(float speed);
void servodriver_run_pwm(int16_t pwm);
void servodriver_run_error(void);
void servodriver_run_debug(uint8_t mode,long param1,long param2,long param3);


extern MOTOR_CTRL_STATUS g_eSysMotionStatus;  


#endif/* __SMARTSERVO_H__ */
