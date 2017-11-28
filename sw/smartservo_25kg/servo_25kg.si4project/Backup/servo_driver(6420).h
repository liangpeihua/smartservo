#ifndef __SERVO_DRIVER_H__
#define __SERVO_DRIVER_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"

#define MP6515  0//10kg
#define MP6528	1//25kg

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
#define MAX_TAR_SPEED 	 (60)    // ---- 0.16sec/60   (28.57 rpm)    0.200s/60°
#define MAX_TORQUE				255

typedef enum
{
    IDLE_MODE = 0,	
    PWM_MODE,
    SPEED_MODE,		   
    POS_MODE,
    TORQUE_MODE,  
    ERROR_MODE,		       
    DEBUG_MODE, //6
} MOTOR_CTRL_STATUS;


void servodriver_init(void);
void servodriver_set_pwm(int16_t pwm);
void servodriver_run_idle(void);
void servodriver_run_abs_pos(int32_t angle,float speed);
void servodriver_run_relative_pos(int32_t angle,float speed);
void servodriver_run_speed(float speed);
void servodriver_run_pwm(int16_t pwm);
void servodriver_run_torque(int32_t angle,float speed,int32_t torque);
void servodriver_run_error(void);
void servodriver_run_debug(uint8_t mode,int32_t param1,int32_t param2,int32_t param3);
int16_t servodriver_getpwmvalue(void);



extern MOTOR_CTRL_STATUS g_eSysMotionStatus;  


#endif/* __SMARTSERVO_H__ */
