/**
 * @file    servo_driver.h
 * @author  Payton
 * @version V1.0.0
 * @date    2017/11/17
 * @brief   
 *
 * \par Description
 * This file is servo driver.
 *
 * \par History:
 * <pre>
 * `<Author>`        `<Time>`         `<Version>`        `<Descr>`
 * Payton            2017/11/17         1.0.0            create
 * </pre>
 *
 */
#ifndef __SERVO_DRIVER_H__
#define __SERVO_DRIVER_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"

#define MP6515  0//10kg
#define MP6528	1//25kg

//speed dir
#define SPEED_DIR   1 //0-顺时针为正，1-逆时针为正，

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

//运动控制模式
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
void servodriver_run_idle(void);
void servodriver_run_abs_pos(int32_t angle,float speed);
void servodriver_run_relative_pos(int32_t angle,float speed);
void servodriver_run_speed(float speed);
void servodriver_run_pwm(int16_t pwm);
void servodriver_run_abspos_torque(int32_t angle,float speed,int32_t torque);
void servodriver_run_relativepos_torque(int32_t angle,float speed,int32_t torque);
void servodriver_run_error(void);
void servodriver_run_debug(uint8_t mode,int32_t param1,int32_t param2,int32_t param3);
int16_t servodriver_get_pwmvalue(void);
int32_t servodriver_limitcurrent_protect(int32_t cur_current, int32_t limit_current,int32_t set_pwm);
void servodriver_set_limitcurrent(int32_t limit_current);
void servodriver_set_pwm(int16_t pwm);




extern MOTOR_CTRL_STATUS g_motion_status;  


#endif/* __SMARTSERVO_H__ */
