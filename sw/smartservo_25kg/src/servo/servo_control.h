/**
 * @file    servo_control.h
 * @author  Payton
 * @version V1.0.0
 * @date    2017/11/17
 * @brief   
 *
 * \par Description
 * This file is servo motion control algorithm.
 *
 * \par History:
 * <pre>
 * `<Author>`        `<Time>`         `<Version>`        `<Descr>`
 * Payton            2017/11/17         1.0.0            create
 * </pre>
 *
 */
#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__
#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"


//死区限制
#define LIMIT_DEATH(x, a)	if( (((x) > 0) && ((x) < (a))) ) (x) = 0;\
                          else if((x) >= (a)) (x) = (x) - (a);\
                          else if( (((x) <= 0) && ((x) >= -(a))) ) (x) = 0;	\
                          else (x) = (x) + (a);

//对数函数，a为底数
#define logab(a,b)    (log(b)/log(a))

typedef struct
{
  int32_t Kp;
  int32_t Ki;
  int32_t Kd;
  int32_t integral;
  int32_t max_integral;
  int32_t last_error;
  int32_t output; 
  int32_t max_output;
}STRUCT_PID;



void motor_process(void);

#endif/* __SMARTSERVO_H__ */
