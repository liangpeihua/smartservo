/**
 * @file    servo_detect.h
 * @author  Payton
 * @version V1.0.0
 * @date    2017/11/17
 * @brief   
 *
 * \par Description
 * This file is servo detect.
 *
 * \par History:
 * <pre>
 * `<Author>`        `<Time>`         `<Version>`        `<Descr>`
 * Payton            2017/11/17         1.0.0            create
 * </pre>
 *
 */
#ifndef __SERVO_DETECT_H__
#define __SERVO_DETECT_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"
#include <stdlib.h>
#include <stdio.h>

typedef struct
{
  boolean ready;
  uint32_t voltage;	    //电压,mV
  int32_t current;	    //总电,mA
  int32_t cur_speed;				//转速,1RPM/min
  int32_t cur_pos;					//位置,1°
  int32_t circular_count;
  int32_t tar_speed;				//转速,1RPM/min
  int32_t tar_pos;					//位置,1°
  int32_t tar_pwm;
  int32_t tar_torque;				//4mA
  int32_t angle_zero_offset;
  int32_t current_zero_offset;
  int32_t temperature;
  int32_t limit_pwm;
  int32_t over_current;
  uint32_t errorid;
  int32_t posmode_tarspeed;		
  boolean reach_tar_pos;
}SERVO_DETECT;


void servodet_init(void);
void servodet_process(void);
SpiFlashOpResult servodet_set_angle_zero(void);


extern SERVO_DETECT g_servo_info;


#endif/* __SMARTSERVO_H__ */
