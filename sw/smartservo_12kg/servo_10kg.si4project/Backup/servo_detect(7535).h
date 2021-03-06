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
	uint32_t current;	    //总电流,mA
	int32_t cur_speed;				//转速，1RPM/min
	int32_t cur_pos;					//位置，0.1°
	int32_t circular_count;
	int32_t tar_speed;				//转速，1RPM/min
	int32_t tar_pos;					//位置，0.1°
	int32_t tar_pwm;
	int32_t angle_zero_offset;
	int32_t current_zero_offset;
	uint32_t temperature;
	int32_t limit_pwm;
	uint32_t errorid;
}SERVO_DETECT;


void servodet_init(void);
void servodet_process(void);
SpiFlashOpResult servodet_set_angle_zero(void);


extern SERVO_DETECT g_servo_info;


#endif/* __SMARTSERVO_H__ */
