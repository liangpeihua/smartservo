#ifndef __SERVO_DETECT_H__
#define __SERVO_DETECT_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"


typedef struct
{
	bool ready;
	uint16_t voltage;	    //电压,mV
	uint16_t current;	    //总电流,mA
	int16_t cur_speed;				//转速，0.1RPM/min
	int32_t cur_pos;					//位置，0.1°
	uint16_t temperature;

}SERVO_DETECT;


void servodet_init(void);
void servodet_process(void);


extern SERVO_DETECT g_servo_info;


#endif/* __SMARTSERVO_H__ */
