#ifndef __MAIN_H__
#define __MAIN_H__

#define USED_MP9960
// #define USED_MA730

#if defined (USED_MP9960)
	#define RAW_ANGLE_MAX_INT 4096
	#define RAW_ANGLE_MAX_FLOAT 4096.0
	#define SMART_SERVO_POS_DEADBAND            (12)//(3)
	#define DECELERATION_DISTANCE_PITCH			4//120
	#define DECELERATION_DISTANCE_PITCH_FLOAT	4.0
#endif

#if defined (USED_MA730)
	#define RAW_ANGLE_MAX_INT 16384
	#define RAW_ANGLE_MAX_FLOAT 16384.0
	#define SMART_SERVO_POS_DEADBAND            (12)
	// #define DECELERATION_DISTANCE_PITCH			16
	#define DECELERATION_DISTANCE_PITCH_FLOAT	16.0
#endif

//
//  configuration parameter
//
extern uint16_t limit_pwm;

#endif

