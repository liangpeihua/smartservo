#ifndef __MAIN_H__
#define __MAIN_H__

#define USED_MP9960
// #define USED_MA730

#if defined (USED_MP9960)
	#define RAW_ANGLE_MAX_INT 4096
	#define RAW_ANGLE_MAX_FLOAT 4096
	#define SMART_SERVO_POS_DEADBAND            (12)//(3)
	#define DECELERATION_DISTANCE_PITCH			4//120
	#define DECELERATION_DISTANCE_PITCH_FLOAT	4

	#define SMART_SERVO_TYPE0_AD	  	P1_0	/* smart servo type */
	#define SMART_SERVO_TYPE1_AD	  	P1_2
	#define SMART_SERVO_HW_Verison_D0	P3_4	/* hardware version */
	#define SMART_SERVO_HW_Verison_D1	P3_5
#endif

#if defined (USED_MA730)
	#define RAW_ANGLE_MAX_INT 16384
	#define RAW_ANGLE_MAX_FLOAT 16384.0
	#define SMART_SERVO_POS_DEADBAND            (12)
	// #define DECELERATION_DISTANCE_PITCH			16
	#define DECELERATION_DISTANCE_PITCH_FLOAT	16.0

	#define SMART_SERVO_TYPE0_AD	  	P1_0	/* smart servo type */
	#define SMART_SERVO_TYPE1_AD	  	P1_2
	#define SMART_SERVO_HW_Verison_D0	P3_4	/* hardware version */
	#define SMART_SERVO_HW_Verison_D1	P3_5
#endif

//
//  configuration parameter
//
extern uint16_t limit_pwm;

#endif

