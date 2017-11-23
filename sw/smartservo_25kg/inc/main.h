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
#endif


//
//  configuration parameter
//

#endif

