#ifndef __SMARTSERVO_H__
#define __SMARTSERVO_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"

typedef enum
{
  SMART_SERVO_DIR_CW = 0,    ///< Clockwise
  SMART_SERVO_DIR_CCW  = 1   ///< Counter-Clockwise
} smart_servo_dir;

//GPIO define
#define SMART_SERVO_LED_B     	P2_4
#define SMART_SERVO_LED_G     	P2_3
#define SMART_SERVO_LED_R     	P2_2

#define SMART_SERVO_ENA 		P3_4
#define SMART_SERVO_ENB 		P3_5
#define SMART_SERVO_PWMA    	P2_5
#define SMART_SERVO_PWMB    	P2_6
#define SMART_SERVO_SLEEP   	P3_6
#define SMART_SERVO_NFAULT  	P3_2

#define SMART_SERVO_TEMP_AD   	P1_3
#define SMART_SERVO_VOL_AD    	P1_5
#define SMART_SERVO_CURR_AD   	P1_4


#define SMART_SERVO_TYPE0_AD	  	P1_0	/* smart servo type */
#define SMART_SERVO_TYPE1_AD	  	P1_2
#define SMART_SERVO_HW_Verison_D0	P3_4	/* hardware version */
#define SMART_SERVO_HW_Verison_D1	P3_5

#define SAMPLING_INTERVAL_TIME    10

#define SMART_SERVO_MIN_POSITION  (0)
#define SMART_SERVO_MAX_POSITION  (RAW_ANGLE_MAX_INT-1)

/*
	50 rpm - 200ms/60°
	40 rpm - 250ms/60°
	55.5 rpm - 180ms/60°
	28.75 rpm - 350ms/60°
	62.5 rmp - 160ms/60°
 */
#define SMART_SERVO_PER_SPEED_MAX  (50)    // ---- 0.16sec/60   (28.57 rpm)    0.200s/60°

// The minimum and maximum output.
#define SMART_SERVO_MAX_OUTPUT              (255)
#define SMART_SERVO_MIN_OUTPUT              (-SMART_SERVO_MAX_OUTPUT)
#define SMART_SERVO_MAX_OUTPUT_FLOAT              (255.0)
// #define SMART_SERVO_POS_DEADBAND            (12)
#define SMART_SERVO_MINIMUM_SPEED           (1)

#define SMART_SERVO_COMMON_MODE             (0)
#define SMART_SERVO_PWM_MODE                (1)


typedef struct
{
  float P, I, D;
  float Setpoint, Output, Integral, differential, last_error;
} PID;

typedef struct
{
  uint8_t R, G, B;
} smart_led_type;

extern volatile boolean pos_lock_flag;
extern volatile boolean protect_flag;
extern volatile boolean release_state_flag;


extern volatile long smart_servo_cur_pos;
extern volatile long smart_servo_target_pos;
extern volatile long smart_servo_pre_target_pos;
extern volatile long smart_servo_circular_turn_count;

extern volatile unsigned long period_time;

extern volatile float smart_servo_target_speed;
extern volatile float smart_servo_cur_speed;

extern volatile int32_t s_counter_for_heat;

extern volatile int16_t pre_pos;
extern volatile int16_t smart_servo_output;
extern volatile int16_t angle_pos_offset;
extern volatile int16_t motion_mode;
extern volatile int16_t smart_servo_pwm;

extern volatile uint16_t blink_count;
extern smart_led_type smart_led;
extern volatile boolean reach_pos_flag;

extern char type0;
extern char type1;

extern void smart_servo_init(void);
extern long smart_servo_distance_togo(void);
extern void smart_servo_led(uint8_t led_r,uint8_t led_g,uint8_t led_b);
extern void smart_led_change(uint8_t led_color);
extern void smart_led_blink(uint16_t blink_time,uint8_t led_r,uint8_t led_g,uint8_t led_b);
extern void smart_servo_break(boolean status);
extern void smart_servo_speed_update(int16_t speed);
extern void smart_servo_rotation(int16_t speed_pwm);
extern void smart_servo_move_to(long absolute_angle,float speed);
extern void smart_servo_move(long angle,float speed);
extern SpiFlashOpResult smart_servo_set_current_angle_zero_degrees(void);
extern void smart_servo_circular_turn_calc(int16_t cur_pos, int16_t pre_pos, float speed_input);
extern void smart_servo_update(void);
extern int16_t normalize_position_difference(int16_t posdiff);
extern int16_t pid_speed_to_pwm(void);
extern int16_t pid_position_to_pwm(void);
extern int16_t adc_get_position_value(void);
extern int16_t adc_get_temperature_value(void);
extern int16_t adc_get_voltage_value(void);
extern int16_t adc_get_current_value(void);
extern int16_t adc_get_smart_servo_type0_value(void); //for servo type low bit
extern int16_t adc_get_smart_servo_type1_value(void); //for servo type high bit
extern int16_t io_get_HW_Version_D0_value(void);  //for hardware type low bit
extern int16_t io_get_HW_Version_D1_value(void);  //for hardware type high bit
extern int16_t io_get_nfault_value(void);
extern void servo_move_test(float speed);
extern float calculate_temp(int16_t In_temp);
extern float calculate_current(int16_t In_cur);
extern float calculate_voltage(int16_t In_vol);
extern void motor_protection(void);
extern void get_smart_servo_type(void);
extern char get_Hardware_Version(void);
#endif/* __SMARTSERVO_H__ */
