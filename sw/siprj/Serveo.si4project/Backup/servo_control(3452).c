#include "servo_control.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"


#define MAX_SPEED      		2500	//0.1rpm/min,实测最大转速
#define _PWM(Speed)		      ((Speed) * MAX_OUTPUT_PWM / MAX_SPEED)	//pwn拟合
				
typedef enum
{
    MIDLE_MODE = 0,	
    SPEED_OPEN_MODE,		   
    SPEED_CLOSE_MODE,
    POS_MODE,
    ERROR_MODE,		    
    BRAKE_MODE,     
    DEBUG_MODE,
} MotionStatus;

typedef struct
{
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;
	int32_t integral;
	int32_t max_integral;
	int32_t last_error;
	int32_t output; 
}STRUCT_PID;

static STRUCT_PID speed_ctrl = {0};
static STRUCT_PID pos_ctrl = {0};
static int32_t s_output_pwm = 0;


int32_t tar_speed;
int32_t tar_pos;



//运动控制状态
static bool s_bMotionStatusChanged = TRUE;
MotionStatus g_eSysMotionStatus = MIDLE_MODE;  


void motor_idle_mode(uint8_t slot)
{
    speed_ctrl.integral = 0;
    speed_ctrl.output = 0;
    pos_ctrl.integral = 0;
    pos_ctrl.output = 0;
    s_output_pwm = 0;
    servodriver_set_pwm(s_output_pwm);
}

void motor_speed_open_mode(void)
{
	int32_t set_pwm = 0; 
	int32_t step_len = 0;

	if(s_bMotionStatusChanged)
	{
		step_len = 20;
		speed_ctrl.output = 0;
	}

	set_pwm = _PWM(tar_speed);

	if(set_pwm > speed_ctrl.output+step_len)
	{
    speed_ctrl.output += step_len; 
  }
  else if(set_pwm < speed_ctrl.output-step_len)
  {
    speed_ctrl.output -= step_len;
  }
  else
  {
    speed_ctrl.output = set_pwm;
  }

  s_output_pwm = speed_ctrl.output;
	servodriver_set_pwm(s_output_pwm);
}

void motor_speed_close_mode(void)
{
	int32_t speed_error;

	if(s_bMotionStatusChanged)
	{
		speed_ctrl.integral = _PWM(g_servo_info.cur_speed) / speed_ctrl.Ki;
		speed_ctrl.output = 0;
	}

	speed_error = tar_speed - g_servo_info.cur_speed;
	speed_error = constrain(speed_error,-50,50);
	speed_ctrl.integral += speed_error;
	speed_ctrl.integral = constrain(speed_ctrl.integral,-1024,1024);
	speed_ctrl.output = speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral;
	speed_ctrl.output = constrain(speed_ctrl.output,-MAX_OUTPUT_PWM,MAX_OUTPUT_PWM);

	s_output_pwm = speed_ctrl.output;
	servodriver_set_pwm(s_output_pwm);
}

void motor_pos_mode(void)
{
	int32_t speed_error;
	int32_t pos_error;
	int32_t target_speed;

	if(s_bMotionStatusChanged)
	{
		speed_ctrl.integral = _PWM(g_servo_info.cur_speed) / speed_ctrl.Ki;
		speed_ctrl.output = 0;
		pos_ctrl.integral = 0;
		pos_ctrl.output = 0;
	}
	
	//pos pid
	pos_error = tar_pos - g_servo_info.cur_pos;
	pos_error = constrain(pos_error,-20,20);
	pos_ctrl.output = pos_ctrl.Kp * pos_error + pos_ctrl.Kd * (pos_error - pos_ctrl.last_error);
	pos_ctrl.output = constrain(pos_ctrl.output,-MAX_OUTPUT_PWM,MAX_OUTPUT_PWM);
	pos_ctrl.last_error = pos_error;
	
	//speed pid
	pos_error = tar_pos - g_servo_info.cur_pos;
	target_speed = abs_user(pos_error) * 0.4;
	target_speed = constrain(target_speed,0,abs_user(tar_speed));
	if(pos_error > 0){
		target_speed= abs_user(target_speed);
	}
	else{
		target_speed= -abs_user(target_speed);
	}
	speed_error = target_speed - g_servo_info.cur_speed;
	speed_error = constrain(speed_error,-50,50);
	speed_ctrl.integral += speed_error;
	speed_ctrl.integral = constrain(speed_ctrl.integral,-1024,1024);
	speed_ctrl.output = speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral;
	speed_ctrl.output = constrain(speed_ctrl.output,-MAX_OUTPUT_PWM,MAX_OUTPUT_PWM);

	s_output_pwm = pos_ctrl.output + speed_ctrl.output;
	s_output_pwm = constrain(s_output_pwm,-MAX_OUTPUT_PWM,MAX_OUTPUT_PWM);
	servodriver_set_pwm(s_output_pwm);
}



