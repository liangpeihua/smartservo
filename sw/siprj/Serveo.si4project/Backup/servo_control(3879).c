#include "servo_control.h"
#include "servo_driver.h"
#include "servo_detect.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"
#include "usart_Fun.h"


#define MAX_SPEED      		2500	//0.1rpm/min,实测最大转速
#define _PWM(Speed)		      ((Speed) * MAX_OUTPUT_PWM / MAX_SPEED)	//pwn拟合
			

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


//运动控制状态
static boolean s_bMotionStatusChanged = TRUE;


static void motor_idle_mode(void)
{
    speed_ctrl.integral = 0;
    speed_ctrl.output = 0;
    pos_ctrl.integral = 0;
    pos_ctrl.output = 0;
    s_output_pwm = 0;
    servodriver_set_pwm(s_output_pwm);
}

static void motor_pwm_mode(void)
{
	int32_t set_pwm = 0; 
	int32_t step_len = 0;

	if(s_bMotionStatusChanged)
	{
		step_len = 20;
		s_output_pwm = 0;
	}

	set_pwm = g_servo_info.tar_pwm;

	if(set_pwm > s_output_pwm+step_len)
	{
    s_output_pwm += step_len; 
  }
  else if(set_pwm < s_output_pwm-step_len)
  {
    s_output_pwm -= step_len;
  }
  else
  {
    s_output_pwm = set_pwm;
  }

	s_output_pwm = constrain(s_output_pwm, -MAX_OUTPUT_PWM, MAX_OUTPUT_PWM);
	servodriver_set_pwm(s_output_pwm);
}

static void motor_speed_mode(void)
{
	int32_t speed_error;

	if(s_bMotionStatusChanged)
	{
		speed_ctrl.integral = _PWM(g_servo_info.cur_speed) / speed_ctrl.Ki;
		speed_ctrl.output = 0;
	}

	speed_error = g_servo_info.tar_speed - g_servo_info.cur_speed;
	speed_error = constrain(speed_error,-50,50);
	speed_ctrl.integral += speed_error;
	speed_ctrl.integral = constrain(speed_ctrl.integral,-1024,1024);
	speed_ctrl.output = speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral;
	speed_ctrl.output = constrain(speed_ctrl.output,-MAX_OUTPUT_PWM,MAX_OUTPUT_PWM);

	s_output_pwm = speed_ctrl.output;
	servodriver_set_pwm(s_output_pwm);
}

static void motor_pos_mode(void)
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
	pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
	pos_error = constrain(pos_error,-20,20);
	pos_ctrl.output = pos_ctrl.Kp * pos_error + pos_ctrl.Kd * (pos_error - pos_ctrl.last_error);
	pos_ctrl.output = constrain(pos_ctrl.output,-MAX_OUTPUT_PWM,MAX_OUTPUT_PWM);
	pos_ctrl.last_error = pos_error;
	
	//speed pid
	pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
	target_speed = abs_user(pos_error) * 0.4;
	target_speed = constrain(target_speed,0,abs_user(g_servo_info.tar_speed));
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

static void motor_error_mode(void)
{
    speed_ctrl.integral = 0;
    speed_ctrl.output = 0;
    pos_ctrl.integral = 0;
    pos_ctrl.output = 0;
    s_output_pwm = 0;
    servodriver_set_pwm(s_output_pwm);
    
    servodriver_run_idle();
}

static void motor_debug_mode(void)
{
	int32_t set_pwm = 0; 
	static int32_t step_len = 0;

	if(s_bMotionStatusChanged)
	{
		step_len = 10;
		s_output_pwm = 100;
	}

	set_pwm = g_servo_info.tar_pwm;

	if(set_pwm > s_output_pwm+step_len)
	{
		s_output_pwm += step_len; 
	}
	else if(set_pwm < s_output_pwm-step_len)
	{
		s_output_pwm -= step_len;
	}
	else
	{
		s_output_pwm = set_pwm;
	}

	s_output_pwm = constrain(s_output_pwm, -MAX_OUTPUT_PWM, MAX_OUTPUT_PWM);
	servodriver_set_pwm(s_output_pwm);
}


void motor_process(void)
{
  static MOTOR_CTRL_STATUS pre_state = (MOTOR_CTRL_STATUS)0xFF;

	//servodriver_run_debug(0,0, 0, MAX_OUTPUT_PWM/2);
{
	uint8_t buff[8] = {0};
	buff[0] = 1;
	buff[1] = 2;
	COMSendBuffer(0x00660001, buff, 8);
}
  s_bMotionStatusChanged = false;
  if(pre_state != g_eSysMotionStatus)
  {
    s_bMotionStatusChanged = true;
  }
  pre_state = g_eSysMotionStatus;

  switch(g_eSysMotionStatus)
  {
    case IDLE_MODE:
      motor_idle_mode();
      break;
        
    case PWM_MODE:
      motor_pwm_mode();
      break;
        
    case SPEED_MODE:
      motor_speed_mode();
      break; 
        
    case POS_MODE:
      motor_pos_mode();
      break;

    case ERROR_MODE:
      motor_error_mode();
      break;
      
		case DEBUG_MODE:
					motor_debug_mode();
					break;
    default: 
			motor_idle_mode();
      break;
  }
}

