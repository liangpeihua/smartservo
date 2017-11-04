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


#define MAX_SPEED      		30	//1rpm/min,实测最大转速
#define _PWM(Speed)		      ((Speed) * MAX_OUTPUT_PWM / MAX_SPEED)	//pwn拟合
			

STRUCT_PID speed_ctrl = {0};
STRUCT_PID pos_ctrl = {0};
int32_t s_output_pwm = 0;


//运动控制状态
static boolean s_bMotionStatusChanged = TRUE;


static void motor_idle_mode(void)
{
    speed_ctrl.integral = 0;
    speed_ctrl.output = 0;
    pos_ctrl.integral = 0;
    pos_ctrl.output = 0;
    s_output_pwm = 0;
    servodriver_run_idle();
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
		speed_ctrl.Kp = 500;
		speed_ctrl.Ki = 30;
	}

	speed_error = g_servo_info.tar_speed - g_servo_info.cur_speed;
	speed_error = constrain(speed_error,-20,20);
	speed_ctrl.integral += speed_error;
	speed_ctrl.integral = constrain(speed_ctrl.integral,-(100*MAX_OUTPUT_PWM/speed_ctrl.Ki),(100*MAX_OUTPUT_PWM/speed_ctrl.Ki));
	speed_ctrl.output = (speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral) / 100;
	speed_ctrl.output = constrain(speed_ctrl.output,-MAX_OUTPUT_PWM,MAX_OUTPUT_PWM);

	s_output_pwm = speed_ctrl.output;
	servodriver_set_pwm(s_output_pwm);
}

int32_t target_speed;

static void motor_pos_mode(void)
{
	int32_t speed_error;
	int32_t pos_error;
	int32_t abspos_error;
	//int32_t target_speed;

	if(s_bMotionStatusChanged)
	{
		speed_ctrl.integral = _PWM(g_servo_info.cur_speed) / speed_ctrl.Ki;
		speed_ctrl.output = 0;
		speed_ctrl.Kp = 100;
		speed_ctrl.Ki = 10;
		
		pos_ctrl.output = 0;
		pos_ctrl.Kp = 1;
		pos_ctrl.Kd = 1;
	}
	
	//pos pid
	pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
	LIMIT_DEATH(pos_error, 10);
	pos_error = constrain(pos_error,-20,20);
	pos_ctrl.output = pos_ctrl.Kp * pos_error + pos_ctrl.Kd * (pos_error - pos_ctrl.last_error);
	pos_ctrl.output = constrain(pos_ctrl.output,-MAX_OUTPUT_PWM,MAX_OUTPUT_PWM);
	pos_ctrl.last_error = pos_error;
	
	//speed pid
	pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
	LIMIT_DEATH(pos_error, 10);
//	if(abs_user(pos_error) > 1)
//	{
//		target_speed = 3*logab(10,abs_user(pos_error));//底数越大，斜率越大
//	}
//	else
//	{
//		target_speed = 0;
//	}

	abspos_error = abs_user(pos_error);

	if(abspos_error == 0)
	{
		target_speed = 0;
		speed_ctrl.Kp = 200;
		speed_ctrl.Ki = 20;
	}
	else if(abspos_error < (abs_user(g_servo_info.tar_speed) * 10 / 25))
	{
		target_speed = abspos_error * 0.1;
		target_speed = 1;
		speed_ctrl.Kp = 200;
		speed_ctrl.Ki = 20;
	}
	else
	{
		target_speed = abspos_error * 1;
		speed_ctrl.Kp = 200;
		speed_ctrl.Ki = 20;
	}
	
	target_speed = constrain(target_speed,0,abs_user(g_servo_info.tar_speed));
	if(pos_error > 0){
		target_speed= abs_user(target_speed);
	}
	else{
		target_speed= -abs_user(target_speed);
	}
	speed_error = target_speed - g_servo_info.cur_speed;
	speed_error = constrain(speed_error,-30,30);
	speed_ctrl.integral += speed_error;
	speed_ctrl.integral = constrain(speed_ctrl.integral,-(10*MAX_OUTPUT_PWM/speed_ctrl.Ki),(10*MAX_OUTPUT_PWM/speed_ctrl.Ki));
	speed_ctrl.output = (speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral) / 10;
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
		step_len = 5;
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


/***********************************************************
* Function Name : motor_process
* Description   : 运动控制线程,5ms执行一次
* Input         : NONE
* Output        : NONE
* Return        : NONE
************************************************************/
void motor_process(void)
{
  static MOTOR_CTRL_STATUS pre_state = (MOTOR_CTRL_STATUS)0xFF;

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

