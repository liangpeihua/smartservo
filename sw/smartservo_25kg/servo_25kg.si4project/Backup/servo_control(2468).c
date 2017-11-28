#include "servo_control.h"
#include "servo_driver.h"
#include "servo_detect.h"
#include "systime.h"
#include "sysinit.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"
#include "usart_Fun.h"
	
#define MAX_SPEED      		38	//1rpm/min,实测最大转速
#define _PWM(Speed)		      ((Speed) * MAX_OUTPUT_PWM / MAX_SPEED)	//拟合pwm

STRUCT_PID speed_ctrl = {0};
STRUCT_PID pos_ctrl = {0};
STRUCT_PID torque_ctrl = {0};
int32_t s_output_pwm = 0;

//运动控制状态
static boolean s_bMotionStatusChanged = TRUE;


static void motor_idle_mode(void)
{
	if(s_bMotionStatusChanged)
	{
    speed_ctrl.integral = 0;
    speed_ctrl.output = 0;
    pos_ctrl.integral = 0;
    pos_ctrl.output = 0;
    s_output_pwm = 0;
    servodriver_run_idle();
		//servodriver_set_pwm(s_output_pwm);
		g_servo_info.reach_tar_pos = true;
	}    
	servodriver_set_pwm(s_output_pwm);
}

static void motor_pwm_mode(void)
{
	int32_t set_pwm = 0; 
	static int32_t step_len = 0;

	if(s_bMotionStatusChanged)
	{
		step_len = 30;
		s_output_pwm = 0;
	}

	if(g_servo_info.errorid != 0)
	{
		g_eSysMotionStatus = ERROR_MODE;
		return;
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

	s_output_pwm = constrain(s_output_pwm, -g_servo_info.limit_pwm, g_servo_info.limit_pwm);
	servodriver_set_pwm(s_output_pwm);
}

static void motor_speed_mode(void)
{
	int32_t speed_error;

	if(s_bMotionStatusChanged)
	{
		speed_ctrl.integral = _PWM(g_servo_info.cur_speed) / speed_ctrl.Ki;
		speed_ctrl.output = 0;
		speed_ctrl.Kp = 200;
		speed_ctrl.Ki = 20;
	}

	if(g_servo_info.errorid != 0)
	{
		g_eSysMotionStatus = ERROR_MODE;
		return;
	}

	speed_error = g_servo_info.tar_speed - g_servo_info.cur_speed;
	speed_error = constrain(speed_error,-20,20);
	speed_ctrl.integral += speed_error;
	speed_ctrl.integral = constrain(speed_ctrl.integral,-(10*g_servo_info.limit_pwm/speed_ctrl.Ki),(10*g_servo_info.limit_pwm/speed_ctrl.Ki));
	speed_ctrl.output = (speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral) / 10;
	speed_ctrl.output = constrain(speed_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);

	s_output_pwm = speed_ctrl.output;
	servodriver_set_pwm(s_output_pwm);
}

//占用时间T= 70us
static void motor_pos_mode(void)
{
	int32_t speed_error;
	int32_t pos_error;
	int32_t abspos_error;
	int32_t target_speed;
	static int32_t pre_tar_pos = 0;
	float A;
	int32_t H;
	int32_t K;

	if(s_bMotionStatusChanged)
	{
		speed_ctrl.integral = _PWM(g_servo_info.cur_speed) / speed_ctrl.Ki;
		speed_ctrl.output = 0;
		speed_ctrl.Kp = 200;
		speed_ctrl.Ki = 20;
		
		pos_ctrl.output = 0;
		pos_ctrl.Kp = 1;
		pos_ctrl.Kd = 2;

		pre_tar_pos = g_servo_info.tar_pos;
	}

	if(g_servo_info.errorid != 0)
	{
		g_eSysMotionStatus = ERROR_MODE;
		return;
	}

	if(pre_tar_pos != g_servo_info.tar_pos)
	{
		pos_ctrl.Kp = 1;
		pos_ctrl.Kd = 2;
		pre_tar_pos = g_servo_info.tar_pos;
	}
	else if(abs_user(pos_error) < 10)
	{
		pos_ctrl.Kp = 10;
		pos_ctrl.Kd = 20;
	}

	//pos pid
	pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
	pos_error = constrain(pos_error,-20,20);
	pos_ctrl.output = pos_ctrl.Kp * pos_error + pos_ctrl.Kd * (pos_error - pos_ctrl.last_error);
	pos_ctrl.output = constrain(pos_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);
	pos_ctrl.last_error = pos_error;
	
	//speed pid
	H = 700;
	K = MAX_TAR_SPEED;
	A = (float)(-MAX_TAR_SPEED) / pow(H,2);
	pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
	LIMIT_DEATH(pos_error, 10);
	abspos_error = abs_user(pos_error);
	if(abspos_error == 0)
	{
		target_speed = 0;
	}
	else if(abspos_error < H)
	{
		target_speed = A*pow((abspos_error - H),2) + K;//二次函数，顶点式：y=a(x-h)²+k(a≠0）
		if(target_speed < 1){
			target_speed = 1;
		}
	}
	else
	{
		target_speed = abs_user(g_servo_info.tar_speed);
	}
	
	target_speed = constrain(target_speed,0,abs_user(g_servo_info.tar_speed));
	g_servo_info.posmode_tarspeed = target_speed;
	if(pos_error > 0){
		target_speed= abs_user(target_speed);
	}
	else{
		target_speed= -abs_user(target_speed);
	}
	speed_error = target_speed - g_servo_info.cur_speed;
	speed_error = constrain(speed_error,-20,20);
	speed_ctrl.integral += speed_error;
	speed_ctrl.integral = constrain(speed_ctrl.integral,-(10*g_servo_info.limit_pwm/speed_ctrl.Ki),(10*g_servo_info.limit_pwm/speed_ctrl.Ki));
	speed_ctrl.output = (speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral) / 10;
	speed_ctrl.output = constrain(speed_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);

	s_output_pwm = pos_ctrl.output + speed_ctrl.output;
	s_output_pwm = constrain(s_output_pwm,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);
	servodriver_set_pwm(s_output_pwm);
}

static void  motor_torque_mode(void)
{
	int32_t speed_error;
	int32_t pos_error;
	int32_t abspos_error;
	int32_t target_speed;
	int32_t torque_error;
	int32_t tar_torque;
	static int32_t pre_tar_pos = 0;
	float A;
	int32_t H;
	int32_t K;

	if(s_bMotionStatusChanged)
	{
		speed_ctrl.integral = _PWM(g_servo_info.cur_speed) / speed_ctrl.Ki;
		speed_ctrl.output = 0;
		speed_ctrl.Kp = 100;
		speed_ctrl.Ki = 2;
		
		pos_ctrl.output = 0;
		pos_ctrl.Kp = 1;
		pos_ctrl.Kd = 2;

		torque_ctrl.integral = 0;
		torque_ctrl.output = 0;
		torque_ctrl.Kp = 100;
		torque_ctrl.Ki = 20;

		pre_tar_pos = g_servo_info.tar_pos;
	}

	if(g_servo_info.errorid != 0)
	{
		g_eSysMotionStatus = ERROR_MODE;
		return;
	}

	if(pre_tar_pos != g_servo_info.tar_pos)
	{
		pos_ctrl.Kp = 1;
		pos_ctrl.Kd = 2;
		pre_tar_pos = g_servo_info.tar_pos;
	}
	else if(abs_user(pos_error) < 10)
	{
		pos_ctrl.Kp = 10;
		pos_ctrl.Kd = 20;
	}

	//pos pid
	pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
	pos_error = constrain(pos_error,-20,20);
	pos_ctrl.output = pos_ctrl.Kp * pos_error + pos_ctrl.Kd * (pos_error - pos_ctrl.last_error);
	pos_ctrl.output = constrain(pos_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);
	pos_ctrl.last_error = pos_error;
	
	//speed pid
	H = 700;
	K = MAX_TAR_SPEED;
	A = (float)(-MAX_TAR_SPEED) / pow(H,2);
	pos_error = g_servo_info.tar_pos - g_servo_info.cur_pos;
	LIMIT_DEATH(pos_error, 10);
	abspos_error = abs_user(pos_error);
	if(abspos_error == 0)
	{
		target_speed = 0;
	}
	else if(abspos_error < H)
	{
		target_speed = A*pow((abspos_error - H),2) + K;//二次函数，顶点式：y=a(x-h)²+k(a≠0）
		if(target_speed < 1){
			target_speed = 1;
		}
	}
	else
	{
		target_speed = abs_user(g_servo_info.tar_speed);
	}
	
	target_speed = constrain(target_speed,0,abs_user(g_servo_info.tar_speed));
	g_servo_info.posmode_tarspeed = target_speed;
	if(pos_error > 0){
		target_speed= abs_user(target_speed);
	}
	else{
		target_speed= -abs_user(target_speed);
	}
	speed_error = target_speed - g_servo_info.cur_speed;
	speed_error = constrain(speed_error,-20,20);
	speed_ctrl.integral += speed_error;
	speed_ctrl.integral = constrain(speed_ctrl.integral,-(10*g_servo_info.limit_pwm/speed_ctrl.Ki),(10*g_servo_info.limit_pwm/speed_ctrl.Ki));
	speed_ctrl.output = (speed_ctrl.Kp * speed_error + speed_ctrl.Ki * speed_ctrl.integral) / 10;
	speed_ctrl.output = constrain(speed_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);

	//torque pid
	tar_torque = g_servo_info.tar_torque*5;
	torque_error = tar_torque - g_servo_info.current;
	torque_error = constrain(torque_error,-20,20);
	torque_ctrl.integral += torque_error;
	torque_ctrl.integral = constrain(torque_ctrl.integral,-(10*g_servo_info.limit_pwm/torque_ctrl.Ki),(10*g_servo_info.limit_pwm/torque_ctrl.Ki));
	torque_ctrl.output = (torque_ctrl.Kp * torque_error + torque_ctrl.Ki * torque_ctrl.integral) / 10;
	torque_ctrl.output = constrain(torque_ctrl.output,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);

	s_output_pwm = pos_ctrl.output + speed_ctrl.output + torque_ctrl.output;
	s_output_pwm = constrain(s_output_pwm,-g_servo_info.limit_pwm,g_servo_info.limit_pwm);
	servodriver_set_pwm(s_output_pwm);
}



static void motor_error_mode(void)
{
	static int32_t step_len = 0;
	int32_t set_pwm = 0; 
		
	if(s_bMotionStatusChanged)
	{
		speed_ctrl.integral = 0;
		speed_ctrl.output = 0;
		pos_ctrl.integral = 0;
		pos_ctrl.output = 0;
		step_len = 0;
	}

	if(s_output_pwm == 0)
	{
		g_eSysMotionStatus = IDLE_MODE;
		return;
	}

	if(abs_user(s_output_pwm) > g_servo_info.limit_pwm/2)
	{
		step_len = 3;
	}
	else
	{
		step_len = 1;
	}

	set_pwm = 0;

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
	
	servodriver_set_pwm(s_output_pwm);
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

	if(g_servo_info.errorid != 0)
	{
		g_eSysMotionStatus = ERROR_MODE;
		return;
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

	s_output_pwm = constrain(s_output_pwm, -g_servo_info.limit_pwm, g_servo_info.limit_pwm);
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

	if(g_servo_info.ready == false)
	{
		return;
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

    case TORQUE_MODE:
    	motor_torque_mode();
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

//void check_whether_reach_the_postion(void)
//{
//	uint8_t checksum;
//	uint8_t reach_pos_flag = 1;

//	if(g_servo_info.reach_tar_pos)
//	
//	//response mesaage to UART0
//	write_byte_uart0(START_SYSEX);
//	write_byte_uart0(device_id);
//	write_byte_uart0(SMART_SERVO);
//	write_byte_uart0(CHECK_WHETHER_REACH_THE_SET_POSITION);
//	write_byte_uart0((uint8_t)reach_pos_flag);
//	checksum = (device_id + SMART_SERVO + CHECK_WHETHER_REACH_THE_SET_POSITION+(uint8_t)reach_pos_flag);
//	checksum = checksum & 0x7f;
//	write_byte_uart0(checksum);
//	write_byte_uart0(END_SYSEX);
//}

