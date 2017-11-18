#include "servo_driver.h"
#include "servo_detect.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"


static int16_t s_driver_pwm = 0;
MOTOR_CTRL_STATUS g_eSysMotionStatus = IDLE_MODE;  

int32_t servodriver_limitcurrent(int32_t cur_current, int32_t limit_current,int32_t set_pwm);

void servodriver_init(void)
{
#if  MP6528
  pinMode(SMART_SERVO_ENA,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_ENB,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_PWMA,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_PWMB,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_SLEEP,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_NFAULT,GPIO_PMD_INPUT);
  digitalWrite(SMART_SERVO_ENA,1);
  digitalWrite(SMART_SERVO_ENB,1);
  pwm_init(SMART_SERVO_PWMA,20000);  //20KHz  PWM Timer period = 50us
  pwm_init(SMART_SERVO_PWMB,20000);
#elif MP6515
  pinMode(SMART_SERVO_SLEEP,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_PHASE,GPIO_PMD_OUTPUT);
	pinMode(SMART_SERVO_ENBL,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_NFAULT,GPIO_PMD_INPUT);
  pwm_init(SMART_SERVO_ENBL,20000);  //20000  PWM Timer period = 50
#endif
}

int32_t set_current;

void servodriver_set_pwm(int16_t pwm)
{
#if 0
  int16_t set_pwm = pwm;
#else
	int16_t set_pwm;
  set_pwm = servodriver_limitcurrent(g_servo_info.current, g_servo_info.limit_current, pwm);
#endif

	if(set_pwm > s_driver_pwm+(MAX_OUTPUT_PWM/25))
	{
		s_driver_pwm += (MAX_OUTPUT_PWM/25); 
	}
	else if(set_pwm < s_driver_pwm-(MAX_OUTPUT_PWM/25))
	{
		s_driver_pwm -= (MAX_OUTPUT_PWM/25);
	}
	else
	{
		s_driver_pwm = set_pwm;
	}

  if(s_driver_pwm > 0)//cw
  {
#if  MP6528
			pwm_write(SMART_SERVO_PWMA,s_driver_pwm,0,MAX_OUTPUT_PWM);
			pwm_write(SMART_SERVO_PWMB,0,0,MAX_OUTPUT_PWM);
#elif MP6515
			pwm_write(SMART_SERVO_ENBL,s_driver_pwm,0,MAX_OUTPUT_PWM);
			digitalWrite(SMART_SERVO_PHASE,1);
#endif
  }
  else//ccw
  {
#if  MP6528
		pwm_write(SMART_SERVO_PWMA,0,0,MAX_OUTPUT_PWM);
		pwm_write(SMART_SERVO_PWMB,-s_driver_pwm,0,MAX_OUTPUT_PWM);
#elif MP6515
		pwm_write(SMART_SERVO_ENBL,-s_driver_pwm,0,MAX_OUTPUT_PWM);
 	  digitalWrite(SMART_SERVO_PHASE,0);
#endif
  }
}

void servodriver_run_idle(void)
{
  digitalWrite(SMART_SERVO_SLEEP,0);
  g_eSysMotionStatus = IDLE_MODE;
  s_driver_pwm = 0;
}

void servodriver_run_abs_pos(int32_t angle,float speed)
{
	if(g_servo_info.errorid == 0)
	{
	  digitalWrite(SMART_SERVO_SLEEP,1);
	  g_eSysMotionStatus = POS_MODE;

	  g_servo_info.tar_pos = angle*10;
	  g_servo_info.tar_speed = constrain(abs_user(speed), -MAX_TAR_SPEED, MAX_TAR_SPEED); 
  }
}

void servodriver_run_relative_pos(int32_t angle,float speed)
{
	int32_t max_speed;
	
	if(g_servo_info.errorid == 0)
	{
	  digitalWrite(SMART_SERVO_SLEEP,1);
	  g_eSysMotionStatus = POS_MODE;

	  g_servo_info.tar_pos = g_servo_info.cur_pos + angle*10;
	  g_servo_info.tar_speed = constrain(abs_user(speed), -MAX_TAR_SPEED, MAX_TAR_SPEED);

		max_speed = MAX_TAR_SPEED * abs_user(g_servo_info.tar_pos-g_servo_info.cur_pos) / 600;
		max_speed = constrain(max_speed, 30, MAX_TAR_SPEED); 
		//g_servo_info.tar_speed = constrain(abs_user(speed), -max_speed,max_speed); 
  }
}

void servodriver_run_speed(float speed)
{	
	if(g_servo_info.errorid == 0)
	{
	  digitalWrite(SMART_SERVO_SLEEP,1);
	  g_eSysMotionStatus = SPEED_MODE;

	  g_servo_info.tar_speed = constrain(speed, -MAX_TAR_SPEED, MAX_TAR_SPEED); 
  }
}

void servodriver_run_pwm(int16_t pwm)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  g_eSysMotionStatus = PWM_MODE;

  g_servo_info.tar_pwm = constrain((pwm*4), -MAX_OUTPUT_PWM, MAX_OUTPUT_PWM); 
}

void servodriver_run_torque(int32_t angle,float speed,int32_t torque)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  g_eSysMotionStatus = TORQUE_MODE;
  
	g_servo_info.tar_pos = g_servo_info.cur_pos + angle*10;
	g_servo_info.tar_speed = constrain(abs_user(speed), -MAX_TAR_SPEED, MAX_TAR_SPEED); 
	g_servo_info.tar_torque = constrain(torque, -MAX_TORQUE, MAX_TORQUE); 
}

void servodriver_run_error(void)
{
  digitalWrite(SMART_SERVO_SLEEP,0);
  g_eSysMotionStatus = ERROR_MODE;

}

void servodriver_run_debug(uint8_t mode,int32_t param1,int32_t param2,int32_t param3)
{
	if(g_servo_info.errorid == 0)
	{
	  digitalWrite(SMART_SERVO_SLEEP,1);
	  //g_eSysMotionStatus = DEBUG_MODE;
	  g_eSysMotionStatus = (MOTOR_CTRL_STATUS)mode;

	  g_servo_info.tar_speed = constrain(param1, -MAX_TAR_SPEED, MAX_TAR_SPEED); 
	  g_servo_info.tar_pos = g_servo_info.cur_pos + param2;
	  //g_servo_info.tar_torque = constrain(param3, -MAX_TORQUE, MAX_TORQUE); 
	  g_servo_info.tar_pwm = constrain(param3, -MAX_OUTPUT_PWM, MAX_OUTPUT_PWM); 
  }
}

/***********************************************************
* Function Name : servodriver_limitcurrent
* Description   : 限流处理
* Input         : speed
* Output        : NONE
* Return        : NONE
************************************************************/
int32_t servodriver_limitcurrent(int32_t cur_current, int32_t limit_current,int32_t set_pwm)
{
	int32_t output_pwm;
	static int32_t limitcurrent_offset = 0;

	if(cur_current > limit_current)
	{
		if(limitcurrent_offset < MAX_OUTPUT_PWM){
			limitcurrent_offset++;
		}
	}
	else if(cur_current < limit_current-100)//100mA滞回
	{
	}
	else
	{
		if(limitcurrent_offset > 0){
			limitcurrent_offset--;
		}
	}
	
	if(set_pwm > 0)
	{
		output_pwm = set_pwm - limitcurrent_offset;
	}
	else
	{
		output_pwm = set_pwm + limitcurrent_offset;
	}

	return output_pwm;
}


