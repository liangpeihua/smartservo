/**
 * @file    servo_driver.c
 * @author  Payton
 * @version V1.0.0
 * @date    2017/11/17
 * @brief   
 *
 * \par Description
 * This file is servo driver.
 *
 * \par History:
 * <pre>
 * `<Author>`        `<Time>`         `<Version>`        `<Descr>`
 * Payton            2017/11/17         1.0.0            create
 * </pre>
 *
 */
#include "servo_driver.h"
#include "servo_detect.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"


int16_t s_driver_pwm = 0;
MOTOR_CTRL_STATUS g_eSysMotionStatus = IDLE_MODE;  
static int32_t limitcurrent_offset = 0;
static int32_t limitcurrent_value = 0;

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

void servodriver_run_idle(void)
{
  digitalWrite(SMART_SERVO_SLEEP,0);
  g_eSysMotionStatus = IDLE_MODE;
  s_driver_pwm = 0;
  limitcurrent_offset = 0;
  limitcurrent_value = 0;
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
  if(g_servo_info.errorid == 0)
  {
    digitalWrite(SMART_SERVO_SLEEP,1);
    g_eSysMotionStatus = POS_MODE;

    g_servo_info.tar_pos = g_servo_info.cur_pos + angle*10;
    g_servo_info.tar_speed = constrain(abs_user(speed), -MAX_TAR_SPEED, MAX_TAR_SPEED);
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
	if(g_servo_info.errorid == 0)
	{
	  digitalWrite(SMART_SERVO_SLEEP,1);
	  g_eSysMotionStatus = PWM_MODE;

	  g_servo_info.tar_pwm = constrain((pwm*4), -MAX_OUTPUT_PWM, MAX_OUTPUT_PWM); 
  }
}

void servodriver_run_abspos_torque(int32_t angle,float speed,int32_t torque)
{
	if(g_servo_info.errorid == 0)
	{
	  digitalWrite(SMART_SERVO_SLEEP,1);
	  g_eSysMotionStatus = TORQUE_MODE;

	  g_servo_info.tar_pos = angle*10;
	  g_servo_info.tar_speed = constrain(abs_user(speed), -20, 20); 
	  g_servo_info.tar_torque = constrain(torque, 0, MAX_TORQUE);
  }
}

void servodriver_run_relativepos_torque(int32_t angle,float speed,int32_t torque)
{
	if(g_servo_info.errorid == 0)
	{
	  digitalWrite(SMART_SERVO_SLEEP,1);
	  g_eSysMotionStatus = TORQUE_MODE;

	  g_servo_info.tar_pos = g_servo_info.cur_pos + angle*10;
	  g_servo_info.tar_speed = constrain(abs_user(speed), -20, 20); 
	  g_servo_info.tar_torque = constrain(torque, 0, MAX_TORQUE);
  }
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
    g_servo_info.tar_pos = param2;//g_servo_info.cur_pos + param2;
    g_servo_info.tar_torque = constrain(param3, 0, MAX_TORQUE); 
    //g_servo_info.tar_pwm = constrain(param3, -MAX_OUTPUT_PWM, MAX_OUTPUT_PWM); 
  }
}

int16_t servodriver_get_pwmvalue(void)
{
  return s_driver_pwm;
}

/***********************************************************
* Function Name : servodriver_set_limitcurrent
* Description   : 设置限流值
* Input         : limit_current，0表示不限流
* Output        : NONE
* Return        : NONE
************************************************************/
void servodriver_set_limitcurrent(int32_t limit_current)
{
  limitcurrent_value = limit_current;
}

/***********************************************************
* Function Name : servodriver_limitcurrent_protect
* Description   : 限流处理
* Input         : cur_current, limit_current,set_pwm
* Output        : NONE
* Return        : NONE
************************************************************/
int32_t servodriver_limitcurrent_protect(int32_t cur_current, int32_t limit_current,int32_t set_pwm)
{
  int32_t output_pwm;
  int32_t step_len = 0;
	
  if(g_eSysMotionStatus == TORQUE_MODE)
  {
    if(cur_current > limit_current)
    {
    	step_len = abs_user(cur_current - limit_current) / 5;
    	step_len = constrain(step_len, 5, 100); 
      if(limitcurrent_offset < (int32_t)(MAX_OUTPUT_PWM-step_len)){
        limitcurrent_offset += step_len;
      }
    }
    else if(cur_current > (int32_t)(limit_current-20))//100mA滞回
    {
    }
    else
    {
      if(limitcurrent_offset > 2){
        limitcurrent_offset -= 2;
      }
    }
  }
  else
  {
    limitcurrent_offset = 0;
  }

  if(set_pwm > 0)
  {
    output_pwm = set_pwm - limitcurrent_offset;
  }
  else
  {
    output_pwm = set_pwm + limitcurrent_offset;
  }
  output_pwm = constrain(output_pwm, -MAX_OUTPUT_PWM, MAX_OUTPUT_PWM); 

  return output_pwm;
}

void servodriver_set_pwm(int16_t pwm)
{
#if 0
  int16_t set_pwm = pwm;
#else
  int16_t set_pwm;
  set_pwm = servodriver_limitcurrent_protect(g_servo_info.current, limitcurrent_value, pwm);
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

  if(s_driver_pwm >= 0)//cw
  {
#if  MP6528
    pwm_canceled(SMART_SERVO_PWMA, 0);	//Disable PWM Output
    pwm_canceled(SMART_SERVO_PWMB, 1);	//Enable PWM Output
    pwm_write(SMART_SERVO_PWMB,s_driver_pwm,0,MAX_OUTPUT_PWM);
#elif MP6515
    pwm_write(SMART_SERVO_ENBL,s_driver_pwm,0,MAX_OUTPUT_PWM);
    digitalWrite(SMART_SERVO_PHASE,1);
#endif
  }
  else//ccw
  {
#if  MP6528
    pwm_canceled(SMART_SERVO_PWMB, 0);	//Disable PWM Output
    pwm_canceled(SMART_SERVO_PWMA, 1);	//Enable PWM Output
    pwm_write(SMART_SERVO_PWMA,-s_driver_pwm,0,MAX_OUTPUT_PWM);
#elif MP6515
    pwm_write(SMART_SERVO_ENBL,-s_driver_pwm,0,MAX_OUTPUT_PWM);
    digitalWrite(SMART_SERVO_PHASE,0);
#endif
  }
}


