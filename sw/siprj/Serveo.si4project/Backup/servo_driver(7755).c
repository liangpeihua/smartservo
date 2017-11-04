#include "servo_driver.h"
#include "servo_detect.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"


//GPIO define
#define SMART_SERVO_ENA 		P3_4
#define SMART_SERVO_ENB 		P3_5
#define SMART_SERVO_PWMA    	P2_5
#define SMART_SERVO_PWMB    	P2_6
#define SMART_SERVO_SLEEP   	P3_6
#define SMART_SERVO_NFAULT  	P3_2


MotionStatus g_eSysMotionStatus = IDLE_MODE;  

void servodriver_init(void)
{
  pinMode(SMART_SERVO_HW_Verison_D0,GPIO_PMD_INPUT);
  pinMode(SMART_SERVO_HW_Verison_D1,GPIO_PMD_INPUT);

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
}

int16_t servodriver_get_nfault_value(void)
{
  int value = -1;
  value = digitalRead(SMART_SERVO_NFAULT);
  return value;
}

void servodriver_set_pwm(int16_t pwm)
{
  int16_t speed_temp = pwm;

  if(speed_temp > 0)
  {
		pwm_write(SMART_SERVO_PWMA,0,0,MAX_OUTPUT_PWM);
		pwm_write(SMART_SERVO_PWMB,speed_temp,0,MAX_OUTPUT_PWM);
  }
  else
  {
		pwm_write(SMART_SERVO_PWMA,-speed_temp,0,MAX_OUTPUT_PWM);
		pwm_write(SMART_SERVO_PWMB,0,0,MAX_OUTPUT_PWM);
  }
}

void servodriver_run_idle(void)
{
  digitalWrite(SMART_SERVO_SLEEP,0);
  g_eSysMotionStatus = IDLE_MODE;
}


void servodriver_run_abs_pos(long angle,float speed)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  g_eSysMotionStatus = POS_MODE;

  g_servo_info.tar_speed = speed*10;
  g_servo_info.tar_pos = angle*10;
}

void servodriver_run_relative_pos(long angle,float speed)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  g_eSysMotionStatus = POS_MODE;

  g_servo_info.tar_speed = speed*10;
  g_servo_info.tar_pos = (g_servo_info.cur_pos + angle)*10;
}

void servodriver_run_speed(float speed)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  g_eSysMotionStatus = SPEED_MODE;

  g_servo_info.tar_speed = speed*10;
}

void servodriver_run_pwm(int16_t pwm)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  g_eSysMotionStatus = PWM_MODE;

  g_servo_info.tar_pwm = pwm;
}

void servodriver_run_error(void)
{
  digitalWrite(SMART_SERVO_SLEEP,0);
  g_eSysMotionStatus = ERROR_MODE;

}

void servodriver_run_debug(long angle,float speed)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  g_eSysMotionStatus = DEBUG_MODE;

  g_servo_info.tar_speed = speed*10;
  g_servo_info.tar_pos = angle*10;
}

