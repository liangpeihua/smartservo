#include "servo_driver.h"
#include "servo_detect.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"


#define MP6515  1//10kg
#define MP6528	0//25kg

//GPIO define 
#if  MP6528
#define SMART_SERVO_ENA 			P3_4
#define SMART_SERVO_ENB 			P3_5
#define SMART_SERVO_PWMA    	P2_5
#define SMART_SERVO_PWMB    	P2_6
#define SMART_SERVO_SLEEP   	P3_6
#define SMART_SERVO_NFAULT  	P3_2
#elif MP6515
#define SMART_SERVO_ENBL      P2_2
#define SMART_SERVO_PHASE     P2_3
#define SMART_SERVO_SLEEP     P3_6
#define SMART_SERVO_NFAULT    P3_2
#endif

MOTOR_CTRL_STATUS g_eSysMotionStatus = IDLE_MODE;  

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

int16_t servodriver_get_nfault_value(void)
{
  int value = -1;
  value = digitalRead(SMART_SERVO_NFAULT);
  return value;
}

void servodriver_set_pwm(int16_t pwm)
{
  int16_t speed_temp = pwm;

  if(speed_temp > 0)//cw
  {
 	#if  MP6528
		pwm_write(SMART_SERVO_PWMA,0,0,MAX_OUTPUT_PWM);
		pwm_write(SMART_SERVO_PWMB,speed_temp,0,MAX_OUTPUT_PWM);
	#elif MP6515
		pwm_write(SMART_SERVO_ENBL,pwm,0,MAX_OUTPUT_PWM);
 	  digitalWrite(SMART_SERVO_PHASE,0);
 	#endif
  }
  else//ccw
  {
  #if  MP6528
		pwm_write(SMART_SERVO_PWMA,-speed_temp,0,MAX_OUTPUT_PWM);
		pwm_write(SMART_SERVO_PWMB,0,0,MAX_OUTPUT_PWM);
	#elif MP6515
		pwm_write(SMART_SERVO_ENBL,-pwm,0,MAX_OUTPUT_PWM);
    digitalWrite(SMART_SERVO_PHASE,1);
  #endif
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

void servodriver_run_debug(uint8_t mode,long param1,long param2,long param3)
{
  digitalWrite(SMART_SERVO_SLEEP,1);
  g_eSysMotionStatus = DEBUG_MODE;
  //g_eSysMotionStatus = (MOTOR_CTRL_STATUS)mode;

  g_servo_info.tar_speed = param1*10;
  g_servo_info.tar_pos = param2*10;
  g_servo_info.tar_pwm = param3;
}

