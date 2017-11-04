#include "servo_driver.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"



void servodriver_init(void)
{
  pinMode(SMART_SERVO_LED_R,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_LED_G,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_LED_B,GPIO_PMD_OUTPUT);
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

  pwm_init(SMART_SERVO_LED_R,1000);
  pwm_init(SMART_SERVO_LED_G,1000);
  pwm_init(SMART_SERVO_LED_B,1000);
  pwm_init(SMART_SERVO_PWMA,20000);  //20KHz  PWM Timer period = 50us
  pwm_init(SMART_SERVO_PWMB,20000);

  smart_servo_led(0,0,255);
}

void servodriver_led(uint8_t led_r,uint8_t led_g,uint8_t led_b)
{
  pwm_write(SMART_SERVO_LED_R,(255-led_r),0,255);
  pwm_write(SMART_SERVO_LED_G,(255-led_g),0,255);
  pwm_write(SMART_SERVO_LED_B,(255-led_b),0,255);
}

void servodriver_led_change(uint8_t led_color)
{
  switch(led_color)
  {
    case 0:
      smart_servo_led(255,0,0);
      break;
    case 1:
      smart_servo_led(0,255,0);
      break;
    case 2:
      smart_servo_led(0,0,255);
      break;
    default:
      smart_servo_led(0,0,0);
      break;
  }
}

void servodriver_led_blink(uint16_t blink_time,uint8_t led_r,uint8_t led_g,uint8_t led_b)
{
  if(millis() - pre_blink_time > blink_time)
  {
    if(blink_flag == false)
    {
      blink_flag = true;
    }
    else
    {
      blink_flag = false;
    }
    blink_count++;
    pre_blink_time = millis();
  }
  if(blink_flag == true)
  {
    smart_servo_led(led_r,led_g,led_b);
  }
  else
  {
    smart_servo_led(0,0,0);
  }
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

int16_t servodriver_get_nfault_value(void)
{
  int value = -1;
  value = digitalRead(SMART_SERVO_NFAULT);
  return value;
}

void servodriver_get_type(void)
{
  int8_t type0_index = -1;
  int8_t type1_index = -1;
  float smart_servo_type0_vol = 0.0;
  float smart_servo_type1_vol = 0.0;

  smart_servo_type0_vol = (float)analogRead(SMART_SERVO_TYPE0_AD)/4096*3300;//for servo type low bit
  type0_index = constrain(round((float)(smart_servo_type0_vol/206)), 0 ,15);
  smart_servo_type1_vol = (float)analogRead(SMART_SERVO_TYPE1_AD)/4096*3300;//for servo type low bit
  type1_index = constrain(round((float)(smart_servo_type1_vol/206)), 0 ,15);

  if(type0_index <= 9 && type0_index >= 0)
  {
    type0 = type0_index + '0';
  }
  else if(type0_index <= 15 && type0_index > 9)
  {
    type0 = (type0_index - 10) + 'A'; //max is 'F' - 16th type
  }

  if(type1_index <= 9 && type1_index >= 0)
  {
    type1 = type1_index + '0';
  }
  else if(type1_index <= 15 && type1_index > 9)
  {
    type1 = (type1_index - 10) + 'A'; //max is 'F' - 16th type
  }
}

char servodriver_get_Hardware_Version(void)
{
  char _Hardware_Version = '0';
  
  if(digitalRead(SMART_SERVO_HW_Verison_D1)
  {
    if(digitalRead(SMART_SERVO_HW_Verison_D0)  // 11
    {
      _Hardware_Version = '4';
    }
    else                              // 10
    {
      _Hardware_Version = '3';
    }
  }
  else
  {
    if(digitalRead(SMART_SERVO_HW_Verison_D0)  // 01
    {
      _Hardware_Version = '2';
    }
    else                              // 00
    {
      _Hardware_Version = '0';
    }
  }

  return _Hardware_Version;
}





