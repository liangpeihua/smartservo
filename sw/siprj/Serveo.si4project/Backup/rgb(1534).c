#include "rgb.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"

volatile uint16_t blink_count = 0;
volatile long pre_blink_time = 0;
static volatile boolean blink_flag = false;

void rgb_init(void)
{
  pinMode(SMART_SERVO_LED_R,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_LED_G,GPIO_PMD_OUTPUT);
  pinMode(SMART_SERVO_LED_B,GPIO_PMD_OUTPUT);

  pwm_init(SMART_SERVO_LED_R,1000);
  pwm_init(SMART_SERVO_LED_G,1000);
  pwm_init(SMART_SERVO_LED_B,1000);

  set_rgb(0,0,255);
}

void set_rgb(uint8_t led_r,uint8_t led_g,uint8_t led_b)
{
  pwm_write(SMART_SERVO_LED_R,(255-led_r),0,255);
  pwm_write(SMART_SERVO_LED_G,(255-led_g),0,255);
  pwm_write(SMART_SERVO_LED_B,(255-led_b),0,255);
}

void rgb_change(uint8_t led_color)
{
  switch(led_color)
  {
    case 0:
      set_rgb(255,0,0);
      break;
    case 1:
      set_rgb(0,255,0);
      break;
    case 2:
      set_rgb(0,0,255);
      break;
    default:
      set_rgb(0,0,0);
      break;
  }
}

void rgb_blink(uint16_t blink_time,uint8_t led_r,uint8_t led_g,uint8_t led_b)
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
    set_rgb(led_r,led_g,led_b);
  }
  else
  {
    set_rgb(0,0,0);
  }
}


