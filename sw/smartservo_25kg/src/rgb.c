#include "rgb.h"
#include "systime.h"
#include "uart_printf.h"
#include "math.h"
#include "mp9960.h"
#include "protocol.h"
#include "main.h"
#include "servo_detect.h"
#include "servo_driver.h"

uint8_t g_user_setrgb = 0;
uint8_t user_set_r, user_set_g, user_set_b;


//GPIO define
#if MP6528
#define SMART_SERVO_LED_B     	P2_4
#define SMART_SERVO_LED_G     	P2_3
#define SMART_SERVO_LED_R     	P2_2
#elif MP6515
#define SMART_SERVO_LED_B     P2_4
#define SMART_SERVO_LED_R     P2_6
#define SMART_SERVO_LED_G     P2_5
#endif

extern volatile boolean shake_hand_flag;

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

void user_set_rgb(uint8_t led_r,uint8_t led_g,uint8_t led_b)
{
	g_user_setrgb = 1;
  user_set_r = led_r;
  user_set_g = led_g;
  user_set_b = led_b;
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

//5ms
void rgb_process(void)
{
	static uint8_t flicker_cnt = 0;

	if(g_user_setrgb == 1)
	{
		set_rgb(user_set_r, user_set_g, user_set_b);
	}
	//3.舵机收到握手命令时，白灯闪烁三次
	else if(shake_hand_flag == true)
	{
		static uint8_t count = 0;
		++count;
		if(count == 1)
		{
			set_rgb(255,255,255);
		}
		else if(count == 80)//400ms
		{
			set_rgb(0,0,0);
		}
		else if(count == 200)//1000ms
		{
			count = 0;
			flicker_cnt++;
		}

		if(flicker_cnt == 3)
		{
			shake_hand_flag = false;
			flicker_cnt = 0;
			count = 0;
		}
	}
	//2.舵机处于报警状态时，红灯闪烁
	else if(g_servo_info.errorid != 0)
	{
		static uint8_t count = 0;
		++count;
		if(count == 1)
		{
			set_rgb(255,0,0);
		}
		else if(count == 80)//400ms
		{
			set_rgb(0,0,0);
		}
		else if(count == 200)//1000ms
		{
			count = 0;
		}
	}
	//1.舵机处于正常工作时，亮蓝灯
	else
	{
		set_rgb(0,0,255);
	}
}


