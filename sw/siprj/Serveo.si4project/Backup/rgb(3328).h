#ifndef __RGB_H__
#define __RGB_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"


//GPIO define
#define SMART_SERVO_LED_B     	P2_4
#define SMART_SERVO_LED_G     	P2_3
#define SMART_SERVO_LED_R     	P2_2

void rgb_init(void);
void set_rgb(uint8_t led_r,uint8_t led_g,uint8_t led_b);
void rgb_blink(uint16_t blink_time,uint8_t led_r,uint8_t led_g,uint8_t led_b);


#endif/* __SMARTSERVO_H__ */
