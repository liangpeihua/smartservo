#ifndef __RGB_H__
#define __RGB_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"


void rgb_init(void);
void set_rgb(uint8_t led_r,uint8_t led_g,uint8_t led_b);
void rgb_blink(uint16_t blink_time,uint8_t led_r,uint8_t led_g,uint8_t led_b);


#endif/* __SMARTSERVO_H__ */
