#ifndef __RGB_H__
#define __RGB_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"


void rgb_init(void);
void set_rgb(uint8_t led_r,uint8_t led_g,uint8_t led_b);
void user_set_rgb(uint8_t led_r,uint8_t led_g,uint8_t led_b);
void rgb_process(void);


#endif/* __SMARTSERVO_H__ */
