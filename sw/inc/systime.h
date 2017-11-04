#ifndef __SYSTIME_H__
#define __SYSTIME_H__

#include "M051Series.h"
extern volatile uint32_t system_time;

extern uint32_t millis(void);
extern uint32_t us100(void);
extern void delayMicroseconds(uint32_t us);
extern void delay(uint32_t ms);
#endif //__SYSTIME_H__

