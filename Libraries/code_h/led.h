#ifndef __LED_H
#define __LED_H
/**/
#include "zf_common_headfile.h"

#define LED1 (IO_P52)
#define LED2 (IO_P37)

void LED_Init(void);
void LED_On(gpio_pin_enum pin);
void LED_Off(gpio_pin_enum pin);
void LED_Switch(gpio_pin_enum pin);

#endif 