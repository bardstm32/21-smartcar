
#include "zf_common_headfile.h"

/* 初始化 LED 引脚为推挽输出，默认熄灭 */
void LED_Init(void)
{
    gpio_init(LED1, GPO, 1, GPO_PUSH_PULL);
    gpio_init(LED2, GPO, 1, GPO_PUSH_PULL);
}

/* 点亮（共阳极接法，输出低电平） */
void LED_On(gpio_pin_enum pin)
{
    gpio_set_level(pin, 0);
}

/* 熄灭（共阳极接法，输出高电平） */
void LED_Off(gpio_pin_enum pin)
{
    gpio_set_level(pin, 1);
}

/* 翻转当前电平 */
void LED_Switch(gpio_pin_enum pin)
{
    gpio_set_level(pin, !(gpio_get_level(pin)));
}
