/*********************************************************************************************************************
 *  @file  Buzzer.c
 *  @brief  LED初始化与控制
 * 该文件包含了LED初始化、打开和关闭的相关函数。
 * 通过初始化指定的GPIO引脚来控制LED的开关状态。
 * 根据需要，可以在头文件中修改LED的引脚配置。
 *  @date              @author            备注
 *  2025-12-21        三帅勇闯智能车-rw   初始版本
 *  修改记录：
 *  @date              @author            备注
 *
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "LED.h"

/**
 * @brief 初始化LED,可根据头文件修改引脚
 * @param 无
 * @return 无
 */
void LED_Init(void)
{
    gpio_init(LED1, GPO, 1, GPO_PUSH_PULL);
    gpio_init(LED2, GPO, 1, GPO_PUSH_PULL);
}

/**
 * @brief 点亮LED
 * @param pin：LED引脚
 * @return 无
 */
void LED_On(gpio_pin_enum pin)
{
    gpio_set_level(pin, 0);
}

/**
 * @brief 熄灭LED
 * @param pin：LED引脚
 * @return 无
 */
void LED_Off(gpio_pin_enum pin)
{
    gpio_set_level(pin, 1);
}

void LED_Switch(gpio_pin_enum pin)
{
    gpio_set_level(pin, !(gpio_get_level(pin)));
}