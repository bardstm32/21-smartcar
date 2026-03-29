#include "zf_common_headfile.h"
/**
  * @brief 初始化蜂鸣器,可根据头文件修改引脚
  * @param 无
  * @return 无
  */
void Buzzer_Init(void)
{
    gpio_init(Buzzer, GPO, 1, GPO_PUSH_PULL);
}
/**
 * @brief 打开蜂鸣器
 * @param pin：蜂鸣器引脚
 * @return 无
 */
void Buzzer_On(void)
{
    gpio_set_level(Buzzer, 0);
}
/**
 * @brief 关闭蜂鸣器
 * @param pin：蜂鸣器引脚
 * @return 无
 */
void Buzzer_Off(void)
{
    gpio_set_level(Buzzer, 1);
}
