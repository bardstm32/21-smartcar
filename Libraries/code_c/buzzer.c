/*********************************************************************************************************************
 *  @file  Buzzer.c
 *  @brief  蜂鸣器初始化与控制
 * 该文件包含了蜂鸣器初始化、打开和关闭的相关函数。
 * 通过初始化指定的GPIO引脚来控制蜂鸣器的开关状态。
 * 根据需要，可以在头文件中修改蜂鸣器的引脚配置。
 *  @date              @author            备注
 *  2025-12-21        三帅勇闯智能车-rw   初始版本
 *  修改记录：
 *  @date              @author            备注
 * 
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "Buzzer.h"

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
