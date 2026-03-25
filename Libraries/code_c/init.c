/*********************************************************************************************************************
 *  @file  init.c
 *  @brief  初始化各个模块
 * 初始化智能车系统
 * 该函数用于初始化智能车系统中的各个模块，包括系统时钟、调试接口、中断、电感传感器、蜂鸣器、IMU传感器、陀螺仪和无线通讯模块。
 * 初始化完成后，启用全局中断。
 *  @date              @author            备注
 *  2026-3-23        三帅勇闯智能车-rw   初始版本
 *  修改记录：
 *  @date              @author            备注
 *
 ********************************************************************************************************************/
#include "init.h"

void Car_Init(void)
{
    clock_init(SYSTEM_CLOCK_30M);
    debug_init();
    interrupt_global_disable();
    Inductance_Init();                                                  // 电感初始化
    Buzzer_Init();                                                      // 蜂鸣器初始化
    imu660ra_init();
    gyro_init();
    wireless_uart_init(); // 无线通讯（调试用）
    pit_ms_init(TIM0_PIT, 1);
    interrupt_global_enable();
}