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
    // 内部函数初始化
    clock_init(SYSTEM_CLOCK_30M);       // 系统时钟初始化
    debug_init();                       // 调试接口初始化
    interrupt_global_disable();         // 全局中断禁用
    gyro_init();                        // 陀螺仪初始化
    Menu_Init();                        // 菜单初始化
    LED_Init();                         // LED 初始化
	voltage_warning_init();
    pit_ms_init(TIM0_PIT, 1);           // 定时器初始化

    // 外部模块初始化
    Inductance_Init();                  // 电感初始化
    Buzzer_Init();                      // 蜂鸣器初始化
    wireless_uart_init();               // 无线UART通讯初始化（调试用）
	encoder_init();
	motor_init();
    interrupt_global_enable();            // 全局中断使能
}
