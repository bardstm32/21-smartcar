#include "zf_device_type.h"

wireless_type_enum  wireless_type	= NO_WIRELESS;
tof_type_enum       tof_type 		= NO_TOF;
gps_type_enum       gps_type 		= NO_GPS;

void (*tof_module_exti_handler)(uint8 dat) = NULL;			// ToF 模块 INT 更新中断

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置无线模块类型
// 参数说明     type_set        选定的无线模块类型
// 参数说明     uart_callback   设备的串口回调函数
// 返回参数     void
// 使用示例     set_wireless_type(WIRELESS_UART, uart_callback);
// 备注信息     一般由各摄像头初始化内部调用
//-------------------------------------------------------------------------------------------------------------------
void set_wireless_type (wireless_type_enum type_set, uart_index_enum uartx, void *uart_callback)
{
    wireless_type = type_set;
    
    if(uartx == UART_1)
    {
        uart1_irq_handler = uart_callback;
    }
    else if(uartx == UART_2)
    {
        uart2_irq_handler = uart_callback;
    }
    else if(uartx == UART_3)
    {
        uart3_irq_handler = uart_callback;
    }
    else if(uartx == UART_4)
    {
        uart4_irq_handler = uart_callback;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置 ToF 模块类型
// 参数说明     type_set        选定的 ToF 模块类型
// 参数说明     exti_callback   设备的外部中断回调函数
// 返回参数     void
// 使用示例     set_tof_type(TOF_DL1A, dl1a_int_handler);
// 备注信息     一般由各摄像头初始化内部调用
//-------------------------------------------------------------------------------------------------------------------
void set_tof_type (tof_type_enum type_set, void *exti_callback)
{
    tof_type = type_set;
    tof_module_exti_handler = exti_callback;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置 ToF 模块类型
// 参数说明     type_set        选定的 ToF 模块类型
// 参数说明     exti_callback   设备的外部中断回调函数
// 返回参数     void
// 使用示例     set_tof_type(TOF_DL1A, dl1a_int_handler);
// 备注信息     一般由各摄像头初始化内部调用
//-------------------------------------------------------------------------------------------------------------------
void set_gps_type (gps_type_enum type_set, uart_index_enum uartx, void *uart_callback)
{
    gps_type = type_set;
    if(uartx == UART_1)
    {
        uart1_irq_handler = uart_callback;
    }
    else if(uartx == UART_2)
    {
        uart2_irq_handler = uart_callback;
    }
    else if(uartx == UART_3)
    {
        uart3_irq_handler = uart_callback;
    }
    else if(uartx == UART_4)
    {
        uart4_irq_handler = uart_callback;
    }
}

