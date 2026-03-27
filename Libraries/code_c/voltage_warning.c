/*********************************************************************************************************************
 *  @file  voltage_warning.c
 *  @brief  电池电压检测与警告
 * 该文件包含了电池电压初始化、电压数据读取和处理的相关函数。
 * 电池电压通过ADC读取，并进行滤波和归一化处理，以提高数据精度和可靠性。
 * 当电池电压低于设定的目标电压时，开启蜂鸣器进行警告。
 *  @date                @author             备注
 *  2026-03-14        三帅勇闯智能车-rw      初始版本
 *  修改记录：
 *  @date              @author            备注
 *
 ********************************************************************************************************************/
#include "voltage_warning.h"

// 定义AD转换结果变量
uint16 ad_result = 0;

// 定义电池电压变量
float battery_voltage;

/**
 *  @brief  初始化电压警告功能
 *  该函数用于初始化指定的ADC通道，以便后续进行电压检测。
 *  @param  channel ADC通道枚举类型，表示要使用的ADC通道。
 *  @return  无
 */
void voltage_warning_init(void)
{
    // 初始化ADC通道为Battery_Channel，设置分辨率为12位
    adc_init(Battery_Channel, ADC_12BIT);
    gpio_init(IO_P01, GPI, 0, GPI_PULL_DOWN);
}

/**
 *  @brief  电压警告任务处理函数
 *  该函数定期读取ADC通道的转换结果，并根据电池电压判断是否需要响铃警告。
 *  @param  无
 *  @return  无
 *  @instruction:放在中断里面读取
 */
void voltage_warning_task(float target_voltage)
{
    char voltage_buf[128];

    // 使用均值滤波方法读取Battery_Channel的转换结果，采样次数为10次
    ad_result = adc_mean_filter_convert(Battery_Channel, 10);

    // 计算电池电压
    // ADC满量程为4096，参考电压为5伏，放大倍数为6
    battery_voltage = ((ad_result * 30) / 4096.0);

    // 如果电池电压低于设定值，则开启蜂鸣器进行警告
    if (battery_voltage <= target_voltage)
    {
        Buzzer_On();
    }
    sprintf(voltage_buf, "voltage: %f\r\n", battery_voltage);
    wireless_uart_send_string(voltage_buf);
}
