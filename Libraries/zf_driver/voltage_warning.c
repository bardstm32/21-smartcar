#include "voltage_warning.h"
#include "zf_driver_pwm.h"

/**
 * @brief 初始化电压警告功能
 * 该函数用于初始化指定的ADC通道，并根据读取的电压值决定是否开启蜂鸣器报警。
 * @param channel ADC通道枚举类型，表示要使用的ADC通道。
 * @return 无
 */
 

void voltage_warning_init(adc_channel_enum channel)
{	
    unsigned int value = 0;  // 声明一个无符号整数变量value，用于存储ADC转换后的电压值

    // 调用adc_mean_filter_convert函数，读取指定ADC通道的电压值，并应用5次采样的平均滤波
    value = adc_mean_filter_convert(channel,1);
    // 检查电压值是否低于1575（假设1575是一个预设的阈值，即11.4V，用于触发电压警告）
    if(value < 1575)
    {
    Buzzer_On();  // 如果电压值低于阈值，则开启蜂鸣器报警
//	pwm_set_duty(PWMA_CH1P_P60,0);    
//	pwm_set_duty(PWMA_CH2P_P62,0);    
	
    }

}

