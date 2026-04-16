#ifndef __INDUCTANCE_H__
#define __INDUCTANCE_H__

// 定义电感传感器连接的IO引脚
#define L1 (ADC_CH8_P00) // 电感传感器L1连接到IO引脚P0.0
#define L2 (ADC_CH9_P01) // 电感传感器L2连接到IO引脚P0.1
#define L3 (ADC_CH13_P05) // 电感传感器L3连接到IO引脚P0.5
#define L4 (ADC_CH14_P06) // 电感传感器L4连接到IO引脚P0.6

// 滤波深度，连读N次的电感进行滤波防止异常抖动
#define Filter_deepth 5

// 用于计算电感误差的缩放因子
#define Str_value 3.0f // 电感误差计算中的Str_value缩放因子
#define Cur_value 7.0f // 电感误差计算中的Cur_value缩放因子


int16 range_protect_int(int16 value, int16 min_value, int16 max_value);
float range_protect_float(float value, float min_value, float max_value);
void Inductance_Init(void);
int Median_Average_Filter(int *arr, int times);
double ADC_Normalize_0_100(uint16 adc_val, uint16 adc_max, uint16 adc_min);
float Inductance_Count_Err(int16 L, int16 LM, int16 RM, int16 R);
void Inductance_Read(uint16 *inductance_filter_data);

#endif // __INDUCTANCE_H__
