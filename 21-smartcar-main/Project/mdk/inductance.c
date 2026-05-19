
#include "zf_common_headfile.h"

uint16 inductance_init_data[5][Filter_deepth];
uint16 inductance_filter_data[5];
int16 CUR_PARA =1, STR_PARA = 2;

/* 把 int16 限制在 [min, max] 区间 */
int16 range_protect_int(int16 value, int16 min_value, int16 max_value)
{
    return (value > max_value) ? max_value : ((value < min_value) ? min_value : value);
}

/* 把 float 限制在 [min, max] 区间 */
float range_protect_float(float value, float min_value, float max_value)
{
    return (value > max_value) ? max_value : ((value < min_value) ? min_value : value);
}

/* 把 L1~L4 四路 ADC 通道初始化为 12 位模式 */
void Inductance_Init(void)
{
    adc_init(L1, ADC_12BIT);
    adc_init(L2, ADC_12BIT);
    adc_init(L3, ADC_12BIT);
    adc_init(L4, ADC_12BIT);
}

/* 中值平均：去掉一个最大值和一个最小值，剩余取平均 */
int Median_Average_Filter(uint16 *arr, int times)
{
    int min = arr[0], max = arr[0], sum = 0, i = 0;

    for (i = 0; i < times; i++)
    {
        if (arr[i] < min) min = arr[i];
        if (arr[i] > max) max = arr[i];
        sum += arr[i];
    }

    return (sum - min - max) / (times - 2);
}

/* 把 ADC 值线性归一化到 [0,100]，越界饱和 */
float ADC_Normalize_0_100(uint16 adc_val, uint16 adc_max, uint16 adc_min)
{
    float temp;

    if (adc_val <= adc_min)
        return 0.0f;

    if (adc_val >= adc_max)
        return 100.0f;

    temp = ((adc_val - adc_min) * 100.0f / (adc_max - adc_min));

    return temp;
}

/* 方向偏差（基础版，纯整数计算后转 float） */
float Inductance_Count_Err(int16 L, int16 LM, int16 RM, int16 R)
{
    int32 num = ((int32)(L - R) + (int32)CUR_PARA * (LM - RM)) * 10000;
    int32 den = (L + R) + IABS(LM - RM) + 1;
    int32 res = num / den;
    if (res >  10000) res =  10000;
    if (res < -10000) res = -10000;
    return (float)res;
}

/* 方向偏差（加权版，由 STR_PARA / CUR_PARA 调直道与弯道权重） */
float Inductance_Count_Err2(int16 L, int16 LM, int16 RM, int16 R)
{
    int32 num = ((int32)STR_PARA * (L - R) + (int32)CUR_PARA * (LM - RM)) * 1000;
    int32 den = (int32)STR_PARA * (L + R) + (int32)CUR_PARA * IABS(LM - RM) + 1;
    int32 res = num / den;
    if (res >  1000) res =  1000;
    if (res < -1000) res = -1000;
    return (float)res;
}

/* 四路电感各采样 Filter_deepth 次 → 中值平均 → 归一化 */
void Inductance_Read(uint16 *inductance_norm_data)
{
    uint8 i;

    for (i = 0; i < Filter_deepth; i++)
    {
        inductance_init_data[1][i] = adc_convert(L1);
        inductance_init_data[2][i] = adc_convert(L2);
        inductance_init_data[3][i] = adc_convert(L3);
        inductance_init_data[4][i] = adc_convert(L4);
    }

    for (i = 1; i <= 4; i++)
    {
        inductance_filter_data[i] = Median_Average_Filter(inductance_init_data[i], Filter_deepth);
    }

    inductance_norm_data[1] = ADC_Normalize_0_100(inductance_filter_data[1], 3675, 1);
    inductance_norm_data[2] = ADC_Normalize_0_100(inductance_filter_data[2], 3673, 1);
    inductance_norm_data[3] = ADC_Normalize_0_100(inductance_filter_data[3], 3685, 1);
    inductance_norm_data[4] = ADC_Normalize_0_100(inductance_filter_data[4], 3676, 3);
}
