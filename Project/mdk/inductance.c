/*********************************************************************************************************************
 * @file inductance.c
 * @brief 电感数据读取与处理
 * 该文件包含了电感初始化、电感数据读取和处理的相关函数。
 * 电感传感器的数据通过ADC读取，并进行滤波和归一化处理，以提高数据精度和可靠性。
 * 适用平台          STC32G128K
 * @date              @author            备注
 * 2026-3-21     三帅勇闯智能车-rw         无
 * 修改记录：
 * @date              @author            备注
 * 
 ********************************************************************************************************************/
#include "zf_common_headfile.h"

/**
 * range_protect - 将值限制在指定的最小值和最大值之间
 * 该函数接收一个值以及最小值和最大值作为参数，如果值大于最大值，则返回最大值；
 * 如果值小于最小值，则返回最小值；否则返回该值本身。
 * @param value     待限制的值
 * @param min_value 最小值
 * @param max_value 最大值
 * @return          限制在[min_value, max_value]之间的值
 */
int8 range_protect(int8 value, int8 min_value, int8 max_value) {
    return (value > max_value) ? max_value : ((value < min_value) ? min_value : value);
}

/**
 * Inductance_Init - 初始化电感传感器
 * 该函数用于初始化四个电感传感器，每个传感器配置为12位ADC模式。
 * @return 无
 */
void Inductance_Init(void) {
    adc_init(L1, ADC_12BIT); // 初始化电感传感器L1为12位ADC模式
    adc_init(L2, ADC_12BIT); // 初始化电感传感器L2为12位ADC模式
    adc_init(L3, ADC_12BIT); // 初始化电感传感器L3为12位ADC模式
    adc_init(L4, ADC_12BIT); // 初始化电感传感器L4为12位ADC模式
}

/**
 * Median_Average_Filter - 计算数组中去掉一个最大值和一个最小值后的平均值
 * 该函数接收一个整型数组和一个整数times作为参数，计算数组中去掉一个最大值
 * 和一个最小值后的平均值。适用于需要滤除异常值的场景。
 * @param arr   整型数组，包含待处理的数据
 * @param times 需要处理的数组元素个数
 * @return      去掉一个最大值和一个最小值后的数组元素平均值
 */
int Median_Average_Filter(int *arr, int times)
{
    // 初始化最小值、最大值和累加和
    int min = arr[0], max = arr[0], sum = 0, i = 0;
    
    // 遍历数组，更新最小值、最大值，并计算累加和
    for(i = 0; i < times; i++) {
        if(arr[i] < min) min = arr[i]; // 更新最小值
        if(arr[i] > max) max = arr[i]; // 更新最大值
        sum += arr[i];                 // 计算累加和
    }
    
    // 返回去掉最大值和最小值后的平均值
    // 注意：该方法假设times至少为3，否则会导致除以零的错误
    return (sum - min - max) / (times - 2);
}

/**
 * Adc_Normalize - 将ADC值归一化到指定范围
 * 该函数接收一个ADC值以及最大值和最小值作为参数，将ADC值归一化到0到100之间，
 * 并使用range_protect函数确保结果在[1.0, 100.0]范围内。
 * @param value     ADC值
 * @param max_value 最大值
 * @param min_value 最小值
 * @return          归一化后的值，范围在[1.0, 100.0]之间
 */
uint8 ADC_Normalize_0_100(uint16 adc_val, uint16 adc_max, uint16 adc_min)
{
    uint16 temp;
    // 公式：(值 - 最小值) * 100 / (最大值 - 最小值)
    temp = (adc_val - adc_min) * 100 / (adc_max - adc_min);
    // 限幅，防止越界
    if(temp > 100) temp = 100;
    if(temp < 0)   temp = 0;
    return (uint8)temp;
}

/* Inductance_Count_Err - 差比和计算（按公式修改版）
 * 该函数接收四个电感值作为参数，按公式计算电感误差并将其限制在-100到100之间。
 * @param L  电感值L（对应公式中a1）
 * @param LM 电感值LM（对应公式中a2）
 * @param RM 电感值RM（对应公式中b1）
 * @param R  电感值R（对应公式中b2）
 * @return   电感误差值，范围在[-100, 100]之间
 */
int8 Inductance_Count_Err(uint16 L, uint16 LM, uint16 RM, uint16 R) 
{
    int8 scaled_err;
    // 1. 计算分子：√(a12+a22) - √(b12+b22)
    float sqrt_a = sqrt((float)L*L + (float)LM*LM);
    float sqrt_b = sqrt((float)R*R + (float)RM*RM);
    float numerator = sqrt_a - sqrt_b;
    float denominator = sqrt_a + sqrt_b;
    
    // 2. 计算并钳制结果
    if (denominator == 0.0f) {
        // 避免除零错误，直接返回0或根据需求处理
        scaled_err = 0;
    } else {
        scaled_err = ((int16)numerator / (int16)denominator) * 100;
    }
    // 使用range_protect函数确保结果在[-100.0, 100.0]范围内
    scaled_err = range_protect(scaled_err, -100, 100);
    // 返回电感误差值
    return scaled_err;
}

/**
 * Inductance_Read - 读取并滤波电感值
 * 该函数用于读取四个电感传感器的原始值，并使用Median_Average_Filter函数进行滤波处理。
 * @param inductance_filter_data 用于存储滤波后的电感值的数组
 * @return 无
 */
void Inductance_Read(uint16 *inductance_norm_data) 
{
    uint8 i; // 声明变量i，必须在作用域最前面
    // 原始值
    uint16 inductance_init_data[5][Filter_deepth];
    uint16 inductance_filter_data[5];
    // 原始值的读取
    for (i = 0; i < Filter_deepth; i++) {
        inductance_init_data[1][i] = adc_convert(L1); // 将电感传感器L1的值读取并存储
        inductance_init_data[2][i] = adc_convert(L2); // 将电感传感器L2的值读取并存储
        inductance_init_data[3][i] = adc_convert(L3); // 将电感传感器L3的值读取并存储
        inductance_init_data[4][i] = adc_convert(L4); // 将电感传感器L4的值读取并存储
    }
    
    // 滤波
    for (i = 1; i <= 4; i++) 
    {
        // 使用Median_Average_Filter函数进行滤波处理
        inductance_filter_data[i] =Median_Average_Filter(inductance_init_data[i],Filter_deepth);
    }
	
	inductance_norm_data[1] = ADC_Normalize_0_100(inductance_filter_data[1], 2400, 0);
    inductance_norm_data[2] = ADC_Normalize_0_100(inductance_filter_data[2], 2400, 0);
    inductance_norm_data[3] = ADC_Normalize_0_100(inductance_filter_data[3], 2400, 0);
    inductance_norm_data[4] = ADC_Normalize_0_100(inductance_filter_data[4], 2400, 0);
}
