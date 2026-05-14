#ifndef __INDUCTANCE_H__
#define __INDUCTANCE_H__

// �����д��������ӵ�IO����
#define L1 (ADC_CH14_P06) // ��д�����L1���ӵ�IO����P0.0
#define L2 (ADC_CH13_P05) // ��д�����L2���ӵ�IO����P0.1
#define L3 (ADC_CH4_P14) // ��д�����L3���ӵ�IO����P0.5
#define L4 (ADC_CH5_P15) // ��д�����L4���ӵ�IO����P0.6

// �˲���ȣ�����N�εĵ�н����˲���ֹ�쳣����
#define Filter_deepth 5


int16 range_protect_int(int16 value, int16 min_value, int16 max_value);
float range_protect_float(float value, float min_value, float max_value);
void Inductance_Init(void);
int Median_Average_Filter(int *arr, int times);
float ADC_Normalize_0_100(uint16 adc_val, uint16 adc_max, uint16 adc_min);
float Inductance_Count_Err(int16 L, int16 LM, int16 RM, int16 R);
void Inductance_Read(uint16 *inductance_filter_data);
float Inductance_Count_Err2(int16 L, int16 LM, int16 RM, int16 R);

#endif // __INDUCTANCE_H__
