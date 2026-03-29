#include "zf_common_headfile.h"
uint16 ADC_value[5]={0};
void main()
{
    clock_init(SYSTEM_CLOCK_30M);
	debug_init();	
	ALL_init();
	ips114_init();
    PID_Init(&pos_pid,    2.8f,  0.15f, 0.6f,  POS_PID_MAX_OUT, POS_PID_MAX_I);
    PID_Init(&left_spid,  2.0f,  0.1f,  0.0f,  SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
    PID_Init(&right_spid, 2.0f,  0.1f,  0.0f,  SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	ips114_clear(IPS114_DEFAULT_BGCOLOR);
	Inductance_Init();
    while(1){
//		Menu_Display(ADC_value);
//		ADC_value[0]=adc_convert(ADC_CH8_P00);
//		ADC_value[1]=adc_convert(ADC_CH9_P01);
//		ADC_value[2]=adc_convert(ADC_CH13_P05);
//		ADC_value[3]=adc_convert(ADC_CH14_P06);
		ips114_show_int16(0,0,real_left);
		ips114_show_int16(0,16,real_right);
		Motor_SetSpeed(500,500);
		
		    }
}



