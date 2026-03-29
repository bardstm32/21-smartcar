#include "zf_common_headfile.h"
uint16 ADC_value[5];
void main()
{
    clock_init(SYSTEM_CLOCK_30M);
	debug_init();	
	ALL_init();

    PID_Init(&pos_pid,    2.8f,  0.15f, 0.6f,  POS_PID_MAX_OUT, POS_PID_MAX_I);
    PID_Init(&left_spid,  6.0f,  0.8f,  0.0f,  SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
    PID_Init(&right_spid, 6.0f,  0.8f,  0.0f,  SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
    while(1)
    {
		Menu_Display(ADC_value);
	}
}



