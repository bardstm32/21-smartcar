#include "zf_common_headfile.h"
int16 adc[5] = {0};

//int16 T[5] = {0};
void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	ALL_init();
	PID_Init(&left_spid, 3.2, 0.6f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&right_spid, 3.0, 0.6f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&Turn_PID, 0.8, 0, 2.0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	while (1)
	{
		ips114_show_uint16(0,0,adc_convert(L1));
		ips114_show_uint16(0,16,adc_convert(L2));
		ips114_show_uint16(0,32,adc_convert(L3));
		ips114_show_uint16(0,48,adc_convert(L4));
		Inductance_Read(adc);
		ips114_show_uint16(70,0,adc[1]);
		ips114_show_uint16(70,16,adc[2]);
		ips114_show_uint16(70,32,adc[3]);
		ips114_show_uint16(70,48,adc[4]);
		//printf("%f\r\n",first_complement.angle.pitch);
		//Parameter_Debug(&Turn_PID.Kp, &Turn_PID.Kd);
	}
}