#include "zf_common_headfile.h"
uint16 adc[5]={0};
uint16 adc1[5]={0};
uint16 t;
void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	ALL_init();
//	PID_Init( &left_spid, 0.53, 0.023f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
//	PID_Init(&right_spid, 0.41, 0.023f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init( &left_spid, 1.20, 0.4f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&right_spid, 1.20, 0.4f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	while (1)
	{
		
	
		 ips114_show_uint16( 0,0,adc[1]);	
		 ips114_show_uint16( 0,16,adc[2]);
		 ips114_show_uint16( 0,32,adc[3]);
		 ips114_show_uint16( 0,48,adc[4]);
		
		Inductance_Read(adc);
		
		
		

		
//		 ips114_show_uint16( 0,64,adc_convert(L1));	
//		 ips114_show_uint16( 0,80,adc_convert(L2));
//		 ips114_show_uint16( 0,96,adc_convert(L3));
//		 ips114_show_uint16( 0,112,adc_convert(L4));



		
		//Parameter_Debug(&left_spid.Kp, &left_spid.Ki,&right_spid.Kp,&right_spid.Ki,&right_spid.target, &left_spid.target);
	}
}
