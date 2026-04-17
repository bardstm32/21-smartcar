#include "zf_common_headfile.h"

uint16 t=0;
void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	ALL_init();
	PID_Init(&left_spid, 3.2, 0.6f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&right_spid, 3.0, 0.6f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&Turn_PID, 3, 0, 0.5, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
//	P77=1;
//	system_delay_ms(2);
//	P77=0;
//	system_delay_us(10);
//	P77=1;
	
	while (1)
	{
	    Oscilloscope_Display(left_spid.out,right_spid.out,left_spid.target,right_spid.target);

		//printf("%f\r\n",first_complement.angle.pitch);
//		Parameter_Debug(&left_spid.Kp, &left_spid.Ki,&right_spid.Kp,&right_spid.Ki,&right_spid.target, &left_spid.target);
	}
}
