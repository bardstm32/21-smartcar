#include "zf_common_headfile.h"
uint16 adc[5] = {0};
uint16 adc1[5] = {0};
uint16 t;
void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	ALL_init();
	//	PID_Init( &left_spid, 0.53, 0.023f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	//	PID_Init(&right_spid, 0.41, 0.023f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&Turn_PID, 5, 0, 0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&left_spid, 1.20, 0.4f, 0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&right_spid, 1.20, 0.4f, 0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	while (1)
	{
		//Parameter_Debug(&Turn_PID.Kp, &Turn_PID.Ki,&Turn_PID.Kp,&right_spid.Ki,&right_spid.target, &left_spid.target);
	}
}
