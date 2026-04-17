#include "zf_common_headfile.h"

uint16 t=0;
void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	ALL_init();
	PID_Init(&left_spid, 3.2, 0.6f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&right_spid, 3.0, 0.6f,  0, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&Turn_PID, 1.0, 0, 2.5, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);

	while (1)
	{

		//printf("%f\r\n",first_complement.angle.pitch);
		Parameter_Debug(&Turn_PID.Kp, &Turn_PID.Kd);
	}
}
