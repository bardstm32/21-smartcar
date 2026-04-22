#include "zf_common_headfile.h"
void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	ALL_init();
	PID_Init(&left_spid,  3.2, 0.6f,   0,  SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&right_spid, 3.0, 0.6f,   0,  SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&Turn_PID, 1.2, 0,120, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	gpio_init(IO_P77, GPO, GPIO_LOW, GPO_PUSH_PULL);
	P77=1;
	system_delay_ms(1);
	P77=0;
	system_delay_us(5);
	P77=1;
	while (1)
	{
		Motor_SetSpeed(1800,-1800);
		Oscilloscope_Display(Turn_PID.P,Turn_PID.D,Turn_PID.out,Turn_PID.err,speed_left,speed_right,imu660ra_gyro_z);
	}
	
	
}