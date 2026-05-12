#include "zf_common_headfile.h"
void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	ALL_init();
	PID_Init(&left_spid,  3.2,0,0.6f,0,SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&right_spid, 3.0,0,0.6f,0,SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&Turn_PID,   0.2,0.0001870,0,4.7, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&Gyro_PID,  4.1, 0, 0,18, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	gpio_init(IO_P77, GPO, GPIO_LOW, GPO_PUSH_PULL);
	
	P77=1;
	system_delay_ms(1);
	P77=0;
	system_delay_us(5);
	P77=1;
	
	while (1)
	{	
		Element_Control(adc_inductance);
//		Oscilloscope_Display(eleOut_0,imu660ra_gyro_z,eleOut_1,0,0,0,left_spid.target,right_spid.target);
		Send_Data_To_PC();
	}
	
	
}