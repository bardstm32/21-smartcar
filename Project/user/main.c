#include "zf_common_headfile.h"
/* 入口：初始化外设与 PID，主循环按 ind_10ms_flag 调度电感读取与状态机 */
uint8 send_div = 0;
void main()
{	
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	ALL_init();


	while (1)
	{
		Element_Control(adc_inductance);
		Motor_Protect(adc_inductance);
//		Oscilloscope_Display(eleOut_0,eleOut_1,speed_left,speed_right,
//					left_spid.out,imu660ra_gyro_z - Gyro_Offset.Zdata,elemid,Gyro_PID.D);

		Send_Data_To_PC();

		
		
		
		
		
		
		
		
		
		
		
		
		
		
	}
}