#include "zf_common_headfile.h"
int ADC[5]={0};
uint16 state = 0;
void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	ALL_init();
	PID_Init(&left_spid,  3.2,0, 0.6f,   0,  SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&right_spid, 3.0,0,0.6f,   0,  SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&Turn_PID,   0.23,0.0000789,0,4.10, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	gpio_init(IO_P77, GPO, GPIO_LOW, GPO_PUSH_PULL);
	
	P77=1;
	system_delay_ms(1);
	P77=0;
	system_delay_us(5);
	P77=1;
	while (1)
	{
		//system_delay_ms(2000);
		Send_Data_To_PC();
//		Element_Control(adc_inductance);
		//Inductance_Read(ADC);
//		if((state != 1) &&(adc_inductance[4]>=58) && (adc_inductance[1]>=50))
//		{
//			EA = 0;
//			Motor_SetSpeed(1500,800);
//			system_delay_ms(50);
//			EA = 1;
//			state = 1;
//		}
	}
	
	
}