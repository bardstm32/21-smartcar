#include "zf_common_headfile.h"
int16 int_data[4];
void main()
{
	Car_Init();
	LED_On(LED1);
	while (1)
	{
		Menu_Display(int_data);
		voltage_warning_task(11.4f); // 设置目标电压为3.7V
		//motor_L(30);
	}
}

