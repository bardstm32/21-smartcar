#include "zf_common_headfile.h"
#define TEMP_BUFFER_SIZE  	64
static  uint8           	temp_uart_buffer[TEMP_BUFFER_SIZE];  // 数据存放数组

float kp,ki,kd;


void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	ips114_init();
	// 此处编写用户代码 例如外设初始化代码等
	
	// 初始化无线转串口
    wireless_uart_init();
	Parameter_Debug_Init();
	// 此处编写用户代码 例如外设初始化代码等

	while (1)
	{
		Parameter_Debug(&kp, &ki, &kd);
		ips114_show_float(64, 0, kp, 3, 2);
		ips114_show_float(64, 16, ki, 3, 2);
		ips114_show_float(64, 32, kd, 3, 2);
	}
}

