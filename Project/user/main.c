#include "zf_common_headfile.h"
#define TEMP_BUFFER_SIZE 64
static uint8 temp_uart_buffer[TEMP_BUFFER_SIZE]; // 数据存放数组

void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	// 此处编写用户代码 例如外设初始化代码等
	ALL_init();
	// 初始化无线转串口

	// 此处编写用户代码 例如外设初始化代码等
	// PID_Init(&right_spid, 6.2f,  1.9f,  0.0f,  SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	while (1)
	{
		// Parameter_Debug(&left_spid.Kp,&left_spid.Ki,&left_spid.Kd,NULL);                 	// 获取编码器计数                       // 清空编码器计数
		ips114_show_float(64, 0, left_spid.Kp, 3, 2);
		ips114_show_uint16(64, 16, real_left);
	}
}
