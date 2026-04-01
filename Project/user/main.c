#include "zf_common_headfile.h"
#define TEMP_BUFFER_SIZE 64
static uint8 temp_uart_buffer[TEMP_BUFFER_SIZE]; // 数据存放数组

void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	// 此处编写用户代码 例如外设初始化代码等
	ALL_init();
	//gpio_init(IO_P33, GPO, 0, GPO_PUSH_PULL);
	// 初始化无线转串口
	// 此处编写用户代码 例如外设初始化代码等
	PID_Init(&left_spid, 8.5f,0.15,0,SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&right_spid, 8.5f,0.15,0,SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	while (1)
	{

		Parameter_Debug(&left_spid.Kp, &left_spid.Ki,&right_spid.Kp,&right_spid.Ki,&left_spid.target, &right_spid.target);
//		ips114_show_int16(64,0,real_right);
//		ips114_show_int16(64,16,real_left);
//		ips114_show_int16(64,32,right_spid.target);
//		ips114_show_int16(64,48,left_spid.target);		// 获取编码器计数                       // 清空编码器计数
//		ips114_show_float(64,64,right_spid.Kp,2,2);
//		ips114_show_float(64,80,right_spid.Ki,2,2);
//		ips114_show_float(64,96,left_spid.Kp,2,2);
//		ips114_show_float(64,112,left_spid.Ki,2,2);
				
	}
}
