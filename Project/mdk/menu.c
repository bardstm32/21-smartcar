#include "zf_common_headfile.h"
#include <stdlib.h>
#include <string.h>





/* 把状态机和四路电感拼成字符串经无线串口上传 */
void Send_Data_To_PC(void)
{

	char send_buf[128];

    sprintf(send_buf, "state: %d %d %d %d %d %d %d \r\n",TrackState,adc_inductance[1],adc_inductance[2]
	,adc_inductance[3],adc_inductance[4],speed_left,speed_right);


    wireless_uart_send_string(send_buf);
}


/* 初始化逐飞参数调试助手的接收回调 */
void Parameter_Debug_Init(void)
{
    seekfree_assistant_receive_callback = wireless_uart_read_buffer;
    seekfree_assistant_init();
}


/* 把上位机下发的参数解析到外部变量 */
void Parameter_Debug(float *param1, float *param2,float *param3, float *param4, int32 *target1, int32 *target2)
{
    uint8 i;
    seekfree_assistant_data_analysis();
    seekfree_assistant_parameter_update_flag[i] = 0;
    *param1 = seekfree_assistant_parameter[0] * 0.01f;
    *param2 = seekfree_assistant_parameter[1] * 0.01f;
    *param3 = seekfree_assistant_parameter[2] * 0.01f;
	*param4 = seekfree_assistant_parameter[3] * 0.01f;
    *target1 = seekfree_assistant_parameter[4];
	*target2 = *target1;
           
}


/* 注册逐飞示波器的发送回调 */
void Oscilloscope_Init(void)
{
    seekfree_assistant_transfer_callback = wireless_uart_send_buffer;
}


/* 8 通道波形上传 */
void Oscilloscope_Display(float num1, float num2,int16 num3,int16 num4,int16 num5,float num6,float num7,int16 num8)
{
    seekfree_assistant_oscilloscope_data.dat[0] = num1;
    seekfree_assistant_oscilloscope_data.dat[1] = num2;
    seekfree_assistant_oscilloscope_data.dat[2] = num3;
	seekfree_assistant_oscilloscope_data.dat[3] = num4;
    seekfree_assistant_oscilloscope_data.dat[4] = num5;
	seekfree_assistant_oscilloscope_data.dat[5] = num6;
	seekfree_assistant_oscilloscope_data.dat[6] = num7;
    seekfree_assistant_oscilloscope_data.dat[7] = num8;
    seekfree_assistant_oscilloscope_data.channel_num =8;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}