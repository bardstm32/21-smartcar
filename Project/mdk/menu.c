/*********************************************************************************************************************
 * @file menu.c
 * @brief 屏幕菜单与数据无线发送功能
 * 该文件包含了IPS114液晶屏幕的菜单初始化与数据刷新函数。
 * 主要用于在屏幕和电脑上实时显示当前的偏航角速度、偏航角度以及四个电感传感器的数值，以便于车辆运行状态的监控与代码调试。
 * 适用平台          STC32G128K
 * @date                    @author                              
 * 2026-3-26     三帅勇闯智能车-rw  三帅勇闯智能车-dyq        
 * 修改记录：
 * @date              @author            备注
 * 2026-3-30     三帅勇闯智能车-rw  添加了波形显示函数        
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <stdlib.h>
#include <string.h>




// 通过无线串口发送可变数据的函数
// 将当前的偏航角速度、偏航角度以及四个电感传感器的数值通过无线串口发送到电脑端
void Send_Data_To_PC(void)
{
    // 1. 申请一个足够大的字符数组，用于存放格式化后的字符串
     // 用于存放格式化后的字符串的缓冲区
	char send_buf[128];
    // 2. 使用 sprintf 将数值格式化为字符串
    // 这里以浮点数保留2位小数 (%.2f)，整数 (%d) 为例。加上 \r\n 以便电脑端串口助手换行显示。
    sprintf(send_buf, "state: %d %d %d %d %d %f \r\n",TrackState,adc_inductance[1],adc_inductance[2]
	,adc_inductance[3],adc_inductance[4],Nowangel-Daty_Z);

    // 3. 调用库函数发送最终拼装好的字符串
    wireless_uart_send_string(send_buf); // 通过无线串口发送格式化后的字符串
}

// 参数调试初始化函数
// 初始化用于参数调试的无线串口接收和发送功能
void Parameter_Debug_Init(void)
{
    seekfree_assistant_receive_callback = wireless_uart_read_buffer; // 设置接收回调函数
    seekfree_assistant_init(); // 初始化参数调试助手
}

// 参数调试函数
// 从无线串口接收数据以更新参数值
void Parameter_Debug(float *param1, float *param2,float *param3, float *param4, int32 *target1, int32 *target2)
{
    uint8 i; // 循环计数器
    seekfree_assistant_data_analysis(); // 分析接收到的数据
    seekfree_assistant_parameter_update_flag[i] = 0; // 重置更新标志
    *param1 = seekfree_assistant_parameter[0] * 0.01f; // 更新参数1
    *param2 = seekfree_assistant_parameter[1] * 0.01f; // 更新参数2
    *param3 = seekfree_assistant_parameter[2] * 0.01f; // 更新参数3
	*param4 = seekfree_assistant_parameter[3] * 0.01f; // 更新参数3
    *target1 = seekfree_assistant_parameter[4]; // 更新速度参数
	*target2 = *target1;
           
}

// 示波器初始化函数
// 初始化用于发送示波器数据的无线串口功能
void Oscilloscope_Init(void)
{
    seekfree_assistant_transfer_callback = wireless_uart_send_buffer; // 设置发送回调函数
}

// 示波器数据发送函数
// 将两个通道的数据打包并通过无线串口发送到电脑端
void Oscilloscope_Display(float num1, float num2,float num3,int16 num4,int16 num5,int16 num6,float num7,float num8)
{
    seekfree_assistant_oscilloscope_data.dat[0] = num1; // 设置第一个通道的数据
    seekfree_assistant_oscilloscope_data.dat[1] = num2; // 设置第二个通道的数据
    seekfree_assistant_oscilloscope_data.dat[2] = num3; // 设置第二个通道的数据
	seekfree_assistant_oscilloscope_data.dat[3] = num4; // 设置第二个通道的数据
    seekfree_assistant_oscilloscope_data.dat[4] = num5; // 设置第二个通道的数据
	seekfree_assistant_oscilloscope_data.dat[5] = num6; // 设置第二个通道的数据
	seekfree_assistant_oscilloscope_data.dat[6] = num7; // 设置第二个通道的数据
    seekfree_assistant_oscilloscope_data.dat[7] = num8; // 设置第二个通道的数据
    seekfree_assistant_oscilloscope_data.channel_num =8; // 设置发送的通道数量为2
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data); // 发送示波器数据至电脑
}