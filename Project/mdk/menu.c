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
uint8 data_buffer[32];
uint8 gyro_buffer[32];

#define TEMP_BUFFER_SIZE 64
static uint8 temp_uart_buffer[TEMP_BUFFER_SIZE]; // 数据存放数组

char send_str[32] = {0};

// 菜单初始化函数
// 初始化IPS114液晶屏幕，并显示静态字符串标签以避免主循环刷新导致的屏幕闪烁
void Menu_Init(void)
{
    ips114_init();                        // 初始化IPS114屏幕
    ips114_clear(IPS114_DEFAULT_BGCOLOR); // 清除屏幕为默认背景色

    // 提前显示静态的字符串标签，避免在主循环中重复刷新导致屏幕闪烁
    // 默认字体下每个字符宽8像素，高16像素
    ips114_show_string(0, 0, "Gyro Z :");    // 偏航角速度标签
    ips114_show_string(0, 16, "Yaw Ang:");   // 偏航角标签
    ips114_show_string(0, 32, "L1:");        // 电感L1标签
    ips114_show_string(72, 32, "L2:");       // 电感L2标签
    ips114_show_string(0, 48, "L3:");        // 电感L3标签
    ips114_show_string(72, 48, "L4:");       // 电感L4标签
    ips114_show_string(0, 64, "Encoder_L:"); // 左编码器标签
    ips114_show_string(0, 80, "Encoder_R:"); // 右编码器标签
}

// 菜单数据显示更新函数
// 更新IPS114液晶屏幕上显示的偏航角速度、偏航角度以及四个电感传感器的数值
void Menu_Display(uint16 *inductance_data)
{
    // 1. 显示当前偏航角速度 (来自 IMU_Data.gyro_z)
    // 使用 ips114_show_float，x=64(刚好接在标签后面), y=0，整数位保留3位，小数位保留2位
    ips114_show_float(64, 0, IMU_Data.gyro_z, 3, 2); // 显示偏航角速度
    gyro_buffer[0] = IMU_Data.gyro_z; // 将偏航角速度存储到缓冲区

    // 2. 显示当前偏航角度 (来自 Daty_Z)
    // 偏航角范围大致在 -180 到 180 之间，整数位预留4位(包含负号)，小数位2位
    ips114_show_float(64, 16, Daty_Z, 4, 2); // 显示偏航角度
    gyro_buffer[1] = Daty_Z; // 将偏航角度存储到缓冲区

    // 3. 显示四个电感的数值 (假设取值范围是ADC的0-4095)
    // ips114_show_uint16 不带对齐功能，如有前置零残留问题，可在外部写个格式化补空格，这里直接调用基本接口
    ips114_show_uint16(24, 32, inductance_data[0]); // L1 数值
    ips114_show_uint16(96, 32, inductance_data[1]); // L2 数值
    ips114_show_uint16(24, 48, inductance_data[2]); // L3 数值
    ips114_show_uint16(96, 48, inductance_data[3]); // L4 数值
    ips114_show_int16(88, 64, speed_left);           // 左编码器数值
    ips114_show_int16(88, 80, speed_right);          // 右编码数值

    //    Send_Data_To_PC(inductance_data); // 将数据通过无线串口发送到电脑端
}

// 通过无线串口发送可变数据的函数
// 将当前的偏航角速度、偏航角度以及四个电感传感器的数值通过无线串口发送到电脑端
void Send_Data_To_PC(uint16 *inductance_data)
{
    // 1. 申请一个足够大的字符数组，用于存放格式化后的字符串
    char send_buf[128]; // 用于存放格式化后的字符串的缓冲区

    // 2. 使用 sprintf 将数值格式化为字符串
    // 这里以浮点数保留2位小数 (%.2f)，整数 (%d) 为例。加上 \r\n 以便电脑端串口助手换行显示。
    sprintf(send_buf, "Yaw: %.2f, GyroZ: %.2f, L1:%d, L2:%d, L3:%d, L4:%d, EL:%d, ER:%d\r\n",
            Daty_Z, // 当前偏航角度
            IMU_Data.gyro_z, // 当前偏航角速度
            inductance_data[0], // 电感L1的数值
            inductance_data[1], // 电感L2的数值
            inductance_data[2], // 电感L3的数值
            inductance_data[3] // 电感L4的数值
    );

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
void Oscilloscope_Display(float num1, float num2,float num3,float num4,int32 num5,int32 num6,float num7,int32 num8)
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