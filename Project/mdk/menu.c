/*********************************************************************************************************************
 * @file menu.c
 * @brief 屏幕菜单与数据无线发送功能
 * 该文件包含了IPS114液晶屏幕的菜单初始化与数据刷新函数。
 * 主要用于在屏幕和电脑上实时显示当前的偏航角速度、偏航角度以及四个电感传感器的数值，以便于车辆运行状态的监控与代码调试。
 * 适用平台          STC32G128K
 * @date              @author                                备注
 * 2026-3-26     三帅勇闯智能车-rw  三帅勇闯智能车-dyq         无
 * 修改记录：
 * @date              @author            备注
 * ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <stdlib.h>
#include <string.h>
uint8 data_buffer[32];
uint8 gyro_buffer[32];

#define TEMP_BUFFER_SIZE 64
static uint8 temp_uart_buffer[TEMP_BUFFER_SIZE]; // 数据存放数组

char send_str[32] = {0};


// 菜单初始化函数
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
void Menu_Display(uint16 *inductance_data)
{
    // 1. 显示当前偏航角速度 (来自 IMU_Data.gyro_z)
    // 使用 ips114_show_float，x=64(刚好接在标签后面), y=0，整数位保留3位，小数位保留2位
    ips114_show_float(64, 0, IMU_Data.gyro_z, 3, 2);
    gyro_buffer[0] = IMU_Data.gyro_z;
    // 2. 显示当前偏航角度 (来自 Daty_Z)
    // 偏航角范围大致在 -180 到 180 之间，整数位预留4位(包含负号)，小数位2位
    ips114_show_float(64, 16, Daty_Z, 4, 2);
    gyro_buffer[1] = Daty_Z;
    // 3. 显示四个电感的数值 (假设取值范围是ADC的0-4095)
    // ips114_show_uint16 不带对齐功能，如有前置零残留问题，可在外部写个格式化补空格，这里直接调用基本接口
    ips114_show_uint16(24, 32, inductance_data[0]); // L1 数值
    ips114_show_uint16(96, 32, inductance_data[1]); // L2 数值
    ips114_show_uint16(24, 48, inductance_data[2]); // L3 数值
    ips114_show_uint16(96, 48, inductance_data[3]); // L4 数值
    ips114_show_int16(88, 64, real_left);           // 左编码器数值
    ips114_show_int16(88, 80, real_right);          // 右编码数值

    //    Send_Data_To_PC(inductance_data); // 将数据通过无线串口发送到电脑端
}

// 通过无线串口发送可变数据的函数
void Send_Data_To_PC(uint16 *inductance_data)
{
    // 1. 申请一个足够大的字符数组，用于存放格式化后的字符串
    char send_buf[128];

    // 2. 使用 sprintf 将数值格式化为字符串
    // 这里以浮点数保留2位小数 (%.2f)，整数 (%d) 为例。加上 \r\n 以便电脑端串口助手换行显示。
    sprintf(send_buf, "Yaw: %.2f, GyroZ: %.2f, L1:%d, L2:%d, L3:%d, L4:%d, EL:%d, ER:%d\r\n",
            Daty_Z,
            IMU_Data.gyro_z,
            inductance_data[0],
            inductance_data[1],
            inductance_data[2],
            inductance_data[3]
            //            real_left,
            //            real_right
    );

    // 3. 调用库函数发送最终拼装好的字符串
    wireless_uart_send_string(send_buf);
}

void Parameter_Debug_Init(void)
{
    seekfree_assistant_receive_callback = wireless_uart_read_buffer;
    seekfree_assistant_init();
}

void Parameter_Debug(float *param1, float *param2, float *param3,uint8 *speed)
{
    uint8 i;
    seekfree_assistant_data_analysis();
    // 遍历
    for (i = 0; i < 3; i++)
    {
        if (seekfree_assistant_parameter_update_flag[i])
        {
            seekfree_assistant_parameter_update_flag[i] = 0;
            sprintf(send_str, "%d\r\n", real_left);
            wireless_uart_send_buffer((uint8 *)send_str, strlen(send_str));
            *param1 = seekfree_assistant_parameter[0] * 0.01f;
            *param2 = seekfree_assistant_parameter[1] * 0.01f;
            *param3 = seekfree_assistant_parameter[2] * 0.01f;
            *speed = seekfree_assistant_parameter[3]*10;
        }
    }
}

void Oscilloscope_Init(void)
{
    seekfree_assistant_transfer_callback = wireless_uart_send_buffer;
}

void Oscilloscope_Display(uint16 num1,uint16 num2)
{
	seekfree_assistant_oscilloscope_data.dat[0] = num1;
    seekfree_assistant_oscilloscope_data.dat[1] = num2;
    seekfree_assistant_oscilloscope_data.channel_num = 2; // 发送两个通道的数据
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}