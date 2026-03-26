#include "menu.h"
#include "zf_device_ips114.h"
#include "imu.h"
#include "inductance.h"

// 菜单初始化函数
void Menu_Init(void)
{
    ips114_init();                        // 初始化IPS114屏幕
    ips114_clear(IPS114_DEFAULT_BGCOLOR); // 清除屏幕为默认背景色

    // 提前显示静态的字符串标签，避免在主循环中重复刷新导致屏幕闪烁
    // 默认字体下每个字符宽8像素，高16像素
    ips114_show_string(0, 0, "Gyro Z :");  // 偏航角速度标签
    ips114_show_string(0, 16, "Yaw Ang:"); // 偏航角标签
    ips114_show_string(0, 32, "L1:");      // 电感L1标签
    ips114_show_string(72, 32, "L2:");     // 电感L2标签
    ips114_show_string(0, 48, "L3:");      // 电感L3标签
    ips114_show_string(72, 48, "L4:");     // 电感L4标签
}

// 菜单数据显示更新函数
void Menu_Display(uint16 *inductance_data)
{
    // 1. 显示当前偏航角速度 (来自 IMU_Data.gyro_z)
    // 使用 ips114_show_float，x=64(刚好接在标签后面), y=0，整数位保留3位，小数位保留2位
    ips114_show_float(64, 0, IMU_Data.gyro_z, 3, 2);

    // 2. 显示当前偏航角度 (来自 Daty_Z)
    // 偏航角范围大致在 -180 到 180 之间，整数位预留4位(包含负号)，小数位2位
    ips114_show_float(64, 16, Daty_Z, 4, 2);

    // 3. 显示四个电感的数值 (假设取值范围是ADC的0-4095)
    // ips114_show_uint16 不带对齐功能，如有前置零残留问题，可在外部写个格式化补空格，这里直接调用基本接口
    ips114_show_uint16(24, 32, inductance_data[0]); // L1 数值
    ips114_show_uint16(96, 32, inductance_data[1]); // L2 数值
    ips114_show_uint16(24, 48, inductance_data[2]); // L3 数值
    ips114_show_uint16(96, 48, inductance_data[3]); // L4 数值
}