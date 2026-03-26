#include "zf_common_headfile.h"
uint16 pit0_count = 0;
uint16 ind_data[4]; // 申请一个全局数组存放4个电感数值
void pit_handler(void);
void main()
{
	Car_Init();
	tim0_irq_handler = pit_handler;
	while (1)
	{
		Inductance_Read(ind_data);
		// 将获取的数据实时更新至屏幕
		Menu_Display(ind_data);

		// 可选：适当延时控制屏幕刷新率，避免主循环跑太快导致屏幕闪烁
		system_delay_ms(50);
	}
}

void pit_handler(void)
{
	imu660ra_get_acc(); // 获取 IMU660RA 的加速度测量数值
	imu660ra_get_gyro();
	pit0_count++;
	if (pit0_count >= 5)
	{
		pit0_count = 0;
		gyro_proc();
	}
}