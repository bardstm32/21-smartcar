#include "zf_common_headfile.h"
uint16 pit0_count = 0;
uint16 ind_data[4]; // 申请一个全局数组存放4个电感数值
void main()
{
	Car_Init();
	while (1)
	{
		Inductance_Read(ind_data);
		// 将获取的数据实时更新至屏幕
		Menu_Display(ind_data);

		// 可选：适当延时控制屏幕刷新率，避免主循环跑太快导致屏幕闪烁
		system_delay_ms(50);
	}
}
