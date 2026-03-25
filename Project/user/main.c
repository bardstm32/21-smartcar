#include "zf_common_headfile.h"
void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	imu_init();
	while (1)
	{
	}
}
