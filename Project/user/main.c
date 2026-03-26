#include "zf_common_headfile.h"
uint16 pit0_count = 0;
void pit_handler(void);

void main()
{
	clock_init(SYSTEM_CLOCK_30M);
	debug_init();
	tim0_irq_handler = pit_handler;	
	interrupt_global_disable();
	pit_init(TIM_0, 1);
	LED_Init();
	interrupt_global_enable();
	
	while (1)
	{
	}
}

void pit_handler(void)
{

}