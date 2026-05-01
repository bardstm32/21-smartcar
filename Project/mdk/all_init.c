#include "zf_common_headfile.h"
void ALL_init()
{
	interrupt_global_disable();
	imu660ra_init();
	gyro_init();
	pwm_init(PWMA_CH1P_P60, 25000, 0);
	pwm_init(PWMA_CH2P_P62, 25000, 0);
//	pwm_init(PWMA_CH4N_P27, 25000, 0);
//	pwm_set_duty(PWMA_CH4N_P27, 5000);
	gpio_init(IO_P61, GPO, GPIO_LOW, GPO_PUSH_PULL);
	gpio_init(IO_P63, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	ips114_init();
	wireless_uart_init();
	encoder_init();
	LED_Init();
	Oscilloscope_Init();
	Parameter_Debug_Init();
	Inductance_Init();
	interrupt_global_enable();
}