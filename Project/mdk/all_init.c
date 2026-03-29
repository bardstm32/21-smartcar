#include "zf_common_headfile.h"
void ALL_init()
{
	imu660ra_init();
	gyro_init();
	pwm_init(PWMA_CH1P_P60, 25000,0);
	pwm_init(PWMA_CH2P_P62, 25000,0);
	ips114_init();
	wireless_uart_init();
	encoder_init();
	LED_Init();
	//Menu_Init();
}