#include "zf_common_headfile.h"
void ALL_init()
{
	imu660ra_init();
	gyro_init();
	pwm_init(PWMA_CH1P_P60, 25000,0);
	pwm_init(PWMA_CH2P_P62, 25000,0);
	
//	gpio_init(IO_P61, GPO,0, GPO_PUSH_PULL);
//	gpio_init(IO_P63, GPO,0, GPO_PUSH_PULL);
//	gpio_init(IO_P67, GPO,0, GPO_PUSH_PULL);	
//	gpio_init(IO_P33, GPO,0, GPO_PUSH_PULL);
//	Inductance_Init();
//	wireless_uart_init();
//	seekfree_assistant_init();
	encoder_init();
	LED_Init();
	Buzzer_Init();
//	Menu_Init();
}