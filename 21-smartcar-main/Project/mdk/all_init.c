#include "zf_common_headfile.h"
/* ȫ�������ʼ����IMU����� PWM������������� ADC�����ߴ��� */
void ALL_init()
{
	interrupt_global_disable();
	imu660ra_init();
	gyro_init();

	pwm_init(PWMA_CH1P_P60, 25000, 0);
	pwm_init(PWMA_CH2P_P62, 25000, 0);
	
	pwm_init(PWMA_CH4N_P27, 25000, 0);
	pwm_set_duty(PWMA_CH4N_P27, 6500);
	Inductance_Init();
	PID_Init(&left_spid,  15.5,0,1.85f,0,SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&right_spid, 15.5,0,1.85f,0,SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&Turn_PID,   0.2,0.0240,0,0.60, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);
	PID_Init(&Gyro_PID,   0.008, 0, 0,2.2, SPEED_PID_MAX_OUT, SPEED_PID_MAX_I);

	encoder_init();
	gpio_init(IO_P61, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	gpio_init(IO_P63, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	wireless_uart_init();
	Oscilloscope_Init();
	
	
	gpio_init(IO_P77, GPO, GPIO_LOW, GPO_PUSH_PULL);
	P77=1;
	system_delay_ms(1);
	P77=0;
	system_delay_us(5);
	P77=1;
	system_delay_ms(500);
	interrupt_global_enable();
}