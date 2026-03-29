#include "zf_common_headfile.h"
uint16  ABS(int16 P)
{
	if(P>0)
		return P;
	else
		return -P;
}

void Motor_SetSpeed(int16 speed_left, int16 speed_right)
{    
    // 左轮方向控制
    if(speed_left > 0) {
       gpio_set_level(IO_P61, 0);    // 正转
    } else if(speed_left < 0) {
       gpio_set_level(IO_P61, 1);    // 正转
    } 
    // 右轮方向控制 - 注意反转了方向
    if(speed_right < 0) {
		gpio_set_level(IO_P63, 0);    // 正转

    } else if(speed_right > 0) {
		gpio_set_level(IO_P63,1);    // 正转
    }
    // 设置PWM值
	pwm_set_duty(PWMA_CH1P_P60,ABS(speed_left));    
	pwm_set_duty(PWMA_CH2P_P62,ABS(speed_right));    
}
