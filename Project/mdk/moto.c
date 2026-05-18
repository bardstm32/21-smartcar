#include "zf_common_headfile.h"

/* 取绝对值 */
uint16 ABS(int16 P)
{
    if (P > 0)
        return P;
    else
        return -P;
}

/* 由速度符号决定方向引脚电平，由幅值决定 PWM 占空比 */
void Motor_SetSpeed(int16 speed_left, int16 speed_right)
{
    if (speed_left < 0)
    {
        gpio_set_level(IO_P63, 0);
    }
    else if (speed_left > 0)
    {
        gpio_set_level(IO_P63, 1);
    }

    if (speed_right < 0)
    {
        gpio_set_level(IO_P61, 0);
    }
    else if (speed_right > 0)
    {
        gpio_set_level(IO_P61, 1);
    }

    pwm_set_duty(PWMA_CH2P_P62, ABS(speed_left));
    pwm_set_duty(PWMA_CH1P_P60, ABS(speed_right));
}

/* 脱线保护：四路电感归一化值之和过低则停车（当前未挂入主流程） */
void Motor_Protect(uint16 *inductance_norm_data)
{
    uint16 sum = inductance_norm_data[1] + inductance_norm_data[2] + inductance_norm_data[3] + inductance_norm_data[4];
    if (sum < 10)
    {
        Motor_SetSpeed(0, 0);
        interrupt_global_disable();
    }
}
