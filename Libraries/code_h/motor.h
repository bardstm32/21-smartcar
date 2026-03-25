#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

#define DIR_L IO_P61
#define DIR_R IO_P63
#define PWM_L (PWMA_CH1P_P60)
#define PWM_R (PWMA_CH2P_P62)
//负压引脚为3.3，控制打开或者是关闭
void motor_init(void);
void motor_L(int16 duty);
void motor_R(int16 duty);

#endif