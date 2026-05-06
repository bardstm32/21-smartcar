#ifndef __MOTO_H__
#define __MOTO_H__
void Motor_SetSpeed(int16 speed_left, int16 speed_right);
void Motor_Protect(uint16 *inductance_norm_data);

#endif