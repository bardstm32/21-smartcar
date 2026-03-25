/*********************************************************************************************************************
 *  @file  motor_control.c
 *  @brief  电机初始化与控制
 * 该文件包含了电机初始化、转速和方向控制的相关函数。
 * 电机通过方向控制GPIO和PWM信号来实现正转、反转和停止。
 * 初始化过程中，配置GPIO为输出模式，默认上拉高电平，并设置PWM频率为25kHz以最小化对电感传感器的影响。
 * 通过调整PWM占空比来控制电机的转速，并通过GPIO设置电机的方向。
 *  @date              @author            备注
 *  2026-03-13        三帅勇闯智能车-rw   初始版本
 *  修改记录：
 *  @date              @author            备注
 * 
 ********************************************************************************************************************/

#include "motor.h"

/**
  * @brief 初始化电机控制所需的GPIO和PWM
  * 该函数用于初始化电机控制中使用的两个GPIO引脚和两个PWM通道。
  * 对于每个电机，初始化其方向控制GPIO为输出模式，默认上拉高电平。
  * 同时，初始化电机的PWM通道，设置频率为25kHz，初始占空比为0。
  * 频率设置成25KHz对电感的影响最小
*/
void motor_init(void)
{
	// 初始化左电机方向控制GPIO为输出模式，默认上拉高电平
	gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	// 初始化左电机PWM通道，频率设置为25kHz，占空比初始为0
	pwm_init(PWM_L, 25000, 0);
	// 初始化右电机方向控制GPIO为输出模式，默认上拉高电平
	gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	// 初始化右电机PWM通道，频率设置为25kHz，占空比初始为0
	pwm_init(PWM_R, 25000, 0);						 
}

/**
 * @brief 控制左电机的转速和方向
 * 通过改变左电机的PWM占空比来控制其转速，根据传入的duty值来决定电机的方向。
 * @param duty 电机转速和方向的控制参数，范围为-100到100。
 *             正值表示正转，绝对值越大转速越快；负值表示反转，绝对值越大转速越快；
 *             0表示停止。
 */
void motor_L(int16 duty)
{
	if (duty > 0)
	{
		// 设置左电机方向控制GPIO为高电平，表示正转方向
		gpio_set_level(DIR_L, GPIO_HIGH);
		// 根据duty值计算左电机的PWM占空比并设置，控制电机转速
		pwm_set_duty(PWM_L, duty * (PWM_DUTY_MAX / 100)); 
	}
	else
	{
		// 设置左电机方向控制GPIO为低电平，表示反转方向
		gpio_set_level(DIR_L, GPIO_LOW);
		// 根据duty值计算左电机的PWM占空比并设置，控制电机转速
		pwm_set_duty(PWM_L, (-duty) * (PWM_DUTY_MAX / 100)); 
	}
}

/**
 * @brief 控制右电机的转速和方向
 * 通过改变右电机的PWM占空比来控制其转速，根据传入的duty值来决定电机的方向。
 * @param duty 电机转速和方向的控制参数，范围为-100到100。
 *             正值表示正转，绝对值越大转速越快；负值表示反转，绝对值越大转速越快；
 *             0表示停止。
 */
void motor_R(int16 duty)
{
	if (duty > 0)
	{
		// 设置右电机方向控制GPIO为高电平，表示正转方向
		gpio_set_level(DIR_R, GPIO_HIGH);
		// 根据duty值计算右电机的PWM占空比并设置，控制电机转速
		pwm_set_duty(PWM_R, duty * (PWM_DUTY_MAX / 100)); 
	}
	else
	{
		// 设置右电机方向控制GPIO为低电平，表示反转方向
		gpio_set_level(DIR_R, GPIO_LOW);
		// 根据duty值计算右电机的PWM占空比并设置，控制电机转速
		pwm_set_duty(PWM_R, (-duty) * (PWM_DUTY_MAX / 100)); 
	}
}
