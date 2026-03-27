#include "control.h"
#include "motor.h" // 需要调用 motor_L 和 motor_R

// ===================== PID 参数实例化 =====================
// 【1. 速度环】(内环) - 使用增量式 PI 控制
// 目标: 迅速消除电机受到阻力时的转速差
PID_TypeDef Speed_PID_L = {0.8f, 0.05f, 0.0f, 0, 0, 0, 0, 0, 0};
PID_TypeDef Speed_PID_R = {0.8f, 0.05f, 0.0f, 0, 0, 0, 0, 0, 0};

// 【2. 角速度环】(中环) - 使用位置式 PD 或 PI
// 目标: 提升车体转向的响应速度和阻尼感，消除过弯时的抖动
PID_TypeDef Gyro_PID = {1.2f, 0.0f, 0.5f, 0, 0, 0, 0, 500, 0};

// 【3. 电磁循迹环】(外环) - 使用位置式 PD
// 目标: 根据电感偏差(差比和)计算出需要旋转的角速度
PID_TypeDef Track_PID = {0.5f, 0.0f, 1.5f, 0, 0, 0, 0, 0, 0};

// ===================== 全局控制变量 =====================
int16 Base_Speed = 0;   // 基础前进速度
int16 Turn_Speed = 0;   // 最终计算出的左右轮差速
float Target_Gyroz = 0; // 期望角速度
float Sensor_Error = 0; // 赛道偏差(需在电感采样的代码中实时更新)

// 初始化函数
void Control_Init(void)
{
    Base_Speed = 0; // 初始状态速度给0，防止上电飞车
}

// 通用限幅函数，防止溢出或满载
int16 Limit_Value(int16 value, int16 max_limit)
{
    if (value > max_limit)
        return max_limit;
    if (value < -max_limit)
        return -max_limit;
    return value;
}

/**
 * @brief 增量式 PID 计算 (专用于内环速度环)
 * 输出的是控制量的增量，累加后作为最终占空比
 */
float PID_Inc_Calc(PID_TypeDef *pid, float target, float current)
{
	float p_out,i_out,d_out;
    pid->error = target - current;

    // 增量式PID公式：ΔU = Kp*(E[k]-E[k-1]) + Ki*E[k] + Kd*(E[k]-2*E[k-1]+E[k-2])
    p_out = pid->kp * (pid->error - pid->last_error);
    i_out = pid->ki * pid->error;
    d_out = pid->kd * (pid->error - 2 * pid->last_error + pid->prev_error);

    pid->output += (p_out + i_out + d_out);

    // 更新历史误差
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}

/**
 * @brief 位置式 PID 计算 (适用于外环循迹和中环角速度)
 * 输出的是直接的控制量绝对值
 */
float PID_Pos_Calc(PID_TypeDef *pid, float target, float current)
{
    pid->error = target - current;

    // 积分累加及积分限幅(抗积分饱和)
    if (pid->ki != 0)
    {
        pid->integral += pid->error;
        if (pid->integral > pid->max_integral)
            pid->integral = pid->max_integral;
        if (pid->integral < -pid->max_integral)
            pid->integral = -pid->max_integral;
    }
    else
    {
        pid->integral = 0;
    }

    // 位置式PID公式：U = Kp*E[k] + Ki*ΣE + Kd*(E[k]-E[k-1])
    pid->output = pid->kp * pid->error +
                  pid->ki * pid->integral +
                  pid->kd * (pid->error - pid->last_error);

    pid->last_error = pid->error;

    return pid->output;
}

/**
 * @brief 三闭环串级控制核心函数
 * @note 此函数需要放在定时器中断中周期性调用（通常为 5ms 或 10ms）
 */
void Cascaded_Loop_Control(void)
{
    static uint8 ctrl_div_cnt = 0;

    int16 target_L = 0;
    int16 target_R = 0;
    int16 out_duty_L = 0;
    int16 out_duty_R = 0;

    ctrl_div_cnt++;

    // 假设此函数在 1ms 中断(如 encoder.c 的 pit_handler)中调用
    // 这里做 10ms 的控制周期分频
    if (ctrl_div_cnt >= 10)
    {
        ctrl_div_cnt = 0;

        // =========================================================
        // 【第一环】外环 (电磁位置环)
        // 输入：期望赛道偏差设为 0，实际反馈为 Sensor_Error
        // 输出：期望的车体旋转角速度 Target_Gyroz
        // =========================================================
        Target_Gyroz = PID_Pos_Calc(&Track_PID, 0.0f, Sensor_Error);

        // =========================================================
        // 【第二环】中环 (陀螺仪角速度环)
        // 输入：期望角速度 Target_Gyroz，实际反馈为 IMU 的 gyroz
        // 输出：左右电机的差速补偿 Turn_Speed
        // =========================================================
        Turn_Speed = (int16)PID_Pos_Calc(&Gyro_PID, Target_Gyroz, gyroz);

        // 差速限幅，防止转向过猛导致单侧轮反转过快或打滑
        Turn_Speed = Limit_Value(Turn_Speed, 100);

        // =========================================================
        // 运动学融合：将基础速度与差速融合，生成左右轮的目标速度
        // (注：加减号取决于您的电机接线方向，可能需要互换)
        // =========================================================
        target_L = Base_Speed + Turn_Speed;
        target_R = Base_Speed - Turn_Speed;

        // =========================================================
        // 【第三环】内环 (电机速度环)
        // 输入：期望轮速 target_L/R，实际反馈为编码器 real_left/right
        // 输出：电机的 PWM 占空比 out_duty_L/R
        // =========================================================
        out_duty_L = (int16)PID_Inc_Calc(&Speed_PID_L, (float)target_L, (float)real_left);
        out_duty_R = (int16)PID_Inc_Calc(&Speed_PID_R, (float)target_R, (float)real_right);

        // PWM占空比绝对限幅 (-100 ~ 100，根据 motor.c 的 PWM_DUTY_MAX 设定)
        out_duty_L = Limit_Value(out_duty_L, 100);
        out_duty_R = Limit_Value(out_duty_R, 100);

        // =========================================================
        // 最终执行：下发PWM到电机
        // =========================================================
        motor_L(out_duty_L);
        motor_R(out_duty_R);
    }
}