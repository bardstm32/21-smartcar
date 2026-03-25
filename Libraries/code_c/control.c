#include "control.h"

Track_State_e current_track_state = TRACK_NORMAL;
Track_PID_t track_pid;

uint16 element_lock_timer = 0; // 元素状态机锁定时间，防止重复误判

/**
 * @brief 循迹模块初始化
 * 配置PID默认参数
 */
void Track_Control_Init(void)
{
    track_pid.kp = 1.2f; // 比例系数 (需自行整定)
    track_pid.kd = 2.5f; // 微分系数 (需自行整定)
    track_pid.error = 0.0f;
    track_pid.last_error = 0.0f;
    track_pid.output = 0.0f;
}

/**
 * @brief 赛道元素识别状态机
 * 根据卓晴老师的电磁分析：
 * 十字：两边横向电感同时感应到垂直电磁线，产生双峰。
 * 环岛：单侧横向电感产生巨大突变峰值。
 * @param ind_data 经过滤波后的四电感数组 (1:L横, 2:LM竖, 3:RM竖, 4:R横)
 */
void Track_Element_Recognize(uint16 *ind_data)
{
    // 提取电感值，方便阅读
    uint16 L_H = ind_data[1]; // 左横
    uint16 L_V = ind_data[2]; // 左竖
    uint16 R_V = ind_data[3]; // 右竖
    uint16 R_H = ind_data[4]; // 右横

    // 丢线检测
    if ((L_V + R_V + L_H + R_H) < LOST_LINE_SUM)
    {
        current_track_state = TRACK_LOST;
        return;
    }

    // 延时锁定期，不进行新的识别
    if (element_lock_timer > 0)
    {
        element_lock_timer--;
        return;
    }

    current_track_state = TRACK_NORMAL; // 默认为正常赛道

    // 1. 识别十字路口：左右横电感同时急剧增大
    if (L_H > CROSS_H_THRESHOLD && R_H > CROSS_H_THRESHOLD)
    {
        current_track_state = TRACK_CROSS;
        element_lock_timer = 50; // 锁定状态一定周期 (根据定时器执行频率调整)
    }
    // 2. 识别左环岛入环：左横电感极大，右横正常
    else if (L_H > RING_H_THRESHOLD && R_H < CROSS_H_THRESHOLD)
    {
        current_track_state = TRACK_RING_L_IN;
        element_lock_timer = 40;
    }
    // 3. 识别右环岛入环：右横电感极大，左横正常
    else if (R_H > RING_H_THRESHOLD && L_H < CROSS_H_THRESHOLD)
    {
        current_track_state = TRACK_RING_R_IN;
        element_lock_timer = 40;
    }
}

/**
 * @brief 优化后的差比和误差计算
 * @param ind_data 经过滤波后的四电感数组
 * @return float 计算出的最终偏差值
 */
float Track_Get_Optimized_Error(uint16 *ind_data)
{
    float scaled_err = 0.0f;
	float denominator,numerator;
    uint16 L_H = ind_data[1];
    uint16 L_V = ind_data[2];
    uint16 R_V = ind_data[3];
    uint16 R_H = ind_data[4];

    // 分母防零除保护
    denominator = Str_value * (float)(L_H + R_H) + Cur_value * (float)(L_V + R_V);
    if (denominator < 1.0f)
    {
        return track_pid.last_error; // 如果分母过小，维持上次偏差
    }

    // 基础差比和计算 (引入了你原代码中的 Str_value 和 Cur_value 权重)
    numerator = Str_value * (float)((int32)L_H - R_H) + Cur_value * (float)((int32)L_V - R_V);
    scaled_err = (numerator / denominator) * 100.0f;

    // 限幅保护
    scaled_err = range_protect(scaled_err, -100.0, 100.0);

    // --- 根据赛道元素强行修正偏差 ---
    switch (current_track_state)
    {
    case TRACK_CROSS:
        // 十字路口中心通常会丢线或严重畸变，直接锁死偏差为0，让车直冲过去
        scaled_err = 0.0f;
        break;
    case TRACK_RING_L_IN:
        // 检测到左环岛，强行赋予一个极大的负偏差，让车猛烈向左打角进环
        scaled_err = -80.0f;
        break;
    case TRACK_RING_R_IN:
        // 检测到右环岛，强行向右切入
        scaled_err = 80.0f;
        break;
    default:
        break;
    }

    return scaled_err;
}

/**
 * @brief 核心控制执行函数 (建议在定时器中断中调用，如 5ms 一次)
 * 包含：采数据 -> 判元素 -> 算PID -> 差速输出
 */
void Track_Execute_Steering(void)
{
    uint16 ind_filtered[5] = {0};
    int16 speed_L = 0, speed_R = 0;

    // 1. 读取并滤波电感数据 (调用你 inductance.c 中的函数)
    Inductance_Read(ind_filtered);

    // 2. 赛道元素识别更新
    Track_Element_Recognize(ind_filtered);

    // 3. 丢线停车保护
    if (current_track_state == TRACK_LOST)
    {
        motor_L(0);
        motor_R(0);
        return;
    }

    // 4. 获取计算出的偏差
    track_pid.error = Track_Get_Optimized_Error(ind_filtered);

    // 5. PD 位置式计算
    track_pid.output = track_pid.kp * track_pid.error +
                       track_pid.kd * (track_pid.error - track_pid.last_error);
    track_pid.last_error = track_pid.error;

    // 6. 差速计算 (基础速度 ± 转向PWM)
    // 假设 error 为正时车体偏右，需要向左拐 -> 左轮减速，右轮加速
    speed_L = BASE_SPEED - (int16)track_pid.output;
    speed_R = BASE_SPEED + (int16)track_pid.output;

    // 7. 速度限幅 (匹配你 motor.c 中 -100 到 100 的范围)
    speed_L = (int16)range_protect(speed_L, MIN_SPEED, MAX_SPEED);
    speed_R = (int16)range_protect(speed_R, MIN_SPEED, MAX_SPEED);

    // 8. 驱动底层电机
    motor_L(speed_L);
    motor_R(speed_R);
}