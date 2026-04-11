#include "zf_common_headfile.h"
// 定义3组PID(位置环+左右轮速度环)
PID_TypeDef Turn_PID;
PID_TypeDef left_spid;
PID_TypeDef right_spid;

#define ENCODER_DIR_LEFT (TIM0_ENCOEDER)           // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_LEFT (IO_P35)              // DIR 对应的引脚
#define ENCODER_DIR_PULSE_LEFT (TIM0_ENCOEDER_P34) // PULSE 对应的引脚

#define ENCODER_DIR_RIGHT (TIM3_ENCOEDER)           // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_RIGHT (IO_P53)              // DIR 对应的引脚
#define ENCODER_DIR_PULSE_RIGHT (TIM3_ENCOEDER_P04) // PULSE 对应的引脚

#define MAX_INC 100 // 2ms 周期下限制单次增量
volatile int32 speed_left = 0;
volatile int32 speed_right = 0;
volatile uint16 adc_inductance[5] = {0};
volatile int32 real_left = 0;
volatile int32 real_right = 0;

double elemid = 0;   // 目标赛道偏差
double eleOut_0 = 0; // 赛道偏差环输出值
double eleOut_1 = 0; // 偏航角速度环输出值
int Gyro_Z = 0;      // 实际偏航角速度
double eleValue = 0;

float left_spid_out = 0;

double Left_High_Speed = 0, Right_High_Speed = 0, High_Speed = 0; // 左右轮目标速度、基础速度

/**
 * @brief  PID初始化(通用)
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, int16 max_out, int16 max_i)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_i = max_i;

    pid->target = 0;
    pid->measure = 0;
    pid->err = 0;
    pid->last_err = 0;
    pid->prev_err = 0; // ——> 【新增】初始化上上次误差
    pid->P = 0;
    pid->I = 0;
    pid->D = 0;
    pid->out = 0;
}

/**
 * @brief  通用位置式PID计算
 */
float PID_Calc(PID_TypeDef *pid, float target, float measure)
{
    pid->target = target;
    pid->measure = measure;

    // 计算误差
    pid->err = pid->target - pid->measure;

    // 比例项
    pid->P = pid->Kp * pid->err;
    // 微分项
    pid->D = pid->Kd * (pid->err - pid->last_err);
    // 总输出+限幅
    pid->out = pid->P + pid->D;
    if (pid->out > pid->max_out)
        pid->out = pid->max_out;
    if (pid->out < -pid->max_out)
        pid->out = -pid->max_out;

    // 更新误差
    pid->last_err = pid->err;

    return pid->out;
}

/**
 * @brief 方向环控制函数
 */
void Dir_Control()
{
    static int t = 0;
    if (++t >= 2)
    {
        // 外环（赛道偏差环）,具体正负号根据实际情况确定
        eleOut_0 = PID_Calc(&Turn_PID, eleValue, elemid);
        // 限幅保护，确保输出结果在 -100 ~ 100 范围内
        eleOut_0 = range_protect(eleOut_0, -100.0, 100.0);
    }
    // 内环（偏航角速度环）,具体正负号根据实际情况确定
    eleOut_1 = PID_Calc(&Gyro_PID, (double)Gyro_Z, eleOut_0);
    // 限幅保护，确保输出结果在 -100 ~ 100 范围内
    eleOut_1 = range_protect(eleOut_1, -100.0, 100.0);
}

void Calculate_Differential_Drive() // 差速计算
{
    float k = 0;                       // 差速比例系数
    k = eleOut_0 * 0.01;               // 将 -100 ~ 100 缩放成 -1 ~ 1
    k = range_protect(k, -0.65, 0.65); // 限制到 -0.65 ~ 0.65，实现差速限幅
    // 计算左右轮目标速度
    if (k >= 0) // 左转
    {
        left_spid.target = BASE_SPEED * (1 - k);
        right_spid.target = BASE_SPEED * (1 + k * 0.2); // 加少减多
    }
    else // 右转
    {
        k *= -1;                                       // 取相反数
        left_spid.target = BASE_SPEED * (1 + k * 0.2); // 加少减多
        right_spid.target = BASE_SPEED * (1 - k);
    }
}

// 增量式PID计算函数
// 公式: ΔU = Kp*(e(k) - e(k-1)) + Ki*e(k)
void IncPID_Calc(PID_TypeDef *pid, int16 current_speed)
{
    float speed_f;

    speed_f = 0.25f * current_speed + 0.75f * pid->last_f_speed;
    pid->last_f_speed = speed_f;
    // 1. 计算当前误差
    pid->err = pid->target - speed_f;
    pid->I = pid->Ki * pid->err;
    // 2. 计算输出增量 (ΔU)
    pid->inc_out = (pid->Kp * (pid->err - pid->last_err) + pid->I);
    if (pid->inc_out > MAX_INC)
        pid->inc_out = MAX_INC;
    if (pid->inc_out < -MAX_INC)
        pid->inc_out = -MAX_INC;
    // 3. 累加增量得到最终实际输出
    pid->out += pid->inc_out;
    // 4. 输出限幅 (防止PWM跑飞)
    if (pid->out > pid->max_out)
        pid->out = pid->max_out;
    if (pid->out < -pid->max_out)
        pid->out = -pid->max_out;
    // 5. 保存历史误差，供下一次计算使用
    pid->prev_err = pid->last_err;
    pid->last_err = pid->err;
}

/**
 * @brief  双闭环循迹控制(核心函数)
 * 流程：电感差比和 → 位置环 → 左右轮目标速度 → 速度环 → 电机PWM
 */
void Dual_Loop_Control(void)
{
    //    int16 inductance_err;  // 电感差比和误差
    //    float pos_out;         // 位置环输出(速度差)

    float left_speed;  // 左轮实际速度(编码器读取)
    float right_speed; // 右轮实际速度
                       //
                       //    int32 left_pwm;        // 左轮最终PWM
                       //    int32 right_pwm;       // 右轮最终PWM

    //===================== 1. 读取电感数据+计算误差 =====================//
    //    Inductance_Read(adc_inductance);
    // 调用原有差比和函数(输入L1/L2/L3/L4,输出-100~100)
    //    inductance_err = Inductance_Count_Err(adc_inductance[1], adc_inductance[2], adc_inductance[3], adc_inductance[4]);

    //    //===================== 2. 位置环PID(循迹外环) =====================//
    //    pos_out = PID_Calc(&pos_pid, 0, inductance_err);
    //
    //    //===================== 3. 计算左右轮目标速度 =====================//
    //    // 位置环输出=速度差：左轮减速、右轮加速(向右偏); 左轮加速、右轮减速(向左偏)
    //    left_target  = BASE_SPEED - pos_out;
    //    right_target = BASE_SPEED + pos_out;

    //===================== 4. 读取编码器实际速度 =====================//
    speed_left = encoder_get_count(TIM0_ENCOEDER); // 获取编码器计数
    speed_right = encoder_get_count(TIM3_ENCOEDER);

    encoder_clear_count(TIM0_ENCOEDER); // 清空编码器计数
    encoder_clear_count(TIM3_ENCOEDER);

    real_left = real_left * 0.8 + speed_left * 0.2;
    real_right = real_right * 0.8 + speed_right * 0.2;

    //    //===================== 5. 速度环PID(稳速内环) =====================//
    IncPID_Calc(&left_spid, real_left);
    IncPID_Calc(&right_spid, real_right);

    Oscilloscope_Display(real_left, real_right, left_spid.target, left_spid.inc_out, left_spid.out, right_spid.out);
    //    //===================== 6. 输出到电机 =====================//
    Motor_SetSpeed(-2000, -2000);
}
