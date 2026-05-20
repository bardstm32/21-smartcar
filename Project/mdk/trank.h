#ifndef __TRANK_H
#define __TRANK_H

// ===================== 全局宏定义 =====================
#define BASE_SPEED        275.0f    // 基础直行速度（编码器计数/控制周期）

// 速度内环输出限幅
#define SPEED_PID_MAX_OUT     8000.0f   // 速度环输出上限（PWM）
#define SPEED_PID_MAX_I      1900     // 速度环积分项限幅

// 整数绝对值宏，避免传 int 给 float 版 My_abs 引入隐式 float 转换
#define IABS(x) ((x) >= 0 ? (x) : -(x))

// ===================== PID 结构体 =====================
// 通用 PID 结构体，用于位置式与增量式两种 PID
typedef struct
{
    float Kp;             // 比例系数
    float Ki;             // 积分系数
    float Kd;             // 微分系数
    float Kp2;            // 二次项系数（用于 |err|*err 的非线性增益）

    float target;         // 目标值
    float measure;        // 测量值
    float err;            // 当前误差 e(k)
    float last_err;       // 上一次误差 e(k-1)
    float prev_err;       // 上上次误差 e(k-2)，预留给增量式 PID
    float P;              // 比例项
    float P2;             // 二次比例项
    float I;              // 积分项
    float D;              // 微分项
    float out;            // 累计输出

    float max_out;        // 输出限幅
    float max_i;          // 积分限幅
    float inc_out;        // 增量式输出（单步 ΔU）
} PID_TypeDef;

// ===================== 全局变量 =====================
extern PID_TypeDef Turn_PID, Gyro_PID;   // 方向位置环 + 陀螺角速度环
extern PID_TypeDef left_spid;            // 左轮速度环
extern PID_TypeDef right_spid;           // 右轮速度环

extern volatile int16 speed_left;
extern volatile int16 speed_right;
extern volatile float Turn_target;
extern float elemid;     // 电感综合偏差（输入到方向环）
extern float eleOut_0;   // 方向位置环输出
extern float eleOut_1;   // 方向角速度环输出

// ===================== 函数声明 =====================
void PID_Init(PID_TypeDef *pid, float kp, float kp2, float ki, float kd, float max_out, float max_i);
float PID_Calc(PID_TypeDef *pid, float target, float measure);
void Dual_Loop_Control(void);                       // 速度双轮内环（2ms）
void IncPID_Calc(PID_TypeDef *pid, int16 current_speed);
void Calculate_Differential_Drive(void);            // 根据方向输出计算左右轮目标速度
void Dir_Control_gyro(void);                        // 方向角速度环
void Dir_Control(void);                             // 方向位置环

#endif
