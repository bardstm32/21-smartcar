#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "zf_common_headfile.h" // 包含逐飞科技或您的系统底层头文件

// ======================= 定义 PID 结构体 =======================
typedef struct
{
    float kp;
    float ki;
    float kd;

    float error;      // 当前次误差
    float last_error; // 上一次误差
    float prev_error; // 上上一次误差

    float integral;     // 误差积分 (用于位置式PID)
    float max_integral; // 积分限幅

    float output; // 输出值
} PID_TypeDef;

// ======================= 外部控制变量声明 =======================
// PID 参数结构体
extern PID_TypeDef Speed_PID_L; // 左轮速度环
extern PID_TypeDef Speed_PID_R; // 右轮速度环
extern PID_TypeDef Gyro_PID;    // 角速度环 (中环)
extern PID_TypeDef Track_PID;   // 电磁循迹环 (外环)

// 运动控制变量
extern int16 Base_Speed;   // 基础直道速度 (脉冲数/控制周期)
extern int16 Turn_Speed;   // 转向差速
extern float Target_Gyroz; // 目标角速度
extern float Sensor_Error; // 赛道电感偏差 (由 inductance.c 计算后赋值)

// 外部引用的真实反馈数据（这些在您之前的代码中已经存在）
extern int16 real_left;  // 来自 encoder.c
extern int16 real_right; // 来自 encoder.c
extern float gyroz;      // 来自 imu.c (Z轴角速度)

// ======================= 控制函数原型声明 =======================
void Control_Init(void);
void Cascaded_Loop_Control(void);

// 通用PID算法与限幅
float PID_Inc_Calc(PID_TypeDef *pid, float target, float current);
float PID_Pos_Calc(PID_TypeDef *pid, float target, float current);
int16 Limit_Value(int16 value, int16 max_limit);

#endif