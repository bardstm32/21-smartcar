#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "zf_common_headfile.h"
#include "inductance.h"
#include "motor.h"

// --- 赛道元素阈值定义 (需根据12位ADC实际读取的峰值进行修改) ---
#define CROSS_H_THRESHOLD 3000 // 十字路口时，左右横向电感的阈值
#define RING_H_THRESHOLD 3200  // 环岛入环时，单侧横向电感的峰值阈值
#define LOST_LINE_SUM 200      // 丢线保护阈值 (电感和低于此值认为脱轨)

// --- 速度配置 ---
#define BASE_SPEED 40 // 直道基础速度 (对应 motor 控制的 duty 范围 -100 ~ 100)
#define MAX_SPEED 90  // 最大限速
#define MIN_SPEED -30 // 最小速度 (允许轻微反转以辅助急转弯)

// --- 赛道状态枚举 ---
typedef enum
{
    TRACK_NORMAL = 0, // 正常直道/弯道
    TRACK_CROSS,      // 识别到十字路口
    TRACK_RING_L_IN,  // 识别到左环岛入环
    TRACK_RING_R_IN,  // 识别到右环岛入环
    TRACK_LOST        // 丢线状态
} Track_State_e;

// --- PID 结构体定义 ---
typedef struct
{
    float kp;
    float kd;
    float error;
    float last_error;
    float output;
} Track_PID_t;

// --- 外部变量声明 ---
extern Track_State_e current_track_state;
extern Track_PID_t track_pid;
extern uint16 element_lock_timer;

// --- 核心控制函数声明 ---
void Track_Control_Init(void);
void Track_Element_Recognize(uint16 *ind_data);
float Track_Get_Optimized_Error(uint16 *ind_data);
void Track_Execute_Steering(void);

#endif // __CONTROL_H__