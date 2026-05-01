#ifndef __TRANK_H
#define __TRANK_H

//===================== 全局宏定义 =====================//
#define PWM_MAX             5000.0f   // 电机PWM最大限幅
#define BASE_SPEED          180.0f    // 基础直行速度(可调)

// 位置环(循迹外环)参数限幅
#define POS_PID_MAX_OUT     600     // 位置环最大输出(左右轮速度差)
#define POS_PID_MAX_I       200     // 位置环积分限幅
// 速度环(稳速内环)参数限幅
#define SPEED_PID_MAX_OUT    PWM_MAX // 速度环最大输出(PWM)
#define SPEED_PID_MAX_I      1900    // 速度环积分限幅
#define MOTOR_DEAD_ZONE      10
//===================== PID结构体 =====================//
// 通用PID结构体(位置/速度环共用)
typedef struct
{
    float Kp; // 比例系数
    float Ki; // 积分系数
    float Kd; // 微分系数
	float Kp2;
	
    int32 target;        // 目标值
    float measure;       // 测量值
    float err;           // 当前误差
    float last_err;      // 上一次误差 e(k-1)
    float prev_err; // 上上次误差 e(k-2) ——> 【新增】用于增量式PID
	float last_f_speed;
	
    float P;   // 比例项
	float P2;
    float I;   // 积分项
    float D;   // 微分项
    float out; // 总输出

    int16 max_out; // 输出限幅
    int16 max_i; // 积分限幅
	float inc_out;	
} PID_TypeDef;
//===================== 全局变量 =====================//
extern PID_TypeDef Turn_PID, Gyro_PID;   // 位置环PID(循迹)
extern PID_TypeDef left_spid; // 左轮速度环PID
extern PID_TypeDef right_spid;// 右轮速度环PID

extern volatile int32 speed_left;
extern volatile int32 speed_right;
extern volatile float Turn_target;
extern float elemid;    // 赛道偏差
extern float eleOut_0;  // 赛道偏差环输出值





//===================== 函数声明 =====================//
void PID_Init(PID_TypeDef *pid, float kp, float kp2, float ki, float kd, float max_out, float max_i);
float PID_Calc(PID_TypeDef *pid, float target, float measure);
void Dual_Loop_Control(void);  // 双闭环核心控制函数
void IncPID_Calc(PID_TypeDef *pid, int16 current_speed);
void Calculate_Differential_Drive() ;// 差速计算

void Dir_Control();
#endif 