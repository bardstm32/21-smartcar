#include "zf_common_headfile.h"
// 定义3组PID(位置环+左右轮速度环)
PID_TypeDef pos_pid;
PID_TypeDef left_spid;
PID_TypeDef right_spid;

#define ENCODER_DIR_LEFT                 	(TIM0_ENCOEDER)                         // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_LEFT              	(IO_P35)            				 	// DIR 对应的引脚
#define ENCODER_DIR_PULSE_LEFT            	(TIM0_ENCOEDER_P34)            			// PULSE 对应的引脚

#define ENCODER_DIR_RIGHT                 	(TIM3_ENCOEDER)                         // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_RIGHT              	(IO_P53)            				 	// DIR 对应的引脚
#define ENCODER_DIR_PULSE_RIGHT            	(TIM3_ENCOEDER_P04)            			// PULSE 对应的引脚

volatile int16 speed_left = 0;
volatile int16 speed_right = 0;

volatile int16 real_left=0;
volatile int16 real_right=0;


/**
 * @brief  PID初始化(通用)
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_i)
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
    
    // 积分项+限幅
    pid->I += pid->Ki * pid->err;
    if(pid->I >  pid->max_i)  pid->I =  pid->max_i;
    if(pid->I < -pid->max_i)  pid->I = -pid->max_i;
    
    // 微分项
    pid->D = pid->Kd * (pid->err - pid->last_err);
    
    // 总输出+限幅
    pid->out = pid->P + pid->I + pid->D;
    if(pid->out >  pid->max_out)  pid->out =  pid->max_out;
    if(pid->out < -pid->max_out)  pid->out = -pid->max_out;
    
    // 更新误差
    pid->last_err = pid->err;
    
    return pid->out;
}
/**
 * @brief  双闭环循迹控制(核心函数)
 * 流程：电感差比和 → 位置环 → 左右轮目标速度 → 速度环 → 电机PWM
 */
void Dual_Loop_Control(void)
{
    int16 inductance_err;  // 电感差比和误差
    float pos_out;         // 位置环输出(速度差)
    
    float left_target;     // 左轮目标速度
    float right_target;    // 右轮目标速度
    float left_speed;      // 左轮实际速度(编码器读取)
    float right_speed;     // 右轮实际速度
    
    int16 left_pwm;        // 左轮最终PWM
    int16 right_pwm;       // 右轮最终PWM
    
    //===================== 1. 读取电感数据+计算误差 =====================//
    uint16 adc[5];
    Inductance_Read(adc);
    // 调用原有差比和函数(输入L1/L2/L3/L4,输出-100~100)
    inductance_err = Inductance_Count_Err(adc[1], adc[2], adc[3], adc[4]);
    
    //===================== 2. 位置环PID(循迹外环) =====================//
    pos_out = PID_Calc(&pos_pid, 0, inductance_err);
    
    //===================== 3. 计算左右轮目标速度 =====================//
    // 位置环输出=速度差：左轮减速、右轮加速(向右偏); 左轮加速、右轮减速(向左偏)
    left_target  = BASE_SPEED - pos_out;
    right_target = BASE_SPEED + pos_out;
    
    //===================== 4. 读取编码器实际速度 =====================//
	speed_left = 	encoder_get_count(TIM0_ENCOEDER);                  	// 获取编码器计数
    speed_right = 	encoder_get_count(TIM3_ENCOEDER);              	
	
    encoder_clear_count(TIM0_ENCOEDER);                          // 清空编码器计数
    encoder_clear_count(TIM3_ENCOEDER);	

    real_left  = real_left  * 0.9 + speed_left  * 0.1;
    real_right = real_right * 0.9 + speed_right * 0.1;
    
    //===================== 5. 速度环PID(稳速内环) =====================//
    left_pwm  = (int16)PID_Calc(&left_spid,  left_target,  real_left);
    right_pwm = (int16)PID_Calc(&right_spid, right_target, real_right);
    
    //===================== 6. 输出到电机 =====================//
    Motor_SetSpeed(left_pwm, right_pwm);
}



