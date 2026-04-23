#include "zf_common_headfile.h"
// 定义3组PID(位置环+左右轮速度环)
PID_TypeDef Turn_PID, Gyro_PID;
PID_TypeDef left_spid;
PID_TypeDef right_spid;

#define ENCODER_DIR_LEFT                 	(TIM0_ENCOEDER)                         // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_LEFT              	(IO_P35)            				 	// DIR 对应的引脚
#define ENCODER_DIR_PULSE_LEFT            	(TIM0_ENCOEDER_P34)            			// PULSE 对应的引脚

#define ENCODER_DIR_RIGHT                 	(TIM3_ENCOEDER)                         // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_RIGHT              	(IO_P53)            				 	// DIR 对应的引脚
#define ENCODER_DIR_PULSE_RIGHT            	(TIM3_ENCOEDER_P04)            			// PULSE 对应的引脚
#define MAX_INC 150.0           // 2ms 周期下限制单次增量
volatile int32 speed_left = 0;
volatile int32 speed_right = 0;


float elemid = 0; // 目标赛道偏差
float eleOut_0 = 0; // 赛道偏差环输出值
float eleOut_1 = 0; // 偏航角速度环输出值
int Gyro_Z = 0; // 实际偏航角速度
float eleValue = 0;



/**
 * @brief  PID初始化(通用)
 */
void PID_Init(PID_TypeDef *pid, float kp, float kp2, float ki, float kd, float max_out, float max_i)
{
    pid->Kp = kp;
    pid->Kp2 = kp2;
	
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_i = max_i;

    pid->target =0;
    pid->measure = 0;
    pid->err = 0;
    pid->last_err = 0;
    pid->prev_err = 0; // ——> 【新增】初始化上上次误差
    pid->P = 0;
	pid->P2 = 0;
    pid->I = 0;
    pid->D = 0;
    pid->out = 0;
	pid->last_f_speed=0;
	

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
	pid->P2 = pid->Kp2 * pid->err*My_abs(pid->err);  

    // 微分项
    pid->D = pid->Kd * (pid->err - pid->last_err);  
    // 总输出+限幅
	pid->last_f_speed=pid->err - pid->last_err;
	
    pid->out = pid->P + pid->P2+ pid->D;   
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
	if(++t>=2)
	{
		t = 0;
		// 外环（赛道偏差环）,具体正负号根据实际情况确定
		eleOut_0 = PID_Calc(&Turn_PID, 0, elemid);
		// 限幅保护，确保输出结果在 -10000 ~ 10000 范围内
		eleOut_0 = range_protect_float(eleOut_0, -10000.0f, 10000.0f);
	}
}

void Calculate_Differential_Drive() // 差速计算
{
	float k = 0; // 差速比例系数
	k = eleOut_0 * 0.0001f; // 缩放成 -1 ~ 1
	k = range_protect_float(k, -0.65, 0.65); // 限制到 -0.65 ~ 0.65，实现差速限幅
	// 计算左右轮目标速度
	if(k >= 0) // 左转
	{
		left_spid.target = BASE_SPEED *(1+k*0.25);
		right_spid.target  = BASE_SPEED *(1-k) ; // 加少减多
	}
	if(k < 0) // 右转
	{
		k *= -1;
		left_spid.target = BASE_SPEED * (1 -k); // 加少减多
		right_spid.target = BASE_SPEED * (1 + k*0.25);
	}
}

// 增量式PID计算函数
// 公式: ΔU = Kp*(e(k) - e(k-1)) + Ki*e(k)
void IncPID_Calc(PID_TypeDef *pid, int16 current_speed)
{
    // 1. 计算当前误差
    pid->err = pid->target - current_speed;
	pid->I=pid->Ki * pid->err;	

    // 2. 计算输出增量 (ΔU)
   pid->inc_out = (pid->Kp * (pid->err - pid->last_err) +pid->I);

    // 3. 累加增量得到最终实际输出
    pid->out += pid->inc_out;
    // 4. 输出限幅 (防止PWM跑飞)
    if (pid->out > pid->max_out)
        pid->out = pid->max_out;
    if (pid->out < -pid->max_out)
        pid->out = -pid->max_out;
	pid->last_err=pid->err;
}

/**
 * @brief  双闭环循迹控制(核心函数)
 * 流程：电感差比和 → 位置环 → 左右轮目标速度 → 速度环 → 电机PWM
 */
void Dual_Loop_Control(void)
{
    

    speed_left = encoder_get_count(TIM0_ENCOEDER); // 获取编码器计数
    speed_right = encoder_get_count(TIM3_ENCOEDER);              	
	
    encoder_clear_count(TIM0_ENCOEDER);                          // 清空编码器计数
    encoder_clear_count(TIM3_ENCOEDER);

    IncPID_Calc(&left_spid,speed_left);
    IncPID_Calc(&right_spid,speed_right);

    //    //===================== 6. 输出到电机 =====================//
    Motor_SetSpeed((int16)left_spid.out, (int16)right_spid.out);
}





