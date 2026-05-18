#include "zf_common_headfile.h"

PID_TypeDef Turn_PID, Gyro_PID;
PID_TypeDef left_spid;
PID_TypeDef right_spid;

#define ENCODER_DIR_LEFT                 	(TIM0_ENCOEDER)
#define ENCODER_DIR_DIR_LEFT              	(IO_P35)
#define ENCODER_DIR_PULSE_LEFT            	(TIM0_ENCOEDER_P34)

#define ENCODER_DIR_RIGHT                 	(TIM3_ENCOEDER)
#define ENCODER_DIR_DIR_RIGHT              	(IO_P53)
#define ENCODER_DIR_PULSE_RIGHT            	(TIM3_ENCOEDER_P04)
#define MAX_INC 150.0
#define GYRO_TARGET_MAX 10000.0f

volatile int16 speed_left = 0;
volatile int16 speed_right = 0;

float elemid = 0;
float eleOut_0 = 0;
float eleOut_1 = 0;
volatile float Turn_target = 0;
volatile uint8 control_ready = 0;        /* 控制就绪门闸：0 = 速度环禁出，1 = 已有有效目标速度可执行 */

/* 通用 PID 初始化：清零状态量并写入增益与限幅 */
void PID_Init(PID_TypeDef *pid, float kp, float kp2, float ki, float kd, float max_out, float max_i)
{
    pid->Kp = kp;
    pid->Kp2 = kp2;

    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_i = max_i;

    pid->target = 0;
    pid->measure = 0;
    pid->err = 0;
    pid->last_err = 0;
    pid->prev_err = 0;
    pid->P = 0;
    pid->P2 = 0;
    pid->I = 0;
    pid->D = 0;
    pid->out = 0;
}

/* 位置式 PID：含 Kp2 非线性比例项与 Kd 一阶差分微分 */
float PID_Calc(PID_TypeDef *pid, float target, float measure)
{
    pid->target = target;
    pid->measure = measure;

    pid->err = pid->target - pid->measure;

    pid->P  = pid->Kp  * pid->err;
    pid->P2 = pid->Kp2 * pid->err * My_abs(pid->err);

    pid->D  = pid->Kd  * (pid->err - pid->last_err);

    pid->out = pid->P + pid->P2 + pid->D;

    pid->last_err = pid->err;
    return pid->out;
}

/* 方向位置环：常态用 Turn_PID 跟踪 elemid，环内强制差速 */
void Dir_Control()
{
    if (TrackState == NORMAL || TrackState == ROUNDIN || TrackState == RIGHT_ROUNDAPPROCH || TrackState == LEFT_ROUNDAPPROCH)
    {
        eleOut_0 = PID_Calc(&Turn_PID, 0, elemid);
    }
    else if (TrackState == RIGHT_ROUND)
    {
        eleOut_0 = 6000;
    }
    else if (TrackState == LEFT_ROUND)
    {
        eleOut_0 = -6000;
    }
    else if (TrackState == ROUNDOUT)
    {
		eleOut_0 = PID_Calc(&Turn_PID, 0, elemid);
//        if(ring_dir == 1){eleOut_0 = -2000;}
//		if(ring_dir == 2){eleOut_0 = 2000;}
    }
    eleOut_0 = range_protect_float(eleOut_0, -GYRO_TARGET_MAX, GYRO_TARGET_MAX);
}

/* 方向角速度环：反馈使用 (原始 LSB - 零漂)，与 Kp/Kd 整定量级匹配 */
void Dir_Control_gyro()
{
    eleOut_1 = PID_Calc(&Gyro_PID, eleOut_0, imu660ra_gyro_z - Gyro_Offset.Zdata);
	eleOut_1 = eleOut_1 + eleOut_0;
	eleOut_1 = range_protect_float(eleOut_1, -9000.0f, 9000.0f);
}

/* 把 eleOut_1 映射成左右轮目标速度（外侧轻加速、内侧减速） */
void Calculate_Differential_Drive() // 差速计算
{
	static float k = 0; // 差速比例系数
    k = eleOut_1 * 0.0001f;                  // 缩放成 -1 ~ 1
    k = range_protect_float(k, -0.70, 0.70); // 限制到 -0.65 ~ 0.65，实现差速限幅
	// 计算左右轮目标速度
	if(k >= 0) // 右转
	{
		left_spid.target = BASE_SPEED *(1+k*0.4);
		right_spid.target  = BASE_SPEED *(1-k) ; // 加少减多
	}
	if(k < 0) // 左转
	{
		k *= -1;
		left_spid.target = BASE_SPEED * (1 - k); // 加少减多
		right_spid.target = BASE_SPEED * (1+k*0.4);
	}
}


/* 增量式速度 PID（带条件积分 anti-windup）                                 */
/*   普通增量式：dU = Kp*(e - e_prev) + Ki*e，每周期累加到 out               */
/*   问题：out 钉在饱和值时若误差仍同向，Ki 持续累加；阻力消失后无法立刻退饱和 */
/*   做法：当 out 已饱和且误差仍在推它继续饱和方向时，本周期把积分项置 0        */
void IncPID_Calc(PID_TypeDef *pid, int16 current_speed)
{
    pid->err = pid->target - current_speed;                                /* 本周期速度误差 e(k) = target - 实际 */

    if ((pid->out >=  pid->max_out && pid->err > 0) ||                     /* 输出已正向饱和且 err 仍 >0（继续推饱和） */
        (pid->out <= -pid->max_out && pid->err < 0))                       /* 或输出已负向饱和且 err 仍 <0 */
        pid->I = 0;                                                        /*   则条件积分：本周期不加积分项，防 windup */
    else                                                                   /* 否则正常计算积分增量 */
        pid->I = pid->Ki * pid->err;                                       /*   单步积分项 = Ki * e(k) */
	pid->P=pid->Kp * (pid->err - pid->last_err) ;
    pid->inc_out = pid->P + pid->I;          /* 单步增量 dU = Kp*de + Ki*e */
    pid->out += pid->inc_out;                                              /* 累加到当前输出 */

    if (pid->out >  pid->max_out) pid->out =  pid->max_out;                /* 正向硬限幅（防 PWM 溢出） */
    if (pid->out < -pid->max_out) pid->out = -pid->max_out;                /* 反向硬限幅 */

    pid->last_err = pid->err;                                              /* 保存本次误差，供下次差分使用 */
}

/* 速度闭环（TIM4 / 2ms 节拍）                                            */
/* 读左右编码器、清零，跑两路增量式 PID，写 PWM                          */
void Dual_Loop_Control(void)
{
    speed_left  = -encoder_get_count(TIM0_ENCOEDER);                       /* 左轮脉冲（按物理安装方向取反） */
    speed_right =  encoder_get_count(TIM3_ENCOEDER);                       /* 右轮脉冲 */

    encoder_clear_count(TIM0_ENCOEDER);                                    /* 清零，准备下一周期累计 */
    encoder_clear_count(TIM3_ENCOEDER);

    IncPID_Calc(&left_spid,  speed_left);                                  /* 左轮速度环 */
    IncPID_Calc(&right_spid, speed_right);                                 /* 右轮速度环 */

    Motor_SetSpeed((int16)left_spid.out, (int16)right_spid.out);           /* 写入 PWM 输出 */
}
