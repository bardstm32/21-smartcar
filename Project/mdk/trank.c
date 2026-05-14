#include "zf_common_headfile.h"
// ����3��PID(λ�û�+�������ٶȻ�)
PID_TypeDef Turn_PID, Gyro_PID;
PID_TypeDef left_spid;
PID_TypeDef right_spid;

#define ENCODER_DIR_LEFT                 	(TIM0_ENCOEDER)                         // ������������Ӧʹ�õı������ӿ� ����ʹ��QTIMER1��ENCOEDER1
#define ENCODER_DIR_DIR_LEFT              	(IO_P35)            				 	// DIR ��Ӧ������
#define ENCODER_DIR_PULSE_LEFT            	(TIM0_ENCOEDER_P34)            			// PULSE ��Ӧ������

#define ENCODER_DIR_RIGHT                 	(TIM3_ENCOEDER)                         // ������������Ӧʹ�õı������ӿ� ����ʹ��QTIMER1��ENCOEDER1
#define ENCODER_DIR_DIR_RIGHT              	(IO_P53)            				 	// DIR ��Ӧ������
#define ENCODER_DIR_PULSE_RIGHT            	(TIM3_ENCOEDER_P04)            			// PULSE ��Ӧ������
#define MAX_INC 150.0           // 2ms ���������Ƶ�������
volatile int16 speed_left = 0;
volatile int16 speed_right = 0;

float elemid = 0; // Ŀ������ƫ��
float eleOut_0 = 0; // ����ƫ����ֵ
float eleOut_1 = 0; // ƫ�����ٶȻ����ֵ
volatile float Turn_target = 0;
/**
 * @brief  PID��ʼ��(ͨ��)
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
    pid->prev_err = 0; // ����> ����������ʼ�����ϴ����
    pid->P = 0;
	pid->P2 = 0;
    pid->I = 0;
    pid->D = 0;
    pid->out = 0;

}

/**
 * @brief  ͨ��λ��ʽPID����
 */
float PID_Calc(PID_TypeDef *pid, float target, float measure)
{
    pid->target = target;
    pid->measure = measure;
    
    // �������
    pid->err = pid->target - pid->measure;
    
    // ������
    pid->P = pid->Kp * pid->err;  
	pid->P2 = pid->Kp2 * pid->err*My_abs(pid->err);  

    // ΢����
    pid->D = pid->Kd * (pid->err - pid->last_err);  
    // �����+�޷�
	
    pid->out = pid->P + pid->P2+ pid->D;   
    // �������
    pid->last_err = pid->err;
    
    return pid->out;
}

/**
 * @brief ���򻷿��ƺ���
 */
void Dir_Control()
{
        // ����״̬�����������������������Ҫ������ʵ�ʵ����һ����������
        if (TrackState == NORMAL || TrackState == ROUNDIN || TrackState == ROUNDOUT 
			|| TrackState ==RIGHT_ROUNDAPPROCH || TrackState ==LEFT_ROUNDAPPROCH)
        {
            // ����ѭ��
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
        // �޷�������ȷ���������� -10000 ~ 10000 ��Χ��
		eleOut_0 = range_protect_float(eleOut_0, -10000.0f, 10000.0f);
}

void Dir_Control_gyro()
{
	eleOut_1 = PID_Calc(&Gyro_PID, eleOut_0, imu660ra_gyro_z);
//	eleOut_1 = range_protect_float(eleOut_1, -10000.0f, 10000.0f);
	
}


void Calculate_Differential_Drive() // ���ټ���
{
	static float k = 0; // ���ٱ���ϵ��
    k = eleOut_1 * 0.0001f;                  // ���ų� -1 ~ 1
    k = range_protect_float(k, -0.75, 0.75); // ���Ƶ� -0.65 ~ 0.65��ʵ�ֲ����޷�
	// ����������Ŀ���ٶ�
	if(k >= 0) // ��ת
	{
		left_spid.target = BASE_SPEED *(1+k*0.2);
		right_spid.target  = BASE_SPEED *(1-k) ; // ���ټ���
	}
	if(k < 0) // ��ת
	{
		k *= -1;
		left_spid.target = BASE_SPEED * (1 - k); // ���ټ���
		right_spid.target = BASE_SPEED * (1+k*0.2);
	}
}

// ����ʽPID���㺯��
// ��ʽ: ��U = Kp*(e(k) - e(k-1)) + Ki*e(k)
void IncPID_Calc(PID_TypeDef *pid, int16 current_speed)
{
    // 1. ���㵱ǰ���
    pid->err = pid->target - current_speed;
	pid->I=pid->Ki * pid->err;	

    // 2. ����������� (��U)
   pid->inc_out = (pid->Kp * (pid->err - pid->last_err) +pid->I);

    // 3. �ۼ������õ�����ʵ�����
    pid->out += pid->inc_out;
    // 4. ����޷� (��ֹPWM�ܷ�)
    if (pid->out > pid->max_out)
        pid->out = pid->max_out;
    if (pid->out < -pid->max_out)
        pid->out = -pid->max_out;
	pid->last_err=pid->err;
}

/**
 * @brief  ˫�ջ�ѭ������(���ĺ���)
 * ���̣���в�Ⱥ� �� λ�û� �� ������Ŀ���ٶ� �� �ٶȻ� �� ���PWM
 */
void Dual_Loop_Control(void)
{
    speed_left = -encoder_get_count(TIM0_ENCOEDER); // ��ȡ����������
    speed_right = encoder_get_count(TIM3_ENCOEDER);              	
	
    encoder_clear_count(TIM0_ENCOEDER);                          // ��ձ���������
    encoder_clear_count(TIM3_ENCOEDER);

    IncPID_Calc(&left_spid,speed_left);
    IncPID_Calc(&right_spid,speed_right);

    Motor_SetSpeed((int16)left_spid.out, (int16)right_spid.out);
}





