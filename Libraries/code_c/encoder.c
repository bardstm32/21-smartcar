#include "zf_common_headfile.h"

#define PIT_CH                          	(TIM1_PIT ) 

#define ENCODER_DIR_LEFT                 	(TIM0_ENCOEDER)                         // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_LEFT              	(IO_P35)            				 	// DIR 对应的引脚
#define ENCODER_DIR_PULSE_LEFT            	(TIM0_ENCOEDER_P34)            			// PULSE 对应的引脚

#define ENCODER_DIR_RIGHT                 	(TIM3_ENCOEDER)                         // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_RIGHT              	(IO_P53)            				 	// DIR 对应的引脚
#define ENCODER_DIR_PULSE_RIGHT            	(TIM3_ENCOEDER_P04)            			// PULSE 对应的引脚

void pit_handler (void);


int16 speed_left = 0;
int16 speed_right = 0;
int16 last_speed_left=0;
int16 last_speed_right=0;
int16 real_left=0;
int16 real_right=0;
uint8 imu_cnt = 0;

void encoder_init()
{
	tim1_irq_handler = pit_handler;
	pit_ms_init(PIT_CH,1);
	encoder_dir_init(ENCODER_DIR_LEFT, ENCODER_DIR_DIR_LEFT, ENCODER_DIR_PULSE_LEFT);
	encoder_dir_init(ENCODER_DIR_RIGHT, ENCODER_DIR_DIR_RIGHT, ENCODER_DIR_PULSE_RIGHT);

}

void pit_handler(void)
{
	imu660ra_get_acc(); // 获取 IMU660RA 的加速度测量数值
	imu660ra_get_gyro();
	imu_cnt++; 
	if (imu_cnt >= 5)
	{
		imu_cnt = 0;
		gyro_proc();
	}
	
    speed_left = encoder_get_count(ENCODER_DIR_LEFT);                  	// 获取编码器计数
    speed_right = encoder_get_count(ENCODER_DIR_RIGHT);              	
	
    encoder_clear_count(ENCODER_DIR_LEFT);                          // 清空编码器计数
    encoder_clear_count(ENCODER_DIR_RIGHT);	

	real_left  = last_speed_left * 0.9 + speed_left  * 0.1;
	last_speed_left=speed_left;
		
	real_right = last_speed_right * 0.9 + speed_right * 0.1;
	last_speed_right=speed_right;
	//PID_Inc_Calc(&Speed_PID_L,20,real_left);
	//	Dual_Loop_Control();
}
