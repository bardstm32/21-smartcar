#include "zf_common_headfile.h"

#define PIT_CH (TIM1_PIT)
#define PIT_CH4 (TIM4_PIT)

volatile uint16 adc_inductance[5] = {0};
void pit_handler(void);
void pit_handler4(void);

#define ENCODER_DIR_LEFT (TIM0_ENCOEDER)		   // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_LEFT (IO_P35)			   // DIR 对应的引脚
#define ENCODER_DIR_PULSE_LEFT (TIM0_ENCOEDER_P34) // PULSE 对应的引脚

#define ENCODER_DIR_RIGHT (TIM3_ENCOEDER)			// 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_RIGHT (IO_P53)				// DIR 对应的引脚
#define ENCODER_DIR_PULSE_RIGHT (TIM3_ENCOEDER_P04) // PULSE 对应的引脚

uint16 imu_cnt = 0;

void encoder_init()
{
	tim1_irq_handler = pit_handler;
	tim4_irq_handler = pit_handler4;
	pit_ms_init(PIT_CH, 1);
//	pit_ms_init(PIT_CH4, 2);

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
//		Inductance_Read(adc_inductance);
//		elemid = Inductance_Count_Err(adc_inductance[1], adc_inductance[2], adc_inductance[3], adc_inductance[4]);
//		Dir_Control();
//		Calculate_Differential_Drive();
		imu_cnt = 0;
		gyro_proc();
	}
	
}

void pit_handler4(void)
{
	Dual_Loop_Control();
}
