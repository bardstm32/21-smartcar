#include "zf_common_headfile.h"

#define PIT_CH (TIM1_PIT)
#define PIT_CH4 (TIM4_PIT)

volatile uint16 adc_inductance[5] = {0};
void pit_handler(void);
void pit_handler4(void);

#define ENCODER_DIR_LEFT (TIM0_ENCOEDER)
#define ENCODER_DIR_DIR_LEFT (IO_P35)
#define ENCODER_DIR_PULSE_LEFT (TIM0_ENCOEDER_P34)

#define ENCODER_DIR_RIGHT (TIM3_ENCOEDER)
#define ENCODER_DIR_DIR_RIGHT (IO_P53)
#define ENCODER_DIR_PULSE_RIGHT (TIM3_ENCOEDER_P04)

int encoder_L_R = 0;
uint32 distance = 0;

/* 累计行程：取左右轮速绝对值的均值并加到 distance */
void Distance_Add(void)
{
	encoder_L_R = (IABS((int32)speed_left) + IABS((int32)speed_right)) >> 1;
	distance += encoder_L_R;
}

volatile uint16 ind_10ms_flag = 0;
static uint16 imu_cnt = 0;
static uint8 ind_cnt = 0;

/* 注册 TIM1 / TIM4 中断，初始化两路编码器 */
void encoder_init()
{
	tim1_irq_handler = pit_handler;
	tim4_irq_handler = pit_handler4;
	interrupt_set_priority(TIMER1_IRQn,3);
	pit_ms_init(PIT_CH, 2);
	pit_ms_init(PIT_CH4, 5);

	encoder_dir_init(ENCODER_DIR_LEFT, ENCODER_DIR_DIR_LEFT, ENCODER_DIR_PULSE_LEFT);
	encoder_dir_init(ENCODER_DIR_RIGHT, ENCODER_DIR_DIR_RIGHT, ENCODER_DIR_PULSE_RIGHT);
}

/* TIM1 2ms 节拍：执行速度闭环 */
void pit_handler(void)
{
//	imu_cnt++;
//	if(imu_cnt>300&&imu_cnt<550)
//	{
//	left_spid.target=100;
//	right_spid.target=100;
//	}
//		
//	if(imu_cnt>550&&imu_cnt<800)
//		{
//	left_spid.target=180;
//	right_spid.target=180;
//		}
//		
//	if(imu_cnt>800)
//		{
//		imu_cnt=0;
//		}
	Dual_Loop_Control();
}

/* TIM4 5ms 节拍：执行 IMU + 方向角速度环 + 差速*/
void pit_handler4(void)
{

//	if (++ind_cnt >= 2)
//	{
//		ind_cnt = 0;
	Inductance_Read(adc_inductance);
	elemid = Inductance_Count_Err2(adc_inductance[1], adc_inductance[2],adc_inductance[3], adc_inductance[4]);
	Dir_Control();
	gyro_proc();
	Dir_Control_gyro();
	Calculate_Differential_Drive();

//	}

}
