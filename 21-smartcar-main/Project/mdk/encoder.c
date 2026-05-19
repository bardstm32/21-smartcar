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
int16 cnt=0;
/* 累计行程：取左右轮速绝对值的均值并加到 distance */
void Distance_Add(void)
{
	encoder_L_R = (IABS((int32)speed_left) + IABS((int32)speed_right)) >> 1;
	distance += encoder_L_R;
}


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
//	cnt++;
//	if(cnt>0&&cnt<400)
//	{
//	left_spid.target=150;
//	right_spid.target=150;
//	}
//	if(cnt>400&&cnt<800)
//	{
//	left_spid.target=200;
//	right_spid.target=200;
//	}
//	if(cnt>800)
//	{
//	cnt=0;
//	}
	
	Dual_Loop_Control();
}

/* TIM4 5ms 节拍：执行 IMU + 方向角速度环 + 差速*/
void pit_handler4(void)
{
		Inductance_Read(adc_inductance);
		elemid = Inductance_Count_Err2(adc_inductance[1], adc_inductance[2],adc_inductance[3], adc_inductance[4]);
		Dir_Control();
		Dir_Control_gyro();
		Calculate_Differential_Drive();
		gyro_proc();

}
