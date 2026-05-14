#include "zf_common_headfile.h"

#define PIT_CH (TIM1_PIT)
#define PIT_CH4 (TIM4_PIT)

volatile uint16 adc_inductance[5] = {0};
void pit_handler(void);
void pit_handler4(void);

#define ENCODER_DIR_LEFT (TIM0_ENCOEDER)		   // ������������Ӧʹ�õı������ӿ� ����ʹ��QTIMER1��ENCOEDER1
#define ENCODER_DIR_DIR_LEFT (IO_P35)			   // DIR ��Ӧ������
#define ENCODER_DIR_PULSE_LEFT (TIM0_ENCOEDER_P34) // PULSE ��Ӧ������

#define ENCODER_DIR_RIGHT (TIM3_ENCOEDER)			// ������������Ӧʹ�õı������ӿ� ����ʹ��QTIMER1��ENCOEDER1
#define ENCODER_DIR_DIR_RIGHT (IO_P53)				// DIR ��Ӧ������
#define ENCODER_DIR_PULSE_RIGHT (TIM3_ENCOEDER_P04) // PULSE ��Ӧ������


int encoder_L_R = 0; // ƽ���ٶ�
uint32 distance = 0; // �ۼƾ���

void Distance_Add()
{
	encoder_L_R = (unsigned int) ((My_abs(speed_left) + My_abs(speed_right)) / 2.0);
	distance += encoder_L_R;
}

uint8 imu_cnt = 0,t = 0;
void encoder_init()
{
	tim1_irq_handler = pit_handler;
	tim4_irq_handler = pit_handler4;
	pit_ms_init(PIT_CH, 1);
	pit_ms_init(PIT_CH4, 2);

	encoder_dir_init(ENCODER_DIR_LEFT, ENCODER_DIR_DIR_LEFT, ENCODER_DIR_PULSE_LEFT);
	encoder_dir_init(ENCODER_DIR_RIGHT, ENCODER_DIR_DIR_RIGHT, ENCODER_DIR_PULSE_RIGHT);
}

void pit_handler(void)
{
	imu660ra_get_acc(); // ��ȡ IMU660RA �ļ��ٶȲ�����ֵ
	imu660ra_get_gyro();
	imu_cnt++;
	if(imu_cnt >= 5)
	{
		imu_cnt = 0;
		gyro_proc();
		t++;
		if(t >= 2)
		{
			t = 0;
			Inductance_Read(adc_inductance);
			elemid = Inductance_Count_Err2(adc_inductance[1], adc_inductance[2], adc_inductance[3], adc_inductance[4]);
			Dir_Control();
		}
		Dir_Control_gyro();
		Calculate_Differential_Drive();
	}
}

void pit_handler4(void)
{
	Dual_Loop_Control();
}
