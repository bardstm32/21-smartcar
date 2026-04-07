#include "zf_common_headfile.h"

#define PIT_CH                          	(TIM1_PIT ) 


void pit_handler 	(void);

#define ENCODER_DIR_LEFT                 	(TIM0_ENCOEDER)                         // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_LEFT              	(IO_P35)            				 	// DIR 对应的引脚
#define ENCODER_DIR_PULSE_LEFT            	(TIM0_ENCOEDER_P34)            			// PULSE 对应的引脚

#define ENCODER_DIR_RIGHT                 	(TIM3_ENCOEDER)                         // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_RIGHT              	(IO_P53)            				 	// DIR 对应的引脚
#define ENCODER_DIR_PULSE_RIGHT            	(TIM3_ENCOEDER_P04)            			// PULSE 对应的引脚

uint16 cnt = 0;
	
void encoder_init()
{	
	tim1_irq_handler = pit_handler;
	pit_ms_init(PIT_CH,2);
	encoder_dir_init(ENCODER_DIR_LEFT, ENCODER_DIR_DIR_LEFT, ENCODER_DIR_PULSE_LEFT);
	encoder_dir_init(ENCODER_DIR_RIGHT, ENCODER_DIR_DIR_RIGHT, ENCODER_DIR_PULSE_RIGHT);

}

void pit_handler (void)
{	
    Dual_Loop_Control();

}

