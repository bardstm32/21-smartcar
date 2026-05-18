#ifndef __ENCODER_H__
#define __ENCODER_H__

void encoder_init();
void Distance_Add();

extern volatile uint16 adc_inductance[];
extern volatile uint16 ind_10ms_flag ;

extern uint32 distance;
#endif